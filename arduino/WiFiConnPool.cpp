/* WiFiConnPool.cpp - see WiFiConnPool.h. MIT, (c) 2026 Nicholas Newdigate. */
#include "WiFiConnPool.h"
#include "Arduino.h"
#include "lwip/pbuf.h"

static WiFiConn s_conns[WIFI_MAX_CONNS];
static uint32_t s_evictions = 0;

namespace WiFiPool {

WiFiConn *slot(uint8_t i) { return (i < WIFI_MAX_CONNS) ? &s_conns[i] : nullptr; }
uint32_t evictions() { return s_evictions; }

static void freeRx(WiFiConn *c) {
    if (c->rxHead) { pbuf_free(c->rxHead); c->rxHead = nullptr; }
    c->rxOff = 0;
}

static void toFree(WiFiConn *c) {
    freeRx(c);
    c->state = WiFiConn::FREE;
    c->pcb = nullptr;
    c->refs = 0;
    c->claimed = false;
    c->serverPort = 0;
    c->connectDone = c->connectOk = false;
}

WiFiConn *alloc() {
    for (uint8_t i = 0; i < WIFI_MAX_CONNS; i++)
        if (s_conns[i].state == WiFiConn::FREE) return &s_conns[i];
    return nullptr;
}

WiFiConn *allocEvicting() {
    WiFiConn *c = alloc();
    if (c) return c;
    WiFiConn *victim = nullptr;
    for (uint8_t i = 0; i < WIFI_MAX_CONNS; i++) {
        WiFiConn *s = &s_conns[i];
        if (s->claimed || s->serverPort == 0) continue;   // only unclaimed accepts
        if (!victim || (int32_t)(victim->lastActivityMs - s->lastActivityMs) > 0)
            victim = s;
    }
    if (!victim) return nullptr;
    s_evictions++;
    (void)closeConn(victim);     // clears callbacks first; not in a callback here
    toFree(victim);
    return victim;
}

void addRef(WiFiConn *c) { if (c) c->refs++; }

void release(WiFiConn *c) {
    if (!c || c->refs == 0) return;
    if (--c->refs == 0) {
        // Last handle gone.  A live conn the sketch abandoned gets closed --
        // Arduino clients don't linger after their last handle dies.
        if (c->pcb) (void)closeConn(c);
        toFree(c);
    }
}

err_t closeConn(WiFiConn *c) {
    struct tcp_pcb *pcb = c->pcb;
    c->pcb = nullptr;
    if (!pcb) return ERR_OK;
    tcp_arg(pcb, nullptr);
    tcp_recv(pcb, nullptr);
    tcp_sent(pcb, nullptr);
    tcp_err(pcb, nullptr);
    tcp_poll(pcb, nullptr, 0);
    if (tcp_close(pcb) != ERR_OK) {
        tcp_abort(pcb);
        return ERR_ABRT;
    }
    return ERR_OK;
}

void abortAll() {
    for (uint8_t i = 0; i < WIFI_MAX_CONNS; i++) {
        WiFiConn *c = &s_conns[i];
        if (c->state == WiFiConn::FREE || !c->pcb) continue;
        struct tcp_pcb *pcb = c->pcb;
        c->pcb = nullptr;
        tcp_arg(pcb, nullptr); tcp_recv(pcb, nullptr); tcp_sent(pcb, nullptr);
        tcp_err(pcb, nullptr); tcp_poll(pcb, nullptr, 0);
        tcp_abort(pcb);                     // link is dead; no FIN possible
        c->state = WiFiConn::PEER_CLOSED;   // rx chain stays readable
        c->connectDone = true; c->connectOk = false;
        if (c->refs == 0) toFree(c);
    }
}

int availableBytes(const WiFiConn *c) {
    if (!c || !c->rxHead) return 0;
    return (int)c->rxHead->tot_len - (int)c->rxOff;
}

int peekByte(const WiFiConn *c) {
    if (availableBytes(c) <= 0) return -1;
    uint8_t b;
    if (pbuf_copy_partial(c->rxHead, &b, 1, c->rxOff) != 1) return -1;
    return b;
}

int consume(WiFiConn *c, uint8_t *buf, int len) {
    int avail = availableBytes(c);
    if (avail <= 0 || len <= 0) return 0;
    uint16_t n = (uint16_t)((len < avail) ? len : avail);
    uint16_t got = pbuf_copy_partial(c->rxHead, buf, n, c->rxOff);
    c->rxOff += got;
    // Free fully-consumed leading pbufs (ref-next-then-free-head; a plain
    // pbuf_free on the head would free the whole chain).
    while (c->rxHead && c->rxOff >= c->rxHead->len) {
        struct pbuf *h = c->rxHead;
        c->rxOff -= h->len;
        c->rxHead = h->next;
        if (c->rxHead) pbuf_ref(c->rxHead);
        pbuf_free(h);
    }
    if (c->pcb) tcp_recved(c->pcb, got);   // opens the window only as consumed
    c->lastActivityMs = millis();
    return got;
}

// --- lwip callbacks (arg is ALWAYS the slot) --------------------------------
static err_t connRecv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t) {
    WiFiConn *c = (WiFiConn *)arg;
    if (p == nullptr) {                    // peer FIN; keep pcb for our close
        c->state = WiFiConn::PEER_CLOSED;
        return ERR_OK;
    }
    if (c->rxHead) pbuf_cat(c->rxHead, p); else { c->rxHead = p; c->rxOff = 0; }
    c->lastActivityMs = millis();
    (void)pcb;
    return ERR_OK;                         // tcp_recved deferred to consume()
}

static void connErr(void *arg, err_t) {    // pcb ALREADY FREED by lwip:
    WiFiConn *c = (WiFiConn *)arg;         // reset state only, never tcp_*
    c->pcb = nullptr;
    c->state = WiFiConn::PEER_CLOSED;
    c->connectDone = true; c->connectOk = false;
    if (c->refs == 0) toFree(c);
}

static err_t connSent(void *arg, struct tcp_pcb *, u16_t) {
    ((WiFiConn *)arg)->lastActivityMs = millis();
    return ERR_OK;
}

// Stall safety valve: an accepted conn the sketch never picked up, idle past
// 30 s, is aborted (2 ticks/s * 60 = tcp_poll interval 60 => ~30 s).  Claimed
// conns get no poll -- an idle-but-held session is the sketch's business.
static err_t connPoll(void *arg, struct tcp_pcb *pcb) {
    WiFiConn *c = (WiFiConn *)arg;
    if (!c->claimed && millis() - c->lastActivityMs > 30000) {
        c->pcb = nullptr;
        tcp_arg(pcb, nullptr); tcp_recv(pcb, nullptr); tcp_sent(pcb, nullptr);
        tcp_err(pcb, nullptr); tcp_poll(pcb, nullptr, 0);
        tcp_abort(pcb);
        toFree(c);
        return ERR_ABRT;                   // in-callback abort contract
    }
    return ERR_OK;
}

void installCallbacks(WiFiConn *c, struct tcp_pcb *pcb) {
    c->pcb = pcb;
    c->lastActivityMs = millis();
    tcp_arg(pcb, c);
    tcp_recv(pcb, connRecv);
    tcp_err(pcb, connErr);
    tcp_sent(pcb, connSent);
    tcp_poll(pcb, connPoll, 60);
}

} // namespace WiFiPool
