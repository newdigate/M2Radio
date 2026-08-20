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
        if (s_conns[i].state == WiFiConn::FREE) {
            // RESERVE it here.  Returning a slot still marked FREE and relying
            // on the caller to claim it before the next alloc() is exactly the
            // review-enforced kind of contract this pool exists to replace --
            // and it fails silently and late.  The caller overwrites state
            // freely afterwards (accept -> ESTABLISHED).
            s_conns[i].state = WiFiConn::CONNECTING;
            return &s_conns[i];
        }
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
    toFree(victim);              // ... which leaves it FREE, so re-reserve it
    victim->state = WiFiConn::CONNECTING;
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
        // No FIN is possible with the link down, so this is an abort, not a
        // close.  tcp_abort DOES attempt an RST -- but linkLost() marked the
        // netif link down two lines earlier, and ip4_route skips a link-down
        // netif, so tcp_route returns NULL and the segment never reaches the
        // driver.  Nothing is transmitted; nothing fails loudly either.
        tcp_abort(pcb);
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
    // Chain-length backpressure.  REFUSE, do not drop: returning a non-OK,
    // non-ABRT error WITHOUT freeing p and WITHOUT cat'ing it makes lwip keep
    // the pbuf as pcb->refused_data (tcp_in.c: `if (err != ERR_OK) {
    // pcb->refused_data = recv_data; }`) and re-offer it from tcp_fasttmr
    // every 250 ms until we take it -- no peer retransmit needed.  Dropping it
    // with ERR_OK instead would leak WINDOW, not just data: tcp_receive has
    // already debited rcv_wnd for those bytes and only tcp_recved re-credits
    // it, so the window would shrink by that segment permanently.
    uint8_t depth = 0;
    for (struct pbuf *q = c->rxHead; q != nullptr; q = q->next) depth++;
    if (depth >= WIFI_RX_MAX_PBUFS) return ERR_MEM;
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

// Stall safety valve: a conn nothing holds and the sketch never picked up,
// idle past 30 s, is aborted.  The poll IS installed on every conn (see
// installCallbacks) and simply no-ops for the ones this test excludes -- an
// idle-but-held session is the sketch's business.
//
// Timing, exactly: TCP_TMR_INTERVAL is 250 ms and TCP_SLOW_INTERVAL is 2x that,
// neither overridden in lwipopts.h, so tcp_poll's interval is in 500 ms ticks
// -- 20 => a poll every 10 s (spec: "~10 s cadence").  With the body requiring
// >30 s of idleness the abort therefore lands somewhere in 30-40 s, NOT at a
// 30 s deadline.  That looseness is fine for a safety valve; being wrong about
// which of the two numbers sets it is not.
//
// The refs test is belt and braces beside `claimed`: they are independent
// fields and toFree() zeroes refs, so gating on `claimed` alone would let a
// stalled connect (alloc -> installCallbacks -> addRef, claimed set later)
// free the slot under a live WiFiClient -- the dangling handle this pool
// exists to make unrepresentable, re-entering through the back door.
static err_t connPoll(void *arg, struct tcp_pcb *pcb) {
    WiFiConn *c = (WiFiConn *)arg;
    if (!c->claimed && c->refs == 0 && millis() - c->lastActivityMs > 30000) {
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
    tcp_poll(pcb, connPoll, 20);     // 20 ticks x 500 ms = 10 s (see connPoll)
}

} // namespace WiFiPool
