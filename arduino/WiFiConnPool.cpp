/* WiFiConnPool.cpp - see WiFiConnPool.h. MIT, (c) 2026 Nicholas Newdigate. */
#include "WiFiConnPool.h"
#include "Arduino.h"
#include "lwip/pbuf.h"

static WiFiConn s_conns[WIFI_MAX_CONNS];
static uint32_t s_evictions = 0;
static uint32_t s_stallAborts = 0;

namespace WiFiPool {

WiFiConn *slot(uint8_t i) { return (i < WIFI_MAX_CONNS) ? &s_conns[i] : nullptr; }
uint32_t evictions() { return s_evictions; }
uint32_t stallAborts() { return s_stallAborts; }

// Every callback lwip can reach this slot through, cleared in one place --
// tcp_connected INCLUDED.  There is no tcp_connected() setter (the callback is
// an argument to tcp_connect), so that one is assigned directly, exactly as
// lwip's own tcp_connect does.  The pool never installs it, but Task 7 will,
// and "clear EVERY callback before you close" has to be true rather than
// nearly true.
static void detachCallbacks(struct tcp_pcb *pcb) {
    tcp_arg(pcb, nullptr);
    tcp_recv(pcb, nullptr);
    tcp_sent(pcb, nullptr);
    tcp_err(pcb, nullptr);
    tcp_poll(pcb, nullptr, 0);
#if LWIP_CALLBACK_API
    pcb->connected = nullptr;
#endif
}

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
    // ABORT, not a graceful close.  We are here BECAUSE the pool is full, and
    // tcp_close on an ESTABLISHED conn leaves the pcb in FIN_WAIT holding one
    // of MEMP_NUM_TCP_PCB=5 -- spending a pcb precisely when pcbs are what ran
    // out.  The victim is unclaimed by definition (the sketch has never seen
    // it), so there is nobody to owe a graceful close to.
    if (victim->pcb) {
        struct tcp_pcb *pcb = victim->pcb;
        victim->pcb = nullptr;
        detachCallbacks(pcb);
        tcp_abort(pcb);
    }
    toFree(victim);              // ... which leaves it FREE, so re-reserve it
    victim->state = WiFiConn::CONNECTING;
    return victim;
}

void addRef(WiFiConn *c) { if (c) c->refs++; }

void release(WiFiConn *c) {
    if (!c) return;
    if (c->refs == 0) {
        // A RESERVATION nobody ever took a handle on: alloc() succeeded and
        // then something before addRef() failed.  Nothing else in this file can
        // collect that slot -- abortAll() skips a null pcb, both valves need
        // callbacks installed, the evictor only considers serverPort != 0 --
        // so without this it is stranded until reboot, and four such failures
        // kill the pool.  Safe because a null pcb means lwip holds no pointer
        // to this slot; a slot that still HAS a pcb is a live connection and
        // must be closed properly rather than silently dropped, so it is left
        // alone here.
        if (c->pcb == nullptr && c->state != WiFiConn::FREE) toFree(c);
        return;
    }
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
    detachCallbacks(pcb);
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
        detachCallbacks(pcb);
        // No FIN is possible with the link down, so this is an abort, not a
        // close.  tcp_abort DOES attempt an RST -- but every caller
        // (WiFiClass::linkDownAndAbort) marks the netif link down first, and
        // ip4_route skips a link-down netif, so tcp_route returns NULL and the
        // segment never reaches the driver.  Nothing is transmitted, and
        // nothing fails loudly either.  Deliberate: the alternative is handing
        // a frame to a driver whose link state we are mid-way through tearing
        // down, to tell a peer something its own timeout will tell it anyway.
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
static err_t connRecv(void *arg, struct tcp_pcb *, struct pbuf *p, err_t) {
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
    //
    // The empty-chain exemption is what makes permanent refusal impossible: if
    // a delivery bigger than the whole cap could be refused into an empty
    // chain, nothing would ever drain and nothing would ever be accepted.
    // pbuf_cat'ing it instead costs nothing real -- p is only ever a long chain
    // because tcp_in.c joined queued out-of-order segments, which are already
    // allocated whether we take them or not.
    uint16_t staged = 0;
    for (struct pbuf *q = c->rxHead; q != nullptr; q = q->next) staged++;
    if (staged != 0) {          // an EMPTY chain ALWAYS accepts -- see below
        uint16_t incoming = 0;  // p is not necessarily one pbuf: tcp_in.c
        for (struct pbuf *q = p; q != nullptr; q = q->next) incoming++;
        if (staged + incoming > WIFI_RX_MAX_PBUFS) return ERR_MEM;
    }
    if (c->rxHead) pbuf_cat(c->rxHead, p); else { c->rxHead = p; c->rxOff = 0; }
    c->lastActivityMs = millis();
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
        detachCallbacks(pcb);
        tcp_abort(pcb);
        toFree(c);
        s_stallAborts++;                   // a conn vanishing on its own must
                                           // leave evidence; this is the only
                                           // trace it leaves
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
