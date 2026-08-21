/* WiFiClient.cpp - see WiFiClient.h. MIT, (c) 2026 Nicholas Newdigate. */
#include "WiFiClient.h"
#include "WiFiConnPool.h"
#include "WiFi.h"
#include "Arduino.h"
#include "lwip/tcp.h"

// How long write() keeps pumping WITHOUT PROGRESS before returning a SHORT
// count, and how long flush()/stop() will wait for lwip's send buffer to drain.
// Both are the spec's defaults (design doc §6).
//
// "Without progress" is the load-bearing half: the budget restarts on every
// accepted tcp_write, so it measures a STALL, not the call.  Measuring from
// entry instead gives a write that transferred happily for 4.9 s and then met
// one full send buffer only 0.1 s of grace before it silently truncates -- and
// with TCP_SND_BUF = 8*MSS = 11680, any write materially over ~11 kB on a slow
// link has to wait for an ACK at least once.  Spec §6's wording is per
// tcp_write ("until accepted or a write timeout expires"), not per call.
static const uint32_t WIFI_TX_TIMEOUT_MS  = 5000;
// A connect() that neither completes nor errors.  10 s per the design doc; the
// abort on expiry runs through the still-attached callbacks, exactly like
// m2_lwip_test.cpp:112's tcpKick.
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

// --- handle lifetime --------------------------------------------------------
WiFiClient::WiFiClient(WiFiConn *conn) : m_conn(conn), m_err(CONNECT_OK) {
    if (m_conn) { WiFiPool::addRef(m_conn); m_conn->claimed = true; }
}

WiFiClient::WiFiClient(const WiFiClient &o) : Client(o), m_conn(o.m_conn), m_err(o.m_err) {
    if (m_conn) WiFiPool::addRef(m_conn);
}

WiFiClient &WiFiClient::operator=(const WiFiClient &o) {
    if (this == &o) return *this;
    // Assign the BASE too.  Stream carries _timeout (setTimeout(), which every
    // readBytes/readString/parseInt honours) and Print carries write_error, and
    // the copy CONSTRUCTOR already copies both via Client(o) -- so without this
    // line `b = a` and `WiFiClient b(a)` disagree, and a client that had its
    // timeout set silently reverts to the 1000 ms default on assignment.  Not
    // observable in this example only because -ffunction-sections/--gc-sections
    // drops the Stream helpers nothing calls; WiFiServer (Task 9) assigns
    // handles for a living, which is where it would first bite.
    Client::operator=(o);
    // addRef the SOURCE before releasing the destination, not after.  This and
    // the self-guard above are ALTERNATIVE defences against the same bug and
    // either alone is sufficient -- MEASURED, not assumed, by the Task-7
    // mutation run recorded in this file's commit message (21 mutations, 20
    // caught by a named self-test check; this pair needed all four variants to
    // pin down and reds only when BOTH defences are reverted -- see gap (a)
    // there.  The ONE uncaught mutation is 06, in connected(), not this
    // pair).  Reverting BOTH makes `a = a` release the
    // last reference (refs 1 -> 0 -> closeConn + toFree) and then read the
    // just-nulled o.m_conn, destroying the connection and leaving a null
    // handle; reverting either ONE leaves every check green.  Both are kept
    // because they fail differently -- the guard is the idiom a reader expects,
    // and this order is what keeps the operator correct if a future handle can
    // ever hold a slot WITHOUT owning a reference, which is the only way the
    // two-distinct-handles-same-slot case could reach zero.
    WiFiConn *n = o.m_conn;
    if (n) WiFiPool::addRef(n);
    detach();
    m_conn = n;
    m_err  = o.m_err;
    return *this;
}

WiFiClient::~WiFiClient() { detach(); }

void WiFiClient::detach() {
    // Clear the member BEFORE releasing: release() can run closeConn() and
    // toFree(), and nothing that runs from there may find this handle still
    // pointing at a slot it has already surrendered.
    WiFiConn *c = m_conn;
    m_conn = nullptr;
    if (c) WiFiPool::release(c);
}

// --- connect ----------------------------------------------------------------
// The pool does NOT install tcp_connected (WiFiConnPool.h, obligation 3): the
// caller passes its own, and `arg` is the SLOT because installCallbacks() set
// tcp_arg.  This callback also owns the state transition -- alloc() reserves
// the slot as CONNECTING and nothing in the pool advances it.
static err_t clientConnected(void *arg, struct tcp_pcb *, err_t) {
    WiFiConn *c = (WiFiConn *)arg;
    c->state = WiFiConn::ESTABLISHED;
    c->connectDone = true;
    c->connectOk   = true;
    return ERR_OK;
}

static bool connectCond(void *arg) { return ((WiFiConn *)arg)->connectDone; }

int WiFiClient::connect(IPAddress ip, uint16_t port) {
    // UNCONDITIONALLY first, above the link check: Arduino's rule is that
    // connect() on a live client replaces the connection, and a rule with an
    // exception for "the link happens to be down" is a rule a reader has to
    // re-derive.  Free on a null handle -- stop() early-outs.
    stop();
    if (!WiFi.lwipUp() || !WiFi.linkUp()) { m_err = NO_LINK; return 0; }
    WiFiConn *c = WiFiPool::alloc();
    if (!c) { m_err = NO_SLOT; return 0; }
    // ★ addRef + adopt IMMEDIATELY, before anything that can fail.  alloc()
    // hands back a RESERVED slot (state CONNECTING, no pcb, refs 0), and a
    // reserved slot with refs 0 is invisible to release(), abortAll(), the
    // eviction scan and both callbacks -- it leaks permanently, and once
    // `claimed` is set it is not even evictable.  tcp_new() below CAN fail, so
    // the natural ordering (alloc -> tcp_new -> early return -> addRef) leaks a
    // slot per failed connect, and four of them kill the pool until reboot.
    // WiFiConnPool.h's contract, obligation 1: addRef() first, ALWAYS;
    // `claimed` is an addition to it, never a substitute.
    WiFiPool::addRef(c);
    m_conn = c;                        // adopt now, so detach() can release it
    c->claimed = true;
    c->connectDone = c->connectOk = false;
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) { detach(); m_err = NO_PCB; return 0; }  // detach -> slot back
    // MUST precede tcp_connect (obligation 2): installCallbacks is what stores
    // c->pcb, and until it runs the pool cannot see the pcb at all -- abortAll()
    // skips a slot whose pcb is null, so a link lost mid-connect would strand
    // this one.
    WiFiPool::installCallbacks(c, pcb);
    ip_addr_t dst;
    IP_ADDR4(&dst, ip[0], ip[1], ip[2], ip[3]);
    if (tcp_connect(pcb, &dst, port, clientConnected) != ERR_OK) {
        // Callbacks are attached, so this abort runs connErr, which nulls
        // c->pcb for us.  Never tcp_* a pcb after that.
        tcp_abort(pcb);
        detach();
        m_err = NO_ROUTE;
        return 0;
    }
    if (!WiFi.pumpUntil(connectCond, c, WIFI_CONNECT_TIMEOUT_MS)) {
        if (c->pcb) tcp_abort(c->pcb);     // -> connErr does the bookkeeping
        detach();
        m_err = TIMED_OUT;
        return 0;
    }
    if (!c->connectOk) { detach(); m_err = REFUSED; return 0; }
    m_err = CONNECT_OK;
    return 1;
}

int WiFiClient::connect(const char *host, uint16_t port) {
    IPAddress ip;
    if (!WiFi.hostByName(host, ip)) {           // lwip DNS; see WiFi.cpp hostByName
        stop();                                 // same replace-the-connection
        m_err = DNS_FAILED;                     // rule as the IP overload
        return 0;
    }
    return connect(ip, port);
}

// --- TX ---------------------------------------------------------------------
int WiFiClient::availableForWrite() {
    // Print's default is `return 0`, which makes the standard Arduino guard
    // `while (!c.availableForWrite()) ;` an infinite loop and
    // `if (c.availableForWrite() > 0) c.write(b);` a silent no-op.
    return (m_conn && m_conn->pcb) ? (int)tcp_sndbuf(m_conn->pcb) : 0;
}

size_t WiFiClient::write(const uint8_t *buf, size_t size) {
    if (size == 0) return 0;                 // not an error: nothing was asked
    if (!m_conn || !m_conn->pcb || buf == nullptr) {
        setWriteError();                     // Print's standard channel: the
        return 0;                            // caller almost always ignores
    }                                        // the size_t (print/println do)
    // NOT gated on state == ESTABLISHED, deliberately: after the peer FINs the
    // slot reads PEER_CLOSED while lwip's pcb is in CLOSE_WAIT, where writing is
    // still legal and is exactly how a half-close reply is sent.  The pcb's own
    // return code is the authority on whether a write is allowed -- which is
    // why only ERR_MEM is retried below.
    size_t sent = 0;
    uint32_t t0 = millis();                  // restarts on progress; see the
    while (sent < size && m_conn->pcb) {     // WIFI_TX_TIMEOUT_MS comment
        u16_t room = tcp_sndbuf(m_conn->pcb);
        if (room == 0) {
            tcp_output(m_conn->pcb);              // push what is queued...
            if (millis() - t0 >= WIFI_TX_TIMEOUT_MS) break;   // ...short write
            WiFi.servicePass();                   // ...and give the ACK a
            delay(1);                             //    chance to come back
            continue;
        }
        u16_t n = (u16_t)((size - sent < room) ? size - sent : room);
        err_t we = tcp_write(m_conn->pcb, buf + sent, n, TCP_WRITE_FLAG_COPY);
        if (we != ERR_OK) {
            // ERR_MEM is the only transient one -- lwip is momentarily out of
            // pbufs or queue slots and an ACK fixes it.  ERR_CONN (wrong pcb
            // state), ERR_ARG and ERR_VAL never come good, so retrying them for
            // five seconds burns the budget to reach the same answer.
            if (we != ERR_MEM) break;
            if (millis() - t0 >= WIFI_TX_TIMEOUT_MS) break;
            WiFi.servicePass();
            delay(1);
            continue;
        }
        sent += n;
        t0 = millis();                       // PROGRESS: restart the stall clock
    }
    // The pcb can have gone away inside the loop (connErr nulls it), and the
    // handle keeps the SLOT alive but not the connection -- re-test.
    if (m_conn->pcb) tcp_output(m_conn->pcb);
    if (sent < size) setWriteError();        // a truncated body is otherwise
    return sent;                             // invisible to print()/println()
}

// --- RX ---------------------------------------------------------------------
// SERVICE ONLY WHEN THE BUFFER IS DRY.  The header explains why; the mechanism
// is that a service pass ends in the driver's trailing delay(1) and delay()
// spins to the next millis tick, so an unconditional pass here costs ~1 ms and,
// during a drain, cannot deliver anything: tcp_recved() is deferred to
// consume(), so while the sketch is draining, the receive window is closed and
// the card genuinely has nothing to hand over.  Servicing when nothing is
// staged is what keeps the header's promise -- a sketch that only ever calls
// available() still polls the card, because an empty buffer always services.
int WiFiClient::available() {
    int n = WiFiPool::availableBytes(m_conn);    // null-safe; "staged now"
    if (n > 0) return n;
    WiFi.servicePass();
    return WiFiPool::availableBytes(m_conn);
}

int WiFiClient::read() {
    uint8_t b;
    return (read(&b, 1) == 1) ? (int)b : -1;
}

int WiFiClient::read(uint8_t *buf, size_t size) {
    if (!m_conn || buf == nullptr) { WiFi.servicePass(); return -1; }
    if (WiFiPool::availableBytes(m_conn) <= 0) WiFi.servicePass();
    int got = WiFiPool::consume(m_conn, buf, (int)size);   // + tcp_recved
    // Arduino's end-of-stream: nothing left AND nothing more coming.  While the
    // connection is live, 0 means "not yet", which is not the same answer.
    if (got == 0 && m_conn->state == WiFiConn::PEER_CLOSED) return -1;
    return got;
}

int WiFiClient::peek() {
    int b = WiFiPool::peekByte(m_conn);          // null-safe; -1 when empty
    if (b >= 0) return b;
    WiFi.servicePass();
    return WiFiPool::peekByte(m_conn);
}

// --- close ------------------------------------------------------------------
static bool drainedCond(void *arg) {
    WiFiConn *c = (WiFiConn *)arg;
    // A vanished pcb counts as drained: there is nothing left to wait for, and
    // waiting out the full timeout on a connection lwip has already torn down
    // is a 5 s stall for no information.
    return !c->pcb || tcp_sndbuf(c->pcb) == TCP_SND_BUF;
}

void WiFiClient::flush() {
    if (!m_conn || !m_conn->pcb) return;
    tcp_output(m_conn->pcb);
    (void)WiFi.pumpUntil(drainedCond, m_conn, WIFI_TX_TIMEOUT_MS);
}

void WiFiClient::stop() {
    if (!m_conn) return;
    flush();                                   // bounded drain, then close
    // closeConn clears every callback before tcp_close and falls back to
    // tcp_abort -- do not hand-roll it.  Its return value is what an
    // IN-CALLBACK caller must return to lwip; we are in sketch context, so it
    // is discarded here on purpose.
    if (m_conn->pcb) (void)WiFiPool::closeConn(m_conn);
    // Other handles may still be reading staged bytes out of this slot, so the
    // slot survives; it is just no longer connected.  detach() frees it if this
    // was the last handle.
    m_conn->state = WiFiConn::PEER_CLOSED;
    detach();
}

uint8_t WiFiClient::connected() {
    // Same dry-buffer rule as the readers, and here the short-circuit is EXACT
    // rather than an approximation: with bytes staged the answer is 1 under
    // both ESTABLISHED and PEER_CLOSED, and a CONNECTING slot can never have
    // any (nothing stages before the connected callback runs).
    if (m_conn && m_conn->state != WiFiConn::CONNECTING &&
        WiFiPool::availableBytes(m_conn) > 0) return 1;
    WiFi.servicePass();
    if (!m_conn) return 0;
    if (m_conn->state == WiFiConn::ESTABLISHED) return 1;
    // Arduino convention: a client with unread bytes still reads as connected,
    // so `while (c.connected()) { if (c.available()) ... }` drains the tail of
    // a connection the peer has already FIN'd instead of dropping it.
    //
    // NOT redundant with the short-circuit at the top, though it looks it: the
    // servicePass() between them can BOTH stage bytes and set PEER_CLOSED (one
    // pass delivers a segment and then the FIN through connRecv), so this is
    // the POST-service test and that one is the pre-service test.  Deleting
    // this would drop the tail of every connection whose data and FIN land in
    // the same pass -- which is the common case for a small HTTP response.
    // Honest coverage note: the on-target self-test cannot reach this branch
    // (its servicePass() is inert with no card), so unlike its neighbours it is
    // reasoned, not measured.  Do not "simplify" it on a green self-test.
    if (m_conn->state == WiFiConn::PEER_CLOSED && WiFiPool::availableBytes(m_conn) > 0)
        return 1;
    return 0;
}
