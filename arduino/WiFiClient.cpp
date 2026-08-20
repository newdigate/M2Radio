/* WiFiClient.cpp - see WiFiClient.h. MIT, (c) 2026 Nicholas Newdigate. */
#include "WiFiClient.h"
#include "WiFiConnPool.h"
#include "WiFi.h"
#include "Arduino.h"
#include "lwip/tcp.h"

// How long a blocked write() keeps pumping before returning a SHORT count, and
// how long flush()/stop() will wait for lwip's send buffer to drain.  Both are
// the spec's defaults (design doc §6).  Named because write()'s two retry arms
// must use the same number, and because a reader hunting "why did this block
// for five seconds" should find it by name.
static const uint32_t WIFI_TX_TIMEOUT_MS  = 5000;
// A connect() that neither completes nor errors.  10 s per the design doc; the
// abort on expiry runs through the still-attached callbacks, exactly like
// m2_throughput_test's tcpKick.
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

// --- handle lifetime --------------------------------------------------------
WiFiClient::WiFiClient(WiFiConn *conn) : m_conn(conn) {
    if (m_conn) { WiFiPool::addRef(m_conn); m_conn->claimed = true; }
}

WiFiClient::WiFiClient(const WiFiClient &o) : Client(o), m_conn(o.m_conn) {
    if (m_conn) WiFiPool::addRef(m_conn);
}

WiFiClient &WiFiClient::operator=(const WiFiClient &o) {
    if (this == &o) return *this;
    // addRef the SOURCE before releasing the destination, not after.  The two
    // lines above and below are ALTERNATIVE defences against the same bug and
    // either alone is sufficient -- measured, not assumed: reverting BOTH makes
    // `a = a` release the last reference (refs 1 -> 0 -> closeConn + toFree) and
    // then read the just-nulled o.m_conn, destroying the connection and leaving
    // a null handle; reverting either ONE leaves every self-test green.  Both
    // are kept because they fail differently -- the guard is the conventional
    // idiom a reader expects, and this order is what keeps the operator correct
    // if a future handle can ever hold a slot WITHOUT owning a reference, which
    // is the only way the two-distinct-handles-same-slot case could reach zero.
    WiFiConn *n = o.m_conn;
    if (n) WiFiPool::addRef(n);
    detach();
    m_conn = n;
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
    if (!WiFi.lwipUp() || !WiFi.linkUp()) return 0;
    stop();                            // Arduino: connect() on a live client
                                       // replaces the connection
    WiFiConn *c = WiFiPool::alloc();
    if (!c) return 0;
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
    if (!pcb) { detach(); return 0; }  // detach -> release -> slot recovered
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
        return 0;
    }
    if (!WiFi.pumpUntil(connectCond, c, WIFI_CONNECT_TIMEOUT_MS)) {
        if (c->pcb) tcp_abort(c->pcb);     // -> connErr does the bookkeeping
        detach();
        return 0;
    }
    if (!c->connectOk) { detach(); return 0; }
    return 1;
}

int WiFiClient::connect(const char *host, uint16_t port) {
    IPAddress ip;
    if (!WiFi.hostByName(host, ip)) return 0;   // stub until Task 8 lands DNS
    return connect(ip, port);
}

// --- TX ---------------------------------------------------------------------
size_t WiFiClient::write(const uint8_t *buf, size_t size) {
    if (!m_conn || !m_conn->pcb || buf == nullptr || size == 0) return 0;
    // NOT gated on state == ESTABLISHED, deliberately: after the peer FINs the
    // slot reads PEER_CLOSED while lwip's pcb is in CLOSE_WAIT, where writing is
    // still legal and is exactly how a half-close reply is sent.  The pcb's own
    // ERR_CONN is the authority on whether a write is allowed.
    size_t sent = 0;
    uint32_t t0 = millis();
    while (sent < size && m_conn->pcb) {
        u16_t room = tcp_sndbuf(m_conn->pcb);
        if (room == 0) {
            tcp_output(m_conn->pcb);              // push what is queued...
            if (millis() - t0 >= WIFI_TX_TIMEOUT_MS) break;   // ...short write
            WiFi.servicePass();                   // ...and give the ACK a
            delay(1);                             //    chance to come back
            continue;
        }
        u16_t n = (u16_t)((size - sent < room) ? size - sent : room);
        if (tcp_write(m_conn->pcb, buf + sent, n, TCP_WRITE_FLAG_COPY) != ERR_OK) {
            if (millis() - t0 >= WIFI_TX_TIMEOUT_MS) break;
            WiFi.servicePass();
            delay(1);
            continue;
        }
        sent += n;
    }
    // The pcb can have gone away inside the loop (connErr nulls it), and the
    // handle keeps the SLOT alive but not the connection -- re-test.
    if (m_conn->pcb) tcp_output(m_conn->pcb);
    return sent;
}

// --- RX ---------------------------------------------------------------------
// Every reader services the link first: this is what makes a sketch that only
// ever calls client.available() still poll the SDIO card.  servicePass() is
// bounded and self-guarding (WiFi.h), so calling it at any rate is safe.
int WiFiClient::available() {
    WiFi.servicePass();
    return WiFiPool::availableBytes(m_conn);     // null-safe; "staged now"
}

int WiFiClient::read() {
    uint8_t b;
    return (read(&b, 1) == 1) ? (int)b : -1;
}

int WiFiClient::read(uint8_t *buf, size_t size) {
    WiFi.servicePass();
    if (!m_conn || buf == nullptr) return -1;
    int got = WiFiPool::consume(m_conn, buf, (int)size);   // + tcp_recved
    // Arduino's end-of-stream: nothing left AND nothing more coming.  While the
    // connection is live, 0 means "not yet", which is not the same answer.
    if (got == 0 && m_conn->state == WiFiConn::PEER_CLOSED) return -1;
    return got;
}

int WiFiClient::peek() {
    WiFi.servicePass();
    return WiFiPool::peekByte(m_conn);           // null-safe; -1 when empty
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
    WiFi.servicePass();
    if (!m_conn) return 0;
    if (m_conn->state == WiFiConn::ESTABLISHED) return 1;
    // Arduino convention: a client with unread bytes still reads as connected,
    // so `while (c.connected()) { if (c.available()) ... }` drains the tail of
    // a connection the peer has already FIN'd instead of dropping it.
    if (m_conn->state == WiFiConn::PEER_CLOSED && WiFiPool::availableBytes(m_conn) > 0)
        return 1;
    return 0;
}
