/* WiFiServer.cpp - see WiFiServer.h. MIT, (c) 2026 Nicholas Newdigate. */
#include "WiFiServer.h"
#include "WiFiConnPool.h"
#include "WiFi.h"
#include "lwip/tcp.h"

// --- accept -----------------------------------------------------------------
// lwip's contract for this callback, all three halves of it (tcp_in.c 2.2.1):
//   * It is NOT called at SYN time.  tcp_listen_input allocates the pcb and
//     answers the SYN itself; the callback fires from tcp_process when the
//     final ACK arrives and the pcb reaches ESTABLISHED (tcp_in.c:956).  So
//     `newpcb` here is a live, established connection, which is why the slot
//     is marked ESTABLISHED rather than CONNECTING.
//   * Returning anything except ERR_OK makes lwip tcp_abort() the new pcb for
//     us -- that IS the way to refuse an accept, and the only way that does
//     not leak the pcb.  ERR_ABRT is the one code we must never return: it
//     tells lwip we already aborted it, and we never have.
//   * It can be called with newpcb == nullptr and err == ERR_MEM, from the
//     tcp_alloc failure path (tcp_in.c:667), where the return value is
//     discarded.  Never dereference newpcb before testing it.
err_t WiFiServer::acceptCb(void *arg, struct tcp_pcb *newpcb, err_t err) {
    WiFiServer *srv = (WiFiServer *)arg;
    if (srv == nullptr || newpcb == nullptr || err != ERR_OK) return ERR_VAL;
    // Read the port BEFORE reserving a slot, so the assignment below cannot
    // sit behind anything that might not run.
    const uint16_t port = srv->m_port;
    WiFiConn *c = WiFiPool::allocEvicting();
    if (!c) return ERR_MEM;              // every slot claimed: refuse; lwip
                                         // aborts the new pcb on non-OK
    // ★ serverPort FIRST.  This is the accept path's whole obligation to the
    // pool: a reserved slot must be reachable by SOME reaper, and for an
    // accept the reapers are the evictor (!claimed && serverPort != 0) and
    // connPoll's stall valve (!claimed && refs == 0).  Both need this field,
    // and neither can see the slot until it is set.
    c->serverPort = port;
    c->state = WiFiConn::ESTABLISHED;
    // ...and refs stays 0, claimed stays false, ON PURPOSE.  See WiFiServer.h.
    // addRef() here would satisfy the pool's ordering rule and leak the
    // connection permanently -- the two valves above are exactly the reapers
    // an unmatched reference would blind.  The reference is taken later, by
    // WiFiClient(WiFiConn*), when a handle actually adopts the slot.
    c->claimed = false;
    c->connectDone = true; c->connectOk = true;
    // Stores c->pcb and overwrites lwip's tcp_arg -- which lwip inherited from
    // the LISTENER (tcp_in.c copies pcb->callback_arg to the new pcb), so
    // until this line every callback on newpcb would arrive with a WiFiServer*
    // where the pool expects a WiFiConn*.  Nothing between the reservation and
    // here can fail, so there is no window in which that matters.
    WiFiPool::installCallbacks(c, newpcb);
    return ERR_OK;
}

// --- listen -----------------------------------------------------------------
void WiFiServer::begin() {
    // Idempotent -- but BELT AND BRACES, not the only thing holding the line,
    // and worth knowing which: with SO_REUSE off, lwip's own tcp_bind refuses
    // a second bind to a port something already listens on (ERR_USE), so
    // dropping this guard changes nothing observable.  MEASURED, not reasoned:
    // a Task-9 mutation run removing this line left all 127 self-test checks
    // green.  It stays because it says the intent, and because the fallback
    // costs a tcp_new + tcp_bind + tcp_abort on every call -- which matters
    // for the sketches that call begin() from loop().
    if (m_listen) return;
    // Port 0 is refused rather than half-supported.  tcp_bind(…, 0) picks an
    // ephemeral port but leaves m_port at 0, and serverPort == 0 is the pool's
    // marker for a CLIENT connection: every conn this server accepted would be
    // invisible to the evictor, and available()/accept() would hand out
    // connections some WiFiClient owns.  Both are silent, so refuse loudly-ish
    // (the server stays falsy) instead.
    if (m_port == 0) return;
    if (!WiFi.lwipUp()) return;               // no-link guard: stays falsy
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) return;
    // tcp_abort on a CLOSED pcb is correct and is NOT the hand-rolled close
    // sequence WiFiConnPool.h warns about: no callbacks have been installed,
    // nothing but lwip can see this pcb, and tcp_abandon's CLOSED branch is
    // what removes a BOUND-but-unlistened pcb from tcp_bound_pcbs (tcp.c).
    // WiFiPool::closeConn cannot be used -- it takes a WiFiConn, and there is
    // no slot here; a listener is not a connection.
    if (tcp_bind(pcb, IP4_ADDR_ANY, m_port) != ERR_OK) { tcp_abort(pcb); return; }
    struct tcp_pcb *l = tcp_listen(pcb);      // frees pcb on success only
    if (!l) { tcp_abort(pcb); return; }       // MEMP_NUM_TCP_PCB_LISTEN is 2
    m_listen = l;
    tcp_arg(m_listen, this);
    tcp_accept(m_listen, acceptCb);
}

// --- getters ----------------------------------------------------------------
// Both service FIRST and unconditionally -- see WiFiServer.h.  For a client,
// servicing is how bytes arrive; for a server it is also how CONNECTIONS
// arrive, because acceptCb runs off lwip's input path.
WiFiClient WiFiServer::available() {
    WiFi.servicePass();
    for (uint8_t i = 0; i < WIFI_MAX_CONNS; i++) {
        WiFiConn *c = WiFiPool::slot(i);
        if (c->state == WiFiConn::FREE || c->serverPort != m_port) continue;
        if (WiFiPool::availableBytes(c) > 0) return WiFiClient(c);  // claims
    }
    return WiFiClient();
}

WiFiClient WiFiServer::accept() {
    WiFi.servicePass();
    for (uint8_t i = 0; i < WIFI_MAX_CONNS; i++) {
        WiFiConn *c = WiFiPool::slot(i);
        if (c->state == WiFiConn::FREE || c->serverPort != m_port) continue;
        if (!c->claimed) return WiFiClient(c);                      // claims
    }
    return WiFiClient();
}

// --- broadcast --------------------------------------------------------------
size_t WiFiServer::write(const uint8_t *buf, size_t size) {
    if (size == 0) return 0;                 // not an error: nothing was asked
    if (buf == nullptr) { setWriteError(); return 0; }
    // tcp_write's length is a u16_t.  A plain (u16_t) cast of a larger size_t
    // truncates SILENTLY and then reports success for the bytes it dropped, so
    // clamp and let the short return say what happened -- print()/println()
    // read a short count as truncation, which is exactly what it is.
    const size_t want = (size > 0xFFFFu) ? 0xFFFFu : size;
    bool partial = (want != size);
    for (uint8_t i = 0; i < WIFI_MAX_CONNS; i++) {
        WiFiConn *c = WiFiPool::slot(i);
        if (c->serverPort != m_port || c->state != WiFiConn::ESTABLISHED || !c->pcb)
            continue;
        // Bounded and non-blocking BY CONSTRUCTION: exactly one tcp_write
        // attempt per connection, no pump, no delay().  WiFiClient::write's
        // 5 s stall budget is deliberately NOT reused here -- four backed-up
        // peers would make one broadcast a 20 s call, and a broadcast has no
        // single peer whose flow control it is entitled to wait on.
        //
        // ALL-OR-NOTHING per connection: a peer without room for the whole
        // call loses this copy rather than receiving the first half of a
        // message whose second half is never sent.  tcp_write itself is
        // all-or-nothing for the length given, so the only way to split one
        // would be to ask for less on purpose.
        u16_t room = tcp_sndbuf(c->pcb);
        if ((size_t)room < want ||
            tcp_write(c->pcb, buf, (u16_t)want, TCP_WRITE_FLAG_COPY) != ERR_OK) {
            partial = true;                  // this peer missed it
            continue;
        }
        tcp_output(c->pcb);
    }
    // The count is BYTES OFFERED, not bytes every peer took: with N peers
    // there is no single honest byte count, and both alternatives mislead --
    // the sum (what EthernetServer returns) makes println() on 3 peers report
    // 3 for a 1-byte write, and the minimum makes one dead peer read as a
    // failed write to everybody.  So the count answers "did the call carry
    // what you asked", and getWriteError() answers "did everyone get it".
    if (partial) setWriteError();
    return want;
}

// --- teardown ---------------------------------------------------------------
void WiFiServer::end() {
    if (!m_listen) return;
    // Callbacks off before the close, same rule as everywhere else -- but a
    // listen pcb is NOT a WiFiConn, so WiFiPool::closeConn/detachCallbacks are
    // the wrong tools and would in fact assert: lwip's tcp_recv/tcp_sent/
    // tcp_err/tcp_poll all LWIP_ASSERT(pcb->state != LISTEN).  A listener has
    // exactly two things attached, and these are they.
    tcp_arg(m_listen, nullptr);
    tcp_accept(m_listen, nullptr);
    // Listen pcbs close synchronously and cannot fail (tcp_close_shutdown's
    // LISTEN branch: tcp_listen_closed, unlink, free, return ERR_OK), so there
    // is no abort fallback to write -- and tcp_abort on a listener is illegal
    // anyway (tcp_abandon asserts state != LISTEN).  tcp_listen_closed also
    // nulls ->listener on every half-open pcb this listener spawned, so a SYN
    // that arrived a moment ago cannot come back through a freed callback.
    (void)tcp_close(m_listen);
    m_listen = nullptr;
}
