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
//
// ★ RE-ENTRANCY, and it is WiFiClass::m_inService that saves us.  allocEvicting()
// below can tcp_abort() a victim, and tcp_abort puts an RST on the wire -- so
// this callback, which is already running inside tcp_input(), reaches
// sendDataFrame()'s delay(1) -> yield() -> servicePass() -> iw416NetifPoll()
// -> tcp_input() AGAIN.  It does not, because m_inService is already true:
// acceptCb only ever runs underneath a servicePass in the first place, and
// servicePass's first line returns on that latch.  Anyone tempted to remove
// m_inService as "a guard on the read path" should read this paragraph first;
// the write path needs it just as much.
err_t WiFiServer::acceptCb(void *arg, struct tcp_pcb *newpcb, err_t err) {
    WiFiServer *srv = (WiFiServer *)arg;
    if (srv == nullptr || newpcb == nullptr || err != ERR_OK) return ERR_VAL;
    // Read the port BEFORE reserving a slot, so the assignment below cannot
    // sit behind anything that might not run.
    const uint16_t port = srv->m_port;
    WiFiConn *c = WiFiPool::allocEvicting();
    if (!c) {
        // Every slot CLAIMED: nothing to evict, so refuse.  A non-OK return is
        // what makes lwip abort the new pcb.  Counted, because otherwise this
        // is the only way a connection disappears in this subsystem that
        // leaves no trace at all -- and it is the one that means the SKETCH is
        // holding four connections it should have stop()ped.
        WiFiPool::countAcceptRefusal();
        return ERR_MEM;
    }
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
    // the LISTENER (tcp_in.c:690 copies pcb->callback_arg to the new pcb), so
    // until this line every callback on newpcb would arrive with a WiFiServer*
    // where the pool expects a WiFiConn*.  Nothing between the reservation and
    // here can fail, so there is no window in which that matters.
    WiFiPool::installCallbacks(c, newpcb);
    return ERR_OK;
}

// --- listen -----------------------------------------------------------------
void WiFiServer::begin() {
    // Idempotent -- and this guard is worth more than it looks.  Removing it
    // changes nothing OBSERVABLE (measured: a Task-9 mutation run deleting
    // this line left all 128 self-test checks green), because lwip's tcp_bind
    // refuses a second bind to a port something already listens on and returns
    // ERR_USE.  What it costs is the tcp_new() on the way to that refusal, and
    // tcp_new -> tcp_alloc is not a benign allocation: when memp_malloc fails
    // it runs tcp_kill_timewait(), tcp_kill_state(LAST_ACK),
    // tcp_kill_state(CLOSING) and finally tcp_kill_prio() -- which ABORTS AN
    // EXISTING ACTIVE CONNECTION.  With MEMP_NUM_TCP_PCB = 5 against 4 pool
    // slots, a sketch calling begin() from loop() with a full pool would
    // periodically destroy a live connection on the way to a bind that was
    // always going to fail.  So: not decoration, and not a micro-optimisation.
    //
    // (On SO_REUSE: turning it on would NOT re-open the rebind by itself --
    // tcp_bind's duplicate check needs BOTH pcbs to carry SOF_REUSEADDR and
    // nothing here sets it.  This guard becomes the only line of defence only
    // if someone later does.)
    if (m_listen) return;
    // Port 0 is refused rather than half-supported.  tcp_bind(…, 0) picks an
    // ephemeral port but leaves m_port at 0, and serverPort == 0 is the pool's
    // marker for a CLIENT connection: every conn this server accepted would be
    // invisible to the evictor, and available()/accept()/write() would reach
    // connections some WiFiClient owns.  Refusing here is only half the fix --
    // the getters are callable without a successful begin(), so each of them
    // carries the same guard.
    if (m_port == 0)              { m_err = BAD_PORT;      return; }
    if (!WiFi.lwipUp())           { m_err = NO_LINK;       return; }
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb)                     { m_err = NO_PCB;        return; }
    // tcp_abort on a CLOSED pcb is correct and is NOT the hand-rolled close
    // sequence WiFiConnPool.h warns about: no callbacks have been installed,
    // nothing but lwip can see this pcb, and tcp_abandon's CLOSED branch is
    // what removes a BOUND-but-unlistened pcb from tcp_bound_pcbs (tcp.c).
    // WiFiPool::closeConn cannot be used -- it takes a WiFiConn, and there is
    // no slot here; a listener is not a connection.
    if (tcp_bind(pcb, IP4_ADDR_ANY, m_port) != ERR_OK) {
        tcp_abort(pcb); m_err = BIND_FAILED; return;      // ERR_USE: taken
    }
    struct tcp_pcb *l = tcp_listen(pcb);      // frees pcb on success only
    if (!l) {
        tcp_abort(pcb); m_err = LISTEN_FAILED; return;    // MEMP_NUM_..._LISTEN is 2
    }
    m_listen = l;
    tcp_arg(m_listen, this);
    tcp_accept(m_listen, acceptCb);
    m_err = LISTEN_OK;
}

// --- getters ----------------------------------------------------------------
// Both service FIRST and unconditionally -- see WiFiServer.h.  For a client,
// servicing is how bytes arrive; for a server it is also how CONNECTIONS
// arrive, because acceptCb runs off lwip's input path.
//
// ★ The m_port == 0 guard is HERE, not only in begin(), and that is the point:
// begin() refusing port 0 leaves the server falsy but does NOT stop a sketch
// calling the getters, and `serverPort != m_port` with m_port == 0 matches
// every CLIENT connection in the pool.  Without this line available() hands
// back a second handle onto a live WiFiClient's connection and reads its bytes
// out from under it.  Guarding on m_listen instead would be wrong for a
// different reason: the getters are deliberately still useful after end().
WiFiClient WiFiServer::available() {
    if (m_port == 0) return WiFiClient();
    WiFi.servicePass();
    for (uint8_t i = 0; i < WIFI_MAX_CONNS; i++) {
        WiFiConn *c = WiFiPool::slot(i);
        if (c->state == WiFiConn::FREE || c->serverPort != m_port) continue;
        if (WiFiPool::availableBytes(c) > 0) return WiFiClient(c);  // claims
    }
    return WiFiClient();
}

WiFiClient WiFiServer::accept() {
    if (m_port == 0) return WiFiClient();     // see available()
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
    // Same hazard as the getters, worse consequence: with m_port == 0 this
    // loop would broadcast into every ESTABLISHED CLIENT connection in the
    // pool.  A refusal, not a silent skip -- the caller asked for something
    // that cannot be done.
    if (m_port == 0) { setWriteError(); return 0; }
    // Two caps, and the SMALLER one is the interesting one.  tcp_write's
    // length is a u16_t, so an unclamped cast truncates silently and then
    // reports success for the bytes it dropped -- but 0xFFFF is not the real
    // ceiling: nothing can ever be handed more than TCP_SND_BUF (11680 here)
    // in one call, and clamping to 0xFFFF instead would make every broadcast
    // between 11681 and 65535 bytes a guaranteed no-op that RETURNS FULL
    // SUCCESS.  Clamping to the send buffer instead puts the truth in the
    // return value, where print()/println() already read a short count as
    // truncation.  There is no queue behind this call: the remainder is the
    // caller's to send next time.
    size_t cap = (size_t)TCP_SND_BUF;
    if (cap > 0xFFFFu) cap = 0xFFFFu;
    const size_t want = (size > cap) ? cap : size;
    bool partial = (want != size);
    for (uint8_t i = 0; i < WIFI_MAX_CONNS; i++) {
        WiFiConn *c = WiFiPool::slot(i);
        // ESTABLISHED only -- deliberately UNLIKE WiFiClient::write, which
        // allows CLOSE_WAIT because a half-close reply is a real conversation.
        // A broadcast to a peer that has already said goodbye is not.
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
    // there is no single honest byte count.  Precedent followed is
    // NativeEthernetServer::write and upstream Arduino Ethernet, both of which
    // return `size`; ~/Development/Ethernet's EthernetServer returns the SUM
    // instead, and that was rejected because it makes println() on 3 peers
    // report 3 for a 1-byte write.  The minimum was rejected too -- it makes
    // one dead peer read as a failed write to everybody.  So the count answers
    // "did the call carry what you asked", and getWriteError() answers "did
    // everyone get it".
    if (partial) setWriteError();
    return want;
}

// --- teardown ---------------------------------------------------------------
void WiFiServer::end() {
    if (!m_listen) return;
    // Callbacks off before the close, same rule as everywhere else -- but done
    // by hand here, and it has to be: a listen pcb is not a WiFiConn, so
    // WiFiPool::closeConn does not apply (it takes a slot), and the pool's
    // detachCallbacks is file-static inside WiFiConnPool.cpp and not reachable
    // from here at all.  It would also be the wrong sequence even if it were:
    // lwip's tcp_recv/tcp_sent/tcp_err/tcp_poll all LWIP_ASSERT(pcb->state !=
    // LISTEN).  A listener has exactly two things attached, and these are they.
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
