#include "BtSinkSession.h"
const char *BtSinkSession::stateName(State s) {
    switch (s) { case IDLE: return "idle"; case LISTENING: return "listening"; case CONNECTING: return "connecting";
        case STREAMING: return "streaming"; case DISCONNECTING: return "disconnecting"; case MANUAL: return "manual"; }
    return "?";
}
void BtSinkSession::begin(BondTable *bonds, uint8_t aclNum, uint32_t now) {
    m_bonds = bonds; m_sink.setBonds(bonds);      // the session and the sink share one table (the inbound accept reads it)
    m_sink.begin(now, aclNum);                    // resets the attempt machine and runs PREPARE; every pairing window re-runs it (openWindow)
    m_stats = Stats{}; m_state = LISTENING;
    m_pairOpen = false; m_pairEnd = PAIR_END_NONE;
    openWindow(now, PAIR_BOOT);                   // begin()'s own PREPARE is in flight, so this one's startPrepare() declines: no double
}
bool BtSinkSession::canPair() const {
    bool linkUp = m_sink.link().linkState() == BtLink::LINK_UP || m_sink.link().linkState() == BtLink::LINK_SECURE;
    return m_state == LISTENING && !linkUp;
}
bool BtSinkSession::openWindow(uint32_t now, PairingReason r) {
    if (r != PAIR_CMD && m_pairMs == 0) return false;               // automatic windows switched off
    if (!canPair()) return false;
    m_pairUntil = now + (m_pairMs ? m_pairMs : PAIR_DEFAULT_MS);
    m_pairOpen = true; m_pairReason = r; m_pairEnd = PAIR_END_NONE;
    // Every window guarantees SSP is on.  startPrepare() returns false when an op is already in flight (the
    // boot PREPARE, or a page being accepted), which is exactly the no-double we want; tickPrepare() writes
    // only idempotent things and never touches the scan bookkeeping, so this needs no begin() -- and MUST NOT
    // use one: BtLink::begin() resets that bookkeeping to "off" and would desync host and controller.
    m_sink.link().startPrepare();
    return true;
}
bool BtSinkSession::enterPairing(uint32_t now, PairingReason r) { return openWindow(now, r); }
void BtSinkSession::tick(uint32_t now) {
    m_sink.tick(now);
    switch (m_state) {
    case LISTENING:
        // A page the controller accepted is the whole trigger: a sink never initiates.
        // attempts counts ATTEMPTS, not triggers: a start() that refuses (busy, or the link went away between
        // the two calls) never became one, and counting it there made rejects+accepts fail to add up.
        if (m_sink.link().inboundUp() && !m_sink.busy()) { if (m_sink.start()) { m_stats.attempts++; m_state = CONNECTING; } }
        break;
    case CONNECTING:
        if (m_sink.state() == A2dpSink::STREAMING) {                        // success: A2dpSink stays busy() in STREAMING
            m_stats.links++; m_stats.accepts++; m_state = STREAMING;
            // The window closes HERE, on the transition, and BEFORE any callback: the app may disconnect() from
            // inside the attempt callback below, and why a window ended must describe the WIRE -- a link reached
            // STREAMING, so it paired -- not which line the app happened to call disconnect() on.  Closed at the
            // end of tick() instead, the same wire outcome read CANCELLED from inside the callback (m_state is
            // DISCONNECTING by then) and PAIRED from loop() one tick later (btsinksession_test Q7 vs P6).
            if (m_pairOpen) { m_pairOpen = false; m_pairEnd = PAIR_END_PAIRED; }
            if (m_attemptCb) m_attemptCb(m_attemptCtx, A2dpSink::OK, m_sink.link().pairedBy());
            // ... and the stream callback only if we are STILL streaming.  The attempt callback above may have
            // called disconnect(), which assigns DISCONNECTING: announcing the stream UP on a session the app
            // has just torn down is a report that never gets its matching onStream(false) -- the DISCONNECTING
            // branch reaches MANUAL without one (btsinksession_test Q7).
            if (m_state == STREAMING && m_streamCb) m_streamCb(m_streamCtx, true, 0);
            break;
        }
        if (m_sink.busy() || m_sink.link().busy()) break;                   // attempt still running
        m_stats.rejects++;
        // m_state is assigned BEFORE the callback, here and at both STREAMING exits below: a disconnect()
        // called from inside a callback assigns DISCONNECTING, and assigning m_state afterwards threw that
        // away silently -- the session went straight back to announcing itself (btsinksession_test Q6).
        m_state = LISTENING;
        openWindow(now, PAIR_DROP);          // a FAILED attempt is the NEW-43 path: re-issue PREPARE
        if (m_attemptCb) m_attemptCb(m_attemptCtx, m_sink.result(), m_sink.link().pairedBy());
        break;
    case STREAMING:
        if (m_sink.result() == A2dpSink::LOST) {                            // the attempt reported the drop (ackLost already ran in m_sink.tick)
            m_stats.lost++; m_stats.lastReason = m_sink.link().lostReason(); m_stats.lostAt = now;
            m_state = LISTENING;
            openWindow(now, PAIR_DROP);
            if (m_streamCb) m_streamCb(m_streamCtx, false, m_stats.lastReason);
            break;
        }
        // A clean CLOSE/ABORT by the source: A2dpSink ends the attempt DISCONNECTING with result OK and
        // reaches DONE (not busy) once the link is torn down -- a completed session, not a failure.  Our own
        // disconnect() cannot be mistaken for one: it leaves STREAMING for DISCONNECTING in the same call.
        if (!m_sink.busy() && m_sink.result() == A2dpSink::OK) {
            m_stats.closed++;
            m_state = LISTENING;
            openWindow(now, PAIR_DROP);
            if (m_streamCb) m_streamCb(m_streamCtx, false, 0);
        }
        break;
    case DISCONNECTING:
        if (!m_sink.busy() && !m_sink.link().busy()) m_state = MANUAL;
        break;
    default: break;                                                          // IDLE, MANUAL: nothing to advance
    }
    // Scanning: page scan (connectable) whenever we are LISTENING with no link up, and inquiry scan
    // (discoverable) on top of it until a bond exists -- a phone that already knows us pages, it does not
    // search -- OR while a pairing window is open.  Computed AFTER the state machine so it reflects THIS
    // tick's state, exactly as BtSession does.
    bool linkUp = m_sink.link().linkState() == BtLink::LINK_UP || m_sink.link().linkState() == BtLink::LINK_SECURE;
    bool listening = (m_state == LISTENING && !linkUp);
    // The CLOCK edge is evaluated HERE, once per tick, so pairingOpen(), the scan line below and the heartbeat
    // all read the same answer for the same pass.  (A window opened this tick by a drop branch above has
    // now + window as its deadline and cannot expire on the same pass.)  The PAIRED close is NOT here: it
    // happens at the CONNECTING -> STREAMING transition above, ahead of the callbacks, so that a disconnect()
    // from inside one cannot change the reason a window ended.  Nothing else can reach STREAMING with a window
    // open -- openWindow() only opens one in LISTENING -- so there is no second close to make here.
    if (m_pairOpen && (int32_t)(now - m_pairUntil) >= 0) { m_pairOpen = false; m_pairEnd = PAIR_END_TIMEOUT; }
    m_sink.link().wantPageScan(listening);
    m_sink.link().wantDiscoverable(listening && (m_alwaysDisc || !m_bonds || m_bonds->count() == 0 || m_pairOpen));
}
// disconnect() closes an open window as CANCELLED, and CANCELLED means exactly what it says: a window that never
// saw STREAMING.  One reaching STREAMING is already closed PAIRED by the transition above, so what is left here
// is the drop window the loss branch opens ONE LINE before the stream callback that calls us (btsinksession_test
// Q6), or a boot/commanded window torn down from LISTENING or CONNECTING.  Unclosed, such a window rode through
// DISCONNECTING and MANUAL with the scans off -- pairingOpen() true on a sink that is not discoverable, the LED
// blinking "pairing" while enterPairing() is refused, and resume() re-entering LISTENING on a stale deadline with
// no PREPARE.  Guarded: with nothing open, the last window's end is left as it was (Q4, P6).
void BtSinkSession::disconnect() {
    if (m_pairOpen) { m_pairOpen = false; m_pairEnd = PAIR_END_CANCELLED; }
    m_sink.stop(); m_state = DISCONNECTING;
}
// resume() opens NO window: "every return to LISTENING" (spec s4) enumerates the ends of ATTEMPTS -- loss, clean
// close, failed pairing -- and this is an app command; the app calls enterPairing() if it wants one (Q4).
void BtSinkSession::resume()     { m_state = LISTENING; }
