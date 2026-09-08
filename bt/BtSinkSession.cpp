#include "BtSinkSession.h"
const char *BtSinkSession::stateName(State s) {
    switch (s) { case IDLE: return "idle"; case LISTENING: return "listening"; case CONNECTING: return "connecting";
        case STREAMING: return "streaming"; case DISCONNECTING: return "disconnecting"; case MANUAL: return "manual"; }
    return "?";
}
void BtSinkSession::begin(BondTable *bonds, uint8_t aclNum, uint32_t now) {
    m_bonds = bonds; m_sink.setBonds(bonds);      // the session and the sink share one table (the inbound accept reads it)
    m_sink.begin(now, aclNum);                    // resets the attempt machine and runs PREPARE once per session
    m_stats = Stats{}; m_state = LISTENING;
}
void BtSinkSession::tick(uint32_t now) {
    m_sink.tick(now);
    switch (m_state) {
    case LISTENING:
        // A page the controller accepted is the whole trigger: a sink never initiates.
        if (m_sink.link().inboundUp() && !m_sink.busy()) { m_stats.attempts++; if (m_sink.start()) m_state = CONNECTING; }
        break;
    case CONNECTING:
        if (m_sink.state() == A2dpSink::STREAMING) {                        // success: A2dpSink stays busy() in STREAMING
            m_stats.links++; m_stats.accepts++; m_state = STREAMING;
            if (m_attemptCb) m_attemptCb(m_attemptCtx, A2dpSink::OK, m_sink.link().pairedBy());
            if (m_streamCb) m_streamCb(m_streamCtx, true, 0);
            break;
        }
        if (m_sink.busy() || m_sink.link().busy()) break;                   // attempt still running
        m_stats.rejects++;
        if (m_attemptCb) m_attemptCb(m_attemptCtx, m_sink.result(), m_sink.link().pairedBy());
        m_state = LISTENING; break;
    case STREAMING:
        if (m_sink.result() == A2dpSink::LOST) {                            // the attempt reported the drop (ackLost already ran in m_sink.tick)
            m_stats.lost++; m_stats.lastReason = m_sink.link().lostReason(); m_stats.lostAt = now;
            if (m_streamCb) m_streamCb(m_streamCtx, false, m_stats.lastReason);
            m_state = LISTENING; break;
        }
        // A clean CLOSE/ABORT by the source: A2dpSink ends the attempt DISCONNECTING with result OK and
        // reaches DONE (not busy) once the link is torn down -- a completed session, not a failure.  Our own
        // disconnect() cannot be mistaken for one: it leaves STREAMING for DISCONNECTING in the same call.
        if (!m_sink.busy() && m_sink.result() == A2dpSink::OK) {
            m_stats.closed++;
            if (m_streamCb) m_streamCb(m_streamCtx, false, 0);
            m_state = LISTENING;
        }
        break;
    case DISCONNECTING:
        if (!m_sink.busy() && !m_sink.link().busy()) m_state = MANUAL;
        break;
    default: break;                                                          // IDLE, MANUAL: nothing to advance
    }
    // Scanning: page scan (connectable) whenever we are LISTENING with no link up, and inquiry scan
    // (discoverable) on top of it until a bond exists -- a phone that already knows us pages, it does not
    // search.  Computed AFTER the state machine so it reflects THIS tick's state, exactly as BtSession does.
    bool linkUp = m_sink.link().linkState() == BtLink::LINK_UP || m_sink.link().linkState() == BtLink::LINK_SECURE;
    bool listening = (m_state == LISTENING && !linkUp);
    m_sink.link().wantPageScan(listening);
    m_sink.link().wantDiscoverable(listening && (m_alwaysDisc || !m_bonds || m_bonds->count() == 0));
}
void BtSinkSession::disconnect() { m_sink.stop(); m_state = DISCONNECTING; }
void BtSinkSession::resume()     { m_state = LISTENING; }
