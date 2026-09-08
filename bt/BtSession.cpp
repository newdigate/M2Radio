#include "BtSession.h"
#include <string.h>

void BtSession::begin(BondTable *bonds, const char *targetName, uint8_t aclNum, uint32_t now) {
    m_bonds = bonds; m_target = targetName; m_aclNum = aclNum;
    m_src.setBonds(bonds);              // the session and the source share one table (inbound accept + reconnect need it)
    m_src.begin(now, aclNum);           // resets the attempt machine and runs PREPARE once per session
    m_stats = Stats{};
    startBootWalk();                    // issues the first attempt; A2dpSource defers the link-op until PREPARE finishes
}

// Issue the next boot candidate (a PAGE), else the inquiry fallback (once).  Returns false only when the
// walk is exhausted (no more bonds AND the inquiry has already been tried).  The candidate order is the
// old A2dpSource::connect walk, moved here: bonds MOST-RECENT-FIRST, filtered by the target name (an
// EMPTY stored name is a WILDCARD, never skipped), the first PAGED candidate getting PAGE_ATTEMPTS and
// each later one 1.
bool BtSession::advanceBootWalk(uint32_t now) {
    (void)now;
    while (m_bonds && m_walkIdx < m_bonds->count()) {
        Bond b = m_bonds->at(m_walkIdx);                       // a COPY: a Link_Key_Notification can reorder the table during an attempt
        m_walkIdx++;
        if (m_target && m_target[0] && b.name[0] && !strstr(b.name, m_target)) continue;   // filtered; empty stored name is a wildcard
        A2dpSource::Target t{}; t.kind = A2dpSource::Target::PAGE;
        memcpy(t.bd, b.bd, 6); t.psrm = b.psrm; t.clkValid = false; t.clk = 0;
        BondTable::copyName(t.name, b.name);
        t.attempts = m_walkFirst ? BtLink::PAGE_ATTEMPTS : 1; m_walkFirst = false;
        beginAttempt(t);
        return true;
    }
    if (!m_triedInquiry) {
        m_triedInquiry = true;
        A2dpSource::Target t{}; t.kind = A2dpSource::Target::INQUIRY; t.nameFilter = m_target;
        beginAttempt(t);
        return true;
    }
    return false;
}

void BtSession::startBootWalk() {
    m_boot = true; m_walkIdx = 0; m_walkFirst = true; m_triedInquiry = false; m_haveLost = false;
    if (!advanceBootWalk(0)) m_state = IDLE;                   // nothing to try (never on the first call: the inquiry always fires)
}

// A reconnect is a ONE-candidate walk: a single page to the lost address only.  The bond supplies the
// page-scan-repetition-mode and name (needed for a re-pair after the peer forgot the key); with no bond
// we page with sane defaults.
void BtSession::startReconnect(uint32_t now) {
    (void)now;
    A2dpSource::Target t{}; t.kind = A2dpSource::Target::PAGE;
    memcpy(t.bd, m_lostBd, 6); t.attempts = 1; t.clkValid = false; t.clk = 0; t.psrm = 1;
    if (m_bonds) { const Bond *b = m_bonds->find(m_lostBd); if (b) { t.psrm = b->psrm; BondTable::copyName(t.name, b->name); } }
    beginAttempt(t);
}

void BtSession::beginAttempt(const A2dpSource::Target &t) {
    switch (t.kind) {
    case A2dpSource::Target::PAGE:    m_stats.by = BY_PAGED;    break;
    case A2dpSource::Target::INQUIRY: m_stats.by = BY_INQUIRY;  break;
    case A2dpSource::Target::INBOUND: m_stats.by = BY_INCOMING; break;
    }
    m_stats.attempts++;
    m_src.start(t);
    m_state = CONNECTING;
}

void BtSession::handleAttemptEnd(uint32_t now) {
    A2dpSource::Result r = m_src.result();
    m_lastResult = r;
    // m_state is settled BEFORE any callback runs.  A disconnect() called from inside a callback assigns
    // DISCONNECTING, and assigning m_state after the callback threw that away silently.  Everything else keeps
    // its old position, so what a callback OBSERVES is unchanged apart from state() itself: the stats are still
    // updated after m_attemptCb, in the same order, and no log line moves.
    if (r == A2dpSource::OK) {
        m_state = STREAMING;
        if (m_attemptCb) m_attemptCb(m_attemptCtx, r, m_src.link().pairedBy());
        m_stats.links++; m_stats.accepts++;
        if (m_haveLost) { m_stats.reconnectMs = now - m_stats.lostAt; m_haveLost = false; }
        if (m_streamCb) m_streamCb(m_streamCtx, true, 0, m_stats.by);
        return;
    }
    m_state = WAITING; m_retryAt = now + m_retryMs;
    if (m_attemptCb) m_attemptCb(m_attemptCtx, r, m_src.link().pairedBy());
    m_stats.rejects++;
    // A CONNECT_FAILED means the link never came up -> try the next boot candidate.  Any other failure
    // (PAIR/L2CAP/AVDTP/LOST) means the page succeeded and a link came up, so we must NOT page later
    // candidates -- stop the walk exactly as the old A2dpSource::connect did (a post-link failure is
    // terminal for the pass).  A reconnect (not m_boot) never advances: it retries the lost address.
    // The m_state test is the other half of the rule above: a callback that disconnected us must not have
    // its DISCONNECTING overwritten by the CONNECTING a new boot candidate would assign.
    if (m_state == WAITING && m_boot && r == A2dpSource::CONNECT_FAILED) advanceBootWalk(now);
}

void BtSession::tick(uint32_t now) {
    m_src.tick(now);
    // An accepted incoming link cancels any pending retry and starts an INBOUND attempt.
    bool handledInbound = false;
    if (m_src.link().inboundUp() && !m_src.busy() && (m_state == WAITING || m_state == IDLE)) {
        A2dpSource::Target t{}; t.kind = A2dpSource::Target::INBOUND; memcpy(t.bd, m_src.link().peer(), 6);
        beginAttempt(t); handledInbound = true;
    }
    if (!handledInbound) switch (m_state) {
    case CONNECTING:
        if (m_src.state() == A2dpSource::STREAMING) { handleAttemptEnd(now); break; }   // success: A2dpSource stays busy() in STREAMING
        if (m_src.busy() || m_src.link().busy()) break;                                  // attempt still running
        handleAttemptEnd(now);                                                           // attempt ended (DONE): failure -> advance/WAITING
        break;
    case STREAMING:
        if (m_src.result() == A2dpSource::LOST) {                                        // the attempt reported the drop (ackLost already ran in m_src.tick)
            m_stats.lost++; m_stats.lastReason = m_src.link().lostReason(); m_stats.lostAt = now;
            memcpy(m_lostBd, m_src.link().peer(), 6); m_haveLost = true;
            m_boot = false; m_state = WAITING; m_retryAt = now + m_retryMs;              // FIRST retry waits a full cycle (the headset may page us first)
            if (m_streamCb) m_streamCb(m_streamCtx, false, m_stats.lastReason, m_stats.by);   // state first: see handleAttemptEnd
        }
        break;
    case WAITING:
        if ((int32_t)(now - m_retryAt) >= 0 && !m_src.busy() && !m_src.link().busy()) {
            if (m_haveLost) startReconnect(now); else startBootWalk();                   // reconnect the lost peer, else re-run the boot walk
        }
        break;
    case DISCONNECTING:
        if (!m_src.busy() && !m_src.link().busy()) m_state = MANUAL;
        break;
    default: break;                                                                      // IDLE, MANUAL: nothing to advance
    }
    // Page scan wanted whenever no link is up and a bond exists; never in MANUAL.  Computed AFTER the
    // state machine so it reflects THIS tick's state (e.g. it goes off the tick we enter MANUAL, not
    // one tick late).
    bool linkUp = m_src.link().linkState() == BtLink::LINK_UP || m_src.link().linkState() == BtLink::LINK_SECURE;
    m_wantScan = (!linkUp && m_bonds && m_bonds->count() > 0 && m_state != MANUAL);
    m_src.link().wantPageScan(m_wantScan);
}

void BtSession::disconnect() { m_src.stop(); m_state = DISCONNECTING; }
void BtSession::resume()     { m_boot = true; startBootWalk(); }
void BtSession::retryNow()   { m_retryAt = 0; }               // fires next tick while WAITING
