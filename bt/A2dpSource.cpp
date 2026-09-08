#include "A2dpSource.h"
#include "HciEvents.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
const char *A2dpSource::resultName(Result r) {
    switch (r) { case OK: return "ok"; case CONNECT_FAILED: return "connect_failed";
        case PAIR_FAILED: return "pair_failed"; case L2CAP_FAILED: return "l2cap_failed";
        case AVDTP_FAILED: return "avdtp_failed"; case LOST: return "lost";
        case STOPPED: return "stopped"; case PENDING: return "pending"; } return "?";
}
void A2dpSource::logf(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); vsnprintf(m_lb, sizeof m_lb, fmt, ap); va_end(ap);
    if (m_log) m_log(m_logCtx, m_lb);
}
void A2dpSource::onData(void *ctx, L2cap::Channel &ch, const uint8_t *p, uint16_t len) {
    A2dpSource *s = (A2dpSource *)ctx;
    if (s->m_sdpServer.onData(ch, p, len)) return;             // the PEER's SDP query of us (its own channel): answered in service()
    if (s->m_avrcp.onData(ch, p, len)) return;                 // the PEER's AV/C command on its AVCTP channel: answered in service()
    if (ch.psm == Avdtp::PSM) {
        // Route the AVDTP signalling channel to the state machine -- both our outbound channel (0x0041)
        // and the peer-opened acceptor channel (its L2cap-assigned CID).  The media channel carries RTP,
        // never signalling, so it is the one AVDTP channel we do NOT forward.
        uint16_t media = s->m_avdtp.mediaRemoteCid();
        if (media != 0 && ch.remoteCid == media) return;
        s->m_avdtp.onSignalling(p, len);
    } else if (ch.psm == Sdp::PSM) { s->m_sdpVer = Sdp::parseAvdtpVersion(p, len); s->m_sdpDone = true; }   // OUR client channel
}
void A2dpSource::adoptConfig() {
    const Avdtp::SbcConfig &c = m_avdtp.sbcConfig();
    m_params.rate = c.rate >= 48000 ? Sbc::RATE_48000 : c.rate >= 44100 ? Sbc::RATE_44100
                  : c.rate >= 32000 ? Sbc::RATE_32000 : Sbc::RATE_16000;
    m_params.mode = c.mode == Avdtp::MONO ? Sbc::MONO : c.mode == Avdtp::DUAL ? Sbc::DUAL
                  : c.mode == Avdtp::STEREO ? Sbc::STEREO : Sbc::JOINT_STEREO;
    m_params.alloc = c.alloc == Avdtp::SNR ? Sbc::SNR : Sbc::LOUDNESS;
    m_params.blocks = c.blocks; m_params.subbands = c.subbands; m_params.bitpool = c.maxBitpool;
    logf("attempt: adopted sbc bitpool=%u mode=%u blocks=%u sub=%u", c.maxBitpool, (unsigned)m_params.mode, c.blocks, c.subbands);
}
void A2dpSource::begin(uint32_t now, uint8_t aclNum) {
    m_aclNum = aclNum; m_st = IDLE; m_result = OK;
    m_link.begin(now); m_link.startPrepare();                  // PREPARE once per session; start() waits for it via LINKING/PAIRING
}
bool A2dpSource::start(const Target &t) {
    if (busy()) return false;
    // A fresh attempt forgets any prior link loss: the DISCONNECTING teardown path is EXCLUDED from tick()'s
    // loss-check, so a deliberate disconnect() (or a reconnect after a drop) leaves BtLink in LINK_LOST; without
    // this ack the first LINKING tick would abort the new attempt as LOST before Connection_Complete is consumed.
    // ackLost() only clears LINK_LOST->LINK_NONE, so it is a no-op for an INBOUND target (link already UP).
    m_link.ackLost();
    // ... and any prior attempt's AVDTP/AVRCP state.  Only tick()'s LOSS branch resets these, so an attempt that
    // ended any OTHER way (STOPPED, CONNECT/PAIR/L2CAP/AVDTP_FAILED) leaves Avdtp::m_sig and m_role stale:
    // adoptInbound()'s `if (!m_sig)` guard then skips adoption, the next INBOUND attempt believes it is already
    // the ACCEPTOR, and answers the peer's DISCOVER on whatever channel that dangling pointer's slot now holds --
    // a different channel, or cid 0x0000 if L2cap::begin() left the slot empty.  a2dpsource_test's last scenario
    // is the regression; A2dpSink::start() carries the same reset for the same reason.
    m_avdtp.reset(); m_avrcp.reset();
    m_t = t; m_inbound = (t.kind == Target::INBOUND);
    // An OUTBOUND attempt negotiates the initiator's own config (bitpool 53); reset m_params so it does NOT
    // inherit a prior INBOUND attempt's ADOPTED config (adoptConfig() is the only other writer).  Without
    // this, an outbound reconnect after an inbound stream (lifecycle leg 3) encodes the adopted bitpool.
    if (!m_inbound) m_params = { Sbc::RATE_44100, Sbc::JOINT_STEREO, 16, 8, Sbc::LOUDNESS, 53 };
    m_pagedFromInquiry = false; m_opIssued = false; m_startWaitAt = 0;
    m_sdpChan = m_sigChan = nullptr; m_result = PENDING;
    switch (t.kind) {
    case Target::PAGE:    m_opIssued = m_link.startPage(t.bd, t.psrm, t.clk, t.clkValid, t.name, t.attempts); m_st = LINKING; break;
    case Target::INQUIRY: m_opIssued = m_link.startInquiry(t.nameFilter); m_st = LINKING; break;
    case Target::INBOUND: m_link.ackInboundUp(); m_opIssued = m_link.startPair(true); m_st = PAIRING; break;
    }
    return true;
}
void A2dpSource::stop() { m_result = STOPPED; m_st = DISCONNECTING; }
// Outbound: open our AVDTP signalling channel and wait (in AVDTP_WAIT) for it to reach OPEN.
void A2dpSource::beginAvdtpConnect(uint32_t now) {
    m_sigChan = m_l2.connect(Avdtp::PSM, 0x0041);
    if (!m_sigChan) { m_result = L2CAP_FAILED; m_st = DISCONNECTING; return; }
    m_st = AVDTP_WAIT; m_deadline = now + 5000;
}
void A2dpSource::tick(uint32_t now) {
    m_link.tick(now);
    // A link loss in any live state tears the media path down and ends the attempt LOST.
    if (m_st != IDLE && m_st != DONE && m_st != DISCONNECTING && m_link.lost()) {
        m_link.ackLost(); m_avdtp.reset(); m_avrcp.reset(); m_l2.reset();
        logf("attempt: link lost reason=0x%02X", m_link.lostReason());
        m_result = LOST; m_st = DONE; return;
    }
    switch (m_st) {
    case LINKING:
        if (m_link.busy()) return;
        if (!m_opIssued) {                                     // PREPARE was still running at start(): launch the op now
            m_opIssued = (m_t.kind == Target::INQUIRY) ? m_link.startInquiry(m_t.nameFilter)
                       : m_link.startPage(m_t.bd, m_t.psrm, m_t.clk, m_t.clkValid, m_t.name, m_t.attempts);
            return;
        }
        if (m_t.kind == Target::INQUIRY && m_link.op() == BtLink::NONE && m_link.result() == BtLink::OK && !m_pagedFromInquiry) {
            BtLink::Target h = m_link.target();
            if (!h.valid) { m_result = CONNECT_FAILED; m_st = DISCONNECTING; break; }
            m_pagedFromInquiry = true; m_link.startPage(h.bd, h.psrm, h.clk, h.clkValid, h.name, BtLink::PAGE_ATTEMPTS); return;
        }
        if (m_link.result() != BtLink::OK) { m_result = CONNECT_FAILED; m_st = DISCONNECTING; break; }
        m_opIssued = m_link.startPair(m_inbound); m_st = PAIRING; break;
    case PAIRING:
        if (m_link.busy()) return;
        if (!m_opIssued) { m_opIssued = m_link.startPair(m_inbound); return; }   // PREPARE delayed the pair (INBOUND)
        if (m_link.result() != BtLink::OK) { m_result = PAIR_FAILED; m_st = DISCONNECTING; break; }
        m_l2.begin(m_link.handle(), m_aclNum); m_l2.acceptIncoming(true);
        m_l2.allowPsm(Avdtp::PSM); m_l2.allowPsm(Sdp::PSM);
        m_l2.allowPsm(Avrcp::PSM);                  // NEW-34 piece 3: the headset's AVCTP channel (measured: opened 1.8 s after START)
        m_l2.onData(onData, this);
        m_st = L2; m_deadline = now + 5000;
        if (!m_inbound) m_sdpChan = m_l2.connect(Sdp::PSM, 0x0040);              // outbound: query the sink's AVDTP version
        break;
    case L2:
        if (m_inbound) { m_st = AVDTP_WAIT; m_deadline = now + m_avdtpWaitMs; m_startWaitAt = 0; break; }
        if (!m_sdpChan) { beginAvdtpConnect(now); break; }                      // no SDP channel available: skip SDP
        if (m_sdpChan->state == L2cap::OPEN) {                                   // query the AudioSink ProtocolDescriptorList
            uint8_t q[18]; m_l2.send(m_sdpChan->remoteCid, q, Sdp::buildAudioSinkPdlRequest(q, 1));
            m_sdpDone = false; m_st = SDP; m_deadline = now + 5000; break;
        }
        if ((int32_t)(now - m_deadline) < 0) return;                            // still waiting for the SDP channel to open
        beginAvdtpConnect(now); break;                                          // SDP never opened -> straight to AVDTP
    case SDP:
        if (m_sdpDone || (int32_t)(now - m_deadline) >= 0) { beginAvdtpConnect(now); break; }   // SDP is informational; a timeout is not fatal
        return;
    case AVDTP_WAIT:
        if (m_inbound) {
            m_avdtp.adoptInbound(m_l2);                                         // adopt the peer's signalling channel once it opens
            if (m_avdtp.role() == Avdtp::ACCEPTOR) { m_st = AVDTP; m_deadline = now + 15000; m_startWaitAt = 0; break; }
            if ((int32_t)(now - m_deadline) < 0) return;
            L2cap::Channel *sig = m_l2.connect(Avdtp::PSM, 0x0041);             // peer never opened AVDTP: initiate ourselves
            if (!sig) { m_result = L2CAP_FAILED; m_st = DISCONNECTING; break; }
            Avdtp::SbcConfig want = { 44100, Avdtp::JOINT_STEREO, 16, 8, Avdtp::LOUDNESS, 2, 53 };
            m_avdtp.begin(m_l2, 0x0041, 0x0042); m_avdtp.setPeerVersion(m_sdpVer); m_avdtp.start(want); m_st = AVDTP; m_deadline = now + 15000; break;
        }
        if (m_sigChan && m_sigChan->state == L2cap::OPEN) {                      // outbound: our signalling channel is up -> initiate
            Avdtp::SbcConfig want = { 44100, Avdtp::JOINT_STEREO, 16, 8, Avdtp::LOUDNESS, 2, 53 };
            m_avdtp.begin(m_l2, 0x0041, 0x0042); m_avdtp.setPeerVersion(m_sdpVer); m_avdtp.start(want); m_st = AVDTP; m_deadline = now + 15000; break;
        }
        if ((int32_t)(now - m_deadline) < 0) return;
        m_result = L2CAP_FAILED; m_st = DISCONNECTING; break;
    case AVDTP:
        if (m_inbound) {
            m_avdtp.adoptInbound(m_l2);                                         // pick up the peer's media channel in OPENING
            if (m_avdtp.configChanged()) adoptConfig();
            if (m_avdtp.mediaReady()) {                                         // media OPEN but no peer START: self-START after START_WAIT_MS
                if (m_startWaitAt == 0) m_startWaitAt = now + START_WAIT_MS;
                else if ((int32_t)(now - m_startWaitAt) >= 0) m_avdtp.startSelf();
            }
        }
        if (m_avdtp.started()) { m_result = OK; m_st = STREAMING; break; }
        if (m_avdtp.state() == Avdtp::FAILED) { m_result = AVDTP_FAILED; m_st = DISCONNECTING; break; }
        if ((int32_t)(now - m_deadline) >= 0) { m_result = AVDTP_FAILED; m_st = DISCONNECTING; break; }
        break;
    case DISCONNECTING:
        if (!m_link.busy() && m_link.op() == BtLink::NONE && m_link.handle() == 0) { m_st = DONE; return; }
        if (m_link.op() != BtLink::DISCONNECT) m_link.startDisconnect();
        break;
    default: break;                                                            // IDLE, STREAMING, DONE: nothing to advance
    }
    m_l2.service(); m_avdtp.service(); m_sdpServer.service(m_l2);
    { uint32_t before = m_avrcp.notifications(); m_avrcp.service(m_l2);   // NEW-34 piece 3: answer the headset's AV/C
      if (m_avrcp.notifications() != before) logf("avrcp: register_notification playback_status -> interim playing"); }
}
