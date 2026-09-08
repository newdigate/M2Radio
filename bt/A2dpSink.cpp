#include "A2dpSink.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
const char *A2dpSink::resultName(Result r) {
    switch (r) { case OK: return "ok"; case PAIR_FAILED: return "pair_failed"; case L2CAP_FAILED: return "l2cap_failed";
        case AVDTP_FAILED: return "avdtp_failed"; case LOST: return "lost"; case STOPPED: return "stopped"; case PENDING: return "pending"; } return "?";
}
void A2dpSink::logf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); vsnprintf(m_lb, sizeof m_lb, fmt, ap); va_end(ap); if (m_log) m_log(m_logCtx, m_lb); }
void A2dpSink::onData(void *ctx, L2cap::Channel &ch, const uint8_t *p, uint16_t len) {
    A2dpSink *s = (A2dpSink *)ctx;
    if (s->m_sdpServer.onData(ch, p, len)) return;
    if (s->m_avrcp.onData(ch, p, len)) return;
    if (ch.psm != Avdtp::PSM) return;
    uint16_t media = s->m_avdtp.mediaRemoteCid();
    if (media != 0 && ch.remoteCid == media) {                                          // the media transport: RTP to the audio node
        if (s->m_mediaFn && s->m_st == STREAMING && !s->suspended()) s->m_mediaFn(s->m_mediaCtx, p, len);
        return;
    }
    s->m_avdtp.onSignalling(p, len);
}
void A2dpSink::adoptConfig() {
    const Avdtp::SbcConfig &c = m_avdtp.sbcConfig();
    m_params.rate = Sbc::RATE_44100; m_params.mode = c.mode == Avdtp::MONO ? Sbc::MONO : c.mode == Avdtp::DUAL ? Sbc::DUAL : c.mode == Avdtp::STEREO ? Sbc::STEREO : Sbc::JOINT_STEREO;
    m_params.alloc = Sbc::LOUDNESS; m_params.blocks = c.blocks; m_params.subbands = c.subbands; m_params.bitpool = c.maxBitpool;
    logf("sink: adopted sbc bitpool=%u mode=%u blocks=%u sub=%u", c.maxBitpool, (unsigned)m_params.mode, c.blocks, c.subbands);
}
void A2dpSink::begin(uint32_t now, uint8_t aclNum) { m_aclNum = aclNum; m_st = IDLE; m_result = OK; m_link.begin(now); m_link.startPrepare(); }
bool A2dpSink::start() {
    if (busy() || !m_link.inboundUp()) return false;
    m_link.ackLost(); m_link.ackInboundUp(); m_delaySent = false; m_result = PENDING;
    m_opIssued = m_link.startPair(true); m_st = PAIRING; return true;
}
void A2dpSink::stop() { m_result = STOPPED; m_st = DISCONNECTING; }
void A2dpSink::tick(uint32_t now) {
    m_link.tick(now);
    if (m_st != IDLE && m_st != DONE && m_st != DISCONNECTING && m_link.lost()) {
        m_link.ackLost(); m_avdtp.reset(); m_avrcp.reset(); m_l2.reset();
        logf("sink: link lost reason=0x%02X", m_link.lostReason()); m_result = LOST; m_st = DONE; return;
    }
    switch (m_st) {
    case PAIRING:
        if (m_link.busy()) return;
        if (!m_opIssued) { m_opIssued = m_link.startPair(true); return; }
        if (m_link.result() != BtLink::OK) { m_result = PAIR_FAILED; m_st = DISCONNECTING; break; }
        m_l2.begin(m_link.handle(), m_aclNum); m_l2.acceptIncoming(true);
        m_l2.allowPsm(Avdtp::PSM); m_l2.allowPsm(Sdp::PSM); m_l2.allowPsm(Avrcp::PSM); m_l2.onData(onData, this);
        m_avdtp.begin(m_l2, 0, 0); m_avdtp.setLocalSep(Avdtp::SEP_SINK);
        m_st = AVDTP_WAIT; m_deadline = now + 20000; break;                                 // a phone opens AVDTP within seconds; iOS sometimes after AVRCP
    case AVDTP_WAIT:
        m_avdtp.adoptInbound(m_l2);
        if (m_avdtp.role() == Avdtp::ACCEPTOR) { m_st = AVDTP; m_deadline = now + 30000; break; }
        if ((int32_t)(now - m_deadline) >= 0) { m_result = L2CAP_FAILED; m_st = DISCONNECTING; }
        break;
    case AVDTP:
        m_avdtp.adoptInbound(m_l2);
        if (m_avdtp.configChanged()) adoptConfig();
        if (m_avdtp.started()) { m_result = OK; m_st = STREAMING; logf("sink: streaming bitpool=%u", m_params.bitpool); break; }
        if (m_avdtp.state() == Avdtp::FAILED || (int32_t)(now - m_deadline) >= 0) { m_result = AVDTP_FAILED; m_st = DISCONNECTING; }
        break;
    case STREAMING:
        m_avdtp.adoptInbound(m_l2);
        if (!m_delaySent && m_avdtp.peerWantsDelayReports() && m_avdtp.sendDelayReport(m_delayTenthMs)) { m_delaySent = true; logf("sink: delay_report=%u", m_delayTenthMs); }
        break;
    case DISCONNECTING:
        if (!m_link.busy() && m_link.op() == BtLink::NONE && m_link.handle() == 0) { m_st = DONE; return; }
        if (m_link.op() != BtLink::DISCONNECT) m_link.startDisconnect();
        break;
    default: break;
    }
    m_l2.service(); m_avdtp.service(); m_sdpServer.service(m_l2); m_avrcp.service(m_l2);
}
