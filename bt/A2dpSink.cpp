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
    if (media != 0) s->m_mediaRemoteCid = media;                                        // latch: Avdtp drops this on CLOSE/ABORT
    if (s->m_mediaRemoteCid != 0 && ch.remoteCid == s->m_mediaRemoteCid) {              // the media transport: RTP to the audio node
        // Delivered only while the stream is actually running: STREAMING excludes a peer SUSPEND (Avdtp
        // SUSPENDED) and a peer CLOSE/ABORT (Avdtp back at IDLE) alike.  Either way the packet stops HERE --
        // it is never offered to onSignalling(), which would answer an RTP header with a General Reject.
        if (s->m_mediaFn && s->m_st == STREAMING && s->m_avdtp.state() == Avdtp::STREAMING) s->m_mediaFn(s->m_mediaCtx, p, len);
        return;
    }
    s->m_avdtp.onSignalling(p, len);
}
// Map the config the peer SET into Sbc::Params.  Every field comes FROM the config -- the same mapping
// A2dpSource::adoptConfig() uses -- rather than being forced to what parseAcceptCfg() currently pins:
// forcing here means a widened SEP (a second rate, SNR allocation) silently plays at the old settings.
void A2dpSink::adoptConfig() {
    const Avdtp::SbcConfig &c = m_avdtp.sbcConfig();
    m_params.rate = c.rate >= 48000 ? Sbc::RATE_48000 : c.rate >= 44100 ? Sbc::RATE_44100
                  : c.rate >= 32000 ? Sbc::RATE_32000 : Sbc::RATE_16000;
    m_params.mode = c.mode == Avdtp::MONO ? Sbc::MONO : c.mode == Avdtp::DUAL ? Sbc::DUAL : c.mode == Avdtp::STEREO ? Sbc::STEREO : Sbc::JOINT_STEREO;
    m_params.alloc = c.alloc == Avdtp::SNR ? Sbc::SNR : Sbc::LOUDNESS;
    m_params.blocks = c.blocks; m_params.subbands = c.subbands; m_params.bitpool = c.maxBitpool;
    logf("sink: adopted sbc bitpool=%u mode=%u blocks=%u sub=%u", c.maxBitpool, (unsigned)m_params.mode, c.blocks, c.subbands);
}
// The peer CLOSEd (0x08) or ABORTed (0x0A) the stream.  Avdtp has NO dedicated signal for this: it accepts
// the command, drops its media channel and reverts m_state to IDLE with m_role still ACCEPTOR -- which is
// byte-for-byte the state a freshly ADOPTED signalling channel is in.  So the transition is what identifies
// it: m_avdtpUp latches once the peer's SET_CONFIGURATION was accepted (Avdtp leaves IDLE), and a return to
// IDLE after that is a close.
// WHICH ending it earns is a SECOND question, and conflating the two was a real defect: a close seen before
// the sink ever reached STREAMING (an ABORT during CONFIGURING or OPEN -- three commands before START) used
// to end the attempt result=OK and log "sink: stream closed", so BtSinkSession reported a completed session
// for one that never played a sample.  OK is reserved for a stream the source closed AFTER it ran.
bool A2dpSink::streamClosed() {
    Avdtp::State s = m_avdtp.state();
    if (s != Avdtp::IDLE && s != Avdtp::FAILED) { m_avdtpUp = true; return false; }
    if (!m_avdtpUp || s != Avdtp::IDLE) return false;
    m_avdtpUp = false; m_mediaRemoteCid = 0;
    if (!m_streamUp) { logf("sink: aborted before start"); m_result = AVDTP_FAILED; m_st = DISCONNECTING; return true; }
    logf("sink: stream closed");
    m_streamUp = false; m_result = OK; m_st = DISCONNECTING; return true;
}
// SINK-INITIATED AVCTP (bench 2026-09-09).  An iPhone streaming to us NEVER opens an AVCTP channel
// (PSM 0x0017) at our AVRCP target -- four connections over ~15 minutes, avctp=0 throughout, even after the
// Target record advertised Category 2 -- so no SetAbsoluteVolume ever arrives and the phone's volume slider
// does nothing.  Real speakers open the channel themselves once A2DP is up, and iOS waits for them to.  So
// the sink asks, ONCE, on the transition to STREAMING.  Two peers must not produce two channels: a headset
// that has already opened AVCTP at us (the Shokz opens it 1.8 s after START, the Bose likewise) is found by
// byPsm() and left alone -- the channel is an ordinary AVCTP channel either way, since Avrcp::onData matches
// on the PSM and answers on the channel's remote CID regardless of who opened it.
void A2dpSink::openAvctp() {
    if (m_avctpTried) return;
    m_avctpTried = true;
    if (m_l2.byPsm(Avrcp::PSM)) return;                       // the peer opened it first: one channel, not two
    m_avctpChan = m_l2.connect(Avrcp::PSM, AVCTP_LOCAL_CID);
    if (!m_avctpChan) { logf("sink: avctp connect unavailable"); return; }   // no free slot / CID in use
    logf("sink: avctp connect");
}
void A2dpSink::begin(uint32_t now, uint8_t aclNum) { m_aclNum = aclNum; m_st = IDLE; m_result = OK; m_link.begin(now); m_link.startPrepare(); }
bool A2dpSink::start() {
    if (busy() || !m_link.inboundUp()) return false;
    // A fresh attempt starts from a CLEAN Avdtp/Avrcp.  Only tick()'s loss branch used to reset them, so an
    // attempt that ended AVDTP_FAILED / L2CAP_FAILED / PAIR_FAILED / STOPPED left Avdtp::m_sig and m_role
    // stale -- adoptInbound()'s `if (!m_sig)` guard then skips adoption, the sink believes it is already the
    // ACCEPTOR, and the next DISCOVER is answered to a dead channel's remote cid (0x0000 once L2cap::begin()
    // has zeroed the table).  a2dpsink_test K4 is the regression.
    m_avdtp.reset(); m_avrcp.reset(); m_mediaRemoteCid = 0; m_avdtpUp = false; m_streamUp = false;
    m_avctpChan = nullptr; m_avctpTried = false; m_avctpRefused = 0;
    m_link.ackLost(); m_link.ackInboundUp(); m_delaySent = false; m_result = PENDING;
    m_opIssued = m_link.startPair(true); m_st = PAIRING; return true;
}
void A2dpSink::stop() { m_result = STOPPED; m_st = DISCONNECTING; }
void A2dpSink::tick(uint32_t now) {
    m_link.tick(now);
    if (m_st != IDLE && m_st != DONE && m_st != DISCONNECTING && m_link.lost()) {
        m_link.ackLost(); m_avdtp.reset(); m_avrcp.reset(); m_l2.reset(); m_mediaRemoteCid = 0; m_avdtpUp = false; m_streamUp = false;
        m_avctpChan = nullptr;
        logf("sink: link lost reason=0x%02X", m_link.lostReason()); m_result = LOST; m_st = DONE; return;
    }
    // The verdict on the channel we asked for, read once it stops being WAIT_CONN (see the member comment).
    if (m_avctpChan) {
        if (m_avctpChan->state == L2cap::CLOSED) { m_avctpRefused++; logf("sink: avctp refused"); m_avctpChan = nullptr; }
        else if (m_avctpChan->state != L2cap::WAIT_CONN) m_avctpChan = nullptr;
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
        // m_streamUp latches HERE -- on the sink's OWN transition to STREAMING -- and nowhere else: it is what
        // tells streamClosed() apart from an abort that never got this far.
        if (m_avdtp.started()) { m_result = OK; m_st = STREAMING; m_streamUp = true; logf("sink: streaming bitpool=%u", m_params.bitpool);
                                 openAvctp(); break; }
        if (streamClosed()) break;
        if (m_avdtp.state() == Avdtp::FAILED || (int32_t)(now - m_deadline) >= 0) { m_result = AVDTP_FAILED; m_st = DISCONNECTING; }
        break;
    case STREAMING:
        m_avdtp.adoptInbound(m_l2);
        if (streamClosed()) break;
        // A signalling failure AFTER START.  streamClosed() deliberately ignores FAILED (it fires only on the
        // return to IDLE that a CLOSE produces), and the AVDTP case's FAILED test is behind us -- so without
        // this the sink reported STREAMING with a dead Avdtp until the link itself dropped (a2dpsink_test K9).
        if (m_avdtp.state() == Avdtp::FAILED) { logf("sink: avdtp failed while streaming"); m_result = AVDTP_FAILED; m_st = DISCONNECTING; break; }
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
