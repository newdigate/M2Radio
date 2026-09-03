#include "A2dpSource.h"
#include <string.h>
const char *A2dpSource::resultName(Result r) {
    switch (r) { case OK: return "ok"; case CONNECT_FAILED: return "connect_failed";
        case PAIR_FAILED: return "pair_failed"; case L2CAP_FAILED: return "l2cap_failed";
        case AVDTP_FAILED: return "avdtp_failed"; } return "?";
}
void A2dpSource::onData(void *ctx, L2cap::Channel &ch, const uint8_t *p, uint16_t len) {
    A2dpSource *s = (A2dpSource *)ctx;
    if (ch.psm == Avdtp::PSM && ch.localCid == 0x0041) s->m_avdtp.onSignalling(p, len);
    else if (ch.psm == Sdp::PSM) { s->m_sdpVer = Sdp::parseAvdtpVersion(p, len); s->m_sdpDone = true; }
}
A2dpSource::Result A2dpSource::connect(const char *name, uint8_t aclNum, uint32_t (*now)(), void (*idle)()) {
    if (m_link.connect(name, now, idle) != BtLink::OK) return CONNECT_FAILED;
    if (m_link.pairAndEncrypt(now, idle) != BtLink::OK) return PAIR_FAILED;
    m_l2.begin(m_link.handle(), aclNum);
    m_l2.acceptIncoming(true);
    m_l2.onData(onData, this);
    // (the app wires hci.onAcl -> a thunk that calls this->onAcl)
    // SDP (informational; failure here does not abort AVDTP)
    L2cap::Channel *sdp = m_l2.connect(Sdp::PSM, 0x0040);
    uint32_t t0 = now();
    if (sdp) while (sdp->state != L2cap::OPEN && now() - t0 < 5000) { m_l2.service(); idle(); }
    if (sdp && sdp->state == L2cap::OPEN) {
        uint8_t q[18]; m_l2.send(sdp->remoteCid, q, Sdp::buildAudioSinkPdlRequest(q, 1));
        m_sdpDone = false; t0 = now();
        while (!m_sdpDone && now() - t0 < 5000) { m_l2.service(); idle(); }
    }
    // AVDTP DISCOVER..START on the signalling channel 0x0041, media 0x0042
    L2cap::Channel *sig = m_l2.connect(Avdtp::PSM, 0x0041);
    if (!sig) return L2CAP_FAILED;
    t0 = now();
    while (sig->state != L2cap::OPEN && now() - t0 < 5000) { m_l2.service(); idle(); }
    if (sig->state != L2cap::OPEN) return L2CAP_FAILED;
    m_avdtp.begin(m_l2, 0x0041, 0x0042);
    Avdtp::SbcConfig want = { 44100, Avdtp::JOINT_STEREO, 16, 8, Avdtp::LOUDNESS, 2, 53 };
    m_avdtp.start(want); t0 = now();
    while (m_avdtp.state() != Avdtp::STREAMING && m_avdtp.state() != Avdtp::FAILED && now() - t0 < 15000) {
        m_l2.service(); m_avdtp.service(); idle();
    }
    return m_avdtp.state() == Avdtp::STREAMING ? OK : AVDTP_FAILED;
}
