#include "A2dpSource.h"
#include "HciEvents.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
const char *A2dpSource::resultName(Result r) {
    switch (r) { case OK: return "ok"; case CONNECT_FAILED: return "connect_failed";
        case PAIR_FAILED: return "pair_failed"; case L2CAP_FAILED: return "l2cap_failed";
        case AVDTP_FAILED: return "avdtp_failed"; } return "?";
}
void A2dpSource::logf(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); vsnprintf(m_lb, sizeof m_lb, fmt, ap); va_end(ap);
    if (m_log) m_log(m_logCtx, m_lb);
}
void A2dpSource::onData(void *ctx, L2cap::Channel &ch, const uint8_t *p, uint16_t len) {
    A2dpSource *s = (A2dpSource *)ctx;
    if (s->m_sdpServer.onData(ch, p, len)) return;             // the PEER's SDP query of us (its own channel): answered in service()
    if (ch.psm == Avdtp::PSM && ch.localCid == 0x0041) s->m_avdtp.onSignalling(p, len);
    else if (ch.psm == Sdp::PSM) { s->m_sdpVer = Sdp::parseAvdtpVersion(p, len); s->m_sdpDone = true; }   // OUR client channel
}
A2dpSource::Result A2dpSource::connect(const char *name, uint8_t aclNum, uint32_t (*now)(), void (*idle)()) {
    // NEW-34: bonded candidates first -- most recent first, filtered by the target name when one
    // is given (an EMPTY stored name is a wildcard: a nameless bond costs one page, never a dead
    // slot), the first candidate paged PAGE_ATTEMPTS times and each later one once -- then today's
    // inquiry path as the fallback on every attempt (brainstorm decision 3).  Passing b.name to
    // page() is load-bearing: after a rejection the bond is erased before the new key is notified,
    // so the name page() was given is the only surviving source for the re-created bond.
    bool linked = false;
    if (m_bonds && m_bonds->count()) {
        bool first = true;
        for (uint8_t i = 0; i < m_bonds->count() && !linked; i++) {
            Bond b = m_bonds->at(i);                                     // a COPY: the ladder may reorder the table later
            if (name && name[0] && b.name[0] && !strstr(b.name, name)) continue;
            char bs[18]; hciFormatBd(b.bd, bs);
            uint8_t attempts = first ? BtLink::PAGE_ATTEMPTS : 1; first = false;
            logf("bond_try: bd=%s name=\"%s\" attempts=%u", bs, b.name, attempts);
            if (m_link.page(b.bd, b.psrm, 0, false, b.name, attempts, now, idle) == BtLink::OK) linked = true;
        }
        if (!linked) logf("bond_page=none -> inquiry");
    }
    if (!linked && m_link.connect(name, now, idle) != BtLink::OK) return CONNECT_FAILED;
    if (m_link.pairAndEncrypt(now, idle) != BtLink::OK) { m_link.disconnect(now, idle); return PAIR_FAILED; }
    m_l2.begin(m_link.handle(), aclNum);
    m_l2.acceptIncoming(true);
    m_l2.onData(onData, this);
    // (the app wires hci.onAcl -> a thunk that calls this->onAcl)
    // SDP (informational; failure here does not abort AVDTP)
    L2cap::Channel *sdp = m_l2.connect(Sdp::PSM, 0x0040);
    uint32_t t0 = now();
    if (sdp) while (sdp->state != L2cap::OPEN && now() - t0 < 5000) { service(); idle(); }
    if (sdp && sdp->state == L2cap::OPEN) {
        uint8_t q[18]; m_l2.send(sdp->remoteCid, q, Sdp::buildAudioSinkPdlRequest(q, 1));
        m_sdpDone = false; t0 = now();
        while (!m_sdpDone && now() - t0 < 5000) { service(); idle(); }
    }
    // AVDTP DISCOVER..START on the signalling channel 0x0041, media 0x0042
    L2cap::Channel *sig = m_l2.connect(Avdtp::PSM, 0x0041);
    if (!sig) { m_link.disconnect(now, idle); return L2CAP_FAILED; }
    t0 = now();
    while (sig->state != L2cap::OPEN && now() - t0 < 5000) { service(); idle(); }
    if (sig->state != L2cap::OPEN) { m_link.disconnect(now, idle); return L2CAP_FAILED; }
    m_avdtp.begin(m_l2, 0x0041, 0x0042);
    Avdtp::SbcConfig want = { 44100, Avdtp::JOINT_STEREO, 16, 8, Avdtp::LOUDNESS, 2, 53 };
    m_avdtp.start(want); t0 = now();
    while (m_avdtp.state() != Avdtp::STREAMING && m_avdtp.state() != Avdtp::FAILED && now() - t0 < 15000) {
        service(); idle();
    }
    if (m_avdtp.state() == Avdtp::STREAMING) return OK;
    m_link.disconnect(now, idle);
    return AVDTP_FAILED;
}
