// A2dpSource -- A2DP source bring-up: inquiry-by-name -> Create_Connection -> pair
// (SSP, or legacy PIN) -> encrypt -> L2cap -> SDP (AVDTP version) -> Avdtp
// DISCOVER..START.  Exposes the ready media channel for AudioOutputBluetooth.
// This is the sequence proven on silicon in BT-3 phase 2, packaged for reuse.  MIT.
#pragma once
#include <stdint.h>
#include "Hci.h"
#include "HciIo.h"
#include "L2cap.h"
#include "BtLink.h"
#include "Sdp.h"
#include "SdpServer.h"
#include "Avdtp.h"
#include "Sbc.h"
class A2dpSource {
public:
    enum Result : uint8_t { OK = 0, CONNECT_FAILED, PAIR_FAILED, L2CAP_FAILED, AVDTP_FAILED };
    static const char *resultName(Result r);
    A2dpSource(Hci &hci, HciIo &io) : m_hci(hci), m_l2(io), m_link(hci) {}
    void setLog(BtLink::LogFn fn, void *ctx) { m_link.setLog(fn, ctx); }
    void setPin(const char *pin4) { m_link.setPin(pin4); }
    void setLegacyPin(bool v)     { m_link.setLegacyPin(v); }
    // Full bring-up.  now()=millis, idle()=pump+yield.  aclNum from Read_Buffer_Size.
    // On any failure AFTER the ACL came up (pair / L2CAP / AVDTP) the link is DISCONNECTED
    // before returning, so a retry starts from a clean controller state rather than paging a
    // device we are still connected to.
    Result connect(const char *name, uint8_t aclNum, uint32_t (*now)(), void (*idle)());
    // Main context, every loop pass: answers the peer's SDP queries (SdpServer), drives L2cap
    // and the AVDTP state machine.  connect() calls it from its own wait loops too.
    void service() { m_sdpServer.service(m_l2); m_l2.service(); m_avdtp.service(); }
    // Forward from the app's Hci handlers:
    void onEvent(uint8_t code, const uint8_t *p, uint8_t len) { m_link.onEvent(code,p,len); m_l2.onEvent(code,p,len); }
    void onAcl(uint16_t h, const uint8_t *d, uint16_t len)    { m_l2.onAcl(h, d, len); }
    // For AudioOutputBluetooth + poll():
    Hci     &hci()       { return m_hci; }
    L2cap   &l2()        { return m_l2; }
    Avdtp   &avdtp()     { return m_avdtp; }
    SdpServer &sdpServer() { return m_sdpServer; }
    BtLink  &link()      { return m_link; }
    uint16_t mediaCid()  { return m_avdtp.mediaRemoteCid(); }
    uint16_t mediaMtu()  { return m_avdtp.mediaMtu(); }
    bool     started()   { return m_avdtp.state() == Avdtp::STREAMING; }
    uint16_t avdtpVersion() const { return m_sdpVer; }
    // The negotiated config as Sbc::Params (streams at bitpool 53).
    const Sbc::Params &sbcParams() const { return m_params; }
private:
    static void onData(void *ctx, L2cap::Channel &ch, const uint8_t *p, uint16_t len);
    Hci   &m_hci;
    L2cap  m_l2;
    BtLink m_link;
    Avdtp  m_avdtp;
    SdpServer m_sdpServer;
    volatile bool     m_sdpDone = false;
    volatile uint16_t m_sdpVer  = 0;
    Sbc::Params m_params = { Sbc::RATE_44100, Sbc::JOINT_STEREO, 16, 8, Sbc::LOUDNESS, 53 };
};
