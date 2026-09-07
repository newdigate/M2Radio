// A2dpSource -- ONE A2DP bring-up ATTEMPT in either direction (NEW-34 piece 2).
// Given a Target (a PAGE, an INQUIRY-by-name, or an already-up INBOUND link), tick()
// walks it LINKING -> PAIRING -> L2 -> SDP -> AVDTP_WAIT -> AVDTP -> STREAMING, or ends
// in a failure Result after tearing the link down.  A link loss in any state resets
// L2cap + Avdtp and ends LOST.  The inbound path skips the SDP client, waits for the
// peer to open AVDTP, and ADOPTS the peer's SBC config (sbcParams() reflects it).
// The bonded-candidate WALK and the lost-peer retry policy live in BtSession, not here.
// Every wait is a state with a deadline advanced by one tick(now) per loop pass.  MIT.
#pragma once
#include <stdint.h>
#include "Hci.h"
#include "HciIo.h"
#include "L2cap.h"
#include "BtLink.h"
#include "Sdp.h"
#include "SdpServer.h"
#include "Avrcp.h"
#include "Avdtp.h"
#include "Sbc.h"
#include "BondTable.h"
class A2dpSource {
public:
    enum Result : uint8_t { OK = 0, CONNECT_FAILED, PAIR_FAILED, L2CAP_FAILED, AVDTP_FAILED, LOST, STOPPED, PENDING };
    static const char *resultName(Result r);
    // What to attempt: page a known address, inquire by name then page the hit, or take over an
    // already-up incoming link (BtLink accepted the page).  psrm/clk/clkValid steer the page; name
    // survives a rejected-bond re-pair; attempts is the page count; nameFilter filters an inquiry.
    struct Target { enum Kind : uint8_t { PAGE, INQUIRY, INBOUND } kind;
                    uint8_t bd[6]; uint8_t psrm; uint16_t clk; bool clkValid; char name[BondTable::NAME_LEN + 1];
                    uint8_t attempts; const char *nameFilter; };
    enum St : uint8_t { IDLE, LINKING, PAIRING, L2, SDP, AVDTP_WAIT, AVDTP, STREAMING, DISCONNECTING, DONE };
    A2dpSource(Hci &hci, HciIo &io) : m_hci(hci), m_l2(io), m_link(hci) {}
    void setLog(BtLink::LogFn fn, void *ctx) { m_link.setLog(fn, ctx); m_log = fn; m_logCtx = ctx; }
    // Bonded devices: forwarded to BtLink (Link_Key_Request answered from the table, notifications
    // upserted, rejected keys erased).  The host persists the table after an attempt returns.  The
    // bonded-candidate WALK that used to live here has moved to BtSession (NEW-34 piece 2).
    void setBonds(BondTable *t) { m_bonds = t; m_link.setBonds(t); }
    BondTable *bonds() { return m_bonds; }
    void setPin(const char *pin4) { m_link.setPin(pin4); }
    void setLegacyPin(bool v)     { m_link.setLegacyPin(v); }
    const Avrcp &avrcp() const { return m_avrcp; }   // NEW-34 piece 3: the minimal AVRCP target on the peer's AVCTP channel
    // ---- The attempt state machine -----------------------------------------------------------
    void begin(uint32_t now, uint8_t aclNum);        // reset the attempt machine; call once per session (with the ACL buffer count)
    bool start(const Target &t);                      // begin one attempt; false if one is already running
    void tick(uint32_t now);                          // advance it
    void stop();                                      // tear down -> FAILED(STOPPED)
    bool     busy()   const { return m_st != IDLE && m_st != DONE; }
    Result   result() const { return m_result; }
    St       state()  const { return m_st; }
    // Main context, every loop pass: answers the peer's SDP queries (SdpServer), drives L2cap
    // and the AVDTP state machine.  tick() runs these too; a caller may call either or both.
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
    // The negotiated SBC config as Sbc::Params.  Outbound: the initiator default (bitpool 53).
    // Inbound: the peer's SET_CONFIGURATION, adopted (bitpool = the peer's chosen maxBitpool).
    const Sbc::Params &sbcParams() const { return m_params; }
private:
    static void onData(void *ctx, L2cap::Channel &ch, const uint8_t *p, uint16_t len);
    void logf(const char *fmt, ...);
    void beginAvdtpConnect(uint32_t now);   // outbound: open our AVDTP signalling channel, wait for OPEN
    void adoptConfig();                     // map m_avdtp.sbcConfig() (acceptor) into m_params
    BtLink::LogFn m_log = nullptr; void *m_logCtx = nullptr; char m_lb[96];
    BondTable *m_bonds = nullptr;
    Hci   &m_hci;
    L2cap  m_l2;
    BtLink m_link;
    Avdtp  m_avdtp;
    SdpServer m_sdpServer;
    Avrcp m_avrcp;
    volatile bool     m_sdpDone = false;
    volatile uint16_t m_sdpVer  = 0;
    Sbc::Params m_params = { Sbc::RATE_44100, Sbc::JOINT_STEREO, 16, 8, Sbc::LOUDNESS, 53 };
    // attempt state
    St m_st = IDLE; Result m_result = OK; uint32_t m_deadline = 0; uint8_t m_aclNum = 0;
    Target m_t{}; bool m_inbound = false;
    bool m_pagedFromInquiry = false;        // INQUIRY: the hit has been paged (page once, not each tick)
    bool m_opIssued = false;                // the LINKING/PAIRING link-op has been launched (PREPARE may delay it)
    L2cap::Channel *m_sdpChan = nullptr;    // outbound SDP client channel
    L2cap::Channel *m_sigChan = nullptr;    // outbound AVDTP signalling channel
    uint32_t m_avdtpWaitMs = 2000;          // inbound: how long to wait for the peer to open AVDTP before initiating
    uint32_t m_startWaitAt = 0;             // inbound: the instant we may self-START if the peer has not
    static const uint32_t START_WAIT_MS = 1000;
};
