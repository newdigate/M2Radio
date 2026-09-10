// A2dpSink -- ONE A2DP bring-up ATTEMPT as the SINK (NEW-41): the mirror of A2dpSource's inbound path with the SNK
// personality.  An accepted incoming link is taken over (start()), paired/encrypted as responder, L2cap begun with the
// SDP server publishing the AudioSink record, then the SOURCE drives AVDTP at us (Avdtp ACCEPTOR) to STREAMING; the
// media channel's PDUs go to onMedia().  SUSPEND/START toggle suspended(); a link loss resets L2cap/Avdtp and ends LOST.
// No outbound paging, no SDP client, no encoder.  Every wait is a state with a deadline.  MIT.
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
class A2dpSink {
public:
    enum Result : uint8_t { OK = 0, PAIR_FAILED, L2CAP_FAILED, AVDTP_FAILED, LOST, STOPPED, PENDING };
    static const char *resultName(Result r);
    enum St : uint8_t { IDLE, PAIRING, AVDTP_WAIT, AVDTP, STREAMING, DISCONNECTING, DONE };
    typedef void (*MediaFn)(void *ctx, const uint8_t *rtp, uint16_t len);
    // NOTE: Sdp::setRole() is PROCESS-GLOBAL -- the SDP server publishes ONE role per image, so an image
    // that constructs an A2dpSink publishes the AudioSink record for everything, A2dpSource included.
    A2dpSink(Hci &hci, HciIo &io) : m_hci(hci), m_l2(io), m_link(hci) { m_avdtp.setLocalSep(Avdtp::SEP_SINK); Sdp::setRole(Sdp::SINK); m_link.acceptUnknown(true); }
    void setLog(BtLink::LogFn fn, void *ctx) { m_link.setLog(fn, ctx); m_log = fn; m_logCtx = ctx; }
    void setBonds(BondTable *t) { m_bonds = t; m_link.setBonds(t); }
    BondTable *bonds() { return m_bonds; }
    // `name` is BORROWED, not copied (BtLink keeps the pointer and writes it to the controller in PREPARE):
    // it must outlive the whole SESSION, not just begin().  PREPARE runs at begin() and again on every pairing
    // window (BtSinkSession::openWindow, NEW-46), re-dereferencing the pointer each time -- so storage that has
    // gone away is a dangling read on every window after it, and a value set after begin() is not ignored: it
    // reaches the controller at the next window.
    void setIdentity(uint32_t cod, const char *name) { m_link.setIdentity(cod, name); }
    void onMedia(MediaFn fn, void *ctx) { m_mediaFn = fn; m_mediaCtx = ctx; }
    void begin(uint32_t now, uint8_t aclNum);         // reset; runs PREPARE (identity written there) -- and so does every pairing window after it
    bool start();                                      // take over the accepted incoming link (link().inboundUp()); false if busy/none
    void tick(uint32_t now);
    void stop();
    bool     busy()      const { return m_st != IDLE && m_st != DONE; }
    Result   result()    const { return m_result; }
    St       state()     const { return m_st; }
    bool     suspended() const { return m_avdtp.state() == Avdtp::SUSPENDED; }
    void service() { m_sdpServer.service(m_l2); m_l2.service(); m_avdtp.service(); m_avrcp.service(m_l2); }
    void onEvent(uint8_t code, const uint8_t *p, uint8_t len) { m_link.onEvent(code, p, len); m_l2.onEvent(code, p, len); }
    void onAcl(uint16_t h, const uint8_t *d, uint16_t len, uint8_t pb = L2cap::PB_FIRST) { m_l2.onAcl(h, d, len, pb); }   // forwarding pb is LOAD-BEARING (see A2dpSource)
    Hci &hci() { return m_hci; } L2cap &l2() { return m_l2; } Avdtp &avdtp() { return m_avdtp; } BtLink &link() { return m_link; } Avrcp &avrcp() { return m_avrcp; }
    uint16_t mediaCid() { return m_avdtp.mediaRemoteCid(); }
    // The peer refused our AVCTP Connection Request (result != 0).  Counted, never retried on that link:
    // a peer with no AVRCP controller would otherwise draw a CONN_REQ every few seconds for the whole stream.
    uint32_t avctpRefused() const { return m_avctpRefused; }
    // OUR AVCTP channel's local CID.  L2cap hands PEER-initiated channels CIDs from 0x0080 up
    // (L2cap::begin: "above the caller-chosen 0x0040-0x005F range"), and the caller-chosen range is used
    // by A2dpSource alone (0x0040 SDP, 0x0041 AVDTP signalling, 0x0042 media) -- the sink opened nothing
    // outbound before this, so 0x0043 can collide with neither.
    static const uint16_t AVCTP_LOCAL_CID = 0x0043;
    const Sbc::Params &sbcParams() const { return m_params; }
    // The sink's own delay report (ring latency, 0.1 ms units): sent once after START if the source configured it.
    void setDelayTenthMs(uint16_t t) { m_delayTenthMs = t; }
private:
    static void onData(void *ctx, L2cap::Channel &ch, const uint8_t *p, uint16_t len);
    void logf(const char *fmt, ...);
    void adoptConfig();
    void openAvctp();             // once, on the transition to STREAMING
    bool streamClosed();          // peer CLOSE/ABORT -> DISCONNECTING(OK); true when it fired this tick
    BtLink::LogFn m_log = nullptr; void *m_logCtx = nullptr; char m_lb[96];
    BondTable *m_bonds = nullptr; Hci &m_hci; L2cap m_l2; BtLink m_link; Avdtp m_avdtp; SdpServer m_sdpServer; Avrcp m_avrcp;
    MediaFn m_mediaFn = nullptr; void *m_mediaCtx = nullptr;
    Sbc::Params m_params = { Sbc::RATE_44100, Sbc::JOINT_STEREO, 16, 8, Sbc::LOUDNESS, 53 };
    St m_st = IDLE; Result m_result = OK; uint32_t m_deadline = 0; uint8_t m_aclNum = 0; bool m_opIssued = false;
    bool m_delaySent = false; uint16_t m_delayTenthMs = 460;      // ~46 ms: a 16-block ring at 44.1 k; the audio node updates it
    // The media channel's REMOTE cid, latched the first time Avdtp reports one and held until the attempt ends.
    // Avdtp FORGETS its media channel on a peer CLOSE/ABORT, so routing on m_avdtp.mediaRemoteCid() lets an
    // in-flight RTP packet fall through to onSignalling() -- where an RTP header parses as a peer command and
    // earns a bogus General Reject.  Channel IDENTITY is the right gate; whether the stream is LIVE is a
    // separate question, asked again before the packet is delivered.
    uint16_t m_mediaRemoteCid = 0;
    // TWO latches, and the distinction is the whole of streamClosed()'s verdict.  m_avdtpUp says the peer's
    // SET_CONFIGURATION was accepted (Avdtp left IDLE), so a RETURN to IDLE is a CLOSE/ABORT rather than a
    // channel that never started.  m_streamUp says WE reached STREAMING.  A close seen with m_avdtpUp but
    // without m_streamUp aborted the attempt before a sample ever played: that is AVDTP_FAILED, not the OK a
    // completed session earns -- m_streamUp used to latch at SET_CONFIGURATION and reported those as OK.
    bool m_avdtpUp = false;
    bool m_streamUp = false;
    // The AVCTP channel WE opened, watched only while it is WAIT_CONN: a CLOSED verdict there is the peer's
    // refusal (counted), and any other state means it was accepted, after which the pointer is dropped -- a
    // later peer DISC_REQ also lands the channel in CLOSED and must not be miscounted as a refusal.
    L2cap::Channel *m_avctpChan = nullptr; uint32_t m_avctpRefused = 0; bool m_avctpTried = false;
};
