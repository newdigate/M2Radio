// L2cap -- basic-mode L2CAP over one ACL link: the signalling channel, up to
// MAX_CHANNELS connection-oriented channels, ACL demux by CID, and ACL credit
// accounting from Number_Of_Completed_Packets.  Pure C++, no heap.  RX entry
// points only RECORD; every byte is transmitted from service() -- writing to
// the transport from the RX pump bus-faults (B6, 2026-08-28).  MIT, clean-room.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "HciIo.h"
class L2cap {
public:
    // Our SDP client, AVDTP signalling, AVDTP media -- PLUS room for peer-initiated
    // channels: both headsets open a reverse SDP channel at us on AVDTP contact
    // (BT-2 transcript 2026-08-29), and the Shokz opens two in the Mac reference.
    // With three slots the media connect() found none and AVDTP failed 0xFD.
    static const uint8_t MAX_CHANNELS = 5;
    static const uint8_t MAX_OPTS = 32;
    enum State : uint8_t { FREE, WAIT_CONN, CONFIG, OPEN, CLOSED };
    struct Channel {
        State state; uint16_t psm, localCid, remoteCid, mtuOut, mtuIn;
        bool cfgReqSent, cfgRspRcvd, cfgReqSeen, cfgRspSent; uint8_t cfgReqId, optLen; uint8_t opts[MAX_OPTS];
        bool peerInitiated;
    };
    typedef void (*DataFn)(void *ctx, Channel &ch, const uint8_t *payload, uint16_t len);
    explicit L2cap(HciIo &io) : m_io(io) {}
    void begin(uint16_t aclHandle, uint8_t aclCredits, uint16_t aclMax = 1021);
    void onData(DataFn fn, void *ctx) { m_onData = fn; m_dataCtx = ctx; }
    // Optional raw-ACL trace: fires for every inbound ACL (before demux) and every
    // outbound ACL, handing over the L2CAP PDU (2-byte length, CID, payload) plus
    // the handle.  Diagnostic only -- off unless set.  Arduino-free (callback).
    typedef void (*TraceFn)(void *ctx, bool out, uint16_t handle, const uint8_t *l2capPdu, uint16_t len);
    void onAclTrace(TraceFn fn, void *ctx) { m_trace = fn; m_traceCtx = ctx; }
    // --- RX (record only) ---
    void onAcl(uint16_t handle, const uint8_t *d, uint16_t len);   // Hci::AclFn payload
    void onEvent(uint8_t code, const uint8_t *p, uint8_t len);      // needs 0x13 only
    // --- main context ---
    Channel *connect(uint16_t psm, uint16_t localCid);              // sends Connection Request on service()
    bool     send(uint16_t remoteCid, const uint8_t *payload, uint16_t len); // queued, credit-paced
    void     service();
    Channel *byLocal(uint16_t cid); Channel *byRemote(uint16_t cid); Channel *byPsm(uint16_t psm);
    uint8_t  credits() const { return m_credits; }
    uint32_t dropped() const { return m_dropped; }
    void     acceptIncoming(bool yes) { m_accept = yes; }          // peer-initiated channels (answered with our next free CID)
    // Restrict which PSMs a peer-initiated CONN_REQ may open (in addition to acceptIncoming()).  Up to
    // two.  With NONE added, acceptIncoming() accepts any PSM (today's behaviour, unchanged).  With any
    // added, a CONN_REQ for a PSM not in the list is refused with result 0x0002 (PSM not supported) --
    // an AVCTP (0x0017/AVRCP, piece 3) channel from a headset lands here rather than consuming a slot.
    void allowPsm(uint16_t psm) { if (m_nAllow < 2) m_allow[m_nAllow++] = psm; }
    // Drop every channel and empty the tx queue -- used between reconnect attempts (== begin(0,0)).
    void reset() { begin(0, 0, m_aclMax); m_nAllow = 0; }
    // Iterate peer-initiated OPEN channels of a PSM in slot order: nextInbound(psm, nullptr) returns the
    // first, nextInbound(psm, prev) the next, nullptr at the end.  The AVDTP acceptor uses it to find the
    // signalling channel the headset opened (first) and, later, the media channel (next).
    const Channel *nextInbound(uint16_t psm, const Channel *after) const;
    // Running minimum ACL credit seen since resetCreditsMin() (piece 4's air-link-starvation floor).
    uint8_t  creditsMin() const { return m_creditsMin; }
    void     resetCreditsMin() { m_creditsMin = m_credits; }
    // Credit-leak instrument (NEW-34 piece 4).  pktsSent == ACL packets written (one per credit consumed);
    // creditsReturned == credits summed from Number_Of_Completed_Packets; by construction
    // credits == maxCredits - (pktsSent - creditsReturned), so pktsSent-creditsReturned is the outstanding count.
    // starves counts entries into "credits==0 with the TXQ non-empty"; starveMaxMs is the longest such stretch --
    // a lost NCP never recovers, so starveMaxMs grows without bound (the host-visible fingerprint vs backpressure,
    // which recovers in ms).  clampHits counts when the NCP handler discards credit above maxCredits (a controller
    // double-count).  tickClock(nowMs) supplies the ms reference for starveMaxMs; call it once per service() pass.
    uint32_t pktsSent()        const { return m_pktsSent; }
    uint32_t creditsReturned() const { return m_creditsReturned; }
    uint32_t starves()         const { return m_starves; }
    uint32_t starveMaxMs()     const { return m_starveMaxMs; }
    uint32_t clampHits()       const { return m_clampHits; }
    void     tickClock(uint32_t nowMs) { m_nowMs = nowMs; }
    void     resetCreditStats() { m_creditsMin = m_credits; m_pktsSent = 0; m_creditsReturned = 0;
                                  m_starves = 0; m_starveMaxMs = 0; m_clampHits = 0; m_starveSince = 0; m_starving = false; }
    // Largest L2CAP payload send() will accept (== one Tx buffer).  A larger
    // payload is DROPPED, not fragmented -- this is basic mode with one ACL
    // packet per SDU -- so a media producer MUST cap its packet at this value.
    // ★ 700 is the WORKING size measured on silicon 2026-09-03: a 5-SBC-frame
    // A2DP media packet (RTP 13 + 5*119 = 608 B, 612 B ACL) streams to the ESP32
    // sink, while an 8-frame / 965 B packet (969 B ACL) is not carried by the
    // BR/EDR link -- it stalls after the partial ramp-up packets.  The controller's
    // ACL data limit sits between the two; do not raise this without confirming
    // the link (and adding ACL fragmentation) or media will wedge.
    static const uint16_t MAX_PAYLOAD = 700;
    // The MTU we advertise in OUR Config Request = the largest SDU the peer may send us.
    // 1004 is the value the Mac's A2DP source negotiates with the Shokz (PacketLogger
    // reference 2026-09-03: option 01 02 EC 03), and it fits this stack's RX path, which
    // reassembles nothing: an SDU must arrive in ONE ACL packet, and the IW416 reports
    // acl_len=1021.  The Shokz never answered our DISCOVER while our Config Request
    // carried NO options; the compliant reference carries exactly this one.
    static const uint16_t RX_MTU = 1004;
private:
    struct Tx { uint16_t cid; uint16_t len; uint8_t buf[MAX_PAYLOAD]; };
    static const uint8_t TXQ = 8;
    bool sig(const uint8_t *cmd, uint16_t len);                     // queue a signalling command; false if the txq is full
    void handleSig(const uint8_t *d, uint16_t len);
    HciIo &m_io; uint16_t m_handle, m_aclMax; uint8_t m_credits, m_maxCredits; bool m_accept;
    uint16_t m_allow[2] = {0, 0}; uint8_t m_nAllow = 0;
    uint8_t  m_creditsMin = 0;
    uint32_t m_pktsSent = 0, m_creditsReturned = 0, m_starves = 0, m_starveMaxMs = 0, m_clampHits = 0;
    uint32_t m_nowMs = 0, m_starveSince = 0; bool m_starving = false;
    Channel m_ch[MAX_CHANNELS]; uint8_t m_nextId; uint16_t m_nextCid;
    Tx m_txq[TXQ]; uint8_t m_txHead, m_txCount; uint32_t m_dropped;
    // Only one request of each type is buffered between service() calls -- fine because service()
    // runs every main-loop pass; a same-type burst within one pass would drop the earlier one.
    struct Pending { bool infoReq; uint8_t infoId; uint16_t infoType; bool echoReq; uint8_t echoId;
                     bool connReq; uint8_t connId; uint16_t connPsm, connScid;
                     // CONN_RSP is computed once and SNAPSHOTTED here; the retried transmit in service() builds the
                     // frame entirely from these four fields, never from the live connId/connScid above -- a second
                     // CONN_REQ overwriting those mid-retry must not splice its identity onto the pending response.
                     bool connRspReady; uint8_t connRspId; uint16_t connRspLocal, connRspScid, connRspRes;
                     bool discReq; uint8_t discId; uint8_t discBytes[4]; } m_p;
    DataFn m_onData; void *m_dataCtx;
    TraceFn m_trace = nullptr; void *m_traceCtx = nullptr;
};
