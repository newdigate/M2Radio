// Avdtp -- AVDTP v1.3 signalling: stateless PDU builders/parsers plus a small
// initiator state machine (DISCOVER..START) driven from service(), and a
// one-shot acceptor for the peer's own commands (its DISCOVER is answered
// with one audio-SRC SEP, its DelayReport accepted, anything else gets a
// General Reject).  Clean-room from the A2DP/AVDTP specs.  MIT, no heap.
//
// The initiator asks GET_ALL_CAPABILITIES (0x0C, AVDTP 1.3) of each candidate
// SEP IN THE SINK'S LIST ORDER until one advertises SBC -- the Shokz OpenMove
// lists its MPEG-1,2 SEP before its SBC one (Mac->Shokz PacketLogger reference,
// 2026-09-03), so "first audio sink" picked the wrong SEP and the session
// failed at caps.  It configures Delay Reporting (0x08) only when that SEP
// advertises it, which is what the Mac does with this headset.
#pragma once
#include <stdint.h>
#include "L2cap.h"
struct Avdtp {
    static const uint16_t PSM = 0x0019;
    enum Mode : uint8_t { MONO = 8, DUAL = 4, STEREO = 2, JOINT_STEREO = 1 };
    enum Alloc : uint8_t { SNR = 2, LOUDNESS = 1 };
    enum MsgType : uint8_t { COMMAND = 0, GENERAL_REJECT = 1, ACCEPT = 2, REJECT = 3 };
    struct SbcConfig { uint32_t rate; Mode mode; uint8_t blocks, subbands; Alloc alloc; uint8_t minBitpool, maxBitpool; };
    struct SbcCaps   { uint8_t rates, modes, blocks, subbands, alloc, minBitpool, maxBitpool;
                       bool delayReporting; };   // the SEP also advertised service category 0x08 (Delay Reporting)
    struct Sep       { uint8_t seid; bool inUse, audio, sink; };
    static uint16_t buildDiscover(uint8_t *o, uint8_t tl);
    static uint16_t buildGetCapabilities(uint8_t *o, uint8_t tl, uint8_t acpSeid);      // 0x02 (AVDTP 1.0)
    static uint16_t buildGetAllCapabilities(uint8_t *o, uint8_t tl, uint8_t acpSeid);   // 0x0C (AVDTP 1.3) -- what the initiator sends
    // delayReporting: append service category 0x08 (len 0) -- ONLY when the sink advertised it (SbcCaps::delayReporting).
    static uint16_t buildSetConfiguration(uint8_t *o, uint8_t tl, uint8_t acpSeid, uint8_t intSeid, const SbcConfig &c, bool delayReporting = false);
    static uint16_t buildOpen(uint8_t *o, uint8_t tl, uint8_t acpSeid);
    static uint16_t buildStart(uint8_t *o, uint8_t tl, uint8_t acpSeid);
    static uint16_t buildDiscoverAcceptOneSource(uint8_t *o, uint8_t hdrFromPeer); // answers a peer's DISCOVER: SEID 1, audio, SRC
    static MsgType  responseType(uint8_t hdr) { return (MsgType)(hdr & 0x03); }
    static uint8_t  rejectError(const uint8_t *p, uint16_t len);          // last byte of a REJECT
    static uint8_t  parseDiscover(const uint8_t *p, uint16_t len, Sep *out, uint8_t max);
    static bool     parseSbcCaps(const uint8_t *p, uint16_t len, SbcCaps &c);
    static void     sbcCie(const SbcConfig &c, uint8_t out[4]);
    // --- initiator, one stream ---
    enum State : uint8_t { IDLE, DISCOVERING, GETTING_CAPS, CONFIGURING, OPENING, MEDIA_CONNECTING, STARTING, STREAMING, FAILED };
    void begin(L2cap &l2, uint16_t sigLocalCid, uint16_t mediaLocalCid);
    bool start(const SbcConfig &want);        // kick off: DISCOVER on the (already OPEN) signalling channel
    void onSignalling(const uint8_t *p, uint16_t len);   // from the L2cap data callback, signalling channel (record only)
    void service();                            // main context: advance the state machine, send commands
    State state() const { return m_state; } uint8_t error() const { return m_err; } const SbcCaps &caps() const { return m_caps; }
    uint8_t acpSeid() const { return m_acp; } uint16_t mediaRemoteCid() const { return m_media ? m_media->remoteCid : 0; }
    uint16_t mediaMtu() const { return m_media ? m_media->mtuOut : 0; }
    bool truncated() const { return m_truncated; }   // an incoming PDU exceeded the m_rsp buffer and was cut down
    uint16_t peerDelayTenthMs() const { return m_peerDelay; }   // last DelayReport the sink sent us (units of 0.1 ms); 0 if none
private:
    L2cap *m_l2 = nullptr; L2cap::Channel *m_sig = nullptr, *m_media = nullptr; uint16_t m_sigCid = 0, m_mediaCid = 0;
    State m_state = IDLE; uint8_t m_tl = 1, m_acp = 0, m_err = 0; SbcConfig m_want; SbcCaps m_caps;
    // 172: big enough for a GET_CAPABILITIES reply carrying every AVDTP service category, not just MEDIA_CODEC.
    volatile bool m_rspSeen = false; uint8_t m_rsp[172]; uint16_t m_rspLen = 0; bool m_peerDiscover = false; uint8_t m_peerHdr = 0;
    volatile uint16_t m_peerDelay = 0;
    bool m_truncated = false;   // an incoming PDU exceeded sizeof m_rsp and was cut down; diagnosis only
    bool m_kickoff = false;     // start() sets this; service() retries the initial DISCOVER until it actually enqueues
    // Candidate SEPs (audio, SNK, not in use) in the sink's DISCOVER order; m_candIdx is the one whose caps are being read.
    uint8_t m_cand[4]; uint8_t m_nCand = 0, m_candIdx = 0;
    // Peer commands recorded by onSignalling(), answered from service() (retried while the TXQ is full):
    // a DelayReport (0x0D) gets an ACCEPT; any other command we do not implement gets a General Reject.
    bool m_peerDelayRpt = false; uint8_t m_peerDelayHdr = 0;
    bool m_peerReject = false;   uint8_t m_peerRejHdr = 0, m_peerRejSig = 0;
    // Returns false if L2cap's TXQ was full and the command was NOT queued -- callers must not advance
    // state on a false return (BT-1's stuck-credit disease: advancing while nothing reached the wire hangs forever).
    bool send(const uint8_t *b, uint16_t n) { return m_l2->send(m_sig->remoteCid, b, n); }
};
