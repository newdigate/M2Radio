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
    enum State : uint8_t { IDLE, DISCOVERING, GETTING_CAPS, CONFIGURING, OPENING, MEDIA_CONNECTING, STARTING, STREAMING, SUSPENDED, FAILED };
    enum Role : uint8_t { RNONE, INITIATOR, ACCEPTOR };
    // Which endpoint WE are (the one SEP the acceptor advertises): SRC (the A2DP source, default) or SNK (NEW-41).
    enum LocalSep : uint8_t { SEP_SOURCE, SEP_SINK };
    void setLocalSep(LocalSep s) { m_localSep = s; }
    bool peerWantsDelayReports() const { return m_peerDelayCfg; }   // the peer's SET_CONFIGURATION carried category 0x08
    // Acceptor with delay reporting configured: send a DelayReport (0.1 ms units) for our SEP.  false if not STREAMING /
    // not configured for it / TXQ full.  The source's ACCEPT is consumed by onSignalling (it matches m_tl) and ignored.
    bool sendDelayReport(uint16_t tenthMs);
    Role role() const { return m_role; }
    void reset();                                        // back to IDLE, RNONE, media released
    void adoptInbound(L2cap &l);                         // find our peer-initiated signalling channel (and later media) from L2cap
    bool started() const { return m_state == STREAMING; }
    const SbcConfig &sbcConfig() const { return m_acceptCfg; }   // the config the peer SET (acceptor); valid once configChanged() fired
    bool configChanged() { bool c = m_cfgChanged; m_cfgChanged = false; return c; }   // one-shot: the app restarts the encoder on true
    // The peer's OPEN completed and its media channel is OPEN, but no START has arrived yet: the attempt
    // layer (A2dpSource, which owns a clock) may call startSelf() after its own START_WAIT deadline.
    bool mediaReady() const { return m_role == ACCEPTOR && m_state == OPENING && m_media && m_media->state == L2cap::OPEN; }
    void startSelf();                                    // we (acceptor) issue START ourselves; moves to STARTING
    void begin(L2cap &l2, uint16_t sigLocalCid, uint16_t mediaLocalCid);
    bool start(const SbcConfig &want);        // kick off: DISCOVER on the (already OPEN) signalling channel
    // The sink's AVDTP version from its SDP AudioSink record (0 = unknown).  < 1.3 asks GET_CAPABILITIES (0x02) instead
    // of GET_ALL_CAPABILITIES (0x0C); either way a General Reject of 0x0C falls back to 0x02 (Bose Mini SoundLink, 2026-09-08).
    void setPeerVersion(uint16_t v) { m_peerVer = v; }
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
    uint16_t m_peerVer = 0; uint8_t m_capSig = 0x0C;   // which capability command this attempt uses (see setPeerVersion)
    uint16_t buildCaps(uint8_t *o, uint8_t tl, uint8_t seid) { return m_capSig == 0x02 ? buildGetCapabilities(o, tl, seid) : buildGetAllCapabilities(o, tl, seid); }
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
    Role m_role = RNONE;
    SbcConfig m_acceptCfg = { 44100, JOINT_STEREO, 16, 8, LOUDNESS, 2, 53 };
    bool m_cfgChanged = false;
    LocalSep m_localSep = SEP_SOURCE;   // identity: set once by the app, NOT cleared by begin()/reset()
    bool m_peerDelayCfg = false;        // the peer's SET_CONFIGURATION carried service category 0x08
    bool m_delayRptOut = false;         // a DelayReport of ours is outstanding: its ACCEPT is consumed in service()
    // peer-command recording for the FULL acceptor (beyond m_peerDiscover/m_peerDelayRpt/m_peerReject):
    bool m_peerCaps = false;    uint8_t m_peerCapsHdr = 0, m_peerCapsSeid = 0, m_peerCapsSig = 0;
    bool m_peerSetCfg = false;  uint8_t m_peerSetHdr = 0; uint8_t m_peerSetPl[20]; uint16_t m_peerSetLen = 0;
    bool m_peerOpen = false;    uint8_t m_peerOpenHdr = 0, m_peerOpenSeid = 0;
    bool m_peerStart = false;   uint8_t m_peerStartHdr = 0;
    bool m_peerSuspend = false; uint8_t m_peerSuspendHdr = 0;
    bool m_peerClose = false;   uint8_t m_peerCloseHdr = 0, m_peerCloseSig = 0;
    static const uint8_t OUR_SEID = 1;
    uint16_t buildCapsAccept(uint8_t *o, uint8_t hdr, uint8_t sig);
    bool parseAcceptCfg(const uint8_t *p, uint16_t len, SbcConfig &c, uint8_t &badCat);
    // Returns false if L2cap's TXQ was full and the command was NOT queued -- callers must not advance
    // state on a false return (BT-1's stuck-credit disease: advancing while nothing reached the wire hangs forever).
    bool send(const uint8_t *b, uint16_t n) { return m_l2->send(m_sig->remoteCid, b, n); }
};
