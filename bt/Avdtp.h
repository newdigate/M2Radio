// Avdtp -- AVDTP v1.3 signalling: stateless PDU builders/parsers plus a small
// initiator state machine (DISCOVER..START) driven from service(), and a
// one-shot acceptor for a peer's own DISCOVER (answers with one audio-SRC
// SEP).  Clean-room from the A2DP/AVDTP specs.  MIT, no heap.
#pragma once
#include <stdint.h>
#include "L2cap.h"
struct Avdtp {
    static const uint16_t PSM = 0x0019;
    enum Mode : uint8_t { MONO = 8, DUAL = 4, STEREO = 2, JOINT_STEREO = 1 };
    enum Alloc : uint8_t { SNR = 2, LOUDNESS = 1 };
    enum MsgType : uint8_t { COMMAND = 0, GENERAL_REJECT = 1, ACCEPT = 2, REJECT = 3 };
    struct SbcConfig { uint32_t rate; Mode mode; uint8_t blocks, subbands; Alloc alloc; uint8_t minBitpool, maxBitpool; };
    struct SbcCaps   { uint8_t rates, modes, blocks, subbands, alloc, minBitpool, maxBitpool; };
    struct Sep       { uint8_t seid; bool inUse, audio, sink; };
    static uint16_t buildDiscover(uint8_t *o, uint8_t tl);
    static uint16_t buildGetCapabilities(uint8_t *o, uint8_t tl, uint8_t acpSeid);
    static uint16_t buildSetConfiguration(uint8_t *o, uint8_t tl, uint8_t acpSeid, uint8_t intSeid, const SbcConfig &c);
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
private:
    L2cap *m_l2 = nullptr; L2cap::Channel *m_sig = nullptr, *m_media = nullptr; uint16_t m_sigCid = 0, m_mediaCid = 0;
    State m_state = IDLE; uint8_t m_tl = 1, m_acp = 0, m_err = 0; SbcConfig m_want; SbcCaps m_caps;
    volatile bool m_rspSeen = false; uint8_t m_rsp[64]; uint16_t m_rspLen = 0; bool m_peerDiscover = false; uint8_t m_peerHdr = 0;
    void send(const uint8_t *b, uint16_t n) { m_l2->send(m_sig->remoteCid, b, n); }
};
