#include "Avdtp.h"
#include "Sdp.h"
#include <stdio.h>
#include <string.h>
#include <vector>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
// --- a scripted L2cap for the initiator state machine (same CapIo shape as l2cap_test) ---
struct CapIo : HciIo {
    std::vector<std::vector<uint8_t> > tx; uint32_t now = 0;
    size_t write(const uint8_t *p, size_t n) override { tx.push_back(std::vector<uint8_t>(p, p + n)); return n; }
    int available() override { return 0; } int read() override { return -1; } uint32_t nowMs() override { return now; }
};
static void feed(L2cap &l, uint16_t cid, std::initializer_list<uint8_t> pl) {   // ACL as Hci hands it to onAcl(): [len][cid][payload]
    std::vector<uint8_t> v = { (uint8_t)pl.size(), 0, (uint8_t)cid, (uint8_t)(cid >> 8) }; v.insert(v.end(), pl);
    l.onAcl(0x0001, v.data(), (uint16_t)v.size()); }
// L2CAP payloads written since the last call, in order (strips the 9-byte H4+ACL+L2CAP header); clears the capture.
static std::vector<std::vector<uint8_t> > drain(CapIo &io) {
    std::vector<std::vector<uint8_t> > out;
    for (auto &p : io.tx) if (p.size() > 9) out.push_back(std::vector<uint8_t>(p.begin() + 9, p.end()));
    io.tx.clear(); return out; }
static bool eq(const std::vector<uint8_t> &v, std::initializer_list<uint8_t> want) { return v == std::vector<uint8_t>(want); }
static void onData(void *ctx, L2cap::Channel &ch, const uint8_t *p, uint16_t len) {
    if (ch.psm == Avdtp::PSM && ch.localCid == 0x0041) ((Avdtp *)ctx)->onSignalling(p, len); }
// Bring the signalling channel 0x0041 to OPEN against a scripted peer (CONN_RSP, its CFG_REQ, its CFG_RSP), start the initiator.
static void openSignalling(CapIo &io, L2cap &l, Avdtp &a) {
    l.begin(0x0001, 100); l.acceptIncoming(true); l.onData(onData, &a);   // ample ACL credits: this script returns none
    CHECK(l.connect(Avdtp::PSM, 0x0041) != nullptr); l.service();
    feed(l, 0x0001, { 0x03, 0x10, 0x08, 0x00, 0x44, 0x0E, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00 });   // CONN_RSP dcid=0x0E44 scid=0x0041 ok
    l.service();
    feed(l, 0x0001, { 0x04, 0x0A, 0x08, 0x00, 0x41, 0x00, 0x00, 0x00, 0x01, 0x02, 0x7F, 0x03 });   // peer CFG_REQ -> our 0x0041, MTU 895
    feed(l, 0x0001, { 0x05, 0x11, 0x06, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00 });               // peer CFG_RSP ok (SCID = ours)
    l.service();
    CHECK(l.byLocal(0x0041)->state == L2cap::OPEN);
    a.begin(l, 0x0041, 0x0042);
    Avdtp::SbcConfig want = { 44100, Avdtp::JOINT_STEREO, 16, 8, Avdtp::LOUDNESS, 2, 53 };
    CHECK(a.start(want)); drain(io);
}
static void tick(L2cap &l, Avdtp &a) { l.service(); a.service(); l.service(); }
int main() {
    {   // 1. SDP: the AudioSink/ProtocolDescriptorList request is the exact 18 bytes proven on three peers
        uint8_t b[32]; uint16_t n = Sdp::buildAudioSinkPdlRequest(b, 0x0001);
        static const uint8_t want[18] = { 0x06, 0x00,0x01, 0x00,0x0D, 0x35,0x03,0x19,0x11,0x0B, 0x03,0xF0, 0x35,0x03,0x09,0x00,0x04, 0x00 };
        CHECK(n == 18 && memcmp(b, want, 18) == 0);
        // and the response seen on the wire yields AVDTP v1.3
        static const uint8_t rsp[39] = { 0x07,0x00,0x01,0x00,0x1E,0x00,0x1B,0x36,0x00,0x18,0x36,0x00,0x15,0x09,0x00,0x04,0x35,0x10,
                                         0x35,0x06,0x19,0x01,0x00,0x09,0x00,0x19, 0x35,0x06,0x19,0x00,0x19,0x09,0x01,0x03, 0x00 };
        CHECK(Sdp::parseAvdtpVersion(rsp, 39) == 0x0103);
    }
    {   // 2. AVDTP command encodings (single packet, our transaction labels)
        uint8_t b[32]; uint16_t n;
        n = Avdtp::buildDiscover(b, 1);                       CHECK(n == 2 && b[0] == 0x10 && b[1] == 0x01);
        n = Avdtp::buildGetCapabilities(b, 2, 1);             CHECK(n == 3 && b[0] == 0x20 && b[1] == 0x02 && b[2] == (1 << 2));
        n = Avdtp::buildGetAllCapabilities(b, 2, 1);          CHECK(n == 3 && b[0] == 0x20 && b[1] == 0x0C && b[2] == (1 << 2));   // AVDTP 1.3 GET_ALL_CAPABILITIES (what the Mac sends)
        Avdtp::SbcConfig cfg = { 44100, Avdtp::JOINT_STEREO, 16, 8, Avdtp::LOUDNESS, 2, 53 };
        n = Avdtp::buildSetConfiguration(b, 3, /*acp seid*/ 1, /*int seid*/ 1, cfg);
        // hdr, sig, ACP SEID<<2, INT SEID<<2, [cat 1 media transport, len 0], [cat 7 media codec, len 6: media type audio<<4, codec SBC=0, cie 21 15 02 35]
        static const uint8_t want[14] = { 0x30, 0x03, 1 << 2, 1 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x35 };
        CHECK(n == 14 && memcmp(b, want, 14) == 0);
        // With delay reporting: the Mac's SET_CONFIGURATION to the Shokz appends service category 0x08 (len 0) --
        // PacketLogger reference 2026-09-03: 40 03 04 08 01 00 07 06 00 00 21 15 02 35 08 00.
        n = Avdtp::buildSetConfiguration(b, 4, 1, 2, cfg, true);
        static const uint8_t wantDr[16] = { 0x40, 0x03, 1 << 2, 2 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x35, 0x08, 0x00 };
        CHECK(n == 16 && memcmp(b, wantDr, 16) == 0);
        n = Avdtp::buildOpen(b, 4, 1);                        CHECK(n == 3 && b[0] == 0x40 && b[1] == 0x06 && b[2] == (1 << 2));
        n = Avdtp::buildStart(b, 5, 1);                       CHECK(n == 3 && b[0] == 0x50 && b[1] == 0x07 && b[2] == (1 << 2));
    }
    {   // 3. Parsing: a Discover accept with two SNK SEPs, and SBC capabilities out of GET_CAPABILITIES
        static const uint8_t disc[6] = { 0x12, 0x01, 0x04, 0x08, 0x08, 0x08 };
        Avdtp::Sep seps[4]; uint8_t n = Avdtp::parseDiscover(disc, 6, seps, 4);
        CHECK(n == 2 && seps[0].seid == 1 && seps[0].sink && seps[0].audio && seps[1].seid == 2);
        static const uint8_t caps[12] = { 0x22, 0x02, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0x35 };  // all rates/modes, all blocks/subbands/alloc, bitpool 2..53
        Avdtp::SbcCaps c; CHECK(Avdtp::parseSbcCaps(caps, 12, c));
        CHECK(c.rates == 0xF && c.modes == 0xF && c.blocks == 0xF && c.subbands == 0x3 && c.alloc == 0x3 && c.minBitpool == 2 && c.maxBitpool == 53);
        CHECK(!c.delayReporting);                              // this sink advertised no category 0x08
        // The Shokz's GET_ALL_CAPABILITIES reply for its SBC SEP (PacketLogger 2026-09-03): media transport, SBC,
        // content protection SCMS-T, DELAY REPORTING (0x08).  SBC fields read the same; delay reporting is seen.
        static const uint8_t shokz[18] = { 0x32, 0x0C, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0x35, 0x04, 0x02, 0x02, 0x00, 0x08, 0x00 };
        Avdtp::SbcCaps c2; CHECK(Avdtp::parseSbcCaps(shokz, 18, c2));
        CHECK(c2.rates == 0xF && c2.minBitpool == 2 && c2.maxBitpool == 53 && c2.delayReporting);
        // Its MPEG-1,2 SEP reply carries codec 0x01 under category 0x07 -- NOT SBC: must be refused, not misread.
        static const uint8_t mpeg[18] = { 0x22, 0x0C, 0x01, 0x00, 0x07, 0x06, 0x00, 0x01, 0x3F, 0x3F, 0xFF, 0xFE, 0x04, 0x02, 0x02, 0x00, 0x08, 0x00 };
        Avdtp::SbcCaps c3; CHECK(!Avdtp::parseSbcCaps(mpeg, 18, c3));
        static const uint8_t rej[3] = { 0x33, 0x03, 0x29 };   // SET_CONFIGURATION reject, error 0x29 (unsupported configuration)
        CHECK(Avdtp::responseType(rej[0]) == Avdtp::REJECT && Avdtp::rejectError(rej, 3) == 0x29);
    }
    {   // 4. The initiator against a Shokz-shaped sink, end to end (every peer byte from the Mac->Shokz PacketLogger
        // reference, 2026-09-03): DISCOVER lists the MPEG SEP (SEID 2) BEFORE the SBC one (SEID 1); GET_ALL_CAPABILITIES
        // is the capability command; the sink advertises delay reporting, so SET_CONFIGURATION configures it; the sink
        // then sends its own DelayReport COMMAND before answering OPEN, and expects an ACCEPT.
        CapIo io; L2cap l(io); Avdtp a; openSignalling(io, l, a);
        tick(l, a); auto o = drain(io);
        CHECK(o.size() == 1 && eq(o[0], { 0x10, 0x01 }));                                            // DISCOVER, tl 1
        a.onSignalling(std::vector<uint8_t>{ 0x12, 0x01, 0x08, 0x08, 0x04, 0x08 }.data(), 6);      // accept: SEID 2 (audio SNK), SEID 1 (audio SNK)
        tick(l, a); o = drain(io);
        CHECK(o.size() == 1 && eq(o[0], { 0x20, 0x0C, 0x08 }));                                      // GET_ALL_CAPABILITIES SEID 2 first (list order)
        CHECK(a.state() == Avdtp::GETTING_CAPS);
        static const uint8_t mpeg[18] = { 0x22, 0x0C, 0x01, 0x00, 0x07, 0x06, 0x00, 0x01, 0x3F, 0x3F, 0xFF, 0xFE, 0x04, 0x02, 0x02, 0x00, 0x08, 0x00 };
        a.onSignalling(mpeg, 18); tick(l, a); o = drain(io);
        CHECK(o.size() == 1 && eq(o[0], { 0x30, 0x0C, 0x04 }));                                      // not SBC -> try the NEXT SEP (SEID 1), not FAILED
        CHECK(a.state() == Avdtp::GETTING_CAPS);
        static const uint8_t sbc[18] = { 0x32, 0x0C, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0x35, 0x04, 0x02, 0x02, 0x00, 0x08, 0x00 };
        a.onSignalling(sbc, 18); tick(l, a); o = drain(io);
        CHECK(o.size() == 1 && eq(o[0], { 0x40, 0x03, 1 << 2, 1 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x35, 0x08, 0x00 }));
        CHECK(a.acpSeid() == 1 && a.caps().delayReporting && a.state() == Avdtp::CONFIGURING);
        a.onSignalling(std::vector<uint8_t>{ 0x42, 0x03 }.data(), 2); tick(l, a); o = drain(io);
        CHECK(o.size() == 1 && eq(o[0], { 0x50, 0x06, 1 << 2 }));                                     // OPEN
        // The sink's DelayReport COMMAND (its own tl=1, 200.0 ms) arrives BEFORE the OPEN response: accept it, stay in OPENING.
        a.onSignalling(std::vector<uint8_t>{ 0x10, 0x0D, 2 << 2, 0x07, 0xD0 }.data(), 5); tick(l, a); o = drain(io);
        CHECK(o.size() == 1 && eq(o[0], { 0x12, 0x0D }));                                             // DelayReport ACCEPT, peer's tl
        CHECK(a.state() == Avdtp::OPENING); CHECK(a.peerDelayTenthMs() == 2000);
        a.onSignalling(std::vector<uint8_t>{ 0x52, 0x06 }.data(), 2); tick(l, a); o = drain(io);   // OPEN accepted -> media channel
        CHECK(a.state() == Avdtp::MEDIA_CONNECTING);
        CHECK(o.size() == 1 && o[0].size() == 8 && o[0][0] == 0x02 && o[0][4] == 0x19 && o[0][6] == 0x42);   // CONN_REQ psm 0x0019 scid 0x0042
        uint8_t connId = (o.size() == 1 && o[0].size() == 8) ? o[0][1] : 0;                                    // (fail, never crash, pre-fix)
        feed(l, 0x0001, { 0x03, connId, 0x08, 0x00, 0xC5, 0x0E, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00 });         // CONN_RSP dcid=0x0EC5
        tick(l, a); drain(io);
        feed(l, 0x0001, { 0x04, 0x0E, 0x08, 0x00, 0x42, 0x00, 0x00, 0x00, 0x01, 0x02, 0x7F, 0x03 });          // peer CFG_REQ MTU 895
        feed(l, 0x0001, { 0x05, 0x12, 0x06, 0x00, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00 });                      // peer CFG_RSP ok
        tick(l, a); o = drain(io);
        bool sawStart = false; for (auto &v : o) if (eq(v, { 0x60, 0x07, 1 << 2 })) sawStart = true;
        CHECK(sawStart); CHECK(a.state() == Avdtp::STARTING);
        a.onSignalling(std::vector<uint8_t>{ 0x62, 0x07 }.data(), 2); tick(l, a);
        CHECK(a.state() == Avdtp::STREAMING); CHECK(a.mediaMtu() == 895); CHECK(a.mediaRemoteCid() == 0x0EC5);
    }
    {   // 5. No SEP advertises SBC: every candidate is asked, then FAILED 0xFE -- not FAILED on the first non-SBC reply.
        CapIo io; L2cap l(io); Avdtp a; openSignalling(io, l, a); tick(l, a); drain(io);
        a.onSignalling(std::vector<uint8_t>{ 0x12, 0x01, 0x08, 0x08, 0x04, 0x08 }.data(), 6); tick(l, a); drain(io);
        static const uint8_t mpeg2[18] = { 0x22, 0x0C, 0x01, 0x00, 0x07, 0x06, 0x00, 0x01, 0x3F, 0x3F, 0xFF, 0xFE, 0x04, 0x02, 0x02, 0x00, 0x08, 0x00 };
        a.onSignalling(mpeg2, 18); tick(l, a); auto o = drain(io);
        CHECK(o.size() == 1 && eq(o[0], { 0x30, 0x0C, 0x04 }));
        static const uint8_t mpeg3[18] = { 0x32, 0x0C, 0x01, 0x00, 0x07, 0x06, 0x00, 0x01, 0x3F, 0x3F, 0xFF, 0xFE, 0x04, 0x02, 0x02, 0x00, 0x08, 0x00 };
        a.onSignalling(mpeg3, 18); tick(l, a);
        CHECK(a.state() == Avdtp::FAILED && a.error() == 0xFE);
    }
    {   // 6. A peer COMMAND we do not implement (GET_CONFIGURATION 0x04) gets a General Reject (message type 0x01, same tl)
        // instead of silence -- a peer waiting on it would otherwise time out its whole session.  A peer DISCOVER is still
        // answered with our one audio-SRC SEP, and neither touches the initiator's own state.
        CapIo io; L2cap l(io); Avdtp a; openSignalling(io, l, a); tick(l, a); drain(io);
        a.onSignalling(std::vector<uint8_t>{ 0x30, 0x04, 1 << 2 }.data(), 3); tick(l, a); auto o = drain(io);
        CHECK(o.size() == 1 && eq(o[0], { 0x31, 0x04 }));
        a.onSignalling(std::vector<uint8_t>{ 0x40, 0x01 }.data(), 2); tick(l, a); o = drain(io);
        CHECK(o.size() == 1 && eq(o[0], { 0x42, 0x01, 1 << 2, 0x00 }));
        CHECK(a.state() == Avdtp::DISCOVERING);
    }
    {   // 7. CONTROL -- a fake-peer-shaped sink (one SBC SEP, no delay reporting): SET_CONFIGURATION is the SAME 14 bytes
        // as before this change, so the ESP32 / [avdtp]-gate path is untouched by the delay-reporting logic.
        CapIo io; L2cap l(io); Avdtp a; openSignalling(io, l, a); tick(l, a); drain(io);
        a.onSignalling(std::vector<uint8_t>{ 0x12, 0x01, 1 << 2, 0x08 }.data(), 4); tick(l, a); auto o = drain(io);
        CHECK(o.size() == 1 && eq(o[0], { 0x20, 0x0C, 1 << 2 }));
        static const uint8_t caps[12] = { 0x22, 0x0C, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0x35 };
        a.onSignalling(caps, 12); tick(l, a); o = drain(io);
        CHECK(o.size() == 1 && eq(o[0], { 0x30, 0x03, 1 << 2, 1 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x35 }));
        CHECK(!a.caps().delayReporting);
    }
    printf("avdtp_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
