// Host tests for A2dpSink as ONE inbound bring-up ATTEMPT as the SINK (NEW-41).  A phone pages us, pairs
// (we are the responder: the peer drives SSP and encryption), reads our AudioSink SDP record, then drives
// AVDTP at us as the INITIATOR while we accept -- DISCOVER (answered with our SNK SEP), GET_ALL_CAPABILITIES,
// SET_CONFIGURATION, OPEN, the media channel, START -- and its media PDUs reach the audio callback.  K2 covers
// SUSPEND/START (no media while suspended) and a link loss that tears the whole media path down; K3 covers a
// source that configures a rate we never advertised (48 kHz): rejected, and the attempt ends AVDTP_FAILED.
// The rig is a2dpsource_test's FakeIo/Hci scaffolding with the sink's own peer helpers.
#include "A2dpSink.h"
#include <stdio.h>
#include <string.h>
#include <vector>
#include <deque>
#include <string>
#include <functional>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
struct FakeIo : HciIo {
    std::deque<uint8_t> rx; std::vector<uint8_t> acc; uint32_t now = 0;
    std::vector<std::pair<uint16_t, std::vector<uint8_t> > > cmds;
    std::vector<std::vector<uint8_t> > aclOut;                 // outgoing L2CAP PDUs (len(2)+cid(2)+payload), in order
    std::function<void(FakeIo &, uint16_t, const std::vector<uint8_t> &)> onCmd;
    size_t write(const uint8_t *p, size_t n) override { acc.insert(acc.end(), p, p + n); parse(); return n; }
    void parse() {
        for (;;) {
            if (acc.size() >= 4 && acc[0] == 0x01) {          // HCI command: [01][op lo][op hi][plen][params]
                uint8_t plen = acc[3]; if (acc.size() < 4u + plen) break;
                uint16_t op = (uint16_t)(acc[1] | (acc[2] << 8)); std::vector<uint8_t> prm(acc.begin() + 4, acc.begin() + 4 + plen);
                acc.erase(acc.begin(), acc.begin() + 4 + plen); cmds.push_back(std::make_pair(op, prm)); if (onCmd) onCmd(*this, op, prm);
            } else if (acc.size() >= 5 && acc[0] == 0x02) {   // ACL out: [02][handle lo][handle hi][alen lo][alen hi][L2CAP PDU]
                uint16_t dlen = (uint16_t)(acc[3] | (acc[4] << 8)); if (acc.size() < 5u + dlen) break;
                aclOut.push_back(std::vector<uint8_t>(acc.begin() + 5, acc.begin() + 5 + dlen));
                acc.erase(acc.begin(), acc.begin() + 5 + dlen);
                // The controller returns the ACL buffer at once: without Number_Of_Completed_Packets the
                // host runs out of credits after aclNum packets and every later reply is silently queued.
                ev(0x13, { 0x01, 0x01, 0x00, 0x01, 0x00 });
            } else break;
        }
    }
    int available() override { return (int)rx.size(); }
    int read() override { if (rx.empty()) return -1; uint8_t b = rx.front(); rx.pop_front(); return b; }
    uint32_t nowMs() override { return now; }
    void ev(uint8_t code, std::vector<uint8_t> p) { rx.push_back(0x04); rx.push_back(code); rx.push_back((uint8_t)p.size()); rx.insert(rx.end(), p.begin(), p.end()); }
    void cc(uint16_t op, std::vector<uint8_t> ret, uint8_t ncmd = 1) { std::vector<uint8_t> p = { ncmd, (uint8_t)op, (uint8_t)(op >> 8) }; p.insert(p.end(), ret.begin(), ret.end()); ev(0x0E, p); }
    void cs(uint16_t op, uint8_t status = 0, uint8_t ncmd = 1) { ev(0x0F, { status, ncmd, (uint8_t)op, (uint8_t)(op >> 8) }); }
    int count(uint16_t op) { int n = 0; for (auto &c : cmds) if (c.first == op) n++; return n; }
};
static FakeIo *g_io = nullptr; static Hci *g_hci = nullptr; static A2dpSink *g_sink = nullptr;
static void evThunk(void *ctx, uint8_t code, const uint8_t *p, uint8_t len) { ((A2dpSink *)ctx)->onEvent(code, p, len); }
static std::vector<std::string> g_log;
static void logFn(void *, const char *line) { g_log.push_back(std::string(line)); }
// The phone that pages us -- an address the sink has never met (acceptUnknown is what lets a stranger in).
static const uint8_t PHONE[6] = { 0x76, 0x1A, 0x7E, 0x8A, 0x0C, 0x00 };     // 00:0C:8A:7E:1A:76, LE byte order
static const uint8_t KEY[16] = { 0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F };
static bool g_refuseAuth = false;    // failPairing(): the controller refuses Authentication_Requested
// The controller for every scenario: setup succeeds, an incoming page is accepted (handle 1), the SSP
// Just-Works dance the PEER drives is answered step by step (btlink_test's bytes), Disconnect completes.
static void controller(FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
    std::vector<uint8_t> bd(PHONE, PHONE + 6);
    if (op == 0x0409) {                                                                     // Accept_Connection_Request
        std::vector<uint8_t> r = { 0x00 }; r.insert(r.end(), prm.begin(), prm.begin() + 6); f.cc(op, r);
        std::vector<uint8_t> cc = { 0x00, 0x01, 0x00 }; cc.insert(cc.end(), PHONE, PHONE + 6); cc.push_back(0x01); cc.push_back(0x00);
        f.ev(0x03, cc); return; }
    if (op == 0x040C) { std::vector<uint8_t> r = { 0x00 }; r.insert(r.end(), prm.begin(), prm.begin() + 6); f.cc(op, r);
                        f.ev(0x31, bd); return; }                                           // neg link-key reply -> IO_Capability_Request
    if (op == 0x042B) { std::vector<uint8_t> r = { 0x00 }; r.insert(r.end(), prm.begin(), prm.begin() + 6); f.cc(op, r);
                        std::vector<uint8_t> rsp = bd; rsp.push_back(0x03); rsp.push_back(0x00); rsp.push_back(0x04); f.ev(0x32, rsp);
                        std::vector<uint8_t> uc = bd; uc.push_back(0x40); uc.push_back(0xE2); uc.push_back(0x01); uc.push_back(0x00); f.ev(0x33, uc); return; }
    if (op == 0x042C) { std::vector<uint8_t> r = { 0x00 }; r.insert(r.end(), prm.begin(), prm.begin() + 6); f.cc(op, r);
                        std::vector<uint8_t> spc = { 0x00 }; spc.insert(spc.end(), PHONE, PHONE + 6); f.ev(0x36, spc);
                        std::vector<uint8_t> lk = bd; lk.insert(lk.end(), KEY, KEY + 16); lk.push_back(0x04); f.ev(0x18, lk);
                        f.ev(0x06, { 0x00, 0x01, 0x00 }); return; }
    if (op == 0x0411) { f.cs(op, g_refuseAuth ? 0x0C : 0x00); return; }                     // Authentication_Requested
    if (op == 0x0406) { f.cs(op); f.ev(0x05, { 0x00, prm[0], prm[1], 0x16 }); return; }     // Disconnect
    f.cc(op, { 0x00 });
}
struct Rig {
    FakeIo io; Hci hci; BondTable bonds; uint8_t sigId = 0x20;
    uint16_t sigLocal = 0, mediaLocal = 0;
    Rig() : hci(io) { g_io = &io; g_hci = &hci; g_sink = nullptr; io.onCmd = controller; g_log.clear(); g_refuseAuth = false; }
    void attach(A2dpSink &s) { g_sink = &s; hci.onEvent(evThunk, &s); s.setLog(logFn, nullptr); }
    // One loop pass: advance fake time, pump the HCI (dispatches events), tick + service the attempt.
    void step() { io.now += 10; hci.service(); if (g_sink) { g_sink->tick(io.now); g_sink->service(); } }
    void tick() { step(); }
    bool runUntil(std::function<bool()> pred, uint32_t ms) {
        uint32_t end = io.now + ms;
        while (io.now < end) { if (pred()) return true; step(); }
        return pred();
    }
    void advanceMs(uint32_t ms) { uint32_t end = io.now + ms; while (io.now < end) step(); }
    void answerPrepare() { CHECK(runUntil([&] { return !g_sink->link().busy(); }, 5000)); }
    // ---- the peer's side of the wire ---------------------------------------------------------------
    void incomingPage(const uint8_t bd[6]) {
        std::vector<uint8_t> cr(bd, bd + 6); cr.push_back(0x18); cr.push_back(0x04); cr.push_back(0x24); cr.push_back(0x01);  // cod A/V, ACL
        io.ev(0x04, cr); step();
    }
    // The SSP Just-Works dance, driven by the PEER (an inbound pair issues no command of its own): the
    // controller's Link_Key_Request lands, we answer negatively, and the dance ends in Encryption_Change.
    void peerAuthenticates() {
        io.ev(0x17, std::vector<uint8_t>(PHONE, PHONE + 6));
        CHECK(runUntil([&] { return io.count(0x042C) >= 1; }, 1000));
        io.ev(0x08, { 0x00, 0x01, 0x00, 0x01 });                                            // Encryption_Change: the peer secured it
        step();
    }
    void failPairing() { g_refuseAuth = true; advanceMs(2500); }                            // the peer never secures; the fallback ladder is refused
    void disconnectionComplete(uint8_t reason) { io.ev(0x05, { 0x00, 0x01, 0x00, reason }); step(); }
    // Deliver an inbound L2CAP PDU addressed to OUR local cid (as Hci would hand it to onAcl).
    void feed(uint16_t localCid, std::vector<uint8_t> pl) {
        std::vector<uint8_t> v = { (uint8_t)pl.size(), (uint8_t)(pl.size() >> 8), (uint8_t)localCid, (uint8_t)(localCid >> 8) };
        v.insert(v.end(), pl.begin(), pl.end()); g_sink->onAcl(0x0001, v.data(), (uint16_t)v.size());
    }
    void peerAcl(uint16_t localCid, std::vector<uint8_t> pl) { feed(localCid, pl); }
    // The peer opens a channel at us: CONN_REQ (its scid), then the config exchange -- avdtp_test's
    // openInboundSignalling bytes.  Returns OUR local cid for that channel (what inbound PDUs address).
    uint16_t peerOpens(uint16_t psm, uint16_t scid) {
        feed(0x0001, { 0x02, sigId++, 0x04, 0x00, (uint8_t)psm, (uint8_t)(psm >> 8), (uint8_t)scid, (uint8_t)(scid >> 8) }); step();
        L2cap::Channel *ch = g_sink->l2().byRemote(scid); CHECK(ch != nullptr);
        uint16_t lc = ch ? ch->localCid : 0;
        feed(0x0001, { 0x04, sigId++, 8, 0, (uint8_t)lc, (uint8_t)(lc >> 8), 0, 0, 0x01, 0x02, 0x7F, 0x03 });
        feed(0x0001, { 0x05, sigId++, 6, 0, (uint8_t)lc, (uint8_t)(lc >> 8), 0, 0, 0, 0 }); step();
        return lc;
    }
    void peerAvdtp(std::vector<uint8_t> pdu) { feed(sigLocal, pdu); step(); }
    uint16_t mediaCid() const { return mediaLocal; }
    // The last AVDTP PDU WE sent (payloads on the signalling channel only -- L2CAP signalling goes on cid 1).
    std::vector<uint8_t> lastAvdtp() {
        const L2cap::Channel *ch = g_sink->l2().byLocal(sigLocal);
        uint16_t rc = ch ? ch->remoteCid : 0;
        for (size_t i = io.aclOut.size(); i-- > 0;) { const std::vector<uint8_t> &p = io.aclOut[i];
            if (p.size() >= 5 && (uint16_t)(p[2] | (p[3] << 8)) == rc) return std::vector<uint8_t>(p.begin() + 4, p.end()); }
        return std::vector<uint8_t>(4, 0);                                                  // never null-deref a check
    }
    int countAvdtp(uint8_t sig) {
        const L2cap::Channel *ch = g_sink->l2().byLocal(sigLocal);
        uint16_t rc = ch ? ch->remoteCid : 0; int n = 0;
        for (auto &p : io.aclOut) if (p.size() >= 6 && (uint16_t)(p[2] | (p[3] << 8)) == rc && p[5] == sig) n++;
        return n;
    }
    // A ServiceSearchAttributeRequest for `uuid`, attribute 0x0001 (ServiceClassIDList), on the peer's SDP channel.
    void peerSdpQuery(uint16_t uuid, uint16_t sdpLocal) {
        feed(sdpLocal, { 0x06, 0x00, 0x01, 0x00, 0x0D, 0x35, 0x03, 0x19, (uint8_t)(uuid >> 8), (uint8_t)uuid,
                         0x03, 0xF0, 0x35, 0x03, 0x09, 0x00, 0x01, 0x00 }); step();
    }
    bool sawSdpResponse(uint16_t uuid) {
        for (auto &p : io.aclOut) { if (p.size() < 8 || p[4] != 0x07) continue;             // ServiceSearchAttributeResponse
            for (size_t i = 4; i + 2 < p.size(); i++)
                if (p[i] == 0x19 && p[i + 1] == (uint8_t)(uuid >> 8) && p[i + 2] == (uint8_t)uuid) return true; }
        return false;
    }
    // ---- composite bring-ups -----------------------------------------------------------------------
    // Link up, paired as responder, the peer's AVDTP signalling channel adopted (role ACCEPTOR).
    void bringToAvdtp(A2dpSink &sink) {
        sink.setBonds(&bonds); sink.begin(io.now, 8); answerPrepare();
        incomingPage(PHONE);
        CHECK(runUntil([&] { return sink.link().inboundUp(); }, 500));
        CHECK(sink.start());
        peerAuthenticates();
        CHECK(runUntil([&] { return sink.state() == A2dpSink::AVDTP_WAIT; }, 2000));
        sigLocal = peerOpens(Avdtp::PSM, 0x00C0);
        CHECK(runUntil([&] { return sink.avdtp().role() == Avdtp::ACCEPTOR; }, 500));
    }
    // ... plus DISCOVER, GET_ALL_CAPABILITIES and a SET_CONFIGURATION whose SBC rate/mode byte is `rateMode`.
    void bringToConfigured(A2dpSink &sink, uint8_t rateMode) {
        bringToAvdtp(sink);
        peerAvdtp({ 0x10, 0x01 }); CHECK(runUntil([&] { return lastAvdtp()[1] == 0x01; }, 200));
        peerAvdtp({ 0x20, 0x0C, 1 << 2 }); CHECK(runUntil([&] { return lastAvdtp()[1] == 0x0C; }, 200));
        peerAvdtp({ 0x30, 0x03, 1 << 2, 1 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, rateMode, 0x15, 0x02, 0x35, 0x08, 0x00 });
        CHECK(runUntil([&] { return lastAvdtp()[1] == 0x03; }, 200));
    }
    // ... plus OPEN, the media channel and START: the sink is STREAMING with media flowing.
    void bringToStreaming(A2dpSink &sink) {
        bringToConfigured(sink, 0x21);                                                      // 44.1 kHz, joint stereo
        peerAvdtp({ 0x40, 0x06, 1 << 2 }); CHECK(runUntil([&] { return lastAvdtp()[1] == 0x06; }, 200));
        mediaLocal = peerOpens(Avdtp::PSM, 0x00C1);
        peerAvdtp({ 0x50, 0x07, 1 << 2 });
        CHECK(runUntil([&] { return sink.state() == A2dpSink::STREAMING; }, 500));
    }
};
int main() {
    {   // K1. A source pages us (Connection_Request -> Accept -> Connection_Complete), authenticates (Link_Key_Request ->
        //     our negative reply -> IO cap dance -> Simple_Pairing_Complete/Link_Key_Notification/Authentication_Complete
        //     -> Encryption_Change), opens L2CAP SDP + AVDTP signalling at us, drives DISCOVER / GET_ALL_CAP / SET_CONFIG
        //     (44.1 joint 16/8 loudness 53, delay reporting) / OPEN / media channel / START.  The sink reaches STREAMING,
        //     sbcParams() reflects the config, the media callback receives the RTP payload of an ACL on the media channel.
        Rig r; A2dpSink sink(r.hci, r.io); r.attach(sink); sink.setBonds(&r.bonds);
        CHECK(Sdp::role() == Sdp::SINK);                                          // the constructor publishes the AudioSink record
        sink.begin(0, 8); r.answerPrepare();
        static std::vector<uint8_t> got; got.clear();
        sink.onMedia([](void *, const uint8_t *p, uint16_t n) { got.assign(p, p + n); }, nullptr);
        r.incomingPage(PHONE);                                                    // a stranger: acceptUnknown is ON for a sink
        CHECK(r.runUntil([&] { return sink.link().inboundUp(); }, 500));
        CHECK(sink.start()); r.peerAuthenticates();
        CHECK(r.runUntil([&] { return sink.state() == A2dpSink::AVDTP_WAIT; }, 2000));   // pairing done, L2cap begun
        CHECK(sink.link().pairedBy() != nullptr && strcmp(sink.link().pairedBy(), "peer") == 0);
        uint16_t sdpLocal = r.peerOpens(Sdp::PSM, 0x0044);                        // the source reads our AudioSink record
        r.peerSdpQuery(0x110B, sdpLocal);
        CHECK(r.runUntil([&] { return r.sawSdpResponse(0x110B); }, 200));
        r.sigLocal = r.peerOpens(Avdtp::PSM, 0x0045);                             // AVDTP signalling
        CHECK(r.runUntil([&] { return sink.avdtp().role() == Avdtp::ACCEPTOR && sink.state() == A2dpSink::AVDTP; }, 500));
        r.peerAvdtp({ 0x10, 0x01 });
        CHECK(r.runUntil([&] { return r.lastAvdtp()[1] == 0x01 && (r.lastAvdtp()[0] & 3) == 2; }, 200));
        CHECK((r.lastAvdtp()[3] & 0x08) != 0);                                    // our one SEP is advertised as a SNK (TSEP bit)
        r.peerAvdtp({ 0x20, 0x0C, 1 << 2 }); CHECK(r.runUntil([&] { return r.lastAvdtp()[1] == 0x0C; }, 200));
        r.peerAvdtp({ 0x30, 0x03, 1 << 2, 1 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x35, 0x08, 0x00 });
        CHECK(r.runUntil([&] { return r.lastAvdtp()[1] == 0x03; }, 200));
        CHECK((r.lastAvdtp()[0] & 3) == 2);                                       // ACCEPTed, not rejected
        CHECK(sink.sbcParams().bitpool == 53 && sink.sbcParams().mode == Sbc::JOINT_STEREO);
        r.peerAvdtp({ 0x40, 0x06, 1 << 2 }); CHECK(r.runUntil([&] { return r.lastAvdtp()[1] == 0x06; }, 200));
        r.mediaLocal = r.peerOpens(Avdtp::PSM, 0x0046);                           // media channel
        r.peerAvdtp({ 0x50, 0x07, 1 << 2 });
        CHECK(r.runUntil([&] { return sink.state() == A2dpSink::STREAMING; }, 1000));
        CHECK(sink.result() == A2dpSink::OK && sink.mediaCid() == 0x0046);
        r.peerAcl(r.mediaCid(), { 0x80, 0x60, 0x00, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0x9C, 0xBD, 0x35 });
        r.tick(); CHECK(got.size() == 16 && got[0] == 0x80 && got[13] == 0x9C);
        // after START with delay reporting configured, the sink sent ONE DelayReport, addressed to the SOURCE's
        // endpoint (the INT SEID of its SET_CONFIGURATION -- AVDTP 1.3 s8.19), and the source's ACCEPT is consumed.
        CHECK(r.countAvdtp(0x0D) == 1);
        std::vector<uint8_t> dr; for (auto &p : r.io.aclOut) if (p.size() >= 6 && p[5] == 0x0D) dr.assign(p.begin() + 4, p.end());
        CHECK(dr.size() == 5 && dr[2] == (1 << 2) && ((dr[3] << 8) | dr[4]) == 460);
        if (dr.size() == 5) r.peerAvdtp({ (uint8_t)((dr[0] & 0xF0) | 0x02), 0x0D });   // the source ACCEPTs it
        r.advanceMs(100);
        CHECK(r.countAvdtp(0x0D) == 1 && sink.avdtp().delayRejects() == 0);       // exactly one, never repeated
    }
    {   // K2. SUSPEND then START again keeps the attempt STREAMING-capable: suspended() is true between them and the media
        //     callback is NOT invoked while suspended; a link loss ends LOST with L2cap/Avdtp reset.
        Rig r; A2dpSink sink(r.hci, r.io); r.attach(sink); r.bringToStreaming(sink);
        r.peerAvdtp({ 0x60, 0x09, 1 << 2 }); CHECK(r.runUntil([&] { return sink.suspended(); }, 200));
        static int calls = 0; calls = 0;
        sink.onMedia([](void *, const uint8_t *, uint16_t) { calls++; }, nullptr);
        r.peerAcl(r.mediaCid(), { 0x80, 0x60, 0, 2, 0,0,0,0, 0,0,0,0, 0x01, 0x9C }); r.tick(); CHECK(calls == 0);
        r.peerAvdtp({ 0x70, 0x07, 1 << 2 });
        CHECK(r.runUntil([&] { return !sink.suspended() && sink.state() == A2dpSink::STREAMING; }, 200));
        r.peerAcl(r.mediaCid(), { 0x80, 0x60, 0, 3, 0,0,0,0, 0,0,0,0, 0x01, 0x9C }); r.tick(); CHECK(calls == 1);
        r.disconnectionComplete(0x08);
        CHECK(r.runUntil([&] { return sink.result() == A2dpSink::LOST; }, 200));
        CHECK(sink.state() == A2dpSink::DONE && !sink.busy());
        CHECK(sink.l2().freeSlots() == L2cap::MAX_CHANNELS);                      // every channel released
    }
    {   // K3. A 48 kHz SET_CONFIGURATION is rejected and the attempt does NOT reach STREAMING; the deadline ends it AVDTP_FAILED.
        Rig r; A2dpSink sink(r.hci, r.io); r.attach(sink);
        r.bringToConfigured(sink, /*48 kHz, joint stereo*/ 0x11);
        CHECK(r.lastAvdtp()[0] == 0x33 && r.lastAvdtp()[3] == 0x29);              // REJECT of tl 3: category 0x07, INVALID_CODEC_PARAMETER
        CHECK(r.lastAvdtp()[2] == 0x07 && sink.state() == A2dpSink::AVDTP);
        r.advanceMs(31000);                                                        // past the AVDTP deadline (30 s)
        CHECK(r.runUntil([&] { return sink.result() == A2dpSink::AVDTP_FAILED; }, 200));
        CHECK(sink.avdtp().state() != Avdtp::STREAMING);
    }
    printf("a2dpsink_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
