// Host tests for BtSinkSession -- the SINK's link-lifecycle POLICY (NEW-41).  A sink initiates nothing: it
// announces itself (page scan always while idle, inquiry scan until a bond exists), takes over each page the
// controller accepted as ONE A2dpSink attempt, drops both scans while a link is up, and returns to LISTENING
// on a loss OR on a clean stream close.  There is no boot walk, no inquiry and no retry timer here -- a phone
// pages a sink it knows -- so what is left to pin is the SCAN policy, the attempt hand-off, the two ways an
// attempt can end well (loss / CLOSE) and MANUAL.  The scaffolding is a2dpsink_test's rig (FakeIo, the SSP
// controller, the peer's AVDTP side) with the SESSION driving the sink instead of the test calling start().
#include "BtSinkSession.h"
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
            } else if (acc.size() >= 5 && acc[0] == 0x02) {   // ACL out: [02][handle][alen][L2CAP PDU]
                uint16_t dlen = (uint16_t)(acc[3] | (acc[4] << 8)); if (acc.size() < 5u + dlen) break;
                aclOut.push_back(std::vector<uint8_t>(acc.begin() + 5, acc.begin() + 5 + dlen));
                acc.erase(acc.begin(), acc.begin() + 5 + dlen);
                ev(0x13, { 0x01, 0x01, 0x00, 0x01, 0x00 });   // the controller returns the ACL buffer at once
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
    // The parameters of the MOST RECENT command with this opcode -- the scan policy is a sequence of
    // Write_Scan_Enable writes, and what it currently ASKS FOR is the last one, not the first.
    std::vector<uint8_t> lastParamsOf(uint16_t op) {
        for (size_t i = cmds.size(); i-- > 0;) if (cmds[i].first == op) return cmds[i].second;
        return std::vector<uint8_t>();
    }
};
static const uint16_t OP_SCAN = 0x0C1A;                        // Write_Scan_Enable: bit1 = page scan, bit0 = inquiry scan
static A2dpSink *g_sink = nullptr;
static void evThunk(void *ctx, uint8_t code, const uint8_t *p, uint8_t len) { ((A2dpSink *)ctx)->onEvent(code, p, len); }
static std::vector<std::string> g_log;
static void logFn(void *, const char *line) { g_log.push_back(std::string(line)); }
static int logCount(const char *needle) { int n = 0; for (auto &l : g_log) if (l.find(needle) != std::string::npos) n++; return n; }
// The phone that pages us -- an address the sink has never met (acceptUnknown is what lets a stranger in).
static const uint8_t PHONE[6] = { 0x76, 0x1A, 0x7E, 0x8A, 0x0C, 0x00 };     // 00:0C:8A:7E:1A:76, LE byte order
static const uint8_t KEY[16] = { 0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F };
static bool g_refuseAuth = false;    // failPairing(): the controller refuses Authentication_Requested
// a2dpsink_test's controller verbatim: setup succeeds, an incoming page is accepted (handle 1), the SSP
// Just-Works dance the PEER drives is answered step by step, Disconnect completes.
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
// Callback recorders.
struct StreamRec { bool streaming; uint8_t reason; };
static std::vector<StreamRec> g_stream;
static void streamCb(void *, bool s, uint8_t r) { g_stream.push_back(StreamRec{ s, r }); }
struct AttRec { A2dpSink::Result r; std::string pairedBy; };
static std::vector<AttRec> g_att;
static void attemptCb(void *, A2dpSink::Result r, const char *pb) { g_att.push_back(AttRec{ r, pb ? pb : "" }); }
// Q6: a stream callback that disconnects the session from INSIDE the callback, on the loss edge.
static BtSinkSession *g_cbSession = nullptr;
static void disconnectOnLossCb(void *, bool streaming, uint8_t) { if (!streaming && g_cbSession) g_cbSession->disconnect(); }
// Q7: the ATTEMPT callback disconnects on the success edge -- the other side of the same rule.  It records the
// attempt exactly as attemptCb does, so the callback ORDER is still observable, and then asks to disconnect.
static void disconnectOnAttemptCb(void *, A2dpSink::Result r, const char *pb) {
    g_att.push_back(AttRec{ r, pb ? pb : "" });
    if (r == A2dpSink::OK && g_cbSession) g_cbSession->disconnect();
}

struct Rig {
    FakeIo io; Hci hci; A2dpSink sink; BtSinkSession session; BondTable bonds;
    uint8_t sigId = 0x20; uint16_t sigLocal = 0, mediaLocal = 0;
    Rig() : hci(io), sink(hci, io), session(sink) {
        g_sink = &sink; hci.onEvent(evThunk, &sink); sink.setLog(logFn, nullptr);
        io.onCmd = controller; g_log.clear(); g_stream.clear(); g_att.clear(); g_refuseAuth = false;
    }
    // One loop pass: advance fake time, pump the HCI, tick the SESSION (which ticks the sink) + service.
    void step() { io.now += 10; hci.service(); session.tick(io.now); sink.service(); }
    void tick() { step(); }
    bool runUntil(std::function<bool()> pred, uint32_t ms) {
        uint32_t end = io.now + ms;
        while (io.now < end) { if (pred()) return true; step(); }
        return pred();
    }
    void advanceMs(uint32_t ms) { uint32_t end = io.now + ms; while (io.now < end) step(); }
    void answerPrepare() { CHECK(runUntil([&] { return !sink.link().busy(); }, 5000)); }
    // ---- the peer's side of the wire ---------------------------------------------------------------
    void incomingPage(const uint8_t bd[6]) {
        std::vector<uint8_t> cr(bd, bd + 6); cr.push_back(0x18); cr.push_back(0x04); cr.push_back(0x24); cr.push_back(0x01);  // cod A/V, ACL
        io.ev(0x04, cr); step();
    }
    // The SSP Just-Works dance, driven by the PEER (an inbound pair issues no command of its own).
    void peerAuthenticates() {
        io.ev(0x17, std::vector<uint8_t>(PHONE, PHONE + 6));
        CHECK(runUntil([&] { return io.count(0x042C) >= 1; }, 1000));
        io.ev(0x08, { 0x00, 0x01, 0x00, 0x01 });                                            // Encryption_Change: the peer secured it
        step();
    }
    void failPairing() { g_refuseAuth = true; advanceMs(2500); }                            // the peer never secures; the fallback ladder is refused
    void disconnectionComplete(uint8_t reason) { io.ev(0x05, { 0x00, 0x01, 0x00, reason }); step(); }
    void feed(uint16_t localCid, std::vector<uint8_t> pl) {
        std::vector<uint8_t> v = { (uint8_t)pl.size(), (uint8_t)(pl.size() >> 8), (uint8_t)localCid, (uint8_t)(localCid >> 8) };
        v.insert(v.end(), pl.begin(), pl.end()); sink.onAcl(0x0001, v.data(), (uint16_t)v.size());
    }
    // The peer opens a channel at us: CONN_REQ (its scid) then the config exchange.  Returns OUR local cid.
    uint16_t peerOpens(uint16_t psm, uint16_t scid) {
        feed(0x0001, { 0x02, sigId++, 0x04, 0x00, (uint8_t)psm, (uint8_t)(psm >> 8), (uint8_t)scid, (uint8_t)(scid >> 8) }); step();
        L2cap::Channel *ch = sink.l2().byRemote(scid); CHECK(ch != nullptr);
        uint16_t lc = ch ? ch->localCid : 0;
        feed(0x0001, { 0x04, sigId++, 8, 0, (uint8_t)lc, (uint8_t)(lc >> 8), 0, 0, 0x01, 0x02, 0x7F, 0x03 });
        feed(0x0001, { 0x05, sigId++, 6, 0, (uint8_t)lc, (uint8_t)(lc >> 8), 0, 0, 0, 0 }); step();
        return lc;
    }
    void peerAvdtp(std::vector<uint8_t> pdu) { feed(sigLocal, pdu); step(); }
    std::vector<uint8_t> lastAvdtp() {
        const L2cap::Channel *ch = sink.l2().byLocal(sigLocal);
        uint16_t rc = ch ? ch->remoteCid : 0;
        for (size_t i = io.aclOut.size(); i-- > 0;) { const std::vector<uint8_t> &p = io.aclOut[i];
            if (p.size() >= 5 && (uint16_t)(p[2] | (p[3] << 8)) == rc) return std::vector<uint8_t>(p.begin() + 4, p.end()); }
        return std::vector<uint8_t>(4, 0);                                                  // never null-deref a check
    }
    // ---- composite bring-up ------------------------------------------------------------------------
    // The phone pages us and the SESSION takes the link over: page -> (session start) -> SSP -> the peer's
    // AVDTP signalling channel -> DISCOVER / GET_ALL_CAP / SET_CONFIG / OPEN / media channel / START.
    // `scid` bases the peer's channel ids so a SECOND bring-up on the same rig uses fresh ones.
    // `expectStreaming` false: the attempt callback took us out of STREAMING inside the same tick, so the
    // session is never OBSERVED there -- wait for it to leave CONNECTING instead (Q7).
    void inboundToStreaming(uint16_t scid, bool expectStreaming = true) {
        incomingPage(PHONE);
        CHECK(runUntil([&] { return session.state() == BtSinkSession::CONNECTING; }, 1000));
        peerAuthenticates();
        CHECK(runUntil([&] { return sink.state() == A2dpSink::AVDTP_WAIT; }, 3000));
        sigLocal = peerOpens(Avdtp::PSM, scid);
        CHECK(runUntil([&] { return sink.avdtp().role() == Avdtp::ACCEPTOR && sink.state() == A2dpSink::AVDTP; }, 500));
        peerAvdtp({ 0x10, 0x01 }); CHECK(runUntil([&] { return lastAvdtp()[1] == 0x01; }, 200));
        peerAvdtp({ 0x20, 0x0C, 1 << 2 }); CHECK(runUntil([&] { return lastAvdtp()[1] == 0x0C; }, 200));
        peerAvdtp({ 0x30, 0x03, 1 << 2, 2 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x35, 0x08, 0x00 });
        CHECK(runUntil([&] { return lastAvdtp()[1] == 0x03; }, 200));
        peerAvdtp({ 0x40, 0x06, 1 << 2 }); CHECK(runUntil([&] { return lastAvdtp()[1] == 0x06; }, 200));
        mediaLocal = peerOpens(Avdtp::PSM, (uint16_t)(scid + 1));
        peerAvdtp({ 0x50, 0x07, 1 << 2 });
        if (expectStreaming) CHECK(runUntil([&] { return session.state() == BtSinkSession::STREAMING; }, 1000));
        else                 CHECK(runUntil([&] { return session.state() != BtSinkSession::CONNECTING; }, 1000));
    }
};
int main() {
    {   // Q1. The idle POLICY and what turns each half of it off.  Unbonded and listening we are BOTH
        //     connectable and discoverable (0x03); an accepted page becomes an attempt without the app
        //     asking; a live link drops both scans (0x00); a drop returns us to LISTENING with the reason
        //     counted; and with a bond in the table we stay connectable but stop advertising (0x02) --
        //     unless the app insists (setAlwaysDiscoverable).
        Rig r; r.session.begin(&r.bonds, 8, 0); r.answerPrepare();
        CHECK(r.runUntil([&] { return r.io.count(OP_SCAN) >= 1; }, 2000));
        CHECK(r.bonds.count() == 0);
        CHECK(r.io.lastParamsOf(OP_SCAN) == std::vector<uint8_t>{ 0x03 });          // page + inquiry scan
        CHECK(r.session.state() == BtSinkSession::LISTENING);
        r.incomingPage(PHONE);
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::CONNECTING; }, 1000));
        CHECK(r.session.stats().attempts == 1);
        r.peerAuthenticates();
        CHECK(r.runUntil([&] { return r.sink.state() == A2dpSink::AVDTP_WAIT; }, 3000));
        r.sigLocal = r.peerOpens(Avdtp::PSM, 0x0060);
        CHECK(r.runUntil([&] { return r.sink.avdtp().role() == Avdtp::ACCEPTOR; }, 500));
        r.peerAvdtp({ 0x10, 0x01 }); CHECK(r.runUntil([&] { return r.lastAvdtp()[1] == 0x01; }, 200));
        r.peerAvdtp({ 0x20, 0x0C, 1 << 2 }); CHECK(r.runUntil([&] { return r.lastAvdtp()[1] == 0x0C; }, 200));
        r.peerAvdtp({ 0x30, 0x03, 1 << 2, 2 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x35, 0x08, 0x00 });
        CHECK(r.runUntil([&] { return r.lastAvdtp()[1] == 0x03; }, 200));
        r.peerAvdtp({ 0x40, 0x06, 1 << 2 }); CHECK(r.runUntil([&] { return r.lastAvdtp()[1] == 0x06; }, 200));
        r.mediaLocal = r.peerOpens(Avdtp::PSM, 0x0061);
        r.peerAvdtp({ 0x50, 0x07, 1 << 2 });
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::STREAMING; }, 1000));
        CHECK(r.session.stats().links == 1 && r.session.stats().accepts == 1);
        CHECK(r.io.lastParamsOf(OP_SCAN) == std::vector<uint8_t>{ 0x00 });          // linked: neither scan
        r.disconnectionComplete(0x13);
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::LISTENING; }, 500));   // <= 50 ticks
        CHECK(r.session.stats().lost == 1 && r.session.stats().lastReason == 0x13);
        CHECK(r.session.stats().closed == 0);
        // The pairing stored a bond, which is what makes us stop advertising -- a real bond from the run,
        // not a synthetic upsert, so the check cannot pass against a table the sink never sees.
        CHECK(r.bonds.count() == 1);
        CHECK(r.runUntil([&] { return r.io.lastParamsOf(OP_SCAN) == std::vector<uint8_t>{ 0x02 }; }, 200));
        r.session.setAlwaysDiscoverable(true); r.tick(); r.tick();
        CHECK(r.io.lastParamsOf(OP_SCAN) == std::vector<uint8_t>{ 0x03 });          // ... unless the app insists
    }
    {   // Q2. Callbacks, and the attempt that fails.  onStream(true) on entry and onStream(false, reason) on
        //     the drop; then a page whose pairing never completes ends the attempt back in LISTENING with
        //     rejects counted, onAttempt reporting PAIR_FAILED, and NO stream callback for it.
        Rig r; r.session.onStream(streamCb, nullptr); r.session.onAttempt(attemptCb, nullptr);
        r.session.begin(&r.bonds, 8, 0); r.answerPrepare();
        r.inboundToStreaming(0x0070);
        CHECK(g_stream.size() == 1 && g_stream[0].streaming && g_stream[0].reason == 0);
        CHECK(g_att.size() == 1 && g_att[0].r == A2dpSink::OK && g_att[0].pairedBy == "peer");
        r.disconnectionComplete(0x08);
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::LISTENING; }, 500));
        CHECK(g_stream.size() == 2 && !g_stream[1].streaming && g_stream[1].reason == 0x08);
        size_t streamAfterLoss = g_stream.size();
        r.incomingPage(PHONE);
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::CONNECTING; }, 1000));
        r.failPairing();
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::LISTENING; }, 3000));
        CHECK(r.session.stats().rejects == 1 && r.session.stats().links == 1);
        CHECK(g_stream.size() == streamAfterLoss);                                  // a failed attempt is not a stream event
        CHECK(g_att.size() == 2 && g_att[1].r == A2dpSink::PAIR_FAILED);
    }
    {   // Q3. The OTHER good ending: the source CLOSEs the stream.  A2dpSink ends that attempt OK (not LOST,
        //     not STOPPED), so the session must count it as a CLOSE, report the stream down with reason 0,
        //     leave `lost` alone, and go back to announcing itself -- and then accept a fresh page all the
        //     way to STREAMING again, which is what exercises A2dpSink::start()'s per-attempt resets through
        //     the session rather than through a hand-driven start().
        Rig r; r.session.onStream(streamCb, nullptr); r.session.onAttempt(attemptCb, nullptr);
        r.session.begin(&r.bonds, 8, 0); r.answerPrepare();
        r.inboundToStreaming(0x0080);
        CHECK(r.io.lastParamsOf(OP_SCAN) == std::vector<uint8_t>{ 0x00 });
        r.peerAvdtp({ 0x60, 0x08, 1 << 2 });                                        // CLOSE
        CHECK(r.lastAvdtp()[0] == 0x62 && r.lastAvdtp()[1] == 0x08);                // ... ACCEPTed
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::LISTENING; }, 3000));
        CHECK(r.session.stats().closed == 1 && r.session.stats().lost == 0);
        CHECK(g_stream.size() == 2 && !g_stream[1].streaming && g_stream[1].reason == 0);
        CHECK(r.sink.result() == A2dpSink::OK && !r.sink.busy());
        CHECK(r.bonds.count() == 1);
        CHECK(r.runUntil([&] { return r.io.lastParamsOf(OP_SCAN) == std::vector<uint8_t>{ 0x02 }; }, 200));   // announcing again
        r.inboundToStreaming(0x0090);                                               // a second phone page, end to end
        CHECK(r.session.stats().links == 2 && r.session.stats().closed == 1 && r.session.stats().lost == 0);
        CHECK(g_stream.size() == 3 && g_stream[2].streaming);
    }
    {   // Q4. MANUAL: an app-driven disconnect takes the link down and stops announcing -- and STAYS off,
        //     which is what makes MANUAL different from LISTENING with no link.  resume() puts it back.
        Rig r; r.session.begin(&r.bonds, 8, 0); r.answerPrepare();
        r.inboundToStreaming(0x00A0);
        r.session.disconnect();
        CHECK(r.session.state() == BtSinkSession::DISCONNECTING);
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::MANUAL; }, 3000));
        CHECK(r.sink.result() == A2dpSink::STOPPED);
        CHECK(r.session.stats().closed == 0);                                       // our own stop() is not a peer CLOSE
        CHECK(r.io.lastParamsOf(OP_SCAN) == std::vector<uint8_t>{ 0x00 });
        int scanWrites = r.io.count(OP_SCAN);
        r.advanceMs(3000);                                                          // MANUAL issues nothing, ever
        CHECK(r.io.count(OP_SCAN) == scanWrites && r.session.state() == BtSinkSession::MANUAL);
        r.session.resume();
        CHECK(r.session.state() == BtSinkSession::LISTENING);
        CHECK(r.runUntil([&] { return r.io.lastParamsOf(OP_SCAN) == std::vector<uint8_t>{ 0x02 }; }, 500));
    }
    {   // Q5. The SAME sequence as a2dpsink_test K7, through the session: the source configures us and then
        //     ABORTs before START.  That is a FAILED attempt -- the session must report AVDTP_FAILED and count
        //     a reject, never a `closed`, and go back to announcing itself.  Note what does NOT discriminate
        //     here: rejects/closed read the same either way, because the session never left CONNECTING; the
        //     teeth are the RESULT the attempt callback carries and the absence of a "stream closed" log.
        Rig r; r.session.onStream(streamCb, nullptr); r.session.onAttempt(attemptCb, nullptr);
        r.session.begin(&r.bonds, 8, 0); r.answerPrepare();
        r.incomingPage(PHONE);
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::CONNECTING; }, 1000));
        r.peerAuthenticates();
        CHECK(r.runUntil([&] { return r.sink.state() == A2dpSink::AVDTP_WAIT; }, 3000));
        r.sigLocal = r.peerOpens(Avdtp::PSM, 0x00B0);
        CHECK(r.runUntil([&] { return r.sink.avdtp().role() == Avdtp::ACCEPTOR; }, 500));
        r.peerAvdtp({ 0x10, 0x01 }); CHECK(r.runUntil([&] { return r.lastAvdtp()[1] == 0x01; }, 200));
        r.peerAvdtp({ 0x20, 0x0C, 1 << 2 }); CHECK(r.runUntil([&] { return r.lastAvdtp()[1] == 0x0C; }, 200));
        r.peerAvdtp({ 0x30, 0x03, 1 << 2, 2 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x35, 0x08, 0x00 });
        CHECK(r.runUntil([&] { return r.lastAvdtp()[1] == 0x03; }, 200));
        r.peerAvdtp({ 0x40, 0x0A, 1 << 2 });                                        // ABORT, before OPEN and before START
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::LISTENING; }, 3000));
        CHECK(r.session.stats().rejects == 1 && r.session.stats().closed == 0 && r.session.stats().links == 0);
        CHECK(g_att.size() == 1 && g_att[0].r == A2dpSink::AVDTP_FAILED);           // RED before the fix: OK
        CHECK(logCount("stream closed") == 0);                                      // RED before the fix: 1
        CHECK(g_stream.empty());                                                    // it never streamed: no stream event either way
        // ... and announcing again.  The SSP dance stored a bond, so page scan only (0x02), not 0x03.
        CHECK(r.bonds.count() == 1);
        CHECK(r.runUntil([&] { return r.io.lastParamsOf(OP_SCAN) == std::vector<uint8_t>{ 0x02 }; }, 500));
    }
    {   // Q6. A callback that calls disconnect().  m_state used to be assigned AFTER the stream callback ran,
        //     so the DISCONNECTING the callback asked for was immediately overwritten with LISTENING and the
        //     session went straight back to announcing itself -- measured: LISTENING with scan 0x02, i.e. the
        //     app's disconnect silently did nothing.  m_state is now assigned BEFORE every callback.
        Rig r; r.session.begin(&r.bonds, 8, 0); r.answerPrepare();
        g_cbSession = &r.session; r.session.onStream(disconnectOnLossCb, nullptr);
        r.inboundToStreaming(0x00C0);
        r.disconnectionComplete(0x08);
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::MANUAL; }, 3000));   // RED: settles LISTENING
        CHECK(r.io.lastParamsOf(OP_SCAN) == std::vector<uint8_t>{ 0x00 });                     // RED: 0x02
        r.advanceMs(1000);
        CHECK(r.session.state() == BtSinkSession::MANUAL);                                     // ... and STAYS there
        g_cbSession = nullptr;
    }
    {   // Q7. The OTHER callback that can disconnect us: the ATTEMPT callback, on the success edge.  m_state is
        //     already STREAMING when it runs (Q6's rule), so a disconnect() from inside it assigns DISCONNECTING
        //     -- and the stream callback that follows on the very next line fired ANYWAY, telling the app the
        //     stream was up on a session it had just torn down.  Nothing would ever send the matching
        //     onStream(false): the DISCONNECTING branch reaches MANUAL without one.  The stream callback is now
        //     guarded on the state the line above assigned.
        Rig r; r.session.begin(&r.bonds, 8, 0); r.answerPrepare();
        g_cbSession = &r.session; r.session.onStream(streamCb, nullptr); r.session.onAttempt(disconnectOnAttemptCb, nullptr);
        r.inboundToStreaming(0x00D0, false);
        CHECK(g_att.size() == 1 && g_att[0].r == A2dpSink::OK);                     // the attempt itself still succeeded
        CHECK(g_stream.empty());                                                    // RED before the fix: one streaming=true
        CHECK(r.runUntil([&] { return r.session.state() == BtSinkSession::MANUAL; }, 3000));
        CHECK(r.io.lastParamsOf(OP_SCAN) == std::vector<uint8_t>{ 0x00 });          // MANUAL announces nothing
        r.advanceMs(1000);
        CHECK(r.session.state() == BtSinkSession::MANUAL);
        CHECK(g_stream.empty());                                                    // ... and no late stream event either
        CHECK(r.session.stats().links == 1 && r.session.stats().accepts == 1);      // stats unchanged by the guard
        g_cbSession = nullptr;
    }
    printf("btsinksession_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
