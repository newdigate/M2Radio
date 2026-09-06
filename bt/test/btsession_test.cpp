// Host tests for BtSession -- the A2DP link-lifecycle POLICY (NEW-34 piece 2, Task 6).  The
// bonded-candidate WALK that used to live in A2dpSource::connect moves here, so its pins move with
// it: boot walk pages bonds MRU-first (first candidate PAGE_ATTEMPTS, later ones once) through the
// target-name filter (an EMPTY stored name is a wildcard), stops at the first successful LINK (a page
// that comes up -- later candidates are not paged even when the attempt then fails), and falls back
// to inquiry.  Beyond the walk this file tests the policy proper: lost-peer retry forever (one page
// per RETRY_MS to the lost address only, the first retry a full cycle away), page-scan-when-idle,
// retry-cancel-on-incoming, stats, callbacks, and MANUAL/resume.  The scaffolding mirrors
// a2dpsource_test.cpp (FakeIo/controller/runUntil); STREAMING is reached via the AVDTP acceptor on an
// INBOUND link, exactly as a2dpsource_test does, because an outbound stream needs a full AVDTP sink.
#include "BtSession.h"
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
    std::function<void(FakeIo &, uint16_t, const std::vector<uint8_t> &)> onCmd;
    size_t write(const uint8_t *p, size_t n) override { acc.insert(acc.end(), p, p + n); parse(); return n; }
    void parse() {
        for (;;) {
            if (acc.size() >= 4 && acc[0] == 0x01) {          // HCI command
                uint8_t plen = acc[3]; if (acc.size() < 4u + plen) break;
                uint16_t op = (uint16_t)(acc[1] | (acc[2] << 8)); std::vector<uint8_t> prm(acc.begin() + 4, acc.begin() + 4 + plen);
                acc.erase(acc.begin(), acc.begin() + 4 + plen); cmds.push_back(std::make_pair(op, prm)); if (onCmd) onCmd(*this, op, prm);
            } else if (acc.size() >= 5 && acc[0] == 0x02) {   // ACL out
                uint16_t dlen = (uint16_t)(acc[3] | (acc[4] << 8)); if (acc.size() < 5u + dlen) break;
                acc.erase(acc.begin(), acc.begin() + 5 + dlen);
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
    // Every Create_Connection's 6-byte BD_ADDR parameter, in issue order.
    std::vector<std::vector<uint8_t> > pages() { std::vector<std::vector<uint8_t> > v; for (auto &c : cmds) if (c.first == 0x0405) v.push_back(std::vector<uint8_t>(c.second.begin(), c.second.end())); return v; }
};
static FakeIo *g_io = nullptr; static Hci *g_hci = nullptr;
static void evThunk(void *ctx, uint8_t code, const uint8_t *p, uint8_t len) { ((A2dpSource *)ctx)->onEvent(code, p, len); }
static std::vector<std::string> g_log;
static void logFn(void *, const char *line) { g_log.push_back(std::string(line)); }
static const uint8_t SHOKZ[6] = { 0x2F, 0x29, 0x31, 0xB3, 0x86, 0xC0 };
static const uint8_t SINK[6]  = { 0x01, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };
static Bond mk(const uint8_t bd[6], const char *name, uint8_t psrm = 1) {
    Bond b; memset(&b, 0, sizeof b); memcpy(b.bd, bd, 6); memset(b.key, 0x5A, 16); b.keyType = 4; b.psrm = psrm; BondTable::copyName(b.name, name); return b;
}
static int countPageTo(FakeIo &io, const uint8_t bd[6]) {
    int n = 0; for (auto &c : io.cmds) if (c.first == 0x0405 && c.second.size() >= 6 && memcmp(c.second.data(), bd, 6) == 0) n++; return n;
}
// Deliver an inbound L2CAP PDU on `cid` (as A2dpSource::onAcl would receive it from Hci).
static void feedAcl(A2dpSource &src, uint16_t cid, std::vector<uint8_t> pl) {
    std::vector<uint8_t> v = { (uint8_t)pl.size(), (uint8_t)(pl.size() >> 8), (uint8_t)cid, (uint8_t)(cid >> 8) };
    v.insert(v.end(), pl.begin(), pl.end()); src.onAcl(0x0001, v.data(), (uint16_t)v.size());
}
// One controller for the whole file, keyed on g_present: PREPARE succeeds; a Create_Connection to
// g_present comes up (handle 1), to anyone else reports Page Timeout (0x04) so the walk retries/advances;
// an inquiry finds NOTHING (so it is merely attempted, never pages a hit); an incoming page is accepted;
// Authentication_Requested is refused (0x0C), so a paged link that comes up ends PAIR_FAILED.
static const uint8_t *g_present = nullptr;
static void controller(FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
    if (op == 0x0C01 || op == 0x0C56 || op == 0x0C18) { f.cc(op, { 0x00 }); return; }                       // PREPARE
    if (op == 0x0405) { f.cs(op);                                                                            // Create_Connection
        std::vector<uint8_t> r = { (uint8_t)((g_present && memcmp(prm.data(), g_present, 6) == 0) ? 0x00 : 0x04), 0x01, 0x00 };
        r.insert(r.end(), prm.begin(), prm.begin() + 6); r.push_back(0x01); r.push_back(0x00); f.ev(0x03, r); return; }
    if (op == 0x0401) { f.cs(op); f.ev(0x01, { 0x00 }); return; }                                            // Inquiry -> Inquiry_Complete, no hits
    if (op == 0x0409) { f.cs(op); return; }                                                                 // Accept_Connection_Request
    if (op == 0x0406) { f.cs(op); f.ev(0x05, { 0x00, prm[0], prm[1], 0x16 }); return; }                     // Disconnect
    if (op == 0x0411) { f.cs(op, 0x0C); return; }                                                           // Authentication_Requested refused
    f.cc(op, { 0x00 });
}
// Callback recorders.
struct StreamRec { bool streaming; uint8_t reason; BtSession::By by; };
static std::vector<StreamRec> g_stream;
static void streamCb(void *, bool s, uint8_t r, BtSession::By by) { g_stream.push_back(StreamRec{ s, r, by }); }
struct AttRec { A2dpSource::Result r; std::string pairedBy; };
static std::vector<AttRec> g_att;
static void attemptCb(void *, A2dpSource::Result r, const char *pb) { g_att.push_back(AttRec{ r, pb ? pb : "" }); }

struct SessRig {
    FakeIo io; Hci hci; A2dpSource src; BtSession session; BondTable bonds;
    SessRig() : hci(io), src(hci, io), session(src) {
        g_io = &io; g_hci = &hci; hci.onEvent(evThunk, &src); src.setLog(logFn, nullptr);
        io.onCmd = controller; g_log.clear(); g_stream.clear(); g_att.clear();
    }
    void step() { io.now += 10; hci.service(); session.tick(io.now); src.service(); }
    void stepN(int n) { for (int i = 0; i < n; i++) step(); }
    uint32_t now() { return io.now; }
};
static bool runUntil(SessRig &R, std::function<bool()> pred, uint32_t ms) {
    uint32_t end = R.io.now + ms;
    while (R.io.now < end) { if (pred()) return true; R.step(); }
    return pred();
}
// Bring an INBOUND link up (the peer pages us) while the session is idle/waiting and the boot walk has
// finished, then drive the session's INBOUND attempt through the AVDTP acceptor to STREAMING.  SHOKZ
// must be bonded (BtLink refuses an incoming page from a stranger).  Leaves the session in STREAMING.
static void inboundToStreaming(SessRig &R) {
    std::vector<uint8_t> creq(SHOKZ, SHOKZ + 6); creq.push_back(0x18); creq.push_back(0x04); creq.push_back(0x24); creq.push_back(0x01);  // cod A/V, ACL
    R.io.ev(0x04, creq); R.stepN(3);                                                                        // Connection_Request -> Accept
    std::vector<uint8_t> cc = { 0x00, 0x01, 0x00 }; cc.insert(cc.end(), SHOKZ, SHOKZ + 6); cc.push_back(0x01); cc.push_back(0x00);
    R.io.ev(0x03, cc); R.stepN(2);                                                                          // Connection_Complete -> inboundUp
    CHECK(runUntil(R, [&]{ return R.src.state() == A2dpSource::PAIRING; }, 3000));                          // session started the INBOUND attempt
    R.io.ev(0x08, { 0x00, 0x01, 0x00, 0x01 });                                                             // Encryption_Change -> secure
    CHECK(runUntil(R, [&]{ return R.src.state() == A2dpSource::AVDTP_WAIT; }, 5000));
    feedAcl(R.src, 0x0001, { 0x02, 0x20, 4, 0, 0x19, 0x00, 0xC0, 0x00 }); R.step();                         // peer opens AVDTP signalling
    L2cap::Channel *sig = R.src.l2().byRemote(0x00C0); CHECK(sig);
    uint16_t sc = sig ? sig->localCid : 0;
    feedAcl(R.src, 0x0001, { 0x04, 0x21, 8, 0, (uint8_t)sc, (uint8_t)(sc >> 8), 0, 0, 0x01, 0x02, 0x7F, 0x03 });
    feedAcl(R.src, 0x0001, { 0x05, 0x22, 6, 0, (uint8_t)sc, (uint8_t)(sc >> 8), 0, 0, 0, 0 }); R.step();
    CHECK(runUntil(R, [&]{ return R.src.avdtp().role() == Avdtp::ACCEPTOR; }, 3000));
    feedAcl(R.src, sc, { 0x30, 0x01 }); R.step();                                                           // DISCOVER
    feedAcl(R.src, sc, { 0x40, 0x0C, 1 << 2 }); R.step();                                                   // GET_ALL_CAPABILITIES
    feedAcl(R.src, sc, { 0x50, 0x03, 1 << 2, 5 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x23 }); R.step();  // SET_CONFIGURATION bitpool 35
    feedAcl(R.src, sc, { 0x60, 0x06, 1 << 2 }); R.step();                                                   // OPEN
    feedAcl(R.src, 0x0001, { 0x02, 0x23, 4, 0, 0x19, 0x00, 0xC1, 0x00 }); R.step();                         // peer opens the media channel
    L2cap::Channel *med = R.src.l2().byRemote(0x00C1); CHECK(med);
    uint16_t mc = med ? med->localCid : 0;
    feedAcl(R.src, 0x0001, { 0x04, 0x24, 8, 0, (uint8_t)mc, (uint8_t)(mc >> 8), 0, 0, 0x01, 0x02, 0x7F, 0x03 });
    feedAcl(R.src, 0x0001, { 0x05, 0x25, 6, 0, (uint8_t)mc, (uint8_t)(mc >> 8), 0, 0, 0, 0 }); R.step();
    CHECK(runUntil(R, [&]{ return R.src.mediaCid() != 0; }, 3000));
    feedAcl(R.src, sc, { 0x70, 0x07, 1 << 2 }); R.step();                                                   // peer START
    CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::STREAMING; }, 3000));
}
int main() {
    {   // B1. Boot walk: two bonds, MRU-first, no name filter -> first candidate PAGE_ATTEMPTS pages,
        //     the second ONE page, then the inquiry fallback.  All pages fail (g_present == nullptr).
        SessRig R; g_present = nullptr;
        R.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz")); R.bonds.upsert(mk(SINK, "EVKB-SINK", 2));           // table: [SINK (MRU), SHOKZ]
        R.session.setRetryMs(3000); R.session.begin(&R.bonds, nullptr, 100, R.now());
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::WAITING; }, 3000));
        std::vector<std::vector<uint8_t> > p = R.io.pages();
        CHECK(p.size() == 4);                                                                               // 3x MRU + 1x next
        CHECK(memcmp(p[0].data(), SINK, 6) == 0 && memcmp(p[1].data(), SINK, 6) == 0 && memcmp(p[2].data(), SINK, 6) == 0);
        CHECK(memcmp(p[3].data(), SHOKZ, 6) == 0);                                                          // MRU-first: SINK (3), then SHOKZ (1)
        CHECK(p[0].size() == 13 && p[0][8] == 2);                                                           // the MRU bond's stored psrm on the wire
        CHECK(p[3].size() == 13 && p[3][8] == 1);                                                           // the second candidate's own psrm
        CHECK(R.io.count(0x0401) == 1);                                                                     // inquiry fallback attempted once
    }
    {   // B1b. The target-name filter skips a non-matching bond even when it is MRU; the first PAGED
        //      candidate is the one that gets PAGE_ATTEMPTS.
        SessRig R; g_present = nullptr;
        R.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz")); R.bonds.upsert(mk(SINK, "EVKB-SINK"));              // table: [SINK (MRU), SHOKZ]
        R.session.setRetryMs(3000); R.session.begin(&R.bonds, "Shokz", 100, R.now());
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::WAITING; }, 3000));
        std::vector<std::vector<uint8_t> > p = R.io.pages();
        CHECK(p.size() == 3 && memcmp(p[0].data(), SHOKZ, 6) == 0);                                         // only SHOKZ, 3 attempts; SINK filtered out
        CHECK(countPageTo(R.io, SINK) == 0 && R.io.count(0x0401) == 1);
    }
    {   // B1c. An EMPTY stored name is a WILDCARD, not a dead slot: paged once, after the named match.
        SessRig R; g_present = nullptr;
        R.bonds.upsert(mk(SINK, "")); R.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz"));                       // table: [SHOKZ (MRU), SINK (nameless)]
        R.session.setRetryMs(3000); R.session.begin(&R.bonds, "Shokz", 100, R.now());
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::WAITING; }, 3000));
        std::vector<std::vector<uint8_t> > p = R.io.pages();
        CHECK(p.size() == 4 && memcmp(p[3].data(), SINK, 6) == 0);                                          // 3x SHOKZ then the nameless bond once
        CHECK(countPageTo(R.io, SHOKZ) == 3 && countPageTo(R.io, SINK) == 1);
    }
    {   // B1d. Stop at the first successful LINK: the MRU candidate's page comes up (g_present == SINK),
        //      so the second candidate is NEVER paged even though the attempt then fails at pairing.
        SessRig R; g_present = SINK;
        R.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz")); R.bonds.upsert(mk(SINK, "EVKB-SINK"));              // table: [SINK (MRU), SHOKZ]
        R.session.setRetryMs(3000); R.session.begin(&R.bonds, nullptr, 100, R.now());
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::WAITING; }, 5000));
        std::vector<std::vector<uint8_t> > p = R.io.pages();
        CHECK(p.size() == 1 && memcmp(p[0].data(), SINK, 6) == 0);                                          // one page, to the MRU only
        CHECK(countPageTo(R.io, SHOKZ) == 0 && R.io.count(0x0406) == 1);                                    // SHOKZ never paged; the link was torn down
    }
    {   // B2. Lost-peer retry: reach STREAMING (inbound), drop the link, and assert exactly ONE
        //     Create_Connection per RETRY_MS to the LOST address only -- never the other bond, never
        //     inquiry -- with the FIRST retry a full cycle away (no immediate page).
        SessRig R; g_present = nullptr;
        R.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz")); R.bonds.upsert(mk(SINK, "EVKB-SINK"));              // SINK is a bond but filtered by "Shokz"
        R.session.setRetryMs(3000); R.session.begin(&R.bonds, "Shokz", 100, R.now());
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::WAITING; }, 3000));                   // boot walk done (SINK never paged)
        inboundToStreaming(R);
        CHECK(R.session.state() == BtSession::STREAMING);
        int baseShokz = countPageTo(R.io, SHOKZ); int baseInq = R.io.count(0x0401);
        R.io.ev(0x05, { 0x00, 0x01, 0x00, 0x08 }); R.stepN(2);                                              // Disconnection_Complete, handle 1
        CHECK(R.session.state() == BtSession::WAITING);
        uint32_t T = R.now();
        CHECK(runUntil(R, [&]{ return R.now() - T >= 2900; }, 3100));                                       // before the first cycle...
        CHECK(countPageTo(R.io, SHOKZ) == baseShokz);                                                       // ...NO retry has fired yet
        CHECK(runUntil(R, [&]{ return countPageTo(R.io, SHOKZ) == baseShokz + 1; }, 1200));                 // first retry ~RETRY_MS after the drop
        T = R.now();
        CHECK(runUntil(R, [&]{ return countPageTo(R.io, SHOKZ) == baseShokz + 2; }, 4000));                 // second retry ~RETRY_MS later
        CHECK(R.now() - T >= 2800);                                                                         // ...and it waited a full cycle, not immediate
        CHECK(runUntil(R, [&]{ return countPageTo(R.io, SHOKZ) == baseShokz + 3; }, 4000));                 // forever, not a bounded count
        CHECK(countPageTo(R.io, SINK) == 0 && R.io.count(0x0401) == baseInq);                               // only the lost address; never inquiry
    }
    {   // B3. Page scan wanted: on before the first link (bonds exist, not linked); off the moment a link
        //     is up; off in MANUAL even though a bond exists and no link is up.
        SessRig R; g_present = nullptr;
        R.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz"));
        R.session.setRetryMs(3000); R.session.begin(&R.bonds, "Shokz", 100, R.now());
        R.step();
        CHECK(R.session.wantPageScan());                                                                    // before any link, a bond exists
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::WAITING; }, 3000));
        CHECK(R.session.wantPageScan());                                                                    // still not linked
        inboundToStreaming(R);
        CHECK(!R.session.wantPageScan());                                                                   // linked -> off
        R.session.disconnect();
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::MANUAL; }, 3000));
        CHECK(!R.session.wantPageScan());                                                                   // MANUAL -> off even with a bond and no link
    }
    {   // B4. An incoming link cancels the pending retry: in WAITING after a drop, inboundUp() starts an
        //     INBOUND attempt and the retry timer does NOT also fire a Create_Connection.
        SessRig R; g_present = nullptr;
        R.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz")); R.bonds.upsert(mk(SINK, "EVKB-SINK"));
        R.session.setRetryMs(3000); R.session.begin(&R.bonds, "Shokz", 100, R.now());
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::WAITING; }, 3000));
        inboundToStreaming(R);
        R.io.ev(0x05, { 0x00, 0x01, 0x00, 0x08 }); R.stepN(2);                                              // drop -> WAITING (retry armed)
        CHECK(R.session.state() == BtSession::WAITING);
        int baseShokz = countPageTo(R.io, SHOKZ);
        R.stepN(50);                                                                                        // ~500 ms: well before the 3000 ms retry
        CHECK(countPageTo(R.io, SHOKZ) == baseShokz);                                                       // retry has not fired
        inboundToStreaming(R);                                                                              // incoming link -> INBOUND attempt, retry cancelled
        CHECK(R.session.state() == BtSession::STREAMING);
        CHECK(countPageTo(R.io, SHOKZ) == baseShokz);                                                       // the retry timer never fired a page
    }
    {   // B5. Stats: one STREAMING entry via inbound sets links, by=INCOMING and counts attempts; a drop
        //     sets lost + lastReason; the reconnect measures reconnectMs (loss -> next STREAMING).
        SessRig R; g_present = nullptr;
        R.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz"));
        R.session.setRetryMs(3000); R.session.begin(&R.bonds, "Shokz", 100, R.now());
        R.session.onStream(streamCb, nullptr); R.session.onAttempt(attemptCb, nullptr);
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::WAITING; }, 3000));
        inboundToStreaming(R);
        const BtSession::Stats &s = R.session.stats();
        CHECK(s.links == 1 && s.by == BtSession::BY_INCOMING);
        CHECK(s.attempts >= 3);                                                                             // 1 page walk + 1 inquiry + 1 inbound (at least)
        uint32_t lossT = R.now();
        R.io.ev(0x05, { 0x00, 0x01, 0x00, 0x08 }); R.stepN(2);
        CHECK(s.lost == 1 && s.lastReason == 0x08);
        R.stepN(20);                                                                                        // a little dead time before the reconnect
        inboundToStreaming(R);                                                                              // reconnect via a fresh inbound link
        CHECK(s.links == 2);
        CHECK(s.reconnectMs > 0 && s.reconnectMs <= R.now() - lossT);                                       // reconnectMs was measured (loss -> STREAMING)
    }
    {   // B6. Callbacks: onStream(true,..,by) on each STREAMING entry, onStream(false,reason,..) on each
        //     loss, onAttempt(result,pairedBy) on every attempt end (failed boot attempts included).
        SessRig R; g_present = nullptr;
        R.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz"));
        R.session.setRetryMs(3000); R.session.begin(&R.bonds, "Shokz", 100, R.now());
        R.session.onStream(streamCb, nullptr); R.session.onAttempt(attemptCb, nullptr);
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::WAITING; }, 3000));
        size_t attAfterBoot = g_att.size();
        CHECK(attAfterBoot >= 2);                                                                           // page walk + inquiry each ended with onAttempt
        for (size_t i = 0; i < attAfterBoot; i++) CHECK(g_att[i].r != A2dpSource::OK);                      // every boot attempt failed
        inboundToStreaming(R);
        CHECK(g_att.size() == attAfterBoot + 1 && g_att.back().r == A2dpSource::OK);                        // the inbound attempt ended OK
        CHECK(g_stream.size() == 1 && g_stream[0].streaming && g_stream[0].by == BtSession::BY_INCOMING);
        R.io.ev(0x05, { 0x00, 0x01, 0x00, 0x08 }); R.stepN(2);
        CHECK(g_stream.size() == 2 && !g_stream[1].streaming && g_stream[1].reason == 0x08 && g_stream[1].by == BtSession::BY_INCOMING);
    }
    {   // B7. MANUAL: disconnect() -> no attempts and page scan off; resume() -> boot policy again.
        SessRig R; g_present = nullptr;
        R.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz"));
        R.session.setRetryMs(3000); R.session.begin(&R.bonds, "Shokz", 100, R.now());
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::WAITING; }, 3000));
        R.session.disconnect();
        CHECK(runUntil(R, [&]{ return R.session.state() == BtSession::MANUAL; }, 3000));
        int pageAtManual = R.io.count(0x0405);
        R.stepN(800);                                                                                       // 8000 ms > 2 retry cycles: MANUAL issues nothing
        CHECK(R.io.count(0x0405) == pageAtManual && !R.session.wantPageScan());
        R.session.resume();
        CHECK(runUntil(R, [&]{ return R.io.count(0x0405) > pageAtManual; }, 3000));                         // the boot walk paged again
        CHECK(R.session.state() == BtSession::CONNECTING || R.session.state() == BtSession::WAITING);
    }
    printf("btsession_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
