// Host tests for A2dpSource::connect()'s bonded-candidate walk (NEW-34 piece 1): which bonds are
// paged, in what order, how many attempts each, the target-name filter and its empty-name
// wildcard, and the inquiry fallback.  The controller model stops at pairing (Authentication_Requested
// is refused), so every scenario ends PAIR_FAILED -- the walk is what is under test, the QEMU
// [reconnect] gate covers the rest.
#include "A2dpSource.h"
#include "HciTransport.h"
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
        while (acc.size() >= 4 && acc[0] == 0x01) { uint8_t plen = acc[3]; if (acc.size() < 4u + plen) break;
            uint16_t op = (uint16_t)(acc[1] | (acc[2] << 8)); std::vector<uint8_t> prm(acc.begin() + 4, acc.begin() + 4 + plen);
            acc.erase(acc.begin(), acc.begin() + 4 + plen); cmds.push_back(std::make_pair(op, prm)); if (onCmd) onCmd(*this, op, prm); } }
    int available() override { return (int)rx.size(); }
    int read() override { if (rx.empty()) return -1; uint8_t b = rx.front(); rx.pop_front(); return b; }
    uint32_t nowMs() override { return now; }
    void ev(uint8_t code, std::vector<uint8_t> p) { rx.push_back(0x04); rx.push_back(code); rx.push_back((uint8_t)p.size()); rx.insert(rx.end(), p.begin(), p.end()); }
    void cc(uint16_t op, std::vector<uint8_t> ret, uint8_t ncmd = 1) { std::vector<uint8_t> p = { ncmd, (uint8_t)op, (uint8_t)(op >> 8) }; p.insert(p.end(), ret.begin(), ret.end()); ev(0x0E, p); }
    void cs(uint16_t op, uint8_t status = 0, uint8_t ncmd = 1) { ev(0x0F, { status, ncmd, (uint8_t)op, (uint8_t)(op >> 8) }); }
    int count(uint16_t op) { int n = 0; for (auto &c : cmds) if (c.first == op) n++; return n; }
    std::vector<std::vector<uint8_t> > pages() { std::vector<std::vector<uint8_t> > v; for (auto &c : cmds) if (c.first == 0x0405) v.push_back(std::vector<uint8_t>(c.second.begin(), c.second.begin() + 6)); return v; }
};
static FakeIo *g_io = nullptr; static Hci *g_hci = nullptr;
static void idle10() { g_io->now += 10; g_hci->service(); }
static uint32_t fakeNow() { return g_io->now; }
static void evThunk(void *ctx, uint8_t code, const uint8_t *p, uint8_t len) { ((A2dpSource *)ctx)->onEvent(code, p, len); }
static std::vector<std::string> g_log;
static void logFn(void *, const char *line) { g_log.push_back(std::string(line)); }
static int logCount(const char *needle) { int n = 0; for (auto &l : g_log) if (l.find(needle) != std::string::npos) n++; return n; }
static const uint8_t SHOKZ[6] = { 0x2F, 0x29, 0x31, 0xB3, 0x86, 0xC0 };
static const uint8_t SINK[6]  = { 0x01, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };
static Bond mk(const uint8_t bd[6], const char *name, uint8_t psrm = 1) {
    Bond b; memset(&b, 0, sizeof b); memcpy(b.bd, bd, 6); memset(b.key, 0x5A, 16); b.keyType = 4; b.psrm = psrm; BondTable::copyName(b.name, name); return b;
}
// The controller: setup commands succeed; Create_Connection to `present` succeeds (handle 1), to anyone
// else reports Page Timeout (0x04); an inquiry finds ONE A/V device (SHOKZ, named "OpenMove by Shokz");
// Authentication_Requested is refused, so connect() returns PAIR_FAILED right after the link comes up.
static const uint8_t *g_present = nullptr;
static void controller(FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
    if (op == 0x0401) { f.cs(op);
        std::vector<uint8_t> r = { 1 }; r.insert(r.end(), SHOKZ, SHOKZ + 6); r.push_back(1); r.push_back(0); r.push_back(0);
        r.push_back(0x18); r.push_back(0x04); r.push_back(0x24); r.push_back(0x54); r.push_back(0x08); f.ev(0x02, r); f.ev(0x01, { 0x00 }); return; }
    if (op == 0x0419) { f.cs(op); std::vector<uint8_t> r = { 0x00 }; r.insert(r.end(), prm.begin(), prm.begin() + 6);
        const char *nm = "OpenMove by Shokz"; r.insert(r.end(), nm, nm + strlen(nm)); r.resize(7 + 248, 0); f.ev(0x07, r); return; }
    if (op == 0x0C01 || op == 0x0C56 || op == 0x0C18) { f.cc(op, { 0x00 }); return; }
    if (op == 0x0405) { f.cs(op);
        std::vector<uint8_t> r = { (uint8_t)((g_present && memcmp(prm.data(), g_present, 6) == 0) ? 0x00 : 0x04), 0x01, 0x00 };
        r.insert(r.end(), prm.begin(), prm.begin() + 6); r.push_back(0x01); r.push_back(0x00); f.ev(0x03, r); return; }
    if (op == 0x0406) { f.cs(op); f.ev(0x05, { 0x00, prm[0], prm[1], 0x16 }); return; }
    if (op == 0x0411) { f.cs(op, 0x0C); return; }                                              // refused: the walk is over, connect() fails at pairing
    f.cc(op, { 0x01 });
}
struct Rig { FakeIo io; Hci hci; HciTransport *unused = nullptr; A2dpSource src; BondTable bonds;
    Rig() : hci(io), src(hci, io) { g_io = &io; g_hci = &hci; hci.onEvent(evThunk, &src); src.setLog(logFn, nullptr); io.onCmd = controller; g_log.clear(); } };
int main() {
    {   // 1. No table: today's behaviour -- inquiry, then the hit is paged.
        Rig r; g_present = SHOKZ;
        CHECK(r.src.connect("Shokz", 0, fakeNow, idle10) == A2dpSource::PAIR_FAILED);
        CHECK(r.io.count(0x0401) == 1 && r.io.count(0x0405) == 1 && logCount("bond_try") == 0);
    }
    {   // 2. One bond, present, name matches: paged DIRECTLY, no inquiry, PAGE_ATTEMPTS granted (one used).
        Rig r; r.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz")); r.src.setBonds(&r.bonds); g_present = SHOKZ;
        CHECK(r.src.connect("Shokz", 0, fakeNow, idle10) == A2dpSource::PAIR_FAILED);
        CHECK(r.io.count(0x0401) == 0 && r.io.count(0x0405) == 1);
        CHECK(logCount("bond_try: bd=C0:86:B3:31:29:2F name=\"OpenMove by Shokz\" attempts=3") == 1);
    }
    {   // 3. Two bonds, most recent first; the most recent is ABSENT: PAGE_ATTEMPTS pages of it, ONE of the
        //    next, which answers.  No inquiry.
        Rig r; r.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz")); r.bonds.upsert(mk(SINK, "EVKB-SINK")); r.src.setBonds(&r.bonds); g_present = SHOKZ;
        CHECK(r.src.connect(nullptr, 0, fakeNow, idle10) == A2dpSource::PAIR_FAILED);
        std::vector<std::vector<uint8_t> > p = r.io.pages();
        CHECK(p.size() == 4 && memcmp(p[0].data(), SINK, 6) == 0 && memcmp(p[2].data(), SINK, 6) == 0 && memcmp(p[3].data(), SHOKZ, 6) == 0);
        CHECK(r.io.count(0x0401) == 0 && logCount("attempts=3") == 1 && logCount("attempts=1") == 1);
    }
    {   // 4. The target-name filter skips a bond whose name does not match, even when it is most recent --
        //    and the FIRST PAGED candidate is the one that gets PAGE_ATTEMPTS.
        Rig r; r.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz")); r.bonds.upsert(mk(SINK, "EVKB-SINK")); r.src.setBonds(&r.bonds); g_present = SHOKZ;
        CHECK(r.src.connect("Shokz", 0, fakeNow, idle10) == A2dpSource::PAIR_FAILED);
        std::vector<std::vector<uint8_t> > p = r.io.pages();
        CHECK(p.size() == 1 && memcmp(p[0].data(), SHOKZ, 6) == 0 && logCount("EVKB-SINK") == 0 && logCount("attempts=3") == 1);
    }
    {   // 5. An EMPTY stored name is a wildcard, not a dead slot: it is paged (once, after the named match).
        Rig r; r.bonds.upsert(mk(SINK, "")); r.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz")); r.src.setBonds(&r.bonds); g_present = nullptr;
        CHECK(r.src.connect("Shokz", 0, fakeNow, idle10) == A2dpSource::CONNECT_FAILED);   // nobody answers: falls to inquiry, whose hit is absent too
        std::vector<std::vector<uint8_t> > p = r.io.pages();
        CHECK(p.size() >= 4 && memcmp(p[3].data(), SINK, 6) == 0);                            // 3 x SHOKZ, then the nameless bond once
        CHECK(r.io.count(0x0401) == 1 && logCount("bond_page=none -> inquiry") == 1);
    }
    {   // 6. Bonds present but none answers: the inquiry fallback runs on the SAME attempt and finds the hit.
        Rig r; r.bonds.upsert(mk(SINK, "EVKB-SINK")); r.src.setBonds(&r.bonds); g_present = SHOKZ;
        CHECK(r.src.connect(nullptr, 0, fakeNow, idle10) == A2dpSource::PAIR_FAILED);
        CHECK(r.io.count(0x0401) == 1 && logCount("bond_page=none -> inquiry") == 1);
        std::vector<std::vector<uint8_t> > p = r.io.pages();
        CHECK(p.size() == 4 && memcmp(p[0].data(), SINK, 6) == 0 && memcmp(p[3].data(), SHOKZ, 6) == 0);
    }
    printf("a2dpsource_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
