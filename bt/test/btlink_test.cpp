// Host tests for BtLink's connect path against a scripted controller: the paging fixes for
// the headset bench (Write_Page_Timeout, no role switch, Create_Connection_Cancel when the
// controller never completes, a retry on Page Timeout) and disconnect().  Every reply the
// fake sends is the byte layout of the event it stands for (Core 5.2 Vol 4 Part E 7.7).
#include "BtLink.h"
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
    std::vector<std::pair<uint16_t, std::vector<uint8_t> > > cmds;                 // every complete command the host wrote
    std::function<void(FakeIo &, uint16_t, const std::vector<uint8_t> &)> onCmd;   // the scenario
    size_t write(const uint8_t *p, size_t n) override { acc.insert(acc.end(), p, p + n); parse(); return n; }
    void parse() {                                                                  // Hci writes header and params separately
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
    const std::vector<uint8_t> *last(uint16_t op) { for (size_t i = cmds.size(); i-- > 0;) if (cmds[i].first == op) return &cmds[i].second; return nullptr; }
    int indexOf(uint16_t op) { for (size_t i = 0; i < cmds.size(); i++) if (cmds[i].first == op) return (int)i; return -1; }
};
static FakeIo *g_io = nullptr; static Hci *g_hci = nullptr;
static void idle10() { g_io->now += 10; g_hci->service(); }
static uint32_t fakeNow() { return g_io->now; }
static void evThunk(void *ctx, uint8_t code, const uint8_t *p, uint8_t len) { ((BtLink *)ctx)->onEvent(code, p, len); }
static std::vector<std::string> g_log;
static void logFn(void *, const char *line) { g_log.push_back(std::string(line)); }
static const uint8_t BD[6] = { 0x2F, 0x29, 0x31, 0xB3, 0x86, 0xC0 };            // the Shokz, LE byte order
// The common preamble every scenario answers the same way: inquiry (one A/V hit), remote name, event mask, SSP mode.
static bool preamble(FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
    if (op == 0x0401) { f.cs(op);
        std::vector<uint8_t> r = { 1 }; r.insert(r.end(), BD, BD + 6); r.push_back(1); r.push_back(0); r.push_back(0);   // psrm R1, reserved
        r.push_back(0x18); r.push_back(0x04); r.push_back(0x24); r.push_back(0x54); r.push_back(0x08);                  // cod 0x240418 (A/V), clk 0x0854
        f.ev(0x02, r); f.ev(0x01, { 0x00 }); return true; }
    if (op == 0x0419) { f.cs(op); std::vector<uint8_t> r = { 0x00 }; r.insert(r.end(), prm.begin(), prm.begin() + 6);
        const char *nm = "OpenMove by Shokz"; r.insert(r.end(), nm, nm + strlen(nm)); r.resize(7 + 248, 0); f.ev(0x07, r); return true; }
    if (op == 0x0C01 || op == 0x0C56 || op == 0x0C18) { f.cc(op, { 0x00 }); return true; }
    if (op == 0x1009) { std::vector<uint8_t> r = { 0x00 }; r.insert(r.end(), BD, BD + 6); f.cc(op, r); return true; }   // Read_BD_ADDR: the "is the link usable" probe
    return false;
}
static std::vector<uint8_t> connComplete(uint8_t status, uint16_t h = 0x0001) {
    std::vector<uint8_t> r = { status, (uint8_t)h, (uint8_t)(h >> 8) }; r.insert(r.end(), BD, BD + 6); r.push_back(0x01); r.push_back(0x00); return r; }
int main() {
    {   // 1. The controller answers Create_Connection with Command Status and then NOTHING (the bench's
        //    "connect=timeout (no Connection_Complete)"): BtLink must (a) have written Write_Page_Timeout 0x2000
        //    before paging, (b) page with role switch NOT allowed (the Mac's Create_Connection to this headset:
        //    ... 18 CC 01 00 54 88 00), (c) send Create_Connection_Cancel for the target so the controller stops
        //    paging and re-reports its credit count, then retry the page, and (d) leave the HCI usable afterwards.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); return; }                                  // ... and silence
            if (op == 0x0408) { std::vector<uint8_t> r = { 0x00 }; r.insert(r.end(), prm.begin(), prm.begin() + 6); f.cc(op, r); f.ev(0x03, connComplete(0x02)); return; }
            f.cc(op, { 0x01 });
        };
        BtLink::Result res = link.connect("Shokz", fakeNow, idle10);
        CHECK(res == BtLink::TIMEOUT);
        const std::vector<uint8_t> *pt = io.last(0x0C18);
        CHECK(pt && pt->size() == 2 && (*pt)[0] == 0x00 && (*pt)[1] == 0x20);       // 0x2000 slots = 5.12 s
        CHECK(io.indexOf(0x0C18) >= 0 && io.indexOf(0x0C18) < io.indexOf(0x0405));   // written BEFORE the first page
        const std::vector<uint8_t> *cc = io.last(0x0405);
        CHECK(cc && cc->size() == 13 && memcmp(cc->data(), BD, 6) == 0 && (*cc)[6] == 0x18 && (*cc)[7] == 0xCC && (*cc)[12] == 0x00);
        CHECK(io.count(0x0408) >= 1);
        const std::vector<uint8_t> *cx = io.last(0x0408);
        CHECK(cx && cx->size() == 6 && memcmp(cx->data(), BD, 6) == 0);
        CHECK(io.count(0x0405) == BtLink::PAGE_ATTEMPTS && io.count(0x0408) == BtLink::PAGE_ATTEMPTS);   // each silent page was cancelled
        Hci::Reply r; CHECK(hci.run(0x1009, nullptr, 0, &r, 500, idle10) == Hci::OK);   // the link is not wedged: a later command runs
        CHECK(hci.starved() == 0);
        bool sawCancelLog = false; for (auto &l : g_log) if (l.find("Create_Connection_Cancel") != std::string::npos) sawCancelLog = true;
        CHECK(sawCancelLog);
    }
    {   // 2. Worse: the Command Status for Create_Connection carries Num_HCI_Command_Packets = 0 and no NOP ever
        //    returns the credit (the bench's ncmd_starved wedge after a silent page).  The timeout path must RECLAIM
        //    the credit so the cancel can leave at all -- otherwise every later command starves, by name, forever.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op, 0x00, /*ncmd*/ 0); return; }
            if (op == 0x0408) { std::vector<uint8_t> r = { 0x00 }; r.insert(r.end(), prm.begin(), prm.begin() + 6); f.cc(op, r); f.ev(0x03, connComplete(0x02)); return; }
            f.cc(op, { 0x01 });
        };
        BtLink::Result res = link.connect("Shokz", fakeNow, idle10);
        CHECK(res == BtLink::TIMEOUT);
        CHECK(io.count(0x0408) >= 1);                                                // the cancel reached the wire
        CHECK(hci.reclaimed() >= 1 && hci.starved() == 0);
        Hci::Reply r; CHECK(hci.run(0x1009, nullptr, 0, &r, 500, idle10) == Hci::OK);
    }
    {   // 3. Page Timeout (Connection_Complete status 0x04) on the first page, success on the second: connect() retries
        //    the page itself -- no fresh inquiry -- and returns OK with the handle.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        static int pages; pages = 0;
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(++pages == 1 ? 0x04 : 0x00)); return; }
            f.cc(op, { 0x01 });
        };
        CHECK(link.connect("Shokz", fakeNow, idle10) == BtLink::OK);
        CHECK(io.count(0x0405) == 2 && io.count(0x0401) == 1 && io.count(0x0408) == 0);
        CHECK(link.handle() == 0x0001);
        // 4. disconnect(): HCI_Disconnect(handle, 0x13 remote user terminated) -> Command Status -> Disconnection_Complete.
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (op == 0x0406) { f.cs(op); f.ev(0x05, { 0x00, prm[0], prm[1], 0x16 }); return; }
            f.cc(op, { 0x01 });
        };
        CHECK(link.disconnect(fakeNow, idle10) == BtLink::OK);
        const std::vector<uint8_t> *d = io.last(0x0406);
        CHECK(d && d->size() == 3 && (*d)[0] == 0x01 && (*d)[1] == 0x00 && (*d)[2] == 0x13);
        CHECK(link.handle() == 0 && !link.encrypted());
        CHECK(link.disconnect(fakeNow, idle10) == BtLink::OK && io.count(0x0406) == 1);   // nothing to do twice
    }
    {   // 5. A page that is REJECTED outright (Command Status non-zero) still fails fast, by name, with no cancel.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op, 0x0B); return; }                             // ACL Connection Already Exists
            f.cc(op, { 0x01 });
        };
        CHECK(link.connect("Shokz", fakeNow, idle10) == BtLink::TIMEOUT);
        CHECK(io.count(0x0405) == 1 && io.count(0x0408) == 0);
    }
    printf("btlink_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
