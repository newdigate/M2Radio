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

    // --- NEW-41 scaffolding (scenarios I1-I3).  The scenarios above script the controller reply by
    // reply through onCmd; the identity/scan/incoming-page arms care only about WHAT was written, so
    // they use a generic answerer instead.  TEST SCAFFOLDING, not library behaviour. ---
    size_t answered = 0;                                                            // how far answerAll() has replied
    bool sawCommand(uint16_t op) { return indexOf(op) >= 0; }
    std::vector<uint8_t> paramsOf(uint16_t op) { int i = indexOf(op); return i >= 0 ? cmds[(size_t)i].second : std::vector<uint8_t>(); }
    std::vector<uint8_t> lastParamsOf(uint16_t op) { const std::vector<uint8_t> *p = last(op); return p ? *p : std::vector<uint8_t>(); }
    size_t countOf(uint16_t op) { return (size_t)count(op); }
    void deliver(const std::vector<uint8_t> &raw) { rx.insert(rx.end(), raw.begin(), raw.end()); }   // raw H4 bytes (0x04 = event)
    // Command Complete, status 0, in the return shape the scenarios above use for that opcode: the
    // bd-echoing replies carry the command's BD_ADDR, everything else is a bare status byte.  (The
    // commands these arms write are all Command-Complete commands; a Command-Status one -- Create_
    // Connection, Authentication_Requested -- would need its own arm, as scenarios 1-19 give it.)
    void answerOne(uint16_t op, const std::vector<uint8_t> &prm) {
        switch (op) {
            case 0x0409: case 0x040A: case 0x040B: case 0x040C: case 0x040D: case 0x042B: case 0x042C: {
                std::vector<uint8_t> r = { 0x00 };
                if (prm.size() >= 6) r.insert(r.end(), prm.begin(), prm.begin() + 6);
                cc(op, r); break; }
            default: cc(op, { 0x00 }); break;
        }
    }
    // Answer every command not yet answered, servicing between each: Hci holds ONE command in flight,
    // so a reply is what lets the next queued command reach the wire (and PREPARE's next sub-state run).
    void answerAll(Hci &h) {
        h.service();
        for (int guard = 0; guard < 64 && answered < cmds.size(); guard++) {
            uint16_t op = cmds[answered].first; std::vector<uint8_t> prm = cmds[answered].second;
            answered++;
            answerOne(op, prm);
            h.service();
        }
    }
};
// Drive an in-flight op (PREPARE, in these arms) to completion: tick, answer, advance the fake clock.
static void drivePrepare(FakeIo &io, Hci &hci, BtLink &l) {
    for (int i = 0; i < 200 && l.busy(); i++) { l.tick(io.now); io.answerAll(hci); io.now += 10; }
}
static FakeIo *g_io = nullptr; static Hci *g_hci = nullptr;
// NEW-34 piece 2: the engine is driven by tick(now).  idle10() also ticks the link under test when
// one is registered (g_link), so runUntil() below can drive an op directly; the blocking wrappers the
// scenarios below call drive their OWN tick() loop and leave g_link null, so there is no double-tick.
static BtLink *g_link = nullptr;
static void idle10() { g_io->now += 10; g_hci->service(); if (g_link) g_link->tick(g_io->now); }
// Drive time+service+tick until pred() or a fake-time budget elapses; returns pred()'s final value.
static bool runUntil(std::function<bool()> pred, uint32_t ms) {
    uint32_t end = g_io->now + ms;
    while (g_io->now < end) { if (pred()) return true; idle10(); }
    return pred();
}
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
static const uint8_t KEY1[16] = { 0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F };
static const uint8_t KEY2[16] = { 0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F };
static void seedBond(BondTable &t, const uint8_t *key, const char *name) {
    Bond b; memset(&b, 0, sizeof b); memcpy(b.bd, BD, 6); memcpy(b.key, key, 16); b.keyType = 4; b.psrm = 1;
    BondTable::copyName(b.name, name); t.upsert(b); t.clearDirty();
}
static std::vector<uint8_t> withBd(std::vector<uint8_t> head, const std::vector<uint8_t> &prm) { head.insert(head.end(), prm.begin(), prm.begin() + 6); return head; }
// The SSP Just-Works dance a controller runs after a NEGATIVE link-key reply, ending in a
// Link_Key_Notification carrying `key` and a successful Authentication_Complete.
static bool sspDance(FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm, const uint8_t *key) {
    std::vector<uint8_t> bd(BD, BD + 6);
    if (op == 0x040C) { f.cc(op, withBd({ 0x00 }, prm)); f.ev(0x31, bd); return true; }                        // -> IO_Capability_Request
    if (op == 0x042B) { f.cc(op, withBd({ 0x00 }, prm));
                        std::vector<uint8_t> rsp = bd; rsp.push_back(0x03); rsp.push_back(0x00); rsp.push_back(0x04); f.ev(0x32, rsp);
                        std::vector<uint8_t> uc = bd; uc.push_back(0x40); uc.push_back(0xE2); uc.push_back(0x01); uc.push_back(0x00); f.ev(0x33, uc); return true; }
    if (op == 0x042C) { f.cc(op, withBd({ 0x00 }, prm));
                        std::vector<uint8_t> spc = { 0x00 }; spc.insert(spc.end(), BD, BD + 6); f.ev(0x36, spc);
                        std::vector<uint8_t> lk = bd; lk.insert(lk.end(), key, key + 16); lk.push_back(0x04); f.ev(0x18, lk);
                        f.ev(0x06, { 0x00, 0x01, 0x00 }); return true; }
    return false;
}
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
        CHECK((*cc)[8] == 0x01 && (*cc)[9] == 0x00);        // psrm from the hit (R1), reserved 0 -- pins the argument order of page()
        CHECK((*cc)[10] == 0x54 && (*cc)[11] == 0x88);      // clk 0x0854 with bit 15 VALID: connect() passes clkValid=true
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
    {   // 6. The catch-all event trace suppresses Number_Of_Completed_Packets (0x13) -- it arrives
        //    at media rate during streaming and floods the injected console, throttling the caller's
        //    main loop (silicon 2026-09-04, acid_box) -- while STILL tracing any other unhandled code.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        uint8_t ncp[] = { 0x01, 0x01, 0x00, 0x05, 0x00 };   // 1 handle 0x0001, 5 packets completed
        link.onEvent(0x13, ncp, sizeof(ncp));
        uint8_t other[] = { 0xAA, 0xBB };
        link.onEvent(0x2F, other, sizeof(other));           // an unhandled code that is NOT 0x13
        bool sawNcp = false, sawOther = false;
        for (auto &l : g_log) {
            if (l.find("hci_event: code=0x13") != std::string::npos) sawNcp = true;
            if (l.find("hci_event: code=0x2F") != std::string::npos) sawOther = true;
        }
        CHECK(!sawNcp);                                     // the flood is suppressed
        CHECK(sawOther);                                    // genuine unknowns are still traced
    }
    {   // 7. EVERY page reports Page Timeout (Connection_Complete status 0x04): the loop exhausts
        //    `attempts` and reports the controller's status -- CONNECT_STATUS, as connect() has
        //    always done (the 0x04 line's `attempt < attempts` guard falls through to the generic
        //    non-zero-status line).  Pins the result code the Task 3 pure-refactor correction
        //    restored: a version returning TIMEOUT here passes every other arm (measured).
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x04)); return; }
            f.cc(op, { 0x01 });
        };
        CHECK(link.connect("Shokz", fakeNow, idle10) == BtLink::CONNECT_STATUS);
        CHECK(io.count(0x0405) == BtLink::PAGE_ATTEMPTS);   // all attempts used
        CHECK(io.count(0x0408) == 0);                        // a REPORTED Page Timeout needs no cancel
        CHECK(io.count(0x0401) == 1);                        // no fresh inquiry between pages
        CHECK(link.handle() == 0);                           // a FAILED Connection_Complete must not latch a handle (the fake sends 0x0001 even on 0x04)
    }
    {   // 8. NEW-34: a bonded page (no inquiry, clock offset invalid) + a stored-key authentication:
        //    Link_Key_Request is answered with Link_Key_Request_Reply carrying the EXACT stored key; no
        //    negative reply and no IO-capability dance follow; pairedBy() reads "stored"; encryption comes up.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        BondTable bonds; seedBond(bonds, KEY1, "OpenMove by Shokz");
        { Bond o; memset(&o, 0, sizeof o); const uint8_t obd[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 }; memcpy(o.bd, obd, 6); memset(o.key, 0x99, 16); o.keyType = 4; o.psrm = 1;
          BondTable::copyName(o.name, "OtherHeadset"); bonds.upsert(o); bonds.clearDirty(); }      // the target is now index 1: a stored-key success must move it to the front
        link.setBonds(&bonds);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x00)); return; }
            if (op == 0x0411) { f.cs(op); f.ev(0x17, std::vector<uint8_t>(BD, BD + 6)); return; }          // Link_Key_Request
            if (op == 0x040B) { f.cc(op, withBd({ 0x00 }, prm)); f.ev(0x06, { 0x00, 0x01, 0x00 }); return; }   // key matched -> Auth Complete ok
            if (op == 0x0413) { f.cs(op); f.ev(0x08, { 0x00, 0x01, 0x00, 0x01 }); return; }
            f.cc(op, { 0x01 });
        };
        CHECK(link.page(BD, 1, 0, false, "OpenMove by Shokz", 1, fakeNow, idle10) == BtLink::OK);
        CHECK(io.count(0x0401) == 0);                                                     // no inquiry
        const std::vector<uint8_t> *cc = io.last(0x0405);
        CHECK(cc && cc->size() == 13 && memcmp(cc->data(), BD, 6) == 0 && (*cc)[8] == 1 && (*cc)[10] == 0x00 && (*cc)[11] == 0x00);   // psrm R1, clock offset 0 / INVALID
        CHECK(io.count(0x0405) == 1);                                                     // `attempts` honoured
        CHECK(link.pairAndEncrypt(fakeNow, idle10) == BtLink::OK);
        const std::vector<uint8_t> *kr = io.last(0x040B);
        CHECK(kr && kr->size() == 22 && memcmp(kr->data(), BD, 6) == 0 && memcmp(kr->data() + 6, KEY1, 16) == 0);
        CHECK(io.count(0x040B) == 1 && io.count(0x040C) == 0 && io.count(0x042B) == 0 && io.count(0x040D) == 0 && io.count(0x0411) == 1);
        CHECK(strcmp(link.pairedBy(), "stored") == 0 && link.encrypted());
        CHECK(bonds.count() == 2 && memcmp(bonds.at(0).bd, BD, 6) == 0 && bonds.dirty());   // touch(): the reconnected peer is most recent again, and the host will persist it
    }
    {   // 9. NEW-34: the peer REJECTS the stored key (Authentication_Complete 0x06, PIN or Key Missing): the bond
        //    is erased, ONE more Authentication_Requested runs with SSP still on, its Link_Key_Request now gets the
        //    negative reply, the SSP dance yields a NEW key, and the table holds that key under the paged name.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        BondTable bonds; seedBond(bonds, KEY1, "OpenMove by Shokz"); link.setBonds(&bonds);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x00)); return; }
            if (op == 0x0411) { f.cs(op); f.ev(0x17, std::vector<uint8_t>(BD, BD + 6)); return; }
            if (op == 0x040B) { f.cc(op, withBd({ 0x00 }, prm)); f.ev(0x06, { 0x06, 0x01, 0x00 }); return; }   // PIN or Key Missing
            if (sspDance(f, op, prm, KEY2)) return;
            if (op == 0x0413) { f.cs(op); f.ev(0x08, { 0x00, 0x01, 0x00, 0x01 }); return; }
            f.cc(op, { 0x01 });
        };
        CHECK(link.page(BD, 2, 0, false, "OpenMove by Shokz", 1, fakeNow, idle10) == BtLink::OK);   // psrm R2: the re-paired bond must store the mode we paged with
        CHECK(link.pairAndEncrypt(fakeNow, idle10) == BtLink::OK);
        CHECK(io.count(0x0411) == 2 && io.count(0x040B) == 1 && io.count(0x040C) == 1 && io.count(0x042B) == 1);
        CHECK(io.count(0x0C56) == 1);                                                     // SSP mode written ONCE (by page): the SSP-off fallback did not run
        const Bond *b = bonds.find(BD);
        CHECK(b && memcmp(b->key, KEY2, 16) == 0 && b->keyType == 4 && b->psrm == 2 && strcmp(b->name, "OpenMove by Shokz") == 0 && bonds.dirty());
        CHECK(strcmp(link.pairedBy(), "ssp") == 0 && link.encrypted());
        bool sawReject = false; for (auto &l : g_log) if (l.find("bond_rejected: status=0x06 -> erased") != std::string::npos) sawReject = true;
        CHECK(sawReject);
    }
    {   // 10. NEW-34: a stored-key authentication that fails for a TRANSIENT reason (0x08 Connection Timeout) keeps
        //     the bond, runs no second Authentication_Requested and no PIN fallback, and fails by name.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        BondTable bonds; seedBond(bonds, KEY1, "OpenMove by Shokz"); link.setBonds(&bonds);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x00)); return; }
            if (op == 0x0411) { f.cs(op); f.ev(0x17, std::vector<uint8_t>(BD, BD + 6)); return; }
            if (op == 0x040B) { f.cc(op, withBd({ 0x00 }, prm)); f.ev(0x06, { 0x08, 0x01, 0x00 }); return; }   // Connection Timeout
            f.cc(op, { 0x01 });
        };
        CHECK(link.page(BD, 1, 0, false, "OpenMove by Shokz", 1, fakeNow, idle10) == BtLink::OK);
        CHECK(link.pairAndEncrypt(fakeNow, idle10) == BtLink::PAIRING_FAILED);
        CHECK(io.count(0x0411) == 1 && io.count(0x040C) == 0 && io.count(0x0C56) == 1 && io.count(0x0413) == 0);
        CHECK(bonds.find(BD) != nullptr && !bonds.dirty());
    }
    {   // 11. No table set (the default): Link_Key_Request still gets the negative reply -- byte-identical to
        //     before NEW-34, which is what keeps every existing gate's wire sequence unchanged.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) { f.cc(op, withBd({ 0x00 }, prm)); };
        std::vector<uint8_t> bd(BD, BD + 6);
        link.onEvent(0x17, bd.data(), 6); idle10(); idle10();
        CHECK(io.count(0x040C) == 1 && io.count(0x040B) == 0);
    }
    {   // 12. NEW-34: a FRESH pairing through connect() (inquiry -> page -> SSP) saves the bond with the key from
        //     the notification, the psrm we paged with and the name from the INQUIRY HIT.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        BondTable bonds; link.setBonds(&bonds);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x00)); return; }
            if (op == 0x0411) { f.cs(op); f.ev(0x17, std::vector<uint8_t>(BD, BD + 6)); return; }
            if (sspDance(f, op, prm, KEY1)) return;
            if (op == 0x0413) { f.cs(op); f.ev(0x08, { 0x00, 0x01, 0x00, 0x01 }); return; }
            f.cc(op, { 0x01 });
        };
        CHECK(link.connect("Shokz", fakeNow, idle10) == BtLink::OK);
        CHECK(link.pairAndEncrypt(fakeNow, idle10) == BtLink::OK);
        CHECK(io.count(0x040B) == 0 && io.count(0x040C) == 1 && strcmp(link.pairedBy(), "ssp") == 0);
        const Bond *b = bonds.find(BD);
        CHECK(b && memcmp(b->key, KEY1, 16) == 0 && b->keyType == 4 && b->psrm == 1 && strcmp(b->name, "OpenMove by Shokz") == 0);
        CHECK(bonds.count() == 1 && bonds.dirty());
        bool sawSaved = false; for (auto &l : g_log) if (l.find("bond=saved") != std::string::npos) sawSaved = true;
        CHECK(sawSaved);
    }
    {   // 13. NEW-34: the peer rejects with 0x05 (Authentication Failure -- it holds a DIFFERENT key for us): the
        //     same erase-and-re-pair rung as 0x06.  (Mutation-found: with only 0x06 in the condition the suite stayed green.)
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        BondTable bonds; seedBond(bonds, KEY1, "OpenMove by Shokz"); link.setBonds(&bonds);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x00)); return; }
            if (op == 0x0411) { f.cs(op); f.ev(0x17, std::vector<uint8_t>(BD, BD + 6)); return; }
            if (op == 0x040B) { f.cc(op, withBd({ 0x00 }, prm)); f.ev(0x06, { 0x05, 0x01, 0x00 }); return; }   // Authentication Failure
            if (sspDance(f, op, prm, KEY2)) return;
            if (op == 0x0413) { f.cs(op); f.ev(0x08, { 0x00, 0x01, 0x00, 0x01 }); return; }
            f.cc(op, { 0x01 });
        };
        CHECK(link.page(BD, 1, 0, false, "OpenMove by Shokz", 1, fakeNow, idle10) == BtLink::OK);
        CHECK(link.pairAndEncrypt(fakeNow, idle10) == BtLink::OK);
        const Bond *b = bonds.find(BD);
        CHECK(b && memcmp(b->key, KEY2, 16) == 0 && strcmp(link.pairedBy(), "ssp") == 0);
        bool sawReject = false; for (auto &l : g_log) if (l.find("bond_rejected: status=0x05 -> erased") != std::string::npos) sawReject = true;
        CHECK(sawReject);
    }
    {   // 13b. NEW-34 piece 5 soak finding (ESP32 sink, 2026-09-07): the peer rejects with 0x24 (LMP PDU Not Allowed -- it forgot us across a power-cycle and its LMP refuses the combination-key auth): the
        //     same erase-and-re-pair rung as 0x06.  (Mutation-found: with only 0x06 in the condition the suite stayed green.)
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        BondTable bonds; seedBond(bonds, KEY1, "OpenMove by Shokz"); link.setBonds(&bonds);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x00)); return; }
            if (op == 0x0411) { f.cs(op); f.ev(0x17, std::vector<uint8_t>(BD, BD + 6)); return; }
            if (op == 0x040B) { f.cc(op, withBd({ 0x00 }, prm)); f.ev(0x06, { 0x24, 0x01, 0x00 }); return; }   // Authentication Failure
            if (sspDance(f, op, prm, KEY2)) return;
            if (op == 0x0413) { f.cs(op); f.ev(0x08, { 0x00, 0x01, 0x00, 0x01 }); return; }
            f.cc(op, { 0x01 });
        };
        CHECK(link.page(BD, 1, 0, false, "OpenMove by Shokz", 1, fakeNow, idle10) == BtLink::OK);
        CHECK(link.pairAndEncrypt(fakeNow, idle10) == BtLink::OK);
        const Bond *b = bonds.find(BD);
        CHECK(b && memcmp(b->key, KEY2, 16) == 0 && strcmp(link.pairedBy(), "ssp") == 0);
        bool sawReject = false; for (auto &l : g_log) if (l.find("bond_rejected: status=0x24 -> erased") != std::string::npos) sawReject = true;
        CHECK(sawReject);
    }
    {   // 14. NEW-34: a Link_Key_Request for a bonded address that is NOT the one we paged is answered from the
        //     table (a controller is entitled to ask), but must NOT mark a key as offered for THIS link: the rung
        //     acts on m_bd, so otherwise a rejection would erase the WRONG bond and re-offer the rejected one.
        //     (Demonstrated in review before this arm existed: peer A's bond erased for peer B's rejection.)
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        static const uint8_t OBD[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
        BondTable bonds; seedBond(bonds, KEY1, "OpenMove by Shokz");
        { Bond o; memset(&o, 0, sizeof o); memcpy(o.bd, OBD, 6); memcpy(o.key, KEY2, 16); o.keyType = 4; o.psrm = 1; BondTable::copyName(o.name, "OtherHeadset"); bonds.upsert(o); bonds.clearDirty(); }
        link.setBonds(&bonds);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x00)); return; }
            if (op == 0x0411) { f.cs(op); f.ev(0x17, std::vector<uint8_t>(OBD, OBD + 6)); return; }          // the controller asks for the OTHER peer's key
            if (op == 0x040B) { f.cc(op, withBd({ 0x00 }, prm)); f.ev(0x06, { 0x06, 0x01, 0x00 }); return; }   // ... and that authentication fails
            if (op == 0x0C56) { f.cc(op, { 0x00 }); return; }
            f.cc(op, { 0x01 });                                                                            // nothing else succeeds: the attempt fails
        };
        CHECK(link.page(BD, 1, 0, false, "OpenMove by Shokz", 1, fakeNow, idle10) == BtLink::OK);
        CHECK(link.pairAndEncrypt(fakeNow, idle10) != BtLink::OK);
        const std::vector<uint8_t> *kr = io.last(0x040B);
        CHECK(kr && kr->size() == 22 && memcmp(kr->data(), OBD, 6) == 0 && memcmp(kr->data() + 6, KEY2, 16) == 0);   // answered from the table
        CHECK(bonds.count() == 2 && bonds.find(BD) && bonds.find(OBD) && !bonds.dirty());                 // NEITHER bond erased, nothing to persist
        bool sawReject = false; for (auto &l : g_log) if (l.find("bond_rejected") != std::string::npos) sawReject = true;
        CHECK(!sawReject);
    }
    {   // 15. page() with attempts == 0 is clamped to one page (a zero would send the setup commands and report
        //     TIMEOUT having paged nothing).
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x00)); return; }
            f.cc(op, { 0x01 });
        };
        CHECK(link.page(BD, 1, 0, false, nullptr, 0, fakeNow, idle10) == BtLink::OK && io.count(0x0405) == 1);
    }
    {   // 16. The cancel RACE: the peer answers the page just as the 10 s wait expires, so the cancel gets
        //     Command Complete 0x02 (Unknown Connection Identifier -- the connection already completed) and
        //     a Connection_Complete status 0x00 lands during the post-cancel wait.  That is a LINK, and page()
        //     must say so -- a TIMEOUT here would let A2dpSource page the next candidate behind a live ACL.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); hci.onEvent(evThunk, &link); link.setLog(logFn, nullptr); g_log.clear();
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); return; }                                   // ... and silence
            if (op == 0x0408) { std::vector<uint8_t> r = { 0x02 }; r.insert(r.end(), prm.begin(), prm.begin() + 6); f.cc(op, r);   // cancel: too late
                                f.ev(0x03, connComplete(0x00, 0x0007)); return; }     // the page had completed
            f.cc(op, { 0x01 });
        };
        CHECK(link.page(BD, 1, 0, false, "OpenMove by Shokz", 3, fakeNow, idle10) == BtLink::OK);
        CHECK(io.count(0x0405) == 1 && io.count(0x0408) == 1 && link.handle() == 0x0007);
        bool sawRace = false; for (auto &l : g_log) if (l.find("raced the cancel") != std::string::npos) sawRace = true;
        CHECK(sawRace);
    }
    {   // 17. NEW-34 piece 2: the page-scan side channel, driven directly on the tick engine (g_link set).
        //     wantPageScan(true) writes Write_Scan_Enable 0x02 (page scan ONLY, not inquiry scan) and logs
        //     page_scan=on; wantPageScan(false) writes 0x00 and logs page_scan=off.  begin() alone (no
        //     wantPageScan) writes nothing -- the post-Reset scan state is known-off.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); g_link = &link; hci.onEvent(evThunk, &link);
        link.setLog(logFn, nullptr); g_log.clear();
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &) { f.cc(op, { 0x00 }); };
        link.begin(io.now);
        CHECK(!runUntil([&]{ return io.count(0x0C1A) >= 1; }, 200));   // no delta yet -> no unsolicited write
        link.wantPageScan(true);
        CHECK(runUntil([&]{ return io.count(0x0C1A) >= 1; }, 500));
        const std::vector<uint8_t> *w = io.last(0x0C1A);
        CHECK(w && w->size() == 1 && (*w)[0] == 0x02);                  // page scan only
        bool sawOn = false; for (auto &l : g_log) if (l.find("page_scan=on") != std::string::npos) sawOn = true;
        CHECK(sawOn);
        link.wantPageScan(false);
        CHECK(runUntil([&]{ return io.count(0x0C1A) >= 2; }, 500));
        const std::vector<uint8_t> *w2 = io.last(0x0C1A);
        CHECK(w2 && w2->size() == 1 && (*w2)[0] == 0x00);              // scan off
        bool sawOff = false; for (auto &l : g_log) if (l.find("page_scan=off") != std::string::npos) sawOff = true;
        CHECK(sawOff);
        g_link = nullptr;
    }
    {   // 18. NEW-34 piece 2: the engine driven DIRECTLY (as A2dpSource will in Task 5): startPrepare ->
        //     startPage (bonded, stored key, clock offset invalid) -> startPair, each advanced by runUntil,
        //     reaches OK with the same wire bytes and result as the blocking path (cf. scenario 8).
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); g_link = &link; hci.onEvent(evThunk, &link);
        link.setLog(logFn, nullptr); g_log.clear();
        BondTable bonds; seedBond(bonds, KEY1, "OpenMove by Shokz"); link.setBonds(&bonds);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x00)); return; }
            if (op == 0x0411) { f.cs(op); f.ev(0x17, std::vector<uint8_t>(BD, BD + 6)); return; }
            if (op == 0x040B) { f.cc(op, withBd({ 0x00 }, prm)); f.ev(0x06, { 0x00, 0x01, 0x00 }); return; }
            if (op == 0x0413) { f.cs(op); f.ev(0x08, { 0x00, 0x01, 0x00, 0x01 }); return; }
            f.cc(op, { 0x01 });
        };
        link.begin(io.now);
        CHECK(link.startPrepare()); CHECK(runUntil([&]{ return !link.busy(); }, 4000) && link.result() == BtLink::OK);
        CHECK(link.startPage(BD, 1, 0, false, "OpenMove by Shokz", 1));
        CHECK(runUntil([&]{ return !link.busy(); }, 20000) && link.result() == BtLink::OK);
        const std::vector<uint8_t> *cc = io.last(0x0405);
        CHECK(cc && cc->size() == 13 && (*cc)[8] == 1 && (*cc)[10] == 0x00 && (*cc)[11] == 0x00);   // psrm R1, clock offset 0/INVALID
        CHECK(link.handle() == 0x0001 && io.count(0x0405) == 1 && io.count(0x0401) == 0);
        CHECK(link.startPair(false));
        CHECK(runUntil([&]{ return !link.busy(); }, 40000) && link.result() == BtLink::OK);
        const std::vector<uint8_t> *kr = io.last(0x040B);
        CHECK(kr && kr->size() == 22 && memcmp(kr->data(), BD, 6) == 0 && memcmp(kr->data() + 6, KEY1, 16) == 0);
        CHECK(io.count(0x040B) == 1 && io.count(0x040C) == 0 && io.count(0x0411) == 1);
        CHECK(strcmp(link.pairedBy(), "stored") == 0 && link.encrypted());
        // startX refuses while an op runs
        CHECK(link.startPage(BD, 1, 0, false, "X", 1));                 // engine idle now -> accepted
        CHECK(!link.startInquiry("Y"));                                 // an op is running -> refused
        g_link = nullptr;
    }
    {   // 19. NEW-34 piece 2: driven directly, EVERY page reports Page Timeout (status 0x04) -> the engine
        //     exhausts attempts and finishes CONNECT_STATUS (the result code sub 4 restores; a finish(OK)
        //     there is the mutation this pins directly, beside scenario 7's wrapper form).
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); g_link = &link; hci.onEvent(evThunk, &link);
        link.setLog(logFn, nullptr); g_log.clear();
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x04)); return; }
            f.cc(op, { 0x01 });
        };
        link.begin(io.now);
        CHECK(link.startPage(BD, 1, 0, false, "X", BtLink::PAGE_ATTEMPTS));
        CHECK(runUntil([&]{ return !link.busy(); }, 20000) && link.result() == BtLink::CONNECT_STATUS);
        CHECK(io.count(0x0405) == BtLink::PAGE_ATTEMPTS && io.count(0x0408) == 0 && link.handle() == 0);
        g_link = nullptr;
    }
    {   // 20. Incoming page while idle with a bond: accept (role 0x01 remain slave), link comes up as incoming.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); g_link = &link; hci.onEvent(evThunk, &link);
        link.setLog(logFn, nullptr); g_log.clear(); link.begin(io.now);
        BondTable bonds; seedBond(bonds, KEY1, "OpenMove by Shokz"); link.setBonds(&bonds);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (op == 0x0409) { f.cc(op, withBd({ 0x00 }, prm)); f.ev(0x03, connComplete(0x00)); return; }   // Accept -> Connection_Complete
            f.cc(op, { 0x00 });
        };
        // Connection_Request: bd(6) cod(3) link_type(1 = ACL 0x01)
        std::vector<uint8_t> cr(BD, BD + 6); cr.push_back(0x04); cr.push_back(0x04); cr.push_back(0x24); cr.push_back(0x01);
        link.onEvent(0x04, cr.data(), (uint8_t)cr.size());
        CHECK(runUntil([&]{ return link.linkState() == BtLink::LINK_UP; }, 2000));
        const std::vector<uint8_t> *acc = io.last(0x0409);
        CHECK(acc && acc->size() == 7 && memcmp(acc->data(), BD, 6) == 0 && (*acc)[6] == 0x01);   // role 0x01 remain slave
        CHECK(link.incoming() && memcmp(link.peer(), BD, 6) == 0 && link.inboundUp());
        CHECK(io.count(0x040A) == 0);
    }
    {   // 21. Incoming page from an UNKNOWN address is rejected 0x0F (host rejected: unacceptable BD_ADDR); no link.
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); g_link = &link; hci.onEvent(evThunk, &link);
        link.setLog(logFn, nullptr); g_log.clear(); link.begin(io.now);
        BondTable bonds; seedBond(bonds, KEY1, "OpenMove by Shokz"); link.setBonds(&bonds);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) { f.cc(op, withBd({ 0x00 }, prm)); };
        static const uint8_t UNK[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
        std::vector<uint8_t> cr(UNK, UNK + 6); cr.push_back(0); cr.push_back(0); cr.push_back(0); cr.push_back(0x01);
        link.onEvent(0x04, cr.data(), (uint8_t)cr.size());
        CHECK(runUntil([&]{ return io.count(0x040A) >= 1; }, 500));
        const std::vector<uint8_t> *rej = io.last(0x040A);
        CHECK(rej && rej->size() == 7 && memcmp(rej->data(), UNK, 6) == 0 && (*rej)[6] == 0x0F);
        CHECK(link.linkState() == BtLink::LINK_NONE && io.count(0x0409) == 0);
    }
    {   // 22. Incoming page while a link is UP is rejected 0x0D (limited resources).
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); g_link = &link; hci.onEvent(evThunk, &link);
        link.setLog(logFn, nullptr); g_log.clear(); link.begin(io.now);
        BondTable bonds; seedBond(bonds, KEY1, "OpenMove by Shokz"); seedBond(bonds, KEY2, "Other"); link.setBonds(&bonds);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (op == 0x0409) { f.cc(op, withBd({ 0x00 }, prm)); f.ev(0x03, connComplete(0x00)); return; }
            f.cc(op, { 0x00 }); };
        std::vector<uint8_t> a(BD, BD + 6); a.push_back(0); a.push_back(0); a.push_back(0); a.push_back(0x01);
        link.onEvent(0x04, a.data(), (uint8_t)a.size());
        CHECK(runUntil([&]{ return link.linkState() == BtLink::LINK_UP; }, 2000));
        // a SECOND request while up: reject busy
        static const uint8_t B2[6] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22 };
        std::vector<uint8_t> b(B2, B2 + 6); b.push_back(0); b.push_back(0); b.push_back(0); b.push_back(0x01);
        link.onEvent(0x04, b.data(), (uint8_t)b.size());
        CHECK(runUntil([&]{ return io.count(0x040A) >= 1; }, 500));
        const std::vector<uint8_t> *rej = io.last(0x040A); CHECK(rej && (*rej)[6] == 0x0D);
    }
    {   // 23. Connection_Complete for a DIFFERENT address than the one we paged is IGNORED (not latched).
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); g_link = &link; hci.onEvent(evThunk, &link);
        link.setLog(logFn, nullptr); g_log.clear(); link.begin(io.now);
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op);
                std::vector<uint8_t> other = connComplete(0x00, 0x0009);
                static const uint8_t OTH[6] = { 0x77, 0x66, 0x55, 0x44, 0x33, 0x22 };
                memcpy(other.data() + 3, OTH, 6);                 // Connection_Complete for someone else
                f.ev(0x03, other); return; }
            f.cc(op, { 0x01 }); };
        link.startPrepare(); CHECK(runUntil([&]{ return !link.busy(); }, 4000));
        link.startPage(BD, 1, 0, false, "X", 1);
        CHECK(!runUntil([&]{ return !link.busy(); }, 3000));      // the foreign Connection_Complete does NOT complete our page
        CHECK(link.handle() == 0);
    }
    {   // 24. Link supervision timeout write, when master, after a link comes up (a bench knob for range-loss detection).
        FakeIo io; g_io = &io; Hci hci(io); g_hci = &hci; BtLink link(hci); g_link = &link; hci.onEvent(evThunk, &link);
        link.setLog(logFn, nullptr); g_log.clear(); link.begin(io.now); link.setSupervisionSlots(0x1F40);   // 5 s
        io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
            if (preamble(f, op, prm)) return;
            if (op == 0x0405) { f.cs(op); f.ev(0x03, connComplete(0x00)); return; }
            f.cc(op, { 0x00 }); };
        link.startPrepare(); CHECK(runUntil([&]{ return !link.busy(); }, 4000));
        link.startPage(BD, 1, 0, false, "X", 1);
        CHECK(runUntil([&]{ return link.linkState() == BtLink::LINK_UP; }, 12000));
        CHECK(runUntil([&]{ return io.count(0x0C37) >= 1; }, 500));
        const std::vector<uint8_t> *w = io.last(0x0C37);
        CHECK(w && w->size() == 4 && (*w)[2] == 0x40 && (*w)[3] == 0x1F);   // handle + 0x1F40 slots
    }
    {   // I1. IDENTITY: setIdentity(cod, name) makes PREPARE write Class_of_Device (0x0C24, 3 bytes LE) and
        //     Write_Local_Name (0x0C13, 248 bytes, NUL-padded) after Write_Page_Timeout; without it PREPARE is unchanged.
        FakeIo io; Hci hci(io); hci.begin(); BtLink l(hci); hci.onEvent(evThunk, &l); l.setLog(logFn, nullptr); g_log.clear(); l.begin(0);
        l.setIdentity(0x240414, "EVKB-SINK");
        l.startPrepare(); drivePrepare(io, hci, l);
        const std::vector<uint8_t> codLe = { 0x14, 0x04, 0x24 };                                     // 0x240414, little-endian
        CHECK(io.sawCommand(0x0C24) && io.paramsOf(0x0C24) == codLe);
        CHECK(io.sawCommand(0x0C13) && io.paramsOf(0x0C13).size() == 248 && memcmp(io.paramsOf(0x0C13).data(), "EVKB-SINK", 10) == 0);
        CHECK(io.indexOf(0x0C18) < io.indexOf(0x0C24) && io.indexOf(0x0C24) < io.indexOf(0x0C13));   // after Write_Page_Timeout, in that order
        FakeIo io2; Hci hci2(io2); hci2.begin(); BtLink l2(hci2); hci2.onEvent(evThunk, &l2); l2.begin(0); l2.startPrepare(); drivePrepare(io2, hci2, l2);
        CHECK(!io2.sawCommand(0x0C24) && !io2.sawCommand(0x0C13));
        CHECK(io2.sawCommand(0x0C01) && io2.sawCommand(0x0C56) && io2.sawCommand(0x0C18));           // the unchanged PREPARE
    }
    {   // I2. DISCOVERABLE: wantDiscoverable(true) reconciles Write_Scan_Enable to 0x03 (inquiry + page scan); page
        //     scan alone stays 0x02; both off -> 0x00.  Only real deltas are written.
        FakeIo io; Hci hci(io); hci.begin(); BtLink l(hci); hci.onEvent(evThunk, &l); l.setLog(logFn, nullptr); g_log.clear(); l.begin(0);
        const std::vector<uint8_t> both = { 0x03 }, pageOnly = { 0x02 }, none = { 0x00 };
        l.wantPageScan(true); l.wantDiscoverable(true); l.tick(1); io.answerAll(hci);
        CHECK(io.lastParamsOf(0x0C1A) == both);
        l.wantDiscoverable(false); l.tick(2); io.answerAll(hci);
        CHECK(io.lastParamsOf(0x0C1A) == pageOnly);
        size_t n = io.countOf(0x0C1A); l.tick(3); io.answerAll(hci); CHECK(io.countOf(0x0C1A) == n);
        l.wantPageScan(false); l.tick(4); io.answerAll(hci); CHECK(io.lastParamsOf(0x0C1A) == none);
        // The [lifecycle] gate's fake peer COUNTS page_scan= lines: an inquiry-scan-only delta must not emit one
        // (the page bit did not move), and the page-bit deltas must still log exactly as scenario 17 asserts.
        int onOff = 0; for (auto &s : g_log) if (s.find("page_scan=") != std::string::npos) onOff++;
        CHECK(onOff == 2);
    }
    {   // I3. ACCEPT STRANGERS: acceptUnknown(true) turns an incoming page from an unbonded address into an
        //     Accept_Connection_Request (slave, role 0x01); the default still rejects it 0x0F.
        FakeIo io; Hci hci(io); hci.begin(); BtLink l(hci); hci.onEvent(evThunk, &l); l.setLog(logFn, nullptr); g_log.clear(); l.begin(0);
        const uint8_t bd[6] = { 0x76, 0x1A, 0x7E, 0x8A, 0x0C, 0x00 };
        std::vector<uint8_t> req = { 0x04, 0x04, 10 }; req.insert(req.end(), bd, bd + 6); req.insert(req.end(), { 0x14, 0x04, 0x24, 0x01 });
        io.deliver(req); io.answerAll(hci);
        CHECK(io.paramsOf(0x040A).size() == 7 && io.paramsOf(0x040A)[6] == 0x0F);                       // rejected (unknown)
        l.acceptUnknown(true); io.deliver(req); io.answerAll(hci);
        CHECK(io.paramsOf(0x0409).size() == 7 && io.paramsOf(0x0409)[6] == 0x01);                       // accepted as slave
        // size-guarded, not `sawCommand(op) && paramsOf(op)[6]`: against a mutant that never writes the
        // command, an unguarded index/memcmp on the empty vector SEGFAULTS instead of failing by name.
        CHECK(io.paramsOf(0x0409).size() == 7 && memcmp(io.paramsOf(0x0409).data(), bd, 6) == 0 && l.incoming() && memcmp(l.peer(), bd, 6) == 0);
        CHECK(io.countOf(0x040A) == 1 && io.countOf(0x0409) == 1);   // one each: the knob decided, not a retry
    }
    printf("btlink_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
