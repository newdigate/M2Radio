// Host unit tests for Hci against a scripted HciIo.  Every failure path the
// [hci] gate later exercises against the Python peer is pinned here first:
// timeout, framing (garbage then reply), credit starvation, late reply.
#include "Hci.h"
#include <stdio.h>
#include <string.h>
#include <vector>
#include <deque>
#include <functional>

static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

struct FakeIo : HciIo {
    std::vector<uint8_t> tx;          // everything the host wrote
    std::deque<uint8_t>  rx;          // what the controller will deliver
    uint32_t now = 0;
    int writes = 0;
    std::function<void(FakeIo &, const std::vector<uint8_t> &)> onWrite;
    size_t write(const uint8_t *p, size_t n) override {
        std::vector<uint8_t> pkt(p, p + n);
        tx.insert(tx.end(), p, p + n);
        writes++;
        if (onWrite) onWrite(*this, pkt);
        return n;
    }
    int available() override { return (int)rx.size(); }
    int read() override { if (rx.empty()) return -1; uint8_t b = rx.front(); rx.pop_front(); return b; }
    uint32_t nowMs() override { return now; }
    void deliver(std::initializer_list<uint8_t> b) { rx.insert(rx.end(), b); }
};

static FakeIo *g_io = nullptr;
static void idle10() { g_io->now += 10; }

static const uint8_t RESET_CMD[] = { 0x01, 0x03, 0x0C, 0x00 };

int main() {
    {   // 1. Reset answered: OK, the H4 command bytes are right, one credit remains
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        io.onWrite = [](FakeIo &f, const std::vector<uint8_t> &) { f.deliver({0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00}); };
        Hci::Reply r;
        CHECK(hci.run(0x0C03, nullptr, 0, &r, 500, idle10) == Hci::OK);
        CHECK(io.tx.size() == 4 && memcmp(io.tx.data(), RESET_CMD, 4) == 0);
        CHECK(r.status == 0); CHECK(r.len == 0); CHECK(!r.statusEvent);
        CHECK(hci.ncmd() == 1); CHECK(hci.timeouts() == 0); CHECK(hci.lastError() == Hci::OK);
    }
    {   // 2. Command with parameters is framed [01][op lo][op hi][plen][params]
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        io.onWrite = [](FakeIo &f, const std::vector<uint8_t> &) { f.deliver({0x04, 0x0F, 0x04, 0x00, 0x01, 0x01, 0x04}); };
        const uint8_t p[5] = { 0x33, 0x8B, 0x9E, 0x08, 0x00 };
        Hci::Reply r;
        CHECK(hci.run(0x0401, p, 5, &r, 500, idle10) == Hci::OK);
        const uint8_t want[] = { 0x01, 0x01, 0x04, 0x05, 0x33, 0x8B, 0x9E, 0x08, 0x00 };
        CHECK(io.tx.size() == sizeof want && memcmp(io.tx.data(), want, sizeof want) == 0);
        CHECK(r.statusEvent); CHECK(r.status == 0);
    }
    {   // 3. No reply: TIMEOUT after the deadline, counted, named
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        Hci::Reply r;
        uint32_t t0 = io.now;
        CHECK(hci.run(0x0C03, nullptr, 0, &r, 500, idle10) == Hci::TIMEOUT);
        CHECK(io.now - t0 >= 500 && io.now - t0 < 600);
        CHECK(hci.timeouts() == 1); CHECK(r.status == 0xFF);
        CHECK(strcmp(Hci::errorName(Hci::TIMEOUT), "no_response") == 0);
    }
    {   // 4. Garbage then the reply in one burst: FRAMING now, and the NEXT command
        //    waits for the 50 ms quiet before it is sent, then succeeds.
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        io.onWrite = [](FakeIo &f, const std::vector<uint8_t> &) {
            if (f.writes == 1) f.deliver({0xFF, 0xFF, 0xFF, 0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00});
            else               f.deliver({0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00});
        };
        Hci::Reply r;
        CHECK(hci.run(0x0C03, nullptr, 0, &r, 500, idle10) == Hci::FRAMING);
        CHECK(hci.ncmd() == 1);   // the killed command's credit came back; without
                                  // this the link deadlocks -- the reply that would
                                  // have restored it was in the discarded burst
        CHECK(hci.framing() == 1);
        uint32_t t1 = io.now;
        CHECK(hci.run(0x0C03, nullptr, 0, &r, 500, idle10) == Hci::OK);
        CHECK(io.writes == 2);
        CHECK(io.now - t1 >= Hci::IDLE_RESYNC_MS);        // it waited for the line to go quiet
        CHECK(hci.timeouts() == 0);
    }
    {   // 5. A line that never goes quiet fails the waiting command as FRAMING, not TIMEOUT
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        io.deliver({0xFF});
        hci.service();                                      // fault -> resync
        CHECK(hci.framing() == 1);
        // keep one garbage byte arriving every 10 ms
        Hci::Reply r;
        static FakeIo *babble = nullptr; babble = &io;
        struct L { static void idle() { babble->now += 10; babble->deliver({0xFF}); } };
        CHECK(hci.run(0x0C03, nullptr, 0, &r, 200, L::idle) == Hci::FRAMING);
        CHECK(io.writes == 0);                              // never dispatched
        CHECK(hci.framing() == 1);                          // discarded bytes are not new faults
    }
    {   // 6. Credit starvation: Reset answered with ncmd=0, the next command starves
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        io.onWrite = [](FakeIo &f, const std::vector<uint8_t> &) { f.deliver({0x04, 0x0E, 0x04, 0x00, 0x03, 0x0C, 0x00}); };
        Hci::Reply r;
        CHECK(hci.run(0x0C03, nullptr, 0, &r, 500, idle10) == Hci::OK);
        CHECK(hci.ncmd() == 0);
        CHECK(hci.run(0x1001, nullptr, 0, &r, 300, idle10) == Hci::NCMD_STARVED);
        CHECK(hci.starved() == 1); CHECK(io.writes == 1);
        CHECK(strcmp(Hci::errorName(Hci::NCMD_STARVED), "ncmd_starved") == 0);
    }
    {   // 7. A reply to a command already given up on is counted as late and restores the credit
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        Hci::Reply r;
        CHECK(hci.run(0x0C03, nullptr, 0, &r, 100, idle10) == Hci::TIMEOUT);
        CHECK(hci.ncmd() == 0);
        io.deliver({0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00});
        hci.service();
        CHECK(hci.late() == 1); CHECK(hci.ncmd() == 1);
    }
    {   // 8. Read_Local_Version return parameters land in the reply after the status byte
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        io.onWrite = [](FakeIo &f, const std::vector<uint8_t> &) {
            f.deliver({0x04, 0x0E, 0x0C, 0x01, 0x01, 0x10, 0x00, 0x0B, 0xEF, 0xBE, 0x0B, 0x34, 0x12, 0xFE, 0xCA});
        };
        Hci::Reply r;
        CHECK(hci.run(0x1001, nullptr, 0, &r, 500, idle10) == Hci::OK);
        CHECK(r.len == 8);
        CHECK(r.params[0] == 0x0B);
        CHECK((r.params[1] | (r.params[2] << 8)) == 0xBEEF);
        CHECK((r.params[4] | (r.params[5] << 8)) == 0x1234);
    }
    {   // 9. Non-zero status is STATUS, with the status byte exposed
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        io.onWrite = [](FakeIo &f, const std::vector<uint8_t> &) { f.deliver({0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x01}); };
        Hci::Reply r;
        CHECK(hci.run(0x0C03, nullptr, 0, &r, 500, idle10) == Hci::STATUS);
        CHECK(r.status == 0x01);
    }
    {   // 10. Asynchronous events reach the callback; Command Complete/Status do not
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        struct Ev { int n = 0; uint8_t code = 0; uint8_t len = 0; uint8_t p0 = 0xEE;
                    static void fn(void *c, uint8_t code, const uint8_t *p, uint8_t len) { Ev *e = (Ev *)c; e->n++; e->code = code; e->len = len; e->p0 = len ? p[0] : 0xEE; } } ev;
        hci.onEvent(Ev::fn, &ev);
        io.deliver({0x04, 0x01, 0x01, 0x00});                // Inquiry Complete, status 0
        io.deliver({0x04, 0x0E, 0x03, 0x01, 0x00, 0x00});    // Command Complete for NOP: credit only
        hci.service();
        CHECK(ev.n == 1); CHECK(ev.code == 0x01); CHECK(ev.len == 1); CHECK(ev.p0 == 0);
        CHECK(hci.events() == 1); CHECK(hci.late() == 0);
    }
    {   // 11. ACL data reaches the ACL callback with the handle masked to 12 bits
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        struct A { uint16_t h = 0; uint16_t len = 0; uint8_t d0 = 0;
                   static void fn(void *c, uint16_t h, const uint8_t *d, uint16_t len) { A *a = (A *)c; a->h = h; a->len = len; a->d0 = d[0]; } } a;
        hci.onAcl(A::fn, &a);
        io.deliver({0x02, 0x01, 0x20, 0x02, 0x00, 0xAA, 0xBB});   // handle 0x0001 with PB flags 0x2
        hci.service();
        CHECK(a.h == 0x0001); CHECK(a.len == 2); CHECK(a.d0 == 0xAA);
    }
    {   // 12. run() refuses to overlap
        FakeIo io; g_io = &io; Hci hci(io); hci.begin();
        Hci::Reply r;
        CHECK(hci.submit(0x0C03, nullptr, 0, nullptr, nullptr) == Hci::OK);
        CHECK(hci.run(0x1001, nullptr, 0, &r, 100, idle10) == Hci::BUSY);
    }
    printf("hci_test: %d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
