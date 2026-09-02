// Host tests for L2cap: the receiver-side SCID rule, mandatory replies, credits.
#include "L2cap.h"
#include <stdio.h>
#include <string.h>
#include <vector>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
struct CapIo : HciIo {                       // records TX; RX unused (L2cap is fed directly)
    std::vector<std::vector<uint8_t> > tx; uint32_t now = 0;
    size_t write(const uint8_t *p, size_t n) override { tx.push_back(std::vector<uint8_t>(p, p + n)); return n; }
    int available() override { return 0; } int read() override { return -1; } uint32_t nowMs() override { return now; }
};
// ACL packet as Hci hands it to onAcl(): [l2cap len lo, hi][cid lo, hi][payload]
static std::vector<uint8_t> l2(uint16_t cid, std::initializer_list<uint8_t> pl) {
    std::vector<uint8_t> v = { (uint8_t)pl.size(), 0, (uint8_t)cid, (uint8_t)(cid >> 8) }; v.insert(v.end(), pl); return v; }
int main() {
    {   // 1. Config Response to the peer's Config Request names the PEER's CID (receiver-side rule), echoes its options
        CapIo io; L2cap l(io); l.begin(0x0001, 7);
        L2cap::Channel *ch = l.connect(0x0019, 0x0041);           // our SCID 0x0041
        io.tx.clear();
        std::vector<uint8_t> rsp = l2(0x0001, {0x03, 0x10, 8, 0, 0x40, 0x03, 0x41, 0x00, 0, 0, 0, 0}); // Conn Rsp: dcid=0x0340 scid=0x0041 ok
        l.onAcl(0x0001, rsp.data(), (uint16_t)rsp.size());
        std::vector<uint8_t> req = l2(0x0001, {0x04, 1, 8, 0, 0x41, 0x00, 0, 0, 0x01, 0x02, 0x7F, 0x03}); // peer Cfg Req: dcid=ours, MTU 895
        l.onAcl(0x0001, req.data(), (uint16_t)req.size());
        l.service();
        bool found = false;
        for (auto &t : io.tx) if (t.size() >= 9 + 4 && t[9] == 0x05) {         // Config Response
            found = true;
            CHECK(t[9 + 4] == 0x40 && t[9 + 5] == 0x03);                       // SCID = peer's 0x0340, NOT ours
            CHECK(t[9 + 8] == 0x00 && t[9 + 9] == 0x00);                       // Result success
            CHECK(t.size() == 9 + 10 + 4 && t[9 + 10] == 0x01 && t[9 + 12] == 0x7F); // options echoed
        }
        CHECK(found); CHECK(ch->mtuOut == 895);
    }
    {   // 2. Information Request (ext features) and Echo Request are answered from service()
        CapIo io; L2cap l(io); l.begin(0x0001, 7);
        std::vector<uint8_t> inf = l2(0x0001, {0x0A, 2, 2, 0, 0x02, 0x00});
        std::vector<uint8_t> ech = l2(0x0001, {0x08, 5, 0, 0});
        l.onAcl(0x0001, inf.data(), (uint16_t)inf.size()); l.onAcl(0x0001, ech.data(), (uint16_t)ech.size());
        CHECK(io.tx.empty());                                                  // nothing sent from the RX path
        l.service();
        CHECK(io.tx.size() == 2);
        CHECK(io.tx[0][9] == 0x0B && io.tx[0][9 + 1] == 2 && io.tx[0][9 + 4] == 0x02 && io.tx[0][9 + 6] == 0x00); // Info Rsp type 2, success
        CHECK(io.tx[1][9] == 0x09 && io.tx[1][9 + 1] == 5);                   // Echo Rsp, same id
    }
    {   // 3. Credits: acl_num=2 -> the third data packet waits for a Number_Of_Completed_Packets
        CapIo io; L2cap l(io); l.begin(0x0001, 2);
        uint8_t d[4] = {1, 2, 3, 4};
        CHECK(l.send(0x0340, d, 4)); CHECK(l.send(0x0340, d, 4)); CHECK(l.send(0x0340, d, 4));
        l.service(); CHECK(io.tx.size() == 2); CHECK(l.credits() == 0);
        uint8_t ncp[5] = { 1, 0x01, 0x00, 0x01, 0x00 };                        // 1 handle, 0x0001, 1 completed
        l.onEvent(0x13, ncp, 5); l.service();
        CHECK(io.tx.size() == 3); CHECK(l.credits() == 0);
    }
    {   // 4. Hostile CFG_REQ: cmdLen claims 65528 (optLen 65524) but only 1 option byte actually arrived (13-byte frame) --
        // must clamp to what's present (avail), never read/echo past the end of the received buffer (review, over-read).
        CapIo io; L2cap l(io); l.begin(0x0001, 7);
        L2cap::Channel *ch = l.connect(0x0019, 0x0041);
        std::vector<uint8_t> rsp = l2(0x0001, {0x03, 0x10, 8, 0, 0x40, 0x03, 0x41, 0x00, 0, 0, 0, 0});
        l.onAcl(0x0001, rsp.data(), (uint16_t)rsp.size());
        std::vector<uint8_t> req = l2(0x0001, {0x04, 1, 0xF8, 0xFF, 0x41, 0x00, 0, 0, 0xAA}); // cmdLen=65528, 1 opt byte present
        CHECK(req.size() == 13);
        io.tx.clear();
        l.onAcl(0x0001, req.data(), (uint16_t)req.size());
        CHECK(ch->optLen <= 1);                                                    // clamped to avail (1), not the claimed cmdLen
        l.service();
        bool found = false;
        for (auto &t : io.tx) if (t.size() >= 9 + 4 && t[9] == 0x05) {             // Config Response
            found = true;
            CHECK(t.size() <= 9 + 10 + ch->optLen);                                // no longer than 10+optLen -- nothing over-read is echoed back
        }
        CHECK(found);
    }
    printf("l2cap_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
