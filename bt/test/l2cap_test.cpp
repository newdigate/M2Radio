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
    {   // 5. A retained CONN_RSP retry must stay faithful to the request it was computed for -- not a later one that
        // overwrote the live pending fields while the retry was still stuck behind a full txq (review, cross-request mix-up).
        CapIo io; L2cap l(io); l.begin(0x0001, 4);
        l.acceptIncoming(true);
        uint8_t filler[4] = { 9, 9, 9, 9 };
        int filled = 0;
        while (l.send(0x0340, filler, 4)) filled++;                            // saturate the txq
        CHECK(filled > 0);
        std::vector<uint8_t> req1 = l2(0x0001, {0x02, 0x21, 4, 0, 0x19, 0x00, 0x11, 0x01}); // CONN_REQ #1: id=0x21 psm=0x0019 scid=0x0111
        l.onAcl(0x0001, req1.data(), (uint16_t)req1.size());
        l.service();                                                           // allocates + snapshots #1's identity; txq still full -> send fails, stays pending
        std::vector<uint8_t> req2 = l2(0x0001, {0x02, 0x22, 4, 0, 0x19, 0x00, 0x22, 0x02}); // DISTINCT CONN_REQ #2: id=0x22 scid=0x0222
        l.onAcl(0x0001, req2.data(), (uint16_t)req2.size());                   // overwrites the LIVE m_p.connId/connScid; must not reach the snapshot
        bool found = false; uint8_t connId = 0; uint16_t connScid = 0;
        for (int i = 0; i < 20 && !found; i++) {                               // drip credits back in and retry until the retained CONN_RSP drains out
            uint8_t ncp[5] = { 1, 0x01, 0x00, 4, 0 };                          // restore credits (capped at maxCredits) so the queue can empty
            l.onEvent(0x13, ncp, 5);
            l.service();
            for (auto &t : io.tx) if (t.size() >= 9 + 12 && t[9] == 0x03) {    // CONN_RSP
                found = true; connId = t[9 + 1]; connScid = (uint16_t)(t[9 + 6] | (t[9 + 7] << 8));
            }
        }
        CHECK(found);
        CHECK(connId == 0x21);                                                 // #1's id, never #2's 0x22
        CHECK(connScid == 0x0111);                                             // #1's scid, never #2's 0x0222
    }
    {   // N. onAclTrace fires for inbound (a fed ACL) and outbound (a queued send), with the L2CAP PDU + handle.
        struct Cap { struct Rec { bool out; uint16_t handle; std::vector<uint8_t> pdu; }; std::vector<Rec> recs; };
        static Cap cap;   // static so the C-style callback can reach it
        cap.recs.clear();
        CapIo io; L2cap l(io); l.begin(0x0001, 7);
        l.onAclTrace([](void *, bool out, uint16_t h, const uint8_t *p, uint16_t n) {
            cap.recs.push_back({ out, h, std::vector<uint8_t>(p, p + n) }); }, nullptr);
        // Inbound: feed an L2CAP Info Request on the signalling CID 0x0001.
        std::vector<uint8_t> in = l2(0x0001, { 0x0A, 0x01, 0x02, 0x00, 0x02, 0x00 });  // INFO_REQ
        l.onAcl(0x0001, in.data(), (uint16_t)in.size());
        // Outbound: connect() queues a Connection Request; service() writes it.
        l.connect(0x0019, 0x0041); l.service();
        bool sawIn = false, sawOut = false;
        for (auto &r : cap.recs) {
            if (!r.out) { sawIn = true; CHECK(r.handle == 0x0001);
                          CHECK(r.pdu.size() >= 4 && r.pdu[2] == 0x01 && r.pdu[3] == 0x00); }   // CID 0x0001
            else        { sawOut = true; CHECK(r.handle == 0x0001);
                          CHECK(r.pdu.size() >= 4 && r.pdu[0] == (uint8_t)(r.pdu.size() - 4)); } // L2CAP len field
        }
        CHECK(sawIn); CHECK(sawOut);
    }
    printf("l2cap_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
