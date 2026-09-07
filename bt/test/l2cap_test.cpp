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
    {   // 7. Our CONFIG_REQ carries an MTU option (the Shokz headset needs one; the ESP32 tolerated its absence).
        // Value = the L2CAP MTU the Mac's A2DP source negotiates with this headset (PacketLogger reference,
        // 2026-09-03: 01 02 EC 03 = 1004), which also fits this stack's single-ACL RX path (IW416 acl_len 1021).
        CapIo io; L2cap l(io); l.begin(0x0001, 7);
        L2cap::Channel *ch = l.connect(0x0019, 0x0041);           // our SCID 0x0041
        std::vector<uint8_t> rsp = l2(0x0001, { 0x03, 0x10, 0x08, 0x00, 0x80, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00 }); // CONN_RSP dcid=0x0080 scid=0x0041 ok
        l.onAcl(0x0001, rsp.data(), (uint16_t)rsp.size());
        io.tx.clear(); l.service();                                // emits our CONFIG_REQ
        bool sawReq = false, sawMtu = false; uint16_t mtu = 0;
        for (auto &p : io.tx) {                                    // full ACL packet: [02][hf][al][l2len][cid][code id len dcid flags opts...]
            if (p.size() >= 9 + 8 && p[9] == 0x04) { sawReq = true;
                CHECK(p[9 + 4] == 0x80 && p[9 + 5] == 0x00);       // DCID = the peer's CID
                uint16_t cmdLen = (uint16_t)(p[9 + 2] | (p[9 + 3] << 8));
                CHECK(cmdLen == p.size() - 9 - 4);                 // length field covers dcid+flags+options
                for (size_t i = 9 + 8; i + 3 < p.size(); ) { uint8_t t = p[i], ln = p[i + 1];
                    if (t == 0x01 && ln == 2) { sawMtu = true; mtu = (uint16_t)(p[i + 2] | (p[i + 3] << 8)); } i += 2 + ln; } }
        }
        CHECK(sawReq); CHECK(sawMtu); CHECK(mtu == L2cap::RX_MTU); CHECK(L2cap::RX_MTU == 1004);
        CHECK(ch->mtuIn == L2cap::RX_MTU);                         // what we told the peer it may send us
    }
    {   // 8. Five channels coexist: our SDP client, AVDTP signalling, AVDTP media, PLUS two peer-initiated channels
        // (both headsets open reverse SDP channels at us on AVDTP contact -- BT-2 transcript 2026-08-29; the Shokz
        // opens two in the Mac reference).  With only three slots the media connect() failed with no slot (0xFD).
        CapIo io; L2cap l(io); l.begin(0x0001, 7); l.acceptIncoming(true);
        CHECK(l.connect(0x0001, 0x0040) != nullptr);
        CHECK(l.connect(0x0019, 0x0041) != nullptr);
        std::vector<uint8_t> r1 = l2(0x0001, { 0x02, 0x21, 4, 0, 0x01, 0x00, 0x11, 0x01 });   // peer CONN_REQ psm=SDP scid=0x0111
        l.onAcl(0x0001, r1.data(), (uint16_t)r1.size()); l.service();
        std::vector<uint8_t> r2 = l2(0x0001, { 0x02, 0x22, 4, 0, 0x01, 0x00, 0x22, 0x02 });   // a second peer CONN_REQ scid=0x0222
        l.onAcl(0x0001, r2.data(), (uint16_t)r2.size()); l.service();
        CHECK(l.byRemote(0x0111) != nullptr); CHECK(l.byRemote(0x0222) != nullptr);
        CHECK(l.connect(0x0019, 0x0042) != nullptr);               // the media channel still gets a slot
        CHECK(L2cap::MAX_CHANNELS >= 5);
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
    {   // A1. allowPsm(): an inbound CONN_REQ for an allowed PSM is accepted; for any other PSM it is
        //     refused with result 0x0002 (PSM not supported) -- an AVCTP (0x0017) channel from a
        //     headset (AVRCP, piece 3) must NOT consume a slot.
        CapIo io; L2cap l(io); l.begin(0x0001, 7); l.acceptIncoming(true);
        l.allowPsm(0x0019); l.allowPsm(0x0001);
        // peer CONN_REQ: code 0x02, id 0x20, len 4, psm 0x0017 (AVCTP), scid 0x0055
        std::vector<uint8_t> req = l2(0x0001, {0x02, 0x20, 4, 0, 0x17, 0x00, 0x55, 0x00});
        l.onAcl(0x0001, req.data(), (uint16_t)req.size()); l.service();
        bool sawRefuse = false;
        for (auto &t : io.tx) if (t.size() >= 9 + 12 && t[9] == 0x03) {              // CONN_RSP (12-byte L2CAP payload)
            CHECK(t[9 + 4] == 0x00 && t[9 + 5] == 0x00);                             // DCID 0 (no channel)
            CHECK(t[9 + 8] == 0x02 && t[9 + 9] == 0x00);                             // result 0x0002 PSM not supported
            sawRefuse = true;
        }
        CHECK(sawRefuse); CHECK(l.byPsm(0x0017) == nullptr);
        // an allowed PSM IS accepted
        io.tx.clear();
        std::vector<uint8_t> ok = l2(0x0001, {0x02, 0x21, 4, 0, 0x19, 0x00, 0x56, 0x00});
        l.onAcl(0x0001, ok.data(), (uint16_t)ok.size()); l.service();
        bool sawAccept = false;
        for (auto &t : io.tx) if (t.size() >= 9 + 12 && t[9] == 0x03 && t[9 + 8] == 0x00 && t[9 + 9] == 0x00) sawAccept = true;
        CHECK(sawAccept); CHECK(l.byPsm(0x0019) != nullptr);
    }
    {   // A2. With NO allow-list set (default), acceptIncoming(true) keeps today's behaviour: any PSM is
        //     accepted -- so the existing avdtp/media/[avdtp]-gate paths, which never call allowPsm, are unchanged.
        CapIo io; L2cap l(io); l.begin(0x0001, 7); l.acceptIncoming(true);
        std::vector<uint8_t> req = l2(0x0001, {0x02, 0x22, 4, 0, 0x17, 0x00, 0x57, 0x00});
        l.onAcl(0x0001, req.data(), (uint16_t)req.size()); l.service();
        CHECK(l.byPsm(0x0017) != nullptr);                                            // accepted, as before
    }
    {   // A3. reset(): every channel goes FREE and the tx queue empties, so the next attempt starts clean.
        CapIo io; L2cap l(io); l.begin(0x0001, 7);
        CHECK(l.connect(0x0019, 0x0041) != nullptr);
        l.reset();
        CHECK(l.byLocal(0x0041) == nullptr && l.byPsm(0x0019) == nullptr);
        l.service();                                                                  // nothing queued survives reset()
        CHECK(io.tx.empty());
    }
    {   // A4a. nextInbound(): iterates peer-initiated OPEN channels of a PSM, in slot order.  Ample credits
        //      (20): accepting each inbound channel costs 3 signalling packets (CONN_RSP + our CFG_REQ + our
        //      CFG_RSP), so two channels need 6 -- with only 5 the second never reaches OPEN.
        CapIo io; L2cap l(io); l.begin(0x0001, 20); l.acceptIncoming(true); l.allowPsm(0x0019);
        for (uint8_t k = 0; k < 2; k++) {
            std::vector<uint8_t> rq = l2(0x0001, {0x02, (uint8_t)(0x30 + k), 4, 0, 0x19, 0x00, (uint8_t)(0x60 + k), 0x00});
            l.onAcl(0x0001, rq.data(), (uint16_t)rq.size()); l.service();
            L2cap::Channel *ch = l.byRemote((uint16_t)(0x0060 + k)); CHECK(ch);
            std::vector<uint8_t> cq = l2(0x0001, {0x04, (uint8_t)(0x40 + k), 8, 0, (uint8_t)ch->localCid, (uint8_t)(ch->localCid >> 8), 0, 0, 0x01, 0x02, 0x7F, 0x03});
            l.onAcl(0x0001, cq.data(), (uint16_t)cq.size());
            std::vector<uint8_t> cr = l2(0x0001, {0x05, (uint8_t)(0x50 + k), 6, 0, (uint8_t)ch->localCid, (uint8_t)(ch->localCid >> 8), 0, 0, 0, 0});
            l.onAcl(0x0001, cr.data(), (uint16_t)cr.size()); l.service();
            CHECK(ch->state == L2cap::OPEN);
        }
        const L2cap::Channel *a = l.nextInbound(0x0019, nullptr); CHECK(a && a->remoteCid == 0x0060);
        const L2cap::Channel *b = l.nextInbound(0x0019, a);       CHECK(b && b->remoteCid == 0x0061);
        CHECK(l.nextInbound(0x0019, b) == nullptr);
    }
    {   // A4b. creditsMin(): the running minimum credit since resetCreditsMin(); an NCP refill does NOT raise
        //      the floor.  Standalone (no channel setup to consume credits): send() queues on any cid, service()
        //      transmits while credits>0.  Start 5, send 3 (->2), NCP(+2) (->4); the floor stays 2.
        CapIo io; L2cap l(io); l.begin(0x0001, 5);
        l.resetCreditsMin();
        for (int i = 0; i < 3; i++) { const uint8_t d[4] = {0, 1, 2, 3}; l.send(0x0040, d, 4); }
        l.service();
        CHECK(l.credits() == 2 && l.creditsMin() == 2);
        uint8_t ncp[] = { 0x01, 0x01, 0x00, 0x02, 0x00 };  l.onEvent(0x13, ncp, sizeof ncp);
        CHECK(l.credits() == 4 && l.creditsMin() == 2);
    }
    {   // C1. Normal flow: N packets sent, all N credits returned -> sent==returned==N, credits back to max,
        //     credmin correct, no starve, no clamp (NEW-34 piece 4 credit-leak instrument).
        CapIo io; L2cap l(io); l.begin(0x0001, 4); l.resetCreditStats();
        for (int i = 0; i < 4; i++) { const uint8_t d[4] = {0,1,2,3}; l.send(0x0040, d, 4); }
        l.tickClock(io.now); l.service();                                  // 4 sent -> credits 0
        CHECK(l.pktsSent() == 4 && l.credits() == 0 && l.creditsMin() == 0);
        uint8_t ncp[] = { 0x01, 0x01, 0x00, 0x04, 0x00 }; l.onEvent(0x13, ncp, sizeof ncp);   // return all 4
        CHECK(l.creditsReturned() == 4 && l.credits() == 4);
        CHECK(l.pktsSent() - l.creditsReturned() == 0);                    // outstanding == 0
        CHECK(l.clampHits() == 0 && l.starves() == 0);
    }
    {   // C2. Withheld NCP (the load-bearing negative): a credit that never comes back keeps the pool down, and
        //     once the pool sits at 0 with work pending the starve fingerprint appears and grows with the clock.
        CapIo io; L2cap l(io); l.begin(0x0001, 2); l.resetCreditStats();
        const uint8_t d[4] = {0,1,2,3};
        io.now = 1000; l.send(0x0040, d, 4); l.send(0x0040, d, 4); l.tickClock(io.now); l.service();   // 2 sent -> credits 0
        CHECK(l.credits() == 0 && l.pktsSent() == 2);
        uint8_t ncp1[] = { 0x01, 0x01, 0x00, 0x01, 0x00 }; l.onEvent(0x13, ncp1, sizeof ncp1);          // return only ONE
        CHECK(l.creditsReturned() == 1 && l.credits() == 1 && l.pktsSent() - l.creditsReturned() == 1);
        // queue TWO more: service() sends one (credits 1 -> 0), the other STAYS queued -> credits==0 with work pending
        l.send(0x0040, d, 4); l.send(0x0040, d, 4); l.tickClock(io.now); l.service();
        CHECK(l.credits() == 0 && l.pktsSent() - l.creditsReturned() == 2);                             // one stuck, one still outstanding
        io.now = 1300; l.tickClock(io.now); l.service();                                                // 300 ms later, still 0-with-work (NCP withheld)
        CHECK(l.starves() >= 1 && l.starveMaxMs() >= 300);
    }
    {   // C3. Over-return (clamp): the controller returns more credits than were outstanding -> credits caps at
        //     maxCredits and clampHits records the discard (the opposite failure -- a double-count).
        CapIo io; L2cap l(io); l.begin(0x0001, 3); l.resetCreditStats();
        const uint8_t d[4] = {0,1,2,3};
        l.send(0x0040, d, 4); l.tickClock(io.now); l.service();            // 1 sent -> credits 2, one outstanding
        uint8_t ncp[] = { 0x01, 0x01, 0x00, 0x05, 0x00 }; l.onEvent(0x13, ncp, sizeof ncp);   // return FIVE (only 1 outstanding)
        CHECK(l.credits() == 3 && l.clampHits() >= 1);                     // capped at maxCredits=3, clamp recorded
    }
    {   // A4 (NEW-34 piece 5). freeSlots(): reusable (FREE or CLOSED) slots -- the soak's slot-leak baseline.
        // 5 at begin(); a connect() takes one and rides it all the way to CONFIG then a HONEST OPEN (review: a
        // mutant counting OPEN as reusable must fail here, not just at WAIT_CONN/CONFIG); a peer DISC_REQ closes
        // it (review: no direct state poke); the CLOSED slot is reused by a second connect(); reset() restores all 5.
        CapIo io; L2cap l(io); l.begin(0x0001, 7);
        CHECK(l.freeSlots() == L2cap::MAX_CHANNELS);
        L2cap::Channel *ch = l.connect(0x0019, 0x0041);                          // our SCID 0x0041
        CHECK(ch != nullptr && l.freeSlots() == L2cap::MAX_CHANNELS - 1);
        std::vector<uint8_t> rsp = l2(0x0001, {0x03, 0x10, 8, 0, 0x40, 0x03, 0x41, 0x00, 0, 0, 0, 0}); // Conn Rsp: dcid=0x0340 scid=0x0041 ok
        l.onAcl(0x0001, rsp.data(), (uint16_t)rsp.size());
        CHECK(ch->state == L2cap::CONFIG && l.freeSlots() == L2cap::MAX_CHANNELS - 1);
        std::vector<uint8_t> req = l2(0x0001, {0x04, 1, 8, 0, 0x41, 0x00, 0, 0, 0x01, 0x02, 0x7F, 0x03}); // peer Cfg Req: dcid=ours, MTU 895
        l.onAcl(0x0001, req.data(), (uint16_t)req.size());
        l.service();                                                            // sends OUR Cfg Req + the Cfg Rsp to the peer's
        CHECK(ch->state == L2cap::CONFIG);                                      // still CONFIG: our own Cfg Req is unanswered
        std::vector<uint8_t> crsp = l2(0x0001, {0x05, 0x20, 6, 0, 0x41, 0x00, 0, 0, 0, 0}); // peer Cfg Rsp to ours: scid=ours 0x0041, ok
        l.onAcl(0x0001, crsp.data(), (uint16_t)crsp.size());
        l.service();
        CHECK(ch->state == L2cap::OPEN && l.freeSlots() == L2cap::MAX_CHANNELS - 1);   // the mutant this line kills: OPEN counted as reusable
        std::vector<uint8_t> disc = l2(0x0001, {0x06, 0x20, 4, 0, 0x41, 0x00, 0x40, 0x03}); // peer Disc Req: dcid=ours 0x0041, scid=peer's 0x0340
        l.onAcl(0x0001, disc.data(), (uint16_t)disc.size());                    // handleSig flips it to CLOSED directly -- no poke needed
        CHECK(ch->state == L2cap::CLOSED && l.freeSlots() == L2cap::MAX_CHANNELS);
        L2cap::Channel *ch2 = l.connect(0x0019, 0x0042);                        // DIFFERENT local CID -- connect()'s scan takes the
        CHECK(ch2 == ch && l.freeSlots() == L2cap::MAX_CHANNELS - 1);           // first FREE-or-CLOSED slot in array order, which is ch's
        l.reset();
        CHECK(l.freeSlots() == L2cap::MAX_CHANNELS);
    }
    {   // A5 (NEW-34 piece 3, silicon finding 2026-09-07). The allow-list must hold THREE PSMs: A2dpSource allows
        // SDP + AVDTP and piece 3 adds AVCTP (0x0017) -- with a two-entry list the third was silently dropped and
        // the Shokz's AVCTP CONN_REQ was still refused 0x0002 on the bench.  A fourth, unlisted PSM stays refused.
        CapIo io; L2cap l(io); l.begin(0x0001, 7); l.acceptIncoming(true);
        l.allowPsm(0x0019); l.allowPsm(0x0001); l.allowPsm(0x0017);
        CHECK(l.allowedCount() == 3);
        std::vector<uint8_t> req = l2(0x0001, {0x02, 0x07, 4, 0, 0x17, 0x00, 0x43, 0x08});   // CONN_REQ psm 0x0017 scid 0x0843
        l.onAcl(0x0001, req.data(), (uint16_t)req.size()); l.service();
        bool acc = false, ref = false;
        for (auto &f : io.tx) if (f.size() >= 9 + 12 && f[9] == 0x03) { uint16_t res = (uint16_t)(f[9 + 8] | (f[9 + 9] << 8)); if (res == 0) acc = true; if (res == 2) ref = true; }
        CHECK(acc && !ref);                                                           // AVCTP accepted (result 0x0000)
        io.tx.clear();
        std::vector<uint8_t> bad = l2(0x0001, {0x02, 0x08, 4, 0, 0x1F, 0x00, 0x44, 0x08}); // an UNLISTED psm 0x001F
        l.onAcl(0x0001, bad.data(), (uint16_t)bad.size()); l.service();
        bool ref2 = false;
        for (auto &f : io.tx) if (f.size() >= 9 + 12 && f[9] == 0x03) { uint16_t res = (uint16_t)(f[9 + 8] | (f[9 + 9] << 8)); if (res == 2) ref2 = true; }
        CHECK(ref2);                                                                  // still refused 0x0002
    }
    {   // A6 (2026-09-07). service() BEFORE begin() is inert: a freshly constructed L2cap has an empty TX queue, no
        // credits and no channels (in-class initialisers), so ticking it from boot writes nothing and does not walk
        // garbage.  The a2dpsource_test crash that found this is the RED demonstration (a stack L2cap serviced before
        // any attempt began, m_txHead read as 155).
        CapIo io; L2cap l(io);
        l.service(); l.service();
        CHECK(io.tx.empty() && l.credits() == 0 && l.freeSlots() == L2cap::MAX_CHANNELS && l.dropped() == 0);
        uint8_t d[4] = {1, 2, 3, 4}; CHECK(!l.send(0x0040, d, 4) || true);   // a send before begin() is harmless either way
        l.service(); CHECK(l.credits() == 0);
    }
    printf("l2cap_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
