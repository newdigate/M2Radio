// Host tests for Avrcp: the Shokz's RegisterNotification, verbatim from the 2026-09-07 bench capture, must be
// answered INTERIM PLAYING on the same transaction label; anything else NOT IMPLEMENTED; wrong PID -> IPID.
#include "Avrcp.h"
#include <stdio.h>
#include <string.h>
#include <vector>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
struct CapIo : HciIo {
    std::vector<std::vector<uint8_t> > tx; uint32_t now = 0;
    size_t write(const uint8_t *p, size_t n) override { tx.push_back(std::vector<uint8_t>(p, p + n)); return n; }
    int available() override { return 0; } int read() override { return -1; } uint32_t nowMs() override { return now; }
};
static bool eq(const uint8_t *a, uint16_t n, std::initializer_list<uint8_t> b) { return n == b.size() && memcmp(a, b.begin(), n) == 0; }
int main() {
    // The captured command: AVCTP 20 11 0E | AV/C 03 48 00 | 00 19 58 | 31 00 | 00 05 | 01 00 00 00 00
    const uint8_t shokz[] = { 0x20, 0x11, 0x0E, 0x03, 0x48, 0x00, 0x00, 0x19, 0x58, 0x31, 0x00, 0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00 };
    {   // 1. INTERIM PLAYING, transaction label 2 echoed, response bit set, PID echoed.
        // kind is seeded to POISON, not 0: 0 is KIND_NOT_IMPLEMENTED, so a respond() that never WRITES kind
        // would be indistinguishable from one that classified correctly.  Every later call must CHANGE it.
        uint8_t out[64]; uint8_t kind = 0xEE; uint16_t n = Avrcp::respond(shokz, sizeof shokz, out, sizeof out, &kind);
        CHECK(kind == Avrcp::KIND_NOTIFICATION);
        CHECK(eq(out, n, { 0x22, 0x11, 0x0E, 0x0F, 0x48, 0x00, 0x00, 0x19, 0x58, 0x31, 0x00, 0x00, 0x02, 0x01, 0x01 }));
    }
    {   // 2. A different transaction label is echoed (label 7).
        uint8_t c[sizeof shokz]; memcpy(c, shokz, sizeof c); c[0] = 0x70;
        uint8_t out[64]; uint16_t n = Avrcp::respond(c, sizeof c, out, sizeof out, nullptr);
        CHECK(n == 15 && out[0] == 0x72 && out[3] == 0x0F && out[14] == 0x01);
    }
    {   // 3. GetCapabilities(EVENTS_SUPPORTED), verbatim from arm 4 of the bench capture (the Shokz sends it BEFORE
        // registering) -> STABLE with the TWO events this target raises: PLAYBACK_STATUS_CHANGED and, since absolute
        // volume, VOLUME_CHANGED (0x0D).  COMPANY_ID -> STABLE with the SIG id.
        const uint8_t gc[] = { 0x10, 0x11, 0x0E, 0x01, 0x48, 0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x01, 0x03 };
        uint8_t out[64]; uint8_t kind = 0xEE; uint16_t n = Avrcp::respond(gc, sizeof gc, out, sizeof out, &kind);
        CHECK(kind == Avrcp::KIND_ANSWERED);
        CHECK(eq(out, n, { 0x12, 0x11, 0x0E, 0x0C, 0x48, 0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01, 0x0D }));
        const uint8_t co[] = { 0x30, 0x11, 0x0E, 0x01, 0x48, 0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x01, 0x02 };
        n = Avrcp::respond(co, sizeof co, out, sizeof out, nullptr);
        CHECK(eq(out, n, { 0x32, 0x11, 0x0E, 0x0C, 0x48, 0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x05, 0x02, 0x01, 0x00, 0x19, 0x58 }));
    }
    {   // 3b. A PDU this target has never seen (GetPlayStatus 0x30) -> NOT IMPLEMENTED, operands echoed.
        const uint8_t gp[] = { 0x40, 0x11, 0x0E, 0x01, 0x48, 0x00, 0x00, 0x19, 0x58, 0x30, 0x00, 0x00, 0x00 };
        uint8_t out[64]; uint8_t kind = 0xEE; uint16_t n = Avrcp::respond(gp, sizeof gp, out, sizeof out, &kind);
        CHECK(kind == Avrcp::KIND_NOT_IMPLEMENTED);
        CHECK(eq(out, n, { 0x42, 0x11, 0x0E, 0x08, 0x48, 0x00, 0x00, 0x19, 0x58, 0x30, 0x00, 0x00, 0x00 }));
    }
    {   // 4. A response frame is ignored; a foreign PID gets the 3-byte IPID reply; a fragment is ignored.
        uint8_t r[sizeof shokz]; memcpy(r, shokz, sizeof r); r[0] = 0x22; uint8_t out[64];
        CHECK(Avrcp::respond(r, sizeof r, out, sizeof out, nullptr) == 0);
        uint8_t f[sizeof shokz]; memcpy(f, shokz, sizeof f); f[1] = 0x11; f[2] = 0x0F; uint8_t kind = 0xEE;
        CHECK(eq(out, Avrcp::respond(f, sizeof f, out, sizeof out, &kind), { 0x23, 0x11, 0x0F }));
        // IPID says "we do not carry that profile", so it belongs in unsupported(), not answered() (NEW-42).
        CHECK(kind == Avrcp::KIND_NOT_IMPLEMENTED);
        uint8_t g[sizeof shokz]; memcpy(g, shokz, sizeof g); g[0] = 0x24;                    // packet type 01 = start fragment
        CHECK(Avrcp::respond(g, sizeof g, out, sizeof out, nullptr) == 0);
    }
    {   // 5. End to end on an L2cap: the command lands on a 0x0017 channel, the INTERIM goes out from service() on the
        // peer's CID, the counter moves; a second command before service() replaces the first (dropped counted).
        CapIo io; L2cap l(io); l.begin(0x0001, 7); l.acceptIncoming(true); l.allowPsm(0x0019); l.allowPsm(0x0001); l.allowPsm(0x0017);
        std::vector<uint8_t> req = { 8, 0, 1, 0, 0x02, 0x07, 4, 0, 0x17, 0x00, 0x01, 0x0C };          // peer CONN_REQ psm 0x0017 scid 0x0C01
        l.onAcl(0x0001, req.data(), (uint16_t)req.size()); l.service();
        L2cap::Channel *ch = l.byRemote(0x0C01); CHECK(ch != nullptr && ch->psm == 0x0017);
        Avrcp a; io.tx.clear();
        CHECK(a.onData(*ch, shokz, sizeof shokz)); CHECK(a.pending()); CHECK(io.tx.empty());   // recorded, not sent from RX
        a.service(l); l.service();
        bool sent = false; for (auto &f : io.tx) if (f.size() == 9 + 15 && f[7] == 0x01 && f[8] == 0x0C && f[9 + 3] == 0x0F && f[9 + 14] == 0x01) sent = true;
        CHECK(sent && a.notifications() == 1 && !a.pending());
        CHECK(a.onData(*ch, shokz, sizeof shokz) && a.onData(*ch, shokz, sizeof shokz) && a.dropped() == 1);
        L2cap::Channel other = *ch; other.psm = 0x0019; CHECK(!a.onData(other, shokz, sizeof shokz));
    }
    {   // V1. SetAbsoluteVolume (PDU 0x50, CONTROL ctype 0x00, 1 param byte 0..127): ACCEPTED (0x09) echoing the volume
        //     we applied, and the registered callback sees it.  Bit 7 of the parameter is reserved and masked.
        static uint8_t seen = 0xFF; Avrcp::setVolumeCallback([](void *, uint8_t v) { seen = v; }, nullptr);
        std::vector<uint8_t> cmd = { 0x00, 0x11, 0x0E, 0x00, 0x48, 0x00, 0x00, 0x19, 0x58, 0x50, 0x00, 0x00, 0x01, 0xE5 };   // volume 0x65 | reserved bit
        uint8_t out[64]; uint8_t kind = 0xEE; uint16_t n = Avrcp::respond(cmd.data(), (uint16_t)cmd.size(), out, sizeof out, &kind);
        CHECK(n == 14 && out[0] == 0x02 && out[3] == 0x09 && out[9] == 0x50 && out[12] == 0x01 && out[13] == 0x65 && seen == 0x65
              && kind == Avrcp::KIND_ANSWERED);
        Avrcp::setVolumeCallback(nullptr, nullptr);
    }
    {   // V2. GetCapabilities(EVENTS_SUPPORTED) now lists BOTH events (PLAYBACK_STATUS_CHANGED 0x01, VOLUME_CHANGED 0x0D).
        std::vector<uint8_t> caps = { 0x10, 0x11, 0x0E, 0x01, 0x48, 0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x01, 0x03 };
        uint8_t out[64]; uint8_t kind = 0xEE; uint16_t n = Avrcp::respond(caps.data(), (uint16_t)caps.size(), out, sizeof out, &kind);
        CHECK(n == 17 && out[11] == 0x00 && out[12] == 0x04 && out[13] == 0x03 && out[14] == 0x02 && out[15] == 0x01 && out[16] == 0x0D
              && kind == Avrcp::KIND_ANSWERED);
    }
    {   // V3. RegisterNotification(VOLUME_CHANGED) is answered INTERIM with the current volume; a later local volume
        //     change (setLocalVolume) produces ONE CHANGED response on the registered label; the registration is one-shot.
        Avrcp a; a.setLocalVolume(80);
        std::vector<uint8_t> reg = { 0x20, 0x11, 0x0E, 0x03, 0x48, 0x00, 0x00, 0x19, 0x58, 0x31, 0x00, 0x00, 0x05, 0x0D, 0, 0, 0, 0 };
        L2cap::Channel ch{}; ch.psm = Avrcp::PSM; ch.remoteCid = 0x0044; ch.state = L2cap::OPEN;
        CHECK(a.onData(ch, reg.data(), (uint16_t)reg.size()));
        CapIo io; L2cap l(io); l.begin(0x0001, 10); a.service(l); l.service();
        CHECK(io.tx.size() == 1 && io.tx[0][9 + 0] == 0x22 && io.tx[0][9 + 3] == 0x0F && io.tx[0][9 + 9] == 0x31 && io.tx[0][9 + 13] == 0x0D && io.tx[0][9 + 14] == 80);
        io.tx.clear(); a.setLocalVolume(100); a.service(l); l.service();
        CHECK(io.tx.size() == 1 && io.tx[0][9 + 0] == 0x22 && io.tx[0][9 + 3] == 0x0D && io.tx[0][9 + 13] == 0x0D && io.tx[0][9 + 14] == 100);   // CHANGED, label 2
        io.tx.clear(); a.setLocalVolume(90); a.service(l); l.service(); CHECK(io.tx.empty());                                            // one-shot
        CHECK(a.notifications() == 1);
        // That path never reaches respond(): service() answers it to carry live state, on its own counter line.
        // It is a NOTIFICATION and must move NEITHER other counter -- nor may the one-shot CHANGED, which answers
        // no command at all, be counted as one (NEW-42: the three counters partition what the target replied to).
        CHECK(a.answered() == 0 && a.unsupported() == 0);
    }
    {   // V4. A SetAbsoluteVolume via the object path (onData/service) reaches the callback AND leaves local volume updated,
        //     so a later VOLUME_CHANGED INTERIM reports what the phone set.
        static uint8_t seen2 = 0; Avrcp::setVolumeCallback([](void *, uint8_t v) { seen2 = v; }, nullptr);
        Avrcp a; std::vector<uint8_t> cmd = { 0x30, 0x11, 0x0E, 0x00, 0x48, 0x00, 0x00, 0x19, 0x58, 0x50, 0x00, 0x00, 0x01, 0x40 };
        L2cap::Channel ch{}; ch.psm = Avrcp::PSM; ch.remoteCid = 0x0044; ch.state = L2cap::OPEN;
        CapIo io; L2cap l(io); l.begin(0x0001, 10);
        CHECK(a.onData(ch, cmd.data(), (uint16_t)cmd.size())); a.service(l); l.service();
        CHECK(io.tx.size() == 1 && io.tx[0][9 + 3] == 0x09 && seen2 == 0x40 && a.volume() == 0x40);
        Avrcp::setVolumeCallback(nullptr, nullptr);
    }
    {   // K1 (NEW-42). WHAT the target answered is a three-way fact, not a bool: a NOTIFICATION, a properly ANSWERED
        //     command, or NOT IMPLEMENTED.  GetCapabilities and SetAbsoluteVolume are answered properly and must land in
        //     answered(), never unsupported() -- on the iPhone bench (2026-09-09) unsupported() climbed with every
        //     volume press and the heartbeat read as the phone sending commands we do not implement.
        //     RED against 9f24315: unsupported() == 2 and answered() does not exist.
        static uint8_t seenK = 0xFF; Avrcp::setVolumeCallback([](void *, uint8_t v) { seenK = v; }, nullptr);
        Avrcp a; CapIo io; L2cap l(io); l.begin(0x0001, 10);
        L2cap::Channel ch{}; ch.psm = Avrcp::PSM; ch.remoteCid = 0x0044; ch.state = L2cap::OPEN;
        std::vector<uint8_t> caps = { 0x10, 0x11, 0x0E, 0x01, 0x48, 0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x01, 0x03 };
        CHECK(a.onData(ch, caps.data(), (uint16_t)caps.size())); a.service(l); l.service();
        CHECK(a.answered() == 1); CHECK(a.unsupported() == 0); CHECK(a.notifications() == 0);
        std::vector<uint8_t> vol = { 0x30, 0x11, 0x0E, 0x00, 0x48, 0x00, 0x00, 0x19, 0x58, 0x50, 0x00, 0x00, 0x01, 0x40 };
        CHECK(a.onData(ch, vol.data(), (uint16_t)vol.size())); a.service(l); l.service();
        CHECK(a.answered() == 2); CHECK(a.unsupported() == 0); CHECK(seenK == 0x40);
        std::vector<uint8_t> gp = { 0x40, 0x11, 0x0E, 0x01, 0x48, 0x00, 0x00, 0x19, 0x58, 0x30, 0x00, 0x00, 0x00 };   // GetPlayStatus: never built
        CHECK(a.onData(ch, gp.data(), (uint16_t)gp.size())); a.service(l); l.service();
        CHECK(a.answered() == 2); CHECK(a.unsupported() == 1);
        // ... and the pure function reports the same three-way kind.
        uint8_t out[64], kind = 0xEE;
        CHECK(Avrcp::respond(caps.data(), (uint16_t)caps.size(), out, sizeof out, &kind) > 0 && kind == Avrcp::KIND_ANSWERED);
        CHECK(Avrcp::respond(gp.data(), (uint16_t)gp.size(), out, sizeof out, &kind) > 0 && kind == Avrcp::KIND_NOT_IMPLEMENTED);
        CHECK(Avrcp::respond(shokz, sizeof shokz, out, sizeof out, &kind) > 0 && kind == Avrcp::KIND_NOTIFICATION);
        Avrcp::setVolumeCallback(nullptr, nullptr);
    }
    printf("avrcp_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
