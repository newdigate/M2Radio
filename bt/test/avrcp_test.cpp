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
        uint8_t out[64]; bool notif = false; uint16_t n = Avrcp::respond(shokz, sizeof shokz, out, sizeof out, &notif);
        CHECK(notif);
        CHECK(eq(out, n, { 0x22, 0x11, 0x0E, 0x0F, 0x48, 0x00, 0x00, 0x19, 0x58, 0x31, 0x00, 0x00, 0x02, 0x01, 0x01 }));
    }
    {   // 2. A different transaction label is echoed (label 7).
        uint8_t c[sizeof shokz]; memcpy(c, shokz, sizeof c); c[0] = 0x70;
        uint8_t out[64]; uint16_t n = Avrcp::respond(c, sizeof c, out, sizeof out, nullptr);
        CHECK(n == 15 && out[0] == 0x72 && out[3] == 0x0F && out[14] == 0x01);
    }
    {   // 3. GetCapabilities(EVENTS_SUPPORTED), verbatim from arm 4 of the bench capture (the Shokz sends it BEFORE
        // registering) -> STABLE, one event: PLAYBACK_STATUS_CHANGED.  COMPANY_ID -> STABLE with the SIG id.
        const uint8_t gc[] = { 0x10, 0x11, 0x0E, 0x01, 0x48, 0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x01, 0x03 };
        uint8_t out[64]; bool notif = true; uint16_t n = Avrcp::respond(gc, sizeof gc, out, sizeof out, &notif);
        CHECK(!notif);
        CHECK(eq(out, n, { 0x12, 0x11, 0x0E, 0x0C, 0x48, 0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x03, 0x03, 0x01, 0x01 }));
        const uint8_t co[] = { 0x30, 0x11, 0x0E, 0x01, 0x48, 0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x01, 0x02 };
        n = Avrcp::respond(co, sizeof co, out, sizeof out, nullptr);
        CHECK(eq(out, n, { 0x32, 0x11, 0x0E, 0x0C, 0x48, 0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x05, 0x02, 0x01, 0x00, 0x19, 0x58 }));
    }
    {   // 3b. A PDU this target has never seen (GetPlayStatus 0x30) -> NOT IMPLEMENTED, operands echoed.
        const uint8_t gp[] = { 0x40, 0x11, 0x0E, 0x01, 0x48, 0x00, 0x00, 0x19, 0x58, 0x30, 0x00, 0x00, 0x00 };
        uint8_t out[64]; bool notif = true; uint16_t n = Avrcp::respond(gp, sizeof gp, out, sizeof out, &notif);
        CHECK(!notif);
        CHECK(eq(out, n, { 0x42, 0x11, 0x0E, 0x08, 0x48, 0x00, 0x00, 0x19, 0x58, 0x30, 0x00, 0x00, 0x00 }));
    }
    {   // 4. A response frame is ignored; a foreign PID gets the 3-byte IPID reply; a fragment is ignored.
        uint8_t r[sizeof shokz]; memcpy(r, shokz, sizeof r); r[0] = 0x22; uint8_t out[64];
        CHECK(Avrcp::respond(r, sizeof r, out, sizeof out, nullptr) == 0);
        uint8_t f[sizeof shokz]; memcpy(f, shokz, sizeof f); f[1] = 0x11; f[2] = 0x0F;
        CHECK(eq(out, Avrcp::respond(f, sizeof f, out, sizeof out, nullptr), { 0x23, 0x11, 0x0F }));
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
    printf("avrcp_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
