// Host tests for the SDP SERVER half of Sdp: the AudioSource record a sink reads before it
// will talk AVDTP to us.  Every request here is a real sink's bytes; the first response is
// the Mac's, byte for byte (Mac->Shokz PacketLogger reference, 2026-09-03).
#include "Sdp.h"
#include "SdpServer.h"
#include <stdio.h>
#include <string.h>
#include <vector>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
static std::vector<uint8_t> serve(std::initializer_list<uint8_t> req, uint16_t mtu = 672) {
    std::vector<uint8_t> r(req); uint8_t out[128]; uint16_t n = Sdp::serve(r.data(), (uint16_t)r.size(), mtu, out, sizeof out);
    return std::vector<uint8_t>(out, out + n); }
static bool eq(const std::vector<uint8_t> &v, std::initializer_list<uint8_t> want) { return v == std::vector<uint8_t>(want); }
int main() {
    {   // 1. The Shokz's query of the SOURCE, verbatim (frame 749 of the reference): ServiceSearchAttributeRequest,
        // txn 1, pattern {AudioSource 0x110A}, max 32 bytes, attribute {0x0009 BluetoothProfileDescriptorList}.
        // The Mac's answer, verbatim: one record, 0x0009 = { { A2DP 0x110D, v1.3 } }.  Ours must be identical.
        auto r = serve({ 0x06, 0x00, 0x01, 0x00, 0x0D, 0x35, 0x03, 0x19, 0x11, 0x0A, 0x00, 0x20, 0x35, 0x03, 0x09, 0x00, 0x09, 0x00 });
        CHECK(eq(r, { 0x07, 0x00, 0x01, 0x00, 0x14, 0x00, 0x11, 0x35, 0x0F, 0x35, 0x0D, 0x09, 0x00, 0x09, 0x35, 0x08, 0x35, 0x06,
                      0x19, 0x11, 0x0D, 0x09, 0x01, 0x03, 0x00 }));
    }
    {   // 2. The Shokz's AVRCP-Target query (frame 677): pattern {0x110C}, attrs {0x0009, 0x0311}.  We have no AVRCP record:
        // a well-formed EMPTY ServiceSearchAttributeResponse (AttributeListsByteCount 2 = an empty outer DES), not silence.
        auto r = serve({ 0x06, 0x00, 0x01, 0x00, 0x10, 0x35, 0x03, 0x19, 0x11, 0x0C, 0x00, 0x26, 0x35, 0x06, 0x09, 0x00, 0x09, 0x09, 0x03, 0x11, 0x00 });
        CHECK(eq(r, { 0x07, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x35, 0x00, 0x00 }));
    }
    {   // 3. The OneOdio's DeviceID/PnP query (BT-2 transcript): pattern {PnPInformation 0x1200}, attrs 0x0201..0x0203 (a range).
        auto r = serve({ 0x06, 0x00, 0x02, 0x00, 0x0F, 0x35, 0x03, 0x19, 0x12, 0x00, 0x03, 0xF0, 0x35, 0x05, 0x0A, 0x02, 0x01, 0x02, 0x03, 0x00 });
        CHECK(eq(r, { 0x07, 0x00, 0x02, 0x00, 0x05, 0x00, 0x02, 0x35, 0x00, 0x00 }));
    }
    {   // 4. Whole record under the Shokz's 32-byte cap: a full-range attribute request walks out in chunks through the
        // continuation state, reassembling to ONE well-formed attribute list that carries every attribute we publish.
        std::vector<uint8_t> all; std::vector<uint8_t> cont; int rounds = 0;
        for (;;) {
            std::vector<uint8_t> req = { 0x06, 0x00, 0x03, 0x00, 0x00, 0x35, 0x03, 0x19, 0x11, 0x0A, 0x00, 0x20, 0x35, 0x05, 0x0A, 0x00, 0x00, 0xFF, 0xFF };
            req.push_back((uint8_t)cont.size()); req.insert(req.end(), cont.begin(), cont.end());
            req[4] = (uint8_t)(req.size() - 5);
            uint8_t out[128]; uint16_t n = Sdp::serve(req.data(), (uint16_t)req.size(), 672, out, sizeof out);
            CHECK(n >= 8 && out[0] == 0x07 && out[1] == 0x00 && out[2] == 0x03);
            uint16_t cnt = (uint16_t)((out[5] << 8) | out[6]); CHECK(cnt <= 32 && 7 + cnt < n);
            all.insert(all.end(), out + 7, out + 7 + cnt);
            uint8_t cl = out[7 + cnt]; CHECK(7 + cnt + 1 + cl == n);
            cont.assign(out + 8 + cnt, out + 8 + cnt + cl);
            if (++rounds > 8 || cl == 0) break;
        }
        CHECK(rounds >= 2 && rounds <= 8);                      // it really took continuation to get it out under 32 bytes
        CHECK(all.size() > 32 && all[0] == 0x35 && all[1] == (uint8_t)(all.size() - 2));   // outer DES, short-form length
        // every published attribute id appears, in ascending order, inside the record
        static const uint16_t ids[] = { 0x0000, 0x0001, 0x0004, 0x0005, 0x0009, 0x0311 }; size_t at = 0;
        for (size_t i = 0; i + 2 < all.size(); i++) if (all[i] == 0x09 && at < 6 && all[i + 1] == (ids[at] >> 8) && all[i + 2] == (ids[at] & 0xFF)) at++;
        CHECK(at == 6);
        // the ProtocolDescriptorList names L2CAP PSM 0x0019 and AVDTP 0x0103, the same bytes BT-2 read from the headsets
        static const uint8_t pdl[18] = { 0x35, 0x10, 0x35, 0x06, 0x19, 0x01, 0x00, 0x09, 0x00, 0x19, 0x35, 0x06, 0x19, 0x00, 0x19, 0x09, 0x01, 0x03 };
        bool sawPdl = false; for (size_t i = 0; i + 18 <= all.size(); i++) if (memcmp(&all[i], pdl, 18) == 0) sawPdl = true;
        CHECK(sawPdl);
    }
    {   // 5. ServiceSearchRequest (0x02) for AudioSource -> one handle 0x00010000; for a UUID we lack -> zero handles.
        auto r = serve({ 0x02, 0x00, 0x04, 0x00, 0x08, 0x35, 0x03, 0x19, 0x11, 0x0A, 0x00, 0x10, 0x00 });
        CHECK(eq(r, { 0x03, 0x00, 0x04, 0x00, 0x09, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00 }));
        r = serve({ 0x02, 0x00, 0x05, 0x00, 0x08, 0x35, 0x03, 0x19, 0x11, 0x1E, 0x00, 0x10, 0x00 });   // Hands-Free
        CHECK(eq(r, { 0x03, 0x00, 0x05, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 }));
    }
    {   // 6. ServiceAttributeRequest (0x04) by handle: ours answers; an unknown handle -> ErrorResponse 0x0002.
        auto r = serve({ 0x04, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x01, 0x00, 0x00, 0x00, 0x20, 0x35, 0x03, 0x09, 0x00, 0x09, 0x00 });
        CHECK(eq(r, { 0x05, 0x00, 0x06, 0x00, 0x12, 0x00, 0x0F, 0x35, 0x0D, 0x09, 0x00, 0x09, 0x35, 0x08, 0x35, 0x06, 0x19, 0x11, 0x0D, 0x09, 0x01, 0x03, 0x00 }));
        r = serve({ 0x04, 0x00, 0x07, 0x00, 0x0C, 0x00, 0x02, 0x00, 0x00, 0x00, 0x20, 0x35, 0x03, 0x09, 0x00, 0x09, 0x00 });
        CHECK(eq(r, { 0x01, 0x00, 0x07, 0x00, 0x02, 0x00, 0x02 }));
    }
    {   // 7. Garbage: a PDU id we do not serve or a truncated request -> ErrorResponse 0x0003 (invalid request syntax); a
        // continuation state we never issued -> 0x0005 (invalid continuation state); never a crash and never silence.
        // And a response never exceeds the peer's L2CAP MTU (the Shokz configures 48 on its SDP channel -- BT-2 transcript).
        auto r = serve({ 0x09, 0x00, 0x08, 0x00, 0x00 });                   CHECK(eq(r, { 0x01, 0x00, 0x08, 0x00, 0x02, 0x00, 0x03 }));
        r = serve({ 0x06, 0x00, 0x09, 0x00, 0x0D, 0x35, 0x03 });            CHECK(eq(r, { 0x01, 0x00, 0x09, 0x00, 0x02, 0x00, 0x03 }));
        r = serve({ 0x06 });                                                 CHECK(r.size() == 0);   // not even a txn id to answer
        r = serve({ 0x06, 0x00, 0x0A, 0x00, 0x0F, 0x35, 0x03, 0x19, 0x11, 0x0A, 0xFF, 0xFF, 0x35, 0x05, 0x0A, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0xFF, 0xFF });
        CHECK(eq(r, { 0x01, 0x00, 0x0A, 0x00, 0x02, 0x00, 0x05 }));
        r = serve({ 0x06, 0x00, 0x0B, 0x00, 0x0F, 0x35, 0x03, 0x19, 0x11, 0x0A, 0xFF, 0xFF, 0x35, 0x05, 0x0A, 0x00, 0x00, 0xFF, 0xFF, 0x00 }, 48);   // Shokz SDP MTU 48
        CHECK(r.size() <= 48 && r[0] == 0x07 && r[r.size() - 3] == 2);      // chunked to fit, continuation pending
    }
    {   // 8. SdpServer on a PEER-INITIATED SDP channel: the request is only RECORDED from the RX path (nothing written --
        // the bus-fault rule), and service() writes the reply on the peer's CID.  Our own client channel's traffic is
        // not consumed (it is a response, not a request).  A truncated request still gets an ErrorResponse.
        struct CapIo : HciIo {
            std::vector<std::vector<uint8_t> > tx;
            size_t write(const uint8_t *p, size_t n) override { tx.push_back(std::vector<uint8_t>(p, p + n)); return n; }
            int available() override { return 0; } int read() override { return -1; } uint32_t nowMs() override { return 0; }
        } io;
        L2cap l(io); l.begin(0x0001, 100); l.acceptIncoming(true);
        static SdpServer srv; static bool consumed[2];
        l.onData([](void *, L2cap::Channel &ch, const uint8_t *p, uint16_t len) { consumed[ch.peerInitiated ? 1 : 0] = srv.onData(ch, p, len); }, nullptr);
        auto feed = [&](uint16_t cid, std::vector<uint8_t> pl) {
            std::vector<uint8_t> v = { (uint8_t)pl.size(), 0, (uint8_t)cid, (uint8_t)(cid >> 8) }; v.insert(v.end(), pl.begin(), pl.end());
            l.onAcl(0x0001, v.data(), (uint16_t)v.size()); };
        // the peer opens PSM 0x0001 at us: CONN_REQ scid=0x0E85 -> we allocate 0x0080; it configures MTU 48 (the Shokz's)
        feed(0x0001, { 0x02, 0x05, 0x04, 0x00, 0x01, 0x00, 0x85, 0x0E }); l.service();
        L2cap::Channel *ch = l.byRemote(0x0E85); CHECK(ch != nullptr && ch->peerInitiated && ch->localCid == 0x0080);
        feed(0x0001, { 0x04, 0x06, 0x08, 0x00, 0x80, 0x00, 0x00, 0x00, 0x01, 0x02, 0x30, 0x00 });
        feed(0x0001, { 0x05, 0x11, 0x06, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 }); l.service();
        CHECK(ch->state == L2cap::OPEN && ch->mtuOut == 48);
        io.tx.clear();
        feed(0x0080, { 0x06, 0x00, 0x01, 0x00, 0x0D, 0x35, 0x03, 0x19, 0x11, 0x0A, 0x00, 0x20, 0x35, 0x03, 0x09, 0x00, 0x09, 0x00 });
        CHECK(consumed[1]); CHECK(io.tx.empty()); CHECK(srv.pending());
        srv.service(l); l.service();
        CHECK(io.tx.size() == 1 && io.tx[0].size() == 9 + 25);
        CHECK(io.tx[0][7] == 0x85 && io.tx[0][8] == 0x0E);                                              // on the peer's CID
        CHECK(memcmp(&io.tx[0][9], "\x07\x00\x01\x00\x14\x00\x11\x35\x0F\x35\x0D\x09\x00\x09\x35\x08\x35\x06\x19\x11\x0D\x09\x01\x03\x00", 25) == 0);
        CHECK(!srv.pending() && srv.answered() == 1);
        // our own SDP CLIENT channel (we initiated it): a response arriving there is NOT a request for the server
        CHECK(l.connect(Sdp::PSM, 0x0040) != nullptr); l.service();
        feed(0x0001, { 0x03, 0x10, 0x08, 0x00, 0x02, 0x0D, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 });
        feed(0x0001, { 0x04, 0x07, 0x08, 0x00, 0x40, 0x00, 0x00, 0x00, 0x01, 0x02, 0x30, 0x00 });
        feed(0x0001, { 0x05, 0x12, 0x06, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 }); l.service();
        consumed[0] = true; feed(0x0040, { 0x07, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x35, 0x00, 0x00 });
        CHECK(!consumed[0]);
        // a truncated request on the peer channel -> ErrorResponse 0x0003, not silence
        io.tx.clear(); feed(0x0080, { 0x06, 0x00, 0x02, 0x00, 0x0D, 0x35 }); srv.service(l); l.service();
        CHECK(io.tx.size() == 1 && io.tx[0].size() == 9 + 7 && io.tx[0][9] == 0x01 && io.tx[0][9 + 6] == 0x03);
    }
    printf("sdp_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
