#include "MediaPacketizer.h"
#include "Rtp.h"
#include <stdio.h>
#include <string.h>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

// A fake sink: records every packet; `accept` gates whether drain() may send.
struct FakeSink {
    static const int MAXP = 256;
    uint8_t pkt[MAXP][2048]; uint16_t len[MAXP]; int n = 0;
    bool accept = true;
    static bool send(void *ctx, const uint8_t *p, uint16_t l) {
        FakeSink *s = (FakeSink *)ctx;
        if (!s->accept || s->n >= MAXP) return false;
        memcpy(s->pkt[s->n], p, l); s->len[s->n] = l; s->n++;
        return true;
    }
};
// A 119-byte fake SBC frame whose first byte is the sync word; byte 4 carries `id`.
static void fakeFrame(uint8_t *f, uint8_t id) { memset(f, 0, 119); f[0] = 0x9C; f[4] = id; }

int main() {
    // MTU 1008 -> (1008 - 13) / 119 = 8 frames per packet.
    {   // 1. Batch to MTU: push 8 frames, drain -> exactly ONE packet of 13 + 8*119.
        MediaPacketizer pk; pk.begin(1008);
        uint8_t f[119];
        for (int i = 0; i < 8; i++) { fakeFrame(f, (uint8_t)i); pk.push(f, 119); }
        FakeSink s; pk.drain(FakeSink::send, &s);
        CHECK(s.n == 1);
        CHECK(s.len[0] == 13 + 8 * 119);
        CHECK(s.pkt[0][12] == 8);                            // frame count in the payload header
        CHECK(s.pkt[0][0] == 0x80 && s.pkt[0][1] == 96);     // it is a real RTP header
        CHECK(s.pkt[0][13] == 0x9C);                          // first SBC frame's sync survived
        CHECK(pk.packets() == 1 && pk.frames() == 8 && pk.drops() == 0);
    }
    {   // 2. Sequence increments and timestamp advances by 128 per frame across packets.
        MediaPacketizer pk; pk.begin(1008);
        uint8_t f[119];
        for (int i = 0; i < 10; i++) { fakeFrame(f, (uint8_t)i); pk.push(f, 119); }
        FakeSink s; pk.drain(FakeSink::send, &s);             // 10 frames -> 8 + 2
        CHECK(s.n == 2);
        uint16_t seq0 = (s.pkt[0][2] << 8) | s.pkt[0][3];
        uint16_t seq1 = (s.pkt[1][2] << 8) | s.pkt[1][3];
        CHECK(seq1 == (uint16_t)(seq0 + 1));
        uint32_t ts0 = ((uint32_t)s.pkt[0][4]<<24)|((uint32_t)s.pkt[0][5]<<16)|((uint32_t)s.pkt[0][6]<<8)|s.pkt[0][7];
        uint32_t ts1 = ((uint32_t)s.pkt[1][4]<<24)|((uint32_t)s.pkt[1][5]<<16)|((uint32_t)s.pkt[1][6]<<8)|s.pkt[1][7];
        CHECK(ts1 == ts0 + 8 * 128);                          // first packet carried 8 frames
        CHECK(s.pkt[1][12] == 2);                             // tail packet has 2 frames
    }
    {   // 3. Credit starvation: sink refuses -> nothing sent, frames retained, no drops yet.
        MediaPacketizer pk; pk.begin(1008);
        uint8_t f[119];
        for (int i = 0; i < 4; i++) { fakeFrame(f, (uint8_t)i); pk.push(f, 119); }
        FakeSink s; s.accept = false; pk.drain(FakeSink::send, &s);
        CHECK(s.n == 0 && pk.packets() == 0 && pk.drops() == 0);
        s.accept = true; pk.drain(FakeSink::send, &s);        // now it flushes
        CHECK(s.n == 1 && s.pkt[0][12] == 4);
    }
    {   // 4. Ring overflow drops the OLDEST: fill 64, push 4 more with the sink refusing,
        //    then drain -> exactly 64 frames delivered, drops == 4, and the FIRST four ids are gone.
        MediaPacketizer pk; pk.begin(1008);
        uint8_t f[119];
        for (int i = 0; i < 64 + 4; i++) { fakeFrame(f, (uint8_t)i); pk.push(f, 119); }
        CHECK(pk.drops() == 4);
        CHECK(pk.queueHighWater() == 64);
        FakeSink s; pk.drain(FakeSink::send, &s);
        int delivered = 0; for (int p = 0; p < s.n; p++) delivered += s.pkt[p][12];
        CHECK(delivered == 64);
        CHECK(s.pkt[0][13 + 4] == 4);                         // first surviving frame's id byte is 4, not 0
    }
    {   // 5. advanceRd: monotonic-forward commit that never clobbers a concurrent drop.
        //    cur = m_rd now (maybe advanced by an ISR drop since gather); rd0 = index
        //    drain gathered from; n = frames drain consumed.
        CHECK(MediaPacketizer::advanceRd(10, 10, 3) == 13);   // no drop: land on rd0+n
        CHECK(MediaPacketizer::advanceRd(12, 10, 3) == 13);   // ISR dropped 2 (<n): still rd0+n
        CHECK(MediaPacketizer::advanceRd(13, 10, 3) == 13);   // ISR dropped exactly n: equal, keep
        CHECK(MediaPacketizer::advanceRd(14, 10, 3) == 14);   // ISR dropped >n: keep ISR's further pos
        CHECK(MediaPacketizer::advanceRd(64, 63, 3) == 1);    // wrap: (63+3)%65 == 1
        CHECK(MediaPacketizer::advanceRd(1, 63, 3) == 1);     // ISR wrapped past rd0+n: keep cur
    }
    printf("mediapacketizer_test: %d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
