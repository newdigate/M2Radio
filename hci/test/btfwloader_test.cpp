// Host unit tests for BtFwLoader (NXP V3 UART firmware download).
//
// ★ The start-indication fixture here is NOT invented: it is the byte sequence
// a real M2-MAYA-W161 transmitted on LPUART2 on 2026-08-23, captured in
// examples/networking/m2_sdio_probe/transcript_hw_evkb.txt.  Its CRC and its
// embedded chip id are both checked against values obtained independently --
// the CRC by computation, the chip id against what GET_HW_SPEC reports over
// SDIO.  That makes this the first fixture in this layer taken from silicon
// rather than from the specification.
#include "BtFwLoader.h"
#include <stdio.h>
#include <string.h>
#include <vector>
#include <deque>
#include <functional>

static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

// The card's real power-up frame, verbatim from the bench capture.
// ★ EVERY case that expects run() to get anywhere must io.feed(REAL_START_IND)
// first: the loader will not serve a byte until the card has greeted it.  This
// was forgotten twice while writing these tests, each time presenting as a
// wholesale failure of the case rather than as a missing greeting.
static const uint8_t REAL_START_IND[5] = { 0xAB, 0x01, 0x72, 0x00, 0x47 };

struct FakeIo : HciIo {
    std::vector<uint8_t> tx;
    std::deque<uint8_t>  rx;
    uint32_t now = 0;
    std::function<void(FakeIo &, const uint8_t *, size_t)> onWrite;
    size_t write(const uint8_t *p, size_t n) override {
        tx.insert(tx.end(), p, p + n);
        if (onWrite) onWrite(*this, p, n);
        return n;
    }
    int available() override { return (int)rx.size(); }
    int read() override { if (rx.empty()) return -1; uint8_t b = rx.front(); rx.pop_front(); return b; }
    uint32_t nowMs() override { return now; }
    void feed(const uint8_t *p, size_t n) { rx.insert(rx.end(), p, p + n); }
    void feed(std::initializer_list<uint8_t> b) { rx.insert(rx.end(), b); }
};

// Build a DATA_REQ frame with a correct CRC.
static void dataReq(std::vector<uint8_t> &out, uint16_t len, uint32_t off, uint16_t err = 0) {
    uint8_t f[10] = { BtFwLoader::DATA_REQ,
                      (uint8_t)(len & 0xFF), (uint8_t)(len >> 8),
                      (uint8_t)(off & 0xFF), (uint8_t)((off >> 8) & 0xFF),
                      (uint8_t)((off >> 16) & 0xFF), (uint8_t)((off >> 24) & 0xFF),
                      (uint8_t)(err & 0xFF), (uint8_t)(err >> 8), 0 };
    f[9] = BtFwLoader::crc8(f, 9);
    out.insert(out.end(), f, f + 10);
}

static FakeIo *g_io = nullptr;
static void idle1() { g_io->now += 1; }

int main() {
    {   // 1. The CRC matches the card's own byte -- the decode, pinned.
        CHECK(BtFwLoader::crc8(REAL_START_IND, 4) == 0x47);
        CHECK(BtFwLoader::crc8(REAL_START_IND, 4) == REAL_START_IND[4]);
        // and the ACK the host must send
        const uint8_t ack = BtFwLoader::ACK;
        CHECK(BtFwLoader::crc8(&ack, 1) == 0x92);
    }
    {   // 2. A start indication is parsed, ACKed, and its chip id read.
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[256]; for (int i = 0; i < 256; i++) img[i] = (uint8_t)i;
        ld.setImage(img, sizeof img);
        io.feed(REAL_START_IND, 5);
        io.onWrite = [](FakeIo &f, const uint8_t *p, size_t n) {
            if (n == 2 && p[0] == BtFwLoader::ACK && f.tx.size() == 2) {
                std::vector<uint8_t> r; dataReq(r, 256, 0);      // one chunk = whole image
                f.feed(r.data(), r.size());
            }
        };
        CHECK(ld.run(1000, 10, 5000, idle1) == BtFwLoader::OK);
        CHECK(ld.chipId() == 0x7201);                            // the real card's id
        CHECK(ld.loaderVer() == 0);
        CHECK(ld.startInds() == 1);
        CHECK(ld.chunks() == 1);
        CHECK(ld.bytesSent() == 256);
        CHECK(ld.crcErrors() == 0);
        CHECK(ld.retransmits() == 0);
        // tx must be: ACK, ACK, then the image
        CHECK(io.tx.size() == 2 + 2 + 256);
        CHECK(io.tx.size() > 1 && io.tx[0] == BtFwLoader::ACK && io.tx[1] == 0x92);
        CHECK(io.tx.size() == 260 && memcmp(io.tx.data() + 4, img, 256) == 0);
    }
    {   // 3. Multi-chunk: the bytes served are the bytes at the requested offsets
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[512]; for (int i = 0; i < 512; i++) img[i] = (uint8_t)(i * 7);
        ld.setImage(img, sizeof img);
        io.feed(REAL_START_IND, 5);
        int step = 0;
        io.onWrite = [&step](FakeIo &f, const uint8_t *p, size_t n) {
            if (!(n == 2 && p[0] == BtFwLoader::ACK)) return;
            std::vector<uint8_t> r;
            if (step == 0)      dataReq(r, 128, 0);
            else if (step == 1) dataReq(r, 128, 128);
            else if (step == 2) dataReq(r, 256, 256);
            if (step <= 2) f.feed(r.data(), r.size());
            step++;
        };
        CHECK(ld.run(1000, 10, 5000, idle1) == BtFwLoader::OK);
        CHECK(ld.chunks() == 3);
        CHECK(ld.bytesSent() == 512);
        CHECK(ld.maxOffset() == 512);
        // strip the four ACK frames (2 bytes each: start-ind + 3 data reqs)
        std::vector<uint8_t> payload;
        size_t i = 0;
        while (i < io.tx.size()) {
            if (io.tx[i] == BtFwLoader::ACK && i + 1 < io.tx.size() && io.tx[i+1] == 0x92) { i += 2; continue; }
            payload.push_back(io.tx[i++]);
        }
        CHECK(payload.size() == 512);
        CHECK(memcmp(payload.data(), img, 512) == 0);
    }
    {   // 4. A retransmit request (same offset twice) is served again and COUNTED
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[128]; memset(img, 0xA5, sizeof img);
        ld.setImage(img, sizeof img);
        io.feed(REAL_START_IND, 5);
        int step = 0;
        io.onWrite = [&step](FakeIo &f, const uint8_t *p, size_t n) {
            if (!(n == 2 && p[0] == BtFwLoader::ACK)) return;
            std::vector<uint8_t> r;
            if (step == 0)      dataReq(r, 128, 0);
            else if (step == 1) dataReq(r, 128, 0, 0x0001);   // CRC_ERR bit: send it again
            if (step <= 1) f.feed(r.data(), r.size());
            step++;
        };
        CHECK(ld.run(1000, 10, 5000, idle1) == BtFwLoader::OK);
        CHECK(ld.retransmits() == 1);
        CHECK(ld.chunks() == 2);
        CHECK(ld.bytesSent() == 256);          // 128 served twice
        CHECK(ld.lastCardErr() == 0x0001);     // the card's reason is RECORDED
    }
    {   // 5. A corrupted frame is rejected by CRC, counted, and does NOT abort:
        //    the card retries and the download completes.
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[64]; memset(img, 0x5A, sizeof img);
        ld.setImage(img, sizeof img);
        uint8_t bad[5]; memcpy(bad, REAL_START_IND, 5); bad[4] ^= 0xFF;   // wrong CRC
        io.feed(bad, 5);
        io.onWrite = [](FakeIo &f, const uint8_t *p, size_t n) {
            if (n == 2 && p[0] == BtFwLoader::ACK && f.tx.size() == 2) {
                std::vector<uint8_t> r; dataReq(r, 64, 0);
                f.feed(r.data(), r.size());
            }
        };
        io.feed(REAL_START_IND, 5);            // the good one, after the bad
        CHECK(ld.run(1000, 10, 5000, idle1) == BtFwLoader::OK);
        CHECK(ld.crcErrors() == 1);
        CHECK(ld.startInds() == 1);            // the bad frame was NOT counted as one
        CHECK(ld.bytesSent() == 64);
    }
    {   // 6. The pad-settle 0x00 the board emits before the first burst is skipped,
        //    not treated as a bad header.
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[64]; memset(img, 1, sizeof img);
        ld.setImage(img, sizeof img);
        io.feed({0x00});
        io.feed(REAL_START_IND, 5);
        io.onWrite = [](FakeIo &f, const uint8_t *p, size_t n) {
            if (n == 2 && p[0] == BtFwLoader::ACK && f.tx.size() == 2) {
                std::vector<uint8_t> r; dataReq(r, 64, 0); f.feed(r.data(), r.size());
            }
        };
        CHECK(ld.run(1000, 10, 5000, idle1) == BtFwLoader::OK);
        CHECK(ld.startInds() == 1);
    }
    {   // 7. Debris BEFORE the first frame is skipped and COUNTED, not fatal:
        //    a greeting truncated by the ring reset leaves its tail behind, and
        //    the [fwdnld] gate went intermittently red on exactly that.
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[64]; memset(img, 3, sizeof img);
        ld.setImage(img, sizeof img);
        io.feed({0x01, 0x72, 0x00, 0x47});      // the tail of a lost start indication
        io.feed(REAL_START_IND, 5);
        io.onWrite = [](FakeIo &f, const uint8_t *p, size_t n) {
            if (n == 2 && p[0] == BtFwLoader::ACK && f.tx.size() == 2) {
                std::vector<uint8_t> r; dataReq(r, 64, 0); f.feed(r.data(), r.size());
            }
        };
        CHECK(ld.run(1000, 10, 5000, idle1) == BtFwLoader::OK);
        CHECK(ld.preSyncSkipped() == 4);
        CHECK(ld.startInds() == 1);
    }
    {   // 7b. AFTER synchronising, a stray byte IS a fault, by name -- the
        //     tolerance above must not become a blanket one.
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[64]; ld.setImage(img, sizeof img);
        io.feed(REAL_START_IND, 5);
        io.onWrite = [](FakeIo &f, const uint8_t *p, size_t n) {
            if (n == 2 && p[0] == BtFwLoader::ACK && f.tx.size() == 2) f.feed({0x5C});
        };
        CHECK(ld.run(1000, 10, 5000, idle1) == BtFwLoader::BAD_HEADER);
        CHECK(strcmp(BtFwLoader::errorName(BtFwLoader::BAD_HEADER), "bad_header") == 0);
    }
    {   // 8. A request past the end of the image is refused rather than padded --
        //    padding would feed the card rubbish it would fail to authenticate
        //    far away from the cause.
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[64]; ld.setImage(img, sizeof img);
        io.feed(REAL_START_IND, 5);
        io.onWrite = [](FakeIo &f, const uint8_t *p, size_t n) {
            if (n == 2 && p[0] == BtFwLoader::ACK && f.tx.size() == 2) {
                std::vector<uint8_t> r; dataReq(r, 128, 0);   // 128 > 64
                f.feed(r.data(), r.size());
            }
        };
        CHECK(ld.run(1000, 10, 5000, idle1) == BtFwLoader::BAD_OFFSET);
        CHECK(ld.bytesSent() == 0);            // nothing was sent
    }
    {   // 9. Silence: NO_START, by name, and no image bytes emitted.
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[16]; ld.setImage(img, sizeof img);
        CHECK(ld.run(200, 10, 5000, idle1) == BtFwLoader::NO_START);
        CHECK(io.tx.empty());
        CHECK(strcmp(BtFwLoader::errorName(BtFwLoader::NO_START), "no_start_indication") == 0);
    }
    {   // 10. A card that greets and then stops asking is STALLED, not OK --
        //     an image that was never fully delivered must not read as success.
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[1024]; ld.setImage(img, sizeof img);
        io.feed(REAL_START_IND, 5);
        io.onWrite = [](FakeIo &f, const uint8_t *p, size_t n) {
            if (n == 2 && p[0] == BtFwLoader::ACK && f.tx.size() == 2) {
                std::vector<uint8_t> r; dataReq(r, 128, 0);   // one chunk of 1024, then silence
                f.feed(r.data(), r.size());
            }
        };
        CHECK(ld.run(1000, 10, 400, idle1) == BtFwLoader::STALLED);
        CHECK(ld.bytesSent() == 128);
        CHECK(ld.maxOffset() == 128);          // and it says how far it got
    }
    {   // 11. No image at all is refused before a byte is written.
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        CHECK(ld.run(1000, 10, 5000, idle1) == BtFwLoader::NO_IMAGE);
        CHECK(io.tx.empty());
    }
    {   // 12. CRC-32 matches the SHIPPED IMAGE's own block header.
        //     The first 16 bytes of uartIW416_bt.bin are
        //       01 00 00 00 10 20 0a 00 00 08 00 00 83 df a3 ea
        //     = type 1, addr 0x000A2010, length 2048, then a big-endian CRC-32
        //     of the preceding twelve.  Recomputing it is how the CRC variant
        //     was identified, so it is pinned here rather than described.
        const uint8_t hdr12[12] = {0x01,0x00,0x00,0x00,0x10,0x20,0x0A,0x00,0x00,0x08,0x00,0x00};
        CHECK(BtFwLoader::crc32(hdr12, 12) == 0x83DFA3EAu);
    }
    {   // 13. The injected UART-config block: served before the image, with the
        //     card's offsets then mapping past it into the file.
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[256]; for (int i = 0; i < 256; i++) img[i] = (uint8_t)(0xC0 + i);
        ld.setImage(img, sizeof img);
        ld.enableUartConfig(BtFwLoader::CLKDIV_115200, BtFwLoader::UARTDIV_115200);
        io.feed(REAL_START_IND, 5);
        int step = 0;
        io.onWrite = [&step](FakeIo &f, const uint8_t *p, size_t n) {
            if (!(n == 2 && p[0] == BtFwLoader::ACK)) return;
            std::vector<uint8_t> r;
            if (step == 0)      dataReq(r, 16, 0);        // header -> gets the config header
            else if (step == 1) dataReq(r, 52, 16);       // -> gets the 52-byte config body
            else if (step == 2) dataReq(r, 256, 68);      // stream 68 == file 0
            if (step <= 2) f.feed(r.data(), r.size());
            step++;
        };
        CHECK(ld.run(1000, 10, 5000, idle1) == BtFwLoader::OK);
        CHECK(ld.injectedBytes() == 68);
        CHECK(ld.bytesSent() == 256);                     // only the IMAGE counts as sent
        CHECK(ld.maxOffset() == 256);
        // Reconstruct what went out, minus the ACK frames.
        std::vector<uint8_t> out; size_t i = 0;
        while (i < io.tx.size()) {
            if (io.tx[i] == BtFwLoader::ACK && i + 1 < io.tx.size() && io.tx[i+1] == 0x92) { i += 2; continue; }
            out.push_back(io.tx[i++]);
        }
        CHECK(out.size() == 16 + 52 + 256);
        // the type-5 header, its length field and its CRC
        CHECK(out.size() > 15 && out[0] == 0x05);
        CHECK(out.size() > 15 && out[8] == 52 && out[9] == 0 && out[10] == 0 && out[11] == 0);
        if (out.size() > 15) {
            uint8_t h[12]; memcpy(h, out.data(), 12);
            uint32_t hc = BtFwLoader::crc32(h, 12);
            CHECK(out[12] == (uint8_t)(hc >> 24) && out[15] == (uint8_t)hc);   // big-endian
        }
        // the first register write, little-endian, and the body CRC
        if (out.size() > 68) {
            CHECK(out[16] == 0x8F && out[17] == 0x00 && out[18] == 0x00 && out[19] == 0x7F);
            uint8_t b[48]; memcpy(b, out.data() + 16, 48);
            uint32_t bc = BtFwLoader::crc32(b, 48);
            CHECK(out[64] == (uint8_t)(bc >> 24) && out[67] == (uint8_t)bc);
        }
        // and then the image itself, from file offset 0
        CHECK(out.size() == 324 && memcmp(out.data() + 68, img, 256) == 0);
    }
    {   // 14. With injection ON, an offset that lands INSIDE the injected block
        //     is a fault -- it cannot be mapped into the file.
        FakeIo io; g_io = &io; BtFwLoader ld(io);
        static uint8_t img[64]; ld.setImage(img, sizeof img);
        ld.enableUartConfig(BtFwLoader::CLKDIV_115200, BtFwLoader::UARTDIV_115200);
        io.feed(REAL_START_IND, 5);
        int step = 0;
        io.onWrite = [&step](FakeIo &f, const uint8_t *p, size_t n) {
            if (!(n == 2 && p[0] == BtFwLoader::ACK)) return;
            std::vector<uint8_t> r;
            if (step == 0)      dataReq(r, 16, 0);
            else if (step == 1) dataReq(r, 52, 16);
            else if (step == 2) dataReq(r, 16, 40);       // 40 < 68: inside the injection
            if (step <= 2) f.feed(r.data(), r.size());
            step++;
        };
        CHECK(ld.run(1000, 10, 5000, idle1) == BtFwLoader::BAD_OFFSET);
    }
    printf("btfwloader_test: %d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
