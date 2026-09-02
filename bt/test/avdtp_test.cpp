#include "Avdtp.h"
#include "Sdp.h"
#include <stdio.h>
#include <string.h>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
int main() {
    {   // 1. SDP: the AudioSink/ProtocolDescriptorList request is the exact 18 bytes proven on three peers
        uint8_t b[32]; uint16_t n = Sdp::buildAudioSinkPdlRequest(b, 0x0001);
        static const uint8_t want[18] = { 0x06, 0x00,0x01, 0x00,0x0D, 0x35,0x03,0x19,0x11,0x0B, 0x03,0xF0, 0x35,0x03,0x09,0x00,0x04, 0x00 };
        CHECK(n == 18 && memcmp(b, want, 18) == 0);
        // and the response seen on the wire yields AVDTP v1.3
        static const uint8_t rsp[39] = { 0x07,0x00,0x01,0x00,0x1E,0x00,0x1B,0x36,0x00,0x18,0x36,0x00,0x15,0x09,0x00,0x04,0x35,0x10,
                                         0x35,0x06,0x19,0x01,0x00,0x09,0x00,0x19, 0x35,0x06,0x19,0x00,0x19,0x09,0x01,0x03, 0x00 };
        CHECK(Sdp::parseAvdtpVersion(rsp, 39) == 0x0103);
    }
    {   // 2. AVDTP command encodings (single packet, our transaction labels)
        uint8_t b[32]; uint16_t n;
        n = Avdtp::buildDiscover(b, 1);                       CHECK(n == 2 && b[0] == 0x10 && b[1] == 0x01);
        n = Avdtp::buildGetCapabilities(b, 2, 1);             CHECK(n == 3 && b[0] == 0x20 && b[1] == 0x02 && b[2] == (1 << 2));
        Avdtp::SbcConfig cfg = { 44100, Avdtp::JOINT_STEREO, 16, 8, Avdtp::LOUDNESS, 2, 53 };
        n = Avdtp::buildSetConfiguration(b, 3, /*acp seid*/ 1, /*int seid*/ 1, cfg);
        // hdr, sig, ACP SEID<<2, INT SEID<<2, [cat 1 media transport, len 0], [cat 7 media codec, len 6: media type audio<<4, codec SBC=0, cie 21 15 02 35]
        static const uint8_t want[14] = { 0x30, 0x03, 1 << 2, 1 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x35 };
        CHECK(n == 14 && memcmp(b, want, 14) == 0);
        n = Avdtp::buildOpen(b, 4, 1);                        CHECK(n == 3 && b[0] == 0x40 && b[1] == 0x06 && b[2] == (1 << 2));
        n = Avdtp::buildStart(b, 5, 1);                       CHECK(n == 3 && b[0] == 0x50 && b[1] == 0x07 && b[2] == (1 << 2));
    }
    {   // 3. Parsing: a Discover accept with two SNK SEPs, and SBC capabilities out of GET_CAPABILITIES
        static const uint8_t disc[6] = { 0x12, 0x01, 0x04, 0x08, 0x08, 0x08 };
        Avdtp::Sep seps[4]; uint8_t n = Avdtp::parseDiscover(disc, 6, seps, 4);
        CHECK(n == 2 && seps[0].seid == 1 && seps[0].sink && seps[0].audio && seps[1].seid == 2);
        static const uint8_t caps[12] = { 0x22, 0x02, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0x35 };  // all rates/modes, all blocks/subbands/alloc, bitpool 2..53
        Avdtp::SbcCaps c; CHECK(Avdtp::parseSbcCaps(caps, 12, c));
        CHECK(c.rates == 0xF && c.modes == 0xF && c.blocks == 0xF && c.subbands == 0x3 && c.alloc == 0x3 && c.minBitpool == 2 && c.maxBitpool == 53);
        static const uint8_t rej[3] = { 0x33, 0x03, 0x29 };   // SET_CONFIGURATION reject, error 0x29 (unsupported configuration)
        CHECK(Avdtp::responseType(rej[0]) == Avdtp::REJECT && Avdtp::rejectError(rej, 3) == 0x29);
    }
    printf("avdtp_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
