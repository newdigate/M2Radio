#include "Rtp.h"
#include <stdio.h>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
int main() {
    uint8_t o[16];
    {   // 1. Header length + fixed fields: V=2/P=0/X=0/CC=0 -> 0x80; M=0|PT=96 -> 0x60.
        uint16_t n = Rtp::header(o, /*seq*/0x1234, /*ts*/0x00ABCDEF, /*frames*/5);
        CHECK(n == 13);
        CHECK(o[0] == 0x80);
        CHECK(o[1] == 96);                                   // 0x60, M bit clear
    }
    {   // 2. seq big-endian at [2..3], timestamp big-endian at [4..7].
        Rtp::header(o, 0x1234, 0x00ABCDEF, 5);
        CHECK(o[2] == 0x12 && o[3] == 0x34);
        CHECK(o[4] == 0x00 && o[5] == 0xAB && o[6] == 0xCD && o[7] == 0xEF);
    }
    {   // 3. SSRC big-endian at [8..11] = Rtp::SSRC.
        Rtp::header(o, 0, 0, 0);
        uint32_t ssrc = ((uint32_t)o[8] << 24) | ((uint32_t)o[9] << 16) | ((uint32_t)o[10] << 8) | o[11];
        CHECK(ssrc == Rtp::SSRC);
    }
    {   // 4. A2DP media payload header at [12]: no fragmentation (top 3 bits 0), frame count in low nibble.
        Rtp::header(o, 0, 0, 8);   CHECK(o[12] == 0x08);
        Rtp::header(o, 0, 0, 1);   CHECK(o[12] == 0x01);
        CHECK((o[12] & 0xE0) == 0);                          // F=S=L=0
    }
    printf("rtp_test: %d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
