#include "HciEvents.h"
#include <stdio.h>
#include <string.h>

static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

int main() {
    {   // Inquiry Result with TWO responses, field-major layout (Vol 4 Part E 7.7.2)
        const uint8_t p[] = {
            0x02,                                            // Num_Responses
            0x01, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA,              // BD_ADDR[0]  (LE on the wire -> AA:BB:CC:DD:EE:01)
            0x02, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA,              // BD_ADDR[1]
            0x01, 0x01,                                      // Page_Scan_Repetition_Mode[0..1]
            0x00, 0x00, 0x00, 0x00,                          // Reserved[0..1]
            0x04, 0x04, 0x24,                                // Class_Of_Device[0] = 0x240404
            0x08, 0x04, 0x24,                                // Class_Of_Device[1] = 0x240408
            0x34, 0x12,                                      // Clock_Offset[0] = 0x1234
            0x78, 0x56,                                      // Clock_Offset[1] = 0x5678
        };
        CHECK(hciInquiryResultCount(p, sizeof p) == 2);
        HciInquiryResult r;
        CHECK(hciParseInquiryResult(p, sizeof p, 0, &r));
        CHECK(r.bd[5] == 0xAA && r.bd[0] == 0x01); CHECK(r.psrm == 1); CHECK(r.cod == 0x240404); CHECK(r.clockOffset == 0x1234);
        CHECK(hciParseInquiryResult(p, sizeof p, 1, &r));
        CHECK(r.bd[0] == 0x02); CHECK(r.cod == 0x240408); CHECK(r.clockOffset == 0x5678);
        CHECK(!hciParseInquiryResult(p, sizeof p, 2, &r));                  // out of range
        CHECK(hciInquiryResultCount(p, sizeof p - 1) == 0);                // truncated -> none
    }
    {   // Remote Name Request Complete: status, BD_ADDR, 248-byte NUL-padded name
        uint8_t p[1 + 6 + 248]; memset(p, 0, sizeof p);
        p[0] = 0x00; const uint8_t bd[6] = { 0x01, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA }; memcpy(p + 1, bd, 6);
        memcpy(p + 7, "FAKE-HEADSET-01", 15);
        HciRemoteName n;
        CHECK(hciParseRemoteNameComplete(p, sizeof p, &n));
        CHECK(n.status == 0); CHECK(memcmp(n.bd, bd, 6) == 0); CHECK(strcmp(n.name, "FAKE-HEADSET-01") == 0);
        CHECK(!hciParseRemoteNameComplete(p, 6, &n));                       // too short
        // a name that fills all 248 bytes is still terminated
        memset(p + 7, 'x', 248);
        CHECK(hciParseRemoteNameComplete(p, sizeof p, &n)); CHECK(strlen(n.name) == 248);
    }
    {   // BD_ADDR formatting: MSB first, the usual representation
        const uint8_t bd[6] = { 0x66, 0x55, 0x44, 0x33, 0x22, 0x11 };
        char s[18]; hciFormatBd(bd, s);
        CHECK(strcmp(s, "11:22:33:44:55:66") == 0);
    }
    printf("hcievents_test: %d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
