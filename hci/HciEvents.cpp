#include "HciEvents.h"
#include <string.h>

uint8_t hciInquiryResultCount(const uint8_t *params, uint8_t len) {
    if (len < 1) return 0;
    uint8_t n = params[0];
    return (size_t)1 + (size_t)n * 14 <= len ? n : 0;
}

bool hciParseInquiryResult(const uint8_t *params, uint8_t len, uint8_t idx, HciInquiryResult *out) {
    uint8_t n = hciInquiryResultCount(params, len);
    if (idx >= n) return false;
    const uint8_t *bd   = params + 1 + 6 * idx;
    const uint8_t *psrm = params + 1 + 6 * n + idx;
    const uint8_t *cod  = params + 1 + 9 * n + 3 * idx;      // after 6n bd + n psrm + 2n reserved
    const uint8_t *clk  = params + 1 + 12 * n + 2 * idx;
    memcpy(out->bd, bd, 6);
    out->psrm = *psrm;
    out->cod = (uint32_t)cod[0] | ((uint32_t)cod[1] << 8) | ((uint32_t)cod[2] << 16);
    out->clockOffset = (uint16_t)(clk[0] | (clk[1] << 8));
    return true;
}

bool hciParseRemoteNameComplete(const uint8_t *params, uint8_t len, HciRemoteName *out) {
    if (len < 7) return false;
    out->status = params[0];
    memcpy(out->bd, params + 1, 6);
    size_t n = len - 7; if (n > 248) n = 248;
    memcpy(out->name, params + 7, n);
    out->name[n] = 0;
    return true;
}

void hciFormatBd(const uint8_t bd[6], char out[18]) {
    static const char hex[] = "0123456789ABCDEF";
    char *o = out;
    for (int i = 5; i >= 0; i--) {
        *o++ = hex[bd[i] >> 4]; *o++ = hex[bd[i] & 0xF];
        if (i) *o++ = ':';
    }
    *o = 0;
}
