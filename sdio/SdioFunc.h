// SDIO function-level operations layered on SdioHost: function enable, block
// size, and CMD53 block transfers.  Still generic SDIO -- nothing here knows
// about the IW416.  See iw416/ for the module-specific part.
#pragma once
#include "SdioHost.h"

class SdioFunc {
public:
    explicit SdioFunc(SdioHost &host) : m_host(host) {}

    // CCCR I/O Enable (0x02) / I/O Ready (0x03).  Sets the bit for `fn`, then
    // polls until the card reports the function ready.
    SdioHost::Status enableFunction(uint8_t fn, uint32_t timeoutMs = 1000);

    // Block size for a function.  Function 0 uses CCCR 0x10/0x11; functions
    // 1..7 use their FBR at 0x*10/0x*11 (fn1 -> 0x110/0x111).
    SdioHost::Status setBlockSize(uint8_t fn, uint16_t bytes);

private:
    SdioHost &m_host;
};
