#include "SdioFunc.h"

// CCCR registers (SDIO simplified spec, function 0)
static const uint32_t CCCR_IO_ENABLE  = 0x02;
static const uint32_t CCCR_IO_READY   = 0x03;
static const uint32_t CCCR_FN0_BLKSZ0 = 0x10;

SdioHost::Status SdioFunc::enableFunction(uint8_t fn, uint32_t timeoutMs) {
    uint8_t en = 0;
    SdioHost::Status s = m_host.cmd52Read(0, CCCR_IO_ENABLE, &en);
    if (s != SdioHost::OK) return s;
    s = m_host.cmd52Write(0, CCCR_IO_ENABLE, (uint8_t)(en | (1u << fn)));
    if (s != SdioHost::OK) return s;

    // The card raises the matching bit in I/O Ready when the function has
    // finished coming up; the spec allows this to take a while.
    for (uint32_t i = 0; i < timeoutMs; i++) {
        uint8_t rdy = 0;
        s = m_host.cmd52Read(0, CCCR_IO_READY, &rdy);
        if (s != SdioHost::OK) return s;
        if (rdy & (1u << fn)) return SdioHost::OK;
        delay(1);
    }
    return SdioHost::CMD_TIMEOUT;
}

SdioHost::Status SdioFunc::setBlockSize(uint8_t fn, uint16_t bytes) {
    // Function 0's block size lives in the CCCR at 0x10/0x11; every other
    // function has it in its FBR at 0x*10/0x*11 (function 1 -> 0x110/0x111).
    uint32_t lo = (fn == 0) ? CCCR_FN0_BLKSZ0 : (((uint32_t)fn << 8) | 0x10u);
    SdioHost::Status s = m_host.cmd52Write(0, lo, (uint8_t)(bytes & 0xFF));
    if (s != SdioHost::OK) return s;
    return m_host.cmd52Write(0, lo + 1, (uint8_t)(bytes >> 8));
}
