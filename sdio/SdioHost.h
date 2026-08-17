// Minimal SDIO host for the i.MX RT1176 uSDHC1 controller.
//
// Deliberately NOT built on SdFat: SdioTeensy is an SD-memory driver
// (CMD0/CMD8/ACMD41/CMD2) with no SDIO path at all.  What IS reused -- by
// copying, not by coupling -- is its RT1176 clock/pad/reset sequence, which is
// already proven on this silicon.
//
// WARNING: on the MIMXRT1170-EVKB, uSDHC1 carries BOTH the M.2 socket J54 and
// the microSD slot J15, wired in parallel.  Only one card may be present.
#pragma once
#include <Arduino.h>

class SdioHost {
public:
    enum Status : int8_t {
        OK             =  0,
        NO_IO_FUNCTION = -1,  // CMD5 got no response, or the card reports 0 functions
        CMD_TIMEOUT    = -2,
        CMD_CRC        = -3,
        CLOCK_UNSTABLE = -4,
        BAD_CIS        = -5,
    };

    // Reset the controller, mux the pads, start the 400 kHz identification
    // clock, then CMD5 -> CMD3 -> CMD7.  Returns NO_IO_FUNCTION when no SDIO
    // card answers -- which is the expected result with an SD memory card
    // present, and in QEMU.
    Status begin();

    uint8_t  ioFunctionCount() const { return m_ioFunctions; }
    uint16_t rca()             const { return m_rca; }
    uint8_t  cccrRevision()    const { return m_cccrRev; }

    // CMD52 IO_RW_DIRECT.  `fn` is the SDIO function number (0 = CCCR/CIS).
    Status cmd52Read(uint8_t fn, uint32_t addr, uint8_t *out);
    Status cmd52Write(uint8_t fn, uint32_t addr, uint8_t value);

    // Walk function 0's CIS for the CISTPL_MANFID tuple.  Both values come off
    // the wire; the driver has no built-in expectation of either.
    Status readManfId(uint16_t *manufacturer, uint16_t *card);

private:
    Status sendCommand(uint8_t index, uint32_t arg, uint32_t xferFlags, uint32_t *resp);
    Status setClock(uint32_t hz);

    uint8_t  m_ioFunctions = 0;
    uint16_t m_rca         = 0;
    uint8_t  m_cccrRev     = 0;
    uint32_t m_cisPtr      = 0;
};
