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
        OK               =  0,
        NO_IO_FUNCTION   = -1,  // card answered CMD5 but reports zero IO functions
        CMD_TIMEOUT      = -2,
        CMD_CRC          = -3,
        CLOCK_UNSTABLE   = -4,
        BAD_CIS          = -5,
        CMD5_NO_RESPONSE = -6,  // nothing answered CMD5 at all
        INIT_CLK_STUCK   = -7,  // SYS_CTRL[INITA] never self-cleared
    };

    // Reset the controller, mux the pads, send the initialisation clocks, run
    // the 400 kHz identification clock, then CMD5 -> CMD3 -> CMD7.
    //
    // CMD5_NO_RESPONSE and NO_IO_FUNCTION were one code until 2026-08-17, and
    // the first hardware run could not tell them apart -- which is the only
    // question that mattered at that moment.  They are deliberately separate:
    // "nothing is on the bus" and "something answered but has no IO function"
    // have nothing in common diagnostically.
    Status begin();

    // Run the bus at 1.8 V instead of the 3.3 V default.  Call BEFORE begin();
    // begin() resets the controller, which clears VEND_SPEC, so the switch is
    // applied there rather than here.
    //
    // On the MIMXRT1170-EVKB this drives USDHC1_VSELECT (GPIO_AD_34 ALT4) into
    // U311 (NX3L1G53GT), selecting VDD_1V8 rather than SENSOR_3V3 onto NVCC_SD.
    // That rail feeds BOTH the bus pull-ups (R373/374/375) and the MCU's own
    // NVCC_SD1 pad supply (ball D14, via R375), so the whole bus moves together.
    //
    // NXP define SDMMCHOST_OPERATION_VOLTAGE_1V8 for every IW416 module config,
    // so 1.8 V is what these cards are expected to run at.
    //
    // WARNING: do not leave this enabled with a 3.3 V-only microSD in J15 --
    // J15 and the M.2 socket J54 are the same bus.
    void useIoVoltage1V8(bool enable) { m_use1V8 = enable; }

    uint8_t  ioFunctionCount() const { return m_ioFunctions; }
    uint16_t rca()             const { return m_rca; }
    uint8_t  cccrRevision()    const { return m_cccrRev; }

    // Raw evidence from the CMD5 attempt, for when begin() fails on hardware.
    uint32_t lastIntStatus()   const { return m_lastIntStatus; }
    uint32_t lastR4()          const { return m_lastR4; }
    // PRES_STATE sampled immediately before CMD5.  Bit 23 (CLSL) is the live
    // CMD line level and bits 31:24 (DLSL) the DAT levels: with the bus idle
    // and pull-ups present these read high, so a low reading means something
    // is holding the bus down rather than merely declining to answer.
    uint32_t lastPresState()   const { return m_lastPresState; }
    // Read back after a useIoVoltage1V8(true) request: VEND_SPEC should show
    // bit 1 set and the mux register should read 4 (ALT4 = USDHC1_VSELECT).
    // Both zero means the switch never happened.
    uint32_t lastVendSpec()    const { return m_lastVendSpec; }
    uint32_t lastVselMux()     const { return m_lastVselMux; }

    // CMD53 IO_RW_EXTENDED, block mode, PIO through the USDHC data port.
    // `incrAddr` false = fixed address, which is what a card FIFO port wants.
    // No DMA: W2 is about proving the protocol, not throughput.
    Status cmd53Write(uint8_t fn, uint32_t addr, bool incrAddr,
                      const uint8_t *src, uint16_t blockSize, uint16_t blocks);
    Status cmd53Read(uint8_t fn, uint32_t addr, bool incrAddr,
                     uint8_t *dst, uint16_t blockSize, uint16_t blocks);

    // CMD52 IO_RW_DIRECT.  `fn` is the SDIO function number (0 = CCCR/CIS).
    Status cmd52Read(uint8_t fn, uint32_t addr, uint8_t *out);
    Status cmd52Write(uint8_t fn, uint32_t addr, uint8_t value);

    // Walk function 0's CIS for the CISTPL_MANFID tuple.  Both values come off
    // the wire; the driver has no built-in expectation of either.
    Status readManfId(uint16_t *manufacturer, uint16_t *card);

private:
    Status sendCommand(uint8_t index, uint32_t arg, uint32_t xferFlags, uint32_t *resp);
    Status cmd53(uint8_t fn, uint32_t addr, bool incrAddr, bool write,
                 uint8_t *buf, uint16_t blockSize, uint16_t blocks);
    Status setClock(uint32_t hz);

    uint8_t  m_ioFunctions = 0;
    uint16_t m_rca         = 0;
    uint8_t  m_cccrRev     = 0;
    uint32_t m_cisPtr      = 0;
    uint32_t m_lastIntStatus = 0;
    uint32_t m_lastR4        = 0;
    uint32_t m_lastPresState = 0;
    bool     m_use1V8        = false;
    uint32_t m_lastVendSpec  = 0;
    uint32_t m_lastVselMux   = 0;
};
