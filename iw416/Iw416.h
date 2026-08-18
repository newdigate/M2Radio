// NXP IW416 (SD8978) module support over SDIO.
//
// Register numbers and the download handshake follow NXP's own driver
// (mcuxsdk middleware/wifi_nxp, BSD-3-Clause): sdio_nxp_abs/fwdnld_sdio.c and
// sdio_nxp_abs/incl/mlan_sdio_defs.h.  Nothing is vendored -- this is an
// independent implementation of the same documented sequence.
#pragma once
#include "SdioHost.h"
#include "SdioFunc.h"

class Iw416 {
public:
    Iw416(SdioHost &host, SdioFunc &func) : m_host(host), m_func(func) {}

    // --- SD8978 card control registers (function 1) ---
    // CARD_TO_HOST_EVENT: DN_LD_CARD_RDY | CARD_IO_READY.
    // 0x5C for SD8978/SD8987/SD89xx/SD9177/IW610. NOTE: 0x30 is the SD8801
    // address, and NXP's own fwdnld_sdio.c carries a stale "(0X30)" comment
    // against this read -- following that comment silently yields 0x00 forever.
    static const uint32_t CARD_STATUS_REG   = 0x5C;
    static const uint32_t HOST_INT_MASK_REG = 0x08;
    static const uint32_t HOST_INT_STATUS   = 0x0C;
    static const uint32_t IO_PORT_0_REG     = 0xE4;
    static const uint32_t CARD_FW_STATUS0   = 0xE8;
    static const uint32_t CARD_FW_STATUS1   = 0xE9;
    static const uint32_t READ_BASE_0_REG   = 0xF8;
    static const uint32_t READ_BASE_1_REG   = 0xF9;
    static const uint8_t  DN_LD_CARD_RDY    = 0x01;  // bit 0
    static const uint8_t  CARD_IO_READY     = 0x08;  // bit 3
    static const uint16_t FIRMWARE_READY    = 0xFEDC;
    static const uint16_t SDIO_BLOCK_SIZE   = 256;

    // Enable function 1, set its block size to 256, and read the I/O port.
    // This is everything needed before a firmware download.
    SdioHost::Status begin();

    uint32_t ioPort()        const { return m_ioPort; }
    uint16_t fwStatus()      const { return m_fwStatus; }
    uint8_t  cardStatus()    const { return m_cardStatus; }
    // Bytes the bootloader is asking for next. Non-zero means the card is in
    // download mode and talking to us.
    uint16_t requestedLen()  const { return m_requestedLen; }

    // Download firmware to the bootloader over CMD53.
    //
    // Protocol (NXP mcuxsdk sdio_nxp_abs/fwdnld_sdio.c, BSD-3-Clause):
    // the bootloader publishes how many bytes it wants next in READ_BASE_0/1;
    // the host writes exactly that many to the I/O port in 256-byte blocks,
    // zero-padded, and repeats until the image is consumed. A zero request
    // after progress has been made means the card is done asking.
    // Then CARD_FW_STATUS0/1 must read FIRMWARE_READY (0xFEDC).
    SdioHost::Status downloadFirmware(const uint8_t *fw, uint32_t len);

    uint32_t bytesSent()   const { return m_bytesSent; }
    uint32_t chunksSent()  const { return m_chunksSent; }
    uint16_t lastRequest() const { return m_lastRequest; }

    SdioHost::Status readFwStatus(uint16_t *out);
    SdioHost::Status readCardStatus(uint8_t *out);
    SdioHost::Status readRequestedLen(uint16_t *out);

private:
    SdioHost &m_host;
    SdioFunc &m_func;
    uint32_t m_ioPort       = 0;
    uint16_t m_fwStatus     = 0;
    uint8_t  m_cardStatus   = 0;
    uint16_t m_requestedLen = 0;
    uint32_t m_bytesSent    = 0;
    uint32_t m_chunksSent   = 0;
    uint16_t m_lastRequest  = 0;
};
