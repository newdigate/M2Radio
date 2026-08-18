#include "Iw416.h"
#include <string.h>

SdioHost::Status Iw416::readFwStatus(uint16_t *out) {
    uint8_t lo = 0, hi = 0;
    SdioHost::Status s = m_host.cmd52Read(1, CARD_FW_STATUS0, &lo);
    if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, CARD_FW_STATUS1, &hi);
    if (s != SdioHost::OK) return s;
    m_fwStatus = (uint16_t)(lo | ((uint16_t)hi << 8));
    if (out) *out = m_fwStatus;
    return SdioHost::OK;
}

SdioHost::Status Iw416::readCardStatus(uint8_t *out) {
    SdioHost::Status s = m_host.cmd52Read(1, CARD_STATUS_REG, &m_cardStatus);
    if (s != SdioHost::OK) return s;
    if (out) *out = m_cardStatus;
    return SdioHost::OK;
}

// How many bytes the bootloader wants next.  A non-zero value is the proof
// that the card is in download mode and answering us.
SdioHost::Status Iw416::readRequestedLen(uint16_t *out) {
    uint8_t lo = 0, hi = 0;
    SdioHost::Status s = m_host.cmd52Read(1, READ_BASE_0_REG, &lo);
    if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, READ_BASE_1_REG, &hi);
    if (s != SdioHost::OK) return s;
    m_requestedLen = (uint16_t)(lo | ((uint16_t)hi << 8));
    if (out) *out = m_requestedLen;
    return SdioHost::OK;
}

SdioHost::Status Iw416::begin() {
    // Function 1 is the WLAN function; the IW416 exposes exactly one.
    SdioHost::Status s = m_func.enableFunction(1);
    if (s != SdioHost::OK) return s;

    s = m_func.setBlockSize(1, SDIO_BLOCK_SIZE);
    if (s != SdioHost::OK) return s;

    // The I/O port is the card-side address CMD53 transfers are aimed at.
    uint8_t p0 = 0, p1 = 0, p2 = 0;
    s = m_host.cmd52Read(1, IO_PORT_0_REG, &p0);     if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, IO_PORT_0_REG + 1, &p1); if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, IO_PORT_0_REG + 2, &p2); if (s != SdioHost::OK) return s;
    m_ioPort = (uint32_t)p0 | ((uint32_t)p1 << 8) | ((uint32_t)p2 << 16);

    // Ack any interrupt the bootloader already raised, then unmask.
    uint8_t dummy = 0;
    (void)m_host.cmd52Read(1, HOST_INT_STATUS, &dummy);

    (void)readFwStatus(nullptr);
    (void)readCardStatus(nullptr);
    (void)readRequestedLen(nullptr);
    return SdioHost::OK;
}

SdioHost::Status Iw416::downloadFirmware(const uint8_t *fw, uint32_t len) {
    // One 256-byte staging block, zero-padded for the final short chunk.
    // NXP always writes whole blocks (calculate_sdio_write_params sets
    // buflen = SDIO_BLOCK_SIZE regardless of txlen), so we do the same.
    // The bootloader has been seen to ask for 2049 bytes, which stages as 9
    // blocks; give it real headroom rather than tracking its appetite.
    static uint8_t blockBuf[SDIO_BLOCK_SIZE * 64];
    const uint32_t maxChunk = sizeof(blockBuf);

    m_bytesSent = 0; m_chunksSent = 0; m_lastRequest = 0;
    uint32_t offset = 0, remaining = len;

    while (remaining > 0) {
        // Ask the bootloader how much it wants. It publishes zero while busy,
        // so poll rather than assuming the previous answer still holds.
        uint16_t want = 0;
        SdioHost::Status s = SdioHost::CMD_TIMEOUT;
        for (uint32_t tries = 0; tries < 1000; tries++) {
            // Gate on the card declaring itself ready before trusting the
            // length, exactly as NXP do.  Skipping this means writing into a
            // card that is not listening, which corrupts the image silently:
            // every chunk still "succeeds" and the ROM simply never validates.
            uint8_t cs = 0;
            s = readCardStatus(&cs);
            if (s != SdioHost::OK) return s;
            if ((cs & (DN_LD_CARD_RDY | CARD_IO_READY)) == (DN_LD_CARD_RDY | CARD_IO_READY)) {
                s = readRequestedLen(&want);
                if (s != SdioHost::OK) return s;
                if (want != 0) break;
            }
            delay(1);
        }
        m_lastRequest = want;
        // A zero request after we have made progress means it stopped asking:
        // the image is in. Zero on the very first pass is a real failure.
        if (want == 0) return (offset > 0) ? SdioHost::OK : SdioHost::CMD_TIMEOUT;
        if (want > maxChunk) return SdioHost::BAD_CIS;   // request we cannot stage

        uint32_t txlen = want;
        if (remaining < txlen) txlen = remaining;

        memset(blockBuf, 0, ((txlen + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE) * SDIO_BLOCK_SIZE);
        memcpy(blockBuf, fw + offset, txlen);
        uint16_t blocks = (uint16_t)((txlen + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE);

        // Fixed address: the I/O port is a FIFO, not a memory window.
        s = m_host.cmd53Write(1, m_ioPort, false, blockBuf, SDIO_BLOCK_SIZE, blocks);
        if (s != SdioHost::OK) return s;

        offset += txlen; remaining -= txlen;
        m_bytesSent = offset; m_chunksSent++;
    }

    // The bootloader jumps to the image and raises FIRMWARE_READY.  Give it
    // real time: the image is ~273 KB and the ROM verifies before starting.
    for (uint32_t i = 0; i < 4000; i++) {
        uint16_t st = 0;
        if (readFwStatus(&st) == SdioHost::OK && st == FIRMWARE_READY) return SdioHost::OK;
        delay(5);
    }
    return SdioHost::CMD_TIMEOUT;
}
