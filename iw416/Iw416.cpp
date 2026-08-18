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
    // Unmask the interrupts the firmware uses to flag uploads. Without this the
    // status bits never set and every command response times out.
    (void)m_host.cmd52Write(1, HOST_INT_MASK_REG, (uint8_t)HIM_ENABLE);

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

SdioHost::Status Iw416::refreshIoPort() {
    uint8_t p0 = 0, p1 = 0, p2 = 0;
    SdioHost::Status s = m_host.cmd52Read(1, IO_PORT_0_REG, &p0);     if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, IO_PORT_0_REG + 1, &p1); if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, IO_PORT_0_REG + 2, &p2); if (s != SdioHost::OK) return s;
    m_ioPort = (uint32_t)p0 | ((uint32_t)p1 << 8) | ((uint32_t)p2 << 16);
    return SdioHost::OK;
}

SdioHost::Status Iw416::sendHostCmd(uint16_t cmd, const uint8_t *body, uint16_t bodyLen) {
    static uint8_t txBuf[SDIO_BLOCK_SIZE * 4];
    const uint16_t hostLen = (uint16_t)(8 + bodyLen);          // HostCmd header + body
    const uint16_t total   = (uint16_t)(INTF_HEADER_LEN + hostLen);
    if (total > sizeof(txBuf)) return SdioHost::BAD_CIS;

    memset(txBuf, 0, ((total + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE) * SDIO_BLOCK_SIZE);
    // All fields little-endian, which is also the CPU's order here.
    txBuf[0] = (uint8_t)(total & 0xFF);      txBuf[1] = (uint8_t)(total >> 8);
    txBuf[2] = (uint8_t)MLAN_TYPE_CMD;       txBuf[3] = 0;
    txBuf[4] = (uint8_t)(cmd & 0xFF);        txBuf[5] = (uint8_t)(cmd >> 8);
    txBuf[6] = (uint8_t)(hostLen & 0xFF);    txBuf[7] = (uint8_t)(hostLen >> 8);
    m_seq++;
    txBuf[8] = (uint8_t)(m_seq & 0xFF);      txBuf[9] = (uint8_t)(m_seq >> 8);
    txBuf[10] = 0; txBuf[11] = 0;                              // result
    if (body && bodyLen) memcpy(&txBuf[12], body, bodyLen);

    uint16_t blocks = (uint16_t)((total + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE);
    return m_host.cmd53Write(1, m_ioPort | CMD_PORT_SLCT, false, txBuf, SDIO_BLOCK_SIZE, blocks);
}

SdioHost::Status Iw416::readHostResp(uint8_t *buf, uint16_t bufLen, uint16_t *outLen) {
    // Wait for the card to flag an upload, then take the length it publishes.
    uint8_t st = 0;
    bool up = false;
    for (uint32_t i = 0; i < 2000; i++) {
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &st);
        if (s != SdioHost::OK) return s;
        if (st & (HOST_INT_UP_LD | CMD_PORT_UPLD)) { up = true; break; }
        delay(1);
    }
    if (!up) return SdioHost::CMD_TIMEOUT;
    // Status bits are write-1-to-clear on this card too.
    (void)m_host.cmd52Write(1, HOST_INT_STATUS,
                            (uint8_t)~(uint8_t)(HOST_INT_UP_LD | CMD_PORT_UPLD));

    uint8_t lo = 0, hi = 0;
    SdioHost::Status s = m_host.cmd52Read(1, RD_LEN_P0_L_REG, &lo); if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, RD_LEN_P0_U_REG, &hi);                  if (s != SdioHost::OK) return s;
    uint16_t len = (uint16_t)(lo | ((uint16_t)hi << 8));
    if (len == 0 || len > bufLen) return SdioHost::BAD_CIS;

    uint16_t blocks = (uint16_t)((len + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE);
    s = m_host.cmd53Read(1, m_ioPort | CMD_PORT_SLCT, false, buf, SDIO_BLOCK_SIZE, blocks);
    if (s != SdioHost::OK) return s;
    if (outLen) *outLen = len;
    return SdioHost::OK;
}

SdioHost::Status Iw416::getHwSpec(uint8_t mac[6], uint32_t *fwRelease, uint16_t *hwVersion) {
    static uint8_t rx[SDIO_BLOCK_SIZE * 4];
    uint16_t rxLen = 0;

    // FUNC_INIT first, as NXP's driver does; its reply is discarded.
    SdioHost::Status s = sendHostCmd(CMD_FUNC_INIT, nullptr, 0);
    if (s != SdioHost::OK) return s;
    (void)readHostResp(rx, sizeof(rx), &rxLen);

    s = sendHostCmd(CMD_GET_HW_SPEC, nullptr, 0);
    if (s != SdioHost::OK) return s;
    s = readHostResp(rx, sizeof(rx), &rxLen);
    if (s != SdioHost::OK) return s;

    // Layout: 4 SDIO header + 8 HostCmd header, then the body:
    //   hw_if_version(2) version(2) reserved(2) num_of_mcast_adr(2)
    //   permanent_addr[6] region_code(2) number_of_antenna(2) fw_release(4)
    const uint16_t body = INTF_HEADER_LEN + 8;
    if (rxLen < body + 22) return SdioHost::BAD_CIS;
    if (hwVersion) *hwVersion = (uint16_t)(rx[body + 2] | (rx[body + 3] << 8));
    if (mac) memcpy(mac, &rx[body + 8], 6);
    if (fwRelease) *fwRelease = (uint32_t)rx[body + 18] | ((uint32_t)rx[body + 19] << 8) |
                                ((uint32_t)rx[body + 20] << 16) | ((uint32_t)rx[body + 21] << 24);
    return SdioHost::OK;
}
