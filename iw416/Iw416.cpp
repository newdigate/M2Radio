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

    // Evidence: the firmware-status scratch BEFORE any configuration write.
    // The 2026-08-18 W3 run read 0xF00B here where every earlier run read
    // 0xFEDC, and could not say whether its own config writes had done that.
    (void)readFwStatus(&m_fwStatusPre);

    // Card-side interrupt and command-port configuration, mirroring NXP's
    // wlan_sdio_init_ioport() (wifidriver/sdio.c, SD8978 arm) step for step.
    // Every interrupt stays MASKED until the firmware is running --
    // sd_wifi_post_init() is what writes HIM_ENABLE, via enableHostInt().
    s = m_host.cmd52Write(1, HOST_INT_MASK_REG, 0x00);
    if (s != SdioHost::OK) return s;
    // CMD53 "new mode": without this the command port (ioport | CMD_PORT_SLCT)
    // is not functional at all.
    s = setCardBits(CARD_CONFIG_2_1_REG, CMD53_NEW_MODE, &m_cfgPre[0], &m_cfgPost[0]);
    if (s != SdioHost::OK) return s;
    // Publish command-reply lengths in CMD_RD_LEN_0/1.  Without this bit the
    // length register never updates and a reply can never be sized.
    s = setCardBits(CMD_CONFIG_0, CMD_PORT_RD_LEN_EN, &m_cfgPre[1], &m_cfgPost[1]);
    if (s != SdioHost::OK) return s;
    // Auto-reset the cmd-port Dnld/Upld ready bits when the CMD53 completes.
    s = setCardBits(CMD_CONFIG_1, CMD_PORT_AUTO_EN, &m_cfgPre[2], &m_cfgPost[2]);
    if (s != SdioHost::OK) return s;
    // HOST_INT_STATUS becomes clear-on-READ.  NXP never writes that register
    // anywhere -- the earlier ~mask write here was wrong on every axis.
    s = setCardBits(HOST_INT_RSR_REG, HOST_INT_RSR_MASK, &m_cfgPre[3], &m_cfgPost[3]);
    if (s != SdioHost::OK) return s;
    // Data-path Dnld/Upld ready auto-reset.
    s = setCardBits(CARD_MISC_CFG_REG, AUTO_RE_ENABLE_INT, &m_cfgPre[4], &m_cfgPost[4]);
    if (s != SdioHost::OK) return s;

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
        if (want == 0) {
            if (offset == 0) return SdioHost::CMD_TIMEOUT;
            break;                      // done -- fall through to the READY wait
        }
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

    // The bootloader jumps to the image and raises FIRMWARE_READY.  This wait
    // runs on EVERY exit path -- including "stopped asking" -- because a
    // caller that trusts OK and immediately sends a host command into a
    // still-booting firmware kills it.  The 2026-08-18 W3 run did exactly
    // that (commands ~50 ms after the last chunk) and the firmware never
    // reached READY; NXP's sdio_post_fwdnld_check_conn_ready() polls this
    // same register for up to 5 s before anything else may touch the card.
    for (uint32_t i = 0; i < 4000; i++) {
        uint16_t st = 0;
        if (readFwStatus(&st) == SdioHost::OK && st == FIRMWARE_READY) return SdioHost::OK;
        delay(5);
    }
    return SdioHost::CMD_TIMEOUT;
}

// Read-modify-write of a function-1 card register, as NXP's init does for
// every configuration register it touches.  The pre-write value and a
// read-back are kept as evidence: a register that reads the same before and
// after, or reads 0xFF both times, is a register that did not take the write.
SdioHost::Status Iw416::setCardBits(uint32_t reg, uint8_t bits,
                                    uint8_t *preOut, uint8_t *postOut) {
    uint8_t v = 0;
    SdioHost::Status s = m_host.cmd52Read(1, reg, &v);
    if (s != SdioHost::OK) return s;
    if (preOut) *preOut = v;
    s = m_host.cmd52Write(1, reg, (uint8_t)(v | bits));
    if (s != SdioHost::OK) return s;
    uint8_t rb = 0;
    s = m_host.cmd52Read(1, reg, &rb);
    if (s != SdioHost::OK) return s;
    if (postOut) *postOut = rb;
    return SdioHost::OK;
}

// Unmask the card's upload/download interrupts, including the command port's.
// NXP does this only AFTER FIRMWARE_READY (sd_wifi_post_init); during the
// bootloader phase everything stays masked.
SdioHost::Status Iw416::enableHostInt() {
    return m_host.cmd52Write(1, HOST_INT_MASK_REG, (uint8_t)HIM_ENABLE);
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

SdioHost::Status Iw416::readHostResp(uint8_t *buf, uint16_t bufLen, uint16_t *outLen,
                                     uint32_t timeoutMs) {
    // Wait for the card to flag a COMMAND-port upload.  HOST_INT_RSR was
    // configured in begin(), so HOST_INT_STATUS clears on read -- capture each
    // sample and keep the union as evidence; never write the register (NXP
    // never does, anywhere).
    uint8_t st = 0;
    bool up = false;
    for (uint32_t i = 0; i < timeoutMs; i++) {
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &st);
        if (s != SdioHost::OK) return s;
        m_intSeen |= st;
        if (st & CMD_PORT_UPLD) { up = true; break; }
        delay(1);
    }
    if (!up) return SdioHost::CMD_TIMEOUT;

    // A command-port reply publishes its length in CMD_RD_LEN_0/1 (enabled by
    // CMD_PORT_RD_LEN_EN in begin()).  RD_LEN_P0 is the DATA port's register
    // and never updates for command replies -- reading it here is the mistake
    // that made every W3 variant time out.
    uint8_t lo = 0, hi = 0;
    SdioHost::Status s = m_host.cmd52Read(1, CMD_RD_LEN_0, &lo); if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, CMD_RD_LEN_1, &hi);                  if (s != SdioHost::OK) return s;
    uint16_t len = (uint16_t)(lo | ((uint16_t)hi << 8));
    m_lastRdLen = len;
    if (len == 0 || len > bufLen) return SdioHost::BAD_CIS;

    uint16_t blocks = (uint16_t)((len + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE);
    s = m_host.cmd53Read(1, m_ioPort | CMD_PORT_SLCT, false, buf, SDIO_BLOCK_SIZE, blocks);
    if (s != SdioHost::OK) return s;
    if (outLen) *outLen = len;
    return SdioHost::OK;
}

// Read command-port packets until the response to `cmd` (its id with
// HOSTCMD_RET_BIT set) arrives, discarding events and stale replies.  The
// header of every packet seen is recorded for the probe's report.
SdioHost::Status Iw416::waitCmdResp(uint16_t cmd, uint8_t *buf, uint16_t bufLen, uint16_t *outLen,
                                    uint32_t timeoutMs) {
    for (int tries = 0; tries < 4; tries++) {
        uint16_t len = 0;
        SdioHost::Status s = readHostResp(buf, bufLen, &len, timeoutMs);
        if (s != SdioHost::OK) return s;
        if (len < 12) continue;                    // shorter than the two headers
        m_lastRespType   = (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8));
        m_lastRespCmd    = (uint16_t)(buf[4] | ((uint16_t)buf[5] << 8));
        m_lastRespResult = (uint16_t)(buf[10] | ((uint16_t)buf[11] << 8));
        if (m_lastRespType == MLAN_TYPE_CMD &&
            m_lastRespCmd == (uint16_t)(cmd | HOSTCMD_RET_BIT)) {
            if (outLen) *outLen = len;
            return SdioHost::OK;
        }
    }
    return SdioHost::BAD_CIS;   // packets kept coming, none was the answer
}

SdioHost::Status Iw416::macControl(uint32_t action) {
    // Body is a single LE u32 of action bits (HostCmd_DS_MAC_CONTROL).
    uint8_t body[4] = { (uint8_t)action, (uint8_t)(action >> 8),
                        (uint8_t)(action >> 16), (uint8_t)(action >> 24) };
    SdioHost::Status s = sendHostCmd(CMD_MAC_CONTROL, body, sizeof(body));
    if (s != SdioHost::OK) return s;
    static uint8_t rx[SDIO_BLOCK_SIZE * 2];
    return waitCmdResp(CMD_MAC_CONTROL, rx, sizeof(rx), nullptr);
}

SdioHost::Status Iw416::scan(ScanResult *out, uint8_t maxOut, uint8_t *outCount) {
    if (outCount) *outCount = 0;
    m_scanSets = 0;

    // Request: bss_mode + zero BSSID + one ChanList TLV, 2.4 GHz channels
    // 1..13, active scan, 100 ms dwell (MRVDRV_ACTIVE_SCAN_CHAN_TIME).
    const uint8_t  CHANNELS  = 13;
    const uint16_t DWELL_MS  = 100;
    uint8_t body[7 + 4 + CHANNELS * 7];
    memset(body, 0, sizeof(body));
    body[0] = BSS_MODE_ANY;                          // bssid[1..6] stay zero
    body[7] = (uint8_t)(TLV_CHANLIST & 0xFF);
    body[8] = (uint8_t)(TLV_CHANLIST >> 8);
    const uint16_t tlvLen = CHANNELS * 7;
    body[9]  = (uint8_t)(tlvLen & 0xFF);
    body[10] = (uint8_t)(tlvLen >> 8);
    for (uint8_t c = 0; c < CHANNELS; c++) {
        uint8_t *p = &body[11 + c * 7];
        p[0] = 0;                                    // radio_type: 2.4 GHz
        p[1] = (uint8_t)(c + 1);                     // channel number
        p[2] = 0;                                    // active, no flags
        p[3] = (uint8_t)(DWELL_MS & 0xFF); p[4] = (uint8_t)(DWELL_MS >> 8);
        p[5] = (uint8_t)(DWELL_MS & 0xFF); p[6] = (uint8_t)(DWELL_MS >> 8);
    }

    SdioHost::Status s = sendHostCmd(CMD_802_11_SCAN, body, sizeof(body));
    if (s != SdioHost::OK) return s;

    // The single response arrives only after every channel has been dwelt on:
    // 13 x 100 ms plus firmware overhead, so the default 2 s wait is too
    // short.  4 KB because a busy bench overflows the 1 KB command buffer.
    static uint8_t rx[4096];
    uint16_t rxLen = 0;
    s = waitCmdResp(CMD_802_11_SCAN, rx, sizeof(rx), &rxLen, 15000);
    if (s != SdioHost::OK) return s;

    // Bound parsing by the SDIOPkt's own size field -- the CMD_RD_LEN value
    // is block-padded and the tail of the last block is garbage.
    uint16_t pktSize = (uint16_t)(rx[0] | ((uint16_t)rx[1] << 8));
    if (pktSize < rxLen) rxLen = pktSize;

    // Legacy response body: u16 bss_descript_size, u8 number_of_sets, then
    // per set [u16 beacon_size][6 BSSID][1 RSSI][8 ts][2 interval][2 cap][IEs].
    const uint16_t bodyOff = INTF_HEADER_LEN + 8;
    if (rxLen < bodyOff + 3) return SdioHost::BAD_CIS;
    uint8_t sets = rx[bodyOff + 2];
    m_scanSets = sets;

    const uint8_t *p = &rx[bodyOff + 3];
    uint32_t left = (uint32_t)rxLen - (bodyOff + 3);
    uint8_t n = 0;
    for (uint8_t i = 0; i < sets && left >= 2; i++) {
        uint16_t beaconSize = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
        p += 2; left -= 2;
        if (beaconSize == 0 || beaconSize > left) break;
        const uint8_t *e = p;
        p += beaconSize; left -= beaconSize;

        const uint32_t FIXED = 6 + 1 + 8 + 2 + 2;    // BSSID RSSI ts intvl cap
        if (beaconSize < FIXED) continue;
        ScanResult r;
        memset(&r, 0, sizeof(r));
        memcpy(r.bssid, e, 6);
        r.rssi = e[6];
        // Capability is the last 2 bytes of FIXED (offset 17..18).  Bit 4
        // (Privacy) set means the network is encrypted; the RSN/WPA IEs below
        // say which scheme.
        r.capability = (uint16_t)(e[17] | ((uint16_t)e[18] << 8));
        bool hasRsn = false, hasWpa = false;
        const uint8_t *ie = e + FIXED;
        uint32_t ieLeft = beaconSize - FIXED;
        while (ieLeft >= 2) {
            uint8_t id = ie[0], ilen = ie[1];
            if ((uint32_t)ilen + 2 > ieLeft) break;
            if (id == 0) {                           // SSID
                uint8_t cpy = (ilen > 32) ? 32 : ilen;
                memcpy(r.ssid, ie + 2, cpy);
            } else if (id == 1 && ilen <= 8) {       // supported rates
                memcpy(r.rates, ie + 2, ilen);
                r.ratesLen = ilen;
            } else if (id == 50) {                   // extended rates -> append
                uint8_t room = (uint8_t)(sizeof(r.rates) - r.ratesLen);
                uint8_t cpy = (ilen > room) ? room : ilen;
                memcpy(r.rates + r.ratesLen, ie + 2, cpy);
                r.ratesLen = (uint8_t)(r.ratesLen + cpy);
            } else if (id == 3 && ilen >= 1) {       // DS Param Set: channel
                r.channel = ie[2];
            } else if (id == 48) {                   // RSN IE -> WPA2/WPA3
                hasRsn = true;
                // Keep the whole IE (id + len + body) for ASSOCIATE.
                uint16_t full = (uint16_t)(ilen + 2);
                if (full <= sizeof(r.rsnIe)) { memcpy(r.rsnIe, ie, full); r.rsnLen = (uint8_t)full; }
            } else if (id == 221 && ilen >= 4 &&     // vendor IE: WPA = 00:50:F2:01
                       ie[2] == 0x00 && ie[3] == 0x50 && ie[4] == 0xF2 && ie[5] == 0x01) {
                hasWpa = true;
            }
            ie += 2 + ilen; ieLeft -= 2 + (uint32_t)ilen;
        }
        // Classify most-secure-first: RSN wins, then WPA, then the bare
        // Privacy bit (WEP), else the network is OPEN -- a W6 candidate.
        if (hasRsn)                    r.security = SEC_WPA2;
        else if (hasWpa)               r.security = SEC_WPA;
        else if (r.capability & 0x0010) r.security = SEC_WEP;
        else                           r.security = SEC_OPEN;
        if (n < maxOut) out[n++] = r;
    }
    if (outCount) *outCount = n;
    return SdioHost::OK;
}

// ---------------------------------------------------------------------------
// W5: monitor mode + data-port RX.

SdioHost::Status Iw416::netMonitor(bool enable, uint8_t channel) {
    // HostCmd_DS_802_11_NET_MONITOR (mlan_misc.c wlan_cmd_802_11_net_monitor):
    //   u16 action, u16 monitor_activity, u16 filter_flags,
    //   ChanBand TLV [type,len,radio_type,chan_number],
    //   Filter TLV  [type,len,filter_num].
    uint8_t body[6 + 6 + 5];
    memset(body, 0, sizeof(body));
    uint16_t act = ACT_GEN_SET;
    uint16_t on  = enable ? 1 : 0;
    uint16_t flt = MON_FILTER_ALL;
    body[0] = (uint8_t)act;  body[1] = (uint8_t)(act >> 8);
    body[2] = (uint8_t)on;   body[3] = (uint8_t)(on >> 8);
    body[4] = (uint8_t)flt;  body[5] = (uint8_t)(flt >> 8);
    // ChanBand TLV: len=2, radio_type=0 (2.4 GHz), chan_number=channel.
    body[6]  = (uint8_t)(TLV_CHAN_BAND & 0xFF); body[7] = (uint8_t)(TLV_CHAN_BAND >> 8);
    body[8]  = 2; body[9] = 0;
    body[10] = 0;             // radio_type 2.4 GHz
    body[11] = channel;
    // Filter TLV: len=1, filter_num=0 (capture everything, no MAC filter).
    body[12] = (uint8_t)(TLV_MON_FILTER & 0xFF); body[13] = (uint8_t)(TLV_MON_FILTER >> 8);
    body[14] = 1; body[15] = 0;
    body[16] = 0;             // filter_num

    SdioHost::Status s = sendHostCmd(CMD_NET_MONITOR, body, sizeof(body));
    if (s != SdioHost::OK) return s;
    static uint8_t rx[SDIO_BLOCK_SIZE * 2];
    s = waitCmdResp(CMD_NET_MONITOR, rx, sizeof(rx), nullptr);
    if (s != SdioHost::OK) return s;
    // A firmware without CONFIG_NET_MONITOR answers with a non-zero result.
    if (m_lastRespResult != 0) return SdioHost::BAD_CIS;
    return SdioHost::OK;
}

SdioHost::Status Iw416::readDataPacket(uint8_t *buf, uint16_t bufLen, uint16_t *outLen,
                                       uint8_t *port, uint16_t *rxType, uint32_t timeoutMs) {
    // Wait for a DATA-port upload (UP_LD bit 0), distinct from the command
    // port's bit 6.  HOST_INT_STATUS is clear-on-READ.
    uint8_t st = 0;
    bool up = false;
    for (uint32_t i = 0; i < timeoutMs; i++) {
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &st);
        if (s != SdioHost::OK) return s;
        m_intSeen |= st;
        m_dbgStatusOr |= st;
        if (st & HOST_INT_UP_LD) { up = true; break; }
        delay(1);
    }
    if (!up) return SdioHost::CMD_TIMEOUT;
    m_dbgUploads++;

    // Which data port(s) have a packet?  Low word of RD_BITMAP is enough here.
    uint8_t bl = 0, bu = 0;
    SdioHost::Status s = m_host.cmd52Read(1, RD_BITMAP_L_REG, &bl); if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, RD_BITMAP_U_REG, &bu);                  if (s != SdioHost::OK) return s;
    uint16_t bitmap = (uint16_t)(bl | ((uint16_t)bu << 8));
    m_dbgBitmapOr |= bitmap;
    if (bitmap == 0) return SdioHost::CMD_TIMEOUT;

    // Lowest set port; its length is at RD_LEN_P0 + (port << 1).
    uint8_t p = 0;
    while (p < 16 && !((bitmap >> p) & 1u)) p++;
    if (p >= 16) return SdioHost::CMD_TIMEOUT;
    if (port) *port = p;

    uint8_t lo = 0, hi = 0;
    s = m_host.cmd52Read(1, RD_LEN_P0_L_REG + (p << 1), &lo); if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, RD_LEN_P0_U_REG + (p << 1), &hi); if (s != SdioHost::OK) return s;
    uint16_t len = (uint16_t)(lo | ((uint16_t)hi << 8));
    if (len == 0 || len > bufLen) return SdioHost::BAD_CIS;

    uint16_t blocks = (uint16_t)((len + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE);
    s = m_host.cmd53Read(1, m_ioPort | p, false, buf, SDIO_BLOCK_SIZE, blocks);
    if (s != SdioHost::OK) return s;
    m_dbgReads++;

    // Trust the SDIOPkt's own size field, not the block-padded read length.
    uint16_t pktSize = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    m_dbgLastPktType = (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8));   // SDIOPkt pkttype
    if (outLen) *outLen = pktSize;
    if (rxType) {
        // rx_pkt_type lives in the RxPD at offset 6 (after the 4-byte SDIOPkt
        // header): bss_type(1) bss_num(1) rx_pkt_length(2) rx_pkt_offset(2)
        // rx_pkt_type(2).  Only meaningful for MLAN_TYPE_DATA packets.
        *rxType = (uint16_t)(buf[INTF_HEADER_LEN + 6] |
                             ((uint16_t)buf[INTF_HEADER_LEN + 7] << 8));
    }
    return SdioHost::OK;
}

SdioHost::Status Iw416::captureMonitor(MonitorFrame *out, uint8_t maxOut, uint8_t *count,
                                       uint32_t windowMs, uint8_t channel) {
    if (count) *count = 0;
    m_framesSeen = 0;
    m_dbgUploads = 0; m_dbgReads = 0; m_dbgBitmapOr = 0; m_dbgStatusOr = 0;
    m_dbgRxTypeOr = 0; m_dbgLastPktType = 0;

    SdioHost::Status s = netMonitor(true, channel);
    if (s != SdioHost::OK) return s;

    // Stop early once plenty of frames have been seen so a busy channel does
    // not run the whole window; a quiet channel runs it out via the 200 ms
    // per-read timeouts.  Both cases are bounded.
    const uint16_t FRAME_CAP = 64;
    static uint8_t rx[2048];
    uint8_t n = 0;
    for (uint32_t elapsed = 0; elapsed < windowMs && m_framesSeen < FRAME_CAP; ) {
        uint16_t pktLen = 0, rxType = 0; uint8_t port = 0;
        // Short per-read timeout so the window is honoured even when quiet.
        SdioHost::Status rs = readDataPacket(rx, sizeof(rx), &pktLen, &port, &rxType, 200);
        if (rs == SdioHost::CMD_TIMEOUT) { elapsed += 200; continue; }
        if (rs != SdioHost::OK) { elapsed += 1; continue; }   // skip a bad read
        elapsed += 1;
        m_dbgRxTypeOr |= rxType;
        if (rxType != PKT_TYPE_802DOT11) continue;            // not a monitor frame

        // RxPD at INTF_HEADER_LEN: rx_pkt_length@2, rx_pkt_offset@4, snr@12,
        // nf@13.  The 802.11 frame is at INTF_HEADER_LEN + rx_pkt_offset.
        const uint8_t *rxpd = &rx[INTF_HEADER_LEN];
        uint16_t frameLen = (uint16_t)(rxpd[2] | ((uint16_t)rxpd[3] << 8));
        uint16_t frameOff = (uint16_t)(rxpd[4] | ((uint16_t)rxpd[5] << 8));
        int8_t snr = (int8_t)rxpd[12];
        int8_t nf  = (int8_t)rxpd[13];
        uint32_t frameStart = (uint32_t)INTF_HEADER_LEN + frameOff;
        if (frameLen < 24 || frameStart + frameLen > pktLen) continue;  // need a MAC hdr
        const uint8_t *f = &rx[frameStart];

        m_framesSeen++;
        if (n >= maxOut) continue;

        MonitorFrame mf;
        memset(&mf, 0, sizeof(mf));
        mf.frameControl = (uint16_t)(f[0] | ((uint16_t)f[1] << 8));
        mf.rssi    = (uint8_t)(nf - snr);      // dBm = snr - nf, so -rssi
        mf.channel = channel;
        mf.len     = frameLen;
        memcpy(mf.ta, &f[10], 6);              // addr2 (transmitter)
        memcpy(mf.bssid, &f[16], 6);           // addr3 (BSSID for mgmt frames)

        // Beacon (0x80) / probe response (0x50): SSID is IE 0 after the
        // 24-byte MAC header + 12-byte fixed params (ts, interval, cap).
        uint8_t subtype = (uint8_t)(mf.frameControl & 0xFC);
        if (subtype == 0x80 || subtype == 0x50) {
            uint32_t ieOff = 24 + 12;
            const uint8_t *ie = f + ieOff;
            uint32_t ieLeft = (frameLen > ieOff) ? (frameLen - ieOff) : 0;
            while (ieLeft >= 2) {
                uint8_t id = ie[0], ilen = ie[1];
                if ((uint32_t)ilen + 2 > ieLeft) break;
                if (id == 0) {                 // SSID
                    uint8_t cpy = (ilen > 32) ? 32 : ilen;
                    memcpy(mf.ssid, ie + 2, cpy);
                    break;
                }
                ie += 2 + ilen; ieLeft -= 2 + (uint32_t)ilen;
            }
        }
        out[n++] = mf;
    }

    if (count) *count = n;
    (void)netMonitor(false, channel);          // leave monitor mode
    return SdioHost::OK;
}

// ---------------------------------------------------------------------------
// W6: WPA2 association via the embedded supplicant.

SdioHost::Status Iw416::setPassphrase(const char *ssid, const char *psk) {
    // HostCmd_DS_802_11_SUPPLICANT_PMK: u16 action, u16 cache_result, then
    // TLVs.  For a passphrase (not a precomputed PMK) send SSID + Passphrase.
    uint16_t ssidLen = (uint16_t)strlen(ssid);
    uint16_t pskLen  = (uint16_t)strlen(psk);
    if (ssidLen == 0 || ssidLen > 32) return SdioHost::BAD_CIS;
    if (pskLen < 8 || pskLen > 63)    return SdioHost::BAD_CIS;   // WPA2-PSK range

    uint8_t body[4 + (4 + 32) + (4 + 63)];
    uint16_t o = 0;
    body[o++] = (uint8_t)ACT_GEN_SET; body[o++] = (uint8_t)(ACT_GEN_SET >> 8);
    body[o++] = 0; body[o++] = 0;                        // cache_result
    // SSID TLV (type 0x0000)
    body[o++] = (uint8_t)TLV_TYPE_SSID_ID; body[o++] = (uint8_t)(TLV_TYPE_SSID_ID >> 8);
    body[o++] = (uint8_t)ssidLen; body[o++] = (uint8_t)(ssidLen >> 8);
    memcpy(&body[o], ssid, ssidLen); o = (uint16_t)(o + ssidLen);
    // Passphrase TLV (type 0x013C)
    body[o++] = (uint8_t)TLV_TYPE_PASSPHRASE; body[o++] = (uint8_t)(TLV_TYPE_PASSPHRASE >> 8);
    body[o++] = (uint8_t)pskLen; body[o++] = (uint8_t)(pskLen >> 8);
    memcpy(&body[o], psk, pskLen); o = (uint16_t)(o + pskLen);

    SdioHost::Status s = sendHostCmd(CMD_SUPPLICANT_PMK, body, o);
    if (s != SdioHost::OK) return s;
    static uint8_t rx[SDIO_BLOCK_SIZE * 2];
    s = waitCmdResp(CMD_SUPPLICANT_PMK, rx, sizeof(rx), nullptr);
    if (s != SdioHost::OK) return s;
    if (m_lastRespResult != 0) return SdioHost::BAD_CIS;    // firmware rejected it
    return SdioHost::OK;
}

SdioHost::Status Iw416::getHwSpec(uint8_t mac[6], uint32_t *fwRelease, uint16_t *hwVersion) {
    static uint8_t rx[SDIO_BLOCK_SIZE * 4];
    uint16_t rxLen = 0;

    // FUNC_INIT first, as NXP's driver does.  Wait for its response (not just
    // any packet) before the next command; its content is discarded.
    SdioHost::Status s = sendHostCmd(CMD_FUNC_INIT, nullptr, 0);
    if (s != SdioHost::OK) return s;
    (void)waitCmdResp(CMD_FUNC_INIT, rx, sizeof(rx), &rxLen);

    s = sendHostCmd(CMD_GET_HW_SPEC, nullptr, 0);
    if (s != SdioHost::OK) return s;
    s = waitCmdResp(CMD_GET_HW_SPEC, rx, sizeof(rx), &rxLen);
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
