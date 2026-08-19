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
    // A fresh firmware starts both data-port rings at slot 0.
    m_txPort = 0;
    m_rxPort = 0;
    // W11: bus-attribution counters attribute a single firmware life -- a
    // mid-soak recovery re-download must not carry the previous life's
    // traffic into the new one.  (The ps* counters deliberately do NOT reset
    // here -- see their group comment in Iw416.h.)
    m_cmd52PollsTx = 0; m_cmd52PollsSvc = 0;
    m_cmd53Count = 0; m_cmd53Bytes = 0; m_cmd53ByteMode = 0;
    // W12: the sticky interrupt state and its safety net are per-firmware-life
    // too.  A pending bit describes an upload the OLD firmware offered; the new
    // image's rings start empty at slot 0 (above), so carrying it over would
    // make the first service pass hunt a packet that no longer exists -- and
    // would let a recovery re-download inherit the previous life's stranded
    // count, which is exactly the number a soak reads to decide whether the
    // lost-interrupt fault is still happening.
    m_intPending = 0; m_svcQuietPasses = 0; m_rxStrandedRecovered = 0;
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
    wakeCardIfSleeping();
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
    // seq_num is only an 8-bit sequence: mlan's HostCmd_GET_SEQ_NO() masks
    // the field with & 0x00FF, and the high byte carries bss_num/bss_type,
    // which we always leave 0.  Wrap m_seq at 8 bits so the wire value
    // (below) is unchanged by this -- txBuf[9] was already always 0 once
    // m_seq stays <= 0xFF.
    m_seq = (uint16_t)((m_seq + 1) & 0xFF);
    m_lastSentSeq = m_seq;   // waitCmdResp correlates the reply against this
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
    //
    // W12 fault #5: this loop is the one that broke the RX path.  It runs on
    // EVERY host command, once a millisecond, and it used to keep only
    // CMD_PORT_UPLD -- so a data upload's HOST_INT_UP_LD that happened to land
    // in one of these samples was read out of the clear-on-read register and
    // thrown away.  The firmware never re-raises it, so the frame was stranded
    // in the ring for good (silicon: rd_bitmap=0x20000 with m_rxPort=17 and
    // the data CMD53 count frozen).  Accumulating into m_intPending hands the
    // bit to serviceLink() instead of dropping it.
    uint8_t st = 0;
    bool up = false;
    for (uint32_t i = 0; i < timeoutMs; i++) {
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &st);
        if (s != SdioHost::OK) return s;
        m_intSeen    |= st;
        m_intPending |= st;
        if (st & CMD_PORT_UPLD) {
            // This call now owns the command-port packet and reads it below,
            // so consume that bit here (whatever a later path may observe, it
            // must not go looking for this reply a second time).  The DATA bit
            // is deliberately left pending -- draining the ring is not this
            // function's job.
            m_intPending &= (uint8_t)~CMD_PORT_UPLD;
            up = true;
            break;
        }
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
// header of every packet seen is recorded for the probe's report.  Note: a
// PS event (EVENT_PS_SLEEP/EVENT_PS_AWAKE) consumed HERE rather than in
// serviceLink()'s demux goes un-confirmed deliberately -- this loop only
// matches pkttype+cmd, it does not parse/act on events -- which is fine: per
// the fw's own contract an unconfirmed sleep just means it stays awake, and
// the next idle period re-raises EVENT_PS_SLEEP.  A PS_AWAKE consumed here
// also skips the HOST_POWER_UP clear that serviceLink's demux performs on
// that event -- if the wake-latch hypothesis (see that clear's comment)
// ever proves true on silicon, mirroring the flip/clear here is the fix
// (queued as a W10 follow-up; soak signature to watch for: psSleeps() flat
// while psHostWakes() stalls).
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
            // Correlate by seq_num, not just cmd id: seq_num sits at the same
            // offset (8:9) as in the request header (SDIOPkt's
            // [size][pkttype] followed by HostCmd's [command][size][seq_num]
            // [result]), since a response echoes the request's header shape.
            // Without this, a stale untracked 0x80E4 -- e.g. the fw's ack to
            // a sendSleepConfirm() this call never issued -- can be mistaken
            // for the answer to THIS request.  Compare only the LOW byte:
            // per mlan's HostCmd_GET_SEQ_NO() (& 0x00FF), seq_num's high byte
            // carries bss_num/bss_type, which sendHostCmd always leaves 0 --
            // masking here keeps the comparison honest even if that ever
            // changes.  If a silicon soak ever shows mismatches on every
            // call (fw not echoing seq faithfully), revert to the narrower
            // action-based exclusion instead of this check.
            uint16_t respSeq = (uint16_t)(buf[8] | ((uint16_t)buf[9] << 8));
            if ((respSeq & 0xFF) == (m_lastSentSeq & 0xFF)) {
                if (outLen) *outLen = len;
                return SdioHost::OK;
            }
            m_seqMismatches++;
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

// W12 diagnostic -- see the header comment for why this reads only the RD
// bitmap and never HOST_INT_STATUS.
uint32_t Iw416::probeRdBitmap() {
    uint32_t bm = 0;
    if (readRdBitmap32(&bm) != SdioHost::OK) return 0;
    return bm;
}

SdioHost::Status Iw416::readRdBitmap32(uint32_t *out) {
    uint8_t b0 = 0, b1 = 0, b2 = 0, b3 = 0;
    SdioHost::Status s;
    // W11: counted at the call site into m_cmd52PollsSvc -- see that
    // counter's comment in Iw416.h for why (this helper is shared beyond
    // serviceLink()).
    s = m_host.cmd52Read(1, RD_BITMAP_L_REG,  &b0); m_cmd52PollsSvc++; if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, RD_BITMAP_U_REG,  &b1); m_cmd52PollsSvc++; if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, RD_BITMAP_1L_REG, &b2); m_cmd52PollsSvc++; if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, RD_BITMAP_1U_REG, &b3); m_cmd52PollsSvc++; if (s != SdioHost::OK) return s;
    *out = (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
    return SdioHost::OK;
}

SdioHost::Status Iw416::readWrBitmap32(uint32_t *out) {
    uint8_t b0 = 0, b1 = 0, b2 = 0, b3 = 0;
    SdioHost::Status s;
    // W11: this is the CMD52 traffic sendDataFrame's wait loop pays per poll
    // -- see m_cmd52PollsTx's comment in Iw416.h.
    s = m_host.cmd52Read(1, WR_BITMAP_L_REG,  &b0); m_cmd52PollsTx++; if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, WR_BITMAP_U_REG,  &b1); m_cmd52PollsTx++; if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, WR_BITMAP_1L_REG, &b2); m_cmd52PollsTx++; if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, WR_BITMAP_1U_REG, &b3); m_cmd52PollsTx++; if (s != SdioHost::OK) return s;
    *out = (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
    return SdioHost::OK;
}

SdioHost::Status Iw416::readRingPacket(uint8_t *buf, uint16_t bufCap, uint16_t *outLen,
                                       uint8_t *portOut) {
    // The firmware uploads to ports strictly in ring order, so the only port
    // worth checking is m_rxPort.  (Verified against NXP's own stack with
    // CONFIG_WIFI_IO_DEBUG: rd_bitmap walks 0x100, 0x200, 0x400... as the
    // ESP's 1 Hz broadcasts arrive -- see transcript_hw_evkb.txt W8.)
    uint32_t bitmap = 0;
    SdioHost::Status s = readRdBitmap32(&bitmap);
    if (s != SdioHost::OK) return s;
    m_dbgBitmapOr |= bitmap;
    if (!(bitmap & (1u << m_rxPort))) {
        if (bitmap != 0) {
            // Bitmap has a packet somewhere else: the ring model was wrong or
            // the firmware restarted.  Resync (counted) rather than starve.
            uint8_t p = 0;
            while (p < MAX_DATA_PORTS && !((bitmap >> p) & 1u)) p++;
            m_rxPort = p;
            m_rxRingResyncs++;
        } else {
            return SdioHost::CMD_TIMEOUT;      // nothing pending
        }
    }
    const uint8_t p = m_rxPort;
    if (portOut) *portOut = p;

    uint8_t lo = 0, hi = 0;
    s = m_host.cmd52Read(1, RD_LEN_P0_L_REG + ((uint32_t)p << 1), &lo); m_cmd52PollsSvc++; if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, RD_LEN_P0_U_REG + ((uint32_t)p << 1), &hi); m_cmd52PollsSvc++; if (s != SdioHost::OK) return s;
    uint16_t len = (uint16_t)(lo | ((uint16_t)hi << 8));
    if (len == 0) return SdioHost::CMD_TIMEOUT;

    // Always read into scratch and consume the ring slot -- an unread port
    // stalls every later upload, so "too big" must still drain the packet.
    static uint8_t scratch[SDIO_BLOCK_SIZE * 16];
    if (len > sizeof(scratch)) return SdioHost::BAD_CIS;   // cannot drain; bug
    uint16_t blocks = (uint16_t)((len + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE);
    // W11: the RX-direction half of m_cmd53Count/m_cmd53Bytes -- see those
    // counters' comment in Iw416.h.  Counted only on a successful read, to
    // match m_dbgReads/m_rxPort's own success-only bookkeeping just below.
    s = m_host.cmd53Read(1, m_ioPort | p, false, scratch, SDIO_BLOCK_SIZE, blocks);
    if (s == SdioHost::OK) { m_cmd53Count++; m_cmd53Bytes += (uint32_t)blocks * SDIO_BLOCK_SIZE; }
    if (s != SdioHost::OK) return s;
    m_rxPort = (uint8_t)((p + 1) % MAX_DATA_PORTS);
    m_dbgReads++;

    // Trust the SDIOPkt's own size field, not the block-padded read length.
    uint16_t pktSize = (uint16_t)(scratch[0] | ((uint16_t)scratch[1] << 8));
    if (pktSize > len) pktSize = len;
    if (outLen) *outLen = pktSize;
    if (pktSize > bufCap) return SdioHost::BAD_CIS;        // read+dropped
    memcpy(buf, scratch, pktSize);
    return SdioHost::OK;
}

SdioHost::Status Iw416::readDataPacket(uint8_t *buf, uint16_t bufLen, uint16_t *outLen,
                                       uint8_t *port, uint16_t *rxType, uint32_t timeoutMs) {
    // Wait for a DATA-port upload (UP_LD bit 0), distinct from the command
    // port's bit 6.  HOST_INT_STATUS is clear-on-READ, so accumulate into
    // m_intPending (W12 fault #5): this loop discards CMD_PORT_UPLD, which is
    // the same interrupt-eating pattern in the other direction.
    // Nothing is cleared here even on success: this reads ONE ring packet, it
    // does not drain the ring, so HOST_INT_UP_LD stays pending and the next
    // serviceLink() pass finishes the job (and clears it once the ring reports
    // empty).
    uint8_t st = 0;
    bool up = false;
    for (uint32_t i = 0; i < timeoutMs; i++) {
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &st);
        if (s != SdioHost::OK) return s;
        m_intSeen     |= st;
        m_intPending  |= st;
        m_dbgStatusOr |= st;
        if (st & HOST_INT_UP_LD) { up = true; break; }
        delay(1);
    }
    if (!up) return SdioHost::CMD_TIMEOUT;
    m_dbgUploads++;

    // Read the upload at the RX ring position (W8: the ports are a 32-slot
    // ring, not a pick-lowest-bit pool).
    uint16_t pktSize = 0;
    SdioHost::Status s = readRingPacket(buf, bufLen, &pktSize, port);
    if (s != SdioHost::OK) return s;

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

SdioHost::Status Iw416::queryPmk(const char *ssid, uint8_t out32[32],
                                 bool *pmkFound, bool *pmkNonZero) {
    if (pmkFound) *pmkFound = false;
    if (pmkNonZero) *pmkNonZero = false;
    // SUPPLICANT_PMK, action GET, with just an SSID TLV -> the firmware returns
    // the cached PMK for that SSID as a PMK TLV (0x0144) if it has derived one.
    uint16_t ssidLen = (uint16_t)strlen(ssid);
    if (ssidLen == 0 || ssidLen > 32) return SdioHost::BAD_CIS;
    uint8_t body[4 + 4 + 32];
    uint16_t o = 0;
    body[o++] = (uint8_t)HostCmd_ACT_GET; body[o++] = (uint8_t)(HostCmd_ACT_GET >> 8);
    body[o++] = 0; body[o++] = 0;                        // cache_result
    body[o++] = (uint8_t)TLV_TYPE_SSID_ID; body[o++] = (uint8_t)(TLV_TYPE_SSID_ID >> 8);
    body[o++] = (uint8_t)ssidLen; body[o++] = (uint8_t)(ssidLen >> 8);
    memcpy(&body[o], ssid, ssidLen); o = (uint16_t)(o + ssidLen);

    SdioHost::Status s = sendHostCmd(CMD_SUPPLICANT_PMK, body, o);
    if (s != SdioHost::OK) return s;
    static uint8_t rx[SDIO_BLOCK_SIZE * 2];
    uint16_t rxLen = 0;
    s = waitCmdResp(CMD_SUPPLICANT_PMK, rx, sizeof(rx), &rxLen);
    if (s != SdioHost::OK) return s;

    // Response body (after 4 SDIO + 8 HostCmd header): action(2) cache_result(2)
    // then TLVs.  Walk them for the PMK TLV (0x0144).
    uint16_t p = INTF_HEADER_LEN + 8 + 4;
    while (p + 4 <= rxLen) {
        uint16_t t = (uint16_t)(rx[p] | (rx[p+1] << 8));
        uint16_t l = (uint16_t)(rx[p+2] | (rx[p+3] << 8));
        if (p + 4 + l > rxLen) break;
        if (t == TLV_TYPE_PMK && l >= 32) {
            if (pmkFound) *pmkFound = true;
            memcpy(out32, &rx[p + 4], 32);
            bool nz = false;
            for (int i = 0; i < 32; i++) if (out32[i]) { nz = true; break; }
            if (pmkNonZero) *pmkNonZero = nz;
            return SdioHost::OK;
        }
        p = (uint16_t)(p + 4 + l);
    }
    return SdioHost::OK;   // no PMK TLV -> pmkFound stays false
}

// Build the ASSOCIATE-request RSN IE from the beacon RSN IE, reproducing the
// parts of NXP's wlan_update_rsn_ie() that matter for WPA2-PSK:
//   * one pairwise cipher (prefer CCMP 00-0F-AC-04 over TKIP), one AKM suite
//     (prefer PSK 00-0F-AC-02),
//   * RSN Capabilities with the PMF bits forced to the STA's posture
//     MFPC=1 / MFPR=0 (capable, not required) -- MFPC_BIT 7, MFPR_BIT 6,
//     PMF_MASK 0xC0: caps = (caps | 0xC0) & ((1<<7)|(0<<6) | ~0xC0).
// PMKID and group-mgmt-cipher tails are dropped (absent on the target APs).
// Returns the full IE length (id+len+body), or 0 on parse failure.
uint8_t Iw416::buildAssocRsnIe(const uint8_t *beacon, uint8_t beaconLen,
                               uint8_t *out, uint8_t outCap) {
    if (beaconLen < 2 + 2 + 4 + 2) return 0;         // id+len+ver+group+pwcount
    const uint8_t *b = beacon + 2;                    // skip id, len -> body
    uint8_t bodyLen = beacon[1];
    const uint8_t *end = b + bodyLen;
    const uint8_t *ver = b;                            // version (2)
    const uint8_t *group = b + 2;                      // group cipher (4)
    const uint8_t *p = b + 6;
    if (p + 2 > end) return 0;
    uint16_t pwCount = (uint16_t)(p[0] | (p[1] << 8)); p += 2;
    if (pwCount == 0 || p + pwCount * 4 > end) return 0;
    // Select pairwise: CCMP (id 4) if offered, else the first suite.
    const uint8_t *pwSel = p;
    for (uint16_t i = 0; i < pwCount; i++) if (p[i*4+3] == 4) { pwSel = p + i*4; break; }
    p += pwCount * 4;
    if (p + 2 > end) return 0;
    uint16_t akmCount = (uint16_t)(p[0] | (p[1] << 8)); p += 2;
    if (akmCount == 0 || p + akmCount * 4 > end) return 0;
    // Select AKM: PSK (id 2) if offered, else the first suite.
    const uint8_t *akmSel = p;
    for (uint16_t i = 0; i < akmCount; i++) if (p[i*4+3] == 2) { akmSel = p + i*4; break; }
    p += akmCount * 4;
    uint16_t caps = 0;
    if (p + 2 <= end) caps = (uint16_t)(p[0] | (p[1] << 8));
    // Force PMF posture MFPC=1, MFPR=0, preserving other capability bits.
    const uint16_t pmfMask = (uint16_t)((1u << 7) | (0u << 6) | (uint16_t)~0xC0u);
    caps = (uint16_t)((caps | 0xC0u) & pmfMask);

    // Emit: id(48) len ver(2) group(4) pwcount(1) pw(4) akmcount(1) akm(4) caps(2)
    uint8_t need = 2 + 2 + 4 + 2 + 4 + 2 + 4 + 2;      // = 22 (id+len+20 body)
    if (need > outCap) return 0;
    uint8_t o = 0;
    out[o++] = 48; out[o++] = 20;
    out[o++] = ver[0]; out[o++] = ver[1];
    memcpy(&out[o], group, 4); o += 4;
    out[o++] = 1; out[o++] = 0;
    memcpy(&out[o], pwSel, 4); o += 4;
    out[o++] = 1; out[o++] = 0;
    memcpy(&out[o], akmSel, 4); o += 4;
    out[o++] = (uint8_t)caps; out[o++] = (uint8_t)(caps >> 8);
    return o;
}

SdioHost::Status Iw416::deauthenticate(const uint8_t bssid[6]) {
    // HostCmd_DS_802_11_DEAUTHENTICATE: mac_addr[6] + reason_code(2).
    // Reason 3 = "STA is leaving".  Best-effort -- clears any prior
    // association the AP (or firmware) still holds before a fresh connect.
    uint8_t body[8];
    memcpy(body, bssid, 6);
    body[6] = 3; body[7] = 0;
    SdioHost::Status s = sendHostCmd(CMD_DEAUTHENTICATE, body, sizeof(body));
    if (s != SdioHost::OK) return s;
    static uint8_t rx[SDIO_BLOCK_SIZE * 2];
    return waitCmdResp(CMD_DEAUTHENTICATE, rx, sizeof(rx), nullptr, 3000);
}

SdioHost::Status Iw416::associate(const ScanResult &ap) {
    m_assocStatus = 0xFFFF;
    m_assocCapInfo = 0;
    // Clear any stale association first (NXP's connect flow does this).  Tested
    // whether this clears the SSID-keyed PMK cache and breaks the handshake:
    // it does NOT -- removing it left the handshake failing identically -- so
    // it is kept for its original purpose (a card associated in a prior boot
    // leaves the AP holding state, and a fresh association then times out).
    (void)deauthenticate(ap.bssid);
    delay(100);

    // HostCmd_DS_802_11_ASSOCIATE fixed head, then TLVs, mirroring NXP's
    // wlan_cmd_802_11_associate (mlan_join.c).
    static uint8_t body[256];
    uint16_t o = 0;
    // peer_sta_addr (BSSID)
    memcpy(&body[o], ap.bssid, 6); o += 6;
    // cap_info: the scanned capability, reserved bits masked (CAPINFO_MASK =
    // ~(bit15|bit14|bit11|bit9)).  The Privacy bit (4) is preserved -- WPA2.
    uint16_t cap = (uint16_t)(ap.capability & ~((1u<<15)|(1u<<14)|(1u<<11)|(1u<<9)));
    body[o++] = (uint8_t)cap; body[o++] = (uint8_t)(cap >> 8);
    // listen_interval, beacon_period, dtim_period
    body[o++] = 10; body[o++] = 0;        // listen_interval = 10
    body[o++] = 100; body[o++] = 0;       // beacon_period = 100 (informational)
    body[o++] = 0;                        // dtim_period

    uint16_t ssidLen = (uint16_t)strlen(ap.ssid);
    // SSID TLV (0x0000)
    body[o++] = 0x00; body[o++] = 0x00;
    body[o++] = (uint8_t)ssidLen; body[o++] = (uint8_t)(ssidLen >> 8);
    memcpy(&body[o], ap.ssid, ssidLen); o = (uint16_t)(o + ssidLen);
    // PHY DS TLV (0x0003): current channel
    body[o++] = 0x03; body[o++] = 0x00;
    body[o++] = 1; body[o++] = 0;
    body[o++] = ap.channel;
    // CF/SS param TLV (0x0004): 6 zero bytes (ignored by an infrastructure STA)
    body[o++] = 0x04; body[o++] = 0x00;
    body[o++] = 6; body[o++] = 0;
    memset(&body[o], 0, 6); o = (uint16_t)(o + 6);
    // Rates TLV (0x0001): the AP's advertised rates (a safe common subset)
    body[o++] = 0x01; body[o++] = 0x00;
    body[o++] = ap.ratesLen; body[o++] = 0;
    memcpy(&body[o], ap.rates, ap.ratesLen); o = (uint16_t)(o + ap.ratesLen);
    // Auth-type TLV (TLV_TYPE_AUTH_TYPE 0x011F): tells the firmware's assoc
    // agent which auth to run.  For WPA2-PSK, NXP's wlan_update_rsn_ie maps the
    // PSK AKM suite (00-0F-AC-02) to AssocAgentAuth_Open (0) -- WPA2-PSK uses
    // open-system 802.11 auth, then the embedded supplicant does the 4-way
    // handshake.
    if (ap.security == SEC_WPA2 || ap.security == SEC_WPA) {
        body[o++] = 0x1F; body[o++] = 0x01;      // 0x011F
        body[o++] = 2; body[o++] = 0;
        body[o++] = 0x00; body[o++] = 0x00;      // AssocAgentAuth_Open
    }
    // RSN TLV: NORMALISE the beacon RSN IE the way NXP's wlan_update_rsn_ie
    // does -- do NOT echo it verbatim.  The critical difference is the RSN
    // Capabilities PMF bits: the association request must carry the STA's own
    // PMF posture (MFPC=1, MFPR=0 -- capable, not required), not the AP's
    // advertised bits.  Echoing the AP's bits is why the 4-way handshake
    // deauthed (iPhone reason 2, Onestream reason 15).  buildAssocRsnIe also
    // reduces the pairwise/AKM lists to the single selected suite.
    if (ap.rsnLen >= 2) {
        uint8_t rsn[64];
        uint8_t rsnLen = buildAssocRsnIe(ap.rsnIe, ap.rsnLen, rsn, sizeof(rsn));
        if (rsnLen >= 2) {
            m_assocRsnLen = (rsnLen > sizeof(m_assocRsn)) ? sizeof(m_assocRsn) : rsnLen;
            memcpy(m_assocRsn, rsn, m_assocRsnLen);   // evidence for the report
            uint8_t rsnBodyLen = rsn[1];
            body[o++] = 48; body[o++] = 0;
            body[o++] = rsnBodyLen; body[o++] = 0;
            memcpy(&body[o], &rsn[2], rsnBodyLen); o = (uint16_t)(o + rsnBodyLen);
        }
    }

    SdioHost::Status s = sendHostCmd(CMD_802_11_ASSOCIATE, body, o);
    if (s != SdioHost::OK) return s;

    // The 4-way handshake happens inside this command; give it room.
    static uint8_t rx[SDIO_BLOCK_SIZE * 4];
    uint16_t rxLen = 0;
    s = waitCmdResp(CMD_802_11_ASSOCIATE, rx, sizeof(rx), &rxLen, 8000);
    if (s != SdioHost::OK) return s;

    // Response body (after 4 SDIO + 8 HostCmd header) = IEEEtypes_AssocRsp:
    // capability(2) status_code(2) a_id(2).  status_code 0 = 802.11 association
    // succeeded.  NOTE: for WPA2 this does NOT yet prove the PSK -- open-system
    // 802.11 auth + association complete before the 4-way handshake, so a wrong
    // PSK still associates here and fails the handshake afterwards.  The
    // handshake result is proven by waitForConnect() (EVENT_PORT_RELEASE).
    const uint16_t bodyOff = INTF_HEADER_LEN + 8;
    if (rxLen < bodyOff + 4) return SdioHost::BAD_CIS;
    // cap_info doubles as an error return: 0xFFFF..0xFFFB are internal-error /
    // timeout codes (e.g. 0xFFFC = timeout waiting for the AP), in which case
    // status_code is a firmware sub-code, NOT an IEEE reject.  Capture both.
    m_assocCapInfo = (uint16_t)(rx[bodyOff + 0] | ((uint16_t)rx[bodyOff + 1] << 8));
    m_assocStatus  = (uint16_t)(rx[bodyOff + 2] | ((uint16_t)rx[bodyOff + 3] << 8));
    return (m_assocStatus == 0) ? SdioHost::OK : SdioHost::BAD_CIS;
}

// Wait for the firmware to signal the WPA2 4-way handshake outcome.  Events
// arrive on the command port (pkttype MLAN_TYPE_EVENT); the event id is the
// u32 at INTF_HEADER_LEN.  EVENT_PORT_RELEASE (0x2B) = handshake done, port
// authorized -- this is the un-fakeable proof the PSK was correct (a wrong PSK
// yields a MIC-error event or a deauth, never a port release).
SdioHost::Status Iw416::waitForConnect(uint32_t timeoutMs) {
    static uint8_t rx[SDIO_BLOCK_SIZE * 4];
    m_lastEvent = 0;
    uint32_t waited = 0;
    while (waited < timeoutMs) {
        uint16_t len = 0;
        SdioHost::Status s = readHostResp(rx, sizeof(rx), &len, 500);
        if (s == SdioHost::CMD_TIMEOUT) { waited += 500; continue; }
        if (s != SdioHost::OK) return s;
        uint16_t pkttype = (uint16_t)(rx[2] | ((uint16_t)rx[3] << 8));
        if (pkttype != MLAN_TYPE_EVENT) continue;              // a cmd response
        uint32_t ev = (uint32_t)rx[INTF_HEADER_LEN] | ((uint32_t)rx[INTF_HEADER_LEN+1] << 8) |
                      ((uint32_t)rx[INTF_HEADER_LEN+2] << 16) | ((uint32_t)rx[INTF_HEADER_LEN+3] << 24);
        m_lastEvent = ev;
        // The 4 bytes after the event cause, for diagnosis.  For
        // EVENT_DEAUTHENTICATED this carries the IEEE reason code (its exact
        // offset varies by event, so keep the raw word): reason 15 = 4-way
        // handshake timeout (a wrong PSK), 2 = prior auth invalid, etc.
        if (len >= INTF_HEADER_LEN + 8)
            m_lastEventInfo = (uint32_t)rx[INTF_HEADER_LEN+4] | ((uint32_t)rx[INTF_HEADER_LEN+5] << 8) |
                              ((uint32_t)rx[INTF_HEADER_LEN+6] << 16) | ((uint32_t)rx[INTF_HEADER_LEN+7] << 24);
        if (ev == EVENT_PORT_RELEASE) return SdioHost::OK;
        if (ev == EVENT_MIC_ERR_UNICAST || ev == EVENT_MIC_ERR_MULTICAST)
            return SdioHost::CMD_CRC;                          // handshake failed
        if (ev == EVENT_DEAUTHENTICATED || ev == EVENT_DISASSOCIATED)
            return SdioHost::CMD_CRC;                          // kicked off -- handshake failed
        // other events (e.g. link stats) -- keep waiting
    }
    return SdioHost::CMD_TIMEOUT;
}

// Diagnostic: after association, watch BOTH ports so we can tell an EMBEDDED
// supplicant (firmware runs the 4-way handshake; EAPOL never reaches us) from a
// HOST supplicant (firmware forwards EAPOL-Key frames, ethertype 0x888E, up the
// data port for us to answer).  HOST_INT_STATUS is clear-on-read, so we peek it
// once per iteration and service whichever bit is set -- delegating to the
// per-port readers would re-poll and lose the other bit.
SdioHost::Status Iw416::diagConnect(uint32_t timeoutMs) {
    static uint8_t rx[SDIO_BLOCK_SIZE * 4];
    m_lastEvent = 0; m_diagDataFrames = 0; m_diagEapol = false; m_diagFirstEthertype = 0;
    for (uint32_t waited = 0; waited < timeoutMs; waited++) {
        uint8_t st = 0;
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &st);
        if (s != SdioHost::OK) return s;
        m_intSeen    |= st;
        m_intPending |= st;   // W12: never discard a bit (see m_intPending)

        if (st & HOST_INT_UP_LD) {                     // data port -- maybe EAPOL
            for (;;) {
                uint16_t len = 0;
                SdioHost::Status rs = readRingPacket(rx, sizeof(rx), &len, nullptr);
                if (rs != SdioHost::OK) break;         // drained or dropped
                m_diagDataFrames++;
                // Data packet: RxPD at +4; the 802.3 frame at +4+rx_pkt_offset;
                // ethertype at frame+12 (dest[6] src[6] ethertype[2]).
                uint16_t frameOff = (uint16_t)(rx[INTF_HEADER_LEN+4] | ((uint16_t)rx[INTF_HEADER_LEN+5] << 8));
                uint32_t fs = (uint32_t)INTF_HEADER_LEN + frameOff;
                if (fs + 14 <= len) {
                    uint16_t et = (uint16_t)((rx[fs+12] << 8) | rx[fs+13]);   // big-endian
                    if (m_diagFirstEthertype == 0) m_diagFirstEthertype = et;
                    if (et == 0x888E) m_diagEapol = true;                     // EAPOL
                }
            }
            // Ring drained to exhaustion above (this loop ends only when
            // readRingPacket reports nothing left, or a read failed and
            // re-running it here would not help) -- so this path has finished
            // servicing the DATA condition and may consume its bit.
            m_intPending &= (uint8_t)~HOST_INT_UP_LD;
        }
        if (st & CMD_PORT_UPLD) {                       // command port -- event/resp
            // Consumed on ENTRY, not on success: this branch takes the one
            // look the command port's single-packet flag entitles it to, and
            // several exits below `return` from the middle of it.  A failed
            // read loses the packet whether or not the bit is held (the
            // command port has no ring to re-read it from).
            m_intPending &= (uint8_t)~CMD_PORT_UPLD;
            uint8_t lo = 0, hi = 0;
            m_host.cmd52Read(1, CMD_RD_LEN_0, &lo);
            m_host.cmd52Read(1, CMD_RD_LEN_1, &hi);
            uint16_t len = (uint16_t)(lo | ((uint16_t)hi << 8));
            if (len && len <= sizeof(rx)) {
                uint16_t blocks = (uint16_t)((len + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE);
                if (m_host.cmd53Read(1, m_ioPort | CMD_PORT_SLCT, false, rx, SDIO_BLOCK_SIZE, blocks) == SdioHost::OK) {
                    uint16_t pkttype = (uint16_t)(rx[2] | ((uint16_t)rx[3] << 8));
                    if (pkttype == MLAN_TYPE_EVENT) {
                        uint32_t ev = (uint32_t)rx[INTF_HEADER_LEN] | ((uint32_t)rx[INTF_HEADER_LEN+1] << 8) |
                                      ((uint32_t)rx[INTF_HEADER_LEN+2] << 16) | ((uint32_t)rx[INTF_HEADER_LEN+3] << 24);
                        m_lastEvent = ev;
                        if (len >= INTF_HEADER_LEN + 8)
                            m_lastEventInfo = (uint32_t)rx[INTF_HEADER_LEN+4] | ((uint32_t)rx[INTF_HEADER_LEN+5] << 8) |
                                              ((uint32_t)rx[INTF_HEADER_LEN+6] << 16) | ((uint32_t)rx[INTF_HEADER_LEN+7] << 24);
                        if (ev == EVENT_PORT_RELEASE) return SdioHost::OK;
                        if (ev == EVENT_DEAUTHENTICATED || ev == EVENT_DISASSOCIATED ||
                            ev == EVENT_MIC_ERR_UNICAST || ev == EVENT_MIC_ERR_MULTICAST)
                            return SdioHost::CMD_CRC;
                    }
                }
            }
        }
        if (!(st & (HOST_INT_UP_LD | CMD_PORT_UPLD))) delay(1);
    }
    return SdioHost::CMD_TIMEOUT;
}

// Full-window connect watcher.  Unlike diagConnect/waitForConnect, this does
// NOT return on the first event -- it drains command-port events for the whole
// window and logs each one, so a connect-then-drop is visible as a sequence:
//   [t=820ms 0x2B(port-release)] [t=1900ms 0x08(deauth) info=..02]
// which is exactly the "link went green for ~1 s then dropped" case.  It stops
// early only once it has BOTH seen a port-release AND then a deauth/disassoc
// (the drop), since there is nothing more to learn after that.
SdioHost::Status Iw416::watchConnect(uint32_t timeoutMs) {
    static uint8_t rx[SDIO_BLOCK_SIZE * 4];
    m_eventLogLen = 0; m_sawPortRelease = false; m_lastEvent = 0;
    bool sawDrop = false;
    for (uint32_t waited = 0; waited < timeoutMs; waited++) {
        uint8_t st = 0;
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &st);
        if (s != SdioHost::OK) return s;
        m_intSeen    |= st;
        m_intPending |= st;   // W12: never discard a bit (see m_intPending)
        if (st & HOST_INT_UP_LD) {                 // drain data port, ignore body
            for (;;) {
                uint16_t len = 0;
                if (readRingPacket(rx, sizeof(rx), &len, nullptr) != SdioHost::OK) break;
                m_diagDataFrames++;
            }
            m_intPending &= (uint8_t)~HOST_INT_UP_LD;   // drained -- see diagConnect
        }
        if (st & CMD_PORT_UPLD) {
            m_intPending &= (uint8_t)~CMD_PORT_UPLD;    // one look -- see diagConnect
            uint8_t lo = 0, hi = 0;
            m_host.cmd52Read(1, CMD_RD_LEN_0, &lo);
            m_host.cmd52Read(1, CMD_RD_LEN_1, &hi);
            uint16_t len = (uint16_t)(lo | ((uint16_t)hi << 8));
            if (len && len <= sizeof(rx)) {
                uint16_t blocks = (uint16_t)((len + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE);
                if (m_host.cmd53Read(1, m_ioPort | CMD_PORT_SLCT, false, rx, SDIO_BLOCK_SIZE, blocks) == SdioHost::OK) {
                    uint16_t pkttype = (uint16_t)(rx[2] | ((uint16_t)rx[3] << 8));
                    if (pkttype == MLAN_TYPE_EVENT) {
                        uint32_t ev = (uint32_t)rx[INTF_HEADER_LEN] | ((uint32_t)rx[INTF_HEADER_LEN+1] << 8) |
                                      ((uint32_t)rx[INTF_HEADER_LEN+2] << 16) | ((uint32_t)rx[INTF_HEADER_LEN+3] << 24);
                        uint32_t info = 0;
                        if (len >= INTF_HEADER_LEN + 8)
                            info = (uint32_t)rx[INTF_HEADER_LEN+4] | ((uint32_t)rx[INTF_HEADER_LEN+5] << 8) |
                                   ((uint32_t)rx[INTF_HEADER_LEN+6] << 16) | ((uint32_t)rx[INTF_HEADER_LEN+7] << 24);
                        m_lastEvent = ev; m_lastEventInfo = info;
                        if (m_eventLogLen < EVENT_LOG_CAP) {
                            m_eventLog[m_eventLogLen]  = ev;
                            m_eventLogI[m_eventLogLen] = info;
                            m_eventLogT[m_eventLogLen] = (uint16_t)waited;
                            m_eventLogLen++;
                        }
                        if (ev == EVENT_PORT_RELEASE) m_sawPortRelease = true;
                        if (ev == EVENT_DEAUTHENTICATED || ev == EVENT_DISASSOCIATED) sawDrop = true;
                        if (m_sawPortRelease && sawDrop) break;   // connect-then-drop captured
                    }
                }
            }
        }
        if (m_sawPortRelease && sawDrop) break;
        if (!(st & (HOST_INT_UP_LD | CMD_PORT_UPLD))) delay(1);
    }
    if (m_sawPortRelease) return SdioHost::OK;
    // any deauth/mic without a release => failure; nothing at all => timeout
    for (uint8_t i = 0; i < m_eventLogLen; i++) {
        uint32_t e = m_eventLog[i];
        if (e == EVENT_DEAUTHENTICATED || e == EVENT_DISASSOCIATED ||
            e == EVENT_MIC_ERR_UNICAST || e == EVENT_MIC_ERR_MULTICAST)
            return SdioHost::CMD_CRC;
    }
    return SdioHost::CMD_TIMEOUT;
}

// ---------------------------------------------------------------------------
// W7: data-path TX + connected-link service.

// TX framing (NXP wifi-sdio.c process_pkt_hdrs / wlan_xmit_pkt):
//   [u16 size][u16 pkttype=MLAN_TYPE_DATA] [TxPD, 22 bytes] [802.3 frame]
// with TxPD.tx_pkt_offset = 22 (their hardcoded 0x16) and tx_pkt_length = the
// frame length.  Every other TxPD field is zero for plain ethernet data
// (tx_pkt_type 0 = 802.3, bss_type/num 0 = the STA interface).  The write
// goes to a port whose WR_BITMAP bit the card has set; NXP round-robins
// through them, but always-lowest-set is equivalent at this traffic level.
SdioHost::Status Iw416::sendDataFrame(const uint8_t *frame, uint16_t frameLen,
                                      uint32_t timeoutMs) {
    wakeCardIfSleeping();
    static uint8_t tx[SDIO_BLOCK_SIZE * 8];
    const uint16_t total = (uint16_t)(INTF_HEADER_LEN + TXPD_LEN + frameLen);
    if (total > sizeof(tx)) return SdioHost::BAD_CIS;

    // Wait for OUR ring port to be free.  W8: the 32 download ports are a
    // ring; the firmware transmits each write promptly but only re-publishes
    // freed ports in a batch once the host has written the ring's top (NXP's
    // own comment: DN_LD "is usually when we write to port most significant
    // port").  So the port to watch is m_txPort, and picking "lowest set bit"
    // instead permanently stalls at 16 sends on a 16-bit bitmap view.
    uint32_t bitmap = 0;
    for (uint32_t waited = 0;; waited++) {
        SdioHost::Status s = readWrBitmap32(&bitmap);
        if (s != SdioHost::OK) return s;
        m_lastWrBitmap = bitmap;
        if (bitmap & (1u << m_txPort)) break;
        if (waited >= timeoutMs) return SdioHost::CMD_TIMEOUT;
        delay(1);
    }
    const uint8_t p = m_txPort;

    uint16_t blocks = (uint16_t)((total + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE);
    memset(tx, 0, (uint32_t)blocks * SDIO_BLOCK_SIZE);
    tx[0] = (uint8_t)(total & 0xFF); tx[1] = (uint8_t)(total >> 8);
    tx[2] = (uint8_t)MLAN_TYPE_DATA; tx[3] = 0;
    // TxPD: only tx_pkt_length (offset 2) and tx_pkt_offset (offset 4) are
    // non-zero.
    tx[INTF_HEADER_LEN + 2] = (uint8_t)(frameLen & 0xFF);
    tx[INTF_HEADER_LEN + 3] = (uint8_t)(frameLen >> 8);
    tx[INTF_HEADER_LEN + 4] = (uint8_t)TXPD_LEN;
    tx[INTF_HEADER_LEN + 5] = 0;
    memcpy(&tx[INTF_HEADER_LEN + TXPD_LEN], frame, frameLen);

    SdioHost::Status s = m_host.cmd53Write(1, m_ioPort | p, false, tx,
                                           SDIO_BLOCK_SIZE, blocks);
    if (s != SdioHost::OK) return s;
    // W11: the TX-direction half of m_cmd53Count/m_cmd53Bytes -- see those
    // counters' comment in Iw416.h.  Counted only on success (the early
    // `return s` above already filtered the failure case out, matching the
    // RX site's success-only bookkeeping).  m_cmd53Bytes takes the
    // block-padded length successfully issued (blocks * SDIO_BLOCK_SIZE),
    // not frameLen.
    m_cmd53Count++;
    m_cmd53Bytes += (uint32_t)blocks * SDIO_BLOCK_SIZE;
    m_txPort = (uint8_t)((p + 1) % MAX_DATA_PORTS);
    m_dataTxCount++;
    return SdioHost::OK;
}

// HostCmd_DS_TXBUF_CFG: action(2) buff_size(2) mp_end_port(2) reserved(2).
// NXP's wifi_prepare_reconfigure_tx_buf_cmd hardcodes SET + 2048 and leaves
// the rest zero.
SdioHost::Status Iw416::reconfigureTxBuffers(uint16_t bufSize) {
    uint8_t body[8];
    memset(body, 0, sizeof(body));
    body[0] = (uint8_t)ACT_GEN_SET; body[1] = (uint8_t)(ACT_GEN_SET >> 8);
    body[2] = (uint8_t)(bufSize & 0xFF); body[3] = (uint8_t)(bufSize >> 8);
    SdioHost::Status s = sendHostCmd(CMD_RECONF_TX_BUF, body, sizeof(body));
    if (s != SdioHost::OK) return s;
    static uint8_t rx[SDIO_BLOCK_SIZE * 2];
    s = waitCmdResp(CMD_RECONF_TX_BUF, rx, sizeof(rx), nullptr);
    if (s != SdioHost::OK) return s;
    if (m_lastRespResult != 0) return SdioHost::BAD_CIS;
    return SdioHost::OK;
}

// HostCmd_DS_11N_CFG: action(2) ht_tx_cap(2) ht_tx_info(2) misc_config(2).
// Values from NXP's wrapper_wlan_cmd_11n_cfg: httxcap = GREENFIELD(0x10) |
// SHORT_GI_20(0x20) | SHORT_GI_40(0x40), httxinfo = RIFS(0x08), misc =
// BAND_SELECT_BOTH(0).
SdioHost::Status Iw416::set11nCfg() {
    uint8_t body[8];
    memset(body, 0, sizeof(body));
    body[0] = (uint8_t)ACT_GEN_SET;
    body[2] = 0x70;                    // ht_tx_cap
    body[4] = 0x08;                    // ht_tx_info
    SdioHost::Status s = sendHostCmd(CMD_11N_CFG, body, sizeof(body));
    if (s != SdioHost::OK) return s;
    static uint8_t rx[SDIO_BLOCK_SIZE * 2];
    s = waitCmdResp(CMD_11N_CFG, rx, sizeof(rx), nullptr);
    if (s != SdioHost::OK) return s;
    return (m_lastRespResult == 0) ? SdioHost::OK : SdioHost::BAD_CIS;
}

// HostCmd_DS_AMSDU_AGGR_CTRL: action(2) enable(2) curr_buf_size(2).
SdioHost::Status Iw416::amsduAggrCtrl() {
    uint8_t body[6];
    memset(body, 0, sizeof(body));
    body[0] = (uint8_t)ACT_GEN_SET;
    body[2] = 0x01;                    // enable
    SdioHost::Status s = sendHostCmd(CMD_AMSDU_AGGR_CTRL, body, sizeof(body));
    if (s != SdioHost::OK) return s;
    static uint8_t rx[SDIO_BLOCK_SIZE * 2];
    s = waitCmdResp(CMD_AMSDU_AGGR_CTRL, rx, sizeof(rx), nullptr);
    if (s != SdioHost::OK) return s;
    return (m_lastRespResult == 0) ? SdioHost::OK : SdioHost::BAD_CIS;
}

SdioHost::Status Iw416::serviceLink(FrameSink sink, void *ctx, bool *dropped,
                                    uint32_t waitMs) {
    static uint8_t rx[SDIO_BLOCK_SIZE * 8];
    if (dropped) *dropped = false;
    bool gotFrame = false, gotDrop = false;
    for (uint32_t waited = 0; waited <= waitMs; waited++) {
        uint8_t fresh = 0;
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &fresh);
        m_cmd52PollsSvc++;   // W11: see m_cmd52PollsSvc's comment in Iw416.h
        if (s != SdioHost::OK) return s;
        m_intSeen    |= fresh;
        m_intPending |= fresh;
        // W12 LAYER 1: work from the union of this read and everything the
        // other four HOST_INT_STATUS readers took out of the clear-on-read
        // register without servicing.  This is what makes the `gotFrame ||
        // gotDrop` early return below safe: anything not finished this pass
        // is still set in m_intPending and is serviced on a later pass.
        uint8_t st = m_intPending;

        // W12 LAYER 2: the net.  If no upload is indicated, the card may still
        // be holding one whose interrupt was lost before Layer 1 existed -- or
        // by some future path that repeats the mistake (this bug class has bitten
        // twice).  Every RX_BITMAP_CHECK_PASSES quiet passes, ask the RD bitmap
        // directly instead of believing the interrupt: it is the same register
        // the silicon freeze dump was read from (rd_bitmap=0x20000 while
        // m_rxPort was 17 and nothing was being read).  A hit here means RX
        // would otherwise have been dead forever, so it is counted, not silent.
        bool drainNow = (st & HOST_INT_UP_LD) != 0;
        bool viaNet   = false;
        if (!drainNow) {
            if (++m_svcQuietPasses >= RX_BITMAP_CHECK_PASSES) {
                m_svcQuietPasses = 0;
                uint32_t bm = 0;
                if (readRdBitmap32(&bm) == SdioHost::OK && (bm & (1u << m_rxPort))) {
                    drainNow = true;
                    viaNet   = true;
                }
            }
        }

        if (drainNow) {
            m_svcQuietPasses = 0;
            // Drain every upload queued at the ring position (W8: the 32
            // ports are a ring; the packets sit at consecutive slots).
            bool drainedAny = false;
            for (;;) {
                uint16_t pktSize = 0;
                SdioHost::Status rs = readRingPacket(rx, sizeof(rx), &pktSize, nullptr);
                if (rs == SdioHost::CMD_TIMEOUT) break;         // ring drained
                if (rs != SdioHost::OK) break;                  // dropped/bus error
                drainedAny = true;
                uint16_t pkttype = (uint16_t)(rx[2] | ((uint16_t)rx[3] << 8));
                if (pkttype != MLAN_TYPE_DATA) continue;
                m_rxDataCount++;
                // RxPD: rx_pkt_length at +2, rx_pkt_offset at +4; the 802.3
                // frame at INTF_HEADER_LEN + rx_pkt_offset.
                const uint8_t *rxpd = &rx[INTF_HEADER_LEN];
                uint16_t plen = (uint16_t)(rxpd[2] | ((uint16_t)rxpd[3] << 8));
                uint16_t poff = (uint16_t)(rxpd[4] | ((uint16_t)rxpd[5] << 8));
                if ((uint32_t)INTF_HEADER_LEN + poff + plen > pktSize) continue;
                gotFrame = true;
                if (sink) sink(ctx, &rx[INTF_HEADER_LEN + poff], plen);
            }
            // Honest counter: only a drain the NET initiated, that actually
            // pulled a packet off the ring, is a recovered strand.  A bitmap
            // bit that evaporated before readRingPacket looked is not one.
            if (viaNet && drainedAny) m_rxStrandedRecovered++;
            // The DATA condition is finished: either the ring reported nothing
            // more (the normal exit) or a read failed, and re-entering this
            // loop on the very next pass could not help with that -- it would
            // only re-run the drain forever and, because the delay(1) below is
            // gated on these same bits, spin the poll at full speed instead of
            // the 1 ms pacing every caller assumes.  The net above re-detects a
            // genuinely still-pending upload within RX_BITMAP_CHECK_PASSES,
            // which is precisely why it exists.
            m_intPending &= (uint8_t)~HOST_INT_UP_LD;
        }
        if (st & CMD_PORT_UPLD) {
            // Consumed on entry: the command port's flag entitles this branch
            // to one look, and it takes it below.  (Same rule as diagConnect.)
            m_intPending &= (uint8_t)~CMD_PORT_UPLD;
            uint8_t lo = 0, hi = 0;
            m_host.cmd52Read(1, CMD_RD_LEN_0, &lo);
            m_host.cmd52Read(1, CMD_RD_LEN_1, &hi);
            m_cmd52PollsSvc += 2;   // W11: see m_cmd52PollsSvc's comment in Iw416.h
            uint16_t len = (uint16_t)(lo | ((uint16_t)hi << 8));
            if (len && len <= sizeof(rx)) {
                uint16_t blocks = (uint16_t)((len + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE);
                // NOTE (W11): this CMD53 is the command/event port, not data --
                // deliberately excluded from m_cmd53Count/m_cmd53Bytes, which
                // are data-port-only (see their comment in Iw416.h).
                if (m_host.cmd53Read(1, m_ioPort | CMD_PORT_SLCT, false, rx, SDIO_BLOCK_SIZE, blocks) == SdioHost::OK) {
                    uint16_t pkttype = (uint16_t)(rx[2] | ((uint16_t)rx[3] << 8));
                    if (pkttype == MLAN_TYPE_EVENT) {
                        uint32_t ev = (uint32_t)rx[INTF_HEADER_LEN] | ((uint32_t)rx[INTF_HEADER_LEN+1] << 8) |
                                      ((uint32_t)rx[INTF_HEADER_LEN+2] << 16) | ((uint32_t)rx[INTF_HEADER_LEN+3] << 24);
                        m_lastEvent = ev;
                        if (len >= INTF_HEADER_LEN + 8)
                            m_lastEventInfo = (uint32_t)rx[INTF_HEADER_LEN+4] | ((uint32_t)rx[INTF_HEADER_LEN+5] << 8) |
                                              ((uint32_t)rx[INTF_HEADER_LEN+6] << 16) | ((uint32_t)rx[INTF_HEADER_LEN+7] << 24);
                        if (ev == EVENT_PS_SLEEP && m_psEnabled) {
                            sendSleepConfirm();
                        } else if (ev == EVENT_PS_AWAKE) {
                            m_psState = PS_AWAKE;
                            m_psWakes++;
                            // Mirror mwifiex's pm_wakeup_card_complete: pair
                            // the HOST_POWER_UP wake write with a clear back
                            // to 0x00 once the fw itself confirms it's awake.
                            // A latched HOST_POWER_UP could pin the card
                            // awake and silently defeat the whole PS
                            // workaround; clearing here is safe (the fw just
                            // told us it IS awake) and cheap.  Best-effort --
                            // the status of this write is not load-bearing.
                            (void)m_host.cmd52Write(1, 0x00, 0x00);
                        }
                        if (ev == EVENT_DEAUTHENTICATED || ev == EVENT_DISASSOCIATED ||
                            ev == EVENT_LINK_LOST) {
                            gotDrop = true;
                            if (dropped) *dropped = true;
                            // PS engagement is per-association, so a dropped
                            // link can't still be "asleep" -- reset the state
                            // machine.  Leave m_psEnabled alone: a wake-gate
                            // write to a card that isn't actually sleeping is
                            // harmless, and connectStation() re-enables PS on
                            // reconnect anyway.
                            m_psState = PS_AWAKE;
                        }
                    }
                }
            }
        }
        if (gotFrame || gotDrop) return SdioHost::OK;
        if (!(st & (HOST_INT_UP_LD | CMD_PORT_UPLD))) delay(1);
    }
    return SdioHost::CMD_TIMEOUT;      // a quiet poll, not an error
}

// pollLink keeps its historical COPY contract for the probe (first frame
// that fits is copied out, later frames in the same pass are drained and
// counted in rxDropped()) on top of serviceLink() -- but its early-return
// contract is now slightly looser than before serviceLink() existed: the
// old pollLink only set gotFrame (and returned OK early) once a frame was
// actually COPIED to the caller, whereas serviceLink() returns OK as soon
// as any frame is SEEN, whether or not the sink below chose to copy or drop
// it.  That is harmless here -- pollLink's only caller re-polls in a loop,
// so an extra "OK, nothing new copied" return just costs one more call, and
// the drop is still recorded in rxDropped() either way.
SdioHost::Status Iw416::pollLink(uint8_t *frameBuf, uint16_t bufCap, uint16_t *frameLen,
                                 bool *dropped, uint32_t waitMs) {
    if (frameLen) *frameLen = 0;
    PollLinkCtx ctx = { frameBuf, bufCap, frameLen, false, this };
    return serviceLink(pollLinkSink, &ctx, dropped, waitMs);
}

void Iw416::pollLinkSink(void *vctx, const uint8_t *frame, uint16_t len) {
    PollLinkCtx *c = (PollLinkCtx *)vctx;
    if (!c->got && c->buf && len <= c->cap) {
        memcpy(c->buf, frame, len);
        if (c->lenOut) *c->lenOut = len;
        c->got = true;
    } else {
        c->self->m_rxDropped++;
    }
}

void Iw416::wakeCardIfSleeping() {
    if (!m_psEnabled || m_psState == PS_AWAKE) return;
    (void)m_host.cmd52Write(1, 0x00, HOST_POWER_UP);
    m_psHostWakes++;   // soak evidence: a host-initiated wake actually fired
    // The fw raises EVENT_PS_AWAKE via the normal event path; state flips
    // there (and clears HOST_POWER_UP -- see serviceLink's demux).  2 ms
    // covers the measured wake latency.  No retry happens here -- see this
    // function's header-comment for why none is needed.
    delay(2);
    m_psState = PS_AWAKE;   // optimistic: the next event will re-assert truth
}

SdioHost::Status Iw416::setIeeePs(bool enable) {
    uint8_t body[24];
    uint16_t len = 0;
    memset(body, 0, sizeof(body));
    if (enable) {
        // action, ps_bitmap, then the PS_PARAM TLV with NXP's defaults.
        body[0] = (uint8_t)PS_EN_AUTO_PS; body[1] = (uint8_t)(PS_EN_AUTO_PS >> 8);
        body[2] = (uint8_t)PS_BITMAP_STA; body[3] = (uint8_t)(PS_BITMAP_STA >> 8);
        body[4] = (uint8_t)TLV_TYPE_PS_PARAM; body[5] = (uint8_t)(TLV_TYPE_PS_PARAM >> 8);
        body[6] = 14; body[7] = 0;                        // TLV len: 7 u16 fields
        // ps_param, STRUCT order (mlan_fw.h __ps_param): null_pkt_interval=0,
        // multiple_dtims=1, bcn_miss_timeout=5, local_listen_interval=0,
        // adhoc_wake_period=0, mode=1 (fw picks PS_POLL vs NULL), delay_to_ps=1000.
        body[10] = 1;                                     // multiple_dtims
        body[12] = 5;                                     // bcn_miss_timeout
        body[18] = 1;                                     // mode = auto
        body[20] = (uint8_t)(1000 & 0xFF); body[21] = (uint8_t)(1000 >> 8);  // delay_to_ps
        len = 22;
    } else {
        body[0] = (uint8_t)PS_DIS_AUTO_PS; body[1] = (uint8_t)(PS_DIS_AUTO_PS >> 8);
        body[2] = (uint8_t)PS_BITMAP_STA; body[3] = (uint8_t)(PS_BITMAP_STA >> 8);
        len = 4;
    }
    SdioHost::Status s = sendHostCmd(CMD_PS_MODE_ENH, body, len);
    if (s != SdioHost::OK) return s;
    static uint8_t rx[SDIO_BLOCK_SIZE * 2];
    s = waitCmdResp(CMD_PS_MODE_ENH, rx, sizeof(rx), nullptr);
    if (s != SdioHost::OK) return s;
    if (m_lastRespResult != 0) return SdioHost::BAD_CIS;
    m_psEnabled = enable;
    m_psState   = PS_AWAKE;
    return SdioHost::OK;
}

// Sent in direct answer to EVENT_PS_SLEEP.  Body: action=SLEEP_CONFIRM,
// resp_ctrl=1 (RESP_NEEDED).  The fw acks with a 0x80E4/action-5 response on
// the command port and THEN sleeps; the ack flows through the normal demux
// (it is not a tracked command, so waitCmdResp is not used here).
void Iw416::sendSleepConfirm() {
    uint8_t body[4];
    body[0] = (uint8_t)PS_SLEEP_CONFIRM; body[1] = (uint8_t)(PS_SLEEP_CONFIRM >> 8);
    body[2] = 1; body[3] = 0;            // resp_ctrl = RESP_NEEDED
    if (sendHostCmd(CMD_PS_MODE_ENH, body, sizeof(body)) == SdioHost::OK) {
        m_psState = PS_SLEEPING;
        m_psSleeps++;
    } else {
        m_psConfirmFails++;   // soak evidence: the confirm write itself failed
    }
}

SdioHost::Status Iw416::connectStation(const char *ssid, const char *psk,
                                       uint8_t attempts, bool psOn) {
    static ScanResult aps[12];
    uint8_t n = 0;
    SdioHost::Status s = scan(aps, 12, &n);
    if (s != SdioHost::OK) return s;
    int idx = -1;
    for (uint8_t i = 0; i < n; i++) {
        // First match wins -- fine for the single-AP bench; a multi-BSSID
        // SSID would deterministically pick the first scan entry, not the
        // strongest.
        if (strcmp(aps[i].ssid, ssid) == 0) { idx = i; break; }
    }
    if (idx < 0) return SdioHost::BAD_CIS;         // SSID not in the scan
    // Local, not m_connectedAp: a failed connect must not clobber the last
    // successful AP (or leave a half-set one if there's never been one).
    const ScanResult *found = &aps[idx];

    if (psk && psk[0]) {
        // Use the SCANNED SSID bytes as the PBKDF2 salt (authoritative --
        // they are what the AP beacons), not the caller's spelling.
        s = setPassphrase(found->ssid, psk);
        if (s != SdioHost::OK) return s;
        // The firmware derives the PMK (PBKDF2) asynchronously after
        // SUPPLICANT_PMK; associating before it's cached races the
        // handshake.  W6 measured the derivation completing within ~300 ms;
        // 50 ms plus associate()'s own internal deauth delay has been
        // reliable on silicon.
        delay(50);
    }
    SdioHost::Status lastFail = SdioHost::CMD_CRC;
    for (uint8_t a = 0; a < attempts; a++) {
        s = associate(*found);
        if (s != SdioHost::OK) { lastFail = s; continue; }
        SdioHost::Status w = watchConnect(2500);
        // Probe rule: OK or a quiet TIMEOUT = up; CMD_CRC = rejected.
        if (w != SdioHost::CMD_CRC) {
            m_connectedAp = *found;   // success-gated: only path that sets it
            // W10: IEEE PS on by default -- the idle RX-death workaround
            // (see the PS constants block in the header).  Best-effort: does
            // not fail the connect.  On a transport failure lastRespResult()
            // is stale (from whatever command last completed, not this
            // one) -- ieeePsEnabled() is the reliable signal for whether PS
            // actually ended up enabled.
            if (psOn) (void)setIeeePs(true);
            return SdioHost::OK;
        }
        lastFail = SdioHost::CMD_CRC;
        // Per the EVENT_DEAUTHENTICATED comment above (lastEventInfo()'s low
        // 16 bits carry the IEEE reason code): reason 15 means the handshake
        // timed out, which more attempts will not fix -- stop retrying.
        if (lastEvent() == EVENT_DEAUTHENTICATED &&
            (lastEventInfo() & 0xFFFFu) == 15u) {
            break;
        }
    }
    return lastFail;
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
