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
    m_cmd53Bytes = 0; m_cmd53ByteMode = 0;
    // W16: the register-port reads are the same kind of measurement and reset
    // with them, so busCommands() describes one firmware life throughout.
    m_cmd53RegsTx = 0; m_cmd53RegsSvc = 0;
    m_mpRegsOk = true; m_mpRegsChecked = false;
    m_mpRegsBadStatus = 0; m_mpRegsErrors = 0;
    m_cmd53Rx = 0; m_cmd53Tx = 0;
    m_rxAggrBatches = 0; m_rxAggrSlots = 0;
    m_txAggrBatches = 0; m_txAggrSlots = 0;
    // A new firmware life starts both rings at slot 0 (above), so a batch
    // staged against the OLD ring's ports must not be written into the new
    // one.  Dropping it is the only correct answer -- those frames were
    // addressed to ports that no longer mean what they meant.
    m_txAggrCount = 0; m_txAggrLen = 0; m_txAggrStart = 0;
    // W12: the sticky interrupt state and its safety net are per-firmware-life
    // too.  A pending bit describes an upload the OLD firmware offered; the new
    // image's rings start empty at slot 0 (above), so carrying it over would
    // make the first service pass hunt a packet that no longer exists -- and
    // would let a recovery re-download inherit the previous life's stranded
    // count, which is exactly the number a soak reads to decide whether the
    // lost-interrupt fault is still happening.
    m_intPending = 0; m_svcQuietPasses = 0; m_rxStrandedRecovered = 0;
    m_rxDrainErrors = 0;
    // W15: DAT1 assertions are per-firmware-life for the same reason -- the
    // number a soak reads to decide whether interrupt mode is actually
    // carrying the traffic must describe THIS firmware, not the one before the
    // recovery re-download.  m_intMode is deliberately NOT reset: it is the
    // caller's mode selection, not a measurement, and a re-download must not
    // silently drop the host back to polling.
    m_cardInts = 0;
    // W13: same rule for the two new per-firmware-life signatures -- the
    // desync-variant recovery count and the "bit set, no length published"
    // diagnostic.  Both describe THIS firmware's ring behaviour.
    m_rxDesyncRecovered = 0; m_rxSlotNotReady = 0;
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
    // W16: a staged TX batch must not sit behind a command.  waitCmdResp()
    // can block for seconds, and data frames held across that would be a
    // latency spike with no upper bound the caller can see.  Best-effort --
    // a flush failure is reported through the ordinary data-path counters,
    // and failing the COMMAND because a data write failed would be worse.
    // ★ AFTER the wake gate, not before: flushing first would push the batch
    // at a card that PS may have put to sleep, and the write's failure is
    // discarded here, so the frames would be lost silently.
    (void)flushTx();
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
    // W13 (residual #1, code-review item M5): this loop used to branch on its
    // OWN fresh sample, which is only half of Layer 1.  A CMD_PORT_UPLD that
    // another site took out of the clear-on-read register and accumulated
    // WITHOUT servicing (readDataPacket does exactly that) is recorded in
    // m_intPending and is never re-raised by the firmware -- so this loop would
    // poll out its whole timeout with the reply already flagged and sitting in
    // the command port, and the command port has no bitmap to re-derive that
    // state from.  Branch on the accumulator instead; the clear below is what
    // keeps that termination-safe (see m_intPending's rule in Iw416.h).
    //
    // Timeout semantics are unchanged: still at most `timeoutMs` iterations of
    // one CMD52 + delay(1), still CMD_TIMEOUT if the bit never appears, and
    // m_intSeen / m_lastRdLen are recorded exactly as before.  The one
    // behavioural difference is the intended one -- a bit already pending on
    // entry is honoured on the first iteration instead of being waited out.
    uint8_t st = 0;
    bool up = false;
    for (uint32_t i = 0; i < timeoutMs; i++) {
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &st);
        if (s != SdioHost::OK) return s;
        // Only the two UPLOAD bits are ever serviced by anyone; latching the
        // download-side bits would make m_intPending's "unfinished work"
        // meaning false forever (see its comment in Iw416.h).  latchIntBits()
        // is the single place that rule is applied.
        latchIntBits(st);
        if (m_intPending & CMD_PORT_UPLD) {
            // This call now owns the command-port packet and reads it below,
            // so consume that bit here (whatever a later path may observe, it
            // must not go looking for this reply a second time).  The DATA bit
            // is deliberately left pending -- draining the ring is not this
            // function's job, and this function never branches on it, so
            // consulting the accumulator above is safe for CMD_PORT_UPLD only.
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
uint32_t Iw416::probeRdBitmap(bool *ok) {
    uint32_t bm = 0;
    SdioHost::Status s = readRdBitmap32(&bm);
    // A bare return of 0 cannot distinguish "the card is offering nothing"
    // (=> the fault is card-side) from "the bus read failed" (=> the probe
    // itself is the thing that broke), and that distinction IS the reason this
    // diagnostic exists.  Report it out of band; the return value keeps its
    // old shape so existing callers are unaffected.
    if (ok) *ok = (s == SdioHost::OK);
    if (s != SdioHost::OK) return 0;
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

// W16: the multiport register-port read -- see readMpRegs()/MpRegs in Iw416.h
// for what it replaces and why it is not a cache.
//
// The address is function 1 register 0 with the address held FIXED (incrAddr
// false) and the transfer in BYTE mode, which is exactly the CMD53 NXP's
// wlan_interrupt() issues: bcnt=1 selects byte mode with count = MAX_MP_REGS,
// and flags=0 leaves OP Code at 0.  On an ordinary SDIO function a fixed
// address would re-read register 0 MP_REGS_LEN times; this card's register
// port streams the register file instead.  That is taken from NXP's driver,
// not from a capture on this board -- if silicon disagrees the snapshot is
// unmistakable (every bitmap and length identical to register 0) and the
// fallback is OP Code 1.
void Iw416::latchIntBits(uint8_t st) {
    m_intSeen |= st;
    const uint8_t serviced = (uint8_t)(st & (HOST_INT_UP_LD | CMD_PORT_UPLD));
    // Count the ARRIVAL, not the state: a bit that is already pending has
    // already been counted, and counting it again would make serviceLink
    // decline to clear a condition it really did finish servicing.
    if ((serviced & HOST_INT_UP_LD) && !(m_intPending & HOST_INT_UP_LD)) {
        m_intUpldLatches++;
    }
    m_intPending |= serviced;
}

SdioHost::Status Iw416::readMpRegs(MpRegs *out, bool txPath) {
    // A card whose register port has already been rejected never gets asked
    // again -- see mpRegsUsable() for the failure this protects against.
    if (!m_mpRegsOk) return readMpRegsCmd52(out, txPath);

    uint8_t *regs = (uint8_t *)(void *)m_mpRaw;
    SdioHost::Status s = m_host.cmd53ReadBytes(1, 0, false, regs, MP_REGS_LEN);
    if (s != SdioHost::OK) {
        // ★ THE CARD HAS ALREADY CONSUMED HOST_INT_STATUS by the time a
        // transfer can fail -- the register is clear-on-read and the bytes
        // clocked out of it before the PIO loop or the CRC check tripped.  So
        // the bits are gone whatever we do, and the only safe assumption is
        // the pessimistic one: latch BOTH serviced conditions.  A spurious
        // HOST_INT_UP_LD costs one drain that finds an empty ring and clears
        // itself; a DROPPED one is W12 fault #5, RX dead until reflash.  This
        // read's failure surface is far larger than the single CMD52 it
        // replaced (a 49-word PIO loop with its own timeouts), so the choice
        // matters more here than it ever did there.
        latchIntBits((uint8_t)(HOST_INT_UP_LD | CMD_PORT_UPLD));
        m_mpRegsErrors++;
        return s;
    }
    // Counted only on success, matching the two data-port CMD53 sites.
    if (txPath) m_cmd53RegsTx++; else m_cmd53RegsSvc++;

    // The once-per-firmware-life proof that this port streams the register
    // file at all.  CARD_STATUS is inside NXP's window and begin() already
    // read it by CMD52, so this compares the two transports against each
    // other rather than against a constant.  See mpRegsUsable().
    if (!m_mpRegsChecked) {
        m_mpRegsChecked = true;
        const uint8_t st = regs[CARD_STATUS_REG];
        if (m_cardStatus != 0 && st != m_cardStatus) {
            m_mpRegsBadStatus = st;
            m_mpRegsOk = false;
            // The bits this read consumed still have to be accounted for
            // before falling back, or the very first register-port read would
            // eat an interrupt on its way out.
            latchIntBits(regs[HOST_INT_STATUS]);
            return readMpRegsCmd52(out, txPath);
        }
    }

    out->intStatus = regs[HOST_INT_STATUS];
    out->rdBitmap  = (uint32_t)regs[RD_BITMAP_L_REG] |
                     ((uint32_t)regs[RD_BITMAP_U_REG]  << 8) |
                     ((uint32_t)regs[RD_BITMAP_1L_REG] << 16) |
                     ((uint32_t)regs[RD_BITMAP_1U_REG] << 24);
    out->wrBitmap  = (uint32_t)regs[WR_BITMAP_L_REG] |
                     ((uint32_t)regs[WR_BITMAP_U_REG]  << 8) |
                     ((uint32_t)regs[WR_BITMAP_1L_REG] << 16) |
                     ((uint32_t)regs[WR_BITMAP_1U_REG] << 24);
    out->cmdRdLen  = (uint16_t)(regs[CMD_RD_LEN_0] |
                                ((uint16_t)regs[CMD_RD_LEN_1] << 8));
    for (uint8_t p = 0; p < MAX_DATA_PORTS; p++) {
        const uint32_t off = RD_LEN_P0_L_REG + ((uint32_t)p << 1);
        out->rdLen[p] = (uint16_t)(regs[off] | ((uint16_t)regs[off + 1] << 8));
    }

    // W12 LAYER 1, in its newest home: this read took the clear-on-read
    // HOST_INT_STATUS out of the card, so its bits MUST land in the sticky
    // accumulator or they are gone for good.  Masked to the two conditions
    // anything in this driver actually services, per m_intPending's rule.
    latchIntBits(out->intStatus);
    m_dbgStatusOr |= out->intStatus;
    m_dbgBitmapOr |= out->rdBitmap;
    return SdioHost::OK;
}

SdioHost::Status Iw416::readWrBitmap32(uint32_t *out) {
    uint8_t b0 = 0, b1 = 0, b2 = 0, b3 = 0;
    SdioHost::Status s;
    // W11: this is the CMD52 traffic sendDataFrame's wait loop used to pay per
    // poll.  W16 moved it into the register-port snapshot; this remains as the
    // fallback's TX half -- see readMpRegsCmd52().
    s = m_host.cmd52Read(1, WR_BITMAP_L_REG,  &b0); m_cmd52PollsTx++; if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, WR_BITMAP_U_REG,  &b1); m_cmd52PollsTx++; if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, WR_BITMAP_1L_REG, &b2); m_cmd52PollsTx++; if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, WR_BITMAP_1U_REG, &b3); m_cmd52PollsTx++; if (s != SdioHost::OK) return s;
    *out = (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
    return SdioHost::OK;
}

// W16: the same information, one CMD52 at a time -- what this driver did
// before the register port existed.  Used only when the register port has been
// rejected (mpRegsUsable() false), which on a card that behaves as NXP's
// driver expects is never.
//
// It fills the WHOLE MpRegs, including all 32 RD_LENs, so that every caller
// sees the same struct whichever transport filled it and no call site has to
// know which one ran.  That is expensive -- 64 CMD52s for the lengths alone --
// but this path exists to keep a link ALIVE on a card that surprised us, not
// to be fast, and reading them lazily would put a transport test on the hot
// path of the case that works.
SdioHost::Status Iw416::readMpRegsCmd52(MpRegs *out, bool txPath) {
    memset(out, 0, sizeof(*out));
    SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &out->intStatus);
    if (txPath) m_cmd52PollsTx++; else m_cmd52PollsSvc++;
    if (s != SdioHost::OK) {
        // Same reasoning as the register-port path: the read may have taken
        // the bits with it, so assume the worst rather than lose them.
        latchIntBits((uint8_t)(HOST_INT_UP_LD | CMD_PORT_UPLD));
        return s;
    }
    latchIntBits(out->intStatus);
    m_dbgStatusOr |= out->intStatus;

    s = readRdBitmap32(&out->rdBitmap);
    if (s != SdioHost::OK) return s;
    m_dbgBitmapOr |= out->rdBitmap;
    s = readWrBitmap32(&out->wrBitmap);
    if (s != SdioHost::OK) return s;
    uint8_t lo = 0, hi = 0;
    s = m_host.cmd52Read(1, CMD_RD_LEN_0, &lo); m_cmd52PollsSvc++;
    if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, CMD_RD_LEN_1, &hi); m_cmd52PollsSvc++;
    if (s != SdioHost::OK) return s;
    out->cmdRdLen = (uint16_t)(lo | ((uint16_t)hi << 8));
    // Only the slots the card is actually offering: a length register for a
    // slot with no upload is 0 anyway, and 64 CMD52s to re-read zeros would
    // treble the cost of every poll on an idle link.
    for (uint8_t p = 0; p < MAX_DATA_PORTS; p++) {
        if (!((out->rdBitmap >> p) & 1u)) continue;
        s = readRdLenPort(p, &out->rdLen[p]);
        if (s != SdioHost::OK) return s;
    }
    return SdioHost::OK;
}

// W13: the per-slot RD_LEN read, factored out of readRingPacket so the safety
// net can probe a CANDIDATE slot's length without going through the ring.  See
// the declaration in Iw416.h for the register layout that makes that legal.
SdioHost::Status Iw416::readRdLenPort(uint8_t port, uint16_t *out) {
    uint8_t lo = 0, hi = 0;
    SdioHost::Status s;
    s = m_host.cmd52Read(1, RD_LEN_P0_L_REG + ((uint32_t)port << 1), &lo);
    m_cmd52PollsSvc++; if (s != SdioHost::OK) return s;
    s = m_host.cmd52Read(1, RD_LEN_P0_U_REG + ((uint32_t)port << 1), &hi);
    m_cmd52PollsSvc++; if (s != SdioHost::OK) return s;
    *out = (uint16_t)(lo | ((uint16_t)hi << 8));
    return SdioHost::OK;
}

SdioHost::Status Iw416::readRingBatch(MpRegs *regs, uint8_t *buf, uint32_t bufCap,
                                      uint32_t *outLen, uint8_t *slotsOut,
                                      uint8_t *startOut, uint16_t *lensOut,
                                      uint8_t maxSlots) {
    if (outLen)   *outLen = 0;
    if (slotsOut) *slotsOut = 0;
    if (maxSlots == 0) return SdioHost::CMD_TIMEOUT;

    // The firmware uploads to ports strictly in ring order, so the only port
    // worth checking is m_rxPort.  (Verified against NXP's own stack with
    // CONFIG_WIFI_IO_DEBUG: rd_bitmap walks 0x100, 0x200, 0x400... as the
    // ESP's 1 Hz broadcasts arrive -- see transcript_hw_evkb.txt W8.)
    uint32_t bitmap = 0;
    SdioHost::Status s;
    if (regs) {
        bitmap = regs->rdBitmap;      // W16: from this pass's register snapshot
    } else {
        s = readRdBitmap32(&bitmap);
        if (s != SdioHost::OK) return s;
        m_dbgBitmapOr |= bitmap;
    }
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

    const uint8_t start = m_rxPort;
    if (startOut) *startOut = start;
    uint32_t total = 0;
    uint8_t  slots = 0;
    while (slots < maxSlots) {
        const uint32_t p = (uint32_t)start + slots;
        if (p >= MAX_DATA_PORTS) break;        // no wrap inside one CMD53
        if (!(bitmap & (1u << p))) break;      // run of occupied slots ended
        uint16_t len = 0;
        if (regs) {
            len = regs->rdLen[p];
        } else {
            s = readRdLenPort((uint8_t)p, &len);
            if (s != SdioHost::OK) return s;
        }
        if (len == 0) {
            // W13: the card offered this slot in the bitmap but has published
            // no length for it.  A SET BIT IS NOT EVIDENCE on this firmware --
            // ~6100 such resyncs over 110 blasts found no data at all -- so
            // the run stops here rather than reading a slot the card has not
            // filled.  Counted only when it is the FIRST slot, because that is
            // the case serviceLink's drain then reads as "ring empty" and
            // clears HOST_INT_UP_LD on; a zero length that merely ends a batch
            // is an ordinary run boundary and is re-examined next pass.
            if (slots == 0) m_rxSlotNotReady++;
            break;
        }
        const uint32_t padded = ((uint32_t)len + SDIO_BLOCK_SIZE - 1) /
                                SDIO_BLOCK_SIZE * SDIO_BLOCK_SIZE;
        if (total + padded > bufCap) {
            // A single packet that cannot fit at all is a bug, not a boundary,
            // and it must be reported rather than silently skipped: skipping a
            // port wedges every later upload behind it.
            if (slots == 0) return SdioHost::BAD_CIS;
            break;
        }
        if (lensOut) lensOut[slots] = len;
        total += padded;
        slots++;
    }
    if (slots == 0) return SdioHost::CMD_TIMEOUT;

    // The aggregated multiport address, NXP's encoding (wifi_tx_data() /
    // wlan_get_rd_port()).  ONE slot keeps the plain ioport|port form -- the
    // card is not offered a run it did not need.
    const uint32_t addr = (slots == 1)
        ? (m_ioPort | start)
        : ((m_ioPort | MPA_ADDR_BASE | ((uint32_t)(slots - 1) << 8)) + start);
    const uint16_t blocks = (uint16_t)(total / SDIO_BLOCK_SIZE);
    s = m_host.cmd53Read(1, addr, false, buf, SDIO_BLOCK_SIZE, blocks);
    if (s != SdioHost::OK) return s;
    // Counted only on a successful read, matching m_dbgReads/m_rxPort's own
    // success-only bookkeeping below.  ONE CMD53 for `slots` packets is the
    // entire point: this is the counter a gate divides by the frame count.
    m_cmd53Rx++;
    m_cmd53Bytes += total;
    if (slots > 1) { m_rxAggrBatches++; m_rxAggrSlots += slots; }

    // The card has now cleared these slots' bits; keep the caller's snapshot in
    // step so a multi-batch drain advances instead of re-reading them.
    if (regs) {
        for (uint8_t i = 0; i < slots; i++) {
            regs->rdBitmap &= ~(1u << (uint32_t)(start + i));
        }
    }
    m_rxPort = (uint8_t)(((uint32_t)start + slots) % MAX_DATA_PORTS);
    m_dbgReads += slots;
    if (outLen)   *outLen = total;
    if (slotsOut) *slotsOut = slots;
    return SdioHost::OK;
}

// One packet from the ring: readRingBatch() bounded to a single slot, plus the
// historical copy-out contract (BAD_CIS = read and dropped, the slot consumed
// either way).  The diagnostic readers -- readDataPacket(), captureMonitor() --
// are its only callers; serviceLink() takes batches.
SdioHost::Status Iw416::readRingPacket(uint8_t *buf, uint16_t bufCap, uint16_t *outLen,
                                       uint8_t *portOut, MpRegs *regs) {
    // Always read into scratch and consume the ring slot -- an unread port
    // stalls every later upload, so "too big" must still drain the packet.
    static uint8_t scratch[SDIO_BLOCK_SIZE * 16];
    uint32_t got = 0;
    uint8_t  slots = 0, start = 0;
    uint16_t lens[1] = {0};
    SdioHost::Status s = readRingBatch(regs, scratch, sizeof(scratch), &got,
                                       &slots, &start, lens, 1);
    if (s != SdioHost::OK) return s;
    if (portOut) *portOut = start;

    // Trust the SDIOPkt's own size field, not the block-padded read length.
    uint16_t pktSize = (uint16_t)(scratch[0] | ((uint16_t)scratch[1] << 8));
    // Clamp to the CARD's length for this slot, not to the block-padded
    // transfer size: the padding is ours, the length is the card's.
    if (pktSize > lens[0]) pktSize = lens[0];
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
    //
    // ★ W13: and THAT is exactly why this is the one HOST_INT_STATUS reader
    // still branching on its own fresh sample rather than on m_intPending.
    // The accumulator may only be consulted for a bit the site also clears
    // (see m_intPending's rule in Iw416.h); a site that clears nothing would
    // see the bit set on every iteration for the rest of the firmware's life,
    // and captureMonitor's window -- which relies on this loop's delay(1)
    // pacing and its CMD_TIMEOUT -- would degenerate into a hot spin.
    // This site is also the reason readHostResp had to be fixed: it is the
    // one path that accumulates CMD_PORT_UPLD and never services it.
    uint8_t st = 0;
    bool up = false;
    for (uint32_t i = 0; i < timeoutMs; i++) {
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &st);
        if (s != SdioHost::OK) return s;
        latchIntBits(st);          // W12 Layer 1: serviced bits only
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
        uint8_t fresh = 0;
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &fresh);
        if (s != SdioHost::OK) return s;
        // W12: never discard a bit anyone services (see m_intPending).
        latchIntBits(fresh);
        // W13: branch on the accumulator, not on this read alone (the M5
        // shape -- see readHostResp).  Legal here for BOTH bits because both
        // branches below clear the bit they service; associate() runs
        // readHostResp immediately before this watcher, so a data upload it
        // accumulated and could not service is exactly what would otherwise be
        // sat out for the whole window.
        uint8_t st = m_intPending;

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
        uint8_t fresh = 0;
        SdioHost::Status s = m_host.cmd52Read(1, HOST_INT_STATUS, &fresh);
        if (s != SdioHost::OK) return s;
        // W12: never discard a bit anyone services (see m_intPending).
        latchIntBits(fresh);
        // W13: branch on the accumulator -- same reasoning as diagConnect, and
        // this is the watcher connectStation() actually uses, so an upload
        // accumulated by associate()'s readHostResp reaches its drain here
        // rather than waiting for the first serviceLink() pass afterwards.
        uint8_t st = m_intPending;
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
    const uint16_t total = (uint16_t)(INTF_HEADER_LEN + TXPD_LEN + frameLen);
    const uint32_t padded = ((uint32_t)total + SDIO_BLOCK_SIZE - 1) /
                            SDIO_BLOCK_SIZE * SDIO_BLOCK_SIZE;
    if (padded > sizeof(m_txAggrBuf)) return SdioHost::BAD_CIS;
    // W16: this frame cannot join the batch in progress if the batch is full,
    // the buffer has no room, or the ring has come back to slot 0 -- a run
    // never wraps inside one CMD53 (see readRingBatch's comment).  Flush and
    // start a new one.
    if (m_txAggrCount &&
        (m_txAggrCount >= AGGR_PKT_LIMIT ||
         m_txAggrLen + padded > sizeof(m_txAggrBuf) ||
         m_txPort == 0)) {
        SdioHost::Status fs = flushTx();
        if (fs != SdioHost::OK) return fs;
    }

    // Wait for OUR ring port to be free.  W8: the 32 download ports are a
    // ring; the firmware transmits each write promptly but only re-publishes
    // freed ports in a batch once the host has written the ring's top (NXP's
    // own comment: DN_LD "is usually when we write to port most significant
    // port").  So the port to watch is m_txPort, and picking "lowest set bit"
    // instead permanently stalls at 16 sends on a 16-bit bitmap view.
    //
    // W16: the wr-bitmap comes out of the register-port snapshot -- ONE CMD53
    // per poll instead of four CMD52s.  A side effect that is not a side
    // effect: that read also consumes the clear-on-read HOST_INT_STATUS, which
    // readWrBitmap32() never touched.  readMpRegs() accumulates those bits into
    // m_intPending, so the TX poll now LATCHES interrupts for serviceLink to
    // service rather than being blind to them -- the W12 Layer 1 rule applied
    // to a path that previously had no reason to obey it.  (It is also what
    // NXP does: their wlan_interrupt() is the only register reader either.)
    //
    // The poll itself is UNCHANGED, and deliberately so: it still re-reads the
    // real register on every iteration and still waits on OUR ring port.  W11
    // proved that this read-per-frame pattern is accidentally load-bearing --
    // it paces the host to the firmware's own credit cadence -- and that a
    // stale cached view in its place costs 2.5x.  Only the transport got
    // cheaper.
    uint32_t bitmap = 0;
    for (uint32_t waited = 0;; waited++) {
        MpRegs mp;
        SdioHost::Status s = readMpRegs(&mp, /*txPath=*/true);
        if (s != SdioHost::OK) return s;
        bitmap = mp.wrBitmap;
        m_lastWrBitmap = bitmap;
        if (bitmap & (1u << m_txPort)) break;
        // W16: never sit on staged frames while waiting for a credit.  The
        // card frees download ports in batches once the ring's top has been
        // written (W8), so holding a batch back during the wait can only make
        // the wait longer -- and it would add the whole timeout to the latency
        // of frames that were ready to go.
        if (m_txAggrCount) {
            SdioHost::Status fs = flushTx();
            if (fs != SdioHost::OK) return fs;
            continue;
        }
        if (waited >= timeoutMs) return SdioHost::CMD_TIMEOUT;
        delay(1);
    }
    const uint8_t p = m_txPort;

    // W16: stage at the aggregation buffer's current fill point.  Read AFTER
    // the two paths above that can empty it -- the "cannot join this batch"
    // flush and the wr-bitmap wait's flush -- because either moves m_txAggrLen.
    uint8_t *tx = (uint8_t *)(void *)m_txAggrBuf + m_txAggrLen;
    memset(tx, 0, padded);
    tx[0] = (uint8_t)(total & 0xFF); tx[1] = (uint8_t)(total >> 8);
    tx[2] = (uint8_t)MLAN_TYPE_DATA; tx[3] = 0;
    // TxPD: only tx_pkt_length (offset 2) and tx_pkt_offset (offset 4) are
    // non-zero.
    tx[INTF_HEADER_LEN + 2] = (uint8_t)(frameLen & 0xFF);
    tx[INTF_HEADER_LEN + 3] = (uint8_t)(frameLen >> 8);
    tx[INTF_HEADER_LEN + 4] = (uint8_t)TXPD_LEN;
    tx[INTF_HEADER_LEN + 5] = 0;
    memcpy(&tx[INTF_HEADER_LEN + TXPD_LEN], frame, frameLen);

    // W16: the frame is now STAGED, not written.  The ring port is consumed
    // here -- in order, exactly as before -- and the bus write happens in
    // flushTx(), which is called below when the batch is complete, or by the
    // caller's poll loop, or by the next send that cannot join this batch.
    if (m_txAggrCount == 0) m_txAggrStart = p;
    m_txAggrLen += padded;
    m_txAggrCount++;
    m_txPort = (uint8_t)((p + 1) % MAX_DATA_PORTS);
    // With aggregation off this flushes every frame immediately, which is
    // byte-for-byte the pre-W16 behaviour: one CMD53 per frame at
    // m_ioPort|port, m_dataTxCount incremented once, the same counters moved.
    if (!m_txAggr || m_txAggrCount >= AGGR_PKT_LIMIT || m_txPort == 0) {
        return flushTx();
    }
    return SdioHost::OK;
}

// W16.  Write the staged batch as ONE CMD53 at the aggregated multiport
// address, or at the plain ioport|port form when it holds a single frame.
//
// ★ THE RING PORTS ARE CONSUMED WHETHER OR NOT THE WRITE SUCCEEDS, and the
// batch is reset before the write for that reason.  m_txPort advanced when
// each frame was staged (that is what reserved its slot), so a failed write
// cannot be retried onto the same ports without either re-using a slot the
// card may have partly taken or rewinding a ring position the card's own
// bitmap disagrees with.  NXP has the same property -- wlan_get_wr_port_data()
// takes the port out of the bitmap before the write -- and the consequence is
// benign: the skipped slots simply stay marked free in WR_BITMAP until the
// firmware's next batch recycle.  Losing the frames is correct; wedging the
// ring would not be.
SdioHost::Status Iw416::flushTx() {
    if (m_txAggrCount == 0) return SdioHost::OK;
    const uint8_t  start = m_txAggrStart;
    const uint8_t  count = m_txAggrCount;
    const uint32_t total = m_txAggrLen;
    m_txAggrCount = 0;
    m_txAggrLen   = 0;
    m_txAggrStart = 0;

    const uint32_t addr = (count == 1)
        ? (m_ioPort | start)
        : ((m_ioPort | MPA_ADDR_BASE | ((uint32_t)(count - 1) << 8)) + start);
    const uint16_t blocks = (uint16_t)(total / SDIO_BLOCK_SIZE);
    SdioHost::Status s = m_host.cmd53Write(1, addr, false,
                                           (const uint8_t *)(void *)m_txAggrBuf,
                                           SDIO_BLOCK_SIZE, blocks);
    if (s != SdioHost::OK) return s;
    // W11: the TX-direction half of the data-CMD53 counters -- see their
    // comment in Iw416.h.  Counted only on success, matching the RX site.
    // ONE command for `count` frames is the whole point.
    m_cmd53Tx++;
    m_cmd53Bytes += total;
    if (count > 1) { m_txAggrBatches++; m_txAggrSlots += count; }
    m_dataTxCount += count;
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
        // ---- W15: is HOST_INT_STATUS worth reading on this pass? ----------
        //
        // Polled (the default), the answer is always yes and this is exactly
        // the code W14 shipped.  In INTERRUPT mode the card raises DAT1 when it
        // has work, so a quiet pass need not touch the bus at all, and three
        // things can make a pass non-quiet:
        //   * the ISR flagged a DAT1 assertion -- the whole point of the mode;
        //   * a bit is already pending from another reader (`m_intPending`
        //     non-zero) -- handled below without a read, since `st` is taken
        //     from the accumulator, not from `fresh`;
        //   * this is the pass on which the safety net runs.
        //
        // ★ THE NET DOES NOT DIE WHEN THE POLL STOPS, AND THAT WAS CHECKED,
        // NOT ASSUMED.  Layer 2 fires every RX_BITMAP_CHECK_PASSES *quiet
        // passes*; what interrupt mode removes is the CMD52 on each pass, not
        // the pass.  The loop below still runs once per delay(1), still counts
        // m_svcQuietPasses the same way, so the net keeps its ~64 ms cadence
        // and its 4 CMD52 per check unchanged.  Had the mode been implemented
        // by blocking on the interrupt instead of by skipping the read, the net
        // would have stopped firing and W13's protection would have been
        // removed in silence -- rxStrandedRecovered() still climbs ~3 per 80
        // blasts on silicon, so it is protection this firmware still needs.
        //
        // The status read is FOLDED INTO the same tick (`netTick` below): in
        // interrupt mode the net becomes a slow poll of BOTH ports rather than
        // of the ring alone.  The ring has a bitmap to interrogate; the command
        // port has nothing of the kind, so if DAT1 turned out not to work on
        // some board this is what keeps deauth/PS events arriving (at up to
        // 64 ms latency) instead of the link going quietly deaf.  Interrupt
        // mode therefore degrades to slow polling, never to nothing.
        bool intFired = false;
        if (m_intMode) {
            intFired = m_host.takeCardInt();
            if (intFired) m_cardInts++;
            // Self-heal: a pass that returned early (or bailed on a bus error)
            // left signalling masked by the ISR.  Re-arming here bounds that to
            // a single pass.  It is free when already armed -- armCardInt()
            // mirrors the enable rather than re-writing it -- and it cannot
            // lose an assertion, because DAT1 is a LEVEL: if the card is still
            // holding it, the write itself delivers the interrupt.
            else m_host.armCardInt();
        }
        bool netTick = (uint16_t)(m_svcQuietPasses + 1) >= RX_BITMAP_CHECK_PASSES;
        // W16: ONE register-port CMD53 in place of the CMD52 that used to read
        // HOST_INT_STATUS -- and it brings both ring bitmaps, all 32 per-slot
        // lengths and the command-port length with it, so the drain, the
        // safety net and the command-port branch below all read from `mp`
        // rather than issuing bus traffic of their own.  See readMpRegs().
        //
        // A pass with a bit ALREADY pending (latched by another reader, e.g.
        // sendDataFrame's own register read) also takes a snapshot even in
        // interrupt mode: it is about to service that condition and needs the
        // registers to do it.  Without that clause an interrupt-mode pass
        // could reach the command-port branch with no length to read.
        MpRegs mp;
        bool haveMp = false;
        if (!m_intMode || intFired || netTick || m_intPending) {
            SdioHost::Status s = readMpRegs(&mp);
            if (s != SdioHost::OK) return s;
            haveMp = true;
            // readMpRegs has already accumulated the serviced bits into
            // m_intPending and the raw union into m_intSeen -- same rule as
            // every other HOST_INT_STATUS reader (W12 Layer 1).
        }
        // W12 LAYER 1: work from the union of this read and everything the
        // other four HOST_INT_STATUS readers took out of the clear-on-read
        // register without servicing.  This is what makes the `gotFrame ||
        // gotDrop` early return below safe: anything not finished this pass
        // is still set in m_intPending and is serviced on a later pass.
        uint8_t st = m_intPending;
        // W16: sampled WITH `st`.  Anything that latches a NEW HOST_INT_UP_LD
        // while this pass is servicing -- above all sendDataFrame's own
        // register read, which the FrameSink contract lets the sink call from
        // inside the drain -- moves this counter, and the post-drain clear
        // below declines to run.  See latchIntBits().
        const uint32_t upldGen = m_intUpldLatches;

        // W12 LAYER 2: the net.  If no upload is indicated, the card may still
        // be holding one whose interrupt was lost before Layer 1 existed -- or
        // by some future path that repeats the mistake (this bug class has bitten
        // twice).  Every RX_BITMAP_CHECK_PASSES quiet passes, ask the RD bitmap
        // directly instead of believing the interrupt: it is the same register
        // the silicon freeze dump was read from (rd_bitmap=0x20000 while
        // m_rxPort was 17 and nothing was being read).  A hit here means RX
        // would otherwise have been dead forever, so it is counted, not silent.
        //
        // There are two variants of this fault and they need different tests:
        //   (a) interrupt lost, ring model still right -- the captured silicon
        //       freeze (rd_bitmap=0x20000 with m_rxPort==17).  `bm` carries our
        //       own slot's bit, and that bit IS evidence: the firmware set it
        //       for the upload it placed there;
        //   (b) interrupt lost AND the host's ring position no longer matches
        //       what the firmware is offering.  Then bits are set but NOT at
        //       m_rxPort, a test that looks only at our own slot never fires,
        //       and RX is dead forever while rxStrandedRecovered() still reads
        //       0 -- the net silently not working.  (b) is plausible on this
        //       card: W12's own characterization is "the trigger is repeated
        //       connections", and connectStation() does NOT re-download the
        //       firmware, so the firmware is free to restart its upload ring
        //       under a host whose m_rxPort keeps counting from wherever it was.
        //
        // ★ W13: (b) is NOT closed by widening the test to `bm != 0`.  That was
        // tried and measured WRONG on this firmware -- ~6100 ring resyncs over
        // 110 blasts that found no data at all.  A SET RD-BITMAP BIT DOES NOT
        // IMPLY A PACKET IS WAITING: stale bits linger, and entering the drain
        // on one makes readRingPacket resync m_rxPort BACKWARDS onto it, so the
        // ring walks the wrong way, repeatedly.  Never re-widen this test.
        // (b) is closed with a POSITIVE check instead: probe the candidate
        // slot's own RD_LEN -- the length registers are PER SLOT, so any slot
        // can be read without selecting it first (see readRdLenPort) -- and
        // enter the drain only if the card has actually published a length
        // there.  A stale bit with zero length is ignored, not chased.  The
        // candidate is the LOWEST set bit, which is exactly the slot
        // readRingPacket would resync to, so probe and resync cannot disagree.
        //
        // Entering the ordinary drain path then handles both: readRingPacket
        // owns the resync (counted in rxRingResyncs()) -- duplicating it here
        // would give two ring models that can disagree.
        bool drainNow  = (st & HOST_INT_UP_LD) != 0;
        bool viaNet    = false;
        bool viaDesync = false;
        if (!drainNow) {
            if (++m_svcQuietPasses >= RX_BITMAP_CHECK_PASSES) {
                // W16: the bitmap and the candidate slot's length both come out
                // of this pass's register snapshot, so the net now costs ZERO
                // extra bus commands rather than 4 CMD52s (+2 for the desync
                // probe).  `haveMp` stands in for the old "the read succeeded"
                // test: a pass that took no snapshot has nothing to check with,
                // and must not re-arm -- a failing bus must not buy itself
                // another 64 passes of silence.  netTick forces the snapshot
                // above, so in practice haveMp is true whenever this runs.
                if (haveMp) {
                    uint32_t bm = mp.rdBitmap;
                    m_svcQuietPasses = 0;
                    if (bm & (1u << m_rxPort)) {
                        drainNow = true;                   // (a) aligned
                        viaNet   = true;
                    } else if (bm != 0) {
                        // (b) candidate elsewhere -- demand positive evidence
                        // before letting the drain move the ring position.
                        uint8_t p = 0;
                        while (p < MAX_DATA_PORTS && !((bm >> p) & 1u)) p++;
                        if (p < MAX_DATA_PORTS && mp.rdLen[p] != 0) {
                            drainNow  = true;
                            viaNet    = true;
                            viaDesync = true;
                        }
                        // rdLen == 0 => stale bit: no drain, no resync, no
                        // counter, and m_rxPort does NOT move.  This is the
                        // whole point of the positive check.
                    }
                }
            }
        }

        if (drainNow) {
            m_svcQuietPasses = 0;
            // Drain every upload queued at the ring position (W8: the 32
            // ports are a ring; the packets sit at consecutive slots).
            bool drainedAny = false;
            // W16: the ring is drained in BATCHES -- one CMD53 per run of
            // consecutive occupied slots rather than one per packet.  With
            // aggregation off the bound is 1, which is byte-for-byte the
            // pre-W16 behaviour; that is what makes the gate's two arms a
            // controlled A/B on one image.
            //
            // The batch buffer is separate from `rx` (which stays the
            // command/event port's staging): frames are handed to the sink
            // straight out of it, which is still exactly the FrameSink
            // contract -- a pointer into driver-owned staging, valid only for
            // the duration of the callback.
            // uint32_t-backed: SdioHost's PIO loop casts the destination to
            // uint32_t*, and a bare uint8_t array carries no such alignment
            // guarantee (the same reason m_mpRaw and m_txAggrBuf are).
            static uint32_t batch32[(uint32_t)SDIO_BLOCK_SIZE * AGGR_BUF_BLOCKS / 4];
            uint8_t *batch = (uint8_t *)(void *)batch32;
            uint16_t lens[AGGR_PKT_LIMIT] = {0};
            for (;;) {
                uint32_t batchLen = 0;
                uint8_t  slots = 0, start = 0;
                SdioHost::Status rs = readRingBatch(haveMp ? &mp : nullptr,
                                                    batch, sizeof(batch32), &batchLen,
                                                    &slots, &start, lens,
                                                    m_rxAggr ? AGGR_PKT_LIMIT : 1);
                if (rs == SdioHost::CMD_TIMEOUT) break;         // ring drained
                if (rs != SdioHost::OK) {
                    // W12 follow-up: THE prime suspect for a residual
                    // rxStrandedRecovered() with Layer 1 in place.  This exit
                    // leaves the ring possibly non-empty and then falls into
                    // the deliberate HOST_INT_UP_LD clear below -- by design
                    // (re-running a failing read at full speed is worse), with
                    // Layer 2 as the backstop.  Counting it is what lets a soak
                    // tell "by-design strands" from a real remaining loss path
                    // -- see rxDrainErrors() in Iw416.h.
                    m_rxDrainErrors++;
                    break;                                      // dropped/bus error
                }
                drainedAny = true;
                // Split the batch the way NXP's receive loop does
                // (wifi-sdio.c): take each packet's own SDIOPkt size, round it
                // UP to a 256-byte boundary, and step by that -- the card pads
                // every slot's packet to a block boundary so the run is
                // self-describing.  A zero size ends the walk; `slots` bounds
                // it so a corrupt length cannot spin.
                //
                // ★ THE WALK STEPS BY THE CARD'S PER-SLOT RD_LEN, NOT BY EACH
                // PACKET'S OWN SIZE FIELD.  Those lengths are what sized the
                // transfer, so they are the only thing guaranteed to land on
                // the boundaries the card actually used.  Stepping by the
                // payload's self-report instead makes every packet's position
                // depend on the one before it: a single slot where the two
                // round differently would desynchronise the rest of the batch
                // and silently drop frames whose ring slots had ALREADY been
                // consumed -- unrecoverable, and invisible, because nothing
                // downstream can tell a short batch from a short burst.  A
                // packet whose size disagrees with its slot is skipped and
                // counted (rxSplitMismatch); the ones behind it still arrive.
                uint32_t off = 0;
                for (uint8_t i = 0; i < slots; i++) {
                    const uint16_t slotLen = lens[i];
                    const uint32_t padded = ((uint32_t)slotLen + SDIO_BLOCK_SIZE - 1) /
                                            SDIO_BLOCK_SIZE * SDIO_BLOCK_SIZE;
                    if (off + padded > batchLen) break;   // cannot happen: same sum
                    uint16_t pktSize = (uint16_t)(batch[off] |
                                                  ((uint16_t)batch[off + 1] << 8));
                    if (pktSize == 0 || pktSize > slotLen) {
                        m_rxSplitMismatch++;
                        off += padded;
                        continue;
                    }
                    uint16_t pkttype = (uint16_t)(batch[off + 2] |
                                                  ((uint16_t)batch[off + 3] << 8));
                    if (pkttype == MLAN_TYPE_DATA) {
                        m_rxDataCount++;
                        // RxPD: rx_pkt_length at +2, rx_pkt_offset at +4; the
                        // 802.3 frame at INTF_HEADER_LEN + rx_pkt_offset.
                        const uint8_t *rxpd = &batch[off + INTF_HEADER_LEN];
                        uint16_t plen = (uint16_t)(rxpd[2] | ((uint16_t)rxpd[3] << 8));
                        uint16_t poff = (uint16_t)(rxpd[4] | ((uint16_t)rxpd[5] << 8));
                        if ((uint32_t)INTF_HEADER_LEN + poff + plen <= pktSize) {
                            gotFrame = true;
                            if (sink) sink(ctx, &batch[off + INTF_HEADER_LEN + poff], plen);
                        }
                    }
                    off += padded;
                }
            }
            // Honest counter: only a drain the NET initiated, that actually
            // pulled a packet off the ring, is a recovered strand.  A bitmap
            // bit that evaporated before readRingPacket looked is not one.
            // W13: split by variant -- (a) the upload was at our own ring slot,
            // (b) it was elsewhere and the ring had desynced too.  They are
            // different faults with different causes, so one counter each; a
            // bench that sees only (b) is looking at a ring-model problem, not
            // at the lost-interrupt problem.
            if (viaNet && drainedAny) {
                if (viaDesync) m_rxDesyncRecovered++;
                else           m_rxStrandedRecovered++;
            }
            // The DATA condition is finished: either the ring reported nothing
            // more (the normal exit) or a read failed, and re-entering this
            // loop on the very next pass could not help with that -- it would
            // only re-run the drain forever and, because the delay(1) below is
            // gated on these same bits, spin the poll at full speed instead of
            // the 1 ms pacing every caller assumes.  The net above re-detects a
            // genuinely still-pending upload within RX_BITMAP_CHECK_PASSES,
            // which is precisely why it exists.
            //
            // Scoped to the SNAPSHOT (`st`), not to the live field: this clears
            // only the bit this pass actually acted on.  A HOST_INT_UP_LD that
            // arrived after `st` was taken is a NEW upload nobody has serviced,
            // and swallowing it here would re-create the very fault Layer 1
            // exists to prevent.  (When the drain came from the net, `st`'s bit
            // was clear and this is correctly a no-op.)
            //
            // W16: and only if no NEW upload was announced while we were
            // draining.  Scoping to the snapshot decides WHETHER to clear; it
            // cannot distinguish WHICH arrival is being cleared, and since W16
            // the TX path consumes HOST_INT_STATUS too, so a frame queued
            // mid-drain can have its bit taken by sendDataFrame and then wiped
            // here on the strength of the bit this pass came in with.  The
            // frame would then wait for the 64 ms safety net and show up as
            // rxStrandedRecovered() -- W12 fault #5 rebuilt, wearing the
            // costume of the counter that detects it.
            if (m_intUpldLatches == upldGen) {
                m_intPending &= (uint8_t)~(st & HOST_INT_UP_LD);
            }
        }
        if (st & CMD_PORT_UPLD) {
            // Consumed on entry: the command port's flag entitles this branch
            // to one look, and it takes it below.  (Same rule as diagConnect.)
            m_intPending &= (uint8_t)~CMD_PORT_UPLD;
            // W16: the command-port length is in the register snapshot at
            // 0xC0/0xC1, which is why NXP's MAX_MP_REGS reaches as far as
            // 0xC3.  The two CMD52s this replaces were the last per-packet
            // register polls on the service path.  The `haveMp` fallback is
            // not dead code: it is what keeps this branch correct if the read
            // condition above is ever narrowed again.
            uint16_t len;
            if (haveMp) {
                len = mp.cmdRdLen;
            } else {
                uint8_t lo = 0, hi = 0;
                m_host.cmd52Read(1, CMD_RD_LEN_0, &lo);
                m_host.cmd52Read(1, CMD_RD_LEN_1, &hi);
                m_cmd52PollsSvc += 2;   // W11: see m_cmd52PollsSvc's comment in Iw416.h
                len = (uint16_t)(lo | ((uint16_t)hi << 8));
            }
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
        // W15: this pass has now done the servicing the ISR masked signalling
        // for, so put it back BEFORE any return below.  Level, not edge: if the
        // card still holds DAT1 (a frame that arrived during the service
        // window, or a ring the drain did not empty) the interrupt fires the
        // instant this write lands and the next pass picks it up -- which is
        // why masking during the window loses nothing.  The `else armCardInt()`
        // at the top of the loop covers the paths that leave via `return s`.
        if (m_intMode && intFired) m_host.armCardInt();
        if (gotFrame || gotDrop) return SdioHost::OK;
        if (!(st & (HOST_INT_UP_LD | CMD_PORT_UPLD))) delay(1);
    }
    return SdioHost::CMD_TIMEOUT;      // a quiet poll, not an error
}

// W15.  Enabling is two independent switches and both must be on: the CARD
// drives DAT1 only for bits its HOST_INT_MASK lets through (enableHostInt()
// does that, and must already have run), and the HOST delivers it to the CPU
// only once INT_SIGNAL_EN[CINT] is set (SdioHost::enableCardInt does that).
// Disabling restores the polled path exactly -- serviceLink() reads
// HOST_INT_STATUS on every pass again, and nothing else in the driver behaves
// differently -- so the fallback cannot rot: it is the same code either way,
// selected by one branch.
void Iw416::setInterruptMode(bool on) {
    if (on == m_intMode) return;
    if (on) {
        // Card-side gate FIRST: SDIO card interrupts are enabled in CCCR
        // 0x04 -- IENM (bit 0) AND IEN1 (bit 1) -- and without that write
        // the card never asserts DAT1 no matter what HOST_INT_MASK says.
        // Silicon proved the omission (W15 phase 3): uSDHC armed (INT_
        // STATUS_EN/INT_SIGNAL_EN bit 8 set), INT_STATUS bit 8 never
        // latched, cardints=0 for the whole run.  HOST_INT_MASK (fn1)
        // selects WHICH conditions the fw raises; CCCR 0x04 gates the PIN.
        // NXP's fsl_sdio.c SDIO_EnableIOInterrupt() writes this register.
        // On failure stay polled and honest: interruptMode() reads false.
        // NOTE: this early-out (on==m_intMode above) means a CARD-level
        // reset (PDn toggle / power cycle) that clears CCCR 0x04 cannot be
        // repaired by calling setInterruptMode(true) again -- m_intMode is
        // still true, so the call no-ops and the mode silently reverts to
        // tick-rate polling.  Any future recovery flow that resets the CARD
        // must toggle false->true (or extend this to re-verify CCCR).  An
        // in-band firmware re-download does NOT clear CCCR -- see
        // downloadFirmware()'s note -- so this only bites on card resets.
        if (m_host.cmd52Write(0, 0x04, 0x03) != SdioHost::OK) return;
        m_intMode = true;
        m_host.enableCardInt(true);
    } else {
        m_intMode = false;
        m_host.enableCardInt(false);
        // Best-effort card-side disable; the controller is already deaf.
        (void)m_host.cmd52Write(0, 0x04, 0x00);
    }
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
