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

    // --- host command interface (available once firmware is running) ---
    //
    // Framing, from NXP's wifi-sdio.c SDIOPkt:
    //   [u16 total_size][u16 pkttype=MLAN_TYPE_CMD]
    //   [u16 command][u16 size][u16 seq_num][u16 result][body...]
    // The whole thing is CMD53-written to ioport | CMD_PORT_SLCT and the reply
    // is read back the same way once the card raises CMD_PORT_UPLD (bit 6) --
    // the data-port UP_LD bit (0) never fires for command traffic.
    static const uint16_t MLAN_TYPE_CMD   = 1;
    static const uint16_t MLAN_TYPE_EVENT = 3;
    // A response echoes the request's command id with this bit set.
    static const uint16_t HOSTCMD_RET_BIT = 0x8000;
    static const uint16_t INTF_HEADER_LEN = 4;
    static const uint32_t HOST_INT_UP_LD  = 0x01;   // data-port upload
    // A reply to a command sent via CMD_PORT_SLCT is flagged by the COMMAND
    // port's own bit, not the data-port UP_LD bit.  Polling UP_LD for a command
    // response waits forever.
    static const uint32_t CMD_PORT_UPLD    = (1u << 6);
    static const uint32_t CMD_PORT_DNLD    = (1u << 7);
    static const uint32_t HIM_ENABLE       = 0x01 | 0x02 | (1u << 6) | (1u << 7);
    // DATA-port read lengths (port 0 of the multiport scheme).  A COMMAND-port
    // reply does NOT publish its length here -- that is CMD_RD_LEN_0/1 below.
    // (0x18/0x19 is the SD8978 address; 0x08/0x09 is SD8801.)
    static const uint32_t RD_LEN_P0_L_REG = 0x18;
    static const uint32_t RD_LEN_P0_U_REG = 0x19;
    // --- SD8978 "new mode" command-port configuration ---
    // NXP's wlan_sdio_init_ioport() (wifidriver/sdio.c) performs ALL of these
    // read-modify-writes before the firmware download, and never talks to the
    // command port without them.  Omitting them is why W3 initially saw no
    // response ever: CMD_PORT_RD_LEN_EN is what makes the card publish a
    // reply's length at CMD_RD_LEN_0/1, and HOST_INT_RSR makes HOST_INT_STATUS
    // clear on READ -- NXP never writes that register anywhere.
    static const uint32_t HOST_INT_RSR_REG    = 0x04;
    static const uint8_t  HOST_INT_RSR_MASK   = 0xFF;   // all bits read-to-clear
    static const uint32_t CARD_CONFIG_2_1_REG = 0xD9;
    static const uint8_t  CMD53_NEW_MODE      = 0x01;
    static const uint32_t CMD_CONFIG_0        = 0xC4;
    static const uint8_t  CMD_PORT_RD_LEN_EN  = (1u << 2);
    static const uint32_t CMD_CONFIG_1        = 0xC5;
    static const uint8_t  CMD_PORT_AUTO_EN    = 0x01;
    static const uint32_t CARD_MISC_CFG_REG   = 0xD8;
    static const uint8_t  AUTO_RE_ENABLE_INT  = (1u << 4);
    static const uint32_t CMD_RD_LEN_0        = 0xC0;   // command-port reply length
    static const uint32_t CMD_RD_LEN_1        = 0xC1;
    // Commands go to ioport | CMD_PORT_SLCT, not the bare I/O port.  The
    // bootloader accepts the bare port during firmware download; the running
    // firmware does not, and a command sent there is simply never answered.
    static const uint32_t CMD_PORT_SLCT   = 0x8000;
    static const uint16_t CMD_FUNC_INIT   = 0x00A9;
    static const uint16_t CMD_GET_HW_SPEC = 0x0003;
    static const uint16_t CMD_MAC_CONTROL = 0x0028;
    static const uint16_t CMD_802_11_SCAN = 0x0006;
    static const uint16_t CMD_NET_MONITOR = 0x0102;
    static const uint16_t CMD_SUPPLICANT_PMK = 0x00C4;   // embedded supplicant
    static const uint16_t CMD_RECONF_TX_BUF  = 0x00D9;   // host TX buffer size
    static const uint16_t CMD_11N_CFG        = 0x00CD;
    static const uint16_t CMD_AMSDU_AGGR_CTRL = 0x00DF;
    static const uint32_t MAC_RTS_CTS        = 0x0200;   // HostCmd_ACT_MAC_RTS_CTS_ENABLE
    static const uint16_t CMD_802_11_ASSOCIATE = 0x0012;
    static const uint16_t CMD_DEAUTHENTICATE   = 0x0024;
    static const uint16_t TLV_TYPE_SSID_ID    = 0x0000;
    static const uint16_t TLV_TYPE_PASSPHRASE = 0x013C;  // PROPRIETARY_BASE+0x3C
    static const uint16_t TLV_TYPE_PMK        = 0x0144;  // PROPRIETARY_BASE+0x44
    // MAC_CONTROL action bits (HostCmd_ACT_MAC_*): the minimum a scan needs.
    static const uint32_t MAC_RX_ON       = 0x0001;
    static const uint32_t MAC_TX_ON       = 0x0002;
    static const uint32_t MAC_ETHERNETII  = 0x0010;
    // Scan request pieces (mlan_fw.h)
    static const uint8_t  BSS_MODE_ANY    = 0x03;
    static const uint16_t TLV_CHANLIST    = 0x0101;  // PROPRIETARY_TLV_BASE_ID+1
    // NET_MONITOR request pieces (mlan_fw.h / mlan_misc.c)
    static const uint16_t ACT_GEN_SET     = 0x0001;
    static const uint16_t HostCmd_ACT_GET = 0x0000;
    static const uint16_t MON_FILTER_ALL  = 0x0007;  // mgmt|ctrl|data
    static const uint16_t TLV_CHAN_BAND   = 0x012A;  // TLV_TYPE_UAP_CHAN_BAND_CONFIG
    static const uint16_t TLV_MON_FILTER  = 0x0138;  // TLV_TYPE_UAP_STA_MAC_ADDR_FILTER
    // Data-port RX (mlan_sdio_defs.h, SD8978 arm).
    // ★ W8: the SD8978 has THIRTY-TWO data ports, not sixteen, and both
    // directions are RINGS.  The firmware uploads each successive RX packet to
    // the NEXT port (0,1,2,...,31, wrap), and only recycles TX download ports
    // in a batch when the host has written the ring's top.  A driver that only
    // reads the low 16 bitmap bits loses every upload after the ring position
    // passes 15 -- which is exactly what the W5 monitor capture did (16 frames
    // = ring slots 0..15), leaving W6/W7 blind to EAPOL and every data frame.
    static const uint32_t RD_BITMAP_L_REG  = 0x10;
    static const uint32_t RD_BITMAP_U_REG  = 0x11;
    static const uint32_t RD_BITMAP_1L_REG = 0x12;
    static const uint32_t RD_BITMAP_1U_REG = 0x13;
    static const uint8_t  MAX_DATA_PORTS   = 32;     // mlan MAX_PORT for SD8978
    static const uint16_t MLAN_TYPE_DATA  = 0x0000;  // SDIOPkt pkttype for data
    static const uint16_t PKT_TYPE_802DOT11 = 0x0005; // RxPD rx_pkt_type in monitor
    // Data-port TX (W7).  The card publishes free download ports in WR_BITMAP;
    // the host writes the ring port (m_txPort) at ioport|port.  (0x14..0x17 is
    // the SD8978 address; SD8801 uses 0x06/0x07 and has only 16 ports.)
    static const uint32_t WR_BITMAP_L_REG  = 0x14;
    static const uint32_t WR_BITMAP_U_REG  = 0x15;
    static const uint32_t WR_BITMAP_1L_REG = 0x16;
    static const uint32_t WR_BITMAP_1U_REG = 0x17;
    static const uint32_t HOST_INT_DN_LD  = 0x02;    // tx ports freed
    // sizeof(TxPD) in NXP's mlan_fw.h.  Their process_pkt_hdrs() hardcodes
    // tx_pkt_offset = 0x16 ("we'll just make this constant"), so the 802.3
    // frame always starts 22 bytes after the SDIOPkt header.
    static const uint16_t TXPD_LEN        = 22;

    // Unmask HIM_ENABLE.  Call AFTER firmware download reports FIRMWARE_READY,
    // never before -- NXP's sd_wifi_post_init() is the reference.  Everything
    // is masked during begin(), matching wlan_sdio_init_ioport().
    SdioHost::Status enableHostInt();

    SdioHost::Status sendHostCmd(uint16_t cmd, const uint8_t *body, uint16_t bodyLen);
    // Same, addressed to a specific BSS.  mlan packs the target interface into
    // seq_num's HIGH byte -- bits 11:8 bss_num, bits 15:12 bss_type
    // (HostCmd_SET_SEQ_NO_BSS_INFO, mlan_fw.h; wlan_ops_uap_prepare_cmd is
    // what routes an APCMD there).  Every command this driver sent before W17
    // left both at 0, i.e. the STA interface, and sendHostCmd() above still
    // does exactly that -- it delegates here with (0, 0), so no existing
    // caller changes shape or wire bytes.
    //
    // BSS_TYPE_UAP is what the AP-mode commands (APCMD 0x00B0..0x00B3) need.
    // ★ waitCmdResp() already compares only the LOW byte of the echoed
    // seq_num, so a reply carrying a non-zero bss nibble still correlates --
    // that masking was written speculatively in W12 ("keeps the comparison
    // honest even if that ever changes") and this is the change it meant.
    static const uint8_t BSS_TYPE_STA = 0;   // MLAN_BSS_TYPE_STA
    static const uint8_t BSS_TYPE_UAP = 1;   // MLAN_BSS_TYPE_UAP
    SdioHost::Status sendHostCmdBss(uint16_t cmd, const uint8_t *body, uint16_t bodyLen,
                                    uint8_t bssType, uint8_t bssNum = 0);
    // --- uAP (AP mode) command ids, mlan_fw.h ---------------------------------
    // W17 Phase 0 only PROBES for these; nothing in this driver implements AP
    // mode yet.  SYS_INFO/BSS_STOP/STA_LIST take an empty body (mlan sends
    // S_DS_GEN and nothing else); SYS_CONFIGURE takes action(2) + TLVs;
    // BSS_START is empty unless host-MLME is in use, which it is not here.
    static const uint16_t CMD_APCMD_SYS_INFO      = 0x00AE;
    static const uint16_t CMD_APCMD_SYS_CONFIGURE = 0x00B0;
    static const uint16_t CMD_APCMD_BSS_START     = 0x00B1;
    static const uint16_t CMD_APCMD_BSS_STOP      = 0x00B2;
    static const uint16_t CMD_APCMD_STA_LIST      = 0x00B3;
    // HostCmd_RESULT_* (mlan_fw.h).  The probe's whole question is which of
    // these -- or a timeout, or no reply at all -- comes back.
    static const uint16_t RESULT_OK          = 0x0000;
    static const uint16_t RESULT_ERROR       = 0x0001;
    static const uint16_t RESULT_NOT_SUPPORT = 0x0002;

    // --- uAP configuration (SYS_CONFIGURE 0x00B0) -----------------------------
    // ★ SILICON, W17 2026-08-21: THIS WORKS, and the reason it is worth saying
    // so loudly is that a MINIMAL SYS_CONFIGURE does not -- it kills the command
    // port outright, with no reply and nothing answering again for the rest of
    // the firmware life.  Both facts were measured in ONE firmware life, on the
    // same card, minutes apart:
    //     SYSCFG.fullopen (this, 82-byte body)  st=ok result=0x0000  on BOTH bss
    //     HWSPEC control immediately after      st=ok          <- port healthy
    //     SYSCFG minimal (action + one TLV)     cmd-timeout, port dead
    // So the firmware's 0x00B0 handler REQUIRES A POPULATED CONFIGURATION.  It
    // is not that the command is unsupported, malformed, or interface-specific:
    // three minimal shapes (bare action; action+CHAN_BAND GET; action+
    // MAX_STA_CNT SET) all wedge identically, while this one is accepted.
    //
    // Two rules follow for anything built on this:
    //   * NEVER send a partial SYS_CONFIGURE, not even to probe.  There is no
    //     harmless GET -- a GET is exactly the shape that wedges.
    //   * A wedge is recoverable ONLY by a card reset (M.2 reset GPIO + blob
    //     re-download; no MCU reboot needed, no power cycle).  Nothing shorter
    //     works: 30 retries over ~90 s never recovered, and draining the data
    //     port found nothing to drain.
    // Full account, with the wire bytes: rt1176-evkb
    // examples/networking/m2_uap_probe/transcript_hw_evkb.txt, "W17 FAULT 1".
    //
    // Open (no security) for now -- it needs no credentials, which keeps this
    // clear of the repo's standing SSID/PSK rule.  WPA2 adds AKMP/cipher/
    // passphrase TLVs on top of the same builder.
    // Configuring does NOT transmit; BSS_START (0x00B1) is what beacons.
    enum UapTlv : uint16_t {
        UAP_TLV_MAC      = 1u << 0,   // 0x012B  MAC address        len 6
        UAP_TLV_SSID     = 1u << 1,   // 0x0000  SSID               len = strlen
        UAP_TLV_BEACON   = 1u << 2,   // 0x012C  beacon period      len 2
        UAP_TLV_DTIM     = 1u << 3,   // 0x012D  DTIM period        len 1
        UAP_TLV_RATES    = 1u << 4,   // 0x0001  supported rates    len = count
        UAP_TLV_BCAST    = 1u << 5,   // 0x0130  broadcast SSID ctl len 1
        UAP_TLV_CHANBAND = 1u << 6,   // 0x012A  band cfg + channel len 2
        UAP_TLV_AUTH     = 1u << 7,   // 0x011F  auth type          len 3
        UAP_TLV_PROTOCOL = 1u << 8,   // 0x0140  encrypt protocol   len 2
        // Everything mlan emits for an OPEN AP on a manual 2.4 GHz channel.
        UAP_TLV_ALL_OPEN = 0x01FF,
    };
    // A mask rather than a fixed set, because the point is to BISECT: if the
    // full configuration is accepted and a partial one is not, the boundary is
    // the finding, and that can only be found by varying the set.
    struct UapConfig {
        const char    *ssid;          // NUL-terminated, <= 32 bytes
        uint8_t        channel;       // 1..14; 2.4 GHz, BAND_CONFIG_MANUAL
        const uint8_t *mac;           // 6 bytes, or nullptr to send zeros
        uint16_t       beaconPeriod;  // mlan range 50..4000, its default is 100
        uint8_t        dtimPeriod;    // mlan range 1..100
        uint8_t        bcastSsidCtl;  // 0 hidden, 1 broadcast, 2 clear
        uint16_t       tlvMask;       // UapTlv bits; UAP_TLV_ALL_OPEN for all
    };
    // Sends one SYS_CONFIGURE SET on the uAP interface and waits for the reply.
    // Returns the transport status; the firmware's own verdict is in
    // lastRespResult().  Does NOT transmit -- BSS_START is what beacons.
    SdioHost::Status uapConfigure(const UapConfig &cfg);
    // The request uapConfigure() last built, so a caller can put the exact
    // bytes on the record.  This project's standard for a firmware claim is the
    // wire bytes quoted, and a TLV soup is precisely the thing nobody should be
    // asked to take on trust.  Valid until the next uapConfigure() call.
    const uint8_t *uapCfgReq()    const { return m_uapCfgReq; }
    uint16_t       uapCfgReqLen() const { return m_uapCfgReqLen; }

    SdioHost::Status readHostResp(uint8_t *buf, uint16_t bufLen, uint16_t *outLen,
                                  uint32_t timeoutMs = 2000);
    // Read command-port packets until the response to `cmd` arrives.  The port
    // also carries EVENT packets and the replies to earlier commands, so the
    // first packet read is NOT necessarily the answer -- taking it on faith is
    // how GET_HW_SPEC first "succeeded" with an all-zero MAC.
    SdioHost::Status waitCmdResp(uint16_t cmd, uint8_t *buf, uint16_t bufLen, uint16_t *outLen,
                                 uint32_t timeoutMs = 2000);

    // MAC_CONTROL (0x0028): body is a single LE u32 of MAC_* action bits.
    // NXP's wlan_fw_init_cfg() sends this before anything RF-facing; a scan
    // without it is the first thing to suspect if scan() returns no sets.
    SdioHost::Status macControl(uint32_t action);

    // One 802.11 scan over the command port (legacy HostCmd 0x0006): active
    // scan of 2.4 GHz channels 1..13, 100 ms dwell each.  Fills `out` with up
    // to maxOut results; *outCount is what fit, scanSetsSeen() is what the
    // card actually reported.  The response wait is long (default 15 s): 13
    // channels x 100 ms plus firmware overhead exceeds the 2 s default.
    // Security classification of a scanned BSS, from the beacon capability
    // Privacy bit and the RSN / WPA IEs.  W6 associates to OPEN networks
    // first, so this is the field that decides which APs are candidates.
    enum Security : uint8_t { SEC_OPEN = 0, SEC_WEP = 1, SEC_WPA = 2, SEC_WPA2 = 3 };
    struct ScanResult {
        uint8_t  bssid[6];
        uint8_t  rssi;        // raw byte from the response; dBm is -rssi
        uint8_t  channel;     // from the DS Param Set IE; 0 if absent
        uint8_t  security;    // Security enum
        uint16_t capability;  // raw capability field (bit 4 = Privacy)
        char     ssid[33];    // NUL-terminated; empty for hidden SSIDs
        // The full RSN IE (id 48 + len + body), kept for W6 ASSOCIATE which
        // echoes it back to the firmware.  rsnLen is 0 for open/WEP/WPA.
        uint8_t  rsnIe[64];
        uint8_t  rsnLen;
        // Supported + extended rates (IE 1 and IE 50 bodies concatenated),
        // for the ASSOCIATE rates TLV.
        uint8_t  rates[16];
        uint8_t  ratesLen;
    };
    SdioHost::Status scan(ScanResult *out, uint8_t maxOut, uint8_t *outCount);
    uint8_t scanSetsSeen() const { return m_scanSets; }

    // --- W6: WPA2 association via the firmware's embedded supplicant ---
    //
    // Hand the firmware the SSID + passphrase (SUPPLICANT_PMK 0x00C4): it
    // derives the PMK (PBKDF2 internally) and will run the 4-way handshake
    // during ASSOCIATE, so the host performs no WPA2 crypto.  Must be sent
    // before associate().  resp_result names a firmware that lacks the
    // embedded supplicant, or a malformed request.
    SdioHost::Status setPassphrase(const char *ssid, const char *psk);
    uint16_t lastSuppResult() const { return m_lastRespResult; }

    // Query the PMK the firmware cached for `ssid` (SUPPLICANT_PMK, action GET).
    // Decisive test of the embedded supplicant: a non-zero 32-byte PMK proves
    // the firmware derived it (supplicant present + keyed); an absent/zero PMK
    // proves SUPPLICANT_PMK is inert (no embedded supplicant in this blob).
    // *pmkNonZero and *pmkFound report the outcome; out32 gets the bytes.
    SdioHost::Status queryPmk(const char *ssid, uint8_t out32[32],
                              bool *pmkFound, bool *pmkNonZero);

    // Associate to a scanned BSS (ASSOCIATE 0x0012).  For WPA2 the firmware
    // completes the 4-way handshake using the PMK cached by setPassphrase(),
    // so this one call both associates AND authenticates.  The 802.11
    // association status code lands in assocStatus() -- 0 = success (a wrong
    // PSK fails the handshake and shows here as a non-zero status).
    SdioHost::Status associate(const ScanResult &ap);
    uint16_t assocStatus() const { return m_assocStatus; }
    // The ASSOCIATE response's cap_info/error-return field.  A normal IEEE
    // capability (small value, Privacy bit maybe set) means status is a real
    // AP status; 0xFFFB..0xFFFF means an internal error/timeout and status is
    // a firmware sub-code (0xFFFC = timed out waiting for the AP).
    uint16_t assocCapInfo() const { return m_assocCapInfo; }
    // The normalised RSN IE actually sent in the last ASSOCIATE (id+len+body),
    // for verifying the PMF-cap rewrite.
    const uint8_t *assocRsnIe() const { return m_assocRsn; }
    uint8_t assocRsnIeLen()   const { return m_assocRsnLen; }
    // Deauthenticate from a BSS (0x0024).  Called automatically before
    // associate() to clear stale state; exposed for explicit disconnects.
    SdioHost::Status deauthenticate(const uint8_t bssid[6]);

    // --- W10: IEEE power save (the fw idle RX-death workaround) ---
    // The firmware's data-RX engine dies stochastically after ~14-44 min of
    // SPARSE traffic when IEEE PS is off (silicon A/B incl. NXP's own stack,
    // 2026-08-19; PS on survived 60 min clean -- see the W10 handoff in the
    // rt1176-evkb repo).  NXP ships PS on by default; so do we, from
    // connectStation().
    static const uint16_t CMD_PS_MODE_ENH   = 0x00E4;
    static const uint16_t PS_EN_AUTO_PS     = 0x00FF;
    static const uint16_t PS_DIS_AUTO_PS    = 0x00FE;
    static const uint16_t PS_SLEEP_CONFIRM  = 0x0005;
    static const uint16_t PS_BITMAP_STA     = 0x0010;
    static const uint16_t TLV_TYPE_PS_PARAM = 0x0172;
    static const uint32_t EVENT_PS_AWAKE    = 0x0000000A;
    static const uint32_t EVENT_PS_SLEEP    = 0x0000000B;
    static const uint8_t  HOST_POWER_UP     = 0x02;   // fn1 reg 0x00 wake write
    enum PsState : uint8_t { PS_AWAKE = 0, PS_SLEEPING = 1 };

    // Enable/disable IEEE PS (EN_AUTO_PS/DIS_AUTO_PS with the STA bitmap and
    // NXP's default ps_param).  Call while associated; the sleep/confirm
    // handshake then runs inside serviceLink()'s event demux, so the app must
    // keep servicing the link (both examples do).  wakeCardIfSleeping() is
    // applied automatically before host-initiated traffic while sleeping.
    // On a transport failure (non-OK return), m_lastRespResult/lastRespResult()
    // is stale (from whatever command last completed, not this one) --
    // ieeePsEnabled() is the reliable signal for whether PS actually ended up
    // enabled.
    SdioHost::Status setIeeePs(bool enable);
    bool     ieeePsEnabled() const { return m_psEnabled; }
    uint8_t  psState()       const { return (uint8_t)m_psState; }
    uint32_t psSleeps()      const { return m_psSleeps; }   // confirms sent
    uint32_t psWakes()       const { return m_psWakes; }    // awake events seen
    // Soak-disambiguation counters (W10 review).  A DATA-path host wake
    // (sendDataFrame -> wakeCardIfSleeping) increments BOTH psHostWakes() (at
    // the gate write itself) AND, once the fw's own PS_AWAKE arrives via
    // serviceLink(), psWakes() -- so in the common case
    // psWakes()+psHostWakes() normally EXCEEDS psSleeps(), by roughly the
    // number of data-path host wakes (each one is double-counted across the
    // two counters).  The honest invariant to watch is the other direction:
    // psSleeps() - psWakes() approximates PS_AWAKE events that were
    // swallowed -- mostly command-path wakes, whose PS_AWAKE arrives while
    // waitCmdResp's discard loop (not serviceLink's demux) is reading the
    // command port and so is never counted (see the comment there) -- plus
    // any wake the fw genuinely raised no event for.  psHostWakes() counts
    // gate writes on BOTH the data and command paths and is orthogonal to
    // that invariant, not a term in it.
    uint32_t psConfirmFails() const { return m_psConfirmFails; } // sendSleepConfirm's send failed
    uint32_t psHostWakes()    const { return m_psHostWakes; }    // wakeCardIfSleeping actually wrote HOST_POWER_UP

    // W11: bus-attribution counters, for throughput work -- how much of the
    // SDIO bus a frame actually costs.  See the m_cmd52PollsTx/m_cmd52PollsSvc/
    // m_cmd53Count/m_cmd53Bytes/m_cmd53ByteMode declarations below for the
    // exact attribution rules (what counts as "Tx" vs "Svc", why
    // wakeCardIfSleeping()'s write is excluded, and why the RX helpers are
    // counted at the CMD52/CMD53 call site rather than by caller).
    uint32_t cmd52PollsTx()   const { return m_cmd52PollsTx; }   // sendDataFrame's wr_bitmap wait
    uint32_t cmd52PollsSvc()  const { return m_cmd52PollsSvc; }  // serviceLink + its RX helpers

    // --- W15: interrupt-driven service (the SDIO card interrupt, DAT1) ------
    //
    // Polled, serviceLink() reads HOST_INT_STATUS once per pass and paces
    // itself with delay(1): ~1000 CMD52 per second on a link doing absolutely
    // nothing.  In interrupt mode the CARD says when it has work -- it pulls
    // DAT1 whenever HOST_INT_STATUS has a bit its HOST_INT_MASK lets through --
    // and a quiet pass issues no bus traffic at all.
    //
    // ★ DEFAULT: OFF, and it stays off until silicon says otherwise.  The
    // RT1176 uSDHC's card-interrupt behaviour has never been exercised on the
    // MIMXRT1170-EVKB, and the QEMU model it was developed against is the one
    // behaviour in that file derived from the SPECIFICATION rather than from a
    // capture (its MODELLING NOTE 10 says so).  A green gate proves this driver
    // matches the spec as read there, not that it matches the card -- so the
    // polled path remains the default and remains correct, and turning this on
    // is a deliberate act.
    //
    // PRECONDITIONS: call after SdioHost::begin() and after enableHostInt()
    // (the card side has to be unmasked for the card to drive DAT1 at all).
    //
    // WHAT DOES *NOT* CHANGE, and this is the load-bearing part: the W12/W13
    // RD-bitmap safety net still runs on exactly the same schedule.  It fires
    // every RX_BITMAP_CHECK_PASSES *quiet passes* and quiet passes are
    // untouched -- serviceLink still loops once per delay(1), it just stops
    // issuing a CMD52 on each one.  See the comment at the top of
    // serviceLink()'s loop for why that had to be checked rather than assumed:
    // a net whose trigger is "quiet passes" would be silently DEAD if
    // interrupt mode had removed the passes instead of the reads, and the
    // W13 evidence (rxStrandedRecovered() still climbing ~3 per 80 blasts on
    // silicon) says this firmware genuinely needs it.
    void setInterruptMode(bool on);
    bool interruptMode() const { return m_intMode; }
    // DAT1 assertions serviced.  This is the un-fakeable "interrupt mode
    // actually engaged" signal: a driver that stayed polled -- or one whose
    // controller-side enable silently did not take -- reads 0 here no matter
    // how many frames it delivers.  Reset per firmware life, like the other
    // service counters.
    uint32_t cardInts() const { return m_cardInts; }
    uint32_t cmd53Count()     const { return m_cmd53Rx + m_cmd53Tx; }  // data CMD53s, both directions
    uint32_t cmd53Bytes()     const { return m_cmd53Bytes; }     // bytes successfully issued on the bus
    // Data-port CMD53s issued in BYTE mode.  Still 0 by construction and still
    // an invariant rather than a detector: both data-port sites pass block
    // mode.  ★ W16 DID introduce byte-mode CMD53 to this driver -- the
    // register-port read is one -- but that is not a DATA-port transfer and is
    // counted in cmd53Regs(); if a data-port byte-mode path is ever added,
    // this needs a real increment site, not a read of a value that already
    // exists.
    uint32_t cmd53ByteMode()  const { return m_cmd53ByteMode; }

    // --- W16: the multiport REGISTER-PORT snapshot -------------------------
    //
    // NXP's wlan_interrupt() (wifidriver/wifi-sdio.c) never polls this card's
    // status and bitmap registers with CMD52.  It reads the whole multiport
    // register group in ONE byte-mode CMD53 at function 1 address 0 --
    //   sdio_drv_read(REG_PORT | MLAN_SDIO_BYTE_MODE_MASK, 1, 1, MAX_MP_REGS, ..)
    // -- and everything the service loop needs comes out of that one buffer:
    //
    //   0x0C        HOST_INT_STATUS   (clear-on-read, as we configure it)
    //   0x10..0x13  RD_BITMAP         (32 bits)
    //   0x14..0x17  WR_BITMAP         (32 bits)
    //   0x18..0x57  RD_LEN_P0..P31    (32 LE pairs, 0x18 + (port << 1))
    //   0xC0..0xC1  CMD_RD_LEN        (command-port reply length)
    //
    // Before W16 this driver paid SEVEN CMD52s per RX frame for exactly those
    // values (1 status + 4 rd-bitmap + 2 RD_LEN) and four per TX wr-bitmap
    // poll.  At the ~0.1 ms per bus command measured in W11 that IS the
    // ~10-commands-per-frame ceiling.
    //
    // ★ THIS IS NOT A CACHE, AND THE DISTINCTION IS THE WHOLE W11 LESSON.
    // W11 tried to amortise the bitmap reads by holding a STALE view across
    // frames and regressed throughput 2.5x: the host burned every ring credit
    // in a burst and then sat in a dry-wait against delay(1) granularity,
    // because the read-per-frame pattern had accidentally been pacing it to
    // the firmware's own cadence.  Here every service pass still issues a real
    // read of the real registers -- one command instead of seven, same
    // freshness, same pacing.  Never turn this into a view that outlives the
    // pass that took it.
    static const uint16_t MP_REGS_LEN = 196;   // MAX_MP_REGS for the SD8978
    struct MpRegs {
        uint8_t  intStatus;
        uint32_t rdBitmap;
        // ★ VALID ON THE TX PATH ONLY when the CMD52 fallback filled this
        // struct (mpRegsUsable() false).  Nothing on the service path reads
        // it, and fetching it there would cost four CMD52s per poll on the one
        // path that is already paying the pre-W16 price.  The register port
        // always fills it -- it comes free in the same 196 bytes.
        uint32_t wrBitmap;
        uint16_t cmdRdLen;
        uint16_t rdLen[MAX_DATA_PORTS];
    };
    // One register-port read.  Accumulates the interrupt bits into the W12
    // sticky accumulator exactly as every other HOST_INT_STATUS reader does --
    // this read CONSUMES the clear-on-read register, so a site that dropped
    // the bits here would re-create fault #5 in a new place.
    // `txPath` splits the cost the same way m_cmd52PollsTx/m_cmd52PollsSvc
    // split it: the TX-side wr-bitmap wait and the service-side poll are
    // separately attributable, which is what let W11 pin its 2.5x regression on
    // the TX path specifically.  Collapsing them into one number would have
    // hidden that.
    SdioHost::Status readMpRegs(MpRegs *out, bool txPath = false);
    // Has the register port been PROVEN to work on this card?
    //
    // ★ THIS EXISTS BECAUSE THE ONE UNVERIFIED ASSUMPTION IN W16 FAILS
    // SILENTLY, AND FAILS LOOKING HEALTHY.  NXP reads the register file with
    // the CMD53 address held FIXED (OP Code 0).  On an ordinary SDIO function
    // that means "read register 0, 196 times"; we believe this card's register
    // port streams the file instead, because NXP's driver depends on it -- but
    // no capture in this tree proves it.  Register 0 is HOST_POWER_UP, which
    // reads back 0, so a card that took OP Code 0 literally would hand us 196
    // zero bytes: intStatus 0, both bitmaps 0, every RD_LEN 0.  That is
    // BYTE-IDENTICAL to a healthy idle card.  RX would go permanently silent
    // (the safety net sees an empty bitmap and never fires), TX would time out
    // on an empty wr-bitmap, and the read would still be eating the card's
    // clear-on-read interrupt status the whole time.
    //
    // So the FIRST snapshot of each firmware life is tested for the shape that
    // failure has: IS EVERY BYTE THE SAME?  A port that obeyed OP Code 0
    // literally returns N copies of one register and cannot produce anything
    // else; a port that streams cannot produce a uniform 196 bytes on a
    // running card (the per-slot RD_LEN block is zeros on an idle link while
    // 0x5C carries the command-port ready bits).  The test is of the failure
    // MODE, not of any register's value, which is what makes it immune to the
    // thing that broke the first attempt.
    //
    // ★ THE FIRST ATTEMPT WAS A STALE-CONSTANT COMPARISON, AND SILICON KILLED
    // IT IN ONE RUN.  It compared the snapshot's byte at 0x5C against the
    // cardStatus() value begin() had read by CMD52 -- "a register that cannot
    // be zero".  On the bench that read 0x40 against an expected 0x0D and the
    // driver dutifully fell back to CMD52 on a card whose register port was
    // working perfectly.  0x5C is CARD_TO_HOST_EVENT_REG: it means
    // DN_LD_CARD_RDY|CARD_IO_READY (0x0D) to the BOOT ROM and UP_LD_CP_RDY
    // (0x40) once firmware is running (NXP mlan_sdio_defs.h).  The reference
    // was not a constant at all -- it was a live register captured minutes
    // earlier, in a different phase of the card's life.
    // The live CMD52 reading is still taken, once, and reported through
    // mpRegsWitness() -- but as EVIDENCE, never as a verdict: two reads of a
    // live register microseconds apart are allowed to differ.
    bool     mpRegsUsable()   const { return m_mpRegsOk; }
    // Force the transport, for a CONTROLLED A/B on hardware.  useRegisterPort
    // (false) selects the pre-W16 CMD52 path on a card whose register port
    // works perfectly, which is the only way to measure the two against each
    // other in ONE firmware life -- the discipline W12 learned the hard way
    // when an A/B whose arms differed in a second respect (fresh boot vs
    // repeat run) measured the difference nobody intended.  It also marks the
    // port as already checked, so a forced arm is not silently re-tested and
    // flipped back mid-run.
    void useRegisterPort(bool on) { m_mpRegsOk = on; m_mpRegsChecked = true; }
    // The repeated byte, when a uniform snapshot got the port rejected.
    uint8_t  mpRegsRejected() const { return m_mpRegsBadStatus; }
    // What the two transports said about register 0x5C at the same moment, as
    // (snapshot << 8) | CMD52.  Diagnostic: on a healthy card both are the
    // card's live CARD_TO_HOST_EVENT value and normally agree.
    uint16_t mpRegsWitness()  const {
        return (uint16_t)(((uint16_t)m_mpRegsSnap << 8) | m_mpRegsLive);
    }
    // Register-port reads that failed on the bus (not the sanity check).
    // Non-zero means interrupt bits were consumed by a transfer that then
    // errored -- see readMpRegs() for what is done about that.
    uint32_t mpRegsErrors()   const { return m_mpRegsErrors; }
    // Register-port CMD53s.  Deliberately NOT folded into cmd53Count(), which
    // stays DATA-port-only so that "data CMD53s per frame" remains the clean
    // aggregation metric: aggregation is supposed to drive that number below
    // one per frame, and mixing register reads into it would hide exactly that.
    uint32_t cmd53RegsTx()  const { return m_cmd53RegsTx; }
    uint32_t cmd53RegsSvc() const { return m_cmd53RegsSvc; }
    uint32_t cmd53Regs()    const { return m_cmd53RegsTx + m_cmd53RegsSvc; }
    // Every SDIO command this driver issued on the DATA and SERVICE paths,
    // summed: the two CMD52 poll counters, both data-port CMD53 directions,
    // and both register-port reads.  THE number to normalise per frame when
    // judging a throughput change -- W11 measured the ceiling as ~10 of these
    // per frame at ~0.1 ms each.  A change that moves one term at another's
    // expense shows up here as flat, which is exactly what it should do.
    //
    // NOT included, deliberately: command/event-port CMD53s and one-off CMD52
    // WRITES (the PS wake gate, the config registers).  Those are per-command
    // or per-connection costs, not per-frame ones, and folding them in would
    // make a busy link's number move with how many host commands were sent.
    // ★ IT MUST NAME EVERY PER-FRAME COUNTER BY MEMBER.  W16 split
    // m_cmd53Count into m_cmd53Rx/m_cmd53Tx and this sum kept adding the old
    // member, which no longer had an increment site -- so it silently stopped
    // counting data CMD53s, the one term aggregation exists to move, while
    // still compiling and still looking like a total.  Any future split of a
    // term in here has to be reflected here in the same commit.
    uint32_t busCommands() const {
        return m_cmd52PollsTx + m_cmd52PollsSvc + m_cmd53Rx + m_cmd53Tx +
               m_cmd53RegsTx + m_cmd53RegsSvc;
    }

    // One-call station bring-up (W9): scan -> find `ssid` (exact byte match
    // against the beacon SSID; first match wins -- fine for a single-AP
    // bench, but a multi-BSSID SSID deterministically picks the first scan
    // entry, not the strongest) -> setPassphrase (skipped when psk is NULL
    // or empty -> open network) -> associate -> watchConnect, retrying the
    // associate up to `attempts` times on a rejection (CMD_CRC).
    // Short-circuits the retry loop early on EVENT_DEAUTHENTICATED reason 15
    // (handshake timeout -- see the EVENT_DEAUTHENTICATED comment below;
    // more attempts won't fix it).  OK means associated and settled (probe
    // rule: no deauth in the watch window; a port-release is not required
    // here) -- connectedAp() is updated ONLY on this path, immediately
    // before returning OK.  BAD_CIS = SSID not found in the scan.  Any other
    // return is the transport failure that stopped the last attempt:
    // CMD_CRC means the last attempt was rejected/handshake-refused, other
    // codes are the underlying bus/command error that associate() or
    // watchConnect() reported.  After a non-OK return, connectedAp() still
    // holds the LAST SUCCESSFUL connect's AP (or an empty ScanResult if
    // there has never been one) -- never the failed attempt's AP.
    // psOn (W10): enable IEEE PS on a successful connect (default true,
    // matching NXP's own default).  Best-effort -- pass false to soak the
    // erratum deliberately (see setIeeePs()).
    SdioHost::Status connectStation(const char *ssid, const char *psk,
                                    uint8_t attempts = 3, bool psOn = true);
    const ScanResult &connectedAp() const { return m_connectedAp; }

    // Firmware events that report the WPA2 handshake outcome.
    static const uint32_t EVENT_PORT_RELEASE     = 0x0000002B;  // handshake OK
    static const uint32_t EVENT_MIC_ERR_UNICAST  = 0x0000000E;
    static const uint32_t EVENT_MIC_ERR_MULTICAST = 0x0000000D;
    // After associate(), wait for the 4-way handshake result on the command
    // port.  OK = EVENT_PORT_RELEASE (authenticated, port open -- proves the
    // PSK); CMD_CRC = a MIC-error event (wrong PSK); CMD_TIMEOUT = neither in
    // time.  lastEvent() is the last event id seen, for the report.
    SdioHost::Status waitForConnect(uint32_t timeoutMs = 5000);
    uint32_t lastEvent() const { return m_lastEvent; }
    // Diagnostic connect watcher: monitors the data port for EAPOL (0x888E)
    // as well as the command-port event, to distinguish an embedded supplicant
    // (no EAPOL to us) from a host supplicant (EAPOL forwarded up).
    SdioHost::Status diagConnect(uint32_t timeoutMs = 6000);
    bool     diagEapolSeen()      const { return m_diagEapol; }
    uint16_t diagDataFrames()     const { return m_diagDataFrames; }
    uint16_t diagFirstEthertype() const { return m_diagFirstEthertype; }
    // Full event-sequence watcher: run the WHOLE window, recording every
    // command-port event id and a coarse ms timestamp, instead of returning on
    // the first one.  Reveals a connect-then-drop -- e.g. PORT_RELEASE (0x2B)
    // at t=X followed by a DEAUTH (0x08) ~1 s later -- which the return-early
    // watchers cannot show.  Returns OK if a PORT_RELEASE was ever seen (even
    // if a later deauth dropped it), CMD_CRC if only deauth/mic, else TIMEOUT.
    SdioHost::Status watchConnect(uint32_t timeoutMs = 8000);
    uint8_t  eventLogLen() const { return m_eventLogLen; }
    uint32_t eventLogId(uint8_t i)   const { return i < m_eventLogLen ? m_eventLog[i]  : 0; }
    uint16_t eventLogTime(uint8_t i) const { return i < m_eventLogLen ? m_eventLogT[i] : 0; }
    uint32_t eventLogInfo(uint8_t i) const { return i < m_eventLogLen ? m_eventLogI[i] : 0; }
    bool     sawPortRelease() const { return m_sawPortRelease; }
    // The 4 bytes after the last event's cause -- for EVENT_DEAUTHENTICATED,
    // the low 16 bits are (near) the IEEE reason code.
    uint32_t lastEventInfo() const { return m_lastEventInfo; }
    // Disassociate the retry loop should NOT keep hammering when the failure is
    // a wrong PSK: EVENT_DEAUTHENTICATED with reason 15 means the handshake
    // timed out, which more attempts will not fix.
    static const uint32_t EVENT_DEAUTHENTICATED = 0x00000008;
    static const uint32_t EVENT_DISASSOCIATED   = 0x00000009;
    // The AP disappearing (powered off / rebooted) is not a deauth -- the
    // firmware reports it as LINK_LOST.  A link watcher that only knows
    // deauth/disassoc keeps "servicing" a dead link forever (measured: the
    // ESP AP was reflashed mid-run and the probe held connected=1 for minutes).
    static const uint32_t EVENT_LINK_LOST       = 0x00000003;

    // --- W5: data-path RX via monitor mode ---
    //
    // NET_MONITOR (0x0102): put the firmware in promiscuous capture on one
    // 2.4 GHz channel with no association.  action SET, activity 1, filter
    // 0x07 (mgmt|ctrl|data).  resp_result names a firmware that lacks the
    // feature rather than hanging.
    SdioHost::Status netMonitor(bool enable, uint8_t channel);

    // Reusable data-port read: poll HOST_INT_STATUS for the DATA UP_LD bit,
    // find the lowest set RD_BITMAP port, read its RD_LEN_P<n>, CMD53-read the
    // packet at ioport|port.  *outLen is the SDIOPkt size, *port the port used,
    // *rxType the RxPD rx_pkt_type (valid only for MLAN_TYPE_DATA packets).
    SdioHost::Status readDataPacket(uint8_t *buf, uint16_t bufLen, uint16_t *outLen,
                                    uint8_t *port, uint16_t *rxType, uint32_t timeoutMs = 2000);

    // --- W7: data-path TX + connected-link service ---
    //
    // Send one 802.3 frame (dst[6] src[6] ethertype[2] payload) down the data
    // path: SDIOPkt header + zeroed TxPD (tx_pkt_type 0 = ethernet) + frame,
    // CMD53-written to the lowest free WR_BITMAP port.  CMD_TIMEOUT means the
    // card never freed a TX buffer inside timeoutMs -- lastWrBitmap() carries
    // the final bitmap read as evidence.
    // W16: with TX aggregation enabled (setTxAggregation) this ACCUMULATES the
    // frame instead of writing it, and the bus write happens in flushTx().  A
    // ring port is still reserved here, in order, and the wr-bitmap is still
    // waited on here -- only the CMD53 is deferred, and only as far as the end
    // of the caller's current poll iteration.
    SdioHost::Status sendDataFrame(const uint8_t *frame, uint16_t frameLen,
                                   uint32_t timeoutMs = 1000);

    // --- W16: multiport aggregation (MPA) ----------------------------------
    //
    // The card's data ports can be addressed as a RUN: one CMD53 carrying N
    // consecutive slots, at
    //     (ioport | SDIO_MPA_ADDR_BASE | ((N - 1) << 8)) + start_port
    // with each slot's packet padded to a 256-byte boundary and laid back to
    // back.  That is NXP's own encoding (wifidriver/wifi-sdio.c, wifi_tx_data()
    // and wlan_get_rd_port()); a single-slot transfer keeps the plain
    // ioport|port form.  Aggregation is the direct attack on the quantity W11
    // measured as the ceiling -- SDIO commands per frame -- because it is the
    // only one that can push DATA CMD53s BELOW one per frame.
    static const uint32_t MPA_ADDR_BASE = 0x1000;   // NXP SDIO_MPA_ADDR_BASE
    // NXP's SDIO_MP_AGGR_DEF_PKT_LIMIT for the SD8978 with WMM.  Their
    // _MAX is 16; 8 is the value their default build actually uses.
    static const uint8_t  AGGR_PKT_LIMIT  = 8;
    // 8 KB per direction: five 1.5 KB frames, or the packet limit's worth of
    // small ones, whichever comes first.
    static const uint16_t AGGR_BUF_BLOCKS = 32;

    // RX aggregation: ★ DEFAULT OFF, because on silicon it COSTS 4.6x
    // THROUGHPUT.  Measured on the house AP, one association, controls either
    // side, isolated by bisecting the three W16 switches one at a time:
    //
    //   all off (pre-W16)        tcp-rx 10.96 Mbps
    //   register port only              11.03      <- harmless
    //   + RX aggregation                 2.39      <- collapse
    //   + TX aggregation                 2.45
    //   + interrupt mode                 2.44
    //   back to pre-W16                 10.49      <- the air was fine
    //
    // AND IT IS NOT THE AGGREGATION FAILING -- it works exactly as designed:
    // 0.43 data CMD53s per frame, 86% of frames arriving batched.  The cost is
    // somewhere else, and the counters name it: rxStrandedRecovered() goes
    // from 26 to 139 over a 10 s blast (23x per frame).  Each of those is an
    // upload the safety net had to rescue at up to 64 ms of latency, and 139
    // of them do not fit in ten seconds without destroying the link.
    //
    // MECHANISM, stated as the leading hypothesis and not as a finding: a
    // multi-slot read holds the bus and the CPU for milliseconds (eight
    // 1.5 KB slots is ~12 KB in one PIO transfer), and the drain works from a
    // SNAPSHOT taken before it.  Uploads the card queues during that window
    // are not in the snapshot, so the drain exits believing the ring is empty
    // and the pass then clears HOST_INT_UP_LD -- stranding them until the net
    // fires.  The bigger the batch, the longer the window.  The likely fix is
    // to stop clearing that bit on the strength of a stale snapshot: take a
    // FRESH register read at the end of the drain and clear only if the ring
    // is really empty.  Not attempted yet; measured first, as W11 should have
    // been.
    //
    // ★ THIS IS W11 HAPPENING AGAIN, and the shape is worth recognising: a
    // change that provably cuts bus commands, whose own counters look
    // excellent, and which costs throughput anyway because of how it perturbs
    // the firmware's timing.  W11's bitmap cache cut reads and cost 2.5x by
    // breaking pacing; this cuts reads and costs 4.6x by widening a race.
    // Bus-command counters are necessary and they are not sufficient.
    void setRxAggregation(bool on) { m_rxAggr = on; }
    bool rxAggregation() const     { return m_rxAggr; }

    // TX aggregation: OFF by default, and that is deliberate.  It changes what
    // sendDataFrame() MEANS -- the frame is queued, not on the wire -- and a
    // caller that sends one frame and then waits for a reply without ever
    // calling flushTx() would wait forever.  Every caller in this tree that
    // has a poll loop (the lwip netif) turns it on and flushes each iteration;
    // the probe and monitor examples, which send a frame and expect it gone,
    // are untouched by default.
    void setTxAggregation(bool on) { m_txAggr = on; }
    bool txAggregation() const     { return m_txAggr; }
    // Write whatever sendDataFrame() has accumulated, as ONE CMD53.  A no-op
    // (and OK) when nothing is queued, so a poll loop can call it every
    // iteration unconditionally.  MUST be called by any caller that enables TX
    // aggregation, or frames sit in the buffer.
    SdioHost::Status flushTx();
    uint8_t txQueued() const { return m_txAggrCount; }

    // Data CMD53s split by direction.  cmd53Count() stays the sum, so every
    // existing reader keeps working; the split is what makes "data CMD53s per
    // RX frame" measurable in an example that also transmits.
    uint32_t cmd53Rx() const { return m_cmd53Rx; }
    uint32_t cmd53Tx() const { return m_cmd53Tx; }
    // Slots carried by CMD53s that carried more than one, per direction.  The
    // un-fakeable "aggregation actually happened" pair: a driver that reads or
    // writes one slot per command cannot make these non-zero however many
    // frames it moves, and however fast.
    // Packets a batch split had to skip because the SDIOPkt size disagreed with
    // the slot length the card published for it.  Zero is the expected value;
    // non-zero means the two ends of the block-padding inference (model NOTE 11
    // -- RD_LEN is published UNPADDED, which is inferred, not measured) do not
    // agree, and the frames behind the bad one were still delivered because the
    // walk steps by RD_LEN rather than by the payload.
    uint32_t rxSplitMismatch() const { return m_rxSplitMismatch; }
    uint32_t rxAggrBatches() const { return m_rxAggrBatches; }
    uint32_t rxAggrSlots()   const { return m_rxAggrSlots; }
    uint32_t txAggrBatches() const { return m_txAggrBatches; }
    uint32_t txAggrSlots()   const { return m_txAggrSlots; }
    // RECONFIGURE_TX_BUFF (0x00D9): tell the firmware the host's per-port TX
    // buffer size.  NXP's wlan_fw_init_cfg() sends this UNCONDITIONALLY between
    // GET_HW_SPEC and MAC_CONTROL ("firmware looses alignment of SDIO Tx
    // buffers" without it) -- and W7 measured exactly that: without this
    // command every data write is accepted on the wire but never consumed, so
    // each send permanently eats one WR_BITMAP port until the bitmap is 0.
    SdioHost::Status reconfigureTxBuffers(uint16_t bufSize = 2048);
    // 11N_CFG (0x00CD): HT TX capability, exactly NXP's wlan_set_11n_cfg
    // values (GREENFIELD|SGI20|SGI40, RIFS, band both).  Part of the
    // unconditional init sequence between MAC_CONTROL and first traffic.
    SdioHost::Status set11nCfg();
    // AMSDU_AGGR_CTRL (0x00DF): enable A-MSDU aggregation handling, NXP's
    // wlan_enable_amsdu (SET, enable=1).
    SdioHost::Status amsduAggrCtrl();
    uint32_t lastWrBitmap() const { return m_lastWrBitmap; }
    // Frames the card has actually been given.  ★ W16: incremented at FLUSH,
    // not at sendDataFrame() -- with TX aggregation on, a successful
    // sendDataFrame() means the frame is STAGED, and counting it there would
    // make this number describe intent rather than bus traffic.  It is
    // therefore the honest partner of cmd53Tx(): frames sent, over commands
    // used.
    uint32_t dataTxCount() const { return m_dataTxCount; }
    // Ring positions, for the probe's report: which TX/RX port the driver will
    // use next.  Both reset to 0 when the firmware is (re)downloaded.
    uint8_t  txPort() const { return m_txPort; }
    uint8_t  rxPort() const { return m_rxPort; }
    // Times the RX ring position disagreed with the bitmap and was resynced to
    // the lowest set bit.  Zero is the expected value; non-zero says the ring
    // model missed something and the resync kept data flowing anyway.
    uint16_t rxRingResyncs() const { return m_rxRingResyncs; }

    // W12 fault #5 signature.  Times serviceLink()'s RD-bitmap safety net
    // found an upload waiting AT OUR OWN RING SLOT that no interrupt had told
    // it about, and drained it.  **Non-zero means an upload interrupt was
    // lost** -- some path consumed the clear-on-read HOST_INT_STATUS bit
    // without servicing the upload -- **and the net caught it.**  A healthy
    // system reads 0.  Read it alongside rxDrainErrors() and rxSlotNotReady()
    // below, which between them say WHICH loss path produced it.
    // W13: the DESYNC variant (bits set, but not at m_rxPort) is counted
    // separately in rxDesyncRecovered() -- the two mechanisms are different
    // faults and collapsing them into one number hides which one is live.
    // Reset when the firmware is (re)downloaded, like the other per-firmware-
    // life counters.
    uint32_t rxStrandedRecovered() const { return m_rxStrandedRecovered; }

    // W13, residual #2.  Times the safety net drained an upload the card was
    // offering at a slot OTHER than m_rxPort -- i.e. an interrupt was lost AND
    // the host's ring position had desynced from the firmware's.  Nothing else
    // in the driver recovers that combination, so before W13 it read as RX
    // permanently dead with rxStrandedRecovered() stuck at 0.
    //
    // ★ The net enters this path only on a POSITIVE check: the candidate slot
    // (lowest set rd-bitmap bit) must publish a NON-ZERO RD_LEN.  A set bitmap
    // bit does NOT imply a packet is waiting on this firmware -- measured on
    // silicon, a net widened to plain `bm != 0` produced ~6100 ring resyncs
    // over 110 blasts that found no data at all, walking m_rxPort backwards
    // over stale bits each time.  Never re-widen it; if this counter needs to
    // fire more often, make the positive check better, not looser.
    // Reset per firmware life, like rxStrandedRecovered().
    uint32_t rxDesyncRecovered() const { return m_rxDesyncRecovered; }

    // Times serviceLink()'s ring drain exited early because readRingPacket()
    // returned something other than OK/CMD_TIMEOUT (a CMD52/CMD53 bus error, or
    // an oversized packet that was consumed but not deliverable).  That exit is
    // DELIBERATE -- re-entering a failing read on every pass would spin the
    // poll at full speed -- and it falls straight into the by-design
    // HOST_INT_UP_LD clear, so each occurrence can strand whatever is left in
    // the ring until Layer 2's next check.
    //
    // READ IT AGAINST rxStrandedRecovered():
    //   drainErrors tracks stranded ~one-for-one  => the residual strands ARE
    //       these by-design error exits; Layer 1 has no remaining hole and the
    //       thing to chase is the bus error, not the interrupt bookkeeping.
    //   stranded MUCH GREATER than drainErrors    => a genuine lost-interrupt
    //       path still exists; hunt for a HOST_INT_STATUS reader that consumes
    //       a bit without servicing it (the fault class has now bitten twice).
    // MEASURED (W12 soaks): stranded 3-7 per ~100 blasts with drainErrors == 0
    // -- so the by-design explanation above is REFUTED and the second branch
    // is the live one.  rxSlotNotReady() below is W13's candidate for it.
    // Reset per firmware life, like rxStrandedRecovered().
    uint32_t rxDrainErrors() const { return m_rxDrainErrors; }

    // W13 DIAGNOSTIC -- the hypothesis for the residual rxStrandedRecovered()
    // that rxDrainErrors() == 0 refuted the old explanation for.
    //
    // Times readRingPacket() found a set rd-bitmap bit for the slot it was
    // about to read (m_rxPort's own, or the slot it resynced to) and then read
    // RD_LEN_P<slot> == 0 -- "the card is offering this slot but has not
    // published a length for it".  readRingPacket reports that as CMD_TIMEOUT,
    // which is the SAME status it uses for "the ring is genuinely empty", and
    // serviceLink()'s drain loop therefore treats it as a completed drain and
    // falls into the deliberate HOST_INT_UP_LD clear.  If the card really did
    // still hold an upload (a stale bit alongside a real one, or a length
    // published a few microseconds late), that upload is now stranded with no
    // interrupt and no bus error -- exactly the observed signature.
    //
    // HOW TO READ IT, on the next silicon soak:
    //   slotNotReady == 0 while stranded > 0  => hypothesis REFUTED; the loss
    //       is somewhere else entirely and this counter has done its job.
    //   slotNotReady >= stranded              => hypothesis SUPPORTED (it is an
    //       upper bound: only the false-empties reached from a genuine
    //       HOST_INT_UP_LD drain can strand, the ones reached from the net
    //       cannot -- the net's clear is scoped to a bit that was never set).
    //       The fix is then to stop conflating the two CMD_TIMEOUTs: hold
    //       HOST_INT_UP_LD across a BOUNDED number of extra passes when the
    //       drain ended on a false empty rather than on an empty bitmap.  It
    //       must be bounded -- serviceLink's delay(1) is gated on those same
    //       bits, so an unbounded hold spins the poll at full speed.
    // Deliberately NOT acted on yet: this counter exists so the next soak can
    // confirm or kill the hypothesis before anything on the hot RX path
    // changes.  That is how the previous explanation got refuted.
    // Reset per firmware life, like rxStrandedRecovered().
    uint32_t rxSlotNotReady() const { return m_rxSlotNotReady; }

    // DIAGNOSTIC ONLY (W12): read the card's 32-bit RX (upload) bitmap right
    // now and return it.  Answers the one question a frozen RX path cannot
    // otherwise answer -- is the FIRMWARE still offering uploads (bits set that
    // we are not draining => our ring position has desynced) or has it stopped
    // uploading altogether (bitmap empty => the fault is card-side)?
    // On a bus error it returns 0 and, if `ok` is non-null, sets *ok=false.
    // PASS `ok` WHENEVER THE ANSWER MATTERS: a bare 0 is ambiguous between
    // "genuinely empty" (card-side fault) and "the probe could not read the
    // card at all" -- and telling those apart is the entire point of the
    // freeze dump.  *ok is set on every path, so it needs no pre-initialising.
    // Safe to call at any time: the RD bitmap registers are plain reads.
    // It deliberately does NOT touch HOST_INT_STATUS, which is CLEAR-ON-READ
    // -- a diagnostic read of that register would steal upload/event bits
    // from serviceLink() and cause the very stall it was called to explain.
    // Costs 4 CMD52s, counted into cmd52PollsSvc() like every other read.
    uint32_t probeRdBitmap(bool *ok = nullptr);

    // One connected-link service poll, servicing BOTH ports from a single
    // HOST_INT_STATUS read (the register is clear-on-read, so split polling
    // loses bits -- same rationale as diagConnect).  Data packets: the first
    // 802.3 frame seen is copied to frameBuf/*frameLen (later frames in the
    // same pass are read and counted in rxDropped()).  Command-port events:
    // recorded in lastEvent()/lastEventInfo(); a DEAUTHENTICATED/DISASSOCIATED
    // sets *dropped.  Returns OK as soon as any data frame was seen (the
    // first is copied out if it fits), or a drop is captured; CMD_TIMEOUT
    // when waitMs passed quietly (not an error), else a bus error.
    SdioHost::Status pollLink(uint8_t *frameBuf, uint16_t bufCap, uint16_t *frameLen,
                              bool *dropped, uint32_t waitMs);

    // Stack-facing service pass (W9).  One HOST_INT_STATUS read; EVERY
    // pending data frame is unwrapped (RxPD) and handed to `sink`;
    // command-port events are recorded and a DEAUTH/DISASSOC/LINK_LOST sets
    // *dropped.  Returns OK if any frame or a drop was seen, CMD_TIMEOUT for
    // a quiet pass (not an error), else the bus error.  pollLink() is this
    // with a copy-first-frame sink.
    //
    // W12: the bits it works from are `fresh read | m_intPending`, and the
    // early return on a frame/drop is safe precisely because of that -- any
    // condition this pass did not finish servicing stays set in m_intPending
    // and is picked up by the next pass.  Backstopping that, every
    // RX_BITMAP_CHECK_PASSES quiet passes it verifies the RD bitmap directly
    // and drains if the card is holding an upload no interrupt ever announced.
    // Two cases, counted apart because they are different faults:
    //   - the bit is at m_rxPort            -> rxStrandedRecovered()
    //   - the bit is elsewhere (ring desync) -> rxDesyncRecovered(), and only
    //     after RD_LEN for that slot reads NON-ZERO.  A set bitmap bit alone
    //     is not evidence on this firmware; see rxDesyncRecovered().
    //
    // FrameSink contract: `frame` aliases serviceLink's internal static RX
    // staging buffer and is valid ONLY for the duration of the callback --
    // copy synchronously out of it, never retain the pointer.  Calling
    // TX-side driver methods (e.g. sendDataFrame) from inside the sink is
    // safe (it has its own staging and only touches the TX ring).  Calling
    // ANY RX-path method (pollLink, serviceLink, readDataPacket,
    // captureMonitor) from the sink is forbidden -- it would reuse this
    // same static buffer mid-drain and corrupt the ring bookkeeping.  The
    // same forbidden-list applies to command-path methods that read the
    // command port (setIeeePs, waitCmdResp, and anything else that calls
    // it, e.g. scan/associate/getHwSpec) -- serviceLink already took its own
    // HOST_INT_STATUS sample for this pass, so a command-port read from
    // inside the sink races against that stale sample and can steal or
    // misparse the reply serviceLink itself is mid-handling.
    typedef void (*FrameSink)(void *ctx, const uint8_t *frame, uint16_t len);
    SdioHost::Status serviceLink(FrameSink sink, void *ctx, bool *dropped,
                                 uint32_t waitMs);

    uint32_t rxDropped()   const { return m_rxDropped; }
    uint32_t rxDataCount() const { return m_rxDataCount; }

    struct MonitorFrame {
        uint16_t frameControl;   // little-endian; 0x80 = beacon
        uint8_t  rssi;           // dBm = -rssi (snr - nf from the RxPD)
        uint8_t  channel;        // the monitored channel
        uint8_t  ta[6];          // transmitter address (802.11 addr2)
        uint8_t  bssid[6];       // addr3; meaningful for beacon/probe-resp
        char     ssid[33];       // from IE 0 on beacon/probe-resp; else empty
        uint16_t len;            // 802.11 frame length
    };
    // Enable monitor on `channel`, capture 802.11 frames for windowMs, disable.
    // Fills up to maxOut; *count is what fit, framesSeen() is the total.
    SdioHost::Status captureMonitor(MonitorFrame *out, uint8_t maxOut, uint8_t *count,
                                    uint32_t windowMs, uint8_t channel);
    uint16_t framesSeen() const { return m_framesSeen; }
    uint16_t lastMonResult() const { return m_lastRespResult; }
    // Capture diagnostics, for locating a frames=0 result: how many DATA
    // UP_LD interrupts fired, how many packets were CMD53-read, the OR of all
    // RD_BITMAP / HOST_INT_STATUS / RxPD-rx_pkt_type / SDIOPkt-pkttype values
    // seen.  uploads=0 -> firmware uploaded nothing; reads>0 with rxtype_or
    // lacking bit for 0x05 -> frames arrive but not as PKT_TYPE_802DOT11.
    uint16_t dbgUploads()    const { return m_dbgUploads; }
    uint16_t dbgReads()      const { return m_dbgReads; }
    uint32_t dbgBitmapOr()   const { return m_dbgBitmapOr; }
    uint8_t  dbgStatusOr()   const { return m_dbgStatusOr; }
    uint16_t dbgRxTypeOr()   const { return m_dbgRxTypeOr; }
    uint16_t dbgLastPktType() const { return m_dbgLastPktType; }

    // Header fields of the last command-port packet seen by waitCmdResp:
    // pkttype (1=cmd resp, 3=event), the command/event id, and the result.
    uint16_t lastRespType()   const { return m_lastRespType; }
    uint16_t lastRespCmd()    const { return m_lastRespCmd; }
    uint16_t lastRespResult() const { return m_lastRespResult; }
    // Times waitCmdResp saw a cmd-id match (pkttype+cmd|RET_BIT) whose
    // seq_num did NOT match the request just sent, and so kept waiting
    // instead of accepting it -- e.g. a stale sleep-confirm ack.  Zero across
    // a soak means the firmware echoes seq_num faithfully; a nonzero count on
    // every call would mean it doesn't, and the seq check should be reverted
    // to the narrower action-based exclusion (see waitCmdResp).
    uint32_t seqMismatches() const { return m_seqMismatches; }

    // Evidence from the response path, for the probe's report: every
    // HOST_INT_STATUS bit ever observed (ORed -- reads clear the register), and
    // the last CMD_RD_LEN value.  A timeout with int_seen=0 says the card never
    // flagged anything; int_seen with bit 6 clear says it flagged the WRONG
    // thing; a bad cmd_rd_len says the flag came but the length did not.
    uint8_t  intStatusSeen() const { return m_intSeen; }
    uint16_t lastCmdRdLen()  const { return m_lastRdLen; }
    // Evidence from begin(): the fw-status scratch BEFORE the config writes,
    // and each config register before/after its read-modify-write, in write
    // order: CARD_CONFIG_2_1, CMD_CONFIG_0, CMD_CONFIG_1, HOST_INT_RSR,
    // CARD_MISC_CFG.  A register whose post value lacks the requested bits
    // did not take the write.
    uint16_t fwStatusPre() const { return m_fwStatusPre; }
    const uint8_t *cfgPre()  const { return m_cfgPre; }
    const uint8_t *cfgPost() const { return m_cfgPost; }
    static const int CFG_REG_COUNT = 5;

    // FUNC_INIT then GET_HW_SPEC.  Fills in the card's burned-in MAC and the
    // running firmware's release number -- neither of which this code knows.
    SdioHost::Status getHwSpec(uint8_t mac[6], uint32_t *fwRelease, uint16_t *hwVersion);

    // Re-read the I/O port after firmware boot; the bootloader's value need not
    // survive the jump into the image.
    SdioHost::Status refreshIoPort();

    SdioHost::Status readFwStatus(uint16_t *out);
    SdioHost::Status readCardStatus(uint8_t *out);
    SdioHost::Status readRequestedLen(uint16_t *out);

private:
    // pollLink's compatibility sink: reproduces the historical contract
    // (first frame that fits is copied out, every other delivered frame --
    // including a first frame that didn't fit -- is counted in
    // rxDropped()) on top of serviceLink().  `self` lets the static member
    // reach the instance's private m_rxDropped.
    struct PollLinkCtx {
        uint8_t  *buf;
        uint16_t  cap;
        uint16_t *lenOut;
        bool      got;
        Iw416    *self;
    };
    static void pollLinkSink(void *ctx, const uint8_t *frame, uint16_t len);

    SdioHost::Status setCardBits(uint32_t reg, uint8_t bits,
                                 uint8_t *preOut, uint8_t *postOut);
    // Fill an MpRegs the slow way, one CMD52 at a time: HOST_INT_STATUS, both
    // bitmaps, CMD_RD_LEN and only the per-slot RD_LEN the caller can reach.
    // This is the pre-W16 traffic, kept as the fallback for a card whose
    // register port does not behave as NXP's driver implies (mpRegsUsable()).
    SdioHost::Status readMpRegsCmd52(MpRegs *out, bool txPath);
    // THE one place HOST_INT_STATUS bits enter the sticky accumulator (W12
    // Layer 1).  Every reader of that clear-on-read register calls this with
    // what it read; the masking rule (only the two conditions anything here
    // services) lives here rather than being restated at each site.
    //
    // It also counts UP_LD ARRIVALS, and that count is load-bearing.
    // serviceLink() clears HOST_INT_UP_LD after its drain, scoped to the
    // snapshot it entered the drain with -- but "scoped to the snapshot" only
    // decides WHETHER to clear, not WHICH arrival is being cleared.  Since W16
    // the TX path reads the register too (sendDataFrame -> readMpRegs), and
    // the FrameSink contract explicitly allows sendDataFrame from inside the
    // drain, so a NEW upload's UP_LD can be latched mid-drain and then be
    // wiped by a clear that was authorised by the OLD one.  That is W12 fault
    // #5 rebuilt in a new place: the frame would sit until the 64 ms safety
    // net, inflating rxStrandedRecovered() -- the very counter this tree reads
    // to decide whether that fault class is back.  Comparing this counter
    // across the drain is what tells the two apart.
    void latchIntBits(uint8_t st);
    // The 32-bit WR bitmap as four CMD52s (the fallback's TX half).  `txPath`
    // decides which poll counter the four reads land in -- without it the
    // service path's own fallback reads were being attributed to
    // cmd52PollsTx(), which on the first silicon run made an idle link look
    // like it was doing 191,607 CMD52s of TX polling for three transmitted
    // frames.  Attribution that lies is worse than no attribution: W11 pinned
    // a 2.5x regression on the TX path using exactly this split.
    SdioHost::Status readWrBitmap32(uint32_t *out, bool txPath);
    // The 32-bit RD bitmap as four CMD52s.  Kept -- rather than replaced by
    // readMpRegs() -- because probeRdBitmap()'s whole contract is that it does
    // NOT touch the clear-on-read HOST_INT_STATUS (a diagnostic that steals
    // interrupt bits causes the stall it was called to explain), and the
    // register port necessarily does.
    SdioHost::Status readRdBitmap32(uint32_t *out);
    // Read the upload waiting at the RX ring position, if any: check the
    // rd-bitmap bit at m_rxPort, read RD_LEN_P<m_rxPort>, CMD53-read the
    // packet (into an internal scratch so an oversized packet still CONSUMES
    // its ring slot -- skipping a port wedges the ring), advance the ring,
    // copy out.  CMD_TIMEOUT = nothing at the ring position (not an error).
    // BAD_CIS = packet read and dropped (did not fit bufCap).
    // W16: `regs`, when non-null, supplies the rd-bitmap and the per-slot
    // length from a register-port snapshot the caller already took this pass,
    // instead of the four + two CMD52s this used to issue per packet.  The
    // caller's copy is MUTATED: the consumed slot's bit is cleared, which is
    // what lets a multi-packet drain walk the ring from one snapshot without
    // re-reading the same slot forever (NXP's mp_rd_bitmap is maintained the
    // same way).  Passing nullptr keeps the original CMD52 path, which is what
    // the diagnostic readers (readDataPacket, captureMonitor) still use --
    // they read ONE packet and hold no snapshot.
    SdioHost::Status readRingPacket(uint8_t *buf, uint16_t bufCap, uint16_t *outLen,
                                    uint8_t *portOut, MpRegs *regs = nullptr);
    // W16: THE ring read.  Walks CONSECUTIVE occupied slots from m_rxPort and
    // pulls them all in one CMD53 at the aggregated multiport address; `buf`
    // receives the slots' packets back to back, each padded to a 256-byte
    // boundary, exactly as the card lays them out.  *outLen is the total
    // padded length, *slotsOut how many slots it covers, *startOut the first
    // slot (which may differ from the m_rxPort the caller saw, because the
    // resync lives in here).
    //
    // maxSlots == 1 reproduces the pre-W16 one-packet-per-CMD53 behaviour
    // EXACTLY, and readRingPacket() is now a thin wrapper around that case.
    // One ring model, one resync, one positive-length check: two
    // implementations of this walk is how W8 and W12 each stayed hidden for
    // days, so the aggregating and non-aggregating paths are the same code
    // with a different bound.
    //
    // It never WRAPS past slot 31 inside one CMD53.  NXP's span test allows a
    // wrapped aggregate of up to 16 ports, but no capture in this tree has
    // ever shown one, and a wrapped transfer is the one shape where a wrong
    // port_count encoding would be invisible in emulation.  Stopping at the
    // ring top costs at most one extra CMD53 per lap of the ring.
    // `lensOut` receives the CARD-PUBLISHED length of each slot in the run, in
    // run order.  It is not a convenience: it is the authoritative way to split
    // the batch afterwards.  A caller that instead walked the buffer by each
    // packet's own SDIOPkt size would be re-deriving the boundaries from the
    // payload, and any slot where the two round differently desynchronises the
    // walk for EVERY packet behind it -- losing the rest of a batch whose slots
    // have already been consumed, silently, with nothing counted.  RD_LEN is
    // what sized the transfer, so RD_LEN is what must step through it.
    SdioHost::Status readRingBatch(MpRegs *regs, uint8_t *buf, uint32_t bufCap,
                                   uint32_t *outLen, uint8_t *slotsOut,
                                   uint8_t *startOut, uint16_t *lensOut,
                                   uint8_t maxSlots);
    // Length the card has published for data port `port`, in bytes.
    //
    // ★ REGISTER LAYOUT (W13): RD_LEN is PER SLOT, not one shared register --
    // 32 little-endian pairs at RD_LEN_P0_L_REG + (port << 1), i.e. 0x18/0x19
    // for port 0 through 0x56/0x57 for port 31 (NXP's mlan_sdio_defs.h reads
    // it the same way: SDIO_RD_LEN_P0_L + (port << 1)).  Nothing has to be
    // "selected" first and the read has no side effects -- it is a plain
    // CMD52 pair -- so ANY slot's length can be probed at any time, which is
    // what makes the safety net's positive desync check possible.  (Contrast
    // CMD_RD_LEN_0/1, which is the single COMMAND-port length register and
    // has no per-slot form.)  Zero means "the card has not published a
    // length for that slot" -- notably NOT the same thing as "that slot's
    // rd-bitmap bit is clear"; see rxSlotNotReady().
    // Counts its 2 CMD52s into m_cmd52PollsSvc, like every other RX-side read.
    SdioHost::Status readRdLenPort(uint8_t port, uint16_t *out);
    // Normalise a beacon RSN IE into the association-request form (single
    // cipher/AKM, PMF caps forced to MFPC=1/MFPR=0), per wlan_update_rsn_ie.
    uint8_t buildAssocRsnIe(const uint8_t *beacon, uint8_t beaconLen,
                            uint8_t *out, uint8_t outCap);
    // Wake a sleeping card before host-initiated traffic: CMD52-write
    // HOST_POWER_UP to fn1 reg 0x00 (NXP wifi.c wifi_wake_up_card), then give
    // the fw a moment.  The PS_AWAKE event arrives through the normal event
    // path; we do NOT drain events here (re-entrancy: this is called from
    // inside send paths).  No retry is done HERE -- the actual mitigations
    // live one level out: sendDataFrame's WR-bitmap wait already polls (and
    // so absorbs wake latency) before its CMD53 write ever happens, and
    // sendHostCmd's CMD53 write surfaces a bus error/CMD_TIMEOUT to callers,
    // all of which already handle a failed/timed-out command.
    void wakeCardIfSleeping();
    // Sent in direct answer to EVENT_PS_SLEEP.  Body: action=SLEEP_CONFIRM,
    // resp_ctrl=1 (RESP_NEEDED).  The fw acks with a 0x80E4/action-5 response
    // on the command port and THEN sleeps; the ack flows through the normal
    // demux (it is not a tracked command, so waitCmdResp is not used here).
    // Calls sendHostCmd(), which has its own wake gate -- that gate is not
    // expected to fire here, since the firmware only ever raises
    // EVENT_PS_SLEEP while the card is awake, so m_psState is still PS_AWAKE
    // at the moment this runs (state flips to PS_SLEEPING only after the
    // send below).  Even if that assumption were ever wrong, the cost is
    // small: a duplicate queued sleep event just costs one harmless extra
    // wake write plus one extra confirm, not a correctness bug.
    void sendSleepConfirm();

    SdioHost &m_host;
    SdioFunc &m_func;
    uint8_t  m_intSeen      = 0;
    // W12 fault #5, LAYER 1.  HOST_INT_STATUS is CLEAR-ON-READ, and five call
    // sites read it (readHostResp, readDataPacket, diagConnect, watchConnect,
    // serviceLink).  Four of them used to look only at the bit they cared
    // about and DISCARD the rest -- so a data upload whose HOST_INT_UP_LD
    // landed inside, say, readHostResp's 1 ms command poll had its interrupt
    // silently eaten.  The firmware does not re-raise an interrupt for an
    // upload it has already offered, so the frame sat in the ring forever and
    // host-visible RX was dead until reflash.  Captured on silicon:
    //   FREEZE rd_bitmap=0x20000 wr_bitmap=0xFFFFF800 ring=12/17/339
    //          c53=26493 rx_data=17616 rx_drop=0
    // -- port 17 pending, m_rxPort ALSO 17 (the ring model was right), yet the
    // data CMD53 count frozen: nobody ever told serviceLink to look.
    // m_intPending is the fix: EVERY read site ORs the bits it just took out
    // of the register into here, and only the code that actually FINISHES
    // servicing a condition clears that condition's bit.  Distinct from
    // m_intSeen, which is a write-only diagnostic union (never consumed).
    //
    // MASKED to (HOST_INT_UP_LD | CMD_PORT_UPLD) at every read site: those are
    // the only conditions any code path services, so they are the only ones
    // that can ever be cleared.  Latching HOST_INT_DN_LD/CMD_PORT_DNLD instead
    // pins this field non-zero from the first download onwards and makes its
    // documented meaning ("work nobody has finished") false.  Use m_intSeen for
    // the raw union -- that is what it is for.
    //
    // W13, THE OTHER HALF OF LAYER 1.  Accumulating is only half a fix: a site
    // that ORs its read in here and then branches on its OWN fresh sample can
    // still sit out a condition that is already flagged, because the bit was
    // taken out of the clear-on-read register by somebody else and the
    // firmware never re-raises it.  readHostResp() had exactly that shape and
    // would time out with the answer already pending.  So:
    //   ** A SITE MUST BRANCH ON m_intPending, NOT ON ITS OWN READ -- for
    //      every bit it also CLEARS when it services that bit. **
    // The clear is what makes consulting the accumulator terminate.  That is
    // why readDataPacket() is the one site still branching on its fresh read:
    // it deliberately clears nothing (it reads ONE ring packet, it does not
    // drain the ring), so consulting a bit it can never clear would make its
    // poll loop fire on every iteration for the rest of the firmware's life.
    uint8_t  m_intPending   = 0;
    uint16_t m_lastRdLen    = 0;
    uint16_t m_fwStatusPre  = 0;
    uint16_t m_lastRespType   = 0;
    uint16_t m_lastRespCmd    = 0;
    uint16_t m_lastRespResult = 0;
    const uint8_t *m_uapCfgReq = nullptr;
    uint16_t m_uapCfgReqLen = 0;
    uint8_t  m_scanSets       = 0;
    uint16_t m_assocStatus    = 0xFFFF;
    uint16_t m_assocCapInfo   = 0;
    uint8_t  m_assocRsn[24]   = {0};
    uint8_t  m_assocRsnLen    = 0;
    ScanResult m_connectedAp  = {};
    uint32_t m_lastEvent      = 0;
    uint32_t m_lastEventInfo  = 0;
    // W10: IEEE power-save state.
    bool     m_psEnabled = false;
    PsState  m_psState   = PS_AWAKE;
    uint32_t m_psSleeps  = 0;
    uint32_t m_psWakes   = 0;
    uint32_t m_psConfirmFails = 0;   // sendSleepConfirm's sendHostCmd failed
    uint32_t m_psHostWakes    = 0;   // wakeCardIfSleeping actually wrote HOST_POWER_UP
    // W11: bus-attribution counters (throughput work) -- per-frame SDIO bus
    // cost, split by call path.  Reset (zeroed) in downloadFirmware()'s
    // prologue alongside m_txPort/m_rxPort/m_bytesSent: these five attribute
    // the cost of a single firmware life, so a mid-soak recovery re-download
    // must not carry the previous life's traffic into the new one.  The ps*
    // counters deliberately do NOT reset there -- they stay cumulative across
    // firmware downloads, as landed in W10.  Attribution rules:
    //  - wakeCardIfSleeping()'s CMD52 write, and serviceLink()'s EVENT_PS_AWAKE
    //    wake-latch clear write (the cmd52Write(1, 0x00, 0x00) at the end of
    //    the PS_AWAKE handler), are deliberately NOT counted in either
    //    cmd52Polls* counter -- both are writes, not polls, and both already
    //    pair 1:1 with an existing PS counter (psHostWakes() and psWakes()
    //    respectively), so counting them here would double up PS-wake
    //    accounting rather than add new information.
    //  - m_cmd52PollsTx counted readWrBitmap32()'s CMD52 reads, issued solely
    //    from sendDataFrame's wr_bitmap wait loop.  ★ SINCE W16 IT READS 0 BY
    //    CONSTRUCTION: that wait takes the wr-bitmap out of the register-port
    //    snapshot instead, and readWrBitmap32() no longer exists.  Its
    //    successor is cmd53RegsTx().  The accessor is kept because examples
    //    and the soak harness print it and a silently vanishing field is worse
    //    than a documented zero -- and because it is the honest way to show
    //    that the TX-side CMD52 polling really did go to nothing rather than
    //    moving somewhere unlabelled.
    //  - m_cmd52PollsSvc counted serviceLink()'s own HOST_INT_STATUS and
    //    CMD_RD_LEN_0/1 reads, plus readRdBitmap32()/readRingBatch()'s CMD52
    //    reads.  ★ SINCE W16 the first three of those come out of the
    //    register-port snapshot instead and are counted in cmd53RegsSvc(); what
    //    still lands here is probeRdBitmap()'s four CMD52s (the freeze
    //    diagnostic, which must not touch HOST_INT_STATUS and so cannot use the
    //    snapshot) and the CMD52 fallback path (see mpRegsUsable()).  The latter two are also reached from diagConnect(),
    //    watchConnect() and readDataPacket() (connection-setup/monitor
    //    diagnostics, not serviceLink) -- splitting that by caller would need
    //    an extra parameter threaded through a hot RX path for no throughput
    //    benefit, so these are counted at the CMD52 call site instead of by
    //    call path; m_cmd52PollsSvc is therefore "RX/service-side CMD52
    //    polling", not strictly "polls issued while inside serviceLink()".
    //  - m_cmd53Count/m_cmd53Bytes count only DATA-port CMD53s: sendDataFrame's
    //    TX write and readRingPacket's RX read (both directions of actual
    //    frame traffic).  Command/event-port CMD53s (sendHostCmd,
    //    serviceLink's own command-port read, readHostResp, ...) are out of
    //    scope -- they carry no data-plane payload.  Both sites count only on
    //    a successful CMD53 (see the call sites' comments).  m_cmd53Bytes is
    //    the block-padded transfer length successfully issued on the bus
    //    (blockSize * blocks), not the caller's unpadded frame/packet length
    //    -- and being a plain uint32 counting bytes, it wraps in roughly 2
    //    hours at full rate, so a soak must consume it as deltas, not a
    //    running total.
    //  - m_cmd53ByteMode documents an invariant, not a live detector: it is 0
    //    by construction because SdioHost::cmd53() hardcodes block mode
    //    (cmd53Arg's blockMode argument is always `true` -- see
    //    sdio/SdioHost.cpp).  It does not sense or flag a future byte-mode
    //    code path; if SdioHost ever grows one, this counter also needs a
    //    real increment site, not just a read of a value that already exists.
    uint32_t m_cmd52PollsTx   = 0;
    uint32_t m_cmd52PollsSvc  = 0;
    // W15: interrupt-driven service.  m_intMode is the mode switch (see
    // setInterruptMode); m_cardInts counts DAT1 assertions serviceLink took
    // delivery of, and is per-firmware-life like the counters above.
    bool     m_intMode        = false;
    uint32_t m_cardInts       = 0;
    // W16: aggregation state.  m_txAggrBuf is uint32_t-backed because the
    // uSDHC PIO port is 32 bits wide and SdioHost casts the caller's pointer.
    bool     m_rxAggr         = false;
    bool     m_txAggr         = false;
    uint32_t m_txAggrBuf[(uint32_t)AGGR_BUF_BLOCKS * SDIO_BLOCK_SIZE / 4] = {0};
    uint32_t m_txAggrLen      = 0;   // padded bytes staged
    uint8_t  m_txAggrCount    = 0;   // frames staged
    uint8_t  m_txAggrStart    = 0;   // ring port the batch starts at
    uint32_t m_cmd53Rx        = 0;
    uint32_t m_cmd53Tx        = 0;
    uint32_t m_rxSplitMismatch = 0;
    uint32_t m_rxAggrBatches  = 0;
    uint32_t m_rxAggrSlots    = 0;
    uint32_t m_txAggrBatches  = 0;
    uint32_t m_txAggrSlots    = 0;
    uint32_t m_cmd53Bytes     = 0;
    uint32_t m_cmd53ByteMode  = 0;
    // W16: register-port CMD53s, and the word-aligned landing buffer for them
    // (the uSDHC data port is 32 bits wide, so the PIO loop casts this to
    // uint32_t*; a bare uint8_t array carries no such alignment guarantee).
    // Per-firmware-life like the other bus counters.
    uint32_t m_cmd53RegsTx    = 0;
    uint32_t m_cmd53RegsSvc   = 0;
    // W16 register-port health.  m_mpRegsOk starts true and can only go false;
    // m_mpRegsChecked makes the sanity check a once-per-firmware-life cost
    // rather than a per-read one.  Both reset with the firmware, because a
    // re-download re-runs begin()'s CMD52 read of CARD_STATUS and the question
    // is worth re-asking against the new life.
    bool     m_mpRegsOk       = true;
    bool     m_mpRegsChecked  = false;
    uint8_t  m_mpRegsBadStatus = 0;
    uint8_t  m_mpRegsSnap     = 0;   // register 0x5C as the register port saw it
    uint8_t  m_mpRegsLive     = 0;   // ... and as CMD52 saw it, same moment
    uint32_t m_mpRegsErrors   = 0;
    // Bumped by latchIntBits() whenever HOST_INT_UP_LD is newly latched; read
    // across serviceLink's drain.  Wraps harmlessly -- only equality matters.
    uint32_t m_intUpldLatches = 0;
    uint32_t m_mpRaw[(MP_REGS_LEN + 3) / 4] = {0};
    bool     m_diagEapol      = false;
    uint16_t m_diagDataFrames = 0;
    uint16_t m_diagFirstEthertype = 0;
    static const uint8_t EVENT_LOG_CAP = 24;
    uint32_t m_eventLog[EVENT_LOG_CAP]  = {0};   // event ids, in order seen
    uint32_t m_eventLogI[EVENT_LOG_CAP] = {0};   // the info word after each
    uint16_t m_eventLogT[EVENT_LOG_CAP] = {0};   // coarse ms since watch start
    uint8_t  m_eventLogLen = 0;
    bool     m_sawPortRelease = false;
    uint16_t m_framesSeen     = 0;
    uint32_t m_lastWrBitmap   = 0;
    uint32_t m_dataTxCount    = 0;
    uint32_t m_rxDropped      = 0;
    uint32_t m_rxDataCount    = 0;
    uint8_t  m_txPort         = 0;   // TX download-port ring position
    uint8_t  m_rxPort         = 0;   // RX upload-port ring position
    uint16_t m_rxRingResyncs  = 0;
    // W12 fault #5, LAYER 2 -- the self-healing net behind the sticky bits
    // above.  Layer 1 fixes the loss this tree knows about; this catches the
    // next one (the class has now bitten twice), so RX cannot be dead
    // FOREVER on a lost interrupt, only until the next check.
    // Interval: serviceLink paces a quiet pass at 1 ms (its delay(1)), so 64
    // quiet passes is ~64 ms of idle -- a recovery latency no application
    // notices, at 4 CMD52s per check, i.e. ~16 CMD52/s while idle and NONE
    // while traffic is flowing (the counter resets on every drain).  Against
    // the ~26k CMD53s of a live soak that is noise.  Deliberately NOT smaller:
    // this is a backstop, not a polling strategy -- if it ever runs often
    // enough to matter, rxStrandedRecovered() will say so.
    //
    // W15: in interrupt mode this same tick ALSO carries the HOST_INT_STATUS
    // read (see serviceLink), which makes it a slow poll of both ports rather
    // than of the ring alone.  That is deliberate: the data ring has a bitmap
    // to check, the COMMAND port has nothing of the kind, so without it a board
    // whose DAT1 never asserts would go deaf to deauth/PS events while RX kept
    // working -- a failure mode far nastier than the one it replaces.  Cost is
    // one extra CMD52 per 64 ms.
    static const uint16_t RX_BITMAP_CHECK_PASSES = 64;
    uint16_t m_svcQuietPasses     = 0;   // consecutive drainless serviceLink passes
    uint32_t m_rxStrandedRecovered = 0;  // net found an upload at m_rxPort, no interrupt
    uint32_t m_rxDesyncRecovered  = 0;   // ... at another slot, positively length-checked
    uint32_t m_rxDrainErrors      = 0;   // drain loop exits on a bus error (see accessor)
    uint32_t m_rxSlotNotReady     = 0;   // set bitmap bit with RD_LEN 0 (see accessor)
    uint16_t m_dbgUploads     = 0;
    uint16_t m_dbgReads       = 0;
    uint32_t m_dbgBitmapOr    = 0;
    uint8_t  m_dbgStatusOr    = 0;
    uint16_t m_dbgRxTypeOr    = 0;
    uint16_t m_dbgLastPktType = 0;
    uint8_t  m_cfgPre[CFG_REG_COUNT]  = {0};
    uint8_t  m_cfgPost[CFG_REG_COUNT] = {0};
    uint32_t m_ioPort       = 0;
    uint16_t m_fwStatus     = 0;
    uint8_t  m_cardStatus   = 0;
    uint16_t m_requestedLen = 0;
    uint32_t m_bytesSent    = 0;
    uint32_t m_chunksSent   = 0;
    uint16_t m_lastRequest  = 0;
    uint16_t m_seq          = 0;
    uint16_t m_lastSentSeq  = 0;   // seq_num of the last sendHostCmd() request
    uint32_t m_seqMismatches = 0;  // waitCmdResp: cmd-id matches with wrong seq
};
