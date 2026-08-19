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
    SdioHost::Status sendDataFrame(const uint8_t *frame, uint16_t frameLen,
                                   uint32_t timeoutMs = 1000);
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
    uint32_t dataTxCount() const { return m_dataTxCount; }
    // Ring positions, for the probe's report: which TX/RX port the driver will
    // use next.  Both reset to 0 when the firmware is (re)downloaded.
    uint8_t  txPort() const { return m_txPort; }
    uint8_t  rxPort() const { return m_rxPort; }
    // Times the RX ring position disagreed with the bitmap and was resynced to
    // the lowest set bit.  Zero is the expected value; non-zero says the ring
    // model missed something and the resync kept data flowing anyway.
    uint16_t rxRingResyncs() const { return m_rxRingResyncs; }

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
    // Full 32-bit multiport bitmaps (all four bytes each).
    SdioHost::Status readRdBitmap32(uint32_t *out);
    SdioHost::Status readWrBitmap32(uint32_t *out);
    // Read the upload waiting at the RX ring position, if any: check the
    // rd-bitmap bit at m_rxPort, read RD_LEN_P<m_rxPort>, CMD53-read the
    // packet (into an internal scratch so an oversized packet still CONSUMES
    // its ring slot -- skipping a port wedges the ring), advance the ring,
    // copy out.  CMD_TIMEOUT = nothing at the ring position (not an error).
    // BAD_CIS = packet read and dropped (did not fit bufCap).
    SdioHost::Status readRingPacket(uint8_t *buf, uint16_t bufCap, uint16_t *outLen,
                                    uint8_t *portOut);
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
    uint16_t m_lastRdLen    = 0;
    uint16_t m_fwStatusPre  = 0;
    uint16_t m_lastRespType   = 0;
    uint16_t m_lastRespCmd    = 0;
    uint16_t m_lastRespResult = 0;
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
