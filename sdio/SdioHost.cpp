#include "SdioHost.h"

// USDHC1 register overlay.  Offsets match the RT1176 USDHC register map; the
// core already carries USDHC1_* macros for the SdFat port, but this library
// keeps its own overlay so it does not depend on SdFat's headers.
#define USDHC1_BASE 0x40418000u
#define REG(off) (*(volatile uint32_t *)(USDHC1_BASE + (off)))
#define DS_ADDR      REG(0x00)
#define BLK_ATT      REG(0x04)
#define CMD_ARG      REG(0x08)
#define CMD_XFR_TYP  REG(0x0C)
#define CMD_RSP0     REG(0x10)
#define CMD_RSP1     REG(0x14)
#define PRES_STATE   REG(0x24)
#define PROT_CTRL    REG(0x28)
#define SYS_CTRL     REG(0x2C)
#define INT_STATUS   REG(0x30)
#define INT_STATUS_EN REG(0x34)
#define INT_SIGNAL_EN REG(0x38)
#define DATPORT      REG(0x20)
#define MIX_CTRL     REG(0x48)
#define VEND_SPEC    REG(0xC0)

// INT_STATUS bits
static const uint32_t INT_CC   = 1u << 0;   // command complete
static const uint32_t INT_CTOE = 1u << 16;  // command timeout
static const uint32_t INT_CCE  = 1u << 17;  // command CRC error
static const uint32_t INT_CEBE = 1u << 18;  // command end-bit error
static const uint32_t INT_CIE  = 1u << 19;  // command index error
static const uint32_t INT_CMD_ERR = INT_CTOE | INT_CCE | INT_CEBE | INT_CIE;
// CINT, the SDIO card interrupt (DAT1).  It is NOT a latched event: the
// controller derives it from the card's line level and INT_STATUS_EN[CINT], so
// writing 1 to it does not clear it while that enable is set -- the card has to
// let go of DAT1 first.  Both sendCommand() and cmd53() write INT_STATUS back
// wholesale (`INT_STATUS = st`, `INT_STATUS = 0xFFFFFFFF`), and this is why
// that is harmless with card interrupts live: the bit declines the write, and
// nothing in either function tests it.
static const uint32_t INT_CINT = 1u << 8;

// CMD_XFR_TYP response types
static const uint32_t RSP_NONE   = 0u << 16;
static const uint32_t RSP_136    = 1u << 16;
static const uint32_t RSP_48     = 2u << 16;
static const uint32_t RSP_48BUSY = 3u << 16;
static const uint32_t CHK_CRC    = 1u << 19;
static const uint32_t CHK_IDX    = 1u << 20;

static void gpioMux(uint8_t mode) {
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_00 = mode;  // USDHC1_CMD
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_01 = mode;  // USDHC1_CLK
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_02 = mode;  // USDHC1_DATA0
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_03 = mode;  // USDHC1_DATA1
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_04 = mode;  // USDHC1_DATA2
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_05 = mode;  // USDHC1_DATA3
}

static void enablePads() {
    // RT1176 SW_PAD_CTL fields: bit4 ODE, bits[3:2] PULL (01=up,10=down,
    // 11=none), bit1 PDRV (0=high drive).  CMD/DATA = pull-up + high drive;
    // CLK = pull-down + high drive.  Copied from SdFat's proven RT1176 branch --
    // the 1062 PKE/DSE/SPEED encoding does not exist on this part.
    const uint32_t DATA_PAD = 0x04;
    const uint32_t CLK_PAD  = 0x08;
    gpioMux(0);  // ALT0 = USDHC1
    IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B1_00 = DATA_PAD;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B1_01 = CLK_PAD;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B1_02 = DATA_PAD;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B1_03 = DATA_PAD;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B1_04 = DATA_PAD;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B1_05 = DATA_PAD;
}

// Root frequency for the divider maths below.  MUST track initClock().
//
// 24 MHz, not 198 MHz: this driver takes OSC_24M rather than SYS_PLL2_PFD2.
// See initClock() for why.
static const uint32_t BASE_CLOCK = 24000000U;

static void initClock() {
    // USDHC1 clock root (CLOCK_ROOT58) <- OSC_24M (mux 1), undivided, then
    // ungate USDHC1 (LPCG117).
    //
    // This deliberately does NOT use SYS_PLL2_PFD2 (mux 4, div 2 -> 198 MHz),
    // which is what SdFat's RT1176 branch does and what this driver did until
    // 2026-08-17.  That path ASSUMES the boot ROM left PLL2's PFD2 at 396 MHz:
    // the imxrt1176 core's startup.c brings up only the ARM PLL and the AHB,
    // never PLL2.  NXP's own BOARD_USDHC1ClockConfiguration() for this board
    // does not assume it either -- it calls CLOCK_InitSysPll2() and
    // CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd2, 24) first.
    //
    // The trap is that a wrong root is nearly invisible: SYS_CTRL's divider
    // still produces a stable clock and PRES_STATE[SDSTB] still asserts, so the
    // controller looks healthy while the card sees a frequency far from the
    // 400 kHz identification clock it is entitled to expect.  OSC_24M is always
    // running and boot-independent, which removes the assumption instead of
    // re-deriving it.  It caps the bus at 24 MHz, ample for enumeration.
    CCM_CLOCK_ROOT58_CONTROL = CCM_CLOCK_ROOT_CONTROL_MUX(1) | CCM_CLOCK_ROOT_CONTROL_DIV(0);
    CCM_LPCG117_DIRECT = 1;
}

SdioHost::Status SdioHost::setClock(uint32_t hz) {
    // Gate the card clock while changing the divider, then wait for SDSTB.
    uint32_t base = BASE_CLOCK;
    uint32_t bestPre = 0, bestDiv = 0;
    uint32_t bestErr = 0xFFFFFFFFu;
    for (uint32_t pre = 1; pre <= 256; pre <<= 1) {
        for (uint32_t div = 1; div <= 16; div++) {
            uint32_t f = base / (pre * div);
            if (f > hz) continue;
            uint32_t err = hz - f;
            if (err < bestErr) { bestErr = err; bestPre = pre; bestDiv = div; }
        }
    }
    if (bestPre == 0) return CLOCK_UNSTABLE;

    // SYS_CTRL: SDCLKFS = prescaler>>1, DVS = divisor-1.
    uint32_t sdclkfs = bestPre >> 1;
    uint32_t dvs     = bestDiv - 1;
    SYS_CTRL = (SYS_CTRL & ~0x0000FFF0u) | (sdclkfs << 8) | (dvs << 4) | (0xEu << 16);

    for (uint32_t i = 0; i < 100000; i++) {
        if (PRES_STATE & (1u << 3)) return OK;   // SDSTB
    }
    return CLOCK_UNSTABLE;
}

SdioHost::Status SdioHost::sendCommand(uint8_t index, uint32_t arg,
                                       uint32_t xferFlags, uint32_t *resp) {
    // Wait for CIHB (command inhibit) to clear.
    for (uint32_t i = 0; PRES_STATE & (1u << 0); i++) {
        if (i > 1000000) return CMD_TIMEOUT;
    }
    INT_STATUS = 0xFFFFFFFFu;          // clear stale status (w1c)
    CMD_ARG    = arg;
    MIX_CTRL  &= ~0x3Fu;               // no data phase for any command here
    CMD_XFR_TYP = ((uint32_t)index << 24) | xferFlags;

    uint32_t st = 0;
    for (uint32_t i = 0; ; i++) {
        st = INT_STATUS;
        if (st & (INT_CC | INT_CMD_ERR)) break;
        if (i > 1000000) { m_lastIntStatus = st; return CMD_TIMEOUT; }
    }
    INT_STATUS = st;                   // w1c
    // Keep the status we actually observed.  Reading INT_STATUS again after
    // this point yields zeros -- it is write-1-to-clear and we just cleared it.
    m_lastIntStatus = st;

    if (st & INT_CTOE) return CMD_TIMEOUT;
    if (st & (INT_CCE | INT_CEBE | INT_CIE)) return CMD_CRC;
    if (resp) *resp = CMD_RSP0;
    return OK;
}

SdioHost::Status SdioHost::begin() {
    m_ioFunctions = 0; m_rca = 0; m_cccrRev = 0; m_cisPtr = 0;
    m_lastIntStatus = 0; m_lastR4 = 0;

    initClock();
    enablePads();

    // Reset the whole controller (SYS_CTRL RSTA) and wait for it to clear.
    SYS_CTRL |= (1u << 24);
    for (uint32_t i = 0; SYS_CTRL & (1u << 24); i++) {
        if (i > 1000000) return CLOCK_UNSTABLE;
    }
    PROT_CTRL = (PROT_CTRL & ~0x6u);   // 1-bit bus width for identification
    INT_STATUS_EN = 0xFFFFFFFFu;
    // No interrupt is SIGNALLED to the CPU by default -- the polled path is
    // what W1..W14 ran and it stays the default.  enableCardInt() is the only
    // thing that ever sets a bit here, and it must be called after this reset:
    // RSTA above wipes both enables, so the mirror is dropped too or an
    // already-"armed" flag would describe a controller that is signalling
    // nothing.
    INT_SIGNAL_EN  = 0;
    m_cardIntArmed = false;

    if (m_use1V8) {
        // GPIO_AD_34 (ball J16) ALT4 = USDHC1_VSELECT -> R168 -> U311.5, which
        // selects VDD_1V8 instead of SENSOR_3V3 onto NVCC_SD.  Must come after
        // the RSTA above, which clears VEND_SPEC.  RM offsets: SW_MUX_CTL for
        // GPIO_AD_34 at 194h, SW_PAD_CTL at 3D8h, IOMUXC base 0x400E8000.
        *(volatile uint32_t *)0x400E8194u = 4u;   // ALT4 = USDHC1_VSELECT
        VEND_SPEC |= (1u << 1);                   // VSELECT: drive the rail to 1.8 V
        delay(10);                                // let NVCC_SD settle
        // Read both back.  Asserting a rail switch and not checking it is how
        // the 2026-08-17 session spent a cycle unable to say whether 1.8 V had
        // been tested at all.
        m_lastVendSpec = VEND_SPEC;
        m_lastVselMux  = *(volatile uint32_t *)0x400E8194u;
    }

    Status s = setClock(400000);       // identification clock
    if (s != OK) return s;

    // Send the 80 initialisation clocks via SYS_CTRL[INITA], and wait for the
    // bit to self-clear.
    //
    // This is NOT interchangeable with a delay, which is what stood here until
    // the first hardware run on 2026-08-17 returned "no card" with the M.2
    // module fitted, powered and out of reset.  i.MX USDHC GATES SD_CLK while
    // idle, so a delayMicroseconds() before the first command emits exactly
    // zero clocks and the card never leaves its power-up state.  SdFat's
    // SdioTeensy -- hardware-proven on this same controller -- has always used
    // INITA here (SdioTeensy.cpp:509); this driver simply omitted it.
    SYS_CTRL |= (1u << 27);
    for (uint32_t i = 0; SYS_CTRL & (1u << 27); i++) {
        if (i > 1000000) return INIT_CLK_STUCK;
    }

    // CMD5 IO_SEND_OP_COND, arg 0 -> read the card's OCR.  R4 has no CRC and
    // no index, so neither is checked.  An SD MEMORY card ignores CMD5 by
    // spec, so no response here is the expected answer when one is present --
    // reported as CMD5_NO_RESPONSE, which is a different fact from a card that
    // answers and reports zero IO functions.
    uint32_t r4 = 0;
    m_lastPresState = PRES_STATE;        // live bus levels, before we drive it
    s = sendCommand(5, 0, RSP_48, &r4);  // records m_lastIntStatus itself
    m_lastR4 = r4;
    if (s != OK) return CMD5_NO_RESPONSE;

    uint8_t nfn = (r4 >> 28) & 0x7;
    if (nfn == 0) return NO_IO_FUNCTION;

    // Re-issue CMD5 with the voltage window until the card reports ready.
    const uint32_t OCR_32_34 = 0x00300000u;
    for (uint32_t i = 0; i < 1000; i++) {
        s = sendCommand(5, r4 & 0x00FFFFFFu ? (r4 & 0x00FFFFFFu) : OCR_32_34,
                        RSP_48, &r4);
        if (s != OK) return s;
        if (r4 & 0x80000000u) break;   // C bit: initialisation complete
        delayMicroseconds(1000);
        if (i == 999) return CMD_TIMEOUT;
    }
    m_ioFunctions = (r4 >> 28) & 0x7;

    // CMD3 SEND_RELATIVE_ADDR -> R6, RCA in bits 31:16.
    uint32_t r6 = 0;
    s = sendCommand(3, 0, RSP_48 | CHK_CRC | CHK_IDX, &r6);
    if (s != OK) return s;
    m_rca = (uint16_t)(r6 >> 16);

    // CMD7 SELECT_CARD with the RCA -> R1b.
    s = sendCommand(7, (uint32_t)m_rca << 16, RSP_48BUSY | CHK_CRC | CHK_IDX, nullptr);
    if (s != OK) return s;

    s = setClock(25000000);            // leave identification speed
    if (s != OK) return s;

    // CCCR revision (function 0, address 0x00) and the function-0 CIS pointer
    // (0x09..0x0B, little-endian).
    uint8_t b = 0;
    s = cmd52Read(0, 0x00, &b);
    if (s != OK) return s;
    m_cccrRev = b & 0x0F;

    m_cisPtr = 0;
    for (int i = 0; i < 3; i++) {
        s = cmd52Read(0, 0x09 + i, &b);
        if (s != OK) return s;
        m_cisPtr |= (uint32_t)b << (8 * i);
    }
    return OK;
}

// CMD52 argument layout: bit31 R/W, bits30:28 function, bit27 RAW,
// bits25:9 register address, bits7:0 write data.
static inline uint32_t cmd52Arg(bool write, uint8_t fn, uint32_t addr, uint8_t data) {
    return ((uint32_t)write << 31) | ((uint32_t)(fn & 0x7) << 28) |
           ((addr & 0x1FFFFu) << 9) | data;
}

SdioHost::Status SdioHost::cmd52Read(uint8_t fn, uint32_t addr, uint8_t *out) {
    uint32_t r5 = 0;
    Status s = sendCommand(52, cmd52Arg(false, fn, addr, 0),
                           RSP_48 | CHK_CRC | CHK_IDX, &r5);
    if (s != OK) return s;
    if (out) *out = (uint8_t)(r5 & 0xFF);
    return OK;
}

SdioHost::Status SdioHost::cmd52Write(uint8_t fn, uint32_t addr, uint8_t value) {
    return sendCommand(52, cmd52Arg(true, fn, addr, value),
                       RSP_48 | CHK_CRC | CHK_IDX, nullptr);
}

// ---------------------------------------------------------------------------
// The SDIO card interrupt (DAT1).  See SdioHost.h for the protocol; this is
// only the plumbing.
//
// The vector table takes a plain function, so the instance that owns uSDHC1's
// interrupt is recorded here while card interrupts are enabled.  There is
// exactly one uSDHC1 on this part and the file-header WARNING already says only
// one card may be present on it, so a single owner is the honest model rather
// than a limitation worth generalising away.
static SdioHost *s_cardIntOwner = nullptr;

static void usdhc1_card_isr(void) {
    if (s_cardIntOwner) s_cardIntOwner->cardIsr();
}

void SdioHost::cardIsr() {
    // MASK FIRST, AND UNCONDITIONALLY.  DAT1 is a level: while the card still
    // has an unmasked cause the controller keeps the IRQ line asserted, so any
    // path out of here that leaves signalling enabled re-enters immediately and
    // the CPU never returns to thread level.  Clearing the whole register is
    // both correct and atomic -- CINT is the only bit this driver ever signals
    // (begin() sets INT_SIGNAL_EN = 0 and armCardInt() writes CINT alone).
    INT_SIGNAL_EN  = 0;
    m_cardIntArmed = false;
    // No SDIO command is issued here: servicing means CMD52/CMD53 traffic, and
    // this ISR may have interrupted a thread half way through a transfer.  All
    // it does is record that the card wants attention.
    if (INT_STATUS & INT_CINT) {
        m_cardIntFlag = true;
        m_cardIntCount++;
    }
}

void SdioHost::enableCardInt(bool enable) {
    if (enable) {
        m_cardIntFlag  = false;
        m_cardIntArmed = false;
        // The status bit only exists while its status-enable is set; begin()
        // sets the whole register, but this makes the call order-independent.
        INT_STATUS_EN |= INT_CINT;
        s_cardIntOwner = this;
        attachInterruptVector(IRQ_USDHC1, &usdhc1_card_isr);
        NVIC_ENABLE_IRQ(IRQ_USDHC1);
        m_cardIntOn = true;
        // Arm LAST.  INT_SIGNAL_EN is still 0 at the NVIC_ENABLE_IRQ above, so
        // no interrupt can arrive between attaching the vector and being ready
        // for it; and if the card is ALREADY holding DAT1 down (it may well be
        // -- the card side is unmasked long before this), this write is what
        // delivers that standing assertion, which is exactly right.
        armCardInt();
    } else {
        m_cardIntOn    = false;
        INT_SIGNAL_EN  = 0;
        NVIC_DISABLE_IRQ(IRQ_USDHC1);
        s_cardIntOwner = nullptr;
        m_cardIntFlag  = false;
        m_cardIntArmed = false;
    }
}

void SdioHost::armCardInt() {
    if (!m_cardIntOn || m_cardIntArmed) return;
    m_cardIntArmed = true;      // before the write -- see the member comment
    INT_SIGNAL_EN  = INT_CINT;
}

bool SdioHost::takeCardInt() {
    if (!m_cardIntFlag) return false;
    m_cardIntFlag = false;
    return true;
}

SdioHost::Status SdioHost::readManfId(uint16_t *manufacturer, uint16_t *card) {
    if (m_cisPtr == 0) return BAD_CIS;
    // Walk the tuple chain looking for CISTPL_MANFID (0x20).  Tuple format is
    // <code><link><body...>; 0xFF terminates the chain.  Bound the walk -- a
    // corrupt link byte must not spin forever.
    uint32_t addr = m_cisPtr;
    for (int guard = 0; guard < 128; guard++) {
        uint8_t code = 0, link = 0;
        Status s = cmd52Read(0, addr, &code);
        if (s != OK) return s;
        if (code == 0xFF) return BAD_CIS;            // end of chain, no MANFID
        s = cmd52Read(0, addr + 1, &link);
        if (s != OK) return s;
        if (code == 0x20) {                          // CISTPL_MANFID
            if (link < 4) return BAD_CIS;
            uint8_t b[4];
            for (int i = 0; i < 4; i++) {
                s = cmd52Read(0, addr + 2 + i, &b[i]);
                if (s != OK) return s;
            }
            if (manufacturer) *manufacturer = (uint16_t)(b[0] | (b[1] << 8));
            if (card)         *card         = (uint16_t)(b[2] | (b[3] << 8));
            return OK;
        }
        addr += 2 + link;
    }
    return BAD_CIS;
}

// ---------------------------------------------------------------------------
// CMD53 IO_RW_EXTENDED (block mode, PIO)
//
// Argument layout: bit31 R/W, bits30:28 function, bit27 block mode,
// bit26 op code (1 = incrementing address), bits25:9 register address,
// bits8:0 block count (0 = 512).
static inline uint32_t cmd53Arg(bool write, uint8_t fn, uint32_t addr,
                                bool incrAddr, bool blockMode, uint16_t count) {
    return ((uint32_t)write << 31) | ((uint32_t)(fn & 0x7) << 28) |
           ((uint32_t)blockMode << 27) | ((uint32_t)incrAddr << 26) |
           ((addr & 0x1FFFFu) << 9) | (count & 0x1FFu);
}

SdioHost::Status SdioHost::cmd53(uint8_t fn, uint32_t addr, bool incrAddr, bool write,
                                 uint8_t *buf, uint16_t blockSize, uint16_t blocks) {
    // Program the transfer before issuing the command: BLK_ATT carries the
    // block size and count, MIX_CTRL the direction and multi-block flags.
    BLK_ATT  = ((uint32_t)blocks << 16) | blockSize;
    uint32_t mix = (1u << 1);                        // BCEN, block count enable
    if (blocks > 1) mix |= (1u << 5);                // MSBSEL, multi-block
    if (!write)     mix |= (1u << 4);                // DTDSEL, 1 = card -> host
    MIX_CTRL = (MIX_CTRL & ~0x3Fu) | mix;

    INT_STATUS = 0xFFFFFFFFu;
    CMD_ARG = cmd53Arg(write, fn, addr, incrAddr, true, blocks);
    // DPSEL (bit 21) tells the controller a data phase follows.
    CMD_XFR_TYP = ((uint32_t)53 << 24) | (1u << 21) | RSP_48 | CHK_CRC | CHK_IDX;

    // Wait for command completion before touching the data port.
    for (uint32_t i = 0; ; i++) {
        uint32_t st = INT_STATUS;
        if (st & INT_CMD_ERR) { m_lastIntStatus = st; INT_STATUS = st;
                                return (st & INT_CTOE) ? CMD_TIMEOUT : CMD_CRC; }
        if (st & INT_CC) { INT_STATUS = INT_CC; break; }
        if (i > 1000000) { m_lastIntStatus = st; return CMD_TIMEOUT; }
    }

    // PIO the payload a word at a time, gated on the buffer-ready flags:
    // BWEN (bit 10) for writes, BREN (bit 11) for reads.
    uint32_t words = ((uint32_t)blockSize * blocks) / 4u;
    uint32_t *p32 = (uint32_t *)(void *)buf;
    const uint32_t readyBit = write ? (1u << 10) : (1u << 11);
    for (uint32_t w = 0; w < words; w++) {
        for (uint32_t i = 0; !(PRES_STATE & readyBit); i++) {
            uint32_t st = INT_STATUS;
            if (st & (1u << 15)) { m_lastIntStatus = st; return CMD_CRC; }  // ERR
            if (i > 1000000) { m_lastIntStatus = INT_STATUS; return CMD_TIMEOUT; }
        }
        if (write) DATPORT = p32[w]; else p32[w] = DATPORT;
    }

    // Transfer Complete is bit 1.
    for (uint32_t i = 0; ; i++) {
        uint32_t st = INT_STATUS;
        if (st & (1u << 1)) { INT_STATUS = st; m_lastIntStatus = st; return OK; }
        if (st & (1u << 15)) { m_lastIntStatus = st; INT_STATUS = st; return CMD_CRC; }
        if (i > 1000000) { m_lastIntStatus = st; return CMD_TIMEOUT; }
    }
}

SdioHost::Status SdioHost::cmd53Write(uint8_t fn, uint32_t addr, bool incrAddr,
                                      const uint8_t *src, uint16_t blockSize, uint16_t blocks) {
    return cmd53(fn, addr, incrAddr, true, (uint8_t *)(void *)(uintptr_t)src, blockSize, blocks);
}

SdioHost::Status SdioHost::cmd53Read(uint8_t fn, uint32_t addr, bool incrAddr,
                                     uint8_t *dst, uint16_t blockSize, uint16_t blocks) {
    return cmd53(fn, addr, incrAddr, false, dst, blockSize, blocks);
}
