// BondStoreEeprom -- persists a BondTable in the core's EEPROM emulation (imxrt1176: the
// top 256 KB of the FlexSPI NOR, 4284 bytes, journaled -- cores/imxrt1176/eeprom.c).
// Header-only ON PURPOSE: bt/test/run.sh compiles every bt/*.cpp into each host suite,
// and this file needs <avr/eeprom.h>.  NEW-34 piece 1.  MIT.
//
// CALLER OBLIGATIONS (this file cannot enforce them):
//  * load() once after Hci::begin(); save() after EVERY A2dpSource::connect() return --
//    success or failure, since an erased stale bond must persist too.  A failed attempt
//    that leaves the table clean costs nothing; one that DIRTIED it on every retry would
//    turn a 5 s retry loop into a flash-write loop, so BtLink dirties only on a real
//    change (a notification, a stored-key success, an erase) -- never on a bare failure.
//  * Call both from setup()/loop() context only, never from an ISR, and only where a burst
//    of IRQ-masked latency is tolerable.  The core programs the NOR with IRQs MASKED: once
//    per changed byte (a 2-byte journal program -- short, but ~234 of them on a first
//    pairing of a virgin part, tens of ms in total) and, when a sector's 2048-entry
//    journal fills, for a whole 4 KB sector ERASE (tens of ms).  The emulation maps EEPROM
//    address a to sector (a>>2)%63, so this 234-byte image spans 59 of the 63 sectors,
//    ~4 bytes each; those journals fill in near-lockstep, and a full image changes only on
//    a pairing or a peer switch -- expect an erase CLUSTER (up to 59 erases, seconds)
//    roughly once per ~512 changing saves.  The figures are estimates from the core's code
//    path, not measurements.  "Outside media streaming" is therefore necessary, not
//    sufficient: acid_box's SAI ISR runs whenever the codec does, and may glitch the
//    LOCAL output at a pairing event.
//  * load() REPLACES the table and clears dirty: a pending unsaved change is discarded.
//  * The write is unverified (a read-back would double the cost).  A torn write leaves a
//    stale CRC over a new body, so the next load() reads EMPTY -- never a spliced key.
//  * A corrupt or foreign image loads as an EMPTY table and is left in place until the
//    next real change rather than repaired eagerly -- a decision, not an accident.
//  * The [reconnect] probe's cold reload calls eeprom_initialize() (the CORE's function)
//    before load(): it re-derives the journal index from the flash, which is strictly
//    stronger than trusting the incrementally maintained copy.
//
// REGION: EEPROM bytes OFFSET..END-1 = 4000..4233 are reserved for this image.  Only 50
// bytes remain above it, so a second record goes BELOW 4000 and static_asserts against
// END.  storage-memory/eeprom_test writes 4200..4211 -- a different firmware, so after
// that example has run the bond image is simply corrupt and loads empty.  OFFSET buys no
// physical isolation: every EEPROM user in the same firmware shares the same journals
// and brings the erase cluster forward.
#pragma once
#include <stdint.h>
#include <avr/eeprom.h>
#include "BondTable.h"

struct BondStoreEeprom {
    static const uint16_t OFFSET = 4000;
    static const uint16_t END = OFFSET + BondTable::IMAGE_SIZE;         // 4234, one past the image
    static_assert(END <= E2END + 1u, "bond image does not fit the emulated EEPROM");
    // Read the image and load the table.  False (and an EMPTY table) on a fresh part (0xFF),
    // a QEMU run (0x00), or anything another firmware scribbled over -- never a stale bond.
    static bool load(BondTable &t) {
        uint8_t img[BondTable::IMAGE_SIZE];
        eeprom_read_block(img, (const void *)(uintptr_t)OFFSET, sizeof img);
        return t.load(img, sizeof img);
    }
    // Write the image if the table changed; true when it was written.  eeprom_write_byte
    // skips a byte the emulation already holds, so an unchanged image costs no flash writes
    // and a move-to-front changes only the entries that moved.
    static bool save(BondTable &t) {
        if (!t.dirty()) return false;
        uint8_t img[BondTable::IMAGE_SIZE];
        if (t.save(img, sizeof img) != BondTable::IMAGE_SIZE) return false;   // unreachable by construction; never write an unfilled buffer
        eeprom_write_block(img, (void *)(uintptr_t)OFFSET, sizeof img);
        t.clearDirty();
        return true;
    }
    // Bench knob (M2_BT_FORGET_BONDS): store the canonical EMPTY image, so a later load()
    // reads true with no bonds -- a deliberate forget stays distinguishable from damage.
    // (clear() dirties the table, which is what lets save() write.)
    static void wipe(BondTable &t) { t.clear(); save(t); }
};
