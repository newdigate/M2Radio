// BondStoreEeprom -- persists a BondTable in the core's EEPROM emulation (imxrt1176: the
// top 256 KB of the FlexSPI NOR, 4284 bytes, journaled -- cores/imxrt1176/eeprom.c).
// Header-only ON PURPOSE: bt/test/run.sh compiles every bt/*.cpp into each host suite,
// and this file needs <avr/eeprom.h>.  Hosts: load() once after Hci::begin(); save()
// after every A2dpSource::connect() return (success or failure -- an erased stale bond
// must persist too).  Every write happens outside streaming, since connect() returns
// before media starts.  A corrupt or foreign image loads as an EMPTY table and is left
// in place (load() clears dirty), so it is overwritten by the next real change rather
// than repaired eagerly -- a decision, not an accident.  NEW-34 piece 1.  MIT.
#pragma once
#include <avr/eeprom.h>
#include "BondTable.h"

struct BondStoreEeprom {
    static const uint16_t OFFSET = 4000;                       // 4000 + 234 = 4234 <= 4284 (E2END 0x10BB)
    static_assert(OFFSET + BondTable::IMAGE_SIZE <= (uint16_t)E2END + 1u, "bond image does not fit the emulated EEPROM");
    // Read the image and load the table.  False (and an EMPTY table) on a fresh part (0xFF),
    // a QEMU run (0x00), or anything storage-memory/eeprom_test scribbled over -- never a stale bond.
    static bool load(BondTable &t) {
        uint8_t img[BondTable::IMAGE_SIZE];
        eeprom_read_block(img, (const void *)(uintptr_t)OFFSET, sizeof img);
        return t.load(img, sizeof img);
    }
    // Write the image if the table changed.  eeprom_write_byte skips a byte the emulation
    // already holds, so an unchanged image costs no flash writes; a first pairing changes
    // ~60 bytes, a move-to-front up to the whole 224-byte body.
    static bool save(BondTable &t) {
        if (!t.dirty()) return false;
        uint8_t img[BondTable::IMAGE_SIZE];
        t.save(img, sizeof img);
        eeprom_write_block(img, (void *)(uintptr_t)OFFSET, sizeof img);
        t.clearDirty();
        return true;
    }
    // Bench knob (M2_BT_FORGET_BONDS): zero the image and empty the table.
    static void wipe(BondTable &t) {
        uint8_t z[BondTable::IMAGE_SIZE];
        for (uint16_t i = 0; i < sizeof z; i++) z[i] = 0;
        eeprom_write_block(z, (void *)(uintptr_t)OFFSET, sizeof z);
        t.clear(); t.clearDirty();
    }
};
