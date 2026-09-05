// BondTable -- the bonded-device table for BtLink: up to MAX peers, most recently
// used first, each with its BR/EDR link key.  Pure C++11, no heap, no Arduino; the
// image codec (save/load) is what a host-side store persists (BondStoreEeprom.h).
// NEW-34 piece 1 -- docs/superpowers/specs/2026-09-05-bt-known-device-reconnect-design.md.  MIT.
#pragma once
#include <stdint.h>
#include <stddef.h>

struct Bond {
    uint8_t bd[6];        // as on the wire (little-endian), like every bd[] in hci/ and bt/
    uint8_t key[16];      // the link key from Link_Key_Notification
    uint8_t keyType;      // Link_Key_Notification Key_Type: 0 = legacy combination, 4 = unauthenticated P-192 (Just Works)
    uint8_t psrm;         // Page_Scan_Repetition_Mode we paged with -- Create_Connection needs it without an inquiry
    char    name[32];     // the inquiry's remote name, truncated to 31 chars + NUL (NEW-35's list; the boot policy's name filter)
};

class BondTable {
public:
    static const uint8_t  MAX = 4;
    static const uint8_t  NAME_LEN = 31;   // chars, excluding the NUL.  Not NAME_MAX: that is a POSIX macro in <limits.h>, which Audio.h pulls in via arm_math.h
    static const uint8_t  VERSION = 1;
    static const uint16_t IMAGE_SIZE = 4 + 1 + 1 + MAX * sizeof(Bond) + 4;   // "BTBD" ver count entries crc32 = 234

    BondTable();
    void clear();                                        // empty the table; sets dirty like every other mutator
    const Bond *find(const uint8_t bd[6]) const;
    // Insert at the front; an existing entry is updated and moved to the front; the LAST entry is
    // evicted when full.  On an update, an EMPTY name keeps the old name and an ALL-ZERO key keeps
    // the old key, keyType and psrm -- so a caller that knows only the address and the name (a
    // remote-name refresh) can never wipe the link key that decides what goes on the wire.  Sets dirty.
    void upsert(const Bond &b);
    void touch(const uint8_t bd[6]);                     // move to the front; dirty only if it moved; no-op when absent
    bool erase(const uint8_t bd[6]);                     // sets dirty when it removed something
    uint8_t count() const { return m_count; }
    const Bond &at(uint8_t i) const { return m_b[i]; }   // 0 = most recent; i < count()
    uint16_t save(uint8_t *out, uint16_t cap) const;     // serialise; IMAGE_SIZE, or 0 when cap < IMAGE_SIZE.  Leaves dirty as is: the store clears it after a successful write
    // Deserialise.  False AND an EMPTY table on any bad magic/version/count/length/CRC or a
    // duplicated address -- a caller can never keep stale RAM entries by mistake.  Loads
    // CANONICALLY: entries beyond `count` are zeroed and every name is re-terminated and zero-tailed,
    // so equal tables give equal images.
    // Clears dirty either way (RAM now == store; a corrupt image is left in place until the
    // next real change, which is a decision, not an accident).
    bool load(const uint8_t *in, uint16_t len);
    bool dirty() const { return m_dirty; }
    void clearDirty() { m_dirty = false; }
    static uint32_t crc32(const uint8_t *p, size_t n);   // IEEE 802.3, reflected, init/xorout 0xFFFFFFFF ("123456789" -> 0xCBF43926)
    static void copyName(char out[32], const char *in);  // truncating copy, NUL-terminated, zero-padded tail
    // ★ Pointers from find() and references from at() are INVALIDATED by upsert()/touch()/erase()
    // (every mutator memmoves the entries): copy a Bond out before mutating or paging.  Not
    // reentrant and not ISR-safe -- call from one context (BtLink's handlers run from the
    // idle()-pumped HCI dispatch and only ever submit(), so they never nest).  Bump VERSION
    // whenever Bond or MAX changes: an old image then fails on VERSION, not merely on CRC.
private:
    int  indexOf(const uint8_t bd[6]) const;
    void moveToFront(int i);
    Bond    m_b[MAX];
    uint8_t m_count;
    bool    m_dirty;
};
