// Host tests for BondTable (NEW-34 piece 1): the image codec, the recency order and the
// bond semantics BtLink relies on.  Pure; no Hci.
#include "BondTable.h"
#include <stdio.h>
#include <string.h>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
// A bond at AA:BB:CC:DD:EE:<last> (little-endian on the wire, like every bd[] in hci/ and bt/)
// with a key that ramps from k0.
static Bond mk(uint8_t last, uint8_t k0, const char *name, uint8_t type = 4, uint8_t psrm = 1) {
    Bond b; memset(&b, 0, sizeof b);
    const uint8_t base[6] = { last, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA }; memcpy(b.bd, base, 6);
    for (int i = 0; i < 16; i++) b.key[i] = (uint8_t)(k0 + i);
    b.keyType = type; b.psrm = psrm; BondTable::copyName(b.name, name); return b;
}
static void fixCrc(uint8_t *img) {   // recompute the trailing CRC so ONLY the field under test is wrong
    uint32_t c = BondTable::crc32(img, BondTable::IMAGE_SIZE - 4);
    img[230] = (uint8_t)c; img[231] = (uint8_t)(c >> 8); img[232] = (uint8_t)(c >> 16); img[233] = (uint8_t)(c >> 24);
}
int main() {
    {   // 1. CRC-32 is the IEEE one (reflected 0xEDB88320, init/xorout 0xFFFFFFFF): the standard check value.
        CHECK(BondTable::crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u);
        CHECK(BondTable::crc32(nullptr, 0) == 0x00000000u);
    }
    {   // 2. Layout: the image depends on a 56-byte Bond and a 234-byte image.
        CHECK(sizeof(Bond) == 56);
        CHECK(BondTable::IMAGE_SIZE == 234);
    }
    {   // 3. Empty table: save/load round trip, not dirty, count 0; a too-small buffer is refused.
        BondTable t; uint8_t img[BondTable::IMAGE_SIZE];
        CHECK(t.count() == 0 && !t.dirty());
        CHECK(t.save(img, sizeof img) == BondTable::IMAGE_SIZE);
        CHECK(memcmp(img, "BTBD", 4) == 0 && img[4] == 1 && img[5] == 0);
        BondTable u; CHECK(u.load(img, sizeof img) && u.count() == 0 && !u.dirty());
        CHECK(t.save(img, 10) == 0);
    }
    {   // 4. upsert: most recent at index 0; an update moves to the front and replaces the key; find/at agree.
        BondTable t;
        t.upsert(mk(0x01, 0x10, "FAKE-HEADSET-01")); t.upsert(mk(0x02, 0x20, "OpenMove by Shokz"));
        CHECK(t.count() == 2 && t.dirty());
        CHECK(t.at(0).bd[0] == 0x02 && t.at(1).bd[0] == 0x01);
        const Bond *f = t.find(mk(0x01, 0, "").bd);
        CHECK(f && f->key[0] == 0x10 && strcmp(f->name, "FAKE-HEADSET-01") == 0 && f->keyType == 4 && f->psrm == 1);
        t.upsert(mk(0x01, 0x30, "FAKE-HEADSET-01"));
        CHECK(t.count() == 2 && t.at(0).bd[0] == 0x01 && t.at(0).key[0] == 0x30 && t.at(1).bd[0] == 0x02);
        CHECK(t.find(mk(0x09, 0, "").bd) == nullptr);
    }
    {   // 5. Eviction: the fifth bond evicts the LAST (least recent) entry.
        BondTable t;
        for (uint8_t i = 1; i <= 4; i++) t.upsert(mk(i, i, "n"));
        CHECK(t.count() == 4 && t.at(3).bd[0] == 1);
        t.upsert(mk(5, 5, "n"));
        CHECK(t.count() == 4 && t.at(0).bd[0] == 5 && t.at(3).bd[0] == 2 && t.find(mk(1, 0, "").bd) == nullptr);
    }
    {   // 6. touch / erase / dirty semantics.
        BondTable t;
        t.upsert(mk(1, 1, "a")); t.upsert(mk(2, 2, "b")); t.upsert(mk(3, 3, "c")); t.clearDirty();
        t.touch(mk(3, 0, "").bd);   CHECK(!t.dirty());                                   // already at the front: nothing changed
        t.touch(mk(1, 0, "").bd);   CHECK(t.dirty() && t.at(0).bd[0] == 1 && t.at(1).bd[0] == 3 && t.at(2).bd[0] == 2);
        t.clearDirty();
        CHECK(!t.erase(mk(9, 0, "").bd) && !t.dirty());
        CHECK(t.erase(mk(3, 0, "").bd) && t.dirty() && t.count() == 2 && t.at(0).bd[0] == 1 && t.at(1).bd[0] == 2);
        t.touch(mk(9, 0, "").bd);   CHECK(t.count() == 2);                              // absent: no-op
    }
    {   // 7. Round trip with content; every bad image loads FALSE AND EMPTY (stale RAM entries gone too).
        BondTable t; t.upsert(mk(1, 0x40, "OpenMove by Shokz", 4, 1)); t.upsert(mk(2, 0x50, "EVKB-SINK", 0, 2));
        uint8_t img[BondTable::IMAGE_SIZE]; CHECK(t.save(img, sizeof img) == BondTable::IMAGE_SIZE);
        BondTable u; CHECK(u.load(img, sizeof img) && u.count() == 2 && !u.dirty());
        CHECK(u.at(0).bd[0] == 2 && u.at(0).keyType == 0 && u.at(0).psrm == 2 && strcmp(u.at(0).name, "EVKB-SINK") == 0);
        CHECK(u.at(1).bd[0] == 1 && memcmp(u.at(1).key, t.at(1).key, 16) == 0);
        // Every bad image below is loaded into a table that HOLDS a stale bond: the contract is
        // false AND empty AND clean, so a rejected image can never leave a key behind for
        // Link_Key_Request.  (Before this, six of these arms asserted emptiness against an
        // already-empty table and passed against a load() that kept stale entries on bad
        // magic/version/count -- found by mutation in review.)
        Bond stale = mk(7, 7, "stale");
        uint8_t bad[BondTable::IMAGE_SIZE];
        { BondTable v; v.upsert(stale); memcpy(bad, img, sizeof bad); bad[6 + 20] ^= 0x01;
          CHECK(!v.load(bad, sizeof bad) && v.count() == 0 && !v.dirty() && v.find(stale.bd) == nullptr); }   // one key byte flipped, CRC wrong
        { BondTable v; v.upsert(stale); memcpy(bad, img, sizeof bad);
          CHECK(!v.load(bad, BondTable::IMAGE_SIZE - 1) && v.count() == 0 && !v.dirty() && v.find(stale.bd) == nullptr); }   // truncated
        { BondTable v; v.upsert(stale); memcpy(bad, img, sizeof bad); bad[0] = 'X'; fixCrc(bad);
          CHECK(!v.load(bad, sizeof bad) && v.count() == 0 && !v.dirty() && v.find(stale.bd) == nullptr); }   // magic
        { BondTable v; v.upsert(stale); memcpy(bad, img, sizeof bad); bad[4] = 2; fixCrc(bad);
          CHECK(!v.load(bad, sizeof bad) && v.count() == 0 && !v.dirty() && v.find(stale.bd) == nullptr); }   // version
        { BondTable v; v.upsert(stale); memcpy(bad, img, sizeof bad); bad[5] = 5; fixCrc(bad);
          CHECK(!v.load(bad, sizeof bad) && v.count() == 0 && !v.dirty() && v.find(stale.bd) == nullptr); }   // count > MAX
        { BondTable v; v.upsert(stale); memset(bad, 0xFF, sizeof bad);
          CHECK(!v.load(bad, sizeof bad) && v.count() == 0 && !v.dirty() && v.find(stale.bd) == nullptr); }   // a fresh part
        { BondTable v; v.upsert(stale); memset(bad, 0x00, sizeof bad);
          CHECK(!v.load(bad, sizeof bad) && v.count() == 0 && !v.dirty() && v.find(stale.bd) == nullptr); }   // a QEMU run
        { BondTable v; v.upsert(stale); CHECK(!v.load(nullptr, 0) && v.count() == 0 && !v.dirty()); }
    }
    {   // 8. Names: truncated to 31 characters, always terminated; an EMPTY name on update keeps the old one
        //    (the notification handler may not know the name of a device it did not inquire this boot).
        BondTable t; Bond b = mk(1, 1, "");
        BondTable::copyName(b.name, "0123456789012345678901234567890123456789");   // 40 chars
        CHECK(strlen(b.name) == 31 && b.name[31] == 0);
        t.upsert(b);
        t.upsert(mk(1, 2, ""));
        CHECK(strlen(t.at(0).name) == 31 && t.at(0).key[0] == 2);
    }
    {   // 9. Boundaries found by mutation in review: a FULL table round-trips (count == MAX is the
        //    in[5] > MAX edge); save() refuses a buffer one byte short; refreshing the LEAST-recent
        //    entry of a full table moves it to the front; erase() at index 0 shifts the survivors
        //    down and zeroes the vacated slot (no stale key material; equal tables give equal
        //    images); an image whose count says 0 over a populated body loads canonically; an
        //    image with a duplicated address is refused as corrupt.
        BondTable t;
        for (uint8_t i = 1; i <= 4; i++) t.upsert(mk(i, i, "n"));
        uint8_t img[BondTable::IMAGE_SIZE];
        CHECK(t.save(img, BondTable::IMAGE_SIZE - 1) == 0);
        CHECK(t.save(img, sizeof img) == BondTable::IMAGE_SIZE && img[5] == 4);
        BondTable u; CHECK(u.load(img, sizeof img) && u.count() == 4 && u.at(0).bd[0] == 4 && u.at(3).bd[0] == 1);
        u.upsert(mk(1, 0x11, "n"));                                                          // refresh the least recent
        CHECK(u.count() == 4 && u.at(0).bd[0] == 1 && u.at(0).key[0] == 0x11 && u.at(1).bd[0] == 4 && u.at(3).bd[0] == 2);
        CHECK(u.erase(mk(1, 0, "").bd) && u.count() == 3 && u.at(0).bd[0] == 4 && u.at(1).bd[0] == 3 && u.at(2).bd[0] == 2);   // erase at index 0
        uint8_t a[BondTable::IMAGE_SIZE], b[BondTable::IMAGE_SIZE];
        u.save(a, sizeof a);
        BondTable w; w.upsert(mk(2, 2, "n")); w.upsert(mk(3, 3, "n")); w.upsert(mk(4, 4, "n"));   // the same three, built fresh
        w.save(b, sizeof b);
        CHECK(memcmp(a, b, sizeof a) == 0);                                                  // the vacated slot was zeroed
        memcpy(img, b, sizeof img); img[5] = 0; fixCrc(img);                                 // count 0 over a populated body
        BondTable x; CHECK(x.load(img, sizeof img) && x.count() == 0);
        uint8_t z[BondTable::IMAGE_SIZE]; BondTable e; e.save(z, sizeof z);
        x.save(img, sizeof img); CHECK(memcmp(img, z, sizeof z) == 0);                      // loaded canonically: body zeroed
        memcpy(img, b, sizeof img); memcpy(img + 6 + 56, img + 6, 6); fixCrc(img);           // entry 1 takes entry 0's address
        BondTable d; d.upsert(mk(7, 7, "stale")); CHECK(!d.load(img, sizeof img) && d.count() == 0 && !d.dirty());
    }
    {   // 10. Six-byte address identity (two bonds differing only in the LAST byte are distinct),
        //     copyName's zero tail and null input, touch() on an ABSENT address leaves dirty alone,
        //     clear() sets dirty, load() clears dirty even on failure.
        BondTable t;
        Bond a = mk(1, 0x10, "A"), b = mk(1, 0x20, "B"); b.bd[5] = 0xAB;
        t.upsert(a); t.upsert(b);
        CHECK(t.count() == 2 && t.find(a.bd) && t.find(a.bd)->key[0] == 0x10 && t.find(b.bd) && t.find(b.bd)->key[0] == 0x20);
        char nm[32]; memset(nm, 0xAA, sizeof nm); BondTable::copyName(nm, "abc");
        bool tailZero = true; for (int i = 3; i < 32; i++) if (nm[i] != 0) tailZero = false;
        CHECK(strcmp(nm, "abc") == 0 && tailZero);
        memset(nm, 0xAA, sizeof nm); BondTable::copyName(nm, nullptr); CHECK(nm[0] == 0 && nm[31] == 0);
        t.clearDirty(); t.touch(mk(9, 0, "").bd); CHECK(!t.dirty());
        t.clear(); CHECK(t.dirty() && t.count() == 0);
        uint8_t ff[BondTable::IMAGE_SIZE]; memset(ff, 0xFF, sizeof ff);
        t.upsert(a); CHECK(!t.load(ff, sizeof ff) && !t.dirty() && t.count() == 0);
    }
    {   // 11. Found in re-review: a VALID image whose name field has no terminator must not let a
        //     name run past its entry (measured: 33 chars, into m_count/m_dirty); a name with junk
        //     after its NUL loads canonically (re-saves byte-identical to a fresh table); and an
        //     update carrying only an address and a name (an ALL-ZERO key) keeps the stored key,
        //     keyType and psrm -- the table decides which key goes on the wire, so a name refresh
        //     can never wipe one.
        BondTable t; t.upsert(mk(1, 0x40, "OpenMove by Shokz", 4, 1));
        uint8_t img[BondTable::IMAGE_SIZE]; t.save(img, sizeof img);
        uint8_t un[BondTable::IMAGE_SIZE]; memcpy(un, img, sizeof un); memset(un + 6 + 24, 'A', 32); fixCrc(un);   // entry 0's name: 32 x 'A', no NUL
        BondTable u; CHECK(u.load(un, sizeof un) && u.count() == 1 && strlen(u.at(0).name) == 31);
        uint8_t jk[BondTable::IMAGE_SIZE]; memcpy(jk, img, sizeof jk); memset(jk + 6 + 24 + 4, 'J', 28); memcpy(jk + 6 + 24, "abc", 4); fixCrc(jk);   // "abc\0" then junk
        BondTable v; CHECK(v.load(jk, sizeof jk) && strcmp(v.at(0).name, "abc") == 0);
        BondTable w; w.upsert(mk(1, 0x40, "abc", 4, 1));
        uint8_t a[BondTable::IMAGE_SIZE], b[BondTable::IMAGE_SIZE]; v.save(a, sizeof a); w.save(b, sizeof b);
        CHECK(memcmp(a, b, sizeof a) == 0);
        Bond r; memset(&r, 0, sizeof r); memcpy(r.bd, t.at(0).bd, 6); BondTable::copyName(r.name, "Renamed");   // address + name only
        t.upsert(mk(2, 0x50, "other")); t.upsert(r);
        const Bond *f = t.find(r.bd);
        CHECK(f && f->key[0] == 0x40 && f->keyType == 4 && f->psrm == 1 && strcmp(f->name, "Renamed") == 0 && t.at(0).bd[0] == 1);
    }
    printf("bondtable_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
