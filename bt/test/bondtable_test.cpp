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
        uint8_t bad[BondTable::IMAGE_SIZE];
        BondTable v;
        memcpy(bad, img, sizeof bad); bad[6 + 20] ^= 0x01;                      // one key byte flipped, CRC now wrong
        v.upsert(mk(7, 7, "stale")); CHECK(!v.load(bad, sizeof bad) && v.count() == 0);
        memcpy(bad, img, sizeof bad); v.upsert(mk(7, 7, "stale")); CHECK(!v.load(bad, BondTable::IMAGE_SIZE - 1) && v.count() == 0);   // truncated
        memcpy(bad, img, sizeof bad); bad[0] = 'X'; fixCrc(bad);              CHECK(!v.load(bad, sizeof bad) && v.count() == 0);     // magic
        memcpy(bad, img, sizeof bad); bad[4] = 2;   fixCrc(bad);              CHECK(!v.load(bad, sizeof bad) && v.count() == 0);     // version
        memcpy(bad, img, sizeof bad); bad[5] = 5;   fixCrc(bad);              CHECK(!v.load(bad, sizeof bad) && v.count() == 0);     // count > MAX
        uint8_t ff[BondTable::IMAGE_SIZE]; memset(ff, 0xFF, sizeof ff);      CHECK(!v.load(ff, sizeof ff) && v.count() == 0);       // a fresh part
        memset(ff, 0x00, sizeof ff);                                          CHECK(!v.load(ff, sizeof ff) && v.count() == 0);       // a QEMU run
        CHECK(!v.load(nullptr, 0) && v.count() == 0);
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
    printf("bondtable_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
