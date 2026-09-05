#include "BondTable.h"
#include <string.h>

static_assert(sizeof(Bond) == 56, "Bond must be 56 bytes with no padding: the image layout depends on it");
static const uint8_t MAGIC[4] = { 'B', 'T', 'B', 'D' };
const uint8_t  BondTable::MAX;
const uint8_t  BondTable::NAME_MAX;
const uint8_t  BondTable::VERSION;
const uint16_t BondTable::IMAGE_SIZE;

BondTable::BondTable() : m_count(0), m_dirty(false) { memset(m_b, 0, sizeof m_b); }

uint32_t BondTable::crc32(const uint8_t *p, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return c ^ 0xFFFFFFFFu;
}

void BondTable::copyName(char out[32], const char *in) {
    size_t n = 0;
    if (in) while (n < NAME_MAX && in[n]) { out[n] = in[n]; n++; }
    for (size_t i = n; i < 32; i++) out[i] = 0;          // terminator + a zero tail, so equal tables give equal images
}

void BondTable::clear() { memset(m_b, 0, sizeof m_b); m_count = 0; m_dirty = true; }

int BondTable::indexOf(const uint8_t bd[6]) const {
    for (uint8_t i = 0; i < m_count; i++) if (memcmp(m_b[i].bd, bd, 6) == 0) return (int)i;
    return -1;
}

const Bond *BondTable::find(const uint8_t bd[6]) const { int i = indexOf(bd); return i < 0 ? nullptr : &m_b[i]; }

void BondTable::moveToFront(int i) {
    if (i <= 0) return;
    Bond t = m_b[i];
    memmove(&m_b[1], &m_b[0], sizeof(Bond) * (size_t)i);
    m_b[0] = t;
}

void BondTable::upsert(const Bond &b) {
    Bond nb = b; copyName(nb.name, b.name);              // normalise: terminator + zero tail
    int i = indexOf(b.bd);
    if (i < 0) {
        if (m_count < MAX) m_count++;                    // else the last entry is evicted by the shift below
        memmove(&m_b[1], &m_b[0], sizeof(Bond) * (size_t)(m_count - 1));
        m_b[0] = nb;
    } else {
        if (nb.name[0] == 0) memcpy(nb.name, m_b[i].name, 32);   // an update without a name keeps the old one
        m_b[i] = nb; moveToFront(i);
    }
    m_dirty = true;
}

void BondTable::touch(const uint8_t bd[6]) {
    int i = indexOf(bd);
    if (i > 0) { moveToFront(i); m_dirty = true; }
}

bool BondTable::erase(const uint8_t bd[6]) {
    int i = indexOf(bd);
    if (i < 0) return false;
    memmove(&m_b[i], &m_b[i + 1], sizeof(Bond) * (size_t)(m_count - 1 - i));
    m_count--; memset(&m_b[m_count], 0, sizeof(Bond)); m_dirty = true;
    return true;
}

uint16_t BondTable::save(uint8_t *out, uint16_t cap) const {
    if (cap < IMAGE_SIZE) return 0;
    memcpy(out, MAGIC, 4); out[4] = VERSION; out[5] = m_count;
    memcpy(out + 6, m_b, sizeof m_b);
    uint32_t c = crc32(out, IMAGE_SIZE - 4);
    out[IMAGE_SIZE - 4] = (uint8_t)c; out[IMAGE_SIZE - 3] = (uint8_t)(c >> 8);
    out[IMAGE_SIZE - 2] = (uint8_t)(c >> 16); out[IMAGE_SIZE - 1] = (uint8_t)(c >> 24);
    return IMAGE_SIZE;
}

bool BondTable::load(const uint8_t *in, uint16_t len) {
    clear(); m_dirty = false;
    if (!in || len < IMAGE_SIZE) return false;
    if (memcmp(in, MAGIC, 4) != 0 || in[4] != VERSION || in[5] > MAX) return false;
    uint32_t want = (uint32_t)in[IMAGE_SIZE - 4] | ((uint32_t)in[IMAGE_SIZE - 3] << 8)
                  | ((uint32_t)in[IMAGE_SIZE - 2] << 16) | ((uint32_t)in[IMAGE_SIZE - 1] << 24);
    if (crc32(in, IMAGE_SIZE - 4) != want) return false;
    memcpy(m_b, in + 6, sizeof m_b); m_count = in[5];
    memset(&m_b[m_count], 0, sizeof(Bond) * (size_t)(MAX - m_count));      // canonical: nothing beyond count survives
    for (uint8_t i = 0; i < m_count; i++) {
        m_b[i].name[NAME_MAX] = 0;                                          // never trust an image's terminator
        for (uint8_t j = 0; j < i; j++)
            if (memcmp(m_b[i].bd, m_b[j].bd, 6) == 0) { clear(); m_dirty = false; return false; }   // a duplicated address is corruption
    }
    return true;
}
