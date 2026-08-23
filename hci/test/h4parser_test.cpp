// Host unit tests for H4Parser.  Fixture bytes are the Core 5.2 Vol 4 Part E
// encodings; the Command Complete for Reset (04 0E 04 01 03 0C 00) is also
// what m2_sdio_probe's B0 bracket reads off the card.
#include "H4Parser.h"
#include <stdio.h>
#include <string.h>

static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

struct Sink {
    int packets = 0, faults = 0;
    uint8_t lastType = 0; uint8_t lastFault = 0; uint8_t lastFaultByte = 0;
    size_t lastLen = 0; uint8_t last[H4Parser::MAX_PACKET];
    static void onPacket(void *ctx, uint8_t type, const uint8_t *pkt, size_t len) {
        Sink *s = (Sink *)ctx; s->packets++; s->lastType = type; s->lastLen = len; memcpy(s->last, pkt, len);
    }
    static void onFault(void *ctx, uint8_t fault, uint8_t byte) {
        Sink *s = (Sink *)ctx; s->faults++; s->lastFault = fault; s->lastFaultByte = byte;
    }
};

static const uint8_t RESET_CC[] = { 0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00 };

int main() {
    {   // 1. one Command Complete, fed whole
        H4Parser p; Sink s; p.setCallbacks(Sink::onPacket, Sink::onFault, &s);
        p.feed(RESET_CC, sizeof RESET_CC);
        CHECK(s.packets == 1); CHECK(s.faults == 0);
        CHECK(s.lastType == H4Parser::EVENT);
        CHECK(s.lastLen == 6);
        CHECK(memcmp(s.last, RESET_CC + 1, 6) == 0);
        CHECK(p.idle());
    }
    {   // 2. byte-at-a-time delivery is identical
        H4Parser p; Sink s; p.setCallbacks(Sink::onPacket, Sink::onFault, &s);
        for (size_t i = 0; i < sizeof RESET_CC; i++) { p.feed(RESET_CC[i]); CHECK(s.packets == (i + 1 == sizeof RESET_CC ? 1 : 0)); }
        CHECK(s.lastLen == 6);
    }
    {   // 3. two packets back to back in one feed
        H4Parser p; Sink s; p.setCallbacks(Sink::onPacket, Sink::onFault, &s);
        uint8_t two[sizeof RESET_CC * 2]; memcpy(two, RESET_CC, sizeof RESET_CC); memcpy(two + sizeof RESET_CC, RESET_CC, sizeof RESET_CC);
        p.feed(two, sizeof two);
        CHECK(s.packets == 2); CHECK(p.packets() == 2);
    }
    {   // 4. an event with zero parameters completes at the header
        H4Parser p; Sink s; p.setCallbacks(Sink::onPacket, Sink::onFault, &s);
        const uint8_t ev[] = { 0x04, 0x10, 0x00 };            // Hardware Error with no params (illegal but framable)
        p.feed(ev, sizeof ev);
        CHECK(s.packets == 1); CHECK(s.lastLen == 2);
    }
    {   // 5. a bad type byte is a fault, then the stream recovers on the next good packet
        H4Parser p; Sink s; p.setCallbacks(Sink::onPacket, Sink::onFault, &s);
        p.feed(0xFF);
        CHECK(s.faults == 1); CHECK(s.lastFault == H4Parser::BAD_TYPE); CHECK(s.lastFaultByte == 0xFF);
        CHECK(p.idle());
        p.feed(RESET_CC, sizeof RESET_CC);
        CHECK(s.packets == 1); CHECK(p.faults() == 1);
    }
    {   // 6. ACL packet: handle 0x0001, 3 data bytes
        H4Parser p; Sink s; p.setCallbacks(Sink::onPacket, Sink::onFault, &s);
        const uint8_t acl[] = { 0x02, 0x01, 0x00, 0x03, 0x00, 0xAA, 0xBB, 0xCC };
        p.feed(acl, sizeof acl);
        CHECK(s.packets == 1); CHECK(s.lastType == H4Parser::ACL); CHECK(s.lastLen == 7);
        CHECK(s.last[4] == 0xAA && s.last[6] == 0xCC);
    }
    {   // 7. ACL length above the plausibility bound is a fault, not a 64 KB wait
        H4Parser p; Sink s; p.setCallbacks(Sink::onPacket, Sink::onFault, &s);
        p.setAclMax(1021);
        const uint8_t acl[] = { 0x02, 0x01, 0x00, 0xFE, 0x03 };   // 1022 > 1021
        p.feed(acl, sizeof acl);
        CHECK(s.faults == 1); CHECK(s.lastFault == H4Parser::BAD_LENGTH); CHECK(p.idle());
        const uint8_t ok[] = { 0x02, 0x01, 0x00, 0xFD, 0x03 };    // 1021 is allowed
        p.feed(ok, sizeof ok);
        CHECK(s.faults == 1); CHECK(!p.idle());
    }
    {   // 8. setAclMax cannot exceed the buffer
        H4Parser p; p.setAclMax(0xFFFF);
        Sink s; p.setCallbacks(Sink::onPacket, Sink::onFault, &s);
        const uint8_t acl[] = { 0x02, 0x01, 0x00, 0x05, 0x04 };   // 1029 > MAX_PACKET - 4
        p.feed(acl, sizeof acl);
        CHECK(s.faults == 1);
    }
    {   // 9. a 255-byte event (Remote Name Request Complete) fits
        H4Parser p; Sink s; p.setCallbacks(Sink::onPacket, Sink::onFault, &s);
        uint8_t ev[3 + 255] = { 0x04, 0x07, 0xFF };
        p.feed(ev, sizeof ev);
        CHECK(s.packets == 1); CHECK(s.lastLen == 257);
    }
    printf("h4parser_test: %d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
