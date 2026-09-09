// Host tests for A2dpSource as ONE bring-up ATTEMPT (NEW-34 piece 2).  The bonded-candidate WALK
// that this file used to test moved to BtSession (btsession_test, Task 6); what stays here is a
// single attempt driven by start(Target)+tick(now): a PAGE attempt that reaches PAIR_FAILED with
// the link torn down, an INBOUND attempt driven to STREAMING by the AVDTP acceptor (with the peer's
// SET_CONFIGURATION bitpool adopted into sbcParams()), the acceptor's self-START when the peer opens
// media but sends no START, and a link loss during STREAMING that tears the media path down (LOST).
#include "A2dpSource.h"
#include <stdio.h>
#include <string.h>
#include <vector>
#include <deque>
#include <string>
#include <functional>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
struct FakeIo : HciIo {
    std::deque<uint8_t> rx; std::vector<uint8_t> acc; uint32_t now = 0;
    std::vector<std::pair<uint16_t, std::vector<uint8_t> > > cmds;
    std::vector<std::vector<uint8_t> > aclOut;                 // outgoing L2CAP PDUs (len(2)+cid(2)+payload), in order
    std::function<void(FakeIo &, uint16_t, const std::vector<uint8_t> &)> onCmd;
    size_t write(const uint8_t *p, size_t n) override { acc.insert(acc.end(), p, p + n); parse(); return n; }
    void parse() {
        for (;;) {
            if (acc.size() >= 4 && acc[0] == 0x01) {          // HCI command: [01][op lo][op hi][plen][params]
                uint8_t plen = acc[3]; if (acc.size() < 4u + plen) break;
                uint16_t op = (uint16_t)(acc[1] | (acc[2] << 8)); std::vector<uint8_t> prm(acc.begin() + 4, acc.begin() + 4 + plen);
                acc.erase(acc.begin(), acc.begin() + 4 + plen); cmds.push_back(std::make_pair(op, prm)); if (onCmd) onCmd(*this, op, prm);
            } else if (acc.size() >= 5 && acc[0] == 0x02) {   // ACL out: [02][handle lo][handle hi][alen lo][alen hi][L2CAP PDU]
                uint16_t dlen = (uint16_t)(acc[3] | (acc[4] << 8)); if (acc.size() < 5u + dlen) break;
                aclOut.push_back(std::vector<uint8_t>(acc.begin() + 5, acc.begin() + 5 + dlen));
                acc.erase(acc.begin(), acc.begin() + 5 + dlen);
            } else break;
        }
    }
    int available() override { return (int)rx.size(); }
    int read() override { if (rx.empty()) return -1; uint8_t b = rx.front(); rx.pop_front(); return b; }
    uint32_t nowMs() override { return now; }
    void ev(uint8_t code, std::vector<uint8_t> p) { rx.push_back(0x04); rx.push_back(code); rx.push_back((uint8_t)p.size()); rx.insert(rx.end(), p.begin(), p.end()); }
    void cc(uint16_t op, std::vector<uint8_t> ret, uint8_t ncmd = 1) { std::vector<uint8_t> p = { ncmd, (uint8_t)op, (uint8_t)(op >> 8) }; p.insert(p.end(), ret.begin(), ret.end()); ev(0x0E, p); }
    void cs(uint16_t op, uint8_t status = 0, uint8_t ncmd = 1) { ev(0x0F, { status, ncmd, (uint8_t)op, (uint8_t)(op >> 8) }); }
    int count(uint16_t op) { int n = 0; for (auto &c : cmds) if (c.first == op) n++; return n; }
};
static FakeIo *g_io = nullptr; static Hci *g_hci = nullptr; static A2dpSource *g_src = nullptr;
// One loop pass: advance fake time, pump the HCI (dispatches events), tick the attempt, service it.
static void step() { g_io->now += 10; g_hci->service(); if (g_src) { g_src->tick(g_io->now); g_src->service(); } }
static bool runUntil(std::function<bool()> pred, uint32_t ms) {
    uint32_t end = g_io->now + ms;
    while (g_io->now < end) { if (pred()) return true; step(); }
    return pred();
}
static uint32_t fakeNow() { return g_io->now; }
static void evThunk(void *ctx, uint8_t code, const uint8_t *p, uint8_t len) { ((A2dpSource *)ctx)->onEvent(code, p, len); }
static std::vector<std::string> g_log;
static void logFn(void *, const char *line) { g_log.push_back(std::string(line)); }
static int logCount(const char *needle) { int n = 0; for (auto &l : g_log) if (l.find(needle) != std::string::npos) n++; return n; }
static const uint8_t SHOKZ[6] = { 0x2F, 0x29, 0x31, 0xB3, 0x86, 0xC0 };
static Bond mk(const uint8_t bd[6], const char *name, uint8_t psrm = 1) {
    Bond b; memset(&b, 0, sizeof b); memcpy(b.bd, bd, 6); memset(b.key, 0x5A, 16); b.keyType = 4; b.psrm = psrm; BondTable::copyName(b.name, name); return b;
}
// Deliver an inbound L2CAP PDU on `cid` (as A2dpSource::onAcl would receive it from Hci).
static void feedAcl(A2dpSource &src, uint16_t cid, std::vector<uint8_t> pl) {
    std::vector<uint8_t> v = { (uint8_t)pl.size(), (uint8_t)(pl.size() >> 8), (uint8_t)cid, (uint8_t)(cid >> 8) };
    v.insert(v.end(), pl.begin(), pl.end()); src.onAcl(0x0001, v.data(), (uint16_t)v.size());
}
// The DESTINATION CID of the most recent outgoing AVDTP response carrying signal id `sig`; 0xFFFF if none.
// Read off the RAW ACL stream, so a reply addressed to a STALE (or zero) channel is visible rather than
// filtered out.  L2CAP's own signalling channel (cid 1) is skipped: its [code][id] can collide with a signal id.
static uint16_t cidOfLastAvdtpAccept(FakeIo &io, uint8_t sig) {
    for (size_t i = io.aclOut.size(); i-- > 0;) { const std::vector<uint8_t> &p = io.aclOut[i];
        if (p.size() < 6) continue;
        uint16_t cid = (uint16_t)(p[2] | (p[3] << 8));
        if (cid == 0x0001) continue;
        if (p[5] == sig && (p[4] & 0x03) == Avdtp::ACCEPT) return cid; }
    return 0xFFFF;
}
// Transaction label of the most recent OUTGOING AVDTP PDU carrying signal id `sig`; -1 if none.
static int lastAvdtpTl(FakeIo &io, uint8_t sig) {
    for (size_t i = io.aclOut.size(); i-- > 0;) { const std::vector<uint8_t> &pdu = io.aclOut[i];
        if (pdu.size() >= 6 && pdu[5] == sig) return pdu[4] >> 4; }
    return -1;
}
// The controller for the PAGE scenario: setup succeeds; Create_Connection to `g_present` succeeds
// (handle 1); Authentication_Requested is refused (0x0C), so the attempt ends PAIR_FAILED after the
// link comes up; Disconnect completes.
static const uint8_t *g_present = nullptr;
static void controller(FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
    if (op == 0x0C01 || op == 0x0C56 || op == 0x0C18) { f.cc(op, { 0x00 }); return; }                    // PREPARE
    if (op == 0x0405) { f.cs(op);                                                                          // Create_Connection
        std::vector<uint8_t> r = { (uint8_t)((g_present && memcmp(prm.data(), g_present, 6) == 0) ? 0x00 : 0x04), 0x01, 0x00 };
        r.insert(r.end(), prm.begin(), prm.begin() + 6); r.push_back(0x01); r.push_back(0x00); f.ev(0x03, r); return; }
    if (op == 0x0406) { f.cs(op); f.ev(0x05, { 0x00, prm[0], prm[1], 0x16 }); return; }                   // Disconnect
    if (op == 0x0411) { f.cs(op, 0x0C); return; }                                                          // Authentication_Requested refused
    f.cc(op, { 0x00 });
}
struct Rig { FakeIo io; Hci hci; A2dpSource src; BondTable bonds;
    Rig() : hci(io), src(hci, io) { g_io = &io; g_hci = &hci; g_src = &src; hci.onEvent(evThunk, &src); src.setLog(logFn, nullptr); io.onCmd = controller; g_log.clear(); } };
// Bring an INBOUND link up: PREPARE, then an incoming page (Connection_Request accepted because the
// peer is bonded, then Connection_Complete).  Uses an inbound controller (no page/auth/inquiry).
static void inboundLinkUp(Rig &r) {
    r.io.onCmd = [](FakeIo &f, uint16_t op, const std::vector<uint8_t> &prm) {
        if (op == 0x0C01 || op == 0x0C56 || op == 0x0C18) { f.cc(op, { 0x00 }); return; }                 // PREPARE
        if (op == 0x0409) { f.cs(op); return; }                                                            // Accept_Connection_Request
        if (op == 0x0406) { f.cs(op); f.ev(0x05, { 0x00, prm[0], prm[1], 0x16 }); return; }                // Disconnect
        f.cc(op, { 0x00 }); };
    r.bonds.upsert(mk(SHOKZ, "OpenMove by Shokz")); r.src.setBonds(&r.bonds);
    r.src.begin(fakeNow(), 100);
    CHECK(runUntil([&]{ return !r.src.link().busy(); }, 5000));                                            // PREPARE done
    std::vector<uint8_t> creq(SHOKZ, SHOKZ + 6); creq.push_back(0x18); creq.push_back(0x04); creq.push_back(0x24); creq.push_back(0x01);  // cod A/V, ACL
    r.io.ev(0x04, creq); step();                                                                           // Connection_Request -> Accept
    std::vector<uint8_t> cc = { 0x00, 0x01, 0x00 }; cc.insert(cc.end(), SHOKZ, SHOKZ + 6); cc.push_back(0x01); cc.push_back(0x00);
    r.io.ev(0x03, cc); step();                                                                             // Connection_Complete -> inboundUp
}
// Drive an inbound attempt through the acceptor to the point where the media channel is OPEN and the
// SEP is configured at bitpool 35 (state OPENING, awaiting START).  Returns the signalling localCid.
static uint16_t inboundToMediaOpen(Rig &r) {
    inboundLinkUp(r);
    A2dpSource::Target t{}; t.kind = A2dpSource::Target::INBOUND; memcpy(t.bd, SHOKZ, 6);
    CHECK(r.src.start(t));
    r.io.ev(0x08, { 0x00, 0x01, 0x00, 0x01 });                                                            // peer secures the link (Encryption_Change)
    CHECK(runUntil([&]{ return r.src.state() == A2dpSource::AVDTP_WAIT; }, 5000));
    feedAcl(r.src, 0x0001, { 0x02, 0x20, 4, 0, 0x19, 0x00, 0xC0, 0x00 }); step();                          // peer opens AVDTP signalling
    L2cap::Channel *sig = r.src.l2().byRemote(0x00C0); CHECK(sig);
    uint16_t sc = sig ? sig->localCid : 0;
    feedAcl(r.src, 0x0001, { 0x04, 0x21, 8, 0, (uint8_t)sc, (uint8_t)(sc >> 8), 0, 0, 0x01, 0x02, 0x7F, 0x03 });
    feedAcl(r.src, 0x0001, { 0x05, 0x22, 6, 0, (uint8_t)sc, (uint8_t)(sc >> 8), 0, 0, 0, 0 }); step();
    CHECK(runUntil([&]{ return r.src.avdtp().role() == Avdtp::ACCEPTOR; }, 3000));
    CHECK(r.src.state() == A2dpSource::AVDTP);
    feedAcl(r.src, sc, { 0x30, 0x01 }); step();                                                            // DISCOVER
    feedAcl(r.src, sc, { 0x40, 0x0C, 1 << 2 }); step();                                                    // GET_ALL_CAPABILITIES
    feedAcl(r.src, sc, { 0x50, 0x03, 1 << 2, 5 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x23 }); step();  // SET_CONFIGURATION bitpool 35
    feedAcl(r.src, sc, { 0x60, 0x06, 1 << 2 }); step();                                                    // OPEN
    feedAcl(r.src, 0x0001, { 0x02, 0x23, 4, 0, 0x19, 0x00, 0xC1, 0x00 }); step();                          // peer opens the media channel
    L2cap::Channel *med = r.src.l2().byRemote(0x00C1); CHECK(med);
    uint16_t mc = med ? med->localCid : 0;
    feedAcl(r.src, 0x0001, { 0x04, 0x24, 8, 0, (uint8_t)mc, (uint8_t)(mc >> 8), 0, 0, 0x01, 0x02, 0x7F, 0x03 });
    feedAcl(r.src, 0x0001, { 0x05, 0x25, 6, 0, (uint8_t)mc, (uint8_t)(mc >> 8), 0, 0, 0, 0 }); step();
    CHECK(runUntil([&]{ return r.src.mediaCid() != 0; }, 3000));                                           // adoptInbound picks up media
    return sc;
}
int main() {
    {   // 1. A PAGE attempt: page the target, pair (controller refuses auth) -> PAIR_FAILED, torn down.
        Rig r; g_present = SHOKZ;
        A2dpSource::Target t{}; t.kind = A2dpSource::Target::PAGE; memcpy(t.bd, SHOKZ, 6); t.psrm = 1; t.attempts = 1;
        r.src.begin(fakeNow(), 0); CHECK(r.src.start(t));
        CHECK(runUntil([&]{ return !r.src.busy(); }, 40000) && r.src.result() == A2dpSource::PAIR_FAILED);
        CHECK(r.io.count(0x0401) == 0 && r.io.count(0x0405) == 1 && r.io.count(0x0406) == 1);              // paged, no inquiry, torn down
    }
    {   // 2. An INBOUND attempt: the peer drives the acceptor to STREAMING, and its SET_CONFIGURATION
        //    bitpool (35) is ADOPTED into sbcParams() -- not the initiator default of 53.
        Rig r; uint16_t sc = inboundToMediaOpen(r);
        feedAcl(r.src, sc, { 0x70, 0x07, 1 << 2 }); step();                                                // peer START
        CHECK(runUntil([&]{ return r.src.state() == A2dpSource::STREAMING; }, 3000));
        CHECK(r.src.result() == A2dpSource::OK && r.src.started());
        CHECK(r.src.sbcParams().bitpool == 35 && r.src.sbcParams().mode == Sbc::JOINT_STEREO);
        CHECK(logCount("adopted") == 1);
        CHECK(r.src.mediaCid() == 0x00C1);
    }
    {   // 3. Self-START: media is OPEN, the peer sends NO START; after START_WAIT_MS we START ourselves,
        //    and the peer's ACCEPT then takes us to STREAMING.
        Rig r; uint16_t sc = inboundToMediaOpen(r);
        CHECK(r.src.avdtp().state() == Avdtp::OPENING && r.src.avdtp().mediaReady());
        CHECK(runUntil([&]{ return r.src.avdtp().state() == Avdtp::STARTING; }, 3000));                    // self-START fired
        int tl = lastAvdtpTl(r.io, 0x07); CHECK(tl >= 0);
        feedAcl(r.src, sc, { (uint8_t)((tl << 4) | 0x02), 0x07 }); step();                                 // peer ACCEPTs our START
        CHECK(runUntil([&]{ return r.src.state() == A2dpSource::STREAMING; }, 3000));
        CHECK(r.src.started() && r.src.result() == A2dpSource::OK);
    }
    {   // 4. A link loss during STREAMING resets L2cap + Avdtp and ends LOST (teardown ran).
        Rig r; uint16_t sc = inboundToMediaOpen(r);
        feedAcl(r.src, sc, { 0x70, 0x07, 1 << 2 }); step();
        CHECK(runUntil([&]{ return r.src.state() == A2dpSource::STREAMING; }, 3000));
        r.io.ev(0x05, { 0x00, 0x01, 0x00, 0x08 });                                                         // Disconnection_Complete, handle 1
        CHECK(runUntil([&]{ return !r.src.busy(); }, 3000));
        CHECK(r.src.result() == A2dpSource::LOST && r.src.state() == A2dpSource::DONE);
        CHECK(r.src.l2().byPsm(Avdtp::PSM) == nullptr && r.src.l2().byPsm(Sdp::PSM) == nullptr);           // media path torn down
    }
    {   // A fresh attempt must ACK a prior link loss (regression for A2dpSource::start()'s ackLost()).  The
        //   DISCONNECTING teardown is EXCLUDED from tick()'s loss-check, so a first attempt that fails at
        //   pairing leaves BtLink in LINK_LOST; without the ack, the NEXT start()'s first LINKING tick aborts
        //   LOST before it even pages -- so the second attempt must reach PAIR_FAILED (link came up), not LOST.
        Rig r; g_present = SHOKZ;
        r.src.begin(fakeNow(), 100);
        CHECK(runUntil([&]{ return !r.src.link().busy(); }, 5000));                                         // PREPARE done
        A2dpSource::Target t1{}; t1.kind = A2dpSource::Target::PAGE; memcpy(t1.bd, SHOKZ, 6); t1.psrm = 1; t1.attempts = 1;
        CHECK(r.src.start(t1));
        CHECK(runUntil([&]{ return !r.src.busy(); }, 40000) && r.src.result() == A2dpSource::PAIR_FAILED);  // link up, auth refused, torn down
        CHECK(r.src.link().lost());                                                                         // the stale LINK_LOST precondition
        A2dpSource::Target t2 = t1;
        CHECK(r.src.start(t2));
        CHECK(runUntil([&]{ return !r.src.busy(); }, 40000));
        CHECK(r.src.result() == A2dpSource::PAIR_FAILED);                                                   // NOT LOST: start() acked the stale loss and the attempt paged
    }
    {   // An OUTBOUND attempt must NOT inherit a prior INBOUND attempt's adopted bitpool.  m_params is written
        //   only by adoptConfig() (inbound); without a reset in start(), an outbound reconnect after an inbound
        //   stream (lifecycle leg 3) encodes the adopted bitpool (35) instead of the initiator default (53).
        //   (Found by the [lifecycle] gate; the other host scenarios only ever run a single attempt.)
        Rig r; uint16_t sc = inboundToMediaOpen(r);
        feedAcl(r.src, sc, { 0x70, 0x07, 1 << 2 }); step();
        CHECK(runUntil([&]{ return r.src.state() == A2dpSource::STREAMING; }, 3000));
        CHECK(r.src.sbcParams().bitpool == 35);                                   // inbound adopted the peer's config
        r.src.stop();
        CHECK(runUntil([&]{ return !r.src.busy(); }, 3000));                      // torn down
        A2dpSource::Target t{}; t.kind = A2dpSource::Target::PAGE; memcpy(t.bd, SHOKZ, 6); t.psrm = 1; t.attempts = 1;
        CHECK(r.src.start(t));
        CHECK(r.src.sbcParams().bitpool == 53);                                   // OUTBOUND resets to the initiator default, NOT 35
    }
    {   // A fresh attempt must reset Avdtp/Avrcp.  Only tick()'s LOSS branch does, so an attempt that ended any
        //   other way (STOPPED here; CONNECT/PAIR/L2CAP/AVDTP_FAILED alike) leaves Avdtp::m_sig and m_role
        //   stale -- adoptInbound()'s `if (!m_sig)` guard then skips adoption, the NEXT inbound attempt believes
        //   it is already the ACCEPTOR, and its DISCOVER reply is addressed to the dead channel's remote cid,
        //   which is 0x0000 once L2cap::begin() has zeroed the table.  (a2dpsink_test K4 is the same defect.)
        Rig r; inboundToMediaOpen(r);
        r.src.stop();
        CHECK(runUntil([&]{ return !r.src.busy(); }, 3000));
        CHECK(r.src.result() == A2dpSource::STOPPED);
        std::vector<uint8_t> creq(SHOKZ, SHOKZ + 6); creq.push_back(0x18); creq.push_back(0x04); creq.push_back(0x24); creq.push_back(0x01);
        r.io.ev(0x04, creq); step();                                                                   // a SECOND incoming page
        std::vector<uint8_t> cc = { 0x00, 0x01, 0x00 }; cc.insert(cc.end(), SHOKZ, SHOKZ + 6); cc.push_back(0x01); cc.push_back(0x00);
        r.io.ev(0x03, cc); step();
        A2dpSource::Target t{}; t.kind = A2dpSource::Target::INBOUND; memcpy(t.bd, SHOKZ, 6);
        CHECK(r.src.start(t));
        r.io.ev(0x08, { 0x00, 0x01, 0x00, 0x01 }); step();                                             // peer secures the new link
        CHECK(runUntil([&]{ return r.src.state() == A2dpSource::AVDTP_WAIT; }, 5000));
        // The peer re-reads our SDP record BEFORE opening AVDTP, so the signalling channel lands in a DIFFERENT
        // L2cap slot than last time.  That is what makes the stale m_sig visible: with both attempts opening AVDTP
        // first the pointer happens to land back on the same (now correct) slot and the defect hides.
        feedAcl(r.src, 0x0001, { 0x02, 0x2D, 4, 0, 0x01, 0x00, 0xC3, 0x00 }); step();                  // peer opens SDP
        L2cap::Channel *sdp2 = r.src.l2().byRemote(0x00C3); CHECK(sdp2);
        uint16_t dc = sdp2 ? sdp2->localCid : 0;
        feedAcl(r.src, 0x0001, { 0x04, 0x2E, 8, 0, (uint8_t)dc, (uint8_t)(dc >> 8), 0, 0, 0x01, 0x02, 0x7F, 0x03 });
        feedAcl(r.src, 0x0001, { 0x05, 0x2F, 6, 0, (uint8_t)dc, (uint8_t)(dc >> 8), 0, 0, 0, 0 }); step();
        feedAcl(r.src, 0x0001, { 0x02, 0x30, 4, 0, 0x19, 0x00, 0xC2, 0x00 }); step();                  // peer opens AVDTP signalling anew
        L2cap::Channel *sig = r.src.l2().byRemote(0x00C2); CHECK(sig);
        uint16_t sc2 = sig ? sig->localCid : 0;
        feedAcl(r.src, 0x0001, { 0x04, 0x31, 8, 0, (uint8_t)sc2, (uint8_t)(sc2 >> 8), 0, 0, 0x01, 0x02, 0x7F, 0x03 });
        feedAcl(r.src, 0x0001, { 0x05, 0x32, 6, 0, (uint8_t)sc2, (uint8_t)(sc2 >> 8), 0, 0, 0, 0 }); step();
        CHECK(runUntil([&]{ return r.src.avdtp().role() == Avdtp::ACCEPTOR; }, 3000));
        feedAcl(r.src, sc2, { 0x30, 0x01 }); step();                                                   // DISCOVER
        CHECK(runUntil([&]{ return cidOfLastAvdtpAccept(r.io, 0x01) != 0xFFFF; }, 500));
        CHECK(cidOfLastAvdtpAccept(r.io, 0x01) == 0x00C2);                                             // the NEW channel, not 0x0000
    }
    {   // The media channel opens LATE, in the SAME tick that answers START -- a2dpsink_test K8's scenario, on
        //   the SOURCE.  The peer's media channel reaches L2CAP OPEN only AFTER the attempt has left AVDTP for
        //   STREAMING -- its CONN_REQ lands before START but its config exchange completes later.  From the
        //   next tick on the attempt is in STREAMING, which called adoptInbound() nowhere -- so mediaRemoteCid()
        //   stayed 0 forever and every media PDU missed the media route.  The sink adopts in STREAMING; the
        //   source now does too.  Note the ONE tick of slack the source has that the sink has not: its AVDTP
        //   case adopts BEFORE it tests started(), so a channel that opens in the very tick that answers START
        //   is still picked up next tick.  Two ticks late is the window that was never closed, which is what
        //   this drives -- the peer STARTs with its own media channel still in L2CAP CONFIG.
        Rig r; inboundLinkUp(r);
        A2dpSource::Target t{}; t.kind = A2dpSource::Target::INBOUND; memcpy(t.bd, SHOKZ, 6);
        CHECK(r.src.start(t));
        r.io.ev(0x08, { 0x00, 0x01, 0x00, 0x01 });                                                         // peer secures the link
        CHECK(runUntil([&]{ return r.src.state() == A2dpSource::AVDTP_WAIT; }, 5000));
        feedAcl(r.src, 0x0001, { 0x02, 0x20, 4, 0, 0x19, 0x00, 0xC0, 0x00 }); step();                      // peer opens AVDTP signalling
        L2cap::Channel *sig = r.src.l2().byRemote(0x00C0); CHECK(sig);
        uint16_t sc = sig ? sig->localCid : 0;
        feedAcl(r.src, 0x0001, { 0x04, 0x21, 8, 0, (uint8_t)sc, (uint8_t)(sc >> 8), 0, 0, 0x01, 0x02, 0x7F, 0x03 });
        feedAcl(r.src, 0x0001, { 0x05, 0x22, 6, 0, (uint8_t)sc, (uint8_t)(sc >> 8), 0, 0, 0, 0 }); step();
        CHECK(runUntil([&]{ return r.src.avdtp().role() == Avdtp::ACCEPTOR; }, 3000));
        feedAcl(r.src, sc, { 0x30, 0x01 }); step();
        feedAcl(r.src, sc, { 0x40, 0x0C, 1 << 2 }); step();
        feedAcl(r.src, sc, { 0x50, 0x03, 1 << 2, 5 << 2, 0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x21, 0x15, 0x02, 0x23 }); step();
        feedAcl(r.src, sc, { 0x60, 0x06, 1 << 2 }); step();                                                // OPEN -> Avdtp OPENING
        feedAcl(r.src, 0x0001, { 0x02, 0x23, 4, 0, 0x19, 0x00, 0xC1, 0x00 }); step();                      // media CONN_REQ: the channel exists, still CONFIG
        L2cap::Channel *med = r.src.l2().byRemote(0x00C1); CHECK(med);
        uint16_t mc = med ? med->localCid : 0;
        CHECK(r.src.mediaCid() == 0);                                                                      // ... but not OPEN, so nothing to adopt yet
        feedAcl(r.src, sc, { 0x70, 0x07, 1 << 2 });                                                        // START, with the media config still outstanding
        CHECK(runUntil([&]{ return r.src.state() == A2dpSource::STREAMING; }, 3000));                       // the attempt leaves AVDTP for good
        CHECK(r.src.mediaCid() == 0);
        feedAcl(r.src, 0x0001, { 0x04, 0x24, 8, 0, (uint8_t)mc, (uint8_t)(mc >> 8), 0, 0, 0x01, 0x02, 0x7F, 0x03 });
        feedAcl(r.src, 0x0001, { 0x05, 0x25, 6, 0, (uint8_t)mc, (uint8_t)(mc >> 8), 0, 0, 0, 0 }); step();  // the peer finally configures it: OPEN
        CHECK(r.src.l2().byRemote(0x00C1)->state == L2cap::OPEN);
        CHECK(runUntil([&]{ return r.src.avdtp().mediaRemoteCid() == 0x00C1; }, 3000));                     // RED before the fix: 0 forever
        CHECK(r.src.mediaCid() == 0x00C1);
    }
    printf("a2dpsource_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
