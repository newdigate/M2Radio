// L2cap -- basic-mode L2CAP over one ACL link: the signalling channel, up to
// MAX_CHANNELS connection-oriented channels, ACL demux by CID, and ACL credit
// accounting from Number_Of_Completed_Packets.  Pure C++, no heap.  RX entry
// points only RECORD; every byte is transmitted from service() -- writing to
// the transport from the RX pump bus-faults (B6, 2026-08-28).  MIT, clean-room.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "HciIo.h"
class L2cap {
public:
    static const uint8_t MAX_CHANNELS = 3;      // SDP, AVDTP signalling, AVDTP media
    static const uint8_t MAX_OPTS = 32;
    enum State : uint8_t { FREE, WAIT_CONN, CONFIG, OPEN, CLOSED };
    struct Channel {
        State state; uint16_t psm, localCid, remoteCid, mtuOut, mtuIn;
        bool cfgReqSent, cfgRspRcvd, cfgReqSeen, cfgRspSent; uint8_t cfgReqId, optLen; uint8_t opts[MAX_OPTS];
        bool peerInitiated;
    };
    typedef void (*DataFn)(void *ctx, Channel &ch, const uint8_t *payload, uint16_t len);
    explicit L2cap(HciIo &io) : m_io(io) {}
    void begin(uint16_t aclHandle, uint8_t aclCredits, uint16_t aclMax = 1021);
    void onData(DataFn fn, void *ctx) { m_onData = fn; m_dataCtx = ctx; }
    // --- RX (record only) ---
    void onAcl(uint16_t handle, const uint8_t *d, uint16_t len);   // Hci::AclFn payload
    void onEvent(uint8_t code, const uint8_t *p, uint8_t len);      // needs 0x13 only
    // --- main context ---
    Channel *connect(uint16_t psm, uint16_t localCid);              // sends Connection Request on service()
    bool     send(uint16_t remoteCid, const uint8_t *payload, uint16_t len); // queued, credit-paced
    void     service();
    Channel *byLocal(uint16_t cid); Channel *byRemote(uint16_t cid); Channel *byPsm(uint16_t psm);
    uint8_t  credits() const { return m_credits; }
    uint32_t dropped() const { return m_dropped; }
    void     acceptIncoming(bool yes) { m_accept = yes; }          // peer-initiated channels (answered with our next free CID)
private:
    struct Tx { uint16_t cid; uint16_t len; uint8_t buf[700]; bool used; };
    static const uint8_t TXQ = 8;
    void sig(const uint8_t *cmd, uint16_t len);                     // queue a signalling command
    void handleSig(const uint8_t *d, uint16_t len);
    HciIo &m_io; uint16_t m_handle, m_aclMax; uint8_t m_credits; bool m_accept;
    Channel m_ch[MAX_CHANNELS]; uint8_t m_nextId; uint16_t m_nextCid;
    Tx m_txq[TXQ]; uint8_t m_txHead, m_txCount; uint32_t m_dropped;
    struct Pending { bool infoReq; uint8_t infoId; uint16_t infoType; bool echoReq; uint8_t echoId;
                     bool connReq; uint8_t connId; uint16_t connPsm, connScid; } m_p;
    DataFn m_onData; void *m_dataCtx;
};
