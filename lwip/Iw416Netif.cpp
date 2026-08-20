#include <string.h>
#include "Iw416Netif.h"
#include "lwip/opt.h"
#include "lwip/pbuf.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"
#include "Iw416.h"

extern "C" unsigned char g_mac[6];   // filled by the sketch (getHwSpec MAC)

#define IW416IF_MAX_FRAME 1536
static uint8_t s_txbuf[IW416IF_MAX_FRAME];

static err_t low_level_output(struct netif *netif, struct pbuf *p) {
    Iw416 *iw = (Iw416 *)netif->state;
    if (p->tot_len > IW416IF_MAX_FRAME) return ERR_IF;
    pbuf_copy_partial(p, s_txbuf, p->tot_len, 0);
    return (iw->sendDataFrame(s_txbuf, (uint16_t)p->tot_len) == SdioHost::OK)
               ? ERR_OK : ERR_IF;
}

err_t iw416NetifInit(struct netif *netif) {
    // W16: this netif is the caller TX aggregation was built for, and the one
    // that can safely turn it on -- because iw416NetifPoll() below flushes on
    // every iteration, so a staged frame is never held longer than one pass of
    // the application's loop.  The driver ships it OFF precisely because that
    // guarantee is the caller's to make, not the driver's: with it on,
    // sendDataFrame() means "queued", and a caller that sends and then blocks
    // without flushing waits forever.
    ((Iw416 *)netif->state)->setTxAggregation(true);
    netif->name[0] = 'w'; netif->name[1] = 'l';
    netif->output     = etharp_output;
    netif->linkoutput = low_level_output;
    netif->mtu        = 1500;
    netif->hwaddr_len = ETH_HWADDR_LEN;
    memcpy(netif->hwaddr, g_mac, ETH_HWADDR_LEN);
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
    return ERR_OK;
}

static void frameSink(void *vctx, const uint8_t *frame, uint16_t len) {
    struct netif *nif = (struct netif *)vctx;
    struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    if (p == NULL) return;                 // pool exhausted: drop this frame
    pbuf_take(p, frame, len);
    if (nif->input(p, nif) != ERR_OK) pbuf_free(p);
}

bool iw416NetifPoll(struct netif *netif) {
    Iw416 *iw = (Iw416 *)netif->state;
    bool dropped = false;
    // W16, BEFORE the service pass: anything lwip queued since the last poll
    // -- a segment from sys_check_timeouts(), an application write -- goes out
    // now rather than waiting behind however long this pass takes.
    (void)iw->flushTx();
    SdioHost::Status st = iw->serviceLink(frameSink, netif, &dropped, 0);
    // A bus error (anything but OK/CMD_TIMEOUT) has to present as link-down
    // too: otherwise a wedged card just stops incrementing rx/tx counters
    // while the netif and DHCP lease keep sitting there looking healthy.
    // ...and AFTER it: the sink hands frames straight to lwip, which answers
    // synchronously (a TCP ACK, an ARP reply), so the replies to everything
    // this pass delivered are staged by the time it returns.  Two flushes, each
    // a no-op when nothing is queued, is what keeps a staged frame's latency
    // bounded by ONE pass rather than by one whole application loop.
    //
    // ★ THE FLUSH MUST NOT PRE-EMPT THE LINK-DOWN CHECK BELOW.  Its status is
    // deliberately discarded: a TX failure is already visible in the data-path
    // counters, and the link verdict this function returns is about the SERVICE
    // pass -- letting a failed data write mark the link down would make a full
    // ring look like a dead card.
    (void)iw->flushTx();
    if (dropped || (st != SdioHost::OK && st != SdioHost::CMD_TIMEOUT)) {
        netif_set_link_down(netif);
        return false;
    }
    return true;
}
