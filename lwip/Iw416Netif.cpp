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
    (void)iw->serviceLink(frameSink, netif, &dropped, 0);
    if (dropped) { netif_set_link_down(netif); return false; }
    return true;
}
