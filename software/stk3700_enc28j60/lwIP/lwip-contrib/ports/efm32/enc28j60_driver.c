/**************************************************************************//**
 * @file
 * @brief driver wrapper for Ethernet controller Micrel KSZ8851SNL used in lwIP
 *
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/

#include <stdint.h>

#ifndef ETH_USE_TINY_PRINTF
#include <stdio.h>
#else
#include "printf.h"
#endif

#include "enc28j60_driver.h"

#include "enc28j60.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/stats.h"

#include "em_device.h"
#include "em_int.h"
#include "em_gpio.h"
#include "bsp.h"
#include "lwipopts.h"

#ifndef LWIP4EFM32
#error "lwiopts.h for EFM32 are not included"
#endif

static sys_mutex_t enc28j60_mutex;

/****************************************************************************//**
 * @brief
 *   Initialize lwIP internal variables and structures.
 *
 * @note
 *   It should be passed to netif_add.
 *
 * @param[in] netif
 *   the used network interface
 *
 * @return
 *   error status
 *****************************************************************************/
err_t enc28j60_driver_init(struct netif *netif)
{
  ENC28J60_Config config;
  config.macAddress = netif->hwaddr;

  EFM_ASSERT(netif != NULL);

#if LWIP_NETIF_HOSTNAME
  netif->hostname = "efm32";               /* initialize interface hostname */
#endif

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;

  /* Set the output methods to be used */
  netif->output     = etharp_output;
  netif->linkoutput = enc28j60_driver_output;

  /* Set the HW address length */
  netif->hwaddr_len = ETH_MAC_ADDR_LEN;

  /* Set the maximum transfer unit */
  netif->mtu = 1500;

  /* Set the flags according to device capabilities */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  /* Initialize the hardware */
  ENC28J60_Init(&config);

  if (sys_mutex_new(&enc28j60_mutex) != ERR_OK) {
    LWIP_ASSERT("failed to create enc28j60_mutex", 0);
  }

  return ERR_OK;
}


/****************************************************************************//**
 * @brief
 *   Transmit a packet.
 *
 * @param[in] netif
 *   The network interface
 *
 * @param[in] p
 *   The packet to be sent
 *
 * @return
 *   error status
 *****************************************************************************/
err_t enc28j60_driver_output(struct netif *netif, struct pbuf *p)
{
  struct eth_hdr *ethhdr = p->payload;

  EFM_ASSERT(netif != NULL);
  EFM_ASSERT(p != NULL);
  EFM_ASSERT(ethhdr != NULL);

  sys_mutex_lock(&enc28j60_mutex);
  /* Tx must be done with interrupts disabled */
  INT_Disable();
  /* Check for LwIP chained buffers */
  /* Figure out how to support linked packets */
  EFM_ASSERT(p->next == NULL);
  ENC28J60_Transmit(p->payload, p->len);
  INT_Enable();
  sys_mutex_unlock(&enc28j60_mutex);
  
  LINK_STATS_INC(link.xmit);
  return ERR_OK;
}

void enc28j60_driver_input(struct netif *netif)
{
  struct eth_hdr *ethhdr;
  struct pbuf    *p;
  err_t          err;

  (void) *ethhdr;
  (void) *p;
  (void) err;
  EFM_ASSERT(netif != NULL);

  p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
  if (p == NULL)
  {
    return;
  }
  
  sys_mutex_lock(&enc28j60_mutex);
  uint32_t n = ENC28J60_Receive(p->payload, p->len);
  sys_mutex_unlock(&enc28j60_mutex);

  if (n == 0)
  {
    pbuf_free(p);
    return;
  }

  p->len = n;
  p->tot_len = n;

  /* Check that we got an IPv4 or ARP packet */
  ethhdr = p->payload;

  switch (htons(ethhdr->type))
  {
  case ETHTYPE_IPv4:
  case ETHTYPE_ARP:
    err = netif->input(p, netif);
    if (err != ERR_OK)
    {
      pbuf_free(p);
    }
    break;
  default:
    pbuf_free(p);
    break;
  }

}

/***************************************************************************//**
 * @brief
 *   Handle an interrupt from the enc28j60 ethernet controller.
 ******************************************************************************/
void enc28j60_driver_isr(struct netif *netif)
{

}

