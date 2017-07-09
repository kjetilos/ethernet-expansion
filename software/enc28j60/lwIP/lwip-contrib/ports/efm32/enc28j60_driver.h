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
#ifndef _LWIP_DRIVER_H_
#define _LWIP_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/**************************************************************************//**
* @addtogroup Lwip
* @{
******************************************************************************/

#include <stdbool.h>
#include "netif/etharp.h"


#define ETH_MAC_ADDR_LEN    6   /**< MAC Address length */
#define ETH_HEADER_LEN      14  /**< Ethernet frame header length */

#define IFNAME0             'e' /**< Interface first byte of name */
#define IFNAME1             'n' /**< Interface second byte of name */

err_t enc28j60_driver_init(struct netif *netif);
void  enc28j60_driver_input(struct netif *netif);
err_t enc28j60_driver_output(struct netif *netif, struct pbuf *p);
void  enc28j60_driver_isr(struct netif *netif);

/** @} (end group Lwip) */
/** @} (end group Drivers) */

#ifdef __cplusplus
}
#endif

#endif /* _LWIP_DRIVER_H_ */
