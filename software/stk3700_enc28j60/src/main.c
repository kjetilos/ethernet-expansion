#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_pcnt.h"
#include "bsp.h"
#include "enc28j60_driver.h"
#include "enc28j60.h"

#include <FreeRTOS.h>
#include <task.h>
#include <lwip/init.h>
#include <lwip/ip_addr.h>
#include <lwip/timers.h>
#include <lwip/tcpip.h>
#include <lwip/sockets.h>
#include <stdint.h>
#include <stdbool.h>

static void GpioSetup( void );

#define STACK_SIZE_FOR_TASK    (256)
#define TASK_PRIORITY          (tskIDLE_PRIORITY + 1)

static void udpTask(void *unused);

/* 00:0B:57:33:EF:78 */
static const uint8_t macAddress[] = {0x00, 0x0B, 0x57, 0x33, 0xEF, 0x78};
static struct netif en0 = {0};

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler( void ) __attribute__( ( naked ) );

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
void HardFault_Handler(void)
{
  __asm volatile
  (
    " tst lr, #4                                                \n"
    " ite eq                                                    \n"
    " mrseq r0, msp                                             \n"
    " mrsne r0, psp                                             \n"
    " ldr r1, [r0, #24]                                         \n"
    " ldr r2, handler2_address_const                            \n"
    " bx r2                                                     \n"
    " handler2_address_const: .word prvGetRegistersFromStack    \n"
  );
}

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
  /* These are volatile to try and prevent the compiler/linker optimising them
  away as the variables never actually get used.  If the debugger won't show the
  values of the variables, make them global my moving their declaration outside
  of this function. */
  volatile uint32_t r0;
  volatile uint32_t r1;
  volatile uint32_t r2;
  volatile uint32_t r3;
  volatile uint32_t r12;
  volatile uint32_t lr; /* Link register. */
  volatile uint32_t pc; /* Program counter. */
  volatile uint32_t psr;/* Program status register. */

  r0 = pulFaultStackAddress[ 0 ];
  r1 = pulFaultStackAddress[ 1 ];
  r2 = pulFaultStackAddress[ 2 ];
  r3 = pulFaultStackAddress[ 3 ];

  r12 = pulFaultStackAddress[ 4 ];
  lr = pulFaultStackAddress[ 5 ];
  pc = pulFaultStackAddress[ 6 ];
  psr = pulFaultStackAddress[ 7 ];

  /* When the following line is hit, the variables contain the register values. */
  for( ;; );
}

/**************************************************************************//**
 * @brief Simple task which is blinking led
 * @param *pParameters pointer to parameters passed to the function
 *****************************************************************************/
static void ethernetTask(void *unused)
{
  (void) unused;

  /* Initialise lwIP */
  tcpip_init(NULL, NULL);

  /* Initialize the IP address fields */
  ip_addr_t ipaddr, netmask, gw;

  IP4_ADDR(&ipaddr, 192, 168, 9, 3);
  IP4_ADDR(&netmask, 255, 255, 255, 0);
  IP4_ADDR(&gw, 192, 168, 9, 1);

  memcpy(en0.hwaddr, macAddress, 6);
  /* Add the current network interface to the lwIP list */
  netif_add(&en0, &ipaddr, &netmask, &gw, NULL, enc28j60_driver_init, tcpip_input);
  /* Set the current network interface as default */
  netif_set_default(&en0);
  netif_set_up(&en0);

  xTaskCreate(udpTask, (const char *) "UdpTask", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);

  while (true)
  {
    enc28j60_driver_input(&en0);
    vTaskDelay(10);
//    sys_check_timeouts();
  }
}

#define BUFFER_SIZE 64
static uint8_t buffer[BUFFER_SIZE];

static void udpTask(void *unused)
{
  int ret;
  struct sockaddr_in sockaddr = {0};
  struct sockaddr_in remote = {0};
  socklen_t addrlen;
  int n;
  int sd = socket(PF_INET, SOCK_DGRAM, 0);
  EFM_ASSERT(sd >= 0);
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  sockaddr.sin_port = htons(1234);

  ret = bind(sd, &sockaddr, sizeof(sockaddr));
  EFM_ASSERT(ret >= 0);

  while (true)
  {
    addrlen = sizeof(struct sockaddr_in);
    n = recvfrom(sd, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&remote, &addrlen);
    if (n < 0)
    {
      EFM_ASSERT(false);
    }
    else
    {
      sendto(sd, buffer, n, 0, (struct sockaddr *)&remote, addrlen);
    }
  }
  close(sd);
}

/**
 * Connectors used
 *
 * Pin 20 - Vcc 3V
 * Pin  1 - Gnd
 * Pin  4 - Tx
 * Pin  6 - Rx
 * Pin  8 - Clk
 * Pin 10 - CS
 */

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  /* Switch to high frequency crystal oscillator instead of HFRCO */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Setup GPIO for pushbuttons. */
  GpioSetup();

  BSP_LedsInit();

  /*Create two task for blinking leds*/
  xTaskCreate(ethernetTask, (const char *) "EthernetTask", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);

  /* Start FreeRTOS Scheduler */
  vTaskStartScheduler();
}

/**************************************************************************//**
 * @brief
 *   Setup GPIO interrupt for pushbuttons.
 *****************************************************************************/
static void GpioSetup( void )
{
  /* Enable GPIO clock. */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure PC8 as input and enable interrupt. */
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInputPull, 1);
  GPIO_IntConfig(gpioPortB, 9, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/**************************************************************************//**
 * @brief
 *   Unified GPIO Interrupt handler
 *     PB0 Transmit packet
 *****************************************************************************/
void GPIO_Unified_IRQ(void) {
  /* Get and clear all pending GPIO interrupts */
  uint32_t flags = GPIO_IntGet();
  GPIO_IntClear(flags);

  if (flags & (1 << 9)) {
  }
}

/**************************************************************************//**
* @brief
*   GPIO Interrupt handler for even pins
*****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

/**************************************************************************//**
* @brief
*   GPIO Interrupt handler for odd pins
*****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}
