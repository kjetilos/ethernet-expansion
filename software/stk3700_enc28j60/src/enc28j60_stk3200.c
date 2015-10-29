/*
 * stk3700_bsp.c
 *
 *  Created on: 23. okt. 2015
 *      Author: kjostera
 */

#include "enc28j60_bsp.h"

#include "em_cmu.h"
#include "em_device.h"
#include "em_usart.h"
#include "em_gpio.h"

#include <stdint.h>

/* Port and pin locations on the stk3200 expansion port */
#define SPI_MOSI_PORT  gpioPortD
#define SPI_MOSI_PIN   7
#define SPI_MISO_PORT  gpioPortD
#define SPI_MISO_PIN   6
#define SPI_CLK_PORT   gpioPortC
#define SPI_CLK_PIN    15
#define SPI_CS_PORT    gpioPortC
#define SPI_CS_PIN     14
#define INT_PORT       gpioPortA
#define INT_PIN        0
#define RESET_PORT     gpioPortD
#define RESET_PIN      4

void BSP_SpiInit(void)
{
  /*
   * Chip select active low
   * Clock is idle low so ClockPolatiry=0
   * ClockPhase=0
   */
  USART_TypeDef *spi;
  USART_InitSync_TypeDef initSpi = USART_INITSYNC_DEFAULT;
  initSpi.baudrate = 20000000; /* 20 Mbits/s */
  initSpi.databits = usartDatabits8;
  initSpi.master = true;
  initSpi.msbf = true; /* Send most significant bit first */
  initSpi.clockMode = usartClockMode0;

//  DEBUG_Print("ENC28J60 Init\r\n");

  /* Enabling clock to USART */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_USART1, true);
  CMU_ClockEnable(cmuClock_GPIO, true);


  /* Setup SPI at USART1 */
  spi = USART1;
  /* Configure SPI */
  USART_Reset(spi);
  /* Initialize USART1, in SPI master mode. */
  USART_InitSync(spi, &initSpi);

  /* Enabling pins and setting location #3, SPI CS not enable */
  spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN | USART_ROUTE_LOCATION_LOC3;
  /* Enabling TX and RX */
  spi->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
  /* Clear previous interrupts */
  spi->IFC = _USART_IFC_MASK;

  /* IO configuration (USART 1, Location #3) */
  GPIO_PinModeSet(SPI_MOSI_PORT, SPI_MOSI_PIN, gpioModePushPull, 1);
  GPIO_PinModeSet(SPI_MISO_PORT, SPI_MISO_PIN, gpioModeInput, 0);
  GPIO_PinModeSet(SPI_CLK_PORT, SPI_CLK_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(SPI_CS_PORT, SPI_CS_PIN, gpioModePushPull, 1);

  /* Interrupt configuration */
  GPIO_PinModeSet(INT_PORT, INT_PIN, gpioModeInput, 0);

  /* Pull the reset pin high */
  GPIO_PinModeSet(RESET_PORT, RESET_PIN, gpioModePushPull, 1);
}

uint8_t BSP_SpiTransfer(uint8_t data)
{
  USART_SpiTransfer(USART1, data);
}

void BSP_ChipOn(void)
{
  GPIO_PinOutClear(SPI_CS_PORT, SPI_CS_PIN);
}

void BSP_ChipOff(void)
{
  GPIO_PinOutSet(SPI_CS_PORT, SPI_CS_PIN);
}
