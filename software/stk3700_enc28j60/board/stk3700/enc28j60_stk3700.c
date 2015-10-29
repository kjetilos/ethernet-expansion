/*
 * This is the board specific code for running the project on a Giant Gecko
 * Starter Kit - STK3700
 */

#include "enc28j60_bsp.h"

#include "em_cmu.h"
#include "em_device.h"
#include "em_usart.h"
#include "em_gpio.h"

#include <stdint.h>

/* Port and pin locations on the stk3700 expansion port */
#define SPI_USART            USART1
#define SPI_USART_LOCATION   _USART_ROUTE_LOCATION_LOC1
#define SPI_USART_CLOCK      cmuClock_USART1
#define SPI_MOSI_PORT        gpioPortD
#define SPI_MOSI_PIN         0
#define SPI_MISO_PORT        gpioPortD
#define SPI_MISO_PIN         1
#define SPI_CLK_PORT         gpioPortD
#define SPI_CLK_PIN          2
#define SPI_CS_PORT          gpioPortD
#define SPI_CS_PIN           3
#define INT_PORT             gpioPortC
#define INT_PIN              5
#define RESET_PORT           gpioPortB
#define RESET_PIN            12

void BSP_SpiInit(void)
{
  /*
   * Chip select active low
   * Clock is idle low so ClockPolatiry=0
   * ClockPhase=0
   */
  USART_InitSync_TypeDef initSpi = USART_INITSYNC_DEFAULT;
  initSpi.baudrate = 20000000; /* 20 Mbits/s */
  initSpi.databits = usartDatabits8;
  initSpi.master = true;
  initSpi.msbf = true; /* Send most significant bit first */
  initSpi.clockMode = usartClockMode0;

  /* Enabling clock to USART */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(SPI_USART_CLOCK, true);
  CMU_ClockEnable(cmuClock_GPIO, true);


  /* Configure SPI */
  USART_Reset(SPI_USART);
  /* Initialize in SPI master mode. */
  USART_InitSync(SPI_USART, &initSpi);

  /* Enabling pins and setting location #3, SPI CS not enable */
  SPI_USART->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN
             | (SPI_USART_LOCATION << _USART_ROUTE_LOCATION_SHIFT);
  /* Enabling TX and RX */
  SPI_USART->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
  /* Clear previous interrupts */
  SPI_USART->IFC = _USART_IFC_MASK;

  /* IO configuration */
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
  return USART_SpiTransfer(SPI_USART, data);
}

void BSP_ChipOn(void)
{
  GPIO_PinOutClear(SPI_CS_PORT, SPI_CS_PIN);
}

void BSP_ChipOff(void)
{
  GPIO_PinOutSet(SPI_CS_PORT, SPI_CS_PIN);
}
