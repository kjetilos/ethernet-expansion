#include <stdint.h>

/**
 * @brief Initialize the spi port on this device
 */
void BSP_SpiInit(void);

/**
 * @brief Transfer a single byte using the spi device
 */
uint8_t BSP_SpiTransfer(uint8_t data);

/**
 * @brief Activate the chip by pulling the chip select line low
 */
void BSP_ChipOn(void);

/**
 * @brief De-Activate the chip by pulling the chip select line high
 */
void BSP_ChipOff(void);

/**
 * @brief Wait for ms milliseconds
 */
void BSP_Delay(uint32_t ms);
