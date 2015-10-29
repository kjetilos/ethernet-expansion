#ifndef ENC28J60_H_
#define ENC28J60_H_

#include <stdint.h>

typedef struct
{
  uint8_t * macAddress;
} ENC28J60_Config;

void ENC28J60_Init(const ENC28J60_Config * config);
void ENC28J60_Transmit(const uint8_t * buffer, uint32_t len);
uint32_t ENC28J60_Receive(uint8_t * buffer, uint32_t len);
void ENC28J60_Interrupt(void);

#endif /* ENC28J60_H_ */
