/*
 * This is the board specific code for running the project on a Giant Gecko
 * Starter Kit - STK3700
 */

#include "segmentlcd.h"

void BSP_DisplayInit(void)
{
  SegmentLCD_Init(false);
}

void BSP_DisplayWrite(char * s)
{
  SegmentLCD_Write(s);
}
