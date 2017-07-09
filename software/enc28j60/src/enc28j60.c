/*
 * enc28j60.c
 *
 *  Created on: 12. apr. 2015
 *      Author: kjostera
 */
#include "enc28j60.h"
#include "enc28j60_bsp.h"

#include <stdint.h>
#include <stdbool.h>

#if defined(DEBUG_PRINTOUT)
#define DEBUG_Print(...)    printf(__VA_ARGS__)
#else
#define DEBUG_Print(...)
#endif

/* Opcodes */
#define RCR 0x0 /* Read Control Register */
#define RBM 0x1 /* Read Buffer Memory */
#define WCR 0x2 /* Write Control Register */
#define WBM 0x3 /* Write Buffer Memory */
#define BFS 0x4 /* Bit Field Set */
#define BFC 0x5 /* Bit Field Clear */
#define SRC 0x7 /* System Reset Command (Soft Reset) */

typedef uint32_t reg_t;

/* Registers */
#define BANK_MASK 0xff00
#define BANK_0   (0x0 << 8)
#define BANK_1   (0x1 << 8)
#define BANK_2   (0x2 << 8)
#define BANK_3   (0x3 << 8)
#define BANK_ANY (0x4 << 8)

/* Register Type */
#define TYPE_MASK 0xff0000
#define REG_ETH (0x00 << 16)
#define REG_MII (0x01 << 16)
#define REG_MAC (0x02 << 16)
#define REG_PHY (0x03 << 16)

#define REG_ADDR(reg) (reg & 0xff)
#define REG_BANK(reg) (reg & BANK_MASK)
#define REG_TYPE(reg) (reg & TYPE_MASK)

/* Bank 0 */
#define ERDPT    (0x00 | BANK_0 | REG_ETH) /* Used when reading 16 bit word */
#define ERDPTL   (0x00 | BANK_0 | REG_ETH)
#define ERDPTH   (0x01 | BANK_0 | REG_ETH)
#define EWRPTL   (0x02 | BANK_0 | REG_ETH)
#define EWRPTH   (0x03 | BANK_0 | REG_ETH)
#define ETXSTL   (0x04 | BANK_0 | REG_ETH)
#define ETXSTH   (0x05 | BANK_0 | REG_ETH)
#define ETXNDL   (0x06 | BANK_0 | REG_ETH)
#define ETXNDH   (0x07 | BANK_0 | REG_ETH)
#define ERXSTL   (0x08 | BANK_0 | REG_MAC)
#define ERXSTH   (0x09 | BANK_0 | REG_MAC)
#define ERXNDL   (0x0a | BANK_0 | REG_ETH)
#define ERXNDH   (0x0b | BANK_0 | REG_ETH)
#define ERXRDPTL (0x0c | BANK_0 | REG_ETH)
#define ERXRDPTH (0x0d | BANK_0 | REG_ETH)
#define ERXWRPTL (0x0e | BANK_0 | REG_ETH)
#define ERXWRPTH (0x0f | BANK_0 | REG_ETH)
#define EDMASTL  (0x10 | BANK_0 | REG_ETH)
#define EDMASTH  (0x11 | BANK_0 | REG_ETH)
#define EDMANDL  (0x12 | BANK_0 | REG_ETH)
#define EDMANDH  (0x13 | BANK_0 | REG_ETH)
#define EDMADSTL (0x14 | BANK_0 | REG_ETH)
#define EDMADSTH (0x15 | BANK_0 | REG_ETH)
#define EDMACSL  (0x16 | BANK_0 | REG_ETH)
#define EDMACSH  (0x17 | BANK_0 | REG_ETH)

/* Bank 1 */
#define EHT0     (0x00 | BANK_1 | REG_ETH)
#define EHT1     (0x01 | BANK_1 | REG_ETH)
#define EHT2     (0x02 | BANK_1 | REG_ETH)
#define EHT3     (0x03 | BANK_1 | REG_ETH)
#define EHT4     (0x04 | BANK_1 | REG_ETH)
#define EHT5     (0x05 | BANK_1 | REG_ETH)
#define EHT6     (0x06 | BANK_1 | REG_ETH)
#define EHT7     (0x07 | BANK_1 | REG_ETH)
#define EPMM0    (0x08 | BANK_1 | REG_ETH)
#define EPMM1    (0x09 | BANK_1 | REG_ETH)
#define EPMM2    (0x0a | BANK_1 | REG_ETH)
#define EPMM3    (0x0b | BANK_1 | REG_ETH)
#define EPMM4    (0x0c | BANK_1 | REG_ETH)
#define EPMM5    (0x0d | BANK_1 | REG_ETH)
#define EPMM6    (0x0e | BANK_1 | REG_ETH)
#define EPMM7    (0x0f | BANK_1 | REG_ETH)
#define EPMCSL   (0x10 | BANK_1 | REG_ETH)
#define EPMCSH   (0x11 | BANK_1 | REG_ETH)
#define EPMOL    (0x14 | BANK_1 | REG_ETH)
#define EPMOH    (0x15 | BANK_1 | REG_ETH)
#define ERXFCON  (0x18 | BANK_1 | REG_ETH)
#define EPKTCNT  (0x19 | BANK_1 | REG_ETH)

/* Bank 2 */
#define MACON1   (0x00 | BANK_2 | REG_MAC)
#define MACON3   (0x02 | BANK_2 | REG_MAC)
#define MACON4   (0x03 | BANK_2 | REG_MAC)
#define MABBIPG  (0x04 | BANK_2 | REG_MAC)
#define MAIPGL   (0x06 | BANK_2 | REG_MAC)
#define MAIPGH   (0x07 | BANK_2 | REG_MAC)
#define MACLCON1 (0x08 | BANK_2 | REG_MAC)
#define MACLCON2 (0x09 | BANK_2 | REG_MAC)
#define MAMXFLL  (0x0a | BANK_2 | REG_MAC)
#define MAMXFLH  (0x0b | BANK_2 | REG_MAC)
#define MICMD    (0x12 | BANK_2 | REG_MII)
#define MIREGADR (0x14 | BANK_2 | REG_MII)
#define MIWRL    (0x16 | BANK_2 | REG_MII)
#define MIWRH    (0x17 | BANK_2 | REG_MII)
#define MIRDL    (0x18 | BANK_2 | REG_MII)
#define MIRDH    (0x19 | BANK_2 | REG_MII)

/* Bank 3 */
#define MAADR5  (0x00 | BANK_3 | REG_MAC)
#define MAADR6  (0x01 | BANK_3 | REG_MAC)
#define MAADR3  (0x02 | BANK_3 | REG_MAC)
#define MAADR4  (0x03 | BANK_3 | REG_MAC)
#define MAADR1  (0x04 | BANK_3 | REG_MAC)
#define MAADR2  (0x05 | BANK_3 | REG_MAC)
#define EBSTSD  (0x06 | BANK_3 | REG_ETH)
#define EBSTCON (0x07 | BANK_3 | REG_ETH)
#define EBSTCSL (0x08 | BANK_3 | REG_ETH)
#define EBSTCSH (0x09 | BANK_3 | REG_ETH)
#define MISTAT  (0x0a | BANK_3 | REG_MII)
#define EREVID  (0x12 | BANK_3 | REG_ETH) /* Ethernet Revision Id */
#define ECOCON  (0x15 | BANK_3 | REG_ETH)
#define EFLOCON (0x17 | BANK_3 | REG_ETH)
#define EPAUSL  (0x18 | BANK_3 | REG_ETH)
#define EPAUSH  (0x19 | BANK_3 | REG_ETH)

/* Non-Banked */
#define EIE     (0x1b | BANK_ANY | REG_ETH) /* Ethernet Interrupt Enable Register */
#define EIR     (0x1c | BANK_ANY | REG_ETH) /* Ethernet Interrupt Request (Flag) Register */
#define ESTAT   (0x1d | BANK_ANY | REG_ETH) /* Ethernet Status Register */
#define ECON2   (0x1e | BANK_ANY | REG_ETH) /* Ethernet Control Register 2 */
#define ECON1   (0x1f | BANK_ANY | REG_ETH) /* Ethernet Control Register 1 */

/* PHY Registers */
#define PHCON1  (0x00 | REG_PHY) /* PHY Control Register 1 */
#define PHSTAT1 (0x01 | REG_PHY) /* Physical Layer Status Register 1 */
#define PHID1   (0x02 | REG_PHY) /* PHY Id Register 1 */
#define PHID2   (0x03 | REG_PHY) /* PHY Id Register 2 */
#define PHCON2  (0x10 | REG_PHY) /* PHY Control Register 2 */
#define PHSTAT2 (0x11 | REG_PHY) /* Physical Layer Status Register 2 */
#define PHIE    (0x12 | REG_PHY) /* PHY Interrupt Enable Register */
#define PHIR    (0x13 | REG_PHY) /* PHY Interrupt Request (Flag) Register */
#define PHLCON  (0x14 | REG_PHY) /* PHY Module Led Control Register */

/* Register Fields of interest */
#define ECON1_RXEN      (1 << 2)  /* RX Enable */
#define ECON1_TXRTS     (1 << 3)  /* Transmit Request to send */
#define ECON2_PKTDEC    (1 << 6)  /* Packet Decrement */
#define EIE_INTIE       (1 << 7)
#define EIE_PKTIE       (1 << 6)
#define MICMD_MIIRD     (1 << 0)
#define MISTAT_BUSY     (1 << 0)
#define PHSTAT1_LLSTAT  (1 << 2)  /* Latching Link Status */
#define PHCON1_PRST     (1 << 15) /* PHY Software Reset */

static int _CurrentBank = BANK_0;

/* We have 8 KiB of buffer space that we divide into
 * rx and tx buffers.
 *
 * [   rx 4 KiB     |     tx  4 KiB  ]
 * 0              0x1000            0x2000
 *
 */
#define RX_BUFFER_START   0x0000
#define RX_BUFFER_SIZE    0x1000
#define TX_BUFFER_START   0x1000
#define TX_BUFFER_SIZE    0x1000

static void BankSelect(reg_t reg);
static void FAIL(void);

static uint8_t ReadRegister(reg_t reg)
{
  BankSelect(reg);
  BSP_ChipOn();
  uint8_t cmd = (RCR << 5) | REG_ADDR(reg);
  BSP_SpiTransfer(cmd);
  uint8_t val = BSP_SpiTransfer(0x00);
  if ((REG_TYPE(reg) == REG_MAC) || (REG_TYPE(reg) == REG_MII))
  {
    val = BSP_SpiTransfer(0x00);
  }
  BSP_ChipOff();
  return val;
}

static uint16_t ReadRegisterWord(reg_t reg)
{
  uint16_t value = ReadRegister(reg);
  value |= ReadRegister(reg + 1) << 8;
  return value;
}

static void WriteRegister(reg_t reg, uint8_t val)
{
  BankSelect(reg);
  BSP_ChipOn();
  uint8_t cmd = (WCR << 5) | REG_ADDR(reg);
  BSP_SpiTransfer(cmd);
  BSP_SpiTransfer(val);
  BSP_ChipOff();
}

static void WriteRegisterWord(reg_t reg, uint16_t val)
{
  WriteRegister(reg, val & 0xff);
  WriteRegister(reg+1, val >> 8);
}

static void ReadBufferMemory(uint8_t * dst, int count)
{
  BSP_ChipOn();
  uint8_t cmd = (RBM << 5) | 0x1a;
  BSP_SpiTransfer(cmd);
  while (count-- > 0)
  {
    *dst++ = BSP_SpiTransfer(0x00);
  }
  BSP_ChipOff();
}

static void WriteBufferMemory(const uint8_t * src, int count)
{
  BSP_ChipOn();
  uint8_t cmd = (WBM << 5) | 0x1a;
  BSP_SpiTransfer(cmd);
  while (count-- > 0)
  {
    BSP_SpiTransfer(*src++);
  }
  BSP_ChipOff();
}

static void BitFieldSet(reg_t reg, uint8_t val)
{
  BankSelect(reg);
  BSP_ChipOn();
  uint8_t cmd = (BFS << 5) | REG_ADDR(reg);
  BSP_SpiTransfer(cmd);
  BSP_SpiTransfer(val);
  BSP_ChipOff();
}

static void BitFieldClear(reg_t reg, uint8_t val)
{
  BankSelect(reg);
  BSP_ChipOn();
  uint8_t cmd = (BFC << 5) | REG_ADDR(reg);
  BSP_SpiTransfer(cmd);
  BSP_SpiTransfer(val);
  BSP_ChipOff();
}

static void ENC28J60_SoftReset(void)
{
  BSP_ChipOn();
  BSP_SpiTransfer(SRC << 5 | 0x1f);
  BSP_ChipOff();
}

static void BankSelect(reg_t reg)
{
  if ((REG_BANK(reg) == BANK_ANY) || (REG_BANK(reg) == _CurrentBank))
  {
    return;
  }
  BitFieldClear(ECON1, 0x03);
  BitFieldSet(ECON1, (REG_BANK(reg) >> 8));
  _CurrentBank = REG_BANK(reg);
}

static uint16_t ReadPhyRegister(reg_t reg)
{
  WriteRegister(MIREGADR, REG_ADDR(reg));
  WriteRegister(MICMD, MICMD_MIIRD);

  while (ReadRegister(MISTAT) & MISTAT_BUSY)
	  ;

  WriteRegister(MICMD, 0x00);
  return ReadRegisterWord(MIRDL);
}

static void ENC28J60_WritePhyRegister(reg_t reg, uint16_t val)
{
  WriteRegister(MIREGADR, reg);
  WriteRegisterWord(MIWRL, val);

  while (ReadRegister(MISTAT) & MISTAT_BUSY)
    ;
}

/**
 * @brief Initialize the SPI connection to the ENC28J60 device
 */
void ENC28J60_Init(const ENC28J60_Config * config)
{
  BSP_SpiInit();

  ENC28J60_SoftReset();

  /* Wait for Oscillator Start-up Timer */
  BSP_Delay(200);
  // TODO: The Errata document says that the Oscillator flag is not reliable
  //       we must wait for 1ms instead

  /* Read Revision ID */
  int revid = ReadRegister(EREVID);
  switch (revid)
  {
  case 0x2: DEBUG_Print("Rev B1\n"); break;
  case 0x4: DEBUG_Print("Rev B4\n"); break;
  case 0x5: DEBUG_Print("Rev B5\n"); break;
  case 0x6: DEBUG_Print("Rev B7\n"); break;
  default:  DEBUG_Print("Rev ?? (%d)\n", revid);
  }

  /* Read PHY Id */
  uint16_t phyid = ReadPhyRegister(PHID1);
  DEBUG_Print("PHID1: 0x%x\n", phyid);
  while (phyid == 0);
  phyid = ReadPhyRegister(PHID2);
  DEBUG_Print("PHID2: 0x%x\n", phyid);
  while (phyid == 0);

  /* Initialize Receive Buffer */
  const uint16_t rx_start = 0x0000;
  const uint16_t rx_end  = RX_BUFFER_SIZE;

  WriteRegisterWord(ERXSTL, rx_start);
  WriteRegisterWord(ERXNDL, rx_end);
  WriteRegisterWord(ERXRDPTL, rx_start);

  DEBUG_Print("ERXST: 0x%04X\n", ReadRegisterWord(ERXSTL));
  DEBUG_Print("ERXNDL: 0x%04X\n", ReadRegisterWord(ERXNDL));

  /* Setup Receive Filters */
  WriteRegister(ERXFCON, 0xa1);

  /* Initialize MAC */
  WriteRegister(MACON1, 0x0d);
  WriteRegister(MACON3, 0x33);
  WriteRegister(MACON4, 0x00);

  uint16_t maxFrameLength = 1518;
  WriteRegisterWord(MAMXFLL, maxFrameLength);

  WriteRegister(MABBIPG, 0x15); /* Full-Duplex */
  WriteRegister(MAIPGL, 0x12);
//  ENC28J60_WriteControlRegister(MAIPGH, 0x0c);

  /* Program Mac Address */
  const uint8_t * mac = config->macAddress;
  WriteRegister(MAADR1, mac[0]);
  WriteRegister(MAADR2, mac[1]);
  WriteRegister(MAADR3, mac[2]);
  WriteRegister(MAADR4, mac[3]);
  WriteRegister(MAADR5, mac[4]);
  WriteRegister(MAADR6, mac[5]);
  DEBUG_Print("Mac: %02X:%02X:%02X:%02X:%02X:%02X\n",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Initialize PHY */
  ENC28J60_WritePhyRegister(PHCON1, PHCON1_PRST); // Reset PHY
  while ((ReadPhyRegister(PHCON1) & PHCON1_PRST))
    ;

  ENC28J60_WritePhyRegister(PHCON1, 0x0100); // Full-duplex

  /* Setup Interrupts */
  WriteRegister(EIE, EIE_INTIE | EIE_PKTIE);

  /* Enable RX */
  BitFieldSet(ECON1, ECON1_RXEN);

  uint16_t phstat1;
  uint16_t phstat2;
  do {
    phstat1 = ReadPhyRegister(PHSTAT1);
    phstat2 = ReadPhyRegister(PHSTAT2);
  } while (!(phstat1 & PHSTAT1_LLSTAT));

  DEBUG_Print("link is up\n");
}

void ENC28J60_WriteFrameData(uint32_t offset, const uint8_t * buffer, uint32_t len)
{
  uint16_t start = TX_BUFFER_START;
  uint8_t control = 0x00;

  if (offset == 0)
  {
    /* Start frame with control byte */
    WriteRegisterWord(EWRPTL, start);
    WriteBufferMemory(&control, sizeof(control));
  }
  else
  {
    WriteRegisterWord(EWRPTL, start + offset + 1);
  }

  WriteBufferMemory(buffer, len);
}

void ENC28J60_Transmit()
{
  uint16_t start = TX_BUFFER_START;
  uint16_t end;

  end = ReadRegisterWord(EWRPTL);

  WriteRegisterWord(ETXSTL, start);
  WriteRegisterWord(ETXNDL, end);

  DEBUG_Print("Transmit %d bytes ETXST=0x%04X ETXNDL=0x%04X\n", len, start, end);
  /* Start transmission and wait for it to complete */
  BitFieldSet(ECON1, ECON1_TXRTS);
  while (ReadRegister(ECON1) & ECON1_TXRTS)
    ;
  DEBUG_Print("Transmit complete\n");
}

uint32_t ENC28J60_Receive(uint8_t * buffer, uint32_t len)
{
  uint8_t packetCount = ReadRegister(EPKTCNT);
  if (packetCount == 0)
    return 0;

  DEBUG_Print("EPKTCNT: %d\n", packetCount);
  uint16_t rxReadPointer;
  uint16_t rxWritePointer;
  uint16_t bufferReadPointer;
  uint16_t nextPacket;
  uint16_t rxLength;
  uint32_t rxStatus;

  rxReadPointer = ReadRegisterWord(ERXRDPTL);
  rxWritePointer = ReadRegisterWord(ERXWRPTL);
  bufferReadPointer = ReadRegisterWord(ERDPTL);

  DEBUG_Print("ERXRDPT: 0x%04X\n", rxReadPointer);
  DEBUG_Print("ERXWRPT: 0x%04X\n", rxWritePointer);
  DEBUG_Print("ERDPT: 0x%04X\n", bufferReadPointer);
  WriteRegisterWord(ERDPTL, rxReadPointer);

  ReadBufferMemory((uint8_t *)&nextPacket, sizeof(nextPacket));
  ReadBufferMemory((uint8_t *)&rxStatus, sizeof(rxStatus));

  bool receiveOk = rxStatus & 0x00800000;
  rxLength = rxStatus & 0xffff;
  rxLength -= 4; /* Remove CRC */
  DEBUG_Print("Packet length %d next packet @ 0x%04X\n", rxLength, nextPacket);

  if (receiveOk)
  {
    ReadBufferMemory(buffer, rxLength);
  }
  else
  {
    rxLength = 0;
  }

  if (nextPacket > (RX_BUFFER_START + RX_BUFFER_SIZE))
  {
    FAIL();
  }

  if (rxLength > len)
  {
    DEBUG_Print("Receive buffer too small. Buffer size %d, packet length %d\n", len, rxLength);
    FAIL();
  }


  /* Advance the rx read pointer, this will free up rx space */
  WriteRegisterWord(ERXRDPTL, nextPacket);
  BitFieldSet(ECON2, ECON2_PKTDEC);
  return rxLength;
}

void ENC28J60_Interrupt(void)
{
  /* Disable INTIE to trigger falling edge if another interrupt appears */
  BitFieldClear(EIE, EIE_INTIE);
  uint8_t estat = ReadRegister(ESTAT);
  uint8_t eir = ReadRegister(EIR);
  BitFieldSet(EIE, EIE_INTIE);
}

void FAIL(void)
{
  uint16_t ledConfig = 0x3000 | (0b1011 << 8) | (0b1010 << 4);
  ENC28J60_WritePhyRegister(PHLCON, ledConfig);
  DEBUG_Print("FAIL!\n");
  while (1)
    ;
}
