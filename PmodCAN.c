/******************************************************************************/
/*                                                                            */
/* PmodCAN.c -- PmodCAN Example Projects                                      */
/*                                                                            */
/******************************************************************************/
/* Author: Arthur Brown                                                       */
/*                                                                            */
/******************************************************************************/
/* File Description:                                                          */
/*                                                                            */
/* This file contains source code for the PmodCAN driver                      */
/*                                                                            */
/******************************************************************************/
/* Revision History:                                                          */
/*                                                                            */
/*    08/30/2017(ArtVVB):   Created                                           */
/*    09/01/2017(jPeyron):  Formatted Validated                               */
/*    02/24/2018(atangzwj): Validated for Vivado 2017.4                       */
/*                                                                            */
/******************************************************************************/
/* Baud Rates:                                                                */
/*                                                                            */
/*    Microblaze: 9600 or what was specified in UARTlite core                 */
/*    Zynq: 115200                                                            */
/*                                                                            */
/******************************************************************************/

#include "PmodCAN.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

static const char *device1 = "/dev/spidev3.0";
static const char *device2 = "/dev/spidev4.0";

static uint32_t mode = 0;
static uint32_t modeDev1 = 0;
static uint32_t modeDev2 = 0;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay = 1;
int ret = 0;
int fd[2];

uint32_t request;


static void pabort(const char *s)
{
	if (errno != 0)
		perror(s);
	else
		printf("%s\n", s);

	abort();
}

/* ------------------------------------------------------------ */
/*   transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
**
**   Parameters:
**      fd:  SPI device to communicate on
**      tx:  pointer to transfer buffer
**      rx:  pointer to receive buffer
**      len: length of tx and rx buffers in bytes
**
**   Description:
**      SPI transfer
*/
static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	/* struct spi_ioc_transfer {
	__u64		tx_buf;
	__u64		rx_buf;
	__u32		len;
	__u32		speed_hz;
	__u16		delay_usecs;
	__u8		bits_per_word;
	__u8		cs_change;
	__u8		tx_nbits;
	__u8		rx_nbits;
	__u8		word_delay_usecs;
	__u8		pad;
    };
	*/

	struct spi_ioc_transfer tr;

	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)rx;
	tr.len = (unsigned int)len;
	tr.speed_hz = (unsigned int)speed;
	tr.delay_usecs = (unsigned short)delay;
	tr.bits_per_word = (unsigned char)bits;
	tr.cs_change = (unsigned char)false;
	tr.tx_nbits = (unsigned char)8;
	tr.rx_nbits = (unsigned char)8;
	tr.word_delay_usecs = (unsigned char)0;
	tr.pad = (unsigned char)0;

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if(errno != 0)
		printf("errno:0x%X\n", errno);

	if (ret < 1)
	{
		pabort("can't send spi message!");
	}
}



/************************** CAN Function Definitions ***************************/

/* ------------------------------------------------------------ */
/*   CAN_begin()
**
**   Description:
**      Initialize SPI for PMOD CAN
*/
void CAN_begin() {
   CAN_SPIInit();
}

/* ------------------------------------------------------------ */
/*   CAN_end()
**
**   Description:
**      Function placeholder for termination
*/
void CAN_end() {
}

/* ------------------------------------------------------------ */
/*   CAN_SPIInit()
**
**   Description:
**      Initializes the SPI devices for PMOD CAN
*/
int CAN_SPIInit() {
   int status = XST_SUCCESS;

   /* open spi devices */
   fd[0] = open(device1, O_RDWR);
   if (fd[0] < 0)
	   pabort("can't open device1\n");
/*
   fd[1] = open(device2, O_RDWR);
   if (fd[1] < 0)
	   pabort("can't open device2\n");
*/

   	/* set spi mode for devices */

   	/* WR is make a request to assign 'mode' */
   	request = mode;
   	ret = ioctl(fd[0], SPI_IOC_WR_MODE32, &mode);
   	if (ret == -1)
   		pabort("can't set spi1 mode wr");
/*
   	ret = ioctl(fd[1], SPI_IOC_WR_MODE32, &mode);
   	if (ret == -1)
   		pabort("can't set spi2 mode wr");
*/
   	/* RD is read what mode the device actually is in */
   	ret = ioctl(fd[0], SPI_IOC_RD_MODE32, &modeDev1);
   	if (ret == -1)
   		pabort("can't get spi1 mode rd");
/*
   	ret = ioctl(fd[1], SPI_IOC_RD_MODE32, &modeDev2);
   	if (ret == -1)
   		pabort("can't get spi2 mode rd");
*/
   	/* drivers can reject some mode bits without returning an error.
   	 * Read the current value to identify what mode it is in, and if it
   	 * differs from the requested mode, warn the user.
   	 */
   	if (request != modeDev1)
   		printf("WARNING dev 1 does not support requested mode 0x%x\n",request);
/*
   	if (request != modeDev2)
   		printf("WARNING dev 2 does not support requested mode 0x%x\n",request);
*/

   	/* set 8 bits per word */

   	ret = ioctl(fd[0], SPI_IOC_WR_BITS_PER_WORD, &bits);
   	if (ret == -1)
   		pabort("can't set bits per word 1");
/*
   	ret = ioctl(fd[1], SPI_IOC_WR_BITS_PER_WORD, &bits);
   	if (ret == -1)
   		pabort("can't set bits per word 2");
*/

   	ret = ioctl(fd[0], SPI_IOC_RD_BITS_PER_WORD, &bits);
   	if (ret == -1)
   		pabort("can't get bits per word 1");
/*
   	ret = ioctl(fd[1], SPI_IOC_RD_BITS_PER_WORD, &bits);
   	if (ret == -1)
   		pabort("can't get bits per word 2");
*/

   	/* set max speed hz*/

   	ret = ioctl(fd[0], SPI_IOC_WR_MAX_SPEED_HZ, &speed);
   	if (ret == -1)
   		pabort("can't set max speed hz 1");
/*
   	ret = ioctl(fd[1], SPI_IOC_WR_MAX_SPEED_HZ, &speed);
   	if (ret == -1)
   		pabort("can't set max speed hz 2");
*/
   	ret = ioctl(fd[0], SPI_IOC_RD_MAX_SPEED_HZ, &speed);
   	if (ret == -1)
     	pabort("can't get max speed hz 1");
/*
   	ret = ioctl(fd[1], SPI_IOC_RD_MAX_SPEED_HZ, &speed);
   	if (ret == -1)
     	pabort("can't get max speed hz 2");
*/
   return status;
}

/* ------------------------------------------------------------ */
/*   CAN_Readbyte(uint32_t bus)
**
**   Parameters:
**      bus:  CAN bus to read from
**
**   Return Value:
**      byte: Byte read from CAN
**
**   Description:
**      Reads byte from CAN bus through SPI
*/
uint8_t CAN_ReadByte(uint32_t bus) {
   uint8_t byte;
   transfer(fd[bus], &byte, &byte, 1);
   return byte;
}

/* ------------------------------------------------------------ */
/*   CAN_WriteByte(uint32_t bus, uint8_t cmd)
**
**   Parameters:
**      bus: PmodCAN device to send to
**      cmd: Command to send
**
**   Return Value:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Writes a single byte over SPI
*/
void CAN_WriteByte(uint32_t bus, uint8_t cmd) {
  transfer(fd[bus],&cmd, NULL, 1);
}



/* ------------------------------------------------------------ */
/*   CAN_WriteSpi(uint8_t reg, uint8_t *wData, int nData)
**
**   Parameters:
**      bus:   Bus ID
**      reg:   Starting register to write to
**      wData: Data to write
**      nData: Number of data bytes to write
**
**   Return Value:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Writes the byte array to the chip via SPI. It will write the first byte
**      into the specified register, then the next into the following register
**      until all of the data has been sent.
*/
void CAN_WriteSpi(uint32_t bus, uint8_t reg, uint8_t *wData, int nData) {
   // As requested by documentation, first byte contains:
   //    bit 7 = 0 because is a write operation
   //    bit 6 = 1 if more than one bytes is written,
   //            0 if a single byte is written
   //    bits 5-0 - the address
   uint8_t bytearray[nData + 1];
   bytearray[0] = ((nData > 1) ? 0x40 : 0) | (reg & 0x3F);
   memcpy(&bytearray[1], wData, nData); // Copy write commands to byte array
   transfer(fd[bus],bytearray, 0, nData + 1);
}

/* ------------------------------------------------------------ */
/*   CAN_ReadSpi(uint32_t bus, uint8_t reg, uint8_t *rData, int nData)
**
**   Parameters:
**      bus:     Bus ID
**      reg:     Starting register to read from
**      rData:   Byte array to read into
**      nData:   Number of data bytes to read
**
**   Return Value:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Reads data in through SPI. It will read the first byte from the starting
**      register, then the next from the following register. Data is stored into
**      rData.
*/
void CAN_ReadSpi(uint32_t bus, uint8_t reg, uint8_t *rData, int nData) {
   // As requested by documentation, first byte contains:
   //    bit 7 = 1 because is a read operation
   //    bit 6 = 1 if more than one bytes is written,
   //            0 if a single byte is written
   //    bits 5-0 - the address
   uint8_t bytearray[nData + 1];

   bytearray[0] = ((nData > 1) ? 0xC0 : 0x80) | (reg & 0x3F);
   transfer(fd[bus],bytearray, bytearray, nData + 1);
   memcpy(rData, &bytearray[1], nData);
}

/* ------------------------------------------------------------ */
/*   CAN_SetRegisterBits(uint32_t bus,uint8_t reg, uint8_t mask, bool fValue)
**
**   Parameters:
**      bus:      Bus ID
**      reg:      Address of the register whose bits are set
**      mask:     Mask indicating which bits are affected
**      fValue:   1 if the bits are set or 0 if their bits are reset
**
**   Return Values:
**      none
**
**   Errors:
**      none
**
**   Description:
**      This function sets the value of some bits (corresponding to the bMask)
**      of a register (indicated by bRegisterAddress) to 1 or 0 (indicated by
**      fValue).
*/
void CAN_SetRegisterBits(uint32_t bus,uint8_t reg, uint8_t mask, bool fValue) {
   uint8_t regval;
   CAN_ReadSpi(bus,reg, &regval, 1);
   if (fValue)
      regval |= mask;
   else
      regval &= ~mask;
   CAN_WriteSpi(bus, reg, &regval, 1);
}

/* ------------------------------------------------------------ */
/*   CAN_GetRegisterBits(uint32_t bus, uint8_t bRegisterAddress, uint8_t bMask)
**
**   Parameters:
**      bus:              Bus ID
**      bRegisterAddress: The address of the register whose bits are read
**      bMask:            The mask indicating which bits are read
**
**   Return Values:
**      uint8_t - a byte containing only the bits corresponding to the mask.
**
**   Errors:
**      none
**
**   Description:
**      Returns a byte containing only the bits from a register (indicated by
**      bRegisterAddress), corresponding to the bMask mask.
*/
uint8_t CAN_GetRegisterBits(uint32_t bus, uint8_t bRegisterAddress, uint8_t bMask) {
   uint8_t bRegValue;
   CAN_ReadSpi(bus, bRegisterAddress, &bRegValue, 1);
   return bRegValue & bMask;
}

/* ------------------------------------------------------------ */
/*   CAN_ModifyReg(uint32_t bus, uint8_t reg, uint8_t mask, uint8_t value)
**
**   Parameters:
**      bus:    Bus ID
**      reg:    Address of the register to modify
**      mask:   CAN mode mask
**      value:  Value to modify register with
**
**   Return Values:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Modifies the command register using a SPI transfer
*/
void CAN_ModifyReg(uint32_t bus, uint8_t reg, uint8_t mask, uint8_t value) {
   uint8_t buf[4] = {CAN_MODIFY_REG_CMD, reg, mask, value};
   transfer(fd[bus],buf, NULL, 4);
}

/* ------------------------------------------------------------ */
/*   CAN_WriteReg(uint32_t bus, uint8_t reg, uint8_t *data, uint32_t nData)
**
**   Parameters:
**      bus:    Bus ID
**      reg:    Address of the register to write to
**      data:   Data array
**      nData:  Size of data in bytes
**
**   Return Values:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Modifies the write register using a SPI transfer
*/
void CAN_WriteReg(uint32_t bus, uint8_t reg, uint8_t *data, uint32_t nData) {
   uint8_t buf[nData + 2];
   uint32_t i;
   buf[0] = CAN_WRITE_REG_CMD;
   buf[1] = reg;
   for (i = 0; i < nData; i++)
      buf[i + 2] = data[i];
   transfer(fd[bus], buf, NULL, nData + 2);
}

/* ------------------------------------------------------------ */
/*   CAN_ClearReg(uint32_t bus, uint8_t reg, uint32_t nData)
**
**   Parameters:
**      bus:     Bus ID
**      reg:     Address of the register whose bits are cleared
**      nData:   Number of bytes to clear
**
**   Return Values:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Clears register using a SPI transfer
*/
void CAN_ClearReg(uint32_t bus, uint8_t reg, uint32_t nData) {
   uint8_t buf[nData + 2];
   buf[0] = CAN_WRITE_REG_CMD;
   buf[1] = reg;
   transfer(fd[bus],buf, NULL, nData + 2);
}

/* ------------------------------------------------------------ */
/*   CAN_LoadTxBuffer(uint32_t bus, uint8_t start_addr, uint8_t *data, uint32_t nData)
**
**   Parameters:
**      bus:         Bus ID
**      start_addr:  CAN load address?
**      data:        Data to load
**      nData:       Size of data array to load
**
**   Return Values:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Loads the TX buffer using a SPI transfer
*/
void CAN_LoadTxBuffer(uint32_t bus, uint8_t start_addr, uint8_t *data, uint32_t nData) {
   uint8_t buf[nData + 1];
   uint32_t i;
   buf[0] = CAN_LOADBUF_CMD | start_addr;
   for (i = 0; i < nData; i++)
      buf[i + 1] = data[i];
   transfer(fd[bus],buf, NULL, nData + 1);
}

/* ------------------------------------------------------------ */
/*   CAN_RequestToSend(uint32_t bus, uint8_t mask)
**
**   Parameters:
**      bus:  Bus ID
**      mask: Request to send TXn mask
**
**   Return Values:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Request to send using SPI transfer
*/
void CAN_RequestToSend(uint32_t bus, uint8_t mask) {
   uint8_t buf[1] = {CAN_RTS_CMD | mask};
   transfer(fd[bus],buf, NULL, 1);
}

/* ------------------------------------------------------------ */
/*   CAN_ReadRxBuffer(uint32_t bus, uint8_t start_addr, uint8_t *data, uint32_t nData)
**
**   Parameters:
**      bus:        Bus ID
**      start_addr: Start read address
**      data:       Data read array
**      nData:      Size of data array in bytes
**
**   Return Values:
**      void
**
**   Errors:
**      void
**
**   Description:
**      Reads the receive buffer using SPI transfer
*/
void CAN_ReadRxBuffer(uint32_t bus, uint8_t start_addr, uint8_t *data, uint32_t nData) {
   uint8_t buf[nData + 1];
   uint32_t i;
   buf[0] = CAN_READBUF_CMD | start_addr;
   transfer(fd[bus],buf, buf, nData + 1);
   for (i = 0; i < nData; i++)
      data[i] = buf[i + 1];
}

/* ------------------------------------------------------------ */
/*   CAN_ReadReg(uint32_t bus,uint8_t reg, uint8_t *data, uint32_t nData)
**
**   Parameters:
**      bus:        Bus ID
**      reg:        Reg to read
**      data:       Data read array
**      nData:      Size of data array in bytes
**
**   Return Values:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Reads register using SPI transfer
*/
void CAN_ReadReg(uint32_t bus,uint8_t reg, uint8_t *data, uint32_t nData) {
   uint8_t buf[nData + 2];
   uint32_t i;
   buf[0] = CAN_READ_REG_CMD;
   buf[1] = reg;

   transfer(fd[bus],buf, buf, nData + 2);
   for (i = 0; i < nData; i++)
      data[i] = buf[i + 2];
}

/* ------------------------------------------------------------ */
/*   CAN_ReadStatus(uint32_t bus)
**
**   Parameters:
**      bus: Bus ID
**
**   Return Values:
**      uint8_t buf[1]: Status of bus
**
**   Errors:
**      none
**
**   Description:
**      Reads the status register using SPI transfer
*/
uint8_t CAN_ReadStatus(uint32_t bus) {
   uint8_t buf[2] = {CAN_READSTATUS_CMD, 0x00};
   transfer(fd[bus], buf, buf, 2);
   return buf[1];
}

/* ------------------------------------------------------------ */
/*   CAN_RxStatus(uint32_t bus)
**
**   Parameters:
**      bus: Bus ID
**
**   Return Values:
**      uint8_t buf[1]: Bus status
**
**   Errors:
**      none
**
**   Description:
**      Reads RXStatus register using SPI transfer
*/
uint8_t CAN_RxStatus(uint32_t bus) {
   uint8_t buf[2] = {CAN_READSTATUS_CMD, 0x00};
   transfer(fd[bus],buf, buf, 2);
   return buf[1];
}

/* ------------------------------------------------------------ */
/*   CAN_Configure(uint32_t bus, uint8_t mode)
**
**   Parameters:
**      bus:  Bus ID
**      mode: Mode to configure bus
**
**   Return Values:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Configures the CAN bus controller  using a SPI transfer
*/
void CAN_Configure(uint32_t bus, uint8_t mode) {
//   uint8_t CNF[3] = {0x86, 0xFB, 0x41}; // Registers to set CAN speed (kBPS) and clock (Hz)
	uint8_t CNF[3] = {0x04, 0x89, 0x80};

   // Set CAN control mode to configuration
   CAN_ModifyReg(bus,CAN_CANCTRL_REG_ADDR, CAN_CAN_CANCTRL_MODE_MASK, CAN_ModeConfiguration);

   // Set config rate and clock for
   CAN_WriteReg(bus,CAN_CNF3_REG_ADDR, CNF, 3);

   // Initiate CAN buffer filters and registers
   CAN_ClearReg(bus,0x00, 12);
   CAN_ClearReg(bus,0x10, 12);
   CAN_ClearReg(bus,0x20, 8);
   CAN_ClearReg(bus,0x30, 14);
   CAN_ClearReg(bus,0x40, 14);
   CAN_ClearReg(bus,0x50, 14);

   // Set the CAN mode for any message type
   CAN_ModifyReg(bus,CAN_RXB0CTRL_REG_ADDR, 0x64, 0x60);

   // Set CAN control mode to selected mode (exit configuration)
   CAN_ModifyReg(bus,CAN_CANCTRL_REG_ADDR, CAN_CAN_CANCTRL_MODE_MASK, mode << CAN_CANCTRL_MODE_BIT);
}

/* ------------------------------------------------------------ */
/*   CAN_SendMessage(uint32_t bus,CAN_Message message, CAN_TxBuffer target)
**
**   Parameters:
**      bus:     Bus ID
**      message: message to send
**      target:  ???
**
**   Return Values:
**      XStatus: XST_SUCCESS
**
**   Errors:
**      !XST_SUCCESS on failure
**
**   Description:
**      Sends message to CAN
*/
XStatus CAN_SendMessage(uint32_t bus,CAN_Message message, CAN_TxBuffer target) {
   uint8_t data[13];
   uint8_t i;

   uint8_t rts_mask;
   uint8_t load_start_addr;

   switch (target) {
   case CAN_Tx0:
      rts_mask = CAN_RTS_TXB0_MASK;
      load_start_addr = CAN_LOADBUF_TXB0SIDH;
      break;
   case CAN_Tx1:
      rts_mask = CAN_RTS_TXB1_MASK;
      load_start_addr = CAN_LOADBUF_TXB1SIDH;
      break;
   case CAN_Tx2:
      rts_mask = CAN_RTS_TXB2_MASK;
      load_start_addr = CAN_LOADBUF_TXB2SIDH;
      break;
   default:
      return XST_FAILURE;
   }

   data[0] = (message.id >> 3) & 0xFF; // TXB0 SIDH

   data[1] = (message.id << 5) & 0xE0; // TXB0 SIDL
   data[1] |= (message.ide << 3) & 0x08;
   data[1] |= (message.eid >> 16) & 0x03;

   data[2] = (message.eid >> 8) & 0xFF;
   data[3] = (message.eid) & 0xFF;

   data[4] = (message.rtr << 6) & 0x40;
   data[4] |= (message.dlc) & 0x0F;

   for (i = 0; i < message.dlc; i++)
      data[i + 5] = message.data[i];

   printf("CAN_SendMessage message.dlc: %02x\r\n", message.dlc);
   for (i = 0; i < 5 + message.dlc; i++)
      printf("CAN_SendMessage: %02x\r\n", data[i]);

   CAN_LoadTxBuffer(bus,load_start_addr, data, message.dlc + 5);
   CAN_RequestToSend(bus,rts_mask);

   return XST_SUCCESS;
}

/* ------------------------------------------------------------ */
/*   CAN_ReceiveMessage(uint32_t bus,CAN_Message *MessagePtr, CAN_RxBuffer target)
**
**   Parameters:
**      bus:        Bus ID
**      MessagePtr: Pointer to message received
**      target:     ???
**
**   Return Values:
**      XStatus XST_SUCCESS
**
**   Errors:
**      !XST_SUCCESS on failure
**
**   Description:
**      Receives message from CAN
*/
XStatus CAN_ReceiveMessage(uint32_t bus,CAN_Message *MessagePtr, CAN_RxBuffer target) {
   uint8_t data[13];
   uint8_t i;
   uint8_t read_start_addr;

   switch (target) {
   case CAN_Rx0:
      read_start_addr = CAN_READBUF_RXB0SIDH;
      break;
   case CAN_Rx1:
      read_start_addr = CAN_READBUF_RXB1SIDH;
      break;
   default:
      return XST_FAILURE;
   }

   CAN_ReadRxBuffer(bus, read_start_addr, data, 13);

   MessagePtr->id = (uint16_t) data[0] << 3;
   MessagePtr->id |= (data[1] & 0xE0) >> 5;

   MessagePtr->ide = (data[1] & 0x08) >> 3;

   MessagePtr->srr = (data[1] & 0x10) >> 4;

   MessagePtr->eid = (uint32_t) (data[1] & 0x03) << 16;
   MessagePtr->eid |= (uint32_t) (data[2] & 0xFF) << 8;
   MessagePtr->eid |= (uint32_t) (data[3] & 0xFF);

   MessagePtr->rtr = (data[4] & 0x40) >> 6;

   MessagePtr->dlc = data[4] & 0x0F;

   // Read only relevant data bytes
   CAN_ReadRxBuffer(bus,read_start_addr, data, MessagePtr->dlc);

   for (i = 0; i < MessagePtr->dlc; i++)
      MessagePtr->data[i] = data[i + 5];

   return XST_SUCCESS;
}
