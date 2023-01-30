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

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
   int ret;
   struct spi_ioc_transfer tr = {
       .tx_buf = (unsigned long)tx,
       .rx_buf = (unsigned long)rx,
       .len = len,
       .delay_usecs = delay,
       .speed_hz = speed,
       .bits_per_word = bits,
   };

   ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   if (ret < 1)
      pabort("can't send spi message");
}

/************************** Function Definitions ***************************/

/* ------------------------------------------------------------ */
/*** void CAN_begin(PmodCAN *InstancePtr, u32 GPIO_Address, u32 SPI_Address)
**
**   Parameters:
**      InstancePtr:  A PmodCAN object to start
**      GPIO_Address: The Base address of the PmodCAN GPIO
**      SPI_Address:  The Base address of the PmodCAN SPI
**
**   Return Value:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Initialize the PmodCAN.
*/
void CAN_begin()
{
   CAN_SPIInit();
}

/* ------------------------------------------------------------ */
/*** CAN_end(PmodCAN *InstancePtr)
**
**   Parameters:
**      InstancePtr: PmodCAN object to stop
**
**   Return Value:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Stops the device
*/
void CAN_end()
{
}

/* ------------------------------------------------------------ */
/*** CAN_SPIInit
**
**   Parameters:
**      none
**
**   Return Value:
**      none
**
**   Errors:
**      none
**
**   Description:
**      Initializes the PmodCAN SPI.
*/
int CAN_SPIInit()
{
   int Status;

   fd[0] = open(device1, O_RDWR);
   if (fd[0] < 0)
      pabort("can't open device1");

   fd[1] = open(device2, O_RDWR);
   if (fd[1] < 0)
      pabort("can't open device1");

   /*
    * spi mode
    */
   /* WR is make a request to assign 'mode' */
   request = mode;
   ret = ioctl(fd[0], SPI_IOC_WR_MODE32, &mode);
   if (ret == -1)
      pabort("can't set spi mode 1");

   ret = ioctl(fd[1], SPI_IOC_WR_MODE32, &mode);
   if (ret == -1)
      pabort("can't set spi mode 2");

   /* RD is read what mode the device actually is in */
   ret = ioctl(fd[0], SPI_IOC_RD_MODE32, &mode);
   if (ret == -1)
      pabort("can't get spi mode 1");

   ret = ioctl(fd[1], SPI_IOC_RD_MODE32, &mode);
   if (ret == -1)
      pabort("can't get spi mode 2");

   /* Drivers can reject some mode bits without returning an error.
    * Read the current value to identify what mode it is in, and if it
    * differs from the requested mode, warn the user.
    */
   if (request != mode)
      printf("WARNING device does not support requested mode 0x%x\n",
             request);

   /*
    * bits per word
    */
   ret = ioctl(fd[0], SPI_IOC_WR_BITS_PER_WORD, &bits);
   if (ret == -1)
      pabort("can't set bits per word 1");

   ret = ioctl(fd[1], SPI_IOC_WR_BITS_PER_WORD, &bits);
   if (ret == -1)
      pabort("can't set bits per word 2");

   ret = ioctl(fd[0], SPI_IOC_RD_BITS_PER_WORD, &bits);
   if (ret == -1)
      pabort("can't get bits per word 1");

   ret = ioctl(fd[1], SPI_IOC_RD_BITS_PER_WORD, &bits);
   if (ret == -1)
      pabort("can't get bits per word 2");

   /*
    * max speed hz
    */

   ret = ioctl(fd[0], SPI_IOC_WR_MAX_SPEED_HZ, &speed);
   if (ret == -1)
      pabort("can't set max speed hz 1");

   ret = ioctl(fd[1], SPI_IOC_WR_MAX_SPEED_HZ, &speed);
   if (ret == -1)
      pabort("can't set max speed hz 2");

   ret = ioctl(fd[0], SPI_IOC_RD_MAX_SPEED_HZ, &speed);
   if (ret == -1)
      pabort("can't get max speed hz 1");

   ret = ioctl(fd[1], SPI_IOC_RD_MAX_SPEED_HZ, &speed);
   if (ret == -1)
      pabort("can't get max speed hz 2");

   return XST_SUCCESS;
}

/* ------------------------------------------------------------ */
/*** CAN_Readbyte(PmodCAN *InstancePtr)
**
**   Synopsis:
**      byte = CAN_Readbyte(&CAN)
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**
**   Return Value:
**      u8 byte - Byte read from XSpi
**
**   Errors:
**      none
**
**   Description:
**      Reads SPI
*/
u8 CAN_ReadByte(u32 bus)
{
   u8 byte;
   transfer(fd[bus], &byte, &byte, 1);
   return byte;
}

/* ------------------------------------------------------------ */
/*** CAN_WriteSPICommand(PmodCAN *InstancePtr, u8 cmd)
**
**   Parameters:
**      InstancePtr: PmodCAN object to send to
**      cmd:         Command to send
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
void CAN_WriteByte(u32 bus, u8 cmd)
{
   transfer(fd[bus], &cmd, NULL, 1);
}

/* ------------------------------------------------------------ */
/*** CAN_WriteSpi(u8 reg, u8 *wData, int nData)
**
**   Synopsis:
**      CAN_WriteSpi(&CANobj, 0x3A, &bytearray, 5);
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      reg:         The starting register to write to
**      wData:       The data to write
**      nData:       The number of data bytes to write
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
void CAN_WriteSpi(u32 bus, u8 reg, u8 *wData, int nData)
{
   // As requested by documentation, first byte contains:
   //    bit 7 = 0 because is a write operation
   //    bit 6 = 1 if more than one bytes is written,
   //            0 if a single byte is written
   //    bits 5-0 - the address
   u8 bytearray[nData + 1];
   bytearray[0] = ((nData > 1) ? 0x40 : 0) | (reg & 0x3F);
   memcpy(&bytearray[1], wData, nData); // Copy write commands over to bytearray
   transfer(fd[bus], bytearray, 0, nData + 1);
}

/* ------------------------------------------------------------ */
/*** CAN_ReadSpi(PmodCAN *InstancePtr, u8 reg, u8 *rData, int nData)
**
**   Synopsis:
**      CAN_ReadSpi(&CANobj, 0x3A, &bytearray, 5);
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      reg:         The starting register to read from
**      rData:       The byte array to read into
**      nData:       The number of data bytes to read
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
void CAN_ReadSpi(u32 bus, u8 reg, u8 *rData, int nData)
{
   // As requested by documentation, first byte contains:
   //    bit 7 = 1 because is a read operation
   //    bit 6 = 1 if more than one bytes is written,
   //            0 if a single byte is written
   //    bits 5-0 - the address
   u8 bytearray[nData + 1];

   bytearray[0] = ((nData > 1) ? 0xC0 : 0x80) | (reg & 0x3F);
   transfer(fd[bus], bytearray, bytearray, nData + 1);
   memcpy(rData, &bytearray[1], nData);
}

/* ------------------------------------------------------------ */
/*** CAN_SetRegisterBits(PmodCAN *InstancePtr, u8 reg, u8 mask, bool fValue)
**
**   Synopsis:
**       SetRegisterBits(&CANobj, CAN_ADR_POWER_CTL, bPowerControlMask, fValue);
**
**   Parameters:
**      InstancePtr:      The PmodCAN object to communicate with
**      bRegisterAddress: The address of the register whose bits are set
**      bMask:            The mask indicating which bits are affected
**      fValue:           1 if the bits are set or 0 if their bits are reset
**
**   Return Values:
**      void
**
**   Errors:
**
**
**   Description:
**      This function sets the value of some bits (corresponding to the bMask)
**      of a register (indicated by bRegisterAddress) to 1 or 0 (indicated by
**      fValue).
*/
void CAN_SetRegisterBits(u32 bus, u8 reg, u8 mask, bool fValue)
{
   u8 regval;
   CAN_ReadSpi(bus, reg, &regval, 1);
   if (fValue)
      regval |= mask;
   else
      regval &= ~mask;
   CAN_WriteSpi(bus, reg, &regval, 1);
}

/* ------------------------------------------------------------ */
/*** CAN_GetRegisterBits(PmodCAN *InstancePtr, u8 bRegisterAddress, u8 bMask)
**
**   Synopsis:
**      return GetRegisterBits(&CANobj, CAN_ADR_BW_RATE, MSK_BW_RATE_RATE);
**
**   Parameters:
**      InstancePtr:      The PmodCAN object to communicate with
**      bRegisterAddress: The address of the register whose bits are read
**      bMask:            The mask indicating which bits are read
**
**
**   Return Values:
**      u8 - a byte containing only the bits corresponding to the mask.
**
**   Errors:
**
**
**   Description:
**      Returns a byte containing only the bits from a register (indicated by
**      bRegisterAddress), corresponding to the bMask mask.
*/
u8 CAN_GetRegisterBits(u32 bus, u8 bRegisterAddress, u8 bMask)
{
   u8 bRegValue;
   CAN_ReadSpi(bus, bRegisterAddress, &bRegValue, 1);
   return bRegValue & bMask;
}

/* ------------------------------------------------------------ */
/*** CAN_ModifyReg(PmodCAN *InstancePtr, u8 reg, u8 mask, u8 value)
**
**   Synopsis:
**      Modifies the command register using a SPI transfer
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      bReg:        The address of the register whose bits are read
**      bMask:       The mask indicating which bits are read
**      value:
**
**   Return Values:
**      void
**
**   Errors:
**
**
**   Description:
**      Modifies command register
*/
void CAN_ModifyReg(u32 bus, u8 reg, u8 mask, u8 value)
{
   u8 buf[4] = {CAN_MODIFY_REG_CMD, reg, mask, value};
   transfer(fd[bus], buf, NULL, 4);
}

/* ------------------------------------------------------------ */
/*** CAN_WriteReg(PmodCAN *InstancePtr, u8 reg, u8 *data, u32 nData)
**
**   Synopsis:
**      Modifies the write register using a SPI transfer
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      reg:         The address of the register whose bits are read
**      data:
**      nData:
**
**   Return Values:
**      void
**
**   Errors:
**
**
**   Description:
**      modifies write register
*/
void CAN_WriteReg(u32 bus, u8 reg, u8 *data, u32 nData)
{
   u8 buf[nData + 2];
   u32 i;
   buf[0] = CAN_WRITE_REG_CMD;
   buf[1] = reg;
   for (i = 0; i < nData; i++)
      buf[i + 2] = data[i];
   transfer(fd[bus], buf, NULL, nData + 2);
}

/* ------------------------------------------------------------ */
/*** CAN_ClearReg(PmodCAN *InstancePtr, u8 reg, u32 nData)
**
**   Synopsis:
**      Clears register using a SPI transfer
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      reg:         The address of the register whose bits are read
**      nData:
**
**   Return Values:
**      void
**
**   Errors:
**
**
**   Description:
**      Clears register
*/
void CAN_ClearReg(u32 bus, u8 reg, u32 nData)
{
   u8 buf[nData + 2];
   buf[0] = CAN_WRITE_REG_CMD;
   buf[1] = reg;
   transfer(fd[bus], buf, NULL, nData + 2);
}

/* ------------------------------------------------------------ */
/*** CAN_LoadTxBuffer(PmodCAN *InstancePtr, u8 start_addr, u8 *data, u32 nData)
**
**   Synopsis:
**      Loads the TX buffer using a SPI transfer
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      start_addr:
**      data:
**      nData:
**
**   Return Values:
**      void
**
**   Errors:
**
**
**   Description:
**      Loads transmit buffer
*/
void CAN_LoadTxBuffer(u32 bus, u8 start_addr, u8 *data,
                      u32 nData)
{
   u8 buf[nData + 1];
   u32 i;
   buf[0] = CAN_LOADBUF_CMD | start_addr;
   for (i = 0; i < nData; i++)
      buf[i + 1] = data[i];
   transfer(fd[bus], buf, NULL, nData + 1);
}

/* ------------------------------------------------------------ */
/*** CAN_RequestToSend(PmodCAN *InstancePtr, u8 mask)
**
**   Synopsis:
**      request to send using a SPI transfer
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      mask:
**
**   Return Values:
**      void
**
**   Errors:
**
**
**   Description:
**      Request to send
*/
void CAN_RequestToSend(u32 bus, u8 mask)
{
   u8 buf[1] = {CAN_RTS_CMD | mask};
   transfer(fd[bus], buf, NULL, 1);
}

/* ------------------------------------------------------------ */
/*** CAN_ReadRxBuffer(PmodCAN *InstancePtr, u8 start_addr, u8 *data, u32 nData)
**
**   Synopsis:
**      Read RX buffer using a SPI transfer
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      start_addr:
**      data:
**      nData:
**
**   Return Values:
**      void
**
**   Errors:
**
**
**   Description:
**      Reads the receive buffer
*/
void CAN_ReadRxBuffer(u32 bus, u8 start_addr, u8 *data,
                      u32 nData)
{
   u8 buf[nData + 1];
   u32 i;
   buf[0] = CAN_READBUF_CMD | start_addr;
   transfer(fd[bus], buf, buf, nData + 1);
   for (i = 0; i < nData; i++)
      data[i] = buf[i + 1];
}

/* ------------------------------------------------------------ */
/*** CAN_ReadReg(PmodCAN *InstancePtr, u8 reg, u8 *data, u32 nData)
**
**   Synopsis:
**      Read register using a SPI transfer
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      reg:
**      data:
**      nData:
**
**   Return Values:
**      void
**
**   Errors:
**
**
**   Description:
**      Reads register
*/
void CAN_ReadReg(u32 bus, u8 reg, u8 *data, u32 nData)
{
   u8 buf[nData + 2];
   u32 i;
   buf[0] = CAN_READ_REG_CMD;
   buf[1] = reg;

   transfer(fd[bus], buf, buf, nData + 2);
   for (i = 0; i < nData; i++)
      data[i] = buf[i + 2];
}

/* ------------------------------------------------------------ */
/*** CAN_ReadStatus(PmodCAN *InstancePtr)
**
**   Synopsis:
**      Read status register using a SPI transfer
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**
**   Return Values:
**      u8 buf
**
**   Errors:
**
**
**   Description:
**      Reads the status register
*/
u8 CAN_ReadStatus(u32 bus)
{
   u8 buf[2] = {CAN_READSTATUS_CMD, 0x00};
   transfer(fd[bus], buf, buf, 2);
   return buf[1];
}

/* ------------------------------------------------------------ */
/*** CAN_RxStatus(PmodCAN *InstancePtr)
**
**   Synopsis:
**      Reads RXStatus register using a SPI transfer
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**
**   Return Values:
**      u8 buf
**
**   Errors:
**
**
**   Description:
**      Reads RXStatus register
*/
u8 CAN_RxStatus(u32 bus)
{
   u8 buf[2] = {CAN_READSTATUS_CMD, 0x00};
   transfer(fd[bus], buf, buf, 2);
   return buf[1];
}

/* ------------------------------------------------------------ */
/*** CAN_Configure(PmodCAN *InstancePtr, u8 mode)
**
**   Synopsis:
**      Configures the PmodCAN using a SPI transfer
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      mode:
**
**   Return Values:
**      void
**
**   Errors:
**
**
**   Description:
**      Configures PmodCAN
*/
void CAN_Configure(u32 bus, u8 mode)
{
   u8 CNF[3] = {0x86, 0xFB, 0x41};

   // Set CAN control mode to configuration
   CAN_ModifyReg(bus, CAN_CANCTRL_REG_ADDR, CAN_CAN_CANCTRL_MODE_MASK,
                 CAN_ModeConfiguration);

   // Set config rate and clock for CAN
   CAN_WriteReg(bus, CAN_CNF3_REG_ADDR, CNF, 3);

   CAN_ClearReg(bus, 0x00, 12); // Initiate CAN buffer filters and
   CAN_ClearReg(bus, 0x10, 12); // registers
   CAN_ClearReg(bus, 0x20, 8);
   CAN_ClearReg(bus, 0x30, 14);
   CAN_ClearReg(bus, 0x40, 14);
   CAN_ClearReg(bus, 0x50, 14);

   // Set the CAN mode for any message type
   CAN_ModifyReg(bus, CAN_RXB0CTRL_REG_ADDR, 0x64, 0x60);

   // Set CAN control mode to selected mode (exit configuration)
   CAN_ModifyReg(bus, CAN_CANCTRL_REG_ADDR, CAN_CAN_CANCTRL_MODE_MASK,
                 mode << CAN_CANCTRL_MODE_BIT);
}

/* ------------------------------------------------------------ */
/*** CAN_SendMessage(PmodCAN *InstancePtr, CAN_Message message,
**         CAN_TxBuffer target)
**
**   Synopsis:
**      Send message
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      message:
**      target:
**
**   Return Values:
**      XStatus XST_SUCCESS
**
**   Errors:
**
**
**   Description:
**      Sends message
*/
XStatus CAN_SendMessage(u32 bus, CAN_Message message,
                        CAN_TxBuffer target)
{
   u8 data[13];
   u8 i;

   u8 rts_mask;
   u8 load_start_addr;

   switch (target)
   {
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

   CAN_LoadTxBuffer(bus, load_start_addr, data, message.dlc + 5);
   CAN_RequestToSend(bus, rts_mask);

   return XST_SUCCESS;
}

/* ------------------------------------------------------------ */
/*** CAN_ReceiveMessage(PmodCAN *InstancePtr, CAN_Message *MessagePtr,
**         CAN_RxBuffer target)
**
**   Synopsis:
**      Receives message
**
**   Parameters:
**      InstancePtr: The PmodCAN object to communicate with
**      MessagePtr:
**      target:
**
**   Return Values:
**      XStatus XST_SUCCESS
**
**   Errors:
**
**
**   Description:
**      Receives message
*/
XStatus CAN_ReceiveMessage(u32 bus, CAN_Message *MessagePtr,
                           CAN_RxBuffer target)
{
   u8 data[13];
   u8 i;
   u8 read_start_addr;

   switch (target)
   {
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

   MessagePtr->id = (u16)data[0] << 3;
   MessagePtr->id |= (data[1] & 0xE0) >> 5;

   MessagePtr->ide = (data[1] & 0x08) >> 3;

   MessagePtr->srr = (data[1] & 0x10) >> 4;

   MessagePtr->eid = (u32)(data[1] & 0x03) << 16;
   MessagePtr->eid |= (u32)(data[2] & 0xFF) << 8;
   MessagePtr->eid |= (u32)(data[3] & 0xFF);

   MessagePtr->rtr = (data[4] & 0x40) >> 6;

   MessagePtr->dlc = data[4] & 0x0F;

   // Read only relevant data bytes
   CAN_ReadRxBuffer(bus, read_start_addr, data, MessagePtr->dlc);

   for (i = 0; i < MessagePtr->dlc; i++)
      MessagePtr->data[i] = data[i + 5];

   return XST_SUCCESS;
}
