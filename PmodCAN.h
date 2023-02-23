/******************************************************************************/
/*                                                                            */
/* PmodCAN.h -- PmodCAN Example Projects                                      */
/*                                                                            */
/******************************************************************************/
/* Author: Arthur Brown                                                       */
/*                                                                            */
/******************************************************************************/
/* File Description:                                                          */
/*                                                                            */
/* This file contains definitions for the PmodCAN driver                      */
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

#ifndef PmodCAN_H
#define PmodCAN_H

/****************** Include Files ********************/
#include "xstatus.h"
#include "stdint.h"

/* ------------------------------------------------------------ */
/*                  Definitions                                 */
/* ------------------------------------------------------------ */

#define bool uint8_t
#define true 1
#define false 0

/** DRIVER HEADER MATERIALS **/

#define CAN_BUS1                  0
#define CAN_BUS2                  1


#define CAN_MODIFY_REG_CMD        0x05

#define CAN_WRITE_REG_CMD         0x02

#define CAN_READ_REG_CMD          0x03

#define CAN_RTS_CMD               0x80
#define CAN_RTS_TXB0_MASK         0x01
#define CAN_RTS_TXB1_MASK         0x02
#define CAN_RTS_TXB2_MASK         0x04

#define CAN_LOADBUF_CMD           0x40
#define CAN_LOADBUF_TXB0SIDH      0x00
#define CAN_LOADBUF_TXB0D0        0x01
#define CAN_LOADBUF_TXB1SIDH      0x02
#define CAN_LOADBUF_TXB1D0        0x03
#define CAN_LOADBUF_TXB2SIDH      0x04
#define CAN_LOADBUF_TXB2D0        0x05

#define CAN_READBUF_CMD           0x90
#define CAN_READBUF_RXB0SIDH      0x00
#define CAN_READBUF_RXB0D0        0x01
#define CAN_READBUF_RXB1SIDH      0x02
#define CAN_READBUF_RXB1D0        0x03

#define CAN_READSTATUS_CMD        0xA0
#define CAN_STATUS_RX0IF_MASK     0x01
#define CAN_STATUS_RX1IF_MASK     0x02
#define CAN_STATUS_TX0REQ_MASK    0x04
#define CAN_STATUS_TX0IF_MASK     0x08
#define CAN_STATUS_TX1REQ_MASK    0x10
#define CAN_STATUS_TX1IF_MASK     0x20
#define CAN_STATUS_TX2REQ_MASK    0x40
#define CAN_STATUS_TX2IF_MASK     0x80

#define CAN_RXSTATUS_CMD          0xB0
#define CAN_RXSTATUS_RX0IF_MASK   0x40
#define CAN_RXSTATUS_RX1IF_MASK   0x80

#define CAN_CANCTRL_REG_ADDR      0x0F
#define CAN_CNF3_REG_ADDR         0x28
#define CAN_CNF2_REG_ADDR         0x29
#define CAN_CNF1_REG_ADDR         0x2A
#define CAN_RXB0CTRL_REG_ADDR     0x60

#define CAN_TXB0CTRL_REG_ADDR     0x30
#define CAN_TXB0CTRL_TXREQ_MASK   0x08

#define CAN_CANINTF_REG_ADDR      0x2C
#define CAN_CANINTF_RX0IF_MASK    0x01
#define CAN_CANINTF_RX1IF_MASK    0x02
#define CAN_CANINTF_TX0IF_MASK    0x04
#define CAN_CANINTF_TX1IF_MASK    0x08
#define CAN_CANINTF_TX2IF_MASK    0x10
#define CAN_CAN_CANCTRL_MODE_MASK 0xE0
#define CAN_CANCTRL_MODE_BIT      5

typedef struct CAN_Message {
   uint16_t id;     // 11 bit id
   uint32_t eid;    // 18 bit extended id
   uint8_t ide;     // 1 to enable sending extended id
   uint8_t rtr;     // Remote transmission request bit
   uint8_t srr;     // Standard Frame Remote Transmit Request
   uint8_t dlc;     // Data length
   uint8_t data[8]; // Data buffer
   // Some additional information has not yet been encapsulated here
   // (ex:priority bits), primarily, no TXBxCTRL bits
} CAN_Message;

typedef enum CAN_RxBuffer {
   CAN_Rx0 = 0, CAN_Rx1
} CAN_RxBuffer;

typedef enum CAN_TxBuffer {
   CAN_Tx0 = 0, CAN_Tx1, CAN_Tx2
} CAN_TxBuffer;

typedef enum CAN_Mode {
   CAN_ModeNormalOperation = 0,
   CAN_ModeSleep,
   CAN_ModeLoopback,
   CAN_ModeListenOnly,
   CAN_ModeConfiguration = 0x80
} CAN_Mode;

void CAN_begin();
void CAN_end();
int CAN_SPIInit();
uint8_t CAN_ReadByte(uint32_t bus);
void CAN_WriteByte(uint32_t bus, uint8_t cmd);
void CAN_WriteSPI(uint32_t bus,uint8_t reg, uint8_t *wData, int nData);
void CAN_ReadSPI(uint32_t bus, uint8_t reg, uint8_t *rData, int nData);
void CAN_SetRegisterBits(uint32_t bus, uint8_t reg, uint8_t mask, bool fValue);
uint8_t CAN_GetRegisterBits(uint32_t bus, uint8_t bRegisterAddress, uint8_t bMask);
void CAN_ModifyReg(uint32_t bus,uint8_t reg, uint8_t mask, uint8_t value);
void CAN_WriteReg(uint32_t bus,uint8_t reg, uint8_t *data, uint32_t nData);
void CAN_ClearReg(uint32_t bus, uint8_t reg, uint32_t nData);
void CAN_LoadTxBuffer(uint32_t bus, uint8_t start_addr, uint8_t *data, uint32_t nData);
void CAN_RequestToSend(uint32_t bus,uint8_t mask);
void CAN_ReadRxBuffer(uint32_t bus, uint8_t start_addr, uint8_t *data, uint32_t nData);
void CAN_ReadReg(uint32_t bus, uint8_t reg, uint8_t *data, uint32_t nData);
uint8_t CAN_ReadStatus(uint32_t bus);
uint8_t CAN_RxStatus(uint32_t bus);
void CAN_Configure(uint32_t bus,uint8_t mode); // This function is missing some potential parameters
XStatus CAN_SendMessage(uint32_t bus,CAN_Message message,
      CAN_TxBuffer target);
XStatus CAN_ReceiveMessage(uint32_t bus,CAN_Message *MessagePtr,
      CAN_RxBuffer target);

#endif // PmodCAN_H
