#include "PmodCAN.h"
#include "CAN.h"

void Initialize();
void Cleanup();
void PrintMessage(const CAN_Message *message);
void WriteCmd(const CAN_Message *message);
void ReadCmd(CAN_Message *message);

void Initialize()
{
   CAN_begin();
   CAN_Configure(CAN_BUS1,CAN_ModeNormalOperation);
}

void PrintMessage(const CAN_Message *message)
{
   uint8_t i;

   printf("message:\r\n");

   printf("    %s Frame\r\n", (message->ide) ? "Extended" : "Standard");
   printf("    ID: %03x\r\n", message->id);

   if (message->ide)
      printf("    EID: %05x\r\n", message->eid);

   if (message->rtr)
      printf("    Remote Transmit Request\r\n");

   else
      printf("    Standard Data Frame\r\n");

   printf("    dlc: %01x\r\n", message->dlc);
   printf("    data:\r\n");

   for (i = 0; i < message->dlc; i++)
      printf("        %02x\r\n", message->data[i]);
}

void WriteCmd(CAN_Message *TxMessage)
{
   uint8_t status;

   printf("Bus 1 *****************************\r\n");
   // Wait for buffer 0 to be clear
   do
   {
      status = CAN_ReadStatus(CAN_BUS1);
      printf("Waiting to send\r\n");
   } while ((status & CAN_STATUS_TX0REQ_MASK) != 0);

   printf("sending ");
   PrintMessage(TxMessage);

   CAN_ModifyReg(CAN_BUS1, CAN_CANINTF_REG_ADDR, CAN_CANINTF_TX0IF_MASK, 0);

   printf("requesting to transmit message through transmit buffer 0\r\n");

   CAN_SendMessage(CAN_BUS1, *TxMessage, CAN_Tx0);
   CAN_ModifyReg(CAN_BUS1, CAN_CANINTF_REG_ADDR, CAN_CANINTF_TX0IF_MASK, 0);

   // Wait for message to transmit successfully
   do
   {
      status = CAN_ReadStatus(CAN_BUS1);
      printf("Waiting to complete transmission\r\n");
   } while ((status & CAN_STATUS_TX0IF_MASK) != 0);
}

void ReadCmd(CAN_Message *RxMessage)
{
   CAN_RxBuffer target;
   uint8_t status;
   uint8_t rx_int_mask;

   do
   {
      status = CAN_ReadStatus(CAN_BUS1);

      printf("Waiting to receive\r\n");
   } while ((status & CAN_STATUS_RX0IF_MASK) != 0 && (status & CAN_STATUS_RX1IF_MASK) != 0);

   printf("------> Read Status: %x \n", status);

   switch (status & 0x03)
   {
   case 0b01:
   case 0b11:
      printf("fetching message from receive buffer 0\r\n");
      target = CAN_Rx0;
      rx_int_mask = CAN_CANINTF_RX0IF_MASK;
      break;
   case 0b10:
      printf("fetching message from receive buffer 1\r\n");
      target = CAN_Rx1;
      rx_int_mask = CAN_CANINTF_RX1IF_MASK;
      break;
   default:
      printf("Error, message not received\r\n");
   }

   CAN_ReceiveMessage(CAN_BUS1, RxMessage, target);
   CAN_ModifyReg(CAN_BUS1,CAN_CANINTF_REG_ADDR, rx_int_mask, 0);

   printf("received \r\n");
   PrintMessage(RxMessage);
}

void Cleanup()
{
   CAN_end();
}
