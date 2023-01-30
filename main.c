#include "PmodCAN.h"
#include "sleep.h"
#include "xparameters.h"
#include "stdio.h"

void Initialize();
void Cleanup();
void PrintMessage(CAN_Message message);

int main()
{
   Initialize();

   // commands
   readPID(0x141);

   Cleanup();
   return 0;
}

void Initialize()
{
   CAN_begin();
   CAN_Configure(CAN_BUS1, CAN_ModeLoopback);
}

void PrintMessage(CAN_Message message)
{
   u8 i;

   printf("message:\r\n");

   printf("    %s Frame\r\n", (message.ide) ? "Extended" : "Standard");
   printf("    ID: %03x\r\n", message.id);

   if (message.ide)
      printf("    EID: %05x\r\n", message.eid);

   if (message.rtr)
      printf("    Remote Transmit Request\r\n");

   else
      printf("    Standard Data Frame\r\n");

   printf("    dlc: %01x\r\n", message.dlc);
   printf("    data:\r\n");

   for (i = 0; i < message.dlc; i++)
      printf("        %02x\r\n", message.data[i]);
}

void WriteCmd(CAN_Message message)
{
   CAN_Message TxMessage;
   u8 status;

   printf("Welcome to the PmodCAN IP Core Loopback Demo\r\n");
   // Set CAN control mode to configuration

   while (1)
   {
      printf("Bus 1 *****************************\r\n");
      // Wait for buffer 0 to be clear
      do
      {
         status = CAN_ReadStatus(CAN_BUS1);
         printf("Waiting to send\r\n");
      } while ((status & CAN_STATUS_TX0REQ_MASK) != 0);

      TxMessage = message;

      printf("sending ");
      PrintMessage(TxMessage);

      CAN_ModifyReg(CAN_BUS1, CAN_CANINTF_REG_ADDR, CAN_CANINTF_TX0IF_MASK, 0);

      printf("requesting to transmit message through transmit buffer 0 \
            \r\n");

      CAN_SendMessage(CAN_BUS1, TxMessage, CAN_Tx0);

      CAN_ModifyReg(CAN_BUS1, CAN_CANINTF_REG_ADDR, CAN_CANINTF_TX0IF_MASK, 0);

      // Wait for message to transmit successfully
      do
      {
         status = CAN_ReadStatus(CAN_BUS1);

         printf("Waiting to complete transmission\r\n");
      } while ((status & CAN_STATUS_TX0IF_MASK) != 0);
   }
}

void ReadCmd(CAN_Message RxMessage)
{
   CAN_RxBuffer target;
   u8 status;
   u8 rx_int_mask;

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

   CAN_ReceiveMessage(CAN_BUS1, &RxMessage, target);
   CAN_ModifyReg(CAN_BUS1, CAN_CANINTF_REG_ADDR, rx_int_mask, 0);

   printf("received \r\n");
   PrintMessage(RxMessage);
}

void Cleanup()
{
   CAN_end();
}
