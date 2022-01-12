// Copyright (c) 2018 Flight Dynamics and Control Lab  Kanishke Gamagedara

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
#include "definitions.h"
#if defined(VECTTORNAV_VN100)                                                   /* ==== Vector NAVS VN100 IMU/AHRS ==== */
#include <stdint.h>
#include "vn100.h"
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Ported from above code by ACP Aviation

/* ====================== function definitions ============================== */ 
void vn100_init_uart( uint8_t uartNo );
uint16_t calculate_imu_crc(uint8_t dataV[], uint16_t length);
uint8_t vn100RecvPacketNoBlock(VN100_VectorNavRcv_t* Response, uint8_t uART_no);
void check_sync_byte(VN100_VectorNavRcv_t *msgIn, uint8_t uART_no);
void read_imu_data(VN100_VectorNavRcv_t *msgIn, uint8_t uART_no);

/*-----------------------------------------------------------------------------
 *      vn100_init_uart :  Initialises the UART to baud rate for vectorNAV 
 *  
 *  Parameters: uint8_t uartNo
 *  Return: void   
 *----------------------------------------------------------------------------*/
void vn100_init_uart( uint8_t uartNo )
{
   switch (uartNo)
   {
      case 1u:
      UART1_Init(VECTNAV_VN100_BAUD);
      break;

      case 2u:
      UART2_Init(VECTNAV_VN100_BAUD);
      break;
     
      case 3u:
      UART3_Init(VECTNAV_VN100_BAUD);
      break;
      
      case 4u:
      UART4_Init(VECTNAV_VN100_BAUD);
      break;
      
      case 5u:
      UART5_Init(VECTNAV_VN100_BAUD);
      break;
      
      case 6u:
      UART6_Init(VECTNAV_VN100_BAUD);
      break;
      
      default:
      break;      
   }
}

/*-----------------------------------------------------------------------------
 *      calculate_imu_crc :  Calculate the 16-bit CRC for the given ASCII or binary message. 
 *  
 *  Parameters: uint8_t dataV[], uint16_t length
 *  Return: uint16_t   
 *----------------------------------------------------------------------------*/
uint16_t calculate_imu_crc(uint8_t dataV[], uint16_t length)
{
  uint16_t i;
  uint16_t crc = 0u;
  for(i=0; i<length; i++)
  {
    crc = (uint8_t)(crc >> 8u) | (crc << 8u);
    crc ^= dataV[i];
    crc ^= (uint8_t)(crc & 0xffu) >> 4u;
    crc ^= crc << 12u;
    crc ^= (crc & 0x00ffu) << 5u;
  }
  return crc;
}

/*-----------------------------------------------------------------------------
 *      vn100RecvPacketNoBlock :  Receives and parses a message from vn100 AHRS 
 *  
 *  Parameters: VN100_VectorNavRcv_t* Response, uint8_t uART_no
 *  Return:  true for char was read   
 *----------------------------------------------------------------------------*/
uint8_t vn100RecvPacketNoBlock(VN100_VectorNavRcv_t* Response, uint8_t uART_no)
{
        int8_t ret = 0;
        switch (uART_no)                                                        // read from the chosen UART
        {
           case 1u:
           if (UART1_Data_Ready())
           {
              Response->in[Response->bytesRead] = UART1_Read();
              ++Response->bytesRead % UINT16_MAX;
              ret = 1;
           }
           break;

           case 2u:
           if (UART2_Data_Ready())
           {
              Response->in[Response->bytesRead] = UART2_Read();
              ++Response->bytesRead % UINT16_MAX;
              ret = 1;
           }
           break;

           case 3u:
           if (UART3_Data_Ready())
           {
              Response->in[Response->bytesRead] = UART3_Read();
              ++Response->bytesRead % UINT16_MAX;
              ret = 1;
           }
           break;

           case 4u:
           if (UART4_Data_Ready())
           {
              Response->in[Response->bytesRead] = UART4_Read();
              ++Response->bytesRead % UINT16_MAX;
              ret = 1;
           }
           break;

           case 5u:
           if (UART5_Data_Ready())
           {
              Response->in[Response->bytesRead] = UART5_Read();
              ++Response->bytesRead % UINT16_MAX;
              ret = 1;
           }
           break;

           case 6u:
           if (UART6_Data_Ready())
           {
              Response->in[Response->bytesRead] = UART6_Read();
              ++Response->bytesRead % UINT16_MAX;
              ret = 1;
           }
           break;
           
           default:                                                             // not a known port
           Response->bytesRead=0U;
           break;
       }
       return ret;
}

/*-----------------------------------------------------------------------------
 *   check_sync_byte :  Check for the sync byte (0xFA)
 *  
 *  Parameters: VN100_VectorNavRcv_t *msgIn, uint8_t uART_no
 *  Return: void   
 *----------------------------------------------------------------------------*/
void check_sync_byte(VN100_VectorNavRcv_t *msgIn, uint8_t uART_no) 
{ 
  int8_t i;
  
  msgIn->bytesRead = 0u;
  for (i = 0; i < 6; i++) 
  {
    if (vn100RecvPacketNoBlock(msgIn, uART_no)==1u)
    {
        if (msgIn->in[0u] == VECTNAV_VN100_STRT) 
        {
           msgIn->imu_sync_detected = 1u;
           break;
        }
    }
  } 
}
 
/*-----------------------------------------------------------------------------
 *   read_imu_data :  Read the IMU bytes
 *  
 *  Parameters: VN100_VectorNavRcv_t *msgIn, uint8_t uART_no
 *  Return: void   
 *----------------------------------------------------------------------------*/
void read_imu_data(VN100_VectorNavRcv_t *msgIn, uint8_t uART_no) 
{
  int8_t i = 0;
  
  switch (msgIn->rcvState)
  {
     case VN100_NO_DATA_READ:
       msgIn->imu_sync_detected = 0u;
       msgIn->rcvState = VN100_PACKET_READ_SYNC;                                /* insert tick delays here if you need them as fast driver didnt bother */
     break;
     
     case VN100_PACKET_READ_SYNC:                                               /* now look for sync */
       check_sync_byte(msgIn, uART_no);
       if (msgIn->imu_sync_detected == 1u) 
       {
          msgIn->rcvState = VN100_PACKET_READ_DATA;
          msgIn->collectCnt = 0u;
       }
     break;
     
     case VN100_PACKET_READ_DATA:                                               /* read packet */
       if (vn100RecvPacketNoBlock(msgIn, uART_no)==1u)
       {
          if (msgIn->bytesRead == VECTNAV_VN100_DATLEN)                         /* message complete */
          {
             msgIn->rcvState = VN100_PACKET_READ_COMPLETE;
          }
          msgIn->collectCnt = 0u;                                               /* comment out if you want timeout to be overall rather than per char */
       }
       else
       {
          if (msgIn->collectCnt >= VECTNAV_VN100_TIMEOUT)                       /* time since last char is too long */
          {
             msgIn->rcvState = VN100_DATA_CORRUPT;           
          }
          ++msgIn->collectCnt % UINT16_MAX;
       }
     break;
     
     case VN100_PACKET_READ_COMPLETE:                                           /* process packet */
       msgIn->checksum.b[0u] = msgIn->in[40u];
       msgIn->checksum.b[1u] = msgIn->in[39u];
       if (calculate_imu_crc(msgIn->in, 39u) == msgIn->checksum.s) 
       {
           for (i = 0; i < VECTNAV_VN100_DATSIZ; i++) 
           {
               msgIn->yaw.b[i] = msgIn->in[3u + i];
               msgIn->pitch.b[i] = msgIn->in[7u + i];
               msgIn->roll.b[i] = msgIn->in[11u + i];
               msgIn->W_x.b[i] = msgIn->in[15u + i];
               msgIn->W_y.b[i] = msgIn->in[19u + i];
               msgIn->W_z.b[i] = msgIn->in[23u + i];
               msgIn->a_x.b[i] = msgIn->in[27u + i];
               msgIn->a_y.b[i] = msgIn->in[31u + i];
               msgIn->a_z.b[i] = msgIn->in[35u + i];
           }
           msgIn->collectCnt = 0u; 
           msgIn->rcvState = VN100_PACKET_PROCESS_COMPLETE;    
       }
       else
       {
          msgIn->rcvState = VN100_DATA_CORRUPT; 
       }
     break;
     
     case VN100_DATA_CORRUPT:                                                   /* count bad messages */
         ++msgIn->msgFail % UINT16_MAX;
         msgIn->rcvState = VN100_NO_DATA_READ;
     break;

     case VN100_PACKET_PROCESS_COMPLETE:                                        /* wait for handshake from caller or a poll timeout */
        ++msgIn->collectCnt % UINT16_MAX;
        if (msgIn->collectCnt >= VECTNAV_VN100_POLLTIME)                        /* handshake too long */
        {
             msgIn->rcvState = VN100_PACKET_READ_SYNC;                          /* go back to looking for new data as poll time expired */         
        }
     break;
          
     default:                                                                   /* catch error */
        msgIn->rcvState = VN100_NO_DATA_READ;
     break;
     
  }

}

#endif