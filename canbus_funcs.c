//   Simple UAVCAN CanBus implementations
//   uses extacts from Pavel Kirienko <pavel.kirienko@zubax.com>
//
//   Version : @(#) 1.0
//   Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#include <stdint.h>
#include "canbus_defs.h"
#include "crc.h"

//volatile canInhibitClass gPrioInhibit;                                        it is this but the find declaration is then locked
volatile uint8_t gPrioInhibit;                                                  // globally set priority level for avcan sending

uint64_t readUptime();
void Init_CAN_Port(unsigned char *Can_Init_Flags, unsigned char *Can_Send_Flags, unsigned char *Can_Rcv_Flags, char *FIFObuffers);
uint16_t make_float16(float32_t value);
int16_t publish_true_airspeed(float32_t mean, float32_t variance,uint8_t uavcan_node_id, uint8_t transfer_id);
int16_t publish_node_status(canAbstractNodeHealthClass health, canCurrentModeClass mode, uint16_t vendor_specific_status_code,uint8_t uavcan_node_id, uint8_t transfer_id);
int16_t uavcan_broadcast(canPriorityClass priority, uint16_t data_type_id, uint8_t transfer_id, const uint8_t* payload, uint16_t payload_len,uint8_t uavcan_node_id);
int8_t computeForwardDistance(uint8_t a, uint8_t b);

/*-----------------------------------------------------------------------------
 *      readUptime:  Makes the uptime global variable (used in e.g. CAN message)
 *
 *  Parameters: (none)
 *  Return:     uint64_t
 *----------------------------------------------------------------------------*/
uint64_t readUptime()
{
   uint16_t secondsLow;
   uint16_t secondsHigh;
   uint16_t secondsActual;
   uint16_t minutesLow;
   uint16_t minutesHigh;
   uint16_t minutesActual;
   uint16_t hoursLow;
   uint16_t hoursHigh;
   uint16_t hoursActual;

   secondsLow = (RTCTIME & 0xF00ul)>>8u;                                        // Grab the RTC to show the timestamp
   secondsHigh = (RTCTIME & 0x7000ul)>>12u;
   secondsActual = (secondsHigh * 10u) + secondsLow;
   minutesLow = (RTCTIME & 0xF0000ul)>>16u;
   minutesHigh = (RTCTIME & 0x700000ul)>>20u;
   minutesActual = (minutesHigh * 10u) + minutesLow;
   hoursLow = (RTCTIME & 0xF000000ul)>>24u;
   hoursHigh = (RTCTIME & 0x30000000ul)>>28u;
   hoursActual = (hoursHigh * 10u) + hoursLow;
   return ((((hoursActual*24u)*60u) + (minutesActual * 60u)) + secondsActual);  // seconds in day consider date if up longer than a day (battery life)
}
/*-----------------------------------------------------------------------------
 *      Init_CAN_Port:  Initialize chosen can port
 *
 *  Parameters: unsigned char *Can_Init_Flags, unsigned char *Can_Send_Flags, unsigned char *Can_Rcv_Flags, char *FIFObuffers
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Init_CAN_Port(unsigned char *Can_Init_Flags, unsigned char *Can_Send_Flags, unsigned char *Can_Rcv_Flags, char *FIFObuffers)
{
  Can_Init_Flags = 0u;                                                          //
  Can_Send_Flags = 0u;                                                          // clear flags
  Can_Rcv_Flags  = 0u;                                                          //

  *Can_Send_Flags = _CAN_TX_PRIORITY_0 &                                        //form value to be used
                   _CAN_TX_XTD_FRAME &                                          // with CANWrite
                   _CAN_TX_NO_RTR_FRAME;

  *Can_Init_Flags = _CAN_CONFIG_SAMPLE_THRICE &                                  // form value to be used
                   _CAN_CONFIG_PHSEG2_PRG_ON &                                  // with CANInit
                   _CAN_CONFIG_XTD_MSG &                                        //_CAN_CONFIG_DBL_BUFFER_ON &
                   _CAN_CONFIG_STD_MSG &
                   _CAN_CONFIG_LINE_FILTER_OFF;

  CAN1Initialize(1,4,5,3,1,*Can_Init_Flags);                                    //'OSC 40Mhz 500Kb/Sec
  CAN1SetOperationMode(_CAN_MODE_CONFIG,0xFFU);                                 // set CONFIGURATION mode
  CAN1SetMask(_CAN_MASK_1,-1,_CAN_CONFIG_STD_MSG);                              // set all mask1 bits to ones
  CAN1SetMask(_CAN_MASK_2,-1,_CAN_CONFIG_STD_MSG);                              // set all mask2 bits to ones
  CAN1AssignBuffer(FIFObuffers);                                                //assign the buffers
  //configure rx fifo
  CAN1ConfigureFIFO(0u, 8u,_CAN_FIFO_RX & _CAN_FULL_MESSAGE);                   //RX buffer 8 messages deep
  //configure tx fifo
  CAN1ConfigureFIFO(1u, 8u,_CAN_FIFO_TX & _CAN_TX_PRIORITY_3 & _CAN_TX_NO_RTR_FRAME);// TX buffer 8 messages deep

  CAN1SetOperationMode(_CAN_MODE_NORMAL,0xFFu);                                 // set NORMAL mode
  crc_ccit16_init();                                                            // initialize crc16
}
 /*-----------------------------------------------------------------------------
 *      computeForwardDistance():  compute the transfer id
 *
 *  Parameters: uint8_t a, uint8_t b
 *
 *  Return:     int8_t : transfer id
 *----------------------------------------------------------------------------*/
#define TransferIDBitLength 5u
int8_t computeForwardDistance(uint8_t a, uint8_t b)
{
    uint8_t MaxValue;
    int16_t d;

    MaxValue = (1U << TransferIDBitLength) - 1U;
    //assert((a <= MaxValue) && (b <= MaxValue));
    if ((a > MaxValue) || (b >= MaxValue))
       return (0u);                                                             // return error

    d = (int16_t)(b) - (int16_t)(a);
    if (d < 0)
    {
        d += 1U << TransferIDBitLength;
    }

    //assert((d <= MaxValue) && (d >= -MaxValue));
    if ((d > MaxValue) || (d < -MaxValue))
       return (0u);                                                             // return error
    //assert(((a + d) & MaxValue) == b);
    if (!(((a + d) & MaxValue) == b))
       return (0u);                                                             // return error
    return (int8_t)(d);                                                         // return value
}
 /*-----------------------------------------------------------------------------
 *      uavcan_broadcast():  send the uavcan byte stream to the target device
 *
 *  Parameters: canPriorityClass priority, uint16_t data_type_id, uint8_t transfer_id, 
 *              const uint8_t* payload, uint16_t payload_len, uint8_t uavcan_node_id
 *  Return:     int16_t : return from the canwrite call
 *----------------------------------------------------------------------------*/
int16_t uavcan_broadcast(canPriorityClass priority, uint16_t data_type_id, uint8_t transfer_id, const uint8_t* payload, uint16_t payload_len, uint8_t uavcan_node_id)
{
    uint32_t can_id;                                                            // can id double word
    uint8_t payload_with_tail_byte[8u];                                         // payload plus tail byte to send
    uint8_t index;                                                              // index for arrays loop counters etc
    uint8_t noOfmsg=1u;                                                         // message counter starts after header hence intialises at 1
    uint8_t tail_byte_toggle;                                                   // toggle bit in tail bit byte at end of can packet
    uint8_t offset=0u;                                                          // offset used as it sends chunks of the payload in each can packet sent
    uint16_t crc_ccit16;                                                        // returned crc16
    
    if (payload == NULL)
    {
        return -1;                                                              // In this super-simple implementation we don't support multi-frame transfers.
    }

    if ((priority >= AVCAN_PRIORITY_LOW) && gPrioInhibit==UAVCAN_INH_LOW)       // less than medium priority and global priority inhibit is set then dont send low priority messages
    {
        return -1;
    }
    else if ((priority >= AVCAN_PRIORITY_MEDIUM) && gPrioInhibit==UAVCAN_INH_MEDIUM)  // less than high priority and global priority inhibit is set then dont send medium or low priority messages
    {
        return -1;
    }
    else if ((priority >= AVCAN_PRIORITY_HIGH) && gPrioInhibit==UAVCAN_INH_HIGH)  // less than highest priority and global priority inhibit is set then dont send high medium low priority messages
    {
        return -1;
    }
    else if ((priority >= AVCAN_PRIORITY_HIGHEST) && gPrioInhibit==UAVCAN_INH_ALL) // inhibit all priority and global priority inhibit is set then dont send any messages
    {
        return -1;
    }
    
    can_id = ((uint32_t)priority << 24U) | ((uint32_t)data_type_id << 8U) | (uint32_t)uavcan_node_id;

    if (payload_len <= 8u)                                                      // single message send 7 or less bytes in payload
    {
      memcpy((void*)payload_with_tail_byte, (void*) payload, (int16_t) payload_len);
      payload_with_tail_byte[payload_len+1u] = 0xC0U | (transfer_id & 31U);     // start bit end bit no toggle
      return(CAN1Write(can_id, (unsigned char*) payload_with_tail_byte, payload_len + 1u, (uint16_t) ((priority & _CAN_TX_STD_FRAME) & _CAN_TX_NO_RTR_FRAME) ));  // need to make can send wrapper
    }
    else
    {
       crc_ccit16=crc_ccit16_fast(payload, (int8_t) payload_len);               // do CRC16
       payload_with_tail_byte[0u] = (crc_ccit16&0xFFu);                         // append CRC16 lobyte
       payload_with_tail_byte[1u] = ((crc_ccit16>>8u)&0xFFu);                   // append CRC16 hibyte
       for (index=2u; index<=6u; index++)
       {
         payload_with_tail_byte[index] = payload[index-2u];
       }
       payload_with_tail_byte[7u] = UAV_START_TRANS | (transfer_id & 31U);      // start bit only
       //payload_with_tail_byte[7u] = 0x80U | (transfer_id & 31U);              // start bit only
       CAN1Write(can_id, (unsigned char*) payload_with_tail_byte, 8u, (uint16_t) ((priority & _CAN_TX_XTD_FRAME) & _CAN_TX_NO_RTR_FRAME) );
       payload_len=payload_len - 5u;                                            // subtract the bytes that were sent
       while (payload_len >= 0u)                                                // we still have bytes to send (consider removing while and iterate the function)
       {
          if (payload_len <= 7u)                                                // single message send 7 or less bytes in payload
          {
             memcpy((void*)payload_with_tail_byte, (void*) payload, (int16_t) payload_len);
             noOfmsg=++noOfmsg % UINT8_MAX;
             tail_byte_toggle=UAV_DO_TOGGLE(noOfmsg);
             payload_with_tail_byte[7u] = ((tail_byte_toggle << 5u) | ((transfer_id & 31U) | UAV_STOP_TRANS)); // no start bit end bit and toggle bit
             //payload_with_tail_byte[payload_len+1u] = 0x60U | (transfer_id & 31U);  // no start bit end bit and toggle bit
             return(CAN1Write(can_id, (unsigned char*) payload_with_tail_byte, payload_len + 1u, (uint16_t) ((priority & _CAN_TX_XTD_FRAME) & _CAN_TX_NO_RTR_FRAME) ));  // need to make can send wrapper
          }
          else
          {
             for (index=0u; index<=7u; index++)                                 // for 7 bytes of the payload do
             {
                if (noOfmsg == 1u)                                              // this is the second message
                {
                   offset = index+5u;
                   payload_with_tail_byte[index] = payload[offset];             // we last sent 5 bytes so start after that
                }
                else
                {
                   offset = index+7u+offset;
                   payload_with_tail_byte[index] = payload[offset];             // we previously sent 7 bytes so start after that
                }
             }
             noOfmsg=++noOfmsg % UINT8_MAX;
             tail_byte_toggle=UAV_DO_TOGGLE(noOfmsg);
             payload_with_tail_byte[7u] = (tail_byte_toggle << 5u) | (transfer_id & 31U);  // toggle bit only
             //payload_with_tail_byte[7u] = 0x20U | (transfer_id & 31U);        // toggle bit only
             CAN1Write(can_id, (unsigned char*) payload_with_tail_byte, 8u, (uint16_t) ((priority & _CAN_TX_XTD_FRAME) & _CAN_TX_NO_RTR_FRAME) );
             payload_len=payload_len - 7u;                                      // subtract the payload bytes that were sent
          }
       }
    }
}

 /*-----------------------------------------------------------------------------
 *      make_float16():  convert to float16 format
 *
 *  Parameters: float32_t value
 *  Return:     uint16_t representing the float16 value (didnt bother to typecast)
 *----------------------------------------------------------------------------*/
uint16_t make_float16(float32_t value)
{
    union fp32
    {
        uint32_t u;
        float f;
    };

    const union fp32 f32infty = { 255U << 23u };
    const union fp32 f16infty = { 31U << 23u };
    const union fp32 magic = { 15U << 23u };
    const uint32_t sign_mask = 0x80000000L;
    const uint32_t round_mask = ~0xFFFU;
    uint32_t sign;
    
    union fp32 in;
    uint16_t out = 0U;

    in.f = value;

    sign = in.u & sign_mask;
    in.u ^= sign;

    if (in.u >= f32infty.u)
    {
        out = (in.u > f32infty.u) ? 0x7FFFU : 0x7C00U;
    }
    else
    {
        in.u &= round_mask;
        in.f *= magic.f;
        in.u -= round_mask;
        if (in.u > f16infty.u)
        {
            in.u = f16infty.u;
        }
        out = (uint16_t)(in.u >> 13u);
    }

    out |= (uint16_t)(sign >> 16u);

    return out;
}
/// Standard data type: uavcan.protocol.NodeStatus
/*-----------------------------------------------------------------------------
 *      publish_node_status():  publish the status of the node
 *
 *  Parameters: canAbstractNodeHealthClass health, canCurrentModeClass mode, 
 *              uint16_t vendor_specific_status_code, uint8_t uavcan_node_id, 
 *              uint8_t transfer_id
 *  Return:     int16_t status from the canwrite
 *----------------------------------------------------------------------------*/
int16_t publish_node_status(canAbstractNodeHealthClass health, canCurrentModeClass mode, uint16_t vendor_specific_status_code, uint8_t uavcan_node_id, uint8_t transfer_id)
{
    uint32_t uptime_sec = 1234L;                                              // stub
    static const uint16_t data_type_id = 341u;
    uint8_t payload[7u];

    uptime_sec = (uint32_t) readUptime();                                       // get the uptime from the RTC

    payload[0u] = (uptime_sec >> 0u)  & 0xFFu;                                  // Uptime in seconds
    payload[1u] = (uptime_sec >> 8u)  & 0xFFu;
    payload[2u] = (uptime_sec >> 16u) & 0xFFu;
    payload[3u] = (uptime_sec >> 24u) & 0xFFu;

    // Health and mode
    payload[4u] = ((uint8_t)health << 6u) | ((uint8_t)mode << 3u);

    // Vendor-specific status code
    payload[5u] = (vendor_specific_status_code >> 0u) & 0xFFu;
    payload[6u] = (vendor_specific_status_code >> 8u) & 0xFFu;
    transfer_id=++transfer_id % UINT8_MAX;
    return uavcan_broadcast(AVCAN_PRIORITY_LOW , data_type_id, transfer_id, payload, sizeof(payload),uavcan_node_id);
}
/// Standard data type: uavcan.equipment.air_data.TrueAirspeed
/*-----------------------------------------------------------------------------
 *      publish_true_airspeed():  example publish data from the node
 *
 *  Parameters: float32_t mean, float32_t variance, uint8_t uavcan_node_id, 
 *              uint8_t transfer_id
 *  Return:     int16_t status from the canwrite
 *----------------------------------------------------------------------------*/
int16_t publish_true_airspeed(float32_t mean, float32_t variance, uint8_t uavcan_node_id, uint8_t transfer_id)
{
    const uint16_t f16_mean     = make_float16(mean);
    const uint16_t f16_variance = make_float16(variance);
    static const uint16_t data_type_id  = 1020U;
    
    uint8_t payload[4u];
    payload[0u] = (f16_mean >> 0u) & 0xFFU;
    payload[1u] = (f16_mean >> 8u) & 0xFFU;
    payload[2u] = (f16_variance >> 0u) & 0xFFU;
    payload[3u] = (f16_variance >> 8u) & 0xFFU;

    transfer_id=++transfer_id % UINT8_MAX;
    return uavcan_broadcast(AVCAN_PRIORITY_MEDIUM, data_type_id, transfer_id, payload, sizeof(payload),uavcan_node_id);
}
/*-----------------------------------------------------------------------------
 *      process_ccp_message():  read a CCP can calibration request
 *
 *  Parameters: ccp_message_t *msg, uint16_t my_station_addr
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void process_ccp_message(ccp_message_t *msg, uint16_t my_station_addr)
{
     switch (msg->common.msg_type)                                              // read the message type
     {    
          case Ccp_connect:                                                     // type was a connect
          if (my_station_addr == msg->connect.station_to_connect)               // did it match our address
          {          
                //ccp_connect ();
          }       
          break;   
          
          case Ccp_disconnect:                                                  // type was a disconnect
          if (my_station_addr == msg->disconnect.station_to_disconnect)         // was it for our station address
          {          
              if (CCP_PERM_DISCONNECT == msg->disconnect.disconnect_command)
              {             
                  //ccp_disconnect ();
              }       
          }       
          break;   
          
          default:       
          break;                                                                /* ignore unknown commands */
     } 
}