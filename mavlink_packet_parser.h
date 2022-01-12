#ifndef __packet_parser_mavlink__
#define __packet_parser_mavlink__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

uint64_t getMAVUptime();
uint8_t receiveMavlinkPacketReader( const unsigned char *buf, uint16_t recsize, mavlink_message_t *msg, mavlink_status_t *status );
void FIRLowPass( int16_t *inputSamples, int16_t outputSamples[] );
void sendMavlinkSerial( uint8_t *buf, uint16_t buf_len );
void sendMavlinkUDP( uint8_t *buf, uint16_t buf_len );

/*-----------------------------------------------------------------------------
 *      getMAVUptime:  Makes the uptime global variable
 *                    (used in e.g. MAVLINK message)
 *
 *  Parameters: (none)
 *  Return:     uint64_t
 *----------------------------------------------------------------------------*/
uint64_t getMAVUptime()
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

   secondsLow = (RTCTIME & 0xF00ul)>>8u;                                        /* Grab the RTC to show the timestamp */
   secondsHigh = (RTCTIME & 0x7000ul)>>12u;
   secondsActual = (secondsHigh * 10u) + secondsLow;
   minutesLow = (RTCTIME & 0xF0000ul)>>16u;
   minutesHigh = (RTCTIME & 0x700000ul)>>20u;
   minutesActual = (minutesHigh * 10u) + minutesLow;
   hoursLow = (RTCTIME & 0xF000000ul)>>24u;
   hoursHigh = (RTCTIME & 0x30000000ul)>>28u;
   hoursActual = (hoursHigh * 10u) + hoursLow;
   return ((((hoursActual*24u)*60u) + (minutesActual * 60u)) + secondsActual);  /* seconds in day consider date if up longer than a day (battery life) */
}
/*-----------------------------------------------------------------------------
 *      receiveMavlinkPacketReader:  Reads the MAVLINK packet
 *
 *  Parameters: const unsigned char *buf, uint16_t recsize, mavlink_message_t *msg, 
 *              mavlink_status_t *status
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t receiveMavlinkPacketReader( const unsigned char *buf, uint16_t recsize, mavlink_message_t *msg, mavlink_status_t *status )
{
    uint8_t i;                                                                  /* loop counter */
    uint8_t ret=0u;

    if (buf == NULL)                                                            /* buffer passed was null then return error */
       return (ret);

    if (recsize > 0u)                                                           /* Something received - print out all bytes and parse packet */
    {
        for (i = 0u; i < recsize; ++i)                                          /* fill the message and status structure with the reply */
        {
            ret=mavlink_parse_char(MAVLINK_COMM_0, buf[i], msg, status);        /* parse MAVLINK channel 0 message from the buf (recieved buffer) to msg and status structures */
        }
        return(ret);
    }
    /* memset(buf, 0, BUFFER_LENGTH); */
}

/* Filter setup:
     Filter kind: FIR
     Filter type: Lowpass filter
     Filter order: 3
     Filter window: Blackman
     Filter borders:
       Wpass:4000 Hz  */
/*-----------------------------------------------------------------------------
 *      FIRLowPass():  low pass filter implementation
 *
 *  Parameters: int16_t *inputSamples, int16_t outputSamples[]
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void FIRLowPass( int16_t *inputSamples, int16_t outputSamples[] )
{
  const uint8_t FILTER_ORDER  = 3;

  const int16_t COEFF_B[FILTER_ORDER+1] = { 0x0000U, 0x1255U, 0x1255U, 0x0000U };
  uint8_t recordNum;
  /* int   outputSamples[6u];  */
  uint8_t numOfOutputs= (sizeof(inputSamples) / sizeof(int16_t));

  for (recordNum = 0u; recordNum < numOfOutputs; recordNum++)
  {
    outputSamples[recordNum] = FIR_Radix(COEFF_B, FILTER_ORDER+1u, inputSamples, 3u,  recordNum);
  }

  /* outputSamples[0] = COEFF_B[0]*inputSamples[0];
  // outputSamples[1] = COEFF_B[0]*inputSamples[1] + COEFF_B[1]*inputSamples[0];
  // outputSamples[2] = COEFF_B[0]*inputSamples[2] + COEFF_B[1]*inputSamples[1] + COEFF_B[2]*inputSamples[0];
  */
}
/*-----------------------------------------------------------------------------
 *      sendMavlinkSerial():  send mavlink serial message
 *
 *
 *  Parameters: uint8_t *buf, uint16_t buf_len
 *
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void sendMavlinkSerial( uint8_t *buf, uint16_t buf_len )
{
   int8_t CharPosn; 
#if defined(MAVLINK_USE_UART1)
      for(CharPosn=0;CharPosn<buf_len;CharPosn++)
      {
        UART1_write(buf[CharPosn]);                                             // Send it to serial UART1
      }
#elif defined(MAVLINK_USE_UART2)
      for(CharPosn=0;CharPosn<buf_len;CharPosn++)
      {
        UART2_write(buf[CharPosn]);                                             // Send it to serial UART2
      }
#elif defined(MAVLINK_USE_UART3)
      for(CharPosn=0;CharPosn<buf_len;CharPosn++)
      {
        UART3_write(buf[CharPosn]);                                             // Send it to serial UART3
      }
#elif defined(MAVLINK_USE_UART4)
      for(CharPosn=0;CharPosn<buf_len;CharPosn++)
      {
        UART4_write(buf[CharPosn]);                                             // Send it to serial UART4
      }
#elif defined(MAVLINK_USE_UART5)
      for(CharPosn=0;CharPosn<buf_len;CharPosn++)
      {
        UART5_write(buf[CharPosn]);                                             // Send it to serial UART5
      }
#elif defined(MAVLINK_USE_UART6)
      for(CharPosn=0;CharPosn<buf_len;CharPosn++)
      {
        UART6_write(buf[CharPosn]);                                             // Send it to serial UART6
      }
#endif
}
/*-----------------------------------------------------------------------------
 *      sendMavlinkUDP():  send mavlink UDP message
 *
 *
 *  Parameters: uint8_t *buf, uint16_t buf_len
 *
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendMavlinkUDP( uint8_t *buf, uint16_t buf_len )
{
   uint8_t udpSent=0u;
   uint8_t retMavUDP=0;

#if (!defined(SERIAL_MAVLINK) && defined(USE_MAVLINK))
    while ((retMavUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
    {
      retMavUDP = Net_Ethernet_Intern_sendUDP(MavLinkBuf.RemoteIpAddr, MavLinkBuf.From_Port, MavLinkBuf.Dest_Port, buf, buf_len);        // send the Buffer contents as UDP datagram.
      udpSent=++udpSent % UINT8_MAX;
    }
#endif
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif