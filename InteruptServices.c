//
//  Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//  Interrupt Code
//  Enabled timer5 (global counter every period = 100 ms)
//  UART2 to read serial for simpleBGC (csn be turned off using UART2_INTERUPT define)
//  UART2_interrupt();  simpleBGC gimbal protocol
//  Timer1_interrupt();  This is used to increment Net_Ethernet_Intern_userTimerSec every second
//  Timer2_3Interrupt();  extended timer ACK interrupt
//  interruptGPS();  interrupt for various gps units
//  UART4_interrupt();  XS Encoder camera protocol
//  UART5_interrupt();  Run Cam camera protocol
//  UART6_interrupt();  LwNx protocol
//  UART1_interrupt();  odrive protocol
//
#include <stdio.h>
#include <stdint.h>
#include "definitions.h"                                                        // Global defines for all
#include "Struts.h"                                                             // Common Structures
#include "gc_events.h"                                                          // Global variable and function definitions
#include "io.h"
#include "crc.h"
#include "cpu_endian_defs.h"                                                    // endian definitions

/*
#if defined(PULSE_TACHO)
#include "mmc_file_handler.h"                                                   // for tick counter function
#endif
*/

#if defined(JRT_LIDDAR_USED)
#include "jrt_lidar.h"
#endif

#if (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY))
#include "SBGC_COMMAND.h"                                                       // SimpleBGC commands
#include "SBGC_PARSER.h"
#include "SBGC_cmd_helpers.h"
#include "SBGC_adj_vars.h"
#endif

#if defined(LWNX_LIDDAR_USED)
#include "lwNx.h"                                                               // liddar from lightware
#endif

#if defined(UART1_INTERUPT)
#include "odrive.h"                                                             // odrive operations
#endif

#include "camera.h"

#if (MAV_VER == 2u)                                                             // ==== MAVLINK ver 2.0 =============
#include "mavlink2_msg_types.h"
#define MAVLINK_STX2 253u
#include "mavlink_crc.h"
#elif (MAV_VER == 1u)                                                           // ==== MAVLINK ver 1.0 =============
#include "mavlink1_msg_types.h"
#define MAVLINK_STX1 254u
#include "mavlink_crc.h"
#endif

//#define SSCANF_GSV(X, Y, Z)         { sscanf(X,"%d,%f", Y, Z); }              sscanf not allowed as parser for ascii in pic32 mikeroe C

#if (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY))
volatile uint8_t g_boardReady=false;
volatile uint16_t g_SBGCmsgCollect=0u;
#endif

unsigned char timer_tmp1 = 0u;

#if defined(BATCH_MIXER)
volatile uint8_t g_saveBatchState=0u;                                           /* batching plant last known state before interrupt occurred */
volatile uint8_t g_saveDeviceState=0u;                                          /* save of current output states before entering interrupt (shared over interrupts therefor volatile and global) */
#endif

uint8_t g_globalStartUp=0u;                                                     /* globally used register for storing state first pass around */

#if defined(PULSE_TACHO)
rotat_sens_obj_t g_TachoObj1 = { 0, 0u, 0.0f, 0.0f, 0u };                       /* define and initialise the tachometer - odometer - speed meas - flow object */
volatile uint8_t g_fromInter=1u;                                                /* global volatile which stores and describes the interrupt state machine for tanks which have 2 low levels and a common high level input */
#endif

//unsigned char          timer_tmp5_2 = 0;
#if (defined(UART2_INTERUPT) && (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)))
unsigned char gimbalStart =0u;
uint16_t typeMsgInvalid;                                                        // Set the type for the msg_id
uint16_t dataSize;                                                              // Length of the payload message used in check
uint16_t checkSumInHead;                                                        // Checksum in the message header
uint16_t checkSumInPayload;                                                     // Checksum for the message payload
uint16_t checkSumCalc;                                                          // Calculated checksum
unsigned char *ptr_buf = (unsigned char *) &UART2.Buffer;                       // Define the pointer to the container struct
uint16_t SBGCmsgCollectLast=0u;                                                 // store of states to look for persistance of state for more than the core timer
#endif

#if defined(GPS_INCLUDED) || defined(GPS_INCLUDED2)                             // Things accociated with GPS interrupts

unsigned char GPStxt[MAX_GPS_MSG_LEN];
volatile uint8_t g_GPS_ready;

/**
 * hex2int
 * take a hex string and convert it to a 32bit number (max 8 hex digits)
 * although the NMEA data is 16 bit checksum we allow for a max of 32 bit CRC
 * also take away 0* if used for 0x infront of digits (UBOX GPS)
 */
uint32_t hex2int(unsigned char *hex)
{
    uint32_t val = 0u;
    unsigned char byte;

    while (*hex)
    {
        byte = *hex++;                                                          // get current character in hex then increment
        if (!((byte == '0') || (byte == '*')))                                  // strip 0 or * from what we have if they are in the data
        {
          if (byte >= '0' && byte <= '9') byte = byte - '0';                    // transform hex character to the 4bit equivalent number, using the ascii table indexes
          else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
          else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
          val = (val << 4) | (byte & 0xF);                                      // shift 4 to make space for new digit, and add the 4 bits of the new digit
        }
    }
    return val;
}
#endif                                                                          // END OF GPS INCLUSION ------------------------------

#if defined(UART1_INTERUPT)
uint8_t packet_len;
#endif

#if (defined(UART4_INTERUPT) && (CAMERA_TYPE == XS_CAM))
unsigned char XSstart =0u;                                                      // STX found
unsigned int XStypeMsgInvalid;                                                  // Set the type for the msg_id
uint8_t XSlen;                                                                  // message length
#endif
#if (CAMERA_TYPE == XS_CAM)
volatile videoChannelStatus_t g_XSVideoChan;                                    // XS encoder video channel recording Status counter object
#endif

#if defined(SERIAL_MAVLINK)
uint16_t packetSignaLen=0u;                                                     /* collection flag for mavlink2 packets that have a signature 16 bit as can be up to 279 bytes */
#if (defined(MAVLINK_TIMESTAMP_CHK) && (MAV_VER == 2u))                         /* we chose to enable the filter on signature */
uint64_t g_mavSignature=0ULL;                                                   /* mavlink 2 signature 6 bytes long */
uint64_t g_mavTimStamp=0ULL;                                                    /* mavlink 2 time stamp 6 bytes long */
uint64_t g_mavTimStampLast=0ULL;                                                /* mavlink 2 last timestamp for checking */
uint8_t g_linkId=0U;                                                            /* mavlink2 signature link id */
#define MAV_SIG_LINK_ID 20u                                                     /* assign a link id */
#endif
uint8_t g_SysId=0u;                                                             /* system id received for mavlink */
uint8_t mv_packet_len;                                                          /* byte in message representing packet length */
#define MAV_SYS_ID 0x1FU                                                        /* define the MAVLINK system id change this you are a multistation master */
#endif

#ifdef UART6_INTERUPT
#include "lwNx.h"                                                               /* lightware liddar unit */
#endif

#ifdef  JRT_LIDDAR_USED
uint8_t usbSendLockOut=0u;                                                      /* usb send gen_write call has blocked */
#define USB_SEND_BLOCK_CNT 3u                                                   /* number of counts to denote we have blocked */
#endif

#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
static ADCStates_e prevAdcState=ENAB_ADC_MAN_SAMP;
#endif

/* =========== function defs ================================================ */
void Timer_5Interrupt();                                                        /* 100 ms timer counter */
void coreTimer();                                                               /* interrupt on tick counter reaching 80M 1 second */
void Io0_Interrupt();                                                           /* INT0 pin rising edge interrupt */

#if defined(BATCH_MIXER)
void Io1_BatchTk_Low();                                                         /* first tank gets a low level while supplying */
void Io2_BatchTk_Full();                                                        /* first tank is filled up after going low */
void Io3_BatchTk_Low();                                                         /* second tank gets a low level while supplying */
void Io4_BatchTk_Full();                                                        /* second tank is re-filled after going low */
#endif

void ChangeNotice();                                                            /* interrupt on change notification pins on PORTB */
#if defined(GPS_INCLUDED) || defined(GPS_INCLUDED2)
uint32_t hex2int(unsigned char *hex);
void interruptGPS();                                                            /* GPS */
#endif
#ifdef UART5_INTERUPT
void UART5_interrupt();                                                         /* Run Cam */
#endif
#ifdef TIMER1_NEEDED
void Timer1_interrupt();
#endif
#ifdef TIMER2_NEEDED
void Timer2_3Interrupt();
#endif
#if (defined(UART2_INTERUPT) && (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)))
void UART2_interrupt();                                                         /* SBGC */
#endif
#ifdef UART6_INTERUPT
void UART6_interrupt();                                                         /* lwnx liddar */
#endif
#if (defined(UART4_INTERUPT) && (CAMERA_TYPE == XS_CAM))
void UART4_interrupt();                                                         /* xs encoder */
#endif
#if defined(UART1_INTERUPT)
void UART1_interrupt();                                                         /* odrive */
#endif
#if defined(SERIAL_MAVLINK)
void MAV_interrupt();
#endif
#ifdef TIMER1_NEEDED                                                            // This is used to increment Net_Ethernet_Intern_userTimerSec every second to prevent lock up
/*-----------------------------------------------------------------------------
 *      Timer1_Interrupt() :  his is used to increment Net_Ethernet_Intern_userTimer
 *  every second to prevent lock up
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Timer1_interrupt() iv IVT_TIMER_1 ilevel 7 ics ICS_AUTO {
  asm EI;                                                                       // Reenable global interrupts MikroE forum suggests needed for int priority
  timer_tmp1=++timer_tmp1 % UINT8_MAX;
  if(timer_tmp1 >= 5u)                                                          // 5 times 200 ms is every second
  {
    timer_tmp1 = CLEAR;
    Net_Ethernet_Intern_userTimerSec++ % UINT32_MAX;                            // Global ethernet counter ref : https://forum.mikroe.com/viewtopic.php?f=194&t=61238
  }
  TMR1     = CLEAR;
  T1IF_bit = CLEAR;                                                             // clear interrupt flag
}
#endif

#ifdef TIMER2_NEEDED                                                            // Disabled UDP ACK check to here atm
/*-----------------------------------------------------------------------------
 *      Timer2_3Interrupt() :  Timer 2
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Timer2_3Interrupt() iv IVT_TIMER_3 ilevel 6 ics ICS_AUTO{

  asm EI;                                                                       //Reenable global interrupts
  T3IF_bit      = CLEAR;
  TMR2          = CLEAR;
  TMR3          = CLEAR;
  //LATD.B2 = ~ PORTD.B2;
  switch (SBGC_DATA.State)
  {

   case  UART2_CHECK_ACK_TIME_OUT :
         {
            T2CON = DISABLE_T2;
            SBGC_DATA.UDP_Send_Slow_ACK++ % UINT8_MAX;
            SBGC_DATA.UDP_Send_Slow_ACK_Timeout++ % UINT8_MAX;
            if(SBGC_DATA.UDP_Send_Slow_ACK_Timeout >=8u)
            {
              SBGC_DATA.State = UART2_ACK_TIME_OUT;
              SBGC_DATA.UDP_Send_Slow_ACK_Timeout=CLEAR;
            }
            break;
         }
   default:
         {
            T2CON = DISABLE_T2;
         }
  }

}
#endif

#define FLOAT64_MAX UINT64_MAX                                                  // set this to float64_t max when you can look it up
/*-----------------------------------------------------------------------------
 *      Timer_5Interrupt() :  Timer 5 runs to provide us with a tick counter which
 *  can be used for checking time e.g. debounce
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Timer_5Interrupt() iv IVT_TIMER_5 ilevel 7 ics ICS_AUTO
{
  asm EI;                                                                       //Reenable global interrupts
  g_timer5_counter=++g_timer5_counter % UINT64_MAX;                             // Global Timer counter to be used for ms elapsed
#if (CAMERA_TYPE == XS_CAM)
  if (g_XSVideoChan.reset1==1u)
  {
    g_XSVideoChan.videoRecCh1=0.0f;
    g_XSVideoChan.reset1=0u;
  }
  if (g_XSVideoChan.reset2==1u)
  {
    g_XSVideoChan.videoRecCh2=0.0f;
    g_XSVideoChan.reset2=0u;
  }
  if (g_XSVideoChan.reset3==1u)
  {
    g_XSVideoChan.videoRecCh3=0.0f;
    g_XSVideoChan.reset3=0u;
  }
  if (g_XSVideoChan.reset4==1u)
  {
    g_XSVideoChan.videoRecCh4=0.0f;
    g_XSVideoChan.reset4=0u;
  }
  if (g_XSVideoChan.start1==1u)
  {
     g_XSVideoChan.videoRecCh1=(0.1f + g_XSVideoChan.videoRecCh1);              // count seconds recording (interrupt is every 100ms)
  }
  if (g_XSVideoChan.start2==1u)
  {
     g_XSVideoChan.videoRecCh2=(0.1f + g_XSVideoChan.videoRecCh2);              // count seconds recording
  }
  if (g_XSVideoChan.start3==1u)
  {
     g_XSVideoChan.videoRecCh3=(0.1f + g_XSVideoChan.videoRecCh3);              // count seconds recording
  }
  if (g_XSVideoChan.start4==1u)
  {
     g_XSVideoChan.videoRecCh4=(0.1f + g_XSVideoChan.videoRecCh4);              // count seconds recording
  }
#endif
#if defined(JRT_LIDDAR_USED)
  if (JRTSerial.WriteState == USB_SEND_CONFIG_PACKET)                           /* write state is in Gen_Write call (check it hasnt blocked) */
  {
     usbSendLockOut=++usbSendLockOut % UINT8_MAX;
     if (usbSendLockOut >= USB_SEND_BLOCK_CNT)                                  /* timeout exceeded in send state block has occurred close it out */
     {
         USB_Break();                                                           /* unblock the program execution */
     }
  }
  ReadUSB_JRT( &distSensIT03Meas, &JRTSerial, &distSensIT03Conf );              // periodically read the USB distance sensor
#endif
  TMR5 = CLEAR;
  T5IF_bit         = 0u;                                                        // Release the timer
}

#if (defined(UART2_INTERUPT) && (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)))  // This is set if we are enabling UART2 with SimpleBGC serial
// ======================= SimpleBGC ===========================================
//
/*-----------------------------------------------------------------------------
 *      UART2_interrupt() :  This interrupts on chars that are recieved at UART2 serial port
 *  It will process these if they are supported SimpleBGC commands
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void UART2_interrupt() iv IVT_UART_2 ilevel 6 ics ICS_AUTO
{
   uint8_t loop,inner;

   asm EI;                                                                      // Reenable global interrupts
   UART2.Bytes_In=++UART2.Bytes_In % UINT16_MAX;                                // Global counter on Bytes arriving at UART2

   if(U2STA &0x0Eu | U2EIF_bit)                                                 // Has bit 1 of the U2STA in the UART2 (the buffer overrun indicator) been set ?
   {
      T2CON = DISABLE_T2;
     /*while(URXDA_U2STA_bit)
     {
      dump= U2RXREG;
     }*/
      if (OERR_U2STA_bit==1u)                                                   // Overrun error (bit 1)
      {
        OERR_U2STA_bit = 0u;
        UART2.Buffer_Overrun=++UART2.Buffer_Overrun % UINT16_MAX;               // Count overruns
      }
      else if (FERR_U2STA_bit==1u)                                              // Framing error (bit 2)
      {
        FERR_U2STA_bit = 0u;
        UART2.Framing_Error=++UART2.Framing_Error % UINT16_MAX;
      }
      else if (PERR_U2STA_bit==1u)                                              // Parity error (bit 3)
      {
        PERR_U2STA_bit = 0u;
        UART2.Parity_Error=++UART2.Parity_Error % UINT16_MAX;
      }
      UART2.Index =0u;                                                          // Set the index to zero
      U2STA &= 0xFFFFFFFDUL;                                                    // Clear the Error Status
      U2EIF_bit =CLEAR;                                                         // Clear the Error IF
      Status(U2STA_Add, UART2_BUFFER_OVERFLOW);                                 // Report the error
      U2RXIF_bit=CLEAR;                                                         // Clear the UART2 IF
   }

   if (U2RXIF_bit)
   {
      if ((gimbalStart==false) || typeMsgInvalid)                               // if we dont already have a STX start byte or we got an invalid msg_id or checksum
      {
          UART2.Buffer[0u] = U2RXREG;
          if ((UART2.Buffer[0u] == g_SBGC_Stx_Char) || ((g_boardReady==false) && ((UART2.Buffer[0u] == SBGC_CMD_START_BYTE) || (UART2.Buffer[0u] == SBGC_CMD_START_BYTE_V2))))                              // Look for the simpleBGC start transmission
          {
             //T2CON = ENABLE_T2;       // Enable Timeout
             UART2.Index=1u;
             gimbalStart =TRUE;
             typeMsgInvalid=0u;                                                 // Set the message type to valid unless we dont get a valid type
             UART2.State = UART2_BYTE_IN_BUFFER;
          }
      }
      else
      {
         while(URXDA_U2STA_bit)                                                 // if there is a char to receive
         //if(URXDA_U2STA_bit)
         {
            UART2.Buffer[UART2.Index] = U2RXREG;                                // put the char from the UART into the buffer
            switch(UART2.Index)                                                 // For the byte position in the incoming message
            {
               case 1u:                                                         // Should be the message index
               if (g_boardReady==false)                                         // if we dont know what firmware you must get a BOARD_INFO_REPLY first
               {
                 switch(UART2.Buffer[1u])                                       // what command did we get ?
                 {
                    case SBGC_CMD_BOARD_INFO:                                   // request for board firmware
                    if ((g_SBGCmsgCollect & SBGC_SER_BOARD_INFO)==1u)           /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    g_SBGC_Stx_Char = UART2.Buffer[0u];                         // confirm the start char
                    break;
                    
                    default:                                                    // Message Id was rejected until we get the board firmware to proceed
                    typeMsgInvalid=1u;
                    UART2.Index=0u;                                             // Restart message collection
                    break;
                 }
               }
               else
               {
                 switch(UART2.Buffer[1u])                                       // what command did we get ? (list is below those supported)
                 {
                    case SBGC_CMD_GET_ANGLES:                                   // command was a SBGC get Angles
                    if ((g_SBGCmsgCollect & SBGC_SER_GET_ANGLES)==1u)           /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;
                    
                    case SBGC_CMD_AUTO_PID:                                     // command was a SBGC auto pid response
                    if ((g_SBGCmsgCollect & SBGC_SER_AUTO_PID)==1u)             /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;

                    case SBGC_CMD_GET_ANGLES_EXT:                               // Command get angles ext
                    if ((g_SBGCmsgCollect & SBGC_SER_GET_ANGLES_EXT)==1u)       /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;
                    
                    case SBGC_CMD_REALTIME_DATA_3:                              // command was a SBGC realtime data.
                    if ((g_SBGCmsgCollect & SBGC_SER_REALTIME_DATA_3)==1u)       /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;
                    
                    case SBGC_CMD_REALTIME_DATA_4:
                    if ((g_SBGCmsgCollect & SBGC_SER_REALTIME_DATA_4)==1u)       /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;
                    
                    case SBGC_CMD_CONFIRM:                                      // command was a SBGC confirm set the relevant request to stop sending once confirmed
                    if ((g_SBGCmsgCollect & SBGC_SER_CONFIRM)==1u)       /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;
                    
                    case SBGC_CMD_ERROR:                                        // command was a SBGC error response
                    if ((g_SBGCmsgCollect & SBGC_SER_ERROR)==1u)       /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;
                    
                    case SBGC_CMD_SET_ADJ_VARS_VAL:                             // command was a SBGC set adj vars val of 7 in response to our get adj vars val
                    if ((g_SBGCmsgCollect & SBGC_SER_SET_ADJ_VARS_VAL)==1u)     /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;
                    
                    case SBGC_CMD_READ_PARAMS_3:                                // command was a pide auto tune or config read/write part1
                    if ((g_SBGCmsgCollect & SBGC_SER_READ_PARAMS_3)==1u)     /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;
                    
                    case SBGC_CMD_BOARD_INFO:                                   // request for board firmware
                    if ((g_SBGCmsgCollect & SBGC_SER_BOARD_INFO)==1u)           /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;
                    
                    case SBGC_CMD_SCRIPT_DEBUG:                                 // request for script running state
                    if ((g_SBGCmsgCollect & SBGC_SER_SCRIPT_DEBUG)==1u)         /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;
                    
                    case SBGC_CMD_EVENT:                                        // requested event occurred
                    if ((g_SBGCmsgCollect & SBGC_SER_EVENT)==1u)                /* the message has already been collected but not processed */
                    {
                        UART2.Index=0u;                                         /* reset the message collection */
                    }
                    else
                    {
                        UART2.Index=++UART2.Index % UINT16_MAX;                 /* collect the message */
                    }
                    break;
                    
                    default:                                                    // Message Id was not recognised
                    typeMsgInvalid=1u;
                    UART2.Index=0u;                                             // Restart message collection
                    break;
                 }
               }
               break;

               case SBGC_PAYLOAD_LENGTH_POS:                                    // CASE 2 : Should be data size
               dataSize=UART2.Buffer[SBGC_PAYLOAD_LENGTH_POS];
               UART2.Index=++UART2.Index % UINT16_MAX;
               break;

               case 3u:                                                         // Should be checksum
               checkSumInHead=UART2.Buffer[3u];
               checkSumCalc=(UART2.Buffer[2u]+UART2.Buffer[1u])%256u;           // Calculate the checksum
               if (checkSumInHead == checkSumCalc)                              // message matches calculation header is good
               {
                  UART2.Index=++UART2.Index % UINT16_MAX;                       // go to collect the paylaod
               }
               else
               {
                  typeMsgInvalid=1u;
                  UART2.Index=0u;                                               // Restart message collection
               }
               break;

               default:                                                         // Should be payload - reset if you got snother STX or payload is too long
               //if (UART2.Buffer[UART2.Index] == SBGC_CMD_START_BYTE)            // another STX was found this is invalid go back to start
               //{
               //   UART2.Buffer[0]= UART2.Buffer[UART2.Index];                   // reset to the start of the message buffer.
               //   UART2.Index=1;                                                // reset to put the reset of the message from here
               //}
               //else                                                           ---- Commented out as may be allowed in simpleBGC message
                                                                                // re-enable if required

               if (UART2.Index >= (dataSize+3u+SBGC_LEN_OF_HEADER))             // message too long - > expected
               {
                  typeMsgInvalid=1u;
                  UART2.Index=0u;                                               // Restart message collection
               }
               else if (UART2.Index >= SBGC_CMD_MAX_BYTES)                      // message too long - > max size
               {
                  typeMsgInvalid=1u;
                  UART2.Index=1u;                                               // Restart message collection
               }
               else
               {
                  UART2.Index=++UART2.Index % UINT16_MAX;                       // increment and fill the buffer with data from the payload
               }
               break;
            }
         }
      }

      if (UART2.Index == (dataSize+SBGC_LEN_OF_HEADER+1u))                      // If we got the length of the full message
      {
        //UART2.Index=++UART2.Index % UINT16_MAX;
        gimbalStart=FALSE;                                                      // Complete message collected start again
        
        if (UART2.Buffer[0u] == SBGC_CMD_START_BYTE)                            //  version with single byte checksum
        {
           checkSumInPayload=(UART2.Buffer[UART2.Index-1u] & 0xFF);             // Body Checksum
           checkSumCalc = (uint16_t) Pay_checksum((ptr_buf+SBGC_LEN_OF_HEADER),dataSize);
        }
        else if (UART2.Buffer[0u] == SBGC_CMD_START_BYTE_V2)                    // version with double byte arc CRC
        {
           checkSumInPayload=((UART2.Buffer[UART2.Index-2u] << 8u) | UART2.Buffer[UART2.Index-1u]);  // Payload CRC calculation
#if defined(_CPU_BIG_ENDIAN)
           checkSumInPayload = SWAPINT16(checkSumInPayload);
#endif
           checkSumCalc = crc16_arc_calculate(dataSize,(ptr_buf+SBGC_LEN_OF_HEADER));
        }
        else { /* for sanity */ }
        
        //checkSumCalc = Pay_checksum(ptr_buf,55);
         if (checkSumCalc == checkSumInPayload)                                 // Checksum matches what is expected then message integrity is good
         {
            switch(UART2.Buffer[1u])                                            // For each message id copy into the right data location
            {
               case SBGC_CMD_GET_ANGLES:                                        // command was a SBGC get Angles
               if ((g_SBGCmsgCollect & SBGC_SER_GET_ANGLES)!=1u)
               {
                  memcpy(&getangles, &UART2.Buffer[SBGC_LEN_OF_HEADER], sizeof(getangles));        // Copy the payload to the get angles readback message container.
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_GET_ANGLES;
               }
#if defined(_CPU_BIG_ENDIAN)
               getangles.imu_angle_roll = SWAPINT16(getangles.imu_angle_roll);
               getangles.target_angle_roll = SWAPINT16(getangles.target_angle_roll);
               getangles.target_speed_roll = SWAPINT16(getangles.target_speed_roll);
               getangles.imu_angle_pitch = SWAPINT16(getangles.imu_angle_pitch);
               getangles.target_angle_pitch = SWAPINT16(getangles.target_angle_pitch);
               getangles.target_speed_pitch = SWAPINT16(getangles.target_speed_pitch);
               getangles.imu_angle_yaw = SWAPINT16(getangles.imu_angle_yaw);
               getangles.target_angle_yaw= SWAPINT16(getangles.target_angle_yaw);
               getangles.target_speed_yaw = SWAPINT16(getangles.target_speed_yaw);
#endif
               break;
               case SBGC_CMD_AUTO_PID:                                          // command was a SBGC auto pid response
               if ((g_SBGCmsgCollect & SBGC_SER_AUTO_PID)!=1u)
               {
                  memcpy(&pidreadbk, &UART2.Buffer[SBGC_LEN_OF_HEADER], sizeof(pidreadbk));        // Copy the payload to the pid readback message container.
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_AUTO_PID;
               }
#if defined(_CPU_BIG_ENDIAN)
               pidreadbk.lpf_roll = SWAPINT16(pidreadbk.lpf_roll);
               pidreadbk.lpf_pitch = SWAPINT16(pidreadbk.lpf_pitch);
               pidreadbk.lpf_yaw = SWAPINT16(pidreadbk.lpf_yaw);
               pidreadbk.iteration_count = SWAPINT16(pidreadbk.iteration_count);
               pidreadbk.tracking_error_roll = SWAPINT32(pidreadbk.tracking_error_roll);
               pidreadbk.tracking_error_pitch = SWAPINT32(pidreadbk.tracking_error_pitch);
               pidreadbk.tracking_error_yaw = SWAPINT32(pidreadbk.tracking_error_yaw);
#endif
               break;
               case SBGC_CMD_GET_ANGLES_EXT:                                    // Command get angles ext
               if ((g_SBGCmsgCollect & SBGC_SER_GET_ANGLES_EXT)!=1u)
               {
                   memcpy(&getanglesext, &UART2.Buffer[SBGC_LEN_OF_HEADER], sizeof(getanglesext));  // Copy the payload to the get ext angles readback message container.
                   g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_GET_ANGLES_EXT;
               }
#if defined(_CPU_BIG_ENDIAN)
               getanglesext.imu_angle_roll = SWAPINT16(getangles.imu_angle_roll);
               getanglesext.target_angle_roll = SWAPINT16(getanglesext.target_angle_roll);
               getanglesext.stator_rotor_angle_roll = SWAPINT16(getanglesext.stator_rotor_angle_roll);
               getanglesext.imu_angle_pitch = SWAPINT16(getanglesext.imu_angle_pitch);
               getanglesext.target_angle_pitch = SWAPINT16(getanglesext.target_angle_pitch);
               getanglesext.stator_rotor_angle_pitch = SWAPINT16(getanglesext.stator_rotor_angle_pitch);
               getanglesext.imu_angle_yaw = SWAPINT16(getanglesext.imu_angle_yaw);
               getanglesext.target_angle_yaw= SWAPINT16(getanglesext.target_angle_yaw);
               getanglesext.stator_rotor_angle_yaw = SWAPINT16(getanglesext.stator_rotor_angle_yaw);
#endif
               break;
               case SBGC_CMD_REALTIME_DATA_3:                                   // command was a SBGC realtime data.
               if ((g_SBGCmsgCollect & SBGC_SER_REALTIME_DATA_3)!=1u)
               {
                  memcpy(&realdata3, &UART2.Buffer[SBGC_LEN_OF_HEADER], sizeof(realdata3));        // Copy the payload to the realtime data 3 readback message container.
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_REALTIME_DATA_3;
               }
#if defined(_CPU_BIG_ENDIAN)
               realdata3.acc_data_roll = SWAPINT16(realdata3.acc_data_roll);
               realdata3.gyro_data_roll = SWAPINT16(realdata3.gyro_data_roll);
               realdata3.acc_data_pitch = SWAPINT16(realdata3.acc_data_pitch);
               realdata3.gyro_data_pitch = SWAPINT16(realdata3.gyro_data_pitch);
               realdata3.acc_data_yaw = SWAPINT16(realdata3.acc_data_yaw);
               realdata3.gyro_data_yaw = SWAPINT16(realdata3.gyro_data_yaw);
               realdata3.serial_error_cnt = SWAPINT16(realdata3.serial_error_cnt);
               realdata3.system_error = SWAPINT16(realdata3.system_error);
               realdata3.system_sub_error = SWAPINT16(realdata3.system_sub_error );
               realdata3.rc_roll = SWAPINT16(realdata3.rc_roll);
               realdata3.rc_pitch = SWAPINT16(realdata3.rc_pitch);
               realdata3.rc_yaw = SWAPINT16(realdata3.rc_yaw);
               realdata3.rc_cmd = SWAPINT16(realdata3.rc_cmd);
               realdata3.ext_fc_roll = SWAPINT16(realdata4.ext_fc_roll);
               realdata3.ext_fc_pitch = SWAPINT16(realdata4.ext_fc_pitch);
               realdata3.imu_angle_roll = SWAPINT16(realdata4.imu_angle_roll);
               realdata3.imu_angle_pitch = SWAPINT16(realdata4.imu_angle_pitch);
               realdata3.imu_angle_yaw = SWAPINT16(realdata3.imu_angle_yaw);
               realdata3.frame_imu_angle_roll = SWAPINT16(realdata3.frame_imu_angle_roll);
               realdata3.frame_imu_angle_pitch = SWAPINT16(realdata3.frame_imu_angle_pitch);
               realdata3.frame_imu_angle_yaw = SWAPINT16(realdata3.frame_imu_angle_yaw);
               realdata3.target_angle_roll = SWAPINT16(realdata3.target_angle_roll);
               realdata3.target_angle_pitch = SWAPINT16(realdata3.target_angle_pitch);
               realdata3.target_angle_yaw = SWAPINT16(realdata3.target_angle_yaw);
               realdata3.cycle_time = SWAPINT16(realdata3.cycle_time);
               realdata3.i2c_error_count = SWAPINT16(realdata3.i2c_error_count);
               realdata3.bat_level = SWAPINT16(realdata3.bat_level);
#endif
               break;
               case SBGC_CMD_REALTIME_DATA_4:
               if ((g_SBGCmsgCollect & SBGC_SER_REALTIME_DATA_4)!=1u)
               {
                  memcpy(&realdata4, &UART2.Buffer[SBGC_LEN_OF_HEADER], sizeof(realdata4));        // Copy the payload to the realtime data 4 readback message container.
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_REALTIME_DATA_4;
               }
#if defined(_CPU_BIG_ENDIAN)
               realdata4.acc_data_roll = SWAPINT16(realdata4.acc_data_roll);
               realdata4.gyro_data_roll = SWAPINT16(realdata4.gyro_data_roll);
               realdata4.acc_data_pitch = SWAPINT16(realdata4.acc_data_pitch);
               realdata4.gyro_data_pitch = SWAPINT16(realdata4.gyro_data_pitch);
               realdata4.acc_data_yaw = SWAPINT16(realdata4.acc_data_yaw);
               realdata4.gyro_data_yaw = SWAPINT16(realdata4.gyro_data_yaw);
               realdata4.serial_error_cnt = SWAPINT16(realdata4.serial_error_cnt);
               realdata4.system_error = SWAPINT16(realdata4.system_error);
               realdata4.system_sub_error = SWAPINT16(realdata4.system_sub_error );
               realdata4.rc_roll = SWAPINT16(realdata4.rc_roll);
               realdata4.rc_pitch = SWAPINT16(realdata4.rc_pitch);
               realdata4.rc_yaw = SWAPINT16(realdata4.rc_yaw);
               realdata4.rc_cmd = SWAPINT16(realdata4.rc_cmd);
               realdata4.ext_fc_roll = SWAPINT16(realdata4.ext_fc_roll);
               realdata4.ext_fc_pitch = SWAPINT16(realdata4.ext_fc_pitch);
               realdata4.imu_angle_roll = SWAPINT16(realdata4.imu_angle_roll);
               realdata4.imu_angle_pitch = SWAPINT16(realdata4.imu_angle_pitch);
               realdata4.imu_angle_yaw = SWAPINT16(realdata4.imu_angle_yaw);
               realdata4.frame_imu_angle_roll = SWAPINT16(realdata4.frame_imu_angle_roll);
               realdata4.frame_imu_angle_pitch = SWAPINT16(realdata4.frame_imu_angle_pitch);
               realdata4.frame_imu_angle_yaw = SWAPINT16(realdata4.frame_imu_angle_yaw);
               realdata4.target_angle_roll = SWAPINT16(realdata4.target_angle_roll);
               realdata4.target_angle_pitch = SWAPINT16(realdata4.target_angle_pitch);
               realdata4.target_angle_yaw = SWAPINT16(realdata4.target_angle_yaw);
               realdata4.cycle_time = SWAPINT16(realdata4.cycle_time);
               realdata4.i2c_error_count = SWAPINT16(realdata4.i2c_error_count);
               realdata4.bat_level = SWAPINT16(realdata4.bat_level);
               realdata4.rotor_angle_roll = SWAPINT16(realdata4.rotor_angle_roll);
               realdata4.rotor_angle_pitch = SWAPINT16(realdata4.rotor_angle_pitch);
               realdata4.rotor_angle_yaw = SWAPINT16(realdata4.rotor_angle_yaw);
               realdata4.balance_error_roll = SWAPINT16(realdata4.balance_error_roll);
               realdata4.balance_error_pitch = SWAPINT16(realdata4.balance_error_pitch);
               realdata4.balance_error_yaw = SWAPINT16(realdata4.balance_error_yaw);
               realdata4.current = SWAPINT16(realdata4.current);
               realdata4.mag_data_roll = SWAPINT16(realdata4.mag_data_roll);
               realdata4.mag_data_pitch = SWAPINT16(realdata4.mag_data_pitch);
               realdata4.mag_data_yaw = SWAPINT16(realdata4.mag_data_yaw);
               realdata4.imu_temp = SWAPINT16(realdata4.imu_temp);
               realdata4.frame_imu_temp = SWAPINT16(realdata4.frame_imu_temp);
               realdata4.imu_g_error = SWAPINT16(realdata4.imu_g_error);
               realdata4.imu_h_error = SWAPINT16(realdata4.imu_h_error);
               realdata4.motor_out_roll = SWAPINT16(realdata4.motor_out_roll);
               realdata4.motor_out_pitch = SWAPINT16(realdata4.motor_out_pitch);
               realdata4.motor_out_yaw = SWAPINT16(realdata4.motor_out_yaw);
#endif
               break;
               case SBGC_CMD_CONFIRM:                                           // command was a SBGC confirm
               if ((g_SBGCmsgCollect & SBGC_SER_CONFIRM)!=1u)
               {
                  memcpy(&cmdconf, &UART2.Buffer[SBGC_LEN_OF_HEADER], sizeof(cmdconf));            // Copy the payload to the command confirmation readback message container.
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_CONFIRM;
               }
#if defined(_CPU_BIG_ENDIAN)
               cmdconf.reply = SWAPINT16(cmdconf.reply);
#endif
               break;
               case SBGC_CMD_ERROR:                                             // command was a SBGC error response
               if ((g_SBGCmsgCollect & SBGC_SER_ERROR)!=1u)
               {
                  memcpy(&cmderror, &UART2.Buffer[SBGC_LEN_OF_HEADER], sizeof(cmderror));          // Copy the payload to the command confirmation readback message container.
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_ERROR;
               }
#if defined(_CPU_BIG_ENDIAN)
               cmderror.reply = SWAPINT32(cmderror.reply);
#endif
               break;
               case SBGC_CMD_SET_ADJ_VARS_VAL:                                  // command was a SBGC set adj vars val of 7 in response to our get adj vars val
               if ((g_SBGCmsgCollect & SBGC_SER_SET_ADJ_VARS_VAL)!=1u)
               {
                  memcpy(&getvar_rd, &UART2.Buffer[0u], sizeof(getvar_rd));        // Copy the payload to the command confirmation readback message container.
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_SET_ADJ_VARS_VAL;
               }
#if defined(_CPU_BIG_ENDIAN)
               getvar_rd.param1_value = SWAPINT32(getvar_rd.param1_value);
               getvar_rd.param2_value = SWAPINT32(getvar_rd.param2_value);
               getvar_rd.param3_value = SWAPINT32(getvar_rd.param3_value);
               getvar_rd.param4_value = SWAPINT32(getvar_rd.param4_value);
               getvar_rd.param5_value = SWAPINT32(getvar_rd.param5_value);
               getvar_rd.param6_value = SWAPINT32(getvar_rd.param6_value);
               getvar_rd.param7_value = SWAPINT32(getvar_rd.param7_value);
#endif
               break;
               case SBGC_CMD_BOARD_INFO:                                        // command was request for information on the firmware/state ofthe board
               if ((g_SBGCmsgCollect & SBGC_SER_BOARD_INFO)!=1u)
               {
                  memcpy(&boardinforb, &UART2.Buffer[SBGC_LEN_OF_HEADER], sizeof(boardinforb));
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_BOARD_INFO;
               }
#if defined(_CPU_BIG_ENDIAN)
               boardinforb.FIRMWARE_VER = SWAPINT16(boardinforb.FIRMWARE_VER);
               boardinforb.BOARD_FEATURES = SWAPINT16(boardinforb.BOARD_FEATURES);
               boardinforb.FRW_EXTRA_ID = SWAPINT32(boardinforb.FRW_EXTRA_ID);
#endif
               break;
               case SBGC_CMD_READ_PARAMS_3:                                     // command was a pide auto tune or config read/write part1
               if ((g_SBGCmsgCollect & SBGC_SER_READ_PARAMS_3)!=1u)
               {
                  memcpy(&readparam3, &UART2.Buffer[SBGC_LEN_OF_HEADER], sizeof(readparam3));
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_READ_PARAMS_3;
               }
#if defined(_CPU_BIG_ENDIAN)
               for (loop=0u;loop<3u;loop++)
               {
                   readparam3.RC_MIN_ANGLE[loop] = SWAPINT16(readparam3.RC_MIN_ANGLE[loop]);
                   readparam3.RC_MAX_ANGLE[loop] = SWAPINT16(readparam3.RC_MAX_ANGLE[loop]);
                   readparam3.RC_MEMORY[loop] = SWAPINT16(readparam3.RC_MEMORY[loop]);
               }
               readparam3.BAT_THRESHOLD_ALARM = SWAPINT16(readparam3.BAT_THRESHOLD_ALARM);
               readparam3.BAT_THRESHOLD_MOTORS = SWAPINT16(readparam3.BAT_THRESHOLD_MOTORS);
               readparam3.BAT_COMP_REF = SWAPINT16(readparam3.BAT_COMP_REF);
               readparam3.GENERAL_FLAGS1 = SWAPINT16(readparam3.GENERAL_FLAGS1);
               readparam3.PROFILE_FLAGS1 = SWAPINT16(readparam3.PROFILE_FLAGS1);
#endif
               break;
               case SBGC_CMD_RESET:                                             // Reset issued feedback response
               break;
               case SBGC_CMD_I2C_READ_REG_BUF:                                  // i2c read either EEPROM or IMU
               break;
               case SBGC_CMD_READ_EXTERNAL_DATA:
               break;
               case SBGC_CMD_SCRIPT_DEBUG:                                      // start / stop script feedback
               if ((g_SBGCmsgCollect & SBGC_SER_SCRIPT_DEBUG)!=1u)
               {
                  memcpy(&scriptstate, &UART2.Buffer[SBGC_LEN_OF_HEADER], sizeof(scriptstate));
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_SCRIPT_DEBUG;
               }
#if defined(_CPU_BIG_ENDIAN)
               scriptstate.CMD_COUNT = SWAPINT16(scriptstate.CMD_COUNT);
#endif
               break;
               case SBGC_CMD_EVENT:                                             // motor state event
               if ((g_SBGCmsgCollect & SBGC_SER_EVENT)!=1u)
               {
                  memcpy(&eventCmd, &UART2.Buffer[SBGC_LEN_OF_HEADER], sizeof(eventCmd));          // copy the data to the object container
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_EVENT;
               }
               break;
               case SBGC_CMD_REALTIME_DATA_CUSTOM:                              // realtime data custom frame was sent
               if ((g_SBGCmsgCollect & SBGC_SER_REALTIME_DATA_CUSTOM)!=1u)
               {
                  memcpy(&realtimedatacust, &UART2.Buffer[0u], sizeof(realtimedatacust));
                  g_SBGCmsgCollect=g_SBGCmsgCollect | SBGC_SER_REALTIME_DATA_CUSTOM;
               }
#if defined(_CPU_BIG_ENDIAN)
               realtimedatacust.timestamp_ms = SWAPINT16(realtimedatacust.timestamp_ms);
               for (loop=0u;loop<3u;loop++)
               {
                  realtimedatacust.imu_angles[loop] = SWAPINT16(realtimedatacust.imu_angles[loop]);
                  realtimedatacust.target_angles[loop] = SWAPINT16(realtimedatacust.target_angles[loop]);
                  realtimedatacust.target_speed[loop] = SWAPINT16(realtimedatacust.target_speed[loop]);
                  realtimedatacust.stator_rotor_angles[loop] = SWAPINT16(realtimedatacust.stator_rotor_angles[loop]);
                  realtimedatacust.gyro_data[loop] = SWAPINT16(realtimedatacust.gyro_data[loop]);
                  realtimedatacust.rc_data[loop] = SWAPINT16(realtimedatacust.rc_data[loop]);
                  realtimedatacust.rc_data[4u+loop] = SWAPINT16(realtimedatacust.rc_data[4u+loop]);
                  realtimedatacust.z_vector[loop] = SWAPINT64(realtimedatacust.z_vector[loop]);
                  realtimedatacust.h_vector[loop] = SWAPINT64(realtimedatacust.h_vector[loop]);
                  realtimedatacust.acc_data[loop] = SWAPINT16(realtimedatacust.acc_data[loop]);
                  realtimedatacust.motor4_control[loop] = SWAPINT16(realtimedatacust.motor4_control[loop]);
                  realtimedatacust.motor4_control[loop+4u] = SWAPINT16(realtimedatacust.motor4_control[loop+4u]);
                  realtimedatacust.imu_angles_rad[loop] = SWAPINT16(realtimedatacust.imu_angles_rad[loop]);
               }
               for (loop=0u;loop<=18u;loop++)
               {
                  realtimedatacust.rc_channels[loop] = SWAPINT16(realtimedatacust.rc_channels[loop]);
               }
               for (loop=0u;loop<=26u;loop++)
               {
                  realtimedatacust.ahs_debug_info[loop] = SWAPINT16(realtimedatacust.ahs_debug_info[loop]);
               }
               for (loop=0u;loop<=26u;loop++)
               {
                  for (inner=0u;inner<=3u;inner++)
                  {
                     realtimedatacust.encoder_raw[loop][inner] = SWAPINT16(realtimedatacust.encoder_raw[loop][inner]);
                  }
               }

#endif
               default:                                                         // Message Id was not recognised
               typeMsgInvalid=1u;
               UART2.Index=1u;                                                  // Restart message collection
               break;
            }
            if (!typeMsgInvalid)                                                // Message is good and supported
            {
               UART2.State = UART2_PACKET_IN_BUFFER;                            // Declare good packet to top level
               UART2.Msg_id_recv = UART2.Buffer[1u];                            // Put the message id into the field
            }
      }
    }
  }  // End U2RXIFBIT
  U2RXIF_bit=CLEAR;

} // End Interrupt
#endif
#ifdef UART5_INTERUPT
void UART5_interrupt() iv IVT_UART_5 ilevel 7 ics ICS_AUTO 
{
   asm EI;                                                                      // Reenable global interrupts
   UART5.Bytes_In=++UART5.Bytes_In % UINT16_MAX;                                // Global counter on Bytes arriving at UART5

   if(U5STA &0x0Eu | U5EIF_bit)                                                 // Has bit 1 of the U5STA in the UART5 (the buffer overrun indicator) been set ?
   {
      /*                                                                        T5CON = DISABLE_T5; */
     /*while(URXDA_U5STA_bit)
     {
      dump= U5RXREG;
     }*/
      if (OERR_U5STA_bit==1u)                                                   /* Overrun error (bit 1) */
      {
        OERR_U5STA_bit = 0u;
        UART5.Buffer_Overrun=++UART5.Buffer_Overrun % UINT16_MAX;               /* Count overruns */
      }
      else if (FERR_U5STA_bit==1u)                                              /* Framing error (bit 2) */
      {
        FERR_U5STA_bit = 0u;
        UART5.Framing_Error=++UART5.Framing_Error % UINT16_MAX;                 /* Count framing errors */
      }
      else if (PERR_U5STA_bit==1u)                                              /* Parity error (bit 3) */
      {
        PERR_U5STA_bit = 0u;
        UART5.Parity_Error=++UART5.Parity_Error % UINT16_MAX;                   /* Count parity errors */
      }

      UART5.Index =0u;                                                          // Set the index to zero
      U5STA &= 0xFFFFFFFDUL;                                                    // Clear the Error Status
      U5EIF_bit =CLEAR;                                                         // Clear the Error IF
      //Status(U5STA_Add, UART5_BUFFER_OVERFLOW);                                 // Report the error
      U5RXIF_bit=CLEAR;                                                         // Clear the UART5 IF
   }

   if (UART5.State==UART5_BUFFER_EMPTY)                                         // new collection
   {
      UART5.Index=0u;                                                           // start at the first characture
   }
   
   if ((U5RXIF_bit) && !(UART5.State == UART5_PACKET_IN_BUFFER))                // char came into UART5
   {
      while(URXDA_U5STA_bit)                                                    // if there is a char to receive
      {
         UART5.Buffer[UART5.Index] = U5RXREG;                                   // put the char from the UART into the buffer
         UART5.State=UART5_BYTE_IN_BUFFER;
         if (UART5.Index >= RC_CMD_MAX_BYTES1 )                                 // message too long - > expected
         {
            UART5.State==UART5_BUFFER_EMPTY;
            UART5.Index=0u;                                                     // Restart message collection
         }
         else if (UART5.Buffer[UART5.Index] == RC_STX_CHAR)                     /* saw the stx again then assume it was a partial then a new one */
         {
             UART5.Index=1u;
         }
         else
         {
            UART5.Index=++UART5.Index % UINT16_MAX;                             // increment and fill the buffer with data from the payload
         }
      }
      UART5.State = UART5_PACKET_IN_BUFFER;                                     // set flag to say we collected the packet if its impartisl the caller (main) should set back to UART5_BYTE_IN_BUFFER to continue collection
  }  // End U5RXIFBIT


  U5RXIF_bit=CLEAR;                                                             // clear the interrupt flag
}
#endif
/*-----------------------------------------------------------------------------
 *      interruptGPS() :  This interrupts on chars that are recieved at UART3 serial port
 *  It will process these if they are supported Nmea or GPS commands
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
#ifdef GPS_INCLUDED1                                                            // also ensure you init the uart3 interrupt
void interruptGPS() iv IVT_UART_3 ilevel 3 ics ICS_AUTO
{
  uint8_t i;
  asm EI;                                                                       //Reenable global interrupts
  GPStxt[i] = UART3_Read();                                                     // This was from the ducati GPS example
  i=++i % UINT8_MAX;
  if (i == MAX_GPS_MSG_LEN)
  {
    i = 0u;
    g_GPS_ready = 1;
  }
  U3RXIF_bit=CLEAR;                                                             // Clear interrupt as it was read
}
#endif                                                                          // END OF GPS INCLUSION -----------------------------------
#ifdef GPS_INCLUDED2                                                            // also ensure you init the uart3 interrupt
void interruptGPS() iv IVT_UART_3 ilevel 3 ics ICS_AUTO
{
  uint8_t i=0U;                                                                 // counter for characture number in buffer
  uint8_t msgIdx=0U;                                                            // message index
  unsigned char msgId[4u];                                                      // characture holding the message id
  uint8_t msgIdPUBX=0U;                                                         // $PUBX message Id number (defines the rest of the fields)
  uint32_t timeGPSPUBX;                                                         // GPS time
  unsigned char timeGPS[15u];                                                   // characture holding the ascii time from the PUBX string message
  float32_t latGPSPUBX=0U;                                                      // latatitude
  unsigned char latGPS[15u];
  unsigned char nsGPS[2u];                                                      // north / south direction
  float32_t longGPSPUBX=0U;                                                     // longditiude
  unsigned char longGPS[15u];
  unsigned char ewGPS[2u];                                                      // east west direction
  float32_t altRefGPSPUBX;                                                      // altitude
  unsigned char altRefGPS[10u];

  float32_t hAccGPSPUBX;
  unsigned char hAccGPS[15u];                                                   // horizontal accuracy
  float32_t vAccGPSPUBX;
  unsigned char vAccGPS[15u];                                                   // vertical accuracy

  float32_t SOGGPSPUBX;                                                         // speed over ground knots
  unsigned char SOGGPS[15u];
  float32_t SOGKGPSPUBX;                                                        // speed over ground kmh
  unsigned char SOGKGPS[15u];
  float32_t COGGPSPUBX;                                                         // course over ground
  unsigned char COGGPS[15u];
  float32_t vVelGPSPUBX;                                                        // vertical velocity
  unsigned char  vVelGPS[15u];
  float32_t diffAgeGPSPUBX;                                                     // age of differential corrections
  unsigned char  diffAgeGPS[15u];
  float32_t HDOPGPSPUBX;                                                        // horizontal dilution of precision
   unsigned char HDOPGPS[15u];
  float32_t VDOPGPSPUBX;                                                        // vertical dilution of precision
  unsigned char  VDOPGPS[15u];
  float32_t TDOPGPSPUBX;                                                        // time of dilution of precision
  unsigned char  TDOPGPS[15u];
  float32_t numSvsGPSPUBX;                                                      // number of satelites being used in this solution
  unsigned char  numSvsGPS[15u];
  float32_t DRGPSPUBX;                                                          // DR used (dead reckoning) compensation from gyro etc when satelite lost
  unsigned char DRGPS[15u];
  uint16_t csGPSPUBX;                                                           // checksum
  unsigned char csGPS[15u];

  unsigned char reservedGPS[5u];                                                // reserved currently set 0

  unsigned char rmcStat[1u];                                                    // single char reads of Status or ununsed
  unsigned char dateStr1[12u];
  unsigned char magVarStr[15u];
  unsigned char magVarEw[1u];
  unsigned char posMode[1u];
  unsigned char navStatus[1u];
  unsigned char charRead[1u];

  unsigned char numSatel[3u];                                                   // string for number of satelites
  uint8_t numOfSatel=0U;                                                        // number of satelites being used for solution
  unsigned char numSatelinSight[3u];                                            // string for satelites in sight
  unsigned char elevSatel[3u];                                                  // string for satelite elevation
  unsigned char azimuthSatel[4u];                                               // string for satelite azimuth
  unsigned char sigStrengthSatel[3u];                                           // string for satelite signal strength
  unsigned char signalID[3u];                                                   // string for signal id
  unsigned char satNumber[3u];                                                  // string containing satelite number

  uint8_t charIdx=0U;                                                           // character index

  uint16_t checkCalc;                                                           // online checksum calculation

  // cant use sscanf ==== unsigned char gnsBuffer[MAX_GPS_MSG_LEN];             // buffer for full GNS type message  (demonstrates different method)
  unsigned char seaLevelAlt[20u];
  unsigned char geoHt[20u];
  float32_t SealevelAltitude;                                                   // Sea level altitude meters
  float32_t GeoidalHt;                                                          // Geoidal height

   asm EI;                                                                      // Reenable global interrupts
   if((U3STA & 0x0E) | U3EIF_bit)                                               // Error check with UART3 buffer ???
   {
      if (OERR_U3STA_bit==1u)                                                   // Overrun error (bit 1)
        OERR_U3STA_bit = 0u;
      if (FERR_U3STA_bit==1u)                                                   // Framing error (bit 2)
        FERR_U3STA_bit = 0u;
      if (PERR_U3STA_bit==1u)                                                   // Parity error (bit 3)
        PERR_U3STA_bit = 0u;

      U3STA &= 0xFFFFFFF1ul;                                                    //Clear the Error Status 0001 no longer 1101 (D) just the overrun
      U3EIF_bit =CLEAR;                                                         //Clear the Error IF
      U3RXIF_bit=CLEAR;                                                         //Clear the UART3 IF
   }
   
  if (((i <= MAX_GPS_MSG_LEN) && (msgIdx!=255u)) && (U3RXIF_bit))               // Cant continue reading beyond length of string if we errored just look for $ again
  {
     if (UART3_Data_Ready())                                                    // Ready the receieve ?
     {
        GPStxt[i] = UART3_Read();                                               // Read the char from UART 3
        switch (GPStxt[i])
        {
//================= STX ========================================================
        case '$':                                                               // Decimal 36 is a $ (STX for the PUBX message
          msgIdx=1u;                                                            // start of message found
          checkCalc=0u;                                                         // reset checksum calculation
          g_GPS_ready=0u;                                                       // inform top level new collection taking place
          break;
//================ Message Mnemonics ===========================================
        case 'P':                                                               // next char found  dec 80  for PUBX
          if (msgIdx==1u)
           msgIdx=2u;
          break;
        case 'U':                                                               // next char found  dec 85  for PUBX
          if (msgIdx==2u)
           msgIdx=3u;
         break;
        case 'B':                                                               // next char found   dec 66  for PUBX
          if (msgIdx==3u)
           msgIdx=4u;
        break;
        case 'X':                                                               // next char found  dec 88  for PUBX
          if (msgIdx==4u)
           msgIdx=5u;                                                           // ===== START PUBX ================
        break;
        case 'R':                                                               // we are looking for $xxRMC Recommended Minimum data
          if (msgIdx<=3u)                                                       // allow for finding a $PURMC should be $GPRMC
           msgIdx=40u;
        break;
        case 'M':
          if (msgIdx==40u)
           msgIdx=41u;
         break;
        case 'C':
          if (msgIdx==41u)
           msgIdx=42u;                                                          // start you're ===RMC=== parsing from step 42 (Recommended Minimum data) also furano
          break;
        case 'G':                                                               // we are looking for $xxGNS Recommended Minimum data
          if (msgIdx<=3u)                                                       // allow for finding a $PUGNS should be $GPRMC
            msgIdx=60u;
          if (msgIdx==81u)
            msgIdx=82u;                                                         // start you're ===VTG=== parsing from step 82 Course over ground and Ground speed  also furano
        break;
        case 'N':
          if (msgIdx==60u)
            msgIdx=61u;
        break;
        case 'S':
          if (msgIdx==60u)
            msgIdx=100u;
          if (msgIdx==61u)
            msgIdx=62u;                                                         // start you're ===GNS=== parsing from step 62 GNSS fix data also furano
        break;
        case 'V':                                                               // we are looking for $xxVTG Course over ground and Ground speed
          if (msgIdx<=3u)                                                       // allow for finding a $PUVTG should be $GPVTG
            msgIdx=80u;
          if (msgIdx<=100u)                                                     // saw ===$GSV=== also on furano
            msgIdx=101u;
        break;
        case 'T':
          if (msgIdx==80u)
            msgIdx=81u;
        break;
        case 'L':
          if (msgIdx==60u)
          {
             msgIdx=130u;
          }
          else if (msgIdx==130u)
          {
             msgIdx=131u;                                                       // start parsing ===GLL=== message Geographic Position Latitude/Longitude also for furano GPS
          }
        break;
//=================== * star (get checksum) ======================================
        case '*':                                                               // The checksum looks like it doesnt always get preceded by a comma so separate here
          if (msgIdx==72u)                                                      // =====GNS======= message
          {                                                                     // field 13 (skipped 2 blank fields) of the GNS message Navigation status
             navStatGPS[3u]='\0';                                               // terminate the string after 2 chars
             if (strcmp("NF",navStatGPS)==0u)                                   // navigation Status reported No Fix
             {
                strcpy(navStatGPS,"No Fix found\n");
                msgIdx=0u;                                                      // reset we got a failed message
             }
             else if (strcmp("DR",navStatGPS)==0u)                              // navigation Status reported dead reckoning only solution
             {
                strcpy(navStatGPS,"Dead reckoning only solution\n");
             }
             else if (strcmp("G2",navStatGPS)==0u)                              // navigation Status reported Stand alone 2D solution
             {
                strcpy(navStatGPS,"Stand alone 2D solution\n");
             }
             else if (strcmp("G3",navStatGPS)==0u)                              // navigation Status reported Stand alone 3D solution
             {
                strcpy(navStatGPS,"Stand alone 3D solution\n");
             }
             else if (strcmp("D2",navStatGPS)==0u)                              // navigation Status reported Differential 2D solution
             {
                strcpy(navStatGPS,"Differential 2D solution\n");
             }
             else if (strcmp("D3",navStatGPS)==0u)                              // navigation Status reported Differential 3D solution
             {
                strcpy(navStatGPS,"Differential 3D solution\n");
             }
             else if (strcmp("RK",navStatGPS)==0u)                              // navigation Status reported  Combined GPS + dead reckoning solution
             {
                strcpy(navStatGPS,"Combined GPS + dead reckoning solution \n");
             }
             else if (strcmp("TT",navStatGPS)==0u)                              // navigation Status reported  Time only solution
             {
                strcpy(navStatGPS," Time only solution\n");
             }
          }
          else if ((msgIdx==25u) && (msgIdPUBX==0u))                            // should be 21th comma and its a position message (dont think we get here we get a * rather than comma)
          {
             DRGPSPUBX=atof(DRGPS);                                             // convert the DR field from the message
          }
          else if (msgIdx==137u)                                                // last =====GLL====== field
          {
             switch(posMode[1u])
            {
                     case 'A':
                       strcpy(positionMode,"GNSS fix\n");
                     break;
                     case 'D':
                        strcpy(positionMode,"Differential GNSS fix\n");
                     break;
                     case 'N':
                        strcpy(positionMode,"No position fix\n");
                        msgIdx=0;                                               // reset we got a failed message
                     break;
                     default:
                       strcpy(positionMode,"Unknown fix\n");
                       msgIdx=0;                                                // reset we got a failed message
                     break;
            }
          }
          else if (msgIdx==91u)
          {
             switch(posMode[1u])
             {
                case 'A':
                  strcpy(positionMode,"GNSS fix\n");
                break;
                case 'D':
                  strcpy(positionMode,"Differential GNSS fix\n");
                break;
                case 'N':
                  strcpy(positionMode,"No position fix\n");
                  msgIdx=255u;                                                  // reset we got a failed message and wait for a $ start
                break;
                default:
                  strcpy(positionMode,"Unknown fix\n");
                  msgIdx=255u;                                                  // reset we got a failed message and wait for a $ start
                break;
             }
          }
          else if ((msgIdx >= 108u) && (msgIdx <= 120u))
          {
              signalIDData=(uint8_t) atoi(signalID);
              msgIdx=120u;                                                      // will set to 121 with the ++ later (means collect CRC)
          }
          charIdx=1u;                                                           // reset the read buffer to the read the first char into the first byte of next characture array
          msgIdx=++msgIdx % UINT8_MAX;                                          // increment to next message index collector slot
        break;
//====================================== Comma ===================================
        case ',':                                                               // Field separator has been found ======================== ,Comma, ==========
          if (!(((((((msgIdx >= 5u) && (msgIdx <= 26u)) || ((msgIdx >= 42u) && (msgIdx <= 56u))) || ((msgIdx >= 62u) && (msgIdx <= 73u))) || ((msgIdx >= 101u) && (msgIdx <= 121u))) || ((msgIdx >= 131u) && (msgIdx <= 138u))) || ((msgIdx >= 82u) && (msgIdx <= 92u))))
          {
             msgIdx=255u;                                                       // go to dormant wait mode for another STX $ as its a message we dont yet support
          }
          else if (msgIdx==5u)                                                  // should be first comma
          {
             msgIdx=6u;                                                         // ====PUBX==== now collect the msgId filed (we dont use)
             charIdx=1u;
          }
          else if (msgIdx==6u)                                                  // should be 2nd comma then convert the collected data
          {
             msgIdPUBX=atoi(msgId);                                             // convert the ascii string received to the message identifier used to read the fields
             msgIdx=7u;                                                         // now collect the msgId filed (we dont use)
             charIdx=1u;
          }
          else if ((msgIdx==7u) && (msgIdPUBX==0u))                             // should be 3rd comma and its a position message
          {
             timeGPSPUBX=atoi(timeGPS);                                         // convert the time from the message
             msgIdx=8u;                                                         // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==8u) && (msgIdPUBX==0u))                             // should be 4th comma and its a position message
          {
             latGPSPUBX=atof(latGPS);                                           // convert the latitude field from the message
             msgIdx=9u;                                                         // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==9u) && (msgIdPUBX==0u))                             // should be 5th comma and its a position message
          {
             if(nsGPS[1u] == 'S')                                               // if the latitude is in the South direction it has minus sign
             {
                latGPSPUBX = 0u - latGPSPUBX;                                   // invert the latitude direction
             }
             msgIdx=10u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==10u) && (msgIdPUBX==0u))                            // should be 6th comma and its a position message
          {
             longGPSPUBX=atof(longGPS);                                         // convert the longditude field from the message
             msgIdx=11u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==11u) && (msgIdPUBX==0u))                            // should be 7th comma and its a position message
          {
             if(ewGPS[1u] == 'W')                                               // if the longitude is in the West direction it has minus sign
             {
                longGPSPUBX = 0u - longGPSPUBX;                                 // invert
             }
             msgIdx=12u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==12u) && (msgIdPUBX==0u))                            // should be 8th comma and its a position message
          {
             altRefGPSPUBX=atof(altRefGPS);                                     // convert the Altitude above user datum ellipsoid field from the message
             msgIdx=13u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==13u) && (msgIdPUBX==0u))                            // should be 9th comma and its a position message
          {
             navStatGPS[3u]='\0';                                               // terminate the string after 2 chars
             if (strcmp("NF",navStatGPS)==0u)                                   // navigation Status reported No Fix
             {
                strcpy(navStatGPS,"No Fix found\n");
                msgIdx=0u;                                                      // reset we got a failed message
             }
             else if (strcmp("DR",navStatGPS)==0u)                              // navigation Status reported dead reckoning only solution
             {
                strcpy(navStatGPS,"Dead reckoning only solution\n");
             }
             else if (strcmp("G2",navStatGPS)==0u)                              // navigation Status reported Stand alone 2D solution
             {
                strcpy(navStatGPS,"Stand alone 2D solution\n");
             }
             else if (strcmp("G3",navStatGPS)==0u)                              // navigation Status reported Stand alone 3D solution
             {
                strcpy(navStatGPS,"Stand alone 3D solution\n");
             }
             else if (strcmp("D2",navStatGPS)==0u)                              // navigation Status reported Differential 2D solution
             {
                strcpy(navStatGPS,"Differential 2D solution\n");
             }
             else if (strcmp("D3",navStatGPS)==0u)                              // navigation Status reported Differential 3D solution
             {
                strcpy(navStatGPS,"Differential 3D solution\n");
             }
             else if (strcmp("RK",navStatGPS)==0u)                              // navigation Status reported  Combined GPS + dead reckoning solution
             {
                strcpy(navStatGPS,"Combined GPS + dead reckoning solution \n");
             }
             else if (strcmp("TT",navStatGPS)==0u)                              // navigation Status reported  Time only solution
             {
                strcpy(navStatGPS," Time only solution\n");
             }
             charIdx=1;
          }
          else if ((msgIdx==14) && (msgIdPUBX==0u))                             // should be 10th comma and its a position message
          {
             hAccGPSPUBX=atof(hAccGPS);                                         // convert the Horizontal accuracy estimate.
             msgIdx=15u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==15u) && (msgIdPUBX==0u))                            // should be 11th comma and its a position message
          {
             vAccGPSPUBX=atof(vAccGPS);                                         // convert the Vertical accuracy estimate
             msgIdx=16u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==16u) && (msgIdPUBX==0u))                            // should be 12th comma and its a position message
          {
             SOGGPSPUBX=atof(SOGGPS);                                           // convert the speed over ground
             msgIdx=17u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==17u) && (msgIdPUBX==0u))                            // should be 13th comma and its a position message
          {
             COGGPSPUBX=atof(COGGPS);                                           // convert the course over ground field from the message
             msgIdx=18u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==18u) && (msgIdPUBX==0u))                            // should be 14th comma and its a position message
          {
             vVelGPSPUBX=atof(vVelGPS);                                         // convert the vertical velocity field from the message
             msgIdx=19u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==19u) && (msgIdPUBX==0u))                            // should be 15th comma and its a position message
          {
             diffAgeGPSPUBX=atof(diffAgeGPS);                                   // convert the Age of differential corrections field from the message
             msgIdx=20u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==20u) && (msgIdPUBX==0u))                            // should be 16th comma and its a position message
          {
             HDOPGPSPUBX=atof(HDOPGPS);                                         // convert the Horizontal Dilution of Precision  field from the message
             msgIdx=21u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==21u) && (msgIdPUBX==0u))                            // should be 17th comma and its a position message
          {
             VDOPGPSPUBX=atof(VDOPGPS);                                         // convert the Vertical Dilution of Precisio field from the message
             msgIdx=22u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==22u) && (msgIdPUBX==0u))                            // should be 18th comma and its a position message
          {
             TDOPGPSPUBX=atof(TDOPGPS);                                         // convert the  Time Dilution of Precision field from the message
             msgIdx=23u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==23u) && (msgIdPUBX==0u))                            // should be 19th comma and its a position message
          {
             numSvsGPSPUBX=atof(numSvsGPS);                                     // convert the Number of satellites used in the navigation solution  field from the message
             msgIdx=24u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==24u) && (msgIdPUBX==0u))                            // should be 20th comma and its a position message
          {
           // reerved field for future                                          // convert the ???? field from the message
             msgIdx=25u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx==25u) && (msgIdPUBX==0u))                            // should be 21th comma and its a position message (dont think we get here we get a * rather than comma)
          {
             DRGPSPUBX=atof(DRGPS);                                             // convert the DR field from the message
             msgIdx=26u;                                                        // now collect the next field
             charIdx=1u;
          }
          else if ((msgIdx>=42u) || (msgIdx==56u))                              // Start of an ===RMC==== message type to End comma means next field response to :: $xxGLQ,msgId*cs<CR><LF>  $EIGLQ,RMC*3A<CR><LF>
          {
             switch(msgIdx)                                                     // For Each field in the RMC message string parse and calculate data values
             {
                case 43u:                                                       // field is time
                   timeGPSPUBX=atoi(timeGPS);
                   break;
                case 44u:                                                       // Status field for RMC message A=ok V=error
                   if (rmcStat[1u]=='V')                                        // invalid data then reset the message collection
                   {
                      i=1u;                                                     // break from collection if Status invalid
                      msgIdx=0u;
                   }
                   break;
                case 45u:
                   latGPSPUBX=atof(latGPS);
                   break;
                case 46u:
                   if(nsGPS[1] == 'S')                                          // if the latitude is in the South direction it has minus sign
                   {
                     latGPSPUBX = 0u - latGPSPUBX;                              // invert the latitude direction
                   }
                   break;
                case 47u:
                   longGPSPUBX=atof(longGPS);
                   break;
                case 48u:
                   if(ewGPS[1] == 'W')                                          // if the longitude is in the West direction it has minus sign
                   {
                      longGPSPUBX = 0u - longGPSPUBX;                           // invert
                   }
                   break;
                case 49u:
                   SOGGPSPUBX=atof(SOGGPS);                                     // speed in knots
                   break;
                case 50u:
                   COGGPSPUBX=atof(COGGPS);                                     // true course
                   break;
                 case 54u:                                                       // position mode satelite
                    switch(posMode[1])
                    {
                        case 'A':
                           strcpy(positionMode,"GNSS fix\n");
                           break;
                        case 'D':
                           strcpy(positionMode,"Differential GNSS fix\n");
                           break;
                        case 'N':
                           strcpy(positionMode,"No position fix\n");
                           msgIdx=0u;                                           // reset we got a failed message
                           break;
                        default:
                           strcpy(positionMode,"Unknown fix\n");
                           msgIdx=0u;                                           // reset we got a failed message
                           break;
                     }
                    break;
                 case 55u:                                                      // navigation Status is always V so ignore will increment to last field CRC after this
                   break;
                 case 56u:                                                      // checksum  (shouldnt collect as it doesnt have a comma separator should do on <CR>
                   csGPSPUBX=((uint16_t)hex2int(csGPS));
                   break;
                 default:
                   break;
             }
             msgIdx=++msgIdx % UINT8_MAX;                                       // go to the next field
             charIdx=1u;                                                        // reset to start at the begining as we got a comma
        }
        else if ((msgIdx>=131u) || (msgIdx==137u))                              // Start of an ===GLL==== message type to End comma means next field response to
        {
           switch(msgIdx)                                                       // For Each field in the RMC message string parse and calculate data values
           {
              case 135u:                                                        // field is time
                timeGPSPUBX=atoi(timeGPS);
              break;
              case 136u:                                                        // Status field for GLL message A=ok V=error
                if (rmcStat[1u]=='V')                                           // invalid data then reset the message collection
                {
                   i=1u;                                                        // break from collection if Status invalid
                   msgIdx=255u;
                }
              break;
              case 131u:
                 latGPSPUBX=atof(latGPS);
              break;
              case 132u:
                 if(nsGPS[1] == 'S')                                            // if the latitude is in the South direction it has minus sign
                 {
                    latGPSPUBX = 0u - latGPSPUBX;                               // invert the latitude direction
                 }
              break;
              case 133u:
                 longGPSPUBX=atof(longGPS);
              break;
              case 134u:
                 if(ewGPS[1u] == 'W')                                            // if the longitude is in the West direction it has minus sign
                 {
                    longGPSPUBX = 0u - longGPSPUBX;                              // invert
                 }
              break;
              case 137u:                                                         // position mode satelite
                  switch(posMode[1u])
                  {
                     case 'A':
                       strcpy(positionMode,"GNSS fix\n");
                     break;
                     case 'D':
                        strcpy(positionMode,"Differential GNSS fix\n");
                     break;
                     case 'N':
                        strcpy(positionMode,"No position fix\n");
                        msgIdx=255u;                                            // reset we got a failed message
                     break;
                     default:
                       strcpy(positionMode,"Unknown fix\n");
                       msgIdx=255u;                                             // reset we got a failed message
                     break;
                  }
                  break;
              default:
              break;
            }
            msgIdx=++msgIdx % UINT8_MAX;                                        // go to the next field
            charIdx=1u;                                                         // reset to start at the begining as we got a comma
        }
        else if ((msgIdx>=82u) || (msgIdx==91u))                                // Start of an ====VTG==== message type to End comma means next field response to
        {
           switch(msgIdx)                                                       // For Each field in the VTG message string parse and calculate data values
           {

                case 82u:
                   COGGPSPUBX=atof(COGGPS);                                     // true course
                   break;
                case 86u:
                   SOGGPSPUBX=atof(SOGGPS);                                     // speed knotts
                   break;
                case 88u:
                   SOGKGPSPUBX=atof(SOGKGPS);                                   // speed  kmh
                   break;
                case 91u:                                                       // position mode satelite (shouldnt reach as separated with * not ,
                  switch(posMode[1])
                  {
                     case 'A':
                       strcpy(positionMode,"GNSS fix\n");
                     break;
                     case 'D':
                        strcpy(positionMode,"Differential GNSS fix\n");
                     break;
                     case 'N':
                       strcpy(positionMode,"No position fix\n");
                       msgIdx=0u;                                               // reset we got a failed message
                     break;
                     default:
                       strcpy(positionMode,"Unknown fix\n");
                       msgIdx=0u;                                               // reset we got a failed message
                     break;
                  }
                  break;
               default:
                 break;
            }
            msgIdx=++msgIdx % UINT8_MAX;                                        // go to the next field
            charIdx=1u;                                                         // reset to start at the begining as we got a comma
          }
          else if ((msgIdx>=101u) || (msgIdx==120u))                            // Start of an ====GSV===== message type to End comma means next field response to
          {
               switch(msgIdx)
               {
                  case 101u:
                    numOfSatel=(uint8_t) atoi(numSatel);                        // convert string with number of satelites to a value
                    msgIdx=102u;                                                // now collect the next field
                    charIdx=1u;
                    break;
                  case 102u:
                    msgIdx=103u;                                                // now collect the next field  (ignore the data)
                    charIdx=1u;
                    break;
                  case 103u:
                    numOfSatel=(uint8_t) atoi(numSatel);                        // convert string with number of satelites in sight to a value
                    msgIdx=104u;                                                // now collect the next field
                    charIdx=1u;
                    break;
                  case 104u:
                    satNumberData[1u]=(uint8_t) atoi(satNumber);                // convert string with the satelite being used number to a value
                    msgIdx=105u;                                                // now collect the next field
                    charIdx=1u;
                    break;
                  case 105u:
                    elevSatelData[1u]=(uint8_t) atoi(elevSatel);                // convert string with number of satelites to a value
                    msgIdx=106u;                                                // now collect the next field
                    charIdx=1u;
                    break;
                  case 106u:
                    azimuthSatelData[1u]=(uint8_t) atoi(azimuthSatel);          // convert string with number of satelites to a value
                    msgIdx=107u;                                                // now collect the next field
                    charIdx=1u;
                    break;
                  case 107u:
                    sigStrengthSatelData[1u]=(uint8_t) atoi(sigStrengthSatel);  // convert string with number of satelites to a value
                    msgIdx=108u;                                                // now collect the next field
                    charIdx=1u;
                    break;
                  case 108u:
                    if (numOfSatel >= 2u)
                    {
                       satNumberData[2u]=(uint8_t) atoi(satNumber);             // convert string with the satelite being used number to a value
                       msgIdx=109u;                                             // now collect the next field
                       charIdx=1u;
                    }
                    else
                    {
                       signalIDData=(uint8_t) atoi(signalID);
                    }
                    break;
                  case 109u:
                    if (numOfSatel >= 2u)
                    {
                       elevSatelData[2u]=(uint8_t) atoi(elevSatel);             // convert string with number of satelites to a value
                       msgIdx=110u;                                             // now collect the next field
                       charIdx=1u;
                    }
                    break;
                  case 110u:
                    if (numOfSatel >= 2u)
                    {
                       azimuthSatelData[2u]=(uint8_t) atoi(azimuthSatel);       // convert string with number of satelites to a value
                       msgIdx=111u;                                             // now collect the next field
                       charIdx=1u;
                    }
                    break;
                  case 111u:
                    if (numOfSatel >= 2u)
                    {
                       sigStrengthSatelData[2u]=(uint8_t) atoi(sigStrengthSatel);// convert string with number of satelites to a value
                       msgIdx=112u;                                             // now collect the next field
                       charIdx=1u;
                    }
                    break;
                  case 112u:
                    if (numOfSatel >= 3u)
                    {
                       satNumberData[3u]=(uint8_t) atoi(satNumber);             // convert string with the satelite being used number to a value
                       msgIdx=++msgIdx % UINT8_MAX;                             // now collect the next field
                       charIdx=1u;
                    }
                    else
                    {
                       signalIDData=(uint8_t) atoi(signalID);
                    }
                    break;
                  case 113u:
                    if (numOfSatel >= 3u)
                    {
                       elevSatelData[3]=(uint8_t) atoi(elevSatel);              // convert string with number of satelites to a value
                       msgIdx=++msgIdx % UINT8_MAX;                             // now collect the next field
                       charIdx=1u;
                    }
                    break;
                  case 114u:
                    if (numOfSatel >= 3u)
                    {
                       azimuthSatelData[3u]=(uint8_t) atoi(azimuthSatel);       // convert string with number of satelites to a value
                       msgIdx=++msgIdx % UINT8_MAX;                             // now collect the next field
                       charIdx=1u;
                    }
                    break;
                  case 115u:
                    if (numOfSatel >= 3u)
                    {
                       sigStrengthSatelData[3u]=(uint8_t) atoi(sigStrengthSatel);// convert string with number of satelites to a value
                       msgIdx=++msgIdx % UINT8_MAX;                             // now collect the next field
                       charIdx=1u;
                    }
                    break;
                  case 116u:
                    if (numOfSatel >= 4u)
                    {
                       satNumberData[4u]=(uint8_t) atoi(satNumber);             // convert string with the satelite being used number to a value
                       msgIdx=++msgIdx % UINT8_MAX;                             // now collect the next field
                       charIdx=1u;
                    }
                    else
                    {
                       signalIDData=(uint8_t) atoi(signalID);
                    }
                    break;
                  case 117u:
                    if (numOfSatel >= 4u)
                    {
                       elevSatelData[4u]=(uint8_t) atoi(elevSatel);             // convert string with number of satelites to a value
                       msgIdx=++msgIdx % UINT8_MAX;                             // now collect the next field
                       charIdx=1u;
                    }
                    break;
                  case 118u:
                    if (numOfSatel >= 4u)
                    {
                       azimuthSatelData[4u]=(uint8_t) atoi(azimuthSatel);       // convert string with number of satelites to a value
                       msgIdx=++msgIdx % UINT8_MAX;                             // now collect the next field
                       charIdx=1u;
                    }
                    break;
                  case 119u:
                    if (numOfSatel >= 4u)
                    {
                       sigStrengthSatelData[4u]=(uint8_t) atoi(sigStrengthSatel);// convert string with number of satelites to a value
                       msgIdx=++msgIdx % UINT8_MAX;                             // now collect the next field
                       charIdx=1u;
                    }
                    break;
                  case 120u:
                    signalIDData=(uint8_t) atoi(signalID);
                    break;
               }
          }
          else if ((msgIdx>=62u) || (msgIdx==73u))                              // Start of an ====GNS===== message type to End comma means next field response to
          {
             if (msgIdx==62u)                                                   // =====GNS====== message
             {
               timeGPSPUBX=atoi(timeGPS);                                       // convert the time from the message
               msgIdx=63u;                                                      // now collect the next field
               charIdx=1u;
             }
             else if (msgIdx==63u)                                              // =====GNS======= message
             {
               latGPSPUBX=atoi(latGPS);                                         // convert the latitude from the message
               msgIdx=64u;                                                      // now collect the next field
               charIdx=1u;                                                      // field 2 of the GNS message latitude
             }
             else if (msgIdx==64u)                                              // =====GNS======= message
             {
               if(nsGPS[1u] == 'S')                                             // if the latitude is in the South direction it has minus sign
               {
                  latGPSPUBX = 0u - latGPSPUBX;                                 // invert the latitude direction
               }
               msgIdx=65u;                                                      // now collect the next field
               charIdx=1u;
             }
             else if (msgIdx==65u)                                              // =====GNS======= message
             {                                                                  // field 4 of the GNS message longditude
                longGPSPUBX=atoi(longGPS);                                      // convert longditude field
                msgIdx=66u;                                                     // now collect the next field
                charIdx=1u;
             }
             else if (msgIdx==66u)                                              // =====GNS======= message
             {                                                                  // field 5 of the GNS e/w indicator
                if(ewGPS[1u] == 'W')                                            // if the longitude is in the West direction it has minus sign
                {
                   longGPSPUBX = 0u - longGPSPUBX;                              // invert the latitude direction
                }
                msgIdx=67u;                                                     // now collect the next field
                charIdx=1u;
             }
             else if (msgIdx==67u)                                              // =====GNS======= message
             {                                                                  // field 6 of the GNS message position mode
                switch(posMode[1u])
                {
                  case 'A':
                    strcpy(positionMode,"GNSS fix\n");
                    break;
                  case 'D':
                    strcpy(positionMode,"Differential GNSS fix\n");
                    break;
                  case 'N':
                     strcpy(positionMode,"No position fix\n");
                     msgIdx=255u;                                               // reset we got a failed message
                     break;
                  default:
                    strcpy(positionMode,"Unknown fix\n");
                    msgIdx=255u;                                                // reset we got a failed message
                    break;
                }
                msgIdx=68u;                                                     // now collect the next field
                charIdx=1u;
             }
             else if (msgIdx==68u)                                              // =====GNS======= message
             {                                                                  // field 7 of the GNS message depending on number of satelites
                 numOfSatel=(uint8_t) atoi(numSatel);                           // convert string with number of satelites in sight to a value
                 msgIdx=69u;                                                    // now collect the next field
                 charIdx=1u;
             }
             else if (msgIdx==69u)                                              // =====GNS======= message
             {                                                                  // field 8 of the GNS message horizontal dilution
                HDOPGPSPUBX=atof(HDOPGPS);                                      // convert the Horizontal Dilution of Precision  field from the message
                msgIdx=70u;                                                     // now collect the next field
                charIdx=1u;
             }
             else if (msgIdx==70u)                                              // =====GNS======= message
             {                                                                  // field 9 of the GNS message sea level altitude (meters)
                SealevelAltitude=atof(seaLevelAlt);                             // convert the sea level altitude (meters)  field from the message
                msgIdx=71u;                                                     // now collect the next field
                charIdx=1u;
             }
             else if (msgIdx==71u)                                              // =====GNS======= message
             {                                                                  // field 10 of the GNS message Geoidal height
                GeoidalHt=atof(geoHt);                                          // convert the Geoidal height (difference between ellipsoid and mean sea level)  field from the message
                msgIdx=72u;                                                     // now collect the next field
                charIdx=1u;
             }
          }
          break;
//=================== <CR> 0x0D ================================================
        case 13u:                                                               // Carriage return is second last char  Terminate regardless when found
           csGPSPUBX=((uint16_t)hex2int(csGPS));                                // Convert the checksum field from the message should just preceede it starts with a (*) char
           msgIdx=250u;                                                         // Look for the final line feed
           charIdx=1u;
           break;
//================== <LF> 0x0A =================================================
        case 10u:                                                               // line feed is end of message
           if (msgIdx==250u)
              msgIdx=251u;                                                      // message read state has been set to complete

           //else if (msgIdx==62)  ============ This method cant work in mikroe C for PIC32 =======
           //{
              //sscanf(gnsBuffer,"%d , %f , %s , %f , %s , %s , %d , %f , %f , %f ,,, %s, %s",timeGPSPUBX,latGPSPUBX,nsGPS,longGPSPUBX,ewGPS,posMode,numOfSatel,HDOPGPSPUBX,SealevelAlt,GeoHt,navStatGPS,csGPS);
              //SSCANF_GSV(gnsBuffer, timeGPSPUBX, latGPSPUBX);
           //}
           break;
//================ Fields to collect as data ===================================
        default:
          if (msgIdx==6u)
          {
             if (charIdx <= sizeof(msgId))
               msgId[charIdx]=GPStxt[i];                                        // read msg id
             charIdx=++charIdx % UINT8_MAX;
          }
          else if (msgIdPUBX==0u)                                               // message was ====PUBX===== ==== MSGID==0 ====== (position message)
          {
             if (msgIdx==7u)                                                    // 2nd field in message string and its a position message
             {
                if (charIdx <= sizeof(timeGPS))
                  timeGPS[charIdx]=GPStxt[i];                                   // read time field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==8u)                                                // 3rd field in message string and its a position message
             {
                if (charIdx <= sizeof(latGPS))
                  latGPS[charIdx]=GPStxt[i];                                    // read latitude field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==9u)                                               // 4th field in message string and its a position message
             {
                if (charIdx <= sizeof(nsGPS))
                  nsGPS[charIdx]=GPStxt[i];                                     // read north / south field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==10u)                                              // 5th field in message string and its a position message
             {
                if (charIdx <= sizeof(longGPS))
                  longGPS[charIdx]=GPStxt[i];                                   // read longditude field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==11u)                                              // 6th field in message string and its a position message
             {
                if (charIdx <= sizeof(ewGPS))
                  ewGPS[charIdx]=GPStxt[i];                                     // read east west field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==12u)                                              // 7th field in message string and its a position message
             {
                if (charIdx <= sizeof(altRefGPS))
                  altRefGPS[charIdx]=GPStxt[i];                                 // read Altitude above user datum ellipsoid field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==13u)                                              // 8th field in message string and its a position message
             {
                if (charIdx <= sizeof(navStatGPS))
                  navStatGPS[charIdx]=GPStxt[i];                                // read navigation Status string
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==14u)                                              // 9th field in message string and its a position message
             {
                if (charIdx <= sizeof(hAccGPS))
                  hAccGPS[charIdx]=GPStxt[i];                                   // read horizontal accuracy estimate string
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==15u)                                              // 10th field in message string and its a position message
             {
                if (charIdx <= sizeof(vAccGPS))
                  vAccGPS[charIdx]=GPStxt[i];                                   // read vertical accuracy estimate string
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==16u)                                              // 11th field in message string and its a position message
             {               
                if (charIdx <= sizeof(SOGGPS))
                 SOGGPS[charIdx]=GPStxt[i];                                     // read speed over ground string
               charIdx=++charIdx % UINT8_MAX;;
             }
             else if (msgIdx==17u)                                              // 12th field in message string and its a position message
             {
               if (charIdx <= sizeof(COGGPS))
                 COGGPS[charIdx]=GPStxt[i];                                     // read course over ground string
               charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==18u)                                              // 13th field in message string and its a position message
            {
               if (charIdx <= sizeof(vVelGPS))
                  vVelGPS[charIdx]=GPStxt[i];                                   // read Vertical velocity (positive downwards)
               charIdx=++charIdx % UINT8_MAX;
            }
            else if (msgIdx==19u)                                               // 14th field in message string and its a position message
            {
               if (charIdx <= sizeof(diffAgeGPS))
                  diffAgeGPS[charIdx]=GPStxt[i];                                // read Vertical velocity (positive downwards)
               charIdx=++charIdx % UINT8_MAX;
            }
            else if (msgIdx==20u)                                               // 15th field in message string and its a position message
            {
               if (charIdx <= sizeof(HDOPGPS))
                  HDOPGPS[charIdx]=GPStxt[i];                                   // read HDOP, Horizontal Dilution of Precision
               charIdx=++charIdx % UINT8_MAX;
            }
            else if (msgIdx==21u)                                               // 16th field in message string and its a position message
            {
               if (charIdx <= sizeof(VDOPGPS))
                  VDOPGPS[charIdx]=GPStxt[i];                                   // read VDOP, Vertical Dilution of Precision
               charIdx=++charIdx % UINT8_MAX;
            }
            else if (msgIdx==22u)                                               // 17th field in message string and its a position message
            {
               if (charIdx <= sizeof(TDOPGPS))
                  TDOPGPS[charIdx]=GPStxt[i];                                   // read TDOP, Time Dilution of Precision
               charIdx=++charIdx % UINT8_MAX;
            }
            else if (msgIdx==23u)                                               // 18th field in message string and its a position message
            {
               if (charIdx <= sizeof(numSvsGPS))
                  numSvsGPS[charIdx]=GPStxt[i];                                 // read number of satelites used in the navigation solution
               charIdx=++charIdx % UINT8_MAX;
            }
            else if (msgIdx==24u)                                               // 19th field in message string and its a position message
            {
               if (charIdx <= sizeof(reservedGPS))
                 reservedGPS[charIdx]=GPStxt[i];                                // read nreserved field (ignore for now)
               charIdx=++charIdx % UINT8_MAX;
            }
            else if (msgIdx==25u)                                               // 20th field in message string and its a position message
            {
               if (charIdx <= sizeof(DRGPS))
                  DRGPS[charIdx]=GPStxt[i];                                     // read DR used
               charIdx=++charIdx % UINT8_MAX;
            }
            else if (msgIdx==26u)                                               // 21th field in message string and its a position message
            {
               if (charIdx <= sizeof(csGPS))
                 csGPS[charIdx]=GPStxt[i];                                      // read hexidecimal characture representation of the checksum
               charIdx=++charIdx % UINT8_MAX;
            }
          }
          else if ((msgIdx >= 43u) && (msgIdx <= 55u))                          // Start of RMC NMEA message parsing------------------
          {
             if (msgIdx==43u)
             {
               if (charIdx <= sizeof(timeGPS))
                 timeGPS[charIdx]=GPStxt[i];                                    // read time field
               charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==44u)                                              // field 2 of the RMC message
             {
               if (charIdx <= 1u)
                 rmcStat[1u]=GPStxt[i];                                         // read rtc status
               charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==45u)                                              // field 3 of the RMC message lat
             {
                if (charIdx <= sizeof(latGPS))
                  latGPS[charIdx]=GPStxt[i];                                    // read latitude field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==46u)                                              // field 4 of the RMC message
             {
                if (charIdx <= sizeof(nsGPS))
                  nsGPS[charIdx]=GPStxt[i];                                     // read north / south field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==47u)                                              // field 5 of the RMC message
             {
                if (charIdx <= sizeof(longGPS))
                  longGPS[charIdx]=GPStxt[i];                                   // read longditude field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==48u)                                              // field 5 of the RMC message
             {
                if (charIdx <= sizeof(ewGPS))
                  ewGPS[charIdx]=GPStxt[i];                                     // read east west field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==49u)                                              // field 6 of the RMC message
             {
                if (charIdx <= sizeof(SOGGPS))
                  SOGGPS[charIdx]=GPStxt[i];                                    // read speed over ground string
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==50u)                                              // field 7 of the RMC message
             {
                if (charIdx <= sizeof(COGGPS))
                   COGGPS[charIdx]=GPStxt[i];                                   // read course over ground string
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==51u)                                              // field 8 of the RMC message
             {
                if (charIdx <= sizeof(dateStr1))
                  dateStr1[charIdx]=GPStxt[i];                                  // date string
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==52u)                                              // field 9 of the RMC message
             {
               if (charIdx <= sizeof(magVarStr))
                  magVarStr[charIdx]=GPStxt[i];                                 // magnetic variation string
               charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==53u)                                              // field 10 of the RMC message
             {
                if (charIdx <= sizeof(magVarEw))
                   magVarEw[charIdx]=GPStxt[i];                                 // magnet variation e/w
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==54u)                                              // field 11 of the RMC message
             {
                if (charIdx <= sizeof(posMode))
                   posMode[charIdx]=GPStxt[i];                                  // mode
               charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==55u)                                              // field 12 of the RMC message
             {
                if (charIdx <= sizeof(navStatus))
                   navStatus[charIdx]=GPStxt[i];                                // navigation status string
                charIdx=++charIdx % UINT8_MAX;
             }
          }
          else if ((msgIdx >= 131u) && (msgIdx <= 137u))                        // start of ==== GLL =======
          {
              if (msgIdx==131u)                                                 // field 1 of the GLL message lat
              {
                if (charIdx <= sizeof(latGPS))
                   latGPS[charIdx]=GPStxt[i];                                   // read latitude field
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==132u)                                            // field 2 of the GLL message
              {
                if (charIdx <= sizeof(nsGPS))
                   nsGPS[charIdx]=GPStxt[i];                                    // read north / south field
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==133u)                                            // field 3 of the GLL message
              {
                if (charIdx <= sizeof(longGPS))
                   longGPS[charIdx]=GPStxt[i];                                  // read longditude field
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==134u)                                            // field 4 of the GLL message
              {
                if (charIdx <= sizeof(ewGPS))
                   ewGPS[charIdx]=GPStxt[i];                                    // read east west field
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==135u)                                            // field 5 of the GLL message
              {
                 if (charIdx <= sizeof(timeGPS))
                    timeGPS[charIdx]=GPStxt[i];                                 // read time field
                 charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==136u)                                            // field 6 of the GLL message
              {
                 if (charIdx <= 1u)
                   rmcStat[1]=GPStxt[i];                                        // read rtc status
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==137u)                                             // field 7 of the GLL message
             {
                if (charIdx <= sizeof(posMode))
                   posMode[charIdx]=GPStxt[i];                                  // mode
                charIdx=++charIdx % UINT8_MAX;
             }
          }
          else if ((msgIdx >= 82u) && (msgIdx <= 91u))                          // ==== VTG message ===========
          {
              if (msgIdx==82u)                                                  // field 1 of the VTG message
              {
                 if (charIdx <= sizeof(COGGPS))
                    COGGPS[charIdx]=GPStxt[i];                                  // read course over ground string
                 charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==83u)                                             // field 2 of the VTG message
              {
                 if (charIdx <= 1u)
                    charRead[1]=GPStxt[i];                                      // read T (true course)
                 charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==84u)                                             // field 3 of the VTG message
              {
                 charIdx=++charIdx % UINT8_MAX;                                 // ignore always a NULL
              }
              else if (msgIdx==85u)                                             // field 4 of the VTG message
              {
                if (charIdx <= 1u)
                   charRead[1]=GPStxt[i];                                       // read M (magnetic)
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==86u)                                             // field 5 of the VTG message
              {
                if (charIdx <= sizeof(SOGGPS))
                   SOGGPS[charIdx]=GPStxt[i];                                   // read speed over ground string knots
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==87u)                                             // field 6 of the VTG message
              {
                if (charIdx <= 1u)
                   charRead[1u]=GPStxt[i];                                      // read M (knots)
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==88u)                                             // field 7 of the VTG message
              {
                if (charIdx <= sizeof(SOGGPS))
                   SOGKGPS[charIdx]=GPStxt[i];                                  // read speed over ground string km/h
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==89u)                                             // field 8 of the VTG message
              {
                if (charIdx <= 1u)
                   charRead[1u]=GPStxt[i];                                      // read K (kmh)
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==90u)                                             // field 8 of the VTG message
              {
                if (charIdx <= 1u)
                   charRead[1u]=GPStxt[i];                                      // read K (kmh)
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==91u)                                             // field 8 of the VTG message
              {
                if (charIdx <= sizeof(posMode))
                  posMode[charIdx]=GPStxt[i];                                   // mode
                charIdx=++charIdx % UINT8_MAX;
              }
          }
          else if ((msgIdx>=101u) && (msgIdx<=120u))                            //============ GSV message =====================
          {
              if (msgIdx==101u)                                                 // field 1 of the GSV message
              {
                if (charIdx <= sizeof(numSatel))
                   numSatel[charIdx]=GPStxt[i];                                 // number of satelites
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==102u)                                            // field 2 of the GSV message
              {
                if (charIdx <= 1u)
                  charRead[1u]=GPStxt[i];                                       // ignore message number for now
                charIdx=++charIdx % UINT8_MAX;
              }
              else if (msgIdx==103u)                                            // field 3 of the GSV message
              {
                if (charIdx <= sizeof(numSatelinSight))
                   numSatelinSight[charIdx]=GPStxt[i];                          // number of satelites in sight
                charIdx=++charIdx % UINT8_MAX;
              }
              else if ((((msgIdx==104u) || ((msgIdx==108u) && (numOfSatel>=2u))) || ((msgIdx==112u) && (numOfSatel>=3u))) || ((msgIdx==116u) && (numOfSatel>=4u)))
              {                                                                 // field 4 of the GSV message
                 if (charIdx <= sizeof(satNumber))
                   satNumber[charIdx]=GPStxt[i];                                // satelite elevation
                charIdx=++charIdx % UINT8_MAX;
              }
              else if ((((msgIdx==105u) || ((msgIdx==109u) && (numOfSatel>=2u))) || ((msgIdx==113u) && (numOfSatel>=3u))) || ((msgIdx==117u) && (numOfSatel>=4u)))
              {                                                                 // field 4 of the GSV message
                 if (charIdx <= sizeof(elevSatel))
                   elevSatel[charIdx]=GPStxt[i];                                // satelite elevation
                 charIdx=++charIdx % UINT8_MAX;
              }
              else if ((((msgIdx==106u) || ((msgIdx==110u) && (numOfSatel>=2u))) || ((msgIdx==114u) && (numOfSatel>=3u))) || ((msgIdx==118u) && (numOfSatel>=4u)))
              {                                                                 // field 5 of the GSV message
                 if (charIdx <= sizeof(azimuthSatel))
                    azimuthSatel[charIdx]=GPStxt[i];                            // satelite azimuth
                 charIdx=++charIdx % UINT8_MAX;
              }
              else if ((((msgIdx==107u) || ((msgIdx==111u) && (numOfSatel>=2u))) || ((msgIdx==115u) && (numOfSatel>=3u))) || ((msgIdx==119u) && (numOfSatel>=4u)))
              {                                                                 // field 6 of the GSV message
                 if (charIdx <= sizeof(sigStrengthSatel))
                   sigStrengthSatel[charIdx]=GPStxt[i];                         // satelite signal stength
                 charIdx=++charIdx % UINT8_MAX;
              }
              else if (((((msgIdx==108u) && (numOfSatel==1u)) || ((msgIdx==112u) && (numOfSatel==2u))) || ((msgIdx==116u) && (numOfSatel==3u))) || ((msgIdx==120u) && (numOfSatel==4u)))
             {                                                                  // field 7 or more of the GSV message depending on number of satelites
                if (charIdx <= sizeof(signalID))
                   signalID[charIdx]=GPStxt[i];                                 // GNSS signal ID
                charIdx=++charIdx % UINT8_MAX;
             }
          }
          else if ((msgIdx >= 62u) && (msgIdx <= 72u))                          // =============== GNS message =========
          {
             if (msgIdx==62u)                                                   // =====GNS======= message
             {                                                                  // field 1 of the GNS message time field
                if (charIdx <= sizeof(timeGPS))
                  timeGPS[charIdx]=GPStxt[i];                                   // read time field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==63u)                                              // =====GNS======= message
             {                                                                  // field 2 of the GNS message latitude
               if (charIdx <= sizeof(latGPS))
                 latGPS[charIdx]=GPStxt[i];                                     // read latitude field
               charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==64u)                                              // =====GNS======= message
             {                                                                  // field 3 of the GNS message n/s indication
                if (charIdx <= sizeof(nsGPS))
                  nsGPS[charIdx]=GPStxt[i];                                     // read north / south field
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==65u)                                              // =====GNS======= message
             {                                                                  // field 4 of the GNS message longditude
               if (charIdx <= sizeof(longGPS))
                 longGPS[charIdx]=GPStxt[i];                                    // read longditude field
               charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==66u)                                              // =====GNS======= message
             {                                                                  // field 5 of the GNS e/w indicator
               if (charIdx <= sizeof(ewGPS))
                 ewGPS[charIdx]=GPStxt[i];                                      // read east west field
               charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==67u)                                              // =====GNS======= message
             {                                                                  // field 6 of the GNS message position mode
                if (charIdx <= sizeof(posMode))
                   posMode[charIdx]=GPStxt[i];                                  // mode
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==68u)                                              // =====GNS======= message
             {                                                                  // field 7 of the GNS message depending on number of satelites
                if (charIdx <= sizeof(numSatel))
                   numSatel[charIdx]=GPStxt[i];                                 // number of satelites
                charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==69u)                                              // =====GNS======= message
             {                                                                  // field 8 of the GNS message horizontal dilution
               if (charIdx <= sizeof(HDOPGPS))
                  HDOPGPS[charIdx]=GPStxt[i];                                   // read HDOP, Horizontal Dilution of Precision
               charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==70u)                                              // =====GNS======= message
             {                                                                  // field 9 of the GNS message sea level altitude (meters)
               if (charIdx <= sizeof(seaLevelAlt))
                  seaLevelAlt[charIdx]=GPStxt[i];                               // read sea level altitude (meters)
               charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==71u)                                              // =====GNS======= message
             {                                                                  // field 10 of the GNS message Geoidal height
               if (charIdx <= sizeof(geoHt))
                  geoHt[charIdx]=GPStxt[i];                                     // Geoidal height (difference between ellipsoid and mean sea level)
               charIdx=++charIdx % UINT8_MAX;
             }
             else if (msgIdx==72u)                                              // =====GNS======= message
             {                                                                  // field 13 (skipped 2 blank fields) of the GNS message Navigation Status
                if (charIdx <= sizeof(navStatus))
                   navStatus[charIdx]=GPStxt[i];                                // navigation status string
                charIdx=++charIdx % UINT8_MAX;
             }
          }
          else if (((((msgIdx==56u) || (msgIdx==138u)) || (msgIdx==73u)) || (msgIdx==92u)) || (msgIdx==121u))      // ==== Collect the checksum after a * char =============
          {                                                                     // seen a * so collect the checksum from the end
             if (charIdx <= sizeof(csGPS))
               csGPS[charIdx]=GPStxt[i];                                        // read hexidecimal characture representation of the checksum
             charIdx=++charIdx % UINT8_MAX;
          }
          break;
       }
       i=++i % UINT8_MAX;                                                       // increment index and read the next char into receive buffer at position i
       if (((((((msgIdx >= 5u) && (msgIdx <= 25u)) || ((msgIdx >= 42u) && (msgIdx <= 55u))) || ((msgIdx >= 62u) && (msgIdx <= 72u))) || ((msgIdx >= 101u) && (msgIdx <= 120u))) || ((msgIdx >= 131u) && (msgIdx <= 137u))) || ((msgIdx >= 82u) && (msgIdx <= 91u)))
          checkCalc^=GPStxt[i];                                                 // Online Checksum (XOR) as we collect each char from start of data to before the checksum word in the message string (2 bytes)
     } // ------------ End of UART3_Data_ready()
  } // --------------- End of main U3RXIF_bit & msgIdx!=255
  else if ((msgIdx==255u) && U3RXIF_bit)                                         // === DORMANT STEP WAIT FOR A NEW START ==== Integrity error found look for a new start do nothing else
  {
    i=1;
    if (UART3_Data_Ready())                                                     // ready the receieve ?
    {
       GPStxt[i] = UART3_Read();                                                // Read the char from UART 3
       if (GPStxt[i]=='$')                                                      // found a start then collect a new message
       {
          msgIdx=1u;                                                            // new start
          checkCalc=0u;                                                         // reset checksum
          g_GPS_ready=0u;                                                       // inform top level new collection taking place
       }
    }
  }
  else
  {
     i=1;                                                                       // initialise i if value is obscure or out of range
  }

  if (msgIdx==251u)                                                             // at the end of a valid message above CRLF sequence completed
  {
    i=1;                                                                        // reset message collection
    if (checkCalc == csGPSPUBX)                                                 // Online checksum matches the data checksum sent so say the data is valid to display or use
    {
       g_GPS_ready = 1u;                                                         // Global represents we read all the NMEA code message from the GPS and its ready for processing
       g_posDataGPS.timestamp =  timeGPSPUBX;                                   // Write the data to the global object for display or control
       g_posDataGPS.lat = latGPSPUBX;
       g_posDataGPS.longd = longGPSPUBX;
       g_posDataGPS.altRef = altRefGPSPUBX;
       g_posDataGPS.hAcc = hAccGPSPUBX;
       g_posDataGPS.vAcc = vAccGPSPUBX;
       g_posDataGPS.SOG =  SOGGPSPUBX;
       g_posDataGPS.COG = COGGPSPUBX;
       g_posDataGPS.vVel = vVelGPSPUBX;
       g_posDataGPS.numOfSatel = numOfSatel;
       g_posDataGPS.SealevelAltitude = SealevelAltitude;
       g_posDataGPS.GeoidalHt = GeoidalHt;
       g_posDataGPS.hdop = HDOPGPSPUBX;
       g_posDataGPS.vdop = VDOPGPSPUBX;
       g_GPS_ready = 2u;                                                        /* data has been transferred to gps object */
    }
  }
  else if (i >= MAX_GPS_MSG_LEN)                                                // longer than the max or you got a CRLF sequence
  {
     i=1;                                                                       // reset message collection
     g_GPS_ready = 0u;
  }
  U3RXIF_bit=CLEAR;                                                             // Clear interrupt as it was read
}
#endif                                                                          // END OF GPS INCLUSION -----------------------------------

// ------------------- XS Encoder Serial  --------------------------------------

#ifdef UART4_INTERUPT                                                           // This is set if we are enabling UART4 with XS encoder serial

/*-----------------------------------------------------------------------------
 *      UART4_interrupt() :  This interrupts on chars that are recieved at UART4 serial port
 *  It will process these if they are supported XS Encoder commands
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void UART4_interrupt() iv IVT_UART_4 ilevel 4 ics ICS_AUTO
{
   asm EI;                                                                      // Re-enable global interrupts
   UART4.Bytes_In=++UART4.Bytes_In % UINT16_MAX;                                               // Global counter on Bytes arriving at UART4

   if((U4STA & 0x0Eu) | U4EIF_bit)                                              // Error check with UART4 buffer ???
   {
      if (OERR_U4STA_bit==1u)                                                   // Overrun error (bit 1)
      {
        OERR_U4STA_bit = 0u;
        UART4.Buffer_Overrun=++UART4.Buffer_Overrun % UINT16_MAX;               // Count overruns
      }
      else if (FERR_U4STA_bit==1u)                                              // Framing error (bit 2)
      {
        FERR_U4STA_bit = 0u;
        UART4.Framing_Error=++UART4.Framing_Error % UINT16_MAX;
      }
      else if (PERR_U4STA_bit==1u)                                              // Parity error (bit 3)
      {
        PERR_U4STA_bit = 0u;
        UART4.Parity_Error=++UART4.Parity_Error % UINT16_MAX;
      }
      
      UART4.Index =0u;                                                          // Set the index to zero
      U4STA &= 0xFFFFFFF1ul;                                                    //Clear the Error Status 0001 no longer 1101 (D) just the overrun , clears frame and parity
      U4EIF_bit =CLEAR;                                                         //Clear the Error IF
      U4RXIF_bit=CLEAR;                                                         //Clear the UART3 IF
   }

   if ((U4RXIF_bit) && (UART4.State < UART4_PACKET_IN_BUFFER))                  /* we received and we didnt have a full packet unprocessed */
   {
      if ((!XSstart) || XStypeMsgInvalid)                                       // if we dont already have a STX start byte or we got an invalid msg_id or checksum
      {
        UART4.Buffer[0u] = U4RXREG;
        if(UART4.Buffer[0u] == XS_CMD_REPLY_STX)                                // Look for the XS serial reply to a message start transmission (+) 0x2B
         {
            //T2CON = ENABLE_T2;       // Enable Timeout
            UART4.Index=1u;
            XSstart =TRUE;                                                      // valid response has been seen
            g_extended=0u;                                                      // extended +ok,<field> response is set to off mode
            XStypeMsgInvalid=0u;                                                // Set the message type to valid unless we dont get a valid type
            UART4.State = UART4_BYTE_IN_BUFFER;
         }
      }
      else
      {
         while(URXDA_U4STA_bit)                                                 // if there is a char to receive
         {
            UART4.Buffer[UART4.Index] = U4RXREG;                                // put the char from the UART4 into the buffer
            switch(UART4.Index)                                                 // For each characture in the response message do.....
            {
               case 1u:                                                         // Identifies the 2nd char which defines the reply response message
               switch(UART4.Buffer[1u])                                         // For the 2nd byte position in the incoming message see what you got
               {
                 case 'O':                                                      // command was a XS response +OK
                   XSlen=3u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'A':                                                      // command was a XS response +AUTH
                   XSlen=5u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'o':                                                      // command was a XS response +ok
                   XSlen=3u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'a':                                                      // command was a XS response +auth
                   XSlen=5u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'I':                                                      // command was a XS response +INVALID
                   XSlen=8u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'i':                                                      // command was a XS response +invalid
                   XSlen=8u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'F':                                                      // command was a XS response +FAIL
                   XSlen=5u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'f':                                                      // command was a XS response +fail
                   XSlen=5u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'W':                                                      // command was a XS response +WRONG
                   XSlen=6u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'w':                                                      // command was a XS response +wrong
                   XSlen=6u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'D':                                                      // command was a XS response +DISKSTATUS,val1,val2,val3
                   XSlen=11u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'd':                                                      // command was a XS response +diskstatus,val1,val2,val3
                   XSlen=11u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 'S':                                                      // command was a XS response +SYSSTATUS,val1,val2,val3,val4,val5,val6
                   XSlen=10u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 case 's':                                                      // command was a XS response +sysstatus,val1,val2,val3,val4,val5,val6
                   XSlen=10u;
                   UART4.Index=++UART4.Index % UINT16_MAX;
                   break;
                 default:                                                       // Data preceding the + char was not recognised
                   XStypeMsgInvalid=1u;
                   UART4.Index=0u;                                              // Restart message collection its not an expected response
                   break;
               }
               break;

               default:                                                         // Should be payload - reset if you got snother STX or payload is too long
               if (UART4.Buffer[UART4.Index] == XS_CMD_REPLY_STX)               // another STX (+) was found this is invalid go back to start
               {
                  UART4.Buffer[0u]= UART4.Buffer[UART4.Index];                  // reset to the start of the message buffer.
                  UART4.Index=0u;                                               // reset to put the rest of the message from index 1
                  g_extended=0u;                                                // extended +ok,<field> response is set to off mode
               }
               else if (UART4.Index >= (XSlen+1u))                              // message is now at the end either <CR> or a +ok,<value>
               {
                  if (UART4.Buffer[UART4.Index]==0x0Du)                         // CR is END of message else try (0x04 EOT)
                  {
                     UART4.Index=++UART4.Index % UINT16_MAX;                    // increment the packets in counter
                     XSstart=FALSE;                                             // Complete message collected start again

                     if (!XStypeMsgInvalid)                                     // Message is good and supported
                     {
                         UART4.State = UART4_PACKET_IN_BUFFER+g_extended;       // Declare good packet to top level (otherwise ignore it)
                     }
                  }
                  else if (((XSlen == 3u) || (XSlen == 11u)) || (XSlen == 10u)) // it may be either +ok +sysstatus or +diskstatus which can be followed legally by a comma
                  {
                     if (UART4.Buffer[UART4.Index]==',')                        // We got a comma set a global bit to explain it has extra extended values to extract from it
                     {
                        g_extended=XSlen;                                       // set a global bit to say we extended using a , separater longer message found
                     }
                     UART4.Index=++UART4.Index % UINT16_MAX;                    // keep collecting the extended message separated by comma's before completing the collection
                  }
                  else
                  {
                    XStypeMsgInvalid=1u;                                        // it was invalid start collecting it again
                    UART4.Index=0u;                                             // Restart message collection
                  }
               }
               else
               {
                  UART4.Index=++UART4.Index % UINT16_MAX;                       // increment and fill the buffer with data from the payload
               }
               break;
            }
         }
      }
    } // End U2RXIFBIT
  U4RXIF_bit=CLEAR;

} // End Interrupt
#endif

#ifdef UART6_INTERUPT                                                           // This is set if we are enabling UART1 with Liddar
/*-----------------------------------------------------------------------------
 *      UART6_interrupt() :  This interrupts on chars that are recieved at UART6 serial port
 *  It will process these if they are supported by lightware commands
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void UART6_interrupt() iv IVT_UART_6 ilevel 6 ics ICS_AUTO
{
   asm EI;                                                                      // Reenable global interrupts
   if((U6STA & 0x0E) | U6EIF_bit)                                               // Error check with UART6 buffer ???
   {
      if (OERR_U6STA_bit==1u)                                                   // Overrun error (bit 1)
      {
        OERR_U6STA_bit = 0u;
        UART6.Buffer_Overrun=++UART6.Buffer_Overrun % UINT16_MAX;               /* Count overruns */
      }
      else if (FERR_U6STA_bit==1u)                                              // Framing error (bit 2)
      {
        FERR_U6STA_bit = 0u;
        UART6.Framing_Error=++UART6.Framing_Error % UINT16_MAX;                 /* Count framing errors */
      }
      else if (PERR_U6STA_bit==1u)                                              // Parity error (bit 3)
      {
        PERR_U6STA_bit = 0u;
        UART6.Parity_Error=++UART6.Parity_Error % UINT16_MAX;                   /* Count parity errors */
      }
      
      /* UART6.Index =0u;                                                       Set the index to zero no need to do as its done by Response->parseState */
      U6STA &= 0xFFFFFFF1UL;                                                    //Clear the Error Status 0001 no longer 1101 (D) just the overrun , clears frame and parity
      U6EIF_bit =CLEAR;                                                         //Clear the Error IF
      U6RXIF_bit=CLEAR;                                                         //Clear the UART3 IF
   }
   if (U6RXIF_bit)                                                              /* a bit came in to receieve */
   {
      UART6.State=lwnxParseData( &LwNxResponse, U6RXREG );                      /* parse the incoming bytes and feedback the result in UART6.State */
   }
   U6RXIF_bit=CLEAR;                                                            // clear the interrupt
}
#endif
#if defined(UART1_INTERUPT)
/*-----------------------------------------------------------------------------
 *      UART1_interrupt() :  This interrupts on chars that are recieved at UART6 serial port
 *  It will process these if they are supported by odrive commands
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void UART1_interrupt() iv IVT_UART_1 ilevel 7 ics ICS_AUTO 
{
   asm EI;                                                                      // Reenable global interrupts
#ifdef outout_use_if_another_uart_for_this                                      // UART1 doesnt seem to have this ???
   if((U1STA & 0x0E) | U1EIF_Bit)                                               // Error check with UART1 buffer ???
   {
      if (OERR_U1STA_bit==1u)                                                   // Overrun error (bit 1)
      {
        OERR_U1STA_bit = 0u;
        UART1.Buffer_Overrun=++UART1.Buffer_Overrun % UINT16_MAX;               // Count overruns
      }
      else if (FERR_U1STA_bit==1u)                                              // Framing error (bit 2)
      {
        FERR_U1STA_bit = 0u;
        UART1.Framing_Error=++UART1.Framing_Error % UINT16_MAX;
      }
      else if (PERR_U1STA_bit==1u)                                              // Parity error (bit 3)
      {
        PERR_U1STA_bit = 0u;
        UART1.Parity_Error=++UART1.Parity_Error % UINT16_MAX;
      }
      UART1.Index =0u;                                                          // Set the index to zero
      U1STA &= 0xFFFFFFF1UL;                                                    //Clear the Error Status 0001 no longer 1101 (D) just the overrun , clears frame and parity
      U1EIF_Bit =CLEAR;                                                         //Clear the Error IF
      U1RXIF_bit=CLEAR;                                                         //Clear the UART3 IF
   }
#endif

  if (UART1.State==UART1_BUFFER_EMPTY)                                          // new collection
  {
      UART1.Index=0u;                                                           // start at the first characture
  }

  if ((U1RXIF_Bit) && !(UART1.State==UART1_PACKET_IN_BUFFER))                   // char came into UART1 and we havent collected and unacknowledged packet
  {
//     while(URXDA_U5STA_bit)                                                     // if there is a char to receive
//     {
        UART1.Buffer[UART1.Index] = U1RXREG;                                    // put the char from the UART into the buffer
        UART1.State=UART1_BYTE_IN_BUFFER;                                       // bytes are being collected for this message
        switch (UART1.Index)
        {
           case 0u:
           if (UART1.Buffer[UART1.Index] == ODRIVE_STX)                         // we have seen the start characture
           {
              UART1.Index=++UART1.Index % UINT16_MAX;                           // increment and fill the buffer with data from the payload
           }
           break;

           case 1u:
           packet_len=UART1.Buffer[1u];
           UART1.Index=++UART1.Index % UINT16_MAX;                              // increment and fill the buffer with data from the payload
           break;

           default:
           if (UART1.Index >= packet_len)                                       // message too long - > expected
           {
              UART1.State==UART1_BUFFER_EMPTY;
              UART1.Index=0u;                                                   // Restart message collection
           }
           else if (UART1.Buffer[UART1.Index] == ODRIVE_STX)                    /* saw the stx again then assume it was a partial then start collecting a new packet */
           {
               UART1.Index=1u;
           }
           else if (UART1.Index == (packet_len-1u))                             // we have just collected the last byte check CRC if there is one ??
           {
              UART1.State = UART1_PACKET_IN_BUFFER;                             // set flag to say we collected the packet
           }
           else
           {
              UART1.Index=++UART1.Index % UINT16_MAX;                           // increment and fill the buffer with data from the payload
           }
           break;
        }
//     }
  }  // End U1RXIFBIT


  U1RXIF_bit=CLEAR;                                                             // clear the interrupt flag
}
#endif

/* ================ Interrupt handler for MAVLINK =========================== */
#if defined(SERIAL_MAVLINK)
#if defined(MAVLINK_USE_UART1)                                                  /* =========================== UART1 ======================================= */
#define UAMAV_VECTOR IVT_UART_1                                                 /* this method uses generic tags to replace for each interrupt it might be more readable to have them as the actual tag ? reason means less error this way */
#define UAMAVSTA U1STA
#define UAMAVRXIF_bit U1RXIF_bit
#define OERR_UAMAV_bit OERR_U1STA_bit
#define FERR_UAMAV_bit FERR_U1STA_bit
#define PERR_UAMAV_bit PERR_U1STA_bit
#define UAMAVEIF_Bit U1EIF_Bit
#define URXDA_UAMAV_bit URXDA_U1STA_bit
#define UAMAVRXREG U1RXREG
#define UAMAVRXIF_bit U1RXIF_bit
#include "mav_generic_interrupt.h"                                              /* substitute the above tags in this routine - not MISRA complient consider copying in here once testing complete for complience */
#undef UAMAV_VECTOR                                                             /* undef the tags used to make the function this is NON-MISRA consider making long hand each section or leave the tags defined (its just clean to remove them or allows you multiple use in interrupts) */
#undef UAMAVSTA
#undef UAMAVRXIF_bit
#undef OERR_UAMAV_bit
#undef FERR_UAMAV_bit
#undef PERR_UAMAV_bit
#undef UAMAVEIF_Bit
#undef URXDA_UAMAV_bit
#undef UAMAVRXREG
#undef UAMAVRXIF_bit
#elif defined(MAVLINK_USE_UART2)                                                /* =========================== UART2 ======================================= */
#define UAMAV_VECTOR IVT_UART_2                                                 /* this method uses generic tags to replace for each interrupt it might be more readable to have them as the actual tag ? reason means less error this way */
#define UAMAVSTA U2STA
#define UAMAVRXIF_bit U2RXIF_bit
#define OERR_UAMAV_bit OERR_U2STA_bit
#define FERR_UAMAV_bit FERR_U2STA_bit
#define PERR_UAMAV_bit PERR_U2STA_bit
#define UAMAVEIF_Bit U2EIF_Bit
#define URXDA_UAMAV_bit URXDA_U2STA_bit
#define UAMAVRXREG U2RXREG
#define UAMAVRXIF_bit U2RXIF_bit
#include "mav_generic_interrupt.h"                                              /* substitute the above tags in this routine - not MISRA complient consider copying in here once testing complete for complience */
#undef UAMAV_VECTOR                                                             /* undef the tags used to make the function this is NON-MISRA consider making long hand each section or leave the tags defined (its just clean to remove them or allows you multiple use in interrupts) */
#undef UAMAVSTA
#undef UAMAVRXIF_bit
#undef OERR_UAMAV_bit
#undef FERR_UAMAV_bit
#undef PERR_UAMAV_bit
#undef UAMAVEIF_Bit
#undef URXDA_UAMAV_bit
#undef UAMAVRXREG
#undef UAMAVRXIF_bit
#elif defined(MAVLINK_USE_UART3)                                                /* =========================== UART3 ======================================= */
#define UAMAV_VECTOR IVT_UART_3                                                 /* this method uses generic tags to replace for each interrupt it might be more readable to have them as the actual tag ? reason means less error this way */
#define UAMAVSTA U3STA
#define UAMAVRXIF_bit U3RXIF_bit
#define OERR_UAMAV_bit OERR_U3STA_bit
#define FERR_UAMAV_bit FERR_U3STA_bit
#define PERR_UAMAV_bit PERR_U3STA_bit
#define UAMAVEIF_Bit U3EIF_Bit
#define URXDA_UAMAV_bit URXDA_U3STA_bit
#define UAMAVRXREG U3RXREG
#define UAMAVRXIF_bit U3RXIF_bit
#include "mav_generic_interrupt.h"                                              /* substitute the above tags in this routine - not MISRA complient consider copying in here once testing complete for complience */
#undef UAMAV_VECTOR                                                             /* undef the tags used to make the function this is NON-MISRA consider making long hand each section or leave the tags defined (its just clean to remove them or allows you multiple use in interrupts) */
#undef UAMAVSTA
#undef UAMAVRXIF_bit
#undef OERR_UAMAV_bit
#undef FERR_UAMAV_bit
#undef PERR_UAMAV_bit
#undef UAMAVEIF_Bit
#undef URXDA_UAMAV_bit
#undef UAMAVRXREG
#undef UAMAVRXIF_bit
#elif defined(MAVLINK_USE_UART4)                                                /* =========================== UART4 ======================================= */
#define UAMAV_VECTOR IVT_UART_4                                                 /* this method uses generic tags to replace for each interrupt it might be more readable to have them as the actual tag ? reason means less error this way */
#define UAMAVSTA U4STA
#define UAMAVRXIF_bit U4RXIF_bit
#define OERR_UAMAV_bit OERR_U4STA_bit
#define FERR_UAMAV_bit FERR_U4STA_bit
#define PERR_UAMAV_bit PERR_U4STA_bit
#define UAMAVEIF_Bit U4EIF_Bit
#define URXDA_UAMAV_bit URXDA_U4STA_bit
#define UAMAVRXREG U4RXREG
#define UAMAVRXIF_bit U4RXIF_bit
#include "mav_generic_interrupt.h"                                              /* substitute the above tags in this routine - not MISRA complient consider copying in here once testing complete for complience */
#undef UAMAV_VECTOR                                                             /* undef the tags used to make the function this is NON-MISRA consider making long hand each section or leave the tags defined (its just clean to remove them or allows you multiple use in interrupts) */
#undef UAMAVSTA
#undef UAMAVRXIF_bit
#undef OERR_UAMAV_bit
#undef FERR_UAMAV_bit
#undef PERR_UAMAV_bit
#undef UAMAVEIF_Bit
#undef URXDA_UAMAV_bit
#undef UAMAVRXREG
#undef UAMAVRXIF_bit
#elif defined(MAVLINK_USE_UART5)                                                /* =========================== UART5 ======================================= */
#define UAMAV_VECTOR IVT_UART_5                                                 /* this method uses generic tags to replace for each interrupt it might be more readable to have them as the actual tag ? reason means less error this way */
#define UAMAVSTA U5STA
#define UAMAVRXIF_bit U5RXIF_bit
#define OERR_UAMAV_bit OERR_U5STA_bit
#define FERR_UAMAV_bit FERR_U5STA_bit
#define PERR_UAMAV_bit PERR_U5STA_bit
#define UAMAVEIF_Bit U5EIF_Bit
#define URXDA_UAMAV_bit URXDA_U5STA_bit
#define UAMAVRXREG U5RXREG
#define UAMAVRXIF_bit U5RXIF_bit
#include "mav_generic_interrupt.h"                                              /* substitute the above tags in this routine - not MISRA complient consider copying in here once testing complete for complience */
#undef UAMAV_VECTOR                                                             /* undef the tags used to make the function this is NON-MISRA consider making long hand each section or leave the tags defined (its just clean to remove them or allows you multiple use in interrupts) */
#undef UAMAVSTA
#undef UAMAVRXIF_bit
#undef OERR_UAMAV_bit
#undef FERR_UAMAV_bit
#undef PERR_UAMAV_bit
#undef UAMAVEIF_Bit
#undef URXDA_UAMAV_bit
#undef UAMAVRXREG
#undef UAMAVRXIF_bit
#elif defined(MAVLINK_USE_UART6)                                                /* ============= UART6 left thia one without generic for now ========= */
void MAV_interrupt() iv IVT_UART_6 ilevel 7 ics ICS_AUTO                        /* interrupt on UART and store data in MAVLINK receive buffer */
{
   uint16_t CRCReadBack;
   uint16_t CRCCalc;
   uint8_t CRC_CKA=0u;
   uint8_t CRC_CKB=0u;

   asm EI;                                                                      /* Reenable global interrupts */
#ifndef MAVLINK_USE_UART1                                                       /* UART1 doesnt seem to have this ???  */
   if((U6STA & 0x0Eu) | U6EIF_Bit)                                              /* Error check with UART buffer */
   {
      if (OERR_U6STA_bit==1u)                                                   /* Overrun error (bit 1) */
      {
        OERR_U6STA_bit = 0u;
        MavLinkBuf.Buffer_Overrun=++MavLinkBuf.Buffer_Overrun % UINT16_MAX;      /* Count overruns  */
      }
      else if (FERR_U6STA_bit==1u)                                              /* Framing error (bit 2) */
      {
        FERR_U6STA_bit = 0u;
        MavLinkBuf.Framing_Error=++MavLinkBuf.Framing_Error % UINT16_MAX;
      }
      else if (PERR_U6STA_bit==1u)                                              /* Parity error (bit 3)    */
      {
        PERR_U6STA_bit = 0u;
        MavLinkBuf.Parity_Error=++MavLinkBuf.Parity_Error % UINT16_MAX;
      }
      MavLinkBuf.Index = MAV_LOOK_FOR_START;                                    /* Set the index to zero to recollect by looking for a new message start */
      U6STA &= 0xFFFFFFF1UL;                                                    /* Clear the Error Status 0001 no longer 1101 (D) just the overrun , clears frame and parity  */
      U6EIF_Bit =CLEAR;                                                         /* Clear the Error IF */
      U6RXIF_bit=CLEAR;                                                         /* Clear the UART IF */
   }
#endif


  if (MavLinkBuf.State==UAMAV_BUFFER_EMPTY)                                     /* new collection */
  {
      MavLinkBuf.Index=MAV_LOOK_FOR_START;                                      /* reset the buffer index */
  }

  if ((U6RXIF_Bit) && !(MavLinkBuf.State==UAMAV_PACKET_IN_BUFFER))              /* a byte was received by the UART and we didnt have a full frame unprocessed */
  {
#ifndef MAVLINK_USE_UART1
     while(URXDA_U6STA_bit)                                                     /* while there is a byte to receive */
     {
#endif  /* ============================= end if_UART1 */
        MavLinkBuf.Buffer[MavLinkBuf.Index] = U6RXREG;                          /* put the byte from the UART into the buffer */
#include "mav_uart_driver_pic32_common.h"                                       /* common functionality to the receive driver NON-MISRA consider hard coding from file after entire unit/boundary/function tests */
#ifndef MAVLINK_USE_UART1
     }
#endif  /* ============================= end if_UART6  */
  }  /* End U_RXIFBIT  */

  U6RXIF_bit=CLEAR;                                                             /* clear the interrupt flag check when packet has been receievd and not processed if we need to remove then or not we want to keep in buffer */
}
#endif /* end ============= use UART for mavlink */
#endif /* end ============= serial mavlink */

/* -------- $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ ---------------------- */
/*                                                                */
/* -------- Navigation Navlock on any UART ---------------------- */
/*                                                                */
/* -------- $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ ---------------------- */
#if defined(SERIAL_NAVLINK)
#define NavOBject g_NavObject                                                   /* global object for controling the link */
#if defined(NAVLINK_USE_UART1)                                                  /* =========================== UART1 ======================================= */
#define UANAV_VECTOR IVT_UART_1                                                 /* this method uses generic tags to replace for each interrupt it might be more readable to have them as the actual tag ? reason means less error this way */
#define UANAVSTA U1STA
#define UANAVRXIF_bit U1RXIF_bit
#define OERR_UANAV_bit OERR_U1STA_bit
#define FERR_UANAV_bit FERR_U1STA_bit
#define PERR_UANAV_bit PERR_U1STA_bit
#define UANAVEIF_Bit U1EIF_Bit
#define URXDA_UANAV_bit URXDA_U1STA_bit
#define UANAVRXREG U1RXREG
#define UANAVRXIF_bit U1RXIF_bit
#include "NAV_generic_interrupt.h"                                              /* substitute the above tags in this routine - not MISRA complient consider copying in here once testing complete for complience */
#undef UANAV_VECTOR                                                             /* undef the tags used to make the function this is NON-MISRA consider making long hand each section or leave the tags defined (its just clean to remove them or allows you multiple use in interrupts) */
#undef UANAVSTA
#undef UANAVRXIF_bit
#undef OERR_UANAV_bit
#undef FERR_UANAV_bit
#undef PERR_UANAV_bit
#undef UANAVEIF_Bit
#undef URXDA_UANAV_bit
#undef UANAVRXREG
#undef UANAVRXIF_bit
#elif defined(NAVLINK_USE_UART2)                                                /* =========================== UART2 ======================================= */
#define UANAV_VECTOR IVT_UART_2                                                 /* this method uses generic tags to replace for each interrupt it might be more readable to have them as the actual tag ? reason means less error this way */
#define UANAVSTA U2STA
#define UANAVRXIF_bit U2RXIF_bit
#define OERR_UANAV_bit OERR_U2STA_bit
#define FERR_UANAV_bit FERR_U2STA_bit
#define PERR_UANAV_bit PERR_U2STA_bit
#define UANAVEIF_Bit U2EIF_Bit
#define URXDA_UANAV_bit URXDA_U2STA_bit
#define UANAVRXREG U2RXREG
#define UANAVRXIF_bit U2RXIF_bit
#include "NAV_generic_interrupt.h"                                              /* substitute the above tags in this routine - not MISRA complient consider copying in here once testing complete for complience */
#undef UANAV_VECTOR                                                             /* undef the tags used to make the function this is NON-MISRA consider making long hand each section or leave the tags defined (its just clean to remove them or allows you multiple use in interrupts) */
#undef UANAVSTA
#undef UANAVRXIF_bit
#undef OERR_UANAV_bit
#undef FERR_UANAV_bit
#undef PERR_UANAV_bit
#undef UANAVEIF_Bit
#undef URXDA_UANAV_bit
#undef UANAVRXREG
#undef UANAVRXIF_bit
#elif defined(NAVLINK_USE_UART3)                                                /* =========================== UART3 ======================================= */
#define UANAV_VECTOR IVT_UART_3                                                 /* this method uses generic tags to replace for each interrupt it might be more readable to have them as the actual tag ? reason means less error this way */
#define UANAVSTA U3STA
#define UANAVRXIF_bit U3RXIF_bit
#define OERR_UANAV_bit OERR_U3STA_bit
#define FERR_UANAV_bit FERR_U3STA_bit
#define PERR_UANAV_bit PERR_U3STA_bit
#define UANAVEIF_Bit U3EIF_Bit
#define URXDA_UANAV_bit URXDA_U3STA_bit
#define UANAVRXREG U3RXREG
#define UANAVRXIF_bit U3RXIF_bit
#include "NAV_generic_interrupt.h"                                              /* substitute the above tags in this routine - not MISRA complient consider copying in here once testing complete for complience */
#undef UANAV_VECTOR                                                             /* undef the tags used to make the function this is NON-MISRA consider making long hand each section or leave the tags defined (its just clean to remove them or allows you multiple use in interrupts) */
#undef UANAVSTA
#undef UANAVRXIF_bit
#undef OERR_UANAV_bit
#undef FERR_UANAV_bit
#undef PERR_UANAV_bit
#undef UANAVEIF_Bit
#undef URXDA_UANAV_bit
#undef UANAVRXREG
#undef UANAVRXIF_bit
#elif defined(NAVLINK_USE_UART4)                                                /* =========================== UART4 ======================================= */
#define UANAV_VECTOR IVT_UART_4                                                 /* this method uses generic tags to replace for each interrupt it might be more readable to have them as the actual tag ? reason means less error this way */
#define UANAVSTA U4STA
#define UANAVRXIF_bit U4RXIF_bit
#define OERR_UANAV_bit OERR_U4STA_bit
#define FERR_UANAV_bit FERR_U4STA_bit
#define PERR_UANAV_bit PERR_U4STA_bit
#define UANAVEIF_Bit U4EIF_Bit
#define URXDA_UANAV_bit URXDA_U4STA_bit
#define UANAVRXREG U4RXREG
#define UANAVRXIF_bit U4RXIF_bit
#include "NAV_generic_interrupt.h"                                              /* substitute the above tags in this routine - not MISRA complient consider copying in here once testing complete for complience */
#undef UANAV_VECTOR                                                             /* undef the tags used to make the function this is NON-MISRA consider making long hand each section or leave the tags defined (its just clean to remove them or allows you multiple use in interrupts) */
#undef UANAVSTA
#undef UANAVRXIF_bit
#undef OERR_UANAV_bit
#undef FERR_UANAV_bit
#undef PERR_UANAV_bit
#undef UANAVEIF_Bit
#undef URXDA_UANAV_bit
#undef UANAVRXREG
#undef UANAVRXIF_bit
#elif defined(NAVLINK_USE_UART5)                                                /* =========================== UART5 ======================================= */
#define UANAV_VECTOR IVT_UART_5                                                 /* this method uses generic tags to replace for each interrupt it might be more readable to have them as the actual tag ? reason means less error this way */
#define UANAVSTA U5STA
#define UANAVRXIF_bit U5RXIF_bit
#define OERR_UANAV_bit OERR_U5STA_bit
#define FERR_UANAV_bit FERR_U5STA_bit
#define PERR_UANAV_bit PERR_U5STA_bit
#define UANAVEIF_Bit U5EIF_Bit
#define URXDA_UANAV_bit URXDA_U5STA_bit
#define UANAVRXREG U5RXREG
#define UANAVRXIF_bit U5RXIF_bit
#include "NAV_generic_interrupt.h"                                              /* substitute the above tags in this routine - not MISRA complient consider copying in here once testing complete for complience */
#undef UANAV_VECTOR                                                             /* undef the tags used to make the function this is NON-MISRA consider making long hand each section or leave the tags defined (its just clean to remove them or allows you multiple use in interrupts) */
#undef UANAVSTA
#undef UANAVRXIF_bit
#undef OERR_UANAV_bit
#undef FERR_UANAV_bit
#undef PERR_UANAV_bit
#undef UANAVEIF_Bit
#undef URXDA_UANAV_bit
#undef UANAVRXREG
#undef UANAVRXIF_bit
#elif defined(NAVLINK_USE_UART6)       
#define UANAV_VECTOR IVT_UART_6                                                 /* this method uses generic tags to replace for each interrupt it might be more readable to have them as the actual tag ? reason means less error this way */
#define UANAVSTA U6STA
#define UANAVRXIF_bit U6RXIF_bit
#define OERR_UANAV_bit OERR_U6STA_bit
#define FERR_UANAV_bit FERR_U6STA_bit
#define PERR_UANAV_bit PERR_U6STA_bit
#define UANAVEIF_Bit U6EIF_Bit
#define URXDA_UANAV_bit URXDA_U6STA_bit
#define UANAVRXREG U6RXREG
#define UANAVRXIF_bit U6RXIF_bit
#include "NAV_generic_interrupt.h"                                              /* substitute the above tags in this routine - not MISRA complient consider copying in here once testing complete for complience */
#undef UANAV_VECTOR                                                             /* undef the tags used to make the function this is NON-MISRA consider making long hand each section or leave the tags defined (its just clean to remove them or allows you multiple use in interrupts) */
#undef UANAVSTA
#undef UANAVRXIF_bit
#undef OERR_UANAV_bit
#undef FERR_UANAV_bit
#undef PERR_UANAV_bit
#undef UANAVEIF_Bit
#undef URXDA_UANAV_bit
#undef UANAVRXREG
#undef UANAVRXIF_bit
#endif /* end ============= use UART for link */
#endif /* end ============= serial navlink */

/*-----------------------------------------------------------------------------
 *      coreTimer():  interrupt when a core tick preset count has been met
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void coreTimer() iv IVT_CORE_TIMER ilevel 2 ics ICS_AUTO                        /* interrupt when tick counter equals CP0_COMPARE in co-processor set to 8M - 1 second */
{
   asm EI;                                                                      /* re-enable all interrupt */
   IFS0CLR=0x0001u;                                                             /* clear the flag */
   /* same    CTIF_bit=CLEAR */
   
#if (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY))
   g_SBGCmsgCollect= g_SBGCmsgCollect & ~(g_SBGCmsgCollect & SBGCmsgCollectLast);     /* take out any flags that we set true for more than 2 ticks presets (2 seconds) */
   SBGCmsgCollectLast= g_SBGCmsgCollect;
#endif

#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
   /* compare interrupt and if you have waited in the state already advance to next state in the ADC engine */
   if ((g_adcState==WAIT_ADC_SAM_TIM) && (prevAdcState==WAIT_ADC_SAM_TIM))      /* waited at least one compare time (1s) for ADC sample collection from SFR */
   {
      g_adcState=WAIT_FOR_SAMPLE;                                               /* now turn the sampling off and wait for the complete signal from the ADC */
   }
   else if ((g_adcState==WAIT_FOR_SAMPLE) && (prevAdcState==WAIT_FOR_SAMPLE))   /* waited too long for ADC sample collection from SFR to indicate complete */
   {
      g_adcState=COLLECT_ADC_TIMEOUT;                                           /* error the collected sample value and continue */
   }
   else {}                                                                      /* do nothing */
   prevAdcState=g_adcState;
#endif

   CP0_SET(CP0_COUNT,0UL);                                                      /* clear the tick counter */
   CP0_SET(CP0_COMPARE,40000000UL);                                             /* set the compare set-point each time to 80M */
}
/*-----------------------------------------------------------------------------
 *      Io0_Interrupt():  interrupt when digital io state transition to true
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void void Io0_InterruptESD() iv IVT_EXTERNAL_0 ilevel 7 ics ICS_AUTO            /* interrupt on D0/INT0 digital pin for example emergency stop / release */
{
   asm EI;
#if defined(EXTENDED_ARM)
   AuxDevState = (AuxDevState & (~extArmRev)) & (~extArmFwd);                   /* stop the sprayer arm reversing or going forward */
   DeviceState = (DeviceState & ~(Valve5Open));                                 /* stop spraying */
   g_hmiResetESD1=true;                                                         /* indicate lockout to the screen (used to disble the drive on the shaft) */
#endif
   IFS0CLR=INT0_ENAB;
   INT0IF_bit=CLEAR;                                                            /* sets bit 3 in the word */
}
/*-----------------------------------------------------------------------------
 *      Io1_BatchTk_Low():  interrupt when first tank is low (NC relay in series)
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void Io1_BatchTk_Low() iv IVT_EXTERNAL_1 ilevel 7 ics ICS_SRS
{
   asm EI;
   g_saveBatchState=g_batchState;                                               /* remember last known plant state */
   g_saveDeviceState= DeviceState & BANK1_IO_ESD_MASK;                          /* remember the selected io to be changed in this interrupt */
   DeviceState= DeviceState & ~(BANK1_IO_ESD_MASK);                             /* remove the selected io to be changed in this interrupt */
   g_batchState=BATCH_HOLD_STATE;                                               /* go to hold state */
   /* saves the timer */
   CLR_PRIO_IO1_INTER(5u,1u);                                                   /* inhibit this interrupt until re-activated from the sequence */
   SET_PRIO_IO2_INTER(5u,1u);                                                   /* now look for the high level on the supply tank */
#if defined(PULSE_TACHO)
   g_fromInter= 1u;                                                             /* when we get interlock 2 tell it to re-enable low level from tank 1 io1 interrupt */
#endif
   IFS0CLR=INT1_ENAB;                                                           /* clear the latch of this interrupt state */
   INT1IF_bit=CLEAR;
}
/*-----------------------------------------------------------------------------
 *      Io2_BatchTk_Full():  interrupt when first tank have enough (NC relay in series)
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void Io2_BatchTk_Full() iv IVT_EXTERNAL_2 ilevel 7 ics ICS_SRS
{
   asm EI;
   /* re-init the timer and add the bit already used, restore the step making sure the action state returns */
   if ((((int64_t)g_timer5_counter)-elapsedTime) > 0)                           /* no rollover occurred */
      lastBatchTime=g_timer5_counter-((uint64_t)elapsedTime);                   /* make the lastBatchTime include what we elapsed before interrupt but with a new start point (minus the interrupt delay) */
   else
      lastBatchTime=(UINT64_MAX - (uint64_t)elapsedTime)+g_timer5_counter;      /* under rollover make the lastBatchTime include what we elapsed before interrupt but with a new start point (minus the interrupt delay) */
   DeviceState= DeviceState | g_saveDeviceState;                                /* re-instate the selected io to be changed in this interrupt */
   g_batchState= g_saveBatchState;                                              /* return the state machine to the previous state before Io1 interrupt occurred */
#if defined(PULSE_TACHO)
   switch(g_fromInter)
   {
       case 1u:                                                                 /* came from i/o interlock 1 */
       SET_PRIO_IO1_INTER(5u,1u);                                               /* now look for the low level on the supply tank */
       g_fromInter= 1u;
       break;
       
       case 3u:                                                                 /* came from i/o interlock 3 */
       SET_PRIO_IO3_INTER(5u,1u);                                               /* now look for the low level on the supply tank */
       g_fromInter= 1u;
       break;
       
       default:                                                                 /* hang here to see why we got this ? */
       break;
   }
#else
   SET_PRIO_IO1_INTER(5u,1u);                                                   /* now look for the low level on the supply tank */
#endif
   CLR_PRIO_IO2_INTER(5u,1u);                                                   /* inhibit this interrupt until re-activated from the sequence */
   IFS0CLR=INT2_ENAB;
   INT2IF_bit=CLEAR;
}
/*-----------------------------------------------------------------------------
 *      Io3_BatchTk_Low():  interrupt when 2nd tank is low (NC relay in series)
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void Io3_BatchTk_Low() iv IVT_EXTERNAL_3 ilevel 7 ics ICS_SRS 
{
   asm EI;
   g_saveBatchState=g_batchState;                                               /* remember last known plant state */
   g_saveDeviceState= DeviceState & BANK1_IO_ESD_MASK;                          /* remember the selected io to be changed in this interrupt */
   DeviceState= DeviceState & ~(BANK1_IO_ESD_MASK);                             /* remove the selected io to be changed in this interrupt */
   g_batchState=BATCH_HOLD_STATE;                                               /* go to hold state */
   /* saves the timer */
   CLR_PRIO_IO3_INTER(5u,1u);                                                   /* inhibit this interrupt until re-activated from the sequence */
#if defined(PULSE_TACHO)
   SET_PRIO_IO2_INTER(5u,1u);                                                   /* now look for the high level on the commoned supply tanks */
   g_fromInter= 3u;                                                             /* tell the global high interrupt which low level to return to e.g. tank 2 here */
#else
   SET_PRIO_IO4_INTER(5u,1u);                                                   /* now look for the high level on the supply tank */
#endif
   IFS0CLR=INT3_ENAB;
   INT3IF_bit=CLEAR;
}
/*-----------------------------------------------------------------------------
 *      Io4_BatchTk_Full():  interrupt when second tank has enough (NC relay in series)
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void Io4_BatchTk_Full() iv IVT_EXTERNAL_4 ilevel 7 ics ICS_SRS
{
   asm EI;
#if defined(PULSE_TACHO)
/* --- tacho code to calcualte speed distance between rotation sensors and time taken --- */
   if (g_globalStartUp==0u)                                                     /* first time ensure we get a zero reading */
   {
      g_TachoObj1.time2Pulse = -1;                                              /* initialise tick timer */
      g_globalStartUp=1u;                                                       /* set init to done */
   }
   else if (g_globalStartUp==1u)                                                /* done once */
   {
      g_globalStartUp=2u;                                                       /* set velocity to done */
   }
   else if (g_globalStartUp==2u)                                                /* done once */
   {
      g_globalStartUp=3u;                                                       /* set acceleration to done */
   }
   
   calculateTick2Now( &g_TachoObj1.time2Pulse, &g_TachoObj1.time2PulseLast );   /* get the ticks since last pulse rising edge input */
   g_TachoObj1.rot_speed = (ROT_SENS_DIST / (((float32_t)g_TachoObj1.time2Pulse)/((float32_t)(CORE_TICK_SECOND)))); /* calculate the speed in distance/s */
   g_TachoObj1.rot_count = ++g_TachoObj1.rot_count %UINT64_MAX;                 /* count pulses distance */
   g_TachoObj1.distance = g_TachoObj1.rot_count * ROT_SENS_DIST;                /* multiply the pulse counts by the distance to sensor on the wheel to give us distance travelled */
   if (g_globalStartUp==3u)                                                     /* third iteration allows us to calculate the acceleration */
   {
      if (((UINT64_MAX - g_TachoObj1.rot_count) > UINT64_MAX/2ULL) && !((UINT64_MAX - g_TachoObj1.rot_count_last) > UINT64_MAX/2ULL)) /* we rolled over acceleration */
      {
         g_TachoObj1.accel = (g_TachoObj1.rot_count + (UINT64_MAX -g_TachoObj1.rot_count_last)) * ROT_SENS_DIST; /* acceleration after rolover */
      }
      else
      {
         g_TachoObj1.accel = (g_TachoObj1.rot_count - g_TachoObj1.rot_count_last) * ROT_SENS_DIST; /* acceleration without rollover  */
      }
   }
   g_TachoObj1.rot_count_last= g_TachoObj1.rot_count;                           /* save last rotation count in order to calculate the acceleration */
#else
   /* re-init the timer and add the bit already used, restore the step making sure the action state returns */
   if ((((int64_t)g_timer5_counter)-elapsedTime) > 0)                           /* no rollover occurred */
      lastBatchTime=g_timer5_counter-((uint64_t)elapsedTime);                   /* make the lastBatchTime include what we elapsed before interrupt but with a new start point (minus the interrupt delay) */
   else
      lastBatchTime=(UINT64_MAX - (uint64_t)elapsedTime)+g_timer5_counter;      /* under rollover make the lastBatchTime include what we elapsed before interrupt but with a new start point (minus the interrupt delay) */
   DeviceState= DeviceState | g_saveDeviceState;                                /* re-instate the selected io to be changed in this interrupt */
   g_BatchState= g_saveBatchState;                                              /* return the state machine to the previous state before Io1 interrupt occurred */
   SET_PRIO_IO3_INTER(5u,1u);                                                   /* now look for the low level on the supply tank */
   CLR_PRIO_IO4_INTER(5u,1u);                                                   /* inhibit this interrupt until re-activated from the sequence */
#endif

   IFS0CLR=INT4_ENAB;
   INT4IF_bit=CLEAR;
}
/*-----------------------------------------------------------------------------
 *      ChangeNotice():  interrupt when the digital inputs of change notice io change state
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void ChangeNotice() iv IVT_CHANGE_NOTICE ilevel 3 ics ICS_AUTO 
{
   /* IFS0CLR=0x0001u;                                                              clear the flag */
   CNIF_bit=CLEAR;
}