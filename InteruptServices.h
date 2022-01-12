#ifndef __interupt_service__
#define __interupt_service__

//    InteruptServices.h : Declarations of interrupts used
//
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
//
//
// ============== Chosen Interrupts ============================================
// #include "lwNx.h"                                                            // liddar from lightware

#ifdef __cplusplus
 extern "C" {
#endif

#include "definitions.h"                                                        // Global defines for all
#if (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY))
#include "SBGC_COMMAND.h"                                                       // SimpleBGC commands
#include "SBGC_PARSER.h"
#include "SBGC_cmd_helpers.h"
#include "SBGC_adj_vars.h"
#endif

#if defined(UART1_INTERUPT)                                                     /* UART1 active */
#include "odrive.h"                                                             /* odrive communcations */
#endif

#ifdef TIMER1_NEEDED                                                            // if you use Ethernet IP
extern void Timer1_interrupt();                                                 // This is used to increment Net_Ethernet_Intern_userTimerSec every second to prevent lock up
#endif
// extern void InitTimer2();
#ifdef UART2_INTERUPT                                                           // SimpleBGC needed for gimbal control on UART2
extern void UART2_interrupt();                                                  // simpleBGC gimbal protocol
#endif
#ifdef TIMER2_NEEDED                                                            // extended ACK
extern void Timer2_3Interrupt();                                                // extended timer ACK interrupt
#endif
#if defined(GPS_INCLUDED) || defined(GPS_INCLUDED2)                             // GPS being used on UART3
extern void interruptGPS();                                                     // interrupt for various gps units
#endif
#ifdef UART4_INTERUPT                                                           // XStream encoder is on UART4
extern void UART4_interrupt();                                                  // XS Encoder camera protocol
#endif
#if (CAMERA_TYPE == XS_CAM)
extern volatile videoChannelStatus_t g_XSVideoChan;                             // global object which stores time recording on each channel
#endif
#ifdef UART1_INTERUPT
extern void UART1_interrupt();                                                  // odrive for gait control on serial
#endif
#ifdef UART5_INTERUPT
extern void UART5_interrupt();                                                  // run cam on serial
#endif
#ifdef UART6_INTERUPT                                                           // LwNx Liddar is on UART6
extern void UART6_interrupt();                                                  // LwNx liddar protocol
#endif
#if defined(SERIAL_MAVLINK)                                                     /* we have mavlink on serial UART */
extern void MAV_interrupt();                                                    /* declare the interrupt */
#endif

// ============ Always used routines ===========================================
extern uint32_t hex2int(unsigned char *hex);                                    // converts hex to integer
extern void Timer_5Interrupt();                                                 // global timer tick counter
extern void coreTimer();                                                        /* interrupt on tick counter reaching 80M 1 second */
extern void Io0_Interrupt();                                                    /* INT0 pin rising edge interrupt example input from safety relay for safe machinery cut off */
extern void ChangeNotice();                                                     /* change notification request handler */
//#ifdef UART6_INTERUPT                                                           // liddar included
//extern lwResponsePacket LwNxResponse;                                           // structure to contain response from interrupt
//#endif
#if defined(PULSE_TACHO)                                                        /* pulse tacho is being used */
extern rotat_sens_obj_t g_TachoObj1;                                            /* define and initialise the tachometer - odometer - speed meas - flow object */
#endif
extern uint8_t g_globalStartUp;                                                 /* globally used register for storing state first pass around */
#if defined(GPS_INCLUDED) || defined(GPS_INCLUDED2)                             /* Things accociated with GPS interrupts   */
extern volatile uint8_t g_GPS_ready;                                            /* set to true state when GPS is ready and message has been collected */
#endif
#if defined(GPS_INCLUDED)
extern unsigned char GPStxt[MAX_GPS_MSG_LEN];                                   /* only type one needs extern */
#endif

#if (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY))
extern volatile uint8_t g_boardReady;
extern volatile uint16_t g_SBGCmsgCollect;
#endif

#ifdef __cplusplus
}
#endif

#endif