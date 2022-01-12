#ifndef __ActiNGT1_
#define __ActiNGT1_

#if defined(MARINE_PROTO_USED)
/*

Reader for Actisense NGT1

Read and write to an Actisense NGT-1 over its serial device.
This can be a serial version connected to an actual serial port
or an USB version connected to the virtual serial port.

(C) 2009-2015, Kees Verruijt, Harlingen, The Netherlands.

Part of this file is part of CANboat.

CANboat is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

CANboat is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

uses extracts of code also from
N2kTypes.h
Copyright (c) 2019 Vassilis Bourdakis, Timo Lappalainen, Kave Oy, www.kave.fi
and seasmart Copyright (c) 2017 Thomas Sarlandie thomas@sarlandie.net
Compiled and ported and re-written by ACP Aviation

*/
#include "definitions.h"
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "cpu_endian_defs.h"                                                    // endian definitions

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define N2KPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define N2KPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define N2KPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define N2KPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define N2KPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

/*
 * Defines to interface with an Actisense NGT-1
 */
/* ASCII characters used to mark packet start/stop */
#define STX (0x02u)                                                              /* Start packet */
#define ETX (0x03u)                                                              /* End packet */
#define DLE (0x10u)                                                              /* Start pto encode a STX or ETX send DLE+STX or DLE+ETX */
#define ESC (0x1Bu)                                                              /* Escape */
/* Actisense message structure is:

   DLE STX <command> <len> [<data> ...]  <checksum> DLE ETX
   Actisense Format:
   e.g. <10><02><93><length (1)><priority (1)><PGN (3)><destination (1)><source (1)><time (4)><len (1)><data (len)><CRC (1)><10><03>

   <command> is a byte from the list below.
   In <data> any DLE characters are double escaped (DLE DLE).
   <len> encodes the unescaped length.
   <checksum> is such that the sum of all unescaped data bytes plus the command
              byte plus the length adds up to zero, modulo 256.
*/
#define N2K_MSG_RECEIVED (0x93u)                                                /* Receive standard N2K message */
#define N2K_MSG_SEND (0x94u)                                                    /* Send N2K message */
#define N2K_INIT_SEQ(A) { DLE,STX,0x00u }                                       /* 0x10 0x02 0x00 It instructs the NGT1 to clear its PGN message TX list, thus it starts  sending all PGNs. */

#define N2K_MAX_RCV_TIME 20000LU                                                /* mx time to recieve a full can frame without resetting the offset */

//*****************************************************************************
// Read Actisense formatted NMEA2000 message from stream
// Actisense Format:
// <10><02><93><length (1)><priority (1)><PGN (3)><destination (1)><source (1)><time (4)><len (1)><data (len)><CRC (1)><10><03>
// or
// <10><02><94><length (1)><priority (1)><PGN (3)><destination (1)><len (1)><data (len)><CRC (1)><10><03>
#define NGT_MSG_RECEIVED (0xA0u)                                                /* Receive NGT specific message */
#define NGT_MSG_SEND (0xA1u)                                                    /* Send NGT message */

typedef enum
{
  MSG_START,
  MSG_ESCAPE,
  MSG_MESSAGE
} Acti_MSG_State_e;

/*
 * The 'converter' programs generate fake PGNs containing data that they generate
 * themselves or via proprietary non-PGN serial messages.
 * These need unique fake PGNs.
 */
#define CANBOAT_PGN_START 0x40000ul
#define CANBOAT_PGN_END 0x401FFul
#define ACTISENSE_BEM 0x40000ul                                                 /* Actisense specific fake PGNs */
#define IKONVERT_BEM 0x40100ul                                                  /* iKonvert specific fake PGNs */

/* ======================= NMEA 2000 ======================================== */
#if defined(NMEA2000_USED)

/*
 * NMEA 2000 uses the 8 'data' bytes as follows:
 * data[0] is an 'order' that increments, or not (depending a bit on implementation).
 * If the size of the packet <= 7 then the data follows in data[1..7]
 * If the size of the packet > 7 then the next byte data[1] is the size of the payload
 * and data[0] is divided into 5 bits index into the fast packet, and 3 bits 'order
 * that increases.
 * This means that for 'fast packets' the first bucket (sub-packet) contains 6 payload
 * bytes and 7 for remaining. Since the max index is 31, the maximal payload is
 * 6 + 31 * 7 = 223 bytes
 */
#define FASTPACKET_INDEX (0u)
#define FASTPACKET_SIZE (1u)
#define FASTPACKET_BUCKET_0_SIZE (6u)
#define FASTPACKET_BUCKET_N_SIZE (7u)
#define FASTPACKET_BUCKET_0_OFFSET (2u)
#define FASTPACKET_BUCKET_N_OFFSET (1u)
#define FASTPACKET_MAX_INDEX (0x1fu)
#define FASTPACKET_MAX_SIZE (FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * (FASTPACKET_MAX_INDEX - 1u))

#define TP_CM 60416LU                                                           /* Multi packet connection management, TP.CM */
#define TP_DT 60160LU                                                           /* Multi packet data transfer, TP.DT */
#define TP_CM_BAM 32u
#define TP_CM_RTS 16u
#define TP_CM_CTS 17u
#define TP_CM_ACK 19u
#define TP_CM_Abort 255u

#define TP_CM_AbortBusy 1u                                                      // Already in one or more connection managed sessions and cannot support another.
#define TP_CM_AbortNoResources 2u                                                // System resources were needed for another task so this connection managed session was terminated.
#define TP_CM_AbortTimeout 3u                                                   // A timeout occurred and this is the connection abort to close the session.

#define N2K_COLLECTED 0u                                                        /* can Bus collection states for multi frame */
#define N2K_COLLECTING 1u
#define N2K_INCOMPLETE_FRAME 2u

#define N2K_FASTPACKET_FORCE 0x80u                                              /* set priority to this to force fast packet send */
/*
 * Notes on the NMEA 2000 packet structure
 * ---------------------------------------
 *
 * http://www.nmea.org/Assets/pgn059392.pdf tells us that:
 * - All messages shall set the reserved bit in the CAN ID field to zero on transmit.
 * - Data field reserve bits or reserve bytes shall be filled with ones. i.e. a reserve
 *   byte will be set to a hex value of FF, a single reservie bit would be set to a value of 1.
 * - Data field extra bytes shall be illed with a hex value of FF.
 * - If the PGN in a Command or Request is not recognized by the destination it shall
 *   reply with the PGN 059392 ACK or NACK message using a destination specific address.
 *
 */
 
 /* ----------------- header of the structure ------------------------- */
 #if defined(D_FT900)
typedef struct N2KPACKED {
  uint8_t FullLen;                                                             /* data length plus 11 */
  uint8_t priority;
  uint32_t PGN;
  uint8_t dst;
  uint8_t src;
  uint32_t messageTyp;
  uint8_t DataLen;
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];
} N2K_Header_t;
#else
N2KPACKED (
typedef struct
{
  uint8_t FullLen;
  uint8_t priority;
  uint32_t PGN;
  uint8_t dst;
  uint8_t src;
  uint32_t messageTyp;
  uint8_t DataLen;
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];
}) N2K_Header_t;
#endif

#define N2K_NACK_PGN 059392UL
/* send this PGN only to :- Request a particular device to send a PGN sewnd this code as the data */
#define N2K_REQ_A_PGN 59904UL
/* send this PGN only to :- Request a particular device to change its network address */
#define N2K_CHG_ADDR_PGN 65240UL
#if defined(D_FT900)
typedef struct N2KPACKED {
        uint32_t PGN;
        uint16_t proprieter;
        uint32_t nameLo;                                                        // xfffffff
        uint32_t nameHi;                                                        // $name >> 32
        uint8_t Address;
} n2k_ChgAddr_t;
#else
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint16_t proprieter;
        uint32_t nameLo;                                                        // xfffffff
        uint32_t nameHi;                                                        // $name >> 32
        uint8_t Address;
}) n2k_ChgAddr_t;
#endif
/*
 * Some packets include a "SID", explained by Maretron as follows:
 * SID: The sequence identifier field is used to tie related PGNs together. For example,
 * the DST100 will transmit identical SIDs for Speed (PGN 128259) and Water depth
 * (128267) to indicate that the readings are linked together (i.e., the data from each
 * PGN was taken at the same time although reported at slightly different times).
 */
#define MaretronProprietary 0x9889u                                             // Maretron 137 + reserved + industry code=marine
//*****************************************************************************
// Trip Volume [as used by MARETRON FFM100]
// Volume should be in litres
// Input:
//  - SID                   Sequence ID.
//  - VolumeInstance        This should be unique at least on one device. May be best to have it unique over all devices sending this PGN.
//  - FluidType             see tN2kFluidType [possible values for this field include Fuel, Fresh Water, Waste Water, Live Well, Oil, and Black Water]
//  - TripVolume            This field is used to indicate the trip volume in units of litres.
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define MARETRON_FFM100_FTv_PGN 65287UL                                         // Trip Volume Volume unit is 1 litre
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;          */
        uint16_t proprieter;
        uint8_t SID;
        uint8_t VolumeInstance;
        uint8_t FluidType;
        float64_t TripVolume;
} maretron_FluidVol_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;        */
        uint16_t proprieter;
        uint8_t SID;
        uint8_t VolumeInstance;
        uint8_t FluidType;
        float64_t TripVolume;
}) maretron_FluidVol_t;
#endif

// Fluid Flow Rate [as used by MARETRON FFM100]
// Flow Rate should be in lt/hour
// Input:
//  - SID                   Sequence ID.
//  - FlowRateInstance      This should be unique at least on one device. May be best to have it unique over all devices sending this PGN.
//  - FluidType             see tN2kFluidType [possible values for this field include Fuel, Fresh Water, Waste Water, Live Well, Oil, and Black Water]
//  - FluidFlowRate         This field is used to indicate the rate of fluid flow in units litres/hour.
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define MARETRON_FFM100_FRate_PGN 65286UL                                       // Fluid Flow Rate Flow unit is 1 lt/hour
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;       */
        uint8_t DataLen;
        uint16_t proprieter;
        uint8_t SID;
        uint8_t FlowRateInstance;
        uint8_t FluidType;
        float64_t FluidFlowRate;
} maretron_flow_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;           */
        uint8_t DataLen;
        uint16_t proprieter;
        uint8_t SID;
        uint8_t FlowRateInstance;
        uint8_t FluidType;
        float64_t FluidFlowRate;
}) maretron_flow_t;
#endif

#if defined(D_FT900)                                                            /* This structure is needed if you use multi frame receive - probably not needed here but not sure */
typedef struct N2KPACKED {
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  maretron_flow_t MaretronFloObj;                                               /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
} N2K_FlowRateRcv_t;
#else
N2KPACKED (
typedef struct
{
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  maretron_flow_t MaretronFloObj;                                               /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
}) N2K_FlowRateRcv_t;
#endif

// Temperature High Range [as used by MARETRON TMP100]
// Temperatures should be in Kelvin
// Input:
//  - SID                   Sequence ID.
//  - TempInstance          This should be unique at least on one device. May be best to have it unique over all devices sending this PGN.
//  - TempSource            see tN2kTempSource
//  - ActualTemperature     Temperature in K. Use function CToKelvin, if you want to use °C.
//  - SetTemperature        Set temperature in K. Use function CToKelvin, if you want to use °C. This is meaningful for temperatures
//                          which can be controlled like cabin, freezer, refrigeration temperature.
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
#define MARETRON_TMP100_PGN 130823UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;         */
        uint16_t proprieter;
        uint8_t SID;
        uint8_t TempInstance;
        uint8_t TempSource;
        float64_t ActualTemperature;
        float64_t SetTemperature;
} maretron_temp_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;             */
        uint16_t proprieter;
        uint8_t SID;
        uint8_t TempInstance;
        uint8_t TempSource;
        float64_t ActualTemperature;
        float64_t SetTemperature;
}) maretron_temp_t;
#endif

#if defined(D_FT900)                                                            /* This structure is needed if you use multi frame receive - probably not needed here but not sure */
typedef struct N2KPACKED {
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  maretron_temp_t MaretronTempObj;                                               /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
} N2K_MarTempRcv_t;
#else
N2KPACKED (
typedef struct
{
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  maretron_temp_t MaretronTempObj;                                               /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
}) N2K_MarTempRcv_t;
#endif
/*
 * NMEA 2000 uses the 8 'data' bytes as follows:
 * data[0] is an 'order' that increments, or not (depending a bit on implementation).
 * If the size of the packet <= 7 then the data follows in data[1..7]
 * If the size of the packet > 7 then the next byte data[1] is the size of the payload
 * and data[0] is divided into 5 bits index into the fast packet, and 3 bits 'order
 * that increases.
 * This means that for 'fast packets' the first bucket (sub-packet) contains 6 payload
 * bytes and 7 for remaining. Since the max index is 31, the maximal payload is
 * 6 + 31 * 7 = 223 bytes
 */
//*****************************************************************************
// System date/time
// Input:
//  - SID                   Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - SystemDate            Days since 1970-01-01
//  - SystemTime            seconds since midnight
//  - TimeSource            see tN2kTimeSource
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_SYSTIM_PGN 126992UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*      uint32_t PGN;
        uint8_t priority;
        uint8_t SID; */
        uint8_t TimeSource;
        uint16_t SystemDate;
        float64_t SystemTime;
} n2k_time_t;
#else
N2KPACKED (
typedef struct {
/*      uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint8_t TimeSource;
        uint16_t SystemDate;
        float64_t SystemTime;
}) n2k_time_t;
#endif

// Vessel Heading
// Input:
//  - SID                   Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - Heading               Heading in radians
//  - Deviation             Magnetic deviation in radians. Use N2kDoubleNA for undefined value.
//  - Variation             Magnetic variation in radians. Use N2kDoubleNA for undefined value.
//  - ref                   Heading reference. See definition of tN2kHeadingReference.
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_VESSEL_PGN 127250UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;       */
        float64_t Heading;
        float64_t Deviation;
        float64_t Variation;
        uint8_t ref : 2u;
        uint8_t spare6 : 6u;
} n2k_vesselHeading_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;        */
        float64_t Heading;
        float64_t Deviation;
        float64_t Variation;
        uint8_t ref : 2u;
        uint8_t spare6 : 6u;
}) n2k_vesselHeading_t;
#endif
// Rate of Turn
// Input:
//  - SID                   Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - Rate of turn          Change in heading in radians per second
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_RATEOFTURN_PGN 127251UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        float64_t RateOfTurn;
} n2k_rateOfturn_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;      */
        float64_t RateOfTurn;
}) n2k_rateOfturn_t;
#endif
// Attitude
// Input:
//  - SID                   Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - Yaw                   Heading in radians.
//  - Pitch                 Pitch in radians. Positive, when your bow rises.
//  - Roll                  Roll in radians. Positive, when tilted right.
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_ATTITUDE_PGN 127257UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;        */
        float64_t Yaw;
        float64_t Pitch;
        float64_t Roll;
} n2k_attitude_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;         */
        float64_t Yaw;
        float64_t Pitch;
        float64_t Roll;
}) n2k_attitude_t;
#endif
// Magnetic Variation
// Input:
//  - SID                   Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - Source                How was the variation value generated
//  - DaysSince1970         Days since January 1, 1970
//  - Variation             Magnetic variation/declination in radians
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_MAGVAR_PGN 127258UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;   */
        uint16_t DaysSince1970;
        float64_t Variation;
} n2k_magVar_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID; */
        uint16_t DaysSince1970;
        float64_t Variation;
}) n2k_magVar_t;
#endif
// Engine parameters rapid
// Input:
//  - EngineInstance        Engine instance.
//  - EngineSpeed           RPM (Revolutions Per Minute)
//  - EngineBoostPressure   in Pascal
//  - EngineTiltTrim        in %
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_ENGRAP_PGN 127488UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;   */
        uint8_t EngineInstance;
        float64_t EngineSpeed;
        float64_t EngineBoostPressure;
        uint8_t EngineTiltTrim;
} n2k_engineRapid_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID; */
        uint8_t EngineInstance;
        float64_t EngineSpeed;
        float64_t EngineBoostPressure;
        uint8_t EngineTiltTrim;
}) n2k_engineRapid_t;
#endif
// Engine parameters dynamic
// Input:
//  - EngineInstance        Engine instance.
//  - EngineOilPress        in Pascal
//  - EngineOilTemp         in Kelvin
//  - EngineCoolantTemp     in Kelvin
//  - AltenatorVoltage      in Voltage
//  - FuelRate              in litres/hour
//  - EngineHours           in seconds
//  - EngineCoolantPress    in Pascal
//  - EngineFuelPress       in Pascal
//  - EngineLoad            in %
//  - EngineTorque          in %
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_ENGDYN_PGN 127489UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint8_t EngineInstance;
        float64_t EngineOilPress;
        float64_t EngineOilTemp;
        float64_t EngineCoolantTemp;
        float64_t AltenatorVoltage;
        float64_t FuelRate;
        float64_t EngineHours;
        float64_t EngineCoolantPress;
        float64_t EngineFuelPress;
        uint8_t Reserved;                                                       // set this to 0xff
        uint16_t Status1;
        uint16_t Status2;
        float64_t EngineLoad;
        float64_t EngineTorque;
} n2k_engineDynamics_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        uint8_t EngineInstance;
        float64_t EngineOilPress;
        float64_t EngineOilTemp;
        float64_t EngineCoolantTemp;
        float64_t AltenatorVoltage;
        float64_t FuelRate;
        float64_t EngineHours;
        float64_t EngineCoolantPress;
        float64_t EngineFuelPress;
        uint8_t Reserved;                                                       // set this to 0xff
        uint16_t Status1;
        uint16_t Status2;
        float64_t EngineLoad;
        float64_t EngineTorque;
}) n2k_engineDynamics_t;
#endif
#define flagCheckEngine (1u<<0u)
#define flagOverTempengine (1u<<1u)
#define flagLowOilPress (1u<<2u)
#define flagLowOilLevel (1u<<3u)
#define flagLowFuelPress (1u<<4u)
#define flagLowSystemVoltage (1u<<5u)
#define flagLowCoolantLevel (1u<<6u)
#define flagWaterFlow (1u<<7u)
#define flagWaterInFuel (1u<<8u)
#define flagChargeIndicator (1u<<9u)
#define flagPreheatIndicator (1u<<10u)
#define flagHighBoostPress (1u<<11u)
#define flagRevLimitExceeded (1u<12u)
#define flagEgrSystem (1u<<13u)
#define engineStatus1P2 (1u<<14u)
#define flagEmergencyStopMode (1u<<15u)
#define flagWarning1 (1u<<0u)
#define flagWarning2 (1u<<1u)
#define flagPowerReduction (1u<<2u)
#define flagMaintenanceNeeded (1u<<3u)
#define flagEngineCommError (1u<<4u)
#define flagSubThrottle (1u<<5u)
#define flagNeutralStartProtect (1u<<6u)
#define flagEngineShuttingDown (1u<<7u)

#if defined(D_FT900)                                                            /* This structure is needed if you use multi frame receive - probably not needed here but not sure */
typedef struct N2KPACKED {
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_engineDynamics_t engDynObj;                                               /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
} N2K_EngineDynRcv_t;
#else
N2KPACKED (
typedef struct
{
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_engineDynamics_t engDynObj;                                               /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
}) N2K_EngineDynRcv_t;
#endif
// Transmission parameters, dynamic
// Input:
//  - EngineInstance        Engine instance.
//  - TransmissionGear      Selected transmission. See tN2kTransmissionGear
//  - OilPressure           in Pascal
//  - OilTemperature        in K
//  - EngineTiltTrim        in %
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_TRANSDYN_PGN 127493UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint8_t EngineInstance;
        uint8_t TransmissionGear;
        float64_t OilPressure;
        float64_t OilTemperature;
        uint8_t DiscreteStatus1;
} n2k_transmissDyn_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;     */
        uint8_t EngineInstance;
        uint8_t TransmissionGear;
        float64_t OilPressure;
        float64_t OilTemperature;
        uint8_t DiscreteStatus1;
}) n2k_transmissDyn_t;
#endif
// Trip Parameters, Engine
// Input:
//  - EngineInstance           Engine instance.
//  - TripFuelUsed             in litres
//  - FuelRateAverage          in litres/hour
//  - FuelRateEconomy          in litres/hour
//  - InstantaneousFuelEconomy in litres/hour
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_TRIPENG_PGN 127497UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;     */
        uint8_t EngineInstance;
        float64_t TripFuelUsed;
        float64_t FuelRateAverage;
        float64_t FuelRateEconomy;
        float64_t InstantaneousFuelEconomy;
} n2k_EngTripDyn_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint8_t EngineInstance;
        float64_t TripFuelUsed;
        float64_t FuelRateAverage;
        float64_t FuelRateEconomy;
        float64_t InstantaneousFuelEconomy;
}) n2k_EngTripDyn_t;
#endif
// binary status  PGN
#define N2K_BINARY_STATUS 127501L
// Fluid level
// Input:
//  - Instance              Tank instance. Different devices handles this a bit differently. So it is best to have instance unique over
//                          all devices on the bus.
//  - FluidType             Defines type of fluid. See definition of tN2kFluidType
//  - Level                 Tank level in % of full tank.
//  - Capacity              Tank Capacity in litres
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_FLUIDLVL_PGN 127505UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;  */
        uint8_t Instance : 4u;
        uint8_t FluidType : 4u;
        float64_t Level;
        float64_t Capacity;
} n2k_FluidLvl_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;    */
        uint8_t Instance : 4u;
        uint8_t FluidType : 4u;
        float64_t Level;
        float64_t Capacity;
}) n2k_FluidLvl_t;
#endif
// DC Detailed Status
// Input:
//  - SID                   Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - DCInstance            DC instance.
//  - DCType                Defines type of DC source. See definition of tN2kDCType
//  - StateOfCharge         % of charge
//  - StateOfHealth         % of heath
//  - TimeRemaining         Time remaining in seconds
//  - RippleVoltage         DC output voltage ripple in V
//  - Capacity              Battery capacity in coulombs
//*****************************************************************************
#define N2K_DCDETAIL_PGN 127506UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        uint8_t DCInstance;
        uint8_t DCType;
        uint8_t StateOfCharge;
        uint8_t StateOfHealth;
        float64_t TimeRemaining;
        float64_t RippleVoltage;
        float64_t Capacity;
} n2k_dc_detail_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;   */
        uint8_t DCInstance;
        uint8_t DCType;
        uint8_t StateOfCharge;
        uint8_t StateOfHealth;
        float64_t TimeRemaining;
        float64_t RippleVoltage;
        float64_t Capacity;
}) n2k_dc_detail_t;
#endif
// Charger Status
// Input:
//  - Instance                     ChargerInstance.
//  - BatteryInstance              BatteryInstance.
//  - Operating State              see. tN2kChargeState
//  - Charger Mode                 see. tN2kChargerMode
//  - Charger Enable/Disable       boolean
//  - Equalization Pending         boolean
//  - Equalization Time Remaining  double seconds
//
//*****************************************************************************
#define N2K_CHARGERSTAT_PGN 127507UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;   */
        uint8_t Instance;
        uint8_t ChargeState : 4u;
        uint8_t ChargerMode : 4u;
        uint8_t Enabled : 2u;
        uint8_t EqualizationPending : 2u;
        uint8_t endByte10 : 4u;
        float64_t EqualizationTimeRemaining;
} n2k_charger_stat_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint8_t Instance;
        uint8_t ChargeState : 4u;
        uint8_t ChargerMode : 4u;
        uint8_t Enabled : 2u;
        uint8_t EqualizationPending : 2u;
        uint8_t endByte10 : 4u;
        float64_t EqualizationTimeRemaining;
}) n2k_charger_stat_t;
#endif
// Battery Configuration Status
// Note this has not yet confirmed to be right. Specially Peukert Exponent can have in
// this configuration values from 1 to 1.504. And I expect on code that I have to send
// value PeukertExponent-1 to the bus.
// Input:
//  - BatteryInstance       BatteryInstance.
//  - BatType               Type of battery. See definition of tN2kBatType
//  - SupportsEqual         Supports equalization. See definition of tN2kBatEqSupport
//  - BatNominalVoltage     Battery nominal voltage. See definition of tN2kBatNomVolt
//  - BatChemistry          Battery See definition of tN2kBatChem
//  - BatCapacity           Battery capacity in Coulombs. Use AhToCoulombs, if you have your value in Ah.
//  - BatTemperatureCoeff   Battery temperature coefficient in %
//  - PeukertExponent       Peukert Exponent
//  - ChargeEfficiencyFactor Charge efficiency factor
//*****************************************************************************
#define N2K_BATCONF_PGN 127513UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint8_t BatInstance;
        uint8_t BatType : 4u;
        uint8_t SupportsEqual : 4u;
        uint8_t BatNominalVoltage : 4u;
        uint8_t BatChemistry : 4u;
        float64_t BatCapacity;
        uint8_t BatTemperatureCoefficient;
        float64_t PeukertExponent;
        float64_t ChargeEfficiencyFactor;
} n2k_bat_conf_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;       */
        uint8_t BatInstance;
        uint8_t BatType : 4u;
        uint8_t SupportsEqual : 4u;
        uint8_t BatNominalVoltage : 4u;
        uint8_t BatChemistry : 4u;
        float64_t BatCapacity;
        uint8_t BatTemperatureCoefficient;
        float64_t PeukertExponent;
        float64_t ChargeEfficiencyFactor;
}) n2k_bat_conf_t;
#endif
#if defined(D_FT900)                                                            /* This structure is needed if you use multi frame receive - probably not needed here but not sure */
typedef struct N2KPACKED {
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_bat_conf_t config;                                               /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
} N2K_BatteryConfigRcv_t;
#else
N2KPACKED (
typedef struct
{
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_bat_conf_t config;                                             /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
}) N2K_BatteryConfigRcv_t;
#endif
/* battery status */
#define N2K_BATSTAT_PGN 127508UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;  */
        uint8_t BatInstance;
        float64_t BatteryVoltage;
        float64_t BatteryCurrent;
        float64_t BatteryTemperature;
/*        uint8_t SID;      */
} n2k_bat_stat_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;      */
        uint8_t BatInstance;
        float64_t BatteryVoltage;
        float64_t BatteryCurrent;
        float64_t BatteryTemperature;
/*        uint8_t SID;         */
}) n2k_bat_stat_t;
#endif
/* Set audio volume    */
#define N2K_SETVOL_PGN 130816UL
#if defined(D_FT900)
typedef struct N2KPACKED {
        uint32_t PGN;
        uint8_t priority;
        uint32_t CodeDWord;                                                     // \\013\\099\\xff\\018
        uint8_t Byte00;                                                         // 0x00
        uint16_t Zone;
        uint16_t Level;
} n2k_audio_vol_t;
#else
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint32_t CodeDWord;                                                     // \\013\\099\\xff\\018
        uint8_t Byte00;                                                         // 0x00
        uint16_t Zone;
        uint16_t Level;
}) n2k_audio_vol_t;
#endif
/* Inverter status. Operating state and inverter are bitfields: 0=off, 1=on  */
#define N2K_INVSTAT_PGN 127509UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;     */
        uint8_t instance;
        uint8_t ac_instance;
        uint8_t dc_instance;
        uint8_t operatingState : 4u;
        uint8_t inverter : 4u;
} n2k_inverter_stat_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;    */
        uint8_t instance;
        uint8_t ac_instance;
        uint8_t dc_instance;
        uint8_t operatingState : 4u;
        uint8_t inverter : 4u;
}) n2k_inverter_stat_t;
#endif
// Leeway
// Input:
//  - SID            Sequence ID field
//  - Leeway         Nautical Leeway Angle, which is defined as the angle between the vessel’s heading (direction to which the
//                   vessel’s bow points) and its course (direction of its motion (track) through the water)
// Output:
//  - N2kMsg         NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_LEEWAY_PGN 128000UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        float64_t Leeway;
        uint8_t byte1;                                                          // set to 0xFFLU
        uint8_t byte2;                                                          // set to 0xFFLU
        uint8_t byte3;                                                          // set to 0xFFLU
        uint8_t byte4;                                                          // set to 0xFFLU
        uint8_t byte5;                                                          // set to 0xFFLU
} n2k_Leeway_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;      */
        float64_t Leeway;
        uint8_t byte1;                                                          // set to 0xFFLU
        uint8_t byte2;                                                          // set to 0xFFLU
        uint8_t byte3;                                                          // set to 0xFFLU
        uint8_t byte4;                                                          // set to 0xFFLU
        uint8_t byte5;                                                          // set to 0xFFLU
}) n2k_Leeway_t;
#endif

/* direction Data */
#define N2K_DIR_DATA_PGN 130577LU
/* vessel speed component and leeway */
#define N2K_VESSEL_SPEED_COMP 130578LU

// Boat speed
// Input:
//  - SID                   Sequence ID. If your device is e.g. boat speed and wind at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - WaterReferenced        Speed over water in m/s
//  - GroundReferenced      Ground referenced speed in m/s
//  - SWRT                  Type of transducer. See definition for tN2kSpeedWaterReferenceType
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_BOATSPEED_PGN 128259UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;           */
        float64_t WaterReferenced;
        float64_t GroundReferenced;
        uint8_t SWRT;
} n2k_BoatSpeed_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;           */
        float64_t WaterReferenced;
        float64_t GroundReferenced;
        uint8_t SWRT;
}) n2k_BoatSpeed_t;
#endif
// Water depth
// Input:
//  - SID                   Sequence ID. If your device is e.g. boat speed and depth at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - DepthBelowTransducer  Depth below transducer in meters
//  - Offset                Distance in meters between transducer and surface (positive) or transducer and keel (negative)
//  - Range                 Measuring range
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_WATERDEPTH_PGN 128267UL
#if defined(D_FT900)
typedef struct N2KPACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;
        float64_t DepthBelowTransducer;
        float64_t Offset;
        float64_t Range;
} n2k_WaterDepth_t;
#else
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;
        float64_t DepthBelowTransducer;
        float64_t Offset;
        float64_t Range;
}) n2k_WaterDepth_t;
#endif
// Distance log
// Input:
//  - DaysSince1970         Timestamp
//  - SecondsSinceMidnight  Timestamp
//  - Log                   Total meters travelled
//  - Trip Log              Meters travelled since last reset
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_DISTLOG_PGN 128275UL
#if defined(D_FT900)
typedef struct N2KPACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;
        uint16_t DaysSince1970;
        float64_t SecondsSinceMidnight;
        uint32_t Log;
        uint32_t TripLog;
} n2k_distLog_t;
#else
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;
        uint16_t DaysSince1970;
        float64_t SecondsSinceMidnight;
        uint32_t Log;
        uint32_t TripLog;
}) n2k_distLog_t;
#endif
// Lat/lon rapid
// Input:
//  - Latitude               Latitude in degrees
//  - Longitude              Longitude in degrees
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_LATLONRAPID_PGN 129025UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;     */
        float64_t Latitude;
        float64_t Longitude;
} n2k_lat_lon_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;           */
        float64_t Latitude;
        float64_t Longitude;
}) n2k_lat_lon_t;
#endif
// COG SOG rapid
// Input:
//  - COG                   Cource Over Ground in radians
//  - SOG                   Speed Over Ground in m/s
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_COGSOG_PGN 129026UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint8_t HeadingRef : 2u;
        uint8_t SpareBits : 6u;
        float64_t COG;
        float64_t SOG;
        uint8_t byte1;
        uint8_t byte2;
} n2k_CogSog_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        uint8_t HeadingRef : 2u;
        uint8_t SpareBits : 6u;
        float64_t COG;
        float64_t SOG;
        uint8_t byte1;
        uint8_t byte2;
}) n2k_CogSog_t;
#endif
// GNSS Position Data
// Input:
//  - SID                   Sequence ID. If your device is e.g. boat speed and GPS at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - DaysSince1970         Days since 1.1.1970. You can find sample how to convert e.g. NMEA0183 date info to this on my NMEA0183 library on
//                          NMEA0183Messages.cpp on function NMEA0183ParseRMC_nc
//  - Latitude              Latitude in degrees
//  - Longitude             Longitude in degrees
//  - Altitude              Altitude in meters
//  - GNSStype              GNSS type. See definition of tN2kGNSStype
//  - GNSSmethod            GNSS method type. See definition of tN2kGNSSmethod
//  - nSatellites           Number of satellites used for data
//  - HDOP                  Horizontal Dilution Of Precision in meters.
//  - PDOP                  Probable dilution of precision in meters.
//  - GeoidalSeparation     Geoidal separation in meters
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_GNSSPOS_PGN 129029UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint16_t DaysSince1970;
        float64_t SecondsSinceMidnight;
        float64_t Latitude;
        float64_t Longitude;
        float64_t Altitude;
        uint8_t GNSStype : 4u;
        uint8_t GNSSmethod : 4u;
        uint8_t Integrity;
        uint8_t nSatellites;
        float64_t HDOP;
        float64_t PDOP;
        float64_t GeoidalSeparation;
        uint8_t nReferenceStations;
        uint16_t ReferenceStationType : 8u;
        uint16_t ReferenceStationID : 4u;
        uint16_t spare : 4u;
        float64_t AgeOfCorrection;
} n2k_gnss_pos_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;      */
        uint16_t DaysSince1970;
        float64_t SecondsSinceMidnight;
        float64_t Latitude;
        float64_t Longitude;
        float64_t Altitude;
        uint8_t GNSStype : 4u;
        uint8_t GNSSmethod : 4u;
        uint8_t Integrity;
        uint8_t nSatellites;
        float64_t HDOP;
        float64_t PDOP;
        float64_t GeoidalSeparation;
        uint8_t nReferenceStations;
        uint16_t ReferenceStationType : 8u;
        uint16_t ReferenceStationID : 4u;
        uint16_t spare : 4u;
        float64_t AgeOfCorrection;
}) n2k_gnss_pos_t;
#endif
#if defined(D_FT900)                                                            /* This structure is needed if you use multi frame receive - probably not needed here but not sure */
typedef struct N2KPACKED {
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_gnss_pos_t GNSSPosObj;                                               /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
} N2K_GNSSPosRcv_t;
#else
N2KPACKED (
typedef struct
{
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_gnss_pos_t GNSSPosObj;                                               /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
}) N2K_GNSSPosRcv_t;
#endif
// Date,Time & Local offset  ( see also PGN 126992 )
// Input:
//  - DaysSince1970         Days since 1.1.1970. You can find sample how to convert e.g. NMEA0183 date info to this on my NMEA0183 library on
//                          NMEA0183Messages.cpp on function NMEA0183ParseRMC_nc
//  - Time                  Seconds since midnight
//  - Local offset          Local offset in minutes
//*****************************************************************************
#define N2K_DATETIM_PGN 129033UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        uint16_t DaysSince1970;
        float64_t SecondsSinceMidnight;
        uint16_t LocalOffset;
} n2k_dateTim_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        uint16_t DaysSince1970;
        float64_t SecondsSinceMidnight;
        uint16_t LocalOffset;
}) n2k_dateTim_t;
#endif
// GNSS DOP data
// Input:
//  - SID                   Sequence ID. If your device is e.g. boat speed and GPS at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - DesiredMode           Desired DOP mode.
//  - ActualMode            Actual DOP mode.
//  - HDOP                  Horizontal Dilution Of Precision in meters.
//  - PDOP                  Probable dilution of precision in meters.
//  - TDOP                  Time dilution of precision
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_GNSSDOP_PGN 129539UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;        */
        uint8_t ActualMode : 3u;
        uint8_t SpareBits : 2u;
        uint8_t DesiredMode : 3u;
        float64_t HDOP;
        float64_t VDOP;
        float64_t TDOP;
} n2k_GNSSDOP_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint8_t ActualMode : 3u;
        uint8_t SpareBits : 2u;
        uint8_t DesiredMode : 3u;
        float64_t HDOP;
        float64_t VDOP;
        float64_t TDOP;
}) n2k_GNSSDOP_t;
#endif
// Waypoint list
// Input:
//  - Start                 The ID of the first waypoint
//  - Database              Database ID
//  - Route                 Route ID
//  - RouteName             The name of the current route
//  - ID                    The ID of the current waypoint
//  - Name                  The name of the current waypoint
//  - Latitude              The latitude of the current waypoint
//  - Longitude             The longitude of the current waypoint
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
//#define N2K_WAYPOINT_PGN 129539UL   ?????

// AIS static data class A
// Input:
//  - MessageID             Message type
//  - Repeat                Repeat indicator
//  - UserID                MMSI
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_AISCLASSA_PGN 129038UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;         */
        uint8_t MessageID : 2u;
        uint8_t SpareBits : 4u;
        uint8_t Repeat : 2u;
        uint32_t UserID;
        float64_t Longditude;
        float64_t Latitude;
        uint8_t Accuracy : 1u;
        uint8_t RAIM : 1u;
        uint8_t Seconds : 2u;
        uint8_t SpareBits2 : 4u;
        float64_t COG;
        float64_t SOG;
        uint16_t CommStat;
        uint8_t TransInfo;
        float64_t Heading;
        float64_t ROT;
        uint8_t NavStatus;
        uint8_t Reserved;
} n2k_AISRepClassA129038_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;                */
        uint8_t MessageID : 2u;
        uint8_t SpareBits : 4u;
        uint8_t Repeat : 2u;
        uint32_t UserID;
        float64_t Longditude;
        float64_t Latitude;
        uint8_t Accuracy : 1u;
        uint8_t RAIM : 1u;
        uint8_t Seconds : 2u;
        uint8_t SpareBits2 : 4u;
        float64_t COG;
        float64_t SOG;
        uint16_t CommStat;
        uint8_t TransInfo;
        float64_t Heading;
        float64_t ROT;
        uint8_t NavStatus;
        uint8_t Reserved;
}) n2k_AISRepClassA129038_t;
#endif
#if defined(D_FT900)                                                            /* This structure is needed if you use multi frame receive - probably not needed here but not sure */
typedef struct N2KPACKED {
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_AISRepClassA129038_t posReport;                                               /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
} N2K_AISRepClassA129038Rcv_t;
#else
N2KPACKED (
typedef struct
{
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_AISRepClassA129038_t posReport;                                             /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
}) N2K_AISRepClassA129038Rcv_t;
#endif
#define N2K_AISCLASSAA_PGN 129794UL                                             /* AIS Static Data Class A message */
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;  */
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        uint32_t UserID;
        uint32_t IMOnumber;
        unsigned char Callsign[7u];
        unsigned char Name[20u];
        uint8_t VesselType;
        float64_t Length;
        float64_t Beam;
        float64_t PosRefStbd;
        float64_t PosRefBow;
        uint16_t ETAdate;
        float64_t ETAtime;
        float64_t Draught;
        unsigned char Destination[20u];
        uint8_t AISversion : 2u;
        uint8_t GNSStype : 4u;
        uint8_t DTE : 1u;
        uint8_t sparebit : 1u;
        uint8_t AISTransieverInfo : 5u;
        uint8_t spare : 3u;
} n2k_AISStaticData129794_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority; */
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        uint32_t UserID;
        uint32_t IMOnumber;
        unsigned char Callsign[7u];
        unsigned char Name[20u];
        uint8_t VesselType;
        float64_t Length;
        float64_t Beam;
        float64_t PosRefStbd;
        float64_t PosRefBow;
        uint16_t ETAdate;
        float64_t ETAtime;
        float64_t Draught;
        unsigned char Destination[20u];
        uint8_t AISversion : 2u;
        uint8_t GNSStype : 4u;
        uint8_t DTE : 1u;
        uint8_t sparebit : 1u;
        uint8_t AISTransieverInfo : 5u;
        uint8_t spare : 3u;
}) n2k_AISStaticData129794_t;
#endif
#define N2K_SAR_AIRPOS_PGN 129798UL                                             /* AIS SAR Aircraft Position Report */
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;        */
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        float64_t Altitude;
        float64_t SOG;
        uint8_t PositionAccuracy;
        float64_t Longitude;
        float64_t Latitude;
        float64_t COG;
        uint32_t Timestamp;
        uint8_t RegionalReserve;
        uint8_t DTE : 1u;
        uint8_t spare1 : 3u;
        uint8_t AISMode : 1u;
        uint8_t RAIM : 1u;
        uint8_t ComState : 1u;
        uint8_t spare2 : 1u;
} n2k_AISSARAirRep129798_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;       */
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        float64_t Altitude;
        float64_t SOG;
        uint8_t PositionAccuracy;
        float64_t Longitude;
        float64_t Latitude;
        float64_t COG;
        uint32_t Timestamp;
        uint8_t RegionalReserve;
        uint8_t DTE : 1u;
        uint8_t spare1 : 3u;
        uint8_t AISMode : 1u;
        uint8_t RAIM : 1u;
        uint8_t ComState : 1u;
        uint8_t spare2 : 1u;
}) n2k_AISSARAirRep129798_t;
#endif
#if defined(D_FT900)                                                            /* This structure is needed if you use multi frame receive - probably not needed here but not sure */
typedef struct N2KPACKED {
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_AISSARAirRep129798_t posReport;                                           /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
} N2K_AISSARAirRep129798Rcv_t;
#else
N2KPACKED (
typedef struct
{
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_AISSARAirRep129798_t posReport;                                           /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
}) N2K_AISSARAirRep129798Rcv_t;
#endif
#define N2K_ADDR_SAFETY_PGN 129801UL                                             /* AIS Addressed Safety Related Message */
#if defined(D_FT900)
typedef struct N2KPACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        uint32_t SourceId : 30u;
        uint32_t SeqNum : 2u;
        uint32_t DestId : 30u;
        uint32_t ReTransmit : 1u;
        uint32_t unusedBit1 : 1u;
        unsigned char SafetyText[156u];
} n2k_AISAddrSafety129801_t;
#else
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        uint32_t SourceId : 30u;
        uint32_t SeqNum : 2u;
        uint32_t DestId : 30u;
        uint32_t ReTransmit : 1u;
        uint32_t unusedBit1 : 1u;
        unsigned char SafetyText[156u];
}) n2k_AISAddrSafety129801_t;
#endif
#define N2K_BROAD_SAFETY_PGN 129802UL                                           /* AIS Safety Related Broadcast Message */
#if defined(D_FT900)
typedef struct N2KPACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        uint32_t SourceId : 30u;
        uint32_t Spare : 2u;
        unsigned char SafetyText[161u];
} n2k_AISBroadSafety129802_t;
#else
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        uint32_t SourceId : 30u;
        uint32_t Spare : 2u;
        unsigned char SafetyText[161u];
}) n2k_AISBroadSafety129802_t;
#endif
#define N2K_AtoN_PGN 129041UL                                                   /* PGN 129041 "AIS Aids to Navigation (AtoN) Report */
#if defined(D_FT900)
typedef struct N2KPACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t MessageID : 6u;
        uint32_t UserID;
        uint8_t AtoNtype;
        unsigned char AtoNName[120u];
        uint8_t Accuracy : 1u;
        uint8_t spare1 : 7u;
        float64_t Longitude;
        float64_t Latitude;
        float64_t ShipDimensions;
        uint8_t GNSSType;
        uint32_t Timestamp;
        uint8_t offPos : 1u;
        uint8_t spare2 : 3u;
        uint8_t RAIM : 1u;
        uint8_t virtualAtoN : 1u;
        uint8_t assignedMode : 1u;
        uint8_t spare3 : 1u;
} n2k_AISAtoN129041_t;
#else
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t MessageID : 6u;
        uint32_t UserID;
        uint8_t AtoNtype;
        unsigned char AtoNName[120u];
        uint8_t Accuracy : 1u;
        uint8_t spare1 : 7u;
        float64_t Longitude;
        float64_t Latitude;
        float64_t ShipDimensions;
        uint8_t GNSSType;
        uint32_t Timestamp;
        uint8_t offPos : 1u;
        uint8_t spare2 : 3u;
        uint8_t RAIM : 1u;
        uint8_t virtualAtoN : 1u;
        uint8_t assignedMode : 1u;
        uint8_t spare3 : 1u;
}) n2k_AISAtoN129041_t;
#endif
// AIS static data class B part A
// Input:
//  - MessageID             Message type
//  - Repeat                Repeat indicator
//  - UserID                MMSI
//  - Name                  Vessel name
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_AISCLASSB_PGN 129039UL
#if defined(D_FT900)
typedef struct N2KPACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;
        uint8_t MessageID : 2u;
        uint8_t SpareBits : 4u;
        uint8_t Repeat : 2u;
        uint32_t UserID;
        float64_t Longditude;
        float64_t Latitude;
        uint8_t Accuracy : 1u;
        uint8_t RAIM : 1u;
        uint8_t Seconds : 2u;
        uint8_t SpareBits2 : 4u;
        float64_t COG;
        float64_t SOG;
        uint16_t CommStat;
        uint8_t TransInfo;
        float64_t Heading;
        uint16_t RegionalApp : 8u;
        uint16_t State : 1u;
        uint16_t spareBit : 1u;
        uint16_t Unit : 1u;
        uint16_t Display : 1u;
        uint16_t DSC : 1u;
        uint16_t Band : 1u;
        uint16_t Msg22 : 1u;
        uint16_t Mode : 1u;
} n2k_AISRepClassB129039_t;
#else
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;
        uint8_t MessageID : 2u;
        uint8_t SpareBits : 4u;
        uint8_t Repeat : 2u;
        uint32_t UserID;
        float64_t Longditude;
        float64_t Latitude;
        uint8_t Accuracy : 1u;
        uint8_t RAIM : 1u;
        uint8_t Seconds : 2u;
        uint8_t SpareBits2 : 4u;
        float64_t COG;
        float64_t SOG;
        uint16_t CommStat;
        uint8_t TransInfo;
        float64_t Heading;
        uint16_t RegionalApp : 8u;
        uint16_t State : 1u;
        uint16_t spareBit : 1u;
        uint16_t Unit : 1u;
        uint16_t Display : 1u;
        uint16_t DSC : 1u;
        uint16_t Band : 1u;
        uint16_t Msg22 : 1u;
        uint16_t Mode : 1u;
}) n2k_AISRepClassB129039_t;
#endif
#define N2K_AISCLASSBA_PGN 129809L                                              /* AIS static data class B part A */
#if defined(D_FT900)
typedef struct N2KPACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        uint32_t UserID;
        unsigned char Name[20u];
} n2k_AISStaticClassBA129809_t;
#else
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        uint32_t UserID;
        unsigned char Name[20u];
}) n2k_AISStaticClassBA129809_t;
#endif
// AIS static data class B part B
// Input:
//  - MessageID             Message type
//  - Repeat                Repeat indicator
//  - UserID                MMSI
//  - Name                  Vessel name
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_AISCLASSBB_PGN 129810UL
#if defined(D_FT900)
typedef struct N2KPACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        uint32_t UserID;
        uint8_t VesselType;
        unsigned char Vendor[7u];
        unsigned char Callsign[7u];
        float64_t Length;
        float64_t Beam;
        float64_t PosRefStbd;
        float64_t PosRefBow;
        uint32_t MothershipID;
        uint8_t byte1;
} n2k_AISStaticClassBB129810_t;
#else
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t MessageID : 6u;
        uint8_t Repeat : 2u;
        uint32_t UserID;
        uint8_t VesselType;
        unsigned char Vendor[7u];
        unsigned char Callsign[7u];
        float64_t Length;
        float64_t Beam;
        float64_t PosRefStbd;
        float64_t PosRefBow;
        uint32_t MothershipID;
        uint8_t byte1;
}) n2k_AISStaticClassBB129810_t;
#endif
// Waypoint list
// Input:
//  - Start                 The ID of the first waypoint
//    NumItems
//  - NumWaypoints          Number of valid WPs in the WP-List
//  - Database              Database ID
//  - ID                    The ID of the current waypoint
//  - Name                  The name of the current waypoint
//  - Latitude              The latitude of the current waypoint
//  - Longitude             The longitude of the current waypoint
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_WAYPOINT_PGN 130074UL
#define MAX_WAYPOINT_LEN 50u
#define MAX_NUM_OF_WAYPOINTS 100u

#if defined(D_FT900)
typedef struct N2KPACKED {
        uint16_t ID;
        uint8_t routeNameLen;                                                   // default 0x03u
        uint8_t startName;                                                      // 0x01u
        unsigned char routeName[MAX_WAYPOINT_LEN];                                              // default 0x00u
        //uint8_t endName;                                                        not seem to be there len includes itself 0xFFu
        float64_t Latitude;
        float64_t Longitude;
} n2k_WayPoint_t;
typedef struct N2KPACKED {
        uint32_t PGN;
        uint8_t priority;
        uint16_t Start;
        uint16_t NumOfItems;
        uint16_t Database;
        uint16_t Route;
        uint8_t NavDirection;
        uint8_t routeNameLen;                                                   // default 0x03u
        uint8_t startName;                                                      // 0x01u
        unsigned char routeName[MAX_WAYPOINT_LEN];
        uint8_t endName;                                                        // 0xFFu
        n2k_WayPoint_t waypoint[MAX_NUM_OF_WAYPOINTS];
} n2k_WayPointList_t;
#else
N2KPACKED (
typedef struct {
        uint16_t ID;
        uint8_t startName;                                                      // 0x01u
        unsigned char routeName[MAX_WAYPOINT_LEN];
        uint8_t endName;                                                        // 0xFFu
        float64_t Latitude;
        float64_t Longitude;
 }) n2k_WayPoint_t;
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint16_t Start;
        uint16_t NumOfItems;
        uint16_t Database;
        uint16_t Route;
        uint8_t NavDirection;
        uint8_t routeNameLen;                                                   // default 0x03u
        uint8_t startName;                                                      // 0x01u
        unsigned char routeName[MAX_WAYPOINT_LEN];
        uint8_t endName;                                                        // 0xFFu
        n2k_WayPoint_t waypoint[MAX_NUM_OF_WAYPOINTS];
 }) n2k_WayPointList_t;
#endif
// Wind Speed
// Input:
//  - SID                   Sequence ID. If your device is e.g. boat speed and wind at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - WindSpeed             Measured wind speed in m/s
//  - WindAngle             Measured wind angle in radians. If you have value in degrees, use function DegToRad(myval) in call.
//  - WindReference         Wind reference, see definition of tN2kWindReference
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_WINDSPEED_PGN 130306UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID; */
        float64_t WindSpeed;
        float64_t WindAngle;
        uint8_t WindReference;
} n2k_WindSpeed_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        float64_t WindSpeed;
        float64_t WindAngle;
        uint8_t WindReference;
 }) n2k_WindSpeed_t;
#endif
// Cross Track Error
#define N2K_CROSSTRAK_PGN 129283UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        uint8_t XTEMode : 4u;
        uint8_t Spare1 : 2u;
        uint8_t NavigationTerminated : 1u;
        uint8_t Spare2 : 1u;
        float64_t XTE;
        uint8_t byte1;
        uint8_t byte2;
} n2k_CrossTrak_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;         */
        uint8_t XTEMode : 4u;
        uint8_t Spare1 : 2u;
        uint8_t NavigationTerminated : 1u;
        uint8_t Spare2 : 1u;
        float64_t XTE;
        uint8_t byte1;
        uint8_t byte2;
 }) n2k_CrossTrak_t;
#endif
//*****************************************************************************
// Outside Environmental parameters
// Input:
//  - SID                   Sequence ID.
//  - WaterTemperature      Water temperature in K. Use function CToKelvin, if you want to use °C.
//  - OutsideAmbientAirTemperature      Outside ambient temperature in K. Use function CToKelvin, if you want to use °C.
//  - AtmosphericPressure   Atmospheric pressure in Pascals. Use function mBarToPascal, if you like to use mBar
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_OUTENV_PGN 130310UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID; */
        float64_t WaterTemperature;
        float64_t OutsideAmbientAirTemperature;
        float64_t AtmosphericPressure;
} n2k_OutEnv_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;   */
        float64_t WaterTemperature;
        float64_t OutsideAmbientAirTemperature;
        float64_t AtmosphericPressure;
 }) n2k_OutEnv_t;
#endif
// Environmental parameters
// Note that in PGN 130311 TempInstance is as TempSource in PGN 130312. I do not know why this
// renaming is confusing.
// Pressure has to be in pascal. Use function mBarToPascal, if you like to use mBar
// Input:
//  - SID                   Sequence ID.
//  - TempSource            see tN2kTempSource
//  - Temperature           Temperature in K. Use function CToKelvin, if you want to use °C.
//  - HumiditySource        see tN2kHumiditySource.
//  - Humidity              Humidity in %
//  - AtmosphericPressure   Atmospheric pressure in Pascals. Use function mBarToPascal, if you like to use mBar
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_ENV_PGN 130311UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint8_t TempSource : 2u;
        uint8_t Spare1 : 4u;
        uint8_t HumiditySource : 2u;
        float64_t Temperature;
        float64_t Humidity;
        float64_t AtmosphericPressure;
} n2k_EnvPar_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;   */
        uint8_t TempSource : 2u;
        uint8_t Spare1 : 4u;
        uint8_t HumiditySource : 2u;
        float64_t Temperature;
        float64_t Humidity;
        float64_t AtmosphericPressure;
 }) n2k_EnvPar_t;
#endif
// Temperature
// Temperatures should be in Kelvins
// Input:
//  - SID                   Sequence ID.
//  - TempInstance          This should be unic at least on one device. May be best to have it unic over all devices sending this PGN.
//  - TempSource            see tN2kTempSource
//  - ActualTemperature     Temperature in K. Use function CToKelvin, if you want to use °C.
//  - SetTemperature        Set temperature in K. Use function CToKelvin, if you want to use °C. This is meaningfull for temperatures,
//                          which can be controlled like cabin, freezer, refridgeration temperature. God can use value for this for
//                          outside and sea temperature values.
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_TEMP_PGN 130312UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;   */
        uint8_t TempInstance;
        uint8_t TempSource;
        float64_t ActualTemperature;
        float64_t SetTemperature;
        uint8_t byte1;
} n2k_Temperature_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;      */
        uint8_t TempInstance;
        uint8_t TempSource;
        float64_t ActualTemperature;
        float64_t SetTemperature;
        uint8_t byte1;
 }) n2k_Temperature_t;
#endif
// Temperature
// Temperatures should be in Kelvins
// Input:
//  - SID                   Sequence ID.
//  - TempInstance          This should be unic at least on one device. May be best to have it unic over all devices sending this PGN.
//  - TempSource            see tN2kTempSource
//  - ActualTemperature     Temperature in K. Use function CToKelvin, if you want to use °C.
//  - SetTemperature        Set temperature in K. Use function CToKelvin, if you want to use °C. This is meaningfull for temperatures,
//                          which can be controlled like cabin, freezer, refridgeration temperature. God can use value for this for
//                          outside and sea temperature values.
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
#define N2K_TEMPEXT_PGN 130316UL

// Humidity
// Humidity should be a percent
// Input:
//  - SID                   Sequence ID.
//  - HumidityInstance      This should be unic at least on one device. May be best to have it unic over all devices sending this PGN.
//  - HumiditySource        see tN2kHumiditySource
//  - Humidity              Humidity in percent
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_HUMID_PGN 130313UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        uint8_t HumidityInstance;
        uint8_t HumiditySource;
        float64_t ActualHumidity;
        float64_t SetHumidity;
        uint8_t byte1;
} n2k_Humidity_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;   */
        uint8_t HumidityInstance;
        uint8_t HumiditySource;
        float64_t ActualHumidity;
        float64_t SetHumidity;
        uint8_t byte1;
 }) n2k_Humidity_t;
#endif
// Pressure
// Pressures should be in Pascals
// Input:
//  - SID                   Sequence ID.
//  - PressureInstance      This should be unic at least on one device. May be best to have it unic over all devices sending this PGN.
//  - PressureSource        see tN2kPressureSource
//  - Pressure              Pressure in Pascals. Use function mBarToPascal, if you like to use mBar
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_PRESS_PGN 130314UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint8_t PressureInstance;
        uint8_t PressureSource;
        float64_t ActualPressure;
        uint8_t byte1;
} n2k_Pressure_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        uint8_t PressureInstance;
        uint8_t PressureSource;
        float64_t ActualPressure;
        uint8_t byte1;
 }) n2k_Pressure_t;
#endif
// Set pressure
// Pressures should be in Pascals
// Input:
//  - SID                       Sequence ID.
//  - PressureInstance          This should be unic at least on one device. May be best to have it unic over all devices sending this PGN.
//  - PressureSource            see tN2kPressureSource
//  - Set pressure              Set pressure in Pascals. Use function mBarToPascal, if you like to use mBar
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_SETPRESS_PGN 130315UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;  */
        uint8_t PressureInstance;
        uint8_t PressureSource;
        float64_t SetPressure;
        uint8_t byte1;
} n2k_SetPressure_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;   */
        uint8_t PressureInstance;
        uint8_t PressureSource;
        float64_t SetPressure;
        uint8_t byte1;
 }) n2k_SetPressure_t;
#endif

//*****************************************************************************
// Small Craft Status (Trim Tab Position)
// Trim tab position is a percentage 0 to 100% where 0 is fully retracted and 100 is fully extended
// Input:
//  - PortTrimTab           Port trim tab position
//  - StbdTrimTab           Starboard trim tab position
//
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
#define N2K_TRIMTAB_PGN 130576UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;   */
        uint8_t PortTrimTab;
        uint8_t StbdTrimTab;
        uint16_t Word1;
        uint32_t dWord1;
} n2k_TrimTab_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;   */
        uint8_t PortTrimTab;
        uint8_t StbdTrimTab;
        uint16_t Word1;
        uint32_t dWord1;
 }) n2k_TrimTab_t;
#endif
//*****************************************************************************
// Navigation Information
//
// Input: As per below structure
//
// Output:
//  - N2kMsg NMEA2000 message ready to be send.
#define N2K_NAV_INFO_PGN 129284UL
#if defined(D_FT900)
typedef struct N2KPACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;
        float64_t DistanceToWaypoint;
        uint8_t BearingReference : 1u;
        uint8_t spare1 : 1u;
        uint8_t PerpendicularCrossed : 1u;
        uint8_t spare2 : 1u;
        uint8_t ArrivalCircleEntered : 1u;
        uint8_t spare3 : 1u;
        uint8_t CalculationType : 1u;                                           // N2kdct_RhumbLine : N2kdct_GreatCircle
        uint8_t spare4 : 1u;
        float64_t ETATime;
        float64_t ETADate;
        float64_t BearingOriginToDestinationWaypoint;
        float64_t BearingPositionToDestinationWaypoint;
        uint32_t OriginWaypointNumber;
        uint32_t DestinationWaypointNumber;
        float64_t DestinationLatitude;
        float64_t DestinationLongitude;
        float64_t WaypointClosingVelocity;
} n2k_NavInfo_t;
#else
N2KPACKED (
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;
        float64_t DistanceToWaypoint;
        uint8_t BearingReference : 1u;
        uint8_t spare1 : 1u;
        uint8_t PerpendicularCrossed : 1u;
        uint8_t spare2 : 1u;
        uint8_t ArrivalCircleEntered : 1u;
        uint8_t spare3 : 1u;
        uint8_t CalculationType : 1u;                                           // N2kdct_RhumbLine = 1 : N2kdct_GreatCircle = 0
        uint8_t spare4 : 1u;
        float64_t ETATime;
        float64_t ETADate;
        float64_t BearingOriginToDestinationWaypoint;
        float64_t BearingPositionToDestinationWaypoint;
        uint32_t OriginWaypointNumber;
        uint32_t DestinationWaypointNumber;
        float64_t DestinationLatitude;
        float64_t DestinationLongitude;
        float64_t WaypointClosingVelocity;
 }) n2k_NavInfo_t;
#endif
#if defined(D_FT900)                                                            /* This structure is needed if you use multi frame receive - probably not needed here but not sure */
typedef struct N2KPACKED {
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_NavInfo_t navigData;                                               /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
} N2K_NaviInfoRcv_t;
#else
N2KPACKED (
typedef struct
{
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus N2k read */
  unsigned char DataBuf[FASTPACKET_MAX_SIZE];                                   /* the n2k raw byte buffer received */
  n2k_NavInfo_t navigData;                                             /* the actual formatted data last received */
  int64_t timePeriod;
  uint64_t timeRef;
  uint8_t State;
}) N2K_NaviInfoRcv_t;
#endif
//*****************************************************************************
// Rudder
// Input:
// - RudderPosition         Current rudder postion in radians.
// - Instance               Rudder instance.
// - RudderDirectionOrder   See tN2kRudderDirectionOrder. Direction, where rudder should be turned.
// - AngleOrder             In radians angle where rudder should be turned.
// Output:
//  - N2kMsg                NMEA2000 message ready to be send.
//*****************************************************************************
#define N2K_RUDDER_PGN 127245UL
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        uint8_t Instance;
        uint8_t RudderDirectionOrder : 3u;
        uint8_t spare1 : 5u;
        float64_t AngleOrder;
        float64_t RudderPosition;
        uint8_t spare2;                                                         // set to 0xfflu
        uint8_t spare3;                                                         // set to 0xfflu
} n2k_Rudder_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
        uint8_t Instance;
        uint8_t RudderDirectionOrder : 3u;
        uint8_t spare1 : 5u;
        float64_t AngleOrder;
        float64_t RudderPosition;
        uint8_t spare2;                                                         // set to 0xfflu
        uint8_t spare3;                                                         // set to 0xfflu
 }) n2k_Rudder_t;
#endif

#define N2K_PROD_INFO_PGN 126996LU
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
       uint16_t N2kVersion;
       uint16_t ProductCode;
       uint8_t N2kModelID;
       uint8_t N2kSwCode;
       uint8_t N2kModelVersion;
       uint8_t N2kModelSerialCode;
       uint8_t CertificationLevel;
       uint8_t LoadEquivalency;
} n2k_ProdInfo_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
       uint16_t N2kVersion;
       uint16_t ProductCode;
       uint8_t N2kModelID;
       uint8_t N2kSwCode;
       uint8_t N2kModelVersion;
       uint8_t N2kModelSerialCode;
       uint8_t CertificationLevel;
       uint8_t LoadEquivalency;
 }) n2k_ProdInfo_t;
#endif

#define N2K_ISO_ACK_PGN 59392LU
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
       uint8_t Control;
       uint8_t Groupfunction;
       uint8_t Res1;
       uint8_t Res2;
       uint32_t Res3 : 8u;
       uint32_t PGNNum : 24u;
} n2k_ISOAck_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
       uint8_t Control;
       uint8_t Groupfunction;
       uint8_t Res1;
       uint8_t Res2;
       uint32_t Res3 : 8u;
       uint32_t PGNNum : 24u;
 }) n2k_ISOAck_t;
#endif

#define N2K_ISO_ADDR_CLAIM_PGN 60928LU
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
       uint32_t UniqueNumber : 21u;
       uint32_t ManufacturerCode : 11u;
       uint8_t DeviceInstance;
       uint8_t DeviceFunction;
       uint8_t spare1 : 1u;
       uint8_t DeviceClass : 7u;
       uint8_t SystemInstance : 4u;
       uint8_t IndustryGroup : 3u;
       uint8_t spare2 : 1u;
} n2k_ISOAddrClaim_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
       uint32_t UniqueNumber : 21u;
       uint32_t ManufacturerCode : 11u;
       uint8_t DeviceInstance;
       uint8_t DeviceFunction;
       uint8_t spare1 : 1u;
       uint8_t DeviceClass : 7u;
       uint8_t SystemInstance : 4u;
       uint8_t IndustryGroup : 3u;
       uint8_t spare2 : 1u;
 }) n2k_ISOAddrClaim_t;
#endif

#define N2K_HEARTBEAT_PGN 126993LU
#define N2K_MAXTIME_HEART 65532LU
#if defined(D_FT900)
typedef struct N2KPACKED {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
       uint16_t timeCode;
       uint8_t status;
       uint8_t spare1;                                                          // 0xff
       uint32_t spare2;                                                         // 0xffffffff
} n2k_heartBeat_t;
#else
N2KPACKED (
typedef struct {
/*        uint32_t PGN;
        uint8_t priority;
        uint8_t SID;    */
       uint16_t timeCode;
       uint8_t status;
       uint8_t spare1;                                                          // 0xff
       uint32_t spare2;                                                         // 0xffffffff
 }) n2k_heartBeat_t;
#endif

#define N2K_PROD_INFOR_PGN 126996LU                                             /* product information PGN */
#if defined(D_FT900)
typedef struct N2KPACKED {
         unsigned char ManModSerial[32u];         //  Default="00000001". Max 32 chars. Manufacturer's Model serial code
         uint16_t ProductCode;                    //  =0xffff Default=666. Manufacturer's product code
         unsigned char ModelID[32u];              // Default="PIC32 N2k->PIC->N2k". Max 33 chars. Manufacturer's  Model ID
         unsigned char SwCode[40u];               // Default="1.0.0.0". Max 40 chars. Manufacturer's software version code
         unsigned char ModelVersion[24u];         // Default="1.0.0". Max 24 chars. Manufacturer's Model version
         uint16_t  LoadEquivalency;               //=0xff,   Default=1. x * 50 mA
         uint16_t N2kVersion;                     // =0xffff,  Default=2101
         uint16_t CertificationLevel;             // Default=1
         int16_t iDev;
} N2K_ModelSerialCode_t;
#else
N2KPACKED (
typedef struct
{
         unsigned char ManModSerial[32u];         //  Default="00000001". Max 32 chars. Manufacturer's Model serial code
         uint16_t ProductCode;                    //  =0xffff Default=666. Manufacturer's product code
         unsigned char ModelID[32u];              // Default="PIC32 N2k->PIC->N2k". Max 33 chars. Manufacturer's  Model ID
         unsigned char SwCode[40u];               // Default="1.0.0.0". Max 40 chars. Manufacturer's software version code
         unsigned char ModelVersion[24u];         // Default="1.0.0". Max 24 chars. Manufacturer's Model version
         uint16_t  LoadEquivalency;               //=0xff,   Default=1. x * 50 mA
         uint16_t N2kVersion;                     // =0xffff,  Default=2101
         uint16_t CertificationLevel;             // Default=1
         int16_t iDev;
}) N2K_ModelSerialCode_t;
#endif

#define N2K_PGN_LIST_PGN 126464LU
#if defined(D_FT900)
typedef struct N2KPACKED {
         uint8_t tr;
         uint8_t PGNNum[3u];                                                    /* uint24_t */
} N2K_pgnList_t;
#else
N2KPACKED (
typedef struct
{
         uint8_t tr;
         uint8_t PGNNum[3u];                                                    /* uint24_t */
}) N2K_pgnList_t;
#endif

#define N2K_DATE_LENGTH 60u                                                     // 1970-01-01-00:00:00.000,6,59904,0,255,3,14,f0,01

#if defined(D_FT900)
typedef struct N2KPACKED {
  char timestamp[N2K_DATE_LENGTH];
  uint8_t  prio;
  uint32_t pgn;
  uint8_t  dst;
  uint8_t  src;
  uint8_t  len;
  uint8_t  dataV[FASTPACKET_MAX_SIZE];
} N2K_RawMessage_t;
#else
N2KPACKED (
typedef struct
{
  char timestamp[N2K_DATE_LENGTH];
  uint8_t  prio;
  uint32_t pgn;
  uint8_t  dst;
  uint8_t  src;
  uint8_t  len;
  uint8_t  dataV[FASTPACKET_MAX_SIZE];
}) N2K_RawMessage_t;
#endif

typedef union {
    uint8_t ty1ByteUInt;
    int16_t ty2ByteInt;
    uint16_t ty2ByteUInt;
    uint32_t ty3ByteUInt;
    uint32_t ty4ByteUInt;
    uint64_t ty8ByteUInt;
    float64_t ty1ByteDouble;
    float64_t ty1ByteUDouble;
    float64_t ty2ByteDouble;
    float64_t ty2ByteUDouble;
    float64_t ty8ByteDouble;
    float64_t ty3ByteDouble;
    float64_t ty4ByteDouble;
    float64_t ty4ByteUDouble;
    unsigned char ty7ByteStr[7u];
    unsigned char ty20ByteStr[20u];
    unsigned char tyWayPoint[MAX_WAYPOINT_LEN];
  } N2k_Type_u;

#if defined(D_FT900)
typedef struct N2KPACKED {
   N2k_Type_u n2kfield;                                               /* field read from a n2k message */
} n2k_data_obj_t;
#else
N2KPACKED (
typedef struct {
   N2k_Type_u n2kfield;                                               /* field read from a n2k message */
}) n2k_data_obj_t;
#endif

#if defined(D_FT900)
typedef struct N2KPACKED {
  uint8_t FullLen;                                                      /* data length plus 11 */
  uint8_t priority;
  uint32_t PGN;
  uint8_t dst;
  uint8_t src;
  uint32_t messageTyp;
  N2K_FlowRateRcv_t FFM100FRate;
  maretron_FluidVol_t FFM100FVol;
  N2K_MarTempRcv_t TMP100Tmp;
  n2k_time_t TimeObj;
  n2k_vesselHeading_t vessHead;
  n2k_rateOfturn_t rateOfTurn;
  n2k_attitude_t attitude;
  n2k_magVar_t magnetVar;
  n2k_engineRapid_t engRapid;
  n2k_transmissDyn_t transDynam;
  n2k_EngTripDyn_t engTripDyn;
  n2k_FluidLvl_t fluidLvl;
  n2k_dc_detail_t dcDetail;
  n2k_charger_stat_t chargerStat;
  n2k_lat_lon_t latLonPos;
  n2k_bat_stat_t batStatus;
  n2k_Leeway_t leeWay;
  n2k_BoatSpeed_t boatSpeed;
  n2k_CogSog_t cogSog;
  N2K_GNSSPosRcv_t gnssPos;
  n2k_dateTim_t dateDate;
  n2k_GNSSDOP_t gnssDop;
  N2K_AISRepClassA129038Rcv_t aisClassA;
  N2K_AISSARAirRep129798Rcv_t airCraft;
  N2K_BatteryConfigRcv_t battery;
  n2k_WindSpeed_t windSpeed;
  n2k_CrossTrak_t crossTrkErr;
  n2k_OutEnv_t outsideEnv;
  n2k_EnvPar_t environ;
  n2k_Temperature_t temperat;
  n2k_Humidity_t humid;
  n2k_Pressure_t press;
  n2k_SetPressure_t setPress;
  n2k_TrimTab_t smallCraftStat;
  n2k_Rudder_t rudder;
  N2K_NaviInfoRcv_t navInfo;
  N2K_EngineDynRcv_t engDynamics;
} N2K_Receiver_t;
#else
N2KPACKED (
typedef struct
{
  uint8_t FullLen;
  uint8_t priority;
  uint32_t PGN;
  uint8_t dst;
  uint8_t src;
  uint32_t messageTyp;
  N2K_FlowRateRcv_t FFM100FRate;
  maretron_FluidVol_t FFM100FVol;
  N2K_MarTempRcv_t TMP100Tmp;
  n2k_time_t TimeObj;
  n2k_vesselHeading_t vessHead;
  n2k_rateOfturn_t rateOfTurn;
  n2k_attitude_t attitude;
  n2k_magVar_t magnetVar;
  n2k_engineRapid_t engRapid;
  n2k_transmissDyn_t transDynam;
  n2k_EngTripDyn_t engTripDyn;
  n2k_FluidLvl_t fluidLvl;
  n2k_dc_detail_t dcDetail;
  n2k_charger_stat_t chargerStat;
  n2k_lat_lon_t latLonPos;
  n2k_bat_stat_t batStatus;
  n2k_Leeway_t leeWay;
  n2k_BoatSpeed_t boatSpeed;
  n2k_CogSog_t cogSog;
  N2K_GNSSPosRcv_t gnssPos;
  n2k_dateTim_t dateDate;
  n2k_GNSSDOP_t gnssDop;
  N2K_AISRepClassA129038Rcv_t aisClassA;
  N2K_AISSARAirRep129798Rcv_t airCraft;
  N2K_BatteryConfigRcv_t battery;
  n2k_WindSpeed_t windSpeed;
  n2k_CrossTrak_t crossTrkErr;
  n2k_OutEnv_t outsideEnv;
  n2k_EnvPar_t environ;
  n2k_Temperature_t temperat;
  n2k_Humidity_t humid;
  n2k_Pressure_t press;
  n2k_SetPressure_t setPress;
  n2k_TrimTab_t smallCraftStat;
  n2k_Rudder_t rudder;
  N2K_NaviInfoRcv_t navInfo;
  N2K_EngineDynRcv_t engDynamics;
}) N2K_Receiver_t;
#endif

typedef enum {typ2ByteInt = 0u, typ2ByteUInt = 1u, typ3ByteUInt=2u, typ4ByteUInt=3u, typ8ByteUInt=4u, typ1ByteDouble=5u, typ1ByteUDouble=6u, typ2ByteDouble=7u, typ2ByteUDouble=8u, typ8ByteDouble=9u, typ3ByteDouble=10u, typ4ByteDouble=11u, typ4ByteUDouble=12u, typ1ByteUint=13u, typ7ByteStr=14u, typ20ByteStr=15u, typWayPoint=16u, NumOfN2KDataTypes=17u } N2K_Data_Type_e;

/* ================= N2K enumerated typess =================================  */
enum tN2kHeadingReference {
                            N2khr_true=0,
                            N2khr_magnetic=1,
                            N2khr_error=2,
                            N2khr_Unavailable=3
                          };
enum tN2kDistanceCalculationType {
                            N2kdct_GreatCircle=0,
                            N2kdct_RhumbLine=1
                          };
enum tN2kXTEMode  {
                            N2kxtem_Autonomous=0,
                            N2kxtem_Differential=1,
                            N2kxtem_Estimated=2,
                            N2kxtem_Simulator=3,
                            N2kxtem_Manual=4
                          };
enum tN2kGNSStype {
                            N2kGNSSt_GPS=0,
                            N2kGNSSt_GLONASS=1,
                            N2kGNSSt_GPSGLONASS=2,
                            N2kGNSSt_GPSSBASWAAS=3,
                            N2kGNSSt_GPSSBASWAASGLONASS=4,
                            N2kGNSSt_Chayka=5,
                            N2kGNSSt_integrated=6,
                            N2kGNSSt_surveyed=7,
                            N2kGNSSt_Galileo=8
                          };
enum tN2kGNSSmethod {
                            N2kGNSSm_noGNSS=0,
                            N2kGNSSm_GNSSfix=1,
                            N2kGNSSm_DGNSS=2,
                            N2kGNSSm_PreciseGNSS=3,
                            N2kGNSSm_RTKFixed=4,
                            N2kGNSSm_RTKFloat=5,
                            N2kGNSSm_Error=14,
                            N2kGNSSm_Unavailable=15
                          };

enum tN2kGNSSDOPmode {
                            N2kGNSSdm_1D=0,
                            N2kGNSSdm_2D=1,
                            N2kGNSSdm_3D=2,
                            N2kGNSSdm_Auto=3,
                            N2kGNSSdm_Reserved=4,
                            N2kGNSSdm_Reserved2=5,
                            N2kGNSSdm_Error=6,
                            N2kGNSSdm_Unavailable=7
                          };

enum tN2kTempSource {
                            N2kts_SeaTemperature=0,
                            N2kts_OutsideTemperature=1,
                            N2kts_InsideTemperature=2,
                            N2kts_EngineRoomTemperature=3,
                            N2kts_MainCabinTemperature=4,
                            N2kts_LiveWellTemperature=5,
                            N2kts_BaitWellTemperature=6,
                            N2kts_RefridgerationTemperature=7,
                            N2kts_HeatingSystemTemperature=8,
                            N2kts_DewPointTemperature=9,
                            N2kts_ApparentWindChillTemperature=10,
                            N2kts_TheoreticalWindChillTemperature=11,
                            N2kts_HeatIndexTemperature=12,
                            N2kts_FreezerTemperature=13,
                            N2kts_ExhaustGasTemperature=14
                          };

enum tN2kHumiditySource {
                            N2khs_InsideHumidity=0,
                            N2khs_OutsideHumidity=1,
                            N2khs_Undef=0xff
                          };

enum tN2kPressureSource {
                            N2kps_Atmospheric = 0,
                            N2kps_Water = 1,
                            N2kps_Steam = 2,
                            N2kps_CompressedAir = 3,
                            N2kps_Hydraulic = 4
                          };

enum tN2kTimeSource {
                            N2ktimes_GPS=0,
                            N2ktimes_GLONASS=1,
                            N2ktimes_RadioStation=2,
                            N2ktimes_LocalCesiumClock=3,
                            N2ktimes_LocalRubidiumClock=4,
                            N2ktimes_LocalCrystalClock=5
                          };

enum tN2kFluidType {
                            N2kft_Fuel=0,
                            N2kft_Water=1,
                            N2kft_GrayWater=2,
                            N2kft_LiveWell=3,
                            N2kft_Oil=4,
                            N2kft_BlackWater=5,
                            N2kft_FuelGasoline=6,
                            N2kft_Error=14,
                            N2kft_Unavailable=15
                          };

enum tN2kWindReference {                                                          // Details found on page 12 of https://www.rocktheboatmarinestereo.com/specs/MSNRX200I.pdf
                            N2kWind_True_North=0,                               // Theoretical Wind (ground referenced, referenced to True North; calculated using COG/SOG)
                            N2kWind_Magnetic=1,                                 // Theoretical Wind (ground referenced, referenced to Magnetic North; calculated using COG/SOG)
                            N2kWind_Apparent=2,                                 // Apparent Wind (relative to the vessel centerline)
                            N2kWind_Apprent=2,                                  // Deprecated - We had the typo in older version of the library
                            N2kWind_True_boat=3,                                // Theoretical (Calculated to Centerline of the vessel, referenced to ground; calculated using COG/SOG)
                            N2kWind_True_water=4,                               // Theoretical (Calculated to Centerline of the vessel, referenced to water; calculated using Heading/Speed through Water)
                            N2kWind_Error=6,
                            N2kWind_Unavailable=7
                          };

enum tN2kSpeedWaterReferenceType {
                            N2kSWRT_Paddle_wheel=0,
                            N2kSWRT_Pitot_tube=1,
                            N2kSWRT_Doppler_log=2,
                            N2kSWRT_Ultra_Sound=3,
                            N2kSWRT_Electro_magnetic=4,
                            N2kSWRT_Error=254,
                            N2kSWRT_Unavailable=255
                          };

enum tN2kRudderDirectionOrder {
                            N2kRDO_NoDirectionOrder=0,
                            N2kRDO_MoveToStarboard=1,
                            N2kRDO_MoveToPort=2,
                            N2kRDO_Unavailable=7
                          };

enum tN2kDCType {
                            N2kDCt_Battery=0,
                            N2kDCt_Alternator=1,
                            N2kDCt_Converter=2,
                            N2kDCt_SolarCell=3,
                            N2kDCt_WindGenerator=4
                          };

enum tN2kBatType  {
                            N2kDCbt_Flooded=0,
                            N2kDCbt_Gel=1,
                            N2kDCbt_AGM=2
                          };

enum tN2kBatEqSupport  {
                            N2kDCES_No=0,                                       // No, Off, Disabled
                            N2kDCES_Yes=1,                                      // Yes, On, Enabled
                            N2kDCES_Error=2,                                    // Error
                            N2kDCES_Unavailable=3                               // Unavailable
                          };

enum tN2kBatChem {
                            N2kDCbc_LeadAcid=0,
                            N2kDCbc_LiIon=1,
                            N2kDCbc_NiCad=2,
                            N2kDCbc_NiMh=3
                          };

enum tN2kBatNomVolt {
                            N2kDCbnv_6v=0,
                            N2kDCbnv_12v=1,
                            N2kDCbnv_24v=2,
                            N2kDCbnv_32v=3,
                            N2kDCbnv_62v=4,
                            N2kDCbnv_42v=5,
                            N2kDCbnv_48v=6
                          };

enum tN2kTransmissionGear {
                            N2kTG_Forward=0,
                            N2kTG_Neutral=1,
                            N2kTG_Reverse=2,
                            N2kTG_Unknown=3,
                          };

enum tN2kAISRepeat {
                            N2kaisr_Initial=0,
                            N2kaisr_First=1,
                            N2kaisr_Second=2,
                            N2kaisr_Final=3,
                          };

enum tN2kAISVersion {
                            N2kaisv_ITU_R_M_1371_1=0,
                            N2kaisv_ITU_R_M_1371_3=1,
                          };

enum tN2kAISTranceiverInfo {
                            N2kaisti_Channel_A_VDL_reception=0,
                            N2kaisti_Channel_B_VDL_reception=1,
                            N2kaisti_Channel_A_VDL_transmission=2,
                            N2kaisti_Channel_B_VDL_transmission=3,
                            N2kaisti_Own_information_not_broadcast=4,
                            N2kaisti_Reserved=5
                          };

enum tN2kAISNavStatus {
                            N2kaisns_Under_Way_Motoring=0,
                            N2kaisns_At_Anchor=1,
                            N2kaisns_Not_Under_Command=2,
                            N2kaisns_Restricted_Manoeuverability=3,
                            N2kaisns_Constrained_By_Draught=4,
                            N2kaisns_Moored=5,
                            N2kaisns_Aground=6,
                            N2kaisns_Fishing=7,
                            N2kaisns_Under_Way_Sailing=8,
                            N2kaisns_Hazardous_Material_High_Speed=9,
                            N2kaisns_Hazardous_Material_Wing_In_Ground=10,
                            N2kaisns_AIS_SART=14,
                          };

enum tN2kAISDTE {
                            N2kaisdte_Ready=0,
                            N2kaisdte_NotReady=1,
                          };

enum tN2kAISUnit {
                            N2kaisunit_ClassB_SOTDMA=0,
                            N2kaisunit_ClassB_CS=1,
                          };

enum tN2kAISMode {
                            N2kaismode_Autonomous=0,
                            N2kaismode_Assigned=1,
                          };
enum tN2kMagneticVariation {
                            N2kmagvar_Manual=0,
                            N2kmagvar_Chart=1,
                            N2kmagvar_Table=2,
                            N2kmagvar_Calc=3,
                            N2kmagvar_WMM2000=4,
                            N2kmagvar_WMM2005=5,
                            N2kmagvar_WMM2010=6,
                            N2kmagvar_WMM2015=7,
                            N2kmagvar_WMM2020=8,
                          };

enum tN2kOnOff  {
                            N2kOnOff_Off=0,                                     // No, Off, Disabled
                            N2kOnOff_On=1,                                      // Yes, On, Enabled
                            N2kOnOff_Error=2,                                   // Error
                            N2kOnOff_Unavailable=3                              // Unavailable
                          };

enum tN2kChargeState  {
                            N2kCS_Not_Charging=0,
                            N2kCS_Bulk=1,
                            N2kCS_Absorption=2,
                            N2kCS_Overcharge=3,
                            N2kCS_Equalise=4,
                            N2kCS_Float=5,
                            N2kCS_No_Float=6,
                            N2kCS_Constant_VI=7,
                            N2kCS_Disabled=8,
                            N2kCS_Fault=9,
                            N2kCS_Unavailable=15
                          };

enum tN2kChargerMode {
                            N2kCM_Standalone=0,
                            N2kCM_Primary=1,
                            N2kCM_Secondary=2,
                            N2kCM_Echo=3,
                            N2kCM_Unavailable=15
                          };

enum tN2kSteeringMode {
                            N2kSM_MainSteering=0,
                            N2kSM_NonFollowUpDevice=1,
                            N2kSM_FollowUpDevice=2,
                            N2kSM_HeadingControlStandalone=3,
                            N2kSM_HeadingControl=4,
                            N2kSM_TrackControl=5,
                            N2kSM_Unavailable=7
                          };

enum tN2kTurnMode {
                            N2kTM_RudderLimitControlled=0,
                            N2kTM_TurnRateControlled=1,
                            N2kTM_RadiusControlled=2,
                            N2kTM_Unavailable=7
                          };
enum tN2kGroupFunctionCode {
                            N2kgfc_Request=0,
                            N2kgfc_Command=1,
                            N2kgfc_Acknowledge=2,
                            N2kgfc_Read=3,
                            N2kgfc_ReadReply=4,
                            N2kgfc_Write=5,
                            N2kgfc_WriteReply=6
                          };

enum tN2kGroupFunctionPGNErrorCode {
                            N2kgfPGNec_Acknowledge=0,
                            N2kgfPGNec_PGNNotSupported=1,
                            N2kgfPGNec_PGNTemporarilyNotAvailable=2,
                            N2kgfPGNec_AccessDenied=3,
                            N2kgfPGNec_RequestOrCommandNotSupported=4,
                            N2kgfPGNec_DefinerTagNotSupported=5,
                            N2kgfPGNec_ReadOrWriteNotSupported=6
                          };

enum tN2kGroupFunctionTransmissionOrPriorityErrorCode {
                            N2kgfTPec_Acknowledge=0,
                            N2kgfTPec_TransmitIntervalOrPriorityNotSupported=1,
                            N2kgfTPec_TransmitIntervalIsLessThanMeasurementInterval=2,
                            N2kgfTPec_AccessDenied=3,
                            N2kgfTPec_RequestNotSupported=4
                          };

enum tN2kGroupFunctionParameterErrorCode {
                            N2kgfpec_Acknowledge=0,
                            N2kgfpec_InvalidRequestOrCommandParameterField=1,
                            N2kgfpec_TemporarilyUnableToComply=2,
                            N2kgfpec_RequestOrCommandParameterOutOfRange=3,
                            N2kgfpec_AccessDenied=4,
                            N2kgfpec_RequestOrCommandNotSupported=5,
                            N2kgfpec_ReadOrWriteIsNotSupported=6
                          };
#define N2K_MAX_DATA_LEN 223u

/* ----------------------- airmar sensor -----------------------------------  */
#define AIRMAR_SENSOR_PORT 2597u
#define AIRMAR_SPEED_CAL_RGF "5 126208 1=0087 3=04 4=29"                        /* request group function */
#define AIRMAR_REQ_ACCESS_RGF "5 65287 1=0087 3=04"
#define AIRMAR_SET_ACCESS_CGF "5 65287 1=0087 3=04 4=01 5="                     /* command group function */
/* ---------------------- NMEA 0183 ---------------------------------------- */
/*
 * NMEA 2000 is all ISO units, so m/s, deg K, etc. except for 'degrees',
 *
 * NMEA 0183 uses various units, including metric derived and colonial.
 */
#define SPEED_M_S_TO_KNOTS(s) (s * 1.94384)
#define SPEED_M_S_TO_KMH(s) ((s) *3.6)
#define DIST_M_TO_KM(d) ((d) / 1000.0)
#define DIST_M_TO_NM(d) ((d) / 1852.0)

// For temperature, original contributor (Julius Pabrinkis) asserts that his
// DST800 shows value in Celcius, but I (Kees) really doubt this.
// By checking for a 'ridiculous' value in kelvin, we can have our cake and eat it.
// Anything below 173 deg K is assumed to be really in Celcius.
#define TEMP_K_TO_C(t) (((t) < 173.15) ? (t) : ((t) -273.15))

#define PGN_VESSEL_HEADING (127250U)
#define PGN_WIND_DATA (130306U)
#define PGN_WATER_DEPTH (128267U)
#define PGN_WATER_SPEED (128259U)
#define PGN_ENVIRONMENTAL (130311U)
#define PGN_DISTANCE_LOG (128275U)
#define PGN_RUDDER (127245U)
#define PGN_SOG_COG (129026U)
#define PGN_GPS_DOP (129539U)
#define PGN_POSITION (129029U)
#define PGN_POSITION_RAPID (129025U)
#define PGN_AIS_A (129038U)
#define PGN_AIS_B (129039U)
#define PGN_AIS_4 (129793U)
#define PGN_AIS_5 (129794U)
#define PGN_AIS_9 (129798U)
#define PGN_AIS_12 (129801U)
#define PGN_AIS_14 (129802U)
#define PGN_AIS_19 (129040U)
#define PGN_AIS_21 (129041U)
#define PGN_AIS_24A (129809U)
#define PGN_AIS_24B (129810U)

#endif /* NMEA 2000 protocol */

enum tNMEA0183Mode {
  RATE_NO_LIMIT       = -1,
  RATE_VESSEL_HEADING = 0,
  RATE_WIND_DATA,
  RATE_WATER_DEPTH,
  RATE_WATER_SPEED,
  RATE_RUDDER,
  RATE_GPS_SPEED,
  RATE_GPS_DOP,
  RATE_GPS_POSITION,
  RATE_ENVIRONMENTAL,
  RATE_DISTANCE_LOG,
  RATE_COUNT
                   };


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%% NMEA0183 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* message definition charactures */
#define NMEA0183_CR 0x0du                                                       /* 13        Carriage return */
#define NMEA0183_EndDelim 0x0au                                                 /* 10        Line feed, end delimiter */
#define NMEA0183_StartSentance 0x21u                                            /* ! 33        Start of encapsulation sentence delimiter */
#define NMEA0183_StartDelim 0x24u                                               /* $ 36        Start delimiter */
#define NMEA0183_StartChecksum 0x2au                                            /* * 42        Checksum delimiter */
#define NMEA0183_FieldDelim 0x2cu                                               /* , 44        Field delimiter  */
#define NMEA0183_TagBlockDelim 0x5cu                                            /* \ 92        TAG block delimiter */
#define NMEA0183_CodeDelim 0x5eu                                                /* ^ 94        Code delimiter for HEX representation of ISO/IEC 8859-1 (ASCII) characters */
#define NMEA0183_Reserved 0x7eu                                                 /* ~ 126        Reserved */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end MARINE_PROTO */

#endif /* end ActiSense library */