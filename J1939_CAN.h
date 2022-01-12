#ifndef __j1939_h
#define __j1939_h
//    J1939 ================== CAnBus library ==================================
//
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#if defined(J1939_USED)
//
#include "definitions.h"                                                        /* global defines are here */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define J1939PACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define J1939PACKED __attribute__((packed)) ALIGNED(1)                        /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define J1939PACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define J1939PACKED
#else                                                                           /* for MPLAB PIC32 */
  #define J1939PACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

/*
 * linux/can.h
 *
 * Definitions for CAN network layer (socket addr / CAN frame / CAN filter)
 *
 * Authors: Oliver Hartkopp <oliver.hartkopp@volkswagen.de>
 *          Urs Thuermann   <urs.thuermann@volkswagen.de>
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved. */
/* reserved addresses */
#define J1939_NULL_ADDR 254u                                                    /* The device is configured as Single Address Capable device (AAC = 0): the device sends the Cannot Claim Address message using the NULL address (254), with a pseudo-random delay between 0 and 153ms. The device cannot send any other messages other than the Cannot Claim Address message. Regular network communications are suspended. */
#define J1939_GLOBAL_ADDR 255u                                                  /* broadcast address */

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U                                                /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U                                                /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U                                                /* error message frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU                                                /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU                                                /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU                                                /* omit EFF, RTR, ERR flags */

/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28        : CAN identifier (11/29 bit)
 * bit 29        : error message frame flag (0 = data frame, 1 = error message)
 * bit 30        : remote transmission request flag (1 = rtr frame)
 * bit 31        : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
#define CAN_ERR_DLC 8 /* dlc for error message frames */

/* error class (mask) in can_id */
#define CAN_ERR_TX_TIMEOUT   0x00000001U /* TX timeout (by netdevice driver) */
#define CAN_ERR_LOSTARB      0x00000002U /* lost arbitration    / data[0]    */
#define CAN_ERR_CRTL         0x00000004U /* controller problems / data[1]    */
#define CAN_ERR_PROT         0x00000008U /* protocol violations / data[2..3] */
#define CAN_ERR_TRX          0x00000010U /* transceiver status  / data[4]    */
#define CAN_ERR_ACK          0x00000020U /* received no ACK on transmission */
#define CAN_ERR_BUSOFF       0x00000040U /* bus off */
#define CAN_ERR_BUSERROR     0x00000080U /* bus error (may flood!) */
#define CAN_ERR_RESTARTED    0x00000100U /* controller restarted */

/* arbitration lost in bit ... / data[0] */
#define CAN_ERR_LOSTARB_UNSPEC   0x00u                                          /* unspecified */
                                      /* else bit number in bitstream */

/* error status of CAN-controller / data[1] */
#define CAN_ERR_CRTL_UNSPEC      0x00u                                          /* unspecified */
#define CAN_ERR_CRTL_RX_OVERFLOW 0x01u                                          /* RX buffer overflow */
#define CAN_ERR_CRTL_TX_OVERFLOW 0x02u                                          /* TX buffer overflow */
#define CAN_ERR_CRTL_RX_WARNING  0x04u                                          /* reached warning level for RX errors */
#define CAN_ERR_CRTL_TX_WARNING  0x08u                                          /* reached warning level for TX errors */
#define CAN_ERR_CRTL_RX_PASSIVE  0x10u                                          /* reached error passive status RX */
#define CAN_ERR_CRTL_TX_PASSIVE  0x20u                                          /* reached error passive status TX */
                                      /* (at least one error counter exceeds */
                                      /* the protocol-defined level of 127)  */
#define CAN_ERR_CRTL_ACTIVE      0x40u                                          /* recovered to error active state */

/* error in CAN protocol (type) / data[2] */
#define CAN_ERR_PROT_UNSPEC      0x00u                                          /* unspecified */
#define CAN_ERR_PROT_BIT         0x01u                                          /* single bit error */
#define CAN_ERR_PROT_FORM        0x02u                                          /* frame format error */
#define CAN_ERR_PROT_STUFF       0x04u                                          /* bit stuffing error */
#define CAN_ERR_PROT_BIT0        0x08u                                          /* unable to send dominant bit */
#define CAN_ERR_PROT_BIT1        0x10u                                          /* unable to send recessive bit */
#define CAN_ERR_PROT_OVERLOAD    0x20u                                          /* bus overload */
#define CAN_ERR_PROT_ACTIVE      0x40u                                          /* active error announcement */
#define CAN_ERR_PROT_TX          0x80u                                          /* error occurred on transmission */

/* error in CAN protocol (location) / data[3] */
#define CAN_ERR_PROT_LOC_UNSPEC  0x00u                                          /* unspecified */
#define CAN_ERR_PROT_LOC_SOF     0x03u                                          /* start of frame */
#define CAN_ERR_PROT_LOC_ID28_21 0x02u                                          /* ID bits 28 - 21 (SFF: 10 - 3) */
#define CAN_ERR_PROT_LOC_ID20_18 0x06u                                          /* ID bits 20 - 18 (SFF: 2 - 0 )*/
#define CAN_ERR_PROT_LOC_SRTR    0x04u                                          /* substitute RTR (SFF: RTR) */
#define CAN_ERR_PROT_LOC_IDE     0x05u                                          /* identifier extension */
#define CAN_ERR_PROT_LOC_ID17_13 0x07u                                          /* ID bits 17-13 */
#define CAN_ERR_PROT_LOC_ID12_05 0x0Fu                                          /* ID bits 12-5 */
#define CAN_ERR_PROT_LOC_ID04_00 0x0Eu                                          /* ID bits 4-0 */
#define CAN_ERR_PROT_LOC_RTR     0x0Cu                                          /* RTR */
#define CAN_ERR_PROT_LOC_RES1    0x0Du                                          /* reserved bit 1 */
#define CAN_ERR_PROT_LOC_RES0    0x09u                                          /* reserved bit 0 */
#define CAN_ERR_PROT_LOC_DLC     0x0Bu                                          /* data length code */
#define CAN_ERR_PROT_LOC_DATA    0x0Au                                          /* data section */
#define CAN_ERR_PROT_LOC_CRC_SEQ 0x08u                                          /* CRC sequence */
#define CAN_ERR_PROT_LOC_CRC_DEL 0x18u                                          /* CRC delimiter */
#define CAN_ERR_PROT_LOC_ACK     0x19u                                          /* ACK slot */
#define CAN_ERR_PROT_LOC_ACK_DEL 0x1Bu                                          /* ACK delimiter */
#define CAN_ERR_PROT_LOC_EOF     0x1Au                                          /* end of frame */
#define CAN_ERR_PROT_LOC_INTERM  0x12u                                          /* intermission */

/* error status of CAN-transceiver / data[4] */
/*                                             CANH CANL */
#define CAN_ERR_TRX_UNSPEC             0x00                                     /* 0000 0000 */
#define CAN_ERR_TRX_CANH_NO_WIRE       0x04                                     /* 0000 0100 */
#define CAN_ERR_TRX_CANH_SHORT_TO_BAT  0x05                                     /* 0000 0101 */
#define CAN_ERR_TRX_CANH_SHORT_TO_VCC  0x06                                     /* 0000 0110 */
#define CAN_ERR_TRX_CANH_SHORT_TO_GND  0x07                                     /* 0000 0111 */
#define CAN_ERR_TRX_CANL_NO_WIRE       0x40                                     /* 0100 0000 */
#define CAN_ERR_TRX_CANL_SHORT_TO_BAT  0x50                                     /* 0101 0000 */
#define CAN_ERR_TRX_CANL_SHORT_TO_VCC  0x60                                     /* 0110 0000 */
#define CAN_ERR_TRX_CANL_SHORT_TO_GND  0x70                                     /* 0111 0000 */
#define CAN_ERR_TRX_CANL_SHORT_TO_CANH 0x80                                     /* 1000 0000 */

/*typedef enum {typ2ByteInt = 0u, typ2ByteUInt = 1u, typ3ByteUInt=2u, typ4ByteUInt=3u, typ8ByteUInt=4u, typ1ByteDouble=5u, typ1ByteUDouble=6u, typ2ByteDouble=7u, typ2ByteUDouble=8u, typ8ByteDouble=9u, typ3ByteDouble=10u, typ4ByteDouble=11u, typ4ByteUDouble=12u, typ1ByteUint=13u, NumOfN2KDataTypes=14u } j1939_Data_Type_e; */
typedef enum {typ2ByteInt = 0u, typ2ByteUInt = 1u, typ3ByteUInt=2u, typ4ByteUInt=3u, typ8ByteUInt=4u, typ1ByteDouble=5u, typ1ByteUDouble=6u, typ2ByteDouble=7u, typ2ByteUDouble=8u, typ8ByteDouble=9u, typ3ByteDouble=10u, typ4ByteDouble=11u, typ4ByteUDouble=12u, typ1ByteUint=13u, typ1ByteUDoubleLinakVal=14u, typ1ByteDoubleLinakCur=15u, typ2ByteDoubleLinakVal=16, NumOfj1939DataTypes=17u } j1939_Data_Type_e;

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

  } j1939_Type_u;

#if defined(D_FT900)
typedef struct J1939PACKED {
   j1939_Type_u j1939field;                                                     /* field read from a j1939 message */
} j1939_data_obj_t;
#else
J1939PACKED (
typedef struct {
   j1939_Type_u j1939field;                                                     /* field read from a j1939 message */
}) j1939_data_obj_t;
#endif

#if defined(APEM_JOYSTCK_USED)
//   ====================== APEM HF series JOYSTICK ============================
//
//   per the SAE J1939-71 message protocol.
//   Primary Axis and button data on Basic Joystick Message 1 (BJM1):
#define J1939_BJM1_JOY_PRIO 3u
#define J1939_BJM1_JOY_PGN 0xFDD6u
#define J1939_BJM1_ADDR 0x13u 
// - Data field: 8 bytes
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        uint8_t priX_neut : 2u;                                                 //Primary X-axis neutral position status
        uint8_t priX_left : 2u;                                                 //Primary X-axis left position status
        uint8_t priX_right : 2u;                                                //Primary X-axis right position status
        uint8_t priX_pos1 : 2u;                                                 // Primary X-axis position data first bits of the 10 bit resolution
        uint8_t priX_pos2;                                                      // Primary X-axis position data last 8 bits of the 10 bit resolution
        uint8_t priY_neut : 2u;                                                 //Primary Y-axis neutral position status
        uint8_t priY_left : 2u;                                                 //Primary Y-axis left position status
        uint8_t priY_right : 2u;                                                //Primary Y-axis right position status
        uint8_t priY_pos1 : 2u;                                                 // Primary Y-axis position data first bits of the 10 bit resolution
        uint8_t priY_pos2;                                                      // Primary Y-axis position data last 8 bits of the 10 bit resolution
        uint8_t byte5;                                                          // not used
        uint8_t byte6spare : 4u;
        uint8_t but2Stat : 2u;                                                  // button 2 status
        uint8_t but1Stat : 2u;                                                  // button 1 status
} CAN_j1939_bjm1_t;
#else
J1939PACKED(
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        uint8_t priX_neut : 2u;                                                 //Primary X-axis neutral position status
        uint8_t priX_left : 2u;                                                 //Primary X-axis left position status
        uint8_t priX_right : 2u;                                                //Primary X-axis right position status
        uint8_t priX_pos1 : 2u;                                                 // Primary X-axis position data first bits of the 10 bit resolution
        uint8_t priX_pos2;                                                      // Primary X-axis position data last 8 bits of the 10 bit resolution
        uint8_t priY_neut : 2u;                                                 //Primary Y-axis neutral position status
        uint8_t priY_left : 2u;                                                 //Primary Y-axis left position status
        uint8_t priY_right : 2u;                                                //Primary Y-axis right position status
        uint8_t priY_pos1 : 2u;                                                 // Primary Y-axis position data first bits of the 10 bit resolution
        uint8_t priY_pos2;                                                      // Primary Y-axis position data last 8 bits of the 10 bit resolution
        uint8_t byte5;                                                          // not used
        uint8_t byte6spare : 4u;
        uint8_t but2Stat : 2u;                                                  // button 2 status
        uint8_t but1Stat : 2u;                                                  // button 1 status
}) CAN_j1939_bjm1_t;
#endif

// Redundant Axis data on Extended Joystick Message 1 ( EJMl):
#define J1939_EJM1_JOY_PRIO 3u
#define J1939_EJM1_JOY_PGN 0xFDD7u
#define J1939_EJM1_ADDR 0x13u
// - Data field: 8 bytes
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        uint16_t priX_neut : 2u;                                                //Primary X-axis neutral position status
        uint16_t priX_left : 2u;                                                //Primary X-axis left position status
        uint16_t priX_right : 2u;                                               //Primary X-axis right position status
        uint16_t priX_pos : 10u;                                                // Primary X-axis position data bits of the 10 bit resolution
        uint16_t priY_neut : 2u;                                                //Primary Y-axis neutral position status
        uint16_t priY_left : 2u;                                                //Primary Y-axis left position status
        uint16_t priY_right : 2u;                                               //Primary Y-axis right position status
        uint16_t priY_pos : 10u;                                                // Primary Y-axis position data bits of the 10 bit resolution
        uint16_t priZ_neut : 2u;                                                //Primary Z-axis neutral position status
        uint16_t priZ_left : 2u;                                                //Primary Z-axis left position status
        uint16_t priZ_right : 2u;                                               //Primary Z-axis right position status
        uint16_t priZ_pos : 10u;                                                // Primary Z-axis position data bits of the 10 bit resolution
} CAN_j1939_ejm1_t;
#else
J1939PACKED(
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        uint16_t priX_neut : 2u;                                                //Primary X-axis neutral position status
        uint16_t priX_left : 2u;                                                //Primary X-axis left position status
        uint16_t priX_right : 2u;                                               //Primary X-axis right position status
        uint16_t priX_pos : 10u;                                                // Primary X-axis position data bits of the 10 bit resolution
        uint16_t priY_neut : 2u;                                                //Primary Y-axis neutral position status
        uint16_t priY_left : 2u;                                                //Primary Y-axis left position status
        uint16_t priY_right : 2u;                                               //Primary Y-axis right position status
        uint16_t priY_pos : 10u;                                                // Primary Y-axis position data bits of the 10 bit resolution
        uint16_t priZ_neut : 2u;                                                //Primary Z-axis neutral position status
        uint16_t priZ_left : 2u;                                                //Primary Z-axis left position status
        uint16_t priZ_right : 2u;                                               //Primary Z-axis right position status
        uint16_t priZ_pos : 10u;                                                // Primary Z-axis position data bits of the 10 bit resolution
}) CAN_j1939_ejm1_t;
#endif

#endif /* end apem joystick */

// ==== SOURCE ADDRESS ======
//      ORANGEWIRE BLUEWIRE
// 0x13 OPEN       OPEN    (default)
// 0x23 OPEN       Grounded
// 0x33 Grounded   OPEN
// 0x43 Grounded    Grounded

// • Baud rate: 125 Kbps, 500Kbps, 1Mbps  (250 Kbps default)

// Engine Total Fuel used LFC :
#define J1939_LFC_PRIO 3u
#define J1939_LFC_PGN 65257u
#define J1939_LFC_ADDR 0x21u
// - Data field: 8 bytes
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        float64_t engTotFuel;                                                   /* precision is 0.5 read example j1939GetBuf(Index, j1939RcvBuf, typ4ByteUDouble, 0.5f, 0.0f, 0.0f, &lfcStrut.engTotFuel); */
} CAN_j1939_LFC_t;
#else
J1939PACKED(
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        float64_t engTotFuel;                                                   /* precision is 0.5 read example j1939GetBuf(Index, j1939RcvBuf, typ4ByteUDouble, 0.5f, 0.0f, 0.0f, &lfcStrut.engTotFuel); */
}) CAN_j1939_LFC_t;
#endif
// Engine Fuel Level used DD :
#define J1939_DD_PRIO 3u
#define J1939_DD_PGN 65276u
#define J1939_DD_ADDR 0x21u
// - Data field: 8 bytes
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        float64_t engFuelLvl;                                                   /* precision is 0.40000000000000002 read example j1939GetBuf(Index, j1939RcvBuf, typ4ByteUDouble, 0.40000000000000002f, 0.0f, 0.0f, &lfcStrut.engFuelLvl); */
} CAN_j1939_DD_t;
#else
J1939PACKED(
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        float64_t engFuelLvl;                                                   /* precision is 0.40000000000000002 read example j1939GetBuf(Index, j1939RcvBuf, typ4ByteUDouble, 0.40000000000000002f, 0.0f, 0.0f, &lfcStrut.engFuelLvl); */
}) CAN_j1939_DD_t;
#endif

// Transmission Oil Temperature 1 used TRF1 :
#define J1939_TRF1_PRIO 6u
#define J1939_TRF1_PGN 65272u
#define J1939_TRF1_ADDR 0x21u
// - Data field: 8 bytes
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        uint8_t clutchPressure;                                                 /* receiving data 8 bytes */
        uint8_t transOilLvl;
        uint8_t transOilDP;
        uint8_t transOilPress;
        uint16_t TransOilTemp;
        uint8_t transOilLvlHiLo;
        uint8_t transOilLvlCnt : 4u;
        uint8_t transOilLvlMeas : 4u;                                           /* end recieved data */
        float64_t calcTransOilTemp;                                             /* precision is 0.03125 read example j1939GetBuf(Index, j1939RcvBuf+4u, typ2ByteUDouble, 0.03125f, -273.0f, 0.0f, &lfcStrut.TransOilTemp); */
} CAN_j1939_TRF1_t;
#else
J1939PACKED(
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        uint8_t clutchPressure;                                                 /* receiving data 8 bytes */
        uint8_t transOilLvl;
        uint8_t transOilDP;
        uint8_t transOilPress;
        uint16_t TransOilTemp;
        uint8_t transOilLvlHiLo;
        uint8_t transOilLvlCnt : 4u;
        uint8_t transOilLvlMeas : 4u;                                           /* end recieved data */
        float64_t calcTransOilTemp;                                             /* precision is 0.03125 read example j1939GetBuf(Index, j1939RcvBuf+4u, typ2ByteUDouble, 0.03125f, -273.0f, 0.0f, &lfcStrut.TransOilTemp); */
}) CAN_j1939_TRF1_t;
#endif

// Engine Temperature 1  :
#define J1939_ENG_TEMP_PRIO 6u
#define J1939_ENG_TEMP_PGN 0x00FEEEu
// - Data field: 8 bytes
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        uint8_t engCoolTemp;                                                    /* receiving data 8 bytes Engine Coolant Temperature */
        uint8_t engFuelTemp1;                                                   /* Engine Fuel Temperature 1 */
        uint16_t engOilTemp1;                                                   /* Engine Temperature */
        uint16_t engTurboCharOilTemp;                                           /* Engine Turbocharger Oil Temperature */
        uint8_t engInterTemp;                                                   /* Engine Intercooler Temperature */
        uint8_t engInterThermoOpen;                                             /* end recieved data Engine Intercooler Thermostat Opening */
        float64_t CalcengCoolTemp;                                              /* receiving data 8 bytes Engine Coolant Temperature */
        float64_t CalcengFuelTemp1;                                             /* Engine Fuel Temperature 1 */
        float64_t CalcengOilTemp1;                                              /* Engine Temperature */
        float64_t CalcengTurboCharOilTemp;                                      /* Engine Turbocharger Oil Temperature */
        float64_t CalcengInterTemp;                                             /* Engine Intercooler Temperature */
        float64_t CalcengInterThermoOpen;                                       /* end recieved data Engine Intercooler Thermostat Opening */
} CAN_j1939_EngTemp_t;
#else
J1939PACKED(
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        uint8_t engCoolTemp;                                                    /* receiving data 8 bytes Engine Coolant Temperature */
        uint8_t engFuelTemp1;                                                   /* Engine Fuel Temperature 1 */
        uint16_t engOilTemp1;                                                   /* Engine Temperature */
        uint16_t engTurboCharOilTemp;                                           /* Engine Turbocharger Oil Temperature */
        uint8_t engInterTemp;                                                   /* Engine Intercooler Temperature */
        uint8_t engInterThermoOpen;                                             /* end recieved data Engine Intercooler Thermostat Opening */
        float64_t CalcengCoolTemp;                                              /* receiving data 8 bytes Engine Coolant Temperature */
        float64_t CalcengFuelTemp1;                                             /* Engine Fuel Temperature 1 */
        float64_t CalcengOilTemp1;                                              /* Engine Temperature */
        float64_t CalcengTurboCharOilTemp;                                      /* Engine Turbocharger Oil Temperature */
        float64_t CalcengInterTemp;                                             /* Engine Intercooler Temperature */
        float64_t CalcengInterThermoOpen;                                       /* end recieved data Engine Intercooler Thermostat Opening */
}) CAN_j1939_EngTemp_t;
#endif

// Engine Speed :
#define J1939_EngSpeed_PRIO 3u
#define J1939_EngSpeed_PGN 61444u

// - Data field: 8 bytes
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        uint8_t torqueMode : 4u;                                                /* receiving data 8 bytes */
        uint8_t spareTor : 4u;
        uint8_t driverDemandTorque;
        uint8_t actualTorque;
        uint16_t engineSpeed;
        uint8_t ecuAddr;
        uint8_t engStarterMode : 4u;
        uint8_t spareStMode : 4u;
        uint8_t engDemTorque;                                                   /* end recieved data */
        float64_t calcEngineSpeed;                                                  /* precision is 0.125 rpm/bit, 0 offset */
} CAN_j1939_EngSpeed_t;
#else
J1939PACKED(
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        uint8_t torqueMode : 4u;                                                /* receiving data 8 bytes */
        uint8_t spareTor : 4u;
        uint8_t driverDemandTorque;
        uint8_t actualTorque;
        uint16_t engineSpeed;
        uint8_t ecuAddr;
        uint8_t engStarterMode : 4u;
        uint8_t spareStMode : 4u;
        uint8_t engDemTorque;                                                   /* end recieved data */
        float64_t calcEngineSpeed;                                                  /* precision is 0.125 rpm/bit, 0 offset */
}) CAN_j1939_EngSpeed_t;
#endif

// Selected Gear used ETC2 :
#define J1939_ETC2_PRIO 3u
#define J1939_ETC2_PGN 61445u
#define J1939_ETC2_ADDR 0x21u
// - Data field: 8 bytes
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        float64_t SelectedGear;                                                 /* precision is 1 read example j1939GetBuf(Index, j1939RcvBuf, typ1ByteUDouble, 1.0f, -125.0f, 0.0f, &lfcStrut.SelectedGear); */
        float64_t CurrentGear;                                                  /* precision is 1 read example j1939GetBuf(Index, j1939RcvBuf+3u, typ1ByteUDouble, 1.0f, -125.0f, 0.0f, &lfcStrut.CurrentGear); */
} CAN_j1939_ETC2_t;
#else
J1939PACKED(
typedef struct {
        uint32_t PGN;
        uint8_t priority;
        uint8_t SA;
        float64_t SelectedGear;                                                 /* precision is 1 read example j1939GetBuf(Index, j1939RcvBuf, typ1ByteUDouble, 1.0f, -125.0f, 0.0f, &lfcStrut.SelectedGear); */
        float64_t CurrentGear;                                                  /* precision is 1 read example j1939GetBuf(Index, j1939RcvBuf+3u, typ1ByteUDouble, 1.0f, -125.0f, 0.0f, &lfcStrut.CurrentGear); */
}) CAN_j1939_ETC2_t;
#endif

/* convert this JSON for the full protocol when you have time
[
        {
                "name" : "VI",
                "pgn" : 65260,
                "length" : 0,
                "spns" :
                [
                        {
                                "name" : "Vehicle Number Identifier",
                                "number" : 237,
                                "type" : 2
                        }
                ]
        },
        {
                "name" : "DI",
                "pgn" : 65131,
                "length" : 0,
                "spns" :
                [
                        {
                                "name" : "Driver 1 Identification",
                                "number" : 1625,
                                "type" : 2
                        },
                        {
                                "name" : "Driver 2 Identification",
                                "number" : 1626,
                                "type" : 2
                        }
                ]
        },
        {
                "name" : "DD",
                "pgn" : 65276,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 1,
                                "formatGain" : 0.40000000000000002,
                                "formatOffset" : 0,
                                "name" : "Fuel Level",
                                "number" : 96,
                                "offset" : 1,
                                "type" : 0,
                                "units" : "%"
                        }
                ]
        },
        {
                "name" : "ETC1",
                "pgn" : 61442,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 1,
                                "formatGain" : 0.4,
                                "formatOffset" : 0,
                                "name" : "Percent Clutch Slip",
                                "number" : 522,
                                "offset" : 3,
                                "type" : 0,
                                "units" : "%"
                        }
                ]
        },
        {
                "name" : "ETC2",
                "pgn" : 61445,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 1,
                                "formatGain" : 1,
                                "formatOffset" : -125,
                                "name" : "Transmission Selected Gear",
                                "number" : 524,
                                "offset" : 0,
                                "type" : 0,
                                "units" : ""
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 1,
                                "formatOffset" : -125,
                                "name" : "Transmission Current Gear",
                                "number" : 523,
                                "offset" : 3,
                                "type" : 0,
                                "units" : ""
                        }
                ]
        },
        {
                "name" : "HOURS",
                "pgn" : 65253,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 4,
                                "formatGain" : 0.05,
                                "formatOffset" : 0,
                                "name" : "Engine Total Hours of Operation",
                                "number" : 247,
                                "offset" : 0,
                                "type" : 0,
                                "units" : "hours"
                        }
                ]
        },
        {
                "name" : "TRF1",
                "pgn" : 65272,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 2,
                                "formatGain" : 0.03125,
                                "formatOffset" : -273,
                                "name" : "Transmission Oil Temperature 1",
                                "number" : 177,
                                "offset" : 4,
                                "type" : 0,
                                "units" : "°C"
                        }
                ]
        },
        {
                "name" : "ERC1",
                "pgn" : 61440,
                "length" : 8,
                "spns" :
                [
                        {
                                "bitOffset" : 4,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Retarder - brake assist disabled",
                                        "Retarder - brake assist enabled",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Retarder Enable - Brake Assist Switch",
                                "number" : 571,
                                "offset" : 0,
                                "type" : 1
                        }
                ]
        },
        {
                "name" : "EBC1",
                "pgn" : 61441,
                "length" : 8,
                "spns" :
                [
                        {
                                "bitOffset" : 0,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Not Fully Operational",
                                        "Fully Operational",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "ABS Fully Operational",
                                "number" : 1243,
                                "offset" : 5,
                                "type" : 1
                        }
                ]
        },
        {
                "name" : "IC1",
                "pgn" : 65270,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 1,
                                "formatGain" : 2,
                                "formatOffset" : 0,
                                "name" : "Engine Intake Manifold #1 Pressure",
                                "number" : 102,
                                "offset" : 1,
                                "type" : 0,
                                "units" : "kPa"
                        }
                ]
        },
        {
                "name" : "RF",
                "pgn" : 65275,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 1,
                                "formatGain" : 16,
                                "formatOffset" : 0,
                                "name" : "Hydraulic Retarder Pressure",
                                "number" : 119,
                                "offset" : 0,
                                "type" : 0,
                                "units" : "kPa"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 1,
                                "formatOffset" : -40,
                                "name" : "Hydraulic Retarder Oil Temperature",
                                "number" : 120,
                                "offset" : 1,
                                "type" : 0,
                                "units" : "°C"
                        }
                ]
        },
        {
                "name" : "VDHR",
                "pgn" : 65217,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 4,
                                "formatGain" : 5,
                                "formatOffset" : 0,
                                "name" : "High resolution total vehicle distance",
                                "number" : 917,
                                "offset" : 0,
                                "type" : 0,
                                "units" : "m"
                        }
                ]
        },
        {
                "name" : "AT1T1I",
                "pgn" : 65110,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 1,
                                "formatGain" : 0.4,
                                "formatOffset" : 0,
                                "name" : "Aftertreatment 1 Diesel Exhaust Fluid Tank 1 Level",
                                "number" : 1761,
                                "offset" : 0,
                                "type" : 0,
                                "units" : "%"
                        }
                ]
        },
        {
                "name" : "EFL",
                "pgn" : 65263,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 1,
                                "formatGain" : 4,
                                "formatOffset" : 0,
                                "name" : "Engine Fuel Delivery Pressure",
                                "number" : 94,
                                "offset" : 0,
                                "type" : 0,
                                "units" : "KPa"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 0.05,
                                "formatOffset" : 0,
                                "name" : "Engine Extended Crankcase Blow-by Pressure",
                                "number" : 22,
                                "offset" : 1,
                                "type" : 0,
                                "units" : "KPa"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 0.4,
                                "formatOffset" : 0,
                                "name" : "Engine Oil Level",
                                "number" : 98,
                                "offset" : 2,
                                "type" : 0,
                                "units" : "%"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 4,
                                "formatOffset" : 0,
                                "name" : "Engine Oil Pressure",
                                "number" : 100,
                                "offset" : 3,
                                "type" : 0,
                                "units" : "KPa"
                        },
                        {
                                "byteSize" : 2,
                                "formatGain" : 0.0078125,
                                "formatOffset" : -250,
                                "name" : "Engine Crankcase Pressure",
                                "number" : 101,
                                "offset" : 4,
                                "type" : 0,
                                "units" : "KPa"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 2,
                                "formatOffset" : 0,
                                "name" : "Engine Coolant Pressure",
                                "number" : 109,
                                "offset" : 6,
                                "type" : 0,
                                "units" : "KPa"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 0.4,
                                "formatOffset" : 0,
                                "name" : "Engine Coolant Level",
                                "number" : 111,
                                "offset" : 7,
                                "type" : 0,
                                "units" : "%"
                        }
                ]
        },
        {
                "name" : "LFC",
                "pgn" : 65257,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 4,
                                "formatGain" : 0.5,
                                "formatOffset" : 0,
                                "name" : "Engine total fuel used",
                                "number" : 250,
                                "offset" : 4,
                                "type" : 0,
                                "units" : "L"
                        }
                ]
        },
        {
                "name" : "ET1",
                "pgn" : 65262,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 1,
                                "formatGain" : 1,
                                "formatOffset" : -40,
                                "name" : "Engine Coolant Temperature",
                                "number" : 110,
                                "offset" : 0,
                                "type" : 0,
                                "units" : "°C"
                        },
                        {
                                "byteSize" : 2,
                                "formatGain" : 0.03125,
                                "formatOffset" : -273,
                                "name" : "Engine Oil Temperature 1",
                                "number" : 175,
                                "offset" : 2,
                                "type" : 0,
                                "units" : "°C"
                        }
                ]
        },
        {
                "name" : "TRF1",
                "pgn" : 65272,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 1,
                                "formatGain" : 16,
                                "formatOffset" : 0,
                                "name" : "Clutch Pressure",
                                "number" : 123,
                                "offset" : 0,
                                "type" : 0,
                                "units" : "kPa"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 0.4,
                                "formatOffset" : 0,
                                "name" : "Transmission Oil Level",
                                "number" : 124,
                                "offset" : 1,
                                "type" : 0,
                                "units" : "%"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 2,
                                "formatOffset" : 0,
                                "name" : "Transmission Filter Differential Pressure",
                                "number" : 126,
                                "offset" : 2,
                                "type" : 0,
                                "units" : "kPa"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 16,
                                "formatOffset" : 0,
                                "name" : "Transmission Oil Pressure",
                                "number" : 127,
                                "offset" : 3,
                                "type" : 0,
                                "units" : "kPa"
                        },
                        {
                                "byteSize" : 2,
                                "formatGain" : 0.03125,
                                "formatOffset" : -273,
                                "name" : "Transmission Oil Temperature",
                                "number" : 177,
                                "offset" : 4,
                                "type" : 0,
                                "units" : "kPa"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 0.5,
                                "formatOffset" : -62.5,
                                "name" : "Transmission Oil Level High / Low",
                                "number" : 3027,
                                "offset" : 6,
                                "type" : 0,
                                "units" : "L"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 0.5,
                                "formatOffset" : -62.5,
                                "name" : "Transmission Oil Level Countdown Timer",
                                "number" : 3027,
                                "offset" : 6,
                                "type" : 0,
                                "units" : "L"
                        },
                        {
                                "bitOffset" : 0,
                                "bitSize" : 4,
                                "name" : "Transmission Oil Level Countdown Timer",
                                "number" : 3028,
                                "offset" : 7,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 4,
                                "bitSize" : 4,
                                "name" : "Transmission Oil Level Measurement Status",
                                "number" : 3026,
                                "offset" : 7,
                                "type" : 1
                        }
                ]
        },
        {
                "name" : "LFE",
                "pgn" : 65266,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 2,
                                "formatGain" : 0.05,
                                "formatOffset" : 0,
                                "name" : "Fuel Rate",
                                "number" : 183,
                                "offset" : 0,
                                "type" : 0,
                                "units" : "L/h"
                        }
                ]
        },
        {
                "name" : "HRLFC",
                "pgn" : 64777,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 4,
                                "formatGain" : 0.001,
                                "formatOffset" : 0,
                                "name" : "High resolution engine total fuel used",
                                "number" : 5054,
                                "offset" : 4,
                                "type" : 0,
                                "units" : "L"
                        }
                ]
        },
        {
                "name" : "EEC2",
                "pgn" : 61443,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 1,
                                "formatGain" : 0.4,
                                "formatOffset" : 0,
                                "name" : "Accelerator pedal position 1",
                                "number" : 91,
                                "offset" : 1,
                                "type" : 0,
                                "units" : "%"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 1,
                                "formatOffset" : 0,
                                "name" : "Engine Percent Load At Current Speed",
                                "number" : 92,
                                "offset" : 2,
                                "type" : 0,
                                "units" : "%"
                        },
                        {
                                "bitOffset" : 0,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Accelerator pedal 1 not in low idle condition",
                                        "Accelerator pedal 1 in low idle condition",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Accelerator Pedal 1 Low Idle Switch",
                                "number" : 558,
                                "offset" : 0,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 6,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Accelerator pedal 2 not in low idle condition",
                                        "Accelerator pedal 2 in low idle condition",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Accelerator Pedal 2 Low Idle Switch",
                                "number" : 2970,
                                "offset" : 0,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 2,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Kickdown passive",
                                        "Kickdown active",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Accelerator Pedal Kickdown Switch",
                                "number" : 559,
                                "offset" : 0,
                                "type" : 1
                        }
                ]
        },
        {
                "name" : "TCO1",
                "pgn" : 65132,
                "length" : 8,
                "spns" :
                [
                        {
                                "bitOffset" : 0,
                                "bitSize" : 3,
                                "descriptions" :
                                [
                                        "Rest",
                                        "Driver Available",
                                        "Work",
                                        "Drive"
                                ],
                                "name" : "Driver 1 Working State",
                                "number" : 1612,
                                "offset" : 0,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 3,
                                "bitSize" : 3,
                                "descriptions" :
                                [
                                        "Rest",
                                        "Driver Available",
                                        "Work",
                                        "Drive"
                                ],
                                "name" : "Driver 2 Working State",
                                "number" : 1613,
                                "offset" : 0,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 4,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Card not present",
                                        "Card present",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Driver 1 Card",
                                "number" : 1615,
                                "offset" : 1,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 4,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Card not present",
                                        "Card present",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Driver 2 Card",
                                "number" : 1616,
                                "offset" : 2,
                                "type" : 1
                        }
                ]
        },
        {
                "name" : "DC1",
                "pgn" : 65102,
                "length" : 8,
                "spns" :
                [
                        {
                                "bitOffset" : 6,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "all bus doors disabled",
                                        "at least 1 bus door enabled",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Status 2 of doors",
                                "number" : 3411,
                                "offset" : 0,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 4,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "inside bus",
                                        "outside bus",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Ramp/Wheel chairlift",
                                "number" : 1820,
                                "offset" : 0,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 0,
                                "bitSize" : 4,
                                "descriptions" :
                                [
                                        "at least 1 door is open",
                                        "closing last door",
                                        "all doors closed"
                                ],
                                "name" : "Position of doors",
                                "number" : 1821,
                                "offset" : 0,
                                "type" : 1
                        }
                ]
        },
        {
                "name" : "AS",
                "pgn" : 65237,
                "length" : 8,
                "spns" :
                [
                        {
                                "bitOffset" : 0,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Alternator 1 not charging",
                                        "Alternator 1 charging",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Alternator 1 Status",
                                "number" : 3353,
                                "offset" : 2,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 2,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Alternator 2 not charging",
                                        "Alternator 2 charging",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Alternator 2 Status",
                                "number" : 3354,
                                "offset" : 2,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 4,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Alternator 3 not charging",
                                        "Alternator 3 charging",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Alternator 3 Status",
                                "number" : 3355,
                                "offset" : 2,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 6,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Alternator 4 not charging",
                                        "Alternator 4 charging",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Alternator 4 Status",
                                "number" : 3356,
                                "offset" : 2,
                                "type" : 1
                        }
                ]
        },
        {
                "name" : "CCVS",
                "pgn" : 65265,
                "length" : 8,
                "spns" :
                [
                        {
                                "bitOffset" : 0,
                                "bitSize" : 5,
                                "descriptions" :
                                [
                                        "Off",
                                        "",
                                        "",
                                        "",
                                        "",
                                        "Set"
                                ],
                                "name" : "PTO State",
                                "number" : 976,
                                "offset" : 6,
                                "type" : 1
                        },
                        {
                                "byteSize" : 2,
                                "formatGain" : 0.00390625,
                                "formatOffset" : 0,
                                "name" : "Wheel Speed",
                                "number" : 84,
                                "offset" : 1,
                                "type" : 0,
                                "units" : "km/h"
                        },
                        {
                                "bitOffset" : 4,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Pedal released",
                                        "Pedal depressed",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Brake Switch",
                                "number" : 597,
                                "offset" : 3,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 6,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Pedal released",
                                        "Pedal depressed",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Clutch Switch",
                                "number" : 598,
                                "offset" : 3,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 2,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Parking brake not set",
                                        "Parking brake set",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Parking Brake Switch",
                                "number" : 70,
                                "offset" : 0,
                                "type" : 1
                        }
                ]
        },
        {
                "name" : "PTODE",
                "pgn" : 64932,
                "length" : 8,
                "spns" :
                [
                        {
                                "bitOffset" : 0,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "No PTO drive is engaged",
                                        "At least one PTO drive is engaged",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "At least one PTO engaged",
                                "number" : 3948,
                                "offset" : 6,
                                "type" : 1
                        }
                ]
        },
        {
                "name" : "EEC1",
                "pgn" : 61444,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 2,
                                "formatGain" : 0.125,
                                "formatOffset" : 0,
                                "name" : "Engine Speed",
                                "number" : 190,
                                "offset" : 3,
                                "type" : 0,
                                "units" : "rpm"
                        },
                        {
                                "byteSize" : 1,
                                "formatGain" : 1.0,
                                "formatOffset" : -125,
                                "name" : "Percent Torque",
                                "number" : 513,
                                "offset" : 2,
                                "type" : 0,
                                "units" : "%"
                        }
                ]
        },
        {
                "name" : "AMB",
                "pgn" : 65269,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 1,
                                "formatGain" : 0.5,
                                "formatOffset" : 0,
                                "name" : "Barometric Pressure",
                                "number" : 108,
                                "offset" : 0,
                                "type" : 0,
                                "units" : "kPa"
                        }
                ]
        },
        {
                "name" : "SERV",
                "pgn" : 65216,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 2,
                                "formatGain" : 5,
                                "formatOffset" : -160635,
                                "name" : "Service distance",
                                "number" : 914,
                                "offset" : 1,
                                "type" : 0,
                                "units" : "km"
                        }
                ]
        },
        {
                "name" : "DC2",
                "pgn" : 64933,
                "length" : 8,
                "spns" :
                [
                        {
                                "bitOffset" : 2,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Closed",
                                        "Open",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Open Status Door 1",
                                "number" : 3413,
                                "offset" : 0,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 0,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Closed",
                                        "Open",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Open Status Door 2",
                                "number" : 3416,
                                "offset" : 1,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 6,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Closed",
                                        "Open",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Open Status Door 3",
                                "number" : 3419,
                                "offset" : 1,
                                "type" : 1
                        },
                        {
                                "bitOffset" : 4,
                                "bitSize" : 2,
                                "descriptions" :
                                [
                                        "Closed",
                                        "Open",
                                        "Error",
                                        "Not available"
                                ],
                                "name" : "Open Status Door 3",
                                "number" : 3422,
                                "offset" : 2,
                                "type" : 1
                        }
                ]
        },
        {
                "name" : "VDC2",
                "pgn" : 61449,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 2,
                                "formatGain" : 0.00048828125,
                                "formatOffset" : -15.687,
                                "name" : "Lateral Acceleration",
                                "number" : 1809,
                                "offset" : 5,
                                "type" : 0,
                                "units" : "m/s2"
                        }
                ]
        },
        {
                "name" : "EBC2",
                "pgn" : 65215,
                "length" : 8,
                "spns" :
                [
                        {
                                "byteSize" : 2,
                                "formatGain" : 0.00390625,
                                "formatOffset" : 0,
                                "name" : "Front Axe Speed",
                                "number" : 904,
                                "offset" : 0,
                                "type" : 0,
                                "units" : "kph"
                        },{
                                "byteSize" : 1,
                                "formatGain" : 0.0625,
                                "formatOffset" : -7.18125,
                                "name" : "Relative Speed; Front Axle, Left Wheel",
                                "number" : 905,
                                "offset" : 2,
                                "type" : 0,
                                "units" : "kph"
                        },{
                                "byteSize" : 1,
                                "formatGain" : 0.0625,
                                "formatOffset" : -7.18125,
                                "name" : "Relative Speed; Front Axle, Right Wheel",
                                "number" : 906,
                                "offset" : 3,
                                "type" : 0,
                                "units" : "kph"
                        },{
                                "byteSize" : 1,
                                "formatGain" : 0.0625,
                                "formatOffset" : -7.18125,
                                "name" : "Relative Speed; Rear Axle 1, Left Wheel",
                                "number" : 907,
                                "offset" : 4,
                                "type" : 0,
                                "units" : "kph"
                        },{
                                "byteSize" : 1,
                                "formatGain" : 0.0625,
                                "formatOffset" : -7.18125,
                                "name" : "Relative Speed; Rear Axle 1, Right Wheel",
                                "number" : 908,
                                "offset" : 5,
                                "type" : 0,
                                "units" : "kph"
                        },{
                                "byteSize" : 1,
                                "formatGain" : 0.0625,
                                "formatOffset" : -7.18125,
                                "name" : "Relative Speed; Rear Axle 2, Left Wheel",
                                "number" : 909,
                                "offset" : 6,
                                "type" : 0,
                                "units" : "kph"
                        },{
                                "byteSize" : 1,
                                "formatGain" : 0.0625,
                                "formatOffset" : -7.18125,
                                "name" : "Relative Speed; Rear Axle 2, Right Wheel",
                                "number" : 910,
                                "offset" : 7,
                                "type" : 0,
                                "units" : "kph"
                        }
                ]
        }
]
*/

/* ======================= LINAK ACTUATOR ================================== */
#ifndef __J1939_Linak_
#define __J1939_Linak_

#if defined(LINAK_ACTUATOR_USED)

#define LINAK_POS_MAX 64255LU
#define LINAK_POS_MAX_F 200.0f
#define LINAK_CLR_ERROR_CODE 64256LU                                            //Clear ErrorCode register
#define LINAK_RUN_2_OUT 64257LU                                                 //Command run to actuator out
#define LINAK_RUN_2_IN 64258LU                                                  //Command run to actuator in
#define LINAK_STOP_ACTUATOR 64259LU                                             // Command stop actuator*
#define LINAK_SPEED_MAX 200                                                     // max actuator speed
#define LINAK_SOFT_MAX 200                                                      // max sofstart rate
#define LINAK_CURRENT_MAX 200                                                   // max current

#define LINAK_POSN_LOST 65024u                                                  // lost position error
#define LINAK_CURR_MEAS_ERR 254u                                                // current measurement error

#define LINNAK_NOT_POSN_TIMOT (CORE_TICK_SECOND * 5u)                           // didnt get to the position in this many ticks
#define LINNAK_NOT_RESP_TIMOT (CORE_TICK_SECOND >> 2u)                          // didnt get a reply in this many ticks

#define LINAK_MAX_SIZE 8U

/* State Engine */
#define LIN_NEW_FWD_TO_SEND 1u
#define LIN_NEW_BWD_TO_SEND 2u
#define LIN_READ_FIELD_STATUS 3u
#define LIN_SEND_STOP 4u
#define LIN_RUN_TO_IN 5u
#define LIN_RUN_TO_OUT 6u
#define LIN_ACTUATOR_ERROR 7u
#define LIN_NEW_PARAMS 8u

/* Actuator CanBus Addresses */
#define LINAK_ACT_NO1 1u

typedef enum {LinakNoErr = 0u, LinakHallErr = 1u, LinakOverVolt=2u, LinakUnderVolt=3u, LinakNoKeepAlive=4u, LinakESSErr=5u, LinakPowerBlkStat=6u, LinakTempErr=7u, LinakHeartBtErr=8u, LinakSMPSErr=9u,  LinakNumOfErrCodes=10u } j1939_LinakError_Type_e;

/* define the physcial system of items conected on your J1939 bus */
typedef enum {actNo1LeftHand, actNo2LeftHand, actNo1RightHand, actNo2RightHand, NumOfJ1939Devices } j1939_Device_Array_Order_e;
typedef enum {actNo1LftHndAddr=0x01, actNo2LftHndAddr=0x0C, actNo1RghtHndAddr=0x1E, actNo2RghtHndAddr=0x76 } j1939_Device_Address_e;

// ================ LINAK ACTUATOR Proprietary A =====================================================================================
//  Function: General request Description Write to proprietary A to clear error state, run out, run in or run to a specific position in
//  addition to setting speed and current limit. Min. transmission rate  250ms PGN 0x00EF0
// Input:
// position - Position [mm*0.1] MSB
// current - Current  [mA*250]
// speed - Speed  [%*0.5]
// softStart Soft Start  [ms*50]
// softStop Soft Stop  [ms*50]
// FFByte6 - Reserved, write 0xFF
// FFByte7 - Reserved, write 0xFF
//
// Output:
//  - J1939Msg                J1939 message ready to be sent.
//*****************************************************************************
#define LINEX_GEN_REC_PGN 0x00EF00UL                                            /* Linex Actuator General Request */
#if defined(D_FT900)
typedef struct J1939PACKED {
        float64_t position;
        float64_t current;
        float64_t speed;
        float64_t softStart;
        float64_t softStop;
        uint8_t FFByte6;
        uint8_t FFByte7;
} linak_general_t;
#else
J1939PACKED (
typedef struct {
        float64_t position;
        float64_t current;
        float64_t speed;
        float64_t softStart;
        float64_t softStop;
        uint8_t FFByte6;
        uint8_t FFByte7;
}) linak_general_t;
#endif

// ================ LINAK ACTUATOR Proprietary B =====================================================================================
//  Function: General status Description Read status parameters, motor current and actuator piston position.
// Input:
// position - Position [mm*0.1] MSB
// current - Current  [mA*250]
// status-
// error
// speed LSB
// speed - Speed [mm*0,1/s] MSB
// External InputState
//
// Output:
//  - J1939Msg                J1939 message ready to be sent.
//*****************************************************************************
#define LINAK_GEN_STAT_PGN 0x00FF00L                                      // Linex Actuator Status Request
#if defined(D_FT900)
typedef struct J1939PACKED {
        float64_t position;
        float64_t current;
        uint8_t ESSinb1 : 1u;
        uint8_t ESSoutb2 : 1u;
        uint8_t Overcurrentb3 : 1u;
        uint8_t Runningoutb4 : 1u;
        uint8_t Runninginb5 : 1u;
        uint8_t Reservedb678 : 3u;
        uint8_t error;
        float64_t speed;
        uint8_t Byte6FF;
        uint8_t inputLevel1 : 2u;
        uint8_t inputLevel2 : 2u;
        uint8_t inputLevel3 : 2u;
        uint8_t errorInCurrentMeas : 1u;
        uint8_t positionLost : 1u;
} linak_status_t;
#else
J1939PACKED (
typedef struct {
        float64_t position;
        float64_t current;
        uint8_t ESSinb1 : 1u;
        uint8_t ESSoutb2 : 1u;
        uint8_t Overcurrentb3 : 1u;
        uint8_t Runningoutb4 : 1u;
        uint8_t Runninginb5 : 1u;
        uint8_t Reservedb678 : 3u;
        uint8_t error;
        float64_t speed;
        uint8_t Byte6FF;
        uint8_t inputLevel1 : 2u;
        uint8_t inputLevel2 : 2u;
        uint8_t inputLevel3 : 2u;
        uint8_t errorInCurrentMeas : 1u;
        uint8_t positionLost : 1u;
}) linak_status_t;
#endif

#if defined(D_FT900)                                                            /* This structure is needed if you use multi frame receive - probably not needed here but not sure */
typedef struct J1939PACKED {
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus j1939 read */
  unsigned char DataBuf[LINAK_MAX_SIZE];                                   /* the j1939 raw byte buffer received */
  linak_status_t LinakStatObj;                                               /* the actual formatted data last received */
  uint32_t timeRef;
  uint8_t State;
  uint8_t StatusWord;
} linak_rcv_t;
#else
J1939PACKED (
typedef struct
{
  uint8_t offset;                                                               /* how many bytes we currently have recieved from CanBus j1939 read */
  unsigned char DataBuf[LINAK_MAX_SIZE];                                   /* the j1939 raw byte buffer received */
  linak_status_t LinakStatObj;                                               /* the actual formatted data last received */
  uint32_t timeRef;
  uint8_t State;
  uint8_t StatusWord;
}) linak_rcv_t;
#endif

#if defined(D_FT900)                                                            /* This structure is needed if you use multi frame receive - probably not needed here but not sure */
typedef struct J1939PACKED {
  linak_general_t LinakCtrlObj;                                                 /* control object */
  uint8_t State;
  uint8_t prevState;
  unsigned char DataBuf[LINAK_MAX_SIZE];                                        /* the j1939 raw byte buffer to send */
} linak_snd_t;
#else
J1939PACKED (
typedef struct
{
  linak_general_t LinakCtrlObj;                                                 /* control object */
  uint8_t State;
  uint8_t prevState;
  unsigned char DataBuf[LINAK_MAX_SIZE];                                        /* the j1939 raw byte buffer received */
}) linak_snd_t;
#endif


#endif   /* ----- end Linak Actuator -------------------- */

#endif /* end linak include definition */

// ================ J1939 General PGN's ========================================
//  Function: Send an ACK or NACK back in reply to a specific PGN
// Default priority: 6
// Input:
// controlByte - 0 = ACK, 1 = NACK, 2 = NO_SECURITY, 3 = BUSY RE_REQUEST LATER
// groupFuncVal - group function value
// reservedAssignment - for future
// verifyAddress -Verifying the address input
// PGN_LSB the least recent 8 bits of the parameter group numbe
// PGN_MID second byte of parameter group number, bit 8 is MSB
// PGN_MSB the most significant 8 bits of the parameter group number
//
// Output:
//  - J1939Msg                J1939 message ready to be sent.
//*****************************************************************************
#define J1939_ACK_PGN 0x00E80016UL                                              // J1939 Acknowledge or negative acknowledge
#define J1939_BCAST_RSP_ACK 0x00FEEB16UL                                        // broadcast response ack

#if defined(D_FT900)
typedef struct J1939PACKED {
        uint8_t controlByte;                                                    // 0 = ACK 1 = NACK
        uint8_t groupFuncVal;                                                   // group function value
        uint16_t reservedAssignment;
        uint8_t verifyAddress;
        uint8_t PGN_LSB;                                                        // the PGN which has been sent and we are acknowledging
        uint8_t PGN_MID;
        uint8_t PGN_MSB;
} J1939_ACK_t;
#else
J1939PACKED (
typedef struct {
        uint8_t controlByte;                                                    // 0 = ACK 1 = NACK 2, Access denied (supported by PGN, but deny access on the security, 3, Do not respond (supported by PGN, but ECUB busy and unable to respond, re-request data at a later time
        uint8_t groupFuncVal;                                                   // group function value
        uint16_t reservedAssignment;
        uint8_t verifyAddress;
        uint8_t PGN_LSB;                                                        // the PGN which has been sent
        uint8_t PGN_MID;
        uint8_t PGN_MSB;
}) J1939_ACK_t;
#endif

//  Function: Address Claim PGN
// Default priority: 6
//
//  If the command is accepted, the configured Address gets immediately active. 
//  The new Preferred Address value is saved in the nonvolatile memory of the J1939 device,
//  being available at next power on. After successfully programming, an Address Claimed message 
//  is sent with the new Source Address.
//
// Output:
//  - J1939Msg                J1939 message ready to be sent.
//*****************************************************************************
#define J1939_ADDR_REQ_PGN(addr) (0x00EA00UL | (addr & 0xFFu))                  // J1939 Address request
#define J1939_ADDR_CLAIM_PGN 0x00EE00UL                                         // J1939 Adress claim reply
#define J1939_SIM_REPLY_PGN 0x00FEDAUL                                          // J1939 Software Information Message

#define J1939_MANU_CODE_GEFRAN 732U                                             /* Manufacturer code assigned by SAE to Gefran S.p.A., Provaglio d’Iseo (Italy) */
#define J1939_MANU_CODE_CARLING 741U                                            /* carling tech key pad which also transmits address claim at start-up */

#if defined(D_FT900)
typedef struct J1939PACKED {
        uint8_t IdLoByte;                                                       // Identity Number (low byte)
        uint8_t IdHiByte;                                                       // Identity Number (high byte)
        uint8_t ManuCodeLSB : 3u;                                               // Manufacturer Code (3 bits LSB)
        uint8_t IdMSBByte : 5u;                                                 // Identity Number (5 bits MSB)
        uint8_t ManuCodeMSB;                                                    // Manufacturer Code (byte MSB)
        uint8_t FuncInstLSB : 5u;                                               // Function Instance (5 bits LSB)
        uint8_t ecuInstance : 3u;                                               // ECU INstance (3 bits)
        uint8_t FuncInstMSB;                                                    // Function Instance MSB                                                       // the PGN which has been sent
        uint8_t VehicleSystem : 7u;                                             // Vehicle System
        uint8_t res_bit : 1u;                                                   // reserved
        uint8_t ArbAddrCapable : 1u;                                            // Arbitary address capable
        uint8_t IndusGroup : 3u;                                                // Industry Group
        uint8_t VehicleSysInst : 4u;                                            // Vehicle System Instance
        uint8_t newSourceAddr;                                                  // must be 2nd frame ? New Source Address (0 to 127 and 248 to 253)
} J1939_ADDR_CLAIM_t;
#else
J1939PACKED (
typedef struct {
        uint8_t IdLoByte;                                                       // Identity Number (low byte)
        uint8_t IdHiByte;                                                       // Identity Number (high byte)
        uint8_t ManuCodeLSB : 3u;                                               // Manufacturer Code (3 bits LSB)
        uint8_t IdMSBByte : 5u;                                                 // Identity Number (5 bits MSB)
        uint8_t ManuCodeMSB;                                                    // Manufacturer Code (byte MSB)
        uint8_t FuncInstLSB : 5u;                                               // Function Instance (5 bits LSB)
        uint8_t ecuInstance : 3u;                                               // ECU INstance (3 bits)
        uint8_t FuncInstMSB;                                                    // Function Instance MSB                                                       // the PGN which has been sent
        uint8_t VehicleSystem : 7u;                                             // Vehicle System
        uint8_t res_bit : 1u;                                                   // reserved
        uint8_t ArbAddrCapable : 1u;                                            // Arbitary address capable 0= The device is configured as Single Address Capable device - Command Configurable Address device. The device is not capable of selecting an alternate source address itself  1 = The device is capable of selecting an alternate source address itself, if needed during address arbitration, in the range of 128 to 247 inclusive.
        uint8_t IndusGroup : 3u;                                                // Industry Group
        uint8_t VehicleSysInst : 4u;                                            // Vehicle System Instance
        uint8_t newSourceAddr;                                                  // must be 2nd frame ? New Source Address (0 to 127 and 248 to 253)
}) J1939_ADDR_CLAIM_t;
#endif

// 0x00 0xEE 0x00 for SAE J1939 Address Claim Message (PGN 0x00EE0
// 0xDA, 0xFE, 0x00 for SAE J1939 Software Identification Message (PGN 0x00FEDA
#define J1939_ACM_REPLY_DATA { 0x00, 0xEE, 0x00 }
#define J1939_SIM_REPLY_DATA { 0xDA, 0xFE, 0x00 }

#if defined(D_FT900)
typedef struct J1939PACKED {
        uint8_t PGN_LSB;                                                        // the PGN which has been sent and we are acknowledging
        uint8_t PGN_MID;
        uint8_t PGN_MSB;
} J1939_ADDR_REPLY_t;
#else
J1939PACKED (
typedef struct {
        uint8_t PGN_LSB;                                                        // the PGN which has been sent and we are acknowledging
        uint8_t PGN_MID;
        uint8_t PGN_MSB;
}) J1939_ADDR_REPLY_t;
#endif

//  Function: Work machine identification, component ID and software identification PGN
// Default priority: 6
// Input:
// PGN_LSB the least recent 8 bits of the parameter group numbe
// PGN_MID second byte of parameter group number, bit 8 is MSB
// PGN_MSB the most significant 8 bits of the parameter group number
// specialIns : special instructions
// response = 00-none, 01-yes, 10 - Undefined, 11 - Dissoed
//
// Output:
//  - J1939Msg                J1939 message ready to be sent.
//*****************************************************************************
#define J1939_WORK_MACH_ID_PGN 0x00C90016UL                                     // J1939 Work machine identification, component ID and software identification PGN
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint8_t PGN_LSB;                                                        // the PGN which has been sent and we are acknowledging
        uint8_t PGN_MID;
        uint8_t PGN_MSB;
        uint8_t specialIns;                                                     // special instructions
        uint8_t response : 2u;                                                  // 00-none, 01-yes, 10 - Undefined, 11 - Dissoed
        uint8_t spare : 6u;
        uint32_t future;
} J1939_WorMachId_t;
#else
J1939PACKED (
typedef struct {
        uint8_t PGN_LSB;                                                        // the PGN which has been sent and we are acknowledging
        uint8_t PGN_MID;
        uint8_t PGN_MSB;
        uint8_t specialIns;                                                     // special instructions
        uint8_t response : 2u;                                                  // 00-none, 01-yes, 10 - Undefined, 11 - Dissoed
        uint8_t spare : 6u;
        uint32_t future;
}) J1939_WorMachId_t;
#endif

//  Function: STOP START BROADCAST (DM13)
// Default priority: 6
// Input:
// The DM13 message is used to stop or start broadcast messages.
// One of the uses for this message is to reduce network traffic during certain diagnostic procedures.
// With this message the periodic transmission of the Data Record message can be temporarily suspended
//
//
//
// Output:
//  - J1939Msg                J1939 message ready to be sent.
//*****************************************************************************
#define J1939_DM13_PGN 0x00DF00UL                                               // J1939 stop braodcast

typedef enum {
  DM13_Stop = 0,
  DM13_Start = 1,
  DM13_Reserved = 2,
  DM13_NoAction = 3
} J1939_dm13_e;

#if defined(D_FT900)
typedef struct J1939PACKED {
    uint8_t CurrentDataLinkBits : 2u;
    uint8_t J1587Bits : 2u;
    uint8_t J1922Bits : 2u;
    uint8_t J1939Network1 : 2u;
    uint8_t J1939Network2 : 2u;
    uint8_t ISO9141 : 2u;
    uint8_t J1850Bits : 2u;
    uint8_t OtherManufacture : 2u;
    uint8_t J1939Network3 : 2u;
    uint8_t ProprietaryNetwork1 : 2u;
    uint8_t ProprietaryNetwork2 : 2u;
    uint8_t J1939Network4 : 2u;
    uint8_t HoldSignalBits : 4u;
    uint8_t SuspendSignal : 4u;
    uint16_t SuspendDuration;
    uint16_t SAEReserved;
} J1939_DM13_t;
#else
J1939PACKED (
typedef struct {
    uint8_t CurrentDataLinkBits : 2u;
    uint8_t J1587Bits : 2u;
    uint8_t J1922Bits : 2u;
    uint8_t J1939Network1 : 2u;
    uint8_t J1939Network2 : 2u;
    uint8_t ISO9141 : 2u;
    uint8_t J1850Bits : 2u;
    uint8_t OtherManufacture  : 2u;
    uint8_t J1939Network3 : 2u;
    uint8_t ProprietaryNetwork1 : 2u;
    uint8_t ProprietaryNetwork2 : 2u;
    uint8_t J1939Network4 : 2u;
    uint8_t HoldSignalBits : 4u;
    uint8_t SuspendSignal : 4u;
    uint16_t SuspendDuration;
    uint16_t SAEReserved;
}) J1939_DM13_t;
#endif

//  Function: Control PGN
// Default priority: 6
// Input:
// PGN_LSB the least recent 8 bits of the parameter group numbe
// PGN_MID second byte of parameter group number, bit 8 is MSB
// PGN_MSB the most significant 8 bits of the parameter group number
//
// Output:
//  - J1939Msg                J1939 message ready to be sent.
//*****************************************************************************
#define J1939_CONTROL_PGN 0x00EC0016UL                                          // J1939 control TP.CM_CTS TP.CM_EndofMsgACK TP.Conn_Abort  TP.CM_BAM
#define J1939_CONTROL1_PGN 0x00EC00UL                                           // another mode has this one

typedef enum {
  J1939_CMRTS = 16,                                                             // Out of Send Request Connection Mode (TP.CM_RTS
  J1939_CMCTS = 17,                                                             // Sendable Connection Mode (TP.CM_CTS) CM_CTS message is used to respond to Request To Send messages. A certain amount notify the other controller that it is ready for a long message. Paquets that can be sent must not exceed the value CM_RTS byte 5 of the source controller TP.
  J1939_EndOfMsgAck = 19,                                                       // Message Termination Confirmation (TP.CM_EndofMsgACK) ndicates that all messages have been received correctly and re-assembled. from the controller that received the long message to the source controller
  J1939_BroadCastNotify = 32,                                                   // Broadcast Notification Messages (TP.CM_BAM CM_BAM is sent only by the source controller
  J1939_Ctrl_Abort= 255                                                         // Aborting a connection (TP.Conn_Abort)
} J1939_ControlMsg_e;

typedef enum {
  J1939_Already = 1,                                                            // Another connection cannot be supported because there is already one or more connection management sessions
  J1939_OutResource = 2,                                                        // This connection management session was terminated because system resources were required for another task
  J1939_TimeOut = 3,                                                            // Time-out occurs. This indicates that the connection was aborted to close the session
  J1939_ISO11783_1 = 251,                                                       // 251-255 According to the definition of ISO 11783-7 and ISO 11783-8.
  J1939_ISO11783_2 = 252,                                                       // 251-255 According to the definition of ISO 11783-7 and ISO 11783-8.
  J1939_ISO11783_3 = 253,                                                       // 251-255 According to the definition of ISO 11783-7 and ISO 11783-8.
  J1939_ISO11783_4 = 254,                                                       // 251-255 According to the definition of ISO 11783-7 and ISO 11783-8.
  J1939_ISO11783_5 = 255,                                                       // 251-255 According to the definition of ISO 11783-7 and ISO 11783-8.
} J1939_Control_Abortion_e;

#if defined(D_FT900)
typedef struct J1939PACKED {
        uint8_t controlMsg;                                                     // J1939_ControlMsg_e
        uint8_t Byte2;                                                          // for TP.Conn_Abort reason of abort, for TP.CM_BAM All message sizes, bytes
        uint8_t Byte3;                                                          // for TP.CM_BAM All message sizes, bytes
        uint8_t Byte4;                                                          // for TP.CM_BAM All Packets
        uint8_t Byte5;                                                          // described as In the allocation reservation for future standardization, this byte must not be sent as FF16 ??
        uint8_t PGN_LSB;                                                        // PGN of packet messages
        uint8_t PGN_MID;
        uint8_t PGN_MSB;
} J1939_ControlMsg_t;
#else
J1939PACKED (
typedef struct {
        uint8_t controlMsg;                                                     // J1939_ControlMsg_e
        uint8_t Byte2;                                                          // for TP.Conn_Abort reason of abort, for TP.CM_BAM All message sizes, bytes
        uint8_t Byte3;                                                          // for TP.CM_BAM All message sizes, bytes
        uint8_t Byte4;                                                          // for TP.CM_BAM All Packets
        uint8_t Byte5;                                                          // described as In the allocation reservation for future standardization, this byte must not be sent as FF16 ??
        uint8_t PGN_LSB;                                                        // PGN of packet messages
        uint8_t PGN_MID;
        uint8_t PGN_MSB;
}) J1939_ControlMsg_t;
#endif

/* ====== stateEngine ======== */
/*       TP.CM_RTS---->        */
/*       <-----TP.CTS          */
/*       TP.DT---->            */
/*       <-----TP.CTS          */
/*       TP.DT---->            */
/*       <-----TP.EOMA         */

#define J1939_DSR_PGN 0x0000EA00UL                                              // J1939 DESTINATION SPECIFIC REQUEST  (info unknown ?? )

// Function: Transport Protocol - Data Transfer (TP.DT) Message
// Default priority: 6
// Input:
// PGN_LSB the least recent 8 bits of the parameter group numbe
// PGN_MID second byte of parameter group number, bit 8 is MSB
// PGN_MSB the most significant 8 bits of the parameter group number
//
// Output:
//  - J1939Msg                J1939 message ready to be sent.
//*****************************************************************************
#define J1939_TP_DTM_PGN 0x00EB0016UL                                           // J1939 control TP.CM_CTS TP.CM_EndofMsgACK TP.Conn_Abort  TP.CM_BAM
#define J1939_TP_DTM1_PGN 0x00EB00UL                                            // aNOTHER FORMAT J1939 control TP.CM_CTS TP.CM_EndofMsgACK TP.Conn_Abort  TP.CM_BAM
#define J1939_TP_DTM_TIMEOUT 750u                                               // ms connection closes by either when a time greater than 750ms elapsed from the last packet received

#if defined(D_FT900)
typedef struct J1939PACKED {
        uint8_t SequenceNo;                                                     //sequence number of multi packet send 1 to 255
        uint8_t Byte[7u];                                                       // packetised data The last packet in a multi-packet parameter group may be less than 8 bytes. (Packets data is less than 7 bytes) the remaining bytes must be FF16
} J1939_TpDtm_t;
#else
J1939PACKED (
typedef struct {
        uint8_t SequenceNo;                                                     //sequence number of multi packet send 1 to 255
        uint8_t Byte[7u];                                                       // packetised data The last packet in a multi-packet parameter group may be less than 8 bytes. (Packets data is less than 7 bytes) the remaining bytes must be FF16
}) J1939_TpDtm_t;
#endif

#if defined(STW_TEMPERAT_USED)
/* SLI Battery 1 Temperature example Temperature Transmitter T01-CAN from Sensor-Technik Wiedemann GmbH */
#define J1939_SLI_BAT_TEMP_PGN 65104UL                                          /* bytes 1-4 */
#define J1939_AUX_TEMP_PGN 65164UL                                              /* bytes 1-2 */
#define J1939_TIRE_TEMP_PGN 65268UL                                             /* positions bytes 3-4 */
#define J1939_VEH_AMB_TEMP_PGN 65269UL                                          /* as below */
#define J1939_CARGO_AMB_TEMP 65276UL                                            /* bytes 5-6 not sure if was typo and should be 65275 or bytes 1-2 ? */

#define SLI_BAT_NUMS 4u

#if defined(D_FT900)
typedef struct J1939PACKED {
        uint8_t Temps[SLI_BAT_NUMS];                                            // temperatures raw
        float64_t calcTemp[SLI_BAT_NUMS];                                       // temperatures calculated
} J1939_SliBattTemp_t;
#else
J1939PACKED (
typedef struct {
        uint8_t Temps[SLI_BAT_NUMS];                                            // temperatures raw
        float64_t calcTemp[SLI_BAT_NUMS];                                       // temperatures calculated
}) J1939_SliBattTemp_t;
#endif

#if defined(D_FT900)
typedef struct J1939PACKED {
        uint8_t spare;
        uint16_t cabInterior;                                                   /* cab interior temp */
        uint16_t ambAir;                                                        /* ambient air temperatuere */
        uint8_t engIntakeAir;                                                   /* engine intake air temperature */
        uint16_t roadSurface;                                                   /* road surface temperature */
        float64_t calcTemp[SLI_BAT_NUMS];                                       // temperatures calculated
} J1939_VehAmbTemp_t;
#else
J1939PACKED (
typedef struct {
        uint8_t spare;
        uint16_t cabInterior;                                                   /* cab interior temp */
        uint16_t ambAir;                                                        /* ambient air temperatuere */
        uint8_t engIntakeAir;                                                   /* engine intake air temperature */
        uint16_t roadSurface;                                                   /* road surface temperature */
        float64_t calcTemp[SLI_BAT_NUMS];                                       // temperatures calculated
}) J1939_VehAmbTemp_t;
#endif

#endif /* st wiedermann temperature probe */

#if defined(GEFRAN_PRESSURE_USED)
/* =============== Gefran Preessure Sensor ================================== */
#define GEF_TIRE_PRESSURE_PGN 65146UL                                           /* proirity 7 */
#define GEF_TIRE_COND_PGN 65268UL                                               /* prio 6 byte 2 */
#define GEF_AMBIENT_PRESS_PGN 65269UL                                           /* prio 6 */
#define GEF_HYDRAULIC_FAN_PGN 65213UL                                           /* hydraulic fan motor pressure  prio 6 bytes 5-6 */
#define GEF_PRESS_4BAR_PGN 65280UL                                              /* prio 6 */
#define GEF_PRESS_6BAR_PGN 65281UL                                              /* prio 6 */
#define GEF_AIR_SUP_PRES_PGN 65198UL                                            /* prio 6 air supply pressure */
#define GEF_TRR_PGN 0x00B200UL                                                  /* transmission repitition rate TRR bytes 4-5 */
#define GEF_TRR_PACKET_DATA(x) { 0x67u, 0x65u, 0x66u, 0x72u, x<<8u, x, 0x00, 0x00 } /* default packet set up 'gefr' in bytes 1-4 for TRR */

#define GEF_PRES_SAEpr05 0.005f                                                 // 0 to 1.25 GEF_AMBIENT_PRESS_PGN or GEF_PRESS_4BAR_PGN
#define GEF_PRES_SAEpr07 0.02f                                                  // 0 to 5
#define GEF_PRES_SAEpr06 0.005f                                                 // 0 to 321.275bar used for TIRE_PRESSURE
#define GEF_PRES_SAEpr13 0.08f                                                  // 0 to 20 for GEF_AIR_SUP_PRES_PGN

/* unused bytes are sent as 0xFF */
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint16_t trailer;                                                       /* Trailer, Tag Or Push Channel Tire Pressure  */
        uint16_t drive;                                                         /* drive channel tire pressure */
        uint16_t steer;                                                         /* steer channel tire pressure */
        float64_t calcPress[3u];                                                /* pressures calculated  */
} J1939_TirePress_t;
#else
J1939PACKED (
typedef struct {
        uint16_t trailer;                                                       /* Trailer, Tag Or Push Channel Tire Pressure  */
        uint16_t drive;                                                         /* drive channel tire pressure */
        uint16_t steer;                                                         /* steer channel tire pressure */
        float64_t calcPress[3u];                                                /* pressures calculated */
}) J1939_TirePress_t;
#endif

#if defined(D_FT900)
typedef struct J1939PACKED {
        uint8_t PneumaticSupplyPressure;
        uint8_t ParkingTrailerAirPressure;
        uint8_t ServiceBrakeCircuit1;
        uint8_t ServiceBrakeCircuit2;
        uint8_t AuxiliaryEquipmentSupplyPressure;
        uint8_t AirSuspensionSupplyPressure;
        uint8_t spare;
        uint8_t PowertrainCircuitAirSupplyPressure;
        float64_t calcPress[7u];                                                // pressures calculated
} J1939_AirSupplyPress_t;
#else
J1939PACKED (
typedef struct {
        uint8_t PneumaticSupplyPressure;
        uint8_t ParkingTrailerAirPressure;
        uint8_t ServiceBrakeCircuit1;
        uint8_t ServiceBrakeCircuit2;
        uint8_t AuxiliaryEquipmentSupplyPressure;
        uint8_t AirSuspensionSupplyPressure;
        uint8_t spare;
        uint8_t PowertrainCircuitAirSupplyPressure;
        float64_t calcPress[7u];                                                // pressures calculated
}) J1939_AirSupplyPress_t;
#endif

#endif /* gefran pressure sensor */

#if defined(CARLING_KEYPAD_USED)
/* =============== keypad CarlingTech CKP =================================== */
#define CKP_KEYPRESS_PGN 0x00FED9UL                                             /* proirity 6 */
#if defined(D_FT900)
typedef struct J1939PACKED {
       uint8_t key4 : 2u;
       uint8_t key3 : 2u;
       uint8_t key2 : 2u;
       uint8_t key1 : 2u;
       uint8_t key8 : 2u;
       uint8_t key7 : 2u;
       uint8_t key6 : 2u;
       uint8_t key5 : 2u;
       uint8_t key12 : 2u;
       uint8_t key11 : 2u;
       uint8_t key10 : 2u;
       uint8_t key9 : 2u;
       uint16_t unused;
} ckp_keys_t;
#else
J1939PACKED (
typedef struct {
       uint8_t key4 : 2u;
       uint8_t key3 : 2u;
       uint8_t key2 : 2u;
       uint8_t key1 : 2u;
       uint8_t key8 : 2u;
       uint8_t key7 : 2u;
       uint8_t key6 : 2u;
       uint8_t key5 : 2u;
       uint8_t key12 : 2u;
       uint8_t key11 : 2u;
       uint8_t key10 : 2u;
       uint8_t key9 : 2u;
       uint16_t unused;
}) ckp_keys_t;
#endif
#define CKP_LEDBANK1_PGN(addr) ( (0x00A7UL << 8u) | addr )                      /* proirity 6 */
#define CKP_LEDBANK2_PGN(addr) ( (0x00A6UL << 8u) | addr )                      /* proirity 6 */
typedef enum {
  CKP_LedOff = 0,
  CKP_LedOn = 1,
  CKP_LedError = 2,
  CKP_NoAction = 3
} CKP_ledState_e;

#if defined(D_FT900)
typedef struct J1939PACKED {
       uint8_t led20 : 2u;                                                      // CKP_ledState_e
       uint8_t led19 : 2u;
       uint8_t led18 : 2u;
       uint8_t led17 : 2u;
       uint8_t led24 : 2u;
       uint8_t led23 : 2u;
       uint8_t led22 : 2u;
       uint8_t led21 : 2u;
       uint8_t led28 : 2u;
       uint8_t led27 : 2u;
       uint8_t led26 : 2u;
       uint8_t led25 : 2u;
       uint8_t led32 : 2u;
       uint8_t led31 : 2u;
       uint8_t led30 : 2u;
       uint8_t led29 : 2u;
       uint8_t led36 : 2u;
       uint8_t led35 : 2u;
       uint8_t led34 : 2u;
       uint8_t led33 : 2u;
       uint8_t led40 : 2u;
       uint8_t led39 : 2u;
       uint8_t led38 : 2u;
       uint8_t led37 : 2u;
       uint8_t led44 : 2u;
       uint8_t led43 : 2u;
       uint8_t led42 : 2u;
       uint8_t led41 : 2u;
       uint8_t led48 : 2u;
       uint8_t led47 : 2u;
       uint8_t led46 : 2u;
       uint8_t led45 : 2u;
} ckp_led1_t;
#else
J1939PACKED (
typedef struct {
       uint8_t led20 : 2u;
       uint8_t led19 : 2u;
       uint8_t led18 : 2u;
       uint8_t led17 : 2u;
       uint8_t led24 : 2u;
       uint8_t led23 : 2u;
       uint8_t led22 : 2u;
       uint8_t led21 : 2u;
       uint8_t led28 : 2u;
       uint8_t led27 : 2u;
       uint8_t led26 : 2u;
       uint8_t led25 : 2u;
       uint8_t led32 : 2u;
       uint8_t led31 : 2u;
       uint8_t led30 : 2u;
       uint8_t led29 : 2u;
       uint8_t led36 : 2u;
       uint8_t led35 : 2u;
       uint8_t led34 : 2u;
       uint8_t led33 : 2u;
       uint8_t led40 : 2u;
       uint8_t led39 : 2u;
       uint8_t led38 : 2u;
       uint8_t led37 : 2u;
       uint8_t led44 : 2u;
       uint8_t led43 : 2u;
       uint8_t led42 : 2u;
       uint8_t led41 : 2u;
       uint8_t led48 : 2u;
       uint8_t led47 : 2u;
       uint8_t led46 : 2u;
       uint8_t led45 : 2u;
}) ckp_led1_t;
#endif
/* ==== brightness control of the unit =================== */
#define CKP_BRIGHTNESS_PGN(addr) ( (0x00D0UL << 8u) | addr )                    /* proirity 6 */
#if defined(D_FT900)
typedef struct J1939PACKED {
       uint8_t illumination;                                                    // 0.4% per bit
       uint8_t switchBacklight;
} ckp_brightness_t;
#else
J1939PACKED (
typedef struct {
       uint8_t illumination;                                                    // 0.4% per bit
       uint8_t switchBacklight;
}) ckp_brightness_t;
#endif

#endif /* end carling tech */
/* =============== Diagnostics ============================================== */
#define J1939_DM1_PGN 0xFECAUL                                                  // active diagnostic trouble codes, automatically send every second
#define J1939_DM2_PGN 0xFECBUL                                                  // previously active diagnostic trouble codes
#define J1939_DM3_PGN 0xFECCUL                                                  // diagnostic data clear/reset for previously active DTC

/* ====== stateEngine ======== */
/*       TP.CM_BAM---->        */
/*       TP.DT---->            */
/*       TP.DT---->            */
/*       TP.DT---->            */

/* =============== General J1939 ======================================== */
#if defined(D_FT900)
typedef struct J1939PACKED {
  uint8_t FullLen;
  uint8_t priority;
  uint32_t PGN;
  uint8_t dst;
  uint8_t src;
  uint32_t messageTyp;
  linak_rcv_t linakStatus;
  linak_snd_t linakCommand;
} j1939_Control_t;                                                              /* contains all polled items */
#else
J1939PACKED (
typedef struct
{
  uint8_t FullLen;
  uint8_t priority;
  uint32_t PGN;
  uint8_t dst;
  uint8_t src;
  uint32_t messageTyp;
  linak_rcv_t linakStatus;
  linak_snd_t linakCommand;
}) j1939_Control_t;                                                             /* contains all polled items */
#endif

//  Function: ISOBUS message  Wheel Based Speed and Distance
// Default priority: 3
// Input:
//
// Output:
//  - J1939Msg                J1939 message ready to be sent.
//*****************************************************************************
typedef enum {
  ISO_NotReversed = 0,
  ISO_Reversed = 1,
  ISO_Error = 2,
  ISO_NoAction = 3
} ISO_OpDirection_e;

typedef enum {
  ISO_MachReversed = 0,
  ISO_MachForward = 1,
  ISO_MachError = 2,
  ISO_MachNoAction = 3
} ISO_MachDirection_e;

typedef enum {
  ISO_KeyOff = 0,
  ISO_KeyNotOff = 1,
  ISO_KeyError = 2,
  ISO_KeyNoAction = 3
} ISO_KeySwitch_e;

typedef enum {
  ISO_StateStop = 0,
  ISO_StateStart = 1,
  ISO_StateError = 2,
  ISO_StateNoAction = 3
} ISO_StateStartStop_e;

#define ISOBUS_SPEED_DIST_PGN 0x00FE48UL                                        // Wheel Based Speed and Distance
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint16_t MachineSpeed;                                                  // 0.001 m/s per bit
        uint32_t MachineDistance;                                               // 0.001 m per bit
        uint8_t MaxTimeTractorPower;                                            // 1 min per bit
        uint8_t OpDirection : 2u;                                               // ISO_OpDirection_e
        uint8_t StartStopState : 2u;                                            // ISO_StateStartStop_e
        uint8_t KeySwitchState : 2u;                                            // ISO_KeySwitch_e
        uint8_t MachineDirection : 2u;                                          // ISO_MachDirection_e
} J1939_WheelSpdDist_t;
#else
J1939PACKED (
typedef struct {
        uint16_t MachineSpeed;                                                  // 0.001 m/s per bit
        uint32_t MachineDistance;                                               // 0.001 m per bit
        uint8_t MaxTimeTractorPower;                                            // 1 min per bit
        uint8_t OpDirection : 2u;                                               // ISO_OpDirection_e
        uint8_t StartStopState : 2u;                                            // ISO_StateStartStop_e
        uint8_t KeySwitchState : 2u;                                            // ISO_KeySwitch_e
        uint8_t MachineDirection : 2u;                                          // ISO_MachDirection_e
}) J1939_WheelSpdDist_t;
#endif

//  Function: ISOBUS message  Automatic Guidance control message
// Default priority: 3
// Input:
//
// Output:
//  - J1939Msg                J1939 message ready to be sent.
//*****************************************************************************
typedef enum {
  ISO_NoSteer = 0,
  ISO_IntendSteer = 1,
  ISO_SteerError = 2,
  ISO_SteerNoAction = 3
} ISO_curveCmdStat_e;

#define ISOBUS_AUT_GUIDE_PGN 0x00AD00UL                                        // guidance control to wheel control system
#if defined(D_FT900)
typedef struct J1939PACKED {
        uint16_t curvatureCmd;
        uint8_t cmdReserved : 6u;
        uint8_t curveCmdStat : 2u;
        uint32_t reserved1;
        uint8_t reserved2;
} ISO_AutoGuide_t;
#else
J1939PACKED (
typedef struct {
        uint16_t curvatureCmd;                                                  // scalar 0.25Km-1 per bit offset -8032Km-1  positive is forward and right
        uint8_t cmdReserved : 6u;
        uint8_t curveCmdStat : 2u;
        uint32_t reserved1;
        uint8_t reserved2;
}) ISO_AutoGuide_t;
#endif

/* define the object for use on the J1939 Can Line */
j1939_Control_t actCntrlObjt[ NumOfJ1939Devices ];

/* Compile CAN ID from J1939 sub fields */
static inline uint32_t get_j1939_id(uint8_t pri, uint32_t pgn, uint8_t sa)
{
    return (uint32_t) ( ((pri & ((1UL <<  3U) - 1UL)) << (18U + 8U)) + ((pgn & ((1UL << 18U) - 1UL)) << 8U) +  ((sa  & ((1UL <<  8U) - 1UL)) << 0U) );
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  /* end library */
#endif  /* end J1939 used */