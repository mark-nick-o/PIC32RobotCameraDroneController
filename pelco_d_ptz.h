// Pelco-D consists of 7 hexadecimal bytes (all byte data used in this page are in Hexadecimal format unless otherwise specified).
//
//   * Written : Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
#ifndef __pelco_d_ptz_h
#define __pelco_d_ptz_h

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define PTZPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define PTZPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define PTZPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define PTZPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define PTZPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define PTZ_SYNC_BYTE 0xFF                                                      // sync (first byte in message)

#if defined(D_FT900)
typedef struct PTZPACKED {
        uint8_t Sync;                                                           // Sync Char 0xFF
        uint8_t CamAddr;                                                        // Cam Address
        uint8_t Command1;                                                       // Command 1 (what it does)
        uint8_t Command2;                                                       // Command 2
        uint8_t Data1;                                                          // Data 1 (values e.g. speed)
        uint8_t Data2;                                                          // Data 2
        uint8_t Checksum;                                                       // Xor sum minus the start byte
} PTZ_pelco_d_t;                                                                   
#else
PTZPACKED(
typedef struct {
        uint8_t Sync;                                                           // Sync Char 0xFF
        uint8_t CamAddr;                                                        // Cam Address
        uint8_t Command1;                                                       // Command 1 (what it does)
        uint8_t Command2;                                                       // Command 2
        uint8_t Data1;                                                          // Data 1 (values e.g. speed)
        uint8_t Data2;                                                          // Data 2
        uint8_t Checksum;                                                       // Xor sum minus the start byte
}) PTZ_pelco_d_t;                                                                                                                      
#endif

// Pan Left at high speed:
#define PTZ_CMD1_LHS 0x00U
#define PTZ_CMD2_LHS 0x04U
#define PTZ_DATA1_LHS 0x3FU
#define PTZ_DATA2_LHS 0x00U

// Pan Right at medium speed:
#define PTZ_CMD1_RMS 0x00U
#define PTZ_CMD2_RMS 0x02U
#define PTZ_DATA1_RMS 0x20U
#define PTZ_DATA2_RMS 0x00U

// Tilt Up at high speed
#define PTZ_CMD1_UHS 0x00U
#define PTZ_CMD2_UHS 0x08U
#define PTZ_DATA1_UHS 0x00U
#define PTZ_DATA2_UHS 0x3FU

// Tilt Down at medium speed:
#define PTZ_CMD1_DMS 0x00U
#define PTZ_CMD2_DMS 0x10U
#define PTZ_DATA1_DMS 0x20U
#define PTZ_DATA2_DMS 0x00U

// STOP
#define PTZ_CMD1_STOP 0x00U
#define PTZ_CMD2_STOP 0x00U
#define PTZ_DATA1_STOP 0x00U
#define PTZ_DATA2_STOP 0x00U

// Command 1
// Command 1        Sense        Reserved        Reserved        Auto / Manual Scan        
// Camera On/Off        Iris Close        Iris Open        Focus Near
#define PTZ_CMD1_SENSE_BIT (1<<7)
#define PTZ_CMD1_AM_BIT (1<<4)
#define PTZ_CMD1_ONOFF_BIT (1<<3)
#define PTZ_CMD1_IRIS_CLOSE_BIT (1<<2)
#define PTZ_CMD1_IRIS_OPEN_BIT (1<<1)
#define PTZ_CMD1_FOCUS_NEAR_BIT (1<<0)

// Command 2
// Command 2        Focus Far        Zoom Wide        Zoom Tele        Tilt Down        
// Tilt Up        Pan Left        Pan Right        Fixed to 0
#define PTZ_CMD2_FOCUS_FAR_BIT (1<<7)
#define PTZ_CMD2_ZOOM_WIDE_BIT (1<<6)
#define PTZ_CMD2_ZOOM_TELE_BIT (1<<5)
#define PTZ_CMD2_TILT_DWN_BIT (1<<4)
#define PTZ_CMD2_TILT_UP_BIT (1<<3)
#define PTZ_CMD2_PAN_LEFT_BIT (1<<2)
#define PTZ_CMD2_PAN_RIGHT_BIT (1<<1)
#define PTZ_CMD2_FIXED0_BIT (1<<0)

// Go to Preset
#define PTZ_GO_TO_PRESET(X,Y) { X.Command1 = 0x00U; X.Command2 = 0x07U; X.Data1 = 0x00U; if (Y>=0xFFU) Y=0xFFU; X.Data2 = Y; }
#define PTZ_SET_ZOOM_SPEED(X,Y) { X.Command1 = 0x00U; X.Command2 = 0x25U; X.Data1 = 0x00U; if (Y>=0x33U) Y=0x33U; X.Data2 = Y; }
#define PTZ_SET_FOCUS(X,Y) { X.Command1 = 0x00U; X.Command2 = 0x27U; X.Data1 = 0x00U; if (Y>=0x33U) Y=0x33U; X.Data2 = Y; }
#define PTZ_ALARM_ACK(X,Y) { X.Command1 = 0x00U; X.Command2 = 0x19U; X.Data1 = 0x00U; if (Y>=0xFFU) Y=0xFFU; X.Data2 = Y; }

#ifdef __cplusplus
}
#endif

#endif