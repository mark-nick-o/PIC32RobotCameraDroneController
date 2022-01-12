#ifndef __VOLZ_PROTO_
#define __VOLZ_PROTO_
/*
 * AP_VOLZ_PROTOCOL.h
 *
 *  Created on: Oct 31, 2017
 *      Author: guy tzoler  Ported by ACP Aviation
 *
 * Baud-Rate: 115.200 bits per second
 * Number of Data bits: 8
 * Number of Stop bits: 1
 * Parity: None
 * Half/Full Duplex: Half Duplex
 *
 * Volz Command and Response are all 6 bytes
 *
 * Command
 * byte        |        Communication Type
 * 1                Command Code
 * 2                Actuator ID
 * 3                Argument 1
 * 4                Argument 2
 * 5                CRC High-byte
 * 6                CRC        Low-Byte
 *
 * byte        |        Communication Type
 * 1                Response Code
 * 2                Actuator ID
 * 3                Argument 1
 * 4                Argument 2
 * 5                CRC High-byte
 * 6                CRC        Low-Byte
 *
 */
 
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define VOLZPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define VOLZPACKED __attribute__((packed))                                    /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define VOLZPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define VOLZPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define VOLZPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define VOLZ_EXTENDED_POSITION_MIN 0x0080U                                      // Extended Position Data Format defines -100 as 0x0080 decimal 128
#define VOLZ_EXTENDED_POSITION_CENTER 0x0800U                                   // Extended Position Data Format defines 0 as 0x0800 - decimal 2048
#define VOLZ_EXTENDED_POSITION_MAX 0x0F80U                                      // Extended Position Data Format defines +100 as 0x0F80 decimal 3968 -> full range decimal 3840
#define VOLZ_SCALE_VALUE (uint16_t)(VOLZ_EXTENDED_POSITION_MAX - VOLZ_EXTENDED_POSITION_MIN) // Extended Position Data Format defines 100 as 0x0F80, which results in 1920 steps for +100 deg and 1920 steps for -100 degs meaning if you take movement a scaled between -1 ... 1 and multiply by 1920 you get the travel from center
#define VOLZ_SET_EXTENDED_POSITION_CMD 0xDCU
#define VOLZ_SET_EXTENDED_POSITION_RSP 0x2CU
#define VOLZ_DATA_FRAME_SIZE 6u

#define VOLZ_PWM_MAX 1023U
#define VOLZ_PWM_MIN 0u

#if defined(D_FT900)
typedef struct VOLZPACKED {
  uint16_t pwmValue;
  uint8_t chan;
  uint8_t uartNo;
} VolZ_t;
#else
VOLZPACKED(
typedef struct {
  uint16_t pwmValue;
  uint8_t chan;
  uint8_t uartNo;
}) VolZ_t;                                                                      // volz protocol
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif