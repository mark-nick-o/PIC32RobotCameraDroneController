#ifndef _tramp_H
#define _tramp_H
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * ported by acp aviation
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

//#pragma once
//#include "drivers/serial.h"
#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define TRAMPPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define TRAMPPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define TRAMPPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define TRAMPPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define TRAMPPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define TRAMP_SERIAL_OPTIONS SERIAL_NOT_INVERTED | SERIAL_BIDIR
#define TRAMP_BAUD 9600u
#define TRAMP_PAYLOAD_LENGTH 12u

#define TRAMP_SYNC_START            0x0F
#define TRAMP_SYNC_STOP             0x00
#define TRAMP_COMMAND_SET_FREQ      'F' // 0x46
#define TRAMP_COMMAND_SET_POWER     'P' // 0x50
#define TRAMP_COMMAND_ACTIVE_STATE  'I' // 0x49
#define TRAMP_COMMAND_GET_CONFIG    'v' // 0x76

#if defined(D_FT900)
typedef struct TRAMPPACKED {
    uint16_t frequency;
    uint16_t power;
    uint8_t raceModeEnabled;
    uint8_t pitModeEnabled;
} trampSettings_t;
#else
TRAMPPACKED (
typedef struct trampSettings_s {
    uint16_t frequency;
    uint16_t power;
    uint8_t raceModeEnabled;
    uint8_t pitModeEnabled;
}) trampSettings_t;
#endif

#if defined(D_FT900)
typedef struct TRAMPPACKED {
    uint8_t syncStart;
    uint8_t command;
} trampFrameHeader_t;
#else
TRAMPPACKED (
typedef struct trampFrameHeader_s {
    uint8_t syncStart;
    uint8_t command;
}) trampFrameHeader_t;
#endif
#define TRAMP_HEADER_LENGTH sizeof(trampFrameHeader_t)

#if defined(D_FT900)
typedef struct TRAMPPACKED {
    uint8_t crc;
    uint8_t syncStop;
} trampFrameFooter_t;
#else
TRAMPPACKED (
typedef struct trampFrameFooter_s {
    uint8_t crc;
    uint8_t syncStop;
}) trampFrameFooter_t;
#endif

typedef union trampPayload_u {
    uint8_t buf[TRAMP_PAYLOAD_LENGTH];
    trampSettings_t settings;
    uint16_t frequency;
    uint16_t power;
    uint8_t active;
} trampPayload_t;

#if defined(D_FT900)
typedef struct TRAMPPACKED {
    trampFrameHeader_t header;
    trampPayload_t payload;
    trampFrameFooter_t footer;
} trampFrame_t;
#else
TRAMPPACKED (
typedef struct trampFrame_s {
    trampFrameHeader_t header;
    trampPayload_t payload;
    trampFrameFooter_t footer;
}) trampFrame_t;
#endif
#define TRAMP_FRAME_LENGTH sizeof(trampFrame_t)

#endif