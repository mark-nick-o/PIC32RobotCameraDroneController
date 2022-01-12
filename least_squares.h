#ifndef _LEASTsQUARES_H
#define _LEASTsQUARES_H
/**
 * This function has came from a port of the follwing code
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Incremental Least Squares Best Fit  By Roxy and Ed Williams
 *
 * This algorithm is high speed and has a very small code footprint.
 * Its results are identical to both the Iterative Least-Squares published
 * earlier by Roxy and the QR_SOLVE solution. If used in place of QR_SOLVE
 * it saves roughly 10K of program memory.   And even better...  the data
 * fed into the algorithm does not need to all be present at the same time.
 * A point can be probed and its values fed into the algorithm and then discarded.
 *
 */
#include <stdlib.h>
#include "definitions.h"                                                        // global defines

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define LSQAURE_PACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define LSQAURE_PACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define LSQAURE_PACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define LSQAURE_PACKED
#else                                                                           /* for MPLAB PIC32 */
  #define LSQAURE_PACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#if defined(D_FT900)
typedef struct LSQAURE_PACKED {
  float64_t xbar;
  float64_t ybar;
  float64_t zbar;
  float64_t x2bar; 
  float64_t y2bar; 
  float64_t z2bar;
  float64_t xybar;
  float64_t xzbar; 
  float64_t yzbar;
  float64_t max_absx;
  float64_t max_absy;
  float64_t A; 
  float64_t B; 
  float64_t D; 
  float64_t N;
} linear_fit_data_t;
#else
LSQAURE_PACKED(
typedef struct  {
  float64_t xbar;
  float64_t ybar;
  float64_t zbar;
  float64_t x2bar; 
  float64_t y2bar; 
  float64_t z2bar;
  float64_t xybar;
  float64_t xzbar; 
  float64_t yzbar;
  float64_t max_absx;
  float64_t max_absy;
  float64_t A; 
  float64_t B; 
  float64_t D; 
  float64_t N;
}) linear_fit_data_t;                                                                
#endif

extern void incremental_LSF_reset(linear_fit_data_t *lsf);
extern void incremental_WLSF(linear_fit_data_t *lsf, const float64_t x, const float64_t y, const float64_t z, const float64_t w);
extern void incremental_LSF(linear_fit_data_t *lsf, const float64_t x, const float64_t y, const float64_t z);

#ifdef __cplusplus
}
#endif

#endif //_CAPACITY_H