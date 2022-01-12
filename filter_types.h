#ifndef __filter_types__
#define __filter_types__
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
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * Ported for mikroE PIC32 C by ACP Aviation
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
 /**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * filter.h - Filtering functions
 * Ported for MikRo E by ACP Aviation 2020
 */
#include <stdint.h>
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define IIR_SHIFT 8u

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define FILTERPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define FILTERPACKED __attribute__((packed)) ALIGNED(1)                        /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define FILTERPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define FILTERPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define FILTERPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif
#ifndef bool
#define bool uint8_t
#endif
struct filter_s;
typedef struct filter_s filter_t;

typedef struct pt1Filter_s 
{
    float32_t state;
    float32_t k;
} pt1Filter_t;

typedef struct slewFilter_s 
{
    float32_t state;
    float32_t slewLimit;
    float32_t threshold;
} slewFilter_t;

typedef struct biquadFilter_s                                                   /* this holds the data required to update samples thru a filter */
{
    float32_t b0, b1, b2, a1, a2;
    float32_t x1, x2, y1, y2;
} biquadFilter_t;

typedef struct laggedMovingAverage_s 
{
    uint16_t movingWindowIndex;
    uint16_t windowSize;
    float32_t movingSum;
    float32_t *buf;
    bool primed;
} laggedMovingAverage_t;

typedef enum 
{
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_KALMAN,
} lowpassFilterType_e;

typedef enum 
{
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

typedef struct fastKalman_s 
{
    float32_t q;                                                                // process noise covariance
    float32_t r;                                                                // measurement noise covariance
    float32_t p;                                                                // estimation error covariance matrix
    float32_t k;                                                                // kalman gain
    float32_t x;                                                                // state
    float32_t lastX;                                                            // previous state
} fastKalman_t;

int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt);

#if defined(D_FT900)                                                           // Structure for a filter object to minimise noise
typedef struct FILTERPACKED {
  float32_t a1;
  float32_t a2;
  float32_t b0;
  float32_t b1;
  float32_t b2;
  float32_t delay_element_1;
  float32_t delay_element_2;
} lpf2pData;                                                                    // LPF filter object
#else
FILTERPACKED(
typedef struct {
  float32_t a1;
  float32_t a2;
  float32_t b0;
  float32_t b1;
  float32_t b2;
  float32_t delay_element_1;
  float32_t delay_element_2;
}) lpf2pData;                                                                    // LPF filter object
#endif

/** Second order low pass filter structure.
 *
 * using biquad filter with bilinear z transform
 *
 * http://en.wikipedia.org/wiki/Digital_biquad_filter
 * http://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform
 *
 * Laplace continious form:
 *
 *                 1
 * H(s) = -------------------
 *        s^2/w^2 + s/w*Q + 1
 *
 *
 * Polynomial discrete form:
 *
 *        b0 + b1 z^-1 + b2 z^-2
 * H(z) = ----------------------
 *        a0 + a1 z^-1 + a2 z^-2
 *
 * with:
 *  a0 = 1
 *  a1 = 2*(K^2 - 1) / (K^2 + K/Q + 1)
 *  a2 = (K^2 - K/Q + 1) / (K^2 + K/Q + 1)
 *  b0 = K^2 / (K^2 + K/Q + 1)
 *  b1 = 2*b0
 *  b2 = b0
 *  K = tan(pi*Fc/Fs) ~ pi*Fc/Fs = Ts/(2*tau)
 *  Fc: cutting frequency
 *  Fs: sampling frequency
 *  Ts: sampling period
 *  tau: time constant (tau = 1/(2*pi*Fc))
 *  Q: gain at cutoff frequency
 *
 * Note that b[0]=b[2], so we don't need to save b[2]
 */

#if defined(D_FT900)                                                                     // Structure for a filter object to minimise noise
typedef struct FILTERPACKED {
  float32_t a[2u];                                                              ///< denominator gains
  float32_t b[2u];                                                              ///< numerator gains
  float32_t i[2u];                                                              ///< input history
  float32_t o[2u];                                                              ///< output history
  uint64_t sample_time;                                                         // sample_time sampling period of the signal
} SecondOrderLowPass;                                                           // 2nd order low pass filter object
#else
FILTERPACKED(
typedef struct {
  float32_t a[2u];                                                              ///< denominator gains
  float32_t b[2u];                                                              ///< numerator gains
  float32_t i[2u];                                                              ///< input history
  float32_t o[2u];                                                              ///< output history
  uint64_t sample_time;                                                         // sample_time sampling period of the signal
}) SecondOrderLowPass;                                                          // 2nd order low pass filter object
#endif

#if defined(D_FT900)
typedef struct FILTERPACKED {
  int32_t a[2u];                                                                //< denominator gains
  int32_t b[2u];                                                                //< numerator gains
  int32_t i[2u];                                                                //< input history
  int32_t o[2u];                                                                //< output history
  int32_t loop_gain;                                                            //< loop gain
} SecondOrderLowPass_int;                                                       // second order low pass object
#else
// Structure for a filter object to minimise noise
FILTERPACKED(
typedef struct {
  int32_t a[2u];                                                                //< denominator gains
  int32_t b[2u];                                                                //< numerator gains
  int32_t i[2u];                                                                //< input history
  int32_t o[2u];                                                                //< output history
  int32_t loop_gain;                                                            //< loop gain
}) SecondOrderLowPass_int;                                                      // second order low pass object
#endif

/** Second order Butterworth low pass filter.
 */
//typedef struct SecondOrderLowPass Butterworth2LowPass;
#if defined(D_FT900)
typedef struct FILTERPACKED {
  float32_t a[2u];                                                              ///< denominator gains
  float32_t b[2u];                                                              ///< numerator gains
  float32_t i[2u];                                                              ///< input history
  float32_t o[2u];                                                              ///< output history
  uint64_t sample_time;                                                         // sample_time sampling period of the signal
} Butterworth2LowPass;                                                          // second order low pass object
#else
FILTERPACKED(                                                                   // Structure for a filter object to minimise noise
typedef struct {
  float32_t a[2u];                                                              ///< denominator gains
  float32_t b[2u];                                                              ///< numerator gains
  float32_t i[2u];                                                              ///< input history
  float32_t o[2u];                                                              ///< output history
  uint64_t sample_time;                                                         // sample_time sampling period of the signal
}) Butterworth2LowPass;                                                         // second order low pass object
#endif


//typedef float32_t (*filterApplyFnPtr)(filter_t *filter, float32_t input);
extern float32_t fastKalmanUpdate(fastKalman_t *filter, float32_t input);
extern float32_t laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float32_t input);
extern void fastKalmanInit(fastKalman_t *filter, float32_t q, float32_t r);
extern void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float32_t *buf);
extern float32_t biquadFilterApply(biquadFilter_t *filter, float32_t input);
extern float32_t biquadFilterApplyDF1(biquadFilter_t *filter, float32_t input);
extern void biquadFilterUpdateLPF(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate);
extern void biquadFilterUpdate(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate, float32_t Q, biquadFilterType_e filterType);
extern void biquadFilterInitLPF(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate);
extern void biquadFilterInit(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate, float32_t Q, biquadFilterType_e filterType);
extern float32_t filterGetNotchQ(float32_t centerFreq, float32_t cutoffFreq);
extern float32_t slewFilterApply(slewFilter_t *filter, float32_t input);
extern void slewFilterInit(slewFilter_t *filter, float32_t slewLimit, float32_t threshold);
extern float32_t pt1FilterApply(pt1Filter_t *filter, float32_t input);
extern void pt1FilterUpdateCutoff(pt1Filter_t *filter, float32_t k);
extern void pt1FilterInit(pt1Filter_t *filter, float32_t k);
extern float32_t pt1FilterGain(uint16_t f_cut, float32_t dT);
extern int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt);
extern void lpf2pInit(lpf2pData* lpfData, float32_t sample_freq, float32_t cutoff_freq);
extern void lpf2pSetCutoffFreq(lpf2pData* lpfData, float32_t sample_freq, float32_t cutoff_freq);
extern float32_t lpf2pApply(lpf2pData* lpfData, float32_t sample);
extern float32_t lpf2pReset(lpf2pData* lpfData, float32_t sample);
extern void init_second_order_low_pass(SecondOrderLowPass *filter, float32_t tau, float32_t Q, float32_t sample_time, float32_t value);
extern float32_t update_second_order_low_pass(SecondOrderLowPass *filter, float32_t value);
extern float32_t get_second_order_low_pass(SecondOrderLowPass *filter);
extern void init_butterworth_2_low_pass(Butterworth2LowPass *filter, float32_t tau, float32_t sample_time, float32_t  value);
extern float32_t update_butterworth_2_low_pass(Butterworth2LowPass *filter, float32_t value);
extern float32_t get_butterworth_2_low_pass(Butterworth2LowPass *filter);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif