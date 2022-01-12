/* ========= Filter Types Library =========================================

 * Written : Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
 *
 * Contains ported code from
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
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "definitions.h"
#include "filter_types.h"
// #include "stdint.h"

#define M_LN2_FLOAT 0.69314718055994530942f
#define BIQUAD_Q 1.0f / sqrt(2.0f)                                              /* quality factor - 2nd order butterworth*/

/* ============== Function Definitions ==================================== */
float32_t fastKalmanUpdate(fastKalman_t *filter, float32_t input);
float32_t laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float32_t input);
void fastKalmanInit(fastKalman_t *filter, float32_t q, float32_t r);
void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float32_t *buf);
float32_t biquadFilterApply(biquadFilter_t *filter, float32_t input);
float32_t biquadFilterApplyDF1(biquadFilter_t *filter, float32_t input);
void biquadFilterUpdateLPF(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate);
void biquadFilterUpdate(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate, float32_t Q, biquadFilterType_e filterType);
void biquadFilterInitLPF(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate);
void biquadFilterInit(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate, float32_t Q, biquadFilterType_e filterType);
float32_t filterGetNotchQ(float32_t centerFreq, float32_t cutoffFreq);
float32_t slewFilterApply(slewFilter_t *filter, float32_t input);
void slewFilterInit(slewFilter_t *filter, float32_t slewLimit, float32_t threshold);
float32_t pt1FilterApply(pt1Filter_t *filter, float32_t input);
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float32_t k);
void pt1FilterInit(pt1Filter_t *filter, float32_t k);
float32_t pt1FilterGain(uint16_t f_cut, float32_t dT);
float32_t lpf2pReset(lpf2pData* lpfData, float32_t sample);
float32_t lpf2pApply(lpf2pData* lpfData, float32_t sample);
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float32_t sample_freq, float32_t cutoff_freq);
void lpf2pInit(lpf2pData* lpfData, float32_t sample_freq, float32_t cutoff_freq);
int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt);
void init_second_order_low_pass(SecondOrderLowPass *filter, float32_t tau, float32_t Q, float32_t sample_time, float32_t value);
float32_t update_second_order_low_pass(SecondOrderLowPass *filter, float32_t value);
float32_t get_second_order_low_pass(SecondOrderLowPass *filter);
void init_butterworth_2_low_pass(Butterworth2LowPass *filter, float32_t tau, float32_t sample_time, float32_t  value);
float32_t update_butterworth_2_low_pass(Butterworth2LowPass *filter, float32_t value);
float32_t get_butterworth_2_low_pass(Butterworth2LowPass *filter);

/*-----------------------------------------------------------------------------
 *      pt1FilterGain : PT1 Low Pass filter
 *
 *
 *  Parameters: uint16_t f_cut, float32_t dT
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t pt1FilterGain(uint16_t f_cut, float32_t dT)
{
    float32_t RC = 1.0f / ( 2,0f * PI * f_cut);
    return dT / (RC + dT);
}

/*-----------------------------------------------------------------------------
 *      pt1FilterInit : PT1 Low Pass filter initialise
 *
 *
 *  Parameters: pt1Filter_t *filter, float32_t k
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void pt1FilterInit(pt1Filter_t *filter, float32_t k)
{
    filter->state = 0.0f;
    filter->k = k;
}

/*-----------------------------------------------------------------------------
 *      pt1FilterUpdateCutoff : PT1 Low Pass filter cutoff limit set
 *
 *
 *  Parameters: pt1Filter_t *filter, float32_t k
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float32_t k)
{
    filter->k = k;
}

/*-----------------------------------------------------------------------------
 *      pt1FilterApply : apply PT1 Low Pass filter 
 *
 *
 *  Parameters: pt1Filter_t *filter, float32_t input
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t pt1FilterApply(pt1Filter_t *filter, float32_t input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

/*-----------------------------------------------------------------------------
 *      slewFilterInit : Slew filter with limit init 
 *
 *
 *  Parameters: slewFilter_t *filter, float32_t slewLimit, float32_t threshold
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void slewFilterInit(slewFilter_t *filter, float32_t slewLimit, float32_t threshold)
{
    filter->state = 0.0f;
    filter->slewLimit = slewLimit;
    filter->threshold = threshold;
}

/*-----------------------------------------------------------------------------
 *      slewFilterApply : Slew filter with limit 
 *
 *
 *  Parameters: slewFilter_t *filter, float32_t input
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t slewFilterApply(slewFilter_t *filter, float32_t input)
{
    if (filter->state >= filter->threshold) 
    {
        if (input >= filter->state - filter->slewLimit) 
        {
            filter->state = input;
        }
    } 
    else if (filter->state <= -filter->threshold) 
    {
        if (input <= filter->state + filter->slewLimit) 
        {
            filter->state = input;
        }
    } 
    else 
    {
        filter->state = input;
    }
    return filter->state;
}

/*-----------------------------------------------------------------------------
 *      filterGetNotchQ : get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
 *                        Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
 *
 *  Parameters: float32_t centerFreq, float32_t cutoffFreq
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t filterGetNotchQ(float32_t centerFreq, float32_t cutoffFreq) 
{
    return (centerFreq * cutoffFreq) / ((centerFreq * centerFreq) - (cutoffFreq * cutoffFreq));
}

/*-----------------------------------------------------------------------------
 *      biquadFilterInit : biquad filter initialise
 *                        
 *
 *  Parameters: biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate, float32_t Q, biquadFilterType_e filterType
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void biquadFilterInit(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate, float32_t Q, biquadFilterType_e filterType)
{
    const float32_t omega = 2.0f * PI * filterFreq * refreshRate * 0.000001f;   // setup variables
    const float32_t sn = sin(omega);
    const float32_t cs = cos(omega);
    const float32_t alpha = sn / (2.0f * Q);

    float32_t b0 = 0.0f, b1 = 0.0f, b2 = 0.0f, a0 = 0.0f, a1 = 0.0f, a2 = 0.0f;

    switch (filterType) 
    {
    case FILTER_LPF:
        // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
        // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
        b0 = (1.0f - cs) * 0.5f;
        b1 = 1.0f - cs;
        b2 = (1.0f - cs) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cs;
        a2 = 1.0f - alpha;
        break;
    case FILTER_NOTCH:
        b0 =  1.0f;
        b1 = -2.0f * cs;
        b2 =  1.0f;
        a0 =  1.0f + alpha;
        a1 = -2.0f * cs;
        a2 =  1.0f - alpha;
        break;
    case FILTER_BPF:
        b0 = alpha;
        b1 = 0.0f;
        b2 = -alpha;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cs;
        a2 = 1.0f - alpha;
        break;
    }
    filter->b0 = b0 / a0;                                                       // precompute the coefficients
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;
    filter->x1 = filter->x2 = 0.0f;                                             // zero initial samples
    filter->y1 = filter->y2 = 0.0f;
}

/*-----------------------------------------------------------------------------
 *      biquadFilterInitLPF : biquad filter initialise
 *                        
 *
 *  Parameters: biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void biquadFilterInitLPF(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate)  /* sets up a biquad Filter */
{
    biquadFilterInit(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF);
}

/*-----------------------------------------------------------------------------
 *      biquadFilterUpdate : biquad filter update
 *                        
 *
 *  Parameters: biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate, float32_t Q, biquadFilterType_e filterType
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void biquadFilterUpdate(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate, float32_t Q, biquadFilterType_e filterType)
{
    float32_t x1 = filter->x1;                                                  // backup state
    float32_t x2 = filter->x2;
    float32_t y1 = filter->y1;
    float32_t y2 = filter->y2;
    biquadFilterInit(filter, filterFreq, refreshRate, Q, filterType);
    filter->x1 = x1;                                                            // restore state
    filter->x2 = x2;
    filter->y1 = y1;
    filter->y2 = y2;
}

/*-----------------------------------------------------------------------------
 *      biquadFilterUpdateLPF : biquad filter update
 *                        
 *
 *  Parameters: biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void biquadFilterUpdateLPF(biquadFilter_t *filter, float32_t filterFreq, uint32_t refreshRate)
{
    biquadFilterUpdate(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF);
}

/*-----------------------------------------------------------------------------
 *      biquadFilterApplyDF1 : biquad filter update
 *                        
 *
 *  Parameters: biquadFilter_t *filter, float32_t input
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t biquadFilterApplyDF1(biquadFilter_t *filter, float32_t input)         /* Computes a biquadFilter_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
{
    /* compute result */
    const float32_t result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    filter->x2 = filter->x1;                                                    /* shift x1 to x2, input to x1 */
    filter->x1 = input;
    filter->y2 = filter->y1;                                                    /* shift y1 to y2, result to y1 */
    filter->y1 = result;
    return result;
}

/*-----------------------------------------------------------------------------
 *      biquadFilterApply : biquad filter update
 *                        
 *
 *  Parameters: biquadFilter_t *filter, float32_t input
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t biquadFilterApply(biquadFilter_t *filter, float32_t input)            /* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
{
    const float32_t result = filter->b0 * input + filter->x1;
    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;
    return result;
}

/*-----------------------------------------------------------------------------
 *      laggedMovingAverageInit : laged moving average intialisation
 *                        
 *
 *  Parameters: laggedMovingAverage_t *filter, uint16_t windowSize, float32_t *buf
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float32_t *buf)
{
    filter->movingWindowIndex = 0;
    filter->windowSize = windowSize;
    filter->buf = buf;
    filter->primed = false;
}

/*-----------------------------------------------------------------------------
 *      fastKalmanInit : fast kalman intialisation
 *                        
 *
 *  Parameters: fastKalman_t *filter, float32_t q, float32_t r
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void fastKalmanInit(fastKalman_t *filter, float32_t q, float32_t r)             // Proper fast two-state Kalman
{
    filter->q     = q * 0.000001f;                                              // add multiplier to make tuning easier
    filter->r     = r * 0.001f;                                                 // add multiplier to make tuning easier
    filter->p     = q * 0.001f;                                                 // add multiplier to make tuning easier
    filter->x     = 0.0f;                                                       // set initial value, can be zero if unknown
    filter->lastX = 0.0f;                                                       // set initial value, can be zero if unknown
    filter->k     = 0.0f;                                                       // kalman gain
}

/*-----------------------------------------------------------------------------
 *      laggedMovingAverageUpdate : lagged moving average update
 *                        
 *
 *  Parameters: laggedMovingAverage_t *filter, float32_t input
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float32_t input)
{
    uint16_t denom;
    filter->movingSum -= filter->buf[filter->movingWindowIndex];
    filter->buf[filter->movingWindowIndex] = input;
    filter->movingSum += input;

    if (++filter->movingWindowIndex == filter->windowSize) {
        filter->movingWindowIndex = 0;
        filter->primed = true;
    }

    denom = filter->primed ? filter->windowSize : filter->movingWindowIndex;
    return filter->movingSum  / denom;
}

/*-----------------------------------------------------------------------------
 *      fastKalmanUpdate : fast kalman update
 *                        
 *
 *  Parameters: fastKalman_t *filter, float32_t input
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t fastKalmanUpdate(fastKalman_t *filter, float32_t input)
{
    filter->x += (filter->x - filter->lastX);                                   // project the state ahead using acceleration
    filter->lastX = filter->x;                                                  // update last state
    filter->p = filter->p + filter->q;                                          // prediction update
    filter->k = filter->p / (filter->p + filter->r);                            // measurement update
    filter->x += filter->k * (input - filter->x);
    filter->p = (1.0f - filter->k) * filter->p;
    return filter->x;
}

/**
 * IIR filter the samples.
 */
/*-----------------------------------------------------------------------------
 *      iirLPFilterSingle : iir filter
 *                        
 *
 *  Parameters: int32_t in, int32_t attenuation,  int32_t* filt
 *
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt)
{
  int32_t inScaled;
  int32_t filttmp = *filt;
  int16_t out;

  if (attenuation > (1<<IIR_SHIFT))
  {
    attenuation = (1<<IIR_SHIFT);
  }
  else if (attenuation < 1)
  {
    attenuation = 1;
  }
  inScaled = in << IIR_SHIFT;                                                           // Shift to keep accuracy
  filttmp = filttmp + (((inScaled-filttmp) >> IIR_SHIFT) * attenuation);                // Calculate IIR filter
  out = (filttmp >> 8) + ((filttmp & (1 << (IIR_SHIFT - 1))) >> (IIR_SHIFT - 1));       // Scale and round
  *filt = filttmp;
  return out;
}

/*-----------------------------------------------------------------------------
 *      lpf2pInit : 2-Pole low pass filter
 *                        
 *
 *  Parameters: lpf2pData* lpfData, float32_t sample_freq, float32_t cutoff_freq
 *
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
void lpf2pInit(lpf2pData* lpfData, float32_t sample_freq, float32_t cutoff_freq)
{
  if (lpfData == NULL || cutoff_freq <= 0.0f)
  {
    return;
  }

  lpf2pSetCutoffFreq(lpfData, sample_freq, cutoff_freq);
}

/*-----------------------------------------------------------------------------
 *      lpf2pSetCutoffFreq : lpf2p filter cut off frequency
 *                        
 *
 *  Parameters: lpf2pData* lpfData, float32_t sample_freq, float32_t cutoff_freq
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float32_t sample_freq, float32_t cutoff_freq)
{
  float32_t fr = sample_freq/cutoff_freq;
  float32_t ohm = tan(PI/fr);
  float32_t c = 1.0f+2.0f*cos(PI/4.0f)*ohm+ohm*ohm;
  lpfData->b0 = ohm*ohm/c;
  lpfData->b1 = 2.0f*lpfData->b0;
  lpfData->b2 = lpfData->b0;
  lpfData->a1 = 2.0f*(ohm*ohm-1.0f)/c;
  lpfData->a2 = (1.0f-2.0f*cos(PI/4.0f)*ohm+ohm*ohm)/c;
  lpfData->delay_element_1 = 0.0f;
  lpfData->delay_element_2 = 0.0f;
}

/*-----------------------------------------------------------------------------
 *      lpf2pApply : lpf2p filter cut off frequency
 *                        
 *
 *  Parameters: lpf2pData* lpfData, float32_t sample
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t lpf2pApply(lpf2pData* lpfData, float32_t sample)
{
  float32_t output;
  float32_t delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
  
  /*** if (!isfinite(delay_element_0))
  {
    delay_element_0 = sample;                                                    don't allow bad values to propigate via the filter
  }    TODO !!!!!!!!       ***/
  output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2; /* calculate the new value which for the pid is the derivative */
  lpfData->delay_element_2 = lpfData->delay_element_1;                          /* now calculate these for the next item in the future */
  lpfData->delay_element_1 = delay_element_0;
  return output;
}

/*-----------------------------------------------------------------------------
 *      lpf2pReset : lpf2p filter reset
 *                        
 *
 *  Parameters: lpf2pData* lpfData, float32_t sample
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t lpf2pReset(lpf2pData* lpfData, float32_t sample)
{
  float32_t dval = sample / (lpfData->b0 + lpfData->b1 + lpfData->b2);
  lpfData->delay_element_1 = dval;
  lpfData->delay_element_2 = dval;
  return lpf2pApply(lpfData, sample);
}

/** Init second order low pass filter.
 *
 * @param filter second order low pass filter structure
 * @param tau time constant of the second order low pass filter
 * @param Q Q value of the second order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
/*-----------------------------------------------------------------------------
 *      init_second_order_low_pass : Init second order low pass filter.
 *                        
 *
 *  Parameters: SecondOrderLowPass *filter, float32_t tau, float32_t Q, float32_t sample_time, float32_t value
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/ 
void init_second_order_low_pass(SecondOrderLowPass *filter, float32_t tau, float32_t Q, float32_t sample_time, float32_t value)
{
  float32_t K;
  float32_t poly;

  K = tan(filter->sample_time / (2.0f * tau));
  poly = K * K + K / Q + 1.0f;
  filter->a[0] = 2.0f * (K * K - 1.0f) / poly;
  filter->a[1] = (K * K - K / Q + 1.0f) / poly;
  filter->b[0] = K * K / poly;
  filter->b[1] = 2.0f * filter->b[0];
  filter->i[0] = filter->i[1] = filter->o[0] = filter->o[1] = value;
}

/** Update second order low pass filter state with a new value.
 *
 * @param filter second order low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
/*-----------------------------------------------------------------------------
 *      update_second_order_low_pass : perform second order low pass filter.
 *                        
 *
 *  Parameters: SecondOrderLowPass *filter, float32_t value
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/ 
float32_t update_second_order_low_pass(SecondOrderLowPass *filter, float32_t value)
{
  float32_t out = filter->b[0] * value + filter->b[1] * filter->i[0] + filter->b[0] * filter->i[1] - filter->a[0] * filter->o[0] - filter->a[1] * filter->o[1];
  filter->i[1] = filter->i[0];
  filter->i[0] = value;
  filter->o[1] = filter->o[0];
  filter->o[0] = out;
  return out;
}

/** Get current value of the second order low pass filter.
 *
 * @param filter second order low pass filter structure
 * @return current value of the filter
 */
/*-----------------------------------------------------------------------------
 *      get_second_order_low_pass : Get current value of the second order low pass filter.
 *                        
 *
 *  Parameters: SecondOrderLowPass *filter
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/ 
float32_t get_second_order_low_pass(SecondOrderLowPass *filter)
{
  return filter->o[0];
}
/** Init a second order Butterworth filter.
 *
 * based on the generic second order filter
 * with Q = 0.7071 = 1/sqrt(2)
 *
 * http://en.wikipedia.org/wiki/Butterworth_filter
 *
 * @param filter second order Butterworth low pass filter structure
 * @param tau time constant of the second order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
/*-----------------------------------------------------------------------------
 *      init_butterworth_2_low_pass : Init a second order Butterworth filter.
 *                        
 *
 *  Parameters: Butterworth2LowPass *filter, float32_t tau, float32_t sample_time, float32_t  value
 *
 *  Return:     void 
 *----------------------------------------------------------------------------*/ 
void init_butterworth_2_low_pass(Butterworth2LowPass *filter, float32_t tau, float32_t sample_time, float32_t  value)
{
  init_second_order_low_pass((SecondOrderLowPass *)filter, tau, 0.7071f, sample_time, value);
}

/** Update second order Butterworth low pass filter state with a new value.
 *
 * @param filter second order Butterworth low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
/*-----------------------------------------------------------------------------
 *      update_butterworth_2_low_pass : Update second order Butterworth low pass filter state with a new value.
 *                        
 *
 *  Parameters: Butterworth2LowPass *filter, float32_t value
 *
 *  Return:     float32_t 
 *----------------------------------------------------------------------------*/ 
float32_t update_butterworth_2_low_pass(Butterworth2LowPass *filter, float32_t value)
{
  return update_second_order_low_pass((SecondOrderLowPass *)filter, value);
}

/** Get current value of the second order Butterworth low pass filter.
 *
 * @param filter second order Butterworth low pass filter structure
 * @return current value of the filter */
/*-----------------------------------------------------------------------------
 *      get_butterworth_2_low_pass : Get current value of the second order Butterworth low pass filter.
 *                        
 *
 *  Parameters: Butterworth2LowPass *filter
 *
 *  Return:     float32_t 
 *----------------------------------------------------------------------------*/
float32_t get_butterworth_2_low_pass(Butterworth2LowPass *filter)
{
  return filter->o[0];
}