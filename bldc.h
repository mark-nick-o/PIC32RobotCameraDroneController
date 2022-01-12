#ifndef __bldc_lib_
#define __bldc_lib_
#include <stdint.h>
#include "Struts.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

extern void PWMStop( uint16_t channel );
extern void PWMStart( uint16_t channel );
extern uint16_t PWMinit(uint32_t freq_hz, uint16_t enable_channel_x, uint16_t timer_prescale, uint16_t use_timer_x);
#if defined(FUTABA303_USED)
extern void phases_set(int16_t pwm, bldc_phase_e p1, bldc_phase_e p2);
extern void bldc_setup();
extern void bldc_commutate(int16_t pwm, uint16_t state);
extern void rotateFutaba( futaba_s3003_obj_t *futaba, uint16_t channel, uint32_t freqHz, pwm_preScale_e timer_prescale, pwm_timer_e use_timer_x );
#endif /* futaba bldc */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif