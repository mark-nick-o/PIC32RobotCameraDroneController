#include <stdint.h>
#include "Struts.h"
#include "io.h"
#include "gc_events.h"

void PWMStop( uint16_t channel );
void PWMStart( uint16_t channel );
uint16_t PWMinit(uint32_t freq_hz, uint16_t enable_channel_x, uint16_t timer_prescale, uint16_t use_timer_x);
#if defined(FUTABA303_USED)
void phases_set(int16_t pwm, bldc_phase_e p1, bldc_phase_e p2);
void bldc_setup();
void bldc_commutate(int16_t pwm, uint16_t state);
void rotateFutaba( futaba_s3003_obj_t *futaba, uint16_t channel, uint32_t freqHz, pwm_preScale_e timer_prescale, pwm_timer_e use_timer_x );
#endif

/*-----------------------------------------------------------------------------
 *      PWMinit():  initialize pwm  (returns pwm_period for set function)
 *
 *  Parameters: uint32_t freq_hz, uint16_t enable_channel_x, uint16_t timer_prescale, uint16_t use_timer_x
 *
 *  Return: return the calculated timer period or 0xFFFF for error
 *----------------------------------------------------------------------------*/
uint16_t PWMinit(uint32_t freq_hz, uint16_t enable_channel_x, uint16_t timer_prescale, uint16_t use_timer_x)
{
     CHECON = 0x32u;                                                             // cache control sfr : 2 wait states & enable predictive prefetch for cacheable and non-cacheable
     // set up your i/o
     //   AD1PCFG = 0xFFFF;                          // Configure AN pins as digital I/O
     //TRISB = 0xFFFF;                            // configure PORTB pins as input
     //PORTD = 0;                                 // set PORTD to 0
     //TRISD = 0;                                 // designate PORTD pins as output
     return( PWM_Init(freq_hz , enable_channel_x, timer_prescale, use_timer_x) );  // return the calculated timer period or 0xFFFF for error
}

/*-----------------------------------------------------------------------------
 *      PWMStart():  start pulse width modulating output
 *
 *  Parameters: uint16_t channel
 *
 *  Return: nothing
 *---------------------------------------------------------------------------- */
void PWMStart( uint16_t channel )
{
   PWMStart(channel);
}
/*-----------------------------------------------------------------------------
 *      PWMStop():  stop pulse width modulating output
 *
 *  Parameters: uint16_t channel
 *
 *  Return: nothing
 *---------------------------------------------------------------------------- */
void PWMStop( uint16_t channel )
{
   PWMStop(channel);
}

#if defined(FUTABA303_USED)
/*-----------------------------------------------------------------------------
 *      phases_set():  set up phases
 *
 *  Parameters: int16_t pwm, bldc_phase_e p1, bldc_phase_e p2
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void phases_set(int16_t pwm, bldc_phase_e p1, bldc_phase_e p2)
{

   static volatile uint16_t * ocr[] = {(uint16_t*)&OC1RS,(uint16_t*)&OC2RS,(uint16_t*)&OC3RS}; /* an array of the addresses of the PWM duty cycle-controlling SFRs  */
   
   /* If p1 and p2 are the energized phases, then floating[p1][p2] gives
      the floating phase. Note p1 should not equal p2 (that would be an error), so
      the diagonal entries in the 2d matrix are the bogus PHASE_NONE.   */
   static bldc_phase_e floating[3u][3u] = {{PHASE_NONE, PHASE_C, PHASE_B}, {PHASE_C, PHASE_NONE, PHASE_A}, {PHASE_B, PHASE_A, PHASE_NONE}};

   static int16_t elow_bits[3u] = {0b110, 0b101, 0b011};                        /* Takes the floating phase pfloat (e.g., pfloat could be PHASE_A) and returns a 3-bit value with a zero in the column corresponding to the floating phase (0th column = A, 1st column = B, 2nd column = C). */
   bldc_phase_e pfloat = floating[p1][p2];                                      /* the floating phase */
   bldc_phase_e phigh, plow;                                                    /* phigh is the PWMed phase, plow is the grounded phase */
   int16_t apwm;                                                                /* magnitude of the pwm count */


   if(pwm > 0)                                                                  /* choose the appropriate direction */
   {
      phigh = p1;
      plow = p2;
      apwm = pwm;
   }
   else
   {
      phigh = p2;
      plow = p1;
      apwm = -pwm;
   }
   DOTPORT3 = ((DOTPORT3 & ~(0x7u)) | elow_bits[pfloat]);                       /* Pin E0 controls enable for phase A; E1 for B; E2 for C. The pfloat phase should have its pin be 0; other pins should be 1. */
  
   /* set the PWM’s appropriately by setting the OCxRS SFRs      */
   *ocr[pfloat] = 0;                                                            /* floating pin has 0 duty cycle  */
   *ocr[plow] = 0;                                                              /* low will always be low, 0 duty cycle  */
   *ocr[phigh] = apwm;                                                          /* the high phase gets the actual duty cycle */
}

/*-----------------------------------------------------------------------------
 *      bldc_setup():  brushless dc motor set-up
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void bldc_setup()
{
  TRISECLR = 0x7;                                                               /* E0, E1, and E2 are outputs   */
  /* Set up timer 4 to use as the PWM clock source. */
  T4CON     = 0x1999U;                                                          /* Timer 4 prescale */
  T4IP0_bit = 0U;                                                               /* set interrupt */
  T4IP1_bit = 1U;                                                               /* priority */
  T4IP2_bit = 1U;                                                               /* to 6 */
  /* T4CONbits.TCKPS = 1;  Timer4 prescaler N=2 (1:2)  */
  /* PR6 = 1999;  period = (PR2+1) * N * 12.5 ns = 50 us, 20 kHz  */
  
  /* Set up OC1, OC2, and OC3 for PWM mode; use defaults otherwise.  */
  OC1CONbits.OCM = 0b110;                                                       /* PWM mode without fault pin; other OC1CON bits are defaults */
  OC2CONbits.OCM = 0b110;
  OC3CONbits.OCM = 0b110;
  T4CONbits.ON = 1;                                                             /* turn on Timer4 */
  OC1CONbits.ON = 1;                                                            /* turn on OC1 */
  OC2CONbits.ON = 1;                                                            /* turn on OC2 */
  OC3CONbits.ON = 1;                                                            /* turn on OC3 */
}

/* Given the Hall sensor state, use the mapping between sensor readings and
   energized phases given in the Pittman ELCOM motor data sheet to determine the
   correct phases to PWM and GND and the phase to leave floating. */
/*-----------------------------------------------------------------------------
 *      bldc_commutate():  brushless dc motor control proceedure
 *
 *  Parameters: int16_t pwm, uint16_t state
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void bldc_commutate(int16_t pwm, uint16_t state)
{
  pwm = ((int16_t)T4CON * pwm)/100;                                             /* convert pwm to ticks   */
  switch(state)
  {
     case 0b100:
     phases_set(pwm,PHASE_B, PHASE_A);                                          /* if pwm > 0, phase A = GND and B is PWMed */
     break;                                                                     /* if pwm < 0, phase B = GND and A is PWMed */

     case 0b110:
     phases_set(pwm,PHASE_C, PHASE_A);
     break;

     case 0b010:
     phases_set(pwm, PHASE_C, PHASE_B);
     break;

     case 0b011:
     phases_set(pwm, PHASE_A, PHASE_B);
     break;

     case 0b001:
     phases_set(pwm,PHASE_A,PHASE_C);
     break;

     case 0b101:
     phases_set(pwm,PHASE_B, PHASE_C);
     break;

     default:
     break;
  }
}

/*-----------------------------------------------------------------------------
 *      rotateFutaba():  make a loop of 90 degree turns with futaba s3003 servo
 *
 *
 *  Parameters: futaba_s3003_obj_t *futaba, uint16_t channel, uint32_t freqHz
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void rotateFutaba( futaba_s3003_obj_t *futaba, uint16_t channel, uint32_t freqHz, pwm_preScale_e timer_prescale, pwm_timer_e use_timer_x )
{

   futaba->pwm_period=PWM_Init(freqHz , 1u, timer_prescale, use_timer_x);
   PWMStart(channel);

   switch(futaba->rotStat)
   {
      case 0u:
      futaba->pwmVal=Scale_Float_Value(0.0f,255.0f,0u,futaba->pwm_period,65.0f);  /* scale pid output in range of pulse width modulation */
      if (futaba->pwmVal > futaba->pwm_period)                                  /* if we increase current_duty greater then possible pwm_period1 value */
      {
         futaba->pwmVal = futaba->pwm_period;                                   /* reset current_duty value to zero */
      }
      if (futaba->pwm_period != 0xFFFFU)                                        /* period returned was not invalid */
      {
         PWM_Set_Duty(futaba->pwmVal , channel);                                /* set newly acquired duty ratio */
         futaba->plsTimeDuration= -1;
         calculateTime2Now( &futaba->plsTimeDuration, &futaba->plsTimeDurationLast );
         futaba->rotStat=1u;
      }

      case 1u:
      calculateTime2Now( &futaba->plsTimeDuration, &futaba->plsTimeDurationLast );
      if (futaba->plsTimeDuration >= futaba->waitTime)
      {
         futaba->rotStat=2u;
      }
      break;

      case 2u:
      futaba->pwmVal =Scale_Float_Value(0.0f,255.0f,0u,futaba->pwm_period,155.0f);   /* scale pid output in range of pulse width modulation */
      if (futaba->pwmVal  > futaba->pwm_period)                                 /* if we increase current_duty greater then possible pwm_period1 value */
      {
         futaba->pwmVal  = futaba->pwm_period;                                  /* reset current_duty value to zero */
      }
      if (futaba->pwm_period != 0xFFFFU)                                        /* period returned was not invalid */
      {
         PWM_Set_Duty(futaba->pwmVal , channel);                                /* set newly acquired duty ratio */
         futaba->plsTimeDuration= -1;
         calculateTime2Now( &futaba->plsTimeDuration, &futaba->plsTimeDurationLast );
         futaba->rotStat=3u;
      }

      case 3u:
      calculateTime2Now( &futaba->plsTimeDuration, &futaba->plsTimeDurationLast );
      if (futaba->plsTimeDuration >= futaba->waitTime)
      {
         futaba->rotStat=4u;
      }
      break;

      case 4u:
      futaba->pwmVal =Scale_Float_Value(0.0f,255.0f,0u,futaba->pwm_period,250.0f);                 /* scale pid output in range of pulse width modulation */
      if (futaba->pwmVal  > futaba->pwm_period)                                 /* if we increase current_duty greater then possible pwm_period1 value */
      {
         futaba->pwmVal  = futaba->pwm_period;                                  /* reset current_duty value to zero */
      }
      if (futaba->pwm_period != 0xFFFFU)                                        /* period returned was not invalid */
      {
         PWM_Set_Duty(futaba->pwmVal , channel);                                /* set newly acquired duty ratio */
         futaba->plsTimeDuration= -1;
         calculateTime2Now( &futaba->plsTimeDuration, &futaba->plsTimeDurationLast );
         futaba->rotStat=5u;
      }

      case 5u:
      calculateTime2Now( &futaba->plsTimeDuration, &futaba->plsTimeDurationLast );
      if (futaba->plsTimeDuration >= futaba->waitTime)
      {
         futaba->rotStat=0u;                                                    /* go back to start */
      }
      break;

      default:
      futaba->rotStat=0u;                                                       /* go back to start */
      break;
   }
}
#endif /* end futaba 303 */