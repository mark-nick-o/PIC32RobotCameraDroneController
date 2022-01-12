#ifndef Robo_h
#define Robo_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*    robot.h : Structures related to gait robotics functionality with odrive
//
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
//
//  This library is a series of helpers for robotics
//  It has been ported from the open source NATE711_DOGO
//
*/
#include "definitions.h"

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define ROBOLIBPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define ROBOLIBPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define ROBOLIBPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define ROBOLIBPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define ROBOLIBPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#if defined(D_FT900)
typedef struct ROBOLIBPACKED {
      float32_t stance_height;
      float32_t down_amp;
      float32_t up_amp;
      float32_t flight_percent;
      float32_t step_length;
      float32_t freq;
      float32_t step_diff;
} GaitParams;
#else
ROBOLIBPACKED(
typedef struct {
      float32_t stance_height;
      float32_t down_amp;
      float32_t up_amp;
      float32_t flight_percent;
      float32_t step_length;
      float32_t freq;
      float32_t step_diff;
}) GaitParams;
#endif

#if defined(D_FT900)
typedef struct ROBOLIBPACKED {
     float32_t kp_theta;
     float32_t kd_theta;
     float32_t kp_gamma;
     float32_t kd_gamma;
} LegGain;
#else
ROBOLIBPACKED(
typedef struct {
     float32_t kp_theta;
     float32_t kd_theta;
     float32_t kp_gamma;
     float32_t kd_gamma;
}) LegGain;
#endif

typedef enum {                                                                  // The States of the robot

    RobSTOP = 0u,
    RobTROT = 1u,
    RobBOUND = 2u,
    RobWALK = 3u,
    RobPRONK = 4u,
    RobJUMP = 5u,
    RobDANCE = 6u,
    RobHOP = 7u,
    RobTEST = 8u,
    RobROTATE = 9u,
    RobFLIP = 10u,
    RobTURN_TROT = 11u,
    RobRESET = 12u

} RobStates;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif                                                                          // end of robo lib