#ifndef __ENG_quadrot_
#define __ENG_quadrot_
#ifdef __cplusplus
extern "C" {
#endif

#include "definitions.h"
#include <stdint.h>
#include "Struts.h"
#include "gc_events.h"

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define QUADENGPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define QUADENGPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define QUADENGPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define QUADENGPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define QUADENGPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define ENG_thrustToDrag 0.016f                                                 /* thrust to drag constant [m] */
#define ENG_length 0.17f                                                        /* Center to rotor length [m] */
#define ENG_noOfMtr 4u                                                          /* number of motors */
#define ENG_Mass 0.468f                                                         /* # Mass [kg] */
#define ENG_intetia_X 0.0023f                                                   /* inertia in x direction [kg m^2] */
#define ENG_intetia_Y 0.0023f                                                   /* inertia in y direction [kg m^2] */
#define ENG_intetia_Z 0.0046f                                                   /* inertia in z direction [kg m^2] */

#if defined(D_FT900)
typedef struct QUADENGPACKED {
  float32_t mr;
  float32_t mp;
  float32_t mq;
} moments_obj_t;
#else
QUADENGPACKED(
typedef struct {
  float32_t mr;
  float32_t mp;
  float32_t mq;
}) moments_obj_t;                                                               /* made redundant used vector mapped x=mr, y=mp, z=mq */
#endif

#if defined(D_FT900)
typedef struct QUADENGPACKED {
  float32_t thrust[ENG_noOfMtr];
} thrust_obj_t;
#else
QUADENGPACKED(
typedef struct {
  float32_t thrust[ENG_noOfMtr];
}) thrust_obj_t;
#endif

#if defined(D_FT900)
typedef struct QUADENGPACKED {
        Vectr moments;
        Vectr acc;
        Vectr omega_dot;
        Vectr euler_dot;
        thrust_obj_t thrusts;
        Vectr velocity;
} integrator_obj_t;
#else
QUADENGPACKED(
typedef struct {
        Vectr moments;
        Vectr acc;
        Vectr omega_dot;
        Vectr euler_dot;
        thrust_obj_t thrusts;
        Vectr velocity;
}) integrator_obj_t;
#endif

/*------------------------------------------------------------------------------
 *                          Function Prototypes.
 *----------------------------------------------------------------------------*/
extern thrust_obj_t ENG_motor_thrust(const Vectr moments, float32_t coll_thrust);
extern mat33 ENG_angular_rotation_matrix(const Vectr euler_angles);
extern mat33 ENG_rotation_matrix(const Vectr euler_angles);
extern mat33 mat33_make_inertia_matrix( const Vectr inertia );
extern Vectr dot_mat33_vectr( const mat33 matr, const Vectr vec );
extern dVectr dot_Matrix3f64_vectr( const Matrix3f64_t matr, const dVectr vec );
extern Vectr ENG_moments(const Vectr desired_acc,const Vectr angular_vel);
extern Vectr ENG_dt_eulerangles_to_angular_velocity(const Vectr dtEuler, const Vectr euler_angles);
extern Vectr ENG_angular_velocity_to_dt_eulerangles(const Vectr omega, const Vectr euler_angles);
extern float32_t sumThrust( const thrust_obj_t thrusts );
extern Vectr ENG_acceleration(thrust_obj_t thrusts, const Vectr euler_angles);
#if (ENG_noOfMtr == 4)
extern Vectr ENG_angular_acceleration(const Vectr omega, const thrust_obj_t thr);
extern void ENG_integrator(integrator_obj_t *out, const Vectr euler, const Vectr omega, const float32_t coll_thrust, const Vectr desired_angular_acc);
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif