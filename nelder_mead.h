#include "definitions.h"
#if defined(ML_AI_NEEDED)                                                       /* ==== machine learning is required */
#if defined(NELDER_MEAD)                                                        /* ==== nelder ead method ========== */

#ifndef __nelder_mead_
#define __nelder_mead_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
   
#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define NELDERMPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define NELDERMPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define NELDERMPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define NELDERMPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define NELDERMPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define RHO 1.0f
#define CHI 2.0f
#define GAMMA 0.5f
#define SIGMA 0.5f
#define SQUARE(x) ((x) * (x))

#if defined(D_FT900)
typedef struct NELDERMPACKED {
  float64_t a;
  float64_t b;
  float64_t c;
} ackley_param_t;
#else
NELDERMPACKED(
typedef struct {
  float64_t a;
  float64_t b;
  float64_t c;
}) ackley_param_t;
#endif

// define a generic point containing a position (x) and a value (fx)
#if defined(D_FT900)
typedef struct NELDERMPACKED {
  float64_t *x;
  float64_t fx;
} nelder_point_t;
#else
NELDERMPACKED(
typedef struct {
  float64_t *x;
  float64_t fx;
}) nelder_point_t;
#endif

// define a simplex struct containing an array of n+1 points (p)
// each having dimension (n)
#if defined(D_FT900)
typedef struct NELDERMPACKED {
  nelder_point_t *p;
  int16_t n;
} simplex_t;
#else
NELDERMPACKED(
typedef struct {
  nelder_point_t *p;
  int16_t n;
}) simplex_t;
#endif

// define optimization settings
#if defined(D_FT900)
typedef struct NELDERMPACKED {
  float64_t tolx;
  float64_t tolf;
  int16_t max_iter;
  int16_t max_eval;
  int8_t verbose;
} optimset_t;
#else
NELDERMPACKED(
typedef struct {
  float64_t tolx;
  float64_t tolf;
  int16_t max_iter;
  int16_t max_eval;
  int8_t verbose;
}) optimset_t;
#endif

extern void initNelderMead( optimset_t *optimset, ackley_param_t *ackley_params );
extern void ackley_fun(int16_t n, nelder_point_t *point, const void *arg);
extern void copy_point(int16_t n, const nelder_point_t *src, nelder_point_t *dst);
extern void swap_points(int16_t n, nelder_point_t *p1, nelder_point_t *p2);
extern void update_point(const simplex_t *simplex, const nelder_point_t *centroid, float64_t lambda, nelder_point_t *point);
extern int16_t continue_minimization(const simplex_t *simplex, int16_t eval_count, int16_t iter_count, const optimset_t *optimset);
extern void get_centroid(const simplex_t *simplex, nelder_point_t *centroid);
extern void nelder_mead_with_ackley(int16_t n, const nelder_point_t *start, nelder_point_t *solution, const void *args, const optimset_t *optimset);
extern void doNelderMead(optimset_t *opt,ackley_param_t *ack, nelder_point_t *solution, nelder_point_t *start );
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end nelder mead */

#endif
#endif /* requirements were nelder mead */