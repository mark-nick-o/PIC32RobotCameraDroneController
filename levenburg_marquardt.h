/* Levenburg Marquardt : Gradient Desent    */
/* ported from Artem Synytsyn Kiev Ukraiane */
/* by ACP Aviation                          */
#ifndef __leven_marq_h_
#define __leven_marq_h_

#include <stdint.h>
#include "definitions.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define TOL 1e-30f                                                              /* smallest value allowed in cholesky_decomp() */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define LEVEN_MARPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define LEVEN_MARPACKED __attribute__((packed))                               /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define LEVEN_MARPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define LEVEN_MARPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define LEVEN_MARPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#if defined(D_FT900)
typedef struct LEVEN_MARPACKED {
  int16_t verbose;
  int16_t max_it;
  float64_t init_lambda;
  float64_t up_factor;
  float64_t down_factor;
  float64_t target_derr;
  int16_t final_it;
  float64_t final_err;
  float64_t final_derr;
} LMstat_t;
#else
LEVEN_MARPACKED(
typedef struct {
  int16_t verbose;
  int16_t max_it;
  float64_t init_lambda;
  float64_t up_factor;
  float64_t down_factor;
  float64_t target_derr;
  int16_t final_it;
  float64_t final_err;
  float64_t final_derr;
}) LMstat_t;
#endif

#if defined(D_FT900)
typedef struct LEVEN_MARPACKED {
  int16_t n_iterations;
  float64_t T_heater;
  float64_t T_0;
  float64_t k;
  float64_t TT;
} LMResults_t;
#else
LEVEN_MARPACKED(
typedef struct {
  int16_t n_iterations;
  float64_t T_heater;
  float64_t T_0;
  float64_t k;
  float64_t TT;
}) LMResults_t;
#endif

#define LM_N_MEASUREMENTS 395u
#define LM_N_PARAMS 3u

float64_t t_data[LM_N_MEASUREMENTS] = { 21.6,  22. ,  22.4,  22.4,  22.5,  22.4,  22.2,  22.3,  22.4,\
                                  22.1,  21.8,  21.6,  22.1,  22. ,  21.7,  21.7,  21.5,  21.5,\
                                  21.4,  21.6,  21.7,  21.8,  21.9,  22. ,  21.9,  22.2,  22.3,\
                                  22.3,  22.4,  22.4,  22.4,  23. ,  23.6,  24.6,  24.3,  24.3,\
                                  24.2,  25. ,  25. ,  25.2,  25.4,  26. ,  26.3,  26.3,  26. ,\
                                  26.1,  25.9,  25.7,  26.1,  26. ,  26.3,  26.4,  26.4,  26.4,\
                                  26.5,  26.8,  26.8,  26.9,  27.2,  27.8,  27.9,  28.2,  28.1,\
                                  28.1,  28. ,  28. ,  28.3,  28.4,  28.8,  29.7,  30. ,  30.1,\
                                  29.8,  30.4,  30.9,  30.6,  30.7,  30.6,  30.3,  30.4,  30.8,\
                                  31.3,  31.3,  31.4,  31.2,  31.4,  31.5,  31.6,  31.8,  32.3,\
                                  32.2,  32.7,  32.4,  32.3,  32.7,  33. ,  33. ,  33.1,  33.2,\
                                  33.3,  33.6,  34. ,  33.9,  34.1,  34.2,  34.4,  34.7,  35.1,\
                                  35.2,  34.9,  35.2,  35.3,  35.1,  35.3,  36. ,  35.9,  35.9,\
                                  36.4,  36.9,  36.5,  36.4,  36.9,  36.9,  36.9,  37.4,  37. ,\
                                  36.9,  36.4,  37. ,  37.1,  37. ,  37.1,  37.1,  37.6,  37.8,\
                                  38. ,  38.7,  39. ,  39.2,  39.5,  40.1,  40.4,  40.6,  40.2,\
                                  40.1,  40.3,  40.4,  40.7,  40.9,  40.5,  40.7,  41.5,  41.4,\
                                  41.3,  40.7,  40.5,  40.8,  41.2,  41.2,  41. ,  41.3,  41.5,\
                                  41.7,  42.1,  42.1,  42.3,  42.2,  41.8,  42. ,  42.4,  42.6,\
                                  42.6,  42.3,  42.4,  42.7,  43.1,  42.9,  42.8,  42.8,  43.1,\
                                  43.4,  44. ,  43.9,  43.9,  43.8,  43.9,  44.4,  44.6,  45.1,\
                                  45.2,  45.2,  45.2,  45.4,  45.7,  45.3,  45.1,  45.1,  45.6,\
                                  45.9,  45.9,  45.9,  46. ,  46.2,  46.2,  46.3,  46.6,  46.9,\
                                  46.9,  47.2,  47.6,  47.5,  47.7,  47.6,  47.7,  48. ,  47.8,\
                                  47.9,  48. ,  48.2,  48. ,  48. ,  48.1,  48.6,  48.6,  49.2,\
                                  48.9,  48.8,  49.2,  49.4,  49.2,  49. ,  49.3,  49.8,  50.3,\
                                  50.7,  50.9,  50.8,  50.6,  50.4,  50.7,  50.9,  51. ,  51.4,\
                                  51.4,  51.8,  51.8,  51.6,  52.3,  52.7,  53.3,  53.1,  54. ,\
                                  53.3,  53.3,  53.6,  53.2,  53. ,  53.1,  53.4,  53.5,  53.6,\
                                  53.9,  53.7,  53.9,  53.9,  52.8,  53.1,  53.1,  53.3,  53.4,\
                                  53.6,  54. ,  54.3,  54.2,  54.4,  54.7,  54.6,  56.5,  56.4,\
                                  55.7,  55.8,  55.9,  56.2,  56.2,  56.3,  56.3,  56.5,  56.9,\
                                  57. ,  57.4,  57.9,  58.2,  57.6,  57.5,  57.9,  58.4,  58.9,\
                                  59.1,  58.4,  59. ,  62.4,  57.6,  56.6,  58.6,  59. ,  59. ,\
                                  59. ,  59.3,  59.5,  59. ,  59.4,  59.2,  59.1,  60. ,  59.8,\
                                  60.3,  60.8,  60.5,  60.4,  61.5,  60.7,  60.7,  61. ,  60.9,\
                                  61. ,  61.3,  61.7,  61.5,  61.4,  61.8,  61.9,  62.6,  62.3,\
                                  62.1,  62. ,  62.3,  62.4,  62.4,  61.7,  62.7,  63.2,  62.8,\
                                  62.9,  63. ,  63.6,  63.9,  63.3,  63.7,  63.5,  63.3,  63.9,\
                                  64. ,  63.8,  63.9,  64.5,  64.3,  63.8,  63.5,  64. ,  64.3,\
                                  65.2,  65.7,  65.9,  65.6,  64.7,  65. ,  65.1,  65.8,  66.2,\
                                  66.6,  66.3,  66. ,  65.5,  66.2,  66.4,  66.8,  67.4,  67.3,\
                                  67. ,  66.8,  66.4,  67.3,  66.9,  67.6,  72.1,  66.6,  66.2,\
                                  67.1,  70. ,  68.8,  68.1,  67.8,  67.3,  67.4,  66.2 };

extern float64_t LM_newton_func(float64_t *par, int16_t x, void *fdata);
extern void LM_gradient(float64_t *g, float64_t *par, int16_t x, void *fdata);
extern int16_t LM_temp_to_time(float64_t *par, float64_t temp);
extern void levmarq_init(LMstat_t *lmstat);
extern void solve_axb_cholesky(int16_t n, float64_t **l, float64_t *x, float64_t *b);
extern int16_t cholesky_decomp(int16_t n, float64_t **l, float64_t **a);
extern float64_t error_func(float64_t *par, int16_t ny, float64_t *y, float64_t *dysq,  float64_t (*func)(float64_t *, int16_t, void *), void *fdata);
extern int16_t levmarq( float64_t *par, float64_t *y, float64_t *dysq, float64_t (*func)(float64_t *, int16_t, void *), void (*grad)(float64_t *, float64_t *, int16_t, void *), void *fdata, LMstat_t *lmstat);
extern void doLevenburgMarquardtNLH( LMResults_t *res, void *fdata, LMstat_t *lmstat, float64_t *par, float64_t *y, float64_t *dysq, float64_t *t_data );
#ifdef __cplusplus
}
#endif

#endif