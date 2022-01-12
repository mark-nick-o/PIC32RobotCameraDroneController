#ifndef __cage_gen_evol_
#define __cage_gen_evol_
/* =============================================================================

CAGE is a very simple but powerful implementation of genetic algorithms in C++. 
In the standard (1+?) ES, each offspring is mutated from the population, whereas 
our offsprings are iteratively mutated, each one being the mutation ofthe previous. 
We apply restarts according to the Luby sequence.

It can be used for real-world applications, for example in this paper.

CAGE is particularly suited for discrete problems that numerical optimization 
methods (like scipy.optimize.minimize) cannot handle. 
If your cost function is very expansive and quite regular, you might want use 
smarter algorithms (e.g. optuna).

Ported from https://github.com/louisabraham/cage by ACP Aviation

============================================================================= */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <stdlib.h>
#include "definitions.h"

#ifndef bool
#define bool uint8_t
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define CAGEPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define CAGEPACKED __attribute__((packed)) ALIGNED(1)                        /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define CAGEPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define CAGEPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define CAGEPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define CAGE_GENEVO_DIM 2u
#define CAGE_GENEVO_N 1000u                                                     // float64_t X[CAGE_GENEVO_N][CAGE_GENEVO_DIM] float64_t y[CAGE_GENEVO_N]

#if defined(D_FT900)
typedef struct CAGEPACKED {
   float64_t pair[CAGE_GENEVO_DIM];
} cage_evo_elem_t;
#else
CAGEPACKED(
typedef struct  {
   float64_t pair[CAGE_GENEVO_DIM];
}) cage_evo_elem_t;                                                           
#endif

#if defined(D_FT900)
typedef struct CAGEPACKED {
   float64_t X[CAGE_GENEVO_N][CAGE_GENEVO_DIM];
   float64_t y[CAGE_GENEVO_N];   
} cage_evo_inputs_t;
#else
CAGEPACKED(
typedef struct  {
   float64_t X[CAGE_GENEVO_N][CAGE_GENEVO_DIM];
   float64_t y[CAGE_GENEVO_N];
}) cage_evo_inputs_t;                                                           
#endif

extern void cage_gen_evo_seed( uint16_t initSeed );
extern float64_t fRand( float64_t fMin, float64_t fMax );
extern cage_evo_elem_t cage_gen_evo_init();
extern void cage_gen_evo_fill_rand_XArray(float64_t **X);
extern void cage_gen_evo_init_YArray(float64_t *y, float64_t **X);
extern float64_t cage_gen_evo_eval( float64_t *coefs, float64_t *x );
extern float64_t cage_gen_evo_score( float64_t *coefs, float64_t **X, float64_t *y );
extern float64_t cage_gen_evo_mutate( float64_t *coefs );
extern uint64_t cage_gen_evo_luby( int16_t *x );
extern bool cage_gen_evo_restart( int16_t *cur_count, int16_t *cur_luby, int16_t *luby_count, int16_t luby_basis );
extern bool cage_gen_evo_maxi( float64_t a, float64_t b );
extern bool cage_gen_evo_mini( float64_t a, float64_t b );
extern void cage_gen_evo_rank( cage_evo_elem_t *e, float64_t **X, float64_t *y );
extern void cage_gen_evo_reset( cage_evo_elem_t *global_best, cage_evo_elem_t *best, float64_t **X, float64_t *y );
extern cage_evo_elem_t cage_evo_genetic_algorithm( float64_t **X, float64_t *y, bool minimize, int16_t pop_size, int16_t steps, int16_t luby_basis );
extern float64_t cage_gen_evo_rmse( float64_t **X, float64_t *y, bool minimize, int16_t pop_size, int16_t steps, int16_t luby_basis );
extern float64_t cage_gen_evo_mean( float64_t *y );
extern float64_t cage_gen_evo_std( float64_t *y, float64_t mean );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end cage generic evolution library */