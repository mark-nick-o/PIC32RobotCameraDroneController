#if defined(ML_AI_NEEDED)                                                       /* ==== machine learning is required */
#if defined(UNIVARIATE_SMOOTH)                                                  /* ==== univariate smooth method ========== */
#include "definitions.h"
#ifndef __UV_SMOOTH_H_
#define __UV_SMOOTH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    US_1 = 0,
    US_2,
    US_3,
    US_4,
    US_5
} US_Cost_func_e;                                                               /* defines the available cost functions */

typedef enum
{
    BISECTION = 0,
    NEWTON,
    SECANT,
    RFM,
    COMPLETE
} US_method_e;                                                                  /* defines the available methods */

extern float64_t US_func1(float64_t x, uint8_t *valid);
extern float64_t US_func2(float64_t x, uint8_t *valid);
extern float64_t US_func3(float64_t x, uint8_t *valid);
extern float64_t US_func4(float64_t x, uint8_t *valid);
extern float64_t US_func5(float64_t x, uint8_t *valid);
extern float64_t US_Diff(US_Cost_func_e cost_func, float64_t x);
extern void US_RFM(US_Cost_func_e cost_func, float64_t *a, float64_t *b, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid);
extern void US_Secant(US_Cost_func_e cost_func, float64_t *x1, float64_t *x0, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid);
extern void US_Newton(US_Cost_func_e cost_func, float64_t *x, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid);
extern void US_doUnivariateSmooth(US_method_e meth, US_Cost_func_e cost_func, float64_t *a, float64_t *b, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

#endif
#endif /* requirements were univariate smooth */