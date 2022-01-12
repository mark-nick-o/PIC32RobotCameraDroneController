#if defined(ML_AI_NEEDED)                                                       /* ==== machine learning is required */
#if defined(NAIVE_BAYES_USED)                                                   /* ==== naive bayes method ========== */

#ifndef __naive_bayes_algo_
#define __naive_bayes_algo_

#ifdef __cplusplus
extern "C" {
#endif

#include "definitions.h"
#include <stdint.h>

/*------------------------------------------------------------------------------
 *                          Function Prototypes.
 *----------------------------------------------------------------------------*/
extern uint16_t NB_CountClass(uint16_t *pClassVec, uint16_t pClassVecLen);
extern void NB_CalculateAveVar(float64_t **pFeatureVec,float64_t ** pParaTable,uint16_t *pClassVec,uint16_t pRow,uint16_t i,uint16_t j);
extern float64_t NB_rand_gauss(void);
extern void NB_AddGaussianWhiteNoise(float64_t **pFeatureVec,uint16_t i,uint16_t pRow);
extern float64_t** NB_TrainModel(float64_t **pFeatureVec,uint16_t pRow,uint16_t pCol,uint16_t *pClassVec,uint16_t ClassNum);
extern void NB_create_2D_array(float64_t **arr, uint16_t len, uint16_t width);
extern void NB_destroy_2D_array(float64_t **arr, uint16_t len, uint16_t width);
extern void NB_FreeTable(float64_t **pParaTable,uint16_t pRow,uint16_t pCol);
extern float64_t NB_mean_of_datastream( float64_t *dStream );
extern float64_t NB_sd_of_datastream( float64_t *dStream, float64_t mean );
extern float64_t POIS_U_Random();
extern int16_t POIS_poisson( int16_t Lambda );
extern float64_t bayes_calc_prob_like( float64_t value, float64_t mean, float64_t stdev );
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

#endif
#endif /* requirements were naive bayes */