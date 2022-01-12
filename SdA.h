#ifndef __SdA_H
#define __SdA_H

#ifdef __cplusplus
extern "C" {
#endif

/*
    Machine Learning and AI section
    Logistic Regression 

    Version : @(#) 1.0
    original from : Yusuke-Sugomori
    Copyright (C) 2020 Port by A C P Avaiation Walkerburn Scotland
*/
#include "definitions.h"
#include <stdint.h>
#include "LogisticRegression.h"

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define SDAREG( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define SDAREG __attribute__((packed))                                        /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define SDAREG __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define SDAREG
#else                                                                           /* for MPLAB PIC32 */
  #define SDAREG( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#if defined(D_FT900)
typedef struct SDAREG {
  int16_t N;
  int16_t n_in;
  int16_t n_out;
  float64_t **W;
  float64_t *b;
} HiddenLayer_t;
#else
SDAREG(                                                                       
typedef struct {
  int16_t N;
  int16_t n_in;
  int16_t n_out;
  float64_t **W;
  float64_t *b;
}) HiddenLayer_t;                                                               
#endif

#if defined(D_FT900)
typedef struct SDAREG {
  int16_t N;
  int16_t n_visible;
  int16_t n_hidden;
  float64_t **W;
  float64_t *hbias;
  float64_t *vbias;
} dA_t;
#else
SDAREG(                                                                       
typedef struct {
  int16_t N;
  int16_t n_visible;
  int16_t n_hidden;
  float64_t **W;
  float64_t *hbias;
  float64_t *vbias;
}) dA_t;                                                               
#endif

#if defined(D_FT900)
typedef struct SDAREG {
  int16_t N;
  int16_t n_ins;
  int16_t *hidden_layer_sizes;
  int16_t n_outs;
  int16_t n_layers;
  HiddenLayer_t *sigmoid_layers;
  dA_t *dA_layers;
  LogisticRegression_t log_layer;
} SdA_t;
#else
SDAREG(                                                                       
typedef struct {
  int16_t N;
  int16_t n_ins;
  int16_t *hidden_layer_sizes;
  int16_t n_outs;
  int16_t n_layers;
  HiddenLayer_t *sigmoid_layers;
  dA_t *dA_layers;
  LogisticRegression_t log_layer;
}) SdA_t;                                                               
#endif

/*------------------------------------------------------------------------------
 *                          Function Prototypes.
 *----------------------------------------------------------------------------*/
extern float64_t sDa_uniform(float64_t min, float64_t max);
extern float64_t sDa2_uniform();
extern int16_t sDa_binomial(int16_t n, float64_t p);
extern float64_t sDa_sigmoid(float64_t x);
extern float64_t sDa_tanh_derivative(float64_t val);                            /* other general AI and ML functions */
extern float64_t sDa_relu_derivative(float64_t val);
extern float64_t sDa_relu(float64_t val);
extern float64_t sDa_sigmoid_derivative(float64_t val);
extern float64_t sDa_normalized_random( uint8_t randomSeeded );
extern float64_t sDa_gaussian_random(float64_t mean, float64_t stddev, uint8_t randomSeeded);
extern float64_t sDa_max_matrix(const float64_t *z);
extern float64_t* sDa_relu_matrix(const float64_t *z, float64_t value, uint8_t prime);
extern float64_t* sDa_softmax_matrix(const float64_t* z, const int16_t dim);
extern float64_t* sDa_transpose_matrix(float64_t *m);
extern float64_t* sDa_dot_matrix(const float64_t* m1, const float64_t* m2);
extern float64_t* sDa_relu_matrix1(float64_t *val);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* -- end library */