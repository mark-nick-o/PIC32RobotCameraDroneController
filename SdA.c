/* ----------------------------------------------------------------------------
   sDa Library contains general machine learning and artificial intelligence 
   algorythms
   ---------------------------------------------------------------------------- 
*/
#ifdef __cplusplus
extern "C" {
#endif

#include "definitions.h"
#include <stdint.h>
#include "LogisticRegression.h"
#include "SdA.h"

/*------------------------------------------------------------------------------
 *                          Function Prototypes.
 *----------------------------------------------------------------------------*/
/* ================== single value functions ================================ */
float64_t sDa_uniform(float64_t min, float64_t max);
float64_t sDa2_uniform();
int16_t sDa_binomial(int16_t n, float64_t p);
float64_t sDa_sigmoid(float64_t x);
float64_t sDa_tanh_derivative(float64_t val);                                   /* other general AI and ML functions */
float64_t sDa_relu_derivative(float64_t val);
float64_t sDa_relu(float64_t val);
float64_t sDa_sigmoid_derivative(float64_t val);
float64_t sDa_normalized_random( uint8_t randomSeeded );
float64_t sDa_gaussian_random(float64_t mean, float64_t stddev, uint8_t randomSeeded);
/* ================== matrix functions ================================ */
float64_t sDa_max_matrix(const float64_t *z);
float64_t* sDa_relu_matrix(const float64_t *z, float64_t value, uint8_t prime);
float64_t* sDa_softmax_matrix(const float64_t* z, const int16_t dim);
float64_t* sDa_transpose_matrix(float64_t *m);
float64_t* sDa_dot_matrix(const float64_t* m1, const float64_t* m2);
float64_t* sDa_relu_matrix1(float64_t *val);
/*-----------------------------------------------------------------------------
 *      sDa_uniform():    
 *
 *  Parameters: float64_t min, float64_t max     
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t sDa_uniform(float64_t min, float64_t max) 
{
  return rand() / (INT16_MAX + 1.0f) * (max - min) + min;  
}
/*-----------------------------------------------------------------------------
 *      sDa2_uniform():    
 *
 *  Parameters:     
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t sDa2_uniform()
{
    return rand() / ((float64_t)INT16_MAX + 1.0f);
}
/*-----------------------------------------------------------------------------
 *      sDa_binomial():    
 *
 *  Parameters: int16_t n, float64_t p    
 *
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
int16_t sDa_binomial(int16_t n, float64_t p) 
{
  int16_t i;
  int16_t c = 0;
  float64_t r;
  
  if(p < 0 || p > 1) return 0;

  for(i=0; i<n; i++) {
    r = rand() / (INT16_MAX + 1.0f);
    if (r < p) c++;
  }

  return c;
}

/*-----------------------------------------------------------------------------
 *      sDa_sigmoid():    
 *
 *  Parameters: float64_t x   
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t sDa_sigmoid(float64_t x) 
{
  return 1.0f / (1.0f + exp(-x));
}

/*
 Copyright (c) 2016 Fabio Nicotra.
 All rights reserved.
 General other ML factorisation 
 Ported by ACP Aviation
 
 Redistribution and use in source and binary forms are permitted
 provided that the above copyright notice and this paragraph are
 duplicated in all such forms and that any documentation,
 advertising materials, and other materials related to such
 distribution and use acknowledge that the software was developed
 by the copyright holder. The name of the
 copyright holder may not be used to endorse or promote products derived
 from this software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
/*-----------------------------------------------------------------------------
 *      sDa_sigmoid_derivative():   sigmoid derivative calc 
 *
 *  Parameters: float64_t val   
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/ 
float64_t sDa_sigmoid_derivative(float64_t val) 
{
    return val * (1.0f - val);
}
/*-----------------------------------------------------------------------------
 *      sDa_relu():   relu function 
 *
 *  Parameters: float64_t val   
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/ 
float64_t sDa_relu(float64_t val) 
{
    return (val >= 0.0f ? val : 0.0f);
}
/*-----------------------------------------------------------------------------
 *      sDa_relu_derivative():   relu derivative 
 *
 *  Parameters: float64_t val   
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t sDa_relu_derivative(float64_t val) 
{
    return (float64_t)(val > 0.0f);
}
/*-----------------------------------------------------------------------------
 *      sDa_tanh_derivative():   tanh derivative 
 *
 *  Parameters: float64_t val   
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t sDa_tanh_derivative(float64_t val) 
{
    return (1.0f - (val * val));
}
/*-----------------------------------------------------------------------------
 *      sDa_normalized_random():   normalised random number 
 *
 *  Parameters: uint8_t randomSeeded  
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t sDa_normalized_random( uint8_t randomSeeded ) 
{
    int16_t r;
    if (!randomSeeded) 
    {
        randomSeeded = 1u;
        srand((uint16_t)(CP0_GET(CP0_COUNT) & UINT16_MAX));
    }
    r = rand();
    return ((float64_t) r / (float64_t) UINT16_MAX);
}
/*-----------------------------------------------------------------------------
 *      sDa_gaussian_random():   gaussian random number 
 *
 *  Parameters: float64_t mean, float64_t stddev, uint8_t randomSeeded  
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t sDa_gaussian_random(float64_t mean, float64_t stddev, uint8_t randomSeeded) 
{
    float64_t theta = 2.0f * PI * sDa_normalized_random(randomSeeded);
    float64_t rho = sqrt(-2.0f * log(1.0f - sDa_normalized_random(randomSeeded)));
    float64_t scale = stddev * rho;
    float64_t x = mean + scale * cos(theta);
    float64_t y = mean + scale * sin(theta);
    float64_t r = sDa_normalized_random(randomSeeded);
    return (r > 0.5f ? y : x);
}

/*  Returns the product of two matrices: m1 x m2.
     Inputs:
     m1: vector, left matrix of size m1_rows x m1_columns
     m2: vector, right matrix of size m1_columns x m2_columns (the number of rows in the right matrix
     must be equal to the number of the columns in the left one)
     m1_rows: int, number of rows in the left matrix m1
     m1_columns: int, number of columns in the left matrix m1
     m2_columns: int, number of columns in the right matrix m2
     Output: vector, m1 * m2, product of two vectors m1 and m2, a matrix of size m1_rows x m2_columns
*/
/* define the size of the matrix we are using */
#define sDa_ROWS 3u
#define sDa_1COLUMNS 3u
#define sDa_2COLUMNS 3u
float64_t* sDa_dot_matrix(const float64_t* m1, const float64_t* m2) 
{  
    float64_t output[sDa_ROWS*sDa_1COLUMNS];
    int16_t row,col,k;
        
    for( row = 0; row != sDa_ROWS; ++row ) 
    {
        for( col = 0; col != sDa_2COLUMNS; ++col ) 
        {
            output[ row * sDa_2COLUMNS + col ] = 0.f;
            for( k = 0; k != sDa_1COLUMNS; ++k ) 
            {
                output[ row * sDa_2COLUMNS + col ] += m1[ row * sDa_1COLUMNS + k ] * m2[ k * sDa_2COLUMNS + col ];
            }
        }
    }   
    return &output;
}
/*  Returns a transpose matrix of input matrix.
     Inputs:
     m: vector, input matrix
     Output: vector, transpose matrix mT of input matrix m
*/
float64_t* sDa_transpose_matrix(float64_t *m) 
{
      
    float64_t  mT[sDa_1COLUMNS*sDa_ROWS];
    uint8_t n,i,j;
        
    for(n = 0; n != sDa_1COLUMNS*sDa_ROWS; n++) 
    {
        i = n/sDa_1COLUMNS;
        j = n%sDa_1COLUMNS;
        mT[n] = m[sDa_ROWS*j + i];
    }
    return &mT;
}
/*-----------------------------------------------------------------------------
 *   sDa_relu_matrix1(): perform relu on matrix defined by constants
 *
 *  Parameters: float64_t *val
 *
 *  Return: float64_t *
 *----------------------------------------------------------------------------*/
float64_t* sDa_relu_matrix1(float64_t *val) 
{
    float64_t output[sDa_ROWS*sDa_1COLUMNS];
    size_t i;
    
    for( i = 0; i < sDa_ROWS*sDa_1COLUMNS; ++i ) 
    {
       output[i] = (val[i] >= 0.0f ? val[i] : 0.0f);
    }
    return &output;
}
/*-----------------------------------------------------------------------------
 *   sDa_relu_matrix(): perform relu on matrix defined by constants
 *
 *  Parameters: const float64_t *z, float64_t value, uint8_t prime
 *
 *  Return: float64_t *
 *----------------------------------------------------------------------------*/
float64_t* sDa_relu_matrix(const float64_t *z, float64_t value, uint8_t prime)
{
    float64_t output[sDa_ROWS*sDa_1COLUMNS];
    size_t i;
        
    for( i = 0; i < sDa_ROWS*sDa_1COLUMNS; ++i ) 
    {
        if (z[i] < value)
        {
            if (prime <= 0u)
            {        
               output[i] = value;
            }
            else
            {
              output[i] = 0.0f;
            }
        }
        else 
        {
            if (prime <= 0u)
            {                        
               output[i] = z[i];
            }
            else
            {
               output[i] = 1.0f;
            }
        }
    }
    return &output;
}
/*-----------------------------------------------------------------------------
 *   sDa_relu_matrix(): perform maximum selection on matrix defined by constants
 *
 *  Parameters: const float64_t *z
 *
 *  Return: float64_t 
 *----------------------------------------------------------------------------*/
float64_t sDa_max_matrix(const float64_t *z)
{
    float64_t maxFlo=0.0f;
    size_t i;
        
    for( i = 0; i < sDa_ROWS*sDa_1COLUMNS; ++i ) 
    {
        if (z[i] > maxFlo)
        {
           maxFlo = z[i];
        }
    }
    return maxFlo;
}
/*-----------------------------------------------------------------------------
 *   sDa_softmax_matrix(): perform softmax on matrix whose size is defined by constants
 *
 *  Parameters: const float64_t* z, const int16_t dim
 *
 *  Return: float64_t* 
 *----------------------------------------------------------------------------*/
float64_t* sDa_softmax_matrix(const float64_t* z, const int16_t dim) 
{
    
    float64_t out[sDa_ROWS*sDa_1COLUMNS];
    float64_t foo[sDa_ROWS*sDa_1COLUMNS];
    float64_t max_foo;
    uint16_t i,j;
    float64_t sum_of_elems = 0.0f;
                
    for (i = 0; i != sDa_ROWS*sDa_1COLUMNS; i += dim) 
    {
        for (j = 0; j != dim; ++j) 
        {
            foo[i] = z[i + j];
        }
        
        max_foo = sDa_max_matrix(z);

        for (j = 0; j != dim; ++j) 
        {
            foo[j] = exp(foo[j] - max_foo);
        }      


        for (j = 0; j != dim; ++j) 
        {
            sum_of_elems = sum_of_elems + foo[j];
        }
        
        for (j = 0; j != dim; ++j) 
        {
            out[j] = (foo[j]/sum_of_elems);
        }
    }
    return &out;
}
#ifdef __cplusplus
}
#endif /* __cplusplus */