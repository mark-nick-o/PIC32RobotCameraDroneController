#include "definitions.h"
#if defined(ML_AI_NEEDED)                                                       /* ==== machine learning is required */
#if defined(LOGISTIC_REGRESSION_USED)                                           /* ==== logistic regression method ========== */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "LogisticRegression.h"

/*------------------------------------------------------------------------------
 *                          Function Prototypes.
 *----------------------------------------------------------------------------*/
void LogisticRegression__construct(LogisticRegression_t *this1, int16_t N, int16_t n_in, int16_t n_out);
void LogisticRegression__destruct(LogisticRegression_t *this1);
void LogisticRegression_softmax(LogisticRegression_t *this1, float64_t *x);
void LogisticRegression_train(LogisticRegression_t *this1, int16_t *x, int16_t *y, float64_t lr);
void LogisticRegression_predict(LogisticRegression_t *this1, int16_t *x, float64_t *y);

/*-----------------------------------------------------------------------------
 *      LogisticRegression__construct():  constructor to allocate memory for the  
 *                                       Logistic Regression
 *
 *  Parameters: LogisticRegression_t *this1, int16_t N, int16_t n_in, int16_t n_out     
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void LogisticRegression__construct(LogisticRegression_t *this1, int16_t N, int16_t n_in, int16_t n_out) 
{
  int16_t i, j;
  this1->N = N;
  this1->n_in = n_in;
  this1->n_out = n_out;

  this1->W = (float64_t **)malloc(sizeof(float64_t*) * n_out);
  this1->W[0] = (float64_t *)malloc(sizeof(float64_t) * n_in * n_out);
  for(i=0; i<n_out; i++) this1->W[i] = this1->W[0] + i * n_in;
  this1->b = (float64_t *)malloc(sizeof(float64_t) * n_out);

  for(i=0; i<n_out; i++) {
    for(j=0; j<n_in; j++) {
      this1->W[i][j] = 0;
    }
    this1->b[i] = 0;
  }
}
/*-----------------------------------------------------------------------------
 *      LogisticRegression__destruct():  destructor to free memory for the  
 *                                       Logistic Regression
 *
 *  Parameters: LogisticRegression_t *this1     
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void LogisticRegression__destruct(LogisticRegression_t *this1) 
{
  free((char*)this1->W[0],sizeof(this1->W[0]));
  free((char*)this1->W,sizeof(this1->W));
  free((char*)this1->b,sizeof(this1->b));
}
/*-----------------------------------------------------------------------------
 *      LogisticRegression_softmax():  calculate softmax for the Logistic Regression 
 *
 *  Parameters: LogisticRegression_t *this1, float64_t *x      
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void LogisticRegression_softmax(LogisticRegression_t *this1, float64_t *x) 
{
  int16_t i;
  float64_t max = 0.0f;
  float64_t sum = 0.0f;

  for(i=0; i<this1->n_out; i++) if(max < x[i]) max = x[i];
  for(i=0; i<this1->n_out; i++) {
    x[i] = exp(x[i] - max);
    sum += x[i];
  }

  for(i=0; i<this1->n_out; i++) x[i] /= sum;
}
/*-----------------------------------------------------------------------------
 *      LogisticRegression_train():  Train the Logistic Regression 
 *
 *  Parameters: LogisticRegression_t *this1, int16_t *x, int16_t *y, float64_t lr      
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void LogisticRegression_train(LogisticRegression_t *this1, int16_t *x, int16_t *y, float64_t lr) 
{
  int16_t i,j;
  float64_t *p_y_given_x = (float64_t *)malloc(sizeof(float64_t) * this1->n_out);
  float64_t *dy = (float64_t *)malloc(sizeof(float64_t) * this1->n_out);

  for(i=0; i<this1->n_out; i++) {
    p_y_given_x[i] = 0.0f;
    for(j=0; j<this1->n_in; j++) {
      p_y_given_x[i] += this1->W[i][j] * x[j];
    }
    p_y_given_x[i] += this1->b[i];
  }
  LogisticRegression_softmax(this1, p_y_given_x);

  for(i=0; i<this1->n_out; i++) {
    dy[i] = y[i] - p_y_given_x[i];

    for(j=0; j<this1->n_in; j++) {
      this1->W[i][j] += lr * dy[i] * x[j] / this1->N;
    }

    this1->b[i] += lr * dy[i] / this1->N;
  }

  free((char*)p_y_given_x,sizeof(p_y_given_x));
  free((char*)dy,sizeof(dy));
}
/*-----------------------------------------------------------------------------
 *      LogisticRegression_predict():  Logistic Regression prediction
 *
 *  Parameters: LogisticRegression_t *this1, int16_t *x, float64_t *y      
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void LogisticRegression_predict(LogisticRegression_t *this1, int16_t *x, float64_t *y) 
{
  int16_t i,j;

  for(i=0; i<this1->n_out; i++) {
    y[i] = 0.0f;
    for(j=0; j<this1->n_in; j++) {
      y[i] += this1->W[i][j] * x[j];
    }
    y[i] += this1->b[i];
  }

  LogisticRegression_softmax(this1, y);
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
#endif /* requirements were logistic regression */