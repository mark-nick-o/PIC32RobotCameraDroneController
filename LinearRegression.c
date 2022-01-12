#include "definitions.h"
#if defined(ML_AI_NEEDED)                                                       /* ==== machine learning is required */
#if defined(LINEAR_REGRESSION_USED)                                             /* ==== linear regression method ========== */

#ifdef __cplusplus
extern "C" {
#endif
/*
   Assembled by ACP Aviation contains extracts from
   Alexandru Rosianu : Linear Regression  https://www.behance.net/aluxian
   gregorydhill@outlook.com : Edinburgh University dept. Informatics
*/
#include <stdint.h>
#include "LinearRegression.h"

/*------------------------------------------------------------------------------
 *                          Function Prototypes.
 *----------------------------------------------------------------------------*/
float64_t** LR_make_matrix(int16_t n, int16_t m);
void LR_matrix_clear(int16_t n, float64_t** X);
float64_t** LR_transpose(int16_t n, int16_t m, float64_t** X);
float64_t** LR_product(int16_t n, int16_t m, int16_t p, int16_t q, float64_t** A, float64_t** B);
void LR_identity(int16_t n, int16_t m, float64_t** X);
float64_t** LR_get_minor(int16_t row, int16_t col, int16_t n, float64_t** M);
float64_t LR_determinant(int16_t n, float64_t** M);
float64_t** LR_inverse(int16_t n, float64_t** M);
float64_t LR_array_sum(float64_t arr[], int16_t len);
float64_t** LR_lstsq(int16_t n, int16_t m, float64_t** X, float64_t** y);
int16_t LR_Run_main(float64_t** w, float64_t** f); 
float64_t* LR_array_pow(float64_t arr[], int16_t len, int16_t power);
float64_t* LR_array_multiplication(float64_t arr1[], float64_t arr2[], int16_t len);
float64_t* LR_array_diff(float64_t arr1[], float64_t arr2[], int16_t len);
void LR_array_clear( float64_t *arr, int16_t len );
float64_t LinearRegression_h(float64_t x, float64_t theta[]);
float64_t *LinearRegression_calculate_predictions(float64_t x[], float64_t theta[], int16_t m);
float64_t LinearRegression_compute_cost(float64_t x[], float64_t y[], float64_t theta[], int16_t m);
float64_t *LinearRegression_gradient_descent(float64_t x[], float64_t y[], float64_t alpha, int16_t iters, float64_t *J, int16_t m);
float64_t doGradDescent(float64_t x[], float64_t y[], float64_t *J, float64_t x_est);

/*-----------------------------------------------------------------------------
 *      LR_make_matrix():   make matrix in memory
 *
 *  Parameters: int16_t n, int16_t m  
 *
 *  Return: float64_t**
 *----------------------------------------------------------------------------*/
float64_t** LR_make_matrix(int16_t n, int16_t m) 
{
  float64_t **X;
  int16_t i;
  X = Malloc(n * sizeof (*X));
  for (i=0; i<n; i++)
    X[i] = Malloc(m * sizeof(*X[i]));
  return X;
}
/*-----------------------------------------------------------------------------
 *      LR_matrix_clear():   destroy matrix from memory
 *
 *  Parameters: int16_t n, float64_t** X 
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void LR_matrix_clear(int16_t n, float64_t** X)
{
  int16_t i; 
  if (X == NULL) return;
  for (i=0; i<n; i++)
    Free((char*)X[i],sizeof(X[i]));
  Free((char*)X,sizeof(X));
}
/*-----------------------------------------------------------------------------
 *      LR_transpose():   transpose matrix 
 *
 *  Parameters: int16_t n, int16_t m, float64_t** X
 *
 *  Return: float64_t**
 *----------------------------------------------------------------------------*/
float64_t** LR_transpose(int16_t n, int16_t m, float64_t** X) 
{
  float64_t** X_ = LR_make_matrix(m,n);
  int16_t i;
  int16_t j;

  if (X == NULL)
  {
      X_[0u][0u] = NaN;
  }
  else
  {    
    for (i = 0; i < m; i++) 
    {
       for (j = 0; j < n; j++) 
       {
          X_[i][j] = X[j][i];
       }
     }
  }
  return X_;
}
/*-----------------------------------------------------------------------------
 *      LR_product():   matrix product
 *
 *  Parameters: int16_t n, int16_t m, int16_t p, int16_t q, float64_t** A, float64_t** B
 *
 *  Return: float64_t**
 *----------------------------------------------------------------------------*/
float64_t** LR_product(int16_t n, int16_t m, int16_t p, int16_t q, float64_t** A, float64_t** B) 
{
  float64_t** C = LR_make_matrix(n,q);
  int16_t i,j,k;

  if ((A == NULL)||(B == NULL))
  {
     C[0u][0u] = NaN;
  }
  else
  {   
     for (i = 0; i < n; i++)
       for (j = 0; j < q; j++)
         C[i][j] = 0;

     for (i = 0; i < n; i++) {
       for (j = 0; j < q; j++) {
         for (k = 0; k < p; k++) {
           C[i][j] += A[i][k]*B[k][j];
         }
       }
     }
  }
  return C;
}
/*-----------------------------------------------------------------------------
 *      LR_identity():   matrix identity
 *
 *  Parameters: int16_t n, int16_t m, float64_t** X
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void LR_identity(int16_t n, int16_t m, float64_t** X) 
{
  int16_t i,j;
  if (X == NULL) return;         
  for (i = 0; i < n; i++) {
    for (j = 0; j < m; j++) {
      if (i==j) X[i][j]=1;
      else X[i][j]=0;
    }
  }
}
/*-----------------------------------------------------------------------------
 *      LR_get_minor():   get minor
 *
 *  Parameters: int16_t row, int16_t col, int16_t n, float64_t** M
 *
 *  Return: float64_t**
 *----------------------------------------------------------------------------*/
float64_t** LR_get_minor(int16_t row, int16_t col, int16_t n, float64_t** M) 
{
  int16_t k = 0;
  int16_t l = 0;
  int16_t s = n-1;
  int16_t i,j;
  float64_t** m = LR_make_matrix(s,s);
  if (M == NULL) 
  { m[0u][0u] = NaN; } 
  else
  {  
     for (i = 0; i < n; i++) {
       for (j = 0; j < n; j++) {
         if (i!=row&&j!=col) {
           m[k][l] = M[i][j]; l++;
         }
       }
       if (i!=row) k++;
       l=0;
     }
  }
  return m;
}
/*-----------------------------------------------------------------------------
 *      LR_determinant():   do determinate (det)
 *
 *  Parameters: int16_t n, float64_t** M
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t LR_determinant(int16_t n, float64_t** M) 
{
  int16_t j;
  float64_t** m;
  float64_t det = 0.0f;

  if (M == NULL) return NaN;     
  if (n==2)
    return (M[0][0]*M[1][1])-(M[0][1]*M[1][0]);

  for (j = 0; j < n; j++) {
    m = LR_get_minor(0, j, n, M);
    det +=  pow((-1),j)*M[0][j]*LR_determinant(n-1, m);
    LR_matrix_clear(n, m);
  }
  return det;
}
/*-----------------------------------------------------------------------------
 *      LR_inverse():   inverse
 *
 *  Parameters: int16_t n, float64_t** M
 *
 *  Return: float64_t**
 *----------------------------------------------------------------------------*/
float64_t** LR_inverse(int16_t n, float64_t** M) 
{
  float64_t** C = LR_make_matrix(n, n);
  float64_t d = LR_determinant(n, M);
  int16_t i,j;
  float64_t** m;
  float64_t** A;

  if (M == NULL) { A[0u][0u] = NaN; }
  else
  {   
    if (n==2) {
      C[0][0] = M[1][1]/d;
      C[0][1] = (-1)*M[0][1]/d;
      C[1][0] = (-1)*M[1][0]/d;
      C[1][1] = M[0][0]/d;
      return C;
     }

    for (i = 0; i < n; i++) {
       for (j = 0; j < n; j++) {
         m = LR_get_minor(i, j, n, M);
         C[i][j] = (pow((-1),i+j)*LR_determinant(n-1, m))/d;
       }
    }

    A = LR_transpose(n, n, C);
    LR_matrix_clear(n, C);
  }
  return A;
}

/*
* Linear Least Squares Optimization - Normal Equations Approach
*
* @param n: number of input rows
* @param m: number of input columns
* @param X: input matrix
* @param y: output vector
*
* @return: linear weights
*/
float64_t** LR_lstsq(int16_t n, int16_t m, float64_t** X, float64_t** y) 
{
  float64_t** X_ = LR_transpose(n, m, X);                                       // TODO: include bias column
  float64_t** A = LR_product(m, n, n, m, X_, X);
  float64_t** A_;
  float64_t** B;
  float64_t** w;

  float64_t d = LR_determinant(m, A);                                           // non-square matrices do not have determinant
  if ((X == NULL)||(y == NULL))  { w[0u][0u] = NaN; }
  else
  {  
     if (d==0) {
       w[0u][0u] = -1;
       return w;                                                                 /* Matrix non-invertible. */
     }

     A_ = LR_inverse(m, A);
     B = LR_product(m, m, m, n, A_, X_);
     w = LR_product(m, n, n, 1, B, y);

     LR_matrix_clear(n, X_);
     LR_matrix_clear(n, A);
     LR_matrix_clear(n, A_);
     LR_matrix_clear(n, B);
  }
  return w;
}
/*-----------------------------------------------------------------------------
 *      LR_Run_main():   run the main regression
 *
 *  Parameters: float64_t** w, float64_t** f
 *
 *  Return: int16_t
 *----------------------------------------------------------------------------*/
int16_t LR_Run_main(float64_t** w, float64_t** f) 
{
  /* ===========  set this up for youre requirement ========== */
  int16_t n = 3;                                                                // rows 
  int16_t m = 2;                                                                // columns
  float64_t** X = LR_make_matrix(n,m);                                          // inputs
  float64_t** y = LR_make_matrix(n,1);                                          // outputs

  if ((w == NULL)||(f == NULL)) return -1;  
  X[0][0] = 1;                                                                  // test input
  X[0][1] = 3;
  X[1][0] = 2;
  X[1][1] = 4;
  X[2][0] = 1;
  X[2][1] = 6;

  y[0][0] = 4;                                                                  // test output
  y[1][0] = 1;
  y[2][0] = 3;

  w = LR_lstsq(n, m, X, y);                                                     // weights
  if (w[0u][0u] != -1)                                                             // we do not have non-square matrices which do not have determinant
  {
     f = LR_product(n, m, m, 1, X, w);                                          // function values

     LR_matrix_clear(n, X);
     LR_matrix_clear(n, y);
     LR_matrix_clear(n, w);
     LR_matrix_clear(n, f);
  }
  return 0;
}
/*-----------------------------------------------------------------------------
 *      LR_array_sum():   sum the elements in the array
 *
 *  Parameters: float64_t arr[], int16_t len
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t LR_array_sum(float64_t arr[], int16_t len) 
{
    float64_t s = 0.0f;
    int16_t i;

  if (arr == NULL) return NaN;  
    for (i = 0; i < len; ++i) 
    {
        s += arr[i];
    }
    return s;
}
/*-----------------------------------------------------------------------------
 *      LR_array_pow():   raise the elements of the array to the power
 *
 *  Parameters: float64_t arr[], int16_t len, int16_t power
 *
 *  Return: float64_t*
 *----------------------------------------------------------------------------*/
float64_t* LR_array_pow(float64_t arr[], int16_t len, int16_t power) 
{
    int16_t i; 
    float64_t *arr2 = Malloc((len*sizeof(float64_t)));

    if (arr == NULL) { arr2[0u] = NaN; }
    else
    {  
        for (i = 0; i < len; ++i) {
           arr2[i] = pow(arr[i], power);
        }
    }
    return arr2;
}
/*-----------------------------------------------------------------------------
 *      LR_array_multiplication(): multiply the elements in the array by the scalar
 *                                 in the other array
 *
 *  Parameters: float64_t arr1[], float64_t arr2[], int16_t len
 *
 *  Return: float64_t*
 *----------------------------------------------------------------------------*/
float64_t* LR_array_multiplication(float64_t arr1[], float64_t arr2[], int16_t len) 
{
    int16_t i;
    float64_t *arr = Malloc((len*sizeof(float64_t)));

    if (arr2 == NULL) { arr[0u] = NaN; }
    else
    {  
       for (i = 0; i < len; ++i) {
          arr[i] = arr1[i] * arr2[i];
       }
    }
    return arr;
}
/*-----------------------------------------------------------------------------
 *      LR_array_diff(): find the difference between elements in the arrays by 
 *
 *  Parameters: float64_t arr1[], float64_t arr2[], int16_t len
 *
 *  Return: float64_t*
 *----------------------------------------------------------------------------*/
float64_t* LR_array_diff(float64_t arr1[], float64_t arr2[], int16_t len) 
{
    int16_t i;   
    float64_t *arr = Malloc(len*sizeof(float64_t));

    if (arr1 == NULL) { arr[0u] = NaN; }
    else
    {  
       for (i = 0; i < len; ++i) {
          arr[i] = arr1[i] - arr2[i];
       }
    }
    return arr;
}
/*-----------------------------------------------------------------------------
 *    LR_array_clear(): distroy the array specified from dynamic memory 
 *
 *  Parameters: float64_t *arr, int16_t len
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void LR_array_clear( float64_t *arr, int16_t len )
{
  if (arr == NULL) return;  
  Free((char*)arr,(len*sizeof(float64_t)));
}
/*-----------------------------------------------------------------------------
 *    LinearRegression_h(): calculate h 
 *
 *  Parameters: float64_t x, float64_t theta[]
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t LinearRegression_h(float64_t x, float64_t theta[]) 
{
    if (theta == NULL) return NaN;  
    return theta[0u] + theta[1u] * x;
}
/*-----------------------------------------------------------------------------
 *    LinearRegression_calculate_predictions(): calculate predictions 
 *
 *  Parameters: float64_t x[], float64_t theta[], int16_t m
 *
 *  Return: float64_t *
 *----------------------------------------------------------------------------*/
float64_t *LinearRegression_calculate_predictions(float64_t x[], float64_t theta[], int16_t m) 
{
    float64_t *predictions = Malloc(m*sizeof(float64_t));
    int16_t i;

    if ((theta == NULL) || (x == NULL)) { predictions[0u] = NaN; }
    else
    { 
       for (i = 0; i < m; ++i) {
          predictions[i] = LinearRegression_h(x[i], theta);
       }
    }
    return predictions;
}
/*-----------------------------------------------------------------------------
 *    LinearRegression_compute_cost(): calculate cost 
 *
 *  Parameters: float64_t x[], float64_t y[], float64_t theta[], int16_t m
 *
 *  Return: float64_t *
 *----------------------------------------------------------------------------*/
float64_t LinearRegression_compute_cost(float64_t x[], float64_t y[], float64_t theta[], int16_t m) 
{
    float64_t *predictions = LinearRegression_calculate_predictions(x, theta, m);
    float64_t *diff = LR_array_diff(predictions, y, m);
    float64_t *sq_errors = LR_array_pow(diff, m, 2);
    return (1.0f / (2.0f * m)) * LR_array_sum(sq_errors, m);
}
/*-----------------------------------------------------------------------------
 *   LinearRegression_gradient_descent(): perform gradient descent method  
 *
 *  Parameters: float64_t x[], float64_t y[], float64_t alpha, int16_t iters, 
 *              float64_t *J, int16_t m
 *
 *  Return: float64_t *
 *----------------------------------------------------------------------------*/
float64_t *LinearRegression_gradient_descent(float64_t x[], float64_t y[], float64_t alpha, int16_t iters, float64_t *J, int16_t m) 
{
    float64_t theta[2u];
    float64_t *predictions;
    float64_t *diff;
    float64_t *errors_x1;
    float64_t *errors_x2;
    int16_t i;
    theta[0u] = 1;
    theta[1u] = 1;

    if ((x == NULL) || (y == NULL)) { theta[0u] = NaN; }
    else
    { 
       for (i = 0; i < iters; ++i) {
          predictions = LinearRegression_calculate_predictions(x, &theta, m);
          diff = LR_array_diff(predictions, y, m);
          errors_x1 = diff;
          errors_x2 = LR_array_multiplication(diff, x, m);

          theta[0u] = theta[0u] - alpha * (1.0f / m) * LR_array_sum(errors_x1, m);
          theta[1u] = theta[1u] - alpha * (1.0f / m) * LR_array_sum(errors_x2, m);
          J[i] = LinearRegression_compute_cost(x, y, &theta, m);
      }
    }
    return &theta;
}
/*-----------------------------------------------------------------------------
 *   doGradDescent(): perform gradient descent method and return estimate 
 *
 *  Parameters: float64_t x[], float64_t y[], float64_t *J, float64_t x
 *
 *  Return: float64_t *
 *----------------------------------------------------------------------------*/
float64_t doGradDescent(float64_t x[], float64_t y[], float64_t *J, float64_t x_est) 
{
    float64_t *theta;
    float64_t value;

    if ((x == NULL) || (y == NULL)) return NaN; 
    theta = Malloc(sizeof(float64_t)*2u);
    theta = LinearRegression_gradient_descent(x, y, 0.01f, 1500, J, (sizeof(x[0u])/sizeof(x)));  /* 1500 iterations with learning rate alpha 0.01 */
    value = LinearRegression_h(x_est, theta);
    Free((char*)theta,sizeof(theta));
    return value;                                                               /* estimate of y */
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
#endif /* requirements were linear regression */