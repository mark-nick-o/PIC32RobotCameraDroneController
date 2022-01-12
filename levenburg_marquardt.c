#include <stdint.h>
#include "definitions.h"
#include "levenburg_marquardt.h"

#ifdef __cplusplus
 extern "C" {
#endif

#pragma funcall doLevenburgMarquardtNLH LM_newton_func LM_gradient

float64_t LM_newton_func(float64_t *par, int16_t x, void *fdata);
void LM_gradient(float64_t *g, float64_t *par, int16_t x, void *fdata);
int16_t LM_temp_to_time(float64_t *par, float64_t temp);
void levmarq_init(LMstat_t *lmstat);
void solve_axb_cholesky(int16_t n, float64_t **l, float64_t *x, float64_t *b);
int16_t cholesky_decomp(int16_t n, float64_t **l, float64_t **a);
float64_t error_func(float64_t *par, int16_t ny, float64_t *y, float64_t *dysq,  float64_t (*func)(float64_t *, int16_t, void *), void *fdata);
int16_t levmarq( float64_t *par, float64_t *y, float64_t *dysq, float64_t (*func)(float64_t *, int16_t, void *), void (*grad)(float64_t *, float64_t *, int16_t, void *), void *fdata, LMstat_t *lmstat);
void doLevenburgMarquardtNLH( LMResults_t *res, void *fdata, LMstat_t *lmstat, float64_t *par, float64_t *y, float64_t *dysq, float64_t *t_data );
/* @brief   Function, describes Newton law of heating/cooling
 *
 * @usage   par[0] - temperature of heater,
 *          par[1] - initial temperature of water,
 *          par[2] - heat transmission coefficient
 *
 * @par     input of Newton Law:
 * @x       samplenumber
 * @fdata   additional data, not used
 *
 * @return  temperature at the time x
 */
float64_t LM_newton_func(float64_t *par, int16_t x, void *fdata)
{
    return par[0u] + (par[1u] - par[0u]) * exp( -par[2u]*x);
}

/*
 * @brief   Gradient function for Newton law of heating
 */
void LM_gradient(float64_t *g, float64_t *par, int16_t x, void *fdata)
{
    g[0u] = 1.0f - exp(-par[2u] * x);
    g[1u] = exp(-par[2u] * x);
    g[2u] = -x * (par[1u] - par[0u]) * exp(-par[2u] * x);
}

/*
 * @brief  Function for prediction of time, when target temperature will be reached
 * 
 * @par    Parameters from Newton equation
 * @temp   Target temperature
 * @return Number of sample
 */
int16_t LM_temp_to_time(float64_t *par, float64_t temp)
{
    return -(1.0f/par[2u]) * log((temp - par[0u])/(par[1u] - par[0u]));
}

/* set parameters required by levmarq() to default values */
void levmarq_init(LMstat_t *lmstat)
{
  lmstat->verbose = 0;
  lmstat->max_it = 10000;
  lmstat->init_lambda = 0.1f;
  lmstat->up_factor = 10.0f;
  lmstat->down_factor = 10.0f;
  lmstat->target_derr = 0.01f;
}

/* solve the equation Ax=b for a symmetric positive-definite matrix A,
   using the Cholesky decomposition A=LL^T.  The matrix L is passed in "l".
   Elements above the diagonal are ignored.
*/
void solve_axb_cholesky(int16_t n, float64_t **l, float64_t *x, float64_t *b)
{
  int16_t i,j;
  float64_t sum;

  /* solve L*y = b for y (where x[] is used to store y) */

  for (i=0; i<n; i++) {
    sum = 0;
    for (j=0; j<i; j++)
      sum += l[i][j] * x[j];
    x[i] = (b[i] - sum)/l[i][i];      
  }

  /* solve L^T*x = y for x (where x[] is used to store both y and x) */

  for (i=n-1; i>=0; i--) {
    sum = 0.0f;
    for (j=i+1; j<n; j++)
      sum += l[j][i] * x[j];
    x[i] = (x[i] - sum)/l[i][i];      
  }
}


/* This function takes a symmetric, positive-definite matrix "a" and returns
   its (lower-triangular) Cholesky factor in "l".  Elements above the
   diagonal are neither used nor modified.  The same array may be passed
   as both l and a, in which case the decomposition is performed in place.
*/
int16_t cholesky_decomp(int16_t n, float64_t **l, float64_t **a)
{
  int16_t i,j,k;
  float64_t sum;

  for (i=0; i<n; i++) {
    for (j=0; j<i; j++) {
      sum = 0.0f;
      for (k=0; k<j; k++)
        sum += l[i][k] * l[j][k];
      l[i][j] = (a[i][j] - sum)/l[j][j];
    }

    sum = 0;
    for (k=0; k<i; k++)
      sum += l[i][k] * l[i][k];
    sum = a[i][i] - sum;
    if (sum<TOL) return 1; /* not positive-definite */
    l[i][i] = sqrt(sum);
  }
  return 0;
}

/* calculate the error function (chi-squared) */
float64_t error_func(float64_t *par, int16_t ny, float64_t *y, float64_t *dysq,  float64_t (*func)(float64_t *, int16_t, void *), void *fdata)
{
  int16_t x;
  float64_t res,e=0;

  for (x=0; x<ny; x++) {
    res = func(par, x, fdata) - y[x];
    if (dysq)  /* weighted least-squares */
      e += res*res/dysq[x];
    else
      e += res*res;
  }
  return e;
}

/* Ron Babich, May 2008  Port by ACP */

/* perform least-squares minimization using the Levenberg-Marquardt
   algorithm.  The arguments are as follows:
   LM_N_PARAMS    number of parameters
   par     array of parameters to be varied
   LM_N_MEASUREMENTS      number of measurements to be fit
   y       array of measurements
   dysq    array of error in measurements, squared
           (set dysq=NULL for unweighted least-squares)
   func    function to be fit
   grad    gradient of "func" with respect to the input parameters
   fdata   pointer to aLM_N_MEASUREMENTS additional data required by the function
   lmstat  pointer to the "status" structure, where minimization parameters
           are set and the final status is returned.
   Before calling levmarq, several of the parameters in lmstat must be set.
   For default values, call levmarq_init(lmstat).
 */
int16_t levmarq( float64_t *par, float64_t *y, float64_t *dysq, float64_t (*func)(float64_t *, int16_t, void *), void (*grad)(float64_t *, float64_t *, int16_t, void *), void *fdata, LMstat_t *lmstat)
{
  int16_t x,i,j,it,nit,ill,verbose;
  float64_t lambda,up,down,mult,weight,err,newerr,derr,target_derr;
  float64_t h[LM_N_PARAMS][LM_N_PARAMS],ch[LM_N_PARAMS][LM_N_PARAMS];
  float64_t g[LM_N_PARAMS],d[LM_N_PARAMS],delta[LM_N_PARAMS],newpar[LM_N_PARAMS];

  verbose = lmstat->verbose;
  nit = lmstat->max_it;
  lambda = lmstat->init_lambda;
  up = lmstat->up_factor;
  down = 1/lmstat->down_factor;
  target_derr = lmstat->target_derr;
  weight = 1;
  derr = newerr = 0; /* to avoid compiler warnings */

  /* calculate the initial error ("chi-squared") */
  err = error_func(par, LM_N_MEASUREMENTS, y, dysq, func, fdata);

  /* main iteration */
  for (it=0; it<nit; it++) {

    /* calculate the approximation to the Hessian and the "derivative" d */
    for (i=0; i<LM_N_PARAMS; i++) {
      d[i] = 0;
      for (j=0; j<=i; j++)
        h[i][j] = 0;
    }
    for (x=0; x<LM_N_MEASUREMENTS; x++) {
      if (dysq) weight = 1/dysq[x]; /* for weighted least-squares */
      grad(g, par, x, fdata);
      for (i=0; i<LM_N_PARAMS; i++) {
        d[i] += (y[x] - func(par, x, fdata))*g[i]*weight;
        for (j=0; j<=i; j++)
          h[i][j] += g[i]*g[j]*weight;
      }
    }

    /*  make a step "delta."  If the step is rejected, increase
       lambda and try again */
    mult = 1 + lambda;
    ill = 1; /* ill-conditioned? */
    while (ill && (it<nit)) {
      for (i=0; i<LM_N_PARAMS; i++)
        h[i][i] = h[i][i]*mult;

      ill = cholesky_decomp(LM_N_PARAMS, ch, h);

      if (!ill) {
        solve_axb_cholesky(LM_N_PARAMS, ch, delta, d);
        for (i=0; i<LM_N_PARAMS; i++)
          newpar[i] = par[i] + delta[i];
        newerr = error_func(newpar, LM_N_MEASUREMENTS, y, dysq, func, fdata);
        derr = newerr - err;
        ill = (derr > 0);
      } 
      if (ill) {
        mult = (1 + lambda*up)/(1 + lambda);
        lambda *= up;
        it++;
      }
    }
    for (i=0; i<LM_N_PARAMS; i++)
      par[i] = newpar[i];
    err = newerr;
    lambda *= down;  

    if ((!ill)&&(-derr<target_derr))
    {
        break;
    }
  }

  lmstat->final_it = it;
  lmstat->final_err = err;
  lmstat->final_derr = derr;

  return it;
}

/* double t_data[N_MEASUREMENTS] - is the measurement data set */
/*-----------------------------------------------------------------------------
 *  doLevenburgMarquardt:  Perform Levenburg Marquardt for solving Newton law of heating 
 *
 *  Parameters: LMResults_t *res, void *fdata, LMstat_t *lmstat, float64_t *par, 
 *              float64_t *y, float64_t *dysq, float64_t *t_data 
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void doLevenburgMarquardtNLH( LMResults_t *res, void *fdata, LMstat_t *lmstat, float64_t *par, float64_t *y, float64_t *dysq, float64_t *t_data )
{
    float64_t params[LM_N_PARAMS] = {200.0f, 20.0f, 0.001f};              // Initial values of parameters  Temp of heater, init temperature of water, heating co-efficient
    levmarq_init(lmstat);
    res->n_iterations = levmarq( &params, t_data, (float64_t*) NULL, &LM_newton_func, &LM_gradient, (void*) NULL, lmstat);
    res->T_heater = params[0u];
    res->T_0 = params[1u]; 
    res->k = params[2u];
    res->TT = LM_temp_to_time(params, 50.0f);
}

#ifdef __cplusplus
}
#endif