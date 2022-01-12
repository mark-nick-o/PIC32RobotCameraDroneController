#include <stdlib.h>
#include "definitions.h"                                                        // global defines
#include "least_squares.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/**
 * Least Squares Best Fit by Roxy and Ed Williams
 *
 * This algorithm is high speed and has a very small code footprint.
 * Its results are identical to both the Iterative Least-Squares published
 * earlier by Roxy and the QR_SOLVE solution. If used in place of QR_SOLVE
 * it saves roughly 10K of program memory. It also does not require all of
 * coordinates to be present during the calculations. Each point can be
 * probed and then discarded.
 *
 */

void incremental_LSF_reset(linear_fit_data_t *lsf);
void incremental_WLSF(linear_fit_data_t *lsf, const float64_t x, const float64_t y, const float64_t z, const float64_t w);
void incremental_LSF(linear_fit_data_t *lsf, const float64_t x, const float64_t y, const float64_t z);
 
void incremental_LSF_reset(linear_fit_data_t *lsf) 
{
  if (lsf == NULL)
    return;
  memset(lsf, 0, sizeof(linear_fit_data_t));
}

void incremental_WLSF(linear_fit_data_t *lsf, const float64_t x, const float64_t y, const float64_t z, const float64_t w) 
{
  // weight each accumulator by factor w, including the "number" of samples
  // (analogous to calling inc_LSF twice with same values to weight it by 2X)
  float64_t wx = w * x, wy = w * y, wz = w * z;
  if (lsf == NULL)
    return;

  lsf->xbar  += wx;
  lsf->ybar  += wy;
  lsf->zbar  += wz;
  lsf->x2bar += wx * x;
  lsf->y2bar += wy * y;
  lsf->z2bar += wz * z;
  lsf->xybar += wx * y;
  lsf->xzbar += wx * z;
  lsf->yzbar += wy * z;
  lsf->N     += w;
  lsf->max_absx = FMAX(abs(wx), lsf->max_absx);
  lsf->max_absy = FMAX(abs(wy), lsf->max_absy);
}

void incremental_LSF(linear_fit_data_t *lsf, const float64_t x, const float64_t y, const float64_t z) 
{
  if (lsf == NULL)
    return;
  lsf->xbar += x;
  lsf->ybar += y;
  lsf->zbar += z;
  lsf->x2bar += pow(x,2.0f);
  lsf->y2bar += pow(y,2.0f);
  lsf->z2bar += pow(z,2.0f);
  lsf->xybar += x * y;
  lsf->xzbar += x * z;
  lsf->yzbar += y * z;
  lsf->max_absx = FMAX(abs(x), lsf->max_absx);
  lsf->max_absy = FMAX(abs(y), lsf->max_absy);
  lsf->N += 1.0;
}

int16_t finish_incremental_LSF(linear_fit_data_t *lsf) 
{

  float64_t N;
  float64_t DD;

  if (lsf == NULL)
    return -1;
    
  N = lsf->N;  
  if (N == 0.0f)
    return 1;

  lsf->xbar /= N;
  lsf->ybar /= N;
  lsf->zbar /= N;
  lsf->x2bar = lsf->x2bar / N - pow(lsf->xbar,2.0f);
  lsf->y2bar = lsf->y2bar / N - pow(lsf->ybar,2.0f);
  lsf->z2bar = lsf->z2bar / N - pow(lsf->zbar,2.0f);
  lsf->xybar = lsf->xybar / N - lsf->xbar * lsf->ybar;
  lsf->yzbar = lsf->yzbar / N - lsf->ybar * lsf->zbar;
  lsf->xzbar = lsf->xzbar / N - lsf->xbar * lsf->zbar;
  DD = lsf->x2bar * lsf->y2bar - pow(lsf->xybar,2.0f);

  if (abs(DD) <= 1e-10f * (lsf->max_absx + lsf->max_absy))
    return 1;

  lsf->A = (lsf->yzbar * lsf->xybar - lsf->xzbar * lsf->y2bar) / DD;
  lsf->B = (lsf->xzbar * lsf->xybar - lsf->yzbar * lsf->x2bar) / DD;
  lsf->D = -(lsf->zbar + lsf->A * lsf->xbar + lsf->B * lsf->ybar);
  return 0;
}

#ifdef __cplusplus
}
#endif