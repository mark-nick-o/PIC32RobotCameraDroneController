#include "definitions.h"
#if defined(ML_AI_NEEDED)                                                       /* ==== machine learning is required */
#if defined(NELDER_MEAD)                                                        /* ==== nelder ead method ========== */
/* =============================================================================

   Fast C implementation of the [Nelder-Mead method]
   (http://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method) 
   for unconstrained function minimization.
   
   This is a Non-smooth multivariate method : like Fibonacci search

   The cost function is passed as a function pointer argument, 
   this provides a general interface allowing for an easy customization. 
   The cost function takes as input a `point_t *` and an (optional) pointer to 
   other constant arguments `void *`. The `point_t` is a custom `struct` 
   composed by a `double *x` (i.e., the n-dimensional position) and a `double fx` 
   (i.e., the value of the cost function calculated at position `x`). 
   On exit the `fx` field of the input will contain the function evaluation. 
   The main `nelder_mead` function takes as input the dimensionality `n` of 
   the function space, the initial n-dimensional `point_t *start` and the final 
   minimizer `point_t *solution`, the pointer to a cost function defined as above, 
   the `void *` pointer to its optional argument, and finally a custom struct 
   `optimset_t` containing the parameters for the optimisation.
   
   Currently the cost function is a simple [Ackely function]
   (http://www.sfu.ca/%7Essurjano/ackley.html).
   This function allows function spaces of arbitrary dimension and it also shows 
   how the optional `void *` argument can be used.

   MIT Licence. Copyright (c) 2017 Matteo Maggioni
   Ported by ACP Aviation
   https://scipy-lectures.org/advanced/mathematical_optimization/
   ============================================================================= */
#include <stdint.h>
#include "nelder_mead.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void initNelderMead( optimset_t *optimset, ackley_param_t *ackley_params );
void ackley_fun(int16_t n, nelder_point_t *point, const void *arg);
void rosenbrock_fun(float64_t *parameters[2u], float64_t *cost, float64_t *gradient[2u] );
void copy_point(int16_t n, const nelder_point_t *src, nelder_point_t *dst);
void swap_points(int16_t n, nelder_point_t *p1, nelder_point_t *p2);
void update_point(const simplex_t *simplex, const nelder_point_t *centroid, float64_t lambda, nelder_point_t *point);
int16_t continue_minimization(const simplex_t *simplex, int16_t eval_count, int16_t iter_count, const optimset_t *optimset);
void get_centroid(const simplex_t *simplex, nelder_point_t *centroid);
void nelder_mead_with_ackley(int16_t n, const nelder_point_t *start, nelder_point_t *solution, const void *args, const optimset_t *optimset);
void doNelderMead(optimset_t *opt,ackley_param_t *ack, nelder_point_t *solution, nelder_point_t *start );
/* addtional cost residual functions */
void Osborne2_Residual(const float64_t *x[11u], float64_t *residual[65u]);
void BiggsEXP6_Residual(const float64_t *x[6u], float64_t *residual[13u]);
void Osborne1_Residual(const float64_t *x[5u], float64_t *residual[33u]);
void BrownDennis_Residual(const float64_t *x[4u], float64_t *residual[20u]);
void KowalikOsborne_Residual(const float64_t *x[4u], float64_t *residual[11u]);
void Box3D_Residual(const float64_t *x[3u], float64_t *residual[3u]);
void Powell_Singular_Residual(const float64_t *x[4u], float64_t *residual[4u]);
void Wood_Residual(const float64_t *x[4u], float64_t *residual[6u]);
void GulfRD_Residual(const float64_t *x[3u], float64_t *residual[100u]);
void Meyer_Residual(const float64_t *x[3u], float64_t *residual[16u]);
void Gaussian_Residual(const float64_t *x[3u], float64_t *residual[15u]);
void Bard_Residual(const float64_t *x[3u], float64_t *residual[15u]);
void HelicalValley_Residual(const float64_t *x[3u], float64_t *residual[3u]);
void JennrichSampson_Residual(const float64_t *x[2u], float64_t *residual[10u]);
void Beale_Residual(const float64_t *x[2u], float64_t *residual[3u]);
void Brown_BS_Residual(const float64_t *x[2u], float64_t *residual[3u]);
void Powell_BS_Residual(const float64_t *x[2u], float64_t *residual[2u]);
void FreudensteinRoth_Residual(const float64_t *x[2u], float64_t *residual[2u]);
void Rosenbrock_Residual(const float64_t *x[2u], float64_t *residual[2u]);

/*-----------------------------------------------------------------------------
 *      initNelderMead:  initialise the parameters for the nelder mead algrythm
 *
 *  Parameters: optimset_t *optimset, ackley_param_t *ackley_params  
 *               
 *  Return:     void
 *----------------------------------------------------------------------------*/   
void initNelderMead( optimset_t *optimset, ackley_param_t *ackley_params ) 
{                                                                               // optimisation settings
  optimset->tolx = 0.001f;                                                      // tolerance on the simplex solutions coordinates
  optimset->tolf = 0.001f;                                                      // tolerance on the function value
  optimset->max_iter = 1000;                                                    // maximum number of allowed iterations
  optimset->max_eval = 1000;                                                    // maximum number of allowed function evaluations
  optimset->verbose = 0;                                                        // toggle off verbose output during minimization (not currently used)

  ackley_params->a = 20.0f;                                                     // cost function parameters
  ackley_params->b = 0.2f;
  ackley_params->c = 2.0f * PI;  
}
//-----------------------------------------------------------------------------
// Implementation of a cost function f : R^n->R compatible with fun_t
// In this instance we use the Ackley Function as it allows us to demonstrate
// the use of the optional fixed arguments.
// More details on the function at http://www.sfu.ca/%7Essurjano/ackley.html
//-----------------------------------------------------------------------------
/*-----------------------------------------------------------------------------
 *      ackley_fun:  Ackley cost function as above
 *
 *  Parameters: int16_t n, nelder_point_t *point, const void *arg  
 *               
 *  Return:     void
 *----------------------------------------------------------------------------*/
void ackley_fun(int16_t n, nelder_point_t *point, const void *arg) 
{
  const ackley_param_t *params = (const ackley_param_t *)arg;                   // cast the void pointer to what we expect to find
  int16_t i;
  float64_t sum_squares = 0.0f;                                                 // cost function computation for arguments of exp
  float64_t sum_cos = 0.0f;
  for (i = 0; i < n; i++) 
  {
    sum_squares += SQUARE(point->x[i]);
    sum_cos += cos(params->c * point->x[i]);
  }

  point->fx = -params->a * exp(-params->b * sqrt(sum_squares / n)) - exp(sum_cos / n) + params->a + exp(1.0f);   // final result
}
// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)
//
// Test problems from the paper
//
// Testing Unconstrained Optimization Software
// Jorge J. More, Burton S. Garbow and Kenneth E. Hillstrom
// ACM Transactions on Mathematical Software, 7(1), pp. 17-41, 1981
//
// A subset of these problems were augmented with bounds and used for
// testing bounds constrained optimization algorithms by
//
// A Trust Region Approach to Linearly Constrained Optimization
// David M. Gay
// Numerical Analysis (Griffiths, D.F., ed.), pp. 72-105
// Lecture Notes in Mathematics 1066, Springer Verlag, 1984.
//
// The latter paper is behind a paywall. We obtained the bounds on the
// variables and the function values at the global minimums from
//
// http://www.mat.univie.ac.at/~neum/glopt/bounds.html
//
// A problem is considered solved if of the log relative error of its
// objective function is at least 4.
// ported by ACP Aviation as various cost function
void rosenbrock_fun(float64_t *parameters[2u], float64_t *cost, float64_t *gradient[2u] )  
{
    float64_t x;  
    float64_t y;  

    x = *parameters[0u];
    y = *parameters[1u];
    *cost = (1.0f - x) * (1.0f - x) + 100.0f * (y - x * x) * (y - x * x);
    if (gradient != NULL) 
    {
      *gradient[0u] = -2.0f * (1.0f - x) - 200.0f * (y - x * x) * 2.0f * x;
      *gradient[1u] = 200.0f * (y - x * x);
    }
}
void Rosenbrock_Residual(const float64_t *x[2u], float64_t *residual[2u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  *residual[0u] = 10.0f * (x2 - x1 * x1);
  *residual[1u] = 1.0f - x1;
}
void FreudensteinRoth_Residual(const float64_t *x[2u], float64_t *residual[2u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  *residual[0u] = -13.0f + x1 + ((5.0f - x2) * x2 - 2.0f) * x2;
  *residual[1u] = -29.0f + x1 + ((x2 + 1.0f) * x2 - 14.0f) * x2;
}
void Powell_BS_Residual(const float64_t *x[2u], float64_t *residual[2u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  *residual[0u] = 10000.0f * x1 * x2 - 1.0f;
  *residual[1u] = exp(-x1) + exp(-x2) - 1.0001f;
}
void Brown_BS_Residual(const float64_t *x[2u], float64_t *residual[3u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  *residual[0u] = x1  - 1000000.0f;
  *residual[1u] = x2 - 0.000002f;
  *residual[2u] = x1 * x2 - 2.0f;
}
void Beale_Residual(const float64_t *x[2u], float64_t *residual[3u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  *residual[0u] = 1.5f - x1 * (1.0f - x2);
  *residual[1u] = 2.25f - x1 * (1.0f - x2 * x2);
  *residual[2u] = 2.625f - x1 * (1.0f - x2 * x2 * x2);
}
void JennrichSampson_Residual(const float64_t *x[2u], float64_t *residual[10u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  uint8_t i = 1u;
  
  do 
  {
     *residual[i - 1u] = 2.0f + 2.0f * i - (exp((float64_t)(i) * x1) + exp((float64_t)(i) * x2));
      i=++i % 11u;
  } while ( i <= 10u );
}
void HelicalValley_Residual(const float64_t *x[3u], float64_t *residual[3u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  const float64_t theta = (0.5f / PI)  * atan(x2 / x1) + (x1 > 0.0f ? 0.0f : 0.5f);
  *residual[0u] = 10.0f * (x3 - 10.0f * theta);
  *residual[1u] = 10.0f * (sqrt(x1 * x1 + x2 * x2) - 1.0f);
  *residual[2u] = x3;
}
void Bard_Residual(const float64_t *x[3u], float64_t *residual[15u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  float64_t u,v,w;
  int16_t i;
  
  float64_t y[15u] = {0.14f, 0.18f, 0.22f, 0.25f, 0.29f, 0.32f, 0.35f, 0.39f, 0.37f, 0.58f, 0.73f, 0.96f, 1.34f, 2.10f, 4.39f};

  for (i = 1; i <=15; ++i) 
  {
    u = i;
    v = 16 - i;
    w = min(i, 16 - i);
    *residual[i - 1] = y[i - 1] - (x1 + u / (v * x2 + w * x3));
  }
}
void Gaussian_Residual(const float64_t *x[3u], float64_t *residual[15u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  const float64_t y[15u] = {0.0009f, 0.0044f, 0.0175f, 0.0540f, 0.1295f, 0.2420f, 0.3521f, 0.3989f, 0.3521f, 0.2420f, 0.1295f, 0.0540f, 0.0175f, 0.0044f, 0.0009f};
  float64_t t_i;
  int16_t i;  
  
  for (i = 0; i < 15; ++i) 
  {
    t_i = (8.0f - i - 1.0f) / 2.0f;
    *residual[i] = x1 * exp(-x2 * (t_i - x3) * (t_i - x3) / 2.0f) - y[i];
  }
}
void Meyer_Residual(const float64_t *x[3u], float64_t *residual[16u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  const float64_t y[16u] = {34780.0f, 28610.0f, 23650.0f, 19630.0f, 16370.0f, 13720.0f, 11540.0f, 9744.0f, 8261.0f, 7030.0f, 6005.0f, 5147.0f, 4427.0f, 3820.0f, 3307.0f, 2872.0f};
  int16_t i;
  float64_t ti;
  
  for (i = 0; i < 16; ++i) 
  {
    ti = 45.0f + 5.0f * (i + 1.0f);
    *residual[i] = x1 * exp(x2 / (ti + x3)) - y[i];
  }
}
void GulfRD_Residual(const float64_t *x[3u], float64_t *residual[100u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  float64_t ti,yi;
  int16_t i;
  
  for (i = 1; i <= 100; ++i) 
  {
    ti = i / 100.0f;
    yi = 25.0f + pow(-50.0f * log(ti), 2.0f / 3.0f);
    *residual[i - 1] = exp(-pow(abs((yi * 100.0f * i) * x2), x3) / x1) - ti;
  }
}
void Wood_Residual(const float64_t *x[4u], float64_t *residual[6u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  const float64_t x4 = *x[3u];

  *residual[0u] = 10.0f * (x2 - x1 * x1);
  *residual[1u] = 1.0f - x1;
  *residual[2u] = sqrt(90.0f) * (x4 - x3 * x3);
  *residual[3u] = 1.0f - x3;
  *residual[4u] = sqrt(10.0f) * (x2 + x4 - 2.0f);
  *residual[5u] = 1.0f / sqrt(10.0f) * (x2 - x4);
}
void Powell_Singular_Residual(const float64_t *x[4u], float64_t *residual[4u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  const float64_t x4 = *x[3u];

  *residual[0u] = x1 + 10.0f * x2;
  *residual[1u] = sqrt(5.0f) * (x3 - x4);
  *residual[2u] = (x2 - 2.0f * x3) * (x2 - 2.0f * x3);
  *residual[3u] = sqrt(10.0f) * (x1 - x4) * (x1 - x4);
}
void Box3D_Residual(const float64_t *x[3u], float64_t *residual[3u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];

  const float64_t t1 = 0.1f;
  const float64_t t2 = 0.2f;
  const float64_t t3 = 0.3f;

  *residual[0u] = exp(-t1 * x1) - exp(-t1 * x2) - x3 * (exp(-t1) - exp(-10.0f * t1));
  *residual[1u] = exp(-t2 * x1) - exp(-t2 * x2) - x3 * (exp(-t2) - exp(-10.0f * t2));
  *residual[2u] = exp(-t3 * x1) - exp(-t3 * x2) - x3 * (exp(-t3) - exp(-10.0f * t3));
}
void KowalikOsborne_Residual(const float64_t *x[4u], float64_t *residual[11u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  const float64_t x4 = *x[3u];

  const float64_t y[11u] = {0.1957f, 0.1947f, 0.1735f, 0.1600f, 0.0844f, 0.0627f, 0.0456f, 0.0342f, 0.0323f, 0.0235f, 0.0246f};
  const float64_t u[11u] = {4.0f, 2.0f, 1.0f, 0.5f, 0.25f, 0.167f, 0.125f, 0.1f, 0.0833f, 0.0714f, 0.0625f};
  int16_t i;
  
  for (i = 0; i < 11; ++i) 
  {
    *residual[i]  = y[i] - x1 * (u[i] * u[i] + u[i] * x2) / (u[i] * u[i]  + u[i] * x3 + x4);
  }
}
void BrownDennis_Residual(const float64_t *x[4u], float64_t *residual[20u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  const float64_t x4 = *x[3u];
  float64_t ti;
  int16_t i;

  for (i = 0; i < 20; ++i) 
  {
    ti = (i + 1.0f) / 5.0f;
    *residual[i] = (x1 + ti * x2 - exp(ti)) * (x1 + ti * x2 - exp(ti)) + (x3 + x4 * sin(ti) - cos(ti)) * (x3 + x4 * sin(ti) - cos(ti));
  }
}
void Osborne1_Residual(const float64_t *x[5u], float64_t *residual[33u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  const float64_t x4 = *x[3u];
  const float64_t x5 = *x[4u];  
  const float64_t y[33] = {0.844f, 0.908f, 0.932f, 0.936f, 0.925f, 0.908f, 0.881f, 0.850f, 0.818f,
                      0.784f, 0.751f, 0.718f, 0.685f, 0.658f, 0.628f, 0.603f, 0.580f, 0.558f,
                      0.538f, 0.522f, 0.506f, 0.490f, 0.478f, 0.467f, 0.457f, 0.448f, 0.438f,
                      0.431f, 0.424f, 0.420f, 0.414f, 0.411f, 0.406f};
  float64_t ti;
  int16_t i;
  for (i = 0; i < 33; ++i) 
  {
    ti = 10.0f * i;
    *residual[i] = y[i] - (x1 + x2 * exp(-ti * x4) + x3 * exp(-ti * x5));
  }
}
void BiggsEXP6_Residual(const float64_t *x[6u], float64_t *residual[13u])
{
  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  const float64_t x4 = *x[3u];
  const float64_t x5 = *x[4u];
  const float64_t x6 = *x[5u];
  float64_t ti,yi;
  int16_t i;
  
  for (i = 0; i < 13; ++i) 
  {
    ti = 0.1f * (i + 1.0f);
    yi = exp(-ti) - 5.0f * exp(-10.0f * ti) + 3.0 * exp(-4.0f * ti);
    *residual[i] = x3 * exp(-ti * x1) - x4 * exp(-ti * x2) + x6 * exp(-ti * x5) - yi;
  }
}
void Osborne2_Residual(const float64_t *x[11u], float64_t *residual[65u])
{

  const float64_t x1 = *x[0u];
  const float64_t x2 = *x[1u];
  const float64_t x3 = *x[2u];
  const float64_t x4 = *x[3u];
  const float64_t x5 = *x[4u];
  const float64_t x6 = *x[5u];
  const float64_t x7 = *x[6u];
  const float64_t x8 = *x[7u];
  const float64_t x9 = *x[8u];
  const float64_t x10 = *x[9u];
  const float64_t x11 = *x[10u];
  const float64_t y[65u] = {1.366f, 1.191f, 1.112f, 1.013f, 0.991f,
                      0.885f, 0.831f, 0.847f, 0.786f, 0.725f,
                      0.746f, 0.679f, 0.608f, 0.655f, 0.616f,
                      0.606f, 0.602f, 0.626f, 0.651f, 0.724f,
                      0.649f, 0.649f, 0.694f, 0.644f, 0.624f,
                      0.661f, 0.612f, 0.558f, 0.533f, 0.495f,
                      0.500f, 0.423f, 0.395f, 0.375f, 0.372f,
                      0.391f, 0.396f, 0.405f, 0.428f, 0.429f,
                      0.523f, 0.562f, 0.607f, 0.653f, 0.672f,
                      0.708f, 0.633f, 0.668f, 0.645f, 0.632f,
                      0.591f, 0.559f, 0.597f, 0.625f, 0.739f,
                      0.710f, 0.729f, 0.720f, 0.636f, 0.581f,
                      0.428f, 0.292f, 0.162f, 0.098f, 0.054f};
  float64_t ti;
  int16_t i;
  
  for (i = 0; i < 65; ++i) 
  {
    ti = i / 10.0f;
    *residual[i] = y[i] - (x1 * exp(-(ti * x5)) + x2 * exp(-(ti - x9)  * (ti - x9)  * x6) + x3 * exp(-(ti - x10) * (ti - x10) * x7) + x4 * exp(-(ti - x11) * (ti - x11) * x8));
  }
}
//-----------------------------------------------------------------------------
// Simple nelder_point_t manipulation utlities
//-----------------------------------------------------------------------------
/*-----------------------------------------------------------------------------
 *      copy_point:  Simple nelder_point_t manipulation utlities
 *
 *  Parameters: int16_t n, const nelder_point_t *src, nelder_point_t *dst  
 *               
 *  Return:     void
 *----------------------------------------------------------------------------*/
void copy_point(int16_t n, const nelder_point_t *src, nelder_point_t *dst) 
{
  int16_t j;        
  for (j = 0; j < n; j++) 
  {
    dst->x[j] = src->x[j];
  }
  dst->fx = src->fx;
}
/*-----------------------------------------------------------------------------
 *      swap_points:  swap 2 points
 *
 *  Parameters: int16_t n, nelder_point_t *p1, nelder_point_t *p2   
 *               
 *  Return:     void
 *----------------------------------------------------------------------------*/
void swap_points(int16_t n, nelder_point_t *p1, nelder_point_t *p2) 
{
  float64_t temp;
  int16_t j;
  for (j = 0; j < n; j++) 
  {
    temp = p1->x[j];
    p1->x[j] = p2->x[j];
    p2->x[j] = temp;
  }
  temp = p1->fx;
  p1->fx = p2->fx;
  p2->fx = temp;
}
//-----------------------------------------------------------------------------
// Update current point
//-----------------------------------------------------------------------------
/*-----------------------------------------------------------------------------
 *      update_point:  Update current point
 *
 *  Parameters: const simplex_t *simplex, const nelder_point_t *centroid,   
 *              float64_t lambda, nelder_point_t *point 
 *  Return:     void
 *----------------------------------------------------------------------------*/
void update_point(const simplex_t *simplex, const nelder_point_t *centroid, float64_t lambda, nelder_point_t *point) 
{
  const int16_t n = simplex->n;
  int16_t j;
  for (j = 0; j < n; j++) 
  {
    point->x[j] = (1.0f + lambda) * centroid->x[j] - lambda * simplex->p[n].x[j];
  }
}
//-----------------------------------------------------------------------------
// Assess if simplex satisfies the minimization requirements
//-----------------------------------------------------------------------------
/*-----------------------------------------------------------------------------
 *      continue_minimization:  Assess if simplex satisfies the minimization requirements
 *
 *  Parameters: const simplex_t *simplex, int16_t eval_count, int16_t iter_count,   
 *              const optimset_t *optimset
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
int16_t continue_minimization(const simplex_t *simplex, int16_t eval_count, int16_t iter_count, const optimset_t *optimset) 
{
  int16_t i,j;
  float64_t condx,condf;
  float64_t temp;
  
  if (eval_count > optimset->max_eval || iter_count > optimset->max_iter) 
  {
    return 0;                                                                   // stop if #evals or #iters are greater than the max allowed
  }
  condx = -1.0f;
  condf = -1.0f;
  for (i = 1; i < simplex->n + 1; i++) 
  {
    temp = fabs(simplex->p[0u].fx - simplex->p[i].fx);
    if (condf < temp) 
        {
      condf = temp;
    }
  }
  for (i = 1; i < simplex->n + 1; i++) 
  {
    for (j = 0; j < simplex->n; j++) 
        {
      temp = fabs(simplex->p[0u].x[j] - simplex->p[i].x[j]);
      if (condx < temp) 
          {
        condx = temp;
      }
    }
  }
  return condx > optimset->tolx || condf > optimset->tolf;                      // continue if both tolx or tolf condition is not met
}
//-----------------------------------------------------------------------------
// Get centroid (average position) of simplex
//-----------------------------------------------------------------------------
/*-----------------------------------------------------------------------------
 *      void get_centroid:  Get centroid (average position) of simplex
 *
 *  Parameters: const simplex_t *simplex, nelder_point_t *centroid  
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void get_centroid(const simplex_t *simplex, nelder_point_t *centroid) 
{ 
  int16_t j,i;
  for (j = 0; j < simplex->n; j++) 
  {
    centroid->x[j] = 0;
    for (i = 0; i < simplex->n; i++) 
    {
      centroid->x[j] += simplex->p[i].x[j];
    }
    centroid->x[j] /= simplex->n;
  }
}
//-----------------------------------------------------------------------------
// Main function
// - n is the dimension of the data
// - start is the initial point (unchanged in output)
// - solution is the minimizer
// - args are the optional arguments of cost_function which is ackley_fun
// - optimset are the optimisation settings
//-----------------------------------------------------------------------------
/*-----------------------------------------------------------------------------
 *      nelder_mead_with_ackley:  perform nelder mead algorythm
 *
 *  Parameters: int16_t n, const nelder_point_t *start, nelder_point_t *solution,  
 *              const void *args, const optimset_t *optimset
 *  Return:     void
 *----------------------------------------------------------------------------*/
void nelder_mead_with_ackley(int16_t n, const nelder_point_t *start, nelder_point_t *solution, const void *args, const optimset_t *optimset) 
{
  nelder_point_t point_r;                                                       // internal points
  nelder_point_t point_e;
  nelder_point_t point_c;
  nelder_point_t centroid;
  int16_t iter_count,eval_count,i,j,shrink;
  simplex_t simplex;                                                            // initial simplex has size n + 1 where n is the dimensionality pf the data
    
  point_r.x = Malloc(n * sizeof(float64_t));                                    // allocate memory for internal points
  point_e.x = Malloc(n * sizeof(float64_t));
  point_c.x = Malloc(n * sizeof(float64_t));
  centroid.x = Malloc(n * sizeof(float64_t));

  iter_count = 0;
  eval_count = 0;

  simplex.n = n;
  simplex.p = Malloc((n + 1) * sizeof(nelder_point_t));
  for (i = 0; i < n + 1; i++) {
    simplex.p[i].x = Malloc(n * sizeof(float64_t));
    for (j = 0; j < n; j++) {
      simplex.p[i].x[j] = (i - 1 == j) ? (start->x[j] != 0.0f ? 1.05f * start->x[j] : 0.00025f) : start->x[j];
    }
    ackley_fun(n, simplex.p + i, args);
    eval_count++;
  }
  // sort points in the simplex so that simplex.p[0] is the point having
  // minimum fx and simplex.p[n] is the one having the maximum fx
  /* TODO :: Port a sort function to mikroE C -------- simplex_sort(&simplex); */
  
  get_centroid(&simplex, &centroid);                                            // compute the simplex centroid
  iter_count=++iter_count % INT16_MAX;

  while (continue_minimization(&simplex, eval_count, iter_count, optimset))     // continue minimization until stop conditions are met
  {
    shrink = 0;

    /*if (optimset->verbose) {
      printf("Iteration %04d     ", iter_count);
    }*/
    update_point(&simplex, &centroid, RHO, &point_r);
    ackley_fun(n, &point_r, args);
    eval_count=++eval_count % INT16_MAX;
    if (point_r.fx < simplex.p[0u].fx) {
      update_point(&simplex, &centroid, RHO * CHI, &point_e);
      ackley_fun(n, &point_e, args);
      eval_count=++eval_count % INT16_MAX;
      if (point_e.fx < point_r.fx) {
        // expand
        /*if (optimset->verbose) {
          printf("expand          ");
        }*/
        copy_point(n, &point_e, simplex.p + n);
      } else {
        // reflect
        /*if (optimset->verbose) {
          printf("reflect         ");
        }*/
        copy_point(n, &point_r, simplex.p + n);
      }
    } else {
      if (point_r.fx < simplex.p[n - 1].fx) {
        // reflect
        /*if (optimset->verbose) {
          printf("reflect         ");
        }*/
        copy_point(n, &point_r, simplex.p + n);
      } else {
        if (point_r.fx < simplex.p[n].fx) {
          update_point(&simplex, &centroid, RHO * GAMMA, &point_c);
          ackley_fun(n, &point_c, args);
          eval_count=++eval_count % INT16_MAX;
          if (point_c.fx <= point_r.fx) {
            // contract outside
            /*if (optimset->verbose) {
              printf("contract out    ");
            }*/
            copy_point(n, &point_c, simplex.p + n);
          } else {
            // shrink
            /*if (optimset->verbose) {
              printf("shrink         ");
            }*/
            shrink = 1;
          }
        } else {
          update_point(&simplex, &centroid, -GAMMA, &point_c);
          ackley_fun(n, &point_c, args);
          eval_count=++eval_count % INT16_MAX;
          if (point_c.fx <= simplex.p[n].fx) {
            // contract inside
            /*if (optimset->verbose) {
              printf("contract in     ");
            }*/
            copy_point(n, &point_c, simplex.p + n);
          } else {
            // shrink
            /*if (optimset->verbose) {
              printf("shrink         ");
            }*/
            shrink = 1;
          }
        }
      }
    }
    if (shrink) {
      for (i = 1; i < n + 1; i++) {
        for (j = 0; j < n; j++) {
          simplex.p[i].x[j] = simplex.p[0].x[j] + SIGMA * (simplex.p[i].x[j] - simplex.p[0].x[j]);
        }
        ackley_fun(n, simplex.p + i, args);
        eval_count=++eval_count % INT16_MAX;
      }
      /* TODO :: Port a sort function to mikroE C -------- simplex_sort(&simplex); */
    } else {
      for (i = n - 1; i >= 0 && simplex.p[i + 1].fx < simplex.p[i].fx; i--) {
        swap_points(n, simplex.p + (i + 1), simplex.p + i);
      }
    }
    get_centroid(&simplex, &centroid);
    iter_count=++iter_count % INT16_MAX;
    /* CURRENTLY DEBUG IS OUT !!!!    if (optimset->verbose) {
      // print current minimum
      printf("[ ");
      for (i = 0; i < n; i++) {
        printf("%.2f ", simplex.p[0].x[i]);
      }
      printf("]    %.2f \n", simplex.p[0].fx);
    }*/
  }

  solution->x = Malloc(n * sizeof(float64_t));                                  // save solution in output argument
  copy_point(n, simplex.p + 0, solution);

  Free((char*)centroid.x,sizeof(centroid.x));                                   // Free memory
  Free((char*)point_r.x,sizeof(point_r.x));
  Free((char*)point_e.x,sizeof(point_e.x));
  Free((char*)point_c.x,sizeof(point_c.x));
  for (i = 0; i < n + 1; i++) 
  {
    Free((char*)simplex.p[i].x,sizeof(simplex.p[i].x));
  }
  Free((char*)simplex.p,sizeof(simplex.p));
}
/*-----------------------------------------------------------------------------
 *      doNelderMead:  perform nelder mead algorythm
 *
 *  Parameters: optimset_t *opt,ackley_param_t *ack, nelder_point_t *solution, 
 *              nelder_point_t *start
 *  Return:     void
 *----------------------------------------------------------------------------*/
void doNelderMead(optimset_t *opt,ackley_param_t *ack, nelder_point_t *solution, nelder_point_t *start )
{
   int16_t n = (sizeof(*start) / sizeof(nelder_point_t));        
   initNelderMead(opt,ack);
   nelder_mead_with_ackley(n, start, solution, ack, opt);   
}
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
#endif /* requirements were nelder mead */