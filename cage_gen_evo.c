#include <stdint.h>
#include "definitions.h"

#if defined(USE_CAGE_GEN_ALGO)
#ifdef __cplusplus
 extern "C" {
#endif

#include "Struts.h"
#include "cage_gen_evo.h"

#ifndef bool
#define bool uint8_t
#endif

bool g_minimize = true;                                                         /* use max or min function */

/* =============================================================================

CAGE is a very simple but powerful implementation of genetic algorithms in C++. 
In the standard (1+?) ES, each offspring is mutated from the population, whereas 
our offsprings are iteratively mutated, each one being the mutation ofthe previous. 
We apply restarts according to the Luby sequence.

CAGE is particularly suited for discrete problems that numerical optimization 
methods (like scipy.optimize.minimize) cannot handle. 
If your cost function is very expansive and quite regular, you might want use 
smarter algorithms (e.g. optuna).

Ported from https://github.com/louisabraham/cage by ACP Aviation

============================================================================= */

void cage_gen_evo_seed( uint16_t initSeed );
float64_t fRand( float64_t fMin, float64_t fMax );
cage_evo_elem_t cage_gen_evo_init();
void cage_gen_evo_fill_rand_XArray(float64_t **X);
float64_t cage_gen_evo_eval( float64_t *coefs, float64_t *x );
void cage_gen_evo_init_YArray(float64_t *y, float64_t **X);
float64_t cage_gen_evo_score( float64_t *coefs, float64_t **X, float64_t *y );
float64_t cage_gen_evo_mutate( float64_t *coefs );
uint64_t cage_gen_evo_luby( int16_t *x );
bool cage_gen_evo_restart( int16_t *cur_count, int16_t *cur_luby, int16_t *luby_count, int16_t luby_basis );
bool cage_gen_evo_maxi( float64_t a, float64_t b );
bool cage_gen_evo_mini( float64_t a, float64_t b );
void cage_gen_evo_rank( cage_evo_elem_t *e, float64_t **X, float64_t *y );
void cage_gen_evo_reset( cage_evo_elem_t *global_best, cage_evo_elem_t *best, float64_t **X, float64_t *y );
cage_evo_elem_t cage_evo_genetic_algorithm( float64_t **X, float64_t *y, bool minimize, int16_t pop_size, int16_t steps, int16_t luby_basis );
float64_t cage_gen_evo_rmse( float64_t **X, float64_t *y, bool minimize, int16_t pop_size, int16_t steps, int16_t luby_basis );
float64_t cage_gen_evo_mean( float64_t *y );
float64_t cage_gen_evo_std( float64_t *y, float64_t mean );

void cage_gen_evo_seed( uint16_t initSeed )
{
   srand( initSeed );
} 

float64_t fRand(float64_t fMin, float64_t fMax)
{
    float64_t f = (float64_t)rand() / RAND_MAX;
    return (fMin + f * (fMax - fMin));
}

cage_evo_elem_t cage_gen_evo_init() 
{
    cage_evo_elem_t ans;
    int16_t i;
    for (i = 0; i < (CAGE_GENEVO_DIM + 1); i++)
       ans.pair[i] = (fRand(MIN_FLOAT64,MAX_FLOAT64));
    return ans;
}

void cage_gen_evo_fill_rand_XArray(float64_t **X) 
{
    int16_t j,i;        
    for (j = 0; j < (CAGE_GENEVO_N); j++)
    {        
       for (i = 0; i < (CAGE_GENEVO_DIM); i++)
          X[j][i] = (fRand(MIN_FLOAT64,MAX_FLOAT64));
    }
}

float64_t cage_gen_evo_eval(float64_t *coefs, float64_t *x) 
{
   int16_t i; 
   float64_t ans = coefs[CAGE_GENEVO_DIM - 1];
   for (i = 0; i < CAGE_GENEVO_DIM; i++)
     ans += coefs[i] * x[i];
   return ans;
}

void cage_gen_evo_init_YArray(float64_t *y, float64_t **X) 
{
    int16_t j;
    float64_t coeffs[2u];
    cage_evo_elem_t true_coeffs; 

    true_coeffs = cage_gen_evo_init();        
    coeffs[0u] = true_coeffs.pair[0u];
    coeffs[1u] = true_coeffs.pair[1u];
        
    for (j = 0; j < (CAGE_GENEVO_N); j++)
    {        
        y[j] = cage_gen_evo_eval( &coeffs, &X[j]  );
    }
}

float64_t cage_gen_evo_score(float64_t *coefs, float64_t **X, float64_t *y) 
{
    int16_t i;
    float64_t e;
    float64_t error = 0;
    for (i = 0; i < CAGE_GENEVO_N; i++)
    {
        e = cage_gen_evo_eval(coefs, X[i]) - y[i];
        error += e * e;
    }
    error /= CAGE_GENEVO_N;
    if (error != error)
        return (float64_t)1e6;
    return error;
}
        
float64_t cage_gen_evo_mutate(float64_t *coefs) 
{
    float64_t x = coefs[rand() % (CAGE_GENEVO_DIM)];
    uint32_t ptr = ROUNDUINT(x);
    ptr ^= (1u << (rand() % 32u));
    return (float64_t)ptr;
}

// luby() function copied from MiniSat 2
// Copyright (c) 2003-2006, Niklas Een, Niklas Sorensson
// Copyright (c) 2007-2010, Niklas Sorensson
uint64_t cage_gen_evo_luby(int16_t *x)
{
    uint16_t subSeqSize, seq;
    for (subSeqSize = 1, seq = 0; subSeqSize < *x + 1; seq++, subSeqSize = 2 * subSeqSize + 1) // Find the finite subsequence that contains index 'x', and the size of that subsequence:
        ;
    while (subSeqSize - 1 != *x)
    {
        subSeqSize = (subSeqSize - 1) >> 1;
        seq--;
        *x = *x % subSeqSize;
    }
    return 1ULL << seq;
}

float64_t minisat2_luby(float64_t y, int16_t *x)
{
    // Find the finite subsequence that contains index 'x', and the
    // size of that subsequence:
    int16_t size, seq;
    for (size = 1, seq = 0; size < *x+1; seq++, size = 2*size+1);

    while (size-1 != *x){
        size = (size-1)>>1;
        seq--;
        *x = *x % size;
    }
    return pow(y, seq);
}

bool cage_gen_evo_restart(int16_t *cur_count, int16_t *cur_luby, int16_t *luby_count, int16_t luby_basis ) 
{
   if (luby_basis == -1)
      return false;
   if (*cur_count++ < *cur_luby)
      return false;
   *cur_count = 0;
   *luby_count++;
   *cur_luby = luby_basis * cage_gen_evo_luby(luby_count);
   return true;
}

bool cage_gen_evo_maxi(float64_t a, float64_t b)
{
    return a > b;
}

bool cage_gen_evo_mini(float64_t a, float64_t b)
{
    return a < b;
}

void cage_gen_evo_rank(cage_evo_elem_t *e, float64_t **X, float64_t *y) 
{
    float64_t coeffs[2u];
    coeffs[0u] = e->pair[0u];
    coeffs[1u] = e->pair[1u];        
    e->pair[0u] = cage_gen_evo_score(coeffs, X, y);
}

void cage_gen_evo_reset( cage_evo_elem_t *global_best, cage_evo_elem_t *best, float64_t **X, float64_t *y )  
{
     if (g_minimize==true)
     {
            if (cage_gen_evo_mini(best->pair[0u], global_best->pair[0])==true)
            {
               global_best->pair[0u] = best->pair[0u]; 
               global_best->pair[1u] = best->pair[1u];                
            }
     }
     else
     {
           if (cage_gen_evo_maxi(best->pair[0u], global_best->pair[0u])==true)
           {
              global_best->pair[0u] = best->pair[0u]; 
              global_best->pair[1u] = best->pair[1u];                 
           }                
     }
     *best = cage_gen_evo_init();   
     cage_gen_evo_rank(best, X, y);
}
        
cage_evo_elem_t cage_evo_genetic_algorithm( float64_t **X, float64_t *y, bool minimize, int16_t pop_size, int16_t steps, int16_t luby_basis )           // if not -1, use optimal Las Vegas restarts
{
    int16_t cur_luby = 1;
    int16_t luby_count = 0;
    int16_t cur_count = 0;
    cage_evo_elem_t best,global_best,cand;
    float64_t candArray[2u];
    int16_t i;
    bool restart;
            
    cage_gen_evo_seed((uint16_t)(CP0_GET(CP0_COUNT)%INT16_MAX));  // initialise the seed

    restart = cage_gen_evo_restart(&cur_count, &cur_luby, &luby_count, luby_basis ); 

    cage_gen_evo_reset( &global_best, &best, X, y );  
    global_best.pair[0u] = best.pair[0u]; 
    global_best.pair[1u] = best.pair[1u];   

    while (true)
    {
        cand.pair[0u] = best.pair[0u];
        cand.pair[1u] = best.pair[1u];
        for (i = 0; i < pop_size; i++)
        {
            candArray[0u] = cand.pair[0u];
            candArray[1u] = cand.pair[1u];                        
            cand.pair[1u] = cage_gen_evo_mutate(&candArray);    
            cage_gen_evo_rank(&cand, X, y);
            if (g_minimize==true)
            {
                if (cage_gen_evo_mini(cand.pair[0u], best.pair[0])==true)
                {
                    best.pair[0u] = cand.pair[0u]; 
                    best.pair[1u] = cand.pair[1u];                
                }
            }
            else
            {
                if (cage_gen_evo_maxi(best.pair[0u], global_best.pair[0u])==true)
                {
                     best.pair[0u] = cand.pair[0u]; 
                     best.pair[1u] = cand.pair[1u];  
                }                                                   
            }
            steps--;
            if (!steps)
            {
                cage_gen_evo_reset( &global_best, &best, X, y );
                return global_best;
            }
            if (cage_gen_evo_restart( &cur_count, &cur_luby, &luby_count, luby_basis )==true)
                cage_gen_evo_reset( &global_best, &best, X, y );
        }
    }
}

float64_t cage_gen_evo_rmse( float64_t **X, float64_t *y, bool minimize, int16_t pop_size, int16_t steps, int16_t luby_basis )
{
    cage_evo_elem_t ele; 
    ele = cage_evo_genetic_algorithm( X, y, minimize, pop_size, steps, luby_basis ); 
    return sqrt(ele.pair[0u]);
}

float64_t cage_gen_evo_mean( float64_t *y )
{
    int16_t i;
    float64_t mean = 0.0f;
    for (i = 0; i < CAGE_GENEVO_N; i++)
        mean += y[i];
    mean /= CAGE_GENEVO_N;
    return mean;
}

float64_t cage_gen_evo_std( float64_t *y, float64_t mean )
{
    int16_t i;
    float64_t var = 0.0f;
    float64_t tmp;
        
    for (i = 0; i < CAGE_GENEVO_N; i++)
    {
        tmp = y[i] - mean;
        var += tmp * tmp;
    }
    var /= CAGE_GENEVO_N;
    return sqrt(var);
}

#ifdef __cplusplus
}
#endif

#endif