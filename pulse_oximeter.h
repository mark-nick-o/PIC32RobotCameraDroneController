#ifndef __OXIMETER__
#define __OXIMETER__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* ===== The FIR filter and decimator is the middle link between the sampling and FFT processes ==============

         ref: Joseph Bailey / Michael Fecteau / Noah Pendleton WIRELESS PULSE OXIMETER
                 WORCESTER POLYTECHNIC INSTITUTE
                 
         Buffers fill on interrupt cyclically suggested max size is 86
         every tenth interrupt we call FIR_Filter_Deci (changed this to whn the buffer has filled up)
                 when the FIR is filled we call the FFT
                
   ===========================================================================================================
*/
#include <stdint.h>
#include "definitions.h"
#include "gc_events.h"
/*#include "mmc_file_handler.h"*/

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define P_OXYPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define P_OXYPACKED __attribute__((packed)) ALIGNED(1)                        /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define P_OXYPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define P_OXYPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define P_OXYPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define MAX_DATA_BUFF_SIZ 86
#define MAX_FIR_BUFF_SIZ 16
#define BPM_SAMP_FREQ_FACT 0.242f

#define OXY_LIB extern /* or static */

typedef enum {
  FFT_DATA_COLLECTING = 0,                                                     /* collect data */
  FFT_DATA_READY = 1,                                                          /* data ready */
  FFT_NUM_ENUM
} FFT_Status_e;

typedef enum {
  FIR_DATA_COLLECTING = 0,                                                     /* collect data */
  FIR_DATA_READY = 1,                                                          /* data ready */
  FIR_NUM_ENUM
} FIR_Status_e;

typedef enum {
  WPO_DATA_CALCULATING = 0,                                                     /* collect data */
  WPO_DATA_READY = 1,                                                           /* data ready */
  WPO_NUM_ENUM
} WPO_Status_e;

typedef enum {
  OXY_INIT = 0,                                                              /* init state */
  OXY_COLLECT = 1,                                                           /* collect data */
  OXY_FINISHED = 2,
  OXY_NUM_ENUM
} OXY_Status_e;

#if defined(D_FT900)
typedef struct P_OXYPACKED {
  int64_t FIRred;
  int64_t FIRir;
} FIR_Filt_Deci64_t;
#else
P_OXYPACKED(
typedef struct {
  int64_t FIRred;
  int64_t FIRir;
}) FIR_Filt_Deci64_t;
#endif

#if defined(D_FT900)
typedef struct P_OXYPACKED {
  int16_t FIRred;
  int16_t FIRir;
  int16_t IIR_order3;
  int16_t FIR_IIR_Diff;
} FIR_Filt_Deci16_t;
#else
P_OXYPACKED(
typedef struct {
  int16_t FIRred;
  int16_t FIRir;
  int16_t IIR_order3;
  int16_t FIR_IIR_Diff;
}) FIR_Filt_Deci16_t;
#endif

#if defined(D_FT900)
typedef struct P_OXYPACKED {
   int16_t FFTinputRed[MAX_FIR_BUFF_SIZ];
   int16_t FFTinputRedMax;
   int16_t FFTinputIr[MAX_FIR_BUFF_SIZ];
   int16_t FFTinputIrMax;
   int16_t BPMzeroItemCnt;
   int8_t FFTstate : 1u;
   int8_t FIRstate : 1u;
   int8_t calcsDone : 1u;
   int8_t ZeroCross : 2u;
   int8_t spare : 3u;
   float64_t calcBPM;
   float64_t SP02;
   float64_t calcBPM_last;
   float64_t calcBPM_delta;
   float64_t SP02_last;
   float64_t SP02_delta;
   int32_t ticks2Now;
   uint32_t lastTickRef;
   int32_t dly2Now;
   uint32_t lastDlyRef;
   uint8_t oxyState;
} FFT_Results_t;
/*   int16_t BPMinputIr[MAX_FIR_BUFF_SIZ]; */
#else
P_OXYPACKED(
typedef struct {
   int16_t FFTinputRed[MAX_FIR_BUFF_SIZ];
   int16_t FFTinputRedMax;
   int16_t FFTinputIr[MAX_FIR_BUFF_SIZ];
   int16_t FFTinputIrMax;
   int16_t BPMzeroItemCnt;
   int8_t FFTstate : 1u;
   int8_t FIRstate : 1u;
   int8_t calcsDone : 1u;
   int8_t ZeroCross : 2u;
   int8_t spare : 3u;
   float64_t calcBPM;
   float64_t SP02;
   float64_t calcBPM_last;
   float64_t calcBPM_delta;
   float64_t SP02_last;
   float64_t SP02_delta;
   int32_t ticks2Now;
   uint32_t lastTickRef;
   int32_t dly2Now;
   uint32_t lastDlyRef;
   uint8_t oxyState;
}) FFT_Results_t;
/*   int16_t BPMinputIr[MAX_FIR_BUFF_SIZ]; */
#endif

#if defined(D_FT900)
typedef struct P_OXYPACKED {
  int16_t a1;
  int16_t a2;
  int16_t h;
} o2Values_t;
#else
P_OXYPACKED(
typedef struct {
  int16_t a1;
  int16_t a2;
  int16_t h;
}) o2Values_t;
#endif

/* ======================================== function prototypes ================================================================================================================ */
void do3rdOrderIIRFilter( FIR_Filt_Deci16_t **firOut, int16_t *idx );
void FIR_Filter_Deci64( FIR_Filt_Deci64_t **firOut, const uint32_t *a1, const uint32_t *a2, const uint32_t *h, int64_t Nb, int16_t oldest );
void FIR_Filter_Deci16( FIR_Filt_Deci16_t **firOut, const uint16_t *a1, const uint16_t *a2, const uint16_t *h, int16_t Nb, int16_t oldest, int16_t *idx, FFT_Results_t *fftRes );
int8_t CollectData4FIR_FFT( FIR_Filt_Deci16_t **firOut, uint16_t *a1, uint16_t *a2, uint16_t *h, int16_t Nb, int16_t *idxD, int16_t *idx, FFT_Results_t *fftRes, const o2Values_t values );
void doFFT( int16_t *inputSamples );
void maxFFTVals( FFT_Results_t *fftRes );
uint8_t oppositeSigns( int16_t x, int16_t y);
void zeroCrossFFTValsBPM( FFT_Results_t *fftRes );
void calcSPO2( FFT_Results_t *fftRes );
void getSP02fromFFTdata( FIR_Filt_Deci16_t **firOut, FFT_Results_t *fftRes );
void runOxyAlgo( FFT_Results_t *fftRes, FIR_Filt_Deci16_t **firOut, uint16_t *a1, uint16_t *a2, uint16_t *h, int16_t Nb, int16_t *idxD, int16_t *idx, const o2Values_t values );

/*-----------------------------------------------------------------------------
 *  do3rdOrderIIRFilter(): perform IIR filter 3rd order
 *              
 *  Parameters: FIR_Filt_Deci16_t **firOut, int16_t *idx
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void do3rdOrderIIRFilter( FIR_Filt_Deci16_t **firOut, int16_t *idx )
{
   const int16_t FILTER_ORDER = 3;
   const int16_t COEFF_B[FILTER_ORDER+1u] = {0x344D, 0x689A, 0x344D};
   const int16_t COEFF_A[FILTER_ORDER+1u] = {0x4000, 0xA003, 0x2687};
   const int16_t SCALE_B = 4u;
   const int16_t SCALE_A = -1;

   int16_t inputSamples[3u]; 
   int16_t outputSamples[10u];
   int16_t i = -1;
  
   while (i <= 1)
   {
      inputSamples[i+1] = firOut[*idx+i]->FIRir;
      i = ++i % INT16_MAX;
   } 
   for(i = 0u; i < 7u; i++)
   {
     outputSamples[i] = IIR_Radix(SCALE_B, SCALE_A, COEFF_B, COEFF_A, FILTER_ORDER+1, inputSamples, 3, outputSamples, i);
   }
   memcpy((void*)&firOut[*idx]->IIR_order3, (void*)&outputSamples, sizeof(outputSamples));
   firOut[*idx]->FIR_IIR_Diff = firOut[*idx]->FIRir - firOut[*idx]->IIR_order3; 
}

/*-----------------------------------------------------------------------------
 *  doFFT(): perform the FFT
 *              
 *  Parameters: int16_t *inputSamples
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
OXY_LIB void doFFT( int16_t *inputSamplesV )
{
   const int16_t Wn[8u] = {0x7fff, 0, 0x5A82, 0xA57E, 0, 0x8000, 0xA57E, 0xA57E};    /* Twiddle factors for 8 pairs complex input values. */
   const uint16_t log2N = 3U;

   FFT(inputSamplesV, Wn, log2N);
   BitReverseComplex(inputSamplesV, log2N);
}

/*-----------------------------------------------------------------------------
 *  maxFFTVals(): max values from FFT
 *              
 *  Parameters: FFT_Results_t *fftRes
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
OXY_LIB void maxFFTVals( FFT_Results_t *fftRes )
{
        int8_t cnt;
        
        for (cnt=0;cnt<=MAX_FIR_BUFF_SIZ;cnt++)
        {
            if (cnt == 0)
            {
               fftRes->FFTinputRedMax = max(fftRes->FFTinputRed[cnt], fftRes->FFTinputRed[cnt+1u]);
               fftRes->FFTinputIrMax = max(fftRes->FFTinputIr[cnt], fftRes->FFTinputIr[cnt+1u]);
            }
            else if (cnt == 1)
            {
                /* ----- done above ----- */
            }
            else
            {
               fftRes->FFTinputRedMax = max(fftRes->FFTinputRedMax, fftRes->FFTinputRed[cnt]);                        
               fftRes->FFTinputIrMax = max(fftRes->FFTinputIrMax, fftRes->FFTinputIr[cnt]);        
            }
        }
}

/*-----------------------------------------------------------------------------
 *  oppositeSigns(): return true if zero crossing between numbers
 *              
 *  Parameters: int16_t x, int16_t y
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
OXY_LIB uint8_t oppositeSigns(int16_t x, int16_t y)
{
    return ((x ^ y) < 0);
}

/*-----------------------------------------------------------------------------
 *  zeroCrossFFTVals(): calculate zero crossing array (contains values between zero crossings)
 *                      hence BPM (pulse rate)
 *              
 *  Parameters: FFT_Results_t *fftRes
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
OXY_LIB void zeroCrossFFTValsBPM( FFT_Results_t *fftRes )
{
        int8_t cnt, bpmIdx = 0, bpmTot = 0;
        int16_t zeroVal1 = 0;
        uint16_t bpmSum = 0U;
        
        for (cnt=0;cnt<=MAX_FIR_BUFF_SIZ;cnt++)
        {
            if (cnt == 0)
            {
                zeroVal1 = fftRes->FFTinputIr[cnt];
                fftRes->ZeroCross = 0;
                fftRes->BPMzeroItemCnt = 0;        
            }
            else
            {
               if ((oppositeSigns(zeroVal1,fftRes->FFTinputIr[cnt])==1) && (fftRes->ZeroCross == 1))
               {
                  /* fftRes->BPMinputIr[bpmIdx] = fftRes->BPMzeroItemCnt; */
                  bpmSum =+ fftRes->BPMzeroItemCnt;
                  bpmTot = ++bpmTot % INT8_MAX;
                  fftRes->BPMzeroItemCnt = 1;        
                  bpmIdx = ++bpmIdx % INT8_MAX;
                  fftRes->ZeroCross = 2;                                
               }
               else if ((oppositeSigns(zeroVal1,fftRes->FFTinputIr[cnt])==1) && (fftRes->ZeroCross == 2))
               {
                  /* fftRes->BPMinputIr[bpmIdx] = fftRes->BPMzeroItemCnt; */
                  bpmSum =+ fftRes->BPMzeroItemCnt;
                  bpmTot = ++bpmTot % INT8_MAX;
                  fftRes->BPMzeroItemCnt = 1;        
                  bpmIdx = ++bpmIdx % INT8_MAX;
                  fftRes->ZeroCross = 1;                                
               }
               else if ((oppositeSigns(zeroVal1,fftRes->FFTinputIr[cnt])==1) && (fftRes->ZeroCross == 0))                
               {
                  fftRes->BPMzeroItemCnt = 1;
                  fftRes->ZeroCross = 1;
               }
               else if ((fftRes->ZeroCross == 1) || (fftRes->ZeroCross == 2))
               {
                  fftRes->BPMzeroItemCnt = ++fftRes->BPMzeroItemCnt % UINT16_MAX;                                
               }
               else
               {
               }                                
               zeroVal1 = fftRes->FFTinputIr[cnt];        
           }
        }
        
        /*********************************************************************** 
        for (cnt=0;cnt<=MAX_FIR_BUFF_SIZ;cnt++)                                 ineffecient but can be used for debug if needed
        {
                if (fftRes->BPMinputIr[bpmIdx]!=0)
                {
                        bpmSum =+ fftRes->BPMinputIr[bpmIdx];
                        bpmTot = ++bpmTot % UINT8_MAX;
               }                        
        } 
        ***********************************************************************/
       if (bpmTot != 0)
       {                
           fftRes->calcBPM = (((float64_t)bpmSum) / ((float64_t)bpmTot)) * BPM_SAMP_FREQ_FACT * 60.0f;
       }
       else
       {
           fftRes->calcBPM = 0.0f;
       }
        
}

/*-----------------------------------------------------------------------------
 *  calcSPO2(): Calculate the sp02
 *              
 *  Parameters: FFT_Results_t *fftRes
 *
 *  Return: void 
 *----------------------------------------------------------------------------*/
OXY_LIB void calcSPO2( FFT_Results_t *fftRes ) 
{
        if (fftRes->FFTinputIrMax != 0)
        {
             fftRes->SP02 = (-25.0f * ((float64_t)fftRes->FFTinputRedMax) / ((float64_t)fftRes->FFTinputIrMax)) + 110.0f;
        }
        else
        {
             fftRes->SP02 = 0.0f;
        }
}

/*-----------------------------------------------------------------------------
 *  getSP02fromFFTdata(): Calculate the sp02 from the FFT data
 *              
 *  Parameters: FIR_Filt_Deci16_t **firOut, FFT_Results_t *fftRes
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
OXY_LIB void getSP02fromFFTdata( FIR_Filt_Deci16_t **firOut, FFT_Results_t *fftRes )
{
    fftRes->calcsDone = WPO_DATA_CALCULATING;
    memcpy((void*)&fftRes->FFTinputRed, (void*)&firOut[0u]->FIRred, MAX_FIR_BUFF_SIZ);    /* to save memory we could do away with the result buffer its here to debug easier */
    doFFT( &fftRes->FFTinputRed );                                               /* example :: doFFT( &firOut[0u]->FIRred ); */
    memcpy((void*)&fftRes->FFTinputIr, (void*)&firOut[0u]->FIRir, MAX_FIR_BUFF_SIZ);
    doFFT( &fftRes->FFTinputIr );        
    maxFFTVals( fftRes );
    calcSPO2( fftRes );
    zeroCrossFFTValsBPM( fftRes );
    fftRes->calcsDone = WPO_DATA_READY;
}
/*-----------------------------------------------------------------------------
 *  FIR_Filter_Deci64(): FIR filter implementation 64bit numbers
 *              
 *  Parameters: FIR_Filt_Deci64_t * firOut, uint32_t *a1, uint32_t *a2, uint32_t *h, 
 *              int64_t Nb, int16_t oldest  
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
OXY_LIB void FIR_Filter_Deci64( FIR_Filt_Deci64_t * firOut, const uint32_t *a1, const uint32_t *a2, const uint32_t *h, int64_t Nb, int16_t oldest )
{
    int64_t k;
        
    for (k = 0; k < Nb; k++) 
    { 
       firOut->FIRred += (int64_t)h[k] * (a1[((int64_t)(oldest + k)) % Nb])/32768LL; 
       firOut->FIRir += (int64_t)h[k] * (a2[((int64_t)(oldest + k)) % Nb])/32768LL; 
    }
}

/*-----------------------------------------------------------------------------
 *  FIR_Filter_Deci16(): FIR filter implementation 16bit numbers
 *              
 *  Parameters: FIR_Filt_Deci16_t **firOut, uint16_t *a1, uint16_t *a2, uint16_t *h,
 *               int16_t Nb, int16_t oldest, int16_t *idx, FFT_Results_t *fftRes  
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
OXY_LIB void FIR_Filter_Deci16( FIR_Filt_Deci16_t **firOut, const uint16_t *a1, const uint16_t *a2, const uint16_t *h, int16_t Nb, int16_t oldest, int16_t *idx, FFT_Results_t *fftRes )
{
    int64_t k;

    *idx = *idx % MAX_FIR_BUFF_SIZ;                                             /* bound the index of the buffer */
        
    for (k = 0; k < Nb; k++) 
    { 
       firOut[*idx]->FIRred += h[k] * (a1[(oldest + k) % Nb])/32768LL; 
       firOut[*idx]->FIRir += h[k] * (a2[(oldest + k) % Nb])/32768LL; 
    }
        
    *idx = ++*idx % MAX_FIR_BUFF_SIZ;
        
    if (*idx == 0)                                                              /* counted to end of buffer (hence reset) */
    {
         getSP02fromFFTdata( firOut, fftRes );
         fftRes->FFTstate = FFT_DATA_READY;
    }
    else
    {
         fftRes->FFTstate = FFT_DATA_COLLECTING;                
    }        
}

/*-----------------------------------------------------------------------------
 *  CollectData4FIR_FFT(): collect the data for the FIR operation runs in 15Hz interrupt
 *              
 *  Parameters: FIR_Filt_Deci16_t **firOut, uint16_t *a1, uint16_t *a2, uint16_t *h, 
 *              int16_t Nb, int16_t *idxD, int16_t *idx, FFT_Results_t *fftRes, const o2Values_t values 
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
OXY_LIB int8_t CollectData4FIR_FFT( FIR_Filt_Deci16_t **firOut, uint16_t *a1, uint16_t *a2, uint16_t *h, int16_t Nb, int16_t *idxD, int16_t *idx, FFT_Results_t *fftRes, const o2Values_t values )
{
        int8_t res;
        
        *idxD = *idxD % MAX_DATA_BUFF_SIZ;                                      /* bound the index of the buffer */
        
        a1[*idxD] = values.a1;
        a2[*idxD] = values.a2;
        h[*idxD] = values.h;
        
        *idxD = ++*idxD % MAX_DATA_BUFF_SIZ;
        
        if (*idxD == 0)                                                         /* counted to end of buffer (hence reset) */
        {
           FIR_Filter_Deci16( firOut, a1, a2, h, Nb, 0, idx, fftRes );
           res = FIR_DATA_READY;
        }
        else
        {
           res = FIR_DATA_COLLECTING;                
        }
        return res;
}



/* this is state engine without interrupt */
/*-----------------------------------------------------------------------------
 *  runOxyAlgo(): run the OXY algorythm
 *              
 *  Parameters: FFT_Results_t *fftRes, FIR_Filt_Deci16_t **firOut, uint16_t *a1,  
 *              uint16_t *a2, uint16_t *h, int16_t Nb, int16_t *idxD, int16_t *idx, 
 *              const o2Values_t values
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
OXY_LIB void runOxyAlgo( FFT_Results_t *fftRes, FIR_Filt_Deci16_t **firOut, uint16_t *a1, uint16_t *a2, uint16_t *h, int16_t Nb, int16_t *idxD, int16_t *idx, const o2Values_t values )
{
        switch (fftRes->oxyState)
        {
                case OXY_INIT:                                                  /* initialise state */
                fftRes->ticks2Now = -1;  
                calculateTick2Now( &fftRes->ticks2Now, &fftRes->lastTickRef );
                fftRes->oxyState = OXY_COLLECT;
                
                case OXY_COLLECT:                                               /* collection state consider using interrupt if time error */
                calculateTick2Now( &fftRes->ticks2Now, &fftRes->lastTickRef );
                if (fftRes->ticks2Now >= 150UL)                                 /* could activate of 150ms interrupt instead */
                {                
                     if (CollectData4FIR_FFT( firOut, a1, a2, h, Nb, idxD, idx, fftRes, values ) == FIR_DATA_READY)
                     {
                         if (fftRes->FFTstate == FFT_DATA_READY)
                         {
                              getSP02fromFFTdata( firOut, fftRes );
                            if ( (abs(fftRes->SP02_last - fftRes->SP02) >= fftRes->SP02_delta) || (abs(fftRes->calcBPM_last - fftRes->calcBPM) >= fftRes->calcBPM_delta) )
                            {
                                fftRes->oxyState = OXY_FINISHED;
                            }
                         }
                     }
                }                        
                break;
                
                case OXY_FINISHED:                                              /* wait for this state then do you're send of information and handshake back to oxy_collect */ 
                fftRes->dly2Now = -1;                                                                           
                calculateTick2Now( &fftRes->dly2Now, &fftRes->lastDlyRef );
                if (fftRes->dly2Now >= 50UL)                                        /* handshake reader did not send ack back so just timeout and look to read new data */
                {
                     fftRes->oxyState = OXY_COLLECT;
                }                        
                break;
                
                default:
                fftRes->oxyState = OXY_INIT;
                break;
        }
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif