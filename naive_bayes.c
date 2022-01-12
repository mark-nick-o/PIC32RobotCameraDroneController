#include "definitions.h"
#if defined(ML_AI_NEEDED)                                                       /* ==== machine learning is required */
#if defined(NAIVE_BAYES_USED)                                                   /* ==== naive bayes method ========== */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include "gc_events.h"                                                          /* === global timer counter for random number function initialisation is defined here */
#include "naive_bayes.h"

/*------------------------------------------------------------------------------
 *                          Function Prototypes.
 *----------------------------------------------------------------------------*/
uint16_t NB_CountClass(uint16_t *pClassVec, uint16_t pClassVecLen);
void NB_CalculateAveVar(float64_t **pFeatureVec,float64_t ** pParaTable,uint16_t *pClassVec,uint16_t pRow,uint16_t i,uint16_t j);
float64_t NB_rand_gauss(void);
void NB_AddGaussianWhiteNoise(float64_t **pFeatureVec,uint16_t i,uint16_t pRow);
float64_t** NB_TrainModel(float64_t **pFeatureVec,uint16_t pRow,uint16_t pCol,uint16_t *pClassVec,uint16_t ClassNum);
void NB_create_2D_array(float64_t **arr, uint16_t len, uint16_t width);
void NB_destroy_2D_array(float64_t **arr, uint16_t len, uint16_t width);
void NB_FreeTable(float64_t **pParaTable,uint16_t pRow,uint16_t pCol);
void runNaive_Bayes( uint16_t *TrainSampleClassVec, uint16_t TrainSampleVecLen, uint16_t numOfFeature, float64_t **resultTable );
float64_t NB_mean_of_datastream( float64_t *dStream );
float64_t NB_sd_of_datastream( float64_t *dStream, float64_t mean );
float64_t POIS_U_Random();
int16_t POIS_poisson( int16_t Lambda );
float64_t bayes_calc_prob_like( float64_t value, float64_t mean, float64_t stdev );

/*------------------------------------------------------------------------------
 *                          Function Definitions.
 *----------------------------------------------------------------------------*/
 /*-----------------------------------------------------------------------------
 *      NB_CountClass : count the classes
 *                       
 *
 *  Parameters: uint16_t *pClassVec, uint16_t pClassVecLen     
 *              
 *  Return:     uint16_t
 *----------------------------------------------------------------------------*/
uint16_t NB_CountClass(uint16_t *pClassVec, uint16_t pClassVecLen)
{
        int16_t pVec[100u];
        uint16_t i,j,RetVal;

        memset(pVec,-1,sizeof(pVec));
        
        for(i=0;i!=pClassVecLen;++i)
        {
           pVec[pClassVec[i]]=pClassVec[i];
        }
        for(j=0;j!=sizeof(pVec)/sizeof(int);++j)
        {
                if(-1!=pVec[j])
                   RetVal=++RetVal % UINT16_MAX;
                else
                   return RetVal;
        }
        return RetVal;
}
 /*-----------------------------------------------------------------------------
 *      NB_CalculateAveVar : calculate average 
 *                       
 *  Parameters: float64_t **pFeatureVec,float64_t ** pParaTable,      
 *              uint16_t *pClassVec,uint16_t pRow,uint16_t i,uint16_t j
 *
 *  Return:     uint16_t
 *----------------------------------------------------------------------------*/
#define PARA_NUM 2u
void NB_CalculateAveVar(float64_t **pFeatureVec,float64_t ** pParaTable, uint16_t *pClassVec,uint16_t pRow,uint16_t i,uint16_t j)
{
        int16_t pTempCount=0,k;
        float64_t pTempAve=0;
        float64_t pTempVar=0;
        for(k=0;k!=pRow;++k)
        {
                if(pClassVec[k]==j)
                {
                        ++pTempCount;
                        pTempAve+=pFeatureVec[k][i];
                }
        }
        if(pTempCount)
        {
                pTempAve/=pTempCount;
        }
        else
        {
                pTempAve=0;
        }
        pParaTable[i][PARA_NUM*j]=pTempAve;

        if(pTempCount)
        {
                for(k=0;k!=pRow;++k)
                {
                        if(pClassVec[k]==j)
                        {
                                pTempVar+=(pFeatureVec[k][i]-pTempAve)*(pFeatureVec[k][i]-pTempAve);
                        }
                }
                pParaTable[i][PARA_NUM*j+1]=pTempVar/(float64_t)(pTempCount-1);
        }
        else
        {
                pParaTable[i][PARA_NUM*j+1]=0;
        }        
}
 /*-----------------------------------------------------------------------------
 *      NB_rand_gauss : random gauss 
 *                       
 *  Parameters: void      
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t NB_rand_gauss(void) 
{
  float64_t v1,v2,s;

  do {
    v1 = 2.0f * ((float64_t) rand()/INT32_MAX) - 1.0f;
    v2 = 2.0f * ((float64_t) rand()/INT32_MAX) - 1.0f;
    s = v1*v1 + v2*v2;
  } while ( s >= 1.0f || s == 0.0f);

  return (v1*sqrt(-2.0f * log(s) / s));
}
 /*-----------------------------------------------------------------------------
 *      NB_AddGaussianWhiteNoise : add gaussian white noise 
 *                       
 *  Parameters: float64_t **pFeatureVec,uint16_t i,uint16_t pRow     
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void NB_AddGaussianWhiteNoise(float64_t **pFeatureVec,uint16_t i,uint16_t pRow)
{
        uint16_t m;
        for(m=0;m!=pRow;++m)
        {
                pFeatureVec[m][i]+=NB_rand_gauss();
        }
}
 /*-----------------------------------------------------------------------------
 *      NB_create_2D_array : create 2d array in memory
 *                       
 *  Parameters: float64_t **arr, uint16_t len, uint16_t width     
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void NB_create_2D_array(float64_t **arr, uint16_t len, uint16_t width)
{
        uint16_t i,m,n;
        arr = (float64_t **)Malloc(len*sizeof(float64_t *));

        for(i = 0; i!=len;++i)
                arr[i]=(float64_t *)Malloc(width*sizeof(float64_t));
        for(m=0; m!=len;++m)
                for( n=0;n!=width;++n)
                        arr[m][n]=0;
}
 /*-----------------------------------------------------------------------------
 *      NB_create_2D_array : delete 2d array from memory
 *                       
 *  Parameters: float64_t **arr, uint16_t len, uint16_t width     
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void NB_destroy_2D_array(float64_t **arr, uint16_t len, uint16_t width)
{
        uint16_t i;
        for(i = 0; i!=len;++i)
        {
                Free((char *)arr[i],sizeof(arr[i]));
                arr[i]=NULL;
        }                
        Free((char *)arr,sizeof(arr));
        arr=NULL;
}
 /*-----------------------------------------------------------------------------
 *      NB_TrainModel : train naive bayes model
 *                       
 *  Parameters: float64_t **pFeatureVec,uint16_t pRow,uint16_t pCol,uint16_t      
 *              *pClassVec,uint16_t ClassNum
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
float64_t** NB_TrainModel(float64_t **pFeatureVec,uint16_t pRow,uint16_t pCol,uint16_t *pClassVec,uint16_t ClassNum)
{
        float64_t **pParaTable=NULL;
        int16_t i,j,m;
        NB_create_2D_array(&pParaTable,pCol,PARA_NUM*ClassNum);
        
        for(i=0;i!=pCol;++i)
        {
                for(j=0;j<ClassNum;++j)
                {
                        NB_CalculateAveVar(pFeatureVec,pParaTable,pClassVec,pRow,i,j);
                }
                for(m=0;m!=ClassNum;++m)
                {
                        if(0==pParaTable[i][PARA_NUM*m+1])
                        {
                                NB_AddGaussianWhiteNoise(pFeatureVec,i,pRow);
                                for(j=0;j<ClassNum;++j)
                                {
                                        NB_CalculateAveVar(pFeatureVec,pParaTable,pClassVec,pRow,i,j);
                                }
                                break;
                        }
                }
        }
        return pParaTable;
}
 /*-----------------------------------------------------------------------------
 *      NB_FreeTable : free parameter table from memory
 *                       
 *  Parameters: float64_t **pParaTable,uint16_t pRow,uint16_t pCol      
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void NB_FreeTable(float64_t **pParaTable,uint16_t pRow,uint16_t pCol)
{
        NB_destroy_2D_array(pParaTable,pRow,pCol);
}
 /*-----------------------------------------------------------------------------
 *      runNaive_Bayes : run the algortyhm consider changing to a 
 *                       state engine to iterate (for passive multi-tasking mind)
 *
 *  Parameters: uint16_t *TrainSampleClassVec, uint16_t TrainSampleVecLen,      
 *              uint16_t numOfFeature, float64_t **resultTable 
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void runNaive_Bayes( uint16_t *TrainSampleClassVec, uint16_t TrainSampleVecLen, uint16_t numOfFeature, float64_t **resultTable )
{        
      float64_t **TrainSampleFeatureVec=NULL;                                
      float64_t **ParaTable=NULL;                                                        
      uint16_t ClassNum=0;                                                

      ClassNum = NB_CountClass(TrainSampleClassVec,TrainSampleVecLen);          /* count the classes with the data stream */
      ParaTable = NB_TrainModel(TrainSampleFeatureVec,TrainSampleVecLen,numOfFeature,TrainSampleClassVec,ClassNum); /* train the model */       
      //if we want to print results to a file (in our case we copy to memory resultTable) PrintModelFile(ParaTable,numOfFeature,ClassNum,argv[4]);
      memcpy((void*)&resultTable,(void*)&ParaTable,sizeof(ParaTable));          /* copy the naswer to the outgoing array */
      //FreeFile(TrainSampleFeatureVec, TrainSampleVecLen, numOfFeature, TrainSampleClassVec); we dont need to atm as we are working on stream data passed.
      NB_FreeTable(ParaTable,numOfFeature,2*ClassNum);                          /* free the result table from memory i have left this as it could be used to write to a file, otherwise we can just use this pointer passed rather than malloc */
}

 /*-----------------------------------------------------------------------------
 *  NB_mean_of_datastream : calculate the mean of the datastream
 *
 *  Parameters: float64_t *dStream      
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t NB_mean_of_datastream( float64_t *dStream )
{
    float64_t tot = 0.0f;
    int32_t i;
    int32_t elements = sizeof(dStream) / sizeof(float64_t);

    for (i = 0; i < elements; ++i)
    {
       tot = tot + dStream[i];
    }
    return (tot / elements);
}

 /*-----------------------------------------------------------------------------
 *  NB_sd_of_datastream : calculate the standard deviation of the datastream
 *
 *  Parameters: float64_t *dStream, float64_t mean      
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t NB_sd_of_datastream( float64_t *dStream, float64_t mean )
{
    float64_t SD = 0.0f;
    int32_t i;
    int32_t elements = sizeof(dStream) / sizeof(float64_t);

    for (i = 0; i < elements; ++i)
    {
       SD = SD + pow((dStream[i] - mean), 2.0f);
    }
    return sqrt(SD / elements);
}
 /*-----------------------------------------------------------------------------
 *  POIS_U_Random : generates a 0 ~ Random number between 1
 *
 *  Parameters:       
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t POIS_U_Random()
{
  float64_t f;
  srand((uint32_t)(g_timer5_counter % UINT32_MAX));
  f = (float64_t) (rand () % INT32_MAX );
  return (f / INT32_MAX);
}
 /*-----------------------------------------------------------------------------
 *  POIS_poisson : generates a random number with a Poisson distribution. 
 *                 Lamda is the average number
 *  Parameters: int16_t Lambda      
 *
 *  Return: int16_t
 *----------------------------------------------------------------------------*/
int16_t POIS_poisson( int16_t Lambda )
{
  int16_t K = 0;
  float64_t u;
  float64_t P = 1.0f;
  float64_t l = exp(-Lambda);                                                   /* it is defined as long double for precision, and exp (-Lambda) is a decimal near 0 */  
  while (P >= l)
  {
    u = POIS_U_Random();
    P *= u;
    ++K;
  }
  return(K-1);
}

 /*-----------------------------------------------------------------------------
 *  bayes_calc_prob_like : using bayes calculate the probability of likelyhood
 *
 *  Parameters: float64_t value, float64_t mean, float64_t stdev      
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t bayes_calc_prob_like( float64_t value, float64_t mean, float64_t stdev )
{
    static const float64_t inv_sqrt_2pi = 0.3989422804014327f;
    float64_t a = (value - mean) / stdev;
    return (inv_sqrt_2pi / stdev * exp(-0.5f * a * a));
}
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
#endif /* requirements were naive bayes */