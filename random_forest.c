#include "definitions.h"
#if defined(ML_AI_NEEDED)                                                       /* ==== machine learning is required */
#if defined(RANDOM_FORREST_USED)                                                /* ==== random forrest method ========== */

#ifdef __cplusplus
extern "C" {
#endif
/* ----------------------------------------------------------------------------
   Random Forest for Machine Learning 
   @author andrii dobroshynskyi
   ported by ACP Aviation
   -------------------------------------------------------------------------- */ 
#include "random_forest.h"

//
// split the data into an array of multidimensional arrays for k folds
//
float64_t** RF_k_fold_split(int16_t n, int16_t m, float64_t* dataV, int16_t k_folds)
{
    float64_t** set_of_folds;
    float64_t* current_fold;
    int16_t offset = 0, fold =0,i=0,j=0;
    set_of_folds = (float64_t**) Malloc(n*m*k_folds * sizeof(float64_t));
        
    for(fold=0; fold < k_folds; fold++)
    {
        current_fold = (float64_t*) Malloc(n*m * sizeof(float64_t));
        for(i=0; i < n / k_folds; i++)
        {
            current_fold[i] = dataV[i+offset];
        }
        set_of_folds[fold] = current_fold;
        offset += (n / k_folds);
    }
    return set_of_folds;
}
//
// returns a random sample of the data set that is the fraction ratio of the size of dataset
//
float64_t* RF_subsample(float64_t* dataV, float64_t ratio, int16_t rows, int16_t cols)
{
    int16_t sample_rows = (int16_t)((float64_t)rows * ratio);
    int16_t* indecies;
    int16_t count,i;
    int16_t max,random_index;
    float64_t* sample = (float64_t*) Malloc(sample_rows * sizeof(float64_t) * cols);

    indecies = Malloc(sample_rows * sizeof(int16_t));
    count = 0;
    for(i=0; i < sample_rows; i++) indecies[i] = -1;

    while(count < sample_rows)
    {
        max = rows-1;
        random_index = rand() % (max + 1);
/*        if(!contains_int(indecies, sample_rows, random_index))
        { */
            sample[count] = dataV[random_index];
            indecies[count] = random_index;              // keep track of this index so that there are no duplicate rows
            count++;
      /*  } */
    }
    return sample;
}
//
// build up a forest of random decision trees based on the given parameters and fit to the training data
//
RF_Node2_t** RF_fit_model(float64_t* training_data, RF_params_t params, int16_t rows, int16_t cols)
{
    float64_t* sample;
    RF_Node2_t** trees;
    RF_Node2_t* tree;
    int16_t sample_row_count,i;

    trees = (RF_Node2_t**) malloc(sizeof(RF_Node2_t) * params.n_estimators);
    for(i=0; i < params.n_estimators; i++)
    {
        sample = RF_subsample(training_data, params.sampling_ratio, rows, cols);
        sample_row_count = (int16_t)((float64_t)rows * params.sampling_ratio);
        /*tree = build_tree(sample, params.max_depth, params.min_samples_leaf, params.max_features, sample_row_count, cols); */         // sub sampling
        trees[i] = tree;
    }
    return trees;
}
#ifdef __cplusplus
}
#endif /* __cplusplus */
/* some functions call themselves and will need re-write or implementation elsewhere */
#endif
#endif /* requirements were random forrest */