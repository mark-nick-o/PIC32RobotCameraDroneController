#include "definitions.h"
#if defined(ML_AI_NEEDED)                                                       /* ==== machine learning is required */
#if defined(RANDOM_FORREST_USED)                                                /* ==== random forrest method ========== */

#ifndef __RAND_FORREST_H_
#define __RAND_FORREST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define RAND_FO_PACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define RAND_FO_PACKED __attribute__((packed))                                     /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define RAND_FO_PACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define RAND_FO_PACKED
#else                                                                           /* for MPLAB PIC32 */
  #define RAND_FO_PACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))
typedef struct RAND_FO_PACKED {
    int16_t rows;
    int16_t cols;
} RF_dim_t;
#else
RAND_FO_PACKED(
typedef struct {
    int16_t rows;
    int16_t cols;
}) RF_dim_t;
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))
typedef struct RAND_FO_PACKED {
    int16_t length;
    float64_t** array;
} RF_var_array_t;
#else
RAND_FO_PACKED(
typedef struct {
    int16_t length;
    float64_t** array;
}) RF_var_array_t;
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))
typedef struct RAND_FO_PACKED {
    int16_t count;
    float64_t* class_labels;
} RF_class_label_struct_t;
#else
RAND_FO_PACKED(
typedef struct {
    int16_t count;
    float64_t* class_labels;
}) RF_class_label_struct_t;
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))
typedef struct RAND_FO_PACKED {
    int16_t index;
    float64_t value;
    float64_t gini;
    RF_var_array_t* two_halves;
} RF_split_params_struct_t;
#else
RAND_FO_PACKED(
typedef struct {
    int16_t index;
    float64_t value;
    float64_t gini;
    RF_var_array_t* two_halves;
}) RF_split_params_struct_t;
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))                                   // struct to keep params for a Random Forest model
typedef struct RAND_FO_PACKED {
    int16_t n_estimators;                                                       // number of trees in a forest
    int16_t max_depth;                                                          // maximum depth of a tree 
    int16_t min_samples_leaf;                                                   // minimum number of data samples at a leaf node
    int16_t max_features;                                                       // number of features considered when calculating the best split
    float64_t sampling_ratio;                                                   // percentage of dataset used to fit a single decision tree at training time
} RF_params_t;
#else
RAND_FO_PACKED(
typedef struct {
    int16_t n_estimators;                                                       // number of trees in a forest
    int16_t max_depth;                                                          // maximum depth of a tree 
    int16_t min_samples_leaf;                                                   // minimum number of data samples at a leaf node
    int16_t max_features;                                                       // number of features considered when calculating the best split
    float64_t sampling_ratio;                                                   // percentage of dataset used to fit a single decision tree at training time
}) RF_params_t;
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))                                   // node in decision tree
typedef struct RAND_FO_PACKED {
    int16_t index;                                                              // for the split_params_struct
    float64_t value;
    RF_var_array_t* two_halves;
    //RF_Node_t* left;                                                            // child nodes if the node is an internal node
    //RF_Node_t* right;
    float64_t left_leaf;                                                        // if the node is a leaf
    float64_t right_leaf;
} RF_Node_t;
#else
RAND_FO_PACKED(
typedef struct {
    int16_t index;                                                              // for the split_params_struct
    float64_t value;
    RF_var_array_t* two_halves;
    //RF_Node_t* left;                                                            // child nodes if the node is an internal node  ( cant call itself )
    //RF_Node_t* right;
    float64_t left_leaf;                                                        // if the node is a leaf
    float64_t right_leaf;
}) RF_Node_t;
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))                                   // node in decision tree
typedef struct RAND_FO_PACKED {
    RF_Node_t node;
    RF_Node_t* left;                                                            // child nodes if the node is an internal node
    RF_Node_t* right;
} RF_Node2_t;
#else
RAND_FO_PACKED(
typedef struct {
    RF_Node_t node;
    RF_Node_t* left;                                                            // child nodes if the node is an internal node
    RF_Node_t* right;
}) RF_Node2_t;
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

#endif
#endif /* requirements were random forrest */