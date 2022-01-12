#ifndef _DALI_CODES_H_
#define DALI__DALI_CODES_H_

#ifdef __cplusplus
 extern "C" {
#endif

//  dali_codes.h - dali "native" operations.
//
//  Ported by (C) 2020 A C P Avaiation Walkerburn Scotland
//
//----- Pre-defined response on query
#define DALI_YES                                    0xFFu
#define DALI_MASK                                   0xFFu
#define DALI_DONT_CHANGE                            0xFFu

//----- No command, setting or query
#define DALI_NO_DALI_COMMAND_CODE                        0x00u

//----- Arc power control commands
#define DALI_IMMEDIATE_OFF                               0x00u
#define DALI_UP_200MS                                    0x01u
#define DALI_DOWN_200MS                                  0x02u
#define DALI_STEP_UP                                     0x03u
#define DALI_STEP_DOWN                                   0x04u
#define DALI_RECALL_MAX_LEVEL                            0x05u
#define DALI_RECALL_MIN_LEVEL                            0x06u
#define DALI_STEP_DOWN_AND_OFF                           0x07u
#define DALI_ON_AND_STEP_UP                              0x08u
#define DALI_GO_TO_SCENE                                 0x10u                  // 'd_data' => "scene"

//----- General configuration commands
    //- Need to be received twice
#define DALI_RESET                                       0x20u
#define DALI_STORE_ACTUAL_DIM_LEVEL_IN_DTR               0x21u                  // "actual_dim_level" => 'dtr'

//----- Arc power parameters settings
    //- Need to be received twice
#define DALI_STORE_THE_DTR_AS_MAX_LEVEL                  0x2Au                  // 'dtr' => "max_level"
#define DALI_STORE_THE_DTR_AS_MIN_LEVEL                  0x2Bu                  // 'dtr' => "min_level"
#define DALI_STORE_THE_DTR_AS_SYSTEM_FAILURE_LEVEL       0x2Cu                  // 'dtr' => "system_failure_level"
#define DALI_STORE_THE_DTR_AS_POWER_ON_LEVEL             0x2Du                  // 'dtr' => "power_on_level"
#define DALI_STORE_THE_DTR_AS_FADE_TIME                  0x2Eu                  // 'dtr' => "fade_time"
#define DALI_STORE_THE_DTR_AS_FADE_RATE                  0x2Fu                  // 'dtr' => "fade_rate"
#define DALI_STORE_THE_DTR_AS_SCENE                      0x40u                  // 'd_data' => "scene", 'dtr' => new "scene_level"

//----- System parameters settings
    //- Need to be received twice
#define DALI_REMOVE_FROM_SCENE                           0x50u                  // 'd_data' => "scene"
#define DALI_ADD_TO_GROUP                                0x60u                  // 'd_data' => "group"
#define DALI_REMOVE_FROM_GROUP                           0x70u                  // 'd_data' => "group"
#define DALI_STORE_DTR_AS_SHORT_ADDRESS                  0x80u                  // 'dtr' => "short_address"

//----- Queries related to status information
#define DALI_QUERY_STATUS                                0x90u                  // "DALI_YES" => 'd_data' else nothing
#define DALI_QUERY_BALLAST                               0x91u                  // "DALI_YES" => 'd_data' else nothing
#define DALI_QUERY_LAMP_FAILURE                          0x92u                  // "DALI_YES" => 'd_data' else nothing
#define DALI_QUERY_LAMP_POWER_ON                         0x93u                  // "DALI_YES" => 'd_data' else nothing
#define DALI_QUERY_LIMIT_ERROR                           0x94u                  // "DALI_YES" => 'd_data' else nothing
#define DALI_QUERY_RESET_STATE                           0x95u                  // "DALI_YES" => 'd_data' else nothing
#define DALI_QUERY_MISSING_SHORT_ADDRESS                 0x96u                  // "DALI_YES" => 'd_data' else nothing
#define DALI_QUERY_VERSION_NUMBER                        0x97u                  // "version_number" => 'd_data'
#define DALI_QUERY_CONTENT_DTR                           0x98u                  // 'dtr' => 'd_data'
#define DALI_QUERY_DEVICE_TYPE                           0x99u                  // "device_type" => 'd_data'
#define DALI_QUERY_PHYSICAL_MINIMUM_LEVEL                0x9Au                  // "physical_minimum_level" => 'd_data'
#define DALI_QUERY_POWER_FAILURE                         0x9Bu                  // ... response (c.f. DALI standard) => 'd_data'

//----- Queries related to arc power parameters settings
#define DALI_QUERY_ACTUAL_LEVEL                          0xA0u                   // "actual_dim_level" or "DALI_MASK" => 'd_data'
#define DALI_QUERY_MAX_LEVEL                             0xA1u                   // "max_level" => 'd_data'
#define DALI_QUERY_MIN_LEVEL                             0xA2u                   // "min_level" => 'd_data'
#define DALI_QUERY_POWER_ON_LEVEL                        0xA3u                   // "power_on_level" => 'd_data'
#define DALI_QUERY_SYSTEM_FAILURE_LEVEL                  0xA4u                   // "system_failure_level" => 'd_data'
#define DALI_QUERY_FADE                                  0xA5u                   // "fade_time,fade_rate" => 'd_data'

//----- Queries related to system parameters settings
#define DALI_QUERY_SCENE_LEVEL                           0xB0u                  // 'd_data' => "scene", "scene_level" => 'd_data'
#define DALI_QUERY_GROUPS_0_7                            0xC0u                  // "group_0_7" => 'd_data'
#define DALI_QUERY_GROUPS_8_15                           0xC1u                  // "group_8_15" => 'd_data'
#define DALI_QUERY_RANDOM_ADDRESS_H                      0xC2u                  // "random_address.h" => 'd_data'
#define DALI_QUERY_RANDOM_ADDRESS_M                      0xC3u                  // "random_address.m" => 'd_data'
#define DALI_QUERY_RANDOM_ADDRESS_L                      0xC4u                  // "random_address.l" => 'd_data'

//----- Use of extended commands
#define DALI_QUERY_APPLICATION_EXTENTED_COMMAND          0xE0u                  // for "device_type" = 0, extended commands are not used

//----- Extended commands - Terminate special processes
#define DALI_TERMINATE                                   0xA1u

//----- Extended commands - Download information to the dtr
#define DALI_DATA_TRANSFER_REGISTER                      0xA3u                  // received direct data => 'dtr'

//----- Extended commands - Addressing commands
    //- Need to be received twice
#define DALI_INITIALISE                                  0xA5u                  // 'd_data' => "reaction_of_ballasts"
#define DALI_RANDOMISE                                   0xA7u
#define DALI_COMPARE                                     0xA9u
#define DALI_WITHDRAW                                    0xABu
#define DALI_SEARCHADDRH                                 0xB1u                  // 'd_data' => "search_address.h"
#define DALI_SEARCHADDRM                                 0xB3u                  // 'd_data' => "search_address.m"
#define DALI_SEARCHADDRL                                 0xB5u                  // 'd_data' => "search_address.l"
#define DALI_PROGRAM_SHORT_ADDRESS                       0xB7u                  // 'd_data' => "short_address"
#define DALI_VERIFY_SHORT_ADDRESS                        0xB9u                  // 'd_data' => "short_address", "DALI_YES" => 'd_data' if equal
#define DALI_QUERY_SHORT_ADDRESS                         0xBBu                  // "short_address" or "DALI_MASK" => 'd_data'
#define DALI_PHYSICAL_SELECTION                          0xBDu

//----- Extended commands - Special extended command
#define DALI_ENABLE_DEVICE_TYPE_X                        0xC1u                   // for "device_type" = 0, extended commands are not used
                                                                                // 'd_data' => "device_type"
                                                            
#define DALI_ERR_NO_ANSWER -100
#define DALI_ERR_INVALID_FRAME -101

#ifdef __cplusplus
}
#endif

#endif