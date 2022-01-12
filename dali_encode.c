//
//  dali_encode.h - Dali protocol "native" operations.
//
// * Ported by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#include <stdint.h>
#include "dali_encode.h"

#define MAX_COMMAND_LENGTH 48u
#define MAX_VALUE_LENGTH  5u
#define _PROGMEM                                                                // from the library
#define FALSE 0U
#define TRUE 1U
#define countof(array) (sizeof(array) / sizeof((array)[0u]))                    /* Determine the number of elements in an array. */

const char identifier_message[] _PROGMEM = "DALI Master\r\n";
const char group_postfix[] _PROGMEM = "_g";
const char broadcast_postfix[] _PROGMEM = "_b";

//special commands
const char command_randomize[] _PROGMEM = "randomize";
const char command_initialize[] _PROGMEM = "initialize";
const char command_dtr[] _PROGMEM = "dtr";
const char command_terminate[] _PROGMEM = "terminate";
const char command_compare[] _PROGMEM = "compare";
const char command_withdraw[] _PROGMEM = "withdraw";
const char command_search_addrh[] _PROGMEM = "search_address_h";
const char command_search_addrm[] _PROGMEM = "search_address_m";
const char command_search_addrl[] _PROGMEM = "search_address_l";
const char command_programm_short_address[] _PROGMEM = "programm_short_address";
const char command_verify_short_address[] _PROGMEM = "verify_short_address";
const char command_query_short_address[] _PROGMEM = "query_short_address";
const char command_physical_selection[] _PROGMEM = "physical_selection";
const char command_enable_device_type[] _PROGMEM = "enable_device_type";

//set level directly
const char command_arc[] _PROGMEM = "arc";
const char command_arc_group[] _PROGMEM = "arc_g";
const char command_arc_broadcast[] _PROGMEM = "arc_b";

//arc commands
const char command_off[] _PROGMEM = "off";
const char command_up[] _PROGMEM = "up";
const char command_down[] _PROGMEM = "down";
const char command_step_up[] _PROGMEM = "step_up";
const char command_step_down[] _PROGMEM = "step_down";
const char command_max_level[] _PROGMEM = "recall_max_level";
const char command_min_level[] _PROGMEM = "recall_min_level";
const char command_step_off[] _PROGMEM = "step_down_and_off";
const char command_step_on[] _PROGMEM = "on_and_step_up";
const char command_reset[] _PROGMEM = "reset";
const char command_store_level_dtr[] _PROGMEM = "store_level_dtr";
const char command_store_dtr_max[] _PROGMEM = "store_dtr_as_max_level";
const char command_store_dtr_min[] _PROGMEM = "store_dtr_as_min_level";
const char command_store_dtr_system_failure[] _PROGMEM = "store_dtr_as_system_failure_level";
const char command_store_dtr_power_on[] _PROGMEM = "store_dtr_as_power_on_level";
const char command_store_dtr_fade_time[] _PROGMEM = "store_dtr_as_fade_time";
const char command_store_dtr_fade_rate[] _PROGMEM = "store_dtr_as_fade_rate";
const char command_store_dtr_short_address[] _PROGMEM = "store_dtr_as_short_address";

//query
const char command_query_status[] _PROGMEM = "query_status";
const char command_query_ballast[] _PROGMEM = "query_ballast";
const char command_query_lamp_failure[] _PROGMEM = "query_lamp_failure";
const char command_query_lamp_power_on[] _PROGMEM = "query_power";
const char command_query_limit_error[] _PROGMEM = "query_limit_error";
const char command_query_reset_state[] _PROGMEM = "query_reset_state";
const char command_query_missing_short_address[] _PROGMEM = "query_missing_short_address";
const char command_query_version_number[] _PROGMEM = "query_version_number";
const char command_query_content_dtr[] _PROGMEM = "query_content_dtr";
const char command_query_device_type[] _PROGMEM = "query_device_type";
const char command_query_physical_minimum_level[] _PROGMEM = "query_physical_minimum_level";
const char command_query_power_failure[] _PROGMEM = "query_power_failure";
const char command_query_actual_level[] _PROGMEM = "query_actual_level";
const char command_query_max_level[] _PROGMEM = "query_max_level";
const char command_query_min_level[] _PROGMEM = "query_min_level";
const char command_query_power_on_level[] _PROGMEM = "query_power_on_level";
const char command_query_system_failure_level[] _PROGMEM = "query_system_failure_level";
const char command_query_fade_time[] _PROGMEM = "query_fade_time";
const char command_query_groups_0_7[] _PROGMEM = "query_groups_0_7";
const char command_query_groups_8_15[] _PROGMEM = "query_groups_8_15";
const char command_query_random_address_h[] _PROGMEM = "query_random_address_h";
const char command_query_random_address_m[] _PROGMEM = "query_random_address_m";
const char command_query_random_address_l[] _PROGMEM = "query_random_address_l";

//commands with param
const char command_goto_scene[] _PROGMEM = "go_to_scene";
const char command_remove_from_scene[] _PROGMEM = "remove_scene";
const char command_add_to_group[] _PROGMEM = "add_group";
const char command_remove_from_group[] _PROGMEM = "remove_group";
const char command_store_dtr_scene[] _PROGMEM = "store_dtr_scene";

//query with param
const char command_query_scene_level[] _PROGMEM = "query_scene_level";
const char NACK[] _PROGMEM = "NACK";
const char ACK[] _PROGMEM = "ACK";

const char internal_command_save_current_as_default[] = "save_current_as_default";
const char internal_command_clear_default_values[] = "clear_default_values";

typedef struct key_value_mode
{
        const char* key;
        byte value;
        byte mode;
} key_value_mode;

typedef struct key_special_mode
{
        const char* key;
        special_command_type special;
        byte mode;
} key_special_mode;

#define COUNT_COMMANDS 41u

const key_value_mode command_list[] = {
        {command_off, DALI_IMMEDIATE_OFF, _MODE_SIMPLE_},
        {command_up, DALI_UP_200MS, _MODE_SIMPLE_},
        {command_down, DALI_DOWN_200MS, _MODE_SIMPLE_},
        {command_step_up, DALI_STEP_UP, _MODE_SIMPLE_},
        {command_step_down, DALI_STEP_DOWN, _MODE_SIMPLE_},
        {command_max_level, DALI_RECALL_MAX_LEVEL, _MODE_SIMPLE_},
        {command_min_level, DALI_RECALL_MIN_LEVEL, _MODE_SIMPLE_},
        {command_step_off, DALI_STEP_DOWN_AND_OFF, _MODE_SIMPLE_},
        {command_step_on, DALI_ON_AND_STEP_UP, _MODE_SIMPLE_},
        {command_reset, DALI_RESET, _MODE_REPEAT_TWICE_},

        //dtr commands
        {command_store_level_dtr, DALI_STORE_ACTUAL_DIM_LEVEL_IN_DTR, _MODE_REPEAT_TWICE_},
        {command_store_dtr_max, DALI_STORE_THE_DTR_AS_MAX_LEVEL, _MODE_REPEAT_TWICE_},
        {command_store_dtr_min, DALI_STORE_THE_DTR_AS_MIN_LEVEL, _MODE_REPEAT_TWICE_},
        {command_store_dtr_system_failure, DALI_STORE_THE_DTR_AS_SYSTEM_FAILURE_LEVEL, _MODE_REPEAT_TWICE_},
        {command_store_dtr_power_on, DALI_STORE_THE_DTR_AS_POWER_ON_LEVEL, _MODE_REPEAT_TWICE_},
        {command_store_dtr_fade_time, DALI_STORE_THE_DTR_AS_FADE_TIME, _MODE_REPEAT_TWICE_},
        {command_store_dtr_fade_rate, DALI_STORE_THE_DTR_AS_FADE_RATE, _MODE_REPEAT_TWICE_},
        {command_store_dtr_short_address, DALI_STORE_DTR_AS_SHORT_ADDRESS, _MODE_REPEAT_TWICE_},

        //query
        {command_query_status, DALI_QUERY_STATUS, _MODE_QUERY_},
        {command_query_ballast, DALI_QUERY_BALLAST, _MODE_QUERY_},
        {command_query_lamp_failure, DALI_QUERY_LAMP_FAILURE, _MODE_QUERY_},
         {command_query_lamp_power_on, DALI_QUERY_LAMP_POWER_ON, _MODE_QUERY_},
        {command_query_limit_error, DALI_QUERY_LIMIT_ERROR, _MODE_QUERY_},
        {command_query_reset_state, DALI_QUERY_RESET_STATE, _MODE_QUERY_},
        {command_query_missing_short_address, DALI_QUERY_MISSING_SHORT_ADDRESS, _MODE_QUERY_},
        {command_query_version_number, DALI_QUERY_VERSION_NUMBER, _MODE_QUERY_},
        {command_query_content_dtr, DALI_QUERY_CONTENT_DTR, _MODE_QUERY_},
        {command_query_device_type, DALI_QUERY_DEVICE_TYPE, _MODE_QUERY_},
        {command_query_physical_minimum_level, DALI_QUERY_PHYSICAL_MINIMUM_LEVEL, _MODE_QUERY_},
        {command_query_power_failure, DALI_QUERY_POWER_FAILURE, _MODE_QUERY_},
        {command_query_actual_level, DALI_QUERY_ACTUAL_LEVEL, _MODE_QUERY_},
        {command_query_max_level, DALI_QUERY_MAX_LEVEL, _MODE_QUERY_},
        {command_query_min_level, DALI_QUERY_MIN_LEVEL, _MODE_QUERY_},
        {command_query_power_on_level, DALI_QUERY_POWER_ON_LEVEL, _MODE_QUERY_},
        {command_query_system_failure_level, DALI_QUERY_SYSTEM_FAILURE_LEVEL, _MODE_QUERY_},
        {command_query_fade_time, DALI_QUERY_FADE, _MODE_QUERY_},
        {command_query_groups_0_7, DALI_QUERY_GROUPS_0_7, _MODE_QUERY_},
        {command_query_groups_8_15, DALI_QUERY_GROUPS_8_15, _MODE_QUERY_},
        {command_query_random_address_h, DALI_QUERY_RANDOM_ADDRESS_H, _MODE_QUERY_},
        {command_query_random_address_m, DALI_QUERY_RANDOM_ADDRESS_M, _MODE_QUERY_},
        {command_query_random_address_l, DALI_QUERY_RANDOM_ADDRESS_L, _MODE_QUERY_},
        };

#define COUNT_COMMANDS_WITH_PARAM 6

const key_value_mode command_with_param_list[] = {
        {command_goto_scene, DALI_GO_TO_SCENE, _MODE_SIMPLE_},
    {command_remove_from_scene, DALI_REMOVE_FROM_SCENE, _MODE_REPEAT_TWICE_},
    {command_add_to_group, DALI_ADD_TO_GROUP, _MODE_REPEAT_TWICE_},
    {command_remove_from_group, DALI_REMOVE_FROM_GROUP, _MODE_REPEAT_TWICE_},
    {command_store_dtr_scene, DALI_STORE_THE_DTR_AS_SCENE, _MODE_REPEAT_TWICE_},
    {command_query_scene_level, DALI_QUERY_SCENE_LEVEL, _MODE_QUERY_}
        };

#define COUNT_SPECIAL_COMMANDS 14

const key_special_mode special_command_list[] = {
        {command_initialize, INITIALIZE, _MODE_REPEAT_TWICE_},
        {command_randomize, RANDOMIZE, _MODE_REPEAT_TWICE_},
        {command_dtr, STORE_DTR, _MODE_SIMPLE_},
        {command_terminate, TERMINATE, _MODE_SIMPLE_},
        {command_compare, COMPARE, _MODE_QUERY_},
        {command_withdraw, WITHDRAW, _MODE_SIMPLE_},
        {command_search_addrh, SEARCH_ADDRESS_H, _MODE_SIMPLE_},
        {command_search_addrm, SEARCH_ADDRESS_M, _MODE_SIMPLE_},
        {command_search_addrl, SEARCH_ADDRESS_L, _MODE_SIMPLE_},
        {command_programm_short_address, PROGRAM_SHORT_ADDRESS, _MODE_SIMPLE_},
        {command_verify_short_address, VERIFY_SHORT_ADDRESS, _MODE_QUERY_},
        {command_query_short_address, QUERY_SHORT_ADDRESS, _MODE_QUERY_},
        {command_physical_selection, PHYSICAL_SELECTION, _MODE_SIMPLE_},
        {command_enable_device_type, ENABLE_DEVICE_TYPE, _MODE_SIMPLE_}
        };

uint8_t arc_device[64u];
uint8_t arc_group[16u];

void dali_init_device();
void dali_init_group();
int16_t dali_slave_direct_arc(word *output, byte address, byte brightness);
int16_t dali_group_direct_arc(word *output, byte address, byte brightness);
int16_t dali_broadcast_direct_arc(word* output, byte brightness);
int16_t dali_slave_command(word *output, byte address, byte command);
int16_t dali_group_command(word *output, byte address, byte command);
int16_t dali_broadcast_command(word* output, byte command);
int16_t dali_slave_command_with_param(word *output, byte address, byte command, byte param);
int16_t dali_group_command_with_param(word *output, byte address, byte command, byte param);
int16_t dali_broadcast_command_with_param(word* output, byte command, byte param);
int16_t dali_special_command(word *output, special_command_type command, byte dataVal);

void dali_init_device()
{
   uint8_t numDev;

   for (numDev=0; numDev<=(countof(arc_device)-1U); numDev++ )
   {
       arc_device[numDev] = 255u;
   }
}
/*-----------------------------------------------------------------------------
 *      dali_init_group():  initialise group
 *
 *  Parameters: 
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void dali_init_group()
{
   uint8_t numG;

   for (numG=0; numG<=(countof(arc_group)-1U); numG++ )
   {
       arc_group[numG] = 255u;
   }
}
/*-----------------------------------------------------------------------------
 *      dali_slave_direct_arc():  control brightness of direct arc
 *
 *  Parameters: word *output, byte address, byte brightness
 *
 *  Return:     int16_t _ERR_OK_ or  _ERR_WRONG_ADDRESS_
 *----------------------------------------------------------------------------*/
int16_t dali_slave_direct_arc(word *output, byte address, byte brightness)
{
        if(address > 63u)
        {
                return _ERR_WRONG_ADDRESS_;
        }
        brightness = brightness > 254u ? 254u : brightness;                       //direct arc value can be 254 maximum
        *output = ((address << 1u) << 8u) + brightness;                           //Frame: 0AAAAAA0 BBBBBBBB
        return _ERR_OK_;
}
/*-----------------------------------------------------------------------------
 *      dali_group_direct_arc():  control brightness of group
 *
 *  Parameters: word *output, byte address, byte brightness
 *
 *  Return:     int16_t _ERR_OK_ or  _ERR_WRONG_ADDRESS_
 *----------------------------------------------------------------------------*/
int16_t dali_group_direct_arc(word *output, byte address, byte brightness)
{
        if(address > 15u)
        {
                return _ERR_WRONG_ADDRESS_;
        }
        brightness = brightness > 254u ? 254u : brightness;                       //direct arc value can be 254 maximum

        *output = ((0x80u + (address << 1u)) << 8u) + brightness;                  //Frame: 100AAAA0 BBBBBBBB
        return _ERR_OK_;
}
/*-----------------------------------------------------------------------------
 *      dali_broadcast_direct_arc():  control brightness broadcast to all
 *
 *  Parameters: word *output, byte address, byte brightness
 *
 *  Return:     int16_t _ERR_OK_ or  _ERR_WRONG_ADDRESS_
 *----------------------------------------------------------------------------*/
int16_t dali_broadcast_direct_arc(word *output, byte brightness)
{
        brightness = brightness > 254u ? 254u : brightness;                       //direct arc value can be 254 maximum
        *output = 0xFE00 + brightness;                                          //Frame: 11111110 BBBBBBBB
        return _ERR_OK_;
}
/*-----------------------------------------------------------------------------
 *      dali_slave_command():  send command to slave
 *
 *  Parameters: word *output, byte address, byte brightness
 *
 *  Return:     int16_t _ERR_OK_ or  _ERR_WRONG_ADDRESS_ or  _ERR_WRONG_COMMAND_
 *              _ERR_RESERVED_COMMAND_
 *----------------------------------------------------------------------------*/
int16_t dali_slave_command(word *output, byte address, byte command)
{
        if(address > 63u)
                return _ERR_WRONG_ADDRESS_;
        if(!((command & 0xF0u) ^ 0x30u))                                          //command 0011xxxx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xF0u) ^ 0x80u))                                          //command 1000xxxx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xFCu) ^ 0x9Cu))                                          //command 100111xx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xF0u) ^ 0xA0u))                                          //command 1010xxxx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xE0u) ^ 0xC0u))                                          //command 110xxxxx is reserved
                return _ERR_RESERVED_COMMAND_;

        if((command == DALI_GO_TO_SCENE) ||
        (command == DALI_STORE_THE_DTR_AS_SCENE) ||
        (command == DALI_REMOVE_FROM_SCENE) ||
        (command == DALI_ADD_TO_GROUP) ||
        (command == DALI_REMOVE_FROM_GROUP) ||
        (command == DALI_QUERY_SCENE_LEVEL))
                return _ERR_WRONG_COMMAND_;

        *output = ((0x01u + (address << 1u)) << 8u) + command;                     //Frame: 0AAAAAA1 CCCCCCCC
        return _ERR_OK_;
}
/*-----------------------------------------------------------------------------
 *      dali_group_command():  send command to group
 *
 *  Parameters: word *output, byte address, byte brightness
 *
 *  Return:     int16_t _ERR_OK_ or  _ERR_WRONG_ADDRESS_ or  _ERR_WRONG_COMMAND_
 *              _ERR_RESERVED_COMMAND_
 *----------------------------------------------------------------------------*/
int16_t dali_group_command(word *output, byte address, byte command)
{
        if(address > 15u)
                return _ERR_WRONG_ADDRESS_;
        if(!((command & 0xF0u) ^ 0x30u))                                          //command 0011xxxx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xF0u) ^ 0x80u))                                          //command 1000xxxx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xFCu) ^ 0x9Cu))                                          //command 100111xx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xF0u) ^ 0xA0u))                                          //command 1010xxxx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xE0u) ^ 0xC0u))                                          //command 110xxxxx is reserved
                return _ERR_RESERVED_COMMAND_;

        if((command == DALI_GO_TO_SCENE) ||
        (command == DALI_STORE_THE_DTR_AS_SCENE) ||
        (command == DALI_REMOVE_FROM_SCENE) ||
        (command == DALI_ADD_TO_GROUP) ||
        (command == DALI_REMOVE_FROM_GROUP) ||
        (command == DALI_QUERY_SCENE_LEVEL))
                return _ERR_WRONG_COMMAND_;

        *output = ((0x81u + (address << 1u)) << 8u) + command;                     //Frame: 100AAAA1 CCCCCCCC
        return _ERR_OK_;
}
/*-----------------------------------------------------------------------------
 *      dali_broadcast_command():  broadcast a command to all
 *
 *  Parameters: word *output, byte address, byte brightness
 *
 *  Return:     int16_t _ERR_OK_ or  _ERR_WRONG_ADDRESS_ or  _ERR_WRONG_COMMAND_
 *              _ERR_RESERVED_COMMAND_
 *----------------------------------------------------------------------------*/
int16_t dali_broadcast_command(word *output, byte command)
{
        if(!((command & 0xF0u) ^ 0x30u))                                          //command 0011xxxx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xF0u) ^ 0x80u))                                          //command 1000xxxx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xFCu) ^ 0x9Cu))                                          //command 100111xx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xF0u) ^ 0xA0u))                                          //command 1010xxxx is reserved
                return _ERR_RESERVED_COMMAND_;
        if(!((command & 0xE0u) ^ 0xC0u))                                          //command 110xxxxx is reserved
                return _ERR_RESERVED_COMMAND_;

        if((command == DALI_GO_TO_SCENE) ||
        (command == DALI_STORE_THE_DTR_AS_SCENE) ||
        (command == DALI_REMOVE_FROM_SCENE) ||
        (command == DALI_ADD_TO_GROUP) ||
        (command == DALI_REMOVE_FROM_GROUP) ||
        (command == DALI_QUERY_SCENE_LEVEL))
                return _ERR_WRONG_COMMAND_;
        *output = 0xFF00LU + command;                                             //Frame: 11111111 CCCCCCCC
        return _ERR_OK_;
}
/*-----------------------------------------------------------------------------
 *      dali_slave_command_with_param():  send command and parameter to a slave
 *
 *  Parameters: word *output, byte address, byte command, byte param
 *
 *  Return:     int16_t _ERR_OK_ or  _ERR_WRONG_ADDRESS_ 
 *              
 *----------------------------------------------------------------------------*/
int16_t dali_slave_command_with_param(word *output, byte address, byte command, byte param)
{
        if(address > 63u)
                return _ERR_WRONG_ADDRESS_;
        if(param > 15u)
                return _ERR_WRONG_COMMAND_;

        if((command == DALI_GO_TO_SCENE) ||
        (command == DALI_STORE_THE_DTR_AS_SCENE) ||
        (command == DALI_REMOVE_FROM_SCENE) ||
        (command == DALI_ADD_TO_GROUP) ||
        (command == DALI_REMOVE_FROM_GROUP) ||
        (command == DALI_QUERY_SCENE_LEVEL))
        {
                *output = ((0x01u + (address << 1u)) << 8u) + command + param;     //Frame: 0AAAAAA1 CCCCPPPP
                return _ERR_OK_;
        }
        else
                return _ERR_WRONG_COMMAND_;
}
/*-----------------------------------------------------------------------------
 *      dali_group_command_with_param():  send command and parameter to a group
 *
 *  Parameters: word *output, byte address, byte command, byte param
 *
 *  Return:     int16_t _ERR_OK_ or  _ERR_WRONG_ADDRESS_ or  _ERR_WRONG_COMMAND_
 *             
 *----------------------------------------------------------------------------*/
int16_t dali_group_command_with_param(word *output, byte address, byte command, byte param)
{
        if(address > 15u)
                return _ERR_WRONG_ADDRESS_;
        if(param > 15u)
                return _ERR_WRONG_COMMAND_;

        if((command == DALI_GO_TO_SCENE) ||
        (command == DALI_STORE_THE_DTR_AS_SCENE) ||
        (command == DALI_REMOVE_FROM_SCENE) ||
        (command == DALI_ADD_TO_GROUP) ||
        (command == DALI_REMOVE_FROM_GROUP) ||
        (command == DALI_QUERY_SCENE_LEVEL))
        {
                *output = ((0x81u + (address << 1u)) << 8u) + command + param;     //Frame: 100AAAA1 CCCCPPPP
                return _ERR_OK_;
        }
        else
                return _ERR_WRONG_COMMAND_;
}
/*-----------------------------------------------------------------------------
 *      dali_broadcast_command_with_param():  broadcast command and parameter to all
 *
 *  Parameters: word *output, byte address, byte command, byte param
 *
 *  Return:     int16_t _ERR_OK_ or  _ERR_WRONG_ADDRESS_ or  _ERR_WRONG_COMMAND_
 *             
 *----------------------------------------------------------------------------*/
int16_t dali_broadcast_command_with_param(word *output, byte command, byte param)
{
        if(param > 15u)
                return _ERR_WRONG_COMMAND_;

        if((command == DALI_GO_TO_SCENE) ||
        (command == DALI_STORE_THE_DTR_AS_SCENE) ||
        (command == DALI_REMOVE_FROM_SCENE) ||
        (command == DALI_ADD_TO_GROUP) ||
        (command == DALI_REMOVE_FROM_GROUP) ||
        (command == DALI_QUERY_SCENE_LEVEL))
        {
                *output = 0xFF00u + command + param;                             //Frame: 100AAAA1 CCCCPPPP
                return _ERR_OK_;
        }
        else
                return _ERR_WRONG_COMMAND_;
}
/*-----------------------------------------------------------------------------
 *      dali_special_command():  send special command and value
 *
 *  Parameters: word *output, special_command_type command, byte dataVal
 *
 *  Return:     int16_t _ERR_OK_ or _ERR_WRONG_COMMAND_
 *             
 *----------------------------------------------------------------------------*/
int16_t dali_special_command(word *output, special_command_type command, byte dataVal)
{
        switch(command)
        {
        case INITIALIZE:
                *output = (DALI_INITIALISE << 8u) + dataVal;                     //Frame: 1010 0101 xxxx xxxx
                break;
        case RANDOMIZE:
                *output = DALI_RANDOMISE << 8u;                                  //1010 0111 0000 0000
                break;
        case TERMINATE:
                *output = DALI_TERMINATE << 8u;                                  //1010 0001 0000 0000
                break;
        case STORE_DTR:
                *output = (DALI_DATA_TRANSFER_REGISTER << 8u) + dataVal;
                break;

        case PROGRAM_SHORT_ADDRESS:
                *output = (DALI_PROGRAM_SHORT_ADDRESS << 8u) + 0x01u + ((0x3Fu & dataVal) << 1u); //1011 0111 0xxx xxx1
                break;

        case VERIFY_SHORT_ADDRESS:
                *output = (DALI_VERIFY_SHORT_ADDRESS << 8u) + 0x01u + ((0x3Fu & dataVal) << 1u);
                break;

        case QUERY_SHORT_ADDRESS:
                *output = DALI_QUERY_SHORT_ADDRESS << 8u;
                break;

        case COMPARE:
                *output = DALI_COMPARE << 8u;
                break;

        case WITHDRAW:
                *output = DALI_WITHDRAW << 8u;
                break;

        case SEARCH_ADDRESS_H:
                *output = (DALI_SEARCHADDRH << 8u) + dataVal;
                break;

        case SEARCH_ADDRESS_M:
                *output = (DALI_SEARCHADDRM << 8u) + dataVal;
                break;

        case SEARCH_ADDRESS_L:
                *output = (DALI_SEARCHADDRL << 8u) + dataVal;
                break;

        case PHYSICAL_SELECTION:
                *output = (DALI_PHYSICAL_SELECTION << 8u);
                break;

        case ENABLE_DEVICE_TYPE:
                *output = (DALI_ENABLE_DEVICE_TYPE_X << 8u) + dataVal;
                break;

        default:
                return _ERR_WRONG_COMMAND_;
                break;
        }
        return _ERR_OK_;
}
/*-----------------------------------------------------------------------------
 *      nibble_to_ascii():  convert nibble to ascii
 *
 *  Parameters: uint8_t nibble
 *
 *  Return:     char
 *             
 *----------------------------------------------------------------------------*/
char nibble_to_ascii(uint8_t nibble)
{

        nibble = nibble & 0x0Fu;
        if(nibble < 0x0Au)
                return nibble + '0';
        else
                return nibble + 'A' - 0x0Au;
}
/*-----------------------------------------------------------------------------
 *      parse_int():  parse string to integer value
 *
 *  Parameters: const char* string, int16_t* integer
 *
 *  Return:     int16_t  _ERR_PARSE_ERROR_ or _ERR_OK_
 *             
 *----------------------------------------------------------------------------*/
inline int16_t parse_int(const char* string, int16_t* integer)
{
        int16_t i = 0;
        uint8_t sign = FALSE;

        if(*string == '-')
        {
           sign = TRUE;
           string++;
        }

        while(*string != 0)
        {
            if(isdigit(*string) && (*string != 0))
            {
                i *= 10;
                i += *string - '0';
                string++;
            }
            else
            {
                return _ERR_PARSE_ERROR_;
            }
        }

        if(sign)
        {
             *integer = -i;
        }
        else
        {
             *integer = i;
        }
        return _ERR_OK_;
}
/*-----------------------------------------------------------------------------
 *      decode_command_to_frame():  decode command to a frame 
 *
 *  Parameters: const char* token, word* output
 *
 *  Return:     int16_t _ERR_PARAMETER_MISSING_ or DALI_ERR_INVALID_FRAME or 
 *              _ERR_PARSE_ERROR_ or _ERR_WRONG_COMMAND_ or _ERR_UNIMPLEMENTED_ 
 *              or _ERR_ACK or _ERR_NACK or _ERR_OK  
 *----------------------------------------------------------------------------*/
int16_t decode_command_to_frame(const char* token, word* output)
{
        char command[MAX_COMMAND_LENGTH+1] = {0};
        char compare_string[MAX_COMMAND_LENGTH] = {0};
        char param1_string[MAX_VALUE_LENGTH+1] = {0};
        char param2_string[MAX_VALUE_LENGTH+1] = {0};
        uint8_t has_param1 = FALSE;
        uint8_t has_param2 = FALSE;
        int16_t param1 = 0;
        int16_t param2 = 0;
        uint16_t i;
        uint16_t u;
        int16_t length;
        int16_t ret;
        char* res;

        *output = DALI_ERR_INVALID_FRAME;

        length = strlen((char*)token);
        if(length == 0)
                return _ERR_PARSE_ERROR_;
        i = 0;
        for(u = 0; (u < length); u++)
        {
                if(token[u] == '#')
                        i = u + 1;                                              //discard beginning of shell command line
        }
        for(; (i < length) && (token[i] == ' '); i++);                          //discard leading spaces

        u = i;

        for(; (i < length) && (token[i] != ' ') && (token[i] != '\n') && (token[i] != '\r'); i++)
        {
                if(i-u >= MAX_COMMAND_LENGTH)
                        return _ERR_PARSE_ERROR_;
                command[i-u] = token[i];
        }

        for(; (i < length) && (token[i] == ' '); i++);                          //discard leading spaces
        u = i;

        for(; (i < length) && (token[i] != ' ') && (token[i] != '\n') && (token[i] != '\r'); i++)
        {
                has_param1 = TRUE;
                if(i-u >= MAX_VALUE_LENGTH)
                        return _ERR_PARSE_ERROR_;
                param1_string[i-u] = token[i];
        }

        for(; (i < length) && (token[i] == ' '); i++);                          //discard leading spaces
        u = i;


        for(; (i < length) && (token[i] != ' ') && (token[i] != '\n') && (token[i] != '\r'); i++)
        {
                has_param2 = TRUE;
                if(i-u >= MAX_VALUE_LENGTH)
                        return _ERR_PARSE_ERROR_;
                param2_string[i-u] = token[i];
        }

        if(has_param1)
        {
                if(_ERR_OK_ != parse_int(param1_string, &param1))
                        return _ERR_PARSE_ERROR_;
                if(param1 > 255 || param1 < 0)
                        return _ERR_PARSE_ERROR_;
        }

        if(has_param2)
        {
                if(_ERR_OK_ !=  parse_int(param2_string, &param2))
                        return _ERR_PARSE_ERROR_;
                if(param2 > 255 || param2 < 0)
                        return _ERR_PARSE_ERROR_;
        }

        if(!strcmp((char*)command,(char*) command_arc))
        {
                if(has_param1 && has_param2)
                {
                        ret = dali_slave_direct_arc(output, (byte)param1, (byte)param2);
                        if(ret == _ERR_OK_)
                        {
                                arc_device[param1] = param2;
                                return _MODE_SIMPLE_;
                        }
                        else return ret;
                }
                return _ERR_PARAMETER_MISSING_;
        }

        if(!strcmp((char*)command,(char*) command_arc_group))
        {
                if(has_param1 && has_param2)
                {
                        ret = dali_group_direct_arc(output, (byte)param1, (byte)param2);
                        if(ret == _ERR_OK_)
                        {
                                arc_group[param1] = param2;
                                return _MODE_SIMPLE_;
                        }
                        else return ret;
                }
                return _ERR_PARAMETER_MISSING_;
        }

        if(!strcmp((char*)command,(char*) command_arc_broadcast))
        {
                if(has_param1)
                {
                        ret = dali_broadcast_direct_arc(output, (byte)param1);
                        if(ret == _ERR_OK_)
                                return _MODE_SIMPLE_;
                        else return ret;
                }
                return _ERR_PARAMETER_MISSING_;
        }

        for(i = 0; i < COUNT_COMMANDS; i++)
        {
                if(!strcmp((char*)command,(char*) command_list[i].key))
                {
                        if(has_param1)
                        {
                                ret = dali_slave_command(output, (byte)param1, command_list[i].value);
                                if(ret == _ERR_OK_)
                                        return pgm_read_byte(command_list[i].mode);
                                else
                                        return ret;
                        }
                        return _ERR_PARAMETER_MISSING_;
                }
        }

        for(i = 0; i < COUNT_COMMANDS; i++)
        {
                res=strcpy((char*)compare_string,(char*) command_list[i].key);
                res=strcat((char*)compare_string,(char*) group_postfix);
                if(!strcmp(compare_string, command))
                {
                        if(has_param1)
                        {
                                ret = dali_group_command(output, (byte)param1, command_list[i].value);
                                if(ret == _ERR_OK_)
                                        return command_list[i].mode;
                                else
                                        return ret;
                        }
                        return _ERR_PARAMETER_MISSING_;
                }
        }

        for(i = 0; i < COUNT_COMMANDS; i++)
        {
                res=strcpy((char*)compare_string,(char *) command_list[i].key);
                res=strcat((char*)compare_string,(char*) broadcast_postfix);
                if(!strcmp(compare_string, command))
                {
                        ret = dali_broadcast_command(output, command_list[i].value);
                        if(ret == _ERR_OK_)
                                return command_list[i].mode;
                        else
                                return ret;
                }
        }

        for(i = 0; i < COUNT_COMMANDS_WITH_PARAM; i++)
        {
                if(!strcmp((char*)command,(char*) command_with_param_list[i].key))
                {
                        if(has_param1 && has_param2)
                        {
                                ret = dali_slave_command_with_param(output, (byte)param1, command_with_param_list[i].value, param2);
                                if(ret == _ERR_OK_)
                                        return command_with_param_list[i].mode;
                                else
                                        return ret;
                        }
                        return _ERR_PARAMETER_MISSING_;
                }
        }

        for(i = 0; i < COUNT_COMMANDS_WITH_PARAM; i++)
        {
                res=strcpy((char*)compare_string,(char*) command_with_param_list[i].key);
                res=strcat((char*)compare_string,(char*) group_postfix);
                if(!strcmp(compare_string, command))
                {
                        if(has_param1 && has_param2)
                        {
                                ret = dali_group_command_with_param(output, (byte)param1, command_with_param_list[i].value, param2);
                                if(ret == _ERR_OK_)
                                        return command_with_param_list[i].mode;
                                else
                                        return ret;
                        }
                        return _ERR_PARAMETER_MISSING_;
                }
        }

        for(i = 0; i < COUNT_COMMANDS_WITH_PARAM; i++)
        {
                res=strcpy((char*)compare_string,(char*) command_with_param_list[i].key);
                res=strcat((char*)compare_string,(char*) broadcast_postfix);
                if(!strcmp(compare_string, command))
                {
                        if(has_param1)
                        {
                                ret = dali_broadcast_command_with_param(output, command_with_param_list[i].value, param1);
                                if(ret == _ERR_OK_)
                                        return command_with_param_list[i].mode;
                                else
                                        return ret;
                        }
                        return _ERR_PARAMETER_MISSING_;
                }
        }

        for(i = 0; i < COUNT_SPECIAL_COMMANDS; i++)
        {
                if(!strcmp((char*)command,(char*) special_command_list[i].key))
                {
                        ret = dali_special_command(output, special_command_list[i].special, (byte)param1);
                        if(ret == _ERR_OK_)
                                return special_command_list[i].mode;
                        else
                                return ret;
                }
        }

        if(!strcmp((char*)command, (char*) NACK))
                return _ERR_NACK;
        if(!strcmp((char*)command, (char*) ACK))
                return _ERR_ACK;

#ifdef DALI_HAS_EEPROM                                                          /* if you have implemented this functionality in the chip's EEPROM (not supported at present consder mmc file system write */
        if(!strcmp(command, internal_command_save_current_as_default))
        {
                save_startup_values();
                return _ERR_ACK;
        }

        if(!strcmp(command, internal_command_clear_default_values))
        {
                clear_startup_values();
                return _ERR_ACK;
        }
#endif
        return _ERR_UNIMPLEMENTED_;

}