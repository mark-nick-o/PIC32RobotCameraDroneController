#ifndef  __SBGC_adj_vars__
#define  __SBGC_adj_vars__

#ifdef __cplusplus
 extern "C" {
#endif

/*
        SBGC_adj_vars.h : SimpleBGC Serial API  library - adjustable variables
        More info: http://www.basecamelectronics.com/serialapi/

  Copyright (c) 2014-2015 Aleksei Moskalenko
  v1.1 Modified ACP Aviation October 2019...
  Ported and expanded by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland

  All rights reserved.
        See license info in the SBGC.h
*/
#include "inttypes.h"

// RC_MODE special value for enocoded lost signal
//#define SBGC_RC_UNDEF -10000    IN _rc.H

// Flags which go with the SBGC CMD_CONTROL
#define SBGC_CONTROL_FLAG_AUTO_TASK (1u<<6u)                                    // For modes MODE_ANGLE and MODE_REL_FRAME Use this flag to move gimbal to a certain position as fast as possible, and receive confirmation when the target is reached.
#define SBGC_CONTROL_FLAG_HIGH_RES_SPEED (1u<<7u)                               // For MODE_SPEED Speed units changed to 0.001 deg/sec for extremely slow motion (like timelapse shooting)

// Flags which go with the SBGC_CMD_CONTROL_CONFIG
#define SBGC_CONTROL_CONFIG_FLAG_NO_CONFIRM (1u<<0u)                            // If set do not give confirm message when CMD_CONTROL is sent

// variables for the SELECT_IMU command
#define SBGC_IMU_TYPE_MAIN 1U
#define SBGC_IMU_TYPE_FRAME 2U

// adjustable variables to the AUTO_PID message
#define SBGC_PID_STOP 0U                                                        // The possible states of the command.   auto pid
#define SBGC_PID_ROLL 1U
#define SBGC_PID_PITCH 2U
#define SBGC_PID_YAW 4U
#define SBGC_PID_GUI 8U
#define SBGC_PID_KEEP 16U
#define SBGC_PID_LPF 32U

#define SBGC_MOM_AUTO 0U
#define SBGC_MTR_STRONG 1U
#define SBGC_MTR_WEAK 255U

#define SBGC_START_TUNE 0U

// values for the CMD_SET_ADJ_VARS_VAL PARAM_ID's (adjustable pin ids)
#define SBGC_P_ROLL 0U
#define SBGC_P_PITCH 1U
#define SBGC_P_YAW 2U
#define SBGC_I_ROLL 3U
#define SBGC_I_PITCH 4U
#define SBGC_I_YAW 5U
#define SBGC_D_ROLL 6U
#define SBGC_D_PITCH 7U
#define SBGC_D_YAW 8U
#define SBGC_POWER_ROLL 9U
#define SBGC_POWER_PITCH 10U
#define SBGC_POWER_YAW 11U
#define SBGC_ACC_LIMITER 12U
#define SBGC_FOLLOW_SPEED_ROLL 13U
#define SBGC_FOLLOW_SPEED_PITCH 14U
#define SBGC_FOLLOW_SPEED_YAW 15U
#define SBGC_FOLLOW_LPF_ROLL 16U
#define SBGC_FOLLOW_LPF_PITCH 17U
#define SBGC_FOLLOW_LPF_YAW 18U
#define SBGC_RC_SPEED_ROLL 19U
#define SBGC_RC_SPEED_PITCH 20U
#define SBGC_RC_SPEED_YAW 21U
#define SBGC_RC_LPF_ROLL 22U
#define SBGC_RC_LPF_PITCH 23U
#define SBGC_RC_LPF_YAW 24U
#define SBGC_RC_TRIM_ROLL 25U
#define SBGC_RC_TRIM_PITCH 26U
#define SBGC_RC_TRIM_YAW 27U
#define SBGC_RC_DEADBAND 28U
#define SBGC_RC_EXPO_RATE 29U
#define SBGC_FOLLOW_PITCH 30U
#define SBGC_FOLLOW_YAW_PITCH 31U
#define SBGC_FOLLOW_DEADBAND 32U
#define SBGC_FOLLOW_EXPO_RATE 33U
#define SBGC_FOLLOW_ROLL_MIX_START 34U
#define SBGC_FOLLOW_ROLL_MIX_RANGE 35U
#define SBGC_GYRO_TRUST 36U
#define SBGC_FRAME_HEADING_ANGLE 37U
#define SBGC_GYRO_HEADING_CORRECTION 38U
#define SBGC_ACC_LIMITER_ROLL 39U
#define SBGC_ACC_LIMITER_PITCH 40U
#define SBGC_ACC_LIMITER_YAW 41U
#define SBGC_PID_GAIN_ROLL 42U
#define SBGC_PID_GAIN_PITCH 43U
#define SBGC_PID_GAIN_YAW 44U
#define SBGC_LPF_FREQ_ROLL 45U
#define SBGC_LPF_FREQ_PITCH 46U
#define SBGC_LPF_FREQ_YAW 47U
#define SBGC_TIMELAPSE_TIME 48U
#define SBGC_MAV_CTRL_MODE 49U
#define SBGC_H_CORR_FACTOR 50U
#define SBGC_SW_LIM_MIN_ROLL 51U
#define SBGC_SW_LIM_MAX_ROLL 52U
#define SBGC_SW_LIM_MIN_PITCH 53U
#define SBGC_SW_LIM_MAX_PITCH 54U
#define SBGC_SW_LIM_MIN_YAW 55U
#define SBGC_SW_LIM_MAX_YAW 56U
#define SBGC_FOLLOW_RANGE_ROLL 57U
#define SBGC_FOLLOW_RANGE_PITCH 58U
#define SBGC_FOLLOW_RANGE_YAW 59U

#define MENU_CMD_NO = 0U                                                         // options for CMD_EXECUTE_MENU
#define MENU_CMD_PROFILE1 1U
#define MENU_CMD_PROFILE2 2U
#define MENU_CMD_PROFILE3 3U
#define MENU_CMD_SWAP_PITCH_ROLL 4U
#define MENU_CMD_SWAP_YAW_ROLL 5U
#define MENU_CMD_CALIB_ACC 6U
#define MENU_CMD_RESET 7U
#define MENU_CMD_SET_ANGLE 8U
#define MENU_CMD_CALIB_GYRO 9U
#define MENU_CMD_MOTOR_TOGGLE 10U
#define MENU_CMD_MOTOR_ON 11U
#define MENU_CMD_MOTOR_OFF 12U
#define MENU_CMD_FRAME_UPSIDE_DOWN 13U
#define MENU_CMD_PROFILE4 14U
#define MENU_CMD_PROFILE5 15U
#define MENU_CMD_AUTO_PID 16U
#define MENU_CMD_LOOK_DOWN 17U
#define MENU_CMD_HOME_POSITION 18U
#define MENU_CMD_RC_BIND 19U
#define MENU_CMD_CALIB_GYRO_TEMP 20U
#define MENU_CMD_CALIB_ACC_TEMP 21U
#define MENU_CMD_BUTTON_PRESS 22U
#define MENU_CMD_RUN_SCRIPT1 23U
#define MENU_CMD_RUN_SCRIPT2 24U
#define MENU_CMD_RUN_SCRIPT3 25U
#define MENU_CMD_RUN_SCRIPT4 26U
#define MENU_CMD_RUN_SCRIPT5 27U
#define MENU_CMD_CALIB_MAG 33U
#define MENU_CMD_LEVEL_ROLL_PITCH 34U
#define MENU_CMD_CENTER_YAW 35U
#define MENU_CMD_UNTWIST_CABLES 36U
#define MENU_CMD_SET_ANGLE_NO_SAVE 37U
#define MENU_HOME_POSITION_SHORTEST 38U
#define MENU_CENTER_YAW_SHORTEST 39U
#define MENU_ROTATE_YAW_180 40U
#define MENU_ROTATE_YAW_180_FRAME_REL 41U
#define MENU_SWITCH_YAW_180_FRAME_REL 42
#define MENU_SWITCH_POS_ROLL_90 43U
#define MENU_START_TIMELAPSE 44U
#define MENU_CALIB_MOMENTUM 45U
#define MENU_LEVEL_ROLL 46U
#define MENU_REPEAT_TIMELAPSE 47U
#define MENU_LOAD_PROFILE_SET1 48U
#define MENU_LOAD_PROFILE_SET2 49U
#define MENU_LOAD_PROFILE_SET3 50U
#define MENU_LOAD_PROFILE_SET4 51U
#define MENU_LOAD_PROFILE_SET5 52U
#define MENU_LOAD_PROFILE_SET_BACKUP 53U
#define MENU_INVERT_RC_ROLL 54U
#define MENU_INVERT_RC_PITCH 55U
#define MENU_INVERT_RC_YAW 56U
#define MENU_SNAP_TO_FIXED_POSITION 57U
#define MENU_CAMERA_REC_PHOTO_EVENT 58U
#define MENU_CAMERA_PHOTO_EVENT 59U
#define MENU_MOTORS_SAFE_STOP 60U
#define MENU_CALIB_ACC_AUTO 61U
#define MENU_RESET_IMU 62U
#define MENU_FORCED_FOLLOW_TOGGLE 63U
#define MENU_AUTO_PID_GAIN_ONLY 64U
#define MENU_LEVEL_PITCH 65U

#define SBGC_IMU_TYPE_MAIN 1U                                                   // options for CMD_SELECT_IMU
#define SBGC_IMU_TYPE_FRAME 2U

#define SBGC_CMD_WRITE_FILE_NO_ERROR 0U                                         // Error codes returned in the confirm message for CMD_WRITE_FILE
#define SBGC_CMD_WRITE_FILE_ERR_EEPROM_FAULT 1U
#define SBGC_CMD_WRITE_FILE_ERR_FILE_NOT_FOUND 2U
#define SBGC_CMD_WRITE_FILE_ERR_FAT 3U
#define SBGC_CMD_WRITE_FILE_ERR_NO_FREE_SPACE 4U
#define SBGC_CMD_WRITE_FILE_ERR_FAT_IS_FULL 5U
#define SBGC_CMD_WRITE_FILE_ERR_FILE_SIZE 6U
#define SBGC_CMD_WRITE_FILE_ERR_CRC 7U
#define SBGC_CMD_WRITE_FILE_ERR_LIMIT_REACHED 8U
#define SBGC_CMD_WRITE_FILE_ERR_FILE_CORRUPTED 9U
#define SBGC_CMD_WRITE_FILE_ERR_WRONG_PARAMS 10U

#define SBGC_CUSTOM_IMU_ANGLES 1U                                               // Define the variables for CMD_REALTIME_DATA_CUSTOM
#define SBGC_CUSTOM_TARGET_ANGLES 2U
#define SBGC_CUSTOM_TARGET_SPEED 4U
#define SBGC_CUSTOM_STATOR_ROTOR_ANGLE 8U
#define SBGC_CUSTOM_GYRO_DATA 16U
#define SBGC_CUSTOM_RC_DATA 32U
#define SBGC_CUSTOM_ZH_VECTOR 64U
#define SBGC_CUSTOM_RC_CHANNELS 128U
#define SBGC_CUSTOM_ACC_DATA 256U
#define SBGC_CUSTOM_MOTOR4_CONTROL 512U
#define SBGC_CUSTOM_AHRS_DEBUG_INFO 1024U
#define SBGC_CUSTOM_ENCODER_RAW24 2048U
#define SBGC_CUSTOM_IMU_ANGLES_RAD 4096U

#define SBGC_BEEPER_MODE_CALIBRATE (1u<<0u)                                     // First byte pre-programed accapela
#define SBGC_BEEPER_MODE_CONFIRM (1u<<1u)
#define SBGC_BEEPER_MODE_ERROR (1u<<2u)
#define SBGC_BEEPER_MODE_CLICK (1u<<4u)
#define SBGC_BEEPER_MODE_COMPLETE (1u<<5u)
#define SBGC_BEEPER_MODE_INTRO (1u<<6u)
#define SBGC_BEEPER_MODE_CUSTOM_MELODY (1u<<15u)                                // Second byte custom melody
#define SBGC_BEEPER_MODE_ALARM 8U
#define SBGC_BEEP_BY_MOTORS 128U

// CMD_EVENT (Event_ID)
#define SBGC_EVENT_ID_MENU_BUTTON 1U                                            // on menu button press
#define SBGC_EVENT_ID_MOTOR_STATE 2U                                            // generated on motor state change
#define SBGC_EVENT_ID_EMERGENCY_STOP 3U                                         // emergency stop error
#define SBGC_EVENT_ID_CAMERA 4U                                                 // on recording a photo
#define SBGC_EVENT_ID_SCRIPT 5U                                                 // script start / stop
// CMD_EVENT (Event_Type)
#define SBGC_EVENT_TYPE_OFF 1U                                                  // generate events on change to false state
#define SBGC_EVENT_TYPE_ON 2U                                                   // generate events on change to true state
#define SBGC_EVENT_TYPE_HOLD 4U                                                 // generate events on button held down
#define SBGC_EVENT_TYPE_REC_PHOTO 1U                                            // generate events on record photo
#define SBGC_EVENT_TYPE_PHOTO 2U                                                // generate events on photo event
// CMD_EVENT (param)
#define SBGC_EVENT_ID_SCRIPT_SLOT0 0U                                           // Run the script which has been loaded to slot 0
#define SBGC_EVENT_ID_SCRIPT_SLOT1 1U                                           // Run the script which has been loaded to slot 1
#define SBGC_EVENT_ID_SCRIPT_SLOT2 2U                                           // Run the script which has been loaded to slot 2
#define SBGC_EVENT_ID_SCRIPT_SLOT3 3U                                           // Run the script which has been loaded to slot 3
#define SBGC_EVENT_ID_SCRIPT_SLOT4 4U                                           // Run the script which has been loaded to slot 4
#define SBGC_EVENT_ID_SCRIPT_SLOT5 5U                                           // Run the script which has been loaded to slot 5
// CMD_BOARD_INFO
#define SBGC_BOARD_FEATURE_3AXIS 1U
#define SBGC_BOARD_FEATURE_BAT_MONITORING 2U
#define SBGC_BOARD_FEATURE_ENCODERS 4U
#define SBGC_BOARD_FEATURE_BODE_TEST 8U
#define SBGC_BOARD_FEATURE_SCRIPTING 16U
#define SBGC_BOARD_FEATURE_CURRENT_SENSOR 32U
#define SBGC_BOARD_DEBUG_MODE (1u<<0u)
#define SBGC_BOARD_IS_FRAME_INVERTED (1u<<1u)
#define SBGC_BOARD_INIT_STEP1_DONE (1u<<2u)
#define SBGC_BOARD_INIT_STEP2_DONE (1u<<3u)
#define SBGC_BOARD_STARTUP_AUTO_ROUTINE_DONE (1u<<4u)
// CMD Profile_SET
#define SBGC_PROFILE_SET_ACTION_SAVE 1U                                         // save current configuration (including all profiles and simple calibrations) to the given slot
#define SBGC_PROFILE_SET_ACTION_CLEAR 2U                                        // cleat the selected slot
#define SBGC_PROFILE_SET_ACTION_LOAD 3U                                         // load configuration from the given slot
// CMD AUTO_PID_2
#define SBGC_PID2_ACTION_START 1U                                               //start tuning (do not update config in EEPROM)
#define SBGC_PID2_ACTION_START_SAVE 2U                                          //save config to EEPROM and start tuning
#define SBGC_PID2_ACTION_SAVE 3U                                                //save config to EEPROM
#define SBGC_PID2_ACTION_STOP 5U                                                //stop tuning
#define SBGC_PID2_ACTION_READ 6U                                                //read config from EEPROM
// PID2 Axis flags options
#define SBGC_PID2_AXIS_ENABLE (1u<<0u)                                          // Axis enabled
#define SBGC_PID2_AXIS_TUNELPF (1u<<1u)                                         // Tune LPF
#define SBGC_PID2_NOTCH_FILTERS1 (1u<<2u)                                       // Notch Filter 1
#define SBGC_PID2_NOTCH_FILTERS2 (1u<<3u)                                       // 2 Notch Filters
#define SBGC_PID2_NOTCH_FILTERS3 ((1u<<2u)+(1u<<3u))                            // 3 Notch Filters
// PID2 General flags
#define SBGC_PID2_USE_CURRENT (1u<<0u)                                          //start from current values
#define SBGC_PID2_SAVE_TO_ALL (1u<<1u)                                          //bit1: save result to all profiles
#define SBGC_PID2_TUNE_GAIN (1u<<2u)                                            //bit2: tune gain only
#define SBGC_PID2_AUTO_SAVE (1u<<4u)                                            //bit4: auto-sav
// PID2 start-up config                                                         //0 - Disabled 1 - Tune gain only 2 - Tune all parameters
#define SBGC_PID2_TUNE_DISABLED 0U
#define SBGC_PID2_TUNE_GAIN_ONLY 1U
#define SBGC_PID2_TUNE_ALL 2U
// CMD_READ_PARAMS_3                                                            // Defined configuration settings
#define SBGC_RC_MODE_ANGLE 0U
#define SBGC_RC_MODE_SPEED 1U
#define SBGC_RC_CONTROL_INVERT 4U
#define SBGC_PWM_FREQ_LOW 0U
#define SBGC_PWM_FREQ_HIGH 1U
#define SBGC_PWM_FREQ_ULTRA_HIGH 2U
#define SBGC_SERIAL_SP_115200 0U
#define SBGC_SERIAL_SP_57600 1U
#define SBGC_SERIAL_SP_38400 2U
#define SBGC_SERIAL_SP_19200 3U
#define SBGC_SERIAL_SP_9600 4U
#define SBGC_SERIAL_SP_256000 5U
#define SBGC_RC_VIRT_MODE_NORMAL  0U
#define SBGC_RC_VIRT_MODE_CPPM 1U
#define SBGC_RC_VIRT_MODE_SBUS 2U
#define SBGC_RC_VIRT_MODE_SPEKTRUM 3U
#define SBGC_RC_VIRT_MODE_API 10U
//#define SBGC_RC_INPUT_ROLL 1U
//#define SBGC_RC_INPUT_PITCH 2U
//#define SBGC_EXT_FC_INPUT_ROLL 3U
//#define SBGC_EXT_FC_INPUT_PITCH 4U
//#define SBGC_RC_INPUT_YAW 5U
#define SBGC_RC_MAP_ADC1 33U
#define SBGC_RC_MAP_ADC2 34U
#define SBGC_RC_MAP_ADC3 35U
#define SBGC_RC_MAP_SS 64U
#define SBGC_RC_MAP_API_VCS 128U
#define SBGC_RC_MAP_API_SSS 160U
#define SBGC_TARGET_FC_NOMIX 0x3FU
#define SBGC_TARGET_FC_ROLL 0x7FU
#define SBGC_TARGET_FC_PITCH 0xBFU
#define SBGC_TARGET_FC_YAW 0xFFU
#define SBGC_FOLLOW_MODE_DISABLED 0U
#define SBHC_FOLLOW_MODE_FC 1U
#define SBGC_FOLLOW_MODE_PITCH 2U
#define SBGC_FRAME_IMU_DISABLED 0U
#define SBGC_FRAME_IMU_BELOW_YAW 1U
#define SBGC_FRAME_IMU_ABOVE_YAW 2U
#define SBGC_FRAME_IMU_BELOW_YAW_PID_SOURCE 3U
#define SBGC_NO_SKIP_GYRO_CAL 0U
#define SBGC_SKIP_GYRO_CAL 1U
#define SBGC_MOTION_SKIP_GYRO_CAL 2U
#define SBGC_MOTOR_OUT_IS_DISABLED 0U
#define SBGC_MOTOR_OUT_IS_ROLL 1U
#define SBGC_MOTOR_OUT_IS_PITCH 2U
#define SBGC_MOTOR_OUT_IS_YAW 3U
#define SBGC_MOTOR_OUT_IS_I2C_DRV_1 4U
#define SBGC_MOTOR_OUT_IS_I2C_DRV_2 5U
#define SBGC_MOTOR_OUT_IS_I2C_DRV_3 6U
#define SBGC_MOTOR_OUT_IS_I2C_DRV_4 7U
//#define SBGC_BEEPER_MODE_CALIBRATE 1
//#define SBGC_BEEPER_MODE_CONFIRM 2
//#define SBGC_BEEPER_MODE_ERROR 4

#define SBGC_ADAPTIVE_PID_DISABLE 0U
#define SBGC_ADAPTIVE_PID_EN_ROLL 1U
#define SBGC_ADAPTIVE_PID_EN_PITCH 2U
#define SBGC_ADAPTIVE_PID_EN_YAW 4U
#define SBGC_GF_REMEMBER_LAST_USED_PROFILE (1u<<0u)
#define SBGC_GF_UPSIDE_DOWN_AUTO (1u<<1u)
#define SBGC_GF_SWAP_FRAME_MAIN_IMU (1u<<2u)
#define SBGC_GF_BLINK_PROFILE (1u<<3u)
#define SBGC_GF_EMERGENCY_STOP (1u<<4u)
#define SBGC_GF_MAGNETOMETER_POS_FRAME (1u<<5u)
#define SBGC_GF_FRAME_IMU_FF (1u<<6u)
#define SBGC_GF_OVERHEAT_STOP_MOTORS (1u<<7u)
#define SBGC_GF_CENTER_YAW_AT_STARTUP (1u<<8u)
#define SBGC_GF_SWAP_RC_SERIAL_UART_B (1u<<9u)
#define SBGC_GF_UART_B_SERIAL_API (1u<<10u)
#define SBGC_GF_BLINK_BAT_LEVEL (1u<<11u)
#define SBGC_GF_ADAPTIVE_GYRO_TRUST (1u<<12u)
#define SBGC_GF_IS_UPSIDE_DOWN (1u<<13u)
#define SBGC_PF_ADC1_AUTO_DETECTION (1u<<0u)
#define SBGC_PF_ADC2_AUTO_DETECTION (1u<<1u)
#define SBGC_PF_ADC3_AUTO_DETECTION (1u<<2u)
#define SBGC_PF_FOLLOW_USE_FRAME_IMU (1u<<4u)
#define SBGC_PF_BRIEFCASE_AUTO_DETECTION (1u<<5u)
#define SBGC_PF_UPSIDE_DOWN_AUTO_ROTATE (1u<<6u)
#define SBGC_PF_FOLLOW_LOCK_OFFSET_CORRECTION (1u<<7u)
#define SBGC_PF_START_NEUTRAL_POSITION (1u<<8u)
#define SBGC_PF_MENU_BUTTON_DISABLE_FOLLOW (1u<<9u)
#define SBGC_PF_TIMELAPSE_FRAME_FIXED (1u<<10u)
#define SBGC_PF_RC_KEEP_MIX_RATE (1u<<11u)
#define SBGC_PF_RC_KEEP_CUR_POS_ON_INIT (1u<<12u)
#define SBGC_PF_OUTER_MOTOR_LIMIT_FREE_ROTATION (1u<<13u)                         // (frw. ver. 2.66+)
#define SBGC_PF_EULER_ORDER_AUTO (1u<<14u)                                        // (frw. ver. 2.67b1+)
#define SBGC_SPEK_MODE_AUTO 0U                                                  //Auto-detection (default)
#define SBGC_SPEK_MODE_DSM2_11ms_10bit 1U                                       // DSM2/11ms/10bit
#define SBGC_SPEK_MODE_DSM2_11ms_11bit 2U                                       //DSM2/11ms/11bit
#define SBGC_SPEK_MODE_DSM2_22ms_10bit 3U                                       //DSM2/22ms/10bit
#define SBGC_SPEK_MODE_DSM2_22ms_11bit 4U                                       //DSM2/22ms/11bit
#define SBGC_SPEK_MODE_DSMX_11ms_10bit 5U                                       //DSMX/11ms/10bit
#define SBGC_SPEK_MODE_DSMX_11ms_11bit 6U                                       //DSMX/11ms/11bit
#define SBGC_SPEK_MODE_DSMX_22ms_10bit 7U                                       //DSMX/22ms/10bit
#define SBGC_SPEK_MODE_DSMX_22ms_11bit 8U                                       //DSMX/22ms/11bit
#define SBGC_ORDER_PITCH_ROLL_YAW 0U
#define SBGC_ORDER_YAW_ROLL_PITCH 1U
#define SBGC_ORDER_ROLL_YAW_PITCH 2U
#define SBGC_ORDER_ROLL_PITCH_YAW 3U
#define SBGC_EULER_ORDER_PITCH_ROLL_YAW 0U
#define SBGC_EULER_ORDER_ROLL_PITCH_YAW 1U
#define SBGC_EULER_ORDER_LOCAL_ROLL 2U
#define SBGC_EULER_ORDER_ROLL_LOCAL 3U
#define SBGC_EULER_ORDER_YAW_ROLL_PITCH 4U
#define SBGC_EULER_ORDER_YAW_PITCH_ROLL 5U
// CMD_READ_PARAMS_EXT                                                            // Defined configuration settings
#define SBGC_FILTERS_DISABLE 0U
#define SBGC_FILTERS_EN_NOTCH1 1U
#define SBGC_FILTERS_EN_NOTCH2 2U
#define SBGC_FILTERS_EN_NOTCH3 4U
#define SBGC_FILTERS_EN_LPF 8U
#define SBGC_ENC_TYPE_AS5048A 1U
#define SBGC_ENC_TYPE_AS5048B 2U
#define SBGC_ENC_TYPE_AS5048_PWM 3U
#define SBGC_ENC_TYPE_AMT203 4U
#define SBGC_ENC_TYPE_MA3_10BIT 5U
#define SBGC_ENC_TYPE_MA3_12BIT 6U
#define SBGC_ENC_TYPE_ANALOG = 7U
#define SBGC_ENC_TYPE_I2C_DRV1 8U
#define SBGC_ENC_TYPE_I2C_DRV2 9U
#define SBGC_ENC_TYPE_I2C_DRV3 10U
#define SBGC_ENC_TYPE_I2C_DRV4 11U
#define SBGC_ENC_TYPE_AS5600_PWM 12U
#define SBGC_ENC_TYPE_AS5600_I2C 13U
#define SBGC_ENC_TYPE_RLS_ORBIS 14U
#define SBGC_ENC_TYPE_RLS_ORBIS_PWM 15U
#define SBGC_ENC_SKIP_DETECTION 16U
#define SBGC_ENC_GEARED 128U
#define SBGC_ENC_SPI_SPEED_1MHz 0U
#define SBGC_ENC_SPI_SPEED_4MHz 2U
#define SBGC_ENC_SPI_SPEED_500kHz 3U
#define SBGC_ENC_SPI_SPEED_2MHz 1U
// CMD_READ_PARAMS_EXT2                                                         // Defined configuration settings
#define SBGC_MAV_SRC_UART1 1U
#define SBGC_MAV_SRC_RC_SERIAL 2U
#define SBGC_MAV_SRC_UART2 3U
#define SBGC_MAV_SRC_USB_VCP 4U
#define SBGC_FLAG_BAUD_MASK ((1u<<0u) | (1u<<1u) | (1u<<2u))                          // baud rate idx 0..5
#define SBGC_FLAG_PARITY_EVEN (1u<<3u)                                            // even parity
#define SBGC_FLAG_HEARTBEAT (1u<<4u)                                              // send heartbeat
#define SBGC_FLAG_DEBUG (1u<<5u)                                                  // send debug to GUI
#define SBGC_FLAG_RC (1u<<6u)                                                     // use RC values

#define SBGC_SEARCH_LIMIT_ROLL (1u<<0u)
#define SBGC_SEARCH_LIMIT_PITCH (1u<<1u)
#define SBGC_SEARCH_LIMIT_YAW (1u<<2u)
#define SBGC_AUTO_CALIBRATE_MOMENTUM (1u<<3u)
#define SBGC_USE_MOMENTUM_FEED_FORWARD (1u<<4u)
#define SBGC_MOTORS_OFF_AT_STARTUP (1u<<5u)
#define SBGC_FC_BELOW_OUTER (1u<<6u)
#define SBGC_DO_NOT_CHECK_ENCODER_LIMITS (1u<<7u)
#define SBGC_AUTO_SAVE_BACKUP_SLOT (1u<<8u)
#define SBGC_FC_BELOW_MIDDLE (1u<<9u)
#define SBGC_ENVIRONMENT_TEMP_UNKNOWN (1u<<10u)
#define SBGC_STAB_AXIS_DEFAULT 0U
#define SBGC_STAB_AXIS_ROLL 1U
#define SBGC_STAB_AXIS_PITCH 2U
#define SBGC_STAB_AXIS_YAW 3U
#define SBGC_STAB_AUTO_ROLL 4U
#define SBGC_STAB_AUTO_PITCH 8U
#define SBGC_STAB_AUTO_YAW 16U
#define SBGC_FORCE_POSITION_FLAG_BUTTON_PRESS (1u<<4u)
#define SBGC_FORCE_POSITION_FLAG_STARTUP (1u<<5u)
#define SBGC_FORCE_POSITION_FLAG_IGNORE_LIMITS (1u<<6u)
#define SBGC_FORCE_POSITION_FLAG_FINE_ADJUST (1u<<7u)
#define SBGC_SSG_MODE_LEVEL_LOW 0U
#define SBGC_SSG_MODE_LEVEL_HIGH 1U
#define SBGC_SSG_MODE_LEVEL_LOW_HIGH 2U
#define SBGC_MAV_CONTROL_DISABLED 0U
#define SBGC_MAV_CONTROL_ROLL_AND_PITCH 1U
#define SBGC_MAV_CONTROL_ALL 2U
// CMD_READ_PARAMS_EXT3 – read/write system configuration part 3 (frw.ver. 2.66+
#define SBGC_EXT_IMU_MAV1 1U
#define SBGC_EXT_IMU_MAV2 2U
#define SBGC_EXT_IMU_VN200 3U
#define SBGC_EXT_IMU_uAHRS 4U
#define SBGC_EXT_PORT_DISABL 0U
#define SBGC_EXT_PORT_UART1 1U
#define SBGC_EXT_PORT_RC_SERIAL 2U
#define SBGC_EXT_PORT_UART2 3U
#define SBGC_EXT_PORT_USB_VCP 4U
#define SBGC_EXT_POS_BELOW_OUTER 1U
#define SBGC_EXT_POS_ABOVE_OUTER 2U
#define SBGC_EXT_POS_BELOW_MIDDLE 8U
#define SBGC_EXT_POS_MAIN_IMU 9U
#define SBGC_EXT_IMU_FLAG_ACC_COMP_ONLY 2U
#define SBGC_EXT_IMU_FLAG_REPLACE 4U
#define SBGC_EXT_IMU_FLAG_Z 8U
#define SBGC_EXT_IMU_FLAG_H 16U
#define SBGC_EXT_IMU_FLAG_FRAME_UPSIDE_DOWN_UPDATE 32U
#define SBGC_EXT_IMU_FLAG_AS_FRAME_IMU 64U
#define SBGC_EXT_IMU_FLAG_GYRO_CORR 128U
// System error codes in CMD_REALTIME_DATA_3
#define SBGC_ERR_NO_SENSOR (1<<0)                                               // No sensor
#define SBGC_ERR_CALIB_ACC (1<<1)                                               // Accelerometer calibration error
#define SBGC_ERR_SET_POWER (1<<2)                                               // power fault
#define SBGC_ERR_CALIB_POLES (1<<3)                                             // calibration of poles error
#define SBGC_ERR_PROTECTION (1<<4)                                              // error protection
#define SBGC_ERR_SERIAL (1<<5)                                                  // serial port error
#define SBGC_ERR_LOW_BAT1 (1<<6)                                                // low battery volatge fault
#define SBGC_ERR_LOW_BAT2 (1<<7)                                                // as above level2
#define SBGC_ERR_GUI_VERSION (1<<8)                                             // gui version error
#define SBGC_ERR_MISS_STEPS (1<<9)                                              // missed steps
#define SBGC_ERR_SYSTEM (1<<10)                                                 // erroneous system error
#define SBGC_ERR_EMERGENCY_STOP (1<<11)                                         // emergency stop
// Emergency stop reason codes from CMD_REALTIME_DATA_3
#define SBGC_SUB_ERR_I2C_ERRORS 1U                                              // High rate of I2C errors
#define SBGC_SUB_ERR_DRV_OTW 2U                                                 // Driver over-temperature protection
#define SBGC_SUB_ERR_DRV_FAULT 3U                                               // Driver fault (undervoltage, over-current, short circuit)
#define SBGC_SUB_ERR_ENCODER_IMU_ANGLE 4U                                       // Encoder/IMU angles mismatch
#define SBGC_SUB_ERR_CALIBRATION_FAILED 5U                                      // Auto calibration process caused serious fault
#define SBGC_SUB_ERR_INTERNAL_SYSTEM_ERROR 6U                                   // Stack is damaged
#define SBGC_SUB_ERR_ENCODER_CALIB_BAD_SCALE 7U                                 // estimated scale differs a lot from configured
#define SBGC_SUB_ERR_OVER_TEMPERATURE 8U                                        // MCU or power board over temperature
#define SBGC_SUB_ERR_BAD_MOTOR_POLES_INVERT 9U                                  // motor n.poles or inversion is wrong
#define SBGC_SUB_ERR_NOT_ENOUGH_MEMORY 10U                                      // static_malloc() can't allocate memory
#define SBGC_SUB_ERR_IMU_SENSOR_NOT_RESPONDING 11U                              // lost connection to IMU sensor
#define SBGC_SUB_ERR_CAN_HARD 12U                                               // CAN on board hardware error
#define SBGC_SUB_ERR_MOTOR_OVERHEAT_PROTECTION 13U                              // overheat protection is triggered
#define SBGC_SUB_ERR_MOTOR_IS_LOCKED 14U                                        // motor is locked during automated task
#define SBGC_SUB_ERR_BAD_IMU_HEALTH 15U                                         // IMU gyroscope and accelerometer error is too big: sensor sends corrupted data or wrong use conditions
#define SBGC_SUB_ERR_INFINITE_RESET 16U                                         // Infinite reset loop is detected
#define SBGC_SUB_ERR_WRONG_INITIAL_POSITION 17U                                 // wrong position: failed to detect encoder angle, or angle is outside soft limits
#define SBGC_SUB_ERR_MOTOR_LOAD_TIME_EXCEEDED 18U                               // motors are fully loaded too long time
#define SBGC_SUB_ERR_CAN_DRV_OVERCURRENT 19U                                    // hardware short-circuit protection
#define SBGC_SUB_ERR_CAN_DRV_UNDERVOLTAGE 20U                                   // hardware or software undervoltage protection
#define SBGC_SUB_ERR_CAN_DRV_EMERGENCY_PIN 21U                                  // external emergency is triggered
#define SBGC_SUB_ERR_CAN_DRV_FOC_DURATION 22U                                   // FOC algorithm duration error
#define SBGC_SUB_ERR_CAN_DRV_MCU_OVERHEAT 23U                                   // driver temperature is to high
#define SBGC_SUB_ERR_CAN_DRV_MOTOR_OVERHEAT 24U                                 // motor temperature is to high
#define SBGC_SUB_ERR_CAN_DRV_OVERCURRENT_SOFT 25U                               // current through motor exceed limit
#define SBGC_SUB_ERR_CAN_DRV_SEVERAL 26U                                        //several errors on driver
#define SBGC_SUB_ERR_CAN_EXT_BUS_OFF 27U                                        // CAN bus high rate errors of slave controller
#define SBGC_SUB_ERR_CAN_INT_BUS_OFF 28U                                        // CAN bus high rate errors of main controller
#define SBGC_SUB_ERR_ENCODER_NOT_FOUND 29U                                      // no any answer from encoder during init
#define SBGC_SUB_ERR_CAN_DRV_NOT_RESPONDING 30U                                 // lost connection to CAN Drv
#define SBGC_SUB_ERR_CAN_DRV_WRONG_PARAMS 31U                                   // some params of CAN Drv isn't correct
#define SBGC_SUB_ERR_OVERCURRENT 32U                                            // fast over current protection of main controller, or short circuit detection on startup
#define SBGC_SUB_ERR_UNSAFE_VOLTAGE 33U                                         // Under voltage protection or supply protection controller fault
#define SBGC_SUB_ERR_WRONG_FULL_BAT_VOLTAGE_PARAM 34U                           //battery voltage is higher than expected at startup sequenc
#define SBGC_SUB_ERR_EEPROM_PARAMS_CORRUPTED 35U                                // parameters are corrupted in EEPROM and can't be restored from backup slot

//#include <stdin.h>
//#include <inttypes.h>


// ID of all available variables

typedef enum {

        ADJ_VAR_P_ROLL = 0
        ,        ADJ_VAR_P_PITCH
        ,        ADJ_VAR_P_YAW
        ,        ADJ_VAR_I_ROLL
        ,        ADJ_VAR_I_PITCH
        ,        ADJ_VAR_I_YAW
        ,        ADJ_VAR_D_ROLL
        ,        ADJ_VAR_D_PITCH
        ,        ADJ_VAR_D_YAW
        ,        ADJ_VAR_POWER_ROLL
        ,        ADJ_VAR_POWER_PITCH
        ,        ADJ_VAR_POWER_YAW
        ,        ADJ_VAR_ACC_LIMITER
        ,        ADJ_VAR_FOLLOW_SPEED_ROLL
        ,        ADJ_VAR_FOLLOW_SPEED_PITCH
        ,        ADJ_VAR_FOLLOW_SPEED_YAW
        ,        ADJ_VAR_FOLLOW_LPF_ROLL
        ,        ADJ_VAR_FOLLOW_LPF_PITCH
        ,        ADJ_VAR_FOLLOW_LPF_YAW
        ,        ADJ_VAR_RC_SPEED_ROLL
        ,        ADJ_VAR_RC_SPEED_PITCH
        ,        ADJ_VAR_RC_SPEED_YAW
        ,        ADJ_VAR_RC_LPF_ROLL
        ,        ADJ_VAR_RC_LPF_PITCH
        ,        ADJ_VAR_RC_LPF_YAW
        ,        ADJ_VAR_RC_TRIM_ROLL
        ,        ADJ_VAR_RC_TRIM_PITCH
        ,        ADJ_VAR_RC_TRIM_YAW
        ,        ADJ_VAR_RC_DEADBAND
        ,        ADJ_VAR_RC_EXPO_RATE
        ,        ADJ_VAR_FOLLOW_MODE
        ,        ADJ_VAR_RC_FOLLOW_YAW
        ,        ADJ_VAR_FOLLOW_DEADBAND
        ,        ADJ_VAR_FOLLOW_EXPO_RATE
        ,        ADJ_VAR_FOLLOW_ROLL_MIX_START
        ,        ADJ_VAR_FOLLOW_ROLL_MIX_RANGE
        ,        ADJ_VAR_GYRO_TRUST
        , ADJ_VAR_FRAME_HEADING
        , ADJ_VAR_GYRO_HEADING_CORR
        , ADJ_VAL_ACC_LIMITER_ROLL
        , ADJ_VAL_ACC_LIMITER_PITCH
        , ADJ_VAL_ACC_LIMITER_YAW

} eAdjvar;
#define ADJ_VAR_NAME_MAX_LENGTH 10U
// Descriptors for adjustable variables  (interface to HMI)
typedef struct {

        uint8_t id;
        unsigned char name[10u];                                                 // max. 10 characters
        int32_t min_val;
        int32_t max_val;

} adjustable_var_cfg_t;                                                         // as per the defines shown below

#define ADJ_VAR_DEF_P_ROLL { ADJ_VAR_P_ROLL, "PID_P.R", 0, 255u }
#define ADJ_VAR_DEF_P_PITCH { ADJ_VAR_P_PITCH, "PID_P.P", 0, 255u }
#define ADJ_VAR_DEF_P_YAW { ADJ_VAR_P_YAW, "PID_P.Y", 0, 255u }
#define ADJ_VAR_DEF_I_ROLL { ADJ_VAR_I_ROLL, "PID_I.R", 0, 255u }
#define ADJ_VAR_DEF_I_PITCH { ADJ_VAR_I_PITCH, "PID_I.P", 0, 255u }
#define ADJ_VAR_DEF_I_YAW { ADJ_VAR_I_YAW, "PID_I.Y", 0, 255u }
#define ADJ_VAR_DEF_D_ROLL { ADJ_VAR_D_ROLL, "PID_D.R", 0, 255u }
#define ADJ_VAR_DEF_D_PITCH { ADJ_VAR_D_PITCH, "PID_D.P", 0, 255u }
#define ADJ_VAR_DEF_D_YAW { ADJ_VAR_D_YAW, "PID_D.Y", 0, 255u }
#define ADJ_VAR_DEF_POWER_ROLL { ADJ_VAR_POWER_ROLL, "POWER.R", 0, 255u }
#define ADJ_VAR_DEF_POWER_PITCH { ADJ_VAR_POWER_PITCH, "POWER.P", 0, 255u }
#define ADJ_VAR_DEF_POWER_YAW { ADJ_VAR_POWER_YAW, "POWER.Y", 0, 255u }
#define ADJ_VAR_DEF_ACC_LIMIT { ADJ_VAR_ACC_LIMITER, "ACC_LIMIT", 0, 200u}
#define ADJ_VAR_DEF_FOLLOW_SPEED_ROLL { ADJ_VAR_FOLLOW_SPEED_ROLL, "F_SPD.R", 0, 255u }
#define ADJ_VAR_DEF_FOLLOW_SPEED_PITCH { ADJ_VAR_FOLLOW_SPEED_PITCH, "F_SPD.P", 0, 255u }
#define ADJ_VAR_DEF_FOLLOW_SPEED_YAW { ADJ_VAR_FOLLOW_SPEED_YAW, "F_SPD.Y", 0, 255u }
#define ADJ_VAR_DEF_FOLLOW_LPF_ROLL { ADJ_VAR_FOLLOW_LPF_ROLL, "F_LPF.R", 0, 16u }
#define ADJ_VAR_DEF_FOLLOW_LPF_PITCH { ADJ_VAR_FOLLOW_LPF_PITCH, "F_LPF.P", 0, 16u }
#define ADJ_VAR_DEF_FOLLOW_LPF_YAW { ADJ_VAR_FOLLOW_LPF_YAW, "F_LPF.Y", 0, 16u }
#define ADJ_VAR_DEF_RC_SPEED_ROLL { ADJ_VAR_RC_SPEED_ROLL, "RC_SPD.R", 0, 255u }
#define ADJ_VAR_DEF_RC_SPEED_PITCH { ADJ_VAR_RC_SPEED_PITCH, "RC_SPD.P", 0, 255u }
#define ADJ_VAR_DEF_RC_SPEED_YAW { ADJ_VAR_RC_SPEED_YAW, "RC_SPD.Y", 0, 255u }
#define ADJ_VAR_DEF_RC_LPF_ROLL { ADJ_VAR_RC_LPF_ROLL, "RC_LPF.R", 0, 16u }
#define ADJ_VAR_DEF_RC_LPF_PITCH { ADJ_VAR_RC_LPF_PITCH, "RC_LPF.P", 0, 16u }
#define ADJ_VAR_DEF_RC_LPF_YAW { ADJ_VAR_RC_LPF_YAW, "RC_LPF.Y", 0, 16u }
#define ADJ_VAR_DEF_RC_TRIM_ROLL { ADJ_VAR_RC_TRIM_ROLL, "RC_TRIM.R", -127, 127u }
#define ADJ_VAR_DEF_RC_TRIM_PITCH { ADJ_VAR_RC_TRIM_PITCH, "RC_TRIM.P", -127, 127u }
#define ADJ_VAR_DEF_RC_TRIM_YAW { ADJ_VAR_RC_TRIM_YAW, "RC_TRIM.Y", -127, 127u }
#define ADJ_VAR_DEF_RC_DEADBAND { ADJ_VAR_RC_DEADBAND, "RC_DBND", 0, 255u }
#define ADJ_VAR_DEF_RC_EXPO_RATE { ADJ_VAR_RC_EXPO_RATE, "RC_EXPO", 0, 100u }
#define ADJ_VAR_DEF_FOLLOW_MODE { ADJ_VAR_FOLLOW_MODE, "F.MODE", 0, 2u }
#define ADJ_VAR_DEF_FOLLOW_DEADBAND { ADJ_VAR_FOLLOW_DEADBAND, "F.DBND", 0, 255u }
#define ADJ_VAR_DEF_FOLLOW_EXPO_RATE { ADJ_VAR_FOLLOW_EXPO_RATE, "F.EXPO", 0, 100u }
#define ADJ_VAR_DEF_FOLLOW_ROLL_MIX_START { ADJ_VAR_FOLLOW_ROLL_MIX_START, "F.RMS", 0, 90u }
#define ADJ_VAR_DEF_FOLLOW_ROLL_MIX_RANGE { ADJ_VAR_FOLLOW_ROLL_MIX_RANGE, "F.RMR", 0, 90u }
#define ADJ_VAR_DEF_GYRO_TRUST { ADJ_VAR_GYRO_TRUST, "GYRO_TRUST", 1, 255u}

// for Write file command
#define SBGC_FILE_NO_ERROR 0U
#define SBGC_FILE_ERR_EEPROM_FAULT 1U
#define SBGC_ERR_FILE_NOT_FOUND 2U
#define SBGC_ERR_FAT 3U
#define SBGC_ERR_NO_FREE_SPACE 4U
#define SBGC_ERR_FAT_IS_FULL 5U
#define SBGC_ERR_FILE_SIZE 6U
#define SBGC_FILE_ERR_CRC 7U
#define SBGC_FILE_ERR_LIMIT_REACHED 8U
#define SBGC_ERR_FILE_CORRUPTED 9U
#define SBGC_FILE_ERR_WRONG_PARAMS 10U

// For Read file command
#define SBGC_FILE_TYPE_SCRIPT 1U
#define SBGC_FILE_TYPE_IMU_CALIB 3U
#define SBGC_FILE_TYPE_COGGING_CORRECTION 4U
#define SBGC_FILE_TYPE_ADJ_VAR_LUT 5U
#define SBGC_FILE_TYPE_PROFILE_SET 6U
#define SBGC_FILE_TYPE_PARAMS 7U
#define SBGC_FILE_TYPE_TUNE 8U
#define SBGC_FILE_TYPE_CANDRV 10U

// For AHRS helper command bits 0-2 this is contained as various reply messages
#define SBGC_STATUS_DISABLED 0U                                                 // status of external IMU
#define SBGC_STATUS_NOT_CONNECTED 1U
#define SBGC_STATUS_UNKNOWN 2U
#define SBGC_STATUS_ERROR 3U
#define SBGC_STATUS_BAD 4U
#define SBGC_STATUS_COARSE 5U
#define SBGC_STATUS_GOOD 6U
#define SBGC_STATUS_FINE 7U

#define SBGC_STATUS_FLAG_BAD_MAG (1u<<6u)                                         // bits 3-7 of the byte
#define SBGC_STATUS_FLAG_NO_GPS_SIGNAL (1u<<7u)

#endif // __SBGC_adj_vars__