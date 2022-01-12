#ifndef __SBGC_CMD_HELPERS__                                                    // This file is to be included if not already
#define __SBGC_CMD_HELPERS__
/*
        SBGC_cmd_helpers.h : SimpleBGC Serial API  library - helpers to pack and parse command data
        More info: http://www.basecamelectronics.com/serialapi/

  Copyright (c) 2014-2015 Aleksei Moskalenko
  v1.1 Modified ACP Aviation October 2019...
  Ported and expanded by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland

  All rights reserved.
        See license info in the SBGC.h
*/
#ifdef __cplusplus
 extern "C" {
#endif

#include "definitions.h"                                                        // Common data for compiler
#include <stdint.h>                                                             // Interger type defines

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define SBGCPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define SBGCPACKED __attribute__((packed)) ALIGNED(1)                         /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define SBGCPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define SBGCPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define SBGCPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif
                                                                                // Declare the global structures for readback here....
/*extern struct SBGC_progress_auto_pid_t pidreadbk;                               //      Container for the PID readback message declared to be global to gc_events.c
extern struct SBGC_cmd_get_angles_t getangles;                                  //        Container for the GET_ANGLES readback message declared to be global to gc_events.c
extern struct SBGC_cmd_get_angles_ext_t getanglesext;                           //        Container for the GET_ANGLES_EXT readback message declared to be global to gc_events.c
extern struct SBGC_cmd_realtime_data_3 realdata3;                               //        Container for the REALTIME_DATA_3 readback message declared to be global to gc_events.c
extern struct SBGC_cmd_realtime_data_4 realdata4;                               //        Container for the REALTIME_DATA_4 readback message declared to be global to gc_events.c
extern struct SBGC_cmd_confirmation_t cmdconf;                                  //        Container for the CMD_CONF readback message declared to be global to gc_events.c
extern struct SBGC_cmd_error_t cmderror;                                        //        Container for the CMD_ERROR readback message declared to be global to gc_events.c
extern struct SBGC_cmd_setget_adj_vars_val_t getvar_rd;                         //        Container for the CMD_SET_ADJ_VARS_VAL command  (7 values deep)
extern struct SBGC_cmd_board_info_t boardinforb;                                //        Container for BOARD_INFO readback
extern struct SBGC_cmd_read_params_3_t readparam3;                              //        Container for config param read/write
extern struct SBGC_cmd_script_debug_t scriptstate;                              //        Container for debug from script run command
extern struct SBGC_cmd_event_t eventCmd;                                        //        Container for reading back status events generated
extern struct SBGC_realtime_data_custom_t realtimedatacust;                     //        Container for reading back realtime data custom message reply   */

//////////////// Units conversion /////////////////

#define SBGC_ANGLE_FULL_TURN 16384

// Conversion from degree/sec to units that command understand

#define SBGC_SPEED_SCALE  (1.0f/0.1220740379f)                                  // Speed Scale when CONTROL_FLAG_HIGH_RES_SPEED not set
#define SBGC_DEGREE_ANGLE_SCALE ((float)SBGC_ANGLE_FULL_TURN/360.0f)
#define SBGC_ANGLE_DEGREE_SCALE (360.0f/(float)SBGC_ANGLE_FULL_TURN)

#define SBGC_SLOW_SPEED_SCALE  (1.0f/0.001f)                                    // Speed Scale when CONTROL_FLAG_HIGH_RES_SPEED set
#define SBGC_MAX_GIMBAL_SPEED 200                                               // Max Speed 200 deg /sec
#define SBGC_MIN_GIMBAL_SPEED 0                                                 // Min Speed 0 deg/sec

#define SBGC_GYRO_TO_DEGREE(val) ((float32_t) ((val)* 0.06103701895f))          // gyro values to deg/sec
#define SBGC_ACC_DATA(val) ((float32_t) ((val) / 512f))                         // acc data to G
#define SBGC_BAT_VOLTS(val) ((float32_t) ((val)* 0.001f))                       // battery volts

#define SBGC_VALUE_TO_DEGREE(d) ((float)((d * 720) >> 15))
#define SBGC_DEGREE_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/0.02197265625f)))
#define SBGC_DEGREE_PER_SEC_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/0.1220740379f)))

// Number of script slots available
#define SBGC_NO_OF_SLOTS 4u

// Extreme angles in degrees, that corresponds to 0..Vcc analog input range
#define PITCH_ANGLE_MIN -60
#define PITCH_ANGLE_MAX 60
#define YAW_ANGLE_MIN -60
#define YAW_ANGLE_MAX 60
#define ROLL_ANGLE_MIN -60
#define ROLL_ANGLE_MAX 60
//c.anglePITCH=SBGC_DEGREE_TO_ANGLE(60);                                        // Set pitch angle to value
#define RC_MIN -500
#define RC_MAX 500
#define RC_MAX_FAST 16384
#define RC_MIN_FAST -16384

// Conversions for angle in degrees to angle in SBGC 14bit representation, and back

#define SBGC_DEGREE_TO_ANGLE(val) ((val)*SBGC_DEGREE_ANGLE_SCALE)
#define SBGC_ANGLE_TO_DEGREE(val) ((val)*SBGC_ANGLE_DEGREE_SCALE)

// The same, optimized for integers

#define SBGC_DEGREE_TO_ANGLE_INT(val) ((int32_t)(val)*SBGC_ANGLE_FULL_TURN/360)
#define SBGC_DEGREE_01_TO_ANGLE_INT(val) ((int32_t)(val)*SBGC_ANGLE_FULL_TURN/3600)
#define SBGC_ANGLE_TO_DEGREE_INT(val) ((int32_t)(val)*360/SBGC_ANGLE_FULL_TURN)
#define SBGC_ANGLE_TO_DEGREE_01_INT(val) ((int32_t)(val)*3600/SBGC_ANGLE_FULL_TURN)
#define SBGC_PID_TO_COMMAND(val) ((int32_t) (val)*46.6666666666f)

#define ROLL 0u
#define PITCH 1u
#define YAW 2u

#define SBGC_EULER_TO_DEGREE(val) (float64_t) (((val)/(2.0f*PI)) * 360.0f)      // For converting EULER into DEGREE
#define SBGC_DEGREE_TO_EULER(val) (float64_t) (((2.0f*PI)*(val)) / 360.0f)      // For coverting DEGREE to EULER

#define noteC3 0x82u                                                             // Define some simple notes for use with the beepwer (refer freq note tables for more)
#define noteC3sh 0x88u
#define noteD3 0x92u
#define noteD3sh 0x9Cu
#define noteE3 0xA4u
#define noteF3 0xAEu
#define noteF3sh 0x89u
#define noteG3 0xC4u
#define noteG3sh 0xD0u
#define noteB3 0xF6u
#define noteA3 0xDCu
#define noteA3sh 0xD9u
#define noteC4 0x105u
#define noteC4ah 0x115u
#define noteD4 0x125u
#define noteC_1 0x08u
#define noteD_1 0x09u
#define noteE_1 0x0Au
#define noteF_1 0x0Bu
#define noteG_1 0x0Bu
#define noteA_1 0x0Du
#define noteB_1 0x0Eu
#define noteC0 0x10u
#define noteD0 0x12u
#define noteE0 0x15u
#define noteF0 0x16u
#define noteG0 0x19u
#define noteA0 0x1Cu
#define noteB0 0x1Fu
#define noteC1 0x21u
#define noteD1 0x25u
#define noteE1 0x29u
#define noteF1 0x2Cu
#define noteG1 0x31u
#define noteA1 0x37u
#define noteB1 0x3Eu
#define noteC2 0x41u
#define noteD2 0x49u
#define noteE2 0x52u
#define noteF2 0x57u
#define noteG2 0x62u
#define noteA2 0x6Eu
#define noteB2 0x7Bu
#define SBGC_MINOR_SECOND(val) ((int8_t)(val)*1.0595f)                          // Macro for other notes
#define SBGC_MAJOR_SECOND(val) ((int8_t)(val)*1.225f)
#define SBGC_MINOR_THIRD(val) ((int8_t)(val)*1.1892f)
#define SBGC_MAJOR_THIRD(val) ((int8_t)(val)*1.2599f)
#define SBGC_PERFECT_FOURTH(val) ((int8_t)(val)*1.13348f)
#define SBGC_TRITONE(val) ((int8_t)(val)*sqrt(2f))
#define SBGC_PERFECT_FIFTH(val) ((int8_t)(val)*1.5f)
#define SBGC_MINOR_SIXTH(val) ((int8_t)(val)*1.6f)
#define SBGC_MAJOR_SIXTH(val) ((int8_t)(val)*1.66667f)
#define SBGC_MINOR_SEVENTH(val) ((int8_t)(val)*1.75f)
#define SBGC_MAJOR_SEVENTH(val) ((int8_t)(val)*1.83333f)
#define SBGC_OCTAVE(val) ((int8_t)(val)*2.0f)

// define the serial recieve states
#define SBGC_SER_GET_ANGLES (1u<<0u)
#define SBGC_SER_AUTO_PID (1u<<1u)
#define SBGC_SER_GET_ANGLES_EXT (1u<<2u)
#define SBGC_SER_REALTIME_DATA_3 (1u<<3u)
#define SBGC_SER_REALTIME_DATA_4 (1u<<4u)
#define SBGC_SER_CONFIRM (1u<<5u)
#define SBGC_SER_ERROR (1u<<6u)
#define SBGC_SER_SET_ADJ_VARS_VAL (1u<<7u)
#define SBGC_SER_BOARD_INFO (1u<<8u)
#define SBGC_SER_READ_PARAMS_3 (1u<<9u)
#define SBGC_SER_SCRIPT_DEBUG (1u<<10u)
#define SBGC_SER_EVENT (1u<<11u)
#define SBGC_SER_REALTIME_DATA_CUSTOM (1u<<12u)

#if defined(D_FT900)
typedef struct SBGCPACKED {
       uint8_t stxCHAR;                                                         // Start Char >
       uint8_t commandID;                                                       // Command ID
       uint8_t sizeData;                                                        // Size of data
       uint8_t cmdSize;                                                         // command id + size modula 256
} SBGC_cmd_header_t;                                                            // structure for a SBGC header part of the message.
#else                                                                           /* e.g. __GBUC__   */
SBGCPACKED(
typedef struct {
       uint8_t stxCHAR;                                                         // Start Char >
       uint8_t commandID;                                                       // Command ID
       uint8_t sizeData;                                                        // Size of data
       uint8_t cmdSize;                                                         // command id + size modula 256
}) SBGC_cmd_header_t;                                                           // structure for a SBGC header part of the message.
#endif

// CMD_GET_ANGLES (incoming message)
#if defined(D_FT900)
typedef struct SBGCPACKED {
        int16_t imu_angle_roll;
        int16_t target_angle_roll;
        int16_t target_speed_roll;
        int16_t imu_angle_pitch;
        int16_t target_angle_pitch;
        int16_t target_speed_pitch;
        int16_t imu_angle_yaw;
        int16_t target_angle_yaw;
        int16_t target_speed_yaw;
} SBGC_cmd_get_angles_t;
#else
SBGCPACKED(
typedef struct {
        int16_t imu_angle_roll;
        int16_t target_angle_roll;
        int16_t target_speed_roll;
        int16_t imu_angle_pitch;
        int16_t target_angle_pitch;
        int16_t target_speed_pitch;
        int16_t imu_angle_yaw;
        int16_t target_angle_yaw;
        int16_t target_speed_yaw;
}) SBGC_cmd_get_angles_t;
#endif

// CMD_GET_ANGLES EXT (incoming message)
#if defined(D_FT900)
typedef struct SBGCPACKED {

        int16_t imu_angle_roll;
        int16_t target_angle_roll;
        int32_t stator_rotor_angle_roll;
        unsigned char reserved_roll[10u];                                                    // 10 bytes reserved.
        int16_t imu_angle_pitch;
        int16_t target_angle_pitch;
        int32_t stator_rotor_angle_pitch;
        unsigned char reserved_pitch[10u];                                                    // 10 bytes reserved.
        int16_t imu_angle_yaw;
        int16_t target_angle_yaw;
        int32_t stator_rotor_angle_yaw;
        unsigned char reserved_yaw[10u];                                                    // 10 bytes reserved.

}  SBGC_cmd_get_angles_ext_t;
#else
// CMD_GET_ANGLES EXT (incoming message)
SBGCPACKED(
typedef struct {

        int16_t imu_angle_roll;
        int16_t target_angle_roll;
        int32_t stator_rotor_angle_roll;
        unsigned char reserved_roll[10u];                                                    // 10 bytes reserved.
        int16_t imu_angle_pitch;
        int16_t target_angle_pitch;
        int32_t stator_rotor_angle_pitch;
        unsigned char reserved_pitch[10u];                                                    // 10 bytes reserved.
        int16_t imu_angle_yaw;
        int16_t target_angle_yaw;
        int32_t stator_rotor_angle_yaw;
        unsigned char reserved_yaw[10u];                                                    // 10 bytes reserved.

} ) SBGC_cmd_get_angles_ext_t;
#endif

// CMD_AUTO_PID (incoming message) response to a type=8 GUI request.
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t pband_roll;
        uint8_t integral_roll;
        uint8_t derivative_roll;
        uint16_t lpf_roll;
        uint8_t pband_pitch;
        uint8_t integral_pitch;
        uint8_t derivative_pitch;
        uint16_t lpf_pitch;
        uint8_t pband_yaw;
        uint8_t integral_yaw;
        uint8_t derivative_yaw;
        uint16_t lpf_yaw;
        uint16_t iteration_count;
        float32_t tracking_error_roll;
        float32_t tracking_error_pitch;
        float32_t tracking_error_yaw;
        unsigned char reserved_roll[6u];                                               // 6 bytes reserved.
        unsigned char reserved_pitch[6u];                                              // 6 bytes reserved.
        unsigned char reserved_yaw[6u];                                                // 6 bytes reserved.
        unsigned char reserved[10u];                                                    // 10 bytes reserved.

} SBGC_progress_auto_pid_t;
#else
// CMD_AUTO_PID (incoming message) response to a type=8 GUI request.
SBGCPACKED(
typedef struct {
        uint8_t pband_roll;
        uint8_t integral_roll;
        uint8_t derivative_roll;
        uint16_t lpf_roll;
        uint8_t pband_pitch;
        uint8_t integral_pitch;
        uint8_t derivative_pitch;
        uint16_t lpf_pitch;
        uint8_t pband_yaw;
        uint8_t integral_yaw;
        uint8_t derivative_yaw;
        uint16_t lpf_yaw;
        uint16_t iteration_count;
        float32_t tracking_error_roll;
        float32_t tracking_error_pitch;
        float32_t tracking_error_yaw;
        unsigned char reserved_roll[6u];                                               // 6 bytes reserved.
        unsigned char reserved_pitch[6u];                                              // 6 bytes reserved.
        unsigned char reserved_yaw[6u];                                                // 6 bytes reserved.
        unsigned char reserved[10u];                                                    // 10 bytes reserved.
}) SBGC_progress_auto_pid_t;
#endif

// CMD_REALTIME_DATA_3 (incoming message) response to ???
#if defined(D_FT900)
typedef struct SBGCPACKED {
        int16_t acc_data_roll;
        int16_t gyro_data_roll;
        int16_t acc_data_pitch;
        int16_t gyro_data_pitch;
        int16_t acc_data_yaw;
        int16_t gyro_data_yaw;
        uint16_t serial_error_cnt;
        uint16_t system_error;
        uint8_t system_sub_error;
        unsigned char reserved1[3u];                                                   // 3 bytes reserved.
        int16_t rc_roll;
        int16_t rc_pitch;
        int16_t rc_yaw;
        int16_t rc_cmd;
        int16_t ext_fc_roll;
        int16_t ext_fc_pitch;
        int16_t imu_angle_roll;
        int16_t imu_angle_pitch;
        int16_t imu_angle_yaw;
        int16_t frame_imu_angle_roll;
        int16_t frame_imu_angle_pitch;
        int16_t frame_imu_angle_yaw;
        int16_t target_angle_roll;
        int16_t target_angle_pitch;
        int16_t target_angle_yaw;
        uint16_t cycle_time;
        uint16_t i2c_error_count;
        uint8_t error_code;
        uint16_t bat_level;
        uint8_t rt_data_flags;
        uint8_t cur_imu;
        uint8_t cur_profile;
        uint8_t motor_power_roll;
        uint8_t motor_power_pitch;
        uint8_t motor_power_yaw;
} SBGC_cmd_realtime_data_3;
#else
// CMD_REALTIME_DATA_3 (incoming message) response to ???
SBGCPACKED(
typedef struct {
        int16_t acc_data_roll;
        int16_t gyro_data_roll;
        int16_t acc_data_pitch;
        int16_t gyro_data_pitch;
        int16_t acc_data_yaw;
        int16_t gyro_data_yaw;
        uint16_t serial_error_cnt;
        uint16_t system_error;
        uint8_t system_sub_error;
        unsigned char reserved1[3u];                                                   // 3 bytes reserved.
        int16_t rc_roll;
        int16_t rc_pitch;
        int16_t rc_yaw;
        int16_t rc_cmd;
        int16_t ext_fc_roll;
        int16_t ext_fc_pitch;
        int16_t imu_angle_roll;
        int16_t imu_angle_pitch;
        int16_t imu_angle_yaw;
        int16_t frame_imu_angle_roll;
        int16_t frame_imu_angle_pitch;
        int16_t frame_imu_angle_yaw;
        int16_t target_angle_roll;
        int16_t target_angle_pitch;
        int16_t target_angle_yaw;
        uint16_t cycle_time;
        uint16_t i2c_error_count;
        uint8_t error_code;
        uint16_t bat_level;
        uint8_t rt_data_flags;
        uint8_t cur_imu;
        uint8_t cur_profile;
        uint8_t motor_power_roll;
        uint8_t motor_power_pitch;
        uint8_t motor_power_yaw;
}) SBGC_cmd_realtime_data_3;
#endif

// CMD_REALTIME_DATA_4 (incoming message) response to ???
#if defined(D_FT900)
typedef struct SBGCPACKED {
        int16_t acc_data_roll;
        int16_t gyro_data_roll;
        int16_t acc_data_pitch;
        int16_t gyro_data_pitch;
        int16_t acc_data_yaw;
        int16_t gyro_data_yaw;
        uint16_t serial_error_cnt;
        uint16_t system_error;
        uint8_t system_sub_error;
        unsigned char reserved1[3u];                                                   // 3 bytes reserved
        int16_t rc_roll;
        int16_t rc_pitch;
        int16_t rc_yaw;
        int16_t rc_cmd;
        int16_t ext_fc_roll;
        int16_t ext_fc_pitch;
        int16_t imu_angle_roll;
        int16_t imu_angle_pitch;
        int16_t imu_angle_yaw;
        int16_t frame_imu_angle_roll;
        int16_t frame_imu_angle_pitch;
        int16_t frame_imu_angle_yaw;
        int16_t target_angle_roll;
        int16_t target_angle_pitch;
        int16_t target_angle_yaw;
        uint16_t cycle_time;
        uint16_t i2c_error_count;
        uint8_t error_code;
        uint16_t bat_level;
        uint8_t rt_data_flags;
        uint8_t cur_imu;
        uint8_t cur_profile;
        uint8_t motor_power_roll;
        uint8_t motor_power_pitch;
        uint8_t motor_power_yaw;
        int16_t rotor_angle_roll;
        int16_t rotor_angle_pitch;
        int16_t rotor_angle_yaw;
        unsigned char reserved2;
        int16_t balance_error_roll;
        int16_t balance_error_pitch;
        int16_t balance_error_yaw;
        uint16_t current;
        int16_t mag_data_roll;
        int16_t mag_data_pitch;
        int16_t mag_data_yaw;
        int8_t imu_temp;
        int8_t frame_imu_temp;
        uint8_t imu_g_error;
        uint8_t imu_h_error;
        int16_t motor_out_roll;
        int16_t motor_out_pitch;
        int16_t motor_out_yaw;
        unsigned char reserved3[30u];                                            // reserve 30 bytes at the end for future.
} SBGC_cmd_realtime_data_4;
#else
// CMD_REALTIME_DATA_4 (incoming message) response to ???
SBGCPACKED(
typedef struct {
        int16_t acc_data_roll;
        int16_t gyro_data_roll;
        int16_t acc_data_pitch;
        int16_t gyro_data_pitch;
        int16_t acc_data_yaw;
        int16_t gyro_data_yaw;
        uint16_t serial_error_cnt;
        uint16_t system_error;
        uint8_t system_sub_error;
        unsigned char reserved1[3u];                                                   // 3 bytes reserved
        int16_t rc_roll;
        int16_t rc_pitch;
        int16_t rc_yaw;
        int16_t rc_cmd;
        int16_t ext_fc_roll;
        int16_t ext_fc_pitch;
        int16_t imu_angle_roll;
        int16_t imu_angle_pitch;
        int16_t imu_angle_yaw;
        int16_t frame_imu_angle_roll;
        int16_t frame_imu_angle_pitch;
        int16_t frame_imu_angle_yaw;
        int16_t target_angle_roll;
        int16_t target_angle_pitch;
        int16_t target_angle_yaw;
        uint16_t cycle_time;
        uint16_t i2c_error_count;
        uint8_t error_code;
        uint16_t bat_level;
        uint8_t rt_data_flags;
        uint8_t cur_imu;
        uint8_t cur_profile;
        uint8_t motor_power_roll;
        uint8_t motor_power_pitch;
        uint8_t motor_power_yaw;
        int16_t rotor_angle_roll;
        int16_t rotor_angle_pitch;
        int16_t rotor_angle_yaw;
        unsigned char reserved2;
        int16_t balance_error_roll;
        int16_t balance_error_pitch;
        int16_t balance_error_yaw;
        uint16_t current;
        int16_t mag_data_roll;
        int16_t mag_data_pitch;
        int16_t mag_data_yaw;
        int8_t imu_temp;
        int8_t frame_imu_temp;
        uint8_t imu_g_error;
        uint8_t imu_h_error;
        int16_t motor_out_roll;
        int16_t motor_out_pitch;
        int16_t motor_out_yaw;
        unsigned char reserved3[30u];                                            // reserve 30 bytes at the end for future.
}) SBGC_cmd_realtime_data_4;
#endif

// CMD CONFIRM (readback reply)
#if defined(D_FT900)
typedef struct SBGCPACKED {
uint8_t cmd_id;                                                                 // command ID
uint16_t reply;                                                                 // confirmation reply
} SBGC_cmd_confirmation_t;
#else
// CMD CONFIRM (readback reply)
SBGCPACKED(
typedef struct {
uint8_t cmd_id;                                                                 // command ID
uint16_t reply;                                                                 // confirmation reply
} )SBGC_cmd_confirmation_t;
#endif

// CMD ERROR (readback reply)
#if defined(D_FT900)
typedef struct SBGCPACKED {
uint8_t cmd_id;                                                                 // command ID
uint32_t reply;                                                                 // confirmation error codes up to 4 bytes.
} SBGC_cmd_error_t;
#else
// CMD ERROR (readback reply)
SBGCPACKED(
typedef struct {
uint8_t cmd_id;                                                                 // command ID
uint32_t reply;                                                                 // confirmation error codes up to 4 bytes.
} ) SBGC_cmd_error_t;
#endif

// CMD RESET
#if defined(D_FT900)
typedef struct SBGCPACKED {
int16_t confirmRequired;                                                        // 1=confirm required
uint32_t delayMS;                                                               // delay MS
} SBGC_cmd_reset_t;
#else
// CMD RESET
SBGCPACKED(
typedef struct {
int16_t confirmRequired;                                                        // 1=confirm required
uint32_t delayMS;                                                               // delay MS
}) SBGC_cmd_reset_t;
#endif


// CMD_CONTROL
#if defined(D_FT900)
typedef struct SBGCPACKED {
  uint8_t mode;
  int16_t speedROLL;
  int16_t angleROLL;
  int16_t speedPITCH;
  int16_t anglePITCH;
  int16_t speedYAW;
  int16_t angleYAW;
} SBGC_cmd_control_t;
#else
// CMD_CONTROL
SBGCPACKED(
typedef struct {
  uint8_t mode;
  int16_t speedROLL;
  int16_t angleROLL;
  int16_t speedPITCH;
  int16_t anglePITCH;
  int16_t speedYAW;
  int16_t angleYAW;
}) SBGC_cmd_control_t;
#endif

// CMD_CONTROL  version > 2.55b5
#if defined(D_FT900)
typedef struct SBGCPACKED {
  uint8_t modeROLL;
  uint8_t modePITCH;
  uint8_t modeYAW;
  int16_t speedROLL;
  int16_t angleROLL;
  int16_t speedPITCH;
  int16_t anglePITCH;
  int16_t speedYAW;
  int16_t angleYAW;
} SBGC_cmd_control_new_t;
#else
// CMD_CONTROL  version > 2.55b5
SBGCPACKED(
typedef struct {
  uint8_t modeROLL;
  uint8_t modePITCH;
  uint8_t modeYAW;
  int16_t speedROLL;
  int16_t angleROLL;
  int16_t speedPITCH;
  int16_t anglePITCH;
  int16_t speedYAW;
  int16_t angleYAW;
}) SBGC_cmd_control_new_t;
#endif


//void SBGC_cmd_control_pack(SBGC_cmd_control_t &p, SerialCommand &cmd);
//inline uint8_t SBGC_cmd_control_send(SBGC_cmd_control_t &p, SBGC_Parser &parser) {
//       SerialCommand cmd;
//        SBGC_cmd_control_pack(p, cmd);
//        return parser.send_cmd(cmd);
//}

// CMD_CONTROL (extended version)
//typedef struct {
//  unsigned short mode[3];
//struct {
//          signed int angle;
//          signed int speed;
//} data[3];
//} SBGC_cmd_control_ext_t;

//void SBGC_cmd_control_ext_pack(SBGC_cmd_control_ext_t &p, SerialCommand &cmd);
//inline uint8_t SBGC_cmd_control_ext_send(SBGC_cmd_control_ext_t &p, SBGC_Parser &parser) {
//        SerialCommand cmd;
//        SBGC_cmd_control_ext_pack(p, cmd);
//        return parser.send_cmd(cmd);
//}


// CMD_API_VIRT_CH_CONTROL
#if defined(D_FT900)
typedef struct SBGCPACKED {
       int16_t VCdata[SBGC_API_VIRT_NUM_CHANNELS];
} SBGC_cmd_api_virt_ch_control_t;
#else
// CMD_API_VIRT_CH_CONTROL
SBGCPACKED(
typedef struct {
       int16_t VCdata[SBGC_API_VIRT_NUM_CHANNELS];
}) SBGC_cmd_api_virt_ch_control_t;
#endif

// CMD EXECUTE MENU
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t cmd_id;
} SBGC_cmd_execute_menu_t;
#else
// CMD EXECUTE MENU
SBGCPACKED(
typedef struct {
        uint8_t cmd_id;
}) SBGC_cmd_execute_menu_t;
#endif

// CMD SELECT IMU
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t cmd_id;
} SBGC_cmd_select_imu_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t cmd_id;
} ) SBGC_cmd_select_imu_t;
#endif

// CMD_SET_ADJ_VARS_VAL  for example 3 values can be set using this structure.
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t num_params;
        uint8_t param1_id;
        uint32_t param1_value;
        uint8_t param2_id;
        uint32_t param2_value;
        uint8_t param3_id;
        uint32_t param3_value;
        uint8_t param4_id;
        uint32_t param4_value;
        uint8_t param5_id;
        uint32_t param5_value;
} SBGC_cmd_set_adj_vars_val_t;
#else
// CMD_SET_ADJ_VARS_VAL  for example 3 values can be set using this structure.
SBGCPACKED(
typedef struct {
        uint8_t num_params;
        uint8_t param1_id;
        uint32_t param1_value;
        uint8_t param2_id;
        uint32_t param2_value;
        uint8_t param3_id;
        uint32_t param3_value;
        uint8_t param4_id;
        uint32_t param4_value;
        uint8_t param5_id;
        uint32_t param5_value;
} )SBGC_cmd_set_adj_vars_val_t;                                                 /* max is 40 parameters we have made ability to have up to 5 at present here */
#endif

// CMD_SET_ADJ_VARS_VAL  for example 7 values can be set/read using this structure. its response is to the get command which is 7 deep
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t num_params;
        uint8_t param1_id;
        uint32_t param1_value;
        uint8_t param2_id;
        uint32_t param2_value;
        uint8_t param3_id;
        uint32_t param3_value;
        uint8_t param4_id;
        uint32_t param4_value;
        uint8_t param5_id;
        uint32_t param5_value;
        uint8_t param6_id;
        uint32_t param6_value;
        uint8_t param7_id;
        uint32_t param7_value;
} SBGC_cmd_setget_adj_vars_val_t;
#else
// CMD_SET_ADJ_VARS_VAL  for example 7 values can be set/read using this structure. its response is to the get command which is 7 deep
SBGCPACKED(
typedef struct {
        uint8_t num_params;
        uint8_t param1_id;
        uint32_t param1_value;
        uint8_t param2_id;
        uint32_t param2_value;
        uint8_t param3_id;
        uint32_t param3_value;
        uint8_t param4_id;
        uint32_t param4_value;
        uint8_t param5_id;
        uint32_t param5_value;
        uint8_t param6_id;
        uint32_t param6_value;
        uint8_t param7_id;
        uint32_t param7_value;
} ) SBGC_cmd_setget_adj_vars_val_t;
#endif

// CMD_GET_ADJ_VARS_VAL  for example 7 values can be read using this structure.  (response is a CMD_SET_ADJ_VARS_VAL)
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t num_params;
        uint8_t param1_id;
        uint8_t param2_id;
        uint8_t param3_id;
        uint8_t param4_id;
        uint8_t param5_id;
        uint8_t param6_id;
        uint8_t param7_id;
} SBGC_cmd_get_adj_vars_val_t;
#else
// CMD_GET_ADJ_VARS_VAL  for example 7 values can be read using this structure.  (response is a CMD_SET_ADJ_VARS_VAL)
SBGCPACKED(
typedef struct {
        uint8_t num_params;
        uint8_t param1_id;
        uint8_t param2_id;
        uint8_t param3_id;
        uint8_t param4_id;
        uint8_t param5_id;
        uint8_t param6_id;
        uint8_t param7_id;
} ) SBGC_cmd_get_adj_vars_val_t;
#endif

//void SBGC_cmd_api_virt_ch_control_pack(SBGC_cmd_api_virt_ch_control_t &p, SerialCommand &cmd);
//inline uint8_t SBGC_cmd_api_virt_ch_control_send(SBGC_cmd_api_virt_ch_control_t &p, SBGC_Parser &parser) {
//        SerialCommand cmd;
//        SBGC_cmd_api_virt_ch_control_pack(p, cmd);
//        return parser.send_cmd(cmd);
//}

#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t pin;
        int8_t state;
} SBGC_cmd_trigger_t;
#else
// CMD_TRIGGER_PIN
SBGCPACKED(
typedef struct {
        uint8_t pin;
        int8_t state;
}) SBGC_cmd_trigger_t;
#endif

#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t profile_id;
        uint8_t cfg_flags;
        uint8_t gain_vs_stability;
        uint8_t momentum;
        uint8_t action;
        unsigned char reserved[14u];                                             // 14 bytes reserved.
} SBGC_cmd_pid_t;
#else
// CMD_AUTO_PID
SBGCPACKED(
typedef struct {
        uint8_t profile_id;
        uint8_t cfg_flags;
        uint8_t gain_vs_stability;
        uint8_t momentum;
        uint8_t action;
        unsigned char reserved[14u];                                             // 14 bytes reserved.
}) SBGC_cmd_pid_t;
#endif

// CMD AUTO PID version 2.
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t action;
        unsigned char reserved1[10u];                                            // 10 bytes reserved.
        uint8_t cfg_version;
        uint8_t axis_flags[3u];                                                  // These are for all 3 axis
        uint8_t gain[3u];
        uint8_t stimulus[3u];
        uint8_t effective_freq[3u];
        uint8_t problem_freq[3u];
        uint8_t problem_margin[3u];
        unsigned char reserved2[6u][3u];                                          // 6 bytes reserved.
        uint16_t general_flags;
        uint8_t startup_cfg;
        unsigned char reserved3[22u];                                            // 22 bytes reserved.
} SBGC_cmd_pid2_t;
#else
// CMD AUTO PID version 2.
SBGCPACKED(
typedef struct {
        uint8_t action;
        unsigned char reserved1[10u];                                            // 10 bytes reserved.
        uint8_t cfg_version;
        uint8_t axis_flags[3u];                                                  // These are for all 3 axis
        uint8_t gain[3u];
        uint8_t stimulus[3u];
        uint8_t effective_freq[3u];
        uint8_t problem_freq[3u];
        uint8_t problem_margin[3u];
        unsigned char reserved2[6u][3u];                                          // 6 bytes reserved.
        uint16_t general_flags;
        uint8_t startup_cfg;
        unsigned char reserved3[22u];                                            // 22 bytes reserved.
}) SBGC_cmd_pid2_t;
#endif

// CMD CONTROL_CONFIG (information to PID auto tune)
#if defined(D_FT900)
typedef struct SBGCPACKED {
            uint16_t timeout_ms;
        uint8_t CH1_PRIORITY;
        uint8_t CH2_PRIORITY;
        uint8_t CH3_PRIORITY;
        uint8_t CH4_PRIORITY;
        uint8_t THIS_CH_PRIORITY;
        uint8_t ANGLE_LPF_ROLL;
        uint8_t SPEED_LPF_ROLL;
        uint8_t RC_LPF_ROLL;
        unsigned char RESERVED_ROLL[4u];
        uint8_t ANGLE_LPF_PITCH;
        uint8_t SPEED_LPF_PITCH;
        uint8_t RC_LPF_PITCH;
        unsigned char RESERVED_PITCH[4u];
        uint8_t ANGLE_LPF_YAW;
        uint8_t SPEED_LPF_YAW;
        uint8_t RC_LPF_YAW;
        unsigned char RESERVED_YAW[4u];
        uint8_t RC_EXPO_RATE;
        uint16_t FLAGS;
        unsigned char reserved[10u];
} SBGC_cmd_control_config_t;
#else
SBGCPACKED(
typedef struct {
        uint16_t timeout_ms;
        uint8_t CH1_PRIORITY;
        uint8_t CH2_PRIORITY;
        uint8_t CH3_PRIORITY;
        uint8_t CH4_PRIORITY;
        uint8_t THIS_CH_PRIORITY;
        uint8_t ANGLE_LPF_ROLL;
        uint8_t SPEED_LPF_ROLL;
        uint8_t RC_LPF_ROLL;
        unsigned char RESERVED_ROLL[4u];
        uint8_t ANGLE_LPF_PITCH;
        uint8_t SPEED_LPF_PITCH;
        uint8_t RC_LPF_PITCH;
        unsigned char RESERVED_PITCH[4u];
        uint8_t ANGLE_LPF_YAW;
        uint8_t SPEED_LPF_YAW;
        uint8_t RC_LPF_YAW;
        unsigned char RESERVED_YAW[4u];
        uint8_t RC_EXPO_RATE;
        uint16_t FLAGS;
        unsigned char reserved[10u];
}) SBGC_cmd_control_config_t;
#endif

//void SBGC_cmd_trigger_pack(SBGC_cmd_trigger_t &p, SerialCommand &cmd);
//inline uint8_t SBGC_cmd_trigger_send(SBGC_cmd_trigger_t &p, SBGC_Parser &parser) {
//        SerialCommand cmd;
//        SBGC_cmd_trigger_pack(p, cmd);
//        return parser.send_cmd(cmd);
//}

// CMD_SERVO_OUT
#if defined(D_FT900)
typedef struct SBGCPACKED {
          uint16_t servo[8u];
} SBGC_cmd_servo_out_t;
#else
SBGCPACKED(
typedef struct {
      uint16_t servo[8u];
}) SBGC_cmd_servo_out_t;
#endif



//void SBGC_cmd_servo_out_pack(SBGC_cmd_servo_out_t &p, SerialCommand &cmd);
//inline uint8_t SBGC_cmd_servo_out_send(SBGC_cmd_servo_out_t &p, SBGC_Parser &parser) {
//        SerialCommand cmd;
//        SBGC_cmd_servo_out_pack(p, cmd);
//        return parser.send_cmd(cmd);
//}


//CMD_SET_ADJ_VARS_VAL
//typedef struct {
//        unsigned short id;
//        signed long val;
//} SBGC_cmd_set_adj_vars_var_t;
// Send events using CMD_DATA_STREAM_INTERVAL
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t cmd_id;
        uint16_t interval_ms;
        unsigned char config[8u];
        unsigned char reserved1[10u];
} SBGC_cmd_data_stream_interval_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t cmd_id;
        uint16_t interval_ms;
        unsigned char config[8u];
        unsigned char reserved1[10u];
}) SBGC_cmd_data_stream_interval_t;
#endif

// Readback container for CMD_EVENT sent using CMD_DATA_STREAM_INTERVAL
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t event_id;
        uint8_t event_type;
        unsigned char param1[2u];
} SBGC_cmd_event_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t event_id;
        uint8_t event_type;
        unsigned char param1[2u];
}) SBGC_cmd_event_t;
#endif

// CMD_REALTIME_DATA_CUSTOM request
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint32_t flags;
        unsigned char reserved1[6u];
} SBGC_cmd_realtime_data_custom_t;
#else
SBGCPACKED(
typedef struct {
        uint32_t flags;
        unsigned char reserved1[6u];
}) SBGC_cmd_realtime_data_custom_t;
#endif

// CMD_REALTIME_DATA_CUSTOM reply
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint16_t timestamp_ms;
        int16_t imu_angles[3u];
        int16_t target_angles[3u];
        int16_t target_speed[3u];
        int16_t stator_rotor_angles[3u];
        int16_t gyro_data[3u];
        int16_t rc_data[6u];
        float64_t z_vector[3u];
        float64_t h_vector[3u];
        int16_t rc_channels[18u];
        int16_t acc_data[3u];
        unsigned char ahs_debug_info[26u];
        unsigned char motor4_control[8u];
        unsigned char encoder_raw[3u][3u];
        float64_t imu_angles_rad[3u];
} SBGC_cmd_realtime_data_custom_reply_t;
#else
SBGCPACKED(
typedef struct {
        uint16_t timestamp_ms;
        int16_t imu_angles[3u];
        int16_t target_angles[3u];
        int16_t target_speed[3u];
        int16_t stator_rotor_angles[3u];
        int16_t gyro_data[3u];
        int16_t rc_data[6u];
        float64_t z_vector[3u];
        float64_t h_vector[3u];
        int16_t rc_channels[18u];
        int16_t acc_data[3u];
        unsigned char ahs_debug_info[26u];
        unsigned char motor4_control[8u];
        unsigned char encoder_raw[3u][3u];
        float64_t imu_angles_rad[3u];
}) SBGC_cmd_realtime_data_custom_reply_t;
#endif

//void SBGC_cmd_set_adj_vars_pack(SBGC_cmd_set_adj_vars_var_t vars[], uint8_t vars_num, SerialCommand &cmd);
//uint8_t SBGC_cmd_set_adj_vars_unpack(SBGC_cmd_set_adj_vars_var_t vars_buf[], uint8_t &vars_num, SerialCommand &cmd);
//inline uint8_t SBGC_cmd_set_adj_vars_send(SBGC_cmd_set_adj_vars_var_t vars[], uint8_t vars_num, SBGC_Parser &parser) {
//        SerialCommand cmd;
//        SBGC_cmd_set_adj_vars_pack(vars, vars_num, cmd);
//        return parser.send_cmd(cmd);
//}


// CMD_REALTIME_DATA_3, CMD_REALTIME_DATA_4
#if defined(D_FT900)
typedef struct SBGCPACKED {
      struct {
       int16_t acc_data;
       int16_t gyro_data;
       } sensor_data[3u];                                                        // ACC and Gyro sensor data (with calibration) for current IMU (see cur_imu field)
      int16_t serial_error_cnt;                                                 // counter for communication errors
      int16_t system_error;                                                     // system error flags, defined in SBGC_SYS_ERR_XX
      uint8_t reserved1[4u];
      int16_t rc_raw_data[SBGC_RC_NUM_CHANNELS];                                // RC signal in 1000..2000 range for ROLL, PITCH, YAW, CMD, EXT_ROLL, EXT_PITCH channels
      int16_t imu_angle[3u];                                                    // ROLL, PITCH, YAW Euler angles of a camera, 16384/360 degrees
      int16_t frame_imu_angle[3u];                                              // ROLL, PITCH, YAW Euler angles of a frame, if known
      int16_t target_angle[3u];                                                 // ROLL, PITCH, YAW target angle
      uint16_t cycle_time_us;                                                   // cycle time in us. Normally should be 800us
      uint16_t i2c_error_count;                                                 // I2C errors counter
      uint8_t error_code;                                                       // error code
      uint16_t battery_voltage;                                                 // units 0.01 V
      uint8_t state_flags1;                                                     // bit0: motor ON/OFF state;  bits1..7: reserved
      uint8_t cur_imu;                                                          // actually selecteted IMU for monitoring. 1: main IMU, 2: frame IMU
      uint8_t cur_profile;                                                      // active profile number starting from 0
      uint8_t motor_power[3u];                                                  // actual motor power for ROLL, PITCH, YAW axis, 0..255
        // Fields below are filled only for CMD_REALTIME_DATA_4 command
      int16_t rotor_angle[3u];                                                   // relative angle of each motor, 16384/360 degrees
      uint8_t reserved3;
      int16_t balance_error[3u];                                                // error in balance. Ranges from -512 to 512,  0 means perfect balance.
      uint16_t current;                                                         // Current that gimbal takes, in mA.
      int16_t magnetometer_data[3u];                                            // magnetometer sensor data (with calibration)
      int8_t  imu_temp_celcius;                                                 // temperature measured by the main IMU sensor, in Celsius
      int8_t  frame_imu_temp_celcius;                                           // temperature measured by the frame IMU sensor, in Celsius
      uint8_t reserved4[38u];
} SBGC_cmd_realtime_data_t;
#else
SBGCPACKED(
typedef struct {
      struct {
       int16_t acc_data;
       int16_t gyro_data;
       } sensor_data[3u];                                                        // ACC and Gyro sensor data (with calibration) for current IMU (see cur_imu field)
      int16_t serial_error_cnt;                                                 // counter for communication errors
      int16_t system_error;                                                     // system error flags, defined in SBGC_SYS_ERR_XX
      uint8_t reserved1[4u];
      int16_t rc_raw_data[SBGC_RC_NUM_CHANNELS];                                // RC signal in 1000..2000 range for ROLL, PITCH, YAW, CMD, EXT_ROLL, EXT_PITCH channels
      int16_t imu_angle[3u];                                                    // ROLL, PITCH, YAW Euler angles of a camera, 16384/360 degrees
      int16_t frame_imu_angle[3u];                                              // ROLL, PITCH, YAW Euler angles of a frame, if known
      int16_t target_angle[3u];                                                 // ROLL, PITCH, YAW target angle
      uint16_t cycle_time_us;                                                   // cycle time in us. Normally should be 800us
      uint16_t i2c_error_count;                                                 // I2C errors counter
      uint8_t error_code;                                                       // error code
      uint16_t battery_voltage;                                                 // units 0.01 V
      uint8_t state_flags1;                                                     // bit0: motor ON/OFF state;  bits1..7: reserved
      uint8_t cur_imu;                                                          // actually selecteted IMU for monitoring. 1: main IMU, 2: frame IMU
      uint8_t cur_profile;                                                      // active profile number starting from 0
      uint8_t motor_power[3u];                                                  // actual motor power for ROLL, PITCH, YAW axis, 0..255
        // Fields below are filled only for CMD_REALTIME_DATA_4 command
      int16_t rotor_angle[3u];                                                   // relative angle of each motor, 16384/360 degrees
      uint8_t reserved3;
      int16_t balance_error[3u];                                                // error in balance. Ranges from -512 to 512,  0 means perfect balance.
      uint16_t current;                                                         // Current that gimbal takes, in mA.
      int16_t magnetometer_data[3u];                                            // magnetometer sensor data (with calibration)
      int8_t  imu_temp_celcius;                                                 // temperature measured by the main IMU sensor, in Celsius
      int8_t  frame_imu_temp_celcius;                                           // temperature measured by the frame IMU sensor, in Celsius
      uint8_t reserved4[38u];
}) SBGC_cmd_realtime_data_t;
#endif


//uint8_t SBGC_cmd_realtime_data_unpack(SBGC_cmd_realtime_data_t &p, SerialCommand &cmd);
//inline uint8_t SBGC_cmd_execute_menu_send(uint8_t menu_action, SBGC_Parser &parser) {
//        SerialCommand cmd;
//        cmd.init(SBGC_CMD_EXECUTE_MENU);
//        cmd.writeByte(menu_action);
//        return parser.send_cmd(cmd);
//}

// Calibration structures
// CMD_CALIB_ACC _GYRO _MAG (extended) done by GUI
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t imu_idx;
        uint8_t action;
        unsigned char reserved1[10u];
} SBGC_cmd_calib_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t imu_idx;
        uint8_t action;
        unsigned char reserved1[10u];
}) SBGC_cmd_calib_t;
#endif

// CMD_CALIB_ORIENT_CORR
#if defined(D_FT900)
typedef struct SBGCPACKED {
        unsigned char reserved1[16u];
} SBGC_cmd_calib_orient_t;
#else
SBGCPACKED(
typedef struct {
        unsigned char reserved1[16u];
}) SBGC_cmd_calib_orient_t;
#endif

//CMD_CALIB_ACC_EXT_REF
#if defined(D_FT900)
typedef struct SBGCPACKED {
        int16_t acc_ref_x;
        int16_t acc_ref_y;
        int16_t acc_ref_z;
        unsigned reserved1[14u];
} SBGC_cmd_calib_acc_ext_ref_t;
#else
SBGCPACKED(
typedef struct {
        int16_t acc_ref_x;
        int16_t acc_ref_y;
        int16_t acc_ref_z;
        unsigned reserved1[14u];
}) SBGC_cmd_calib_acc_ext_ref_t;
#endif

//CMD_ENCODERS_CALIB_FLD_OFFSET_4
#if defined(D_FT900)
typedef struct SBGCPACKED {
        int16_t calib_angle_x;
        int16_t calib_angle_y;
        int16_t calib_angle_z;
} SBGC_cmd_calib_field_offset_t;
#else
SBGCPACKED(
typedef struct {
        int16_t calib_angle_x;
        int16_t calib_angle_y;
        int16_t calib_angle_z;
}) SBGC_cmd_calib_field_offset_t;
#endif

// CMD_GYRO_CORRECTION
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t imu_type;
        int16_t gyro_zero_corr_x;
        int16_t gyro_zero_corr_y;
        int16_t gyro_zero_corr_z;
        int16_t gyro_zero_heading_corr;
} SBGC_cmd_gyro_correction_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t imu_type;
        int16_t gyro_zero_corr_x;
        int16_t gyro_zero_corr_y;
        int16_t gyro_zero_corr_z;
        int16_t gyro_zero_heading_corr;
}) SBGC_cmd_gyro_correction_t;
#endif

// CMD_CALIB_BAT
#if defined(D_FT900)
typedef struct SBGCPACKED {
      uint16_t actual_voltage;
} SBGC_calib_bat;
#else
SBGCPACKED(
typedef struct {
      uint16_t actual_voltage;
}) SBGC_calib_bat;
#endif

// CMD_CALIB_INFO
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t progress;
        uint8_t imu_type;
        int16_t acc_data[3u];
        uint16_t gyro_abs_value;
        uint8_t acc_curr_axis;
        uint8_t acc_limits_info;
        int8_t imu_temp_cells;
        uint8_t temp_calib_gyro_enabled;
        int8_t temp_calib_gyro_t_min_cels;
        int8_t temp_calib_gyro_t_max_cels;
        uint8_t temp_calib_acc_enabled;
        uint8_t temp_calib_acc_slot_num[6u];
        int8_t temp_calib_acc_t_min_cels;
        int8_t temp_calib_acc_t_max_cels;
        uint8_t h_err_length;
        unsigned char reserved[7u];
} SBGC_cmd_calib_info_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t progress;
        uint8_t imu_type;
        int16_t acc_data[3u];
        uint16_t gyro_abs_value;
        uint8_t acc_curr_axis;
        uint8_t acc_limits_info;
        int8_t imu_temp_cells;
        uint8_t temp_calib_gyro_enabled;
        int8_t temp_calib_gyro_t_min_cels;
        int8_t temp_calib_gyro_t_max_cels;
        uint8_t temp_calib_acc_enabled;
        uint8_t temp_calib_acc_slot_num[6u];
        int8_t temp_calib_acc_t_min_cels;
        int8_t temp_calib_acc_t_max_cels;
        uint8_t h_err_length;
        unsigned char reserved[7u];
}) SBGC_cmd_calib_info_t;
#endif

// CMD_BEEP_SOUND
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint16_t mode;
        uint8_t note_length;
        uint8_t decay_factor;
        unsigned char reserved[8u];
        uint16_t note_freq_hz[12u];                                              // play a maximum of 12 half meldodies (6 bar melody)
} SBGC_cmd_beep_sound_t;
#else
SBGCPACKED(
typedef struct {
        uint16_t mode;
        uint8_t note_length;
        uint8_t decay_factor;
        unsigned char reserved[8u];
        uint16_t note_freq_hz[12u];                                              // play a maximum of 12 half meldodies (6 bar melody)
}) SBGC_cmd_beep_sound_t;
#endif

// CMD_I2C_READ_REG_BUF   same struct as WRITE_REG_BUF
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t device_addr;
        uint8_t reg_addr;
        uint8_t data_len;
} SBGC_i2c_read_reg_buf_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t device_addr;
        uint8_t reg_addr;
        uint8_t data_len;
}) SBGC_i2c_read_reg_buf_t;
#endif

// CMD_BOARD_INFO (readback)
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t BOARD_VER;
        uint16_t FIRMWARE_VER;
        uint8_t STATE_FLAGS1;
        uint16_t BOARD_FEATURES;
        uint8_t CONNECTION_FLAG;
        uint32_t FRW_EXTRA_ID;
        unsigned char RESERVED[7u];
} SBGC_cmd_board_info_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t BOARD_VER;
        uint16_t FIRMWARE_VER;
        uint8_t STATE_FLAGS1;
        uint16_t BOARD_FEATURES;
        uint8_t CONNECTION_FLAG;
        uint32_t FRW_EXTRA_ID;
        unsigned char RESERVED[7u];
}) SBGC_cmd_board_info_t;
#endif

// CMD_BOARD_INFO extended format
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint16_t CFG;
        unsigned char reserved[3u];
} SBGC_cmd_board_info_request_t;
#else
SBGCPACKED(
typedef struct {
        uint16_t CFG;
        unsigned char reserved[3u];
} ) SBGC_cmd_board_info_request_t;
#endif

// CMD_PROFILE_SET (save the configured profile or load the configured profile)
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t slot;
        uint8_t action;
        unsigned char reserved[8u];
} SBGC_cmd_profile_set_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t slot;
        uint8_t action;
        unsigned char reserved[8u];
}) SBGC_cmd_profile_set_t;
#endif

// CMD_READ_PARAMS_3 (read/write system configuration for a single profile part1 or reply from new pid tune )
// CMD_WRITE_PARAMS_3
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t PROFILE_ID;
        uint8_t PROPORTIONAL[3u];
        uint8_t INTEGRAL[3u];
        uint8_t DERIVATIVE[3u];
        uint8_t POWER[3u];
        uint8_t INVERT[3u];
        uint8_t POLES[3u];
        uint8_t ACC_LIMITER_ALL;
        int8_t EXT_FC_GAIN[2u];
        int16_t RC_MIN_ANGLE[3u];
        int16_t RC_MAX_ANGLE[3u];
        uint8_t RC_MODE[3u];
        uint8_t RC_LPF[3u];
        uint8_t RC_SPEED[3u];
        int8_t RC_FOLLOW[3u];
        uint8_t GYRO_TRUST;
        uint8_t USE_MODEL;
        uint8_t PWM_FREQ;
        uint8_t SERIAL_SPEED;
        int8_t RC_TRIM[3u];
        uint8_t RC_DEADBAND;
        uint8_t RC_EXPO_RATE;
        uint8_t RC_VIRT_MODE;
        uint8_t RC_MAP_ROLL;
        uint8_t RC_MAP_PITCH;
        uint8_t RC_MAP_YAW;
        uint8_t RC_MAP_CMD;
        uint8_t RC_MAP_FC_ROLL;
        uint8_t RC_MAP_FC_PITCH;
        uint8_t RC_MIX_FC_ROLL;
        uint8_t RC_MIX_FC_PITCH;
        uint8_t FOLLOW_MODE;
        uint8_t FOLLOW_DEADBAND;
        uint8_t FOLLOW_EXPO_RATE;
        int8_t FOLLOW_OFFSET[3u];
        int8_t AXIS_TOP;
        int8_t AXIS_RIGHT;
        int8_t FRAME_AXIS_TOP;
        int8_t FRAME_AXIS_RIGHT;
        uint8_t FRAME_IMU_POS;
        uint8_t GYRO_DEADBAND;
        uint8_t GYRO_SENS;
        uint8_t I2C_SPEED_FAST;
        uint8_t SKIP_GYRO_CALIB;
        uint8_t RC_CMD_LOW;
        uint8_t RC_CMD_MID;
        uint8_t RC_CMD_HIGH;
        uint8_t MENU_CMD_1;
        uint8_t MENU_CMD_2;
        uint8_t MENU_CMD_3;
        uint8_t MENU_CMD_4;
        uint8_t MENU_CMD_5;
        uint8_t MENU_CMD_LONG;
        uint8_t MOTOR_OUTPUT[3u];
        int16_t BAT_THRESHOLD_ALARM;
        int16_t BAT_THRESHOLD_MOTORS;
        int16_t BAT_COMP_REF;
        uint8_t BEEPER_MODES;
        uint8_t FOLLOW_ROLL_MIX_START;
        uint8_t FOLLOW_ROLL_MIX_RANGE;
        uint8_t BOOSTER_POWER[3u];
        uint8_t FOLLOW_SPEED[3u];
        uint8_t FRAME_ANGLE_FROM_MOTORS;
        int16_t RC_MEMORY[3];
        uint8_t SERVO1_OUT;
        uint8_t SERVO2_OUT;
        uint8_t SERVO3_OUT;
        uint8_t SERVO4_OUT;
        uint8_t SERVO_RATE;
        uint8_t ADAPTIVE_PID_ENABLED;
        uint8_t ADAPTIVE_PID_THRESHOLD;
        uint8_t ADAPTIVE_PID_RATE;
        uint8_t ADAPTIVE_PID_RECOVERY_FACTOR;
        uint8_t FOLLOW_LPF[3u];
        uint16_t GENERAL_FLAGS1;
        uint16_t PROFILE_FLAGS1;
        uint8_t SPEKTRUM_MODE;
        uint8_t ORDER_OF_AXES;
        uint8_t EULER_ORDER;
        uint8_t CUR_IMU;
        uint8_t CUR_PROFILE_ID;
} SBGC_cmd_read_params_3_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t PROFILE_ID;
        uint8_t PROPORTIONAL[3u];
        uint8_t INTEGRAL[3u];
        uint8_t DERIVATIVE[3u];
        uint8_t POWER[3u];
        uint8_t INVERT[3u];
        uint8_t POLES[3u];
        uint8_t ACC_LIMITER_ALL;
        int8_t EXT_FC_GAIN[2u];
        int16_t RC_MIN_ANGLE[3u];
        int16_t RC_MAX_ANGLE[3u];
        uint8_t RC_MODE[3u];
        uint8_t RC_LPF[3u];
        uint8_t RC_SPEED[3u];
        int8_t RC_FOLLOW[3u];
        uint8_t GYRO_TRUST;
        uint8_t USE_MODEL;
        uint8_t PWM_FREQ;
        uint8_t SERIAL_SPEED;
        int8_t RC_TRIM[3u];
        uint8_t RC_DEADBAND;
        uint8_t RC_EXPO_RATE;
        uint8_t RC_VIRT_MODE;
        uint8_t RC_MAP_ROLL;
        uint8_t RC_MAP_PITCH;
        uint8_t RC_MAP_YAW;
        uint8_t RC_MAP_CMD;
        uint8_t RC_MAP_FC_ROLL;
        uint8_t RC_MAP_FC_PITCH;
        uint8_t RC_MIX_FC_ROLL;
        uint8_t RC_MIX_FC_PITCH;
        uint8_t FOLLOW_MODE;
        uint8_t FOLLOW_DEADBAND;
        uint8_t FOLLOW_EXPO_RATE;
        int8_t FOLLOW_OFFSET[3u];
        int8_t AXIS_TOP;
        int8_t AXIS_RIGHT;
        int8_t FRAME_AXIS_TOP;
        int8_t FRAME_AXIS_RIGHT;
        uint8_t FRAME_IMU_POS;
        uint8_t GYRO_DEADBAND;
        uint8_t GYRO_SENS;
        uint8_t I2C_SPEED_FAST;
        uint8_t SKIP_GYRO_CALIB;
        uint8_t RC_CMD_LOW;
        uint8_t RC_CMD_MID;
        uint8_t RC_CMD_HIGH;
        uint8_t MENU_CMD_1;
        uint8_t MENU_CMD_2;
        uint8_t MENU_CMD_3;
        uint8_t MENU_CMD_4;
        uint8_t MENU_CMD_5;
        uint8_t MENU_CMD_LONG;
        uint8_t MOTOR_OUTPUT[3u];
        int16_t BAT_THRESHOLD_ALARM;
        int16_t BAT_THRESHOLD_MOTORS;
        int16_t BAT_COMP_REF;
        uint8_t BEEPER_MODES;
        uint8_t FOLLOW_ROLL_MIX_START;
        uint8_t FOLLOW_ROLL_MIX_RANGE;
        uint8_t BOOSTER_POWER[3u];
        uint8_t FOLLOW_SPEED[3u];
        uint8_t FRAME_ANGLE_FROM_MOTORS;
        int16_t RC_MEMORY[3];
        uint8_t SERVO1_OUT;
        uint8_t SERVO2_OUT;
        uint8_t SERVO3_OUT;
        uint8_t SERVO4_OUT;
        uint8_t SERVO_RATE;
        uint8_t ADAPTIVE_PID_ENABLED;
        uint8_t ADAPTIVE_PID_THRESHOLD;
        uint8_t ADAPTIVE_PID_RATE;
        uint8_t ADAPTIVE_PID_RECOVERY_FACTOR;
        uint8_t FOLLOW_LPF[3u];
        uint16_t GENERAL_FLAGS1;
        uint16_t PROFILE_FLAGS1;
        uint8_t SPEKTRUM_MODE;
        uint8_t ORDER_OF_AXES;
        uint8_t EULER_ORDER;
        uint8_t CUR_IMU;
        uint8_t CUR_PROFILE_ID;
}) SBGC_cmd_read_params_3_t;
#endif

// CMD_READ_PARAMS_EXT (read/write system configuration for a single profile part2)
// CMD_WRITE_PARAMS_EXT
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t PROFILE_ID;
        uint8_t NOTCH_FREQ[3u];
        uint8_t NOTCH_WIDTH[3u];
        int16_t LPF_FREQ[3u];
        uint8_t FILTERS_EN[3u];
        int16_t ENCODER_OFFSET[3u];
        int16_t ENCODER_FLD_OFFSET[3u];
        uint8_t ENCODER_MANUAL_SET_TIME[3u];
        uint8_t MOTOR_HEATING_FACTOR[3u];
        uint8_t MOTOR_COOLING_FACTOR[3u];
        unsigned char RESERVED[2u];
        uint8_t FOLLOW_INSIDE_DEADBAND;
        uint8_t MOTOR_MAG_LINK[3u];
        uint16_t MOTOR_GEARING[3u];
        uint8_t ENCODER_LIMIT_MIN[3u];
        uint8_t ENCODER_LIMIT_MAX[3u];
        int8_t NOTCH1_GAIN[3u];
        int8_t NOTCH2_GAIN[3u];
        int8_t NOTCH3_GAIN[3u];
        uint8_t BEEPER_VOLUME;
        uint16_t ENCODER_GEAR_RATIO[3u];
        uint8_t ENCODER_TYPE[3u];
        uint8_t ENCODER_CFG[3u];
        uint8_t OUTER_P[3u];
        uint8_t OUTER_I[3u];
        int8_t MAG_AXIS_TOP;
        int8_t MAG_AXIS_RIGHT;
        uint8_t MAG_TRUST;
        int8_t MAG_DECLINATION;
        uint16_t ACC_LPF_FREQ;
        uint8_t D_TERM_LPF_FREQ[3u];
} SBGC_cmd_read_params_ext_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t PROFILE_ID;
        uint8_t NOTCH_FREQ[3u];
        uint8_t NOTCH_WIDTH[3u];
        int16_t LPF_FREQ[3u];
        uint8_t FILTERS_EN[3u];
        int16_t ENCODER_OFFSET[3u];
        int16_t ENCODER_FLD_OFFSET[3u];
        uint8_t ENCODER_MANUAL_SET_TIME[3u];
        uint8_t MOTOR_HEATING_FACTOR[3u];
        uint8_t MOTOR_COOLING_FACTOR[3u];
        unsigned char RESERVED[2u];
        uint8_t FOLLOW_INSIDE_DEADBAND;
        uint8_t MOTOR_MAG_LINK[3u];
        uint16_t MOTOR_GEARING[3u];
        uint8_t ENCODER_LIMIT_MIN[3u];
        uint8_t ENCODER_LIMIT_MAX[3u];
        int8_t NOTCH1_GAIN[3u];
        int8_t NOTCH2_GAIN[3u];
        int8_t NOTCH3_GAIN[3u];
        uint8_t BEEPER_VOLUME;
        uint16_t ENCODER_GEAR_RATIO[3u];
        uint8_t ENCODER_TYPE[3u];
        uint8_t ENCODER_CFG[3u];
        uint8_t OUTER_P[3u];
        uint8_t OUTER_I[3u];
        int8_t MAG_AXIS_TOP;
        int8_t MAG_AXIS_RIGHT;
        uint8_t MAG_TRUST;
        int8_t MAG_DECLINATION;
        uint16_t ACC_LPF_FREQ;
        uint8_t D_TERM_LPF_FREQ[3u];
}) SBGC_cmd_read_params_ext_t;
#endif

//CMD_READ_PARAMS_EXT2 – read/write system configuration part 3 (frw.ver. 2.66+)
// same strut used for CMD_WRITE_PARAMS_EXT2
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t PROFILE_ID;
        uint8_t MAV_SRC[2u];
        uint8_t MAV_SYS_ID[2u];
        uint8_t MAV_COMP_ID[2u];
        uint8_t MAV_CFG_FLAGS[2u];
        unsigned char MAV_RESERVED[4u][2u];
        uint16_t MOTOR_MAG_LINK_FINE[3u];
        uint8_t ACC_LIMITER[3u];
        uint8_t PID_GAIN[3u];
        uint8_t FRAME_IMU_LPF_FREQ;
        uint8_t AUTO_PID_CFG;
        uint8_t AUTO_PID_GAIN;
        int16_t FRAME_CAM_ANGLE_MIN[3u];
        int16_t FRAME_CAM_ANGLE_MAX[3u];
        uint16_t GENERAL_FLAGS2;
        uint8_t AUTO_SPEED;
        uint8_t AUTO_ACC_LIMITER;
        int16_t IMU_ORIENTATION_CORR[3u];
        uint16_t TIMELAPSE_TIME;
        uint16_t EMERGENCY_STOP_REST;
        uint8_t ART_DELAY;
        uint8_t TIMELAPSE_ACC_PART;
        int16_t MOMENTUM[3u];
        uint8_t MOMENTUM_CALIB_STIMULUS[3u];
        uint8_t MOMENTUM_ELITPICITY[3u];
        uint8_t FOLLOW_RANGE[3u];
        uint8_t STAB_AXIS[3u];
        int8_t OUTER_MOT_TILT_ANGLE;
        uint8_t STARTUP_ACTION[4u];                                              // params from here only when version > 2.66
        uint8_t STARTUP_ACTION_SRC[2u][4u];
        int8_t STARTUP_ACTION_THRESHOLD[2u][4u];
        uint8_t FORCE_POSITION_CFG[3u];
        uint8_t STEP_SIGNAL_SRC[6u];
        uint8_t STEP_SIGNAL_CFG[6u];
        uint8_t RC_CALIB_SRC[5u];
        uint8_t RC_CALIB_OFFSET[5u];
        uint8_t RC_CALIB_NEG_SCALE[5u];
        uint8_t RC_CALIB_POS_SCALE[5u];
        uint8_t PARKING_POS_CFG;
        uint8_t EXT_LED_PIN_ID;
        uint16_t INTERRUPT_CFG;
        uint8_t OVERLOAD_TIME;
        uint8_t AUTO_PID_MOMENTUM;
        uint8_t JERK_SLOPE[3u];
        uint8_t MAV_CTRL_MODE;
        uint8_t RC_SERIAL_SPEED;
        uint8_t UART2_SPEED;
        uint8_t MOTOR_RES[3u];
        uint16_t CURRENT_LIMIT;
        int8_t MIDDLE_MOT_TILT_ANGLE;
} SBGC_cmd_read_params_ext2_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t PROFILE_ID;
        uint8_t MAV_SRC[2u];
        uint8_t MAV_SYS_ID[2u];
        uint8_t MAV_COMP_ID[2u];
        uint8_t MAV_CFG_FLAGS[2u];
        unsigned char MAV_RESERVED[4u][2u];
        uint16_t MOTOR_MAG_LINK_FINE[3u];
        uint8_t ACC_LIMITER[3u];
        uint8_t PID_GAIN[3u];
        uint8_t FRAME_IMU_LPF_FREQ;
        uint8_t AUTO_PID_CFG;
        uint8_t AUTO_PID_GAIN;
        int16_t FRAME_CAM_ANGLE_MIN[3u];
        int16_t FRAME_CAM_ANGLE_MAX[3u];
        uint16_t GENERAL_FLAGS2;
        uint8_t AUTO_SPEED;
        uint8_t AUTO_ACC_LIMITER;
        int16_t IMU_ORIENTATION_CORR[3u];
        uint16_t TIMELAPSE_TIME;
        uint16_t EMERGENCY_STOP_REST;
        uint8_t ART_DELAY;
        uint8_t TIMELAPSE_ACC_PART;
        int16_t MOMENTUM[3u];
        uint8_t MOMENTUM_CALIB_STIMULUS[3u];
        uint8_t MOMENTUM_ELITPICITY[3u];
        uint8_t FOLLOW_RANGE[3u];
        uint8_t STAB_AXIS[3u];
        int8_t OUTER_MOT_TILT_ANGLE;
        uint8_t STARTUP_ACTION[4u];                                              // params from here only when version > 2.66
        uint8_t STARTUP_ACTION_SRC[2u][4u];
        int8_t STARTUP_ACTION_THRESHOLD[2u][4u];
        uint8_t FORCE_POSITION_CFG[3u];
        uint8_t STEP_SIGNAL_SRC[6u];
        uint8_t STEP_SIGNAL_CFG[6u];
        uint8_t RC_CALIB_SRC[5u];
        uint8_t RC_CALIB_OFFSET[5u];
        uint8_t RC_CALIB_NEG_SCALE[5u];
        uint8_t RC_CALIB_POS_SCALE[5u];
        uint8_t PARKING_POS_CFG;
        uint8_t EXT_LED_PIN_ID;
        uint16_t INTERRUPT_CFG;
        uint8_t OVERLOAD_TIME;
        uint8_t AUTO_PID_MOMENTUM;
        uint8_t JERK_SLOPE[3u];
        uint8_t MAV_CTRL_MODE;
        uint8_t RC_SERIAL_SPEED;
        uint8_t UART2_SPEED;
        uint8_t MOTOR_RES[3u];
        uint16_t CURRENT_LIMIT;
        int8_t MIDDLE_MOT_TILT_ANGLE;
}) SBGC_cmd_read_params_ext2_t;
#endif

//CMD_READ_PARAMS_EXT3 – read/write system configuration part 3 (frw.ver. 2.66+)
// same strut used for CMD_WRITE_PARAMS_EXT3
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t PROFILE_ID;
        unsigned char RESERVED1[21u];
        uint8_t EXT_IMU_TYPE;
        uint8_t EXT_IMU_PORT;
        uint8_t EXT_IMU_POSITION;
        uint8_t EXT_IMU_ORIENTATION;
        uint16_t EXT_IMU_FLAGS;
        unsigned char EXT_IMU_RESERVED[12u];
        uint8_t SOFT_LIMIT_WIDTH[3u];
        uint8_t ADC_REPLACE_SRC[3u];
        uint8_t GLOCK_MID_MOT_POS_CORR_RATE;
        unsigned char RESERVED2[174u];
} SBGC_cmd_read_params_ext3_t;                                                  // only for version > 2.66
#else
SBGCPACKED(
typedef struct {
        uint8_t PROFILE_ID;
        unsigned char RESERVED1[21u];
        uint8_t EXT_IMU_TYPE;
        uint8_t EXT_IMU_PORT;
        uint8_t EXT_IMU_POSITION;
        uint8_t EXT_IMU_ORIENTATION;
        uint16_t EXT_IMU_FLAGS;
        unsigned char EXT_IMU_RESERVED[12u];
        uint8_t SOFT_LIMIT_WIDTH[3u];
        uint8_t ADC_REPLACE_SRC[3u];
        uint8_t GLOCK_MID_MOT_POS_CORR_RATE;
        unsigned char RESERVED2[174u];
}) SBGC_cmd_read_params_ext3_t;                                                 // only for version > 2.66
#endif

//CMD_READ_PARAMS, CMD_READ_PARAMS_3  – request parameters from the board
//CMD_READ_PARAMS_EXT – request extended parameters part1
//CMD_READ_PARAMS_EXT2 – request extended parameters part2
//CMD_READ_PARAMS_EXT3 – request extended parameters part3 (frw.ver. 2.66+)
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t PROFILE_ID;
} SBGC_cmd_read_params_request_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t PROFILE_ID;
}) SBGC_cmd_read_params_request_t;
#endif
//CMD_BOOT_MODE_3
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t CONFIRM;
        uint8_t DELAY_MS;
} SBGC_cmd_boot_mode_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t CONFIRM;
        uint8_t DELAY_MS;
}) SBGC_cmd_boot_mode_t;
#endif

//
//  CMD_I2C_WRITE_REG_BUF Write info to device connected on i2c on STM Mikro Gimbal
//  1 for internal port EEPROM
//
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t DEVICE_ADDR_PORT : 1u;                                          // port 1 = internal EEPROM 0 = external e.g IMU sensor
        uint8_t DEVICE_ADDR : 7u;                                               // device address
        uint8_t REG_ADDR;                                                       // register address for data
        uint8_t DATA[10u];                                                      // 10 byte long data string to write starting at reg address
} SBGC_cmd_i2c_write_reg_buf_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t DEVICE_ADDR_PORT : 1u;                                          // port 1 = internal EEPROM 0 = external e.g IMU sensor
        uint8_t DEVICE_ADDR : 7u;                                               // device address
        uint8_t REG_ADDR;                                                       // register address for data
        uint8_t DATA[10u];                                                      // 10 byte long data string to write starting at reg address
}) SBGC_cmd_i2c_write_reg_buf_t;
#endif

//
// CMD_I2C_READ_REG_BUF Read info from the i2c line
//
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t DEVICE_ADDR_PORT : 1u;                                          // port 1 = internal EEPROM 0 = external e.g IMU sensor
        uint8_t DEVICE_ADDR : 7u;                                               // device address
        uint8_t REG_ADDR;                                                       // register address for data
        uint8_t DATA_LEN;                                                       // number of bytes to read starting at reg address
} SBGC_cmd_i2c_read_reg_buf_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t DEVICE_ADDR_PORT : 1u;                                          // port 1 = internal EEPROM 0 = external e.g IMU sensor
        uint8_t DEVICE_ADDR : 7u;                                               // device address
        uint8_t REG_ADDR;                                                       // register address for data
        uint8_t DATA_LEN;                                                       // number of bytes to read starting at reg address
}) SBGC_cmd_i2c_read_reg_buf_t;
#endif

// CMD_WRITE_EXTERNAL_DATA
//
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t DATA[128u];                                                     // data string to write to EEPROM
} SBGC_cmd_write_external_data_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t DATA[128u];                                                     // data string to write to EEPROM
}) SBGC_cmd_write_external_data_t;
#endif

//
// CMD_READ_FILE
//
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint16_t FILE_ID;                                                       // File type
        uint16_t PAGE_OFFSET;
        uint16_t MAX_SIZE;
        unsigned char RESERVED[14];
} SBGC_cmd_read_file_t;
#else
SBGCPACKED(
typedef struct {
        uint16_t FILE_ID;                                                       // File type
        uint16_t PAGE_OFFSET;
        uint16_t MAX_SIZE;
        unsigned char RESERVED[14];
}) SBGC_cmd_read_file_t;
#endif
//
// CMD_WRITE_FILE
//
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint16_t FILE_ID;                                                       // File type see file read command e.g. SBGC_FILE_TYPE_SCRIPT
        uint16_t FILE_SIZE;                                                     // Full size of a file
        uint16_t PAGE_OFFSET;                                                   // offset from the beginning, in pages. 1 page = 64 bytes.
        unsigned char DATA[250u];                                               // 1 page of data should be less than FILE_SIZE (size of your script)
} SBGC_cmd_write_file_t;
#else
SBGCPACKED(
typedef struct {
        uint16_t FILE_ID;                                                       // File type see file read command e.g. SBGC_FILE_TYPE_SCRIPT
        uint16_t FILE_SIZE;                                                     // Full size of a file
        uint16_t PAGE_OFFSET;                                                   // offset from the beginning, in pages. 1 page = 64 bytes.
        unsigned char DATA[250u];                                               // 1 page of data should be less than FILE_SIZE (size of your script)
}) SBGC_cmd_write_file_t;
#endif
//
// CMD_RUN_SCRIPT
//
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t MODE;                                                           //  0 – stop 1 – start 2 – start with debug information is sent back in the CMD_SCRIPT_DEBUG
        uint8_t SLOT;
        unsigned char RESERVED[32u];                                            // 1 page of data should be less than FILE_SIZE
} SBGC_cmd_run_script_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t MODE;                                                           //  0 – stop 1 – start 2 – start with debug information is sent back in the CMD_SCRIPT_DEBUG
        uint8_t SLOT;
        unsigned char RESERVED[32u];                                            // 1 page of data should be less than FILE_SIZE
}) SBGC_cmd_run_script_t;
#endif
//
// CMD_RUN_SCRIPT
//
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint16_t CMD_COUNT;                                                      //  0 – stop 1 – start 2 – start with debug information is sent back in the CMD_SCRIPT_DEBUG
        uint8_t ERR_CODE;
} SBGC_cmd_script_debug_t;
#else
SBGCPACKED(
typedef struct {
        uint16_t CMD_COUNT;                                                      //  0 – stop 1 – start 2 – start with debug information is sent back in the CMD_SCRIPT_DEBUG
        uint8_t ERR_CODE;
}) SBGC_cmd_script_debug_t;
#endif
//CMD_AHRS_HELPER – send or request attitude of the IMU sensor. use an external accelometer which is more accurate
//SBGCPACKED(
//typedef struct {
//        uint8_t MODE;
//       float32_t Z_VECT[3];
//       float32_t H_VECT[3]
//}) SBGC_cmd_ahrs_helper_t;

//AHRS_DEBUG_INFO part of other commands e.g. Real_Time_Data_Custom or EXT_IMU_DEBUG_INFO 26bytes sent in these messages on fw > 2.60
#if defined(D_FT900)
typedef struct SBGCPACKED {
        uint8_t MAIN_IMU_REF_SRC;
        uint8_t FRAME_IMU_REF_SRC;
        uint8_t MAIN_IMU_Z_REF_ERR;
        uint8_t MAIN_IMU_H_REF_ERR;
        uint8_t FRAME_IMU_Z_REF_ERR;
        uint8_t FRAME_IMU_H_REF_ERR;
        uint8_t EXT_IMU_STATUS;
        uint16_t EXT_IMU_PACKETS_RECEIVED_CNT;
        uint16_t EXT_IMU_PARSE_ERR_CNT;
        uint8_t EXT_CORR_H_ERR;
        uint8_t EXT_CORR_Z_ERR;
        uint8_t RESERVED[13u];
} SBGC_ahrs_debug_info_t;
#else
SBGCPACKED(
typedef struct {
        uint8_t MAIN_IMU_REF_SRC;
        uint8_t FRAME_IMU_REF_SRC;
        uint8_t MAIN_IMU_Z_REF_ERR;
        uint8_t MAIN_IMU_H_REF_ERR;
        uint8_t FRAME_IMU_Z_REF_ERR;
        uint8_t FRAME_IMU_H_REF_ERR;
        uint8_t EXT_IMU_STATUS;
        uint16_t EXT_IMU_PACKETS_RECEIVED_CNT;
        uint16_t EXT_IMU_PARSE_ERR_CNT;
        uint8_t EXT_CORR_H_ERR;
        uint8_t EXT_CORR_Z_ERR;
        uint8_t RESERVED[13u];
}) SBGC_ahrs_debug_info_t;
#endif

#ifdef __cplusplus
}
#endif

#endif