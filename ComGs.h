#ifndef __comgs_h
#define __comgs_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//    ComGS.h : Commander Ground Station (various HMI and general functionality)
//              various general and common functions for GPS Robot and PID controls
//              various position and trajectory generation and control algorythms 
//
//    Version : @(#) 1.2
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
// Defines the structures used between ComGs (data display) and Gimbal / Camera
// Also provides a library of functions which do calculations and conversions manipulation on this data
//
//
#include "definitions.h"                                                        /* global defines are here */
#include "gc_events.h"
/*#include "mmc_file_handler.h"*/
#include "lwNx_defs.h"
#include "odrive.h"
#include "Struts.h"

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define COMGSPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define COMGSPACKED __attribute__((packed)) ALIGNED(1)                        /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define COMGSPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define COMGSPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define COMGSPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

extern struct COMGS_cmd_get_angles_t commangles;
extern struct XS_Hmi_Slider_t hmiTimeCalcTotal;                                 // Calculated from the Time Entered on the HMI
extern struct XS_Hmi_Slider_t hmiDateCalcTotal;                                 // Calculated from the Date Entered on the HMI
extern struct XS_Hmi_Slider_t hmiRawContrast[4u];                               // encoder Contrast request state (per channel)
extern struct XS_Hmi_Slider_t hmiRawHue[4u];                                    // encoder Hue request state (per channel)
extern struct XS_Hmi_Slider_t hmiRawSaturation[4u];                             // encoder Saturation request state (per channel)
extern struct XS_Hmi_Slider_t hmiRawClipsz[4u];                                 // encoder Clip size request state (per channel)
extern struct XS_Hmi_Slider_t hmiRawCliplen[4u];                                // encoder Clip length request state (per channel)
extern struct XS_Hmi_Slider_t hmiRawBright[4u];                                 // encoder Brightness request state (per channel)
extern struct XS_Hmi_Slider_t hmiRawFramert[4u];                                // encoder Frame Rate request state (per channel)
extern struct XS_Hmi_Slider_t hmiRawFramescl[4u];                               // encoder Frame Scale request state (per channel)
extern struct XS_Hmi_Slider_t hmiRawFrameint[4u];                               // encoder Frame Interval request state (per channel)
extern struct XS_Hmi_Slider_t hmiRawBitrate[4u];                                // encoder Bit rate request state

// #define USE_CAL_UNLOCK                                                       uncomment to : Unlock calibration and allow other parallel action during calibration of joystick
#define MAX_ZERO_ACCS 25u                                                       // zero velocity if we exceed this many accelerometer readings at zero

// defines for AHRS quarternion functions
#define deltat 0.001f                                                           // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f)                       // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError                                  // compute beta

#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f)                        // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError                                  // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift                                  // compute zeta

#define white_space(c) ((c) == ' ' || (c) == '\t')                              // Check for whitespace
#define valid_digit2(c) ((c) >= '0' && (c) <= '9')                              // Check for valid digits

// COMGS_quarternion quartCoOrd;                                                // quarternion co-ordinates
// for calibrate
#include "io.h"

#include "pid.h"                                                                // This is for pid blocks globals
#include "robot.h"                                                              // This is for robot helper functions
#include "MahonyAHRS.h"                                                         // includes for Mahony
#include <math.h>
#include "AlarmDefinitions.h"                                                   // for alarms
#include "Madgwick.h"                                                           // includes for madgwick

//---------------------------------------------------------------------------------------------------
// Definitions for Mahony

// #define sampleFreq 512.0f                                                    // sample frequency in Hz (in our case we calculate this from the iteration)
#define twoKpDef (2.0f * 0.4f)                                                  // 2 * proportional gain
#define twoKiDef (2.0f * 0.0001f)                                               // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Definitions for Madgewick

// #define sampleFreq 512.0f                                                       // sample frequency in Hz (in our case we calculate this from the iteration)
#define betaDef 0.01f                                                           // proportional gain

//---------------------------------------------------------------------------------------------------
// variable declarations for Mahony algorythm (global volatiles)

volatile float32_t twoKp = twoKpDef;                                            // 2 * proportional gain (Kp)
volatile float32_t twoKi = twoKiDef;                                            // 2 * integral gain (Ki)
volatile float32_t q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                  // quaternion of sensor frame relative to auxiliary frame
volatile float32_t integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
//---------------------------------------------------------------------------------------------------
// variable declarations for Madgewick & Mahony algorythm (global volatiles)

// The acc in Z for static position (g)
// Set on first update, assuming we are in a static position since the sensors were just calibrates.
// This value will be better the more level the copter is at calibration time
volatile float32_t baseZacc = 1.0f;
volatile float32_t gravX = 0.0f, gravY = 0.0f, gravZ = 0.0f;                    // Unit vector in the estimated gravity direction
//---------------------------------------------------------------------------------------------------
// Variable definitions for Madgwick (global)

volatile float32_t betaMadg = betaDef;                                                // 2 * proportional gain (Kp)
// volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                        ignore same as mahony quaternion of sensor frame relative to auxiliary frame

// The data from a Get_Angles readback SBGC message.
#if defined(D_FT900)
typedef struct COMGSPACKED {
        float32_t imu_angle_roll;
        float32_t target_angle_roll;
        float32_t target_speed_roll;
        float32_t imu_angle_pitch;
        float32_t target_angle_pitch;
        float32_t target_speed_pitch;
        float32_t imu_angle_yaw;
        float32_t target_angle_yaw;
        float32_t target_speed_yaw;
        float32_t stator_rotor_ang_yaw;
        float32_t stator_rotor_ang_roll;
        float32_t stator_rotor_ang_pitch;
} COMGS_cmd_get_angles_t;
#else
COMGSPACKED(
typedef struct {
        float32_t imu_angle_roll;
        float32_t target_angle_roll;
        float32_t target_speed_roll;
        float32_t imu_angle_pitch;
        float32_t target_angle_pitch;
        float32_t target_speed_pitch;
        float32_t imu_angle_yaw;
        float32_t target_angle_yaw;
        float32_t target_speed_yaw;
        float32_t stator_rotor_ang_yaw;
        float32_t stator_rotor_ang_roll;
        float32_t stator_rotor_ang_pitch;
}) COMGS_cmd_get_angles_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
        int32_t gyro_roll;                                                      // raw data from gyro
        int32_t acc_roll;                                                       // raw data from accelerometer
        int32_t gyro_pitch;
        int32_t acc_pitch;
        int32_t gyro_yaw;
        int32_t acc_yaw;
        int32_t mavg_acc_roll;                                                  // moving averages
        int32_t mavg_acc_pitch;
        int32_t mavg_acc_yaw;
        int32_t mavg_acc_rolls[2u];                                             // last known averages first and second
        int32_t mavg_acc_pitchs[2u];                                            // last known averages first and second
        int32_t mavg_acc_yaws[2u];                                              // last known averages first and second
        uint8_t g_count_sam;                                                    // number of samples taken
        uint8_t g_count_acc;                                                    // first or second sample chosen
        uint8_t zero_acc_cnt_roll;                                              // count the number of zero accelerations
        uint8_t zero_acc_cnt_pitch;                                             // count the number of zero accelerations
        uint8_t zero_acc_cnt_yaw;                                               // count the number of zero accelerations
        int32_t vel_roll[2u];                                                   // calculated velocity
        int32_t pos_roll[2u];                                                   // calculated position
        int32_t vel_pitch[2u];                                                  // calculated velocity
        int32_t pos_pitch[2u];                                                  // calculated position
        int32_t vel_yaw[2u];                                                    // calculated velocity
        int32_t pos_yaw[2u];                                                    // calculated position
        float32_t theta_yaw;
        float32_t theta_roll;
        float32_t theta_pitch;
} COMGS_gyro_acc_data_t;
#else
COMGSPACKED(
typedef struct {
        int32_t gyro_roll;                                                      // raw data from gyro
        int32_t acc_roll;                                                       // raw data from accelerometer
        int32_t gyro_pitch;
        int32_t acc_pitch;
        int32_t gyro_yaw;
        int32_t acc_yaw;
        int32_t mavg_acc_roll;                                                  // moving averages
        int32_t mavg_acc_pitch;
        int32_t mavg_acc_yaw;
        int32_t mavg_acc_rolls[2u];                                             // last known averages first and second
        int32_t mavg_acc_pitchs[2u];                                            // last known averages first and second
        int32_t mavg_acc_yaws[2u];                                              // last known averages first and second
        uint8_t g_count_sam;                                                    // number of samples taken
        uint8_t g_count_acc;                                                    // first or second sample chosen
        uint8_t zero_acc_cnt_roll;                                              // count the number of zero accelerations
        uint8_t zero_acc_cnt_pitch;                                             // count the number of zero accelerations
        uint8_t zero_acc_cnt_yaw;                                               // count the number of zero accelerations
        int32_t vel_roll[2u];                                                   // calculated velocity
        int32_t pos_roll[2u];                                                   // calculated position
        int32_t vel_pitch[2u];                                                  // calculated velocity
        int32_t pos_pitch[2u];                                                  // calculated position
        int32_t vel_yaw[2u];                                                    // calculated velocity
        int32_t pos_yaw[2u];                                                    // calculated position
        float32_t theta_yaw;
        float32_t theta_roll;
        float32_t theta_pitch;
}) COMGS_gyro_acc_data_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
        float32_t IMU_ANGLE[3u];
        float32_t FRAME_IMU_ANGLE[3u];
        float32_t TARGET_ANGLE[3u];
        float32_t BAT_LEVEL;
        uint8_t MOTORS_STATE;
        uint8_t CUR_IMU;
        uint8_t CUR_PROFILE;
} COMGS_real_data_3_t;
#else
COMGSPACKED(
typedef struct {
        float32_t IMU_ANGLE[3u];
        float32_t FRAME_IMU_ANGLE[3u];
        float32_t TARGET_ANGLE[3u];
        float32_t BAT_LEVEL;
        uint8_t MOTORS_STATE;
        uint8_t CUR_IMU;
        uint8_t CUR_PROFILE;
}) COMGS_real_data_3_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      float32_t imu_temp;
      float32_t frame_imu_temp;
} COMGS_temp_t;
#else
COMGSPACKED(
typedef struct {
      float32_t imu_temp;
      float32_t frame_imu_temp;
}) COMGS_temp_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      float32_t mag_data_roll;
      float32_t mag_data_pitch;
      float32_t mag_data_yaw;
} COMGS_magnet_t;
#else
COMGSPACKED(
typedef struct {
      float32_t mag_data_roll;
      float32_t mag_data_pitch;
      float32_t mag_data_yaw;
}) COMGS_magnet_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      float32_t motor_power_roll;
      float32_t motor_power_pitch;
      float32_t motor_power_yaw;
      float32_t CURRENT;
} COMGS_motor_power_t;
#else
COMGSPACKED(
typedef struct {
      float32_t motor_power_roll;
      float32_t motor_power_pitch;
      float32_t motor_power_yaw;
      float32_t CURRENT;
}) COMGS_motor_power_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      float32_t v1;                                                             // Configurable data got with a CMD_GET_ADJ_VARS_VAL command
      float32_t v2;
      float32_t v3;
      float32_t v4;
      float32_t v5;
      float32_t v6;
      float32_t v7;
} COMGS_get_vars;
#else
COMGSPACKED(
typedef struct {
      float32_t v1;                                                             // Configurable data got with a CMD_GET_ADJ_VARS_VAL command
      float32_t v2;
      float32_t v3;
      float32_t v4;
      float32_t v5;
      float32_t v6;
      float32_t v7;
}) COMGS_get_vars;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      uint8_t AlarmWdCurrent;                                                   // Current Data associated with an ERROR Alarm
      uint8_t AlarmWdAck;                                                       // Acknowledgement state of the Alarm Word
      uint8_t AlarmWdState;                                                     // Alarm Status word to ComGs
      uint8_t AlarmWdAcked;                                                     // Re-trig for Ack (Ack button removed after timer T) DON (T) of ACK
      uint8_t AlarmWdAckLastScan;                                               // Last Scan of Alarm ACK
      uint16_t AlarmTmRun : 1u;                                                 // Turn the timer on/off
      uint16_t AlarmTm : 15u;                                                   // Timer Tick for DON
} COMGS_error_alarm;
#else
COMGSPACKED(
typedef struct {
      uint8_t AlarmWdCurrent;                                                   // Current Data associated with an ERROR Alarm
      uint8_t AlarmWdAck;                                                       // Acknowledgement state of the Alarm Word
      uint8_t AlarmWdState;                                                     // Alarm Status word to ComGs
      uint8_t AlarmWdAcked;                                                     // Re-trig for Ack (Ack button removed after timer T) DON (T) of ACK
      uint8_t AlarmWdAckLastScan;                                               // Last Scan of Alarm ACK
      uint16_t AlarmTmRun : 1u;                                                 // Turn the timer on/off
      uint16_t AlarmTm : 15u;                                                   // Timer Tick for DON
}) COMGS_error_alarm;
#endif

// -------- HMI Command numbers ----------
//
#define COMGS_OFF_NORMAL 4U                                                      // The possible states of the command  for motor from the commGS
#define COMGS_ON 1U
#define COMGS_OFF_SAFE 3U
#define COMGS_OFF_BREAK 2U
#define SEND_MOTOR_OFF 5U
#define SEND_MOTOR_ON 6U

#define COMGS_PID_STOP 0U                                                        // The possible states of the command for the PID tune frm commGS
#define COMGS_PID_ROLL 1U
#define COMGS_PID_PITCH 2U
#define COMGS_PID_YAW 4U
#define COMGS_PID_GUI 8U
#define COMGS_PID_KEEP 16U
#define COMGS_PID_LPF 32U
#define COMGS_PID2_ACTION_START 40U
#define COMGS_PID2_ACTION_START_SAVE 41U
#define COMGS_PID2_ACTION_SAVE 42U
#define COMGS_PID2_ACTION_STOP 43U
#define COMGS_PID2_ACTION_READ 44U

#define COMGS_PROFILE3 1U                                                       // define the request values for the menu_execute remote word from Com GS GUI
#define COMGS_CALIB_ACC 2U
#define COMGS_CALIB_GYRO 3U
#define COMGS_MOTOR_TOGGLE 4U
#define COMGS_HOME_POS 5U
#define COMGS_CAM_EV 6U
#define COMGS_PH_EV 7U
#define COMGS_CALIB_MAG 8U

#define COMGS_PROFILE_SET_ACTION_SAVE 1U                                        // define the request values for the profile_set remote word from Com GS GUI
#define COMGS_PROFILE_SET_ACTION_CLEAR 2U
#define COMGS_PROFILE_SET_ACTION_LOAD 3U

#define COMGS_EXPO_RATE_MODE_RC 1U                                              // define the request values for the configure control remote word from Com GS GUI
#define COMGS_ANGLE_LPF 2U
#define COMGS_SPEED_LPF 3U
#define COMGS_RC_LPF 4U

// Command Adj_var_val
#define COMGS_SPEED100 1U                                                       // Set Roll Pitch and Yaw speeds to 100%
#define COMGS_FOLLOW60 2U                                                       // Set Roll Pitch and Yaw follow speed to 60%
#define COMGS_PIDGAIN 3U                                                        // Set all PID Gain to value
#define MAV_SET_SPEED 4U                                                        // mavlink command to set the speed of all 3 axis
#define MAV_SET_ANGLE 5U                                                        // mavlink command to use quartenion to set axis angle

                                                                                // Command IMU
#define COMGS_IMU_MAIN 1U                                                       // Request Main IMU
#define COMGS_GET7_1 1U                                                         // get the 1st set of 7 values
#define COMGS_IMU_FRAME 2U                                                      // Request Frame IMU

#define COMGS_ALARM_ACK_TIME 200U                                               // Time for alarm ACK to be active

#define CMD_NOCMD 0U                                                            // No Command Active (test initial state)
#define CMD_SEND 1U                                                             // States of each of the SBGC send requests from ComGS.
#define CMD_SENT 2U                                                             // We sent a UDP frame with the simpleBGC request
#define CMD_CONFIRMED 3U                                                        // We got a confirm for the message sent from the remote device
#define CMD_COMPLETE 4U                                                         // We completed the message
#define CMD_RESEND 5U                                                           // We re-triggered the send due to timeout on recieving the confirm message back
#define CMD_SENT_2 6U                                                           // poll period is being set after event registration

#define COMGS_RUN_NEW_SCRIPT 1U                                                 // Request from COM GS to Write to EEPROM slot 0 a new script and run it
#define COMGS_RUN_A_SCRIPT 2U                                                   // Request from COM GS to run new script @ slot 0
#define COMGS_STOP_SCRIPT 3U                                                    // Request from COM GS to stop new script @ slot 0

// State engine for scripting on gimbal host
#define CMD_SSEND 1U                                                            // send a script
#define CMD_WSEND 2U                                                            // write to eeprom
#define CMD_SSENT 3U                                                            // sent a script start wait for confirm
#define CMD_WSENT 4U                                                            // written to eeprom wait for confirm
#define CMD_WRITTEN 5U                                                          // eeprom write confirmation
#define CMD_WCONFIRMED 6U                                                       // script start confirmed
#define CMD_WCOMPLETE 7U                                                        // command completed
#define CMD_SSTOP 8U                                                            // send a stop to the gimbal to end script execution

// valid values for the event logger set-up
#define COMGS_CMD_REALTIME_DATA_3 1U
#define COMGS_CMD_REALTIME_DATA_4 2U
#define COMGS_CMD_REALTIME_DATA_CUSTOM 3U
#define COMGS_CMD_AHRS_HELPER 4U
#define COMGS_CMD_EVENT 5U

#define COMGS_EVENT_ID_MENU_BUTTON_OFF 1U
#define COMGS_EVENT_ID_MENU_BUTTON_ON 2U
#define COMGS_EVENT_ID_MENU_BUTTON_HOLD 3U
#define COMGS_EVENT_ID_MOTOR_STATE_OFF 4U
#define COMGS_EVENT_ID_MOTOR_STATE_ON 5U
#define COMGS_EVENT_ID_EMERGENCY_STOP_OFF 6U
#define COMGS_EVENT_ID_EMERGENCY_STOP_ON 7U
#define COMGS_EVENT_ID_CAMERA_REC_PHO 8U
#define COMGS_EVENT_ID_CAMERA_PHO 9U
#define COMGS_EVENT_ID_SCRIPT_OFF 10U
#define COMGS_EVENT_ID_SCRIPT_ON 11U


static const uint16_t Udays[4u][12u] =                                          // Used for calculating time stamp since 1970

{

    {   0U,  31U,     60U,     91U,     121U,    152U,    182U,    213U,    244U,    274U,    305U,    335U},

    { 366U,  397U,    425U,    456U,    486U,    517U,    547U,    578U,    609U,    639U,    670U,    700U},

    { 731U,  762U,    790U,    821U,    851U,    882U,    912U,    943U,    974U,    1004U,   1035U,   1065U},

    {1096U,  1127U,   1155U,   1186U,   1216U,   1247U,   1277U,   1308U,   1339U,   1369U,   1400U,   1430U},

};

/* ------ PMLPS Kaz Kojima ---------- */
// Default values.

// PMLPS server udp port
#define LPS_PORT 5770u

// APM telemetry tcp
#define TLM_ADDR "192.168.11.1"
#define TLM_PORT 5900u

// CAM x-axis from north
#define CAM_DIRECTION 2.6f

// CAM height in cm
#define CAM_HEIGHT 220.0f

// CAM image resolution type
#define CAM_IMAGE_WIDTH 320.0f
#define CAM_IMAGE_HEIGHT 240.0f

// Lens ratio
#define CAM_LENS_RATIO 0.003f

// Marker type
#define MARKER_TYPE_I 1u
#define MARKER_TYPE_I3 2u

// Square size of marker surround
#define SQ_SIZEOF_SURROUND 91.0f
// Square ratio of head_to_mid/head_to_tail
#define SQ_RATIO_I3 (0.25f*0.25f)

#define PMLPS_YAW_INITIALIZE_COUNT 40u

static const float32_t frame_epsilon = 30.0f;
static const float32_t position_sq_epsilon = 120.0f;
float32_t PMLPS_Q = 0.01f;
float32_t PMLPS_R = 0.0001f;
float32_t PMLPS_H = 1.0f;
float32_t PMLPS_B = 0.025f;

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float32_t _ix;
  float32_t _iy;
  float32_t _iw;
  float32_t _ih;
  float32_t _xhat;
  float32_t _xhat_prev;
  float32_t _p;
  float32_t _u;
  int16_t _ct;
} ImageSensorBlob_t;
#else
COMGSPACKED(
typedef struct {
  float32_t _ix;
  float32_t _iy;
  float32_t _iw;
  float32_t _ih;
  float32_t _xhat;
  float32_t _xhat_prev;
  float32_t _p;
  float32_t _u;
  int16_t _ct;
}) ImageSensorBlob_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  int16_t lps_port;
  char *tlm_addr;
  int16_t tlm_port;
  uint16_t cam_image_width;
  uint16_t cam_image_height;
  uint16_t cam_lens_ratio;
  float32_t cam_direction;
  float32_t cam_height;
  uint16_t marker_type;
  float32_t marker_sqsize;
  float32_t marker_sqratio;
  uint8_t cam_lens_fisheye : 1u;
  uint8_t use_position_delta : 1u;
  uint8_t _initialized : 1u;
  uint8_t use_prefilter : 1u;
  uint8_t update_attitude : 1u;
  uint8_t suc : 1u;
  uint8_t prev_yaw_angle_init : 1u;
  uint8_t prev : 1u;
  float32_t prev_yaw_angle;
  float32_t yaw_angle;
  float32_t pitch_angle;
} camera_config_t;
#else
COMGSPACKED(
typedef struct {
  int16_t lps_port;
  char *tlm_addr;
  int16_t tlm_port;
  uint16_t cam_image_width;
  uint16_t cam_image_height;
  uint16_t cam_lens_ratio;
  float32_t cam_direction;
  float32_t cam_height;
  uint16_t marker_type;
  float32_t marker_sqsize;
  float32_t marker_sqratio;
  uint8_t cam_lens_fisheye : 1u;
  uint8_t use_position_delta : 1u;
  uint8_t _initialized : 1u;
  uint8_t use_prefilter : 1u;
  uint8_t update_attitude : 1u;
  uint8_t suc : 1u;
  uint8_t prev_yaw_angle_init : 1u;
  uint8_t prev : 1u;
  float32_t prev_yaw_angle;
  float32_t yaw_angle;
  float32_t pitch_angle;
}) camera_config_t;
#endif

#if defined(NAVILOCK_USED)


#if defined(D_FT900)
typedef struct COMGSPACKED {
   uint8_t hour;
   uint8_t min;
   uint8_t sec;
} nvETime_t;
#else
  COMGSPACKED (
typedef struct {
   uint8_t hour;
   uint8_t min;
   uint8_t sec;
}) nvETime_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
   uint8_t day;
   uint8_t month;
   uint16_t year;
} nvEDate_t;
#else
  COMGSPACKED (
typedef struct {
   uint8_t day;
   uint8_t month;
   uint16_t year;
}) nvEDate_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint8_t points[];
  float64_t longditude;
  float64_t lattitude;
  uint8_t type;
  float64_t speed;
  nvETime_t time;
  float64_t altitude;
} nvEPoint_t;
#else
  COMGSPACKED (
typedef struct {
  uint8_t points[];
  float64_t longditude;
  float64_t lattitude;
  uint8_t type;
  float64_t speed;
  nvETime_t time;
  float64_t altitude;
}) nvEPoint_t;
#endif

        
#if defined(D_FT900)
typedef struct COMGSPACKED {
        nvEPoint_t* points;
        uint64_t point_count;                                                   //all points, including POI count u32 + byte so its a 64 bit int
        uint64_t points_done;
        uint8_t poi_count;                                                      //points of interest
        uint32_t start_addr;
        
        nvEDate_t start_date;
        nvETime_t start_time;
        
        nvEDate_t end_date;
        nvETime_t end_time;
        
        float64_t tot_distance;
        uint8_t max_speed;
        float32_t getMaxSpeed;                                                  //[km/h]
        int16_t min_altitude;
        int16_t max_altitude;
        int16_t elevation;                                                      //[m] in total
        int16_t descent;                                                        //[m] in total
        int16_t time_zero_speed;                                                //[s] total time with speed==0 -> end time - start time - time_zero_speed = driving speed
        int16_t tripDuration;  
} navi_track_t;
#else
  COMGSPACKED (
typedef struct {
        nvEPoint_t* points;
        uint64_t point_count; //all points, including POI count
        uint64_t points_done;
        uint8_t poi_count; //points of interest
        uint32_t start_addr;
        
        nvEDate_t start_date;
        nvETime_t start_time;
        
        nvEDate_t end_date;
        nvETime_t end_time;
        
        float64_t tot_distance;
        uint8_t max_speed;
        float32_t getMaxSpeed; //[km/h]
        int16_t min_altitude;
        int16_t max_altitude;
        int16_t elevation; //[m] in total
        int16_t descent; //[m] in total
        int16_t time_zero_speed; //[s] total time with speed==0 -> end time - start time - time_zero_speed = driving speed
        int16_t tripDuration; 
}) navi_track_t;
#endif

#endif /* navilock */

#define ALGO_IS_MADGEWICK 0u                                                    // defines the algorythm
#define ALGO_IS_MAHONY 1u

#if defined(D_FT900)
typedef struct COMGSPACKED {
      uint8_t past_end : 1u;        
      uint8_t spare : 7u;                                                 
      float64_t distance;
      float64_t bearing;
} crosstrack_error_t;
#else
COMGSPACKED(
typedef struct {
      uint8_t past_end : 1u;        
      uint8_t spare : 7u;                                                 
      float64_t distance;
      float64_t bearing;
}) crosstrack_error_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      uint8_t mode_began : 1u;                                                  // initialise integrator first time get the 100ms tick and store in lastitme
      uint8_t isCalib : 1u;                                                     // is calibration completed
      uint8_t algoType : 1u;                                                    // algorythm Type for filter madgewick or mahony as defined above
      uint8_t isEstComp : 1u;                                                   // initialise of complementary estimator
      uint8_t spare1 : 4u;

      float32_t SEq_1;                                                          // estimated orientation quaternion elements
      float32_t SEq_2;
      float32_t SEq_3;
      float32_t SEq_4;
      float32_t b_x;                                                            // reference direction of flux in earth frame
      float32_t b_z;
      float32_t w_bx;                                                           // estimate gyroscope biases error
      float32_t w_by;
      float32_t w_bz;
      float32_t hx;
      float32_t hy;
      uint64_t lasttime;                                                        // last known 100ms tick counter value from the interrupt
} COMGS_quarternion;
#else
COMGSPACKED(
typedef struct {
      uint8_t mode_began : 1u;                                                  // initialise integrator first time get the 100ms tick and store in lastitme
      uint8_t isCalib : 1u;                                                     // is calibration completed
      uint8_t algoType : 1u;                                                    // algorythm Type for filter madgewick or mahony as defined above
      uint8_t isEstComp : 1u;                                                   // initialise of complementary estimator
      uint8_t spare1 : 4u;

      float32_t SEq_1;                                                          // estimated orientation quaternion elements
      float32_t SEq_2;
      float32_t SEq_3;
      float32_t SEq_4;
      float32_t b_x;                                                            // reference direction of flux in earth frame
      float32_t b_z;
      float32_t w_bx;                                                           // estimate gyroscope biases error
      float32_t w_by;
      float32_t w_bz;
      float32_t hx;
      float32_t hy;
      uint64_t lasttime;                                                        // last known 100ms tick counter value from the interrupt
}) COMGS_quarternion;
#endif


#if defined(D_FT900)
typedef struct COMGSPACKED {
      float32_t vx;
      float32_t vy;
      float32_t vz;
      float32_t wx;
      float32_t wy;
      float32_t wz;
} COMGS_magcorr_t;
#else
COMGSPACKED(
typedef struct {
      float32_t vx;
      float32_t vy;
      float32_t vz;
      float32_t wx;
      float32_t wy;
      float32_t wz;
}) COMGS_magcorr_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      float32_t x;
      float32_t y;
} COMGS_coordObj_t;
#else
COMGSPACKED(
typedef struct {
      float32_t x;
      float32_t y;
}) COMGS_coordObj_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      float32_t angle;                                                          // axis angle object
      float32_t x;
      float32_t y;
      float32_t z;
} COMGS_axis_angle;
#else
COMGSPACKED(
typedef struct {
      float32_t angle;                                                          // axis angle object
      float32_t x;
      float32_t y;
      float32_t z;
}) COMGS_axis_angle;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      float32_t heading;                                                        // euler angle object
      float32_t attitude;
      float32_t bank;
} COMGS_euler_angle;
#else
COMGSPACKED(
typedef struct {
      float32_t heading;                                                        // euler angle object
      float32_t attitude;
      float32_t bank;
}) COMGS_euler_angle;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float32_t estimatedZ;                                                         // The current Z estimate, has same offset as asl
  float32_t velocityZ;                                                          // Vertical speed (world frame) integrated from vertical acceleration (m/s)
  float32_t estAlphaZrange;
  float32_t estAlphaAsl;
  float32_t velocityFactor;
  float32_t vAccDeadband;                                                       // Vertical acceleration deadband
  float32_t velZAlpha;                                                          // Blending factor to avoid vertical speed to accumulate error
  float32_t estimatedVZ;
} selfState_s;
#else
COMGSPACKED(
typedef struct {
  float32_t estimatedZ;                                                         // The current Z estimate, has same offset as asl
  float32_t velocityZ;                                                          // Vertical speed (world frame) integrated from vertical acceleration (m/s)
  float32_t estAlphaZrange;
  float32_t estAlphaAsl;
  float32_t velocityFactor;
  float32_t vAccDeadband;                                                       // Vertical acceleration deadband
  float32_t velZAlpha;                                                          // Blending factor to avoid vertical speed to accumulate error
  float32_t estimatedVZ;
}) selfState_s;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float32_t distance;                                                           // distance measurement
  uint64_t timestamp;                                                           // measurement timestamp
} tofMeasurement_t;
#else
COMGSPACKED(
typedef struct {
  float32_t distance;                                                           // distance measurement
  uint64_t timestamp;                                                           // measurement timestamp
}) tofMeasurement_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float32_t asl;                                                                // altitude above sea level
  float32_t hov;                                                                // height above ground
  float32_t press;
  float32_t temper;
} PressReading_t;
#else
COMGSPACKED(
typedef struct {
  float32_t asl;                                                                // altitude above sea level
  float32_t hov;                                                                // height above ground
  float32_t press;
  float32_t temper;
}) PressReading_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float64_t x;
  float64_t y;
  float64_t z;
  float64_t w;
} dVectr4d_t;
#else
COMGSPACKED(
typedef struct {
  float64_t x;
  float64_t y;
  float64_t z;
  float64_t w;
}) dVectr4d_t;                                                                  /* made this for double maths consider making union */
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint32_t timestamp;
  union {
    struct {
      float32_t q0;
      float32_t q1;
      float32_t q2;
      float32_t q3;
    };
    struct {
      float32_t x;
      float32_t y;
      float32_t z;
      float32_t w;
    };
  };
} quaternion_t;
#else
COMGSPACKED(
typedef struct {
  uint32_t timestamp;
  union {
    struct {
      float32_t q0;
      float32_t q1;
      float32_t q2;
      float32_t q3;
    };
    struct {
      float32_t x;
      float32_t y;
      float32_t z;
      float32_t w;
    };
  };
}) quaternion_t;
#endif

typedef union {
struct {
    float32_t x;
    float32_t y;
    float32_t z;
 };
 float32_t axis[3u];
} Axis3f_u;
 
#if defined(D_FT900)
typedef struct COMGSPACKED {
  PressReading_t baro;                                                          // associated with the barometer values
  Axis3f_u acc;                                                                 // accelerometer
  Axis3f_u gyro;                                                                // gyrometer
  Axis3f_u mag;                                                                 // magnetrometer
  uint64_t interruptTimestamp;
} sensorData_t;
#else
COMGSPACKED(
typedef struct {
  PressReading_t baro;                                                          // associated with the barometer values
  Axis3f_u acc;                                                                 // accelerometer
  Axis3f_u gyro;                                                                // gyrometer
  Axis3f_u mag;                                                                 // magnetrometer
  uint64_t interruptTimestamp;
}) sensorData_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint32_t timestamp;                                                           // Timestamp when the data was computed
  float32_t roll;
  float32_t pitch;
  float32_t yaw;
} attitude_t;
#else
COMGSPACKED(
typedef struct {
  uint32_t timestamp;                                                           // Timestamp when the data was computed
  float32_t roll;
  float32_t pitch;
  float32_t yaw;
}) attitude_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint32_t timestamp;                                                           // Timestamp when the data was computed
  float32_t x;
  float32_t y;
  float32_t z;
} point_t;
#else
COMGSPACKED(
typedef struct {
  uint32_t timestamp;                                                           // Timestamp when the data was computed
  float32_t x;
  float32_t y;
  float32_t z;
}) point_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint32_t tickLast;                                                            // Timestamp when the data was computed
  uint32_t ticksAccum;                                                          // accumulated ticks for the second stage timer
} secondStageTiming_t;
#else
COMGSPACKED(
typedef struct {
  uint32_t tickLast;                                                            // Timestamp when the data was computed
  uint32_t ticksAccum;                                                          // accumulated ticks for the second stage timer
}) secondStageTiming_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint32_t tickLast;                                                            // Timestamp when the data was computed for Tim (1st timer)
  uint32_t tickLast1;                                                           // Timestamp when the data was computed for Fin (2nd timer)
  uint32_t indiContLoopTRef;                                                    // indy controller loop time reference
  float32_t sample_time;                                                        // calculated sample period
  uint32_t ticksAccum;                                                          // accumulated ticks for the second stage timer
  uint8_t IndiTimInit : 1u;
  uint8_t IndiPosInit : 1u;
  uint8_t IndiFinInit : 1u;
  uint8_t IndiInFlight : 1u;
  uint8_t outerLoopActive : 1u;
  uint8_t indiContLoopTInit : 1u;
  uint8_t spare : 2u;
} IndyTiming_t;
#else
COMGSPACKED(
typedef struct {
  uint32_t tickLast;                                                            // Timestamp when the data was computed for Tim
  uint32_t tickLast1;                                                           // Timestamp when the data was computed for Fin
  uint32_t indiContLoopTRef;                                                    // indy controller loop time reference
  float32_t sample_time;                                                        // calculated sample period
  uint32_t ticksAccum;                                                          // accumulated ticks for the second stage timer
  uint8_t IndiTimInit : 1u;
  uint8_t IndiPosInit : 1u;
  uint8_t IndiFinInit : 1u;
  uint8_t IndiInFlight : 1u;
  uint8_t outerLoopActive : 1u;
  uint8_t indiContLoopTInit : 1u;
  uint8_t spare : 2u;
}) IndyTiming_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint32_t tickLast;                                                            // Timestamp when the data was computed for Tim (1st timer)
  float32_t sample_time;                                                        // calculated sample period
  uint32_t ticksAccum;                                                          // accumulated ticks for the second stage timer
  uint8_t MelTimInit : 1u;
  uint8_t spare : 7u;
} MellTiming_t;
#else
COMGSPACKED(
typedef struct {
  uint32_t tickLast;                                                            // Timestamp when the data was computed for Tim (1st timer)
  float32_t sample_time;                                                        // calculated sample period
  uint32_t ticksAccum;                                                          // accumulated ticks for the second stage timer
  uint8_t MelTimInit : 1u;
  uint8_t spare : 7u;
}) MellTiming_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  point_t position;                                                             // m
  point_t velocity;                                                             // m/s
  attitude_t attitude;                                                          // deg (legacy CF2 body coordinate system, where pitch is inverted)
  quaternion_t attitudeQuaternion;
  point_t acc;
} state_t;
#else
COMGSPACKED(
typedef struct {
  point_t position;                                                             // m
  point_t velocity;                                                             // m/s
  attitude_t attitude;                                                          // deg (legacy CF2 body coordinate system, where pitch is inverted)
  quaternion_t attitudeQuaternion;
  point_t acc;
}) state_t;
#endif

/* ----------- INDI Position Controller ----------------------------------------
    Copyright (c) 2019 Ewoud Smeur and Evghenii Volodscoi

 * This control algorithm is the Incremental Nonlinear Dynamic Inversion (INDI)
 * controller to control the position of the Crazyflie. It can be seen as an extension
 * (outer loop) to the already existing INDI attitude controller (inner loop).
 *
 * The control algorithm was implemented according to the publication in the
 * journal od Control Engineering Practice: Cascaded Incremental Nonlinear Dynamic
 * Inversion for MAV Disturbance Rejection
 * https://doi.org/10.1016/j.conengprac.2018.01.003
   ------------------------------------------------------------------------- */
#define MIN_THRUST  0                                                           // Thrust command (in motor units)
#define MAX_THRUST  60000
#define POSITION_INDI_FILT_CUTOFF 8.0f                                          // Cutoff frequency used in the filtering

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float32_t phi;
  float32_t theta;
  float32_t psi;
} Angles;
#else
COMGSPACKED(
typedef struct {
  float32_t phi;
  float32_t theta;
  float32_t psi;
}) Angles;
#endif

typedef enum
{
    CTRLStabilRate = 0,
    CTRLStabilReturnToHome,
    CTRLStabilFollow,
    CTRLStabilStabilize
} CTRLStabilStabilizeState_e;                                                   /* state engine for the controller stabilizer */

#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint8_t altitudeHold : 1u;
  uint8_t mode : 4u;                                                            /* CTRLStabilStabilizeState_e */
  uint8_t airMode : 1u;
  uint8_t spare : 2u;
} CtrlStabilizer_t;
#else
COMGSPACKED(
typedef struct {
  uint8_t altitudeHold : 1u;
  uint8_t mode : 4u;                                                            /* CTRLStabilStabilizeState_e */
  uint8_t airMode : 1u;
  uint8_t spare : 2u;
}) CtrlStabilizer_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float32_t thrust;
} control_t;
#else
COMGSPACKED(
typedef struct {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float32_t thrust;
}) control_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  Butterworth2LowPass ddxi[3u];
  Butterworth2LowPass ang[3u];
  Butterworth2LowPass thr[3u];
  float32_t filt_cutoff;
  float32_t act_dyn_posINDI;
  Vectr linear_accel_ref;
  Vectr linear_accel_err;
  Vectr linear_accel_s;                                                         // acceleration sensed
  Vectr linear_accel_f;                                                         // acceleration filtered
  Vectr linear_accel_ft;                                                        // acceleration filtered transformed to NED
  Angles attitude_s;                                                            // attitude senssed (here estimated)
  Angles attitude_f;                                                            // attitude filtered
  Angles attitude_c;                                                            // attitude commanded to the inner loop
  float32_t phi_tilde;                                                          // roll angle increment
  float32_t theta_tilde;                                                        // pitch angle increment
  float32_t T_tilde;                                                            // thrust increment
  float32_t T_inner;                                                            // thrust to inner INDI
  float32_t T_inner_f;
  float32_t T_incremented;
} IndiOuterVariables;
#else
COMGSPACKED(
typedef struct {
  Butterworth2LowPass ddxi[3u];
  Butterworth2LowPass ang[3u];
  Butterworth2LowPass thr[3u];
  float32_t filt_cutoff;
  float32_t act_dyn_posINDI;
  Vectr linear_accel_ref;
  Vectr linear_accel_err;
  Vectr linear_accel_s;                                                         // acceleration sensed
  Vectr linear_accel_f;                                                         // acceleration filtered
  Vectr linear_accel_ft;                                                        // acceleration filtered transformed to NED
  Angles attitude_s;                                                            // attitude senssed (here estimated)
  Angles attitude_f;                                                            // attitude filtered
  Angles attitude_c;                                                            // attitude commanded to the inner loop
  float32_t phi_tilde;                                                          // roll angle increment
  float32_t theta_tilde;                                                        // pitch angle increment
  float32_t T_tilde;                                                            // thrust increment
  float32_t T_inner;                                                            // thrust to inner INDI
  float32_t T_inner_f;
  float32_t T_incremented;
}) IndiOuterVariables;
#endif

typedef enum
{
  modeDisable = 0,
  modeAbs,
  modeVelocity
} stab_mode_t;

#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint32_t timestamp;
  attitude_t attitude;                                                          // deg
  attitude_t attitudeRate;                                                      // deg/s
  quaternion_t attitudeQuaternion;
  float32_t thrust;
  point_t position;                                                             // m
  point_t velocity;                                                             // m/s
  point_t acceleration;                                                         // m/s^2
  uint8_t velocity_body : 1u;                                                   // true if velocity is given in body frame; false if velocity is given in world frame
  uint8_t spare7 : 7u;
  struct {
    stab_mode_t x;
    stab_mode_t y;
    stab_mode_t z;
    stab_mode_t roll;
    stab_mode_t pitch;
    stab_mode_t yaw;
    stab_mode_t quat;
  } mode;
} setpoint_t;
#else
COMGSPACKED(
typedef struct {
  uint32_t timestamp;
  attitude_t attitude;                                                          // deg
  attitude_t attitudeRate;                                                      // deg/s
  quaternion_t attitudeQuaternion;
  float32_t thrust;
  point_t position;                                                             // m
  point_t velocity;                                                             // m/s
  point_t acceleration;                                                         // m/s^2
  uint8_t velocity_body : 1u;                                                   // true if velocity is given in body frame; false if velocity is given in world frame
  uint8_t spare7 : 7u;
  struct {
    stab_mode_t x;
    stab_mode_t y;
    stab_mode_t z;
    stab_mode_t roll;
    stab_mode_t pitch;
    stab_mode_t yaw;
    stab_mode_t quat;
  } mode;
}) setpoint_t;
#endif

/* ----------- INDI Attitude Controller ----------------------------------------
     Copyright (c) 2019 Ewoud Smeur and Andre Luis Ogando Paraense

 * This control algorithm is the Incremental Nonlinear Dynamic Inversion (INDI)
 * controller to control the position of the Crazyflie. It can be seen as an extension
 * (outer loop) to the already existing INDI attitude controller (inner loop).
 *
 * This is an implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 * http://arc.aiaa.org/doi/pdf/10.2514/1.G001490
   ------------------------------------------------------------------------- */
#define STABILIZATION_INDI_FILT_CUTOFF 8.0f                                     // these parameters are used in the filtering of the angular acceleration
#define STABILIZATION_INDI_FILT_CUTOFF_R STABILIZATION_INDI_FILT_CUTOFF         // the yaw sometimes requires more filtering
#define STABILIZATION_INDI_G1_P 0.0066146f                                      // these parameters are used in the filtering of the angular acceleration
#define STABILIZATION_INDI_G1_Q 0.0052125f
#define STABILIZATION_INDI_G1_R -0.001497f
#define STABILIZATION_INDI_G2_R 0.000043475f
#define STABILIZATION_INDI_REF_ERR_P 24.0f
#define STABILIZATION_INDI_REF_ERR_Q 24.0f
#define STABILIZATION_INDI_REF_ERR_R 24.0f
#define STABILIZATION_INDI_REF_RATE_P 14.0f
#define STABILIZATION_INDI_REF_RATE_Q 14.0f
#define STABILIZATION_INDI_REF_RATE_R 14.0f
#define STABILIZATION_INDI_ACT_DYN_P 0.03149f
#define STABILIZATION_INDI_ACT_DYN_Q 0.03149f
#define STABILIZATION_INDI_ACT_DYN_R 0.03149f

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float32_t p;                                                                  // rad/s
  float32_t q;
  float32_t r;
} FloatRates;
#else
COMGSPACKED(
typedef struct {
  float32_t p;                                                                  // rad/s
  float32_t q;
  float32_t r;
}) FloatRates;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float32_t err_p;
  float32_t err_q;
  float32_t err_r;
  float32_t rate_p;
  float32_t rate_q;
  float32_t rate_r;
} ReferenceSystem;
#else
COMGSPACKED(
typedef struct {
  float32_t err_p;
  float32_t err_q;
  float32_t err_r;
  float32_t rate_p;
  float32_t rate_q;
  float32_t rate_r;
}) ReferenceSystem;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float32_t thrust;
  FloatRates angular_accel_ref;
  FloatRates du;
  FloatRates u_in;
  FloatRates u_act_dyn;
  float32_t rate_d[3];
  Butterworth2LowPass u[3];
  Butterworth2LowPass rate[3];
  FloatRates g1;
  float32_t g2;
  ReferenceSystem reference_acceleration;
  FloatRates act_dyn;
  float32_t filt_cutoff;
  float32_t filt_cutoff_r;
} IndiVariables;
#else
COMGSPACKED(
typedef struct {
  float32_t thrust;
  FloatRates angular_accel_ref;
  FloatRates du;
  FloatRates u_in;
  FloatRates u_act_dyn;
  float32_t rate_d[3];
  Butterworth2LowPass u[3];
  Butterworth2LowPass rate[3];
  FloatRates g1;
  float32_t g2;
  ReferenceSystem reference_acceleration;
  FloatRates act_dyn;
  float32_t filt_cutoff;
  float32_t filt_cutoff_r;
}) IndiVariables;
#endif
/* ------------- end INDI Controllers -------------------------------------   */
/* ---------------------- Mellinger PP trajectory --------------------------- */
#define PP_DEGREE (7)
#define PP_SIZE (PP_DEGREE + 1)

#if defined(D_FT900)
typedef struct COMGSPACKED {
   Vectr pos;
   Vectr vel;
   Vectr acc;
   Vectr omega;
   float32_t yaw;
} traj_eval;                                                                    // Trajectory Evaluation Object
#else
COMGSPACKED(
typedef struct {
   Vectr pos;
   Vectr vel;
   Vectr acc;
   Vectr omega;
   float32_t yaw;
}) traj_eval;                                                                   // Trajectory Evaluation Object
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float32_t p[4u][PP_SIZE];
  float32_t duration;                                                           // TODO use int millis instead?
} poly4d;                                                                       // 4d single polynomial piece for x-y-z-yaw, includes duration.
#else
COMGSPACKED(
typedef struct {
  float32_t p[4u][PP_SIZE];
  float32_t duration;                                                           // TODO use int millis instead?
}) poly4d;                                                                      // 4d single polynomial piece for x-y-z-yaw, includes duration.
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float32_t t_begin;
  float32_t timescale;
  Vectr shift;
  unsigned char n_pieces;
  poly4d* pieces;
} piecewise_traj;                                                               // piecewise polynomial trajectories
#else
COMGSPACKED(
typedef struct {
  float32_t t_begin;
  float32_t timescale;
  Vectr shift;
  unsigned char n_pieces;
  poly4d* pieces;
}) piecewise_traj;                                                              // piecewise polynomial trajectories
#endif

/* ------------- end PPTraj -----------------------------------------------   */

#if defined(D_FT900)
typedef struct COMGSPACKED {
   uint8_t ProfileId;                                                           // The profile in question 0-4 or 255 = current
   uint8_t P[3u];                                                               // Proportional bands (pitch,roll,yaw)
   uint8_t I[3u];                                                               // Integral band (pitch,roll,yaw)
   uint8_t D[3u];                                                               // Derivative bands (pitch,roll,yaw)
   uint8_t power[3u];                                                           // power
   uint8_t invert[3u];                                                          // reverse or forward direction for PID
   uint8_t poles[3u];                                                           // number of poles in motor
   uint8_t rc_mode;                                                             // speed or angle
   uint8_t pwm_freq;                                                            // speed of pwm outputs (PID.out)
} COMGS_pid_object;
#else
COMGSPACKED(
typedef struct {
   uint8_t ProfileId;                                                           // The profile in question 0-4 or 255 = current
   uint8_t P[3u];                                                               // Proportional bands (pitch,roll,yaw)
   uint8_t I[3u];                                                               // Integral band (pitch,roll,yaw)
   uint8_t D[3u];                                                               // Derivative bands (pitch,roll,yaw)
   uint8_t power[3u];                                                           // power
   uint8_t invert[3u];                                                          // reverse or forward direction for PID
   uint8_t poles[3u];                                                           // number of poles in motor
   uint8_t rc_mode;                                                             // speed or angle
   uint8_t pwm_freq;                                                            // speed of pwm outputs (PID.out)
}) COMGS_pid_object;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      float32_t z_real;                                                         // Ideal position: the ideal value we wish to measure
      float32_t z_measured;                                                     // Measured position:  the 'noisy' value we measured
      float32_t x_est;                                                          // Kalman position:
      float32_t sum_error_measure;                                              // Total error using measurement raw
      float32_t sum_error_kalman;                                               // Total error using kalman
      float32_t Diff_err;                                                       // Reduction in error with kalman
} COMGS_kalman;
#else
COMGSPACKED(
typedef struct {
      float32_t z_real;                                                         // Ideal position: the ideal value we wish to measure
      float32_t z_measured;                                                     // Measured position:  the 'noisy' value we measured
      float32_t x_est;                                                          // Kalman position:
      float32_t sum_error_measure;                                              // Total error using measurement raw
      float32_t sum_error_kalman;                                               // Total error using kalman
      float32_t Diff_err;                                                       // Reduction in error with kalman
}) COMGS_kalman;                                                                // Kalman filter structure
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
        uint8_t  phase;                                                         // current phase of encoder (from 0 to 3)
        int32_t  distance_count;                                                // distance measured by cumulative steps forward or backwards since last update
        uint32_t total_count;                                                   // total number of successful readings from sensor (used for sensor quality calcs)
        uint32_t error_count;                                                   // total number of errors reading from sensor (used for sensor quality calcs)
        uint32_t last_reading_ms;                                               // system time of last update from encoder
        uint8_t last_pin_a : 1u;
        uint8_t last_pin_b : 1u;
        uint8_t last_pin_a_value : 1u;
        uint8_t last_pin_b_value : 1u;
        uint8_t pin_a : 1u;
        uint8_t pin_b : 1u;
        uint8_t spare : 2u;
} Quad_Enc_t;                                                                   /* quaderature encoder */
#else
COMGSPACKED(
typedef struct  {
        uint8_t  phase;                                                         // current phase of encoder (from 0 to 3)
        int32_t  distance_count;                                                // distance measured by cumulative steps forward or backwards since last update
        uint32_t total_count;                                                   // total number of successful readings from sensor (used for sensor quality calcs)
        uint32_t error_count;                                                   // total number of errors reading from sensor (used for sensor quality calcs)
        uint32_t last_reading_ms;                                               // system time of last update from encoder
        uint8_t last_pin_a : 1u;
        uint8_t last_pin_b : 1u;
        uint8_t last_pin_a_value : 1u;
        uint8_t last_pin_b_value : 1u;
        uint8_t pin_a : 1u;
        uint8_t pin_b : 1u;
        uint8_t spare : 2u;
}) Quad_Enc_t;
#endif

/* ------------ from Clover by Oleg Kalachev ------------------------------- */
#if defined(D_FT900)
typedef struct COMGSPACKED {
  Vectr position;
  quat orientation;
  Vectr theta;
  uint32_t lastTimRef;
  uint32_t stamp;
  uint8_t state;
} geoPose_t;
#else
COMGSPACKED(
typedef struct {
  Vectr position;
  quat orientation;
  Vectr theta;
  uint32_t lastTimRef;
  uint32_t stamp;
  uint8_t state;
}) geoPose_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  Vectr linear;
  Vectr angular;
} twist_t;
#else
COMGSPACKED(
typedef struct {
  Vectr linear;
  Vectr angular;
}) twist_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  Vectr vector;
  quat prev_rot;
  quat curr_rot;
  float32_t roi_px_;
  float32_t width;
  float32_t height;
  uint8_t frame_id;
  uint32_t stamp;
  uint32_t prev_stamp;
  uint32_t integration_time_us;
  float32_t integrated_xgyro;
  float32_t integrated_ygyro;
  float32_t integrated_zgyro;
  float32_t integrated_x;
  float32_t integrated_y;
  uint8_t quality;
  twist_t twist;
  uint32_t lastTimRef;
  uint8_t state;
} optical_flowObj_t;
#else
COMGSPACKED(
typedef struct {
  Vectr vector;
  quat prev_rot;
  quat curr_rot;
  float32_t roi_px_;
  float32_t width;
  float32_t height;
  uint8_t frame_id;
  uint32_t stamp;
  uint32_t prev_stamp;
  uint32_t integration_time_us;
  float32_t integrated_xgyro;
  float32_t integrated_ygyro;
  float32_t integrated_zgyro;
  float32_t integrated_x;
  float32_t integrated_y;
  uint8_t quality;
  twist_t twist;
  uint32_t lastTimRef;
  uint8_t state;
}) optical_flowObj_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  Vectr force;
  Vectr torque;
} wrench_t;
#else
COMGSPACKED(
typedef struct {
  Vectr force;
  Vectr torque;
}) wrench_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  geoPose_t pose;
  twist_t twist;
  float64_t leftJoint;
  float64_t rightJoint;
} odom_t;
#else
COMGSPACKED(
typedef struct {
  geoPose_t pose;
  twist_t twist;
  float64_t leftJoint;
  float64_t rightJoint;
}) odom_t;
#endif

#if defined(TF_DISTORT)
/* ==== motion capture systems ETH Zurich Ref: Markus Achtelik ====== */
#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint16_t frameID;
  uint8_t once : 1u;
  uint8_t spare : 7u;
  uint32_t last_time;
  Vectr p;
  quat orient;
  int16_t storedSeed;
} StampedTransform_t;
#else
COMGSPACKED(
typedef struct {
  uint16_t frameID;
  uint8_t once : 1u;
  uint8_t spare : 7u;
  uint32_t last_time;
  Vectr p;
  quat orient;
  int16_t storedSeed;
}) StampedTransform_t;
#endif

typedef enum
{
  vicon_distort_UNIFORM = 1,
  vicon_distort_NORMAL
} vicon_noise_type_e;

#if defined(D_FT900)
typedef struct COMGSPACKED {
  vicon_noise_type_e noise_type;
  float64_t sigma_roll_pitch;
  float64_t sigma_yaw;
  float64_t sigma_xy;
  float64_t sigma_z;
  float64_t random_walk_tau_xy;
  float64_t random_walk_tau_z;
  float64_t random_walk_sigma_xy;
  float64_t random_walk_sigma_z;
  float64_t random_walk_k_xy;
  float64_t random_walk_k_z;
  float64_t position_scale;
  uint16_t tf_frame_in;
  uint16_t tf_frame_out;
  uint16_t tf_pub_rate;
  uint16_t tf_ref_frame;
} distort_config_t;
#else
COMGSPACKED(
typedef struct {
  vicon_noise_type_e noise_type;
  float64_t sigma_roll_pitch;
  float64_t sigma_yaw;
  float64_t sigma_xy;
  float64_t sigma_z;
  float64_t random_walk_tau_xy;
  float64_t random_walk_tau_z;
  float64_t random_walk_sigma_xy;
  float64_t random_walk_sigma_z;
  float64_t random_walk_k_xy;
  float64_t random_walk_k_z;
  float64_t position_scale;
  uint16_t tf_frame_in;
  uint16_t tf_frame_out;
  uint16_t tf_pub_rate;
  uint16_t tf_ref_frame;
}) distort_config_t;
#endif
#endif /* --- end tf dstortion --- */

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float64_t left_pos;                                                           // = left_->getPosition();
  float64_t right_pos;                                                          // = right_->getPosition();
  float64_t left_dx;                                                            // = angles::shortest_angular_distance(left_last_position_, left_pos)/radians_per_meter_;
  float64_t right_dx;                                                           // = angles::shortest_angular_distance(right_last_position_, right_pos)/radians_per_meter_;
  float64_t left_vel;                                                           // = static_cast<double>(left_->getVelocity())/radians_per_meter_;
  float64_t right_vel;                                                          // = static_cast<double>(right_->getVelocity())/radians_per_meter_;
} movingObjectStat_t;
#else
COMGSPACKED(
typedef struct {
  float64_t left_pos;                                                           // = left_->getPosition();
  float64_t right_pos;                                                          // = right_->getPosition();
  float64_t left_dx;                                                            // = angles::shortest_angular_distance(left_last_position_, left_pos)/radians_per_meter_;
  float64_t right_dx;                                                           // = angles::shortest_angular_distance(right_last_position_, right_pos)/radians_per_meter_;
  float64_t left_vel;                                                           // = static_cast<double>(left_->getVelocity())/radians_per_meter_;
  float64_t right_vel;                                                          // = static_cast<double>(right_->getVelocity())/radians_per_meter_;
}) movingObjectStat_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  int16_t id;
  float32_t size;
  float32_t x;
  float32_t y;
  float32_t z;
  float32_t yaw;
  float32_t pitch;
  float32_t roll;
} aruco_markerFile_t;
#else
COMGSPACKED(
typedef struct {
  int16_t id;
  float32_t size;
  float32_t x;
  float32_t y;
  float32_t z;
  float32_t yaw;
  float32_t pitch;
  float32_t roll;
}) aruco_markerFile_t;                                                          /* aruco marker file definition */
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
   float32_t Actuator1;
   float32_t Actuator2;
   float32_t Actuator3;
   float32_t Actuator4;
   float32_t Actuator5;
   float32_t Actuator6;
   float32_t Actuator7;
   float32_t Actuator8;
} kinovaAngles_t;
#else
COMGSPACKED(
typedef struct {
   float32_t Actuator1;
   float32_t Actuator2;
   float32_t Actuator3;
   float32_t Actuator4;
   float32_t Actuator5;
   float32_t Actuator6;
   float32_t Actuator7;
   float32_t Actuator8;
}) kinovaAngles_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
  float64_t control[4u];
  float64_t dx[12u];
  float64_t state_matrix[12u][12u];
  float64_t gain_matrix[12u][12u];
  float64_t gain_matrixC[12u][12u];
  float64_t state[12u];
} state_control_vector_t;
#else
COMGSPACKED(
typedef struct {
  float64_t control[4u];
  float64_t dx[12u];
  float64_t state_matrix[12u][12u];                                             // A
  float64_t gain_matrix[12u][12u];                                              // B
  float64_t gain_matrixC[12u][12u];                                             // C
  float64_t state[12u];                                                         // D ode
}) state_control_vector_t;
#endif

/* ==== PMLPS Poor Man's Local Positioning System ===== */
/* Local positioning system using OpenMV cam M7 */
#if defined(D_FT900)
typedef struct COMGSPACKED {
  float32_t m_b0;
  float32_t m_b1;
  float32_t m_b2;
  float32_t m_a1;
  float32_t m_a2;
  float32_t m_d1;
  float32_t m_d2;
} biquadLPF_t;
#else
COMGSPACKED(
typedef struct {
  float32_t m_b0;
  float32_t m_b1;
  float32_t m_b2;
  float32_t m_a1;
  float32_t m_a2;
  float32_t m_d1;
  float32_t m_d2;
}) biquadLPF_t;
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      uint8_t mode_began : 1u;                                                  // flag for initialise
      uint8_t spare : 7u;
      float32_t eInt[3u];                                                       // Integral error
      float32_t Ki;                                                             // Integral band
      float32_t Kp;                                                             // Proportional band
      float32_t Quaternion[4u];                                                 // Output
      uint64_t lasttime;                                                        // last time that we got the gloabal 100 ms interrupt tick
} COMGS_QControl_t;
#else
COMGSPACKED(
typedef struct {
      uint8_t mode_began : 1u;                                                  // flag for initialise
      uint8_t spare : 7u;
      float32_t eInt[3u];                                                       // Integral error
      float32_t Ki;                                                             // Integral band
      float32_t Kp;                                                             // Proportional band
      float32_t  Quaternion[4u];                                                // Output
      uint64_t lasttime;                                                        // last time that we got the gloabal 100 ms interrupt tick
}) COMGS_QControl_t;
#endif

#ifndef USE_CAL_UNLOCK                                                          // The define does not wait for calibration to complete
#if defined(D_FT900)
typedef struct COMGSPACKED {
     uint16_t centerValx;                                                       // X center
     uint16_t centerValy;                                                       // Y center
     uint16_t centerValz;                                                       // Z center
     uint16_t maxValx;                                                          // X max
     uint16_t maxValy;                                                          // Y max
     uint16_t maxValz;                                                          // Z max
     uint16_t minValx;                                                          // X min
     uint16_t minValy;                                                          // Y min
     uint16_t minValz;                                                          // Z min
     uint8_t butState;                                                          // The tick boxes for completion
} COMGS_joy_calib_t;                                                           // Joystick calibration correction structure
#else
COMGSPACKED(
typedef struct {
     uint16_t centerValx;                                                       // X center
     uint16_t centerValy;                                                       // Y center
     uint16_t centerValz;                                                       // Z center
     uint16_t maxValx;                                                          // X max
     uint16_t maxValy;                                                          // Y max
     uint16_t maxValz;                                                          // Z max
     uint16_t minValx;                                                          // X min
     uint16_t minValy;                                                          // Y min
     uint16_t minValz;                                                          // Z min
     uint8_t butState;                                                          // The tick boxes for completion
}) COMGS_joy_calib_t;                                                           // Joystick calibration correction structure
#endif
#else
#if defined(D_FT900)
typedef struct COMGSPACKED {
     uint16_t centerValx;                                                       // X center
     uint16_t centerValy;                                                       // Y center
     uint16_t centerValz;                                                       // Z center
     uint16_t maxValx;                                                          // X max
     uint16_t maxValy;                                                          // Y max
     uint16_t maxValz;                                                          // Z max
     uint16_t minValx;                                                          // X min
     uint16_t minValy;                                                          // Y min
     uint16_t minValz;                                                          // Z min
     uint8_t butState;                                                          // The tick boxes for completion
     uint8_t seqState;                                                          // sequence state if non waiting
     uint64_t lasttime;                                                         // stored time for non waiitng
     uint8_t counter1;                                                          // internal sequence step for time calculation
} COMGS_joy_calib_t;                                                           // Joystick calibration correction structure
#else
COMGSPACKED(
typedef struct {
     uint16_t centerValx;                                                       // X center
     uint16_t centerValy;                                                       // Y center
     uint16_t centerValz;                                                       // Z center
     uint16_t maxValx;                                                          // X max
     uint16_t maxValy;                                                          // Y max
     uint16_t maxValz;                                                          // Z max
     uint16_t minValx;                                                          // X min
     uint16_t minValy;                                                          // Y min
     uint16_t minValz;                                                          // Z min
     uint8_t butState;                                                          // The tick boxes for completion
     uint8_t seqState;                                                          // sequence state if non waiting
     uint64_t lasttime;                                                         // stored time for non waiitng
     uint8_t counter1;                                                          // internal sequence step for time calculation
}) COMGS_joy_calib_t;                                                           // Joystick calibration correction structure
#endif
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      uint8_t angleReq : 1u;                                                    // Option flags
      uint8_t useDefault: 1u;                                                   // use the default script parameters
      uint8_t swapPitchRoll : 1u;                                               // swaps the pitch for roll
      uint8_t swapRollYaw : 1u;                                                 // swap the roll for yaw
      uint8_t ScriptNumber : 4u;                                                // The valid script number to select (atm 0 ir 1)
      float32_t PA;                                                             // Pitch Angle
      float32_t YA;                                                             // Yaw Angle
      float32_t YS;                                                             // Yaw Speed
      float32_t PS;                                                             // Pitch Speed
      float32_t RA;                                                             // Roll Angle
      float32_t RS;                                                             // Roll Speed
} COMGS_Script_t;                                                              // Time lapse script type structure
#else
COMGSPACKED(
typedef struct {
      uint8_t angleReq : 1u;                                                    // Option flags
      uint8_t useDefault: 1u;                                                   // use the default script parameters
      uint8_t swapPitchRoll : 1u;                                               // swaps the pitch for roll
      uint8_t swapRollYaw : 1u;                                                 // swap the roll for yaw
      uint8_t ScriptNumber : 4u;                                                // The valid script number to select (atm 0 ir 1)
      float32_t PA;                                                             // Pitch Angle
      float32_t YA;                                                             // Yaw Angle
      float32_t YS;                                                             // Yaw Speed
      float32_t PS;                                                             // Pitch Speed
      float32_t RA;                                                             // Roll Angle
      float32_t RS;                                                             // Roll Speed
}) COMGS_Script_t;                                                              // Time lapse script type structure
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      uint8_t ReqState : 1u;                                                    // Requested state of the motor
      uint8_t LastReq: 1u;                                                      // Last requested state
      uint8_t State: 1u;                                                        // Motor state event
      uint8_t ESD: 1u;                                                          // Motor ESD Event
      uint8_t FTS : 1u;                                                         // Fail to stop/start alarm
      uint8_t Hold : 1u;                                                        // Motor in Hold state
      uint8_t EcalErr: 1u;                                                      // Encoder calibration error
      uint8_t Changing : 1u;                                                    // Change of state requested

      float32_t TimeMax;                                                        // Maximum time to wait for a change of state

      uint64_t timeElapsed;                                                     // time elapsed since last motion
      uint64_t lasttime;                                                        // Last known ms counter (counted in 100ms slices)
} COMGS_Motor_t;                                                               // Motor object
#else
COMGSPACKED(
typedef struct {
      uint8_t ReqState : 1u;                                                    // Requested state of the motor
      uint8_t LastReq: 1u;                                                      // Last requested state
      uint8_t State: 1u;                                                        // Motor state event
      uint8_t ESD: 1u;                                                          // Motor ESD Event
      uint8_t FTS : 1u;                                                         // Fail to stop/start alarm
      uint8_t Hold : 1u;                                                        // Motor in Hold state
      uint8_t EcalErr: 1u;                                                      // Encoder calibration error
      uint8_t Changing : 1u;                                                    // Change of state requested

      float32_t TimeMax;                                                        // Maximum time to wait for a change of state

      int64_t timeElapsed;                                                      // time elapsed since last motion
      uint64_t lasttime;                                                        // Last known ms counter (counted in 100ms slices)
}) COMGS_Motor_t;                                                               // Motor object
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      uint16_t Year;                                                            // 4 digit year
      uint8_t Month;                                                            // Month
      uint8_t Day;                                                              // Day
      uint8_t Hour;                                                             // Motor ESD Event
      uint8_t Minute;                                                           // Fail to stop/start alarm
      uint8_t Second;                                                           // Motor in Hold state
} COMGS_DateTime_t;                                                             // Date and Time object
#else
COMGSPACKED(
typedef struct {
      uint16_t Year;                                                            // 4 digit year
      uint8_t Month;                                                            // Month
      uint8_t Day;                                                              // Day
      uint8_t Hour;                                                             // Motor ESD Event
      uint8_t Minute;                                                           // Fail to stop/start alarm
      uint8_t Second;                                                           // Motor in Hold state
}) COMGS_DateTime_t;                                                            // Date and Time object
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
      uint32_t hmiCommitRequest : 1u;                                           // Requested to do an encoder commit
      uint32_t hmiShutdRequest : 1u;                                            // Requested to do an encoder shutdown
      uint32_t hmiRebootRequest : 1u;                                           // Requested to do an encoder reboot
      uint32_t hmiRecordRequest : 1u;                                           // Requested to do an encoder record to disk for all channels
      uint32_t hmiStopRequest : 1u;                                             // Requested to do an encoder recording stop for all channels
      uint32_t hmiMarkRequest : 1u;                                             // Requested to do an encoder mark with a text the disk record for all channels
      uint32_t hmiRecordRequestCh1 : 1u;                                        // Requested to do an encoder record to disk for channel 1
      uint32_t hmiStopRequestCh1 : 1u;                                          // Requested to do an encoder recording stop for channel 1
      uint32_t hmiMarkRequestCh1 : 1u;                                          // Requested to do an encoder mark with a text the disk record for channel 1
      uint32_t hmiRecordRequestCh2 : 1u;                                        // Requested to do an encoder record to disk for channel 2
      uint32_t hmiStopRequestCh2 : 1u;                                          // Requested to do an encoder recording stop for channel 2
      uint32_t hmiMarkRequestCh2 : 1u;                                          // Requested to do an encoder mark with a text the disk record for channel 2
      uint32_t hmiRecordRequestCh3 : 1u;                                        // Requested to do an encoder record to disk for channel 3
      uint32_t hmiStopRequestCh3 : 1u;                                          // Requested to do an encoder recording stop for channel 3
      uint32_t hmiMarkRequestCh3 : 1u;                                          // Requested to do an encoder mark with a text the disk record for channel 3
      uint32_t hmiRecordRequestCh4 : 1u;                                        // Requested to do an encoder record to disk for channel 4
      uint32_t hmiStopRequestCh4 : 1u;                                          // Requested to do an encoder recording stop for channel 4
      uint32_t hmiMarkRequestCh4 : 1u;                                          // Requested to do an encoder mark with a text the disk record for channel 4
      uint32_t hmiEraseRequest : 1u;                                            // Request to erase on all channels
      uint32_t hmiEraseRequestCh1 : 1u;                                         // Request to erase on channel 1
      uint32_t hmiEraseRequestCh2 : 1u;                                         // Request to erase on channel 2
      uint32_t hmiEraseRequestCh3 : 1u;                                         // Request to erase on channel 3
      uint32_t hmiEraseRequestCh4 : 1u;                                         // Request to erase on channel 4
      uint32_t sparebits : 9u;
} COMGS_Request_t;                                                              // HMI flag request object
#else
COMGSPACKED(
typedef struct {
      uint32_t hmiCommitRequest : 1u;                                           // Requested to do an encoder commit
      uint32_t hmiShutdRequest : 1u;                                            // Requested to do an encoder shutdown
      uint32_t hmiRebootRequest : 1u;                                           // Requested to do an encoder reboot
      uint32_t hmiRecordRequest : 1u;                                           // Requested to do an encoder record to disk for all channels
      uint32_t hmiStopRequest : 1u;                                             // Requested to do an encoder recording stop for all channels
      uint32_t hmiMarkRequest : 1u;                                             // Requested to do an encoder mark with a text the disk record for all channels
      uint32_t hmiRecordRequestCh1 : 1u;                                        // Requested to do an encoder record to disk for channel 1
      uint32_t hmiStopRequestCh1 : 1u;                                          // Requested to do an encoder recording stop for channel 1
      uint32_t hmiMarkRequestCh1 : 1u;                                          // Requested to do an encoder mark with a text the disk record for channel 1
      uint32_t hmiRecordRequestCh2 : 1u;                                        // Requested to do an encoder record to disk for channel 2
      uint32_t hmiStopRequestCh2 : 1u;                                          // Requested to do an encoder recording stop for channel 2
      uint32_t hmiMarkRequestCh2 : 1u;                                          // Requested to do an encoder mark with a text the disk record for channel 2
      uint32_t hmiRecordRequestCh3 : 1u;                                        // Requested to do an encoder record to disk for channel 3
      uint32_t hmiStopRequestCh3 : 1u;                                          // Requested to do an encoder recording stop for channel 3
      uint32_t hmiMarkRequestCh3 : 1u;                                          // Requested to do an encoder mark with a text the disk record for channel 3
      uint32_t hmiRecordRequestCh4 : 1u;                                        // Requested to do an encoder record to disk for channel 4
      uint32_t hmiStopRequestCh4 : 1u;                                          // Requested to do an encoder recording stop for channel 4
      uint32_t hmiMarkRequestCh4 : 1u;                                          // Requested to do an encoder mark with a text the disk record for channel 4
      uint32_t hmiEraseRequest : 1u;                                            // Request to erase on all channels
      uint32_t hmiEraseRequestCh1 : 1u;                                         // Request to erase on channel 1
      uint32_t hmiEraseRequestCh2 : 1u;                                         // Request to erase on channel 2
      uint32_t hmiEraseRequestCh3 : 1u;                                         // Request to erase on channel 3
      uint32_t hmiEraseRequestCh4 : 1u;                                         // Request to erase on channel 4
      uint32_t sparebits : 9u;
}) COMGS_Request_t;                                                             // HMI flag request object
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
       uint8_t   delta;                                                         // delta change value, must change by this to send a message
       uint32_t  value;                                                         // Current value store (largest value bit rate > 16 bits)
       uint32_t  last;                                                          // Last Stored value
} XS_Hmi_Slider_t;                                                              // A slider object on the HMI requesting a change message to the encoder
#else
// Structure for a camera object on the GUI
COMGSPACKED(
typedef struct {
       uint8_t   delta;                                                         // delta change value, must change by this to send a message
       uint32_t  value;                                                         // Current value store (largest value bit rate > 16 bits)
       uint32_t  last;                                                          // Last Stored value
}) XS_Hmi_Slider_t;                                                             // A slider object on the HMI requesting a change message to the encoder
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
       uint8_t wifi : 1u;                                                       // toggle wifi state
       uint8_t record : 1u;                                                     // record to disk
       uint8_t stop : 1u;                                                       // stop recording to disk
       uint8_t up : 1u;                                                         // up keypad
       uint8_t down : 1u;                                                       // down keypad
       uint8_t right : 1u;                                                      // right keypad
       uint8_t left : 1u;                                                       // left keypad
       uint8_t confirm : 1u;                                                    // confirm keypad

       uint8_t ntsc : 1u;                                                       // 1=ntsc 0=pal
       uint8_t writetxt : 1u;                                                   // text write request to screen
       uint8_t mode : 2u;                                                       // mode video photo osd
       uint8_t time : 1u;                                                       // change the time of the camera
       uint8_t resol : 1u;                                                      // change the resolution
       uint8_t charset : 1u;                                                    // select the charset
       uint8_t spare : 1u;
} RC_Hmi_Request_t;                                                            // Run Cam HMI Pushbuttons
#else
COMGSPACKED(
typedef struct {
       uint8_t wifi : 1u;                                                       // toggle wifi state
       uint8_t record : 1u;                                                     // record to disk
       uint8_t stop : 1u;                                                       // stop recording to disk
       uint8_t up : 1u;                                                         // up keypad
       uint8_t down : 1u;                                                       // down keypad
       uint8_t right : 1u;                                                      // right keypad
       uint8_t left : 1u;                                                       // left keypad
       uint8_t confirm : 1u;                                                    // confirm keypad

       uint8_t ntsc : 1u;                                                       // 1=ntsc 0=pal
       uint8_t writetxt : 1u;                                                   // text write request to screen
       uint8_t mode : 2u;                                                       // mode video photo osd
       uint8_t time : 1u;                                                       // change the time of the camera
       uint8_t resol : 1u;                                                      // change the resolution
       uint8_t charset : 1u;                                                    // select the charset
       uint8_t spare : 1u;
}) RC_Hmi_Request_t;                                                            // Run Cam HMI Pushbuttons
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
        uint8_t CableConnection  : 1u;                                          // Cable is connected
        uint8_t VidOrPho  : 1u;                                                 // video mode = 0 or photo mode = 1
        uint8_t SDCardStatus : 1u;                                              // status of the SD Card
        uint8_t spare : 4u;

        uint8_t Battpercent;                                                    // percent remaining in battery
        // uint32_t SDcardSpace;                                                   // SD Card space remaining
        uint8_t Token;                                                          // Token
        uint32_t size;                                                          // file size
        int8_t rval;                                                            // function return code
        uint32_t free_space;                                                    // SD Card free space bytes
        uint32_t total_space;                                                   // SD Card total space bytes
        unsigned char LastPicName[60u];                                          // full path and last picture name
        unsigned char md5sum[MD5_MAX_LEN];                                      // the mds info for the given file
        unsigned char resolution[10u];                                           // resolution string returned
} COMGS_YiDig_t;                                                                // Struct to hold the information read back
#else
COMGSPACKED(
typedef struct  {
        uint8_t CableConnection  : 1u;                                          // Cable is connected
        uint8_t VidOrPho  : 1u;                                                 // video mode = 0 or photo mode = 1
        uint8_t SDCardStatus : 1u;                                              // status of the SD Card
        uint8_t spare : 4u;

        uint8_t Battpercent;                                                    // percent remaining in battery
        // uint32_t SDcardSpace;                                                   // SD Card space remaining
        uint8_t Token;                                                          // Token
        uint32_t size;                                                          // file size
        int8_t rval;                                                            // function return code
        uint32_t free_space;                                                    // SD Card free space bytes
        uint32_t total_space;                                                   // SD Card total space bytes
        unsigned char LastPicName[60u];                                          // full path and last picture name
        unsigned char md5sum[MD5_MAX_LEN];                                      // the mds info for the given file
        unsigned char resolution[10u];                                           // resolution string returned
}) COMGS_YiDig_t;                                                               // Struct to hold the information read back
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
   uint8_t VideoRes : 4u;                                                        // 4 bit selection for xy_video_res
   uint8_t PhotoSz : 3u;                                                         // 2 bit selection for xy_photo_sz
   uint8_t OnOffState : 1u;                                                      // On or Off for the On_Off Type Selection
   uint8_t TimeLapOpt : 4u;                                                      // time lapse option
   uint8_t TimeLapDuration : 4u;                                                 // Time Lapse duration
   uint8_t SlowMotion : 4u;                                                      // Slow motion option
   uint8_t BurstCap : 4u;                                                        // Burst Capture setting
   uint8_t PreContTim : 4u;                                                      // Precise cont time setting
   uint8_t RecPhoTim : 4u;                                                       // Record or photo time
   uint8_t LoopRecDur : 4u;                                                      // Loop record duration
   uint8_t VidPhoQ : 4u;                                                         // Video photo quality
   uint8_t VidPhoStamp : 4u;                                                     // Video photo stamp
   uint8_t VideoStd: 3u;                                                         // Video Standard
   uint8_t OnOffTypSel : 8u;                                                     // On_Off Type Selection
   uint8_t VidOutDev : 4u;                                                       // Video output device type selection
   uint8_t MeterOpt : 4u;                                                        // meter options
   uint8_t LedMode : 4u;                                                         // led mode
   uint8_t BuzzVol : 4u;                                                         // buzzer volume
   uint8_t CapMode : 4u;                                                         // capture modes
   uint8_t spare : 4u;
} COMGS_YiOptSelect_t;                                                          // Struct to hold the various options for YiCam configurations
#else
COMGSPACKED(
typedef struct  {
   uint8_t VideoRes : 4u;                                                        // 4 bit selection for xy_video_res
   uint8_t PhotoSz : 3u;                                                         // 2 bit selection for xy_photo_sz
   uint8_t OnOffState : 1u;                                                      // On or Off for the On_Off Type Selection
   uint8_t TimeLapOpt : 4u;                                                      // time lapse option
   uint8_t TimeLapDuration : 4u;                                                 // Time Lapse duration
   uint8_t SlowMotion : 4u;                                                      // Slow motion option
   uint8_t BurstCap : 4u;                                                        // Burst Capture setting
   uint8_t PreContTim : 4u;                                                      // Precise cont time setting
   uint8_t RecPhoTim : 4u;                                                       // Record or photo time
   uint8_t LoopRecDur : 4u;                                                      // Loop record duration
   uint8_t VidPhoQ : 4u;                                                         // Video photo quality
   uint8_t VidPhoStamp : 4u;                                                     // Video photo stamp
   uint8_t VideoStd: 3u;                                                         // Video Standard
   uint8_t OnOffTypSel : 8u;                                                     // On_Off Type Selection
   uint8_t VidOutDev : 4u;                                                       // Video output device type selection
   uint8_t MeterOpt : 4u;                                                        // meter options
   uint8_t LedMode : 4u;                                                         // led mode
   uint8_t BuzzVol : 4u;                                                         // buzzer volume
   uint8_t CapMode : 4u;                                                         // capture modes
   uint8_t spare : 4u;
}) COMGS_YiOptSelect_t;                                                         // Struct to hold the various options for YiCam configurations
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
   uint8_t laserStateOn : 1U;
   uint8_t laserStateOff : 1U;
   uint8_t getTemp : 1U;
   uint8_t getVolts : 1U;
   uint8_t getAlarm : 1U;                                                       // timed pulse to retrieve alarms
   uint8_t setAlarm : 1U;                                                       // request to set an alarm
   uint8_t spare : 2U;
} COMGS_SF40OptSelect_t;                                                        // Struct to hold the various options for SF40 liddar configurations
#else
COMGSPACKED(
typedef struct  {
   uint8_t laserStateOn : 1U;
   uint8_t laserStateOff : 1U;
   uint8_t getTemp : 1U;
   uint8_t getVolts : 1U;
   uint8_t getAlarm : 1U;                                                       // timed pulse to retrieve alarms
   uint8_t setAlarm : 1U;                                                       // request to set an alarm
   uint8_t spare : 2U;
}) COMGS_SF40OptSelect_t;                                                       // Struct to hold the various options for SF40 liddar configurations
#endif

#ifdef MACHINE_LEARN                                                            /* ============== machine learning ============ */
#if defined(D_FT900)
typedef struct COMGSPACKED {
   float64_t a;
   float64_t b;
   float64_t f1;
   float64_t f2;
   float64_t x1;
   float64_t x2;
   float64_t minX;
   float64_t minY;
} golden_section_t;                                                             // golden section iterative object
#else
COMGSPACKED(
typedef struct  {
   float64_t a;
   float64_t b;
   float64_t f1;
   float64_t f2;
   float64_t x1;
   float64_t x2;
   float64_t minX;
   float64_t minY;
}) golden_section_t;                                                            // golden section iterative object
#endif
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
   float64_t x;
   float64_t y;
   float64_t dx;
   float64_t dy;
   float64_t x1;
   float64_t x2;
} robot_nav_t;                                                             
#else
COMGSPACKED(
typedef struct  {
   float64_t x;
   float64_t y;
   float64_t dx;
   float64_t dy;
   float64_t x1;
   float64_t x2;
}) robot_nav_t;                                                            
#endif



#ifdef ROBOT_HELPER

#if defined(D_FT900)
typedef struct COMGSPACKED {
   float64_t x;
   float64_t y;
   float64_t dx;
   float64_t dy;
   float64_t x1;
   float64_t x2;
} robot_navig_t;                                                             
#else
COMGSPACKED(
typedef struct  {
   float64_t x;
   float64_t y;
   float64_t dx;
   float64_t dy;
   float64_t x1;
   float64_t x2;
}) robot_navig_t;                                                            
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
   float64_t x_1;
   float64_t y_1;
   float64_t x_2;
   float64_t y_2;
} homo_2d_t;                                                             
#else
COMGSPACKED(
typedef struct  {
   float64_t x_1;
   float64_t y_1;
   float64_t x_2;
   float64_t y_2;
}) homo_2d_t;                                                            
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
   float32_t sp_theta;
   float32_t sp_gamma;
} COMGS_Odrive_t;                                                               // Struct to hold odrive settings
#else
COMGSPACKED(
typedef struct  {
   float32_t sp_theta;
   float32_t sp_gamma;
}) COMGS_Odrive_t;                                                              // Struct to hold odrive settings
#endif

#if defined(D_FT900)
typedef struct COMGSPACKED {
 uint16_t kFrontLeft_val;
 uint16_t kFrontRight_val;
 uint16_t kRearLeft_val;
 uint16_t kRearRight_val;
} roboMtr_t;
#else
COMGSPACKED(
typedef struct  {
 uint16_t kFrontLeft_val;
 uint16_t kFrontRight_val;
 uint16_t kRearLeft_val;
 uint16_t kRearRight_val;
}) roboMtr_t;
#endif
#endif  /* endif robot */

#if defined(USE_MAV_PULSE_OBJECT)
#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint8_t relayNum;
  uint64_t TimeDurLast;
  int64_t TimeDiff;
  int64_t cycTim;
  uint8_t cycls;
  uint8_t cyclCnt;
  uint8_t activate;
} pulse_obj_t;                                                                  /* pulse object */
#else
COMGSPACKED(
typedef struct  {
  uint8_t relayNum;
  uint64_t TimeDurLast;
  int64_t TimeDiff;
  int64_t cycTim;
  uint8_t cycls;
  uint8_t cyclCnt;
  uint8_t activate;
}) pulse_obj_t;                                                                 /* pulse object */
#endif
#endif

#ifdef ENCODER_HELPER
typedef enum {
  ENCODER_GRAY_2_BIN = 1u,
  ENCODER_BIN_2_GRAY = 2u,
  ENCODER_COMPLETE = 3u,
  ENCODER_IN_USE = 4u
} mtrEncodingState_e;

#if defined(D_FT900)
typedef struct COMGSPACKED {
  unsigned char binary[16u];                                                    // 16 bits binary encoded
  unsigned char gray[16u];                                                      // 16 bits gray encoded
  unsigned char inUse[16u];                                                     // 16 bits to write to when encoded value is being read by another process
  uint16_t parIdx;                                                              // index for next state read inUse array
  uint16_t binIdx;                                                              // index for next state read binary array
  uint16_t grayIdx;                                                             // index for next state read gray array
  mtrEncodingState_e transState : 4u;                                           // state of transfer or read state machine
  uint8_t spare : 4u;
} gray_bin_t;
#else
COMGSPACKED(
typedef struct {
  unsigned char binary[16u];                                                    // 16 bits binary encoded
  unsigned char gray[16u];                                                      // 16 bits gray encoded
  unsigned char inUse[16u];                                                     // 16 bits to write to when encoded value is being read by another process
  uint16_t parIdx;                                                              // index for next state read inUse array
  uint16_t binIdx;                                                              // index for next state read binary array
  uint16_t grayIdx;                                                             // index for next state read gray array
  mtrEncodingState_e transState : 4u;                                           // state of transfer or read state machine
  uint8_t spare : 4u;
}) gray_bin_t;
#endif
#endif

/* ============== Stepper Motor =========================================== */
#define SinglePhaseSteppingRow1 0b1000                                          //1 cycle = 4 steps with lesser torque
#define SinglePhaseSteppingRow2 0b0100
#define SinglePhaseSteppingRow3 0b0010
#define SinglePhaseSteppingRow4 0b0001

#define DualPhaseSteppingRow1 0b1001                                            //1 cycle = 4 steps with higher torque and current
#define DualPhaseSteppingRow2 0b1100
#define DualPhaseSteppingRow3 0b0110
#define DualPhaseSteppingRow4 0b0011

#define HalfSteppingRow1 0b1001                                                 //1 cycle = 8 steps with lesser torque than full stepping
#define HalfSteppingRow2 0b1000
#define HalfSteppingRow3 0b1100
#define HalfSteppingRow4 0b0100
#define HalfSteppingRow5 0b0110
#define HalfSteppingRow6 0b0001
#define HalfSteppingRow7 0b0011
#define HalfSteppingRow8 0b0001

#define STEPPER_NO_OF_PINS 4u                                                   /* number of pins connected */
typedef enum {
  SingPhse,
  DualPhse,
  HalfPhse
} stepper_types_e;
typedef enum {
  StepFwd,
  StepRev,
  StepStop
} stepper_direction_e;
typedef enum {
  pinUsesPORTA,
  pinUsesPORTB,
  pinUsesPORTC,
  pinUsesPORTD,
  pinUsesPORTE,
  pinUsesPORTF,
  pinUsesPORTG,
} stepper_banks_e;
#if defined(D_FT900)
typedef struct COMGSPACKED {
   unsigned char name;
   uint8_t pins[STEPPER_NO_OF_PINS];
   uint8_t bank[STEPPER_NO_OF_PINS];
   uint8_t moving : 1u;
   uint8_t type : 3u;
   uint8_t direction : 4u;
   uint32_t phase;
   uint8_t stepsPerRev;
   uint8_t stepNum;
   uint16_t speed;
} StepperDriver_t;
#else
COMGSPACKED(
typedef struct {
   unsigned char name;
   uint8_t pins[STEPPER_NO_OF_PINS];
   uint8_t bank[STEPPER_NO_OF_PINS];
   uint8_t moving : 1u;
   uint8_t type : 3u;
   uint8_t direction : 4u;
   uint32_t phase;
   uint8_t stepsPerRev;
   uint8_t stepNum;
   uint16_t speed;
}) StepperDriver_t;
#endif
/* ================= radar obstacle avoidance =============================== */
#if defined(D_FT900)
typedef struct COMGSPACKED {
  uint8_t init : 1u;
  uint8_t spare : 7u;
  float64_t angle_min;
  float64_t angle_increment;
  float64_t range_min;
  float64_t ranges[];
  uint32_t lastLaserTime;
  uint32_t lastUpdTime;
  int32_t scanTimOut;                                                           // set to be scan timeout
  int32_t laserTimOut;
  float64_t safety_scaling_;
  float64_t last_sent_x_;
  float64_t last_sent_r_;
  float64_t desired_x_;                                                         // desired linear velocity twist.linear.x
  float64_t desired_r_;                                                         // desired angular velocity twist.angular.z (gyro)
  float64_t left_last_position_;
  float64_t right_last_position_;
} LaserScan_t;                                                                  // laser scan object
#else
COMGSPACKED(
typedef struct {
  uint8_t init : 1u;
  uint8_t spare : 7u;
  float64_t angle_min;
  float64_t angle_increment;
  float64_t range_min;
  float64_t ranges[];
  uint32_t lastLaserTime;
  uint32_t lastUpdTime;
  int32_t scanTimOut;                                                           // set to be scan timeout
  int32_t laserTimOut;
  float64_t safety_scaling_;
  float64_t last_sent_x_;
  float64_t last_sent_r_;
  float64_t desired_x_;                                                         // desired linear velocity twist.linear.x
  float64_t desired_r_;                                                         // desired angular velocity twist.angular.z (gyro)
  float64_t left_last_position_;
  float64_t right_last_position_;
}) LaserScan_t;                                                                 // laser scan object
#endif

const float64_t robot_width_ = 0.7f;                                            /* robot_safety_width */
const float64_t safety_scaling_distance_ = 0.01f;                               /* laser_safety_dist */

/* ================= joystick =============================================== */
#define JOY_CENTER_MIN 100u                                                     // worst case center calib value accepted

// define the alarm word for calibration
#define JCAL_SUCCESS 0u                                                         // calibration success
#define JCAL_CEN_FLT 1u<<0u                                                     // not at center
#define JCAL_XDIR_FLT 1u<<1u                                                    // xdir calibration fault
#define JCAL_YDIR_FLT 1u<<2u                                                    // ydir calibration fault
#define JCAL_ZDIR_FLT 1u<<3u                                                    // zdir calibration fault
#define JCAL_STEP_FLT 1u<<4u                                                    // not all step completed calibration fault
#define JCAL_OOR_FLT 1u<<5u                                                     // calibration out of range fault
#define JCAL_TIME_FLT 1u<<6u                                                    // calibration out of range fault

/* ===================== IndI Position Controller =========================== */

float32_t K_xi_x = 1.0f;                                                        // Position controller gains
float32_t K_xi_y = 1.0f;
float32_t K_xi_z = 1.0f;
float32_t K_dxi_x = 5.0f;                                                       // Velocity controller gains
float32_t K_dxi_y = 5.0f;
float32_t K_dxi_z = 5.0f;
float32_t K_thr = 0.00024730f;                                                  // Thrust mapping parameter

volatile float32_t posS_x, posS_y, posS_z;                                      // Current position
volatile float32_t velS_x, velS_y, velS_z;                                      // Current velocity
volatile float32_t gyr_p, gyr_q, gyr_r;                                         // Current rates
volatile Vectr positionRef;                                                     // Reference values
volatile Vectr velocityRef;

IndiOuterVariables indiOuter;

/* ===================== IndI Attitude Controller =========================== */
float32_t thrust_threshold = 300.0f;
float32_t bound_control_input = 32000.0f;
volatile attitude_t attitudeDesired;
volatile attitude_t rateDesired;
float32_t actuatorThrust;
float32_t roll_kp = 5.0f;
float32_t pitch_kp = 5.0f;
float32_t yaw_kp = 5.0f;
volatile float32_t attYawError;
static float32_t r_roll;
static float32_t r_pitch;
static float32_t r_yaw;
static float32_t accelz;
volatile Vectr refOuterINDI;                                                    // Reference values from outer loop INDI
IndiVariables indi;

/* ---------- Mellinger Controller ------------------------------------------ */
static float32_t g_vehicleMass = 0.032f;                                        // TODO: should be CF global for other modules
static float32_t massThrust = 132000.0f;

// XY Position PID
static float32_t kp_xy = 0.4f;                                                  // P
static float32_t kd_xy = 0.2f;                                                  // D
static float32_t ki_xy = 0.05f;                                                 // I
static float32_t i_range_xy = 2.0f;

// Z Position
static float32_t kp_z = 1.25f;                                                  // P
static float32_t kd_z = 0.4f;                                                   // D
static float32_t ki_z = 0.05f;                                                  // I
static float32_t i_range_z  = 0.4f;

// Attitude
static float32_t kR_xy = 70000.0f;                                              // P
static float32_t kw_xy = 20000.0f;                                              // D
static float32_t ki_m_xy = 0.0f;                                                // I
static float32_t i_range_m_xy = 1.0f;

// Yaw
static float32_t kR_z = 60000.0f;                                               // P
static float32_t kw_z = 12000.0f;                                               // D
static float32_t ki_m_z = 500.0f;                                               // I
static float32_t i_range_m_z  = 1500.0f;

// roll and pitch angular velocity
static float32_t kd_omega_rp = 200.0f;                                          // D

// Helper variables
static float32_t i_error_x = 0.0f;
static float32_t i_error_y = 0.0f;
static float32_t i_error_z = 0.0f;

static float32_t prev_omega_roll;
static float32_t prev_omega_pitch;
static float32_t prev_setpoint_omega_roll;
static float32_t prev_setpoint_omega_pitch;

static float32_t i_error_m_x = 0.0f;
static float32_t i_error_m_y = 0.0f;
static float32_t i_error_m_z = 0.0f;

// Logging variables
static Vectr z_axis_desired;

static float32_t cmd_thrust;
static float32_t cmd_roll;
static float32_t cmd_pitch;
static float32_t cmd_yaw;

static float32_t m_r_roll;
static float32_t m_r_pitch;
static float32_t m_r_yaw;
static float32_t m_accelz;

/*
implementation of piecewise polynomial trajectories
See Daniel Mellinger, Vijay Kumar: 
"Minimum snap trajectory generation and control for quadrotors". ICRA 2011: 2520-2525
*/
// precalculated factorials that we will need
static const int16_t facs[PP_SIZE] = { 1, 1, 2, 6, 24, 120, 720, 5040 };
volatile poly4d poly4d_tmp;

/*  -------------  quadrotor  --------------------------------------------------
 Copyright 2014-2019 Markus Giftthaler, Michael Neunert, Markus Stäuble.
Copyright 2014-2018 ETH Zurich.
All rights reserved.
*/
// mass / inertia
static const float64_t mQ = 0.546f;                                             // mass of quadcopter [ kg ]
static const float64_t arm_len = 0.175f;                                        // length of quadcopter arm [ m ]
static const dVectr4d_t kFs;                                                    //= {kF, kF, kF, kF};
static const dVectr4d_t kMs;                                                    //= {kM, kM, kM, kM};

/* ---------------- dcm matrix ---------------------------------------------- */
// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 10.3f
#define Ki_YAW 0.002f
#define DCM_GYRO_GAIN 0.06957f                                                  // Same gain on all axes gain is for ITG-3200 on Navstik
#define DCM_GYRO_SCALED_RAD(x) (x * DEGREE_TO_RADIAN(DCM_GYRO_GAIN))            // Calculate the scaled gyro readings in radians per second
float64_t DCM_Matrix[3u][3u] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float64_t Accel_Vector[3u]= {0, 0, 0};                                          // Store the acceleration in a vector
float64_t Gyro_Vector[3u]= {0, 0, 0};                                           // Store the gyros turn rate in a vector
float64_t Omega_Vector[3u]= {0, 0, 0};                                          // Corrected Gyro_Vector data
float64_t Omega_P[3u]= {0, 0, 0};                                               // Omega Proportional correction
float64_t Omega_I[3u]= {0, 0, 0};                                               // Omega Integrator
float64_t Omega[3u]= {0, 0, 0};
float64_t errorRollPitch[3u] = {0, 0, 0};
float64_t errorYaw[3u] = {0, 0, 0};
float64_t Update_Matrix[3u][3u] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float64_t Temporary_Matrix[3u][3u] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

/* ====== data for other AHRS compensation ===================== */
/*
%   JUSTA, Josef; ŠMÍDL, Václav; HAMÁÈEK, Aleš. Fast AHRS Filter for Accelerometer, Magnetometer, 
%   and Gyroscope Combination with Separated Sensor Corrections. Sensors, 2020, 20.14: 3824.
%
%   Date          Author          Notes
%   30/5/2020     Josef Justa     Initial release
*/

typedef enum {
 _PURE_JUSTA = 0,
 _PURE_FAST_JUSTA_LINEAR = 1,
 _PURE_FAST_JUSTA = 2,
 _PURE_FAST_JUSTA_CONST_CORR = 3 
} justa_type_e;

#if defined(D_FT900)
typedef struct COMGSPACKED {
        float64_t SamplePeriod;
        quat Quaternion;                                                        // output quaternion describing the Earth relative to the sensor
        float64_t Beta;                                                               // algorithm gain
        quat test;
        quat test2;
        float64_t gain;
        float64_t wAcc;
        float64_t wMag;
        float64_t mr_z;
        justa_type_e option;
} JustaAHRSPure_t;
#else
COMGSPACKED(
typedef struct {
        float64_t SamplePeriod;
        quat Quaternion;                                                        // output quaternion describing the Earth relative to the sensor
        float64_t Beta;                                                               // algorithm gain
        quat test;
        quat test2;
        float64_t gain;
        float64_t wAcc;
        float64_t wMag;
        float64_t mr_z;
        justa_type_e option;
}) JustaAHRSPure_t;
#endif

volatile JustaAHRSPure_t JustaAHRSPureObj = { 1.0f/256.0f, {1.0f,0.0f,0.0f,0.0f}, 1.0f, {0.0f,0.0f,0.0f,0.0f}, {0.0f,0.0f,0.0f,0.0f}, 0.0528152f, 0.00248f, 1.35e-04f };

#if defined(D_FT900)
typedef struct COMGSPACKED {
        float64_t SamplePeriod;
        quat Quaternion;                                                        // output quaternion describing the Earth relative to the sensor
        Vectr q_err;                                                               
        quat test;
        quat test2;
        float64_t rg;
        float64_t ra;
        float64_t rm;
        float64_t alf;
        float64_t bet;
        float64_t gam;
        float64_t kAlf;
        float64_t kBet;
        uint8_t iter;
} SuhSooYoung_t;
#else
COMGSPACKED(
typedef struct {
        float64_t SamplePeriod;
        quat Quaternion;                                                        // output quaternion describing the Earth relative to the sensor
        Vectr q_err;                                                               
        quat test;
        quat test2;
        float64_t rg;
        float64_t ra;
        float64_t rm;
        float64_t alf;
        float64_t bet;
        float64_t gam;
        float64_t kAlf;
        float64_t kBet;
        uint8_t iter;
}) SuhSooYoung_t;
#endif

volatile SuhSooYoung_t YoungSooSuhAHRSPureObj = { 1.0f/256.0f, {1.0f,0.0f,0.0f,0.0f}, {0.0f,0.0f,0.0f}, {0.0f,0.0f,0.0f,0.0f}, {0.0f,0.0f,0.0f,0.0f}, 0.317f, 0.0004156f, 0.00057f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

#if defined(D_FT900)
typedef struct COMGSPACKED {
        float64_t SamplePeriod;
        quat q;                                                                 // output quaternion 
        dVectr mag_r;
        quat q_;
        float64_t jaCob[4u][6u];                                                // output jacobean
} JinWu_t;
#else
COMGSPACKED(
typedef struct {
        float64_t SamplePeriod;
        quat q;                                                                 // output quaternion 
        dVectr mag_r;
        quat q_;
        float64_t jaCob[4u][6u];                                                // output jacobean
}) JinWu_t;
#endif

/*#######################################################################################*/
// information ported from the following project
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + www.MikroKopter.com
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + der Fa. HiSystems GmbH, Flachsmeerstrasse 2, 26802 Moormerland - nachfolgend Lizenzgeber genannt -
// +  Hinweis: Informationen über erweiterte Nutzungsrechte (wie z.B. Nutzung für nicht-private Zwecke) sind auf Anfrage per Email an info(@)hisystems.de verfügbar.
#if defined(D_FT900)                                                            
typedef struct COMGSPACKED {
  uint8_t Hoehe_HoverBand;
  uint8_t Gyro_Stability;
  uint8_t DynamicStability;
} mk_EEprom_param_t;
#else
COMGSPACKED(
typedef struct {
  uint8_t Hoehe_HoverBand;
  uint8_t Gyro_Stability;
  uint8_t DynamicStability;
}) mk_EEprom_param_t;                                                   
#endif

#if defined(D_FT900)                                                            
typedef struct COMGSPACKED {
   uint8_t flying : 1u;
   uint8_t esd : 1u;
   uint8_t init : 1u;
   uint8_t StartTrigger : 2u;
   uint8_t thrustHtLim : 1u;
   uint8_t AltitudeSetpointTrimming : 1u;
   uint8_t spare : 1u;
   uint32_t HoverGasFilter;
   int32_t time2Now;
   uint32_t tickRef;
   int16_t HoverGas;
   int16_t HoverGasMin;
   int16_t HoverGasMax;
   int64_t GasReduction;
} mk_Hover_t;
#else
COMGSPACKED(
typedef struct {
   uint8_t flying : 1u;
   uint8_t esd : 1u;
   uint8_t init : 1u;
   uint8_t StartTrigger : 2u;
   uint8_t thrustHtLim : 1u;
   uint8_t AltitudeSetpointTrimming : 1u;
   uint8_t spare : 1u;
   uint32_t HoverGasFilter;
   int32_t time2Now;
   uint32_t tickRef;
   int16_t HoverGas;
   int16_t HoverGasMin;
   int16_t HoverGasMax;
   int64_t GasReduction;
}) mk_Hover_t;                                                   
#endif

#if defined(D_FT900)                                                            
typedef struct COMGSPACKED {
  uint32_t IntegralNickMalFaktor;
  uint32_t MesswertNick;
  uint32_t StickNick;
  uint32_t Ki;
  uint32_t SummeNick;
  uint32_t pd_ergebnis_nick;
  uint8_t IntegralFaktor : 1u;
  uint8_t spare : 7u;
} mk_PitchAxisCon_t;
#else
COMGSPACKED(
typedef struct {
  uint32_t IntegralNickMalFaktor;
  uint32_t MesswertNick;
  uint32_t StickNick;
  uint32_t Ki;
  uint32_t SummeNick;
  uint32_t pd_ergebnis_nick;
  uint8_t IntegralFaktor : 1u;
  uint8_t spare : 7u;
}) mk_PitchAxisCon_t;                                                   
#endif
//
//
//  --------------------  Function Declarations --------------------------------
//
//
/* ============ miscelaneous calculations ======================== */
float32_t invSqrt(float32_t x);
float32_t frand_func();
void Number2String(float32_t x, unsigned char *str, unsigned char precision);
void moving_average( COMGS_gyro_acc_data_t *accStruct );
float64_t * plane_normalize(float64_t *dPln[5u]);
float64_t sphere_calcRadius(float64_t *sph[5u]);
quat * sphere_vector_normalizeVec(quat *dSph);
float64_t * sphere_vector_normalize(float64_t *dSph[5u]);
/* ============ miscelaneous alarm time converwsions ======================== */
void Process_alarm_ack( COMGS_error_alarm *almWord, AlarmStack_t *alarmStack, const AlarmDefinitionObject_t *almDesc, AlarmLineObject_t *almLineObj );
void Update_Alm_Line( const AlarmStack_t *alarmStack, const AlarmDefinitionObject_t *almDesc, AlarmLineObject_t *almLineObj );
void pushAlmToStack( AlarmStack_t *alarmStack, const AlarmDefinitionObject_t *almDesc, uint8_t positionInobj );
void popAlmOffStack( AlarmStack_t *alarmStack, const AlarmDefinitionObject_t *almDesc, uint8_t indexInAlmDefObj );
void Process_Motor_State( COMGS_Motor_t *mtrBlk );
int16_t is_valid_ip(unsigned char *ip_str);
void compute_time(uint32_t time, COMGS_DateTime_t *hmiTm);
void compute_secs(XS_Hmi_Slider_t *CalcTime, const COMGS_DateTime_t *hmiTm);
void compute_date(XS_Hmi_Slider_t *CalcTime, const COMGS_DateTime_t *hmiTm);
uint8_t check_datetime(COMGS_DateTime_t *hmiTm);
float32_t fastA2F(const char *p);
int16_t fastA2I(const char *s);
int16_t a2d(char ch);
uint32_t fastA2UL(const char *p);
float32_t fastA2F(const char *p);
uint32_t bswap_32(uint32_t num2ByteSwap);
uint16_t bswap_16(uint16_t x);
uint32_t dateTimeToRtcTime(COMGS_DateTime_t *dt);
float32_t farenhFromcelsius( float32_t celsius );
int16_t is_valid_ip(unsigned char *ip_str);
int16_t valid_digit(unsigned char *ip_str);

/* ============ miscelaneous angle functions ================================ */
void calculate_velocity_position( COMGS_gyro_acc_data_t *accStruct );
void calculate_theta( COMGS_gyro_acc_data_t *accStruct );
int16_t anglefromMag( int16_t magX, int16_t magY );
uint8_t NormaliseAccMag( float32_t ax, float32_t ay, float32_t az );
void RefDirEarthMag(float32_t mx, float32_t my, float32_t mz, COMGS_quarternion *quartCalculated, COMGS_magcorr_t *magCalculated);
void AHRS2quartAGM(COMGS_quarternion *quartCalculated, const COMGS_magcorr_t *magCalculated, float32_t ax, float32_t ay, float32_t az, float32_t gx, float32_t gy, float32_t gz, float32_t mx, float32_t my, float32_t mz, COMGS_QControl_t *QContObj );
void AHRS2quartAG(COMGS_quarternion *quartCalculated, float32_t ax, float32_t ay, float32_t az, float32_t gx, float32_t gy, float32_t gz, COMGS_magcorr_t *magCalculated, COMGS_QControl_t *QContObj );
void AxisAngle_to_Quarternion( COMGS_quarternion *quartCalculated, const COMGS_axis_angle *axisAngle );
void Quarternion_to_AxisAngle( const COMGS_quarternion *quartCalculated, COMGS_axis_angle *axisAngle );
void Quarternion_to_euler( const COMGS_quarternion *quartCalculated, COMGS_euler_angle *eulerCoord );
void euler_to_Quarternion(COMGS_quarternion *quartCalculated, const COMGS_euler_angle *eulerCoord);
float64_t Euclid_dist2point(float64_t x1, float64_t y1, float64_t x2, float64_t y2);
float64_t calcAngle( float64_t magY, float64_t magX );
void getXYfromAziDist( float64_t azi, float64_t dist, COMGS_coordObj_t *coOrd );
float64_t calcAzi( float64_t magY, float64_t magX );
float64_t calcDist( float64_t magY, float64_t magX );
int16_t JoyStick2Speed( int16_t nJoyX, uint8_t choice );
float64_t calcHeadRoll( quat *q );
float64_t calcHeadPitch( quat *q );
float64_t calcHeadYaw( quat *q );
float64_t calcHeadingFromMag(float64_t mx, float64_t my, float64_t mz, quat *q);
uint16_t MapSpeedToPpm(float64_t speed);
uint16_t MapAngleToPpm(float64_t angle);
Vectr swapLeftRightCoord( Vectr v );
float64_t quaternion_get_yaw(const quat q);
/* ==== path trajectory smoothing ======= */
#define LEN_OF_PATH 10u                                                         /* number of frames in trajectory */
float64_t v2Float( Vectr a );
Vectr * PathSmoothing( Vectr path[LEN_OF_PATH] );
/* ============ Vector math helpers for float arrays ================ */
float64_t Vector_Dot_Product(const float64_t v1[3u], const float64_t v2[3u]);
void Vector_Cross_Product(float64_t out[3u], const float64_t v1[3u], const float64_t v2[3u]);
void Vector_Scale(float64_t out[3u], const float64_t v[3u], float64_t scale);
void Vector_Add(float64_t out[3u], const float64_t v1[3u], const float64_t v2[3u]);
void Matrix_Multiply(const float64_t a[3u][3u], const float64_t b[3u][3u], float64_t out[3u][3u]);
void Matrix_Vector_Multiply(const float64_t a[3u][3u], const float64_t b[3u], float64_t *out[3u]);
/* ============ DCM implementation ================================== */
void init_rotation_matrix(float64_t m[3][3], float64_t yaw, float64_t pitch, float64_t roll);
void DCM_Compass_Heading( navstik_object_t *nav, dcm_object_t *dcm );
void DCM_reset_sensor_fusion( dcm_object_t *dcm, navstik_object_t *nav, navstickUseCalib_e *mode, char* inpStr );
void DCM_Matrix_update(navstik_object_t *nav, dcm_object_t *dcm);
void DCM_Normalize(void);
void DCM_Euler_angles(dcm_object_t *dcm);
void DCM_Drift_correction( const dcm_object_t *dcm );
void DCM_Algo_go( char* inpStr, navstik_object_t *nav, dcm_object_t *dcm, navstickUseCalib_e *mode );
/* ============ air speed compensation ============================== */
void calc3headingTriangular( wind_compens_t *triangle );
void calcBoxPattern( wind_compens_t *boxPatt );
void bluffBodyDrag( drag_coef_t *dragData, float32_t flowRelative2Fluid, float32_t mass, float32_t frontArea, float32_t sideArea, float32_t a_x);
#if defined(SBGC_GIMBAL_JOY)
void JoyStick2BearSteer( int16_t nJoyX, int16_t nJoyY, int16_t nMotMixL, int16_t nMotMixR);
void JoyStick2DiffSteer( int16_t nJoyX, int16_t nJoyY, int16_t nMotMixL, int16_t nMotMixR);
uint8_t CalibrateCenter( COMGS_joy_calib_t *calibObj );
uint8_t CalibrateSpan( COMGS_joy_calib_t *calibObj );
#endif
#ifdef GPS_INCLUDED                                                             // Quectel GPS version wanted
void ReadGPSData( GPS_Info_t *gpsInfo );
void Calc_Cursor_LL(signed int lat, signed int lon, char *lt_Str, char *lng_Str, unsigned int color);
#endif
#ifdef ROBOT_HELPER                                                             // Robot leg library wanted
uint8_t XorShort(int16_t val);
void readJoyStickInput( uint16_t xIn, uint16_t yIn, uint16_t rotation, roboMtr_t *MotorType );
/* =============== NATE711 Dogo Project ===================================== */
void hop(const GaitParams params, COMGS_Odrive_t *od1, COMGS_Odrive_t *od2, COMGS_Odrive_t *od3, COMGS_Odrive_t *od4);
void TransitionToDance();
void gait(const GaitParams params, float32_t leg0_offset, float32_t leg1_offset, float32_t leg2_offset, float32_t leg3_offset,const LegGain *gains, COMGS_Odrive_t *od1, COMGS_Odrive_t *od2, COMGS_Odrive_t *od3, COMGS_Odrive_t *od4);
void CoupledMoveLeg( float32_t t,const GaitParams params, float32_t gait_offset, float32_t leg_direction,const LegGain *gains, float32_t theta, float32_t gamma);
void UpdateStateGaitParams(const RobStates curr_state);
uint8_t IsValidLegGain(const LegGain gains);
uint8_t IsValidGaitParams(const GaitParams params);
void SinTrajectory(float32_t t, const GaitParams params, float32_t gaitOffset, float32_t x, float32_t y);
void CartesianToThetaGamma(float32_t x, float32_t y, float32_t leg_direction, float32_t theta, float32_t gamma);
void CartesianToLegParams(float32_t x, float32_t y, float32_t leg_direction, float32_t L, float32_t theta);
void LegParamsToCartesian(float32_t L, float32_t theta, float32_t leg_direction, float32_t x, float32_t y);
void GetGamma(float32_t L, float32_t theta, float32_t gamma);
void CommandAllLegs(float32_t theta, float32_t gamma,const LegGain *gains, COMGS_Odrive_t *od1, COMGS_Odrive_t *od2, COMGS_Odrive_t *od3, COMGS_Odrive_t *od4);
void SetPositionUsingGains( const LegGain *gains, char* sndBuf );
void SetCoupledPosition(float32_t sp_theta, float32_t sp_gamma,const LegGain *gains, char *sndBuf);
int16_t ParseDualPosition(char* msg, int16_t len, float32_t  *th, float32_t *ga);
void SetThetaGamma(float32_t theta, float32_t gamma);
int16_t ParseMtrStateReply(char* msg);
void precomputed_jacobian( float32_t joints[4u], float32_t *jacobian[3u][4u] );
void manual_forward_kinematics( float32_t joints[4u], float32_t *array[3u] );
void automatic_forward_kinematics(float32_t joints[4u], float32_t t10[4u][4u], float32_t t21[4u][4u], float32_t t32[4u][4u], float32_t t43[4u][4u]  );
void transform_matrix(float32_t alpha,float32_t a, float32_t d, float32_t theta, float32_t array[4u][4u]);
/* ======= Kinova Robotics ============================================ */
uint8_t areValuesClose(float32_t first, float32_t second, float32_t tolerance);
uint8_t valid_kinovaRobotType(const unsigned char *kinova_RobotType);
uint8_t KinovaAngles_isCloseToOther(const kinovaAngles_t *angle,  float32_t tolerance,const Vectr *posn, const kinovaAngles_t *actuator);
uint8_t FingerAngles_isCloseToOther(const Vectr *other, float32_t const tolerance, const Vectr *Finger);
uint8_t KinovaPose_isCloseToOther(const geoPose_t *other, float32_t position_tolerance, float32_t EulerAngle_tolerance,const Vectr *posn, const Vectr *theta);
/* ====== helper functions ===================================== */
float64_t doHimmelBlau( robot_navig_t *rob );
Vectr CalcJacobiOfHimmel( robot_navig_t *rob );
void SteepestDescentMethod( Vectr param );
homo_2d_t HomogeneousTransformation2D(homo_2d_t *in, homo_2d_t *base, uint8_t mode);
Vectr doSLAMControl( uint64_t time );
void doSLAMAngle( Vectr *angle );
void doASTARAngle( Vectr *angle, uint32_t tick );
quat MotionModelASTAR( uint32_t tick );
void SnakeSlam_ComputeThreeMaxima(int16_t* histo, const int16_t histoLen, int16_t *ind1, int16_t *ind2, int16_t *ind3);
#endif  /* endif robot */
/* ====== machine learning functions ===================================== */
#if defined(MACHINE_LEARN)
float64_t golden_f( float64_t x );
void goldenInit( golden_section_t *gold );
void goldenIterate( golden_section_t *gold );
#endif
/* ===================== vector math library =============================== */
void quaternionNormalize(quat *vAcc);
quat mkquat(float32_t x, float32_t y, float32_t z, float32_t w);
mat33 quat2rotmat(quat q);
Vectr quat2rpy(quat q);
quat rpy2quat(Vectr rpy);
float32_t quat2angle(quat q);
quat qeye(void);
Vectr vscl(float32_t s, Vectr v);
Vectr vdiv(Vectr v, float32_t s);
float32_t vdot(Vectr a, Vectr b);
float32_t vmag2(Vectr v);
float32_t vmag(Vectr v);
float32_t vdist2(Vectr a, Vectr b);
Vectr vnormalize(Vectr v);
Vectr mvmul(mat33 a, Vectr v);
mat33 mcolumns(Vectr a, Vectr b, Vectr c);
Vectr mcolumn(mat33 m, int16_t col);
Vectr vmin(Vectr a, Vectr b);
Vectr vmax(Vectr a, Vectr b);
float32_t quat2angle(quat q);
quat qeye(void);
float32_t vmagfast(Vectr v);
quat qaxisangle(Vectr axis, float32_t angle);
float32_t * vminArr(Vectr a, Vectr b);
float32_t math_copysign(float32_t val, float32_t sign);
quat mat2quat(mat33 m);
Vectr vadd3(Vectr a, Vectr b, Vectr c);
Vectr qvrot(quat q, Vectr v);
/* == compensation ========================================================== */
void applySensorCorrectionUsingMAG(quat *vError, const mat33 qpAttitude, const quat mag, quat *vMagAverage );
void applySensorCorrectionUsingCOG(quat *vError,const mat33 qpAttitude, const GPS_Info_t *gps, Vectr *attitude,const bool fixedWing, float32_t GPS_distanceToHome );
void applyAccError(const quat *vAcc, quat *vError, const mat33 qpAttitude);
#ifdef PID_LOOPS                                                                // PID loop library wanted
uint8_t tunePID( pidBlock_t *pidObj, pidTuner_t *pidTuner );
void DcMotorPIDloop(pidBlock_t *pidObj, uint16_t pwm_period, uint16_t channel);
void PWMSetDuty( uint16_t pid_out, uint16_t pwm_period, uint16_t channel );

uint8_t computePID( pidBlock_t *pidObj );
void kalman_filter( COMGS_kalman *kalmanData );
void DcMotorComplexPIDloop(pidBlock_t *speedPidObj, pidBlock_t *posPidObj, uint16_t pwm_period, uint16_t channel);
/* =========== Madgwick and Mahony AHRS compensation and PID ========== */
void IMU_filterUpdate(float32_t w_x, float32_t w_y, float32_t w_z, float32_t a_x, float32_t a_y, float32_t a_z, COMGS_quarternion *quartCalculated );
void MARG_AHRS_filterUpdate(float32_t w_x, float32_t w_y, float32_t w_z, float32_t a_x, float32_t a_y, float32_t a_z, float32_t m_x, float32_t m_y, float32_t m_z, COMGS_quarternion *quartCalculated);
void MadgwickAHRSupdate(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, float32_t mx, float32_t my, float32_t mz, COMGS_quarternion *quartCalculated);
void MadgwickAHRSupdateIMU(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, COMGS_quarternion *quartCalculated);
void MahonyAHRSupdate(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, float32_t mx, float32_t my, float32_t mz, COMGS_quarternion *quartCalculated);
void MahonyAHRSupdateIMU(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, COMGS_quarternion *quartCalculated);
/* =========== Sensor Fusion 6 Position Estimator ========== */
void quatdecompress(uint32_t comp, float32_t *q[4u]);
uint32_t quatcompress(float32_t const q[4u]);
void estimatedGravityDirection(float32_t* gx, float32_t* gy, float32_t* gz);
float32_t sensfusion6GetAccZ(const float32_t ax, const float32_t ay, const float32_t az);
float32_t sensfusion6GetAccZWithoutGravity(const float32_t ax, const float32_t ay, const float32_t az);
float32_t sensfusion6GetInvThrustCompensationForTilt();
void sensfusion6GetEulerRPY(float32_t* roll, float32_t* pitch, float32_t* yaw);
void sensfusion6UpdateQ(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, COMGS_quarternion *quartCalculated);
void initPosEstselfState( selfState_s *state );
void positionUpdateVelocity(float32_t accWZ, float32_t dt, selfState_s* state);
float32_t deadband(float32_t value, const float32_t threshold);
void positionEstimate(state_t* estimate, const sensorData_t* sensorData, const tofMeasurement_t* tofMeasurement, float32_t dt, selfState_s* state);
void readLwNxLaser( tofMeasurement_t *tof, lwDistanceResp_t *laser );
void estimatorComplementary(state_t *state, sensorData_t *sensorData, secondStageTiming_t *tickLast, COMGS_quarternion *quartCalculated, selfState_s* SelfState);
/* =========== INDI Position Controller ========== */
void initIndiOuterVariables( IndiOuterVariables* indiOuter );
int8_t position_indi_init_filters(IndyTiming_t *timer);
void filter_ddxi(Butterworth2LowPass *filter, Vectr *old_values, Vectr *new_values);
void filter_ang(Butterworth2LowPass *filter, Angles *old_values, Angles *new_values);
void filter_thrust(Butterworth2LowPass *filter, float32_t *old_thrust, float32_t *new_thrust);
void m_ob(const Angles *att, float32_t matrix[3u][3u]);
void positionControllerINDI(const sensorData_t *sensors, setpoint_t *setpoint, const state_t *state, Vectr *refOuterINDI);
/* =========== INDI Attitude Controller ========== */
void initIndiInnerVariables( IndiVariables* indi );
void float_rates_zero(FloatRates *fr);
int8_t indi_init_filters(IndyTiming_t *timer);
void filter_pqr(Butterworth2LowPass *filter, FloatRates *new_values);
int8_t finite_difference_from_filter(float32_t *output, Butterworth2LowPass *filter, IndyTiming_t *timer);
int8_t controllerINDIInit(IndyTiming_t *timer);
float32_t capAngle(float32_t angle);
void controllerINDI(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, IndyTiming_t *timer);
/* =========== Mellinger Controller ============ */
void controllerMellingerReset(void);
void controllerMellinger(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, MellTiming_t *tick);
/* ============= crazy swarm ============= */
float32_t piecewise_duration(piecewise_traj const *pp);
void polylinear(float32_t *p[PP_SIZE], float32_t duration, float32_t x0, float32_t x1);
void polystretchtime(float32_t *p[PP_SIZE], float32_t s);
void polyreflect(float32_t *p[PP_SIZE]);
float32_t polyval(const float32_t *p[PP_SIZE], float32_t t);
void polyder(float32_t *p[PP_SIZE]);
void poly5(float32_t *poly[PP_SIZE], float32_t T,float32_t x0, float32_t dx0, float32_t ddx0, float32_t xf, float32_t dxf, float32_t ddxf);
void poly7_nojerk(float32_t *poly[PP_SIZE], float32_t T, float32_t x0, float32_t dx0, float32_t ddx0, float32_t xf, float32_t dxf, float32_t ddxf);
void polybezier(float32_t *p[PP_SIZE], float32_t duration, float32_t *x, int16_t dim);
poly4d poly4d_zero(float32_t duration);
poly4d poly4d_linear(float32_t duration, Vectr p0, Vectr p1, float32_t yaw0, float32_t yaw1);
void polyscale(float32_t *p[PP_SIZE], float32_t s);
void poly4d_scale(poly4d *p, float32_t x, float32_t y, float32_t z, float32_t yaw);
void poly4d_shift(poly4d *p, float32_t x, float32_t y, float32_t z, float32_t yaw);
void poly4d_stretchtime(poly4d *p, float32_t s);
void polyder4d(poly4d *p);
Vectr polyval_xyz(poly4d const *p, float32_t t);
float32_t polyval_yaw(poly4d const *p, float32_t t);
float32_t vnorm1(Vectr *v);
float32_t poly4d_max_accel_approx(poly4d const *p);
Vectr vrepeat(float32_t x);
Vectr vzero(void);
traj_eval traj_eval_zero();
uint8_t visnan(Vectr v);
traj_eval traj_eval_invalid();
uint8_t is_traj_eval_valid(traj_eval const *ev);
Vectr vprojectunit(Vectr a, Vectr b_unit);
Vectr vorthunit(Vectr a, Vectr b_unit);
traj_eval poly4d_eval(poly4d const *p, float32_t t);
traj_eval piecewise_eval( piecewise_traj const *traj, float32_t t);
traj_eval piecewise_eval_reversed( piecewise_traj const *traj, float32_t t);
void piecewise_plan_5th_order(piecewise_traj *pp, float32_t duration, Vectr p0, float32_t y0, Vectr v0, float32_t dy0, Vectr a0, Vectr p1, float32_t y1, Vectr v1, float32_t dy1, Vectr a1);
void piecewise_plan_7th_order_no_jerk(piecewise_traj *pp, float32_t duration, Vectr p0, float32_t y0, Vectr v0, float32_t dy0, Vectr a0, Vectr p1, float32_t y1, Vectr v1, float32_t dy1, Vectr a1);
/* ====== end poly priecewise projectory ====== */
/* +++++ mikro Kopter controls ++++++++ */
void getCosAttitude( uint32_t IntegralNick, uint32_t IntegralRoll, float64_t *CosAttitude );
void mikroKopter_HoverGasEstimation( uint32_t currentThrust, int16_t CosAttitude, int16_t VarioMeter, uint32_t HoehenWertF, uint32_t SollHoehe, const mk_EEprom_param_t EE_Parameter, mk_Hover_t *status  );
void mikroKopterPitchAxisContoller( const mk_EEprom_param_t EE_Parameter, mk_PitchAxisCon_t *pitch, uint32_t GasMischanteil, uint32_t GierMischanteil  );
void mikRoKopterDpart3GpsZ( int16_t Parameter_Hoehe_GPS_Z, int16_t FromNaviCtrl_Value_GpsZ, mk_Hover_t *hov, int16_t *HCGas, int16_t HeightDeviation );
void mikroKopterGierAnteilRegler( uint32_t MesswertGier, uint32_t sollGier, uint32_t GasMischanteil, uint32_t *GierMischanteil );
/* ====== co-ordinate and robotics functions ====== */
float32_t Kepler( float32_t mk, float32_t e );
Vectr Lla2Ecef( GPS_Info_t *gps );
mat33 mzero(void);
mat33 RotEcef2Ned( GPS_Info_t *gps );
Vectr getNedfromECEF( mat33 Re2n, Vectr ecef );
mat33 mtranspose(mat33 m);
Vectr getECEFfromNed( mat33 Re2n, Vectr ned );
GPS_Info_t Xyz2Lla( Vectr eCef );
Vectr dllh2denu( Vectr *llh0, Vectr *llh );
void dllh2denu2( Vectr *llh0, Vectr *llh[], Vectr *v[] );
/* ====== Optical Flow clover pose control ============================== */
Vectr vtrans(Vectr q);
quat qtrans(quat q);
quat qscl(quat q, float32_t r);
quat qmul(quat q, quat r);
quat qsub(quat q, quat r);
quat qinv(quat q);
void alignObjPointsToCenter(Vectr *obj_points[], float32_t *center_x, float32_t *center_y, float32_t *center_z);
void fillPose(geoPose_t *pose, const Vectr *rvec, const Vectr *tvec);
optical_flowObj_t * doOpticalFlo( optical_flowObj_t *flow, float32_t roi_rad_, mat33 camera_matrix_, mat33 dist_coeffs);
optical_flowObj_t * calcFlowGyro(const uint8_t frame_id, optical_flowObj_t *flow);
/* ===== kinova angle ============================== */
void getEulerXYZ(mat33 *Rot_matrix, float32_t *tx, float32_t *ty, float32_t *tz);
void getQuatXYZ(quat *q, float32_t *tx, float32_t *ty, float32_t *tz);
mat33 EulerXYZ2Matrix3x3(float32_t tx, float32_t ty, float32_t tz);
quat EulerXYZ2Quaternion(float32_t tx, float32_t ty, float32_t tz);
/* ======== object avoidance laser visual odometry ==================== */
void calcSafetyScalefromLaser( LaserScan_t *scan );
void setPoseAnTwist( LaserScan_t *odomObj, odom_t *odomPoseTwist, movingObjectStat_t *movObj );
void doVeloAccLimit( LaserScan_t *odomObj );
/* ===== stabilzer ==================== */
void defaultControllerExpo( quat *mExpo );
void Controller_setRoll( float32_t value, quat *mExpo, control_t *mRPY );
void Controller_setPitch( float32_t value, quat *mExpo, control_t *mRPY );
void Controller_setYaw( float32_t value, quat *mExpo, control_t *mRPY );
void Controller_setThrust( float32_t value, quat *mExpo, control_t *mRPY, CtrlStabilizer_t *thrustCtrl );
void Controller_InitStabilizer( Vectr *mHorizonMultiplier, float32_t *mRateFactor, Vectr *mHorizonMaxRate );
/* ===== acurate position helpers (satelite) ========================= */
void get_alt_az_coordinates(float64_t Ha, float64_t Dec, float64_t Lat, float64_t *Alt1, float64_t *Az1);
float64_t estimate_geocentric_elevation(float64_t Lat, float64_t El);
float64_t estimate_field_rotation_rate(float64_t Alt, float64_t Az, float64_t Lat);
float64_t estimate_field_rotation(float64_t HA, float64_t rate);
uint8_t Dome_Intersection(const dVectr *p1,const Vectr *dp, float64_t r, float64_t *mu1, float64_t *mu2);
/* ================ pose =======================*/
void generatePose(geoPose_t *pose, float32_t setCoord, float32_t setpointZ);
/* ================ ETH Zurich =============== */
void A_quadrotor(state_control_vector_t *x);
void B_quadrotor(state_control_vector_t *x);
void C_quadrotor(state_control_vector_t *x);
void quadrotor_ode(state_control_vector_t *x);
#endif
#ifdef GPS_POS_SPRAYER
int8_t doesPosFitSquare( const geoCoord_Square_t * geoJson, const GPS_Info_t * actPos, float32_t sptLookUp[NO_SPRAY_OF_AREAS], pidBlock_t *pidObj  );
void guiInterfacePID( pidBlock_t *pidObj, interfacePIDObj_t *hmiOrPush );
#endif
#if defined(USE_MAV_PULSE_OBJECT)
void relayPulseAction( uint8_t relayNum, uint64_t TimeDurLast, int64_t TimeDiff, float32_t cycTim, float32_t cycls, uint8_t cyclCnt );
#endif
#ifdef ENCODER_HELPER
char int_to_ascii(uint8_t nibble);
void collectGray(gray_bin_t *word, uint16_t bitNo);
uint16_t graytoBinary(gray_bin_t *word);
uint16_t binarytoGray(gray_bin_t *word);
#endif
#if defined(STEPPER_MOTOR)
void setPhaseStepper( StepperDriver_t *motor );
void oneStepMotor( StepperDriver_t *smtr );
#endif
float64_t pointDistanceSquare(const Vectr p1, const Vectr p2);
float64_t pointDistance(const Vectr p1, const Vectr p2);
Vectr E3dPoint(const GPS_Info_t *gps);
float64_t E3dPoint_EllipsoidDistWGS84(const float64_t latitude);
uint8_t EDate_isLeapYear(uint8_t year);
uint8_t EDate_dayCountFromMonth(uint8_t month, uint8_t year);
void EDate_increaseDay(uint8_t days, uint8_t *day, uint8_t *month, uint8_t *year);
#if defined(NAVILOCK_USED)
int16_t getDeltaTimeSec(const nvEDate_t *start_date, const nvETime_t *start_time, const nvEDate_t *end_date, const nvETime_t *end_time);
void navilock_initUartPort( int16_t comport );
void CNavilock_readTrackInfo(uint8_t track, char* buffer, uint8_t comport, navilock_t *nav, navi_track_t *trak);
float64_t setLatitLon(int16_t degrees, int16_t min, float64_t sec);
void navilock_write_uart( int16_t comport, uint8_t *buf, uint8_t len  );
#endif
/* ========= distance calculation for waypoints ============================= */
float64_t get_distance_to_next_waypoint(float64_t lat_now, float64_t lon_now, float64_t lat_next, float64_t lon_next);
float64_t get_bearing_to_next_waypoint(float64_t lat_now, float64_t lon_now, float64_t lat_next, float64_t lon_next);
void get_vector_to_next_waypoint(float64_t lat_now, float64_t lon_now, float64_t lat_next, float64_t lon_next, float64_t *v_n, float64_t *v_e);
void get_vector_to_next_waypoint_fast(float64_t lat_now, float64_t lon_now, float64_t lat_next, float64_t lon_next, float64_t *v_n, float64_t *v_e);
void add_vector_to_global_position(float64_t lat_now, float64_t lon_now, float64_t v_n, float64_t v_e, float64_t *lat_res, float64_t *lon_res);
void waypoint_from_heading_and_distance(float64_t lat_start, float64_t lon_start, float64_t bearing, float64_t dist,float64_t *lat_target, float64_t *lon_target);
float64_t mavlink_wpm_distance_to_point_local(float64_t x_now, float64_t y_now, float64_t z_now, float64_t x_next, float64_t y_next, float64_t z_next, float64_t *dist_xy, float64_t *dist_z);
float64_t get_distance_to_point_global_wgs84(float64_t lat_now, float64_t lon_now, float64_t alt_now, float64_t lat_next, float64_t lon_next, float64_t alt_next, float64_t *dist_xy, float64_t *dist_z);
int16_t get_distance_to_line(crosstrack_error_t *crosstrack_error, float64_t lat_now, float64_t lon_now, float64_t lat_start, float64_t lon_start, float64_t lat_end, float64_t lon_end);
int16_t get_distance_to_arc(crosstrack_error_t *crosstrack_error, float64_t lat_now, float64_t lon_now, float64_t lat_center, float64_t lon_center, float64_t radius, float64_t arc_start_bearing, float64_t arc_sweep);
float64_t hypot(float64_t x, float64_t y, float64_t z);
float64_t getDistanceVector(const Vectr from, const Vectr to);
float64_t fmin( float64_t a, float64_t b );
void getNavigateSetpoint( float64_t speed, geoPose_t nav_start, geoPose_t setpoint_position_transformed, Vectr *nav_setpoint, bool wait_armed );
/* ===== optical flow ==== */
void biquadLPF_init( biquadLPF_t *biq );
void biquadLPF_doBiquadLPF(float32_t filtf, float32_t q, float32_t sampf, biquadLPF_t *biq);
float32_t biquadLPF_update (float32_t input, biquadLPF_t *biq);
float32_t biquadLPF_reset (float32_t value, biquadLPF_t *biq);
Vectr * T265convQuaternionToEuler(float32_t x, float32_t y, float32_t z, float32_t w);
void T265oscEvent(const geoPose_t msg, T265Pose_t *pose);
mat33 mmul(mat33 a, mat33 b);
void mtranslate(mat33 *m, Vectr v);
void mscale(mat33 *m, Vectr v);
mat33 T265_calcHeadMatrix2(T265Pose_t *pose);
mat33 T265_calcHeadMatrix(float32_t tx, float32_t ty, float32_t tz, float32_t qw, float32_t qx, float32_t qy, float32_t qz);
#if defined(AIR_DENSITY_CALC)
float64_t getSatWaterVP(float64_t temperat);
float64_t getWaterVP(float64_t RH, float64_t satWaterVP);
float64_t getDryAirPPA(float64_t P, float64_t P_V);
float64_t getHumidAirDensity(float64_t ppDA, float64_t T_K, float64_t P_V );
float64_t getHumidAirDensity2(float64_t P, float64_t T_K, float64_t P_V );
float64_t getGeometricAlt(float64_t H);
float64_t getVirtTemp(float64_t T, float64_t E, float64_t P);
float64_t getGeoPotDenAlt(float64_t VirtTemp, float64_t P);
float64_t getActStatPress(float64_t AS, float64_t H);
float64_t getDensAlt(float64_t PA, float64_t T);
float64_t getPressError(float64_t H, float64_t D);
float64_t getDynPress(float64_t A, float64_t D);
float64_t getBaro1_WCF_LFT(float64_t Pd, float64_t Pe);
#endif
Vectr tripleValidateSignal(Vectr sig1, Vectr sig2, Vectr sig3, float64_t velocity);

//==============================================================================
//                                 Functions
//
//==============================================================================
// Orientation filter for IMU
// Filter for Accelerometer and Gyroscope data
// w_x w_y w_z = gyroscope readings in rad/s
// a_x a_y a_z = accelorometer readings
//
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float32_t invSqrt(float32_t x)
{
    float32_t halfx = 0.5f * x;
    float32_t y = x;
    uint32_t i = *(uint32_t*)&y;
    i = 0x5f3759dful - (i>>1u);
    y = *(float32_t*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
/* consider this version instead */
float32_t InvSqrt_2(float32_t x)
{
   uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
   float32_t tmp = *(float32_t*)&i;
   return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}
/*
The 3-heading triangular method from Gratton
Groundspeed must be measured, using GPS, whilst flying the aircraft on three
headings (not tracks – so heading must be measured using an error corrected compass,
not GPS) that differ by 120 degrees (eg 50, 170 and 290 degrees).  These speeds will
be termed V1, V2 and V3 */
/*-----------------------------------------------------------------------------
 *      calc3headingTriangular :  Wind Speed Estimator using 3 heading triangular
 *                        method from guy gratton (Brunel Uni)
 *
 *  Parameters: wind_compens_t *triangle
 *
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void calc3headingTriangular( wind_compens_t *triangle )
{
  float64_t A1, A2, A3, Y, V12;

  V12=(triangle->v1+triangle->v2+triangle->v3)/3.0f;
  A1=((triangle->v1*triangle->v1)/(V12*V12))-1.0f;
  A2=((triangle->v2*triangle->v1)/(V12*V12))-1.0f;
  A3=((triangle->v3*triangle->v1)/(V12*V12))-1.0f;
  Y=(((A1*A1)+(A2*A2))+(A3*A3))/6.0f;
  triangle->TrueAirSpeed=V12*sqrt(sqrt(1.0f/(4.0f-Y))+0.5f);                    /* TAS estimate */
  triangle->WindSpeed=(Y/(1.0f/(4.0f-Y))+0.5f)*V12;                             /* wind speed estimate */
}

/* This technique requires the aircraft to fly three legs at 90° spaced magnetic headings
   Three groundspeeds (V1, V2, V3) are recorded for each IAS value, each flown on an orthogonal
   cardinal heading (e.g. North, East then South) Ref : Lewis and Lowry
*/
/*-----------------------------------------------------------------------------
 *      calcBoxPattern :  Wind Speed Estimator using box pattern method of lowry 
 *                        and lewis
 *
 *  Parameters: wind_compens_t *boxPatt
 *
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void calcBoxPattern( wind_compens_t *boxPatt )
{
    float64_t relWindDir= 0.0f;
    float32_t windVelocity= 0.0f;
    float64_t windMantissaPos= 0.0f;
    float64_t windMantissaNeg= 0.0f;

    /* Lowry8 recommends that the first heading flown is due North, and thus ?
       becomes actual wind direction    */
    relWindDir= tanh((float64_t)((((((boxPatt->v2)*(boxPatt->v2))*2.0f)-((boxPatt->v1)*(boxPatt->v1)))-((boxPatt->v3)*(boxPatt->v3))) / (((boxPatt->v3)*(boxPatt->v3))-((boxPatt->v1)*(boxPatt->v1)))));

    windMantissaPos= (((float64_t)(((boxPatt->v3)*(boxPatt->v3))-((boxPatt->v1)*(boxPatt->v1)))) + sqrt((float64_t) ((((boxPatt->v3)*(boxPatt->v3))-((boxPatt->v1)*(boxPatt->v1)))*(((boxPatt->v3)*(boxPatt->v3))-((boxPatt->v1)*(boxPatt->v1)))) - ((((((boxPatt->v2)*(boxPatt->v2))*2.0f)-((boxPatt->v1)*(boxPatt->v1)))-((boxPatt->v3)*(boxPatt->v3))) / sin(relWindDir))));
    windMantissaNeg= (((float64_t)(((boxPatt->v3)*(boxPatt->v3))-((boxPatt->v1)*(boxPatt->v1)))) - sqrt((float64_t) ((((boxPatt->v3)*(boxPatt->v3))-((boxPatt->v1)*(boxPatt->v1)))*(((boxPatt->v3)*(boxPatt->v3))-((boxPatt->v1)*(boxPatt->v1)))) - ((((((boxPatt->v2)*(boxPatt->v2))*2.0f)-((boxPatt->v1)*(boxPatt->v1)))-((boxPatt->v3)*(boxPatt->v3))) / sin(relWindDir))));
    if (windMantissaPos < windMantissaNeg)                                      /* take the one which is highest positive */
    {
       boxPatt->WindSpeed= ((pow(windMantissaNeg,0.5f))*0.5f);                  /* use subtraction for wind speed estimate */
    }
    else
    {
       boxPatt->WindSpeed= ((pow(windMantissaPos,0.5f))*0.5f);                  /* use addition for wind speed estimate */
    }
    boxPatt->TrueAirSpeed= sqrt(((((((boxPatt->v2)*(boxPatt->v2))*2.0f)-((boxPatt->v1)*(boxPatt->v1)))-((boxPatt->v3)*(boxPatt->v3))) / 2.0f) - boxPatt->WindSpeed);  /* TAS estimate */
}
/*-----------------------------------------------------------------------------
 *  bluffBodyDrag :  Calculate the bluff body drag and momentum drag
 *
 *  Parameters: drag_coef_t *dragData, float32_t flowRelative2Fluid, float32_t mass, 
 *              float32_t frontArea, float32_t sideArea, float32_t a_x
 *
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
 void bluffBodyDrag( drag_coef_t *dragData, float32_t flowRelative2Fluid, float32_t mass, float32_t frontArea, float32_t sideArea, float32_t a_x )
 {
      dragData->ek3DragBCoeffX = mass / frontArea;
      dragData->ek3DragBCoeffY = mass / sideArea;      
      dragData->bluffBodyDragAcc = (0.5f * MASS_DENSITY_FLUID * (flowRelative2Fluid*flowRelative2Fluid)) / dragData->ek3DragBCoeffX; 
      dragData->momentumDragAcc = a_x - dragData->bluffBodyDragAcc;
      dragData->ek2DragMCoeff = dragData->momentumDragAcc / flowRelative2Fluid; /* this is the figure used in the ekf compensation */
 }
/*-----------------------------------------------------------------------------
 *  IMU_filterUpdate :  Filter for Accelerometer and Gyroscope data
 *
 *  Parameters: float32_t w_x, float32_t w_y, float32_t w_z, float32_t a_x,   
 *              float32_t a_y, float32_t a_z, COMGS_quarternion *quartCalculated
 *
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void IMU_filterUpdate(float32_t w_x, float32_t w_y, float32_t w_z, float32_t a_x, float32_t a_y, float32_t a_z, COMGS_quarternion *quartCalculated )
{
  
  // Local system variables
  float32_t norm;                                                               // vector norm
  float32_t SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;     // quaternion derrivative from gyroscopes elements
  float32_t f_1, f_2, f_3;                                                      // objective function elements
  float32_t J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;                 // objective function Jacobian elements
  float32_t SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;                 // estimated direction of the gyroscope error

  // Axulirary variables to avoid reapeated calcualtions
  float32_t halfSEq_1 = 0.5f * quartCalculated->SEq_1;
  float32_t halfSEq_2 = 0.5f * quartCalculated->SEq_2;
  float32_t halfSEq_3 = 0.5f * quartCalculated->SEq_3;
  float32_t halfSEq_4 = 0.5f * quartCalculated->SEq_4;
  float32_t twoSEq_1 = 2.0f * quartCalculated->SEq_1;
  float32_t twoSEq_2 = 2.0f * quartCalculated->SEq_2;
  float32_t twoSEq_3 = 2.0f * quartCalculated->SEq_3;
  
  // Initialisation of values (not sure if needed everytime or only on creation)
  quartCalculated->SEq_1=0;                                                     // estimated orientation quaternion elements with initial conditions
  quartCalculated->SEq_2=0;
  quartCalculated->SEq_3=0;
  quartCalculated->SEq_4=0;
  
  // Normalise the accelerometer measurement
  norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  a_x /= norm;
  a_y /= norm;
  a_z /= norm;
  // Compute the objective function and Jacobian
  f_1 = twoSEq_2 * quartCalculated->SEq_4 - twoSEq_1 * quartCalculated->SEq_3 - a_x;
  f_2 = twoSEq_1 * quartCalculated->SEq_2 + twoSEq_3 * quartCalculated->SEq_4 - a_y;
  f_3 = 1.0f - twoSEq_2 * quartCalculated->SEq_2 - twoSEq_3 * quartCalculated->SEq_3 - a_z;
  J_11or24 = twoSEq_3;
  // J_11 negated in matrix multiplication
  J_12or23 = 2.0f * quartCalculated->SEq_4;
  J_13or22 = twoSEq_1;                                                          // J_12 negated in matrix multiplication
  J_14or21 = twoSEq_2; J_32 = 2.0f * J_14or21;                                  // negated in matrix multiplication
  J_33 = 2.0f * J_11or24;                                                       // negated in matrix multiplication
  // Compute the gradient (matrix multiplication)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
  // Normalise the gradient
  norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
  SEqHatDot_1 /= norm;
  SEqHatDot_2 /= norm;
  SEqHatDot_3 /= norm;
  SEqHatDot_4 /= norm;
  // Compute the quaternion derrivative measured by gyroscopes
  SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
  SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
  SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
  SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
  // Compute then integrate the estimated quaternion derrivative
  quartCalculated->SEq_1 +=  SEqDot_omega_1;
  quartCalculated->SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
  quartCalculated->SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
  quartCalculated->SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
  quartCalculated->SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
  // Normalise quaternion
  norm = sqrt(pow(quartCalculated->SEq_1,2) + pow(quartCalculated->SEq_2,2) + pow(quartCalculated->SEq_3,2) + pow(quartCalculated->SEq_4,2));
  quartCalculated->SEq_1 /= norm;
  quartCalculated->SEq_2 /= norm;
  quartCalculated->SEq_3 /= norm;
  quartCalculated->SEq_4 /= norm;
}

//
// w_(x) y z = gyro data
// a_(x) y z = accelerometer data
// m_(x) y z = magnetometer data
/*-----------------------------------------------------------------------------
 *      MARG_AHRS_filterUpdate :  Marginal AHRS Filter
 *  Parameters: float32_t w_x, float32_t w_y, float32_t w_z, float32_t a_x, float32_t a_y, float32_t a_z, float32_t m_x, float32_t m_y, float32_t m_z,
 *  COMGS_quarternion *quartCalculated
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void MARG_AHRS_filterUpdate(float32_t w_x, float32_t w_y, float32_t w_z, float32_t a_x, float32_t a_y, float32_t a_z, float32_t m_x, float32_t m_y, float32_t m_z, COMGS_quarternion *quartCalculated)
{

   // local system variables
  float32_t norm; // vector norm
  float32_t SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;         // quaternion rate from gyroscopes elements
  float32_t f_1, f_2, f_3, f_4, f_5, f_6;                                           // objective function elements
  float32_t J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64;  // objective function Jacobian elements
  float32_t SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;                     // estimated direction of the gyroscope error
  float32_t w_err_x, w_err_y, w_err_z;                                              // estimated direction of the gyroscope error (angular)
  float32_t h_x, h_y, h_z;                                                          // computed flux in the earth frame
 
  // axulirary variables to avoid reapeated calcualtions
  float32_t halfSEq_1 = 0.5f * quartCalculated->SEq_1;
  float32_t halfSEq_2 = 0.5f * quartCalculated->SEq_2;
  float32_t halfSEq_3 = 0.5f * quartCalculated->SEq_3;
  float32_t halfSEq_4 = 0.5f * quartCalculated->SEq_4;
  float32_t twoSEq_1 = 2.0f * quartCalculated->SEq_1;
  float32_t twoSEq_2 = 2.0f * quartCalculated->SEq_2;
  float32_t twoSEq_3 = 2.0f * quartCalculated->SEq_3;
  float32_t twoSEq_4 = 2.0f * quartCalculated->SEq_4;
  float32_t twob_x = 2.0f * quartCalculated->b_x;
  float32_t twob_z = 2.0f * quartCalculated->b_z;
  float32_t twob_xSEq_1 = 2.0f * quartCalculated->b_x * quartCalculated->SEq_1;
  float32_t twob_xSEq_2 = 2.0f * quartCalculated->b_x * quartCalculated->SEq_2;
  float32_t twob_xSEq_3 = 2.0f * quartCalculated->b_x * quartCalculated->SEq_3;
  float32_t twob_xSEq_4 = 2.0f * quartCalculated->b_x * quartCalculated->SEq_4;
  float32_t twob_zSEq_1 = 2.0f * quartCalculated->b_z * quartCalculated->SEq_1;
  float32_t twob_zSEq_2 = 2.0f * quartCalculated->b_z * quartCalculated->SEq_2;
  float32_t twob_zSEq_3 = 2.0f * quartCalculated->b_z * quartCalculated->SEq_3;
  float32_t twob_zSEq_4 = 2.0f * quartCalculated->b_z * quartCalculated->SEq_4;
  float32_t SEq_1SEq_2;
  float32_t SEq_1SEq_3 = quartCalculated->SEq_1 * quartCalculated->SEq_3;
  float32_t SEq_1SEq_4;
  float32_t SEq_2SEq_3;
  float32_t SEq_2SEq_4 = quartCalculated->SEq_2 * quartCalculated->SEq_4;
  float32_t SEq_3SEq_4;
  float32_t twom_x = 2.0f * m_x;
  float32_t twom_y = 2.0f * m_y;
  float32_t twom_z = 2.0f * m_z;
  
  // Initialisation of values (not sure if needed everytime or on creation)
  quartCalculated->SEq_1 = 1, quartCalculated->SEq_2 = 0, quartCalculated->SEq_3 = 0, quartCalculated->SEq_4 = 0;        // estimated orientation quaternion elements with initial conditions
  quartCalculated->b_x = 1, quartCalculated->b_z = 0;                           // reference direction of flux in earth frame
  quartCalculated->w_bx = 0, quartCalculated->w_by = 0, quartCalculated->w_bz = 0;        // estimate gyroscope biases error

  // normalise the accelerometer measurement
  norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  a_x /= norm;
  a_y /= norm;
  a_z /= norm;
  // normalise the magnetometer measurement
  norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
  m_x /= norm;
  m_y /= norm;
  m_z /= norm;

  // compute the objective function and Jacobian
  f_1 = twoSEq_2 * quartCalculated->SEq_4 - twoSEq_1 * quartCalculated->SEq_3 - a_x;
  f_2 = twoSEq_1 * quartCalculated->SEq_2 + twoSEq_3 * quartCalculated->SEq_4 - a_y;
  f_3 = 1.0f - twoSEq_2 * quartCalculated->SEq_2 - twoSEq_3 * quartCalculated->SEq_3 - a_z;
  f_4 = twob_x * (0.5f - pow(quartCalculated->SEq_3,2) - pow(quartCalculated->SEq_4,2)) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x; 
  f_5 = twob_x * (quartCalculated->SEq_2 * quartCalculated->SEq_3 - quartCalculated->SEq_1 * quartCalculated->SEq_4) + twob_z * (quartCalculated->SEq_1 * quartCalculated->SEq_2 + quartCalculated->SEq_3 * quartCalculated->SEq_4) - m_y; 
  f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - pow(quartCalculated->SEq_2,2) - pow(quartCalculated->SEq_3,2)) - m_z;
  J_11or24 = twoSEq_3;
  // J_11 negated in matrix multiplication
  J_12or23 = 2.0f * quartCalculated->SEq_4;
  J_13or22 = twoSEq_1;
  // J_12 negated in matrix multiplication
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21;
  // negated in matrix multiplication
  J_33 = 2.0f * J_11or24;
  // negated in matrix multiplication
  J_41 = twob_zSEq_3;
  // negated in matrix multiplication
  J_42 = twob_zSEq_4;
  J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;
  // negated in matrix multiplication
  J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;
  // negated in matrix multiplication
  J_51 = twob_xSEq_4 - twob_zSEq_2;
  // negated in matrix multiplication
  J_52 = twob_xSEq_3 + twob_zSEq_1;
  J_53 = twob_xSEq_2 + twob_zSEq_4;
  J_54 = twob_xSEq_1 - twob_zSEq_3;
  // negated in matrix multiplication
  J_61 = twob_xSEq_3;
  J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
  J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
  J_64 = twob_xSEq_2;

  // compute the gradient (matrix multiplication)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6; SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6; SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
  // normalise the gradient to estimate direction of the gyroscope error
  norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
  SEqHatDot_1 = SEqHatDot_1 / norm;
  SEqHatDot_2 = SEqHatDot_2 / norm;
  SEqHatDot_3 = SEqHatDot_3 / norm;
  SEqHatDot_4 = SEqHatDot_4 / norm;
  // compute angular estimated direction of the gyroscope error
  w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
  w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
  w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
  // compute and remove the gyroscope baises
  quartCalculated->w_bx += w_err_x * deltat * zeta;
  quartCalculated->w_by += w_err_y * deltat * zeta;
  quartCalculated->w_bz += w_err_z * deltat * zeta;
  w_x -= quartCalculated->w_bx;
  w_y -= quartCalculated->w_by;
  w_z -= quartCalculated->w_bz;
  // compute the quaternion rate measured by gyroscopes
  SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
  SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
  SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
  SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
  // compute then integrate the estimated quaternion rate
  quartCalculated->SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
  quartCalculated->SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
  quartCalculated->SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
  quartCalculated->SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
  // normalise quaternion
  norm = sqrt(pow(quartCalculated->SEq_1,2) + pow(quartCalculated->SEq_2,2) + pow(quartCalculated->SEq_3,2) + pow(quartCalculated->SEq_4,2));
  quartCalculated->SEq_1 /= norm;
  quartCalculated->SEq_2 /= norm;
  quartCalculated->SEq_3 /= norm;
  quartCalculated->SEq_4 /= norm;
  // compute flux in the earth frame
  SEq_1SEq_2 = quartCalculated->SEq_1 * quartCalculated->SEq_2;
  // recompute axulirary variables
  SEq_1SEq_3 = quartCalculated->SEq_1 * quartCalculated->SEq_3;
  SEq_1SEq_4 = quartCalculated->SEq_1 * quartCalculated->SEq_4;
  SEq_3SEq_4 = quartCalculated->SEq_3 * quartCalculated->SEq_4;
  SEq_2SEq_3 = quartCalculated->SEq_2 * quartCalculated->SEq_3;
  SEq_2SEq_4 = quartCalculated->SEq_2 * quartCalculated->SEq_4;
  h_x = twom_x * (0.5f - quartCalculated->SEq_3 * quartCalculated->SEq_3 - quartCalculated->SEq_4 * quartCalculated->SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
  h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - quartCalculated->SEq_2 * quartCalculated->SEq_2 - quartCalculated->SEq_4 * quartCalculated->SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
  h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - quartCalculated->SEq_2 * quartCalculated->SEq_2 - quartCalculated->SEq_3 * quartCalculated->SEq_3);
  // normalise the flux vector to have only components in the x and z
  quartCalculated->b_x = sqrt((h_x * h_x) + (h_y * h_y));
  quartCalculated->b_z = h_z;
  
}

/*-----------------------------------------------------------------------------
 *      frand_func :  floating point random number generator
 *  Parameters: none
 *  Return:
 *         float32_t random number
 *----------------------------------------------------------------------------*/
#ifndef RAND_MAX                                                                /* set it to 32767 becasue thats wha tthe microE C  function states */
#define RAND_MAX 2147483647                                                     /*implementation defined for GNU C consider 32767 if not work */
#endif
float32_t frand_func()
{
    //return (rand());
    //return ( (rand() / (double) RAND_MAX) );
    return (2.0f * ((rand()/(float64_t)RAND_MAX) - 0.5f));

}

/*-----------------------------------------------------------------------------
 *      anglefromMag :  Calculate the angle from the magnetrometer data (X Y co-ordinates)
 *  Parameters: int16_t magX, int16_t magY
 *  Return:
 *         int16_t angle
 *----------------------------------------------------------------------------*/
int16_t anglefromMag( int16_t magX, int16_t magY )                              // Calculate Angle from magnetrometer data
{
    int16_t Angle;
    
    Angle = (int16_t)((atan2((float64_t)magY,(float64_t) magX) * 180.0f) / (float64_t) (4.0f * atan(1.0f)));
    if (Angle < 0)  Angle += 360;
    return (Angle);
}

/*-----------------------------------------------------------------------------
 *      NormaliseAccMag :  Normalise function for Magnetrometer or Accelorometer
 *  Parameters: float32_t ax, float32_t ay, float32_t az
 *  Return:
 *         uint8_t return 0=error 1=success
 *----------------------------------------------------------------------------*/
uint8_t NormaliseAccMag( float32_t ax, float32_t ay, float32_t az )
{
  float32_t norm;
  norm = (float32_t) sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return(0u);                                                 /* handle NaN */
  norm = 1.0f / norm;                                                           /* use reciprocal for division */
  ax *= norm;
  ay *= norm;
  az *= norm;
  return(1u);
}

/*-----------------------------------------------------------------------------
 *      RefDirEarthMag :  Reference direction of Earth's magnetic field and estimated direction of gravity and magnetic field
 *  Parameters: float32_t mx, float32_t my, float32_t mz, COMGS_quarternion *quartCalculated, COMGS_magcorr_t *magCalculated
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void RefDirEarthMag(float32_t mx, float32_t my, float32_t mz, COMGS_quarternion *quartCalculated, COMGS_magcorr_t *magCalculated)
{
    // Auxiliary variables to avoid repeated arithmetic
    float32_t q1q1 = quartCalculated->SEq_1 * quartCalculated->SEq_1;
    float32_t q1q2 = quartCalculated->SEq_1 * quartCalculated->SEq_2;
    float32_t q1q3 = quartCalculated->SEq_1 * quartCalculated->SEq_3;
    float32_t q1q4 = quartCalculated->SEq_1 * quartCalculated->SEq_4;
    float32_t q2q2 = quartCalculated->SEq_2 * quartCalculated->SEq_2;
    float32_t q2q3 = quartCalculated->SEq_2 * quartCalculated->SEq_3;
    float32_t q2q4 = quartCalculated->SEq_2 * quartCalculated->SEq_4;
    float32_t q3q3 = quartCalculated->SEq_3 * quartCalculated->SEq_3;
    float32_t q3q4 = quartCalculated->SEq_3 * quartCalculated->SEq_4;
    float32_t q4q4 = quartCalculated->SEq_4 * quartCalculated->SEq_4;
    
    // Reference direction of Earth's magnetic field
    quartCalculated->hx = 2f * mx * (0.5f - q3q3 - q4q4) + 2f * my * (q2q3 - q1q4) + 2f * mz * (q2q4 + q1q3);
    quartCalculated->hy = 2f * mx * (q2q3 + q1q4) + 2f * my * (0.5f - q2q2 - q4q4) + 2f * mz * (q3q4 - q1q2);
    quartCalculated->b_x = (float32_t) sqrt((quartCalculated->hx * quartCalculated->hx) + (quartCalculated->hy * quartCalculated->hy));
    quartCalculated->b_z = 2f * mx * (q2q4 - q1q3) + 2f * my * (q3q4 + q1q2) + 2f * mz * (0.5f - q2q2 - q3q3);
    
    // Estimated direction of gravity and magnetic field
    magCalculated->vx = 2f * (q2q4 - q1q3);
    magCalculated->vy = 2f * (q1q2 + q3q4);
    magCalculated->vz = q1q1 - q2q2 - q3q3 + q4q4;
    magCalculated->wx = 2f * quartCalculated->b_x * (0.5f - q3q3 - q4q4) + 2f * quartCalculated->b_z * (q2q4 - q1q3);
    magCalculated->wy = 2f * quartCalculated->b_x * (q2q3 - q1q4) + 2f * quartCalculated->b_z * (q1q2 + q3q4);
    magCalculated->wz = 2f * quartCalculated->b_x * (q1q3 + q2q4) + 2f * quartCalculated->b_z * (0.5f - q2q2 - q3q3);
}

/*-----------------------------------------------------------------------------
 *      AHRS2quartAGM :
 * PI Control giving quarternion output
 * reads accelorometer magnetrometer and gyro values as well as quartObject and magcorr objects
 * Algorithm AHRS update method.(another variant of maghony)
 *
 *  Parameters: COMGS_quarternion *quartCalculated, COMGS_magcorr_t *magCalculated, float32_t ax, float32_t ay, float32_t az, float32_t gx, float32_t gy, 
 *  float32_t gz, float32_t mx, float32_t my, float32_t mz, COMGS_QControl_t *QContObj
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void AHRS2quartAGM(COMGS_quarternion *quartCalculated, const COMGS_magcorr_t *magCalculated, float32_t ax, float32_t ay, float32_t az, float32_t gx, float32_t gy, float32_t gz, float32_t mx, float32_t my, float32_t mz, COMGS_QControl_t *QContObj )
{
            float32_t ex,ey,ez;                                                 // errors calculated
            float32_t pa,pb,pc;
            float32_t norm;                                                     // normalised quartenion
            int32_t samplePeriod;                                               // sample period calculated from the 100ms interrupt tick
            
             if (QContObj->mode_began == false)                                 // time initialisation request
             {
                  samplePeriod = -1;                                            // initialise the timer if its the first time we called this function
                  QContObj->mode_began = true;                                  // initialisation of time complete
             }

            calculateTimeDiff((int64_t*) &samplePeriod, (uint64_t*) &QContObj->lasttime);  // time in 100 ms counts from the interrupt
            samplePeriod *= 10;                                                 // convert to seconds
            
            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * magCalculated->vz - az * magCalculated->vy) + (my * magCalculated->wz - mz * magCalculated->wy);
            ey = (az * magCalculated->vx - ax * magCalculated->vz) + (mz * magCalculated->wx - mx * magCalculated->wz);
            ez = (ax * magCalculated->vy - ay * magCalculated->vx) + (mx * magCalculated->wy - my * magCalculated->wx);

            if (QContObj->Ki > 0f)                                              // integral parameter of q control object
            {
                QContObj->eInt[0u] += ex;                                       // accumulate integral error
                QContObj->eInt[1u] += ey;
                QContObj->eInt[2u] += ez;
            }
            else
            {
                QContObj->eInt[0u] = 0.0f;                                      // prevent integral wind up
                QContObj->eInt[1u] = 0.0f;
                QContObj->eInt[2u] = 0.0f;
            }

            gx = gx + QContObj->Kp * ex + QContObj->Ki * QContObj->eInt[0u];    // Apply feedback terms
            gy = gy + QContObj->Kp * ey + QContObj->Ki * QContObj->eInt[1u];
            gz = gz + QContObj->Kp * ez + QContObj->Ki * QContObj->eInt[2u];

            pa = quartCalculated->SEq_2;                                        // Integrate rate of change of quaternion
            pb = quartCalculated->SEq_3;
            pc = quartCalculated->SEq_4;
            quartCalculated->SEq_1 = quartCalculated->SEq_1 + (-quartCalculated->SEq_2 * gx - quartCalculated->SEq_3 * gy - quartCalculated->SEq_4 * gz) * (0.5f * samplePeriod);
            quartCalculated->SEq_2 = pa + (quartCalculated->SEq_1 * gx + pb * gz - pc * gy) * (0.5f * samplePeriod);
            quartCalculated->SEq_3 = pb + (quartCalculated->SEq_1 * gy - pa * gz + pc * gx) * (0.5f * samplePeriod);
            quartCalculated->SEq_4 = pc + (quartCalculated->SEq_1 * gz + pa * gy - pb * gx) * (0.5f * samplePeriod);

            // Normalise quaternion
            norm = (float32_t) sqrt(quartCalculated->SEq_1 * quartCalculated->SEq_1 + quartCalculated->SEq_2 * quartCalculated->SEq_2 + quartCalculated->SEq_3 * quartCalculated->SEq_3 + quartCalculated->SEq_4 * quartCalculated->SEq_4);
            norm = 1.0f / norm;

            QContObj->Quaternion[0u] = quartCalculated->SEq_1 * norm;
            QContObj->Quaternion[1u] = quartCalculated->SEq_2 * norm;
            QContObj->Quaternion[2u] = quartCalculated->SEq_3 * norm;
            QContObj->Quaternion[3u] = quartCalculated->SEq_4 * norm;

}

/*-----------------------------------------------------------------------------
 *      AHRS2quartAG :
 * PI Control giving quarternion output
 * reads accelorometer magnetrometer and gyro values as well as quartObject and magcorr objects
 * Algorithm AHRS update method.(another variant of maghony)
 *
 *  Parameters: COMGS_quarternion *quartCalculated, float32_t ax, float32_t ay, float32_t az, float32_t gx, float32_t gy, float32_t gz, 
 *  COMGS_magcorr_t *magCalculated, COMGS_QControl_t *QContObj
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void AHRS2quartAG(COMGS_quarternion *quartCalculated, float32_t ax, float32_t ay, float32_t az, float32_t gx, float32_t gy, float32_t gz, COMGS_magcorr_t *magCalculated, COMGS_QControl_t *QContObj )
{
            float32_t ex,ey,ez;                                                 // errors calculated
            float32_t pa,pb,pc;
            float32_t norm;                                                     // normalised quartenion
            int64_t samplePeriod;                                               // sample period calculated from the 100ms interrupt tick

             if (QContObj->mode_began == false)                                 // time initialisation request
             {
                  samplePeriod = -1;                                            // initialise the timer if its the first time we called this function
                  QContObj->mode_began = true;                                  // initialisation of time complete
             }

            calculateTimeDiff(&samplePeriod,&QContObj->lasttime);               // time in 100 ms counts from the interrupt
            samplePeriod *= 10.0f;
            if (NormaliseAccMag(ax,ay,az))                                      // Normalise acc data
            {
               // Estimated direction of gravity
               magCalculated->vx = 2.0f * (quartCalculated->SEq_2 * quartCalculated->SEq_4 - quartCalculated->SEq_1 * quartCalculated->SEq_3);
               magCalculated->vy = 2.0f * (quartCalculated->SEq_1 * quartCalculated->SEq_2 + quartCalculated->SEq_3 * quartCalculated->SEq_4);
               magCalculated->vz = quartCalculated->SEq_1 * quartCalculated->SEq_1 - quartCalculated->SEq_2 * quartCalculated->SEq_2 - quartCalculated->SEq_3 * quartCalculated->SEq_3 + quartCalculated->SEq_4 * quartCalculated->SEq_4;

              // Error is cross product between estimated direction and measured direction of gravity
              ex = (ay * magCalculated->vz - az * magCalculated->vy);
              ey = (az * magCalculated->vx - ax * magCalculated->vz);
              ez = (ax * magCalculated->vy - ay * magCalculated->vx);

              if (QContObj->Ki > 0f)
              {
                QContObj->eInt[0u] += ex;                                       // accumulate integral error
                QContObj->eInt[1u] += ey;
                QContObj->eInt[2u] += ez;
              }
              else
              {
                QContObj->eInt[0u] = 0.0f;                                      // prevent integral wind up
                QContObj->eInt[1u] = 0.0f;
                QContObj->eInt[2u] = 0.0f;
              }

              gx = gx + QContObj->Kp * ex + QContObj->Ki * QContObj->eInt[0u];  // Apply feedback terms
              gy = gy + QContObj->Kp * ey + QContObj->Ki * QContObj->eInt[1u];
              gz = gz + QContObj->Kp * ez + QContObj->Ki * QContObj->eInt[2u];

              // Integrate rate of change of quaternion
              pa = quartCalculated->SEq_2;
              pb = quartCalculated->SEq_3;
              pc = quartCalculated->SEq_4;
              quartCalculated->SEq_1 = quartCalculated->SEq_1 + (-quartCalculated->SEq_2 * gx - quartCalculated->SEq_3 * gy - quartCalculated->SEq_4 * gz) * (0.5f * samplePeriod);
              quartCalculated->SEq_2 = pa + (quartCalculated->SEq_1 * gx + pb * gz - pc * gy) * (0.5f * samplePeriod);
              quartCalculated->SEq_3 = pb + (quartCalculated->SEq_1 * gy - pa * gz + pc * gx) * (0.5f * samplePeriod);
              quartCalculated->SEq_4 = pc + (quartCalculated->SEq_1 * gz + pa * gy - pb * gx) * (0.5f * samplePeriod);

              // Normalise quaternion
              norm = (float32_t) sqrt(quartCalculated->SEq_1 * quartCalculated->SEq_1 + quartCalculated->SEq_2 * quartCalculated->SEq_2 + quartCalculated->SEq_3 * quartCalculated->SEq_3 + quartCalculated->SEq_4 * quartCalculated->SEq_4);
              norm = 1.0f / norm;
              QContObj->Quaternion[0u] = quartCalculated->SEq_1 * norm;
              QContObj->Quaternion[1u] = quartCalculated->SEq_2 * norm;
              QContObj->Quaternion[2u] = quartCalculated->SEq_3 * norm;
              QContObj->Quaternion[3u] = quartCalculated->SEq_4 * norm;
           }
}

/*-----------------------------------------------------------------------------
 *      Number2String : Convert a number to a string (should be less overhead than sprintf - test performance)
 *
 *  Parameters: float32_t x, unsigned char *str, unsigned char precision
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void Number2String(float32_t x, unsigned char *str, unsigned char precision)    // Convert a number to an ascii string
{
   int16_t ie, i, k, ndig;
   float64_t y;
   if (precision>=7u) precision=7u;  else precision++;                          // max digits is 7
   ndig =   precision;
   ie = 0;
   if ( x < 0.0000f)                                                             // negative
   {
     *str++ = '-';
     x = -x;
   }
   if (x > 0.00000f) while (x < 1.00000f)                                         // less than 1
   {
     x *= 10.000f;
     ie--;
   }
   while (x >= 10.000f)                                                          // bigger than 10
   {
     x = x/10.000f;
     ie=++ie % UINT16_MAX;
   }
   ndig += ie;
   for (y = i = 1; i < ndig; i++)
   y = y/10.000f;
   x += y/2.000f;
   if (x >= 10.000f) {x = 1.000f; ie++;}                                          // between 10 and 1
   if (ie<0)
   {
     *str++ = '0'; *str++ = '.';
     if (ndig < 0) ie = ie-ndig;
     for (i = -1; i > ie; i--)  *str++ = '0';
   }
   for (i=0; i < ndig; i++)
   {
     k = x;
     *str++ = k + '0';
     if (i ==  ie ) *str++ = '.';
     x -= (y=k);
     x *= 10.000f;
   }
   *str = '\0';
}
/*-----------------------------------------------------------------------------
 *      quatcompress : compress quartenion
 *      Ported from craZyFlie == MIT License
*       Copyright (c) 2016 James A. Preiss
 *  Parameters: float32_t const q[4u]
 *  Return:
 *        uint32_t compressed value
 *----------------------------------------------------------------------------*/
uint32_t quatcompress(float32_t const q[4u])
{

        uint32_t i_largest = 0u;                                                 // we send the values of the quaternion's smallest 3 elements.
        uint32_t i,negate,comp,negbit,mag;
        
        for (i = 1u; i < 4u; ++i)
        {
           if (fabs(q[i]) > fabs(q[i_largest]))
           {
              i_largest = i;
           }
        }

        // since -q represents the same rotation as q,
        // transform the quaternion so the largest element is positive.
        // this avoids having to send its sign bit.
        negate = q[i_largest] < 0;

        // 1/sqrt(2) is the largest possible value
        // of the second-largest element in a unit quaternion.

        // do compression using sign bit and 9-bit precision per element.
        comp = i_largest;
        for (i = 0u; i < 4u; ++i)
        {
            if (i != i_largest)
            {
                negbit = (q[i] < 0u) ^ negate;
                mag = ((1u << 9u) - 1u) * (fabs(q[i]) / sqrt(2.0f)) + 0.5f;
                comp = (comp << 10u) | (negbit << 9u) | mag;
            }
        }
        return comp;
}
/*-----------------------------------------------------------------------------
 *      quatdecompress : de-compress quartenion
 *      Ported from craZyFlie == MIT License
*       Copyright (c) 2016 James A. Preiss
 *  Parameters: uint32_t comp, float32_t q[4u]
 *  Return:
 *        void
 *----------------------------------------------------------------------------*/
void quatdecompress(uint32_t comp, float32_t *q[4u])
{
        uint32_t const mask = (1u << 9u) - 1u;
        int16_t const i_largest = comp >> 30u;
        float32_t sum_squares = 0.0f;
        int16_t i;
        uint32_t mag,negbit;
        
        for (i = 3; i >= 0; --i) 
        {
                if (i != i_largest) 
                {
                   mag = comp & mask;
                   negbit = (comp >> 9) & 0x1;
                   comp = comp >> 10;
                   *q[i] = (sqrt(2.0f)) * ((float32_t)mag) / mask;
                   if (negbit == 1) 
                   {
                      *q[i] = -*q[i];
                   }
                   sum_squares += *q[i] * *q[i];
                }
        }
        *q[i_largest] = sqrt(1.0f - sum_squares);
}

/*-----------------------------------------------------------------------------
 *      clampReal : clamp floating Point value between limits
 *  Parameters: float32_t value, float32_t min, float32_t max
 *  Return:
 *        float32_t
 *----------------------------------------------------------------------------*/
float32_t clampReal(float32_t value, float32_t min, float32_t max) 
{
  float32_t ret;
  if (value < min) 
  {
    ret = min;
  }
  else if (value > max) 
  {
    ret = max;
  }
  else
  {
    ret = value;
  }
  return ret;
}
/*-----------------------------------------------------------------------------
 *      clampU32 : clamp integer value between limits
 *  Parameters: uint32_t value, uint32_t min, uint32_t max
 *  Return:
 *        uint32_t
 *----------------------------------------------------------------------------*/
uint32_t clampU32(uint32_t value, uint32_t min, uint32_t max)
{
  uint32_t ret;
  if (value < min)
  {
    ret = min;
  }
  else if (value > max)
  {
    ret = max;
  }
  else
  {
    ret = value;
  }
  return ret;
}
/*-----------------------------------------------------------------------------
 *      clampI16 : clamp integer 16 value between limits
 *  Parameters: int16_t value, int16_t min, int16_t max
 *  Return:
 *        uint16_t
 *----------------------------------------------------------------------------*/
uint16_t clampI16(int16_t value, int16_t min, int16_t max)
{
  uint16_t ret;
  if (value < min)
  {
    ret = min;
  }
  else if (value > max)
  {
    ret = max;
  }
  else
  {
    ret = value;
  }
  return ret;
}
/*-----------------------------------------------------------------------------
 *      MahonyAHRSupdate : AHRS algorithm update
 *
 *  Parameters: float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, 
 *  float32_t mx, float32_t my, float32_t mz,  COMGS_quarternion *quartCalculated
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void MahonyAHRSupdate(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, float32_t mx, float32_t my, float32_t mz,  COMGS_quarternion *quartCalculated)
{
        float32_t recipNorm;
        float32_t q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        float32_t hx, hy, bx, bz;
        float32_t halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
        float32_t halfex, halfey, halfez;
        float32_t qa, qb, qc;
        float32_t sampleFreq;                                                   // calculated sample frequency
        int64_t samplePeriod;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) 
        {
           MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, quartCalculated);
           return;
        }

        if (quartCalculated->mode_began == false)                                // time initialisation request
        {
          samplePeriod = -1;                                                    // initialise the timer if its the first time we called this function
          quartCalculated->mode_began = true;                                   // initialisation of time complete
          quartCalculated->isCalib = false;                                     // request calibration
        }

        calculateTimeDiff(&samplePeriod, &quartCalculated->lasttime);             // time in 100 ms counts from the interrupt
        samplePeriod /= 10;                                                     // in seconds
        sampleFreq = 1.0f / (float32_t) samplePeriod;                           // convert to hertz
        
        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
        {

           // Normalise accelerometer measurement
           recipNorm = invSqrt(ax * ax + ay * ay + az * az);
           ax *= recipNorm;
           ay *= recipNorm;
           az *= recipNorm;

           // Normalise magnetometer measurement
           recipNorm = invSqrt(mx * mx + my * my + mz * mz);
           mx *= recipNorm;
           my *= recipNorm;
           mz *= recipNorm;

           // Auxiliary variables to avoid repeated arithmetic
           q0q0 = q0 * q0;
           q0q1 = q0 * q1;
           q0q2 = q0 * q2;
           q0q3 = q0 * q3;
           q1q1 = q1 * q1;
           q1q2 = q1 * q2;
           q1q3 = q1 * q3;
           q2q2 = q2 * q2;
           q2q3 = q2 * q3;
           q3q3 = q3 * q3;

           // Reference direction of Earth's magnetic field
           hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
           hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
           bx = sqrt(hx * hx + hy * hy);
           bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

           // Estimated direction of gravity and magnetic field
          halfvx = q1q3 - q0q2;
          halfvy = q0q1 + q2q3;
          halfvz = q0q0 - 0.5f + q3q3;
          halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
          halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
          halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
          
          // Error is sum of cross product between estimated direction and measured direction of field vectors
          halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
          halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
          halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
          
          // Compute and apply integral feedback if enabled
          if(twoKi > 0.0f)
          {
             integralFBx += twoKi * halfex * (1.0f / sampleFreq);                 // integral error scaled by Ki
             integralFBy += twoKi * halfey * (1.0f / sampleFreq);
             integralFBz += twoKi * halfez * (1.0f / sampleFreq);
             gx += integralFBx;                                                   // apply integral feedback
             gy += integralFBy;
             gz += integralFBz;
          }
          else
          {
             integralFBx = 0.0f;                                                 // prevent integral windup
             integralFBy = 0.0f;
             integralFBz = 0.0f;
          }
          
          // Apply proportional feedback
          gx += twoKp * halfex;
          gy += twoKp * halfey;
          gz += twoKp * halfez;
       }
       // Integrate rate of change of quaternion
       gx *= (0.5f * (1.0f / sampleFreq));                                         // pre-multiply common factors
       gy *= (0.5f * (1.0f / sampleFreq));
       gz *= (0.5f * (1.0f / sampleFreq));
       qa = q0;
       qb = q1;
       qc = q2;
       q0 += (-qb * gx - qc * gy - q3 * gz);
       q1 += (qa * gx + qc * gz - q3 * gy);
       q2 += (qa * gy - qb * gz + q3 * gx);
       q3 += (qa * gz + qb * gy - qc * gx);

       // Normalise quaternion and write back to the structure
       recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
       q0 *= recipNorm;
       q1 *= recipNorm;
       q2 *= recipNorm;
       q3 *= recipNorm;
       quartCalculated->SEq_1 = q0;
       quartCalculated->SEq_2 = q1;
       quartCalculated->SEq_3 = q2;
       quartCalculated->SEq_4 = q3;
}

/*-----------------------------------------------------------------------------
 *      MahonyAHRSupdateIMU : AHRS algorithm update
 *
 *  Parameters: float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az,
 *  COMGS_quarternion *quartCalculated
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void MahonyAHRSupdateIMU(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, COMGS_quarternion *quartCalculated )
{
        float32_t recipNorm;
        float32_t halfvx, halfvy, halfvz;
        float32_t halfex, halfey, halfez;
        float32_t qa, qb, qc;
        int64_t samplePeriod;                                                   // calculated sample period from 100 ms interrupt timer
        float32_t sampleFreq;                                                   // calcualted frquency of iteration

        if (quartCalculated->mode_began == false)                                // time initialisation request
        {
          samplePeriod = -1;                                                    // initialise the timer if its the first time we called this function
          quartCalculated->mode_began = true;                                   // initialisation of time complete
          quartCalculated->isCalib = false;                                     // request calibration
        }

        calculateTimeDiff(&samplePeriod, &quartCalculated->lasttime);             // time in 100 ms counts from the interrupt
        samplePeriod /= 10;                                                     // in seconds
        sampleFreq = 1.0f / (float32_t) samplePeriod;                           // convert to hertz

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

                // Normalise accelerometer measurement
                recipNorm = invSqrt(ax * ax + ay * ay + az * az);
                ax *= recipNorm;
                ay *= recipNorm;
                az *= recipNorm;

                // Estimated direction of gravity and vector perpendicular to magnetic flux
                halfvx = q1 * q3 - q0 * q2;
                halfvy = q0 * q1 + q2 * q3;
                halfvz = q0 * q0 - 0.5f + q3 * q3;

                // Error is sum of cross product between estimated and measured direction of gravity
                halfex = (ay * halfvz - az * halfvy);
                halfey = (az * halfvx - ax * halfvz);
                halfez = (ax * halfvy - ay * halfvx);

                // Compute and apply integral feedback if enabled
                if(twoKi > 0.0f) 
                {
                    integralFBx += twoKi * halfex * (1.0f / sampleFreq);        // integral error scaled by Ki
                    integralFBy += twoKi * halfey * (1.0f / sampleFreq);
                    integralFBz += twoKi * halfez * (1.0f / sampleFreq);
                    gx += integralFBx;                                          // apply integral feedback
                    gy += integralFBy;
                    gz += integralFBz;
                }
                else 
                {
                    integralFBx = 0.0f;                                         // prevent integral windup
                    integralFBy = 0.0f;
                    integralFBz = 0.0f;
                }

                // Apply proportional feedback
                gx += twoKp * halfex;
                gy += twoKp * halfey;
                gz += twoKp * halfez;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5f * (1.0f / sampleFreq));                                        // pre-multiply common factors
        gy *= (0.5f * (1.0f / sampleFreq));
        gz *= (0.5f * (1.0f / sampleFreq));
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        quartCalculated->SEq_1 = q0;
        quartCalculated->SEq_2 = q1;
        quartCalculated->SEq_3 = q2;
        quartCalculated->SEq_4 = q3;
}
/*-----------------------------------------------------------------------------
 *      sensfusion6GetQuaternion : get quartenion from the global volatile registers
 *
 *  Parameters: float* qx, float* qy, float* qz, float* qw
 *
 *  Return:
 *         void returns the value into the variables passed to the routine
 *----------------------------------------------------------------------------*/
void sensfusion6GetQuaternion(float32_t* qx, float32_t* qy, float32_t* qz, float32_t* qw)
{
  *qx = q1;
  *qy = q2;
  *qz = q3;
  *qw = q0;
}
/*-----------------------------------------------------------------------------
 *      sensfusion6GetEulerRPY : compensation using madgewick or mahony
 *
 *  Parameters: float32_t* roll, float32_t* pitch, float32_t* yaw
 *
 *  Return:
 *         void
 *----------------------------------------------------------------------------*/
void sensfusion6GetEulerRPY(float32_t* roll, float32_t* pitch, float32_t* yaw)
{
  float32_t gx = gravX;
  float32_t gy = gravY;
  float32_t gz = gravZ;

  if (gx>1.0f) gx=1.0f;
  if (gx<-1.0f) gx=-1.0f;

  *yaw = atan2(2.0f*(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 180.0f / PI;
  *pitch = asin(gx) * 180.0f / PI;                                              //Pitch seems to be inverted
  *roll = atan2(gy, gz) * 180.0f / PI;
}
/*-----------------------------------------------------------------------------
 *      sensfusion6GetInvThrustCompensationForTilt :
 *
 *  Parameters: none
 *
 *  Return:
 *         float32_t  Return the z component of the estimated gravity direction
 *----------------------------------------------------------------------------*/
float32_t sensfusion6GetInvThrustCompensationForTilt()
{
  return gravZ;                                                                 // (0, 0, 1) dot G
}
/*-----------------------------------------------------------------------------
 *      sensfusion6GetAccZ : sensor fusion 6 get accelerometer
 *
 *  Parameters: const float32_t ax, const float32_t ay, const float32_t az
 *
 *  Return:
 *         float32_t
 *----------------------------------------------------------------------------*/
float32_t sensfusion6GetAccZ(const float32_t ax, const float32_t ay, const float32_t az)
{
  return (ax * gravX + ay * gravY + az * gravZ);                                // return vertical acceleration (A dot G) / |G|,  (|G| = 1) -> (A dot G)
}
/*-----------------------------------------------------------------------------
 *      estimatedGravityDirection : estimated gravity direction
 *
 *  Parameters: float32_t* gx, float32_t* gy, float32_t* gz
 *
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void estimatedGravityDirection(float32_t* gx, float32_t* gy, float32_t* gz)
{
  *gx = 2 * (q1 * q3 - q0 * q2);
  *gy = 2 * (q0 * q1 + q2 * q3);
  *gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}
/*-----------------------------------------------------------------------------
 *      sensfusion6GetAccZWithoutGravity :
 *
 *  Parameters: const float32_t ax, const float32_t ay, const float32_t az
 *
 *  Return:
 *         float32_t
 *----------------------------------------------------------------------------*/
float32_t sensfusion6GetAccZWithoutGravity(const float32_t ax, const float32_t ay, const float32_t az)
{
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}
/*-----------------------------------------------------------------------------
 *      MadgwickAHRSupdateIMU : IMU algorithm update Madgwick
 *
 *  Parameters: float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az,
 *  COMGS_quarternion *quartCalculated
 *  See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void MadgwickAHRSupdateIMU(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, COMGS_quarternion *quartCalculated)
{
        float32_t recipNorm;
        float32_t s0, s1, s2, s3;
        float32_t qDot1, qDot2, qDot3, qDot4;
        float32_t _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
        int64_t samplePeriod;                                                   // calculated sample period from 100 ms interrupt timer
        float32_t sampleFreq;                                                   // calcualted frquency of iteration

        if (quartCalculated->mode_began == false)                                // time initialisation request
        {
          samplePeriod = -1;                                                    // initialise the timer if its the first time we called this function
          quartCalculated->mode_began = true;                                   // initialisation of time complete
          quartCalculated->isCalib = false;                                     // request calibration
        }

        calculateTimeDiff(&samplePeriod, &quartCalculated->lasttime);           // time in 100 ms counts from the interrupt
        samplePeriod /= 10;                                                     // in seconds
        sampleFreq = 1.0f / (float32_t) samplePeriod;                           // convert to hertz
        
        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
        {

           // Normalise accelerometer measurement
           recipNorm = invSqrt(ax * ax + ay * ay + az * az);
           ax *= recipNorm;
           ay *= recipNorm;
           az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _8q1 = 8.0f * q1;
            _8q2 = 8.0f * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);     // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= betaMadg * s0;
            qDot2 -= betaMadg * s1;
            qDot3 -= betaMadg * s2;
            qDot4 -= betaMadg * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * (1.0f / sampleFreq);
        q1 += qDot2 * (1.0f / sampleFreq);
        q2 += qDot3 * (1.0f / sampleFreq);
        q3 += qDot4 * (1.0f / sampleFreq);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        quartCalculated->SEq_1 = q0;                                            // set the values in the external structure
        quartCalculated->SEq_2 = q1;
        quartCalculated->SEq_3 = q2;
        quartCalculated->SEq_4 = q3;
}

/*-----------------------------------------------------------------------------
 *      MadgwickAHRSupdate:  AHRS algorithm update Madgwick
 *
 *  Parameters: float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, 
 *              float32_t az, float32_t mx, float32_t my, float32_t mz, COMGS_quarternion *quartCalculated
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void MadgwickAHRSupdate(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, float32_t mx, float32_t my, float32_t mz, COMGS_quarternion *quartCalculated)
{
        float32_t recipNorm;
        float32_t s0, s1, s2, s3;
        float32_t qDot1, qDot2, qDot3, qDot4;
        float32_t hx, hy;
        float32_t _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        int64_t samplePeriod;                                                   // calculated sample period from 100 ms interrupt timer
        float32_t sampleFreq;                                                   // calcualted frquency of iteration

        if (quartCalculated->mode_began == false)                                // time initialisation request
        {
          samplePeriod = -1;                                                    // initialise the timer if its the first time we called this function
          quartCalculated->mode_began = true;                                   // initialisation of time complete
          quartCalculated->isCalib = false;                                     // request calibration
        }

        calculateTimeDiff(&samplePeriod, &quartCalculated->lasttime);           // time in 100 ms counts from the interrupt
        samplePeriod /= 10.0f;                                                  // in seconds
        sampleFreq = 1.0f / (float32_t) samplePeriod;                           // convert to hertz

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) 
        {
            MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, quartCalculated);
            return;
        }

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
        {

           // Normalise accelerometer measurement
           recipNorm = invSqrt(ax * ax + ay * ay + az * az);
           ax *= recipNorm;
           ay *= recipNorm;
           az *= recipNorm;

           // Normalise magnetometer measurement
           recipNorm = invSqrt(mx * mx + my * my + mz * mz);
           mx *= recipNorm;
           my *= recipNorm;
           mz *= recipNorm;

          // Auxiliary variables to avoid repeated arithmetic
          _2q0mx = 2.0f * q0 * mx;
          _2q0my = 2.0f * q0 * my;
          _2q0mz = 2.0f * q0 * mz;
          _2q1mx = 2.0f * q1 * mx;
          _2q0 = 2.0f * q0;
          _2q1 = 2.0f * q1;
          _2q2 = 2.0f * q2;
          _2q3 = 2.0f * q3;
          _2q0q2 = 2.0f * q0 * q2;
          _2q2q3 = 2.0f * q2 * q3;
          q0q0 = q0 * q0;
          q0q1 = q0 * q1;
          q0q2 = q0 * q2;
          q0q3 = q0 * q3;
          q1q1 = q1 * q1;
          q1q2 = q1 * q2;
          q1q3 = q1 * q3;
          q2q2 = q2 * q2;
          q2q3 = q2 * q3;
          q3q3 = q3 * q3;

          // Reference direction of Earth's magnetic field
          hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
          hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
         _2bx = sqrt(hx * hx + hy * hy);
         _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
         _4bx = 2.0f * _2bx;
         _4bz = 2.0f * _2bz;

          // Gradient decent algorithm corrective step
          s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
          s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
          s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
          s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
          recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);     // normalise step magnitude
          s0 *= recipNorm;
          s1 *= recipNorm;
          s2 *= recipNorm;
          s3 *= recipNorm;

          // Apply feedback step
          qDot1 -= betaMadg * s0;
          qDot2 -= betaMadg * s1;
          qDot3 -= betaMadg * s2;
          qDot4 -= betaMadg * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * (1.0f / sampleFreq);
        q1 += qDot2 * (1.0f / sampleFreq);
        q2 += qDot3 * (1.0f / sampleFreq);
        q3 += qDot4 * (1.0f / sampleFreq);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        quartCalculated->SEq_1 = q0;                                            // set the values in the external structure
        quartCalculated->SEq_2 = q1;
        quartCalculated->SEq_3 = q2;
        quartCalculated->SEq_4 = q3;
}
/*-----------------------------------------------------------------------------
 *      sensfusion6UpdateQ:  perform correction using madgewick
 *
 *  Parameters: float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, 
 *              float32_t az, COMGS_quarternion *quartCalculated
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void sensfusion6UpdateQ(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, COMGS_quarternion *quartCalculated)
{
  switch (quartCalculated->algoType)
  {
     case ALGO_IS_MADGEWICK:
     MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, quartCalculated);            /* compensate using madgewick */
     break;
     
     case ALGO_IS_MAHONY:
     MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, quartCalculated);              /* compensate using mahony */
     break;
  }
  estimatedGravityDirection(&gravX, &gravY, &gravZ);

  if (quartCalculated->isCalib == false)                                        /* needs calibration */
  {
    baseZacc = sensfusion6GetAccZ(ax, ay, az);
    quartCalculated->isCalib = true;
  }
}
/*-----------------------------------------------------------------------------
 *      initPosEstselfState:  initialise the position estimator self state object
 *
 *  Parameters: selfState_s *state
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void initPosEstselfState( selfState_s *state )
{
  state->estimatedZ = 0.0f;                                                     // The current Z estimate, has same offset as asl
  state->velocityZ = 0.0f;                                                      // Vertical speed (world frame) integrated from vertical acceleration (m/s)
  state->estAlphaZrange = 0.9f;
  state->estAlphaAsl = 0.997f;
  state->velocityFactor = 1.0f;
  state->vAccDeadband = 0.04f;                                                  // Vertical acceleration deadband
  state->velZAlpha = 0.995f;                                                    // Blending factor to avoid vertical speed to accumulate error
  state->estimatedVZ = 0.0f;
}
/*-----------------------------------------------------------------------------
 *      deadband:  deadband calculation used in positon estimation
 *
 *  Parameters: float32_t value, const float32_t threshold
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/
float32_t deadband(float32_t value, const float32_t threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0.0f;
  }
  else if (value > 0.0f)
  {
    value -= threshold;
  }
  else if (value < 0.0f)
  {
    value += threshold;
  }
  return value;
}
/*-----------------------------------------------------------------------------
 *      positionUpdateVelocity:  update the velocity in the self state object
 *
 *  Parameters: float32_t accWZ, float32_t dt, selfState_s* state
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void positionUpdateVelocity(float32_t accWZ, float32_t dt, selfState_s* state)
{
  state->velocityZ += deadband(accWZ, state->vAccDeadband) * dt * GRAV_CONST;
  state->velocityZ *= state->velZAlpha;
}
/*-----------------------------------------------------------------------------
 *      positionEstimate:  position estimate from instruments
 *
 *  Parameters: state_t* estimate, const sensorData_t* sensorData, const tofMeasurement_t* tofMeasurement, 
 *              float32_t dt, selfState_s* state
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void positionEstimate(state_t* estimate, const sensorData_t* sensorData, const tofMeasurement_t* tofMeasurement, float32_t dt, selfState_s* state)
{
  float32_t filteredZ;
  static float32_t prev_estimatedZ = 0.0f;
  static uint8_t surfaceFollowingMode = false;

  const uint32_t MAX_SAMPLE_AGE = 10000LU;                                      /* in co-processor ticks */

  uint32_t now = CP0_GET(CP0_COUNT);                                            /* get the ticks from the co-processor */
  uint8_t isSampleUseful = ((now - tofMeasurement->timestamp) <= MAX_SAMPLE_AGE);

  if (isSampleUseful) 
  {
    surfaceFollowingMode = true;
  }
  if (surfaceFollowingMode)
  {
    if (isSampleUseful) 
    {
      filteredZ = (state->estAlphaZrange) * state->estimatedZ + (1.0f - state->estAlphaZrange) * tofMeasurement->distance;       // IIR filter zrange
      state->estimatedZ = filteredZ + (state->velocityFactor * state->velocityZ * dt);       // Use zrange as base and add velocity changes.
    }
  } 
  else 
  {
    if (state->estimatedZ == 0.0f)                                              // FIXME: A bit of an hack to init IIR filter
    {
      filteredZ = sensorData->baro.asl;
    } 
    else 
    {
      filteredZ = (state->estAlphaAsl) * state->estimatedZ + (1.0f - state->estAlphaAsl) * sensorData->baro.asl;         // IIR filter asl
    }
    state->estimatedZ = filteredZ + (state->velocityFactor * state->velocityZ * dt);      // Use asl as base and add velocity changes.
  }

  estimate->position.x = 0.0f;
  estimate->position.y = 0.0f;
  estimate->position.z = state->estimatedZ;
  estimate->velocity.z = (state->estimatedZ - prev_estimatedZ) / dt;
  state->estimatedVZ = estimate->velocity.z;
  prev_estimatedZ = state->estimatedZ;
}
/*-----------------------------------------------------------------------------
 *      readLwNxLaser:  Read the laser distance sensor for use in position estimator
 *
 *  Parameters: tofMeasurement_t *tof, lwDistanceResp_t *laser
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void readLwNxLaser( tofMeasurement_t *tof, lwDistanceResp_t *laser )
{
        tof->distance = (float32_t) laser->avg_dist;
        tof->timestamp = CP0_GET(CP0_COUNT) - ((laser->calcTime / LWNX_SAMP_PER_SCALAR) * CPU_TICKS_PER_SECOND);
}
/*-----------------------------------------------------------------------------
 *      estimatorComplementary:  Estimator Complementary
 *
 *  Parameters: state_t *state, sensorData_t *sensorData, secondStageTiming_t *tickLast,
 *              COMGS_quarternion *quartCalculated, selfState_s* SelfState
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void estimatorComplementary(state_t *state, sensorData_t *sensorData, secondStageTiming_t *tickLast, COMGS_quarternion *quartCalculated, selfState_s* SelfState, lwDistanceResp_t *laser)
{
  int32_t ticksToNow = 0LU;
  float32_t periodInSec = 0.0f;
  float32_t posUpdPeriodSec = 0.0f;

  if (quartCalculated->isEstComp == 0u)                                         /* has been initialised */
  {
     ticksToNow = -1;                                                           /* on first pass initialise the timer */
     calculateTick2Now( &ticksToNow, &tickLast->tickLast );                     /* ticksNow also initialised at zero */
     quartCalculated->isEstComp == 1u;                                          /* latch the first pass off */
  }
  else
  {
     calculateTick2Now( &ticksToNow, &tickLast->tickLast );                     /* calculate the ticks to now since last initialised */
  }
  periodInSec = ticksToNow / CPU_TICKS_PER_SECOND;                              /* calculate the period of this call in seconds */

  //---------------------------------------------------------------------------- sensorsAcquire(sensorData, tick);  Read sensors at full rate (1000Hz)
  if ( ticksToNow >= ATTITUDE_UPDATE_RATE )                                     /* we have waited until an other update period expired */
  {
    sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z, sensorData->acc.x, sensorData->acc.y, sensorData->acc.z, quartCalculated);
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);      // Save attitude, adjusted for the legacy CF2 body coordinate system
    // Save quaternion, hopefully one day this could be used in a better controller.
    // Note that this is not adjusted for the legacy coordinate system
    sensfusion6GetQuaternion( &state->attitudeQuaternion.x, &state->attitudeQuaternion.y, &state->attitudeQuaternion.z, &state->attitudeQuaternion.w);
    state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x, sensorData->acc.y, sensorData->acc.z);
    positionUpdateVelocity(state->acc.z, periodInSec, SelfState);
    tickLast->ticksAccum = (tickLast->ticksAccum + ticksToNow) % UINT32_MAX;    /* accumulate the overall timer with the time to now */
    calculateTickDiff( &ticksToNow, &tickLast->tickLast );                      /* tickLast becomes a new time so we re-initialise the counter here */
    tickLast->ticksAccum = (tickLast->ticksAccum + ticksToNow) % UINT32_MAX;    /* accumulate the overall timer with the time to do the maths in here */
  }

  if (tickLast->ticksAccum >= POS_UPDATE_RATE)                                  /* we have accumulated ticks over the position update period */
  {
    tofMeasurement_t tofMeasurement;
    readLwNxLaser( &tofMeasurement, laser );                                    /* TO DO make this accept various Liddar */
    posUpdPeriodSec = tickLast->ticksAccum / CPU_TICKS_PER_SECOND;              /* calculate the period of this call in seconds */
    positionEstimate(state, sensorData, &tofMeasurement, posUpdPeriodSec, SelfState);
    tickLast->ticksAccum = 0LU;                                                 /* reset the accumulator */
  }
}
/* ============ INDI Position Controller =======================================
 *
 * Copyright (c) 2019 Ewoud Smeur and Evghenii Volodscoi
 *
  * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * Ported to MikRoE C for Microchip PIC32 MCU By ACP Aviation
 *
 * This control algorithm is the Incremental Nonlinear Dynamic Inversion (INDI)
 * controller to control the position of the Crazyflie. It can be seen as an extension
 * (outer loop) to the already existing INDI attitude controller (inner loop).
 *
 * The control algorithm was implemented according to the publication in the
 * journal od Control Engineering Practice: Cascaded Incremental Nonlinear Dynamic
 * Inversion for MAV Disturbance Rejection
 * https://doi.org/10.1016/j.conengprac.2018.01.003
 */
/*-----------------------------------------------------------------------------
 *      initIndiOuterVariables:  initialise the indi outer variables
 *
 *  Parameters: IndiOuterVariables* indiOuter
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void initIndiOuterVariables( IndiOuterVariables* indiOuter )
{
   if ( indiOuter != NULL )
   {
     memset((void*)indiOuter,0u,sizeof(IndiOuterVariables));
     indiOuter->filt_cutoff = POSITION_INDI_FILT_CUTOFF;
     indiOuter->act_dyn_posINDI = STABILIZATION_INDI_ACT_DYN_P;
   }
   return;
}
/*-----------------------------------------------------------------------------
 *      position_indi_init_filters:  initialise the indi filters for position control
 *
 *  Parameters: posIndyTiming_t *timer
 *
 *  Return: int8_t 0 = initialised 1 = set
 *----------------------------------------------------------------------------*/
int8_t position_indi_init_filters(IndyTiming_t *timer)
{
   float32_t tau = 1.0f / (2.0f * PI * indiOuter.filt_cutoff);                  // tau = 1/(2*pi*Fc)
   float32_t tau_axis[3u];
   /* float32_t  sample_time = 1.0f / ATTITUDE_RATE; we calculate it rather than defining its frequency */
   int32_t ticksToNow=0;
   int8_t i;
   int8_t ret=0;
   
   for (i = 0; i < 3; i++)
   {
     tau_axis[i] = tau;
   }
   if (timer->IndiTimInit == 0u)                                                /* if it has not been initialised get a START point to measure it */
   {
     ticksToNow = -1;                                                           /* on first pass initialise the timer */
     calculateTick2Now( &ticksToNow, &timer->tickLast );                        /* ticksNow also initialised at zero */
     timer->IndiTimInit == 1u;                                                  /* latch the first pass off to do the filter on the next iteration */
   }
   else
   {
     calculateTickDiff( &ticksToNow, &timer->tickLast );                        /* calculate the ticks to now since last initialised and take a new time reference for next iteration if there is one */
     timer->sample_time = ticksToNow / CPU_TICKS_PER_SECOND;                    /* calculate the period of this call in seconds */
     for (i = 0; i < 3; i++)                                                    /* Filtering of linear acceleration, attitude and thrust */
     {
       init_butterworth_2_low_pass(&indiOuter.ddxi[i], tau_axis[i], timer->sample_time, 0.0f);
       init_butterworth_2_low_pass(&indiOuter.ang[i], tau_axis[i], timer->sample_time, 0.0f);
       init_butterworth_2_low_pass(&indiOuter.thr[i], tau_axis[i], timer->sample_time, 0.0f);
     }
     ret = 1;
   }
   return ret;
}
/*-----------------------------------------------------------------------------
 *      filter_ddxi:  Linear acceleration filter
 *
 *  Parameters: Butterworth2LowPass *filter, Vectr *old_values, Vectr *new_values
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void filter_ddxi(Butterworth2LowPass *filter, Vectr *old_values, Vectr *new_values)
{
   if (((filter != NULL) && (old_values != NULL)) && (new_values != NULL))
   {
        new_values->x = update_butterworth_2_low_pass(&filter[0u], old_values->x);
        new_values->y = update_butterworth_2_low_pass(&filter[1u], old_values->y);
        new_values->z = update_butterworth_2_low_pass(&filter[2u], old_values->z);
   }
}
/*-----------------------------------------------------------------------------
 *      filter_ang:  Attitude filter
 *
 *  Parameters: Butterworth2LowPass *filter, Angles *old_values, Angles *new_values
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void filter_ang(Butterworth2LowPass *filter, Angles *old_values, Angles *new_values)
{
   if (((filter != NULL) && (old_values != NULL)) && (new_values != NULL))
   {
        new_values->phi = update_butterworth_2_low_pass(&filter[0u], old_values->phi);
        new_values->theta = update_butterworth_2_low_pass(&filter[1u], old_values->theta);
        new_values->psi = update_butterworth_2_low_pass(&filter[2u], old_values->psi);
   }
}
/*-----------------------------------------------------------------------------
 *      filter_thrust:  Thrust filter
 *
 *  Parameters: Butterworth2LowPass *filter, float32_t *old_thrust, float32_t *new_thrust
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void filter_thrust(Butterworth2LowPass *filter, float32_t *old_thrust, float32_t *new_thrust)
{
   if (((filter != NULL) && (old_thrust != NULL)) && (new_thrust != NULL))
   {
        *new_thrust = update_butterworth_2_low_pass(&filter[0u], *old_thrust);
   }
}
/*-----------------------------------------------------------------------------
 *      m_ob:  Computes transformation matrix from body frame (index B) into NED frame (index O)
 *
 *  Parameters: const Angles *att, float32_t *matrix[3u][3u]
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void m_ob(const Angles *att, float32_t matrix[3u][3u])
{
   matrix[0u][0u] = cos(att->theta)*cos(att->psi);
   matrix[0u][1u] = sin(att->phi)*sin(att->theta)*cos(att->psi) - cos(att->phi)*sin(att->psi);
   matrix[0u][2u] = cos(att->phi)*sin(att->theta)*cos(att->psi) + sin(att->phi)*sin(att->psi);
   matrix[1u][0u] = cos(att->theta)*sin(att->psi);
   matrix[1u][1u] = sin(att->phi)*sin(att->theta)*sin(att->psi) + cos(att->phi)*cos(att->psi);
   matrix[1u][2u] = cos(att->phi)*sin(att->theta)*sin(att->psi) - sin(att->phi)*cos(att->psi);
   matrix[2u][0u] = -sin(att->theta);
   matrix[2u][1u] = sin(att->phi)*cos(att->theta);
   matrix[2u][2u] = cos(att->phi)*cos(att->theta);
}
/*-----------------------------------------------------------------------------
 *      positionControllerINDI:  INDI Position Controller
 *
 *  Parameters: const sensorData_t *sensors, setpoint_t *setpoint, 
 *              const state_t *state, Vectr *refOuterINDI
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void positionControllerINDI(const sensorData_t *sensors, setpoint_t *setpoint, const state_t *state, Vectr *refOuterINDI)
{

    float32_t M_OB[3u][3u] = {0};
    float32_t g11,g12,g13,g21,g22,g23,g31,g32,g33;
    float32_t a11,a12,a13,a21,a22,a23,a31,a32,a33;
    float32_t detG;
    float32_t g11_inv,g12_inv,g13_inv,g21_inv,g22_inv,g23_inv,g31_inv,g32_inv,g33_inv;
    float32_t a11_inv,a12_inv,a13_inv,a21_inv,a22_inv,a23_inv,a31_inv,a32_inv,a33_inv;
    Angles att;

    posS_x = state->position.x;                                                 // Read states (position, velocity)
    posS_y = -state->position.y;
    posS_z = -state->position.z;
    velS_x = state->velocity.x;
    velS_y = -state->velocity.y;
    velS_z = -state->velocity.z;
    gyr_p = sensors->gyro.x;
    gyr_q = sensors->gyro.y;
    gyr_r = sensors->gyro.z;

    velocityRef.x = setpoint->velocity.x;                                       // Read in velocity setpoints
    velocityRef.y = -setpoint->velocity.y;
    velocityRef.z = -setpoint->velocity.z;

    if (setpoint->mode.x == modeAbs)                                            // Position controller (Proportional)
    {
        positionRef.x = setpoint->position.x;
        velocityRef.x = K_xi_x*(positionRef.x - posS_x);
    }
    if (setpoint->mode.y == modeAbs)
    {
        positionRef.y = -setpoint->position.y;
        velocityRef.y = K_xi_y*(positionRef.y - posS_y);
    }
    if (setpoint->mode.z == modeAbs) 
    {
        positionRef.z = -setpoint->position.z;
        velocityRef.z = K_xi_z*(positionRef.z - posS_z);
    }

    indiOuter.linear_accel_ref.x = K_dxi_x*(velocityRef.x - velS_x);            // Velocity controller (Proportional)
    indiOuter.linear_accel_ref.y = K_dxi_y*(velocityRef.y - velS_y);
    indiOuter.linear_accel_ref.z = K_dxi_z*(velocityRef.z - velS_z);

    // Acceleration controller (INDI)
    indiOuter.linear_accel_s.x = (sensors->acc.x)*9.81f;                        // Read lin. acceleration (Body-fixed) obtained from sensors CHECKED
    indiOuter.linear_accel_s.y = (-sensors->acc.y)*9.81f;
    indiOuter.linear_accel_s.z = (-sensors->acc.z)*9.81f;

    filter_ddxi(indiOuter.ddxi, &indiOuter.linear_accel_s, &indiOuter.linear_accel_f);      // Filter lin. acceleration

    indiOuter.attitude_s.phi = state->attitude.roll;                            // Obtain actual attitude values (in deg)
    indiOuter.attitude_s.theta = state->attitude.pitch;
    indiOuter.attitude_s.psi = -state->attitude.yaw;
    filter_ang(indiOuter.ang, &indiOuter.attitude_s, &indiOuter.attitude_f);

    att.phi = indiOuter.attitude_f.phi/180.0f*PI,                               // Actual attitude (in rad)
    att.theta = indiOuter.attitude_f.theta/180.0f*PI,
    att.psi = indiOuter.attitude_f.psi/180.0f*PI,

    m_ob(&att, M_OB);                                                           // Compute transformation matrix from body frame (index B) into NED frame (index O)

    indiOuter.linear_accel_ft.x = M_OB[0u][0u]*indiOuter.linear_accel_f.x + M_OB[0u][1u]*indiOuter.linear_accel_f.y + M_OB[0u][2u]*indiOuter.linear_accel_f.z;      // Transform lin. acceleration in NED (add gravity to the z-component)
    indiOuter.linear_accel_ft.y = M_OB[1u][0u]*indiOuter.linear_accel_f.x + M_OB[1u][1u]*indiOuter.linear_accel_f.y + M_OB[1u][2u]*indiOuter.linear_accel_f.z;
    indiOuter.linear_accel_ft.z = M_OB[2u][0u]*indiOuter.linear_accel_f.x + M_OB[2u][1u]*indiOuter.linear_accel_f.y + M_OB[2u][2u]*indiOuter.linear_accel_f.z + 9.81f;

    indiOuter.linear_accel_err.x = indiOuter.linear_accel_ref.x - indiOuter.linear_accel_ft.x;     // Compute lin. acceleration error
    indiOuter.linear_accel_err.y = indiOuter.linear_accel_ref.y - indiOuter.linear_accel_ft.y;
    indiOuter.linear_accel_err.z = indiOuter.linear_accel_ref.z - indiOuter.linear_accel_ft.z;

    // Elements of the G matrix (see publication for more information)
    // ("-" because T points in neg. z-direction, "*9.81" GRAV_CONST because T/m=a=g,
    // negative psi to account for wrong coordinate frame in the implementation of the inner loop)
    g11 = (cos(att.phi)*sin(-att.psi) - sin(att.phi)*sin(att.theta)*cos(-att.psi))*(-GRAV_CONST);
    g12 = (cos(att.phi)*cos(-att.theta)*cos(-att.psi))*(-GRAV_CONST);
    g13 = (sin(att.phi)*sin(-att.psi) + cos(att.phi)*sin(att.theta)*cos(-att.psi));
    g21 = (-cos(att.phi)*cos(-att.psi) - sin(att.phi)*sin(att.theta)*sin(-att.psi))*(-GRAV_CONST);
    g22 = (cos(att.phi)*cos(att.theta)*sin(-att.psi))*(-GRAV_CONST);
    g23 = (-sin(att.phi)*cos(-att.psi) + cos(att.phi)*sin(att.theta)*sin(-att.psi));
    g31 = (-sin(att.phi)*cos(att.theta))*(-GRAV_CONST);
    g32 = (-cos(att.phi)*sin(att.theta))*(-GRAV_CONST);
    g33 = (cos(att.phi)*cos(att.theta));

    a11 = g11*g11 + g21*g21 + g31*g31;                                          // Next four blocks of the code are to compute the Moore-Penrose inverse of the G matrix (G'*G)
    a12 = g11*g12 + g21*g22 + g31*g32;
    a13 = g11*g13 + g21*g23 + g31*g33;
    a21 = g12*g11 + g22*g21 + g32*g31;
    a22 = g12*g12 + g22*g22 + g32*g32;
    a23 = g12*g13 + g22*g23 + g32*g33;
    a31 = g13*g11 + g23*g21 + g33*g31;
    a32 = g13*g12 + g23*g22 + g33*g32;
    a33 = g13*g13 + g23*g23 + g33*g33;

    detG = (a11*a22*a33 + a12*a23*a31 + a21*a32*a13) - (a13*a22*a31 + a11*a32*a23 + a12*a21*a33);     // Determinant of (G'*G)

    a11_inv = (a22*a33 - a23*a32)/detG;                                         // Inverse of (G'*G)
    a12_inv = (a13*a32 - a12*a33)/detG;
    a13_inv = (a12*a23 - a13*a22)/detG;
    a21_inv = (a23*a31 - a21*a33)/detG;
    a22_inv = (a11*a33 - a13*a31)/detG;
    a23_inv = (a13*a21 - a11*a23)/detG;
    a31_inv = (a21*a32 - a22*a31)/detG;
    a32_inv = (a12*a31 - a11*a32)/detG;
    a33_inv = (a11*a22 - a12*a21)/detG;

    g11_inv = a11_inv*g11 + a12_inv*g12 + a13_inv*g13;                          // G_inv = (G'*G)_inv*G'
    g12_inv = a11_inv*g21 + a12_inv*g22 + a13_inv*g23;
    g13_inv = a11_inv*g31 + a12_inv*g32 + a13_inv*g33;
    g21_inv = a21_inv*g11 + a22_inv*g12 + a23_inv*g13;
    g22_inv = a21_inv*g21 + a22_inv*g22 + a23_inv*g23;
    g23_inv = a21_inv*g31 + a22_inv*g32 + a23_inv*g33;
    g31_inv = a31_inv*g11 + a32_inv*g12 + a33_inv*g13;
    g32_inv = a31_inv*g21 + a32_inv*g22 + a33_inv*g23;
    g33_inv = a31_inv*g31 + a32_inv*g32 + a33_inv*g33;

    indiOuter.phi_tilde = (g11_inv*indiOuter.linear_accel_err.x + g12_inv*indiOuter.linear_accel_err.y + g13_inv*indiOuter.linear_accel_err.z);     // Lin. accel. error multiplied  G^(-1) matrix (T_tilde negated because motor accepts only positiv commands, angles are in rad)
    indiOuter.theta_tilde = (g21_inv*indiOuter.linear_accel_err.x + g22_inv*indiOuter.linear_accel_err.y + g23_inv*indiOuter.linear_accel_err.z);
    indiOuter.T_tilde = -(g31_inv*indiOuter.linear_accel_err.x + g32_inv*indiOuter.linear_accel_err.y + g33_inv*indiOuter.linear_accel_err.z)/K_thr;
    
    filter_thrust(indiOuter.thr, &indiOuter.T_incremented, &indiOuter.T_inner_f);       // Filter thrust
    
    indiOuter.T_inner = indiOuter.T_inner + indiOuter.act_dyn_posINDI*(indiOuter.T_inner_f - indiOuter.T_inner);     // Pass thrust through the model of the actuator dynamics
    
    indiOuter.T_incremented = indiOuter.T_tilde + indiOuter.T_inner;            // Compute trust that goes into the inner loop
    
    indiOuter.attitude_c.phi = indiOuter.attitude_f.phi + indiOuter.phi_tilde*180.0f/PI;     // Compute commanded attitude to the inner INDI
    indiOuter.attitude_c.theta = indiOuter.attitude_f.theta + indiOuter.theta_tilde*180.0f/PI;
    
    indiOuter.T_incremented = clampReal(indiOuter.T_incremented, MIN_THRUST, MAX_THRUST);      // Clamp commands
    indiOuter.attitude_c.phi = clampReal(indiOuter.attitude_c.phi, -10.0f, 10.0f);
    indiOuter.attitude_c.theta = clampReal(indiOuter.attitude_c.theta, -10.0f, 10.0f);
    
    refOuterINDI->x = indiOuter.attitude_c.phi;                                 // Reference values, which are passed to the inner loop INDI (attitude controller)
    refOuterINDI->y = indiOuter.attitude_c.theta;
    refOuterINDI->z = indiOuter.T_incremented;

}
/* ================= INDI Attitude Controler ===================================
 *                   As per above position controller
 *
 * Copyright (c) 2019 Ewoud Smeur and Andre Luis Ogando Paraense
 * Ported for mikRo E C for microchip PIC32 by ACP Aviation
 *
 * ========================================================================== */
/*-----------------------------------------------------------------------------
 *      initIndiInnerVariables:  initialise the indi inner variables
 *
 *  Parameters: IndiVariables* indiOuter
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void initIndiInnerVariables( IndiVariables* indi )
{
   if ( indi != NULL )
   {
     memset((void*)indi,0u,sizeof(IndiVariables));
     indi->g1.p = STABILIZATION_INDI_G1_P;
     indi->g1.q = STABILIZATION_INDI_G1_Q;
     indi->g1.r = STABILIZATION_INDI_G1_R;
     indi->g2 = STABILIZATION_INDI_G2_R;
     indi->reference_acceleration.err_p = STABILIZATION_INDI_REF_ERR_P;
     indi->reference_acceleration.err_q = STABILIZATION_INDI_REF_ERR_Q;
     indi->reference_acceleration.err_r = STABILIZATION_INDI_REF_ERR_R;
     indi->reference_acceleration.rate_p = STABILIZATION_INDI_REF_RATE_P;
     indi->reference_acceleration.rate_q = STABILIZATION_INDI_REF_RATE_Q;
     indi->reference_acceleration.rate_r = STABILIZATION_INDI_REF_RATE_R;
     indi->act_dyn.p = STABILIZATION_INDI_ACT_DYN_P;
     indi->act_dyn.q = STABILIZATION_INDI_ACT_DYN_Q;
     indi->act_dyn.r = STABILIZATION_INDI_ACT_DYN_R;
     indi->filt_cutoff = STABILIZATION_INDI_FILT_CUTOFF;
     indi->filt_cutoff_r = STABILIZATION_INDI_FILT_CUTOFF_R;
   }
   return;
}
/*-----------------------------------------------------------------------------
 *      float_rates_zero:  set rates to zero
 *
 *  Parameters: FloatRates *fr
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void float_rates_zero(FloatRates *fr)
{
        fr->p = 0.0f;
        fr->q = 0.0f;
        fr->r = 0.0f;
}
/*-----------------------------------------------------------------------------
 *      indi_init_filters:  Initialise the Indy filters
 *
 *  Parameters: IndyTiming_t *timer
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t indi_init_filters(IndyTiming_t *timer)
{
        float32_t tau = 1.0f / (2.0f * PI * indi.filt_cutoff);                  // tau = 1/(2*pi*Fc)
        float32_t tau_r = 1.0f / (2.0f * PI * indi.filt_cutoff_r);
        float32_t tau_axis[3u];
        /* float32_t sample_time = 1.0f / ATTITUDE_RATE; now calculated each step */
        int32_t ticksToNow = 0;
        int8_t ret = 0;
        int8_t i;
        
        tau_axis[0u] = tau;
        tau_axis[1u] = tau;
        tau_axis[2u] = tau_r;
        if (timer->IndiTimInit == 0u)                                           /* if it has not been initialised get a START point to measure it */
        {
            ticksToNow = -1;                                                    /* on first pass initialise the timer */
            calculateTick2Now( &ticksToNow, &timer->tickLast );                 /* ticksNow also initialised at zero */
            timer->IndiTimInit == 1u;                                           /* latch the first pass off to do the filter on the next iteration */
        }
        else
        {
           calculateTick2Now( &ticksToNow, &timer->tickLast );                  /* calculate the ticks to now since last initialised Change to Diff if you want to use 2 separate ones rather than the chain to position_indi_init_filters */
           timer->sample_time = ticksToNow / CPU_TICKS_PER_SECOND;              /* calculate the period of this call in seconds */
           for (i = 0; i < 3; i++)                                              /* Filtering of gyroscope and actuators */
           {
              init_butterworth_2_low_pass(&indi.u[i], tau_axis[i], timer->sample_time, 0.0f);
              init_butterworth_2_low_pass(&indi.rate[i], tau_axis[i], timer->sample_time, 0.0f);
           }
        }
        return ret;
}
/**
 * @brief Update butterworth filter for p, q and r of a FloatRates struct
 *
 * @param filter The filter array to use
 * @param new_values The new values
 */
/*-----------------------------------------------------------------------------
 *      filter_pqr : Update butterworth filter for p, q and r of a FloatRates struct
 *                             
 *  Parameters: Butterworth2LowPass *filter, FloatRates *new_values
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void filter_pqr(Butterworth2LowPass *filter, FloatRates *new_values)
{
        update_butterworth_2_low_pass(&filter[0u], new_values->p);
        update_butterworth_2_low_pass(&filter[1u], new_values->q);
        update_butterworth_2_low_pass(&filter[2u], new_values->r);
}

/**
 * @brief Caclulate finite difference form a filter array
 * The filter already contains the previous values
 *
 * @param output The output array
 * @param filter The filter array input
 */
/*-----------------------------------------------------------------------------
 *      finite_difference_from_filter : finite difference filter
 *                             
 *  Parameters: float32_t *output, Butterworth2LowPass *filter, IndyTiming_t *timer
 *
 *  Return:     int8_t
 *----------------------------------------------------------------------------*/
int8_t finite_difference_from_filter(float32_t *output, Butterworth2LowPass *filter, IndyTiming_t *timer)
{
     int8_t i;
     int8_t ret=0;
     int32_t ticksToNow = 0;
     
     if (timer->IndiFinInit == 0u)                                              /* if it has not been initialised get a START point to measure it */
     {
        ticksToNow = -1;                                                        /* on first pass initialise the timer */
        calculateTick2Now( &ticksToNow, &timer->tickLast1 );                     /* ticksNow also initialised at zero */
        timer->IndiFinInit == 1u;                                               /* latch the first pass off to do the filter on the next iteration */
     }
     else
     {
        calculateTick2Now( &ticksToNow, &timer->tickLast1 );                    /* calculate the ticks to now since last initialised */
        timer->sample_time = ticksToNow / CPU_TICKS_PER_SECOND;                 /* calculate the period of this call in seconds */
        for (i = 0; i < 3; i++)
        {
           output[i] = (filter[i].o[0u] - filter[i].o[1u]) * timer->sample_time;
        }
        ret = 1;
     }
     return ret;
}
/*-----------------------------------------------------------------------------
 *      controllerINDIInit:  Initialise the Indy attitude controller
 *                           Assumes filter time shoulbe from time iterated in each call
 *                           of the function if you want the time between each measurement
 *                           tag the tickLast value for IndyTiming_t in that function instead
 *
 *  Parameters: IndyTiming_t *timer
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t controllerINDIInit(IndyTiming_t *timer)
{
    int8_t ret=0;
    int32_t ticksToNow = 0;
    /*
         * TODO
         * Can this also be called during flight, for instance when switching controllers?
         * Then the filters should not be reset to zero but to the current values of sensors and actuators.
         * ACP : Done this as below but not tested it
    */
    if (timer->IndiInFlight == 0u)
    {
           float_rates_zero(&indi.angular_accel_ref);
           float_rates_zero(&indi.u_act_dyn);
           float_rates_zero(&indi.u_in);
           ret = indi_init_filters(timer);          // Re-initialize filters
           if (ret == 1)
           {
/*   ========= TO DO FIND AND IMPLEMENT THIS !!!! ==============
           attitudeControllerInit(ATTITUDE_UPDATE_DT);
           positionControllerInit();
*/
              ret = position_indi_init_filters(timer);
              if (ret == 1u) timer->IndiInFlight = 1u;
              timer->IndiPosInit = 0;
           }
    }
    else    /* This can now be called during flight */
    {
           if (timer->IndiPosInit == 0u)                                        /* we only initialised now we repeated */
           {
              ticksToNow = -1;                                                  /* initialise the timer for the next function iteration */
              calculateTick2Now( &ticksToNow, &timer->tickLast );               /* ticksNow also initialised at zero */
              timer->IndiPosInit == 1u;
           }
           else                                                                 /* you can now keep iterating this to reset take timer->IndiPosInit off in the caller function */
           {
              ret = indi_init_filters(timer);                                          /* Re-initialize filters */
              if (ret == 1)
              {
/*   ========= TO DO FIND AND IMPLEMENT THIS !!!! ==============
           attitudeControllerInit(ATTITUDE_UPDATE_DT);
           positionControllerInit();
*/
                  ret = position_indi_init_filters(timer);                      /* you could toggle timer->IndiPosInit for a double period if you wanted */
              }
           }
    }
    return ret;
}
/*-----------------------------------------------------------------------------
 *      capAngle:  range the angle
 *
 *  Parameters: float32_t angle
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/
float32_t capAngle(float32_t angle)
{
  float32_t result = angle;

  while (result > 180.0f) 
  {
    result -= 360.0f;
  }
  while (result < -180.0f)
  {
    result += 360.0f;
  }
  return result;
}
/*-----------------------------------------------------------------------------
 *      controllerINDI:  INDI attitude controller
 *
 *  Parameters: control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, 
 *              const state_t *state, IndyTiming_t *timer
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void controllerINDI(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, IndyTiming_t *timer)
{

    float32_t stateAttitudeRateRoll;
    float32_t stateAttitudeRatePitch;
    float32_t stateAttitudeRateYaw;
    float32_t attitude_error_p;
    float32_t attitude_error_q;
    float32_t attitude_error_r;
    FloatRates body_rates;
    int32_t ticksToNow = 0;

    if (timer->indiContLoopTInit == 0u)                                         /* has been initialised */
    {
       ticksToNow = -1;                                                         /* on first pass initialise the timer */
       calculateTick2Now( &ticksToNow, &timer->indiContLoopTRef );              /* ticksNow also initialised at zero */
       timer->indiContLoopTInit == 1u;                                          /* latch the first pass off */
    }
    else
    {
       calculateTick2Now( &ticksToNow, &timer->indiContLoopTRef );              /* calculate the ticks to now since last initialised */
    }
  
    if ( ticksToNow >= ATTITUDE_UPDATE_RATE )                                   /* we have waited until an other update period expired */
    {
        if (setpoint->mode.yaw == modeVelocity)                                 /* Rate-controled YAW is moving YAW angle setpoint   */
        {
             attitudeDesired.yaw += setpoint->attitudeRate.yaw * ticksToNow;
             while (attitudeDesired.yaw > 180.0f)
                attitudeDesired.yaw -= 360.0f;
             while (attitudeDesired.yaw < -180.0f)
                attitudeDesired.yaw += 360.0f;
         }
         else
         {
             attitudeDesired.yaw = setpoint->attitude.yaw;
         }
         timer->ticksAccum = (timer->ticksAccum + ticksToNow) % UINT32_MAX;     /* accumulate the overall timer with the time to now */
    }

    if ((timer->ticksAccum >= POS_UPDATE_RATE) && !(timer->outerLoopActive))    /* we have accumulated ticks over the position update period */
    {
         /* TO FIND THIS positionController(&actuatorThrust, &attitudeDesired, setpoint, state); */
         timer->ticksAccum = 0lu;
    }

    if ( ticksToNow >= ATTITUDE_UPDATE_RATE )                                   /* we have waited until an other update period expired */
    {
         if (timer->outerLoopActive)                                            // Call outer loop INDI (position controller)
         {
            positionControllerINDI(sensors, setpoint, state, &refOuterINDI);
         }
         if (setpoint->mode.z == modeDisable)                                   // Switch between manual and automatic position control
         {
             actuatorThrust = setpoint->thrust;                                 // INDI position controller not active, INDI attitude controller is main loop
         }
         else
         {
             if (timer->outerLoopActive)
             {
                actuatorThrust = refOuterINDI.z;                                // INDI position controller active, INDI attitude controller becomes inner loop
             }
         }
         if (setpoint->mode.x == modeDisable)
         {
             attitudeDesired.roll = setpoint->attitude.roll;                    // INDI position controller not active, INDI attitude controller is main loop
         }
         else
         {
             if (timer->outerLoopActive)
             {
                 attitudeDesired.roll = refOuterINDI.x;                         // INDI position controller active, INDI attitude controller becomes inner loop
             }
         }
         if (setpoint->mode.y == modeDisable)
         {
             attitudeDesired.pitch = setpoint->attitude.pitch;                  // INDI position controller not active, INDI attitude controller is main loop
         }
         else
         {
              if (timer->outerLoopActive)
              {
                   attitudeDesired.pitch = refOuterINDI.y;                      // INDI position controller active, INDI attitude controller becomes inner loop
              }
         }
//            attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
//            attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
//            &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);
         rateDesired.roll = roll_kp*(attitudeDesired.roll - state->attitude.roll);
         rateDesired.pitch = pitch_kp*(attitudeDesired.pitch - state->attitude.pitch);

         attYawError = attitudeDesired.yaw - state->attitude.yaw;               //rateDesired.yaw = yaw_kp*(attitudeDesired.yaw - state->attitude.yaw);
         attYawError = capAngle(attYawError);
         rateDesired.yaw = yaw_kp*attYawError;

         // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
         // value. Also reset the PID to avoid error buildup, which can lead to unstable
         // behavior if level mode is engaged later
         if (setpoint->mode.roll == modeVelocity)
         {
             rateDesired.roll = setpoint->attitudeRate.roll;
/* TODO                        attitudeControllerResetRollAttitudePID();                        */
         }
         if (setpoint->mode.pitch == modeVelocity)
         {
             rateDesired.pitch = setpoint->attitudeRate.pitch;
/* TODO                        attitudeControllerResetPitchAttitudePID();                       */
         }

         /*
         * 1 - Update the gyro filter with the new measurements.
         */
         stateAttitudeRateRoll = DEGREE_TO_RADIAN(sensors->gyro.x);
         stateAttitudeRatePitch = -DEGREE_TO_RADIAN(sensors->gyro.y); // Account for Crazyflie coordinate system
         stateAttitudeRateYaw = DEGREE_TO_RADIAN(sensors->gyro.z);
         body_rates.p = stateAttitudeRateRoll,
         body_rates.q = stateAttitudeRatePitch,
         body_rates.r = stateAttitudeRateYaw,
         filter_pqr(indi.rate, &body_rates);
         /*
         * 2 - Calculate the derivative with finite difference.
         */
         finite_difference_from_filter(indi.rate_d, indi.rate, timer);
         /*
         * 3 - same filter on the actuators (or control_t values), using the commands from the previous timestep.
         */
         filter_pqr(indi.u, &indi.u_act_dyn);
         /*
          * 4 - Calculate the desired angular acceleration by:
          * 4.1 - Rate_reference = P * attitude_error, where attitude error can be calculated with your favorite
          * algorithm. You may even use a function that is already there, such as attitudeControllerCorrectAttitudePID(),
          * though this will be inaccurate for large attitude errors, but it will be ok for now.
          * 4.2 Angular_acceleration_reference = D * (rate_reference – rate_measurement)
          */
          attitude_error_p = DEGREE_TO_RADIAN(rateDesired.roll) - stateAttitudeRateRoll;
          attitude_error_q = DEGREE_TO_RADIAN(rateDesired.pitch) - stateAttitudeRatePitch;
          attitude_error_r = DEGREE_TO_RADIAN(rateDesired.yaw) - stateAttitudeRateYaw;
          indi.angular_accel_ref.p = indi.reference_acceleration.err_p * attitude_error_p;
          indi.angular_accel_ref.q = indi.reference_acceleration.err_q * attitude_error_q;
          indi.angular_accel_ref.r = indi.reference_acceleration.err_r * attitude_error_r;
          /*
          * 5. Update the For each axis: delta_command = 1/control_effectiveness * (angular_acceleration_reference – angular_acceleration)
          */
          //Increment in angular acceleration requires increment in control input
          //G1 is the control effectiveness. In the yaw axis, we need something additional: G2.
          //It takes care of the angular acceleration caused by the change in rotation rate of the propellers
          //(they have significant inertia, see the paper mentioned in the header for more explanation)
          indi.du.p = 1.0f / indi.g1.p * (indi.angular_accel_ref.p - indi.rate_d[0u]);
          indi.du.q = 1.0f / indi.g1.q * (indi.angular_accel_ref.q - indi.rate_d[1u]);
          indi.du.r = 1.0f / (indi.g1.r - indi.g2) * (indi.angular_accel_ref.r - indi.rate_d[2] - indi.g2 * indi.du.r);
          /*
          * 6. Add delta_commands to commands and bound to allowable values
          */
          indi.u_in.p = indi.u[0u].o[0u] + indi.du.p;
          indi.u_in.q = indi.u[1u].o[0u] + indi.du.q;
          indi.u_in.r = indi.u[2u].o[0u] + indi.du.r;
           
          indi.u_in.p = clampReal(indi.u_in.p, -1.0f*bound_control_input, bound_control_input);                 //bound the total control input
          indi.u_in.q = clampReal(indi.u_in.q, -1.0f*bound_control_input, bound_control_input);
          indi.u_in.r = clampReal(indi.u_in.r, -1.0f*bound_control_input, bound_control_input);

          indi.u_act_dyn.p = indi.u_act_dyn.p + indi.act_dyn.p * (indi.u_in.p - indi.u_act_dyn.p);                 //Propagate input filters first order actuator dynamics
          indi.u_act_dyn.q = indi.u_act_dyn.q + indi.act_dyn.q * (indi.u_in.q - indi.u_act_dyn.q);
          indi.u_act_dyn.r = indi.u_act_dyn.r + indi.act_dyn.r * (indi.u_in.r - indi.u_act_dyn.r);
          calculateTickDiff( &ticksToNow, &timer->indiContLoopTRef  );          /* timer becomes a new time so we re-initialise the counter here */
          timer->ticksAccum = (timer->ticksAccum + ticksToNow) % UINT32_MAX;    /* accumulate the overall timer with the time to do the maths in here */
    }

    indi.thrust = actuatorThrust;
    r_roll = DEGREE_TO_RADIAN(sensors->gyro.x);
    r_pitch = -DEGREE_TO_RADIAN(sensors->gyro.y);
    r_yaw = DEGREE_TO_RADIAN(sensors->gyro.z);
    accelz = sensors->acc.z;

    //Don't increment if thrust is off
    //TODO: this should be something more elegant, but without this the inputs
    //will increment to the maximum before even getting in the air.
    if(indi.thrust < thrust_threshold)
    {
         float_rates_zero(&indi.angular_accel_ref);
         float_rates_zero(&indi.u_act_dyn);
         float_rates_zero(&indi.u_in);
         if(indi.thrust == 0.0f)
         {
/* ====================== TODO
              attitudeControllerResetAllPID();
              positionControllerResetAllPID();    */
              attitudeDesired.yaw = state->attitude.yaw;                         // Reset the calculated YAW angle for rate control
          }
    }
    control->thrust = indi.thrust;                                              /*  INDI feedback */
    control->roll = indi.u_in.p;
    control->pitch = indi.u_in.q;
    control->yaw  = indi.u_in.r;
}
/* ================= Helper Functions ======================================= */
// Function that normalizes a conformal vector
// Created by Aksel Sveier. Spring 2016
float64_t * sphere_vector_normalize(float64_t *dSph[5u])
{
    float64_t ret[5u];
    int8_t i;
    
    for(i = 0; i<5 ;i++)
    {
        ret[i] = *dSph[i] / *dSph[3u];
    }
    return &ret;
}
quat * sphere_vector_normalizeVec(quat *dSph)
{
    quat ret;
    ret.x = dSph->x / dSph->w;
    ret.y = dSph->y / dSph->w;
    ret.z = dSph->z / dSph->w;
    ret.w = dSph->w / dSph->w;
    return &ret;
}
//Function that calculates radius
float64_t sphere_calcRadius(float64_t *sph[5u])
{
    return sqrt(((1.0f / pow(*sph[3u],2.0f))*(pow(*sph[0u],2.0f)+pow(*sph[1u],2.0f)+pow(*sph[2u],2.0f)))-(2.0f*(*sph[4u] / *sph[3u])));  
}
//Function that normalizes a conformal vector
float64_t * plane_normalize(float64_t *dPln[5u])
{
    float64_t normDualPlane[5u];
    int8_t i;
    
    for(i = 0; i < 5; i++)
    {
        normDualPlane[i] = *dPln[i]/sqrt(pow(*dPln[0u],2.0f)+pow(*dPln[1u],2.0f)+pow(*dPln[2u],2.0f));
    }
    return &normDualPlane;   
}

/* ================= Mellanger Controler ==================================== */
/*
The MIT License (MIT)
Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

This controller is based on the following publication:

Daniel Mellinger, Vijay Kumar:
Minimum snap trajectory generation and control for quadrotors.
IEEE International Conference on Robotics and Automation (ICRA), 2011.

We added the following:
 * Integral terms (compensates for: battery voltage drop over time, unbalanced center of mass due to asymmmetries, and uneven wear on propellers and motors)
 * D-term for angular velocity
 * Support to use this controller as an attitude-only controller for manual flight
*/
/* ---------------- Math Lib BY : James Alan Preiss------------------------- */
// 
 
/*-----------------------------------------------------------------------------
 *      mkquat : construct a quaternion from its x, y, z, w elements..
 *                             
 *  Parameters: float32_t x, float32_t y, float32_t z, float32_t w
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat mkquat(float32_t x, float32_t y, float32_t z, float32_t w) 
{
        quat q;
        q.x = x; q.y = y; q.z = z; q.w = w;
        return q;
} 
/*-----------------------------------------------------------------------------
 *      quat2rotmat : convert a quaternion into a 3x3 rotation matrix.
 *                             
 *  Parameters: quat q
 *
 *  Return:     mat33
 *----------------------------------------------------------------------------*/
mat33 quat2rotmat(quat q)
{
        // from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        float32_t x = q.x;
        float32_t y = q.y;
        float32_t z = q.z;
        float32_t w = q.w;
        mat33 m;
        
        m.m[0u][0u] = 1.0f - 2.0f*y*y - 2.0f*z*z;
        m.m[0u][1u] = 2.0f*x*y - 2.0f*z*w;
        m.m[0u][2u] = 2.0f*x*z + 2.0f*y*w,
        m.m[1u][0u] = 2.0f*x*y + 2.0f*z*w;
        m.m[1u][1u] = 1.0f - 2.0f*x*x - 2.0f*z*z;
        m.m[1u][2u] = 2.0f*y*z - 2.0f*x*w,
        m.m[2u][0u] = 2.0f*x*z - 2.0f*y*w;
        m.m[2u][1u] = 2.0f*y*z + 2.0f*x*w;
        m.m[2u][2u] = 1.0f - 2.0f*x*x - 2.0f*y*y;
        return m;
}
// convert quaternion to (roll, pitch, yaw) Euler angles using Tait-Bryan convention
// (yaw, then pitch about new pitch axis, then roll about new roll axis)
/*-----------------------------------------------------------------------------
 *      quat2rpy : convert quaternion to (roll, pitch, yaw) Euler angles using Tait-Bryan convention
 *                             
 *  Parameters: quat q
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr quat2rpy(quat q) 
{
        // from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        Vectr v;
        v.x = atan2(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (sqrt(q.x) + sqrt(q.y)));      // roll ======== CHECK @@_fsqr is sqrt        !! ====
        v.y = asin(2.0f * (q.w * q.y - q.x * q.z));                             // pitch
        v.z = atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (sqrt(q.y) + sqrt(q.z))); // yaw
        return v;
} 
/*-----------------------------------------------------------------------------
 *      vscl : multiply a vector by a scalar.
 *                             
 *  Parameters: float32_t s, Vectr v
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vscl(float32_t s, Vectr v) 
{
        return mkvec(s * v.x , s * v.y, s * v.z);
}
/*-----------------------------------------------------------------------------
 *      vdiv : divide a vector by a scalar.
 *                             
 *  Parameters: Vectr a, Vectr b
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vdiv(Vectr v, float32_t s)
{
   //return vscl(1.0f/s, v);   below for speed
   if (s == 0.0f) 
   {
     return v;
   }
   else
   {
      return mkvec((1.0f/s) * v.x , (1.0f/s) * v.y, (1.0f/s) * v.z);
   }
}
 

/*-----------------------------------------------------------------------------
 *      vadd3 : sumate 3 vectors
 *                             
 *  Parameters: Vectr a, Vectr b, Vectr c 
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vadd3(Vectr a, Vectr b, Vectr c) 
{
        return vadd(vadd(a, b), c);
}
/*-----------------------------------------------------------------------------
 *      vdot : vector dot product.
 *                             
 *  Parameters: Vectr a, Vectr b 
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t vdot(Vectr a, Vectr b) 
{
   return a.x * b.x + a.y * b.y + a.z * b.z;
} 
/*-----------------------------------------------------------------------------
 *      vmag2 : vector magnitude squared.
 *                             
 *  Parameters: Vectr v 
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t vmag2(Vectr v) 
{
   return vdot(v, v);
}
/*-----------------------------------------------------------------------------
 *      vmag : vector magnitude
 *                             
 *  Parameters: Vectr v 
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t vmag(Vectr v) 
{
    return sqrt(vmag2(v));
}
/*-----------------------------------------------------------------------------
 *      vdist2 : vector Euclidean distance squared.
 *                             
 *  Parameters: Vectr a, Vectr b 
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t vdist2(Vectr a, Vectr b) 
{
  return vmag2(vsub(a, b));
} 
/*-----------------------------------------------------------------------------
 *      mvmul : normalize a vector (make a unit vector).
 *                             
 *  Parameters: Vectr v 
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vnormalize(Vectr v) 
{
   //return vdiv(v, vmag(v));   this is it but try the faster one below as this calls so much recursive
   Vectr vec;
   vec.x = (1.0f/(sqrt(v.x * v.x + v.y * v.y + v.z * v.z))) * v.x;
   vec.y = (1.0f/(sqrt(v.x * v.x + v.y * v.y + v.z * v.z))) * v.y;
   vec.z = (1.0f/(sqrt(v.x * v.x + v.y * v.y + v.z * v.z))) * v.z;
   return vec;
} 

/*-----------------------------------------------------------------------------
 *      mvmul : multiply a matrix by a vector.
 *                             
 *  Parameters: mat33 a, Vectr v 
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr mvmul(mat33 a, Vectr v) 
{
    /* float32_t x,y,z;     */
    Vectr vec;
    /* x = a.m[0u][0u] * v.x + a.m[0u][1u] * v.y + a.m[0u][2u] * v.z;
    y = a.m[1u][0u] * v.x + a.m[1u][1u] * v.y + a.m[1u][2u] * v.z;
    z = a.m[2u][0u] * v.x + a.m[2u][1u] * v.y + a.m[2u][2u] * v.z;
    return mkvec(x, y, z); */
    vec.x = a.m[0u][0u] * v.x + a.m[0u][1u] * v.y + a.m[0u][2u] * v.z;
    vec.y = a.m[1u][0u] * v.x + a.m[1u][1u] * v.y + a.m[1u][2u] * v.z;
    vec.z = a.m[2u][0u] * v.x + a.m[2u][1u] * v.y + a.m[2u][2u] * v.z;
    return vec;
} 

/*-----------------------------------------------------------------------------
 *      mcolumn : construct a matrix from three column vectors.
 *                             
 *  Parameters: Vectr a, Vectr b, Vectr c
 *
 *  Return:     mat33
 *----------------------------------------------------------------------------*/
mat33 mcolumns(Vectr a, Vectr b, Vectr c) 
{
   mat33 m;
   m.m[0u][0u] = a.x;
   m.m[1u][0u] = a.y;
   m.m[2u][0u] = a.z;
   m.m[0u][1u] = b.x;
   m.m[1u][1u] = b.y;
   m.m[2u][1u] = b.z;
   m.m[0u][2u] = c.x;
   m.m[1u][2u] = c.y;
   m.m[2u][2u] = c.z;
   return m;
} 
/*-----------------------------------------------------------------------------
 *      mcolumn : return one column of a matrix as a vector.
 *                             
 *  Parameters: mat33 m, int16_t col
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr mcolumn(mat33 m, int16_t col) 
{
   return mkvec(m.m[0u][col], m.m[1u][col], m.m[2u][col]);
}
/*-----------------------------------------------------------------------------
 *      controllerMellingerReset:   Reset the Mellinger Controller
 *
 *  Parameters: void
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void controllerMellingerReset(void)
{
  i_error_x = 0;
  i_error_y = 0;
  i_error_z = 0;
  i_error_m_x = 0;
  i_error_m_y = 0;
  i_error_m_z = 0;
}
/*-----------------------------------------------------------------------------
 *      controllerMellinger:   Mellinger Controller
 *
 *  Parameters: control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, 
 *              const state_t *state, MellTiming_t *tick
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void controllerMellinger(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, MellTiming_t *tick)
{
  Vectr r_error;
  Vectr v_error;
  Vectr target_thrust;
  Vectr z_axis;
  float32_t current_thrust;
  Vectr x_axis_desired;
  Vectr y_axis_desired;
  Vectr x_c_des;
  Vectr eR, ew, M;
  float32_t dt;
  float32_t desiredYaw = 0; //deg
  Vectr y_yaw;
  /* mat33 R_yaw; */
  mat33 R_yaw_only;
  Vectr setpointPos;
  Vectr setpointVel;
  Vectr statePos;
  Vectr stateVel;
  quat setpoint_quat;
  Vectr rpy;
  quat q;
  mat33 R;
  Vectr x_yaw;
  float32_t x,y,z,w;
  float32_t err_d_roll;
  float32_t err_d_pitch;
  float32_t stateAttitudeRateRoll;
  float32_t stateAttitudeRatePitch;
  float32_t stateAttitudeRateYaw;
  int32_t ticksToNow = 0;

  if (tick->MelTimInit == 0u)                                         /* has been initialised */
  {
     ticksToNow = -1;                                                           /* on first pass initialise the timer */
     calculateTick2Now( &ticksToNow, &tick->tickLast );                     /* ticksNow also initialised at zero */
     tick->MelTimInit == 1u;                                          /* latch the first pass off */
  }
  else
  {
     calculateTick2Now( &ticksToNow, &tick->tickLast );                     /* calculate the ticks to now since last initialised */
  }
  dt = ticksToNow / CPU_TICKS_PER_SECOND;                              /* calculate the period of this call in seconds */
  tick->sample_time = dt;                     /* to observe the sample period for debug */
  /* if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

   dt = (float32_t)(1.0f/ATTITUDE_RATE); */

  if ( ticksToNow >= ATTITUDE_UPDATE_RATE )                                     /* we have waited until an other update period expired */
  {
     setpointPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
     setpointVel = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
     statePos = mkvec(state->position.x, state->position.y, state->position.z);
     stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

     // Position Error (ep)
     r_error = vsub(setpointPos, statePos);

     // Velocity Error (ev)
     v_error = vsub(setpointVel, stateVel);

     // Integral Error
     i_error_z += r_error.z * dt;
     i_error_z = clampReal(i_error_z, -i_range_z, i_range_z);
     i_error_x += r_error.x * dt;
     i_error_x = clampReal(i_error_x, -i_range_xy, i_range_xy);
     i_error_y += r_error.y * dt;
     i_error_y = clampReal(i_error_y, -i_range_xy, i_range_xy);

     // Desired thrust [F_des]
     if (setpoint->mode.x == modeAbs)
     {
       target_thrust.x = g_vehicleMass * setpoint->acceleration.x + kp_xy * r_error.x + kd_xy * v_error.x + ki_xy * i_error_x;
       target_thrust.y = g_vehicleMass * setpoint->acceleration.y + kp_xy * r_error.y + kd_xy * v_error.y + ki_xy * i_error_y;
       target_thrust.z = g_vehicleMass * (setpoint->acceleration.z + GRAV_CONST) + kp_z  * r_error.z + kd_z  * v_error.z + ki_z  * i_error_z;
     }
     else
     {
       target_thrust.x = -sin(DEGREE_TO_RADIAN(setpoint->attitude.pitch));
       target_thrust.y = -sin(DEGREE_TO_RADIAN(setpoint->attitude.roll));
       // In case of a timeout, the commander tries to level, ie. x/y are disabled, but z will use the previous setting
       // In that case we ignore the last feedforward term for acceleration
       if (setpoint->mode.z == modeAbs)
           {
          target_thrust.z = g_vehicleMass * GRAV_CONST + kp_z  * r_error.z + kd_z  * v_error.z + ki_z  * i_error_z;
       }
           else
           {
          target_thrust.z = 1;
       }
     }
     // Rate-controlled YAW is moving YAW angle setpoint
     if (setpoint->mode.yaw == modeVelocity)
     {
        desiredYaw = state->attitude.yaw + setpoint->attitudeRate.yaw * dt;
     }
     else if (setpoint->mode.yaw == modeAbs)
     {
        desiredYaw = setpoint->attitude.yaw;
     }
     else if (setpoint->mode.quat == modeAbs)
     {
        setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
        rpy = quat2rpy(setpoint_quat);
        desiredYaw = RADIAN_TO_DEGREE(rpy.z);
     }
     // Z-Axis [zB]
     q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
     R = quat2rotmat(q);
     z_axis = mcolumn(R, 2);

     if (setpoint->mode.x != modeAbs)                                           // yaw correction (only if position control is not used)
     {
        x_yaw = mcolumn(R, 0);
        x_yaw.z = 0;
        x_yaw = vnormalize(x_yaw);
        y_yaw = vcross(mkvec(0, 0, 1), x_yaw);
        R_yaw_only = mcolumns(x_yaw, y_yaw, mkvec(0, 0, 1));
        target_thrust = mvmul(R_yaw_only, target_thrust);
     }

     current_thrust = vdot(target_thrust, z_axis);                              // Current thrust [F]
     z_axis_desired = vnormalize(target_thrust);                                // Calculate axis [zB_des]

     x_c_des.x = cos(DEGREE_TO_RADIAN(desiredYaw));                             // [xC_des] x_axis_desired = z_axis_desired x [sin(yaw), cos(yaw), 0]^T
     x_c_des.y = sin(DEGREE_TO_RADIAN(desiredYaw));
     x_c_des.z = 0.0f;

     y_axis_desired = vnormalize(vcross(z_axis_desired, x_c_des));              // [yB_des]
     x_axis_desired = vcross(y_axis_desired, z_axis_desired);                   // [xB_des]
     // [eR]
     // Slow version
     // struct mat33 Rdes = mcolumns(
     //   mkvec(x_axis_desired.x, x_axis_desired.y, x_axis_desired.z),
     //   mkvec(y_axis_desired.x, y_axis_desired.y, y_axis_desired.z),
     //   mkvec(z_axis_desired.x, z_axis_desired.y, z_axis_desired.z));
     // struct mat33 R_transpose = mtranspose(R);
     // struct mat33 Rdes_transpose = mtranspose(Rdes);
     // struct mat33 eRM = msub(mmult(Rdes_transpose, R), mmult(R_transpose, Rdes));
     // eR.x = eRM.m[2][1];
     // eR.y = -eRM.m[0][2];
     // eR.z = eRM.m[1][0];
     // Fast version (generated using Mathematica)
     x = q.x;
     y = q.y;
     z = q.z;
     w = q.w;
     eR.x = (-1.0f + 2.0f*sqrt(x) + 2.0f*sqrt(y))*y_axis_desired.z + z_axis_desired.y - 2.0f*(x*y_axis_desired.x*z + y*y_axis_desired.y*z - x*y*z_axis_desired.x + sqrt(x)*z_axis_desired.y + sqrt(z)*z_axis_desired.y - y*z*z_axis_desired.z) +    2.0f*w*(-(y*y_axis_desired.x) - z*z_axis_desired.x + x*(y_axis_desired.y + z_axis_desired.z));
     eR.y = x_axis_desired.z - z_axis_desired.x - 2.0f*(sqrt(x)*x_axis_desired.z + y*(x_axis_desired.z*y - x_axis_desired.y*z) - (sqrt(y) + sqrt(z))*z_axis_desired.x + x*(-(x_axis_desired.x*z) + y*z_axis_desired.y + z*z_axis_desired.z) + w*(x*x_axis_desired.y + z*z_axis_desired.y - y*(x_axis_desired.x + z_axis_desired.z)));
     eR.z = y_axis_desired.x - 2.0f*(y*(x*x_axis_desired.x + y*y_axis_desired.x - x*y_axis_desired.y) + w*(x*x_axis_desired.z + y*y_axis_desired.z)) + 2.0f*(-(x_axis_desired.z*y) + w*(x_axis_desired.x + y_axis_desired.y) + x*y_axis_desired.z)*z - 2.0f*y_axis_desired.x*sqrt(z) + x_axis_desired.y*(-1.0f + 2.0f*sqrt(x) + 2.0f*sqrt(z));
     eR.y = -eR.y;                                                              // Account for Crazyflie coordinate system
     err_d_roll = 0.0f;                                                         // [ew]
     err_d_pitch = 0.0f;
     stateAttitudeRateRoll = DEGREE_TO_RADIAN(sensors->gyro.x);
     stateAttitudeRatePitch = -DEGREE_TO_RADIAN(sensors->gyro.y);
     stateAttitudeRateYaw = DEGREE_TO_RADIAN(sensors->gyro.z);
     ew.x = DEGREE_TO_RADIAN(setpoint->attitudeRate.roll) - stateAttitudeRateRoll;
     ew.y = -DEGREE_TO_RADIAN(setpoint->attitudeRate.pitch) - stateAttitudeRatePitch;
     ew.z = DEGREE_TO_RADIAN(setpoint->attitudeRate.yaw) - stateAttitudeRateYaw;
     /* if (prev_omega_roll == prev_omega_roll) d part initialized*/
     if (stateAttitudeRateRoll == prev_omega_roll)                              /* is this not d part initlaised ?? commented above out ACP */
     {
        err_d_roll = ((DEGREE_TO_RADIAN(setpoint->attitudeRate.roll) - prev_setpoint_omega_roll) - (stateAttitudeRateRoll - prev_omega_roll)) / dt;
        err_d_pitch = (-(DEGREE_TO_RADIAN(setpoint->attitudeRate.pitch) - prev_setpoint_omega_pitch) - (stateAttitudeRatePitch - prev_omega_pitch)) / dt;
     }
     prev_omega_roll = stateAttitudeRateRoll;
     prev_omega_pitch = stateAttitudeRatePitch;
     prev_setpoint_omega_roll = DEGREE_TO_RADIAN(setpoint->attitudeRate.roll);
     prev_setpoint_omega_pitch = DEGREE_TO_RADIAN(setpoint->attitudeRate.pitch);
     i_error_m_x += (-eR.x) * dt;                                               // Integral Error
     i_error_m_x = clampReal(i_error_m_x, -i_range_m_xy, i_range_m_xy);
     i_error_m_y += (-eR.y) * dt;
     i_error_m_y = clampReal(i_error_m_y, -i_range_m_xy, i_range_m_xy);
     i_error_m_z += (-eR.z) * dt;
     i_error_m_z = clampReal(i_error_m_z, -i_range_m_z, i_range_m_z);
     M.x = -kR_xy * eR.x + kw_xy * ew.x + ki_m_xy * i_error_m_x + kd_omega_rp * err_d_roll;       // Moment:
     M.y = -kR_xy * eR.y + kw_xy * ew.y + ki_m_xy * i_error_m_y + kd_omega_rp * err_d_pitch;
     M.z = -kR_z  * eR.z + kw_z  * ew.z + ki_m_z  * i_error_m_z;

     if (setpoint->mode.z == modeDisable)                                       // Output
     {
        control->thrust = setpoint->thrust;
     }
     else
     {
       control->thrust = massThrust * current_thrust;
     }
     cmd_thrust = control->thrust;
     m_r_roll = DEGREE_TO_RADIAN(sensors->gyro.x);
     m_r_pitch = -DEGREE_TO_RADIAN(sensors->gyro.y);
     m_r_yaw = DEGREE_TO_RADIAN(sensors->gyro.z);
     m_accelz = sensors->acc.z;
     if (control->thrust > 0)
     {
        control->roll = clampI16(M.x, -32000, 32000);
        control->pitch = clampI16(M.y, -32000, 32000);
        control->yaw = clampI16(-M.z, -32000, 32000);
        cmd_roll = control->roll;
        cmd_pitch = control->pitch;
        cmd_yaw = control->yaw;
     }
     else
     {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        cmd_roll = control->roll;
        cmd_pitch = control->pitch;
        cmd_yaw = control->yaw;
        controllerMellingerReset();
     }
  }
}
/*
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *

The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss
Ported by ACP Aviation

implementation of piecewise polynomial trajectories
See Daniel Mellinger, Vijay Kumar: "Minimum snap trajectory generation and control for quadrotors". ICRA 2011: 2520-2525
*/
/*-----------------------------------------------------------------------------
 *      piecewise_duration : calculates duration from piecewise trajectory
 *                             
 *  Parameters: piecewise_traj const *pp
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t piecewise_duration(piecewise_traj const *pp)
{
        int16_t i;
        float32_t total_dur = 0;
        for (i = 0; i < pp->n_pieces; ++i) 
        {
           total_dur += pp->pieces[i].duration;
        }
        return total_dur * pp->timescale;
}
/*-----------------------------------------------------------------------------
 *      polylinear polynomials are stored with ascending degree
 *                             
 *  Parameters: float32_t *p[PP_SIZE], float32_t duration, float32_t x0, float32_t x1
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void polylinear(float32_t *p[PP_SIZE], float32_t duration, float32_t x0, float32_t x1)
{
        int16_t i;
        
        *p[0u] = x0;
        *p[1u] = (x1 - x0) / duration;
        for (i = 2; i < PP_SIZE; ++i)
        {
           *p[i] = 0.0f;
        }
}
 
/*-----------------------------------------------------------------------------
 *      polystretchtime : e.g. if s==2 the new polynomial will be stretched to take 2x longer
 *                             
 *  Parameters: float32_t *p[PP_SIZE], float32_t s
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void polystretchtime(float32_t *p[PP_SIZE], float32_t s)
{
        float32_t recip = 1.0f / s;
        float32_t scale = recip;
        int16_t i;
        for (i = 1; i < PP_SIZE; ++i) 
        {
           *p[i] *= scale;
           scale *= recip;
        }
}
/*-----------------------------------------------------------------------------
 *      polyreflect : polynomial reflection
 *                             
 *  Parameters: float32_t *p[PP_SIZE]
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void polyreflect(float32_t *p[PP_SIZE])
{
        int16_t i;
        for (i = 1; i < PP_SIZE; i += 2) 
        {
          *p[i] = -*p[i];
        }
} 
/*-----------------------------------------------------------------------------
 *      polyval : evaluate a polynomial using horner's rule.
 *                             
 *  Parameters: const float32_t *p[PP_SIZE], float32_t t
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t polyval(const float32_t *p[PP_SIZE], float32_t t)
{
        float32_t x = 0.0f;
        int16_t i;
        for (i = PP_DEGREE; i >= 0; --i)
        {
           x = x * t + *p[i];
        }
        return x;
}

/*-----------------------------------------------------------------------------
 *      polyder : compute derivative of a polynomial in place
 *                             
 *  Parameters: float32_t *p[PP_SIZE]
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void polyder(float32_t *p[PP_SIZE])
{
        int16_t i;
        for (i = 1; i <= PP_DEGREE; ++i)
        {
          *p[i-1u] = i * *p[i];
        }
        *p[PP_DEGREE] = 0.0f;
}
/*-----------------------------------------------------------------------------
 *      poly5 : make 5 element polynomial
 *                             
 *  Parameters: float32_t *poly[PP_SIZE], float32_t T,float32_t x0, float32_t dx0, float32_t ddx0, float32_t xf, float32_t dxf, float32_t ddxf
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void poly5(float32_t *poly[PP_SIZE], float32_t T,float32_t x0, float32_t dx0, float32_t ddx0, float32_t xf, float32_t dxf, float32_t ddxf)
{
        float32_t T2 = T * T;
        float32_t T3 = T2 * T;
        float32_t T4 = T3 * T;
        float32_t T5 = T4 * T;
        int16_t i;
        *poly[0u] = x0;
        *poly[1u] = dx0;
        *poly[2u] = ddx0 / 2.0f;
        *poly[3u] = (-12.0f*dx0*T - 8.0f*dxf*T - 3.0f*ddx0*T2 + ddxf*T2 - 20.0f*x0 + 20.0f*xf)/(2.0f*T3);
        *poly[4u] = (16.0f*dx0*T + 14.0f*dxf*T + 3.0f*ddx0*T2 - 2.0f*ddxf*T2 + 30.0f*x0 - 30.0f*xf)/(2.0f*T4);
        *poly[5u] = (-6.0f*dx0*T - 6.0f*dxf*T - ddx0*T2 + ddxf*T2 - 12.0f*x0 + 12.0f*xf)/(2.0f*T5);
        for (i = 6; i < PP_SIZE; ++i) 
        {
          *poly[i] = 0.0f;
        }
}
/*-----------------------------------------------------------------------------
 *      poly7_nojerk : polynomial 7 with no jerk
 *                             
 *  Parameters: float32_t *poly[PP_SIZE], float32_t T, float32_t x0, float32_t dx0, float32_t ddx0, float32_t xf, float32_t dxf, float32_t ddxf 
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void poly7_nojerk(float32_t *poly[PP_SIZE], float32_t T, float32_t x0, float32_t dx0, float32_t ddx0, float32_t xf, float32_t dxf, float32_t ddxf)
{
        int16_t i;
        float32_t T2;
        float32_t T3;
        float32_t T4;
        float32_t T5;
        float32_t T6;
        float32_t T7;
        
        if (T <= 0.0f) 
        {
          *poly[0u] = xf;
          *poly[1u] = dxf;
          *poly[2u] = ddxf/2.0f;
          for (i = 3; i < PP_SIZE; ++i) 
          {
             *poly[i] = 0.0f;
          }
        } 
        else
        {
          T2 = T * T;
          T3 = T2 * T;
          T4 = T3 * T;
          T5 = T4 * T;
          T6 = T5 * T;
          T7 = T6 * T;
          *poly[0u] = x0;
          *poly[1u] = dx0;
          *poly[2u] = ddx0/2;
          *poly[3u] = 0.0f;
          *poly[4u] = -(5.0f*(14.0f*x0 - 14.0f*xf + 8.0f*T*dx0 + 6.0f*T*dxf + 2.0f*T2*ddx0 - T2*ddxf))/(2.0f*T4);
          *poly[5u] = (84.0f*x0 - 84.0f*xf + 45.0f*T*dx0 + 39.0f*T*dxf + 10.0f*T2*ddx0 - 7.0f*T2*ddxf)/T5;
          *poly[6u] = -(140.0f*x0 - 140.0f*xf + 72.0f*T*dx0 + 68.0f*T*dxf + 15.0f*T2*ddx0 - 13.0f*T2*ddxf)/(2.0f*T6);
          *poly[7u] = (2.0f*(10.0f*x0 - 10.0f*xf + 5.0f*T*dx0 + 5.0f*T*dxf + T2*ddx0 - T2*ddxf))/T7;
          for (i = 8; i < PP_SIZE; ++i) 
          {
             *poly[i] = 0.0f;
          }
        }
}
/*-----------------------------------------------------------------------------
 *      polybezier : polynomial with bezier
 *                             
 *  Parameters: float32_t *p[PP_SIZE], float32_t duration, float32_t *x, int16_t dim 
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void polybezier(float32_t *p[PP_SIZE], float32_t duration, float32_t *x, int16_t dim)
{
        int16_t i, j, n, sign;
        float32_t coeff;

        if (dim <= 0)
        {
           /* nothing to do */
        }
        else if (dim == 1)
        {
           *p[0u] = *x;
        }
        else if (dim == 2)
        {
           polylinear(p, duration, *x, *(x+(1*sizeof(float))));
        }
        else
        {
           n = ((dim < PP_SIZE) ? dim : PP_SIZE) - 1;
           sign = 1;
           for (j = 0; j <= n; j++)
           {
             coeff = 0.0f;
             sign = (j % 2) ? -1 : 1;
             for (i = 0; i <= j; i++, sign *= -1)
             {
                coeff += sign * *(x+(i*sizeof(float))) / facs[i] / facs[j-i];
             }
             *p[j] = coeff * facs[n] / facs[n-j];
           }
           polystretchtime(p, duration);
        }
}
//
// =============== 4d single-piece polynomials ==============================
//

/*-----------------------------------------------------------------------------
 *      poly4d_zero : construct a 4d zero polynomial.
 *                             
 *  Parameters: float32_t duration
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
poly4d poly4d_zero(float32_t duration)
{
        poly4d p;
        int16_t i;
        memset((void*)&p,0u,sizeof(poly4d));
        p.duration = duration;
        return p;
}
/*-----------------------------------------------------------------------------
 *      poly4d_linear : polynomial 3d with linear
 *                             
 *  Parameters: float32_t duration, Vectr p0, Vectr p1, float32_t yaw0, float32_t yaw1
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
poly4d poly4d_linear(float32_t duration, Vectr p0, Vectr p1, float32_t yaw0, float32_t yaw1)
{
        poly4d p;
        p.duration = duration;
        polylinear(&p.p, duration, p0.x, p1.x);
        polylinear(&p.p+(sizeof(float)*PP_SIZE), duration, p0.y, p1.y);
        polylinear(&p.p+(2*(sizeof(float)*PP_SIZE)), duration, p0.z, p1.z);
        polylinear(&p.p+(3*(sizeof(float)*PP_SIZE)), duration, yaw0, yaw1);
        return p;
}
/*-----------------------------------------------------------------------------
 *      polyscale : scale a 4d polynomial.
 *                             
 *  Parameters: float32_t *p[PP_SIZE], float32_t s
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void polyscale(float32_t *p[PP_SIZE], float32_t s)
{
        int16_t i;
        for (i = 0; i < PP_SIZE; ++i) 
        {
           *p[i] *= s;
        }
}
/*-----------------------------------------------------------------------------
 *      poly4d_scale : scale a 4d polynomial by a vector and yaw (quat)
 *                             
 *  Parameters: poly4d *p, float32_t x, float32_t y, float32_t z, float32_t yaw
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void poly4d_scale(poly4d *p, float32_t x, float32_t y, float32_t z, float32_t yaw)
{
        polyscale(&p->p, x);
        polyscale(&p->p+(sizeof(float)*PP_SIZE), y);
        polyscale(&p->p+(2*(sizeof(float)*PP_SIZE)), z);
        polyscale(&p->p+(3*(sizeof(float)*PP_SIZE)), yaw);

}
/*-----------------------------------------------------------------------------
 *      poly4d_shift : shift (sum) a 4d polynomial by a vector and yaw (quat)
 *                             
 *  Parameters: poly4d *p, float32_t x, float32_t y, float32_t z, float32_t yaw
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void poly4d_shift(poly4d *p, float32_t x, float32_t y, float32_t z, float32_t yaw)
{
        p->p[0u][0u] += x;
        p->p[1u][0u] += y;
        p->p[2u][0u] += z;
        p->p[3u][0u] += yaw;
}
/*-----------------------------------------------------------------------------
 *      poly4d_stretchtime : strech time by s
 *                             
 *  Parameters: poly4d *p, float32_t s
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void poly4d_stretchtime(poly4d *p, float32_t s)
{
        int16_t i;
        for (i = 0; i < 4; ++i) 
        {
           polystretchtime(&p->p+(i*(sizeof(float)*PP_SIZE)), s);
        }
        p->duration *= s;
}
/*-----------------------------------------------------------------------------
 *      polyder4d :
 *
 *  Parameters: poly4d *p
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void polyder4d(poly4d *p)
{
        int16_t i;
        for (i = 0; i < 4; ++i) 
        {
           polyder(&p->p+(i*(sizeof(float)*PP_SIZE)));
        }
}
/*-----------------------------------------------------------------------------
 *      polyval_xyz :   calculate polyval for xyz reading
 *
 *  Parameters: poly4d const *p, float32_t t
 *
 *  Return: Vectr
 *----------------------------------------------------------------------------*/
Vectr polyval_xyz(poly4d const *p, float32_t t)
{
        return mkvec(polyval(&p->p, t), polyval(&p->p+(1*(sizeof(float)*PP_SIZE)), t), polyval(&p->p+(2*(sizeof(float)*PP_SIZE)), t));
}
/*-----------------------------------------------------------------------------
 *      polyval_yaw :   calculate polyval for yaw reading
 *
 *  Parameters: poly4d const *p, float32_t t
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/
float32_t polyval_yaw(poly4d const *p, float32_t t)
{
        return polyval(p->p+(3*(sizeof(float)*PP_SIZE)), t);
}

/*-----------------------------------------------------------------------------
 *      vnorm1 :   L1 norm (aka Minkowski, Taxicab, Manhattan norm) of a vector.
 *
 *  Parameters: Vectr *v
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/
float32_t vnorm1(Vectr *v)
{
        return fabs((float64_t)v->x) + fabs((float64_t)v->y) + fabs((float64_t)v->z);
}

/*-----------------------------------------------------------------------------
 *      poly4d_max_accel_approx :   compute loose maximum of acceleration -
 *      uses L1 norm instead of Euclidean, evaluates polynomial instead of root-finding
 *
 *  Parameters: poly4d const *p
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/
float32_t poly4d_max_accel_approx(poly4d const *p)
{
        Vectr ddx;
        float32_t steps;
        float32_t step;
        float32_t t;
        float32_t amax;
        int16_t i;
        float32_t ddx_minkowski;
        poly4d* acc = &poly4d_tmp;
        *acc = *p;
        polyder4d(acc);
        polyder4d(acc);
        steps = 10.0f * p->duration;
        step = p->duration / (steps - 1.0f);
        t = 0.0f;
        amax = 0.0f;
        for (i = 0; i < ((int16_t)ROUND(steps,0)); ++i) 
        {
           ddx = polyval_xyz(acc, t);
           ddx_minkowski = vnorm1(&ddx);
           if (ddx_minkowski > amax) amax = ddx_minkowski;
           t += step;
        }
        return amax;
}
/*-----------------------------------------------------------------------------
 *      vrepeat :   construct a vector with the same value repeated for x, y, and z.
 *
 *  Parameters: float32_t x
 *
 *  Return: Vectr
 *----------------------------------------------------------------------------*/
Vectr vrepeat(float32_t x) 
{
        //return mkvec(x, x, x);
        Vectr v;
        v.x = x; v.y = x; v.z = x;
        return v;
}
/*-----------------------------------------------------------------------------
 *      traj_eval_zero :   construct a zero-vector.
 *
 *  Parameters: void
 *
 *  Return: Vectr
 *----------------------------------------------------------------------------*/
Vectr vzero(void) 
{
        //return vrepeat(0.0f);
        Vectr v;
        v.x = 0.0f; v.y = 0.0f; v.z = 0.0f;
        return v;
}
/*-----------------------------------------------------------------------------
 *      traj_eval_zero :   zero the trajectory evauation object
 *
 *  Parameters: Vectr v
 *
 *  Return: traj_eval
 *----------------------------------------------------------------------------*/
traj_eval traj_eval_zero()
{
        traj_eval ev;
        ev.pos = vzero();
        ev.vel = vzero();
        ev.acc = vzero();
        ev.yaw = 0.0f;
        ev.omega = vzero();
        return ev;
}
/*-----------------------------------------------------------------------------
 *      visnan :   test if any element of a vector is NaN.
 *
 *  Parameters: Vectr v
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t visnan(Vectr v) 
{
        return IsaNan(v.x) || IsaNan(v.y) || IsaNan(v.z);
}
/*-----------------------------------------------------------------------------
 *      traj_eval_invalid :   set the trajectory evaluation to invalid
 *
 *  Parameters: void
 *
 *  Return: traj_eval
 *----------------------------------------------------------------------------*/
traj_eval traj_eval_invalid()
{
        traj_eval ev;
        ev.pos = vrepeat(NaN);
        return ev;
}
/*-----------------------------------------------------------------------------
 *      is_traj_eval_valid :   check on trajectory evaluation
 *
 *  Parameters: traj_eval const *ev
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t is_traj_eval_valid(traj_eval const *ev)
{
        return !visnan(ev->pos);
}
/*-----------------------------------------------------------------------------
 *      vprojectunit :   projection of a onto b, where b is a unit vector.
 *
 *  Parameters: Vectr a, Vectr b_unit
 *
 *  Return: Vectr
 *----------------------------------------------------------------------------*/
Vectr vprojectunit(Vectr a, Vectr b_unit) 
{
        return vscl(vdot(a, b_unit), b_unit);
}
/*-----------------------------------------------------------------------------
 *      vorthunit :  component of a orthogonal to b, where b is a unit vector.
 *
 *  Parameters: Vectr a, Vectr b_unit
 *
 *  Return: Vectr
 *----------------------------------------------------------------------------*/
Vectr vorthunit(Vectr a, Vectr b_unit) 
{
        // original is here consider the new faster version below uncommented
        // as this is called too mnay recursive functions
        // return vsub(a, vprojectunit(a, b_unit));
        Vectr v;
        v.x=a.x-((a.x * b_unit.x + a.y + b_unit.y + a.z * b_unit.z) * b_unit.x);
        v.y=a.y-((a.x * b_unit.x + a.y + b_unit.y + a.z * b_unit.z) * b_unit.y);
        v.z=a.z-((a.x * b_unit.x + a.y + b_unit.y + a.z * b_unit.z) * b_unit.z);
        return v;
}
/*-----------------------------------------------------------------------------
 *      poly4d_eval :  4d polynomial evaluation
 *
 *  Parameters: poly4d const *p, float32_t t
 *
 *  Return: traj_eval
 *----------------------------------------------------------------------------*/
traj_eval poly4d_eval(poly4d const *p, float32_t t)
{
        traj_eval out;                                                          // flat variables
        poly4d* deriv = &poly4d_tmp;
        float32_t dyaw;
        Vectr jerk;
        Vectr thrust;
        Vectr z_body;
        Vectr x_world;
        Vectr y_body;
        Vectr x_body;
        Vectr jerk_orth_zbody;
        Vectr h_w;
        
        out.pos = polyval_xyz(p, t);
        out.yaw = polyval_yaw(p, t);
        *deriv = *p;                                                            // 1st derivative
        polyder4d(deriv);
        out.vel = polyval_xyz(deriv, t);
        dyaw = polyval_yaw(deriv, t);
        polyder4d(deriv);                                                       // 2nd derivative
        out.acc = polyval_xyz(deriv, t);
        polyder4d(deriv);                                                       // 3rd derivative
        jerk = polyval_xyz(deriv, t);

        thrust = vadd(out.acc, mkvec(0, 0, GRAV_CONST));
        // float thrust_mag = mass * vmag(thrust);
        z_body = vnormalize(thrust);
        x_world = mkvec(cos(out.yaw), sin(out.yaw), 0);
        y_body = vnormalize(vcross(z_body, x_world));
        x_body = vcross(y_body, z_body);
        jerk_orth_zbody = vorthunit(jerk, z_body);
        h_w = vscl(1.0f / vmag(thrust), jerk_orth_zbody);
        out.omega.x = -vdot(h_w, y_body);
        out.omega.y = vdot(h_w, x_body);
        out.omega.z = z_body.z * dyaw;
        return out;
}
//
// piecewise 4d polynomials
//

/*-----------------------------------------------------------------------------
 *      piecewise_eval :  piecewise evaluation
 *
 *  Parameters: piecewise_traj const *traj, float32_t t
 *
 *  Return: traj_eval
 *----------------------------------------------------------------------------*/
traj_eval piecewise_eval( piecewise_traj const *traj, float32_t t)
{
        int16_t cursor = 0;
        poly4d const *piece;
        poly4d const *end_piece;
        traj_eval ev;

        t = t - traj->t_begin;
        while (cursor < traj->n_pieces)
        {
                piece = &(traj->pieces[cursor]);
                if (t <= piece->duration * traj->timescale)
                {
                        poly4d_tmp = *piece;
                        poly4d_shift(&poly4d_tmp, traj->shift.x, traj->shift.y, traj->shift.z, 0);
                        poly4d_stretchtime(&poly4d_tmp, traj->timescale);
                        return poly4d_eval(&poly4d_tmp, t);
                }
                t -= piece->duration * traj->timescale;
                cursor=++cursor % INT16_MAX;
        }
        // if we get here, the trajectory has ended
        end_piece = &(traj->pieces[traj->n_pieces - 1]);
        ev = poly4d_eval(end_piece, end_piece->duration);
        ev.pos = vadd(ev.pos, traj->shift);
        ev.vel = vzero();
        ev.acc = vzero();
        ev.omega = vzero();
        return ev;
}
/*-----------------------------------------------------------------------------
 *      piecewise_eval_reversed:  piecewise evaluation reversed
 *
 *  Parameters: piecewise_traj const *traj, float32_t t
 *
 *  Return: traj_eval
 *----------------------------------------------------------------------------*/
traj_eval piecewise_eval_reversed( piecewise_traj const *traj, float32_t t)
{
        int16_t cursor = traj->n_pieces - 1;
        poly4d const *piece;
        int16_t i;
        poly4d const *end_piece;
        traj_eval ev;

        t = t - traj->t_begin;
        while (cursor >= 0)
        {
                piece = &(traj->pieces[cursor]);
                if (t <= piece->duration * traj->timescale)
                {
                        poly4d_tmp = *piece;
                        poly4d_shift(&poly4d_tmp, traj->shift.x, traj->shift.y, traj->shift.z, 0);
                        poly4d_stretchtime(&poly4d_tmp, traj->timescale);
                        for (i = 0; i < 4; ++i)
                        {
                                polyreflect(&poly4d_tmp.p+(i*(sizeof(float)*PP_SIZE)));
                        }
                        t = t - piece->duration * traj->timescale;
                        return poly4d_eval(&poly4d_tmp, t);
                }
                t -= piece->duration * traj->timescale;
                --cursor;
        }
        end_piece = &(traj->pieces[0u]);                                        // if we get here, the trajectory has ended
        ev = poly4d_eval(end_piece, 0.0f);
        ev.pos = vadd(ev.pos, traj->shift);
        ev.vel = vzero();
        ev.acc = vzero();
        ev.omega = vzero();
        return ev;
}
// y, dy == yaw, derivative of yaw
/*-----------------------------------------------------------------------------
 *      piecewise_plan_5th_order:  5th order piecewise projectory plan
 *
 *  Parameters: piecewise_traj *pp, float32_t duration, Vectr p0, float32_t y0,
 *              Vectr v0, float32_t dy0, Vectr a0, Vectr p1, float32_t y1, Vectr v1,
 *              float32_t dy1, Vectr a1
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void piecewise_plan_5th_order(piecewise_traj *pp, float32_t duration, Vectr p0, float32_t y0, Vectr v0, float32_t dy0, Vectr a0, Vectr p1, float32_t y1, Vectr v1, float32_t dy1, Vectr a1)
{
        poly4d *p = &pp->pieces[0u];
        p->duration = duration;
        pp->timescale = 1.0f;
        pp->shift = vzero();
        pp->n_pieces = 1;
        poly5(&p->p, duration, p0.x, v0.x, a0.x, p1.x, v1.x, a1.x);
        poly5(&p->p+(1u*(sizeof(float)*PP_SIZE)), duration, p0.y, v0.y, a0.y, p1.y, v1.y, a1.y);
        poly5(&p->p+(2u*(sizeof(float)*PP_SIZE)), duration, p0.z, v0.z, a0.z, p1.z, v1.z, a1.z);
        poly5(&p->p+(3u*(sizeof(float)*PP_SIZE)), duration, y0, dy0, 0, y1, dy1, 0);
}
/*-----------------------------------------------------------------------------
 *      piecewise_plan_7th_order_no_jerk:  7th order piecewise projectory plan
 *
 *  Parameters: piecewise_traj *pp, float32_t duration, Vectr p0, float32_t y0, 
 *              Vectr v0, float32_t dy0, Vectr a0, Vectr p1, float32_t y1, Vectr v1, 
 *              float32_t dy1, Vectr a1
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
// y, dy == yaw, derivative of yaw
void piecewise_plan_7th_order_no_jerk(piecewise_traj *pp, float32_t duration, Vectr p0, float32_t y0, Vectr v0, float32_t dy0, Vectr a0, Vectr p1, float32_t y1, Vectr v1, float32_t dy1, Vectr a1)
{
        poly4d *p = &pp->pieces[0u];
        p->duration = duration;
        pp->timescale = 1.0f;
        pp->shift = vzero();
        pp->n_pieces = 1;
        poly7_nojerk(&p->p, duration, p0.x, v0.x, a0.x, p1.x, v1.x, a1.x);
        poly7_nojerk(&p->p+(1u*(sizeof(float)*PP_SIZE)), duration, p0.y, v0.y, a0.y, p1.y, v1.y, a1.y);
        poly7_nojerk(&p->p+(2u*(sizeof(float)*PP_SIZE)), duration, p0.z, v0.z, a0.z, p1.z, v1.z, a1.z);
        poly7_nojerk(&p->p+(3u*(sizeof(float)*PP_SIZE)), duration, y0, dy0, 0, y1, dy1, 0);
}
/*-----------------------------------------------------------------------------
 *      hypot:  hypothesis
 *
 *  Parameters: float64_t x, float64_t y, float64_t z
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t hypot(float64_t x, float64_t y, float64_t z)
{
        return sqrt(x * x + y * y + z * z);
}
/*-----------------------------------------------------------------------------
 *      getDistanceVector:  get distancce between 2 vectors
 *
 *  Parameters: const Vectr from, const Vectr to
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t getDistanceVector(const Vectr from, const Vectr to)
{
        return hypot(from.x - to.x, from.y - to.y, from.z - to.z);
}

/*-----------------------------------------------------------------------------
 *      Kepler:  Solves Kepler's equation for ek through iteration
 *
 *  Parameters: mk: mean anomaly (rad), e:  eccentricity
 *
 *  Return: ek: eccentric anomaly
 *  ported from matlab code by Frank Van Diggelen
 *----------------------------------------------------------------------------*/
#define KEPLER_MAX_ITER 20u
float32_t Kepler( float32_t mk, float32_t e )
{
  float32_t ek = mk;
  float32_t err=1.0f;
  int8_t iterCount = 0;
  while (((err) > 1e-8) && (iterCount < KEPLER_MAX_ITER))
  {
     err = ek - mk - (e*sin(ek));
     ek = ek - err;
     iterCount = ++iterCount % INT8_MAX;
  }
  return ek;
}
/*-----------------------------------------------------------------------------
 *      Lla2Ecef:  Transform latitude, longitude, altitude to ECEF coordinates.
 *
 *  Parameters: mk: mean anomaly (rad), e:  eccentricity
 *
 *  Return: Vectr
 *  ported from matlab code by Frank Van Diggelen
 *----------------------------------------------------------------------------*/
#define EARTHSEMIMAJOR 6378137.0f                                               /* length of the earths semi major axis in meters  */
#define EARTHSEMIMINOR 6356752.3142f                                            /* length of the earths semi minor axis in meters  */
#define EARTHECCEN2 6.69437990136e-2f                                           /* first numerical eccentricity */
#define HW_EARTHSEMMAJ_SQR (EARTHSEMIMAJOR*EARTHSEMIMAJOR)
Vectr Lla2Ecef( GPS_Info_t *gps )
{
   Vectr Ecef;
   float32_t clat = cos(gps->lat*(PI/180.0f));
   float32_t clon = cos(gps->longd*(PI/180.0f));
   float32_t slat = sin(gps->lat*(PI/180.0f));
   float32_t slon = sin(gps->longd*(PI/180.0f));
   /* Compute position vector in ECEF coordinates. */
   float32_t r0 = EARTHSEMIMAJOR * pow((sqrt(1.0f - EARTHECCEN2 * slat * slat)),(-1));
   Ecef.x = (gps->altRef + r0) * clat * clon;                                   /* x coordinate */
   Ecef.y = (gps->altRef + r0) * clat * slon;                                   /* y coordinate */
   Ecef.z = (gps->altRef + r0 * (1.0f - EARTHECCEN2))*slat;                     /* z coordinate */
   return Ecef;
}
/*-----------------------------------------------------------------------------
 *      mzero:  set matrix 3x3 to zero
 *
 *  Parameters: void
 *
 *  Return: mat33 of 0.0
 *
 *----------------------------------------------------------------------------*/
mat33 mzero(void) 
{
   mat33 m;
   int8_t i,j;
   for (i = 0; i < 3; ++i)
   {
     for (j = 0; j < 3; ++j) 
     {
        m.m[i][j] = 0.0f;
     }
  }
  return m;
}
/*-----------------------------------------------------------------------------
 *      RotEcef2Ned:  Rotation matrix to convert an lat and lon to NED coordinate
 *
 *  Parameters: GPS_Info_t *gps (lat long from gPS)
 *
 *  Return: mat33  Re2n,   3x3 unitary rotation matrix
 *
 *  To Convert
 *      Ned = Re2n*vEcef,
 *      Re2n*vNed = vEcef
 *  ported from matlab code by Frank Van Diggelen
 *----------------------------------------------------------------------------*/
mat33 RotEcef2Ned( GPS_Info_t *gps )
{

   mat33 Re2n; // = mzero();
   float32_t latRad=(PI/180.0f)*gps->lat;
   float32_t lonRad=(PI/180.0f)*gps->longd;
   float32_t clat = cos(latRad);
   float32_t slat = sin(latRad);
   float32_t clon = cos(lonRad);
   float32_t slon = sin(lonRad);

   Re2n.m[0u][0u] = -slat*clon;
   Re2n.m[0u][1u] = -slat*slon;
   Re2n.m[0u][2u] = clat;
   Re2n.m[1u][0u] = -slon;
   Re2n.m[1u][1u] = clon;
   Re2n.m[1u][2u] = 0;
   Re2n.m[2u][0u] = -clat*clon;
   Re2n.m[2u][1u] = -clat*slon;
   Re2n.m[3u][2u] = -slat;
   return Re2n;
}
/*#######################################################################################
Flight Control
#######################################################################################*/
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + www.MikroKopter.com
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define GIER_GRAD_FAKTOR 1291u
#define Parameter_Hoehe_TiltCompensation 110.0f
/*-----------------------------------------------------------------------------
 *      getCosAttitude:  get cos attitude
 *
 *  Parameters: uint32_t IntegralNick, uint32_t IntegralRoll, float64_t *CosAttitude
 *
 *  Return: void result is cosAttitude
 *
 *----------------------------------------------------------------------------*/
void getCosAttitude( uint32_t IntegralNick, uint32_t IntegralRoll, float64_t *CosAttitude )
{
        // calculate cos of pitch (nick) and roll angle used for projection of the vertical hover gas
        float64_t tmp_int  = (float64_t)(IntegralNick/GIER_GRAD_FAKTOR);        // pitch angle in deg
        float64_t tmp_int2 = (float64_t)(IntegralRoll/GIER_GRAD_FAKTOR);        // roll angle in deg                 
        tmp_int = FMIN(((sqrt((tmp_int*tmp_int)+(tmp_int2*tmp_int2)) * Parameter_Hoehe_TiltCompensation) / 100.0f), 60.0f); // phytagoras gives effective attitude angle in deg & limit effective attitude angle to be below 60                                       
        *CosAttitude = cos(tmp_int);                                           // cos of actual attitude
}
#define HOVER_GAS_AVERAGE 16384L                                                // 16384 * 2ms = 32s averaging
#define HC_GAS_AVERAGE 4                                                        // 4 * 2ms= 8ms averaging
// SollHoehe - setpoint height
// HoehenWertF - current height
/*-----------------------------------------------------------------------------
 *      mikroKopter_HoverGasEstimation:  calculate hover gas estimate for height control
 *
 *  Parameters: uint32_t currentThrust, int16_t CosAttitude, int16_t VarioMeter, 
 *              uint32_t HoehenWertF, uint32_t SollHoehe, 
 *              const mk_EEprom_param_t EE_Parameter, mk_Hover_t *status
 *
 *  Return: void result is cosAttitude
 *
 *----------------------------------------------------------------------------*/
void mikroKopter_HoverGasEstimation( uint32_t currentThrust, int16_t CosAttitude, int16_t VarioMeter, uint32_t HoehenWertF, uint32_t SollHoehe, const mk_EEprom_param_t EE_Parameter, mk_Hover_t *status  )
{
        uint32_t tmp_long2;                
        int16_t band;
        float64_t modell_fliegt;
                
        if (status->init==false)
        {
           status->tickRef = CP0_GET(CP0_COUNT);
           status->time2Now = 0u;
           status->init==true;
        }
        else
        {                        
            calculateTick2Now(&status->time2Now,&status->tickRef);                /* update the time we are running counter */        
            modell_fliegt = TICKS2SEC(status->time2Now);                        
        }
        // Hover gas estimation by averaging gas control output on small z-velocities 
        // this is done only if height contol option is selected in global config and aircraft is flying
        if((status->flying==true)&&(status->esd==true))                         // FC_StatusFlags & FC_STATUS_FLY && !(FC_SatusFlags & FC_STATUS_EMERGENCY_LANDING))
        {
                        //if(status->HoverGasFilter == 0 || status->StartTrigger == 1)  status->HoverGasFilter = HOVER_GAS_AVERAGE * (unsigned long)(GasMischanteil); // init estimation
                if(status->HoverGasFilter == 0 || status->StartTrigger == 1u)  status->HoverGasFilter = HOVER_GAS_AVERAGE * (unsigned long)(status->HoverGas); // 0.90f: geändert
                if(status->StartTrigger == 1u) status->StartTrigger = 2u;
                tmp_long2 = currentThrust;                                      // take current thrust
                tmp_long2 *= CosAttitude;                                       // apply attitude projection
                tmp_long2 /= 8192;
                // average vertical projected thrust
                if(modell_fliegt < 8.0f)                                        // the first 8 seconds
                {                                                               // reduce the time constant of averaging by factor of 4 to get much faster a stable value
                    status->HoverGasFilter -= status->HoverGasFilter/(HOVER_GAS_AVERAGE/16L);
                    status->HoverGasFilter += 16L * tmp_long2;
                }
                if(modell_fliegt < 16.0f)                                       // the first 16 seconds
                {                                                               // reduce the time constant of averaging by factor of 2 to get much faster a stable value
                   status->HoverGasFilter -= status->HoverGasFilter/(HOVER_GAS_AVERAGE/4L);
                   status->HoverGasFilter += 4L * tmp_long2;
                }
                else                                                            //later
                if(abs(VarioMeter) < 100 && abs(HoehenWertF - SollHoehe) < 256) // only on small vertical speed & difference is small (only descending)
                {
                   status->HoverGasFilter -= status->HoverGasFilter/HOVER_GAS_AVERAGE;
                   status->HoverGasFilter += tmp_long2;
                }
                status->HoverGas = (int16_t)(status->HoverGasFilter/HOVER_GAS_AVERAGE);
                if(EE_Parameter.Hoehe_HoverBand)
                {
                   band = status->HoverGas / (int16_t)EE_Parameter.Hoehe_HoverBand; // the higher the parameter the smaller the range
                   status->HoverGasMin = status->HoverGas - band;
                   status->HoverGasMax = status->HoverGas + band;
                }
                else
                {                                                                // no limit
                   status->HoverGasMin = 0;
                   status->HoverGasMax = 1023;
                }
        }
        else
        {
           status->StartTrigger = 0;
           status->HoverGasFilter = 0;
           status->HoverGas = 0;
        }
}
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Nick-Achse Regler : Pitch-Axis-Control
// MesswertNick - measured pitch
// StickNick - setpoint pitch
// pd_ergebnis_nick - PD controller output for pitch servo
// GasMischanteil - thrust of gas
// GierMischanteil - gear
// TODO : same controller used for roll so name it as a generic axis controller 
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*-----------------------------------------------------------------------------
 *      mikroKopterPitchAxisContoller:  Nick-Achse Regler : Pitch-Roll-Axis-Control
 *
 *  Parameters: const mk_EEprom_param_t EE_Parameter, mk_PitchAxisCon_t *pitch,  
 *              uint32_t GasMischanteil, uint32_t GierMischanteil 
 *
 *  Return: void 
 *
 *----------------------------------------------------------------------------*/
#define STICK_GAIN 4
void mikroKopterPitchAxisContoller( const mk_EEprom_param_t EE_Parameter, mk_PitchAxisCon_t *pitch, uint32_t GasMischanteil, uint32_t GierMischanteil  )
{
    uint64_t tmp_int; 
    uint32_t DiffNick = pitch->MesswertNick - pitch->StickNick;                        // Differenz bestimmen
    if(pitch->IntegralFaktor) pitch->SummeNick += pitch->IntegralNickMalFaktor - pitch->StickNick; // I-Anteil bei Winkelregelung
    else  pitch->SummeNick += DiffNick;                                         // I-Anteil bei HH
    if(pitch->SummeNick >  (STICK_GAIN * 16000L)) pitch->SummeNick =  (STICK_GAIN * 16000L);
    if(pitch->SummeNick < -(16000L * STICK_GAIN)) pitch->SummeNick = -(16000L * STICK_GAIN);

    if(EE_Parameter.Gyro_Stability <= 8) pitch->pd_ergebnis_nick = (EE_Parameter.Gyro_Stability * DiffNick) / 8; // PI-Regler für Nick
    else pitch->pd_ergebnis_nick = ((EE_Parameter.Gyro_Stability / 2) * DiffNick) / 4; // Überlauf verhindern
    pitch->pd_ergebnis_nick +=  pitch->SummeNick / pitch->Ki;

    tmp_int = (EE_Parameter.DynamicStability * (GasMischanteil + abs(GierMischanteil)/2)) / 64;
    if(pitch->pd_ergebnis_nick >  tmp_int) pitch->pd_ergebnis_nick = (uint32_t)tmp_int;
    if(pitch->pd_ergebnis_nick < -tmp_int) pitch->pd_ergebnis_nick = (uint32_t)-tmp_int;
}
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Gier-Anteil
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// GasMischanteil - thrust
// GierMischanteil - gear
#define MAX_GAS 230                                                             // range 33-247
#define Hoehe_MinGas 30                                                         // rabge 0-100
/*-----------------------------------------------------------------------------
 *      mikroKopterGierAnteilRegler:  Gier-Anteil Regler : Gear-Control
 *
 *  Parameters: uint32_t MesswertGier, uint32_t sollGier, uint32_t GasMischanteil,   
 *              uint32_t *GierMischanteil 
 *
 *  Return: void 
 *
 *----------------------------------------------------------------------------*/
void mikroKopterGierAnteilRegler( uint32_t MesswertGier, uint32_t sollGier, uint32_t GasMischanteil, uint32_t *GierMischanteil )
{
     uint32_t tmp_int;
    *GierMischanteil = MesswertGier - sollGier * STICK_GAIN;                    // Regler für Gier
#define MIN_GIERGAS  (40*STICK_GAIN)                                            // unter diesem Gaswert trotzdem Gieren
   if(GasMischanteil > MIN_GIERGAS)
    {
     if(*GierMischanteil > (GasMischanteil / 2)) *GierMischanteil = GasMischanteil / 2;
     if(*GierMischanteil < -(GasMischanteil / 2)) *GierMischanteil = -(GasMischanteil / 2);
    }
    else
    {
     if(*GierMischanteil > (MIN_GIERGAS / 2))  *GierMischanteil = MIN_GIERGAS / 2;
     if(*GierMischanteil < -(MIN_GIERGAS / 2)) *GierMischanteil = -(MIN_GIERGAS / 2);
    }
    tmp_int = MAX_GAS*STICK_GAIN;
    if(*GierMischanteil > ((tmp_int - GasMischanteil))) *GierMischanteil = ((tmp_int - GasMischanteil));
    if(*GierMischanteil < -((tmp_int - GasMischanteil))) *GierMischanteil = -((tmp_int - GasMischanteil));
}
/*-----------------------------------------------------------------------------
 *      mikRoKopterDpart3GpsZ:  trim of the gas thrust setting 
 *
 *  Parameters: int16_t Parameter_Hoehe_GPS_Z, int16_t FromNaviCtrl_Value_GpsZ,     
 *              mk_Hover_t *hov, int16_t *HCGas, int16_t HeightDeviation,
 *
 *  Return: void 
 *
 *----------------------------------------------------------------------------*/
void mikRoKopterDpart3GpsZ( int16_t Parameter_Hoehe_GPS_Z, int16_t FromNaviCtrl_Value_GpsZ, mk_Hover_t *hov, int16_t *HCGas, int16_t HeightDeviation )
{
        uint16_t tmp;
        int16_t tmp_int = (Parameter_Hoehe_GPS_Z * FromNaviCtrl_Value_GpsZ)/128L;
        tmp_int = min(max(tmp_int, (-32 * STICK_GAIN)), (64 * STICK_GAIN));
        hov->GasReduction += tmp_int;
        hov->GasReduction = (hov->GasReduction * (int64_t)hov->HoverGas) / 512LL; // scale to the gas value

        *HCGas -= hov->GasReduction;
        // limit deviation from hoover point within the target region
        if((hov->AltitudeSetpointTrimming==false) && (hov->HoverGas > 0))       // height setpoint is not changed and hoover gas not zero
        {
            tmp = abs(HeightDeviation);
           if(tmp <= 60)
           {
                min(max(*HCGas, hov->HoverGasMin), hov->HoverGasMax);           // limit gas around the hoover point
           }
           else
           {
                tmp = (tmp - 60) / 32;
                if(tmp > 15) tmp = 15;
                if(HeightDeviation > 0)
                {
                   tmp = (hov->HoverGasMin * (16 - tmp)) / 16;
                   min(max(*HCGas, tmp), hov->HoverGasMax);                     // limit gas around the hoover point
                }
                else
                {
                   tmp = (hov->HoverGasMax * (tmp + 16)) / 16;
                   min(max(*HCGas, hov->HoverGasMin), tmp);                     // limit gas around the hoover point
                }
           }
       }
}
/*-----------------------------------------------------------------------------
 *      trimGasThrustSetting:  trim of the gas thrust setting 
 *
 *  Parameters: int16_t *GasMischanteil, int16_t *FilterHCGas, int32_t CosAttitude,    
 *              int16_t *HCGas, mk_Hover_t *hov
 *
 *  Return: void 
 *
 *----------------------------------------------------------------------------*/
void trimGasThrustSetting( int16_t *GasMischanteil, int16_t *FilterHCGas, int32_t CosAttitude, int16_t *HCGas, mk_Hover_t *hov )
{                        
        // strech control output by inverse attitude projection 1/cos
        // + 1/cos(angle)  ++++++++++++++++++++++++++
        int32_t tmp_long2 = (int32_t)*HCGas;
        tmp_long2 *= 8192L;
        tmp_long2 /= CosAttitude;
        *HCGas = (int16_t)tmp_long2;

        *FilterHCGas = (*FilterHCGas * (HC_GAS_AVERAGE - 1) + *HCGas) / HC_GAS_AVERAGE;  // update height control gas averaging

        *FilterHCGas = max((Hoehe_MinGas * STICK_GAIN) ,min(((MAX_GAS - 20) * STICK_GAIN),*FilterHCGas));  // limit height control gas pd-control output
        if (hov->thrustHtLim==true)
        /* set GasMischanteil to HeightControlGasFilter
         if(Parameter_ExtraConfig & CFG2_HEIGHT_LIMIT) */       
        {  
          min(*FilterHCGas, *GasMischanteil);                                   // nicht mehr als Gas
          *GasMischanteil = *FilterHCGas;
        }
        else 
        *GasMischanteil = *FilterHCGas + (*GasMischanteil - hov->HoverGas) / 4; // only in Vario-Mode
}

/*#######################################################################################
(GPS Co-ordinates) to ENU (Navigation co-orsinates) conversion
#######################################################################################*/
// dllh to denu co-ordinates phi lambda h (GPS Co-ordinates) to ENU (Navigation co-orsinates) conversion
// translated from matlab code from Simon Picton Drake of the ADF (Austrailian Defense Force)
// llh0 = a vector indicating the location of the reference Point e.g radar
// llh = is a n x 3 matrix indicating the points of interest e.g. aeroplane
// ouput is a n x 3 matrix representing east, north, up displacement in meters
/*-----------------------------------------------------------------------------
 *      dllh2denu : llh co-ord to enu east north up co-ord
 *                             
 *  Parameters: Vectr *llh0, Vectr *llh 
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr dllh2denu( Vectr *llh0, Vectr *llh )
{
  Vectr v;
  float32_t e2 = 1.0f - ((EARTHSEMIMAJOR/EARTHSEMIMINOR)*(EARTHSEMIMAJOR/EARTHSEMIMINOR));
  float32_t tmp1,cl,sl,cp,sp;
  float32_t phi,lam,h,dphi,dlam,dh;

  phi = llh0->x * PI/180.0f;                                                    /* location of referecne Point in radians */
  lam = llh0->y * PI/180.0f;
  h = llh0->z * PI/180.0f;

  dphi = (llh->x * PI/180.0f) - phi;                                            /* location of data points in radians */
  dlam = (llh->y * PI/180.0f) - lam;
  dh = (llh->z * PI/180.0f) - h;

  sp = sin(phi);
  tmp1 = sqrt(1.0f - ((e2*e2)*(sp*sp)));                                        /* some useful definitions */
  cl = cos(lam);
  sl = sin(lam);
  cp = cos(phi);

  v.x = (EARTHSEMIMAJOR/tmp1+h)*cp*dlam -(EARTHSEMIMAJOR*(1-e2)/(pow(tmp1,3.0f))+h)*sp*dphi*dlam + cp*dlam*dh; /* transformations */
  v.y = ((EARTHSEMIMAJOR *(1-e2)/pow(tmp1,3.0f)) +h)*dphi +1.5f*cp*sp*EARTHSEMIMAJOR*e2*(dphi*dphi)+(sp*sp)*dh*dphi -0.5f*sp*cp*(EARTHSEMIMAJOR/tmp1 +h)*(dlam*dlam);
  v.z = dh - (EARTHSEMIMAJOR-1.5f*EARTHSEMIMAJOR*e2*pow(cp,2.0f)+0.5f*EARTHSEMIMAJOR*e2+h)*(dphi*dphi) -0.5f*(cp*cp)*(EARTHSEMIMAJOR/tmp1 -h)*(dlam*dlam)  ;
  return v;
}
/*-----------------------------------------------------------------------------
 *      dllh2denu2 : llh co-ord to enu east north up co-ord
 *                             
 *  Parameters: Vectr *llh0, Vectr *llh[], Vectr *v[]  
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void dllh2denu2( Vectr *llh0, Vectr *llh[], Vectr *v[] )
{

  float32_t e2 = 1.0f - ((EARTHSEMIMAJOR/EARTHSEMIMINOR)*(EARTHSEMIMAJOR/EARTHSEMIMINOR));
  float32_t tmp1,cl,sl,cp,sp;
  float32_t phi,lam,h,dphi,dlam,dh;
  int8_t numOfEle = (sizeof(*llh)/sizeof(Vectr));
  int8_t ele;

  for (ele=0; ele<numOfEle; ele++)
  {
    phi = llh0->x * PI/180.0f;                                                  /* location of referecne Point in radians */
    lam = llh0->y * PI/180.0f;
    h = llh0->z * PI/180.0f;

    dphi = (llh[ele]->x * PI/180.0f) - phi;                                     /* location of data points in radians */
    dlam = (llh[ele]->y * PI/180.0f) - lam;
    dh = (llh[ele]->z * PI/180.0f) - h;
    sp = sin(phi);
    tmp1 = sqrt(1.0f - ((e2*e2)*(sp*sp)));                                      /* some useful definitions */
    cl = cos(lam);
    sl = sin(lam);
    cp = cos(phi);

    v[ele]->x = (EARTHSEMIMAJOR/tmp1+h)*cp*dlam -(EARTHSEMIMAJOR*(1-e2)/(pow(tmp1,3.0f))+h)*sp*dphi*dlam + cp*dlam*dh; /* transformations */
    v[ele]->y = ((EARTHSEMIMAJOR *(1-e2)/pow(tmp1,3.0f)) +h)*dphi +1.5f*cp*sp*EARTHSEMIMAJOR*e2*(dphi*dphi)+(sp*sp)*dh*dphi -0.5f*sp*cp*(EARTHSEMIMAJOR/tmp1 +h)*(dlam*dlam);
    v[ele]->z = dh - (EARTHSEMIMAJOR-1.5f*EARTHSEMIMAJOR*e2*pow(cp,2.0f)+0.5f*EARTHSEMIMAJOR*e2+h)*(dphi*dphi) -0.5f*(cp*cp)*(EARTHSEMIMAJOR/tmp1 -h)*(dlam*dlam);
  }

}
 
/*-----------------------------------------------------------------------------
 *      getNedfromECEF : NED from ECEF
 *                             
 *  Parameters: mat33 Re2n, Vectr ecef 
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr getNedfromECEF( mat33 Re2n, Vectr ecef )
{
    return mvmul(Re2n, ecef);
}
/*-----------------------------------------------------------------------------
 *      mtranspose : ECEF from NED
 *                             
 *  Parameters: mat33 m 
 *
 *  Return:     mat33
 *----------------------------------------------------------------------------*/
mat33 mtranspose(mat33 m)
{
   mat33 mt;
   int8_t i,j;
   for (i = 0; i < 3; ++i)
   {
        for (j = 0; j < 3; ++j)
        {
              mt.m[i][j] = m.m[j][i];
        }
   }
   return mt;
}  
/*-----------------------------------------------------------------------------
 *      dllh2denu : ECEF from NED
 *                             
 *  Parameters: mat33 Re2n, Vectr ned 
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr getECEFfromNed( mat33 Re2n, Vectr ned )
{
    return mvmul(mtranspose(Re2n), ned);
}
// Transform Earth-Centered-Earth-Fixed x,y,z, coordinates to lat,lon,alt.
// Hoffman-Wellenhof, Lichtenegger & Collins "GPS Theory & Practice"
// ported from matlab by ACP Aviation
// original matlab Author: Frank van Diggelen
/*-----------------------------------------------------------------------------
 *      Xyz2Lla : xyz to lla
 *                             
 *  Parameters: Vectr eCef 
 *
 *  Return:     GPS_Info_t
 *----------------------------------------------------------------------------*/
GPS_Info_t Xyz2Lla( Vectr eCef )
{
  GPS_Info_t gps;
  float32_t xM,yM,zM,b2,b,p,s1,s2,h;
  
  if (( eCef.y == 0.0f ) && ( eCef.x == 0.0f ))                                 // if x and y ecef positions are both zero then lla is undefined
  {
     memset((void*)&gps,0,sizeof(GPS_Info_t));
  }
  else
  {
     b2 = HW_EARTHSEMMAJ_SQR*(1.0f-EARTHECCEN2);                                // following algorithm from Hoffman-Wellenhof, et al. "GPS Theory & Practice":
     b = sqrt(b2);
     p=sqrt((eCef.x*eCef.x) + (eCef.y*eCef.y));
     s1 = eCef.z*EARTHSEMIMAJOR;                                                // two sides and hypotenuse of right angle triangle with one angle = theta:
     s2 = p*b;
     h = sqrt((s1*s1) + (s2*s2));
     s1 = eCef.z+((HW_EARTHSEMMAJ_SQR-b2)/b2)*b*(pow((s1/h),3.0f));             // % two sides and hypotenuse of right angle triangle with one angle = lat:
     s2 = p-EARTHSEMIMAJOR*EARTHECCEN2*(pow((s2/h),3.0f));
     gps.lat = atan(s1/s2);
     gps.lat = RADIAN_TO_DEGREE(gps.lat);
     gps.altRef = (HW_EARTHSEMMAJ_SQR*(pow((HW_EARTHSEMMAJ_SQR*((s2/h)*(s2/h)) + b2*((s1/h)*(s1/h))),-0.5f)));
     gps.longd = atan2(eCef.y,eCef.x);                                          // since deltaTime = 0 for ECEF
     gps.longd = RADIAN_TO_DEGREE(gps.longd);                                   // consider ??? TODO: GpsConstants.WE*deltaTime
     if (gps.longd>180.0f)
        gps.longd = gps.longd-360.0f;
  }
  return gps;
}
// QUAD ENCODER For Wheel :: convert pin a and pin b state to a wheel encoder phase
// from Ardupilot ported by Air Cam Pro
/*-----------------------------------------------------------------------------
 *      encQuadraturePin2phase:  wheel encoder return phase from pin position
 *
 *  Parameters: Quad_Enc_t *quadObj
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t encQuadraturePin2phase( Quad_Enc_t *quad )
{
    if (!quad->last_pin_a_value)
    {
        if (!quad->last_pin_b_value)
        {
            return 0u;                                                           // A = 0, B = 0
        } else 
        {
            return 1u;                                                          // A = 0, B = 1
        }
    } 
    else 
    {
        if (!quad->last_pin_b_value)
        {
            return 3u;                                                          // A = 1, B = 0
        } else 
        {
            return 2u;                                                          // A = 1, B = 1
        }
    }
    return (uint8_t)quad->last_pin_a_value << 1u | (uint8_t)quad->last_pin_b_value;
}
/*-----------------------------------------------------------------------------
 *      encodQuadPhaseErroCnt:  wheel quadrature encoder phase and error counter
 *
 *  Parameters: Quad_Enc_t *quadObj
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void encodQuadPhaseErroCnt( Quad_Enc_t *quadObj )
{
    uint8_t phase_after = encQuadraturePin2phase(quadObj);                      // convert pin state before and after to phases
    uint8_t step_forward = quadObj->phase < 3 ? quadObj->phase+1 : 0;           // look for invalid changes
    uint8_t step_back = quadObj->phase > 0 ? quadObj->phase-1 : 3;
    if (phase_after == step_forward) 
    {
        quadObj->phase = phase_after;
        quadObj->distance_count=++quadObj->distance_count % INT32_MAX;
    } 
    else if (phase_after == step_back) 
    {
        quadObj->phase = phase_after;
        quadObj->distance_count=--quadObj->distance_count % INT32_MIN;
    } 
    else 
    {
        quadObj->error_count=++quadObj->error_count % UINT32_MAX;
    }
    quadObj->total_count=++quadObj->total_count % UINT32_MAX;
}
/*-----------------------------------------------------------------------------
 *      encodWheelRead:  read wheel encoder
 *
 *  Parameters: uint8_t pin,  bool pin_value, uint32_t timestamp, Quad_Enc_t *encObj
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void encodWheelRead(uint8_t pin,  bool pin_value, uint32_t timestamp, Quad_Enc_t *encObj)
{

    if (encObj->last_pin_a == 0 || encObj->last_pin_b == 0) {                   // sanity check
        return;
    }

    if (pin == encObj->last_pin_a)                                              // update distance and error counts
    {
        encObj->last_pin_a_value = pin_value;
    }
    else if (pin == encObj->last_pin_b)
    {
        encObj->last_pin_b_value = pin_value;
    }
    else
    {
        return;
    }
    encodQuadPhaseErroCnt(encObj);

    encObj->last_reading_ms = timestamp;                                        // record update time
}
/*-----------------------------------------------------------------------------
 *      Quarternion_to_AxisAngle:  Converting quartenion to Axis Angle
 *
 *  Parameters: const COMGS_quarternion *quartCalculated, const COMGS_axis_angle *axisAngle
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void Quarternion_to_AxisAngle( const COMGS_quarternion *quartCalculated, COMGS_axis_angle *axisAngle )
{
  axisAngle->angle = 2.0f * acos(quartCalculated->SEq_4);
  axisAngle->x = quartCalculated->SEq_1 / (sqrt(1.0f-quartCalculated->SEq_4));
  axisAngle->y = quartCalculated->SEq_2 / (sqrt(1.0f-quartCalculated->SEq_4));
  axisAngle->z = quartCalculated->SEq_3 / (sqrt(1.0f-quartCalculated->SEq_4));
}

/*-----------------------------------------------------------------------------
 *      AxisAngle_to_Quarternion:  Converting an Axis Angle to quartenion
 *
 *  Parameters: const COMGS_quarternion *quartCalculated, const COMGS_axis_angle *axisAngle
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void AxisAngle_to_Quarternion( COMGS_quarternion *quartCalculated, const COMGS_axis_angle *axisAngle )
{
   quartCalculated->SEq_1= axisAngle->x * sin(axisAngle->angle/2.0f);
   quartCalculated->SEq_2 = axisAngle->y * sin(axisAngle->angle/2.0f);
   quartCalculated->SEq_3 = axisAngle->z * sin(axisAngle->angle/2.0f);
   quartCalculated->SEq_4 = cos(axisAngle->angle/2.0f);
}
/*-----------------------------------------------------------------------------
 *      computeEulerAngles:  pitch roll and yaw from quatenion as floats
 *
 *  Parameters: float32_t q0, float32_t q1, float32_t q2, float32_t q3, float32_t euler[3u]
 *
 *  from Hackflight Copyright (c) 2018 Simon D. Levy
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void computeEulerAngles(float32_t q0, float32_t q1, float32_t q2, float32_t q3, float32_t *euler[3u])
{
    float64_t a = 0.04f;
    float64_t b = 0.678f;
    float64_t d;
    /* *euler[0u] = atan2((2.0f*(q0*q1 + q2*q3)), (q0*q0 - q1*q1 - q2*q2 + q3*q3));   roll  replaced atan2 with safe checker macro */
    IKatan2(*euler[0u],(2.0f*(q0*q1 + q2*q3)),(q0*q0 - q1*q1 - q2*q2 + q3*q3));
    /* *euler[1u] =  asin((2.0f*(q1*q3 - q0*q2)));                                  pitch replaced asin with safe checker macro */
    IKsin(*euler[1u],(2.0f*(q1*q3 - q0*q2)));
    // *euler[2u] = atan2((2.0f*(q1*q2 + q0*q3)), (q0*q0 + q1*q1 - q2*q2 - q3*q3));   yaw (heading) replaced atan2 with safe checker macro
    IKatan2(*euler[2u],(2.0f*(q1*q2 + q0*q3)),(q0*q0 + q1*q1 - q2*q2 - q3*q3));
}
/*-----------------------------------------------------------------------------
 *      Quarternion_to_euler:  Convert the quartenion to euler co-ordinates
 *
 *  Parameters: const COMGS_quarternion *quartCalculated, COMGS_euler_angle *eulerCoord
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void Quarternion_to_euler( const COMGS_quarternion *quartCalculated, COMGS_euler_angle *eulerCoord )
{
   if (((quartCalculated->SEq_1 * quartCalculated->SEq_2) + (quartCalculated->SEq_3 * quartCalculated->SEq_4)) == 0.5f)         // north pole
   {
      eulerCoord->heading = 2.0f * atan2(quartCalculated->SEq_1,quartCalculated->SEq_4);
      eulerCoord->bank = 0.0f;
      eulerCoord->attitude = asin(2.0f*quartCalculated->SEq_1*quartCalculated->SEq_2 + 2.0f*quartCalculated->SEq_3*quartCalculated->SEq_4);
   } 
   else if (((quartCalculated->SEq_1 * quartCalculated->SEq_2) + (quartCalculated->SEq_3 * quartCalculated->SEq_4)) == -0.5f)     // south pole
   {
      eulerCoord->heading = 0.0f - (2.0f * atan2(quartCalculated->SEq_1,quartCalculated->SEq_4));
      eulerCoord->bank = 0.0f;
      eulerCoord->attitude = asin(2.0f*quartCalculated->SEq_1*quartCalculated->SEq_2 + 2.0f*quartCalculated->SEq_3*quartCalculated->SEq_4);
   }
   else
   {
     eulerCoord->heading = atan2(2.0f*quartCalculated->SEq_2*quartCalculated->SEq_4 - 2.0f*quartCalculated->SEq_1*quartCalculated->SEq_3 , 1.0f - 2.0f*pow(quartCalculated->SEq_2,2.0f) - 2.0f*pow(quartCalculated->SEq_3,2.0f));
     eulerCoord->attitude = asin(2.0f*quartCalculated->SEq_1*quartCalculated->SEq_2 + 2.0f*quartCalculated->SEq_3*quartCalculated->SEq_4);
     eulerCoord->bank = atan2(2.0f*quartCalculated->SEq_1*quartCalculated->SEq_4 - 2.0f*quartCalculated->SEq_2*quartCalculated->SEq_3 , 1.0f - 2.0f*pow(quartCalculated->SEq_1,2.0f) - 2.0f*pow(quartCalculated->SEq_3,2.0f));
   }

}

//**************************************************************************************************
//
// Euclidian: https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
//
//  Euler to quarternion and not via madgwick, mahony etc
//
/*-----------------------------------------------------------------------------
 *      euler_to_Quarternion():  euler to quartenion conversion function
 *
 *  Parameters: COMGS_quarternion *quartCalculated, const COMGS_euler_angle *eulerCoord
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void euler_to_Quarternion(COMGS_quarternion *quartCalculated, const COMGS_euler_angle *eulerCoord)

{

  float32_t c1 = cos(eulerCoord->heading/2.0f);                                 /* The Euler angles should be in radians. */
  float32_t s1 = sin(eulerCoord->heading/2.0f);
  float32_t c2 = cos(eulerCoord->attitude/2.0f);
  float32_t s2 = sin(eulerCoord->attitude/2.0f);
  float32_t c3 = cos(eulerCoord->bank/2.0f);
  float32_t s3 = sin(eulerCoord->bank/2.0f);
  float32_t c1c2 = c1*c2;
  float32_t s1s2 = s1*s2;

  float32_t w = c1c2*c3 - s1s2*s3;
  float32_t x = c1c2*s3 + s1s2*c3;
  float32_t y = s1*c2*c3 + c1*s2*s3;
  float32_t z = c1*s2*c3 - s1*c2*s3;

  quartCalculated->SEq_4 = w;                                                   /* quarternion calculated */
  quartCalculated->SEq_1 = x;
  quartCalculated->SEq_2 = y;
  quartCalculated->SEq_3 = z;

}

/* detect "overvoltage" if over 4.6 V and "undervoltage" if under 0.4 V,
   detection points set outside pot range to account for resistor tolerance
   and change of value over time/temperature -- this produces "fudge zones" between 401 and 499 mV
   and between 4501 and 4599 mV that are set to PotMin and 100% respectively 
   
   Function not needed ????
   
   */
   
/*-----------------------------------------------------------------------------
 *      calcAngle():  calculate angle from x,y co-ordinate
 *
 *  Parameters: float64_t magY, float64_t magX
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t calcAngle( float64_t magY, float64_t magX )
{
    return ((atan2(magY, magX) * 180.0f) / PI);
}
/*-----------------------------------------------------------------------------
 *      calcDist():  calculate distance from x,y co-ordinate
 *
 *  Parameters: float64_t magY, float64_t magX
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t calcDist( float64_t magY, float64_t magX )
{
    return (sqrt((magY*magY)+(magX*magX)));
}
/*-----------------------------------------------------------------------------
 *      calcAzi():  calculate azimuth from x,y co-ordinate
 *
 *  Parameters: float64_t magY, float64_t magX
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t calcAzi( float64_t magY, float64_t magX )
{
    return (90.0f - ((atan2(magY, magX) * 180.0f) / PI));
}
/*-----------------------------------------------------------------------------
 *      getXYfromAziDist():  calculate  x,y co-ordinate from azimuth & distance
 *
 *  Parameters: float64_t azi, float64_t dist
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void getXYfromAziDist( float64_t azi, float64_t dist, COMGS_coordObj_t *coOrd )
{
    coOrd->x = dist*sin(DEGREE_TO_RADIAN(azi));
    coOrd->y =  dist*cos(DEGREE_TO_RADIAN(azi));
}

/*
 * BCFlight
 * Copyright (C) 2016 Adrien Aubry (drich)  Ported by ACP Aviation
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
**/
/*-----------------------------------------------------------------------------
 *      defaultControllerExpo():  default controller expo
 *
 *  Parameters: quat *mExpo
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void defaultControllerExpo( quat *mExpo )
{
        if ( mExpo->x <= 0.0f ) {
                mExpo->x = 3.0f;
        }
        else if ( mExpo->x < 0.01f ) {
                mExpo->x = 0.01f;
        }

        if ( mExpo->y <= 0.0f ) {
                mExpo->y = 3.0f;
        }
        else if ( mExpo->y < 0.01f ) {
                mExpo->y = 0.01f;
        }

        if ( mExpo->z <= 0.0f ) {
                mExpo->z = 3.0f;
        }
        else if ( mExpo->z < 0.01f ) {
                mExpo->z = 0.01f;
        }

        if ( mExpo->w <= 0.0f ) {
                mExpo->w = 2.0f;
        }
        else if ( mExpo->w < 1.01f ) {
                mExpo->w = 1.01f;
        }
}
/*-----------------------------------------------------------------------------
 *      Controller_setRoll():  set roll from value and expo
 *
 *  Parameters: float32_t value, quat *mExpo, control_t *mRPY
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Controller_setRoll( float32_t value, quat *mExpo, control_t *mRPY )
{
        if ( value >= 0.0f ) {
                value = ( exp( value * mExpo->x ) - 1.0f ) / ( exp( mExpo->x ) - 1.0f );
        } else {
                value = -( exp( -value * mExpo->x ) - 1.0f ) / ( exp( mExpo->x ) - 1.0f );
        }
        mRPY->roll = value;
}
/*-----------------------------------------------------------------------------
 *      Controller_setPitch():  set pitch from value and expo
 *
 *  Parameters: float32_t value, quat *mExpo, control_t *mRPY
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Controller_setPitch( float32_t value, quat *mExpo, control_t *mRPY )
{
        if ( value >= 0.0f ) {
                value = ( exp( value * mExpo->y ) - 1.0f ) / ( exp( mExpo->y ) - 1.0f );
        } else {
                value = -( exp( -value * mExpo->y ) - 1.0f ) / ( exp( mExpo->y ) - 1.0f );
        }
        mRPY->pitch = value;
}
/*-----------------------------------------------------------------------------
 *      Controller_setYaw():  set yaw from value and expo
 *
 *  Parameters: float32_t value, quat *mExpo, control_t *mRPY
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Controller_setYaw( float32_t value, quat *mExpo, control_t *mRPY )
{
        if ( abs( value ) < 0.05f ) {
                value = 0.0f;
        }
        if ( value >= 0.0f ) {
                value = ( exp( value * mExpo->z ) - 1.0f ) / ( exp( mExpo->z ) - 1.0f );
        } else {
                value = -( exp( -value * mExpo->z ) - 1.0f ) / ( exp( mExpo->z ) - 1.0f );
        }
        mRPY->yaw = value;
}
/*-----------------------------------------------------------------------------
 *      Controller_setThrust():  set thrust from value and expo
 *
 *  Parameters: float32_t value, quat *mExpo, control_t *mRPY, CtrlStabilizer_t *thrustCtrl
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Controller_setThrust( float32_t value, quat *mExpo, control_t *mRPY, CtrlStabilizer_t *thrustCtrl )
{
        if ( abs( value ) < 0.05f ) {
                value = 0.0f;
        }
        if ( ! thrustCtrl->altitudeHold ) {
                /* value = log( value * ( mExpo->z - 1.0f ) + 1.0f ) / log( mExpo->z ); changed from original as i thought this should be expo for thrust not yaw */
                value = log( value * ( mExpo->w - 1.0f ) + 1.0f ) / log( mExpo->w );
                if ( value < 0.0f || IsaNan( value ) ) {
                        value = 0.0f;
                }
                if ( value > 1.0f  ) {
                        value = 1.0f;
                }
                mRPY->thrust = value;
        }
}
/*-----------------------------------------------------------------------------
 *      Controller_InitStabilizer():  initialize the stablizer
 *
 *  Parameters: Vectr *mHorizonMultiplier, float32_t *mRateFactor, Vectr *mHorizonMaxRate
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Controller_InitStabilizer( Vectr *mHorizonMultiplier, float32_t *mRateFactor, Vectr *mHorizonMaxRate )
{
     mHorizonMultiplier->x = 15.0f;                                             /* horizon multiplier vector initialisation */
     mHorizonMultiplier->y = 15.0f;
     mHorizonMultiplier->z = 1.0f;
     
     if ( *mRateFactor <= 0.0f ) {                                              /* stabilizer.rate_speed */
        *mRateFactor = 200.0f;
     }
     
     if ( mHorizonMaxRate->x <= 0.0f ) {                                        /* stabilizer.horizon_max_rate */
        mHorizonMaxRate->x = 0.0f;
     }
     else if ( mHorizonMaxRate->x >= 300.0f ) {
        mHorizonMaxRate->x = 300.0f;
     }

     if ( mHorizonMaxRate->y <= 0.0f ) {
        mHorizonMaxRate->y = 0.0f;
     }
     else if ( mHorizonMaxRate->y >= 300.0f ) {
        mHorizonMaxRate->y = 300.0f;
     }
     
     if ( mHorizonMaxRate->z <= 0.0f ) {
        mHorizonMaxRate->z = 0.0f;
     }
     else if ( mHorizonMaxRate->z >= 300.0f ) {
        mHorizonMaxRate->z = 300.0f;
     }
}
/*-----------------------------------------------------------------------------
 *      Controller_Stabilizer():  perform stabilization
 *
 *  Parameters: Vectr *mHorizonMultiplier, Vectr *mHorizonOffset, float32_t *mRateFactor, 
 *              control_t *ctrl, Vectr *mHorizonMaxRate, CtrlStabilizer_t  *thrustCtrl
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Controller_Stabilizer( Vectr *mHorizonMultiplier, Vectr *mHorizonOffset, float32_t *mRateFactor, control_t *ctrl, Vectr *mHorizonMaxRate, CtrlStabilizer_t  *thrustCtrl  )
{
        control_t control_angles;
        control_t rate_control;
        float32_t mAltitudeControl;

        /* We didn't take off yet (or really close to ground just after take-off), so we bypass stabilization, and just return */
        if ( thrustCtrl->airMode == false && ctrl->thrust <= 0.15f ) {
                return;
        }
        
        switch ( thrustCtrl->mode ) {
                case CTRLStabilRate :
                rate_control.roll = ctrl->roll * *mRateFactor;
                rate_control.pitch = ctrl->pitch * *mRateFactor;
                rate_control.yaw = ctrl->yaw * *mRateFactor;
                rate_control.thrust = ctrl->thrust * *mRateFactor;
                break;

                case CTRLStabilReturnToHome :
                case CTRLStabilFollow :
                case CTRLStabilStabilize :
                default :
                control_angles.roll = ctrl->roll;
                control_angles.pitch = ctrl->pitch;
                control_angles.yaw = ctrl->yaw;
                control_angles.roll = mHorizonMultiplier->x * min( max( control_angles.roll, -1.0f ), 1.0f ) + mHorizonOffset->x;
                control_angles.pitch = mHorizonMultiplier->y * min( max( control_angles.pitch, -1.0f ), 1.0f ) + mHorizonOffset->y;
                // TODO : when user-input is 0, set control_angles by using imu->velocity() to compensate position drifting
                /*  ====== does Horizon Control control_angles = desired set-point =====
                 mHorizonPID.Process( control_angles, imu->RPY(), dt );
                rate_control = mHorizonPID.state();      */
                rate_control.roll = max( -mHorizonMaxRate->x, min( mHorizonMaxRate->x, rate_control.roll ) );
                rate_control.pitch = max( -mHorizonMaxRate->y, min( mHorizonMaxRate->y, rate_control.pitch ) );
                rate_control.yaw = control_angles.yaw * *mRateFactor; // TEST : Bypass heading for now
                break;
        }
        /* ==== does rate control rate_control is the desired setpoint =======
        mRateRollPID.Process( rate_control.x, imu->rate().x, dt );
        mRatePitchPID.Process( rate_control.y, imu->rate().y, dt );
        mRateYawPID.Process( rate_control.z, imu->rate().z, dt );
        */
        if ( thrustCtrl->altitudeHold ) {
                rate_control.thrust = rate_control.thrust * 2.0f - 1.0f;
                if ( abs( rate_control.thrust ) < 0.1f ) {
                        rate_control.thrust = 0.0f;
                } else if ( rate_control.thrust > 0.0f ) {
                        rate_control.thrust = ( rate_control.thrust - 0.1f ) * ( 1.0f / 0.9f );
                } else if ( rate_control.thrust < 0.0f ) {
                        rate_control.thrust = ( rate_control.thrust + 0.1f ) * ( 1.0f / 0.9f );
                }
                rate_control.thrust *= 0.01f; // Reduce to 1m/s
                mAltitudeControl += rate_control.thrust;
                /* ======= does attitude control mAltitudeControl is the desired setpoint =====
                 mAltitudePID.Process( mAltitudeControl, imu->altitude(), dt ); */
                rate_control.thrust = mAltitudeControl;
        }
}

/*
BSD 2-Clause License

Copyright 2014-2019 Markus Giftthaler, Michael Neunert, Markus Stäuble.
Copyright 2014-2018 ETH Zurich.  Ported for PIC32 by ACP Aviation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*-----------------------------------------------------------------------------
 *      A_quadrotor():  A Quadrotor control
 *
 *  Parameters: state_control_vector_t *x
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void A_quadrotor(state_control_vector_t *x)
{
    float64_t qph =  x->state[3u];
    float64_t qth =  x->state[4u];
    float64_t qps =  x->state[5u];
    float64_t dqph =  x->state[9u];
    float64_t dqth =  x->state[10u];
    float64_t dqps =  x->state[11u];
    float64_t Fz = x->control[0u];
    float64_t Mx = x->control[1u];
    float64_t My = x->control[2u];

    float64_t t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23, t24, t25;

    t2 = 1.0f / mQ;
    t3 = cos(qth);
    t4 = sin(qph);
    t5 = cos(qph);
    t6 = sin(qth);
    t7 = 1.0f / Thxxyy;
    t8 = cos(qps);
    t9 = sin(qps);
    t10 = 1.0f / t3;
    t11 = Thxxyy * 2.0f;
    t12 = Thzz - t11;
    t13 = qth * 2.0f;
    t14 = cos(t13);
    t15 = My * t9;
    t16 = sin(t13);
    t17 = 1.0f / (t3 * t3);
    t18 = qth * 3.0f;
    t19 = sin(t18);
    t20 = My * t8;
    t21 = Mx * t9;
    t22 = t20 + t21;
    t23 = t3 * t3;
    t24 = t6 * t6;
    t25 = Thzz * dqps * t6;

    memset((void*)&x->state_matrix,0u,sizeof(x->state_matrix));
    x->state_matrix[0u][6u] = 1.0f;
    x->state_matrix[1u][7u] = 1.0f;
    x->state_matrix[2u][8u] = 1.0f;
    x->state_matrix[3u][9u] = 1.0f;
    x->state_matrix[4u][10u] = 1.0f;
    x->state_matrix[5u][11u] = 1.0f;
    x->state_matrix[6u][4u] = Fz * t2 * t3;
    x->state_matrix[7u][3u] = -Fz * t2 * t3 * t5;
    x->state_matrix[7u][4u] = Fz * t2 * t4 * t6;
    x->state_matrix[8u][3u] = -Fz * t2 * t3 * t4;
    x->state_matrix[8u][4u] = -Fz * t2 * t5 * t6;
    x->state_matrix[9u][4u] = -t6 * t7 * t17 * (t15 - Mx * t8 + Thzz * dqps * dqth - Thxxyy * dqph * dqth * t6 * 2.0f + Thzz * dqph * dqth * t6) - dqph * dqth * t7 * t12;
    x->state_matrix[9u][5u] = -t7 * t10 * t22;
    x->state_matrix[9u][9u] = -dqth * t6 * t7 * t10 * t12;
    x->state_matrix[9u][10u] = -t7 * t10 * (Thzz * dqps - Thxxyy * dqph * t6 * 2.0f + Thzz * dqph * t6);
    x->state_matrix[9u][11u] = -Thzz * dqth * t7 * t10;
    x->state_matrix[10u][4u] = -dqph * t7 * (t25 + Thxxyy * dqph * t14 - Thzz * dqph * t14);
    x->state_matrix[10u][5u] = -t7 * (t15 - Mx * t8);
    x->state_matrix[10u][9u] = t7 * (-Thxxyy * dqph * t16 + Thzz * dqps * t3 + Thzz * dqph * t16);
    x->state_matrix[10u][11u] = Thzz * dqph * t3 * t7;
    x->state_matrix[11u][4u] = t7 * t17 * (Mx * t8 * -4.0f + My * t9 * 4.0f + Thzz * dqps * dqth * 4.0f - Thxxyy * dqph * dqth * t6 * 9.0f - Thxxyy * dqph * dqth * t19 + Thzz * dqph * dqth * t6 * 5.0f + Thzz * dqph * dqth * t19) * (1.0f / 4.0f);
    x->state_matrix[11u][5u] = t6 * t7 * t10 * t22;
    x->state_matrix[11u][9u] = dqth * t7 * t10 * (Thzz - t11 + Thxxyy * t23 - Thzz * t23);
    x->state_matrix[11u][10u] = t7 * t10 * (t25 - Thxxyy * dqph - Thxxyy * dqph * t24 + Thzz * dqph * t24);
    x->state_matrix[11u][11u] = Thzz * dqth * t6 * t7 * t10;

}
/*-----------------------------------------------------------------------------
 *      B_quadrotor():  B Quadrotor control
 *
 *  Parameters: state_control_vector_t *x
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void B_quadrotor(state_control_vector_t *x)
{
    float64_t qph = x->state[3u];                                               // euler angles xyz
    float64_t qth = x->state[4u];
    float64_t qps = x->state[5u];

    float64_t t2, t3, t4, t5, t6, t7, t8;

    t2 = 1.0f / mQ;
    t3 = cos(qth);
    t4 = 1.0f / Thxxyy;
    t5 = 1.0f / t3;
    t6 = sin(qps);
    t7 = cos(qps);
    t8 = sin(qth);

    memset((void*)&x->gain_matrix,0u,sizeof(x->gain_matrix));
    x->gain_matrix[6u][0u] = t2 * t8;
    x->gain_matrix[7u][0u] = -t2 * t3 * sin(qph);
    x->gain_matrix[8u][0u] = t2 * t3 * cos(qph);
    x->gain_matrix[9u][1u] = t4 * t5 * t7;
    x->gain_matrix[9u][2u] = -t4 * t5 * t6;
    x->gain_matrix[10u][1u] = t4 * t6;
    x->gain_matrix[10u][2u] = t4 * t7;
    x->gain_matrix[11u][1u] = -t4 * t5 * t7 * t8;
    x->gain_matrix[11u][2u] = t4 * t5 * t6 * t8;
    x->gain_matrix[11u][3u] = 1.0f / Thzz;

}
/*-----------------------------------------------------------------------------
 *      C_quadrotor():  C Quadrotor control
 *
 *  Parameters: state_control_vector_t *x
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void C_quadrotor(state_control_vector_t *x)
{
    //        double qxQ = x(0);
    //        double qyQ = x(1);
    //        double qzQ = x(2);
    float64_t qph = x->state[3u];
    float64_t qth = x->state[4u];
    float64_t qps = x->state[5u];
    //        double dqxQ = x(6);
    //        double dqyQ = x(7);
    //        double dqzQ = x(8);
    //        double dqph = x(9);
    //        double dqth = x(10);
    //        double dqps = x(11);

    float64_t t2, t3, t4, t5, t6, t7, t8;  // t9, t10, t11;

    t2 = 1.0f / mQ;
    t3 = cos(qth);
    t4 = 1.0f / Thxxyy;
    t5 = 1.0f / t3;
    t6 = sin(qps);
    t7 = cos(qps);
    t8 = sin(qth);

    memset((void*)&x->gain_matrixC,0u,sizeof(x->gain_matrixC));
    x->gain_matrixC[6u][0u] = t2 * t8 * (1.0f / 2.0E1f);
    x->gain_matrixC[7u][0u] = t2 * t3 * sin(qph) * (-1.0f / 2.0E1f);
    x->gain_matrixC[8u][0u] = t2 * t3 * cos(qph) * (1.0 / 2.0E1f);
    x->gain_matrixC[9u][1u] = t4 * t5 * t7 * (1.0f / 2.0E1f);
    x->gain_matrixC[9u][2u] = t4 * t5 * t6 * (-1.0f / 2.0E1f);
    x->gain_matrixC[10u][1u] = t4 * t6 * (1.0f / 2.0E1f);
    x->gain_matrixC[10u][2u] = t4 * t7 * (1.0f / 2.0E1f);
    x->gain_matrixC[11u][1u] = t4 * t5 * t7 * t8 * (-1.0f / 2.0E1f);
    x->gain_matrixC[11u][2u] = t4 * t5 * t6 * t8 * (1.0f / 2.0E1f);
    x->gain_matrixC[11u][3u] = (1.0f / 2.0E1f) / Thzz;

}
/*-----------------------------------------------------------------------------
 *      quadrotor_ode():  C Quadrotor control
 *
 *  Parameters: state_control_vector_t *x
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void quadrotor_ode(state_control_vector_t *x)
{
    float64_t x_norm=0.0f;
    uint8_t i;
    float64_t t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13;

    if (x == NULL)
    {
       // positions
       //    float64_t qxQ = x(0);  // x
       //    float64_t qyQ = x(1);  // y
       //    float64_t qzQ = x(2);  // z

       float64_t qph = x->state[3u];                                            // euler angles xyz
       float64_t qth = x->state[4u];
       float64_t qps = x->state[5u];

       float64_t dqxQ = x->state[6u];                                           // positions derivatives x
       float64_t dqyQ = x->state[7u];                                           // y
       float64_t dqzQ = x->state[8u];                                           // z

       float64_t dqph = x->state[9u];                                           // euler angle derivatives xyz
       float64_t dqth = x->state[10u];
       float64_t dqps = x->state[11u];

       float64_t Fz = x->control[0u];                                           // Applied force and momentums
       float64_t Mx = x->control[1u];
       float64_t My = x->control[2u];
       float64_t Mz = x->control[3u];

       t2 = 1.0f / mQ;
       t3 = cos(qth);
       t4 = sin(qth);
       t5 = 1.0f / Thxxyy;
       t6 = cos(qps);
       t7 = sin(qps);
       t8 = dqph * dqph;
       t9 = qth * 2.0f;
       t10 = sin(t9);
       t11 = 1.0f / t3;
       t12 = Thzz * Thzz;
       t13 = t3 * t3;

       memset((void*)&x->dx,0,sizeof(x->dx));
       x->dx[0u] = dqxQ;
       x->dx[1u] = dqyQ;
       x->dx[2u] = dqzQ;
       x->dx[3u] = dqph;
       x->dx[4u] = dqth;
       x->dx[5u] = dqps;
       x->dx[6u] = Fz * t2 * t4;
       x->dx[7u] = -Fz * t2 * t3 * sin(qph);
       x->dx[8u] = t2 * (mQ * 9.81E2f - Fz * t3 * cos(qph) * 1.0E2f) * (-1.0f / 1.0E2f);
       x->dx[9u] = -t5 * t11 * (-Mx * t6 + My * t7 + Thzz * dqps * dqth - Thxxyy * dqph * dqth * t4 * 2.0f + Thzz * dqph * dqth * t4);
       x->dx[10u] = t5 * (Mx * t7 + My * t6 - Thxxyy * t8 * t10 * (1.0f / 2.0f) + Thzz * t8 * t10 * (1.0f / 2.0f) +  Thzz * dqph * dqps * t3);
       x->dx[11u] = (t5 * t11 * (Mz * Thxxyy * t3 + dqph * dqth * t12 - dqph * dqth * t12 * t13 + dqps * dqth * t4 * t12 - Thxxyy * Thzz * dqph * dqth * 2.0f - Mx * Thzz * t4 * t6 + My * Thzz * t4 * t7 + Thxxyy * Thzz * dqph * dqth * t13)) / Thzz;

       for (i=0u;i < 12u;i++)                                                   // a hacky check to prevent integration from becoming unstable:
              x_norm += x->state[i];
       if (x_norm > 1e20f)
             memset((void*)&x->dx,0,sizeof(x->dx));
   }
}

//                    Differential Steering Joystick Algorithm
// =============================================================================
//                           by Calvin Hass
//   https://www.impulseadventure.com/elec/
//
// Converts a single dual-axis joystick into a differential
// drive motor control, with support for both drive, turn
// and pivot operations.
//
//
// INPUTS
//int     nJoyX;              // Joystick X input                     (-128..+127)
//int     nJoyY;              // Joystick Y input                     (-128..+127)
//
// OUTPUTS
//int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
//int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

void JoyStick2DiffSteer( int16_t nJoyX, int16_t nJoyY, int16_t nMotMixL, int16_t nMotMixR)
{
  // CONFIG
  // - fPivYLimt  : The threshold at which the pivot action starts
  //                This threshold is measured in units on the Y-axis
  //                away from the X-axis (Y=0). A greater value will assign
  //                more of the joystick's range to pivot actions.
  //                Allowable range: (0..+127)
  float32_t fPivYLimit = 32.0f;

  // TEMP VARIABLES
  float32_t   nMotPremixL;                                                      // Motor (left)  premixed output        (-128..+127)
  float32_t   nMotPremixR;                                                      // Motor (right) premixed output        (-128..+127)
  int16_t     nPivSpeed;                                                        // Pivot Speed                          (-128..+127)
  float32_t   fPivScale;                                                        // Balance scale b/w drive and pivot    (   0..1   )

  if (nJoyY >= 0)                                                               // Calculate Drive Turn output due to Joystick X input
  {
    nMotPremixL = (nJoyX>=0)? 127.0f : (127.0f + nJoyX);                          // Forward
    nMotPremixR = (nJoyX>=0)? (127.0f - nJoyX) : 127.0f;
  }
  else
  {
    nMotPremixL = (nJoyX>=0)? (127.0f - nJoyX) : 127.0f;                          // Reverse
    nMotPremixR = (nJoyX>=0)? 127.0f : (127.0f + nJoyX);
  }

  nMotPremixL = nMotPremixL * nJoyY/128.0f;                                      // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixR = nMotPremixR * nJoyY/128.0f;
                                                                                // Now calculate pivot amount
  nPivSpeed = nJoyX;                                                            // - Strength of pivot (nPivSpeed) based on Joystick X input
  fPivScale = (abs(nJoyY)>fPivYLimit)? 0.0f : (1.0f - abs(nJoyY)/fPivYLimit);     // - Blending of pivot vs drive (fPivScale) based on Joystick Y input

  nMotMixL = (1.0f-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);              // Calculate final mix of Drive and Pivot
  nMotMixR = (1.0f-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);

}

// Bearing based Steering Joystick Algorithm
// ========================================
//   by Calvin Hass
//   https://www.impulseadventure.com/elec/
//
// Converts a single dual-axis joystick into a Bearing
// drive motor control, with support for both drive, turn
// and pivot operations.
//
//
// INPUTS
//int     nJoyX;              // Joystick X input                     (-128..+127)
//int     nJoyY;              // Joystick Y input                     (-128..+127)
//
// OUTPUTS
//int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
//int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

void JoyStick2BearSteer( int16_t nJoyX, int16_t nJoyY, int16_t nMotMixL, int16_t nMotMixR)
{
  // CONFIG
  float32_t fPivBearLimit = 75.0f;                                               // Bearing threshold for pivot action (degrees)

  // TEMP VARIABLES
  float32_t nMotPremixL;                                                        // Motor (left)  premixed output        (-128..+127)
  float32_t nMotPremixR;                                                        // Motor (right) premixed output        (-128..+127)
  int16_t nPivSpeed;                                                            // Pivot Speed                          (-128..+127)
  float32_t fPivScale;                                                          // Balance scale b/w drive and pivot    (   0..1   )
  float32_t fBearMag;

  if (nJoyY >= 0)                                                               // Calculate Drive Turn output due to Joystick X input
  {
    nMotPremixL = (nJoyX>=0)? 127.0f : (127.0f + nJoyX);                          // Forward
    nMotPremixR = (nJoyX>=0)? (127.0f - nJoyX) : 127.0f;
  }
  else
  {
    nMotPremixL = (nJoyX>=0)? (127.0f - nJoyX) : 127.0f;                          // Reverse
    nMotPremixR = (nJoyX>=0)? 127.0f : (127.0f + nJoyX);
  }

  nMotPremixL = nMotPremixL * nJoyY/128.0f;                                      // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixR = nMotPremixR * nJoyY/128.0f;

  if (nJoyY == 0)
  {
    if (nJoyX == 0)                                                             // Handle special case of Y-axis=0
    {
      fBearMag = 0;
    }
    else
    {
      fBearMag = 90;
    }
  }
  else
  {
    // Bearing (magnitude) away from the Y-axis is calculated based on the
    // Joystick X & Y input. The arc-tangent angle is then converted
    // from radians to degrees.
    fBearMag = atan( (float32_t)abs(nJoyX) / (float32_t)abs(nJoyY) ) *90.0f/(3.14159f/2.0f);
  }

  // Now calculate pivot amount
  // Blending of pivot vs drive (fPivScale) based on Joystick bearing
  nPivSpeed = nJoyX;
  fPivScale = (fBearMag<fPivBearLimit)? 0.0f : (fBearMag-fPivBearLimit)/(90.0f-fPivBearLimit);

  nMotMixL = (1.0f-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);              // Calculate final mix of Drive and Pivot
  nMotMixR = (1.0f-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);

}

//
// Another set of speed corrections from L Robbins for speed values passed to a controller from a joystick
// these can also be tested during the test phase
//
/*-----------------------------------------------------------------------------
 *      JoyStick2Speed():  joystick smooth algorythms by L Robbins
 *
 *  Parameters: int16_t nJoyX, uint8_t choice
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
int16_t JoyStick2Speed( int16_t nJoyX, uint8_t choice )
{
    switch (choice)
    {
      case 1u:
        return (int16_t) ((((float64_t)nJoyX) + pow(((float64_t)nJoyX),2.0f))/2.0f);         // option 1
      case 2u:
        return (int16_t) (((2.0f*((float64_t)nJoyX)) + pow(((float64_t)nJoyX),2.0f))/3.0f);  // option 2
      case 3u:
        return (int16_t) ((((float64_t)nJoyX)+(2.0f*pow(((float64_t)nJoyX),2.0f)))/3.0f);    // option 3
      case 4u:
         return (int16_t) (pow(((float64_t)nJoyX),2.0f));                       // option 4
      default:
         return(nJoyX);                                                         // option not specified then return it back
    }
}

/*-----------------------------------------------------------------------------
 *      moving_average():  moving average for gyro acceleromter data
 *
 *  Parameters: COMGS_gyro_acc_data_t *accStruct
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void moving_average( COMGS_gyro_acc_data_t *accStruct )
{
  if ((accStruct->acc_roll <=3) && (accStruct->acc_roll >= -3))                 // test for near zero and eliminate for each axis
  {
     accStruct->acc_roll = 0;
     accStruct->zero_acc_cnt_roll=++accStruct->zero_acc_cnt_roll % UINT8_MAX;
  }
  if ((accStruct->acc_yaw <=3) && (accStruct->acc_yaw >= -3))
  {
     accStruct->acc_yaw = 0;
     accStruct->zero_acc_cnt_yaw=++accStruct->zero_acc_cnt_yaw % UINT8_MAX;
  }
  if ((accStruct->acc_pitch <=3) && (accStruct->acc_pitch >= -3))
  {
     accStruct->acc_pitch = 0;
     accStruct->zero_acc_cnt_pitch=++accStruct->zero_acc_cnt_pitch % UINT8_MAX;
  }

  if (accStruct->g_count_sam!=0x40u)                                            // iterate this function 64 times before writing result and reseting  (moving average filter)
  {
    accStruct->mavg_acc_roll=accStruct->mavg_acc_roll + accStruct->acc_roll;
    accStruct->mavg_acc_pitch=accStruct->mavg_acc_pitch + accStruct->acc_pitch;
    accStruct->mavg_acc_yaw=accStruct->mavg_acc_yaw + accStruct->acc_yaw;
    accStruct->g_count_sam=++accStruct->g_count_sam % UINT8_MAX;                // count number of samples
  }
  else
  {
     accStruct->mavg_acc_roll = ( ((uint32_t) accStruct->mavg_acc_roll) >> 6u); // divide by 64 the sum
     accStruct->mavg_acc_pitch = ( ((uint32_t) accStruct->mavg_acc_pitch) >> 6u);
     accStruct->mavg_acc_yaw = ( ((uint32_t) accStruct->mavg_acc_yaw) >> 6u);

     accStruct->mavg_acc_rolls[1u] = accStruct->mavg_acc_roll;                   // copy current moving average to actual moving average values stored as 1 and 2
     accStruct->mavg_acc_pitchs[1u] = accStruct->mavg_acc_pitch;
     accStruct->mavg_acc_yaws[1u] = accStruct->mavg_acc_yaw;
     accStruct->g_count_sam=0u;                                                 // reset and start collecting another 64 samples
     accStruct->g_count_acc=++accStruct->g_count_acc % UINT8_MAX;               // Rolling Counter (msg id)
  }

}

// see https://github.com/walchko/ahrs for more info.
inline float64_t calcHeadRoll( quat *q ){ return atan2(2.0f*q->z*q->w-2.0f*q->x*q->y,2.0f*q->x*q->x+2.0f*q->w*q->w-1.0f); }
inline float64_t calcHeadPitch( quat *q ){ return -asin(2.0f*q->y*q->w+2.0f*q->x*q->z); }
inline float64_t calcHeadYaw( quat *q ){ return atan2(2.0f*q->y*q->z-2.0f*q->x*q->w,2.0f*q->x*q->x+2.0f*q->y*q->y-1.0f); }

// from "tilt compensated compass.pdf" (LSM303DLH tech note)
// calculate heading from magnetometers and return heading in degrees.
// uses mag reading from lsm303 and quat from madgewick / mahony
/*-----------------------------------------------------------------------------
 *      calcHeadingFromMag():  get heading from mag and madgewick/mahony quat
 *
 *  Parameters: float64_t mx, float64_t my, float64_t mz, quat *q
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t calcHeadingFromMag(float64_t mx, float64_t my, float64_t mz, quat *q)
{
            float64_t heading = 0.0f;
            
            float64_t norm = sqrt(mx*mx+my*my+mz*mz);
            float64_t mx1 = mx/norm;
            float64_t my1 = my/norm;
            float64_t mz1 = mz/norm;
            
            float64_t r = calcHeadRoll(q);                                          // roll
            float64_t g = calcHeadPitch(q);                                         // pitch
            
            float64_t mx2 = mx1*cos(r)+mz1*sin(r);
            float64_t my2 = mx1*sin(g)*sin(r)+my1*cos(g)-mz1*sin(g)*sin(r);
            float64_t mz2 = -mx1*cos(g)*sin(r)+my1*sin(g)+mz1*cos(g)*cos(r);
            
            heading = atan2(my2,mx2);                                               // dboule check this
            
            if(mx2 > 0.0f && my2 >= 0.0f);                                          // all good 
            else if( mx2 < 0.0f) heading = PI+heading;
            else if(mx2>0.0f && my2 <=0.0f) heading = 2.0f*PI+heading;
            else if(mx2 == 0.0f && my2 < 0.0f) heading = PI/2.0f;                   // 90 deg
            else if(mx2 == 0.0f && my2 > 0.0f) heading = 1.5f*PI;                   // 270 deg
            
            return 180.0f/PI*heading;
}
/*-----------------------------------------------------------------------------
 *      calculate_velocity_position():  calculate velocity position
 *
 *  Parameters: COMGS_gyro_acc_data_t *accStruct
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void calculate_velocity_position( COMGS_gyro_acc_data_t *accStruct )
{

   if (accStruct->zero_acc_cnt_roll >= MAX_ZERO_ACCS)
   {
      accStruct->vel_roll[0u] = 0;
      accStruct->vel_roll[1u] = 0;
   }
   if (accStruct->zero_acc_cnt_pitch >= MAX_ZERO_ACCS)
   {
      accStruct->vel_pitch[0u] = 0;
      accStruct->vel_pitch[1u] = 0;
   }
   if (accStruct->zero_acc_cnt_yaw >= MAX_ZERO_ACCS)
   {
      accStruct->vel_yaw[0u] = 0;
      accStruct->vel_yaw[1u] = 0;
   }

   // 1st iteration
   accStruct->vel_roll[1u] = accStruct->vel_roll[0u] + accStruct->mavg_acc_rolls[0u] + ((accStruct->mavg_acc_rolls[1u] - accStruct->mavg_acc_rolls[0u])>>1u);
   accStruct->vel_pitch[1u] = accStruct->vel_pitch[0u] + accStruct->mavg_acc_pitchs[0u] + ((accStruct->mavg_acc_pitchs[1u] - accStruct->mavg_acc_pitchs[0u])>>1u);
   accStruct->vel_yaw[1u] = accStruct->vel_yaw[0u] + accStruct->mavg_acc_yaws[0u] + ((accStruct->mavg_acc_yaws[1u] - accStruct->mavg_acc_yaws[0u])>>1u);

   // 2nd iteration
   accStruct->pos_roll[1u] = accStruct->pos_roll[0u] + accStruct->vel_roll[0u] + ((accStruct->vel_roll[1u] - accStruct->vel_roll[0u])>>1u);
   accStruct->pos_pitch[1u] = accStruct->pos_pitch[0u] + accStruct->vel_pitch[0u] + ((accStruct->vel_pitch[1u] - accStruct->vel_pitch[0u])>>1u);
   accStruct->pos_yaw[1u] = accStruct->pos_yaw[0u] + accStruct->vel_yaw[0u] + ((accStruct->vel_yaw[1u] - accStruct->vel_yaw[0u])>>1u);

  // Copy current velocity and acceleration from previous
  accStruct->vel_roll[0u] = accStruct->vel_roll[1u];
  accStruct->vel_pitch[0u] = accStruct->vel_pitch[1u];
  accStruct->vel_yaw[0u] = accStruct->vel_yaw[1u];
  accStruct->mavg_acc_rolls[0u] = accStruct->mavg_acc_rolls[1u];
  accStruct->mavg_acc_pitchs[0u] = accStruct->mavg_acc_pitchs[1u];
  accStruct->mavg_acc_yaws[0u] = accStruct->mavg_acc_yaws[1u];
  accStruct->pos_roll[0u] =  accStruct->pos_roll[1u];
  accStruct->pos_pitch[0u] = accStruct->pos_pitch[1u];
  accStruct->pos_yaw[0u] = accStruct->pos_yaw[1u];

}

// Function to calculate theta
// theta (pitch)=atan2(Rx/sqrt(Ry^2+Rz^2)
// theta (roll)= atan2(Ry/sqrt(Rx^2+Rz^2)
// theta (yaw)= atan2(Rz/sqrt(Rx^2+Ry^2)
//
/*-----------------------------------------------------------------------------
 *      calculate_theta():  calculate theta from gyro object data
 *
 *  Parameters: COMGS_gyro_acc_data_t *accStruct
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void calculate_theta(COMGS_gyro_acc_data_t *accStruct)
{
    accStruct->theta_yaw = atan2( (float64_t) accStruct->mavg_acc_yaws[1u], sqrt( pow((float64_t)accStruct->mavg_acc_pitchs[1u],2.0f)+ pow((float64_t)accStruct->mavg_acc_rolls[1u],2.0f) ) );
    accStruct->theta_roll = atan2( (float64_t) accStruct->mavg_acc_rolls[1u], sqrt( pow((float64_t)accStruct->mavg_acc_pitchs[1u],2.0f)+ pow((float64_t)accStruct->mavg_acc_yaws[1u],2.0f) ) );
    accStruct->theta_pitch = atan2( (float64_t) accStruct->mavg_acc_pitchs[1u], sqrt( pow((float64_t)accStruct->mavg_acc_yaws[1u],2.0f)+ pow((float64_t)accStruct->mavg_acc_rolls[1u],2.0f) ) );
}
// The first and third Euler angles in the sequence (phi and psi) become
// unreliable when the middle angles of the sequence (theta) approaches ±90
// degrees. This problem commonly referred to as Gimbal Lock.
// See: http://en.wikipedia.org/wiki/Gimbal_lock    -- possibly need to monitor ?

/*-----------------------------------------------------------------------------
 *      popAlmOffStack():  remove alarm from alarm stack (called from Process_Alarm_Ack) 
 *                         not globally defined
 *
 *  Parameters: AlarmStack_t *alarmStack, const AlarmDefinitionObject_t *almDesc, 
 *              uint8_t indexInAlmDefObj
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void popAlmOffStack( AlarmStack_t *alarmStack, const AlarmDefinitionObject_t *almDesc, uint8_t indexInAlmDefObj )
{
    uint8_t elementNo,elementNo2;                                               // indexs in the arrays

    if ((indexInAlmDefObj >= 9u) || (indexInAlmDefObj <= 0u))                     // invalid structure position for the tag number has been passed
    {
       return;                                                                  // return function use invalid
    }
    
    for(elementNo=1;elementNo<=alarmStack->numObjects;elementNo++)              // for stack start to the end, check if its in the stack
    {
      if (alarmStack->AlarmStack[elementNo] == almDesc->objectNo[indexInAlmDefObj+8u]) // match was found actual id found in AlarmDefinitionObject_t offset by 8
      {
         for(elementNo2=elementNo+1;elementNo2<=alarmStack->numObjects;elementNo2++)   // from the next position down to the end of the stack
         {
           alarmStack->AlarmStack[elementNo2-1u] = alarmStack->AlarmStack[elementNo2];   // remove and shift the remainder of the stack upwards
         }
         return;                                                                // excercise is complete
      }
    }
    
}

/*-----------------------------------------------------------------------------
 *      pushAlmToStack():  Add a new alarm to the display stack  returns without 
 *                         a code if already in stack or invalid index is passed 
 *                        (called from Process_Alarm_Ack) not globally defined
 *
 *  Parameters: AlarmStack_t *alarmStack, const AlarmDefinitionObject_t *almDesc, 
 *              uint8_t positionInobj
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void pushAlmToStack( AlarmStack_t *alarmStack, const AlarmDefinitionObject_t *almDesc, uint8_t positionInobj )
{
    uint8_t elementNo;                                                          // index in the array
    
    if ((positionInobj >= 9u) || (positionInobj <= 0u))                           // invalid structure position for the tag number has been passed
    {
       return;                                                                  // return function use invalid
    }
    
    if (alarmStack->numObjects >= 0u)                                            // not first element in stack
    {
       for(elementNo=1u;elementNo<=alarmStack->numObjects;elementNo++)           // for stack to the end
       {
           if (alarmStack->AlarmStack[elementNo] == almDesc->objectNo[positionInobj+8u])   // check if its already in the stack if so do nothing, object id starts after descriptions (+8)
           {
              return;                                                           // so return
           }
       }
      alarmStack->AlarmStack[alarmStack->numObjects+1] = almDesc->objectNo[positionInobj+8u];    // place at next position in the queue stack, object id starts after descriptions (+8)
      if (alarmStack->numObjects <= TOTAL_NO_OF_ALARMS ) alarmStack->numObjects=++alarmStack->numObjects % UINT8_MAX; // increment the alarm stack index
    }
    else
    {
       alarmStack->AlarmStack[1] = almDesc->objectNo[positionInobj+8u];          // place at the head of the queue, object id starts after descriptions (+8)
       if (alarmStack->numObjects <= TOTAL_NO_OF_ALARMS ) alarmStack->numObjects=++alarmStack->numObjects % UINT8_MAX;  // increment the alarm stack index
    }
}

/*-----------------------------------------------------------------------------
 *      Update_Alm_Line():  Display the Alarm Stack to a banner defined by the AlarmLineObject
 *
 *  Parameters: const AlarmStack_t *alarmStack, const AlarmDefinitionObject_t *almDesc, 
 *              AlarmLineObject_t *almLineObj
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Update_Alm_Line( const AlarmStack_t *alarmStack, const AlarmDefinitionObject_t *almDesc, AlarmLineObject_t *almLineObj )
{

   if (alarmStack->numObjects <= 0u)                                             // nothing to display
   {
      strcpy(&almLineObj->Caption,"          \n");                              // blank display
      almLineObj->CapVisible = false;
      almLineObj->AckVisible = false;
   }
   else
   {
     almLineObj->CapVisible = true;                                             // we have an alarm make it have caption and visible acknowledge button
     almLineObj->AckVisible = true;
     almLineObj->AckActive = true;
     if ((alarmStack->AlarmStack[1u] >= almDesc->objectNo[1u]) && (alarmStack->AlarmStack[1u] <= almDesc->objectNo[8u]))
     {                                                                          // Alarm at top of stack is in range of this Alarm Definition Object passed
        if ((alarmStack->AlarmStack[1u]%8u) == 0u)                                 // its element 8 of a definition object thats at position 1 in the stack
        {
          strcpy(&almLineObj->Caption,(char*) &almDesc->Caption[8u]);           // take description 8 as its the last one
        }
        else
        {
          strcpy(&almLineObj->Caption,(char*) &almDesc->Caption[(alarmStack->AlarmStack[1u]%8u)]);   // take description of index mod 8 (elements 1-7)
        }
     }
   }
}

/*-----------------------------------------------------------------------------
 *      Process_alarm_ack():  Process the Alarm word and HMI Acknowledge button 
 *                            to cancel it  (called from Process_Alarm_Ack) 
 *                            not globally defined
 *
 *  Parameters: COMGS_error_alarm *almWord, AlarmStack_t *alarmStack, 
 *              const AlarmDefinitionObject_t *almDesc, AlarmLineObject_t *almLineObj
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Process_alarm_ack( COMGS_error_alarm *almWord, AlarmStack_t *alarmStack, const AlarmDefinitionObject_t *almDesc, AlarmLineObject_t *almLineObj )
{
   uint8_t counterAlm,ackBit;                                                   // counter to iterate each bit in the word
   
   if ( almWord->AlarmWdAck != almWord->AlarmWdAckLastScan )                    // Ack was pressed on the HMI (one-shot)
   {
     almWord->AlarmWdAckLastScan = almWord->AlarmWdAck;                         // Make the condition one shot
     almWord->AlarmTmRun = true;                                                // Start delay off (DOFF) timer
     for(ackBit=1u;ackBit<=8u;ackBit++)                                           // for all the bits in the word
     {                                                                          // if the ack is on call to remove from the stack
        if (almWord->AlarmWdAck & ((uint8_t) pow(2.0f,((float64_t) ackBit))))      // the ack bit was ON
        {
           popAlmOffStack(alarmStack, almDesc, ((uint8_t) pow(2.0f,((float64_t) ackBit)))); // remove the alarm that was acknowledged from the stack and GUI
        }
     }
   }
   if ( almWord->AlarmTmRun == true )                                           // You had an ACK pressed
   {
      almWord->AlarmTm=++almWord->AlarmTm % UINT16_MAX;                                          // Increment ACK timer
   }
   if ( almWord->AlarmTm > COMGS_ALARM_ACK_TIME )                               // ACK timer expired then clear the ACK request
   {
       almWord->AlarmWdAcked = almWord->AlarmWdAck;                             // Set the ACKED to ACK
       almWord->AlarmTm=0u;                                                      // Reset the timer
       almWord->AlarmTmRun = false;                                             // Stop the timer
    }
    else
    {
        almWord->AlarmWdAcked = almWord->AlarmWdAcked&almWord->AlarmWdAck;
    }
    almWord->AlarmWdAck = almWord->AlarmWdAck^almWord->AlarmWdAcked;            // After DOFF (COMS_ALARM_ACK_TIME) remove the GUI Ack
    almWord->AlarmWdCurrent = almWord->AlarmWdCurrent^almWord->AlarmWdAck;      // Alarm is removed when Acknowledged on GUI
    
    for(counterAlm=0u; counterAlm<=7u; counterAlm++)                              // Now process the alarm line display and stack
    {
       if ((almWord->AlarmWdCurrent & ((uint8_t)(pow(2.0f,((float64_t) counterAlm)))))!=0u)                    // if the bit is true We have an alarm
       {
          pushAlmToStack(alarmStack,almDesc,((uint8_t) pow(2.0f,((float64_t) counterAlm))));   // place the alarm on the stack
       }
    }
   
   Update_Alm_Line( alarmStack, almDesc, almLineObj );                          // update the chosen banner with the alarm message if its now the top of the stack
}

/*-----------------------------------------------------------------------------
 *      Process_Motor_State():  Check Motor States and alarm if neccessary and 
 *                              use HMI Acknowledge button to cancel it
 *
 *  Parameters: COMGS_Motor_t *mtrBlk
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Process_Motor_State( COMGS_Motor_t *mtrBlk )
{
   if ( mtrBlk->ReqState != mtrBlk->LastReq )                                   // Requested change of state has been seen
   {
      mtrBlk->Changing = true;                                                  // request change of state of the motor
      mtrBlk->timeElapsed=-1;                                                   // initialise the timer
   }
   if (( mtrBlk->ReqState != mtrBlk->State ) &&  ( mtrBlk->Changing==true ))    // state mismatch and a change of state was requested
   {
       calculateTimeDiff(&mtrBlk->timeElapsed, (uint64_t*) &mtrBlk->lasttime);                // Calculate time difference since last function call
       mtrBlk->FTS = ((((float32_t) mtrBlk->timeElapsed)/10.0f) >= mtrBlk->TimeMax);     // Alarm if time greater than requested
   }
   else if (( mtrBlk->ReqState != mtrBlk->State ) &&  ( mtrBlk->Changing==false ))    // state mismatch and no requested change (ESTOP or TRIP FAULT or other request not us)
   {
       mtrBlk->FTS = true;                                                      // Alarm without question (immediately)
   }
   else                                                                         // motor at commanded state
   {
      mtrBlk->FTS = false;                                                      // no fail to start or stop alarm
      mtrBlk->Changing = false;                                                 // no request to change
   }
   mtrBlk->LastReq = mtrBlk->ReqState;                                          // remember last known state in order to detect a state change on HMI
}

/*-----------------------------------------------------------------------------
 *      kalman_filter():  Simple Kalman function
 *
 *  Parameters: COMGS_kalman *kalmanData
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void kalman_filter( COMGS_kalman *kalmanData )
{
    float32_t x_est_last = 0f;                                                  //initial values for the kalman filter
    float32_t P_last = 0f;
    float32_t Q = 0.022f;                                                       //the noise in the system
    float32_t R = 0.617f;

    float32_t K;
    float32_t P;
    float32_t P_temp;
    float32_t x_temp_est;
    float32_t x_est;
    float32_t z_measured;
    float32_t z_real = 0.5;                                                     // default for desired value (meas and desired should be passed in struct)
    float32_t sum_error_kalman = 0f;
    float32_t sum_error_measure = 0f;
    uint8_t i;
    
    z_measured = kalmanData->z_measured;                                        // the 'noisy' value we measured
    z_real = kalmanData->z_real;                                                // the ideal value we wish to measure
    
    srand(0u);                                                                  // initialise random number generator

    //initialize with a measurement
    x_est_last = z_real + frand_func()*0.09f;                                   // last estimation plus random noise

    for (i=0u;i<30u;i++)                                                        // Repeat for 30 iterations
    {

        x_temp_est = x_est_last;                                                //do a prediction
        P_temp = P_last + Q;

        K = P_temp * (1.0f/(P_temp + R));                                        //calculate the Kalman gain

        z_measured = z_real + frand_func()*0.09f;                                // the real measurement plus noise

        x_est = x_temp_est + K * (z_measured - x_temp_est);                     // correction
        P = (1- K) * P_temp;

        //we have our new system
        kalmanData->x_est = x_est;                                              // write the kalman position
        sum_error_kalman += fabs(z_real - x_est);                               // sum of errors using kalman
        sum_error_measure += fabs(z_real-z_measured);                           // sum of errors in real value

        P_last = P;                                                             // update our last's
        x_est_last = x_est;

    }
    kalmanData->sum_error_measure = sum_error_measure;                          // Total error if using raw measured:
    kalmanData->sum_error_kalman = sum_error_kalman;                            // Total error if using kalman filter
    kalmanData->Diff_err = 100-(int16_t)((sum_error_kalman/sum_error_measure)*100.0f); // Reduction in error:

}
#define POS_TIM_STUP_DLY (CORE_TICK_SECOND*3u)
/*-----------------------------------------------------------------------------
 *     generatePose :  Generate Pose by iterating this function
 *
 * Ref : C.(Cees)Trouwborst UniversityofTwente (matlab report)
 *
 *
 *  Parameters: geoPose_t *pose, float32_t setCoord, float32_t setpointZ
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void generatePose(geoPose_t *pose, float32_t setCoord, float32_t setpointZ)
{
        int32_t timeDelay;

        switch (pose->state)
        {
          case 0u: /* initialise */
          pose->position.x = 0.0f;
          pose->position.y = 0.0f;
          pose->lastTimRef = CP0_GET(CP0_COUNT);
          pose->state=++pose->state % UINT8_MAX;
          break;

          case 1u: /* wait */
          calculateTick2Now(&timeDelay,&pose->lastTimRef );
          if (timeDelay > POS_TIM_STUP_DLY)
            pose->state = 2u;
          break;

          case 2u: /* iterate */
          pose->position.x = cos((-timeDelay*0.3f)*setCoord);
          pose->position.y = sin((-timeDelay*0.3f)*setCoord);
          pose->position.z = setpointZ;
          pose->orientation.w = 1.0f;
          break;
        }

}
#if defined(SBGC_GIMBAL_JOY)
/*-----------------------------------------------------------------------------
 *      CalibrateCenter():  Calibrate (change to get a calibration object and set it back)
 *
 *      triggered by a 3xDIN being pressed simultaneously                           ((di5VCMode==1) && (di2PosHalt==1) && (di4RCMode==1))
 *  Parameters: COMGS_joy_calib_t *calibObj
 *
 *  Return:     returns 1 if too far out, returns 0 if at center
 *----------------------------------------------------------------------------*/
uint8_t CalibrateCenter( COMGS_joy_calib_t *calibObj )
{
  uint8_t counter1=0u;                                                          // intialise counter 1
  uint64_t centerValx=0LL, centerValy=0LL, centerValz=0LL;                      // initial values set to zero (reset total for samples)

  calibObj->butState &= !0x38u;                                                 // REmove the tick boxes for center from the HMI
  
  while( counter1 != 0x0400u )                                                  // sample 1024 times quicky at start of calibration (should be at center position)
  {
      centerValx =+ ADC1_Get_Sample(1u);                                        // Get Raw ADC input 1 in units 0-1023
      centerValy =+ ADC1_Get_Sample(2u);                                        // Get Raw ADC input 1 in units 0-1023
      centerValz =+ ADC1_Get_Sample(3u);                                        // Get Raw ADC input 1 in units 0-1023
      asm nop;                                                                  // Wait for a tick
  }
  centerValx = centerValx >> 10LL;                                              // divide by 1024
  centerValy = centerValy >> 10LL;                                              // divide by 1024
  centerValz = centerValz >> 10LL;                                              // divide by 1024
  
  if (abs((RAW_MAX>>1LL)-centerValx) <= JOY_CENTER_MIN)                           // set the calibration object with the new center positions read so long as okay
  {
     calibObj->centerValx = centerValx;                                         // center position for split range if required
     calibObj->butState |= 8u;                                                   // tick box for calibration state on GUI
     if (abs((RAW_MAX>>1LL)-centerValy) <= JOY_CENTER_MIN)
     {
        calibObj->centerValy = centerValy;                                      // center position for split range if required
        calibObj->butState |= 16u;                                              // tick box for calibration state on GUI
        if (abs((RAW_MAX>>1LL)-centerValz) <= JOY_CENTER_MIN)
        {
           calibObj->centerValz = centerValz;                                   // center position for split range if required
           calibObj->butState |= 32u;                                           // tick box for calibration state on GUI
           return(JCAL_SUCCESS);                                                // return 0 (success)
        }
     }
  }
  return(JCAL_CEN_FLT);                                                         // return an error not close enough centered
}


//
//  Calibrate MAX and ( MIN to do) for each direction
//  triggered by a 3xDIN being pressed simultaneously                           ((di5VCMode==1) && (di2PosHalt==1) && (di4RCMode==1))
//  and then pressing DIN to be at direction each time
//
//  The routine uses a while to lock the joystick controller in calibrate mode during
//  the process, use USE_CAL_UNLOCK, should you want to send updates to GUI
//  or send other messages types during calibration 
//  then iterate this function within the main body (it then no longer waits for calibration)
//  it will perform this as a paralell task
//
//  define USE_CAL_UNLOCK to unlock and allow other activity as described above
//
uint8_t CalibrateSpan( COMGS_joy_calib_t *calibObj )
{
  uint64_t counter1=0LL;
  uint16_t maxValx=0u, maxValy=0u, maxValz=0u;                                  // Reset the values so they will collect new ones (when in CAL_LOCK)
  uint16_t minValx=0u, minValy=0u, minValz=0u;
  uint16_t Valx=0u, Valy=0u, Valz=0u;                                           // actual raw ADC 10 bit counter
  
  calibObj->butState &= 0x38u;                                                  // reset tick boxes except the center calibration ones already done

#ifndef USE_CAL_UNLOCK
  while (di3Reset==1u)                                                          // press button 3 to engage
  {
      Valx = ADC1_Get_Sample(1u);                                               // Get Raw ADC input 1 in units 0-1024
      Valy = ADC1_Get_Sample(2u);                                               // Get Raw ADC input 2 in units 0-1024
      Valz = ADC1_Get_Sample(3u);                                               // Get Raw ADC input 2 in units 0-1024
      if (Valx > maxValx) maxValx = Valx;                                       // take max if its higher
      if (Valx < minValx) minValx = Valx;                                       // take min if its lower
      if (abs((RAW_MAX>>1u)-Valz) <= JOY_CENTER_MIN) return(JCAL_XDIR_FLT);      // return fail if we wernt in only x direction
      if (abs((RAW_MAX>>1u)-Valy) <= JOY_CENTER_MIN) return(JCAL_XDIR_FLT);
      asm nop;                                                                  // Wait for a tick
      calibObj->butState |= 1u;                                                  // indicate the step in calibration
  }
#else
  if ((di3Reset==1u) || (calibObj->seqState == 1u))                               // press button 3 the 1st time
  {

      Valx = ADC1_Get_Sample(1u);                                                // Get Raw ADC input 1 in units 0-1024
      Valy = ADC1_Get_Sample(2u);                                                // Get Raw ADC input 2 in units 0-1024
      Valz = ADC1_Get_Sample(3u);                                                // Get Raw ADC input 2 in units 0-1024
      if (abs((RAW_MAX>>1u)-Valz) <= JOY_CENTER_MIN) return(JCAL_XDIR_FLT);      // return fail if we wernt in only x direction
      if (abs((RAW_MAX>>1u)-Valy) <= JOY_CENTER_MIN) return(JCAL_XDIR_FLT);
      if (Valx > calibObj->maxValx) calibObj->maxValx = Valx;                   // take max if its higher
      if (Valx < calibObj->minValx) calibObj->minValx = Valx;                   // take min if its lower
      calibObj->butState |= 1u;                                                  // indicate the step in calibration
      calibObj->seqState = 1u;                                                   // indicate the step in calibration
      if ((calibObj->seqState == 1u) && (di3Reset==0u))                           // ready to change sequence step
      {
        calibObj->seqState = 2u;
      }
  }
#endif

#ifndef USE_CAL_UNLOCK
  while ((di3Reset==0u) && (counter1 <= 80000000LL))                            // wait for button or timeout  ( 1sec at 80 MHz)
  {
     counter1=++counter1 % UINT64_MAX;
     asm nop;
  }
#else
  if ((di3Reset==0u) || (calibObj->seqState == 2u))                               // wait for button or timeout  ( 1sec at 80 MHz)
  {
     if (calibObj->counter1 <= 0u)                                               // first time in sequence
     {
        calculateTimeDiff(&counter1, &calibObj->lasttime);                        // store the time from global counter in obj->lasttime
        calibObj->counter1=1u;                                                   // next step for initialise (wait for next press)
     }
     else
     {
        calculateTimeDiff(&counter1, &calibObj->lasttime);                        // calculate time difference from the interrrupt
        if (counter1 > (200LL))                                                   // 100 ms counts have exceeded the timeout
        {
            calibObj->counter1=0u;
            return(JCAL_OOR_FLT);                                               // timeout error alarm
        }
        if ((calibObj->seqState == 2u) && (di3Reset==1u))                         // ready to change sequence step
        {
           calibObj->seqState = 3u;
        }
     }
  }
#endif

#ifndef USE_CAL_UNLOCK
  while (di3Reset==1u)                                                           // press button 3 to engage
  {
      counter1=0LL;
      Valx = ADC1_Get_Sample(1u);                                               // Get Raw ADC input 1 in units 0-1024
      Valy = ADC1_Get_Sample(2u);                                               // Get Raw ADC input 2 in units 0-1024
      Valz = ADC1_Get_Sample(3u);                                               // Get Raw ADC input 2 in units 0-1024
      if (Valy > maxValy) maxValy = Valy;                                       // take max if its higher
      if (Valy < minValy) minValy = Valy;                                       // take min if its lower
      if (abs((RAW_MAX>>1u)-Valz) <= JOY_CENTER_MIN) return(JCAL_YDIR_FLT);      // return fail if we wernt in only y direction
      if (abs((RAW_MAX>>1u)-Valx) <= JOY_CENTER_MIN) return(JCAL_YDIR_FLT);
      asm nop;                                                                  // Wait for a tick
      calibObj->butState |= 2u;                                                  // indicate the step in calibration
  }
#else
  if ((di3Reset==1u) || (calibObj->seqState == 3u))                               // button 3 pressed and in step 3 of calibration
  {
      Valx = ADC1_Get_Sample(1u);                                                // Get Raw ADC input 1 in units 0-1024
      Valy = ADC1_Get_Sample(2u);                                                // Get Raw ADC input 2 in units 0-1024
      Valz = ADC1_Get_Sample(3u);                                                // Get Raw ADC input 2 in units 0-1024
      if (abs((RAW_MAX>>1u)-Valz) <= JOY_CENTER_MIN) return(JCAL_YDIR_FLT);      // return fail if we wernt in only x direction
      if (abs((RAW_MAX>>1u)-Valx) <= JOY_CENTER_MIN) return(JCAL_YDIR_FLT);
      if (Valy > calibObj->maxValy) calibObj->maxValy = Valy;                   // take max if its higher
      if (Valy < calibObj->minValy) calibObj->minValy = Valy;                   // take min if its lower
      calibObj->butState |= 2u;                                                  // indicate the step in calibration
      if ((calibObj->seqState == 3u) && (di3Reset==0u))                           // ready to change sequence step
      {
        calibObj->seqState = 4u;                                                 // advance to next step in calibration
      }
  }
#endif

#ifndef USE_CAL_UNLOCK
  while ((di3Reset==0u) && (counter1 <= 80000000LL))                            // wait for button or timeout
  {
     counter1=++counter1 % UINT64_MAX;
     asm nop;
  }
#else
  if ((di3Reset==0u) || (calibObj->seqState == 4u))                               // wait for button or timeout  ( 1sec at 80 MHz)
  {
     if (calibObj->counter1 == 1u)                                               // second time to check the global 100ms timer
     {
        calculateTimeDiff(&counter1, &calibObj->lasttime);                        // store the time from global counter in obj->lasttime
        calibObj->counter1=0u;
     }
     else
     {
        calculateTimeDiff(&counter1, &calibObj->lasttime);                        // calculate time difference from the interrrupt
        if (counter1 > (200LL))                                                   // 100 ms counts have exceeded the timeout
        {
            calibObj->counter1=0u;
            return(JCAL_TIME_FLT);                                              // timeout error alarm
        }
        if ((calibObj->seqState == 4u) && (di3Reset==1u))                         // ready to change sequence step
        {
           calibObj->seqState = 5u;                                              // advance to step 5 of the calibration
        }
     }
  }
#endif

#ifndef USE_CAL_UNLOCK
  while (di3Reset==1u)                                                           // press button 3 to engage
  {
      counter1=0LL;
      Valx = ADC1_Get_Sample(1u);                                                // Get Raw ADC input 1 in units 0-1024
      Valy = ADC1_Get_Sample(2u);                                                // Get Raw ADC input 2 in units 0-1024
      Valz = ADC1_Get_Sample(3u);                                                // Get Raw ADC input 2 in units 0-1024
      if (Valz > maxValz) maxValz = Valz;                                       // take max if its higher
      if (Valz < minValz) minValz = Valz;                                       // take min if its lower
      if (abs((RAW_MAX>>1u)-Valy) <= JOY_CENTER_MIN) return(JCAL_ZDIR_FLT);      // return fail if we wernt in only z direction
      if (abs((RAW_MAX>>1u)-Valx) <= JOY_CENTER_MIN) return(JCAL_ZDIR_FLT);
      asm nop;                                                                  // Wait for a tick
      calibObj->butState |= 4u;                                                  // indicate the step in calibration
  }
  
  if (abs(RAW_MAX - maxValx) > 400u) return(JCAL_OOR_FLT);                       // out of expected range
  if (abs(RAW_MAX - maxValy) > 400u) return(JCAL_OOR_FLT);                       // out of expected range
  if (abs(RAW_MAX - maxValz) > 400u) return(JCAL_OOR_FLT);                       // out of expected range
  if (abs(RAW_MAX - minValx) > 400u) return(JCAL_OOR_FLT);                       // out of expected range
  if (abs(RAW_MAX - minValy) > 400u) return(JCAL_OOR_FLT);                       // out of expected range
  if (abs(RAW_MAX - minValz) > 400u) return(JCAL_OOR_FLT);                       // out of expected range

  if ((calibObj->butState&7u) && (di3Reset==0u) )                                 // saw all steps in the calibration sequence and finished
  {
      calibObj->maxValx = maxValx;                                              // set the values to be used in calibration to the cal object
      calibObj->maxValy = maxValy;
      calibObj->maxValz = maxValz;
      calibObj->minValx = minValx;
      calibObj->minValy = minValy;
      calibObj->minValz = minValz;
      return(JCAL_SUCCESS);                                                     // success
  }
  else
  {
      return(JCAL_STEP_FLT);                                                    // incomplete steps
  }
#else
  if ((di3Reset==1u) || (calibObj->seqState == 5u))                               // button pressed and in step 5 of calibration
  {
      Valx = ADC1_Get_Sample(1u);                                                // Get Raw ADC input 1 in units 0-1024
      Valy = ADC1_Get_Sample(2u);                                                // Get Raw ADC input 2 in units 0-1024
      Valz = ADC1_Get_Sample(3u);                                                // Get Raw ADC input 2 in units 0-1024
      if (abs((RAW_MAX>>1u)-Valy) <= JOY_CENTER_MIN) return(JCAL_ZDIR_FLT);      // return fail if we wernt in only x direction
      if (abs((RAW_MAX>>1u)-Valx) <= JOY_CENTER_MIN) return(JCAL_ZDIR_FLT);
      if (Valz > calibObj->maxValz) calibObj->maxValz = Valz;                   // take max if its higher
      if (Valz < calibObj->minValz) calibObj->minValz = Valz;                   // take min if its lower
      calibObj->butState |= 4u;                                                  // indicate the step in calibration
      if ((calibObj->seqState == 5u) && (di3Reset==0u))                           // ready to change sequence step
      {
        calibObj->seqState = 6u;                                                 // advance to next step in calibration
        if (calibObj->butState&7u)                                               // saw all steps in the calibration sequence
        {
           return(JCAL_SUCCESS);                                                // success
        }
        else
        {
           return(JCAL_STEP_FLT);                                               // incomplete steps
        }
      }
  }
#endif

}
#endif
/* --- some of these functions were ported from OPTICAL FLOW CLOVER
 *
 * Detecting and pose estimation of ArUco markers maps
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * Code is based on https://github.com/UbiquityRobotics/fiducials, which is distributed
 * under the BSD license.
 */
  
/*-----------------------------------------------------------------------------
 *      alignObjPointsToCenter():  Align object points to the center of mass
 *
 *  Parameters: Vectr *obj_points[], float32_t *center_x, float32_t *center_y, float32_t *center_z
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void alignObjPointsToCenter(Vectr *obj_points[], float32_t *center_x, float32_t *center_y, float32_t *center_z)
{

                float32_t sum_x = 0.0f;
                float32_t sum_y = 0.0f;
                float32_t sum_z = 0.0f;
                int16_t i;
                int8_t numOfEle = (sizeof(*obj_points)/sizeof(Vectr));

                for (i = 0; i < numOfEle; i++)
                {
                        sum_x += obj_points[i]->x;
                        sum_y += obj_points[i]->y;
                        sum_z += obj_points[i]->z;
                }

                *center_x = sum_x / numOfEle;
                *center_y = sum_y / numOfEle;
                *center_z = sum_z / numOfEle;

                for (i = 0; i < numOfEle; i++)
                {
                        obj_points[i]->x -= *center_x;
                        obj_points[i]->y -= *center_y;
                        obj_points[i]->z -= *center_z;
                }
}
/*-----------------------------------------------------------------------------
 *      fillPose():  fill pose
 *
 *  Parameters: geoPose_t *pose, const Vectr *rvec, const Vectr *tvec
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void fillPose(geoPose_t *pose, const Vectr *rvec, const Vectr *tvec)
{
   float32_t angle = vnorm1(rvec);
   Vectr axis; 
   axis = vdiv(*rvec, angle);
   pose->position.x = tvec->x;
   pose->position.y = tvec->y;
   pose->position.z = tvec->z;

   pose->orientation = mkquat(axis.x, axis.y, axis.z, angle);

}
/*-----------------------------------------------------------------------------
 *      qinv():  invert a quaternion
 *
 *  Parameters: quat q
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat qinv(quat q) 
{
  return mkquat(-q.x, -q.y, -q.z, q.w);
} 
/*-----------------------------------------------------------------------------
 *      qsub():  subtract two quaternion.
 *
 *  Parameters: quat q
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat qsub(quat q, quat r) 
{
  return mkquat((q.x-r.x), (q.y-r.y), (q.z-r.z), (q.w-r.w));
}
/*-----------------------------------------------------------------------------
 *      qmul():   multiply two quaternion. if not correct try qqmul
 *
 *  Parameters: quat q, quat r
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/ 
quat qmul(quat q, quat r) 
{
  return mkquat((q.x*r.x), (q.y*r.y), (q.z*r.z), (q.w*r.w));
}  
/*-----------------------------------------------------------------------------
 *      qscl():   multiply quaternion by scalar
 *
 *  Parameters: quat q, float32_t r
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat qscl(quat q, float32_t r) 
{
  return mkquat((q.x*r), (q.y*r), (q.z*r), (q.w*r));
} 
/*-----------------------------------------------------------------------------
 *      qtrans():   transform quaternion  TODO : (double check this maths from memory) 
 *
 *  Parameters: quat q
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat qtrans(quat q) 
{
  return mkquat( q.w, q.x, q.y, q.z );
} 
/*-----------------------------------------------------------------------------
 *      vtrans():   transform quaternion  TODO : (double check this maths from memory) 
 *
 *  Parameters: Vectr q
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vtrans(Vectr q) 
{
  return mkvec( q.z, q.x, q.y );
} 
// https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Quaternion_?_angular_velocities
/*-----------------------------------------------------------------------------
 *      calcFlowGyro(): calculate optical flow from a given current and previous gyro vector 
 *
 *  Parameters: const uint8_t frame_id, optical_flowObj_t *flow
 *
 *  Return:     optical_flowObj_t *
 *----------------------------------------------------------------------------*/
optical_flowObj_t * calcFlowGyro(const uint8_t frame_id, optical_flowObj_t *flow)
{
        quat diff;

        flow->frame_id = frame_id;
        flow->stamp = CP0_GET(CP0_COUNT);

        diff = qscl(qmul(qsub(flow->curr_rot, flow->prev_rot) ,qinv(flow->prev_rot)) ,2.0f);
        flow->vector = mkvec(-diff.x,-diff.y,-diff.z);
        return flow;
}
/*-----------------------------------------------------------------------------
 *      doOpticalFlo(): Do optical flow (TO DO Port openCV comands)
 *
 *  Parameters: optical_flowObj_t *flow, float32_t roi_rad_, mat33 camera_matrix_, 
 *              mat33 dist_coeffs
 *
 *  Return:     optical_flowObj_t *
 *----------------------------------------------------------------------------*/
optical_flowObj_t * doOpticalFlo( optical_flowObj_t *flow, float32_t roi_rad_, mat33 camera_matrix_, mat33 dist_coeffs)
{
        uint32_t flow_center_x, flow_center_y, integration_time;
        Vectr img_points[2u];                                                   /* square box for ROI */
        Vectr v3 = {0, 0, 0};
        Vectr shift = {0, 0, 0};
        Vectr points_dist, points_undist[2u];
        optical_flowObj_t shift_vec, flow_camera, flow_fcu, *flow_gyro_camera, flow_gyro_fcu;
        quat roi_;
        float64_t focal_length_x, focal_length_y, flow_x, flow_y, response;
        Vectr v1,v2;                                 
        mat33 object_points;  
                   
        if (flow->roi_px_ > 0.0f)
        {
           /* calculate ROI */
           v1 = mkvec(((float32_t)-sin(roi_rad_ / 2.0f)), ((float32_t) -sin(roi_rad_ / 2.0f)), ((float32_t) cos(roi_rad_ / 2.0f))); /* construct region of interest in picture from radius */
           v2 = mkvec(((float32_t)sin(roi_rad_ / 2.0f)), ((float32_t)sin(roi_rad_ / 2.0f)), ((float32_t) cos(roi_rad_ / 2.0f)));    
           object_points = mcolumns(v1, v2, v3);
        /* done in openCV we need to port this function */
        /* cv::projectPoints(object_points, vec, vec, camera_matrix_, dist_coeffs_, img_points); need to replicate PORT OpenCV commands */
           img_points[0u].x = object_points.m[0u][0u] * camera_matrix_.m[0u][0u] * dist_coeffs.m[0u][0u]; 
           img_points[0u].y = object_points.m[0u][1u] * camera_matrix_.m[0u][1u] * dist_coeffs.m[0u][1u]; 
           img_points[0u].z = object_points.m[0u][2u] * camera_matrix_.m[0u][2u] * dist_coeffs.m[0u][2u]; 
           img_points[1u].x = object_points.m[1u][0u] * camera_matrix_.m[1u][0u] * dist_coeffs.m[1u][0u]; 
           img_points[2u].y = object_points.m[1u][1u] * camera_matrix_.m[1u][1u] * dist_coeffs.m[1u][1u]; 
           img_points[3u].z = object_points.m[1u][2u] * camera_matrix_.m[1u][2u] * dist_coeffs.m[1u][2u]; 
           roi_ = mkquat(img_points[0u].x, img_points[0u].y, img_points[1u].x, img_points[1u].y );
        }
        else
        {
           roi_ = mkquat((flow->width / 2.0f - flow->roi_px_ / 2.0f), (flow->height / 2.0f - flow->roi_px_ / 2.0f), flow->roi_px_, flow->roi_px_);
        }
        /** openCV                         img = img(roi_);                  img.convertTo(curr_, CV_32F); format convert */
        
        
        if ((((flow->prev_rot.x == 0.0f) && (flow->prev_rot.y == 0.0f)) && (flow->prev_rot.z == 0.0f)) && (flow->prev_rot.w == 0.0f))
        {
           flow->prev_rot = mkquat( flow->curr_rot.x, flow->curr_rot.y, flow->curr_rot.z, flow->curr_rot.w );
           flow->prev_stamp = flow->stamp;
           /* flow->stamp = CP0_GET(CP0_COUNT);   let the stamp happen when we read the data ?? */
          /*                         cv::createHanningWindow(hann_, curr_.size(), CV_32F); */
        }
        else
        {
         /*                        cv::Point2d shift = cv::phaseCorrelate(prev_, curr_, hann_, &response); */
           shift_vec.vector.x = shift.x;
           shift_vec.vector.y = shift.y;  
           shift_vec.stamp = flow->stamp;
           shift_vec.frame_id  = flow->frame_id;
           /* publish to ros shift vector */
           
           flow_center_x = (uint32_t)(ROUND(flow->width / 2.0f, 0.0f));         /* Undistort flow in pixels */
           flow_center_y = (uint32_t)(ROUND(flow->height / 2.0f, 0.0f));
           shift.x += flow_center_x;
           shift.y += flow_center_y; 
           
           /*         cv::undistortPoints(points_dist, points_undist, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_); */
           points_undist[0u].x -= flow_center_x;
           points_undist[0u].y -= flow_center_y;  
           
           focal_length_x = camera_matrix_.m[0u][0u];                           /* Calculate flow in radians */
           focal_length_y = camera_matrix_.m[1u][1u];
           flow_x = atan2(points_undist[0u].x, focal_length_x);
           flow_y = atan2(points_undist[0u].y, focal_length_y);
           
           flow_camera.frame_id = flow->frame_id;                               /* Convert to FCU frame */
           flow_camera.stamp = flow->stamp;
           flow_camera.vector.x = flow_y;                                       /* +y means counter-clockwise rotation around Y axis */
           flow_camera.vector.y = -flow_x;                                      /* +x means clockwise rotation around X axis         */
           /* TODO transform
                                           tf_buffer_->transform(flow_camera, flow_fcu, fcu_frame_id_); */
           flow_fcu.vector = vtrans(flow_camera.vector);
           
           calculateTime2Tick( &integration_time, &flow->prev_stamp, &flow->stamp );                          /* Calculate integration time */
           flow->integration_time_us = (uint32_t)(ROUND(SEC2US(integration_time / CPU_TICKS_PER_SECOND),0.0f));
           
           flow_gyro_camera = calcFlowGyro(flow->frame_id, flow);
           /* TODO this transform                                         static geometry_msgs::Vector3Stamped flow_gyro_fcu;
           tf_buffer_->transform(flow_gyro_camera, flow_gyro_fcu, fcu_frame_id_); */
           flow_gyro_fcu.vector = vtrans(flow_gyro_camera->vector);
                      
           flow->integrated_xgyro = flow_gyro_fcu.vector.x;
           flow->integrated_ygyro = flow_gyro_fcu.vector.y;
           flow->integrated_zgyro = flow_gyro_fcu.vector.z;
           flow->integrated_x = flow_fcu.vector.x;
           flow->integrated_y = flow_fcu.vector.y;
           flow->quality = ((uint8_t) ROUND((response*255u),0.0f));

           flow->twist.angular.x = (flow->integrated_x * CPU_TICKS_PER_SECOND) / integration_time;    /* Publish estimated angular velocity */
           flow->twist.angular.y = (flow->integrated_y * CPU_TICKS_PER_SECOND) / integration_time; 

           memcpy((void*)&flow->prev_rot,(void*)&flow->curr_rot,sizeof(quat));  /* copy current to previous */
           flow->prev_stamp = flow->stamp;           
        }
        return flow;
}
/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™
 *
 *  File: kinova_ros_types.cpp
 *  Desc: Wrappers around Kinova structs to facilitate easier conversion to ROS
 *                  types.
 *  Auth: Alex Bencz
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com
 *
 * These helpers were ported to mikroE C by ACP Aviation
 */
 /**
 * @brief EulerXYZ2Matrix3x3
 * Build rotation matrix using Euler-XYZ angles from KinovaPose
 * @param tx input Euler angle tx
 * @param ty input Euler angle ty
 * @param tz input Euler angle tz
 * @return output rotation matrix
 * @warning DSP(KinovaPose) use Euler-XYZ convention, while ROS use Euler-ZYX by default.
 */
mat33 EulerXYZ2Matrix3x3(float32_t tx, float32_t ty, float32_t tz)
{
    mat33 Rot_m;
    float32_t sx = sin(tx);
    float32_t cx = cos(tx);
    float32_t sy = sin(ty);
    float32_t cy = cos(ty);
    float32_t sz = sin(tz);
    float32_t cz = cos(tz);

    Rot_m.m[0u][0u] = cz*cy;
    Rot_m.m[0u][1u] = -sz*cy;
    Rot_m.m[0u][2u] = sy;
    Rot_m.m[1u][0u] = cz*sy*sx+sz*cx;
    Rot_m.m[1u][1u] = -sz*sy*sx+cz*cx;
    Rot_m.m[1u][2u] = -cy*sx;
    Rot_m.m[2u][0u] = -cz*sy*cx+sz*sx;
    Rot_m.m[2u][1u] = sz*sy*cx+cz*sx;
    Rot_m.m[2u][2u] = cy*cx;

    return Rot_m;
}
/**
 * @brief getEulerXYZ
 * getEulerXYZ get Euler-XYZ convention for KinovaPose orientation from Rotation matrix convention.
 * @param Rot_matrix input rotation matrix
 * @param tx output Euler angle tx
 * @param ty output Euler angle ty
 * @param tz output Euler angle tz
 * @warning DSP(KinovaPose) use Euler-XYZ convention, while ROS use Euler-ZYX by default.
 */
void getEulerXYZ(mat33 *Rot_matrix, float32_t *tx, float32_t *ty, float32_t *tz)
{
    float32_t a11;
    float32_t a12;
    float32_t a13;
    float32_t a23;
    float32_t a33;

    a11 = Rot_matrix->m[0u][0u];
    a12 = Rot_matrix->m[0u][1u];
    a13 = Rot_matrix->m[0u][2u];
    a23 = Rot_matrix->m[1u][2u];
    a33 = Rot_matrix->m[2u][2u];
    
    *tx = atan2(-a23, a33);
    *ty = atan2(a13, sqrt(1.0f-a13*a13));
    *tz = atan2(-a12, a11);
}
/**
 * @brief EulerXYZ2Quaternion
 * @param tx input Euler angle tx
 * @param ty input Euler angle ty
 * @param tz input Euler angle tz
 * @return output Quaternion
 * @warning DSP(KinovaPose) use Euler-XYZ convention, while ROS use Euler-ZYX by default.
 */
quat EulerXYZ2Quaternion(float32_t tx, float32_t ty, float32_t tz)
{
    float32_t sx = sin(0.5f*tx);
    float32_t cx = cos(0.5f*tx);
    float32_t sy = sin(0.5f*ty);
    float32_t cy = cos(0.5f*ty);
    float32_t sz = sin(0.5f*tz);
    float32_t cz = cos(0.5f*tz);
    quat q;
    
    q.x =  sx*cy*cz + cx*sy*sz;
    q.y = -sx*cy*sz + cx*sy*cz;
    q.z =  sx*sy*cz + cx*cy*sz;
    q.w = -sx*sy*sz + cx*cy*cz;

    return q;
}
/**
 * @brief PickPlace::generate_gripper_align_pose
 * @param targetpose_msg pick/place pose (object location): where gripper close/open the fingers (grasp/release the object). Only position information is used.
 * @param dist distance of returned pose to targetpose
 * @param azimuth an angle measured from the x-axis in the xy-plane in spherical coordinates, denoted theta (0<= theta < 2pi ).
 * @param polar also named zenith, colatitude, denoted phi (0<=phi<=pi). It is the angle from the positive z-axis to the vector.  phi= pi/2 - delta where delta is the latitude.
 * @param rot_gripper_z rotation along the z axis of the gripper reference frame (last joint rotation)
 * @return a pose defined in a spherical coordinates where origin is located at the target pose. Normally it is a pre_grasp/post_realease pose, where gripper axis (last joint axis) is pointing to the object (target_pose).
 */
geoPose_t generate_gripper_align_pose(geoPose_t *targetpose_msg, float32_t dist, float32_t  azimuth, float32_t  polar, float32_t rot_gripper_z)
{
    geoPose_t pose_msg;
    float64_t delta_x;
    float64_t delta_y;
    float64_t delta_z;
    quat q;
    /* pose_msg.header.frame_id = "root";  */

    // computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    delta_x = -dist * cos(azimuth) * sin(polar);
    delta_y = -dist * sin(azimuth) * sin(polar);
    delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
    q = EulerXYZ2Quaternion(azimuth, polar, rot_gripper_z);

    pose_msg.position.x = targetpose_msg->position.x + delta_x;
    pose_msg.position.y = targetpose_msg->position.y + delta_y;
    pose_msg.position.z = targetpose_msg->position.z + delta_z;
    pose_msg.orientation.x = q.x;
    pose_msg.orientation.y = q.y;
    pose_msg.orientation.z = q.z;
    pose_msg.orientation.w = q.w;

    return pose_msg;

}
/**
 * @brief getQuatXYZ get Euler-XYZ convention for KinovaPose orientation from Quaternion convention.
 * @param q input Quaternion
 * @param tx output Euler angle tx
 * @param ty output Euler angle ty
 * @param tz output Euler angle tz
 * @warning DSP(KinovaPose) use Euler-XYZ convention, while ROS use Euler-ZYX by default.
 */
void getQuatXYZ(quat *q, float32_t *tx, float32_t *ty, float32_t *tz)
{
    float32_t qx = q->x;
    float32_t qy = q->y;
    float32_t qz = q->z;
    float32_t qw = q->w;

    *tx = atan2((2.0f*qw*qx-2.0f*qy*qz),(qw*qw-qx*qx-qy*qy+qz*qz));
    *ty = asin(2.0f*qw*qy+2*qx*qz);
    *tz = atan2((2.0f*qw*qz-2.0f*qx*qy),(qw*qw+qx*qx-qy*qy-qz*qz));
}
/*-----------------------------------------------------------------------------
 *      areValuesClose():  Checks 2 real values against a tolerance
 *
 *  Parameters: float32_t first, float32_t second, float32_t tolerance
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t areValuesClose(float32_t first, float32_t second, float32_t tolerance)
{
    return ((first <= second + tolerance) && (first >= second - tolerance));
}
/*-----------------------------------------------------------------------------
 *      valid_kinovaRobotType():  Checks robot type definition
 *
 *  Parameters: const unsigned char *kinova_RobotType
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t valid_kinovaRobotType(const unsigned char *kinova_RobotType)
{
    if(strlen((char*)kinova_RobotType)!=8)
    {
        return false;                                                           /* incorect format */
    }

    // kinova_RobotType = [robot_categary, robot_version, wrist, dof, model, fingerNum, res7, res8];

    if (kinova_RobotType[0]=='j')                                               // check categary
    {
        if ((kinova_RobotType[1]!='1') && (kinova_RobotType[1]!='2'))           // check version
        {
            return false;                                                       /* version error */
        }

        if ((kinova_RobotType[2]!='s') && (kinova_RobotType[2]!='n'))           // check wrist
        {
            return false;                                                       /* wrist error */
        }

        if ((kinova_RobotType[3]!='4') && (kinova_RobotType[3]!='6') && (kinova_RobotType[3]!='7'))         // check dof
        {
            return false;                                                       /* Number of expected joints for standard robot configurations is 4, 6, 7 */
        }

        if ((kinova_RobotType[4]!='s') && (kinova_RobotType[4]!='a'))           // check model
        {
            return false;                                                       /* model type error.*/
        }

        // check number of fingers
        if ((kinova_RobotType[5]!='2') && (kinova_RobotType[5]!='3'))           /* finger number error */
        {
            return false;
        }
    }
    else if (kinova_RobotType[0]=='m')
    {
        if ((kinova_RobotType[1]!='1')) return false;                                        // check version
        if ((kinova_RobotType[2]!='s') && (kinova_RobotType[2]!='n')) return false;          // check wrist
        if ((kinova_RobotType[3]!='4') && (kinova_RobotType[3]!='6')) return false;          // check dof
        if ((kinova_RobotType[4]!='s') && (kinova_RobotType[4]!='a')) return false;          // check model
        if ((kinova_RobotType[5]!='2') && (kinova_RobotType[5]!='3')) return false;          // check number of fingers
    }
    else if (kinova_RobotType[0]=='r')
    {
        // roco property check
    }
    else if (kinova_RobotType[0]=='c')
    {
        // customized robot property check
    }
    else
    {
        return false;
    }

    return true;

}
/**
 * @brief construct geometric::Wrench message from KinovaPose data structure
 *
 * KinovaPose[X,Y,Z,ThetaX,ThetaY,ThetaZ] doesn't store pose information in this case. Instead, it stores force and torque correspondingly.
 *
 * @return geometry_msgs [Fx,Fy,Fz,Tx,Ty,Tz] in Newton and Nm.
 */
wrench_t KinovaPose_constructWrenchMsg( geoPose_t *pose )
{
    wrench_t wrench;

    wrench.force.x  = pose->position.x;
    wrench.force.y  = pose->position.y;
    wrench.force.z  = pose->position.z;
    wrench.torque.x = pose->theta.x;
    wrench.torque.y = pose->theta.y;
    wrench.torque.z = pose->theta.z;
    return wrench;
}
/*-----------------------------------------------------------------------------
 *      KinovaPose_getQuaternion():  get quat and convert to euler angles
 *
 *  Parameters: float32_t ThetaX, float32_t ThetaY, float32_t ThetaZ
 *
 *  Return: quat
 *----------------------------------------------------------------------------*/
quat KinovaPose_getQuaternion(float32_t ThetaX, float32_t ThetaY, float32_t ThetaZ)
{
    return(EulerXYZ2Quaternion(ThetaX, ThetaY, ThetaZ));
}
/*-----------------------------------------------------------------------------
 *      KinovaPose_setPoseQuaternion():  set kinova pose quat
 *
 *  Parameters: geoPose_t *pose
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void KinovaPose_setPoseQuaternion(geoPose_t *pose)
{
    pose->orientation = EulerXYZ2Quaternion(pose->theta.x, pose->theta.x, pose->theta.x);
}
/*-----------------------------------------------------------------------------
 *      KinovaPose_constructKinovaPoseMsg():  construct kinova pose
 *
 *  Parameters: float32_t X, float32_t Y, float32_t Z, float32_t ThetaX, float32_t ThetaY, float32_t ThetaZ
 *
 *  Return: geoPose_t
 *----------------------------------------------------------------------------*/
geoPose_t KinovaPose_constructKinovaPoseMsg( float32_t X, float32_t Y, float32_t Z, float32_t ThetaX, float32_t ThetaY, float32_t ThetaZ )
{
    geoPose_t pose;

    pose.position.x = X;
    pose.position.y = Y;
    pose.position.z = Z;
    pose.theta.x = ThetaX;
    pose.theta.y = ThetaY;
    pose.theta.z = ThetaZ;
    return pose;
}
/**
 * @brief check all the position and orientation values, to determine if current KinovaPose reach "other"
 *
 * @param other the pose to be compared with, normally the motion goal. In meters and radians.
 * @param position_tolerance  threshold to be considered as identical between two position values in the same axis.
 * @param EulerAngle_tolerance threshold to be considered as identical between two EulerAngle values along the same axis.
 * @return true if both position and orientation can be considerred identical.
 */
uint8_t KinovaPose_isCloseToOther(const geoPose_t *other, float32_t position_tolerance, float32_t EulerAngle_tolerance,const Vectr *posn, const Vectr *theta)
{
    uint8_t status = true;
    float32_t f1,f2;
    status = status && areValuesClose(posn->x, other->position.x, position_tolerance);
    status = status && areValuesClose(posn->y, other->position.y, position_tolerance);
    status = status && areValuesClose(posn->z, other->position.z, position_tolerance);
    f1 = FMOD2(theta->x, 2.0f*PI);
    f2 = FMOD2(other->theta.x, 2.0f*PI);
    status = status && areValuesClose(f1, f2, EulerAngle_tolerance);
    f1 = FMOD2(theta->x, 2.0f*PI) + 2.0f*PI;
    f2 = FMOD2(other->theta.x, 2.0f*PI);
    status = status && areValuesClose(f1, f2, EulerAngle_tolerance);
    f1 = FMOD2(theta->x, 2.0f*PI);
    f2 = FMOD2(other->theta.x, 2.0f*PI) + 2.0f*PI;
    status = status && areValuesClose(f1, f2, EulerAngle_tolerance);
    f1 = FMOD2(theta->y, 2.0f*PI);
    f2 = FMOD2(other->theta.y, 2.0f*PI);
    status = status && areValuesClose(f1, f2, EulerAngle_tolerance);
    f1 = FMOD2(theta->y, 2.0f*PI) + 2.0f*PI;
    f2 = FMOD2(other->theta.y, 2.0f*PI);
    status = status && areValuesClose(f1, f2, EulerAngle_tolerance);
    f1 = FMOD2(other->theta.y, 2.0f*PI);
    f2 = FMOD2(other->theta.y, 2.0f*PI) + 2.0f*PI;
    status = status && areValuesClose(f1, f2, EulerAngle_tolerance);
    f1 = FMOD2(theta->z, 2.0f*PI);
    f2 = FMOD2(other->theta.z, 2.0f*PI);
    status = status && areValuesClose(f1, f2, EulerAngle_tolerance);
    f1 = FMOD2(theta->z, 2.0f*PI) + 2.0f*PI;
    f2 = FMOD2(other->theta.z, 2.0f*PI);
    status = status && areValuesClose(f1, f2, EulerAngle_tolerance);
    f1 = FMOD2(theta->z, 2.0f*PI);
    f2 = FMOD2(other->theta.z, 2.0f*PI) + 2.0f*PI;
    status = status && areValuesClose(f1, f2, EulerAngle_tolerance);
    return status;
}
/*-----------------------------------------------------------------------------
 *      FingerAngles_isCloseToOther():  check finger angles
 *
 *  Parameters: const Vectr *other, float32_t const tolerance, const Vectr *Finger
 *
 *  Return: uint8_t = 1 success
 *----------------------------------------------------------------------------*/
uint8_t FingerAngles_isCloseToOther(const Vectr *other, float32_t const tolerance, const Vectr *Finger)
{
    uint8_t status = true;
    status = status && areValuesClose(Finger->x, other->x, tolerance);
    status = status && areValuesClose(Finger->y, other->y, tolerance);
    status = status && areValuesClose(Finger->z, other->z, tolerance);
    return status;
}
/**
 * @brief check all the joint values, to determine if current KinovaAngles reach "other"
 * @param other joint angles to be compared with, in degrees.
 * @param tolerance threshold to be considered as identical between two joints.
 * @return true if all joint values can be considerred identical.
 */
uint8_t KinovaAngles_isCloseToOther(const kinovaAngles_t *angle,  float32_t tolerance,const Vectr *posn, const kinovaAngles_t *actuator)
{
    uint8_t status = true;
    float32_t f1,f2;
    f1 = FMOD2(actuator->Actuator1, 360.0f);
    f2 = FMOD2(angle->Actuator1, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator1, 360.0f) + 360.0f;
    f2 = FMOD2(angle->Actuator1, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator1, 360.0f);
    f2 = FMOD2(angle->Actuator1, 360.0f) + 360.0f;
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator2, 360.0f);
    f2 = FMOD2(angle->Actuator2, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator2, 360.0f) + 360.0f;
    f2 = FMOD2(angle->Actuator2, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator2, 360.0f);
    f2 = FMOD2(angle->Actuator2, 360.0f) + 360.0f;
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator3, 360.0f);
    f2 = FMOD2(angle->Actuator3, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator3, 360.0f) + 360.0f;
    f2 = FMOD2(angle->Actuator3, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator3, 360.0f);
    f2 = FMOD2(angle->Actuator3, 360.0f) + 360.0f;
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator4, 360.0f);
    f2 = FMOD2(angle->Actuator4, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator4, 360.0f) + 360.0f;
    f2 = FMOD2(angle->Actuator4, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator4, 360.0f);
    f2 = FMOD2(angle->Actuator4, 360.0f) + 360.0f;
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator5, 360.0f);
    f2 = FMOD2(angle->Actuator5, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator5, 360.0f) + 360.0f;
    f2 = FMOD2(angle->Actuator5, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator5, 360.0f);
    f2 = FMOD2(angle->Actuator5, 360.0f) + 360.0f;
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator6, 360.0f);
    f2 = FMOD2(angle->Actuator6, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator6, 360.0f) + 360.0f;
    f2 = FMOD2(angle->Actuator6, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator6, 360.0f);
    f2 = FMOD2(angle->Actuator6, 360.0f) + 360.0f;
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator7, 360.0f);
    f2 = FMOD2(angle->Actuator7, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator7, 360.0f) + 360.0f;
    f2 = FMOD2(angle->Actuator7, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator7, 360.0f);
    f2 = FMOD2(angle->Actuator7, 360.0f) + 360.0f;
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator8, 360.0f);
    f2 = FMOD2(angle->Actuator8, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator8, 360.0f) + 360.0f;
    f2 = FMOD2(angle->Actuator8, 360.0f);
    status = status && areValuesClose(f1, f2, tolerance);
    f1 = FMOD2(actuator->Actuator8, 360.0f);
    f2 = FMOD2(angle->Actuator8, 360.0f) + 360.0f;
    status = status && areValuesClose(f1, f2, tolerance);
    return status;
}
#ifdef PID_LOOPS                                                                // PID Loop library included
/*-----------------------------------------------------------------------------
 *      computePID():  Compute PID algorythm
 *
 *  Parameters: pidBlock_t *pidObj
 *
 *  Return: Returns 1 on success 0 on error (no integral time found or manual mode set)
 *----------------------------------------------------------------------------*/
 /* 
    Useful information if using PID for Copter Controls
    It is also worth checking that the oscillations do not occur during sharp 
    descent (otherwise, reduce P).
    Slow copter rocking from side to side while trying to maintain the predetermined position
    is related to excessive value I.
    If the copter swings during movement, the value should be increased.
    If the copter keeps the preset position poorly, you should increase the D parameter;
    it the D parameter is too high or too low, oscillations occur.
    The Rate Pitch and Rate Roll parameters should be the same.
    YAW parameters should be changed individually,
*/
uint8_t computePID( pidBlock_t *pidObj )                                        //compute(int32_t setpoint, int32_t feedback)  kp ki kd  lastError   lasttime output
{

  int64_t sampleTime=0;
  int32_t curError;
  float32_t pTerm;
  float32_t iTerm;
  float32_t dTerm;
  int32_t diffInput;
  float32_t RC;

  if (pidObj->mode_began == false)                                              // time initialisation request
  {
     sampleTime = -1;                                                           // initialise the timer
     pidObj->lastFeedBack = pidObj->feedback;
     pidObj->mode_began = true;                                                 // initialisation of time complete
     pidObj->last_derivative = NaN;                                             // for first time filter
     pidObj->lastError = 0;
     pidObj->integral = 0;
     pidObj->lastIntegral = 0;
     if (pidObj->type == pid_Type_craZyFlie) pidObj->filter_derivative = lpf2pReset(&pidObj->dFilter, pidObj->filter_derivative);
  }

  //   curTime = micros();                                                      get the current time in micro seconds from the interrupt counter !!!
  calculateTimeDiff(&sampleTime, &pidObj->lasttime);                              // time in 100 ms counts from the interrupt

  if ((sampleTime <= 0u) || (pidObj->mode_hld==true))                           // no time difference or hold then exit
  {
     pidObj->lastFeedBack = pidObj->feedback;
     return(0u);                                                                // exit function its the first time or interrupt not working  would give divide by zero error (exit)
  }

  sampleTime /= 10u;                                                            // time in second interrupt is 100 ms counts
  if (pidObj->mode_rem == false)                                                // in local
  {
    if (pidObj->thrust_tilt_comp==true)                                         /* ------ tilt compensation if its a thrust PID ------ */
    {
      pidObj->setpoint = pidObj->setpoint / sensfusion6GetInvThrustCompensationForTilt();
    }
    curError = pidObj->setpoint - pidObj->feedback;                             // find the error between measurement feedback and requested value
  }
  else                                                                          // in remote
  {
    if (pidObj->thrust_tilt_comp==true)                                         /* ------ tilt compensation if its a thrust PID ------ */
    {
      pidObj->rem_setpoint = pidObj->rem_setpoint / sensfusion6GetInvThrustCompensationForTilt();
    }
    curError = pidObj->rem_setpoint - pidObj->feedback;                         // find the error between measurement feedback and requested value
    if (pidObj->mode_bmp == true)                                               // if bumpless on let local spt track remote
    {
       pidObj->setpoint =  pidObj->rem_setpoint;
    }
  }

  if (pidObj->mode_reverse==1u)
  {
     pTerm = -pidObj->kp * curError;
  }
  else
  {
     pTerm = pidObj->kp * curError;                                             /* proportional term computation */
  }

  if(pidObj->ki == 0.0f)                                                        /* integral term computation */
    pidObj->integral = 0u;
  else
  {
    if (pidObj->mode_reverse==1u)
    {
       iTerm = -pidObj->ki * curError * sampleTime;                             // calculate itegral
    }
    else
    {
       iTerm = pidObj->ki * curError * sampleTime;                              // calculate itegral
    }
    if(iTerm > 0u)
      iTerm  = ceil(iTerm);                                                     // round up
    else
      iTerm  = floor(iTerm);                                                    // round down

    iTermMax = integralMax / pidObj->ki;                                        /* iterm max is (output max) / K[integral] */
    iTermMin = -(integralMax / pidObj->ki);                                     /* iterm min is -iterm max */

    if(iTermMax != 0u &&(iTerm > iTermMax))                                     /* Check iTerm bound */
      iTerm = iTermMax;                                                         /* cap the maximum integral to avoid windup */
    if(iTermMin != 0u &&(iTerm < iTermMin))
      iTerm = iTermMin;                                                         /* cap the minimum integral to avoid windup */
    pidObj->lastIntegral = pidObj->integral;
    pidObj->integral += iTerm;

    if(pidObj->integral > 0u && iTerm < 0u && pidObj->lastIntegral < 0u)        /* Check  integral overflow  */
      pidObj->integral = INT32_MIN;
    else if(pidObj->integral < 0u && iTerm > 0u && pidObj->lastIntegral > 0u)
      pidObj->integral = INT32_MAX;

    if(integralMax != 0u &&(pidObj->integral > integralMax))                    /* Check integral bound */
      pidObj->integral = integralMax;

    if(integralMin != 0u &&(pidObj->integral < integralMin))
      pidObj->integral = integralMin;

    pidObj->lastIntegral = pidObj->integral;                                    // remember last
    pidObj->integral += iTerm;                                                  // integrate
  }

  if (pidObj->mode_reverse==1u)
  {
         switch(pidObj->type)
         {
              case pid_Type_Der1:                                                /* type 1 pid */
              dTerm = -pidObj->kd * (curError - pidObj->lastError) / sampleTime;
              break;

              case pid_Type_Der2:                                                /* type 2 pid */
              diffInput = pidObj->feedback - pidObj->lastFeedBack;              // used for 2nd type calculation
              dTerm = -pidObj->kd * diffInput;
              break;

              case pid_Type_ArduPiDer3:                                         /* ardupilot pid derivative compensation */
              // Compute derivative component if time has elapsed
              // This correction comes from the ArduPilot project
              //
              if (IsaNan(pidObj->last_derivative))
              {
                // we've just done a reset, suppress the first derivative
                // term as we don't want a sudden change in input to cause
                // a large D output change
                pidObj->filter_derivative = 0;
                pidObj->last_derivative = 0;
                pidObj->cutoffFreq = PID_fCut;                                  // set the cutoff frequency from the define
              }
              else
              {
                 pidObj->filter_derivative = (curError - pidObj->lastError) / sampleTime;
                 // discrete low pass filter, cuts out the
                 // high frequency noise that can drive the controller crazy
                 RC = 1.0f/(2.0f*PI*pidObj->cutoffFreq);                        /* fCut defines the freq typically 20Hz */
                 pidObj->filter_derivative = pidObj->last_derivative + ((sampleTime / (RC + sampleTime)) * (pidObj->filter_derivative - pidObj->last_derivative));
                 pidObj->lastError = curError ;                                 // update state
                 pidObj->last_derivative = pidObj->filter_derivative;
              }
              dTerm = -pidObj->kd  * pidObj->filter_derivative;                 // subtract derivative component
              break;

              case pid_Type_craZyFlie:                                          /* chose the craZyflie lpf2 filter */
              if (IsaNan(pidObj->last_derivative))                              /* initialise the filter */
              {
                 lpf2pInit(&pidObj->dFilter, PID_samplingRate, pidObj->cutoffFreq);
                 dTerm = 0u;                                                    /* no derivative to START */
              }
              else
              {
                 pidObj->filter_derivative = (curError - pidObj->lastError) / sampleTime;  /* calculate derivative from error and last error */
                 pidObj->filter_derivative = lpf2pApply(&pidObj->dFilter, pidObj->filter_derivative);  /* apply the lpf2 filter to the derivative */
                 if (IsaNan(pidObj->last_derivative)) pidObj->filter_derivative = 0u;  /* not a number will yield a zero */
                 dTerm = -pidObj->kd  * pidObj->filter_derivative;              // subtract derivative component
              }
              break;
              
              default:                                                          /* type not supported PI only */
              break;
         }
  }
  else
  {
         switch(pidObj->type)
         {
             case pid_Type_Der1:
             dTerm = pidObj->kd * (curError - pidObj->lastError) / sampleTime;  /* differential term computation */
             break;

             case pid_Type_Der2:                                                /* type 2 pid */
             diffInput = pidObj->feedback - pidObj->lastFeedBack;               // used for 2nd type calculation
             dTerm = pidObj->kd * diffInput;
             break;

             case pid_Type_ArduPiDer3:                                          /* ardupilot derivative compensation filter */
             // Compute derivative component if time has elapsed
             // This correction comes from the ArduPilot project
             //
             if (IsaNan(pidObj->last_derivative))                               /* first time round */
             {
                // we've just done a reset, suppress the first derivative
                // term as we don't want a sudden change in input to cause
                // a large D output change
                pidObj->filter_derivative = 0;
                pidObj->last_derivative = 0;
                pidObj->cutoffFreq = PID_fCut;                                  // set the cutoff frequency from the define
             }
             else
             {
                pidObj->filter_derivative = (curError - pidObj->lastError) / sampleTime;   /* calculate derivative from the error */
                // discrete low pass filter, cuts out the
                // high frequency noise that can drive the controller crazy
                RC = 1.0f/(2.0f*PI*pidObj->cutoffFreq);
                pidObj->filter_derivative = pidObj->last_derivative + ((sampleTime / (RC + sampleTime)) * (pidObj->filter_derivative - pidObj->last_derivative));
                pidObj->lastError = curError ;                                  // update state
                pidObj->last_derivative = pidObj->filter_derivative;
             }
             dTerm = pidObj->kd  * pidObj->filter_derivative;                   // add derivative component
             break;

              case pid_Type_craZyFlie:                                          /* chose the craZyflie lpf2 filter */
              if (IsaNan(pidObj->last_derivative))
              {
                 lpf2pInit(&pidObj->dFilter, PID_samplingRate, pidObj->cutoffFreq);   /* initialise the filter */
                 dTerm = 0u;                                                    /* no derivative at the beginning */
              }
              else
              {
                 pidObj->filter_derivative = (curError - pidObj->lastError) / sampleTime;  /* calculate derivative from error and last error */
                 pidObj->filter_derivative = lpf2pApply(&pidObj->dFilter, pidObj->filter_derivative);   /* apply the lpf2 filter to the derivative */
                 if (IsaNan(pidObj->last_derivative)) pidObj->filter_derivative = 0u;  /* zero if its not an number */
                 dTerm = pidObj->kd  * pidObj->filter_derivative;               // subtract derivative component
              }
              break;
              
             default:                                                           /* type not supported PI only */
             break;
         }
  }
  pidObj->lastError = curError;                                                 // save last error calculation

  if (((pidObj->mode_man==false) && (pidObj->mode_trk==false)) && (pidObj->mode_init==false))
  {                                                                             // dont write output in manual state or track is on
    pidObj->fout = pTerm + pidObj->integral + dTerm;
    pidObj->fout = clampReal(pidObj->fout,0.0f,100.0f);
    pidObj->output = ROUND((pTerm + pidObj->integral + dTerm),0);               // return the output for the PID loop
    pidObj->output = clampU32(pidObj->output,0u,100u);
    return (1u);                                                                // return success from auto
  }
  else if (pidObj->mode_trk==true)                                              // track means set output to measurement
  {
    pidObj->fout = ((float32_t)pidObj->feedback);
    pidObj->fout = clampReal(pidObj->fout,0.0f,100.0f);
    pidObj->output = pidObj->feedback;
    pidObj->output = clampU32(pidObj->output,0u,100u);
    return (2u);                                                                // return success from track
  }
  else if (pidObj->mode_init==true)                                             // initialise sets output to specified value or 0 (reset function)
  {
    pidObj->fout = ((float32_t)pidObj->initVal);
    pidObj->fout = clampReal(pidObj->fout,0.0f,100.0f);
    pidObj->output = pidObj->initVal;                                           // can be set to the downstream output when in remote or to zero for reset function
    pidObj->output = clampU32(pidObj->output,0u,100u);
    return (3u);                                                                // return success from init
  }
  else if (pidObj->mode_man==true)                                              // set to be in manual set the ouptut via hmi
  {
    return (4u);                                                                // return success in manual
  }
  else
  {
    return (5u);                                                                  // return its in the wrong mode continue
  }
}

// %%%%%% The AUTO Tuning process Ref : JiangTingjia(kyzy_duck@163.com) %%%%%%
/*-----------------------------------------------------------------------------
 *      JingTing_SetLookbackSec : set loop feedback
 *                             
 *  Parameters: int16_t value, pideAutoTuner_t *sTempAuto
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void JingTing_SetLookbackSec(int16_t value, pideAutoTuner_t *sTempAuto)
{
        if (value < 1) value = 1;

        if (value < 25)
        {
                sTempAuto->nLookBack = value * 4;
                sTempAuto->sampleTime = 250;
        }
        else
        {
                sTempAuto->nLookBack = 100;
                sTempAuto->sampleTime = value * 10;
        }
}
/*-----------------------------------------------------------------------------
 *      JingTing_PID_ATune : auto tuner
 *                             
 *  Parameters: float64_t* Input, float64_t* Output, float64_t *setpoint, pideAutoTuner_t *sTempAuto
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void JingTing_PID_ATune(float64_t* Input, float64_t* Output, float64_t *setpoint, pideAutoTuner_t *sTempAuto)
{
        float64_t noiseBand;        
        float64_t oStep;                
        int8_t LookBack = 10;                
        float64_t StartVal;        
        sTempAuto->Current_time = 0.0f;

        sTempAuto->input = Input;                                               // Connect pointer to current PID
        sTempAuto->output = Output;
        sTempAuto->setpoint = setpoint;

        sTempAuto->controlType = 0;                                             // default PI
        noiseBand = *(sTempAuto->setpoint) * 0.02f;                             //default  2% of SP
        sTempAuto->noiseBand = noiseBand;
        oStep = *(sTempAuto->output);                                           // default step
        sTempAuto->oStep = oStep;
        StartVal = *(sTempAuto->output);                                        // default StartV
        sTempAuto->outputStart = StartVal;
        JingTing_SetLookbackSec(LookBack, sTempAuto);        
        sTempAuto->running = false;        
        sTempAuto->timeLast = g_timer5_counter;
}
/*-----------------------------------------------------------------------------
 *      JingTing_Cancel : cancel the auto tuner
 *                             
 *  Parameters: pideAutoTuner_t *sTempAuto
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void JingTing_Cancel(pideAutoTuner_t *sTempAuto)
{
        sTempAuto->running = false;
}
/*-----------------------------------------------------------------------------
 *      JingTing_FinishUp : auto tuner calculate params
 *                             
 *  Parameters: pideAutoTuner_t *sTempAuto
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void JingTing_FinishUp(pideAutoTuner_t *sTempAuto)
{
        *(sTempAuto->output) = sTempAuto->outputStart;
        sTempAuto->Ku = 4.0f * (2.0f * sTempAuto->oStep) / ((sTempAuto->absMax - sTempAuto->absMin)*3.14159f);
        sTempAuto->Pu = (float64_t)(sTempAuto->peak1 - sTempAuto->peak2) / 1000.0f;
}
/*-----------------------------------------------------------------------------
 *      JingTing_TunePI : PI Controller auto tuner 
 *                             
 *  Parameters: pideAutoTuner_t *sTempAuto
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
int8_t JingTing_TunePI(pideAutoTuner_t *sTempAuto)
{

        float64_t refVal = 0.0f;
        float64_t avgSeparation;
        float64_t val;        
        int16_t i = 0;        
        
        sTempAuto->justevaled = false;                                
        if (sTempAuto->peakCount > 5 && sTempAuto->running)
        {
                sTempAuto->running = false;        
                JingTing_FinishUp(sTempAuto);                
                return 1;
        }
        calculateTimeDiff(&sTempAuto->Current_time,&sTempAuto->timeLast);                
        /***********************************************/
        if (sTempAuto->Current_time < sTempAuto->sampleTime) 
        {
                return 0;
        }
        /***********************************************/
        
        refVal = *(sTempAuto->input);                
        sTempAuto->justevaled = true;                
        
        if (!sTempAuto->running)
        {
                sTempAuto->peakType = 0;                                        //initialize working variables the first time around
                sTempAuto->peakCount = 0;                
                sTempAuto->justchanged = false;                
                sTempAuto->absMax = refVal;
                sTempAuto->absMin = refVal;
                sTempAuto->running = true;                
                *(sTempAuto->output) = sTempAuto->outputStart + sTempAuto->oStep;
        }
        else
        {
                if (refVal > sTempAuto->absMax)
                {
                        sTempAuto->absMax = refVal;
                }
                if (refVal < sTempAuto->absMin)
                {
                        sTempAuto->absMin = refVal;
                }
        }

        if (refVal > *(sTempAuto->setpoint) + sTempAuto->noiseBand)             //oscillate the output base on the input's relation to the setpoint
        {
                *(sTempAuto->output) = sTempAuto->outputStart - sTempAuto->oStep;
        }
        else if (refVal < *(sTempAuto->setpoint) - sTempAuto->noiseBand) 
        {
                *(sTempAuto->output) = sTempAuto->outputStart + sTempAuto->oStep;
        }

        sTempAuto->isMax = true; sTempAuto->isMin = true;                
        for (i = sTempAuto->nLookBack - 1; i >= 0; i--)
        {
                val = sTempAuto->lastInputs[i];
                if (sTempAuto->isMax) 
                {
                        sTempAuto->isMax = refVal > val;
                }
                if (sTempAuto->isMin) 
                {
                        sTempAuto->isMin = refVal < val;
                }
                sTempAuto->lastInputs[i + 1] = sTempAuto->lastInputs[i];
        }
        sTempAuto->lastInputs[0] = refVal;

        if (sTempAuto->nLookBack < 9)                                           //we don't want to trust the maxes or mins until the inputs array has been filled
        {
                return 0;
        }

        if (sTempAuto->isMax)
        {
                if (sTempAuto->peakType == 0)sTempAuto->peakType = 1;
                if (sTempAuto->peakType == -1)
                {
                        sTempAuto->peakType = 1;
                        sTempAuto->justchanged = true;
                        sTempAuto->peak2 = sTempAuto->peak1;
                }
                sTempAuto->peak1 = sTempAuto->Current_time;
                sTempAuto->peaks[sTempAuto->peakCount] = refVal;
        }
        else if (sTempAuto->isMin)
        {
                if (sTempAuto->peakType == 0)sTempAuto->peakType = -1;
                if (sTempAuto->peakType == 1)
                {
                        sTempAuto->peakType = -1;
                        sTempAuto->peakCount++;
                        sTempAuto->justchanged = true;
                }
                if (sTempAuto->peakCount < 10) sTempAuto->peaks[sTempAuto->peakCount] = refVal;
        }

        if (sTempAuto->justchanged && sTempAuto->peakCount > 3)
        { 
                avgSeparation = (fabs(sTempAuto->peaks[sTempAuto->peakCount - 1] - sTempAuto->peaks[sTempAuto->peakCount - 2]) + fabs(sTempAuto->peaks[sTempAuto->peakCount - 2] - sTempAuto->peaks[sTempAuto->peakCount - 3])) / 2;
                if (avgSeparation < 0.05f *(sTempAuto->absMax - sTempAuto->absMin))
                {
                        JingTing_FinishUp(sTempAuto);
                        sTempAuto->running = false;
                        return 1;
                }
        }
        
        sTempAuto->justchanged = false;
        return 0;
}
/*-----------------------------------------------------------------------------
 *      PWMSetDuty():  set pwm from PID output  takes pidObj.output and pwmperiod 
 *                     from the Init function and the channel for output
 *
 *  Parameters: float32_t pid_out, uint16_t pwm_period, uint16_t channel
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void PWMSetDuty( float32_t pid_out, uint16_t pwm_period, uint16_t channel )
{
   uint16_t pwmVal=0u;
   
   pwmVal=Scale_Float_Value(0.0f,100.0f,0u,pwm_period,pid_out);                 // scale pid output in range of pulse width modulation
   if (pwmVal > pwm_period)                                                     // if we increase current_duty greater then possible pwm_period1 value
   {
      pwmVal = pwm_period;                                                      // reset current_duty value to zero
   }
   if (pwm_period != 0xFFFFU)                                                   // period returned was not invalid
   {
      PWM_Set_Duty(pwmVal, channel);                                            // set newly acquired duty ratio
   }
}

/*-----------------------------------------------------------------------------
 *      DcMotorPIDloop():  DC Servo Motor Application using above PID  can be 
 *                        speed or angle/position
 *
 *  Parameters: pidBlock_t *pidObj, uint16_t pwm_period, uint16_t channel
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void DcMotorPIDloop(pidBlock_t *pidObj, uint16_t pwm_period, uint16_t channel)
{
  uint32_t feedback=0u;

  switch (channel)                                                              /* Depending on output channel decide which loop it is and choose measurement */
  {
     case 1u:                                                                   /* position only */
     //feedback = getEncoderPosition();
     break;
     case 2u:                                                                   /* speed only */
     //feedback = getSpeed();
     break;
  }
  pidObj->feedback = feedback;                                                  /* make feedback what you measured */
  while(!computePID(pidObj));                                                   /* do the PID loop more than once and calculate PWM width */

  if(pidObj->fout >= 0u)                                                        /* computed pid output is pwm  */
    PWMSetDuty( pidObj->fout, pwm_period, channel );

}

/*-----------------------------------------------------------------------------
 *      DcMotorComplexPIDloop():  DC Servo Motor Application using above PID  can be
 *                        speed or angle/position
 *
 *  Parameters: pidBlock_t *speedPidObj, pidBlock_t *posPidObj, uint16_t pwm_period, 
 *              uint16_t channel
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void DcMotorComplexPIDloop(pidBlock_t *speedPidObj, pidBlock_t *posPidObj, uint16_t pwm_period, uint16_t channel)
{
  uint32_t feedback=0u;


  //posPidObj->feedback = getEncoderPosition();
  //speedPidObj->feedback = getSpeed();
  posPidObj->mode_rem = false;
  if ((speedPidObj->mode_rem == true) && (speedPidObj->mode_man == false))      /* speed in remote gets setpoint from secondary loop of position */
  {
     speedPidObj->rem_setpoint=posPidObj->output;                               /* speed is primary loop in cascade set-up */
  }
  else if ((speedPidObj->mode_rem == false) && (speedPidObj->mode_man == false))
  {
     posPidObj->mode_init = true;                                               /* set secondary position track primary local setpoint */
     posPidObj->initVal = speedPidObj->setpoint;
  }
  else if (speedPidObj->mode_man == true)
  {
     speedPidObj->mode_init = true;                                             /* set primary speed controller to re-initialise with last output value */
     speedPidObj->initVal = speedPidObj->output;
  }
  else if (posPidObj->mode_man == true)
  {
     posPidObj->mode_init = true;                                               /* set secondary position controller to re-initialise with last output value */
     posPidObj->initVal = posPidObj->output;
  }

  while(!computePID(posPidObj));                                                /* do remote setpoint calculation */
  while(!computePID(speedPidObj));                                              /* do the PID loop more than once and calculate PWM width */

  if(speedPidObj->fout >= 0u)                                                   /* computed pid output is pwm  */
    PWMSetDuty( speedPidObj->fout, pwm_period, channel );

}

/*-----------------------------------------------------------------------------
 *      tunePID():  Auto a PID using ziegler nicholls method
 *
 *  Parameters: pidBlock_t *pidObj, pidTuner_t *pidTuner
 *
 *  Return: success=1
 *----------------------------------------------------------------------------*/
uint8_t tunePID( pidBlock_t *pidObj, pidTuner_t *pidTuner )
{
  // Useful information on the algorithm used (Ziegler-Nichols method/Relay method)
  // http://www.processcontrolstuff.net/wp-content/uploads/2015/02/relay_autot-2.pdf
  // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
  // https://www.cds.caltech.edu/~murray/courses/cds101/fa04/caltech/am04_ch8-3nov04.pdf

  // Basic explanation of how this works:
  //  * Turn on the output of the PID controller to full power
  //  * Wait for the output of the system being tuned to reach the target input value
  //      and then turn the controller output off
  //  * Wait for the output of the system being tuned to decrease below the target input
  //      value and turn the controller output back on
  //  * Do this a lot
  //  * Calculate the ultimate gain using the amplitude of the controller output and
  //      system output
  //  * Use this and the period of oscillation to calculate PID gains using the
  //      Ziegler-Nichols method

  uint64_t t1=0LLU,t2=0LLU;
  int64_t microseconds=0;
  float32_t ku;
  float32_t tu;
  float32_t tHigh;
  float32_t tLow;
  float32_t max=0.0f,maxOutput=100.0f;
  float32_t min=100.0f,minOutput=0.0f;
  float32_t kpConstant, tiConstant, tdConstant;
  float32_t pAverage=0.0f, iAverage=0.0f, dAverage=0.0f;
  uint8_t i=0u;
  float32_t loopInterval;

  float32_t deltaT;

  if (pidTuner->init == false)                                                  // first use of the tuner (reset from top level)
  {
     i = 0u;                                                                    // Cycle counter
     pidObj->mode_man = true;                                                   // Control state is manual set the output to max
     pidObj->output = 100u;                                                     // set output to start at maximum
     t1 = t2 = g_timer5_counter;                                                // Times used for calculating period
     tHigh = tLow = 0.0f;
     max = 0.0f;                                                                // Max input
     min = 100.0f;                                                              // Min input
     pAverage = iAverage = dAverage = 0.0f;
     pidTuner->init = true;
     pidObj->mode_hld = false;                                                  // Control state is not hold
     return(0u);                                                                // Return without completion
  }
  else
  {
     pidObj->mode_man = false;                                                  // Put the PID back to auto after setting it to max in the first iteration
  }
  
  calculateTimeDiff(&microseconds, &pidObj->lasttime);                          // calculate a time difference from last time
  if (microseconds >= 0)
  {
    deltaT = microseconds;
    loopInterval = deltaT;                                                      // make loop interval the time since last time
  }
  else
  {
     return(0u);                                                                // first time initialise wait til next iteration
  }

  if (max <= pidObj->output)                                                    // Calculate max and min
          max = pidObj->output;
  else if (min >= pidObj->output)
          min = pidObj->output;

  minOutput - min;
  maxOutput = max;

  if (!(pidObj->mode_hld) && (pidObj->output > pidTuner->targetInputValue))     // Output is on and input signal has risen to target
  {
    // Turn output off, record current time as t1, calculate tHigh, and reset maximum
    pidObj->mode_hld = true;
    //pidObj->output = minOutput;                                                 // pid output
    t1 = g_timer5_counter;
    if ( t2 > t1 )                                                              // Overrun of iterrrupt 100 ms counts
    {
       tHigh = (UINT64_MAX - t2) + t1;                                          // Calculate difference in time
    }
    else
    {
       tHigh = (t1 - t2);                                                       // Calculate difference in time
    }
    // tHigh = t1 - t2;
    max = pidTuner->targetInputValue;
  }

  // Output is off and input signal has dropped to target
  if ((pidObj->mode_hld==true) && pidObj->output < pidTuner->targetInputValue)
  {
    pidObj->mode_hld = false;                                                   // Turn output on (donthold), record current time as t2, calculate tLow
    //pidObj->output = maxOutput;
    t2 = g_timer5_counter;
    if ( t1 > t2 )                                                              // Overrun
    {
       tLow = (UINT64_MAX - t1) + t2;                                           // Calculate difference in time
    }
    else
    {
       tLow = (t2 - t1);                                                        // Calculate difference in time
    }
    //tLow = t2 - t1;

    // Calculate Ku (ultimate gain)
    // Formula given is Ku = 4d / pa
    // d is the amplitude of the output signal
    // a is the amplitude of the input signal
    ku = (4.0f * ((maxOutput - minOutput) / 2.0f)) / (PI * (max - min) / 2.0f);    // calculated gain

    // Calculate Tu (period of output oscillations)
    tu = (tLow * 10.0f) + (tHigh * 10.0f);                                      // calculated in 100 milliseconds slices

    // How gains are calculated
    // PID control algorithm needs Kp, Ki, and Kd
    // Ziegler-Nichols tuning method gives Kp, Ti, and Td
    //
    // Kp = 0.6Ku = Kc
    // Ti = 0.5Tu = Kc/Ki
    // Td = 0.125Tu = Kd/Kc
    //
    // Solving these equations for Kp, Ki, and Kd gives this:
    //
    // Kp = 0.6Ku
    // Ki = Kp / (0.5Tu)
    // Kd = 0.125 * Kp * Tu

    // Constants
    // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method


    switch (pidTuner->znMode)                                                   // possible modes of tune set in the tune object
    {
        case ZNModeBasicPID:
        kpConstant = 0.6f;                                                      // 20 / 34
        tiConstant = 0.5f;                                                      // 20 / 40
        tdConstant = 0.125f;                                                    // 20 / 160
        break;
        
        case ZNModeLessOvershoot:
        kpConstant = 0.33f;                                                     // 20 / 60
        tiConstant = 0.5f;                                                      // 20 / 40
        tdConstant = 0.33f;                                                     // 20 / 60
        break;
        
        case ZNModeNoOvershoot:
        kpConstant = 0.2f;                                                      // 20 /100
        tiConstant = 0.5f;                                                      // 20 / 40
        tdConstant = 0.33f;                                                     // 20 / 60
        break;
        
        case tyreus_luyben: 
        kpConstant = 0.4636363f;                                                // 20 / 44
        tiConstant = 2.2222222f;                                                // 20 / 9
        tdConstant = 0.1587301f;                                                // 20 / 126
        break;
        
        case ciancone_marlin:
        kpConstant = 0.3090909f;                                                // 20 / 66
        tiConstant = 0.2272727f;                                                // 20 / 88
        tdConstant = 0.1234567f;                                                // 20 / 162
        break;  
        
        case pessen_integral:
        kpConstant = 0.7285714f;                                                // 20 / 28
        tiConstant = 0.4f;                                                      // 20 / 50
        tdConstant = 0.1503759f;                                                // 20 / 133
        break;
        
        case kettle_brewer:
        kpConstant = 8.0f;                                                      // 20 / 2.5
        tiConstant = 3.33333333f;                                               // 20 / 6
        tdConstant = 0.05263157f;                                               // 20 / 380
        break;

        case PCUnderdampedPID:                                                  // Petit Carr underdamped
        kpConstant = 1.0f;                                                      // 20 / 34
        tiConstant = 0.5f;                                                      // 20 / 40
        tdConstant = 0.125f;                                                    // 20 / 160
        break;

        case PCCriticaldampedPID:                                               // Petit Carr critically damped
        kpConstant = 0.67f;                                                      // 20 / 34
        tiConstant = 1.0f;                                                      // 20 / 40
        tdConstant = 0.167f;                                                    // 20 / 160
        break;

        case PCOverdampedPID:                                                   // Petit Carr over damped
        kpConstant = 0.5f;                                                      // 20 / 34
        tiConstant = 1.5f;                                                      // 20 / 40
        tdConstant = 0.167f;                                                    // 20 / 160
        break;

        case BuczOvershootPID:                                                  // Bucz Overshoot ?max=20%  ref : Faculty of Electrical Engineering and Information Technology, Slovak University of Technology, Bratislava, Slovak RepublicFaculty of Electrical Engineering and Information Technology, Slovak University of Technology, Bratislava, Slovak Republic
        kpConstant = 0.54f;                                                     
        tiConstant = 0.79f;                                                     
        tdConstant = 0.199f;                                                    
        break;

        case BuczSettlePID:                                                     // Bucz  Settling time ts=13/?c
        kpConstant = 0.28f;                                                     
        tiConstant = 1.44f;                                                     
        tdConstant = 0.359f;                                                    
        break;

        case Chau1PID:                                                          // No overshoot
        kpConstant = 0.2f;                                                     
        tiConstant = 0.55f;                                                     
        tdConstant = 0.333f;                                                    
        break;

        case Chau2PID:                                                          // small overshoot
        kpConstant = 0.33f;                                                     
        tiConstant = 0.5f;                                                     
        tdConstant = 0.333f;                                                    
        break;
                                                                
        default:
        break;
             
    } 

    
    pidObj->kp = kpConstant * ku;                                               // Calculate gains
    pidObj->ki = (pidObj->kp / (tiConstant * tu)) * loopInterval;
    pidObj->kd = (tdConstant * pidObj->kp * tu) / loopInterval;

    if (i > 1u)
    {
      pAverage += pidObj->kp;                                                   // Average all gains after the first two cycles
      iAverage += pidObj->ki;
      dAverage += pidObj->kd;
    }

    min = pidTuner->targetInputValue;                                           // Reset minimum

    i= ++i % UINT8_MAX;                                                         // Increment cycle count
  }

  if (i >= pidTuner->cycles)                                                    // If loop is done, disable output and calculate averages
  {
     pidObj->mode_hld = true;
     pidObj->output = minOutput;
     pidObj->kp = pAverage / (i - 1u);
     pidObj->ki = iAverage / (i - 1u);
     pidObj->kd = dAverage / (i - 1u);
     return (1u);                                                               // return complete state
  }
  return(0u);                                                                   // wait until all cycles done iteration is from top loop

}
#endif                                                                          // end of PID loops library

#ifdef ROBOT_HELPER                                                             // include the robot helper
/////////------------------- Robotics helpers ----------------------------------
//////// Refer open source project https://github.com/Nate711/Doggo/blob/7f2044c0d9b43046c2e013c68de10a85d67f346c/src/position_control.cpp
///////  NATE 711 Doggo

/**
* Takes the leg parameters and returns the gamma angle (rad) of the legs
*/
#include "odrive.h"                                                             // communication helper library not misra as include consider moving whole section to a new file
GaitParams state_gait_params[13u] = {                                           // This structure matches the enum types above and is of type gait

    //{s.h, d.a., u.a., f.p., s.l., fr., s.d.}

    {NaN, NaN, NaN, NaN, NaN, NaN, NaN},                                        // STOP

    {0.17f, 0.04f, 0.06f, 0.35f, 0.15f, 2.0f, 0.0f},                            // TROT

    {0.17f, 0.04f, 0.06f, 0.35f, 0.0f, 2.0f, 0.0f},                             // BOUND

    {0.15f, 0.00f, 0.06f, 0.25f, 0.0f, 1.5f, 0.0f},                             // WALK

    {0.12f, 0.05f, 0.0f, 0.75f, 0.0f, 1.0f, 0.0f},                              // PRONK

    {NaN, NaN, NaN, NaN, NaN, NaN, NaN},                                        // JUMP

    {0.15f, 0.05f, 0.05f, 0.35f, 0.0f, 1.5f, 0.0f},                             // DANCE

    {0.15f, 0.05f, 0.05f, 0.2f, 0.0f, 1.0f, 0.0f},                              // HOP

    {NaN, NaN, NaN, NaN, NaN, 1.0f, NaN},                                       // TEST

    {NaN, NaN, NaN, NaN, NaN, NaN, NaN},                                        // ROTATE

    {0.15f, 0.07f, 0.06f, 0.2f, 0.0f, 1.0f, 0.0f},                              // FLIP

    {0.17f, 0.04f, 0.06f, 0.35f, 0.1f, 2.0f, 0.06f},                            // TURN_TROT

    {NaN, NaN, NaN, NaN, NaN, NaN, NaN}                                         // RESET

};

LegGain gait_gains = {80f, 0.5f, 50f, 0.5f};
/*-----------------------------------------------------------------------------
 *      readJoyStickInput:  read joystick input
 *
 *  Parameters: uint16_t xIn, uint16_t yIn, uint16_t rotation, roboMtr_t *MotorType
 *
 *  Return: none
 *----------------------------------------------------------------------------*/
void readJoyStickInput( uint16_t xIn, uint16_t yIn, uint16_t rotation, roboMtr_t *MotorType )
{

  MotorType->kFrontLeft_val = xIn + yIn + rotation;
  MotorType->kFrontRight_val = -xIn + yIn - rotation;
  MotorType->kRearLeft_val = -xIn + yIn + rotation;
  MotorType->kRearRight_val = xIn + yIn - rotation;

}

/**
 * XOR operation on first 8 bits and last 8 bits of a short
 * @param  val  The short to work on
 * @return      Byte resulting from xoring the first 8 and last 8 bits.
 */
uint8_t XorShort(int16_t val) 
{
    uint8_t v0 = val & 0xFFU;
    uint8_t v1 = (val >> 8U) & 0xFFU;
    return v0 ^ v1;
}
/**
* Parses the encoder position message and stores positions as counts
* Assumes the message is in format "<1><6><'P'><short1><short2><checksum>"
* This would greatly improve the noise on the Kd term!!
* @param msg    String: Message to parse
* @param th     float&: Output parameter for theta reading
* @param ga     float&: Output parameter for gamma reading
* @return       int:    1 if success, -1 if failed to find get full message or checksum failed
*/
int16_t ParseDualPosition(char* msg, int16_t len, float32_t  *th, float32_t *ga)
{
    uint8_t rcvdCheckSum;
    uint16_t th_16;
    uint16_t ga_16;
    uint8_t checkSum;
    
    if (len != 6)                                                               /* check if 1 byte for "P", 4 bytes holding encoder data, and 1 checksum byte were received */
    {
        return -1;                                                              /* return -1 to indicate that the message length was wrong */
    } 
    else if (msg[0U] == 'P' )
    {
        /* retrieve short from byte stream
         remember that the first character is 'P'  */
        th_16 = (msg[2U] << 8U) | msg[1U];
        ga_16 = (msg[4U] << 8U) | msg[3U];
        rcvdCheckSum = msg[5U];

        /* compute checksum */
        checkSum = 0U;
        checkSum ^= msg[0U];                                                    // letter 'P'
        checkSum ^= msg[1U];
        checkSum ^= msg[2U];
        checkSum ^= msg[3U];
        checkSum ^= msg[4U];

        // if the computed and received check sums match then update the motor position variables
        if (checkSum == rcvdCheckSum) 
        {
            *th = ((float32_t)th_16)/1000.0f;
            *ga = ((float32_t)ga_16)/1000.0f;
            return 1;
        } 
        else 
        {
            // return -1 to indicate that the checksums didn't match
            return -1;
        }
    }
    else
    {
       return -1;
    }
    return 1;
}
/**
* ParseMtrStateReply parses motor state reply message
* @param msg    String: Message to parse
* @param th     float&: Output parameter for theta reading
* @param ga     float&: Output parameter for gamma reading
* @return       int:    Axis state is returned if success, -1 if failed
*/
int16_t ParseMtrStateReply(char* msg)
{
    char * pch = NULL;
    uint8_t numOfStr = 0U;                                                      // number of strings found
    uint8_t axisState;                                                          // axis state readback
    
    if (!strncmp(msg,"r axis",6U))                                              // message is r axis
    {
       pch = strtok (msg," ");                                                  // split it by space
       while (pch != NULL)
       {
          numOfStr = ++numOfStr % UINT8_MAX;
          if (numOfStr==3U)                                                     // 3rd value on the line
          {
              axisState=atoi(pch);
              if ((axisState<=AXIS_STATE_UNDEFINED) && (axisState>=AXIS_STATE_CLOSED_LOOP_CONTROL))
                  return axisState;
              else
                  return -1;
              pch = strtok (NULL, " ");                                         // get the next string in the line
          }
       }
    }
    return -1;
}
/**
* SetPositionUsingGains set position using gains
* @param gains    LegGain: leg gains to odrive
* @param sndBuf   char *: buffer message to odrive
*/
void SetPositionUsingGains( const LegGain *gains, char* sndBuf )
{
    const int POS_MULTIPLIER = 1000U;
    const int GAIN_MULTIPLIER = 100U;
    ODRIVE_SndPkt_t odrive;
    uint8_t checkSum;

    int16_t kp_theta_16;
    int16_t kd_theta_16;
    int16_t kp_gamma_16;
    int16_t kd_gamma_16;

    kp_theta_16 = (gains->kp_theta * GAIN_MULTIPLIER);
    kd_theta_16 = (gains->kd_theta * GAIN_MULTIPLIER);
    kp_gamma_16 = (gains->kp_gamma * GAIN_MULTIPLIER);
    kd_gamma_16 = (gains->kd_gamma * GAIN_MULTIPLIER);

    // Calculate the checksum based on the 2 current value shorts
    checkSum = 'S';                                                             // what it was spec says a crc8 ? of CRC8 of bytes 0 and 1
    checkSum ^= XorShort(kp_theta_16);
    checkSum ^= XorShort(kd_theta_16);
    checkSum ^= XorShort(kp_gamma_16);
    checkSum ^= XorShort(kd_gamma_16);

    // Command in Stream format
    odrive.Stx = 0xAAU;                                                         // send start byte 0xAA
    odrive.Length=14U;                                                          // payload length
    odrive.CRC8='S';                                                            // dual current command
    odrive.DataV[0U]=kp_theta_16;
    odrive.DataV[1U]=kd_theta_16;
    odrive.DataV[2U]=kp_gamma_16;
    odrive.DataV[3U]=kd_gamma_16;
    odrive.checksum=checkSum;

    memcpy(sndBuf, &odrive, sizeof(odrive));
    memset(sndBuf+(odrive.Length+1U),(void *) '\0',sizeof(char));
}
/**
* SetThetaGamma set theta and gamma
* @param theta    float32_t: theta
* @param gamma   float32_t: gamma
* @param sndBuf char : send buffer containing message to odrive
*/
void SetThetaGamma(float32_t theta, float32_t gamma, char *sndBuf)
{
    const int16_t MULTIPLIER = 1000;
    ODRIVE_SndPkt_t odrive;
    int16_t theta_16;
    int16_t gamma_16;
    uint8_t checkSum;

    // Calculate the checksum based on the 2 current value shorts
    checkSum = 'P';
    theta_16 = (theta * MULTIPLIER);
    gamma_16 = (gamma * MULTIPLIER);
    checkSum ^= XorShort(theta_16);
    checkSum ^= XorShort(gamma_16);

    // Send off bytes
    odrive.Stx = 0xAAU;                                                         // send start byte 0xAA
    odrive.Length=14U;                                                          // payload length
    odrive.CRC8='S';                                                            // dual current command
    odrive.DataV[0U]=theta_16;
    odrive.DataV[1U]=gamma_16;
    odrive.checksum=checkSum;

    memcpy(sndBuf, &odrive, sizeof(odrive));
    memset(sndBuf+(odrive.Length+1U),(void *) '\0',sizeof(char));
}

/**
* SetCoupledPosition set coupled position
* @param sp_theta    float32_t: theta
* @param sp_gamma   float32_t: gamma
* @param gains   LegGain*: leg gains
* @param sndBuf char : send buffer containing message to odrive
*/
void SetCoupledPosition(float32_t sp_theta, float32_t sp_gamma,const LegGain *gains, char *sndBuf)
{

    const int16_t POS_MULTIPLIER = 1000;
    const int16_t GAIN_MULTIPLIER = 100;
    uint8_t checkSum;
    int16_t sp_theta_16;
    int16_t kp_theta_16;
    int16_t kd_theta_16;
    int16_t sp_gamma_16;
    int16_t kp_gamma_16;
    int16_t kd_gamma_16;
    ODRIVE_SndPkt_t odrive;

    sp_theta_16 = (sp_theta * POS_MULTIPLIER);
    kp_theta_16 = (gains->kp_theta * GAIN_MULTIPLIER);
    kd_theta_16 = (gains->kd_theta * GAIN_MULTIPLIER);

    sp_gamma_16 = (sp_gamma * POS_MULTIPLIER);
    kp_gamma_16 = (gains->kp_gamma * GAIN_MULTIPLIER);
    kd_gamma_16 = (gains->kd_gamma * GAIN_MULTIPLIER);

    // Calculate the checksum based on the 2 current value shorts
    checkSum = 'S';
    checkSum ^= XorShort(sp_theta_16);
    checkSum ^= XorShort(kp_theta_16);
    checkSum ^= XorShort(kd_theta_16);
    checkSum ^= XorShort(sp_gamma_16);
    checkSum ^= XorShort(kp_gamma_16);
    checkSum ^= XorShort(kd_gamma_16);

    // Send off bytes
    odrive.Stx = 0xAAU;                                                         // send start byte 0xAA
    odrive.Length=14U;                                                          // payload length
    odrive.CRC8='S';                                                            // dual current command
    odrive.DataV[0U]=sp_theta_16;
    odrive.DataV[1U]=kp_theta_16;
    odrive.DataV[2U]=kd_theta_16;
    odrive.DataV[3U]=sp_gamma_16;
    odrive.DataV[4U]=kp_gamma_16;
    odrive.DataV[5U]=kd_gamma_16;
    odrive.checksum=checkSum;

    memcpy(sndBuf, &odrive, sizeof(odrive));
    memset(sndBuf+(odrive.Length+1U),(void *) '\0',sizeof(char));
}

/**
* GetGamma set coupled position
* @param L   float32_t: leg parameter
* @param theta    float32_t: theta
* @param sp_gamma   float32_t: gamma
*/
void GetGamma(float32_t L, float32_t theta, float32_t gamma) 
{
    float32_t L1 = 0.09f;                                                       // upper leg length (m)

    float32_t L2 = 0.162f;                                                      // lower leg length (m)

    float32_t cos_param = (pow(L1,2.0f) + pow(L,2.0f) - pow(L2,2.0f)) / (2.0f*L1*L);

    if (cos_param < -1.0f)
    {
        gamma = PI;
    } 
    else if (cos_param > 1.0f)
    {
        gamma = 0f;
    }
    else
    {
        gamma = acos(cos_param);
    }

}

/**
* Converts the leg params L, gamma to cartesian coordinates x, y (in m)
* Set x_direction to 1.0 or -1.0 to change which direction the leg walks
*/
void LegParamsToCartesian(float32_t L, float32_t theta, float32_t leg_direction, float32_t x, float32_t y)
{
    x = leg_direction * L * cos(theta);
    y = L * sin(theta);
}

/**
* Converts the cartesian coords x, y (m) to leg params L (m), theta (rad)
*/
void CartesianToLegParams(float32_t x, float32_t y, float32_t leg_direction, float32_t L, float32_t theta)
{
    L = pow((pow(x,2.0f) + pow(y,2.0f)), 0.5f);
    theta = atan2(leg_direction * x, y);
}
/*-----------------------------------------------------------------------------
 *      CartesianToThetaGamma:  cartesian conversion to theta gama
 *
 *  Parameters: float32_t x, float32_t y, float32_t leg_direction, float32_t theta, 
 *              float32_t gamma
 *
 *  Return: none
 *----------------------------------------------------------------------------*/
void CartesianToThetaGamma(float32_t x, float32_t y, float32_t leg_direction, float32_t theta, float32_t gamma)
{
    float32_t L = 0.0f;

    CartesianToLegParams(x, y, leg_direction, L, theta);
    GetGamma(L, theta, gamma);
}

/*-----------------------------------------------------------------------------
 *      SinTrajectory:  Sinusoidal trajectory generator function with flexibility 
 *                      from parameters described below. Can do 4-beat, 2-beat, 
 *                      trotting, etc with this.
 *
 *  Parameters: float32_t t, const GaitParams params, float32_t gaitOffset, float32_t x, 
 *              float32_t y
 *
 *  Return: none
 *----------------------------------------------------------------------------*/
void SinTrajectory(float32_t t, const GaitParams params, float32_t gaitOffset, float32_t x, float32_t y)
{
    static float32_t p = 0.0f;
    static float32_t prev_t = 0.0f;
    float32_t gp;

    float32_t stanceHeight = params.stance_height;
    float32_t downAMP = params.down_amp;
    float32_t upAMP = params.up_amp;
    float32_t flightPercent = params.flight_percent;
    float32_t stepLength = params.step_length;
    float32_t FREQ = params.freq;
    float32_t powof;
    float32_t percentBack;

    p += FREQ * (t - prev_t < 0.5f ? t - prev_t : 0.0f);                            // should reduce the lurching when starting a new gait
    prev_t = t;

    //float gp = fmod((p+gaitOffset),1.0);                                      // mod(a,m) returns remainder division of a by m
    powof=1.0f;
    gp = modf((p+gaitOffset),&powof);
    
    if (gp <= flightPercent) 
    {
        x = (gp/flightPercent)*stepLength - stepLength/2.0f;
        y = -upAMP*sin(PI*gp/flightPercent) + stanceHeight;
    }
    else 
    {
        percentBack = (gp-flightPercent)/(1.0f-flightPercent);
        x = -percentBack*stepLength + stepLength/2.0f;
        y = downAMP*sin(PI*percentBack) + stanceHeight;
    }
}
/*-----------------------------------------------------------------------------
 *      IsValidGaitParams:  checks for validity of gain parameters passed
 *
 *  Parameters: const GaitParams params
 *
 *  Return: true when they are valid
 *----------------------------------------------------------------------------*/
uint8_t IsValidGaitParams(const GaitParams params)
{

    const float32_t maxL = 0.25f;
    const float32_t minL = 0.08f;

    float32_t stanceHeight = params.stance_height;
    float32_t downAMP = params.down_amp;
    float32_t upAMP = params.up_amp;
    float32_t flightPercent = params.flight_percent;
    float32_t stepLength = params.step_length;
    float32_t FREQ = params.freq;

    if (stanceHeight + downAMP > maxL || sqrt(pow(stanceHeight, 2) + pow(stepLength / 2.0, 2)) > maxL) 
    {
        return false;
    }
    if (stanceHeight - upAMP < minL) 
    {
        return false;
    }
    if (flightPercent <= 0.0f || flightPercent > 1.0f)
    {
        return false;
    }
    if (FREQ < 0.0f)
    {
        return false;
    }
    if (FREQ > 10.0f)
    {
        return false;
    }
    return true;
}
/*-----------------------------------------------------------------------------
 *      IsValidLegGain:  checks for validity of gain parameters passed
 *
 *  Parameters: const LegGain gains
 *
 *  Return: true when they are valid
 *----------------------------------------------------------------------------*/
uint8_t IsValidLegGain(const LegGain gains)
{
    uint8_t bad =  gains.kp_theta < 0.0f || gains.kd_theta < 0.0f ||                  // check for unstable gains
                gains.kp_gamma < 0.0f || gains.kd_gamma < 0.0f;

    if (bad) {
        return false;
    }

    bad = bad || gains.kp_theta > 320.0f || gains.kd_theta > 10.0f ||                 // check for instability / sensor noise amplification
                 gains.kp_gamma > 320.0f || gains.kd_gamma > 10.0f;

    if (bad) {                                                                  // too high
        return false;
    }

    bad = bad || (gains.kp_theta > 200.0f && gains.kd_theta < 0.1f);                // check for underdamping -> instability
    bad = bad || (gains.kp_gamma > 200.0f && gains.kd_gamma < 0.1f);

    if (bad) {                                                                  // underdamped
        return false;
    }
    return true;
}
/*-----------------------------------------------------------------------------
 *      CommandAllLegs:  checks for validity of gain parameters passed
 *
 *  Parameters: float32_t theta, float32_t gamma,const LegGain *gains, COMGS_Odrive_t *od1, 
 *              COMGS_Odrive_t *od2, COMGS_Odrive_t *od3, COMGS_Odrive_t *od4
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void CommandAllLegs(float32_t theta, float32_t gamma,const LegGain *gains, COMGS_Odrive_t *od1, COMGS_Odrive_t *od2, COMGS_Odrive_t *od3, COMGS_Odrive_t *od4)
{

    char * odriveMsg = NULL;
    //odrv0Interface.SetCoupledPosition(theta, gamma, gains);
    //ODRIVE_API_SetPositionStr(odriveMsg, 0U, theta, gamma, 0U )                 // for motor number 0
    SetCoupledPosition(theta, gamma, gains, odriveMsg);
    UART1_Write_Text(odriveMsg);                                                // write the message to port 1
    //odrv1Interface.SetCoupledPosition(theta, gamma, gains);
    //ODRIVE_API_SetPositionStr(odriveMsg, 1U, theta, gamma, 0U )                 // for motor number 1
    SetCoupledPosition(theta, gamma, gains, odriveMsg);
    UART1_Write_Text(odriveMsg);                                                // write the message to port 2
    //odrv2Interface.SetCoupledPosition(theta, gamma, gains);
    //ODRIVE_API_SetPositionStr(odriveMsg, 2U, theta, gamma, 0U )                 // for motor number 2
    SetCoupledPosition(theta, gamma, gains, odriveMsg);
    UART1_Write_Text(odriveMsg);                                                // write the message to port 3
    //odrv3Interface.SetCoupledPosition(theta, gamma, gains);
    //ODRIVE_API_SetPositionStr(odriveMsg, 3U, theta, gamma, 0U )                 // for motor number 3
    SetCoupledPosition(theta, gamma, gains, odriveMsg);
    UART1_Write_Text(odriveMsg);                                                // write the message to port 4

    od1->sp_theta = theta;
    od1->sp_gamma = gamma;
    od2->sp_theta = theta;
    od2->sp_gamma = gamma;
    od3->sp_theta = theta;
    od3->sp_gamma = gamma;
    od4->sp_theta = theta;
    od4->sp_gamma = gamma;
}
/*-----------------------------------------------------------------------------
 *      UpdateStateGaitParams:  update state gain params
 *
 *  Parameters: const RobStates curr_state
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void UpdateStateGaitParams(const RobStates curr_state)
{
    
    if (!IsaNan(state_gait_params[RobSTOP].stance_height)) 
    {
        state_gait_params[curr_state].stance_height = state_gait_params[RobSTOP].stance_height;
        state_gait_params[RobSTOP].stance_height = NaN;
    }

    if (!IsaNan(state_gait_params[RobSTOP].down_amp)) 
    {
        state_gait_params[curr_state].down_amp = state_gait_params[RobSTOP].down_amp;
        state_gait_params[RobSTOP].down_amp = NaN;
    }

    if (!IsaNan(state_gait_params[RobSTOP].up_amp)) 
    {
        state_gait_params[curr_state].up_amp = state_gait_params[RobSTOP].up_amp;
        state_gait_params[RobSTOP].up_amp = NaN;
    }

    if (!IsaNan(state_gait_params[RobSTOP].flight_percent)) 
    {
        state_gait_params[curr_state].flight_percent = state_gait_params[RobSTOP].flight_percent;
        state_gait_params[RobSTOP].flight_percent = NaN;
    }

    if (!IsaNan(state_gait_params[RobSTOP].step_length)) 
    {
        state_gait_params[curr_state].step_length = state_gait_params[RobSTOP].step_length;
        state_gait_params[RobSTOP].step_length = NaN;
    }

    if (!IsaNan(state_gait_params[RobSTOP].freq)) 
    {
        state_gait_params[curr_state].freq = state_gait_params[RobSTOP].freq;
        state_gait_params[RobSTOP].freq = NaN;
    }

    if (!IsaNan(state_gait_params[RobSTOP].step_diff)) 
    {
        state_gait_params[curr_state].step_diff = state_gait_params[RobSTOP].step_diff;
        state_gait_params[RobSTOP].step_diff = NaN;
    }

}
/*-----------------------------------------------------------------------------
 *      CoupledMoveLeg:  coupled move of leg joint
 *
 *  Parameters: float32_t t,const GaitParams params, float32_t gait_offset, 
 *              float32_t leg_direction,const LegGain *gains, float32_t theta, 
 *              float32_t gamma
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void CoupledMoveLeg( float32_t t,const GaitParams params, float32_t gait_offset, float32_t leg_direction,const LegGain *gains, float32_t theta, float32_t gamma)
{                                                                               //  ODriveArduino& odrive,  interface missing here
    float32_t x=0.0f;                                                                  // float x for leg 0 to be set by the sin trajectory
    float32_t y=0.0f;
    char * Xbuf = NULL;                                                         // the buffer you are sending through the serial port connected

    SinTrajectory(t, params, gait_offset, x, y);
    CartesianToThetaGamma(x, y, leg_direction, theta, gamma);
   // odrive.SetCoupledPosition(theta, gamma, gains);                           // This is arduino   comms to odrive pic is below
   // ODRIVE_API_SetPositionStr(Xbuf, 0U, theta, gamma, 0U );
   // UART1_write_text(Xbuf);
   SetCoupledPosition( theta, gamma, gains, Xbuf );
   UART1_Write_Text(Xbuf);
}
/*-----------------------------------------------------------------------------
 *      gait:  move the legs as desired
 *
 *  Parameters: const GaitParams params, float32_t leg0_offset, float32_t leg1_offset, 
 *              float32_t leg2_offset, float32_t leg3_offset,const LegGain *gains, 
 *              COMGS_Odrive_t *od1, COMGS_Odrive_t *od2, COMGS_Odrive_t *od3,
 *              COMGS_Odrive_t *od4
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void gait(const GaitParams params, float32_t leg0_offset, float32_t leg1_offset, float32_t leg2_offset, float32_t leg3_offset,const LegGain *gains, COMGS_Odrive_t *od1, COMGS_Odrive_t *od2, COMGS_Odrive_t *od3, COMGS_Odrive_t *od4)
{

    GaitParams paramsR;
    GaitParams paramsL;
    float32_t t;
    float32_t leg0_direction, leg1_direction, leg2_direction, leg3_direction;

    paramsR.step_length -= params.step_diff;
    paramsL.step_length += params.step_diff;

    if (!IsValidGaitParams(paramsR) || !IsValidGaitParams(paramsL) || !IsValidLegGain(*gains)) {
        return;
    }

    t = ((float32_t) g_timer5_counter)/100.0f;

    leg0_direction = -1.0f;
    // Drive the odrive motor with the parameters
    //
    CoupledMoveLeg( t, paramsL, leg0_offset, leg0_direction, gains,  od1->sp_theta, od1->sp_gamma);

    leg1_direction = -1.0f;
    CoupledMoveLeg( t, paramsL, leg1_offset, leg1_direction, gains, od2->sp_theta, od2->sp_gamma);

    leg2_direction = 1.0f;
    CoupledMoveLeg(t, paramsR, leg2_offset, leg2_direction, gains, od3->sp_theta, od3->sp_gamma);

    leg3_direction = 1.0f;
    CoupledMoveLeg( t, paramsR, leg3_offset, leg3_direction, gains, od4->sp_theta, od4->sp_gamma);

}

/*-----------------------------------------------------------------------------
 *      TransitionToDance:  example of how to drive state engine
 *
 *  Parameters: none
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void TransitionToDance() 
{
    LegGain gait_gains;
    RobStates state;
    
    gait_gains.kp_theta = 50.0f;
    gait_gains.kd_theta = 0.5f;
    gait_gains.kp_gamma = 30.0f;
    gait_gains.kd_gamma = 0.5f;

    state = RobDANCE;
    UpdateStateGaitParams(state);
}
/*-----------------------------------------------------------------------------
 *      hop:  example of how to drive state engine to hopping using 4 odrives
 *
 *  Parameters: const GaitParams params, COMGS_Odrive_t *od1, COMGS_Odrive_t *od2, 
 *              COMGS_Odrive_t *od3, COMGS_Odrive_t *od4
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void hop(const GaitParams params, COMGS_Odrive_t *od1, COMGS_Odrive_t *od2, COMGS_Odrive_t *od3, COMGS_Odrive_t *od4)
{
    float32_t freq = params.freq;
    LegGain hop_gains;
    LegGain land_gains;
    float32_t theta=0.0f, gamma=0.0f;                                           // theta and gamma are calculated but this initialises to stop warning

    hop_gains.kp_theta = 120.0f;                                                // kp_theta
    hop_gains.kd_theta = 1.0f;                                                  // kd_theta
    hop_gains.kp_gamma = 80.0f;                                                 // kp_gamma
    hop_gains.kd_gamma = 1.0f;                                                  // kd_gamma

    land_gains.kp_theta = 120.0f;                                               // kp_theta
    land_gains.kd_theta = 2.0f;                                                 // kd_theta
    land_gains.kp_gamma = 20.0f;                                                // kp_gamma
    land_gains.kd_gamma = 2.0f;                                                 // kd_gamma

    CartesianToThetaGamma(0.0f, params.stance_height - params.up_amp, 1.0f, theta, gamma);
    CommandAllLegs(theta, gamma, &land_gains, od1, od2, od3, od4);
    Delay_ms(50U);

    CartesianToThetaGamma(0.0f, params.stance_height + params.down_amp, 1.0f, theta, gamma);
    CommandAllLegs(theta, gamma, &hop_gains, od1, od2, od3, od4);
    Delay_ms(50U);

    CartesianToThetaGamma(0.0f, params.stance_height, 1.0f, theta, gamma);
    CommandAllLegs(theta, gamma, &land_gains, od1, od2, od3, od4);
    Delay_ms(50U);
}
/* kinematics functions
   port from python https://github.com/theodor1289/sphere-follower-robot
                                                                             */
/*-----------------------------------------------------------------------------
 *      precomputed_jacobian:  used in kinematic movement
 *
 *  Parameters: float32_t joints[4u], float32_t jacobian[3u][4u]
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void precomputed_jacobian( float32_t joints[4u], float32_t *jacobian[3u][4u] )
{

    float32_t s1,c1,s2,c2,s3,c3,s4,c4;

    s1 = sin(joints[0u]);
    c1 = cos(joints[0u]);
    s2 = sin(joints[1u]);
    c2 = cos(joints[1u]);
    s3 = sin(joints[2u]);
    c3 = cos(joints[2u]);
    s4 = sin(joints[3u]);
    c4 = cos(joints[3u]);

    //jacobian = np.zeros((3, 4))

    *jacobian[0u][0u] = 2.0f * c1 * s2 * c3 * c4 - 2.0f * s1 * s3 * c4 + 2.0f * c1 * c2 * s4 + 3.0f * c1 * s2 * c3 - 3.0f * s1 * s3;
    *jacobian[0u][1u] = 2.0f * s1 * c2 * c3 * c4 - 2.0f * s1 * s2 * s4 + 3.0f * s1 * c2 * c3;
    *jacobian[0u][2u] = - 2.0f * s1 * s2 * s3 * c4 + 2.0f * c1 * c3 * c4 - 3.0f * s1 * s2 * s3 + 3.0f * c1 * c3;
    *jacobian[0u][3u] = - 2.0f * s1 * s2 * c3 * s4 - 2.0f * c1 * s3 * s4 + 2.0f * s1 * c2 * c4;
    *jacobian[1u][0u] = 2.0f * s1 * s2 * c3 * c4 + 2.0f * c1 * s3 * c4 + 2.0f * s1 * c2 * s4 + 3.0f * s1 * s2 * c3 + 3.0f * c1 * s3;
    *jacobian[1u][1u] = - 2.0f * c1 * c2 * c3 * c4 + 2.0f * c1 * s2 * s4 - 3.0f * c1 * c2 * c3;
    *jacobian[1u][2u] = 2.0f * c1 * s2 * s3 * c4 + 2.0f * s1 * c3 * c4 + 3.0f * c1 * s2 * s3 + 3.0f * s1 * c3;
    *jacobian[1u][3u] = 2.0f * c1 * s2 * c3 * s4 - 2.0f * s1 * s3 * s4 - 2.0f * c1 * c2 * c4;

    *jacobian[2u][0u] = 0.0f;
    *jacobian[2u][1u] = - 2.0f * s2 * c3 * c4 - 2.0f * c2 * s4 - 3.0f * s2 * c3;
    *jacobian[2u][2u] = - 2.0f * c2 * s3 * c4 - 3.0f * c2 * s3;
    *jacobian[2u][3u] = - 2.0f * c2 * c3 * s4 - 2.0f * s2 * c4;

}
/*-----------------------------------------------------------------------------
 *      transform_matrix:  matrix transform
 *
 *  Parameters: float32_t alpha,float32_t a, float32_t d, float32_t theta, 
 *              float32_t *array[4u][4u]
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void transform_matrix(float32_t alpha,float32_t a, float32_t d, float32_t theta, float32_t array[4u][4u])
{
        array[0u][0u] = cos(theta);
        array[1u][0u] = -sin(theta) * cos(alpha);
        array[2u][0u] = sin(theta) * sin(alpha);
        array[3u][0u] = a * cos(theta);

        array[0u][1u] = sin(theta);
        array[1u][1u] = cos(theta) * cos(alpha);
        array[2u][1u] = -cos(theta) * sin(alpha);
        array[3u][1u] = a * sin(theta);

        array[0u][2u] = 0.0f;
        array[1u][2u] = sin(alpha);
        array[2u][2u] = cos(alpha);
        array[3u][2u] = d;

        array[0u][3u] = 0.0f;
        array[1u][3u] = 0.0f;
        array[2u][3u] = 0.0f;
        array[3u][3u] = 1.0f;
}
/*-----------------------------------------------------------------------------
 *      automatic_forward_kinematics:  automatic forward kinematics
 *
 *  Parameters: float32_t joints[4u], float32_t *t10[4u][4u], float32_t *t21[4u][4u],
 *              float32_t *t32[4u][4u], float32_t *t43[4u][4u]
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void automatic_forward_kinematics(float32_t joints[4u], float32_t t10[4u][4u], float32_t t21[4u][4u], float32_t t32[4u][4u], float32_t t43[4u][4u]  )
{
    float32_t t10[4u][4u];
    float32_t t21[4u][4u];
    
    transform_matrix(PI / 2.0f, 0.0f, 2.0f, (joints[0u] + PI) / 2.0f, t10);                  // we add pi/2 to make 0, 0, 0, 0 an
    transform_matrix(PI / 2.0f, 0.0f, 0.0f, (joints[1u] + PI) / 2.0f, t21);                  // we add pi/2 to make 0, 0, 0, 0 an
    transform_matrix(-PI / 2.0f, 3.0f, 0.0f, joints[2u], t32);
    transform_matrix(0.0f, 2.0f, 0.0f, joints[3u], t43);
}
/*-----------------------------------------------------------------------------
 *      manual_forward_kinematics:  manual forward kinematics
 *
 *  Parameters: float32_t joints[4u], float32_t *array[3u]
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void manual_forward_kinematics( float32_t joints[4u], float32_t *array[3u] )
{
    float32_t s1,s2,s3,s4,c1,c2,c3,c4;
    float32_t x_e,y_e,z_e;
    
    s1 = sin(joints[0u]);
    c1 = cos(joints[0u]);
    s2 = sin(joints[1u]);
    c2 = cos(joints[1u]);
    s3 = sin(joints[2u]);
    c3 = cos(joints[2u]);
    s4 = sin(joints[3u]);
    c4 = cos(joints[3u]);

    x_e = 2.0f * s1 * s2 * c3 * c4 + 2.0f * c1 * s3 * c4 + 2.0f * s1 * c2 * s4 + 3.0f * s1 * s2 * c3 + 3.0f * c1 * s3;
    y_e = - 2.0f * c1 * s2 * c3 * c4 + 2.0f * s1 * s3 * c4 - 2.0f * c1 * c2 * s4 - 3.0f * c1 * s2 * c3 + 3.0f * s1 * s3;
    z_e = 2.0f * c2 * c3 * c4 - 2.0f * s2 * s4 + 3.0f * c2 * c3 + 2.0f;

    *array[0u]=x_e;
    *array[1u]=y_e;
    *array[2u]=z_e;
}

#ifdef NEED_2_PORT
//  ---------- This needs the gsl libraries and we need to port this
/**
 * Compute the (Moore-Penrose) pseudo-inverse of a matrix.
 *
 * If the singular value decomposition (SVD) of A = USV? then the pseudoinverse A?¹ = VS?¹U?, where ? indicates transpose and S?¹ is obtained by taking the reciprocal of each nonzero element on the diagonal, leaving zeros in place. Elements on the diagonal smaller than ``rcond`` times the largest singular value are considered zero.
 *
 * @parameter A                Input matrix. **WARNING**: the input matrix ``A`` is destroyed. However, it is still the responsibility of the caller to free it.
 * @parameter rcond                A real number specifying the singular value threshold for inclusion. NumPy default for ``rcond`` is 1E-15.
 *
 * @returns A_pinv                Matrix containing the result. ``A_pinv`` is allocated in this function and it is the responsibility of the caller to free it.
**/
gsl_matrix* moore_penrose_pinv(gsl_matrix *A, const realtype rcond) 
{
        gsl_matrix *V, *Sigma_pinv, *U, *A_pinv;
        gsl_matrix *_tmp_mat = NULL;
        gsl_vector *_tmp_vec;
        gsl_vector *u;
        realtype x, cutoff;
        size_t i, j;
        unsigned int n = A->size1;
        unsigned int m = A->size2;
        bool was_swapped = false;

        if (m > n) {
                /* libgsl SVD can only handle the case m <= n - transpose matrix */
                was_swapped = true;
                _tmp_mat = gsl_matrix_alloc(m, n);
                gsl_matrix_transpose_memcpy(_tmp_mat, A);
                A = _tmp_mat;
                i = m;
                m = n;
                n = i;
        }

        /* do SVD */
        V = gsl_matrix_alloc(m, m);
        u = gsl_vector_alloc(m);
        _tmp_vec = gsl_vector_alloc(m);
        gsl_linalg_SV_decomp(A, V, u, _tmp_vec);
        gsl_vector_free(_tmp_vec);

        /* compute S?¹ */
        Sigma_pinv = gsl_matrix_alloc(m, n);
        gsl_matrix_set_zero(Sigma_pinv);
        cutoff = rcond * gsl_vector_max(u);

        for (i = 0; i < m; ++i) {
                if (gsl_vector_get(u, i) > cutoff) {
                        x = 1. / gsl_vector_get(u, i);
                }
                else {
                        x = 0.;
                }
                gsl_matrix_set(Sigma_pinv, i, i, x);
        }

        /* libgsl SVD yields "thin" SVD - pad to full matrix by adding zeros */
        U = gsl_matrix_alloc(n, n);
        gsl_matrix_set_zero(U);

        for (i = 0; i < n; ++i) {
                for (j = 0; j < m; ++j) {
                        gsl_matrix_set(U, i, j, gsl_matrix_get(A, i, j));
                }
        }

        if (_tmp_mat != NULL) {
                gsl_matrix_free(_tmp_mat);
        }
        /* two dot products to obtain pseudoinverse */
        _tmp_mat = gsl_matrix_alloc(m, n);
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1., V, Sigma_pinv, 0., _tmp_mat);

        if (was_swapped) {
                A_pinv = gsl_matrix_alloc(n, m);
                gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1., U, _tmp_mat, 0., A_pinv);
        }
        else {
                A_pinv = gsl_matrix_alloc(m, n);
                gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1., _tmp_mat, U, 0., A_pinv);
        }
        gsl_matrix_free(_tmp_mat);
        gsl_matrix_free(U);
        gsl_matrix_free(Sigma_pinv);
        gsl_vector_free(u);
        gsl_matrix_free(V);

        return A_pinv;
}
#endif
#endif                                                                          // End of Robot helper

#ifdef GPS_INCLUDED                                                             // You need a GPS
/*-----------------------------------------------------------------------------
 *      ReadGPSData:  GPS reader for QUECEL or UBLOX reads 2 types of GPS 
 *                    receiver and creeates lat long co-ordinates
 *
 *  Parameters: GPS_Info_t *gpsInfo
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void ReadGPSData( GPS_Info_t *gpsInfo )
{
   //unsigned char txt[768u];                                                   // the string that is filled with GPS receive info via interrupt
   int16_t latitude, longitude, oldlatitude=0, oldlongitude=0;
   unsigned char *string;
   unsigned char lat_str[8u];
   unsigned char long_str[9u];

   string = strstr(GPStxt,"$GPGLL");                                               // If txt array contains "$GPGLL" string we proceed...
   if(string != 0)                                                              // string match was found
   {
      if(string[7u] != ',')                                                     // if "$GPGLL" NMEA message have ',' sign in the 8-th
      {
#if (TYPE_GPS == QUECTEL)                                                       // If QUECTEL GPS receiver module is used :
            latitude = (string[7u]-48u)*10u + (string[8u]-48u);
            longitude = (string[19u]-48u)*100u + (string[20u]-48u)*10u + (string[21u]-48u);
            if(string[17u] == 'S')                                               // if the latitude is in the South direction it has minus sign
            {
              latitude = 0 - latitude;
            }

            if(string[30u] == 'W')                                              // if the longitude is in the West direction it has minus sign
            {
              longitude = 0 - longitude;
            }
            gpsInfo->lat= latitude;
            gpsInfo->longd= longitude;
            
            if ((oldlongitude != longitude) | (oldlatitude != latitude))
            {
                 // Display_Cursor(latitude, longitude, &lat_str, &long_str, CL_WHITE);     // Display the cursor on the world map
              lat_str[0u] = string[7u];
              lat_str[1u] = string[8u];
              lat_str[2u] = ',';
              lat_str[3u] = string[9u];
              lat_str[4u] = string[10u];
              lat_str[5u] = ' ';
              lat_str[6u] = string[17u];
              lat_str[7u] = 0;

              long_str[0u] = string[19u];
              long_str[1u] = string[20u];
              long_str[2u] = string[21u];
              long_str[3u] = ',';
              long_str[4u] = string[22u];
              long_str[5u] = string[23u];
              long_str[6u] = ' ';
              long_str[7u] = string[30u];
              long_str[8u] = 0;
            }

#elif (TYPE_GPS == UBLOX)                                                       // If uBlox GPS receiver module is used :

            latitude = (string[7u]-48u)*10u + (string[8u]-48u);
            longitude = (string[20u]-48u)*100u + (string[21u]-48u)*10u + (string[22u]-48u);

            if(string[18u] == 'S')                                               // if the latitude is in the South direction it has minus sign
            {
              latitude = 0 - latitude;
            }

            if(string[32u] == 'W')                                               // if the longitude is in the West direction it has minus sign
            {
              longitude = 0 - longitude;
            }
            gpsInfo->lat= latitude;
            gpsInfo->longd= longitude;
            
            if ((oldlongitude != longitude) | (oldlatitude != latitude))
            {
              // Display_Cursor(latitude, longitude, &lat_str, &long_str, CL_WHITE);     // Display the cursor on the world map
              lat_str[0u] = string[7u];
              lat_str[1u] = string[8u];
              lat_str[2u] = ',';
              lat_str[3u] = string[9u];
              lat_str[4u] = string[10u];
              lat_str[5u] = ' ';
              lat_str[6u] = string[18u];
              lat_str[7u] = 0u;

              long_str[0u] = string[20u];
              long_str[1u] = string[21u];
              long_str[2u] = string[22u];
              long_str[3u] = ',';
              long_str[4u] = string[23u];
              long_str[5u] = string[24u];
              long_str[6u] = ' ';
              long_str[7u] = string[32u];
              long_str[8u] = 0u;
           }
#endif

            // Display_Cursor(latitude, longitude, &lat_str, &long_str, CL_RED);     // Display the cursor on the world map
      }
   }
}
//
// Scale the GPS reading for a display
//
void Calc_Cursor_LL(int16_t lat, int16_t lon, char *lt_Str, char *lng_Str, uint16_t color)
{

  uint32_t latitude_y, longitude_x;
  // Latitude and Longitude scaling for 320x240 display:
  // Latitude: Input range is -90 to 90 degrees
  // Longitude: Input range is -180 to 180 degrees

  latitude_y = ((160U*(90U - lat))/180U) + 40U;
  longitude_x = (320U*(lon + 180U));
  longitude_x = longitude_x/360U - 10U;

  // Cursor drawing                                                             Do this to suit youre system
  //TFT_Set_Pen(CL_RED, 1);
  //TFT_H_Line(longitude_x - 2, longitude_x + 2, latitude_y );
  //TFT_V_Line( latitude_y - 2, latitude_y + 2, longitude_x );
  //TFT_Set_Font(&TFT_defaultFont, color, FO_HORIZONTAL);
  //TFT_Write_Text(lt_Str, 160, 200);
  //TFT_Write_Text(lng_Str, 160, 215);
}
#endif                                                                          // -----------------   END OF GPS INCLUSION

/* return 1 if string contain only digits, else return 0 */
int16_t valid_digit(unsigned char *ip_str)
{
    while (*ip_str) 
    {
        if (*ip_str >= '0' && *ip_str <= '9')
            ++ip_str;
        else
            return 0;
    }
    return 1;
}

/*-----------------------------------------------------------------------------
 *      is_valid_ip:  check for a valid ip address
 *
 *  Parameters: unsigned char *ip_str
 *
 *  Return: return 1 if IP string is valid, else return 0
 *----------------------------------------------------------------------------*/
int16_t is_valid_ip(unsigned char *ip_str)
{
    int16_t  num, dots = 0;
    unsigned char *ptr;

    if (ip_str == NULL)
        return 0;

    ptr = strtok(ip_str, ".");                                                  // http://pubs.opengroup.org/onlinepubs/009695399/functions/strtok_r.html

    if (ptr == NULL)
        return 0;

    while (ptr) 
    {
        if (!valid_digit(ptr))                                                  // after parsing string, it must contain only digits */
            return 0;

        num = atoi(ptr);

        if (num >= 0 && num <= 255)                                             // check for valid IP */
        {
            ptr = strtok(NULL, ".");                                            // parse remaining string delimetered by .
            if (ptr != NULL)
                ++dots;
        } 
        else
           return 0;
    }

    if (dots != 3)                                                              // valid IP string must contain 3 dots */
        return 0;
    return 1;                                                                   // success
}

/*-----------------------------------------------------------------------------
 *      farenhFromcelsius:  convert celcius to farenheit
 *
 *  Parameters: float32_t celsius
 *
 *  Return: converted value as float32_t
 *----------------------------------------------------------------------------*/
float32_t farenhFromcelsius( float32_t celsius )
{
   return((celsius * 9.0f / 5.0f) + 32.0f);
}

#ifdef GPS_POS_SPRAYER
/*-----------------------------------------------------------------------------
 *      doesPosFitSquare:  Does position fit square defined by co-ordinates
 *
 *  Parameters: const geoCoord_Square_t * geoJson, const GPS_Info_t * actPos, 
 *              float32_t sptLookUp[NO_SPRAY_OF_AREAS], pidBlock_t *pidObj
 *
 *  Return: int8_t matching square_id or GPS_GEO_NO_AREA_MATCH
 *----------------------------------------------------------------------------*/
int8_t doesPosFitSquare( const geoCoord_Square_t * geoJson, const GPS_Info_t * actPos, float32_t sptLookUp[NO_SPRAY_OF_AREAS], pidBlock_t *pidObj  )
{
   if ((actPos->GeoidalHt<=(geoJson->GeoidalHt-GPS_POS_HT_DB)) ||  (actPos->GeoidalHt>=(geoJson->GeoidalHt+GPS_POS_HT_DB)))
   {
      pidObj->rem_setpoint= _defSpraySpeed;                                     // set to default speed
      return GPS_GEO_WRONG_HEIGHT;                                              // return we are at the wrong height
   }
   if (((((actPos->longd) <= (geoJson->long1)  && (actPos->longd) >= (geoJson->long3)) &&    // first co-ordinate check
      ((actPos->lat) <= (geoJson->lat4)  && (actPos->lat) >= (geoJson->lat3))) &&          // second co-ordinate check
      ((actPos->longd) <= (geoJson->long2)  && (actPos->longd) >= (geoJson->long4))) &&     // third co-ordinate check
      ((actPos->lat) <= (geoJson->lat2)  && (actPos->lat) >= (geoJson->lat1)))            // fourth co-ordinate check
   {
       pidObj->mode_init==false;                                                // remove pid init
       pidObj->mode_trk==false;                                                 // remove track bit
       pidObj->mode_man=false;                                                  // set to automatic
       if (geoJson->square_id<=sizeof(sptLookUp))                               // within the bounds of the array
          pidObj->rem_setpoint=sptLookUp[geoJson->square_id];                   // set the remote setpoint
       pidObj->mode_rem = true;                                                 // set to remote
       return geoJson->square_id;
   }
   else
   {
      pidObj->rem_setpoint= _defSpraySpeed;                                     // set to default speed
      return GPS_GEO_NO_AREA_MATCH;                                             // return we have no match not in the square
   }
}
/*-----------------------------------------------------------------------------
 *      guiInterfacePID:  Graphical User Interface to sprayer controls PID loop
 *
 *  Parameters: pidBlock_t *pidObj, interfacePIDObj_t *hmiOrPush
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void guiInterfacePID( pidBlock_t *pidObj, interfacePIDObj_t *hmiOrPush )       // call function with the object for every pid sprayer
{
   float32_t changeVal;
   
   switch(hmiOrPush->sysState)
   {
      case SPRAY_SET_ON:
      if (pidObj->mode_man)
        pidObj->mode_man=false;                                                 /* place in automatic mode */
      hmiOrPush = SPRAY_NO_ACTION;                                              // take off the action latch
      break;
      
      case SPRAY_SET_OFF:
      pidObj->mode_man=true;                                                    /* place in manual */
      pidObj->output = 0.0f;                                                    /* switch output to off */
      hmiOrPush = SPRAY_NO_ACTION;                                              // take off the action latch
      break;

      case SPRAY_GO_UP:
      if (pidObj->mode_man)                                                     // manual drive output
      {
         changeVal = pidObj->output + 0.5f;
         if (changeVal >= 100.0f)
         {
            pidObj->output = 100.0f;
         }
         else if (changeVal <= 0.0f)
         {
            pidObj->output = 0.0f;
         }
         else
         {
            pidObj->output = changeVal;                                         // increment by delta
         }
      }
      else if ((!pidObj->mode_man) && (!pidObj->mode_rem))                      // in local drive setpoint
      {
         changeVal = pidObj->setpoint + 0.5f;
         if (changeVal >= 100.0f)
         {
            pidObj->setpoint = 100.0f;                                          // set point max range
         }
         else if (changeVal <= 0.0f)
         {
            pidObj->setpoint = 0.0f;                                            // set point min range
         }
         else
         {
            pidObj->setpoint = changeVal;                                       // increment by delta
         }
      }
      hmiOrPush = SPRAY_NO_ACTION;                                              // take off the action latch
      break;

      case SPRAY_GO_DOWN:
      if (pidObj->mode_man)                                                     // manual drive output
      {
         changeVal = pidObj->output - 0.5f;
         if (changeVal >= 100.0f)
         {
            pidObj->output = 100.0f;
         }
         else if (changeVal <= 0.0f)
         {
            pidObj->output = 0.0f;
         }
         else
         {
            pidObj->output = changeVal;                                         // increment by delta
         }
      }
      else if ((!pidObj->mode_man) && (!pidObj->mode_rem))                      // in local drive setpoint
      {
         changeVal = pidObj->setpoint - 0.5f;
         if (changeVal >= 100.0f)
         {
            pidObj->setpoint = 100.0f;                                          // set point max range
         }
         else if (changeVal <= 0.0f)
         {
            pidObj->setpoint = 0.0f;                                            // set point min range
         }
         else
         {
            pidObj->setpoint = changeVal;                                       // increment by delta
         }
      }
      hmiOrPush = SPRAY_NO_ACTION;                                              // take off the action latch
      break;

      case SPRAY_SET_VAL:
      if (pidObj->mode_man)                                                     // manual drive output
      {
         changeVal = hmiOrPush->pidValue;
         if (changeVal >= 100.0f)
         {
            pidObj->output = 100.0f;
         }
         else if (changeVal <= 0.0f)
         {
            pidObj->output = 0.0f;
         }
         else
         {
            pidObj->output = changeVal;                                         // increment by delta
         }
      }
      else if ((!pidObj->mode_man) && (!pidObj->mode_rem))                      // in local drive setpoint
      {
         changeVal = hmiOrPush->pidValue;
         if (changeVal >= 100.0f)
         {
            pidObj->setpoint = 100.0f;                                          // set point max range
         }
         else if (changeVal <= 0.0f)
         {
            pidObj->setpoint = 0.0f;                                            // set point min range
         }
         else
         {
            pidObj->setpoint = changeVal;                                       // increment by delta
         }
      }
      hmiOrPush = SPRAY_NO_ACTION;                                              // take off the action latch
      break;

      case SPRAY_RESET:                                                         // back to position control
      pidObj->mode_man=false;                                                   // auto
      pidObj->mode_rem=true;                                                    // position control
      pidObj->mode_hld=false;                                                   // no hold
      pidObj->mode_trk=false;                                                   // no track
      pidObj->mode_init=false;                                                  // no intitialise
      pidObj->mode_began=false;                                                 // re-intialise
      hmiOrPush = SPRAY_NO_ACTION;                                              // take off the action latch
      break;
      
      case SPRAY_NO_ACTION:
      break;
      
      default:
      break;
   }
}
#endif
/*-----------------------------------------------------------------------------
 *      compute_time:  compute days hours minutes seconds from overall seconds time
 *
 *  Parameters: uint32_t time, COMGS_DateTime_t *hmiTm
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/

void compute_time(uint32_t time, COMGS_DateTime_t *hmiTm)                       // compute days hours minutes seconds from overall seconds time
{
 uint32_t days, hours, minutes, seconds;
 uint32_t timeTemp=0LU;
 
 days = time / (24L * 3600L);
 time -= days * 24L * 3600L;

 hours = time / 3600L;                                                         // time now contains the number of seconds in the last day */
 timeTemp = time - (hours * 3600L);

 minutes = timeTemp / 60L;                                                      // time now contains the number of seconds in the last hour */
 seconds = timeTemp - minutes * 60L;
 
 hmiTm->Day = days;                                                             // Set the values on the HMI
 hmiTm->Hour = hours;
 hmiTm->Minute = minutes;
 hmiTm->Second = seconds;
 
}
/*-----------------------------------------------------------------------------
 *      compute_secs:  compute overall seconds from a time of H:M:S on the HMI
 *
 *  Parameters: XS_Hmi_Slider_t *CalcTime, const COMGS_DateTime_t *hmiTm
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void compute_secs(XS_Hmi_Slider_t *CalcTime, const COMGS_DateTime_t *hmiTm)     // compute overall seconds from a time of H:M:S on the HMI
{

 CalcTime->value = hmiTm->Second + (hmiTm->Minute * 60U) + (hmiTm->Hour * 60U * 24U);

}
/*-----------------------------------------------------------------------------
 *      compute_date:  compute a 32 bit integer from the date
 *
 *  Parameters: XS_Hmi_Slider_t *CalcTime, const COMGS_DateTime_t *hmiTm
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void compute_date(XS_Hmi_Slider_t *CalcTime, const COMGS_DateTime_t *hmiTm)     // compute a 32 bit integer from the date
{

 CalcTime->value = hmiTm->Year + (hmiTm->Month << 17u) + (hmiTm->Day << 22u);      // make a 32 bit word that detects a change in date

}
/*-----------------------------------------------------------------------------
 *      check_datetime:  check for valid date and time fields entered on the HMI
 *
 *  Parameters: COMGS_DateTime_t *hmiTm
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
uint8_t check_datetime(COMGS_DateTime_t *hmiTm)                                 // check for valid date and time fields entered on the HMI
{

 if (hmiTm->Year <= 2019u)
 {
    if ((hmiTm->Year >= 0u) && (hmiTm->Year <= 99u))                              // Did he miss the 2000 part and only do the last 2 figures ?
       hmiTm->Year += 2000u;                                                     // Add the 2000
    else
       return 0u;
 }
 else if (hmiTm->Year >= 9999u)
 {
    return 0u;                                                                   // Invalid Year
 }
 
 if ((hmiTm->Month >= 1u) && (hmiTm->Month <= 12u))                               // Valid month found then check for a valid date combination
 {
    if (!((hmiTm->Day>=1u && hmiTm->Day<=31u) && (hmiTm->Month==1u || hmiTm->Month==3u || hmiTm->Month==5u || hmiTm->Month==7u || hmiTm->Month==8u || hmiTm->Month==10u || hmiTm->Month==12u)))
    {
       if (!((hmiTm->Day>=1u && hmiTm->Day<=30u) && (hmiTm->Month==4u || hmiTm->Month==6u || hmiTm->Month==9u || hmiTm->Month==11u)))
       {
          if (!((hmiTm->Day>=1u && hmiTm->Day<=30u) && (hmiTm->Month==4u || hmiTm->Month==6u || hmiTm->Month==9u || hmiTm->Month==11u)))
          {
            if (!((hmiTm->Day>=1u && hmiTm->Day<=28u) && (hmiTm->Month==2u)))
            {
              if (!(hmiTm->Day==29u && hmiTm->Month==2u && (hmiTm->Year%400u==0u ||(hmiTm->Year%4u==0u && hmiTm->Year%100u!=0u))))    // 29 of feb on a leap year
              {
                 return 0u;                                                      // Invalid day month combination entered
              }
            }
          }
       }
    }
 }
 else
 {
    return 0u;                                                                   // Invalid month
 }

 if (!((hmiTm->Hour >= 0U) && (hmiTm->Hour <= 24U)))
    return 0u;                                                                   // Invalid hour
 if (!((hmiTm->Minute >= 0U) && (hmiTm->Minute <= 59U)))
    return 0u;                                                                   // Invalid minute
 if (!((hmiTm->Second >= 0) && (hmiTm->Second <= 59U)))
    return 0u;                                                                   // Invalid Second

 return 1u;                                                                      // All were valid return success
 
}
/*-----------------------------------------------------------------------------
 *      dateTimeToRtcTime:  Return the unix time (seconds since 1-1-1970) 
 *                          can be used as aa single uint32 for time or for EGD protocol
 *
 *  Parameters: COMGS_DateTime_t *dt
 *
 *  Return: uint32_t seconds since 1-1-1970 Unix time
 *----------------------------------------------------------------------------*/
uint32_t dateTimeToRtcTime(COMGS_DateTime_t *dt)                                // Return the unix time (seconds since 1-1-1970) can be used as aa single uint32 for time or for EGD protocol
{
    uint16_t second = dt->Second;                                               // 0-59
    uint16_t minute = dt->Minute;                                               // 0-59
    uint16_t hour = dt->Hour;                                                   // 0-23
    uint16_t day = dt->Day - 1U;                                                // 0-30
    uint16_t month = dt->Month - 1U;                                            // 0-11
    uint16_t year;                                                              // Year 4 digit (convert below or return a zero on error)
    
    if (dt->Year <= 2019U)                                                      // The year is invalid from now
    {
       if ((dt->Year >= 0U) && (dt->Year <= 99U))                               // Did he miss the 2000 part and only do the last 2 figures ?
          dt->Year += 2000U;                                                    // Add the 2000
       else
          return 0U;                                                            // Invalid year given
    }
    else if (dt->Year >= 9999U)
    {
       return 0U;                                                               // Invalid Year
    }

    year = dt->Year - UNIX_REFERENCE_YEAR;                                      // now subtract 1970 the reference year

    return ((((year / 4U * (365U * 4U + 1U) + Udays[year % 4U][month] + day) * 24U + hour) * 60U + minute) * 60U + second);
}
/*-----------------------------------------------------------------------------
 *      bswap_16:  byte swap 16 bit
 *
 *  Parameters: uint16_t x
 *
 *  Return: uint16_t as opposite endian value
 *----------------------------------------------------------------------------*/
uint16_t bswap_16(uint16_t x)                                                   // byte swaps 16 bit
{
    return (x >> 8u) | (x << 8u);
}
/*-----------------------------------------------------------------------------
 *      bswap_32:  byte swap 32 bit
 *
 *  Parameters: uint32_t x
 *
 *  Return: uint32_t as opposite endian value
 *----------------------------------------------------------------------------*/
uint32_t bswap_32(uint32_t num2ByteSwap)                                        // byte swaps 32 bit
{
    return (bswap_16(num2ByteSwap & 0xffffL) << 16L) | (bswap_16(num2ByteSwap >> 16L));
}


// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
/*-----------------------------------------------------------------------------
 *      fastA2F:  ascii to float function low footprint
 *
 *  Parameters: const char *p
 *
 *  Return: float32_t value of the string passed if possible
 *----------------------------------------------------------------------------*/
float32_t fastA2F(const char *p)
{
    int16_t frac = 0;
    float32_t sign, value, scale;
    uint16_t expon;
    float32_t pow10;

    while (white_space(*p))                                                     // Skip leading white space, if any.
    {
        p += 1;
    }

    sign = 1.0f;                                                                // Get sign, if any.

    if (*p == '-') 
    {
        sign = -1.0f;
        p += 1;
    }
    else if (*p == '+')
    {
        p += 1;
    }
    value = 0.0f;                                                               // Get digits before decimal point or exponent, if any.

    while (valid_digit2(*p))
    {
        value = value * 10.0f + (*p - '0');
        p += 1;
    }

    if (*p == '.')                                                              // Get digits after decimal point, if any.
    {
        pow10 = 10.0f;
        p += 1;

        while (valid_digit2(*p))
        {
            value += (*p - '0') / pow10;
            pow10 *= 10.0f;
            p += 1;
        }
    }

    scale = 1.0f;                                                               // Handle exponent, if any.
    if ((*p == 'e') || (*p == 'E')) 
    {
        p += 1;
        frac = 0;                                                               // Get sign of exponent, if any.

        if (*p == '-') 
        {
            frac = 1;
            p += 1;
        } 
        else if (*p == '+') 
        {
            p += 1;
        }

        expon = 0;                                                              // Get digits of exponent, if any.

        while (valid_digit2(*p))
        {
            expon = expon * 10u + (*p - '0');
            p += 1;
        }

        if (expon > 308U)
            expon = 308U;

        while (expon >= 8U)                                                      // Calculate scaling factor.
        {
            scale *= 1E8f;
            expon -= 8U;
        }

        while (expon > 0U)
        {
            scale *= 10.0f;
            expon -= 1U;
        }
    }
    return sign * (frac ? (value / scale) : (value * scale));                   // Return signed and scaled floating point result.
}
/*-----------------------------------------------------------------------------
 *      fastA2UL:  ascii to u32 function low footprint
 *
 *  Parameters: const char *p
 *
 *  Return: uint32_t value of the string passed if possible
 *----------------------------------------------------------------------------*/
uint32_t fastA2UL(const char *p)
{
    uint32_t result = 0U;
    unsigned char digit;

    while (white_space(*p)) 
    {
        p += 1;
    }

    for ( ; ; p++) 
    {
        digit = *p - '0';

        if (digit > 9U)
        {
            break;
        }
        result *= 10U;
        result += digit;
    }
    return result;
}
/*-----------------------------------------------------------------------------
 *      a2d:  ascii to decimal
 *
 *  Parameters: char ch
 *
 *  Return: int16_t decimal of char passed to function
 *----------------------------------------------------------------------------*/
int16_t a2d(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    else if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    else if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    else
        return -1;
}
/*-----------------------------------------------------------------------------
 *      fastA2I:  ascii to signed integer 16 bit number
 *
 *  Parameters: const char *s
 *
 *  Return: int16_t decimal of string passed to function
 *----------------------------------------------------------------------------*/
int16_t fastA2I(const char *s)
{
    int16_t sign = 1;
    int16_t num = 0;
    int16_t digit;

    while (white_space(*s)) 
    {
        s++;
    }

    if (*s == '-') 
    {
        sign = -1;
        s++;
    }

    while ((digit = a2d(*s)) >= 0) 
    {
        if (digit > 10U)
            break;

        num = num * 10U + digit;
        s++;
    }
    return sign * num;
}

#if defined(USE_MAV_PULSE_OBJECT)
/*-----------------------------------------------------------------------------
 *      relayPulseAction:  relay slow pulse output action
 *
 *  Parameters: pulse_obj_t *pulsObj
 *
 *  Return: none
 *----------------------------------------------------------------------------*/
void relayPulseAction( pulse_obj_t *pulsObj )                                   /* Function is continually ran on top level */
{

   if ((pulsObj->cycls <=  pulsObj->cyclCnt) && (pulsObj->activate==true))
   {
      calculateTimeDiff( &pulsObj->TimeDiff, &pulsObj->TimeDurLast );
    
      if (pulsObj->cycTim > pulsObj->TimeDiff)                                         /* first duty cycle */
      {
         MAV_SET_RELAY(pulsObj->relayNum);
      }
      else if ((pulsObj->TimeDiff > pulsObj->cycTim) && (pulsObj->TimeDiff < (pulsObj->cycTim*2LL))) /* second duty cycle */
      {
         MAV_CLR_RELAY(pulsObj->relayNum);
      }
      else                                                                      /* reset after second duty cycle complete */
      {
        pulsObj->TimeDiff = -1;                                                          /*  reset the cycle counter */
        pulsObj->cyclCnt=++pulsObj->cyclCnt % UINT8_MAX;
      }
   }
   else
   {
      pulsObj->activate = false;
   }
}
#endif
// =================== Encoder helpers =========================================
#ifdef ENCODER_HELPER

/*-----------------------------------------------------------------------------
 *      xor_c:  do XOR function on char
 *
 *  Parameters: unsigned char a, unsigned char b
 *
 *  Return: xor value
 *----------------------------------------------------------------------------*/
unsigned char xor_c(unsigned char a, unsigned char b) { return (a == b) ? '0' : '1'; }

/*-----------------------------------------------------------------------------
 *      flip:  Helper function to flip the bit
 *
 *  Parameters: unsigned char c
 *
 *  Return: inverted value (~)
 *----------------------------------------------------------------------------*/
unsigned char flip(unsigned char c) { return (c == '0') ? '1' : '0'; }

/*-----------------------------------------------------------------------------
 *      binarytoGray:  function to convert binary string to gray string
 *
 *  Parameters: gray_bin_t *word
 *
 *  Return: converted string
 *----------------------------------------------------------------------------*/
uint16_t binarytoGray(gray_bin_t *word)
{
    //unsigned char gray[16u];
    //uint32_t g1;
    int8_t i;
    uint16_t totalWord=0u;

    word->transState = ENCODER_BIN_2_GRAY;                                      // set lock so not to interrupt write until completed processing these 16 bits
    word->gray[0u] = word->binary[0u];                                          // MSB of gray code is same as binary code

    for (i = 1; i < strlen(word->binary); i++)                                  // Compute remaining bits, next bit is comuted by doing XOR of previous and current in Binary
    {
        word->gray[i] = xor_c(word->binary[i - 1],word->binary[i]);             // Concatenate XOR of previous bit with current bit
        if (word->gray[i] == '1')
          totalWord=totalWord + (1u<<(i-1u));
    }
    word->transState = ENCODER_COMPLETE;
    return totalWord;
}

/*-----------------------------------------------------------------------------
 *      graytoBinary:  function to convert gray code string to binary string
 *
 *  Parameters: gray_bin_t *word
 *
 *  Return: converted string
 *----------------------------------------------------------------------------*/
uint16_t graytoBinary(gray_bin_t *word)
{
    //unsigned char binary[16u];
    int8_t i;
    uint16_t totalWord=0u;

    word->transState = ENCODER_GRAY_2_BIN;                                      // set lock so not to interrupt write until completed processing these 16 bits
    word->binary[0u] = word->gray[0u];                                          // MSB of binary code is same as gray code

    for (i = 1; i < strlen(word->gray); i++)                                    // Compute remaining bits
    {
        if (word->gray[i] == '0')                                               // If current bit is 0, concatenate previous bit
            word->binary[i] = word->binary[i - 1u];
        else
            word->binary[i] = flip(word->binary[i - 1u]);                       // Else, concatenate invert of previous bit
        if (word->binary[i] == '1')
          totalWord=totalWord + (1u<<(i-1u));
    }
    word->transState = ENCODER_COMPLETE;
    return totalWord;
}
/*-----------------------------------------------------------------------------
 *      int_to_ascii:  integer to ascii
 *
 *  Parameters: uint8_t nibble
 *
 *  Return: ascii characture
 *----------------------------------------------------------------------------*/
char int_to_ascii(uint8_t nibble)
{
   nibble = nibble & 0x0F;
   if(nibble < 0x0A)
     return nibble + '0';
   else
     return nibble + 'A' - 0x0A;
}
/*-----------------------------------------------------------------------------
 *      collectGray:  collect grayscale input
 *
 *  Parameters: gray_bin_t *word, uint16_t bitNo
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void collectGray(gray_bin_t *word, uint16_t bitNo)
{
    uint8_t cntBin;

    switch(word->transState)                                                    /* check the state of the transfer */
    {
        case ENCODER_COMPLETE:                                                  /* nothing else is trying to read the data so we can unconditionally write to it */
        if (word->parIdx >= 1u)                                                 /* at least one char came in via interrupt while reading encoded input */
        {
           for (cntBin=0u; cntBin<= word->parIdx; cntBin++)                     /* for each char you collected */
           {
               word->gray[word->grayIdx] = word->inUse[cntBin];                 /* append the gray code cyclic buffer with the input states read */
               word->grayIdx = ++word->grayIdx % 16u;                           /* increment the buffer pointer */
           }                          
           word->parIdx = 0u;                                                   /* reset the parameter index which is used to read to the alternate buffer during encoder reading (busy stat) */
        }
        if (bitNo <= 255u)                                                      /*  our input encoder signal is in the first word */
        {
           word->gray[word->grayIdx] = int_to_ascii(DINPORT1 & bitNo);          /* 16 bit cyclic read into char buffer */
        }
        else                                                                    /* our input encoder signal is in the second word */
        {
           word->gray[word->grayIdx] = int_to_ascii(DINPORT2 & (bitNo>>8u));    /* 16 bit cyclic read into char buffer */
        }
        word->grayIdx = ++word->grayIdx % 16u;
        break;

        default:                                                                /* something is using the data so we write to another buffer until it has completed */
        if (bitNo <= 255u)                                                      /* our input encoder signal is in the first word */
        {
            word->inUse[word->parIdx] = int_to_ascii(DINPORT1 & bitNo);
        }
        else                                                                    /* our input encoder signal is in the second word */
        {
            word->inUse[word->grayIdx] = int_to_ascii(DINPORT2 & (bitNo>>8u));  /* 16 bit cyclic read into char buffer */
        }
        word->parIdx = ++word->parIdx % 16u;
        break;
    }

}

#endif                                                                          // End of encoder helper
#if defined(STEPPER_MOTOR)
/*-----------------------------------------------------------------------------
 *      setPhaseStepper():  set the stepper motor phase
 *
 *  Parameters: StepperDriver_t *motor
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void setPhaseStepper( StepperDriver_t *motor )
{
   switch(motor->type)
   {
      case SingPhse:
      motor->phase = ((SinglePhaseSteppingRow4 << 12u) | (SinglePhaseSteppingRow3 << 8u) | (SinglePhaseSteppingRow2 << 4u) | (SinglePhaseSteppingRow1));
      motor->stepsPerRev = 4u;
      break;
      
      case DualPhse:
      motor->phase = ((DualPhaseSteppingRow4 << 12u) | (DualPhaseSteppingRow3 << 8u) | (DualPhaseSteppingRow2 << 4u) | (DualPhaseSteppingRow1));
      motor->stepsPerRev = 4u;
      break;
      
      case HalfPhse:
      motor->phase = (((HalfSteppingRow8 << 28u) | (HalfSteppingRow7 << 24u) | (HalfSteppingRow6 << 20u) | (HalfSteppingRow5 << 16u)) | ((HalfSteppingRow4 << 12u) | (HalfSteppingRow3 << 8u) | (HalfSteppingRow2 << 4u) | (HalfSteppingRow1)));
      motor->stepsPerRev = 8u;
      break;
      
      default:
      break;
   }
}
/*-----------------------------------------------------------------------------
 *      oneStepMotor():  Step moves motor one step in the chosen direction
 *
 *  Parameters: StepperDriver_t *smtr
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void oneStepMotor( StepperDriver_t *smtr )
{
    uint8_t phseEle;
    uint8_t stepSet;
    uint8_t i;
    
    if (smtr->direction == StepFwd)
    {
       smtr->stepNum = ++smtr->stepNum % smtr->stepsPerRev;
    }
    else if (smtr->stepNum <= 0u)
    {
       smtr->stepNum = smtr->stepsPerRev - 1u;
    }
    else if (smtr->direction == StepStop)
    {
       return;
    }
    else
    {
       smtr->stepNum = --smtr->stepNum;
    }
    phseEle = smtr->stepNum % 4u;
    stepSet = ((uint8_t)(smtr->phase >> (smtr->stepNum*4u)) & 0xFFu);
    for (i=0; i<=STEPPER_NO_OF_PINS; i++)
    {
       if ( stepSet & ((uint8_t)pow(2.0f,(float64_t)i)) )
       {
          switch(smtr->bank[i])
          {
             case pinUsesPORTA:
             LATA = LATA | ((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));     /* consider look up table from memory if too slow here */
             break;

             case pinUsesPORTB:
             LATB = LATB | ((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;
             
             case pinUsesPORTC:
             LATC = LATC | ((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;

             case pinUsesPORTD:
             LATD = LATD | ((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;
             
             case pinUsesPORTE:
             LATE = LATE | ((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;

             case pinUsesPORTF:
             LATF = LATF | ((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;
             
             case pinUsesPORTG:
             LATG = LATG | ((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;
          }
       }
       else
       {
          switch(smtr->bank[i])
          {
             case pinUsesPORTA:
             LATA = LATA & ~((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;

             case pinUsesPORTB:
             LATB = LATB & ~((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;

             case pinUsesPORTC:
             LATC = LATC & ~((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;

             case pinUsesPORTD:
             LATD = LATD & ~((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;

             case pinUsesPORTE:
             LATE = LATE & ~((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;

             case pinUsesPORTF:
             LATF = LATF & ~((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;

             case pinUsesPORTG:
             LATG = LATG & ~((uint8_t)pow(2.0f,((float64_t)smtr->pins[i])));
             break;
          }
       }
    }
}
#endif
/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2017, Fetch Robotics Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  All rights reserved.
 *  Author: Michael Ferguson
 *
 *  ported to pic32/FT900 by ACP Aviation
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Unbounded Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
 /*-----------------------------------------------------------------------------
 *      calcSafetyScalefromLaser:  calculate the safety scaling for object avoidance
 *
 *  Parameters: LaserScan_t *scan
 *
 *  Return: none
 *----------------------------------------------------------------------------*/
void calcSafetyScalefromLaser( LaserScan_t *scan )
{
  float64_t angle = scan->angle_min;
  float64_t min_dist = safety_scaling_distance_;
  float64_t py;
  size_t i;

  if (scan == NULL) return;
  
  for (i = 0; i < sizeof(scan->ranges); ++i, angle += scan->angle_increment)
  {
    if ((scan->ranges[i] > scan->range_min) && (scan->ranges[i] < min_dist))
    {
      if (angle < -1.5f || angle > 1.5f)                                        /* Only test points in the forward 180 degrees */
        continue;

      py = sin(angle) * scan->ranges[i];                                        /* Check if point is inside the width of the robot */
      if (fabs(py) < (robot_width_/2.0f))
        min_dist = scan->ranges[i];
    }
  }

  if (0.1f > (min_dist / safety_scaling_distance_))
  {
     scan->safety_scaling_ = 0.1f;
  }
  else
  {
     scan->safety_scaling_ = min_dist / safety_scaling_distance_;
  }
  scan->lastLaserTime = CP0_GET(CP0_COUNT);
}
 /*-----------------------------------------------------------------------------
 *      doVeloAccLimit:  set velocity and acceloration limits for object avoidance
 *
 *  Parameters: LaserScan_t *scan
 *
 *  Return: none
 *----------------------------------------------------------------------------*/
void doVeloAccLimit( LaserScan_t *odomObj )
{
   const float64_t max_velocity_x_ = 1.0f;
   const float64_t max_velocity_r_ = 4.5f;
   const float64_t max_acceleration_x_ = 0.75f;
   const float64_t max_acceleration_r_ = 3.0f;
   float64_t x, r, actual_scaling;

   int32_t timNow;
   int32_t LasrTim2Now;

  if (odomObj == NULL) return;
  
  if (odomObj->init == 0u)
  {
      odomObj->lastUpdTime = CP0_GET(CP0_COUNT);                                /* initialise the timer */
      odomObj->init = 1u;
      return;
  }

  calculateTickDiff(&timNow,&odomObj->lastUpdTime);                             // See if we have timed out and need to stop
  if (timNow >= odomObj->scanTimOut)
  {
    odomObj->desired_x_ = odomObj->desired_r_ = 0.0f;
  }

  calculateTickDiff(&LasrTim2Now,&odomObj->lastLaserTime);                      // Make sure laser has not timed out
  if ((safety_scaling_distance_ > 0.0f) &&  (LasrTim2Now > odomObj->lastLaserTime))
  {
    odomObj->safety_scaling_ = 0.1f;
  }
  
  // Limit linear velocity based on obstacles twist.linear.x
  // x = max(-max_velocity_x_ * odomObj->safety_scaling_, min(desired_x_, max_velocity_x_ * odomObj->safety_scaling_));
  if ( odomObj->desired_x_ < (max_velocity_x_ * odomObj->safety_scaling_))
  {
    x = odomObj->desired_x_;
  }
  else
  {
    x = (max_velocity_x_ * odomObj->safety_scaling_);
  }
  if ((-max_velocity_x_ * odomObj->safety_scaling_) > x)
  {
    x = -max_velocity_x_ * odomObj->safety_scaling_;
  }
  actual_scaling = 1.0f;                                                        // Compute how much we actually scaled the linear velocity
  if (odomObj->desired_x_ != 0.0f)
    actual_scaling = x/odomObj->desired_x_;
  // Limit angular velocity twist.angular.z;
  // Scale same amount as linear velocity so that robot still follows the same "curvature"
  //r = std::max(-max_velocity_r_, std::min(actual_scaling * desired_r_, max_velocity_r_));
  if ( (actual_scaling * odomObj->desired_r_) < max_velocity_r_)
  {
    r = (actual_scaling * odomObj->desired_r_);
  }
  else
  {
    r = max_velocity_r_;
  }
  if (-max_velocity_r_ > r)
  {
    r = -max_velocity_r_;
  }

  if (x > odomObj->last_sent_x_)
  {
    odomObj->last_sent_x_ += max_acceleration_x_ * TICKS2SEC(timNow);
    if (odomObj->last_sent_x_ > x)
      odomObj->last_sent_x_ = x;
  }
  else
  {
    odomObj->last_sent_x_ -= max_acceleration_x_ * TICKS2SEC(timNow);
    if (odomObj->last_sent_x_ < x)
      odomObj->last_sent_x_ = x;
  }
  if (r > odomObj->last_sent_r_)
  {
    odomObj->last_sent_r_ += max_acceleration_r_ * TICKS2SEC(timNow);
    if (odomObj->last_sent_r_ > r)
      odomObj->last_sent_r_ = r;
  }
  else
  {
    odomObj->last_sent_r_ -= max_acceleration_r_ * TICKS2SEC(timNow);
    if (odomObj->last_sent_r_ < r)
      odomObj->last_sent_r_ = r;
  }
  odomObj->lastUpdTime = CP0_GET(CP0_COUNT);                                    /* delete the execution time element of the phase */

}
 /*-----------------------------------------------------------------------------
 *      setPoseAnTwist:  set pose and twist for odometery
 *
 *  Parameters: LaserScan_t *odomObj, odom_t *odomPoseTwist, movingObjectStat_t *movObj
 *
 *  Return: none
 *----------------------------------------------------------------------------*/
void setPoseAnTwist( LaserScan_t *odomObj, odom_t *odomPoseTwist, movingObjectStat_t *movObj )
{
  const float64_t track_width_ = 0.37476f;
  const float64_t wheel_rotating_threshold_ = 0.001f;
  const float64_t moving_threshold_ = 0.05f;
  const float64_t rotating_threshold_ = 0.05f;
  float64_t dx = 0.0f;
  float64_t dr = 0.0f;
  float64_t d;
  float64_t th;

  // Threshold the odometry to avoid noise (especially in simulation)
  if (fabs(movObj->left_dx) > wheel_rotating_threshold_ || fabs(movObj->right_dx) > wheel_rotating_threshold_ || odomObj->last_sent_x_ != 0.0f || odomObj->last_sent_r_ != 0.0f)
  {
    odomObj->left_last_position_ = movObj->left_pos;                            // Above threshold, update last position
    odomObj->right_last_position_ = movObj->right_pos;
  }
  else
  {
    movObj->left_dx = movObj->right_dx = 0.0f;                                  // Below threshold, zero delta/velocities
    movObj->left_vel = movObj->right_vel = 0.0f;
  }

  d = (movObj->left_dx+movObj->right_dx)/2.0f;                                  // Calculate forward and angular differences
  th = (movObj->right_dx-movObj->left_dx)/track_width_;

  dx = (movObj->left_vel + movObj->right_vel)/2.0f;                             // Calculate forward and angular velocities
  dr = (movObj->right_vel - movObj->left_vel)/track_width_;

  if (fabs(dx) > moving_threshold_ || fabs(dr) > rotating_threshold_ || odomObj->last_sent_x_ != 0.0f || odomObj->last_sent_r_ != 0.0f)   // Actually set command
  {
     odomPoseTwist->leftJoint = (odomObj->last_sent_x_ - ((odomObj->last_sent_r_/2.0f) * track_width_));    /* left wheel joint */
     odomPoseTwist->rightJoint = (odomObj->last_sent_x_ + ((odomObj->last_sent_r_/2.0f) * track_width_));   /* right wheel joint */
  }
  
  odomPoseTwist->pose.theta.x += th/2.0f;                                       // Update stored odometry pose...
  odomPoseTwist->pose.position.x += d*cos(odomPoseTwist->pose.theta.x);
  odomPoseTwist->pose.position.y += d*sin(odomPoseTwist->pose.theta.x);
  odomPoseTwist->pose.theta.x += th/2.0f;
  odomPoseTwist->pose.orientation.z = sin(odomPoseTwist->pose.theta.x/2.0f);
  odomPoseTwist->pose.orientation.w = cos(odomPoseTwist->pose.theta.x/2.0f);

  odomPoseTwist->twist.linear.x = dx;                                           // ...and twist    (for left joint handle)
  odomPoseTwist->twist.angular.z = dr;                                          // (right joint handle)
  odomObj->lastUpdTime = CP0_GET(CP0_COUNT);                                    /* delete the execution time element of the phase */
}
/*******************************************************************************
 INDI Dome Base Class
 Copyright(c) 2014 Elwood C Downey, Jason Harris, Jasem Mutlaq. All rights reserved.

 The code used calculate dome target AZ and ZD is written by Ferran Casarramona, and adapted from code from Markus Wildi.
 The transformations are based on the paper Matrix Method for Coodinates Transformation written by Toshimi Taki (http://www.asahi-net.or.jp/~zs3t-tk).

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.

 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/
void get_alt_az_coordinates(float64_t Ha, float64_t Dec, float64_t Lat, float64_t *Alt1, float64_t *Az1)
{
    float64_t alt, az;
    Ha *= PI / 180.0f;
    Dec *= PI / 180.0f;
    Lat *= PI / 180.0f;
    alt = asin(sin(Dec) * sin(Lat) + cos(Dec) * cos(Lat) * cos(Ha));
    az = acos((sin(Dec) - sin(alt)*sin(Lat)) / (cos(alt) * cos(Lat)));
    alt *= 180.0f / PI;
    az *= 180.0f / PI;
    if (sin(Ha) >= 0.0f)
        az = 360u - az;
    *Alt1 = alt;
    *Az1 = az;
}
 /*-----------------------------------------------------------------------------
 *      estimate_geocentric_elevation:
 *
 *  Parameters: float64_t Lat, float64_t El
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t estimate_geocentric_elevation(float64_t Lat, float64_t El)
{
    Lat *= PI / 180.0f;
    Lat = sin(Lat);
    El += Lat * (EARTHRADIUSPOLAR - EARTHRADIUSEQUATORIAL);
    return El;
}
 /*-----------------------------------------------------------------------------
 *      estimate_field_rotation_rate:
 *
 *  Parameters: float64_t Alt, float64_t Az, float64_t Lat
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t estimate_field_rotation_rate(float64_t Alt, float64_t Az, float64_t Lat)
{
    float64_t ret;
    Alt *= PI / 180.0f;
    Az *= PI / 180.0f;
    Lat *= PI / 180.0f;
    ret = cos(Lat) * cos(Az) / cos(Alt);
    ret *= 180.0f / PI;
    return ret;
}
 /*-----------------------------------------------------------------------------
 *      estimate_field_rotation:
 *
 *  Parameters: float64_t HA, float64_t rate
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t estimate_field_rotation(float64_t HA, float64_t rate)
{
    HA *= rate;
    while(HA >= 360.0f)
        HA -= 360.0f;
    while(HA < 0)
        HA += 360.0f;
    return HA;
}
/*
The problem to get a dome azimuth given a telescope azimuth, altitude and geometry (telescope placement, mount geometry) can be seen as solve the intersection between the optical axis with the dome, that is, the intersection between a line and a sphere.
To do that we need to calculate the optical axis line taking the centre of the dome as origin of coordinates.
*/
 /*-----------------------------------------------------------------------------
 *       Dome_Intersection :
 *
 *  Parameters: const dVectr *p1,const Vectr *dp, float64_t r, float64_t *mu1, float64_t *mu2
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t Dome_Intersection(const dVectr *p1,const Vectr *dp, float64_t r, float64_t *mu1, float64_t *mu2)
{
    float64_t a, b, c;
    float64_t bb4ac;

    if (((p1 == NULL) || (dp == NULL)) || ((mu1 == NULL) || (mu2 == NULL))) return false;
    
    a = dp->x * dp->x + dp->y * dp->y + dp->z * dp->z;
    b = 2 * (dp->x * p1->x + dp->y * p1->y + dp->z * p1->z);
    c = 0.0;
    c = c + p1->x * p1->x + p1->y * p1->y + p1->z * p1->z;
    c = c - r * r;
    bb4ac = b * b - 4 * a * c;
    if ((fabs(a) < 0.0000001f) || (bb4ac < 0))
    {
        *mu1 = 0.0f;
        *mu2 = 0.0f;
        return false;
    }

    *mu1 = (-b + sqrt(bb4ac)) / (2.0f * a);
    *mu2 = (-b - sqrt(bb4ac)) / (2.0f * a);

    return true;
}
 /*-----------------------------------------------------------------------------
 *      Dome_OpticalCenter:  dome optical center
 *
 *  Parameters: dVectr *MountCenter, float64_t dOpticalAxis, float64_t Lat, float64_t Ah, dVectr *OP
 *
 *  Return: none
 *----------------------------------------------------------------------------*/
uint8_t Dome_OpticalCenter(dVectr *MountCenter, float64_t dOpticalAxis, float64_t Lat, float64_t Ah, dVectr *OP)
{
    float64_t q, f;
    float64_t cosf1, sinf1, cosq1, sinq1;

    if (OP == NULL) return false;
    
    q = PI * (90.0f - Lat) / 180.0f;                                            // Note: this transformation is a circle rotated around X axis -(90 - Lat) degrees
    f = -PI * (180.0f + Ah * 15.0f) / 180.0f;

    cosf1 = cos(f);
    sinf1 = sin(f);
    cosq1 = cos(q);
    sinq1 = sin(q);

    OP->x = (dOpticalAxis * cosf1 + MountCenter->x);                            // The sign of dOpticalAxis determines de side of the tube
    OP->y = (dOpticalAxis * sinf1 * cosq1 + MountCenter->y);
    OP->z = (dOpticalAxis * sinf1 * sinq1 + MountCenter->z);

    return true;
}
 /*-----------------------------------------------------------------------------
 *       Dome_OpticalVector :
 *
 *  Parameters: float64_t Az, float64_t Alt, dVectr *OV
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t Dome_OpticalVector(float64_t Az, float64_t Alt, dVectr *OV)
{
    float64_t q, f;

    if (OV == NULL) return false;
    
    q    = PI * Alt / 180.0f;
    f    = PI * Az / 180.0f;
    OV->x = cos(q) * sin(f);
    OV->y = cos(q) * cos(f);
    OV->z = sin(q);

    return true;
}
 /*-----------------------------------------------------------------------------
 *       Dome_Csc :
 *
 *  Parameters: float64_t x
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t Dome_Csc(float64_t x)
{
    return 1.0f / sin(x);
}
 /*-----------------------------------------------------------------------------
 *       Dome_Sec :
 *
 *  Parameters: float64_t x
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t Dome_Sec(float64_t x)
{
    return 1.0f / cos(x);
}
 /*-----------------------------------------------------------------------------
 *       Dome_CheckHorizon :
 *
 *  Parameters: float64_t HA, float64_t dec, float64_t lat
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t Dome_CheckHorizon(float64_t HA, float64_t dec, float64_t lat)
{
    float64_t sinh_value;

    sinh_value = cos(lat) * cos(HA) * cos(dec) + sin(lat) * sin(dec);

    if (sinh_value >= 0.0f)
        return true;

    return false;
}

/*-----------------------------------------------------------------------------
 *       Euclid_dist2point : Computes Euclidean distance between point (x1,y1) and point (x2,y2).
 *
 *  Copyright 2010-2011 rafael grompone von gioi (grompone@gmail.com)
 *
 *  Parameters: float64_t x1, float64_t y1, float64_t x2, float64_t y2
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t Euclid_dist2point(float64_t x1, float64_t y1, float64_t x2, float64_t y2)
{
  return sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
}
#if defined(MACHINE_LEARN)
/* -------------------------------------------------------------------------
%
% GoldenSectionMethod()
%
% Discription : Sample code of the golden section method for linear search
% 
% Environment : Matlab ported to C by ACP 
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai Ported by ACP Aviation
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------  */
#define GOLDEN_RATIO 1.6180339887498948482045868343656f
/*-----------------------------------------------------------------------------
 *      golden_f : golden section function
 *                             
 *  Parameters: float64_t x  
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t golden_f( float64_t x )
{
    return (pow((x-2.0f),2.0f)+5.0f);
}
/*-----------------------------------------------------------------------------
 *      goldenInit : initialise goldensection object
 *                             
 *  Parameters: golden_section_t *gold  
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void goldenInit( golden_section_t *gold )
{
  gold->a=-50.0f;
  gold->b=100.0f;

  gold->x1 = ((gold->a - gold->b)/(GOLDEN_RATIO + 1.0f)) + gold->b;
  gold->x2 = ((gold->a - gold->b)/GOLDEN_RATIO) + gold->b;
  gold->f1 = golden_f(gold->x1);
  gold->f2 = golden_f(gold->x2);
}
/*-----------------------------------------------------------------------------
 *      goldenIterate : Linear search with golden Section
 *                             
 *  Parameters: golden_section_t *gold  
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void goldenIterate( golden_section_t *gold )
{
    if (gold->f1 < gold->f2)
    {
        gold->a = gold->x2;
        gold->x2 = gold->x1;
        gold->f2 = gold->f1;
        gold->x1 = ((gold->a - gold->b)/(GOLDEN_RATIO + 1.0f)) + gold->b;
        gold->f1 = golden_f(gold->x1);
    }
    else
    {
        gold->b = gold->x1;
        gold->x1 = gold->x2;
        gold->f1 = gold->f2;
        gold->x2 = ((gold->a - gold->b)/GOLDEN_RATIO) + gold->b;
        gold->f2 = golden_f(gold->x2);
    }

    if (abs(gold->a-gold->b)<=pow(10.0f,-3.0f))
    {
        gold->minX = (gold->a + gold->b)/2.0f;
        gold->minY = golden_f((gold->a + gold->b)/2.0f);
    }
}
#endif

#ifdef ROBOT_HELPER
/* Himmelblau's function
% see Himmelblau's function - Wikipedia, the free encyclopedia 
% http://en.wikipedia.org/wiki/Himmelblau%27s_function   */
/*-----------------------------------------------------------------------------
 *      doHimmelBlau : Himmelblau's function
 *                             
 *  Parameters: robot_navig_t *rob  
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t doHimmelBlau( robot_navig_t *rob )
{
    return(pow((pow(rob->x1,2.0f)+rob->x2-11.0f),2.0f)+pow((pow((rob->x1+rob->x2),2.0f)-7.0f),2.0f));
}

/*-----------------------------------------------------------------------------
 *      CalcJacobiOfHimmel : jacobi matrix of Himmelblau's function
 *                             
 *  Parameters: robot_navig_t *rob  
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr CalcJacobiOfHimmel( robot_navig_t *rob )
{
   rob->dx=4.0f*pow(rob->x,3.0f)+4.0f*rob->x*rob->y-44.0f*rob->x+2.0f*rob->x+2.0f*pow(rob->y,2.0f)-14.0f;
   rob->dy=2.0f*pow(rob->x,2.0f)+4.0f*rob->x*rob->y+4.0f*pow(rob->y,3.0f)-26.0f*rob->y-22.0f;
   return mkvec( rob->dx, rob->dy, 0.0f);
} 
/*-----------------------------------------------------------------------------
 *      SteepestDescentMethod : SteepestDescentMethodOptimization
 *                             
 *  Parameters: Vectr param 
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void SteepestDescentMethod( Vectr param )
{
  uint8_t i,iterationMax=50u; 
  float64_t alpha=0.01f;
  Vectr J;
  robot_navig_t robo;
  
  for (i=0;i<=iterationMax;i++)
  {
    robo.x = param.x;
    robo.y = param.y;
    J = CalcJacobiOfHimmel( &robo );
    param.x = (param.x - alpha*J.x);
    param.y = (param.y - alpha*J.y);
    robo.x1 = param.x;
    robo.x2 = param.y;
    param.z = doHimmelBlau( &robo );
  }
}
/* HOMOGENEOUSTRANSFORMATION2D 
%
%   Input1: [x_in_1 y_in_1;
%                        x_in_2  y_in_2;
%                               ....]

%   Input2: [x_base y_base theta_base]
%   Input3:
%           mode=0.
%           mode=1
%
%   Output1: [x_out y_out;
%                        x_out_2  y_out_2;
%                               ....]   */
homo_2d_t HomogeneousTransformation2D(homo_2d_t *in, homo_2d_t *base, uint8_t mode)
{
   homo_2d_t Rot;
   homo_2d_t baseMat;
   homo_2d_t out;
   
   Rot.x_1 = cos(base->x_2);
   Rot.y_1 = sin(base->x_2);
   Rot.x_2 = -sin(base->x_2);
   Rot.y_2 = cos(base->x_2); 
   
   baseMat.x_1 = base->x_1;
   baseMat.y_1 = base->y_1;
   baseMat.x_2 = in->x_1;
   baseMat.y_2 = 1.0f;

   if (mode == 0)
   {
      out.x_1=baseMat.x_1+in->x_1*Rot.x_1;
      out.y_1=baseMat.x_1+in->x_1*Rot.x_1;
      out.x_2=baseMat.x_2+in->x_2*Rot.x_2;
      out.y_2=baseMat.y_2+in->y_2*Rot.y_2;
   }
   else
   {
       out.x_1=(baseMat.x_1+in->x_1)*Rot.x_1;
       out.y_1=(baseMat.y_1+in->y_1)*Rot.y_1;
       out.x_2=(baseMat.x_2+in->x_2)*Rot.x_2;
       out.y_2=(baseMat.y_2+in->y_2)*Rot.y_2;
   }     
   return out;
   
}
/*-----------------------------------------------------------------------------
 *      doSLAMControl : do slam control in 2d
 *                             
 *  Parameters: uint64_t time
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr doSLAMControl( uint64_t time )                                            // iterative time of function
{
   // Calc Input Parameter
   float64_t T=10.0f*10.0f;                                                     // 10[sec]
   Vectr res;
   
   // [V yawrate]
   float64_t V=1.0f;                                                            // [m/s]
   float64_t yawrate = 5.0f;                                                    // [deg/s]
 
   res.x = V*(1.0f-exp(-time/T));
   res.y = DEGREE_TO_RADIAN(yawrate)*(1.0f-exp(-time/T));
   return res;
}
/*-----------------------------------------------------------------------------
 *      doSLAMAngle : compute slam angle vector
 *                             
 *  Parameters: Vectr *angle
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void doSLAMAngle( Vectr *angle )                                                // vector of angles in radians
{
  angle->x = FMOD2(angle->x, (2.0f*PI));
  angle->y = FMOD2(angle->y, (2.0f*PI));
  angle->z = FMOD2(angle->z, (2.0f*PI));
  
  if (angle->x>PI)
    angle->x = angle->x - 2.0f*PI;
  if (angle->y>PI)
    angle->y = angle->y - 2.0f*PI;
  if (angle->z>PI)
    angle->z = angle->z - 2.0f*PI;

  if (angle->x<-PI)
    angle->x = angle->x + 2.0f*PI;
  if (angle->y<-PI)
    angle->y = angle->y + 2.0f*PI;
  if (angle->z<-PI)
    angle->z = angle->z + 2.0f*PI;
}

/*-----------------------------------------------------------------------------
 *      doASTARAngle : compute ASTAR angle vector
 *                             
 *  Parameters: Vectr *angle, uint32_t tick
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void doASTARAngle( Vectr *angle, uint32_t tick )                                // orientation angles in degrees, cpu ticks
{
    Vectr angledeg;
    
    angledeg.x = angle->x * tick;
    angledeg.y = angle->y * tick;
    angledeg.z = angle->z * tick;
    
    while (angledeg.x > 180.0f)
        angledeg.x=angledeg.x-360.0f;
    while (angledeg.y > 180.0f)
        angledeg.y=angledeg.y-360.0f;
    while (angledeg.z > 180.0f)
        angledeg.z=angledeg.z-360.0f;

    while (angledeg.x < -180.0f )
        angledeg.x=angledeg.x+360.0f;
    while (angledeg.y < -180.0f)
        angledeg.y=angledeg.y+360.0f;
    while (angledeg.z < -180.0f)
        angledeg.z=angledeg.z+360.0f;
                                
    angle->x=angledeg.x/tick;
    angle->y=angledeg.y/tick;
    angle->z=angledeg.z/tick;
}
/*-----------------------------------------------------------------------------
 *      MotionModelASTAR : motion model astar
 *                             
 *  Parameters: uint32_t tick
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat MotionModelASTAR( uint32_t tick )
/*???????????? ??????????????????????
% [dx dy dyaw cost]
% next=[1 0    0/tick 1
%       1 1   45/tick 2
%       1 -1  -45/tick 2
%       -1 0   0/tick 3
%       -1 1  45/tick 4
%       -1 -1 -45/tick 4]; */
{
  float64_t movedis = 1.0f;
  float64_t dangle[3u] = {-30.0f,15.0f,30.0f};
  quat next;
  float64_t rad[3u];
  int8_t i;
  
  for (i=1;i<3;i++)
  {
    rad[i] = DEGREE_TO_RADIAN(dangle[i]);
    next.x = movedis*cos(rad[i]);
    next.y = movedis*sin(rad[i]);
    next.z = -dangle[i]/tick;
  }
  next.w = 1.0f;
  return next;
}
/* 
   https://github.com/darglein/Snake-SLAM
   Darius Rückert - University of Erlangen
   Ported by ACP Aviation
*/

/*-----------------------------------------------------------------------------
 *  SnakeSlam_ComputeThreeMaxima : Return the indexs of the 3 largests values in the 
 *                             histo array.
 *
 *  Parameters: int16_t* histo, const int16_t histoLen, int16_t *ind1, 
 *              int16_t *ind2, int16_t *ind3
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/
void SnakeSlam_ComputeThreeMaxima(int16_t* histo, const int16_t histoLen, int16_t *ind1, int16_t *ind2, int16_t *ind3)
{
    int16_t max1 = 0;
    int16_t max2 = 0;
    int16_t max3 = 0;
    int16_t i,s;
        
    for (i = 0; i < histoLen; i++)
    {
        s = histo[i];
        if (s > max1)
        {
            max3 = max2;
            max2 = max1;
            max1 = s;
            *ind3 = *ind2;
            *ind2 = *ind1;
            *ind1 = i;
        }
        else if (s > max2)
        {
            max3 = max2;
            max2 = s;
            *ind3 = *ind2;
            *ind2 = i;
        }
        else if (s > max3)
        {
            max3 = s;
            *ind3 = i;
        }
    }

    if (max2 < 0.1f * (float64_t)max1)
    {
        *ind2 = -1;
        *ind3 = -1;
    }
    else if (max3 < 0.1f * (float64_t)max1)
    {
        *ind3 = -1;
    }
}
#endif
/*-----------------------------------------------------------------------------
 *      v2Float : convert vector to scalar
 *                             
 *  Parameters: Vectr a
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t v2Float( Vectr a )                                                    /* make a float from the vector */
{
  return ((a.x + a.y + a.z) / 3.0f);
}

/* Reference:MATLAB (Path Smoothing) - 
%           http://d.hatena.ne.jp/meison_amsl/20140510/1399694663   */
/*-----------------------------------------------------------------------------
 *      PathSmoothing : path smoothing as above ref
 *                             
 *  Parameters: Vectr path[LEN_OF_PATH]
 *
 *  Return:     Vectr *
 *----------------------------------------------------------------------------*/
Vectr * PathSmoothing( Vectr path[LEN_OF_PATH] )
{

   float64_t alpha1 = 0.5f, beta1 = 0.2f, torelance = 0.00001f;
   float64_t change=torelance;
   Vectr optPath[LEN_OF_PATH];
   Vectr prePath;
   int8_t ip;

   while (change>=torelance)
   { 
      change=0.0f;
      for (ip=2;ip<LEN_OF_PATH;ip++)
      { 
         //prePath=optPath(ip,:);%??????
         prePath = mkvec(optPath[ip].x, optPath[ip].y, optPath[ip].z);
         optPath[ip] = vsub(optPath[ip], vscl(alpha1, vsub(optPath[ip], path[ip])));
         optPath[ip] = vsub(optPath[ip], vscl(beta1, vsub(vsub(vscl(2.0f, optPath[ip]), optPath[ip-1u]), optPath[ip+1u])));
         //change=change+norm(optPath[ip]-prePath);
         change=change+v2Float(vnormalize(vsub(optPath[ip], prePath)));
      }
   }

   return &optPath;

}
// Map [-pi, pi] -> [1000, 2000]
/*-----------------------------------------------------------------------------
 *      MapAngleToPpm : map angle to ppm as above
 *                             
 *  Parameters: float64_t angle
 *
 *  Return:     uint16_t
 *----------------------------------------------------------------------------*/
uint16_t MapAngleToPpm(float64_t angle) {
  uint16_t ppm = ((uint16_t)((angle - (-PI)) / (PI - (-PI)) * (1000.0f) + 1000.0f));
  return ppm;
}
// Map [-1.0, 1.0] -> [1000, 2000]
/*-----------------------------------------------------------------------------
 *      MapSpeedToPpm : map speed to ppm as above
 *                             
 *  Parameters: float64_t speed
 *
 *  Return:     uint16_t
 *----------------------------------------------------------------------------*/
uint16_t MapSpeedToPpm(float64_t speed) {
  if (speed > 1.0f || speed < -1.0f) {
    return 1500u;
  }
  return ((uint16_t)(1500.0f + speed * 500.0f));
}

/*-----------------------------------------------------------------------------
 *      init_rotation_matrix : Init rotation matrix using euler angles
 *                             
 *  Parameters: float64_t m[3][3], float64_t yaw, float64_t pitch, float64_t roll
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void init_rotation_matrix(float64_t m[3][3], float64_t yaw, float64_t pitch, float64_t roll)
{
  float64_t c1 = cos(roll);
  float64_t s1 = sin(roll);
  float64_t c2 = cos(pitch);
  float64_t s2 = sin(pitch);
  float64_t c3 = cos(yaw);
  float64_t s3 = sin(yaw);

  // Euler angles, right-handed, intrinsic, XYZ convention
  // (which means: rotate around body axes Z, Y', X'') 
  m[0u][0u] = c2 * c3;
  m[0u][1u] = c3 * s1 * s2 - c1 * s3;
  m[0u][2u] = s1 * s3 + c1 * c3 * s2;

  m[1u][0u] = c2 * s3;
  m[1u][1u] = c1 * c3 + s1 * s2 * s3;
  m[1u][2u] = c1 * s2 * s3 - c3 * s1;

  m[2u][0u] = -s2;
  m[2u][1u] = c2 * s1;
  m[2u][2u] = c1 * c2;
}
/*-----------------------------------------------------------------------------
 *      DCM_Compass_Heading : DCM Sensor fusion
 *                             
 *  Parameters: navstik_object_t *nav, dcm_object_t *dcm
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void DCM_Compass_Heading( navstik_object_t *nav, dcm_object_t *dcm )
{
  float64_t mag_x;
  float64_t mag_y;
  float64_t cos_roll;
  float64_t sin_roll;
  float64_t cos_pitch;
  float64_t sin_pitch;
  
  cos_roll = cos(dcm->roll);
  sin_roll = sin(dcm->roll);
  cos_pitch = cos(dcm->pitch);
  sin_pitch = sin(dcm->pitch);
  
  mag_x = nav->navstik_MAGX * cos_pitch + nav->navstik_MAGY * sin_roll * sin_pitch + nav->navstik_MAGZ * cos_roll * sin_pitch;    /* Tilt compensated magnetic field X */
  mag_y = nav->navstik_MAGY * cos_roll - nav->navstik_MAGZ * sin_roll;          /* Tilt compensated magnetic field Y */
  dcm->MAG_Heading = atan2(-mag_y, mag_x);                                      /* Magnetic Heading    */
}
/*-----------------------------------------------------------------------------
 *      DCM_reset_sensor_fusion : DCM Sensor fusion
 *                             
 *  Parameters: dcm_object_t *dcm, navstik_object_t *nav, navstickUseCalib_e *mode, char* inpStr
 *
 *  Return:     void
 *---------------------------------------------------------------------------- */
#ifdef outout
void DCM_reset_sensor_fusion( dcm_object_t *dcm, navstik_object_t *nav, navstickUseCalib_e *mode, char* inpStr ) 
{
  dVectr temp1;
  dVectr temp2;
  dVectr accel; 
  dVectr xAxis;

  NavStik_getAllValues( inpStr, nav, mode  );
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  dcm->pitch = -atan2(nav->navstik_ACCELX, sqrt(nav->navstik_ACCELY * nav->navstik_ACCELY + nav->navstik_ACCELZ * nav->navstik_ACCELZ));      // MATH FUNCTIONS USED ATAN2 ETC.
  
  // GET ROLL
  // Compensate pitch of gravity vector 
  accel = mkvec(nav->navstik_ACCELX, nav->navstik_ACCELY, nav->navstik_ACCELZ );
  xAxis = mkvec( 1.0f, 0.0f, 0.0f );
  temp1 = vcross(accel,xAxis);
  temp2 = vcross(xAxis,temp1);
  dcm->roll = atan2(temp2.y, temp2.z);
  
  // GET YAW
  DCM_Compass_Heading( nav, dcm );                 
  dcm->yaw = dcm->MAG_Heading;                  
  
  init_rotation_matrix(DCM_Matrix, dcm->yaw, dcm->pitch, dcm->roll);
}
#endif

/*-----------------------------------------------------------------------------
 *      Vector_Dot_Product : Computes the dot product of two vectors
 *                             
 *  Parameters: const float64_t v1[3], const float64_t v2[3]
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t Vector_Dot_Product(const float64_t v1[3], const float64_t v2[3])
{
  float64_t result = 0;
  int8_t c;
  
  for(c = 0; c < 3; c++)
  {
    result += v1[c] * v2[c];
  }
  
  return result; 
}
 
/*-----------------------------------------------------------------------------
 *      Vector_Cross_Product : Computes the cross product of two vectors
 *                             out has to different from v1 and v2 (no in-place)!
 *  Parameters: float64_t out[3], const float64_t v1[3], const float64_t v2[3]
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Vector_Cross_Product(float64_t out[3], const float64_t v1[3], const float64_t v2[3])
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
} 
/*-----------------------------------------------------------------------------
 *      Vector_Scale : Multiply the vector by a scalar
 *
 *  Parameters: float64_t out[3], const float64_t v[3], float64_t scale
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Vector_Scale(float64_t out[3], const float64_t v[3], float64_t scale)
{
  int8_t c;
  for(c = 0; c < 3; c++)
  {
    out[c] = v[c] * scale; 
  }
}

/*-----------------------------------------------------------------------------
 *      Vector_Add : Adds two vectors
 *
 *  Parameters: float64_t out[3], const float64_t v1[3], const float64_t v2[3]
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Vector_Add(float64_t out[3], const float64_t v1[3], const float64_t v2[3])
{
  int8_t c;
  for(c = 0; c < 3; c++)
  {
    out[c] = v1[c] + v2[c];
  }
}
 
/*-----------------------------------------------------------------------------
 *      Matrix_Multiply : Multiply two 3x3 matrices: out = a * b
 *                               out has to different from a and b (no in-place)!
 *
 *  Parameters: const float64_t a[3][3], const float64_t b[3][3], float64_t out[3][3]
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Matrix_Multiply(const float64_t a[3][3], const float64_t b[3][3], float64_t out[3][3])
{
  int8_t x,y;
  for(x = 0; x < 3; x++)  // rows
  {
    for(y = 0; y < 3; y++)  // columns
    {
      out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
    }
  }
}
 
/*-----------------------------------------------------------------------------
 *      Matrix_Vector_Multiply : Multiply 3x3 matrix with vector: out = a * b
 *                               out has to different from b (no in-place)!
 *
 *  Parameters: const float64_t a[3][3], const float64_t b[3], float64_t out[3]
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Matrix_Vector_Multiply(const float64_t a[3][3], const float64_t b[3], float64_t *out[3])
{
  int8_t x;
  for(x = 0; x < 3; x++)
  {
    *out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}
/*-----------------------------------------------------------------------------
 *      DCM_Drift_correction : DCM drift correction
 *
 *
 *  Parameters: const dcm_object_t *dcm
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void DCM_Drift_correction( const dcm_object_t *dcm )
{
  float64_t mag_heading_x;
  float64_t mag_heading_y;
  float64_t errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float64_t Scaled_Omega_P[3u];
  static float64_t Scaled_Omega_I[3u];
  float64_t Accel_magnitude;
  float64_t Accel_weight;
  
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0u]*Accel_Vector[0u] + Accel_Vector[1u]*Accel_Vector[1u] + Accel_Vector[2u]*Accel_Vector[2u]);
  Accel_magnitude = Accel_magnitude /GRAV_CONST;                                // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = CONSTRAIN(1.0f - 2.0f*(1.0f - Accel_magnitude),0.0f,1.0f);     //  

  Vector_Cross_Product(&errorRollPitch[0u],&Accel_Vector[0u],&DCM_Matrix[2u][0u]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0u],&errorRollPitch[0u],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0u],&errorRollPitch[0u],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(dcm->MAG_Heading);
  mag_heading_y = sin(dcm->MAG_Heading);
  errorCourse=(DCM_Matrix[0u][0u]*mag_heading_y) - (DCM_Matrix[1u][0u]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2u][0u],errorCourse);                       //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0u],&errorYaw[0u],Kp_YAW);                       //.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);                                   //Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0u],&errorYaw[0u],Ki_YAW);                       //.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);                                   //adding integrator to the Omega_I
}
/*-----------------------------------------------------------------------------
 *      DCM_Euler_angles : DCM euler angles
 *
 *
 *  Parameters: dcm_object_t *dcm
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void DCM_Euler_angles(dcm_object_t *dcm)
{
  dcm->pitch = -asin(DCM_Matrix[2u][0u]);
  dcm->roll = atan2(DCM_Matrix[2u][1u],DCM_Matrix[2u][2u]);
  dcm->yaw = atan2(DCM_Matrix[1u][0u],DCM_Matrix[0u][0u]);
}
/*-----------------------------------------------------------------------------
 *      DCM_Normalize : DCM normalize
 *
 *
 *  Parameters: void
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void DCM_Normalize(void)
{
  float64_t error=0.0f;
  float64_t temporary[3u][3u];
  float64_t renorm=0.0f;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0u][0u],&DCM_Matrix[1u][0u])*0.5f;     //eq.19

  Vector_Scale(&temporary[0u][0u], &DCM_Matrix[1u][0u], error);                 //eq.19
  Vector_Scale(&temporary[1u][0u], &DCM_Matrix[0u][0u], error);                 //eq.19
  
  Vector_Add(&temporary[0u][0u], &temporary[0u][0u], &DCM_Matrix[0u][0u]);      //eq.19
  Vector_Add(&temporary[1u][0u], &temporary[1u][0u], &DCM_Matrix[1u][0u]);      //eq.19
  
  Vector_Cross_Product(&temporary[2u][0u],&temporary[0u][0u],&temporary[1u][0u]); // c= a x b //eq.20
  
  renorm= 0.5f *(3.0f - Vector_Dot_Product(&temporary[0u][0u],&temporary[0u][0u])); //eq.21
  Vector_Scale(&DCM_Matrix[0u][0u], &temporary[0u][0u], renorm);
  
  renorm= 0.5f *(3.0f - Vector_Dot_Product(&temporary[1u][0u],&temporary[1u][0u])); //eq.21
  Vector_Scale(&DCM_Matrix[1u][0u], &temporary[1u][0u], renorm);
  
  renorm= 0.5f *(3.0f - Vector_Dot_Product(&temporary[2u][0u],&temporary[2u][0u])); //eq.21
  Vector_Scale(&DCM_Matrix[2u][0u], &temporary[2u][0u], renorm);
}
/*-----------------------------------------------------------------------------
 *      DCM_Matrix_update : DCM matrix update 
 *
 *
 *  Parameters: navstik_object_t *nav, dcm_object_t *dcm
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void DCM_Matrix_update(navstik_object_t *nav, dcm_object_t *dcm)
{
  int8_t x,y;
  Gyro_Vector[0u]=DCM_GYRO_SCALED_RAD(nav->navstik_GYROX);                      //gyro x roll
  Gyro_Vector[1u]=DCM_GYRO_SCALED_RAD(nav->navstik_GYROY);                      //gyro y pitch
  Gyro_Vector[2u]=DCM_GYRO_SCALED_RAD(nav->navstik_GYROZ);                      //gyro z yaw
  
  Accel_Vector[0u]=nav->navstik_ACCELX;
  Accel_Vector[1u]=nav->navstik_ACCELY;
  Accel_Vector[2u]=nav->navstik_ACCELZ;
    
  Vector_Add(&Omega[0u], &Gyro_Vector[0u], &Omega_I[0u]);                       //adding proportional term
  Vector_Add(&Omega_Vector[0u], &Omega[0u], &Omega_P[0u]);                      //adding Integrator term
  
  if (dcm->timeInitDone == 0u)                                                  /* first time round */
  {
     dcm->lastTick = CP0_GET(CP0_COUNT);
     dcm->G_Dt = 0u;                                                            /* no drift correction */
     dcm->timeInitDone = 1u;
  }
  else
  {
     calculateTickDiff(&dcm->G_Dt, &dcm->lastTick);
  }                                  
  // Use drift correction
  Update_Matrix[0u][0u]=0;
  Update_Matrix[0u][1u]=-TICKS2SEC(dcm->G_Dt)*Omega_Vector[2u];                 //-z
  Update_Matrix[0u][2u]=TICKS2SEC(dcm->G_Dt)*Omega_Vector[1u];                  //y
  Update_Matrix[1u][0u]=TICKS2SEC(dcm->G_Dt)*Omega_Vector[2u];                  //z
  Update_Matrix[1u][1u]=0;
  Update_Matrix[1u][2u]=-TICKS2SEC(dcm->G_Dt)*Omega_Vector[0u];                 //-xw
  Update_Matrix[2u][0u]=-TICKS2SEC(dcm->G_Dt)*Omega_Vector[1u];                 //-y
  Update_Matrix[2u][1u]=TICKS2SEC(dcm->G_Dt)*Omega_Vector[0u];                  //x
  Update_Matrix[2u][2u]=0;

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix);                   //a*b=c
  for(x=0; x<3; x++)                                                            //Matrix Addition (update)
  {
    for(y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}
/*-----------------------------------------------------------------------------
 *      DCM_Algo_go : DCM ALgoryhtm 
 *
 *
 *  Parameters: char* inpStr, navstik_object_t *nav, dcm_object_t *dcm, navstickUseCalib_e *mode
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void DCM_Algo_go( char* inpStr, navstik_object_t *nav, dcm_object_t *dcm, navstickUseCalib_e *mode )
{
      NavStik_getAllValues( inpStr, nav, mode  );                               /* we are using the navstik for these in this example */

      DCM_Compass_Heading( nav, dcm );                                          // Run DCM algorithm
      DCM_Matrix_update( nav, dcm );
      DCM_Normalize();
      DCM_Drift_correction( dcm );
      DCM_Euler_angles( dcm );
}
// from James Alain Preiss
/*-----------------------------------------------------------------------------
 *      veltmul : element-wise vector multiply. 
 *
 *
 *  Parameters: Vectr a, Vectr b
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr veltmul(Vectr a, Vectr b) 
{
   return mkvec(a.x * b.x, a.y * b.y, a.z * b.z);
}
 
/*-----------------------------------------------------------------------------
 *      veltdiv : element-wise vector divide.
 *
 *
 *  Parameters: Vectr a, Vectr b
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr veltdiv(Vectr a, Vectr b) 
{
   return mkvec(a.x / b.x, a.y / b.y, a.z / b.z);
}
/*
%QUATERNPROD Calculates the quaternion product (from JUSTA)
%
%   ab = quaternProd(a, b)
%
%   Calculates the quaternion product of quaternion a and b.
%
%   For more information see:
%   http://www.x-io.co.uk/node/8#quaternions
%
%        Date          Author          Notes
%        27/09/2011    SOH Madgwick    Initial release
%      ab=[1,0,0,0];
%
%      Converted from matlab to C by ACP Aviation 
*/
quat quaternProd( quat a, quat b )
{
   quat ret = {0.0f, 0.0f, 0.0f, 0.0f };
   
   ret.x = a.x*b.x-a.y*b.y-a.z*b.z-a.w*b.w;
   ret.y = a.x*b.y+a.y*b.x+a.z*b.w-a.w*b.z;
   ret.z = a.x*b.z-a.y*b.w+a.z*b.x+a.w*b.y;
   ret.w = a.x*b.w+a.y*b.z-a.z*b.y+a.w*b.x;
   
   return ret;
}
/*-----------------------------------------------------------------------------
 *      quaternConj : quaternion conjugate 
 *
 *
 *  Parameters: quat q
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat quaternConj( quat q )
{
   return (mkquat(q.x,-q.y,-q.z,-q.w));
}
/*-----------------------------------------------------------------------------
 *      mulquat : multiply 2 quaternion 
 *
 *
 *  Parameters: quat a, float64_t b
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat mulquat(quat a, float64_t b)
{
   quat q;
   q.x = b*a.x;
   q.y = b*a.y;
   q.z = b*a.z;
   q.w = b*q.w;
   return q;
}
/*-----------------------------------------------------------------------------
 *      addquat : add 2 quaternion 
 *
 *
 *  Parameters: quat q, quat q1
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat addquat( quat q, quat q1 )
{
  return (mkquat(q.x+q1.x, q.y+q1.y, q.z+q1.z, q.w+q1.w));
}
/*-----------------------------------------------------------------------------
 *      divquat : divide quaternion by a scalar
 *
 *
 *  Parameters: quat a, float64_t div
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat divquat(quat a, float64_t div)
{
   quat q;
   q.x = q.x / div;
   q.y = q.y / div;
   q.z = q.z / div;
   q.w = q.w / div;   
   return q;
}
/*-----------------------------------------------------------------------------
 *      qnorm : normlaise a quarternion to scalar
 *
 *
 *  Parameters: quat quatV
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t qnorm( quat quatV )
{
  return (sqrt(quatV.x * quatV.x + quatV.y * quatV.y + quatV.z * quatV.z + quatV.w * quatV.w));
}
/*-----------------------------------------------------------------------------
 *      vnormVec : normlaise vector to scalar
 *
 *
 *  Parameters: Vectr quatV 
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t vnormVec( Vectr quatV )
{
  return (sqrt(quatV.x * quatV.x + quatV.y * quatV.y + quatV.z * quatV.z));
}
/* 
%   JUSTA, Josef; ŠMÍDL, Václav; HAMÁÈEK, Aleš. Fast AHRS Filter for Accelerometer, Magnetometer, 
%   and Gyroscope Combination with Separated Sensor Corrections. Sensors, 2020, 20.14: 3824.
%
%   Date          Author          Notes
%   30/5/2020     Josef Justa     Initial release
*/
/*-----------------------------------------------------------------------------
 *      JustaAHRSPureUpdate : Josef Justa algorythm for AHRS as above paper and matlab code
 *
 *
 *  Parameters: JustaAHRSPure_t *obj, Vectr Gyroscope, Vectr Accelerometer, Vectr Magnetometer 
 *
 *  Return:     JustaAHRSPure_t *
 *----------------------------------------------------------------------------*/
JustaAHRSPure_t * JustaAHRSPureUpdate( JustaAHRSPure_t *obj, Vectr Gyroscope, Vectr Accelerometer, Vectr Magnetometer )
{
   quat q,g; 
   float64_t acc;
   float64_t mag;
   Vectr Vacc;
   Vectr Vmag;
   Vectr wt;
   quat qDot;
   quat quatGyrPred,qp;
   float64_t qDotx,qDoty,qDotz,qDotw;                                           /* left rather than just using the quat as its shifted when making qdot */
   mat33 R;
   Vectr ar = {0.0f, 0.0f, 1.0f};
   Vectr accMesPred,magMesPred;
   float32_t mr_z,mr_x;
   Vectr mr,ca,vecA,cm,vecB,im,im2;
   quat qCor;
   quat quat1;
   quat h,qMagTmp;
   float64_t phia,phim;
   Vectr vTmp;
   
   if (obj != NULL)
   {
      q = mkquat( obj->Quaternion.x, obj->Quaternion.y, obj->Quaternion.z, obj->Quaternion.w );  /* short name local variable for readability */  
      g = mkquat( 0.0f, Gyroscope.x, Gyroscope.y, Gyroscope.z );                                      
      acc = sqrt(Accelerometer.x * Accelerometer.x + Accelerometer.y * Accelerometer.y + Accelerometer.z * Accelerometer.z);                                                               /* Normalise accelerometer measurement */                                                
      if (acc != 0.0f) 
      {
         Vacc = vdiv( Accelerometer, acc );
         mag = sqrt(Magnetometer.x * Magnetometer.x + Magnetometer.y * Magnetometer.y + Magnetometer.z * Magnetometer.z);   /* Normalise magnetometer measurement */                                                  
         if (mag != 0.0f) 
         {
            Vmag = vdiv( Magnetometer, mag );
            qDot = quaternProd(q, g);                                           /* ? his code seems to overwrite this */
            qDot = mulquat(qDot,0.5f);
            qp =  addquat(q,qDot);
            if (obj->option == _PURE_JUSTA)
            { 
              wt =  vdiv(vscl(obj->SamplePeriod,Gyroscope),2.0f);
/*
%             qDotw=cos(wt(1))*cos(wt(2))*cos(wt(3))-sin(wt(1))*sin(wt(2))*sin(wt(3));  what was commented out
%             qDotx=cos(wt(1))*sin(wt(2))*sin(wt(3))+sin(wt(1))*cos(wt(2))*cos(wt(3));
%             qDoty=sin(wt(1))*cos(wt(2))*sin(wt(3))+cos(wt(1))*sin(wt(2))*cos(wt(3));
%             qDotz=sin(wt(1))*sin(wt(2))*cos(wt(3))+cos(wt(1))*cos(wt(2))*sin(wt(3));
%             qDotw=cos(wt(1))*cos(wt(2))*cos(wt(3))-sin(wt(1))*sin(wt(2))*sin(wt(3));
*/       
              qDotx=sin(wt.x);
              qDoty=sin(wt.y);
              qDotz=sin(wt.z);
              qDotw=sqrt(1.0f-(qDotx*qDotx + qDoty*qDoty + qDotz*qDotz)); 
              qDot = mkquat(qDotw, qDotx, qDoty, qDotz);                        /* alters order here */
              quatGyrPred = quaternProd(q, qDot);                               /* q + qDot*obj.SamplePeriod;   commented out by justa */
              qp = mkquat(quatGyrPred.x, quatGyrPred.y, quatGyrPred.z, quatGyrPred.w);
            }
            else
            {
               qp =  addquat(q,qDot);
               if (obj->option == _PURE_FAST_JUSTA) qp = mkquat(qp.x/qnorm(qp),qp.y/qnorm(qp),qp.z/qnorm(qp),qp.w/qnorm(qp));
            }
            R.m[0u][0u] = 2.0f*(0.5f - (qp.z*qp.z) - (qp.w*qp.w));
            R.m[0u][1u] = 0.0f;
            R.m[0u][2u] = 2.0f*((qp.y*qp.w) - (qp.x*qp.z));
            R.m[1u][0u] = 2.0f*((qp.y*qp.z) - (qp.x*qp.w));
            R.m[1u][1u] = 0.0f;
            R.m[1u][2u] = 2.0f*((qp.x*qp.y) + (qp.z*qp.w));
            R.m[2u][0u] = 2.0f*((qp.x*qp.z) + (qp.y*qp.w));
            R.m[2u][1u] = 0.0f;
            R.m[2u][2u] = 2.0f*(0.5f - ((qp.y*qp.y) - (qp.z*qp.z)));  
            accMesPred = mvmul(R,ar);                                           // NB : conjugate transpose ' ?? how is product vector ? accMesPred=(R*ar')';
            /*             alf=0.01;   */
            /*             obj.mr_z= (1-alf)*obj.mr_z + alf*dot(accMesPred,mag);MesPred   */
            
            if (obj->option == _PURE_FAST_JUSTA_CONST_CORR)
            {
                 obj->mr_z = vdot(accMesPred,Vmag);
                 mr_x = sqrt(1.0f-(obj->mr_z*obj->mr_z));
                 mr = mkvec(mr_x, 0.0f, obj->mr_z);
            }
            else if ((obj->option == _PURE_FAST_JUSTA) || (obj->option == _PURE_FAST_JUSTA_LINEAR))
            {
                 qMagTmp = mkquat(0.0f, Vmag.x, Vmag.y, Vmag.z);
                 h = quaternProd(q, quaternProd(qMagTmp, quaternConj(q)));
                 mr = vdiv(mkvec(sqrt(h.y*h.y + h.z*h.z), 0.0f, h.w), vnormVec(mkvec(sqrt(h.y*h.y + h.z*h.z), 0.0f, h.w)));
            }
            else
            {            
               mr_z = vdot(accMesPred,Vmag);
               mr_x = sqrt(1.0f-(mr_z*mr_z));
               mr = mkvec(mr_x, 0.0f, mr_z);
            }
            
            magMesPred = mvmul(R,mr);                                           // NB: conjugate transpose  magMesPred=(R*mr')';
            ca = vcross(Vacc,accMesPred);                                       // ca = cross([0 0 1], accRefPred)/norm(cross([0 0 1], accRefPred))*obj.wAcc;
            vecA = vdiv(ca , sqrt((ca.x*ca.x) + ((ca.y*ca.y) + (ca.z*ca.z))));
            cm = vcross(Vmag,magMesPred);
            vecB = vdiv(cm , sqrt((cm.x*cm.x) + ((cm.y*cm.y) + (cm.z*cm.z))));

            if (obj->option >= _PURE_FAST_JUSTA)                                /* FAST options */
            {
               phia=(asin(vnormVec(ca))*obj->gain);
               if (phia > obj->wAcc) phia = obj->wAcc;
               phim = (asin(vnormVec(cm))*obj->gain); 
               if (phim > obj->wMag) phim = obj->wMag;
               vTmp = vadd(vdiv(vscl(phia,vecA),2.0f),vdiv(vscl(phim,vecB),2.0f));   
               qCor = mkquat(1.0f, vTmp.x, vTmp.y, vTmp.z); 
               obj->test = mkquat(asin(sqrt((ca.x*ca.x) + ((ca.y*ca.y) + (ca.z*ca.z)))),asin(sqrt((cm.x*cm.x) + ((cm.y*cm.y) + (cm.z*cm.z)))),0.0f,0.0f);
               quat1=quaternProd(qp,qCor);         
            }
            else                                                                /* PURE_JUSTA or LINEAR */
            {
               im = vadd(vscl(obj->wAcc/2.0f,vecA), vscl(obj->wMag/2.0f,vecB));
               im2 = vscl(sin(sqrt((im.x*im.x) + ((im.y*im.y) + (im.z*im.z)))/PI),im);     // NB sinc ? im2=im*sinc(norm(im)/pi);
               qCor = mkquat(sqrt(1.0f-((im2.x*im2.x)+((im2.y*im2.y)+(im2.z+im2.z)))), im2.x, im2.y, im2.z);            
               quat1 = quaternProd(quatGyrPred,qCor);
            } 
            
            if (quat1.y<0.0f)
            {
               quat1.x = -quat1.x;  
               quat1.y = -quat1.y; 
               quat1.z = -quat1.z;  
               quat1.w = -quat1.w; 
            }
            obj->Quaternion.x = quat1.x / sqrt(quat1.x * quat1.x + quat1.y * quat1.y + quat1.z * quat1.z + quat1.w * quat1.w);   
            obj->Quaternion.y = quat1.y / sqrt(quat1.x * quat1.x + quat1.y * quat1.y + quat1.z * quat1.z + quat1.w * quat1.w);   
            obj->Quaternion.z = quat1.z / sqrt(quat1.x * quat1.x + quat1.y * quat1.y + quat1.z * quat1.z + quat1.w * quat1.w);   
            obj->Quaternion.w = quat1.w / sqrt(quat1.x * quat1.x + quat1.y * quat1.y + quat1.z * quat1.z + quat1.w * quat1.w);   
            /* mr = qnorm(qMagTmp)/qnorm(qMagTmp);  */              
         }
       }
   }  
   return obj; 
}

/*
% Suh, Young.Soo. Simple-Structured Quaternion Estimator Separating Inertial and Magnetic Sensor Effects. 
% IEEE, Transactions on Aerospace and Electronic Systems 2019, 55, 2698–2706.
%
% Implemented by Justa ported to mikroE C by ACP Aviation
*/
/*-----------------------------------------------------------------------------
 *      YoungSooSuhUpdate : above named alorythm for AHRS correction
 *
 *
 *  Parameters: SuhSooYoung_t *obj, Vectr Gyroscope, Vectr Accelerometer, Vectr Magnetometer
 *
 *  Return:     SuhSooYoung_t *
 *----------------------------------------------------------------------------*/
SuhSooYoung_t * YoungSooSuhUpdate( SuhSooYoung_t *obj, Vectr Gyroscope, Vectr Accelerometer, Vectr Magnetometer )
{
   quat q,gyr,q_dif_est,q_est,aRot,mRot,normQ;
   float64_t g = 1.0f;
   float64_t acc,mag,cosAccMag,sinAccMag,sinDipAng,cosDipAng,ak_minus,bk_minus,z2;
   Vectr Vacc,Vmag,z1;
   mat33 K,H;
        
  if (obj != NULL)
  {
     q = mkquat( obj->Quaternion.x, obj->Quaternion.y, obj->Quaternion.z, obj->Quaternion.w );  /* short name local variable for readability */ 
     gyr = mkquat( 0.0f, Gyroscope.x, Gyroscope.y, Gyroscope.z ); 
     q_dif_est = mulquat(quaternProd(q,gyr),0.5f); 
     q_est = addquat(q, mulquat(q_dif_est,obj->SamplePeriod));  
     acc = sqrt(Accelerometer.x * Accelerometer.x + Accelerometer.y * Accelerometer.y + Accelerometer.z * Accelerometer.z);                                                               /* Normalise accelerometer measurement */                                                
     if (acc != 0.0f) 
     {
         Vacc = vdiv( Accelerometer, acc );
         mag = sqrt(Magnetometer.x * Magnetometer.x + Magnetometer.y * Magnetometer.y + Magnetometer.z * Magnetometer.z);   /* Normalise magnetometer measurement */                                                  
         if (mag != 0.0f) 
         {
            Vmag = vdiv( Magnetometer, mag ); 
            cosAccMag = vdot(Vacc,Magnetometer);                                  // sind(-50);
            sinAccMag = sqrt(1.0f-(cosAccMag*cosAccMag));
            sinDipAng = cosAccMag;
            cosDipAng = sinAccMag;
            
            ak_minus = obj->alf+obj->rg*pow(obj->SamplePeriod,2.0f)/4.0f;
            bk_minus = obj->bet+obj->rm*pow(obj->SamplePeriod,2.0f)/4.0f;
            
            obj->kAlf = 2.0f*ak_minus*g/(4.0f*ak_minus*pow(g,2.0f)+obj->ra);
            obj->kBet = -2.0f*(bk_minus*cosDipAng-4.0f*obj->gam*sinDipAng)/(4.0f*ak_minus*pow(sinDipAng,2.0f)+4.0f*bk_minus*pow(cosDipAng,2.0f)-8.0f*obj->gam*cosDipAng*sinDipAng+obj->rm);
            obj->alf = (4.0f*ak_minus*pow(g,2.0f)+obj->ra)*pow(obj->kAlf,2.0f)-4.0f*ak_minus*g*obj->kAlf+ak_minus;
            obj->bet = (4.0f*ak_minus*pow(sinDipAng,2.0f)+4.0f*bk_minus*pow(cosDipAng,2.0f)+obj->rm-8.0f*obj->gam*cosDipAng*sinDipAng)*pow(obj->kBet,2.0f) +4.0f*(bk_minus*cosDipAng-4.0f*obj->gam*sinDipAng)*obj->kBet+bk_minus;    
            obj->gam = -(2.0f*g*obj->kAlf-1.0f)*(obj->gam+2.0f*(obj->gam*cosDipAng-ak_minus*sinDipAng)*obj->kBet);

            aRot = quaternProd(q_est,quaternProd(mkquat(0.0f, Vacc.x, Vacc.y, Vacc.z),quaternConj(q_est)));
            z1 = vsub(mkvec(aRot.y,aRot.z,aRot.w),mkvec(0.0f,0.0f,g));
            // z1=z1(1:2)'; TODO ? conjugate transpose of a vector not a 3x3 matrix (z1.x -z1.y) ??
            mRot = quaternProd(q_est,quaternProd(mkquat(0.0f, Vmag.x, Vmag.y, Vmag.z),quaternConj(q_est)));
            z2 = mRot.z;
            K.m[0u][0u] = 0.0f;
            K.m[0u][1u] = obj->kAlf;
            K.m[0u][2u] = 0.0f; 
            K.m[1u][0u] = -obj->kAlf;  
            K.m[1u][1u] = 0.0f;
            K.m[1u][2u] = 0.0f;   
            K.m[2u][0u] = 0.0f;  
            K.m[2u][1u] = 0.0f;
            K.m[2u][2u] = obj->kBet;
            H.m[0u][0u] = 0.0f;
            H.m[0u][1u] = -2.0f;
            H.m[0u][2u] = 0.0f; 
            H.m[1u][0u] = 2.0f;  
            H.m[1u][1u] = 0.0f;
            H.m[1u][2u] = 0.0f;   
            H.m[2u][0u] = sinDipAng*2.0f;  
            H.m[2u][1u] = 0.0f;
            H.m[2u][2u] = -cosDipAng*2.0f; 
            
            obj->q_err = vadd(obj->q_err, mvmul(K, vsub(mkvec(z1.x,z1.y,z2), mcolumn(mtranspose(mscl(obj->q_err.x, H)), 1.0f))));  //  obj.q_err=obj.q_err+(K*([z1;z2]-H*obj.q_err'))';
            // alternate :: obj->q_err = vadd(obj->q_err, mvmul(K, vsub(mkvec(z1.x,z1.y,z2), mvmul(H, obj->q_err))));
            normQ = mkquat(1.0f, obj->q_err.x, obj->q_err.y, obj->q_err.z);
            normQ = divquat(normQ, qnorm(normQ));    
            q = quaternProd(normQ, q_est); 
            obj->Quaternion = divquat(q , qnorm(q));               
         }
     }            
  }
  else { /* for misra */ }
  return obj;
}
/*
%Fast Kalman Filter for Attitude Estimation
%author: Jin Wu
%e-mail: jin_wu_uestc@hotmail.com
ported for c by ACP Aviation
*/
/*-----------------------------------------------------------------------------
 *      jaCob2Zero : construct a zero matrix jacobian
 *
 *
 *  Parameters: JinWu_t *jin
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void jaCob2Zero( JinWu_t *jin ) 
{
        int8_t i,j;
        for (i = 0; i < 4; ++i) 
        {
           for (j = 0; j < 6; ++j) 
           {
               jin->jaCob[i][j] = 0.0f;
           }
        }
}
/*-----------------------------------------------------------------------------
 *      JINWU_measurement_quaternion_acc_mag : jin wu compensation
 *
 *
 *  Parameters: dVectr acc, dVectr mag, JinWu_t *jin
 *
 *  Return:     JinWu_t *
 *----------------------------------------------------------------------------*/
JinWu_t * JINWU_measurement_quaternion_acc_mag( dVectr acc, dVectr mag, JinWu_t *jin)
{
   float64_t ax,ay,az,mx,my,mz,mN,mD,q0,q1,q2,q3;
   
   ax=acc.x;  ay=acc.y;  az=acc.z;
   mx=mag.x;  my=mag.y;  mz=mag.z;
   mN=jin->mag_r.x; mD=jin->mag_r.z;

   q0=jin->q_.x;   q1=jin->q_.y;   q2=jin->q_.z;   q3=jin->q_.w;
   jin->q.x = 0.0f; jin->q.y = 0.0f; jin->q.z = 0.0f; jin->q.w = 0.0f;
   jaCob2Zero( jin ); 

   jin->q.x = (ay*mD*my + (1.0f + az)*(1.0f + mN*mx + mD*mz) + ax*(mD*mx - mN*mz))*q0 + ((mD + az*mD - ax*mN)*my + ay*(1.0f + mN*mx - mD*mz))*q1 + (ay*mN*my + ax*(-1.0f + mN*mx + mD*mz) + (1.0f + az)*(-(mD*mx) + mN*mz))*q2 + (-((ax*mD + mN + az*mN)*my) + ay*(mD*mx + mN*mz))*q3;   
   jin->q.y = ((mD - az*mD - ax*mN)*my + ay*(1.0f + mN*mx + mD*mz))*q0 + (ay*mD*my - (-1.0f + az)*(1.0f + mN*mx - mD*mz) + ax*(mD*mx + mN*mz))*q1 + ((ax*mD + mN - az*mN)*my + ay*(-(mD*mx) + mN*mz))*q2 + (-(ay*mN*my) + ax*(1.0f - mN*mx + mD*mz) - (-1.0f + az)*(mD*mx + mN*mz))*q3;   
   jin->q.z = (-(ay*mN*my) - ax*(1.0f + mN*mx + mD*mz) + (-1.0f + az)*(mD*mx - mN*mz))*q0 + ((-(ax*mD) + mN - az*mN)*my + ay*(mD*mx + mN*mz))*q1 + (ay*mD*my + (-1.0f + az)*(-1.0f + mN*mx + mD*mz) + ax*(mD*mx - mN*mz))*q2 + ((mD - az*mD + ax*mN)*my + ay*(1.0f - mN*mx + mD*mz))*q3;
   jin->q.w = ax*(q1 + mN*mx*q1 + mN*my*q2 + mN*mz*q3 + mD*(my*q0 - mz*q1 + mx*q3)) + (1.0f + az)*(mD*mx*q1 + mD*my*q2 + q3 + mD*mz*q3 - mN*(my*q0 - mz*q1 + mx*q3)) + ay*(mN*mz*q0 + mN*my*q1 + q2 - mN*mx*q2 - mD*(mx*q0 + mz*q2 - my*q3));

   jin->jaCob[0u][0u] = -q2 - mN*(mz*q0 + my*q1 - mx*q2) + mD*(mx*q0 + mz*q2 - my*q3);
   jin->jaCob[0u][1u] = q1 + mN*mx*q1 + mN*my*q2 + mN*mz*q3 + mD*(my*q0 - mz*q1 + mx*q3);
   jin->jaCob[0u][2u] = q0 + mN*mx*q0 + mD*mz*q0 + mD*my*q1 - mD*mx*q2 + mN*mz*q2 - mN*my*q3;
   jin->jaCob[0u][3u] = (ax*mD + mN + az*mN)*q0 + ay*mN*q1 + (-((1 + az)*mD) + ax*mN)*q2 + ay*mD*q3;
   jin->jaCob[0u][4u] = ay*mD*q0 + (mD + az*mD - ax*mN)*q1 + ay*mN*q2 - (ax*mD + mN + az*mN)*q3;
   jin->jaCob[0u][5u] = mD*(q0 + az*q0 - ay*q1 + ax*q2) + mN*(-(ax*q0) + q2 + az*q2 + ay*q3);
   
   jin->jaCob[1u][0u] = q3 - mN*(my*q0 - mz*q1 + mx*q3) + mD*(mx*q1 + my*q2 + mz*q3);
   jin->jaCob[1u][1u] = q0 + mN*mx*q0 + mD*mz*q0 + mD*my*q1 - mD*mx*q2 + mN*mz*q2 - mN*my*q3;
   jin->jaCob[1u][2u] = -((1 + mN*mx)*q1) - mD*(my*q0 - mz*q1 + mx*q3) - mN*(my*q2 + mz*q3);
   jin->jaCob[1u][3u] = ay*(mN*q0 - mD*q2) - (-1 + az)*(mN*q1 + mD*q3) + ax*(mD*q1 - mN*q3);
   jin->jaCob[1u][4u] = mD*(q0 - az*q0 + ay*q1 + ax*q2) - mN*(ax*q0 + (-1 + az)*q2 + ay*q3);
   jin->jaCob[1u][5u] = ay*(mD*q0 + mN*q2) + mD*((-1 + az)*q1 + ax*q3) + mN*(ax*q1 + q3 - az*q3);

   jin->jaCob[2u][0u] = -((1 + mN*mx + mD*mz)*q0) - mD*my*q1 + mD*mx*q2 - mN*mz*q2 + mN*my*q3;
   jin->jaCob[2u][1u] = q3 - mN*(my*q0 - mz*q1 + mx*q3) + mD*(mx*q1 + my*q2 + mz*q3);
   jin->jaCob[2u][2u] = -q2 - mN*(mz*q0 + my*q1 - mx*q2) + mD*(mx*q0 + mz*q2 - my*q3);
   jin->jaCob[2u][3u] = mD*((-1 + az)*q0 + ay*q1 + ax*q2) - mN*(ax*q0 + q2 - az*q2 + ay*q3);
   jin->jaCob[2u][4u] = ay*(-(mN*q0) + mD*q2) - (-1 + az)*(mN*q1 + mD*q3) + ax*(-(mD*q1) + mN*q3);
   jin->jaCob[2u][5u] = mN*(q0 - az*q0 + ay*q1) - ax*(mD*q0 + mN*q2) + mD*((-1 + az)*q2 + ay*q3);

   jin->jaCob[3u][0u] = q1 + mN*mx*q1 + mN*my*q2 + mN*mz*q3 + mD*(my*q0 - mz*q1 + mx*q3);
   jin->jaCob[3u][1u] = q2 + mN*(mz*q0 + my*q1 - mx*q2) - mD*(mx*q0 + mz*q2 - my*q3);
   jin->jaCob[3u][2u] = q3 - mN*(my*q0 - mz*q1 + mx*q3) + mD*(mx*q1 + my*q2 + mz*q3);
   jin->jaCob[3u][3u] = -(ay*(mD*q0 + mN*q2)) + ax*(mN*q1 + mD*q3) + (1 + az)*(mD*q1 - mN*q3);
   jin->jaCob[3u][4u] = (1 + az)*(-(mN*q0) + mD*q2) + ax*(mD*q0 + mN*q2) + ay*(mN*q1 + mD*q3);
   jin->jaCob[3u][5u] = ay*(mN*q0 - mD*q2) + (1 + az)*(mN*q1 + mD*q3) + ax*(-(mD*q1) + mN*q3);
   return jin;
}
#if defined(TF_DISTORT)
/*

Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
/*-----------------------------------------------------------------------------
 *      qqmul : multiply 2 quaternion, such that qvrot(qqmul(q, p), v) == qvrot(q, qvrot(p, v)).
 *
 *
 *  Parameters: quat q, quat p
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat qqmul(quat q, quat p) 
{
        quat qRet;
        qRet.x =  q.w*p.x + q.z*p.y - q.y*p.z + q.x*p.w;
        qRet.y = -q.z*p.x + q.w*p.y + q.x*p.z + q.y*p.w;
        qRet.z =  q.y*p.x - q.x*p.y + q.w*p.z + q.z*p.w;
        qRet.w = -q.x*p.x - q.y*p.y - q.z*p.z + q.w*p.w;
        return qRet;
}
/*-----------------------------------------------------------------------------
 *      rs2_quaternion_multiply : multiply 2 quaternion, as used in realsense
 *
 *
 *  Parameters: quat q, quat p
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat rs2_quaternion_multiply(quat a, quat b)
{
    return mkquat((a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z), (a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z),(a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z), (a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z) );
}
/*-----------------------------------------------------------------------------
 *      TfDistort_addNoise : do distortion 
 *
 *
 *  Parameters: StampedTransform_t *tf, const distort_config_t config_
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void TfDistort_addNoise(StampedTransform_t *tf, const distort_config_t config_)
{
  const float64_t deg2rad = PI / 180.0f;
  float64_t dt;
  int32_t getTickCount;
  float64_t noise_x = 0.0f, noise_y = 0.0f, noise_z = 0.0f, noise_roll = 0.0f, noise_pitch = 0.0f, noise_yaw = 0.0f;
  quat q;
  
  if (!tf->once)
  {
    whiteGaussInitSeed( &tf->storedSeed );  
    tf->once = true;
    tf->last_time = CP0_GET(CP0_COUNT);
    return;
  }

  calculateTickDiff(&getTickCount,&tf->last_time);
  dt = TICKS2SEC(getTickCount);
  
  if (config_.noise_type == vicon_distort_NORMAL)
  {
    whiteGaussianNoise(&noise_roll, &noise_pitch);
    whiteGaussianNoise(&noise_yaw, &noise_x);
    whiteGaussianNoise(&noise_y, &noise_z);

    noise_roll *= config_.sigma_roll_pitch * deg2rad;
    noise_pitch *= config_.sigma_roll_pitch * deg2rad;
    noise_yaw *= config_.sigma_yaw * deg2rad;

    noise_x *= config_.sigma_xy;
    noise_y *= config_.sigma_xy;
    noise_z *= config_.sigma_z;
  }
  else if (config_.noise_type == vicon_distort_UNIFORM)
  {
    noise_roll = uniformNoise(config_.sigma_roll_pitch * deg2rad);
    noise_pitch = uniformNoise(config_.sigma_roll_pitch * deg2rad);
    noise_yaw = uniformNoise(config_.sigma_yaw * deg2rad);
    noise_x = uniformNoise(config_.sigma_xy);
    noise_y = uniformNoise(config_.sigma_xy);
    noise_z = uniformNoise(config_.sigma_z);
  }

  noise_x += Vicon_update( dt, config_.random_walk_tau_xy ) * config_.random_walk_k_xy;
  noise_y += Vicon_update( dt, config_.random_walk_tau_xy ) * config_.random_walk_k_xy;
  noise_z += Vicon_update( dt, config_.random_walk_tau_z ) * config_.random_walk_k_z;

  tf->p.x = tf->p.x + noise_x;
  tf->p.y = tf->p.y + noise_y;
  tf->p.z = tf->p.z + noise_z;

  //p *= config_.position_scale;
  tf->p = vscl(config_.position_scale,tf->p);

  q = mkquat(noise_roll, noise_pitch, noise_yaw, 0.0f);
  tf->orient = qqmul(tf->orient,q);

  tf->last_time = CP0_GET(CP0_COUNT);
}
#endif /* -- tf distort -- */

// Zup : +X forward, +Y left, and +Z up Right Handed  (Forward, Left, Up ) 
// Yup : +X forward, +Y up, and +Z left Y-up : SetAxisMapping( Forward, Up, Right )
// Specify the direction of your X, Y, and Z axis relative to yourself as the observer
/* 
   Remaps the 3D axis. 
   Vicon Data uses a right handed co-ordinate system, with +X forward, +Y left, 
   and +Z up. Other systems use different coordinate systems. The function can transform 
   its data into any valid right-handed co-ordinate system by re-mapping each axis.  
   Specify the direction of your X, Y, and Z axis relative to yourself as the observer. 
   Valid directions are "Up", "Down", "Left", "Right", "Forward", and "Backward". 
   Note that "Forward" means moving away from you, and "Backward" is moving towards you.
*/ 
/*-----------------------------------------------------------------------------
 *      swapLeftRightCoord : swap Zup Yup formats example for Vicon
 *
 *
 *  Parameters: Vectr v
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr swapLeftRightCoord( Vectr v )
{
  return mkvec( v.x, v.z, v.y);
}

/* from cleanflight betterflight for correction using COG from GPS */
// 
/*-----------------------------------------------------------------------------
 *      applyVectorError : Rotate mag error vector back to BF and accumulate
 *
 *
 *  Parameters: float32_t ez_ef, quat *vError, const mat33 qpAttitude
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void applyVectorError(float32_t ez_ef, quat *vError, const mat33 qpAttitude )
{

    vError->x += (2.0f * (qpAttitude.m[0u][2u] + -qpAttitude.m[3u][1u])) * ez_ef;
    vError->y += (2.0f * (qpAttitude.m[1u][2u] - -qpAttitude.m[3u][0u])) * ez_ef;
    vError->z += (1.0f - 2.0f * qpAttitude.m[0u][0u] - 2.0f * qpAttitude.m[1u][1u]) * ez_ef;
}
/*-----------------------------------------------------------------------------
 *  quaternionNormalize : normalise quatenion
 *
 *
 *  Parameters: quat *vAcc
 *
 *  Return:     Void
 *----------------------------------------------------------------------------*/
void quaternionNormalize(quat *vAcc)
{
   float32_t norm = sqrt((vAcc->x*vAcc->x)+(vAcc->y*vAcc->y)+(vAcc->z*vAcc->z)+(vAcc->w*vAcc->w));
   if (norm == 0) return;
   vAcc->x = vAcc->x / norm;
   vAcc->y = vAcc->y / norm;
   vAcc->z = vAcc->z / norm;
   vAcc->w = vAcc->w / norm;   
} 
/*-----------------------------------------------------------------------------
 *      applyAccError : Error is sum of cross product between 
 *                      estimated direction and measured direction of gravity
 *
 *  Parameters: const quat *vAcc, quat *vError, const mat33 qpAttitude
 *
 *  Return:     Void
 *----------------------------------------------------------------------------*/
void applyAccError(const quat *vAcc, quat *vError, const mat33 qpAttitude) 
{
    quaternionNormalize(vAcc);

    vError->x += (vAcc->y * (1.0f - 2.0f * qpAttitude.m[0u][0u] - 2.0f * qpAttitude.m[1u][1u]) - vAcc->z * (2.0f * (qpAttitude.m[1u][2u] - -qpAttitude.m[3u][0u])));
    vError->y += (vAcc->z * (2.0f * (qpAttitude.m[0u][2u] + -qpAttitude.m[3u][1u])) - vAcc->x * (1.0f - 2.0f * qpAttitude.m[0u][0u] - 2.0f * qpAttitude.m[1u][1u]));
    vError->z += (vAcc->x * (2.0f * (qpAttitude.m[1u][2u] - -qpAttitude.m[3u][0u])) - vAcc->y * (2.0f * (qpAttitude.m[0u][2u] + -qpAttitude.m[3u][1u])));
}
/*-----------------------------------------------------------------------------
 *      applySensorCorrectionUsingCOG : use COG from GPS for corection quaternion vError
 *
 *
 *  Parameters: quat *vError,const mat33 qpAttitude, const GPS_Info_t *gps
 *               Vectr *attitude,const bool fixedWing ,float32_t GPS_distanceToHome
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void applySensorCorrectionUsingCOG(quat *vError,const mat33 qpAttitude, const GPS_Info_t *gps, Vectr *attitude,const bool fixedWing, float32_t GPS_distanceToHome )   // qpAttitude is 3x3 from mahony AHRS
{
  float32_t courseOverGround = DEGREE_TO_RADIAN(gps->COG); 
  static uint8_t hasInitializedGPSHeading = false;
  float32_t tiltDirection;
          
  if ((vError == NULL) || (gps == NULL))
  { /* for misra */ }
  else
  {
    if ( gps->numOfSatel >= 5u && gps->SOG >= 600.0f)                           // gps->timestamp   
    {
        /* In case of a fixed-wing aircraft we can use GPS course over ground to correct heading */
        if(fixedWing == false)                                                  // {-- compensation --} 
        {
            tiltDirection = atan2(attitude->x, attitude->y);                    // For applying correction to heading based on craft tilt in 2d space
            courseOverGround += tiltDirection;

            if (!hasInitializedGPSHeading && (GPS_distanceToHome > 50.0f)) 
            {                                                                   // Initially correct the gps heading, we can deal with gradual corrections later
                attitude->z = RADIAN_TO_DEGREE(courseOverGround);
                hasInitializedGPSHeading = true;
            }
        } 
        // Use raw heading error (from GPS or whatever else)
        while (courseOverGround >  PI) courseOverGround -= (2.0f * PI);
        while (courseOverGround < -PI) courseOverGround += (2.0f * PI);

        // William Premerlani and Paul Bizard, Direction Cosine Matrix IMU - Eqn. 22-23
        // (Rxx; Ryx) - measured (estimated) heading vector (EF)
        // (cos(COG), sin(COG)) - reference heading vector (EF)
        // error is cross product between reference heading and estimated heading (calculated in EF)
        courseOverGround = -(float32_t)sin(courseOverGround) * (1.0f - 2.0f * qpAttitude.m[1u][1u] - 2.0f * qpAttitude.m[2u][2u]) - cos(courseOverGround) * (2.0f * (qpAttitude.m[0u][1u] - -qpAttitude.m[3u][2u]));
        applyVectorError(courseOverGround, vError, qpAttitude);
    }
  }
}
/*-----------------------------------------------------------------------------
 *      applySensorCorrectionUsingMAG : use magnetometer for corection quaternion vError
 *
 *
 *  Parameters: quat *vError, const mat33 qpAttitude, const quat mag ,quat *vMagAverage
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void applySensorCorrectionUsingMAG(quat *vError, const mat33 qpAttitude, const quat mag, quat *vMagAverage )
{
      float32_t hx,hy,bx;
      
      // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
      // This way magnetic field will only affect heading and wont mess roll/pitch angles

      // we pass the corrected compass reading as we chose to correct to here ..... compassGetAverage(&vMagAverage);
      
      quaternionNormalize(vMagAverage);

      // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
      // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
      hx = (1.0f - 2.0f * qpAttitude.m[1u][1u] - 2.0f * qpAttitude.m[2u][2u]) * vMagAverage->x + (2.0f * (qpAttitude.m[0u][1u] + -qpAttitude.m[3u][2u])) * vMagAverage->y + (2.0f * (qpAttitude.m[0u][2u] - -qpAttitude.m[3u][1u])) * vMagAverage->z;
      hy = (2.0f * (qpAttitude.m[0u][1u] - -qpAttitude.m[3u][2u])) * vMagAverage->x + (1.0f - 2.0f * qpAttitude.m[0u][0u] - 2.0f * qpAttitude.m[2u][2u]) * vMagAverage->y + (2.0f * (qpAttitude.m[1u][2u] + -qpAttitude.m[3u][0u])) * vMagAverage->z;
      bx = sqrt((hx*hx) + (hy*hy));

      // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
      applyVectorError(-(float32_t)(hy * bx), vError, qpAttitude);
}
/*
 * Copyright (C) 2010 Beat Küng <beat-kueng@gmx.net>
 * ported by acp aviation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*-----------------------------------------------------------------------------
 *      EDate_isLeapYear : checks for leap year
 *
 *
 *  Parameters: uint8_t year
 *
 *  Return:     uint8_t 1 for leap year
 *----------------------------------------------------------------------------*/
uint8_t EDate_isLeapYear(uint8_t year) 
{
        if(year%4u==0u) 
        {
          if(year%100u==0u && year%400u!=0u) return(false);
          return(true);
        }
        return(false);
}
/*-----------------------------------------------------------------------------
 *      EDate_dayCountFromMonth : increase the day of the date by specified ammount
 *
 *  Parameters: uint8_t days, uint8_t *day, uint8_t *month, uint8_t *year               
 *
 *  Return:     uint8_t 
 *----------------------------------------------------------------------------*/
uint8_t EDate_dayCountFromMonth(uint8_t month, uint8_t year) 
{
        switch(month) 
        {
           case 1u: return(31u);
           case 2u: if(EDate_isLeapYear(year)) return(29u); return(28u);
           case 3u: return(31u);
           case 4u: return(30u);
           case 5u: return(31u);
           case 6u: return(30u);
           case 7u: return(31u);
           case 8u: return(31u);
           case 9u: return(30u);
           case 10u: return(31u);
           case 11u: return(30u);
           case 12u: return(31u);
        }
        return(30u);
}
/*-----------------------------------------------------------------------------
 *      EDate_increaseDay : increase the day of the date by specified ammount
 *
 *  Parameters: uint8_t days, uint8_t *day, uint8_t *month, uint8_t *year               
 *
 *  Return:     void 
 *----------------------------------------------------------------------------*/
void EDate_increaseDay(uint8_t days, uint8_t *day, uint8_t *month, uint8_t *year) 
{
        uint8_t day_count=EDate_dayCountFromMonth(*month, *year);
        if((*day+=days) > day_count) 
        {
                *day -= day_count;
                if(++*month>12u) 
                {
                   ++*year;
                   *month=1u;
                }
        }
}
#if defined(NAVILOCK_USED)
/*-----------------------------------------------------------------------------
 *      getDeltaTimeSec : get delta time in seconds
 *
 *  Parameters: const nvEDate_t *start_date, const nvETime_t *start_time, 
 *              const nvEDate_t *end_date, const nvETime_t *end_time
 *
 *  Return:     int16_t time diff between 2 objects 
 *----------------------------------------------------------------------------*/
int16_t getDeltaTimeSec(const nvEDate_t *start_date, const nvETime_t *start_time, const nvEDate_t *end_date, const nvETime_t *end_time) 
{
        
        int16_t ret=(((int16_t)end_time->hour-(int16_t)start_time->hour)*60+((int16_t)end_time->min-(int16_t)start_time->min))*60+((int16_t)end_time->sec-(int16_t)start_time->sec);
        uint8_t i;
        int16_t a = (int16_t)start_date->year*13+(int16_t)start_date->month;
        int16_t b = (int16_t)end_date->year*13+(int16_t)end_date->month;
        for(i=a; i<b; ++i) 
        {
           ret += (int16_t)EDate_dayCountFromMonth(i%13, i/13)*3600*24;
        }
        ret += ((int16_t)end_date->day-(int16_t)start_date->day)*3600*24;
        
        return(ret);
}
#endif
/*-----------------------------------------------------------------------------
 *      E3dPoint_EllipsoidDistWGS84 : elipsoid wgs84
 *
 *
 *  Parameters: const float64_t latitude
 *
 *  Return:     float64_t corrected lattitude
 *----------------------------------------------------------------------------*/
float64_t E3dPoint_EllipsoidDistWGS84(const float64_t latitude) 
{
        float64_t sin_lat=sin(latitude/180.0*PI);
        float64_t bovera=EARTH_SEMI_MINOR_AXIS_B/EARTH_SEMI_MAJOR_AXIS_A;
        return(EARTH_SEMI_MAJOR_AXIS_A / sqrt(1.0f-sin_lat*sin_lat*(1.0f-bovera*bovera)));
}
/*-----------------------------------------------------------------------------
 *      E3dPoint : calculate vector from gps co-ord
 *
 *
 *  Parameters: const GPS_Info_t *gps
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr E3dPoint(const GPS_Info_t *gps) 
{
        float64_t bovera;
        Vectr vec;
        float64_t N=E3dPoint_EllipsoidDistWGS84(gps->lat);
        float64_t cos_lat=cos(gps->lat/180.0f*PI);
        vec.y=(N+(float64_t)gps->SealevelAltitude)*cos_lat*cos(gps->longd/180.0f*PI);
        vec.x=(N+(float64_t)gps->SealevelAltitude)*cos_lat*sin(gps->longd/180.0f*PI);
        bovera=EARTH_SEMI_MINOR_AXIS_B/EARTH_SEMI_MAJOR_AXIS_A;
        vec.z=(bovera*bovera*N+(float64_t)gps->SealevelAltitude)*sin(gps->lat/180.0f*PI);
        return vec;
}
/*-----------------------------------------------------------------------------
 *      pointDistanceSquare : Point Distance square from 2 vectors 
 *
 *
 *  Parameters: const Vectr p1, const Vectr p2
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t pointDistanceSquare(const Vectr p1, const Vectr p2) 
{
        return((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}
/*-----------------------------------------------------------------------------
 *      pointDistance : distance between 2 vectors 
 *
 *
 *  Parameters: const Vectr p1, const Vectr p2
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t pointDistance(const Vectr p1, const Vectr p2) 
{
        return(sqrt(pointDistanceSquare(p1, p2)));
}
#if defined(NAVILOCK_USED)
/*-----------------------------------------------------------------------------
 *      navilock_write_uart : write navilock data wrapper
 *
 *
 *  Parameters: int16_t comport, uint8_t *buf, uint8_t len
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void navilock_write_uart( int16_t comport, uint8_t *buf, uint8_t len  )
{
   int8_t loop;
   
   if (buf != NULL)
   {           
      switch (comport)
      {
          case 1:
          for (loop=0;loop<len;loop++)
             UART1_write(buf[loop]);
          break;
          
          case 2:
          for (loop=0;loop<len;loop++)
             UART2_write(buf[loop]);
          break;
          
          case 3:
          for (loop=0;loop<len;loop++)
             UART3_write(buf[loop]);
          break;

          case 4:
          for (loop=0;loop<len;loop++)
             UART4_write(buf[loop]);
          break;
          
          case 5:
          for (loop=0;loop<len;loop++)
             UART5_write(buf[loop]);
          break;

          case 6:
          for (loop=0;loop<len;loop++)
             UART6_write(buf[loop]);
          break;
          
          default:
          break;
      }
   } else { /* for sanity */ }
}
/*-----------------------------------------------------------------------------
 *      setLatitLon : convert lat / lon co-ord from message data
 *
 *
 *  Parameters: int16_t degrees, int16_t min, float64_t sec
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t setLatitLon(int16_t degrees, int16_t min, float64_t sec) 
{ 
                float64_t degree;
                if(degrees >= 1000.0f)                                          //negative
                { 
                   degree=-((float64_t)degrees-1000.0f+((float64_t)sec/60.0f+(float64_t)min)/60.0f); 
                } else {
                   degree=(float64_t)degrees+((float64_t)sec/60.0f+(float64_t)min)/60.0f; 
                }
                return degree;
}
/*-----------------------------------------------------------------------------
 *      CNavilock_readTrackInfo : read the navilock track info
 *
 *
 *  Parameters: uint8_t track, char* buffer, uint8_t comport, navilock_t *nav, navi_track_t *trak
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void CNavilock_readTrackInfo(uint8_t track, char* buffer, uint8_t comport, navilock_t *nav, navi_track_t *trak) 
{        
        char request[6u]= { 0x54u, 0x46u, 0x00u, 0x00u, 0x00u, 0x00u };
        static const unsigned char no_track[]= { 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu };
        uint32_t val;
        uint16_t addr;
        
        if (buffer == NULL) return;        
        
        request[4u]=((track>>8u) & 0xFFu);
        request[5u]=(track & 0xFF);
        switch(nav->state)
        {
                case NAVILOCK_REQ_TRACK:                                        /* write request out */
                navilock_write_uart(comport, request, 6);
                nav->state = NAVILOCK_WAIT_REPLY;
                nav->lastTickRef = CP0_GET(CP0_COUNT);
                nav->ResendCnt = ++nav->ResendCnt % UINT8_MAX;
                //trak->points_done = 0;                                        
                if (nav->ResendCnt >= NAVI_RESEND_MAX)
                {
                    nav->readTrak = 1u;                                         /* set triger back on */
                    nav->charIdx = 0u;                                          /* reset the collection buffer flush the receiver */
                    nav->state = NAVILOCK_REQ_DONE;                             /* finish and re-queue the request */
                }
                break;
                
                case NAVILOCK_WAIT_REPLY:                                       /* waiting for interrupt */
                calculateTick2Now(&nav->ticksSoFar,&nav->lastTickRef);
                if (nav->ticksSoFar > NAVI_RETRY_TIM) nav->state = NAVILOCK_REQ_TRACK;
                break;
                
                case NAVILOCK_REPLY_RCV:                                        /* state set in interrupt */
                if ((memcmp((void*)&nav->buf,(void*) no_track,(int16_t) sizeof(no_track))!=0) && (sizeof(nav->buf) >= NAVI_TRAK_DATA_LEN))           /* reply wasnt no track and was right length */
                {
                   nav->state = NAVILOCK_PROCESS_TRACK;
                }   
                else 
                { 
                   calculateTick2Now(&nav->ticksSoFar,&nav->lastTickRef); 
                   if (nav->ticksSoFar > NAVI_RETRY_TIM) nav->state = NAVILOCK_REQ_TRACK;        
                }
                break;

                case NAVILOCK_PROCESS_TRACK:                                    /* process the track data recieved */
                trak->poi_count = nav->buf[12u];
                trak->point_count = NAVI_READ_UINT32(nav->buf, 0) + trak->poi_count;
                trak->start_addr = NAVI_READ_UINT32(nav->buf, 4);
                trak->start_date.year = NAVI_READ_USHORT16(nav->buf,8);
                trak->tot_distance = NAVI_READ_UINT32(nav->buf,12) / 10.0f;     // ?
                trak->start_date.month = nav->buf[14u];
                trak->start_date.day = nav->buf[15u];
                trak->start_time.hour = nav->buf[16u];
                trak->start_time.min = nav->buf[17u];   
                trak->start_time.sec = nav->buf[18u];   
                trak->end_date.year = NAVI_READ_USHORT16(nav->buf,10);
                trak->end_date.month = nav->buf[19u];
                trak->end_date.day = nav->buf[20u];
                trak->end_time.hour = nav->buf[21u];
                trak->end_time.min = nav->buf[22u];   
                trak->end_time.sec = nav->buf[23u]; 
                trak->points_done = 0lu;                                        /* reset the tracks done as we are requesting a new list of traks to get points from */
                if (nav->charIdx-NAVI_TRAK_DATA_LEN > 0)
                {
                   nav->charIdx = nav->charIdx-NAVI_TRAK_DATA_LEN;              /* subtract what was read from the incoming interrupt buffer (chunk reader) */
                   memcpy((void*)&nav->buf,(void*)&nav->buf[NAVI_TRAK_DATA_LEN],nav->charIdx); /* move the next chunk up */
                }
                else
                {
                   nav->charIdx = 0u;                                           /* START  again at the begining */
                }   
                nav->state = NAVILOCK_REQ_POINT;                        
                break;
                
                case NAVILOCK_REQ_POINT:                                        /* request a frame from the Point cloud */
                if (nav->ResendCnt >= NAVI_RESEND_MAX)
                {
                    nav->readTrak = 1u;                                         /* set triger back on */
                    nav->charIdx = 0u;                                          /* reset the collection buffer flush the receiver */
                    nav->state = NAVILOCK_REQ_DONE;                             /* finish and re-queue the request */
                }
                request[1u] = 0x50u;
                if (trak->points_done<trak->point_count)                        /* not all done */
                {
                   addr = trak->start_addr + 0x10u + trak->points_done;
                   request[2u]=(addr>>24u) & 0xFFu;
                   request[3u]=(addr>>16u) & 0xFFu;
                   request[4u]=(addr>>8u) & 0xFFu;
                   request[5u]=(addr & 0xFFu);  
                   navilock_write_uart(comport, request, 6);                    /* send a request for a frame from the Point cloud data */ 
                   nav->lastTickRef = CP0_GET(CP0_COUNT);
                   nav->state = NAVILOCK_WAIT_POINT_REPLY;                      /* wait for the Point reply */
                }             
                else { nav->state = NAVILOCK_REQ_DONE; }                        /* complete as all Point returned now */
                break;

                case NAVILOCK_WAIT_POINT_REPLY:                                 /* wait for the Point reply */
                calculateTick2Now(&nav->ticksSoFar,&nav->lastTickRef);
                if (nav->ticksSoFar > NAVI_RETRY_TIM) nav->state = NAVILOCK_REQ_POINT;
                break;
                
                case NAVILOCK_POINTS_RCV:                                       /* check vailidity of size of reply for Point cloud request */
                if (sizeof(nav->buf) >= NAVI_POINT_DATA_LEN)
                {
                   nav->state = NAVILOCK_PROCESS_POINT;                         /* right size process it */
                }
                else
                {
                   nav->state = NAVILOCK_REQ_POINT;                             /* wrong size then request it again */
                }               
                break; 
                                               
                case NAVILOCK_PROCESS_POINT:                                    /* process the Point cloud */
                val=NAVI_READ_UINT32(nav->buf, 0);
                trak->points[trak->points_done]->lattitude = setLatitLon((int16_t)(val/100000), (int16_t)(val/1000)%100, (float64_t)(val%1000)/10.0);
                val=NAVI_READ_UINT32(nav->buf, 4);
                trak->points[trak->points_done]->longditude = setLatitLon((int16_t)(val/100000), (int16_t)(val/1000)%100, (float64_t)(val%1000)/10.0);
                trak->points[trak->points_done]->type = nav->buf[8u];
                trak->points[trak->points_done]->speed = nav->buf[9u]*NAVI_SPEED_FACTOR; // km/h
                trak->points[trak->points_done]->time.hour = nav->buf[10u];
                trak->points[trak->points_done]->time.min = nav->buf[11u];
                trak->points[trak->points_done]->time.sec = nav->buf[12u];    
                trak->points[trak->points_done]->altitude = NAVI_READ_USHORT16(nav->buf,14);   
                trak->points_done = ++trak->points_done % UINT64_MAX;
                if (nav->charIdx-NAVI_POINT_DATA_LEN > 0)
                {
                   nav->charIdx = nav->charIdx-NAVI_POINT_DATA_LEN;              /* subtract what was read from the incoming interrupt buffer (chunk reader) */
                   memcpy((void*)&nav->buf,(void*)&nav->buf[NAVI_POINT_DATA_LEN],nav->charIdx); /* move the next chunk up */
                }
                else
                {
                   nav->charIdx = 0u;                
                } 
                if (trak->points_done == trak->point_count)
                {
                  trak->point_count = 0u;
                  nav->state = NAVILOCK_REQ_DONE;                               /* complete the request sequence */
                }
                else
                {
                  if (nav->charIdx == 0u)                                       /* if no more data in the incoming buffer (otherwise process it here next time round) */
                  {
                    nav->state = NAVILOCK_REQ_POINT;                            /* request the next Point */
                  }
                }
                break;
                
                case NAVILOCK_REQ_SET_TOT_DIST:                                 /* request message to set total distance */
                if (nav->ResendCnt >= NAVI_RESEND_MAX)
                {
                    nav->setTotDist= 1u;                                         /* set triger back on */
                    nav->charIdx = 0u;                                          /* reset the collection buffer flush the receiver */
                    nav->state = NAVILOCK_REQ_DONE;                             /* finish and re-queue the request */
                }
                request[1u] = 0x44u;
                request[2u]=(nav->new_distance>>24u) & 0xFFu;
                request[3u]=(nav->new_distance>>16u) & 0xFFu;
                request[4u]=(nav->new_distance>>8u) & 0xFFu;
                request[5u]=(nav->new_distance & 0xFFu);  
                navilock_write_uart(comport, request, 6); 
                nav->lastTickRef = CP0_GET(CP0_COUNT);
                nav->state = NAVILOCK_WAIT_DIST_SET_REPLY;                      /* wait for reply */
                break;
                
                case NAVILOCK_WAIT_DIST_SET_REPLY:                              /* wait for interrupt to get distance response expected */
                calculateTick2Now(&nav->ticksSoFar,&nav->lastTickRef);
                if (nav->ticksSoFar > NAVI_RETRY_TIM) nav->state = NAVILOCK_REQ_SET_TOT_DIST; /* time out retry by resending request message */
                break;
                
                case NAVILOCK_SET_TOT_DIST_RCV:                                 /* set by interrupt for distance response expected */
                if (sizeof(nav->buf) == 3u)
                {
                   if (nav->charIdx-3 > 0)
                   {
                      nav->charIdx = nav->charIdx-3;                            /* subtract what was read from the incoming interrupt buffer (chunk reader) */
                      memcpy((void*)&nav->buf,(void*)&nav->buf[3u],nav->charIdx); /* move the next chunk up */
                   }
                   else
                   {
                      nav->charIdx = 0u;                                        /* START  again at the begining */
                   } 
                   nav->state = NAVILOCK_REQ_DONE;                              /* no need to check this reply just an ack */
                }
                else if (sizeof(nav->buf) == NAVI_POINT_DATA_LEN)               /* it was actually a Point reply out of sequence (remove if never happens)  */ 
                {
                   nav->state = nav->state + NAVILOCK_NUM_OF_STATES;            /* jump to exception and process the Point it arrived out of sequence (may not need this) */
                }
                else
                {
                   nav->state = NAVILOCK_REQ_TOT_DIST;                          /* re-request the setting an invalid response came back */
                }                
                break; 
                
                case NAVILOCK_REQ_TOT_DIST:                                     /* in sync request for total distance reading */
                if (nav->ResendCnt >= NAVI_RESEND_MAX)
                {
                    nav->getTotDist= 1u;                                        /* set triger back on */
                    nav->charIdx = 0u;                                          /* reset the collection buffer flush the receiver */
                    nav->state = NAVILOCK_REQ_DONE;                             /* finish and re-queue the request */
                }
                request[1u] = 0x50u;
                addr = 0;
                request[2u]=(addr>>24u) & 0xFFu;
                request[3u]=(addr>>16u) & 0xFFu;
                request[4u]=(addr>>8u) & 0xFFu;
                request[5u]=(addr & 0xFFu);  
                navilock_write_uart(comport, request, 6); 
                nav->lastTickRef = CP0_GET(CP0_COUNT);
                nav->state = NAVILOCK_WAIT_GET_TOT_DIST_REPLY;                  /* wait for reply via port interrupt or a timeout on no response */
                break;
                
                case NAVILOCK_WAIT_GET_TOT_DIST_REPLY:                          /* wait for reply via port interrupt */
                calculateTick2Now(&nav->ticksSoFar,&nav->lastTickRef);
                if (nav->ticksSoFar > NAVI_RETRY_TIM) nav->state = NAVILOCK_REQ_TOT_DIST; /* time out re-request it */
                break;
                
                case NAVILOCK_GET_TOT_DIST_RCV:                                 /* set by interrupt */
                if (sizeof(nav->buf) == 16)
                {
                   nav->state = NAVILOCK_PROCESS_TOT_DIST;                      /* compute the total distance from the message */
                }
                else
                {
                   nav->state = NAVILOCK_REQ_TOT_DIST;                          /* request it again */
                }                
                break;
                
                case NAVILOCK_PROCESS_TOT_DIST:                                 /* compute total distance from reply message byte */
                val=NAVI_READ_UINT32(nav->buf, 12);
                trak->tot_distance = (((float64_t)val)/10.0f);
                if (nav->charIdx-16 > 0)
                {
                   nav->charIdx = nav->charIdx-16;                              /* subtract what was read from the incoming interrupt buffer (chunk reader) */
                   memcpy((void*)&nav->buf,(void*)&nav->buf[16u],nav->charIdx); /* move the next chunk up */
                }
                else
                {
                   nav->charIdx = 0u;                                           /* START  again at the begining */
                } 
                nav->state = NAVILOCK_REQ_DONE;                                 /* complete and do any other pending requests */
                break;
                
                case NAVILOCK_REQ_ERAS_TRAK:                                    /* request to erase tracks */
                if (nav->ResendCnt >= NAVI_RESEND_MAX)
                {
                    nav->erasTrak = 1u;                                         /* set triger back on */
                    nav->charIdx = 0u;                                          /* reset the collection buffer flush the receiver */
                    nav->state = NAVILOCK_REQ_DONE;                             /* finish and re-queue the request */
                }
                request[0u] = 0x45u;
                request[1u] = 0x52u;
                request[2u] = 0u;
                request[3u] = 0u;
                request[4u] = 0u;
                request[5u] = 0u;  
                navilock_write_uart(comport, request, 6); 
                nav->lastTickRef = CP0_GET(CP0_COUNT);
                nav->state = NAVILOCK_WAIT_ERAS_TRAK_REPLY;                     /* wait for interrupt or timeout */
                break;

                case NAVILOCK_WAIT_ERAS_TRAK_REPLY:                             /* wait for interrupt or timeout */
                calculateTick2Now(&nav->ticksSoFar,&nav->lastTickRef);
                if (nav->ticksSoFar > NAVI_RETRY_TIM) nav->state = NAVILOCK_REQ_ERAS_TRAK;  /* re-try send message  */
                break;                

                case NAVILOCK_ERAS_TRAK_RCV:                                    /* set by interrupt */
                if (sizeof(nav->buf) == 3)                                      /* correct reply is 3 bytes long */
                {
                   if (nav->charIdx-3 > 0)
                   {
                      nav->charIdx = nav->charIdx-16;                           /* subtract what was read from the incoming interrupt buffer (chunk reader) */
                      memcpy((void*)&nav->buf,(void*)&nav->buf[3u],nav->charIdx); /* move the next chunk up */
                   }
                   else
                   {
                      nav->charIdx = 0u;                                        /* START  again at the begining */
                   }
                   if (nav->buf[2u]>=0x64u)                                     /* incoming message inidcates that we completed the erase */
                   {
                      nav->state = NAVILOCK_REQ_DONE;                           /* erase of tracks are completed */
                   }
                   else
                   {
                      nav->state = NAVILOCK_WAIT_ERAS_TRAK_REPLY;               /* wait for next erase reply */
                   }
                   break; 
                }
                else if (sizeof(nav->buf) == NAVI_POINT_DATA_LEN)               /* it was actually a Point reply out of sequence (remove if never happens)  */ 
                {
                   nav->state = nav->state + NAVILOCK_NUM_OF_STATES;            /* jump to exception and process the Point it arrived out of sequence (may not need this) */
                }
                else
                {
                   nav->state = NAVILOCK_REQ_DONE;                              /* nothing to erase */
                   break; 
                }                

                case NAVILOCK_REQ_DONE:                                         /* dormant state wait for request */
                nav->ResendCnt = 0u;
                if (nav->erasTrak)                                              /* HMI or poll sequence requests a message to erase tracks highest priority request */
                {
                   nav->state = NAVILOCK_REQ_ERAS_TRAK;
                   nav->erasTrak = 0u;                
                }
                else if (nav->readTrak)                                         /* request to read tracks abd their points */
                {
                   nav->state = NAVILOCK_REQ_TRACK;
                   nav->readTrak = 0u;
                }
                else if (nav->setTotDist)                                       /* HMI or poll sequence requests a message to set total distance */
                {
                   nav->state = NAVILOCK_REQ_SET_TOT_DIST;
                   nav->setTotDist = 0u;                
                }
                else if (nav->getTotDist)                                       /* HMI or poll sequence requests a message to get total distance */
                {
                   nav->state = NAVILOCK_REQ_TOT_DIST;
                   nav->getTotDist = 0u;                
                }
                else if (trak->points_done <= trak->point_count)                /* trap if we are still needing to collect all points */
                {
                  if (nav->charIdx >= NAVI_POINT_DATA_LEN)                      /* at least one Point is in the buffer  */
                  {
                    nav->state = NAVILOCK_PROCESS_POINT;                        /* process the remaining chunks already in the interrupt receive buffer */
                  }
                  else
                  {
                    nav->state = NAVILOCK_REQ_POINT;                            /* request the next Point */                  
                  }
                }   
                break;
                                
                default:                                                        /* it came here perhaps because we receiveed a Point cloud instead of our expected request */
                if ((nav->state >= NAVILOCK_NUM_OF_STATES+2) && (trak->points_done < trak->point_count)) /* STATE EXCEPTION HANDLER :: process Point handler out of step */
                {
                   val=NAVI_READ_UINT32(nav->buf, 0);
                   trak->points[trak->points_done]->lattitude = setLatitLon((int16_t)(val/100000), (int16_t)(val/1000)%100, (float64_t)(val%1000)/10.0);
                   val=NAVI_READ_UINT32(nav->buf, 4);
                   trak->points[trak->points_done]->longditude = setLatitLon((int16_t)(val/100000), (int16_t)(val/1000)%100, (float64_t)(val%1000)/10.0);
                   trak->points[trak->points_done]->type = nav->buf[8u];
                   trak->points[trak->points_done]->speed = nav->buf[9u]*NAVI_SPEED_FACTOR; // km/h
                   trak->points[trak->points_done]->time.hour = nav->buf[10u];
                   trak->points[trak->points_done]->time.min = nav->buf[11u];
                   trak->points[trak->points_done]->time.sec = nav->buf[12u];    
                   trak->points[trak->points_done]->altitude = NAVI_READ_USHORT16(nav->buf,14);   
                   trak->points_done = ++trak->points_done % UINT64_MAX;
                   nav->state = nav->state - NAVILOCK_NUM_OF_STATES;            /* go back to the previous satte where you came from next scan */                
                }
                break;              
        }
}
/*-----------------------------------------------------------------------------
 *      navilock_initUartPort : initialise uart port
 *
 *
 *  Parameters: int16_t comport
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void navilock_initUartPort( int16_t comport )
{
    switch(comport)
    {
       case 1u:
       UART1_Init(NAVI_BAUD_RATE);
       break;

       case 2u:
       UART2_Init(NAVI_BAUD_RATE);
       break;
       
       case 3u:
       UART3_Init(NAVI_BAUD_RATE);
       break; 
       
       case 4u:
       UART4_Init(NAVI_BAUD_RATE);
       break;

       case 5u:
       UART5_Init(NAVI_BAUD_RATE);
       break;
       
       case 6u:
       UART6_Init(NAVI_BAUD_RATE);
       break;      
    }

}
#endif /* navilock */
/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTY INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file geo.c
 *
 * Geo / math functions to perform geodesic calculations
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Doug Weibel <douglas.weibel@colorado.edu>
 *
 * Azimuthal Equidistant Projection
 * formulas according to: http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html
 *
 * ported for mikroE C and PIC32 ACP Aviation
 */
 
/*-----------------------------------------------------------------------------
 *      get_distance_to_next_waypoint():  get distance to waypoint
 *
 *  Parameters: float64_t lat_now, float64_t lon_now, float64_t lat_next,   
 *              float64_t lon_next
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/ 
float64_t get_distance_to_next_waypoint(float64_t lat_now, float64_t lon_now, float64_t lat_next, float64_t lon_next)
{
        const float64_t lat_now_rad = DEGREE_TO_RADIAN(lat_now);
        const float64_t lat_next_rad = DEGREE_TO_RADIAN(lat_next);

        const float64_t d_lat = lat_next_rad - lat_now_rad;
        const float64_t d_lon = DEGREE_TO_RADIAN(lon_next) - DEGREE_TO_RADIAN(lon_now);

        const float64_t a = sin(d_lat / 2.0f) * sin(d_lat / 2.0f) + sin(d_lon / 2.0f) * sin(d_lon / 2.0f) * cos(lat_now_rad) * cos(lat_next_rad);

        const float64_t c = atan2(sqrt(a), sqrt(1.0f - a));

        return (EARTH_RADIUS * 2.0f * c);
}
/*-----------------------------------------------------------------------------
 *      get_bearing_to_next_waypoint():  get bearing to waypoint
 *
 *  Parameters: float64_t lat_now, float64_t lon_now, float64_t lat_next, float64_t lon_next  
 *
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/ 
float64_t get_bearing_to_next_waypoint(float64_t lat_now, float64_t lon_now, float64_t lat_next, float64_t lon_next)
{
        const float64_t lat_now_rad = DEGREE_TO_RADIAN(lat_now);
        const float64_t lat_next_rad = DEGREE_TO_RADIAN(lat_next);

        const float64_t cos_lat_next = cos(lat_next_rad);
        const float64_t d_lon = DEGREE_TO_RADIAN(lon_next - lon_now);

        const float64_t y = (sin(d_lon) * cos_lat_next);
        const float64_t x = (cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos_lat_next * cos(d_lon));

        return wrap_PI(atan2(y, x));
}
/*-----------------------------------------------------------------------------
 *      get_vector_to_next_waypoint():  get vector to waypoint
 *
 *  Parameters: float64_t lat_now, float64_t lon_now, float64_t lat_next,  
 *              float64_t lon_next, float64_t *v_n, float64_t *v_e
 *
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void get_vector_to_next_waypoint(float64_t lat_now, float64_t lon_now, float64_t lat_next, float64_t lon_next, float64_t *v_n, float64_t *v_e)
{
        const float64_t lat_now_rad = DEGREE_TO_RADIAN(lat_now);
        const float64_t lat_next_rad = DEGREE_TO_RADIAN(lat_next);
        const float64_t d_lon = DEGREE_TO_RADIAN(lon_next) - DEGREE_TO_RADIAN(lon_now);

        *v_n = (EARTH_RADIUS * (cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(d_lon)));
        *v_e = (EARTH_RADIUS * sin(d_lon) * cos(lat_next_rad));
}
/*-----------------------------------------------------------------------------
 *      get_vector_to_next_waypoint_fast():  get vector to waypoint
 *
 *  Parameters: float64_t lat_now, float64_t lon_now, float64_t lat_next, 
 *              float64_t lon_next, float64_t *v_n, float64_t *v_e
 *
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void get_vector_to_next_waypoint_fast(float64_t lat_now, float64_t lon_now, float64_t lat_next, float64_t lon_next, float64_t *v_n, float64_t *v_e)
{
        float64_t lat_now_rad = DEGREE_TO_RADIAN(lat_now);
        float64_t lon_now_rad = DEGREE_TO_RADIAN(lon_now);
        float64_t lat_next_rad = DEGREE_TO_RADIAN(lat_next);
        float64_t lon_next_rad = DEGREE_TO_RADIAN(lon_next);

        float64_t d_lat = lat_next_rad - lat_now_rad;
        float64_t d_lon = lon_next_rad - lon_now_rad;

        *v_n = (EARTH_RADIUS * d_lat);
        *v_e = (EARTH_RADIUS * d_lon * cos(lat_now_rad));
}
/*-----------------------------------------------------------------------------
 *      add_vector_to_global_position():  get distance to wgs84 point
 *
 *  Parameters: float64_t lat_now, float64_t lon_now, float64_t v_n, float64_t v_e, 
 *              float64_t *lat_res, float64_t *lon_res
 *
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void add_vector_to_global_position(float64_t lat_now, float64_t lon_now, float64_t v_n, float64_t v_e, float64_t *lat_res, float64_t *lon_res)
{
        float64_t lat_now_rad = DEGREE_TO_RADIAN(lat_now);
        float64_t lon_now_rad = DEGREE_TO_RADIAN(lon_now);

        *lat_res = RADIAN_TO_DEGREE(lat_now_rad + (float64_t)v_n / EARTH_RADIUS);
        *lon_res = RADIAN_TO_DEGREE(lon_now_rad + (float64_t)v_e / (EARTH_RADIUS * cos(lat_now_rad)));
}
/*-----------------------------------------------------------------------------
 *      waypoint_from_heading_and_distance():  get distance to wgs84 point
 *
 *  Parameters: float64_t lat_start, float64_t lon_start, float64_t bearing, float64_t dist,
 *              float64_t *lat_target, float64_t *lon_target 
 *
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void waypoint_from_heading_and_distance(float64_t lat_start, float64_t lon_start, float64_t bearing, float64_t dist,float64_t *lat_target, float64_t *lon_target)
{
        float64_t radius_ratio = (float64_t)fabs((float64_t)dist) / EARTH_RADIUS;

        float64_t lat_start_rad = DEGREE_TO_RADIAN(lat_start);
        float64_t lon_start_rad = DEGREE_TO_RADIAN(lon_start);

        *lat_target = asin(sin(lat_start_rad) * cos(radius_ratio) + cos(lat_start_rad) * sin(radius_ratio) * cos((double)bearing));
        *lon_target = lon_start_rad + atan2(sin((double)bearing) * sin(radius_ratio) * cos(lat_start_rad), cos(radius_ratio) - sin(lat_start_rad) * sin(*lat_target));

        *lat_target = RADIAN_TO_DEGREE(*lat_target);
        *lon_target = RADIAN_TO_DEGREE(*lon_target);
}
/*-----------------------------------------------------------------------------
 *      mavlink_wpm_distance_to_point_local():  get distance to wgs84 point
 *
 *  Parameters: float64_t x_now, float64_t y_now, float64_t z_now, float64_t x_next, 
 *              float64_t y_next, float64_t z_next, float64_t *dist_xy, float64_t *dist_z 
 *
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t mavlink_wpm_distance_to_point_local(float64_t x_now, float64_t y_now, float64_t z_now, float64_t x_next, float64_t y_next, float64_t z_next, float64_t *dist_xy, float64_t *dist_z)
{
        float64_t dx = x_now - x_next;
        float64_t dy = y_now - y_next;
        float64_t dz = z_now - z_next;

        *dist_xy = sqrt(dx * dx + dy * dy);
        *dist_z = fabs(dz);

        return sqrt(dx * dx + dy * dy + dz * dz);
}
/*-----------------------------------------------------------------------------
 *      get_distance_to_point_global_wgs84():  get distance to wgs84 point
 *
 *  Parameters: float64_t lat_now, float64_t lon_now, float64_t alt_now, 
 *              float64_t lat_next, float64_t lon_next, float64_t alt_next, 
 *              float64_t *dist_xy, float64_t *dist_z 
 *
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t get_distance_to_point_global_wgs84(float64_t lat_now, float64_t lon_now, float64_t alt_now, float64_t lat_next, float64_t lon_next, float64_t alt_next, float64_t *dist_xy, float64_t *dist_z)
{
        float64_t c, dxy, dz;
        float64_t current_x_rad = DEGREE_TO_RADIAN(lat_next);
        float64_t current_y_rad = DEGREE_TO_RADIAN(lon_next);
        float64_t x_rad = DEGREE_TO_RADIAN(lat_now);
        float64_t y_rad = DEGREE_TO_RADIAN(lon_now);
        float64_t safeAtan2,safeSin1,safeSin2;

        float64_t d_lat = x_rad - current_x_rad;
        float64_t d_lon = y_rad - current_y_rad;

        float64_t a = sin(d_lat / 2.0f) * sin(d_lat / 2.0f) + sin(d_lon / 2.0f) * sin(d_lon / 2.0f) * cos(current_x_rad) * cos(x_rad);

        IKatan2(safeAtan2, sqrt(a), sqrt(1.0f - a));
        c = 2.0f * safeAtan2;
        dxy = (EARTH_RADIUS * c);
        dz = (alt_now - alt_next);

        *dist_xy = fabs(dxy);
        *dist_z = fabs(dz);

        return sqrt(dxy * dxy + dz * dz);
}
/*-----------------------------------------------------------------------------
 *      get_distance_to_line():  get distance to line
 *
 *  Parameters: crosstrack_error_t *crosstrack_error, float64_t lat_now, 
 *              float64_t lon_now, float64_t lat_start, float64_t lon_start, 
 *              float64_t lat_end, float64_t lon_end 
 *
 *
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
int16_t get_distance_to_line(crosstrack_error_t *crosstrack_error, float64_t lat_now, float64_t lon_now, float64_t lat_start, float64_t lon_start, float64_t lat_end, float64_t lon_end)
{
        // This function returns the distance to the nearest point on the track line.  Distance is positive if current
        // position is right of the track and negative if left of the track as seen from a point on the track line
        // headed towards the end point.

        int16_t return_value = -1;                                              // Set error flag, cleared when valid result calculated.
        float64_t dist_to_end,bearing_end,bearing_track,bearing_diff;
        float64_t safeSin;
        
        crosstrack_error->past_end = false;
        crosstrack_error->distance = 0.0f;
        crosstrack_error->bearing = 0.0f;

        dist_to_end = get_distance_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);

        if (dist_to_end < 0.1f)                                                 // Return error if arguments are bad
        {
                return -1;
        }

        bearing_end = get_bearing_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
        bearing_track = get_bearing_to_next_waypoint(lat_start, lon_start, lat_end, lon_end);
        bearing_diff = wrap_PI(bearing_track - bearing_end);

        // Return past_end = true if past end point of line
        if (bearing_diff > IKPI_2 || bearing_diff < -IKPI_2) {
                crosstrack_error->past_end = true;
                return_value = 0;
                return return_value;
        }

        IKsin(safeSin,bearing_diff);
        crosstrack_error->distance = (dist_to_end) * safeSin;

        if (safeSin >= 0) 
        {
                crosstrack_error->bearing = wrap_PI(bearing_track - IKPI_2);

        } else {
                crosstrack_error->bearing = wrap_PI(bearing_track + IKPI_2);
        }

        return_value = 0;

        return return_value;
}
/*-----------------------------------------------------------------------------
 *      get_distance_to_arc():  get distance to arc
 *
 *  Parameters: crosstrack_error_t *crosstrack_error, float64_t lat_now, 
 *              float64_t lon_now, float64_t lat_center, float64_t lon_center, 
 *              float64_t radius, float64_t arc_start_bearing, float64_t arc_sweep
 *
 *  Return:     int16_t -1 error, 0 okay
 *----------------------------------------------------------------------------*/
int16_t get_distance_to_arc(crosstrack_error_t *crosstrack_error, float64_t lat_now, float64_t lon_now, float64_t lat_center, float64_t lon_center, float64_t radius, float64_t arc_start_bearing, float64_t arc_sweep)
{
        // This function returns the distance to the nearest point on the track arc.  Distance is positive if current
        // position is right of the arc and negative if left of the arc as seen from the closest point on the arc and
        // headed towards the end point.

        // Determine if the current position is inside or outside the sector between the line from the center
        // to the arc start and the line from the center to the arc end
        float64_t bearing_sector_start = 0.0f;
        float64_t bearing_sector_end = 0.0f;
        float64_t bearing_now = get_bearing_to_next_waypoint(lat_now, lon_now, lat_center, lon_center);

        int16_t return_value = -1;                                              // Set error flag, cleared when valid result calculated.
        
        float64_t start_disp_x; 
        float64_t start_disp_y; 
        float64_t end_disp_x; 
        float64_t end_disp_y;
        float64_t lon_start; 
        float64_t lat_start;
        float64_t lon_end; 
        float64_t lat_end; 
        float64_t dist_to_start;
        float64_t dist_to_end; 
        float64_t dist_to_center;
        uint8_t in_sector;
        
        crosstrack_error->past_end = false;
        crosstrack_error->distance = 0.0f;
        crosstrack_error->bearing = 0.0f;

        if (radius < 0.1f)                                                      // Return error if arguments are bad
        {
                return return_value;
        }

        if (arc_sweep >= 0.0f) 
        {
                bearing_sector_start = arc_start_bearing;
                bearing_sector_end = arc_start_bearing + arc_sweep;

                if (bearing_sector_end > 2.0f * PI) { bearing_sector_end -= (2.0f * PI); }

        } 
        else 
        {
                bearing_sector_end = arc_start_bearing;
                bearing_sector_start = arc_start_bearing - arc_sweep;

                if (bearing_sector_start < 0.0f) { bearing_sector_start += (2.0f * PI); }
        }

        in_sector = false;

        // Case where sector does not span zero
        if (bearing_sector_end >= bearing_sector_start && bearing_now >= bearing_sector_start && bearing_now <= bearing_sector_end) 
        {
                in_sector = true;
        }

        // Case where sector does span zero
        if (bearing_sector_end < bearing_sector_start && (bearing_now > bearing_sector_start || bearing_now < bearing_sector_end)) 
        {

                in_sector = true;
        }

        // If in the sector then calculate distance and bearing to closest point
        if (in_sector) 
        {
                crosstrack_error->past_end = false;
                dist_to_center = get_distance_to_next_waypoint(lat_now, lon_now, lat_center, lon_center);

                if (dist_to_center <= radius) 
                {
                        crosstrack_error->distance = radius - dist_to_center;
                        crosstrack_error->bearing = bearing_now + PI;

                } 
                else 
                {
                        crosstrack_error->distance = dist_to_center - radius;
                        crosstrack_error->bearing = bearing_now;
                }

                // If out of the sector then calculate dist and bearing to start or end point

        } 
        else 
        {

                // Use the approximation  that 111,111 meters in the y direction is 1 degree (of latitude)
                // and 111,111 * cos(latitude) meters in the x direction is 1 degree (of longitude) to
                // calculate the position of the start and end points.  We should not be doing this often
                // as this function generally will not be called repeatedly when we are out of the sector.

                start_disp_x = radius * sin(arc_start_bearing);
                start_disp_y = radius * cos(arc_start_bearing);
                end_disp_x = radius * sin(wrap_PI(arc_start_bearing + arc_sweep));
                end_disp_y = radius * cos(wrap_PI(arc_start_bearing + arc_sweep));
                lon_start = lon_now + start_disp_x / 111111.0f;
                lat_start = lat_now + start_disp_y * cos(lat_now) / 111111.0f;
                lon_end = lon_now + end_disp_x / 111111.0f;
                lat_end = lat_now + end_disp_y * cos(lat_now) / 111111.0f;
                dist_to_start = get_distance_to_next_waypoint(lat_now, lon_now, lat_start, lon_start);
                dist_to_end = get_distance_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);

                if (dist_to_start < dist_to_end) 
                {
                        crosstrack_error->distance = dist_to_start;
                        crosstrack_error->bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_start, lon_start);

                } 
                else 
                {
                        crosstrack_error->past_end = true;
                        crosstrack_error->distance = dist_to_end;
                        crosstrack_error->bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
                }
        }

        crosstrack_error->bearing = wrap_PI(crosstrack_error->bearing);
        return_value = 0;

        return return_value;
}
/*-----------------------------------------------------------------------------
 *      fmin():  return min of 2 values
 *
 *  Parameters: float64_t a, float64_t b 
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t fmin( float64_t a, float64_t b )
{
  return (a < b) ? a : b;
}
/*-----------------------------------------------------------------------------
 *      getNavigateSetpoint():  get new navigation set point
 *
 *  Parameters: float64_t speed, geoPose_t nav_start, geoPose_t setpoint_position_transformed, 
 *              Vectr *nav_setpoint, bool wait_armed
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void getNavigateSetpoint( float64_t speed, geoPose_t nav_start, geoPose_t setpoint_position_transformed, Vectr *nav_setpoint, bool wait_armed )
{
        float64_t distance;
        float64_t time;
        float64_t passed;
        uint32_t currentTick,TickDuration;
        
        if (wait_armed)                                                         // don't start navigating if we're waiting arming
        {
           nav_start.stamp = CP0_GET(CP0_COUNT);
        }

        distance = getDistanceVector(nav_start.position, setpoint_position_transformed.position);
        time = distance / speed;
        currentTick = CP0_GET(CP0_COUNT);
        calculateTime2Tick( &TickDuration, &nav_start.stamp, &currentTick );
        passed = fmin((TICKS2SEC(TickDuration) / time), 1.0f);

        nav_setpoint->x = nav_start.position.x + (setpoint_position_transformed.position.x - nav_start.position.x) * passed;
        nav_setpoint->y = nav_start.position.y + (setpoint_position_transformed.position.y - nav_start.position.y) * passed;
        nav_setpoint->z = nav_start.position.z + (setpoint_position_transformed.position.z - nav_start.position.z) * passed;
}
// Copyright (C) 2018 kaz Kojima
//
// This file is part of PMLPS program.  This program is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.
// ----- Ported to mikroE C by ACP Aviation ------
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
/*-----------------------------------------------------------------------------
 *      biquadLPF_init():  initialises a biquad_t filter array
 *
 *  Parameters: biquadLPF_t *biq
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void biquadLPF_init( biquadLPF_t *biq )
{
  biq->m_b0 = 1.0f;
  biq->m_b1 = 0.0f;
  biq->m_b2 = 0.0f;
  biq->m_a1 = 0.0f;
  biq->m_a2 = 0.0f;
  biq->m_d1 = 0.0f;
  biq->m_d2 = 0.0f;        
}
/*-----------------------------------------------------------------------------
 *      biquadLPF_doBiquadLPF():  Computes a biquad filter on a sample
 *
 *  Parameters: float32_t filtf, float32_t q, float32_t sampf, biquadLPF_t *biq
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void biquadLPF_doBiquadLPF(float32_t filtf, float32_t q, float32_t sampf, biquadLPF_t *biq)
{

  const float32_t omega = 2.0f*PI*filtf/sampf;                                  /* setup variables */
  const float32_t sn = sin(omega);
  const float32_t cs = cos(omega);
  const float32_t alpha = sn/(2.0f*q);
  float32_t b0, b1, b2, a0, a1, a2;
  // FILTER_NOTCH:
  // b0 =  1;
  // b1 = -2 * cs;
  // b2 =  1;
  b0 = (1.0f - cs)/2.0f;
  b1 = 1.0f - cs;
  b2 = (1.0f - cs)/2.0f;
  a0 =  1.0f + alpha;
  a1 = -2.0f * cs;
  a2 =  1.0f - alpha;

  biq->m_b0 = b0/a0;                                                            /* precompute the coefficients */
  biq->m_b1 = b1/a0;
  biq->m_b2 = b2/a0;
  biq->m_a1 = a1/a0;
  biq->m_a2 = a2/a0;
  
  biq->m_d1 = biq->m_d2 = 0.0f;                                                 /* zero initial samples  */
}
 
/*-----------------------------------------------------------------------------
 *      biquadLPF_reset():  Computes a biquad_t filter on a sample
 *
 *  Parameters: float32_t input, biquadLPF_t *biq
 *       
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t biquadLPF_update (float32_t input, biquadLPF_t *biq)
{
  const float32_t result = biq->m_b0*input + biq->m_d1;
  biq->m_d1 = biq->m_b1*input - biq->m_a1*result + biq->m_d2;
  biq->m_d2 = biq->m_b2*input - biq->m_a2*result;
  return result;
}
/*-----------------------------------------------------------------------------
 *      biquadLPF_reset():  reset biquat filter
 *
 *  Parameters: float32_t value, biquadLPF_t *biq 
 *       
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t biquadLPF_reset (float32_t value, biquadLPF_t *biq)
{
  biq->m_d1 = value - (value*biq->m_b0);
  biq->m_d2 = (biq->m_b2 - biq->m_a2)*value;
  return value;
}
/*-----------------------------------------------------------------------------
 *      T265convQuaternionToEuler():  quat to euler
 *
 *  Parameters: float32_t x, float32_t y, float32_t z, float32_t w
 *       
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr * T265convQuaternionToEuler(float32_t x, float32_t y, float32_t z, float32_t w) 
{
  const float32_t ysqr = y * y;
  const float32_t t0 = 2.0f * (w * x + y * z);
  const float32_t t1 = 1.0f - 2.0f * (x * x + ysqr);
  const float32_t t3 = 2.0f * (w * z + x * y);
  const float32_t t4 = 1.0f - 2.0f * (ysqr + z * z);
  float32_t euler_y;
  Vectr out;
  float32_t euler_x = atan2(t0, t1);
  float32_t euler_z = atan2(t3, t4);
  float32_t t2 = 2.0f * (w * y - z * x);
  t2 = (t2 > 1.0f) ? 1.0f : t2;
  t2 = (t2 < -1.0f) ? -1.0f : t2;
  euler_y = asin(t2);

  out.x = euler_x;
  out.y = euler_y;
  out.z = euler_z;

  return &out;
}
/*-----------------------------------------------------------------------------
 *      T265oscEvent():  T265 event
 *
 *  Parameters: const geoPose_t msg, T265Pose_t *pose
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void T265oscEvent(const geoPose_t msg, T265Pose_t *pose)
{
  const float32_t scale_f = 1000.0f;                                            /* [m]->[mm]  */
  Vectr *euler;

  if (pose == NULL)
  { /* for misra */
  }
  else
  {   
     pose->translation.x =  scale_f * msg.position.x;                           // position / translation vector
     pose->translation.y = -scale_f * msg.position.y;
     pose->translation.z =  scale_f * msg.position.z;

     pose->rotation.w = msg.orientation.x;                                      // rotation / orientation quaternion
     pose->rotation.x = msg.orientation.y;
     pose->rotation.y = msg.orientation.z;
     pose->rotation.z = msg.orientation.w;

     euler = T265convQuaternionToEuler(pose->rotation.x, -pose->rotation.y, pose->rotation.z, pose->rotation.w);       // quaternion -> euler angle
     pose->euler.x = -euler->x;
     pose->euler.y = -euler->y;
     pose->euler.z = -euler->z;
  }
}
/*-----------------------------------------------------------------------------
 *      mmul():  multiply two 3x3 matrices.
 *
 *  Parameters: mat33 a, mat33 b
 *       
 *
 *  Return:     mat33
 *----------------------------------------------------------------------------*/
mat33 mmul(mat33 a, mat33 b) 
{
        mat33 ab;
        int8_t i,j,k;
        float32_t accum;
        for (i = 0; i < 3; ++i) 
        {
                for (j = 0; j < 3; ++j) 
                {
                        accum = 0.0f;
                        for (k = 0; k < 3; ++k) 
                        {
                          accum += a.m[i][k] * b.m[k][j];
                        }
                        ab.m[i][j] = accum;
                }
        }
        return ab;
}
/*-----------------------------------------------------------------------------
 *      mtranslate():  multiply matrix by vector.
 *
 *  Parameters: mat33 *m, Vectr v
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void mtranslate(mat33 *m, Vectr v)
{
   m->m[0u][0u] = m->m[0u][0u] * v.x;
   m->m[0u][1u] = m->m[0u][1u] * v.y;
   m->m[0u][2u] = m->m[0u][2u] * v.z;
   m->m[1u][0u] = m->m[1u][0u] * v.x;
   m->m[1u][1u] = m->m[1u][1u] * v.y;
   m->m[1u][2u] = m->m[1u][2u] * v.z;
   m->m[2u][0u] = m->m[2u][0u] * v.x;
   m->m[2u][1u] = m->m[2u][1u] * v.y;
   m->m[2u][2u] = m->m[2u][2u] * v.z;
}
/*-----------------------------------------------------------------------------
 *      mscale():  multiply matrix by scalar.
 *
 *  Parameters: mat33 *m, Vectr v
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void mscale(mat33 *m, Vectr v)
{
   m->m[0u][0u] = m->m[0u][0u] * v.x;
   m->m[0u][1u] = m->m[0u][1u] * v.x;
   m->m[0u][2u] = m->m[0u][2u] * v.x;
   m->m[1u][0u] = m->m[1u][0u] * v.y;
   m->m[1u][1u] = m->m[1u][1u] * v.y;
   m->m[1u][2u] = m->m[1u][2u] * v.y;
   m->m[2u][0u] = m->m[2u][0u] * v.z;
   m->m[2u][1u] = m->m[2u][1u] * v.z;
   m->m[2u][2u] = m->m[2u][2u] * v.z;
}
/*-----------------------------------------------------------------------------
 *      T265_calcHeadMatrix():  calc head matrix
 *
 *  Parameters: float32_t tx, float32_t ty, float32_t tz, float32_t qw, float32_t qx, float32_t qy, float32_t qz
 *       
 *
 *  Return:     mat33
 *----------------------------------------------------------------------------*/
mat33 T265_calcHeadMatrix(float32_t tx, float32_t ty, float32_t tz, float32_t qw, float32_t qx, float32_t qy, float32_t qz) 
{
  mat33 mat;
  mat33 toLeftHand = { 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f }; /* array for converting right hand orientation to left hand orientation */
  Vectr vScale = { 1.0f, -1.0f, 1.0f };
  
  float32_t two_xSquared = 2.0f * qx * qx;                                      /* calculate matrix terms */
  float32_t two_ySquared = 2.0f * qy * qy;
  float32_t two_zSquared = 2.0f * qz * qz;
  float32_t two_xy = 2.0f * qx * qy;
  float32_t two_wz = 2.0f * qw * qz;
  float32_t two_xz = 2.0f * qx * qz;
  float32_t two_wy = 2.0f * qw * qy;
  float32_t two_yz = 2.0f * qy * qz;
  float32_t two_wx = 2.0f * qw * qx;
  Vectr transLation; 
  
  mat.m[0u][0u] = 1.0f - two_ySquared - two_zSquared;                           /* update view matrix orientation */
  mat.m[0u][1u] = two_xy + two_wz;
  mat.m[0u][2u] = two_xz - two_wy;
  mat.m[1u][0u] = two_xy - two_wz;
  mat.m[1u][1u] = 1.0f - two_xSquared - two_zSquared;
  mat.m[1u][2u] = two_yz + two_wx;
  mat.m[2u][0u] = two_xz + two_wy;
  mat.m[2u][1u] = two_yz - two_wx;
  mat.m[2u][2u] = 1.0f - two_xSquared - two_ySquared;
  /* mat.translate(-tx, -ty, -tz);    */
  mtranslate(&mat, mkvec(-tx, -ty, -tz));

  /* change right-hand to left-hand
  mat.preApply(
    1, 0, 0, 0, 
    0, -1, 0, 0, 
    0, 0, 1, 0, 
    0, 0, 0, 1
    );
  mat.scale(1, -1, 1);      */
  mat = mmul(mat, toLeftHand);
  mscale(&mat, vScale);
  return mat;
}
/*-----------------------------------------------------------------------------
 *      T265_calcHeadMatrix2():  calc head matrix
 *
 *  Parameters: T265Pose_t *pose
 *       
 *
 *  Return:     mat33
 *----------------------------------------------------------------------------*/
mat33 T265_calcHeadMatrix2(T265Pose_t *pose) 
{
  mat33 mat;
  mat33 toLeftHand = { 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f }; /* array for converting right hand orientation to left hand orientation */
  Vectr vScale = { 1.0f, -1.0f, 1.0f };
  
  float32_t two_xSquared = 2.0f * pose->rotation.x * pose->rotation.x;          /* calculate matrix terms */
  float32_t two_ySquared = 2.0f * pose->rotation.y * pose->rotation.y;
  float32_t two_zSquared = 2.0f * pose->rotation.z * pose->rotation.z;
  float32_t two_xy = 2.0f * pose->rotation.x * pose->rotation.y;
  float32_t two_wz = 2.0f * pose->rotation.w * pose->rotation.z;
  float32_t two_xz = 2.0f * pose->rotation.x * pose->rotation.z;
  float32_t two_wy = 2.0f * pose->rotation.w * pose->rotation.y;
  float32_t two_yz = 2.0f * pose->rotation.y * pose->rotation.z;
  float32_t two_wx = 2.0f * pose->rotation.w * pose->rotation.x;
  Vectr transLation; 
  
  mat.m[0u][0u] = 1.0f - two_ySquared - two_zSquared;                           /* update view matrix orientation */
  mat.m[0u][1u] = two_xy + two_wz;
  mat.m[0u][2u] = two_xz - two_wy;
  mat.m[1u][0u] = two_xy - two_wz;
  mat.m[1u][1u] = 1.0f - two_xSquared - two_zSquared;
  mat.m[1u][2u] = two_yz + two_wx;
  mat.m[2u][0u] = two_xz + two_wy;
  mat.m[2u][1u] = two_yz - two_wx;
  mat.m[2u][2u] = 1.0f - two_xSquared - two_ySquared;
  /* mat.translate(-tx, -ty, -tz);    */
  mtranslate(&mat, mkvec(-pose->translation.x, -pose->translation.y, -pose->translation.z));

  /* change right-hand to left-hand
  mat.preApply(
    1, 0, 0, 0, 
    0, -1, 0, 0, 
    0, 0, 1, 0, 
    0, 0, 0, 1
    );
  mat.scale(1, -1, 1);      */
  mat = mmul(mat, toLeftHand);
  mscale(&mat, vScale);
  return mat;
}
// Copyright (C) 2018 kaz Kojima
//
// This file is part of PMLPS program.  This program is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program;
// see the files COPYING and EXCEPTION respectively.
/*-----------------------------------------------------------------------------
 *      dsq():  cross of difference between 2 vectors
 *
 *  Parameters: const Vectr p, const Vectr q
 *       
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t dsq(const Vectr p, const Vectr q) 
{
    return ((p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y) + (p.z-q.z)*(p.z-q.z));
}
/*-----------------------------------------------------------------------------
 *      ImageSensorBlob():  look at blob in picture
 *
 *  Parameters: const uint16_t x, const uint16_t y, const uint16_t w, const uint16_t h, 
 *              ImageSensorBlob_t *blob, const camera_config_t config     
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void ImageSensorBlob(const uint16_t x, const uint16_t y, const uint16_t w, const uint16_t h, ImageSensorBlob_t *blob, const camera_config_t config)
{
      uint16_t cx = config.cam_image_width/2u;
      uint16_t cy = config.cam_image_height/2u;
      float32_t lens_ratio = config.cam_lens_ratio;

      blob->_ix = (float32_t)(x-cx) * lens_ratio;
      blob->_iy = (float32_t)(cy-y) * lens_ratio;
      blob->_iw = (float32_t)w * lens_ratio;
      blob->_ih = (float32_t)h * lens_ratio;
}
/*-----------------------------------------------------------------------------
 *      pmlps_correct():  corrector step
 *
 *  Parameters: float32_t yaw_meas, ImageSensorBlob_t *blob     
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t pmlps_correct(float32_t yaw_meas, ImageSensorBlob_t *blob)
{
    float32_t PMLPS_K = PMLPS_H*blob->_p/(PMLPS_H*PMLPS_H*blob->_p + PMLPS_R);
    blob->_xhat = blob->_xhat + PMLPS_K*(yaw_meas - PMLPS_H*blob->_xhat);
    blob->_p = blob->_p*(1.0f - PMLPS_H*PMLPS_K);
    blob->_u = blob->_xhat - blob->_xhat_prev;
    return blob->_xhat;
}
/*-----------------------------------------------------------------------------
 *      pmlps_predict():  predictor step
 *
 *  Parameters: ImageSensorBlob_t *blob    
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void pmlps_predict( ImageSensorBlob_t *blob )
{
    blob->_xhat_prev = blob->_xhat;
    blob->_xhat = blob->_xhat + PMLPS_B*blob->_u;
    blob->_p = blob->_p + PMLPS_Q;
}
/*-----------------------------------------------------------------------------
 *      pmlps_VisualYawEstimater():  visual yaw estimator
 *
 *  Parameters: float32_t a, float32_t b, float32_t q, float32_t r, ImageSensorBlob_t *blob    
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void pmlps_VisualYawEstimater(float32_t a, float32_t b, float32_t q, float32_t r, ImageSensorBlob_t *blob) 
{
      PMLPS_H = a; PMLPS_B = b; PMLPS_Q = q; PMLPS_R = r;
      blob->_xhat = 0.0f; blob->_p = PMLPS_Q; blob->_u = 0.0f; blob->_ct = 0;
}
/*-----------------------------------------------------------------------------
 *      pmlps_merge(): merge 2 vectors  
 *
 *  Parameters: Vectr p, Vectr q    
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr pmlps_merge(Vectr p, Vectr q)
{
  return mkvec((p.x + q.x)/2.0f, (p.y + q.y)/2.0f, (p.z + q.z)/2.0f);
} 
/*-----------------------------------------------------------------------------
 *      vmin(): element-wise minimum of vector. 
 *
 *  Parameters: Vectr a, Vectr b    
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vmin(Vectr a, Vectr b) 
{
        return mkvec(FMIN(a.x, b.x), FMIN(a.y, b.y), FMIN(a.z, b.z));
}
/*-----------------------------------------------------------------------------
 *      vminArr(): element-wise minimum of vector. 
 *
 *  Parameters: Vectr a, Vectr b    
 *
 *  Return:     float32_t *
 *----------------------------------------------------------------------------*/
float32_t * vminArr(Vectr a, Vectr b) 
{
        float32_t Arr[3u];
        Arr[0u] = FMIN(a.x, b.x);
        Arr[1u] = FMIN(a.y, b.y);
        Arr[2u] = FMIN(a.z, b.z);
        return &Arr;
}
/*-----------------------------------------------------------------------------
 *      vmax(): element-wise maximum of vector. 
 *
 *  Parameters: Vectr a, Vectr b    
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vmax(Vectr a, Vectr b) 
{
        return mkvec(FMAX(a.x, b.x), FMAX(a.y, b.y), FMAX(a.z, b.z));
}
#ifndef M_PI
#define M_PI PI
#endif
#define GA_VERSOR_VERSION
/* Estimate yaw from markers and measured yaw value (instrument reading received via mavlink).
   Assume MARKER_TYPE_I or MARKER_TYPE_I3. */
/*-----------------------------------------------------------------------------
 *      VisualYawEstimater_estimate_visual_yaw():  Estimate yaw from markers and measured yaw value 
 *
 *  Parameters: Vectr fm[], camera_config_t *config, ImageSensorBlob_t *blob    
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t VisualYawEstimater_estimate_visual_yaw(Vectr fm[], camera_config_t *config, ImageSensorBlob_t *blob )
{
  static float32_t prev_yaw = 0.0f;
  float32_t yaw_direction_offset = config->cam_direction;
  float32_t x = fm[0u].x - fm[1u].x;
  float32_t y = fm[0u].y - fm[1u].y;
  float32_t alpha;
  float32_t hx,hy,yaw_meas, cx, cy, cz, s, c, tau, calpha, salpha, ctau, fn, yaw;
#if defined(GA_VERSOR_VERSION)
  float32_t nv,nu;
  Vectr rop,dlp0,cir;
#endif
  if (config->prev_yaw_angle_init == false)
  {
    config->prev_yaw_angle = 10.0f;                                             // Invalid angle
    config->prev_yaw_angle_init = true;
  }
  if (config->update_attitude)
  {
     alpha = -(config->yaw_angle + yaw_direction_offset);
     prev_yaw = alpha;
     if (!config->_initialized && fabs(config->yaw_angle - config->prev_yaw_angle) > 0.05f)            // Return yaw_angle if it looks unstable.
     {
          config->prev_yaw_angle = config->yaw_angle;
          return config->yaw_angle;
     }
     if (++blob->_ct > PMLPS_YAW_INITIALIZE_COUNT)
          config->_initialized = true;
  }
  else if (!config->_initialized)
  {
     if (++blob->_ct > PMLPS_YAW_INITIALIZE_COUNT)
            config->_initialized = true;
     return prev_yaw;                                                         // Not right yet
  }
  else
  {
     alpha = prev_yaw;
  }

  hx = cos(alpha);
  hy = sin(alpha);

  if (x*x + y*y < 0.5f)                                                         // Give up visual estimation
  {
      config->suc = false;                                                      // Last resort
      return alpha;
  }
  config->suc = true;

  if (config->marker_type != MARKER_TYPE_I3 && x * hx + y * hy <= 0.0f)         // (x, y) gives the heading for MARKER_TYPE_I3.
  {
      x = -x;
      y = -y;
  }

  // TODO: Don't use old pitch_angle.
  cx = (fm[0u].x + fm[1u].x)/2.0f;                                              // Adjust with attitude
  cy = (fm[0u].y + fm[1u].y)/2.0f;
  cz = (fm[0u].z + fm[1u].z)/2.0f;
#ifndef GA_VERSOR_VERSION
  // ===== Spherical triangle version. ============
  s = cz/sqrt(cx*cx+cy*cy+cz*cz);
  c = sqrt(1.0f-s*s);

  tau = 0.0f;
  if (cos(config->pitch_angle) > c && s > 0.2f && fabs(cx*y - cy*x) > 100.0f)
    {
      salpha = c/cos(config->pitch_angle);
      calpha = sqrt(1.0f-salpha*salpha);
      ctau = calpha/s;
      tau = acos(ctau);
      if (config->pitch_angle < 0.0f)
            tau = -tau;
      if (cx*y - cy*x < 0.0f)
            tau = -tau;
    }
  yaw_meas = atan2(y, x);
  yaw_meas += tau;
#else
  // GA version. -- need to be able to decifer Versor library from pablo collapinto 
#ifdef HAVE_COLAPINTO
  float nv = sqrtf(x*x+y*y);
  Vec v = Vec(x/nv, y/nv, 0.0f);
  Vec w = Vec(y/nv, -x/nv, 0.0f);
  auto bi = v ^ Vec(0.0f, 0.0f, 1.0f);
  auto rop = Gen::rot(bi*(-pitch_angle/2.0f));
  //v.sp(rop).print();
  float nu = sqrtf(cx*cx+cy*cy+cz*cz);
  Vec u = Vec(cx/nu, cy/nu, -cz/nu);
  auto PointO = PT(0, 0, 0);
  auto dlp0 = (PointO ^ (v ^ u) ^ Inf(1)).dual();
  // circle as a plunge point ^ dual_plane ^ dual_plane
  auto cir = v.sp(rop).null() ^ Biv::xy;
  auto pp = dlp0 <= cir;
  auto pp0 = Round::split(pp, true);
  yaw_meas = atan2f(pp0[1u], pp0[0u]);
  assert(!isnan(yaw_meas));
#else
  nv = sqrt(x*x+y*y);  
  rop = vscl((-config->pitch_angle/2.0f), pmlps_merge(mkvec(x/nv, y/nv, 0.0f), mkvec(0.0f, 0.0f, 1.0f)));
  nu = sqrt(cx*cx+cy*cy+cz*cz);
  dlp0 = pmlps_merge(mkvec(0.0f, 0.0f, 0.0f), pmlps_merge(mkvec(x/nv, y/nv, 0.0f), mkvec(cx/nu, cy/nu, -cz/nu))); 
  /*   auto cir = v.sp(rop).null() ^ Biv::xy;
       auto pp = dlp0 <= cir; */
  cir = pmlps_merge(pmlps_merge(rop,dlp0), dlp0);                               // circle as a plunge point ^ dual_plane ^ dual_plane
  dlp0 = vmin(dlp0, cir);
  yaw_meas = atan2(dlp0.y, dlp0.x);                                             
#endif
#endif

  fn = ROUND(((prev_yaw - yaw_meas)/(2.0f*M_PI)),0.0f);                         // Uncover
  yaw_meas = yaw_meas + 2.0f*M_PI*fn;

  pmlps_predict( blob );                                                        // Predict and Update
  yaw = pmlps_correct(yaw_meas, blob);
  if (IsaNan(yaw)) return 0.0f;

  if (config->_initialized)
    prev_yaw = yaw;
  yaw = yaw + yaw_direction_offset;
  fn = ROUND((yaw / (2.0f*M_PI)),0.0f);
  yaw = -(yaw - 2.0f*M_PI*fn);
  if (yaw > M_PI)
    yaw -= 2.0f*M_PI;
  if (yaw < -M_PI)
    yaw += 2.0f*M_PI;
  return yaw;
}
/*-----------------------------------------------------------------------------
 *      pmlps_linear():   
 *
 *  Parameters: Vectr p, Vectr q, Vectr r    
 *
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool pmlps_linear(Vectr p, Vectr q, Vectr r)
{
  float32_t sq;                                                                  
  float32_t ax = p.x - q.x;
  float32_t ay = p.y - q.y;
  float32_t bx = r.x - q.x;
  float32_t by = r.y - q.y;
  float32_t sqa = ax*ax + ay*ay;
  float32_t sqb = bx*bx + by*by;
  if (sqa < 0.1f || sqb < 0.f)
    return false;
  sq = fabs(ax*by - ay*bx);
  sq = sq*sq;
  return (sq < sqa*sqb*0.01f);
}
// Map 3-D to 2-D with Fish-eye.
// Input: (x, y, z) normalized position
// Output: (ix, iy) sensor image cordinate
// Output: rho      estimated bounded box size
/*-----------------------------------------------------------------------------
 *      pmlps_fish(): Map 3-D to 2-D with Fish-eye.  
 *
 *  Parameters: const float32_t x, const float32_t y, const float32_t z, int16_t *ix, 
 *              int16_t *iy, int16_t *rho, const camera_config_t config   
 *
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool pmlps_fish(const float32_t x, const float32_t y, const float32_t z, int16_t *ix, int16_t *iy, int16_t *rho, const camera_config_t config)
{
  Vectr v;
  float32_t hx, hy;
  float32_t nv, lens_ratio;
  uint16_t cx, cy;
  float32_t marker_sq, rk;
  float32_t pos[4u];
  
  if (config.cam_lens_fisheye)
    {
      nv = sqrt(x*x+y*y+z*z);
      if (nv < 1)
              return false;
      v = mkvec(x/nv, y/nv, -z/nv - 1.0f);
    }
  else
    {
      v = mkvec(x, y, -z - 1.0f);
    }

  /* +++++++++++++++++++ TODO : versor lib to be converted !!!!!! 
  v.print();
  auto pointN = Construct::point(0,0,1);
  auto line = pointN ^ v ^ Inf(1);
  auto pos = (Vec(0.0f,0.0f,1.0f) <= line);
  pos.print();   
  
  Vectr line = pmlps_merge(mkvec(0.0f,0.0f,1.0f),v);
  pos = vminArr(line,mkvec(0.0f,0.0f,1.0f));    but where do we get pos[3] from ???
  */

  hx = pos[0u]/pos[3u];
  hy = pos[1u]/pos[3u];

  cx = config.cam_image_width/2.0f;
  cy = config.cam_image_height/2.0f;
  lens_ratio = config.cam_lens_ratio;
  *ix = (int16_t)(cx + hx/lens_ratio);
  *iy = (int16_t)(cy - hy/lens_ratio);

  // Estimate bounded box size roughly.
  // Assume that the marker will move N(=2) times of its size between frames
  // and use it for the size of bounded sphere. Scale that sphere with
  // the distance from the origin and use the size of scaled sphere for
  // the bounded box size on the image plane. This isn't acculate at all
  // but will be ok for a very rough estimation.
  marker_sq = config.marker_sqsize;
  // Add marker_sq to divisor so as to avoid the floting division issue.
  rk = 2.0f*sqrt(marker_sq/(marker_sq+x*x+y*y+z*z));
  *rho = (int16_t)(rk/lens_ratio);
  //printf("bounded box size %d\n", rho);
  return true;
}
/*-----------------------------------------------------------------------------
 *      vectrMul(): multiply 2 vector  
 *
 *  Parameters: Vectr a, Vectr b  
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vectrMul( Vectr a, Vectr b )
{
        return mkvec(a.x*b.x, a.y*b.y, a.z*b.z );
}
/*-----------------------------------------------------------------------------
 *      vectrDiv(): divide vector a by b  
 *
 *  Parameters: Vectr a, Vectr b  
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vectrDiv( Vectr a, Vectr b )
{
        return mkvec(a.x/b.x, a.y/b.y, a.z/b.z );
}
// 3-D reconstruction from Fish-eye image and height info.
// Input: (x, y) normalized position on image sensor
// Input: h the vertical distance from sensor
// Output: pos[0], pos[1], pos[2] estimated position (X, Y, -h)
/*-----------------------------------------------------------------------------
 *      pmlps_unfish(): 3-D reconstruction from Fish-eye image and height info.  
 *
 *  Parameters: float32_t x, float32_t y, float32_t h, float32_t *hx, float32_t *hy, const camera_config_t config  
 *
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool pmlps_unfish(float32_t x, float32_t y, float32_t h, float32_t *hx, float32_t *hy, const camera_config_t config)
{
  Vectr v;
  float32_t u2, pos[4u];
  
  if (config.cam_lens_fisheye)
  {
#if 0
      // v = -(u-e3) e3 / (u-e3) i.e. the stereographic projection from
      // x-y plane to unit sphere.
      v = - vectrDiv( vectrMul(mkvec(x,y,-1.0f) , mkvec(0.0f,0.0f,1.0f)) , mkvec(x,y,-1.0f));
#else
      u2 = x*x + y*y;
      v = mkvec(2.0f*x/(u2+1.0f), 2.0f*y/(u2+1.0f), (u2-1.0f)/(u2+1.0f));
#endif
  }
  else
  {
      v = mkvec(x,y,-1.0f);                                                     // straight projection
  }

  /* ++++++++++ versor library TO DO !!!!!!! 
  auto pointO = Construct::point(0,0,0);
  auto pointP = Construct::point(0,0,-h);
  auto line = pointO ^ v ^ Inf(1);
  auto dlp = (pointP <= Drv(0,0,1));
  // flat point as meet
  auto pos = (dlp <= line);    */
  *hx = pos[0u]/pos[3u];
  *hy = pos[1u]/pos[3u];
  return true;
}
/*-----------------------------------------------------------------------------
 *      pmlps_find_frame(): find frame  
 *
 *  Parameters: Vectr *m, float32_t h, Vectr fm[], const camera_config_t config, float32_t *herr 
 *
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
int16_t pmlps_find_frame(Vectr *m, float32_t h, Vectr fm[], const camera_config_t config, float32_t *herr)
{
  size_t n = sizeof(*m);
  Vectr *um;
  Vectr pm[2u];
  int16_t np = 0;
  float32_t sq = 0;
  float32_t marker_sq = config.marker_sqsize;
  float32_t size_sq_min = config.marker_sqsize * 0.0625f;
  float32_t size_sq_max = config.marker_sqsize * 4.0f;
  float32_t errmin, d01, d10, d, dij, sqdiff, x,y;
  int16_t i,j;
  
  um = (Vectr*)Malloc(n);                                                       /* allocate enough memory for the size of the incoming vector point cloud */
  
  for(i = 0; i < n; i++)                                                        // Find frame with hint height h
  {
      pmlps_unfish(m[i].x, m[i].y, h, &x, &y, config);                          // unfish all
      um[i] = mkvec(x, y, h);
  }

  errmin = UINT64_MAX; //FLOAT_MAX; /// find !!! FLOAT32_MAX;

  if (config.prev)
  {
      pm[0u] = mkvec(fm[0u].x, fm[0u].y, fm[0u].z);
      pm[1u] = mkvec(fm[1u].x, fm[1u].y, fm[1u].z);

      for(i = 0; i < n; i++)                                                    // double loop. match with previous result
          for(j = i+1; j < n; j++)
          {
            d01 = dsq(pm[0u], um[i]) + dsq(pm[1u], um[j]);
            d10 = dsq(pm[1u], um[i]) + dsq(pm[0u], um[j]);
            d = FMIN(d01, d10);
            if (d > errmin)
              continue;
            errmin = d;
            fm[0u] = mkvec(um[i].x, um[i].y, um[i].z);
            fm[1u] = mkvec(um[j].x, um[j].y, um[j].z);
          }

      sq = dsq(fm[0u], fm[1u]);
      if (errmin < position_sq_epsilon && sq > size_sq_min && sq < size_sq_max)
      {
             np = 2;
      }
  }

  if (np == 0)                                                                  // double loop. match with square size
  {
      errmin = UINT64_MAX; //FLT_MAX;
      for(i = 0; i < n; i++)
          for(j = i+1; j < n; j++)
          {
            dij = dsq(um[i], um[j]);
            sqdiff = fabs(marker_sq - dij);
            if (sqdiff > errmin)
              continue;
            errmin = sqdiff;
            fm[0u] = mkvec(um[i].x, um[i].y, um[i].z);
            fm[1u] = mkvec(um[j].x, um[j].y, um[j].z);
            sq = dij;
          }

    if (sq > size_sq_min && sq < size_sq_max)
    {
          np = 2;
    }
  }
  Free((char*)um,sizeof(um));                                                   /* free the memory allocated to the pointer as we are finished with it */
  
  if (np == 0)                                                                  // No frame marker found.
    return np;

  *herr = sqrt(marker_sq / sq);                                                 // Estimate the height error ratio from sq ratio.

  return np;
}
/*-----------------------------------------------------------------------------
 *      rpy2quat(): construct from (roll, pitch, yaw) Euler angles using Tait-Bryan convention
 *                  (yaw, then pitch about new pitch axis, then roll about new roll axis)
 *
 *  Parameters: Vectr rpy 
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat rpy2quat(Vectr rpy) 
{
        // from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        float32_t r = rpy.x;
        float32_t p = rpy.y;
        float32_t y = rpy.z;
        float32_t cr = cos(r / 2.0f); float32_t sr = sin(r / 2.0f);
        float32_t cp = cos(p / 2.0f); float32_t sp = sin(p / 2.0f);
        float32_t cy = cos(y / 2.0f); float32_t sy = sin(y / 2.0f);

        float32_t qx = sr * cp * cy -  cr * sp * sy;
        float32_t qy = cr * sp * cy +  sr * cp * sy;
        float32_t qz = cr * cp * sy -  sr * sp * cy;
        float32_t qw = cr * cp * cy +  sr * sp * sy;

        return mkquat(qx, qy, qz, qw);
}
/*-----------------------------------------------------------------------------
 *      quaternion_get_yaw(): return yaw from quaternion  
 *      ported from information source mavros 
 *      Copyright 2015,2016 Vladimir Ermakov.
 *
 *  Parameters: const quat q
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t quaternion_get_yaw(const quat q)
{
        // to match equation from:
        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        const float64_t q0 = q.w;
        const float64_t q1 = q.x;
        const float64_t q2 = q.y;
        const float64_t q3 = q.z;

        return atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));
}
// compute the angle of a quaternion's axis-angle decomposition.
// result lies in domain (-pi, pi].
/*-----------------------------------------------------------------------------
 *      quat2angle():   
 *
 *  Parameters: quat q 
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t quat2angle(quat q) 
{
        float32_t angle = 2.0f * acos(q.w);
        if (angle > PI) 
        {
                angle -= 2.0f * PI;
        }
        return angle;
}
 
/*-----------------------------------------------------------------------------
 *      qeye(): construct an identity quaternion.  
 *
 *  Parameters: void 
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat qeye(void) 
{
        return mkquat(0, 0, 0, 1);
}

/*-----------------------------------------------------------------------------
 *      vmagfast(): vector magnitude..  
 *
 *  Parameters: Vectr v 
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t vmagfast(Vectr v) 
{
        return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
 
/*-----------------------------------------------------------------------------
 *      qaxisangle(): construct a quaternion from an axis and angle of rotation.  
 *                    does not assume axis is normalized.
 *  Parameters: Vectr axis, float32_t angle 
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
quat qaxisangle(Vectr axis, float32_t angle) 
{
        float32_t scale = sin(angle / 2.0f) / vmag(axis);
        quat q;
        q.x = scale * axis.x;
        q.y = scale * axis.y;
        q.z = scale * axis.z;
        q.w = cos(angle/2.0f);
        return q;
}
/*-----------------------------------------------------------------------------
 *      math_copysign(): replicate ath copysign in c++
 *              
 *  Parameters: float32_t val, float32_t sign
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/
float32_t math_copysign(float32_t val, float32_t sign)
{
    if (val >= 0.0f)
       return (sign >= 0.0f ? val : -val);
    else
       return (sign >= 0.0f ? -val : val);
}
/*-----------------------------------------------------------------------------
 *      mat2quat(): matrix to quartenion
 *              
 *  Parameters: mat33 m
 *
 *  Return: quat
 *----------------------------------------------------------------------------*/
quat mat2quat(mat33 m) 
{
        float32_t w = sqrt(FMAX(0.0f, 1.0f + m.m[0u][0u] + m.m[1u][1u] + m.m[2u][2u])) / 2.0f;
        float32_t x = sqrt(FMAX(0.0f, 1.0f + m.m[0u][0u] - m.m[1u][1u] - m.m[2u][2u])) / 2.0f;
        float32_t y = sqrt(FMAX(0.0f, 1.0f - m.m[0u][0u] + m.m[1u][1u] - m.m[2u][2u])) / 2.0f;
        float32_t z = sqrt(FMAX(0.0f, 1.0f - m.m[0u][0u] - m.m[1u][1u] + m.m[2u][2u])) / 2.0f;
        x = math_copysign(x, m.m[2u][1u] - m.m[1u][2u]);
        y = math_copysign(y, m.m[0u][2u] - m.m[2u][0u]);
        z = math_copysign(z, m.m[1u][0u] - m.m[0u][1u]);
        return mkquat(x, y, z, w);
} 
/*-----------------------------------------------------------------------------
 *      qvrot(): rotate a vector by a quaternion. 
 *      from http://gamedev.stackexchange.com/a/50545 
 *              
 *  Parameters: quat q, Vectr v
 *
 *  Return: Vectr
 *----------------------------------------------------------------------------*/
Vectr qvrot(quat q, Vectr v) 
{
        Vectr qv;
        qv = mkvec(q.x, q.y, q.z);
        return vadd3(vscl(2.0f * vdot(qv, v), qv),vscl(q.w * q.w - vmag2(qv), v),vscl(2.0f * q.w, vcross(qv, v)));
}
#if defined(AIR_DENSITY_CALC)
/*   Ref : Brisbane Air Baloon Club and 
     Richard Shelquist
     Longmont, Colorado                                                      */
/*-----------------------------------------------------------------------------
 *      getSatWaterVP(): Calculating saturation pressure of Water Vapour using herman wobus 
 *              
 *  Parameters: float64_t temerat
 *
 *  Return: float64_t (satWaterVP)
 *----------------------------------------------------------------------------*/
float64_t getSatWaterVP(float64_t temperat) 
{
        return (6.1078f / (0.99999683f+temperat*(-0.90826951e-2f+temperat*(0.78736169e-4f+temperat*(-0.61117958e-6+temperat*(0.43884187e-8f+temperat*(-0.29883885e-10f+temperat*(0.21874425e-12f+temperat*(-0.17892321e-14f+temperat*(0.11112018e-16f+temperat*(-0.30994571e-19)))))))))));
}
/*-----------------------------------------------------------------------------
 *      getWaterVP(): Calculating actual Vapour Pressure using relative humidity 
 *              
 *  Parameters: float64_t RH, float64_t satWaterVP
 *
 *  Return: float64_t (P_V)
 *----------------------------------------------------------------------------*/
float64_t getWaterVP(float64_t RH, float64_t satWaterVP) 
{
        return (RH*satWaterVP);
}
/*-----------------------------------------------------------------------------
 *      getDryAirPPA(): Calculating dry Air pressure (pressure due to dry Air) 
 *              
 *  Parameters: float64_t P (total), float64_t P_V (pressure due to water vapor)
 *
 *  Return: float64_t (ppDA)
 *----------------------------------------------------------------------------*/
float64_t getDryAirPPA(float64_t P, float64_t P_V) 
{
        return (P-P_V);
}
/*-----------------------------------------------------------------------------
 *      getHumidAirDensity(): Moist Air desity calculation
 *              
 *  Parameters: float64_t ppDA (partial press dry Air)
 *              float64_t T_K (kelvin)
 *              float64_t P_V (water vapor pressure calcualted above)
 *
 *  Return: float64_t 
 *----------------------------------------------------------------------------*/
float64_t getHumidAirDensity(float64_t ppDA, float64_t T_K, float64_t P_V ) 
{
        return ((ppDA/(R_D*T_K))+(P_V/(R_V*T_K)));
}
/*-----------------------------------------------------------------------------
 *      getHumidAirDensity2(): Moist Air desity calculation
 *              
 *  Parameters: float64_t P total air pressure, Pascals ( multiply mb by 100 to get Pascals)
 *              float64_t T_K (kelvin)
 *              float64_t P_V (water vapor pressure calcualted above)
 *
 *  Return: float64_t 
 *----------------------------------------------------------------------------*/
float64_t getHumidAirDensity2(float64_t P, float64_t T_K, float64_t P_V ) 
{
        return ((P/(R_D*T_K))+(1.0f-((0.378f*P_V)/P)));
}
/*-----------------------------------------------------------------------------
 *      getGeometricAlt(): Geometric Altitiude
 *              
 *  Parameters: float64_t H geopotntial latitude (Km)
 *
 *  Return: float64_t 
 *----------------------------------------------------------------------------*/
float64_t getGeometricAlt(float64_t H) 
{
        return (EARTHRADIUS1976ISA*H)/(EARTHRADIUS1976ISA-H);
}
/*-----------------------------------------------------------------------------
 *      getVirtTemp():  Tv = virtual temperature, deg K
 *              
 *  Parameters: float64_t  T = ambient temperature, deg K, 
 *              float64_t E = vapor pressure, mb
 *              float64_t  P = actual (station) pressure, mb
 *
 *  Return: float64_t (VirtTemp)
 *----------------------------------------------------------------------------*/
float64_t getVirtTemp(float64_t T, float64_t E, float64_t P) 
{
        return (T/(1.0f-(C1_CONST*(E/P))));
}
/*-----------------------------------------------------------------------------
 *      getGeoPotDenAlt():  geopotential density altitude, km
 *              
 *  Parameters: float64_t  VirtTemp =  virtual temperature, deg K 
 *              float64_t  P = actual (station) pressure, Pascals
 *
 *  Return: float64_t (VirtTemp)
 *----------------------------------------------------------------------------*/
float64_t getGeoPotDenAlt(float64_t VirtTemp, float64_t P) 
{
        return (44.3308f - (11.1802f*pow((P/VirtTemp),0.234969f)));
}
/*-----------------------------------------------------------------------------
 *      getActStatPress():  actual staion pressure
 *              
 *  Parameters: float64_t AS = altimeter setting, mb
 *              float64_t H = geopotential station elevation, m
 *
 *  Return: float64_t (VirtTemp)
 *----------------------------------------------------------------------------*/
float64_t getActStatPress(float64_t AS, float64_t H) 
{
        return  pow( (pow(AS, 0.190263f) - (8.417286e-5f * H)), (1.0f/0.190263f) );
}
/*-----------------------------------------------------------------------------
 *      getDensAlt():  density altitude feet
 *              
 *  Parameters: float64_t PA actual pressure (station pressure), inches Hg
 *              float64_t T temperature, deg R (deg F + 459.67)
 *
 *  Return: float64_t (VirtTemp)
 *----------------------------------------------------------------------------*/
float64_t getDensAlt(float64_t PA, float64_t T) 
{
        return (145442.16f*(1.0f- pow(((17.326f*PA)/T),0.235f)));
}
/* ref : P Riseborough Ardupilot Ausraila */
/*-----------------------------------------------------------------------------
 *      getPressError():  pressure error
 *              
 *  Parameters: float64_t H Height
 *              float64_t D Density as above
 *
 *  Return: float64_t (Pe)
 *----------------------------------------------------------------------------*/
float64_t getPressError(float64_t H, float64_t D) 
{
        return (H*GRAV_CONST*D);
}
/*-----------------------------------------------------------------------------
 *      getDynPress():  pressure dynamic
 *              
 *  Parameters: float64_t A Air speed
 *              float64_t D Density as above
 *
 *  Return: float64_t (Pd)
 *----------------------------------------------------------------------------*/
float64_t getDynPress(float64_t A, float64_t D) 
{
        return (0.5f*(D*(A*A)));
}
/*-----------------------------------------------------------------------------
 *      getBaro1_WCF_LFT():  BARO01_WCF_LFT
 *              
 *  Parameters: float64_t Pd - dyanmic pressure
 *              float64_t Pe - pressure error
 *
 *  Return: float64_t (Baro01_WCF_LFT)
 *----------------------------------------------------------------------------*/
float64_t getBaro1_WCF_LFT(float64_t Pd, float64_t Pe) 
{
        return (Pe/Pd);
}
#endif
/*-----------------------------------------------------------------------------
 *      tripleValidateSignal():  triple validation of signal
 *              
 *  Parameters: Vectr sig1 - position from measuremnt system 1
 *              Vectr sig2 - position from measuremnt system 2
 *              Vectr sig3 - position from measuremnt system 3
 *
 *  limits max,min, deviation from each other, estimate to velocity (reality)
 *
 *  Return: float64_t (Baro01_WCF_LFT)
 *----------------------------------------------------------------------------*/
Vectr tripleValidateSignal(Vectr sig1, Vectr sig2, Vectr sig3, float64_t velocity) 
{
        uint8_t ValidVals = 0u;
        float32_t max_sig = 100.0f;
        float32_t min_sig = 0f;
        float32_t k = 0.123f; /* consatnt to scale velocity */
        float32_t max_trav = k*velocity;
        float32_t min_trav = (k/5.0f)*velocity;
        float32_t deviation = 0.5f;
        float64_t valSum = 0.0f;
        float32_t valCnt = 0.0f;
        Vectr tripValPos = {0.0f,0.0f,0.0f};
        
        ValidVals = (((sig1.x > max_sig) | (sig1.x < min_sig)) | (sig1.x > max_trav) | (sig1.x < min_trav));
        ValidVals = ValidVals | ((((sig2.x > max_sig) | (sig2.x < min_sig)) | (sig2.x > max_trav) | (sig2.x < min_trav))<<1u);
        ValidVals = ValidVals | ((((sig3.x > max_sig) | (sig3.x < min_sig)) | (sig3.x > max_trav) | (sig3.x < min_trav))<<2u);
        ValidVals = ValidVals | (((abs(sig1.x - sig2.x) > deviation) & (abs(sig1.x - sig3.x) > deviation))<<3u);
        ValidVals = ValidVals | (((abs(sig2.x - sig1.x) > deviation) & (abs(sig2.x - sig3.x) > deviation))<<4u);     
        ValidVals = ValidVals | (((abs(sig3.x - sig1.x) > deviation) & (abs(sig3.x - sig2.x) > deviation))<<5u); 
        valSum =  (~((ValidVals & 1u) | ((ValidVals & 8u)>>3u))&1u) * sig1.x + (~((ValidVals & 2u) | ((ValidVals & 16u)>>4u))&1u) * sig2.x + (~((ValidVals & 4u) | ((ValidVals & 32u)>>5u))&1u) * sig3.x;
        valCnt = (~((ValidVals & 1u) | ((ValidVals & 8u)>>3u))&1u) * 1.0f + (~((ValidVals & 2u) | ((ValidVals & 16u)>>4u))&1u) * 1.0f + (~((ValidVals & 4u) | ((ValidVals & 32u)>>5u))&1u) * 1.0f;
        tripValPos.x = valSum / valCnt;    

        ValidVals = (((sig1.y > max_sig) | (sig1.y < min_sig)) | (sig1.y > max_trav) | (sig1.y < min_trav));
        ValidVals = ValidVals | ((((sig2.y > max_sig) | (sig2.y < min_sig)) | (sig2.y > max_trav) | (sig2.y < min_trav))<<1u);
        ValidVals = ValidVals | ((((sig3.y > max_sig) | (sig3.y < min_sig)) | (sig3.y > max_trav) | (sig3.y < min_trav))<<2u);
        ValidVals = ValidVals | (((abs(sig1.y - sig2.y) > deviation) & (abs(sig1.y - sig3.y) > deviation))<<3u);
        ValidVals = ValidVals | (((abs(sig2.y - sig1.y) > deviation) & (abs(sig2.y - sig3.y) > deviation))<<4u);     
        ValidVals = ValidVals | (((abs(sig3.y - sig1.y) > deviation) & (abs(sig3.y - sig2.y) > deviation))<<5u); 
        valSum =  (~((ValidVals & 1u) | ((ValidVals & 8u)>>3u))&1u) * sig1.y + (~((ValidVals & 2u) | ((ValidVals & 16u)>>4u))&1u) * sig2.y + (~((ValidVals & 4u) | ((ValidVals & 32u)>>5u))&1u) * sig3.y;
        valCnt = (~((ValidVals & 1u) | ((ValidVals & 8u)>>3u))&1u) * 1.0f + (~((ValidVals & 2u) | ((ValidVals & 16u)>>4u))&1u) * 1.0f + (~((ValidVals & 4u) | ((ValidVals & 32u)>>5u))&1u) * 1.0f;
        tripValPos.y = valSum / valCnt; 

        ValidVals = (((sig1.z > max_sig) | (sig1.z < min_sig)) | (sig1.z > max_trav) | (sig1.z < min_trav));
        ValidVals = ValidVals | ((((sig2.z > max_sig) | (sig2.z < min_sig)) | (sig2.z > max_trav) | (sig2.z < min_trav))<<1u);
        ValidVals = ValidVals | ((((sig3.z > max_sig) | (sig3.z < min_sig)) | (sig3.z > max_trav) | (sig3.z < min_trav))<<2u);
        ValidVals = ValidVals | (((abs(sig1.z - sig2.z) > deviation) & (abs(sig1.z - sig3.z) > deviation))<<3u);
        ValidVals = ValidVals | (((abs(sig2.z - sig1.z) > deviation) & (abs(sig2.z - sig3.z) > deviation))<<4u);     
        ValidVals = ValidVals | (((abs(sig3.z - sig1.z) > deviation) & (abs(sig3.z - sig2.z) > deviation))<<5u); 
        valSum =  (~((ValidVals & 1u) | ((ValidVals & 8u)>>3u))&1u) * sig1.z + (~((ValidVals & 2u) | ((ValidVals & 16u)>>4u))&1u) * sig2.z + (~((ValidVals & 4u) | ((ValidVals & 32u)>>5u))&1u) * sig3.z;
        valCnt = (~((ValidVals & 1u) | ((ValidVals & 8u)>>3u))&1u) * 1.0f + (~((ValidVals & 2u) | ((ValidVals & 16u)>>4u))&1u) * 1.0f + (~((ValidVals & 4u) | ((ValidVals & 32u)>>5u))&1u) * 1.0f;
        tripValPos.z = valSum / valCnt;  
        
        return tripValPos;       
}
                                                                                                           
/*----------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end common function library */