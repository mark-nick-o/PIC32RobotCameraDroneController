 /*
        SBGC_command.h : SimpleBGC Serial API  library - definition of commands
        More info: http://www.basecamelectronics.com/serialapi/

        Copyright (c) 2014-2015 Aleksei Moskalenko
        Ported and expanded by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland

        All rights reserved.
        See license info in the SBGC.h
*/
#ifndef  __SBGC_command__
#define  __SBGC_command__

#ifdef __cplusplus
 extern "C" {
#endif

//#include <inttypes.h>
#include "inttypes.h"
#include "SBGC_rc.h"

#define SBGC_NO_CONFIRM 0u
#define SBGC_CONFIRM 1u

// Size of header and checksums
#define SBGC_CMD_NON_PAYLOAD_BYTES 5u

// Max. size of a command after packing to bytes
#define SBGC_CMD_MAX_BYTES 255u

// Max. size of a payload data
#define SBGC_CMD_DATA_SIZE (SBGC_CMD_MAX_BYTES - SBGC_CMD_NON_PAYLOAD_BYTES)

// For MOTORS CMD
#define SBGC_MOTOR_OFF_NORMAL 0u
#define SBGC_MOTOR_ON 1u
#define SBGC_MOTOR_OFF_BREAK 1u
#define SBGC_MOTOR_OFF_SAFE 2u

#define SBGC_NO_CMD_FOUND 200u                                                   // defined this state as No Message Found.

////////////////////// Command ID definitions ////////////////

#define SBGC_CMD_READ_PARAMS 82u
#define SBGC_CMD_WRITE_PARAMS 87u
#define SBGC_CMD_REALTIME_DATA 68u
#define SBGC_CMD_BOARD_INFO 86u
#define SBGC_CMD_CALIB_ACC 65u
#define SBGC_CMD_CALIB_GYRO 103u
#define SBGC_CMD_CALIB_EXT_GAIN 71u
#define SBGC_CMD_USE_DEFAULTS 70u
#define SBGC_CMD_CALIB_POLES 80u
#define SBGC_CMD_RESET 114u
#define SBGC_CMD_HELPER_DATA 72u
#define SBGC_CMD_CALIB_OFFSET 79u
#define SBGC_CMD_CALIB_BAT 66u
#define SBGC_CMD_MOTORS_ON 77u
#define SBGC_CMD_MOTORS_OFF  109u
#define SBGC_CMD_CONTROL 67u
#define SBGC_CMD_TRIGGER_PIN  84u
#define SBGC_CMD_EXECUTE_MENU 69u
#define SBGC_CMD_GET_ANGLES 73u
#define SBGC_CMD_CONFIRM 67u

#define SBGC_CMD_CONTROL_CONFIG 90u

// Starting from board ver.3.0
#define SBGC_CMD_BOARD_INFO_3 20u
#define SBGC_CMD_READ_PARAMS_3 21u
#define SBGC_CMD_WRITE_PARAMS_3 22u
#define SBGC_CMD_REALTIME_DATA_3 23u
#define SBGC_CMD_SELECT_IMU_3 24u
#define SBGC_CMD_REALTIME_DATA_4 25u
#define SBGC_CMD_ENCODERS_CALIB_OFFSET_4 26u
#define SBGC_CMD_ENCODERS_CALIB_FLD_OFFSET_4 27u
#define SBGC_CMD_READ_PROFILE_NAMES 28u
#define SBGC_CMD_WRITE_PROFILE_NAMES 29u
#define SBGC_CMD_QUEUE_PARAMS_INFO_3 30u
#define SBGC_CMD_SET_ADJ_VARS_VAL 31u
#define SBGC_CMD_SAVE_PARAMS_3 32u
#define SBGC_CMD_READ_PARAMS_EXT 33u
#define SBGC_CMD_WRITE_PARAMS_EXT 34u
#define SBGC_CMD_AUTO_PID 35u
#define SBGC_CMD_SERVO_OUT 36u
#define SBGC_CMD_BODE_TEST_START_STOP 37u
#define SBGC_CMD_BODE_TEST_DATA 38u
#define SBGC_CMD_I2C_WRITE_REG_BUF 39u
#define SBGC_CMD_I2C_READ_REG_BUF 40u
#define SBGC_CMD_WRITE_EXTERNAL_DATA 41u
#define SBGC_CMD_READ_EXTERNAL_DATA 42u
#define SBGC_CMD_READ_ADJ_VARS_CFG 43u
#define SBGC_CMD_WRITE_ADJ_VARS_CFG 44u
#define SBGC_CMD_API_VIRT_CH_CONTROL 45u
#define SBGC_CMD_ADJ_VARS_STATE 46u
#define SBGC_CMD_EEPROM_WRITE 47u
#define SBGC_CMD_EEPROM_READ 48u
#define SBGC_CMD_CALIB_INFO 49u
#define SBGC_CMD_SIGN_MESSAGE_3 50u
#define SBGC_CMD_BOOT_MODE_3 51u
#define SBGC_CMD_SYSTEM_STATE 52u
#define SBGC_CMD_READ_FILE 53u
#define SBGC_CMD_WRITE_FILE 54u
#define SBGC_CMD_FS_CLEAR_ALL 55u
#define SBGC_CMD_AHRS_HELPER 56u
#define SBGC_CMD_RUN_SCRIPT 57u
#define SBGC_CMD_SCRIPT_DEBUG 58u
#define SBGC_CMD_CALIB_MAG 59u
#define SBGC_CMD_UART_BYPASS 60u
#define SBGC_CMD_GET_ANGLES_EXT 61u
#define SBGC_CMD_READ_PARAMS_EXT2 62u
#define SBGC_CMD_WRITE_PARAMS_EXT2 63u
#define SBGC_CMD_GET_ADJ_VARS_VAL 64u
#define SBGC_CMD_CALIB_MOTOR_MAG_LINK 74u
#define SBGC_CMD_GYRO_CORRECTION 75u
#define SBGC_CMD_DATA_STREAM_INTERVAL 85u
#define SBGC_CMD_REALTIME_DATA_CUSTOM 88u
#define SBGC_CMD_BEEP_SOUND 89u
#define SBGC_CMD_ENCODERS_CALIB_OFFSET_4 26u
#define SBGC_CMD_ENCODERS_CALIB_FLD_OFFSET_4 27u
#define SBGC_CMD_CONTROL_CONFIG 90u
#define SBGC_CMD_CALIB_ORIENT_CORR 91u
#define SBGC_CMD_COGGING_CALIB_INFO 92u
#define SBGC_CMD_CALIB_COGGING 93u
#define SBGC_CMD_CALIB_ACC_EXT_REF 94u
#define SBGC_CMD_PROFILE_SET 95u
#define SBGC_CMD_CAN_DEVICE_SCAN 96u
#define SBGC_CMD_CAN_DRV_HARD_PARAMS 97u
#define SBGC_CMD_CAN_DRV_STATE 98u
#define SBGC_CMD_CAN_DRV_CALIBRATE 99u
#define SBGC_CMD_READ_RC_INPUTS 100u
#define SBGC_CMD_REALTIME_DATA_CAN_DRV 101u
#define SBGC_CMD_EVENT 102u
#define SBGC_CMD_READ_PARAMS_EXT3 104u
#define SBGC_CMD_WRITE_PARAMS_EXT3 105u
#define SBGC_CMD_EXT_IMU_DEBUG_INFO 106u
#define SBGC_CMD_SET_DEVICE_ADDR 107u
#define SBGC_CMD_AUTO_PID2 108u
#define SBGC_CMD_EXT_IMU_CMD 110u
#define SBGC_CMD_READ_STATE_VARS 111u
#define SBGC_CMD_WRITE_STATE_VARS 112u
#define SBGC_CMD_SERIAL_PROXY 113u
#define SBGC_CMD_IMU_ADVANCED_CALIB 115u
#define SBGC_CMD_API_VIRT_CH_HIGH_RES 116u
#define CMD_SET_DEBUG_PORT 249u
#define CMD_MAVLINK_INFO 250u
#define CMD_MAVLINK_DEBUG 25u
#define SBGC_CMD_DEBUG_VARS_INFO_3 253u
#define SBGC_CMD_DEBUG_VARS_3 254u
#define SBGC_CMD_ERROR 255u

// When requesting the board info define resend time
#define SBGC_FIRMWR_RSND_DELAY 2000u                                            // Time to wait in ticks for a good status to comeback after a reply
#define SBGC_FIRMWR_NOREP_DELAY 10000u                                          // Time in ticks to wait until resending the other firmware version request

#ifdef __cplusplus
}
#endif

#endif //__SBGC_command__