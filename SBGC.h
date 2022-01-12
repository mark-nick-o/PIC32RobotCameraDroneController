#ifndef  __SBGC__
#define  __SBGC__

#ifdef __cplusplus
 extern "C" {
#endif

/*
        SBGC.h : SimpleBGC Serial API  library - adjustable variables
        More info: http://www.basecamelectronics.com/serialapi/

  Copyright (c) 2014-2015 Aleksei Moskalenko
  v1.1 Modified ACP Aviation October 2019...
  Ported and expanded by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland

  All rights reserved.
        See license info in the SBGC.h
*/
/*
        SimpleBGC Serial API  library
        More info: http://www.basecamelectronics.com/serialapi/
        * compatible with the revision 2.4, 2.5 of Serial API specification.

        Copyright (c) 2014-2015 Aleksei Moskalenko
        All rights reserved.
        This software is free for use and redistribution under a BSD license:
        Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions are met:
            * Redistributions of source code must retain the above copyright
              notice, this list of conditions and the following disclaimer.
            * Redistributions in binary form must reproduce the above copyright
              notice, this list of conditions and the following disclaimer in the
              documentation and/or other materials provided with the distribution.
            * Neither the name of the Basecamelectronics company nor the
              names of its contributors may be used to endorse or promote products
              derived from this software without specific prior written permission.

        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
        ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
        WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
        DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
        DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
        (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
        LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
        ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
        (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
        SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

        All libraries are edited and updated by AirCamPro both for porting compatibility
        with mikroE C and for extra functionality
        Therefore to include new versions of any BaseCam library you need to check
        the differences.
*/
#define SYS_LITTLE_ENDIAN                                                       // !!!!Define the endianess of your machine!!!!

#ifdef ARDUINO_SBGC                                                             // If you are using arduino and the parser code in SBGC_PARSER.h
  #define SYS_STRUCT_PACKED                                                     // !!!!Uncomment only if structures are packed (use it for 8bit machines to save some program space)
  #if(defined(SYS_LITTLE_ENDIAN) && defined(SYS_STRUCT_PACKED))
      #define SBGC_CMD_STRUCT_ALIGNED                                           // Optimize commapnd packing if it's good aligned in the memory
  #endif
#endif

//#include <inttypes.h>

#include "inttypes.h"
#include "SBGC_command.h"
#include "SBGC_rc.h"
#include "SBGC_adj_vars.h"
#include "SBGC_parser.h"
#include "SBGC_cmd_helpers.h"

#define SBGC_NUM_CHECKSUM_BYTES 1U
#define SBGC_CORE_HEADER_LEN 3U                                                 ///< Length of core header (of the comm. layer): stx (1 byte) + command id (1 byte) + data size (1 byte)
#define SBGC_NUM_HEADER_BYTES (SBGC_CORE_HEADER_LEN + SBGC_NUM_CHECKSUM_BYTES)  ///< Length of all header bytes, including core and checksum
#define SBGC_NUM_NON_PAYLOAD_BYTES (SBGC_NUM_HEADER_BYTES + SBGC_NUM_CHECKSUM_BYTES)  // header plus payload checksum
#define SBGC_MAX_PACKET_LEN (SBGC_MAX_PAYLOAD_LEN + SBGC_NUM_NON_PAYLOAD_BYTES) ///< Maximum packet length

// Error codes in command response
#define SBGC_ERR_CMD_SIZE 1U
#define SBGC_ERR_WRONG_PARAMS 2U
#define SBGC_ERR_GET_DEVICE_ID  3U
#define SBGC_ERR_CRYPTO 4U
#define SBGC_ERR_CALIBRATE_BAT 5U
#define SBGC_ERR_UNKNOWN_COMMAND 6U

// errors from the PID2 autotune message
#define SBGC_ERR_READ_FROM_EEPROM 1U                                            // read from EEPROM failed (data is corrupted or empty)
#define SBGC_ERR_CANT_RUN_ALGORITHM 2U                                          // can't run algorithm at this moment
#define SBGC_ERR_WRITE_TO_EEPROM 3U                                             // write to EEPROM failed
#define SBGC_ERR_UNKNOWN 4U                                                     // unknown action
#define SBGC_ERR_WRONG_CMD_SIZE 5U                                              // wrong command size

// System error flags that controller may set
#define SBGC_SYS_ERR_NO_SENSOR (1<<0)
#define SBGC_SYS_ERR_CALIB_ACC (1<<1)
#define SBGC_SYS_ERR_SET_POWER (1<<2)
#define SBGC_SYS_ERR_CALIB_POLES (1<<3)
#define SBGC_SYS_ERR_PROTECTION (1<<4)
#define SBGC_SYS_ERR_SERIAL (1<<5)
#define SBGC_SYS_ERR_BAT_LOW (1<<6)
#define SBGC_SYS_ERR_BAT_CRITICAL (1<<7)
#define SBGC_SYS_ERR_GUI_VERSION (1<<8)
#define SBGC_SYS_ERR_MISS_STEPS (1<<9)
#define SBGC_SYS_ERR_SYSTEM (1<<10)
#define SBGC_SYS_ERR_EMERGENCY_STOP (1<<11)

// Trigger pins
#define SBGC_PIN_AUX1 16U
#define SBGC_PIN_AUX2 17U
#define SBGC_PIN_AUX3 18U
#define SBGC_PIN_BUZZER 32U
#define CMD_PIN_SSAT_POWER 33U                                                  // pin that control Spektrum Satellite 3.3V power line (low state enables line)

#define SBGC_SERVO_OUT_DISABLED -1                                              // Value passed in CMD_SERVO_OUT to disable servo output

// Menu actions (used in the SBGC_MENU_BUTTON_PRESS command, menu button assignment, RC_CMD channel assignment)
#define SBGC_MENU_PROFILE1 1U
#define SBGC_MENU_PROFILE2 2U
#define SBGC_MENU_PROFILE3 3U
#define SBGC_MENU_SWAP_PITCH_ROLL 4U
#define SBGC_MENU_SWAP_YAW_ROLL 5U
#define SBGC_MENU_CALIB_ACC 6U
#define SBGC_MENU_RESET 7U
#define SBGC_MENU_SET_ANGLE 8U
#define SBGC_MENU_CALIB_GYRO 9U
#define SBGC_MENU_MOTOR_TOGGLE 10U
#define SBGC_MENU_MOTOR_ON 11U
#define SBGC_MENU_MOTOR_OFF 12U
#define SBGC_MENU_FRAME_UPSIDE_DOWN 13U
#define SBGC_MENU_PROFILE4 14U
#define SBGC_MENU_PROFILE5 15U
#define SBGC_MENU_AUTO_PID 16U
#define SBGC_MENU_LOOK_DOWN 17U
#define SBGC_MENU_HOME_POSITION 18U
#define SBGC_MENU_RC_BIND 19U
#define SBGC_MENU_CALIB_GYRO_TEMP 20U
#define SBGC_MENU_CALIB_ACC_TEMP 21U
#define SBGC_MENU_BUTTON_PRESS 22U
#define SBGC_MENU_RUN_SCRIPT1 23U
#define SBGC_MENU_RUN_SCRIPT2 24U
#define SBGC_MENU_RUN_SCRIPT3 25U
#define SBGC_MENU_RUN_SCRIPT4 26U
#define SBGC_MENU_RUN_SCRIPT5 27U
#define SBGC_MENU_RUN_SCRIPT6 28U
#define SBGC_MENU_RUN_SCRIPT7 29U
#define SBGC_MENU_RUN_SCRIPT8 30U
#define SBGC_MENU_RUN_SCRIPT9 31U
#define SBGC_MENU_RUN_SCRIPT10 32U
#define SBGC_MENU_CALIB_MAG 33U
#define SBGC_MENU_LEVEL_ROLL_PITCH 34U
#define SBGC_MENU_CENTER_YAW 35U
#define SBGC_MENU_UNTWIST_CABLES 36U
#define SBGC_MENU_SET_ANGLE_NO_SAVE 37U
#define SBGC_MENU_HOME_POSITION_SHORTEST 38U
#define SBGC_MENU_CENTER_YAW_SHORTEST 39U

// Control modes
#define SBGC_CONTROL_MODE_NO 0U
#define SBGC_CONTROL_MODE_SPEED 1U
#define SBGC_CONTROL_MODE_ANGLE 2U
#define SBGC_CONTROL_MODE_SPEED_ANGLE 3U
#define SBGC_CONTROL_MODE_RC 4U
#define SBGC_CONTROL_MODE_ANGLE_REL_FRAME 5U

#define SBGC_CONTROL_MODE_MASK 0x0FU                                            // bits0..3 used for mode, other for flags

#define SBGC_CONTROL_MODE_FLAG_UNTWIST (1<<7)

#ifdef __cplusplus
}
#endif

#endif //__SBGC__