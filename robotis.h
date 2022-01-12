#ifndef __Robotis_
#define __Robotis_
/*
  implementation of Robotis Dynamixel 2.0 protocol for controlling servos

  Portions of this code are based on the dynamixel_sdk code:
  https://github.com/ROBOTIS-GIT/DynamixelSDK
  which is under the following license:

* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* Ported to mikroE C for PIC32 by ACP Aviation
*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define BROADCAST_ID 0xFEu
#define MAX_ID 0xFCu

// DXL protocol common commands
#define INST_PING          1u
#define INST_READ          2u
#define INST_WRITE         3u
#define INST_REG_WRITE     4u
#define INST_ACTION        5u
#define INST_FACTORY_RESET 6u
#define INST_CLEAR        16u
#define INST_SYNC_WRITE  131u
#define INST_BULK_READ   146u

// 2.0 protocol commands
#define INST_REBOOT       8u
#define INST_STATUS      85u
#define INST_SYNC_READ  130u
#define INST_BULK_WRITE 147u

// 2.0 protocol packet offsets
#define PKT_HEADER0     0u
#define PKT_HEADER1     1u
#define PKT_HEADER2     2u
#define PKT_RESERVED    3u
#define PKT_ID          4u
#define PKT_LENGTH_L    5u
#define PKT_LENGTH_H    6u
#define PKT_INSTRUCTION 7u
#define PKT_ERROR       8u
#define PKT_PARAMETER0  8u

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xffu)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xffu))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffffu)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffffu))) << 16))
#define DXL_LOWORD(l) ((uint16_t)(((uint64_t)(l)) & 0xffffu))
#define DXL_HIWORD(l) ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffffu))
#define DXL_LOBYTE(w) ((uint8_t)(((uint64_t)(w)) & 0xffu))
#define DXL_HIBYTE(w) ((uint8_t)((((uint64_t)(w)) >> 8) & 0xffu))

// register offsets
#define REG_OPERATING_MODE 11u
#define OPMODE_CURR_CONTROL 0u
#define OPMODE_VEL_CONTROL 1u
#define OPMODE_POS_CONTROL 3u
#define OPMODE_EXT_POS_CONTROL 4u

#define REG_TORQUE_ENABLE  64u

#define REG_STATUS_RETURN  68u
#define STATUS_RETURN_NONE 0u
#define STATUS_RETURN_READ 1u
#define STATUS_RETURN_ALL  2u

#define REG_GOAL_POSITION 116u

#define CONFIGURE_SERVO_COUNT 4u                                                // how many times to send servo configure msgs

#define DETECT_SERVO_COUNT 4u                                                   // how many times to send servo detection

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif