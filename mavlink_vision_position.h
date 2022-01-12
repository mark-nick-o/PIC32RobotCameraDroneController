#ifndef __vis_posn_calc_mavlink_
#define __vis_posn_calc_mavlink_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* =============================================================================
 Copyright (C) 2020 ACP Aviation this contains code and ideas taken from projects by
 Copyright (C) 2018 Kaz Kojima  (C) 2016 by Laurent Itti

 This file is part of PMLPS program.  This program is free
 software; you can redistribute it and/or modify it under the
 terms of the GNU General Public License as published by the
 Free Software Foundation; either version 3, or (at your option)
 any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program;
 see the files COPYING and EXCEPTION respectively.
  =========================================================================== */
#include "Struts.h"
#include "mavlink_packet_parser.h"                                              /* general function for parsing the packet of data recieved */
#if (MAV_VER == 1u)
//#include "msg1_vision_position_delta.h"
#include "msg1_vision_position_estimate.h"
#include "msg1_vision_speed_est.h"
#include "msg1_request_data_stream.h"
#include "msg1_attitude.h"
#include "msg1_vicon_pose_est.h"
#include "msg1_set_gps_global_origin.h"
#include "msg1_pose.h"
#include "msg1_heartbeat.h"
#else
#include "msg2_vision_position_delta.h"
#include "msg2_vision_position_estimate.h"
#include "msg2_vision_position_speed_estimate.h"
#include "msg2_request_data_stream.h"
#include "msg2_attitude.h"
#include "msg2_vicon_pose_est.h"
#include "msg2_set_gps_global_origin.h"
#include "msg2_pose.h"
#include "msg2_heartbeat.h"
#endif        
#include "mavlink_packet_parser.h"

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define MVVISPOSPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define MVVISPOSPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define MVVISPOSPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define MVVISPOSPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define MVVISPOSPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#if defined(D_FT900)
typedef struct MVVISPOSPACKED {
  uint8_t hb_count;
  uint8_t request_sent : 1u;
  uint8_t origin_sent : 1u;
  uint8_t spare : 6u;
  uint32_t tick_Ref;
  int32_t getTickCount;
  mavlink_attitude_t attitude;
  mavlink_heartbeat_t heart;
} optFloVisRcvData_t;
#else
MVVISPOSPACKED(
typedef struct {
  uint8_t hb_count;
  uint8_t request_sent : 1u;
  uint8_t origin_sent : 1u;
  uint8_t spare : 6u;
  uint32_t tick_Ref;
  int32_t getTickCount;
  mavlink_attitude_t attitude;
  mavlink_heartbeat_t heart;
}) optFloVisRcvData_t;
#endif

/* ####################################################################################################
   Processing functions : for optical flow to mavlink or optical flow from mavlink
   #################################################################################################### */
void pmlps_frame_rotate(float32_t a, float32_t b, float32_t alpha, float32_t *x, float32_t *y);
float32_t pmlps_angle_mod(float32_t alpha);
void defaultCoVarparams( opt_flow_coVar_t *coVar );
mat44 mat44from4quats(quat a, quat b, quat c, quat d);
float32_t mat44dot(mat44 m, mat44 m1);
mat44 mat44scl(mat44 m1, float32_t scl);
mat44 quat2mat44(quat q);
void setUpCamOrientation( T265Orientation_t *ori, const opt_flow_cam_config_t cam );
void sendVisPosEstimateMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_vision_position_estimate_t *visDeltaData );
void sendVisPosSpeedEstimateMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_vision_speed_estimate_t *visDeltaData );
void mavlink_package_speed_estimate( uint8_t target_sys, uint8_t target_comp, opt_flow_coVar_t *coVar, T265Pose_t *t265, mavlink_vision_speed_estimate_t *mavMsg );
void sendReqDataStreamMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_request_data_stream_t *visDeltaData, uint8_t cmd, uint8_t rate_hz );
void sendTextUpdateMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_statustext_t *visDeltaData, uint8_t sev, const opt_flow_coVar_t trakConf );
void mavlink_package_est_position( uint8_t target_sys, uint8_t target_comp, opt_flow_coVar_t *coVar, pmlpsEstimatedPosition_t *pos, mavlink_vision_position_estimate_t *mavMsg, float32_t cam_direction, mavlink_attitude_t att, const opt_flow_cam_config_t cam );
void sendViconPosEstimateMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_vicon_position_estimate_t *visPosData );
void mavlink_package_vicon_pose( uint8_t target_sys, uint8_t target_comp, opt_flow_coVar_t *coVar, pmlpsEstimatedPosition_t *pos, mavlink_vicon_position_estimate_t *mavMsg, float32_t cam_direction, mavlink_attitude_t att, const opt_flow_cam_config_t cam );
void correct4CamOrientation( T265Orientation_t *ori, const T265Pose_t *t265 );
void sendErrorWrongVerMavlink( uint8_t target_sys, uint8_t target_comp, uint8_t sev, uint8_t servMav, uint8_t clientMav );
void sendHeartbeatMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_heartbeat_t *heartbeat );
void sendOptFloAltitudeMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_altitudes_t *visAltData, uint8_t altit );
void sendStateQuatPoseMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_hil_state_quaternion_t *visPoseData );
void mavlink_package_pose( uint8_t target_sys, uint8_t target_comp, const T265Pose_t pose, mavlink_hil_state_quaternion_t *mav );
void sendGPSOriginMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_set_gps_global_origin_t *gpsOrg );
#if (MAV_VER == 2u)
void sendVisPosDeltaToMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_vision_position_delta_t *visDeltaData );
void mavlink_package_position_delta( uint8_t target_sys, uint8_t target_comp, pmlpsEstimatedPosition_t *pos, mavlink_vision_position_delta_t *delta, float32_t cam_direction, mavlink_attitude_t att );
#endif
void rcvMavData( optFloVisRcvData_t *rcv );
/*-----------------------------------------------------------------------------
 *      pmlps_frame_rotate():  frame rotate
 *
 *  Parameters: float32_t a, float32_t b, float32_t alpha, float32_t *x, float32_t *y 
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void pmlps_frame_rotate(float32_t a, float32_t b, float32_t alpha, float32_t *x, float32_t *y)
{
  *x = cos(alpha)*a - sin(alpha)*b;
  *y = -(sin(alpha)*a + cos(alpha)*b);
}
/*-----------------------------------------------------------------------------
 *      pmlps_angle_mod():  angle wrap around
 *
 *  Parameters: float32_t alpha
 *       
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t pmlps_angle_mod(float32_t alpha)
{
  if (alpha < -PI)
    alpha += 2.0f*PI;
  else if (alpha > PI)
    alpha -= 2.0f*PI;
  return alpha;
}
/*-----------------------------------------------------------------------------
 *      mat44from4quats():  make 4 quaternions a 4x4 matrix 
 *
 *  Parameters: quat a, quat b, quat c, quat d
 *       
 *  Return:     mat44
 *----------------------------------------------------------------------------*/
mat44 mat44from4quats(quat a, quat b, quat c, quat d) 
{
        mat44 m;
        m.m[0u][0u] = a.x;
        m.m[0u][1u] = a.y;
        m.m[0u][2u] = a.z;
        m.m[0u][3u] = a.w;
        m.m[1u][0u] = b.x;
        m.m[1u][1u] = b.y;
        m.m[1u][2u] = b.z;
        m.m[1u][3u] = b.w;
        m.m[2u][0u] = c.x;
        m.m[2u][1u] = c.y;
        m.m[2u][2u] = c.z;
        m.m[2u][3u] = c.w;   
        m.m[3u][0u] = d.x;
        m.m[3u][1u] = d.y;
        m.m[3u][2u] = d.z;
        m.m[3u][3u] = d.w;     
        return m;
}
/*-----------------------------------------------------------------------------
 *      mat44dot():  dot product of 2 4x4 matrix 
 *
 *  Parameters: mat44 m, mat44 m1
 *       
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t mat44dot(mat44 m, mat44 m1) 
{
   return (m.m[0u][0u] * m1.m[0u][0u]) + (m.m[0u][1u] * m1.m[0u][1u]) + (m.m[0u][2u] * m1.m[0u][2u]) + (m.m[0u][3u] * m1.m[0u][3u]) + (m.m[1u][0u] * m1.m[1u][0u]) + (m.m[1u][0u] * m1.m[1u][0u]) + (m.m[1u][1u] * m1.m[1u][1u]) + (m.m[1u][2u] * m1.m[1u][2u]) + (m.m[1u][3u] * m1.m[1u][3u]) + (m.m[2u][0u] * m1.m[2u][0u]) + (m.m[2u][1u] * m1.m[2u][1u]) + (m.m[2u][2u] * m1.m[2u][2u]) + (m.m[2u][3u] * m1.m[2u][3u]) + (m.m[3u][0u] * m1.m[3u][0u]) + (m.m[3u][1u] * m1.m[3u][1u]) + (m.m[3u][2u] * m1.m[3u][2u]) + (m.m[3u][3u] * m1.m[3u][3u]);
}
/*-----------------------------------------------------------------------------
 *      mat44scl():  scale a 4x4 rotation matrix by the scalar variable
 *
 *  Parameters: mat44 m1, float32_t scl
 *       
 *  Return:     mat44
 *----------------------------------------------------------------------------*/
mat44 mat44scl(mat44 m1, float32_t scl) 
{
    mat44 m;
    m.m[0u][0u] = m1.m[0u][0u] * scl;
    m.m[0u][1u] = m1.m[0u][1u] * scl;
    m.m[0u][2u] = m1.m[0u][2u] * scl; 
    m.m[0u][3u] = m1.m[0u][3u] * scl; 
    m.m[1u][0u] = m1.m[1u][0u] * scl; 
    m.m[1u][0u] = m1.m[1u][0u] * scl; 
    m.m[1u][1u] = m1.m[1u][1u] * scl; 
    m.m[1u][2u] = m1.m[1u][2u] * scl; 
    m.m[1u][3u] = m1.m[1u][3u] * scl; 
    m.m[2u][0u] = m1.m[2u][0u] * scl; 
    m.m[2u][1u] = m1.m[2u][1u] * scl; 
    m.m[2u][2u] = m1.m[2u][2u] * scl; 
    m.m[2u][3u] = m1.m[2u][3u] * scl; 
    m.m[3u][0u] = m1.m[3u][0u] * scl; 
    m.m[3u][1u] = m1.m[3u][1u] * scl; 
    m.m[3u][2u] = m1.m[3u][2u] * scl; 
    m.m[3u][3u] = m1.m[3u][3u] * scl;
    return m;
}
/*-----------------------------------------------------------------------------
 *      quat2mat44():  convert a quaternion into a 4x4 rotation matrix.
 *
 *  Parameters: quat q
 *       
 *  Return:     mat44
 *----------------------------------------------------------------------------*/
mat44 quat2mat44(quat q) 
{
        // from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        float32_t x = q.x;
        float32_t y = q.y;
        float32_t z = q.z;
        float32_t w = q.w;

        mat44 m;
        m.m[0u][0u] = 1 - 2*y*y - 2*z*z;
        m.m[0u][1u] = 2*x*y - 2*z*w;
        m.m[0u][2u] = 2*x*z + 2*y*w;
        m.m[0u][3u] = 0.0f;
        m.m[1u][0u] = 2*x*y + 2*z*w;
        m.m[1u][1u] = 1 - 2*x*x - 2*z*z;
        m.m[1u][2u] = 2*y*z - 2*x*w;
        m.m[1u][3u] = 0.0f;
        m.m[2u][0u] = 2*x*z - 2*y*w;
        m.m[2u][1u] = 2*y*z + 2*x*w;
        m.m[2u][2u] = 1 - 2*x*x - 2*y*y;
        m.m[2u][3u] = 0.0f;
        return m;
}
/*-----------------------------------------------------------------------------
 *      setUpCamOrientation():  set up camera orientations for use in optical
 *                              flow calculations
 *
 *  Parameters: T265Orientation_t *ori, const opt_flow_cam_config_t cam
 *       
 *  Return:     void
 *----------------------------------------------------------------------------*/
void setUpCamOrientation( T265Orientation_t *ori, const opt_flow_cam_config_t cam )
{

   ori->H_aeroRef_T265Ref = mat44from4quats( mkquat(0.0f,0.0f,-1.0f,0.0f),mkquat(1.0f,0.0f,0.0f,0.0f),mkquat(0.0f,-1.0f,0.0f,0.0f),mkquat(0.0f,0.0f,0.0f,1.0f) );  /* np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]]) */
   
   switch (cam.camera_orientation)
   {
      case FwdPortRight:                                                        /* Forward, USB port to the right */
      /*     H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref) */
      if (matrix44_inverse(&ori->inv_aeroRef_T265Ref, ori->H_aeroRef_T265Ref)==true)
        ori->H_T265body_aeroBody = ori->inv_aeroRef_T265Ref;
      break;
   
      case DwnPortRight:                                                        /* Downfacing, USB port to the right */
      ori->H_T265body_aeroBody = mat44from4quats( mkquat(0.0f,1.0f,0.0f,0.0f),mkquat(1.0f,0.0f,0.0f,0.0f),mkquat(0.0f,0.0f,-1.0f,0.0f),mkquat(0.0f,0.0f,0.0f,1.0f) );
      break;
      
      case Fwd45:                                                               /* 45degree forward */
      /*     H_T265body_aeroBody = (tf.euler_matrix(m.pi/4, 0, 0)).dot(np.linalg.inv(H_aeroRef_T265Ref)) */
      if (matrix44_inverse(&ori->inv_aeroRef_T265Ref, ori->H_aeroRef_T265Ref)==true)
         ori->H_T265body_aeroBody = m44scl( matrix33_dot(Matrix33_from_euler((PI/4.0f), 0.0f, 0.0f)), ori->inv_aeroRef_T265Ref ); 
      break;

      case Bck45:                                                               /* 45degree backward */
      /*     H_T265body_aeroBody = (tf.euler_matrix(-m.pi/4, 0, 0)).dot(np.linalg.inv(H_aeroRef_T265Ref)) */
      /* ------ TO DO :: -------- */
      /* ------ this is what it would be for 3x3 matrix we need to do the same for 4x4 matrix ------ */
      /* if (matrix33_inverse(&inv_aeroRef_T265Ref, ori->H_aeroRef_T265Ref)==true)
         ori->H_T265body_aeroBody = mscl( matrix33_dot( inv_aeroRef_T265Ref ), Matrix3_from_euler((m.pi/4.0f), 0.0f, 0.0f) ); */
      if (matrix44_inverse(&ori->inv_aeroRef_T265Ref, ori->H_aeroRef_T265Ref)==true)
         ori->H_T265body_aeroBody = m44scl( matrix33_dot(Matrix33_from_euler(-(PI/4.0f), 0.0f, 0.0f)), ori->inv_aeroRef_T265Ref ); 
      break;
            
      default:                                                                  /* Default is facing forward, USB port to the right */
      /*     H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref) */
      break;
   }
}

/*-----------------------------------------------------------------------------
 *      sendHeartbeatMavlink():  send a heartbeat for mavlink
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, mavlink_heartbeat_t *heartbeat
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendHeartbeatMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_heartbeat_t *heartbeat )
{
  
#ifdef MAV_LIB_GENERATED
  uint8_t buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];                                    /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  
  if (heartbeat == NULL)                                                        /* if null was passed return */
    return;

  heartbeat->type = MAV_TYPE_ONBOARD_CONTROLLER;
  heartbeat->autopilot = MAV_AUTOPILOT_GENERIC;
  heartbeat->base_mode = MAV_MODE_GUIDED_ARMED;
  heartbeat->custom_mode = 0u;
  heartbeat->system_status = MAV_STATE_ACTIVE;
#if (MAV_VER == 1)
  heartbeat->mavlink_version = 1u;
#elif (MAV_VER == 2)
  heartbeat->mavlink_version = 2u;
#endif
  len = mavlink_msg_heartbeat_encode(target_sys, target_comp, &message, heartbeat);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_HEARTBEAT_LEN );          // Send message you write to your serial UART_Write
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_HEARTBEAT_LEN );             // Send message you write to your UDP Port
#endif
}

/*-----------------------------------------------------------------------------
 *      sendGPSOriginMavlink():  wrap and send gps origin message
 *                                   from the optical flow device
 *                               If you cant get one you need to use fake data
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, mavlink_set_gps_global_origin_t *gpsOrg
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendGPSOriginMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_set_gps_global_origin_t *gpsOrg )
{
  
#ifdef MAV_LIB_GENERATED

  uint8_t buf[MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN];                        /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  
  if (gpsOrg == NULL)                                                           /* if null was passed return */
    return;

  len = mavlink_msg_set_gps_global_origin_encode(target_sys, target_comp, &message, gpsOrg);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN );          // Send message you write to your serial UART_Write
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN );             // Send message you write to your UDP Port
#endif
}

#if (MAV_VER == 2u)
/*-----------------------------------------------------------------------------
 *      sendVisPosDeltaToMavlink():  wrap and send vision position delta message
 *                                   from the optical flow device
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, mavlink_vision_position_delta_t *visDeltaData
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendVisPosDeltaToMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_vision_position_delta_t *visDeltaData )
{
  
#ifdef MAV_LIB_GENERATED

  uint8_t buf[MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN];                             /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  
  if (visDeltaData == NULL)                                                          /* if null was passed return */
    return;

  len = mavlink_msg_vision_position_delta_encode(target_sys, target_comp, &message, visDeltaData);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN );          // Send message you write to your serial UART_Write
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN );             // Send message you write to your UDP Port
#endif
}

/*-----------------------------------------------------------------------------
 *      mavlink_package_position_delta():  Fill delta with updated position and current attitude
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, pmlpsEstimatedPosition_t *pos, 
 *              mavlink_vision_position_delta_t *delta, float32_t cam_direction, mavlink_attitude_t att
 *       
 *  Return:     void
 *----------------------------------------------------------------------------*/
void mavlink_package_position_delta( uint8_t target_sys, uint8_t target_comp, pmlpsEstimatedPosition_t *pos, mavlink_vision_position_delta_t *delta, float32_t cam_direction, mavlink_attitude_t att )
{
   float32_t fx, fy, fz;
   float32_t yaw_direction_offset = cam_direction;

   //mavlink_vision_position_delta_t delta;

  if ((pos == NULL) || (delta == NULL))
  {   return; }
  else
  {           
          delta->time_usec = pos->time;
          delta->time_delta_usec = delta->time_usec - pos->prev_timestamp;

          delta->angle_delta[0u] = pmlps_angle_mod(att.roll - pos->prev_angle[0u]);   /* --- set angle delta to mavlink --- */
          delta->angle_delta[1u] = pmlps_angle_mod(att.pitch - pos->prev_angle[1u]);
          delta->angle_delta[2u] = pmlps_angle_mod(pos->yaw - pos->prev_angle[2u]);

          pmlps_frame_rotate(pos->position.x - pos->prev_pos[0u], pos->position.y - pos->prev_pos[1u], pos->yaw + yaw_direction_offset, &fx, &fy);
          fz = pos->position.z - pos->prev_pos[2u];

          delta->position_delta[0u] = fx;                                       /* --- set position delta to mavlink --- */
          delta->position_delta[1u] = fy;
          delta->position_delta[2u] = fz;
          delta->confidence = pos->SQUAL;
          // ----------------- look into this send struct e.g #define MAVLINK_USE_CONVENIENCE_FUNCTIONS 1 ----- mavlink_msg_vision_position_delta_send_struct(MAVLINK_COMM_0, &delta);
          pos->prev_timestamp = delta->time_usec;

          pos->prev_angle[0u] = att.roll;                                       /* --- remember the pose for the next iteration --- */
          pos->prev_angle[1u] = att.pitch;
          pos->prev_angle[2u] = pos->yaw;
          pos->prev_pos[0u] = pos->position.x;
          pos->prev_pos[1u] = pos->position.y;
          pos->prev_pos[2u] = pos->position.z;
          sendVisPosDeltaToMavlink( target_sys, target_comp, delta );
  }
}

#endif /* end of MAV2 only functionality */
/*-----------------------------------------------------------------------------
 *      sendVisPosEstimateMavlink():  wrap and send vision position estimate message
 *                                   from the optical flow device
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, mavlink_vision_position_estimate_t *visDeltaData
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendVisPosEstimateMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_vision_position_estimate_t *visDeltaData )
{
  
#ifdef MAV_LIB_GENERATED

  uint8_t buf[MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN];                     /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  
  if (visDeltaData == NULL)                                                     /* if null was passed return */
    return;

  len = mavlink_msg_vision_position_estimate_encode(target_sys, target_comp, &message, visDeltaData);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN );       // Send message you write to your serial UART_Write
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN );          // Send message you write to your UDP Port
#endif
}
/*-----------------------------------------------------------------------------
 *      defaultCoVarparams():  default parameters for co-variance matrix
 *
 *  Parameters: opt_flow_coVar_t *coVar 
 *       
 *  Return:     void
 *----------------------------------------------------------------------------*/
void defaultCoVarparams( opt_flow_coVar_t *coVar )
{ 
   coVar->linear_accel_cov = 0.01f;
   coVar->angular_vel_cov = 0.01f;
}
/*-----------------------------------------------------------------------------
 *      sendStateQuatPoseMavlink():  wrap and send vision pose estimate message
 *                                   from the optical flow device
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, mavlink_hil_state_quaternion_t *visPoseData
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendStateQuatPoseMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_hil_state_quaternion_t *visPoseData )
{
  
#ifdef MAV_LIB_GENERATED

  uint8_t buf[MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN];                     /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  
  if (visPoseData == NULL)                                                     /* if null was passed return */
    return;

  len = mavlink_msg_hil_state_quaternion_encode(target_sys, target_comp, &message, visPoseData);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN );       // Send message you write to your serial UART_Write
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN );          // Send message you write to your UDP Port
#endif
}
/*-----------------------------------------------------------------------------
 *      mavlink_package_pose():  package and send pose from T265 over mavlink
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, const T265Pose_t pose, 
 *              mavlink_hil_state_quaternion_t *mav 
 *       
 *  Return:     void
 *----------------------------------------------------------------------------*/
void mavlink_package_pose( uint8_t target_sys, uint8_t target_comp, const T265Pose_t pose, mavlink_hil_state_quaternion_t *mav )
{
    mav->attitude_quaternion[0u] = pose.rotation.x;
    mav->attitude_quaternion[1u] = pose.rotation.y; 
    mav->attitude_quaternion[2u] = pose.rotation.z;  
    mav->attitude_quaternion[3u] = pose.rotation.w; 
    mav->lat = pose.translation.x;
    mav->lon = pose.translation.y;
    mav->alt = pose.translation.z;
    mav->rollspeed = pose.velocity.x;
    mav->pitchspeed = pose.velocity.y;
    mav->yawspeed = pose.velocity.z;
    sendStateQuatPoseMavlink( target_sys, target_comp, mav );
}
/*-----------------------------------------------------------------------------
 *      mavlink_package_est_position():  package and send positon estimate over mavlink
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, opt_flow_coVar_t *coVar, pmlpsEstimatedPosition_t *pos, 
 *              mavlink_vision_position_estimate_t *mavMsg, float32_t cam_direction, 
 *              mavlink_attitude_t att, const opt_flow_cam_config_t cam
 *       
 *  Return:     void
 *----------------------------------------------------------------------------*/
void mavlink_package_est_position( uint8_t target_sys, uint8_t target_comp, opt_flow_coVar_t *coVar, pmlpsEstimatedPosition_t *pos, mavlink_vision_position_estimate_t *mavMsg, float32_t cam_direction, mavlink_attitude_t att, const opt_flow_cam_config_t cam )
{
   float32_t fx, fy, fz;
   float32_t yaw_direction_offset = cam.cam_direction;
#if (MAV_VER == 2u)
   int8_t i;
#endif
   
  if ((pos == NULL) || (mavMsg == NULL))
  {   return; }
  else
  {           
          mavMsg->usec = pos->time;

          pmlps_frame_rotate(pos->position.x, pos->position.y , yaw_direction_offset, &fx, &fy);
          fz = pos->position.z - cam.cam_height / 100.0f;

          mavMsg->x = fx;                                                       /* --- set position to mavlink --- */
          mavMsg->y = fy;
          mavMsg->z = fz;
          pos->prev_timestamp = mavMsg->usec;

#if (MAV_VER == 2u)
/*
            # Setup covariance data, which is the upper right triangle of the covariance matrix, see here: https://files.gitter.im/ArduPilot/VisionProjects/1DpU/image.png
            # Attemp #01: following this formula https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/src/base_realsense_node.cpp#L1406-L1411 */
                        
          coVar->cov_pose = coVar->linear_accel_cov * pow(10.0f, 3.0f - coVar->tracker_confidence);   // confidence is 0-3 value 
          coVar->cov_twist = coVar->angular_vel_cov  * pow(10.0f, 1.0f - coVar->tracker_confidence);
          mavMsg->covariance[0u] = coVar->cov_pose; 
          for (i=1;i<=5;i++)
                mavMsg->covariance[i] = 0.0f;
          mavMsg->covariance[6u] = coVar->cov_pose;
          for (i=7;i<=10;i++)
                mavMsg->covariance[i] = 0.0f;                   
          mavMsg->covariance[11u] = coVar->cov_pose;
          for (i=12;i<=14;i++)
                mavMsg->covariance[i] = 0.0f; 
          mavMsg->covariance[15u] = coVar->cov_twist;        
          for (i=16;i<=17;i++)
                mavMsg->covariance[i] = 0.0f;        
          mavMsg->covariance[18u] = coVar->cov_twist;
          mavMsg->covariance[19u] = 0.0f;
          mavMsg->covariance[20u] = coVar->cov_twist;
#endif

          mavMsg->roll = att.roll;                                              /* --- remember the pose for the next iteration --- */
          mavMsg->pitch = att.pitch;
          mavMsg->yaw = pos->yaw;
          pos->prev_angle[0u] = att.roll;                                       /* --- remember the pose for the next iteration --- */
          pos->prev_angle[1u] = att.pitch;
          pos->prev_angle[2u] = pos->yaw;
          pos->prev_pos[0u] = pos->position.x;
          pos->prev_pos[1u] = pos->position.y;
          pos->prev_pos[2u] = pos->position.z;
          sendVisPosEstimateMavlink( target_sys, target_comp, mavMsg );
  }
}
/*-----------------------------------------------------------------------------
 *      sendViconPosEstimateMavlink():  wrap and send vision position estimate message
 *                                      from the optical flow device with co-variance matrix
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, mavlink_vicon_position_estimate_t *visPosData
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendViconPosEstimateMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_vicon_position_estimate_t *visPosData )
{
  
#ifdef MAV_LIB_GENERATED

  uint8_t buf[MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN];                     /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  
  if (visPosData == NULL)                                                     /* if null was passed return */
    return;

  len = mavlink_msg_vicon_position_estimate_encode(target_sys, target_comp, &message, visPosData);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN );       // Send message you write to your serial UART_Write
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN );          // Send message you write to your UDP Port
#endif
}

/*-----------------------------------------------------------------------------
 *      mavlink_package_vicon_pose():  wrap and send vision position estimate message
 *                                     from the optical flow device with co-variance matrix
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, opt_flow_coVar_t *coVar, 
 *              pmlpsEstimatedPosition_t *pos, mavlink_vicon_position_estimate_t *mavMsg, 
 *             float32_t cam_direction, mavlink_attitude_t att, const opt_flow_cam_config_t cam      
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void mavlink_package_vicon_pose( uint8_t target_sys, uint8_t target_comp, opt_flow_coVar_t *coVar, pmlpsEstimatedPosition_t *pos, mavlink_vicon_position_estimate_t *mavMsg, float32_t cam_direction, mavlink_attitude_t att, const opt_flow_cam_config_t cam )
{
   float32_t fx, fy, fz;
   float32_t yaw_direction_offset = cam.cam_direction;
#if (MAV_VER == 2u)
   int8_t i;
#endif
   
  if ((pos == NULL) || (mavMsg == NULL))
  {   return; }
  else
  {           
          mavMsg->usec = pos->time;

          pmlps_frame_rotate(pos->position.x, pos->position.y , yaw_direction_offset, &fx, &fy);
          fz = pos->position.z - cam.cam_height / 100.0f;

          mavMsg->x = fx;                                                       /* --- set position to mavlink --- */
          mavMsg->y = fy;
          mavMsg->z = fz;
          pos->prev_timestamp = mavMsg->usec;

          mavMsg->roll = att.roll;                                              /* --- remember the pose for the next iteration --- */
          mavMsg->pitch = att.pitch;
          mavMsg->yaw = pos->yaw;
          pos->prev_angle[0u] = att.roll;                                       /* --- remember the pose for the next iteration --- */
          pos->prev_angle[1u] = att.pitch;
          pos->prev_angle[2u] = pos->yaw;
          pos->prev_pos[0u] = pos->position.x;
          pos->prev_pos[1u] = pos->position.y;
          pos->prev_pos[2u] = pos->position.z;

#if (MAV_VER == 2u)
/*
            # Setup covariance data, which is the upper right triangle of the covariance matrix, see here: https://files.gitter.im/ArduPilot/VisionProjects/1DpU/image.png
            # Attemp #01: following this formula https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/src/base_realsense_node.cpp#L1406-L1411 */
                        
          coVar->cov_pose = coVar->linear_accel_cov * pow(10.0f, 3.0f - coVar->tracker_confidence);
          coVar->cov_twist = coVar->angular_vel_cov  * pow(10.0f, 1.0f - coVar->tracker_confidence);
          mavMsg->covariance[0u] = coVar->cov_pose; 
          for (i=1;i<=5;i++)
                mavMsg->covariance[i] = 0.0f;
          mavMsg->covariance[6u] = coVar->cov_pose;
          for (i=7;i<=10;i++)
                mavMsg->covariance[i] = 0.0f;                   
          mavMsg->covariance[11u] = coVar->cov_pose;
          for (i=12;i<=14;i++)
                mavMsg->covariance[i] = 0.0f; 
          mavMsg->covariance[15u] = coVar->cov_twist;        
          for (i=16;i<=17;i++)
                mavMsg->covariance[i] = 0.0f;        
          mavMsg->covariance[18u] = coVar->cov_twist;
          mavMsg->covariance[19u] = 0.0f;
          mavMsg->covariance[20u] = coVar->cov_twist;
#endif
          sendViconPosEstimateMavlink( target_sys, target_comp, mavMsg ); 
  }
}

/*-----------------------------------------------------------------------------
 *      sendVisPosSpeedEstimateMavlink():  wrap and send vision position speed estimate message
 *                                   from the optical flow device
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, mavlink_vision_speed_estimate_t *visDeltaData
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendVisPosSpeedEstimateMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_vision_speed_estimate_t *visDeltaData )
{
  
#ifdef MAV_LIB_GENERATED

  uint8_t buf[MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN];                     /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  
  if (visDeltaData == NULL)                                                     /* if null was passed return */
    return;

  len = mavlink_msg_vision_speed_estimate_encode(target_sys, target_comp, &message, visDeltaData);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN );          // Send message you write to your serial UART_Write
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN );             // Send message you write to your UDP Port
#endif
}
/*-----------------------------------------------------------------------------
 *      mavlink_package_speed_estimate():  set a speed estimate update via mavlink protocol
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, opt_flow_coVar_t *coVar, T265Pose_t *t265, mavlink_vision_speed_estimate_t *mavMsg
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void mavlink_package_speed_estimate( uint8_t target_sys, uint8_t target_comp, opt_flow_coVar_t *coVar, T265Pose_t *t265, mavlink_vision_speed_estimate_t *mavMsg )
{
#if (MAV_VER == 2u)
   coVar->cov_pose = coVar->linear_accel_cov * pow(10.0f, 3.0f - coVar->tracker_confidence);
   memset((void*)&mavMsg->covariance,0.0f,sizeof(mavMsg->covariance));
   mavMsg->covariance[0u] = coVar->cov_pose;
   mavMsg->covariance[4u] = coVar->cov_pose; 
   mavMsg->covariance[8u] = coVar->cov_pose;
#endif 
   mavMsg->x = t265->velocity.x;
   mavMsg->y = t265->velocity.y;
   mavMsg->z = t265->velocity.z;
   sendVisPosSpeedEstimateMavlink( target_sys, target_comp, mavMsg );          
}
/*-----------------------------------------------------------------------------
 *      correct4CamOrientation():  correct camera orientation
 *
 *  Parameters: T265Orientation_t *ori, const T265Pose_t *t265
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void correct4CamOrientation( T265Orientation_t *ori, const T265Pose_t *t265 )
{
  ori->V_aeroRef_aeroBody = quat2mat44(mkquat(1.0f,0.0f,0.0f,0.0f));            /*  V_aeroRef_aeroBody = tf.quaternion_matrix([1,0,0,0]) */
  ori->V_aeroRef_aeroBody.m[0u][3u] = t265->velocity.x;
  ori->V_aeroRef_aeroBody.m[1u][3u] = t265->velocity.y;
  ori->V_aeroRef_aeroBody.m[2u][3u] = t265->velocity.z;   
  ori->V_aeroRef_aeroBody = mat44scl(ori->H_aeroRef_T265Ref, mat44dot(ori->V_aeroRef_aeroBody, ori->V_aeroRef_aeroBody)); /* V_aeroRef_aeroBody = H_aeroRef_T265Ref.dot(V_aeroRef_aeroBody) */
} 
/*-----------------------------------------------------------------------------
 *      sendReqDataStreamMavlink():  set a request data stream frequency request to mavlink server
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, mavlink_request_data_stream_t *visDeltaData, uint8_t cmd
 *              uint8_t rate_hz
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendReqDataStreamMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_request_data_stream_t *visDeltaData, uint8_t cmd, uint8_t rate_hz )
{
  
#ifdef MAV_LIB_GENERATED
  uint8_t buf[MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN];                     /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  
  if (visDeltaData == NULL)                                                     /* if null was passed return */
    return;

  visDeltaData->req_stream_id = MAV_DATA_STREAM_EXTRA1;
  visDeltaData->start_stop = cmd;
  visDeltaData->req_message_rate = rate_hz;
  len = mavlink_msg_request_data_stream_encode(target_sys, target_comp, &message, visDeltaData);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN );          // Send message you write to your serial UART_Write
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN );             // Send message you write to your UDP Port
#endif
}
/*-----------------------------------------------------------------------------
 *      sendOptFloAltitudeMavlink():  semd the optical flow altitude to the mavlink server
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, mavlink_altitudes_t *visDeltaData, 
 *              uint8_t altit
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendOptFloAltitudeMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_altitudes_t *visAltData, uint8_t altit )
{
  
#ifdef MAV_LIB_GENERATED
  uint8_t buf[MAVLINK_MSG_ID_ALTITUDES_LEN];                     /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  
  if (visAltData == NULL)                                        /* if null was passed return */
    return;

  visAltData->alt_optical_flow = altit;
  len = mavlink_msg_altitudes_encode(target_sys, target_comp, &message, visAltData);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_ALTITUDES_LEN);          // Send message you write to your serial UART_Write
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_ALTITUDES_LEN);             // Send message you write to your UDP Port
#endif
}
/*-----------------------------------------------------------------------------
 *      sendTextUpdateMavlink():  set a text message with status to mavlink server
 *      # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
 *      # Defined here: https://mavlink.io/en/messages/common.html#MAV_SEVERITY
 *      # MAV_SEVERITY = 3 will let the message be displayed on Mission Planner HUD, but 6 is ok for QGroundControl
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, mavlink_statustext_t *visDeltaData, uint8_t sev
 *              const opt_flow_coVar_t trakConf
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
#define GCS_IS_MISSION_PLAN                                                     /* choose you're GCS here and use this token as severity flag */
#if defined(GCS_IS_QGROUNDCONTROL)
#define OPT_FLOW_TXT_SEV 6u
#elif defined(GCS_IS_MISSION_PLAN)
#define OPT_FLOW_TXT_SEV 3u
#else
#define OPT_FLOW_TXT_SEV 1u
#endif 
void sendTextUpdateMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_statustext_t *visDeltaData, uint8_t sev, const opt_flow_coVar_t trakConf )
{
  
#ifdef MAV_LIB_GENERATED
  uint8_t buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];                                   /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  
  if (visDeltaData == NULL)                                                     /* if null was passed return */
    return;
  
  sprintf(visDeltaData->text,"Tracking confidence: %s\n",trakConf.tracker_confidence);   /* send tracking confidence to gcs */
  
  visDeltaData->severity = sev;
  len = mavlink_msg_statustext_encode(target_sys, target_comp, &message, visDeltaData);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_STATUSTEXT_LEN );                     /* Send message you write to your serial UART_Write  */
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_STATUSTEXT_LEN );                        /* Send message you write to your UDP Port */
#endif
}

/*-----------------------------------------------------------------------------
 *      sendErrorWrongVerMavlink():  send alarm for mavlink version mismatch
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, uint8_t sev, 
 *              uint8_t servMav, uint8_t clientMav
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendErrorWrongVerMavlink( uint8_t target_sys, uint8_t target_comp, uint8_t sev, uint8_t servMav, uint8_t clientMav )
{
  
#ifdef MAV_LIB_GENERATED
  uint8_t buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];                                   /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  mavlink_statustext_t textAlarm;
  
  sprintf(&textAlarm.text,"Mismatch MAVLINK version server is : %d client is %d\n",servMav,clientMav);   /* send error message to gcs */
  
  textAlarm.severity = sev;
  len = mavlink_msg_statustext_encode(target_sys, target_comp, &message, &textAlarm);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_STATUSTEXT_LEN );                     /* Send message you write to your serial UART_Write  */
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_STATUSTEXT_LEN );                        /* Send message you write to your UDP Port */
#endif
}

/*-----------------------------------------------------------------------------
 *      rcvMavData():  parse received mavlink message
 *
 *  Parameters: optFloVisRcvData_t *rcv
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
#define MAV_STRM_UPDT_HZ 20u                                                    /* stream update frequency */
#define MAV_GCS_UPDT_HZ 1u                                                      /* GCS connection status update text message frequency */
#define MAV_GCS_SYS_ID 255u                                                     /* GCS sys id */
#define MAV_HEART_TMOT 5.0f                                                     /* mavlink heartbeat timeout */
void rcvMavData( optFloVisRcvData_t *rcv )
{
   float64_t dt;
   mavlink_message_t msg;
   mavlink_set_gps_global_origin_t origin;                                      // send SET_GPS_GLOBAL_ORIGIN with fake data
   mavlink_request_data_stream_t visDeltaData; 
   mavlink_status_t status; 

   if (MavLinkBuf.State != UAMAV_PACKET_IN_BUFFER) return;                      /* interrupt not ready with a packet */
   receiveMavlinkPacketReader( &MavLinkBuf.Buffer, MAVLINK_MSG_ID_HEARTBEAT_LEN, &msg, &status ); /* parse the header up to the message type into a mavlink message structure msg */
   
   switch(msg.msgid)                                                            /* check the message code you received and respond accordingly if neccessary */
   {
        case MAVLINK_MSG_ID_HEARTBEAT:
        if (msg.sysid == MAV_GCS_SYS_ID)                                        // skip message from GCS 
           break;
        rcv->hb_count=++rcv->hb_count % UINT8_MAX;

        if (!rcv->request_sent)                                                 /* if we have not synced with the server yet */
        {
           mavlink_msg_heartbeat_decode(&msg, &rcv->heart);
#if (MAV_VER == 1)
           if (rcv->heart.mavlink_version == 1u)
           {
              sendReqDataStreamMavlink( msg.sysid, msg.compid, &visDeltaData, 1, MAV_STRM_UPDT_HZ );  /* can also use this from the mavlink interrupt if speed needed : g_SysId MavLinkBuf.Buffer[MAV1_GET_COMPONENT]; */ 
              rcv->request_sent = true;
              rcv->tick_Ref = CP0_GET(CP0_COUNT);
           }                                                                    /* update mavlink GUI with alarm to say incompatible versions */ 
           else
           {
              sendErrorWrongVerMavlink( msg.sysid, msg.compid, OPT_FLOW_TXT_SEV, rcv->heart.mavlink_version, 1u );
           }
#elif (MAV_VER == 2)
           if (rcv->heart.mavlink_version == 2u)
           {
              sendReqDataStreamMavlink( msg.sysid, msg.compid, &visDeltaData, 1, MAV_STRM_UPDT_HZ );  /* can also use this from the mavlink interrupt if speed needed : g_SysId MavLinkBuf.Buffer[MAV1_GET_COMPONENT]; */ 
              rcv->request_sent = true;
              rcv->tick_Ref = CP0_GET(CP0_COUNT);
           }
           else
           {
              sendErrorWrongVerMavlink( msg.sysid, msg.compid, OPT_FLOW_TXT_SEV, rcv->heart.mavlink_version, 2u );
           }           
#endif
        }
        else
        {
           if (!rcv->origin_sent && rcv->hb_count > 8u)                         /* wait a bit for target VO initialization. Is 8 beats enough? Feb.2018 SET_..._ORIGIN might not effect without it. */
           {
                origin.latitude = 37.2343f * 1E7f;
                origin.longitude = -115.8067f * 1E7f;
                origin.altitude = 61.0f * 1E3f;
                origin.target_system = msg.sysid;
#if (MAV_VER == 2)
                origin.time_usec = getMAVUptime();
#endif      
                sendGPSOriginMavlink( msg.sysid, msg.compid, &origin );
                rcv->origin_sent = true;
                rcv->tick_Ref = CP0_GET(CP0_COUNT);
             }
             else
             {
                calculateTickDiff(&rcv->getTickCount, &rcv->tick_Ref);          /* time since last heart beat */
                dt = TICKS2SEC(rcv->getTickCount);
                if (dt > MAV_HEART_TMOT)                                        /* heart beat timeout then re-connect */
                {
                   rcv->request_sent = false;
                   rcv->origin_sent = false; 
                   rcv->hb_count = 0u;               
                }
             }
         }
         break;
          
         case MAVLINK_MSG_ID_ATTITUDE:
         receiveMavlinkPacketReader( &MavLinkBuf.Buffer, MAVLINK_MSG_ID_ATTITUDE_LEN, &msg, &status );  /* double parsing is slow but we now parse the interrupt recieved packet for the full length of the attitude response message */
         mavlink_msg_attitude_decode(&msg, &rcv->attitude);
         break;
         
         case MAVLINK_MSG_ID_STATUSTEXT:
         break;
         
         default:
         break;
   }
   
   MavLinkBuf.State = UAMAV_BUFFER_EMPTY;                                       /* indicate we have processed the incoming interrupt */
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* --- end vision position --- */