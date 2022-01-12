#ifndef __cmd_long_mavlink__
#define __cmd_long_mavlink__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MAVLINK_ONBOARD_ON 0.7f
#define MAVLINK_ONBOARD_OFF 0.3f
#include "Struts.h"                                                             /* common structure interfacing with global structure for video recording time */
#include "ComGS.h"
#include "io.h"
#include "mavlink_packet_parser.h"                                              /* general function for parsing the packet of data recieved */

/* ================== Function Declarations ==================================*/
void get_location_offset_meters( GPS_Position_t *newLoc, const GPS_Position_t *orgLoc, float32_t dNorth, float32_t dEast, float32_t dAlt );
void SendOnboardMAVLinkMessage( float32_t OnBoardstate, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId);
void SendLongCommandMAVLinkMessage( float32_t States[6u], uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId, MAV_CMD command);
void SendDoParachuteMAVLinkMessage( PARACHUTE_ACTION Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId);
void SendDoLoiterMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId, MAV_CMD loiterBehave );
void SendDoNavWaypointMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId);
void SendDoTakeOffMAVLinkMessage( const mavCmdMsgPayload_t *Action, mav_craft_typ_e takeOffTyp, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId);
void SendDoNavLandMAVLinkMessage( const mavCmdMsgPayload_t *Action, mav_craft_typ_e takeOffTyp, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId);
void doSimpleMission1( mavCmdMsgPayload_t Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId, const GPS_Position_t *gpsLoc );
void SendDoChangeAltitiudeMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId);
void SendDoDelayMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId, mav_delay_e delayBehave );
void SendDoOrbitMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId);
void SendDoFollowMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId);
void SendGoTargetMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId);
void SendRePosMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId);
void SendMsgIntervalMAVLinkMessage( uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId);

#if (CAMERA_TYPE == XS_CAM)
void getXStimeVideoRecording( videoChannelStatus_t *vidChans );
#endif
#if ((((CAMERA_TYPE == XS_CAM) && defined(YI_CAM_USED)) && defined(RUN_CAM_USED)) && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, RC_Hmi_Request_t *RCButtons1, COMGS_YiDig_t *YiValues, pulse_obj_t *pulsObj  );
#elif ((((CAMERA_TYPE == XS_CAM) && defined(YI_CAM_USED)) && defined(RUN_CAM_USED)) && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, RC_Hmi_Request_t *RCButtons1, COMGS_YiDig_t *YiValues  );
#elif (((CAMERA_TYPE == XS_CAM) && defined(RUN_CAM_USED)) && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, RC_Hmi_Request_t *RCButtons1, pulse_obj_t *pulsObj );
#elif (((CAMERA_TYPE == XS_CAM) && defined(RUN_CAM_USED)) && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, RC_Hmi_Request_t *RCButtons1 );
#elif (((CAMERA_TYPE == XS_CAM) && defined(YI_CAM_USED))  && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, COMGS_YiDig_t *YiValues, pulse_obj_t *pulsObj  );
#elif (((CAMERA_TYPE == XS_CAM) && defined(YI_CAM_USED))  && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, COMGS_YiDig_t *YiValues  );
#elif ((CAMERA_TYPE == XS_CAM) && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, pulse_obj_t *pulsObj );
#elif ((CAMERA_TYPE == XS_CAM) && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1 );
#elif (defined(RUN_CAM_USED) && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, RC_Hmi_Request_t *RCButtons1, pulse_obj_t *pulsObj );
#elif (defined(RUN_CAM_USED) && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, RC_Hmi_Request_t *RCButtons1 );
#elif (defined(YI_CAM_USED) && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_YiDig_t *YiValues, pulse_obj_t *pulsObj   );
#elif (defined(YI_CAM_USED) && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_YiDig_t *YiValues  );
#endif
/* ================== Function Definitions ===================================*/

/*-----------------------------------------------------------------------------
 *      get_location_offset_meters():  write the location offset to the mavlink object
 *
 *  Parameters: GPS_Position_t *newLoc, const GPS_Position_t *orgLoc,
 *              float32_t dNorth, float32_t dEast, float32_t dAlt
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void get_location_offset_meters( GPS_Position_t *newLoc, const GPS_Position_t *orgLoc, float32_t dNorth, float32_t dEast, float32_t dAlt )
{
   float64_t dLat = (((float64_t)dNorth) / EARTH_RADIUS);                       /* co-ordinate offsets in radians */
   float64_t dLon = (dEast / (EARTH_RADIUS*cos((PI*((float64_t)orgLoc->latitude))/180.0f)));

   newLoc->latitude = ((float32_t)(orgLoc->latitude + ((dLat * 180.0f)/PI)));   /* new position in decimal degrees */
   newLoc->longditude = ((float32_t)(orgLoc->longditude + ((dLon * 180.0f)/PI)));
   newLoc->altitude = (orgLoc->altitude + dAlt);
}

/*-----------------------------------------------------------------------------
 *      SendOnboardMAVLinkMessage():  send mavlink message to enable/disable 
 *                                    onboard automatic or manual control
 *
 *  Parameters: float32_t OnBoardstate, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendOnboardMAVLinkMessage( float32_t OnBoardstate, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId)
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system    = target_sys;
  com.target_component = target_comp;
  com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
  com.confirmation     = conf;
  com.param1           = OnBoardstate;                                          // flag >0.5 => start, <0.5 => stop

  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif

}

/*-----------------------------------------------------------------------------
 *      SendMsgIntervalMAVLinkMessage():  send mavlink message to enable/disable
 *                                    onboard automatic or manual control
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId
 *
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendMsgIntervalMAVLinkMessage( uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId)
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system    = target_sys;
  com.target_component = target_comp;
  com.command          = MAV_CMD_SET_MESSAGE_INTERVAL;
  com.confirmation     = 0;
  com.param1           = 30.0f;                                                 // mavlink_attitude_t
  com.param2           = 100000;                                                // 10 Hz
  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif

}
/*-----------------------------------------------------------------------------
 *      SendDoParachuteMAVLinkMessage():  send mavlink message to enable parachute
 *
 *  Parameters: PARACHUTE_ACTION Action, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendDoParachuteMAVLinkMessage( PARACHUTE_ACTION Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId)
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system    = target_sys;
  com.target_component = target_comp;
  com.command          = MAV_CMD_DO_PARACHUTE;
  com.confirmation     = conf;
  com.param1           = Action;                                                // as per mavlink_common header for enum type e.g PARACHUTE_ENABLE and PARACHUTE_RELEASE

  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif

}

/*-----------------------------------------------------------------------------
 *      SendDoTakeOffMAVLinkMessage():  send mavlink message to take off
 *
 *  Parameters: const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendDoTakeOffMAVLinkMessage( const mavCmdMsgPayload_t *Action, mav_craft_typ_e takeOffTyp, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId)
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system = target_sys;
  com.target_component = target_comp;
  switch(takeOffTyp)
  {
     case MV_CRAFT_NORMAL:
     com.command = MAV_CMD_NAV_TAKEOFF;                                         // MAV_CMD_NAV_TAKEOFF Takeoff from ground / hand
     com.param1 = Action->payload.takeoff->min_pitch;                           // Minimum pitch (if airspeed sensor present), desired pitch without sensor
     break;
     
     case MV_CRAFT_VTOL:
     com.command = MAV_CMD_NAV_VTOL_TAKEOFF;
     com.param2 = Action->payload.takeoff->frontTransHdg;                       // Front transition heading.
     break;
     
     default:
     com.command = MAV_CMD_NAV_TAKEOFF;                                         // MAV_CMD_NAV_TAKEOFF Takeoff from ground / hand
     com.param1 = Action->payload.takeoff->min_pitch;                           // Minimum pitch (if airspeed sensor present), desired pitch without sensor
     break;
  }
  com.confirmation = conf;
/*  com.param2 =  Action->empty1;                                                               empty
  com.param3 = Action->empty2;                                                                  empty        */
  com.param4 = Action->payload.takeoff->yaw_angle;                              // Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged
  com.param5 = Action->payload.takeoff->pos->latitude;                          // Latitude
  com.param6 = Action->payload.takeoff->pos->longditude;                        // Longitude
  com.param7 = Action->payload.takeoff->pos->altitude;                          // Altitude
  
  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif
}

/*-----------------------------------------------------------------------------
 *      SendDoNavWaypointMAVLinkMessage():  send mavlink message to navigate waypoint
 *
 *  Parameters: const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendDoNavWaypointMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId)
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system = target_sys;
  com.target_component = target_comp;
  com.command = MAV_CMD_NAV_WAYPOINT;                                           // Navigate to waypoint.
  com.confirmation = conf;
  com.param1 = Action->payload.waypoint->hold_time;                             // Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
  com.param2 = Action->payload.waypoint->acc_rad;                               //Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)
  com.param3 = Action->payload.waypoint->traj_cont;                             // 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
  com.param4 = Action->payload.waypoint->yaw_angle;                             // Desired yaw angle at waypoint (rotary wing). NaN for unchanged.
  com.param5 = Action->payload.waypoint->pos->latitude;                         // Latitude
  com.param6 = Action->payload.waypoint->pos->longditude;                       // Longitude
  com.param7 = Action->payload.waypoint->pos->altitude;                         // Altitude

  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif
}

/*-----------------------------------------------------------------------------
 *      SendDoChangeAltitiudeMAVLinkMessage():  send mavlink message to chnage altitude
 *
 *  Parameters: const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendDoChangeAltitiudeMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId)
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system = target_sys;
  com.target_component = target_comp;
  com.command = MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;                            // change altitude command.
  com.confirmation = conf;
  com.param1 = Action->payload.altcont->mode;                                   // Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.
  com.param7 = Action->payload.altcont->altitude;                               // Altitude

  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif
}

/*-----------------------------------------------------------------------------
 *      SendDoOrbitMAVLinkMessage():  send mavlink message to orbit the craft round a circle
 *
 *  Parameters: const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendDoOrbitMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId)
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system = target_sys;
  com.target_component = target_comp;
  com.command = MAV_CMD_DO_ORBIT;                                               // orbit according to the parameters set
  com.confirmation = conf;
  com.param1 = Action->payload.orbit->orbRadius;                                // Radius of the circle.
  com.param2 = Action->payload.orbit->tangVel;                                  // Tangential Velocity
  com.param3 = Action->payload.orbit->orbYawBehave;                             // Yaw behavior of the vehicle as per enumerated type mav_orb_yaw_e
  com.param4 = Action->payload.orbit->dyCentBeacon;                             // for dynamic center beacon options).
  com.param5 = Action->payload.orbit->pos->latitude;                            // Latitude
  com.param6 = Action->payload.orbit->pos->longditude;                          // Longitude
  com.param7 = Action->payload.orbit->pos->altitude;                            // Altitude

  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif
}

/*-----------------------------------------------------------------------------
 *      SendRePosMAVLinkMessage():  send mavlink message to reposition after follow
 *
 *  Parameters: const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendRePosMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId)
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system = target_sys;
  com.target_component = target_comp;
  com.command = MAV_CMD_DO_FOLLOW_REPOSITION;                                   // orbit according to the parameters set
  com.confirmation = conf;
  com.param1 = Action->payload.reposn->camQ1;
  com.param2 = Action->payload.reposn->camQ2;
  com.param3 = Action->payload.reposn->camQ3;
  com.param4 = Action->payload.reposn->camQ4;
  com.param5 = Action->payload.reposn->altOff;
  com.param6 = Action->payload.reposn->xOff;
  com.param7 = Action->payload.reposn->yOff;
  
  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif
}

/*-----------------------------------------------------------------------------
 *      SendGoTargetMAVLinkMessage():  send mavlink message to get the craft to follow target
 *
 *  Parameters: const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendGoTargetMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId)
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system = target_sys;
  com.target_component = target_comp;
  com.command = MAV_CMD_DO_FOLLOW;                                              // follow target according to the parameters set
  com.confirmation = conf;
  com.param1 = Action->payload.target->SysID;                                   // System ID (of the FOLLOW_TARGET beacon). Send 0 to disable
  com.param4 = Action->payload.target->alt_mode;                                // mode of type mav_alt_targ_e
  if (com.param4 == TAR_FIXED_ABOVE)
    com.param5 = Action->payload.target->alt_abv_hm;                            // altitude above home
  else
    com.param5 = 0u;                                                            // not using altitude above home set to zero
  com.param7 = Action->payload.target->time2land;                               // Time to land in which the MAV should go to the default position hold mode after a message RX timeout

  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);   /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif
}

/*-----------------------------------------------------------------------------
 *      SendDoFollowMAVLinkMessage():  send mavlink message to get the craft to follow object
 *
 *  Parameters: const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendDoFollowMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId)
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system = target_sys;
  com.target_component = target_comp;
  com.command = MAV_CMD_NAV_FOLLOW;                                             // follow according to the parameters set
  com.confirmation = conf;
  com.param1 = Action->payload.follow->logic;                                   // depends on specific autopilot implementation
  com.param2 = Action->payload.follow->groundSpd;                               // Ground speed of vehicle to be followed
  com.param3 = Action->payload.follow->radius;                                  // Radius around waypoint. If positive loiter clockwise, else counter-clockwise
  com.param4 = Action->payload.follow->yaw_angle;                               // Desired yaw angle
  com.param5 = Action->payload.follow->pos->latitude;                           // Latitude
  com.param6 = Action->payload.follow->pos->longditude;                         // Longitude
  com.param7 = Action->payload.follow->pos->altitude;                           // Altitude

  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif
}

/*-----------------------------------------------------------------------------
 *      SendDoNavLandMAVLinkMessage():  send mavlink message to land craft
 *
 *  Parameters: const mavCmdMsgPayload_t *Action, mav_craft_typ_e takeOffTyp, uint8_t target_sys,
 *              uint8_t target_comp, uint8_t conf, uint8_t componentId
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendDoNavLandMAVLinkMessage( const mavCmdMsgPayload_t *Action, mav_craft_typ_e takeOffTyp, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId)
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system = target_sys;
  com.target_component = target_comp;
  switch(takeOffTyp)
  {
     case MV_CRAFT_NORMAL:                                                      // craft normal and lands at location specified
     com.command = MAV_CMD_NAV_LAND;                                            // Land at location.
     com.param1 = Action->payload.landing->target_alt;                          // Minimum target altitude if landing is aborted (0 = undefined/use system default).
     com.param2 = Action->payload.landing->pres_land;                           // Precision land mode
     com.param4 = Action->payload.landing->yaw_angle;                           // Desired yaw angle. NaN for unchanged.
     com.param5 = Action->payload.landing->pos->latitude;                       // Latitude
     com.param6 = Action->payload.landing->pos->longditude;                     // Longitude
     com.param7 = Action->payload.landing->pos->altitude;                       // Altitude
     break;
     
     case MV_CRAFT_VTOL:                                                        // craft uses vtol and lands at location specified
     com.command = MAV_CMD_NAV_VTOL_LAND;                                       // Land at location VTOL mode
     com.param3 = Action->payload.landing->vtol_app_alt;                        // Approach altitude (with the same reference as the Altitude field). NaN if unspecified
     com.param4 = Action->payload.landing->yaw_angle;                           // Desired yaw angle. NaN for unchanged.
     com.param5 = Action->payload.landing->pos->latitude;                       // Latitude
     com.param6 = Action->payload.landing->pos->longditude;                     // Longitude
     com.param7 = Action->payload.landing->pos->altitude;                       // Altitude
     break;

     case MV_RETURN_BASE:                                                       // craft shall return back to base (start location)
     com.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;                                // return to launch position command
     break;

     case MV_DROP_PAYLOAD:                                                      // craft to drop a payload at location specified
     com.command = MAV_CMD_NAV_PAYLOAD_PLACE;                                   /* Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload. */
     com.param1 = Action->payload.landing->target_alt;                          // Minimum target altitude if landing is aborted (0 = undefined/use system default).
     com.param5 = Action->payload.landing->pos->latitude;                       // Latitude
     com.param6 = Action->payload.landing->pos->longditude;                     // Longitude
     com.param7 = Action->payload.landing->pos->altitude;                       // Altitude
     break;
     
     default:                                                                   // unspecified craft determines normal default behaviour
     com.command = MAV_CMD_NAV_LAND;                                            // Land at location.
     com.param1 = Action->payload.landing->target_alt;                          // Minimum target altitude if landing is aborted (0 = undefined/use system default).
     com.param2 = Action->payload.landing->pres_land;                           // Precision land mode
     break;
  }

  com.confirmation = conf;


  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif
}

/*-----------------------------------------------------------------------------
 *      SendDoLoiterMAVLinkMessage():  send mavlink message to land craft
 *
 *  Parameters: const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId  MAV_CMD loiterBehave
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendDoLoiterMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId, MAV_CMD loiterBehave )
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system = target_sys;
  com.target_component = target_comp;
  com.confirmation = conf;
  switch (loiterBehave)
  {
     case MAV_CMD_NAV_LOITER_UNLIM:
     com.command = MAV_CMD_NAV_LOITER_UNLIM;                                    // loiter around this waypoint an unlimited amount of time
     com.param1 = Action->payload.loiter->type->empty1;                         // empty
     break;
     
     case MAV_CMD_NAV_LOITER_TURNS:
     com.command = MAV_CMD_NAV_LOITER_TURNS;                                    // loiter around this waypoint a specified amount of turns
     com.param1 = Action->payload.loiter->type->num_of_turns;                   // number of turns
     break;
     
     case MAV_CMD_NAV_LOITER_TIME:
     com.command = MAV_CMD_NAV_LOITER_TIME;                                     // loiter around this waypoint a specified ammount of time
     com.param1 = Action->payload.loiter->type->time_to_loiter;                 // time to loiter
     break;

     case MAV_CMD_NAV_LOITER_TO_ALT:
     com.command = MAV_CMD_NAV_LOITER_TO_ALT;                                   // Begin loiter at the specified Latitude and Longitude.
     break;
     
     default:
     com.command = MAV_CMD_NAV_LOITER_TURNS;                                    // loiter around this waypoint a specified amount of turns
     com.param1 = Action->payload.loiter->type->num_of_turns;                   // number of turns
     break;
  }
  com.param2 = Action->payload.loiter->empty2;                                  // empty
  com.param3 = Action->payload.loiter->radius;                                  // Radius around waypoint. If positive loiter clockwise, else counter-clockwise
  com.param4 = Action->payload.loiter->yaw_angle;                               // Desired yaw angle. NaN for unchanged.
  com.param5 = Action->payload.loiter->pos->latitude;                           // Latitude
  com.param6 = Action->payload.loiter->pos->longditude;                         // Longitude
  com.param7 = Action->payload.loiter->pos->altitude;                           // Altitude

  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif
}
/*-----------------------------------------------------------------------------
 *      SendDoDelayMAVLinkMessage():  send mavlink message delay action of the
 *                                    land craft
 *
 *  Parameters: const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId, mav_delay_e delayBehave
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendDoDelayMAVLinkMessage( const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId, mav_delay_e delayBehave )
{

 /*
 * @brief Pack a on board off / on message
 *
 * @param OnBoardState float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system = target_sys;
  com.target_component = target_comp;
  com.confirmation = conf;
  switch (delayBehave)
  {
     case DLY_MV_STATE_MACH:
     com.command = MAV_CMD_CONDITION_DELAY;                                     // Delay mission state machine
     com.param1 = Action->payload.delay->time;                                  // time to delay by (seconds)
     break;

     case DLY_MV_CMD_SECS:
     com.command = MAV_CMD_NAV_DELAY;                                           // Delay the next navigation command a number of seconds or until a specified time
     com.param1 = Action->payload.delay->time;                                  // time to delay by (seconds)
     break;

     case DLY_MV_CMD_HRS:
     com.command = MAV_CMD_NAV_DELAY;                                           // loiter around this waypoint a specified ammount of time
     com.param1 = -1;                                                           // check hours
     com.param2 = Action->payload.delay->hrs;                                   // set the hours
     com.param3 = 0u;                                                           // set the minutes
     com.param4 = 0u;                                                           // set the seconds
     break;

     case DLY_MV_CMD_MINS:
     com.command = MAV_CMD_NAV_DELAY;                                           // loiter around this waypoint a specified ammount of time
     com.param1 = -1;                                                           // check minutes
     com.param2 = 0u;
     com.param3 = Action->payload.delay->mins;                                  // set the minutes
     com.param4 = 0u;
     break;

     case DLY_MV_CMD_TIMESPEC:
     com.command = MAV_CMD_NAV_DELAY;                                           // loiter around this waypoint a specified ammount of time
     com.param1 = -1;                                                           // check entire time sepcified
     com.param2 = Action->payload.delay->hrs;
     com.param3 = Action->payload.delay->mins;                                  // set the minutes
     com.param4 = Action->payload.delay->secs;
     break;
     
     default:
     com.command = MAV_CMD_CONDITION_DELAY;                                     // Delay mission state machine
     com.param1 = Action->payload.delay->time;                                  // time to delay by (seconds)
     break;
  }

  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif
}

/*-----------------------------------------------------------------------------
 *      doSimpleMission1():  perform a simple mission for the craft as designated
 *
 *  Parameters: const mavCmdMsgPayload_t *Action, uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId, const GPS_Position_t *gpsLoc
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void doSimpleMission1( mavCmdMsgPayload_t Action, uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId, const GPS_Position_t *gpsLoc )
{
   SendOnboardMAVLinkMessage( 0.6f, target_sys, target_comp, conf, componentId);  /* place into guided enable mode */
   
   get_location_offset_meters( &Action.payload.takeoff.pos, gpsLoc, 0.0f, 0.0f, 10.0f ); /* take off to 10 meters high */
   SendDoTakeOffMAVLinkMessage( &Action, MV_CRAFT_NORMAL, target_sys, target_comp, conf, componentId);

   get_location_offset_meters( &Action.payload.waypoint.pos, &Action.payload.takeoff.pos, 10.0f, 0.0f, 0.0f );   /* move 10 meters north */
   SendDoNavWaypointMAVLinkMessage( &Action, target_sys, target_comp, conf, componentId);

   get_location_offset_meters( &Action.payload.waypoint.pos, &Action.payload.waypoint.pos, 0.0f, 10.0f, 0.0f );   /* move 10 meters east */
   SendDoNavWaypointMAVLinkMessage( &Action, target_sys, target_comp, conf, componentId);

   get_location_offset_meters( &Action.payload.waypoint.pos, &Action.payload.waypoint.pos, -10.0f, 0.0f, 0.0f );  /* move 10 meters south */
   SendDoNavWaypointMAVLinkMessage( &Action, target_sys, target_comp, conf, componentId);

   get_location_offset_meters( &Action.payload.waypoint.pos, &Action.payload.waypoint.pos, 0.0f, -10.0f, 0.0f );  /* move 10 meters west */
   SendDoNavWaypointMAVLinkMessage( &Action, target_sys, target_comp, conf, componentId);

   Action.payload.loiter.type.time_to_loiter=500u;                              /* loiter for X */
   SendDoLoiterMAVLinkMessage( &Action, target_sys, target_comp, conf, componentId, MAV_CMD_NAV_LOITER_TIME );

   get_location_offset_meters( &Action.payload.landing.pos, gpsLoc, 0.0f, 0.0f, 10.0f );  /* land */
   SendDoNavLandMAVLinkMessage( &Action, MV_CRAFT_NORMAL, target_sys, target_comp, conf, componentId);
}

/*-----------------------------------------------------------------------------
 *      SendLongCommandMAVLinkMessage():  send long mavlink message
 *
 *  Parameters: float32_t States[6u], uint8_t target_sys, uint8_t target_comp,
 *              uint8_t conf, uint8_t componentId, MAV_CMD command
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendLongCommandMAVLinkMessage( float32_t States[6u], uint8_t target_sys, uint8_t target_comp, uint8_t conf, uint8_t componentId, MAV_CMD command)
{

  /**
 * @brief Pack a on board off / on message
 *
 * @param states[0->6] float32_t <0.5f off >0.5f on
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @param conf uint8_t confirmation
 * @param componentId uint8_t component id
 * @param cmdId uint16_t command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
   uint16_t len=0u;

#ifdef MAV_LIB_GENERATED
  mavlink_command_long_t com = { 0u };                                          // Command Type
  mavlink_message_t message;                                                    // encoder type
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  com.target_system    = target_sys;
  com.target_component = target_comp;
  com.command          = command;
  com.confirmation     = conf;
  com.param1           = States[0u];
  com.param2           = States[1u];
  com.param3           = States[2u];
  com.param4           = States[3u];
  com.param5           = States[4u];
  com.param6           = States[5u];
  com.param7           = States[6u];

  /* encode */
  len = mavlink_msg_command_long_encode(target_sys, componentId, &message, &com);

  /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
  len = mavlink_msg_to_send_buffer(buf, &message);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN );
#endif
}

#if (CAMERA_TYPE == XS_CAM)
/*-----------------------------------------------------------------------------
 *      getXStimeVideoRecording():  gets video recording time on mavlink
 *
 *  Parameters: videoChannelStatus_t *vidChans
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void getXStimeVideoRecording( videoChannelStatus_t *vidChans )
{
    if (!strcmp(g_ch1Status,"RECORDING"))                                       /* video channel No.1 is recording */
    {
        if ((vidChans->start1)==0u)                                             /* we just started recording on this channel */
        {
           vidChans->start1 = 1u;                                               /* set the start to count the gloabal timer interrupt */
#if !defined(XS_CUMULATE_REC_TIM)                                               /* we dont want to reset each time we start and stop then control via HMI */
           vidChans->reset1 = 1u;                                               /* on first transition to begin recording then reset the last counted time */
#endif
        }
    }
    else
    {
       vidChans->start1 = 0u;                                                   /* stop the timer (we keep the value until it begins recording again when its reset */
    }

    if (!strcmp(g_ch2Status,"RECORDING"))                                       /* video channel No.2 is recording */
    {
        if ((vidChans->start2)==0u)                                             /* we just started recording on this channel */
        {
           vidChans->start2 = 1u;                                               /* set the start to count the gloabal timer interrupt */
#if !defined(XS_CUMULATE_REC_TIM)                                               /* we dont want to reset each time we start and stop then control via HMI */
           vidChans->reset2 = 1u;                                               /* on first transition to begin recording then reset the last counted time */
#endif
        }
    }
    else
    {
       vidChans->start2 = 0u;                                                   /* stop the timer (we keep the value until it begins recording again when its reset */
    }

    if (!strcmp(g_ch3Status,"RECORDING"))                                       /* video channel No.3 is recording */
    {
        if ((vidChans->start3)==0u)                                             /* we just started recording on this channel */
        {
           vidChans->start3 = 1u;                                               /* set the start to count the gloabal timer interrupt */
#if !defined(XS_CUMULATE_REC_TIM)                                               /* we dont want to reset each time we start and stop then control via HMI */
           vidChans->reset3 = 1u;                                               /* on first transition to begin recording then reset the last counted time */
#endif
        }
    }
    else
    {
       vidChans->start3 = 0u;                                                   /* stop the timer (we keep the value until it begins recording again when its reset */
    }

    if (!strcmp(g_ch4Status,"RECORDING"))                                       /* video channel No.4 is recording */
    {
        if ((vidChans->start4)==0u)                                             /* we just started recording on this channel */
        {
           vidChans->start4 = 1u;                                               /* set the start to count the gloabal timer interrupt */
#if !defined(XS_CUMULATE_REC_TIM)                                               /* we dont want to reset each time we start and stop then control via HMI */
           vidChans->reset4 = 1u;                                               /* on first transition to begin recording then reset the last counted time */
#endif
        }
    }
    else
    {
       vidChans->start4 = 0u;                                                   /* stop the timer (we keep the value until it begins recording again when its reset */
    }
    
}
#endif

/*-----------------------------------------------------------------------------
 *      checkMavlinkMsgRecievedCommandLong():  checks relevant mavlink messages
 *
 *  Parameters: deepnds on objects devices we have control of as below
 *
 *  Return:     uint16_t equivalent to the command request
 *----------------------------------------------------------------------------*/
#if ((((CAMERA_TYPE == XS_CAM) && defined(YI_CAM_USED)) && defined(RUN_CAM_USED)) && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, RC_Hmi_Request_t *RCButtons1, COMGS_YiDig_t *YiValues, pulse_obj_t *pulsObj  )
#elif ((((CAMERA_TYPE == XS_CAM) && defined(YI_CAM_USED)) && defined(RUN_CAM_USED)) && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, RC_Hmi_Request_t *RCButtons1, COMGS_YiDig_t *YiValues  )
#elif (((CAMERA_TYPE == XS_CAM) && defined(RUN_CAM_USED)) && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, RC_Hmi_Request_t *RCButtons1, pulse_obj_t *pulsObj )
#elif (((CAMERA_TYPE == XS_CAM) && defined(RUN_CAM_USED)) && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, RC_Hmi_Request_t *RCButtons1 )
#elif (((CAMERA_TYPE == XS_CAM) && defined(YI_CAM_USED))  && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, COMGS_YiDig_t *YiValues, pulse_obj_t *pulsObj  )
#elif (((CAMERA_TYPE == XS_CAM) && defined(YI_CAM_USED))  && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, COMGS_YiDig_t *YiValues  )
#elif ((CAMERA_TYPE == XS_CAM) && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1, pulse_obj_t *pulsObj )
#elif ((CAMERA_TYPE == XS_CAM) && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_Request_t *XSButtons1 )
#elif ((defined(RUN_CAM_USED) && defined(USE_MAV_PULSE_OBJECT)) && defined(YI_CAM_USED))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, RC_Hmi_Request_t *RCButtons1, pulse_obj_t *pulsObj, COMGS_YiDig_t *YiValues  )
#elif (defined(RUN_CAM_USED) && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, RC_Hmi_Request_t *RCButtons1, pulse_obj_t *pulsObj )
#elif (defined(RUN_CAM_USED) && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, RC_Hmi_Request_t *RCButtons1 )
#elif (defined(YI_CAM_USED) && defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_YiDig_t *YiValues, pulse_obj_t *pulsObj   )
#elif (defined(YI_CAM_USED) && !defined(USE_MAV_PULSE_OBJECT))
uint16_t checkMavlinkMsgRecievedCommandLong( const mavlink_message_t* msg, uint8_t targetIsUs, COMGS_YiDig_t *YiValues  )
#endif
{
   mavlink_command_long_t decodedLong;
   uint8_t reqChan;
   uint8_t streamStat=0u;                                                       /* camera commands */
   uint8_t imageStat=0u;
   uint32_t recording_time_ms=0u;

#if !(defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY))
   uint8_t relayNumber;                                                         /* relay control */
   uint8_t relayState;

#if defined(USE_MAV_PULSE_OBJECT)
   float32_t relayNoOfCycles;
   float32_t relayCycleTime;
#endif       /* end pulse object */
#endif       /* end not joystick or Gimbal HMI */

   float32_t servoNumber;                                                       /* servo command control */
   float32_t servoPwm;
   float32_t servoCycleCnt;
   float32_t servoCycleTim;

   float32_t altitudeSetPoint;                                                  /* altitude control */
   float32_t altitudeFrame;

   float32_t directionMode;                                                     /* direction control */

   float32_t paraChuteAction;                                                   /* parachute action */

   float32_t SEq_1;                                                             /* quartenion to gimbal */
   float32_t SEq_2;
   float32_t SEq_3;
   float32_t SEq_4;

#if (MAV_VER == 2u)
   uint8_t buf[MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN];
   uint16_t ret;
#endif

   decodedLong.target_system = mavlink_msg_command_long_get_target_component( msg );  /* get the target */

   if (decodedLong.target_system == targetIsUs )                                /* check that the target is us */
   {
       decodedLong.command = mavlink_msg_command_long_get_command( msg);        /* get the command and branch accordingly */
       
       switch(decodedLong.command)                                              /* check the command for the long message */
       {
          case MAV_CMD_VIDEO_STOP_CAPTURE :                                     /* request to stop video capture to file on disk */
#if defined(RUN_CAM_USED)
          reqChan=mavlink_msg_command_long_get_target_component(msg);           /* get video channel 0 is all */
          switch(reqChan)
          {
             case 0u :                                                          /* all channels */
             RCButtons1->stop=true;                                             /* set the stop for all channels latch cleared when event queued to camera */
             break;
             
             default:                                                           /* channel unknown Run Cam only has one */
             RCButtons1->stop=true;
             break;
          }
#endif
          return MAV_CMD_VIDEO_STOP_CAPTURE;
          break;
       
          case MAV_CMD_VIDEO_START_CAPTURE :                                    /* request to start video capture to file on disk */
#if defined(RUN_CAM_USED)
          reqChan=mavlink_msg_command_long_get_target_component(msg);           /* get video channel 0 is all */
          switch(reqChan)
          {
             case 0u :                                                          /* all channels */
             RCButtons1->record=true;                                           /* set the stop for all channels latch cleared when event queued to camera */
             break;

             default:                                                           /* channel unknown Run Cam only has one */
             RCButtons1->record=true;
             break;
          }
#endif
          return MAV_CMD_VIDEO_START_CAPTURE;
          break;

          case MAV_CMD_IMAGE_STOP_CAPTURE :                                     /* request to stop image capture to file on disk */
          return MAV_CMD_IMAGE_STOP_CAPTURE;
          break;

          case MAV_CMD_IMAGE_START_CAPTURE :                                    /* request to start image capture to file on disk */
          return MAV_CMD_IMAGE_STOP_CAPTURE;
          break;
       
          case MAV_CMD_VIDEO_STOP_STREAMING :                                   /* request to stop video streaming */
#if (CAMERA_TYPE == XS_CAM)
          reqChan=mavlink_msg_command_long_get_target_component(msg);           /* get video channel 0 is all */
          switch(reqChan)
          {
             case 0u :                                                          /* all channels */
             XSButtons1->hmiStopRequest=true;                                   /* set the stop for all channels latch cleared when event queued to camera */
             break;

             case 1u :                                                          /* channel No.1 */
             XSButtons1->hmiStopRequestCh1=true;                                /* set the stop for all channels latch cleared when event queued to camera */
             break;

             case 2u :                                                          /* channel No.2 */
             XSButtons1->hmiStopRequestCh2=true;                                /* set the stop for all channels latch cleared when event queued to camera */
             break;

             case 3u :                                                          /* channel No.3 */
             XSButtons1->hmiStopRequestCh3=true;                                /* set the stop for all channels latch cleared when event queued to camera */
             break;

             case 4u :                                                          /* channel No.4 */
             XSButtons1->hmiStopRequestCh4=true;                                /* set the stop for all channels latch cleared when event queued to camera */
             break;

             default:                                                           /* channel unknown */
             break;
          }
#endif
          return MAV_CMD_VIDEO_STOP_STREAMING;
          break;

          case MAV_CMD_VIDEO_START_STREAMING :                                  /* request to start video streaming */
#if (CAMERA_TYPE == XS_CAM)
          reqChan=mavlink_msg_command_long_get_target_component(msg);           /* get video channel 0 is all */
          switch(reqChan)
          {
             case 0u :                                                          /* all channels */
             XSButtons1->hmiRecordRequest=true;                                 /* set the start for all channels latch cleared when event queued to camera */
             break;

             case 1u :                                                          /* channel No.1 */
             XSButtons1->hmiRecordRequestCh1=true;                              /* set the start for all channels latch cleared when event queued to camera */
             break;

             case 2u :                                                          /* channel No.2 */
             XSButtons1->hmiRecordRequestCh2=true;                              /* set the start for all channels latch cleared when event queued to camera */
             break;

             case 3u :                                                          /* channel No.3 */
             XSButtons1->hmiRecordRequestCh3=true;                              /* set the start for all channels latch cleared when event queued to camera */
             break;

             case 4u :                                                          /* channel No.4 */
             XSButtons1->hmiRecordRequestCh4=true;                              /* set the start for all channels latch cleared when event queued to camera */
             break;

             default:                                                           /* channel unknown */
             break;
          }
#endif
          return MAV_CMD_VIDEO_START_STREAMING;
          break;

          case MAV_CMD_SET_CAMERA_MODE :                                        /* request to set the camera mode */
#if defined(RUN_CAM_USED)
          decodedLong.param1 = mavlink_msg_command_long_get_param1(msg);        /* get request mode */
          RCButtons1->mode = ((uint8_t) decodedLong.param1) % 3u;               /* set the mode to the parameter received (convert from float and range 0..2) */
#endif
          return MAV_CMD_SET_CAMERA_MODE;
          break;

          case MAV_CMD_SET_CAMERA_ZOOM :                                        /* request to set the camera zoom */
          decodedLong.param1 = mavlink_msg_command_long_get_param1(msg);
          return MAV_CMD_SET_CAMERA_ZOOM;
          break;

          case MAV_CMD_SET_CAMERA_FOCUS :                                       /* request to set the camera focus */
          decodedLong.param1 = mavlink_msg_command_long_get_param1(msg);
          return MAV_CMD_SET_CAMERA_FOCUS;
          break;

          case MAV_CMD_DO_DIGICAM_CONFIGURE :                                   /* request to configure the digicam */
          mavlink_msg_command_long_decode( msg,&decodedLong );
          return MAV_CMD_DO_DIGICAM_CONFIGURE;
          break;

          case MAV_CMD_DO_DIGICAM_CONTROL :                                     /* request to control the digicam */
          mavlink_msg_command_long_decode( msg,&decodedLong );
          return MAV_CMD_DO_DIGICAM_CONTROL;
          break;

          case MAV_CMD_DO_CONTROL_VIDEO :                                       /* request to control the video */
          mavlink_msg_command_long_decode( msg,&decodedLong );
          return MAV_CMD_DO_CONTROL_VIDEO;
          break;

          case MAV_CMD_DO_FLIGHTTERMINATION :                                   /* request to terminate flight */
          return MAV_CMD_DO_FLIGHTTERMINATION;
          break;

          case MAV_CMD_REQUEST_CAMERA_SETTINGS :                                /* request the camera information */
          return MAV_CMD_REQUEST_CAMERA_SETTINGS;
          break;

          case MAV_CMD_REQUEST_STORAGE_INFORMATION :                            /* request the storage information of the disk */
#if (MAV_VER == 2u)                                                             /* only supported in version 2 mavlink */
#if ((CAMERA_TYPE == XS_CAM) && defined(CAMERA_TYPE))
          reqChan=mavlink_msg_command_long_get_target_component(msg);           /* get video channel 0 is all */
          switch (reqChan)                                                      /* look at the channel which was returned */
          {
             case 0u:                                                           /* all channels */
             if ((!strcmp(g_sysStatus,"RECORDING")) && (!strcmp(g_diskStatus,"RECORDING")))
             {
                streamStat=1u;
             }
             break;
             
             case 1u:                                                           /* channel 1 */
             if (!strcmp(g_ch1Status,"RECORDING"))
             {
                streamStat=1u;
                recording_time_ms=((uint32_t)(g_XSVideoChan.videoRecCh1*100.0f));  /* use global recording time in seconds */
             }
             break;
             
             case 2u:                                                           /* channel 2 */
             if (!strcmp(g_ch1Status,"RECORDING"))
             {
                streamStat=1u;
                recording_time_ms=((uint32_t)(g_XSVideoChan.videoRecCh2*100.0f));  /* use global recording time in seconds */
             }
             break;

             case 3u:                                                           /* channel 3 */
             if (!strcmp(g_ch1Status,"RECORDING"))
             {
                streamStat=1u;
                recording_time_ms=((uint32_t)(g_XSVideoChan.videoRecCh3*100.0f));  /* use global recording time in seconds */
             }
             break;

             case 4u:                                                           /* channel 4 */
             if (!strcmp(g_ch1Status,"RECORDING"))
             {
                streamStat=1u;
                recording_time_ms=((uint32_t)(g_XSVideoChan.videoRecCh4*100.0f));  /* use global recording time in seconds */
             }
             break;
             
             default:                                                           /* not a known channel */
             break;
          }
#endif /* ==== XS Encoder ==== */
#if defined(YI_CAM_USED)
          switch (YiValues->VidOrPho)                                           /* take the image status from the YiCam */
          {
              case 0u:                                                           /* recording */
              imageStat=CAMERA_MODE_VIDEO;
              break;
              
              case 1u:                                                          /* picture mode */
              imageStat=CAMERA_MODE_IMAGE;
              break;
              
              default:
              break;
          }
#endif /* ===== Yi Cam ====== */
          //imageStat=CAMERA_MODE_VIDEO; or CAMERA_MODE_IMAGE;
#if ((CAMERA_TYPE == XS_CAM) && defined(CAMERA_TYPE))
          ret= mavlink_msg_camera_capture_status_pack(targetIsUs, 0u, msg, (uint32_t) getMAVUptime(), imageStat, streamStat, 0u, recording_time_ms, (float32_t) g_remSizeXS);
#elif defined(YI_CAM_USED)
          ret= mavlink_msg_camera_capture_status_pack(targetIsUs, 0u, msg, (uint32_t) getMAVUptime(), imageStat, streamStat, 0u, recording_time_ms, 0.0f);
#endif
          ret = mavlink_msg_to_send_buffer(buf, &msg);                          /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN]; */
          // now send the buf either via serial or UDP packet

#endif /* ===== mavlink2 ===== */
          return MAV_CMD_REQUEST_STORAGE_INFORMATION;
          break;

          case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS :                          /* request the camera capture status  */
          return MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
          break;

          case MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION :                       /* request the video stream status and config */
          return MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
          break;

          case MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL :                              /* set the triger interval to constantly take pictures every time trigger expires */
          break;

#if !(defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY))
          case MAV_CMD_DO_SET_RELAY:                                            /* set a relay on/off */
          mavlink_msg_command_long_decode( msg,&decodedLong );
          relayNumber=((uint8_t)decodedLong.param1);                            /* relay number */
          relayState=((uint8_t)decodedLong.param2);                             /* 1=on, 0=off otherwise can be bitmap of multistates to group */
          switch (relayState)
          {
              case 0u:                                                          /* clear the relay */
              MAV_CLR_RELAY(relayNumber);
              break;
              
              case 1u:
              MAV_SET_RELAY(relayNumber);                                       /* set the relay */
              break;

              case 2u:                                                          /* drive custom defined sequence of 16 digital relays from remote state engine */
              MAV_UNIT_OP1_SEQ_STEP(relayNumber);                               /* here relay number is sequence state number (steps) for the unit operation number 1 */
              break;

              case 3u:                                                          /* drive custom defined sequence of 16 digital relays from remote state engine */
              MAV_UNIT_OP2_SEQ_STEP(relayNumber);                               /* here relay number is sequence state number (steps) for the unit operation number 2 */
              break;
              
              default:                                                          /* set the bank */
              MAV_SET_BANK(relayNumber);                                        /* here relayNumber is bankNumber, 1 is PORTC & 2 PORTD and 3 and 4 are mask off rathen than turn on for the banks respectively */
              break;
          }
          break;


#if defined(USE_MAV_PULSE_OBJECT)
          case MAV_CMD_DO_REPEAT_RELAY:                                         /* set a pulse train on a relay output */
          mavlink_msg_command_long_decode( msg,&decodedLong );
          relayNumber=decodedLong.param1;                                       /* relay number */
          relayNoOfCycles=decodedLong.param2;                                   /* number of cycles */
          relayCycleTime=decodedLong.param3;                                    /* cycle time */
          pulsObj->cycTim = ((int64_t)relayCycleTime);                          /* now write the data to the pulse sending object which is called in the main code relayPulseAction() */
          pulsObj->cycls = ((uint8_t)relayNoOfCycles);                          /* set requested number of cycles */
          pulsObj->relayNum = ((uint8_t)relayNumber);                           /* set the requested relay number */
          pulsObj->TimeDiff = -1;                                               /* initialise the time function */
          pulsObj->cyclCnt = 0u;                                                /* reset the cycle counter aswe have a new loop */
          pulsObj->activate = true;                                             /* set the loop to activate */
          break;
#endif  /* end pulse object */
#endif  /* end not gimbal joystick or HMI e.g. something driving signals */

          case MAV_CMD_DO_SET_SERVO:                                            /* set servo motor */
          mavlink_msg_command_long_decode( msg,&decodedLong );
          servoNumber=decodedLong.param1;                                       /* servo number */
          servoPwm=decodedLong.param2;                                          /* servo pwm */
          break;
          
          case MAV_CMD_DO_REPEAT_SERVO:                                         /* repeat servo */
          mavlink_msg_command_long_decode( msg,&decodedLong );
          servoNumber=decodedLong.param1;                                       /* servo number */
          servoPwm=decodedLong.param2;                                          /* servo pwm */
          servoCycleCnt=decodedLong.param3;                                     /* servo cycle count */
          servoCycleTim==decodedLong.param4;                                    /* servo cycle time */
          break;

          case MAV_CMD_DO_CHANGE_ALTITUDE:                                      /* change the altitude set-point */
          mavlink_msg_command_long_decode( msg,&decodedLong );
          altitudeSetPoint=decodedLong.param1;                                  /* altitude setpoint */
          altitudeFrame=decodedLong.param2;                                     /* altitude frame */
          break;

          case MAV_CMD_DO_SET_REVERSE:                                          /* set to reverse mode */
          decodedLong.param1 = mavlink_msg_command_long_get_param1(msg);        /* get request mode */
          directionMode=decodedLong.param1;                                     /* Direction (0=Forward, 1=Reverse) */
          break;

          case MAV_CMD_DO_PARACHUTE:                                            /* command to control parachute */
          decodedLong.param1 = mavlink_msg_command_long_get_param1(msg);        /* get request mode */
          paraChuteAction=decodedLong.param1;                                   /* as per PARACHUTE_ACTION */
          break;

          case MAV_CMD_DO_MOUNT_CONTROL_QUAT:                                   /* control the camera or gimbal to a given quartenion co-ordinate */
          mavlink_msg_command_long_decode( msg,&decodedLong );
          SEq_1=decodedLong.param1;                                             /* quaternion param q1, w (1 in null-rotation) */
          SEq_2=decodedLong.param2;                                             /* quaternion param q2, x (0 in null-rotation) */
          SEq_3=decodedLong.param3;                                             /* quaternion param q3, y (0 in null-rotation) */
          SEq_4=decodedLong.param4;                                             /* quaternion param q4, z (0 in null-rotation) */
          break;
          
          default:
          return MAV_CMD_ENUM_END;                                              /* return the END status for anything we dont currently support in action */
          break;
      }
   }
   
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end custom comand library wrapper */