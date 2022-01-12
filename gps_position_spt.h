#ifndef __gps_position_spt
#define __gps_position_spt

#include <stdint.h>
//#ifdef MAV_LIB_GENERATED                                                        // defout until mavlink lib is generated on pauls pc
//#include "mavlink.h"                                                            // generate using mavgenerate for the version of mavlink youre flight controller is using
//#endif
#include "definitions.h"
#include "rion.h"                                                               // we are using Raw Internet Object notation for timespec
#include "Struts.h"                                                             // for GPS type

#ifdef __cplusplus
extern "C"
{
#endif

// gps_posiiton_spt.h : from a given gps position detmnine the required pid setpoint
//
//  Contains code by (C) Randy MacKay for GPS correction
//  Written and Ported by (C) 2020 A C P Avaiation Walkerburn Scotland
//
#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define GPSPOSSPTPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define GPSPOSSPTPACKED __attribute__((packed))                                 /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define GPSPOSSPTPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define GPSPOSSPTPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define GPSPOSSPTPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

// mavlink parameters
#define LATITUDE_BASE (36.324187f * 1.0e7f)
#define LONGITUDE_BASE (138.639212f * 1.0e7f)

#define LOCATION_SCALING_FACTOR_INV_MM 0.08983204953368922f
#define DEG_TO_RAD (PI / 180.0f)

// manual anchor
// change these to the network ids of your anchors
#define ANCHOR_NETWORK  { 0x601CU, 0x6020U, 0x6057U, 0x605EU};                  // (0,0) // x-axis // y-axis // z-axis
#define ANCHOR_X {0L,    18600L, 0L,     18600L};                               // manual anchor x-coorindates in mm (horizontal)
#define ANCHOR_Y {0L,    0L,     10000L, 10000L};                               // anchor y-coordinates in mm (vertical)
#define ANCHOR_Z {1420L, 0L,     0L,     1450L};                                // anchor z-coordinates in mm

GPSPOSSPTPACKED(
typedef struct {
   uint16_t network_id;                                                         // network id
   float32_t x;                                                                 // anchor X offset east
   float32_t y;                                                                 // anchor Y offset north
   float32_t z;                                                                 // anchor Z offset down
}) coordinates_t;                                                               // anchor co-ordinates from pozyx

// ================== Function Declarations ====================================
float32_t longitude_scale(uint32_t lat);
void make_gps_time(uint32_t time_week_ms, uint16_t time_week, const rionUTCDateTimeField_t *now_time );
void location_offset(int32_t lat, int32_t lng, int32_t offset_north_mm, int32_t offset_east_mm);
void SendGPSMAVLinkMessage(coordinates_t position, const rionUTCDateTimeField_t *now_time);
void SendGPSMAVLinkMessage2(const GPS_Info_t *position, const rionUTCDateTimeField_t *now_time);

// ================== Function Definitions =====================================
float32_t longitude_scale(uint32_t lat)
{
    static int32_t last_lat = 0;
    static float32_t scale = 1.0f;
    if (labs(last_lat - lat) < 100000UL)
    {
        // we are within 0.01 degrees (about 1km) of the
        // previous latitude. We can avoid the cos() and return
        // the same scale factor.
        return scale;
    }
    //scale = cosf(lat * 1.0e-7f * DEG_TO_RAD);
    scale = cos(lat * 1.0e-7f * DEG_TO_RAD);
    if (scale < 0.01f) 
    {
      scale = 0.01f;
    }
    if (scale > 1.0f) 
    {
      scale = 1.0f;
    }
    last_lat = lat;
    return scale;
}

void location_offset(int32_t lat, int32_t lng, int32_t offset_north_mm, int32_t offset_east_mm)
{
    int32_t dlat = offset_north_mm * LOCATION_SCALING_FACTOR_INV_MM;
    int32_t dlng = (offset_east_mm * LOCATION_SCALING_FACTOR_INV_MM) / longitude_scale(lat);
    lat += dlat;
    lng += dlng;
}

// calculate GPS time
// based ardupilot/libraries/AP_GPS/GPS_Backend.cpp
// As we do not really have accurate RTC when offline and we use it for uptime
// here i'm using a timesync from a rion UTC object for the timestamp
void make_gps_time(uint32_t time_week_ms, uint16_t time_week, const rionUTCDateTimeField_t *now_time )
{
    uint8_t year, mon, day, hour, min, sec;
    uint16_t msec;
    uint32_t ret;
    int8_t rmon;

    year = now_time->year;
    mon  = now_time->month;
    day  = now_time->day;

    msec = now_time->millis;
    sec  = now_time->sec;
    min  = now_time->min;
    hour = now_time->hour;

    rmon = mon - 2u;
    if (0u >= rmon)
    {
        rmon += 12u;
        year -= 1u;
    }

    ret = (year/4u) - 15u + 367u*rmon/12u + day;                                // get time in seconds since unix epoch
    ret += year*365u + 10501u;
    ret = ret*24u + hour;
    ret = ret*60u + min;
    ret = ret*60u + sec;

    ret -= 272764785UL;                                                         // convert to time since GPS epoch

    time_week = ret / (7*86400UL);                                              // get GPS week and time
    time_week_ms = (ret % (7*86400UL)) * 1000U;
    time_week_ms += msec;
}

// GPS MAVLink message using position
void SendGPSMAVLinkMessage(coordinates_t position, const rionUTCDateTimeField_t *now_time)
{
// Initialize the required buffers
#ifdef MAV_LIB_GENERATED                                                        // defout until mavlink lib is generated on pauls pc
    mavlink_message_t msg;                                                      //from the mavlink library generated using mavgenerate GTK tool
    uint8_t buf[MAVLINK_MSG_ID_GPS_INPUT_LEN];
#endif
  /**
 * @brief Pack a gps_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param gps_id ID of the GPS for multiple GPS inputs
 * @param ignore_flags Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.
 * @param time_week_ms GPS time (milliseconds from start of GPS week)
 * @param time_week GPS week number
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in m (positive for up)
 * @param hdop GPS HDOP horizontal dilution of position in m
 * @param vdop GPS VDOP vertical dilution of position in m
 * @param vn GPS velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in m/s in DOWN direction in earth-fixed NED frame
 * @param speed_accuracy GPS speed accuracy in m/s
 * @param horiz_accuracy GPS horizontal accuracy in m
 * @param vert_accuracy GPS vertical accuracy in m
 * @param satellites_visible Number of satellites visible.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
#ifdef MAV_LIB_GENERATED
  uint16_t ignore_flags = GPS_INPUT_IGNORE_FLAG_VEL_HORIZ|
                          GPS_INPUT_IGNORE_FLAG_VEL_VERT|
                          GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY|
                          GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY|
                          GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
#endif
  uint32_t time_week_ms = 0u;
  uint16_t time_week = 0u;
  uint16_t len=0u;
  int32_t latitude = LATITUDE_BASE;                                             // adjust position
  int32_t longitude = LONGITUDE_BASE;

  make_gps_time(time_week_ms, time_week, now_time);

  location_offset(latitude, longitude, position.y, position.x);

#ifdef MAV_LIB_GENERATED                                                        // defout until mavlink lib is generated on pauls pc
  len = mavlink_msg_gps_input_pack(
                    1,
                    0,
                    &msg,
                    now_time->micros,                                           // time_usec,
                    0,                                                          // gps_id,
                    ignore_flags,
                    time_week_ms,                                               // time_week_ms,
                    time_week,                                                  // time_week,
                    3,                                                          // fix_type,
                    latitude,                                                   // latitude,
                    longitude,                                                  // longitude,
                    10,                                                         // altitude,
                    1.0f,                                                       // hdop,
                    1.0f,                                                       // vdop,
                    0.0f,                                                       // vn
                    0.0f,                                                       // ve
                    0.0f,                                                       // vd
                    0.0f,                                                       // speed_accuracy
                    0.0f,                                                       // horiz_accuracy
                    0.0f,                                                       // vert_accuracy,
                    14u                                                         // satellites_visible
                    );

   // Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_GPS_INPUT_LEN];
   len = mavlink_msg_to_send_buffer(buf, &msg);
#endif

  // Send message you write to your serial UART_Write or over UDP as you choose
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);

}
void SendGPSMAVLinkMessage2(const GPS_Info_t *position, const rionUTCDateTimeField_t *now_time)
{
  // Initialize the required buffers
#ifdef MAV_LIB_GENERATED                                                        // defout until mavlink lib is generated on pauls pc
    mavlink_message_t msg;                                                      //from the mavlink library generated using mavgenerate GTK tool
    uint8_t buf[MAVLINK_MSG_ID_GPS_INPUT_LEN];
#endif
  /**
 * @brief Pack a gps_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param gps_id ID of the GPS for multiple GPS inputs
 * @param ignore_flags Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.
 * @param time_week_ms GPS time (milliseconds from start of GPS week)
 * @param time_week GPS week number
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in m (positive for up)
 * @param hdop GPS HDOP horizontal dilution of position in m
 * @param vdop GPS VDOP vertical dilution of position in m
 * @param vn GPS velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in m/s in DOWN direction in earth-fixed NED frame
 * @param speed_accuracy GPS speed accuracy in m/s
 * @param horiz_accuracy GPS horizontal accuracy in m
 * @param vert_accuracy GPS vertical accuracy in m
 * @param satellites_visible Number of satellites visible.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
#ifdef MAV_LIB_GENERATED
  uint16_t ignore_flags = GPS_INPUT_IGNORE_FLAG_VEL_HORIZ|
                          GPS_INPUT_IGNORE_FLAG_VEL_VERT;
//                          GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY|
//                          GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY|
//                          GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
#endif
  uint32_t time_week_ms = 0u;
  uint16_t time_week = 0u;
  uint16_t len=0u;
  int32_t latitude = LATITUDE_BASE;                                             // adjust position
  int32_t longitude = LONGITUDE_BASE;
  
  make_gps_time(time_week_ms, time_week, now_time);

  location_offset(latitude, longitude, position->longd , position->lat);

#ifdef MAV_LIB_GENERATED                                                        // defout until mavlink lib is generated on pauls pc
  len = mavlink_msg_gps_input_pack(
                    1u,
                    0u,
                    &msg,
                    now_time->micros,                                           // time_usec,
                    0u,                                                         // gps_id,
                    ignore_flags,
                    time_week_ms,                                               // time_week_ms,
                    time_week,                                                  // time_week,
                    3u,                                                          // fix_type, consider converting positonMode string from the GPS data
                    latitude,                                                   // latitude,
                    longitude,                                                  // longitude,
                    position->altRef,                                           // altitude,
                    position->hdop,                                             // hdop,
                    position->vdop,                                             // vdop,
                    0.0f,                                                       // vn
                    0.0f,                                                       // ve
                    0.0f,                                                       // vd
                    0.0f,                                                       // speed_accuracy
                    position->hAcc,                                             // horiz_accuracy
                    position->vAcc,                                             // vert_accuracy,
                    position->numOfSatel                                        // satellites_visible
                    );

  // Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_GPS_INPUT_LEN];
   len = mavlink_msg_to_send_buffer(buf, &msg);
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);

}

#ifdef __cplusplus
}
#endif

#endif