#ifndef __AirMap_
#define __AirMap_
/*
        airmap.h : Interface to Airmap flight authorisatin system

  Copyright (c) ACP Aviation
  contains information from Copyright(c) <2016> <Juan Gonzalez Burgos>

  All rights reserved.
*/

#include "GeoJSON.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define AIRMAPPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define AIRMAPPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define AIRMAPPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define AIRMAPPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define AIRMAPPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define AIRMAP_TELEM_UDP1_PORT 7070U
#define AIRMAP_HOST_UDP1_URL "udp.telemetry.k8s.airmap.io"
#define AIRMAP_TELEM_UDP2_PORT 16060u
#define AIRMAP_HOST_UDP2_URL "telemetry.airmap.com"

#if defined(D_FT900)
typedef struct AIRMAPPACKED {
    uint8_t valid : 1u;
    uint8_t spare1 : 7u;
    unsigned char condition[];
    unsigned char icon;
    uint32_t windHeading;
    float32_t windSpeed;
    uint32_t windGusting;
    float32_t temperature;
    float32_t humidity;
    float32_t visibility;
    float32_t precipitation;
} AirMap_WeatherInfo_t;
#else
AIRMAPPACKED (
typedef struct
{
    uint8_t valid : 1u;
    uint8_t spare1 : 7u;
    unsigned char condition[];
    unsigned char icon;
    uint32_t windHeading;
    float32_t windSpeed;
    uint32_t windGusting;
    float32_t temperature;
    float32_t humidity;
    float32_t visibility;
    float32_t precipitation;
}) AirMap_WeatherInfo_t;
#endif

/* ------- Telemetry Updates ---------------------------------------------- */
#if defined(D_FT900)
typedef struct AIRMAPPACKED {
    uint64_t timestamp;                                                         ///< Ingestion timestamp of the update.
    float64_t latitude;                                                         ///< The latitude of the position [°].
    float64_t longitude;                                                        ///< The longitude of the position in [°].
    float64_t altitude_msl;                                                     ///< The altitude above mean sea level of the position in [m].
    float64_t altitude_gl;                                                      ///< The altitude above ground level of the position in [m].
    float64_t horizontal_accuracy;                                              ///< The horizontal accuracy of the position in [m].
} AirMap_Position_t;
#else
AIRMAPPACKED (
typedef struct
{
    uint64_t timestamp;                                                         ///< Ingestion timestamp of the update.
    float64_t latitude;                                                         ///< The latitude of the position [°].
    float64_t longitude;                                                        ///< The longitude of the position in [°].
    float64_t altitude_msl;                                                     ///< The altitude above mean sea level of the position in [m].
    float64_t altitude_gl;                                                      ///< The altitude above ground level of the position in [m].
    float64_t horizontal_accuracy;                                              ///< The horizontal accuracy of the position in [m].
}) AirMap_Position_t;
#endif

#if defined(D_FT900)
typedef struct AIRMAPPACKED {
    uint64_t timestamp;                                                         ///< Ingestion timestamp of the update.
    float32_t velocity_x;                                                       ///< The velocity of the vehicle in direction of the x axis in [m/s].
    float32_t velocity_y;                                                       ///< The velocity of the vehicle in direction of the y axis in [m/s].
    float32_t velocity_z;                                                       ///< The velocity of the vehicle in direction of the z axis in [m/s].
} AirMap_Velocity_t;
#else
AIRMAPPACKED (
typedef struct
{
    uint64_t timestamp;                                                         ///< Ingestion timestamp of the update.
    float32_t velocity_x;                                                       ///< The velocity of the vehicle in direction of the x axis in [m/s].
    float32_t velocity_y;                                                       ///< The velocity of the vehicle in direction of the y axis in [m/s].
    float32_t velocity_z;                                                       ///< The velocity of the vehicle in direction of the z axis in [m/s].
}) AirMap_Velocity_t;
#endif

#if defined(D_FT900)
typedef struct AIRMAPPACKED {
    uint64_t timestamp;                                                         ///< Ingestion timestamp of the update.
    float32_t yaw;                                                              ///< The yaw of the vehicle in [°/s].
    float32_t pitch;                                                            ///< The pitch of the vehicle in [°/s].
    float32_t roll;                                                             ///< The roll of the vehicle in [°/s].
} AirMap_Attitude_t;
#else
AIRMAPPACKED (
typedef struct
{
    uint64_t timestamp;                                                         ///< Ingestion timestamp of the update.
    float32_t yaw;                                                              ///< The yaw of the vehicle in [°/s].
    float32_t pitch;                                                            ///< The pitch of the vehicle in [°/s].
    float32_t roll;                                                             ///< The roll of the vehicle in [°/s].
}) AirMap_Attitude_t;
#endif

#if defined(D_FT900)
typedef struct AIRMAPPACKED {
    uint64_t timestamp;                                                         ///< Ingestion timestamp of the update.
    float32_t pressure;                                                         ///< The atmospheric pressure measurement in [Pa].
} AirMap_Barometer_t;
#else
AIRMAPPACKED (
typedef struct
{
    uint64_t timestamp;                                                         ///< Ingestion timestamp of the update.
    float32_t pressure;                                                         ///< The atmospheric pressure measurement in [Pa].
}) AirMap_Barometer_t;
#endif

typedef enum
{
  aes256cbc = 1
} Airmap_encrypt_e;                                                             /* only supports aes atm */

typedef enum
{
  am_udp_port = 16060u,
  am_http_port = 80u,
  am_https_port = 8080u
} Airmap_mehod_port_e;                                                          /* only supports aes atm */

#define AM_TOKEN_URL "https:\/\/api.airmap.com\/auth\/v1\/anonymous\/token"     /* ----------- json method post get_token -------------- */
#define AM_JSON_API_KEY(key,A) do{  sprintf(A,"\"X-API-Key\": %u\n",key); } while(0)  /* used in all post "X-API-Key": api_key */
#define AM_JSON_USERID(usr,A) do{  sprintf(A,"\"user_id\": %u\n",usr); } while(0) /* "user_id" */
#define AM_JSON_TOKEN_GOOD 200u

/* should be returned */
#define AM_JSON_AUTH(token,A) do{  sprintf(A,"Authorization": \"Bearer \" %u\n",token); } while(0)   /* send the authorisation token */

#define AM_JSON_CREATE_PLAN_URL "https:\/\/api.airmap.com\/flight\/v2\/plan"    /* ----------- json method post create_plan -------------- */
/* first send api key AM_JSON_API_KEY as before then the authroisation token AM_JSON_AUTH */
#define AM_JSON_CONTEXT "\"Accept-Language\", \"en-US,en;q=0.5\" \"Accept: application/json\"; \"Content-Type\": \"application/json; charset=utf-8\""   /* and send the context */

/* json data    "takeoff_latitude": 33.85505651142062,
                "takeoff_longitude": -118.37099075317383,
                "pilot_id": pilotID,
                "start_time": "2020-04-23T18:38:44.730Z",
                "end_time": "2020-04-23T18:53:42.000Z",
                "max_altitude_agl": 100,
                "buffer": 1,
                "geometry":                                            
                { "type": "Polygon", "coordinates": [ [ [ -118.37099075317383, 33.85505651142062 ], [ -118.37305068969727, 33.85502978214579 ], [
-118.37347984313963, 33.854673391015496 ], [ -118.37306141853333, 33.85231226221667 ], [ -118.37193489074707, 33.85174201755203 ], [ -118.36997151374815, 33.85176874785573 ], [
-118.36995005607605, 33.8528112231754 ], [ -118.37099075317383, 33.85505651142062 ] ] ] }
            })                                                                */

#define AM_MAX_TIME_LEN 50u                                                     /* work this out !! */

#if defined(D_FT900)
typedef struct AIRMAPPACKED {
    float32_t takeoff_latitude;
    float32_t takeoff_longitude;
    uint8_t pilot_id;
    unsigned char start_time[AM_MAX_TIME_LEN];
    unsigned char end_time[AM_MAX_TIME_LEN];
    uint16_t max_altitude_agl;
    uint8_t buffer;
    geoJsonPolygon_t geometry;
} AirMap_jsonData_t;
#else
AIRMAPPACKED (
typedef struct
{
    float32_t takeoff_latitude;
    float32_t takeoff_longitude;
    uint8_t pilot_id;
    unsigned char start_time[AM_MAX_TIME_LEN];
    unsigned char end_time[AM_MAX_TIME_LEN];
    uint16_t max_altitude_agl;
    uint8_t buffer;
    geoJsonPolygon_t geometry;
}) AirMap_jsonData_t;                                                           /* need to write the json sender for this structure */
#endif

#define AM_JSON_SUBMIT_PLAN_URL(plan,A) do{ sprintf(A,"https:\/\/api.airmap.com\/flight\/v2\/plan\/%s/submit",plan); } while(0)
/* first send api key AM_JSON_API_KEY as before then the authroisation token AM_JSON_AUTH */

#define AM_JSON_END_COMM_URL(plan,A) do{ sprintf(A,"https:\/\/api.airmap.com\/flight\/v2\/plan\/%s/end-comm",plan); } while(0)
/* first send api key AM_JSON_API_KEY as before then the authroisation token AM_JSON_AUTH */

#define AM_JSON_END_FLIGHT(flightId,A)_URL do{ sprintf(A,"https:\/\/api.airmap.com\/flight\/v2\/%s/end",flightId); } while(0)
/* first send api key AM_JSON_API_KEY as before then the authroisation token AM_JSON_AUTH */

#define AM_HOSTNAME "telemetry.airmap.com"

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end AirMap */