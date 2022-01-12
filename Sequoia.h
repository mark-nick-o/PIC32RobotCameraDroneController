// Library for Parrot sequoia camera
// https://developer.parrot.com/docs/sequoia/#http-control-api
//
// Written Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#ifndef PARROT_SEQ_CAM
#define PARROT_SEQ_CAM

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define SEQPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define SEQPACKED __attribute__((packed))                                     /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define SEQPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define SEQPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define SEQPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

// http: GET functions

// http://<ip_address>/status
// JSON object is status
#define SEQ_HTTP_GET_STATUS(A,B) { sprintf(A,"http:\/\/%s/status",B); }         // starts the status request for the given IP address passes as a string in dotted format 10.1.2.3

// query parameters
#define SEQ_STAT_REQ "/request"                                                 // returns string 'get_status’
#define SEQ_STAT_SUN "/sunshine"                                                // JSON object All sunshine sensors values
#define SEQ_STAT_TEMP "/temperature"                                            // JSON object All temperatures values
#define SEQ_STAT_GPS "/gps"                                                     // JSON object All informations about gps state
#define SEQ_STST_INST "/instruments"                                            // JSON object All informations about Sequoia and sunshine sensor angles

// structures for the JSON objects
#if defined(D_FT900)
typedef struct SEQPACKED {
      uint8_t green_0;
      uint8_t red_0;
      uint8_t red_edge_0;
      uint8_t near_infrared_0;
      uint8_t green_1;
      uint8_t red_1;
      uint8_t red_edge_1;
      uint8_t near_infrared_1;
      unsigned char status[50U];
} SEQStat_Sunshine_t;                                                           // structure holding the status of the sunshines information
#else
SEQPACKED(
typedef struct {
      uint8_t green_0;
      uint8_t red_0;
      uint8_t red_edge_0;
      uint8_t near_infrared_0;
      uint8_t green_1;
      uint8_t red_1;
      uint8_t red_edge_1;
      uint8_t near_infrared_1;
      unsigned char status[50U];
}) SEQStat_Sunshine_t;                                                          // structure holding the status of the sunshines information
#endif

#if defined(D_FT900)
typedef struct SEQPACKED {
      uint8_t satellite_number;
      float32_t precision;
      float32_t speed;
      uint8_t fix;
      uint8_t body_imu;
      uint8_t sunshine_imu;
      unsigned char status[50u];
} SEQStat_Gps_t;                                                               // structure holding the status of the gps information
#else
SEQPACKED(
typedef struct {
      uint8_t satellite_number;
      float32_t precision;
      float32_t speed;
      uint8_t fix;
      uint8_t body_imu;
      uint8_t sunshine_imu;
      unsigned char status[50u];
}) SEQStat_Gps_t;                                                               // structure holding the status of the gps information
#endif

#if defined(D_FT900)
typedef struct SEQPACKED {
      float32_t body_p7;
      float32_t body_p7mu;
      float32_t body_ddr3;
      float32_t body_wifi;
      float32_t body_imu;
      float32_t sunshine_imu;
      unsigned char status[50u];
} SEQStat_Temperature_t;                                                        // structure holding the status of the temperatures information
#else
SEQPACKED(
typedef struct {
      float32_t body_p7;
      float32_t body_p7mu;
      float32_t body_ddr3;
      float32_t body_wifi;
      float32_t body_imu;
      float32_t sunshine_imu;
      unsigned char status[50u];
}) SEQStat_Temperature_t;                                                       // structure holding the status of the temperatures information
#endif

#if defined(D_FT900)
typedef struct SEQPACKED {
      float32_t body_yaw;
      float32_t body_pitch;
      float32_t body_roll;
      float32_t sunshine_yaw;
      float32_t sunshine_pitch;
      float32_t sunshine_roll;
      unsigned char status[50u];
} SEQStat_Instruments_t;                                                        // structure holding the status of the temperatures information
#else
SEQPACKED(
typedef struct {
      float32_t body_yaw;
      float32_t body_pitch;
      float32_t body_roll;
      float32_t sunshine_yaw;
      float32_t sunshine_pitch;
      float32_t sunshine_roll;
      unsigned char status[50u];
}) SEQStat_Instruments_t;                                                       // structure holding the status of the temperatures information
#endif

// http://<ip_address>/capture
//
#define SEQ_HTTP_GET_CAPTURE(A,B) { sprintf(A,"http:\/\/%s/capture",B); }       // starts the capture request for the given IP address passes as a string in dotted format 10.1.2.3

// query parameters
#define SEQ_CAP_START "/start"                                                  // Start a capture
#define SEQ_CAP_STOP "/stop"                                                   // Stop a capture

#if defined(D_FT900)
typedef struct SEQPACKED {
      unsigned char request[50u];
      unsigned char status[50u];
      unsigned char path[50u];
} SEQStat_Capture_t;                                                            // returned with the capture request
#else
SEQPACKED(
typedef struct {
      unsigned char request[50u];
      unsigned char status[50u];
      unsigned char path[50u];
}) SEQStat_Capture_t;                                                           // returned with the capture request
#endif 

// http://<ip_address>/config
//
#define SEQ_HTTP_GET_POST_CONFIG(A,B) { sprintf(A,"http:\/\/%s/config",B); }    // starts the config request for the given IP address passes as a string in dotted format 10.1.2.3

// query parameters
#define SEQ_CONF_START "/start"                                                 // Start a capture
#define SEQ_CONF_STOP "/stop"                                                  // Stop a capture
#define SEQ_CONF_REQ "/request"                                                 // returns string 'get_status’
#define SEQ_CONF_CM "/capture_mode"                                             // returns string Ready’ or 'Calibration running’ or 'Snapshot running’ or 'Timelapse running’
#define SEQ_CONF_PATH "/path"                                                   // Path where pictures are saved if a capture is running

#if defined(D_FT900)
typedef struct SEQPACKED {
      unsigned char request[50U];
      unsigned char capture_mode[50U];                                          // Capture state or error message
      float64_t timelapse_param;                                                // Time interval
      float64_t gps_param;                                                      // Distance interval
      float64_t overlap_param;                                                  // Overlap parameter
      int16_t resolution_rgb;                                                   // 12 or 16
      float64_t resolution_mono;                                                // 0.3 or 1.2
      int16_t bit_depth;                                                        // 8 or 10
      int16_t sensors_mask;                                                     // Mask on 5 bits {nir, red-edge, red, green, rgb}
      unsigned char storage_selected[15U];                                      // 'internal’ or 'sd’
      unsigned char auto_select[5U];                                            // on or off
} SEQStat_Config_t;                                                             // structure holding the configuration
#else
SEQPACKED(
typedef struct {
      unsigned char request[50U];
      unsigned char capture_mode[50U];                                          // Capture state or error message
      float64_t timelapse_param;                                                // Time interval
      float64_t gps_param;                                                      // Distance interval
      float64_t overlap_param;                                                  // Overlap parameter
      int16_t resolution_rgb;                                                   // 12 or 16
      float64_t resolution_mono;                                                // 0.3 or 1.2
      int16_t bit_depth;                                                        // 8 or 10
      int16_t sensors_mask;                                                     // Mask on 5 bits {nir, red-edge, red, green, rgb}
      unsigned char storage_selected[15U];                                      // 'internal’ or 'sd’
      unsigned char auto_select[5U];                                            // on or off
}) SEQStat_Config_t;                                                            // structure holding the configuration
#endif 

/* POST CONFIGURATION USING PAYLoad AS BELOW

   payload = { "capture_mode": "single",
            "timelapse_param": 2.5,
            "gps_param": 25.0,
            "overlap_param": 80.0,
            "resolution_rgb": 12,
            "resolution_mono": 1.2,
            "bit_depth": 10,
            "sensors_mask": 31,
            "storage_selected": "internal",
            "auto_select": "off",
          }
 
*/
#define seq_CONF_NAMES SC("capture_mode")SC("timelapse_param")SC("gps_param")SC("overlap_param")SC("resolution_rgb")SC("resolution_mono")SC("bit_depth")SC("sensors_mask")SC("resolution_mono")SC("storage_selected")SC("auto_select")
#define SC(x) #x,
const char * const seq_config_values[] = { seq_CONF_NAMES };                    // Array of string definitions for the config values

// http://<ip_address>/calibration
//
#define SEQ_HTTP_GET_CALI(A,B) { sprintf(A,"http:\/\/%s/calibration",B); }      // calibration control http get

// query parameters
#define SEQ_CALI_START "/start"                                                 // Start a calibration of both devices
#define SEQ_CALI_STOP "/stop"                                                  // Stop a calibraton of both devices
#define SEQ_CALI_START_SEQ "/body/start"                                        // Start a calibration of Sequoia
#define SEQ_CALI_STOP_SEQ "/body/stop"                                         // Stop a calibraton of Sequoia
#define SEQ_CALI_START_SUN "/sunshine/start"                                    // Start a calibration of sunshine
#define SEQ_CALI_STOP_SUN "/sunshine/stop"                                     // Stop a calibraton of sunshine

/* The calibration status should have those different states:
None: device need a calibration
Ok: No calibration is required
Running: A calibration is running
Psi pending: Psi (Yaw) rotation needed
Theta pending: Theta (Pitch) rotation needed
Phi pending: Phi (Roll) rotation needed
Failed: Calibration failed
Abort: Calibration aborted */

#if defined(D_FT900)
typedef struct SEQPACKED {
      unsigned char request[50U];
      unsigned char body[50U];                                                  // Capture state or error message
      unsigned char sunshine[5U];                                               // on or off
} SEQStat_Calib_t;                                                             // structure holding the calibration status
#else
SEQPACKED(
typedef struct {
      unsigned char request[50U];
      unsigned char body[50U];                                                  // Capture state or error message
      unsigned char sunshine[5U];                                               // on or off
}) SEQStat_Calib_t;                                                             // structure holding the calibration status
#endif 

// http://<ip_address>/storage
//
#define SEQ_HTTP_GET_STOR(A,B) { sprintf(A,"http:\/\/%s/storage",B); }          // storage control http get

#define SEQ_STOR_SD_PATH "/sd"

#define seq_STOR_NAMES ST("request")ST("storage_selected")ST("internal")ST("sd")
#define ST(x) #x,
const char * const seq_storage_values[] = { seq_STOR_NAMES };                   // Array of string definitions for the storage class

// JSON object structures internal or sd objects
#if defined(D_FT900)
typedef struct SEQPACKED {
      float64_t total;                                                          // Total memory
      float64_t free;                                                           // Free memory
      unsigned char path[50U];                                                  // Path of the root directory
      unsigned char read_only[5U];                                              // 'yes’ or 'no’
      unsigned char corrupted[5U];                                              // 'yes’ or 'no’
} SEQStat_StorageDev_t;                                                         // structure holding the configuration for either internal or SD
#else
SEQPACKED(
typedef struct {
      float64_t total;                                                          // Total memory
      float64_t free;                                                           // Free memory
      unsigned char path[50U];                                                  // Path of the root directory
      unsigned char read_only[5U];                                              // 'yes’ or 'no’
      unsigned char corrupted[5U];                                              // 'yes’ or 'no’
}) SEQStat_StorageDev_t;                                                        // structure holding the configuration for either internal or SD
#endif  

#if defined(D_FT900)
typedef struct SEQPACKED {
      unsigned char request[20U];                                               // 'get_storage’
      unsigned char storage_selected[15U];                                      // 'internal’ or 'sd’
      SEQStat_StorageDev_t internal;                                            // JSON obj for the internal storage
      SEQStat_StorageDev_t sd;                                                  // JSON obj for the sd card storage
} SEQStat_Storage_t;                                                            // structure returned with a storage http get query
#else
SEQPACKED(
typedef struct {
      unsigned char request[20U];                                               // 'get_storage’
      unsigned char storage_selected[15U];                                      // 'internal’ or 'sd’
      SEQStat_StorageDev_t internal;                                            // JSON obj for the internal storage
      SEQStat_StorageDev_t sd;                                                  // JSON obj for the sd card storage
}) SEQStat_Storage_t;                                                           // structure returned with a storage http get query
#endif

// http://<ip_address>/file
//
#define SEQ_HTTP_GET_FILE(A,B) { sprintf(A,"http:\/\/%s/file",B); }             // file control http get

// query parameters
#define SEQ_FILE_INT_NUM "/internal"                                            // List internal memory content
#define SEQ_FILE_SD_NUM "/sd"                                                   // list sd memory content
#define SEQ_FILE_FOLDER_SZ "/internal/"                                         // List 'folder name’ content

#define seq_FILE_NAMES FI("request")FI("/internal")FI("/sd")
#define FI(x) #x,
const char * const seq_file_values[] = { seq_FILE_NAMES };                      // Array of string definitions for file request

#if defined(D_FT900)
typedef struct SEQPACKED {
      unsigned char request[20U];                                               // 'get_storage’
      float64_t int_sz;
      float64_t sd_sz;
} SEQStat_File_t;                                                               // struct for the http get file request
#else
SEQPACKED(
typedef struct {
      unsigned char request[20U];                                               // 'get_storage’
      float64_t int_sz;
      float64_t sd_sz;
}) SEQStat_File_t;                                                              // struct for the http get file request
#endif

#if defined(D_FT900)
typedef struct SEQPACKED {
      unsigned char request[20U];                                               // 'get_storage’
      unsigned char string[];                                                   // 'returns the path names and number of pics in lines { "/internal/0001": 5, "/internal/0002": 5 }
} SEQStat_File_NoOfPic_t;                                                       // struct for the http get file request either for sd or internal
#else
SEQPACKED(
typedef struct {
      unsigned char request[20U];                                               // 'get_storage’
      unsigned char string[];                                                   // 'returns the path names and number of pics in lines { "/internal/0001": 5, "/internal/0002": 5 }
}) SEQStat_File_NoOfPic_t;                                                      // struct for the http get file request either for sd or internal
#endif
 
#if defined(D_FT900)
typedef struct SEQPACKED {
      unsigned char request[20U];                                               // 'get_storage’
      unsigned char string[];                                                   // 'returns the path names and number of pics in lines { "/internal/0002/IMG_160629_123655_0000_RGB.JPG": 1246414, "/internal/0002/IMG_160629_123656_0000_NIR.TIF": 2486514 }
} SEQStat_File_OctetSz_t;                                                       // struct for the http get file request for internal/
#else
SEQPACKED(
typedef struct {
      unsigned char request[20U];                                               // 'get_storage’
      unsigned char string[];                                                   // 'returns the path names and number of pics in lines { "/internal/0002/IMG_160629_123655_0000_RGB.JPG": 1246414, "/internal/0002/IMG_160629_123656_0000_NIR.TIF": 2486514 }
}) SEQStat_File_OctetSz_t;                                                      // struct for the http get file request for internal/
#endif 

// http://<ip_address>/download
//
#define SEQ_HTTP_DOWNLD(A,B) { sprintf(A,"http:\/\/%s/download",B); }           // download control http get

// either full path name "GET http://192.168.47.1/download/internal/0000/IMG_160704_133619_0000_GRE.TIF"
// or directory for zip file "GET http://192.168.47.1/download/internal/0000"

// http://<ip_address>/delete
//
#define SEQ_HTTP_DEL(A,B) { sprintf(A,"http:\/\/%s/delete",B); }                // delete control http get

// either full path name for file GET http://192.168.47.1/delete/internal/0000/IMG_160704_133619_0000_GRE.TIF"
// or for deleteing entire directory GET http://192.168.47.1/delete/internal/0000"

// http://<ip_address>/version
//
#define SEQ_HTTP_VER(A,B) { sprintf(A,"http:\/\/%s/version",B); }               // version control http get
#if defined(D_FT900)
typedef struct SEQPACKED {
      unsigned char request[20U];                                               // 'get_version’
      unsigned char version[20U];                                               // version id "1.1.0"
      unsigned char bodySerial[32U];                                            // sequoia serial number
      unsigned char sunSerial[32U];                                             // sunshine sensor serial number
} SEQStat_Version_t;                                                            // struct for the http get version command
#else
SEQPACKED(
typedef struct {
      unsigned char request[20U];                                               // 'get_version’
      unsigned char version[20U];                                               // version id "1.1.0"
      unsigned char bodySerial[32U];                                            // sequoia serial number
      unsigned char sunSerial[32U];                                             // sunshine sensor serial number
}) SEQStat_Version_t;                                                           // struct for the http get version command
#endif 

// http://<ip_address>/wifi
//
#define SEQ_HTTP_WIFI(A,B) { sprintf(A,"http:\/\/%s/wifi",B); }                 // wifi control http get
#if defined(D_FT900)
typedef struct SEQPACKED {
      unsigned char request[20U];                                               // 'get_version’
      unsigned char ssid[20U];                                                  // ssid  "Sequoia_0109"
      unsigned char status[5U];                                                 // on or off
} SEQStat_WiFi_t;                                                               // struct for the http get wifi command
#else
SEQPACKED(
typedef struct {
      unsigned char request[20U];                                               // 'get_version’
      unsigned char ssid[20U];                                                  // ssid  "Sequoia_0109"
      unsigned char status[5U];                                                 // on or off
}) SEQStat_WiFi_t;                                                              // struct for the http get wifi command
#endif 

// http://<ip_address>/manualmode
//
#define SEQ_HTTP_MAN(A,B) { sprintf(A,"http:\/\/%s/manualmode",B); }            // manual mode control http get

#define seq_MAN_NAMES MA("mode")MA("exp_us")MA("iso")
#define MA(x) #x,
const char * const seq_manmode_values[] = { seq_MAN_NAMES };                    // Array of string definitions for the manualmode values

#if defined(D_FT900)
typedef struct SEQPACKED {
      int16_t exp_us;                                                           // exposure
      int16_t iso;                                                              // Gain value x 100
} SEQStat_MAN_t;                                                                // struct for the http get manual mode
#else
SEQPACKED(
typedef struct {
      int16_t exp_us;                                                           // exposure
      int16_t iso;                                                              // Gain value x 100
}) SEQStat_MAN_t;                                                               // struct for the http get manual mode
#endif  

#if defined(D_FT900)
typedef struct SEQPACKED {
      unsigned char mode[10U];                                                  // mode
      SEQStat_MAN_t rgb;                                                        // json object for exposure and gain
} SEQStat_RGB_t;                                                                // struct for the http get manual mode rgb
#else
SEQPACKED(
typedef struct {
      unsigned char mode[10U];                                                  // mode
      SEQStat_MAN_t rgb;                                                        // json object for exposure and gain
}) SEQStat_RGB_t;                                                               // struct for the http get manual mode rgb
#endif 

#if defined(D_FT900)
typedef struct SEQPACKED {
     unsigned char mode[10U];                                                   // mode
     SEQStat_MAN_t colorGreen;                                                  // json object for exposure and gain green parameters
     SEQStat_MAN_t colorRed;                                                    // json object for exposure and gain red parameters
     SEQStat_MAN_t red_edge;                                                    // json object for exposure and gain red_edge parameters
     SEQStat_MAN_t near_infrared;                                               // json object for exposure and gain near_infrared parameters
} SEQStat_MONO_t;                                                               // struct for the http get manual mode rgb
#else
SEQPACKED(
typedef struct {
     unsigned char mode[10U];                                                   // mode
     SEQStat_MAN_t colorGreen;                                                  // json object for exposure and gain green parameters
     SEQStat_MAN_t colorRed;                                                    // json object for exposure and gain red parameters
     SEQStat_MAN_t red_edge;                                                    // json object for exposure and gain red_edge parameters
     SEQStat_MAN_t near_infrared;                                               // json object for exposure and gain near_infrared parameters
}) SEQStat_MONO_t;                                                              // struct for the http get manual mode rgb
#endif 

// http://<ip_address>/websocket    EVENT DRIVEN STATUS UPDATE
//
#define SEQ_HTTP_EVENT(A,B) { sprintf(A,"http:\/\/%s/websocket",B); }           // websocket (event notification) mode control via http get

// PING
// In order to stay connected to the websocket, you have to reply “pong” on each “ping” sent by the server.
// not sure if we can emulate this in the mikroE C for PIC32 as ICMP reply automatic

// events
// status string "change"
// Capture string (ready or running)
// sunshine string (connected or disconnected
// staystill string yes or no
// storage_select string internal or sd
// storage_status string as above
// state string Storage status event: “connected” , “disconnected”, “full”, “no_longer_full”, “error”
// calibration string body or sunshine
// state string calib events “None”, “Ok”, “Running”, “Phi pending”, “Theta pending”, “Psi pending”, “Failed”, “Aborted”
// capture_mode stringnew capture mode: “single”, “timelapse”, “gps_position”, “auto”, “radiometric”
// timelapse_param double  value of timelapse in second, for timelapse mode
// gps_param double value of gps distance in meters, for gps mode
// overlap_param int Value of overlap in percent, for auto mode
// sensors_mask int Sensors mask of activated sensors on the camera
// resolution_rgb int resolution of sensors in Mpx
// resolution_mono double resolution of sensors in Mpx
// bit_depth int bit depth of mono sensors
// fix string yes or no representing GPS fix state
#define seq_EVENT_NAMES EV("status")EV("capture")EV("sunshine")EV("staystill")EV("storage_select")EV("storage_status")EV("state")EV("calibration")EV("capture_mode")EV("timelapse_param")EV("gps_param")EV("overlap_param")EV("sensors_mask")EV("resolution_rgb")EV("resolution_mono")EV("bit_depth")EV("fix")
#define EV(x) #x,
const char * const seq_event_values[] = { seq_EVENT_NAMES };                    // Array of string definitions for event reply strings

#if defined(D_FT900)
typedef struct SEQPACKED {
     unsigned char status[10U];                                                 // "change" for event
     unsigned char capture[10U];                                                // (ready or running)
     unsigned char sunshine[15U];                                               // connected or disconnected
     unsigned char staystill[5U];                                               // yes or no
     unsigned char storage_select[10U];                                         // internal or sd
     unsigned char storage_status[10U];                                         // internal or sd
     unsigned char state[15U];                                                  // “connected” , “disconnected”, “full”, “no_longer_full”, “error”
     unsigned char calibration[10U];                                            // body or sunshine
     unsigned char capture_mode[15U];                                           //  “single”, “timelapse”, “gps_position”, “auto”, “radiometric”
     float64_t timelapse_param;
     float64_t gps_param;
     int16_t overlap_param;
     int16_t sensors_mask;
     int16_t resolution_rgb;
     float64_t resolution_mono;
     int16_t bit_depth;
     unsigned char fix[5U];                                                     // yes or no
} SEQStat_EVENT_t;                                                              // struct for the http get websocket event status replies to be stored in
#else
SEQPACKED(
typedef struct {
     unsigned char status[10U];                                                 // "change" for event
     unsigned char capture[10U];                                                // (ready or running)
     unsigned char sunshine[15U];                                               // connected or disconnected
     unsigned char staystill[5U];                                               // yes or no
     unsigned char storage_select[10U];                                         // internal or sd
     unsigned char storage_status[10U];                                         // internal or sd
     unsigned char state[15U];                                                  // “connected” , “disconnected”, “full”, “no_longer_full”, “error”
     unsigned char calibration[10U];                                            // body or sunshine
     unsigned char capture_mode[15U];                                           //  “single”, “timelapse”, “gps_position”, “auto”, “radiometric”
     float64_t timelapse_param;
     float64_t gps_param;
     int16_t overlap_param;
     int16_t sensors_mask;
     int16_t resolution_rgb;
     float64_t resolution_mono;
     int16_t bit_depth;
     unsigned char fix[5U];                                                     // yes or no
}) SEQStat_EVENT_t;                                                             // struct for the http get websocket event status replies to be stored in
#endif  

// PTP Protocol
#define SEQ_PTP_PORT 15740U                                                     // UDP port for PTP
#define SEQ_PTP_IP 192.168.47.1                                                 // Fixed IP for sequioa cameraz

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif                                                                          // include this file or not