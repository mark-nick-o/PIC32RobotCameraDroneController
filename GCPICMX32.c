//
//                   Name : GCPICMX32.c
//                   Version : @(#) 1.0
//                   Description : This is a 32PICMX7 program for a joystick sending commands over SimpleBGC protocol.
//                                 Communication is over serial or UDP datagrams with Gimbal controller
//                                 Communication to COM GS GUI is over CanBus.
//                                 Communication to AMP camera encoders using serial over UDP
//                                 Data maniulation for filters and quarternion co-ordinates for AHRS system
//
//
//#define PROCESS_SMOOTH_INPUT_BEAR                                             define these if you want non-linear processing of ADC inputs
//#define PROCESS_SMOOTH_INPUT_DIFF
//#define USE_CALIB_VALS                                                         define if you want to have non-linear calibration for the ANI (exact center)

#include <stdint.h>
//#include "stdint.h"
//#include "__NetEthInternal.h"
#include "gc_events.h"                                                          // Common to the events.c functions
#include "Struts.h"
#include "InteruptServices.h"                                                   // Common for the interupts
#include "definitions.h"                                                        // general config e.g. Node, compiler
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
#include "SBGC.h"                                                               // SBGC general
#include "SBGC_rc.h"                                                            // SBGC rc commands
#include "SBGC_COMMAND.h"                                                       // SBGC commands
#include "SBGC_PARSER.h"                                                        // SBGC parser (mostly for aduino) some general lengths etc
#include "SBGC_cmd_helpers.h"                                                   // SBGC all message conatiners
#include "SBGC_adj_vars.h"                                                      // SBGC variables
#endif
#include "ports.h"                                                              // Definition of ethernet ports used
#include "io.h"                                                                 // input output declaration                         
#include "ComGs.h"                                                              // Common library of functionality
#ifdef BOSCH_BME680_USED
#include "boschSensorBME680.h"                                                  // Temperature Humidity Pressure i2c
#endif
#ifdef ALARM_BANNER_USED
#include "AlarmDefinitions.h"                                                   // Alarm Banner
#endif
#if defined(GPS_INCLUDED) || defined(GPS_INCLUDED2)
#include "GPS.h"                                                                // GPS commands to initiate NMEA position commands
#endif
#ifdef CANON_CAM_USED
#include "canon.h"                                                              // canon camera is attached
#endif
#ifdef PELCO_PTZ_USED
#include "pelco_d_ptz.h"                                                        // security protocol for PTZ on CCTV cams
#endif
#ifdef SMPTE_USED
#include "smpte.h"                                                              // SMPTE & MIDI MTC library needed
#endif
#ifdef ART_NET_USED
#include "art_net4.h"                                                           // Art-Net is being used for DMX512
#include "art_net4_funcs.h"
#endif
#ifdef ASN_E1_USED
#include "asn_e1_31.h"                                                          // Lightweight streaming protocol for transport of DMX512 using ACN
#endif
#ifdef LW3_SWITCH_USED
#include "lwLW3.h"                                                              // LW3 protocol for video / audio switch
#include "LW3StringHelper.h"
#endif
#ifdef GEF_EGD_PLC                                                              // We have command set from a GEC Fanuc PLC
#include "EGD.h"                                                                // GEC Fanuc PLC communication
#endif
#ifdef PENTAX_CAM_USED
#include "pentax.h"                                                             // pentax camera library
#endif
#ifdef COLOR_CODEC_USED
#include "bitmap.h"
#include "colorhelper.h"                                                        // Color definitions and RGB/HSV conversion
#endif

//#include "lwNx.h"                                                               // Light Ware optoelectronics library
#if defined(RUN_CAM_USED) || (CAMERA_TYPE == XS_CAM)
#include "camera.h"                                                             // For dealing with encoder / run cam / Yi Cam
#endif

//#include "config.h"
#include "crc.h"                                                                // Cyclic redundancy library

#if defined(LEDDARTECH_LIDDAR_USED) || defined(MODBUS_NEEDED)
#include "mbproto.h"                                                            // modbus protocol library
#include "cognex_mbus.h"                                                        // cognex modbus library
#endif

#ifdef JRT_LIDDAR_USED
#include "jrt_lidar.h"                                                          // JRT liddar sensor Chengdu JRT Meter Technology Co., Ltd.
#endif

#include "pulse_oximeter.h"

#define MAX_NUM_OF_UDP_SENDS 3U                                                 // Max no of times to send UDP
#define MSG_REPOLL_TICKS 1000U                                                  // The re-poll frequency of a non confirmed message

// Set up the poll duration between the data request for each poll message e.g. for  get_angles,get_angles_ext,Realtime_data3
#define POLL_MODULUS 640U                                                       // The modulus of the polled event (higher means less frequent)
#define POLL_SLOT 4U                                                            // the duration between slots of the polled events (they stagger 1,2,3)

// Set up the poll duration for RunCam messages
#define RC_POLL_MODULUS 6400U                                                   // frequency of call
#define RC_POLL_SLOT 2U                                                         // 2 items time_remaining and space_left_on_disk

// The default parametrs for SimpleBGC serial mode
#define SERIAL_SPEED 115200UL                                                   // Serial baud rate should match with the rate, configured for the SimpleBGC controller (default)
#define CMD_CONTROL_DELAY 20U                                                   // delay between CMD_CONTROL commands, ms
#define UDP_SEND 0U
#define UDP_NO_SEND 1U

#ifdef UBIBOT_USED
#include "ubibot.h"                                                             // library for uniBot precision agri instruments
#include "jsmn.h"                                                               // Jasmine libraries for parsing JSON replies from the uBi-Bot devices
#endif

#ifdef YI_CAM_USED
#include "jsmn.h"                                                               // Jasmine libraries for parsing JSON replies from the Yi Action Cam
#include "YiAction.h"                                                           // Include macros for enumerated types in Yi Action Cam
#endif

#ifdef LWNX_LIDDAR_USED
#include "lwNx_defs.h"
#include "lwNx.h"                                                               // Liddar Library
#endif

#ifdef SEQ_CAM_USED
#include "frozen.h"                                                             // for writing and (reading? no scanf) JSON strings
#include "sequoia.h"                                                            // speaking with parrot sequoia camera
#endif

#if defined(SONY_VISCA_PROTO_USED) || defined(ZEUS_CAM_USED)
#include "visca.h"                                                              // library for sony visca Zeus thermal imaging camera
#endif

#ifdef MICROSCAN_USED
#include "microscan.h"                                                          // library for microscan ocr and image matching camera
#include "microscan_funcs.h"
#endif

//#include "msgpk_object.h"
//#include "msgpk_pack_template.h"
#ifdef LASER_DISPLAY_USED
#include "ilda.h"                                                               // library for handling laser control files or making them
#endif

#ifdef MIDI_USED
#include "midi.h"                                                               // compose midi control messages (keyboard interface)
#endif

#ifdef USE_CAN
#include "canBus_defs.h"                                                        // definition of the canBus (UAVCAN) protocol
#endif

#ifdef RION_USED
#include "rion.h"                                                               // definition of raw internet object notation RION
#endif

#if defined(USE_SPEKTRUM)                                                       // we want Spektrum bus over i2c
#include "spektrum_bus.h"                                                       // spektrum bus over i2c library
#endif

#if defined(LINEAR_FLOAT_PMBUS)                                                 // We need to convert power values to PMBUS Linear11 Linear16 and vice versa
#include "linear_float_conversions.h"                                           // convert pmbus linear11 and 16 values to IEEE float
#endif

#if defined(FLASH_MEM_USED)                                                     // we are storing data in flash memory
#include "flash_mem.h"
#endif

#if defined(TERRAIN_FOLLOW_USED)
#include "terrain_follow.h"                                                     /* enable the terrain follower */
#endif

#if defined(MPU6050_ACC_GYRO)
#include "mpu6050_acc_gyro.h"                                                   /* i2c registers for reading the accelerometer data from the MPU6050 */
#endif

#if defined(I2C_CAMERA)
#include "ovm7690_i2c_cam.h"                                                    /* omni camera is on i2c */
#endif

#ifdef USE_BISON
#include "bison.h"
#endif

#if defined(USE_MAVLINK)                                                        // we are using MAVLINK - defined in defintions
#include "mavlink_crc.h"                                                        // cyclic redundancy for MAVLINK
#include "mavlink_conversions.h"                                                // useful math conversions
#if (MAV_VER == 2u)                                                             // ==== MAVLINK ver 2.0 =============
#include "mavlink2.h"                                                           // general MAV system defines
#include "mavlink2_msg_types.h"                                                 // structures for message types
#include "mavlink2_common.h"                                                    // common enum and structs for command set
#include "mavlink2_protocol.h"                                                  // protocol driver
#include "mavlink2_sha256.h"                                                    // sha256 implementation
#include "mavlink2_get_info.h"                                                  // get info request if 24bit set
#include "mavlink2_helpers.h"                                                   // general helper functions
// ------------------ GPS ------------------------------------------------------
#include "msg2_gps_input.h"                                                     // GPS signal can be sent/read
#include "msg2_global_position_int.h"                                           // global posiiton for airmap
// ------------------ RTK/GNSS -------------------------------------------------
#include "msg2_mavlink_msg_gps_rtk.h"                                           // realtime kinematics correction
// ------------------ HEARTBEAT  -----------------------------------------------
#include "msg2_heartbeat.h"                                                     // MAVLINK watchdog always needed
// ------------------ GIMBAL  --------------------------------------------------
#include "msg2_gimbal_control.h"                                                // Gimbal control message from MAVLINK
#include "msg2_attitude_quartenion.h"                                           // quartenion and position feed back for gimbal
#include "msg2_gimbal_device_set_attitude.h"                                    // gimbal device set attitude (from qgroundcontrol)
#include "msg2_gimbal_device_attitude_status.h"                                 // gimbal device attitude status (from qgroundcontrol)
#include "msg2_gimbal_manager_set_attitude.h"                                   // gimbal multiple device set attitude (from qgroundcontrol)
#include "msg2_gimbal_device_informations.h"                                    // gimbal device extended data (from qgroundcontrol)
// ------------------ CAMERA  --------------------------------------------------
#include "msg2_digicam_control.h"                                               // Focus/Zoom control message from MAVLINK
#include "msg2_camera_feedback.h"                                               // position Status message from MAVLINK
#include "msg2_camera_capture_status.h"                                         // Space left time recording etc message to MAVLINK
#include "msg2_camera_image_captured.h"                                         // image name GPS position message to MAVLINK
#include "msg2_camera_information.h"                                            // resolution message to MAVLINK
#include "msg2_camera_settings.h"                                               // zoom focus message from MAVLINK
// ------------------ DISTANCE  ------------------------------------------------
#include "msg2_distance_sensor.h"                                               // distance sensor info to MAVLINK
// ------------------ BATTERY  -------------------------------------------------
#include "msg2_asluav_batmon.h"                                                 // battery monitor message MAVLINK
// ------------------ DATA  ----------------------------------------------------
#include "msg2_airspeeds.h"                                                     // airspeed (various sensors) message MAVLINK
#include "msg2_altitudes.h"                                                     // altitude (various sensors) message MAVLINK
#include "msg2_hires_imu.h"                                                     // IMU sensor message MAVLINK
#include "msg2_power_board.h"                                                   // Power Board message MAVLINK
// ------------------ CONTROL --------------------------------------------------
#include "msg2_com_long.h"                                                      // custom command MAVLINK
#include "msg2_follow_target.h"                                                 // follow a target message
// ------------------ OPTICAL FLOW ---------------------------------------------
#include "msg2_optical_flow.h"                                                  // optical flow
#include "msg2_optical_flow_rad.h"
// ------------------ VIDEO STREAM ---------------------------------------------
#include "msg2_video_stream_info.h"
#include "msg2_video_stream_status.h"
// ------------------ VICON POSE ESTIMATION ------------------------------------
#include "msg2_vicon_pose_est.h"                                                // vicon datastream communication via mavlink to bridge
// ------------------ VISION POSITION ------------------------------------------
#include "msg2_vision_position_delta.h"
#include "msg2_vision_position_speed_estimate.h"
#include "msg2_vision_position_estimate.h"
#include "msg2_set_gps_global_origin.h"
#include "msg2_request_data_stream.h"                                           // set data stream poll intervals (heartbeat)
#include "msg2_pose.h"
// ----------------- ATTITUDE --------------------------------------------------
#include "msg2_attitude.h"
// ----------------- STATUS TEXT  ----------------------------------------------
#include "msg2_statustext.h"                                                    // update controller status
// ----------------- STATUS UPDATE ---------------------------------------------
#include "msg2_sys_status.h"
// ----------------- SYSTEM TIME SYNC ------------------------------------------
#include "msg2_sys_time.h"
#define MAV_LIB_GENERATED                                                       // enable the MAV send command
#elif (MAV_VER == 1u)
#include "mavlink1.h"                                                           // standard defintion of system
#include "mavlink1_msg_types.h"                                                 // ==== MAVLINK ver 1.0 =============
#include "mavlink1_common.h"
#include "mavlink1_protocol.h"                                                  // protocol driver
#include "mavlink1_helpers.h"                                                   // helper function
// ------------------ GPS ------------------------------------------------------
#include "msg1_gps_input.h"                                                     // GPS signal can be sent/read
#include "msg1_global_position_int.h"                                           // global posiiton for airmap
// ------------------ RTK/GNSS -------------------------------------------------
#include "msg1_mavlink_msg_gps_rtk.h"                                           // realtime kinematics correction
// ------------------ HEARTBEAT  -----------------------------------------------
#include "msg1_heartbeat.h"                                                     // MAVLINK watchdog always needed
// ------------------ GIMBAL  --------------------------------------------------
#include "msg1_gimbal_control.h"                                                // Gimbal control message from MAVLINK
#include "msg1_attitude_quartenion.h"                                           // quartenion and position feed back for gimbal
// ------------------ CAMERA  --------------------------------------------------
#include "msg1_digicam_control.h"                                               // Focus/Zoom control message from MAVLINK
#include "msg1_camera_feedback.h"                                               // position Status message from MAVLINK
// ------------------ DISTANCE  ------------------------------------------------
#include "msg1_distance_sensor.h"                                               // distance sensor info to MAVLINK
// ------------------ BATTERY  -------------------------------------------------
#include "msg1_asluav_batmon.h"                                                 // battery monitor message MAVLINK
// ------------------ DATA  ----------------------------------------------------
#include "msg1_airspeeds.h"                                                     // airspeed (various sensors) message MAVLINK
#include "msg1_altitudes.h"                                                     // altitude (various sensors) message MAVLINK
#include "msg1_hires_imu.h"                                                     // IMU sensor message MAVLINK
#include "msg1_power_board.h"                                                   // Power Board message MAVLINK
// ------------------ CONTROL --------------------------------------------------
#include "msg1_com_long.h"                                                      // custom command MAVLINK
#include "msg1_follow_target.h"                                                 // follow a target message
// ------------------ OPTICAL FLOW ---------------------------------------------
#include "msg1_optical_flow.h"                                                  // optical flow
#include "msg1_optical_flow_rad.h"
// ------------------ VICON POSE ESTIMATION ------------------------------------
#include "msg1_vicon_pose_est.h"
// ----------------- VISION POSITION ESTIMATION --------------------------------
#include "msg1_vision_speed_est.h"
#include "msg1_vision_position_estimate.h"
#include "msg1_request_data_stream.h"                                           // set data stream poll intervals (heartbeat)
#include "msg1_set_gps_global_origin.h"
#include "msg1_pose.h"
// ----------------- ATTITUDE --------------------------------------------------
#include "msg1_attitude.h"
// ----------------- STATUS TEXT -----------------------------------------------
#include "msg1_statustext.h"                                                    // send a message used by T265 optical flow bridge
// ----------------- STATUS UPDATE -----------------------------------------------
#include "msg1_sys_status.h"
// ----------------- SYSTEM TIME SYNC -----------------------------------------------
#include "msg1_sys_time.h"
#define MAV_LIB_GENERATED                                                       // enable the MAV SEND command
#else
#error "MAVLINK version either 1 or 2 must be #define'd."
#endif
#include "gps_position_spt.h"                                                   // send GPS read over MAVLINK
#include "mavlink_send_long_cmd.h"                                              // long command e.g. onboard on/off MAVLINK
#include "mavlink_power_data.h"                                                 // read convert and package to UAVCAN from MAVLINK power data
#include "mavlink_attitude_quartenion.h"                                        // attitude quartenion message
#include "mavlink_vision_position.h"                                            // pmlps vision position update to from mavlink

#endif // ----------- end MAVLINK ----------------------------------------------

#if defined(USE_MMC_SPI2)                                                       /* use the mmc fat16 file handler */
#include "mmc_file_handler.h"                                                   /* mmc FAT16 file system on SPI2 */
#endif

#if defined(GEO_JSON_USED)
#include "geoJSON.h"                                                            /* geoJSON library */
#endif

#if defined(USE_NTP)
#include "ntp.h"                                                                /* ntp timesync structure send a set of zero and get the time reply from the server */
#endif

#if defined(GPRS_GSM_MODEM)
#include "GSM_Modem.h"                                                          /* communicate with gsm or GPRS */
#endif

#if defined(XTCALL_DISPLAY)
#include "XTCalls_Proto.h"                                                      /* drive display */
#endif
#if defined(MOVSIN_DISPLAY)
#include "MovingSignProtocolV2.h"                                               /* drive display */
#endif

//#if defined(PROTOBUF_USED)
//#include "protobuf_c.h"                                                         /* protocol buffers for google */
//#endif

#if defined(CYPRESS_2_4GHZ_USB)
//#include "cyrf6936.h"
#endif
//#include "qmp6988_baro.h"

#if defined(AIRMAP_USED)
#include "airmap.h"                                                             /* air traffic control system */
#endif

#if defined(USE_WAV_SOUND)                                                      /* include the use of wav files */
#include "wav_file.h"
#endif

#if defined(SLMP_MITSI)
#include "slmp_mitsi.h"
#endif

#if defined(USE_MORTON_3D)
#include "morton_3D_LUTs.h"
#endif

#if defined(USE_MORTON_2D)
#include "morton_2D_LUTs.h"
#endif

#if defined(YANDEX_DISK_FILE)
#include "yandex_disk.h"
#endif

//================================== DEFINITIONS ===============================
//  __attribute__((aligned(4)))                                                    Use this to align to 4 byte boundaries where needed

// ----------------------------- version information ---------------------------
unsigned char versionNo[10u] = "ver 1.0.0";

// --------------------------------- UDP ---------------------------------------
unsigned char doPacketReturn;                                                   // Return from the UDP read Buffer
uint8_t msgType;                                                                // Type (Cmd) of SBGC message received.
unsigned char udpReturn;                                                        // return from the UDP send function
uint8_t udpSent;                                                                // retries on UDP datagrams
uint16_t totalMsgLen;                                                           // The total length of the packet
unsigned char Buffer[SBGC_CMD_MAX_BYTES];                                       // The Buffer for read write of UDP set to length of bigest packet
//---------------- CAMERA GIMBAL SBGC Protocol ---------------------------------
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
unsigned char crc;                                                              // The Buffer for the CRC for simpleBGC version less than 2.68b0
uint16_t newCRC;                                                                // The Buffer for the CRC for simpleBGC version 2.68b0 or greater
uint8_t stateEngine;
uint8_t gimStartUpComplete=false;                                                  // flag to indicate we have got the revision and the gimbal is ready

SBGC_cmd_header_t head = { SBGC_CMD_START_BYTE, SBGC_CMD_CONTROL, 0U, 0U };     // the container for the first SimpleBGC header

SBGC_cmd_control_t c = { SBGC_CONTROL_MODE_SPEED, 0U, 0U, 0U, 0U, 0U, 0U };     // The control object for the message container for CMD_CONRTROL at zero state.for v<2.55b5
unsigned char *ptr_c = (unsigned char *) &c;                                    // Define a pointer to the container struct
SBGC_cmd_control_new_t cnew = { SBGC_CONTROL_MODE_SPEED, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };  // The control object for the message container for CMD_CONRTROL at zero state for v>2.55b5
unsigned char *ptr_cnew = (unsigned char *) &cnew;                                 // Define a pointer to the container struct

SBGC_cmd_control_config_t cc = { SBGC_CMD_CONTROL_CONFIG, 0U, 0U, 0U, 0U, 0U, 0U };   // The control object for the CMD_CONREOL_CONFIG which allows user to turn on/off confirm to CMD_CONTROL
unsigned char *ptr_cc = (unsigned char *) &cc;                                  // Define the pointer to the conrainer struct
uint8_t *ptr_new_cc = (uint8_t *) &cc;

SBGC_cmd_reset_t r = { 0U, 5U };                                                // The control object for the message container is no confirm 5 ms wait.
unsigned char *ptr_r = (unsigned char *) &r;                                    // Define the pointer to the conrainer struct

SBGC_cmd_api_virt_ch_control_t vc = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };        // virtual control channel
unsigned char *ptr_vc = (unsigned char *) &vc;                                   // Define the pointer to the container struct

SBGC_cmd_pid_t pida;                                                            // Container for PID auto tune request. v2.7 <
unsigned char *ptr_pida = (unsigned char *) &pida;                              // Define the pointer to the container struct
SBGC_cmd_pid2_t pida2;                                                          // Container for PID auto tune request. v2.7 >
unsigned char *ptr_pida2 = (unsigned char *) &pida2;                            // Define the pointer to the container struct

/*   moved to interrupt to avoid race hazard
SBGC_progress_auto_pid_t pidreadbk;                                             // Container for the PID readback message
unsigned char *ptr_pidreadbk = (unsigned char *) &pidreadbk;                    // Define the pointer to the container struct

SBGC_cmd_get_angles_t getangles;                                                // Conatiner for the GET_ANGLES readback message
unsigned char *ptr_getangles = (unsigned char *) &getangles;                    // Define the pointer to the container struct

SBGC_cmd_get_angles_ext_t getanglesext;                                         // Conatiner for the GET_ANGLES_EXT readback message
unsigned char *ptr_getanglesext = (unsigned char *) &getanglesext;              // Define the pointer to the container struct

SBGC_cmd_realtime_data_3 realdata3;                                             // Conatiner for the REALTIME_DATA_3 readback message
unsigned char *ptr_realdata3 = (unsigned char *) &realdata3;                    // Define the pointer to the container struct

SBGC_cmd_realtime_data_4 realdata4;                                             // Conatiner for the REALTIME_DATA_4 readback message
unsigned char *ptr_realdata4 = (unsigned char *) &realdata4;                    // Define the pointer to the container struct

SBGC_cmd_confirmation_t cmdconf;                                                // Conatiner for the CMD_CONFIRM readback message
unsigned char *ptr_cmdconf = (unsigned char *) &cmdconf;                        // Define the pointer to the container struct

SBGC_cmd_error_t cmderror;                                                      // Conatiner for the CMD_ERROR readback message
unsigned char *ptr_cmderror = (unsigned char *) &cmderror;                      // Define the pointer to the container struct   */

SBGC_cmd_execute_menu_t menu;                                                   // Container for the CMD_EXECUTE_MENU command
unsigned char *ptr_menu = (unsigned char *) &menu;                              // Define the pointer to the container struct

SBGC_cmd_set_adj_vars_val_t adjvar;                                             // Container for the CMD_SET_ADJ_VARS_VAL command  (3 values deep)
unsigned char *ptr_adjvar = (unsigned char *) &adjvar;                          // Define the pointer to the container struct

SBGC_cmd_get_adj_vars_val_t getvar;                                             // Container for the CMD_GET_ADJ_VARS_VAL command (7 values deep) 1 CanFrame
unsigned char *ptr_getvar = (unsigned char *) &getvar;                          // Define the pointer to the container struct
/* SBGC_cmd_setget_adj_vars_val_t getvar_rd;                                     Container for the CMD_SET_ADJ_VARS_VAL command in response to above (7 values deep) */

SBGC_cmd_select_imu_t imusel;                                                   // Container for the CMD_SELECT_IMU
unsigned char *ptr_imusel = (unsigned char *) &imusel;                          // Define the pointer to the container struct

SBGC_cmd_beep_sound_t musicPlay;                                                // Container for the CMD_BEEP_SOUND
unsigned char *ptr_musicPlay = (unsigned char *) &musicPlay;                    // Define the pointer to the container struct
unsigned char melodyACP1[5u] = { noteG3, noteA3, noteB3, noteD3, noteE3 };      // Play G – A – B – D – E   on initialisation to the gimbal controller.
unsigned char melodyACP2[5u] = { SBGC_MAJOR_THIRD(noteG3), noteA3, SBGC_MAJOR_SIXTH(noteB3), noteD3, SBGC_MAJOR_SEVENTH(noteE3) };
                                                                                // Play Gmaj3 – A – Bmaj6 – D – Emaj7   on completing a re-boot cycle in the gimbal controller.
unsigned char melodyACP3[5u] = { noteD1, noteE1, noteD1, noteG0, noteE0 };      // Play D – E – D – G – E   on initialisation to the gimbal controller.

/* SBGC_cmd_event_t eventCmd;                                                      Container for the CMD_EVENT which is a readback
unsigned char *ptr_eve = (unsigned char *) &eventCmd;                            Define the pointer to the container struct */
SBGC_cmd_data_stream_interval_t dataStreamInt;                                  // Container for the CMD_DATA_STREAM_INTERVAL
unsigned char *ptr_dsi = (unsigned char *) &dataStreamInt;                      // Define the pointer to the container struct

/* SBGC_cmd_board_info_t boardinforb;                                            Container for BOARD_INFO readback
unsigned char *ptr_bi = (unsigned char *) &boardinforb;                          Define the pointer to the container struct */
//uint8_t boardReady=false;                                                       // Reply to say Gimbal has completed boot up okay
// cmd board info
uint8_t boardInfo=CMD_SEND;                                                     // Status of board info request (start-up asking for information)
uint16_t boardExcTimer=0u;                                                      // Timer in ticks for retry on board not ready at right firmware
uint16_t boardExcTimer2=0u;                                                     // Timer in ticks for retry on no reply to board info request
uint8_t noOfBoardSends=0u;                                                      // Total number of alternating requests for firmware (attempts using v1 then v2 etc)

SBGC_cmd_profile_set_t profileset;                                              // Container for profile load / save object
unsigned char *ptr_ps = (unsigned char *) &profileset;                          // Define the pointer to the container struct
SBGC_cmd_profile_set_t *ptr_profileset  = ((void*)0);                           // pointer of type SBGC_cmd_profile_set_t
uint16_t sz_profileset=0u;                                                      // Size not using sizeof

/* SBGC_cmd_read_params_3_t readparam3;                                          Container for config param read/write
unsigned char *ptr_rp3 = (unsigned char *) &readparam3;                          Define the pointer to the container struct */

// EXAMPLE OF RUN SCRIPT FUNCTION
SBGC_cmd_write_file_t writescript;                                              // Container for write file function
unsigned char *ptr_wsc = (unsigned char *) &writescript;                        // Define the pointer to the container struct
SBGC_cmd_run_script_t runscript;                                                // Container for run a script function
unsigned char *ptr_rsc = (unsigned char *) &runscript;                          // Define the pointer to the container struct
/*SBGC_cmd_script_debug_t scriptstate;                                             returned script state   */

IO_btn_state_t debounceResetPB;                                                 // de-bounced Reset Pushbutton for SBGC

uint8_t selctIMUReq;                                                            // IMU Type to be sent with Select IMU request

// Read back of REAL_TIME_DATA_CUSTOM                                           // can be set-up from CMD_DATA_STREAM_INTERVAL
/* SBGC_cmd_realtime_data_custom_reply_t realtimedatacust;                      Container for real time data custom */
SBGC_ahrs_debug_info_t ahrsdebuginfo;                                           // Container for ahrs data for external imu got from command above

// Extra data for VC Control
uint8_t channels;                                                               // counter for number of VC channels
uint8_t firstTimeforVC=true;                                                    // Reset the VC Channels upon boot up
uint8_t onFastVCSelect;                                                         // Remeber if we are fast or slow VC mode

// cmd event
uint8_t eventstate1;                                                            // state of first defined event
uint8_t eventMsgState=0u;                                                        // state of second defined event and first defined event
uint8_t eventNo;                                                                // event counter

uint32_t timedPolling;                                                          // define the timed poll rates for periodic requests on SimpleBGC commands
uint8_t msgToSend;                                                              // Define for the joystick message if we are in CMD_CONTROL mode or we are waiting for CMD_CONTROL_CONFIG

// Sizes of SimpleBGC Structures
uint16_t sz_r=sizeof(r);                                                        // Size of the reset payload
uint16_t sz_c=sizeof(c);                                                        // Size of the control message payload < v2.55b5
uint16_t sz_cnew=sizeof(cnew);                                                  // Size of the control message payload < v2.55b5
uint16_t sz_vc=sizeof(vc);                                                      // Size of the virtual channel payload
uint16_t sz_pida=sizeof(pida);                                                  // Size of the pid autotune payload
uint16_t sz_musicPlay=sizeof(musicPlay);                                        // Size of the music play payload

// Raw ADC Variables
uint16_t XrawComp;                                                              // Compensated X direction
uint16_t YrawComp;                                                              // Compensated Y direction

float32_t pitchAngleReq;                                                        // Angle Request from Joystick (pitch)
float32_t yawAngleReq;                                                          // Angle Request from Joystick (yaw)
float32_t rollAngleReq;                                                         // Angle Request from Joystick (roll)
float32_t pitchSpeedReq;                                                        // Speed Request from Joystick (pitch)
float32_t yawSpeedReq;                                                          // Speed Request from Joystick (yaw)
float32_t rollSpeedReq;                                                         // Speed Request from Joystick (roll)

// SBGC Command Motor
uint8_t commGSMotorsOn;                                                        // Command to turn motors on from Commander GS HMI state1
uint8_t commGSMotorsOnLast=0U;                                                 // Last known Command to turn motors on from Commander GS HMI state1
uint64_t countMotor;                                                            // unconfiemd message re-poll counter
uint8_t motorOffState;                                                          // Motor off state uint16_t

uint8_t commGSAutoPID;                                                          // Command to auto tune on from Commander GS HMI
uint8_t commGSAutoPIDLast=0U;                                                   // Last known Command to auto tune on from Commander GS HMI

unsigned char selectedProfile=1U;                                               // PID profile selected
unsigned char selectedGVS=2U;                                                   // Selection Gain vs Stability

// SBGC Cmd_Get_Angles
COMGS_cmd_get_angles_t commangles;                                              // a struct for the comGS to display values.
COMGS_temp_t commtemp;                                                          // a struct for the comGS to display temperatures
COMGS_get_vars userselect;                                                      // a struct for the comGS user selected values
COMGS_gyro_acc_data_t commgyroacc;                                              // Gyro and Acc data
COMGS_real_data_3_t commgsrealdata3;                                            // other data from realtimedata3 request

uint8_t commGSMotor;                                                            // State variable storing the request state for commands composed from Com GS
uint8_t commGSPID;
uint64_t countPID;                                                              // message re-send poll request on no confirm

uint16_t commGSWriteFileState;                                                  // state of write file request returned by the confirm command
uint8_t timedMsgToSend;                                                         // state engine for the polled requests

// SBGC Command Menu
uint8_t commGSMenu;                                                             // Ground station execute menu command as listed in the state list below
uint8_t commGSMenuLast;
uint64_t countMenu;                                                             // Counter for the re-semd poll on non confirmed request

// SBGC Command Adj_Var_Val
uint8_t commGSAdjvar;                                                           // Command for sending an adj var set from Commander GS HMI
float32_t commGSPIDGain;                                                        // Value for P sent from Commander GS
uint8_t commGSAdjvarLast=0U;                                                   // Last known Command to adjust variables from Commander GS HMI
uint64_t countAdjvar=0U;                                                        // Timer counter for message re-send

// SBGC Command Get_var_val
uint8_t commGSGetvar;                                                           // Command for sending an adj var get from Commander GS HMI
uint8_t commGSGetvarLast=0U;                                                   // Last known Command to get variables from Commander GS HMI
uint64_t countGetvar;                                                           // message re-poll timer (no confirm)

// SBGC Command IMU
uint8_t commGSImu;                                                              // Command for sending imu selection from Commander GS HMI
uint8_t commGSImuLast=0U;                                                      // Last known Command for imu selection from Commander GS HMI
uint64_t countImu;                                                              // re-send poll counter if no confirm of message

// SBGC Command Profile set
uint8_t commGSProSet;                                                           // Command to save profile from Commander GS HMI
uint8_t commGSProSetLast=0U;                                                   // Last known Command to save profile from Commander GS HMI
uint64_t countProSet;                                                           // re-send poll counter if no confirm of message
uint8_t commGSSlotNo;                                                           // Requested slot from CommGS GUI

// Command Control (RC Mode)
uint8_t modeAlreadyRC=false;                                                    // We have entered CMD_CONTROL(RC Mode)
uint8_t modeInConfirm=false;                                                    // We have configured CMD_CONTROL for confirm message back, confirm NO_CONTROL before we preceed to RC Mode
uint8_t ccReturn;                                                               // Control_CMD_CONFIG return code
uint64_t countControlCnf;                                                       // Counter for the CMD_CONTROL_CONFIG command

// SBGC Command Control Config
uint8_t commGSConSet;                                                           // Command to save profile from Commander GS HMI
uint8_t commGSConSetLast=0U;                                                    // Last known Command to save profile from Commander GS HMI
uint64_t countConSet;                                                           // re-send poll counter if no confirm of message
uint8_t commGSConfConVal;                                                       // Requested value from CommGS GUI for paramter specified e.g. lpf / expo etc

// SBGC Command run a script
uint8_t commGSRunAScript;                                                       // state engine for running a script
uint64_t countRunAScript;                                                       // counter for message re-poll
uint8_t commGSScriptSigLast;                                                    // last known request from Com GS
uint8_t commGSScriptSig;                                                        // request from Com GS
COMGS_Script_t comScriptParams;                                                 // Data written from HMI for script angles and speeds

// SBGC command to calibrate motor encoder
uint8_t commGSEncLast;                                                          // last request state
uint8_t commGSEnc;                                                              // Com GS requests a calibration of the motor encoder

uint8_t SBGCcnt;                                                                // count used for the SBGC serial message out (byte by byte send)
#endif                                                                          // ==================   eNd  SBGC    =========================================

// --------------------------------- GPS ---------------------------------------
#if  (defined(GPS_INCLUDED2) || defined(GPS_INCLUDED))                          // We included the GPS System 2 UBLOX or Furano Type
volatile GPS_Info_t g_posDataGPS;                                               // Global structure for containing data read from the GPS in interrupt
volatile unsigned char navStatGPS[50u];                                         // navigation Status description
volatile unsigned char positionMode[20u];                                       // description of the position mode to be used with checksum for validity
#endif

#if defined(GPS_POS_SPRAYER)
geoCoord_Square_t square[NO_SPRAY_OF_AREAS];                                    // there are NO_SPRAY_OF_AREAS squares that make up the grid
   // GPS_Info_t locOfSpray;                                                      // position given by the gps
float32_t arrOfSpraySpt[NO_SPRAY_OF_AREAS] = { 23.4f, 34.7f, 7.6f, 9.1f };      // pid setpoint for each area
pidBlock_t posSprayDrive;                                                       // pid algorthm for the sprayer inverter
uint16_t sprayTimeout=0u;                                                       // initialise at zero
uint8_t sprayArea=0u;                                                           // initialise at zero
uint32_t feedback=5670L;                                                        // measurement to loop e.g. speed (dummy variable)
uint16_t pwm_period=10u, channel=1u;                                            // pwm period and channel
uint8_t posSprayerState=0u;
uint8_t fld=0u;                                                                 // loop iterator for each field
#endif

#ifdef RUN_CAM_USED
// ------------------------------ RUN CAM CAMERA -------------------------------
int8_t RunCcnt=0;                                                               // counter used to send through serial
// The structures dealing with the RunCam Camera

// Request for Camera Info 0x0
RC_req_get_info_t RCCamReqInfo = { RC_STX_CHAR, RC_PROTO_CMD_GET_DEVICE_INFO, 0x60U  };
RC_res_get_info_t RCCamRepInfo;                                                 // Camera info reply information

// Perform Control Actions with the camera
RC_req_action_handshake_t RCCamControl = { RC_STX_CHAR, RC_PROTO_CMD_CAMERA_CONTROL, RC_PROTO_SIMULATE_POWER_BTN, 0xE7U  }; // Camera Control default switch power on
RC_res_keyaction_t RCCamKeyConfirm;                                             // Confirmation to key action requests
RC_res_handshake_t RCCamHandshakeConfirm;                                       // Confirmation of handshake action

// Set up the keypress sequence default here is = 6 downs then confirm = {SHARP_VIEW on the Racer 2}
uint8_t RCCamKeySequence[10u] = { RC_PROTO_5KEY_SIMULATION_DOWN, RC_PROTO_5KEY_SIMULATION_DOWN, RC_PROTO_5KEY_SIMULATION_DOWN, RC_PROTO_5KEY_SIMULATION_DOWN, RC_PROTO_5KEY_SIMULATION_DOWN, RC_PROTO_5KEY_SIMULATION_DOWN, RC_PROTO_5KEY_SIMULATION_SET, 0, 0, 0, };

// Request to read a setting (e.g. read reamianing disk recording time left)
RC_req_getread_settings_t RCCamReqReadSetup = { RC_STX_CHAR, RC_PROTO_CMD_READ_SETTING_DETAIL, RC_PROTO_SETTINGID_DISP_REMAIN_RECORDING_TIME, 0x00U, 0xBCU  };
RC_res_read_settings_t RCCamRepReadSetup;                                       // Read Set-up parameter request reply

// Request to write a setting (e.g. NTSC or PAL)
RC_req_write_settings_t RCCamReqWriteSetup = { RC_STX_CHAR, RC_PROTO_CMD_WRITE_SETTING, RC_PROTO_SETTINGID_DISP_TV_MODE, 0x00U, 0x55U  };
RC_res_handshake_t RCCamRepWriteSetup;                                          // Reply to the write settings request

RC_res_get_settings_t RCGetSettings;                                            // Container to hold a get settings request if we asked for one (supported in readback)

RC_req_write_str_t RCText2Screen = { RC_STX_CHAR, RC_PROTO_CMD_DISP_WRITE_HORT_STRING, 0x1DU, 0x05U, 0x05U, "Please enter your text on HMI", 0x78U };   // Run Cam horizontal text write

RC_change_queue_t RCCamRequest;                                                 // Object controlling camera requests to ensure only one at a time

unsigned char RCBuffer[10u];                                                     // Buffer to store RunCam message to send
uint32_t RCRequestTimer;                                                        // Counter for message timeout

uint8_t g_RunCamSent;                                                           // Current message being sent
uint8_t RCCameraResp;                                                           // Reply from the Run Cam message returned

// GUI variables
RC_Hmi_Request_t hmiRCRequest;                                                  // Pushbuttons on GUI for change of RunCam States

uint8_t RCcrc=0U;                                                               // Run Cam 8bit CRC_DVB_S2 for outgoing message
uint8_t RCRepcrc=0U;                                                            // Run Cam 8bit CRC_DVB_S2 for incoming message
uint8_t RCRequestUDP=0U;                                                        // UDP return code

unsigned char hmiRCCamMemory[32u];                                              // the Run Cam remaining memory shown in manual to be a string
unsigned char hmiRCCamTimeLeft[32u];                                            // the Run Cam remaining time shown in manual to be a string

unsigned char hmiRCCamNtscPal[5u];                                              // String returned for Mode NTSC/PAL
unsigned char commGSRCCamMode[5u];                                              // Screen request for Run Cam to be NTSC or PAL
unsigned char commGSRCCamModeLast[5u];                                          // Last known screen request for Run Cam to be NTSC or PAL
unsigned char hmiHorizText[RC_MAX_TXT_LINE];                                    // The maximum length of the horizontal text line

uint32_t RCtimedPolling;                                                        // define the timed poll rates for periodic requests on Run Cam commands
#endif                                                                          // ==================== eNd RunCam ==================================

#if (CAMERA_TYPE == XS_CAM)
// ----------------------------- VIDEO ENCODER ---------------------------------
// The structures dealing with the XStream Video Encoder set-up and control

// Login message must be done at boot up
XS_login_t XStreamLogin = { 0x24U, "AUTHENTICATE", 0x2CU, "USER", 0x2CU, "PASSWD", 0x0DU };
// Set up the RTSP Port to 554
XS_rtspport_t XStreamRTSP = { 0x24U, "RTSPPORT", 0x2CU, XS_RTSP_Port, 0x0DU };
// Enable all streams multicast
XS_stream_t XStreamConf = { 0x24U, "STREAM", 0x2CU, -1, 0x2CU, "MULTICAST", 0x2CU, 1, 0x0DU };
// Configure Network
XS_network_t XStreamNetwork = { 0x24U, "NETWORK", 0x2CU, 0U, 0x2CU, "STATIC", 0x2CU, XSencoderIP, 0x2CU, "255.255.255.0", 0x2CU, XSgatewayIP, 0x0DU };
// Set the data disk to Number 1
XS_datadisk_t XStreamDisk = { 0x24U, "DATADISK", 0x2CU, 1U, 0x0DU };

// Realtime data change messages  (all channels are -1)

// bit rate of all channels to 2Mbps
XS_bitrate_t XStreamBitRate = { 0x24U, "BITRATE", 0x2CU, -1, 0x2CU, 2000000UL, 0x0DU };

// channel interval for all to 60 frames
XS_iframeinterval_t XStreamIInterval = { 0x24U, "IINTERVAL", 0x2CU, -1, 0x2CU, 60U, 0x0DU };

// channel frame rate for all to 7.5 fps
XS_framerate_t XStreamFrameRate = { 0x24U, "FRAMERATE", 0x2CU, -1, 0x2CU, 15U, 0x2CU, 2U, 0x0DU };

// channel brightness for all to between 0-10000
XS_brightness_t XStreamBrightness = { 0x24U, "BRIGHTNESS", 0x2CU, -1, 0x2CU, 5000U, 0x0DU };

// channel contrast for all to between 0-10000
XS_contrast_t XStreamContrast = { 0x24U, "CONTRAST", 0x2CU, -1, 0x2CU, 5000U, 0x0DU };

// channel hue for all to between 0-10000
XS_hue_t XStreamHue = { 0x24U, "HUE", 0x2CU, -1, 0x2CU, 5000U, 0x0DU };

// channel saturation for all to between 0-10000
XS_saturation_t XStreamSaturation = { 0x24U, "SATURATION", 0x2CU, -1, 0x2CU, 5000U, 0x0DU };

// channel clip size for all to 2000 MB
XS_clipsize_t XStreamClipSize = { 0x24U, "CLIPSIZE", 0x2CU, -1, 0x2CU, 2000U, 0x0DU };

// channel clip length for all to 1800 secs
XS_cliplength_t XStreamCliplength = { 0x24U, "CLIPLENGTH", 0x2CU, -1, 0x2CU, 1800U, 0x0DU };

// start recording (commanded from the GUI)  on channel 0
//SCORD_XS_record_t XStreamRec = { 0x24, "RECORD", 0x2C, 0, 0x0D };             Done the other way

// stop recording (commanded from the GUI)  on channel 0
//SCORD_XS_stop_t XStreamStop = { 0x24, "STOP", 0x2C, 0, 0x0D };

// mark recording (commanded from the GUI)  on channel 0
//XS_mark_recording_t XStreamMark = { 0x24, "MARK", 0x2C, 0, 0x2C, "ABP Trigger Pressed", 0x0D };

// XS Encoder Start-up sequence
uint8_t XSEncoderResp;                                                          // This is a confirm message back from Encoder checks for +OK reply
uint16_t XSReturn;                                                              // Return code from the send for XS initialisation sequence
uint8_t XSStartUpSequence=CMD_SEND;                                             // State of the XS Initialisation messages, initialise to start
uint64_t XSLoginTime=0U;                                                        // Timer waiting for ok resposne at encoder initialisation
uint8_t XSInitState=0U;                                                         // Start up initialisation sequence for encoder (start @ login step)

uint8_t XSChangeOfStateQueue[8u];                                                // Word indicating which items have changed state

// XS encoder contrast message
uint32_t XSRequestTimer;                                                        // Tick counter contrast time
uint16_t XSRequestUDP;                                                          // XS encoder message udp send return code, used to control re-send

uint16_t hmiRawFrameRtScalar[4u];                                                // Scalar value for the frame rate (per channel)

XS_Hmi_Slider_t  hmiRawContrast[4u];                                             // encoder Contrast request state (per channel)
XS_Hmi_Slider_t  hmiRawHue[4u];                                                  // encoder Hue request state (per channel)
XS_Hmi_Slider_t  hmiRawSaturation[4u];                                           // encoder Saturation request state (per channel)
XS_Hmi_Slider_t  hmiRawClipSz[4u];                                               // encoder Clip size request state (per channel)
XS_Hmi_Slider_t  hmiRawCliplen[4u];                                              // encoder Clip length request state (per channel)
XS_Hmi_Slider_t  hmiRawBright[4u];                                               // encoder Brightness request state (per channel)
XS_Hmi_Slider_t  hmiRawFramert[4u];                                              // encoder Frame Rate request state (per channel)
XS_Hmi_Slider_t  hmiRawFramescl[4u];                                             // encoder Frame Scale request state (per channel)
XS_Hmi_Slider_t  hmiRawFrameint[4u];                                             // encoder Frame Interval request state (per channel)
XS_Hmi_Slider_t  hmiRawBitrate[4u];                                              // encoder Bit rate request state
uint8_t XSChan;                                                                 // current encoder channel number

XS_Hmi_Slider_t hmiTimeCalcTotal;                                               // Calculated from the Time Entered on the HMI
XS_Hmi_Slider_t hmiDateCalcTotal;                                               // Calculated from the Date Entered on the HMI

XS_change_queue_t XS_encoder_Q;                                                 // The messages to send to the encoder queue (all real time encoder requests) and queue element

// XS encoder variables for display
XS_channel_t gXSBitRate;                                                        // attributes returned per channel
XS_channel_t gXSAvgBitRate;
XS_channel_t gXSIInt;
XS_channel_t gXSFrameRate;
XS_channel_t gXSFrameScale;
XS_channel_t gXSBright;
XS_channel_t gXSContrast;
XS_channel_t gXSHue;
XS_channel_t gXSSatur;
XS_channel_t gXSClipsz;
XS_channel_t gXSClipLn;
XS_edisk_t gXSEdisk;                                                            // attributes associated with Disk
uint32_t gXSRTSPPort;                                                           // RTSP port number being used
uint32_t gXSFileDelThreshold;                                                   // REcycle % (file deletion threshold)

uint8_t g_secondsLow;
uint8_t g_secondsHigh;
uint8_t g_secondsActual;                                                        // actual seconds from the MCU RTC

uint8_t COMGS_New_Password;                                                     // Set to true by GUI interface when new user or password has been entered
uint8_t COMGS_New_Network;                                                      // Set to true by GUI interface when new network settings been entered

COMGS_Request_t XSEncButtons1;                                                  // XS Encoder button type for HMI (binary resets and commands)
int8_t XSRecordChan;                                                            // Channel chosen to record on

unsigned char hmiEncoderIP[16u];                                                 // Encoder IP Address used for RTSP on the HMI screen
unsigned char hmiEncoderGW[16u];                                                 // Gateway Encoder IP Address used for RTSP on the HMI screen
unsigned char XSencoderIPUse[16u];                                               // Encoder IP Address used for RTSP used in the set-up
unsigned char XSgatewayIPUse[16u];                                               // Encoder gw IP Address used for RTSP used in the set-up

uint8_t XScontrastChan;
uint8_t XShueChan;
uint8_t XSsaturationChan;
uint8_t XSclipszChan;
uint8_t XScliplenChan;
uint8_t XSbrightChan;
uint8_t XSframeRateChan;
uint8_t XSframeIntChan;
uint8_t XSbitRateChan;

unsigned char XSTimeDate[24u];                                                   // A string Holding the date and time string
COMGS_DateTime_t hmiDateTime;                                                   // Data displayed on HMI or result from a totalseconds update

#endif                                                                          // ======================== XS CAM

#ifdef GEF_EGD_PLC
uint8_t respEGD=0u;
#endif

#ifdef ALARM_BANNER_USED
#if (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY))
// Commander Ground Station Objects (Alarms, AHRS data, power etc)
// ==================== ALARMS =================================================
//
// =================== SBGC    =================================================
COMGS_error_alarm SBGCErrorCode;                                                // Alarm Structure for Error Codes
AlarmDefinitionObject_t SBGCDescriptions = { "CMD_ERROR : Read from EEPROM Failed\n", "CMD_ERROR : Cant run Algorythm\n", "CMD_ERROR : Write to EEPROM Failed\n", "CMD_ERROR : Unknown Action\n", "CMD_ERROR : Wrong Command Size\n", ". ", ". ", ". ",1U,2U,3U,4U,5U,6U,7U,8U};
COMGS_error_alarm SBGCPIDTune2Error;                                            // For PID Tuning firmware version v>2.7
AlarmDefinitionObject_t PIDTuErDescriptions = { "PID_ERROR : Read from EEPROM Failed\n", "PID_ERROR : Cant run Algorythm\n", "PID_ERROR : Write to EEPROM Failed\n", "PID_ERROR : Unknown Action\n", "PID_ERROR : Wrong Command Size\n", ". ", ". ", ". ",9U,10U,11U,12U,13U,14U,15U,16U};
COMGS_error_alarm SBGCSystemError1;                                             // For System Error diagnostics word 1
AlarmDefinitionObject_t SBGCSysErDescrip1 = { "REALDATA3_SYS : No Error\n", "ERR_NO_SENSOR : Error No Sensor\n", "ERR_CALIB_ACC : Error Calibration Accelorometer\n", "ERROR_SET_POWER : Error setting power\n", "ERROR_CALIB_POLES : Error Calibrating Poles\n", "ERR_PROTECTION : Error with Protection\n ", "ERR_SERIAL : Communication Error ", "ERR_LO_BAT1 : Error low battery level 1\n",17U,18U,19U,20U,21U,22U,23U,24U};
COMGS_error_alarm SBGCSystemError2;                                             // For System Error diagnostics word 2
AlarmDefinitionObject_t SBGCSysErDescrip2 = { "ERR_LO_BAT2 : Error low battery level 2\n ", "ERR_GUI_VERSION : Error wrong GUI version\n", "ERROR_MISS_STEPS : Error program missed steps\n", "ERROR_SYSTEM : System Error\n", "ERR_ESD : Emergency Stop ", ". ", ". ",25U,26U,27U,28U,29U,30U,31U,32U};
COMGS_error_alarm SBGCESDReason[5];                                             // ESD diagnostics words 1 to 5
AlarmDefinitionObject_t SBGCESDReasonDesc1 = { "SUB_ERR_I2C_ERRORS : ESD due to i2c errors\n ", "SUB_ERR_DRV_OTW : ESD due to driver over temperature\n", "SUB_ERR_DRV_FAULT :  Driver fault undervoltage over-current short circuit \n", "SUB_ERR_ENCODER_IMU_ANGLE : Encoder IMU angles mismatch\n", "SUB_ERR_CALIBRATION_FAILED  Auto calibration process caused serious fault\n", "SUB_ERR_INTERNAL_SYSTEM_ERROR Stack is damaged\n", "SUB_ERR_ENCODER_CALIB_BAD_SCALE estimated scale differs a lot from configure\n ", "SUB_ERR_OVER_TEMPERATURE MCU or power board over temperature\n", 25U,26U,27U,28U,29U,30U,31U,32U};
AlarmDefinitionObject_t SBGCESDReasonDesc2 = { "SUB_ERR_BAD_MOTOR_POLES_INVERT : Motor n.poles or inversion is wrong\n ", "SUB_ERR_NOT_ENOUGH_MEMORY : Static_malloc() can't allocate memory\n", "SUB_ERR_IMU_SENSOR_NOT_RESPONDING : Lost connection to IMU sensor \n", "SUB_ERR_CAN_HARD : CAN on board hardware error\n", "SUB_ERR_MOTOR_OVERHEAT_PROTECTION : Overheat protection is triggered\n", "SUB_ERR_MOTOR_IS_LOCKED : Motor is locked during automated task\n", "SUB_ERR_BAD_IMU_HEALTH : IMU gyroscope and accelerometer error is too big: sensor sends corrupted data or wrong use conditions\n ", "SUB_ERR_INFINITE_RESET : Infinite reset loop is detected\n", 33U,34U,35U,36U,37U,38U,39U,40U};
AlarmDefinitionObject_t SBGCESDReasonDesc3 = { "SUB_ERR_WRONG_INITIAL_POSITION : Wrong position: failed to detect encoder angle or angle is outside soft limits\n ", "SUB_ERR_MOTOR_LOAD_TIME_EXCEEDED : Motors are fully loaded too long time\n", "SUB_ERR_CAN_DRV_OVERCURRENT : Hardware short-circuit protection \n", "SUB_ERR_CAN_DRV_UNDERVOLTAGE : Hardware or software undervoltage protection\n", "SUB_ERR_CAN_DRV_EMERGENCY_PIN : External emergency is triggered\n", "SUB_ERR_CAN_DRV_FOC_DURATION : FOC algorithm duration error\n", "SUB_ERR_CAN_DRV_MCU_OVERHEAT : Driver temperature is to high\n ", "SUB_ERR_CAN_DRV_MOTOR_OVERHEAT : Motor temperature is to high\n", 41U,42U,43U,44U,45U,46U,47U,48U};
AlarmDefinitionObject_t SBGCESDReasonDesc4 = { "SUB_ERR_CAN_DRV_OVERCURRENT_SOFT : Current through motor exceed limit\n ", "SUB_ERR_CAN_DRV_SEVERAL : Several errors on driver \n", "SUB_ERR_CAN_EXT_BUS_OFF : CAN bus high rate errors of slave controller \n", "SUB_ERR_CAN_INT_BUS_OFF : CAN bus high rate errors of main controller\n", "SUB_ERR_ENCODER_NOT_FOUND : No answer from encoder during initiatise\n", "SUB_ERR_CAN_DRV_NOT_RESPONDING : Lost connection to CAN Drv\n", "SUB_ERR_CAN_DRV_WRONG_PARAMS : Some params of CAN Drv isn't correct\n ", "SUB_ERR_OVERCURRENT : Fast over current protection of main controller, or short circuit detection on startup\n", 49U,50U,51U,52U,53U,54U,55U,56U};
AlarmDefinitionObject_t SBGCESDReasonDesc5 = { "SUB_ERR_UNSAFE_VOLTAGE : Under voltage protection or supply protection controller fault\n ", "SUB_ERR_WRONG_FULL_BAT_VOLTAGE_PARAM : Battery voltage is higher than expected at startup sequence\n", "SUB_ERR_EEPROM_PARAMS_CORRUPTED : Parameters are corrupted in EEPROM and can't be restored from backup slot\n", " ", " ", " ", "  ", " ", 57U,58U,59U,60U,61U,62U,63U,64U};
COMGS_error_alarm JOYCalAlarms;                                                 // Joystick calibration alarms
COMGS_error_alarm ScriptRunAlarms1;                                             // Script Run alarms
AlarmDefinitionObject_t ScriptRunAlarmDesc1 = { "NO_ERROR : No errors occurred in script\n ", "ERR_EEPROM_FAULT : Script run error with EEPROM \n", "ERR_FILE_NOT_FOUND : Script run error no file? \n", "ERR_FAT : Script run file allocation table error\n", "ERR_NO_FREE_SPACE  : Script run error no space on drive\n", "ERR_FAT_IS_FULL : Script run error File Allocation Table is full\n", "ERR_FILE_SIZE : Script run error file size is zero\n ", "ERR_CRC : Script run error with cyclic redundancy chack\n", 65U,66U,67U,68U,69U,70U,71U,72U};
COMGS_error_alarm ScriptRunAlarms2;                                             // Script Run alarms
AlarmDefinitionObject_t ScriptRunAlarmDesc2 = { "ERR_LIMIT_REACHED : Script run error maximum limit of files reached\n ", "ERR_FILE_CORRUPTED : Script run error file is not a valid script\n", "ERR_WRONG_PARAMS : Script run error invalid parameters passed to script\n", " ", " ", " ", " ", " ", 73U,74U,75U,76U,77U,78U,79U,80U};
COMGS_error_alarm MotorAlarms;                                                  // Motor alarms
AlarmDefinitionObject_t MotorAlarmDesc = { " ", " ", " ", "MTR_ESD : Motor Stop ESD occurred\n", "MTR_FTS : Gimbal Motor failed to start or stop\n", "MTR_HLD : Gimbal motor in hold state\n", "CAL_ERR : Encoder calibration error please start motor first\n", " ", 81U,82U,83U,84U,85U,86U,87U,88U};
COMGS_error_alarm ExtIMUAlarms1;                                                // External IMU alarms word 1
AlarmDefinitionObject_t ExtIMUAlarmDesc1 = { "STATUS_DISABLED : External IMU disbaled\n", "STATUS_NOT_CONNECTED : External IMU not connected properly\n", "STATUS_UNKNOWN : Unknown state for external IMU\n", "STATUS_ERROR : External IMU is in error\n", "STATUS_BAD : External IMU in bad data state\n", "STATUS_COARSE : External IMU in coarse alignment\n", "STATUS_GOOD : External IMU Status is good\n", "STATUS_FINE : External IMU is in fine static alignment (high accuracy)\n", 89U,90U,91U,92U,93U,94U,95U,96U};
COMGS_error_alarm ExtIMUAlarms2;                                                // External IMU alarms word 2
AlarmDefinitionObject_t ExtIMUAlarmDesc2 = { "STATUS_BAD_MAG : External IMU bad magnetrometer\n", "STATUS_NO_GPS_SIGNAL : External IMU no GPS Signal\n", " ", "  ", " ", " ", " ", " ", 97U,98U,99U,100U,101U,102U,103U,104U};

AlarmStack_t AlarmStack;                                                        // FIFO stack for most recent alarms to display (if we have SD Card make file to remember it)
AlarmLineObject_t AlarmBanner1;                                                 // The main Alarm Banner at bottom of displays
uint8_t countEsdAlarms;                                                         // Counter for ESD Alarm word processing
#endif                                                                          /// ================= eNd SimpleBGC Alarm ===================================
#if (CAMERA_TYPE == XS_CAM)
// ================ XS Encoder Alarms ==========================================
COMGS_error_alarm XSEncoderAlarms1;                                             // XS Encoder Alarms word
AlarmDefinitionObject_t XSEncoderAlarmsDesc1 = { "XS Encoder: user password incorrect\n ", "XS Encoder: invalid login name and paddword\n", "XS Encoder: Error with login to unit (retry)\n", "XS Encoder : Channel in wrong state \n", "XS Encoder : Could not configure RTSP \n", "XS Encoder : Wrong Time and Date Specified \n", "  ", " ", 1U,2U,3U,4U,5U,6U,7U,8U};
AlarmStack_t XSAlarmStack;                                                      // make new FIFO for Encoder
AlarmLineObject_t XSAlarmBanner;                                                // encoder alarm banner
#endif                                                                          // ================= eNd XS Encoder Alarm ==================================
#endif                                                                          // ================= eNd Alarm Handler =====================================

// =============== Data to display =============================================
#if (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY))
COMGS_magnet_t commgsmagnet;                                                    // object for magnetometer readings
COMGS_motor_power_t commgsmtrpwr;                                               // object for motor power
COMGS_quarternion quartAHRS;                                                    // quarternion co-ordinate structure to hold calculations from AHRS
COMGS_pid_object PIDBlock;                                                      // PID Block for the current profile
uint8_t eachPID;                                                                // for counter for each pitch,roll,yaw pid
COMGS_Motor_t gimbalMotor;                                                      // Motor Status object for GUI
// set up an array to show the Status of each slots script state
uint8_t scriptStateSlot[SBGC_NO_OF_SLOTS];                                              // Status of the scripts for each slot, each index is a slot
uint8_t commGSEvSet;                                                            // event chosen from HMI
uint8_t commGSEvTyp;                                                            // event type or state to monitor chosen from HMI
#endif                                                                          // =============== eNd Gimbal HMI =========================================

#ifdef SBGC_GIMBAL_JOY
// Calibration check on the joystick center (correction object)
COMGS_joy_calib_t calibrationObj= { 1024u>>1u, 1024u>>1u, 1024u>>1u, 1024u, 1024u, 1024u, 0u, 0u, 0u };    // Object containing corrections initialised as what ANI should be
#endif                                                                          // ============== eNd Gimbal Joystick =====================================

#ifdef YI_CAM_USED
// ========================= Xiong Yi Action Cam ===============================
XY_reply_t XYjsonReply;                                                         // JSON reply structure from Yi Cam webserver
//SOCKET_Intern_Dsc *YiSocket;                                                    // TCP ip socket
XY_JSparse_t XY_ParseStruct;                                                    // word holding the parse result of what keys where found in a typedef struct
int16_t XY_ParseState;                                                          // word holding the parse result of what keys where found
COMGS_YiDig_t hmiYiValues;                                                      // Structure containing data for the HMI screen
uint8_t XY_retTCP;                                                              // Socket open command return code
uint8_t XYuseToken=0U;                                                          // token returned from the stream
uint16_t xyCamStateTimer;                                                       // Timer which counts for each XY action cam ajax request until valid response or retry

COMGS_YiOptSelect_t hmiXYOption;                                                // number on HMI selecting the video resolution range 0-11 (4bit number) photo size selecter ranged 0-3 (2bit number)
//uint8_t XYRequest=0U;                                                         // Current button request (HMI Action Queue)
unsigned char XY_zoomString[20u];                                               // String containing zoom options ???
uint16_t XY_hmiBitRate;                                                         // value representing bit rate
unsigned char XY_hmiDelFilNm[60u];                                              // full file path for file you want to delete e.g. /tmp/fuse_d/DCIM/129MEDIA/YDXJ2007.jpg
unsigned char XY_formatString[40u];                                             // format string
unsigned char XY_dirString[40u];                                                // directory listing string
unsigned char XY_hmiSndGetFilNm [60u];                                          // File to get or send
unsigned char XY_hmiGetMDA[60u];                                                // string to get media info (file and path name ??)
unsigned char XY_hmiThumb[60u];                                                 // file for thumb nail
uint8_t XY_thumbInt;
uint16_t XY_offset;                                                             // file offset
uint16_t XY_fet;                                                                // file fet
unsigned char XY_md5[MD5_MAX_LEN];                                              // md5 for file you want to put as script
uint32_t XY_size;                                                               // size for file you to put as script
XY_config_t XYConfig;                                                           // structure to hold the camera configuration
unsigned char XYstringStore[XY_MSG_MAX_LEN];                                    // temporary string to hold configuration reply
#endif                                                                          // ====================== eNd Yi Action Cam =================================

//--------------------------- LIDDAR -------------------------------------------
#if defined(UART6_INTERUPT) || defined(LWNX_LIDDAR_USED)
lwResponsePacket LwNxResponse;                                                  // Liddar response message packet
lwHardware_t LwNxHardWare;                                                      // Structure containing the hardware
uint8_t liddarMsgState=0U;                                                      // state engine for liddar message sending
unsigned char firmwareVersionStr[16U];
lwOutputData_t LwNxOutput;                                                      // output data structure from the liddar
uint16_t LwNxPoint=0U;                                                          // counter up to number of points returned by the stream
uint8_t LwNxCollection=0U;                                                      // state engine to save n collections returned
uint32_t LwNxTimerTicks=0L;                                                     // tick counter that restarts streaming after LWNX_MAX_WAIT ticks
uint8_t LwNxWriteI32[4U];                                                       // value array to write uint32 out
uint8_t LwNxWriteI8;                                                            // value to write uint8_t out
COMGS_SF40OptSelect_t hmiLwNx;                                                  // selections on the HMI
uint8_t LwNxLaserState;                                                         // read back state of laser
uint16_t LwNxLaserCnt;
uint32_t LwNxTemp;
uint32_t hmiLwNxTemp;                                                           // temperature to display on GUI
uint32_t LwNxVoltRaw;
uint32_t hmiLwNxVolt;                                                           // calculated incoming voltage
uint8_t LwNxMotorState;                                                         // states of the motor
uint8_t LwNxAlarmWord;                                                          // liddar alarm word
uint32_t LwNxPeriodCnt;                                                         // period counter used to trigger polled event updates
#define LWNX_PERIOD_ALARM 200U                                                  // period for alarm updates
#define LWNX_PERIOD_TEMP 20000U                                                 // period for temperature updates
lwAlarm1_t LwNxAlarmConfig;                                                     // structure containing the alarm config
uint8_t hmiAlarmNo;                                                             // alarm number to configure from GUI
#endif
//-------------------------- INTERUPT ------------------------------------------

//volatile uint8_t g_SBGC_Stx_Char = SBGC_CMD_START_BYTE;                         // The SBGC command start byte depends on version

#ifdef UART4_INTERUPT
volatile uint8_t g_extended = 0U;                                               // Camera encoder reply message was extended type
#endif
// These are times made from the global timer counting in the interrupt above
int64_t cycTimeDuration;                                                        // cycle time duration
uint64_t cycTimeDurationLast;                                                   // cycle time absolute last cycle value
uint64_t cycTotalTime;                                                          // Total cycle times since reset performed

uint16_t totalBadUDPSends=0U;                                                   // Declare a counter of bad send messages in total and when it exceeds its limit go to boot ARP
uint8_t resetDone;                                                              // already did a reset on bad number of sends
#define MAX_REPEAT_TIME 10000U

// declare heap size
#ifdef CS_FROZEN_FROZEN_H_
const HEAP_SIZE = 25U;                                                          // initialisation for Memory Manager _LibMemoryManager.h (Malloc and Free)
// const HEAP_START = NULL;
#endif

sprayVehicleState_t SprayerVehicle;                                             // TODO : put into spray section

#if defined(BATCH_MIXER)
mmc_file_read_t *fileDatInput;                                                  /* mmc data file structures for initialisation */
mov_avg_data_t levelMovAvg;                                                     /* moving average of level instrument */
uint8_t curProduct=0u;                                                          /* current product we have chosen to spray on the field */
uint8_t lastProduct=0u;                                                         /* previous product */

mmc_file_writ_t fileWritDat;                                                    /* file write union */
gui_spray_data_t guiFileManager; /* = {false, false, 0u, 0u };   */                    /* create graphical interface object to save and delete product files and data */

#if defined(USE_MMC_SPI2)
int32_t mmcFATresetTm=0u;
uint32_t mmcFATresetTmLast=0u;
#endif

/* ====================  Sequence Alarm definitions  ======================== */
uint8_t BtchAlmWord=0u;                                                         /* alarm word for batch make-up and spraying sequence */
unsigned char realTimeFile[20u];
#define NO_PRODUCT_FOUND (1u<<0u)
#define ERR_NO_REALTM_DAT (1u<<1u)
#define ERR_NO_RECIPE_DAT (1u<<2u)
#define ERR_NO_SPRAY_DAT (1u<<3u)
#define ERR_LEVEL_MEAS (1u<<4u)
#define ERR_ARM_POSN (1u<<5u)
#endif

#if defined(CRAFT_MTR_PIDS)                                                     /* for aero craft motor control */
uint16_t craftSpeedReading[CRAFT_NUM_OF_MTRS], aero_pwm_period=10u, aero_channel[CRAFT_NUM_OF_MTRS];
float32_t reqCraftSpt[CRAFT_NUM_OF_MTRS];                                       /* store for the requested scaled local setpoint */
#if defined(USE_SPEKTRUM)
STRU_TELE_ESC_t SpekBusObject[CRAFT_NUM_OF_MTRS];                               /* RPM is being read over spektrum bus from each motor */
#elif defined(ENCODER_HELPER)
gray_bin_t craftEncoderIn[CRAFT_NUM_OF_MTRS];                                   /* if we are using a gray scale encoder input */
#endif
pidBlock_t craftSpeedPID[CRAFT_NUM_OF_MTRS];                                    /* pid algoryhtm for each motor */
uint8_t craftMtr;                                                               /* loop counter for the current motor in sequence */
uint16_t craftItm;
#if defined(TERRAIN_FOLLOW_USED)
uint8_t craftMtrUp[CRAFT_NUM_OF_MTRS] = { 1u, 2u, 3u, 4u, 5u, 0u, 0u, 0u, 0u, 0u };  /* which motors are on top orientation */
uint8_t craftMtrDwn[CRAFT_NUM_OF_MTRS] = { 6u, 7u, 8u, 9u, 10u, 0u, 0u, 0u, 0u, 0u }; /* which motors are on down orientation */
uint8_t craftMtrLeft[CRAFT_NUM_OF_MTRS] = { 1u, 2u, 6u, 7u, 0u, 0u, 0u, 0u, 0u, 0u }; /* which motors are on left orientation */
uint8_t craftMtrRight[CRAFT_NUM_OF_MTRS] = { 4u, 5u, 10u, 9u, 0u, 0u, 0u, 0u, 0u, 0u }; /* which motors are on right orientation */
uint8_t craftMtrFwd[CRAFT_NUM_OF_MTRS] = { 1u, 4u, 3u, 6u, 0u, 0u, 0u, 0u, 0u, 0u };  /* which motors are on forward orientation */
uint8_t craftMtrBack[CRAFT_NUM_OF_MTRS] = { 2u, 10u, 8u, 7u, 0u, 0u, 0u, 0u, 0u, 0u }; /* which motors are on back orientation */
#endif
#define CRAFT_START_UP_SPEED 10u                                                /* default initial setpoint */
#endif
#if defined(USE_MAV_PULSE_OBJECT)
pulse_obj_t pulseDigitalOutputs;                                                // at present we have one pulsed output at a time consider making more objects here, if you need more in parallel
#endif                                                                          // ================= eNd pulse Object control via MAVLINK =========

#if defined(JRT_LIDDAR_USED)
//extern JRT_write_measure_t distSensIT03Conf;                                           /* Chengdu JRT meter IT03 distance sensor measurement configuration object */
//extern JRT_read_measure_t distSensIT03Meas;                                            /* Chengdu JRT meter IT03 distance sensor measurement reading */
//JRT_write_measure_t distSensIT03Conf;                                           /* Chengdu JRT meter IT03 distance sensor measurement configuration object */
//JRT_read_measure_t distSensIT03Meas;                                            /* Chengdu JRT meter IT03 distance sensor measurement reading */
#if defined(PNEU_HT_CONTROL)                                                    /* we want pneumatic height control from distance sensor to maintain spray cone height */
uint16_t sprayHtdelta=2u;                                                       /* delta for no change on level setpoint in cm */
uint16_t sprayHtSpt=185u;                                                       /* spray height setpoint for keeping spray cone in cm */
uint8_t sprayHtActiveOn=1u;                                                     /* HMI flag to allow user to activate or deactivate the height control (disable if problems) */
#endif /* end pneumatic height */
#endif /* end jrt distance sensor */

#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
uint8_t nextAin=START_MAN_AIN_PIN;
#endif

#if ((AIN_ADC_METHOD == ADC_AUTO_COLLECT) || (AIN_ADC_METHOD == ADC_MAN_COLLECT))
uint16_t valueADC[NUM_OF_AINS+1u];                                              /* index the AINS to reference the pin number as the index */
#endif

//
// ==================== Functions ==============================================
//
// ====================== Position height control ==============================
#if defined(JRT_LIDDAR_USED)
#define JRT_WRITE_RESEND_TIM 20000U
void configDistanceSensor( JRT_write_measure_t *jrtDistSens, USBObject_t *USBObj );
#if defined(PNEU_HT_CONTROL)
void keepSprayHeight(JRT_read_measure_t *distMeas );
#endif /* end pneumatic height */
#endif /* end jrt distance sensor */

#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
// ====================== SBGC =================================================
#if defined(SBGC_GIMBAL_HMI)
void createSBGCScript( SBGC_cmd_write_file_t *writeFile );
void createSBGCTimelapse( SBGC_cmd_write_file_t *writeFile, COMGS_Script_t *configParams);
#endif
uint8_t ChgStat_CONFIG_for_CONTROL(SBGC_cmd_control_config_t *fdata, uint8_t on_off, uint16_t sz_fdata);
void SBGC_Process_Data_Read( uint8_t msgTypeIn );
void SBGC_Play_Sound( unsigned char *pMusicSequence, uint8_t noteLength );
#endif                                                                          // ================ eNd SBGC Functions ====================================
#ifdef YI_CAM_USED
// ===================== Yi Action Cam =========================================
int8_t XY_Check_Return_Code( uint8_t msgTypeIn );
#endif                                                                          // ================ eNd Yi Action Cam =====================================
// ====================== Sequoia ==============================================
#ifdef SEQ_CAM_USED
int16_t sendJSONConfig2Sequoia( SEQStat_Config_t *confStruct );
int16_t getJSONConfigFrmSequoia( SEQStat_Config_t *confStruct, const char *str );
int16_t chkJSONreplyFrmSequioa( const char *str, const char *cmd );
#endif                                                                          // ================= eNd Parrot Sequioa using frozen lib for JSON =========

#if defined(UBIBOT_DEVICE_USED)
int16_t priNt_mY_uBiConfig( char * buf, const uBiMetaData_t *p);
#endif                                                                          // ================= eNd uBiBot using frozen lib for JSON =========

#if (CAMERA_TYPE == XS_CAM)
uint8_t XSChangeDetected( const XS_Hmi_Slider_t *ptr_HMIobject );
#endif

// ===================== MAVLINK ===============================================
#if defined(MAV_GIMBAL_CTRL)
mavlink_attitude_quaternion_t mavQuartSpd;
#endif
#if defined(USE_MAVLINK)
mavlink_message_t mavMsgRcv;
mavlink_status_t mavStat;
#endif

uint8_t mainStartState= 0u;                                                     /* state which represents bit wise what we have done at start-up */
//
//
// --- Functions used directly in main program
//
#if defined(JRT_LIDDAR_USED)
/*-----------------------------------------------------------------------------
 *      configDistanceSensor(): set up JRT distance sensor over usb link 
 *
 *  Parameters: JRT_write_measure_t *jrtDistSens, USBObject_t *USBObj
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void configDistanceSensor( JRT_write_measure_t *jrtDistSens, USBObject_t *USBObj )
{
   if ((jrtDistSens == NULL) || (USBObj == NULL))
   {
      return;
   }

   if ((USBObj->State==USB_PACKET_READ_CONFIG) || ((USBObj->State==USB_PACKET_READ_DATA) && (USBObj->WriteState==USB_CONFIG_WRITE_START)))
   {
      USBObj->WriteState=USB_CONFIG_COMPLETE;
      return;
   }

   switch(USBObj->WriteState)
   {
      case USB_CONFIG_WRITE_START:
      jrtDistSens->MsgType=JRT_WRITE_START_MEASURE_STX;
      jrtDistSens->MsgCode=JRT_WRITE_START_MEASURE_MSG;
      jrtDistSens->TransId=0u;
      jrtDistSens->PayLoadLen=JRT_READ_STARTSTOP_PAYLOAD_LEN;
      jrtDistSens->Payload[0u]=JRT_WRITE_START_MEASURE_CMD;
      USBObj->WriteState=USB_CONFIG_SETUP;

      case USB_CONFIG_WRITE_STOP:
      jrtDistSens->MsgType=JRT_WRITE_START_MEASURE_STX;
      jrtDistSens->MsgCode=JRT_READ_STARTSTOP_MEASURE_MSG;
      jrtDistSens->TransId=0u;
      jrtDistSens->PayLoadLen=JRT_READ_STARTSTOP_PAYLOAD_LEN;
      jrtDistSens->Payload[0u]=JRT_WRITE_STOP_MEASURE_CMD;
      USBObj->WriteState=USB_CONFIG_SETUP;

      case USB_CONFIG_SETUP:
      memcpy((void*) USBObj->WriteBuffer, (void*) jrtDistSens, JRT_STARTSTOP_LEN);
      /* state is sending a config start request if it blocks we trap this in a timed interrupt (count 2 @ this state) then use USB_Break */
      USBObj->WriteState=USB_SEND_CONFIG_PACKET;

      case USB_SEND_FAIL:
      USBObj->WriteState=USB_CONFIG_SETUP; /* after 2 scans try again */

      case USB_SEND_CONFIG_PACKET:
      USBObj->writeTimeOut=0u;
      if (Gen_Write(USBObj->WriteBuffer,JRT_STARTSTOP_LEN,MY_USB_HID_EP_IS)!=JRT_STARTSTOP_LEN)   /* send this on USB : USB_HID_EP=1 */
      {
         USBObj->WriteState=USB_SEND_FAIL; /* state hasnt done it */
      }
      else
      {
         USBObj->WriteState=USB_SENT_CONFIG_PACKET; /* config write complete now wait for the config reply */
      }
      break;

      case USB_SENT_CONFIG_PACKET:
      USBObj->writeTimeOut=++USBObj->writeTimeOut % UINT16_MAX;
      if (USBObj->writeTimeOut > JRT_WRITE_RESEND_TIM)
      USBObj->WriteState=USB_SEND_CONFIG_PACKET;

      default:
      USBObj->WriteState=USB_SEND_CONFIG_PACKET;
      break;

   }

}
#if defined(PNEU_HT_CONTROL)
/*-----------------------------------------------------------------------------
 *      keepSprayHeight(): keep spray height using distance sensor  
 *
 *  Parameters: JRT_read_measure_t *distMeas
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void keepSprayHeight(JRT_read_measure_t *distMeas )
{
   if ((distMeas->data_valid_ind==0u) && (sprayHtActiveOn==1u))                 /* measurement subject to low light consider a lux sensor and light to assist */
   {
      if (distMeas->filter_dist > (sprayHtSpt + sprayHtdelta))                  /* nozzle is too high */
      {
        AuxDevState = (AuxDevState & (~hydLevlUp)) | hydLevlDwn;                /* drive pnuematics downward check against up_MIN */
      }
      else if (distMeas->filter_dist < (sprayHtSpt - sprayHtdelta))             /* nozzle is too low */
      {
         AuxDevState = (AuxDevState & (~hydLevlDwn)) | hydLevlUp;               /* drive pnuematics upward up_MAX */
      }
      else
      {
         AuxDevState = (AuxDevState & (~hydLevlDwn)) & (~hydLevlUp);            /* hold position */
      }
   }
   else
   {
     AuxDevState = (AuxDevState & (~hydLevlDwn)) & (~hydLevlUp);                /* error with sensor or feature turned off by user then hold position */
   }
}
#endif /* pneumatic height control active */
#endif /* end chengdu jrt distance sensor */

#if defined(UBIBOT_DEVICE_USED)
/*******************************************************************************
* Function Name: priNt_mY_uBiConfig
********************************************************************************
* Summary:
*  Prints uBi Bot configuration structure as a JSON object in buf
*
* Parameters:
*  const char * buf, uBiMetaData_t *p
*
* Return:
*  0 on error
*
*******************************************************************************/
static int16_t priNt_mY_uBiConfig(char * buf, const uBiMetaData_t *p)
{
   struct json_out out;
   int16_t lenIn;                                                               // length in
   int16_t lenWrit;                                                             // length written in configuration string

   if ((p == NULL) || (buf == NULL))
     return -1;
     
   strcpy((unsigned char*)buf,"{");                                             // we think there is an extra bracket here
   lenIn=json_printer_buf(&out, buf, sizeof(buf));                              // appends the message in buf to the output structure

   lenWrit=json_printf(&out, "{fn_th: %d, fn_light: %d, fn_mag: %ul, fn_mag_int: %d, fn_acc_tap1: %d, fn_acc_tap2: %d, fn_acc_act: %d, fn_acc_min: %d, fn_bt: %d, fn_ext_t: %ul, fn_battery: %d, fn_dp: %d, cg_data_led: %s }", p->fn_th, p->fn_light, p->fn_mag, p->fn_mag_int, p->fn_acc_tap1, p->fn_acc_tap2, p->fn_acc_act, p->fn_acc_min, p->fn_bt, p->fn_ext, p->fn_battery, p->fn_dp, p->cg_data_led );
   if (sizeof(buf)>out.u.buf.len)
     memcpy((void*) buf+lenIn,(void*) &out.u.buf.buf, out.u.buf.len);           // copies the new json string into the buffer
   else
      return (0u);
   if (sizeof(buf)>out.u.buf.len)
     strcpy((unsigned char*) buf+lenIn+out.u.buf.len+1u,"}");                   // now the close bracket
   else
      return (0u);
   lenWrit=json_printf(&out, "{ command: SetMetaData, metadata: %M }",buf);     // now add the start of the message to the json out
   if (sizeof(buf)>out.u.buf.len)
      memcpy((void*) buf,(void*) &out.u.buf.buf, out.u.buf.len);                // copy back to the buffer on the top level to send via comms
}
#endif

#ifdef SEQ_CAM_USED
/*******************************************************************************
* Function Name: sendJSONConfig2Sequoia
********************************************************************************
* Summary:
*  Create a config JSON string for the sequioa camera
*
* Parameters:
*  SEQStat_Config_t *confStruct
*
* Return:
*  0 on error
*
*******************************************************************************/
int16_t sendJSONConfig2Sequoia( SEQStat_Config_t *confStruct )
{
    struct json_out out;
    const char *buf = NULL;
    int16_t retCode=0U;
    
    if ( confStruct == NULL )
       return retCode;
       
    // const char *result;                                                      use with strcmp to test result
    // out = JSON_OUT_BUF(buf, sizeof(buf));
    retCode=json_printer_buf(&out, buf, sizeof(buf));
    // result = "{\"foo\": 123, \"x\": [false, true], \"y\": \"hi\"}";          you can put a test string here for testing
    // json_printf(&out, "{%Q: %d, x: [%B, %B], y: %Q}", "foo", 123, 0, -1, "hi");  example as above
    json_printf(&out, "{ %Q: %Q, %Q: %f, %Q: %f, %Q: %f, %Q: %d, %Q: %f, %Q: %d, %Q: %d, %Q: %Q, %Q: %Q }",seq_config_values[0U], confStruct->capture_mode, seq_config_values[1U],
    confStruct->timelapse_param, seq_config_values[2U], confStruct->gps_param, seq_config_values[3U], confStruct->overlap_param,
    seq_config_values[4U], confStruct->resolution_rgb, seq_config_values[5U], confStruct->resolution_mono, seq_config_values[6U], confStruct->bit_depth,
    seq_config_values[6U], confStruct->sensors_mask, seq_config_values[7U], confStruct->storage_selected, seq_config_values[8U], confStruct->auto_select);
    return retCode;
}
/*******************************************************************************
* Function Name: getJSONConfigFrmSequoia
********************************************************************************
* Summary:
*  Get the config JSON string from the sequioa camera
*
* Parameters:
*  SEQStat_Config_t *confStruct, const char *str
*
* Return:
*  -1 on error  otherwise number of keys retrieved
*
*******************************************************************************/
int16_t getJSONConfigFrmSequoia( SEQStat_Config_t *confStruct, const char *str )
{
  const char *request = NULL;
  int16_t retCode=0;

  if ((confStruct == NULL ) || ( str == NULL))
    return -1;
    
  retCode = json_scanf(str, strlen((char*)str),"{request:%Q, capture_mode:%Q, timelapse_param:%f, gps_param:%f, overlap_param:%f, resolution_rgb:%d, resolution_mono:%f, bit_depth:%d, sensors_mask:%d, storage_selected:%Q, auto_select: %Q }",request,
  &confStruct->capture_mode,&confStruct->timelapse_param, &confStruct->gps_param, &confStruct->overlap_param, &confStruct->resolution_rgb, &confStruct->resolution_mono,  &confStruct->bit_depth, 
  &confStruct->sensors_mask, confStruct->storage_selected, confStruct->auto_select   );
  if (strcmp((char*)request,"get_config")==0)
    return retCode;
  else
    return -1;
}
/*******************************************************************************
* Function Name: chkJSONreplyFrmSequioa
********************************************************************************
* Summary:
*  Check that the response string in the AJAX reply matches that with the request sent
*
* Parameters:
*  SEQStat_Config_t *confStruct, const char *cmd
*
* Return:
*  -1 no request in the AJAX response
*   0 not the request you specified with cmd variable
*   1 the resposne matches the cmd variable (reply to you're message)
*
*******************************************************************************/
int16_t chkJSONreplyFrmSequioa( const char *str, const char *cmd )
{
  const char *request = NULL;
  
  if ((str == NULL ) || (cmd == NULL))
    return -1;
    
  if (json_scanf(str, strlen((char*)str), "{ request:%Q, }", request)==1U)
  {
     if (strcmp((char*)request,(char*)cmd)==0)
        return 1;                                                              // return correct response found
     else
        return 0;                                                              // not correct request
  }
  return -1;                                                                    // no request string found
}
#endif                                                                          // ============ eNd Parrot Sequioa Functions ================================
#if defined(SBGC_GIMBAL_HMI)
/*******************************************************************************
* Function Name: createSBGCScript
********************************************************************************
* Summary:
*  createSBGCScript function performs following functions:
*   1. creates a script to be sent to the Gimbal
*   2. Script is Example 2 of the manual
*   3. Pan at 5 deg/sec by 90 degrees. AUX1 operates recording
* Parameters:
*  SBGC_cmd_write_file_t *writeFile - write structure to send in message
*
* Return:
*  None.
*
*******************************************************************************/
void createSBGCScript( SBGC_cmd_write_file_t *writeFile )
{
  unsigned char line1[] = "R";                                                  // Reset the origin of the YAW: Start recording with the current azimuth
  unsigned char line2[] = "A RA(0) PA(30) YA(0)";                               // Tilt the camera 30 degrees down and level it
  unsigned char line3[] = "T 16(1)";                                            // Start recording
  unsigned char line4[] = "D TIMEOUT(3)";                                       // Writing freeze for 3 seconds.
  unsigned char line5[] = "CONFIG ACC_LIMIT_Y(5)";                              // Setup low acceleration for smooth start and stop of the motion
  unsigned char line6[] = "S YS(5)";                                            // Panning with a speed of 5 degree/sec. clockwise
  unsigned char line7[] = "W YA(90)";                                           // Wait until turned by 90 degrees
  unsigned char line8[] = "S YS(0)";                                            // Stop panning (de-acceleration starts here)
  unsigned char line9[] = "W YS(0)";                                            // Wait until de-acceleration is finished
  unsigned char line10[] = "D TIMEOUT(3)";                                      // Writing freeze for 3 seconds
  unsigned char line11[] = "T 16(0)";                                           // Stop recording
  uint8_t messageLen;

  if (writeFile == NULL)
    return;
    
  writeFile->FILE_ID = SBGC_FILE_TYPE_SCRIPT;                                   // File type is script
  writeFile->PAGE_OFFSET = 0U;
  sprintf(writeFile->DATA,"%s\n",line1);                                        // write line 1 of the script
  messageLen =+ sizeof(line1);
  sprintf(writeFile->DATA+messageLen,"%s\n",line2);                             // write line of the script
  messageLen =+ sizeof(line2);
  sprintf(writeFile->DATA+messageLen,"%s\n",line3);                             // write line of the script
  messageLen =+ sizeof(line3);
  sprintf(writeFile->DATA+messageLen,"%s\n",line4);                             // write line of the script
  messageLen =+ sizeof(line4);
  sprintf(writeFile->DATA+messageLen,"%s\n",line5);                             // write line of the script
  messageLen =+ sizeof(line5);
  sprintf(writeFile->DATA+messageLen,"%s\n",line6);                             // write line of the script
  messageLen =+ sizeof(line6);
  sprintf(writeFile->DATA+messageLen,"%s\n",line7);                             // write line of the script
  messageLen =+ sizeof(line7);
  sprintf(writeFile->DATA+messageLen,"%s\n",line8);                             // write line of the script
  messageLen =+ sizeof(line8);
  sprintf(writeFile->DATA+messageLen,"%s\n",line9);                             // write line of the script
  messageLen =+ sizeof(line9);
  sprintf(writeFile->DATA+messageLen,"%s\n",line10);                            // write line of the script
  messageLen =+ sizeof(line10);
  sprintf(writeFile->DATA+messageLen,"%s\n",line11);                            // write line of the script
  messageLen =+ sizeof(line11);
  writeFile->FILE_SIZE = sizeof(writeFile->DATA);                               // size of the file
}

/*******************************************************************************
* Function Name: createSBGCTimelapse
********************************************************************************
* Summary:
*  createSBGCTimelapse function performs following functions:
*   1. creates a script to be sent to the Gimbal
*   2. pan left with speed 0.1 degrees/sec where 0.1 is changable
*   3. Ptilt up with the speed half slower which is also changable
*   4. Allow axis inversion
* Parameters:
*  SBGC_cmd_write_file_t *writeFile - write structure to send in message
*  COMGS_Script_t *configParams - values to adjust from the GUI
* Return:
*  None.
*
*******************************************************************************/
void createSBGCTimelapse( SBGC_cmd_write_file_t *writeFile, COMGS_Script_t *configParams)
{
  unsigned char line1[] = "SET_ADJ_VAR NAME(FRAME_HEADING_ANGLE) VALUE(0)";     //  Let system to know that the frame is still, to compensate a drift of gyroscope
  unsigned char line2[] = "SET_ADJ_VAR NAME(GYRO_TRUST) VALUE(60)";             // Set the 'gyro trust' parameter low enough to better compensate drift of gyroscope
  unsigned char line3[] = "";                                                   // (Optional) move camera to the desired initial position. Skip this command to start from the current position
  unsigned char line4[] = "";

  uint8_t messageLen=0U;

  if ((writeFile == NULL) || (configParams == NULL))
     return;
     
  writeFile->FILE_ID = SBGC_FILE_TYPE_SCRIPT;                                   // File type is script
  writeFile->PAGE_OFFSET = 0U;
  sprintf(writeFile->DATA,"%s\n",line1);                                        // write line 1 of the script
  messageLen = sizeof(line1);
  sprintf(writeFile->DATA+messageLen,"%s\n",line2);                             // write line of the script
  messageLen =+ sizeof(line2);
  if (configParams->angleReq == true)                                           // if requested
  {
     if (configParams->swapRollYaw==true)
     {
        sprintf(line3,"ANGLE PA(%d) YA(%d)\n",configParams->PA,configParams->YA);
     }
     else if (configParams->swapPitchRoll==true)
     {
        sprintf(line3,"ANGLE RA(%d) YA(%d)\n",configParams->RA,configParams->YA);
     }
     else
     {
       sprintf(line3,"ANGLE PA(%d) RA(%d)\n",configParams->PA,configParams->RA);    // (Optional) move camera to the desired initial position. Skip this command to start from the current position
     }
     messageLen =+ sizeof(line3);
     sprintf(writeFile->DATA+messageLen,"%s",line3);
  }
  if (configParams->useDefault == true)
  {
     configParams->YS = 0.1f;                                                   // pan left with speed 0.1 degrees/sec
     configParams->PS = -0.05f;                                                 //tilt up with the speed half slower
  }
  if (configParams->swapPitchRoll==true)
  {
     sprintf(line4,"SPEED YS(%d) RS(%d)\n",configParams->YS,configParams->RS);
  }
  else if (configParams->swapRollYaw==true)
  {
     sprintf(line4,"SPEED RS(%d) PS(%d)\n",configParams->RS,configParams->PS);
  }
  else
  {
     sprintf(line4,"SPEED YS(%d) PS(%d)\n",configParams->YS,configParams->PS);       //Pan (right or left) with the speed given and .tilt (up or down) with the speed given
  }
  messageLen =+ sizeof(line4);
  sprintf(writeFile->DATA+messageLen,"%s",line4);

  writeFile->FILE_SIZE = messageLen;                                            // size of the file
}
#endif                                                                          // ============== eNd SBGC GUI Functions =============================
#if (CAMERA_TYPE == XS_CAM)
/*******************************************************************************
* Function Name: XSChangeDetected
********************************************************************************
* Summary:
*  XSChangeDetected function performs following functions:
*   1. looks for a change in request value for a setting in the XS encoder
* Parameters:
*  const XS_Hmi_Slider_t *ptr_HMIobject
* Return:
*  1 for change 0 for no-change 
*  (value should cause entry into poll table for that parameter, to change encoder)
*
*******************************************************************************/
uint8_t XSChangeDetected( const XS_Hmi_Slider_t *ptr_HMIobject )                // Function to detect a data change by more than X
{
   if (ptr_HMIobject == NULL)
     return 0u;
   return (uint8_t) (abs(ptr_HMIobject->value - ptr_HMIobject->last)>ptr_HMIobject->delta); // 1 for change 0 for no-change
}
#endif  

#if (defined(SBGC_GIMBAL_JOY) || defined(SBGC_GIMBAL_HMI))
/*******************************************************************************
* Function Name: ChgStat_CONFIG_for_CONTROL
********************************************************************************
* Summary:
*  ChgStat_CONFIG_for_CONTROL A Function to turn on / off the CMD_CONTROL CONFIRM
*   1. turns on or off (toggle) of the confirm messages sent back for CMD_CONTROL
* Parameters:
*  SBGC_cmd_control_config_t *fdata, uint8_t on_off, uint16_t sz_fdata
* Return:
*  uint8_t
*
*******************************************************************************/
uint8_t ChgStat_CONFIG_for_CONTROL(SBGC_cmd_control_config_t *fdata, uint8_t on_off, uint16_t sz_fdata)   // A Function to turn on / off the CMD_CONTROL CONFIRM
{

  //uint8_t returnState;                                                        // Message state returned to the calling level
  uint8_t noSent;                                                               // number of sends counter
  uint8_t loop=0;
  
  if (fdata == NULL)
    return 0u;
    
  // We use a pointer to the CONTROL_CMD structure to allow multiple control command structures to define the other settings at the top level
  //
  if (on_off=true)                                                              // true=no confirm false=confirm
  {
     fdata->FLAGS = fdata->FLAGS & !SBGC_CONTROL_CONFIG_FLAG_NO_CONFIRM;        // Remove the Flag to stop CONFIRM message being sent for CMD_CONTROL  (send confirm)
  }
  else
  {
     fdata->FLAGS = fdata->FLAGS | SBGC_CONTROL_CONFIG_FLAG_NO_CONFIRM;         // Set the Flag to stop CONFIRM message being sent for CMD_CONTROL
  }

  head.commandID = SBGC_CMD_CONTROL_CONFIG;                                     // Issue a request to configure the CMD_CONTROL
  head.sizeData=sz_fdata;                                                       // Byte 3 of the header
  head.cmdSize=(head.commandID + head.sizeData) % 256U;                         // Byte 4 of the header

  memcpy(Buffer, &head, sizeof(head));                                          // Copy the header for the music message
  memcpy(Buffer+sizeof(head), fdata, head.sizeData);                            // Copy the payload
  if ((boardinforb.FIRMWARE_VER >= 2680U) && (g_boardReady))                       // If we have the new firmware loaded
  {
     head.stxCHAR=SBGC_CMD_START_BYTE_V2;                                       // $ char to start message when 2.68b0 or greater
     newCRC = crc16_arc_calculate(sz_fdata, ptr_new_cc);                        // Calculate new 16 bit CRC for new version
     memcpy(Buffer+sz_fdata+sizeof(head),&newCRC,sizeof(newCRC));               // Tag the CRC
     totalMsgLen = sz_fdata+sizeof(head)+sizeof(newCRC);                        // message length
  }
  else
  {
     head.stxCHAR=SBGC_CMD_START_BYTE;                                          // > char to start message
     crc = checksum(ptr_cc, sz_fdata);                                          // caculate the payload checksum
     memcpy(Buffer+sz_fdata+sizeof(head),&crc,sizeof(crc));                     // Tag the CRC
     totalMsgLen = sz_fdata+sizeof(head)+sizeof(crc);                           // message length
  }
  memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                        // Null terminate the Buffer for serial
  for (loop=0u;loop<totalMsgLen;loop++)
  {
    UART2_write(Buffer[loop]);
  }
  noSent = 0u;
  while ((udpReturn == UDP_SEND) && (noSent <= MAX_NUM_OF_UDP_SENDS))
  {
     udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
     noSent =++noSent % UINT8_MAX;
  }
  memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                        // Null terminate the Buffer for serial
  for (loop=0u;loop<totalMsgLen;loop++)
  {
    UART2_write(Buffer[loop]);
  }
  if (udpReturn != UDP_SEND)                                                    // Return the Status of the UDP datagram transmission
  {
     return (CMD_SENT);                                                         // Did complete request now wait for confirm if you turned it on.
  }
  else
  {
     return (UDP_SEND);                                                         // Didnt complete request
  }

}
#endif                                                                          // ==================== SBGC Gimbal control functions =========================
#ifdef YI_CAM_USED
/*******************************************************************************
* Function Name: XY_Check_Return_Code
********************************************************************************
* Summary:
*  XY_Check_Return_Code function performs following functions:
*   1. checks the reply from the JSON server to match the message type
*
* Parameters:
*  uint8_t msgTypeIn - command id as per camera.h
* Return:
*  The next step or -1 if the message was not the expected AJAX reply
*
*******************************************************************************/
int8_t XY_Check_Return_Code( uint8_t msgTypeIn )
{
   int8_t returnCode=-1;                                                        // returns error unless token response found
   
   if ((XY_ParseStruct.f_rval) && (XY_ParseStruct.f_msg_id))                    // response returned a rval and msg_id
   {
      if (XYjsonReply.msg_id == msgTypeIn)                                      // expected message reply
      {
         switch(XYjsonReply.rval)                                               // Check the return value
         {
            case XY_RESP_OK:                                                    // No Error
            if (XYjsonReply.msg_id == XY_SND_TK_PHO_CMD)                        // If we are taking a photo go to the next step of looking for a event start_photo_capture
            {
               g_YiCamReqState = 100u;                                          // Now look for the next TCP message to be received in this sequence
            }
            else if (XYjsonReply.msg_id == XY_SND_GET_SPC_CMD)                  // a call for SD Card space
            {
              removeSubstr((char*) &XYjsonReply.param_str, "\"");               // remove the quotes
              if ((XY_ParseStruct.f_param) && (XYRequest==52u))
              {
                 hmiYiValues.free_space = atol(XYjsonReply.param_str);          // put the free space on the SD card in bytes into the display register
              }
              else if ((XY_ParseStruct.f_param) && (XYRequest==53u))
              {
                 hmiYiValues.total_space = atol(XYjsonReply.param_str);         // put the total space on the SD card in bytes into the display register
              }
            }

            if (XYjsonReply.msg_id == XY_GET_SET_CMD)                           // a full configuration request
            {
              // copy to XY_config_t from XY_reply_t XYJsonReply.param_str      ===== REQUIRES SECOND PARSE ======
              RemoveCharFromString( (char*) &XYjsonReply.param_str, ':');
              //SubCharFromString( (unsigned char*) &XYJsonReply.param_str, ':', ',');
              RemoveCharFromString( (char*) &XYjsonReply.param_str, '{');
              RemoveCharFromString( (char*) &XYjsonReply.param_str, '}');
              sprintf(XYstringStore,"{\"groups\" : %s }\r",XYjsonReply.param_str);       // make the group string
              XY_ParseState=xy_parse_reply( XYstringStore, &XYjsonReply,XY_READ_ALL,&XYConfig);  // populate XYConfig_t
            }
            else
            {
              XYRequest=++XYRequest % UINT8_MAX;                                // Go to the next request
            }
            g_YiCamReqState = 2u;                                               // Now look for the next TCP message to be sent (stream is started)
            break;

            case XY_RESP_REQ_NEW_TOKEN:                                         // request a new token (input object is empty)
            g_YiCamReqState = 0u;                                               // start again to open the stream
            break;

            case XY_RESP_CRASH:                                                 // camera crash
            if (XYjsonReply.msg_id == XY_GET_SET_CMD)                           // afull configuration request
            {
               g_YiCamReqState = 10u;                                           // try again
            }
            else
            {
               g_YiCamReqState = 2u;                                            // request same send message again
            }
            break;

            case XY_RESP_INVALID:                                               // invalid response
            g_YiCamReqState = 2u;                                               // try again
            break;

            case XY_INVALID_JSON_OBJ:                                           // invalid JSON object
            g_YiCamReqState = 0u;                                               // start again to open the stream
            break;

            case XY_INVALID_CMD:                                                // invalid command
            if (XYjsonReply.msg_id == XY_GET_SET_CMD)                           // a full configuration request
            {
               g_YiCamReqState = 2u;                                            // Now look for the next TCP message to be sent (stream is started)
            }
            else
            {
               XYRequest=++XYRequest % UINT8_MAX;                               // Go to the next request
               g_YiCamReqState = 2u;                                            // Now look for the next TCP message to be sent (stream is started)
            }
            break;

            default:                                                            // return code not found defined ?
            if (XYjsonReply.msg_id == XY_GET_SET_CMD)                           // afull configuration request
            {
               g_YiCamReqState = 10u;                                           // try again
            }
            else
            {
               XYRequest=++XYRequest % UINT8_MAX;                               // Go to the next request
               g_YiCamReqState = 2u;                                            // Now look for the next TCP message to be sent (stream is started)
            }
            break;
        }
        xyCamStateTimer=0u;                                                     // reset the step timeout as we exited with a response
        return g_YiCamReqState;                                                 // return the requested state
     }
  }
  return returnCode;                                                            // return that the intended message was not found
}
#endif                                                                          // ================= eNd Yi Action Cam Functions ===========================
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
/*******************************************************************************
* Function Name: SBGC_Process_Data_Read
********************************************************************************
* Summary:
*  Process the SGBC Read data from either UDP or SERIAL
*  (Controlled from the main program below)
* Parameters:
*  uint8_t msgTypeIn - type of message
* Return:
*  nothing
*
*******************************************************************************/
void SBGC_Process_Data_Read( uint8_t msgTypeIn )
{
      switch(msgTypeIn)                                                         // For the state of the switches from the Commander GS
      {
        case SBGC_CMD_GET_ANGLES:                                               // command was a SBGC get Angles
          // commangles.imu_angle_roll = SBGC_EULER_TO_DEGREE(SBGC_ANGLE_TO_DEGREE_INT(getangles.imu_angle_roll));               Convert the angles read from the SBGC message back to degree
          commangles.imu_angle_roll = SBGC_ANGLE_TO_DEGREE_INT(getangles.imu_angle_roll);              // Convert the angles read from the SBGC message back to degree
          commangles.imu_angle_pitch = SBGC_ANGLE_TO_DEGREE_INT(getangles.imu_angle_pitch);            // Convert the angles read from the SBGC message back to euler
          commangles.imu_angle_yaw = SBGC_ANGLE_TO_DEGREE_INT(getangles.imu_angle_yaw);                // Convert the angles read from the SBGC message back to euler
          commangles.target_angle_roll = SBGC_ANGLE_TO_DEGREE_INT(getangles.target_angle_roll);        // Convert the angles read from the SBGC message back to euler
          commangles.target_angle_pitch = SBGC_ANGLE_TO_DEGREE_INT(getangles.target_angle_pitch);      // Convert the angles read from the SBGC message back to euler
          commangles.target_angle_yaw = SBGC_ANGLE_TO_DEGREE_INT(getangles.target_angle_yaw);          // Convert the angles read from the SBGC message back to euler
          commangles.target_speed_roll = (float32_t) getangles.target_speed_roll / SBGC_SPEED_SCALE;   // Convert the speed from the message read
          commangles.target_speed_pitch = (float32_t) getangles.target_speed_pitch / SBGC_SPEED_SCALE; // Convert the speed from the message read
          commangles.target_speed_yaw = (float32_t) getangles.target_speed_yaw / SBGC_SPEED_SCALE;     // Convert the speed from the message read
          g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_GET_ANGLES;                  // set the flag to say we processed it
          break;
        case SBGC_CMD_AUTO_PID:                                                 // command was a SBGC auto pid response
          g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_AUTO_PID;                  // set the flag to say we processed it
          break;
        case SBGC_CMD_GET_ANGLES_EXT:                                           // Command get angles ext
          commangles.imu_angle_roll = SBGC_ANGLE_TO_DEGREE_INT(getanglesext.imu_angle_roll);              // Convert the angles read from the SBGC message to euler
          commangles.imu_angle_pitch = SBGC_ANGLE_TO_DEGREE_INT(getanglesext.imu_angle_pitch);            // Convert the angles read from the SBGC message back to euler
          commangles.imu_angle_yaw = SBGC_ANGLE_TO_DEGREE_INT(getanglesext.imu_angle_yaw);                // Convert the angles read from the SBGC message back to euler
          commangles.target_angle_roll = SBGC_ANGLE_TO_DEGREE_INT(getanglesext.target_angle_roll);        // Convert the angles read from the SBGC message back to euler
          commangles.target_angle_pitch = SBGC_ANGLE_TO_DEGREE_INT(getanglesext.target_angle_pitch);      // Convert the angles read from the SBGC message back to euler
          commangles.target_angle_yaw = SBGC_ANGLE_TO_DEGREE_INT(getanglesext.target_angle_yaw);          // Convert the angles read from the SBGC message back to euler
          commangles.stator_rotor_ang_yaw = SBGC_ANGLE_TO_DEGREE_INT(getanglesext.stator_rotor_angle_yaw);
          commangles.stator_rotor_ang_roll = SBGC_ANGLE_TO_DEGREE_INT(getanglesext.stator_rotor_angle_roll);
          commangles.stator_rotor_ang_pitch = SBGC_ANGLE_TO_DEGREE_INT(getanglesext.stator_rotor_angle_pitch);
          g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_GET_ANGLES_EXT;                  // set the flag to say we processed it
          break;
        case SBGC_CMD_REALTIME_DATA_3:                                          // command was a SBGC realtime data.
          commgyroacc.gyro_roll = SBGC_GYRO_TO_DEGREE(realdata3.gyro_data_roll);
          commgyroacc.gyro_pitch = SBGC_GYRO_TO_DEGREE(realdata3.gyro_data_pitch);
          commgyroacc.gyro_yaw = SBGC_GYRO_TO_DEGREE(realdata3.gyro_data_yaw);
          commgyroacc.acc_roll = SBGC_ACC_DATA(realdata3.acc_data_roll);
          commgyroacc.acc_pitch = SBGC_ACC_DATA(realdata3.acc_data_pitch);
          commgyroacc.acc_yaw = SBGC_ACC_DATA(realdata3.acc_data_yaw);
          moving_average( &commgyroacc );                                       // Compute the moving averages
          commgsrealdata3.IMU_ANGLE[0u] = SBGC_ANGLE_TO_DEGREE_INT(realdata3.imu_angle_roll);
          commgsrealdata3.IMU_ANGLE[1u] = SBGC_ANGLE_TO_DEGREE_INT(realdata3.imu_angle_pitch);
          commgsrealdata3.IMU_ANGLE[2u] = SBGC_ANGLE_TO_DEGREE_INT(realdata3.imu_angle_yaw);
          commgsrealdata3.FRAME_IMU_ANGLE[0u] = SBGC_ANGLE_TO_DEGREE_INT(realdata3.frame_imu_angle_roll);
          commgsrealdata3.FRAME_IMU_ANGLE[1u] = SBGC_ANGLE_TO_DEGREE_INT(realdata3.frame_imu_angle_pitch);
          commgsrealdata3.FRAME_IMU_ANGLE[2u] = SBGC_ANGLE_TO_DEGREE_INT(realdata3.frame_imu_angle_yaw);
          commgsrealdata3.TARGET_ANGLE[0u] = SBGC_ANGLE_TO_DEGREE_INT(realdata3.target_angle_roll);
          commgsrealdata3.TARGET_ANGLE[1u] = SBGC_ANGLE_TO_DEGREE_INT(realdata3.target_angle_pitch);
          commgsrealdata3.TARGET_ANGLE[2u] = SBGC_ANGLE_TO_DEGREE_INT(realdata3.target_angle_yaw);
          commgsrealdata3.BAT_LEVEL = SBGC_BAT_VOLTS(realdata3.bat_level);
          commgsrealdata3.CUR_IMU = realdata3.cur_imu;
          commgsrealdata3.CUR_PROFILE = realdata3.cur_profile;
          SBGCSystemError1.AlarmWdCurrent = (uint8_t) (realdata3.system_error & 0x0FU);
          SBGCSystemError2.AlarmWdCurrent = (uint8_t) ((realdata3.system_error>>8) & 0x0FU);
          switch(realdata3.system_sub_error)
          {
               case SBGC_SUB_ERR_I2C_ERRORS:
                  SBGCESDReason[0u].AlarmWdCurrent = SBGCESDReason[0u].AlarmWdCurrent | 1U;
                  break;
               case SBGC_SUB_ERR_DRV_OTW:
                  SBGCESDReason[0u].AlarmWdCurrent = SBGCESDReason[0u].AlarmWdCurrent | 2U;
                  break;
               case SBGC_SUB_ERR_DRV_FAULT:
                  SBGCESDReason[0u].AlarmWdCurrent = SBGCESDReason[0u].AlarmWdCurrent | 4U;
                  break;
              case SBGC_SUB_ERR_ENCODER_IMU_ANGLE:
                  SBGCESDReason[0u].AlarmWdCurrent = SBGCESDReason[0u].AlarmWdCurrent | 8U;
                  break;
              case SBGC_SUB_ERR_CALIBRATION_FAILED:
                  SBGCESDReason[0u].AlarmWdCurrent = SBGCESDReason[0u].AlarmWdCurrent | 16U;
                  break;
              case SBGC_SUB_ERR_INTERNAL_SYSTEM_ERROR:
                  SBGCESDReason[0u].AlarmWdCurrent = SBGCESDReason[0u].AlarmWdCurrent | 32U;
                  break;
              case SBGC_SUB_ERR_ENCODER_CALIB_BAD_SCALE:
                  SBGCESDReason[0u].AlarmWdCurrent = SBGCESDReason[0u].AlarmWdCurrent | 64U;
                  break;
              case SBGC_SUB_ERR_OVER_TEMPERATURE:
                  SBGCESDReason[0u].AlarmWdCurrent = SBGCESDReason[0u].AlarmWdCurrent | 128U;
                  break;
              case SBGC_SUB_ERR_BAD_MOTOR_POLES_INVERT:
                  SBGCESDReason[1u].AlarmWdCurrent = SBGCESDReason[1u].AlarmWdCurrent | 1U;
                  break;
              case SBGC_SUB_ERR_NOT_ENOUGH_MEMORY:
                  SBGCESDReason[1u].AlarmWdCurrent = SBGCESDReason[1u].AlarmWdCurrent | 2U;
                  break;
               case SBGC_SUB_ERR_IMU_SENSOR_NOT_RESPONDING:
                  SBGCESDReason[1u].AlarmWdCurrent = SBGCESDReason[1u].AlarmWdCurrent | 4U;
                  break;
              case SBGC_SUB_ERR_CAN_HARD:
                  SBGCESDReason[1u].AlarmWdCurrent = SBGCESDReason[1u].AlarmWdCurrent | 8U;
                  break;
              case SBGC_SUB_ERR_MOTOR_OVERHEAT_PROTECTION:
                  SBGCESDReason[1u].AlarmWdCurrent = SBGCESDReason[1u].AlarmWdCurrent | 16U;
                  break;
              case SBGC_SUB_ERR_MOTOR_IS_LOCKED:
                  SBGCESDReason[1u].AlarmWdCurrent = SBGCESDReason[1u].AlarmWdCurrent | 32U;
                  break;
              case SBGC_SUB_ERR_BAD_IMU_HEALTH:
                  SBGCESDReason[1u].AlarmWdCurrent = SBGCESDReason[1u].AlarmWdCurrent | 64U;
                  break;
              case SBGC_SUB_ERR_INFINITE_RESET:
                  SBGCESDReason[1u].AlarmWdCurrent = SBGCESDReason[1u].AlarmWdCurrent | 128U;
                  break;
              case SBGC_SUB_ERR_WRONG_INITIAL_POSITION:
                  SBGCESDReason[2u].AlarmWdCurrent = SBGCESDReason[2u].AlarmWdCurrent | 1U;
                  break;
              case SBGC_SUB_ERR_MOTOR_LOAD_TIME_EXCEEDED:
                  SBGCESDReason[2u].AlarmWdCurrent = SBGCESDReason[2u].AlarmWdCurrent | 2U;
                  break;
               case SBGC_SUB_ERR_CAN_DRV_OVERCURRENT:
                  SBGCESDReason[2u].AlarmWdCurrent = SBGCESDReason[2u].AlarmWdCurrent | 4U;
                  break;
              case SBGC_SUB_ERR_CAN_DRV_UNDERVOLTAGE:
                  SBGCESDReason[2u].AlarmWdCurrent = SBGCESDReason[2u].AlarmWdCurrent | 8U;
                  break;
              case SBGC_SUB_ERR_CAN_DRV_EMERGENCY_PIN:
                  SBGCESDReason[2u].AlarmWdCurrent = SBGCESDReason[2u].AlarmWdCurrent | 16U;
                  break;
              case SBGC_SUB_ERR_CAN_DRV_FOC_DURATION:
                  SBGCESDReason[2u].AlarmWdCurrent = SBGCESDReason[2u].AlarmWdCurrent | 32U;
                  break;
              case SBGC_SUB_ERR_CAN_DRV_MCU_OVERHEAT:
                  SBGCESDReason[2u].AlarmWdCurrent = SBGCESDReason[2u].AlarmWdCurrent | 64U;
                  break;
              case SBGC_SUB_ERR_CAN_DRV_MOTOR_OVERHEAT:
                  SBGCESDReason[2u].AlarmWdCurrent = SBGCESDReason[2u].AlarmWdCurrent | 128U;
                  break;
              case SBGC_SUB_ERR_CAN_DRV_OVERCURRENT_SOFT:
                  SBGCESDReason[3u].AlarmWdCurrent = SBGCESDReason[3u].AlarmWdCurrent | 1U;
                  break;
              case SBGC_SUB_ERR_CAN_DRV_SEVERAL:
                  SBGCESDReason[3u].AlarmWdCurrent = SBGCESDReason[3u].AlarmWdCurrent | 2U;
                  break;
               case SBGC_SUB_ERR_CAN_EXT_BUS_OFF:
                  SBGCESDReason[3u].AlarmWdCurrent = SBGCESDReason[3u].AlarmWdCurrent | 4U;
                  break;
              case SBGC_SUB_ERR_CAN_INT_BUS_OFF:
                  SBGCESDReason[3u].AlarmWdCurrent = SBGCESDReason[3u].AlarmWdCurrent | 8U;
                  break;
              case SBGC_SUB_ERR_ENCODER_NOT_FOUND:
                  SBGCESDReason[3u].AlarmWdCurrent = SBGCESDReason[3u].AlarmWdCurrent | 16U;
                  break;
              case SBGC_SUB_ERR_CAN_DRV_NOT_RESPONDING:
                  SBGCESDReason[3u].AlarmWdCurrent = SBGCESDReason[3u].AlarmWdCurrent | 32U;
                  break;
              case SBGC_SUB_ERR_CAN_DRV_WRONG_PARAMS:
                  SBGCESDReason[3u].AlarmWdCurrent = SBGCESDReason[3u].AlarmWdCurrent | 64U;
                  break;
              case SBGC_SUB_ERR_OVERCURRENT:
                  SBGCESDReason[3u].AlarmWdCurrent = SBGCESDReason[3u].AlarmWdCurrent | 128U;
                  break;
              case SBGC_SUB_ERR_UNSAFE_VOLTAGE:
                  SBGCESDReason[4u].AlarmWdCurrent = SBGCESDReason[4u].AlarmWdCurrent | 1U;
                  break;
              case SBGC_SUB_ERR_WRONG_FULL_BAT_VOLTAGE_PARAM:
                  SBGCESDReason[4u].AlarmWdCurrent = SBGCESDReason[4u].AlarmWdCurrent | 2U;
                  break;
               case SBGC_SUB_ERR_EEPROM_PARAMS_CORRUPTED:
                  SBGCESDReason[4u].AlarmWdCurrent = SBGCESDReason[4u].AlarmWdCurrent | 4U;
                  break;
                  
               default:
                  break;
          }
          g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_REALTIME_DATA_3;                  // set the flag to say we processed it
          break;
        case SBGC_CMD_REALTIME_DATA_4:
           commtemp.imu_temp = realdata4.imu_temp;                              // read the imu temp
           commtemp.frame_imu_temp = realdata4.frame_imu_temp;                  // read the frame imu temp
           commgsmagnet.mag_data_roll = realdata4.mag_data_roll;
           commgsmagnet.mag_data_pitch = realdata4.mag_data_pitch;
           commgsmagnet.mag_data_yaw = realdata4.mag_data_yaw;
           commgsmtrpwr.motor_power_roll = Scale_Raw_Value(-1000, 1000, 0.0f, 100.0f, realdata4.motor_out_roll);
           commgsmtrpwr.motor_power_pitch = Scale_Raw_Value(-1000, 1000, 0.0f, 100.0f, realdata4.motor_out_pitch);
           commgsmtrpwr.motor_power_yaw = Scale_Raw_Value(-1000, 1000, 0.0f, 100.0f, realdata4.motor_out_yaw);
           commgsmtrpwr.CURRENT = realdata4.current;
           g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_REALTIME_DATA_4;                  // set the flag to say we processed it
          break;
        case SBGC_CMD_CONFIRM:                                                  // command was a SBGC confirm set the relevant request to stop sending once confirmed
           switch(cmdconf.cmd_id)
           {
               case SBGC_CMD_CALIB_ACC:                                         // response to a ACC Calibration
                  if (commGSMenu == CMD_SENT)
                  {
                      commGSMenu = CMD_CONFIRMED;                               // We got the response so stop sending request
                  }
               break;
               case SBGC_CMD_CALIB_GYRO:                                        // reswponse to a Gyro Calibration
                  if (commGSMenu == CMD_SENT)
                  {
                      commGSMenu = CMD_CONFIRMED;                               // We got the response so stop sending request
                  }
               break;
               case SBGC_CMD_CALIB_MAG:                                         // response to a Mag Calibration
                  if (commGSMenu == CMD_SENT)
                  {
                      commGSMenu = CMD_CONFIRMED;                               // We got the response so stop sending request
                  }
               break;
               case SBGC_CMD_BOOT_MODE_3:                                       // response to command boot mode
               break;
               case SBGC_CMD_CALIB_BAT:                                         // response to battery calibration
               break;
               case SBGC_CMD_TRIGGER_PIN:                                       // response to trigger pin
               break;
               case SBGC_CMD_MOTORS_ON:                                         // resposne to a motors on command
                  if (commGSMotor == CMD_SENT)
                  {
                     commGSMotor = CMD_CONFIRMED;                               // We got the response so stop sending request
                  }
               break;
               case SBGC_CMD_SET_ADJ_VARS_VAL:                                  // response to set adjust variables request
                  if (commGSAdjvar == CMD_SENT)
                  {
                     commGSAdjvar = CMD_CONFIRMED;                              // We got the response so stop sending the request
                  }
               break;
               case SBGC_CMD_AUTO_PID:                                          // response to PID tune request v2.70<
                 if (commGSPID == CMD_SENT)                                     // only accept a confirm if we already sent
                 {
                    commGSPID = CMD_CONFIRMED;                                  // stop sending the request
                 }
               break;
               case SBGC_CMD_AUTO_PID2:                                          // response to PID tune request v2.70+
                 if (commGSPID == CMD_SENT)                                     // only accept a confirm if we already sent
                 {
                    commGSPID = CMD_CONFIRMED;                                  // stop sending the request
                 }
               break;
               case SBGC_CMD_I2C_WRITE_REG_BUF:                                 // response to the i2c write reg Buffer
               break;
               case SBGC_CMD_WRITE_EXTERNAL_DATA:                               // response to the write external data request
               break;
               case SBGC_CMD_WRITE_ADJ_VARS_CFG:                                // response to the write adjustable variables request
               break;
               case SBGC_CMD_WRITE_FILE:                                        // response to the write file command
                  commGSWriteFileState = cmdconf.reply;                         // check the reply
                  switch (commGSWriteFileState)
                  {
                    case SBGC_FILE_NO_ERROR:
                    if (commGSRunAScript = CMD_WSENT)
                    {
                       commGSRunAScript = CMD_WRITTEN;                          // Now advance to run the script
                    }
                    break;
                    case SBGC_FILE_ERR_EEPROM_FAULT:
                    if (commGSRunAScript = CMD_WSENT)
                    {
                       commGSRunAScript = CMD_WCOMPLETE;                        // Complete the command
                    }
                    ScriptRunAlarms1.AlarmWdCurrent |= 1U;
                    break;
                    case SBGC_ERR_FILE_NOT_FOUND:
                    if (commGSRunAScript = CMD_WSENT)
                    {
                       commGSRunAScript = CMD_WCOMPLETE;                        // Complete the command
                    }
                    ScriptRunAlarms1.AlarmWdCurrent |= 2U;
                    break;
                    case SBGC_ERR_FAT:
                    if (commGSRunAScript = CMD_WSENT)
                    {
                       commGSRunAScript = CMD_WCOMPLETE;                        // Complete the command
                    }
                    ScriptRunAlarms1.AlarmWdCurrent |= 4U;
                    break;
                    case SBGC_ERR_NO_FREE_SPACE:
                    if (commGSRunAScript = CMD_WSENT)
                    {
                       commGSRunAScript = CMD_WCOMPLETE;                        // Complete the command
                    }
                    ScriptRunAlarms1.AlarmWdCurrent |= 8U;
                    break;
                    case SBGC_ERR_FAT_IS_FULL:
                    if (commGSRunAScript = CMD_WSENT)
                    {
                       commGSRunAScript = CMD_WCOMPLETE;                        // Complete the command
                    }
                    ScriptRunAlarms1.AlarmWdCurrent |= 16U;
                    break;
                    case SBGC_ERR_FILE_SIZE:
                    if (commGSRunAScript = CMD_WSENT)
                    {
                       commGSRunAScript = CMD_WCOMPLETE;                        // Complete the command
                    }
                    ScriptRunAlarms1.AlarmWdCurrent |= 32U;
                    break;
                    case SBGC_FILE_ERR_CRC:
                    if (commGSRunAScript = CMD_WSENT)
                    {
                       commGSRunAScript = CMD_WSEND;                            // try to EEprom write the script again
                    }
                    ScriptRunAlarms1.AlarmWdCurrent |= 64U;
                    break;
                    case SBGC_FILE_ERR_LIMIT_REACHED:
                    if (commGSRunAScript = CMD_WSENT)
                    {
                       commGSRunAScript = CMD_WCOMPLETE;                        // Complete the command
                    }
                    ScriptRunAlarms1.AlarmWdCurrent |= 128U;
                    break;
                    case SBGC_ERR_FILE_CORRUPTED:
                    if (commGSRunAScript = CMD_WSENT)
                    {
                       commGSRunAScript = CMD_WSEND;                            // try to EEprom write the script again
                    }
                    ScriptRunAlarms2.AlarmWdCurrent |= 1U;
                    break;
                    case SBGC_FILE_ERR_WRONG_PARAMS:
                    if (commGSRunAScript = CMD_WSENT)
                    {
                       commGSRunAScript = CMD_WCOMPLETE;                        // Complete the command
                    }
                    ScriptRunAlarms2.AlarmWdCurrent |= 2U;
                    break;
                  }
                  g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_CONFIRM;                  // set the flag to say we processed it
                  break;
               case SBGC_CMD_CONTROL:                                           // response to the cmd_control command
#if defined(SBGC_GIMBAL_JOY)
                 if (!modeAlreadyRC)
                 {
                     ccReturn=ChgStat_CONFIG_for_CONTROL(&cc,false,sizeof(cc)); // We can now stop the confirm for the CMD_CONTROL request
                     modeAlreadyRC=true;                                        // normally we ignore but check if switching to RC_MODE via NO_CONTROL_MODE
                 }
#endif
               break;
               case SBGC_CMD_CONTROL_CONFIG:                                    // response to the cmd_control_config command (now ready to do start of RC_MODE
                 if (commGSConSet == CMD_SENT)                                  // only accept a confirm if we already sent
                 {
                    commGSConSet = CMD_CONFIRMED;                               // stop sending the request
                 }
                 else
                 {
                    modeInConfirm=modeInConfirm^1U;                             // We can declare we got s CMD_CONTROL_CONFIG success so toggle the mode for CMD_CONTROL
                 }
               break;
               case SBGC_CMD_DATA_STREAM_INTERVAL:                              // confirm to CMD_DATA_STREAM_INTERVAL
                 if (eventMsgState == CMD_SENT_2)                               // We sent the data stream interval request to the registered event
                 {
                     eventMsgState = CMD_CONFIRMED;
                 }
               break;
               case SBGC_CMD_PROFILE_SET:
                 if (commGSProSet == CMD_SENT)                                  // only accept a confirm if we already sent
                 {
                    commGSProSet = CMD_CONFIRMED;                               // stop sending the request
                 }
                 break;
               case SBGC_CMD_I2C_WRITE_REG_BUF:                                 // handle the confirm for EEPROM write on i2c if needed
               break;
               case SBGC_CMD_WRITE_EXTERNAL_DATA:                               // external EEPROM write
               break;
           }
          break;

        case SBGC_CMD_ERROR:                                                    // command was a SBGC error response  (Set an Alarm Word to trigger alarms)
          switch (cmderror.cmd_id)
          {
             case SBGC_ERR_CMD_SIZE:
               SBGCErrorCode.AlarmWdCurrent = SBGCErrorCode.AlarmWdCurrent | 1U;
             break;
             case SBGC_ERR_WRONG_PARAMS:
               SBGCErrorCode.AlarmWdCurrent = SBGCErrorCode.AlarmWdCurrent | 2U;
             break;
             case SBGC_ERR_GET_DEVICE_ID:
               SBGCErrorCode.AlarmWdCurrent = SBGCErrorCode.AlarmWdCurrent | 4U;
             break;
             case SBGC_ERR_CRYPTO:
               SBGCErrorCode.AlarmWdCurrent = SBGCErrorCode.AlarmWdCurrent | 8U;
             break;
             case SBGC_ERR_CALIBRATE_BAT:
               SBGCErrorCode.AlarmWdCurrent = SBGCErrorCode.AlarmWdCurrent | 16U;
             break;
             case SBGC_ERR_UNKNOWN_COMMAND:
               SBGCErrorCode.AlarmWdCurrent = SBGCErrorCode.AlarmWdCurrent | 32U;
             case SBGC_CMD_AUTO_PID2:
                switch (cmderror.reply)
                {
                   case SBGC_ERR_READ_FROM_EEPROM:
                      SBGCPIDTune2Error.AlarmWdCurrent = SBGCPIDTune2Error.AlarmWdCurrent | 1U;
                   break;
                   case SBGC_ERR_CANT_RUN_ALGORITHM:
                      SBGCPIDTune2Error.AlarmWdCurrent = SBGCPIDTune2Error.AlarmWdCurrent | 2U;
                   break;
                   case SBGC_ERR_WRITE_TO_EEPROM:
                      SBGCPIDTune2Error.AlarmWdCurrent = SBGCPIDTune2Error.AlarmWdCurrent | 4U;
                   break;
                   case SBGC_ERR_UNKNOWN:
                      SBGCPIDTune2Error.AlarmWdCurrent = SBGCPIDTune2Error.AlarmWdCurrent | 8U;
                   break;
                   case SBGC_ERR_WRONG_CMD_SIZE:
                      SBGCPIDTune2Error.AlarmWdCurrent = SBGCPIDTune2Error.AlarmWdCurrent | 16U;
                   break;
                   default:
                   break;
                }
             break;
          }
          g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_ERROR;                  // set the flag to say we processed it
          break;
        case SBGC_CMD_SET_ADJ_VARS_VAL:                                         // command was a SBGC set adj vars val of 7 in response to our get adj vars val
          userselect.v1 = (float32_t) getvar_rd.param1_value;                       // This is the data the user selected with the CMD_GET_ADJ_VARS_VAL request
          userselect.v2 = (float32_t) getvar_rd.param2_value;
          userselect.v3 = (float32_t) getvar_rd.param3_value;
          userselect.v4 = (float32_t) getvar_rd.param4_value;
          userselect.v5 = (float32_t) getvar_rd.param5_value;
          userselect.v6 = (float32_t) getvar_rd.param6_value;
          userselect.v7 = (float32_t) getvar_rd.param7_value;
          g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_SET_ADJ_VARS_VAL;         // set the flag to say we processed it
          break;
        case SBGC_CMD_BOARD_INFO:                                               // We received a board info request stop once we got it
          if (boardInfo == CMD_SENT)
          {
              boardInfo = CMD_CONFIRMED;
              if (boardinforb.FIRMWARE_VER >= 2660U)                            // We look at the boot state of the board if we have firmware > 2.66
              {
                  g_boardReady = boardinforb.STATE_FLAGS1 & SBGC_BOARD_STARTUP_AUTO_ROUTINE_DONE;
              }
              else
              {
                  g_boardReady = !(boardinforb.STATE_FLAGS1 & SBGC_BOARD_DEBUG_MODE);
              }
              g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_BOARD_INFO;         // set the flag to say we processed it
          }
          break;
        case SBGC_CMD_READ_PARAMS_3:                                            // This is returned when we do a CMD_AUTO_PID (auto tune) or a CMD_WRITE_PARAMS_3 (config)
           PIDBlock.ProfileId = readparam3.PROFILE_ID;
           for (eachPID=0U; eachPID<=2U ; eachPID++)
           {
              PIDBlock.P[eachPID] = readparam3.PROPORTIONAL[eachPID];           // P Band
              PIDBlock.I[eachPID] = readparam3.INTEGRAL[eachPID];               // I Band
              PIDBlock.D[eachPID] = readparam3.DERIVATIVE[eachPID];             // D Band
              PIDBlock.power[eachPID] = readparam3.POWER[eachPID];              // Power
              PIDBlock.invert[eachPID] = readparam3.INVERT[eachPID];            // Invert
              PIDBlock.poles[eachPID] = readparam3.POLES[eachPID];              // Poles
           }
          g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_READ_PARAMS_3;                  // set the flag to say we processed it
        break;
        case SBGC_CMD_READ_PARAMS:                                              // Not used at present
        break;
        case SBGC_CMD_READ_PARAMS_EXT:                                          // Not used at present
        break;
        case SBGC_CMD_READ_PARAMS_EXT2:                                         // Not used at present
        break;
        case SBGC_CMD_READ_PARAMS_EXT3:                                         // Not used at present
        break;
        case SBGC_CMD_BOOT_MODE_3:                                              // Not used at present
        break;
        case SBGC_CMD_REALTIME_DATA_CUSTOM:                                     // Not currently used
        // %$$1 realtimedatacust.ahs_debug_info need to copy to ahrs_helper_strut here
          memcpy(&ahrsdebuginfo, &realtimedatacust.ahs_debug_info, sizeof(realtimedatacust.ahs_debug_info));
          switch ((ahrsdebuginfo.EXT_IMU_STATUS&0x7U))                          // bottom 3 bits
          {
             case SBGC_STATUS_DISABLED:
               ExtIMUAlarms1.AlarmWdCurrent |= 1U;                              // External IMU alarms word 1
               break;
             case SBGC_STATUS_NOT_CONNECTED:
               ExtIMUAlarms1.AlarmWdCurrent |= 2U;
               break;
             case SBGC_STATUS_UNKNOWN:
               ExtIMUAlarms1.AlarmWdCurrent |= 4U;
               break;
             case SBGC_STATUS_ERROR:
               ExtIMUAlarms1.AlarmWdCurrent |= 8U;
               break;
             case SBGC_STATUS_BAD:
               ExtIMUAlarms1.AlarmWdCurrent |= 16U;
               break;
             case SBGC_STATUS_COARSE:
               ExtIMUAlarms1.AlarmWdCurrent |= 32U;
               break;
             case SBGC_STATUS_GOOD:
               ExtIMUAlarms1.AlarmWdCurrent |= 64U;
               break;
             case SBGC_STATUS_FINE:
               ExtIMUAlarms1.AlarmWdCurrent |= 128U;
               break;
        }
        ExtIMUAlarms2.AlarmWdCurrent |= (ahrsdebuginfo.EXT_IMU_STATUS&SBGC_STATUS_FLAG_BAD_MAG);
        ExtIMUAlarms2.AlarmWdCurrent |= ((ahrsdebuginfo.EXT_IMU_STATUS&SBGC_STATUS_FLAG_NO_GPS_SIGNAL) * 2U);
        g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_REALTIME_DATA_CUSTOM;                  // set the flag to say we processed it
        break;
        case SBGC_CMD_RESET:                                                    // from CMD_RESET or CMD_BOOT_MODE_3 (when requested)
        break;
        case SBGC_CMD_EVENT:                                                    // state change event to process
           if (eventCmd.event_id == SBGC_EVENT_ID_MOTOR_STATE)
           {
              switch(eventCmd.event_type)
              {
                  case SBGC_EVENT_TYPE_OFF:
                    gimbalMotor.State = false;
                    break;
                  case SBGC_EVENT_TYPE_ON:
                    gimbalMotor.State = true;
                    break;
                  case SBGC_EVENT_TYPE_HOLD:
                    gimbalMotor.Hold= true;
                    break;
                  default:
                     break;
              }
           }
           else if (eventCmd.event_id == SBGC_EVENT_ID_EMERGENCY_STOP)
           {
                  case SBGC_EVENT_TYPE_OFF:
                    gimbalMotor.ESD = false;
                    break;
                  case SBGC_EVENT_TYPE_ON:
                    gimbalMotor.ESD = true;
                    gimbalMotor.State = false;
                    break;
                  default:
                     break;
           }
           else if (eventCmd.event_id == SBGC_EVENT_ID_SCRIPT)
           {
              switch(eventCmd.event_type)
              {
                  case SBGC_EVENT_TYPE_OFF:
                    scriptStateSlot[eventCmd.param1[0u]] = false;               // set the state for the script state (run/stopped) where the array index is the slot position
                    break;
                  case SBGC_EVENT_TYPE_ON:
                    scriptStateSlot[eventCmd.param1[0u]] = true;                // set the state for the script state (run/stopped) where the array index is the slot position
                    break;
                  default:
                     break;
              }                                                                 // write the handler for script start stop
           }
        g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_EVENT;                  // set the flag to say we processed it
        break;
        case SBGC_CMD_SCRIPT_DEBUG:                                             // feedback from starting script on gimbal
          switch (scriptstate.ERR_CODE)
          {
             case SBGC_FILE_NO_ERROR:
              if (commGSRunAScript = CMD_SSENT)
              {
                commGSRunAScript = CMD_WCOMPLETE;                               // Now advance to be able to stop the script
              }
              break;
              case SBGC_FILE_ERR_EEPROM_FAULT:
              if (commGSRunAScript = CMD_SSENT)
              {
                  commGSRunAScript = CMD_WCOMPLETE;                             // Complete the command
               }
              ScriptRunAlarms1.AlarmWdCurrent |= 1U;
              break;
              case SBGC_ERR_FILE_NOT_FOUND:
              if (commGSRunAScript = CMD_SSENT)
              {
                 commGSRunAScript = CMD_WCOMPLETE;                              // Complete the command
              }
              ScriptRunAlarms1.AlarmWdCurrent |= 2U;
              break;
              case SBGC_ERR_FAT:
              if (commGSRunAScript = CMD_SSENT)
              {
                  commGSRunAScript = CMD_WCOMPLETE;                             // Complete the command
              }
              ScriptRunAlarms1.AlarmWdCurrent |= 4U;
              break;
              case SBGC_ERR_NO_FREE_SPACE:
              if (commGSRunAScript = CMD_SSENT)
              {
                  commGSRunAScript = CMD_WCOMPLETE;                             // Complete the command
              }
              ScriptRunAlarms1.AlarmWdCurrent |= 8U;
              break;
              case SBGC_ERR_FAT_IS_FULL:
              if (commGSRunAScript = CMD_SSENT)
              {
                  commGSRunAScript = CMD_WCOMPLETE;                             // Complete the command
              }
              ScriptRunAlarms1.AlarmWdCurrent |= 16U;
              break;
              case SBGC_ERR_FILE_SIZE:
              if (commGSRunAScript = CMD_SSENT)
              {
                  commGSRunAScript = CMD_WCOMPLETE;                             // Complete the command
              }
              ScriptRunAlarms1.AlarmWdCurrent |= 32U;
              break;
              case SBGC_FILE_ERR_CRC:
              if (commGSRunAScript = CMD_SSENT)
              {
                  commGSRunAScript = CMD_WSEND;                                 // try to EEprom write the script again
              }
              ScriptRunAlarms1.AlarmWdCurrent |= 64U;
              break;
              case SBGC_FILE_ERR_LIMIT_REACHED:
              if (commGSRunAScript = CMD_SSENT)
              {
                 commGSRunAScript = CMD_WCOMPLETE;                              // Complete the command
              }
              ScriptRunAlarms1.AlarmWdCurrent |= 128U;
              break;
              case SBGC_ERR_FILE_CORRUPTED:
              if (commGSRunAScript = CMD_SSENT)
              {
                 commGSRunAScript = CMD_WSEND;                                  // try to EEprom write the script again
              }
              ScriptRunAlarms2.AlarmWdCurrent |= 1U;
              break;
              case SBGC_FILE_ERR_WRONG_PARAMS:
              if (commGSRunAScript = CMD_SSENT)
              {
                 commGSRunAScript = CMD_WCOMPLETE;                              // Complete the command
              }
              ScriptRunAlarms2.AlarmWdCurrent |= 2U;
              break;
           }
           g_SBGCmsgCollect=g_SBGCmsgCollect & !SBGC_SER_SCRIPT_DEBUG;          // set the flag to say we processed it
           break;
           
           default:
           break;
      }

      // examples on calculating quarternion co-ordinates from gyro,acc and mag data any of these algorythms may be chosen and used
      // from just gyro and acc to be used then set mag data to zero
      //
      // 3 different methods are shown below (You only need one but we can try each one, during testing).....
      //

       // Mahony
       MahonyAHRSupdate( commgyroacc.gyro_roll, commgyroacc.gyro_pitch, commgyroacc.gyro_yaw, commgyroacc.acc_roll, commgyroacc.acc_pitch, commgyroacc.acc_yaw, commgsmagnet.mag_data_roll, commgsmagnet.mag_data_pitch, commgsmagnet.mag_data_yaw, &quartAHRS);
       // Madgewick
       MadgwickAHRSupdate( commgyroacc.gyro_roll, commgyroacc.gyro_pitch, commgyroacc.gyro_yaw, commgyroacc.acc_roll, commgyroacc.acc_pitch, commgyroacc.acc_yaw, commgsmagnet.mag_data_roll, commgsmagnet.mag_data_pitch, commgsmagnet.mag_data_yaw, &quartAHRS);
       // Orientation Filter for an IMU / AHRS sensor
       if ((( commgsmagnet.mag_data_roll == 0u ) && ( commgsmagnet.mag_data_pitch == 0u )) && ( commgsmagnet.mag_data_yaw == 0u ))
       {
          IMU_filterUpdate( commgyroacc.gyro_roll, commgyroacc.gyro_pitch, commgyroacc.gyro_yaw, commgyroacc.acc_roll, commgyroacc.acc_pitch, commgyroacc.acc_yaw, &quartAHRS);
       }
       else
       {
          MARG_AHRS_filterUpdate( commgyroacc.gyro_roll, commgyroacc.gyro_pitch, commgyroacc.gyro_yaw, commgyroacc.acc_roll, commgyroacc.acc_pitch, commgyroacc.acc_yaw, commgsmagnet.mag_data_roll, commgsmagnet.mag_data_pitch, commgsmagnet.mag_data_yaw, &quartAHRS);
       }
}

/*******************************************************************************
* Function Name: SBGC_Play_Sound
********************************************************************************
* Summary:
*  Example to play a sound on the Gimbal
* Parameters:
*  unsigned char *pMusicSequence, uint8_t noteLength
* Return:
*  nothing
*
*******************************************************************************/
void SBGC_Play_Sound( unsigned char *pMusicSequence, uint8_t noteLength )
{
  uint8_t loop;
  
  if (pMusicSequence == NULL)
      return;

  switch (noteLength)                                                           // Play a tune on the gimbal to indicate communication initiation
  {
      case 1u:
      musicPlay.note_length=0xFFU;                                              // duration 127 2 seconds full note
      break;
      case 2u:
      musicPlay.note_length=0x7FU;                                              // duration 127 1 seconds 1/2 note
      break;
      case 4u:
      musicPlay.note_length=0x3FU;                                              // duration 127 0.5 seconds 1/4 note
      break;
      case 8u:
      musicPlay.note_length=0x1FU;                                              // duration 127 0.25 seconds 1/8 note
      break;
      case 16u:
      musicPlay.note_length=0x0FU;                                              // duration 127 0.125 seconds 1/16 note
      break;
      case 32u:
      musicPlay.note_length=0x08U;                                              // duration 127 0.0625 seconds 1/32 note
      break;
  }
  head.commandID = SBGC_CMD_BEEP_SOUND;                                         // Issue a music request
  head.sizeData=sizeof(musicPlay);                                              // Byte 3 of the header
  if (boardinforb.FIRMWARE_VER >= 2680U)                                         // If we have the new firmware loaded
  {
     head.stxCHAR=SBGC_CMD_START_BYTE_V2;                                       // $ char to start message when 2.68b0 or greater
  }
  else
  {
     head.stxCHAR=SBGC_CMD_START_BYTE;                                          // $ char to start message when 2.68b0 or greater
  }
  head.cmdSize=(head.commandID + head.sizeData) % 256U;                         // Byte 4 of the header
  musicPlay.mode = SBGC_BEEPER_MODE_CUSTOM_MELODY;                              // choose to play a custom tune
  musicPlay.decay_factor=0x05u;                                                 // decay
  memcpy(&musicPlay+12u, pMusicSequence, sizeof(pMusicSequence));               // Copy in the tune to that part of the payload
  memcpy(Buffer, &head, sizeof(head));                                          // Copy the header for the music message
  memcpy(Buffer+sizeof(head), &musicPlay, head.sizeData);                       // Copy the payload
  if (boardinforb.FIRMWARE_VER >= 2680U)                                         // If we have the new firmware loaded
  {
     newCRC = crc16_arc_calculate(sz_musicPlay, ptr_musicPlay);                 // Calculate new 16 bit CRC for new version
     memcpy(Buffer+sz_musicPlay+sizeof(head),&newCRC,sizeof(newCRC));           // Tag the CRC
     totalMsgLen = sizeof(musicPlay)+sizeof(head)+sizeof(newCRC);               // message length
  }
  else
  {
     crc = checksum(ptr_musicPlay, sz_musicPlay);                               // caculate the payload checksum
     memcpy(Buffer+sizeof(musicPlay)+sizeof(head),&crc,sizeof(crc));            // Tag the CRC
     totalMsgLen = sizeof(musicPlay)+sizeof(head)+sizeof(crc);                  // message length
  }
  memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                        // Null terminate the Buffer for serial
  for(loop=0;loop<totalMsgLen;loop++)
  {
    UART2_write(Buffer[loop]);                                                  // Send it to serial UART2
  }
  udpSent = 0U;
  while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
  {
     udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
     udpSent = ++udpSent % UINT8_MAX;
  }
}
#endif                                                                          // ================= eNd Gimbal common functions ==========================

/*
   -------------$$$$$$$$$----------------------- Main Program Loop -------------------------------------------------$$$$$$$$$------------------------------
*/
void main()
{
   /* Define the Analogue inputs & Digital input & output pins
    the path to these is shown in io.h */
#if defined(ENCODER_HELPER)
   DINTRIS2=0b11111111;                                                         // Set DIN port (defined in io.h) as input DIN bits.
   INVDIN2=0b00000000;                                                          // no inputs on the rail 2 are inverted
#else
   AINTRIS1=0b00000111;                                                         // Set the AIN port as AIN words (1st 3 are inputs)
#endif
   DINTRIS1=0b11111111;                                                         // Set DIN port (defined in io.h) as input DIN bits.
   INVDIN1=0b00000000;                                                          // no inputs on the rail 1 are inverted
   DINTRIS3=0b11111111;                                                         // set DIN port 3 to be all digital inputs
   INVDIN3=0b00000000;                                                          // no inputs on the rail 3 are inverted
   DINTRIS4=0b11111111;                                                         // set DIN port 4 to be all digital inputs
   INVDIN4=0b00000000;                                                          // no inputs on the rail 4 are inverted
   DOTTRIS1=0b00000000;                                                         // Set DOT 1 port as output DOT bits.
   DOT1ODRAIN=0b00000000;                                                       // all buffered no open-drain
   DOTTRIS2=0b00000000;                                                         // Set DOT 2 port as output DOT bits.
   DOT2ODRAIN=0b00000000;                                                       // all buffered no open-drain
   DOTTRIS3=0b00000000;                                                         // Set DOT 3 port as output DOT bits.
   DOT3ODRAIN=0b11111111;                                                       // all open-drain no buffered
   AD1PCFG = IOCONFIG;                                                          // PORTB config analog/digital As per io.h first 6 inputs analog the rest are change notification digitals on PORT B
   AD1CSSL=0x07U;                                                               // Set the first 3 AIN pins to be scanned as analogue input (next 3 are spares not activated as yet)
   ANI_VOLT_EXT_REF_0V;                                                         // external supply volatge to analogue VREFH is the voltage on the VREF+ pin and VREFL is 0 V
   AD1CON1=IO_AD_ENABLE | IO_FORM_ASAM | IO_FORM_I32;                           // Set up the control register SFR for ADC  (32 bit format)
   AD1CON3 = AIN_TAD_NS(150u) | (AIN_TAD_SAM<<8u);                              // set tad to 150 ns and length of autosampling time as 750 ns (5 times the tad as defined in io.h)
   //AD1CHS=IO_CH0SA_0 | IO_CH0SB_1;                                            // Register selects the input pins to connect to SHA
   CLR_INVERT_ADC;                                                              // make VREFL the -ve input for the analog input channel

   AD1CON1=((IO_AD_ENABLE | (AD1CON1 & IO_FORM_MSAM)) | IO_FORM_I32);           // enable ADC set it to manual sample form i32

   OSCCON=IO_FRC_OSC;                                                           // Set the oscillator to FRC internal fast RC (default)
   CM1CON=0U;                                                                   // Disable comparitor

   INTCON=INTCON | 0b10101;                                                     // set the bits true so that the i/o interrupts are rising edge (1) ESD on INT0 and full batch tanks on INT2 + 4 and falling edge (0) and low batch tanks on INT1 + 3
#if defined(EXTENDED_ARM)                                                       // extended arm has a esd trip for overwindings on the motor
   SET_PRIO_IO0_INTER(6u,1u);                                                   // set up the rising edge esd trip for overwindings / esd push buttons safety relay input (PL? or SIL? rated)
#endif
   IPTMR=0u;                                                                    // IPTMR set to delay an interrupt execution we set to zero atm

   CP0_SET(CP0_COMPARE,40000000UL);                                             // set tick interrupt in co-processor register to 40 M which is 1 second (this interrupt can be very fast or slow its works on ticks)
   SET_PRIO_COMPARE0_INTER(2u,0u);                                              // interrupt when ticks reach 40M

   CN_PULL_UP_CONFIG = ESD_INPUT_PU1 | ESD_INPUT_PU2;                           // configure pull resistors to be active for CN6 and CN7 inputs
   SET_PRIO_CN_INTER(3u,2u);                                                    // enable the change notification interrupt
   
#if (defined(SBGC_GIMBAL_HMI) && defined(SBGC_GIMBAL_JOY))
   // Initialise the header with the STX character.
   head.stxCHAR=SBGC_CMD_START_BYTE;                                            // > char to start message this will change if its not the right version once successfully connected
#endif

#if defined(CS_FROZEN_FROZEN_H_) || defined(__ColorHelp_h) || defined(__comgs_h) || defined(_RADIX64_H) || defined(PROTOBUF_C_H) /* uses malloc calls for dynamic memory */
   MM_Init();                                                                   // above libraries use Lib_MemManager.h so initialise
#endif
   Init_MCU();                                                                  // Initialise the MCU and RTC
   ADC1_Init();                                                                 // Initialise the ADC for use with ADC1_Get_Sample();

   Init_Node();                                                                 // Set up the Remote and Source IP & MAC Address
   //Init_PHYPins();                                                               Initialise physical pins
   Init_Ether();                                                                // Initialise the ethernet

   Init_UART(_UART_8BIT_EVENPARITY);                                            // Initialise all the serial UART, UART2 parity for serial message. (echo debug)

   EnableInterrupts();                                                          // Enable all interrupts
   INTCON |= (GIE_ON | PEIE_ON);                                                // enable peripheal interrupts and global interrupts  (think above already does ?)

#ifdef TIMER1_NEEDED
   Init_Timer1();                                                               // Initialise the timer interrupt to count Net_Ethernet_Intern_userTimerSec for UDP frame retry
#endif
   Init_Timer5();                                                               // Initialise the timer interrupt for counting time (used by debounce)

#if defined(JRT_LIDDAR_USED)
   initUSB( &JRTSerial );                                                       // distance sensor using USB
#endif
   //ReadUSB_JRT( &distSensIT03Meas, &JRTSerial, &distSensIT03Conf );           Now read in the interrupt poll reply freq 100Hz

   initRTOfromNTP( );                                                           /* read a timesync message initialise the RTC and RION object from the message */
   
   Air.Link_Established=false;                                                  // Set the link to down unless succesful ARP resolved force at least 1 boot ARP request
#if defined(SEQ_CAM_USED) || defined(YI_CAM_USED) || defined(HTTP_USED) || defined(MODBUS_TCP) || defined(REMOTE_TCP_RUN_CAM) || defined(PENTAX_CAM_USED) || defined(JSON_COMS_USED) || defined(REMOTE_TCP_SBGC) || defined(REMOTE_TCP_AMP_XY_CAM) || defined(USE_TCP_CAN) || defined(MICROSCAN_USED) || defined(LW3_SWITCH_USED)
   StartTCPCommunication();                                                     // if you want TCP/IP
#endif
   while (!Air.Link_Established)                                                // Wait for good ARP link is up
   {
     boot_arp();                                                                // Do an ARP upon boot-up
     doPacketReturn=Net_Ethernet_Intern_doPacket();                             // process an incoming packets
     if (doPacketReturn==1u)                                                    // Reply to ARP if requested to if 1 returns reset needed
     {
       Init_Ether();                                                            // Initialise the ethernet
     }
     else if(doPacketReturn==0u)                                                //  The UDP Packet incoming was good and for us so look at it
     {
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
       msgType=SBGC_process_UDP_InBuffer( &boardinforb );                       // Read what you had in the SBGC UDP datagram data segment.
       // Uart5_write_text("Packet seen for us\n\r");                           ---- DEBUG might need enabling for unit testing -------
       SBGC_Process_Data_Read( msgType );                                       // Place the data from the Buffer as read response from gimbal controller
#endif                                                                          // ========== eNd SBGC Gimbal =================================
#if (CAMERA_TYPE == XS_CAM)
       XSEncoderResp=XS_process_UDP_InBuffer();                                 // See if we got a confirm response from the XSStream encoder
       if (XSEncoderResp == XS_AUTH)                                            // The response was we need to autherise then re-log in
       {
         memcpy(Buffer,&XStreamLogin,sizeof(XStreamLogin));
         totalMsgLen=sizeof(XStreamLogin);
         udpSent = 0U;
         while ((XSReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))    // send up to 3 times dont handle error anymore as we should re-trigger anyway
         {
           XSReturn = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, sizeof(XStreamLogin));
                                                                                // send the Buffer contents as UDP datagram.
           udpSent=++udpSent % UINT8_MAX;
         }
         if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;         // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif                                                                          // ========== eNd UART4 ======================================
       }
#endif                                                                          // ========== eNd XS Encoder =================================
     }
   }

#ifdef ETHER_ENABLE_MAC_FILTER                                                  // We want Receive filtering turned on  Received packet Destination address is us ? but did it mean remote MAC ???
   compute_hashtable_fast32( This_Node.MacAddr );                               // Receive Filtering put SOURCE MAC Address into hash table to filter only those messages
   compute_hashtable_fast32( gRemoteUnit.mac );                                 // Receive Filtering put DEST MAC Address into hash table to filter only those messages
#endif

#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
   gimbalMotor.TimeMax = 10U;                                                   // Max time for motor to change state without alarm generation
#endif                                                                          // ========== eNd SBGC Gimbal =================================

#if (CAMERA_TYPE == XS_CAM)
  strcpy(XSencoderIPUse,XSencoderIP);                                           // Copy the encoder IP from default in the program
  strcpy(XSgatewayIPUse,XSgatewayIP);                                           // Copy the encoder gw IP from default in the program
#endif                                                                          // ========== eNd XS Encoder =================================

#if defined(CRAFT_MTR_PIDS)                                                     /* flight craft control loops for engine speed */
  for (craftMtr=1u;craftMtr<=CRAFT_NUM_OF_MTRS;craftMtr++)                      /* for each motor in the craft */
  {
#if defined(USE_SPEKTRUM)
    SpekBusObject[craftMtr-1u].identifier = SPEK_ELE_SPEED_CONT;                /* object is an electric speed controller */
    SpekBusObject[craftMtr-1u].sID = craftMtr;                                  /* the controller address for each motor */
#endif
    craftSpeedPID[craftMtr-1u].type = pid_Type_Der2;                            /* type 2 algo as per (C) 2015 Jesus Ruben Santa Anna Zamudio */
    craftSpeedPID[craftMtr-1u].mode_began == false;                             /* set loop timer to initialise */
    craftSpeedPID[craftMtr-1u].mode_man==false;                                 /* make sure its not in manual */
#if defined(TERRAIN_FOLLOW_USED)
    craftSpeedPID[craftMtr-1u].mode_rem==true;                                  /* make sure its not in remote setpoint mode */
    craftSpeedPID[craftMtr-1u].rem_setpoint = CRAFT_START_UP_SPEED;
#else
    craftSpeedPID[craftMtr-1u].setpoint = CRAFT_START_UP_SPEED;
#endif
  }
#endif
#if defined(JRT_LIDDAR_USED)
    JRTSerial.State = USB_NO_DATA_READ;                                         /* initialise the read state to no message */
    JRTSerial.WriteState = USB_CONFIG_WRITE_START;                              /* set the write state to write a configuration message to start the distance sensor polled measurement message */
#endif /* end chengdhu jrt distance sensor */

//
// ============== CONTINUOUS LOOP ==============================================
//
for (;;)                                                                        // Repeat FOREVER
{
   updateRTOfromRTC( );                                                         /* update the global RION object from the RTC for global time */
#if defined(USE_MAVLINK)
   receiveMavlinkPacketReader( &MavLinkBuf.Buffer, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN, &mavMsgRcv, &mavStat );
   if ((mavStat.parse_state==MAVLINK_PARSE_STATE_GOT_CRC1) && (MavLinkBuf.State==UAMAV_PACKET_IN_BUFFER))  /* got a message with a good CRC ignore bad ones */
   {
#if defined(MAV_GIMBAL_CTRL)
      if (getQuartenionDataFromMAV( &mavMsgRcv, &mavQuartSpd )==1)
      {
         if (((mavQuartSpd.q1!=0.0f) && ((mavQuartSpd.q2!=0.0f) && (mavQuartSpd.q3!=0.0f))) && (mavQuartSpd.q4!=0.0f))  // if we are sending quartenion we are selecting angle control
            setGimbalAnglefromMAV( &mavQuartSpd, &adjvar, &commGSAdjvar );      // read the latest data from the incoming MAVLINK message and set angle params in the gimbal adjvar object
         else
            setGimbalSpeedfromMAV( &mavQuartSpd, &adjvar, &commGSAdjvar );      // read the latest data from the incoming MAVLINK message and set speed param in the gimbal adjvar object
      }
#endif  /* mavlink controls the gimbal */
      mavStat.parse_state= MAVLINK_PARSE_STATE_IDLE;                            // looks like we need to set the state engine back to idle once we have processed the message
   }
#endif  /* mavlink is being used */
   
   fileDatInput->fileread.spraymix.recipe.xTraStep = 0;                         /* ====== TODO::: !!! to change to this i think below is an error with the pointers */
   
#if defined(USE_MMC_SPI2)
/* =================== Read realtime saved data file ======================== */
   initMMCFileSys( );                                                           /* initialise the multimedia file system then wait unless failures request re-init */
   if ((g_mmcStep==MMC_FAT16_READY) && !(mainStartState&_MMC_REALTIME_DONE))    /* FAT 16 file system is ready and we havent read the realtime parameters yet */
   {
      strcpy(realTimeFile,"realtime.dat");
      if (mmcOpenFileRecord( realTimeFile, fileDatInput, MMC_REALTIME_DATA, 0u )!=MMC_FILE_OK)   /* read realtime data input mmc saved parameters file */
      {
          /* error opening realtime data input file default applied here */
          fileDatInput->realTimeData->curProduct=curProduct;                    /* set the product to the default product */
          fileDatInput->realTimeData->posSpDrvRem=true;                         /* default to remote */
      }
      mainStartState= mainStartState | _MMC_REALTIME_DONE;                      /* set state that we read the realtime file */
   }
   else if ((g_mmcStep!=MMC_FAT16_READY) && !(mainStartState&_MMC_REALTIME_DONE))
   {
      if (g_FATprevState==MMC_FAT_FAIL)                                         /* mmc fat failure has occurred */
      {
         calculateTick2Now( &g_mmcTimeDuration, &g_mmcTimeDurationLast );         /* calculate global timer for File allocation table timeout at start-up */
         if ( g_mmcTimeDuration >= MMC_INIT_MAX_WAIT )                          /* FAT initialisation has exceeded maximum timer */
         {
            fileDatInput->realTimeData->curProduct=curProduct;                  /* set the product to the default product */
            fileDatInput->realTimeData->posSpDrvRem=true;                       /* default to remote */
            mainStartState= mainStartState | _MMC_REALTIME_DONE;                /* set state that we read the realtime file */
            g_mmcStep==MMC_SLOW_POLL;                                           /* fail the mmc conenction and set it to slowly poll incase card was forgotten */
         }
      }
   }
#else
   fileDatInput->realTimeData->curProduct=curProduct;                           /* set the product to the default product */
   fileDatInput->realTimeData->posSpDrvRem=true;                                /* default to remote */
   mainStartState= mainStartState | _MMC_REALTIME_DONE;                         /* set state that we read the realtime file */
#endif
   if (mainStartState&_MMC_REALTIME_DONE)                                       /* we have either read the realtime file into memory or defaulted the parameters */
   {
      posSprayDrive.mode_rem=fileDatInput->realTimeData->posSpDrvRem;           /* recall the previous state for the sprayer speed controls */
   }
   
/* ========== Read the product data input file ======================== */
#if (defined(GPS_POS_SPRAYER) && defined(BATCH_MIXER))                          /* we also have position data */
   if (fileDatInput->realTimeData->curProduct != lastProduct)                   /* product change or start */
   {
#if defined(USE_MMC_SPI2)
      if (((g_mmcStep==MMC_FAT16_READY) && (mainStartState&_MMC_REALTIME_DONE)) && !(mainStartState&_MMC_RCP_DONE))    /* file system ready realtime read complete now read recipe */
      {
         if (mmcOpenFileRecord( "products.dat", fileDatInput, MMC_SPRAY_RCP, fileDatInput->realTimeData->curProduct )!=MMC_FILE_OK)       /* batch recipe and field sprayer info data input */
         {
             /* ------------- default parameters ------------- */
             fileDatInput->fileread.spraymix->recipe->level1SptCorse=LevelFirstAdditCorse;  /* first tank corse level */
             fileDatInput->fileread.spraymix->recipe->level1SptFine=LevelFirstAdditFine;    /* first tank fine level */
             fileDatInput->fileread.spraymix->recipe->level2SptCorse=LevelSecAdditCorse;    /* second tank corse level */
             fileDatInput->fileread.spraymix->recipe->level2SptFine=LevelSecAdditFine;      /* second tank fine level */
             fileDatInput->fileread.spraymix->recipe->LevelEmpty=_LevelEmpty;   /* level to start a new re-fill batch */
             fileDatInput->fileread.spraymix->recipe->corse1Time=_corse1Time;   /* first tank corse additon time */
             fileDatInput->fileread.spraymix->recipe->corse2Time=_corse2Time;   /* first tank fine addition time */
             fileDatInput->fileread.spraymix->recipe->fine1Time=_fine1Time;     /* second tank corse additon time */
             fileDatInput->fileread.spraymix->recipe->fine2Time=_fine2Time;     /* second tank fine additon time */
             fileDatInput->fileread.spraymix->recipe->dosingTime=_dosingTime;   /* dosing spray limit tank empty warning */
             fileDatInput->fileread.spraymix->recipe->mixTime=_mixTime;         /* mixing time */
             fileDatInput->fileread.spraymix->recipe->solid1Time=_solid1Time;   /* time to add solids */
             fileDatInput->fileread.spraymix->recipe->errorTime=_errorTime;     /* time to add extra to the timed addtion setpoints to advance sequence if level not met */
             fileDatInput->fileread.spraymix->recipe->timeOverride=false;       /* override level use timed sequence always is default off */
             fileDatInput->fileread.spraymix->recipe->solidMakeUp=false;        /* add solids is default off */
             fileDatInput->fileread.spraymix->recipe->xTraStep=false;           /* 3 step 2 liquid and 1 solid is default off */

             /* ------ gps position sprayer Field No.1 ------ */
             while (fld<=NO_SPRAY_OF_AREAS)                                     /* for each field quadrant set the default spray */
             {
                fileDatInput->fileread.spraymix->field[fld]->arrOfSpraySpt=arrOfSpraySpt[fld];    /* set the spray speed to each quadrant to default */
                fileDatInput->fileread.spraymix->field[fld]->geoCoord->GeoidalHt=_defSprayConeHt;  /* set the cone height to default */
                fileDatInput->fileread.spraymix->field[fld]->geoCoord->coordChkDisAbl=true;  /* spray without co-ordinates */
                fld=++fld % UINT8_MAX;                                          /* iterate to next field */
             }
         }
         lastProduct=fileDatInput->realTimeData->curProduct;                    /* now save this product so as not to do this until it changes again unless we reset and it recalls it as this sets to zero */
         mmcCreateNewFile( realTimeFile, fileWritDat, MMC_REALTIME_DATA, &g_timeDate );      /* write these changes to the realtime file to remember which product we are using if we go down */
         mainStartState= _MMC_RCP_DONE;                                         /* completed recipe read */
      }
      else if (((g_mmcStep!=MMC_FAT16_READY) && (mainStartState&_MMC_REALTIME_DONE)) && !(mainStartState&_MMC_RCP_DONE))  /* file system failed and realtime completed */
      {
         /* ------------- default parameters ------------- */
         fileDatInput->fileread.spraymix->recipe->level1SptCorse=LevelFirstAdditCorse;  /* first tank corse level */
         fileDatInput->fileread.spraymix->recipe->level1SptFine=LevelFirstAdditFine;    /* first tank fine level */
         fileDatInput->fileread.spraymix->recipe->level2SptCorse=LevelSecAdditCorse;    /* second tank corse level */
         fileDatInput->fileread.spraymix->recipe->level2SptFine=LevelSecAdditFine;      /* second tank fine level */
         fileDatInput->fileread.spraymix->recipe->LevelEmpty=_LevelEmpty;       /* level to start a new re-fill batch */
         fileDatInput->fileread.spraymix->recipe->corse1Time=_corse1Time;       /* first tank corse additon time */
         fileDatInput->fileread.spraymix->recipe->corse2Time=_corse2Time;       /* first tank fine addition time */
         fileDatInput->fileread.spraymix->recipe->fine1Time=_fine1Time;         /* second tank corse additon time */
         fileDatInput->fileread.spraymix->recipe->fine2Time=_fine2Time;         /* second tank fine additon time */
         fileDatInput->fileread.spraymix->recipe->dosingTime=_dosingTime;       /* dosing spray limit tank empty warning */
         fileDatInput->fileread.spraymix->recipe->mixTime=_mixTime;             /* mixing time */
         fileDatInput->fileread.spraymix->recipe->solid1Time=_solid1Time;       /* time to add solids */
         fileDatInput->fileread.spraymix->recipe->errorTime=_errorTime;         /* time to add extra to the timed addtion setpoints to advance sequence if level not met */
         fileDatInput->fileread.spraymix->recipe->timeOverride=false;           /* override level use timed sequence always is default off */
         fileDatInput->fileread.spraymix->recipe->solidMakeUp=false;            /* add solids is default off */
         fileDatInput->fileread.spraymix->recipe->xTraStep=false;               /* 3 step 2 liquid and 1 solid is default off */

         /* ------ gps position sprayer Field No.1 ------ */
         while (fld<=NO_SPRAY_OF_AREAS)                                         /* for each field quadrant set the default spray */
         {
            fileDatInput->fileread.spraymix->field[fld]->arrOfSpraySpt=arrOfSpraySpt[fld];    /* set the spray speed to each quadrant to default */
            fileDatInput->fileread.spraymix->field[fld]->geoCoord->GeoidalHt=_defSprayConeHt;  /* set the cone height to default */
            fileDatInput->fileread.spraymix->field[fld]->geoCoord->coordChkDisAbl=true;  /* spray without co-ordinates */
            fld=++fld % UINT8_MAX;                                              /* iterate to next field */
         }
         mainStartState= _MMC_RCP_DONE;                                         /* completed recipe read */
      }
#else
     /* ------------- default parameters ------------- */
     fileDatInput->fileread.spraymix->recipe->level1SptCorse=LevelFirstAdditCorse;  /* first tank corse level */
     fileDatInput->fileread.spraymix->recipe->level1SptFine=LevelFirstAdditFine;  /* first tank fine level */
     fileDatInput->fileread.spraymix->recipe->level2SptCorse=LevelSecAdditCorse;  /* second tank corse level */
     fileDatInput->fileread.spraymix->recipe->level2SptFine=LevelSecAdditFine;    /* second tank fine level */
     fileDatInput->fileread.spraymix->recipe->LevelEmpty=_LevelEmpty;           /* level to start a new re-fill batch */
     fileDatInput->fileread.spraymix->recipe->corse1Time=_corse1Time;           /* first tank corse additon time */
     fileDatInput->fileread.spraymix->recipe->corse2Time=_corse2Time;           /* first tank fine addition time */
     fileDatInput->fileread.spraymix->recipe->fine1Time=_fine1Time;             /* second tank corse additon time */
     fileDatInput->fileread.spraymix->recipe->fine2Time=_fine2Time;             /* second tank fine additon time */
     fileDatInput->fileread.spraymix->recipe->dosingTime=_dosingTime;           /* dosing spray limit tank empty warning */
     fileDatInput->fileread.spraymix->recipe->mixTime=_mixTime;                 /* mixing time */
     fileDatInput->fileread.spraymix->recipe->solid1Time=_solid1Time;           /* time to add solids */
     fileDatInput->fileread.spraymix->recipe->errorTime=_errorTime;             /* time to add extra to the timed addtion setpoints to advance sequence if level not met */
     fileDatInput->fileread.spraymix->recipe->timeOverride=false;               /* override level use timed sequence always is default off */
     fileDatInput->fileread.spraymix->recipe->solidMakeUp=false;                /* add solids is default off */
     fileDatInput->fileread.spraymix->recipe->xTraStep=false;                   /* 3 step 2 liquid and 1 solid is default off */
     
     /* ------ gps position sprayer Field No.1 ------- */
     while (fld<=NO_SPRAY_OF_AREAS)                                             /* for each field quadrant set the default spray */
     {
        fileDatInput->fileread.spraymix->field[fld]->arrOfSpraySpt=arrOfSpraySpt[fld]; /* set the spray speed to each quadrant to default */
        fileDatInput->fileread.spraymix->field[fld]->geoCoord->GeoidalHt=_defSprayConeHt;  /* set the cone height to default */
        fileDatInput->fileread.spraymix->field[fld]->geoCoord->coordChkDisAbl=true;  /* spray without co-ordinates */
        fld=++fld % UINT8_MAX;                                                  /* iterate to next field */
     }
     lastProduct=fileDatInput->realTimeData->curProduct;                        /* now save this product so as not to do this until it changes again unless we reset and it recalls it as this sets to zero */
     mainStartState= _MMC_RCP_DONE;                                             /* completed recipe read */
#endif
   }
#elif (!defined(GPS_POS_SPRAYER) && defined(BATCH_MIXER))
   if (fileDatInput->realTimeData->curProduct != lastProduct)                   /* product change or start */
   {
#if defined(USE_MMC_SPI2)
      if (((g_mmcStep==MMC_FAT16_READY) && (mainStartState&_MMC_REALTIME_DONE)) && !(mainStartState&_MMC_RCP_DONE))    /* file system ready realtime read complete now read recipe */
      {
         if (mmcOpenFileRecord( "products.dat", fileDatInput, MMC_SPRAY_RCP, fileDatInput->realTimeData->curProduct )!=MMC_FILE_OK)       /* batch recipe and field sprayer info data input */
         {
             /* ------------- default parameters ------------- */
             fileDatInput->fileread.spraymix->recipe->level1SptCorse=LevelFirstAdditCorse;  /* first tank corse level */
             fileDatInput->fileread.spraymix->recipe->level1SptFine=LevelFirstAdditFine;    /* first tank fine level */
             fileDatInput->fileread.spraymix->recipe->level2SptCorse=LevelSecAdditCorse;    /* second tank corse level */
             fileDatInput->fileread.spraymix->recipe->level2SptFine=LevelSecAdditFine;      /* second tank fine level */
             fileDatInput->fileread.spraymix->recipe->LevelEmpty=_LevelEmpty;   /* level to start a new re-fill batch */
             fileDatInput->fileread.spraymix->recipe->corse1Time=_corse1Time;   /* first tank corse additon time */
             fileDatInput->fileread.spraymix->recipe->corse2Time=_corse2Time;   /* first tank fine addition time */
             fileDatInput->fileread.spraymix->recipe->fine1Time=_fine1Time;     /* second tank corse additon time */
             fileDatInput->fileread.spraymix->recipe->fine2Time=_fine2Time;     /* second tank fine additon time */
             fileDatInput->fileread.spraymix->recipe->dosingTime=_dosingTime;   /* dosing spray limit tank empty warning */
             fileDatInput->fileread.spraymix->recipe->mixTime=_mixTime;         /* mixing time */
             fileDatInput->fileread.spraymix->recipe->solid1Time=_solid1Time;   /* time to add solids */
             fileDatInput->fileread.spraymix->recipe->errorTime=_errorTime;     /* time to add extra to the timed addtion setpoints to advance sequence if level not met */
             fileDatInput->fileread.spraymix->recipe->timeOverride=false;       /* override level use timed sequence always is default off */
             fileDatInput->fileread.spraymix->recipe->solidMakeUp=false;        /* add solids is default off */
             fileDatInput->fileread.spraymix->recipe->xTraStep=false;           /* 3 step 2 liquid and 1 solid is default off */

         }
         lastProduct=fileDatInput->realTimeData->curProduct;                    /* now save this product so as not to do this until it changes again unless we reset and it recalls it as this sets to zero */
         mmcCreateNewFile( realTimeFile, fileWritDat, MMC_REALTIME_DATA, &g_timeDate );      /* write these changes to the realtime file to remember which product we are using if we go down */
         mainStartState= _MMC_RCP_DONE;                                         /* completed recipe read */
      }
      else if (((g_mmcStep!=MMC_FAT16_READY) && (mainStartState&_MMC_REALTIME_DONE)) && !(mainStartState&_MMC_RCP_DONE))  /* file system failed and realtime completed */
      {
         /* ------------- default parameters ------------- */
         fileDatInput->fileread.spraymix->recipe->level1SptCorse=LevelFirstAdditCorse;  /* first tank corse level */
         fileDatInput->fileread.spraymix->recipe->level1SptFine=LevelFirstAdditFine;    /* first tank fine level */
         fileDatInput->fileread.spraymix->recipe->level2SptCorse=LevelSecAdditCorse;    /* second tank corse level */
         fileDatInput->fileread.spraymix->recipe->level2SptFine=LevelSecAdditFine;      /* second tank fine level */
         fileDatInput->fileread.spraymix->recipe->LevelEmpty=_LevelEmpty;       /* level to start a new re-fill batch */
         fileDatInput->fileread.spraymix->recipe->corse1Time=_corse1Time;       /* first tank corse additon time */
         fileDatInput->fileread.spraymix->recipe->corse2Time=_corse2Time;       /* first tank fine addition time */
         fileDatInput->fileread.spraymix->recipe->fine1Time=_fine1Time;         /* second tank corse additon time */
         fileDatInput->fileread.spraymix->recipe->fine2Time=_fine2Time;         /* second tank fine additon time */
         fileDatInput->fileread.spraymix->recipe->dosingTime=_dosingTime;       /* dosing spray limit tank empty warning */
         fileDatInput->fileread.spraymix->recipe->mixTime=_mixTime;             /* mixing time */
         fileDatInput->fileread.spraymix->recipe->solid1Time=_solid1Time;       /* time to add solids */
         fileDatInput->fileread.spraymix->recipe->errorTime=_errorTime;         /* time to add extra to the timed addtion setpoints to advance sequence if level not met */
         fileDatInput->fileread.spraymix->recipe->timeOverride=false;           /* override level use timed sequence always is default off */
         fileDatInput->fileread.spraymix->recipe->solidMakeUp=false;            /* add solids is default off */
         fileDatInput->fileread.spraymix->recipe->xTraStep=false;               /* 3 step 2 liquid and 1 solid is default off */
         mainStartState= _MMC_RCP_DONE;                                         /* completed recipe read */
      }
#else
    /* ------------- default parameters ------------- */
    fileDatInput->fileread.spraymix->recipe->level1SptCorse=LevelFirstAdditCorse;  /* first tank corse level */
    fileDatInput->fileread.spraymix->recipe->level1SptFine=LevelFirstAdditFine;    /* first tank fine level */
    fileDatInput->fileread.spraymix->recipe->level2SptCorse=LevelSecAdditCorse;    /* second tank corse level */
    fileDatInput->fileread.spraymix->recipe->level2SptFine=LevelSecAdditFine;      /* second tank fine level */
    fileDatInput->fileread.spraymix->recipe->LevelEmpty=_LevelEmpty;            /* level to start a new re-fill batch */
    fileDatInput->fileread.spraymix->recipe->corse1Time=_corse1Time;            /* first tank corse additon time */
    fileDatInput->fileread.spraymix->recipe->corse2Time=_corse2Time;            /* first tank fine addition time */
    fileDatInput->fileread.spraymix->recipe->fine1Time=_fine1Time;              /* second tank corse additon time */
    fileDatInput->fileread.spraymix->recipe->fine2Time=_fine2Time;              /* second tank fine additon time */
    fileDatInput->fileread.spraymix->recipe->dosingTime=_dosingTime;            /* dosing spray limit tank empty warning */
    fileDatInput->fileread.spraymix->recipe->mixTime=_mixTime;                  /* mixing time */
    fileDatInput->fileread.spraymix->recipe->solid1Time=_solid1Time;            /* time to add solids */
    fileDatInput->fileread.spraymix->recipe->errorTime=_errorTime;              /* time to add extra to the timed addtion setpoints to advance sequence if level not met */
    fileDatInput->fileread.spraymix->recipe->timeOverride=false;                /* override level use timed sequence always is default off */
    fileDatInput->fileread.spraymix->recipe->solidMakeUp=false;                 /* add solids is default off */
    fileDatInput->fileread.spraymix->recipe->xTraStep=false;                    /* 3 step 2 liquid and 1 solid is default off */
    lastProduct=fileDatInput->realTimeData->curProduct;                         /* now save this product so as not to do this until it changes again unless we reset and it recalls it as this sets to zero */
    mainStartState= _MMC_RCP_DONE;                                              /* completed recipe read */
#endif
   }
#elif (defined(GPS_POS_SPRAYER) && !defined(BATCH_MIXER))
   if (fileDatInput->realTimeData->curProduct != lastProduct)                   /* product change or start */
   {
#if defined(USE_MMC_SPI2)
      if (((g_mmcStep==MMC_FAT16_READY) && (mainStartState&_MMC_REALTIME_DONE)) && !(mainStartState&_MMC_RCP_DONE))    /* file system ready realtime read complete now read recipe */
      {
         if (mmcOpenFileRecord( "products.dat", fileDatInput, MMC_SPRAY_RCP, fileDatInput->realTimeData->curProduct )!=MMC_FILE_OK)  /* batch recipe and field sprayer info data input */
         {
             /* ------ gps position sprayer Field No.1 ------ */
             while (fld<=NO_SPRAY_OF_AREAS)                                     /* for each field quadrant set the default spray */
             {
                fileDatInput->fileread.spraymix->field[fld]->arrOfSpraySpt=arrOfSpraySpt[fld];    /* set the spray speed to each quadrant to default */
                fileDatInput->fileread.spraymix->field[fld]->geoCoord->GeoidalHt=_defSprayConeHt;  /* set the cone height to default */
                fileDatInput->fileread.spraymix->field[fld]->geoCoord->coordChkDisAbl=true;  /* spray without co-ordinates */
                fld=++fld % UINT8_MAX;                                          /* iterate to next field */
             }
         }
         lastProduct=fileDatInput->realTimeData->curProduct;                    /* now save this product so as not to do this until it changes again unless we reset and it recalls it as this sets to zero */
         mmcCreateNewFile( realTimeFile, fileWritDat, MMC_REALTIME_DATA, &g_timeDate );      /* write these changes to the realtime file to remember which product we are using if we go down */
         mainStartState= _MMC_RCP_DONE;                                         /* completed recipe read */
      }
      else if (((g_mmcStep!=MMC_FAT16_READY) && (mainStartState&_MMC_REALTIME_DONE)) && !(mainStartState&_MMC_RCP_DONE))  /* file system failed and realtime completed */
      {
         /* ------ gps position sprayer Field No.1 ------ */
         while (fld<=NO_SPRAY_OF_AREAS)                                         /* for each field quadrant set the default spray */
         {
            fileDatInput->fileread.spraymix->field[fld]->arrOfSpraySpt=arrOfSpraySpt[fld];    /* set the spray speed to each quadrant to default */
            fileDatInput->fileread.spraymix->field[fld]->geoCoord->GeoidalHt=_defSprayConeHt;  /* set the cone height to default */
            fileDatInput->fileread.spraymix->field[fld]->geoCoord->coordChkDisAbl=true;  /* spray without co-ordinates */
            fld=++fld % UINT8_MAX;                                              /* iterate to next field */
         }
         mainStartState= _MMC_RCP_DONE;                                         /* completed recipe read */
      }
#else
   /* ------ gps position sprayer Field No.1 ------ */
   while (fld<=NO_SPRAY_OF_AREAS)                                               /* for each field quadrant set the default spray */
   {
      fileDatInput->fileread.spraymix->field[fld]->arrOfSpraySpt=arrOfSpraySpt[fld]; /* set the spray speed to each quadrant to default */
      fileDatInput->fileread.spraymix->field[fld]->geoCoord->GeoidalHt=_defSprayConeHt;  /* set the cone height to default */
      fileDatInput->fileread.spraymix->field[fld]->geoCoord->coordChkDisAbl=true;  /* spray without co-ordinates */
      fld=++fld % UINT8_MAX;                                                    /* iterate to next field */
   }
   lastProduct=fileDatInput->realTimeData->curProduct;                          /* now save this product so as not to do this until it changes again unless we reset and it recalls it as this sets to zero */
   mainStartState= _MMC_RCP_DONE;                                               /* completed recipe read */
#endif
   }
#endif

/* ================== Run the Batch Make Up Sequence ======================== */
#if defined(BATCH_MIXER)
#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
   if (mainStartState&_MMC_RCP_DONE)                                            /* run the batch sequencer once we have read recipe or defaulted it */
      makeUpDoseTank( &valueADC[1u], &fileDatInput->fileread.spraymix->recipe, &levelMovAvg );
#else
   if (mainStartState&_MMC_RCP_DONE)                                            /* run the batch sequencer once we have read recipe or defaulted it */
      makeUpDoseTank( &fileDatInput->fileread.spraymix->recipe, &levelMovAvg );
#endif
#if defined(USE_MMC_SPI2)
   if ((g_mmcStep==MMC_FAT16_READY) && (guiFileManager.type == MMC_SPRAY_RCP))  /* filesystem ready and full recipe and field info save requested from gui */
   {
      if (guiFileManager.tristate != MMC_ERROR_RESET)                           /* if we tried to initialise the FAT after a handle error dont reset again if it fails */
      {
         guiFileManager.tristate = MMC_ERROR_INIT;                              /* initialise fail reply */
      }
      
      if (mmcManageRecords( "recipe.dat", fileDatInput, MMC_SPRAY_RCP, fileWritDat, &guiFileManager, &g_timeDate )!=MMC_NO_HANDLER)   /* delete or append records for tank batch and field settings */
      {
         guiFileManager.type = MMC_RCP_REQ_DONE;                                /* complete the HMI state to allow another request from the GUI */
         guiFileManager.tristate = MMC_ERROR_INIT;                              /* clear the error state */
      }
      else                                                                      /* command returned no file handle for deleting */
      {
         if (guiFileManager.tristate = MMC_ERROR_INIT)                          /* first time we got an error latch */
         {
             mmcFATresetTm = -1;                                                /* initialise tick timer */
             calculateTick2Now( &mmcFATresetTm, &mmcFATresetTmLast );
             guiFileManager.tristate = MMC_ERROR_TIME;                          /* set to reset timeout */
         }
         else if (guiFileManager.tristate = MMC_ERROR_TIME)                     /* timer is initialised */
         {
             calculateTick2Now( &mmcFATresetTm, &mmcFATresetTmLast );             /* get time to now */
             if (mmcFATresetTm > MMC_FAT_RESET)                                 /* timer exceeds the max time for retrying to get a handle */
             {
                g_mmcStep==MMC_SPI_INIT;                                        /* reset SPI master interface to SD Card */
                guiFileManager.tristate = MMC_ERROR_RESET;                      /* let us know if we re-enter here */
             }
         }
         else                                                                   /* tristate = 2 second time after FAT reset occured and still failure of handle */
         {
             guiFileManager.type = MMC_RCP_REQ_DONE;                            /* complete the HMI state but keep the error level to prevent any more resets for this */
         }
      }
   }
   else if ((g_mmcStep==MMC_FAT16_READY) && (guiFileManager.type == MMC_TANK_RCP_DAT)) /* filesystem ready and only recipe save requested from gui */
   {
      if (guiFileManager.tristate != MMC_ERROR_RESET)                           /* if we tried to initialise the FAT after a handle error dont reset again if it fails */
      {
         guiFileManager.tristate = MMC_ERROR_INIT;                              /* initialise fail reply */
      }
      
      if (mmcManageRecords( "recipe.dat", fileDatInput, MMC_TANK_RCP_DAT, fileWritDat, &guiFileManager, &g_timeDate )!=MMC_NO_HANDLER) /* delete or append records for tank makeup */
      {
         guiFileManager.type = MMC_RCP_REQ_DONE;                                /* complete the HMI state to allow another request from the GUI */
         guiFileManager.tristate = MMC_ERROR_INIT;                              /* clear the error state */
      }
      else                                                                      /* command returned no file handle for deleting */
      {
         if (guiFileManager.tristate = MMC_ERROR_INIT)                          /* first time we got an error latch */
         {
             mmcFATresetTm = -1;                                                /* initialise tick timer */
             calculateTick2Now( &mmcFATresetTm, &mmcFATresetTmLast );
             guiFileManager.tristate = MMC_ERROR_TIME;                          /* set to reset timeout */
         }
         else if (guiFileManager.tristate = MMC_ERROR_TIME)                     /* timer is initialised */
         {
             calculateTick2Now( &mmcFATresetTm, &mmcFATresetTmLast );             /* get time to now */
             if (mmcFATresetTm > MMC_FAT_RESET)                                 /* timer exceeds the max time for retrying to get a handle */
             {
                g_mmcStep=MMC_SPI_INIT;                                         /* reset SPI master interface to SD Card */
                guiFileManager.tristate = MMC_ERROR_RESET;                      /* let us know if we re-enter here */
             }
         }
         else                                                                   /* tristate = 2 second time after FAT reset occured and still failure of handle */
         {
             guiFileManager.type = MMC_RCP_REQ_DONE;                            /* complete the HMI state but keep the error level to prevent any more resets for this */
         }
      }
   }
   else if ((g_mmcStep==MMC_FAT16_READY) && (guiFileManager.type == MMC_FIELD_DATA)) /* filesystem ready and only field info save requested from gui */
   {
      if (guiFileManager.tristate != MMC_ERROR_RESET)                           /* if we tried to initialise the FAT after a handle error dont reset again if it fails */
      {
         guiFileManager.tristate = MMC_ERROR_INIT;                              /* initialise fail reply */
      }
      
      if (mmcManageRecords( "recipe.dat", fileDatInput, MMC_FIELD_DATA, fileWritDat, &guiFileManager, &g_timeDate )!=MMC_NO_HANDLER)   /* delete or append records for field spray data */
      {
         guiFileManager.type = MMC_RCP_REQ_DONE;                               /* complete the HMI state to allow another request from the GUI */
         guiFileManager.tristate=MMC_ERROR_INIT;                                /* clear the error state */
      }
      else                                                                      /* command returned no file handle for deleting */
      {
         if (guiFileManager.tristate = MMC_ERROR_INIT)                          /* first time we got an error latch */
         {
             mmcFATresetTm = -1;                                                /* initialise tick timer */
             calculateTick2Now( &mmcFATresetTm, &mmcFATresetTmLast );
             guiFileManager.tristate = MMC_ERROR_TIME;                          /* set to reset timeout */
         }
         else if (guiFileManager.tristate = MMC_ERROR_TIME)                     /* timer is initialised */
         {
             calculateTick2Now( &mmcFATresetTm, &mmcFATresetTmLast );             /* get time to now */
             if (mmcFATresetTm > MMC_FAT_RESET)                                 /* timer exceeds the max time for retrying to get a handle */
             {
                g_mmcStep=MMC_SPI_INIT;                                         /* reset SPI master interface to SD Card */
                guiFileManager.tristate = MMC_ERROR_RESET;                      /* let us know if we re-enter here */
             }
         }
         else                                                                   /* tristate = 2 second time after FAT reset occured and still failure of handle */
         {
             guiFileManager.type = MMC_RCP_REQ_DONE;                            /* complete the HMI state but keep the error level to prevent any more resets for this */
         }
      }
   }
#endif /* end file system FAT16 SPI2 connection */
#endif /* batch mixer */

// ============= SBGC FIRMWARE =================================================
   // Request the Gimbal Board Info State and Firmware
   // this is needed in order to know which commands are relevant
   //
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
   if ((boardInfo == CMD_SEND ) || ((g_boardReady==false) && (boardInfo == CMD_COMPLETE)))
   {
     boardExcTimer=0U;
     boardExcTimer2=0U;
     head.commandID = SBGC_CMD_BOARD_INFO;                                      // request the board information to be sent back
     head.sizeData = 2U;                                                        // changed against manual after testing from 0

     head.cmdSize = head.commandID+head.sizeData;                               // Tag the header checksum

     if ((noOfBoardSends % 2u)==0)                                              // alternate every other command until we got a good message as we dont know which firware request is correct
     {
        head.stxCHAR=SBGC_CMD_START_BYTE_V2;                                    // $ char to start message when 2.68b0 or greater when success this is set for the rest
     }
     else
     {
        head.stxCHAR=SBGC_CMD_START_BYTE;                                       // > char to start message  version < 2.68b0 when success this is set for the rest
     }
     g_SBGC_Stx_Char=head.stxCHAR;                                              // Set the global SimpleBGC STX Char to be that found to give the firmware reply this is used on receive
     memcpy(Buffer, &head, sizeof(head));                                       // Copy the new header to the message
     totalMsgLen = SBGC_NUM_HEADER_BYTES+head.sizeData;

     while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
     {
       udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);
                                                                                // Send the Buffer contents as UDP datagram.
       udpSent=++udpSent % UINT8_MAX;
     }
     if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;             // Accumulate the bad send counter (remove the 1 before summing)
     if (udpReturn != UDP_SEND)                                                 // We did send it so wait until confirmed
     {
        boardInfo = CMD_SENT;                                                   // We sent it okay so wait for a board info reply
     }
     memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                     // Null terminate the Buffer for serial
     for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
     {
       UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
     }
     udpSent = 0U;
   }
   else if ((boardInfo == CMD_CONFIRMED) && (g_boardReady==false))                 // We didnt get the right reply but it replied to this firmware request
   {
     (boardExcTimer > SBGC_FIRMWR_RSND_DELAY) ? boardInfo=CMD_COMPLETE : boardExcTimer++ % UINT16_MAX;   // Send another request keep this firmware
     if (boardinforb.FIRMWARE_VER >= 2660U)                                     // We look at the boot state of the board if we have firmware > 2.66
     {
        g_boardReady = boardinforb.STATE_FLAGS1 & SBGC_BOARD_STARTUP_AUTO_ROUTINE_DONE;   // set the ready if the startup routine has shown completion
     }
     else
     {
        g_boardReady = !(boardinforb.STATE_FLAGS1 & SBGC_BOARD_DEBUG_MODE);       // set the ready when you're not in debug mode
     }
   }
   else if ((boardInfo == CMD_SENT) && (g_boardReady==false))                      // We didnt get a reply at all (could be wrong firware request)
   {
      if (boardExcTimer2 >= SBGC_FIRMWR_NOREP_DELAY)                            // No reply within a time
      {
         boardInfo=CMD_COMPLETE;
         noOfBoardSends=++noOfBoardSends % UINT8_MAX;                           // Count the number of attempts at getting firmware revision to toggle the request to see if its the other firmware
      }
      else
      {
         boardExcTimer2=++boardExcTimer2 % UINT16_MAX;
      }
   }
   else if (g_boardReady==true)                                                   // We completed the read of the firmware version and gimbal state then play a tune on the gimbal
   {
       if (!gimStartUpComplete)
       {
          // Play a tune on the gimbal to indicate communication initiation
          head.commandID = SBGC_CMD_BEEP_SOUND;                                 // Issue a music request
          head.sizeData=sizeof(musicPlay);                                      // Byte 3 of the header
          head.cmdSize=(head.commandID + head.sizeData) % 256U;                 // Byte 4 of the header
          musicPlay.mode = SBGC_BEEPER_MODE_CUSTOM_MELODY;                      // choose to play a custom tune
          musicPlay.note_length=0x7FU;                                          // duration 127 1/2 note
          musicPlay.decay_factor=0x05U;                                         // decay
          memcpy(&musicPlay.note_freq_hz, melodyACP1, sizeof(melodyACP1));                // Copy in the tune to that part of the payload
          memcpy(Buffer, &head, sizeof(head));                                  // Copy the header for the music message
          memcpy(Buffer+sizeof(head), &musicPlay, head.sizeData);               // Copy the payload
          if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
          {
             newCRC = crc16_arc_calculate(sz_musicPlay, ptr_musicPlay);         // Calculate new 16 bit CRC for new version
             memcpy(Buffer+sz_musicPlay+sizeof(head),&newCRC,sizeof(newCRC));   // Tag the CRC
             totalMsgLen = sizeof(musicPlay)+sizeof(head)+sizeof(newCRC);       // message length
          }
          else
          {
             crc = checksum(ptr_musicPlay, sz_musicPlay);                       // calculate the payload checksum
             memcpy(Buffer+sizeof(musicPlay)+sizeof(head),&crc,sizeof(crc));    // Tag the CRC
             totalMsgLen = sizeof(musicPlay)+sizeof(head)+sizeof(crc);          // message length
          }
          memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                // Null terminate the Buffer for serial
          for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
          {
             UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
          }
          udpSent = 0U;
          while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {
              udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
              udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // Accumulate the bad send counter (remove the 1 before summing)
          gimStartUpComplete=true;
       }
   }
#endif                                                                          // ================== eNd Init State SBGC Gimbal controller ===================

#if (CAMERA_TYPE == XS_CAM)
// ============= THIS IS ENCODER INIT STATE ENGINE =============================

   // This sequence should run once at boot-up
   // if we fail we wait until a new authentication

   // Login to the XStream video encoder and issue the set-up commands in order
   // This might require modification if the login times out (it assumes you are logged in at all times)
   // ===== ADDED A GET for all values to read current settings 1st ============
   //
   if ((XSStartUpSequence == CMD_SEND) || (XSLoginTime > MSG_REPOLL_TICKS))
   {
      XSLoginTime=0;
      switch (XSInitState)
      {
         case 0u:                                                                // Login State
         memcpy(Buffer,&XStreamLogin,sizeof(XStreamLogin));
         totalMsgLen=sizeof(XStreamLogin);
         break;

         case 1u:                                                                // Configure Network
         strcpy(XStreamNetwork.IPAddr,XSencoderIPUse);                          // Copy in the current values
         strcpy(XStreamNetwork.Gateway,XSgatewayIPUse);
         memcpy(Buffer,&XStreamNetwork,sizeof(XStreamNetwork));
         totalMsgLen=sizeof(XStreamNetwork);
         break;

         case 2u:
         memcpy(Buffer,&XStreamConf,sizeof(XStreamConf));                       // Configure Stream
         totalMsgLen=sizeof(XStreamConf);
         break;

         case 3u:                                                                //  set the RTSP port
         memcpy(Buffer,&XStreamRTSP,sizeof(XStreamRTSP));
         totalMsgLen=sizeof(XStreamRTSP);
         break;

         case 4u:                                                                // Set up the disk drive
         memcpy(Buffer,&XStreamDisk,sizeof(XStreamDisk));
         totalMsgLen=sizeof(XStreamDisk);
         break;

         case 5u:                                                                // Set up bit stream rate
         memcpy(Buffer,&XStreamBitRate, sizeof(XStreamBitRate));
         totalMsgLen=sizeof(XStreamBitRate);
         break;

         case 6u:
         memcpy(Buffer,&XStreamIInterval,sizeof(XStreamIInterval));             // Stream Interval
         totalMsgLen=sizeof(XStreamIInterval);
         break;

         case 7u:                                                                // Frames per second
         memcpy(Buffer,&XStreamFrameRate,sizeof(XStreamFrameRate));
         totalMsgLen=sizeof(XStreamFrameRate);
         break;
#ifndef ORIG_SIMPLE_CAM
         case 8u:                                                               // get the current settings for HMI variables to start with
         API_XS_GETCMD1(Buffer,"BITRATE",-1);
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 9u:                                                               // get the current settings for HMI variables to start with
         API_XS_GETCMD1(Buffer,"AVGBITRATE",-1);                                // Get the average bitrate for all channels
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 10u:                                                               // get the current settings for HMI variables to start with
         API_XS_GETCMD1(Buffer,"IINTERVAL",-1);                                 // Get the i-frame interval for all channels
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 11u:                                                              // get the current settings for HMI variables to start with
         API_XS_GETCMD1(Buffer,"FRAMERATE",-1);                                 // Get the frame rate for all channels
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 12u:                                                              // get the current settings for HMI variables to start with
         API_XS_GETCMD1(Buffer,"BRIGHTNESS",-1);                                // Get the brightness for all channels
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 13u:                                                              // get the current settings for HMI variables to start with
         API_XS_GETCMD1(Buffer,"CONTRAST",-1);                                  // Get the contrast for all channels
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 14u:                                                              // get the current settings for HMI variables to start with
         API_XS_GETCMD1(Buffer,"HUE",-1);                                       // Get the hue for all channels
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 15u:                                                              // get the current settings for HMI variables to start with
         API_XS_GETCMD1(Buffer,"SATURATION",-1);                                // Get the saturation for all channels
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 16u:                                                              // get the current settings for HMI variables to start with
         API_XS_GETCMD1(Buffer,"CLIPSIZE",-1);                                  // Get the clip size for all channels
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 17u:                                                              // get the current settings for HMI variables to start with
         API_XS_GETCMD1(Buffer,"CLIPLENGTH",-1);                                // Get the clip length for all channels
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 18u:                                                              // get the current settings for HMI variables to start with
         API_XS_GET(Buffer,"RTSPPORT");                                         // Get the RTSP port (real time streaming protocol) port
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 19u:                                                              // get the current settings for HMI variables to start with
         API_XS_ENUMD(Buffer);                                                  // Get the information of number of disks and currrent disk
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 20u:                                                              // recycle % request
         API_XS_GET(Buffer,"RECYCLE");                                          // Get the information on the threshold for automatic file deletion
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;

         case 21u:                                                              // issue a stop to all channels (this is an exception step)
         API_XS_STOPRECORD(Buffer,-1);                                          // formulate stop to all channels
         totalMsgLen=(sizeToChar( Buffer,'\r' ))+1U;                            // termination is a CR 0x0D or \r  (add 1 to include it)
         break;
#endif
      }
      udpSent = 0U;
      while ((XSReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
      {
        XSReturn = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, totalMsgLen);
                                                                                // send the Buffer contents as UDP datagram.
        udpSent=++udpSent % UINT8_MAX;
      }
      if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;            // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif
      if (XSReturn == 1u)                                                       // Was sent successfully
      {
        XSStartUpSequence == CMD_SENT;
      }
   }
   else if (XSStartUpSequence == CMD_SENT)                                      // Now wait for the response
   {
       switch (XSEncoderResp)
       {
         case XS_OK:                                                            // Login completed correctly
#ifdef ORIG_SIMPLE_CAM
         if (XSInitState==7u)                                                   // Last one done then complete sequence
         {
            XSStartUpSequence = CMD_COMPLETE;
            XSInitState=XS_STEP_SETHMI;                                         // Now set the HMI varaibles to those read from the encoder if valid
            XS_encoder_Q.loggedin=true;                                         // set a flag to say you logged and completed the start-up
         }
         //else if (XSInitState==0)                                             // You are logging in to it
         //{
         //   XS_encoder_Q.loggedin=true;                                       // set a flag to say you logged
         //}
         else if (XSInitState==21u)                                             // You issued a stop to it
         {
           XSInitState==1u;                                                     // Go back to set-up RTSP
         }
         else
         {
#endif                                                                          // end ORIG_SIMPLE_CAM
#ifndef ORIG_SIMPLE_CAM
         if (XSInitState==20u)                                                  // Last one done then complete sequence
         {
            XSStartUpSequence = CMD_COMPLETE;
            XSInitState=XS_STEP_SETHMI;                                         // Now set the HMI varaibles to those read from the encoder if valid
            XS_encoder_Q.loggedin=true;                                         // set a flag to say you logged and completed the start-up
         }
         //else if (XSInitState==0)                                               // You are logging in to it
         //{
         //   XS_encoder_Q.loggedin=true;                                         // set a flag to say you logged
         //}
         else if (XSInitState==21u)                                             // You issued a stop to it
         {
           XSInitState==1u;                                                     // Go back to set-up RTSP
         }
         else
         {
#endif                                                                          // end NOT ORIG_SIMPLE_CAM
            XSStartUpSequence = CMD_SEND;
#ifdef ORIG_SIMPLE_CAM
         }
#endif                                                                          // end NOT ORIG_SIMPLE_CAM
#ifndef ORIG_SIMPLE_CAM
         }
#endif                                                                          // end NOT NOT ORIG_SIMPLE_CAM

         XSInitState=++XSInitState % UINT8_MAX;                                             // Go to the next set-up step as we have just completed one
         break;

         case XS_AUTH:                                                          // Need to login
         if (XSInitState==0u)                                                   // You are already logging in to it and it failed
         {
            XSStartUpSequence = CMD_COMPLETE;                                   // Set the start-up sequence as complete
            XSInitState=XS_PASWD_WRONG;                                         // Exit with an alarm state (wrong password)
            XSEncoderResp = XS_AUTH_WAIT;                                       // wait for the HMI to indicate a new password or user
            XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_PW;                     // Alarm this event
         }
         else
         {
            XSStartUpSequence = CMD_SEND;
            XSInitState=0;                                                      // reset the sequence to step 1 as we need to login again
         }
         break;

         case XS_ERROR:                                                         // Error with the command try again
         XSStartUpSequence = CMD_SEND;
         break;

#ifndef ORIG_SIMPLE_CAM                                                         // We are not just a sender we also readback current information
        case XS_OK1:                                                            // Return was a +OK with one parameter
        switch (XSInitState)
        {
            case 18u:                                                           // RTSP port request
            gXSRTSPPort = g_okField1;                                           // first channel returned
            break;

            case 20u:                                                           // Get recycle % request
            gXSFileDelThreshold = g_okField1;                                   // first channel returned
            break;
        }
        XSInitState=++XSInitState % UINT8_MAX;                                  // Go to the next set-up step as we have just completed one
        break;

        case XS_OK2:                                                            // Return was a +OK with 2 parameter
        XSInitState=++XSInitState % UINT8_MAX;                                  // Go to the next set-up step as we have just completed one
        break;

        case XS_OK3:                                                            // Return was a +OK with 3 parameter
        XSInitState=++XSInitState % UINT8_MAX;                                  // Go to the next set-up step as we have just completed one
        break;

        case XS_OK4:                                                            // Return was a +OK with 4 parameter
        switch (XSInitState)
        {
            case 8u:                                                            // Bit Rate Request
            gXSBitRate.Ch1 = g_okField1;                                        // first channel bitrate returned
            gXSBitRate.Ch2 = g_okField2;                                        // second channel bitrate returned
            gXSBitRate.Ch3 = g_okField3;                                        // third channel bitrate returned
            gXSBitRate.Ch4 = g_okField4;                                        // fourth channel bitrate returned
            break;

            case 9u:                                                            // Average Bit Rate Request
            gXSAvgBitRate.Ch1 = g_okField1;                                     // first channel returned
            gXSAvgBitRate.Ch2 = g_okField2;                                     // second channel returned
            gXSAvgBitRate.Ch3 = g_okField3;                                     // third channel returned
            gXSAvgBitRate.Ch4 = g_okField4;                                     // fourth channel returned
            break;

            case 10u:                                                           // IIinterval Request
            gXSIInt.Ch1 = g_okField1;                                           // first channel returned
            gXSIInt.Ch2 = g_okField2;                                           // second channel returned
            gXSIInt.Ch3 = g_okField3;                                           // third channel returned
            gXSIInt.Ch4 = g_okField4;                                           // fourth channel returned
            break;

            case 12u:                                                           // Brightness request
            gXSBright.Ch1 = g_okField1;                                         // first channel returned
            gXSBright.Ch2 = g_okField2;                                         // second channel returned
            gXSBright.Ch3 = g_okField3;                                         // third channel returned
            gXSBright.Ch4 = g_okField4;                                         // fourth channel returned
            break;

            case 13u:                                                           // Contrast request
            gXSContrast.Ch1 = g_okField1;                                       // first channel returned
            gXSContrast.Ch2 = g_okField2;                                       // second channel returned
            gXSContrast.Ch3 = g_okField3;                                       // third channel returned
            gXSContrast.Ch4 = g_okField4;                                       // fourth channel returned
            break;

            case 14u:                                                           // Hue request
            gXSHue.Ch1 = g_okField1;                                            // first channel returned
            gXSHue.Ch2 = g_okField2;                                            // second channel returned
            gXSHue.Ch3 = g_okField3;                                            // third channel returned
            gXSHue.Ch4 = g_okField4;                                            // fourth channel returned
            break;

            case 15u:                                                           // Saturation request
            gXSSatur.Ch1 = g_okField1;                                          // first channel returned
            gXSSatur.Ch2 = g_okField2;                                          // second channel returned
            gXSSatur.Ch3 = g_okField3;                                          // third channel returned
            gXSSatur.Ch4 = g_okField4;                                          // fourth channel returned
            break;

            case 16u:                                                           // Clipsize request
            gXSClipsz.Ch1 = g_okField1;                                         // first channel returned
            gXSClipsz.Ch2 = g_okField2;                                         // second channel returned
            gXSClipsz.Ch3 = g_okField3;                                         // third channel returned
            gXSClipsz.Ch4 = g_okField4;                                         // fourth channel returned
            break;

            case 17u:                                                           // Clip length request
            gXSClipLn.Ch1 = g_okField1;                                         // first channel returned
            gXSClipLn.Ch2 = g_okField2;                                         // second channel returned
            gXSClipLn.Ch3 = g_okField3;                                         // third channel returned
            gXSClipLn.Ch4 = g_okField4;                                         // fourth channel returned
            break;

            case 19u:                                                           // Enum disk request
            gXSEdisk.numdisks = (uint8_t) g_okField1;                           // first channel returned
            strcpy(gXSEdisk.id,g_strField2);                                    // second channel returned
            gXSEdisk.size = g_okField3;                                         // third channel returned
            strcpy(gXSEdisk.description,g_strField4);                           // fourth channel returned
            break;
        }
        XSInitState=++XSInitState % UINT8_MAX;                                              // Go to the next set-up step as we have just completed one
        break;

        case XS_OK5:                                                            // Return was a +OK with 5 parameter
        XSInitState=++XSInitState % UINT8_MAX;                                              // Go to the next set-up step as we have just completed one
        break;

        case XS_OK8:                                                            // Return was 8 values plus ok
        switch (XSInitState)
        {
            case 11u:                                                           // Frame Rate request
            gXSFrameRate.Ch1 = g_okField1;                                      // first channel returned
            gXSFrameScale.Ch1 = g_okField2;                                     // second channel returned
            gXSFrameRate.Ch2 = g_okField3;                                      // third channel returned
            gXSFrameScale.Ch2 = g_okField4;                                     // fourth channel returned
            gXSFrameRate.Ch3 = g_okField5;                                      // fifth channel returned
            gXSFrameScale.Ch3 = g_okField6;                                     // sixth channel returned
            gXSFrameRate.Ch4 = g_okField7;                                      // seventh channel returned
            gXSFrameScale.Ch4 = g_okField8;                                     // eighth channel returned
            break;
        }
        XSInitState=++XSInitState % UINT8_MAX;                                  // Go to the next set-up step as we have just completed one
        break;

        case XS_DISK:                                                           // reply to a diskstatus request
        XSInitState=++XSInitState % UINT8_MAX;                                  // Go to the next set-up step as we have just completed one
        break;

        case XS_SYS:                                                            // reply to s sysstatus request
        XSInitState=++XSInitState % UINT8_MAX;                                  // Go to the next set-up step as we have just completed one
        break;
#endif
         case XS_FAIL:
         if (XSInitState==0u)                                                   // You are already logging in to it (retry)
         {
            //XSInitState=0;                                                      // Retry the failed event     (not needed just here for reading)
            XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_REPLY;                  // set alarm event wrong reply
         }
         XSLoginTime=MSG_REPOLL_TICKS;                                          // Immediatly retry response was a failure (may be bad comms)
         break;

         case XS_WRONG:
         if (XSInitState==1)                                                    // You are setting up RTSP
         {
            XSStartUpSequence = CMD_COMPLETE;                                   // Set the start-up sequence as complete
            XSInitState=2u;                                                     // issue a stop on all channels
            XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_STATE;                  // set alarm event RTSP channel in wrong state
            //XSEncoderResp = XS_RTSP_ERROR;                                      // wait for the HMI to indicate a new RTSP set-up provided
         }
         XSLoginTime=++XSLoginTime % UINT64_MAX;                                            // skip this request
         break;

         case XS_INVALID:
         if (XSInitState==0u)                                                   // You are already logging in to it
         {
            XSStartUpSequence = CMD_COMPLETE;                                   // Set the start-up sequence as complete
            XSInitState=XS_LOGIN_INVALID;                                       // Exit with an alarm state (wrong password)
            XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_USER;                   // set alarm event wrong user specified
            XSEncoderResp = XS_AUTH_WAIT;                                       // wait for the HMI to indicate a new password or user
         }
         else if (XSInitState==1u)                                              // You are setting up RTSP
         {
            XSStartUpSequence = CMD_COMPLETE;                                   // Set the start-up sequence as complete
            XSInitState=XS_RTSP_ERROR;                                          // Exit with an alarm state (unable to set-up RTSP)
            XSEncoderAlarms1.AlarmWdCurrent |= XS_RTSP_FAIL;                    // set alarm event RTSP set-up failure
            XSEncoderResp = XS_RTSP_ERROR;                                      // wait for the HMI to indicate a new RTSP set-up provided
         }
         break;

         default:
         XSLoginTime=++XSLoginTime % UINT64_MAX;                                // Counter the ticks until MSG_REPOLL_TICKS then re-trigger
         break;
       }
   }

//
// ============== COPY FROM ENCODER TO HMI (if valid) ==========================
//
hmiTimeCalcTotal.delta = 0U;                                                    // Any change makes a change
hmiDateCalcTotal.delta = 0U;                                                    // Any change makes a change

if (XSInitState==XS_STEP_SETHMI)
{
   if ((gXSContrast.Ch1 >= 0U) && (gXSContrast.Ch1 <= 10000U))                  // In range
   {
      hmiRawContrast[0u].value = gXSContrast.Ch1;                                // Set the 1st channel contrast to that of the encoder
      hmiRawContrast[0u].last = gXSContrast.Ch1;                                 // Set the 1st channel last to that of the encoder no send
      hmiRawContrast[0u].delta = 10U;                                            // Set the delta value for contrast changes
   }
   if ((gXSContrast.Ch2 >= 0U) && (gXSContrast.Ch2 <= 10000U))                  // In range
   {
      hmiRawContrast[1u].value = gXSContrast.Ch2;                                // Set the 2nd channel contrast to that of the encoder
      hmiRawContrast[1u].last = gXSContrast.Ch2;                                 // Set the 2nd channel last to that of the encoder no send
      hmiRawContrast[1u].delta = 10U;                                            // Set the delta value for contrast changes
   }
   if ((gXSContrast.Ch3 >= 0U) && (gXSContrast.Ch3 <= 10000U))                  // In range
   {
      hmiRawContrast[2u].value = gXSContrast.Ch3;                                // Set the 3rd channel contrast to that of the encoder
      hmiRawContrast[2u].last = gXSContrast.Ch3;                                 // Set the 3rd channel last to that of the encoder no send
      hmiRawContrast[2u].delta = 10U;                                            // Set the delta value for contrast changes
   }
   if ((gXSContrast.Ch4 >= 0U) && (gXSContrast.Ch4 <= 10000U))                  // In range
   {
      hmiRawContrast[3u].value = gXSContrast.Ch4;                                // Set the 4th channel contrast to that of the encoder
      hmiRawContrast[3u].last = gXSContrast.Ch4;                                 // Set the 4th channel last to that of the encoder no send
      hmiRawContrast[3u].delta = 10U;                                            // Set the delta value for contrast changes
   }
   if ((gXSBright.Ch1 >= 0U) && (gXSBright.Ch1 <= 10000U))                      // In range
   {
      hmiRawBright[0u].value = gXSBright.Ch1;                                    // Set the 1st channel brightness to that of the encoder
      hmiRawBright[0u].last = gXSBright.Ch1;                                     // Set the 1st channel last to that of the encoder no send
      hmiRawBright[0u].delta = 10U;                                              // Set the delta value for brightness changes
   }
   if ((gXSBright.Ch2 >= 0U) && (gXSBright.Ch2 <= 10000U))                      // In range
   {
      hmiRawBright[1u].value = gXSBright.Ch2;                                    // Set the 2nd channel brightness to that of the encoder
      hmiRawBright[1u].last = gXSBright.Ch2;                                     // Set the 2nd channel last to that of the encoder no send
      hmiRawBright[1u].delta = 10U;                                              // Set the delta value for brightness changes
   }
   if ((gXSBright.Ch3 >= 0U) && (gXSBright.Ch3 <= 10000U))                      // In range
   {
      hmiRawBright[2u].value = gXSBright.Ch3;                                    // Set the 3rd channel brightness to that of the encoder
      hmiRawBright[2u].last = gXSBright.Ch3;                                     // Set the 3rd channel last to that of the encoder no send
      hmiRawBright[2u].delta = 10U;                                              // Set the delta value for brightness changes
   }
   if ((gXSBright.Ch4 >= 0U) && (gXSBright.Ch4 <= 10000U))                      // In range
   {
      hmiRawBright[3u].value = gXSBright.Ch4;                                    // Set the 4th channel brightness to that of the encoder
      hmiRawBright[3u].last = gXSBright.Ch4;                                     // Set the 4th channel last to that of the encoder no send
      hmiRawBright[3u].delta = 10U;                                              // Set the delta value for brightness changes
   }
   if ((gXSHue.Ch1 >= 0U) && (gXSHue.Ch1 <= 10000U))                            // In range
   {
      hmiRawHue[0u].value = gXSHue.Ch1;                                          // Set the 1st channel hue to that of the encoder
      hmiRawHue[0u].last = gXSHue.Ch1;                                           // Set the 1st channel last to that of the encoder no send
      hmiRawHue[0u].delta = 10U;                                                 // Set the delta value for hue changes
   }
   if ((gXSHue.Ch2 >= 0U) && (gXSHue.Ch2 <= 10000U))                            // In range
   {
      hmiRawHue[1u].value = gXSHue.Ch2;                                          // Set the 2nd channel hue to that of the encoder
      hmiRawHue[1u].last = gXSHue.Ch2;                                           // Set the 2nd channel last to that of the encoder no send
      hmiRawHue[1u].delta = 10U;                                                 // Set the delta value for hue changes
   }
   if ((gXSHue.Ch3 >= 0U) && (gXSHue.Ch3 <= 10000U))                            // In range
   {
      hmiRawHue[2u].value = gXSHue.Ch3;                                          // Set the 3rd channel hue to that of the encoder
      hmiRawHue[2u].last = gXSHue.Ch3;                                           // Set the 3rd channel last to that of the encoder no send
      hmiRawHue[2u].delta = 10U;                                                 // Set the delta value for hue changes
   }
   if ((gXSHue.Ch4 >= 0U) && (gXSHue.Ch4 <= 10000U))                            // In range
   {
      hmiRawHue[3u].value = gXSHue.Ch4;                                          // Set the 4th channel hue to that of the encoder
      hmiRawHue[3u].last = gXSHue.Ch4;                                           // Set the 4th channel last to that of the encoder no send
      hmiRawHue[3u].delta = 10U;                                                 // Set the delta value for hue changes
   }
   if ((gXSSatur.Ch1 >= 0U) && (gXSSatur.Ch1 <= 10000U))                        // In range
   {
      hmiRawSaturation[0u].value = gXSSatur.Ch1;                                 // Set the 1st channel saturation to that of the encoder
      hmiRawSaturation[0u].last = gXSSatur.Ch1;                                  // Set the 1st channel last to that of the encoder no send
      hmiRawSaturation[0u].delta = 10U;                                          // Set the delta value for saturation changes
   }
   if ((gXSSatur.Ch2 >= 0U) && (gXSSatur.Ch2 <= 10000U))                        // In range
   {
      hmiRawSaturation[1u].value = gXSSatur.Ch2;                                 // Set the 2nd channel saturation to that of the encoder
      hmiRawSaturation[1u].last = gXSSatur.Ch2;                                  // Set the 2nd channel last to that of the encoder no send
      hmiRawSaturation[1u].delta = 10U;                                          // Set the delta value for saturation changes
   }
   if ((gXSSatur.Ch3 >= 0U) && (gXSSatur.Ch3 <= 10000U))                        // In range
   {
      hmiRawSaturation[2u].value = gXSSatur.Ch3;                                 // Set the 3rd channel saturation to that of the encoder
      hmiRawSaturation[2u].last = gXSSatur.Ch3;                                  // Set the 3rd channel last to that of the encoder no send
      hmiRawSaturation[2u].delta = 10U;                                          // Set the delta value for saturation changes
   }
   if ((gXSSatur.Ch4 >= 0U) && (gXSSatur.Ch4 <= 10000U))                        // In range
   {
      hmiRawSaturation[3u].value = gXSSatur.Ch4;                                 // Set the 4th channel saturation to that of the encoder
      hmiRawSaturation[3u].last = gXSSatur.Ch4;                                  // Set the 4th channel last to that of the encoder no send
      hmiRawSaturation[3u].delta = 10U;                                          // Set the delta value for saturation changes
   }
   if ((gXSClipsz.Ch1 >= 0U) && (gXSClipsz.Ch1 <= 10000U))                      // In range
   {
      hmiRawClipSz[0u].value = gXSClipsz.Ch1;                                    // Set the 1st channel clip size to that of the encoder
      hmiRawClipSz[0u].last = gXSClipsz.Ch1;                                     // Set the 1st channel last to that of the encoder no send
      hmiRawClipSz[0u].delta = 10U;                                              // Set the delta value for sclip size changes
   }
   if ((gXSClipsz.Ch2 >= 0U) && (gXSClipsz.Ch2 <= 10000U))                      // In range
   {
      hmiRawClipSz[1u].value = gXSClipsz.Ch2;                                    // Set the 2nd channel clip size to that of the encoder
      hmiRawClipSz[1u].last = gXSClipsz.Ch2;                                     // Set the 2nd channel last to that of the encoder no send
      hmiRawClipSz[1u].delta = 10U;                                              // Set the delta value for sclip size changes
   }
   if ((gXSClipsz.Ch3 >= 0U) && (gXSClipsz.Ch3 <= 10000U))                      // In range
   {
      hmiRawClipSz[2u].value = gXSClipsz.Ch3;                                    // Set the 3rd channel clip size to that of the encoder
      hmiRawClipSz[2u].last = gXSClipsz.Ch3;                                     // Set the 3rd channel last to that of the encoder no send
      hmiRawClipSz[2u].delta = 10U;                                              // Set the delta value for sclip size changes
   }
   if ((gXSClipsz.Ch4 >= 0U) && (gXSClipsz.Ch4 <= 10000U))                      // In range
   {
      hmiRawClipSz[3u].value = gXSClipsz.Ch4;                                    // Set the 4th channel clip size to that of the encoder
      hmiRawClipSz[3u].last = gXSClipsz.Ch4;                                     // Set the 4th channel last to that of the encoder no send
      hmiRawClipSz[3u].delta = 10U;                                              // Set the delta value for sclip size changes
   }
   if ((gXSClipLn.Ch1 >= 0U) && (gXSClipLn.Ch1 <= UINT16_MAX))                  // In range
   {
      hmiRawCliplen[0u].value = gXSClipLn.Ch1;                                   // Set the 1st channel clip length to that of the encoder
      hmiRawCliplen[0u].last = gXSClipLn.Ch1;                                    // Set the 1st channel last to that of the encoder no send
      hmiRawCliplen[0u].delta = 1U;                                              // Set the delta value for clip length changes
   }
   if ((gXSClipLn.Ch2 >= 0U) && (gXSClipLn.Ch2 <= UINT16_MAX))                  // In range
   {
      hmiRawCliplen[1u].value = gXSClipLn.Ch2;                                   // Set the 2nd channel clip length to that of the encoder
      hmiRawCliplen[1u].last = gXSClipLn.Ch2;                                    // Set the 2nd channel last to that of the encoder no send
      hmiRawCliplen[1u].delta = 1U;                                              // Set the delta value for clip length changes
   }
   if ((gXSClipLn.Ch3 >= 0U) && (gXSClipLn.Ch3 <= UINT16_MAX))                  // In range
   {
      hmiRawCliplen[2u].value = gXSClipLn.Ch3;                                   // Set the 3rd channel clip length to that of the encoder
      hmiRawCliplen[2u].last = gXSClipLn.Ch3;                                    // Set the 3rd channel last to that of the encoder no send
      hmiRawCliplen[2u].delta = 1U;                                              // Set the delta value for clip length changes
   }
   if ((gXSClipLn.Ch4 >= 0U) && (gXSClipLn.Ch4 <= UINT16_MAX))                  // In range
   {
      hmiRawCliplen[3u].value = gXSClipLn.Ch4;                                   // Set the 4th channel clip length to that of the encoder
      hmiRawCliplen[3u].last = gXSClipLn.Ch4;                                    // Set the 4th channel last to that of the encoder no send
      hmiRawCliplen[3u].delta = 1U;                                              // Set the delta value for clip length changes
   }
   if (((gXSFrameRate.Ch1/gXSFrameScale.Ch1) >= 0U) && ((gXSFrameRate.Ch1/gXSFrameScale.Ch1) <= MAX_FPS))  // In range with video input system
   {
      hmiRawFramert[0u].value = gXSFrameRate.Ch1;                                // Set the 1st channel frame rate to that of the encoder
      hmiRawFramert[0u].last = gXSFrameRate.Ch1;                                 // Set the 1st channel last to that of the encoder no send
      hmiRawFrameRtScalar[0u] = gXSFrameScale.Ch1;                               // Set the 1st channel scalar for the frame rate to that of the encoder
      hmiRawFramert[0u].delta = 1U;                                              // Set the delta value for frame rate changes
   }
   if (((gXSFrameRate.Ch2/gXSFrameScale.Ch2) >= 0U) && ((gXSFrameRate.Ch2/gXSFrameScale.Ch2) <= MAX_FPS))  // In range with video input system
   {
      hmiRawFramert[1u].value = gXSFrameRate.Ch1;                                // Set the 2nd channel frame rate to that of the encoder
      hmiRawFramert[1u].last = gXSFrameRate.Ch1;                                 // Set the 2nd channel last to that of the encoder no send
      hmiRawFrameRtScalar[1u] = gXSFrameScale.Ch1;                               // Set the 2nd channel scalar for the frame rate to that of the encoder
      hmiRawFramert[1u].delta = 1U;                                              // Set the delta value for frame rate changes
   }
   if (((gXSFrameRate.Ch3/gXSFrameScale.Ch3) >= 0U) && ((gXSFrameRate.Ch3/gXSFrameScale.Ch3) <= MAX_FPS))  // In range with video input system
   {
      hmiRawFramert[2u].value = gXSFrameRate.Ch1;                                // Set the 3rd channel frame rate to that of the encoder
      hmiRawFramert[2u].last = gXSFrameRate.Ch1;                                 // Set the 3rd channel last to that of the encoder no send
      hmiRawFrameRtScalar[2u] = gXSFrameScale.Ch1;                               // Set the 3rd channel scalar for the frame rate to that of the encoder
      hmiRawFramert[2u].delta = 1U;                                              // Set the delta value for frame rate changes
   }
   if (((gXSFrameRate.Ch4/gXSFrameScale.Ch4) >= 0U) && ((gXSFrameRate.Ch4/gXSFrameScale.Ch4) <= MAX_FPS))  // In range with video input system
   {
      hmiRawFramert[3u].value = gXSFrameRate.Ch1;                                // Set the 4th channel frame rate to that of the encoder
      hmiRawFramert[3u].last = gXSFrameRate.Ch1;                                 // Set the 4th channel last to that of the encoder no send
      hmiRawFrameRtScalar[3u] = gXSFrameScale.Ch1;                               // Set the 4th channel scalar for the frame rate to that of the encoder
      hmiRawFramert[3u].delta = 1U;                                              // Set the delta value for frame rate changes
   }
   if ((gXSBitRate.Ch1 >= XS_MIN_BITRATE) && (gXSBitRate.Ch1 <= XS_MAX_BITRATE))// In range with video encoder system
   {
      hmiRawBitrate[0u].value = gXSBitRate.Ch1;                                  // Set the 1st channel bit rate to that of the encoder
      hmiRawBitrate[0u].last = gXSBitRate.Ch1;                                   // Set the 1st channel last to that of the encoder no send
      hmiRawBitrate[0u].delta = 1U;                                              // Set the delta value for bit rate changes
   }
   if ((gXSBitRate.Ch2 >= XS_MIN_BITRATE) && (gXSBitRate.Ch2 <= XS_MAX_BITRATE))// In range with video encoder system
   {
      hmiRawBitrate[1u].value = gXSBitRate.Ch2;                                  // Set the 2nd channel bit rate to that of the encoder
      hmiRawBitrate[1u].last = gXSBitRate.Ch2;                                   // Set the 2nd channel last to that of the encoder no send
      hmiRawBitrate[1u].delta = 1U;                                              // Set the delta value for bit rate changes
   }
   if ((gXSBitRate.Ch3 >= XS_MIN_BITRATE) && (gXSBitRate.Ch3 <= XS_MAX_BITRATE))// In range with video encoder system
   {
      hmiRawBitrate[2u].value = gXSBitRate.Ch3;                                  // Set the 3rd channel bit rate to that of the encoder
      hmiRawBitrate[2u].last = gXSBitRate.Ch3;                                   // Set the 3rd channel last to that of the encoder no send
      hmiRawBitrate[2u].delta = 1U;                                              // Set the delta value for bit rate changes
   }
   if ((gXSBitRate.Ch4 >= XS_MIN_BITRATE) && (gXSBitRate.Ch4 <= XS_MAX_BITRATE))// In range with video encoder system
   {
      hmiRawBitrate[3u].value = gXSBitRate.Ch4;                                  // Set the 4th channel bit rate to that of the encoder
      hmiRawBitrate[3u].last = gXSBitRate.Ch4;                                   // Set the 4th channel last to that of the encoder no send
      hmiRawBitrate[3u].delta = 1U;                                              // Set the delta value for bit rate changes
   }
   if ((gXSIInt.Ch1 >= XS_MIN_IINTERVAL) && (gXSIInt.Ch1 <= XS_MAX_IINTERVAL))  // In range with video encoder system
   {
      hmiRawFrameint[0u].value = gXSIInt.Ch1;                                    // Set the 1st channel frame interval to that of the encoder
      hmiRawFrameint[0u].last = gXSIInt.Ch1;                                     // Set the 1st channel last to that of the encoder no send
      hmiRawFrameint[0u].delta = 1U;                                             // Set the delta value for frame rate changes
   }
   if ((gXSIInt.Ch2 >= XS_MIN_IINTERVAL) && (gXSIInt.Ch2 <= XS_MAX_IINTERVAL))  // In range with video encoder system
   {
      hmiRawFrameint[1u].value = gXSIInt.Ch2;                                    // Set the 2nd channel frame interval to that of the encoder
      hmiRawFrameint[1u].last = gXSIInt.Ch2;                                     // Set the 2nd channel last to that of the encoder no send
      hmiRawFrameint[1u].delta = 1U;                                             // Set the delta value for frame rate changes
   }
   if ((gXSIInt.Ch3 >= XS_MIN_IINTERVAL) && (gXSIInt.Ch3 <= XS_MAX_IINTERVAL))  // In range with video encoder system
   {
      hmiRawFrameint[2u].value = gXSIInt.Ch3;                                    // Set the 3rd channel frame interval to that of the encoder
      hmiRawFrameint[2u].last = gXSIInt.Ch3;                                     // Set the 3rd channel last to that of the encoder no send
      hmiRawFrameint[2u].delta = 1U;                                             // Set the delta value for frame rate changes
   }
   if ((gXSIInt.Ch4 >= XS_MIN_IINTERVAL) && (gXSIInt.Ch4 <= XS_MAX_IINTERVAL))  // In range with video encoder system
   {
      hmiRawFrameint[3u].value = gXSIInt.Ch4;                                    // Set the 4th channel frame interval to that of the encoder
      hmiRawFrameint[3u].last = gXSIInt.Ch4;                                     // Set the 4th channel last to that of the encoder no send
      hmiRawFrameint[3u].delta = 1U;                                             // Set the delta value for bit rate changes
   }
   XSInitState==XS_STEP_SETHMI+1u;                                              // Initialisation complete
}
#endif                                                                          // ================= eNd XS Encoder
// ================== visca ====================================================
//g_visca_p=VISCA_SCALE_TO_P_PQ(hmi_rgain);
//g_visca_q=VISCA_SCALE_TO_Q_PQ(hmi_rgain);

#if defined(SBGC_GIMBAL_JOY)
// ======================== SBGC GIMBAL ========================================
   //
   //  For VC Virtual Channel initialisation
   //
   if ((!((di2PosHalt==0u) && (di5VCMode==1u))) ||  firstTimeforVC)               // NOT in (Virtual Channel Control) or firstTimeforVC
   {
      if (((boardinforb.FIRMWARE_VER >= 2687U) && (g_boardReady)) && di6FastSelect)// version is > 2.68b7 and fast input is on
      {
        for(channels=0u; channels<SBGC_API_VIRT_NUM_CHANNELS; channels++)       // For each channel defined
        {
          vc.VCdata[channels] = SBGC_RC_HIGH_UNDEF;                             // Mark all channels as 'no signal'
        }
        onFastVCSelect=true;                                                    // fast mode is ready
      }
      else
      {
        for(channels=0u; channels<SBGC_API_VIRT_NUM_CHANNELS; channels++)       // For each channel defined
        {
          vc.VCdata[channels] = SBGC_RC_UNDEF;                                  // Mark all channels as 'no signal'
        }
        onFastVCSelect=false;                                                   // slow mode is ready
      }
      firstTimeforVC = false;                                                   // Dont do again
   }
   else if ((di2PosHalt==0u) && (di5VCMode==1u))                                  // in Virtual Channel Control and there has been a change of speed
   {
       if (!(onFastVCSelect) && (((boardinforb.FIRMWARE_VER >= 2687U) && (g_boardReady)) && di6FastSelect))  // version is > 2.68b7 and fast input is on
       {
         for(channels=0u; channels<SBGC_API_VIRT_NUM_CHANNELS; channels++)      // For each channel defined
         {
           vc.VCdata[channels] = SBGC_RC_HIGH_UNDEF;                            // Mark all channels as 'no signal'
         }
         onFastVCSelect=true;                                                   // you are in fast mode
       }
       else if (onFastVCSelect==true)                                           // you were in the fast mode
       {
          for(channels=0u; channels<SBGC_API_VIRT_NUM_CHANNELS; channels++)     // For each channel defined
          {
            vc.VCdata[channels] = SBGC_RC_UNDEF;                                // Mark all channels as 'no signal'
          }
          onFastVCSelect=false;                                                 // you are in slow mode
       }
    }                                                                           // This shall be sent when the message is formulated later
#endif

#ifdef RUN_CAM_USED
// ======================== Run Cam Camera =====================================
     // If the current command is contrast and we had a change send a request to change it
     // This message is a test stub to see if we need to convert to ascii or just send the integer value
     // To test as ascii set the define CAM_USE_ASCII
     //

     if (RCCamRequest.current == RC_PROTO_GETINFO)                              // current command that we performing
     {
        if (RCCamRequest.getinfo == CMD_SEND)                                   // We are requesting a state update from the cam
        {                                                                       // message is fixed and the CRC8_DVB has been hardcoded for speed
          totalMsgLen=sizeof(RCCamReqInfo);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamReqInfo,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container
          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_GET_DEVICE_INFO;                          // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
            RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
            udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1u) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
          //UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.getinfo == CMD_SENT;                                     // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
          RCCameraResp = RC_MSG_WAIT;
        }
        else if (RCCamRequest.getinfo == CMD_SENT)
        {
           if (RCCameraResp == RC_MSG_OK)                                       // Response came back okay
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepInfo.Header);              // Compute the CRC8 using the reply message data
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepInfo.ProtocolVersion);
              RCRepcrc=crc8_dvb_s2(RCRepcrc, (uint8_t) (RCCamRepInfo.Feature & 0xFFU));
              RCRepcrc=crc8_dvb_s2(RCRepcrc, (uint8_t) ((RCCamRepInfo.Feature >> 8) & 0xFFU));
           }
           if ((RCCamRepInfo.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK))  // feedback message check the CRC returned
           {
              RCCamRequest.getinfo= CMD_COMPLETE;
              RCRequestTimer=0;                                                 // Reset the timer
              RCCamRequest.current=RC_PROTO_WIFI;                               // Advance to the next item
           }
           else if (RCCameraResp  == RC_MSG_FAIL)                               // We got a bad reply to our send
           {
              RCCamRequest.getinfo = CMD_SEND;                                  // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.getinfo = CMD_COMPLETE;                           // Stop sending look for next request
                 RCRequestTimer=0;                                              // Reset the timer
                 RCCamRequest.current=RC_PROTO_WIFI;                            // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.getinfo = CMD_SEND;                               // Send another request
              }
           }
         }
         else if (RCCamRequest.getinfo == CMD_COMPLETE)                         // We complete so advance to the next poll element
         {
            RCRequestTimer=0U;                                                  // Reset the timer
            RCCamRequest.current=RC_PROTO_MODE;                                 // Advance to the next item
         }
     }

     // Toggle the state if the GUI request if it does not match the camera state
     //
     if (RCCamRequest.getinfo == CMD_COMPLETE)                                  // We successfully got the camera state
     {
        if (((hmiRCRequest.wifi==true && (!(RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_WIFI_BUTTON))) || (hmiRCRequest.wifi==false && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_WIFI_BUTTON))) && (RCCamRequest.wifi == CMD_COMPLETE))
        {
           RCCamRequest.wifi == CMD_SEND;                                       // Set the request to change wifi
        }
        if (((hmiRCRequest.mode==RC_MODE_VIDEO && (!(RCCamRepInfo.Feature & RC_PROTO_FEAT_CHANGE_MODE))) || (hmiRCRequest.mode==RC_MODE_PHOTO && (RCCamRepInfo.Feature & RC_PROTO_FEAT_CHANGE_MODE))) && (RCCamRequest.mode == CMD_COMPLETE))   // photo and not photo or video and not video (check reply state)
        {
           RCCamRequest.mode == CMD_SEND;                                       // Set the request to change the camera mode
        }
        if (((hmiRCRequest.mode==RC_MODE_OSD && (!(RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))) || (!(hmiRCRequest.mode==RC_MODE_OSD) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))) && (RCCamRequest.mode == CMD_COMPLETE))  // go to osd or out of osd
        {
           RCCamRequest.mode == CMD_SEND;                                       // Set the request to change the camera mode
        }
        if (((hmiRCRequest.record==true && (!(RCCamRepInfo.Feature & RC_PROTO_FEAT_START_RECORDING))) || (hmiRCRequest.record==false && (RCCamRepInfo.Feature & RC_PROTO_FEAT_START_RECORDING))) && (RCCamRequest.record == CMD_COMPLETE)) // request to record or take a picture
        {
           RCCamRequest.record == CMD_SEND;                                     // Set the request to start recording to memory
        }
        if (((hmiRCRequest.stop==true && (!(RCCamRepInfo.Feature & RC_PROTO_FEAT_STOP_RECORDING))) || (hmiRCRequest.stop==false && (RCCamRepInfo.Feature & RC_PROTO_FEAT_STOP_RECORDING))) && (RCCamRequest.stop == CMD_COMPLETE))  // request to stop record
        {
           RCCamRequest.stop == CMD_SEND;                                       // Set the request to start recording to memory
        }
        if (hmiRCRequest.up==true && (RCCamRequest.up == CMD_COMPLETE))
        {
           if (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE)   // We are in OSD mode
           {
              RCCamRequest.up == CMD_SEND;                                      // Set the request to go up
              hmiRCRequest.up==false;
           }
        }
        if (hmiRCRequest.down==true && (RCCamRequest.down == CMD_COMPLETE))
        {
           if (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE)   // We are in OSD mode
           {
              RCCamRequest.down == CMD_SEND;                                    // Set the request to go down
              hmiRCRequest.down==false;
           }
           else
           {
              RCCamRequest.mode == CMD_SEND;                                    // Set the request to go to osd mode
              hmiRCRequest.mode==RC_MODE_OSD;                                   // Request OSD mode
           }
        }
        if (hmiRCRequest.right==true && (RCCamRequest.right == CMD_COMPLETE))
        {
           if (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE)   // We are in OSD mode
           {
              RCCamRequest.right == CMD_SEND;                                   // Set the request to go right
              hmiRCRequest.right==false;
           }
           else
           {
              RCCamRequest.mode == CMD_SEND;                                    // Set the request to go to osd mode
              hmiRCRequest.mode==RC_MODE_OSD;                                   // Request OSD mode
           }
        }
        if (hmiRCRequest.left==true && (RCCamRequest.left == CMD_COMPLETE))
        {
           if (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE)   // We are in OSD mode
           {
              RCCamRequest.left == CMD_SEND;                                    // Set the request to go up
              hmiRCRequest.left==false;
           }
           else
           {
              RCCamRequest.mode == CMD_SEND;                                    // Set the request to go to osd mode
              hmiRCRequest.mode==RC_MODE_OSD;                                   // Request OSD mode
           }
        }
        if (hmiRCRequest.confirm==true && (RCCamRequest.confirm == CMD_COMPLETE))
        {
           if (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE)   // We are in OSD mode
           {
              RCCamRequest.confirm == CMD_SEND;                                 // Set the request to confirm (accept menu option pointed to)
              hmiRCRequest.confirm==false;
           }
           else
           {
              RCCamRequest.mode == CMD_SEND;                                    // Set the request to go to osd mode
              hmiRCRequest.mode==RC_MODE_OSD;                                   // Request OSD mode
           }
        }
     }

     if ((RCtimedPolling % RC_POLL_MODULUS)==0U)
     {
       RCCamRequest.ds_rectime == CMD_SEND;                                     // periodic request for recording time left
     }
     else if (((RCtimedPolling-RC_POLL_SLOT) % RC_POLL_MODULUS)==0U)
     {
       RCCamRequest.ds_memory == CMD_SEND;                                      // periodic request for disk space left
     }
     RCRequestTimer=++RCRequestTimer % UINT32_MAX;                              // Increment the poll counter

     if ((strcmp(commGSRCCamMode,"PAL") || strcmp(commGSRCCamMode,"NTSC")) && (!strcmp(commGSRCCamMode,commGSRCCamModeLast)))
     {
        RCCamRequest.ds_ntsc_pal = CMD_SEND;                                    // Send a request to find out what mode we have
        strcpy(commGSRCCamMode,commGSRCCamModeLast);                            // Disable call until next string change
     }

     if ((((RCCamRequest.stop == CMD_SENT) || (RCCamRequest.record == CMD_SENT)) || (RCCamRequest.wifi == CMD_SENT))  && (RCCamRequest.getinfo == CMD_COMPLETE))
     {
        RCCamRequest.getinfo = CMD_SEND;                                        // If we requested a change then after it read it back to clear the request of changed state
     }

     switch (RCCamRequest.current)                                              // current command that we performing
     {
        case RC_PROTO_MODE:
        if (RCCamRequest.mode == CMD_SEND)                                      // We got a recording stop request and we are the current item in the poll list sequence
        {
           if (!(hmiRCRequest.mode==RC_MODE_OSD)&& !(RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))   // Rquest was to toggle the picture / video mode
           {
              // Camera control command
              RCCamControl.CommandID = RC_PROTO_CMD_CAMERA_CONTROL;             // Command key actions Change picture mode
              RCCamControl.Action=RC_PROTO_FEAT_CHANGE_MODE;                    // Set the feature to stop recording
              g_RunCamSent = RC_PROTO_CMD_CAMERA_CONTROL;                       // Set the global send message so we know what to expect back

           }
           else if ((hmiRCRequest.mode==RC_MODE_OSD) && !(RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // Rquest was to toggle the osd mode
           {
              // Simulate handshake/disconnection command
              RCCamControl.CommandID = RC_PROTO_CMD_5KEY_CONNECTION;            // Command key actions OSD
              RCCamControl.Action = RC_PROTO_5KEY_FUNCTION_OPEN;                // Open the connection OSD mode ?
              g_RunCamSent = RC_PROTO_CMD_5KEY_CONNECTION;                      // Set the global send message so we know what to expect back
           }
           else if (!(hmiRCRequest.mode==RC_MODE_OSD) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))   // Rquest was to go to picture or video but first we must exit osd
           {
              // Simulate handshake/disconnection command
              RCCamControl.CommandID = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;    // Command key actions OSD
              RCCamControl.Action = RC_PROTO_5KEY_FUNCTION_CLOSE;               // Close the connection OSD mode ?
              g_RunCamSent = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;              // Set the global send message so we know what to expect back
           }
           RCcrc=0;                                                             // initialise CRC
           RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                       // Compute the CRC8 using the message data
           RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
           RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
           RCCamControl.Crc8 = RCcrc;
           totalMsgLen=sizeof(RCCamControl);                                    // Calculate the message length for the data portion of the UDP datagram
           memcpy(RCBuffer,&RCCamControl,totalMsgLen);                          // Copy to the UDP send Buffer the conte4nts of the request information container

           udpSent = 0U;
           while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
           {
              RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
              udpSent=++udpSent % UINT8_MAX;
           }
           RCRequestUDP = UDP_SEND;                                             // Reset the value returned from the send function
           if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;       // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
           memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));   // Null terminate the Buffer for serial
           //UART5_write_text(RCBuffer);                                          // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
           RCCamRequest.mode == CMD_SENT;                                       // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
           RCCameraResp = RC_MSG_WAIT;                                          // wait for good reply
        }
        else if ((RCCamRequest.mode == CMD_SENT) && (g_RunCamSent == RC_PROTO_CMD_CAMERA_CONTROL))  // One whole cycle later
        {                                                                       // There is no feedback message so just go to send the next one the state will re-send if not met
           RCCamRequest.mode= CMD_COMPLETE;
           RCRequestTimer=0U;                                                   // Reset the timer
           RCCamRequest.current=RC_PROTO_WIFI;                                  // Advance to the start to check the status
        }
        else if ((RCCamRequest.mode == CMD_SENT) && !(g_RunCamSent == RC_PROTO_CMD_CAMERA_CONTROL))  // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamHandshakeConfirm.Header);     // Compute the CRC8 using the reply message data
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamHandshakeConfirm.Response);
           }
           if ((RCCamHandshakeConfirm.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK))// feedback message check the CRC returned
           {
              if (RC_HANDSHAKE_OK(RCCamHandshakeConfirm.Response,RCCamControl.Action)==1) // right command and success
              {
                 RCCamRequest.mode= CMD_COMPLETE;
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_WIFI;                            // Advance to the start to check the status
              }
              else
              {
                 RCCamRequest.mode = CMD_SEND;                                  // Resend the command failed
                 RCRequestTimer=0U;                                             // Reset the timer
              }
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamHandshakeConfirm.Crc8 != RCRepcrc))
           {
              RCCamRequest.up = CMD_SEND;                                       // Send another request
              RCRequestTimer=0U;                                                // Reset the timer
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.mode = CMD_COMPLETE;                              // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_WIFI;                            // Advance to release operation sub state
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.mode = CMD_SEND;                                  // Send another request
              }
           }
        }
        else if (RCCamRequest.mode == CMD_COMPLETE)
        {
           RCRequestTimer=0U;                                                   // Reset the timer
           RCCamRequest.current=RC_PROTO_WIFI;                                  // Advance to the start to check the status
        }
        break;

        case  RC_PROTO_WIFI:
        if (RCCamRequest.wifi == CMD_SEND)                                       // We got a wifi change request and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_CAMERA_CONTROL;                 // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_FEAT_SIMULATE_WIFI_BUTTON;               // Set the feature to toggle the Wifi
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_CAMERA_CONTROL;                           // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
              RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
              udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
          //UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART5
          }
#endif
          RCCamRequest.wifi == CMD_SENT;                                        // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
        }
        else if (RCCamRequest.wifi == CMD_SENT)                                 // One whole cycle later
        {                                                                       // There is no feedback message so just go to send the next one the state will re-send if not met
            RCCamRequest.wifi= CMD_COMPLETE;
            RCRequestTimer=0U;                                                  // Reset the timer
            RCCamRequest.current=RC_PROTO_RECORD;                               // Advance to the next item
        }
        else if (RCCamRequest.wifi == CMD_COMPLETE)                             // One whole cycle later
        {                                                                       // There is no feedback message so just go to send the next one the state will re-send if not met
            RCRequestTimer=0U;                                                  // Reset the timer
            RCCamRequest.current=RC_PROTO_RECORD;                               // Advance to the next item
        }
        break;

        case RC_PROTO_RECORD:
        if ((RCCamRequest.record == CMD_SEND) && !(RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))   // We got request to record to disk and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_CAMERA_CONTROL;                 // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_FEAT_START_RECORDING;                    // Set the feature to start recording
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          RCcrc=0U;                                                             // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_CAMERA_CONTROL;                           // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
            RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
            udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
          UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
#endif
          RCCamRequest.record == CMD_SENT;                                      // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
        }
        else if (RCCamRequest.record == CMD_SENT)                               // One whole cycle later
        {                                                                       // There is no feedback message so just go to send the next one the state will re-send if not met
            RCCamRequest.record= CMD_COMPLETE;
            RCRequestTimer=0U;                                                  // Reset the timer
            RCCamRequest.current=RC_PROTO_STOP;                                 // Advance to the next item
        }
        else if (RCCamRequest.record == CMD_COMPLETE)                           // One whole cycle later
        {                                                                       // There is no feedback message so just go to send the next one the state will re-send if not met
            RCRequestTimer=0U;                                                  // Reset the timer
            RCCamRequest.current=RC_PROTO_STOP;                                 // Advance to the next item
        }
        break;

        case RC_PROTO_STOP:
        if ((RCCamRequest.stop == CMD_SEND) && !(RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // We got a recording stop request and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_CAMERA_CONTROL;                 // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_FEAT_STOP_RECORDING;                     // Set the feature to stop recording
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_CAMERA_CONTROL;                           // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
          //UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.stop == CMD_SENT;                                        // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
        }
        else if (RCCamRequest.stop == CMD_SENT)                                 // One whole cycle later
        {                                                                       // There is no feedback message so just go to send the next one the state will re-send if not met
              RCCamRequest.stop= CMD_COMPLETE;
              RCRequestTimer=0U;                                                // Reset the timer
              RCCamRequest.current=RC_PROTO_UP;                                 // Advance to the next item
        }
        else if (RCCamRequest.stop == CMD_COMPLETE)                             // One whole cycle later
        {                                                                       // There is no feedback message so just go to send the next one the state will re-send if not met
              RCRequestTimer=0U;                                                // Reset the timer
              RCCamRequest.current=RC_PROTO_UP;                                 // Advance to the next item
        }
        break;

        case RC_PROTO_UP:
        if ((RCCamRequest.up == CMD_SEND) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // We got a up request and are in osd and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_5KEY_SIMULATION_PRESS;          // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_5KEY_SIMULATION_UP;                      // Set the feature to key up
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_5KEY_SIMULATION_PRESS;                    // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
          //UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.up == CMD_SENT;                                          // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
          RCCameraResp = RC_MSG_WAIT;                                           // wait for good reply
        }
        else if (RCCamRequest.up == CMD_SENT)                                   // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamKeyConfirm.Header);           // Compute the CRC8 using the reply message data (consider hardcoding for speed)
           }
           if ((RCCamKeyConfirm.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK))  // feedback message check the CRC returned
           {
              RCCamRequest.up= CMD_SEND;
              RCRequestTimer=0u;                                                // Reset the timer
              RCCamRequest.current=RC_PROTO_UP_REL;                             // Advance to release operation sub state
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamKeyConfirm.Crc8 != RCRepcrc))
           {
              RCCamRequest.up = CMD_SEND;                                       // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.up = CMD_COMPLETE;                                // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_UP_REL;                          // Advance to release operation sub state
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.up = CMD_SEND;                                    // Send another request
              }
           }
        }
        else if (RCCamRequest.up == CMD_COMPLETE)
        {                                                                       // There is no feedback message so just go to send the next one the state will re-send if not met
           RCRequestTimer=0u;                                                   // Reset the timer
           RCCamRequest.current=RC_PROTO_DOWN;                                  // Advance to the next item
        }
        break;

        case RC_PROTO_UP_REL:
        if ((RCCamRequest.up == CMD_SEND) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // We got a up request and are in osd and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;        // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_5KEY_SIMULATION_UP;                      // Set the feature to key up
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;                  // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
          //UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.up == CMD_SENT;                                          // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
          RCCameraResp = RC_MSG_WAIT;                                           // wait for good reply
        }
        else if (RCCamRequest.up == CMD_SENT)                                   // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamKeyConfirm.Header);           // Compute the CRC8 using the reply message data (consider hardcoding for speed)
           }
           if ((RCCamKeyConfirm.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK))  // feedback message check the CRC returned
           {
              RCCamRequest.up= CMD_COMPLETE;
              RCRequestTimer=0u;                                                // Reset the timer
              RCCamRequest.current=RC_PROTO_DOWN;                               // Advance to the next item
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamKeyConfirm.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.up = CMD_SEND;                                       // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.up = CMD_COMPLETE;                                // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_DOWN;                            // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.up = CMD_SEND;                                    // Send another request
              }
           }
        }
        else if (RCCamRequest.up == CMD_COMPLETE)
        {                                                                       // There is no feedback message so just go to send the next one the state will re-send if not met
           RCRequestTimer=0u;                                                   // Reset the timer
           RCCamRequest.current=RC_PROTO_DOWN;                                  // Advance to the next item
        }
        break;

        case RC_PROTO_DOWN:
        if ((RCCamRequest.down == CMD_SEND) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // We got a up request and are in osd and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_5KEY_SIMULATION_PRESS;          // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_5KEY_SIMULATION_DOWN;                    // Set the feature to key down
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_5KEY_SIMULATION_PRESS;                    // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
          //UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.down == CMD_SENT;                                        // Advance to sent mode
          RCCameraResp = RC_MSG_WAIT;
        }
        else if (RCCamRequest.down == CMD_SENT)                                 // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamKeyConfirm.Header);           // Compute the CRC8 using the reply message data (consider hardcoding for speed)
           }
           if ((RCCamKeyConfirm.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK))  // feedback message check the CRC returned
           {
              RCCamRequest.down= CMD_SEND;
              RCRequestTimer=0u;                                                // Reset the timer
              RCCamRequest.current=RC_PROTO_DOWN_REL;                           // Advance to release the down press
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamKeyConfirm.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.down = CMD_SEND;                                       // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.down = CMD_COMPLETE;                              // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_DOWN_REL;                        // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.down = CMD_SEND;                                  // Send another request
              }
           }
        }
        else if (RCCamRequest.down == CMD_COMPLETE)
        {
           RCRequestTimer=0u;                                                   // Reset the timer
           RCCamRequest.current=RC_PROTO_RIGHT;                                 // Advance to the next item
        }
        break;

        case RC_PROTO_DOWN_REL:
        if ((RCCamRequest.down == CMD_SEND) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // We got a up request and are in osd and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;        // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_5KEY_SIMULATION_DOWN;                    // Set the feature to key down
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;                  // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
         // UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.down == CMD_SENT;                                        // Advance to sent mode
          RCCameraResp = RC_MSG_WAIT;
        }
        else if (RCCamRequest.down == CMD_SENT)                                 // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamKeyConfirm.Header);           // Compute the CRC8 using the reply message data (consider hardcoding for speed)
           }
           if ((RCCamKeyConfirm.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK))  // feedback message check the CRC returned
           {
              RCCamRequest.down= CMD_COMPLETE;
              RCRequestTimer=0u;                                                // Reset the timer
              RCCamRequest.current=RC_PROTO_RIGHT;                              // Advance to the next item
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamKeyConfirm.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.down = CMD_SEND;                                       // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.down = CMD_COMPLETE;                              // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_RIGHT;                           // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.down = CMD_SEND;                                  // Send another request
              }
           }
        }
        else if (RCCamRequest.down == CMD_COMPLETE)
        {
           RCRequestTimer=0u;                                                   // Reset the timer
           RCCamRequest.current=RC_PROTO_RIGHT;                                 // Advance to the next item
        }
        break;

        case RC_PROTO_RIGHT:
        if ((RCCamRequest.down == CMD_SEND) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // We got a up request and are in osd and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_5KEY_SIMULATION_PRESS;          // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_5KEY_SIMULATION_RIGHT;                   // Set the feature to key right
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_5KEY_SIMULATION_PRESS;                    // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
          //UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.right == CMD_SENT;                                       // Advance to sent mode
          RCCameraResp == RC_MSG_WAIT;                                          // Wait for a good reply
        }
        else if (RCCamRequest.right == CMD_SENT)                                // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamKeyConfirm.Header);           // Compute the CRC8 using the reply message data (consider hardcoding for speed)
           }
            if ((RCCamKeyConfirm.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK)) // feedback message check the CRC returned
            {
               RCCamRequest.right= CMD_SEND;
               RCRequestTimer=0U;                                               // Reset the timer
               RCCamRequest.current=RC_PROTO_RIGHT_REL;                         // Advance to the release state for this button
            }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamKeyConfirm.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.right = CMD_SEND;                                       // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.right = CMD_COMPLETE;                             // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_RIGHT_REL;                       // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.right = CMD_SEND;                                 // Send another request
              }
           }
        }
        else if (RCCamRequest.right == CMD_COMPLETE)
        {
            RCRequestTimer=0U;                                                  // Reset the timer
            RCCamRequest.current=RC_PROTO_LEFT;                                 // Advance to the next item
        }
        break;

        case RC_PROTO_RIGHT_REL:
        if ((RCCamRequest.down == CMD_SEND) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // We got a up request and are in osd and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;        // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_5KEY_SIMULATION_RIGHT;                   // Set the feature to key right
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;                  // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
          //UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.right == CMD_SENT;                                       // Advance to sent mode
          RCCameraResp == RC_MSG_WAIT;                                          // Wait for a good reply
        }
        else if (RCCamRequest.right == CMD_SENT)                                // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamKeyConfirm.Header);           // Compute the CRC8 using the reply message data (consider hardcoding for speed)
           }
            if ((RCCamKeyConfirm.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK)) // feedback message check the CRC returned
            {
               RCCamRequest.right= CMD_COMPLETE;
               RCRequestTimer=0U;                                               // Reset the timer
               RCCamRequest.current=RC_PROTO_LEFT;                              // Advance to the next item
            }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamKeyConfirm.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.right = CMD_SEND;                                       // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.right = CMD_COMPLETE;                             // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_LEFT;                            // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.right = CMD_SEND;                                 // Send another request
              }
           }
        }
        else if (RCCamRequest.right == CMD_COMPLETE)
        {
            RCRequestTimer=0U;                                                  // Reset the timer
            RCCamRequest.current=RC_PROTO_LEFT;                                 // Advance to the next item
        }
        break;

        case RC_PROTO_LEFT:
        if ((RCCamRequest.down == CMD_SEND) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // We got a up request and are in osd and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_5KEY_SIMULATION_PRESS;          // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_5KEY_SIMULATION_LEFT;                    // Set the feature to key left
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_5KEY_SIMULATION_PRESS;                    // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
         // UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.left == CMD_SENT;                                        // Advance to sent mode
          RCCameraResp = RC_MSG_WAIT;                                           // Wait til we get a good response
        }
        else if (RCCamRequest.left == CMD_SENT)                                 // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamKeyConfirm.Header);           // Compute the CRC8 using the reply message data (consider hardcoding for speed)
           }
           if ((RCCamKeyConfirm.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK))  // feedback message check the CRC returned
           {
              RCCamRequest.left= CMD_SEND;                                      // send again a release now
              RCRequestTimer=0U;                                                // Reset the timer
              RCCamRequest.current=RC_PROTO_LEFT_REL;                           // Advance to the release of the left key
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamKeyConfirm.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.left = CMD_SEND;                                       // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.left = CMD_COMPLETE;                              // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_LEFT_REL;                        // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.left = CMD_SEND;                                  // Send another request
              }
           }
        }
        else if (RCCamRequest.left == CMD_COMPLETE)
        {
           RCRequestTimer=0U;                                                   // Reset the timer
           RCCamRequest.current=RC_PROTO_CONFIRM ;                              // Advance to the next item
        }
        break;

        case RC_PROTO_LEFT_REL:
        if ((RCCamRequest.down == CMD_SEND) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // We got a up request and are in osd and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;        // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_5KEY_SIMULATION_LEFT;                    // Set the feature to key left
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;                  // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
          //UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.left == CMD_SENT;                                        // Advance to sent mode
          RCCameraResp = RC_MSG_WAIT;                                           // Wait til we get a good response
        }
        else if (RCCamRequest.left == CMD_SENT)                                 // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamKeyConfirm.Header);           // Compute the CRC8 using the reply message data (consider hardcoding for speed)
           }
           if ((RCCamKeyConfirm.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK))  // feedback message check the CRC returned
           {
              RCCamRequest.left= CMD_COMPLETE;
              RCRequestTimer=0U;                                                // Reset the timer
              RCCamRequest.current=RC_PROTO_CONFIRM;                            // Advance to the next item
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamKeyConfirm.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.left = CMD_SEND;                                       // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.left = CMD_COMPLETE;                              // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_CONFIRM;                         // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.left = CMD_SEND;                                  // Send another request
              }
           }
        }
        else if (RCCamRequest.left == CMD_COMPLETE)
        {
           RCRequestTimer=0U;                                                   // Reset the timer
           RCCamRequest.current=RC_PROTO_CONFIRM ;                              // Advance to the next item
        }
        break;

        case RC_PROTO_CONFIRM:
        if ((RCCamRequest.confirm == CMD_SEND) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // We got a up request and are in osd and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_5KEY_SIMULATION_PRESS;          // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_5KEY_SIMULATION_SET;                     // Set the feature to confirm the selection
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_5KEY_SIMULATION_PRESS;                    // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
          //UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.confirm == CMD_SENT;                                     // Advance to sent mode
          RCCameraResp = RC_MSG_WAIT;                                           // Wait til we get a good response
        }
        else if (RCCamRequest.confirm == CMD_SENT)                              // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamKeyConfirm.Header);           // Compute the CRC8 using the reply message data (consider hardcoding for speed)
           }
           if ((RCCamKeyConfirm.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK))  // feedback message check the CRC returned
           {
              RCCamRequest.confirm= CMD_SEND;
              RCRequestTimer=0U;                                                // Reset the timer
              RCCamRequest.current=RC_PROTO_CONFIRM_REL;                        // Advance to the confirm release step
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamKeyConfirm.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.confirm = CMD_SEND;                                  // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.confirm = CMD_COMPLETE;                           // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_CONFIRM_REL;                     // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.confirm = CMD_SEND;                                  // Send another request
              }
           }
        }
        else if (RCCamRequest.confirm == CMD_COMPLETE)
        {
            RCRequestTimer=0U;                                                  // Reset the timer
            RCCamRequest.current=RC_PROTO_DS_MEMORY;                            // Advance to the next item
        }
        break;

        case RC_PROTO_CONFIRM_REL:
        if ((RCCamRequest.confirm == CMD_SEND) && (RCCamRepInfo.Feature & RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE))  // We got a up request and are in osd and we are the current item in the poll list sequence
        {
          RCCamControl.CommandID = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;        // Command key actions Change picture mode
          RCCamControl.Action=RC_PROTO_5KEY_SIMULATION_SET;                     // Set the feature to confirm the selection
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamControl.Action);
          RCCamControl.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamControl);                                     // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamControl,totalMsgLen);                           // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_5KEY_SIMULATION_RELEASE;                  // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
//          UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.confirm == CMD_SENT;                                     // Advance to sent mode
          RCCameraResp = RC_MSG_WAIT;                                           // Wait til we get a good response
        }
        else if (RCCamRequest.confirm == CMD_SENT)                              // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamKeyConfirm.Header);           // Compute the CRC8 using the reply message data (consider hardcoding for speed)
           }
           if ((RCCamKeyConfirm.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK)) // feedback message check the CRC returned
           {
              RCCamRequest.confirm= CMD_COMPLETE;
              RCRequestTimer=0U;                                                // Reset the timer
              RCCamRequest.current=RC_PROTO_DS_MEMORY;                          // Advance to the next item
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamKeyConfirm.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.confirm = CMD_SEND;                                       // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.confirm = CMD_COMPLETE;                           // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_DS_MEMORY;                       // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.confirm = CMD_SEND;                               // Send another request
              }
           }
        }
        else if (RCCamRequest.confirm == CMD_COMPLETE)
        {
            RCRequestTimer=0U;                                                  // Reset the timer
            RCCamRequest.current=RC_PROTO_DS_MEMORY;                            // Advance to the next item
        }
        break;

        case RC_PROTO_DS_MEMORY:
        if (RCCamRequest.ds_memory == CMD_SEND)                                 // We polling a request for memory
        {
          RCCamReqReadSetup.CommandID = RC_PROTO_CMD_GET_SETTINGS;              // Command get settings
          RCCamReqReadSetup.Setting = RC_PROTO_SETTINGID_DISP_SDCARD_CAPACIT;   // get the memory capacity
          RCCamReqReadSetup.ChunkIdx = 0U;                                      // Chunk index is 0 ????
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.Header);                   // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.Setting);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.ChunkIdx);
          RCCamReqReadSetup.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamReqReadSetup);                                // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamReqReadSetup,totalMsgLen);                      // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_GET_SETTINGS;                             // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
//          UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.ds_memory == CMD_SENT;                                   // Advance to sent mode
          RCCameraResp = RC_MSG_WAIT;                                           // Wait til we get a good response
        }
        else if (RCCamRequest.ds_memory == CMD_SENT)                            // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.RemainingChunk); // Compute the CRC8 using the reply message data (consider hardcoding for speed)
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.DataLength);
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.SettingType);
              switch(RCCamRepReadSetup.SettingType)
              {
                   case RC_PROTO_TYPE_UINT8:                                    // retruned type was a uint8_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.Value.Uint8);
                   break;

                   case RC_PROTO_TYPE_INT8:                                     // retruned type was a int8_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.Value.Int8);
                   break;

                   case RC_PROTO_TYPE_UINT16:                                   // retruned type was a uint16_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) (RCCamRepReadSetup.Value.Uint16 & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) ((RCCamRepReadSetup.Value.Uint16>>8) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_INT16:                                    // retruned type was a int16_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) (RCCamRepReadSetup.Value.Int16 & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) ((RCCamRepReadSetup.Value.Int16>>8) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_FLOAT:                                    // retruned type was a float32_t (done longhand)
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,((uint8_t) (RCCamRepReadSetup.Value.Float32) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>8U) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>16U) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>24U) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_TEXT_SELECTION:                           // retruned type was a text
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,RCCamRepReadSetup.Value.TextSelection,strlen(RCCamRepReadSetup.Value.TextSelection));
                   break;

                   case RC_PROTO_TYPE_STRING:                                   // retruned type was a uint16_t (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.StringSz,sizeof(RCCamRepReadSetup.Value.StringSz));
                   break;

                   case RC_PROTO_TYPE_FOLDER:                                   // retruned type was a folder (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Settings,sizeof(RCCamRepReadSetup.Value.Settings));
                   break;

                   case RC_PROTO_TYPE_INFO:                                     // retruned type was an information reply (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Info,sizeof(RCCamRepReadSetup.Value.Info));
                   break;

                   case RC_PROTO_TYPE_COMMAND:
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Command,sizeof(RCCamRepReadSetup.Value.Command));
                   break;

                   default:
                   break;
              }

              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MinValue,sizeof(RCCamRepReadSetup.MinValue));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MaxValue,sizeof(RCCamRepReadSetup.MaxValue));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.DecPoint,sizeof(RCCamRepReadSetup.DecPoint));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.StepSize,sizeof(RCCamRepReadSetup.StepSize));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MaxStringSize,sizeof(RCCamRepReadSetup.MaxStringSize));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,RCCamRepReadSetup.TextSelections,strlen(RCCamRepReadSetup.TextSelections));
           }
           if ((RCCamRepReadSetup.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK)) // feedback message check the CRC returned
           {
              RCCamRequest.ds_memory= CMD_COMPLETE;
              RCRequestTimer=0u;                                                // Reset the timer
              switch(RCCamRepReadSetup.SettingType)                             // Setting type returns tells us what typw it is
              {                                                                 // We are not sure of the return type for memory hence read it into a union
                 case RC_PROTO_TYPE_UINT8:
                 // RCCamRepReadSetup.Value.Uint8
                 break;

                 case RC_PROTO_TYPE_INT8:
                 // RCCamRepReadSetup.Value.Int8;
                 break;

                 case RC_PROTO_TYPE_UINT16:
                 // RCCamRepReadSetup.Value.Uint16;                             // Read this if its not a string as suggested
                 break;

                 case RC_PROTO_TYPE_INT16:
                 // RCCamRepReadSetup.Value.Int16;
                 break;

                 case RC_PROTO_TYPE_FLOAT:
                 // RCCamRepReadSetup.Value.Float32                             // Read this if its not a string as suggested
                 break;

                 case RC_PROTO_TYPE_TEXT_SELECTION:
                 strcpy(hmiRCCamMemory,RCCamRepReadSetup.Value.TextSelection);  // Read the memory from the structure union
                 break;

                 case RC_PROTO_TYPE_STRING:
                 // RCCamRepReadSetup.Value.StringSz
                 strcpy(hmiRCCamMemory,RCCamRepReadSetup.TextSelections);       // Read the memory from the structure union
                 break;

                 case RC_PROTO_TYPE_FOLDER:
                 // RCCamRepReadSetup.Value.Settings
                 break;

                 case RC_PROTO_TYPE_INFO:
                 // RCCamRepReadSetup.Value.Info
                 break;

                 case RC_PROTO_TYPE_COMMAND:
                 // RCCamRepReadSetup.Value.Command

                 default:
                 break;

              }
              RCCamRequest.current=RC_PROTO_DS_RECTIME;                         // Advance to the next item
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamRepReadSetup.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.ds_memory = CMD_SEND;                                // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.ds_memory = CMD_COMPLETE;                         // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_DS_RECTIME;                      // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.ds_memory = CMD_SEND;                               // Send another request
              }
           }
        }
        else if (RCCamRequest.ds_memory == CMD_COMPLETE)
        {
           RCCamRequest.current=RC_PROTO_DS_RECTIME;                            // Advance to the next item
        }
        break;

        case RC_PROTO_DS_RECTIME:
        if (RCCamRequest.ds_rectime == CMD_SEND)                                // We polling a request for reamining recording time on the disk
        {
          RCCamReqReadSetup.CommandID = RC_PROTO_CMD_GET_SETTINGS;              // Command get settings
          RCCamReqReadSetup.Setting = RC_PROTO_SETTINGID_DISP_REMAIN_RECORDING_TIME;   // get the recording time left on the disk
          RCCamReqReadSetup.ChunkIdx = 0U;                                      // Chunk index is 0 ????
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.Header);                        // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.Setting);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.ChunkIdx);
          RCCamReqReadSetup.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamReqReadSetup);                                // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamReqReadSetup,totalMsgLen);                      // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_GET_SETTINGS;                             // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
//          UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.ds_rectime == CMD_SENT;                                  // Advance to sent mode
          RCCameraResp = RC_MSG_WAIT;                                           // Wait til we get a good response
        }
        else if (RCCamRequest.ds_rectime == CMD_SENT)                           // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.RemainingChunk); // Compute the CRC8 using the reply message data (consider hardcoding for speed)
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.DataLength);
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.SettingType);
              switch(RCCamRepReadSetup.SettingType)
              {
                   case RC_PROTO_TYPE_UINT8:                                    // retruned type was a uint8_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.Value.Uint8);
                   break;

                   case RC_PROTO_TYPE_INT8:                                     // retruned type was a int8_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.Value.Int8);
                   break;

                   case RC_PROTO_TYPE_UINT16:                                   // retruned type was a uint16_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) (RCCamRepReadSetup.Value.Uint16 & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) ((RCCamRepReadSetup.Value.Uint16>>8) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_INT16:                                    // retruned type was a int16_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) (RCCamRepReadSetup.Value.Int16 & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) ((RCCamRepReadSetup.Value.Int16>>8) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_FLOAT:                                    // retruned type was a float32_t (done longhand)
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,((uint8_t) (RCCamRepReadSetup.Value.Float32) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>8U) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>16U) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>24U) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_TEXT_SELECTION:                           // retruned type was a text
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,RCCamRepReadSetup.Value.TextSelection,strlen(RCCamRepReadSetup.Value.TextSelection));
                   break;

                   case RC_PROTO_TYPE_STRING:                                   // retruned type was a uint16_t (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.StringSz,sizeof(RCCamRepReadSetup.Value.StringSz));
                   break;

                   case RC_PROTO_TYPE_FOLDER:                                   // retruned type was a folder (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Settings,sizeof(RCCamRepReadSetup.Value.Settings));
                   break;

                   case RC_PROTO_TYPE_INFO:                                     // retruned type was an information reply (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Info,sizeof(RCCamRepReadSetup.Value.Info));
                   break;

                   case RC_PROTO_TYPE_COMMAND:
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Command,sizeof(RCCamRepReadSetup.Value.Command));
                   break;

                   default:
                   break;
              }

              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MinValue,sizeof(RCCamRepReadSetup.MinValue));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MaxValue,sizeof(RCCamRepReadSetup.MaxValue));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.DecPoint,sizeof(RCCamRepReadSetup.DecPoint));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.StepSize,sizeof(RCCamRepReadSetup.StepSize));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MaxStringSize,sizeof(RCCamRepReadSetup.MaxStringSize));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,RCCamRepReadSetup.TextSelections,strlen(RCCamRepReadSetup.TextSelections));
           }
           if ((RCCamRepReadSetup.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK))// feedback message check the CRC returned
           {
              RCCamRequest.ds_rectime= CMD_COMPLETE;
              RCRequestTimer=0u;                                                // Reset the timer
              switch(RCCamRepReadSetup.SettingType)                             // Setting type returns tells us what typw it is
              {                                                                 // We are not sure of the return type for memory hence read it into a union
                 case RC_PROTO_TYPE_UINT8:
                 // RCCamRepReadSetup.Value.Uint8
                 break;

                 case RC_PROTO_TYPE_INT8:
                 // RCCamRepReadSetup.Value.Int8;
                 break;

                 case RC_PROTO_TYPE_UINT16:
                 // RCCamRepReadSetup.Value.Uint16;                             // Read this if its not a string as suggested
                 break;

                 case RC_PROTO_TYPE_INT16:
                 // RCCamRepReadSetup.Value.Int16;
                 break;

                 case RC_PROTO_TYPE_FLOAT:
                 // RCCamRepReadSetup.Value.Float32                             // Read this if its not a string as suggested
                 break;

                 case RC_PROTO_TYPE_TEXT_SELECTION:
                 strcpy(hmiRCCamTimeLeft,RCCamRepReadSetup.Value.TextSelection);// Read the time left from the structure union
                 break;

                 case RC_PROTO_TYPE_STRING:
                 // RCCamRepReadSetup.Value.StringSz
                 strcpy(hmiRCCamTimeLeft,RCCamRepReadSetup.TextSelections);     // Read the time left from the structure union
                 break;

                 case RC_PROTO_TYPE_FOLDER:
                 // RCCamRepReadSetup.Value.Settings
                 break;

                 case RC_PROTO_TYPE_INFO:
                 // RCCamRepReadSetup.Value.Info
                 break;

                 case RC_PROTO_TYPE_COMMAND:
                 // RCCamRepReadSetup.Value.Command

                 default:
                 break;

              }
              RCCamRequest.current=RC_PROTO_DS_NTSC_PAL;                        // Advance to the next item
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamRepReadSetup.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.ds_rectime = CMD_SEND;                                // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.ds_rectime = CMD_COMPLETE;                         // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_DS_NTSC_PAL;                      // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.ds_rectime = CMD_SEND;                            // Send another request
              }
           }
        }
        else if (RCCamRequest.ds_rectime == CMD_SENT)
        {
           RCCamRequest.current=RC_PROTO_DS_NTSC_PAL;                           // Advance to the next item
        }
        break;

        case RC_PROTO_DS_NTSC_PAL:
        if (RCCamRequest.ds_ntsc_pal == CMD_SEND)                               // We polling a request for reamining recording time on the disk
        {
          RCCamReqReadSetup.CommandID = RC_PROTO_CMD_GET_SETTINGS;              // Command get settings
          RCCamReqReadSetup.Setting = RC_PROTO_SETTINGID_DISP_TV_MODE;          // get the recording time left on the disk
          RCCamReqReadSetup.ChunkIdx = 0U;                                      // Chunk index is 0 ????
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.Header);                   // Compute the CRC8 using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.Setting);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqReadSetup.ChunkIdx);
          RCCamReqReadSetup.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamReqReadSetup);                                // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamReqReadSetup,totalMsgLen);                      // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_GET_SETTINGS;                             // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
//          UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.ds_ntsc_pal == CMD_SENT;                                 // Advance to sent mode
          RCCameraResp = RC_MSG_WAIT;                                           // Wait til we get a good response
        }
        else if (RCCamRequest.ds_ntsc_pal == CMD_SENT)                          // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.RemainingChunk); // Compute the CRC8 using the reply message data (consider hardcoding for speed)
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.DataLength);
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.SettingType);
              switch(RCCamRepReadSetup.SettingType)
              {
                   case RC_PROTO_TYPE_UINT8:                                    // retruned type was a uint8_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.Value.Uint8);
                   break;

                   case RC_PROTO_TYPE_INT8:                                     // retruned type was a int8_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.Value.Int8);
                   break;

                   case RC_PROTO_TYPE_UINT16:                                   // retruned type was a uint16_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) (RCCamRepReadSetup.Value.Uint16 & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) ((RCCamRepReadSetup.Value.Uint16>>8) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_INT16:                                    // retruned type was a int16_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) (RCCamRepReadSetup.Value.Int16 & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) ((RCCamRepReadSetup.Value.Int16>>8) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_FLOAT:                                    // retruned type was a float32_t (done longhand)
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,((uint8_t) (RCCamRepReadSetup.Value.Float32) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>8U) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>16U) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>24U) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_TEXT_SELECTION:                           // retruned type was a text
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,RCCamRepReadSetup.Value.TextSelection,strlen(RCCamRepReadSetup.Value.TextSelection));
                   break;

                   case RC_PROTO_TYPE_STRING:                                   // retruned type was a uint16_t (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.StringSz,sizeof(RCCamRepReadSetup.Value.StringSz));
                   break;

                   case RC_PROTO_TYPE_FOLDER:                                   // retruned type was a folder (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Settings,sizeof(RCCamRepReadSetup.Value.Settings));
                   break;

                   case RC_PROTO_TYPE_INFO:                                     // retruned type was an information reply (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Info,sizeof(RCCamRepReadSetup.Value.Info));
                   break;

                   case RC_PROTO_TYPE_COMMAND:
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Command,sizeof(RCCamRepReadSetup.Value.Command));
                   break;

                   default:
                   break;
              }

              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MinValue,sizeof(RCCamRepReadSetup.MinValue));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MaxValue,sizeof(RCCamRepReadSetup.MaxValue));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.DecPoint,sizeof(RCCamRepReadSetup.DecPoint));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.StepSize,sizeof(RCCamRepReadSetup.StepSize));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MaxStringSize,sizeof(RCCamRepReadSetup.MaxStringSize));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,RCCamRepReadSetup.TextSelections,strlen(RCCamRepReadSetup.TextSelections));
           }
           if ((RCCamRepReadSetup.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK))// feedback message check the CRC returned
           {
              RCCamRequest.ds_ntsc_pal= CMD_COMPLETE;
              RCRequestTimer=0u;                                                // Reset the timer
              switch(RCCamRepReadSetup.SettingType)                             // Setting type returns tells us what typw it is
              {                                                                 // We are not sure of the return type for memory hence read it into a union
                 case RC_PROTO_TYPE_UINT8:
                 // RCCamRepReadSetup.Value.Uint8
                 break;

                 case RC_PROTO_TYPE_INT8:
                 // RCCamRepReadSetup.Value.Int8;
                 break;

                 case RC_PROTO_TYPE_UINT16:
                 // RCCamRepReadSetup.Value.Uint16;                             // Read this if its not a string as suggested
                 break;

                 case RC_PROTO_TYPE_INT16:
                 // RCCamRepReadSetup.Value.Int16;
                 break;

                 case RC_PROTO_TYPE_FLOAT:
                 // RCCamRepReadSetup.Value.Float32                             // Read this if its not a string as suggested
                 break;

                 case RC_PROTO_TYPE_TEXT_SELECTION:
                 strcpy(hmiRCCamNtscPal,RCCamRepReadSetup.Value.TextSelection); // Read if the camera is NTSC or PAL
                 if (strcmp(hmiRCCamNtscPal,commGSRCCamMode))                   // The mode is as requested
                 {
                    RCCamRequest.current=RC_PROTO_GETINFO;                      // Advance to the start to check the status
                 }
                 else
                 {
                    RCCamRequest.ds_ntsc_pal == CMD_SEND;
                    RCCamRequest.current=RC_PROTO_CHG_MODE;                     // Change the camera mode
                 }
                 break;

                 case RC_PROTO_TYPE_STRING:
                 // RCCamRepReadSetup.Value.StringSz
                 strcpy(hmiRCCamNtscPal,(char *) &RCCamRepReadSetup.Value.StringSz);      // Read if the camera is NTSC or PAL
                 if (strcmp(hmiRCCamNtscPal,commGSRCCamMode))                   // The mode is as requested
                 {
                    RCCamRequest.current=RC_PROTO_GETINFO;                      // Advance to the start to check the status
                 }
                 else
                 {
                    RCCamRequest.ds_ntsc_pal == CMD_SEND;
                    RCCamRequest.current=RC_PROTO_CHG_MODE;                     // Change the camera mode
                 }
                 break;

                 case RC_PROTO_TYPE_FOLDER:
                 // RCCamRepReadSetup.Value.Settings
                 break;

                 case RC_PROTO_TYPE_INFO:
                 // RCCamRepReadSetup.Value.Info
                 break;

                 case RC_PROTO_TYPE_COMMAND:
                 // RCCamRepReadSetup.Value.Command

                 default:
                 break;

              }
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamRepReadSetup.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.ds_ntsc_pal = CMD_SEND;                              // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.ds_ntsc_pal = CMD_COMPLETE;                       // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_GETINFO;                         // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.ds_ntsc_pal = CMD_SEND;                           // Send another request
              }
           }
        }
        else if (RCCamRequest.ds_ntsc_pal == CMD_COMPLETE)
        {
           RCCamRequest.current=RC_PROTO_GETINFO;                               // Advance to the start to check the Status
        }
        break;
     //}

        case RC_PROTO_CHG_MODE:
        if (RCCamRequest.ds_ntsc_pal == CMD_SEND)                               // We polling a request for reamining recording time on the disk
        {
          RCCamReqWriteSetup.CommandID = RC_PROTO_CMD_WRITE_SETTING;            // Command write settings
          RCCamReqWriteSetup.Setting = RC_PROTO_SETTINGID_DISP_TV_MODE;         // change the ntsc pal setting
          if (strcmp(commGSRCCamMode,"PAL"))                                    // Selection was for PAL
          {
              RCCamReqWriteSetup.Value.Uint8 = RC_PROTO_WANT_PAL;               // we want PAL
          }
          else
          {
              RCCamReqWriteSetup.Value.Uint8 = RC_PROTO_WANT_NTSC;              // we want NTSC
          }
          RCcrc=0u;                                                             // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqWriteSetup.Header);                  // Compute the CRC8 DVB using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqWriteSetup.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqWriteSetup.Setting);
          RCcrc=crc8_dvb_s2(RCcrc, RCCamReqWriteSetup.Value.Uint8);
          RCCamReqWriteSetup.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCCamReqWriteSetup.Header)+sizeof(RCCamReqWriteSetup.CommandID)+sizeof(RCCamReqWriteSetup.Setting)+sizeof(RCCamReqWriteSetup.Value.Uint8);                                // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCCamReqWriteSetup,totalMsgLen);                     // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_WRITE_SETTING;                            // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
//          UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.ds_ntsc_pal == CMD_SENT;                                 // Advance to sent mode
          RCCameraResp = RC_MSG_WAIT;                                           // Wait til we get a good response
        }
        else if (RCCamRequest.ds_ntsc_pal == CMD_SENT)                          // One whole cycle later
        {
           if (RCCameraResp == RC_MSG_OK)                                       // A good message came back chack the CRC8 DVB checksum
           {
              RCRepcrc=0U;
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.RemainingChunk); // Compute the CRC8 using the reply message data (consider hardcoding for speed)
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.DataLength);
              RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.SettingType);
              switch(RCCamRepReadSetup.SettingType)
              {
                   case RC_PROTO_TYPE_UINT8:                                    // retruned type was a uint8_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.Value.Uint8);
                   break;

                   case RC_PROTO_TYPE_INT8:                                     // retruned type was a int8_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc, RCCamRepReadSetup.Value.Int8);
                   break;

                   case RC_PROTO_TYPE_UINT16:                                   // retruned type was a uint16_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) (RCCamRepReadSetup.Value.Uint16 & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) ((RCCamRepReadSetup.Value.Uint16>>8) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_INT16:                                    // retruned type was a int16_t
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) (RCCamRepReadSetup.Value.Int16 & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(uint8_t) ((RCCamRepReadSetup.Value.Int16>>8) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_FLOAT:                                    // retruned type was a float32_t (done longhand)
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,((uint8_t) (RCCamRepReadSetup.Value.Float32) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>8U) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>16U) & 0xFFU));
                   RCRepcrc=crc8_dvb_s2(RCRepcrc,(((uint8_t) (RCCamRepReadSetup.Value.Float32)>>24U) & 0xFFU));
                   break;

                   case RC_PROTO_TYPE_TEXT_SELECTION:                           // retruned type was a text
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,RCCamRepReadSetup.Value.TextSelection,strlen(RCCamRepReadSetup.Value.TextSelection));
                   break;

                   case RC_PROTO_TYPE_STRING:                                   // retruned type was a uint16_t (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.StringSz,sizeof(RCCamRepReadSetup.Value.StringSz));
                   break;

                   case RC_PROTO_TYPE_FOLDER:                                   // retruned type was a folder (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Settings,sizeof(RCCamRepReadSetup.Value.Settings));
                   break;

                   case RC_PROTO_TYPE_INFO:                                     // retruned type was an information reply (might a string ?)
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Info,sizeof(RCCamRepReadSetup.Value.Info));
                   break;

                   case RC_PROTO_TYPE_COMMAND:
                   RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.Value.Command,sizeof(RCCamRepReadSetup.Value.Command));
                   break;

                   default:
                   break;
              }

              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MinValue,sizeof(RCCamRepReadSetup.MinValue));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MaxValue,sizeof(RCCamRepReadSetup.MaxValue));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.DecPoint,sizeof(RCCamRepReadSetup.DecPoint));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.StepSize,sizeof(RCCamRepReadSetup.StepSize));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,&RCCamRepReadSetup.MaxStringSize,sizeof(RCCamRepReadSetup.MaxStringSize));
              RCRepcrc=crc8_dvb_s2_update(RCRepcrc,RCCamRepReadSetup.TextSelections,strlen(RCCamRepReadSetup.TextSelections));
           }
           if ((RCCamRepWriteSetup.Crc8 == RCRepcrc) && (RCCameraResp == RC_MSG_OK)) // feedback message check the CRC returned
           {
              RCCamRequest.ds_ntsc_pal= CMD_COMPLETE;
              RCRequestTimer=0u;                                                // Reset the timer
              RCCamRequest.current=RC_PROTO_WRITE_TXT;                          // Advance back to the start to check the status
           }
           else if ((RCCameraResp  == RC_MSG_FAIL) || (RCCamRepReadSetup.Crc8 != RCRepcrc))  // We got a bad reply to our send
           {
              RCCamRequest.ds_ntsc_pal = CMD_SEND;                              // Send another request
           }
           else
           {
              RCRequestTimer=++RCRequestTimer % UINT32_MAX;
              if (RCRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
                 RCCamRequest.ds_ntsc_pal = CMD_COMPLETE;                       // Stop sending look for next request
                 RCRequestTimer=0U;                                             // Reset the timer
                 RCCamRequest.current=RC_PROTO_WRITE_TXT;                         // Advance to the next item
              }
              else if (RCRequestTimer > (MSG_REPOLL_TICKS/2U))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 RCCamRequest.ds_ntsc_pal = CMD_SEND;                           // Send another request
              }
           }
        }
        else if (RCCamRequest.ds_ntsc_pal == CMD_COMPLETE)
        {
           RCCamRequest.current=RC_PROTO_WRITE_TXT;                             // Advance to the start to check the Status
        }
        break;

        case RC_PROTO_WRITE_TXT:                                                // Write a string horizontally to the screen
        if (RCCamRequest.write_txt == CMD_SEND)                                 // We requesting to write a new string
        {
          RCText2Screen.CommandID = RC_PROTO_CMD_DISP_WRITE_HORT_STRING;        // Command write settings
          RCText2Screen.X = 0x05U;                                              // x=5 horizontal position on the video monitor
          RCText2Screen.Y = 0x05U;                                              // y=5 vertical position on the video monitor
          RCText2Screen.MsgLength = strlen(hmiHorizText);
          if (RCText2Screen.MsgLength <= sizeof(RCText2Screen.MsgToWrite))
          strcpy(RCText2Screen.MsgToWrite,hmiHorizText);
          RCcrc=0;                                                              // initialise CRC
          RCcrc=crc8_dvb_s2(RCcrc, RCText2Screen.Header);                       // Compute the CRC8 DVB using the message data
          RCcrc=crc8_dvb_s2(RCcrc, RCText2Screen.CommandID);
          RCcrc=crc8_dvb_s2(RCcrc, RCText2Screen.X);
          RCcrc=crc8_dvb_s2(RCcrc, RCText2Screen.Y);
          RCcrc=crc8_dvb_s2(RCcrc, RCText2Screen.MsgLength);
          RCcrc=crc8_dvb_s2_update(RCcrc, &RCText2Screen.MsgToWrite, (uint32_t) RCText2Screen.MsgLength);
          RCText2Screen.Crc8 = RCcrc;
          totalMsgLen=sizeof(RCText2Screen.Header)+sizeof(RCText2Screen.CommandID)+sizeof(RCText2Screen.X)+sizeof(RCText2Screen.Y)+sizeof(RCText2Screen.MsgLength)+RCText2Screen.MsgLength;                                // Calculate the message length for the data portion of the UDP datagram
          memcpy(RCBuffer,&RCText2Screen,totalMsgLen);                          // Copy to the UDP send Buffer the conte4nts of the request information container

          udpSent = 0U;
          g_RunCamSent = RC_PROTO_CMD_DISP_WRITE_HORT_STRING;                   // Set the global send message so we know what to expect back
          while ((RCRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))  // send the Buffer contents as UDP datagram.
          {
             RCRequestUDP = Net_Ethernet_Intern_sendUDP(RunC_DATA.RemoteIpAddr, RC_From_Port, RC_Dest_Port, RCBuffer, totalMsgLen);
             udpSent=++udpSent % UINT8_MAX;
          }
          RCRequestUDP = UDP_SEND;                                              // Reset the value returned from the send function
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART5_INTERUPT
          memset((void *) RCBuffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
//          UART5_write_text(RCBuffer);                                           // Send it to serial UART 5
          for(RunCcnt=0;RunCcnt<totalMsgLen;RunCcnt++)
          {
             UART5_write(RCBuffer[RunCcnt]);                                      // Send it to serial UART2
          }
#endif
          RCCamRequest.write_txt == CMD_SENT;                                   // Advance to sent mode
          RCCameraResp = RC_MSG_WAIT;                                           // Wait til we get a good response
        }
        else if (RCCamRequest.write_txt == CMD_SENT)                            // One whole cycle later
        {

            RCCamRequest.write_txt= CMD_COMPLETE;
            RCRequestTimer=0;                                                   // Reset the timer
            RCCamRequest.current=RC_PROTO_GETINFO;                              // Advance back to the start to check the status
        }
        else if (RCCamRequest.write_txt == CMD_COMPLETE)
        {
           RCCamRequest.current=RC_PROTO_GETINFO;                               // Advance to the start to check the Status
        }
        break;
     }                                                                          // End of main state engine case
#endif                                                                          // ===================== eNd of Run Cam requests ===========================

#if (CAMERA_TYPE == XS_CAM)
// ======================= XS ENCODER ==========================================

   // Create a queue of messages to send to the encoder based on delta values
   // (hmi movement of the slider)
   // We have to queue them as the encoder only replies with +OK per message
   // so we cant identify completion in multiple sends (one at a time only)
   // Modified so that queue is a FIFO and the channel number always sequences 1,2,3,4
   // Always picks next one
   //
   if (XS_encoder_Q.contrast == CMD_COMPLETE)                                   // The last contrast message completed
   {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC                                           // Continually send it until its sent a successful feedback
      hmiRawContrast[XScontrastChan].last = hmiRawContrast[XScontrastChan].value; // Take the delta away only after a success
#endif
      XScontrastChan = ++XScontrastChan % 5U;                                   // Count XScontrastChan in range 1 to 4
      if (XScontrastChan==0) XScontrastChan=1;
      if (XSChangeDetected(&hmiRawContrast[XScontrastChan])==1)                 // HMI Contrast change has been made
      {                                                                         // Data Change after complete
         XS_encoder_Q.contrast == CMD_SEND;                                     // Set the request to change the contrast
#if XS_DELTA_MODE == XS_TRY_TWICE                                               // Send message twice
         hmiRawContrast[XScontrastChan].last = hmiRawContrast[XScontrastChan].value; // Save the last HMI change we latched the request for it
#endif
      }
   }
   else if (XS_encoder_Q.contrast == CMD_RESEND)
   {
      XS_encoder_Q.contrast == CMD_SEND;                                        // keep on the same channel and re-send it
   }

   if (XS_encoder_Q.hue == CMD_COMPLETE)
   {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
         hmiRawHue[XShueChan].last = hmiRawHue[XShueChan].value;                // Save the last HMI change we latched the request for it
#endif
      XShueChan = ++XShueChan % 5U;                                             // Count in range 1 to 4
      if (XShueChan==0) XShueChan=1;
      if (XSChangeDetected(&hmiRawHue[XShueChan]) )
      {                                                                         // Data Change after complete
         XS_encoder_Q.hue == CMD_SEND;                                          // Set the request to change the hue
#if XS_DELTA_MODE == XS_TRY_TWICE
         hmiRawHue[XShueChan].last = hmiRawHue[XShueChan].value;                // Save the last HMI change we latched the request for it
#endif
      }
   }
   else if (XS_encoder_Q.hue == CMD_RESEND)
   {
      XS_encoder_Q.hue == CMD_SEND;                                             // keep on the same channel and re-send it
   }

   if (XS_encoder_Q.saturation == CMD_COMPLETE)
   {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC                                           /* in this mode continues until command has fed back completion */
         hmiRawSaturation[XSsaturationChan].last = hmiRawSaturation[XSsaturationChan].value;        // Save the last HMI change we latched the request for it
#endif
      XSsaturationChan = (XSsaturationChan++ % 5U);                             // Count in range 1 to 4
      if (XSsaturationChan==0) XSsaturationChan=1;
      if (XSChangeDetected(&hmiRawSaturation[XSsaturationChan]))
      {                                                                         // Data Change after complete
         XS_encoder_Q.saturation == CMD_SEND;                                   // Set the request to change the saturation
#if XS_DELTA_MODE == XS_TRY_TWICE                                               /* in this mode we cancel the HMI trigger once we send a request */
         hmiRawSaturation[XSsaturationChan].last = hmiRawSaturation[XSsaturationChan].value;        // Save the last HMI change we latched the request for it
#endif
      }
   }
   else if (XS_encoder_Q.saturation == CMD_RESEND)
   {
      XS_encoder_Q.saturation == CMD_SEND;                                      // keep on the same channel and re-send it
   }

   if (XS_encoder_Q.clipsz == CMD_COMPLETE)
   {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
         hmiRawClipSz[XSclipszChan].last = hmiRawClipSz[XSclipszChan].value;    // Save the last HMI change we latched the request for it
#endif
      XSclipszChan = (XSclipszChan++ % 5U);                                     // Count in range 1 to 4
      if (XSclipszChan==0) XSclipszChan=1;
      if (XSChangeDetected(&hmiRawClipSz[XSclipszChan]))
      {                                                                         // Data Change after complete
         XS_encoder_Q.clipsz == CMD_SEND;                                       // Set the request to change the clip size
#if XS_DELTA_MODE == XS_TRY_TWICE
         hmiRawClipSz[XSclipszChan].last = hmiRawClipsz[XSclipszChan].value;    // Save the last HMI change we latched the request for it
#endif
      }
   }
   else if (XS_encoder_Q.clipsz == CMD_RESEND)
   {
      XS_encoder_Q.clipsz == CMD_SEND;                                          // keep on the same channel and re-send it
   }

   if (XS_encoder_Q.cliplen == CMD_COMPLETE)
   {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
         hmiRawCliplen[XScliplenChan].last = hmiRawCliplen[XScliplenChan].value;// Save the last HMI change we latched the request for it
#endif
      XScliplenChan = (XScliplenChan++ % 5U);                                    // Count in range 1 to 4
      if (XScliplenChan==0) XScliplenChan=1;
      if (XSChangeDetected(&hmiRawCliplen[XScliplenChan]))
      {                                                                         // Data Change after complete
         XS_encoder_Q.cliplen == CMD_SEND;                                      // Set the request to change the clip length
#if XS_DELTA_MODE == XS_TRY_TWICE
         hmiRawCliplen[XScliplenChan].last = hmiRawCliplen[XScliplenChan].value;// Save the last HMI change we latched the request for it
#endif
      }
   }
   else if (XS_encoder_Q.cliplen == CMD_RESEND)
   {
      XS_encoder_Q.cliplen == CMD_SEND;                                          // keep on the same channel and re-send it
   }

   if (XS_encoder_Q.bright == CMD_COMPLETE)
   {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
         hmiRawBright[XSbrightChan].last = hmiRawBright[XSbrightChan].value;    // Save the last HMI change we latched the request for it
#endif
      XSbrightChan = (XSbrightChan++ % 5U);                                     // Count in range 1 to 4
      if (XSbrightChan==0) XSbrightChan=1;
      if (XSChangeDetected(&hmiRawBright[XSbrightChan]) )
      {                                                                         // Data Change after complete
         XS_encoder_Q.bright == CMD_SEND;                                       // Set the request to change the brightness
#if XS_DELTA_MODE == XS_TRY_TWICE
         hmiRawBright[XSbrightChan].last = hmiRawBright[XSbrightChan].value;    // Save the last HMI change we latched the request for it
#endif
      }
   }
   else if (XS_encoder_Q.bright == CMD_RESEND)
   {
      XS_encoder_Q.bright == CMD_SEND;                                          // keep on the same channel and re-send it
   }

   if (XS_encoder_Q.framert == CMD_COMPLETE)
   {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
         hmiRawFramert[XSframeRateChan].last = hmiRawFramert[XSframeRateChan].value; // Save the last HMI change we latched the request for it
#endif
      XSframeRateChan = (XSframeRateChan++ % 5U);                               // Count  in range 1 to 4
      if (XSframeRateChan==0) XSframeRateChan=1U;
      if (XSChangeDetected(&hmiRawFramert[XSframeRateChan]))
      {                                                                         // Data Change after complete
         XS_encoder_Q.framert == CMD_SEND;                                      // Set the request to change the frame rate
#if XS_DELTA_MODE == XS_TRY_TWICE
         hmiRawFramert[XSframeRateChan].last = hmiRawFramert[XSframeRateChan].value; // Save the last HMI change we latched the request for it
#endif
      }
   }
   else if (XS_encoder_Q.framert == CMD_RESEND)
   {
      XS_encoder_Q.framert == CMD_SEND;                                         // keep on the same channel and re-send it
   }

   if (XS_encoder_Q.frameint == CMD_COMPLETE)
   {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
         hmiRawFrameint[XSframeIntChan].last = hmiRawFrameint[XSframeIntChan].value; // Save the last HMI change we latched the request for it
#endif
      XSframeIntChan = (XSframeIntChan++ % 5U);                                 // Count  in range 1 to 4
      if (XSframeIntChan==0) XSframeIntChan=1;
      if (XSChangeDetected(&hmiRawFrameint[XSframeIntChan]))
      {                                                                         // Data Change after complete
         XS_encoder_Q.frameint == CMD_SEND;                                     // Set the request to change the frame interval
#if XS_DELTA_MODE == XS_TRY_TWICE
         hmiRawFrameint[XSframeIntChan].last = hmiRawFrameint[XSframeIntChan].value; // Save the last HMI change we latched the request for it
#endif
      }
   }
   else if (XS_encoder_Q.frameint == CMD_RESEND)
   {
      XS_encoder_Q.frameint == CMD_SEND;                                        // keep on the same channel and re-send it
   }

   if (XS_encoder_Q.bitrate == CMD_COMPLETE)
   {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
         hmiRawBitrate[XSbitRateChan].last = hmiRawBitrate[XSbitRateChan].value; // Save the last HMI change we latched the request for it
#endif
      XSbitRateChan = (XSbitRateChan++ % 5U);                                   // Count  in range 1 to 4
      if (XSbitRateChan==0) XSbitRateChan=1;
      if (XSChangeDetected(&hmiRawBitrate[XSbitRateChan]))
      {                                                                         // Data Change after complete
         XS_encoder_Q.bitrate == CMD_SEND;                                      // Set the request to change the bit rate
#if XS_DELTA_MODE == XS_TRY_TWICE
         hmiRawBitrate[XSbitRateChan].last = hmiRawBitrate[XSbitRateChan].value; // Save the last HMI change we latched the request for it
#endif
      }
   }
   else if (XS_encoder_Q.bitrate == CMD_RESEND)
   {
      XS_encoder_Q.bitrate == CMD_SEND;                                         // keep on the same channel and re-send it
   }

     // If the current command is contrast and we had a change send a request to change it
     // This message is a test stub to see if we need to convert to ascii or just send the integer value
     // To test as ascii set the define CAM_USE_ASCII
     //
     if ((XS_encoder_Q.current == XS_CONTRAST)&&(XS_encoder_Q.loggedin==true))  // current command and authorised
     {
        if (XS_encoder_Q.contrast == CMD_SEND)                                  // We got a contrast change and we are the current item in the poll list sequence
        {
#ifndef CAM_USE_ASCII                                                           // The define decides if we are using pot switch or HMI
          XStreamContrast.Contrast= hmiRawContrast[XScontrastChan].value;       // Set the byte in the message to the scaled contrast value equal to the raw hmi contrast slider
          XStreamContrast.Channel=XScontrastChan;
#else
          Number2String(XScontrastChan,&XStreamContrast.asciiChannel,2);
          Number2String(hmiRawContrast[XScontrastChan].value,&XStreamContrast.asciiContrast,5);   // EXAMPLE if we need to convert value to ascii (5 chars long) compare with a strcpy if needed
#endif
          memcpy(Buffer,&XStreamContrast,sizeof(XStreamContrast));              // Copy to the UDP send Buffer the conte4nts of the contrast container
          totalMsgLen=sizeof(XStreamContrast);                                  // Calculate the message length for the data portion of the UDP datagram
          udpSent = 0;
          while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {
            XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, sizeof(XStreamContrast));        // send the Buffer contents as UDP datagram.
            udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
          XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
             memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));   // Null terminate the Buffer for serial
             UART4_write_text(Buffer);                                          // Send it to serial UART4
#endif
          XS_encoder_Q.contrast == CMD_SENT;                                    // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
        }
        else if (XS_encoder_Q.contrast == CMD_SENT)
        {
           if (XSEncoderResp  == XS_OK)                                         // The encoder response message was +OK
           {
              XS_encoder_Q.contrast= CMD_COMPLETE;
              XSRequestTimer=0u;                                                 // Reset the timer
              XS_encoder_Q.current=XS_BRIGHTNESS;                               // Advance to the next item BRIGHTNESS
           }
           else
           {
              XSRequestTimer=++XSRequestTimer % UINT32_MAX;
              if (XSRequestTimer > MSG_REPOLL_TICKS)                            // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
              {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
                 XS_encoder_Q.contrast = CMD_RESEND;                            // Stop sending look for next request but retry afterwards
#else
                 XS_encoder_Q.contrast = CMD_COMPLETE;                          // Stop sending look for next request
#endif
                 XSRequestTimer=0;                                              // Reset the timer
                 XS_encoder_Q.current=XS_BRIGHTNESS;                            // Advance to the next item BRIGHTNESS
              }
              else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                  // We are half way to timing out so send it again. (incase we missed the okay)
              {
                 XS_encoder_Q.contrast = CMD_SEND;                              // Send another request
              }
           }
         }
         else if (XS_encoder_Q.contrast == CMD_COMPLETE)                        // We complete so advance to the next poll element
         {
            XSRequestTimer=0u;                                                  // Reset the timer
            XS_encoder_Q.current=XS_BRIGHTNESS;                                 // Advance to the next item BRIGHTNESS
         }
       }

       // If the current command is brightness and we had a change send a request to change it
       //
      if ((XS_encoder_Q.current == XS_BRIGHTNESS)&&(XS_encoder_Q.loggedin==true))  // current command and authorised
      {
         if (XS_encoder_Q.bright == CMD_SEND)                                   // We got a brightness change and we are the current item in the poll list sequence
         {
#ifndef CAM_USE_ASCII                                                           // The define decides if we are using pot switch or HMI
              XStreamBrightness.Brightness= hmiRawBright[XSbrightChan].value;   // Set the byte in the message to the scaled contrast value equal to the raw hmi contrast slider
              XStreamBrightness.Channel=XSbrightChan;
#else
              Number2String(XSbrightChan,&XStreamBrightness.asciiChannel,2);
              Number2String(hmiRawBright[XSbrightChan].value,&XStreamBrightness.asciiBrightness,5);   // EXAMPLE if we need to convert value to ascii (5 chars long) compare with a strcpy if needed
#endif
             memcpy(Buffer,&XStreamBrightness,sizeof(XStreamBrightness));       // Copy to the UDP send Buffer the conte4nts of the brightness container
             totalMsgLen=sizeof(XStreamBrightness);                             // Calculate the message length for the data portion of the UDP datagram
             udpSent = 0u;
             while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
             {
                XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, sizeof(XStreamBrightness));        // send the Buffer contents as UDP datagram.
                udpSent=++udpSent % UINT8_MAX;
             }
             if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;     // accumulate the bad send counter (remove the 1 before summing)
             XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
             memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));   // Null terminate the Buffer for serial
             UART4_write_text(Buffer);                                          // Send it to serial UART4
#endif
             XS_encoder_Q.bright == CMD_SENT;                                   // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
         }
         else if (XS_encoder_Q.bright == CMD_SENT)
         {
            if (XSEncoderResp  == XS_OK)                                        // The encoder response message was +OK
            {
               XS_encoder_Q.bright= CMD_COMPLETE;
               XSRequestTimer=0U;                                               // Reset the timer
               XS_encoder_Q.current=XS_HUE;                                     // Advance to the next item HUE
            }
            else
            {
               XSRequestTimer=++XSRequestTimer % UINT32_MAX;
               if (XSRequestTimer > MSG_REPOLL_TICKS)                           // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
               {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
                  XS_encoder_Q.bright = CMD_RESEND;                             // Stop sending look for next request but come back
#else
                  XS_encoder_Q.bright = CMD_COMPLETE;                           // Stop sending look for next request
#endif
                  XSRequestTimer=0U;                                            // Reset the timer
                  XS_encoder_Q.current=XS_HUE;                                   // Advance to the next item HUE
               }
               else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                 // We are half way to timing out so send it again. (incase we missed the okay)
               {
                  XS_encoder_Q.bright = CMD_SEND;                               // Send another request
               }
            }
         }
         else if (XS_encoder_Q.bright == CMD_COMPLETE)                          // We complete so advance to the next poll element
         {
            XSRequestTimer=0u;                                                  // Reset the timer
            XS_encoder_Q.current=XS_HUE;                                        // Advance to the next item HUE
         }
      }

      // If the current command is hue and we had a change send a request to change it
      //
      if ((XS_encoder_Q.current == XS_HUE)&&(XS_encoder_Q.loggedin==true))      // current command and authorised
      {
         if (XS_encoder_Q.hue == CMD_SEND)                                      // We got a hue change and we are the current item in the poll list sequence
         {
#ifndef CAM_USE_ASCII                                                           // The define decides if we are using pot switch or HMI
            XStreamHue.hue= hmiRawBright[XShueChan].value;                      // Set the byte in the message to the scaled contrast value equal to the raw hmi contrast slider
            XStreamHue.Channel=XShueChan;
#else
            Number2String(XShueChan,&XStreamHue.asciiChannel,2U);
            Number2String(hmiRawHue[XShueChan].value,&XStreamHue.asciiHue,5U);  // EXAMPLE if we need to convert value to ascii (5 chars long) compare with a strcpy if needed
#endif
           memcpy(Buffer,&XStreamHue,sizeof(XStreamHue));                       // Copy to the UDP send Buffer the conte4nts of the hue container
           totalMsgLen=sizeof(XStreamHue);                                      // Calculate the message length for the data portion of the UDP datagram
           udpSent = 0U;
           while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
           {
              XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, sizeof(XStreamHue));        // send the Buffer contents as UDP datagram.
              udpSent=++udpSent % UINT8_MAX;
           }
           if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;       // accumulate the bad send counter (remove the 1 before summing)
           XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
           memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));     // Null terminate the Buffer for serial
           UART4_write_text(Buffer);                                            // Send it to serial UART4
#endif
           XS_encoder_Q.hue == CMD_SENT;                                        // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
         }
         else if (XS_encoder_Q.hue == CMD_SENT)
         {
           if (XSEncoderResp  == XS_OK)                                         // The encoder response message was +OK
           {
               XS_encoder_Q.hue= CMD_COMPLETE;
               XSRequestTimer=0U;                                               // Reset the timer
               XS_encoder_Q.current=XS_SATURATION;                              // Advance to the next item
           }
           else
           {
               XSRequestTimer=++XSRequestTimer % UINT32_MAX;
               if (XSRequestTimer > MSG_REPOLL_TICKS)                           // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
               {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
                  XS_encoder_Q.hue = CMD_RESEND;                                // Stop sending look for next request but come back
#else
                  XS_encoder_Q.hue = CMD_COMPLETE;                              // Stop sending look for next request
#endif
                  XSRequestTimer=0U;                                            // Reset the timer
                  XS_encoder_Q.current=XS_SATURATION;                           // Advance to the next item
               }
               else if (XSRequestTimer > (MSG_REPOLL_TICKS/2))                  // We are half way to timing out so send it again. (incase we missed the okay)
               {
                  XS_encoder_Q.hue = CMD_SEND;                                  // Send another request
               }
           }
         }
         else if (XS_encoder_Q.hue == CMD_COMPLETE)                             // We complete so advance to the next poll element
         {
            XSRequestTimer=0U;                                                  // Reset the timer
            XS_encoder_Q.current=XS_SATURATION;                                 // Advance to the next item
         }
       }

      // If the current command is clip size and we had a change send a request to change it
      //
      if ((XS_encoder_Q.current == XS_SATURATION)&&(XS_encoder_Q.loggedin==true))  // current command and authorised
      {
         if (XS_encoder_Q.saturation == CMD_SEND)                               // We got a saturation change and we are the current item in the poll list sequence
         {
#ifndef CAM_USE_ASCII                                                           // The define decides if we are using pot switch or HMI
            XStreamSaturation.Saturation= hmiRawSaturation[XSsaturationChan].value;       // Set the byte in the message to the scaled contrast value equal to the raw hmi contrast slider
            XStreamSaturation.Channel=XSsaturationChan;
#else
            Number2String(XSsaturationChan,&XStreamSaturation.asciiChannel,2);
            Number2String(hmiRawSaturation[XSsaturationChan].value,&XStreamSaturation.asciiSaturation,5);   // EXAMPLE if we need to convert value to ascii (5 chars long) compare with a strcpy if needed
#endif
            memcpy(Buffer,&XStreamSaturation,sizeof(XStreamSaturation));        // Copy to the UDP send Buffer the conte4nts of the saturation container
            totalMsgLen=sizeof(XStreamSaturation);                              // Calculate the message length for the data portion of the UDP datagram
            udpSent = 0U;
           while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
            {
                XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, sizeof(XStreamSaturation));        // send the Buffer contents as UDP datagram.
                udpSent=++udpSent % UINT8_MAX;
            }
            if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;      // accumulate the bad send counter (remove the 1 before summing)
            XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
            memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
            UART4_write_text(Buffer);                                           // Send it to serial UART4
#endif
            XS_encoder_Q.saturation == CMD_SENT;                                // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
         }
         else if (XS_encoder_Q.saturation == CMD_SENT)
         {
            if (XSEncoderResp  == XS_OK)                                        // The encoder response message was +OK
            {
               XS_encoder_Q.saturation= CMD_COMPLETE;
               XSRequestTimer=0U;                                               // Reset the timer
               XS_encoder_Q.current=XS_CLIPSIZE;                                // Advance to the next item
            }
            else
            {
               XSRequestTimer=++XSRequestTimer % UINT32_MAX;
               if (XSRequestTimer > MSG_REPOLL_TICKS)                           // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
               {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
                  XS_encoder_Q.saturation = CMD_RESEND;                                // Stop sending look for next request but come back
#else
                  XS_encoder_Q.saturation = CMD_COMPLETE;                              // Stop sending look for next request
#endif
                  XSRequestTimer=0U;                                            // Reset the timer
                  XS_encoder_Q.current=XS_CLIPSIZE;                             // Advance to the next item
               }
               else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                 // We are half way to timing out so send it again. (incase we missed the okay)
               {
                  XS_encoder_Q.saturation = CMD_SEND;                           // Send another request
               }
            }
          }
          else if (XS_encoder_Q.saturation == CMD_COMPLETE)                     // We complete so advance to the next poll element
          {
              XSRequestTimer=0U;                                                // Reset the timer
              XS_encoder_Q.current=XS_CLIPSIZE;                                 // Advance to the next item
          }
      }

      // If the current command is clip size and we had a change send a request to change it
      //
      if ((XS_encoder_Q.current == XS_CLIPSIZE)&&(XS_encoder_Q.loggedin==true))  // current command and authorised
      {
         if (XS_encoder_Q.clipsz == CMD_SEND)                                   // We got a clipsize change and we are the current item in the poll list sequence
         {
#ifndef CAM_USE_ASCII                                                           // The define decides if we are using pot switch or HMI
            XStreamClipSize.Size= hmiRawClipSz[XSclipszChan].value;             // Set the byte in the message to the scaled contrast value equal to the raw hmi contrast slider
            XStreamClipSize.Channel=XSclipszChan;
#else
            Number2String(XSclipszChan,&XStreamClipSize.asciiChannel,2U);
            Number2String(hmiRawClipSz[XSclipszChan].value,&XStreamClipSize.asciiClipSz,5U);   // EXAMPLE if we need to convert value to ascii (5 chars long) compare with a strcpy if needed
#endif
            memcpy(Buffer,&XStreamClipSize,sizeof(XStreamClipSize));            // Copy to the UDP send Buffer the conte4nts of the clipsize container
            totalMsgLen=sizeof(XStreamClipSize);                                // Calculate the message length for the data portion of the UDP datagram
            udpSent = 0U;
            while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
            {
               XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, sizeof(XStreamClipSize));        // send the Buffer contents as UDP datagram.
               udpSent=++udpSent % UINT8_MAX;
            }
            if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;      // accumulate the bad send counter (remove the 1 before summing)
            XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
            memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
            UART4_write_text(Buffer);                                           // Send it to serial UART4
#endif
            XS_encoder_Q.clipsz == CMD_SENT;                                    // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XContrast)
         }
         else if (XS_encoder_Q.clipsz == CMD_SENT)
         {
            if (XSEncoderResp  == XS_OK)                                        // The encoder response message was +OK
            {
               XS_encoder_Q.clipsz= CMD_COMPLETE;
               XSRequestTimer=0U;                                               // Reset the timer
               XS_encoder_Q.current=XS_CLIPLEN;                                 // Advance to the next item
            }
            else
            {
               XSRequestTimer=++XSRequestTimer % UINT32_MAX;
               if (XSRequestTimer > MSG_REPOLL_TICKS)                           // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
               {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
                  XS_encoder_Q.clipsz = CMD_RESEND;                             // Stop sending look for next request but come back
#else
                  XS_encoder_Q.clipsz = CMD_COMPLETE;                           // Stop sending look for next request
#endif
                  XSRequestTimer=0U;                                            // Reset the timer
                  XS_encoder_Q.current=XS_CLIPLEN;                              // Advance to the next item
               }
               else if (XSRequestTimer > (MSG_REPOLL_TICKS/2U))                 // We are half way to timing out so send it again. (incase we missed the okay)
               {
                  XS_encoder_Q.clipsz = CMD_SEND;                               // Send another request
               }
            }
         }
         else if (XS_encoder_Q.clipsz == CMD_COMPLETE)                          // We complete so advance to the next poll element
         {
            XSRequestTimer=0U;                                                  // Reset the timer
            XS_encoder_Q.current=XS_CLIPLEN;                                    // Advance to the next item
         }
       }

       // If the current command is clip length and we had a change send a request to change it
       //
       if ((XS_encoder_Q.current == XS_CLIPLEN)&&(XS_encoder_Q.loggedin==true))  // current command and authorised
       {
          if (XS_encoder_Q.cliplen == CMD_SEND)                                 // We got a clip length change and we are the current item in the poll list sequence
          {
#ifndef CAM_USE_ASCII                                                           // The define decides if we are using pot switch or HMI
             XStreamCliplength.Length= (uint16_t) hmiRawCliplen[XScliplenChan].value;  // Set the byte in the message to the scaled contrast value equal to the raw hmi contrast slider
             XStreamCliplength.Channel=XScliplenChan;
#else
             Number2String(XScliplenChan,&XStreamCliplength.asciiChannel,2);
             Number2String(hmiRawCliplen[XScliplenChan].value,&XStreamCliplength.asciiClipLn,5U);   // EXAMPLE if we need to convert value to ascii (5 chars long) compare with a strcpy if needed
#endif
             memcpy(Buffer,&XStreamCliplength,sizeof(XStreamCliplength));       // Copy to the UDP send Buffer the conte4nts of the clip length container
             totalMsgLen=sizeof(XStreamCliplength);                             // Calculate the message length for the data portion of the UDP datagram
             udpSent = 0U;
             while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
             {
                XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, sizeof(XStreamCliplength));        // send the Buffer contents as UDP datagram.
                udpSent=++udpSent % UINT8_MAX;
             }
             if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;         // accumulate the bad send counter (remove the 1 before summing)
             XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
             memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));   // Null terminate the Buffer for serial
             UART4_write_text(Buffer);                                          // Send it to serial UART4
#endif
             XS_encoder_Q.cliplen == CMD_SENT;                                  // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XClipLen)
         }
         else if (XS_encoder_Q.cliplen == CMD_SENT)
         {
            if (XSEncoderResp  == XS_OK)                                        // The encoder response message was +OK
            {
               XS_encoder_Q.cliplen= CMD_COMPLETE;
               XSRequestTimer=0U;                                               // Reset the timer
               XS_encoder_Q.current=XS_FRAMERTE;                                // Advance to the next item
            }
            else
            {
               XSRequestTimer=++XSRequestTimer % UINT32_MAX;
               if (XSRequestTimer > MSG_REPOLL_TICKS)                           // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
               {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
                  XS_encoder_Q.cliplen = CMD_RESEND;                            // Stop sending look for next request but come back
#else
                  XS_encoder_Q.cliplen = CMD_COMPLETE;                          // Stop sending look for next request
#endif
                  XSRequestTimer=0U;                                            // Reset the timer
                  XS_encoder_Q.current=XS_FRAMERTE;                             // Advance to the next item
               }
               else if (XSRequestTimer > (MSG_REPOLL_TICKS/2U))                 // We are half way to timing out so send it again. (incase we missed the okay)
               {
                  XS_encoder_Q.cliplen = CMD_SEND;                              // Send another request
               }
            }
         }
         else if (XS_encoder_Q.cliplen == CMD_COMPLETE)                         // We complete so advance to the next poll element
         {
            XSRequestTimer=0U;                                                  // Reset the timer
            XS_encoder_Q.current=XS_FRAMERTE;                                   // Advance to the next item
         }
      }

      // If the current command is frame rate and we had a change send a request to change it
      //
      if ((XS_encoder_Q.current == XS_FRAMERTE)&&(XS_encoder_Q.loggedin==true))  // current command and authorised
      {
          if (XS_encoder_Q.framert == CMD_SEND)                                 // We got a frame rate change and we are the current item in the poll list sequence
          {
#ifndef CAM_USE_ASCII                                                           // The define decides if we are using pot switch or HMI
             XStreamFrameRate.Length= (uint16_t) hmiRawFramert[XSframerateChan].value;   // Set the byte in the message to the scaled contrast value equal to the raw hmi contrast slider
             XStreamFrameRate.Channel=XSframerateChan;
             XStreamFrameRate.Scalar= hmiRawFrameRtScalar[XSframerateChan];     // Set the scalar for decimal
#else
             Number2String(XSframeRateChan,&XStreamFrameRate.asciiChannel,2U);
             Number2String(hmiRawFramert[XSframeRateChan].value,&XStreamFrameRate.asciiRate,5U);   // EXAMPLE if we need to convert value to ascii (5 chars long) compare with a strcpy if needed
             Number2String(hmiRawFrameRtScalar[XSframeRateChan],&XStreamFrameRate.asciiScalar,5U);
#endif
             memcpy(Buffer,&XStreamFrameRate,sizeof(XStreamFrameRate));         // Copy to the UDP send Buffer the conte4nts of the frame rate container
             totalMsgLen=sizeof(XStreamFrameRate);                              // Calculate the message length for the data portion of the UDP datagram
             udpSent = 0U;
             while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
             {
                XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, sizeof(XStreamFrameRate));        // send the Buffer contents as UDP datagram.
                udpSent=++udpSent % UINT8_MAX;
             }
             if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;     // accumulate the bad send counter (remove the 1 before summing)
             XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
             memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));   // Null terminate the Buffer for serial
             UART4_write_text(Buffer);                                          // Send it to serial UART4
#endif
             XS_encoder_Q.framert == CMD_SENT;                                  // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XframeRate)
          }
          else if (XS_encoder_Q.framert == CMD_SENT)
          {
              if (XSEncoderResp  == XS_OK)                                      // The encoder response message was +OK
              {
                  XS_encoder_Q.framert= CMD_COMPLETE;
                  XSRequestTimer=0U;                                            // Reset the timer
                  XS_encoder_Q.current=XS_BITRTE;                               // Advance to the next item
              }
              else
              {
                 XSRequestTimer=++XSRequestTimer % UINT32_MAX;
                 if (XSRequestTimer > MSG_REPOLL_TICKS)                         // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
                 {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
                    XS_encoder_Q.framert = CMD_RESEND;                          // Stop sending look for next request but come back
#else
                    XS_encoder_Q.framert = CMD_COMPLETE;                        // Stop sending look for next request
#endif
                    XSRequestTimer=0U;                                          // Reset the timer
                    XS_encoder_Q.current=XS_BITRTE;                             // Advance to the next item
                 }
                 else if (XSRequestTimer > (MSG_REPOLL_TICKS/2U))               // We are half way to timing out so send it again. (incase we missed the okay)
                 {
                    XS_encoder_Q.framert = CMD_SEND;                            // Send another request
                 }
              }
          }
          else if (XS_encoder_Q.framert == CMD_COMPLETE)                        // We complete so advance to the next poll element
          {
             XSRequestTimer=0;                                                  // Reset the timer
             XS_encoder_Q.current=XS_BITRTE;                                    // Advance to the next item
          }
       }

       // If the current command is bit rate and we had a change send a request to change it
       //
       if ((XS_encoder_Q.current == XS_BITRTE)&&(XS_encoder_Q.loggedin==true))  // current command and authorised
       {
          if (XS_encoder_Q.bitrate == CMD_SEND)                                 // We got a bit rate change and we are the current item in the poll list sequence
          {
#ifndef CAM_USE_ASCII                                                           // The define decides if we are using pot switch or HMI
             XStreamBitRate.bitrate= (uint16_t) hmiRawBitrate[XSbitRateChan].value;    // Set the byte in the message to the scaled contrast value equal to the raw hmi contrast slider
             XStreamBitRate.Channel=XSbitRateChan;
#else
             Number2String(XSbitRateChan,&XStreamBitRate.asciiChannel,2U);
             Number2String(hmiRawBitrate[XSbitRateChan].value,&XStreamBitRate.asciiBitrate,5U);   // EXAMPLE if we need to convert value to ascii (5 chars long) compare with a strcpy if needed
#endif
            memcpy(Buffer,&XStreamBitRate,sizeof(XStreamBitRate));              // Copy to the UDP send Buffer the conte4nts of the bit rate container
            totalMsgLen=sizeof(XStreamBitRate);                                 // Calculate the message length for the data portion of the UDP datagram
            udpSent = 0U;
            while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
            {
               XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, sizeof(XStreamBitRate));        // send the Buffer contents as UDP datagram.
               udpSent=++udpSent % UINT8_MAX;
            }
            if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;         // accumulate the bad send counter (remove the 1 before summing)
            XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
            memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));    // Null terminate the Buffer for serial
            UART4_write_text(Buffer);                                           // Send it to serial UART4
#endif
            XS_encoder_Q.bitrate == CMD_SENT;                                   // Advance to sent mode (timer will re-intialise if we didnt get a 1 in XBitRate)
         }
         else if (XS_encoder_Q.bitrate == CMD_SENT)
         {
            if (XSEncoderResp  == XS_OK)                                        // The encoder response message was +OK
            {
               XS_encoder_Q.bitrate= CMD_COMPLETE;
               XSRequestTimer=0U;                                                // Reset the timer
               XS_encoder_Q.current=XS_INTVAL;                                  // Advance to the next item
            }
            else
            {
               XSRequestTimer=++XSRequestTimer % UINT32_MAX;
               if (XSRequestTimer > MSG_REPOLL_TICKS)                           // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
               {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
                  XS_encoder_Q.bitrate = CMD_RESEND;                            // Stop sending look for next request but come back
#else
                  XS_encoder_Q.bitrate = CMD_COMPLETE;                          // Stop sending look for next request
#endif
                  XSRequestTimer=0U;                                            // Reset the timer
                  XS_encoder_Q.current=XS_INTVAL;                               // Advance to the next item
               }
               else if (XSRequestTimer > (MSG_REPOLL_TICKS/2U))                 // We are half way to timing out so send it again. (incase we missed the okay)
               {
                  XS_encoder_Q.bitrate = CMD_SEND;                              // Send another request
               }
            }
         }
         else if (XS_encoder_Q.bitrate == CMD_COMPLETE)                         // We complete so advance to the next poll element
         {
            XSRequestTimer=0U;                                                  // Reset the timer
            XS_encoder_Q.current=XS_INTVAL;                                     // Advance to the next item
         }
      }

      // If the current command is intval and we had a change send a request to change it
      //
      if ((XS_encoder_Q.current == XS_INTVAL)&&(XS_encoder_Q.loggedin==true))   // current command and authorised
      {
         if (XS_encoder_Q.frameint == CMD_SEND)                                 // We got a stream interval change and we are the current item in the poll list sequence
         {
#ifndef CAM_USE_ASCII                                                           // The define decides if we are using pot switch or HMI
             XStreamIInterval.interval= (uint16_t) hmiRawFrameint[XSframeintChan].value;// Set the byte in the message to the scaled contrast value equal to the raw hmi contrast slider
             XStreamIInterval.Channel=XSframeintChan;

#else
             Number2String(XSframeIntChan,&XStreamIInterval.asciiChannel,2);
             Number2String(hmiRawFrameint[XSframeIntChan].value,&XStreamIInterval.asciiInterval,5);   // EXAMPLE if we need to convert value to ascii (5 chars long) compare with a strcpy if needed
#endif
             memcpy(Buffer,&XStreamIInterval,sizeof(XStreamIInterval));         // Copy to the UDP send Buffer the conte4nts of the frame interval container
             totalMsgLen=sizeof(XStreamIInterval);                              // Calculate the message length for the data portion of the UDP datagram
             udpSent = 0U;
             while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
             {
                XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, sizeof(XStreamIInterval));        // send the Buffer contents as UDP datagram.
                udpSent=++udpSent % UINT8_MAX;
             }
             if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;     // accumulate the bad send counter (remove the 1 before summing)
             XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
             memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));   // Null terminate the Buffer for serial
             UART4_write_text(Buffer);                                          // Send it to serial UART4
#endif
             XS_encoder_Q.frameint == CMD_SENT;                                 // Advance to sent mode (timer will re-intialise if we didnt get a 1 in Xframe interval)
         }
         else if (XS_encoder_Q.frameint == CMD_SENT)
         {
            if (XSEncoderResp  == XS_OK)                                        // The encoder response message was +OK
            {
               XS_encoder_Q.frameint= CMD_COMPLETE;
               XSRequestTimer=0;                                                // Reset the timer
               XS_encoder_Q.current=XS_SYSSTAT;                                 // Advance to the next item
            }
            else
            {
               XSRequestTimer=++XSRequestTimer % UINT32_MAX;
               if (XSRequestTimer > MSG_REPOLL_TICKS)                           // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
               {
#if XS_DELTA_MODE == XS_CONT_TIL_SUCC
                  XS_encoder_Q.frameint = CMD_RESEND;                           // Stop sending look for next request but come back
#else
                  XS_encoder_Q.frameint = CMD_COMPLETE;                         // Stop sending look for next request
#endif
                  XSRequestTimer=0;                                             // Reset the timer
                  XS_encoder_Q.current=XS_SYSSTAT;                              // Advance to the next item
               }
               else if (XSRequestTimer > (MSG_REPOLL_TICKS/2))                  // We are half way to timing out so send it again. (incase we missed the okay)
               {
                  XS_encoder_Q.frameint = CMD_SEND;                             // Send another request
               }
            }
         }
         else if (XS_encoder_Q.frameint == CMD_COMPLETE)                        // We complete so advance to the next poll element
         {
             XSRequestTimer=0;                                                  // Reset the timer
             XS_encoder_Q.current=XS_SYSSTAT;                                   // Advance to the next item
         }
      }

// ============== Look for change of state of other GUI variables ==============

      g_secondsLow = (RTCTIME & 0xF00)>>8;                                      // get the seconds from the RTC to poll information requests
      g_secondsHigh = (RTCTIME & 0x7000)>>12;
      g_secondsActual = (g_secondsHigh * 10) + g_secondsLow;

      if (((g_secondsActual % 30u)==0u) && (XS_encoder_Q.sysstatus == CMD_COMPLETE))  // every 30 seconds poll this request on second and half second
      {
         XS_encoder_Q.sysstatus == CMD_SEND;                                    // request the system status
      }
      if ((g_secondsActual==10u) && (XS_encoder_Q.diskstatus == CMD_COMPLETE))     // every minute poll this request at 10 seconds past the minute
      {
         XS_encoder_Q.diskstatus == CMD_SEND;                                   // request the disk status
      }
      if (XSEncButtons1.hmiCommitRequest==true && (XS_encoder_Q.commit == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.commit == CMD_SEND;                                       // Set the request to change the bit rate
         XSEncButtons1.hmiCommitRequest=false;                                  // reset the request
      }
      if (XSEncButtons1.hmiRecordRequest==true && (XS_encoder_Q.startrecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.startrecord == CMD_SEND;                                  // Set the request to start recording
         XSEncButtons1.hmiRecordRequest=false;                                  // reset the request
         XSRecordChan = -1;                                                     // set the record channel to all
      }
      else if (XSEncButtons1.hmiRecordRequestCh1==true && (XS_encoder_Q.startrecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.startrecord == CMD_SEND;                                  // Set the request to change the bit rate
         XSEncButtons1.hmiRecordRequestCh1=false;                               // reset the request
         XSRecordChan = 1;                                                      // set the record channel to 1
      }
      else if (XSEncButtons1.hmiRecordRequestCh2==true && (XS_encoder_Q.startrecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.startrecord == CMD_SEND;                                  // Set the request to change the bit rate
         XSEncButtons1.hmiRecordRequestCh2=false;                               // reset the request
         XSRecordChan = 2;                                                      // set the record channel to 2
      }
      else if (XSEncButtons1.hmiRecordRequestCh3==true && (XS_encoder_Q.startrecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.startrecord == CMD_SEND;                                  // Set the request to change the bit rate
         XSEncButtons1.hmiRecordRequestCh3=false;                               // reset the request
         XSRecordChan = 3;                                                      // set the record channel to 3
      }
      else if (XSEncButtons1.hmiRecordRequestCh4==true && (XS_encoder_Q.startrecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.startrecord == CMD_SEND;                                  // Set the request to change the bit rate
         XSEncButtons1.hmiRecordRequestCh4=false;                               // reset the request
         XSRecordChan = 4;                                                      // set the record channel to 4
      }
      if (XSEncButtons1.hmiStopRequest==true && (XS_encoder_Q.stoprecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.stoprecord == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiStopRequest=false;                                    // reset the request
         XSRecordChan = -1;                                                     // set the record channel to all
      }
      else if (XSEncButtons1.hmiStopRequestCh1==true && (XS_encoder_Q.stoprecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.stoprecord == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiStopRequestCh1=false;                                 // reset the request
         XSRecordChan = 1;                                                      // set the record channel to 1
      }
      else if (XSEncButtons1.hmiStopRequestCh2==true && (XS_encoder_Q.stoprecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.stoprecord == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiStopRequestCh2=false;                                 // reset the request
         XSRecordChan = 2;                                                      // set the record channel to 2
      }
      else if (XSEncButtons1.hmiStopRequestCh3==true && (XS_encoder_Q.stoprecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.stoprecord == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiStopRequestCh1=false;                                 // reset the request
         XSRecordChan = 3;                                                      // set the record channel to 3
      }
      else if (XSEncButtons1.hmiStopRequestCh4==true && (XS_encoder_Q.stoprecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.stoprecord == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiStopRequestCh4=false;                                 // reset the request
         XSRecordChan = 4;                                                      // set the record channel to 4
      }
      if (XSEncButtons1.hmiMarkRequest==true && (XS_encoder_Q.markrecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.markrecord == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiMarkRequest=false;                                    // reset the request
         XSRecordChan = -1;                                                     // set the record channel to all
      }
      else if (XSEncButtons1.hmiMarkRequestCh1==true && (XS_encoder_Q.markrecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.markrecord == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiMarkRequestCh1=false;                                 // reset the request
         XSRecordChan = 1;                                                      // set the record channel to 1
      }
      else if (XSEncButtons1.hmiMarkRequestCh2==true && (XS_encoder_Q.markrecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.markrecord == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiMarkRequestCh2=false;                                 // reset the request
         XSRecordChan = 2;                                                      // set the record channel to 2
      }
      else if (XSEncButtons1.hmiMarkRequestCh3==true && (XS_encoder_Q.markrecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.markrecord == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiMarkRequestCh3=false;                                 // reset the request
         XSRecordChan = 3;                                                      // set the record channel to 3
      }
      else if (XSEncButtons1.hmiMarkRequestCh4==true && (XS_encoder_Q.markrecord == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.markrecord == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiMarkRequestCh4=false;                                 // reset the request
         XSRecordChan = 4;                                                      // set the record channel to 4
      }
      if (XSEncButtons1.hmiEraseRequest==true && (XS_encoder_Q.erasefiles == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.erasefiles == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiEraseRequest=false;                                   // reset the request
         XSRecordChan = -1;                                                     // set the record channel to all
      }
      else if (XSEncButtons1.hmiEraseRequestCh1==true && (XS_encoder_Q.erasefiles == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.erasefiles == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiEraseRequestCh1=false;                                // reset the request
         XSRecordChan = 1;                                                      // set the record channel to all
      }
      else if (XSEncButtons1.hmiEraseRequestCh2==true && (XS_encoder_Q.erasefiles == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.erasefiles == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiEraseRequestCh2=false;                                // reset the request
         XSRecordChan = 2;                                                      // set the record channel to all
      }
      else if (XSEncButtons1.hmiEraseRequestCh3==true && (XS_encoder_Q.erasefiles == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.erasefiles == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiEraseRequestCh3=false;                                // reset the request
         XSRecordChan = 3;                                                      // set the record channel to all
      }
      else if (XSEncButtons1.hmiEraseRequestCh4==true && (XS_encoder_Q.erasefiles == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.erasefiles == CMD_SEND;                                   // Set the request to change the bit rate
         XSEncButtons1.hmiEraseRequestCh4=false;                                // reset the request
         XSRecordChan = 4;                                                      // set the record channel to all
      }
      if (XSEncButtons1.hmiShutdRequest==true && (XS_encoder_Q.shutdown == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.shutdown == CMD_SEND;                                     // Set the request to change the bit rate
         XSEncButtons1.hmiShutdRequest=false;                                   // reset the request
      }
      if (XSEncButtons1.hmiRebootRequest==true && (XS_encoder_Q.reboot == CMD_COMPLETE))
      {                                                                         // Data Change after complete
         XS_encoder_Q.reboot == CMD_SEND;                                       // Set the request to change the bit rate
         XSEncButtons1.hmiRebootRequest=false;                                  // reset the request
      }
      compute_secs(&hmiTimeCalcTotal, &hmiDateTime);                            // Compute total seconds to see if any time was changed on the GUI
      compute_date(&hmiDateCalcTotal, &hmiDateTime);                            // Compute integer from date to see if changed on the GUI

      if (((XS_encoder_Q.settime == CMD_COMPLETE) && ((XSChangeDetected(&hmiTimeCalcTotal)==1) || (XSChangeDetected(&hmiDateCalcTotal)==1))) && (check_datetime(&hmiDateTime)==1))
      {                                                                         // Data Change after complete
         XS_encoder_Q.settime == CMD_SEND;                                      // Set the request to change the hue
         hmiTimeCalcTotal.last = hmiTimeCalcTotal.value;                        // Save the last HMI change on time we have latched the request for it
         hmiDateCalcTotal.last = hmiDateCalcTotal.value;                        // Save the last HMI change on date we have latched the request for it
      }
      else if (check_datetime(&hmiDateTime)==0)                                 // An invalid date or time format was specified on the GUI
      {
         XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_TIME;                      // Alarm time or DATE specified is wrong
      }

// ===================== CAMERA POLL DATA for disk and system Status ===========

   if ((XS_encoder_Q.current == XS_SYSSTAT)&&(XS_encoder_Q.loggedin==true))  // current command and authorised
   {
      if (XS_encoder_Q.sysstatus == CMD_SEND)                                   // poll request made
      {
        API_XS_SYS(Buffer);                                                     // Request the system state
        totalMsgLen=(sizeToChar( Buffer,'\r' ))+1u;                             // termination is a CR 0x0D or \r  (add 1 to include it)
        udpSent = 0u;
        while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
        {
          XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
          udpSent=++udpSent % UINT8_MAX;
        }
        if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;         // accumulate the bad send counter (remove the 1 before summing)
        XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif
        XS_encoder_Q.sysstatus == CMD_SENT;                                      // Advance to sent mode (timer will re-intialise if we didnt get a 1 in Xframe interval)
      }
      else if (XS_encoder_Q.sysstatus == CMD_SENT)
      {
         if (XSEncoderResp  == XS_SYS)                                           // The encoder response message was +SYSSTATUS
         {
           XS_encoder_Q.sysstatus= CMD_COMPLETE;
           XSRequestTimer=0;                                                    // Reset the timer
           XS_encoder_Q.current=XS_DISKSTAT;                                    // Advance to the next item
         }
         else
         {
            XSRequestTimer=++XSRequestTimer % UINT32_MAX;
            if (XSRequestTimer > MSG_REPOLL_TICKS)                              // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
            {
               XS_encoder_Q.sysstatus = CMD_COMPLETE;                           // Stop sending look for next request
               XSRequestTimer=0;                                                // Reset the timer
               XS_encoder_Q.current=XS_DISKSTAT;                                // Advance to the next item
            }
            else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                    // We are half way to timing out so send it again. (incase we missed the okay)
            {
               XS_encoder_Q.sysstatus = CMD_SEND;                               // Send another request
            }
         }
      }
      else if (XS_encoder_Q.sysstatus == CMD_COMPLETE)                          // We complete so advance to the next poll element
      {
        XSRequestTimer=0u;                                                       // Reset the timer
        XS_encoder_Q.current=XS_DISKSTAT;                                       // Advance to the next item
      }
    }

   if ((XS_encoder_Q.current == XS_DISKSTAT)&&(XS_encoder_Q.loggedin==true))    // current command and authorised
   {
      if (XS_encoder_Q.diskstatus == CMD_SEND)                                  // diskstatus request
      {
        API_XS_DISK(Buffer);                                                    // Request the disk state
        totalMsgLen=(sizeToChar( Buffer,'\r' ))+1;                              // termination is a CR 0x0D or \r  (add 1 to include it)
        udpSent = 0u;
        while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
        {
          XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
          udpSent=++udpSent % UINT8_MAX;
        }
        if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;          // accumulate the bad send counter (remove the 1 before summing)
        XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif
         XS_encoder_Q.diskstatus == CMD_SENT;                                   // Advance to sent mode (timer will re-intialise if we didnt get a 1 in Xframe interval)
      }
      else if (XS_encoder_Q.diskstatus == CMD_SENT)
      {
         if (XSEncoderResp  == XS_DISK)                                         // The encoder response message was +DISKSTATUS
         {
           XS_encoder_Q.diskstatus= CMD_COMPLETE;
           XSRequestTimer=0u;                                                   // Reset the timer
           XS_encoder_Q.current= XS_COMMIT;                                     // Advance to next
         }
         else
         {
            XSRequestTimer=++XSRequestTimer % UINT32_MAX;
            if (XSRequestTimer > MSG_REPOLL_TICKS)                              // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
            {
               XS_encoder_Q.diskstatus = CMD_COMPLETE;                          // Stop sending look for next request
               XSRequestTimer=0u;                                               // Reset the timer
               XS_encoder_Q.current= XS_COMMIT;                                 // Advance to next
            }
            else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                     // We are half way to timing out so send it again. (incase we missed the okay)
            {
               XS_encoder_Q.diskstatus = CMD_SEND;                              // Send another request
            }
         }
       }
       else if (XS_encoder_Q.diskstatus == CMD_COMPLETE)                        // We complete so advance to the next poll element
       {
         XSRequestTimer=0;                                                      // Reset the timer
         XS_encoder_Q.current= XS_COMMIT;                                       // Advance to next
       }
    }
// ===================== Camera Encoder write by exception =====================
   if ((XS_encoder_Q.current == XS_COMMIT)&&(XS_encoder_Q.loggedin==true))      // current command and authorised
   {
      if (XS_encoder_Q.commit == CMD_SEND)                                      // commit changes to disk requested
      {
        API_XS_COMT(Buffer);                                                    // Request the commit
        totalMsgLen=(sizeToChar( Buffer,'\r' ))+1u;                              // termination is a CR 0x0D or \r  (add 1 to include it)
        udpSent = 0u;
        while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
        {
          XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
          udpSent=++udpSent % UINT8_MAX;
        }
        if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;          // accumulate the bad send counter (remove the 1 before summing)
        XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif
         XS_encoder_Q.commit == CMD_SENT;                                       // Advance to sent mode (timer will re-intialise if we didnt get a 1 in Xframe interval)
      }
      else if (XS_encoder_Q.commit == CMD_SENT)
      {
         if (XSEncoderResp  == XS_OK)                                           // The encoder response message was +OK
         {
           XS_encoder_Q.commit= CMD_COMPLETE;
           XSRequestTimer=0;                                                    // Reset the timer
           XS_encoder_Q.current=XS_REBOOT;                                      // Advance to the next item
         }
         else
         {
            XSRequestTimer=++XSRequestTimer % UINT32_MAX;
            if (XSRequestTimer > MSG_REPOLL_TICKS)                              // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
            {
               XS_encoder_Q.commit = CMD_COMPLETE;                              // Stop sending look for next request
               XSRequestTimer=0u;                                                // Reset the timer
               XS_encoder_Q.current=XS_REBOOT;                                  // Advance to the next item
            }
            else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                     // We are half way to timing out so send it again. (incase we missed the okay)
            {
               XS_encoder_Q.commit= CMD_SEND;                                   // Send another request
            }
         }
       }
       else if (XS_encoder_Q.commit == CMD_COMPLETE)                            // We complete so advance to the next poll element
       {
         XSRequestTimer=0u;                                                     // Reset the timer
         XS_encoder_Q.current=XS_REBOOT;                                        // Advance to the next item
       }
    }

   if ((XS_encoder_Q.current == XS_REBOOT)&&(XS_encoder_Q.loggedin==true))      // current command and authorised
   {
      if (XS_encoder_Q.reboot == CMD_SEND)                                      // reboot encoder
      {
        API_XS_REBOOT(Buffer);                                                  // Request the encoder re-start
        totalMsgLen=(sizeToChar( Buffer,'\r' ))+1u;                              // termination is a CR 0x0D or \r  (add 1 to include it)
        udpSent = 0u;
        while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
        {
          XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
          udpSent=++udpSent % UINT8_MAX;
        }
        if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;          // accumulate the bad send counter (remove the 1 before summing)
        XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif
         XS_encoder_Q.reboot == CMD_SENT;                                       // Advance to sent mode (timer will re-intialise if we didnt get a 1 in Xframe interval)
      }
      else if (XS_encoder_Q.reboot == CMD_SENT)
      {
         if (XSEncoderResp  == XS_OK)                                           // The encoder response message was +OK
         {
           XS_encoder_Q.reboot= CMD_COMPLETE;
           XSRequestTimer=0u;                                                   // Reset the timer
           XS_encoder_Q.current=XS_SHUTD;                                       // Advance to the next item
         }
         else
         {
            XSRequestTimer=++XSRequestTimer % UINT32_MAX;
            if (XSRequestTimer > MSG_REPOLL_TICKS)                              // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
            {
               XS_encoder_Q.reboot = CMD_COMPLETE;                              // Stop sending look for next request
               XSRequestTimer=0;                                                // Reset the timer
               XS_encoder_Q.current=XS_SHUTD;                                   // Advance to the next item
            }
            else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                     // We are half way to timing out so send it again. (incase we missed the okay)
            {
               XS_encoder_Q.commit= CMD_SEND;                                   // Send another request
            }
         }
       }
       else if (XS_encoder_Q.reboot == CMD_COMPLETE)                            // We complete so advance to the next poll element
       {
         XSRequestTimer=0u;                                                      // Reset the timer
         XS_encoder_Q.current=XS_SHUTD;                                         // Advance to the next item
       }
    }

   if ((XS_encoder_Q.current == XS_SHUTD)&&(XS_encoder_Q.loggedin==true))       // current command and authorised
   {
      if (XS_encoder_Q.shutdown == CMD_SEND)                                    // shutdown encoder
      {
        API_XS_SHUTD(Buffer);                                                   // Request the encoder re-start
        totalMsgLen=(sizeToChar( Buffer,'\r' ))+1u;                             // termination is a CR 0x0D or \r  (add 1 to include it)
        udpSent = 0u;
        while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
        {
          XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
          udpSent=++udpSent % UINT8_MAX;
        }
        if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;         // accumulate the bad send counter (remove the 1 before summing)
        XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif
         XS_encoder_Q.shutdown == CMD_SENT;                                       // Advance to sent mode (timer will re-intialise if we didnt get a 1 in Xframe interval)
      }
      else if (XS_encoder_Q.shutdown == CMD_SENT)
      {
         if (XSEncoderResp  == XS_OK)                                           // The encoder response message was +OK
         {
           XS_encoder_Q.shutdown= CMD_COMPLETE;
           XSRequestTimer=0u;                                                   // Reset the timer
           XS_encoder_Q.current=XS_STARTREC;                                    // advance to next
         }
         else
         {
            XSRequestTimer=++XSRequestTimer % UINT32_MAX;
            if (XSRequestTimer > MSG_REPOLL_TICKS)                              // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
            {
               XS_encoder_Q.shutdown = CMD_COMPLETE;                            // Stop sending look for next request
               XSRequestTimer=0;                                                // Reset the timer
               XS_encoder_Q.current=XS_STARTREC;                                // back to first item in sequence
            }
            else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                     // We are half way to timing out so send it again. (incase we missed the okay)
            {
               XS_encoder_Q.shutdown= CMD_SEND;                                 // Send another request
            }
         }
       }
       else if (XS_encoder_Q.shutdown == CMD_COMPLETE)                          // We complete so advance to the next poll element
       {
         XSRequestTimer=0u;                                                     // Reset the timer
         XS_encoder_Q.current=XS_STARTREC;                                      // advance to next
       }
    }

   if ((XS_encoder_Q.current == XS_STARTREC)&&(XS_encoder_Q.loggedin==true))    // current command and authorised
   {
      if (XS_encoder_Q.startrecord == CMD_SEND)                                 // start recording to disk
      {
        API_XS_RECORD(Buffer,XSRecordChan);                                     // Request the encoder start recording to disk on the specified channel
        totalMsgLen=(sizeToChar( Buffer,'\r' ))+1u;                             // termination is a CR 0x0D or \r  (add 1 to include it)
        udpSent = 0;
        while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
        {
          XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
          udpSent=++udpSent % UINT8_MAX;
        }
        if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;          // accumulate the bad send counter (remove the 1 before summing)
        XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif
         XS_encoder_Q.startrecord == CMD_SENT;                                       // Advance to sent mode (timer will re-intialise if we didnt get a 1 in Xframe interval)
      }
      else if (XS_encoder_Q.startrecord == CMD_SENT)
      {
         switch(XSEncoderResp)
         {
            case XS_OK:                                                         // The encoder response message was +OK
            XS_encoder_Q.startrecord= CMD_COMPLETE;
            XSRequestTimer=0;                                                   // Reset the timer
            XS_encoder_Q.current=XS_STOPREC;                                    // advance to next item in sequence
            break;

            case XS_WRONG:                                                      // Channel already recording
            XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_STATE;                  // Alarm channel is in wrong state
            XS_encoder_Q.startrecord= CMD_COMPLETE;                             // Complete the request
            XSRequestTimer=0;                                                   // Reset the timer
            XS_encoder_Q.current=XS_STOPREC;                                    // to the next item in sequence
            break;

            default:
            XSRequestTimer=++XSRequestTimer % UINT32_MAX;
            if (XSRequestTimer > MSG_REPOLL_TICKS)                              // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
            {
               XS_encoder_Q.startrecord = CMD_COMPLETE;                         // Stop sending look for next request
               XSRequestTimer=0;                                                // Reset the timer
               XS_encoder_Q.current=XS_STOPREC;                                 // advance to next item in sequence
            }
            else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                     // We are half way to timing out so send it again. (incase we missed the okay)
            {
               XS_encoder_Q.startrecord= CMD_SEND;                              // Send another request
            }
            break;
         }
       }
       else if (XS_encoder_Q.startrecord == CMD_COMPLETE)                       // We complete so advance to the next poll element
       {
         XSRequestTimer=0;                                                      // Reset the timer
         XS_encoder_Q.current=XS_STOPREC;                                       // advance to next item in sequence
       }
    }

   if ((XS_encoder_Q.current == XS_STOPREC)&&(XS_encoder_Q.loggedin==true))    // current command and authorised
   {
      if (XS_encoder_Q.stoprecord == CMD_SEND)                                  // stop recording to disk
      {
        API_XS_STOPRECORD(Buffer,XSRecordChan);                                 // Request the encoder to stop recording to disk on the specified channel
        totalMsgLen=(sizeToChar( Buffer,'\r' ))+1u;                              // termination is a CR 0x0D or \r  (add 1 to include it)
        udpSent = 0u;
        while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
        {
          XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
          udpSent=++udpSent % UINT8_MAX;
        }
        if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;          // accumulate the bad send counter (remove the 1 before summing)
        XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif
         XS_encoder_Q.stoprecord == CMD_SENT;                                   // Advance to sent mode (timer will re-intialise if we didnt get a 1 in Xframe interval)
      }
      else if (XS_encoder_Q.stoprecord == CMD_SENT)
      {
         switch(XSEncoderResp)
         {
            case XS_OK:                                                         // Success
            XS_encoder_Q.stoprecord= CMD_COMPLETE;
            XSRequestTimer=0u;                                                  // Reset the timer
            XS_encoder_Q.current=XS_MARKREC;                                    // to next item in sequence
            break;

            case XS_WRONG:                                                      // Channel not recording
            XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_STATE;                  // Alarm channel is in wrong state
            XS_encoder_Q.stoprecord= CMD_COMPLETE;                              // Complete the request
            XSRequestTimer=0u;                                                  // Reset the timer
            XS_encoder_Q.current=XS_MARKREC;                                    // to the next item in sequence
            break;

            default:                                                            // Otherwise do
            XSRequestTimer=++XSRequestTimer % UINT32_MAX;
            if (XSRequestTimer > MSG_REPOLL_TICKS)                              // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
            {
               XS_encoder_Q.stoprecord = CMD_COMPLETE;                          // Stop sending look for next request
               XSRequestTimer=0u;                                               // Reset the timer
               XS_encoder_Q.current=XS_MARKREC;                                 // to next item in sequence
            }
            else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                     // We are half way to timing out so send it again. (incase we missed the okay)
            {
               XS_encoder_Q.stoprecord= CMD_SEND;                               // Send another request
            }
            break;
         }
       }
       else if (XS_encoder_Q.stoprecord == CMD_COMPLETE)                        // We complete so advance to the next poll element
       {
         XSRequestTimer=0u;                                                     // Reset the timer
         XS_encoder_Q.current=XS_MARKREC;                                       // to next item in sequence
       }
    }

   if ((XS_encoder_Q.current == XS_MARKREC)&&(XS_encoder_Q.loggedin==true))     // current command and authorised
   {
      if (XS_encoder_Q.markrecord == CMD_SEND)                                 // start recording to disk
      {
        API_XS_MARK(Buffer,XSRecordChan);                                       // Request the encoder start recording to disk on the specified channel
        totalMsgLen=(sizeToChar( Buffer,'\r' ))+1u;                              // termination is a CR 0x0D or \r  (add 1 to include it)
        udpSent = 0u;
        while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
        {
          XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
          udpSent=++udpSent % UINT8_MAX;
        }
        if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;          // accumulate the bad send counter (remove the 1 before summing)
        XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif
         XS_encoder_Q.markrecord == CMD_SENT;                                   // Advance to sent mode (timer will re-intialise if we didnt get a 1 in Xframe interval)
      }
      else if (XS_encoder_Q.markrecord == CMD_SENT)
      {
         switch(XSEncoderResp)
         {
            case XS_OK:                                                         // Success
            XS_encoder_Q.markrecord= CMD_COMPLETE;                              // Complete the request
            XSRequestTimer=0u;                                                  // Reset the timer
            XS_encoder_Q.current=XS_CONTRAST;                                   // back to first item in sequence
            break;

            case XS_WRONG:                                                      // Channel not recording
            XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_STATE;                  // Alarm channel is in wrong state
            XS_encoder_Q.markrecord= CMD_COMPLETE;                              // Complete the request
            XSRequestTimer=0u;                                                  // Reset the timer
            XS_encoder_Q.current=XS_ERASE;                                      // next item in sequence
            break;

            default:                                                            // Otherwise do
            XSRequestTimer=++XSRequestTimer % UINT32_MAX;                       // Count the tick timer
            if (XSRequestTimer > MSG_REPOLL_TICKS)                              // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
            {
               XS_encoder_Q.markrecord = CMD_COMPLETE;                          // Stop sending look for next request
               XSRequestTimer=0u;                                               // Reset the timer
               XS_encoder_Q.current=XS_ERASE;                                   // next item in sequence
            }
            else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                    // We are half way to timing out so send it again. (incase we missed the okay)
            {
               XS_encoder_Q.markrecord= CMD_SEND;                               // Send another request
            }
            break;
         }
       }
       else if (XS_encoder_Q.markrecord == CMD_COMPLETE)                        // We complete so advance to the next poll element
       {
         XSRequestTimer=0u;                                                     // Reset the timer
         XS_encoder_Q.current=XS_ERASE;                                         // next item in sequence
       }
    }

   if ((XS_encoder_Q.current == XS_ERASE)&&(XS_encoder_Q.loggedin==true))       // current command and authorised
   {
      if (XS_encoder_Q.erasefiles == CMD_SEND)                                  // start erasing the files from disk
      {
        API_XS_ERASE(Buffer,XSRecordChan);                                      // Request the encoder to start erasing the recordings on disk for the specified channel
        totalMsgLen=(sizeToChar( Buffer,'\r' ))+1u;                             // termination is a CR 0x0D or \r  (add 1 to include it)
        udpSent = 0u;
        while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
        {
          XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
          udpSent=++udpSent % UINT8_MAX;
        }
        if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;          // accumulate the bad send counter (remove the 1 before summing)
        XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif
         XS_encoder_Q.erasefiles == CMD_SENT;                                   // Advance to sent mode (timer will re-intialise if we didnt get a 1 in Xframe interval)
      }
      else if (XS_encoder_Q.erasefiles == CMD_SENT)
      {
         switch(XSEncoderResp)
         {
            case XS_OK:                                                         // Success
            XS_encoder_Q.erasefiles= CMD_COMPLETE;                              // Complete the request
            XSRequestTimer=0u;                                                  // Reset the timer
            XS_encoder_Q.current=XS_SETTIME;                                    // back to first item in sequence
            break;

            case XS_WRONG:                                                      // Channel not recording
            XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_STATE;                  // Alarm channel is in wrong state
            XS_encoder_Q.erasefiles= CMD_COMPLETE;                              // Complete the request
            XSRequestTimer=0u;                                                  // Reset the timer
            XS_encoder_Q.current=XS_SETTIME;                                    // back to first item in sequence
            break;

            default:                                                            // Otherwise do
            XSRequestTimer=++XSRequestTimer % UINT32_MAX;                       // Count the tick timer
            if (XSRequestTimer > MSG_REPOLL_TICKS)                              // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
            {
               XS_encoder_Q.erasefiles = CMD_COMPLETE;                          // Stop sending look for next request
               XSRequestTimer=0u;                                               // Reset the timer
               XS_encoder_Q.current=XS_SETTIME;                                 // back to first item in sequence
            }
            else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                    // We are half way to timing out so send it again. (incase we missed the okay)
            {
               XS_encoder_Q.erasefiles= CMD_SEND;                               // Send another request
            }
            break;
         }
       }
       else if (XS_encoder_Q.erasefiles == CMD_COMPLETE)                        // We complete so advance to the next poll element
       {
         XSRequestTimer=0u;                                                     // Reset the timer
         XS_encoder_Q.current=XS_SETTIME;                                       // back to first item in sequence
       }
    }

   if ((XS_encoder_Q.current == XS_SETTIME)&&(XS_encoder_Q.loggedin==true))       // current command and authorised
   {
      if (XS_encoder_Q.settime == CMD_SEND)                                     // send a message to set the encoder time
      {
        API_XS_DATEFORMAT(XSTimeDate,hmiDateTime.Year,hmiDateTime.Month,hmiDateTime.Day,hmiDateTime.Hour,hmiDateTime.Minute,hmiDateTime.Second); // Create a string for the time and date in the format accepted by the encoder
        API_XS_TIME(Buffer,XSTimeDate);                                         // Request the encoder to set the time and date on the encoder
        totalMsgLen=(sizeToChar( Buffer,'\r' ))+1u;                             // termination is a CR 0x0D or \r  (add 1 to include it)
        udpSent = 0u;
       while ((XSRequestUDP == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
        {
          XSRequestUDP = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
          udpSent=++udpSent % UINT8_MAX;
        }
        if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;          // accumulate the bad send counter (remove the 1 before summing)
        XSEncoderResp=XS_WAIT;
#ifdef UART4_INTERUPT
         memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
         UART4_write_text(Buffer);                                              // Send it to serial UART4
#endif
         XS_encoder_Q.settime == CMD_SENT;                                      // Advance to sent mode (timer will re-intialise if we didnt get a 1 in Xframe interval)
      }
      else if (XS_encoder_Q.settime == CMD_SENT)
      {
         switch(XSEncoderResp)
         {
            case XS_OK:                                                         // Success
            XS_encoder_Q.settime= CMD_COMPLETE;                                 // Complete the request
            XSRequestTimer=0u;                                                  // Reset the timer
            XS_encoder_Q.current=XS_CONTRAST;                                   // back to first item in sequence
            break;

            case XS_INVALID:                                                    // Time is invalid
            XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_TIME;                   // Alarm time or DATE specified is wrong
            XS_encoder_Q.settime= CMD_COMPLETE;                                 // Complete the request
            XSRequestTimer=0u;                                                  // Reset the timer
            XS_encoder_Q.current=XS_CONTRAST;                                   // back to first item in sequence
            break;

            default:                                                            // Otherwise do
            XSRequestTimer=++XSRequestTimer % UINT32_MAX;                       // Count the tick timer
            if (XSRequestTimer > MSG_REPOLL_TICKS)                              // We timed out waiting for response so just complete this message sequence, we would hang up the other requests
            {
               XS_encoder_Q.settime = CMD_COMPLETE;                             // Stop sending look for next request
               XSRequestTimer=0u;                                               // Reset the timer
               XS_encoder_Q.current=XS_CONTRAST;                                // back to first item in sequence
            }
            else if (XSRequestTimer > (MSG_REPOLL_TICKS/2u))                    // We are half way to timing out so send it again. (incase we missed the okay)
            {
               XS_encoder_Q.settime= CMD_SEND;                                  // Send another request
            }
            break;
         }
       }
       else if (XS_encoder_Q.settime == CMD_COMPLETE)                           // We complete so advance to the next poll element
       {
         XSRequestTimer=0u;                                                     // Reset the timer
         XS_encoder_Q.current=XS_CONTRAST;                                      // back to first item in sequence
       }
    }
#endif                                                                          // ============ eNd XS Encoder data requests =====================
#if defined(SBGC_GIMBAL_HMI)
if (gimStartUpComplete==true)
{
// ==================== SBGC GIMBAL REQUESTS ===================================
   // Now process the SBGC message requests ---------------------------------------------------------------------------------------------------
   //
   // Process the data from the GS Commander -- we read in commGSRunAScript from a message......
   //
   // Change the state of the motors from the GUI
   //
   if (commGSRunAScript == CMD_WRITTEN)                                         // from EEPROM write confirmation
   {
      commGSRunAScript = CMD_SSEND;
   }
   else if (commGSRunAScript == CMD_WCONFIRMED)                                  // From script confirmation
   {
      commGSRunAScript = CMD_WCOMPLETE;
   }
   else if (commGSRunAScript ==  CMD_SSENT)                                     // We sent the messaage wait for the re-poll timer
   {
        if (countRunAScript <= MSG_REPOLL_TICKS)                                // Counter is below the re-send delay off time (ticks)
        {
           countRunAScript=++countRunAScript % UINT64_MAX;                      // Count for the time to delay the re-send if no confirm message
        }
        else
        {
           countRunAScript=0u;                                                  // Reset the message counter
           commGSRunAScript = CMD_SSEND;                                        // Re-issue the message
        }
   }
   else if (commGSRunAScript ==  CMD_WSENT)                                     // We sent the messaage wait for the re-poll timer
   {
        if (countRunAScript <= MSG_REPOLL_TICKS)                                // Counter is below the re-send delay off time (ticks)
        {
           countRunAScript=++countRunAScript % UINT64_MAX;                      // Count for the time to delay the re-send if no confirm message
        }
        else
        {
           countRunAScript=0u;                                                  // Reset the message counter
           commGSRunAScript = CMD_WSEND;                                        // Re-issue the message
        }
   }
   if (commGSScriptSigLast != commGSScriptSig)                                  // received a command from the commander GS GUI
   {
      switch (commGSScriptSig)
      {
         case COMGS_RUN_NEW_SCRIPT:                                             // Com GS requested a new script to be written and ran on gimbal
            commGSRunAScript = CMD_WSEND;                                       // state engine begin eeprom write
            commGSScriptSigLast = commGSScriptSig;                              // save last and prevent re-start of sequence until comGS changes request
            break;
         case COMGS_RUN_A_SCRIPT:                                               // Com GS requested a run of the script (assumes at present only one script in slot one)
            commGSRunAScript = CMD_SSEND;                                       // state engine begin request to run script
            commGSScriptSigLast = commGSScriptSig;                              // save last and prevent re-start of sequence until comGS changes request
            break;
         case COMGS_STOP_SCRIPT:                                                // Com GS requested a stop the script (assumes at present only one script in slot one)
            commGSRunAScript = CMD_SSTOP;                                       // state engine begin request to stop script
            commGSScriptSigLast = commGSScriptSig;                              // save last and prevent re-start of sequence until comGS changes request
            break;
      }
   }

   if ((commGSRunAScript == CMD_WSEND) || (commGSRunAScript == CMD_SSEND))      // Change of state read from the Commander GS
   {

        if (commGSRunAScript == CMD_SSEND)
        {
          head.commandID = SBGC_CMD_RUN_SCRIPT;                                 // Issue run script State request
          head.sizeData=sizeof(runscript);                                      // Byte 3 of the header
          head.cmdSize=(head.commandID + head.sizeData) % 256U;                 // Byte 4 of the header
          runscript.MODE = 2U;                                                  // start the script with a debug feedback
          runscript.SLOT = 0U;                                                  // slot containing the script

          memcpy(Buffer, &head, sizeof(head));                                  // Copy the header for RunAScripts on
          memcpy(Buffer+sizeof(head), &runscript, head.sizeData);               // Copy the payload
          if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
          {
              newCRC = crc16_arc_calculate(sizeof(runscript), ptr_rsc);         // Calculate new 16 bit CRC for new version
              memcpy(Buffer+sizeof(runscript)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(runscript)+sizeof(newCRC);      // message length
          }
          else
          {
              crc = checksum(ptr_rsc, sizeof(runscript));                       // caculate the payload checksum
              memcpy(Buffer+sizeof(runscript)+sizeof(head),(void *) &crc, sizeof(crc));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(runscript)+sizeof(crc);         // message length
          }
          udpReturn = UDP_SEND;
        }
        else if (commGSRunAScript == CMD_WSEND)
        {
          head.commandID = SBGC_CMD_WRITE_FILE;                                 // Issue write file State request
          head.sizeData=sizeof(writescript);                                    // Byte 3 of the header
          head.cmdSize=(head.commandID + head.sizeData) % 256u;                 // Byte 4 of the header
          switch (comScriptParams.ScriptNumber)
          {
             case 0u:                                                           // Preset No.1
                createSBGCScript(&writescript);                                 // Write the script written inside the function createSBGCScript
                break;
             case 1u:                                                           // Preset No.2
               comScriptParams.angleReq = false;                                // start from current position
               comScriptParams.useDefault = true;                               // use the defaults in the script
               createSBGCTimelapse(&writescript, &comScriptParams);             // Write the configured timelapse sequence default
               break;
             case 2u:                                                           // Preset No.3
                comScriptParams.angleReq = true;                                // move to the angle requested before start
                comScriptParams.useDefault = false;                             // use params below
                comScriptParams.PA = 0u;                                        // pitch angle 0
                comScriptParams.RA = 0u;                                        // roll angle 0
                comScriptParams.YS = -0.1f;                                     // Pan right instead of left
                comScriptParams.PS = 0.05f;                                     // Down instead of up
                createSBGCTimelapse(&writescript, &comScriptParams);            // Write the configured timelapse sequence
             case 3:                                                            // Preset No.4
                comScriptParams.angleReq = true;                                // move to the angle requested before start
                comScriptParams.useDefault = false;                             // use params below
                comScriptParams.swapPitchRoll = true;                           // swap pitch with roll
                comScriptParams.swapRollYaw = true;                             // swap yaw with pitch
                comScriptParams.PA = 90u;                                       // move to these angles pitch 90 degrees
                comScriptParams.YA = 180u;                                      // yaw angle 180 degrees
                comScriptParams.YS = 0.1f;                                      // yaw speed 0,1 deg/sec pan left
                comScriptParams.RS = -0.1f;                                     // roll speed 0,1 deg/sec up
                createSBGCTimelapse(&writescript, &comScriptParams);             // Write the configured timelapse sequence
             default:
                createSBGCTimelapse(&writescript, &comScriptParams);            // Write the configured timelapse sequence with params as set elsewhere
          }

          memcpy(Buffer, &head, sizeof(head));                                  // Copy the header for RunAScripts on
          memcpy(Buffer+sizeof(head), &writescript, head.sizeData);             // Copy the payload
          if (boardinforb.FIRMWARE_VER >= 2680u)                                // If we have the new firmware loaded
          {
              newCRC = crc16_arc_calculate(sizeof(writescript), ptr_wsc);       // Calculate new 16 bit CRC for new version
              memcpy(Buffer+sizeof(writescript)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(writescript)+sizeof(newCRC);    // message length
          }
          else
          {
              crc = checksum(ptr_wsc, sizeof(writescript));                     // caculate the payload checksum
              memcpy(Buffer+sizeof(writescript)+sizeof(head),(void *) &crc,sizeof(crc));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(writescript)+sizeof(crc);       // message length
          }
          udpReturn = UDP_SEND;
        }
        else if (commGSRunAScript == CMD_SSTOP)
        {
          head.commandID = SBGC_CMD_RUN_SCRIPT;                                 // Issue run script State request
          head.sizeData=sizeof(runscript);                                      // Byte 3 of the header
          head.cmdSize=(head.commandID + head.sizeData) % 256U;                 // Byte 4 of the header
          runscript.MODE = 0U;                                                  // stop the script no feedback
          runscript.SLOT = 0U;                                                  // slot containing the script

          memcpy(Buffer, &head, sizeof(head));                                  // Copy the header for RunAScripts on
          memcpy(Buffer+sizeof(head), &runscript, head.sizeData);               // Copy the payload
          if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
          {
              newCRC = crc16_arc_calculate(sizeof(runscript), ptr_rsc);         // Calculate new 16 bit CRC for new version
              memcpy(Buffer+sizeof(runscript)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(runscript)+sizeof(newCRC);      // message length
          }
          else
          {
             crc = checksum(ptr_rsc, sizeof(runscript));                        // caculate the payload checksum
             memcpy(Buffer+sizeof(runscript)+sizeof(head),(void *) &crc,sizeof(crc));   // Tag the CRC
             totalMsgLen = sizeof(head)+sizeof(runscript)+sizeof(crc);          // message length
          }
          udpReturn = UDP_SEND;
        }

        if (udpReturn == UDP_SEND)                                              // We packaged a good request for the RunAScript Command
        {
          // Package the command in the Buffer and send to serial and UDP
          memset(Buffer+totalMsgLen,(char) '\0',sizeof(char));                  // Null terminate the Buffer for serial
          for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
          {
            UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
          }
          udpSent = 0u;
          while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {
            udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
            udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
          countRunAScript=0u;
          if ((udpReturn != UDP_SEND) && (commGSRunAScript == CMD_WSEND))       // We did send it so wait for another Com GS Request
          {
             commGSRunAScript = CMD_WSENT;                                      // We sent it okay so wait for a confirm
          }
          else if ((udpReturn != UDP_SEND) && (commGSRunAScript == CMD_SSEND))                                           // We did send it so wait for another Com GS Request
          {
             commGSRunAScript = CMD_SSENT;                                      // We sent it okay so wait for a confirm
          }
          else if ((udpReturn != UDP_SEND) && (commGSRunAScript == CMD_SSTOP))                                           // We did send it so wait for another Com GS Request
          {
             commGSRunAScript = CMD_WCOMPLETE;                                  // We sent it stop so finish as has no feedback
          }
        }
    }

   // Now process the SBGC message requests
   // Process the data from the GS Commander -- we read in commGSMotorsOn from a message......
   //
   // Change the state of the motors from the GUI
   //
   if (commGSMotor == CMD_CONFIRMED)                                            // We did send it so wait for another Com GS Request
   {
      commGSMotorsOnLast = commGSMotorsOn;                                      // We sent it okay and got confirm so stop sending.
      commGSMotor = CMD_COMPLETE;
   }
   else if (commGSMotor ==  CMD_SENT)                                           // We sent the messaage wait for the re-poll timer
   {
        if (countMotor <= MSG_REPOLL_TICKS)                                     // Counter is below the re-send delay off time (ticks)
        {
           countMotor=++countMotor % UINT64_MAX;                                // Count for the time to delay the re-send if no confirm message
        }
        else
        {
           countMotor=0u;                                                       // Reset the message counter
           commGSMotor = CMD_RESEND;                                            // Re-issue the message
        }
   }
   if ((commGSMotorsOnLast != commGSMotorsOn) && (commGSMotor != CMD_SENT))     // Change of state read from the Commander GS
   {
        commGSMotor = CMD_SEND;                                                 // request to compose and send
        udpReturn = UDP_NO_SEND;                                                // If we dont package a good request dont send it
        switch(commGSMotorsOn)                                                  // For the state of the switches from the Commander GS
        {
               case COMGS_ON:                                                   // Motor On State Request
                     head.commandID = SBGC_CMD_MOTORS_ON;                       // Issue Motor State request
                     head.sizeData=0u;                                          // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256u;      // Byte 4 of the header
                     gimbalMotor.ReqState = true;                               // indicate request to start to GUI
                     break;
                case COMGS_OFF_BREAK:                                           // Motor Off Break State Request
                     head.commandID = SBGC_CMD_MOTORS_OFF;                      // Issue Motor State request
                     head.sizeData=sizeof(motorOffState);                       // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256u;      // Byte 4 of the header
                     if ((boardinforb.FIRMWARE_VER >= 2687u) && (g_boardReady))   // use extended mode or not
                     {
                       motorOffState = SBGC_MOTOR_OFF_BREAK;
                       crc = (unsigned char) SBGC_MOTOR_OFF_BREAK;
                     }
                     commGSMotorsOn = SEND_MOTOR_OFF;                           // Send the Message out.
                     gimbalMotor.ReqState = false;                              // indicate request to stop to GUI
                     break;
                case COMGS_OFF_SAFE:                                            // Motor Off Safe State Request
                     head.commandID = SBGC_CMD_MOTORS_OFF;                      // Issue Motor State request
                     head.sizeData=sizeof(motorOffState);                             // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256u;      // Byte 4 of the header
                     if ((boardinforb.FIRMWARE_VER >= 2687u) && (g_boardReady))   // use extended mode or not
                     {
                       motorOffState = SBGC_MOTOR_OFF_SAFE;
                       crc = (unsigned char) SBGC_MOTOR_OFF_SAFE;
                     }
                     commGSMotorsOn = SEND_MOTOR_OFF;                           // Send the Message out.
                     gimbalMotor.ReqState = false;                              // indicate request to stop to GUI
                     break;
                case COMGS_OFF_NORMAL:                                          // Motor Off Normal State Request
                     head.commandID = SBGC_CMD_MOTORS_OFF;                      // Issue Motor State request
                     head.sizeData=sizeof(motorOffState);                             // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256u;      // Byte 4 of the header
                     if ((boardinforb.FIRMWARE_VER >= 2687u) && (g_boardReady))   // use extended mode or not
                     {
                       motorOffState = SBGC_MOTOR_OFF_NORMAL;
                       crc = (unsigned char) SBGC_MOTOR_OFF_NORMAL;
                     }
                     commGSMotorsOn = SEND_MOTOR_OFF;                           // Send the Message out.
                     gimbalMotor.ReqState = false;                              // indicate request to stop to GUI
                     break;

        }

        if (commGSMotorsOn = SEND_MOTOR_OFF)                                    // Send the off Message out.
        {
          memcpy(Buffer, &head, sizeof(head));                                  // Copy the header for motors off
          if ((boardinforb.FIRMWARE_VER >= 2687U) && (g_boardReady))              // use extended mode or not
          {
             memcpy(Buffer+sizeof(head), &motorOffState, head.sizeData);        // Copy the payload
             memcpy(Buffer+sizeof(motorOffState)+sizeof(head),(void *) &newCRC,sizeof(newCRC));   // Tag the CRC
             totalMsgLen = sizeof(head)+sizeof(motorOffState)+sizeof(newCRC);   // message length
          }
          else
          {
             totalMsgLen = sizeof(head);
          }
          udpReturn = UDP_SEND;
        }
        else if (commGSMotorsOn = SEND_MOTOR_ON)                                // Send the on Message out.
        {
          memcpy(Buffer, &head, sizeof(head));                                  // Copy the header for motors on
          totalMsgLen = sizeof(head);                                           // message length
          udpReturn = UDP_SEND;
        }

        if (udpReturn == UDP_SEND)                                              // We packaged a good request for the Motor Command
        {
          // Package the command in the Buffer and send to serial and UDP
          stateEngine = stateEngine | GSCMotorSend;                             // Flash the LED to say sending motor command from GS
          memset(Buffer+totalMsgLen,(char) '\0',sizeof(char));                         // Null terminate the Buffer for serial
          for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
          {
             UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
          }
          udpSent = 0U;
          while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {
            udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
            udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
          if (udpReturn != UDP_SEND)                                            // We did send it so wait for another Com GS Request
          {
             commGSMotor = CMD_SENT;                                            // We sent it okay so wait for a confirm
          }
        }
    }

   // Process the data from the GS Commander -- we read in commGSAutoPID from a message......
   //
   //  PID Tune Request
   //
   if (commGSPID = CMD_CONFIRMED)                                               // We did confirm it so wait for new Com GS Request
   {
      commGSAutoPIDLast = commGSAutoPID;                                        // We sent it okay so leave the change of state state (no more send)
      commGSPID = CMD_COMPLETE;
   }
   else if (commGSPID ==  CMD_SENT)                                             // We sent the messaage wait for the re-poll timer
   {
        if (countPID <= MSG_REPOLL_TICKS)                                       // Counter is below the re-send delay off time (ticks)
        {
           countPID=++countPID % UINT64_MAX;                                             // Count for the time to delay the re-send if no confirm message
        }
        else
        {
           countPID=0u;                                                         // Reset the message counter
           commGSPID = CMD_RESEND;                                              // Re-issue the message
        }
   }
   if (((commGSAutoPIDLast != commGSAutoPID) && (commGSPID != CMD_SENT)) && ((boardinforb.FIRMWARE_VER < 2700) && (g_boardReady)))      // Change of state read from the Commander GS
   {
        udpReturn = UDP_NO_SEND;                                                // If we dont package a good request dont send it
        head.commandID = SBGC_CMD_AUTO_PID;                                     // Issue PID Tune State request
        head.sizeData=sizeof(pida);                                             // Byte 3 of the header
        head.cmdSize=(head.commandID + head.sizeData) % 256U;                   // Byte 4 of the header

        switch(commGSAutoPID)                                                   // For the selected command variables passed to autotune from the Commander GS
        {
               case COMGS_PID_STOP:                                             // PID Stop State Request
                     pida.profile_id = selectedProfile;                         // Select the PID Profile
                     pida.cfg_flags = SBGC_PID_STOP;
                     pida.gain_vs_stability = selectedGVS;
                     pida.momentum = SBGC_MOM_AUTO;
                     pida.action = SBGC_START_TUNE;
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
                case COMGS_PID_ROLL:                                            // Roll PID Autotune State Request
                     pida.profile_id = selectedProfile;                         // Select the PID Profile
                     pida.cfg_flags = SBGC_PID_ROLL;
                     pida.gain_vs_stability = selectedGVS;
                     pida.momentum = SBGC_MOM_AUTO;
                     pida.action = SBGC_START_TUNE;
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
                case COMGS_PID_PITCH:                                            // Pitch PID Autotune State Request
                     pida.profile_id = selectedProfile;                         // Select the PID Profile
                     pida.cfg_flags = SBGC_PID_PITCH;
                     pida.gain_vs_stability = selectedGVS;
                     pida.momentum = SBGC_MOM_AUTO;
                     pida.action = SBGC_START_TUNE;
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
                case COMGS_PID_YAW:                                             // Yaw PID Autotune State Request
                     pida.profile_id = selectedProfile;                         // Select the PID Profile
                     pida.cfg_flags = SBGC_PID_YAW;
                     pida.gain_vs_stability = selectedGVS;
                     pida.momentum = SBGC_MOM_AUTO;
                     pida.action = SBGC_START_TUNE;
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
                case COMGS_PID_GUI:                                             // PID Autotune with feedback
                     pida.profile_id = selectedProfile;                         // Select the PID Profile
                     pida.cfg_flags = SBGC_PID_GUI;
                     pida.gain_vs_stability = selectedGVS;
                     pida.momentum = SBGC_MOM_AUTO;
                     pida.action = SBGC_START_TUNE;
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
                case COMGS_PID_KEEP:                                            // PID autotune start from current State Request
                     pida.profile_id = selectedProfile;                         // Select the PID Profile
                     pida.cfg_flags = SBGC_PID_KEEP;
                     pida.gain_vs_stability = selectedGVS;
                     pida.momentum = SBGC_MOM_AUTO;
                     pida.action = SBGC_START_TUNE;
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
                case COMGS_PID_LPF:                                             // PID Lpf State Request
                     pida.profile_id = selectedProfile;                         // Select the PID Profile
                     pida.cfg_flags = SBGC_PID_LPF;
                     pida.gain_vs_stability = selectedGVS;
                     pida.momentum = SBGC_MOM_AUTO;
                     pida.action = SBGC_START_TUNE;
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
        }
        if (udpReturn == UDP_SEND)                                              // We packaged a good request for the Motor Command
        {
           memcpy(Buffer, &head, sizeof(head));                                 // Copy the header for pid autotune
           memcpy(Buffer+sizeof(head), &pida, head.sizeData);                   // Copy the payload
           if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
           {
              newCRC = crc16_arc_calculate(sz_pida, ptr_pida);                  // Calculate new 16 bit CRC for new version
              memcpy(Buffer+sizeof(pida)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(pida)+sizeof(newCRC);           // message length
           }
           else
           {
              crc = checksum(ptr_pida, sz_pida);                                // calculate the payload checksum
              memcpy(Buffer+sizeof(pida)+sizeof(head),(void *) &crc,sizeof(crc));// Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(pida)+sizeof(crc);              // message length
           }

          // Package the command in the Buffer and send to serial and UDP
          stateEngine = stateEngine | GSCPIDAutotune;                           // Flash the LED to say sending motor command from GS
          memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));      // Null terminate the Buffer for serial
          for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
          {
             UART2_write(Buffer[SBGCcnt]);                                      // Send it to serial UART2
          }
          udpSent = 0u;
          while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {
            udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
            udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
          if (udpReturn != UDP_SEND)                                            // We did send it so wait for another Com GS Request
          {
             commGSPID = CMD_SENT;                                              // We sent it okay so wait for a confirm reply message
          }
        }
    }
    else if (((commGSAutoPIDLast != commGSAutoPID) && (commGSPID != CMD_SENT)) && ((boardinforb.FIRMWARE_VER >= 2700u) && (g_boardReady)))
    {
        udpReturn = UDP_NO_SEND;                                                // If we dont package a good request dont send it
        head.commandID = SBGC_CMD_AUTO_PID2;                                    // Issue PID Tune State request
        head.sizeData=sizeof(pida2);                                            // Byte 3 of the header
        head.cmdSize=(head.commandID + head.sizeData) % 256u;                   // Byte 4 of the header

        switch(commGSAutoPID)                                                   // For the selected command variables passed to autotune from the Commander GS
        {
               case COMGS_PID2_ACTION_START:                                    // PID start tuning (do not update config in EEPROM)
                     pida2.action = SBGC_PID2_ACTION_START;                     // Select the PID action request
                     pida2.cfg_version = 1u;
                     pida2.axis_flags[1u] = SBGC_PID2_AXIS_ENABLE + SBGC_PID2_AXIS_TUNELPF + SBGC_PID2_NOTCH_FILTERS3;
                     pida2.axis_flags[2u] = SBGC_PID2_AXIS_ENABLE + SBGC_PID2_AXIS_TUNELPF + SBGC_PID2_NOTCH_FILTERS3;
                     pida2.axis_flags[3u] = SBGC_PID2_AXIS_ENABLE + SBGC_PID2_AXIS_TUNELPF + SBGC_PID2_NOTCH_FILTERS3;
                     pida2.gain[1u] = selectedGVS;                               // Chosen gain / stability
                     pida2.gain[2u] = selectedGVS;
                     pida2.gain[3u] = selectedGVS;
                     pida2.general_flags = SBGC_PID2_USE_CURRENT;               // Start using the current values
                     pida2.startup_cfg = SBGC_PID2_TUNE_ALL;
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
                case COMGS_PID2_ACTION_START_SAVE:                              // save config to EEPROM and start tuning
                     pida2.action = SBGC_PID2_ACTION_START_SAVE;                // Select the PID action request
                     pida2.cfg_version = 1u;
                     pida2.axis_flags[1u] = SBGC_PID2_AXIS_ENABLE + SBGC_PID2_AXIS_TUNELPF + SBGC_PID2_NOTCH_FILTERS3;
                     pida2.axis_flags[2u] = SBGC_PID2_AXIS_ENABLE + SBGC_PID2_AXIS_TUNELPF + SBGC_PID2_NOTCH_FILTERS3;
                     pida2.axis_flags[3u] = SBGC_PID2_AXIS_ENABLE + SBGC_PID2_AXIS_TUNELPF + SBGC_PID2_NOTCH_FILTERS3;
                     pida2.gain[1] = selectedGVS;
                     pida2.gain[2] = selectedGVS;
                     pida2.gain[3] = selectedGVS;
                     pida2.general_flags = SBGC_PID2_SAVE_TO_ALL;               // Save to all profiles start at zero
                     pida2.startup_cfg = SBGC_PID2_TUNE_ALL;
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
                case COMGS_PID2_ACTION_SAVE:                                    // save config to EEPROM
                     pida2.action = SBGC_PID2_ACTION_SAVE;                      // Select the PID action request
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
                case COMGS_PID2_ACTION_STOP:                                     //stop tuning
                     pida2.action = SBGC_PID2_ACTION_STOP;                      // Select the PID action request
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
                case COMGS_PID2_ACTION_READ:                                     //read config from EEPROM
                     pida2.action = SBGC_PID2_ACTION_READ;                      // Select the PID action request
                     udpReturn = UDP_SEND;                                      // Formulated a message so send it.
                     break;
        }
        if (udpReturn == UDP_SEND)                                              // We packaged a good request for the Motor Command
        {
           memcpy(Buffer, &head, sizeof(head));                                 // Copy the header for pid autotune
           memcpy(Buffer+sizeof(head), &pida2, head.sizeData);                  // Copy the payload
           if (boardinforb.FIRMWARE_VER >= 2680U)                               // If we have the new firmware loaded
           {
              newCRC = crc16_arc_calculate(sizeof(pida2),ptr_pida2);            // Calculate new 16 bit CRC for new version
              memcpy(Buffer+sizeof(pida2)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(pida2)+sizeof(newCRC);          // message length
           }
           else
           {
              crc = checksum(ptr_pida2, sizeof(pida2));                         // calculate the payload checksum
              memcpy(Buffer+sizeof(pida2)+sizeof(head),(void *) &crc,sizeof(crc)); // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(pida2)+sizeof(crc);             // message length
           }

          // Package the command in the Buffer and send to serial and UDP
          stateEngine = stateEngine | GSCPIDAutotune;                           // Flash the LED to say sending motor command from GS
          memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));      // Null terminate the Buffer for serial
          for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
          {
            UART2_write(Buffer[SBGCcnt]);                                       // Send it to serial UART2
          }
          udpSent = 0u;
          while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {
            udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
            udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
          if (udpReturn != UDP_SEND)                                            // We did send it so wait for another Com GS Request
          {
             commGSPID = CMD_SENT;                                              // We sent it okay so wait for a confirm reply message
          }
        }
    }

   // Now process the command execute menu commands from the Com GS
   //
   //  Menu send from GUI
   //
   if (commGSMenu == CMD_CONFIRMED)                                             // We did send it so wait for another Com GS Request
   {
      commGSMenuLast = commGSMenu;                                              // We sent it okay and got confirm so stop sending.
      commGSMenu = CMD_COMPLETE;
      DOTPORT2 = DOTPORT2 & !GSCMenuSend;                                       // Remove the LED indicator
   }
   else if (commGSMenu == CMD_SENT)
      {
        if (countMenu <= MSG_REPOLL_TICKS)                                      // Counter is below the re-send delay off time (ticks)
        {
           countMenu=++countMenu % UINT64_MAX;                                  // Count for the time to delay the re-send if no confirm message
        }
        else
        {
           countMenu=0u;                                                        // Reset the message counter
           commGSMenu = CMD_RESEND;                                             // Re-issue the message
        }
   }
   if ((commGSMenuLast != commGSMenu) && (commGSMenu != CMD_SENT))              // Change of state read from the Commander GS
   {
        commGSMenu = CMD_COMPLETE;                                              // no request unless it matches the case
        udpReturn = UDP_NO_SEND;                                                // If we dont package a good request dont send it
        switch(commGSMenu)                                                      // For the state of the switches from the Commander GS
        {
               case COMGS_PROFILE3:                                             // Profile3 State Request
                     head.commandID = SBGC_CMD_EXECUTE_MENU;                    // Issue Menu request to execute
                     head.sizeData=sizeof(menu);                                // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256u;      // Byte 4 of the header
                     menu.cmd_id = MENU_CMD_PROFILE3;                           // Payload requested
                     if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
                     {
                         newCRC=0x140U;                                         // Calculate new 16 bit CRC for new version
                     }
                     else
                     {
                         crc = (unsigned char) MENU_CMD_PROFILE3;
                     }
                     commGSMenu = CMD_SEND;                                     // Send it
                     break;
                case COMGS_CALIB_ACC:                                           // Calibrate Accelorometer State Request
                     head.commandID = SBGC_CMD_EXECUTE_MENU;                    // Issue Menu request to execute
                     head.sizeData=sizeof(menu);                                // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     menu.cmd_id = MENU_CMD_CALIB_ACC;                          // Payload requested
                     if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
                     {
                         newCRC=0x280U;                                         // Calculate new 16 bit CRC for new version
                     }
                     else
                     {
                         crc = (unsigned char) MENU_CMD_CALIB_ACC;
                     }
                     commGSMenu = CMD_SEND;                                     // Send it
                     break;
                case COMGS_CALIB_GYRO:                                          // Calibrate Gyro State Request
                     head.commandID = SBGC_CMD_EXECUTE_MENU;                    // Issue Menu request to execute
                     head.sizeData=sizeof(menu);                                // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     menu.cmd_id = MENU_CMD_CALIB_GYRO;                         // Payload requested
                     if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
                     {
                         newCRC=0x6C0U;                                         // Calculate new 16 bit CRC for new version
                     }
                     else
                     {
                         crc = (unsigned char) MENU_CMD_CALIB_GYRO;
                     }
                     commGSMenu = CMD_SEND;                                     // Send it
                     break;
                case COMGS_CALIB_MAG:                                           // Calibrate Magnetometer State Request
                     head.commandID = SBGC_CMD_EXECUTE_MENU;                    // Issue Menu request to execute
                     head.sizeData=sizeof(menu);                                // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     menu.cmd_id = MENU_CMD_CALIB_MAG;                          // Payload requested
                     if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
                     {
                         newCRC=0x1540U;                                        // Calculate new 16 bit CRC for new version
                     }
                     else
                     {
                         crc = (unsigned char) MENU_CMD_CALIB_MAG;
                     }
                     commGSMenu = CMD_SEND;                                     // Send it
                     break;
                case COMGS_MOTOR_TOGGLE:                                        // Motor Toggle State Request
                     head.commandID = SBGC_CMD_EXECUTE_MENU;                    // Issue Menu request to execute
                     head.sizeData=sizeof(menu);                                // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     menu.cmd_id = MENU_CMD_MOTOR_TOGGLE;                       // Payload requested
                     if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
                     {
                         newCRC=0xCC01U;                                        // Calculate new 16 bit CRC for new version
                     }
                     else
                     {
                         crc = (unsigned char) MENU_CMD_MOTOR_TOGGLE;
                     }
                     commGSMenu = CMD_SEND;                                     // Send it
                     break;
                case COMGS_HOME_POS:                                            // Home position State Request
                     head.commandID = SBGC_CMD_EXECUTE_MENU;                    // Issue Menu request to execute
                     head.sizeData=sizeof(menu);                                // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     menu.cmd_id = MENU_CMD_HOME_POSITION;                      // Payload requested
                     if (boardinforb.FIRMWARE_VER >= 2680U)                     // If we have the new firmware loaded
                     {
                         newCRC=0xA00U;                                         // Calculate new 16 bit CRC for new version
                     }
                     else
                     {
                         crc = (unsigned char) MENU_CMD_HOME_POSITION;
                     }
                     commGSMenu = CMD_SEND;                                     // Send it
                     break;
                case COMGS_CAM_EV:                                              // Camera Record Event State Request
                     head.commandID = SBGC_CMD_EXECUTE_MENU;                    // Issue Menu request to execute
                     head.sizeData=sizeof(menu);                                // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     menu.cmd_id = MENU_CAMERA_REC_PHOTO_EVENT;                 // Payload requested
                     if (boardinforb.FIRMWARE_VER >= 2680U)                     // If we have the new firmware loaded
                     {
                         newCRC=0xFA01U;                                        // Calculate new 16 bit CRC for new version
                     }
                     else
                     {
                         crc = (unsigned char) MENU_CAMERA_REC_PHOTO_EVENT;
                     }
                     commGSMenu = CMD_SEND;                                     // Send it
                     break;
                case COMGS_PH_EV:                                               // Camera Event State Request
                     head.commandID = SBGC_CMD_EXECUTE_MENU;                    // Issue Menu request to execute
                     head.sizeData=sizeof(menu);                                // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     menu.cmd_id = MENU_CAMERA_PHOTO_EVENT;                     // Payload requested
                     if (boardinforb.FIRMWARE_VER >= 2680U)                     // If we have the new firmware loaded
                     {
                         newCRC=0x3AC0U;                                        // Calculate new 16 bit CRC for new version
                     }
                     else
                     {
                         crc = (unsigned char) MENU_CAMERA_PHOTO_EVENT;
                     }
                     commGSMenu = CMD_SEND;                                     // Send it
                     break;

        }

        if (commGSMenu == CMD_SEND)                                             // Send the Message out if we had a valid request
        {
          memcpy(Buffer, &head, sizeof(head));                                  // Copy the header for menu execute
          memcpy(Buffer+sizeof(head), &menu, head.sizeData);                    // Copy the payload
          if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
          {
               memcpy(Buffer+sizeof(menu)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
               totalMsgLen = sizeof(head)+sizeof(menu)+sizeof(newCRC);          // message length
           }
           else
           {
              memcpy(Buffer+sizeof(menu)+sizeof(head),&crc,sizeof(crc));        // Tag the CRC
              totalMsgLen = sizeof(head) + sizeof(menu) + sizeof(crc);          // The message length
           }
           udpReturn = UDP_SEND;
        }
        if (udpReturn == UDP_SEND)                                              // We packaged a good request for the menu Command
        {
          // Package the command in the Buffer and send to serial and UDP
          DOTPORT2 = DOTPORT2 | GSCMenuSend;                                    // Flash the LED to say menu command from GS
          memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));      // Null terminate the Buffer for serial
          for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
          {
             UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
          }
          udpSent = 0u;
          while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {                                                                     // send the Buffer contents as UDP datagram.
            udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);
            udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
          if (udpReturn != UDP_SEND)                                            // We did send it so wait for another Com GS Request
          {
             commGSMenu = CMD_SENT;                                             // We sent it okay so wait for a confirm
          }
        }
        // When there is not going to be confirm sent back by the action (just ack it back -- Should link to UDP ACK
    }

   // Now process the command set adj vars val from the Com GS
   //
   if (commGSAdjvar == CMD_CONFIRMED)                                           // We did send it so wait for another Com GS Request
   {
      commGSAdjvarLast = commGSAdjvar;                                          // We sent it okay and got confirm so stop sending.
      commGSAdjvar = CMD_COMPLETE;
      DOTPORT2 = DOTPORT2 & !GSCAdjvar;                                         // Take the LED off when we confirmed it
   }
   else if (commGSAdjvar ==  CMD_SENT)                                          // We sent the messaage wait for the re-poll timer
   {
        if (countAdjvar <= MSG_REPOLL_TICKS)                                    // Counter is below the re-send delay off time (ticks)
        {
           countAdjvar=++countAdjvar % UINT64_MAX;                                          // Count for the time to delay the re-send if no confirm message
        }
        else
        {
           countAdjvar=0u;                                                      // Reset the message counter
           commGSAdjvar = CMD_RESEND;                                           // Re-issue the message
        }
   }
   if ((commGSAdjvarLast != commGSAdjvar) && (commGSAdjvar != CMD_SENT))        // Change of state read from the Commander GS
   {
        commGSAdjvar = CMD_COMPLETE;                                            // dont send if we dont recognise the mode from COM GS
        udpReturn = UDP_NO_SEND;                                                // If we dont package a good request dont send it
        switch(commGSAdjvar)                                                    // For the state of the switches from the Commander GS
        {
               case COMGS_SPEED100:                                             // Speed of all axis to 100
                     head.commandID = SBGC_CMD_SET_ADJ_VARS_VAL;                // Issue adjvar (wire configurable) request to execute
                     head.sizeData=sizeof(adjvar);                              // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     adjvar.num_params = 3U;                                    // Payload requested
                     adjvar.param1_id= SBGC_RC_SPEED_ROLL;                      // Configure 1st wire to rc speed roll
                     adjvar.param1_value= 100U;                                 // Set the value
                     adjvar.param2_id= SBGC_RC_SPEED_PITCH;                     // Configure the 2nd wire to rc speed pitch
                     adjvar.param2_value=100U;
                     adjvar.param3_id= SBGC_RC_SPEED_YAW;                       // Configure the 3rd wire to rc speed yaw
                     adjvar.param3_value= 100U;
                     //crc = checksum(ptr_adjvar, sizeof(adjvar));                // caculate the payload checksum
                     commGSAdjvar = CMD_SEND;                                   // Send it
                     break;
               case COMGS_FOLLOW60:                                             // Speed of follow mode control all axis to 60
                     head.commandID = SBGC_CMD_SET_ADJ_VARS_VAL;                // Issue adjvar (wire configurable) request to execute
                     head.sizeData=sizeof(adjvar);                              // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     adjvar.num_params = 3U;                                    // Payload requested
                     adjvar.param1_id= SBGC_FOLLOW_SPEED_ROLL;                  // Configure 1st wire to follow speed roll
                     adjvar.param1_value= 60U;                                  // Set the value
                     adjvar.param2_id= SBGC_FOLLOW_SPEED_PITCH;                 // Configure the 2nd wire to follow speed pitch
                     adjvar.param2_value=60U;
                     adjvar.param3_id= SBGC_FOLLOW_SPEED_YAW;                   // Configure the 3rd wire to follow speed yaw
                     adjvar.param3_value= 60U;
                     //crc = checksum(ptr_adjvar, sizeof(adjvar));                // caculate the payload checksum
                     commGSAdjvar = CMD_SEND;                                   // Send it
                     break;
               case MAV_SET_SPEED:
                     head.commandID = SBGC_CMD_SET_ADJ_VARS_VAL;                // Issue adjvar (wire configurable) request to execute
                     head.sizeData=sizeof(adjvar);                              // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     adjvar.num_params = 3U;                                    // Payload requested
                     adjvar.param1_id= SBGC_FOLLOW_SPEED_ROLL;                  // Configure 1st wire to follow speed roll
                     adjvar.param2_id= SBGC_FOLLOW_SPEED_PITCH;                 // Configure the 2nd wire to follow speed pitch
                     adjvar.param3_id= SBGC_FOLLOW_SPEED_YAW;                   // Configure the 3rd wire to follow speed yaw
                     commGSAdjvar = CMD_SEND;                                   // Send it
                     break;
               case MAV_SET_ANGLE:
                     head.commandID = SBGC_CMD_SET_ADJ_VARS_VAL;                // Issue adjvar (wire configurable) request to execute
                     head.sizeData=sizeof(adjvar);                              // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     adjvar.num_params = 4U;                                    // Payload requested
                     adjvar.param1_id= SBGC_FOLLOW_RANGE_ROLL;                  // Configure 1st wire to follow angle roll from quartenion object
                     adjvar.param2_id= SBGC_FOLLOW_RANGE_PITCH;                 // Configure the 2nd wire to follow angle pitch from quartenion object
                     adjvar.param3_id= SBGC_FOLLOW_RANGE_YAW;                   // Configure the 3rd wire to follow angle yaw from quartenion object
                     adjvar.param4_id= SBGC_FRAME_HEADING_ANGLE;                // Configure the 3rd wire to follow angle from quartenion object
                     commGSAdjvar = CMD_SEND;                                   // Send it
                     break;
               case COMGS_PIDGAIN:                                              // All PID P (gain) to variable
                     head.commandID = SBGC_CMD_SET_ADJ_VARS_VAL;                // Issue adjvar (wire configurable) request to execute
                     head.sizeData=sizeof(adjvar);                              // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     adjvar.num_params = 3U;                                     // Payload requested
                     adjvar.param1_id= SBGC_PID_GAIN_ROLL;                      // Configure 1st wire to follow speed roll
                     adjvar.param1_value= SBGC_PID_TO_COMMAND(commGSPIDGain);      // Set the value
                     adjvar.param2_id= SBGC_PID_GAIN_PITCH;                     // Configure the 2nd wire to follow speed pitch
                     adjvar.param2_value=SBGC_PID_TO_COMMAND(commGSPIDGain);
                     adjvar.param3_id= SBGC_PID_GAIN_YAW;                       // Configure the 3rd wire to follow speed yaw
                     adjvar.param3_value=SBGC_PID_TO_COMMAND(commGSPIDGain);
                     //crc = checksum(ptr_adjvar, sizeof(adjvar));                // caculate the payload checksum
                     commGSAdjvar = CMD_SEND;                                   // Send it
                     break;
        }

        if (commGSAdjvar == CMD_SEND)                                           // Send the Message out if we had a valid request
        {
            memcpy(Buffer, &head, sizeof(head));                                // Copy the header for motors off
            memcpy(Buffer+sizeof(head), &adjvar, head.sizeData);                // Copy the payload
            if (boardinforb.FIRMWARE_VER >= 2680U)                              // If we have the new firmware loaded
            {
               newCRC = crc16_arc_calculate(sizeof(adjvar), ptr_adjvar);        // Calculate new 16 bit CRC for new version
               memcpy(Buffer+sizeof(adjvar)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
               totalMsgLen = sizeof(head)+sizeof(adjvar)+sizeof(newCRC);        // message length
            }
            else
            {
               crc = checksum(ptr_adjvar, sizeof(adjvar));                      // Calculate the payload checksum
               memcpy(Buffer+sizeof(adjvar)+sizeof(head),(void *) &crc,sizeof(crc)); // Tag the CRC
               totalMsgLen = sizeof(head)+sizeof(adjvar)+sizeof(crc);           // message length
            }
           udpReturn = UDP_SEND;
        }
        if (udpReturn == UDP_SEND)                                              // We packaged a good request for the motor command
        {
          // Package the command in the Buffer and send to serial and UDP
          DOTPORT2 = DOTPORT2 | GSCAdjvar;                                      // Flash the LED to say sending adjustable variables command from GS
          memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));      // Null terminate the Buffer for serial
          for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
          {
             UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
          }
          udpSent = 0u;
          while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {
            udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
            udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
          if (udpReturn != UDP_SEND)                                            // We did send it so wait for another Com GS Request
          {
             commGSAdjvar = CMD_SENT;                                           // We sent it okay so wait for a confirm
          }
        }
    }

   // Now process the command get adj vars val from the Com GS  - we can read 7 items per structure
   //
   if (commGSGetvar = CMD_CONFIRMED)                                            // We did send it so wait for another Com GS Request
   {
      commGSGetvarLast = commGSGetvar;                                          // We sent it okay and got confirm so stop sending.
      commGSGetvar = CMD_COMPLETE;
      DOTPORT2 = DOTPORT2 & !GSCGetvar;                                         // Take the LED off when we confirmed it
   }
   else if (commGSGetvar ==  CMD_SENT)                                          // We sent the messaage wait for the re-poll timer
   {
        if (countGetvar <= MSG_REPOLL_TICKS)                                    // Counter is below the re-send delay off time (ticks)
        {
           countGetvar=++countGetvar % UINT64_MAX;                                          // Count for the time to delay the re-send if no confirm message
        }
        else
        {
           countGetvar=0u;                                                       // Reset the message counter
           commGSGetvar = CMD_RESEND;                                           // Re-issue the message
        }
   }
   if ((commGSGetvarLast != commGSGetvar) && (commGSGetvarLast != CMD_SENT))    // Change of state read from the Commander GS
   {
        commGSGetvar = CMD_COMPLETE;                                            // dont send if we dont recognise the mode from COM GS
        udpReturn = UDP_NO_SEND;                                                // If we dont package a good request dont send it
        switch(commGSGetvar)                                                    // For the state of the switches from the Commander GS
        {
               case COMGS_GET7_1:                                               // Request 7 parameters
                     head.commandID = SBGC_CMD_GET_ADJ_VARS_VAL;                // Issue adjvar (wire configurable) request to execute
                     head.sizeData=sizeof(getvar);                              // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256u;      // Byte 4 of the header
                     getvar.num_params = 7u;                                    // Payload requested
                     getvar.param1_id= SBGC_RC_SPEED_ROLL;                      // Configure 1st wire to rc speed roll
                     getvar.param2_id= SBGC_RC_SPEED_PITCH;                     // Configure the 2nd wire to rc speed pitch
                     getvar.param3_id= SBGC_RC_SPEED_YAW;                       // Configure the 3rd wire to rc speed yaw
                     getvar.param4_id= SBGC_FRAME_HEADING_ANGLE;                // Configure the 4th wire to frame heading angle
                     getvar.param5_id= SBGC_H_CORR_FACTOR;                      // Configure the 5th wire to H Corr Factor
                     getvar.param6_id= SBGC_GYRO_TRUST;                         // Configure the 6th wire to Gyro Thrust
                     getvar.param7_id= SBGC_FOLLOW_EXPO_RATE;                   // Configure the 7th wire to Follow expo rate
                     //crc = checksum(ptr_getvar, sizeof(getvar));                // caculate the payload checksum
                     commGSGetvar = CMD_SEND;                                   // Send it
                     break;

        }

        if (commGSGetvar == CMD_SEND)                                           // Send the Message out if we had a valid request
        {
           memcpy(Buffer, &head, sizeof(head));                                 // Copy the header for motors off
           memcpy(Buffer+sizeof(head), &getvar, head.sizeData);                 // Copy the payload
           if (boardinforb.FIRMWARE_VER >= 2680U)                               // If we have the new firmware loaded
           {
              newCRC = crc16_arc_calculate(sizeof(getvar), ptr_getvar);         // Calculate new 16 bit CRC for new version
              memcpy(Buffer+sizeof(getvar)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(getvar)+sizeof(newCRC);         // message length
           }
           else
           {
              crc = checksum(ptr_getvar, sizeof(getvar));                       // calculate the payload checksum
              memcpy(Buffer+sizeof(getvar)+sizeof(head),(void *) &crc,sizeof(crc)); // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(getvar)+sizeof(crc);            // message length
           }
           udpReturn = UDP_SEND;
        }
        if (udpReturn == UDP_SEND)                                              // We packaged a good request for the Motor Command
        {
          // Package the command in the Buffer and send to serial and UDP
          DOTPORT2 = DOTPORT2 | GSCGetvar;                                      // Flash the LED to say sending adjustable variables command from GS
          memset(Buffer+totalMsgLen,'\0',sizeof(char));                         // Null terminate the Buffer for serial
          for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
          {
             UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
          }
          udpSent = 0u;
          while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {
            udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
            udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
          if (udpReturn != UDP_SEND)                                            // We did send it so wait for another Com GS Request
          {
             commGSGetvar = CMD_SENT;                                           // We sent it okay so wait for a confirm
          }
        }
    }

   //
   //  Now process a request to calibrate the motor feeback encoders
   //
   if ((commGSEncLast != commGSEnc) && (commGSEnc != CMD_SENT))                 // Change of state read from the Commander GS
   {
       head.commandID = SBGC_CMD_ENCODERS_CALIB_OFFSET_4 ;                      // select encoder calibrate message type
       head.sizeData=0u;                                                        // Byte 3 of the header
       head.cmdSize=(head.commandID + head.sizeData) % 256U;                    // Byte 4 of the header
       crc = 0u;                                                                // caculate the payload checksum
       commGSEnc = CMD_SEND;                                                    // Send it
   }

   if (commGSEnc == CMD_SEND)                                                   // Send the Message out if we had a valid request
   {
      memcpy(Buffer, &head, sizeof(head));                                      // Copy the header for imu selection
      totalMsgLen = sizeof(head);                                               // The message length
      udpReturn = UDP_SEND;
   }
   if (udpReturn == UDP_SEND)                                                   // We packaged a good request for the Motor Command
   {
      gimbalMotor.EcalErr = !gimbalMotor.State;                                 // Set a calibration alarm if the motor is not already running
      memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                    // Null terminate the Buffer for serial
      for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
      {
        UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
      }
      udpSent = 0U;
      while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
      {                                                                         // Send the Buffer contents as UDP datagram.
         udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);
         udpSent=++udpSent % UINT8_MAX;
      }
      if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;            // accumulate the bad send counter (remove the 1 before summing)
      if (udpReturn != UDP_SEND)                                                // We did send it so wait for another Com GS Request
      {
         commGSEnc = CMD_SENT;                                                  // We sent it okay so wait for a confirm
      }
    }

   //
   // Now process the command IMU_SELECT
   //
   if (commGSImu == CMD_CONFIRMED)                                              // We did send it so wait for another Com GS Request
   {
      commGSImuLast = commGSImu;                                                // We sent it okay and got confirm so stop sending.
      commGSImu = CMD_COMPLETE;
      DOTPORT2 = DOTPORT2 & !GSCImu;                                            // Take the LED off when we confirmed it
   }
   else if (commGSImu ==  CMD_SENT)                                             // We sent the messaage wait for the re-poll timer
   {
        if (countImu <= MSG_REPOLL_TICKS)                                       // Counter is below the re-send delay off time (ticks)
        {
           countImu=++countImu % UINT64_MAX;                                             // Count for the time to delay the re-send if no confirm message
        }
        else
        {
           countImu=0u;                                                         // Reset the message counter
           commGSImu = CMD_RESEND;                                              // Re-issue the message
        }
   }
   if ((commGSImuLast != commGSImu) && (commGSImu != CMD_SENT))                 // Change of state read from the Commander GS
   {
        commGSImu = CMD_COMPLETE;                                               // dont send if we dont recognise the mode from COM GS
        udpReturn = UDP_NO_SEND;                                                // If we dont package a good request dont send it
        switch(commGSImu)                                                       // For the state of the switches from the Commander GS
        {
               case COMGS_IMU_MAIN:                                             // Selected Main IMU
                     head.commandID = SBGC_CMD_SELECT_IMU_3;                    // select imu message type
                     head.sizeData=sizeof(imusel);                              // Byte 3 of the header
                     head.cmdSize=(head.commandID + head.sizeData) % 256U;      // Byte 4 of the header
                     imusel.cmd_id = SBGC_IMU_TYPE_MAIN;                        // Select the main imu
                     //crc = checksum(ptr_imusel, sizeof(imusel));                // caculate the payload checksum
                     commGSImu = CMD_SEND;                                      // Send it
                     break;

        }

        if (commGSImu == CMD_SEND)                                              // Send the Message out if we had a valid request
        {
           memcpy(Buffer, &head, sizeof(head));                                  // Copy the header for imu selection
           memcpy(Buffer+sizeof(head), &imusel, head.sizeData);                  // Copy the payload
           if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
           {
              newCRC = crc16_arc_calculate(sizeof(imusel), ptr_imusel);         // Calculate new 16 bit CRC for new version
              memcpy(Buffer+sizeof(imusel)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(imusel)+sizeof(newCRC);         // Message length
           }
           else
           {
              crc = checksum(ptr_imusel, sizeof(imusel));                       // calculate the payload checksum
              memcpy(Buffer+sizeof(imusel)+sizeof(head),(void *) &crc,sizeof(crc)); // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(imusel)+sizeof(crc);            // Message length
           }
           udpReturn = UDP_SEND;
        }
        if (udpReturn == UDP_SEND)                                              // We packaged a good request for the Motor Command
        {
          // Package the command in the Buffer and send to serial and UDP
          DOTPORT2 = DOTPORT2 | GSCImu;                                         // Flash the LED to say sending adjustable variables command from GS
          memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                // Null terminate the Buffer for serial
          for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
          {
             UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
          }
          udpSent = 0u;
          while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {
            udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
            udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
          if (udpReturn != UDP_SEND)                                            // We did send it so wait for another Com GS Request
          {
             commGSImu = CMD_SENT;                                              // We sent it okay so wait for a confirm
          }
        }
    }

   // Now process any request to load a new profile or save the current set-up to a profile
   //
   if (commGSProSet == CMD_CONFIRMED)                                           // We did send it so wait for another Com GS Request
   {
      commGSProSetLast = commGSProSet;                                          // We sent it okay and got confirm so stop sending.
      commGSProSet= CMD_COMPLETE;
      DOTPORT2 = DOTPORT2 & !GSCProSet;                                         // Take the LED off when we confirmed it
   }
   else if (commGSProSet ==  CMD_SENT)                                          // We sent the messaage wait for the re-poll timer
   {
     if (countProSet <= MSG_REPOLL_TICKS)                                       // Counter is below the re-send delay off time (ticks)
     {
        countProSet=++countProSet % UINT64_MAX;                                             // Count for the time to delay the re-send if no confirm message
     }
     else
     {
        countProSet=0U;                                                         // Reset the message counter
        commGSProSet = CMD_RESEND;                                              // Re-issue the message
     }
   }
   if ((commGSProSetLast != commGSProSet) && (commGSProSet != CMD_SENT))        // Change of state read from the Commander GS
   {
        commGSProSet = CMD_COMPLETE;                                            // dont send if we dont recognise the mode from COM GS
        udpReturn = UDP_NO_SEND;                                                // If we dont package a good request dont send it
        switch(commGSProSet)                                                    // For the state of the switches from the Commander GS
        {
           case COMGS_PROFILE_SET_ACTION_SAVE:                                  // Save the params to the given slot
              head.commandID = SBGC_CMD_PROFILE_SET;                            // select profile set message type
              head.sizeData=sizeof(profileset);                                 // Byte 3 of the header
              head.cmdSize=(head.commandID + head.sizeData) % 256U;             // Byte 4 of the header
              profileset.action = SBGC_PROFILE_SET_ACTION_SAVE;
              break;
           case COMGS_PROFILE_SET_ACTION_CLEAR:                                 // Clear the params to the given slot
              head.commandID = SBGC_CMD_PROFILE_SET;                            // select profile set message type
              head.sizeData=sizeof(profileset);                                 // Byte 3 of the header
              head.cmdSize=(head.commandID + head.sizeData) % 256U;             // Byte 4 of the header
              profileset.action = SBGC_PROFILE_SET_ACTION_CLEAR;
              break;
           case COMGS_PROFILE_SET_ACTION_LOAD:                                  // Load the params from the given slot
              head.commandID = SBGC_CMD_PROFILE_SET;                            // select profile set message type
              head.sizeData=sizeof(profileset);                                 // Byte 3 of the header
              head.cmdSize=(head.commandID + head.sizeData) % 256U;             // Byte 4 of the header
              profileset.action = SBGC_PROFILE_SET_ACTION_CLEAR;
              break;
        }
        profileset.slot = commGSSlotNo;                                         // The Slot number sent from commGS
        crc = checksum(ptr_ps, sizeof(profileset));                             // calculate the payload checksum
        commGSProSet = CMD_SEND;                                                // Send it

        if (commGSProSet == CMD_SEND)                                           // Send the Message out if we had a valid request
        {
          memcpy(Buffer, &head, sizeof(head));                                  // Copy the header for profile set selection
          memcpy(Buffer+sizeof(head), &profileset, head.sizeData);              // Copy the payload
          if (boardinforb.FIRMWARE_VER >= 2680u)                                // If we have the new firmware loaded
          {
              newCRC = crc16_arc_calculate(sizeof(profileset), ptr_ps);         // Calculate new 16 bit CRC for new version
              memcpy(Buffer+sizeof(profileset)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(profileset)+sizeof(newCRC);      // message length
          }
          else
          {
             crc = checksum(ptr_ps, sizeof(profileset));                        // caculate the payload checksum
             memcpy(Buffer+sizeof(profileset)+sizeof(head),(void *) &crc,sizeof(crc));   // Tag the CRC
             totalMsgLen = sizeof(head)+sizeof(profileset)+sizeof(crc);          // message length
          }
          udpReturn = UDP_SEND;
        }
        if (udpReturn == UDP_SEND)                                              // We packaged a good request for the Motor Command
        {
          // Package the command in the Buffer and send to serial and UDP
          DOTPORT2 = DOTPORT2 | GSCProSet;                                      // Flash the LED to say sending adjustable variables command from GS
          memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                // Null terminate the Buffer for serial
          for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
          {
             UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
          }
          udpSent = 0u;
          while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {
            udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
            udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
          if (udpReturn != UDP_SEND)                                            // We did send it so wait for another Com GS Request
          {
             commGSProSet = CMD_SENT;                                           // We sent it okay so wait for a confirm
          }
        }
    }

    //
    // -------------------   Now process any request to configure the joystick controls CMD_CONTROL_CONFIG with confirmation ---------
    //
    //
   if (commGSConSet == CMD_CONFIRMED)                                           // We did send it so wait for another Com GS Request
   {
      commGSConSetLast = commGSConSet;                                          // We sent it okay and got confirm so stop sending.
      commGSConSet= CMD_COMPLETE;
      DOTPORT2 = DOTPORT2 & !GSCConSet;                                         // Take the LED off when we confirmed it
   }
   else if (commGSConSet ==  CMD_SENT)                                          // We sent the messaage wait for the re-poll timer
   {
        if (countConSet <= MSG_REPOLL_TICKS)                                    // Counter is below the re-send delay off time (ticks)
        {
           countConSet=++countConSet % UINT64_MAX;                                          // Count for the time to delay the re-send if no confirm message
        }
        else
        {
           countConSet=0U;                                                      // Reset the message counter
           commGSConSet = CMD_RESEND;                                           // Re-issue the message
        }
   }
   if ((commGSConSetLast != commGSConSet) && (commGSConSet != CMD_SENT))        // Change of state read from the Commander GS
   {
        commGSConSet = CMD_COMPLETE;                                            // dont send if we dont recognise the mode from COM GS
        udpReturn = UDP_NO_SEND;                                                // If we dont package a good request dont send it
        switch(commGSConSet)                                                       // For the state of the switches from the Commander GS
        {
               case COMGS_EXPO_RATE_MODE_RC:                                    // Set the expo rate for mode RC
                     head.commandID = SBGC_CMD_CONTROL_CONFIG;                  // select configure control message type
                     head.sizeData=sizeof(cc);                                  // Byte 3 of the header length
                     head.cmdSize=(head.commandID + head.sizeData) % 256u;      // Byte 4 of the header
                     cc.RC_EXPO_RATE = commGSConfConVal;                        // Set the expo rate
                     break;
                case COMGS_ANGLE_LPF:                                           // change angle lpf filter
                     head.commandID = SBGC_CMD_CONTROL_CONFIG;                  // select profile set message type
                     head.sizeData=sizeof(cc);                                  // Byte 3 of the header length
                     head.cmdSize=(head.commandID + head.sizeData) % 256u;      // Byte 4 of the header
                     cc.ANGLE_LPF_ROLL = commGSConfConVal;                      // Set the angle lpf for each axis
                     cc.ANGLE_LPF_PITCH = commGSConfConVal;                     // Set the angle lpf for each axis
                     cc.ANGLE_LPF_YAW = commGSConfConVal;                       // Set the angle lpf for each axis
                     break;
                case COMGS_SPEED_LPF:                                           // change speed lpf filter
                     head.commandID = SBGC_CMD_CONTROL_CONFIG;                  // select profile set message type
                     head.sizeData=sizeof(cc);                                  // Byte 3 of the header length
                     head.cmdSize=(head.commandID + head.sizeData) % 256u;      // Byte 4 of the header
                     cc.SPEED_LPF_ROLL = commGSConfConVal;                      // Set the speed lpf for each axis
                     cc.SPEED_LPF_PITCH = commGSConfConVal;                     // Set the speed lpf for each axis
                     cc.SPEED_LPF_YAW = commGSConfConVal;                       // Set the speed lpf for each axis
                     break;
                 case COMGS_RC_LPF:                                             // change rc of lpf filter
                     head.commandID = SBGC_CMD_CONTROL_CONFIG;                  // select profile set message type
                     head.sizeData=sizeof(cc);                                  // Byte 3 of the header length
                     head.cmdSize=(head.commandID + head.sizeData) % 256u;      // Byte 4 of the header
                     cc.RC_LPF_ROLL = commGSConfConVal;                         // Set the rc lpf for each axis
                     cc.RC_LPF_PITCH = commGSConfConVal;                        // Set the rc lpf for each axis
                     cc.RC_LPF_YAW = commGSConfConVal;                          // Set the rc lpf for each axis
                     break;
                 default:                                                       // If commGSConSet not defined we skip this step
                     eventMsgState == CMD_SENT;
                     break;
        }
        //crc = checksum(ptr_cc, sizeof(cc));                                     // calculate the payload checksum
        commGSConSet = CMD_SEND;                                                // Send it

        if (commGSConSet == CMD_SEND)                                           // Send the Message out if we had a valid request
        {
          memcpy(Buffer, &head, sizeof(head));                                  // Copy the header for profile set selection
          memcpy(Buffer+sizeof(head), &cc, head.sizeData);                      // Copy the payload
          if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
          {
              newCRC = crc16_arc_calculate(sizeof(cc), ptr_cc);                 // Calculate new 16 bit CRC for new version
              memcpy(Buffer+sizeof(cc)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(cc)+sizeof(newCRC);             // message length
          }
          else
          {
              crc = checksum(ptr_cc, sizeof(cc));                               // calculate the payload checksum
              memcpy((void *) Buffer[sizeof(cc)+sizeof(head)],(void *) &crc,sizeof(char));         // Tag the CRC
              totalMsgLen = sizeof(head) + sizeof(cc) + sizeof(char);           // The message length
          }
          udpReturn = UDP_SEND;
        }
        if (udpReturn == UDP_SEND)                                              // We packaged a good request for the Motor Command
        {
          // Package the command in the Buffer and send to serial and UDP
          DOTPORT2 = DOTPORT2 | GSCConSet;                                      // Flash the LED to say sending adjustable variables command from GS
          memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                // Null terminate the Buffer for serial
          for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
          {
            UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
          }
          udpSent = 0u;
          while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
          {
            udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
            udpSent=++udpSent % UINT8_MAX;
          }
          if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;        // accumulate the bad send counter (remove the 1 before summing)
          if (udpReturn != UDP_SEND)                                            // We did send it so wait for another Com GS Request
          {
             commGSConSet = CMD_SENT;                                           // We sent it okay so wait for a confirm
          }
        }
    }

    // Now configure the CMD_DATA_STREAM_INTERVAL for the CMD_EVENT alarms/state changes
    if (boardinforb.FIRMWARE_VER < 2600U)                                       // if data stream interval commands not supported
    {
       eventMsgState=CMD_CONFIRMED;                                             // skip this step as its not possible to do any with firmware this old
    }

    if (eventMsgState == CMD_SENT)
    {
       if ((boardinforb.FIRMWARE_VER < 2657U) && (commGSEvSet>= COMGS_CMD_EVENT))// if event command not supported
       {
          eventMsgState=CMD_CONFIRMED;                                          // skip this step as its not possible to do it the firmware doesnt support it
       }
       head.commandID = SBGC_CMD_DATA_STREAM_INTERVAL;                          // Issue request to set the event poll rate
       head.sizeData=sizeof(dataStreamInt);                                     // Byte 3 of the header
       head.cmdSize=(head.commandID + head.sizeData) % 256U;                    // Byte 4 of the header

       switch(commGSEvSet)                                                      // byte describing which event is required
       {
          case COMGS_CMD_REALTIME_DATA_3:
             dataStreamInt.cmd_id = SBGC_CMD_REALTIME_DATA_3;                   // event of real time data custon 3 update
             dataStreamInt.interval_ms = 200U;                                  // Set to 200 ms update period
             break;
          case COMGS_CMD_REALTIME_DATA_4:
             dataStreamInt.cmd_id = SBGC_CMD_REALTIME_DATA_4;                   // event of real time data custon 4 update
             dataStreamInt.interval_ms = 200U;                                  // Set to 200 ms update period
             break;
          case COMGS_CMD_REALTIME_DATA_CUSTOM:
             dataStreamInt.cmd_id = SBGC_CMD_REALTIME_DATA_CUSTOM;              // event of real time data custon helper update
             dataStreamInt.interval_ms = 200U;                                  // Set to 200 ms update period
             break;
          case COMGS_CMD_AHRS_HELPER:
             dataStreamInt.cmd_id = SBGC_CMD_AHRS_HELPER;                       // event of AHRS helper update
             dataStreamInt.interval_ms = 200U;                                  // Set to 200 ms update period
             dataStreamInt.config[0] = commGSImu;                               // chosen IMU
             break;
          case COMGS_CMD_EVENT:
             dataStreamInt.cmd_id = SBGC_CMD_EVENT;                             // Set the poll rate for EVENT type
             dataStreamInt.interval_ms = 200U;                                  // Set to 200 ms update period
             switch(commGSEvTyp)                                                // Event type from HMI command
             {
                 case COMGS_EVENT_ID_MENU_BUTTON_OFF:
                    dataStreamInt.config[0u] = SBGC_EVENT_ID_MENU_BUTTON;       // menu button press
                    dataStreamInt.config[1u] = SBGC_EVENT_TYPE_OFF;             // each time its off
                    break;
                 case COMGS_EVENT_ID_MENU_BUTTON_ON:
                    dataStreamInt.config[0u] = SBGC_EVENT_ID_MENU_BUTTON;       // menu button press
                    dataStreamInt.config[1u] = SBGC_EVENT_TYPE_ON;              // each time its on
                    break;
                 case COMGS_EVENT_ID_MENU_BUTTON_HOLD:
                    dataStreamInt.config[0u] = SBGC_EVENT_ID_MENU_BUTTON;       // menu button press
                    dataStreamInt.config[1u] = SBGC_EVENT_TYPE_HOLD;            // each time in hold
                    break;
                 case COMGS_EVENT_ID_MOTOR_STATE_OFF:                           // motor status
                    dataStreamInt.config[0u] = SBGC_EVENT_ID_MOTOR_STATE;
                    dataStreamInt.config[1u] = SBGC_EVENT_TYPE_OFF;             // changed to stopped
                    break;
                 case COMGS_EVENT_ID_MOTOR_STATE_ON:                            // motor status
                    dataStreamInt.config[0u] = SBGC_EVENT_ID_MOTOR_STATE;
                    dataStreamInt.config[1u] = SBGC_EVENT_TYPE_ON;              // changed to running
                    break;
                 case COMGS_EVENT_ID_EMERGENCY_STOP_OFF:                        // e-stop
                    dataStreamInt.config[0u] = SBGC_EVENT_ID_MOTOR_STATE;
                    dataStreamInt.config[1u] = SBGC_EVENT_TYPE_OFF;             // changd to off
                    break;
                 case COMGS_EVENT_ID_EMERGENCY_STOP_ON:                         // e-stop
                    dataStreamInt.config[0u] = SBGC_EVENT_ID_MOTOR_STATE;
                    dataStreamInt.config[1u] = SBGC_EVENT_TYPE_ON;              // changed to on
                    break;
                 case COMGS_EVENT_ID_CAMERA_REC_PHO:                            // record photo event
                    dataStreamInt.config[0u] = SBGC_EVENT_ID_CAMERA;
                    dataStreamInt.config[1u] = SBGC_EVENT_TYPE_REC_PHOTO;       // record pressed
                    break;
                 case COMGS_EVENT_ID_CAMERA_PHO:                                // photo event
                    dataStreamInt.config[0u] = SBGC_EVENT_ID_CAMERA;
                    dataStreamInt.config[1u] = SBGC_EVENT_TYPE_PHOTO;           // still picture
                    break;
                 case COMGS_EVENT_ID_SCRIPT_OFF:                                // script
                    dataStreamInt.config[0u] = SBGC_EVENT_ID_SCRIPT;
                    dataStreamInt.config[1u] = SBGC_EVENT_TYPE_OFF;             // changed to stop
                    dataStreamInt.config[2u] = commGSSlotNo;                    // slot for script
                    break;
                 case COMGS_EVENT_ID_SCRIPT_ON:                                 // script
                    dataStreamInt.config[0u] = SBGC_EVENT_ID_SCRIPT;
                    dataStreamInt.config[1u] = SBGC_EVENT_TYPE_OFF;             // start to execute
                    dataStreamInt.config[2u] = commGSSlotNo;                    // slot for script
                    break;
                 default:                                                       // Not a valid event choice from HMI
                    eventMsgState=CMD_CONFIRMED;                                // skip this step chosen
                    break;
             }
             break;
          default:                                                              // Not a valid event choice from HMI
             eventMsgState=CMD_CONFIRMED;                                       // skip this step chosen
             break; // skip
       }
       if (eventMsgState == CMD_SENT)                                           // You send if you got a valid request
       {
         memcpy(Buffer, &head, sizeof(head));                                   // Copy the header for imu selection
         memcpy(Buffer+SBGC_NUM_HEADER_BYTES, &dataStreamInt, head.sizeData);   // Copy the payload
         if (boardinforb.FIRMWARE_VER >= 2680U)                                 // If we have the new firmware loaded
         {
            newCRC = crc16_arc_calculate(sizeof(dataStreamInt), ptr_dsi);       // Calculate new 16 bit CRC for new version
            memcpy(Buffer+sizeof(dataStreamInt)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
            totalMsgLen = sizeof(head)+sizeof(dataStreamInt)+sizeof(newCRC);    // Message length
         }
         else
         {
            crc = checksum(ptr_dsi, sizeof(dataStreamInt));                     // calculate the payload checksum
            memcpy((void *) Buffer[sizeof(dataStreamInt)+SBGC_NUM_HEADER_BYTES],(void *) &crc,sizeof(crc)); // Tag the CRC
            totalMsgLen = SBGC_NUM_HEADER_BYTES + sizeof(dataStreamInt) + sizeof(crc);   // The message length
         }
         udpReturn = UDP_SEND;
         memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                 // Null terminate the Buffer for serial
         for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
         {
            UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
         }
         while ((udpReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
         {                                                                      // Send the Buffer contents as UDP datagram.
           udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);
           udpSent=++udpSent % UINT8_MAX;
         }
         if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;         // accumulate the bad send counter (remove the 1 before summing)
         if (udpReturn != UDP_SEND)
         {
           eventMsgState = CMD_SENT_2;                                          // We sent them both okay so wait for a confirm
         }
       }
    }
}
#endif                                                                          // ================== eNd simpleBGC from GUI ===========================

#if defined(SBGC_GIMBAL_JOY)
//======================= SBGC GIMBAL ==========================================
   //
   // ========================== JoyStick Controls =============================
   //
   // Now Process the joystick and pad pushbutton states
   //
   stateEngine=stateEngine & JoyClear;                                          /* Reset the Joystick LEDS */
#if (AIN_ADC_METHOD == ADC_AUTO_COLLECT)
   valueADC[1u]=ADC1_Get_Sample(X_AXIS_IN);                              /* Get Raw ADC input 1 in units 0-1023   */
   valueADC[2u]=ADC1_Get_Sample(Y_AXIS_IN);                              /* Get Raw ADC input 2 in units 0-1023   */
   valueADC[3u]=ADC1_Get_Sample(Z_AXIS_IN);                              /* Get Raw ADC input 3 in units 0-1023   */
#elif (AIN_ADC_METHOD == ADC_MAN_COLLECT)
   if (readManualADC(nextAin,&valueADC[nextAin-START_MAN_AIN_PIN+1u])==ADC_COLLECT_COMPLETE)
   {
      g_adcState=CHOOSE_ADC_PIN;                                                /* set the ADC state to change the pin number */
      nextAin=(++nextAin % NUM_OF_AINS)+START_MAN_AIN_PIN;                      /* count in bounds 1 to num_of_ains */
   }
#endif

#ifdef USE_CALIB_VALS
   if (valueADC[1u] >= calibrationObj.centerValx) valueADC[1u] = (int16_t) Scale_Raw_Value(calibrationObj.centerValx ,calibrationObj.minValx,RAW_MAX>>1,RAW_MAX,valueADC[1u]);
   if (valueADC[1u] < calibrationObj.centerValx) valueADC[1u] = (int16_t) Scale_Raw_Value(calibrationObj.minValx ,calibrationObj.centerValx,0,RAW_MAX>>1,valueADC[1u]);
   if (valueADC[2u] >= calibrationObj.centerValx) valueADC[2u] = (int16_t) Scale_Raw_Value(calibrationObj.centerValx ,calibrationObj.minValx,RAW_MAX>>1,RAW_MAX,valueADC[2u]);
   if (valueADC[2u] < calibrationObj.centerValx) valueADC[2u] = (int16_t) Scale_Raw_Value(calibrationObj.minValx ,calibrationObj.centerValx,0,RAW_MAX>>1,valueADC[2u]);
   if (valueADC[3u] >= calibrationObj.centerValx) valueADC[3u] = (int16_t) Scale_Raw_Value(calibrationObj.centerValx ,calibrationObj.minValx,RAW_MAX>>1,RAW_MAX,valueADC[3u]);
   if (valueADC[3u] < calibrationObj.centerValx) valueADC[3u] = (int16_t) Scale_Raw_Value(calibrationObj.minValx ,calibrationObj.centerValx,0,RAW_MAX>>1,valueADC[3u]);
#endif

// If we need to process the joystick as non-linear do this here (various funcs in ComGs.h
#if defined(PROCESS_SMOOTH_INPUT_BEAR)                                                
   JoyStick2BearSteer((int16_t) Scale_Raw_Value(0,RAW_MAX,-128,127,valueADC[1u]),(int16_t) Scale_Raw_Value(0,RAW_MAX,-128,127,valueADC[2u]),XrawComp, YrawComp);
   valueADC[1u] = (int16_t) Scale_Raw_Value(-128,127,0,RAW_MAX,XrawComp);
   valueADC[2u] = (int16_t) Scale_Raw_Value(-128,127,0,RAW_MAX,YrawComp);
#elif defined(PROCESS_SMOOTH_INPUT_DIFF)
   JoyStick2DiffSteer((int16_t) Scale_Raw_Value(0,RAW_MAX,-128,127,valueADC[1u]),(int16_t) Scale_Raw_Value(0,RAW_MAX,-128,127,valueADC[2u]),XrawComp, YrawComp);
   valueADC[1u] = (int16_t) Scale_Raw_Value(-128,127,0,RAW_MAX,XrawComp);
   valueADC[2u] = (int16_t) Scale_Raw_Value(-128,127,0,RAW_MAX,YrawComp);
#endif

   //
   // ---------- Now choose the function for the joystick from the pushbutton / switch digital input Status and firmware revision ------------
   //
   if ((boardinforb.FIRMWARE_VER < 2555U) && (g_boardReady))                      // Control for board less than v2.555
   {
      if(debounce_button(&debounceResetPB, (di3Reset == true) ? BTN_STATE_PRESSED : BTN_STATE_RELEASED, BTN_BOUNCE_THRESHOLD_MS))   // Change of state for more than timer ms
      {
        if (debounceResetPB.trigger_state == BTN_STATE_PRESSED)                 // Actual State is set to Presses
        {
           head.commandID = SBGC_CMD_RESET;                                     // Issue a Reset to the gimbal
           head.sizeData=sizeof(r);                                             // Byte 3 of the header
           head.cmdSize=(head.commandID + head.sizeData) % 256U;                // Byte 4 of the header

           r.confirmRequired = SBGC_NO_CONFIRM;                                 // No Confirm Required
           r.delayMS = 5U;                                                       // delay is 5 ms

           memcpy(Buffer, &head, sizeof(head));                                 // Copy the header
           memcpy(Buffer+sizeof(head), &r, sizeof(r));                          // Copy the payload
           crc = checksum(ptr_r, sz_r);                                         // Calculate the payload checksum
           memcpy(Buffer+sizeof(r)+sizeof(head),(void *) &crc,sizeof(crc));     // Copy the CRC
           totalMsgLen = sizeof(head) + sizeof(r) + sizeof(crc);                // Message length
           stateEngine=stateEngine | JoyReset;                                  // indicate on LED reset state
           msgToSend=true;                                                      // Send what you composed in the Buffer here
        }
      }
      else if ((di2PosHalt==0u) && (di4RCMode==1u))                             // (RC Mode Control)  - to switch here we must 1st do a MODE_NO_CONTROL
      {
         // Initialise the header for CONTROL_MODE
        head.commandID=SBGC_CMD_CONTROL;                                        // Command ID is control mode.
        head.sizeData=sizeof(c);                                                // Byte 3 of the header
        head.cmdSize=(head.commandID + head.sizeData) % 256u;                   // Byte 4 of the header

        if (di6FastSelect)                                                      // Fast mode selected
        {
           pitchAngleReq=Scale_Raw_Value(0,RAW_MAX,RC_MIN_FAST,RC_MAX_FAST,valueADC[1u]);       // Calculate the Speeds from the ADC range +- 16384
           yawAngleReq=Scale_Raw_Value(0,RAW_MAX,RC_MIN_FAST,RC_MAX_FAST,valueADC[2u]);
           rollAngleReq=Scale_Raw_Value(0,RAW_MAX,RC_MIN_FAST,RC_MAX_FAST,valueADC[3u]);
        }
        else
        {
           pitchAngleReq=Scale_Raw_Value(0,RAW_MAX,RC_MIN,RC_MAX,valueADC[1u]);    // Calculate the Speeds from the ADC range +- 500
           yawAngleReq=Scale_Raw_Value(0,RAW_MAX,RC_MIN,RC_MAX,valueADC[2u]);
           rollAngleReq=Scale_Raw_Value(0,RAW_MAX,RC_MIN,RC_MAX,valueADC[3u]);
        }

        if (!modeAlreadyRC)                                                     // We havent locked into this mode yet (this is set when Confirm comes back)
        {
           if (modeInConfirm)                                                   // Control config message to set confirm has completed
           {
              c.mode = SBGC_CONTROL_MODE_NO;                                    // We must go through a NO_CONTROL reset before we engage mode
              c.speedROLL = 0u;                                                 // ReSet the roll speed
              c.speedPITCH = 0u;                                                // ReSet the pitch speed
              c.speedYAW = 0u;                                                  // ReSet the yaw speed
              c.angleROLL = 0u;                                                 // ReSet the roll angle
              c.anglePITCH = 0u;                                                // ReSet the pitch angle
              c.angleYAW = 0u;                                                  // ReSet the yaw angle
           }
           else                                                                 // Set the mode to send CONFIRM in CMD_CONTROL to verify we have CONTROL_MODE_NO before proceedeing with RC Mode
           {
              if (ccReturn == CMD_SENT)                                         // already sent a control_configure request but no reply as yet
              {
                 if (countControlCnf <= MSG_REPOLL_TICKS)                       // Counter is below the re-send delay off time (ticks)
                 {
                    countControlCnf=++countControlCnf % UINT64_MAX;                             // Count for the time to delay the re-send if no confirm message
                 }
                 else
                 {
                    countControlCnf=0u;                                         // Reset the message counter
                    ccReturn = CMD_RESEND;                                      // Re-issue the message
                 }
              }
              else
              {
                 ccReturn=ChgStat_CONFIG_for_CONTROL(&cc,true,sizeof(cc));      // Set the confirm bits for the CMD_CONTROL_CONFIG request to give CONFIRM for CMD_CONTROL
              }
           }
        }
        else
        {
           if (ccReturn == CMD_SENT)                                            // already sent a control_configure request but no reply as yet
           {
              if (countControlCnf <= MSG_REPOLL_TICKS)                          // Counter is below the re-send delay off time (ticks)
              {
                 countControlCnf=++countControlCnf % UINT64_MAX;                                // Count for the time to delay the re-send if no confirm message
              }
              else
              {
                 countControlCnf=0u;                                            // Reset the message counter
                 ccReturn = CMD_RESEND;                                         // Re-issue the message
              }
           }
           else
           {
              ccReturn=ChgStat_CONFIG_for_CONTROL(&cc,false,sizeof(cc));        // We can now stop the confirm for the CMD_CONTROL request
           }
           c.mode = SBGC_CONTROL_MODE_RC;                                       // Set Control Mode to RC Speed
           c.speedROLL = 0U;                                                    // Set the roll speed
           c.speedPITCH = 0U;                                                   // Set the pitch speed
           c.speedYAW = 0U;                                                     // Set the yaw speed
           c.angleROLL = SBGC_DEGREE_TO_ANGLE_INT(rollAngleReq);                // Set the roll angle
           c.anglePITCH = SBGC_DEGREE_TO_ANGLE_INT(pitchAngleReq);              // Set the pitch angle
           c.angleYAW =  SBGC_DEGREE_TO_ANGLE_INT(yawAngleReq);                 // Set the yaw angle
        }

        if (!((!modeInConfirm) && (!modeAlreadyRC)))                            // Wait for succesful CMD_CONRTOL_CONFIG to turn on confirm for NO_CONTROL
        {
           crc = checksum(ptr_c, sz_c);                                         // calculate the payload checksum

           memcpy(Buffer, &head, sizeof(head));                                 // Copy the header
           memcpy(Buffer+sizeof(head), &c, sizeof(c));                          // Copy the payload
           memcpy(Buffer+sizeof(c)+sizeof(head),(void *) &crc,sizeof(crc));     // Tag the CRC byte on the end.

           totalMsgLen = sizeof(head) + sizeof(c) + 1u;                         // message length
           stateEngine=stateEngine | JoySpeed;                                  // State Engine = RC Mode (set the same as speed on led).
           msgToSend=true;                                                      // Send what you composed in the Buffer here
        }
        else
        {
           msgToSend=false;                                                     // Dont send the Buffer until the CMD_CONTROL_CONFIG message has been confirmed
        }

     }
     else if ((di2PosHalt==0u) && (di5VCMode==1u))                              // (Virtual Channel Control)
     {
        // Initialise the header for VIRTUAL CHANNEL MODE
        head.commandID=SBGC_CMD_API_VIRT_CH_CONTROL;                            // Command ID is virtual channel mode.
        head.sizeData=sizeof(vc);                                               // Byte 3 of the header
        head.cmdSize=(head.commandID + head.sizeData) % 256U;                   // Byte 4 of the header

        // Calculate the vc from the ADC
        vc.VCdata[0u]=(int16_t) Scale_Raw_Value(0,RAW_MAX,RC_MIN,RC_MAX,valueADC[1u]);        // range -500 to 500
        vc.VCdata[1u]=(int16_t) Scale_Raw_Value(0,RAW_MAX,RC_MIN,RC_MAX,valueADC[2u]);
        vc.VCdata[2u]=(int16_t) Scale_Raw_Value(0,RAW_MAX,RC_MIN,RC_MAX,valueADC[3u]);
        crc = checksum(ptr_vc, sz_vc);                                          // caculate the payload checksum
        memcpy(Buffer, &head, sizeof(head));                                    // Copy the header
        memcpy(Buffer+sizeof(head), &vc, sizeof(vc));                           // Copy the payload
        memcpy(Buffer+sizeof(vc)+sizeof(head),(void *) &crc,sizeof(char));       // Tag the CRC byte on the end.
        totalMsgLen = sizeof(head) + sizeof(vc) + 1u;                           // message length
        stateEngine=stateEngine | JoySpeed;                                     // State Engine = virtual channel (same ind. as speed)
        modeAlreadyRC = false;                                                  // Force a NO_CONTROL 1st before going into RC_MODE
        modeInConfirm = false;                                                  // remove confirm flag before we go to RC_MODE
        msgToSend=true;                                                         // Send what you composed in the Buffer here
     }
     else if ((di2PosHalt==0u) && (di1SpeedAngle==0u))                          // Input 2 and Input1 abd Input 3 all false   (Angle Mode)
     {
        // Initialise the header for CONTROL_MODE
        head.commandID=SBGC_CMD_CONTROL;                                        // Command ID is control mode.
        head.sizeData=sizeof(c);                                                // Byte 3 of the header
        head.cmdSize=(head.commandID + head.sizeData) % 256U;                   // Byte 4 of the header

        // fix the speed at 30 deg/sec
        c.speedROLL = c.speedPITCH = c.speedYAW = 30U * SBGC_SPEED_SCALE;

        // Calculate the current angle data
        pitchAngleReq=PITCH_ANGLE_MIN + ((valueADC[1u]*(PITCH_ANGLE_MAX - PITCH_ANGLE_MIN))/RAW_MAX);
        yawAngleReq=YAW_ANGLE_MIN + ((valueADC[2u]*(YAW_ANGLE_MAX - YAW_ANGLE_MIN))/RAW_MAX);
        rollAngleReq=ROLL_ANGLE_MIN + ((valueADC[3u]*(ROLL_ANGLE_MAX - ROLL_ANGLE_MIN))/RAW_MAX);

        // Send the payload for angle control
        c.mode=(SBGC_CONTROL_MODE_ANGLE | (di6FastSelect * SBGC_CONTROL_FLAG_AUTO_TASK));  // Set Control Mode to Angle (The Fast is controled by a switch)
        c.anglePITCH=SBGC_DEGREE_TO_ANGLE_INT(pitchAngleReq);                   // Set pitch angle to ADC1
        c.angleYAW=SBGC_DEGREE_TO_ANGLE_INT(yawAngleReq);                       // Set yaw angle to ADC2
        c.angleROLL=SBGC_DEGREE_TO_ANGLE_INT(rollAngleReq);                     // Set roll angle to ADC3

        crc = checksum(ptr_c, sz_c);                                            // caculate the payload checksum

        memcpy(Buffer, &head, sizeof(head));                                    // Copy the header
        memcpy(Buffer+sizeof(head), &c, sizeof(c));                             // Copy the payload
        memcpy((void *) Buffer+sizeof(c)+sizeof(head),(void *) &crc,sizeof(crc));         // Tag the CRC byte on the end

        totalMsgLen = sizeof(head) + sizeof(c) + sizeof(crc);                   // message length
        stateEngine=stateEngine | JoyAngle;                                     // State Engine = Control Angle Mode
        modeAlreadyRC = false;                                                  // Force a NO_CONTROL 1st before going into RC_MODE
        modeInConfirm = false;                                                  // remove confirm flag before we go to RC_MODE
        msgToSend=true;                                                         // Send what you composed in the Buffer here
     }
     else if ((di2PosHalt==0u) && (di1SpeedAngle==1u))                          // Input 2 and Input1 abd Input 3 all false    (Speed Mode)
     {
        // Initialise the header for CONTROL_MODE
        head.commandID=SBGC_CMD_CONTROL;                                        // Command ID is control mode
        head.sizeData=sizeof(c);                                                // Byte 3 of the header
        head.cmdSize=(head.commandID + head.sizeData) % 256u;                   // Byte 4 of the header

        //   scale the joystick to max gimbal speed in deg/sec
        pitchSpeedReq=Scale_Raw_Value(0u,RAW_MAX,0,SBGC_MAX_GIMBAL_SPEED,valueADC[1u]);   // Calculate the Speeds from the ADC this is 0-200 deg/sec
        yawSpeedReq=Scale_Raw_Value(0u,RAW_MAX,0,SBGC_MAX_GIMBAL_SPEED,valueADC[2u]);
        rollSpeedReq=Scale_Raw_Value(0u,RAW_MAX,0,SBGC_MAX_GIMBAL_SPEED,valueADC[3u]);

        c.mode = (SBGC_CONTROL_MODE_SPEED | (!di6FastSelect * SBGC_CONTROL_FLAG_HIGH_RES_SPEED));   // Set Control Mode to Speed (The Fast / Slow motion is same input switch used in Angle Mode)
        if (di6FastSelect)
        {
           c.speedROLL = (uint16_t) (rollSpeedReq * SBGC_SLOW_SPEED_SCALE);     // Set the roll speed using 0.001 deg/sec
           c.speedPITCH = (uint16_t) (pitchSpeedReq * SBGC_SLOW_SPEED_SCALE);   // Set the pitch speed
           c.speedYAW = (uint16_t) (yawSpeedReq * SBGC_SLOW_SPEED_SCALE);       // Set the yaw speed
        }
        else
        {
           c.speedROLL = (uint16_t) (rollSpeedReq * SBGC_SPEED_SCALE);          // Set the roll speed using 0.122 deg/sec
           c.speedPITCH = (uint16_t) (pitchSpeedReq * SBGC_SPEED_SCALE);        // Set the pitch speed
           c.speedYAW = (uint16_t) (yawSpeedReq * SBGC_SPEED_SCALE);            // Set the yaw speed
        }

        crc = checksum(ptr_c, sz_c);                                            // caculate the payload checksum

        memcpy(Buffer, &head, sizeof(head));                                    // Copy the header
        memcpy(Buffer+sizeof(head), &c, sizeof(c));                             // Copy the payload
        memcpy((void *) Buffer+sizeof(c)+sizeof(head),(void *) &crc,sizeof(char));  // Tag the CRC byte on the end.

        totalMsgLen = sizeof(head) + sizeof(c) + 1u;                            // message length
        stateEngine=stateEngine | JoySpeed;                                     // State Engine = Control Angle Speed
        modeAlreadyRC = false;                                                  // Force a NO_CONTROL 1st before going into RC_MODE
        modeInConfirm = false;                                                  // remove confirm flag before we go to RC_MODE
        msgToSend=true;                                                         // Send what you composed in the Buffer here
     }
     else if ((di5VCMode==1u) && (di2PosHalt==1u) && (di4RCMode==1u))           // Calibration Mode  ( di2 , di4 and di5 )
     {
         JOYCalAlarms.AlarmWdCurrent = CalibrateCenter( &calibrationObj );      // take and average of the center co-ordinate (use for correction if needed)
#ifdef USE_CAL_UNLOCK                                                           // if we are unlock type then initialise the last settings to find new ones
          calibrationObj.maxValx = 0u;                                          // call for new max values
          calibrationObj.maxValy = 0u;                                          // call for new max values
          calibrationObj.maxValz = 0u;                                          // call for new max values
          calibrationObj.minValx = (RAW_MAX>>1);                                // call for new min values
          calibrationObj.minValy = (RAW_MAX>>1);                                // call for new min values
          calibrationObj.minValz = (RAW_MAX>>1);                                // call for new min values
          calibrationObj.counter1 = 0u;                                         // call for new min values
#endif
         JOYCalAlarms.AlarmWdCurrent |= CalibrateSpan( &calibrationObj );
     }
     else                                                                       // Input 2 is on we have no control joystick off
     {
        stateEngine=stateEngine | JoyOff;                                       // State Engine = Joystick Off Mode
        msgToSend=false;                                                        // Dont send a command from the joystick
     }
  }
  else if ((boardinforb.FIRMWARE_VER >= 2555U) && (g_boardReady))                 // Board is version > 2.55b5
  {
      if(debounce_button(&debounceResetPB, (di3Reset == true) ? BTN_STATE_PRESSED : BTN_STATE_RELEASED, BTN_BOUNCE_THRESHOLD_MS))   // Change of state for more than timer ms
      {
        if (debounceResetPB.trigger_state == BTN_STATE_PRESSED)                 // Actual State is set to Presses
        {
           head.commandID = SBGC_CMD_RESET;                                     // Issue a Reset to the gimbal
           head.sizeData=sizeof(r);                                             // Byte 3 of the header
           head.cmdSize=(head.commandID + head.sizeData) % 256U;                // Byte 4 of the header

           r.confirmRequired = SBGC_NO_CONFIRM;                                 // No Confirm Required
           r.delayMS = 5U;                                                      // delay is 5 ms

           memcpy(Buffer, &head, sizeof(head));                                 // Copy the header
           memcpy(Buffer+sizeof(head), &r, sizeof(r));                          // Copy the payload
           if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
           {
              newCRC = crc16_arc_calculate(sizeof(r), ptr_r);                   // Calculate new 16 bit CRC for new version
              memcpy(Buffer+sizeof(r)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(r)+sizeof(newCRC);              // Message length
           }
           else
           {
              crc = checksum(ptr_r, sz_r);                                      // Calculate the payload checksum
              memcpy(Buffer+sizeof(r)+sizeof(head),(void *) &crc,sizeof(crc));   // Copy the CRC
              totalMsgLen = sizeof(head) + sizeof(r) + sizeof(crc);             // Message length
           }
           stateEngine=stateEngine | JoyReset;                                  // indicate on LED reset state
           msgToSend=true;                                                      // Send what you composed in the Buffer here
        }
      }
      else if ((di2PosHalt==0u) && (di4RCMode==1u))                             // (RC Mode Control)  - to switch here we must 1st do a MODE_NO_CONTROL
      {
         // Initialise the header for CONTROL_MODE
        head.commandID=SBGC_CMD_CONTROL;                                        // Command ID is control mode.
        head.sizeData=sizeof(c);                                                // Byte 3 of the header
        head.cmdSize=(head.commandID + head.sizeData) % 256u;                   // Byte 4 of the header

        if (di6FastSelect)                                                      // Fast mode selected
        {
           pitchAngleReq=Scale_Raw_Value(0u,RAW_MAX,RC_MIN_FAST,RC_MAX_FAST,valueADC[1u]);       // Calculate the Speeds from the ADC range +- 16384
           yawAngleReq=Scale_Raw_Value(0u,RAW_MAX,RC_MIN_FAST,RC_MAX_FAST,valueADC[2u]);
           rollAngleReq=Scale_Raw_Value(0u,RAW_MAX,RC_MIN_FAST,RC_MAX_FAST,valueADC[3u]);
        }
        else
        {
           pitchAngleReq=Scale_Raw_Value(0u,RAW_MAX,RC_MIN,RC_MAX,valueADC[1u]);    // Calculate the Speeds from the ADC range +- 500
           yawAngleReq=Scale_Raw_Value(0u,RAW_MAX,RC_MIN,RC_MAX,valueADC[2u]);
           rollAngleReq=Scale_Raw_Value(0u,RAW_MAX,RC_MIN,RC_MAX,valueADC[3u]);
        }

        if (!modeAlreadyRC)                                                     // We havent locked into this mode yet (this is set when Confirm comes back)
        {
           if (modeInConfirm)                                                   // Control config message to set confirm has completed
           {
              cnew.modeROLL = SBGC_CONTROL_MODE_NO;                             // We must go through a NO_CONTROL reset before we engage mode
              cnew.modePITCH = SBGC_CONTROL_MODE_NO;                            // We must go through a NO_CONTROL reset before we engage mode
              cnew.modeYAW = SBGC_CONTROL_MODE_NO;                              // We must go through a NO_CONTROL reset before we engage mode
              cnew.speedROLL = 0U;                                              // ReSet the roll speed
              cnew.speedPITCH = 0U;                                             // ReSet the pitch speed
              cnew.speedYAW = 0U;                                               // ReSet the yaw speed
              cnew.angleROLL = 0U;                                              // ReSet the roll angle
              cnew.anglePITCH = 0U;                                             // ReSet the pitch angle
              cnew.angleYAW = 0U;                                               // ReSet the yaw angle
           }
           else                                                                 // Set the mode to send CONFIRM in CMD_CONTROL to verify we have CONTROL_MODE_NO before proceedeing with RC Mode
           {
              if (ccReturn == CMD_SENT)                                         // already sent a control_configure request but no reply as yet
              {
                 if (countControlCnf <= MSG_REPOLL_TICKS)                       // Counter is below the re-send delay off time (ticks)
                 {
                    countControlCnf=++countControlCnf % UINT64_MAX;                             // Count for the time to delay the re-send if no confirm message
                 }
                 else
                 {
                    countControlCnf=0;                                          // Reset the message counter
                    ccReturn = CMD_RESEND;                                      // Re-issue the message
                 }
              }
              else
              {
                 ccReturn=ChgStat_CONFIG_for_CONTROL(&cc,true,sizeof(cc));      // Set the confirm bits for the CMD_CONTROL_CONFIG request to give CONFIRM for CMD_CONTROL
              }
           }
        }
        else
        {
           if (ccReturn == CMD_SENT)                                            // already sent a control_configure request but no reply as yet
           {
              if (countControlCnf <= MSG_REPOLL_TICKS)                          // Counter is below the re-send delay off time (ticks)
              {
                 countControlCnf=++countControlCnf % UINT64_MAX;                                // Count for the time to delay the re-send if no confirm message
              }
              else
              {
                 countControlCnf=0u;                                            // Reset the message counter
                 ccReturn = CMD_RESEND;                                         // Re-issue the message
              }
           }
           else
           {
              ccReturn=ChgStat_CONFIG_for_CONTROL(&cc,false,sizeof(cc));        // We can now stop the confirm for the CMD_CONTROL request
           }
           cnew.modeROLL = SBGC_CONTROL_MODE_RC;                                // Set Control Mode to RC Speed
           cnew.modePITCH = SBGC_CONTROL_MODE_RC;                               // Set Control Mode to RC Speed
           cnew.modeYAW = SBGC_CONTROL_MODE_RC;                                 // Set Control Mode to RC Speed
           cnew.angleROLL =  SBGC_DEGREE_TO_ANGLE_INT(rollAngleReq);            // Set the roll angle
           cnew.anglePITCH = SBGC_DEGREE_TO_ANGLE_INT(pitchAngleReq);           // Set the pitch angle
           cnew.angleYAW =  SBGC_DEGREE_TO_ANGLE_INT(yawAngleReq);              // Set the yaw angle
           cnew.speedROLL = 0U;                                                 // Set the roll speed
           cnew.speedPITCH = 0U;                                                // Set the pitch speed
           cnew.speedYAW = 0U;                                                  // Set the yaw speed
        }

        if (!((!modeInConfirm) && (!modeAlreadyRC)))                            // Wait for succesful CMD_CONRTOL_CONFIG to turn on confirm for NO_CONTROL
        {
           memcpy(Buffer, &head, sizeof(head));                                 // Copy the header
           memcpy(Buffer+sizeof(head), &cnew, sizeof(cnew));                    // Copy the payload
           if (boardinforb.FIRMWARE_VER >= 2680U)                                // If we have the new firmware loaded
           {
              newCRC = crc16_arc_calculate(sz_cnew, ptr_cnew);                  // Calculate new 16 bit CRC for new version
              memcpy(Buffer+sizeof(cnew)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
              totalMsgLen = sizeof(head)+sizeof(cnew)+sizeof(newCRC);           // Message length
           }
           else
           {
              crc = checksum(ptr_cnew, sz_cnew);                                // Calculate the payload checksum
              memcpy(Buffer+sizeof(cnew)+sizeof(head),(void *) &crc,sizeof(crc));   // Copy the CRC
              totalMsgLen = sizeof(head) + sizeof(cnew) + sizeof(crc);          // Message length
           }
           stateEngine=stateEngine | JoySpeed;                                  // State Engine = RC Mode (set the same as speed on led).
           msgToSend=true;                                                      // Send what you composed in the Buffer here
        }
        else
        {
           msgToSend=false;                                                     // Dont send the Buffer until the CMD_CONTROL_CONFIG message has been confirmed
        }

     }
     else if ((di2PosHalt==0u) && (di5VCMode==1u))                              // (Virtual Channel Control)
     {
        // Initialise the header for VIRTUAL CHANNEL MODE
        if (((boardinforb.FIRMWARE_VER >= 2687U) && (g_boardReady)) && di6FastSelect)
        {
           head.commandID=SBGC_CMD_API_VIRT_CH_HIGH_RES;                        // Command ID is virtual channel mode high resolution

        }
        else
        {
           head.commandID=SBGC_CMD_API_VIRT_CH_CONTROL;                         // Command ID is virtual channel mode.
        }
        head.sizeData=sizeof(vc);                                               // Byte 3 of the header
        head.cmdSize=(head.commandID + head.sizeData) % 256u;                   // Byte 4 of the header
        // Calculate the vc from the ADC
        if (((boardinforb.FIRMWARE_VER >= 2687U) && (g_boardReady)) && di6FastSelect)
        {
           vc.VCdata[0u]=(int16_t) Scale_Raw_Value(0u,RAW_MAX,RC_MIN_FAST,RC_MAX_FAST,valueADC[1u]);        // range -16384 to 16384
           vc.VCdata[1u]=(int16_t) Scale_Raw_Value(0u,RAW_MAX,RC_MIN_FAST,RC_MAX_FAST,valueADC[2u]);
           vc.VCdata[2u]=(int16_t) Scale_Raw_Value(0u,RAW_MAX,RC_MIN_FAST,RC_MAX_FAST,valueADC[3u]);
        }
        else
        {
           vc.VCdata[0u]=(int16_t) Scale_Raw_Value(0u,RAW_MAX,RC_MIN,RC_MAX,valueADC[1u]);        // range -500 to 500
           vc.VCdata[1u]=(int16_t) Scale_Raw_Value(0u,RAW_MAX,RC_MIN,RC_MAX,valueADC[2u]);
           vc.VCdata[2u]=(int16_t) Scale_Raw_Value(0u,RAW_MAX,RC_MIN,RC_MAX,valueADC[3u]);
        }

        memcpy(Buffer, &head, sizeof(head));                                    // Copy the header
        memcpy(Buffer+sizeof(head), &vc, sizeof(vc));                           // Copy the payload
        if (boardinforb.FIRMWARE_VER >= 2680U)                                  // If we have the new firmware loaded
        {
            newCRC = crc16_arc_calculate(sizeof(vc), ptr_vc);                   // Calculate new 16 bit CRC for new version
            memcpy(Buffer+sizeof(vc)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
            totalMsgLen = sizeof(head)+sizeof(vc)+sizeof(newCRC);               // Message length
        }
        else
        {
            crc = checksum(ptr_vc, sz_vc);                                      // Calculate the payload checksum
            memcpy(Buffer+sizeof(vc)+sizeof(head),(void *) &crc,sizeof(crc));    // Copy the CRC
            totalMsgLen = sizeof(head) + sizeof(vc) + sizeof(crc);              // Message length
        }
        stateEngine=stateEngine | JoySpeed;                                     // State Engine = virtual channel (same ind. as speed)
        modeAlreadyRC = false;                                                  // Force a NO_CONTROL 1st before going into RC_MODE
        modeInConfirm = false;                                                  // remove confirm flag before we go to RC_MODE
        msgToSend=true;                                                         // Send what you composed in the Buffer here
     }
     else if ((di2PosHalt==0u) && (di1SpeedAngle==0u))                          // Input 2 and Input1 abd Input 3 all false   (Angle Mode)
     {
        // Initialise the header for CONTROL_MODE
        head.commandID=SBGC_CMD_CONTROL;                                        // Command ID is control mode.
        head.sizeData=sizeof(c);                                                // Byte 3 of the header
        head.cmdSize=(head.commandID + head.sizeData) % 256u;                   // Byte 4 of the header

        // Calculate the current angle data
        pitchAngleReq=PITCH_ANGLE_MIN + ((valueADC[1u]*(PITCH_ANGLE_MAX - PITCH_ANGLE_MIN))/RAW_MAX);
        yawAngleReq=YAW_ANGLE_MIN + ((valueADC[2u]*(YAW_ANGLE_MAX - YAW_ANGLE_MIN))/RAW_MAX);
        rollAngleReq=ROLL_ANGLE_MIN + ((valueADC[3u]*(ROLL_ANGLE_MAX - ROLL_ANGLE_MIN))/RAW_MAX);

        // Send the payload for angle control
        cnew.modeROLL=(SBGC_CONTROL_MODE_ANGLE | (di6FastSelect * SBGC_CONTROL_FLAG_AUTO_TASK));  // Set Control Mode to Angle (The Fast is controled by a switch)
        cnew.modePITCH=(SBGC_CONTROL_MODE_ANGLE | (di6FastSelect * SBGC_CONTROL_FLAG_AUTO_TASK));  // Set Control Mode to Angle (The Fast is controled by a switch)
        cnew.modeYAW=(SBGC_CONTROL_MODE_ANGLE | (di6FastSelect * SBGC_CONTROL_FLAG_AUTO_TASK));  // Set Control Mode to Angle (The Fast is controled by a switch)
        cnew.anglePITCH=SBGC_DEGREE_TO_ANGLE_INT(pitchAngleReq);                // Set pitch angle to ADC1
        cnew.angleYAW=SBGC_DEGREE_TO_ANGLE_INT(yawAngleReq);                    // Set yaw angle to ADC2
        cnew.angleROLL=SBGC_DEGREE_TO_ANGLE_INT(rollAngleReq);                  // Set roll angle to ADC3

        memcpy(Buffer, &head, sizeof(head));                                    // Copy the header
        memcpy(Buffer+sizeof(head), &cnew, sizeof(cnew));                             // Copy the payload
        if (boardinforb.FIRMWARE_VER >= 2680U)                                  // If we have the new firmware loaded
        {
            newCRC = crc16_arc_calculate(sizeof(cnew), ptr_cnew);               // Calculate new 16 bit CRC for new version
            memcpy(Buffer+sizeof(cnew)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
            totalMsgLen = sizeof(head)+sizeof(cnew)+sizeof(newCRC);             // Message length
        }
        else
        {
            crc = checksum(ptr_cnew, sz_cnew);                                  // calculate the payload checksum
            memcpy(Buffer+sizeof(cnew)+sizeof(head),(void *) &crc,sizeof(crc));  // Copy the CRC
            totalMsgLen = sizeof(head) + sizeof(cnew) + sizeof(crc);            // Message length
        }
        stateEngine=stateEngine | JoyAngle;                                     // State Engine = Control Angle Mode
        modeAlreadyRC = false;                                                  // Force a NO_CONTROL 1st before going into RC_MODE
        modeInConfirm = false;                                                  // remove confirm flag before we go to RC_MODE
        msgToSend=true;                                                         // Send what you composed in the Buffer here
     }
     else if ((di2PosHalt==0u) && (di1SpeedAngle==1u))                          // Input 2 and Input1 abd Input 3 all false    (Speed Mode)
     {
        // Initialise the header for CONTROL_MODE
        head.commandID=SBGC_CMD_CONTROL;                                        // Command ID is control mode
        head.sizeData=sizeof(cnew);                                             // Byte 3 of the header
        head.cmdSize=(head.commandID + head.sizeData) % 256U;                   // Byte 4 of the header

        // Calculate the Speeds from the ADC
        pitchSpeedReq=Scale_Raw_Value(0u,RAW_MAX,0.0f,SBGC_MAX_GIMBAL_SPEED,valueADC[1u]);      // range 0-200 deg/sec
        yawSpeedReq=Scale_Raw_Value(0u,RAW_MAX,0.0f,SBGC_MAX_GIMBAL_SPEED,valueADC[2u]);
        rollSpeedReq=Scale_Raw_Value(0u,RAW_MAX,0.0f,SBGC_MAX_GIMBAL_SPEED,valueADC[3u]);

        cnew.modeROLL = (SBGC_CONTROL_MODE_SPEED | (!di6FastSelect * SBGC_CONTROL_FLAG_HIGH_RES_SPEED));   // Set Control Mode to Speed (The Fast / Slow motion is same input switch used in Angle Mode)
        cnew.modePITCH = (SBGC_CONTROL_MODE_SPEED | (!di6FastSelect * SBGC_CONTROL_FLAG_HIGH_RES_SPEED));   // Set Control Mode to Speed (The Fast / Slow motion is same input switch used in Angle Mode)
        cnew.modeYAW = (SBGC_CONTROL_MODE_SPEED | (!di6FastSelect * SBGC_CONTROL_FLAG_HIGH_RES_SPEED));   // Set Control Mode to Speed (The Fast / Slow motion is same input switch used in Angle Mode)
        if (di6FastSelect)
        {
           cnew.speedROLL = (uint16_t) (rollSpeedReq * SBGC_SLOW_SPEED_SCALE);  // Set the roll speed using 0.001 deg/sec
           cnew.speedPITCH = (uint16_t) (pitchSpeedReq * SBGC_SLOW_SPEED_SCALE);// Set the pitch speed
           cnew.speedYAW = (uint16_t) (yawSpeedReq * SBGC_SLOW_SPEED_SCALE);    // Set the yaw speed
        }
        else
        {
           cnew.speedROLL = (uint16_t) (rollSpeedReq * SBGC_SPEED_SCALE);       // Set the roll speed using 0.122 deg/sec
           cnew.speedPITCH = (uint16_t) (pitchSpeedReq * SBGC_SPEED_SCALE);     // Set the pitch speed
           cnew.speedYAW = (uint16_t) (yawSpeedReq * SBGC_SPEED_SCALE);         // Set the yaw speed
        }

        memcpy(Buffer, &head, sizeof(head));                                    // Copy the header
        memcpy(Buffer+sizeof(head), &cnew, sizeof(cnew));                       // Copy the payload
        if (boardinforb.FIRMWARE_VER >= 2680U)                                  // If we have the new firmware loaded
        {
            newCRC = crc16_arc_calculate(sizeof(cnew), ptr_cnew);               // Calculate new 16 bit CRC for new version
            memcpy(Buffer+sizeof(cnew)+sizeof(head),(void *) &newCRC, sizeof(newCRC));   // Tag the CRC
            totalMsgLen = sizeof(head)+sizeof(cnew)+sizeof(newCRC);             // Message length
        }
        else
        {
            crc = checksum(ptr_cnew, sz_cnew);                                  // calculate the payload checksum
            memcpy(Buffer+sizeof(cnew)+sizeof(head),(void *) &crc,sizeof(crc));  // Copy the CRC
            totalMsgLen = sizeof(head) + sizeof(cnew) + sizeof(crc);            // Message length
        }
        stateEngine=stateEngine | JoySpeed;                                     // State Engine = Control Angle Speed
        modeAlreadyRC = false;                                                  // Force a NO_CONTROL 1st before going into RC_MODE
        modeInConfirm = false;                                                  // remove confirm flag before we go to RC_MODE
        msgToSend=true;                                                         // Send what you composed in the Buffer here
     }
     else if ((di5VCMode==1u) && (di2PosHalt==1u) && (di4RCMode==1u))              // Calibration Mode  ( di2 , di4 and di5 )
     {
         JOYCalAlarms.AlarmWdCurrent = CalibrateCenter( &calibrationObj );      // take and average of the center co-ordinate (use for correction if needed)
#ifdef USE_CAL_UNLOCK                                                           // if we are unlock type then initialise the last settings to find new ones
          calibrationObj.maxValx = 0U;                                          // call for new max values
          calibrationObj.maxValy = 0U;                                          // call for new max values
          calibrationObj.maxValz = 0U;                                          // call for new max values
          calibrationObj.minValx = (RAW_MAX>>1);                                // call for new min values
          calibrationObj.minValy = (RAW_MAX>>1);                                // call for new min values
          calibrationObj.minValz = (RAW_MAX>>1);                                // call for new min values
          calibrationObj.counter1 = 0U;                                         // call for new min values
#endif
         JOYCalAlarms.AlarmWdCurrent |= CalibrateSpan( &calibrationObj );
     }
     else                                                                          // Input 2 is on we have no control joystick off
     {
        stateEngine=stateEngine | JoyOff;                                       // State Engine = Joystick Off Mode
        msgToSend=false;                                                        // Dont send a command from the joystick
     }
  }
  DOTPORT1 = stateEngine;                                                       // Drive the outputs to the state chosen from the digital input switches.

  // Handler for : If we jumped out of the RC_MODE and left the confirm message ON
  //
  if (ccReturn == CMD_SENT)                                                     // already sent a control_configure request but no reply as yet
  {
     if (countControlCnf <= MSG_REPOLL_TICKS)                                   // Counter is below the re-send delay off time (ticks)
     {
        countControlCnf=++countControlCnf % UINT64_MAX;                         // Count for the time to delay the re-send if no confirm message
     }
     else
     {
        countControlCnf=0U;                                                      // Reset the message counter
        ccReturn = CMD_RESEND;                                                  // Re-issue the message
     }
  }
  else
  {
     ccReturn=ChgStat_CONFIG_for_CONTROL(&cc,false,sizeof(cc));                 // We can now stop the confirm for the CMD_CONTROL request
  }

  // Package the command in the Buffer and send to serial and UDP (could be func but as needs to be fast kept on top level )
  //
  if (msgToSend)
  {
    stateEngine = stateEngine | JoySend;                                        // Flash the LED to say sending
    memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                      // Null terminate the Buffer for serial
    for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
    {
       UART2_write(Buffer[SBGCcnt]);                                            // Send it to serial UART2
    }
    udpReturn = 0U;
    udpSent = 0U;
    while ((udpReturn == 0U) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
    {
       udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
       udpSent=++udpSent % UINT8_MAX;
    }
    if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;              // accumulate the bad send counter (remove the 1 before summing)
    msgToSend=false;
  }
#endif                                                                          // ==================== eNd simpleBGC Gimbal Joystick Controller =============

#if defined(SBGC_GIMBAL_HMI)
if (gimStartUpComplete==true)
{
// ======================== SBGC GIMBAL DATA ===================================
  // ===================== DATA periodic Polling ===============================
  //
  // Send data poll requests for CMD_GET_ANGLES. CMD_GET_ANGLES_EXT CMD_REALTIME_DATA_4
  //
  // POLL MODULUS = frequency
  // POLL SLOT = position in poll table
  //
  timedMsgToSend=CMD_SENT;                                                      // These are polls so dont care about response just re-poll for info
  if ((timedPolling % POLL_MODULUS)==0U)
  {
     // Initialise the header for GET_ANGLES
     head.commandID=SBGC_CMD_GET_ANGLES;                                        // Command ID is get_angles (data request for ComGS
     head.sizeData=0U;                                                          // Byte 3 of the header
     head.cmdSize=head.commandID;                                               // Byte 4 of the header
     memcpy(Buffer, &head, sizeof(head));                                       // Copy the header
     timedMsgToSend=CMD_SEND;
     DOTPORT2 = GetAngles;                                                         // Flash LED PORT D 0
  }
  else if (((timedPolling-POLL_SLOT) % POLL_MODULUS)==0U)
  {
      // Initialise the header for GET_ANGLES_EXT
     head.commandID=SBGC_CMD_GET_ANGLES_EXT;                                    // Command ID is get_angles_ext (data request for ComGS
     head.sizeData=0U;                                                          // Byte 3 of the header
     head.cmdSize=head.commandID;                                               // Byte 4 of the header
     memcpy(Buffer, &head, sizeof(head));                                       // Copy the header
     timedMsgToSend=CMD_SEND;
     DOTPORT2 = GetExtAngles;                                                   // Flash LED PORT D 1
  }
  else if (((timedPolling-(2U*POLL_SLOT)) % POLL_MODULUS)==0U)
  {
      // Initialise the header for SBGC_CMD_REALTIME_DATA_4
     head.commandID=SBGC_CMD_REALTIME_DATA_4;                                   // Command ID is realtime data 4 (data request for ComGS
     head.sizeData=0U;                                                          // Byte 3 of the header
     head.cmdSize=head.commandID;                                               // Byte 4 of the header
     memcpy(Buffer, &head, sizeof(head));                                       // Copy the header
     timedMsgToSend=CMD_SEND;
     DOTPORT2 = GetRealtime4;                                                   // Flash LED according to io definition in io.h
  }
  else
  {
     DOTPORT2 = DOTPORT2 & Clear3LED;                                           // Clear the LED
  }
  timedPolling=++timedPolling % UINT32_MAX;                                     // Increment the timer
  totalMsgLen=sizeof(head);                                                     // They are just the size of the header (no payload).
  if(timedMsgToSend==CMD_SEND)
  {
      // Package the command in the Buffer and send to serial and UDP
      memset(Buffer+totalMsgLen,(void *) '\0',sizeof(char));                    // Null terminate the Buffer for serial
      for(SBGCcnt=0;SBGCcnt<totalMsgLen;SBGCcnt++)
      {
        UART2_write(Buffer[SBGCcnt]);                                           // Send it to serial UART2
      }
      udpReturn = 0U;
      udpSent = 0U;
      while ((udpReturn == 0U) && (udpSent <= MAX_NUM_OF_UDP_SENDS))
      {
         udpReturn = Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_From_Port, SBGC_Dest_Port, Buffer, totalMsgLen);        // send the Buffer contents as UDP datagram.
         udpSent=++udpSent % UINT8_MAX;
      }
      if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;            // accumulate the bad send counter (remove the 1 before summing)
  }
}
#endif                                                                          // ============ eNd data to the HMI for the gimbal ========================

// ======================= BAD COMMS RESET ETHER ===============================
  //
  // If we got a lot of bad sends - do when fail occura and after MAX_REPEAT_TIME expired
  //
  if ((((totalBadUDPSends) > MAX_NUM_BAD_UDP)&&!resetDone) || (((totalBadUDPSends) > MAX_NUM_BAD_UDP)&&(resetDone && (cycTotalTime > MAX_REPEAT_TIME))))                                   // Total bad in one cycle exceeds the limit
  {
      boot_arp();
      if (!Air.Link_Established)                                                // try ARP resolve if no success then init ether and try again
      {
         Init_Ether();                                                          //  When 1 returned then we must re-start ethernet controller refer manual.
         boot_arp();                                                            //  Start ARP and wait for connection
      }
#if defined(SBGC_GIMBAL_JOY) || defined(SBGC_GIMBAL_HMI)
      SBGC_Play_Sound( melodyACP3, 4U);                                         //  Play a sound on the gimbal to let you know
#endif
      resetDone=!Air.Link_Established;                                          // if we didnt connect then wait for timeout before repeat
      cycTotalTime=0U;                                                          // when reset done reset the time counter
      cycTimeDuration=-1U;                                                      // initialise the counter upon reset
  }
  totalBadUDPSends=0U;                                                          // reset the bad send counter
  if (resetDone)                                                                // Link not established after reset performed
  {
      calculateTimeDiff( &cycTimeDuration, &cycTimeDurationLast);                 // calculate a time since last called
      cycTotalTime =+ cycTimeDuration;                                          // calculate a total cycle time
  }

// ================ READ ETHER DATA ============================================
//-------------- Ethernet TCP Read Data ----------------------------------------
//
// If we read some ethernet and it was a TCP Packet for YiCam process it
//
//
#if defined(YI_CAM_USED)
  // bound the YiCam selections from the GUI to the allowable options
  // prevent array out of bounds
  // can move this to HMI button code in some instance
  //
  hmiXYOption.VideoStd=(((hmiXYOption.VideoStd)%(MAX_VID_STD-1U))+1U);
  if (!(strcmp((char*) xy_video_standard[NTSC], (char*) xy_video_standard[hmiXYOption.VideoStd])))    // If you choose NTSC then choose those options otherwise PAL
     hmiXYOption.VideoRes=((hmiXYOption.VideoRes)%(MAX_VID_RES-NO_OF_VID_RES_TYPE)+NO_OF_VID_RES_TYPE);  // NTSC
  else
     hmiXYOption.VideoRes=((hmiXYOption.VideoRes)%(MAX_VID_RES-NO_OF_VID_RES_TYPE)+NO_OF_VID_RES_TYPE+XY_NO_OF_NTSC);  // PAL
 
  hmiXYOption.PhotoSz=(((hmiXYOption.PhotoSz)%(MAX_PHOTO_SZ-1U))+1U);
  hmiXYOption.TimeLapOpt=(((hmiXYOption.TimeLapOpt)%(MAX_TIMELAP-1U))+1U);
  hmiXYOption.TimeLapDuration=(((hmiXYOption.TimeLapDuration)%(MAX_TLPVID-1U))+1U);
  hmiXYOption.SlowMotion=(((hmiXYOption.SlowMotion)%(MAX_SLOWMOT-1U))+1U);
  hmiXYOption.BurstCap=(((hmiXYOption.BurstCap)%(MAX_BURST-1U))+1U);
  hmiXYOption.VidPhoQ=(((hmiXYOption.VidPhoQ)%(MAX_QUALITY-2U))+2U);
  hmiXYOption.VidPhoStamp=(((hmiXYOption.VidPhoStamp)%(MAX_STAMP-2U))+2U);
  hmiXYOption.PreContTim=(((hmiXYOption.PreContTim)%(MAX_CONT_TIME-1U))+1U);
  hmiXYOption.RecPhoTim=(((hmiXYOption.RecPhoTim)%(MAX_RECTIM-1U))+1U);
  hmiXYOption.LoopRecDur=(((hmiXYOption.LoopRecDur)%(MAX_LOOPTIM-1U))+1U);
  hmiXYOption.OnOffTypSel=((hmiXYOption.OnOffTypSel)%(MAX_ONOFF-1U));
  hmiXYOption.VidOutDev=((hmiXYOption.VidOutDev)%(MAX_VIDOUT-1U));
  hmiXYOption.MeterOpt=((hmiXYOption.MeterOpt)%(MAX_METER-1U));
  hmiXYOption.LedMode=((hmiXYOption.LedMode)%(MAX_LED-1U));
  hmiXYOption.BuzzVol=((hmiXYOption.BuzzVol)%(MAX_BUZZER-1U));
  hmiXYOption.CapMode=((hmiXYOption.CapMode)%(MAX_CAP_MODE-2U));
  
  if(doPacketReturn==0)                                                         // one scan later process the incoming YiCam message if it had been found
  {
     switch(YiCam_DATA.State)                                                   // States of the Yi Action Cam
     {
        case ETH_WAIT_ACK_CRC:                                                  // You just received a TCP packet so send an ACK back
        if (Net_Ethernet_Intern_startSendTCP(YiSocket)==0u)                     // 1 - error occur. 0 - routine sent dummy ACK to remote host. If remote host answer, Net_Ethernet_Intern_UserTCP routine will be called, and than We can put our data in TCP Tx buffer
        {
           YiCam_DATA.State = ETH_ACK_SENT_TCP;                                 // Dummy ACK was sent
        }
        // break;                                                               then do the reading of the incoming message

        case ETH_ACK_SENT_TCP:                                                  // Sent the ACK for the incoming packet
        XY_ParseState=xy_parse_reply((char*) &YiCam_DATA.TCP_Buffer, &XYjsonReply,XY_READ_ALL,&XYConfig);
        memcpy((void *) &XY_ParseStruct, (void *) &XY_ParseState, sizeof(XY_ParseState));     // copy into the structure which defines the bits
        if (ReportYiCamtoHMI(&XY_ParseStruct, &hmiYiValues, &XYjsonReply )==1u); // update the screen with the replies
        YiCam_DATA.State = ETH_PROCESSED_TCP;                                   // processed the incoming TCP packet
        break;

        default:                                                                // neither waiting to send an ACK nor reading an incoming packet
        break;                                                                  // no action
     }
  }
  TCPStateCheck( YiSocket );                                                    // Check the states of the TCP socket and try to keep it open

  switch (g_YiCamReqState)
  {
     case 0u:
     //if (hmiReqActive.getToken==1)                                              // Send the stream and get the token
     //{
        // memcpy(g_XYtcpBuffer,API_XY_INIT_FAST, strlen(API_XY_INIT_FAST));       // Command to start the stream
        API_XY_INIT_SESSION(g_XYtcpBuffer);                                     // Alternative start command
        g_YiCamReqState = 5u;                                                   // Now queue the TCP message to be sent
     //}
     break;
         
     case 1u:                                                                   // Wait for the token to come back
     if ((XYjsonReply.token != 0U) && ((XY_ParseStruct.f_rval) && (XY_ParseStruct.f_token)))      // response returned a token and rval
     {
         switch(XYjsonReply.rval)
         {
            case XY_RESP_OK:                                                    // No Error
            XYuseToken=XYjsonReply.token;
            //g_YiCamReqState = 2u;                                               // Now look for any other TCP message to be sent (stream is started)
            g_YiCamReqState = 10u;                                              // get the full camera configuration
            break;

            case XY_RESP_REQ_NEW_TOKEN:                                         // request a new token (input object is empty)
            g_YiCamReqState = 0u;                                               // try again
            break;
            
            case XY_RESP_CRASH:                                                 // camera crash
            g_YiCamReqState = 0u;                                               // try again
            break;
            
            case XY_RESP_INVALID:                                               // invalid response
            g_YiCamReqState = 0u;                                               // try again
            break;
            
            case XY_INVALID_JSON_OBJ:                                           // invalid JSON object
            // Exit ?
            break;
            
            case XY_INVALID_CMD:                                                // invalid command
            g_YiCamReqState = 0u;                                               // try again
            break;
            
            default:                                                            // return code not found defined ?
            // Exit ?
            break;
         }
     }
     break;
     
     case 2u:                                                                   // Now send any other request that are required
     XYRequest=XYRequest % (53u +1u);                                           // set to range to be the number of steps below 0-51
     switch (XYRequest)
     {
        case 0u: 
        if (hmiReqActive.batteryLeft==1)
        {
           XY_API_BAT_LVL(g_XYtcpBuffer,XYuseToken);                            // get battery Status
        // memcpy(g_XYtcpBuffer,XY_API_BAT_LVL_FAST(XYuseToken), strlen(XY_API_BAT_LVL_FAST(XYuseToken)));       // Command to get the battery status
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        
        case 1u: 
        if (hmiReqActive.startEvent==1u)
        {
            XY_API_AUTOSTAT(g_XYtcpBuffer,XYuseToken);                          // start the event reporter
            // memcpy(g_XYtcpBuffer,XY_API_AUTOSTAT_FAST(XYuseToken), strlen(XY_API_AUTOSTAT_FAST(XYuseToken)));       // Command to start the event reporter
            g_YiCamReqState = 6u;                                                   // Now queue the TCP message to be sent
            break;
        }
        else
        {
            XYRequest = ++XYRequest % UINT8_MAX;                                // No request look at the next case
        }

        
        case 2u: 
        if (hmiReqActive.tkPhoto==1u)
        {
           XY_API_TK_PHO(g_XYtcpBuffer,XYuseToken);                             // take a photo  reply is "type": "start_photo_capture"
           //memcpy(g_XYtcpBuffer,XY_API_TK_PHO_FAST(XYuseToken), strlen(XY_API_TK_PHO_FAST(XYuseToken)));       // Command to start taking photo
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }


        case 3u:  
        if (hmiReqActive.setPIV==1u)
        {
           XY_API_SND_PIV(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_SND_PIV_FAST(XYuseToken), strlen(XY_API_SND_PIV_FAST(XYuseToken)));       // Command to send PIV ?
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        
        case 4u:
        if (hmiReqActive.getRecTime==1u)
        {
           XY_API_GET_RCT(g_XYtcpBuffer,XYuseToken);
        // memcpy(g_XYtcpBuffer,XY_API_GET_RCT_FAST(XYuseToken), strlen(XY_API_GET_RCT_FAST(XYuseToken)));       // Command sendGetRecordTime()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }


        case 5u:
        if (hmiReqActive.startRecord==1u)
        {
           XY_API_START_REC(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_START_REC_FAST(XYuseToken), strlen(XY_API_START_REC_FAST(XYuseToken)));       // Command sendStartRecord()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }


        case 6u:
        if (hmiReqActive.stopRecord==1U)
        {
           XY_API_STOP_REC(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_STOP_REC_FAST(XYuseToken), strlen(XY_API_STOP_REC_FAST(XYuseToken)));       // Command sendStopRecord()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
            XYRequest = ++XYRequest % UINT8_MAX;                                // No request look at the next case
        }


        case 7u:
        if (hmiReqActive.ccsStop==1U)
        {
           XY_API_CCS(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_CCS_FAST(XYuseToken), strlen(XY_API_CCS_FAST(XYuseToken)));       // Command sendContinueCaptureStop (Null)
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        
        case 8u:
        if (hmiReqActive.quickRecord==1U)
        {
           XY_API_SND_QCK_REC_STRT(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_SND_QCK_REC_STRT_FAST(XYuseToken), strlen(XY_API_SND_QCK_REC_STRT_FAST(XYuseToken)));       // Command sendQuickRecordStart()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }


        case 9u:
        if (hmiReqActive.quickRecPause==1u)
        {
           XY_API_SND_QCK_REC_PAU(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_SND_QCK_REC_PAU_FAST(XYuseToken), strlen(XY_API_SND_QCK_REC_PAU_FAST(XYuseToken)));       // Command sendQuickRecordPause()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }


        case 10u:
        if (hmiReqActive.quickRecResume==1U)
        {
           XY_API_SND_QCK_REC_RES(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_SND_QCK_REC_RES_FAST(XYuseToken), strlen(XY_API_SND_QCK_REC_RES_FAST(XYuseToken)));       // Command sendQuickRecordResume()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }


        case 11u:
        if (hmiReqActive.resetWifi==1U)
        {
           XY_API_SND_RST_WIFI(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_SND_RST_WIFI_FAST(XYuseToken), strlen(XY_API_SND_RST_WIFI_FAST(XYuseToken)));       // Command sendRestartWiFi()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }


        case 12u:
        if (hmiReqActive.videoStd==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_video_standard[video_standard],xy_video_standard[hmiXYOption.VideoStd]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        
        case 13u:
        if (hmiReqActive.videoLoopRes==1)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_video_res[VIDRESPARAM_1],xy_video_res[hmiXYOption.VideoRes]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_video_res[VIDRESPARAM_1],xy_video_res[hmiXYOption.VideoRes]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_video_res[VIDRESPARAM_1],xy_video_res[hmiXYOption.VideoRes])));     // Command sendSetting( video_loop_resolution, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }


        case 14u:
        if (hmiReqActive.videoPhoRes==1u)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_video_res[VIDRESPARAM_2],xy_video_res[hmiXYOption.VideoRes]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_video_res[VIDRESPARAM_2],xy_video_res[hmiXYOption.VideoRes]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_video_res[VIDRESPARAM_2],xy_video_res[hmiXYOption.VideoRes])));     // Command sendSetting( video_photo_resolution, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }


        case 15u:
        if (hmiReqActive.videoLapseRes==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_video_res[VIDRESPARAM_3],xy_video_res[hmiXYOption.VideoRes]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_video_res[VIDRESPARAM_3],xy_video_res[hmiXYOption.VideoRes]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_video_res[VIDRESPARAM_3],xy_video_res[hmiXYOption.VideoRes])));     // Command sendSetting( timelapse_video_resolution, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }


        case 16u:
        if (hmiReqActive.videoRes==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_video_res[VIDRESPARAM_4],xy_video_res[hmiXYOption.VideoRes]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_video_res[VIDRESPARAM_4],xy_video_res[hmiXYOption.VideoRes]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_video_res[VIDRESPARAM_4],xy_video_res[hmiXYOption.VideoRes])));     // Command sendSetting( video_resolution, option )
           g_YiCamReqState = 6u;                                                   // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }


        case 17u:
        if (hmiReqActive.photoSize==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_photo_sz[photo_sz],xy_photo_sz[hmiXYOption.PhotoSz]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_photo_sz[photo_sz],xy_photo_sz[hmiXYOption.PhotoSz]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_photo_sz[photo_sz],xy_video_res[hmiXYOption.PhotoSz])));     // Command sendSetting( photo_size, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }
        
        case 18u:
        if (hmiReqActive.idleMode==1U)
        {
           XY_API_SND_STATE_IDLE(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_SND_STATE_IDLE_FAST(XYuseToken), strlen(XY_API_SND_STATE_IDLE_FAST(XYuseToken)));     // Command sendIdleMode()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }
        
        case 19u:
        if (hmiReqActive.wakeUpMode==1U)
        {
           XY_API_SND_STATE_WAKE(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_SND_STATE_WAKE_FAST(XYuseToken), strlen(XY_API_SND_STATE_WAKE_FAST(XYuseToken)));     // sendWakeUpMode()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }
        
        case 20u:
        if (hmiReqActive.logStart==1U)
        {
           XY_API_LOG_START(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_LOG_START_FAST(XYuseToken), strlen(XY_LOG_START_FAST(XYuseToken)));     // sendStartSaveLog()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 21u:
        if (hmiReqActive.resetStream==1U)
        {
           XY_API_RESET_STREAM(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_RESET_STREAM_FAST(XYuseToken), strlen(XY_API_RESET_STREAM_FAST(XYuseToken)));     // sendResetVF( none_force )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 22u:
        if (hmiReqActive.stopStream==1U)
        {
           XY_API_STOP_STREAM(g_XYtcpBuffer,XYuseToken);
           //memcpy(g_XYtcpBuffer,XY_API_STOP_STREAM_FAST(XYuseToken), strlen(XY_API_STOP_STREAM_FAST(XYuseToken)));     // sendStopVF()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 23u:
        if (hmiReqActive.bindBlue==1U)
        {
           XY_API_BBD(g_XYtcpBuffer,XYuseToken);
           //memcpy(g_XYtcpBuffer,XY_API_BBD_FAST(XYuseToken), strlen(XY_API_BBD_FAST(XYuseToken)));     // sendIsBindedBluetoothDevs()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 24u:
        if (hmiReqActive.bindBlue==1U)
        {
           XY_API_UBIND_BLUE(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_UBIND_BLUE_FAST(XYuseToken), strlen(XY_API_UBIND_BLUE_FAST(XYuseToken)));     // sendUnbindBluetoothDevs()
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 25u:
        if (hmiReqActive.lapseVidTime==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_tlpvid_opt[timelapse_video_duration],xy_tlpvid_opt[hmiXYOption.TimeLapDuration]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_tlpvid_opt[timelapse_video_duration],xy_tlpvid_opt[hmiXYOption.TimeLapDuration]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_video_res[timelapse_video_duration],xy_video_res[hmiXYOption.TimeLapDuration])));     // Command sendSetting( timelapse_video_duration, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }
        
        case 26u:
        if (hmiReqActive.lapseVidOpt==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_timelap_opt[timelapse_video],xy_timelap_opt[hmiXYOption.TimeLapOpt]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_timelap_opt[timelapse_video],xy_timelap_opt[hmiXYOption.TimeLapOpt]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_video_res[timelapse_video],xy_video_res[hmiXYOption.TimeLapOpt])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 27u:
        if (hmiReqActive.slowMotionRt==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_slowmot_opt[slow_motion_rate],xy_slowmot_opt[hmiXYOption.SlowMotion]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_slowmot_opt[slow_motion_rate],xy_slowmot_opt[hmiXYOption.SlowMotion]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_slowmot_opt[slow_motion_rate],xy_slowmot_opt[hmiXYOption.SlowMotion])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 28u:
        if (hmiReqActive.burstCap==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 29u:
        if (hmiReqActive.preciseContTim==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_conttime_opt[precise_cont_time],xy_conttime_opt[hmiXYOption.PreContTim]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 30u:
        if (hmiReqActive.recPhoTime==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_rectim_opt[record_photo_time],xy_rectim_opt[hmiXYOption.RecPhoTim]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 31u:
        if (hmiReqActive.loopRecDur==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_looptim_opt[loop_rec_duration],xy_looptim_opt[hmiXYOption.LoopRecDur]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 32u:
        if (hmiReqActive.videoQ==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_vidpho_qual[vid_qual],xy_vidpho_qual[hmiXYOption.VidPhoQ]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 33u:
        if (hmiReqActive.photoQ==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_vidpho_qual[photo_qual],xy_vidpho_qual[hmiXYOption.VidPhoQ]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 34u:
        if (hmiReqActive.vidStampVal==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_vidpho_stamp[video_stamp_val],xy_vidpho_stamp[hmiXYOption.VidPhoStamp]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 35u:
        if (hmiReqActive.phoStampVal==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_vidpho_stamp[photo_stamp_val],xy_vidpho_stamp[hmiXYOption.VidPhoStamp]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 36u:
        if ((hmiXYOption.OnOffState == 1U) && (hmiReqActive.onOffOpt==1U))   // The selection either on or off
        {
           XY_API_SND_ON(g_XYtcpBuffer,XYuseToken,xy_onofftypes_opt[hmiXYOption.OnOffTypSel]);
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else if ((hmiXYOption.OnOffState == 0U) && (hmiReqActive.onOffOpt==1U))
        {
           XY_API_SND_OFF(g_XYtcpBuffer,XYuseToken,xy_onofftypes_opt[hmiXYOption.OnOffTypSel]);
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }
        // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )


        case 37u:
        if (hmiReqActive.vidOutDev==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_vidoutdev_opt[video_output_dev_type],xy_vidoutdev_opt[hmiXYOption.VidOutDev]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 38u:
        if (hmiReqActive.meterOpt==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_meter_opt[meter_mode],xy_meter_opt[hmiXYOption.MeterOpt]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 39u:
        if (hmiReqActive.ledMode==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_led_opt[led_mode_val],xy_led_opt[hmiXYOption.LedMode]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 40u:
        if (hmiReqActive.buzzerOpt==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_buzzer_opt[buzzer_volume],xy_buzzer_opt[hmiXYOption.BuzzVol]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 41u:
        if (hmiReqActive.captureMode==1U)
        {
           XY_API_SND_SET(g_XYtcpBuffer,XYuseToken,xy_capmode_opt[capture_mode],xy_capmode_opt[hmiXYOption.CapMode]);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 42u:
        if (hmiReqActive.fastZoom==1U)
        {
           XY_API_SND_SET_FAST_ZOOM(g_XYtcpBuffer,XYuseToken,XY_zoomString);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 43u:
        if (hmiReqActive.bitRate==1U)
        {
           XY_API_BITRATE(g_XYtcpBuffer,XYuseToken,XY_hmiBitRate);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 44u:
        if (hmiReqActive.delFile==1U)
        {
           XY_API_DEL_FILE(g_XYtcpBuffer,XYuseToken,XY_hmiDelFilNm);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 45u:
        if (hmiReqActive.format==1U)
        {
           XY_API_SND_FMT(g_XYtcpBuffer,XYuseToken,XY_formatString);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 46u:
        if (hmiReqActive.dir==1U)
        {
           XY_API_DIR(g_XYtcpBuffer,XYuseToken,XY_dirString);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 47u:
        if (hmiReqActive.sndGetFile==1U)
        {
           XY_API_GET_FIL(g_XYtcpBuffer,XYuseToken,XY_hmiSndGetFilNm,XY_offset,XY_fet);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 48u:
        if (hmiReqActive.getMedia==1U)
        {
           XY_API_GET_MDA(g_XYtcpBuffer,XYuseToken,XY_hmiGetMDA);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 49u:
        if (hmiReqActive.thumbNail==1U)
        {
           XY_API_SND_GET_TH(g_XYtcpBuffer,XYuseToken,XY_thumbInt,XY_hmiThumb);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 50u:                                                               // enable a script you must know md5 info and size e.g. '/tmp/fuse_d/commands/exec.ash'
        if (hmiReqActive.enabScript==1U)
        {
           XY_API_SND_PUT_FILE(g_XYtcpBuffer,XYuseToken,XY_md5,"enable_info_display.script",XY_size);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 51u:                                                               // get the name of the last photo
        if (hmiReqActive.getLastPho==1U)
        {
           XY_API_GET_LAST_PHO(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }

        case 52u:                                                               // get the free space in bytes on the SD Card
        if (hmiReqActive.getFreeSD==1U)
        {
           XY_API_GET_SPC_FREE(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }
        
        case 53u:                                                               // get the total space of the SD Card
        if (hmiReqActive.getTotalSD==1U)
        {
           XY_API_GET_SPC_TOT(g_XYtcpBuffer,XYuseToken);
           // memcpy(g_XYtcpBuffer,XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap]), strlen(XY_API_SND_SET_FAST(XYuseToken,xy_burstcap_opt[param_burst_cap_no],xy_burstcap_opt[hmiXYOption.BurstCap])));     // Command sendSetting( timelapse_video, option )
           g_YiCamReqState = 6u;                                                // Now queue the TCP message to be sent
           break;
        }
        else
        {
           XYRequest = ++XYRequest % UINT8_MAX;                                 // No request look at the next case
        }
        
        default:
          XYRequest=0U;                                                         // start again
        break;
     }
     break;
     
     case 3u:                                                                   // Wait for the readback
     switch (XYRequest)                                                         // Check what request you made
     {
        case 0u:                                                                // get battery Status
        if ((XY_Check_Return_Code( XY_BAT_LVL_CMD ))==-1)                       // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0)                                            // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat the next command
        }
        break;

        case 1u:                                                                // started the event reporter
        if ((XY_Check_Return_Code( XY_STATUS_CMD ))==-1)                       // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0)                                            // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat the next command
        }
        break;

        case 2u:                                                                // take a photo  ((( 2 stage reply --- look at rval first )))
        if ((XY_Check_Return_Code( XY_SND_TK_PHO_CMD ))==-1)                    // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0)                                            // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 100u:                                                              // second stage of photo taking look for the event
        if ((XY_ParseStruct.f_type) && (XY_ParseStruct.f_msg_id))               // response returned a type and msg_id
        {
           if ((XYjsonReply.msg_id == 7u) && (!strcmp(XYjsonReply.type_str,"start_photo_capture")))   // expected start state message reply
           {
              XYRequest=++XYRequest % UINT8_MAX;                                // Go to the next request
              g_YiCamReqState = 2u;                                             // Now look for the next TCP message to be sent (stream is started)
           }
           else
           {
              g_YiCamReqState = 2u;                                             // try again with same send
           }
        }
        break;

        case 3u:
        if ((XY_Check_Return_Code( XY_SND_PIV_CMD ))==-1)                       // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 4u:
        if ((XY_Check_Return_Code( XY_SND_GET_RCT_CMD ))==-1)                   // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 5u:
        if ((XY_Check_Return_Code( XY_SND_STRT_REC_CMD ))==-1)                  // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 6u:
        if ((XY_Check_Return_Code( XY_SND_STOP_REC_CMD ))==-1)                  // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 7u:
        if ((XY_Check_Return_Code( XY_CCS_CMD ))==-1)                           // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 8u:
        if ((XY_Check_Return_Code( XY_SND_QCK_REC_STRT_CMD ))==-1)              // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 9u:
        if ((XY_Check_Return_Code( XY_SND_QCK_REC_PAU_CMD ))==-1)               // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 10u:
        if ((XY_Check_Return_Code( XY_SND_QCK_REC_RES_CMD ))==-1)               // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 11u:
        if ((XY_Check_Return_Code( XY_SND_RST_WIFI_CMD ))==-1)                  // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 17u:
        if ((XY_Check_Return_Code( XY_SND_STATE_IDLE_CMD ))==-1)                // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 18u:
        if ((XY_Check_Return_Code( XY_SND_STATE_WAKE_CMD ))==-1)                // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 20u:
        if ((XY_Check_Return_Code( XY_SND_RST_VF_CMD ))==-1)                    // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 21u:
        if ((XY_Check_Return_Code( XY_SND_STOP_VF_CMD ))==-1)                   // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 22u:
        if ((XY_Check_Return_Code( XY_BBD_CMD ))==-1)                           // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 23u:
        if ((XY_Check_Return_Code( XY_SND_UBIND_BLUE_CMD ))==-1)                // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 42u:
        if ((XY_Check_Return_Code( XY_SND_SET_ZOOM_CMD ))==-1)                   // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 43u:
        if ((XY_Check_Return_Code( XY_BITRATE_CMD ))==-1)                       // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0U)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 44u:
        if ((XY_Check_Return_Code( XY_DEL_FILE_CMD ))==-1)                      // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0U)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 45u:
        if ((XY_Check_Return_Code( XY_SND_FMT_CMD ))==-1)                      // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0U)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 46u:
        if ((XY_Check_Return_Code( XY_DIR_CMD ))==-1)                           // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0U)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 47u:
        if ((XY_Check_Return_Code( XY_SND_GET_FIL_CMD ))==-1)                   // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0U)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 48u:
        if ((XY_Check_Return_Code( XY_SND_GET_MDA_CMD ))==-1)                   // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0U)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 49u:
        if ((XY_Check_Return_Code( XY_SND_GET_TH_CMD ))==-1)                    // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0U)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 50u:
        if ((XY_Check_Return_Code( XY_SND_PUT_FILE_CMD ))==-1)                  // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0U)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 51u:
        if ((XY_Check_Return_Code( XY_GET_LAST_PHO_CMD ))==-1)                    // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0U)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 52u:
        if ((XY_Check_Return_Code( XY_SND_GET_SPC_CMD ))==-1)                    // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0U)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;

        case 53u:
        if ((XY_Check_Return_Code( XY_SND_GET_SPC_CMD ))==-1)                    // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0U)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;
        
        default:                                                                // Any other values > 24 then it was a set command request
        if ((XY_Check_Return_Code( XY_SND_SET_SET_CMD ))==-1)                   // did we get the correct reply for the AJAX request otherwise timeout step
        {
             xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;          // increment the state timer until it reaches the timeout preset value and resets
             if (xyCamStateTimer==0u)                                           // timer pre-set has been met
                g_YiCamReqState = 2u;                                           // change the step back to repeat command
        }
        break;
     }
     break;

     case 10u:                                                                  // request full configuration request
     XY_API_GET_SET_ALL(g_XYtcpBuffer,XYuseToken);
     g_YiCamReqState = 7u;                                                      // Now queue the TCP message to be sent
     break;

     case 11u:                                                                  // read back the configuration
     if ((XY_Check_Return_Code( XY_GET_SET_CMD ))==-1)                          // did we get the correct reply for the AJAX request otherwise timeout step
     {
        xyCamStateTimer=++xyCamStateTimer % XY_STATE_PRE_MAX_TIM;               // increment the state timer until it reaches the timeout preset value and resets
        if (xyCamStateTimer==0u)                                                // timer pre-set has been met
          g_YiCamReqState = 10u;                                                // change the step back to repeat command
     }
     g_YiCamReqState = 2u;                                                      // start the send sequence
     break;
     
     case 4u:                                                                   // WAit for an action
     break;
     
     case 5u:                                                                   // WAit for the routine to write the start-up data it will send back to step 1 once done
     break;
     
     case 6u:                                                                   // Wait for the routine to write the rest of the data
     break;

     case 7u:                                                                   // Wait for the routine to return the full configuration
     break;
     
     default:                                                                   // Wrong state reset to the start
     break;
  }
  
#endif                                                                          // ================== eNd Yi Action Cam data section ======================

#if (((defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)) || (CAMERA_TYPE == XS_CAM)) || defined(RUN_CAM_USED))
//-------------- Ethernet Read Data ------------------------------------------
//
// Read the UDP and TCP incoming datagrams and understand the message
//
//
  doPacketReturn=Net_Ethernet_Intern_doPacket();                                //  Process incoming UDP and TCP packets using functions and send TCP queue
  if(doPacketReturn==1u)                                                        // On an ethernet error
  {
      Init_Ether();                                                             //  When 1 returned then we must re-start ethernet controller refer manual.
      boot_arp();                                                               //  Start ARP
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
      SBGC_Play_Sound( melodyACP2, 4U);                                         //  Play a sound on the gimbal to let you know we reset the MCU
#endif
  }
  else if(doPacketReturn==0u)                                                   //  The UDP Packet incoming was good and for us so look at it
  {
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
     if (SBGC_DATA.State == ETH_UDP_PACKET_RECEIVED)
     {
        msgType=SBGC_process_UDP_InBuffer( &boardinforb );                      // Read what you had in the SBGC UDP datagram data segment.
        // Uart5_write_text("Packet seen for us\n\r");
        SBGC_Process_Data_Read( msgType );                                      // Place the data from the Buffer as read response from gimbal controller
#if defined(SBGC_ACK_USED)
        SBGC_DATA.State=ETH_WAIT_ACK_CRC;                                       // set towait for ACK
#else
        SBGC_DATA.State=ETH_UDP_PACKET_PROCESSED;                               // unlatch receive state we processed it
#endif
     }
#endif
#if (CAMERA_TYPE == XS_CAM)
     if (XS_DATA.State == ETH_UDP_PACKET_RECEIVED)
     {
        XSEncoderResp=XS_process_UDP_InBuffer();                                // See if we got a confirm response from the XSStream encoder
#if defined(XS_ACK_USED)
        XS_DATA.State=ETH_WAIT_ACK_CRC;                                       // set towait for ACK
#else
        XS_DATA.State=ETH_UDP_PACKET_PROCESSED;                               // unlatch receive state we processed it
#endif
     }
#endif
#if defined(RUN_CAM_USED)
#if defined(UART5_INTERUPT)
     if (UART5.State == UART5_PACKET_IN_BUFFER)                                 /* something was received on the interrupt */
     {
        RCCameraResp=ReadRunCam( &UART5.Buffer, UART5.Index );                  /* parse the incoming packet */
        switch (RCCameraResp)                                                   /* test the message response from the parse */
        {
            case RC_MSG_INCOMP:                                                 /* message incomplete keep collecting */
            break;
            
            default:                                                            /* good or fail start collecting again as a new frame */
            UART5.State=UART5_BUFFER_EMPTY;
            break;
        }
     }
#endif
     if (RunC_DATA.State == ETH_UDP_PACKET_RECEIVED)
     {
        RCCameraResp=ReadRunCam( &RunC_DATA.UDP_Buffer, sizeof(RunC_DATA.UDP_Buffer)); /* parse the message recieved on the Run Cam UDP Port */
        switch (RCCameraResp)                                                   /* test the message response from the parse */
        {
            case RC_MSG_INCOMP:                                                 /* message incomplete keep collecting */
            break;

            default:
#if defined(XS_ACK_USED)
            RunC_DATA.State=ETH_WAIT_ACK_CRC;                                   /* set towait for ACK  */
#else
            RunC_DATA.State=ETH_UDP_PACKET_PROCESSED;                           /* unlatch receive state we processed it */
#endif
            break;
       }
     }
#endif
#if defined(GEF_EGD_PLC)
     if (EGD_DATA.State == ETH_UDP_PACKET_RECEIVED)
     {
        respEGD=ReadEGD( &EGD_DATA.UDP_Buffer, sizeof(EGD_DATA.UDP_Buffer));    // Check for EGD messages on the GEFANUC UDP Port
#if defined(XS_ACK_USED)
        EGD_DATA.State=ETH_WAIT_ACK_CRC;                                       // set towait for ACK
#else
        EGD_DATA.State=ETH_UDP_PACKET_PROCESSED;                               // unlatch receive state we processed it
#endif
     }
#endif
#if defined(ART_NET_USED)
     if (artNet_DATA.State == ETH_UDP_PACKET_RECEIVED)
     {
        respEGD=ReadArtNet( &artNet_DATA.UDP_Buffer, sizeof(artNet_DATA.UDP_Buffer)); // Check for ArtNet messages on the UDP Port
#if defined(XS_ACK_USED)
        artNet_DATA.State=ETH_WAIT_ACK_CRC;                                     // set towait for ACK
#else
        artNet_DATA.State=ETH_UDP_PACKET_PROCESSED;                             // unlatch receive state we processed it
#endif
     }
#endif
  }
  else                                                                          // The packet was not for our ip or type.
  {
     // Uart5_write_text("Packet seen not for us\n\r");                            // not for us or idle
  }

// ================ SBGC GIMBAL SERIAL UART 2 READ =============================
// Process the serial message if it has been enabled (definitions.h)
// Messages can be serial or ethernet
//
#if (defined(UART2_INTERUPT) && (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)))  // This is set if we are enabling UART2 with SimpleBGC serial
  if (UART2.State == UART2_PACKET_IN_BUFFER)                                    // You got a complete message @ UART2
  {
     SBGC_Process_Data_Read( UART2.Msg_id_recv );                               // Place the data from the Buffer as read response from gimbal controller
     UART2.State = UART2_BUFFER_EMPTY;                                          // Set the state of the UART2 Buffer to be read complete
  }
#endif

// ============= XS ENCODER SERAIL UART 4 READ =================================
#if (defined(UART4_INTERUPT) && (CAMERA_TYPE == XS_CAM))                        // This is set if we are enabling UART4 with XS Encoder serial protocol
  if (UART4.State >= UART4_PACKET_IN_BUFFER)                                    // You got a complete message @ UART4
  {
     XSEncoderResp=XS_process_SERIAL_InBuffer();                                // See if we got a confirm response from the XSStream encoder
     UART4.State = UART4_BUFFER_EMPTY;                                          // Set the state of the UART4 Buffer to be read complete
  }
#endif

#endif                                                                          // ======== eND section on SBGC or XS Cam or Run Cam =========================

// ============= Check HMI RTSP did we change it ===============================
#if (CAMERA_TYPE == XS_CAM)
   COMGS_New_Network = ((is_valid_ip(hmiEncoderIP)==1u) & (is_valid_ip(hmiEncoderGW)==1u));  // The HMI entries are a valid ip address
   COMGS_New_Network &= !((strcmp(hmiEncoderIP,XSencoderIPUse)==0u) & (strcmp(hmiEncoderGW,XSencoderIPUse)==0u));  // Change of state between HMI and program detected

// ============= XS ENCODER NEEDS LOGIN ========================================
   if ((XSEncoderResp == XS_AUTH) || (XSLoginTime > MSG_REPOLL_TICKS))          // The response was we need to autherise then re-log in
   {
       XS_encoder_Q.loggedin=false;                                             // set a flag to turn off any camera polls until auth complete
       XSLoginTime=0U;                                                          // reset the  timer
       XSEncoderResp = 0U;                                                      // clear the response
       memcpy(Buffer,&XStreamLogin,sizeof(XStreamLogin));
       totalMsgLen=sizeof(XStreamLogin);
       udpSent = 0U;
       while ((XSReturn == UDP_SEND) && (udpSent <= MAX_NUM_OF_UDP_SENDS))      // send up to 3 times dont handle error anymore as we should re-trigger anyway
       {
         XSReturn = Net_Ethernet_Intern_sendUDP(XS_DATA.RemoteIpAddr, XS_From_Port, XS_Dest_Port, Buffer, sizeof(XStreamLogin));
                                                                                // send the Buffer contents as UDP datagram.
         udpSent=++udpSent % UINT8_MAX;
       }
       if (udpSent>>1) totalBadUDPSends =+ (udpSent>>1) % UINT16_MAX;           // accumulate the bad send counter (remove the 1 before summing)
#ifdef UART4_INTERUPT
       memset((void *) Buffer[totalMsgLen],(void *) '\0',sizeof(char));         // Null terminate the Buffer for serial
       UART4_write_text(Buffer);                                                // Send it to serial UART4
#endif
   }
   else if (XS_encoder_Q.loggedin==false)                                       // waiting for authentication
   {
     switch(XSEncoderResp)                                                      // wait for response from authentication
     {
         case XS_OK:                                                            // You got OK so continue as you are now logged in
         XS_encoder_Q.loggedin==true;                                           // now return to the command sequence
         break;

         case XS_FAIL:                                                          // error occurred keep trying after re-try period
         XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_REPLY;                     // Set an alarm to the XS Encoder alarm banner
         XSLoginTime=MSG_REPOLL_TICKS;                                          // immediately retry
         break;

         case XS_INVALID:                                                       // invalid parameters (prompt user for another login name and password)
         XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_USER;                      // Set an alarm to the XS Encoder alarm banner
         XSEncoderResp = XS_AUTH_WAIT;                                          // wait for a new user and password from the HMI
         // XSLoginTime++ % UINT64_MAX;
         break;

         case XS_AUTH:                                                          // invalid password (prompt and ask for another)
         XSEncoderAlarms1.AlarmWdCurrent |= XS_WRONG_PW;                        // Set an alarm to the XS Encoder alarm banner
         XSEncoderResp = XS_AUTH_WAIT;                                          // Wait for a new password from the HMI
         // XSLoginTime++ % UINT64_MAX;
         break;

         default:                                                               // no response yet ?
         XSLoginTime=++XSLoginTime % UINT64_MAX;                                // increment timer up to a max unsigned 64 bit number
     }
   }
   else if (XSEncoderResp == XS_AUTH_WAIT)                                      // Failed to login due to wrong user or password
   {
       if (COMGS_New_Password==true)                                            // HMI response was new user and or password
       {
          XSEncoderResp = XS_AUTH;                                              // try to re-authenticate
          COMGS_New_Password=false;                                             // Reset HMI flag for response
       }
   }
   else if (XSEncoderResp == XS_RTSP_ERROR)                                     // Failed to set-up RTSP
   {
       if (COMGS_New_Network==true)                                             // HMI response was new network settings
       {
          strcpy(XSencoderIPUse,hmiEncoderIP);                                  // Copy the new HMI encoder IP Address
          strcpy(XSencoderIPUse,hmiEncoderGW);                                  // Copy the new HMI encoder gateway IP Address
          XSInitState = 1U;                                                     // Go back to trying to set-up RTSP
          XSEncoderResp = 0U;                                                   // try to re-authenticate
          //COMGS_New_Network=false;                                            // Reset HMI flag for response  its re-calculated using new values for XSencoderIPUse
       }
   }
#endif

#ifdef ALARM_BANNER_USED
   // ================== DATA AND ALARM STATES TO GUI ==========================
   // Check for State Changes of the Motor
   //
#if (defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY))
if (gimStartUpComplete==true)
{
   Process_Motor_State( &gimbalMotor );                                         // Update Motor status
   MotorAlarms.AlarmWdCurrent = (gimbalMotor.FTS + (gimbalMotor.EcalErr*2U) + (gimbalMotor.ESD*4U));    // State Error is bit0 of the alarm word ecal is bit1 and ESD bit2

    // Process the alarm words =================================================
    //
    //
    Process_alarm_ack(&SBGCErrorCode,&AlarmStack,&SBGCDescriptions,&AlarmBanner1);            // Process the alarm word for CMD_ERROR messages
    //
    // Made this above a function (also) can be - check speed is okay before enabling
    //
    if ((boardinforb.FIRMWARE_VER >= 2700U) && (g_boardReady))
    {
       Process_alarm_ack(&SBGCPIDTune2Error,&AlarmStack,&PIDTuErDescriptions,&AlarmBanner1);  // Same as above but for PID2 auto tuning alarms
    }
    Process_alarm_ack(&SBGCSystemError1,&AlarmStack,&SBGCSysErDescrip1,&AlarmBanner1);        // HMI ACK for system alarms word 1
    Process_alarm_ack(&SBGCSystemError2,&AlarmStack,&SBGCSysErDescrip2,&AlarmBanner1);        // HMI ACK for system alarms word 2
    for (countEsdAlarms=1u;countEsdAlarms<=5u;countEsdAlarms++)                   // For all of the reasons for a ESD to occur process the alarm bits
    {
       switch(countEsdAlarms)
       {
          case 1u:
            Process_alarm_ack(&SBGCESDReason[countEsdAlarms],&AlarmStack, &SBGCESDReasonDesc1,&AlarmBanner1);   // HMI ACK for ESD reason words 1-5
            break;
          case 2u:
            Process_alarm_ack(&SBGCESDReason[countEsdAlarms],&AlarmStack, &SBGCESDReasonDesc2,&AlarmBanner1);   // HMI ACK for ESD reason words 1-5
            break;
          case 3u:
            Process_alarm_ack(&SBGCESDReason[countEsdAlarms],&AlarmStack, &SBGCESDReasonDesc3,&AlarmBanner1);   // HMI ACK for ESD reason words 1-5
            break;
          case 4u:
            Process_alarm_ack(&SBGCESDReason[countEsdAlarms],&AlarmStack, &SBGCESDReasonDesc4,&AlarmBanner1);   // HMI ACK for ESD reason words 1-5
            break;
          case 5u:
            Process_alarm_ack(&SBGCESDReason[countEsdAlarms],&AlarmStack, &SBGCESDReasonDesc5,&AlarmBanner1);   // HMI ACK for ESD reason words 1-5
            break;
       }
    }
    //Process_alarm_ack(&JOYCalAlarms);                                           // HMI ACK for Joystick calibration alarms
    Process_alarm_ack(&ScriptRunAlarms1,&AlarmStack,&ScriptRunAlarmDesc1,&AlarmBanner1);      // HMI ACK for Script running alarms
    Process_alarm_ack(&ScriptRunAlarms2,&AlarmStack,&ScriptRunAlarmDesc2,&AlarmBanner1);      // HMI ACK for Script running alarms
    Process_alarm_ack(&MotorAlarms,&AlarmStack,&MotorAlarmDesc,&AlarmBanner1);                // HMI ACK for Motor state alarms
    Process_alarm_ack(&ExtIMUAlarms1,&AlarmStack,&ExtIMUAlarmDesc1,&AlarmBanner1);            // HMI ACK for External IMU status
    Process_alarm_ack(&ExtIMUAlarms2,&AlarmStack,&ExtIMUAlarmDesc2,&AlarmBanner1);            // HMI ACK for External IMU Status
}
#endif /* gimbal alrms */

#if (CAMERA_TYPE == XS_CAM)
    // process encoder alarms
    Process_alarm_ack(&XSEncoderAlarms1,&XSAlarmStack,&XSEncoderAlarmsDesc1,&XSAlarmBanner);
#endif

#endif                                                                          // ================ eNd Alarm Banner on GUI =================================
// ===================== LIDDAR ================================================

#ifdef LWNX_LIDDAR_USED                                                         // Liddar defined as UART6
   LwNxPeriodCnt = ++LwNxPeriodCnt % UINT32_MAX;
   if ((LwNxPeriodCnt % LWNX_PERIOD_TEMP) == 0u)                                // period time counted for temp update
   {
      hmiLwNx.getVolts = 1U;                                                    // latch to get an update
   }
   if ((LwNxPeriodCnt % LWNX_PERIOD_ALARM) == 0u)                               // period time counted for alarm update
   {
      hmiLwNx.getAlarm = 1U;                                                    // latch to get an update
   }
   
   switch (liddarMsgState)
   {
       case 0u:
       lwnxInitResponsePacket(&LwNxResponse);                                   // initialise the response structures
       lwnxSendPacketBytes(LWNX_CMD_PRODUCT_NM, false, NULL, 0U, LIDDAR_PORT);  // send out request 0 for the product name
       UART6.State=0U;
       liddarMsgState=1U;

       case 1u:
       if ((UART6.State==1U) && (LwNxResponse.dataVal[3U]==LWNX_CMD_PRODUCT_NM))// message collection in the interrupt is complete
       {
          strcpy(&LwNxHardWare.modelName,&LwNxResponse.dataVal + 4U);           // Copy the value
          liddarMsgState=2U;
       }
       else if ((UART6.State==2U) || (UART6.State==3U))                         // message had wrong size or crc
       {
          liddarMsgState=0U;                                                    // reset the send state
       }
       break;
   
      case 2u:
      lwnxInitResponsePacket(&LwNxResponse);                                    // initialise the response structures
      lwnxSendPacketBytes(LWNX_CMD_HW_VER, false, NULL, 0U, LIDDAR_PORT);       // send out request 1 for the hardware version
      UART6.State=0U;
      liddarMsgState=3U;
      
      case 3u:
      if ((UART6.State==1U) && (LwNxResponse.dataVal[3U]==LWNX_CMD_HW_VER))     // message collection in the interrupt is complete
      {
         //LwNxHardWare.hardwareVersion = atol(LwNxResponse.dataVal);           // Copy the value
         memcpy(&LwNxHardWare.hardwareVersion, LwNxResponse.dataVal + 4U, sizeof(uint32_t));
         liddarMsgState=4U;
      }
      else if ((UART6.State==2U) || (UART6.State==3U))                          // message had wrong size or crc
      {
         liddarMsgState=2U;                                                     // reset the send state
      }
      break;
      
      case 4u:
      lwnxInitResponsePacket(&LwNxResponse);                                    // initialise the response structures
      lwnxSendPacketBytes(LWNX_CMD_FW_VER, false, NULL, 0U, LIDDAR_PORT);       // send out request 2 for the firmware version
      UART6.State=0U;
      liddarMsgState=5U;

      case 5u:
      if ((UART6.State==1U) && (LwNxResponse.dataVal[3U]==LWNX_CMD_FW_VER))     // message collection in the interrupt is complete
      {
         //LwNxHardWare.firmwareVersion = atol(LwNxResponse.dataVal);           // Copy the value
         memcpy(&LwNxHardWare.firmwareVersion, LwNxResponse.dataVal + 4U, sizeof(uint32_t));
         lwnxConvertFirmwareVersionToStr(LwNxHardWare.firmwareVersion, firmwareVersionStr);
         liddarMsgState=6U;
      }
      else if ((UART6.State==2U) || (UART6.State==3U))                          // message had wrong size or crc
      {
         liddarMsgState=4U;                                                     // reset the send state
      }
      break;
      
      case 6u:
      lwnxInitResponsePacket(&LwNxResponse);                                    // initialise the response structures
      lwnxSendPacketBytes(LWNX_CMD_SERIAL_NO, false, NULL, 0U, LIDDAR_PORT);    // send out request 3 for the serial number
      UART6.State=0U;
      liddarMsgState=6U;
      
      case 7u:
      if ((UART6.State==1U) && (LwNxResponse.dataVal[3U]==LWNX_CMD_SERIAL_NO))  // message collection in the interrupt is complete
      {
         strcpy(&LwNxHardWare.serialNumber,&LwNxResponse.dataVal + 4U);         // Copy the value
         liddarMsgState=18U;                                                    // additional check for motor running performed before start-up
      }
      else if ((UART6.State==2U) || (UART6.State==3U))                          // message had wrong size or crc
      {
         liddarMsgState=6U;                                                     // reset the send state
      }
      break;

      case 18u:                                                                 // ---- check if motors are running
      lwnxInitResponsePacket(&LwNxResponse);                                    // initialise the response structures
      lwnxSendPacketBytes(LWNX_CMD_MOTOR_STATE, false, NULL, 0U, LIDDAR_PORT);    // send out request 3 for the serial number
      UART6.State=0U;
      LwNxLaserCnt=0U;
      liddarMsgState=19U;

      case 19u:
      if ((UART6.State==1U) && (LwNxResponse.dataVal[3U]==LWNX_CMD_MOTOR_STATE))  // message collection in the interrupt is complete
      {
         memcpy(&LwNxMotorState, LwNxResponse.dataVal + 4U, sizeof(uint8_t));
         if (LWNX_MOTOR_RUN == LwNxMotorState)
         {
            liddarMsgState=8U;                                                  // we are in a running state so go to the step which starts the stream
         }
         else
         {
            LwNxLaserCnt = ++LwNxLaserCnt % UINT16_MAX;
            if (LwNxLaserCnt > LWNX_RETRY_TICKS)
              liddarMsgState=18U;                                               // go back to request another motor state after a timeout
         }
      }
      else if ((UART6.State==2U) || (UART6.State==3U))                          // message had wrong size or crc
      {
         liddarMsgState=18U;                                                     // reset the send state
      }
      else
      {
         LwNxLaserCnt = ++LwNxLaserCnt % UINT16_MAX;
         if (LwNxLaserCnt > LWNX_RETRY_TICKS)
           liddarMsgState=18U;                                                  // go back to request another motor state after a timeout
      }
      break;
      
      case 8u:                                                                  // ----- start streaming --------------
      lwnxInitResponsePacket(&LwNxResponse);                                    // initialise the response structures
      LwNxWriteI8=LWNX_STREAM_20010;
      lwnxSendPacketBytes(LWNX_CMD_STREAM_RATE, true, &LwNxWriteI8, sizeof(uint8_t), LIDDAR_PORT);  // send out request to set stream rate
      UART6.State=0U;
      liddarMsgState=9U;
      break;
      
      case 9u:
      lwnxInitResponsePacket(&LwNxResponse);                                    // initialise the response structures
      LwNxWriteI32[0U]=LWNX_WRITE_STREAM_ON;
      lwnxSendPacketBytes(LWNX_CMD_STREAM, true, &LwNxWriteI32, sizeof(uint32_t), LIDDAR_PORT);       // send out request to begin streaming
      UART6.State=0U;
      liddarMsgState=10U;                                                       // now wait for stream responses
      break;
      
      case 10u:
      if ((UART6.State==1U) && (LwNxResponse.dataVal[3U]==LWNX_CMD_DIST_OBJ))   // message collection in the interrupt is complete and its a distance object being returned
      {
         switch(LwNxCollection)
         {
            case 0u:                                                             // first collection of stream points
            LwNxOutput.alarmState = LwNxResponse.dataVal[4U];
            LwNxOutput.pointsPerSecond = (LwNxResponse.dataVal[6U] << 8U) | LwNxResponse.dataVal[5U];
            LwNxOutput.forwardOffset = (LwNxResponse.dataVal[8U] << 8U) | LwNxResponse.dataVal[7U];
            LwNxOutput.motorVoltage = (LwNxResponse.dataVal[10U] << 8U) | LwNxResponse.dataVal[9U];
            LwNxOutput.revolutionIndex = LwNxResponse.dataVal[11U];
            LwNxOutput.pointTotal = (LwNxResponse.dataVal[13U] << 8U) | LwNxResponse.dataVal[12U];
            LwNxOutput.pointCount = (LwNxResponse.dataVal[15U] << 8U) | LwNxResponse.dataVal[14U];
            LwNxOutput.pointStartIndex = (LwNxResponse.dataVal[17U] << 8U) | LwNxResponse.dataVal[16U];
            memcpy(&LwNxOutput.pointDistAngle[LwNxCollection].pointDistance, LwNxResponse.dataVal + 18U, LwNxOutput.pointCount * 2U);
            for (LwNxPoint= 0; LwNxPoint < LwNxOutput.pointCount; ++LwNxPoint)
            {
               LwNxOutput.pointDistAngle[LwNxCollection].angle[LwNxPoint] = ((float32_t)(LwNxOutput.pointStartIndex + LwNxPoint) / (float32_t )LwNxOutput.pointTotal) * 360.0f;
            }
            UART6.State=0U;                                                     // reset collection state
            LwNxCollection = ++LwNxCollection % UINT8_MAX;
            break;
            
            case 1u:
            LwNxOutput.alarmState = LwNxResponse.dataVal[4U];
            LwNxOutput.pointsPerSecond = (LwNxResponse.dataVal[6U] << 8U) | LwNxResponse.dataVal[5U];
            LwNxOutput.forwardOffset = (LwNxResponse.dataVal[8U] << 8U) | LwNxResponse.dataVal[7U];
            LwNxOutput.motorVoltage = (LwNxResponse.dataVal[10U] << 8U) | LwNxResponse.dataVal[9U];
            LwNxOutput.revolutionIndex = LwNxResponse.dataVal[11U];
            LwNxOutput.pointTotal = (LwNxResponse.dataVal[13U] << 8U) | LwNxResponse.dataVal[12U];
            LwNxOutput.pointCount = (LwNxResponse.dataVal[15U] << 8U) | LwNxResponse.dataVal[14U];
            LwNxOutput.pointStartIndex = (LwNxResponse.dataVal[17U] << 8U) | LwNxResponse.dataVal[16U];
            memcpy(&LwNxOutput.pointDistAngle[LwNxCollection].pointDistance, LwNxResponse.dataVal + 18U, LwNxOutput.pointCount * 2U);
            for (LwNxPoint= 0; LwNxPoint < LwNxOutput.pointCount; ++LwNxPoint)
            {
               LwNxOutput.pointDistAngle[LwNxCollection].angle[LwNxPoint] = ((float32_t)(LwNxOutput.pointStartIndex + LwNxPoint) / (float32_t )LwNxOutput.pointTotal) * 360.0f;
            }
            UART6.State=0U;                                                     // reset collection state
            LwNxCollection = ++LwNxCollection % UINT8_MAX;
            break;
            
            case 2u:
            LwNxOutput.alarmState = LwNxResponse.dataVal[4U];
            LwNxOutput.pointsPerSecond = (LwNxResponse.dataVal[6U] << 8U) | LwNxResponse.dataVal[5U];
            LwNxOutput.forwardOffset = (LwNxResponse.dataVal[8U] << 8U) | LwNxResponse.dataVal[7U];
            LwNxOutput.motorVoltage = (LwNxResponse.dataVal[10U] << 8U) | LwNxResponse.dataVal[9U];
            LwNxOutput.revolutionIndex = LwNxResponse.dataVal[11U];
            LwNxOutput.pointTotal = (LwNxResponse.dataVal[13U] << 8U) | LwNxResponse.dataVal[12U];
            LwNxOutput.pointCount = (LwNxResponse.dataVal[15U] << 8U) | LwNxResponse.dataVal[14U];
            LwNxOutput.pointStartIndex = (LwNxResponse.dataVal[17U] << 8U) | LwNxResponse.dataVal[16U];
            memcpy(&LwNxOutput.pointDistAngle[LwNxCollection].pointDistance, LwNxResponse.dataVal + 18U, LwNxOutput.pointCount * 2U);
            for (LwNxPoint= 0; LwNxPoint < LwNxOutput.pointCount; ++LwNxPoint)
            {
               LwNxOutput.pointDistAngle[LwNxCollection].angle[LwNxPoint] = ((float32_t)(LwNxOutput.pointStartIndex + LwNxPoint) / (float32_t )LwNxOutput.pointTotal) * 360.0f;
            }
            UART6.State=0U;                                                     // reset collection state
            LwNxCollection = 0U;                                                // start at array 0 (make this as many as you want to do)
            liddarMsgState=11U;                                                 // look at requests first
            break;
            
            default:
            UART6.State=0U;                                                     // reset collection state
            LwNxCollection = 0U;                                                // start at array 0 (make this as many as you want to do)
            break;
         }
      }
      else                                                                      // waiting for a reply
      {
         LwNxTimerTicks = ++LwNxTimerTicks % UINT32_MAX;
         if (LwNxTimerTicks > LWNX_MAX_WAIT)                                    // ticks are greaterthan the maximum wait for a stream point response
         {
            lwnxInitResponsePacket(&LwNxResponse);                              // initialise the response structures
            lwnxSendPacketBytes(LWNX_CMD_RESET, false, NULL, 0U, LIDDAR_PORT);  // send out request to reset
            liddarMsgState=8U;                                                  // go back to set-up streaming
         }
      }
      
      case 11u:                                                                 // Check for actions
      if ((hmiLwNx.laserStateOn) || (hmiLwNx.laserStateOff))                    // request to change the laser state from GUI
      {
         LwNxLaserCnt=0U;                                                       // reset timeout
         if (hmiLwNx.laserStateOn)
            LwNxWriteI8=LWNX_LASER_ON;
         else if (hmiLwNx.laserStateOff)
            LwNxWriteI8=LWNX_LASER_OFF;
         lwnxSendPacketBytes(LWNX_CMD_LASER_STATE, true, &LwNxWriteI8, sizeof(uint8_t), LIDDAR_PORT);
         liddarMsgState=12U;                                                    // wait for state reply
      }
      else if (hmiLwNx.getTemp)                                                 // request to get the temperature from GUI
      {
         lwnxSendPacketBytes(LWNX_CMD_TEMP, false, NULL, 0U, LIDDAR_PORT);
         LwNxLaserCnt=0U;                                                       // reset timeout
         liddarMsgState=14U;                                                    // wait for state reply
      }
      else if (hmiLwNx.getVolts)                                                // timed request to get the voltage
      {
         lwnxSendPacketBytes(LWNX_CMD_VOLT, false, NULL, 0U, LIDDAR_PORT);
         LwNxLaserCnt=0U;                                                       // reset timeout
         liddarMsgState=15U;                                                    // wait for state reply
      }
      else if (hmiLwNx.getAlarm)                                                // timed request to get the alarms
      {
         lwnxSendPacketBytes(LWNX_CMD_ALARM_STATE, false, NULL, 0U, LIDDAR_PORT);
         LwNxLaserCnt=0U;                                                       // reset timeout
         liddarMsgState=16U;                                                    // wait for state reply
      }
      else if (hmiLwNx.setAlarm)                                                // timed request to set an alarm
      {
         switch (hmiAlarmNo)                                                    // Alarm number chosen on the GUI
         {
            case 1u:
            LwNxAlarmConfig.Enabled=1U;                                         // ensure enable is set
            lwnxSendPacketBytes(LWNX_CMD_ALARM1, true,(uint8_t *) &LwNxAlarmConfig, sizeof(LwNxAlarmConfig), LIDDAR_PORT);
            LwNxLaserCnt=0U;                                                    // reset timeout
            liddarMsgState=17U;                                                 // read back for state reply
            break;
            
            case 2u:
            LwNxAlarmConfig.Enabled=1U;                                         // ensure enable is set
            lwnxSendPacketBytes(LWNX_CMD_ALARM2, true,(uint8_t *) &LwNxAlarmConfig, sizeof(LwNxAlarmConfig), LIDDAR_PORT);
            LwNxLaserCnt=0U;                                                    // reset timeout
            liddarMsgState=17U;                                                 // read back for state reply
            break;
            
            case 3u:
            LwNxAlarmConfig.Enabled=1U;                                         // ensure enable is set
            lwnxSendPacketBytes(LWNX_CMD_ALARM3, true,(uint8_t *) &LwNxAlarmConfig, sizeof(LwNxAlarmConfig), LIDDAR_PORT);
            LwNxLaserCnt=0U;                                                    // reset timeout
            liddarMsgState=17U;                                                 // read back for state reply
            break;
            
            case 4u:
            LwNxAlarmConfig.Enabled=1U;                                         // ensure enable is set
            lwnxSendPacketBytes(LWNX_CMD_ALARM4, true,(uint8_t *) &LwNxAlarmConfig, sizeof(LwNxAlarmConfig), LIDDAR_PORT);
            LwNxLaserCnt=0U;                                                    // reset timeout
            liddarMsgState=17U;                                                 // read back for state reply
            break;
            
            case 5u:
            LwNxAlarmConfig.Enabled=1U;                                         // ensure enable is set
            lwnxSendPacketBytes(LWNX_CMD_ALARM5, true,(uint8_t *) &LwNxAlarmConfig, sizeof(LwNxAlarmConfig), LIDDAR_PORT);
            LwNxLaserCnt=0U;                                                    // reset timeout
            liddarMsgState=17U;                                                 // read back for state reply
            break;
            
            case 6u:
            LwNxAlarmConfig.Enabled=1U;                                         // ensure enable is set
            lwnxSendPacketBytes(LWNX_CMD_ALARM6, true,(uint8_t *) &LwNxAlarmConfig, sizeof(LwNxAlarmConfig), LIDDAR_PORT);
            LwNxLaserCnt=0U;                                                    // reset timeout
            liddarMsgState=17U;                                                 // read back for state reply
            break;
            
            case 7u:
            LwNxAlarmConfig.Enabled=1U;                                         // ensure enable is set
            lwnxSendPacketBytes(LWNX_CMD_ALARM7, true,(uint8_t *) &LwNxAlarmConfig, sizeof(LwNxAlarmConfig), LIDDAR_PORT);
            LwNxLaserCnt=0U;                                                    // reset timeout
            liddarMsgState=17U;                                                 // read back for state reply
            break;
         }
      }
      else
      {
         liddarMsgState=10U;                                                    // go back reading streaming points
      }
      break;
      
      case 12u:                                                                 // requested a change of state of laser
      lwnxSendPacketBytes(LWNX_CMD_LASER_STATE, false, NULL, 0U, LIDDAR_PORT);  // send a state read
      liddarMsgState=13U;                                                       // wait for the laser state message
      break;
      
      case 13u:
      if ((UART6.State==1U) && (LwNxResponse.dataVal[3U]==LWNX_CMD_LASER_STATE))     // message collection in the interrupt is complete
      {
         memcpy(&LwNxLaserState, LwNxResponse.dataVal + 4U, sizeof(uint8_t));
         if (LwNxLaserState == LwNxWriteI8)                                     // state matches request
         {
            if (LwNxWriteI8==LWNX_LASER_ON)
            {
               hmiLwNx.laserStateOn=0U;
               liddarMsgState=10U;                                              // go back to stream point collection
            }
            else if (LwNxWriteI8==LWNX_LASER_OFF)
            {
               hmiLwNx.laserStateOff=0U;
               liddarMsgState=50U;                                              // wait for laser re-start
            }
         }
         else
         {
            LwNxLaserCnt = ++LwNxLaserCnt % UINT16_MAX;
            if (LwNxLaserCnt > LWNX_RETRY_TICKS)
              liddarMsgState=11U;                                               // go back to request laser state change
            else
              liddarMsgState=12U;                                               // go back to request laser read request
         }
         
      }
      else if ((UART6.State==2U) || (UART6.State==3U))                          // message had wrong size or crc
      {
         liddarMsgState=12U;                                                    // request another read
      }
      break;
      
      case 14u:
      if ((UART6.State==1U) && (LwNxResponse.dataVal[3U]==LWNX_CMD_TEMP))       // message collection in the interrupt is complete
      {
         memcpy(&LwNxTemp, LwNxResponse.dataVal + 4U, sizeof(uint32_t));        // get temperature
         hmiLwNxTemp=LWNX_TEMP(LwNxTemp);                                       // convert temperature for display
         hmiLwNx.getTemp=0U;                                                    // reset HMI request for temperature update
         liddarMsgState=10U;                                                    // go back to stream point collection
      }
      else if ((UART6.State==2U) || (UART6.State==3U))                          // message had wrong size or crc
      {
         liddarMsgState=11U;                                                    // request another read
      }
      else
      {
         LwNxLaserCnt = ++LwNxLaserCnt % UINT16_MAX;
         if (LwNxLaserCnt > LWNX_RETRY_TICKS)
           liddarMsgState=11U;                                                  // go back to request another temperature update
      }
      break;

      case 15u:
      if ((UART6.State==1U) && (LwNxResponse.dataVal[3U]==LWNX_CMD_VOLT))       // message collection in the interrupt is complete
      {
         memcpy(&LwNxVoltRaw, LwNxResponse.dataVal + 4U, sizeof(uint32_t));     // get raw counts for volts
         hmiLwNxVolt=LWNX_VOLTS(LwNxVoltRaw);                                   // convert raw to volts for power management
         hmiLwNx.getVolts=0U;                                                   // reset request for voltage update
         liddarMsgState=10U;                                                    // go back to stream point collection
      }
      else if ((UART6.State==2U) || (UART6.State==3U))                          // message had wrong size or crc
      {
         liddarMsgState=11U;                                                    // request another read
      }
      else
      {
         LwNxLaserCnt = ++LwNxLaserCnt % UINT16_MAX;
         if (LwNxLaserCnt > LWNX_RETRY_TICKS)
           liddarMsgState=11U;                                                  // go back to request another temperature update
      }
      break;

      case 16u:
      if ((UART6.State==1U) && (LwNxResponse.dataVal[3U]==LWNX_CMD_ALARM_STATE))// message collection in the interrupt is complete
      {
         memcpy(&LwNxAlarmWord, LwNxResponse.dataVal + 4U, sizeof(uint8_t));    // get the alarm word
         hmiLwNx.getAlarm=0U;                                                   // reset request for alarm update
         liddarMsgState=10U;                                                    // go back to stream point collection
      }
      else if ((UART6.State==2U) || (UART6.State==3U))                          // message had wrong size or crc
      {
         liddarMsgState=11U;                                                    // request another read
      }
      else
      {
         LwNxLaserCnt = ++LwNxLaserCnt % UINT16_MAX;
         if (LwNxLaserCnt > LWNX_RETRY_TICKS)
           liddarMsgState=11U;                                                  // go back to request another temperature update
      }
      break;

      case 17u:
      if (UART6.State==1U)                                                      // message collection in the interrupt is complete
      {
         if ((((((((LwNxResponse.dataVal[3U]==LWNX_CMD_ALARM1) && (hmiAlarmNo == 1U)) || ((LwNxResponse.dataVal[3U]==LWNX_CMD_ALARM2) && (hmiAlarmNo == 2U)))  \
         || ((LwNxResponse.dataVal[3U]==LWNX_CMD_ALARM3) && (hmiAlarmNo == 3U))) || ((LwNxResponse.dataVal[3U]==LWNX_CMD_ALARM4) && (hmiAlarmNo == 4U))) \
         || ((LwNxResponse.dataVal[3U]==LWNX_CMD_ALARM5) && (hmiAlarmNo == 5U))) || ((LwNxResponse.dataVal[3U]==LWNX_CMD_ALARM6) && (hmiAlarmNo == 6U))) \
         || ((LwNxResponse.dataVal[3U]==LWNX_CMD_ALARM7) && (hmiAlarmNo == 7U)))
         {
            memcpy(&LwNxAlarmConfig, LwNxResponse.dataVal + 4U, sizeof(LwNxAlarmConfig));    // get the alarm word
            hmiLwNx.setAlarm=0U;                                                // reset request for alarm update
            liddarMsgState=10U;                                                 // go back to stream point collection
         }
      }
      else if ((UART6.State==2U) || (UART6.State==3U))                          // message had wrong size or crc
      {
         liddarMsgState=11U;                                                    // request another another alarm update
      }
      else
      {
         LwNxLaserCnt = ++LwNxLaserCnt % UINT16_MAX;
         if (LwNxLaserCnt > LWNX_RETRY_TICKS)
           liddarMsgState=11U;                                                  // go back to request another alarm update
      }
      break;
      
      case 50u:                                                                 // laser is off wait for re-start
      if (hmiLwNx.laserStateOn)
         liddarMsgState=11U;                                                    // go back to re-start the laser
      break;
      
      default:
      liddarMsgState=0U;
      break;
   }
#endif                                                                          // ================= eNd Liddar LWNX used ========================

#if defined(GPS_POS_SPRAYER)                                                    // ================= sTart GPS Position Sprayer used =============

#ifdef GPS_INCLUDED                                                             /* gps type 1 being used else read interrupt byte stream for uart3 */
   ReadGPSData( &g_posDataGPS );
#endif

   sprayTimeout=0u;                                                             // reset the timeout to wait to be in an area
   // sprayArea=0u;                                                             do not reset start from last known

   if (posSprayDrive.mode_rem=true)                                             /* pid controller is put into remote */
   {
      while((doesPosFitSquare(&square[sprayArea], &g_posDataGPS, arrOfSpraySpt, &posSprayDrive) <= 0u)&&(sprayTimeout <= MAX_TIME_TO_FIND_AREA))   // consider if to while shift to prevent hanging on this !!!! kept arrOfSpraySpt although now overriden from recipe (in case useful in future)
      {                                                                         // get the setpoint for the square that matches our current position
        sprayArea=++sprayArea % (NO_SPRAY_OF_AREAS+1u);                         // dont exceed the number of areas
        sprayTimeout=++sprayTimeout % UINT16_MAX;                               // count the timeout ticks and roll over the counter if it goes to max
      }
      if (sprayTimeout<=MAX_TIME_TO_FIND_AREA)                                  /* found that our position matches one of the squares in the defined map of fields */
      {
         if (fileDatInput->fileread.spraymix->field[sprayArea]->geoCoord->coordChkDisAbl==false) /* product has an updated spray setpoint */
         {
#if defined(POS_SPEED_COMPENSATE)
             posSprayDrive.rem_setpoint = ((g_posDataGPS.SOG * POS_SPEED_FACTOR) * fileDatInput->fileread.spraymix->field[sprayArea]->arrOfSpraySpt);  /* multiply setpoint by speed of craft (might need factor) */
#else
             posSprayDrive.rem_setpoint = fileDatInput->fileread.spraymix->field[sprayArea]->arrOfSpraySpt; /* set the spray speed for the current quadrant from the product recipe data */
                                                                                /* else take what was defaulted (product independant) from  arrOfSpraySpt */
#endif
             posSprayDrive.rem_setpoint = max(posSprayDrive.rem_setpoint,0.0f); /* clamp the set-point */
             posSprayDrive.rem_setpoint = min(posSprayDrive.rem_setpoint,MAX_REM_SPT);
         }
      }
   }

   if (g_posDataGPS.SOG >= SPRAYER_IS_MOVING)                                   /* check if the sprayer vehicle is moving to interlock the spray */
   {
      switch (SprayerVehicle.State)                                             /* look at the state of the vehcile */
      {
         case SPRAYER_STOPPED:                                                  /* stopped */
         SprayerVehicle.InitTm = -1;
         calculateTime2Now(&SprayerVehicle.InitTm,&SprayerVehicle.Tm);          /* initialise timer */
         SprayerVehicle.State = SPRAYER_MOVING;
         break;
         
         case SPRAYER_MOVING:                                                   /* startup moving */
         calculateTime2Now(&SprayerVehicle.InitTm,&SprayerVehicle.Tm);
         if (SprayerVehicle.Tm > SPRAYER_START_DELAY)                           /* been running long enough */
         {
            SprayerVehicle.State = SPRAYER_RUNNING_LONG_ENOUGH;
         }
         break;
         
         case SPRAYER_RUNNING_LONG_ENOUGH:                                      /* its now moving */
         break;
         
         default:                                                               /* unknown state (hack) */
         SprayerVehicle.State = SPRAYER_STOPPED;                                /* state screwed up so set it back to stop state (catch error) */
         break;
      }
   }
   else if (g_posDataGPS.SOG <= (SPRAYER_IS_MOVING - SPRAYER_SLOW_DOWN_DELTA))
   {
      SprayerVehicle.State = SPRAYER_STOPPED;
   }
   
   if (((sprayTimeout<=MAX_TIME_TO_FIND_AREA) || (posSprayDrive.mode_rem=false)) && (SprayerVehicle.State == SPRAYER_RUNNING_LONG_ENOUGH)) // we found an area or was in local joystick control and the vehcile has been moving beyond the delay
   {
     if ((posSprayDrive.mode_hld==true) && (posSprayerState==false))            // was previously out of area
     {
        posSprayDrive.mode_hld=false;                                           // turn off the hold
        posSprayerState=true;                                                   // set state to operational
     }
#if defined(PULSE_TACHO)
     posSprayDrive.feedback = g_TachoObj1.rot_speed;                            // make feedback the pulse tachometer
#else
     posSprayDrive.feedback = feedback;                                         // make feedback what you measured (dummy enter other options here)
#endif
     while(!computePID(&posSprayDrive));                                        // compute desired output from pid algorythm --- consider if to while shift to prevent hanging on this !!!!
   }
   else if (sprayTimeout>=MAX_TIME_TO_FIND_AREA)                                // area not found (outside grid)
   {
      posSprayDrive.mode_hld=true;                                              // disble PID loop
      posSprayDrive.fout=0.0f;                                                  // turn off sprayer
      posSprayerState=false;                                                    // set state to false
   }
   if(posSprayDrive.fout >= 0.0f)                                               // computed pid output is pwm
     PWMSetDuty( posSprayDrive.fout, pwm_period, channel );                     // set the pwm to the servo motor

#endif                                                                          // ================= eNd GPS Position Sprayer used ===============

#if defined(JRT_LIDDAR_USED)
  if (JRTSerial.State=USB_PACKET_READ_DATA)                                     /* a new usb polled data message was received in the 100ms interrupt */
  {
#if defined(PNEU_HT_CONTROL)
     if (sprayTimeout<=MAX_TIME_TO_FIND_AREA)                                   /* we are currently in a defined square of the field */
        sprayHtSpt=((uint16_t)((fileDatInput->fileread.spraymix->field[sprayArea]->geoCoord->GeoidalHt)*100.0f)); /* convert meters spray height to centimeters setpoint used by the distance sensor controls */
     else
        sprayHtSpt=_defSprayConeHt;                                             /* default the height parameter if we are not in the co-ordinates of a defined square in the fields */
     keepSprayHeight( &distSensIT03Meas );                                      /* control the pneumatic height for sprayer cone */
#endif
     JRTSerial.State=USB_PACKET_DATA_COMPLETE;                                  /* tell the usb driver we read the last packet so bother to colect another */
  }
  else
  {
     configDistanceSensor( &distSensIT03Conf, &JRTSerial );                     /* start communication with the distance sensor on usb for sprayer height control */
  }
#endif /* end jrt distance sensor */

#if defined(USE_MAV_PULSE_OBJECT)
   relayPulseAction( &pulseDigitalOutputs );
#endif  

#if defined(CRAFT_MTR_PIDS)
  for (craftMtr=1u;craftMtr<=CRAFT_NUM_OF_MTRS;craftMtr++)
  {
#if defined(USE_SPEKTRUM)
     if (SpekBusObject[craftMtr-1u].RPM == 0xFFU)
     {
        craftSpeedReading[craftMtr-1u] = SpekBusObject[craftMtr-1u].RPM;         /* read the RPM straight off spektrum bus (need to implement reader) */
        craftSpeedPID[craftMtr-1u].setpoint = ((int32_t) Scale_Float_Value(0, SPEK_SCALED_RPM, 0, SPEK_RAW_RPM, reqCraftSpt[craftMtr-1u]));      /* if we scale the measurement we loose accuracy so we scale up the setpoint from 0-10 to scale_raw_value(0,10.0f,0,655340, and the PID works on raw integers */
     }
#elif defined(ENCODER_HELPER)
     craftItm=(uint16_t) pow(2.0f,(float64_t) (craftMtr-1u));                   /* for each of the hardwired encoder input numbers */
     collectGray( &craftEncoderIn[craftMtr-1u], craftItm );                     /* read the encoder (should be in timed interrupt ??) */
     craftSpeedReading[craftMtr-1u] = graytoBinary( &craftEncoderIn[craftMtr-1u] ); /* read the gray code input if we are using this */
     craftSpeedPID[craftMtr-1u].setpoint = ((int32_t) Scale_Float_Value(0, ENCO_MAX_RPM, 0, ENCO_MAX_RAW, reqCraftSpt[craftMtr-1u]));
#endif  /* end encoder or spektrum */
     craftSpeedPID[craftMtr-1u].feedback = ((int32_t) craftSpeedReading[craftMtr-1u]);  // read the speed encoder input
     while(!computePID(&craftSpeedPID[craftMtr-1u]));                           // do the PID
     if(craftSpeedPID[craftMtr-1u].fout >= 0.0f)                                // computed pid output is pwm
#if defined(USE_SPEKTRUM)
     SpekBusObject[craftMtr-1u].throttle = ((uint8_t) Scale_Float_Value(0, 100.0f, 0, SPEK_THROT_MAX, craftSpeedPID[craftMtr-1u].fout));
#else
     PWMSetDuty( craftSpeedPID[craftMtr-1u].fout, aero_pwm_period, aero_channel[craftMtr-1u] ); // set the pwm to the servo motor
#endif /* end spektrum or direct for encoder */
  }                                                                             // ================= eNd craft motor controls ===============
#endif /* end craft motor */

} // END OF FOREVER LOOPING

}