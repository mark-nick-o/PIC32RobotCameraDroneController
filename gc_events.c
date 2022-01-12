//    gc_events.h : General functions 
//                  Ethernet handlers etc
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
//
/* out for now #include "types.h"      */
#include <stdlib.h>
#include "definitions.h"                                                        // global defines
#include "gc_events.h"
#include "Struts.h"                                                             // common struts for UART
#include "ports.h"                                                              // IP ports
#include "Config.h"                                                             // config
//#include "Transceiver_events.h"
#include "cpu_endian_defs.h"
#include <stdint.h>
/* out for now #include<built_in.h>  */
//#include "__NetEthInternal.h"                                                   // tcpip stack already in gc_events.h
//#ifndef Net_Ethernet_Intern_arpCacheStruct
//#include "__NetEthInternal.h"
//#endif
#include "io.h"
/*#include "mmc_file_handler.h"*/

#if defined(JRT_LIDDAR_USED)
#include "jrt_lidar.h"
#endif

#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
#include "SBGC.h"                                                               // SBGC general
#include "SBGC_rc.h"                                                            // SBGC rc commands
#include "SBGC_COMMAND.h"                                                       // SBGC commands
#include "SBGC_PARSER.h"                                                        // SBGC parser (mostly for aduino) some general lengths etc
#include "SBGC_cmd_helpers.h"                                                   // SBGC all message conatiners
#include "SBGC_adj_vars.h"                                                      // SBGC variables
#endif

#if (defined(RUN_CAM_USED) || (CAMERA_TYPE == XS_CAM))
#include "camera.h"                                                             // camera helpers
#endif

#include "p32mx795f512l.h"                                                      // mcu library
#include "pic32mx_eth.h"                                                        // eth phy library
//#include "lwNx.h"                                                               // liddar from lightware

#ifdef GEF_EGD_PLC
#include "EGD.h"                                                                // Library for communicating UDP to GEC Fanuc PLC
#endif

#ifdef SMPTE_USED
#include "smpte.h"                                                              // library for communciating with UDP to smpte (orangepi)
#endif

#ifdef MODBUS_TCP
//#include "mbproto.h"                                                          // Library for communicating Modbus over TCP
#define MB_TCP_PORT 502U                                                        //Modbus/TCP port number
#define MB_TCP_SECURE_PORT 802U                                                 //Secure Modbus/TCP port number
#define MB_MAX_ADU_SIZE 260U                                                    //Maximum size of Modbus/TCP ADU
#endif

#if defined(MICROSCAN_USED)
//#include "microscan.h"                                                        // Library for communicating with microscan camera
#define MIRSC_TELNET_PORT1 49211U
#define MIRSC_TELNET_PORT2 49212U
#endif

#ifdef ART_NET_USED                                                             //  DMX512 over ArtNet4
#include "art_net4.h"
#endif

#ifdef ASN_E1_USED                                                              //  DMX512 over ASN
#include "asn_e1_31.h"                                                          //  Camera Cranes, Control Protocols, Electrical Power, Event Safety, Floors, Fog and Smoke, Followspot Position, Photometrics, and Rigging
#endif

#if defined(TRAMP_PROTOCOL)
#include "tramp_protocol.h"
#endif

/* #if defined(USE_MAVLINK)                                                        // we are using MAVLINK - defined in defintions
#include "mavlink_crc.h"                                                        // cyclic redundancy for MAVLINK
#include "mavlink_conversions.h"                                                // useful math conversions
#if (MAV_VER == 2u)                                                             // ==== MAVLINK ver 2.0 =============
#include "mavlink2.h"                                                           // general MAV system defines
#include "mavlink2_msg_types.h"                                                 // structures for message types
#define MAVLINK_STX2 253u
#include "mavlink2_common.h"                                                    // common enum and structs for command set
#include "mavlink2_protocol.h"                                                  // protocol driver
#include "mavlink2_sha256.h"                                                    // sha256 implementation
#include "mavlink2_get_info.h"                                                  // get info request if 24bit set
#include "mavlink2_helpers.h"                                                   // general helper functions
#include "msg2_attitude_quartenion.h"                                           // position feed back pitch roll and yaw
#elif (MAV_VER == 1u)
#include "mavlink1.h"                                                           // standard defintion of system
#include "mavlink1_msg_types.h"                                                 // ==== MAVLINK ver 1.0 =============
#include "mavlink1_common.h"
#include "mavlink1_protocol.h"                                                  // protocol driver
#include "mavlink1_helpers.h"                                                   // helper function
#define MAVLINK_STX1 254u
#include "msg1_attitude_quartenion.h"                                           // position feed back pitch roll and yaw
#endif
#endif */
typedef struct __mavlink_attitude_quaternion_t {
 uint32_t time_boot_ms;                                                         /*< [ms] Timestamp (time since system boot).*/
 float32_t q1;                                                                  /*<  Quaternion component 1, w (1 in null-rotation)*/
 float32_t q2;                                                                  /*<  Quaternion component 2, x (0 in null-rotation)*/
 float32_t q3;                                                                  /*<  Quaternion component 3, y (0 in null-rotation)*/
 float32_t q4;                                                                  /*<  Quaternion component 4, z (0 in null-rotation)*/
 float32_t rollspeed;                                                           /*< [rad/s] Roll angular speed*/
 float32_t pitchspeed;                                                          /*< [rad/s] Pitch angular speed*/
 float32_t yawspeed;                                                            /*< [rad/s] Yaw angular speed*/
} mavlink_attitude_quaternion_t;

typedef struct __mavlink_gps_input_t {
 uint64_t time_usec;                                                            /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
 uint32_t time_week_ms;                                                         /*< [ms] GPS time (from start of GPS week)*/
 int32_t lat;                                                                   /*< [degE7] Latitude (WGS84)*/
 int32_t lon;                                                                   /*< [degE7] Longitude (WGS84)*/
 float32_t alt;                                                                 /*< [m] Altitude (MSL). Positive for up.*/
 float32_t hdop;                                                                /*< [m] GPS HDOP horizontal dilution of position*/
 float32_t vdop;                                                                /*< [m] GPS VDOP vertical dilution of position*/
 float32_t vn;                                                                  /*< [m/s] GPS velocity in NORTH direction in earth-fixed NED frame*/
 float32_t ve;                                                                  /*< [m/s] GPS velocity in EAST direction in earth-fixed NED frame*/
 float32_t vd;                                                                  /*< [m/s] GPS velocity in DOWN direction in earth-fixed NED frame*/
 float32_t speed_accuracy;                                                      /*< [m/s] GPS speed accuracy*/
 float32_t horiz_accuracy;                                                      /*< [m] GPS horizontal accuracy*/
 float32_t vert_accuracy;                                                       /*< [m] GPS vertical accuracy*/
 uint16_t ignore_flags;                                                         /*<  Bitmap indicating which GPS input flags fields to ignore.  All other fields must be provided.*/
 uint16_t time_week;                                                            /*<  GPS week number*/
 uint8_t gps_id;                                                                /*<  ID of the GPS for multiple GPS inputs*/
 uint8_t fix_type;                                                              /*<  0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK*/
 uint8_t satellites_visible;                                                    /*<  Number of satellites visible.*/
} mavlink_gps_input_t;

typedef struct __mavlink_global_position_int_t {
 uint32_t time_boot_ms;                                                         /*< [ms] Timestamp (time since system boot).*/
 int32_t lat;                                                                   /*< [degE7] Latitude, expressed*/
 int32_t lon;                                                                   /*< [degE7] Longitude, expressed*/
 int32_t alt;                                                                   /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
 int32_t relative_alt;                                                          /*< [mm] Altitude above ground*/
 int16_t vx;                                                                    /*< [cm/s] Ground X Speed (Latitude, positive north)*/
 int16_t vy;                                                                    /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
 int16_t vz;                                                                    /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
 uint16_t hdg;                                                                  /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
} mavlink_global_position_int_t;

#if (MAV_VER == 2u)                                                             // ==== MAVLINK ver 2.0 =============
#include "mavlink2_msg_types.h"
#define MAVLINK_STX2 253u
#include "mavlink_crc.h"
//#include "msg2_attitude_quartenion.h"                                           // position feed back pitch roll and yaw
#elif (MAV_VER == 1u)                                                           // ==== MAVLINK ver 1.0 =============
#include "mavlink1_msg_types.h"
#define MAVLINK_STX1 254u
#include "mavlink_crc.h"
//#include "msg1_attitude_quartenion.h"                                           // position feed back pitch roll and yaw
#endif

#include "CRC.h"

#ifdef RION_USED
#include "rion.h"
rionUTCDateTimeField_t  g_timeDate;                                             // global time using rion object
#endif

//#if defined(HTTP_USED)
#include "encoders.h"
//#endif

#if defined(UBIDOT_USED)
#ifndef HTTP_USED
#define HTTP_USED
#endif
#include "UbiDot.h"
#endif

#if defined(AIRMAP_USED)
#include "airmap.h"
#ifndef HTTP_USED
#define HTTP_USED
#ifndef PROTOBUF_USED                                                           /* we cant not have protobuf if we need airmap */
#define PROTOBUF_USED
#endif
#endif
#endif

#if defined(VOLZ_PROTOCOL)
#include "volz_protocol.h"
#endif

#if defined(ROBOTIS_PROTOCOL)
#include "robotis.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* =============== Communication Objects ================================== */
/* --------------- used in interrupts ------------------------------------- */
#ifdef UART1_INTERUPT                                                           // serial UART1 (for test purposes)
SerialObject_t UART1;                                                           // Define UART1 as UART object.
#endif
#ifdef UART2_INTERUPT
SerialSBGCObject_t UART2;                                                       // Define UART2 as UART object.
#endif
#ifdef UART4_INTERUPT                                                           // read the XS camera on serial UART4 (for test purposes)
SerialObject_t UART4;                                                           // Define UART4 as UART object.
#endif
#ifdef UART5_INTERUPT                                                           // read the Run Cam camera on serial UART5 (for test purposes)
SerialObject_t UART5;                                                           // Define UART5 as UART object.
#endif
#ifdef UART6_INTERUPT                                                           // read the liddar on serial UART6 (for test purposes)
SerialLwNxObject_t UART6;                                                       // Define UART6 as UART object.
// lwResponsePacket LwNxResponse;                                                  // LightWare liddar packet and status
#endif
#if defined(SERIAL_MAVLINK)                                                     /* mavlink has been defined as serial */
SerialMavObject_t MavLinkBuf;                                                   /* Define MAVLinkBuf as UART object */
#elif defined(USE_MAVLINK)                                                      /* then it must be over UDP */
EthMavlinkUDPObject_t MavLinkBuf;                                               /* Define MAVLinkBuf as UDP object */
#endif

#if (defined(USE_MAVLINK) && !defined(SERIAL_MAVLINK))
uint16_t packetSignaLen=0u;                                                     /* collection flag for mavlink2 packets that have a signature 16 bit as can be up to 279 bytes */
#if (defined(MAVLINK_TIMESTAMP_CHK) && (MAV_VER == 2u))                         /* we chose to enable the filter on signature */
uint64_t g_mavSignature=0ULL;                                                   /* mavlink 2 signature 6 bytes long */
uint64_t g_mavTimStamp=0ULL;                                                    /* mavlink 2 time stamp 6 bytes long */
uint64_t g_mavTimStampLast=0ULL;                                                /* mavlink 2 last timestamp for checking */
uint8_t g_linkId=0U;                                                            /* mavlink2 signature link id */
#define MAV_SIG_LINK_ID 20u                                                     /* assign a link id */
#endif
uint8_t g_SysId=0u;                                                             /* system id received for mavlink */
uint8_t mv_packet_len;                                                          /* byte in message representing packet length */
#define MAV_SYS_ID 0x1FU                                                        /* define the MAVLINK system id change this you are a multistation master */
uint16_t CRCReadBack;
uint16_t CRCCalc;
uint8_t CRC_CKA=0u;
uint8_t CRC_CKB=0u;
#endif   /* end if mavlink */

volatile uint8_t g_hmiResetESD1;                                         /* global volatile which is set when an ESD lockout occurs and is reset by the HMI */

#ifdef UBIBOT_DEVICE_USED
SerialObject_t UbiSerial;                                                       // Define UbiSerial as UART object.
#endif
#ifdef SONY_VISCA_PROTO_USED
SerialObject_t viscaSerial;                                                     // Define viscaSerial as UART object.
#endif
#ifdef JRT_LIDDAR_USED
USBObject_t JRTSerial;                                                          // Define JRTSerial as USB object
JRT_write_measure_t distSensIT03Conf;                                           /* Chengdu JRT meter IT03 distance sensor measurement configuration object */
JRT_read_measure_t distSensIT03Meas;                                            /* Chengdu JRT meter IT03 distance sensor measurement reading */
#endif

#if (CAMERA_TYPE == XS_CAM)
EthXSUDPObject_t XS_DATA;                                                        // Define struct to store XStream input data
#endif
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
EthSBGCUDPObject_t SBGC_DATA;                                                   // Define struct to store SBGC input data
volatile uint8_t g_SBGC_Stx_Char;                                               // Global start transmission char set in main and also read in interrupt
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
unsigned char *ptr_cmderror = (unsigned char *) &cmderror;                      // Define the pointer to the container struct
SBGC_cmd_setget_adj_vars_val_t getvar_rd;                                       // Container for the CMD_SET_ADJ_VARS_VAL command in response to above (7 values deep)
SBGC_cmd_board_info_t boardinforb;                                              // Container for BOARD_INFO readback
unsigned char *ptr_bi = (unsigned char *) &boardinforb;                         // Define the pointer to the container struct
SBGC_cmd_read_params_3_t readparam3;                                            // Container for config param read/write
unsigned char *ptr_rp3 = (unsigned char *) &readparam3;                         // Define the pointer to the container struct
SBGC_cmd_script_debug_t scriptstate;                                            // returned script state
SBGC_cmd_event_t eventCmd;                                                      // Container for the CMD_EVENT which is a readback
unsigned char *ptr_eve = (unsigned char *) &eventCmd;                           // Define the pointer to the container struct
SBGC_cmd_realtime_data_custom_reply_t realtimedatacust;                         // Container for real time data custom
#endif
#ifdef RUN_CAM_USED
EthRunCUDPObject_t RunC_DATA;                                                   // Define struct to store run cam input data
#endif
#if defined(ART_NET_USED)
EthArtNetUDPObject_t artNet_DATA;                                               // Define struct to store artnet4 protocol data
#endif
#if defined(ASN_E1_USED)
EthArtNetUDPObject_t ASN_E1_DATA;                                               // Define struct to store ASN E1 messages
#endif
#if defined(CBOR_COMS_USED)
EthCoAPUDPObject_t CBOR_DATA;                                                   // UDP data for CoAP using cbor
#define CBOR_UDP_PORT 1010u                                                     // check !!!!!!!!!!!!!!!!
#endif
#ifdef GEF_EGD_PLC
EthEGDUDPObject_t EGD_DATA;                                                        // Define struct to store EGD Data from GEC Fanuc PLC
#endif

#ifdef YI_CAM_USED
SOCKET_Intern_Dsc *YiSocket;                                                    // TCP ip socket
EthTCPObject_t YiCam_DATA;                                                      // Define struct to store yi action cam input data
TcpStateTimers YiTimer;                                                         // State timeouts for each TCP Step for the Socket used for Yi Cam
volatile uint8_t g_YiCamReqState;                                               // State of YiCam write messages
COMGS_YiOptActive_t hmiReqActive;                                               // structure to hold requests for each send message
volatile uint8_t XYRequest=0U;                                                  // Current button request (HMI Action Queue)
#endif
#ifdef SEQ_CAM_USED
#ifndef HTTP_USED
#define HTTP_USED                                                               // uses http protocol and that socket
#endif
EthTCPObject_t SeqCam_DATA;                                                     // Define struct to store sequioa cam input data
TcpStateTimers SeqTimer;                                                        // State timeouts for each TCP Step for the Socket used for Yi Cam
volatile uint8_t g_SeqCamReqState;                                              // State of Sequioa Cam write messages
#endif
#if defined(MODBUS_TCP)
SOCKET_Intern_Dsc *MbusSocket;                                                  // TCP ip socket for modbus TCP/ip
EthTCPObject_t Modbus_DATA;                                                     // modbus tcp receive/send Packet from/to slaves
#endif
#ifdef UBIBOT_SERVER_USED                                                       // UbiBot Server communications
#ifndef HTTP_USED
#define HTTP_USED                                                               // uses http protocol
#endif
#endif
#if defined(LW3_SWITCH_USED)
SOCKET_Intern_Dsc *Lw3Socket;                                                   // TCP ip socket for LW3 over TCP/ip
EthTCPObject_t LW3_DATA;                                                        // LW3 tcp receive/send Packet from/to slaves
#ifndef TCP_LW3_PORT
#define TCP_LW3_PORT 6107U                                                      // tcp port to use (save memory dont include library)
#endif /* end ifndef in case the chain already has the lib */
#endif
#if defined(MICROSCAN_USED)
#if defined(MICROSCAN_OUT_TCP)
SOCKET_Intern_Dsc *MiCroScanSocket;                                             // TCP ip socket for microscan ocr camera
EthTCPObject_t MiCroScaN_DATA;                                                  // modbus tcp receive/send Packet from/to camera
#elif defined(MICROSCAN_OUT_UART)                                               // uart serial at 11500 receive/send Packet from/to camera
SerialObject_t MiCroScaN_SER_LR;                                                // left right camera
SerialObject_t MiCroScaN_SER_FB;                                                // front back camera
#endif /* end microscan camera mode */
#endif /* end microscan camera include */

#if defined(USE_TCP_CAN)
SOCKET_Intern_Dsc *tcpCanSocket;                                                // TCP ip socket for can over TCP/ip
#endif
#if defined(REMOTE_TCP_AMP_XY_CAM)
SOCKET_Intern_Dsc *remAmpSocket;                                                // TCP ip socket for remote amp encoder/decoder socket
#endif
#if defined(REMOTE_TCP_RUN_CAM)
SOCKET_Intern_Dsc *remRunCamSocket;                                             // TCP ip socket for run cam camera
#endif
#if defined(REMOTE_TCP_SBGC)
SOCKET_Intern_Dsc *remSBGCSocket;                                               // TCP ip socket for sbgc gimbal
#endif
#if defined(JSON_COMS_USED)
SOCKET_Intern_Dsc *jSONSocket;                                                  // TCP ip socket for json
#endif
#if defined(PENTAX_CAM_USED)
SOCKET_Intern_Dsc *pentaxSocket;                                                // TCP ip socket for pentax camera
#endif
#if defined(SMPTE_USED)
EthCoAPUDPObject_t SMPTE_DATA;                                                  // UDP buffer for smpte over udp (uses coAp struct assame max size buffer)
#endif
#if defined(NAVILOCK_USED)
navilock_t g_NavObject;                                                         // navilock object
#endif

// Declare gloabls used in interrupt as volatile for compiler optimisation to work
volatile uint8_t g_start_timer5=0u;                                             // Global signal to atart the timer
volatile uint64_t g_timer5_counter = 0LL;                                       // Global Timer Counter volatile as globally written in interrupt

#if defined(HTTP_USED)
#include "http.h"                                                               // include http helpers
#endif

#if defined(USE_RS485)
#include "RS485_ChipSel.h"                                                      // RS485 pin connections
#endif

unsigned char tmp;
// ================= Function definitions ======================================
void calculateTickDiff( int32_t *TimeDuration, uint32_t *TimeDurationLast );
void calculateTick2Now( int32_t *TimeDuration, uint32_t *TimeDurationLast );
void calculateTime2Tick( uint32_t *TimeDuration, uint32_t *TimeDurationLast, uint32_t *TimeDurationCurrent );
void setBit(uint16_t* mask, uint16_t n);
void eraseBit(uint16_t* mask, uint16_t n);
uint16_t isSet(uint16_t mask, uint16_t n);
uint16_t onesCount(uint16_t mask);
uint16_t zerosCount(uint16_t mask);
void StartTimeout();
void Status(int16_t address, int16_t state);
void Init_Ether();
uint16_t CRC16_Calc( const unsigned char *Packet, uint16_t Packet_length);
uint8_t CheckTimout();
void StartTCPCommunication();                                                   // Initialise the TCP Stack
void TCPStateCheck( SOCKET_Intern_Dsc *used_tcp_socket);                        // Check the State of the YiCam socket
void Net_Ethernet_Intern_UserTCP(SOCKET_Intern_Dsc *used_tcp_socket);
uint8_t Pay_checksum(const unsigned char *pdata, uint16_t sz);                  // A Function to calculate a checksum for a struct
uint16_t Net_Ethernet_Intern_UserUDP(UDP_Intern_Dsc *udpDsc);                   // Process a UDP Packet which came in
uint8_t sizeToChar( const unsigned char* stringX, unsigned char val );
void bubble_sort_array(int16_t *temp_array3, int8_t order);
void quick_list(int16_t *temp_array3, int8_t order);
void swap_double(float64_t* a, float64_t* b);
void selectionSort_double(float64_t arr[], int16_t n);
void selectionSortAux_double(float64_t arr[], float64_t arr2[], int16_t n);
void diff_double(float64_t *y, float64_t *f, int16_t  sz);
int16_t isneg_double(float64_t *y, int16_t  sz);
void do_rmse_retain_double( dVectr *a, dVectr *b, uint16_t num_points, dVectr *diff );
void do_rmse_iter_double( dVectr *a, dVectr *b, uint16_t num_points, dVectr *diff );
void set_rmse_iter_double( dVectr *diff );
dVectr* do_rmse_ap_double( dVectr *a, uint16_t num_points );
dVectr* do_rmse_vp_double( dVectr *a, uint16_t num_points );
dVectr* do_rmse_double( dVectr *a, dVectr *b, uint16_t num_points );
float64_t boxbetts_f(const float64_t *a, float64_t *grad);
void humdev(const float64_t x, const float64_t y, float64_t *k, float64_t *l, float64_t *dkdx, float64_t *dkdy);
void shirley_bg( dVectr **pp, int16_t ppSize );
float64_t Guess_find_hwhm(int16_t pos, float64_t* area, float64_t *yy_, float64_t *xx_, int16_t yySiz );
dVectr Guess_estimate_peak_parameters( float64_t *yy_, size_t yySiz, float64_t *sigma_, float64_t *xx_, float64_t height_correction, float64_t width_correction );
dVectr Guess_estimate_linear_parameters( float64_t *xx_, float64_t *yy_, size_t yySiz );
dquat Guess_estimate_sigmoid_parameters( float64_t *xx_, float64_t *yy_, size_t yySize, float64_t upper, float64_t lower );
bool is_left_of_line( const dVectr p0, const dVectr p1, const dVectr p2 );
int16_t moser_de_bruijn_gen(const int16_t n, int16_t *S);                       /* moser de-bruijn seuence for creating a unique encoded sending sequence */
int8_t doMoserDeBruijn(const int16_t n, int16_t *sequence);
int8_t doBellmanFord(BF_Graph_t* graph, const int16_t src);                     /* bellman ford - alternative dijsktra */
void doFloydWarshall(const int16_t graph[FW_V][FW_V], int16_t **dist);          /* ffloyd warshall */
int8_t SieveOfEratosthenes(const int32_t n, int32_t *result);                   /* list prime numbers */
int16_t newman_conway_gen(const int16_t n, int16_t *f);
int8_t doNewmanConway(const int16_t n, int16_t *sequence);
int16_t collatz_gen(const int16_t n);
int8_t doCollatz(const int16_t n, int16_t *sequence);
void doFareySequence(const int16_t n, float64_t *sequence);
uint16_t randhash(uint8_t *init, uint16_t seed);
uint16_t unrandhash(uint16_t h);
float64_t randhashdouble(uint16_t seed, uint8_t *init, float64_t a, float64_t b);
int16_t intlog2(int16_t x);
void get_barycentric(float64_t x, int16_t *i, float64_t *f, int16_t i_low, int16_t i_high);
float64_t lerp(const float64_t value0, const float64_t value1, float64_t f);
float64_t bilerp(const float64_t v00, const float64_t v10, const float64_t v01, const float64_t v11, const float64_t fx, const float64_t fy);
float64_t trilerp(const float64_t v000, const float64_t v100, const float64_t v010, const float64_t v110, const float64_t v001, const float64_t v101, const float64_t v011, const float64_t v111, const float64_t fx, const float64_t fy, const float64_t fz);
float64_t quadlerp(const float64_t v0000, const float64_t v1000, const float64_t v0100, const float64_t v1100, const float64_t v0010, const float64_t v1010, const float64_t v0110, const float64_t v1110, const float64_t v0001, const float64_t v1001, const float64_t v0101, const float64_t v1101, const float64_t v0011, const float64_t v1011, const float64_t v0111, const float64_t v1111, float64_t fx, float64_t fy, float64_t fz, float64_t ft);
void quadratic_bspline_weights(const float64_t f, float64_t *w0, float64_t *w1, float64_t *w2);
void cubic_interp_weights(const float64_t f, float64_t *wneg1, float64_t *w0, float64_t *w1, float64_t *w2);
float64_t cubic_interp(const float64_t value_neg1, const float64_t value0, const float64_t value1, const float64_t value2, float64_t f);
void zero(float64_t *v);
float64_t abs_max(const float64_t *v);
float64_t bezlerp(const float64_t a,const float64_t b,const float64_t c);
float64_t quadBezier(const float64_t t,const float64_t p0,const float64_t p1,const float64_t p2);
float64_t cubicBezier( const float64_t t,const float64_t p0,const float64_t p1,const float64_t p2,const float64_t p3);
float32_t cross2d( Vectr a, Vectr b );
float32_t dot2d( Vectr a, Vectr b );
Vectr invBilinear( Vectr p, Vectr a, Vectr b, Vectr c, Vectr d );
float32_t sdSegment( Vectr p, Vectr a, Vectr b );
float32_t vlength( Vectr v );
Vectr vmul( Vectr a, float32_t x );
void CERES_AngleAxisRotatePoint(float64_t *angle_axis, float64_t *pt, float64_t *result);
bool SnavelyReprojectionError(float64_t *camera, float64_t *point, float64_t *residuals, float64_t observed_x, float64_t observed_y );
int8_t CERES_QuaternionProduct(const float64_t *z, const float64_t *w, float64_t *zw);
int8_t CERES_AngleAxisToQuaternion(const float64_t* angle_axis, float64_t* quaternion);
int8_t CERES_QuaternionToAngleAxis(const float64_t* quaternion, float64_t* angle_axis);
int8_t CERES_AngleAxisToRotationMatrix( const float64_t* angle_axis, float64_t** R );
int8_t CERES_UnitQuaternionRotatePoint(const float64_t *q, const float64_t *pt, float64_t *result);
int8_t CERES_QuaternionRotatePoint(const float64_t *q, const float64_t *pt, float64_t *result);
int8_t CERES_QuaternionToScaledRotation( const float64_t *q, float64_t **R );
int8_t CERES_QuaternionToRotation( const float64_t *q, float64_t **R );
int8_t CERES_EulerAnglesToRotationMatrix(  const float64_t* euler, float64_t **R );
int8_t CERES_RotationMatrixToQuaternion(  const float64_t **R, float64_t *quaternion );
void CERES_RotationMatrixToAngleAxis( const float64_t **R, float64_t *angle_axis );
void KnuthRng_seed(uint32_t *s0, uint32_t *s1, uint32_t *state0, uint32_t *state1);
uint32_t KnuthRng_get( uint32_t *state0, uint32_t *state1 );
#if defined(FFT_NOTCH_REQ)
float64_t quinn_tau(float64_t x);
void do_quinn_est( quinn_est_t *quin, float64_t sampleRate, float64_t peakPosIndex, float64_t N, float64_t **out );
float64_t parzen(int16_t i, int16_t nn);
float64_t welch(int16_t i, int16_t nn);
float64_t hanning(int16_t i, int16_t nn);
float64_t hamming(int16_t i, int16_t nn);
float64_t blackman(int16_t i, int16_t nn);
float64_t steeper(int16_t i, int16_t nn);
float64_t planck_taper(int16_t i, int16_t nn);
float64_t gaussian(int16_t i, int16_t nn);
float64_t tukey(int16_t i, int16_t nn);
float64_t flattop(int16_t i, int16_t nn);
float64_t blackman_harris(int16_t i, int16_t nn);
float64_t blackman_nutall(int16_t i, int16_t nn);
float64_t nutall(int16_t i, int16_t nn);
int8_t power_subtract_octave(int16_t n, float64_t *p, float64_t factor);
int8_t power_subtract_ave(int16_t n, float64_t *p, int16_t m, float64_t factor);
int8_t HC_complex_phase_vocoder(int16_t len, const float64_t *fs, const float64_t *ft, const float64_t *f_out_old, float64_t *f_out);
void HC_puckette_lock(int64_t len, const float64_t *y, float64_t *z);
void HC_abs(int64_t len, const float64_t *x, float64_t *z);
void HC_div(int64_t len, const float64_t *x, const float64_t *y, float64_t *z);
void HC_mul(int64_t len, const float64_t *x, const float64_t *y, float64_t *z);
void polar_to_HC_scale(int64_t len, const float64_t *amp, const float64_t *phs, int16_t conj, int16_t scale, float64_t *freq);
void polar_to_HC(int64_t len, const float64_t *amp, const float64_t *phs, int16_t conj, float64_t *freq);
void HC_to_amp2(int64_t len, const float64_t *freq, float64_t scale, float64_t *amp2);
void HC_to_polar2(int64_t len, const float64_t *freq, int16_t conj, float64_t scale,  float64_t *amp2, float64_t *phs);
void HC_to_polar(int64_t len, const float64_t *freq, int16_t conj, float64_t *amp, float64_t *phs);
int8_t init_den(int16_t n, char flag_window, float64_t *den);
int8_t windowing(int16_t n, const float64_t *dataV, int16_t flag_window, float64_t scale, float64_t *out);
void apply_FFT(int16_t len, const float64_t *dataV, int16_t flag_window, float64_t *in, float64_t *out, float64_t scale, float64_t *amp, float64_t *phs);
#endif
#if defined(WEATHER_STAT)
void RstDavisWeatherStation( uint8_t uartNo );
void RstKestrelWeatherStation( );
int16_t getKestrelValues( char* msg, float32_t* values );
void decodeKestrel( uint8_t *packet, WeatherStat_t *kesDat );
void decodeDavisNet( uint8_t *packet, WeatherStat_t *davDat, uint8_t portNo );
#endif
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
uint8_t ReadSBGC(const unsigned char *pdata, uint16_t psz, uint16_t arrPosition);     // Read the data portion of the UDP datagram and get SBGC data.
uint8_t SBGC_process_UDP_InBuffer( const SBGC_cmd_board_info_t *boardinf );           // Process the UDP buffer for SimpleBGC
uint8_t check_payload(uint16_t buffStartPos, const SBGC_cmd_board_info_t *boardinf );
#endif                                                                          // ==================== eNd SBGC ===================================
#if defined(YI_CAM_USED)
uint8_t XY_OpenTCPSocketRawJson( SOCKET_Intern_Dsc **sock1 );                   // Opens RAW tcp socket for JSON communication for Yi Action Cam
uint8_t XY_CloseTCPSocketRawJson( SOCKET_Intern_Dsc *sock1 );                   // Closes RAW tcp socket for JSON communication for Yi Action Cam
void resetXYActions();
#endif                                                                          // ==================== eNd Yi Action Cam ============================
#if defined(SEQ_CAM_USED)
uint8_t SEQ_OpenTCPSocketHttpJson( SOCKET_Intern_Dsc **sock1 );                 // Opens http api tcp socket for JSON communication for Sequoia Cam
uint8_t SEQ_CloseTCPSocketHttpJson( SOCKET_Intern_Dsc *sock1 );                 // Closes http api tcp socket for JSON communication for Sequoia Cam
//void resetXYActions();
#endif                                                                          // ==================== eNd Parrot ==================================
#if defined(YI_CAM_USED) || defined(SEQ_CAM_USED)
void ProcessTcpMsg();
#endif                                                                          // ================== eNd Yi or Parrot (JSON TCP/IP) ===============
#ifdef RUN_CAM_USED
uint8_t ReadRunCam( const unsigned char *pdata, uint16_t psz );                       // Read the data portion of the UDP datagram and get Run Cam data.
#endif                                                                          // ================== Run Cam ======================================
#if (CAMERA_TYPE == XS_CAM)
int8_t XS_process_UDP_InBuffer();                                               // Process the in buffer from XStream and decide what it means
int8_t XS_process_SERIAL_InBuffer();                                            // Process the in buffer from XStream and decide what it means
#endif                                                                          // ================== AMP XS Encoder / Decoder =====================
#if defined(GEF_EGD_PLC)
uint8_t ReadEGD( const unsigned char *pdata, uint16_t psz );                    // STUB TO DO !!
#endif
#if defined(ART_NET_USED)
uint8_t ReadArtNet( const unsigned char *pdata, uint16_t psz );                 // STUB TO DO !! process the ArtNet data received
#endif
#ifdef USE_CAN
void Can2_Test();
void Init_Can();
void CanUDP_doData();
void bitWrite(char *x, char n, char value);
char bitRead(char *x, char n);
#endif
char ARP(const unsigned char *NodeIPAddr);
void boot_arp();
void StopTimer1();
void Init_MCU();
#ifdef TIMER1_NEEDED                                                            // if you use Ethernet IP
void Init_Timer1();
#endif
#ifdef TIMER2_NEEDED
void Init_Timer2();
void Init_Timer2_3();
#endif
void Init_Timer5();
// Init_PHYPins();
void Init_UART( uint16_t parityValue );
#if defined (HTTP_USED)
uint64_t dateRIONTimeToUnixTime(rionUTCDateTimeField_t *dt);
uint8_t airMapAddPosn(const mavlink_gps_input_t *gpsDat, mavlink_global_position_int_t *gpInt, AirMap_Position_t *amPosn, rionUTCDateTimeField_t *syncedTime );
uint8_t airMapAddVelocity(const mavlink_gps_input_t *gpsDat,  AirMap_Velocity_t *amSpeed, rionUTCDateTimeField_t *syncedTime );
uint8_t airMapaddaTtit(const mavlink_attitude_quaternion_t *attQuart,  AirMap_Attitude_t *amAttit, rionUTCDateTimeField_t *syncedTime );
uint8_t airMapaddPress(float32_t *press,  AirMap_Barometer_t *amBaro, rionUTCDateTimeField_t *syncedTime );
uint8_t UbiTCP_checkIpAddress( char *ipAddress );
float32_t checkHttpRespUbiDots( HttpConnect_t *conn, HttpClientContext_t *context );
uint8_t UbiDotaddContext(char *key_label, char *key_value, UbiProtocolHandler_t *proto, ContextUbi_t *_context);
uint8_t UbiDotaddDot(char *key_label, float32_t *dot_value, UbiProtocolHandler_t *proto, ubiValue_t *_dots, rionUTCDateTimeField_t *syncedTime );
void UbiDot_floatToChar(char *str_value, float32_t value);
void UbiDotbuildAnyPayload(char *payload, UbiProtocolHandler_t *ubiDots, ubiValue_t *_dots);
int16_t http_read_cb( HttpConnect_t* conn, int16_t revents, SOCKET_Intern_Dsc *sock1  );   // http chunk reader
error_t httpClientShutdownConnection(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
error_t httpClientReceiveData(HttpClientContext_t *context, void *dataV, size_t size, size_t *received, uint8_t flags);
void httpClientChangeState(HttpClientContext_t *context, HttpClientState newState);
error_t httpClientDisconnect(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
error_t httpClientCreateRequest(HttpClientContext_t *context);
error_t httpClientSetMethod(HttpClientContext_t *context, const char_t *method);
error_t httpClientSetUri(HttpClientContext_t *context, const char_t *uri);
error_t httpClientAddQueryParam(HttpClientContext_t *context, const char_t *name, const char_t *value);
error_t httpClientAddHeaderField(HttpClientContext_t *context, const char_t *name, const char_t *value);
error_t httpClientWriteHeader(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
error_t httpClientWriteBody(HttpClientContext_t *context, const void *dataV, size_t length, size_t *written, uint8_t flags, SOCKET_Intern_Dsc *socket);
error_t httpClientReadHeader(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
uint16_t httpClientGetStatus(HttpClientContext_t *context);
const char_t *httpClientGetHeaderField(HttpClientContext_t *context, const char_t *name);
error_t httpClientReadBody(HttpClientContext_t *context, void *dataV, size_t size, size_t *received, uint8_t flags);
error_t httpClientReadTrailer(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
error_t httpClientCloseBody(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
error_t httpClientSetVersion(HttpClientContext_t *context, HttpVersion version);
error_t httpCheckCharset(const char_t *s, size_t length, uint8_t charset);
void httpClientChangeRequestState(HttpClientContext_t *context, HttpRequestState newState);
error_t httpClientParseHeaderField(HttpClientContext_t *context, char_t *line, size_t length);
error_t httpClientParseConnectionField(HttpClientContext_t *context, const char_t *value);
int16_t isprintable(char c);
error_t httpClientParseTransferEncodingField(HttpClientContext_t *context, const char_t *value);
error_t httpClientParseContentLengthField(HttpClientContext_t *context, const char_t *value);
error_t httpClientParseStatusLine(HttpClientContext_t *context, char_t *line, size_t length);
error_t httpClientCheckTimeout(HttpClientContext_t *context);
error_t httpClientFormatRequestHeader(HttpClientContext_t *context);
error_t httpClientFormatChunkSize(HttpClientContext_t *context, size_t length);
error_t httpClientParseChunkSize(HttpClientContext_t *context, char_t *line, size_t length);
error_t httpClientWriteTrailer(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
error_t httpClientSendData(HttpClientContext_t *context, const void *dataV, size_t length, size_t *written, uint8_t flags, SOCKET_Intern_Dsc *socket);
error_t httpClientConnect(HttpClientContext_t *context, uint8_t *serverIpAddr, uint16_t serverPort);
void httpClientCloseConnection(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
error_t httpClientFormatAuthorizationField(HttpClientContext_t *context);
void httpClientInitAuthParams(HttpClientAuthParams_t *authParams);
#endif
#if defined(JRT_LIDDAR_USED)
void initUSB( USBObject_t *USBObj );
void ReadUSB_JRT( JRT_read_measure_t *distMeas, USBObject_t *USBObj, JRT_write_measure_t *deviceState );
#endif
#ifdef DO_COMPIC
char do_UART_to_UDP();
#endif
void Init_Node();
uint8_t checksum(const unsigned char *fdata, uint16_t sz);                  // A Function to calculate a checksum for old version of simpleBGC
uint8_t DataChangeDetected( uint16_t dataVal, uint16_t dataLast, uint8_t delta);// Function to detect a data change by more than X
float32_t Scale_Raw_Value( int16_t raw_min, int16_t raw_max, float32_t scale_min, float32_t scale_max, int16_t raw_value);
uint16_t Scale_Float_Value( float32_t raw_min, float32_t raw_max, uint16_t scale_min, uint16_t scale_max, float32_t float_value);
void Test_CRC();
void StartTimeout();
uint8_t debounce_button( IO_btn_state_t *btn, uint8_t new_state, uint64_t TimeThreshold );
void calculateTimeDiff( int64_t *TimeDuration, uint64_t *TimeDurationLast );
void calculateTime2Now( int64_t *TimeDuration, uint64_t *TimeDurationLast );
void calculateHrs2Now( uint8_t *TimeDuration, uint8_t *TimeDurationLast );
void calculateMins2Now( uint8_t *TimeDuration, uint8_t *TimeDurationLast );
uint8_t mov_avg( mov_avg_data_t *accStruct );
int16_t setRandRange(int16_t min, int16_t max);
#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
ADCStates_e readManualADC( uint8_t pinNo, uint16_t *rawValADC );
#endif
#if (!defined(SERIAL_MAVLINK) && defined(USE_MAVLINK))
void mavInitUDP();
void chooseMavlinkSink( mavSinkClass mavSink );
#endif
#if defined(BATCH_MIXER)
#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
void makeUpDoseTank( uint16_t *valADC, batchSeq1Recipe_t *rcpDat, mov_avg_data_t *movAvg  );
#else
void makeUpDoseTank( batchSeq1Recipe_t *rcpDat, mov_avg_data_t *movAvg  );
#endif
#endif /* end dose tank */
#if defined(NAVSTIK_USED)
float64_t NavStik_getRawValues( char* inpStr, navstickWordOffset_e offset  );
void NavStik_getAllValues( char* inpStr, navstik_object_t *nav, navstickUseCalib_e *mode  );
#endif
void updateRTOfromRTC( );
void initRTOfromNTP( );
void checkValve( device_t *dev );
void checkMotor( device_t *dev );
void checkPulseIntegrity( rotat_sens_obj_t *risingEdge1Obj, rotat_sens_obj_t *fallingEdge1Obj, rotat_sens_obj_t *risingEdge2Obj, rotat_sens_obj_t *fallingEdge2Obj, pulse_integrity_t *puls );
unsigned char *strTrimWhitespace(unsigned char *s);
void strRemoveTrailingSpace(unsigned char *s);
int32_t strtol(const char *nptr, char **endptr, register int16_t base);
uint32_t strtoul(const char *nptr, char **endptr, register int16_t base);
void moveVehicle( ocr_movement_obj_t *move, uint8_t *output );
#if defined(ROBOTIS_PROTOCOL)
void RobotisServo_add_stuffing(uint8_t *packet);
void Robotis_send_packet(uint8_t *txpacket);
void Robotis_send_command(uint8_t id, uint16_t reg, uint32_t value, uint8_t len);
#endif
#if defined(TRAMP_PROTOCOL)
void trampFrameGetSettings(trampFrame_t *frame);
void trampFrameSetFrequency(trampFrame_t *frame, const uint16_t frequency);
void trampFrameSetPower(trampFrame_t *frame, const uint16_t power);
void trampFrameSetActiveState(trampFrame_t *frame, const uint8_t active);
uint8_t trampParseResponseBuffer(trampSettings_t *settings, const uint8_t *buffer, size_t bufferLen);
#endif
#if defined(VOLZ_PROTOCOL)
void Volz_send_value( VolZ_t *vObj );
#endif
#if defined(AVNav_USED)
int16_t SerialMbed_writeCommand(char prefix, int16_t num, int8_t uartNo);
#endif
#if defined(KIN_SCUL)
void setKineticSculpture( int8_t servoNo, float32_t float_value, uint8_t uartPort );
#endif
#if defined(VIDEORAY_M5_USED)
uint32_t vm5_crc_slow(uint8_t const * const p_message, uint8_t n_bytes);
void vm5_static_thrust_msg(vm5_object_t *vm5);
#endif
#if defined(TILT_SERVO1_USED)
void servo1_write_uart(int16_t comport, uint8_t *buf, uint8_t len  );
void pop_queue( servo1_t *servo );
void push_queue( servo1_t *servo, servo1_request_e request );
void servo1_move(int16_t comport, int16_t ID, int16_t angle, int16_t rpm, servo1_t *servo);
void servo1_read_angle(int16_t comport, int16_t ID, servo1_t *servo);
void servo1_read_speed(int16_t comport, int16_t ID, servo1_t *servo);
void  servo1_read_ret_dly_time(int16_t comport, int16_t ID, servo1_t *servo);
void servo1_read_voltage(int16_t comport, int16_t ID, servo1_t *servo);
void servo1_read_temp(int16_t comport, int16_t ID, servo1_t *servo); 
#endif
/* Air Core Inductor Calculator */
float64_t Induc_f1( float64_t x );
float64_t Induc_f2( float64_t x );
float64_t InductanceCalc( float64_t a, float64_t b, float64_t n );
float64_t InductorCoilQ( float64_t a, float64_t b, float64_t f );
// External Calls protocol
uint16_t XtCalls_checksum(const unsigned char *pdata, uint16_t sz);
// video time conversion
void getTimeFromString( mavDelay_t *clock, unsigned char *inputString );
// sexagesimal conversion
float64_t valueFromSexagesimal( char *inputString, char *delim );
#if defined(TF_DISTORT)
void whiteGaussInitSeed( int16_t *seedIterator );
void whiteGaussianNoise(float64_t * n1, float64_t * n2);
float32_t generateGaussianNoise(float32_t mu, float32_t variance);
float64_t uniformNoise( const float64_t mag );
Vicon_update( const float64_t dt, float64_t tau_ );
#endif
#if defined(NAVILOCK_USED)
void navilockInterruptStateManager( navilock_t *nav );
#endif
/* ========== vision tool kit ======= */
float64_t ran2(int16_t *idum);
int16_t getIdum(const bool useRandom);
int16_t setIdumRand(const bool useRandom);
float64_t gasdev (int16_t *idum);
float64_t expdev(int16_t *idum);
float64_t iNvLab_angle( Vectr* pt1, Vectr* pt2, Vectr* pt0 );
float64_t lngamma(float64_t x);
float64_t poisson(const int16_t k, const float64_t mu);
float64_t AUC(const float64_t* model, const float64_t* rand, size_t sm, size_t sr, const float64_t step);
float32_t * inplaceAddBGnoise2(float32_t *src, const float32_t range, int16_t w, int16_t h);
float64_t uniformRandom(void);
float64_t gaussianRand(void);
float64_t ParticleFilter_getLikelihood(const float64_t z, const float64_t X);
float64_t AngleCosineFrom2DVectr(const Vectr p0, const Vectr p1, const Vectr p2);
float64_t Value_saturate(float64_t A, const float64_t n);
/* quaternion */
Vectr mkvec(float32_t x, float32_t y, float32_t z);
Vectr vneg(Vectr v);
Vectr vsub(Vectr a, Vectr b);
void fqAeqNormqA(quat *pqA);
void qAeqBxC(quat *pqA, const quat *pqB, const quat *pqC);
void fqAeq1(quat *pqA);
quat qconjgAxB(const quat *pqA, const quat *pqB);
float32_t matrix33_det( mat33 a );
bool matrix33_inverse(mat33 *inv, const mat33 a);
float64_t wrap_PI( float64_t AngleX ); 
mat33 mscl(float64_t s, mat33 a);
bool matrix44_inverse(mat44 *inv, const mat44 a);
mat44 m44scl(float64_t s, mat44 a);
void mul_mat44( const mat44 m0, const mat44 m1, mat44 *a );
void set_mat44( const dquat v0, const dquat v1, const dquat v2, const dquat v3, mat44 *a );
mat33 Matrix33_from_euler(float32_t roll, float32_t pitch, float32_t yaw);
void matrix33_to_eulerAngle( const mat33 mat, Vectr *theta );
float64_t Matrix4_det( const Matrix4d64_t a );
void invert_mat4x4(const float32_t src[16u], float32_t *dst[16u]);
bool inverseMatrix4x4(const float64_t *m, float64_t *out);
float64_t invf(int16_t *i, int16_t *j, const float64_t* m);
void Line_setQuantizedDir(int16_t numDirections, const xyline_t p1,const xyline_t p2, int16_t *itsDirectionIdx);
bool Line_quantize(int16_t numDirections, xyline_t * p1, xyline_t * p2, int16_t *itsDirectionIdx );
float64_t Line_getLength(const xyline_t p1, const xyline_t p2);
float64_t Line_getOri(const xyline_t p1, const xyline_t p2);
xyline_t Line_getCenter(const xyline_t p1, const xyline_t p2 );
bool Line_trans(xyline_t * p, xyline_t * p1, xyline_t * p2);
bool Line_shear(float64_t k1, float64_t k2, xyline_t *p1, xyline_t *p2);
bool Line_rotate(float64_t theta, xyline_t *p1, xyline_t *p2);
float64_t* Matrix99_mtranspose(float64_t m[9][9]);
dVectr* Vector3_rotate(uint8_t rotation, dVectr *Vect);
bool Quaternion_to_axis_angle(dVectr *v,const dquat q);
bool Quaternion_earth_to_body(dVectr *v, const dquat q);
dVectr matf64_dVec_mul(Matrix3f64_t a, dVectr v);
Matrix3f64_t Matrix3_from_euler(float64_t roll, float64_t pitch, float64_t yaw);
void Matrix3_to_euler(const Matrix3f64_t m, float64_t *roll, float64_t *pitch, float64_t *yaw);
dquat Quaternion_from_rotation(uint8_t rotation);
dVectr Quaternion_to_vector312(const dquat q);
Matrix3f64_t Matrix3_from_euler312(float64_t roll, float64_t pitch, float64_t yaw);
dVectr Vector3_to_euler312( const Matrix3f64_t m );
bool Quaternion_to_euler(float64_t *roll, float64_t *pitch, float64_t *yaw, const dquat q);
bool Quaternion_rotate(const dVectr *v, dquat *q);
dquat Quaternion_div(const dquat q, const dquat v);
dquat Quaternion_mul(const dquat v,const dquat q);
float64_t Quaternion_get_euler_yaw(const dquat q);
float64_t Quaternion_get_euler_pitch(const dquat q);
float64_t Quaternion_get_euler_roll(const dquat q);
bool Quaternion_from_axis_angle3(dVectr *v, dquat *q);
bool Quaternion_from_axis_angle2(dVectr *v, dquat *q, const float64_t theta);
dVectr dvecDiv( const dVectr q, float64_t div );
dquat dquatDiv( const dquat q, float64_t div );
bool Quaternion_from_axis_angle(const dVectr axis, float64_t theta, dquat *q);
bool Quaternion_from_euler(float64_t roll, float64_t pitch, float64_t yaw, dquat *q);
bool Quaternion_rotation_matrix(Matrix3f64_t *m, const dquat q);
bool Quaternion_rotation_matrix_norm(Matrix3f64_t *m, const dquat q);
bool Quaternion_from_rotation_matrix2(const Matrix3f64_t m, quat *q);
quat Quaternion_from_rotation_matrix(const Matrix3f64_t m);
float64_t Vectr_length(const dVectr q);
float64_t Quaternion_length(const dquat q);
dquat Quaternion_inverse(const dquat q);
Vectr vcross(Vectr a, Vectr b);
Vectr vadd(Vectr a, Vectr b);
bool matrix33_to_Matrix3f64(const mat33 mtx, Matrix3f64_t *a);
bool Matrix3f64_to_matrix33(mat33 *mtx, const Matrix3f64_t a);
float64_t Matrix3_det( const Matrix3f64_t a );
float64_t * mNsub(float64_t s, float64_t * a, float64_t * res, uint16_t nEle);
float64_t * mNadd(float64_t s, float64_t * a, float64_t * res, uint16_t nEle);
float64_t * mNdiv(float64_t s, float64_t * a, float64_t * res, uint16_t nEle);
float64_t * mNscl(float64_t s, float64_t * a, float64_t * res, uint16_t nEle);
Matrix4d64_t Matrix4_setIdentity();
void getRowMat4f64(uint16_t row, dquat *v, Matrix4d64_t *m00);
void getColumnMat4f64(uint16_t column, dquat *v, Matrix4d64_t *m00);
void setColumnMat4f64(uint16_t column, const quat v, Matrix4d64_t *m00);
void setRowMat4f64(uint16_t row, const dquat v, Matrix4d64_t *m00);
void Matrix4_transpose( Matrix4d64_t *mat );
void set_Matrix4( const dquat v0, const dquat v1, const dquat v2, const dquat v3, Matrix4d64_t *a );
void mul_Matrix4( const Matrix4d64_t m0, const Matrix4d64_t m1, Matrix4d64_t *out );
void setRotationZ_Matrix4(float64_t angle, Matrix4d64_t *out);
void setRotationY_Matrix4(float64_t angle, Matrix4d64_t *out);
void setRotationX_Matrix4(float64_t angle, Matrix4d64_t *out);
void setRotationX_mat44(float64_t angle, mat44 *a);
void setRotationY_mat44(float64_t angle, mat44 *a);
void setRotationZ_mat44(float64_t angle, mat44 *a);
void invertR_Matrix4(const Matrix4d64_t m, Matrix4d64_t *mat);
void invertR_mat44(const mat44 m, mat44 *mat);
void transform_Matrix4(const dquat in, const mat44 m, dquat *out  );
dquat transform_mat44(const dquat in, const mat44 m );
dquat mkdquat(float64_t x, float64_t y, float64_t z, float64_t w);
void setTranslation_mat16(const dVectr v, float64_t* matrix);
mat44 setTranslation_mat44(const dVectr v);
Matrix4d64_t setTranslation_Matrix4d64(const dVectr v);
void setRotationX_mat16(float64_t angle, float64_t *matrix);
Matrix4d64_t setRotationX_Matrix4d64(float64_t angle);
mat44 setRotationX_mat44_2(float64_t angle);
void setRotationY_mat16(float64_t angle, float64_t *matrix);
Matrix4d64_t setRotationY_Matrix4d64(float64_t angle);
mat44 setRotationY_mat44_2(float64_t angle);
void setRotationZ_matrix16(float64_t angle, float64_t *matrix);
Matrix4d64_t setRotationZ_Matrix4d64(float64_t angle);
mat44 setRotationZ_mat44_2(float64_t angle);
void setScale_mat16(const dVectr vec, float64_t *matrix);
mat44 setScale_mat44(const dVectr vec);
Matrix4d64_t setScale_Matrix4d64(const dVectr vec);
dquat dquatFrom_matrix16(const dVectr v, const float64_t *matrix);
dquat dquatFrom_mat44(const dVectr v, const mat44 matrix);
dquat dquatFrom_Matrix4d64(const dVectr v, const Matrix4d64_t matrix);
bool isAffine_matrix16(float64_t *matrix);
float64_t getNormOfStream( const float64_t* v, int16_t n );
uint8_t Polygon_outside(const u32point_t P, const u32point_t **V, uint16_t n);
uint32_t Polygon_complete(const u32point_t **V, uint32_t n);
/* ------ matrix rotation maths ------------------------------------------- */
bool Vectr_to_doubleStream(const Vectr *vec, float64_t *strm);
bool Vectr_from_doubleStream(Vectr *vec, const float64_t *strm);
bool doubleStream_from_matrix33(const mat33 *mtx, float64_t *strm);
bool matrix33_from_doubleStream(mat33 *mtx, const float64_t *strm);
void SVD_jacobi(float64_t* a, int16_t n, float64_t *s, float64_t *u, float64_t *v);
void SVD_singular_value_e(float64_t * a, int16_t n, singular_value_t* temp);
void bubble_sort_singular_value(singular_value_t* temp_array3, bubble_sort_action_e order);
void SVD_rotation(float64_t *a, int16_t n, int16_t p, int16_t q, float64_t* u);
void SVD_arrange(float64_t* a, int16_t n);
float64_t SVD_sign(float64_t val);
float64_t SVD_g_eigen(float64_t* a, int16_t n, int16_t row, int16_t col);
mat33 moore_penrose( const mat33 V, const mat33 U, Vectr W );
mat33 math_arun( const mat33 V, const mat33 U );
Matrix3f64_t cayley2rotF64( const dVectr cayley );
mat33 cayley2rotM33( const Vectr cayley );
Matrix3f64_t cayley2rot_reducedF64( const dVectr cayley );
mat33 cayley2rot_reducedM33( const Vectr cayley);
float32_t** cayley2rot_reducedF32( const float32_t *cayley );
dVectr math_rot2cayleyF64( const Matrix3f64_t R );
int8_t math_rot2cayleyM33( const mat33 *R, Vectr *cayley );
Matrix3f64_t Matrix3_multiply( const Matrix3f64_t m0, const Matrix3f64_t m1  );
bool matrix33_multiply( mat33 *a, const mat33 m0, const mat33 m1  );
Matrix3f64_t Matrix3f64Scl(float64_t s, const Matrix3f64_t a);
float32_t** mArrayScl(float64_t s, float32_t a[3][3]);
Matrix3f64_t Matrix3_inverse(const Matrix3f64_t a, const float64_t d);
Matrix3f64_t Matrix3_add( const Matrix3f64_t a, const Matrix3f64_t b );
bool matrix33_add(mat33 *mtx, mat33 *a, mat33 *b);
Matrix3f64_t Matrix3_subtract( const Matrix3f64_t a, const Matrix3f64_t b );
bool matrix33_subtract(mat33 *mtx, mat33 *a, mat33 *b);
Matrix3f64_t Matrix3_setIdentity();
bool matrix33_identity(mat33 *mtx);
bool Matrix3f64_Identity(Matrix3f64_t *a);
mat33 do_moore_penrose_inv( const mat33 V, const mat33 U, Vectr W );
void do_svd(float64_t *A, float64_t *S2, int16_t n);
/* ------ distance functions ---------------------------------------------- */
float32_t AVF_gcDistance(float32_t ilat1,float32_t ilon1,float32_t ilat2,float32_t ilon2);
float32_t AVF_gcDistanceNm(float32_t lat1,float32_t lon1,float32_t lat2,float32_t lon2);
GPS_Position_t* AVF_gcIntermediatePoint(float32_t dlat1, float32_t dlon1, float32_t dlat2, float32_t dlon2, float32_t *f);
float32_t AVF_calcBearing(float32_t ilat1,float32_t ilon1,float32_t ilat2,float32_t ilon2);
/* ------ AI Machine Learning --------------------------------------------- */
void ML_softmax(const float64_t* z[], float64_t *out[], float64_t *foo[], uint16_t zsize, const int16_t dim);
void ML_transpose(float64_t *m[], const int16_t C, const int16_t R, float64_t *mT[]);
void ML_dot(const float64_t* m1[], const float64_t* m2[], const int16_t m1_rows, const int16_t m1_columns, const int16_t m2_columns, float64_t *output[]);
float64_t ML_max(float64_t *z[], uint16_t datSize);
void ML_relu(const float64_t *z[], float64_t *o[], uint16_t datSize, float64_t value, uint8_t prime);
void ML_sigmoid(float64_t *in, uint16_t datSize, typeOfSigmoid_e sigAlgo, float64_t *out);
void linearTransFormation_Matrix4(quat v, Matrix4d64_t m, quat *mv);
quat linearTransFormation2_Matrix4(quat v, Matrix4d64_t m);
float64_t polyApprox_sigmoid_g3( float64_t x );
float64_t polyApprox_sigmoid_g5( float64_t x );
float64_t polyApprox_sigmoid_g7( float64_t x );
mat33 matrix9x9_ctAB( const mat33 ctA, mat33 *ctB, uint8_t k);
mat33 matrix33_transpose( const mat33 ct );
void matrix33_transpose2( const mat33 ct, mat33 *ct_T );
float64_t horners_rule(float64_t *coeffs, float64_t x);
float64_t horner_equation( float64_t x );
#if defined(ALLAN_VAR_FUNCS)
uint16_t returnAllenVarLen( const uint16_t T );
void avar_mo_cpp(const Vectr **Accel,const Vectr **Gyro, const uint16_t T, const uint16_t AllanVarlen, Vectr** out[DIMENSION_3D]);
uint16_t avar_getMaxyBarSize( const uint16_t AllanVarLen, const uint16_t T );
void avar_to_cpp(const Vectr **Accel, const Vectr **Gyro, const uint16_t T, const uint16_t AllanVarlen, const uint16_t maxBarLen, Vectr **out[DIMENSION_3D]);
#endif
/* quaternion rotation math functions */
void quatMultiply(float64_t *qr, float64_t *q1, float64_t *q2);
void eulerToQuatYPR(float64_t *q, float64_t yaw, float64_t pitch, float64_t roll);
void eulerToQuatRPY(float64_t *q, float64_t roll, float64_t pitch, float64_t yaw);
void vectorNormalize(float64_t *v, int16_t n);
void nlerp(float64_t *r, const float64_t *a, const float64_t *b, const float64_t t);

//Extern unsigned int UART1TCP;
#ifdef USE_CAN
extern uint16_t Can1_RX_MSG_ID;
extern uint16_t Can1_TX_MSG_ID;
extern uint16_t Can2_RX_MSG_ID;
extern uint16_t Can2_TX_MSG_ID;
extern uint16_t Can1_Rcv_Flags;
extern uint16_t Can2_Rcv_Flags;
unsigned char getSBGCRequest[15u];                                              // SBGC over UDP request buffer
uint16_t Can2_Init_Flags, Can2_Rcv_Flags;                                       // can flags
#endif

unsigned char   dyna[31u] ;                                                     // buffer for dynamic response used in Test mode

#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
volatile ADCStates_e g_adcState;                                                // global state for the ADC
#endif

#ifdef YI_CAM_USED
unsigned char g_XYtcpBuffer[XY_MSG_MAX_SND_LEN];                                // Buffer to send JSON requests to the webserver set in the main program
unsigned char xyTcpSendBuff[XY_MSG_MAX_SND_LEN];                                // TCP message send buffer for Yi Action Camera used in the TCP handler
#endif

//uint8_t Eth_Process_Status;                                                     // saved status of Net_Ethernet_Intern_doPacket() procedure
uint32_t EthTimePrevCounter =0U;
unsigned bit TimoutRunning;
uint16_t Ethernet_Fault =0U;
uint8_t Ether_Link_Present = FALSE;
uint8_t Ether_Link_Present_Was = 10U;
uint16_t EFinish = 0U;                                                          // Flag set to 1 when a complete message has been received
extern Net_Ethernet_Intern_arpCacheStruct Net_Ethernet_Intern_arpCache[];       // ARP cash, 3 entries max
Net_Ethernet_Intern_arpCacheStruct Private_Ethernet_Intern_arpCache[3u];

uint16_t UDP_CRC_Reg =0U;
uint16_t UDP_CRC_Good_Reg =0U;
uint16_t UDP_CRC_Bad_Reg =0U;

Net_Ethernet_Intern_arpCacheStruct gRemoteUnit;                                 // Declare the ARP cache structure to get the MAC Address
unsigned char *ptr_ru = (unsigned char *) &gRemoteUnit;                         // Define the pointer to the container struct for arpCache


H_B Heart;                                                                      // watchdog heartbeat type

#if defined(HTTP_USED)
SOCKET_Intern_Dsc *socketHTML;
HttpClientContext_t HTMLContext;
HttpConnect_t HTMLConn;
#endif

#if defined(BATCH_MIXER)
volatile batch_steps_e g_batchState=100u;                                       /* initialise the batch timers and start */

int64_t elapsedTime=0u;                                                         /* elapsed time in sequence */
volatile uint64_t lastBatchTime=0u;                                             /* last stored timer seconds */
uint16_t LevelInRaw=0u;                                                         /* raw level measurement 10bit ADC */
float32_t LevelInput=0u;                                                        /* conditioned value storing the level input */
uint8_t doseDuringMakeUp=1u;                                                    /* flag to keep doisng spraying when making up the tank */
uint8_t solidMakeUp=0u;                                                         /* flag to use solids as make-up rather than liquid */
uint8_t prev_ReqSprayOn=0u;
volatile uint8_t g_hmiReqSprayOn=0u;
volatile uint8_t batchState=100u;                                               /* SHOULD BE g_batchSatte initialise the batch timers and start */
#if defined(EXTENDED_ARM)
uint8_t extArmStep=0u;                                                          /* stage in sequence for extended arm control */
uint8_t batchStateLast=0u;                                                      /* memory of last batch state before exception action */
uint8_t extArmState;                                                            /* HMI variable or input switches commanding the arm */
uint8_t extArmStateLast=EXT_ARM_ERR_POS;                                        /* drive the arm */
uint8_t extActualState;                                                         /* actual status from limit switches for extender position */
uint8_t startedSpray=false;                                                     /* only enable extended arm position control after first pass of the sequence */
IO_btn_state_t armActState;                                                     /* debounce value from the actual arm position feedback proximity sensors */
int64_t elapsedTimeArm=0u;                                                      /* elapsed time in sequence */
uint64_t lastBatchTimeArm=0u;                                                   /* last stored timer seconds */

#endif

#endif  /* end batch mixer */

#if defined(AVNav_USED)

#define AVNav_DATA_NUM_TX (sizeof(AVNav_txList) / sizeof(AVNav_txList[0u]))
#define AVNav_DATA_NUM_RX (sizeof(AVNav_rxList) / sizeof(AVNav_rxList[0u]))
/*
 * Valid commands to send to AVNavControl.
 * Format is {[prefix], [source]}.
 * [source] is where Serial will get data to send to AVNavControl.
 */
const int16_t AVNav_txList[][2u] =
{
        {'h', 359},
        {'d', 200},
        {'p', 200},
        {'r', 2},
};

/* 
 * Valid commands to receive from AVNavControl.
 * Format is {[prefix], [destination]}
 * [destination] is where Serial will put data from AVNavControl.
 */
const int16_t AVNav_rxList[][2u] =
{
        {'h', 359},
        {'d', 200},
        {'p', 200},
        {'e', 0},
};
#define EMERGENCIES_NUM 4
const int16_t AVNav_statePropRanges[STATE_PROP_NUM][2u] =
{
        {0, 359},                                                                // current heading
        {0, 200},                                                                // current depth
        {0, 200},                                                                // current power
        {0, 359},                                                                // desired heading
        {0, 200},                                                                // desired depth
        {0, 200},                                                                // desired power
        {0,   2},                                                                // dropper state        
        {R_ALIVE, R_STOPPED | R_KILLED},                                        // run state
        {T_GATE, T_PARTY},                                                        // current task
        {T_NUL, T_PARTY},                                                        // last task completed
        {0, EMERGENCIES_NUM - 1}                                                // emergency code
};
const int16_t AVNav_stateProps[STATE_PROP_NUM] =
{
        0,                                                                      /* current heading */
        0,                                                                      /* current depth */
        100,                                                                    /* current power */
        0,                                                                      /* desired heading */
        0,                                                                      /* desired depth */
        100,                                                                    /* desired power */
        0,                                                                      /* dropper state */
        R_KILLED,                                                               /* runState */
        T_GATE,                                                                 /* missionState*/
        T_NUL,                                                                  /* lastTask */
        0                                                                       /* emergencyCode */
};
#endif /* end AvNav */

static const uint16_t Unixdays[4u][12u] =                                       // Used for calculating time stamp since 1970

{

    {   0U,  31U,     60U,     91U,     121U,    152U,    182U,    213U,    244U,    274U,    305U,    335U},

    { 366U,  397U,    425U,    456U,    486U,    517U,    547U,    578U,    609U,    639U,    670U,    700U},

    { 731U,  762U,    790U,    821U,    851U,    882U,    912U,    943U,    974U,    1004U,   1035U,   1065U},

    {1096U,  1127U,   1155U,   1186U,   1216U,   1247U,   1277U,   1308U,   1339U,   1369U,   1400U,   1430U},

};

/*-----------------------------------------------------------------------------
 *      calculateTickDiff:  Simple function to give a time difference between 
 *                          two tick counters
 *
 *  Parameters:  int32_t *TimeDuration, uint32_t *TimeDurationLast
 *               TimeDuration set to -ve means initialise
 *
 *  Return: nothing value put into timeduration variable
 *----------------------------------------------------------------------------*/
void calculateTickDiff( int32_t *TimeDuration, uint32_t *TimeDurationLast )
{
  uint32_t ticksNow=0ul;                                                        // varible we store the ticks to

  if (*TimeDuration<=-1)                                                         // Value to initialise
  {
    *TimeDurationLast=CP0_GET(CP0_COUNT);;                                       // Remember the current tick value
    *TimeDuration==0;                                                            // first time differnce after init is zero
  }
  else
  {
    ticksNow=CP0_GET(CP0_COUNT);                                                // get the current ticks from the co-processor
    if ( *TimeDurationLast > ticksNow )                                          // Overrun
    {
       *TimeDuration = (UINT64_MAX - *TimeDurationLast) + ticksNow;               // Calculate difference in time
    }
    else
    {
       *TimeDuration = (ticksNow - *TimeDurationLast);                            // Calculate difference in time
    }
    *TimeDurationLast = ticksNow;                                                // Remember the last one
  }
}
/*-----------------------------------------------------------------------------
 *      calculateTick2Now:  Simple function to give a time difference between
 *                          two tick counters
 *
 *  Parameters:  int32_t *TimeDuration, uint32_t *TimeDurationLast
 *               TimeDuration set to -ve means initialise
 *
 *  Return: nothing value put into timeduration variable
 *----------------------------------------------------------------------------*/
void calculateTick2Now( int32_t *TimeDuration, uint32_t *TimeDurationLast )
{
  uint32_t ticksNow=0ul;                                                        // varible we store the ticks to

  if (*TimeDuration<=-1)                                                         // Value to initialise
  {
    *TimeDurationLast=CP0_GET(CP0_COUNT);;                                       // Remember the current tick value
    *TimeDuration==0;                                                            // first time differnce after init is zero
  }
  else
  {
    ticksNow=CP0_GET(CP0_COUNT);                                                // get the current ticks from the co-processor
    if ( *TimeDurationLast > ticksNow )                                          // Overrun
    {
       *TimeDuration = (UINT64_MAX - *TimeDurationLast) + ticksNow;               // Calculate difference in time
    }
    else
    {
       *TimeDuration = (ticksNow - *TimeDurationLast);                            // Calculate difference in time
    }
    // TimeDurationLast = ticksNow;                                             // dont Remember the last one only from start trigger
  }
}
/*-----------------------------------------------------------------------------
 *      calculateTime2Tick:  Simple function to give a time difference between
 *                          two tick counters
 *
 *  Parameters:  uint32_t *TimeDuration, uint32_t *TimeDurationLast
 *               uint32_t *TimeDurationCurrent
 *
 *  Return: nothing value put into timeduration variable
 *----------------------------------------------------------------------------*/
void calculateTime2Tick( uint32_t *TimeDuration, uint32_t *TimeDurationLast, uint32_t *TimeDurationCurrent )
{
    if ( *TimeDurationLast > *TimeDurationCurrent )                             // Overrun
    {
       *TimeDuration = (UINT64_MAX - *TimeDurationLast) + *TimeDurationCurrent; // Calculate difference in time
    }
    else
    {
       *TimeDuration = (*TimeDurationCurrent - *TimeDurationLast);              // Calculate difference in time
    }
}
/**
 * @brief Removes all leading and trailing whitespace from a string
 * @param[in] s The string that will be trimmed
 * @return String with whitespace stripped from the beginning and end
 **/
unsigned char *strTrimWhitespace(unsigned char *s)
{
   unsigned char *end;
   unsigned char *result;

   //Trim whitespace from the beginning
   while(isspace((uint8_t) *s))
   {
      s++;
   }

   //Save the current position
   result = s;

   //Search for the first whitespace to remove
   //at the end of the string
   for(end = NULL; *s != '\0'; s++)
   {
      if(!isspace((uint8_t) *s))
         end = NULL;
      else if(!end)
         end = s;
   }

   //Trim whitespace from the end
   if(end)
      *end = '\0';

   //Return the string with leading and
   //trailing whitespace omitted
   return result;
}

/**
 * @brief Removes all trailing whitespace from a string
 * @param[in,out] s Pointer to a NULL-terminated character string
 **/
void strRemoveTrailingSpace(unsigned char *s)
{
   unsigned char *end;

   //Search for the first whitespace to remove
   //at the end of the string
   for(end = NULL; *s != '\0'; s++)
   {
      if(!isspace((uint8_t) *s))
         end = NULL;
      else if(!end)
         end = s;
   }

   //Trim whitespace from the end
   if(end)
      *end = '\0';
}

/* basic bitwise maths */
/*-----------------------------------------------------------------------------
 *  setBit:  sets bit in word
 *
 *  Parameters: uint16_t* mask, uint16_t n
 *
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/
void setBit(uint16_t* mask, uint16_t n)
{
    *mask |= (1 << n);
}

/*-----------------------------------------------------------------------------
 *  eraseBit:  unsets bit in word
 *
 *  Parameters: uint16_t* mask, uint16_t n
 *
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/ 
void eraseBit(uint16_t* mask, uint16_t n)
{
    *mask &= ~(1 << n);
}

/*-----------------------------------------------------------------------------
 *  isSet:  checks if nth bit set in word
 *
 *  Parameters: uint16_t* mask, uint16_t n
 *
 *  Return: uint16_t
 *             
 *----------------------------------------------------------------------------*/  
uint16_t isSet(uint16_t mask, uint16_t n)
{
    return mask & (1 << n);
}

/*-----------------------------------------------------------------------------
 *  onesCount:  counts total bits set in word
 *
 *  Parameters: uint16_t mask
 *
 *  Return: uint16_t
 *             
 *----------------------------------------------------------------------------*/  
uint16_t onesCount(uint16_t mask)
{
    uint16_t count = 0;
    uint16_t i;
    for (i = 0; i < sizeof(uint16_t) * 8; i++)
    {
        if (isSet(mask, i))
        {
            count++;
        }
    }
    return count;
}

/*-----------------------------------------------------------------------------
 *  zerosCount:  counts total bits unset in word
 *
 *  Parameters: uint16_t mask
 *
 *  Return: uint16_t
 *             
 *----------------------------------------------------------------------------*/  
uint16_t zerosCount(uint16_t mask)
{
    return sizeof(uint16_t) * 8 - onesCount(mask);
}
/*-
 * Copyright (c) 1990 The Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. [rescinded 22 July 1999]
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
@deftypefn Supplemental {long int} strtol (const char *@var{string}, @
  char **@var{endptr}, int @var{base})
@deftypefnx Supplemental {unsigned long int} strtoul (const char *@var{string}, @
  char **@var{endptr}, int @var{base})

The @code{strtol} function converts the string in @var{string} to a
long integer value according to the given @var{base}, which must be
between 2 and 36 inclusive, or be the special value 0.  If @var{base}
is 0, @code{strtol} will look for the prefixes @code{0} and @code{0x}
to indicate bases 8 and 16, respectively, else default to base 10.
When the base is 16 (either explicitly or implicitly), a prefix of
@code{0x} is allowed.  The handling of @var{endptr} is as that of
@code{strtod} above.  The @code{strtoul} function is the same, except
that the converted value is unsigned.

@end deftypefn
*/

/* FIXME: It'd be nice to configure around these, but the include files are too
   painful.  These macros should at least be more portable than hardwired hex
   constants. */

#ifndef ULONG_MAX
#define        ULONG_MAX        ((unsigned long)(~0L))                /* 0xFFFFFFFF */
#endif

#ifndef LONG_MAX
#define        LONG_MAX        ((long)(ULONG_MAX >> 1))        /* 0x7FFFFFFF */
#endif

#ifndef LONG_MIN
#define        LONG_MIN        ((long)(~LONG_MAX))                /* 0x80000000 */
#endif
/*
 * Convert a string to a long integer.
 *
 * Ignores `locale' stuff.  Assumes that the upper and lower case
 * alphabets and digits are each contiguous.
 */
int32_t strtol(const char *nptr, char **endptr, register int16_t base)
{
   const char *s = nptr;
   uint32_t acc;
   int16_t c;
   uint32_t cutoff;
   int16_t neg = 0, any, cutlim;
   /*
    * Skip white space and pick up leading +/- sign if any.
    * If base is 0, allow 0x for hex and 0 for octal, else
    * assume decimal; if base is already 16, allow 0x.
   */
        do {
                c = *s++;
        } while (isspace(c));

        if (c == '-') {
                neg = 1;
                c = *s++;
        } else if (c == '+')
                c = *s++;
        if ((base == 0 || base == 16) &&
            c == '0' && (*s == 'x' || *s == 'X')) {
                c = s[1];
                s += 2;
                base = 16;
        }

        if (base == 0)
                base = c == '0' ? 8 : 10;
        /*
         * Compute the cutoff value between legal numbers and illegal
         * numbers.  That is the largest legal value, divided by the
         * base.  An input number that is greater than this value, if
         * followed by a legal input character, is too big.  One that
         * is equal to this value may be valid or not; the limit
         * between valid and invalid numbers is then based on the last
         * digit.  For instance, if the range for longs is
         * [-2147483648..2147483647] and the input base is 10,
         * cutoff will be set to 214748364 and cutlim to either
         * 7 (neg==0) or 8 (neg==1), meaning that if we have accumulated
         * a value > 214748364, or equal but the next digit is > 7 (or 8),
         * the number is too big, and we will return a range error.
         *
         * Set any if any `digits' consumed; make it negative to indicate
         * overflow.
         */
        cutoff = neg ? -(unsigned long)LONG_MIN : LONG_MAX;
        cutlim = cutoff % (unsigned long)base;
        cutoff /= (unsigned long)base;

        for (acc = 0, any = 0;; c = *s++) {
                if (isdigit(c))
                        c -= '0';
                else if (isalpha(c))
                        c -= isupper(c) ? 'A' - 10 : 'a' - 10;
                else
                        break;
                if (c >= base)
                        break;
                if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim))
                        any = -1;
                else {
                        any = 1;
                        acc *= base;
                        acc += c;
                }
        }

        if (any < 0) {
                acc = neg ? LONG_MIN : LONG_MAX;
                //errno = ERANGE;
        } else if (neg)
                acc = -acc;
        if (endptr != 0)
                *endptr = (char *) (any ? s - 1 : nptr);
        return (acc);
}
/*
 * Copyright (c) 1990 Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. [rescinded 22 July 1999]
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/*
 * Convert a string to an unsigned long integer.
 *
 * Ignores `locale' stuff.  Assumes that the upper and lower case
 * alphabets and digits are each contiguous.
 */
uint32_t strtoul(const char *nptr, char **endptr, register int16_t base)
{
        const char *s = nptr;
        uint32_t acc;
        int16_t c;
        uint32_t cutoff;
        int16_t neg = 0, any, cutlim;
        /*
         * See strtol for comments as to the logic used.
         */
        do {
                c = *s++;
        } while (isspace(c));

        if (c == '-') {
                neg = 1;
                c = *s++;

        } else if (c == '+')
                c = *s++;

        if ((base == 0 || base == 16) &&
            c == '0' && (*s == 'x' || *s == 'X')) {
                c = s[1];
                s += 2;
                base = 16;
        }

        if (base == 0)
                base = c == '0' ? 8 : 10;

        cutoff = (uint32_t)ULONG_MAX / (uint32_t)base;
        cutlim = (uint32_t)ULONG_MAX % (uint32_t)base;

        for (acc = 0, any = 0;; c = *s++) {
                if (isdigit(c))
                        c -= '0';
                else if (isalpha(c))
                        c -= isupper(c) ? 'A' - 10 : 'a' - 10;
                else
                        break;

                if (c >= base)
                        break;

                if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim))
                        any = -1;
                else {
                        any = 1;
                        acc *= base;
                        acc += c;
                }
        }
        if (any < 0) {
                acc = ULONG_MAX;
                //errno = ERANGE;

        } else if (neg)
                acc = -acc;

        if (endptr != 0)
                *endptr = (char *) (any ? s - 1 : nptr);
        return (acc);

}
///===================================================================================================================================================
//
//                                                 UDP Handler
//
//*****************************************************************************************************************************************************
#if (!defined(SERIAL_MAVLINK) && defined(USE_MAVLINK))
void mavInitUDP()
{
#if (((THIS_NODE_IS==GIMJOY) || (THIS_NODE_IS==COMGS)) || (THIS_NODE_IS==FIRGS))
    MavLinkBuf.From_Port=MAV_GC_UDP_DATA_PORT;                                  /* if you are the ground talking to the Air */
    MavLinkBuf.Dest_Port=MAV_STA_UDP_DATA_PORT;                                 /* this is the Air station */
#elif (THIS_NODE_IS==AIR)
    MavLinkBuf.From_Port=MAV_STA_UDP_DATA_PORT;                                 /* if you are the robot or air talking to the Ground/Master */
    MavLinkBuf.Dest_Port=MAV_GC_UDP_DATA_PORT;                                  /* this is the Ground/Master station */
#endif
}
#endif
/*
 * Function Name:
 *   Net_Ethernet_Intern_UserUDP
 * Description:
 *   This is UDP module routine. It is internally called by the library. The user accesses to the UDP request by using some of the
 *   Net_Ethernet_Intern_get routines. The user puts data in the transmit buffer by using some of the Net_Ethernet_Intern_put routines.
 *   The function must return the length in bytes of the UDP reply, or 0 if nothing to transmit. If you don't need
 *   to reply to the UDP requests, just define this function with a return(0) as single statement.
 *
 * Arguments:
 * void Net_Ethernet_Intern_UserUDP(UDP_Intern_Dsc *udpDsc);
 * udpDsc : pointer to global udpRecord_Intern variable, which contains properties of last received UDP Packet
   typedef struct {
  char remoteIP[4];              // Remote IP address
  char remoteMAC[6];             // Remote MAC address
  unsigned int remotePort;       // Remote UDP port
  unsigned int destPort;         // Destination UDP port
  unsigned int dataLength;       // Current UDP datagram payload size
  unsigned int broadcastMark;    // =0 -> Not broadcast; =1 -> Broadcast
  } UDP_Intern_Dsc;
 *Returns:
 *   0 - there should not be a reply to the request.
 *   Length of UDP reply data field - otherwise. This will use flushUDP to send what you wrote with putbytes
 */
uint16_t Net_Ethernet_Intern_UserUDP(UDP_Intern_Dsc *udpDsc)      
{
#ifdef USE_CAN
  uint16_t Can2_UDP_RX_Packet[8u];
  uint16_t reqLength=0u;
  uint16_t counter=0u,sendValue=0u;
#endif
  //int counter=0;
  unsigned char len;

      if (UDP_Test_Mode)
      {
        len = 0u;                                                                // my reply length
                                                                                // reply is made of the remote host IP address in human readable format
        ByteToStr(udpDsc->remoteIP[0u], dyna);                                   // first IP address byte
        dyna[3u] = '.';
        ByteToStr(udpDsc->remoteIP[1u], dyna + 4u);                               // second
        dyna[7u] = '.';
        ByteToStr(udpDsc->remoteIP[2u], dyna + 8u);                               // third
        dyna[11u] = '.';
        ByteToStr(udpDsc->remoteIP[3u], dyna + 12u);                              // fourth

        dyna[15u] = ':';                                                         // add separator

        // then remote host port number
        WordToStr(udpDsc->remotePort, dyna + 16u);
        dyna[21] = '[';
        WordToStr(udpDsc->destPort, dyna + 22u);
        dyna[27u] = ']';
        dyna[28u] = 0;

        // the total length of the request is the length of the dynamic string plus the text of the request
        len = 28u + udpDsc->dataLength;

        // puts the dynamic string into the transmit buffer
        Net_Ethernet_Intern_putBytes(dyna, 28u);

        // then puts the request string converted into upper char into the transmit buffer
        while(udpDsc->dataLength--)
        {
           Net_Ethernet_Intern_putByte(toupper(Net_Ethernet_Intern_getByte()));
        }
     }
     else if (~UDP_Test_Mode)
     {
       //timer_tmp5_1=0;
       // ANY port Heart Beat

        //Net_Ethernet_Intern_sendUDP(udpDsc->remoteIP, 11111, 11111, &SYN,1);
        if(udpDsc->remoteIP[3u] == Air.IpAddr[3u]) 
        {
            Air.Link_Alive   = TRUE;
            Air.Link_Alive_Time=Heart.Time;     
            if(Air.Link_Alive_Was != Air.Link_Alive)
            {
              Air.Link_Alive_Was = Air.Link_Alive;
              Status(ETHER_LINK_STATE,AIR_LINK_UP);
            }
        }
        else if (udpDsc->remoteIP[3u] == MasGs.IpAddr[3u])
        {
             MasGs.Link_Alive = TRUE;
             MasGs.Link_Alive_Time=Heart.Time;
             if(MasGs.Link_Alive_Was != MasGs.Link_Alive)
             {
               MasGs.Link_Alive_Was = MasGs.Link_Alive;
               Status(ETHER_LINK_STATE,MASTER_LINK_UP);
             }
        }
        else if (udpDsc->remoteIP[3u] == FirGs.IpAddr[3u])
        {
             FirGs.Link_Alive = TRUE;
             FirGs.Link_Alive_Time=Heart.Time;
             if(FirGs.Link_Alive_Was != FirGs.Link_Alive)
             {
                Status(ETHER_LINK_STATE,FIRE_LINK_UP);
             }
        }

      switch(udpDsc->destPort)                                                  // Test the destination port for the received UDP packet
      {
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
      // ====================== SimpleBGC message ============================== From STM32 Gimbal Controller
      
         case(SBGC_Dest_Port):                                                  // Port is for SimpleBGC UDP receive packet
         {
            if (SBGC_DATA.State==ETH_UDP_PACKET_PROCESSED)                      /* only if successfully processed reset the buffer to collect a new frame */
               SBGC_DATA.UDP_Index =CLEAR;                                      // Reset buffer index
            SBGC_DATA.UDP_Buffer_Len =  udpDsc->dataLength;

            while(udpDsc->dataLength--)                                         // Keep filling SBGC_DATA.UDP_Buffer until UDP receive buffer is empty
            {
               SBGC_DATA.UDP_Buffer[SBGC_DATA.UDP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
               SBGC_DATA.UDP_Index++ % UINT8_MAX;
               if ( SBGC_CMD_MAX_BYTES <= SBGC_DATA.UDP_Index )                 // counted past the end of the UDP receive buffer
               {
                  return(0u);                                                   // return with no reply message was longer than max expected for SimpleBGC
               }
            }
            SBGC_DATA.UDP_Index--;
            SBGC_DATA.State = ETH_UDP_PACKET_RECEIVED;                          // We have received a Packet from the gimbal, now let main process it
            UDP_CRC_Reg |= 0b10;                                                // Set UDP_UART2 CRC check needed bit
            UDP_CRC_Bad_Reg &= 0xFFFDu;                                         // Clear Bad bit
#if defined(REMOTE_UDP_SBGC)
            UART2_Write_Text(SBGC_DATA.UDP_Buffer);                             // remote write to serial over udp is chosen
#endif
            break;                                                              // Message was sent to us from the AIR COM PIC for SBGC
         }

      // ==================== SimpleBGC ACK ====================================  If required (most messages get confirm from STM32 controller so dont need ack from COM PIC but use if needed
      
         case(SBGC_Ack_Port):                                                   // Port is for SimpleBGC UDP ACK receive
         {
            T2CON = DISABLE_T2;                                                 // Stop time our
            if (SBGC_DATA.State==ETH_UDP_PACKET_PROCESSED)                      /* only if successfully processed reset the buffer to collect a new frame */
               SBGC_DATA.UDP_CRC_Index= CLEAR;                                  // Reset buffer index
            while(udpDsc->dataLength--)                                         // Keep filling SBGC_DATA.UDP_CRC buffer until UDP receive buffer is empty
            {
                SBGC_DATA.UDP_CRC[SBGC_DATA.UDP_CRC_Index] = Net_Ethernet_Intern_getByte();
                SBGC_DATA.UDP_CRC_Index++ % UINT8_MAX;
                if ( 5u <= SBGC_DATA.UDP_CRC_Index )                            // counted past the end of the UDP ACK receive buffer which is UDP_CRC[]
                {
                  return(0u);                                                   // return with no reply message was longer than max expected for SimpleBGC extra CRC
                }
            }
            SBGC_DATA.UDP_CRC_Index--;
            SBGC_DATA.UDP_Send_Trys =CLEAR;                                     // We received a good ACK clear the trys.
            SBGC_DATA.State = ETH_RECEIVED_ACK_CRC;
            //status(UDPSTA_Add,UDP_PACKET_ACK_RECEIVED);
            break;
         }
#endif
#if (CAMERA_TYPE == XS_CAM)
      // ================== XSStream messages ================================== From A.M.P. Camera Encoder
     
         case(XS_Dest_Port):                                                    // Port is for XStream encoder UDP receive packet
         {
            if (XS_DATA.State==ETH_UDP_PACKET_PROCESSED)                        /* only if successfully processed reset the buffer to collect a new frame */
               XS_DATA.UDP_Index =CLEAR;                                        // Reset buffer index
            XS_DATA.UDP_Buffer_Len =  udpDsc->dataLength;

            while(udpDsc->dataLength--)                                         // Keep filling XS_DATA.UDP_Buffer until UDP receive buffer is empty
            {
               XS_DATA.UDP_Buffer[XS_DATA.UDP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
               XS_DATA.UDP_Index++ % UINT8_MAX;
               if ( XS_CMD_MAX_BYTES <= XS_DATA.UDP_Index )                     // counted past the end of the UDP receive buffer
               {
                  return(0u);                                                   // return with no reply message was longer than max expected for XStream
               }                                                                // As we use a common UDP receive structure the max len is actually SimpleBGC
            }
            XS_DATA.UDP_Index--;
            XS_DATA.State = ETH_UDP_PACKET_RECEIVED;                            // We have received a Packet for UART2, now we need to test the CRC
            UDP_CRC_Reg |= 0b10;                                                // Set UDP_UART2 CRC check needed bit
            UDP_CRC_Bad_Reg &= 0xFFFDu;                                         // Clear Bad bit
#if defined(REMOTE_UDP_AMP_XY_CAM)
            UART4_Write_Text(XS_DATA.UDP_Buffer);                               // remote AMP XS encoder message over udp
#endif
            break;                                                              //case(UART2.From_Port)
         }
      
      // =============== XSStream ACK messages ================================= If required encoder usually replies with OK or AUTH so no need from COM PIC (leave if required)
      
         case(XS_Ack_Port):                                                     // Port is for XStream ACK CRC
         {
            //T2CON = DISABLE_T2;                                               // Stop time our
            if (XS_DATA.State==ETH_UDP_PACKET_PROCESSED)                        /* only if successfully processed reset the buffer to collect a new frame */
              XS_DATA.UDP_CRC_Index= CLEAR;                                     // Reset buffer index
            while(udpDsc->dataLength--)                                         // Keep filling UART2 UDP CRC burrer untill UDP buffer is empty
            {
               XS_DATA.UDP_CRC[XS_DATA.UDP_CRC_Index] = Net_Ethernet_Intern_getByte();
               XS_DATA.UDP_CRC_Index++ % UINT8_MAX;
               if ( 5u <= XS_DATA.UDP_CRC_Index )                               // counted past the end of the UDP ACK receive buffer which is UDP_CRC[]
               {
                  return(0u);                                                   // return with no reply message was longer than max expected for XStream
               }
            }
            XS_DATA.UDP_CRC_Index--;
            XS_DATA.UDP_Send_Trys =CLEAR;                                       // We received a good ACK clear the trys.
            XS_DATA.State = ETH_RECEIVED_ACK_CRC;
            Status(UDPSTA_Add,UDP_PACKET_ACK_RECEIVED);
            break;
         }
         //break;                                                                 //switch(udpDsc->destPort)
#endif
#if defined(RUN_CAM_USED)
      // ================== Run Cam messages ================================== From Run Cam camera

         case(RC_Dest_Port):                                                    // Port is for Run Cam camera UDP receive packet
         {
            if (RunC_DATA.State==ETH_UDP_PACKET_PROCESSED)                      /* only if successfully processed reset the buffer to collect a new frame */
               RunC_DATA.UDP_Index =CLEAR;                                      // Reset buffer index
            RunC_DATA.UDP_Buffer_Len =  udpDsc->dataLength;

            while(udpDsc->dataLength--)                                         // Keep filling RunC_DATA.UDP_Buffer until UDP receive buffer is empty
            {
               RunC_DATA.UDP_Buffer[RunC_DATA.UDP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
               RunC_DATA.UDP_Index++ % UINT8_MAX;
               if ( RC_CMD_MAX_BYTES <= RunC_DATA.UDP_Index )                   // counted past the end of the UDP receive buffer
               {
                  return(0u);                                                   // return with no reply message was longer than max expected for XStream
               }                                                                // As we use a common UDP receive structure the max len is actually SimpleBGC
            }
            RunC_DATA.UDP_Index--;
            RunC_DATA.State = ETH_UDP_PACKET_RECEIVED;                          // We have received a Packet for UART, now we need to test the CRC
            UDP_CRC_Reg |= 0b10;                                                // Set UDP_UART CRC check needed bit
            UDP_CRC_Bad_Reg &= 0xFFFDu;                                         // Clear Bad bit
#if defined(REMOTE_UDP_RUN_CAM)
            UART5_Write_Text(RunC_DATA.UDP_Buffer);                             // remote runCam message to serial over udp
#endif
            break;
         }
      // =============== Run Cam ACK messages ================================= If required camera usually has its own serial reply so not always needed here if you want to use

         case(RC_Ack_Port):                                                     // Port is for Run Cam ACK CRC
         {
            if (RunC_DATA.State==ETH_UDP_PACKET_PROCESSED)                      /* only if successfully processed reset the buffer to collect a new frame */
               RunC_DATA.UDP_CRC_Index= CLEAR;                                  // Reset buffer index
            while(udpDsc->dataLength--)                                         // Keep filling UART UDP CRC burrer untill UDP buffer is empty
            {
               RunC_DATA.UDP_CRC[RunC_DATA.UDP_CRC_Index] = Net_Ethernet_Intern_getByte();
               RunC_DATA.UDP_CRC_Index++ % UINT8_MAX;
               if ( 5u <= RunC_DATA.UDP_CRC_Index )                             // counted past the end of the UDP ACK receive buffer which is UDP_CRC[]
               {
                  return(0u);                                                   // return with no reply message was longer than max expected for XStream
               }
            }
            RunC_DATA.UDP_CRC_Index--;
            RunC_DATA.UDP_Send_Trys =CLEAR;                                     // We received a good ACK clear the trys.
            RunC_DATA.State = ETH_RECEIVED_ACK_CRC;
            Status(UDPSTA_Add,UDP_PACKET_ACK_RECEIVED);
            break;
         }
         break;
#endif
#ifdef GEF_EGD_PLC
         case(GEF_EGD_UDP_DATA_PORT):                                           // Port is reading EGD Protocol
         {
            if (EGD_DATA.State==ETH_UDP_PACKET_PROCESSED)                       /* only if successfully processed reset the buffer to collect a new frame */
               EGD_DATA.UDP_Index =CLEAR;                                       // Reset buffer index
            EGD_DATA.UDP_Buffer_Len =  udpDsc->dataLength;

            while(udpDsc->dataLength--)                                         // Keep filling EGD_DATA.UDP_Buffer until UDP receive buffer is empty
            {
               EGD_DATA.UDP_Buffer[EGD_DATA.UDP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
               EGD_DATA.UDP_Index++ % UINT8_MAX;
               if ( GEF_EGD_UDP_MAX_LEN <= EGD_DATA.UDP_Index )                 // counted past the end of the UDP receive buffer
               {
                  return(0u);                                                   // return with no reply message was longer than max expected for EGD
               }                                                                //
            }
            EGD_DATA.UDP_Index--;
            EGD_DATA.State = ETH_UDP_PACKET_RECEIVED;                           // We have received a Packet for UART, now we need to test the CRC
            UDP_CRC_Reg |= 0b10;                                                // Set UDP_UART CRC check needed bit
            UDP_CRC_Bad_Reg &= 0xFFFDu;                                         // Clear Bad bit
            break;
         }
#endif
#ifdef ART_NET_USED
         case(ARTNET4_UDP_PORT):                                                // Port is reading ArtNet-4 Protocol
         {
            if (artNet_DATA.State==ETH_UDP_PACKET_PROCESSED)                    /* only if successfully processed reset the buffer to collect a new frame */
               artNet_DATA.UDP_Index =CLEAR;                                    // Reset buffer index
            artNet_DATA.UDP_Buffer_Len =  udpDsc->dataLength;

            while(udpDsc->dataLength--)                                         // Keep filling artNet_DATA.UDP_Buffer until UDP receive buffer is empty
            {
               artNet_DATA.UDP_Buffer[artNet_DATA.UDP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
               artNet_DATA.UDP_Index++ % UINT8_MAX;
               if ( ARTNET4_MAX_LEN <= artNet_DATA.UDP_Index )                  // counted past the end of the UDP receive buffer
               {
                  return(0u);                                                   // return with no reply message was longer than max expected for Artnet4
               }                                                                //
            }
            artNet_DATA.UDP_Index--;
            artNet_DATA.State = ETH_WAIT_ACK_CRC;                               // We have received a Packet for UART, now we need to test the CRC
            UDP_CRC_Reg |= 0b10;                                                // Set UDP_UART CRC check needed bit
            UDP_CRC_Bad_Reg &= 0xFFFDu;                                         // Clear Bad bit
            break;
         }
#endif
#ifdef SMPTE_USED
         case(SMPTE_UDP_PORT):                                                  // Port is reading SMPTE over UDP Protocol
         {
            if (SMPTE_DATA.State==ETH_UDP_PACKET_PROCESSED)                     /* only if successfully processed reset the buffer to collect a new frame */
               SMPTE_DATA.UDP_Index =CLEAR;                                     // Reset buffer index
            SMPTE_DATA.UDP_Buffer_Len =  udpDsc->dataLength;

            while(udpDsc->dataLength--)                                         // Keep filling SMPTE_DATA.UDP_Buffer until UDP receive buffer is empty
            {
               SMPTE_DATA.UDP_Buffer[SMPTE_DATA.UDP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
               SMPTE_DATA.UDP_Index=++SMPTE_DATA.UDP_Index % UINT8_MAX;
               if ( SMPTE_CMD_MAX_BYTES <= SMPTE_DATA.UDP_Index )               // counted past the end of the UDP receive buffer (carrys SMPTE data)
               {
                  return(0u);                                                   // return with no reply message was longer than max expected for XStream
               }                                                                // As we use a common UDP receive structure the max len is actually SimpleBGC
            }
            SMPTE_DATA.UDP_Index--;
            SMPTE_DATA.State = ETH_WAIT_ACK_CRC;                                // We have received a Packet for UART, now we need to test the CRC
            UDP_CRC_Reg |= 0b10;                                                // Set UDP_UART CRC check needed bit
            UDP_CRC_Bad_Reg &= 0xFFFDu;                                         // Clear Bad bit
            break;
         }
#endif
#ifdef ASN_E1_USED
         case(ACN_SDT_MULTICAST_PORT):                                          // Port is reading ASN E1 (ACN) Protocol
         {
            if (ASN_E1_DATA.State==ETH_UDP_PACKET_PROCESSED)                    /* only if successfully processed reset the buffer to collect a new frame */
               ASN_E1_DATA.UDP_Index =CLEAR;                                    // Reset buffer index
            ASN_E1_DATA.UDP_Buffer_Len =  udpDsc->dataLength;

            while(udpDsc->dataLength--)                                         // Keep filling ASN_E1_DATA.UDP_Buffer until UDP receive buffer is empty
            {
               ASN_E1_DATA.UDP_Buffer[ASN_E1_DATA.UDP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
               ASN_E1_DATA.UDP_Index++ % UINT8_MAX;
               if ( ARTNET4_MAX_LEN <= ASN_E1_DATA.UDP_Index )                  // counted past the end of the UDP receive buffer (carrys DMX512 as does Artnet)
               {
                  return(0u);                                                   // return with no reply message was longer than max expected for XStream
               }                                                                // As we use a common UDP receive structure the max len is actually SimpleBGC
            }
            ASN_E1_DATA.UDP_Index--;
            ASN_E1_DATA.State = ETH_WAIT_ACK_CRC;                               // We have received a Packet for UART, now we need to test the CRC
            UDP_CRC_Reg |= 0b10;                                                // Set UDP_UART CRC check needed bit
            UDP_CRC_Bad_Reg &= 0xFFFDu;                                         // Clear Bad bit
            break;
         }
#endif
#ifdef CBOR_COMS_USED
         case(CBOR_UDP_PORT):                                                  // Port using coAP Protocol (iot)
         {
            if (CBOR_DATA.State==ETH_UDP_PACKET_PROCESSED)                      /* only if successfully processed reset the buffer to collect a new frame */
               CBOR_DATA.UDP_Index =CLEAR;                                      // Reset buffer index
            CBOR_DATA.UDP_Buffer_Len =  udpDsc->dataLength;

            while(udpDsc->dataLength--)                                         // Keep filling CBOR_DATA.UDP_Buffer until UDP receive buffer is empty
            {
               CBOR_DATA.UDP_Buffer[CBOR_DATA.UDP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
               CBOR_DATA.UDP_Index++ % UINT8_MAX;
               if ( CoAP_CMD_MAX_BYTES <= CBOR_DATA.UDP_Index )                 // counted past the end of the UDP receive buffer
               {
                  return(0u);                                                   // return with no reply
               }                                                                //
            }
            CBOR_DATA.UDP_Index--;
            CBOR_DATA.State = ETH_WAIT_ACK_CRC;                                 // We have received a Packet for UART, now we need to test the CRC
            UDP_CRC_Reg |= 0b10;                                                // Set UDP_UART CRC check needed bit
            UDP_CRC_Bad_Reg &= 0xFFFDu;                                         // Clear Bad bit
            break;
         }
#endif
#if (defined(USE_MAVLINK) && !defined(SERIAL_MAVLINK))
         case(MAV_STA_UDP_DATA_PORT):                                           // Port is reading UDP Mavlink Protocol MavLinkBuf.Dest_Port
         {
            if (MavLinkBuf.State==ETH_UDP_PACKET_PROCESSED)                       /* only if successfully processed reset the buffer to collect a new frame */
               MavLinkBuf.UDP_Index =CLEAR;                                       // Reset buffer index
            MavLinkBuf.UDP_Buffer_Len =  udpDsc->dataLength;

            while(udpDsc->dataLength--)                                         // Keep filling EGD_DATA.UDP_Buffer until UDP receive buffer is empty
            {
               MavLinkBuf.Buffer[MavLinkBuf.UDP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
#if defined(MAV_FILTER_INCLUDE)                                                 /* include the mavlink reader filter checks CRC etc */
#include "mav_uart_driver_pic32_common.h"                                       /* not misra include here once testing complete */
#else
               MavLinkBuf.UDP_Index=++MavLinkBuf.UDP_Index % UINT8_MAX;
#endif /* end of mav filter */
               if ( MAV_MAX_MSG_LEN <= MavLinkBuf.UDP_Index )                 // counted past the end of the UDP receive buffer
               {
                  return(0u);                                                   // return with no reply message was longer than max expected for EGD
               }                                                                //
            }
            MavLinkBuf.UDP_Index--;
            MavLinkBuf.State = ETH_UDP_PACKET_RECEIVED;                           // We have received a Packet for UART, now we need to test the CRC
            break;
         }
#endif
     }

#ifdef USE_CAN
     // Process UDP packets for CAN
     if (udpDsc->destPort==Can2fromPort)
     {
        // Load first two bytes into message ID
        Lo(Can2_TX_MSG_ID)=Net_Ethernet_Intern_getByte();
        Hi(Can2_TX_MSG_ID)=Net_Ethernet_Intern_getByte();

        // load the remaining of the message into Can2_UDP_RX_Packet EXLUDING the MASK which is the last byte in the UDP Packet.

        reqLength = udpDsc->dataLength;
        while(counter < reqLength)
        {
           Can2_UDP_RX_Packet[counter] = Net_Ethernet_Intern_getByte();
           counter=++counter % UINT16_MAX;
        }
        counter=0u;                                                             // reset counter
        sendValue=0u;
        // Sent message out CAN2
        while((!sendValue)&&(counter < CAN_RESEND_CNT))                         // !!!!!! ============ consider separate can handler too busy reset no while but if
        {
           sendValue=CAN2Write(Can2_TX_MSG_ID,(unsigned char*) Can2_UDP_RX_Packet,udpDsc->dataLength ,Can2_Send_Flags);
           counter=++counter % UINT16_MAX;
           asm nop;
        }
     }
#endif
   }
   return(0u);                                                                  // back to the library with the length to reply with (in this case nothing)
 }
 
/*-----------------------------------------------------------------------------
 *      Pay_checksum:  Calculate the payload chacksum for SBGC message
 *
 *  Parameters: unsigned char *pdata, uint16_t sz
 *  Return:     char checksum
 *----------------------------------------------------------------------------*/
uint8_t Pay_checksum(const unsigned char *pdata, uint16_t sz)                   // A Function to calculate a checksum for a struct
{
  uint16_t i;
  uint64_t checksum;
  uint16_t checkCalc=0u;
  
  if (pdata==NULL)                                                              /* NULL ponter passed then return a zero */
  {
    for(i=0, checksum=0; i<sz; i++)                                             /* for each bytein the message */
      checksum+=pdata[i];                                                       /* sum the bytes */
    checkCalc= (uint8_t) (checksum % 256u);                                    /* reduce the sum to a single BYTE to return as the checksum */
  }
  return checkCalc;
}
/*-----------------------------------------------------------------------------
 *      extCalls_checksum:  Calculate the payload chacksum for external calls message
 *
 *
 *  Data checksum is cumulative each byte of data, use the 16bit (2 bytes) unsigned number to represent,
 *  so when the data validation is more than 0xFFFF, the checksum, and retain only 16bit value.
 *  For example, 0xFFFA + 0x09 = 0x0003.
 *
 *  Parameters: unsigned char *pdata, uint16_t sz
 *  Return:     uint16_t checksum
 *----------------------------------------------------------------------------*/
uint16_t XtCalls_checksum1(const unsigned char *pdata, uint16_t sz)               // A Function to calculate a checksum for a struct
{
  uint16_t i;
  uint16_t checksum;                                                            /* this is limited to 16 bit as described in external calls manual */
  uint16_t checkCalc=0u;

  if (pdata==NULL)                                                              /* NULL ponter passed then return a zero */
  {
    for(i=0, checksum=0; i<sz; i++)                                             /* for each bytein the message */
      checksum= (uint16_t) ((checksum + pdata[i]) % 256u);                      /* sum the bytes in a 16bit rollover style */
    checkCalc= checksum;                                                        /* now just send it */
  }
  return checkCalc;
}
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
 /*
 * Function Name:
 *   Test_CRC()
 * Description:
 *   Tests incoming UDP ACK CRC packets
 * Arguments:
 *   None
 * Returns:
 *   None
 * NOTES:
 *   <notes>
 */
void Test_CRC()
{
  uint16_t crc_test;
  if (UDP_CRC_Reg & 0b10u)                                                      // do we have a UDP_UART2 Packet CRC to Test?
  {
   crc_test = CRC16_Calc(SBGC_DATA.UDP_Buffer,SBGC_DATA.UDP_Buffer_Len-2u);

   if ((Hi(crc_test) == SBGC_DATA.UDP_Buffer[SBGC_DATA.UDP_Buffer_Len-2u]) && (Lo(crc_test) == SBGC_DATA.UDP_Buffer[SBGC_DATA.UDP_Buffer_Len-1u]))
      {
       // CRC was good
       SBGC_DATA.UDP_CRC[0u] = 0x06u;                                           // ACK
       SBGC_DATA.UDP_CRC[1u] = Hi(crc_test);                                    // ACK
       SBGC_DATA.UDP_CRC[2u] = Lo(crc_test);                                    // ACK
       UDP_CRC_Good_Reg |= 0b10u;                                                //Set good CRC
       UDP_CRC_Reg      &= 0xFFFDu;                                             // Clear CRC
      }
      else
      {
       //CRC was bad
       UDP_CRC_Good_Reg &= 0xFFFDu;                                             // Clear good CRC
       UDP_CRC_Bad_Reg  |= 0b10u;                                                // Set bad CRC
       UDP_CRC_Reg      &= 0xFFFDu;                                             // Clear CRC
       //Status(UDPSTA_Add,UDP_UART2_Packet_BAD_CRC);
      }
  }
}

 // void do_UDP_to_UART2()
 // {
   
 //  UART2.UDP_Buffer[UART2.UDP_Buffer_Len-2] =0; //Delete the UDP CRC
 //  UART2_write_text(UART2.UDP_Buffer);
 //  Status(UDPSTA_Add,UDP_UART2_Packet_RECEIVED);
   //Send an ACK

   
 //  Net_Ethernet_Intern_sendUDP(UART2.RemoteIpAddr, UART2.ACK_Port, UART2.ACK_Port, UART2.UDP_CRC,3);
 //  UDP_CRC_Good_Reg &= 0xFFFD; // Clear CRC
 //  Heart.Enabled =TRUE; // Re-enable heart beat
 // }

//================ SBGC GIMBAL READ ============================================
//
//==============================================================================
/*******************************************************************************
* Function Name: ReadSBGC
********************************************************************************
* Summary:
*  ReadSBGC function performs following functions:
*   1. Reads the UDP buffer data and checks for it being a simpleBGC message
*
*
* Parameters:
*  unsigned char *pdata, uint16_t psz, uint16_t arrPosition
*  pdata = UDP data, psz=size, arrPosition=starting position of message
*
* Return:
* Byte representing the type of SimpleBGC message found or SBGC_NO_CMD_FOUND
*
*******************************************************************************/
uint8_t ReadSBGC(const unsigned char *pdata, uint16_t psz, uint16_t arrPosition)      // Read the data portion of the UDP datagram and get SBGC data.
{
  uint8_t start=0u;
  uint8_t i;
  uint16_t cmd_state=SBGC_NO_CMD_FOUND;                                         // Initialise the command state and set to the one found once STX has been seen.
  //unsigned int paylen;                                                          // payload length
  //unsigned int nexthead=0;                                                      // next head byte in header
  
  //unsigned char headchksum;                                                     // header checksum
  //unsigned char chksum_calc;                                                    // calculated checksum

  if (pdata == NULL)
    return 0u;
    
  arrPosition=0u;                                                               // reset the position index to the start at the begining of the message
  
  for (i = 0u; i < psz ; i++)                                                   // For each char in the buffer
  {
    if (!start)                                                                 // if we dont already have a 0XAA start byte for index 0
    {
      if(pdata[i] == g_SBGC_Stx_Char)                                           // Look for the STX being SimpleBGC
      {
       start =1u;                                                               // We got the start so get the comamnd
       arrPosition=i;                                                           // return the start position so we strip leading 0 or F if needed.
      }
    }
    else                                                                        // You saw the simpleBGC start so look at the next byte and check if the cmd is expected.
    {
      switch(pdata[arrPosition+1u])                                             // For the next byte after the STX char has been discovered in the Packet
      {
        case SBGC_CMD_GET_ANGLES:                                               // command was a SBGC get Angles
          return (SBGC_CMD_GET_ANGLES);
          break;
        case SBGC_CMD_AUTO_PID:                                                 // command was a SBGC auto pid response
          return (SBGC_CMD_AUTO_PID);
          break;
        case SBGC_CMD_GET_ANGLES_EXT:                                           // Command get angles ext
          return (SBGC_CMD_GET_ANGLES_EXT);
          break;
        case SBGC_CMD_REALTIME_DATA_3:                                          // command was a SBGC realtime data.
          return (SBGC_CMD_REALTIME_DATA_3);
          break;
        case SBGC_CMD_REALTIME_DATA_4:
          return (SBGC_CMD_REALTIME_DATA_4);
          break;
        case SBGC_CMD_CONFIRM:                                                  // command was a SBGC confirm
          return (SBGC_CMD_CONFIRM);
          break;
        case SBGC_CMD_ERROR:                                                    // command was a SBGC error response
          return (SBGC_CMD_ERROR);
          break;
        case SBGC_CMD_I2C_READ_REG_BUF:                                         // i2c read message   (not used atm)
          return (SBGC_CMD_I2C_READ_REG_BUF);
          break;
        case SBGC_CMD_SET_ADJ_VARS_VAL:                                         // set adjustable vars vals message
          return (SBGC_CMD_SET_ADJ_VARS_VAL);
          break;
        case SBGC_CMD_READ_ADJ_VARS_CFG:                                        // read adjustable vars config message
          return (SBGC_CMD_READ_ADJ_VARS_CFG);
          break;
        case SBGC_CMD_CALIB_INFO:                                               // calibration information command
          return (SBGC_CMD_CALIB_INFO);
          break;
        case SBGC_CMD_REALTIME_DATA_CUSTOM:                                     // Realtime data message (custom)
          return(SBGC_CMD_REALTIME_DATA_CUSTOM);
          break;
        case SBGC_CMD_EVENT:                                                    // Command Event
          return(SBGC_CMD_EVENT);
          break;
        case SBGC_CMD_READ_PARAMS_3:                                            // command was pide request or config read/write part1
          return(SBGC_CMD_READ_PARAMS_3);
          break;
        case SBGC_CMD_BOARD_INFO:                                              // command was request for information on the firmware/state ofthe board
          return(SBGC_CMD_BOARD_INFO);
          break;
        case SBGC_CMD_RESET:                                                    // Reset or
          return(SBGC_CMD_RESET);
          break;
        case SBGC_CMD_I2C_READ_REG_BUF:                                         // i2c read (EEPROM OR IMU)
          return (SBGC_CMD_I2C_READ_REG_BUF);
          break;
        case SBGC_CMD_READ_EXTERNAL_DATA:                                       // Read external data from EEPROM
          return(SBGC_CMD_READ_EXTERNAL_DATA);
          break;
        case SBGC_CMD_SCRIPT_DEBUG:                                             // return from running the script
          return(SBGC_CMD_SCRIPT_DEBUG);
          break;
        default:                                                                // Nothing matched the support.
          return (cmd_state);
          break;
      }
    }
    return (cmd_state);                                                         // return the recognised supported command state.
  }
}
// ======================= SBGC PAYLOAD CHECK ==================================
/*******************************************************************************
* Function Name: check_payload
********************************************************************************
* Summary:
*  check_payload function performs following functions:
*   1. Checks the CRC for a simpleBGC message
*
*
* Parameters:
*  unsigned int buffStartPos - start pos in the buffer
*  SBGC_cmd_board_info_t *boardinf - board revision info
*
* Return:
* 1 if okay 0 if not
*
*******************************************************************************/
uint8_t check_payload(uint16_t buffStartPos, const SBGC_cmd_board_info_t *boardinf )
{
   unsigned char *ptr_udp = (unsigned char *) &SBGC_DATA.UDP_Buffer;            // Define the pointer to the container struct
   uint16_t sz_udp=sizeof(SBGC_DATA.UDP_Buffer);                                // Size of the udp payload
   unsigned char paychksum;                                                     // payload checksum sent
   unsigned char calcCRC;                                                       // calculated payload CRC
   uint16_t paylength;                                                          // payload length
   uint16_t payStartLen;                                                        // Paylaod start length
   uint16_t payStartPos;                                                        // Payload Start position
   uint16_t newpayCRC;                                                          // new payload checksum
   uint16_t newCRC;                                                             // 16 bit CRC

   if (boardinf == NULL)
     return 0u;
     
   payStartLen=SBGC_PAYLOAD_LENGTH_POS+buffStartPos;                            // location in buffer of payload length
   payStartPos=SBGC_LEN_OF_HEADER+buffStartPos;                                 // start location in buffer of payload
   
   paylength= (uint16_t) SBGC_DATA.UDP_Buffer[payStartLen];                     // Read the SBGC payload message length from the SGBC Packet in the data segment of the UDP datagram
   if (sz_udp >= (paylength+SBGC_LEN_OF_HEADER))                                // check we have a full message.
   {
      if (boardinf->FIRMWARE_VER >= 2680U)                                      // New version of code is now a 16 bit checksum
      {
          newpayCRC=((SBGC_DATA.UDP_Buffer[paylength+SBGC_LEN_OF_HEADER-1u]<<8u) | SBGC_DATA.UDP_Buffer[paylength+SBGC_LEN_OF_HEADER]);  // read the SBGC payload crc from the UDP data segment
          newCRC = crc16_arc_calculate((sz_udp-SBGC_LEN_OF_HEADER),(ptr_udp+payStartPos));               // Calculate new 16 bit CRC for new version
          if (newpayCRC == newCRC)                                              // They match
          {
            return(1u);                                                         // return payload CRC okay.
          }
      }
      else
      {
         paychksum=SBGC_DATA.UDP_Buffer[paylength+SBGC_LEN_OF_HEADER];          // read the SBGC payload crc from the UDP data segment
         calcCRC = Pay_checksum((ptr_udp+payStartPos),(sz_udp-SBGC_LEN_OF_HEADER));      // calculate the payload checksum from the data in the UDP data segment
         if (paychksum == calcCRC)                                              // They match
         {
            return(1u);                                                         // return payload CRC okay.
         }
      }
   }
   return (0u);                                                                 // return error (the CRC in UDPdata and that calc from payload didnt match).
}
// ===================== SBGC GIMBAL MESSAGE ===================================
//
/*******************************************************************************
* Function Name: SBGC_process_UDP_InBuffer
********************************************************************************
* Summary:
*  Process the SGBC message on the relevant UDP port
*
* Parameters:
*  SBGC_cmd_board_info_t *boardinf
*
* Return:
* uint8_t  returns a code to say the type of message
*
*******************************************************************************/
uint8_t SBGC_process_UDP_InBuffer( const SBGC_cmd_board_info_t *boardinf )      // Process the in buffer from Gimbal and decide what it means
{
   uint16_t msgCmdType;                                                         // The Cmd Type found in the SBGC Packet

   uint16_t msgStart=0u;                                                        // pointer in the buffer to a STX in message
   uint16_t startPayPos;                                                        // start position of the payload in the UDP data buffer

   if (boardinf == NULL)
     return 0u;
     
   SBGC_DATA.UDP_Buffer[SBGC_DATA.UDP_Buffer_Len-2u] =0u;                       // Delete the UDP CRC

   msgCmdType=ReadSBGC(&SBGC_DATA.UDP_Buffer,sizeof(SBGC_DATA.UDP_Buffer),msgStart);    // Read the buffer to verify type and return start location
   
   if ((check_payload(msgStart,boardinf)==1u)&&(msgCmdType!=SBGC_NO_CMD_FOUND)) // Check the CRC payload is correct before copying also chk type supported.
   {
      startPayPos=msgStart+SBGC_START_PAYLOAD_POS;                              // go to the start position for payload.
      switch(msgCmdType)                                                        // For the state of the switches from the Commander GS
      {
        case SBGC_CMD_GET_ANGLES:                                               // command was a SBGC get Angles
          memcpy(&getangles, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(getangles));  // Copy the payload to the get angles readback message container.
          break;
        case SBGC_CMD_AUTO_PID:                                                 // command was a SBGC auto pid response
          memcpy(&pidreadbk, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(pidreadbk));  // Copy the payload to the pid readback message container.
          break;
        case SBGC_CMD_GET_ANGLES_EXT:                                           // Command get angles ext
          memcpy(&getanglesext, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(getanglesext));    // Copy the payload to the get ext angles readback message container.
          break;
        case SBGC_CMD_REALTIME_DATA_3:                                          // command was a SBGC realtime data.
          memcpy(&realdata3, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(realdata3));    // Copy the payload to the realtime data 3 readback message container.
          break;
        case SBGC_CMD_REALTIME_DATA_4:
          memcpy(&realdata4, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(realdata4));    // Copy the payload to the realtime data 4 readback message container.
          break;
        case SBGC_CMD_CONFIRM:                                                  // command was a SBGC confirm
          memcpy(&cmdconf, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(cmdconf));        // Copy the payload to the command confirmation readback message container.
          break;
        case SBGC_CMD_ERROR:                                                    // command was a SBGC error response
          memcpy(&cmderror, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(cmderror));      // Copy the payload to the command confirmation readback message container.
          break;
        case SBGC_CMD_SET_ADJ_VARS_VAL:                                         // command was a SBGC set adj vars val of 7 in response to our get adj vars val
          memcpy(&getvar_rd, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(getvar_rd));     // Copy the payload to the command confirmation readback message container.
          break;
        case SBGC_CMD_BOARD_INFO:                                               // command was request for information on the firmware/state ofthe board
          memcpy(&boardinforb, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(boardinforb));
          break;
        case SBGC_CMD_READ_PARAMS_3:                                            // command was request for pide tune or config part 1 read/write
          memcpy(&readparam3, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(readparam3));
          break;
        case SBGC_CMD_RESET:                                                    // Reset issued feedback response
          break;
        case SBGC_CMD_I2C_READ_REG_BUF:                                         // i2c EEPROM or IMU read
          break;
        case SBGC_CMD_READ_EXTERNAL_DATA:                                       // external EEPROM read
          break;
        case SBGC_CMD_SCRIPT_DEBUG:                                             // start / stop script feedback
          memcpy(&scriptstate, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(scriptstate));
          break;
        case SBGC_CMD_EVENT:                                                    // motor state change event
          memcpy(&eventCmd, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(eventCmd));
          break;
        case SBGC_CMD_REALTIME_DATA_CUSTOM:                                     // realtime data custom frame readback
          memcpy(&realtimedatacust, &SBGC_DATA.UDP_Buffer[startPayPos], sizeof(realtimedatacust));
          break;
      }
    }
    
    if (msgCmdType!=SBGC_NO_CMD_FOUND)                                          // Message was recognised okay
    {
       //Status(UDPSTA_Add,UDP_UART2_Packet_RECEIVED);

       //Send an ACK to the COM PIC (May not be needed)
       //
       Net_Ethernet_Intern_sendUDP(SBGC_DATA.RemoteIpAddr, SBGC_Ack_Port, SBGC_Ack_Port, SBGC_DATA.UDP_CRC, 3u);
       UDP_CRC_Good_Reg &= 0xFFFDu;                                             // Clear CRC
       Heart.Enabled =TRUE;                                                     // Re-enable heart beat
   }
   return (msgCmdType);                                                         // return a code to say the type of message.
}
#endif                                                                          // ================ eNd simpleBGC protocol ==============================
#ifdef RUN_CAM_USED
/*******************************************************************************
* Function Name: ReadRunCam
********************************************************************************
* Summary:
*  Process the Run Cam message on the relevant UDP port
*
* Parameters:
*  unsigned char *pdata, uint16_t psz
*
* Return:
* uint8_t  returns a code to say the type of message or failure
*
*******************************************************************************/
uint8_t ReadRunCam( const unsigned char *pdata, uint16_t psz )                  // Read the data portion of the UDP datagram and get Run Cam data.
{
  uint8_t start=0u;
  uint8_t i;
  uint16_t cmd_state=RUN_CAM_NO_CMD_FOUND;                                      // Initialise the command state and set to the one found once STX has been seen.
  uint16_t arrPosition=0u;                                                      // position of the STX in the UDP message data segment

  if (pdata == NULL)
    return 0u;
    
  if (g_RunCamSent==RC_PROTO_CMD_READ_SETTING_DETAIL)                           // Setting detail doesnt seem to have a STX so handle here as well
  {
     if (strlen((char*)pdata) == sizeof(RCCamRepReadSetup))
     {
        memcpy((void*) &RCCamRepReadSetup,(void*) pdata,(int16_t) sizeof(RCCamRepReadSetup));
        return(RC_PROTO_CMD_READ_SETTING_DETAIL);
     }
     else if (strlen((char*)pdata) < sizeof(RCCamRepReadSetup))
     {
        return(RC_MSG_INCOMP);
     }
     else
     {
        return(RC_MSG_RESET);                                                   // message too long reset the buffer
     }
  }
  else if (g_RunCamSent==RC_PROTO_CMD_GET_SETTINGS)                             // Setting detail doesnt seem to have a STX so handle here as well
  {
      if (strlen((char*)pdata) == sizeof(RCGetSettings))
      {
        memcpy((void*)&RCGetSettings,(void*) pdata,(int16_t) sizeof(RCGetSettings));
      }
      else if (strlen((char*)pdata) < sizeof(RCGetSettings))
      {
        return(RC_MSG_INCOMP);
      }
      else
      {
        return(RC_MSG_RESET);                                                   // message too long reset the buffer
      }
  }
  else
  {
    for (i = 0u; i < psz ; i++)                                                 // For each char in the buffer if we dont already have a 0XAA start byte for index 0 (setting details have no STX)
    {
      if ((pdata[i] == RC_STX_CHAR) && (start==0u))                             // Look for the STX being Run Cam protocol
      {
        start = 1u;                                                             // We got the start so get the comamnd
        arrPosition=i;                                                          // return the start position so we strip leading 0 or F if needed.
        i=psz+1u;                                                               // exit loop
      }
    }

    if (start==1u)
    {
      switch(g_RunCamSent)                                                      // Global send for Run Cam it doesnt reply with the message id
      {
          case RC_PROTO_CMD_GET_DEVICE_INFO:                                    // 0x00U INFO
          if (strlen((char*)(pdata+arrPosition)) == sizeof(RCCamRepInfo))
          {
             memcpy((void*)&RCCamRepInfo,(void*) pdata+arrPosition,(int16_t) sizeof(RCCamRepInfo));
             return (RC_PROTO_CMD_GET_DEVICE_INFO);
          }
          else if (strlen((char*)(pdata+arrPosition)) < sizeof(RCCamRepInfo))
             return(RC_MSG_INCOMP);
          else
             return(RC_MSG_RESET);
          break;

          case RC_PROTO_CMD_CAMERA_CONTROL:                                     // 0x01U CONTROL REQUEST (no reply ?)
          return (RC_PROTO_CMD_CAMERA_CONTROL);
          break;

          case RC_PROTO_CMD_5KEY_SIMULATION_PRESS:                              // 0x02U Key pad press
          if (strlen((char*)(pdata+arrPosition)) == sizeof(RCCamKeyConfirm))
          {
             memcpy((void*)&RCCamKeyConfirm,(void*) (pdata+arrPosition),(int16_t) sizeof(RCCamKeyConfirm));
             return(RC_PROTO_CMD_5KEY_SIMULATION_PRESS);
          }
          else if (strlen((char*)(pdata+arrPosition)) < sizeof(RCCamKeyConfirm))
             return(RC_MSG_INCOMP);
          else
             return(RC_MSG_RESET);
          break;

          case RC_PROTO_CMD_5KEY_SIMULATION_RELEASE:                            // 0x03U Key pad release
          if (strlen((char*)(pdata+arrPosition)) == sizeof(RCCamKeyConfirm))
          {
            memcpy((void*)&RCCamKeyConfirm,(void*) pdata+arrPosition,(int16_t) sizeof(RCCamKeyConfirm));
            return(RC_PROTO_CMD_5KEY_SIMULATION_RELEASE);
          }
          else if (strlen((char*)(pdata+arrPosition)) < sizeof(RCCamKeyConfirm))
             return(RC_MSG_INCOMP);
          else
             return(RC_MSG_RESET);
          break;

          case RC_PROTO_CMD_5KEY_CONNECTION:                                    // 0x04U Connection to camera
          if (strlen((char*)(pdata+arrPosition)) == sizeof(RCCamHandshakeConfirm))
          {
             memcpy((void*)&RCCamHandshakeConfirm,(void*) pdata+arrPosition,(int16_t) sizeof(RCCamHandshakeConfirm));
             return(RC_PROTO_CMD_5KEY_CONNECTION);
          }
          else if (strlen((char*)(pdata+arrPosition)) < sizeof(RCCamHandshakeConfirm))
             return(RC_MSG_INCOMP);
          else
             return(RC_MSG_RESET);
          break;

          case RC_PROTO_CMD_GET_SETTINGS:                                       // 0x10U Sub settings
          if (strlen((char*)(pdata+arrPosition)) == sizeof(RCCamHandshakeConfirm))
          {
             memcpy((void*)&RCCamHandshakeConfirm,(void*) pdata+arrPosition,(int16_t) sizeof(RCCamHandshakeConfirm));
             return(RC_PROTO_CMD_GET_SETTINGS);
          }
          else if (strlen((char*)(pdata+arrPosition)) < sizeof(RCCamHandshakeConfirm))
             return(RC_MSG_INCOMP);
          else
             return(RC_MSG_RESET);
          break;

          case RC_PROTO_CMD_READ_SETTING_DETAIL:                                // 0x11U read
          if (strlen((char*)(pdata+arrPosition)) == sizeof(RCGetSettings))
          {
             memcpy((void*)&RCGetSettings,(void*) pdata+arrPosition,(int16_t) sizeof(RCGetSettings));
             return(RC_PROTO_CMD_READ_SETTING_DETAIL);
          }
          if (strlen((char*)(pdata+arrPosition)) < sizeof(RCGetSettings))
             return(RC_MSG_INCOMP);
          else
             return(RC_MSG_RESET);
          break;

          case RC_PROTO_CMD_WRITE_SETTING:                                      // 0x12U write
          if (strlen((char*)(pdata+arrPosition)) == sizeof(RCCamRepWriteSetup))
          {
             memcpy((void*)&RCCamRepWriteSetup,(void*) pdata+arrPosition,(int16_t) sizeof(RCCamRepWriteSetup));
             return(RC_PROTO_CMD_WRITE_SETTING);
          }
          else if (strlen((char*)(pdata+arrPosition)) < sizeof(RCCamRepWriteSetup))
             return(RC_MSG_INCOMP);
          else
             return(RC_MSG_RESET);
          break;

          case RC_PROTO_CMD_DISP_FILL_REGION:
                  return(RC_PROTO_CMD_DISP_FILL_REGION);
          break;

          case RC_PROTO_CMD_DISP_WRITE_CHAR:
                  return(RC_PROTO_CMD_DISP_WRITE_CHAR);
          break;

          case RC_PROTO_CMD_DISP_WRITE_HORT_STRING:
                  return(RC_PROTO_CMD_DISP_WRITE_HORT_STRING);
          break;

          case RC_PROTO_CMD_DISP_WRITE_VERT_STRING:
                  return(RC_PROTO_CMD_DISP_WRITE_VERT_STRING);
          break;

          case RC_PROTO_CMD_DISP_WRITE_CHARS:
                  return(RC_PROTO_CMD_DISP_WRITE_CHARS);
          break;

          default:
          return(RC_MSG_UNKNO);                                                  // Incorrect message was sent already
          break;
      }
        }
  }
  return(RC_MSG_FAIL);                                                          // Bad Run Cam message no STX end message partial then trash it
}
#endif                                                                          // =============== eNd Run Cam ===================================
#if defined(GEF_EGD_PLC)
/*******************************************************************************
* Function Name: ReadEGD
********************************************************************************
* Summary:
*  Process the EGD message on the relevant UDP port
*
* Parameters:
*  unsigned char *pdata, uint16_t psz
*
* Return:
* uint8_t  returns a code to say the type of message or failure
*
*******************************************************************************/
uint8_t ReadEGD( const unsigned char *pdata, uint16_t psz )                        // Read the data portion of the UDP datagram and get EGD data.
{
   if ( pdata == NULL)
     return 0u;
   return 1u;
}
#endif                                                                          // =============== eNd EGD protcol ===================================
#if defined(ART_NET_USED)
/*******************************************************************************
* Function Name: ReadArtNet
********************************************************************************
* Summary:
*  Process the ArtNet message on the relevant UDP port
*
* Parameters:
*  unsigned char *pdata, uint16_t psz
*
* Return:
* uint8_t  returns a code to say the type of message or failure
*
*******************************************************************************/
uint8_t ReadArtNet( const unsigned char *pdata, uint16_t psz )                        // Read the data portion of the UDP datagram and get ArtNet data.
{
   if ( pdata == NULL)
     return 0u;
   return 1u;
}
#endif
#if (CAMERA_TYPE == XS_CAM)
/*******************************************************************************
* Function Name: XS_process_UDP_InBuffer
********************************************************************************
* Summary:
*  Process the AMP Encoder/Decoder message on the relevant UDP port
*
* Parameters:
*  none
*
* Return:
* uint8_t  returns a code to say the type of message or failure
*
*******************************************************************************/
//=================== XS UDP MESSAGE PROCESSING ================================
// Simple Handler ORIG_SIMPLE does not cater for return values from the encoder
//        e.g. $GETHUE >>
//        << +OK,20
//==============================================================================
#ifdef ORIG_SIMPLE_CAM                                                          // ====== If we ignore $GET $SYS and $DISK messsages (simple reply) =========================
int8_t XS_process_UDP_InBuffer()                                                // Process the in buffer from XStream and decide what it means
{
  
  //  XS_DATA.UDP_Buffer[XS_DATA.UDP_Buffer_Len-2] =0;                              // Delete the UDP CRC (?? check if still valid in COM PIC)
  
  if (XS_DATA.UDP_Buffer[0u] == 43u)                                            // Valid Reply only begins with a + sign
  {
     if (!strncmp(&XS_DATA.UDP_Buffer,"+OK",3))                                 // Return XS_OK if we match (command complete)
     {
        return (XS_OK);
     }
     else if (!strncmp(&XS_DATA.UDP_Buffer,"+AUTH",5))                          // Return XS_AUTH if we match (need to login)
     {
        return (XS_AUTH);
     }
     else                                                                        // Return an Error code
     {
       return (XS_ERROR);
     }
  }
  else
  {
    return (XS_ERROR);                                                          // Return and Error code
  }

}
#endif
#ifndef ORIG_SIMPLE_CAM
/*******************************************************************************
* Function Name: XS_process_UDP_InBuffer
********************************************************************************
* Summary:
*  Process the AMP Encoder/Decoder message on the relevant UDP port
*
* Parameters:
*  none
*
* Return:
* uint8_t  returns a code to say the type of message or failure
*
*******************************************************************************/
//=================== XS UDP MESSAGE PROCESSING ================================
// Simple Handler ORIG_SIMPLE does not cater for return values from the encoder
//        e.g. $GETHUE >>
//        << +OK,20
//==============================================================================
int8_t XS_process_UDP_InBuffer()                                                // Process the in buffer from XStream and decide what it means
{
  uint8_t switchValue;                                                          // what tells you if it was extended or not
  unsigned char* token;                                                         // what is extracted from the , field
  uint8_t fieldNo=0u;                                                           // counter representing the number of fields separated by a comma in the message
  
  //  XS_DATA.UDP_Buffer[XS_DATA.UDP_Buffer_Len-2] =0;                              // Delete the UDP CRC (?? check if still valid in COM PIC)
  token = strtok(&XS_DATA.UDP_Buffer, ",");                                     // Returns first token
  if (token != NULL)                                                            // Conatains no comma's
  {
     if (XS_DATA.UDP_Buffer[0u] == 43u)                                           // Valid Reply only begins with a + sign
     {
        if (!strncmp(&XS_DATA.UDP_Buffer,"+OK",3))                              // Return XS_OK if we match (command complete)
        {
           return (XS_OK);
        }
        else if (!strncmp(&XS_DATA.UDP_Buffer,"+AUTH",5))                       // Return XS_AUTH if we match (need to login)
        {
           return (XS_AUTH);
        }
        else if (!strncmp(&XS_DATA.UDP_Buffer,"+FAIL",5))                       // Return XS_FAIL if the command failed
        {
           return (XS_FAIL);
        }
        else if (!strncmp(&XS_DATA.UDP_Buffer,"+WRONG",6))                      // Return XS_WRONG if we got a wrong channel request
        {
           return (XS_WRONG);
        }
        else if (!strncmp(&XS_DATA.UDP_Buffer,"+INVALID",8))                    // Return XS_INVALID if we got a wrong parameter value sent
        {
           return (XS_INVALID);
        }
        else
        {
          return (XS_ERROR);                                                    // Return an Error code
        }
     }
     else
     {
       return (XS_ERROR);                                                       // Return and Error code
     }
  }
  else
  {
     if (!strncmp(token,"+OK",3))                                               // it was XS_OK with value fields after it
     {
        fieldNo=0;
        token = strtok(&XS_DATA.UDP_Buffer, ",");                               // Returns first token that was preceeding the comma
        while (token != NULL)                                                   // read each delimetered field in the string
        {
          switch (fieldNo)
          {

             case 0u:                                                            // 1st case <Time field> or other value
             sprintf( g_encTime,"%s\n", token);
             if (!strncmp(g_encTime,"OK",2))                                    // if it was a 2nd OK its a reply to stop/mark all channels so just report ok back
             {
                return(XS_OK);                                                  // return an ok reposonse if got +ok,ok etc
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 1u:                                                            // 2nd case is that we have more fields than one after a comma
             sprintf( g_strField2,"%s\n", token);
             if(isdigit(g_strField2[0u]))                                        // the 1st char was an ascii number then convert it
             {
                g_okField2 = atol(g_strField2);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX ;
             break;

             case 2u:                                                            // 3rd case is that we have more fields than one after a comma
             sprintf( g_strField3,"%s\n", token);
             if(isdigit(g_strField3[0u]))                                        // the 1st char was an ascii number then convert it
             {
                g_okField3 = atol(g_strField3);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 3u:                                                           // 4th case is that we have more fields than one after a comma
             sprintf( g_strField4,"%s\n", token);
             if(isdigit(g_strField4[0u]))                                        // the 1st char was an ascii number then convert it
             {
                g_okField4 = atol(g_strField4);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 4u:                                                           // 5th case is that we have more fields than one after a comma
             sprintf( g_strField5,"%s\n", token);
             if(isdigit(g_strField5[0u]))                                       // the 1st char was an ascii number then convert it
             {
                g_okField5 = atoi(g_strField5);                                 // convert to int8_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;
             
             case 5u:                                                            // 5th case is that we have more fields than one after a comma
             sprintf( g_strField5,"%s\n", token);
             if(isdigit(g_strField5[0u]))                                        // the 1st char was an ascii number then convert it
             {
                g_okField5 = atoi(g_strField5);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 6u:                                                            // 6th case is that we have more fields than one after a comma
             sprintf( g_strField6,"%s\n", token);
             if(isdigit(g_strField6[0u]))                                        // the 1st char was an ascii number then convert it
             {
                g_okField6 = atoi(g_strField6);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 7u:                                                           // 7th case is that we have more fields than one after a comma
             sprintf( g_strField7,"%s\n", token);
             if(isdigit(g_strField7[0u]))                                        // the 1st char was an ascii number then convert it
             {
                g_okField7 = atoi(g_strField7);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 8u:                                                           // 8th case is that we have more fields than one after a comma
             sprintf( g_strField8,"%s\n", token);
             if(isdigit(g_strField8[0u]))                                       // the 1st char was an ascii number then convert it
             {
                g_okField8 = atoi(g_strField8);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;
             
             default:                                                           // Too much to parse
             return(XS_ERROR);                                                  // return an error
             break;
          }
        }
        
        token = strtok(g_encTime, ":");                                         // Returns first token that was preceeding the : char i.e. its a time
        if (token != NULL)                                                      // It was a time string that was returned (it contained a : )
           return(XS_TIME);                                                     // return time request found
        else
        {
           //if((encTime[0]>=0x30) && (encTime[0]<=0x39))                       // It was an ascii number with no : so not a time assume this is a value to return
           //{
           if(isdigit(g_encTime[0u]))                                           // the 1st char was an ascii number with no : so not a time assume this is a value to return
           {
              g_okField1 = atol(g_encTime);                                     // convert the value to a number
           }
           else
           {
              strcpy(g_strField1,g_encTime);                                    // copy it to a string field
           }
        }
        return (XS_OK+fieldNo);                                                 // return an okay plus the number of fields parsed
     }
     else if (!strncmp(token,"+DISKSTATUS",11))                                 // it was a diskstatus request
     {
        fieldNo=0u;
        token = strtok(&XS_DATA.UDP_Buffer, ",");                               // Returns first token
        while (token != NULL)                                                   // read each delimetered field in the string
        {
          switch (fieldNo)
          {
             case 0u:                                                            // 1st case <diskstatus> diskStatus string
             sprintf( g_diskStatus,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 1u:                                                            // 2nd case is <totalsize>
             g_totalSizeXS = atol(token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 2:                                                            // 3rd case is <remaining>
             g_remSizeXS = atol(token);
             token = strtok(NULL, ",");
             fieldNo=0u;                                                        // reset the field number for next time it is used
             return(XS_DISK);                                                   // Tell the upper caller it was a DISKSTATUS reply
             break;

             default:
             return(XS_ERROR);                                                  // return an error
             break;
          }
        }
     }
     else if (!strncmp(token,"+SYSSTATUS",10))                                  // it was a sysstatus request
     {
        fieldNo=0u;
        token = strtok(&XS_DATA.UDP_Buffer, ",");                               // Returns first token
        while (token != NULL)                                                   // read each delimetered field in the string
        {
          switch (fieldNo)
          {
             case 0u:                                                            // 1st case <systemstatus>
             sprintf( g_sysStatus,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 1u:                                                            // 2nd case is <numchannels> = 4
             //numChan = atol(token);                                           Dont bother as always set to 4 for our encoder
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 2u:                                                            // 3rd case is <channelstatus> No.1
             sprintf( g_ch1Status,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 3u:                                                            // 4th case is <channelstatus> No.2
             sprintf( g_ch2Status,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 4u:                                                            // 5th case is <channelstatus> No.3
             sprintf( g_ch3Status,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 5u:                                                            // 6th case is <channelstatus> No.4
             sprintf( g_ch4Status,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=0u;                                                        // reset the field number for next time it is used
             return(XS_SYS);                                                    // Tell the upper caller it was a SYSSTATUS reply
             break;
             
             default:
             return(XS_ERROR);
             break;
          }
        }
     }
  }
}
#endif                                                                          // end NOT ORIG_SIMPLE_CAM
#ifdef UART4_INTERUPT                                                           // This is set if we are enabling UART4 with XS encoder serial
//=================== XS SERIAL MESSAGE PROCESSING =============================
//
/*******************************************************************************
* Function Name: XS_process_SERIAL_InBuffer
********************************************************************************
* Summary:
*  Process the XS message on the relevant serial port
*
* Parameters:
*  none
*
* Return:
* uint8_t  returns a code to say the type of message or failure
*
*******************************************************************************/
int8_t XS_process_SERIAL_InBuffer()                                             // Process the in buffer from XStream and decide what it means
{
  uint8_t switchValue;                                                          // what tells you if it was extended or not
  unsigned char* token;                                                         // what is extracted from the , field
  uint8_t fieldNo=0u;                                                            // counter representing the number of fields separated by a comma in the message
  
  switchValue=UART4.State - UART4_PACKET_IN_BUFFER;                             // SUBTRACT the ready to get the g_extended state which was added at time of end message (which identified various char on parsing)
  
  if (UART4.Buffer[0u] == 43u)                                                    // Valid Reply only begins with a + sign
  {
     switch(switchValue)
     {
        case 0u:                                                                 // no extended message
        if (!strncmp(&UART4.Buffer,"+OK",3))                                    // Return XS_OK if we match (command complete)
        {
           return (XS_OK);
        }
        else if (!strncmp(&UART4.Buffer,"+AUTH",5))                             // Return XS_AUTH if we match (need to login)
        {
           return (XS_AUTH);
        }
        else if (!strncmp(&XS_DATA.UDP_Buffer,"+FAIL",5))                       // Return XS_FAIL if the command failed
        {
           return (XS_FAIL);
        }
        else if (!strncmp(&XS_DATA.UDP_Buffer,"+WRONG",6))                      // Return XS_WRONG if we got a wrong channel request
        {
           return (XS_WRONG);
        }
        else if (!strncmp(&XS_DATA.UDP_Buffer,"+INVALID",8))                    // Return XS_INVALID if we got a wrong parameter value sent
        {
           return (XS_INVALID);
        }
        else                                                                    // Return an Error code as it was +wrong or +fail (prog just ignores these)
        {
           return (XS_ERROR);
        }
        break;
        
        case 3u:                                                                // +OK extended was found we asume its a time request other get calls not used
        fieldNo=0u;
        token = strtok(&UART4.Buffer, ",");                                     // Returns first token that was preceeding the comma
        while (token != NULL)                                                   // read each delimetered field in the string
        {
          switch (fieldNo)
          {
             case 0u:                                                            // The command should be +OK
             if (!strncmp(token,"+OK",3))
             {
               token = strtok(NULL, ",");                                       // get the next field separated by a comma
               fieldNo=++fieldNo % UINT8_MAX;                                   // increment the field number
             }
             else
             {
               token = NULL;                                                    // terminate wrong message was parsed
               return(XS_ERROR);                                                // return an error
             }
             break;

             case 1u:                                                           // 1st case <Time field> or other value
             sprintf( g_encTime,"%s\n", token);
             if (!strncmp(g_encTime,"OK",2))                                    // if it was a 2nd OK its a reply to stop/mark all channels so just report ok back
             {
                return(XS_OK);                                                  // return an ok reposonse if got +ok,ok etc
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 2u:                                                           // 2nd case is that we have more fields than one after a comma
             sprintf( g_strField2,"%s\n", token);
             if(isdigit(g_strField2[0u]))                                       // the 1st char was an ascii number then convert it
             {
                g_okField2 = atol(g_strField2);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 3u:                                                           // 3rd case is that we have more fields than one after a comma
             sprintf( g_strField3,"%s\n", token);
             if(isdigit(g_strField3[0u]))                                       // the 1st char was an ascii number then convert it
             {
                g_okField3 = atol(g_strField3);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;
             
             case 4u:                                                           // 4th case is that we have more fields than one after a comma
             sprintf( g_strField4,"%s\n", token);
             if(isdigit(g_strField4[0u]))                                       // the 1st char was an ascii number then convert it
             {
                g_okField4 = atol(g_strField4);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 5u:                                                           // 5th case is that we have more fields than one after a comma
             sprintf( g_strField5,"%s\n", token);
             if(isdigit(g_strField5[0u]))                                       // the 1st char was an ascii number then convert it
             {
                g_okField5 = atoi(g_strField5);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 6u:                                                           // 6th case is that we have more fields than one after a comma
             sprintf( g_strField6,"%s\n", token);
             if(isdigit(g_strField6[0u]))                                       // the 1st char was an ascii number then convert it
             {
                g_okField6 = atoi(g_strField6);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 7u:                                                           // 7th case is that we have more fields than one after a comma
             sprintf( g_strField7,"%s\n", token);
             if(isdigit(g_strField7[0u]))                                       // the 1st char was an ascii number then convert it
             {
                g_okField7 = atoi(g_strField7);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;
             
             case 8u:                                                            // 8th case is that we have more fields than one after a comma
             sprintf( g_strField8,"%s\n", token);
             if(isdigit(g_strField8[0u]))                                        // the 1st char was an ascii number then convert it
             {
                g_okField8 = atoi(g_strField8);                                 // convert to int32_t
             }
             token = strtok(NULL, ",");                                         // check for another comma
             fieldNo=++fieldNo % UINT8_MAX;
             break;
             
             default:
             return(XS_ERROR);
             break;
          }
        }
        token = strtok(g_encTime, ":");                                         // Returns first token that was preceeding the : char i.e. its a time
        if (token != NULL)                                                      // It was a time string that was returned (it contained a : )
           return(XS_TIME);                                                     // return time request found
        else
        {
           //if((encTime[0]>=0x30) && (encTime[0]<=0x39))                       // It was an ascii number with no : so not a time assume this is a value to return
           //{
           if(isdigit(g_encTime[0u]))                                            // the 1st char was an ascii number with no : so not a time assume this is a value to return
           {
              g_okField1 = atol(g_encTime);                                     // convert the value to a number
           }
           else
           {
              strcpy(g_strField1,g_encTime);                                    // copy it to a string field
           }
        }
        return (XS_OK+fieldNo);                                                 // return an okay plus the number of fields parsed
        break;
        
        case 10u:                                                                // Should be a +SYSSTATUS reply
        fieldNo=0u;
        token = strtok(&UART4.Buffer, ",");                                     // Returns first token
        while (token != NULL)                                                   // read each delimetered field in the string
        {
          switch (fieldNo)
          {
             case 0u:                                                            // The command should be +SYSSTATUS
             if (!strncmp(token,"+SYSSTATUS",10))
             {
               token = strtok(NULL, ",");                                       // get the next field
               fieldNo=++fieldNo % UINT8_MAX;
             }
             else
             {
               token = NULL;                                                    // terminate wrong message was parsed
               return(XS_ERROR);                                                // return an error
             }
             break;

             case 1u:                                                           // 1st case <systemstatus>
             sprintf( g_sysStatus,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 2u:                                                           // 2nd case is <numchannels> = 4
             //numChan = atol(token);                                           Dont bother as always set to 4 for our encoder
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;

             case 3u:                                                            // 3rd case is <channelstatus> No.1
             sprintf( g_ch1Status,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;
             
             case 4u:                                                           // 4th case is <channelstatus> No.2
             sprintf( g_ch2Status,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;
             
             case 5u:                                                           // 5th case is <channelstatus> No.3
             sprintf( g_ch3Status,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;
             
             case 6u:                                                           // 6th case is <channelstatus> No.4
             sprintf( g_ch4Status,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=0u;                                                        // reset the field number for next time it is used
             return(XS_SYS);                                                    // Tell the upper caller it was a SYSSTATUS reply
             break;
             
             default:
             return(XS_ERROR);
             break;
          }
        }
        break;
        
        case 11u:                                                               // Should mean we have a +DISKSTATUS
        fieldNo=0u;
        token = strtok(&UART4.Buffer, ",");                                     // Returns first token
        while (token != NULL)                                                   // read each delimetered field in the string
        {
          switch (fieldNo)
          {
             case 0u:                                                           // The command should be +DISKSTATUS
             if (!strncmp(token,"+DISKSTATUS",10))
             {
               token = strtok(NULL, ",");                                       // get the next field
               fieldNo=++fieldNo % UINT8_MAX;
             }
             else
             {
               token = NULL;                                                    // terminate wrong message was parsed
               return(XS_ERROR);                                                // return an error
             }
             break;
             
             case 1u:                                                           // 1st case <diskstatus> diskStatus string
             sprintf( g_diskStatus,"%s\n", token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;
             
             case 2u:                                                           // 2nd case is <totalsize>
             g_totalSizeXS = atol(token);
             token = strtok(NULL, ",");
             fieldNo=++fieldNo % UINT8_MAX;
             break;
             
             case 3u:                                                           // 3rd case is <remaining>
             g_remSizeXS = atol(token);
             token = strtok(NULL, ",");
             fieldNo=0u;                                                        // reset the field number for next time it is used
             return(XS_DISK);                                                   // Tell the upper caller it was a DISKSTATUS reply
             break;
             
             default:
             return(XS_ERROR);                                                  // return an error
             break;
          }
        }
        break;
     }
  }
  else
  {
     return (XS_ERROR);                                                         // Return and Error code
  }

}
#endif
#endif                                                                          // ======================= eNd XS Encoder / Decoder ======================

/*******************************************************************************
* Function Name: sizeToChar
********************************************************************************
* Summary:
*  String function to calculate a length up to the given char specified
*
* Parameters:
*   unsigned char* stringX, unsigned char val
*
* Return:
* uint8_t  returns length (up to the char)
*
*******************************************************************************/
uint8_t sizeToChar( const unsigned char* stringX, unsigned char val )
{
   uint8_t lentoChar=0U;

   while (*stringX++ != val) lentoChar=++lentoChar % UINT8_MAX;
   return(lentoChar);                                                           // if you want to include the char then add 1 to it
}

/*
 * Function Name:
 *   do_UART_to_UDP()
 * Description:
 *   Sends incoming packetsd from UART out UDP
 * Arguments:
 *   None
 * Returns:
 *   None
 * NOTES:
 *   <notes>
 */
#ifdef DO_COMPIC                                                                /* this is from the COM PIC Code this core may now be superceeded */
/*******************************************************************************
* Function Name: do_UART_to_UDP
********************************************************************************
* Summary:
*  convert message on uart 2 to udp
*
* Parameters:
*  none
*
* Return:
* char
*
*******************************************************************************/
char do_UART_to_UDP()
{
    uint16_t ethOk;
    char str1[32u]={"No State\n\r"};
    // @@250919 init to zero not 1 because Net_Ethernet_Intern_sendUDP returns a zero on error.
    char stat =0;
   // char str1[100]={"No known state"};
   
   //Uart5_write_text("in doUART\n\r");
    // If Packet comes from UART 2 send it UDP to remote IP address, sourcePort 10002 (UART2RXPort), destination port 6767 (UART2TXPort)
   if (UART2.State == UART2_SEND_UDP)
   {
      Heart.Enabled = FALSE;
      // wait for the etthernet state not to be busy txbusy or rxbusy
      //  ethOk = (ETHSTAT & 0x60)>>5;
      ethOk = (ETHSTAT & 0x40u)>>5u;                                            // ignore RXBUSY only wait for TXBUSY
      //ETHCON1CLR = 0x8300;
      T2CON = ENABLE_T2;
      while  (ethOk != 0u)
      {
         //ethOk = (ETHSTAT & 0x60)>>5;
         ethOk = (ETHSTAT & 0x40u)>>5u;                                          // just wait for not TXBUSY ignore RXBUSY
         sprintf(str1,"ETH_NOT_FREE : %d\n\r",ETHSTAT);
         Uart2_write_text(&str1);
       }
       UART2.Index_Was = UART2.Index;
    // T5CON         = 0x00; //Stop timer 5 i.e. stop Heart beat so we dont try sending a UDP Packet whilst sending this one.
       if(Air.Link_Alive | MasGs.Link_Alive | FirGs.Link_Alive)
       {

          stat = Net_Ethernet_Intern_sendUDP(UART2.RemoteIpAddr, UART2.From_Port, UART2.Dest_Port, &UART2.Buffer,UART2.Index);
          Status(UDPSTA_Add,stat);//UDP_STATE: -UDP Packet sent or  UDP_STATE: -Packet Send Failed
          // @@12092019 commented this out for now ---- if (!stat){Init_Ether(); uart5_write("Ether Reset\n\r");}

           Heart.Enabled = TRUE;
           UART2.UDP_Byte_Sent += UART2.Index;
           UART2.UDP_Packets_Sent=++UART2.UDP_Packets_Sent % UINT16_MAX;
           UART2.UDP_Send_trys=++UART2.UDP_Send_trys % UINT8_MAX;
           UART2.Index=CLEAR;
           UART2.State = UART2_CHECK_ACK_TIME_OUT;
       }
       else
       {
          Heart.Enabled = TRUE;
       //UART2.UDP_Byte_Sent += UART2.Index;
       //UART2.UDP_Packets_Sent++;
       //UART2.UDP_Send_trys++;
          UART2.Index_Was = UART2.Index;
          UART2.Index = CLEAR;
          UART2.State = CLEAR;
          UART2.UDP_Send_Fail=++UART2.UDP_Send_Fail % UINT8_MAX;
          Status(ETHER_LINK_STATE,LINK_DOWN);
        }

    //T2CON = ENABLE_T2;       // Enable Timeout
      }                                                                         /* end while */
      return (stat);
  }
#endif

#ifdef USE_CAN
/*******************************************************************************
* Function Name: do_CAN_to_UDP
********************************************************************************
* Summary:
*  convert message on canbus to udp
*
* Parameters:
*  none
*
* Return:
* void
*
*******************************************************************************/
/* void do_CAN_to_UDP()
{
   uint16_t Can2_UDP_TX_Packet[12u];
   uint16_t MSGLen=1u;
   uint16_t counter=0u;

   //Can_Rcv_Flags =0; //Clear the message flag
   if (Can_msg_rcvd=CAN2Read((uint32_t*)&Can2_RX_MSG_ID,(unsigned char*) Can2_Msg_Rcvd,(uint16_t*) &Can2_MSG_Len,(uint16_t*) &Can2_Rcv_Flags))
   {
      LATB0_bit = ~PORTB;
      Can2_UDP_TX_Packet[0u] = Lo(Can2_RX_MSG_ID);
      Can2_UDP_TX_Packet[1u] = Hi(Can2_RX_MSG_ID);

      counter = 0u;
      while(counter < Can2_MSG_Len)
      {
         Can2_UDP_TX_Packet[counter+2u] = Can2_Msg_Rcvd[counter];

         while(counter<12u)                                                     // CLEAR OUT TX STRING
         {
           Can2_UDP_TX_Packet[counter]=0u;
           counter=++counter % UINT16_MAX;
         }
         counter=++counter % UINT16_MAX;
       }
       counter = 0u;

      //MSGLen=strlen(Can2_UDP_TX_Packet);
      MSGLen=Can2_MSG_Len+2u;
      for (counter =0u; counter <10u;)
      {
        if (Remote_Node[counter].Link_Alive)         
        {
         Net_Ethernet_Intern_sendUDP((unsigned char*) &Remote_Node->IPAddr[counter],Can2fromPort,Can2destPort,(unsigned char*) &Can2_UDP_TX_Packet, MSGLen);
         counter=++counter % UINT16_MAX;
        }
      }
   }

 } */

/*******************************************************************************
* Function Name: Can2_Test
********************************************************************************
* Summary:
*  Can test
*
* Parameters:
*   none
*
* Return:
*   none
*
*******************************************************************************/
void Can2_Test()
{
  Can_RxTx_Data1[0u] = 0xFEu;
  Can_RxTx_Data1[1u] = 0xFFu;
  while(1u)
  {
     Can_RxTx_Data1[0u]++;                                                      // increment received data
     Can_RxTx_Data1[1u]++;                                                      // increment received data
     Can_RxTx_Data1[2u]=0X02u;                                                  // increment received data
     Delay_ms(500u);
     CAN2Write(Can2_TX_MSG_ID, Can_RxTx_Data1, 8u, Can2_Send_Flags);            // send incremented data back
   }

}
#endif
/*******************************************************************************
* Function Name: bitWrite
********************************************************************************
* Summary:
*  Can test
*
* Parameters:
*   char *x, char n, char value
*
* Return:
*   none
*
*******************************************************************************/
void bitWrite(char *x, char n, char value) 
{
   if (value)
      *x |= (1u << n);
   else
      *x &= ~(1u << n);
}
/*******************************************************************************
* Function Name: bitRead
********************************************************************************
* Summary:
*  Can test
*
* Parameters:
*   char *x, char n
*
* Return:
*   char 1 if match 0 not
*
*******************************************************************************/
char bitRead(char *x, char n) 
{
   return (*x & (1u << n)) ? 1 : 0;
}

//=========================== ETHERNET INIT ====================================
//
/*******************************************************************************
* Function Name: Init_Ether()
********************************************************************************
* Summary:
*  Reset and initialise the NIC
*
* Parameters:
*   none
*
* Return:
*   none
*
*******************************************************************************/
void Init_Ether()
{
  // added this here to reset the NIC @@12092019
  IEC1CLR = _IEC1_ETHIE_MASK;                                                   // disable NIC interrupts 0x10000000
  // used to be ETHCON1CLR = 0x00008300;                                         the 300 isnt in the cpu header file under ETHCON1
  ETHCON1CLR =  _ETHCON1_ON_MASK | _ETHCON1_TXRTS_POSITION | _ETHCON1_RXEN_MASK;// turn the ethernet controller off 0x00008109
  while(ETHSTAT&_ETHSTAT_ETHBUSY_MASK);                                         // Wait activity abort by polling the ETHBUSY bit (bit 7) 0x80
  ETHIENCLR = 0x000063efUL;                                                       // Disable any Ethernet controller interrupt generation
  ETHIRQCLR = 0x000063efUL;                                                       // Disable any Ethernet controller interrupt generation
  ETHCON1SET = _ETHCON1_ON_MASK;                                                // Enable the Ethernet controller by setting the ON bit 0x00008000
  IFS1CLR = 0x10000000UL;                                                         // Clear Ethernet interrupt flag
  ETHIEN = 0u;                                                                   // Disable any Ethernet controller interrupt generation
  ETHIRQ = 0u;                                                                   // Disable any Ethernet controller interrupt generation
  ETHTXST = 0u;                                                                  // Clear the TX and RX start addresses
  ETHRXST = 0u;                                                                  // Clear the TX and RX start addresses
  // ETHIENSET = 0x0000400c;                                                     Were what i had before  // enable RXACT interrupt
  // IPC12CLR = 0x0000001f;
  // IPC12SET = 0x000016;
  // IEC1SET = 0x10000000;                                                      // RE_Enable Ethernet MAC interrupts
  SPI1CONbits.ON= FALSE;                                                        // disable SPI for TCPIP interface
  WDTCONbits.ON = FALSE;                                                        // watchdog off
  
  EMAC1CFG1SET = 0x00008000UL;                                                    // Reset the MAC using SOFTRESET
  EMAC1CFG1CLR = 0x00008000UL;                                                    // Reset the MAC using SOFTRESET
  EMAC1SUPPSET = 0x00000800UL;                                                    // Reset the RMII module
  EMAC1SUPPCLR = 0x00000800UL;                                                    // Reset the RMII module

  EMAC1MCFGSET = 0x00008000UL;                                                    // Issue an MIIM block reset by setting the RESETMGMT bit
  EMAC1MCFGCLR = 0x00008000UL;                                                    // Issue an MIIM block reset by setting the RESETMGMT bit
  
  EMAC1MCFG = _EMAC1MCFG_CLKSEL_DIV40;                                          // Select the proper divider for the MDC clock (8<<0x00000002)

  Net_Ethernet_Intern_Set_Default_PHY();                                        // Not sure if we need this here or at start

  ETHHT0 = 0u;                                                                   // Initialize hash table
  ETHHT1 = 0u;

  ETHRXFC =  _ETHRXFC_HTEN_MASK | _ETHRXFC_CRCOKEN_MASK  |  _ETHRXFC_RUNTEN_MASK | _ETHRXFC_UCEN_MASK |  _ETHRXFC_BCEN_MASK;    // Configure the receive filter

  Net_Ethernet_Intern_Init(This_Node.MacAddr, This_Node.IpAddr, Net_Eth_Int_AUTO_NEGOTIATION & Net_Eth_Int_DEFAULT_MAC & Net_Eth_Int_SPEED_100 & Net_Eth_Int_FULLDUPLEX ); // init ethernet board as My
  Net_Ethernet_Intern_confNetwork(ipMask, gwIpAddr, dnsIpAddr);
  Net_Ethernet_Intern_Enable( _Net_Ethernet_Intern_UNICAST | _Net_Ethernet_Intern_BROADCAST | _Net_Ethernet_Intern_CRC); // enable Unicast traffic
  EMAC1CFG1 = 0x00000001UL;                                                     // Disable flow control RX_ENAB (MAC)
  EMAC1CFG2 = 0x00000020UL | 0x00000010UL;                                      // Automatic padding and CRC generation This is youre own Intern_Enable
  EMAC1MAXF = 1518U;                                                            // Set the maximum frame length
  Net_Ethernet_Intern_Disable( _Net_Ethernet_Intern_MULTICAST );                // disable Multicast traffic
  // Net_Ethernet_Intern_Disable(_Net_Ethernet_Intern_BROADCAST);               // disable Broadcast traffic  (we accept e.g. gratuotous ARP)

  ETHCON2 = PIC32MX_ETH_RX_BUFFER_SIZE;                                         // Set receive buffer size
  ETHCON1SET = 0x00008100UL;                                                    // Enable the reception by setting the EN AND RXEN bit
  //  ETHTXST = (uint32_t) KVA_TO_PA(&txBufferDesc[0]);
  
}

/*******************************************************************************
* Function Name: ARP
********************************************************************************
* Summary:
*  Perform ARP request to remote node
*
* Parameters:
*   unsigned char *NodeIPAddr
*
* Return:
*   char  FALSE = error TRUE = okay
*
*******************************************************************************/
char ARP(unsigned char *NodeIPAddr)
 {   
     uint16_t LC1;
     uint16_t LC2;

     if (NodeIPAddr == NULL)
        return 0;
        
     if (!CheckTimout())
     {
       LC1 = 0u;
       EFinish = 1u;
       Ether_Link_Present = FALSE;                                              // Set the link down

       Net_Ethernet_Intern_arpCache[0u].valid = 0u;                             // Flush the ARP Cache
       Net_Ethernet_Intern_arpCache[1u].valid = 0u;
       Net_Ethernet_Intern_arpCache[2u].valid = 0u;
       // Do an ARP request sequence for NMS1
       //Delay_ms(1000);
       while ((LC1++ < 3u) && (EFinish == 1u))
       {
          ptr_ru = Net_Ethernet_Intern_arpResolve(NodeIPAddr, 1u);              // get MAC address behind the above IP address, wait up to 1 secs for the response
          //Net_Ethernet_Intern_arpResolve(NodeIPAddr, 1);
          Delay_ms(100u);                                                       // ok as doing nothing else until resolved (does nothing else until ARP complete anyway
          for ( LC2 = 0u; LC2 < ARPCACHESIZE_Intern; ++LC2 )
          {
              if ( Net_Ethernet_Intern_arpCache[LC2].valid == 1u )
              {

               if
               (
                  (Net_Ethernet_Intern_arpCache[LC2].ip[0u] == NodeIPAddr[0u]) && (Net_Ethernet_Intern_arpCache[LC2].ip[1u] == NodeIPAddr[1u]) && (Net_Ethernet_Intern_arpCache[LC2].ip[2u] == NodeIPAddr[2u]) && (Net_Ethernet_Intern_arpCache[LC2].ip[3u] == NodeIPAddr[3u]) )
                  {
                      Ethernet_Fault = FALSE;
                      Ether_Link_Present = TRUE;                                // Ethernet is present
                      LC2 = 3u;                                                 // set to exit for loop
                      EFinish = 0u;                                             // set to exit the for loop

                      if (Ether_Link_Present != Ether_Link_Present_Was)
                      {
                        Ether_Link_Present_Was = Ether_Link_Present;
                        Status(ETHER_LINK_STATE,Ether_Link_Present);
                      }
                      if (!Ether_Link_Present){StartTimeout();}
                        return TRUE;                                            // return success
                   }
                   else                                                         // Not the ip address requested
                   {
                      Ethernet_Fault = TRUE;
                      Ether_Link_Present = FALSE;                               // Ethernet is NOT present
                      Heart.Time = Net_Ethernet_Intern_userTimerSec;
                      Heart.Enabled = TRUE;
                      if (Ether_Link_Present != Ether_Link_Present_Was)
                      {
                          Ether_Link_Present_Was = Ether_Link_Present;
                          Status(ETHER_LINK_STATE,Ether_Link_Present);
                      }
                     if (!Ether_Link_Present){StartTimeout();}
                     //Uart2_write_text("ETH_ARP_FAILED\n\r");                  // record the event
                       return FALSE;                                            // return an error
                   }
               }
            }

         }
      }
 }

/*******************************************************************************
* Function Name: boot_arp()
********************************************************************************
* Summary:
*  Call the ARP at boot-up or on reset (also called on exceeding consequtive bad sends)
*
* Parameters:
*   none
*
* Return:
*   none
*
*******************************************************************************/
void boot_arp()
{
   while(Ether_Link_Present==FALSE)
   {
      WDTCONSET = 0x01u;
      // Uart5_write_text("ARP\n\r");               
      if (Node==AIR)  {MasGs.Link_Established = ARP(MasGs.IpAddr);              // MASTER GS
                       FirGs.Link_Established = ARP(FirGs.IpAddr);}             // FIRE GS
      else if (Node==MASGS){Air.Link_Established = ARP(Air.IpAddr);}            // COM PIC AIR
      else if (Node==FIRGS){Air.Link_Established = ARP(Air.IpAddr);}            // COM PIC AIR
      else if (Node==GIMJOY){Air.Link_Established = ARP(Air.IpAddr);}           // COM PIC (gimbal / xstream)
      else if (Node==COMGS){Air.Link_Established = ARP(Air.IpAddr);}            // COM PIC (gimbal / xstream)
      // Uart5_write_text("ARP While @boot\n\r");
      asm nop;                                                                  // wait a tick
   }
}

/*******************************************************************************
* Function Name: StartTimeout()
********************************************************************************
* Summary:
*  Collect time ran by ARP function
*
* Parameters:
*   none
*
* Return:
*   none
*
*******************************************************************************/
void StartTimeout() 
{
  TimoutRunning = TRUE;                                                         // set timeout as running
  EthTimePrevCounter = Net_Ethernet_Intern_userTimerSec;                        // save present time
}

// --[ Function ] -----------------------------------------------------------------------------------------------------
// Function Name:               CheckTimout
// Description:                 This function checks if a ruuning TCP timeout has expired and returns a 1 if yes
// Input Parameters:            None
// Output Parameters:           None
// Input/Output Parameters:     None
// Return:                      TRUE if timeout else FALSE
// Notes:                       Nil
// Side Effects:                Nil
// Status:                      Active
// --------------------------------------------------------------------------------------------------------------------
uint8_t CheckTimout()
{
  if ( TimoutRunning == TRUE )                                                  // set timeout as running
  {
    if ( (Net_Ethernet_Intern_userTimerSec - EthTimePrevCounter) > 4u )
    {
      TimoutRunning = FALSE;                                                    // stop timout from running
      return TRUE;                                                              // a timeout occurred
    } 
    else
      return FALSE;                                                              // else no timeout yet
  } 
  return FALSE;                                                                 // a timeout was not running anyway
}

// --[ Function ] -----------------------------------------------------------------------------------------------------
// Function Name:               StopTimer1
// Description:                 This function is used to stop timer 1 so that the timer can be restarted
// Input Parameters:            None
// Output Parameters:           None
// Input/Output Parameters:     None
// Return:                      None
// Notes:                       Nil
// Side Effects:                Nil
// Status:                      Active
// --------------------------------------------------------------------------------------------------------------------
void StopTimer1() 
{
  T1IF_bit = 0u;                                                                // clear interrupt flag - Added in by DNT for restarting on the fly purposes
  TMR1 = 0u;                                                                    // reset timer value to zero
  T1IE_bit = 0u;                                                                // Enable Timer1 Interrupt
  ON__T1CON_bit = 0u;                                                           // Disable Timer1
  asm nop;                                                                      // delay to allow settlement
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// --[ Function ] -----------------------------------------------------------------------------------------------------
// Function Name:             ProcessTcpMsg()
// Description:               This function uses the function Net_Ethernet_Intern_startSendTCP() in the
//                            Net_Ethernet_Internal library to send unsolicited TCP messages to the Remote PIC.
//                            The Tx message buffer should be loaded within Net_Ethernet_Intern_UserTCP() or elsewhere
//                            before transmission is required. It does the unsolicited Tx by sending a dummy ACK to the
//                            Remote which responds. That response causes the the calling of
//                            Net_Ethernet_Intern_UserTCP() which provides the opportunity to tx a message out. Without
//                            that, the library provides NO OTHER WAY to send unsolicited messages, and assumes that you
//                            are always writing code from the Remote perspective. An assumption that is not always
//                            true. It can also be used to process received messages within
//                            Net_Ethernet_Intern_UserTCP(). The function does not hold the cpu until completion so it
//                            must be called contineously from within main().
//                            This routine can be called in different modes:
//                            1) Open the Socket if closed and send a message.
//                            2) Close the socket.
// Input Parameters:          EtherConnState must be set to the entry condition defined in "globaldefs.h" in enum Ethstate
//                            typically starting at OPEN_SOCKET or CLOSE_SOCKET. If you do not want the function to
//                            Close the socket after sending the message, set the flag CloseTcpSocket = 0.
//                            if  EtherConnState = OPEN_SOCKET && CloseTcpSocket = 1;
//                                The TCP Socket will be Auto OPENED, a message SENT, and the socket CLOSED
//                            if  EtherConnState = OPEN_SOCKET && CloseTcpSocket = 0;
//                                The TCP Socket will be "Auto OPENED", a message SENT, and the functions exits without
//                                closing the socket. ("Auto OPENED" means that if the socket is closed, the code will
//                                try and open it. If it is Open , it will nothing and the socket will stay open)
//                            if  EtherConnState = CLOSE_SOCKET && CloseTcpSocket = 0|1;
//                                The TCP Socket will be CLOSED, and the functions exits
//                            if  EtherConnState = UNDEFINED;
//                                The functionality of the routine is disabled and enters without doing anything
//                            StartEthComms must be set 1 at the start of a transmission and further transmissions
//                            must not be started until it has been cleared by this function
// Output Parameters:         none
// Input/Output Parameters:   none
// Return:                    none
// Notes:                     The entire process took about 40mS to 50mS with worst case about 55mS and typical case
//                            about 42mS on a 13 byte send message with nio close socket at the end.
//                            TESTING:
//                            When testing the routine for a fully bi-directional scenario using "TCP/IP Builder", Set
//                            the Destination and Remote port, both to 502. Enter a test string such as "Hi", and use
//                            the "Send" button to send it.
// Side Effects:              None
// Status:                    Active
// --------------------------------------------------------------------------------------------------------------------
 // --------------------------------------------------------------------------------------------------------------------
void ProcessTcpMsg() 
{
 //Modec to ProcessTcpMsg.txt for now
}

//#if  defined(PARROT_SEQ_CAM) || defined(__Yi_c)
//
// To use TCP/IP This should be called after Net_Ethernet_Intern_confNetwork
//
/*******************************************************************************
* Function Name: StartTCPCommunication()
********************************************************************************
* Summary:
* Initialises TCP/IP Stack for use
*
* Parameters:
* none
*
* Return:
* none
*
*******************************************************************************/
void StartTCPCommunication()
{
   Net_Ethernet_Intern_stackInitTCP();                                          // Initialise TCP Stack
}

#if defined(YI_CAM_USED)
/*******************************************************************************
* Function Name: XY_OpenTCPSocketRawJson
********************************************************************************
* Summary:
*  Open the socket for the Yi Cam RAW to the JSON port
*
* Parameters:
*   SOCKET_Intern_Dsc **sock1
*
* Return:
* return if success is 1
* ======================================
* 0 - no successful transmit SYN segment.
* 1 - successful transmit SYN segment.
* 2 - no available socket.
* 3 - no ARP resolve
*
*******************************************************************************/
uint8_t XY_OpenTCPSocketRawJson( SOCKET_Intern_Dsc **sock1 )
{
  return(Net_Ethernet_Intern_connectTCP(XY_USE_IP, XY_DEF_PORT, XY_DEF_PORT, sock1));     // open the socket
}

/*******************************************************************************
* Function Name: XY_CloseTCPSocketRawJson
********************************************************************************
* Summary:
*  Close the socket for the Yi Cam RAW to the JSON port
*
* Parameters:
*   SOCKET_Intern_Dsc **sock1
*
* Return:
* return if success is 1
* ======================================
* 0 - no successful transmit SYN segment.
* 1 - successful transmit SYN segment.
* 2 - no available socket.
* 3 - no ARP resolve
*
*******************************************************************************/
uint8_t XY_CloseTCPSocketRawJson( SOCKET_Intern_Dsc *sock1 )
{
  return(Net_Ethernet_Intern_disconnectTCP(sock1));                             // close the socket
}

/*******************************************************************************
* Function Name: TCPStateCheck
********************************************************************************
* Summary:
*  TCPStateCheck function performs following functions:
*   1. Checks the state of the socket passed to the function
*   2. Attempts to keep the connection alive
*
* Parameters:
*  SOCKET_Intern_Dsc *used_tcp_socket - The TCP socket.
*
* Return:
*  None.
*
*******************************************************************************/
void TCPStateCheck( SOCKET_Intern_Dsc *used_tcp_socket)
{
  uint8_t XY_retTCP;                                                            // Socket open command return code
     
  switch(used_tcp_socket->state)                                                // for the states of the Yi Cam TCP socket
  {
    case TCP_STATE_CLOSED:                                                      // We dont have a connection
    XY_retTCP=XY_OpenTCPSocketRawJson( &used_tcp_socket );                      // try to open a socket
    switch(XY_retTCP)
    {
        case TCP_NO_SYN_SEND:                                                   // didnt transmit SYN
        XY_retTCP=XY_OpenTCPSocketRawJson( &used_tcp_socket );                  // try again to open a socket
        if (XY_retTCP==TCP_SYN_SUCCESS)
          YiCam.Link_Alive   = TRUE;
        else
          YiCam.Link_Alive   = FALSE;
        break;

        case TCP_SYN_SUCCESS:                                                   // continue the SYN was sent
        YiCam.Link_Alive   = TRUE;
        break;

        case TCP_NO_SOCK:                                                       // no socket
        YiCam.Link_Alive   = FALSE;
        break;

        case TCP_NO_ARP:                                                        // no arp
        YiCam.Link_Alive   = FALSE;
        break;

        default:                                                                // invalid return code
        break;
    }
    YiTimer.tcpClosedTm=++YiTimer.tcpClosedTm % UINT32_MAX;                     // Count the time in closed
    break;

    case TCP_STATE_LISTEN:                                                      // Server is listening for a SYN_ACK from the client in response to the SYN sent
    YiTimer.tcpListenTm=++YiTimer.tcpListenTm % UINT32_MAX;                     // Count the time in listen
    if (YiTimer.tcpListenTm > XY_TCP_TIMEOUT)
      if (Net_Ethernet_Intern_startSendTCP( used_tcp_socket )==0)               // 1 - error occur. 0 - routine sent dummy ACK to remote host. If remote host answer, Net_Ethernet_Intern_UserTCP routine will be called, and than We can put our data in TCP Tx buffer
         YiCam_DATA.State = ETH_ACK_RESENT;                                     // Dummy ACK was sent
    break;

    case TCP_STATE_ESTABLISHED:                                                 // Socket ready to send so dont do anything here
    break;

    case TCP_STATE_SYN_SENT:                                                    // You need to look for a SYN
    YiTimer.tcpSynSentTm=++YiTimer.tcpSynSentTm % UINT32_MAX;                   // Count the time in SYN sent
    if (YiTimer.tcpSynSentTm > XY_TCP_TIMEOUT)
       XY_retTCP=XY_OpenTCPSocketRawJson( &used_tcp_socket );                   // try again to open the socket by sending another SYN
    break;

    case TCP_STATE_FIN_WAIT_1:                                                  // You sent a FIN
    YiTimer.tcpSynSentTm=++YiTimer.tcpSynSentTm % UINT32_MAX;                   // Count the time in fin wait 1
    if (YiTimer.tcpFinWait1Tm > XY_TCP_TIMEOUT)
       XY_retTCP=XY_CloseTCPSocketRawJson( used_tcp_socket );                   // try again to close the socket
    break;

    case TCP_STATE_FIN_WAIT_2:                                                  // You wait for a remote FIN
    YiTimer.tcpFinWait2Tm=++YiTimer.tcpFinWait2Tm % UINT32_MAX;                 // Count the time in fin wait 2
    if (YiTimer.tcpFinWait2Tm > XY_TCP_TIMEOUT)
       XY_retTCP=XY_CloseTCPSocketRawJson( used_tcp_socket );                   // try again to close the socket
    break;

    case TCP_STATE_RETRANSMIT:                                                  // Asked to re-transmit
    if (Net_Ethernet_Intern_startSendTCP( used_tcp_socket )==0)                 // 1 - error occur. 0 - routine sent dummy ACK to remote host. If remote host answer, Net_Ethernet_Intern_UserTCP routine will be called, and than We can put our data in TCP Tx buffer
    {
       YiCam_DATA.State = ETH_ACK_OLD_DATA_REQ;                                 // Dummy ACK was sent
    }
    YiTimer.tcpRetransTm=++YiTimer.tcpRetransTm % UINT32_MAX;                   // Count the time in retransmit
    break;
  }
}
/*******************************************************************************
* Function Name: resetXYActions()
********************************************************************************
* Summary:
*  Reset of the HMI state request from the TCP state engine
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void resetXYActions()
{
// iterative action make the command hmiReqActive a uint64_t if you want
// otherwise case is faster although long to look @
//
//for (counter=1; counter < 32u ; counter++)
//{
//     if (XYRequest == counter)
//     {
//        hmiReqActive1 = hmiReqActive1 ^ (uint64_t) pow(2,((double) counter)); // remove the request bit for the case if we are in that step of the sequence
//        hmiReqActive = (COMGS_YiOptActive_t) hmiReqActive1;                   // set equal will not type cast must make the COMGS_YiOptActive_t a uint64_t if you want this method
//        break;
//     }
//}
   switch (XYRequest)
   {
      case 0u:
      hmiReqActive.batteryLeft=0U;
      break;

      case 1u:
      hmiReqActive.startEvent=0U;
      break;

      case 2u:
      hmiReqActive.tkPhoto=0U;
      break;

      case 3u:
      hmiReqActive.setPIV=0U;
      break;

      case 4u:
      hmiReqActive.getRecTime=0U;
      break;

      case 5u:
      hmiReqActive.startRecord=0U;
      break;

      case 6u:
      hmiReqActive.stopRecord=0U;
      break;

      case 7u:
      hmiReqActive.ccsStop=0U;
      break;

      case 8u:
      hmiReqActive.quickRecord=0U;
      break;

      case 9u:
      hmiReqActive.quickRecPause=0U;
      break;

      case 10u:
      hmiReqActive.quickRecResume=0U;
      break;

      case 11u:
      hmiReqActive.resetWifi=0U;
      break;

      case 12u:
      hmiReqActive.videoStd=0U;
      break;

      case 13u:
      hmiReqActive.videoLoopRes=0U;
      break;

      case 14u:
      hmiReqActive.videoPhoRes=0U;
      break;

      case 15u:
      hmiReqActive.videoLapseRes=0U;
      break;

      case 16u:
      hmiReqActive.videoRes=0U;
      break;

      case 17u:
      hmiReqActive.photoSize=0U;
      break;

      case 18u:
      hmiReqActive.idleMode=0U;
      break;

      case 19u:
      hmiReqActive.wakeUpMode=0U;
      break;

      case 20u:
      hmiReqActive.logStart=0U;
      break;

      case 21u:
      hmiReqActive.resetStream=0U;
      break;

      case 22u:
      hmiReqActive.stopStream=0U;
      break;

      case 23u:
      hmiReqActive.bindBlue=0U;
      break;

      case 24u:
      hmiReqActive.unBindBlue=0U;
      break;

      case 25u:
      hmiReqActive.lapseVidTime=0U;
      break;

      case 26u:
      hmiReqActive.lapseVidOpt=0U;
      break;

      case 27u:
      hmiReqActive.slowMotionRt=0U;
      break;

      case 28u:
      hmiReqActive.burstCap=0U;
      break;

      case 29u:
      hmiReqActive.preciseContTim=0U;
      break;

      case 30u:
      hmiReqActive.recPhoTime=0U;
      break;

      case 31u:
      hmiReqActive.loopRecDur=0U;
      break;

      case 32u:
      hmiReqActive.videoQ=0U;
      break;

      case 33u:
      hmiReqActive.photoQ=0U;
      break;

      case 34u:
      hmiReqActive.vidStampVal=0U;
      break;

      case 35u:
      hmiReqActive.phoStampVal=0U;
      break;

      case 36u:
      hmiReqActive.onOffOpt=0U;
      break;

      case 37u:
      hmiReqActive.vidOutDev=0U;
      break;

      case 38u:
      hmiReqActive.meterOpt=0U;
      break;

      case 39u:
      hmiReqActive.ledMode=0U;
      break;

      case 40u:
      hmiReqActive.buzzerOpt=0U;
      break;

      case 41u:
      hmiReqActive.captureMode=0U;
      break;

      case 42u:
      hmiReqActive.fastZoom=0U;
      break;

      case 43u:
      hmiReqActive.bitRate=0U;
      break;

      case 44u:
      hmiReqActive.delFile=0U;
      break;

      case 45u:
      hmiReqActive.format=0U;
      break;

      case 46u:
      hmiReqActive.dir=0U;
      break;

      case 47u:
      hmiReqActive.sndGetFile=0U;
      break;

      case 48u:
      hmiReqActive.getMedia=0U;
      break;

      case 49u:
      hmiReqActive.thumbNail=0U;
      break;

      case 50u:
      hmiReqActive.enabScript=0U;
      break;

      case 51u:
      hmiReqActive.getLastPho=0U;
      break;

      case 52u:
      hmiReqActive.getFreeSD=0U;
      break;
      
      case 53u:
      hmiReqActive.getTotalSD=0U;
      break;
      
      default:
      break;
   }
   
}
#endif                                                                          // ======================== eNd TCP/IP YiCam included ====================
#if defined(SEQ_CAM_USED)
/*******************************************************************************
* Function Name: SEQ_OpenTCPSocketHttpJson
********************************************************************************
* Summary:
*  Open the socket for the http put and get requests for the parrot sequioa cam
*  the replies and configuration requests are JSON objects
*
* Parameters:
*   SOCKET_Intern_Dsc **sock1
*
* Return:
* return if success is 1
* ======================================
* 0 - no successful transmit SYN segment.
* 1 - successful transmit SYN segment.
* 2 - no available socket.
* 3 - no ARP resolve
*
*******************************************************************************/
uint8_t SEQ_OpenTCPSocketHttpJson( SOCKET_Intern_Dsc **sock1 )
{
  return(Net_Ethernet_Intern_connectTCP(SEQ_USE_IP, SEQ_DEF_PORT,SEQ_DEF_PORT, sock1));     // open the socket
}

/*******************************************************************************
* Function Name: SEQ_CloseTCPSocketHttpJson
********************************************************************************
* Summary:
*  Close the socket for the http put and get requests for the parrot sequioa cam
*  the replies and configuration requests are JSON objects from the http API
*
* Parameters:
*   SOCKET_Intern_Dsc **sock1
*
* Return:
* return if success is 1
* ======================================
* 0 - no successful transmit SYN segment.
* 1 - successful transmit SYN segment.
* 2 - no available socket.
* 3 - no ARP resolve
*
*******************************************************************************/
uint8_t SEQ_CloseTCPSocketHttpJson( SOCKET_Intern_Dsc *sock2 )
{
  return(Net_Ethernet_Intern_disconnectTCP(sock2));                             // close the socket
}
#endif                                                                          // ================== eNd Parrot Sequoia camera ======================

///===================================================================================================================================================
//
//                                                 TCP Handler
//
//*****************************************************************************************************************************************************
/*******************************************************************************
* Function Name: Net_Ethernet_Intern_UserTCP
********************************************************************************
* Summary:
*  Net_Ethernet_Intern_UserTCP function performs following functions:
*   1. Checks the state of the socket passed to the function
*   2. Reads data sent from the TCP Serve and places into a read buffer
*
* Parameters:
*  SOCKET_Intern_Dsc *used_tcp_socket - The TCP socket.
*
* Return:
*  None.
*
*******************************************************************************/
void Net_Ethernet_Intern_UserTCP(SOCKET_Intern_Dsc *used_tcp_socket)
 {
#if (((((defined(YI_CAM_USED) || defined(PENTAX_CAM_USED)) || defined(SEQ_CAM_USED)) || defined(CBOR_COMS_USED)) || defined(JSON_COMS_USED)) || defined(USE_TCP_CAN))

#if defined(SEQ_CAM_USED) || defined(YI_CAM_USED)
     unsigned char rcvIpAddr[17u];                                              // String to hold the incoming ip address
#endif
#if defined(SEQ_CAM_USED)
     uint8_t SEQ_retTCP;                                                        // Socket open command return code
#endif
#if defined(USE_TCP_CAN)
    uint16_t Can2_TCP_RX_Packet[8u];                                            // Can bus RX packet
    uint16_t reqLength =0u;                                                     // canbus packet length
    uint16_t sendValue=0u;                                                      // return code from can write
#endif
#if defined(YI_CAM_USED) || defined(USE_TCP_CAN)
     uint8_t counter=0u;                                                        // common counter
#endif
#if defined(HTTP_USED)                                                          // defined if the interface requires it
  uint8_t len;
  unsigned char getRequest[15u];                                                // HTTP request buffer
#endif
#if defined(YI_CAM_USED)
     uint8_t XY_retTCP;                                                         // Socket open command return code
     uint8_t dat_pos=0U;                                                        // writing postion in the array

     uint64_t val;
     uint64_t hmiReqActive1;
#endif
#if defined(HTTP_USED)
  
  switch(used_tcp_socket->destPort)
  {
     case HTTP_PORT:                                                            // port 80 http
     switch(used_tcp_socket->state)                                             // for the states of the sockets
     {
        case TCP_STATE_ESTABLISHED:                                             // Socket ready to send or receive
        for(len = 0u; len < strlen(HTTP11_GET_REQ); len++)                      // get 14 first bytes only of the request, the rest does not matter here
        {
           getRequest[len] = Net_Ethernet_Intern_getByte();                     // read the BYTE stream
        }
        getRequest[len] = 0u;
        if(memcmp(getRequest, HTTP11_GET_REQ, strlen(HTTP11_GET_REQ))==0u)      // was a get request version 1.1
        {
          strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"GET");  // set method received to get
          HTMLContext.version = HTTP_VERSION_1_1;                               // set version to 1.1
          socketHTML = used_tcp_socket;                                         // set the TCP/ip socket for the html message
        }
        else if (memcmp(getRequest, HTTP11_PUT_REQ, strlen(HTTP11_PUT_REQ))==0u)
        {
          strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"PUT");  // set method received to put
          HTMLContext.version = HTTP_VERSION_1_1;                               // set version to 1.1
          socketHTML = used_tcp_socket;                                         // set the TCP/ip socket for the html message
        }
        else if(memcmp(getRequest, HTTP10_GET_REQ, strlen(HTTP10_GET_REQ))==0u)      // was a get request version 1.1
        {
          strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"GET");  // set method received to get
          HTMLContext.version = HTTP_VERSION_1_0;                               // set version to 1.0
          socketHTML = used_tcp_socket;                                         // set the TCP/ip socket for the html message
        }
        else if (memcmp(getRequest, HTTP10_PUT_REQ, strlen(HTTP10_PUT_REQ))==0u)
        {
          strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"PUT");  // set method received to put
          HTMLContext.version = HTTP_VERSION_1_0;                               // set version to 1.0
          socketHTML = used_tcp_socket;                                         // set the TCP/ip socket for the html message
        }
        else if(memcmp(getRequest, HTTP09_GET_REQ, strlen(HTTP09_GET_REQ))==0u)      // was a get request version 1.1
        {
          strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"GET");  // set method received to get
          HTMLContext.version = HTTP_VERSION_0_9;                               // set version to 0.9
          socketHTML = used_tcp_socket;                                         // set the TCP/ip socket for the html message
        }
        else if (memcmp(getRequest, HTTP09_PUT_REQ, strlen(HTTP09_PUT_REQ))==0u)
        {
          strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"PUT");  // set method received to put
          HTMLContext.version = HTTP_VERSION_0_9;                               // set version to 0.9
          socketHTML = used_tcp_socket;                                         // set the TCP/ip socket for the html message
        }
        if (HTMLContext.version == HTTP_VERSION_1_1)
        {
           HTMLConn.bytes_received =0U;                                         // Reset the counter of bytes
           HTMLConn.bytes_to_read = socketHTML->dataLength;
           while(used_tcp_socket->dataLength--)                                 // Keep filling TCP Buffer until TCP receive buffer is empty
           {
               HTMLConn.buf[HTMLConn.bytes_received] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
               HTMLConn.bytes_received=++HTMLConn.bytes_received % INT16_MAX;
               if ( NX_CONNECTION_BUF <= HTMLConn.bytes_received )              // counted past the end of the TCP receive buffer
               {
                  // ------ Want to flush this ?
                  break;                                                        // return with no reply message was longer than max expected for http
               }
           }
        }
        break;
     
        default:
        break;
     }
     break;
     
     case HTTPS_PORT:                                                           // https
     switch(used_tcp_socket->state)                                             // for the states of the sockets
     {
        case TCP_STATE_ESTABLISHED:                                             // Socket ready to send or receive
        switch (HTMLConn.state)
        {
           case C_CONNECTING:
           break;
           
           case C_HANDSHAKING:
           break;
           
           case C_WRITING:
           break;
           
           case C_READING_HEADERS:
           for(len = 0u; len < strlen(HTTP11_GET_REQ); len++)                   // get 14 first bytes only of the request, the rest does not matter here
           {
              getRequest[len] = Net_Ethernet_Intern_getByte();                  // read the BYTE stream
           }
           getRequest[len] = 0u;
           if(memcmp(getRequest, HTTP11_GET_REQ, strlen(HTTP11_GET_REQ))==0u)   // was a get request version 1.1
           {
             HTMLConn.state = C_READING_BODY;                                   // now read the message body
             HTMLConn.bytes_received = 0u;                                      // reset the message reciewed length
             strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"GET");  // set method received to get
             HTMLContext.version = HTTP_VERSION_1_1;                            // set version to 1.1
             socketHTML = used_tcp_socket;                                      // set the TCP/ip socket for the html message
           }
           else if (memcmp(getRequest, HTTP11_PUT_REQ, strlen(HTTP11_PUT_REQ))==0u)
           {
              HTMLConn.state = C_READING_BODY;                                   // now read the message body
              HTMLConn.bytes_received = 0u;                                      // reset the message reciewed length
              strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"PUT");  // set method received to put
              HTMLContext.version = HTTP_VERSION_1_1;                           // set version to 1.1
              socketHTML = used_tcp_socket;                                     // set the TCP/ip socket for the html message
           }
           else if (memcmp(getRequest, HTTP11_POST_REQ, strlen(HTTP11_POST_REQ))==0u)
           {
              HTMLConn.state = C_READING_BODY;                                   // now read the message body
              HTMLConn.bytes_received = 0u;                                      // reset the message reciewed length
              strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"POST");  // set method received to post
              HTMLContext.version = HTTP_VERSION_1_1;                           // set version to 1.1
              socketHTML = used_tcp_socket;                                     // set the TCP/ip socket for the html message
           }
           else if(memcmp(getRequest, HTTP10_GET_REQ, strlen(HTTP10_GET_REQ))==0u)      // was a get request version 1.1
           {
              strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"GET");  // set method received to get
              HTMLContext.version = HTTP_VERSION_1_0;                           // set version to 1.0
              socketHTML = used_tcp_socket;                                     // set the TCP/ip socket for the html message
           }
           else if (memcmp(getRequest, HTTP10_PUT_REQ, strlen(HTTP10_PUT_REQ))==0u)
           {
              strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"PUT");  // set method received to put
              HTMLContext.version = HTTP_VERSION_1_0;                           // set version to 1.0
              socketHTML = used_tcp_socket;                                     // set the TCP/ip socket for the html message
           }
           else if (memcmp(getRequest, HTTP10_POST_REQ, strlen(HTTP10_POST_REQ))==0u)
           {
              HTMLConn.state = C_READING_BODY;                                   // now read the message body
              strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"POST");  // set method received to post
              HTMLContext.version = HTTP_VERSION_1_0;                           // set version to 1.1
              socketHTML = used_tcp_socket;                                     // set the TCP/ip socket for the html message
           }
           else if(memcmp(getRequest, HTTP09_GET_REQ, strlen(HTTP09_GET_REQ))==0u)      // was a get request version 1.1
           {
              strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"GET");  // set method received to get
              HTMLContext.version = HTTP_VERSION_0_9;                           // set version to 0.9
              socketHTML = used_tcp_socket;                                     // set the TCP/ip socket for the html message
           }
           else if (memcmp(getRequest, HTTP09_PUT_REQ, strlen(HTTP09_PUT_REQ))==0u)
           {
              strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"PUT");  // set method received to put
              HTMLContext.version = HTTP_VERSION_0_9;                           // set version to 0.9
              socketHTML = used_tcp_socket;                                     // set the TCP/ip socket for the html message
           }
           else if (memcmp(getRequest, HTTP09_POST_REQ, strlen(HTTP09_POST_REQ))==0u)
           {
              HTMLConn.state = C_READING_BODY;                                   // now read the message body
              strcpy((unsigned char*) &HTMLContext.method1,(unsigned char*)"POST");  // set method received to post
              HTMLContext.version = HTTP_VERSION_0_9;                           // set version to 1.1
              socketHTML = used_tcp_socket;                                     // set the TCP/ip socket for the html message
           }
           else                                                                 // unknown packetf
           { /*just for sanity */   }
           break;
           
           case C_READING_BODY:
           if (HTMLContext.version == HTTP_VERSION_1_1)
           {
              HTMLConn.bytes_received =0U;                                      // Reset the counter of bytes
              HTMLConn.bytes_to_read = socketHTML->dataLength;
              while(used_tcp_socket->dataLength--)                              // Keep filling TCP Buffer until TCP receive buffer is empty
              {
                 HTMLConn.buf[HTMLConn.bytes_received] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
                 HTMLConn.bytes_received=++HTMLConn.bytes_received % INT16_MAX;
                 if ( NX_CONNECTION_BUF <= HTMLConn.bytes_received )            // counted past the end of the TCP receive buffer
                 {
                    // ------ Want to flush this ?
                    break;                                                      // return with no reply message was longer than max expected for http
                 }
              }
              HTMLConn.buf[HTMLConn.bytes_received] = '\0';                     // terminate the buffer
              HTMLConn.state = C_READ_COMPLETED;                                // completed reading body
           }
           break;
           
           case C_READ_COMPLETED:
           break;
           
           default:
           break;
        }
        break;

        default:
        break;
     }
     break;
     
  }

#endif
#if defined(YI_CAM_USED)

     if ((YiCam_DATA.State != ETH_ACK_OLD_DATA_REQ) && (g_YiCamReqState == 5u)) // We have not been asked to re-transmit old data and have new data to send
     {
         memcpy(xyTcpSendBuff,g_XYtcpBuffer,sizeof(g_XYtcpBuffer));             // Copy the new message to the send buffer
     }
     
     if (used_tcp_socket->open==0)                                              // Socket is not busy
     {
        switch(used_tcp_socket->state)                                          // for the states of the sockets
        {
#ifdef outdoinmain                                                              // causes cross calling
           case TCP_STATE_CLOSED:                                               // We dont have a connection
           XY_retTCP=XY_OpenTCPSocketRawJson( &used_tcp_socket );               // try to open a socket
           switch(XY_retTCP)
           {
              case TCP_NO_SYN_SEND:                                             // didnt transmit SYN
              XY_retTCP=XY_OpenTCPSocketRawJson( &used_tcp_socket );            // try again to open a socket
              break;

              case TCP_SYN_SUCCESS:                                             // continue the SYN was sent
              break;

              case TCP_NO_SOCK:                                                 // no socket
              YiCam.Link_Alive   = FALSE;
              break;

              case TCP_NO_ARP:                                                  // no arp
              YiCam.Link_Alive   = FALSE;
              break;

              default:                                                          // invalid return code
              break;
           }
           break;

           case TCP_STATE_LISTEN:                                               // Server is listening for a SYN or ACK from the client
           break;
#endif
           case TCP_STATE_ESTABLISHED:                                          // Socket ready to send or receive
           //
           // ===================== Now Send Any Data =============================
           //
           YiSocket=used_tcp_socket;                                            // set the application socket
           if ((YiCam_DATA.State == ETH_ACK_OLD_DATA_REQ) && ((g_YiCamReqState == 5u) || (g_YiCamReqState == 6u)))
           {
               if((Net_Ethernet_Intern_putBytesTCP(xyTcpSendBuff, strlen(xyTcpSendBuff), used_tcp_socket))==strlen(xyTcpSendBuff))  // may be faster to put all bytes
               {
                  switch(g_YiCamReqState)
                  {
                     case 5u:                                                   // We sent a request for a start token
                     g_YiCamReqState == 1u;                                     // wait for a response message to the stream start token request
                     // hmiReqActive.getToken=0U;                                  not sure this is needed
                     break;
                 
                     case 6u:                                                   // we sent a message to do a camera action
                     g_YiCamReqState == 3u;                                     // wait for a response message to request made
                     resetXYActions();                                          // Reset the stimulus as the message was sent
                     break;

                     case 7u:                                                   // we sent a message to do a camera action
                     g_YiCamReqState == 11u;                                    // wait for a response message to request made
                     break;
                  
                     default:                                                   // request was re-send old data
                     YiCam_DATA.State == ETH_OLD_DATA_SENT;                     // mark the port state as we have sent the old data
                     break;
                  }
               }
//
//              ================== Disconnect ? =====================================
//
             if( Net_Ethernet_Intern_bufferEmptyTCP(used_tcp_socket)&&(dat_pos >= strlen(xyTcpSendBuff)) ) // If all bytes written, and TCP Tx buffer empty, We can close connection.
             {
                  Net_Ethernet_Intern_disconnectTCP(used_tcp_socket);           // Not sure we should disconnect the socket every send yes for http ?
             }
           }
           sprintf(rcvIpAddr,"%d.%d.%d.%d",used_tcp_socket->remoteIP[0],used_tcp_socket->remoteIP[1],used_tcp_socket->remoteIP[2],used_tcp_socket->remoteIP[3]);
           if(!strcmp(rcvIpAddr,XY_USE_IP))                                     // Check if we have received from the Xiong Yi Cam
           {
               YiCam.Link_Alive   = TRUE;
               YiCam.Link_Alive_Time=Heart.Time;  
               if(YiCam.Link_Alive_Was != YiCam.Link_Alive)
               {
                   YiCam.Link_Alive_Was = YiCam.Link_Alive;                     // set link alive status
                //Status(ETHER_LINK_STATE,AIR_LINK_UP);
               }

               switch(used_tcp_socket->destPort)                                // Test the destination port for the received TCP packet
               {
                // ======================  JSON message ======================== From Yi Cam
                  case XY_DEF_PORT:                                             // The JSON port of the Yi Action Camera
                  YiCam_DATA.TCP_Index =0U;                                     // Reset buffer index
                  YiCam_DATA.TCP_Buffer_Len = used_tcp_socket->dataLength;
                  while(used_tcp_socket->dataLength--)                          // Keep filling YiCam_DATA.UDP_Buffer until TCP receive buffer is empty (@@strut name should change)
                  {
                      YiCam_DATA.TCP_Buffer[YiCam_DATA.TCP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
                      YiCam_DATA.TCP_Index=++YiCam_DATA.TCP_Index % UINT8_MAX;
                      if ( XY_MSG_MAX_LEN <= YiCam_DATA.TCP_Index )             // counted past the end of the TCP receive buffer
                      {
                      // ----- Want to flush this ???
                      break;                                                    // break from the loop
                      }
                  }
                  YiCam_DATA.TCP_Index--;                                       // Number of bytes received is minus one
                  YiCam_DATA.State = ETH_WAIT_ACK_CRC;                          // Set the port to received a packet and tell to send a dummy ack
                  break;                                                        // Message was sent to us from the Yi Action Cam Server on JSON port

                  case TELNET_PORT:                                             // The the Yi Action Camera connected via telnet session
                  YiCam_DATA.TCP_Index =0U;                                     // Reset buffer index
                  YiCam_DATA.TCP_Buffer_Len = used_tcp_socket->dataLength;
                  while(used_tcp_socket->dataLength--)                          // Keep filling YiCam_DATA.UDP_Buffer until TCP receive buffer is empty (@@strut name should change)
                  {
                     YiCam_DATA.TCP_Buffer[YiCam_DATA.TCP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
                     YiCam_DATA.TCP_Index=++YiCam_DATA.TCP_Index % UINT8_MAX;
                     if ( XY_MSG_MAX_LEN <= YiCam_DATA.TCP_Index )              // counted past the end of the TCP receive buffer
                     {
                     // ------ Want to flush this ?
                     break;                                                     // return with no reply message was longer than max expected for Yi Action Cam
                     }
                  }
                  YiCam_DATA.TCP_Index--;                                       // Number of bytes received is minus one
                  YiCam_DATA.State = ETH_WAIT_ACK_CRC;                          // Set the port to received a packet and tell to send a dummy ack
                  break;                                                        // Message was sent to us from the Yi Action Cam Server on JSON port

                  default:                                                      // An unknown TCP message
                  break;
               }
           }
           break;

           case TCP_STATE_SYN_SENT:                                             // You need to look for a SYN
           break;

           case TCP_STATE_FIN_WAIT_1:                                           // You sent a FIN
           break;

           case TCP_STATE_FIN_WAIT_2:                                           // You wait for a remote FIN
           break;

           case TCP_STATE_RETRANSMIT:                                           // Asked to re-transmit
           break;
        }                                                                       // END CASE SOCKET STATE
     }
     else                                                                       // Socket is busy
     {
         // wait for the socket to become free or exit with an error
     }
#endif                                                                          // ===================== eNd use Yi Cam ===================================

#if defined(SEQ_CAM_USED)
     if (used_tcp_socket->destPort==SEQ_DEF_PORT)
     {
        switch(used_tcp_socket->state)                                          // for the states of the sockets
        {
#ifdef outdoinmain
           case TCP_STATE_CLOSED:                                               // We dont have a connection
           SEQ_retTCP=SEQ_OpenTCPSockethttpJson( &used_tcp_socket );            // try to open a socket
           switch(SEQ_retTCP)
           {
              case TCP_NO_SYN_SEND:                                             // didnt transmit SYN
              SEQ_retTCP=SEQ_OpenTCPSockethttpJson( &used_tcp_socket );         // try again to open a socket
              break;

              case TCP_SYN_SUCCESS:                                             // continue the SYN was sent
              break;

              case TCP_NO_SOCK:                                                 // no socket
              ParrotSeq.Link_Alive   = FALSE;
              break;

              case TCP_NO_ARP:                                                  // no arp
              ParrotSeq.Link_Alive   = FALSE;
              break;

              default:                                                          // invalid return code
              break;
           }
           break;

           case TCP_STATE_LISTEN:                                               // Server is listening for a SYN or ACK from the client
           break;
#endif
           case TCP_STATE_ESTABLISHED:
           // if((Net_Ethernet_Intern_putBytesTCP(xyTcpSendBuff, strlen(xyTcpSendBuff), used_tcp_socket))==strlen(xyTcpSendBuff))
           sprintf(rcvIpAddr,"%d.%d.%d.%d",used_tcp_socket->remoteIP[0u],used_tcp_socket->remoteIP[1u],used_tcp_socket->remoteIP[2u],used_tcp_socket->remoteIP[3u]);
           if(!strcmp(rcvIpAddr,SEQ_USE_IP))                                    // Check if we have received from the Parrot Seq Cam
           {
               ParrotSeq.Link_Alive = true;
               ParrotSeq.Link_Alive_Time=Heart.Time;
               if(ParrotSeq.Link_Alive_Was != ParrotSeq.Link_Alive)
               {
                  ParrotSeq.Link_Alive_Was = ParrotSeq.Link_Alive;              // set link alive status
               }

               switch(used_tcp_socket->destPort)                                // Test the destination port for the received TCP packet
               {
                  // ======================  JSON message ============================== From http get request
                  case SEQ_DEF_PORT:                                            // The http port for the Parrot Sequoia Camera
                  SeqCam_DATA.TCP_Index =0U;                                    // Reset buffer index
                  SeqCam_DATA.TCP_Buffer_Len = used_tcp_socket->dataLength;
                  while(used_tcp_socket->dataLength--)                          // Keep filling SeqCam_DATA.UDP_Buffer until TCP receive buffer is empty (@@strut name should change)
                  {
                      SeqCam_DATA.TCP_Buffer[SeqCam_DATA.TCP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
                      SeqCam_DATA.TCP_Index=++SeqCam_DATA.TCP_Index % UINT8_MAX;
                      if ( SEQ_MSG_MAX_LEN <= SeqCam_DATA.TCP_Index )           // counted past the end of the TCP receive buffer
                      {
                      // ----- Want to flush this ???
                      break;                                                    // break from the loop
                      }
                  }
                  SeqCam_DATA.TCP_Index--;                                      // Number of bytes received is minus one
                  SeqCam_DATA.State = ETH_WAIT_ACK_CRC;                         // Set the port to received a packet and tell to send a dummy ack
                  break;
                }
           }
           break;
       
           case TCP_STATE_SYN_SENT:                                             // You need to look for a SYN
           break;

           case TCP_STATE_FIN_WAIT_1:                                           // You sent a FIN
           break;

           case TCP_STATE_FIN_WAIT_2:                                           // You wait for a remote FIN
           break;

           case TCP_STATE_RETRANSMIT:                                           // Asked to re-transmit
           break;
       }                                                                        // END CASE SOCKET STATE
     }
     else                                                                       // Socket is busy
     {
         // wait for the socket to become free or exit with an error
     }
#endif                                                                          // ===================== eNd use Sequoia Cam ==============================
#if defined(PENTAX_CAM_USED)
#endif                                                                          // ===================== eNd use Pentax Cam ==============================
#if defined(CBOR_COMS_USED)
#endif                                                                          // ===================== eNd use CBOR iOt communication  =================
#if defined(JSON_COMS_USED)
#endif                                                                          // ===================== eNd use JSON webservice  ========================

#if defined(REMOTE_TCP_SBGC)
  if (used_tcp_socket->destPort==SBGC_Dest_Port)                                // remote simpleBGC message sent over tcp
  {
     switch(used_tcp_socket->state)
     {
        case TCP_STATE_ESTABLISHED:                                             // connection is established
        while(used_tcp_socket->dataLength--)                                    // Keep sending a BYTE until we have no more
        {
            UART2_Write(Net_Ethernet_Intern_getByte());                         // remote SBGC message over TCP to serial
        }
        remSBGCSocket=used_tcp_socket;                                          // set soecket
        break;
        
        default:
        break;
     }
  }
#endif

#if defined(REMOTE_TCP_RUN_CAM)
  if (used_tcp_socket->destPort==RC_Dest_Port)
  {
     switch(used_tcp_socket->state)
     {
        case TCP_STATE_ESTABLISHED:                                             // connection is established
        while(used_tcp_socket->dataLength--)                                    // Keep sending a BYTE until we have no more
        {
           UART5_Write(Net_Ethernet_Intern_getByte());                          // remote runCam message over tcp to serial
        }
        remRunCamSocket=used_tcp_socket;                                        // set socket
        break;
        
        default:
        break;
     }
  }
#endif

#if defined(REMOTE_TCP_AMP_XY_CAM)
  if (used_tcp_socket->destPort==XS_Dest_Port)
  {
     switch(used_tcp_socket->state)
     {
        case TCP_STATE_ESTABLISHED:                                             // connection is established
        while(used_tcp_socket->dataLength--)                                    // Keep sending a BYTE until we have no more
        {
           UART4_Write(Net_Ethernet_Intern_getByte());                          // remote AMP encoder message over TCP to serial
        }
        remAmpSocket=used_tcp_socket;                                           // set socket
        break;
        
        default:
        break;
     }
  }
#endif

#if defined(MODBUS_TCP)
  if (( used_tcp_socket->destPort == MB_TCP_PORT ) || ( used_tcp_socket->destPort == MB_TCP_SECURE_PORT ))    // MODBUS TCP is on this Remote machines port 502
  {
     switch(used_tcp_socket->state)
     {
        case TCP_STATE_ESTABLISHED:                                             // connection is established
        if (ETH_PROCESSED_TCP == Modbus_DATA.State)
        {
           Modbus_DATA.TCP_Index =0U;                                           // Reset buffer index
        }
        Modbus_DATA.TCP_Buffer_Len = used_tcp_socket->dataLength;
        while(used_tcp_socket->dataLength--)                                    // Keep filling Modbus_DATA.UDP_Buffer until TCP receive buffer is empty (@@strut name should change)
        {
           Modbus_DATA.TCP_Buffer[Modbus_DATA.TCP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
           Modbus_DATA.TCP_Index=++Modbus_DATA.TCP_Index % UINT8_MAX;
           if ( MB_MAX_ADU_SIZE <= Modbus_DATA.TCP_Index )                      // counted past the end of the TCP receive buffer
           {
            // ----- Want to flush this ???
               Modbus_DATA.TCP_Index =0U;
               break;                                                           // break from the loop
           }
        }
        Modbus_DATA.TCP_Index--;                                                // Number of bytes received is minus one
        Modbus_DATA.State = ETH_WAIT_ACK_CRC;                                   // Set the port to received a packet and tell to send a dummy ack
        MbusSocket=used_tcp_socket;                                             // set socket
        break;
        
        default:
        break;
     }
  }
#endif

#if defined(LW3_SWITCH_USED)
  if ( used_tcp_socket->destPort == TCP_LW3_PORT )                              // LW3 is on this Remote machines port 6107
  {
     switch(used_tcp_socket->state)
     {
        case TCP_STATE_ESTABLISHED:                                             // connection is established
        if (ETH_PROCESSED_TCP == LW3_DATA.State)
        {
            LW3_DATA.TCP_Index =0U;                                             // Reset buffer index
        }
        LW3_DATA.TCP_Buffer_Len = used_tcp_socket->dataLength;
        while(used_tcp_socket->dataLength--)                                    // Keep filling Modbus_DATA.UDP_Buffer until TCP receive buffer is empty (@@strut name should change)
        {
           LW3_DATA.TCP_Buffer[LW3_DATA.TCP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
           LW3_DATA.TCP_Index=++LW3_DATA.TCP_Index % UINT8_MAX;
           if ( TCP_IP_MAX_BYTES <= LW3_DATA.TCP_Index )                        // counted past the end of the TCP receive buffer
           {
            // ----- Want to flush this ???
               LW3_DATA.TCP_Index =0U;
               break;                                                           // break from the loop
           }
        }
        LW3_DATA.TCP_Index--;                                                   // Number of bytes received is minus one
        LW3_DATA.State = ETH_WAIT_ACK_CRC;                                      // Set the port to received a packet and tell to send a dummy ack
        Lw3Socket=used_tcp_socket;                                              // set socket
        break;

        default:
        break;
     }
  }
#endif

#if defined(MICROSCAN_OUT_TCP )
  if (( used_tcp_socket->destPort == MIRSC_TELNET_PORT1 ) || ( used_tcp_socket->destPort == MIRSC_TELNET_PORT2 ))    // MODBUS TCP is on this Remote machines port 502
  {
     switch(used_tcp_socket->state)
     {
        case TCP_STATE_ESTABLISHED:                                             // connection is established
        if (ETH_PROCESSED_TCP == MiCroScaN_DATA.State)
        {
           MiCroScaN_DATA.TCP_Index =0U;                                        // Reset buffer index
        }
        MiCroScaN_DATA.TCP_Buffer_Len = used_tcp_socket->dataLength;
        while(used_tcp_socket->dataLength--)                                    // Keep filling Modbus_DATA.UDP_Buffer until TCP receive buffer is empty (@@strut name should change)
        {
           MiCroScaN_DATA.TCP_Buffer[Modbus_DATA.TCP_Index] = Net_Ethernet_Intern_getByte();   // get each byte and put it in the buffer
           MiCroScaN_DATA.TCP_Index=++MiCroScaN_DATA.TCP_Index % UINT8_MAX;
           if ( TCP_IP_MAX_BYTES <= MiCroScaN_DATA.TCP_Index )                  // counted past the end of the TCP receive buffer
           {
            // ----- Want to flush this ???
               MiCroScaN_DATA.TCP_Index =0U;
               break;                                                           // break from the loop
           }
        }
        MiCroScaN_DATA.TCP_Index--;                                             // Number of bytes received is minus one
        MiCroScaN_DATA.State = ETH_WAIT_ACK_CRC;                                // Set the port to received a packet and tell to send a dummy ack
        MiCroScanSocket=used_tcp_socket;                                        // set socket
        break;

        default:
        break;
     }
  }
#endif

#ifdef USE_TCP_CAN
  if (used_tcp_socket->destPort==Can2destPort)                                  // Process TCP packets for CAN
  {
     switch(used_tcp_socket->state)
     {
        case TCP_STATE_ESTABLISHED:                                             // connection is established
        Lo(Can2_TX_MSG_ID)=Net_Ethernet_Intern_getByte();                           // Load first two bytes into message ID
        Hi(Can2_TX_MSG_ID)=Net_Ethernet_Intern_getByte();

        reqLength = used_tcp_socket->dataLength;                                // load the remaining of the message into Can2_UDP_RX_Packet EXLUDING the MASK which is the last byte in the UDP Packet.
        while((counter < reqLength) && (counter<MAX_CAN_MSG_LEN))               // count the bytes recieved or 8 bytes (max can message)
        {
           Can2_TCP_RX_Packet[counter] = Net_Ethernet_Intern_getByte();
           counter=++counter % UINT16_MAX;
        }
        counter=0u;
        sendValue=0u;
        while((!sendValue)&&(counter < CAN_RESEND_CNT))                             // !!!!!! ============ consider separate can handler too busy reset no while but if
        {
          sendValue=CAN2Write((uint32_t) Can2_TX_MSG_ID,(unsigned char*) Can2_TCP_RX_Packet,(uint16_t) reqLength,(uint16_t) Can2_Send_Flags);  // Sent message out CAN2
          counter=++counter % UINT16_MAX;
          asm nop;
        }
        tcpCanSocket=used_tcp_socket;
        break;
       
       default:
       break;
     }
  }
#endif                                                                          // ======================== eNd use Can over UDP  =======================

#endif                                                                          // ======================== eNd TCP/IP Stack used =======================
 }

/*-----------------------------------------------------------------------------
 *      Init_MCU():  Initialize the MCU (CPU) and RTC for the application
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Init_MCU()
{
  // RTCC registers unlock procedure
  SYSKEY = 0xAA996655UL;                                                        // Write first unlock key to SYSKEY
  SYSKEY = 0x556699AAUL;                                                        // Write second unlock key to SYSKEY
  RTCWREN_bit = 1U;                                                             // RTC Value registers can be written and read.
  ON__RTCCON_bit= 0U;                                                           // Turn off the RTCC module
  Delay_100ms;
  while(RTCCLKON_bit)                                                           // Wait for clock to be turned on
  //  ;
  RTCTIME = 0x0U;                                                               // Reset uptime
  RTCDATE = 0x0U;                                                               // Reset update
  
  //while(ALRMSYNC_bit)       // Wait for clock to be turned off
  //  ;
    
  RTSECSEL_bit = 0U;                                                            // Use RTC interrupt
  ON__RTCCON_bit= 1U;                                                           // Turn on the RTCC module
  //while(!RTCCLKON_bit)      // Wait for clock to be turned on
}

#ifdef TIMER1_NEEDED
/*-----------------------------------------------------------------------------
 *      Init_Timer1():  Initialize interrupt timer no. 1
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Init_Timer1() {

  T1CON     = 0x8030U;                                                          // Timer 1 prescale
  T1IP0_bit = 1U;                                                               // set interrupt
  T1IP1_bit = 1U;                                                               // priority
  T1IP2_bit = 1U;                                                               // to 7

  TCKPS0_bit = 1U;                                                              // Set Timer Input Clock
  TCKPS1_bit = 1U;                                                              // Prescale value to 1:256
  T1IF_bit   = 0U;

  T1IE_bit = 1U;                                                                // Enable Timer1 Interrupt
  PR1 = 62500UL;                                                                // Load period register with 200 ms (timer duration)
  TMR1 = 0U;                                                                    // set to start from 0
  ON__T1CON_bit = 1U;                                                           // Enable Timer1

}
#endif

#ifdef TIMER2_NEEDED
/*-----------------------------------------------------------------------------
 *      Init_Timer2():  Initialize interrupt timer no. 2
 *      combine Timer2 and 3 for a 400 ms timeout for UART2
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Init_Timer2(){
  T2CON         = 0x70U;
  PR2           = 62500UL;
  IPC2SET       = 0b11001;                                                      // Set Priority 6 and sub 1 so it is below UART2
  IPC2CLR       = 0b00110;                                                      // Set Priority 6 and sub 1 so it is below UART2
  T2IF_bit      = 0U;                                                           // Clear If
  ON__T2CON_bit = 0U;                                                           // Disable Timer2
  T2IE_bit      = 1U;                                                           // Enable IF
  TMR2          = 0U;

}
/*-----------------------------------------------------------------------------
 *      Init_Timer2_3():  Initialize interrupt timer no. 3
 *      combine Timer2 and 3 for a 400 ms timeout for UART2
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Init_Timer2_3(){
  T2CON                 = 0x08U;
  T3CON                 = 0x0U;
  TMR2                  = 0u;
  TMR3                  = 0u;
  T3IS0_bit             = 0u;
  T3IS1_bit             = 1u;
  T3IP0_bit             = 0u;
  T3IP1_bit             = 1u;
  T3IP2_bit             = 1u;
  T3IF_bit              = 0u;
  T3IE_bit              = 1u;
  PR2                   = 18432UL;
  PR3                   = 488UL;
}
#endif

/*-----------------------------------------------------------------------------
 *      Init_Timer5():  Initialize interrupt timer no. 5
 *      This timer is used as an interrupt driven clock tick
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Init_Timer5()
{                                                                               // Initialise timer 5 to 200 ms - change PR5= 62500 if you want 200 ms
  T5CON         = 0x8070UL;                                                     // timer set point from timer tool
  T5IP0_bit     = 1u;
  T5IP1_bit     = 1u;
  T5IP2_bit     = 1u;                                                           // set priority to 0b111 (7)
  T5IF_bit      = 0u;                                                           // remove pending
  T5IE_bit       = 1u;                                                          // enable interrupt
  PR5           = 31250UL;                                                      // 100 ms timer interrupt for a global counter
  TMR5           = 0u;
}

/*-----------------------------------------------------------------------------
 *      Init_PHYPins():  Initialise the pins from MCU to PHY chip
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Init_PHYPins()
{
  TRISD11_bit = 0u;                                                             // ETH_ALT_MDC_BIT
  TRISD8_bit  = 1u;                                                             // ETH_ALT_MDIO_BIT

  TRISD6_bit  = 0u;                                                             // ETH_ALT_TXEN_BIT
  TRISF1_bit  = 0u;                                                             // ETH_ALT_TXD0_BIT
  TRISF0_bit  = 0u;                                                             // ETH_ALT_TXD1_BIT

  TRISG9_bit  = 1u;                                                             // ETH_ALT_RXCLK_BIT
  TRISG8_bit  = 1u;                                                             // ETH_ALT_RXDV_BIT
  TRISB12_bit = 1u;                                                             // ETH_ALT_RXD0_BIT
  TRISB13_bit = 1u;                                                             // ETH_ALT_RXD1_BIT
  TRISB11_bit = 1u;                                                             // ETH_ALT_RXERR_BIT
  //TrisB0_bit  = 0;                                                              // LED 1
  //TrisB1_Bit  = 0;                                                              // LED 2
  //TrisB2_Bit  = 0;                                                              // LED 3
}

/*-----------------------------------------------------------------------------
 *      Init_UART:  Initialise the required serial UART ports
 *  Parameters: uint16_t parityValue for SimpleBGC port
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Init_UART( uint16_t parityValue )
{
  uint16_t pb_divisor;                                                          // depends on PBDIV0 and 1.
  uint32_t periph_clock;                                                        // required for advanced function
  
#ifdef UART2_INTERUPT                                                           // SimpleBGC is connected on serial direct
  if ((PBDIV1_bit == 1u) && (PBDIV0_bit == 1u)) pb_divisor = 8u;
  else if ((PBDIV1_bit == 1u) && (PBDIV0_bit == 0u)) pb_divisor = 4u;
  else if ((PBDIV1_bit == 0u) && (PBDIV0_bit == 1u)) pb_divisor = 2u;
  else pb_divisor = 1u;
  periph_clock = Get_Fosc_kHz()/pb_divisor;                                     // calculated clock speed   you may have to set to 20000 hardcoded (try)
  
  UART2.Baud = 115200UL;                                                        // Set the Baud for SimpleBGC
  
  if (parityValue == _UART_8BIT_NOPARITY)
  {
     UART2_Init(UART2.Baud);                                                    // Set the Baud Rate for UART2
     UART2_Init_Advanced(UART2.Baud, periph_clock, _UART_HIGH_SPEED, _UART_8BIT_NOPARITY, _UART_ONE_STOPBIT);
  }
  else if (parityValue == _UART_8BIT_EVENPARITY)
  {
     UART2_Init_Advanced(UART2.Baud, periph_clock, _UART_HIGH_SPEED, _UART_8BIT_EVENPARITY, _UART_ONE_STOPBIT);
  }                                                                             // 9 bit or odd not supported by gimbal so ignore.
  
  U2IP0_bit = 0u;                                                               // Set UART2 interrupt
  U2IP1_bit = 1u;                                                               // Set interrupt priorities
  U2IP2_bit = 1u;                                                               // Set UART2 interrupt to level 6
  U2EIF_bit = 0u;                                                               // Clear IF
  U2RXIF_bit = 0u;                                                              // Clear RXIF
  U2EIE_bit = 0u;                                                               // Enable UART2 error interupts  (could also use   IEC1BITS.U2EIE =1)
  UTXISEL0_U2STA_bit = 0u;                                                      // Set UART IF to trigger when buffer is 3/4 full
  UTXISEL1_U2STA_bit = 1u;                                                      // "                                           "
  URXEN_U2STA_bit = 0u;                                                         // Disable UART 2 receiver to prevent craching uppon boot   ????
  UART2.Index=0u;                                                               // Set read index to 0
//  U2STA = U2STA & ~((1u<<7u) | (1u<<6u));                                       // enable interrupt on 1 or more char

  CHECON = (((CHECON&~(_CHECON_PFMWS_MASK))&~(_CHECON_PREFEN_MASK)) | ((2u<<_CHECON_PFMWS_POSITION) | (3u<<_CHECON_PREFEN_POSITION)));; // PFMWS=0x2 set the prefectch cache wait state to 2, as per the electrical characteristics data sheet PREFEN = 0x3 enable prefetch for cacheable and noncacheable memory
  BMXCON = BMXCON & ~(1u<<_BMXCON_BMXWSDRM_POSITION);                           // BMXWSDRM=0 0 data RAM access wait states
  INTCON = (INTCON | (1u<<_INTCON_MVEC_POSITION));                              // MVEC = 1 enable multi vector interrupts
  DDPCON = DDPCON & ~(1u<<_DDPCON_JTAGEN_POSITION);                             // JTAGEN=0 disable JTAG to get B10, B11, B12 and B13 back
  U2STA = U2STA & ~(3u<<_U2STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U2STA =  (U2STA | ((1u<<_U2STA_UTXEN_POSITION) | (1u<<_U2STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  //U2MODE = ((U2MODE&~(_U2MODE_UEN_MASK)) | ((2u<<_U2MODE_UEN_POSITION) | (1u<<_U2MODE_ON_POSITION)));  // UEN=2 configure hardware flow control using RTS and CTS ON=1 enable the uart
  U2RXIE_bit =1u;                                                               // Enable UART2 RX IF
#endif
#if defined(GPS_INCLUDED) || defined(GPS_INCLUDED2)                             // A GPS option has been selected on port 3

  UART3_Init(GPS_BAUD_RATE);                                                    // Set the Baud Rate for UART3
  UART_Set_Active(&UART3_Read, &UART3_Write, &UART3_Data_Ready, &UART3_Tx_Idle);// Initialise the handlers for UART3

  U3IP0_bit = 1u;                                                                // Set UART3 interrupt
  U3IP1_bit = 1u;                                                                // Set interrupt priorities
  U3IP2_bit = 0u;                                                                // Set UART3 interrupt to level 3
  U3EIF_bit = 0u;                                                                // Clear IF
  U3RXIF_bit = 0u;                                                               // Clear RXIF
  U3EIE_bit = 0u;                                                                // Enable UART3 error interupts  (could also use   IEC1BITS.U2EIE =1)
  U3RXIE_bit =0u;                                                                // Enable UART3 RX IF not 3/4 full as below
  UTXISEL0_U3STA_bit = 0u;                                                       // Set UART IF to trigger when buffer is 3/4 full
  UTXISEL1_U3STA_bit = 1u;                                                       // "                                           "
  URXEN_U3STA_bit = 0u;                                                          // Disable UART 3 receiver to prevent craching uppon boot
  U3STA = U3STA & ~(3u<<_U3STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U3STA =  (U3STA | ((1u<<_U3STA_UTXEN_POSITION) | (1u<<_U3STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  U3RXIE_bit =1u;                                                               // Enable UART3 RX IF
#endif
#ifdef UART4_INTERUPT                                                           // GPS requested

  UART4_Init(GPS_BAUD_RATE);                                                    // Set the Baud Rate for UART4
  UART_Set_Active(&UART4_Read, &UART4_Write, &UART4_Data_Ready, &UART4_Tx_Idle);// Initialise the handlers for UART4

  U4IP0_bit = 0u;                                                               // Set UART4 interrupt
  U4IP1_bit = 0u;                                                               // Set interrupt priorities
  U4IP2_bit = 1u;                                                               // Set UART4 interrupt to level 4
  U4EIF_bit = 0u;                                                               // Clear IF
  U4RXIF_bit = 0u;                                                              // Clear RXIF
  U4EIE_bit = 0u;                                                               // Enable UART4 error interupts  (could also use   IEC1BITS.U2EIE =1)
  U4RXIE_bit =0u;                                                               // Enable UART4 RX IF not 3/4 full as below
  UTXISEL0_U4STA_bit = 0u;                                                      // Set UART IF to trigger when buffer is 3/4 full
  UTXISEL1_U4STA_bit = 1u;                                                      // "                                           "
  URXEN_U4STA_bit = 0u;                                                         // Disable UART 4 receiver to prevent craching uppon boot
  UART4.Index=1u;                                                               // Set read index to 1
  U4STA = U4STA & ~(3u<<_U4STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U4STA =  (U4STA | ((1u<<_U4STA_UTXEN_POSITION) | (1u<<_U4STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  U4RXIE_bit =1u;                                                               // Enable UART4 RX IF
#endif
#ifdef UART6_INTERUPT                                                           // Liddar is requested

  UART6_Init(LIDDAR_BAUD_RATE);                                                 // Set the Baud Rate for UART6 to that of the Liddar
  UART_Set_Active(&UART6_Read, &UART6_Write, &UART6_Data_Ready, &UART6_Tx_Idle);// Initialise the handlers for UART6

  U6IP0_bit = 0u;                                                               // Set UART6 interrupt
  U6IP1_bit = 0u;                                                               // Set interrupt priorities
  U6IP2_bit = 1u;                                                               // Set UART6 interrupt to level 4
  U6EIF_bit = 0u;                                                               // Clear IF
  U6RXIF_bit = 0u;                                                              // Clear RXIF
  U6EIE_bit = 0u;                                                               // Enable UART6 error interupts  (could also use   IEC1BITS.U2EIE =1)
  U6RXIE_bit =0u;                                                               // Enable UART6 RX IF not 3/4 full as below
  UTXISEL0_U6STA_bit = 0u;                                                      // Set UART IF to trigger when buffer is 3/4 full
  UTXISEL1_U6STA_bit = 1u;                                                      // "                                           "
  URXEN_U6STA_bit = 0u;                                                         // Disable UART 6 receiver to prevent craching uppon boot
  U6STA = U6STA & ~(3u<<_U6STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U6STA =  (U6STA | ((1u<<_U6STA_UTXEN_POSITION) | (1u<<_U6STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  U6RXIE_bit =1u;                                                               // Enable UART6 RX IF
#endif
#if (defined(ROBOT_HELPER) && defined(UART1_INTERUPT))                          // Odrive is requested and connected on UART1

  UART1_Init(OD_BAUD_RATE);                                                     // Set the Baud Rate for UART1 to that of the odrive
  UART_Set_Active(&UART1_Read, &UART1_Write, &UART1_Data_Ready, &UART1_Tx_Idle);// Initialise the handlers for UART1

  U1IP0_bit = 0u;                                                               // Set UART1 interrupt
  U1IP1_bit = 0u;                                                               // Set interrupt priorities
  U1IP2_bit = 1u;                                                               // Set UART1 interrupt to level 4
  U1EIF_bit = 0u;                                                               // Clear IF
  U1RXIF_bit = 0u;                                                              // Clear RXIF
  U1EIE_bit = 0u;                                                               // Enable UART1 error interupts  (could also use   IEC1BITS.U2EIE =1)
  U1RXIE_bit = 1u;                                                              // Enable UART1 RX IF not 3/4 full as below
  //UTXISEL0_U1STA_bit = 0;                                                       // Set UART IF to trigger when buffer is 3/4 full
  UART1.Index=1u;                                                               // Set read index to 1
  //UTXISEL1_U1STA_bit = 1;                                                       // "                                           "
  //URXEN_U1STA_bit = 0;                                                          // Disable UART 1 receiver to prevent craching uppon boot
  U1STA = U1STA & ~(3u<<_U1STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U1STA =  (U1STA | ((1u<<_U1STA_UTXEN_POSITION) | (1u<<_U1STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  U1RXIE_bit =1u;                                                               // Enable UART1 RX IF
#endif  /* endif robot */
#ifdef UART5_INTERUPT                                                           // Run Cam is requested on serial UART5

  UART5_Init(RC_BAUD_RATE);                                                     // Set the Baud Rate for UART5 to that of the Run Cam
  UART_Set_Active(&UART5_Read, &UART5_Write, &UART5_Data_Ready, &UART5_Tx_Idle);// Initialise the handlers for UART5

  U5IP0_bit = 0u;                                                               // Set UART5 interrupt
  U5IP1_bit = 0u;                                                               // Set interrupt priorities
  U5IP2_bit = 1u;                                                               // Set UART5 interrupt to level 4
  U5EIF_bit = 0u;                                                               // Clear IF
  U5RXIF_bit = 0u;                                                              // Clear RXIF
  U5EIE_bit = 0u;                                                               // Enable UART error interupts  (could also use   IEC5BITS.U5EIE =1)
  U5RXIE_bit = 0u;                                                              // Enable UART RX IF not 3/4 full as below
  UTXISEL0_U5STA_bit = 0u;                                                      // Set UART IF to trigger when buffer is 3/4 full
  UTXISEL1_U5STA_bit = 1u;                                                      // "                                           "
  URXEN_U5STA_bit = 0u;                                                         // Disable UART 5 receiver to prevent crashing upon boot
  UART5.Index=1u;                                                               // Set read index to 1
  U5STA = U5STA & ~(3u<<_U5STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U5STA =  (U5STA | ((1u<<_U5STA_UTXEN_POSITION) | (1u<<_U5STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  U5RXIE_bit =1u;                                                               // Enable UART5 RX IF
#endif
#if defined(SERIAL_MAVLINK)                                                     /* serial mavlink has been selected to be compiled */
#if defined(MAVLINK_USE_UART1)                                                  /* ----- we chose port 1 ------*/
  UART1_Init(MavLinkBuf.Baud);                                                  // Set the Baud Rate for the defined UART to that defined as the MAVLINK serial port speed
  UART_Set_Active(&UART1_Read, &UART1_Write, &UART1_Data_Ready, &UART1_Tx_Idle);// Initialise the handlers for UART1

  U1IP0_bit = 0u;                                                               // Set UART1 interrupt
  U1IP1_bit = 0u;                                                               // Set interrupt priorities
  U1IP2_bit = 1u;                                                               // Set UART1 interrupt to level 4
  U1EIF_bit = 0u;                                                               // Clear IF
  U1RXIF_bit = 0u;                                                              // Clear RXIF
  U1EIE_bit = 0u;                                                               // Enable UART1 error interupts  (could also use   IEC1BITS.U2EIE =1)
  U1RXIE_bit = 0u;                                                              // Enable UART1 RX IF not 3/4 full as below
  MavLinkBuf.Index=1u;                                                          // Set read index to 1
  U1STA = U1STA & ~(3u<<_U1STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U1STA =  (U1STA | ((1u<<_U1STA_UTXEN_POSITION) | (1u<<_U1STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  U1RXIE_bit =1u;                                                               // Enable UART1 RX IF
#elif defined(MAVLINK_USE_UART2)                                                /* ------ we chose port 2 -------*/
  UART2_Init(MavLinkBuf.Baud);                                                  // Set the Baud Rate for UART2

  U2IP0_bit = 0u;                                                               // Set UART2 interrupt
  U2IP1_bit = 1u;                                                               // Set interrupt priorities
  U2IP2_bit = 1u;                                                               // Set UART2 interrupt to level 6
  U2EIF_bit = 0u;                                                               // Clear IF
  U2RXIF_bit = 0u;                                                              // Clear RXIF
  U2EIE_bit = 0u;                                                               // Enable UART2 error interupts  (could also use   IEC1BITS.U2EIE =1)
  U2RXIE_bit =0u;                                                               // Enable UART2 RX IF
  UTXISEL0_U2STA_bit = 0u;                                                      // Set UART IF to trigger when buffer is 3/4 full
  UTXISEL1_U2STA_bit = 1u;                                                      // "                                           "
  URXEN_U2STA_bit = 0u;                                                         // Disable UART 2 receiver to prevent craching uppon boot
  MavLinkBuf.Index=1u;                                                          // Set read index to 1
  U2STA = U2STA & ~(3u<<_U2STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U2STA =  (U2STA | ((1u<<_U2STA_UTXEN_POSITION) | (1u<<_U2STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  U2RXIE_bit =1u;                                                               // Enable UART2 RX IF
#elif defined(MAVLINK_USE_UART3)                                                /* ------ we chose port 3 -------*/
  UART3_Init(MavLinkBuf.Baud);                                                  // Set the Baud Rate for UART3
  UART_Set_Active(&UART3_Read, &UART3_Write, &UART3_Data_Ready, &UART3_Tx_Idle);// Initialise the handlers for UART3

  U3IP0_bit = 1u;                                                               // Set UART3 interrupt
  U3IP1_bit = 1u;                                                               // Set interrupt priorities
  U3IP2_bit = 0u;                                                               // Set UART3 interrupt to level 3
  U3EIF_bit = 0u;                                                               // Clear IF
  U3RXIF_bit = 0u;                                                              // Clear RXIF
  U3EIE_bit = 0u;                                                               // Enable UART3 error interupts  (could also use   IEC1BITS.U2EIE =1)
  U3RXIE_bit =0u;                                                               // Enable UART3 RX IF not 3/4 full as below
  UTXISEL0_U3STA_bit = 0u;                                                      // Set UART IF to trigger when buffer is 3/4 full
  UTXISEL1_U3STA_bit = 1u;                                                      // "                                           "
  URXEN_U3STA_bit = 0u;                                                         // Disable UART 3 receiver to prevent craching uppon boot
  MavLinkBuf.Index=1u;                                                          // Set read index to 1
  U3STA = U3STA & ~(3u<<_U3STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U3STA =  (U3STA | ((1u<<_U3STA_UTXEN_POSITION) | (1u<<_U3STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  U3RXIE_bit =1u;                                                               // Enable UART3 RX IF
#elif defined(MAVLINK_USE_UART4)                                                /* ------ we chose port 4 -------*/
  UART4_Init(MavLinkBuf.Baud);                                                  // Set the Baud Rate for UART4
  UART_Set_Active(&UART4_Read, &UART4_Write, &UART4_Data_Ready, &UART4_Tx_Idle);// Initialise the handlers for UART4

  U4IP0_bit = 0u;                                                               // Set UART4 interrupt
  U4IP1_bit = 0u;                                                               // Set interrupt priorities
  U4IP2_bit = 1u;                                                               // Set UART4 interrupt to level 4
  U4EIF_bit = 0u;                                                               // Clear IF
  U4RXIF_bit = 0u;                                                              // Clear RXIF
  U4EIE_bit = 0u;                                                               // Enable UART4 error interupts  (could also use   IEC1BITS.U2EIE =1)
  U4RXIE_bit =0u;                                                               // Enable UART4 RX IF not 3/4 full as below
  UTXISEL0_U4STA_bit = 0u;                                                      // Set UART IF to trigger when buffer is 3/4 full
  UTXISEL1_U4STA_bit = 1u;                                                      // "                                           "
  URXEN_U4STA_bit = 0u;                                                         // Disable UART 4 receiver to prevent craching uppon boot
  MavLinkBuf.Index=1u;                                                          // Set read index to 1
  U4STA = U4STA & ~(3u<<_U4STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U4STA =  (U4STA | ((1u<<_U4STA_UTXEN_POSITION) | (1u<<_U4STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  U4RXIE_bit =1u;                                                               // Enable UART4 RX IF
#elif defined(MAVLINK_USE_UART5)                                                /* ------ we chose port 5 -------*/
  UART5_Init(MavLinkBuf.Baud);                                                  // Set the Baud Rate for UART5 to that of the Run Cam
  UART_Set_Active(&UART5_Read, &UART5_Write, &UART5_Data_Ready, &UART5_Tx_Idle);// Initialise the handlers for UART5

  U5IP0_bit = 0u;                                                               // Set UART5 interrupt
  U5IP1_bit = 0u;                                                               // Set interrupt priorities
  U5IP2_bit = 1u;                                                               // Set UART5 interrupt to level 4
  U5EIF_bit = 0u;                                                               // Clear IF
  U5RXIF_bit = 0u;                                                              // Clear RXIF
  U5EIE_bit = 0u;                                                               // Enable UART error interupts  (could also use   IEC5BITS.U5EIE =1)
  U5RXIE_bit = 0u;                                                              // Enable UART RX IF not 3/4 full as below
  UTXISEL0_U5STA_bit = 0u;                                                      // Set UART IF to trigger when buffer is 3/4 full
  UTXISEL1_U5STA_bit = 1u;                                                      // "                                           "
  URXEN_U5STA_bit = 0u;                                                         // Disable UART 5 receiver to prevent crashing upon boot
  MavLinkBuf.Index=1u;                                                          // Set read index to 1
  U5STA = U5STA & ~(3u<<_U5STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U5STA =  (U5STA | ((1u<<_U5STA_UTXEN_POSITION) | (1u<<_U5STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  U5RXIE_bit =1u;                                                               // Enable UART5 RX IF
#elif defined(MAVLINK_USE_UART6)                                                /* ------ we chose port 6 -------*/
  UART6_Init(MavLinkBuf.Baud);                                                  // Set the Baud Rate for UART6 to that of the Liddar
  UART_Set_Active(&UART6_Read, &UART6_Write, &UART6_Data_Ready, &UART6_Tx_Idle);// Initialise the handlers for UART6

  U6IP0_bit = 0u;                                                               // Set UART6 interrupt
  U6IP1_bit = 0u;                                                               // Set interrupt priorities
  U6IP2_bit = 1u;                                                               // Set UART6 interrupt to level 4
  U6EIF_bit = 0u;                                                               // Clear IF
  U6RXIF_bit = 0u;                                                              // Clear RXIF
  U6EIE_bit = 0u;                                                               // Enable UART6 error interupts  (could also use   IEC1BITS.U2EIE =1)
  U6RXIE_bit =0u;                                                               // Enable UART6 RX IF not 3/4 full as below
  UTXISEL0_U6STA_bit = 0u;                                                      // Set UART IF to trigger when buffer is 3/4 full
  UTXISEL1_U6STA_bit = 1u;                                                      // "                                           "
  URXEN_U6STA_bit = 0u;                                                         // Disable UART 6 receiver to prevent craching uppon boot
  MavLinkBuf.Index=1u;                                                          // Set read index to 1
  U6STA = U6STA & ~(3u<<_U6STA_URXISEL_POSITION);                               // URXISEL=0 enable interrupt on 1 or more char set bits 6 and 7 to off
  U6STA =  (U6STA | ((1u<<_U6STA_UTXEN_POSITION) | (1u<<_U6STA_URXEN_POSITION))); // RXEN=1 TXEN=1 configure TX & RX pins as output & input pins
  U6RXIE_bit =1u;                                                               // Enable UART6 RX IF
#endif /* choose port */
#endif /* we have mavlink on serial */
}

#ifdef USE_CAN
/*-----------------------------------------------------------------------------
 *      Init_Can():  Initialize CAN open interface
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Init_Can()
{
  Can2_Init_Flags = 0u;                                                         //
  //Can2_Send_Flags = 0;                                                        // clear flags
  Can2_Rcv_Flags  = 0u;                                                         //

  CAN2Initialize(SJW, BRP, PHSEG1, PHSEG2, PROPSEG, CAN_CONFIG_FLAGS);

  CAN2SetOperationMode(_CAN_MODE_CONFIG,0xFFu);                                 // set CONFIGURATION mode

  CAN2AssignBuffer(FIFObuffers);                                                //assign the buffers
  //configure rx fifo
  CAN2ConfigureFIFO(0u, 8u,_CAN_FIFO_RX & _CAN_FULL_MESSAGE);                   //RX buffer 8 messages deep
  //configure tx fifo
  CAN2ConfigureFIFO(1u, 8u,_CAN_FIFO_TX & _CAN_TX_PRIORITY_3 & _CAN_TX_NO_RTR_FRAME);// TX buffer 8 messages deep

  //set filter 1
  CAN2SetFilter(_CAN_FILTER_31, Can2_MSG_ID, _CAN_MASK_0, _CAN_BUFFER_0, _CAN_CONFIG_XTD_MSG);// set id of filter1 to 2nd node ID
  // set NORMAL mode
  CAN2SetOperationMode(_CAN_MODE_NORMAL,0xFFu);
}
#endif
/*-----------------------------------------------------------------------------
 *      initRTOfromNTP():  Initialise the real time clock and raw internet object notation
 *                         structure from a radio clock or ntp Server boot up
 *
 *  Parameters: (none)
 *  Return:     (none) updates global RION object
 *----------------------------------------------------------------------------*/
void initRTOfromNTP( )
{
   int16_t secondsLow;
   int16_t secondsHigh;
   int16_t minutesLow;
   int16_t minutesHigh;
   int16_t hoursLow;
   int16_t hoursHigh;

   g_timeDate.field_type= RION_UTCDateTime;                                     /* set the rion object for reading to other slaves */
   /* get the time from the ntp Server and set the RTC and RION object here  */
   secondsLow = (RTCTIME & 0xF00ul)>>8u;                                        /* grab the low seconds BYTE */
   secondsHigh = (RTCTIME & 0x7000ul)>>12u;                                     /* fetch teh high seconds BYTE  */
   g_timeDate.sec = (secondsHigh * 10u) + secondsLow;                           /* combine the bytes and update the seonds in the raw internet object */
   minutesLow = (RTCTIME & 0xF0000ul)>>16u;
   minutesHigh = (RTCTIME & 0x700000ul)>>20u;
   g_timeDate.min = (minutesHigh * 10u) + minutesLow;                           /* update minutes */
   hoursLow = (RTCTIME & 0xF000000ul)>>24u;
   hoursHigh = (RTCTIME & 0x30000000ul)>>28u;
   g_timeDate.hour = (hoursHigh * 10u) + hoursLow;                              /* update hour */
   g_timeDate.millis = g_timer5_counter * 100ULL;                               /* update miliseconds from global interupt */
}

/*-----------------------------------------------------------------------------
 *      updateRTOfromRTC():  Updates the global real time object from the real 
 *                           time clock
 *
 *  Parameters: (none)
 *  Return:     (none) updates global RION object
 *----------------------------------------------------------------------------*/
void updateRTOfromRTC( )
{
   int16_t secondsLow;
   int16_t secondsHigh;
   int16_t minutesLow;
   int16_t minutesHigh;
   int16_t hoursLow;
   int16_t hoursHigh;

   secondsLow = (RTCTIME & 0xF00ul)>>8u;                                        /* grab the low seconds BYTE */
   secondsHigh = (RTCTIME & 0x7000ul)>>12u;                                     /* fetch teh high seconds BYTE  */
   g_timeDate.sec = (secondsHigh * 10u) + secondsLow;                           /* combine the bytes and update the seonds in the raw internet object */
   minutesLow = (RTCTIME & 0xF0000ul)>>16u;
   minutesHigh = (RTCTIME & 0x700000ul)>>20u;
   g_timeDate.min = (minutesHigh * 10u) + minutesLow;                           /* update minutes */
   hoursLow = (RTCTIME & 0xF000000ul)>>24u;
   hoursHigh = (RTCTIME & 0x30000000ul)>>28u;
   g_timeDate.hour = (hoursHigh * 10u) + hoursLow;                              /* update hour */
   g_timeDate.millis = g_timer5_counter * 100ULL;                               /* update miliseconds */
}

/*-----------------------------------------------------------------------------
 *      Status:  Print debug Status messages
 *
 *  Parameters: int16_t address, int16_t state
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void Status(int16_t address, int16_t state)
 {

#ifndef UART5_INTERUPT
#ifdef PRINT_DEBUG_MESSAGES                                                     // Print messages to UART5 when enabled

   char txt[7u];
   char str1[100u]={"No known state"};

   int16_t secondsLow;
   int16_t secondsHigh;
   int16_t secondsActual;
   int16_t minutesLow;
   int16_t minutesHigh;
   int16_t minutesActual;
   int16_t hoursLow;
   int16_t hoursHigh;
   int16_t hoursActual;

   // Grab the RTC to show the timestamp
   secondsLow = (RTCTIME & 0xF00ul)>>8u;
   //secondsLow = RTCTIME;
   secondsHigh = (RTCTIME & 0x7000ul)>>12u;
   secondsActual = (secondsHigh * 10u) + secondsLow;
   minutesLow = (RTCTIME & 0xF0000ul)>>16u;
   minutesHigh = (RTCTIME & 0x700000ul)>>20u;
   minutesActual = (minutesHigh * 10u) + minutesLow;
   hoursLow = (RTCTIME & 0xF000000ul)>>24u;
   hoursHigh = (RTCTIME & 0x30000000ul)>>28u;
   hoursActual = (hoursHigh * 10u) + hoursLow;

   IntToStr(Status_Count++,txt);
   Uart5_write_text(txt);
   Uart5_write_text("-");

   switch(address)
  {
     case SOCKET_STATE :
    {
      switch(state)
      {
      /*      state  0 - connection closed
                     1 - remote SYN segment received, our SYN segment sent, and We wait for ACK (Remote mode)
                     3 - connection established
                     4 - our SYN segment sent, and We wait for SYN response (My mode)
                     5 - FIN segment sent, wait for ACK.
                     6 - Received ACK on our FIN, wait for remote FIN.
                     7 - Expired ACK wait time. We retransmit last sent packet, and again set Wait-Timer. If this happen again
                         connection close.                                    */
       case 0u :
       {
        sprintf(str1,"ETH_SOCKET_STATE: - connection closed %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        // Uart5_write_text("ETH_SOCKET_STATE: - connection closed %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        break;
        }
       case 1u :
       {
        sprintf(str1,"ETH_SOCKET_STATE: - remote SYN segment received, our SYN segment sent, and We wait for ACK (Remote mode) %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        // Uart5_write_text("ETH_SOCKET_STATE: - remote SYN segment received, our SYN segment sent, and We wait for ACK (Remote mode) %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        break;
        }
       case 3u :
       {
        sprintf(str1,"ETH_SOCKET_STATE: - connection established %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        // Uart5_write_text("ETH_SOCKET_STATE: - connection established %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        break;
        }
       case 4u :
       {
        sprintf(str1,"ETH_SOCKET_STATE: - our SYN segment sent, and We wait for SYN response (My mode) %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        //Uart5_write_text("ETH_SOCKET_STATE: - our SYN segment sent, and We wait for SYN response (My mode) %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        break;
        }
       case 5u :
       {
        sprintf(str1,"ETH_SOCKET_STATE: - FIN segment sent, wait for ACK %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        //Uart5_write_text("ETH_SOCKET_STATE: - FIN segment sent, wait for ACK %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        break;
        }
       case 6u :
       {
        sprintf(str1,"ETH_SOCKET_STATE: - Received ACK on our FIN, wait for remote FIN %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
       // Uart5_write_text("ETH_SOCKET_STATE: - Received ACK on our FIN, wait for remote FIN %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        break;
        }
       case 7u :
       {
         sprintf(str1,"ETH_SOCKET_STATE: - Expired ACK wait time. We retransmit last sent packet, and again set Wait-Timer. If this happen again connection close%d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        //Uart5_write_text("ETH_SOCKET_STATE: - Expired ACK wait time. We retransmit last sent packet, and again set Wait-Timer. If this happen again connection close %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        break;
        }
        break; //switch(state)
       }
       break;//case SOCKET_STATE
     }
     
     case Eth_Progress :
     {
      /*  Net_Ethernet_Intern_doPacket() Return
          0 - upon successful packet processing (zero packets received or received packet processed successfully).
          1 - upon reception Status or receive buffer corruption. internal Ethernet controller needs to be restarted.
          2 - received packet was not sent to us (not our IP, nor IP broadcast address).
          3 - received IP packet was not IPv4.
          4 - received packet was of type unknown to the library.*       */
       switch(state)
       {
        case 0u :
        {
         sprintf(str1,"ETH_PROGRESS_STATE: - upon successful packet processing (zero packets received or received packet processed successfully)%d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        //Uart5_write_text("ETH_PROGRESS_STATE: - upon successful packet processing (zero packets received or received packet processed successfully)\n\r");
        break;
        }
        case 1u :
        {
        sprintf(str1,"ETH_PROGRESS_STATE: - upon reception Status or receive buffer corruption. internal Ethernet controller needs to be restarted)%d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        //Uart5_write_text("ETH_PROGRESS_STATE: - upon reception Status or receive buffer corruption. internal Ethernet controller needs to be restarted)\n\r");
        break;
        }
        case 2u :
        {
        sprintf(str1,"ETH_PROGRESS_STATE: - received packet was not sent to us (not our IP, nor IP broadcast address)%d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
        //Uart5_write_text("ETH_PROGRESS_STATE: - received packet was not sent to us (not our IP, nor IP broadcast address)\n\r");
        break;
        }
        case 3u :
        {
        sprintf(str1,"ETH_PROGRESS_STATE: - received IP packet was not IPv4 %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);

        //Uart5_write_text("ETH_PROGRESS_STATE: - received IP packet was not IPv4\n\r");
        break;
        }
        case 4u :
        {
         sprintf(str1,"ETH_PROGRESS_STATE: - received packet was of type unknown to the stack %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
       // Uart5_write_text("ETH_PROGRESS_STATE: - received packet was of type unknown to the stack\n\r");
        break;
        }

       }//switch(state) case Eth_Progress
       break;
      }//case Eth_Progress
      
      case(ETHER_LINK_STATE):
      {
         switch(State)
         {
           case TRUE :
            {
               sprintf(str1,"ETH_Present_STATE: - TRUE %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("ETH_Present_STATE: - TRUE\n\r");
               break;
            }
           case FALSE :
           {
               sprintf(str1,"ETH_Present_STATE: - FALSE %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("ETH_Present_STATE: - FALSE\n\r");
               break;
            }
            break;
            
           case AIR_LINK_UP  :
           {
               sprintf(str1,"ETH_Present_STATE: - AIR_LINK_UP %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("ETH_Present_STATE: - AIR_LINK_UP\n\r");
               break;
           }
           case AIR_LINK_DOWN  :
           {
               sprintf(str1,"ETH_Present_STATE: - AIR_LINK_DOWN %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("ETH_Present_STATE: - AIR_LINK_DOWN\n\r");
               break;
           }
           case MASTER_LINK_UP  :
           {
               sprintf(str1,"ETH_Present_STATE: - MASTER_LINK_UP %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
            // Uart5_write_text("ETH_Present_STATE: - MASTER_LINK_UP\n\r");
               break;
           }
           case MASTER_LINK_DOWN  :
           {
               sprintf(str1,"ETH_Present_STATE: - MASTER_LINK_DOWN %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
            // Uart5_write_text("ETH_Present_STATE: - MASTER_LINK_DOWN\n\r");
               break;
           }
           case FIRE_LINK_UP  :
           {
               sprintf(str1,"ETH_Present_STATE: - FIRE_LINK_UP %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("ETH_Present_STATE: - FIRE_LINK_UP\n\r");
               break;
           }
           case FIRE_LINK_DOWN  :
           {
               sprintf(str1,"ETH_Present_STATE: - FIRE_LINK_DOWN %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("ETH_Present_STATE: - FIRE_LINK_DOWN\n\r");
               break;
           }
           case LINK_DOWN  :
           {
               sprintf(str1,"ETH_Present_STATE: - LINK_DOWN_PACKET_NOT_SENT %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("ETH_Present_STATE: - FIRE_LINK_DOWN\n\r");
               break;
           }

           break;
         }// case ETHER_LINK_STATE

      
        break;
      }//case (ETHER_LINK_STATE):
      
      case(U2STA_Add):
      {
         switch(State)
         {
           case U2STA_B2TO :
           {
               sprintf(str1,"UART2_STATE: - Buffer underflow %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("UART2_STATE: - Buffer underflow\n\r");
               break;//case U2STA_B2TO
           }
           case UART2_BUFFER_OVERFLOW :
           {
               sprintf(str1,"UDP_STATE: -UART2 Buffer Overrun %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
            // Uart5_write_text("UDP_STATE: -UART2 Buffer Overrun\n\r");
               UART2.State = CLEAR;
               break;//case UART2_SEND_UDP_PACKET_FAIL
            }
            case UART2_BAD_CRC :
            {
               sprintf(str1,"UDP_STATE: -UART2 BAD CRC %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
            // Uart5_write_text("UDP_STATE: -UART2 Buffer Overrun\n\r");
               UART2.State = CLEAR;
               break;//case UART2_SEND_UDP_PACKET_FAIL
            }
          break;//switch(State)
         }

      break;//case (U2STA_Add)
      }
      case(UDPSTA_Add):
      {
         switch(State)
         {
           case UDP_PACKET_SEND_FAIL :
           {
               sprintf(str1,"UDP_STATE: -Packet Send Failed :%d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
               bad_ether++ % UINT8_MAX;

             //Uart5_write_text("UDP_STATE: -Paket Send Failed\n\r");
               break;//case UDP_PACKET_SEND_FAIL
           }
           case UDP_PACKET_SEND_GOOD :
           {
               sprintf(str1,"UDP_STATE: -Packet Sent %d : %d : %d\n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("UDP_STATE: -Paket Sent\n\r");
               break;//case UDP_PACKET_SEND_GOOD
           }

           case UDP_UART2_PACKET_ACK_TIMEOUT  :
           {
              sprintf(str1,"UDP_STATE: -Packet UART2 ACK TIMEOUT %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("UDP_STATE: -Paket UART2 ACK TIMEOUT\n\r");
              break;//case UDP_PACKER_ACK_TIMEOUT
           }
           case UDP_UART2_SEND_PACKET_FAIL  :
           {
              sprintf(str1,"UDP_STATE: -Packet UART2 NO ACK RECEIVED %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("UDP_STATE: -Paket UART2 NO ACK RECEIVED\n\r");
              break;//case UART2_SEND_UDP_PACKET_FAIL
           }
           case UDP_PACKET_ACK_RECEIVED  :
           {
              sprintf(str1,"UDP_STATE: -Packet UART2 GOOD ACK RECEIVED %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("UDP_STATE: -Paket UART2 GOOD ACK RECEIVED\n\r");
              break;//case UART2_SEND_UDP_PACKET_FAIL
           }
           case UDP_UART2_Packet_RECEIVED  :
           {
              sprintf(str1,"UDP_STATE: -UDP_UART2_Packet_RECEIVED %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("UDP_STATE: -Paket UART2 GOOD ACK RECEIVED\n\r");
              break;//case UART2_SEND_UDP_PACKET_FAIL
           }
           case UDP_UART2_Packet_BAD_CRC  :
           {
              sprintf(str1,"UDP_STATE: -UDP_UART2_Packet_BAD_CRC %d : %d : %d \n\r",hoursActual,minutesActual,secondsActual);
             //Uart5_write_text("UDP_STATE: -Paket UART2 GOOD ACK RECEIVED\n\r");
              break;//case UART2_SEND_UDP_PACKET_FAIL
           }
           break;//switch(State)
         }

        break;
      }//case(UDPSTA_Add)
   }//switch(address)
   Uart5_write_text(&str1);
#endif
#endif
 }
/*-----------------------------------------------------------------------------
 *      CRC16_Calc:  CRC16 calc
 *
 *  Parameters: unsigned char *Packet,unsigned char Packet_length
 *  Return:     uint16_t = CRC16
 *----------------------------------------------------------------------------*/
uint16_t CRC16_Calc( const unsigned char *Packet, uint16_t Packet_length)
{
  uint16_t dat=0u, cnt=0u, nibblemax=0u, reg=0u, look=0u, pos=0u, crc=0u;
  const uint16_t CRC16_TBL[16u] = {0x0000u,0xDA44u,0x3081u,0xEAC5u,0x6102u,
  0xBB46u,0x5183u,0x8BC7u,0xC204u,0x1840u,0xF285u,0x28C1u,0xA306u, 0x7942u,
  0x9387u,0x49C3u};
  
  if (Packet == NULL)
    return 0u;
    
  /**************************************************************
  * Initialize
  **************************************************************/
  nibblemax = Packet_length << 1u;                                              /* 2 nibbles per byte */
  /**************************************************************
  * Run over whole byte vector
  **************************************************************/
  for( cnt = 0u; cnt < nibblemax; cnt++ )
  {
     pos = cnt >> 1u;                                                           /* byte counter */
     dat = Packet[pos];
     pos = ( cnt & 0x0001u ) << 2u;                                             /* lower/upper nibble */
     dat >>= pos;                                                               /* shift by 4 or by 0 */
     dat &= 0x000Fu;
  /***********************************************************
  * xor register with LSB input data nibble
  * lookup intermediate value
  * shift register one nibble right
  * xor register with intermediate value
  ***********************************************************/
     reg ^= dat;
     pos = reg & 0x000Fu;
     look = CRC16_TBL[pos];
     reg >>= 4u;
     reg ^= look;
  }
  /* end for( cnt < nibblemax ) */
  crc = reg;
  return (crc);
}

/*-----------------------------------------------------------------------------
 *      Init_Node():  Initialise the Node with ip address and remote (send) ip address
 *                    This is the physical definition ofwho talks to who and where
 *                    each functionality resides
 *
 *  Parameters: none
 *  Return:     none
 *----------------------------------------------------------------------------*/
void Init_Node()
 {
#if (THIS_NODE_IS==GIMJOY)                                                      /* ========= Joystick (Gimbal Motion) Controller ========== */
     This_Node.Is=GIMJOY; 
     This_Node.IpAddr[0u]= GimJoy_IpAddr[0u]; 
     This_Node.IpAddr[1u]= GimJoy_IpAddr[1u];
     This_Node.IpAddr[2u]= GimJoy_IpAddr[2u];
     This_Node.IpAddr[3u]= GimJoy_IpAddr[3u];

#ifdef ETHER_USE_DEFINED_MAC                                                    // Otherwise use the physical MAC Address of the Card
     This_Node.MacAddr[0u]=0x54U;                                               // What was hardcoded in the definition file
     This_Node.MacAddr[1u]=0x10U;
     This_Node.MacAddr[2u]=0xecU;
     This_Node.MacAddr[3u]=0xdcU;
     This_Node.MacAddr[4u]=0xe1U;
     This_Node.MacAddr[5u]=0x70U;
#else
     This_Node.MacAddr[0u]=(EMAC1SA2 & 0xFFu);                                  // Read what is physically there and use that
     This_Node.MacAddr[1u]=((EMAC1SA2>>8u) & 0xFFu);
     This_Node.MacAddr[2u]=(EMAC1SA1 & 0xFFu);
     This_Node.MacAddr[3u]=((EMAC1SA1>>8u) & 0xFFu);
     This_Node.MacAddr[4u]=(EMAC1SA0 & 0xFFu);
     This_Node.MacAddr[5u]=((EMAC1SA0>>8u) & 0xFFu);
#endif

     // Set-Up Remote IP (of the corresponding serial decoder) for each protocol
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
     SBGC_DATA.RemoteIpAddr[0u] = Air_IpAddr[0u]; SBGC_DATA.RemoteIpAddr[1u] = Air_IpAddr[1u]; SBGC_DATA.RemoteIpAddr[2u] =Air_IpAddr[2u];SBGC_DATA.RemoteIpAddr[3u] = Air_IpAddr[3u];    // Air COM PIC
#endif
#if (CAMERA_TYPE == XS_CAM)
     XS_DATA.RemoteIpAddr[0u] = Air_IpAddr[0u]; XS_DATA.RemoteIpAddr[1u] = Air_IpAddr[1u]; XS_DATA.RemoteIpAddr[2u] =Air_IpAddr[2u];XS_DATA.RemoteIpAddr[3u] = Air_IpAddr[3u];
#endif
#if defined(RUN_CAM_USED)
     RunC_DATA.RemoteIpAddr[0u] = Air_IpAddr[0u]; RunC_DATA.RemoteIpAddr[1u] = Air_IpAddr[1u]; RunC_DATA.RemoteIpAddr[2u] =Air_IpAddr[2u];RunC_DATA.RemoteIpAddr[3u] = Air_IpAddr[3u];  // Air COM PIC
#endif
#if (defined(USE_MAVLINK) && !defined(SERIAL_MAVLINK))
     MavLinkBuf.RemoteIpAddr[0u] = Air_IpAddr[0u]; MavLinkBuf.RemoteIpAddr[1u] = Air_IpAddr[1u]; MavLinkBuf.RemoteIpAddr[2u] =Air_IpAddr[2u];MavLinkBuf.RemoteIpAddr[3u] = Air_IpAddr[3u];  // Air COM PIC
     mavInitUDP();                                                              /* set the port definitions */
#elif defined(SERIAL_MAVLINK)
     MavLinkBuf.Baud = MAV_BAUD_RATE;                                           /* set baud rate */
#endif
#if defined(SEQ_CAM_USED)
     SeqCam_DATA.RemoteIpAddr[0u] = ParrotSeq_IpAddr[0u]; SeqCam_DATA.RemoteIpAddr[1u] = ParrotSeq_IpAddr[1u]; SeqCam_DATA.RemoteIpAddr[2u] =ParrotSeq_IpAddr[2u];SeqCam_DATA.RemoteIpAddr[3u] = ParrotSeq_IpAddr[3u];  // Parrot SEquioa camera
#endif
#if defined(YI_CAM_USED)
     YiCam_DATA.RemoteIpAddr[0u] = YiCam_IpAddr[0u]; YiCam_DATA.RemoteIpAddr[1u] = YiCam_IpAddr[1u]; YiCam_DATA.RemoteIpAddr[2u] =YiCam_IpAddr[2u];YiCam_DATA.RemoteIpAddr[3u] = YiCam_IpAddr[3u];  // Yi Action camera
#endif
#if defined(MICROSCAN_OUT_TCP)
     MiCroScaN_DATA.RemoteIpAddr[0u] = microScan_IpAddr[0u]; MiCroScaN_DATA.RemoteIpAddr[1u] = microScan_IpAddr[1u]; MiCroScaN_DATA.RemoteIpAddr[2u] =microScan_IpAddr[2u];MiCroScaN_DATA.RemoteIpAddr[3u] = microScan_IpAddr[3u];  // microscan ANPR camera
#endif
#if defined(LW3_SWITCH_USED)
     LW3_DATA.RemoteIpAddr[0u] = vidAudSwitch_IpAddr[0u]; LW3_DATA.RemoteIpAddr[1u] = vidAudSwitch_IpAddr[1u]; LW3_DATA.RemoteIpAddr[2u] =vidAudSwitch_IpAddr[2u];LW3_DATA.RemoteIpAddr[3u] = vidAudSwitch_IpAddr[3u];  // lightware video and audio switch
#endif
#if defined(GEF_EGD_PLC)
     EGD_DATA.RemoteIpAddr[0u] = gefPLC_IpAddr[0u]; EGD_DATA.RemoteIpAddr[1u] = gefPLC_IpAddr[1u]; EGD_DATA.RemoteIpAddr[2u] =gefPLC_IpAddr[2u];EGD_DATA.RemoteIpAddr[3u] = gefPLC_IpAddr[3u];  // GE Fanuc PLC
#endif
#if defined(ART_NET_USED)
     artNet_DATA.RemoteIpAddr[0u] = artNet_IpAddr[0u]; artNet_DATA.RemoteIpAddr[1u] = artNet_IpAddr[1u]; artNet_DATA.RemoteIpAddr[2u] =artNet_IpAddr[2u];artNet_DATA.RemoteIpAddr[3u] = artNet_IpAddr[3u];  // ArtNet-4 Node
#endif
#if defined(CBOR_COMS_USED)
     CBOR_DATA.RemoteIpAddr[0u] = cborM2M_IpAddr[0u]; CBOR_DATA.RemoteIpAddr[1u] = cborM2M_IpAddr[1u]; CBOR_DATA.RemoteIpAddr[2u] =cborM2M_IpAddr[2u];CBOR_DATA.RemoteIpAddr[3u] = cborM2M_IpAddr[3u];  // m2m using iOt cbor objects
#endif
#if defined(MODBUS_TCP)
     Modbus_DATA.RemoteIpAddr[0u] = modTCP1_IpAddr[0u]; Modbus_DATA.RemoteIpAddr[1u] = modTCP1_IpAddr[1u]; Modbus_DATA.RemoteIpAddr[2u] =modTCP1_IpAddr[2u];Modbus_DATA.RemoteIpAddr[3u] = modTCP1_IpAddr[3u];  // m2m using iOt cbor objects
#endif
#if defined(SMPTE_USED)
     SMPTE_DATA.RemoteIpAddr[0u] = smpte_IpAddr[0u]; SMPTE_DATA.RemoteIpAddr[1u] = smpte_IpAddr[1u]; SMPTE_DATA.RemoteIpAddr[2u] =smpte_IpAddr[2u];SMPTE_DATA.RemoteIpAddr[3u] = smpte_IpAddr[3u];  // smpte video timestamper
#endif
#if defined(ASN_E1_USED)
     ASN_E1_DATA.RemoteIpAddr[0u] = asnE1_IpAddr[0u]; ASN_E1_DATA.RemoteIpAddr[1u] = asnE1_IpAddr[1u]; ASN_E1_DATA.RemoteIpAddr[2u] =asnE1_IpAddr[2u];ASN_E1_DATA.RemoteIpAddr[3u] = asnE1_IpAddr[3u];  // smpte video timestamper
#endif
#ifdef UBIBOT_DEVICE_USED
     UbiSerial.Baud=UBIBOT_SERIAL_BAUD;
#endif
#elif (THIS_NODE_IS==COMGS)                                                     /* ========= Command (GUI) Ground Station ========== */
     This_Node.Is=COMGS;
     This_Node.IPAddr[0u]= Com1Gs_IpAddr[0u];
     This_Node.IPAddr[1u]= Com1Gs_IpAddr[1u];
     This_Node.IPAddr[2u]= Com1Gs_IpAddr[2u];
     This_Node.IPAddr[3u]= Com1Gs_IpAddr[3u];
#ifdef ETHER_USE_DEFINED_MAC                                                    // Otherwise use the physical MAC Address of the Card
     This_Node.MacAddr[0u]=Com1Gs_MACaddr[0u];
     This_Node.MacAddr[1u]=Com1Gs_MACaddr[1u];
     This_Node.MacAddr[2u]=Com1Gs_MACaddr[2u];
     This_Node.MacAddr[3u]=Com1Gs_MACaddr[3u];
     This_Node.MacAddr[4u]=Com1Gs_MACaddr[4u];
     This_Node.MacAddr[5u]=Com1Gs_MACaddr[5u];
#else
     This_Node.MacAddr[0u]=(EMAC1SA2 & 0xFFu);                                  // What is physically there read from the PHY chip
     This_Node.MacAddr[1u]=((EMAC1SA2>>8u) & 0xFFu);
     This_Node.MacAddr[2u]=(EMAC1SA1 & 0xFFu);
     This_Node.MacAddr[3u]=((EMAC1SA1>>8u) & 0xFFu);
     This_Node.MacAddr[4u]=(EMAC1SA0 & 0xFFu);
     This_Node.MacAddr[5u]=((EMAC1SA0>>8u) & 0xFFu);
#endif

     // Set-Up Remote IP (of the corresponding serial decoder) for each protocol
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
     SBGC_DATA.RemoteIpAddr[0u] = Air_IpAddr[0u]; SBGC_DATA.RemoteIpAddr[1u] = Air_IpAddr[1u]; SBGC_DATA.RemoteIpAddr[2u] =Air_IpAddr[2u];SBGC_DATA.RemoteIpAddr[3u] = Air_IpAddr[3u];
#endif
#if (CAMERA_TYPE == XS_CAM)
     XS_DATA.RemoteIpAddr[0u] = Air_IpAddr[0u]; XS_DATA.RemoteIpAddr[1u] = Air_IpAddr[1u]; XS_DATA.RemoteIpAddr[2u] =Air_IpAddr[2u];XS_DATA.RemoteIpAddr[3u] = Air_IpAddr[3u];  // Air COM PIC
#endif
#if defined(RUN_CAM_USED)
     RunC_DATA.RemoteIpAddr[0u] = Air_IpAddr[0u]; RunC_DATA.RemoteIpAddr[1u] = Air_IpAddr[1u]; RunC_DATA.RemoteIpAddr[2u] =Air_IpAddr[2u];RunC_DATA.RemoteIpAddr[3u] = Air_IpAddr[3u];  // Air COM PIC
#endif
#if (defined(USE_MAVLINK) && !defined(SERIAL_MAVLINK))
     MavLinkBuf.RemoteIpAddr[0u] = Air_IpAddr[0u]; MavLinkBuf.RemoteIpAddr[1u] = Air_IpAddr[1u]; MavLinkBuf.RemoteIpAddr[2u] =Air_IpAddr[2u];MavLinkBuf.RemoteIpAddr[3u] = Air_IpAddr[3u];  // Air COM PIC
     mavInitUDP();                                                              /* set the port definitions */
#elif defined(SERIAL_MAVLINK)
     MavLinkBuf.Baud = MAV_BAUD_RATE;                                           /* set baud rate */
#endif
#if defined(SEQ_CAM_USED)
     SeqCam_DATA.RemoteIpAddr[0u] = ParrotSeq_IpAddr[0u]; SeqCam_DATA.RemoteIpAddr[1u] = ParrotSeq_IpAddr[1u]; SeqCam_DATA.RemoteIpAddr[2u] =ParrotSeq_IpAddr[2u];SeqCam_DATA.RemoteIpAddr[3u] = ParrotSeq_IpAddr[3u];  // Parrot SEquioa camera
#endif
#if defined(YI_CAM_USED)
     YiCam_DATA.RemoteIpAddr[0u] = YiCam_IpAddr[0u]; YiCam_DATA.RemoteIpAddr[1u] = YiCam_IpAddr[1u]; YiCam_DATA.RemoteIpAddr[2u] =YiCam_IpAddr[2u];YiCam_DATA.RemoteIpAddr[3u] = YiCam_IpAddr[3u];  // Yi Action camera
#endif
#if defined(MICROSCAN_USED)
     MiCroScaN_DATA.RemoteIpAddr[0u] = microScan_IpAddr[0u]; MiCroScaN_DATA.RemoteIpAddr[1u] = microScan_IpAddr[1u]; MiCroScaN_DATA.RemoteIpAddr[2u] =microScan_IpAddr[2u];MiCroScaN_DATA.RemoteIpAddr[3u] = microScan_IpAddr[3u];  // microscan ANPR camera
#endif
#if defined(LW3_SWITCH_USED)
     LW3_DATA.RemoteIpAddr[0u] = vidAudSwitch_IpAddr[0u]; LW3_DATA.RemoteIpAddr[1u] = vidAudSwitch_IpAddr[1u]; LW3_DATA.RemoteIpAddr[2u] =vidAudSwitch_IpAddr[2u];LW3_DATA.RemoteIpAddr[3u] = vidAudSwitch_IpAddr[3u];  // lightware video and audio switch
#endif
#if defined(GEF_EGD_PLC)
     EGD_DATA.RemoteIpAddr[0u] = gefPLC_IpAddr[0u]; EGD_DATA.RemoteIpAddr[1u] = gefPLC_IpAddr[1u]; EGD_DATA.RemoteIpAddr[2u] =gefPLC_IpAddr[2u];EGD_DATA.RemoteIpAddr[3u] = gefPLC_IpAddr[3u];  // GE Fanuc PLC
#endif
#if defined(ART_NET_USED)
     artNet_DATA.RemoteIpAddr[0u] = artNet_IpAddr[0u]; artNet_DATA.RemoteIpAddr[1u] = artNet_IpAddr[1u]; artNet_DATA.RemoteIpAddr[2u] =artNet_IpAddr[2u];artNet_DATA.RemoteIpAddr[3u] = artNet_IpAddr[3u];  // ArtNet-4 Node
#endif
#if defined(CBOR_COMS_USED)
     CBOR_DATA.RemoteIpAddr[0u] = cborM2M_IpAddr[0u]; CBOR_DATA.RemoteIpAddr[1u] = cborM2M_IpAddr[1u]; CBOR_DATA.RemoteIpAddr[2u] =cborM2M_IpAddr[2u];CBOR_DATA.RemoteIpAddr[3u] = cborM2M_IpAddr[3u];  // m2m using iOt cbor objects
#endif
#if defined(MODBUS_TCP)
     Modbus_DATA.RemoteIpAddr[0u] = modTCP1_IpAddr[0u]; Modbus_DATA.RemoteIpAddr[1u] = modTCP1_IpAddr[1u]; Modbus_DATA.RemoteIpAddr[2u] =modTCP1_IpAddr[2u];Modbus_DATA.RemoteIpAddr[3u] = modTCP1_IpAddr[3u];  // m2m using iOt cbor objects
#endif
#ifdef UBIBOT_DEVICE_USED
     UbiSerial.Baud=UBIBOT_SERIAL_BAUD;
#endif
#elif (THIS_NODE_IS==AIR)                                                       /* ========= AirCraft or Remote Robot ========== */
     This_Node.Is=AIR;
     This_Node.IPAddr[0u]= Air_IpAddr[0u];
     This_Node.IPAddr[1u]= Air_IpAddr[1u];
     This_Node.IPAddr[2u]= Air_IpAddr[2u];
     This_Node.IPAddr[3u]= Air_IpAddr[3u];
#ifdef ETHER_USE_DEFINED_MAC                                                    // Otherwise use the physical MAC Address of the Card
     This_Node.MacAddr[0u]=Air_MacAddr[0u];
     This_Node.MacAddr[1u]=Air_MacAddr[1u];
     This_Node.MacAddr[2u]=Air_MacAddr[2u];
     This_Node.MacAddr[3u]=Air_MacAddr[3u];
     This_Node.MacAddr[4u]=Air_MacAddr[4u];
     This_Node.MacAddr[5u]=Air_MacAddr[5u];
#else
     This_Node.MacAddr[0u]=(EMAC1SA2 & 0xFFu);                                  // What is physically there read from the PHY chip
     This_Node.MacAddr[1u]=((EMAC1SA2>>8u) & 0xFFu);
     This_Node.MacAddr[2u]=(EMAC1SA1 & 0xFFu);
     This_Node.MacAddr[3u]=((EMAC1SA1>>8u) & 0xFFu);
     This_Node.MacAddr[4u]=(EMAC1SA0 & 0xFFu);
     This_Node.MacAddr[5u]=((EMAC1SA0>>8u) & 0xFFu);
#endif

     // Set-Up Remote IP (of the corresponding serial decoder) for each protocol
#if defined(SBGC_GIMBAL_JOY)
     SBGC_DATA.RemoteIpAddr[0u] = GimJoy_IpAddr[0u]; SBGC_DATA.RemoteIpAddr[1u] = GimJoy_IpAddr[1u]; SBGC_DATA.RemoteIpAddr[2u] =GimJoy_IpAddr[2u];SBGC_DATA.RemoteIpAddr[3u] = GimJoy_IpAddr[3u];
#elif defined(SBGC_GIMBAL_HMI)
     SBGC_DATA.RemoteIpAddr[0u] = Com1Gs_IpAddr[0u]; SBGC_DATA.RemoteIpAddr[1u] = Com1Gs_IpAddr[1u]; SBGC_DATA.RemoteIpAddr[2u] =Com1Gs_IpAddr[2u];SBGC_DATA.RemoteIpAddr[3u] = Com1Gs_IpAddr[3u];
#endif
#if (CAMERA_TYPE == XS_CAM)
     XS_DATA.RemoteIpAddr[0u] = Com1Gs_IpAddr[0u]; XS_DATA.RemoteIpAddr[1u] = Com1Gs_IpAddr[1u]; XS_DATA.RemoteIpAddr[2u] =Com1Gs_IpAddr[2u];XS_DATA.RemoteIpAddr[3u] = Com1Gs_IpAddr[3u];  // Air COM PIC
#endif
#if defined(RUN_CAM_USED)
     RunC_DATA.RemoteIpAddr[0u] = Com1Gs_IpAddr[0u]; RunC_DATA.RemoteIpAddr[1u] = Com1Gs_IpAddr[1u]; RunC_DATA.RemoteIpAddr[2u] =Com1Gs_IpAddr[2u];RunC_DATA.RemoteIpAddr[3u] = Com1Gs_IpAddr[3u];  // Air COM PIC
#endif
#if (defined(USE_MAVLINK) && !defined(SERIAL_MAVLINK))
     MavLinkBuf.RemoteIpAddr[0u] = Ardupilot_IpAddr[0u]; MavLinkBuf.RemoteIpAddr[1u] = Ardupilot_IpAddr[1u]; MavLinkBuf.RemoteIpAddr[2u] =Ardupilot_IpAddr[2u];MavLinkBuf.RemoteIpAddr[3u] = Ardupilot_IpAddr[3u];  // Air COM PIC
     mavInitUDP();                                                              /* set the port definitions */
#elif defined(SERIAL_MAVLINK)
     MavLinkBuf.Baud = MAV_BAUD_RATE;                                           /* set baud rate */
#endif
#if defined(SEQ_CAM_USED)
     SeqCam_DATA.RemoteIpAddr[0u] = ParrotSeq_IpAddr[0u]; SeqCam_DATA.RemoteIpAddr[1u] = ParrotSeq_IpAddr[1u]; SeqCam_DATA.RemoteIpAddr[2u] =ParrotSeq_IpAddr[2u];SeqCam_DATA.RemoteIpAddr[3u] = ParrotSeq_IpAddr[3u];  // Parrot SEquioa camera
#endif
#if defined(YI_CAM_USED)
     YiCam_DATA.RemoteIpAddr[0u] = YiCam_IpAddr[0u]; YiCam_DATA.RemoteIpAddr[1u] = YiCam_IpAddr[1u]; YiCam_DATA.RemoteIpAddr[2u] =YiCam_IpAddr[2u];YiCam_DATA.RemoteIpAddr[3u] = YiCam_IpAddr[3u];  // Yi Action camera
#endif
#if defined(MICROSCAN_USED)
     MiCroScaN_DATA.RemoteIpAddr[0u] = microScan_IpAddr[0u]; MiCroScaN_DATA.RemoteIpAddr[1u] = microScan_IpAddr[1u]; MiCroScaN_DATA.RemoteIpAddr[2u] =microScan_IpAddr[2u];MiCroScaN_DATA.RemoteIpAddr[3u] = microScan_IpAddr[3u];  // microscan ANPR camera
#endif
#if defined(LW3_SWITCH_USED)
     LW3_DATA.RemoteIpAddr[0u] = vidAudSwitch_IpAddr[0u]; LW3_DATA.RemoteIpAddr[1u] = vidAudSwitch_IpAddr[1u]; LW3_DATA.RemoteIpAddr[2u] =vidAudSwitch_IpAddr[2u];LW3_DATA.RemoteIpAddr[3u] = vidAudSwitch_IpAddr[3u];  // lightware video and audio switch
#endif
#if defined(GEF_EGD_PLC)
     EGD_DATA.RemoteIpAddr[0u] = gefPLC_IpAddr[0u]; EGD_DATA.RemoteIpAddr[1u] = gefPLC_IpAddr[1u]; EGD_DATA.RemoteIpAddr[2u] =gefPLC_IpAddr[2u];EGD_DATA.RemoteIpAddr[3u] = gefPLC_IpAddr[3u];  // GE Fanuc PLC
#endif
#if defined(ART_NET_USED)
     artNet_DATA.RemoteIpAddr[0u] = artNet_IpAddr[0u]; artNet_DATA.RemoteIpAddr[1u] = artNet_IpAddr[1u]; artNet_DATA.RemoteIpAddr[2u] =artNet_IpAddr[2u];artNet_DATA.RemoteIpAddr[3u] = artNet_IpAddr[3u];  // ArtNet-4 Node
#endif
#if defined(CBOR_COMS_USED)
     CBOR_DATA.RemoteIpAddr[0u] = cborM2M_IpAddr[0u]; CBOR_DATA.RemoteIpAddr[1u] = cborM2M_IpAddr[1u]; CBOR_DATA.RemoteIpAddr[2u] =cborM2M_IpAddr[2u];CBOR_DATA.RemoteIpAddr[3u] = cborM2M_IpAddr[3u];  // m2m using iOt cbor objects
#endif
#if defined(MODBUS_TCP)
     Modbus_DATA.RemoteIpAddr[0u] = modTCP1_IpAddr[0u]; Modbus_DATA.RemoteIpAddr[1u] = modTCP1_IpAddr[1u]; Modbus_DATA.RemoteIpAddr[2u] =modTCP1_IpAddr[2u];Modbus_DATA.RemoteIpAddr[3u] = modTCP1_IpAddr[3u];  // m2m using iOt cbor objects
#endif
#ifdef UBIBOT_DEVICE_USED
   UbiSerial.Baud=UBIBOT_SERIAL_BAUD;
#endif
#elif (THIS_NODE_IS==FIRGS)                                                     /* ========= Fire Ground Station ========== */
     This_Node.Is=FIRGS;
     This_Node.IPAddr[0u]= FirGs_IpAddr[0u];
     This_Node.IPAddr[1u]= FirGs_IpAddr[1u];
     This_Node.IPAddr[2u]= FirGs_IpAddr[2u];
     This_Node.IPAddr[3u]= FirGs_IpAddr[3u];
#ifdef ETHER_USE_DEFINED_MAC                                                    // Otherwise use the physical MAC Address of the Card
     This_Node.MacAddr[0u]=FirGs_MacAddr[0u];
     This_Node.MacAddr[1u]=FirGs_MacAddr[1u];
     This_Node.MacAddr[2u]=FirGs_MacAddr[2u];
     This_Node.MacAddr[3u]=FirGs_MacAddr[3u];
     This_Node.MacAddr[4u]=FirGs_MacAddr[4u];
     This_Node.MacAddr[5u]=FirGs_MacAddr[5u];
#else
     This_Node.MacAddr[0u]=(EMAC1SA2 & 0xFFu);                                  // What is physically there read from the PHY chip
     This_Node.MacAddr[1u]=((EMAC1SA2>>8u) & 0xFFu);
     This_Node.MacAddr[2u]=(EMAC1SA1 & 0xFFu);
     This_Node.MacAddr[3u]=((EMAC1SA1>>8u) & 0xFFu);
     This_Node.MacAddr[4u]=(EMAC1SA0 & 0xFFu);
     This_Node.MacAddr[5u]=((EMAC1SA0>>8u) & 0xFFu);
#endif

     // Set-Up Remote IP (of the corresponding serial decoder) for each protocol
#if defined(SBGC_GIMBAL_JOY)
     SBGC_DATA.RemoteIpAddr[0u] = GimJoy_IpAddr[0u]; SBGC_DATA.RemoteIpAddr[1u] = GimJoy_IpAddr[1u]; SBGC_DATA.RemoteIpAddr[2u] =GimJoy_IpAddr[2u];SBGC_DATA.RemoteIpAddr[3u] = GimJoy_IpAddr[3u];
#elif defined(SBGC_GIMBAL_HMI)
     SBGC_DATA.RemoteIpAddr[0u] = Com1Gs_IpAddr[0u]; SBGC_DATA.RemoteIpAddr[1u] = Com1Gs_IpAddr[1u]; SBGC_DATA.RemoteIpAddr[2u] =Com1Gs_IpAddr[2u];SBGC_DATA.RemoteIpAddr[3u] = Com1Gs_IpAddr[3u];
#endif
#if (CAMERA_TYPE == XS_CAM)
     XS_DATA.RemoteIpAddr[0u] = Com1Gs_IpAddr[0u]; XS_DATA.RemoteIpAddr[1u] = Com1Gs_IpAddr[1u]; XS_DATA.RemoteIpAddr[2u] =Com1Gs_IpAddr[2u];XS_DATA.RemoteIpAddr[3u] = Com1Gs_IpAddr[3u];  // Air COM PIC
#endif
#if defined(RUN_CAM_USED)
     RunC_DATA.RemoteIpAddr[0u] = Com1Gs_IpAddr[0u]; RunC_DATA.RemoteIpAddr[1u] = Com1Gs_IpAddr[1u]; RunC_DATA.RemoteIpAddr[2u] =Com1Gs_IpAddr[2u];RunC_DATA.RemoteIpAddr[3u] = Com1Gs_IpAddr[3u];  // Air COM PIC
#endif
#if (defined(USE_MAVLINK) && !defined(SERIAL_MAVLINK))
     MavLinkBuf.RemoteIpAddr[0u] = Ardupilot_IpAddr[0u]; MavLinkBuf.RemoteIpAddr[1u] = Ardupilot_IpAddr[1u]; MavLinkBuf.RemoteIpAddr[2u] =Ardupilot_IpAddr[2u];MavLinkBuf.RemoteIpAddr[3u] = Ardupilot_IpAddr[3u];  // Air COM PIC
     mavInitUDP();                                                              /* set the port definitions */
#elif defined(SERIAL_MAVLINK)
     MavLinkBuf.Baud = MAV_BAUD_RATE;                                           /* set baud rate */
#endif
#if defined(SEQ_CAM_USED)
     SeqCam_DATA.RemoteIpAddr[0u] = ParrotSeq_IpAddr[0u]; SeqCam_DATA.RemoteIpAddr[1u] = ParrotSeq_IpAddr[1u]; SeqCam_DATA.RemoteIpAddr[2u] =ParrotSeq_IpAddr[2u];SeqCam_DATA.RemoteIpAddr[3u] = ParrotSeq_IpAddr[3u];  // Parrot SEquioa camera
#endif
#if defined(YI_CAM_USED)
     YiCam_DATA.RemoteIpAddr[0u] = YiCam_IpAddr[0u]; YiCam_DATA.RemoteIpAddr[1u] = YiCam_IpAddr[1u]; YiCam_DATA.RemoteIpAddr[2u] =YiCam_IpAddr[2u];YiCam_DATA.RemoteIpAddr[3u] = YiCam_IpAddr[3u];  // Yi Action camera
#endif
#if defined(MICROSCAN_USED)
     MiCroScaN_DATA.RemoteIpAddr[0u] = microScan_IpAddr[0u]; MiCroScaN_DATA.RemoteIpAddr[1u] = microScan_IpAddr[1u]; MiCroScaN_DATA.RemoteIpAddr[2u] =microScan_IpAddr[2u];MiCroScaN_DATA.RemoteIpAddr[3u] = microScan_IpAddr[3u];  // microscan ANPR camera
#endif
#if defined(LW3_SWITCH_USED)
     LW3_DATA.RemoteIpAddr[0u] = vidAudSwitch_IpAddr[0u]; LW3_DATA.RemoteIpAddr[1u] = vidAudSwitch_IpAddr[1u]; LW3_DATA.RemoteIpAddr[2u] =vidAudSwitch_IpAddr[2u];LW3_DATA.RemoteIpAddr[3u] = vidAudSwitch_IpAddr[3u];  // lightware video and audio switch
#endif
#if defined(GEF_EGD_PLC)
     EGD_DATA.RemoteIpAddr[0u] = gefPLC_IpAddr[0u]; EGD_DATA.RemoteIpAddr[1u] = gefPLC_IpAddr[1u]; EGD_DATA.RemoteIpAddr[2u] =gefPLC_IpAddr[2u];EGD_DATA.RemoteIpAddr[3u] = gefPLC_IpAddr[3u];  // GE Fanuc PLC
#endif
#if defined(ART_NET_USED)
     artNet_DATA.RemoteIpAddr[0u] = artNet_IpAddr[0u]; artNet_DATA.RemoteIpAddr[1u] = artNet_IpAddr[1u]; artNet_DATA.RemoteIpAddr[2u] =artNet_IpAddr[2u];artNet_DATA.RemoteIpAddr[3u] = artNet_IpAddr[3u];  // ArtNet-4 Node
#endif
#if defined(CBOR_COMS_USED)
     CBOR_DATA.RemoteIpAddr[0u] = cborM2M_IpAddr[0u]; CBOR_DATA.RemoteIpAddr[1u] = cborM2M_IpAddr[1u]; CBOR_DATA.RemoteIpAddr[2u] =cborM2M_IpAddr[2u];CBOR_DATA.RemoteIpAddr[3u] = cborM2M_IpAddr[3u];  // m2m using iOt cbor objects
#endif
#if defined(MODBUS_TCP)
     Modbus_DATA.RemoteIpAddr[0u] = modTCP1_IpAddr[0u]; Modbus_DATA.RemoteIpAddr[1u] = modTCP1_IpAddr[1u]; Modbus_DATA.RemoteIpAddr[2u] =modTCP1_IpAddr[2u];Modbus_DATA.RemoteIpAddr[3u] = modTCP1_IpAddr[3u];  // m2m using iOt cbor objects
#endif
#if defined(UBIBOT_DEVICE_USED)
     UbiSerial.Baud=UBIBOT_SERIAL_BAUD;
#endif
#endif /* end of choose Node on defines */

 }
#if (!defined(SERIAL_MAVLINK) && defined(USE_MAVLINK))
/*-----------------------------------------------------------------------------
 *      chooseMavlinkSink:  dynamically assign mavlink sink ip address
 *  defines the current mavlink sink for receiving messages over UDP
 *  use when you need to send to various sinks from this source
 *
 *  Parameters: mavSinkClass mavSink
 *  Return:     nothing global ip address is assigned to the chosen host
 *----------------------------------------------------------------------------*/
void chooseMavlinkSink( mavSinkClass mavSink )
{
   switch(mavSink)
   {
      case MAV_TO_ARDUPILOT:
      MavLinkBuf.RemoteIpAddr[0u] = Ardupilot_IpAddr[0u]; MavLinkBuf.RemoteIpAddr[1u] = Ardupilot_IpAddr[1u]; MavLinkBuf.RemoteIpAddr[2u] =Ardupilot_IpAddr[2u];MavLinkBuf.RemoteIpAddr[3u] = Ardupilot_IpAddr[3u];
      break;
      
      case MAV_TO_GROUND1:
      MavLinkBuf.RemoteIpAddr[0u] = Com1Gs_IpAddr[0u]; MavLinkBuf.RemoteIpAddr[1u] = Com1Gs_IpAddr[1u]; MavLinkBuf.RemoteIpAddr[2u] =Com1Gs_IpAddr[2u];MavLinkBuf.RemoteIpAddr[3u] = Com1Gs_IpAddr[3u];
      break;
      
      case MAV_TO_JOY:
      MavLinkBuf.RemoteIpAddr[0u] = GimJoy_IpAddr[0u]; MavLinkBuf.RemoteIpAddr[1u] = GimJoy_IpAddr[1u]; MavLinkBuf.RemoteIpAddr[2u] =GimJoy_IpAddr[2u];MavLinkBuf.RemoteIpAddr[3u] = GimJoy_IpAddr[3u];
      break;

      case MAV_TO_FLIGHT:
      MavLinkBuf.RemoteIpAddr[0u] = Air_IpAddr[0u]; MavLinkBuf.RemoteIpAddr[1u] = Air_IpAddr[1u]; MavLinkBuf.RemoteIpAddr[2u] =Air_IpAddr[2u];MavLinkBuf.RemoteIpAddr[3u] = Air_IpAddr[3u];
      break;

      case MAV_TO_FIREGS:
      MavLinkBuf.RemoteIpAddr[0u] = FirGs_IpAddr[0u]; MavLinkBuf.RemoteIpAddr[1u] = FirGs_IpAddr[1u]; MavLinkBuf.RemoteIpAddr[2u] =FirGs_IpAddr[2u];MavLinkBuf.RemoteIpAddr[3u] = FirGs_IpAddr[3u];
      break;
      
      default:
      break;
   }
}
#endif /* end mavlink UDP multiple sink function include */
/*-----------------------------------------------------------------------------
 *      checksum:  Function to calculate a checksum for old version SimpleBGC message
 *
 *  Parameters: unsigned char *fdata, unsigned int sz
 *  Return:     unsigned char (8bit checksum)
 *----------------------------------------------------------------------------*/
uint8_t checksum(const unsigned char *fdata, uint16_t sz)                       // A Function to calculate a checksum for old version of simpleBGC
{
  int16_t cnt_ck;
  uint8_t checksum;
  for(cnt_ck=0u, checksum=0u; cnt_ck<sz; cnt_ck++)                              // initialise checksum to zero and iterate through each char in the message
    checksum+=fdata[cnt_ck];                                                    // Add them together
  return checksum;
}

/*-----------------------------------------------------------------------------
 *      DataChangeDetected:  Return true if a datachange occurred
 *
 *  Parameters: uint16_t dataVal, uint16_t dataLast, uint8_t delta
 *  Return:     uint8_t true if the new value is greater than the old by more than delta
 *----------------------------------------------------------------------------*/
uint8_t DataChangeDetected( uint16_t dataVal, uint16_t dataLast, uint8_t delta) // Function to detect a data change by more than X
{
   return (uint8_t) (abs(dataVal - dataLast)>delta);                            // 1 for change 0 for no-change
}

/*-----------------------------------------------------------------------------
 *      Scale_Raw_Value:  Scale a raw float value
 *
 *  Parameters: uint16_t raw_min, uint16_t raw_max, float scale_min, float scale_max, uint16_t raw_value
 *  Return:     float32_t : scaled value
 *----------------------------------------------------------------------------*/
float32_t Scale_Raw_Value( int16_t raw_min, int16_t raw_max, float32_t scale_min, float32_t scale_max, int16_t raw_value)
{
    return  ((( (float32_t) (scale_max - scale_min) / (float32_t) (raw_max - raw_min)) * raw_value) + (float32_t) scale_min);
}

/*-----------------------------------------------------------------------------
 *      Scale_Float_Value:  Scale a floating point real value to an integer ADC scale
 *
 *  Parameters: float32_t raw_min, float32_t raw_max, uint16_t scale_min, uint16_t scale_max, float32_t float_value
 *  Return:     uint16_t : scaled integer 16 bit value
 *----------------------------------------------------------------------------*/
uint16_t Scale_Float_Value( float32_t raw_min, float32_t raw_max, uint16_t scale_min, uint16_t scale_max, float32_t float_value)
{
    return (uint16_t) ((( ((float32_t) (scale_max - scale_min)) / (raw_max - raw_min)) * float_value) + scale_min);
}
/*-----------------------------------------------------------------------------
 *      http_read_cb :  State engine and reader for http chunked block
 *
 *  Parameters: HttpConnect_t* conn, int revents, SOCKET_Intern_Dsc *sock1
 *  Return:     int16_t : state of the read to return to the next interrupt
 *----------------------------------------------------------------------------*/
#if defined (HTTP_USED)
HTTP_LIB int16_t http_read_cb( HttpConnect_t* conn, int16_t revents, SOCKET_Intern_Dsc *sock1  )
{
  int16_t r;
  int16_t room_avail=0, bytes_received=0, bytes_received2=0;
  int16_t bytes_left;
  int16_t funcRet=-1;

  if (conn->state==C_READING_HEADERS)                                           // http read state is reading headers
  {
//   do{
     if (find_end_of_http_headers(conn->buf, conn->read_pos, &conn->body_ptr))
     {
        parse_headers(conn);
        if (conn->bytes_to_read<0 && !conn->chunked)                            // didnt read anything and not chunked
        {
          funcRet=Net_Ethernet_Intern_disconnectTCP(sock1);                     // conn_close(conn, 0);   close TCP
          conn->fail_counter=++conn->fail_counter % UINT16_MAX;                 // inc_fail(conn);  count MIB Counter
          if (funcRet==1)
             //funcRet=Net_Ethernet_Intern_connectTCP(XY_USE_IP, XY_DEF_PORT, XY_DEF_PORT, &sock1);
             funcRet=Net_Ethernet_Intern_connectTCP((unsigned char*) &conn->ip_addr, conn->rcv_pt, conn->dst_pt, &sock1);   // open_socket(conn);  re-open socket
          return (funcRet | HTTP_NO_READ_NOT_CHUNK);                            // no bytes to read and connection not chunked
        }

        if (!conn->bytes_to_read)                                               // empty body
        {
          // rearm_socket(conn);
          return HTTP_NO_READ_CHUNK;                                            // empty body
        }
        conn->state=C_READING_BODY;                                             // set state to reading body

        if (!conn->chunked)
        {
          if (conn->bytes_received>=conn->bytes_to_read)
          {
            // already read all
            // rearm_socket(conn);
            return HTTP_HEAD_NO_BODY;                                           // head received no body
          }
        }
        else
        {
          r=decode_chunked_stream((HttpChunkDecode_t*) &conn->cdstate, (char*) &conn->body_ptr,(int16_t*) &conn->bytes_received);
          if (r<=0)                                                             // no data
          {
            funcRet=Net_Ethernet_Intern_disconnectTCP(sock1);                   // conn_close(conn, 0);   close TCP
            conn->fail_counter=++conn->fail_counter % UINT16_MAX;               // inc_fail(conn);  count MIB Counter
            if (funcRet==1)
               funcRet=Net_Ethernet_Intern_connectTCP(&conn->ip_addr, conn->rcv_pt, conn->dst_pt, &sock1);   // open_socket(conn);  re-open socket
            return ((funcRet) | HTTP_NO_CHUNK);                                 // no data
          }
          else if (r>0)                                                         // data
          {
            // read all
            // rearm_socket(conn);  re-arm
            // do we not wish to continue here ?? return HTTP_HEAD_CHUNK;
            return READ_CHUNK_HEAD;                                             // read chunked head
          }
        }
        //return;
      }
      else
        return HTTP_INVALID_HEAD;
  }

  if (conn->state==C_READING_BODY)                                              // reading http body
  {
// do {
    room_avail=sizeof(conn->buf);
    if (conn->bytes_to_read>0)
    {
      bytes_left=conn->bytes_to_read - conn->bytes_received;
      if (bytes_left<room_avail) room_avail=bytes_left;
    }
    //   change this one as its in interrupt -----------------------                              bytes_received=conn_read(conn, conn->buf, room_avail);
    if (bytes_received<=0)
    {
      if (bytes_received==ERR_AGAIN) return HTTP_BODY_EMPTY;                    // return no body found
      if (bytes_received==ERR_RDCLOSED)
      {
        funcRet=Net_Ethernet_Intern_disconnectTCP(sock1);                       // conn_close(conn, 0);   close TCP
        conn->fail_counter=++conn->fail_counter % UINT16_MAX;                   // inc_fail(conn);  count MIB Counter
        if (funcRet==1)
          funcRet=Net_Ethernet_Intern_connectTCP(&conn->ip_addr, conn->rcv_pt, conn->dst_pt, &sock1);   // open_socket(conn);  re-open socket
        return ((funcRet) | HTTP_BODY_EMPTY);
      }
    }

    if (!conn->chunked)                                                         // body not chunked
    {
      conn->bytes_received+=bytes_received;
      if (conn->bytes_received>=conn->bytes_to_read)                            // everything is read
      {
          // read all
          //rearm_socket(conn);
         return HTTP_BODY_READ;                                                 // return http body read
      }
    }
    else
    {
       bytes_received2=bytes_received;
       r=decode_chunked_stream(&conn->cdstate, conn->buf, &bytes_received2);    // decode chunked stream
       if (r<=0)                                                                // nothing returned from decode chunked stream call
       {
         funcRet=Net_Ethernet_Intern_disconnectTCP(sock1);                      // conn_close(conn, 0);   close TCP
         conn->fail_counter=++conn->fail_counter % UINT16_MAX;                  // inc_fail(conn);  count MIB Counter
         if (funcRet==1)
           funcRet=Net_Ethernet_Intern_connectTCP(&conn->ip_addr, conn->rcv_pt, conn->dst_pt, &sock1);   // open_socket(conn);  re-open socket
         return (funcRet | HTTP_BODY_NO_CHUNK);                                 // no chunked body decoded
       }
       else if (r>0)                                                            // something returned from decoded read of chunk
       {
          conn->bytes_received+=bytes_received2;
          // read all
          //rearm_socket(conn);
          return HTTP_BODY_CHUNK;                                               // read the chunked body
       }
    }

    // }                                                                         while (bytes_received==room_avail);
    //return;
  }
  return HTTP_UNKNOWN;
}

uint64_t osGetSystemTime()
{
   return g_timer5_counter;
}
/*-----------------------------------------------------------------------------
 *      dateRIONTimeToUnixTime:  Return the unix time (seconds since 1-1-1970)
 *                          from the RION object containing the time
 *
 *  Parameters: rionUTCDateTimeField_t *dt
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t dateRIONTimeToUnixTime(rionUTCDateTimeField_t *dt)                     // Return the unix time (seconds since 1-1-1970) can be used as aa single uint32 for time or for EGD protocol
{
    uint16_t second = dt->sec;                                                  // 0-59
    uint16_t minute = dt->min;                                                  // 0-59
    uint16_t hour = dt->hour;                                                   // 0-23
    uint16_t day = dt->day - 1U;                                                // 0-30
    uint16_t month = dt->month - 1U;                                            // 0-11
    uint16_t year;                                                              // Year 4 digit (convert below or return a zero on error)

    if (dt->year <= 2019U)                                                      // The year is invalid from now
    {
       if ((dt->year >= 0U) && (dt->year <= 99U))                               // Did he miss the 2000 part and only do the last 2 figures ?
          dt->year += 2000U;                                                    // Add the 2000
       else
          return 0U;                                                            // Invalid year given
    }
    else if (dt->year >= 9999U)
    {
       return 0U;                                                               // Invalid Year
    }

    year = dt->year - UNIX_REFERENCE_YEAR;                                      // now subtract 1970 the reference year

    return ((((year / 4U * (365U * 4U + 1U) + Unixdays[year % 4U][month] + day) * 24U + hour) * 60U + minute) * 60U + second);
}
/*******************************************************************************
* Function Name: airMapAddPosn
********************************************************************************
* Summary:
* create object for position (from mavlink) to airmap telemetry
* Parameters:
* const mavlink_gps_input_t *gpsDat, mavlink_global_position_int_t *gpInt, 
* AirMap_Position_t *amPosn, rionUTCDateTimeField_t *syncedTime
* Return:
*  uint8_t
*
*******************************************************************************/
uint8_t airMapAddPosn(const mavlink_gps_input_t *gpsDat, mavlink_global_position_int_t *gpInt, AirMap_Position_t *amPosn, rionUTCDateTimeField_t *syncedTime )
{
  uint8_t ret = 0u;
  if ((((gpsDat != NULL) && (amPosn != NULL)) && (gpInt != NULL)) && (syncedTime != NULL))
  {
     amPosn->latitude = (gpInt->lat / 1e7f);                                    /*< [degE7] Latitude, expressed*/
     amPosn->longitude = (gpInt->lon / 1e7f);                                   /*< [degE7] Longitude, expressed*/
     amPosn->altitude_msl = (gpInt->alt / 1000.0f);                             /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
     amPosn->altitude_gl = (gpInt->relative_alt / 1000.0f);                     /*< [mm] Altitude above ground*/
     amPosn->horizontal_accuracy = gpsDat->horiz_accuracy;
     amPosn->timestamp = dateRIONTimeToUnixTime(syncedTime);                    /* secs since 1970 */
     ret = 1;
  }
  return ret;
}
/*******************************************************************************
* Function Name: airMapAddVelocity
********************************************************************************
* Summary:
* create object for velocity (from mavlink) to airmap telemetry
* Parameters:
* const mavlink_gps_input_t *gpsDat,  AirMap_Velocity_t *amSpeed, rionUTCDateTimeField_t *syncedTime
* Return:
*  uint8_t
*
*******************************************************************************/
uint8_t airMapAddVelocity(const mavlink_gps_input_t *gpsDat,  AirMap_Velocity_t *amSpeed, rionUTCDateTimeField_t *syncedTime )
{
  uint8_t ret = 0u;
  if ( (gpsDat != NULL) && (amSpeed != NULL) )
  {
     amSpeed->velocity_x = gpsDat->vn;                                          /* velocity m/s North */
     amSpeed->velocity_y = gpsDat->ve;                                          /* velocity m/s East */
     amSpeed->velocity_z = gpsDat->vd;                                          /* velocity m/s Down */
     amSpeed->timestamp = dateRIONTimeToUnixTime(syncedTime);                   /* secs since 1970 */
     ret = 1;
  }
  return ret;
}
/*******************************************************************************
* Function Name: airMapaddaTtit
********************************************************************************
* Summary:
* create object for attitude (from mavlink) to airmap telemetry
* Parameters:
* float32_t *press,  const mavlink_attitude_quaternion_t *attQuart,  
* AirMap_Attitude_t *amAttit, rionUTCDateTimeField_t *syncedTime
* Return:
*  uint8_t
*
*******************************************************************************/
uint8_t airMapaddaTtit(const mavlink_attitude_quaternion_t *attQuart,  AirMap_Attitude_t *amAttit, rionUTCDateTimeField_t *syncedTime )
{
  uint8_t ret = 0u;
  uint64_t lastIntegration = amAttit->timestamp;
  
  if (((attQuart != NULL) && (amAttit != NULL) ) && (lastIntegration != 0u))
  {
     amAttit->timestamp = dateRIONTimeToUnixTime(syncedTime);                   /*  secs since 1970  */
     amAttit->yaw = ((attQuart->yawspeed * 180.0f) / PI) * (amAttit->timestamp - lastIntegration);   /* radians per second to degrees 0-360 */
     if (((attQuart->rollspeed * 180.0f) / PI) > 180.0f)                        /* radians per second to degrees -180 - 180 */
       amAttit->roll = -(((attQuart->rollspeed * 180.0f) / PI) - 180.0f) * (amAttit->timestamp - lastIntegration);
     else
       amAttit->roll = ((attQuart->rollspeed * 180.0f) / PI) * (amAttit->timestamp - lastIntegration);
     if (((attQuart->pitchspeed * 180.0f) / PI) > 180.0f)                       /* radians to degrees -180 - 180 */
       amAttit->pitch = -(((attQuart->pitchspeed * 180.0f) / PI) - 180.0f) * (amAttit->timestamp - lastIntegration);
     else
       amAttit->pitch = ((attQuart->pitchspeed * 180.0f) / PI) * (amAttit->timestamp - lastIntegration);
     ret = 1;
  }
  else
  {
     if (lastIntegration == 0u)                                                 /* first call the function */
        amAttit->timestamp = dateRIONTimeToUnixTime(syncedTime);                /*  secs since 1970  */
  }
  return ret;
}
/*******************************************************************************
* Function Name: airMapaddPress
********************************************************************************
* Summary:
* create object for pressure to airmap telemetry
* Parameters:
* float32_t *press,  AirMap_Barometer_t *amBaro, rionUTCDateTimeField_t *syncedTime
* Return:
*  uint8_t
*
*******************************************************************************/
uint8_t airMapaddPress( float32_t *press,  AirMap_Barometer_t *amBaro, rionUTCDateTimeField_t *syncedTime )
{
  uint8_t ret = 0u;
  if ( (press != NULL) && (amBaro != NULL) )
  {
     amBaro->pressure = *press;
     amBaro->timestamp = dateRIONTimeToUnixTime(syncedTime);                    /* secs since 1970  */
     ret = 1;
  }
  return ret;
}
/*******************************************************************************
* Function Name: checkHttpRespUbiDots
********************************************************************************
* Summary:
* checks http response from ubidots iot server
* Parameters:
* HttpConnect_t *conn, HttpClientContext_t *context
* Return:
*  float32_t
*
*******************************************************************************/
float32_t checkHttpRespUbiDots( HttpConnect_t *conn, HttpClientContext_t *context )
{
  char *pch;
  float32_t res = 0u; // UBI_ERROR_VALUE;

  if (conn == NULL)
  {    }
  else if (conn->state == C_READ_COMPLETED)
  {
    // POST
    if (strcmp(context->method1, "POST") == 0)
    {
      pch = strstr(conn->buf, "OK");
      if (pch != NULL)
      {
        res = 1;
      }
    }
    else
    {
       // LV
       pch = strchr(conn->buf, '|');
       if (pch != NULL)
       {
          res = atof(pch + 1);
       }
    }
  }
  return res;
}
/*
 * Stores the float type value into the char array input
 * @str_value [Mandatory] char payload pointer to store the value.
 * @value [Mandatory] Float value to convert
 */
void UbiDot_floatToChar(char *str_value, float32_t value)
{
  char temp_arr[20u];
  uint8_t j = 0u;
  uint8_t k = 0u;

  sprintf(temp_arr, "%17g", value);

  while (j < 20u)
  {
    if (temp_arr[j] != ' ')             /* strip space chars */
        {
      str_value[k] = temp_arr[j];
      k++;
    }
    if (temp_arr[j] == '\0')          /* termninate end */
        {
      str_value[k] = temp_arr[j];
      break;
    }
    j++;
  }
}

/*
 * Adds to the context structure values to retrieve later it easily by the user
 * returns 0 if we have overflowed the storage buffer else a number
 */
uint8_t UbiDotaddContext(char *key_label, char *key_value, UbiProtocolHandler_t *proto, ContextUbi_t *_context)
{
  uint8_t ret = 0u;
  if (((proto != NULL) && (key_label != NULL)) && (_context != NULL))
  {
     (_context + proto->_current_context)->key_label = key_label;
     (_context + proto->_current_context)->key_value = key_value;
     proto->_current_context=++proto->_current_context % UINT8_MAX;
/* comment in only if you want to latch to the maximum wihout anymore we roll over at present
  if (proto->_current_context >= UBI_CONTEXT_MAX_VALUES)
  {
    proto->_current_context = UBI_CONTEXT_MAX_VALUES;
  }  */
     ret = proto->_current_context;
  }
  return ret;
}

/*
 * Adds to the point (dot) structure values to retrieve later it easily by the user
 * returns 0 if we have overflowed the storage buffer or an error with the call else
 * its the number of the next item
 */
uint8_t UbiDotaddDot(char *key_label, float32_t *dot_value, UbiProtocolHandler_t *proto, ubiValue_t *_dots, rionUTCDateTimeField_t *syncedTime )
{
  uint8_t ret = 0u;
  if (( (proto != NULL) && (key_label != NULL) ) && (_dots != NULL) )
  {
     if ( proto->_current_value < (sizeof(ubiValue_t) * UBI_POINT_MAX_VALUES) ) /* can be disabled incrementer should prevent */
     {
        (_dots + proto->_current_value)->variable_label = key_label;
        (_dots + proto->_current_value)->dot_value = *dot_value;
        (_dots + proto->_current_value)->dot_timestamp_seconds = syncedTime->sec;
        (_dots + proto->_current_value)->dot_timestamp_millis = syncedTime->millis;
        proto->_current_value=++proto->_current_value % (UBI_POINT_MAX_VALUES % UINT8_MAX);  /* increment number of samples bounded by the storage array and variable size */
/*   comment in only if you want to latch to the maximum wihout anymore we roll over at present
  
     if (proto->_current_context >= UBI_POINT_MAX_VALUES)
     {
       proto->_current_context = UBI_POINT_MAX_VALUES;
     }
*/
        ret = proto->_current_value;
     }
  }
  return ret;
}

/**
 * Builds the TCP or UDP payload to send and saves it to the input char pointer.
 * @payload [Mandatory] char payload pointer to store the built structure.
 * @timestamp_global [Optional] If set, it will be used for any dot without
 * timestamp.
 */
void UbiDotbuildAnyPayload(char *payload, UbiProtocolHandler_t *ubiDots, ubiValue_t *_dots)
{
  uint8_t i;
  char str_value[20u];
  uint16_t timestamp_millis;
  char milliseconds[3u];
  uint8_t units;
  uint8_t dec;
  uint8_t hund;

  if (((ubiDots == NULL) || (_dots == NULL)) || (payload == NULL))
  {
     return;
  }

  if ((sizeof(_dots) < ubiDots->_current_value) || (sizeof(payload) < (MIN_UBI_PAYLOAD_HEAD_LEN + (MIN_UBI_PAYLOAD_ITEM_LEN * ubiDots->_current_value))))
  {
     return;
  }
  switch (ubiDots->ubiType)                                                     /* build the header */
  {
    case UBI_HTTP:
    sprintf(payload, "{");
    break;

    case UBI_TCP:
    sprintf(payload, "");
    sprintf(payload, "%s|POST|%s|", ubiDots->user_agent, ubiDots->_token);
    sprintf(payload, "%s%s:%s", payload, ubiDots->device_label, ubiDots->device_name);
    sprintf(payload, "%s=>", payload);
    break;

    case UBI_UDP:
    sprintf(payload, "");
    sprintf(payload, "%s|POST|%s|", ubiDots->user_agent, ubiDots->_token);
    sprintf(payload, "%s%s:%s", payload, ubiDots->device_label, ubiDots->device_name);
    sprintf(payload, "%s=>", payload);
    break;

    default:
    return;
  }
  sprintf(payload, "%s=>", payload);
  for (i = 0; i < ubiDots->_current_value;)
  {
    UbiDot_floatToChar(str_value, _dots->dot_value);
    switch (ubiDots->ubiType)
    {
        case UBI_HTTP:
        sprintf(payload, "{");
        sprintf(payload, "%s\"%s\":{\"value\":%s", payload, _dots->variable_label, str_value);
        if (_dots->dot_timestamp_seconds != NULL)                               // Adds timestamp seconds
        {
          sprintf(payload, "%s,\"timestamp\":%lu", payload, _dots->dot_timestamp_seconds);

          if (_dots->dot_timestamp_millis != NULL)                              // Adds timestamp milliseconds
          {
            timestamp_millis = _dots->dot_timestamp_millis;
            units = timestamp_millis % 10u;
            dec = (timestamp_millis / 10u) % 10u;
            hund = (timestamp_millis / 100u) % 10u;
            sprintf(milliseconds, "%d%d%d", hund, dec, units);
            sprintf(payload, "%s%s", payload, milliseconds);
          }
          else
          {
            sprintf(payload, "%s000", payload);
          }
          if (_dots->dot_context != NULL)                                       // Adds dot context
          {
              sprintf(payload, "%s,\"context\": {%s}", payload, _dots->dot_context);
          }
          sprintf(payload, "%s}", payload);                                     /* end it now keep this from above incase you can send more than one value */
        }
        break;

        case UBI_TCP:
        sprintf(payload, "%s%s:%s", payload, _dots->variable_label, str_value);
        if (_dots->dot_context != NULL)                                         // Adds dot context
        {
           sprintf(payload, "%s$%s", payload, _dots->dot_context);
        }
        if (_dots->dot_timestamp_seconds != NULL)                               // Adds timestamp seconds
        {
          sprintf(payload, "%s@%lu", payload, _dots->dot_timestamp_seconds);
          sprintf(payload, "%s,\"timestamp\":%lu", payload, (_dots + i)->dot_timestamp_seconds);

          if (_dots->dot_timestamp_millis != NULL)                              // Adds timestamp milliseconds
          {
            timestamp_millis = _dots->dot_timestamp_millis;
            units = timestamp_millis % 10u;
            dec = (timestamp_millis / 10u) % 10u;
            hund = (timestamp_millis / 100u) % 10u;
            sprintf(milliseconds, "%d%d%d", hund, dec, units);
            sprintf(payload, "%s%s", payload, milliseconds);
          }
          else
          {
            sprintf(payload, "%s000", payload);
          }
        }
        break;

        case UBI_UDP:
        sprintf(payload, "%s%s:%s", payload, _dots->variable_label, str_value);
        if (_dots->dot_context != NULL)                                         // Adds dot context
        {
           sprintf(payload, "%s$%s", payload, _dots->dot_context);
        }
        if (_dots->dot_timestamp_seconds != NULL)                               // Adds timestamp seconds
        {
          sprintf(payload, "%s@%lu", payload, _dots->dot_timestamp_seconds);
          sprintf(payload, "%s,\"timestamp\":%lu", payload, (_dots + i)->dot_timestamp_seconds);

          if (_dots->dot_timestamp_millis != NULL)                              // Adds timestamp milliseconds
          {
            timestamp_millis = _dots->dot_timestamp_millis;
            units = timestamp_millis % 10u;
            dec = (timestamp_millis / 10u) % 10u;
            hund = (timestamp_millis / 100u) % 10u;
            sprintf(milliseconds, "%d%d%d", hund, dec, units);
            sprintf(payload, "%s%s", payload, milliseconds);
          }
          else
          {
            sprintf(payload, "%s000", payload);
          }
        }
        break;

        default:
        return;
    }

    if (i < (ubiDots->_current_value - 1))                                      /* if we arent the last */
    {
      sprintf(payload, "%s,", payload);                                         /* append a comma */
    }
    else                                                                        /* its the last one */
    {
      switch (ubiDots->ubiType)                                                 /* check the type and CLOSE the communication string */
      {
         case UBI_HTTP:
         sprintf(payload, "%s}", payload);                                      /* tag the end */
         break;
         
         case UBI_TCP:
         sprintf(payload, "%s|end", payload);                                   /* tag the end */
         break;
         
         case UBI_UDP:
         sprintf(payload, "%s|end", payload);                                   /* tag the end */
         break;
      }
      ubiDots->_current_value = 0;                                              /* reset the data collector */
    }

    i++;
    _dots++;                                                                    /* increment to the next value required to send */
  }

}
/*******************************************************************************
* Function Name: httpClientCheckTimeout
********************************************************************************
* Summary:
* checks current time against timestamp
* Parameters:
* HttpClientContext_t *context
* Return:
*  error_t  ERROR_TIMEOUT if timeout occurred
*
*******************************************************************************/
error_t httpClientCheckTimeout(HttpClientContext_t *context)
{
   error_t error=0u;
   int64_t TimeDuration=0ull;

   calculateTime2Now( &TimeDuration, &context->timestamp );                       //Check whether the timeout has elapsed
   if(TimeDuration >= context->timeout)
   {
      error = ERROR_TIMEOUT;                                                    //Report a timeout error
   }
   else
   {
      error = ERROR_WOULD_BLOCK;                                                //The operation would block
   }
   return error;                                                                //Return status code
}
/**
 * @brief Establish a connection with the specified HTTP server
 * @param[in] context Pointer to the HTTP client context
 * @param[in] serverIpAddr IP address of the HTTP server to connect to
 * @param[in] serverPort Port number
 * @return Error code
 **/
error_t httpClientConnect(HttpClientContext_t *context, uint8_t *serverIpAddr, uint16_t serverPort, SOCKET_Intern_Dsc *socket )
{
   error_t error;

   if(context == NULL)                                                          //Make sure the HTTP client context is valid
      return ERROR_INVALID_PARAMETER;

   error = NO_ERROR;                                                            //Initialize status code

   while(!error)                                                                //Establish connection with the HTTP server
   {
      if(context->state == HTTP_CLIENT_STATE_DISCONNECTED)                      //Check HTTP connection state
      {
         if(serverIpAddr != NULL)                                               //First connection attempt?
         {
            memcpy((void*) &context->serverIpAddr, (void*)serverIpAddr, 4u);    //Save the IP address of the HTTP server
            context->serverPort = serverPort;                                   //Save the TCP port number to be used

#if (HTTP_CLIENT_AUTH_SUPPORT == ENABLED)
            httpClientInitAuthParams(&context->authParams);                     //HTTP authentication is not used for the first connection attempt
#endif
         }

         //Open network connection
         //error = httpClientOpenConnection(context);
         if(!error)                                                             //Check status code
         {
            httpClientChangeState(context, HTTP_CLIENT_STATE_CONNECTING);       //Establish network connection
         }
      }
      else if(context->state == HTTP_CLIENT_STATE_CONNECTING)
      {
         //Establish network connection
         //error = httpClientEstablishConnection(context, &context->serverIpAddr, context->serverPort);

         if(error == NO_ERROR)                                                  //Check status code
         {
            httpClientChangeState(context, HTTP_CLIENT_STATE_CONNECTED);        //The client is connected to the HTTP server
         }
         else if(error == ERROR_WOULD_BLOCK || error == ERROR_TIMEOUT)
         {
            error = httpClientCheckTimeout(context);                            //Check whether the timeout has elapsed
         }
         else
         {
            //A communication error has occured
         }
      }
      else if(context->state == HTTP_CLIENT_STATE_CONNECTED)
      {
         break;                                                                 //The HTTP client is connected
      }
      else
      {
         error = ERROR_WRONG_STATE;                                             //Invalid state
      }
   }

   //Failed to establish connection with the HTTP server?
   if(error != NO_ERROR && error != ERROR_WOULD_BLOCK)
   {

      httpClientCloseConnection(context, socket);                                       //Clean up side effects
      httpClientChangeState(context, HTTP_CLIENT_STATE_DISCONNECTED);           //Update HTTP connection state
   }

   //Return status code
   return error;
}
/**
 * @brief Update HTTP request state
 * @param[in] context Pointer to the HTTP client context
 * @param[in] newState New state to switch to
 **/

void httpClientChangeRequestState(HttpClientContext_t *context, HttpRequestState newState)
{
   //Update HTTP request state
   context->requestState = newState;

   //Save current time
   context->timestamp = osGetSystemTime();
}
/**
 * @brief Shutdown network connection
 * @param[in] context Pointer to the HTTP client context
 * @return Error code
 **/
error_t httpClientShutdownConnection(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket)
{
   error_t error= NO_ERROR;                                                     //Initialize status code
#if (HTTP_CLIENT_TLS_SUPPORT == ENABLED)
   /* Valid TLS context?
   if(context->tlsContext != NULL)
   {
      Shutdown TLS session
      error = tlsShutdown(context->tlsContext);
   }    */
#endif
   if(!error)                                                                   //Check status code
   {
      if(context->socket != NULL)                                               //Valid TCP socket?
      {
         //error = socketShutdown(context->socket, SOCKET_SD_BOTH);               //Shutdown TCP connection
         error= Net_Ethernet_Intern_disconnectTCP(socket);
      }
   }
   return error;                                                                //Return status code
}
/**
 * @brief Receive data using the relevant transport protocol
 * @param[in] context Pointer to the HTTP client context
 * @param[out] data Buffer into which received data will be placed
 * @param[in] size Maximum number of bytes that can be received
 * @param[out] received Number of bytes that have been received
 * @param[in] flags Set of flags that influences the behavior of this function
 * @return Error code
 **/

error_t httpClientReceiveData(HttpClientContext_t *context, void *dataV, size_t size, size_t *received, uint8_t flags)
{
   error_t error=0u;

   if ((context == NULL) || (dataV == NULL))
     return -1;
#if (HTTP_CLIENT_TLS_SUPPORT == ENABLED)
   //TLS-secured connection?
   /* if(context->tlsContext != NULL)                                               if you have asked for encryption
   {
      Receive TLS-encrypted data
      error = tlsRead(context->tlsContext, dataV, size, received, flags);
   }
   else  */
#endif
   /* {
         Receive data
         error = socketReceive(context->socket, dataV, size, received, flags);    */
      if (sizeof(dataV)<=size)
        memcpy(dataV,(void*) &SeqCam_DATA.TCP_Buffer,size);                     /* copy from the TCP receive object on port80 (currently sequoia) */
   /*}*/

   return error;                                                                /* Return status code */
}

void httpClientChangeState(HttpClientContext_t *context, HttpClientState newState)
{
   context->state = newState;                                                   //Update HTTP connnection state
   context->timestamp = osGetSystemTime();                                      //Save current time
}
/**
 * @brief Close network connection
 * @param[in] context Pointer to the HTTP client context
 **/
void httpClientCloseConnection(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket)
{

#if (HTTP_CLIENT_TLS_SUPPORT == ENABLED)
   //Release TLS context
   /* if(context->tlsContext != NULL)
   {
      tlsFree(context->tlsContext);
      context->tlsContext = NULL;
   }  */
#endif
   if(context->socket1 != NULL)                                                 //Close TCP connection if not already closed
   {
      //socketClose(context->socket);
      if (Net_Ethernet_Intern_disconnectTCP(socket)==1)                         // socket successfully CLOSED
      {
         context->socket1 = NULL;
      }
   }
}

/**
 * @brief Gracefully disconnect from the HTTP server
 * @param[in] context Pointer to the HTTP client context
 * @return Error code
 **/
error_t httpClientDisconnect(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket)
{
   error_t error;

   if(context == NULL)                                                          //Make sure the HTTP client context is valid
      return ERROR_INVALID_PARAMETER;

   error = NO_ERROR;                                                            //Initialize status code

   while(!error)                                                                //Execute HTTP command sequence
   {
      if(context->state == HTTP_CLIENT_STATE_CONNECTED)                         //Check HTTP connection state
      {
         httpClientChangeState(context, HTTP_CLIENT_STATE_DISCONNECTING);       //Gracefully disconnect from the HTTP server
      }
      else if(context->state == HTTP_CLIENT_STATE_DISCONNECTING)
      {
         error = httpClientShutdownConnection(context, socket);                 //Shutdown connection

         if(error == 1)                                                         //Check status code 1=success
         {
            httpClientCloseConnection(context, socket);                                 //Close connection
            httpClientChangeState(context, HTTP_CLIENT_STATE_DISCONNECTED);     //Update HTTP connection state
         }
         else
         {
            error = httpClientCheckTimeout(context);                            //Check whether the timeout has elapsed
            error = httpClientShutdownConnection(context, socket);              //try again to Shutdown connection
         }
      }
      else if(context->state == HTTP_CLIENT_STATE_DISCONNECTED)
      {
         break;                                                                 //We are done
      }
      else
      {
         error = ERROR_WRONG_STATE;                                             //Invalid state
      }
   }

   if(error != NO_ERROR && error != ERROR_WOULD_BLOCK)                          //Failed to gracefully disconnect from the HTTP server?
   {
      httpClientCloseConnection(context,socket);                                       //Close connection
      httpClientChangeState(context, HTTP_CLIENT_STATE_DISCONNECTED);           //Update HTTP connection state
   }

   return error;                                                                //Return status code
}
/**
 * @brief Format default HTTP request header
 * @param[in] context Pointer to the HTTP client context
 * @return Error code
 **/
error_t httpClientFormatRequestHeader(HttpClientContext_t *context)
{
   size_t n;
   const char_t *version;

   if(context->version == HTTP_VERSION_1_0)                                     //Check HTTP version
   {
      version = "HTTP/1.0";                                                     //Select protocol version 1.0

      //Under HTTP/1.0, connections are not considered persistent unless a
      //keepalive header is included
      context->keepAlive = FALSE;

      //a persistent connection with an HTTP/1.0 client cannot make use of
      //the chunked transfer-coding, and therefore must use a Content-Length
      //for marking the ending boundary of each message (refer to RFC 2068,
      //section 19.7.1)
      context->chunkedEncoding = FALSE;
   }
   else if(context->version == HTTP_VERSION_1_1)
   {
      version = "HTTP/1.1";                                                     //Select protocol version 1.1

      //In HTTP/1.1, all connections are considered persistent unless declared
      //otherwise
      context->keepAlive = TRUE;

      //The chunked encoding modifies the body of a message in order to transfer
      //it as a series of chunks, each with its own size indicator
      context->chunkedEncoding = FALSE;
   }
   else
   {
      return ERROR_INVALID_VERSION;                                             //Unknown HTTP version
   }
   strcpy(context->method1, "GET");                                              //Set default HTTP request method

   //The Request-Line begins with a method token, followed by the Request-URI
   //and the protocol version, and ending with CRLF
   n = sprintf(context->buffer, "GET / %s\r\n", version);

   n += sprintf(context->buffer + n, "\r\n");                                   //Terminate the HTTP request header with a CRLF sequence

   context->bufferLen = n;                                                      //Set the length of the request header
   context->bufferPos = 0;

   return NO_ERROR;                                                             //Successful processing
}
/**
 * @brief Create a new HTTP request
 * @param[in] context Pointer to the HTTP client context
 * @return Error code
 *
 * example :- httpClientCreateRequest(&httpClientContext);
 *
 **/
error_t httpClientCreateRequest(HttpClientContext_t *context)
{
   error_t error=0U;

   if(context == NULL)
      return ERROR_INVALID_PARAMETER;                                           // Make sure the HTTP client context is valid

   error = httpClientFormatRequestHeader(context);                              // Format default HTTP request header

   if(!error)                                                                   // Check status code
   {
      context->bodyLen = 0u;                                                    // The HTTP request body is empty
      context->bodyPos = 0u;
      context->statusCode = 0u;                                                 // Reset status code
      httpClientChangeRequestState(context, HTTP_REQ_STATE_FORMAT_HEADER);      // Update HTTP request state
   }

   return error;                                                                // Return status code
}

/**
 * @brief Set HTTP request method
 * @param[in] context Pointer to the HTTP client context
 * @param[in] method NULL-terminating string containing the HTTP method
 * @return Error code
 *
 * httpClientSetMethod(&httpClientContext, "POST");
 *
 **/
error_t httpClientSetMethod(HttpClientContext_t *context, const char_t *method)
{
   size_t m;
   size_t n;
   char_t *p;

   if(context == NULL || method == NULL)
      return ERROR_INVALID_PARAMETER;                                           // Check parameters

   n = strlen((char*)method);                                                   // Compute the length of the HTTP method

   if(n == 0 || n > HTTP_CLIENT_MAX_METHOD_LEN)
      return ERROR_INVALID_LENGTH;                                              // Make sure the length of the user name is acceptable

   if(context->bufferLen > HTTP_CLIENT_BUFFER_SIZE)
      return ERROR_INVALID_SYNTAX;                                              // Make sure the buffer contains a valid HTTP request

   context->buffer[context->bufferLen] = '\0';                                  // Properly terminate the string with a NULL character

   p = strchr(context->buffer, ' ');                                            // The Request-Line begins with a method token

   if(p == NULL)
      return ERROR_INVALID_SYNTAX;                                              // Any parsing error?

   m = p - context->buffer;                                                     // Compute the length of the current method token

   if((context->bufferLen + n - m) > HTTP_CLIENT_BUFFER_SIZE)
      return ERROR_BUFFER_OVERFLOW;                                             // Make sure the buffer is large enough to hold the new HTTP request method

   memmove(context->buffer + n, p, context->bufferLen + 1 - m);                 // Make room for the new method token

   strncpy((unsigned char*)context->buffer,(unsigned char*)method, n);          // Copy the new method token

   context->bufferLen = context->bufferLen + n - m;                             // Adjust the length of the request header

   strcpy((char*)context->method1,(char*)method);                               // Save HTTP request method

   return NO_ERROR;                                                             // Successful processing
}
/**
 * @brief Set request URI
 * @param[in] context Pointer to the HTTP client context
 * @param[in] uri NULL-terminated string that contains the resource name
 * @return Error code
 *
 *  httpClientSetUri(&httpClientContext, APP_HTTP_URI);
 *
 **/
error_t httpClientSetUri(HttpClientContext_t *context, const char_t *uri)
{
   size_t m;
   size_t n;
   char_t *p;
   char_t *q;

   if(context == NULL || uri == NULL)                                           // Check parameters
      return ERROR_INVALID_PARAMETER;
   if(uri[0] == '\0')                                                           // The resource name must not be empty
      return ERROR_INVALID_PARAMETER;
   if(context->requestState != HTTP_REQ_STATE_FORMAT_HEADER)                    // Check HTTP request state
      return ERROR_WRONG_STATE;
   if(context->bufferLen > HTTP_CLIENT_BUFFER_SIZE)                             // Make sure the buffer contains a valid HTTP request
      return ERROR_INVALID_SYNTAX;
   context->buffer[context->bufferLen] = '\0';                                  // Properly terminate the string with a NULL character
   p = strchr(context->buffer, ' ');                                            // The Request-Line begins with a method token
   if(p == NULL)
      return ERROR_INVALID_SYNTAX;                                              // Any parsing error?
   p++;                                                                         // The method token is followed by the Request-URI
   q = strpbrk(p, " ?");                                                        // Point to the end of the Request-URI
   if(q == NULL)
      return ERROR_INVALID_SYNTAX;                                              // Any parsing error?
   m = q - p;                                                                   // Compute the length of the current URI
   n = strlen((char*)uri);                                                      // Compute the length of the new URI
   if((context->bufferLen + n - m) > HTTP_CLIENT_BUFFER_SIZE)                   // Make sure the buffer is large enough to hold the new resource name
      return ERROR_BUFFER_OVERFLOW;
   memmove(p + n, q, context->buffer + context->bufferLen + 1 - q);             // Make room for the new resource name
   strncpy((char*)p,(char*)uri, n);                                             // Copy the new resource name
   context->bufferLen = context->bufferLen + n - m;                             // Adjust the length of the request header

   return NO_ERROR;                                                             // Successful processing

}
/**
 * @brief Add a key/value pair to the query string
 * @param[in] context Pointer to the HTTP client context
 * @param[in] name NULL-terminated string that holds the parameter name
 * @param[in] value NULL-terminated string that holds the parameter value
 * @return Error code
 *
 *  httpClientAddQueryParam(&httpClientContext, "param1", "value1");
 *
 **/
error_t httpClientAddQueryParam(HttpClientContext_t *context, const char_t *name, const char_t *value)
{
   size_t nameLen;
   size_t valueLen;
   char_t separator;
   char_t *p;

   if(context == NULL || name == NULL)                                          //Check parameters
      return ERROR_INVALID_PARAMETER;

   if(name[0] == '\0')                                                          //The parameter name must not be empty
      return ERROR_INVALID_PARAMETER;

   if(context->requestState != HTTP_REQ_STATE_FORMAT_HEADER)                    //Check HTTP request state
      return ERROR_WRONG_STATE;

   if(context->bufferLen > HTTP_CLIENT_BUFFER_SIZE)                             //Make sure the buffer contains a valid HTTP request
      return ERROR_INVALID_SYNTAX;

   context->buffer[context->bufferLen] = '\0';                                  //Properly terminate the string with a NULL character

   p = strchr(context->buffer, ' ');                                            //The Request-Line begins with a method token

   if(p == NULL)                                                                //Any parsing error?
      return ERROR_INVALID_SYNTAX;

   p = strpbrk(p + 1, " ?");                                                    //The method token is followed by the Request-URI

   if(p == NULL)                                                                //Any parsing error?
      return ERROR_INVALID_SYNTAX;

   if(*p == '?')                                                                //The question mark is used as a separator
   {
      p = strchr(p + 1, ' ');                                                   //Point to the end of the query string

      if(p == NULL)                                                             //Any parsing error?
         return ERROR_INVALID_SYNTAX;

      separator = '&';                                                          //multiple query parameters are separated by an ampersand
   }
   else
   {
      separator = '?';                                                          //The query string is not present
   }
   nameLen = strlen((char*)name);                                               //Compute the length of the parameter value

   if(value == NULL)                                                            //Empty parameter value?
   {
      if((context->bufferLen + nameLen + 1) > HTTP_CLIENT_BUFFER_SIZE)          //Make sure the buffer is large enough to hold the new query parameter
         return ERROR_BUFFER_OVERFLOW;

      memmove(p + nameLen + 1, p, context->buffer + context->bufferLen + 1 - p);      //Make room for the new query parameter
      p[0] = separator;                                                         //Multiple query parameters are separated by a delimiter
      strncpy((char*) p + 1,(char*) name, nameLen);                             //Copy parameter name
      context->bufferLen += nameLen + 1;                                        //Adjust the length of the request header
   }
   else
   {
      valueLen = strlen((char*)value);                                          // Compute the length of the parameter value

      if((context->bufferLen + nameLen + valueLen + 2u) > HTTP_CLIENT_BUFFER_SIZE)
         return ERROR_BUFFER_OVERFLOW;                                          // Make sure the buffer is large enough to hold the new query parameter

      memmove(p + nameLen + valueLen + 2u, p, context->buffer +                 // Make room for the new query parameter
         context->bufferLen + 1u - p);
      p[0u] = separator;                                                        // Multiple query parameters are separated by a delimiter
      strncpy((char*)p + 1u,(char*) name, nameLen);                             // Copy parameter name
      p[nameLen + 1u] = '=';                                                    // The field name and value are separated by an equals sign
      strncpy((char*)p + nameLen + 2u,(char*) value, valueLen);                 // Copy parameter value
      context->bufferLen += nameLen + valueLen + 2u;                            // Adjust the length of the request header
   }

   return NO_ERROR;                                                             // Successful processing
}
/**
 * @brief Parse Connection header field
 * @param[in] context Pointer to the HTTP client context
 * @param[in] value NULL-terminated string that contains the field value
 * @return Error code
 **/

error_t httpClientParseConnectionField(HttpClientContext_t *context, const char_t *value)
{
   size_t n;

   while(value[0] != '\0')                                                      //Parse the comma-separated list
   {
      n = strcspn((char*)value,(char*) ", \t");                                 //Get the length of the current token

      if(n == 10 && !strncmp((char*)value,(char*) "keep-alive", 10))            //Check current value
      {
         if(context->requestState == HTTP_REQ_STATE_FORMAT_HEADER)              //Check HTTP request state
         {
            context->keepAlive = TRUE;                                          //Make the connection persistent
         }
      }
      else if(n == 5 && !strncmp((char*)value,(char*) "close", 5))
      {
         context->keepAlive = FALSE;                                            //The connection will be closed after completion of the response
      }
      else
      {
         //Just for sanity
      }

      if(value[n] != '\0')                                                      //Advance the pointer over the separator
         n++;
      value += n;                                                               //Point to the next token
   }
   return NO_ERROR;                                                             //Successful processing
}
/**
 * @brief Parse Transfer-Encoding header field
 * @param[in] context Pointer to the HTTP client context
 * @param[in] value NULL-terminated string that contains the field value
 * @return Error code
 **/

error_t httpClientParseTransferEncodingField(HttpClientContext_t *context, const char_t *value)
{
   if(!strcmp((char*)value,(char*) "chunked"))                                  //Check the value of the header field
   {
      //If a Transfer-Encoding header field is present and the chunked
      //transfer coding is the final encoding, the message body length
      //is determined by reading and decoding the chunked data until the
      //transfer coding indicates the data is complete
      context->chunkedEncoding = TRUE;

      //If a message is received with both a Transfer-Encoding and a
      //Content-Length header field, the Transfer-Encoding overrides
      //the Content-Length (refer to RFC 7230, section 3.3.3)
      context->bodyLen = 0;
   }
   return NO_ERROR;                                                             //Successful processing
}
/**
 * @brief Parse Content-Length header field
 * @param[in] context Pointer to the HTTP client context
 * @param[in] value NULL-terminated string that contains the field value
 * @return Error code
 **/
error_t httpClientParseContentLengthField(HttpClientContext_t *context, const char_t *value)
{
   char_t *p;

   //If a valid Content-Length header field is present without
   //Transfer-Encoding, its decimal value defines the expected message
   //body length in octets (refer to RFC 7230, section 3.3.3)
   if(!context->chunkedEncoding)
   {
      context->bodyLen = strtoul(value, &p, 10);                                //Retrieve the length of the body

      if(*p != '\0')                                                            //Any parsing error?
         return ERROR_INVALID_SYNTAX;
   }
   return NO_ERROR;                                                             //Successful processing
}
/**
 * @brief Add a header field to the HTTP request
 * @param[in] context Pointer to the HTTP client context
 * @param[in] name NULL-terminated string that holds the header field name
 * @param[in] value NULL-terminated string that holds the header field value
 * @return Error code
 *
 * example :-
 *     httpClientAddHeaderField(&httpClientContext, "User-Agent", "Mozilla/5.0");
 *     httpClientAddHeaderField(&httpClientContext, "Content-Type", "text/plain");
 *     for posting JSON object "Content-Type", "application/json; charset=utf-8"
 *     httpClientAddHeaderField(&httpClientContext, "Transfer-Encoding", "chunked");
 *
 **/
error_t httpClientAddHeaderField(HttpClientContext_t *context, const char_t *name, const char_t *value)
{
   error_t error;
   size_t n;
   size_t nameLen;
   size_t valueLen;

   if(context == NULL || name == NULL || value == NULL)                         //Check parameters
      return ERROR_INVALID_PARAMETER;

   if(name[0] == '\0')                                                          //The field name must not be empty
      return ERROR_INVALID_PARAMETER;

   if(context->requestState != HTTP_REQ_STATE_FORMAT_HEADER && context->requestState != HTTP_REQ_STATE_FORMAT_TRAILER)    //Check HTTP request state
   {
      return ERROR_WRONG_STATE;
   }

   if(context->bufferLen < 2 || context->bufferLen > HTTP_CLIENT_BUFFER_SIZE)   //Make sure the buffer contains a valid HTTP request
      return ERROR_INVALID_SYNTAX;

   nameLen = strlen((char*)name);                                               //Retrieve the length of the field name and value
   valueLen = strlen((char*)value);

   n = nameLen + valueLen + 4;                                                  //Determine the length of the new header field

   if((context->bufferLen + n) > HTTP_CLIENT_BUFFER_SIZE)                       //Make sure the buffer is large enough to hold the new header field
      return ERROR_BUFFER_OVERFLOW;

   //Each header field consists of a case-insensitive field name followed
   //by a colon, optional leading whitespace and the field value
   sprintf(context->buffer + context->bufferLen - 2, "%s: %s\r\n\r\n",name, value);

   if(!strcmp((char*)name, "Connection"))                                       //Check header field name
   {
      error = httpClientParseConnectionField(context, value);                   //Parse Connection header field
   }
   else if(!strcmp((char*)name, "Transfer-Encoding"))
   {
      error = httpClientParseTransferEncodingField(context, value);             //Parse Transfer-Encoding header field
   }
   else if(!strcmp((char*)name, "Content-Length"))
   {
      error = httpClientParseContentLengthField(context, value);                //Parse Content-Length header field
   }
   else
   {
      error = NO_ERROR;                                                         //Discard unknown header fields
   }
   context->bufferLen += n;                                                     //Adjust the length of the request header
   return error;                                                                //Return status code
}
/**
 * @brief Initialize HTTP authentication parameters
 * @param[in] authParams HTTP authentication parameters
 **/
void httpClientInitAuthParams(HttpClientAuthParams_t *authParams)
{
   authParams->mode = HTTP_AUTH_MODE_NONE;                                      //Reset authentication scheme to its default value
   strcpy(authParams->realm, "");                                               //Clear realm

#if (HTTP_CLIENT_DIGEST_AUTH_SUPPORT == ENABLED)
   authParams->qop = HTTP_AUTH_QOP_NONE;                                        //Reset authentication parameters
   authParams->algorithm = NULL;
   strcpy(authParams->nonce, "");
   strcpy(authParams->cnonce, "");
   strcpy(authParams->opaque, "");
   authParams->stale = FALSE;                                                   //Reset stale flag
#endif
}
/**
 * @brief Check the Octecs from the IP Address to make a valid request for the
 * IMEI, it splits the IP into tokes by the "." sparator, if there are 4 so it
 * is ok
 *
 * @return true There a valid 4 byte ip address passed
 * @return false There is NOT a valid 4 byte ip address passed
 */
uint8_t UbiTCP_checkIpAddress( char *ipAddress )
{

  char *separator = ".";
  char *octet;
  uint8_t octecNumber = 0u;
  uint8_t ret=0;

  if (ipAddress == NULL)
  {
     octet = strtok(ipAddress, separator);
     while (octet != NULL)
     {
       octecNumber = ++octecNumber % UINT8_MAX;
       octet = strtok(NULL, separator);
     }
     if (octecNumber == 4u)
     {
       ret=1u;
     }
     else
     {
       /* just for misra */
     }
  }
  return ret;
}
/**
 * @brief Format Authorization header field
 * @param[in] context Pointer to the HTTP client context
 * @return Error code
 **/
error_t httpClientFormatAuthorizationField(HttpClientContext_t *context)
{
   size_t n;
   char_t *p;
   HttpClientAuthParams_t *authParams;
   uint16_t RandomNumcnonce=0u;
   uint16_t seed = 1u;
   uint16_t randnum;   
   //Make sure the buffer contains a valid HTTP request
   if(context->bufferLen < 2 || context->bufferLen > HTTP_CLIENT_BUFFER_SIZE)
      return ERROR_INVALID_SYNTAX;

   //Point to the HTTP authentication parameters
   authParams = &context->authParams;

#if (HTTP_CLIENT_BASIC_AUTH_SUPPORT == ENABLED)
   //Basic authentication scheme?
   if(authParams->mode == HTTP_AUTH_MODE_BASIC)
   {
      size_t k;
      size_t m;

      //Calculate the length of the username and password
      n = strlen(authParams->username) + strlen(authParams->password);

      //Make sure the buffer is large enough
      if((context->bufferLen + n + 22) > HTTP_CLIENT_BUFFER_SIZE)
         return ERROR_BUFFER_OVERFLOW;

      //Point to the buffer where to format the Authorization header field
      p = context->buffer + context->bufferLen - 2;

      //Format Authorization header field
      n = sprintf(p, "Authorization: Basic ");

      //The client sends the username and password, separated by a single
      //colon character, within a Base64-encoded string in the credentials
      m = sprintf(p + n, "%s:%s", authParams->username, authParams->password);

      //The first pass calculates the length of the Base64-encoded string
      base64Encode(p + n, m, NULL, &k);

      //Make sure the buffer is large enough
      if((context->bufferLen + n + k) > HTTP_CLIENT_BUFFER_SIZE)
         return ERROR_BUFFER_OVERFLOW;

      //The second pass encodes the string using Base64
      base64Encode(p + n, m, p + n, &k);
      //Update the total length of the header field
      n += k;

      //Make sure the buffer is large enough
      if((context->bufferLen + n + 2) > HTTP_CLIENT_BUFFER_SIZE)
         return ERROR_BUFFER_OVERFLOW;

      //Terminate the header field with a CRLF sequence
      sprintf(p + n, "\r\n\r\n");

      //Adjust the length of the request header
      context->bufferLen = context->bufferLen + n + 2;
   }
   else
#endif
#if (HTTP_CLIENT_DIGEST_AUTH_SUPPORT == ENABLED)
   //Digest authentication scheme?
   if(authParams->mode == HTTP_AUTH_MODE_DIGEST)
   {
      error_t error=0u;
      const char_t *q;
      const char_t *uri;
      size_t uriLen;
      char response[HTTP_CLIENT_MAX_RESPONSE_LEN + 1u];

      //Properly terminate the string with a NULL character
      context->buffer[context->bufferLen] = '\0';

      //The Request-Line begins with a method token
      q = strchr(context->buffer, ' ');
      //Any parsing error?
      if(q == NULL)
         return ERROR_INVALID_SYNTAX;

      //The method token is followed by the Request-URI
      uri = q + 1;

      //Point to the end of the Request-URI
      q = strchr((char*)uri, ' ');
      //Any parsing error?
      if(q == NULL)
         return ERROR_INVALID_SYNTAX;

      //Compute the length of the current URI
      uriLen = q - uri;

      //Check quality of protection
      if(authParams->qop == HTTP_AUTH_QOP_AUTH || authParams->qop == HTTP_AUTH_QOP_AUTH_INT)
      {
         //Make sure that a valid callback function has been registered
         /* if(context->randCallback == NULL)
            return ERROR_PRNG_NOT_READY;                    */

         //A cryptographically strong random number generator must be used to
         //generate the cnonce
         //error = context->randCallback(authParams->cnonce, HTTP_CLIENT_CNONCE_SIZE);
         
         /* ==== this is an alternative method more random ==== */
         randnum = randhash(&authParams->init, seed);
         sprintf((char*)&authParams->cnonce,"%x",(int16_t)randhashdouble(randnum,&authParams->init,0.0f,(float64_t)HTTP_CLIENT_CNONCE_SIZE));
         /* ==== old method ==== 
         RandomNumcnonce= setRandRange(0, HTTP_CLIENT_CNONCE_SIZE);
         sprintf(authParams->cnonce,"%x",RandomNumcnonce);
          */
         //Any error to report?
         if(error)
            return error;

         //Convert the byte array to hex string
         //   httpEncodeHexString(authParams->cnonce, HTTP_CLIENT_CNONCE_SIZE,
         //   authParams->cnonce);

         //Count of the number of requests (including the current request)
         //that the client has sent with the nonce value in this request
         authParams->nc++;
      }

      //Perform digest operation
      //  error = httpClientComputeDigest(authParams, context->method, strlen(context->method), uri, uriLen, response);
      //Any error to report?
      if(error)
         return error;

      //Determine the length of the header field
      n = strlen((char*)authParams->username) + strlen((char*)authParams->realm) + uriLen + sizeof(authParams->nonce) + sizeof(authParams->cnonce) + strlen((char*)response) + strlen((char*)authParams->opaque);

      //Make sure the buffer is large enough
      if((context->bufferLen + n + 121) > HTTP_CLIENT_BUFFER_SIZE)
         return ERROR_BUFFER_OVERFLOW;

      //Point to the buffer where to format the Authorization header field
      p = context->buffer + context->bufferLen - 2;

      //Format Authorization header field
      n = sprintf(p, "Authorization: Digest ");

      //Format username and realm parameter
      n += sprintf(p + n, "username=\"%s\", ", authParams->username);
      n += sprintf(p + n, "realm=\"%s\", ", authParams->realm);

      //Format uri parameter
      n += sprintf(p + n, "uri=\"");
      strncpy((char*)p + n,(char*) uri,(int16_t) uriLen);
      n += uriLen;
      n += sprintf(p + n, "\", ");

      //Format nonce parameter
      n += sprintf(p + n, "nonce=\"%s\", ", authParams->nonce);

      //Check quality of protection
      if(authParams->qop == HTTP_AUTH_QOP_AUTH)
      {
         //Format qop, nc, cnonce parameters
         n += sprintf(p + n, "qop=auth, ");
         n += sprintf(p + n, "nc=%08x, ", authParams->nc);
         n += sprintf(p + n, "cnonce=\"%s\", ", authParams->cnonce);
      }

      //Format response parameter
      n += sprintf(p + n, "response=\"%s\"", response);

      //The opaque parameter should be returned by the client unchanged in
      //the Authorization header field of subsequent requests
      if(authParams->opaque[0] != '\0')
      {
         //Format opaque parameter
         n += sprintf(p + n, ", opaque=\"%s\"", authParams->opaque);
      }

      //Terminate the header field with a CRLF sequence
      sprintf(p + n, "\r\n\r\n");

      //Adjust the length of the request header
      context->bufferLen = context->bufferLen + n + 2;
   }
   else
#endif
   //Unknown authentication scheme?
   {
      //Just for sanity
   }

   //Successful processing
   return NO_ERROR;
}
/**
 * @brief Write HTTP request header
 * @param[in] context Pointer to the HTTP client context
 * @return Error code
 *
 *       error = httpClientWriteHeader(&httpClientContext);
 *
 **/
error_t httpClientWriteHeader(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket)
{
   error_t error;
   size_t pBuf=0u;                                                              // buffer pointer

   if(context == NULL)                                                          // Make sure the HTTP client context is valid
      return ERROR_INVALID_PARAMETER;

   error = NO_ERROR;                                                            // Initialize status code

   while(!error)                                                                // Send HTTP request header
   {
      if(context->state == HTTP_CLIENT_STATE_DISCONNECTED || context->state == HTTP_CLIENT_STATE_CONNECTING)   // Check HTTP connection state
      {
         error = httpClientConnect(context, NULL, 0, socket);                           // If the HTTP connection is not persistent, then a new connection must be established
      }
      else if(context->state == HTTP_CLIENT_STATE_CONNECTED)
      {
         if(context->requestState == HTTP_REQ_STATE_FORMAT_HEADER)              // Check HTTP request state
         {
#if (HTTP_CLIENT_AUTH_SUPPORT == ENABLED)
            if(context->authParams.mode != HTTP_AUTH_MODE_NONE && context->authParams.username[0] != '\0') // HTTP authentication requested by the server?
            {
               error = httpClientFormatAuthorizationField(context);             // Format Authorization header field
            }
#endif
            if(!error)                                                          // Check status code
            {
               // TRACE_DEBUG(context->buffer,"HTTP request header:\r\n%s");      // Dump HTTP request header
               httpClientChangeRequestState(context, HTTP_REQ_STATE_SEND_HEADER);                 //Initiate the sending process
            }
         }
         else if(context->requestState == HTTP_REQ_STATE_SEND_HEADER)
         {
            if(context->bufferPos < context->bufferLen)                         // Any remaining data to be sent?
            {
               error = httpClientSendData(context, context->buffer + context->bufferPos, context->bufferLen - context->bufferPos, &pBuf, 0, socket);              // Send more data
               if(error == NO_ERROR || error == ERROR_TIMEOUT)                  // Check status code
               {
                  context->bufferPos += pBuf;                                   // Advance data pointer
               }
            }
            else
            {
               if(!strcmp(context->method1, "POST") || !strcmp(context->method1, "PUT") || !strcmp(context->method1, "PATCH"))
               {                                                                //The request header has been successfully transmitted
                  httpClientChangeRequestState(context, HTTP_REQ_STATE_SEND_BODY);  //POST, PUT and PATCH requests have a body
               }
               else
               {
                  httpClientChangeRequestState(context,HTTP_REQ_STATE_RECEIVE_STATUS_LINE);
               }                                                                // GET, HEAD and DELETE requests do not have a body
            }
         }
         else if(context->requestState == HTTP_REQ_STATE_SEND_BODY || context->requestState == HTTP_REQ_STATE_RECEIVE_STATUS_LINE)
         {
            break;                                                              // We are done
         }
         else
         {
            error = ERROR_WRONG_STATE;                                          // Invalid HTTP request state
         }
      }
      else
      {

         error = ERROR_WRONG_STATE;                                             // Invalid HTTP connection state
      }
   }

   if(error == ERROR_WOULD_BLOCK || error == ERROR_TIMEOUT)                     // Check status code
   {
      error = httpClientCheckTimeout(context);                                  // Check whether the timeout has elapsed
   }

   return error;                                                                // Return status code
}
/**
 * @brief Format chunk-size field
 * @param[in] context Pointer to the HTTP client context
 * @param[in] length Size of the chunk
 * @return Error code
 **/
//#define PRIXSIZE X
error_t httpClientFormatChunkSize(HttpClientContext_t *context, size_t length)
{
   size_t n = 0;

   //The chunked encoding modifies the body in order to transfer it as a
   //series of chunks, each with its own size indicator
   context->bodyLen = length;
   context->bodyPos = 0;

   //Check whether the chunk is the first one or not
   if(context->requestState == HTTP_REQ_STATE_SEND_CHUNK_DATA)
   {
      //Terminate the data of the previous chunk with a CRLF sequence
      n += sprintf(context->buffer + n, "\r\n");
   }

   //The chunk-size field is a string of hex digits indicating the size of the
   //chunk
   n += sprintf(context->buffer + n, "%X \r\n", length);

   //Check whether the chunk is the last one
   if(length == 0)
   {
      //The trailer allows the sender to include additional HTTP header fields
      //at the end of the message. The trailer is terminated with a CRLF
      n += sprintf(context->buffer + n, "\r\n");
   }

   //Set the length of the chunk-size field
   context->bufferLen = n;
   context->bufferPos = 0;

   //The chunked encoding is ended by any chunk whose size is zero
   if(length == 0)
   {
      //The last chunk is followed by an optional trailer
      httpClientChangeRequestState(context, HTTP_REQ_STATE_FORMAT_TRAILER);
   }
   else
   {
      //Send the chunk-size field
      httpClientChangeRequestState(context, HTTP_REQ_STATE_SEND_CHUNK_SIZE);
   }

   //Successful processing
   return NO_ERROR;
}
/**
 * @brief Send data using the relevant transport protocol
 * @param[in] context Pointer to the HTTP client context
 * @param[in] data Pointer to a buffer containing the data to be transmitted
 * @param[in] length Number of bytes to be transmitted
 * @param[out] written Actual number of bytes written (optional parameter)
 * @param[in] flags Set of flags that influences the behavior of this function
 * @return Error code
 **/
error_t httpClientSendData(HttpClientContext_t *context, const void *dataV, size_t length, size_t *written, uint8_t flags, SOCKET_Intern_Dsc *socket)
{

   error_t error;

#if (HTTP_CLIENT_TLS_SUPPORT == ENABLED)
   //TLS-secured connection?
/*   if(context->tlsContext != NULL)
   {
      //Send TLS-encrypted data
      error = tlsWrite(context->tlsContext, dataV, length, written, flags);
   }
   else       */
#endif
   /* {        */
      //Transmit data
      //error = socketSend(context->socket, dataV, length, written, flags);
   error= Net_Ethernet_Intern_putBytesTCP((char *)dataV, length, socket);
   /* }   */

   return error;                                                                //Return status code
}
/**
 * @brief Write HTTP request body
 * @param[in] context Pointer to the HTTP client context
 * @param[in] data Pointer to the buffer containing the data to be transmitted
 * @param[in] length Number of data bytes to send
 * @param[out] written Actual number of bytes written (optional parameter)
 * @param[in] flags Set of flags that influences the behavior of this function
 * @return Error code
 *
 * Example :- error = httpClientWriteBody(&httpClientContext, "Hello World!", 12, NULL, 0);
 *
 **/
error_t httpClientWriteBody(HttpClientContext_t *context, const void *dataV, size_t length, size_t *written, uint8_t flags, SOCKET_Intern_Dsc *socket)
{
   error_t error;
   size_t n=0u;                                                                 // buffer pointer
   size_t totalLength;

   if(context == NULL)                                                          // Make sure the HTTP client context is valid
      return ERROR_INVALID_PARAMETER;

   if(context->state != HTTP_CLIENT_STATE_CONNECTED)                            // Check HTTP connection state
      return ERROR_WRONG_STATE;

   error = NO_ERROR;                                                            // Initialize status code
   totalLength = 0u;                                                            // Actual number of bytes written

   while(totalLength < length && !error)                                        //Send as much data as possible
   {
      if(context->requestState == HTTP_REQ_STATE_SEND_BODY)                     //Check HTTP request state
      {
         if(context->chunkedEncoding)                                           //Chunked transfer encoding?
         {
            error = httpClientFormatChunkSize(context, length - totalLength);   //The chunked encoding modifies the body in order to transfer it as a series of chunks, each with its own size indicator
         }
         else
         {
            n = length - totalLength;                                           //Number of bytes left to send
            if(n <= (context->bodyLen - context->bodyPos))                      //The length of the body shall not exceed the value specified in the Content-Length field
            {
               error = httpClientSendData(context, dataV, n, &n, flags, socket);         //Send request body
               if(error == NO_ERROR || error == ERROR_TIMEOUT)                  //Check status code
               {
                  if(n > 0)                                                     //Any data transmitted?
                  {
                     dataV = (uint8_t *) dataV + n;                             //Advance data pointer
                     totalLength += n;
                     context->bodyPos += n;
                     context->timestamp = osGetSystemTime();                    //Save current time
                  }
               }
            }
            else
            {
               error = ERROR_INVALID_LENGTH;                                    //Report an error
            }
         }
      }
      else if(context->requestState == HTTP_REQ_STATE_SEND_CHUNK_SIZE)
      {
         if(context->bufferPos < context->bufferLen)                            //Send the chunk-size field
         {

            error = httpClientSendData(context, context->buffer + context->bufferPos, context->bufferLen - context->bufferPos, &n, 0, socket);             //Send more data

            if(error == NO_ERROR || error == ERROR_TIMEOUT)                     //Check status code
            {
               context->bufferPos += n;                                         //Advance data pointer
            }
         }
         else
         {
            httpClientChangeRequestState(context, HTTP_REQ_STATE_SEND_CHUNK_DATA);              //Send chunk data
         }
      }
      else if(context->requestState == HTTP_REQ_STATE_SEND_CHUNK_DATA)
      {
         if(context->bodyPos < context->bodyLen)                                // The data stream is divided into a series of chunks
         {
            n = min(length - totalLength, context->bodyLen - context->bodyPos); // The length of the chunk shall not exceed the value specified in the chunk-size field

            error = httpClientSendData(context, dataV, n, &n, flags, socket);            // Send chunk data

            if(error == NO_ERROR || error == ERROR_TIMEOUT)                     // Check status code
            {
               if(n > 0)                                                        // Any data transmitted?
               {
                  dataV = (uint8_t *) dataV + n;                                // Advance data pointer
                  totalLength += n;
                  context->bodyPos += n;
                  context->timestamp = osGetSystemTime();                       //Save current time
               }
            }
         }
         else
         {
            error = httpClientFormatChunkSize(context, length - totalLength);              //The chunked encoding modifies the body in order to transfer it as a series of chunks, each with its own size indicator
         }
      }
      else
      {
         error = ERROR_WRONG_STATE;                                             //Invalid state
      }
   }
   if(error == ERROR_WOULD_BLOCK || error == ERROR_TIMEOUT)                     //Check status code
   {

      error = httpClientCheckTimeout(context);                                  //Check whether the timeout has elapsed
   }

   if(written != NULL)
      *written = totalLength;                                                   //Total number of data that have been written

   return error;                                                                //Return status code
}
static int16_t isprintable(char c)
{
  return 0x20U <= c && c <= 0x7EU;
}
/**
 * @brief Check whether a string contains valid characters
 * @param[in] s Pointer to the string
 * @param[in] length Length of the string
 * @param[in] charset Acceptable charset
 * @return Error code
 **/
error_t httpCheckCharset(const char_t *s, size_t length, uint8_t charset)
{
   error_t error;
   size_t i;
   uint8_t c;
   uint8_t m;

   //Initialize status code
   error = NO_ERROR;

   //Parse string
   for(i = 0; i < length; i++)
   {
      //Get current character
      c = (uint8_t) s[i];

      //Any 8-bit sequence of data
      m = HTTP_CHARSET_OCTET;

      //Check if character is a control character
      if(iscntrl(c))
         m |= HTTP_CHARSET_CTL;

      //Check if character is printable
      if(isprintable(c) && c <= 126)
         m |= HTTP_CHARSET_TEXT | HTTP_CHARSET_VCHAR;

      //Check if character is blank
      if(c == ' ' || c == '\t')
         m |= HTTP_CHARSET_TEXT | HTTP_CHARSET_LWS;

      //Check if character is alphabetic
      if(isalpha(c))
         m |= HTTP_CHARSET_TCHAR | HTTP_CHARSET_ALPHA;

      //Check if character is decimal digit
      if(isdigit(c))
         m |= HTTP_CHARSET_TCHAR | HTTP_CHARSET_DIGIT;

      //Check if character is hexadecimal digit
      if(isxdigit(c))
         m |= HTTP_CHARSET_HEX;

      //Check if character is in the extended character set
      if(c >= 128)
         m |= HTTP_CHARSET_TEXT | HTTP_CHARSET_OBS_TEXT;

      //Check if character is a token character
      if(strchr("!#$%&'*+-.^_`|~", c))
         m |= HTTP_CHARSET_TCHAR;

      //Invalid character?
      if((m & charset) == 0)
         error = ERROR_INVALID_SYNTAX;
   }

   //Return status code
   return error;
}
#ifdef outout
/**
 * @brief Parse WWW-Authenticate header field
 * @param[in] context Pointer to the HTTP client context
 * @param[in] value NULL-terminated string that contains the value of header field
 * @return Error code
 **/
error_t httpClientParseWwwAuthenticateField(HttpClientContext_t *context, const char_t *value)
{
   error_t error;
   const char_t *p;
   HttpParam_t param;
   HttpWwwAuthenticateHeader_t authHeader;

   //Point to the first character of the string
   p = value;

   //Get the first token
   error = httpParseParam(&p, &param);

   //The WWW-Authenticate header field indicates the authentication scheme(s)
   //and parameters applicable to the target resource
   while(!error)
   {
      //The authentication scheme must be a valid token followed by a
      //BWS character
      if(param.value == NULL && (*p == ' ' || *p == '\t'))
      {
         //Clear authentication parameters
         memset(&authHeader, 0, sizeof(HttpWwwAuthenticateHeader));

#if (HTTP_CLIENT_DIGEST_AUTH_SUPPORT == ENABLED && HTTP_CLIENT_MD5_SUPPORT == ENABLED)
         //If the algorithm parameter is not present, it is assumed to be
         //MD5 (refer to RFC 7616, section 3.3)
         authHeader.algorithm = MD5_HASH_ALGO;
#endif
         //A case-insensitive token is used to identify the authentication
         //scheme
         if(httpCompareParamName(&param, "Basic"))
         {
            //Basic access authentication
            authHeader.mode = HTTP_AUTH_MODE_BASIC;
         }
         else if(httpCompareParamName(&param, "Digest"))
         {
            //Digest access authentication
            authHeader.mode = HTTP_AUTH_MODE_DIGEST;
         }
         else
         {
            //Unknown authentication scheme
            authHeader.mode = HTTP_AUTH_MODE_NONE;
         }

         //The authentication scheme is followed by a comma-separated list
         //of attribute-value pairs which carry the parameters necessary for
         //achieving authentication via that scheme
         while(!error)
         {
            //Parse authentication parameter
            error = httpParseParam(&p, &param);

            //Check status code
            if(!error)
            {
               //Valid attribute-value pair?
               if(param.value != NULL)
               {
                  //Realm parameter?
                  if(httpCompareParamName(&param, "realm"))
                  {
                     //The realm is a string to be displayed to users so they
                     //know which username and password to use
                     authHeader.realm = param.value;
                     authHeader.realmLen = param.valueLen;
                  }
#if (HTTP_CLIENT_DIGEST_AUTH_SUPPORT == ENABLED)
                  //Quality of protection parameter?
                  else if(httpCompareParamName(&param, "qop"))
                  {
                     //The qop parameter is a quoted string of one or more
                     //tokens indicating the "quality of protection" values
                     //supported by the server
                     httpClientParseQopParam(&param, &authHeader);
                  }
                  //Algorithm parameter?
                  else if(httpCompareParamName(&param, "algorithm"))
                  {
                     //This parameter indicates the algorithm used to produce
                     //the digest
                     httpClientParseAlgorithmParam(&param, &authHeader);
                  }
                  //Nonce parameter?
                  else if(httpCompareParamName(&param, "nonce"))
                  {
                     //The nonce is a server-specified string which should be
                     //uniquely generated each time a 401 response is made
                     authHeader.nonce = param.value;
                     authHeader.nonceLen = param.valueLen;
                  }
                  //Opaque parameter?
                  else if(httpCompareParamName(&param, "opaque"))
                  {
                     //The opaque parameter is a string of data, specified by the
                     //server, that should be returned by the client unchanged in
                     //the Authorization header field of subsequent requests
                     authHeader.opaque = param.value;
                     authHeader.opaqueLen = param.valueLen;
                  }
                  //Stale parameter?
                  else if(httpCompareParamName(&param, "stale"))
                  {
                     //The stale flag is a case-insensitive flag indicating that
                     //the previous request from the client was rejected because
                     //the nonce value was stale
                     if(httpCompareParamValue(&param, "true"))
                        authHeader.stale = TRUE;
                     else
                        authHeader.stale = FALSE;
                  }
#endif
                  //Unknown parameter?
                  else
                  {
                     //Discard unknown attributes
                  }
               }
               else
               {
                  //Parse next authentication scheme
                  break;
               }
            }
         }

#if (HTTP_CLIENT_BASIC_AUTH_SUPPORT == ENABLED)
         //Valid basic authentication parameters?
         if(authHeader.mode == HTTP_AUTH_MODE_BASIC &&
            authHeader.realmLen > 0 &&
            authHeader.realmLen <= HTTP_CLIENT_MAX_REALM_LEN)
         {
            //Save authentication mode
            context->authParams.mode = authHeader.mode;

            //Save realm
            strncpy(context->authParams.realm, authHeader.realm, authHeader.realmLen);
            context->authParams.realm[authHeader.realmLen] = '\0';
         }
         else
#endif
#if (HTTP_CLIENT_DIGEST_AUTH_SUPPORT == ENABLED)
         //Valid digest authentication parameters?
         if(authHeader.mode == HTTP_AUTH_MODE_DIGEST &&
            authHeader.algorithm != NULL &&
            authHeader.realmLen > 0 &&
            authHeader.realmLen <= HTTP_CLIENT_MAX_REALM_LEN &&
            authHeader.nonceLen > 0 &&
            authHeader.nonceLen <= HTTP_CLIENT_MAX_NONCE_LEN &&
            authHeader.opaqueLen <= HTTP_CLIENT_MAX_OPAQUE_LEN)
         {
            //Save authentication mode
            context->authParams.mode = authHeader.mode;
            //Save qop parameter
            context->authParams.qop = authHeader.qop;
            //Save digest algorithm
            context->authParams.algorithm = authHeader.algorithm;

            //Save realm
            strncpy(context->authParams.realm, authHeader.realm, authHeader.realmLen);
            context->authParams.realm[authHeader.realmLen] = '\0';

            //Save nonce value
            strncpy(context->authParams.nonce, authHeader.nonce, authHeader.nonceLen);
            context->authParams.nonce[authHeader.nonceLen] = '\0';

            //Save opaque parameter
            strncpy(context->authParams.opaque, authHeader.opaque, authHeader.opaqueLen);
            context->authParams.opaque[authHeader.opaqueLen] = '\0';

            //Save stale flag
            context->authParams.stale = authHeader.stale;
         }
         else
#endif
         //Invalid parameters
         {
            //Ignore the challenge
         }
      }
      else
      {
         //Get next token
         error = httpParseParam(&p, &param);
      }
   }

   //Successful processing
   return NO_ERROR;
}
#endif

/**
 * @brief Parse HTTP response header field
 * @param[in] context Pointer to the HTTP client context
 * @param[in] line Pointer to the header field
 * @param[in] length Length of the header field
 * @return Error code
 **/
error_t httpClientParseHeaderField(HttpClientContext_t *context, char_t *line, size_t length)
{
   error_t error;
   char_t *name;
   size_t nameLen;
   char_t *value;
   size_t valueLen;
   char_t *separator;

   //Properly terminate the string with a NULL character
   line[length] = '\0';

   //Debug message
   //TRACE_DEBUG("%s\r\n", line);

   //The string must contains visible characters only
   error = httpCheckCharset(line, length, HTTP_CHARSET_TEXT);
   //Any error to report?
   if(error)
      return error;

   //Header field values can be folded onto multiple lines if the continuation
   //line begins with a space or horizontal tab
   if(line[0] == ' ' || line[0] == '\t')
   {
      //A continuation line cannot immediately follows the Status-Line
      if(context->bufferPos == 0)
         return ERROR_INVALID_SYNTAX;

      //Remove optional leading and trailing whitespace
      value = strTrimWhitespace(line);
      //Retrieve the length of the resulting string
      valueLen = strlen(value);

      //Sanity check
      if(valueLen > 0)
      {
         //The folding LWS is replaced with a single SP before interpretation
         //of the TEXT value (refer to RFC 2616, section 2.2)
         context->buffer[context->bufferPos - 1] = ' ';

         //Save field value
         memmove(context->buffer + context->bufferPos, value, valueLen + 1);

         //Update the size of the hash table
         context->bufferLen = context->bufferPos + valueLen + 1;
      }
   }
   else
   {
      //Each header field consists of a case-insensitive field name followed
      //by a colon, optional leading whitespace, the field value, and optional
      //trailing whitespace (refer to RFC 7230, section 3.2)
      separator = strchr(line, ':');

      //Any parsing error?
      if(separator == NULL)
         return ERROR_INVALID_SYNTAX;

      //Split the line
      *separator = '\0';

      //Remove optional leading and trailing whitespace
      name = strTrimWhitespace(line);
      value = strTrimWhitespace(separator + 1);

      //Retrieve the length of the resulting strings
      nameLen = strlen(name);
      valueLen = strlen(value);

      //The field name cannot be empty
      if(nameLen == 0)
         return ERROR_INVALID_SYNTAX;

      //Check header field name
      if(!strcmp(name, "Connection"))
      {
         //Parse Connection header field
         httpClientParseConnectionField(context, value);
      }
      else if(!strcmp(name, "Transfer-Encoding"))
      {
         //Parse Transfer-Encoding header field
         httpClientParseTransferEncodingField(context, value);
      }
      else if(!strcmp(name, "Content-Length"))
      {
         //Parse Content-Length header field
         httpClientParseContentLengthField(context, value);
      }
#if (HTTP_CLIENT_AUTH_SUPPORT == ENABLED)
      //WWW-Authenticate header field found?
      else if(!strcmp(name, "WWW-Authenticate"))
      {
         //Parse WWW-Authenticate header field
         //httpClientParseWwwAuthenticateField(context, value);
      }
#endif
      else
      {
         //Discard unknown header fields
      }

      //Save each header field into a hash table by field name
      memmove(context->buffer + context->bufferPos, name, nameLen + 1);

      //Save field value
      memmove(context->buffer + context->bufferPos + nameLen + 1, value,
         valueLen + 1);

      //Update the size of the hash table
      context->bufferLen = context->bufferPos + nameLen + valueLen + 2;
   }

   //Decode the next header field
   context->bufferPos = context->bufferLen;

   //Successful processing
   return NO_ERROR;
}
/*
 * Copyright (c) 1998 Softweyr LLC.  All rights reserved.
 *
 * strtok_r, from Berkeley strtok
 * Oct 13, 1998 by Wes Peters <wes@softweyr.com>
 *
 * Copyright (c) 1988, 1993
 *    The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notices, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notices, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *
 *    This product includes software developed by Softweyr LLC, the
 *      University of California, Berkeley, and its contributors.
 *
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SOFTWEYR LLC, THE REGENTS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL SOFTWEYR LLC, THE
 * REGENTS, OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
char *strtok_r(char *s, const char *delim, char **last)
{
    char *spanp;
    int c, sc;
    char *tok;

    if (s == NULL && (s = *last) == NULL)
    {
        return NULL;
    }

    /*
     * Skip (span) leading delimiters (s += strspn(s, delim), sort of).
     */
cont:                                                                           /* not misra complient but as per original function */
    c = *s++;
    for (spanp = (char *)delim; (sc = *spanp++) != 0; )
    {
        if (c == sc)
        {
            goto cont;
        }
    }

    if (c == 0)        /* no non-delimiter characters */
    {
        *last = NULL;
        return NULL;
    }
    tok = s - 1;

    /*
     * Scan token (scan for delimiters: s += strcspn(s, delim), sort of).
     * Note that delim must have one NUL; we stop if we see that, too.
     */
    for (;;)
    {
        c = *s++;
        spanp = (char *)delim;
        do
        {
            if ((sc = *spanp++) == c)
            {
                if (c == 0)
                {
                    s = NULL;
                }
                else
                {
                    char *w = s - 1;
                    *w = '\0';
                }
                *last = s;
                return tok;
            }
        }
        while (sc != 0);
    }
}

/**
 * @brief Parse HTTP status line
 * @param[in] context Pointer to the HTTP client context
 * @param[in] line Pointer to the status line
 * @param[in] length Length of the status line
 * @return Error code
 **/

error_t httpClientParseStatusLine(HttpClientContext_t *context, char_t *line, size_t length)
{
   error_t error;
   char_t *p;
   char_t *token;

   line[length] = '\0';                                                         //Properly terminate the string with a NULL character
   //Debug message
   //TRACE_DEBUG("HTTP response header:\r\n%s\r\n", line);

   error = httpCheckCharset(line, length, HTTP_CHARSET_TEXT);                   //The string must contains visible characters only

   if(error)                                                                    //Any error to report?
      return error;

   token = strtok_r(line, " ", &p);                                             //  Parse HTTP-Version field
   //token = strtok((char*)line," ");
   
   if(token == NULL)                                                            //Any parsing error?
      return ERROR_INVALID_SYNTAX;

   if(!strcmp((char*)token,(char*)"HTTP/1.0"))                                  //Check protocol version
   {
      //Under HTTP/1.0, connections are not considered persistent unless a
      //keepalive header is included
      context->keepAlive = FALSE;
   }
   else if(!strcmp((char*)token,(char*)"HTTP/1.1"))
   {
      //In HTTP/1.1, all connections are considered persistent unless declared
      //otherwise
      if(context->version != HTTP_VERSION_1_1)
         context->keepAlive = FALSE;
   }
   else
   {
      return ERROR_INVALID_VERSION;                                             //Unknown HTTP version
   }

   token = strtok_r(NULL, " ", &p);                                             //Parse Status-Code field
   //token = strtok(NULL, " ");
   
   if(token == NULL)                                                            //Any parsing error?
      return ERROR_INVALID_SYNTAX;

   context->statusCode = strtoul(token, &p, 10);                                //The Status-Code element is a 3-digit integer

   if(*p != '\0')                                                               //Any parsing error?
      return ERROR_INVALID_SYNTAX;

   //The chunked encoding modifies the body of a message in order to transfer
   //it as a series of chunks, each with its own size indicator
   context->chunkedEncoding = FALSE;

   //For response message without a declared message body length, the message
   //body length is determined by the number of octets received prior to the
   //server closing the connection
   context->bodyLen = UINT8_MAX;

   context->bufferLen = 0;                                                      //Flush receive buffer
   context->bufferPos = 0;
   context->bodyPos = 0;

   return NO_ERROR;                                                             //Successful processing
}
/**
 * @brief Write HTTP trailer
 * @param[in] context Pointer to the HTTP client context
 * @return Error code
 **/
error_t httpClientWriteTrailer(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket)
{
   error_t error;
   size_t n;

   //Make sure the HTTP client context is valid
   if(context == NULL)
      return ERROR_INVALID_PARAMETER;

   //Check HTTP connection state
   if(context->state != HTTP_CLIENT_STATE_CONNECTED)
      return ERROR_WRONG_STATE;

   //Initialize status code
   error = NO_ERROR;

   //Send HTTP request header
   while(!error)
   {
      //Check HTTP request state
      if(context->requestState == HTTP_REQ_STATE_FORMAT_TRAILER)
      {
         //Initiate the sending process
         httpClientChangeRequestState(context, HTTP_REQ_STATE_SEND_TRAILER);
      }
      else if(context->requestState == HTTP_REQ_STATE_SEND_TRAILER)
      {
         //Any remaining data to be sent?
         if(context->bufferPos < context->bufferLen)
         {
            //Send more data
            error = httpClientSendData(context, context->buffer + context->bufferPos, context->bufferLen - context->bufferPos, &n, 0, socket);

            //Check status code
            if(error == NO_ERROR || error == ERROR_TIMEOUT)
            {
               //Advance data pointer
               context->bufferPos += n;
            }
         }
         else
         {
            //The request trailer has been successfully transmitted
            httpClientChangeRequestState(context, HTTP_REQ_STATE_RECEIVE_STATUS_LINE);
         }
      }
      else if(context->requestState == HTTP_REQ_STATE_RECEIVE_STATUS_LINE)
      {
         //We are done
         break;
      }
      else
      {
         //Invalid state
         error = ERROR_WRONG_STATE;
      }
   }

   //Check status code
   if(error == ERROR_WOULD_BLOCK || error == ERROR_TIMEOUT)
   {
      //Check whether the timeout has elapsed
      error = httpClientCheckTimeout(context);
   }

   //Return status code
   return error;
}
/**
 * @brief Read HTTP response header
 * @param[in] context Pointer to the HTTP client context
 * @return Error code
 **/
error_t httpClientReadHeader(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket)
{
   error_t error;
   size_t n=0u;                                                                 // buffer pointer

   if(context == NULL)                                                          //Make sure the HTTP client context is valid
      return ERROR_INVALID_PARAMETER;

   if(context->state != HTTP_CLIENT_STATE_CONNECTED)                            //Check HTTP connection state
      return ERROR_WRONG_STATE;

   error = NO_ERROR;                                                            //Initialize status code

   while(!error)                                                                //Send HTTP request header
   {
      if(context->requestState == HTTP_REQ_STATE_SEND_BODY || context->requestState == HTTP_REQ_STATE_SEND_CHUNK_DATA)       //Check HTTP request state
      {
         error = httpClientCloseBody(context,socket);                           //Close HTTP request body
      }
      else if(context->requestState == HTTP_REQ_STATE_FORMAT_TRAILER || context->requestState == HTTP_REQ_STATE_SEND_TRAILER)
      {
         error = httpClientWriteTrailer(context, socket);                       //The last chunk is followed by an optional trailer
      }
      else if(context->requestState == HTTP_REQ_STATE_RECEIVE_STATUS_LINE || context->requestState == HTTP_REQ_STATE_RECEIVE_HEADER)
      {
         //The CRLF sequence is always used to terminate a line
         if(context->bufferLen >= (context->bufferPos + 2) && context->buffer[context->bufferLen - 2] == '\r' && context->buffer[context->bufferLen - 1] == '\n')
         {
            context->bufferLen -= 2;                                            //Strip the CRLF at the end of the line

            if(context->requestState == HTTP_REQ_STATE_RECEIVE_STATUS_LINE)     //The first line of a response message is the Status-Line
            {
               //Thee Status-Line consists of the protocol version followed by
               //a numeric status code and its associated textual phrase
               error = httpClientParseStatusLine(context, context->buffer + context->bufferPos, context->bufferLen - context->bufferPos);

               if(!error)                                                       //Valid syntax?
               {
                  httpClientChangeRequestState(context, HTTP_REQ_STATE_RECEIVE_HEADER);  //Receive response header fields
               }
            }
            else
            {
               if(context->bufferLen == context->bufferPos)                     //An empty line indicates the end of the header fields
               {
                  error = 0u; // httpClientSaveSession(context);                       //Save TLS session
                  if(!error)                                                    //Check status code
                  {
                     httpClientChangeRequestState(context, HTTP_REQ_STATE_PARSE_HEADER); //The HTTP response header has been successfully received
                  }
               }
               else
               {
                  //The response header fields allow the server to pass additional
                  //information about the response which cannot be placed in the
                  //Status-Line
                  error = httpClientParseHeaderField(context, context->buffer + context->bufferPos, context->bufferLen - context->bufferPos);
               }
            }
         }
         else if(context->bufferLen < HTTP_CLIENT_BUFFER_SIZE)
         {
            //Receive more data
            error = httpClientReceiveData(context, context->buffer + context->bufferLen, HTTP_CLIENT_BUFFER_SIZE - context->bufferLen, &n, HTTP_FLAG_BREAK_CRLF);

            if(!error)                                                          //Check status code
            {
               context->bufferLen += n;                                         //Adjust the length of the buffer
               context->timestamp = osGetSystemTime();                          //Save current time
            }
         }
         else
         {
            error = ERROR_BUFFER_OVERFLOW;                                      //The client implementation limits the size of headers it accepts
         }
      }
      else if(context->requestState == HTTP_REQ_STATE_PARSE_HEADER)
      {
         context->bufferPos = 0;                                                //Rewind to the beginning of the buffer
         break;                                                                 //We are done
      }
      else
      {
         error = ERROR_WRONG_STATE;                                             //Invalid state
      }
   }

   if(error == ERROR_WOULD_BLOCK || error == ERROR_TIMEOUT)                     //Check status code
   {

      error = httpClientCheckTimeout(context);                                  //Check whether the timeout has elapsed
   }

   return error;                                                                //Return status code
}
/**
 * @brief Retrieve the HTTP status code of the response
 * @param[in] context Pointer to the HTTP client context
 * @return HTTP status code
 *
 * Example :-  status = httpClientGetStatus(&httpClientContext);
 **/
uint16_t httpClientGetStatus(HttpClientContext_t *context)
{
   uint16_t status;
   status = 0;                                                                  //Initialize status code

   if(context != NULL)                                                          //Make sure the HTTP client context is valid
   {
      //The status code is a three-digit integer code giving the result
      //of the attempt to understand and satisfy the request
      status = context->statusCode;
   }
   return status;                                                               //Return HTTP status code
}
/**
 * @brief Retrieve the value of the specified header field name
 * @param[in] context Pointer to the HTTP client context
 * @param[in] name NULL-terminated string that specifies the header field name
 * @return Value of the header field
 *
 * Example :- value = httpClientGetHeaderField(&httpClientContext, "Content-Type");
 **/
const char_t *httpClientGetHeaderField(HttpClientContext_t *context, const char_t *name)
{
   size_t i;
   size_t nameLen;
   size_t valueLen;
   const char_t *value;

   value = NULL;                                                                //Initialize field value

   if(context != NULL && name != NULL)                                          //Check parameters
   {
      //Check HTTP request state
      if(context->requestState == HTTP_REQ_STATE_PARSE_HEADER || context->requestState == HTTP_REQ_STATE_PARSE_TRAILER)
      {
         i = 0;                                                                 //Point to the first header field of the response

         while(i < context->bufferLen)                                          //Parse HTTP response header
         {
            nameLen = strlen((char*)context->buffer + i);                       //Calculate the length of the field name
            valueLen = strlen((char*)context->buffer + i + nameLen + 1u);       //Calculate the length of the field value
            if(!strcmp((char*)context->buffer + i,(char*)name))                 //Check whether the current header field matches the specified name
            {
               value = context->buffer + i + nameLen + 1u;                      //Retrieve the value of the header field
               break;                                                           //Exit immediately
            }
            i += nameLen + valueLen + 2;                                        //Point to next header field
         }
      }
   }
   return value;                                                                //Return the value of the header field
}
/**
 * @brief Read HTTP response body
 * @param[in] context Pointer to the HTTP client context
 * @param[out] data Buffer where to store the incoming data
 * @param[in] size Maximum number of bytes that can be received
 * @param[out] received Number of bytes that have been received
 * @param[in] flags Set of flags that influences the behavior of this function
 * @return Error code
 *
 * Example :- error = httpClientReadBody(&httpClientContext, buffer, sizeof(buffer) - 1, &length, 0);
 **/
error_t httpClientReadBody(HttpClientContext_t *context, void *dataV, size_t size, size_t *received, uint8_t flags)
{
   error_t error;
   size_t n=0u;
   char_t *p;

   if(context == NULL || dataV == NULL || received == NULL)                     // Check parameters
      return ERROR_INVALID_PARAMETER;

   if(context->state != HTTP_CLIENT_STATE_CONNECTED)                            //Check HTTP connection state
      return ERROR_WRONG_STATE;
   error = NO_ERROR;                                                            //Initialize status code
   p = dataV;                                                                   //Point to the output buffer
   *received = 0;                                                               //No data has been read yet

   while(*received < size && !error)                                            //Read as much data as possible
   {
      if(context->requestState == HTTP_REQ_STATE_PARSE_HEADER)                  //Check HTTP request state
      {
         context->bufferLen = 0;                                                //Flush receive buffer
         context->bufferPos = 0;

         //Any response to a HEAD request and any response with a 1xx, 204 or
         //304 status code is always terminated by the first empty line after
         //the header fields, regardless of the header fields present in the
         //message, and thus cannot contain a message body
         if(!strcmp(context->method1, "HEAD") || context->statusCode == 204 || context->statusCode == 304) // || HTTP_STATUS_CODE_1YZ(context->statusCode)
         {
            //The HTTP response does not contain a body
            httpClientChangeRequestState(context, HTTP_REQ_STATE_COMPLETE);
         }
         else
         {
            //The HTTP response contains a body
            httpClientChangeRequestState(context, HTTP_REQ_STATE_RECEIVE_BODY);
         }
      }
      else if(context->requestState == HTTP_REQ_STATE_RECEIVE_BODY)
      {
         if(context->chunkedEncoding)                                           //Chunked transfer encoding?
         {
            //The chunked encoding modifies the body in order to transfer it
            //as a series of chunks, each with its own size indicator
            httpClientChangeRequestState(context, HTTP_REQ_STATE_RECEIVE_CHUNK_SIZE);
         }
         else
         {
            //The length of the body is determined by the Content-Length field
            if(context->bodyPos < context->bodyLen)
            {
               n = min(size - *received, context->bodyLen - context->bodyPos);  //Limit the number of bytes to read

               //Read response body
               error = httpClientReceiveData(context, p, n, &n, flags);

               if(!error)                                                       //Check status code
               {
                  p += n;                                                       //Advance data pointer
                  *received += n;
                  context->bodyPos += n;

                  //Save current time
                  context->timestamp = osGetSystemTime();
                  break;                                                        //We are done
               }
               else
               {
                  //In practice, many widely deployed HTTPS servers close connections
                  //abruptly, without any prior notice, and in particular without
                  //sending the close_notify alert
                  if(!context->keepAlive && context->bodyLen == UINT16_MAX)
                  {
                     //The HTTP transaction is complete
                     httpClientChangeRequestState(context, HTTP_REQ_STATE_COMPLETE);
                     //Close the HTTP connection
                     httpClientChangeState(context, HTTP_CLIENT_STATE_DISCONNECTING);
                     //The end of the response body has been reached
                     error = ERROR_END_OF_STREAM;
                  }
               }
            }
            else
            {
               //The HTTP transaction is complete
               httpClientChangeRequestState(context, HTTP_REQ_STATE_COMPLETE);
               //The end of the response body has been reached
               error = ERROR_END_OF_STREAM;
            }
         }
      }
      else if(context->requestState == HTTP_REQ_STATE_RECEIVE_CHUNK_SIZE)
      {
         //The chunked encoding modifies the body in order to transfer it as
         //a series of chunks, each with its own size indicator
         if(context->bufferLen >= 3 && context->buffer[context->bufferLen - 2] == '\r' && context->buffer[context->bufferLen - 1] == '\n')
         {
            error = httpClientParseChunkSize(context, context->buffer, context->bufferLen);   //Parse chunk-size field
         }
         else if(context->bufferLen < HTTP_CLIENT_BUFFER_SIZE)
         {
            //Receive more data
            error = httpClientReceiveData(context, context->buffer + context->bufferLen, HTTP_CLIENT_BUFFER_SIZE - context->bufferLen, &n, HTTP_FLAG_BREAK_CRLF);
            if(!error)                                                          //Check status code
            {
               context->bufferLen += n;                                         //Adjust the length of the buffer
               context->timestamp = osGetSystemTime();                          //Save current time
            }
         }
         else
         {
            error = ERROR_BUFFER_OVERFLOW;                                      //The client implementation limits the length of the chunk-size field
         }
      }
      else if(context->requestState == HTTP_REQ_STATE_RECEIVE_CHUNK_DATA)
      {
         if(context->bodyPos < context->bodyLen)                                //The data stream is divided into a series of chunks
         {
            n = min(size - *received, context->bodyLen - context->bodyPos);     //The length of the data chunk is determined by the chunk-size field
            //Read chunk data
            error = httpClientReceiveData(context, p, n, &n, flags);

            if(!error)                                                          //Check status code
            {
               *received += n;                                                  //Total number of data that have been read
               context->bodyPos += n;                                           //Number of bytes left to process in the current chunk

               //Save current time
               context->timestamp = osGetSystemTime();

               if(flags & HTTP_FLAG_BREAK_CRLF)                                 //Check flags
               {
                  //The HTTP_FLAG_BREAK_CHAR flag causes the function to stop
                  //reading data as soon as the specified break character is
                  //encountered
                  if(p[n - 1] == (flags & 0x0Fu))
                     break;
               }
               else if(!(flags & HTTP_FLAG_WAIT_ALL))
               {
                  //The HTTP_FLAG_WAIT_ALL flag causes the function to return
                  //only when the requested number of bytes have been read
                  break;
               }
               else
               {
                  //Just for sanity
               }
               p += n;                                                          //Advance data pointer
            }
         }
         else
         {
            //The chunked encoding modifies the body in order to transfer it
            //as a series of chunks, each with its own size indicator
            httpClientChangeRequestState(context, HTTP_REQ_STATE_RECEIVE_CHUNK_SIZE);
         }
      }
      else if(context->requestState == HTTP_REQ_STATE_RECEIVE_TRAILER || context->requestState == HTTP_REQ_STATE_PARSE_TRAILER || context->requestState == HTTP_REQ_STATE_COMPLETE)
      {
         if(*received > 0)                                                      //The user must be satisfied with data already on hand
         {
            break;                                                              //Some data are pending in the receive buffer
         }
         else
         {
            error = ERROR_END_OF_STREAM;                                        //The end of the response body has been reached
         }
      }
      else
      {
         error = ERROR_WRONG_STATE;                                             //Invalid state
      }
   }

   if(error == ERROR_WOULD_BLOCK || error == ERROR_TIMEOUT)                     //Check status code
   {
      //Check whether the timeout has elapsed
      error = httpClientCheckTimeout(context);
   }

   //Return status code
   return error;
}
/**
 * @brief Read HTTP trailer
 * @param[in] context Pointer to the HTTP client context
 * @return Error code
 **/

error_t httpClientReadTrailer(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket)
{
   error_t error;
   size_t n;

   //Make sure the HTTP client context is valid
   if(context == NULL)
      return ERROR_INVALID_PARAMETER;

   //Initialize status code
   error = NO_ERROR;

   //Send HTTP request header
   while(!error)
   {
      //Check HTTP connection state
      if(context->state == HTTP_CLIENT_STATE_CONNECTED)
      {
         //Check HTTP request state
         if(context->requestState == HTTP_REQ_STATE_RECEIVE_TRAILER)
         {
            //The CRLF sequence is always used to terminate a line
            if(context->bufferLen >= (context->bufferPos + 2) &&
               context->buffer[context->bufferLen - 2] == '\r' &&
               context->buffer[context->bufferLen - 1] == '\n')
            {
               //Strip the CRLF at the end of the line
               context->bufferLen -= 2;

               //An empty line indicates the end of the header fields
               if(context->bufferLen == context->bufferPos)
               {
                  //The HTTP transaction is complete
                  httpClientChangeRequestState(context, HTTP_REQ_STATE_PARSE_TRAILER);
               }
               else
               {
                  //The response header fields allow the server to pass additional
                  //information about the response which cannot be placed in the
                  //Status-Line
                  error = httpClientParseHeaderField(context, context->buffer + context->bufferPos, context->bufferLen - context->bufferPos);
               }
            }
            else if(context->bufferLen < HTTP_CLIENT_BUFFER_SIZE)
            {
               //Receive more data
               error = httpClientReceiveData(context, context->buffer + context->bufferLen, HTTP_CLIENT_BUFFER_SIZE - context->bufferLen, &n, HTTP_FLAG_BREAK_CRLF);

               //Check status code
               if(!error)
               {
                  //Adjust the length of the buffer
                  context->bufferLen += n;
                  //Save current time
                  context->timestamp = osGetSystemTime();
               }
            }
            else
            {
               //The client implementation limits the size of headers it accepts
               error = ERROR_BUFFER_OVERFLOW;
            }
         }
         else if(context->requestState == HTTP_REQ_STATE_PARSE_TRAILER ||
            context->requestState == HTTP_REQ_STATE_COMPLETE)
         {
            //Rewind to the beginning of the buffer
            context->bufferPos = 0;

            //Persistent HTTP connection?
            if(context->keepAlive)
            {
               //Persistent connections stay open across transactions, until
               //either the client or the server decides to close them
               break;
            }
            else
            {
               //The connection will be closed after completion of the response
               httpClientChangeState(context, HTTP_CLIENT_STATE_DISCONNECTING);
            }
         }
         else
         {
            //Invalid HTTP request state
            error = ERROR_WRONG_STATE;
         }
      }
      else if(context->state == HTTP_CLIENT_STATE_DISCONNECTING)
      {
         //Shutdown connection
         error = httpClientDisconnect(context, socket);
      }
      else if(context->state == HTTP_CLIENT_STATE_DISCONNECTED)
      {
         //Rewind to the beginning of the buffer
         context->bufferPos = 0;
         //We are done
         break;
      }
      else
      {
         //Invalid HTTP connection state
         error = ERROR_WRONG_STATE;
      }
   }

   //Check status code
   if(error == ERROR_WOULD_BLOCK || error == ERROR_TIMEOUT)
   {
      //Check whether the timeout has elapsed
      error = httpClientCheckTimeout(context);
   }

   //Return status code
   return error;
}
/**
 * @brief Parse chunk-size field
 * @param[in] context Pointer to the HTTP client context
 * @param[in] line Pointer to the chunk-size field
 * @param[in] length Length of the chunk-size field
 * @return Error code
 **/
error_t httpClientParseChunkSize(HttpClientContext_t *context, char_t *line, size_t length)
{
   char_t *p;

   //Properly terminate the string with a NULL character
   line[length] = '\0';

   //Remove leading and trailing whitespace
   line = strTrimWhitespace(line);

   //The chunk-size field is a string of hex digits indicating the size
   //of the chunk
   context->bodyLen = strtoul(line, &p, 16);

   //Any parsing error?
   if(*p != '\0')
      return ERROR_INVALID_SYNTAX;

   //Flush receive buffer
   context->bufferLen = 0;
   context->bufferPos = 0;
   context->bodyPos = 0;

   //The chunked encoding is ended by any chunk whose size is zero
   if(context->bodyLen == 0)
   {
      //The last chunk is followed by an optional trailer
      httpClientChangeRequestState(context, HTTP_REQ_STATE_RECEIVE_TRAILER);
   }
   else
   {
      //Receive chunk data
      httpClientChangeRequestState(context, HTTP_REQ_STATE_RECEIVE_CHUNK_DATA);
   }

   //Successful processing
   return NO_ERROR;
}
/**
 * @brief Close HTTP request or response body
 * @param[in] context Pointer to the HTTP client context
 * @return Error code
 *
 * Example :-  error = httpClientCloseBody(&httpClientContext);
 **/
error_t httpClientCloseBody(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket)
{
   error_t error;
   size_t n;

   if(context == NULL)                                                          //Make sure the HTTP client context is valid
      return ERROR_INVALID_PARAMETER;

   error = NO_ERROR;                                                            //Initialize status code

   while(!error)                                                                //Close HTTP request or response body
   {
      if(context->state == HTTP_CLIENT_STATE_CONNECTED)                         //Check HTTP connection state
      {
         if(context->requestState == HTTP_REQ_STATE_SEND_BODY)                  //Check HTTP request state
         {
            if(context->chunkedEncoding)                                        //Chunked transfer encoding?
            {
               //The chunked encoding is ended by any chunk whose size is zero
               error = httpClientFormatChunkSize(context, 0);
            }
            else
            {
               if(context->bodyPos == context->bodyLen)                         //Ensure the HTTP request body is complete
               {
                  context->bufferLen = 0;                                       //Flush receive buffer
                  context->bufferPos = 0;
                  httpClientChangeRequestState(context, HTTP_REQ_STATE_RECEIVE_STATUS_LINE);  //Receive the HTTP response header
               }
               else
               {
                  error = ERROR_WRONG_STATE;                                    //Incomplete request body
               }
            }
         }
         else if(context->requestState == HTTP_REQ_STATE_SEND_CHUNK_DATA)
         {
            if(context->bodyPos == context->bodyLen)                             //Ensure the chunk data is complete
            {
               //The chunked encoding is ended by any chunk whose size is zero
               error = httpClientFormatChunkSize(context, 0);
            }
            else
            {
               error = ERROR_WRONG_STATE;                                       //Incomplete chunk data
            }
         }
         else if(context->requestState == HTTP_REQ_STATE_FORMAT_TRAILER || context->requestState == HTTP_REQ_STATE_RECEIVE_STATUS_LINE)
         {
            break;                                                              //The HTTP request body is closed
         }
         else if(context->requestState == HTTP_REQ_STATE_PARSE_HEADER || context->requestState == HTTP_REQ_STATE_RECEIVE_BODY || context->requestState == HTTP_REQ_STATE_RECEIVE_CHUNK_SIZE || context->requestState == HTTP_REQ_STATE_RECEIVE_CHUNK_DATA)
         {
            //Consume HTTP response body
            error = httpClientReadBody(context, context->buffer, HTTP_CLIENT_BUFFER_SIZE, &n, 0);

            if(error == ERROR_END_OF_STREAM)                                    //Check whether the end of the response body has been reached
            {
               error = NO_ERROR;                                                //Continue reading the optional trailer
            }
         }
         else if(context->requestState == HTTP_REQ_STATE_RECEIVE_TRAILER)
         {
            error = httpClientReadTrailer(context,socket);                      //Consume HTTP trailer
         }
         else if(context->requestState == HTTP_REQ_STATE_PARSE_TRAILER || context->requestState == HTTP_REQ_STATE_COMPLETE)
         {
            break;                                                              //The HTTP response body is closed
         }
         else
         {
            error = ERROR_WRONG_STATE;                                          //Invalid HTTP request state
         }
      }
      else if(context->state == HTTP_CLIENT_STATE_DISCONNECTING)
      {
         error = httpClientDisconnect(context, socket);                         //Shutdown connection
      }
      else if(context->state == HTTP_CLIENT_STATE_DISCONNECTED)
      {
         context->bufferPos = 0;                                                //Rewind to the beginning of the buffer
         break;                                                                 //We are done
      }
      else
      {
         error = ERROR_WRONG_STATE;                                             //Invalid HTTP connection state
      }
   }

   if(error == ERROR_WOULD_BLOCK || error == ERROR_TIMEOUT)                     //Check status code
   {
      error = httpClientCheckTimeout(context);                                  //Check whether the timeout has elapsed
   }

   return error;                                                                //Return status code
}
/**
 * @brief Set the HTTP protocol version to be used
 * @param[in] context Pointer to the HTTP client context
 * @param[in] version HTTP protocol version (1.0 or 1.1)
 * @return Error code
 *
 * Example :- error = httpClientSetVersion(&httpClientContext, HTTP_VERSION_1_1);
 **/
error_t httpClientSetVersion(HttpClientContext_t *context, HttpVersion version)
{

   if(context == NULL)                                                          //Make sure the HTTP client context is valid
      return ERROR_INVALID_PARAMETER;

   if(version != HTTP_VERSION_1_0 && version != HTTP_VERSION_1_1)               //Check HTTP version
      return ERROR_INVALID_VERSION;

   context->version = version;                                                  //Save the protocol version to be used

   return NO_ERROR;                                                             //Successful processing
}

#endif   /* end if http */
#if defined(JRT_LIDDAR_USED)
/*-----------------------------------------------------------------------------
 *      initUSB:  Initialise the USB port
 *
 *  Parameters: USBObject_t *USBObj
 *
 *  Return:     nothing
 *----------------------------------------------------------------------------*/
void initUSB( USBObject_t *USBObj )
{
  HID_Enable(USBObj->ReadBuffer, USBObj->WriteBuffer);
  Gen_Enable(USBObj->ReadBuffer, USBObj->WriteBuffer);
}
/*-----------------------------------------------------------------------------
 *      ReadUSB_JRT:  Reads JRT position sensor on USB port
 *
 *  Parameters: JRT_read_measure_t *distMeas, USBObject_t *USBObj, 
 *              JRT_write_measure_t *deviceState
 *
 *  Return:     nothing
 *----------------------------------------------------------------------------*/
void ReadUSB_JRT( JRT_read_measure_t *distMeas, USBObject_t *USBObj, JRT_write_measure_t *deviceState )
{

  if (((distMeas == NULL) && (USBObj == NULL)) && (deviceState == NULL))
  {
     return;
  }

  if (USBObj->Packet_In_Err >= JRT_MAX_ERR_RESET)                               /* number of corrupt packages exceeded */
  {
     initUSB( USBObj );                                                         /* re-enable USB */
     USBObj->Packet_In_Err=0u;                                                  /* reset the Packet error counter */
  }
  
  if ((USBObj->State==USB_PARTIAL_READ_DATA) || (USBObj->State=USB_PARTIAL_READ_CONFIG))     /* if we got partial read it again */
  {
    USB_Polling_Proc();                                                         /* call this inside the 100ms timed interrupt */
    USBObj->Bytes_In_Read2=Gen_Read(USBObj->ReadBufTmp,(JRT_MEAS_LEN-USBObj->Bytes_In),1);   /* read the bytes in up to max which is 16 providing call is non blocking if so consider the config len msg is different */
    memcpy((void*)USBObj->ReadBuffer+USBObj->Bytes_In,(void*)USBObj->ReadBufTmp,USBObj->Bytes_In_Read2); /* append the new bytes recieved at the port to the exisiting message */
    USBObj->Bytes_In=USBObj->Bytes_In+USBObj->Bytes_In_Read2;                   /* calculate the new message length by summing the two packet lengths */
  }
  else if (!(USBObj->State==USB_PACKET_READ_DATA) && !(USBObj->State=USB_PACKET_READ_CONFIG))  /* so long as we havent recieved a packet not handshaken by the caller */
  {
    USB_Polling_Proc();                                                         /* call this inside the 100ms timed interrupt */
    USBObj->Bytes_In=Gen_Read(USBObj->ReadBuffer,JRT_MEAS_LEN,1);               /* read the bytes in up to max JRT packet length which is 16 */
  }

  if ((USBObj->Bytes_In==JRT_MEAS_LEN) && (USBObj->ReadBuffer[3u] == JRT_READ_DATA_MEAS_PAY_LEN))  /* measurement frame found */
  {
    if ((USBObj->ReadBuffer[0u] == JRT_READ_STARTSTOP_MEASURE_STX) && (USBObj->ReadBuffer[1u] == JRT_READ_STARTSTOP_MEASURE_MSG))
    {
       memcpy((void*)distMeas,(void*)USBObj->ReadBuffer,JRT_MEAS_LEN);
       USBObj->State=USB_PACKET_READ_DATA;
    }
    else
    {
       USBObj->State=USB_DATA_CORRUPT;
    }
  }
  else if (((USBObj->Bytes_In<JRT_MEAS_LEN) && (USBObj->ReadBuffer[3u] == JRT_READ_DATA_MEAS_PAY_LEN)) && (USBObj->Bytes_In>=3u))   /* partial measurement frame found */
  {
    if ((USBObj->ReadBuffer[0u] == JRT_READ_STARTSTOP_MEASURE_STX) && (USBObj->ReadBuffer[1u] == JRT_READ_STARTSTOP_MEASURE_MSG))
    {
       USBObj->State=USB_PARTIAL_READ_DATA;
    }
    else
    {
       USBObj->State=USB_NO_DATA_READ;
    }
  }
  else if ((USBObj->Bytes_In==JRT_STARTSTOP_LEN) && (USBObj->ReadBuffer[3u] == JRT_READ_STARTSTOP_PAYLOAD_LEN))   /* start/stop frame found */
  {
    if ((USBObj->ReadBuffer[0u] == JRT_READ_STARTSTOP_MEASURE_STX) && (USBObj->ReadBuffer[1u] == JRT_READ_STARTSTOP_MEASURE_MSG))
    {
       memcpy((void*)deviceState,(void*)USBObj->ReadBuffer,JRT_STARTSTOP_LEN);
       USBObj->State=USB_PACKET_READ_CONFIG;
    }
    else
    {
      USBObj->State=USB_DATA_CORRUPT;
    }
  }
  else if (((USBObj->Bytes_In<JRT_STARTSTOP_LEN) && (USBObj->ReadBuffer[3u] == JRT_READ_STARTSTOP_PAYLOAD_LEN)) && (USBObj->Bytes_In>=3u))  /* partial start/stop frame found */
  {
    if ((USBObj->ReadBuffer[0u] == JRT_READ_STARTSTOP_MEASURE_STX) && (USBObj->ReadBuffer[1u] == JRT_READ_STARTSTOP_MEASURE_MSG))
    {
       USBObj->State=USB_PARTIAL_READ_CONFIG;
    }
    else
    {
       USBObj->State=USB_NO_DATA_READ;
    }
  }
  else
  {
    if ((USBObj->Bytes_In<=2u) && (USBObj->Bytes_In!=0u))                       /* very small 1 or 2 BYTE message received */
    {
       USBObj->State=USB_PARTIAL_READ_DATA;
    }
    else
    {
       USBObj->State=USB_DATA_CORRUPT;
    }
  }

  if ((USBObj->State==USB_DATA_CORRUPT) && ((USBObj->prevState==USB_DATA_CORRUPT) || (USBObj->prevState==USB_PARTIAL_READ_DATA)))   /* count consecutively corrupt packages */
  {
     USBObj->Packet_In_Err=++USBObj->Packet_In_Err % UINT16_MAX;                /* count consecutive bad packets */
  }
  if (USBObj->Packet_In_Err >= JRT_MAX_ERR_RESET)                               /* number of corrupt packages exceeded limit for reset */
  {
     HID_Disable();                                                             /* disbale the port to reset it, it gets re-enabled next time round */
  }
  USBObj->prevState=USBObj->State;
}
#endif /* end JRT distance sensor on USB */
#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
/*-----------------------------------------------------------------------------
 *      readManualADC:  Read a ADC using manual sampling method
 *
 *  Parameters:  uint8_t pinNo, uint16_t *rawValADC
 *
 *
 *  Return: ADCStates_e the current state of the operation
 *----------------------------------------------------------------------------*/
ADCStates_e readManualADC( uint8_t pinNo, uint16_t *rawValADC )
{
   switch(g_adcState)
   {
      case ENAB_ADC_MAN_SAMP:
      AD1CON1=((IO_AD_ENABLE | (AD1CON1 & IO_FORM_MSAM)) | IO_FORM_I32);        /* enable ADC set it to manual sample form i32 */
      g_adcState=CHOOSE_ADC_PIN;
      break;

      case CHOOSE_ADC_PIN:                                                      /* start sampling */
      IO_CH0SA_PIN(pinNo);                                                      /* When MUX A is selected (the default), this input is the positive input to the differencing amplifier. The value of this field determines which ANx pin is used */
      AD1CON1 = AD1CON1 | IO_FORM_SAMP;                                         /* set manual sampling to on  */
      g_adcState=WAIT_ADC_SAM_TIM;
      break;

      case WAIT_ADC_SAM_TIM:                                                    /* wait at least the sample time done inside tick compare interrupt  */
      /* time delay */
      break;

      case WAIT_FOR_SAMPLE:
      AD1CON1 = AD1CON1 & IO_FORM_NO_SAMP;                                      /* set manual sampling to off */
      if (AD1CON1 & IO_AD_STATE)                                                /* sampling complete or timeout state is advanced inside tick interrupt */
      {
        g_adcState=COLLECT_ADC_RAW;
      }
      break;

      case COLLECT_ADC_RAW:                                                     /* collect the ADC */
      *rawValADC=ADC1BUF0;
      g_adcState=ADC_COLLECT_COMPLETE;
      break;

      case COLLECT_ADC_TIMEOUT:                                                 /* collect the ADC */
      *rawValADC=0u;
      g_adcState=ADC_COLLECT_COMPLETE;
      break;

      case ADC_COLLECT_COMPLETE:                                                /* state of analogue is collected global state is reset by calling level (back to 1) */
      break;

      case ENAB_ADC_AUT_SAMP:
      AD1CON1=IO_AD_ENABLE | IO_FORM_ASAM | IO_FORM_I32;                        /* turn on auto sampling */
      g_adcState=MAN_SAMPLING_OFF;
      break;

      case MAN_SAMPLING_OFF:
      break;

      default:
      g_adcState=ENAB_ADC_MAN_SAMP;                                             /* re-initialise */
      break;
   }
   return g_adcState;
}
#endif
/* De-bounce button: it should keep its state for a given period of time (300ms)
   Returns 1 if btn.trigger_state is changed.                                 */
/*-----------------------------------------------------------------------------
 *      debounce_button:  De-bounce button: it should keep
 *                        its state for a given period of time (300ms)
 *  Parameters:  SBGC_btn_state_t *btn, uint8_t new_state , uint64_t TimeThreshold
 *
 *
 *  Return: nothing value put into timeduration variable
 *----------------------------------------------------------------------------*/
uint8_t debounce_button( IO_btn_state_t *btn, uint8_t new_state, uint64_t TimeThreshold )
{

   uint64_t cur_time_ms = g_timer5_counter;                                     // Take the global tick counter and store (interrupt timer 5)
   uint64_t diff_time=0ULL;

    if ((cur_time_ms - btn->last_time_ms) < 0u)                                 // We rolled over
    {
       diff_time = (cur_time_ms - btn->last_time_ms) + UINT_LEAST64_MAX;        // Correct the diff by adding Uint_MAX
    }
    else
    {
       diff_time = (cur_time_ms - btn->last_time_ms);
    }

    if(new_state != btn->state)                                                 // The I/O State is different from last known
    {
        btn->state = new_state;                                                 // Set the new button state
        btn->last_time_ms = cur_time_ms;                                        // Remeber last tick counter
    }
    else if ((btn->trigger_state != btn->state) && (diff_time > BTN_BOUNCE_THRESHOLD_MS))
    {
         btn->trigger_state = btn->state;                                       // We persisted in this state for more than 30 ticks set the trigger
         return 1u;
    }
    return 0u;
}
/*-----------------------------------------------------------------------------
 *      mov_avg():  moving average for real data
 *
 *  Parameters: COMGS_gyro_acc_data_t *accStruct
 *  Return:     uint8_t = 1 for complete
 *----------------------------------------------------------------------------*/
uint8_t mov_avg( mov_avg_data_t *accStruct )
{
  uint8_t retCode=0u;                                                           // initialise to no sample state

  if (accStruct->g_count_sam!=0x40u)                                            // iterate this function 64 times before writing result and reseting  (moving average filter)
  {
    accStruct->mavg_sum=accStruct->mavg_sum + accStruct->value_in;
    accStruct->g_count_sam=++accStruct->g_count_sam % UINT8_MAX;                // count number of samples
  }
  else
  {
    accStruct->mavg_sum = ( ((uint32_t) accStruct->mavg_sum) >> 6u);            // divide by 64 the sum
    accStruct->mavg_last = accStruct->mavg_value;                               // copy current moving average to actual moving average values stored as 1 and 2
    accStruct->mavg_value = accStruct->mavg_sum;
    accStruct->g_count_sam=0u;                                                  // reset and start collecting another 64 samples
    accStruct->g_count_acc=++accStruct->g_count_acc % UINT8_MAX;                // Rolling Counter (msg id)
        retCode=1u;
  }

  return retCode;

}
#if defined(BATCH_MIXER)
/*******************************************************************************
* Function Name: makeUpDoseTank
********************************************************************************
* Summary:
*  makeUpDoseTank function performs following functions:
*   1. makes up a batch using 2 liquids and solids
*   2. allows movement of a sprayer arm retracted middle or extended
* Parameters:
*  uint16_t *valADC array containing the position of the level instrument
*  storedRealTimeData_t *rcpDat recipe data for the batch related to product
*                               you are making from the saved mmc file
* Return:
*  None.
*
*******************************************************************************/
#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
void makeUpDoseTank( uint16_t *valADC, batchSeq1Recipe_t *rcpDat, mov_avg_data_t *movAvg  )
#else
void makeUpDoseTank( batchSeq1Recipe_t *rcpDat, mov_avg_data_t *movAvg  )
#endif
{

#if defined(EXTENDED_ARM)
   extActualState=((extArmRevStop<<2U) | (extArmEndStop<<1u) | extArmMidStop);  /* monitor the current state from the limit switches */
   debounce_button(&armActState, extActualState, EXT_ARM_BOUNCE_THRESHOLD);     /* debounce the digital input states to ensure we dont get a wrong value due to vibration */

   if ((extArmState!=extArmStateLast) && (startedSpray==true))                  /* continously monitored exception once we have started up */
   {
      /*  batchStateLast=g_batchState;                                                 save the last state of the sequence as you will return to it when exception has completed */
      if ((extArmState==EXT_ARM_FULL_POS) || ((extArmState==EXT_ARM_MID_POS) && (armActState.trigger_state=EXT_ARM_RET_POS)))
      {
         extArmStep=ARM_MTR_FORWARD;                                            /* go to extend drive hydraulic motor forward */
      }
      else
      {
         extArmStep=ARM_MTR_BACKWD;                                             /* go to retract drive hydraulic motor backward */
      }
      extArmStateLast=extArmState;                                              /* unlatch the trigger store the last state (one-shot) */
   }
#endif

   switch (g_batchState)                                                        /* for each step of the spraying and batch make-up sequence */
   {
      case BATCH_TK_WAIT_EMPTY:
      CLR_PRIO_IO1_INTER(5u,1u);                                                /* inhibit the tank 1 interrupts (here in case of ability to jump step from GUI */
      CLR_PRIO_IO2_INTER(5u,2u);
      CLR_PRIO_IO3_INTER(5u,1u);                                                /* inhibit the tank 2 interrupts */
      CLR_PRIO_IO4_INTER(5u,2u);
#if (AIN_ADC_METHOD == ADC_AUTO_COLLECT)
      movAvg->value_in=ADC1_Read(LevelInChan);                                  /* read the level input measured */
#elif (AIN_ADC_METHOD == ADC_MAN_COLLECT)
#if defined(SBGC_GIMBAL_JOY)
      movAvg->value_in==*valADC+3u;                                             /* level reading starts at input channel 3 */
#else
      movAvg->value_in==*valADC;                                                /* level reading starts at input channel 1 */
#endif /* end gimbal included as well */
#endif /* ADC method branch */
      if (mov_avg( movAvg )==1u)                                                /* new sample complete from moving average algorythm */
      {
         LevelInRaw=movAvg->mavg_value;                                         /* get the moving avaerage value */
         LevelInput=Scale_Raw_Value(0u,RAW_MAX,BATCH_LEVL_MIN,BATCH_LEVL_MAX,LevelInRaw);  /* scale the instrument */
         if (LevelInput <= rcpDat->LevelEmpty)                                  /* tank needs topping up */
         {
            g_batchState = BATCH_TK_OPN_VLV1;                                   /* start the batch make-up */
         }
         else if (rcpDat->timeOverride == true)                                 /* level instrument not present or fault do on time */
         {
            calculateTime2Now(&elapsedTime, &lastBatchTime);                      /* get the time from the global interrupt and store the previous timer value */
            if (elapsedTime > rcpDat->dosingTime)                               /* more then the time to dose */
            {
               g_batchState = BATCH_TK_OPN_VLV1;                                /* start the batch make-up process */
            }
         }
         else
         {
#if defined(EXTENDED_ARM)
            if (startedSpray==false)                                            /* first time sequence ran */
            {
               startedSpray=true;                                               /* enable the ended arm controls as we have enough level */
               extArmStateLast=extArmState;                                     /* remember the current requested state only do exception to change arm if we asked to change it */
            }
#endif
         /* g_batchState = 11u;   go to the dosing step start the sprayer by opening pneumatic valve (allow dosing from here HMI commands spray valve */
         }
      }
      DeviceState = (DeviceState ^ ((Valve5Open & !(g_hmiReqSprayOn==prev_ReqSprayOn)) & !g_hmiResetESD1));    /* toggle the sprayer state from HMI */
      if (g_hmiResetESD1==false)                                                /* no esd present */
         prev_ReqSprayOn=g_hmiReqSprayOn;                                       /* save the HMI pneumatic valve request */
      break;

      case BATCH_TK_OPN_VLV1:
      CLR_PRIO_IO3_INTER(5u,1u);                                                /* inhibit the tank 2 interrupts */
      CLR_PRIO_IO4_INTER(5u,2u);
      SET_PRIO_IO1_INTER(5u,1u);                                                /* set falling edge interrupt on INT1 for low supply tank on batching plant interrupt */
      if (doseDuringMakeUp==1u)                                                 /* set via HMI whether you dose while making up or stop to batch (most acurate) */
      {
        DeviceState = DeviceState | Valve1Open;                                 /* make up with sprayer valve remaining open */
      }
      else
      {
        /* startedSpray=false;                                                      disable the ended arm controls */
        DeviceState = (DeviceState & ~(Valve5Open)) | Valve1Open;               /* make up with sprayer valve closed */
      }
      elapsedTime=-1;                                                           /* initialise timer */
      calculateTime2Now(&elapsedTime, &lastBatchTime);                            /* get the time from the global interrupt and store the previous timer value */
      g_batchState = BATCH_WAIT_LVL1;                                           /* go to watching tank fill step */

      case BATCH_WAIT_LVL1:
#if (AIN_ADC_METHOD == ADC_AUTO_COLLECT)
      movAvg->value_in=ADC1_Read(LevelInChan);                                        /* read the level input measured */
#elif (AIN_ADC_METHOD == ADC_MAN_COLLECT)
#if defined(SBGC_GIMBAL_JOY)
      movAvg->value_in=*valADC+3u;                                                    /* level reading starts at input channel 3 */
#else
      movAvg->value_in=*valADC;                                                       /* level reading starts at input channel 1 */
#endif /* end gimbal included as well */
#endif /* ADC method branch */
      if (mov_avg( movAvg )==1u)                                                /* new sample complete from moving average algorythm */
      {
         LevelInRaw=movAvg->mavg_value;                                         /* get the moving avaerage value */
         LevelInput=Scale_Raw_Value(0u,RAW_MAX,BATCH_LEVL_MIN,BATCH_LEVL_MAX,LevelInRaw);  /* scale the instrument */
         calculateTime2Now(&elapsedTime, &lastBatchTime);                         /* get the time from the global interrupt and store the previous timer value */
         if ((LevelInput >= rcpDat->level1SptCorse) || (elapsedTime > (rcpDat->corse1Time+rcpDat->errorTime)))  /* level has reached first corse setting */
         {
            g_batchState = BATCH_TK_OPN_VLV2;                                   /* go dose fine (slowly) */
         }
         else if (rcpDat->timeOverride == true)                                 /* GUI set sequence to operate on time not level */
         {
            if (elapsedTime > rcpDat->corse1Time)                               /* we have waited for the max time to dose corse fluid 1 */
            {
               g_batchState = BATCH_TK_OPN_VLV2;                                /* go dose fine (slowly) */
            }
         }
      }
       
      case BATCH_TK_OPN_VLV2:
      DeviceState = (DeviceState & (~Valve1Open)) | Valve2Open;                 /* close the corse valve and open the fine valve */
      elapsedTime=-1;                                                           /* initialise timer */
      calculateTime2Now(&elapsedTime, &lastBatchTime);
      g_batchState = BATCH_WAIT_LVL2;                                           /* go to waiting for the fine level setpoint */

      case BATCH_WAIT_LVL2:
#if (AIN_ADC_METHOD == ADC_AUTO_COLLECT)
      movAvg->value_in=ADC1_Read(LevelInChan);                                  /* read the level input measured */
#elif (AIN_ADC_METHOD == ADC_MAN_COLLECT)
#if defined(SBGC_GIMBAL_JOY)
      movAvg->value_in=*valADC+3u;                                              /* level reading starts at input channel 3 */
#else
      movAvg->value_in=*valADC;                                                 /* level reading starts at input channel 1 */
#endif /* end gimbal included as well */
#endif /* ADC method branch */
      if (mov_avg( movAvg )==1u)                                                /* new sample complete from moving average algorythm */
      {
         LevelInRaw=movAvg->mavg_value;                                         /* get the moving avaerage value */
         LevelInput=Scale_Raw_Value(0u,RAW_MAX,BATCH_LEVL_MIN,BATCH_LEVL_MAX,LevelInRaw);  /* scale the instrument */
         calculateTime2Now(&elapsedTime, &lastBatchTime);                         /* get interrupt timer */
         if ((LevelInput >= rcpDat->level1SptFine) || (elapsedTime > (rcpDat->fine1Time+rcpDat->errorTime)))  /* level input greater than fine level setpoint */
         {
            if (rcpDat->solidMakeUp==1u)                                          /* make up liquid 1 with solids */
            {
               g_batchState = BATCH_ST_VIB;                                        /* go to second addition stage solid */
            }
            else
            {
               g_batchState = BATCH_TK_OPN_VLV3;                                   /* go to second addition stage liquid */
            }
         }
         else if (rcpDat->timeOverride == true)                                    /* time override is on */
         {
            if (elapsedTime > rcpDat->fine1Time)                                   /* time expired */
            {
               if (rcpDat->solidMakeUp==1u)                                        /* make up liquid 1 with solids */
               {
                  g_batchState = BATCH_ST_VIB;                                     /* advance to second addition phase solid */
               }
               else
               {
                  g_batchState = BATCH_TK_OPN_VLV3;                                 /* advance to second addition phase liquid */
               }
            }
         }
      }
       
      case BATCH_TK_OPN_VLV3:
      CLR_PRIO_IO1_INTER(5u,1u);                                                /* inhibit the tank 1 interrupts */
      CLR_PRIO_IO2_INTER(5u,2u);
      SET_PRIO_IO3_INTER(5u,1u);                                                /* set falling edge interrupt on INT1 for low supply tank on batching plant interrupt */
      DeviceState = (DeviceState & (~Valve2Open)) | Valve3Open;                 /* open corse valve */
      elapsedTime=-1;                                                           /* init timer */
      calculateTime2Now(&elapsedTime, &lastBatchTime);                            /* get timer */
      g_batchState = BATCH_WAIT_LVL3;                                           /* go to waiting for level step */

      case BATCH_WAIT_LVL3:
#if (AIN_ADC_METHOD == ADC_AUTO_COLLECT)
      movAvg->value_in=ADC1_Read(LevelInChan);                                  /* read the level input measured */
#elif (AIN_ADC_METHOD == ADC_MAN_COLLECT)
#if defined(SBGC_GIMBAL_JOY)
      movAvg->value_in=*valADC+3u;                                              /* level reading starts at input channel 3 */
#else
      movAvg->value_in=*valADC;                                                 /* level reading starts at input channel 1 */
#endif /* end gimbal included as well */
#endif /* ADC method branch */
      if (mov_avg( movAvg )==1u)                                                /* new sample complete from moving average algorythm */
      {
         LevelInRaw=movAvg->mavg_value;                                         /* get the moving avaerage value */
         LevelInput=Scale_Raw_Value(0u,RAW_MAX,BATCH_LEVL_MIN,BATCH_LEVL_MAX,LevelInRaw);  /* scale the instrument */
         if ((LevelInput >= rcpDat->level2SptCorse) || (elapsedTime > (rcpDat->corse2Time+rcpDat->errorTime))) /* level exceeds second additon corse setpoint */
         {
            g_batchState = BATCH_TK_OPN_VLV4;                                   /* close corse valve and open fine valve */
         }
         else if (rcpDat->timeOverride == true)                                 /* timed override */
         {
            calculateTime2Now(&elapsedTime, &lastBatchTime);
            if (elapsedTime > rcpDat->corse2Time)                               /* we have waited long enough time */
            {
               g_batchState = BATCH_TK_OPN_VLV4;                                /* close corse valve and open fine valve */
            }
         }
      }

      case BATCH_TK_OPN_VLV4:
      DeviceState = (DeviceState & (~Valve3Open)) | Valve4Open;                 /* close corse valve and open fine valve */
      elapsedTime=-1;                                                           /* init timer */
      calculateTime2Now(&elapsedTime, &lastBatchTime);
      g_batchState = BATCH_WAIT_LVL4;                                           /* wait for the level to reach setting 4 */

      case BATCH_WAIT_LVL4:
#if (AIN_ADC_METHOD == ADC_AUTO_COLLECT)
      movAvg->value_in=ADC1_Read(LevelInChan);                                  /* read the level input measured */
#elif (AIN_ADC_METHOD == ADC_MAN_COLLECT)
#if defined(SBGC_GIMBAL_JOY)
      movAvg->value_in=*valADC+3u;                                              /* level reading starts at input channel 3 */
#else
      movAvg->value_in=*valADC;                                                 /* level reading starts at input channel 1 */
#endif /* end gimbal included as well */
#endif /* ADC method branch */
      if (mov_avg( movAvg )==1u)                                                /* new sample complete from moving average algorythm */
      {
         LevelInRaw=movAvg->mavg_value;                                         /* get the moving avaerage value */
         LevelInput=Scale_Raw_Value(0u,RAW_MAX,BATCH_LEVL_MIN,BATCH_LEVL_MAX,LevelInRaw);  /* scale the instrument */
         if ((LevelInput >= rcpDat->level2SptFine) || (elapsedTime > (rcpDat->fine2Time+rcpDat->errorTime))) /* level is above second addition fine level setting */
         {
            g_batchState = BATCH_ST_MIX;                                        /* go to start the mixer or add solids if extra step set true */
         }
         else if (rcpDat->timeOverride == true)                                 /* timed override is on */
         {
            calculateTime2Now(&elapsedTime, &lastBatchTime);
            if (elapsedTime > rcpDat->fine2Time)                                /* fine time for liquid 2 expired */
            {
               g_batchState = BATCH_ST_MIX;                                     /* go to the close the fine valve step */
            }
         }
      }
      
      case BATCH_ST_MIX:
      CLR_PRIO_IO1_INTER(5u,1u);                                                /* inhibit the tank 1 interrupts */
      CLR_PRIO_IO2_INTER(5u,2u);
      CLR_PRIO_IO3_INTER(5u,1u);                                                /* inhibit the tank 2 interrupts */
      CLR_PRIO_IO4_INTER(5u,2u);
      if (rcpDat->xTraStep==true)                                               /* extra solid addition step requested in the product recipe */
      {
         DeviceState = (DeviceState & (~Valve4Open));                           /* close the valve */
         g_batchState = BATCH_ST_VIB;                                           /* add solids */
      }
      else
      {
         DeviceState = (DeviceState & (~Valve4Open)) | Mixer1Run;               /* start the mixer and close the valve */
         elapsedTime=-1;                                                        /* initialise timer */
         calculateTime2Now(&elapsedTime, &lastBatchTime);
         g_batchState = BATCH_WT_MIX;                                           /* go to mixing */
      }
      break;

      case BATCH_WT_MIX:
      calculateTime2Now(&elapsedTime, &lastBatchTime);
      if (elapsedTime > rcpDat->mixTime)                                        /* mixed for long enough */
      {
#if defined(EXTENDED_ARM)
         if (startedSpray==false)                                               /* first time sequence ran */
         {
            extArmStep = ARM_MTR_FORWARD;                                       /* extend the arm */
            g_batchState = BATCH_EXT_ARM;
         }
         else
         {
            g_batchState = BATCH_END_MIX;                                       /* stop mixing and start spraying */
         }
#else
         g_batchState = BATCH_END_MIX;                                          /* stop mixing and start spraying */
#endif      
      }

      case BATCH_END_MIX:
      DeviceState = ((DeviceState & (~Mixer1Run)) | (Valve5Open & !g_hmiResetESD1));  /* stop mixer open dosing / sprayer valve */
#if defined(EXTENDED_ARM)
      if (startedSpray==false)                                                  /* first time sequence ran */
      {
         startedSpray=true;                                                     /* enable the ended arm controls */
         extArmStateLast=extArmState;                                           /* remember the current requested state only do exception to change arm if we asked to change it */
      }
#endif
      elapsedTime=-1;                                                           /* initialise timer */
      calculateTime2Now(&elapsedTime, &lastBatchTime);
      if (g_hmiResetESD1==false)                                                /* no trip condition */
      {
         g_batchState = BATCH_TK_WAIT_EMPTY;                                    /* go to wait until we empty while spraying */
      }
      break;

      case BATCH_ST_VIB:
      DeviceState = (DeviceState & (~Valve3Open)) | Vib1Start;                  /* start the hopper vibrator */
      elapsedTime=-1;                                                           /* initialise timer */
      calculateTime2Now(&elapsedTime, &lastBatchTime);
      g_batchState = BATCH_WAIT_SOLID;                                          /* wait for pneumatic conveying or vibration time */

      case BATCH_WAIT_SOLID:
      calculateTime2Now(&elapsedTime, &lastBatchTime);
      if (elapsedTime > rcpDat->solid1Time)                                     /* waited long enough for correct ammount of solids */
      {
         g_batchState = BATCH_END_SOLID;                                        /* stop vibrator on hopper and/or pneumatic conveyor */
      }

      case BATCH_END_SOLID:
      DeviceState = (DeviceState & (~Vib1Start)) | Mixer1Run;                   /* stop vibrator and/or conveyancing and start the mixer */
      elapsedTime=-1;                                                           /* initialise timer */
      calculateTime2Now(&elapsedTime, &lastBatchTime);
      g_batchState = BATCH_WT_MIX;                                              /* wait for the mixing time */
      break;

      case BATCH_EXT_ARM:                                                       /* waiting for an extended arm completion flag */
      DeviceState = (DeviceState & (~Mixer1Run));                               /* stop the mixer */
      if (extArmStep==ARM_SPR_ACT_MTR_IDLE)                                     /* completed the extended arm */
        g_batchState=BATCH_END_MIX;
      break;

      case BATCH_INIT_RESET:                                                    /* initial reset state */
      elapsedTime=-1;                                                           /* initialise timer first time */
      calculateTime2Now(&elapsedTime, &lastBatchTime);                            /* get the time from the global interrupt and store the previous timer value */
      DeviceState=0u;                                                           /* close all valves and stop dosing/spraying */
#if defined(EXTENDED_ARM)
      /* AuxDevState default setting ?? we leave extended arm as is here for now */
#endif
      g_batchState = BATCH_TK_WAIT_EMPTY;                                       /* wait for the holding tank to empty */
      break;

      case BATCH_HOLD_STATE:                                                    /* hold state set from interrupt */
      break;

      case BATCH_HOLD_RETURN:                                                   /* reset after hold state controlled by interrupt */
      break;

      default:
      elapsedTime=-1;                                                           /* initialise the timer to take a snapshot of the global counter */
      calculateTime2Now(&elapsedTime, &lastBatchTime);                            /* start timer at zero */
      g_batchState = BATCH_TK_WAIT_EMPTY;                                       /* wait for the holding tank to empty */
      break;

  }
#if defined(EXTENDED_ARM)
  switch(extArmStep)
  {
      case ARM_MTR_FORWARD:                                                     /* extend the long arm on the sprayer */
      AuxDevState = ((AuxDevState & (~extArmRev)) | (extArmFwd & !g_hmiResetESD1)); /* drive motor fwd extend the sprayer arm */
      if (g_hmiResetESD1==false)                                                /* no esd interrupt has been present and not reset from the HMI */
      {
         switch (extArmState)                                                   /* depends on input switches or selection */
         {
            case EXT_ARM_MID_POS:
            elapsedTimeArm=-1;                                                  /* initialise the timer to take a snapshot of the global counter */
            calculateTime2Now(&elapsedTimeArm, &lastBatchTimeArm);                /* start timer at zero */
            extArmStep = ARM_FWD_TO_MID;                                        /* arm to go to middle */
            break;

            case EXT_ARM_FULL_POS:
            elapsedTimeArm=-1;                                                  /* initialise the timer */
            calculateTime2Now(&elapsedTimeArm, &lastBatchTimeArm);                /* start timer at zero */
            extArmStep = ARM_FWD_TO_END;                                        /* arm to go fully out */
            break;

            case EXT_ARM_RET_POS:
            extArmStep = ARM_MTR_BACKWD;                                        /* just start spraying without arm */
            break;

            default:                                                            /* selection error */
            /* extArmStep = 31u;    */
            break;
         }
      }

      case ARM_FWD_TO_END:
      calculateTime2Now(&elapsedTimeArm, &lastBatchTimeArm);                      /* calculate the time delay from initialisation to now */
      if ((extArmEndStop) || (elapsedTimeArm >= sprayEndStopMaxtime))           /* reaches extended stop or time */
      {
          AuxDevState = (AuxDevState & (~extArmRev)) & (~extArmFwd);            /* stop the sprayer arm going forward*/
          extArmStep = ARM_SPR_ACT_MTR_IDLE;                                    /* start sparaying */
      }
      else if (extArmRevStop && (elapsedTimeArm >= ARM_START_DLY))              /* reached the wrong end after start-up */
      {
          extArmStep = ARM_MTR_FORWARD;                                         /* drive the sprayer head in the forward direction */
      }
      break;

      case ARM_FWD_TO_MID:
      calculateTime2Now(&elapsedTimeArm, &lastBatchTimeArm);                      /* calculate the time delay from initialisation to now */
      if ((extArmMidStop) || (elapsedTimeArm >= sprayMidStopMaxtime))           /* reaches mid stop or time */
      {
          AuxDevState = (AuxDevState & (~extArmRev)) & (~extArmFwd);            /* stop the sprayer arm going forward or backward */
          extArmStep = ARM_SPR_ACT_MTR_IDLE;                                    /* start sparaying */
      }
      else if (extArmEndStop)                                                   /* reached the wrong end */
      {
          extArmStep = ARM_MTR_BACKWD;                                          /* reverse the motor */
      }
      else if (extArmRevStop && (elapsedTimeArm >= ARM_START_DLY))              /* reached the wrong end after start-up */
      {
          extArmStep = ARM_MTR_FORWARD;
      }
      break;

      case ARM_MTR_BACKWD:                                                      /* retract the long arm on the sprayer */
      AuxDevState = ((AuxDevState & (~extArmFwd)) | (extArmRev & !g_hmiResetESD1)); /* drive motor backwards and retract the sprayer arm */
      if (g_hmiResetESD1==false)
      {
         switch (extArmState)                                                   /* depends on input switches or selection */
         {
            case EXT_ARM_MID_POS:                                               /* arm to mid position */
            elapsedTimeArm=-1;                                                  /* initialise the timer */
            calculateTime2Now(&elapsedTimeArm, &lastBatchTimeArm);                /* start timer at zero */
            extArmStep = ARM_REV_TO_MID;                                        /* arm to go to middle */
            break;

            case EXT_ARM_FULL_POS:
            /* batchState = batchStateLast;                                         arm to go fully out */
            extArmStep = ARM_MTR_FORWARD;
            break;

            case EXT_ARM_RET_POS:
            elapsedTimeArm=-1;                                                  /* initialise the timer */
            calculateTime2Now(&elapsedTimeArm, &lastBatchTimeArm);                /* start timer at zero */
            extArmStep = ARM_REV_TO_RETR;                                       /* just start spraying without arm */
            break;

            default:
            /* batchState = 35u;  */
            break;
         }
      }

      case ARM_REV_TO_MID:
      calculateTime2Now(&elapsedTimeArm, &lastBatchTimeArm);
      if ((extArmMidStop) || (elapsedTimeArm >= sprayMidStopMaxtime))           /* reaches mid stop or time */
      {
          AuxDevState = (AuxDevState & (~extArmRev)) & (~extArmFwd);            /* stop the sprayer arm going backwards */
          extArmStep = ARM_SPR_ACT_MTR_IDLE;                                    /* start sparaying */
          /* batchState = batchStateLast;                                           go back to where you where in the sequence */
      }
      else if (extArmRevStop)                                                   /* reached the wrong end */
      {
          extArmStep = ARM_MTR_FORWARD;
      }
      else if (extArmEndStop && (elapsedTimeArm >= ARM_START_DLY))              /* reached the wrong end after start-up */
      {
          extArmStep = ARM_MTR_BACKWD;
      }
      break;

      case ARM_REV_TO_RETR:
      calculateTime2Now(&elapsedTimeArm, &lastBatchTimeArm);
      if ((extArmRevStop) || (elapsedTimeArm >= sprayRevStopMaxtime))           /* reaches retracted stop or time */
      {
          AuxDevState = (AuxDevState & (~extArmRev)) & (~extArmFwd);            /* stop the sprayer arm reversing */
          DeviceState = (DeviceState & ~(Valve5Open));                          /* stop spraying */
          /* batchState = batchStateLast;                                           go back to where you where in the sequence */
      }
      else if (extArmEndStop && (elapsedTimeArm >= ARM_START_DLY))              /* reached the wrong end after start-up */
      {
          extArmStep = ARM_MTR_BACKWD;
      }
      break;

      case ARM_SPR_ACT_MTR_IDLE:                                                /* complete */
      break;

      default:
      break;
  }
#endif /* extended arm */
}
#endif                                                                          // ============== eNd Batch plant functions ===========================
/*-----------------------------------------------------------------------------
 *      calculateTimeDiff:  Simple function to give a time difference between two tick counters
 *
 *  Parameters:  int64_t *TimeDuration, uint64_t *TimeDurationLast
 *               TimeDuration set to -ve means initialise
 *
 *  Return: nothing value put into timeduration variable
 *----------------------------------------------------------------------------*/
void calculateTimeDiff( int64_t *TimeDuration, uint64_t *TimeDurationLast )
{
  if (*TimeDuration<=-1)                                                        // Value to initialise
  {
    *TimeDurationLast=g_timer5_counter;                                         // Remember the current counter value
    *TimeDuration==0;                                                           // first time differnce after init is zero
  }
  else
  {
    if ( *TimeDurationLast > g_timer5_counter )                                 // Overrun
    {
       *TimeDuration = (UINT64_MAX - *TimeDurationLast) + g_timer5_counter;     // Calculate difference in time
    }
    else
    {
       *TimeDuration = (g_timer5_counter - *TimeDurationLast);                  // Calculate difference in time
    }
    *TimeDurationLast = g_timer5_counter;                                       // Remember the last one
  }
}
/*-----------------------------------------------------------------------------
 *      calculateTime2Now:  Simple function to give a time difference from a start
 *                          trigger until the current moment
 *
 *  Parameters:  int64_t TimeDuration, uint64_t TimeDurationLast
 *               TimeDuration set to -ve means initialise
 *
 *  Return: nothing value put into timeduration variable
 *----------------------------------------------------------------------------*/
void calculateTime2Now( int64_t *TimeDuration, uint64_t *TimeDurationLast )
{
  if (*TimeDuration<=-1)                                                        // Value to initialise
  {
    *TimeDurationLast=g_timer5_counter;                                         // Remember the current counter value at initialisation only
    *TimeDuration==0;                                                           // first time differnce after init is zero
  }
  else
  {
    if ( *TimeDurationLast > g_timer5_counter )                                 // Overrun
    {
       *TimeDuration = (UINT64_MAX - *TimeDurationLast) + g_timer5_counter;     // Calculate difference in time
    }
    else
    {
       *TimeDuration = (g_timer5_counter - *TimeDurationLast);                  // Calculate difference in time
    }
  }
}
/*-----------------------------------------------------------------------------
 *      calculateMins2Now:  Simple function to give a time difference from a start
 *                          trigger until the current moment in minutes
 *
 *  Parameters:  uint8_t TimeDuration, uint8_t TimeDurationLast
 *               TimeDuration set to -ve means initialise
 *
 *  Return: nothing value put into timeduration variable
 *----------------------------------------------------------------------------*/
void calculateMins2Now( uint8_t *TimeDuration, uint8_t *TimeDurationLast )
{
  if (*TimeDuration<=-1)                                                        // Value to initialise
  {
    *TimeDurationLast=g_timeDate.min;                                           // Remember the current counter value at initialisation only
    *TimeDuration==0;                                                           // first time differnce after init is zero
  }
  else
  {
    if ( *TimeDurationLast > g_timeDate.min )                                   // Overrun
    {
       *TimeDuration = (UINT64_MAX - *TimeDurationLast) + g_timeDate.min;       // Calculate difference in time
    }
    else
    {
       *TimeDuration = (g_timeDate.min - *TimeDurationLast);                    // Calculate difference in time
    }
  }
}
/*-----------------------------------------------------------------------------
 *      calculateHrs2Now:  Simple function to give a time difference from a start
 *                          trigger until the current moment in hours
 *
 *  Parameters:  uint8_t TimeDuration, uint8_t TimeDurationLast
 *               TimeDuration set to -ve means initialise
 *
 *  Return: nothing value put into timeduration variable
 *----------------------------------------------------------------------------*/
void calculateHrs2Now( uint8_t *TimeDuration, uint8_t *TimeDurationLast )
{
  if (*TimeDuration<=-1)                                                        // Value to initialise
  {
    *TimeDurationLast=g_timeDate.hour;                                          // Remember the current counter value at initialisation only
    *TimeDuration==0;                                                           // first time differnce after init is zero
  }
  else
  {
    if ( *TimeDurationLast > g_timeDate.hour )                                  // Overrun
    {
       *TimeDuration = (UINT64_MAX - *TimeDurationLast) + g_timeDate.hour;      // Calculate difference in time
    }
    else
    {
       *TimeDuration = (g_timeDate.hour - *TimeDurationLast);                   // Calculate difference in time
    }
  }
}
/*-----------------------------------------------------------------------------
 *      setRandRange:  Get a random value in the specified range
 *
 *  Parameters:  min Lower bound, max Upper bound
 *
 *  Return: Random value in the specified range
 *----------------------------------------------------------------------------*/
int16_t setRandRange(int16_t min, int16_t max)
{
   int16_t value;

   if(max > min)                                                                //Valid parameters?
   {
      value = min + (rand() % (max - min + 1));                                 //Pick up a random value in the given range
   }
   else
   {
      value = min;                                                              //Use default value
   }

   return value;                                                                //Return the random value
}
/*-----------------------------------------------------------------------------
 *      checkMotor():  checks that a motor starts in a time or indicates an alarm
 *                     updates state machine for GUI
 *
 *  Parameters: device_t *dev
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void checkMotor( device_t *dev )
{
   if (dev->cmd != dev->cmdLast)                                                /* change of commanded state */
   {
      dev->TimeDuration=-1;
      calculateTime2Now( &dev->TimeDuration, &dev->TimeDurationLast );
      dev->alarm = false;
      dev->cmdLast = dev->cmd;
   }
   else if (dev->cmd==START)                                                    /* cmd to START */
   {
      calculateTime2Now( &dev->TimeDuration, &dev->TimeDurationLast );
      if ((dev->TimeDuration > dev->AlmTimeOut) && (dev->fb==false))
      {
         dev->alarm = true;
         dev->state = RUN_TIMEOUT;
      }
      else if (dev->fb==true)
      {
         dev->state = RUNNING;
      }
      else
      {
         dev->state = STARTING;
      }
   }
   else if (dev->cmd==STOP)                                                     /* cmd to stop*/
   {
      calculateTime2Now( &dev->TimeDuration, &dev->TimeDurationLast );
      if ((dev->TimeDuration > dev->AlmTimeOut) && (dev->fb==true))
      {
         dev->alarm = true;
         dev->state = STOP_TIMEOUT;
      }
      else if (dev->fb==false)
      {
         dev->state = STOPPED;
      }
      else
      {
         dev->state = STOPPING;
      }
   }
}

/*-----------------------------------------------------------------------------
 *      checkValve():  checks that a valve opens or closes in a time or indicate an alarm
 *                     also updates state machine for GUI allows 1 or 2 feedbacks
 *
 *  Parameters: device_t *dev
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void checkValve( device_t *dev )
{
   if (dev->cmd != dev->cmdLast)                                                /* change of state occurred */
   {
      dev->TimeDuration=-1;
      calculateTime2Now( &dev->TimeDuration, &dev->TimeDurationLast );
      dev->alarm = false;
      dev->cmdLast = dev->cmd;
   }
   else if (dev->cmd==OPEN)                                                     /* comand to OPEN */
   {
      calculateTime2Now( &dev->TimeDuration, &dev->TimeDurationLast );
      if (((dev->TimeDuration > dev->AlmTimeOut) && (dev->fb==false)) && (dev->numfb==1u))
      {
         dev->alarm = true;
         dev->state = OPN_TIMEOUT;
      }
      else if ((((dev->TimeDuration > dev->AlmTimeOut) && (dev->fb==false)) && (dev->fb2==true)) && (dev->numfb==2u))
      {
         dev->alarm = true;
         dev->state = OPN_TIMEOUT;
      }
      else if (dev->fb==true)
      {
         dev->state = OPEN;
      }
      else
      {
         dev->state = OPENING;
      }
   }
   else if (dev->cmd==CLOSE)                                                    /* command to CLOSE */
   {
      calculateTime2Now( &dev->TimeDuration, &dev->TimeDurationLast );
      if (((dev->TimeDuration > dev->AlmTimeOut) && (dev->fb==true)) && (dev->numfb==1u))
      {
         dev->alarm = true;
         dev->state = CLS_TIMEOUT;
      }
      else if ((((dev->TimeDuration > dev->AlmTimeOut) && (dev->fb==true)) && (dev->fb2==false)) && (dev->numfb==2u))
      {
         dev->alarm = true;
         dev->state = CLS_TIMEOUT;
      }
      else if (dev->fb==false)
      {
         dev->state = CLOSED;
      }
      else
      {
         dev->state = CLOSING;
      }
   }
}
/*-----------------------------------------------------------------------------
 *      checkPulseIntegrity():  check pulse integrity of rotary speed / flow measurement
 *
 *  Parameters: rotat_sens_obj_t *risingEdge1Obj, rotat_sens_obj_t *fallingEdge1Obj,
 *              rotat_sens_obj_t *risingEdge2Obj, rotat_sens_obj_t *fallingEdge2Obj,
 *              pulse_integrity_t *puls
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void checkPulseIntegrity( rotat_sens_obj_t *risingEdge1Obj, rotat_sens_obj_t *fallingEdge1Obj, rotat_sens_obj_t *risingEdge2Obj, rotat_sens_obj_t *fallingEdge2Obj, pulse_integrity_t *puls )
{
    /* the boundaries of the test is each rising and falling edge input and their previous state */
    float32_t risingEdge1,risingEdge2,fallingEdge1,fallingEdge2;
    uint8_t pulsIntegStat=0u;                                                  /* restart the sequence of events to look at the spped signals */

    while(pulsIntegStat < 2u)
    {
        switch (pulsIntegStat)
        {
           case 0u:
           risingEdge1=risingEdge1Obj->rot_speed;
           risingEdge2=risingEdge2Obj->rot_speed;
           fallingEdge1=fallingEdge1Obj->rot_speed;
           fallingEdge2=fallingEdge1Obj->rot_speed;
           break;

           case 1u:
           risingEdge1=risingEdge1Obj->accel;
           risingEdge2=risingEdge2Obj->accel;
           fallingEdge1=fallingEdge1Obj->accel;
           fallingEdge2=fallingEdge1Obj->accel;
           break;
         }

         /* check rising and falling edge to ensure we have no errors in sampling and are sampling quick enough where pi_db is a reasonable ammount of acceleration per scan  */
        if (((abs(risingEdge1-fallingEdge1) < puls->pi_db) && (abs(risingEdge2-fallingEdge2) < puls->pi_db)) || ((abs(risingEdge1-fallingEdge1) > puls->pi_db) && (abs(risingEdge2-fallingEdge2) > puls->pi_db))) /* all match or all different (high noise or sampling too slow) then average them all */
        {
           puls->re1_match= true;
           puls->fe1_match= true;
           puls->re2_match= true;
           puls->fe2_match= true;
        }
        else if ((abs(risingEdge1-fallingEdge1) > puls->pi_db) && (abs(risingEdge2-fallingEdge2) < puls->pi_db)) /* errors are on channel 1 */
        {
           puls->re1_match= false;
           puls->fe1_match= false;
           puls->re2_match= true;
           puls->fe2_match= true;
        }
        else if ((abs(risingEdge1-fallingEdge1) < puls->pi_db) && (abs(risingEdge2-fallingEdge2) > puls->pi_db)) /* errors are on channel 2 */
        {
           puls->re1_match= true;
           puls->fe1_match= true;
           puls->re2_match= false;
           puls->fe2_match= false;
        }

        /* check each of the 2 pulse inputs against each other they are either 90 out of phase or repeated */
        if (((abs(risingEdge1-risingEdge2) < puls->pi_db) && (abs(fallingEdge1-fallingEdge2) < puls->pi_db)) || ((abs(risingEdge1-risingEdge2) < puls->pi_db) && (abs(fallingEdge1-fallingEdge2) < puls->pi_db))) /* all match or not then average them all */
        {
           /* puls.re1_match= true;
           puls.fe1_match= true;
           puls.re2_match= true;
           puls.fe2_match= true; under this scenario leave as is ? */
           if ((abs(risingEdge1-risingEdge2) < puls->pi_db) && (abs(fallingEdge1-fallingEdge2) < puls->pi_db))     /* all within range */
           {
              switch(puls->int_state)
              {
                  case 1u:                                                      /* we have seen an error on rising edge counter */
                  puls->count1Good=++puls->count1Good % UINT8_MAX;                /* count the errors  */
                  if (puls->count1Good >= NO_OF_ROT_TEETH)                       /* we counted each tooth on the pickup */
                      puls->missingTeeth= false;                                 /* set a state of random noise being the error */
                  break;

                  case 2u:
                  puls->count2Good=++puls->count2Good % UINT8_MAX;               /* we have seen a second error on rising edge counter */
                  if (puls->count2Good >= NO_OF_ROT_TEETH)                       /* we counted each tooth on the pickup */
                      puls->missingTeeth= false;                                 /* set a state of random noise being the error */
                  break;

                 case 3u:
                 if (puls->count1Good == puls->count2Good)                      /* regular pattern to the error */
                 {
                    puls->missingTeeth= true;                                 /* set a state of missing teeth */
                 }
                 else
                 {
                    puls->missingTeeth= false;                                 /* set a state of random noise being the error */
                 }
                 puls->int_state= 0u;
                 puls->count1Good=0u;
                 puls->count2Good=0u;
                 break;
              }
           }
        }
        else if ((abs(risingEdge1-risingEdge2) > puls->pi_db) && (abs(fallingEdge1-fallingEdge2) < puls->pi_db)) /* errors are on rising edge (instability) */
        {
           puls->re1_match= puls->re1_match & false;
           puls->fe1_match= puls->fe1_match & true;
           puls->re2_match= puls->re2_match & false;
           puls->fe2_match= puls->fe2_match & true;
           if (abs(risingEdge1-risingEdge2) > puls->pi_db)                                                /* look for a regular pattern of error example missing teeth on the pickup sensor */
           {
               switch(puls->int_state)
               {
                  case 0u:
                  puls->int_state= 1u;
                  break;

                  case 1u:
                  puls->int_state= 2u;
                  break;

                  case 2u:
                  puls->int_state= 3u;
                  break;
                }
                if (puls->missingTeeth == true)                                                   /* if the encoder inputs are 90 out of phase then take the max value */
                {
                    if (risingEdge1 < risingEdge2)
                    {
                       puls->re2_match= true;
                    }
                    else
                    {
                       puls->re1_match= true;
                    }
                 }
            }
        }
        else if ((abs(risingEdge1-risingEdge2) < puls->pi_db) && (abs(fallingEdge1-fallingEdge2) > puls->pi_db)) /* errors are on channel falling edge (instability) */
        {
           puls->re1_match= puls->re1_match & true;
           puls->fe1_match= puls->fe1_match & false;
           puls->re2_match= puls->re2_match & true;
           puls->fe2_match= puls->fe2_match & false;
           if (puls->missingTeeth == true)                                                   /* if the encoder inputs are 90 out of phase then take the max value */
           {
              if (fallingEdge1 < fallingEdge2)
              {
                puls->fe2_match= true;
              }
              else
              {
                puls->fe1_match= true;
              }
           }
        }

        switch (pulsIntegStat)
        {
           case 0u:
           /* calculate average speed using values which meet the integrity */
           puls->pic_Velocity= (risingEdge1*puls->re1_match)+(risingEdge2*puls->re2_match)+(fallingEdge1*puls->fe1_match)+(fallingEdge2*puls->fe2_match)/(puls->re1_match+puls->re2_match+puls->fe1_match+puls->fe2_match);
           pulsIntegStat=1u;
           break;

           case 1u:
           /* calculate average acceleration using values which meet the integrity */
           puls->pic_Accel= (risingEdge1*puls->re1_match)+(risingEdge2*puls->re2_match)+(fallingEdge1*puls->fe1_match)+(fallingEdge2*puls->fe2_match)/(puls->re1_match+puls->re2_match+puls->fe1_match+puls->fe2_match);
           pulsIntegStat=2u;
           break;
        }

    }

}
/*-----------------------------------------------------------------------------
 *      moveVehicle():  moves the craft left right or forward back according to 
 *                      state object passed and places them into the output WORD  
 *                      output as defined in io.h
 *
 *  Parameters: ocr_movement_obj_t *move, uint8_t *output
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void moveVehicle( ocr_movement_obj_t *move, uint8_t *output )
{
   uint8_t output1=0u;
   uint8_t output2=0u;
   
   switch (move->leftRig)                                                       /* left right movement */
   {
      case __roStop:
      output1 = 0u;
      break;

      case Lef:
      output1 = go_left | reverse_LR;                                           /* put gear box into reverse and go left */
      //move->reverse_lr = 1u;
      break;

      case Rig:
      output1 = go_right;
      //move->reverse_lr = 0u;
      break;
      
      default:
      break;
   }
   switch (move->fwdBack)                                                       /* forward back movement */
   {
      case __roStop:
      output2 = 0;
      break;

      case Back:
      output2 = go_back | reverse_FB;                                           /* put gearbox into reverse and go backwards */
      //move->reverse_fb = 1u;
      break;
      
      case fwd:
      output2 = go_fwd;
      //move->reverse_fb = 0u;
      break;

      default:
      break;

   }
   //*output=(((((output2 | move->reverse_fb)<<4u) | (output1)) | (move->reverse_lr)) | (move->turnby90<<3u));
   *output= (((output2<<4u) | output1) | (move->turnby90<<3u));
}
#if defined(WEATHER_STAT)
/*
 * Copyright (C) 2011 Andreas Gaeb, Max Chtangeev
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version
 * The functions reads a Davis VantagePro(2) weather station Packet collected
 * from a serial port.  It asks for new data (Davis' LOOP command) in the
 * specified intervals, extracts the relevant data (ambient pressure and
 * temperature, wind speed and direction)
 * Also handlers for Kestrel Station added
 *
 * Ported by ACP Aviation
 *
 * Useful links:
 * - <a href="http://www.davisnet.com/weather/products/vantagepro.asp">Weather Stations</a>
 * - <a href="http://www.davisnet.com/support/weather/download/VantageSerialProtocolDocs_v230.pdf">Communication docs</a>
 *
 */
 /*-----------------------------------------------------------------------------
 *      RstDavisWeatherStation():  reset or START the davis workstation
 *
 *  Parameters: uint8_t uartNo
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void RstDavisWeatherStation( uint8_t uartNo )
{
  char newline = '\n';
  switch (uartNo)
  {
     case 1u:
     UART1_Write_Text((char *)newline);
     break;

     case 2u:
     UART2_Write_Text((char *)newline);
     break;
     
     case 3u:
     UART3_Write_Text((char *)newline);
     break;
     
     case 4u:
     UART4_Write_Text((char *)newline);
     break;
     
     case 5u:
     UART5_Write_Text((char *)newline);
     break;
     
     case 6u:
     UART6_Write_Text((char *)newline);
     break;
     
     default:
     break;
  }
}
 /*-----------------------------------------------------------------------------
 *      RstKestrelWeatherStation():  reset or START the kestrel workstation
 *
 *  Parameters: uint8_t uartNo
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void RstKestrelWeatherStation( )                                                /* Send an 'S' command (read sensor data) to the station */
{
  char cmd[KES_CMD_LENGTH];

  memset((void*)cmd,0,strlen(cmd));
  strcpy((char*)cmd,"S\r");
}

 /*-----------------------------------------------------------------------------
 *      getKestrelValues():  Populate values[] array with floats extracted from the packet string.
 *
 *  Parameters: char* msg, float32_t* values
 *
 *  Return:     int16_t number of chars it has fetched from the kestrel response
 *----------------------------------------------------------------------------*/
int16_t getKestrelValues(char* msg, float32_t* values)
{
  int16_t i,ret=0;
  char* tok;
  char delim = ',';

  if ((msg != NULL) && (values != NULL))
  {
     tok = strtok(msg, &delim);
     for(i=0; i<KES_NUM_PARAMS; i++)
     {
       if (tok == NULL)
       {
          ret = i;
          i=KES_NUM_PARAMS+1;
       }
       else
       {
         values[i] = atof(tok);
         ret = i;
         tok = strtok(NULL, &delim);
       }
     }
  }
  return ret;
}
 /*-----------------------------------------------------------------------------
 *      decodeDavisNet():  Populate davDat object with floats extracted from the packet string.
 *
 *  Parameters: uint8_t *packet, WeatherStat_t *davDat, uint8_t portNo
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void decodeDavisNet( uint8_t *packet, WeatherStat_t *davDat, uint8_t portNo )
{
  char expected[3u];                                                            // check packet integrity
  
  if ((packet == NULL) || (davDat == NULL)) return;
  
  if (strncmp((char *)packet,(char *)expected, 3) != 0)
  {
    RstDavisWeatherStation( portNo );
    return;
  }
  strcpy((char*)expected,(char*)"LOO");
  // TODO CRC checking (is rather involved for the Davis protocol)

  if (davDat->setUnits == WEATHER_UNIT_IMPERIAL)                                // get relevant data and convert to SI units see chapter IX.1 of the protocol definition
  {
     davDat->pstatic= ((float32_t)(packet[7u] | packet[8u] << 8u));             // original is inches Hg / 1000
     davDat->temp = ((float32_t)(packet[12u] | packet[13u] << 8u)) / 10.0f;     // original is deg F / 10
     davDat->windspeed = ((float32_t)packet[14u]);                              // original is miles per hour
     davDat->winddir_deg = ((float32_t)(packet[16u] | packet[17u] << 8u));
     davDat->rel_humidity = -1.;                                                // TODO Get the real humidity value from message
  }
  else
  {
     davDat->pstatic = ((float32_t)(packet[7u] | packet[8u] << 8u))*3.386388640341f; // original is inches Hg / 1000
     davDat->temp = (((float32_t)(packet[12u] | packet[13u] << 8u))/10.0f - 32.0f)*5.0f/9.0f;   // original is deg F / 10
     davDat->windspeed = ((float32_t)packet[14u])*0.44704f;                     // original is miles per hour
     davDat->winddir_deg = ((float32_t)(packet[16u] | packet[17u] << 8u));
     davDat->rel_humidity = -1.;                                                // TODO Get the real humidity value from message
  }
}
 /*-----------------------------------------------------------------------------
 *      decodeKestrel():  Populate kesDat object with floats extracted from the packet string.
 *
 *  Parameters: uint8_t *packet, WeatherStat_t *kesDat
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void decodeKestrel( uint8_t *packet, WeatherStat_t *kesDat )
{

  char expected[3u];                                                            // check packet integrity
  float32_t values[KES_NUM_PARAMS];
  
  if ((packet == NULL) || (kesDat == NULL)) return;

  if (strncmp((char *)packet,(char *)expected, 3) != 0)
  {
    RstKestrelWeatherStation( );                                                // START kestrel comms
    return;
  }
  strcpy((char*)expected,(char*)"LOO");
  
  if (getKestrelValues((char*) packet, &values)>=5)                             // parse the Packet from commas and make floats
  {
     switch (kesDat->setUnits)                                                  // get relevant data and convert to SI units see chapter IX.1 of the protocol definition
     {
        case WEATHER_UNIT_IMPERIAL:
        kesDat->pstatic = values[KES_BP];                                       // inches Hg
        kesDat->temp = values[KES_TP];                                          // original is deg F
        kesDat->windspeed = values[KES_WS];                                     // miles per hour
        kesDat->winddir_deg = values[KES_MG];
        kesDat->rel_humidity = values[KES_RH];
        break;
     
        case WEATHER_UNIT_FROMIMP:
        kesDat->pstatic = values[KES_BP] * 3386.4f;                             // original is inches Hg
        kesDat->temp = (values[KES_TP] - 32.0f) * 5.0f/9.0f;                    // original is deg F
        kesDat->windspeed = values[KES_WS] * 0.44704f;                          // original is miles per hour
        kesDat->winddir_deg = values[KES_MG];
        kesDat->rel_humidity = values[KES_RH];
        break;
     
        case WEATHER_UNIT_METRIC:
        kesDat->pstatic = values[KES_BP] * 100.0f;                              // original is hPa
        kesDat->temp = values[KES_TP];                                          // original is deg C
        kesDat->windspeed = values[KES_WS] * 0.2778f;                           // original is km/h
        kesDat->winddir_deg = values[KES_MG];
        kesDat->rel_humidity = values[KES_RH];
        break;
     
        default:
        break;
     }
  }
}
#endif
#if defined(VOLZ_PROTOCOL)
 /*-----------------------------------------------------------------------------
 *      Volz_send_value():  calculate CRC for volz serial protocol and send the data.
 *
 *  Parameters: VolZ_t *vObj
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Volz_send_value( VolZ_t *vObj )
{
    uint8_t dataV[VOLZ_DATA_FRAME_SIZE];                                        // prepare Volz protocol data.
    uint16_t value;
    uint16_t tmp1;
    uint8_t i,j;
    uint16_t crc = 0xFFFFU;

    value = vObj->pwmValue + VOLZ_EXTENDED_POSITION_MIN;                              // scale the PWM value to Volz value
    value = vObj->pwmValue * VOLZ_SCALE_VALUE / (VOLZ_PWM_MAX - VOLZ_PWM_MIN);

    if (value > VOLZ_EXTENDED_POSITION_MAX)                                     // make sure value stays in range
    {
       value = VOLZ_EXTENDED_POSITION_MAX;
    }
    dataV[0u] = VOLZ_SET_EXTENDED_POSITION_CMD;
    dataV[1u] = vObj->chan;                                                     // send actuator id as 1 based index so ch1 will have id 1, ch2 will have id 2 ....
    dataV[2u] = Hi(value);
    dataV[3u] = Lo(value);
    for(i=0u; i<4u; i++)                                                        // calculate Volz CRC value according to protocol definition
    {
        tmp1 = (uint16_t)dataV[i];
       //crc = (tmp1 << 8u); ^ crc;                                             take input data into message that will be transmitted
        crc = (tmp1 << 8u) ^ crc;
        for(j=0u; j<8u; j++)
        {
            if (crc & 0x8000U)
            {
                crc = (crc << 1u) ^ 0x8005U;
            }
            else
            {
                crc = crc << 1u;
            }
        }
    }
    dataV[4u] = Hi(crc);                                                        // add CRC result to the message
    dataV[5u] = Lo(crc);

    switch(vObj->uartNo)
    {
          case 1u:
          for(i=0;i<VOLZ_DATA_FRAME_SIZE;i++)
          {
            UART1_write(dataV[i]);                                              // Send it to serial UART1
          }
          break;
          
          case 2u:
          for(i=0;i<VOLZ_DATA_FRAME_SIZE;i++)
          {
            UART2_write(dataV[i]);                                              // Send it to serial UART2
          }
          break;
          
          case 3u:
          for(i=0;i<VOLZ_DATA_FRAME_SIZE;i++)
          {
            UART3_write(dataV[i]);                                              // Send it to serial UART3
          }
          break;

          case 4u:
          for(i=0;i<VOLZ_DATA_FRAME_SIZE;i++)
          {
            UART4_write(dataV[i]);                                              // Send it to serial UART4
          }
          break;
          
          case 5u:
          for(i=0;i<VOLZ_DATA_FRAME_SIZE;i++)
          {
            UART5_write(dataV[i]);                                              // Send it to serial UART5
          }
          break;

          case 6u:
          for(i=0;i<VOLZ_DATA_FRAME_SIZE;i++)
          {
            UART6_write(dataV[i]);                                              // Send it to serial UART6
          }
          break;
    }
}
#endif
#if defined(ROBOTIS_PROTOCOL)
 /*-----------------------------------------------------------------------------
 *      RobotisServo_add_stuffing():  add byte stuffing to Packet to send
 *                                    from Robotis SDK. This pads the packet as required by the protocol
 *  Parameters: uint8_t *packet
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void RobotisServo_add_stuffing(uint8_t *packet)
{
    int16_t packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int16_t packet_length_out = packet_length_in;
    uint16_t packet_length_before_crc;
    uint8_t *packet_ptr;
    uint16_t i;
    uint16_t out_index;
    uint16_t in_index;

    if (packet_length_in < 8) 
    {
        return;                                                                 // INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
    }

    packet_length_before_crc = packet_length_in - 2u;
    for (i = 3u; i < packet_length_before_crc; i++) 
    {
        packet_ptr = &packet[i+PKT_INSTRUCTION-2u];
        if (packet_ptr[0u] == 0xFFu && packet_ptr[1u] == 0xFFu && packet_ptr[2u] == 0xFDu) 
        {
            packet_length_out++;
        }
    }

    if (packet_length_in == packet_length_out) 
    {
        return;                                                                 // no stuffing required
    }
    out_index  = packet_length_out + 6u - 2u;                                   // last index before crc
    in_index   = packet_length_in + 6u - 2u;                                    // last index before crc
    while (out_index != in_index)
    {
            if (packet[in_index] == 0xFDu && packet[in_index-1u] == 0xFFu && packet[in_index-2u] == 0xFFu)
            {
               packet[out_index--] = 0xFDu;                                     // byte stuffing
               if (out_index != in_index) 
               {
                packet[out_index--] = packet[in_index--];                       // FD
                packet[out_index--] = packet[in_index--];                       // FF
                packet[out_index--] = packet[in_index--];                   // FF
               }
            }
            else
            {
               packet[out_index--] = packet[in_index--];
            }
    }

    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}
 /*-----------------------------------------------------------------------------
 *      Robotis_send_packet():  set up buffer with Packet to send
 *
 *  Parameters: uint8_t id, uint16_t reg, uint32_t value, uint8_t len
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Robotis_send_packet(uint8_t *txpacket)
{
    uint16_t crc;

    uint16_t total_packet_length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7u;      // check max packet length

    RobotisServo_add_stuffing(txpacket);

    txpacket[PKT_HEADER0]   = 0xFFu;                                            // make packet header
    txpacket[PKT_HEADER1]   = 0xFFu;
    txpacket[PKT_HEADER2]   = 0xFDu;
    txpacket[PKT_RESERVED]  = 0x00u;

    // add CRC16
    crc = RobotisServo_crc(0, txpacket, total_packet_length - 2u);    // 2: CRC16
    txpacket[total_packet_length - 2u] = DXL_LOBYTE(crc);
    txpacket[total_packet_length - 1u] = DXL_HIBYTE(crc);
}

 /*-----------------------------------------------------------------------------
 *      Robotis_send_command():  send a command to a single servo, changing a register value
 *
 *  Parameters: uint8_t id, uint16_t reg, uint32_t value, uint8_t len
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Robotis_send_command(uint8_t id, uint16_t reg, uint32_t value, uint8_t len)
{
    uint8_t txpacket[16];

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH_L] = 5u + len;
    txpacket[PKT_LENGTH_H] = 0u;
    txpacket[PKT_INSTRUCTION] = INST_WRITE;
    txpacket[PKT_INSTRUCTION+1u] = DXL_LOBYTE(reg);
    txpacket[PKT_INSTRUCTION+2u] = DXL_HIBYTE(reg);
    memcpy(&txpacket[PKT_INSTRUCTION+3u], &value, min(len,4u));

    Robotis_send_packet(txpacket);
}
#endif
#if defined(DSHOT_MOTOR)
/*
   ESP32 Arduino code for DSHOT600 protocol

   Copyright (c) 2020 Simon D. Levy

   Adapted from https://github.com/JyeSmith/dshot-esc-tester/blob/master/dshot-esc-tester.ino,
   which contains the following licensing notice:

   "THE PROP-WARE LICENSE" (Revision 42):
   <https://github.com/JyeSmith> wrote this file.  As long as you retain this notice you
   can do whatever you want with this stuff. If we meet some day, and you think
   this stuff is worth it, you can buy me some props in return.   Jye Smith

   This file is part of Hackflight.
   Its been ported by ACP Aviation

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */
 /*-----------------------------------------------------------------------------
 *      dShotOutputOne():  drive dshot motor
 *
 *  Parameters: dShot_motor_t *motor
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void dShotOutputOne(dShot_motor_t *motor)
{
     uint16_t packet = (motor->outputValue << 1) /* | (motor->telemetry ? 1 : 0) */ ;

     // https://github.com/betaflight/betaflight/blob/09b52975fbd8f6fcccb22228745d1548b8c3daab/src/main/drivers/pwm_output.c#L523
     int16_t csum = 0;
     int16_t i;
     int16_t csum_data = packet;
     for (i = 0; i < 3; i++) {
         csum ^=  csum_data;
         csum_data >>= 4;
     }
     csum &= 0xf;
     packet = (packet << 4) | csum;

     // durations are for dshot600
     // https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
     // Bit length (total timing period) is 1.67 microseconds (T0H + T0L or T1H + T1L).
     // For a bit to be 1, the pulse width is 1250 nanoseconds (T1H – time the pulse is high for a bit value of ONE)
     // For a bit to be 0, the pulse width is 625 nanoseconds (T0H – time the pulse is high for a bit value of ZERO)
     for (i = 0; i < DSHOT_PACKT_SZ; i++) {
         if (packet & 0x8000) {
            motor->dshotPacket[i].level0 = 1;
            motor->dshotPacket[i].duration0 = 100;
            motor->dshotPacket[i].level1 = 0;
            motor->dshotPacket[i].duration1 = 34;
     } else {
            motor->dshotPacket[i].level0 = 1;
            motor->dshotPacket[i].duration0 = 50;
            motor->dshotPacket[i].level1 = 0;
            motor->dshotPacket[i].duration1 = 84;
     }
     packet <<= 1;
  }

} // outputOne
 /*-----------------------------------------------------------------------------
 *      writedShotOut():  Scale output to given motor
 *
 *  Parameters: uint8_t index, float32_t value, dShot_motor_t *motor
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void writedShotOut(uint8_t index, float32_t value, dShot_motor_t *motor )
{
   motor[index]->outputValue = DSHOT_MIN + (uint16_t)(value * (DSHOT_MAX-DSHOT_MIN));
}

#endif

#if defined(KIN_SCUL)
#define LEN_OF_KINETIC 4u
#define KIN_SCUL_STX 0x84u
/*-----------------------------------------------------------------------------
 *      setKineticSculpture():  set a kinetic sculpture drive
 *
 *  Parameters: int8_t servoNo, float32_t float_value, uint8_t uartPort
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void setKineticSculpture( int8_t servoNo, float32_t float_value, uint8_t uartPort )
{
   const uint16_t minServoQuarters = 704u * 4u;
   const uint16_t maxServoQuarters = 2496u * 4u;
   unsigned char buf[LEN_OF_KINETIC];
   uint8_t loop;
   uint16_t driveValue = Scale_Float_Value( 0.0f, 100.0f, minServoQuarters, maxServoQuarters, float_value );

   buf[0u] = KIN_SCUL_STX;
   buf[1u] = servoNo & 0xFFu;
   buf[2u] = driveValue & 0x7Fu;
   buf[3u] = (driveValue>>7u) & 0x7Fu;

   switch (uartPort)
   {
          case 1u:
          for (loop=0u;loop<LEN_OF_KINETIC;loop++)
             UART1_write(buf[loop]);
          break;
          
          case 2u:
          for (loop=0u;loop<LEN_OF_KINETIC;loop++)
             UART2_write(buf[loop]);
          break;
          
          case 3u:
          for (loop=0u;loop<LEN_OF_KINETIC;loop++)
             UART3_write(buf[loop]);
          break;

          case 4u:
          for (loop=0u;loop<LEN_OF_KINETIC;loop++)
             UART4_write(buf[loop]);
          break;
          
          case 5u:
          for (loop=0u;loop<LEN_OF_KINETIC;loop++)
             UART5_write(buf[loop]);
          break;

          case 6u:
          for (loop=0u;loop<LEN_OF_KINETIC;loop++)
             UART6_write(buf[loop]);
          break;
          
          default:
          break;
    }
} 
#endif

#if defined(AVNav_USED)
/* Copyright (c) 2014 AVBotz
 * Ported by ACP Aviation
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ========================================================================== */
int16_t State_getEquivalentAngle(int16_t theta)
{
        if (theta < 0.0f) {
           return (theta+360.0f);
        }
        if (theta >= 360.0f) {
           return (theta-360.0f);
        }
        return theta;
}

// returns + if theta1 is CW of theta2
// returns - if theta1 is CCW of theta2
int16_t State_getAngleDifference(int16_t theta1, int16_t theta2)
{
        int16_t dif = theta1 - theta2;
        if (-180.0f < dif && dif < 180.0f)
           return dif;
                
        if (dif > 180.0f)
           return dif - 360.0f;
                
        if (dif < -180.0f)
           return dif + 360.0f;
                
        return 180.0f;                                                          // dif is 180 or -180
}
void VisionTask_initializePan(int16_t center, int16_t delta, float32_t *panHeading[3u])
{
        *panHeading[0u] = center;
        *panHeading[1u] = State_getEquivalentAngle(center - delta);
        *panHeading[2u] = State_getEquivalentAngle(center + delta);
}
float32_t VisionTask_objectDistance(int16_t px, int16_t in, bool horizontal, opt_flow_cam_config_t* cam)
{
        float32_t angle = (float32_t)(horizontal?cam->fov_horizontal:cam->fov_vertical) * px/2.0f / (horizontal?cam->cam_width:cam->cam_height);          // figure out what is half of the angle to object fills
        return ((float32_t)in)/2.0f / tan(PI / 180.0f * angle);                 // use trig
}
int16_t State_getProperty(VISTASK_STATE_PROPERTY_e prop, int16_t *stateProps[STATE_PROP_NUM])
{
        if (prop < 0 || prop >= STATE_PROP_NUM)
           return -1;
        
        return *stateProps[prop];
}
// returns true if the value is valid for a given property
// returns false if it isn't
bool State_checkProperty(VISTASK_STATE_PROPERTY_e prop, int16_t value, int16_t *statePropRanges[STATE_PROP_NUM][2u])
{
        return (value >= *statePropRanges[prop][0u] && value <= *statePropRanges[prop][1u]);
}
int16_t VisionTask_getPointHeading(const Vectr p, const opt_flow_cam_config_t cam, VISTASK_STATE_PROPERTY_e currentHeading, int16_t *stateProps[STATE_PROP_NUM])
{
        int16_t heading;
        if (stateProps == NULL) return -127;                                    /* misra null pointer check returns -127 */
        heading = State_getProperty(currentHeading,stateProps);
        heading += p.x*cam.fov_horizontal/cam.cam_width - cam.fov_horizontal/2.0f; /* Vectr p is the Point (from opencv) */ 
        return State_getEquivalentAngle(heading);
}
// return -1 if bad index, otherwise what the property was actually set to
int16_t State_setProperty(VISTASK_STATE_PROPERTY_e prop, int16_t newValue, int16_t *stateProps[STATE_PROP_NUM], int16_t *statePropRanges[STATE_PROP_NUM][2u])
{
        if (prop < 0 || prop >= STATE_PROP_NUM)
                return -1;
        if (*stateProps[prop] == newValue)
                return newValue;
        
        if (newValue < *statePropRanges[prop][0u])                              // clamp the values
                newValue = *statePropRanges[prop][0u];
        else if (newValue > *statePropRanges[prop][1u])
                newValue = *statePropRanges[prop][1u];

        *stateProps[prop] = newValue;
        return newValue;
}
void VisionTask_goToNextPanHeading(uint8_t *currentPanHeading, float32_t *panHeading, VISTASK_STATE_PROPERTY_e desiredHeading, int16_t *stateProps[STATE_PROP_NUM], int16_t *statePropRanges[STATE_PROP_NUM][2u] )
{
        *currentPanHeading=++*currentPanHeading%4u;
        switch (*currentPanHeading)
        {
           case 0u:
           case 2u:
           State_setProperty(desiredHeading, ROUNDSINT(*panHeading), stateProps, statePropRanges ); break;
           case 1u:
           State_setProperty(desiredHeading, ROUNDSINT(*(panHeading+sizeof(float32_t))), stateProps, statePropRanges ); break;
           case 3u:
           State_setProperty(desiredHeading, ROUNDSINT(*(panHeading+(2u*sizeof(float32_t)))), stateProps, statePropRanges ); break;
           default: break;
        }
}
/*
 * Sends a command to the mbed in AVNav format
 * The prefix determines the value set
 * Add valid prefixes in Serial::commandList
 * The value is set to num
 */
 /* Copyright (c) 2014 AVBotz */
void SerialMbed_writeCommand(char prefix, int16_t num, int8_t uartNo)
{
        int8_t AvnCnt;
        char command[4u];                                                       // assemble the command
        command[0u] = prefix;
        command[1u] = ((num>>6u) & 0x3fu) + 0x20u;
        command[2u] = (num & 0x3fu) + 0x20u;
        command[3u] = '\n';
        
        switch (uartNo)                                                         // send it on the serial line 
        {
           case 1u:
           for(AvnCnt=0;AvnCnt<5;AvnCnt++)
           {
             UART1_write(command[AvnCnt]);                                      // Send it to serial UART1
           }
           break;
           
           case 2u:
           for(AvnCnt=0;AvnCnt<5;AvnCnt++)
           {
             UART2_write(command[AvnCnt]);                                      // Send it to serial UART2
           }
           break;
           
           case 3u:
           for(AvnCnt=0;AvnCnt<5;AvnCnt++)
           {
             UART3_write(command[AvnCnt]);                                      // Send it to serial UART3
           }
           break;
           
           case 4u:
           for(AvnCnt=0;AvnCnt<5;AvnCnt++)
           {
             UART4_write(command[AvnCnt]);                                      // Send it to serial UART4
           }
           break;
           
           case 5u:
           for(AvnCnt=0;AvnCnt<5;AvnCnt++)
           {
             UART5_write(command[AvnCnt]);                                      // Send it to serial UART5
           }
           break;
           
           case 6u:
           for(AvnCnt=0;AvnCnt<5;AvnCnt++)
           {
             UART6_write(command[AvnCnt]);                                      // Send it to serial UART6
           }
           break;
           
           default:
           break;
        }                                                                                
}
void SerialMbed_sendData( int16_t *stateProps[STATE_PROP_NUM], int8_t uartNo )
{
        int16_t i;
        for (i = 0; i < AVNav_DATA_NUM_TX; i++)
        {
           SerialMbed_writeCommand(AVNav_txList[i][0u], State_getProperty(AVNav_txList[i][1u],stateProps), uartNo);
        }
}
void setStateEngineFromRequest( AVNav_Data_t dat, int16_t *stateProp[STATE_PROP_NUM] )
{
        *stateProp[desiredHeading] = dat.desiredHeading;
        *stateProp[desiredDepth] = dat.desiredDepth;
        *stateProp[desiredPower] = dat.desiredPower;        
}
/*
 * Converts encoded number from AVNav into an int
 *
 * AVNav data encoding:
 * Numbers are sent in 2 bytes, each with a value 0x20-0x5f, 32-95, or ' ' to '_'
 * The numbers are sent in base 64, with 0x20 being equal to 0 and 0x5f being equal to 63.
 * The first byte is the upper digit (most significant byte/big endian)
 * So the number 299 is sent as "$K" and is decoded as follows:
 *        $K => 0x24, 0x4B => 36, 75 => 4, 43 => 4*64 + 43 = 299
 * This keeps both bytes as readily readable ASCII characters that 
 *        are not the lower case letters used for commands
 */
 /*-----------------------------------------------------------------------------
 *      SerialMbed_decodeAVNav : decode the AvNAV
 *
 *  Parameters: char* dataV 
 *
 *  Return:     int16_t 
 *----------------------------------------------------------------------------*/
int16_t SerialMbed_decodeAVNav(char* dataV)
{
        return ((*dataV - 0x20)<<6u) | (*(dataV+1) - 0x20);
}
 /*-----------------------------------------------------------------------------
 *      SerialMbed_parseInBuffr : parse AvNAV interrupt receive buffer
 *
 *  Parameters: uint8_t *messageBuf, int16_t *stateProps[STATE_PROP_NUM],  
 *              int16_t *statePropRanges[STATE_PROP_NUM][2u]
 *
 *  Return:     void 
 *----------------------------------------------------------------------------*/
void SerialMbed_parseInBuffr( uint8_t *messageBuf, int16_t *stateProps[STATE_PROP_NUM], int16_t *statePropRanges[STATE_PROP_NUM][2u] )
{
        int16_t i = 0;
        int16_t var, kill = -1;
        int16_t j;
        
        if ((messageBuf==NULL) || ((stateProps==NULL) || (statePropRanges==NULL))) return;
        
        // the format for the message being processed:
        //        h__d__p__[l|k]e__
        // where [l|k] is l or k depending on the kill state
        while (messageBuf[i] != 0 && i < 20)
        {
                var = -1;
                for (j = 0; j < AVNav_DATA_NUM_RX; j++)
                {
                        if (messageBuf[i] == AVNav_rxList[j][0u])
                        {
                                var = SerialMbed_decodeAVNav(&messageBuf[i+1u]);
                                if (State_checkProperty(AVNav_rxList[j][1u], var, statePropRanges )==true)    
                                        State_setProperty(AVNav_rxList[j][1u], var, stateProps, statePropRanges);
                                break;
                        }
                        else if (messageBuf[i] == 'l')
                        {
                                kill = 0;                                       /* not currently handled */
                                break;
                        }
                        else if (messageBuf[i] == 'k')
                        {
                                kill = 1;                                       /* not currently handled */
                                break;
                        }
                }
                if (var != -1)
                {
                        i+=3;
                        var = -1;
                }
                else
                {
                        i=++i % 22;
                }
        }
}
#endif /* end AvNAV */

#if defined(VIDEORAY_M5_USED)
/*-----------------------------------------------------------------------------
 *      vm5_crc_slow:  Compute the chosen CRC for VM5 message
 *
 *  Parameters: uint8_t const * const p_message, uint8_t n_bytes
 *
 *  Return:     uint32_t - as chosen by the type define
 *----------------------------------------------------------------------------*/
uint32_t vm5_crc_slow(uint8_t const * const p_message, uint8_t n_bytes)
{
   uint32_t remainder = VM5_CRC_POST_R_ID;
   uint8_t byte1;
   uint8_t bitcount;

   if (p_message != NULL)
   {
      for (byte1 = 0u; (byte1 < n_bytes); byte1++)                                 /* Perform modulo-2 division, one byte at a time. */
      {
         remainder ^= p_message[byte1];
         for (bitcount = VM5_BITS_PER_BYTE; bitcount > 0u; bitcount--)               /* Perform modulo-2 division, one bit at a time. */
         {
            if (remainder & 1U)
            {
               remainder = (remainder >> 1u) ^ VM5_CRC32_POLY;                     /*  note right rather than left shift */
            }
            else
            {
               remainder >>= 1;
            }
         }
      }
   }

#if defined(_CPU_BIG_ENDIAN )
   return (SWAPINT32(remainder) ^ 0xFFFFFFFFLU);
#else
   return (remainder ^ 0xFFFFFFFFLU);                                           /* The final remainder is the CRC result. */
#endif

}  /* vm5_crc_slow() */
 /*-----------------------------------------------------------------------------
 *      vm5_static_thrust_msg():  send static thrust message
 *
 *  Parameters: vm5_object_t *vm5
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void vm5_static_thrust_msg(vm5_object_t *vm5)
{
        uint32_t crc;
        uint8_t t,x;
        uint8_t const headerBytes[VM5_HEADER_LEN] = VM5THRUSTER_HEADER;
        uint8_t const staticBytes[VM5_PROP_CMD_LEN] = VM5THRUSTER_PROP_CMD;
        
        switch (vm5->state)                                                     /* select the action from the vm5 object state machine */
        {
           case VM5_INIT:                                                       /* send out the uart port */
           switch (vm5->uartPort)                                               /* choose the port and send the byte */
           {
                case 1u:
                UART1_Init(VM5_BAUD);                                           /* set baud 8,n,1 on the uart */
                break;

                case 2u:
                UART2_Init(VM5_BAUD);                                           /* set baud 8,n,1 on the uart */
                break;

                case 3u:
                UART3_Init(VM5_BAUD);                                           /* set baud 8,n,1 on the uart */
                break;

                case 4u:
                UART4_Init(VM5_BAUD);                                           /* set baud 8,n,1 on the uart */
                break;

                case 5u:
                UART5_Init(VM5_BAUD);                                           /* set baud 8,n,1 on the uart */
                break;

                case 6u:
                UART6_Init(VM5_BAUD);                                           /* set baud 8,n,1 on the uart */
                break;

                default:
                break;
            }
            RS485Master_Init();                                                 // initialize MCU as Master for RS485
            vm5->state = VM5_HEADER;
             
            case VM5_HEADER:                                                    /* form header */
            memcpy((void*)&vm5->Buf,(void*)&headerBytes,VM5_HEADER_LEN);        /* copy the header to the send buffer */
            vm5->Buf += VM5_HEADER_LEN;                                         /* increment the pointer by the data length */
            vm5->state = VM5_R_ID;                                              /* go to next state */

            case VM5_R_ID:                                                      /* form command */
            memcpy((void*)&vm5->Buf,(void*)&staticBytes,VM5_PROP_CMD_LEN);      /* copy command */
            vm5->Buf += VM5_PROP_CMD_LEN;                                       /* increment the pointer by the data length */
            vm5->state = VM5_POWER;                                             /* go to next state */

            case VM5_POWER:                                                     /* add the power */
            for (x=0u; x<3u; x++)
            {
                for (t = 0u; t < VM5_NUM_THRUSTERS; ++t)                        // The PROPULSION_COMMAND is structured such that thrust value floats are given for each consecutive motor id, starting from 0.
                {
                   if ((vm5->m5[x].power[t] >= 1.0f) && (vm5->m5[x].power[t] <= 1.0f))  /* check validity of the value is in range -1.0 is full reverse and 1.0 full forward on the thrusters. */
                   {
                      memcpy((void*)&vm5->Buf,(void*)&vm5->m5[x].power[t],sizeof(float32_t));  /* copy the value to the message */
                   }
                   else
                   {
                      memset((void*)&vm5->Buf,0.0f,sizeof(float32_t));
                   }
                   vm5->Buf += sizeof(float32_t);                               /* move down the message by 1 float size */
                }
             }
             memset((void*)&vm5->Buf,0,1);
             vm5->state = VM5_CRC;

            case VM5_POWER_ONLY:                                                /* resend only power array without rewriting the header and comand bytes */
            vm5->Buf += VM5_HEADER_LEN + VM5_PROP_CMD_LEN;
            for (x=0u; x<3u; x++)
            {
                for (t = 0u; t < VM5_NUM_THRUSTERS; ++t)                        // The PROPULSION_COMMAND is structured such that thrust value floats are given for each consecutive motor id, starting from 0.
                {
                   if ((vm5->m5[x].power[t] >= 1.0f) && (vm5->m5[x].power[t] <= 1.0f))  /* check validity of the value is in range */
                   {
                      memcpy((void*)&vm5->Buf,(void*)&vm5->m5[x].power[t],sizeof(float32_t));  /* copy the value to the message */
                   }
                   else
                   {
                      memset((void*)&vm5->Buf,0.0f,sizeof(float32_t));
                   }
                   vm5->Buf += sizeof(float32_t);                               /* move down the message by 1 float size */
                }
             }
             memset((void*)&vm5->Buf,0,1);
             vm5->state = VM5_CRC;
             
             case VM5_CRC:                                                      /* add the CRC */
             crc = vm5_crc_slow(&vm5->Buf, strlen(vm5->Buf));
             memcpy((void*)&vm5->Buf, (void*)&crc, VM5_CRC_LEN);
             vm5->state = VM5_SEND;
             
             case VM5_SEND:                                                     /* send out the uart port */
             RS485_rxtx_pin_direction = 1;                                      /* ready to send */
             switch (vm5->uartPort)                                             /* choose the port and send the byte */
             {
                case 1u:
                for (x=0u;x<(VM5_PROPULSION_COMMAND_PACKET_LEN+VM5_CRC_LEN);x++) /* for each byte in the Packet */
                {
                   UART1_write(vm5->Buf[x]);                                    /* send each char through the uart */
                }
                break;

                case 2u:
                for (x=0u;x<(VM5_PROPULSION_COMMAND_PACKET_LEN+VM5_CRC_LEN);x++)
                {
                   UART2_write(vm5->Buf[x]);
                }
                break;

                case 3u:
                for (x=0u;x<(VM5_PROPULSION_COMMAND_PACKET_LEN+VM5_CRC_LEN);x++)
                {
                   UART3_write(vm5->Buf[x]);
                }
                break;

                case 4u:
                for (x=0u;x<(VM5_PROPULSION_COMMAND_PACKET_LEN+VM5_CRC_LEN);x++)
                {
                   UART4_write(vm5->Buf[x]);
                }
                break;

                case 5u:
                for (x=0u;x<(VM5_PROPULSION_COMMAND_PACKET_LEN+VM5_CRC_LEN);x++)
                {
                   UART5_write(vm5->Buf[x]);
                }
                break;

                case 6u:
                for (x=0u;x<(VM5_PROPULSION_COMMAND_PACKET_LEN+VM5_CRC_LEN);x++)
                {
                   UART6_write(vm5->Buf[x]);
                }
                break;

                default:
                break;
             }
             RS485_rxtx_pin_direction = 0;                                      /* STOP sending and set pin to receive direction */
             vm5->state = VM5_SENT;
             
             case VM5_SENT:                                                     /* now wait for caller fill some new data in (either polled or by exception) */
             break;

             case VM5_POWER_SET:                                                /* set the power array with new data then re-schedule the send */
             break;
             
             default:
             break;
        }
}
#endif /* videoray m5 */

#if defined(TILT_SERVO1_USED)
/* hokuyo_node 
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, SOH DE LOONG.
 *  All rights reserved.
 *
 *  Ported and converted to PIC32/FT900 by ACP Aviation
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
 */

/*-----------------------------------------------------------------------------
 *      servo1_write_uart():  write to chosen uart port the tilting servo mesage 
 *
 *  Parameters: int16_t comport, uint8_t *buf, uint8_t len
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void servo1_write_uart(int16_t comport, uint8_t *buf, uint8_t len  )
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
 *      pop_queue():  pop queue 
 *
 *  Parameters: servo1_t *servo
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/   
void pop_queue( servo1_t *servo )
{
        int8_t i;
        servo->noInQ = servo->noInQ % SERVO1_MAX_REQ_TYPES;
        for (i = 0; i < servo->noInQ ; i++)
        {
                servo->reqQueue[i] = servo->reqQueue[i+1u];
        }
        servo->noInQ--;        
}
/*-----------------------------------------------------------------------------
 *      push_queue():  push queue 
 *
 *  Parameters: servo1_t *servo, servo1_request_e request
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void push_queue( servo1_t *servo, servo1_request_e request )
{
        int8_t i; 
        servo->noInQ = servo->noInQ % SERVO1_MAX_REQ_TYPES;
        for (i = 0; i < servo->noInQ ; i++)
        {
                if (servo->reqQueue[i] == request) return;
        }        
        servo->reqQueue[servo->noInQ] = request;
        servo->noInQ=++servo->noInQ % SERVO1_MAX_REQ_TYPES;        
}

/*-----------------------------------------------------------------------------
 *      servo1_move():  move servo to a position with a certain speed 
 *
 *  Parameters: int16_t comport, int16_t ID, int16_t angle, int16_t rpm, servo1_t *servo
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void servo1_move(int16_t comport, int16_t ID, int16_t angle, int16_t rpm, servo1_t *servo)
{
        const int16_t upper_limit = 205;
        const int16_t lower_limit = 130;
        uint8_t servo_id = ID;        
        uint8_t angle_h,angle_l,rpm_h,rpm_l,chksum,inst[11u],i;
        
        if (angle > upper_limit)
                angle = upper_limit;
        else if (angle < lower_limit)
                angle = lower_limit;         

        angle_h = (angle * 0X3FFu) / (256u * 360u);
        angle_l = ((angle * 0X3FFu) / 360u) % 256u;
        
        rpm_h = (rpm * 0X3FFu) / (256u * 114u);
        rpm_l = ((rpm * 0X3FFu) / 114u) % 256u;

        chksum = 0XFFu - servo_id - 0X07u - 0X03u - 0X1Eu - angle_l - angle_h - rpm_l - rpm_h;
        
        sprintf(inst,"%c%c%c%c%c%c%c%c%c%c%c",0XFFu, 0XFFu, servo_id, 0X07u, 0X03u, 0X1Eu, angle_l, angle_h, rpm_l, rpm_h, chksum);
        
        switch(servo->state)
        {
                case SERVO1_REQ_MOVE:
                servo1_write_uart(comport, inst, 8);
                servo->state = SERVO1_WAIT_MOVE_REPLY;
                servo->lastTickRef = CP0_GET(CP0_COUNT);
                servo->ResendCnt = ++servo->ResendCnt % UINT8_MAX;
                if (servo->ResendCnt >= SERVO1_RESEND_MAX)
                {
                   servo->reqMove = 1u;                                         /* set triger back on */
                   servo->state = SERVO1_REQ_DONE;                              /* finish and re-queue the request */
                }                        
                break;
                
                case SERVO1_WAIT_MOVE_REPLY:
                calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef);
                if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_MOVE;                
                break;
                
                case SERVO1_MOVE_REPLY_RCV:                                     /* state set in interrupt */
                while ((servo->buf[i]!=0XFFu && servo->buf[i + 1u]!=0XFFu) && (i<20))
                 i=++i % 21;
                if ((i < 20) && (servo->buf[i + 2u] == ID))
                {                        
                  servo->state = SERVO1_REQ_DONE;                  
                }
                else if (servo->buf[i + 2u] == ID)
                {
                  servo->state = SERVO1_REQ_MOVE;         
                }                        
                else 
                { 
                  calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef);
                  if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_MOVE;        
                }
                break;
                
                case SERVO1_REQ_DONE:
                servo->ResendCnt = 0u;
                if (servo->reqMove)
                {
                   push_queue(servo,SERVO1_REQ_MOVE);
                   servo->reqMove = 0u;
                }                        
                if (servo->reqQueue[0u] == SERVO1_REQ_MOVE)
                {
                   servo->state = SERVO1_REQ_MOVE;
                   pop_queue(servo);
                }                         
                break;
        }

}

/*-----------------------------------------------------------------------------
 *      servo1_read_angle():  read the position of the servo 
 *
 *  Parameters: int16_t comport, int16_t ID, servo1_t *servo
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
#define TILT_SERV1_READ_ANGL(A,ID,chksum) do{ sprintf(A,"%c%c%c%c%c%c%c%c",0XFFu, 0XFFu, ID, 0X04u, 0X02u, 0X24u, 0X02u, chksum); } while(0)
void servo1_read_angle(int16_t comport, int16_t ID, servo1_t *servo)
{
        int8_t i = 0;
        uint8_t chksum = 0XFFu - ID - 0X2Cu;
        uint8_t inst[8u];                                                       //= {0XFFu, 0XFFu, ID, 0X04u, 0X02u, 0X24u, 0X02u, chksum};
        
        switch(servo->state)
        {
                case SERVO1_REQ_ANGLE:
                TILT_SERV1_READ_ANGL(inst,ID,chksum);
                servo1_write_uart(comport, inst, 8);
                servo->state = SERVO1_WAIT_ANGLE_REPLY;
                servo->lastTickRef = CP0_GET(CP0_COUNT);
                servo->ResendCnt = ++servo->ResendCnt % UINT8_MAX;
                if (servo->ResendCnt >= SERVO1_RESEND_MAX)
                {
                        servo->reqAngle = 1u; /* set triger back on */
                        servo->state = SERVO1_REQ_DONE; /* finish and re-queue the request */
                }
                break;
                
                case SERVO1_WAIT_ANGLE_REPLY:
                calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef);
                if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_ANGLE;                
                break;
                
                case SERVO1_ANGLE_REPLY_RCV:                    /* state set in interrupt */
                while ((servo->buf[i]!=0XFFu && servo->buf[i + 1u]!=0XFFu) && (i<20))
                 i=++i % 21;
                if ((i < 20) && (servo->buf[i + 2u] == ID))
                {                        
                  servo->ang = ((float64_t)servo->buf[i + 6u] * 256u + (float64_t)servo->buf[i + 5u]) * 360u / 0X3FFu;
                  servo->state = SERVO1_REQ_DONE;                  
                }
                else if (servo->buf[i + 2u] == ID)
                {
                   servo->state = SERVO1_REQ_ANGLE;         
                }                        
                else 
                { 
                  calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef);
                  if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_ANGLE;        
                }
                break;
                
                case SERVO1_REQ_DONE:
                servo->ResendCnt = 0u;
                if (servo->reqAngle)
                {
                   push_queue(servo,SERVO1_REQ_ANGLE);
                   servo->reqAngle = 0u;
                }                        
                else if (servo->reqQueue[0u] == SERVO1_REQ_ANGLE)
                {
                  servo->state = SERVO1_REQ_ANGLE;
                  pop_queue(servo);
                }                         
                break;
        }

}

/*-----------------------------------------------------------------------------
 *      servo1_read_speed():  read the speed of the servo
 *
 *  Parameters: int16_t comport, int16_t ID, servo1_t *servo
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
#define TILT_SERV1_READ_SPEED(A,ID,chksum) do{ sprintf(A,"%c%c%c%c%c%c%c%c",0XFFu, 0XFFu, ID, 0X04u, 0X02u, 0X26u, 0X02u, chksum); } while(0)
void servo1_read_speed(int16_t comport, int16_t ID, servo1_t *servo)
{
        int8_t i = 0;
        uint8_t chksum = 0XFFu - ID - 0X2Au;
        uint8_t inst[8u]; //= {0XFFu, 0XFFu, ID, 0X04u, 0X02u, 0X26u, 0X02u, chksum};
        
        switch(servo->state)
        {
                case SERVO1_REQ_SPEED:
                TILT_SERV1_READ_SPEED(inst,ID,chksum);
                servo1_write_uart(comport, inst, 8);
                servo->state = SERVO1_WAIT_SPEED_REPLY;
                servo->lastTickRef = CP0_GET(CP0_COUNT);
                servo->ResendCnt = ++servo->ResendCnt % UINT8_MAX;
                if (servo->ResendCnt >= SERVO1_RESEND_MAX)
                {
                        servo->reqSpeed = 1u; /* set triger back on */
                        servo->state = SERVO1_REQ_DONE; /* finish and re-queue the request */
                }
                break;
                
                case SERVO1_WAIT_SPEED_REPLY:
                calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef);
                if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_SPEED;
                break;
                
                case SERVO1_SPEED_REPLY_RCV:                    /* state set in interrupt */
                while ((servo->buf[i]!=0XFFu && servo->buf[i + 1u]!=0XFFu) && (i<10))
                 i=++i % 11;
                if ((i < 10) && (servo->buf[i + 2u] == ID))
                {                        
                  servo->speed = ((float64_t)servo->buf[6u] * 256u + (float64_t)servo->buf[5u]) * 114u / 0X3FFu;
                  servo->state = SERVO1_REQ_DONE;
                }
                else if (servo->buf[i + 2u] == ID)
                {
                   servo->state = SERVO1_REQ_SPEED;         
                }                        
                else 
                { 
                   calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef);
                   if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_SPEED;        
                }
                break;
                
                case SERVO1_REQ_DONE:
                servo->ResendCnt = 0u;
                if (servo->reqSpeed)
                {
                        push_queue(servo,SERVO1_REQ_SPEED);
                        servo->reqSpeed = 0u;
                }                        
                else if (servo->reqQueue[0u] == SERVO1_REQ_SPEED)
                {
                   servo->state = SERVO1_REQ_SPEED;
               pop_queue(servo);
        } 
                break;
        }        
        
}
/*-----------------------------------------------------------------------------
 *      servo1_read_ret_dly_time():  read the reply delay time of the servo
 *
 *  Parameters: int16_t comport, int16_t ID, servo1_t *servo
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
#define TILT_SERV1_READ_DLY(A,ID,chksum) do{ sprintf(A,"%c%c%c%c%c%c%c%c",0XFFu, 0XFFu, ID, 0X04u, 0X02u, 0X05u, 0X01u, chksum); } while(0)
void  servo1_read_ret_dly_time(int16_t comport, int16_t ID, servo1_t *servo)
{
        int8_t i = 0;
        uint8_t chksum = 0XFFu - ID - 0X0Cu;
        uint8_t inst[8u]; //= {0XFFu, 0XFFu, ID, 0X04u, 0X02u, 0X05u, 0X01u, chksum};

        switch(servo->state)
        {
                case SERVO1_REQ_REP_DLY:
                TILT_SERV1_READ_DLY(inst,ID,chksum);
                servo1_write_uart(comport, inst, 8);
                servo->state = SERVO1_WAIT_REP_DLY_REPLY;
                servo->lastTickRef = CP0_GET(CP0_COUNT);
                servo->ResendCnt = ++servo->ResendCnt % UINT8_MAX;
                if (servo->ResendCnt >= SERVO1_RESEND_MAX)
                {
                        servo->reqDly = 1u; /* set triger back on */
                        servo->state = SERVO1_REQ_DONE; /* finish and re-queue the request */
                }
                break;
                
                case SERVO1_WAIT_REP_DLY_REPLY:
                calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef);
                if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_REP_DLY;
                break;
                
                case SERVO1_REP_DLY_REPLY_RCV:                    /* state set in interrupt */
                while ((servo->buf[i]!=0XFFu && servo->buf[i + 1u]!=0XFFu) && (i<10))
                    i=++i % 11;
                if ((i < 10) && (servo->buf[i + 2u] == ID))
                {                        
                  servo->RepDelayTime = servo->buf[5u];
                  servo->state = SERVO1_REQ_DONE;
                }
                else if (servo->buf[i + 2u] == ID)
                {
                   servo->state = SERVO1_REQ_REP_DLY;         
                }                        
                else 
                { 
                  calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef);
                  if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_REP_DLY;        
                }
                break;
                
                case SERVO1_REQ_DONE:
                servo->ResendCnt = 0u;
                if (servo->reqDly)
                {
                   push_queue(servo,SERVO1_REQ_REP_DLY);
                   servo->reqDly = 0u;
                }                        
                else if (servo->reqQueue[0u] == SERVO1_REQ_REP_DLY)
                {
                   servo->state = SERVO1_REQ_REP_DLY;
                   pop_queue(servo);
                } 
                break;
        }        
        
}

/*-----------------------------------------------------------------------------
 *      servo1_read_voltage():  read voltage of servo
 *
 *  Parameters: int16_t comport, int16_t ID, servo1_t *servo
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
#define TILT_SERV1_READ_VOLT(A,ID,chksum) do{ sprintf(A,"%c%c%c%c%c%c%c%c",0XFFu, 0XFFu, ID, 0X04u, 0X02u, 0X02u, 0X01u, chksum); } while(0)
void servo1_read_voltage(int16_t comport, int16_t ID, servo1_t *servo)
{
        int8_t i = 0;
        uint8_t chksum = 0XFFu - ID - 0X31u;
        uint8_t inst[8u]; //= {0XFFu, 0XFFu, ID, 0X04u, 0X02u, 0X2Au, 0X01u, chksum};
        
        switch(servo->state)
        {
                case SERVO1_REQ_VOLT:
                TILT_SERV1_READ_VOLT(inst,ID,chksum);
                servo1_write_uart(comport, inst, 8);
                servo->state = SERVO1_WAIT_VOLT_REPLY;
                servo->lastTickRef = CP0_GET(CP0_COUNT);
                servo->ResendCnt = ++servo->ResendCnt % UINT8_MAX;
                if (servo->ResendCnt >= SERVO1_RESEND_MAX)
                {
                        servo->reqVolt = 1u; /* set triger back on */
                        servo->state = SERVO1_REQ_DONE; /* finish and re-queue the request */
                }
                break;
                
                case SERVO1_WAIT_VOLT_REPLY:
                calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef);
                if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_VOLT;
                break;
                
                case SERVO1_VOLT_REPLY_RCV:                    /* state set in interrupt */
                while ((servo->buf[i]!=0XFFu && servo->buf[i + 1u]!=0XFFu) && (i<10))
                    i=++i % 11;
                if ((i < 10) && (servo->buf[i + 2u] == ID))
                {                        
                  servo->voltage = servo->buf[5u] / 10.0f;
                  servo->state = SERVO1_REQ_DONE;
                }
                else if (servo->buf[i + 2u] == ID)
                {
                   servo->state = SERVO1_REQ_VOLT;         
                }                        
                else 
                { 
                  calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef);
                  if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_VOLT;        
                }
                break;
                
                case SERVO1_REQ_DONE:
                servo->ResendCnt = 0u;
                if (servo->reqVolt)
                {
                   push_queue(servo,SERVO1_REQ_VOLT);
                   servo->reqVolt = 0u;
                }                        
                else if (servo->reqQueue[0u] == SERVO1_REQ_VOLT)
                {
                  servo->state = SERVO1_REQ_VOLT;
                  pop_queue(servo);
                } 
                break;
        }
        
}
/*-----------------------------------------------------------------------------
 *      servo1_read_temp():  read temperature of servo in celsius
 *
 *  Parameters: int16_t comport, int16_t ID, servo1_t *servo
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
#define TILT_SERV1_READ_TEMP(A,ID,chksum) do{ sprintf(A,"%c%c%c%c%c%c%c%c",0XFFu, 0XFFu, ID, 0X04u, 0X02u, 0X2Bu, 0X01u, chksum); } while(0)
void servo1_read_temp(int16_t comport, int16_t ID, servo1_t *servo)
{
        int8_t i = 0;
        uint8_t chksum = 0XFFu - ID - 0X32u;
        uint8_t inst[8u]; //= {0XFFu, 0XFFu, ID, 0X04u, 0X02u, 0X2Bu, 0X01u, chksum};

        switch(servo->state)
        {
                case SERVO1_REQ_TEMPERAT:
                TILT_SERV1_READ_TEMP(inst,ID,chksum);
                servo1_write_uart(comport, inst, 8);
                servo->state = SERVO1_WAIT_TEMPERAT_REPLY;
                servo->lastTickRef = CP0_GET(CP0_COUNT);
                servo->ResendCnt = ++servo->ResendCnt % UINT8_MAX;
                if (servo->ResendCnt >= SERVO1_RESEND_MAX)
                {
                        servo->reqTemp = 1u; /* set triger back on */
                        servo->state = SERVO1_REQ_DONE; /* finish and re-queue the request */
                }
                break;
                
                case SERVO1_WAIT_TEMPERAT_REPLY:
                calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef);
                if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_TEMPERAT;
                break;
                
                case SERVO1_TEMPERAT_REPLY_RCV:                    /* state set in interrupt */
                while ((servo->buf[i]!=0XFFu && servo->buf[i + 1u]!=0XFFu) && (i<10))
                    i=++i % 11;
                if ((i < 10) && (servo->buf[i + 2u] == ID))
                {                        
                  servo->temperatCel = servo->buf[5u];
                  servo->state = SERVO1_REQ_DONE;
                }
                else if (servo->buf[i + 2u] == ID)
                {
                   servo->state = SERVO1_REQ_TEMPERAT;         
                }                        
                else 
                { 
                   calculateTick2Now(&servo->ticksSoFar,&servo->lastTickRef); 
                   if (servo->ticksSoFar > SERVO1_RETRY_LIM) servo->state = SERVO1_REQ_TEMPERAT;        
                }
                break;
                
                case SERVO1_REQ_DONE:
                servo->ResendCnt = 0u;
                if (servo->reqTemp)
                {
                   push_queue(servo,SERVO1_REQ_TEMPERAT);
                   servo->reqTemp = 0u;
                }                        
                else if (servo->reqQueue[0u] == SERVO1_REQ_TEMPERAT)
                {
                   servo->state = SERVO1_REQ_TEMPERAT;
                   pop_queue(servo);
                } 
                break;
        }
        
} 
#endif  /* -- end tilt servo hokuyo_node -- */

 /*-----------------------------------------------------------------------------
 *      getTimeFromString():  fills a mavlink time object with the time from a
 *      string of of format 00:00:00
 *
 *  Parameters: mavDelay_t *clock, unsigned char *inputString
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void getTimeFromString( mavDelay_t *clock, unsigned char *inputString )
{
  unsigned char *token;

  if ((clock == NULL) || (inputString == NULL))
  { /* for misra */  }
  else
  {
     token = strtok(inputString, ":");                                          // Returns first token
     if ( token == NULL )                                                       // spec needs more : in string this is an error
     {  }
     else                                                                       // the first token is hours
     {
       memset(token,0u,strlen(token)+1u);                                       // terminate string returned
       clock->hrs = atoi(token);                                                // clock hours
       token = strtok(inputString, ":");                                        // Returns second token
       if ( token == NULL )                                                     // no more tokens is error
       {  }
       else                                                                     // use the next token as the minutes
       {
          memset(token,0u,strlen(token)+1u);
          clock->mins = atoi(token);                                            // clock minutes
          token = strtok(inputString, ":");                                     // Returns third token
          if ( token == NULL )                                                  // should be the last token seconds
          {
             memset(token,0u,strlen(token)+1u);
             clock->secs = atoi(token);                                         // clock seconds
             clock->time = clock->secs + (clock->mins * 60ul) + (clock->hrs * 3600ul); // total seconds
          }
       }
     }
  }
}
 /*-----------------------------------------------------------------------------
 *      valueFromSexagesimal():  return a float from sexagesimal string X.Y.Z
 *      string of of format 00:00:00
 *
 *  Parameters: char *inputString, char *delim
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t valueFromSexagesimal( char *inputString, char *delim )
{
  char *token;
  float64_t a,b,c;

  if ((delim == NULL) || (inputString == NULL))
  { /* for misra */  }
  else
  {
     token = strtok(inputString, delim);                                        // Returns first token
     if ( token == NULL )                                                       // spec needs more : in string this is an error
     {  }
     else                                                                       // the first token is hours
     {
       memset(token,0u,strlen(token)+1u);                                       // terminate string returned
       a = atof(token);                                                         // first string
       token = strtok(inputString, delim);                                      // Returns second token
       if ( token == NULL )                                                     // no more tokens is error
       {  }
       else                                                                     // use the next token as the minutes
       {
          memset(token,0u,strlen(token)+1u);
          b = atof(token);                                                      // 2nd string
          token = strtok(inputString, delim);                                   // Returns third token
          if ( token == NULL )                                                  // should be the last token seconds
          {
             memset(token,0u,strlen(token)+1u);
             c = atof(token);                                                   // 3rd string
             return a + (b / 60.0f) + (c / 3600.0f);                            // value converted
          }
       }
     }
  }
}

#if defined(NAVSTIK_USED)
 /*-----------------------------------------------------------------------------
 *      NavStik_getRawValues():  gets a raw value from the navstik
 *
 *  Parameters: char* inpStr, navstickWordOffset_e offset
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t NavStik_getRawValues( char* inpStr, navstickWordOffset_e offset  )
{
   char* val = NULL;
   char* pos;
        
   if ((pos = strstr(inpStr,"NAVSTIK1")) != 0)
   {
     memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+offset), 7u);
   }
        
   return atof(val);
}
 /*-----------------------------------------------------------------------------
 *      NavStik_getAllValues():  gets all values from the navstik
 *                               performs calibration and uses if specified in mode
 *                               use mode = _NAVSTIK_START_CALIB on initialisation
 *
 *  Parameters: char* inpStr, navstik_object_t *nav, navstickUseCalib_e *mode
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void NavStik_getAllValues( char* inpStr, navstik_object_t *nav, navstickUseCalib_e *mode  )
{
   char* val = NULL;
   char* pos;
        
   if ((pos = strstr(inpStr,"NAVSTIK1")) != 0)
   {
      switch (*mode)                                                             /* do the gyro */
     {
         case _NAVSTIK_NO_CALIB:
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROX), 7u);
         nav->navstik_GYROX = atof(val);  
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROY), 7u);
         nav->navstik_GYROY = atof(val); 
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROZ), 7u);
         nav->navstik_GYROZ = atof(val); 
         break;
         
         case _NAVSTIK_START_CALIB:                                             /* set the caller variable to this at initialisation if you want calibration */
         nav->navstik_BiasG_x = -0.056503f;
         nav->navstik_BiasG_y = 0.023454f;
         nav->navstik_BiasG_z  =0.019339f;
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROX), 7u);
         nav->navstik_GYROX_OFTO = atof(val);
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROY), 7u);
         nav->navstik_GYROY_OFTO = atof(val); 
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROZ), 7u);
         nav->navstik_GYROZ_OFTO = atof(val); 
         nav->numOfSam = 1u;
         *mode == _NAVSTIK_REMOVING_OFFSET;
         break;
         
         case _NAVSTIK_REMOVING_OFFSET:                                         /* calculating gyro co-effients */
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROX), 7u);
         nav->navstik_GYROX_OFTO = atof(val);
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROY), 7u);
         nav->navstik_GYROY_OFTO = atof(val); 
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROZ), 7u);
         nav->navstik_GYROZ_OFTO = atof(val);
         if (nav->numOfSam >= NAVSTIK_NUM_OF_SAMP)
         {
            nav->navstik_BiasG_x = nav->navstik_GYROX_OFTO/NAVSTIK_NUM_OF_SAMP;
            nav->navstik_BiasG_y = nav->navstik_GYROY_OFTO/NAVSTIK_NUM_OF_SAMP;
            nav->navstik_BiasG_z = nav->navstik_GYROZ_OFTO/NAVSTIK_NUM_OF_SAMP;
            nav->navstik_GYROX_OFTO = 0.0f;
            nav->navstik_GYROY_OFTO = 0.0f;
            nav->navstik_GYROZ_OFTO = 0.0f; 
            *mode = _NAVSTIK_USE_CALIB;           
         } 
         nav->numOfSam = ++nav->numOfSam % (NAVSTIK_NUM_OF_SAMP+1u);
#if defined(NAVSTIK_GYRO_ALWAYS_UPD)
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROX), 7u);
         nav->navstik_GYROX = atof(val) - nav->navstik_BiasG_x;
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROY), 7u);
         nav->navstik_GYROY = atof(val) - nav->navstik_BiasG_y; 
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROZ), 7u);
         nav->navstik_GYROZ = atof(val) - nav->navstik_BiasG_z; 
#endif
         break;
         
         case _NAVSTIK_USE_CALIB:                                               /* calcualte the value */
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROX), 7u);
         nav->navstik_GYROX = atof(val) - nav->navstik_BiasG_x;
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROY), 7u);
         nav->navstik_GYROY = atof(val) - nav->navstik_BiasG_y; 
         memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_GYROZ), 7u);
         nav->navstik_GYROZ = atof(val) - nav->navstik_BiasG_z; 
         nav->numOfSam  = 0u;
         *mode = _NAVSTIK_REMOVING_OFFSET;                                      /* restart calib */
         break;
         
         default:
         break;                 
     }     
        
     if ( *mode == _NAVSTIK_NO_CALIB )                                          /* do the accelerometer and magnetrometer */
     {
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_ACCELX), 7u);
        nav->navstik_ACCELX = atof(val);
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_ACCELY), 7u);
        nav->navstik_ACCELY = atof(val);
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_ACCELZ), 7u);
        nav->navstik_ACCELZ = atof(val);  
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_MAGX), 7u);
        nav->navstik_MAGX = atof(val);  
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_MAGY), 7u);
        nav->navstik_MAGY = atof(val); 
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_MAGZ), 7u);
        nav->navstik_MAGZ = atof(val);    
     }
     else
     {
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_ACCELX), 7u);
        nav->navstik_ACCELX = ( atof(val) - navstik_BiasA_x )*navstik_Alpha_A;
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_ACCELY), 7u);
        nav->navstik_ACCELY = ( atof(val) - navstik_BiasA_y )*navstik_Beta_A;
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_ACCELZ), 7u);
        nav->navstik_ACCELZ = ( atof(val) - navstik_BiasA_z )*navstik_Gamma_A;
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_MAGX), 7u);
        nav->navstik_MAGX = (atof(val) - navstik_BiasH_x )*navstik_Alpha_H;  
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_MAGY), 7u);
        nav->navstik_MAGY = (atof(val) - navstik_BiasH_y )*navstik_Beta_H; 
        memcpy((void*)val, (void*)inpStr+(((uint32_t)pos)+_NAVSTIK_OFST_MAGZ), 7u);
        nav->navstik_MAGZ = (atof(val) - navstik_BiasH_z )*navstik_Gamma_H; 
     }
  }
}
#endif /* navstik */

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
#define GAUSS_NUMBER_OF_SEED_START 10u
/*-----------------------------------------------------------------------------
 *      whiteGaussInitSeed : init random generator with a new seed 
 *
 *
 *  Parameters: int16_t *seedIterator
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void whiteGaussInitSeed( int16_t *seedIterator )
{
  uint8_t seedInits[GAUSS_NUMBER_OF_SEED_START] = { 234u, 57667u, 76u, 10u, 111u, 2u, 7u, 11115u, 16u, 1014u };
  uint16_t seed;
  
  seed = (rand() * *seedIterator) % GAUSS_NUMBER_OF_SEED_START;
  srand(seedInits[seed]);
  *seedIterator = ++*seedIterator % INT16_MAX;
}
/*-----------------------------------------------------------------------------
 *      whiteGaussianNoise : white gaussian noise creation
 *
 *
 *  Parameters: float64_t * n1, float64_t * n2
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void whiteGaussianNoise(float64_t * n1, float64_t * n2)
{
  // Box Muller Method - from http://www.dspguru.com/dsp/howtos/how-to-generate-white-gaussian-noise
  float64_t v1, v2, s;
  uint16_t seed;

  seed = rand();
  srand(seed);                                                                  /* re-seed our random generator for next time */
    
  do
  {
    v1 = 2 * (rand()) / INT16_MAX - 1.0f;
    v2 = 2 * (rand()) / INT16_MAX - 1.0f;
    s = v1 * v1 + v2 * v2;
  } while (s >= 1);
   
  *n1 = sqrt(-2.0 * log(s) / s) * v1;
  if (n2)
    *n2 = sqrt(-2.0 * log(s) / s) * v2;
}
/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: Eddy Scott <scott.edward@aurora.aero>
 *   Ported by ACP Aviation
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 /*-----------------------------------------------------------------------------
 *      generateGaussianNoise : gaussian noise creation
 *
 *
 *  Parameters: float32_t mu, float32_t variance 
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t generateGaussianNoise(float32_t mu, float32_t variance)               // mu - mean of the noise for gaussian = 0, variance - variance of the noise
{
        /* Calculate normally distributed variable noise with mean = mu and variance = variance.  Calculated according to 
        Box-Muller transform */

        static const float32_t two_pi = 2.0*3.14159265358979323846f;            // 2*pi
        static float32_t z0;                                                    //calculated normal distribution random variables with mu = 0, var = 1;
        float32_t u1, u2,noise,s;                                                //random variables generated from c++ rand();

        do                                                                      /*Generate random variables in range (0 1] */
        {
            u1 = rand() * (1.0f / RAND_MAX);
            u2 = rand() * (1.0f / RAND_MAX);
            s = u1 * u1 + u2 * u2;
        }
        while (s >= 1);                                                         // Have a catch to ensure non-zero for log()

        z0 = sqrt(-2.0f * log(u1)) * cos(two_pi * u2);                          //calculate normally distributed variable with mu = 0, var = 1
        noise = z0 * sqrt(variance) + mu;                                       //calculate normally distributed variable with mu = mu, std = var^2
        return noise;
}
/*-----------------------------------------------------------------------------
 *      uniformNoise : create uniform noise
 *
 *
 *  Parameters: const float64_t mag
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t uniformNoise( const float64_t mag )
{
  return (((float64_t)rand()) / INT16_MAX * 2.0f - 1.0f) * mag;
}
/*-----------------------------------------------------------------------------
 *      Vicon_update : viocn update
 *
 *
 *  Parameters: const float64_t dt, float64_t tau_
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t Vicon_update( const float64_t dt, float64_t tau_ )
{
    float64_t n,x_;
    
    whiteGaussianNoise(&n,NULL);
    x_ = n * sqrt(1.0 - exp(-dt / tau_)) + x_ * exp(-dt / tau_);
    return x_;
}
#endif /* -- tf distort -- */

#if defined(TRAMP_PROTOCOL)
/*-----------------------------------------------------------------------------
 *      trampCrc : tramp protocol cyclic redundancy check
 *
 *
 *  Parameters: const trampFrame_t *frame
 *
 *  Return:     uint8_t 
 *----------------------------------------------------------------------------*/
static uint8_t trampCrc(const trampFrame_t *frame)
{
    uint8_t crc = 0;
    const uint8_t *p = (const uint8_t *)frame;
    const uint8_t *pEnd = p + (TRAMP_HEADER_LENGTH + TRAMP_PAYLOAD_LENGTH);
    for (; p != pEnd; p++) {
        crc += *p;
    }
    return crc;
}
/*-----------------------------------------------------------------------------
 *      trampFrameInit : tramp protocol frame init
 *
 *
 *  Parameters: uint8_t frameType, trampFrame_t *frame
 *
 *  Return:     void 
 *----------------------------------------------------------------------------*/
static void trampFrameInit(uint8_t frameType, trampFrame_t *frame)
{
    frame->header.syncStart = TRAMP_SYNC_START;
    frame->header.command = frameType;
    memset(frame->payload.buf, 0, sizeof(frame->payload.buf));
}
/*-----------------------------------------------------------------------------
 *      trampFrameClose : tramp protocol frame close
 *
 *
 *  Parameters: trampFrame_t *frame
 *
 *  Return:     void 
 *----------------------------------------------------------------------------*/
static void trampFrameClose(trampFrame_t *frame)
{
    frame->footer.crc = trampCrc(frame);
    frame->footer.syncStop = TRAMP_SYNC_STOP;
}
/*-----------------------------------------------------------------------------
 *      trampFrameGetSettings : tramp protocol get settings
 *
 *
 *  Parameters: trampFrame_t *frame
 *
 *  Return:     void 
 *----------------------------------------------------------------------------*/
void trampFrameGetSettings(trampFrame_t *frame)
{
    trampFrameInit(TRAMP_COMMAND_GET_CONFIG, frame);
    trampFrameClose(frame);
}
/*-----------------------------------------------------------------------------
 *      trampFrameSetFrequency : tramp protocol set frequency
 *
 *
 *  Parameters: trampFrame_t *frame, const uint16_t frequency
 *
 *  Return:     void 
 *----------------------------------------------------------------------------*/
void trampFrameSetFrequency(trampFrame_t *frame, const uint16_t frequency)
{
    trampFrameInit(TRAMP_COMMAND_SET_FREQ, frame);
    frame->payload.frequency = frequency;
    trampFrameClose(frame);
}
/*-----------------------------------------------------------------------------
 *      trampFrameSetPower : tramp protocol set power level
 *
 *
 *  Parameters: trampFrame_t *frame, const uint16_t power
 *
 *  Return:     void 
 *----------------------------------------------------------------------------*/
void trampFrameSetPower(trampFrame_t *frame, const uint16_t power)
{
    trampFrameInit(TRAMP_COMMAND_SET_POWER, frame);
    frame->payload.power = power;
    trampFrameClose(frame);
}
/*-----------------------------------------------------------------------------
 *      trampFrameSetActiveState : tramp protocol set active state
 *
 *
 *  Parameters: trampFrame_t *frame, const uint8_t active
 *
 *  Return:     void 
 *----------------------------------------------------------------------------*/
void trampFrameSetActiveState(trampFrame_t *frame, const uint8_t active)
{
    trampFrameInit(TRAMP_COMMAND_ACTIVE_STATE, frame);
    frame->payload.active = (uint8_t) active;
    trampFrameClose(frame);
}
/*-----------------------------------------------------------------------------
 *      trampParseResponseBuffer : tramp protocol parse response buffer
 *
 *
 *  Parameters: trampSettings_t *settings, const uint8_t *buffer, size_t bufferLen
 *
 *  Return:     uint8_t 
 *----------------------------------------------------------------------------*/
uint8_t trampParseResponseBuffer(trampSettings_t *settings, const uint8_t *buffer, size_t bufferLen)
{
    const trampFrame_t *frame = (const trampFrame_t *)buffer;
    const uint8_t crc = trampCrc(frame);
    if (bufferLen != TRAMP_FRAME_LENGTH) {
        return false;
    }
    if (crc != frame->footer.crc) {
        return false;
    }
    memcpy((void*)settings,(void*)&frame->payload.settings,(int16_t)sizeof(*settings));
    return true;
}
#endif
#if defined(NAVILOCK_USED)
/*-----------------------------------------------------------------------------
 *      navilockInterruptStateManager():  navilock interrupt state maanger
 *
 *  Parameters: navilock_t *nav
 *  Return:     void
 *----------------------------------------------------------------------------*/
void navilockInterruptStateManager( navilock_t *nav )                           /* called from within the port interrupt */
{
  switch(nav->state)
  {
      case NAVILOCK_REQ_TRACK:
      if (sizeof(nav->buf) >= NAVI_TRAK_DATA_LEN)
      nav->state = NAVILOCK_REPLY_RCV;        
      break;
      
      case NAVILOCK_WAIT_REPLY:
      if (sizeof(nav->buf) >= NAVI_TRAK_DATA_LEN)
      nav->state = NAVILOCK_REPLY_RCV;        
      break;

      case NAVILOCK_REQ_POINT:
      if (sizeof(nav->buf) >= NAVI_POINT_DATA_LEN)
      nav->state = NAVILOCK_POINTS_RCV;        
      break;
      
      case NAVILOCK_WAIT_POINT_REPLY:
      if (sizeof(nav->buf) >= NAVI_POINT_DATA_LEN)
      nav->state = NAVILOCK_POINTS_RCV;        
      break;        

      case NAVILOCK_REQ_SET_TOT_DIST:
      if (sizeof(nav->buf) >= 3)
      nav->state = NAVILOCK_SET_TOT_DIST_RCV;        
      break;
                
      case NAVILOCK_WAIT_DIST_SET_REPLY:
      if (sizeof(nav->buf) >= 3)
      nav->state = NAVILOCK_SET_TOT_DIST_RCV;        
      break;          

      case NAVILOCK_REQ_TOT_DIST:
      if (sizeof(nav->buf) >= 16)
      nav->state = NAVILOCK_GET_TOT_DIST_RCV;        
      break;
      
      case NAVILOCK_WAIT_GET_TOT_DIST_REPLY:
      if (sizeof(nav->buf) >= 16)
      nav->state = NAVILOCK_GET_TOT_DIST_RCV;        
      break;

      case NAVILOCK_REQ_ERAS_TRAK:
      if (sizeof(nav->buf) >= 3)
      nav->state = NAVILOCK_ERAS_TRAK_RCV;        
      break;
      
      case NAVILOCK_WAIT_ERAS_TRAK_REPLY:
      if (sizeof(nav->buf) >= 3)
      nav->state = NAVILOCK_ERAS_TRAK_RCV;        
      break;

      default:
      break;        
  }
}
#endif   /* end navlock */


// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/MathFunctions.C $
// $Id: MathFunctions.C 15461 2013-02-27 21:38:04Z jshen $ 
// Also code by : Rob Peters <rjpeters at usc dot edu> and Lior Elazary
// Ported to PIC32 by ACP Aviation

//! Random number generator from Numerical Recipes in C book
/*! Long period >2x10^18 random number generator of L Ecuyer with
    Bays-Durham shuffle. Return a uniform deviate in the interval
    (0,1).  Call with int variable as argument set to a negative
    value, and don't change this variable between calls unless you
    want to reseed the generator. You can get ain initial seed using
    the function getIdum() */
/*-----------------------------------------------------------------------------
 *  ran2 : Random number generator from Numerical Recipes in C book
 *
 *  Parameters: int16_t *idum
 *   
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t ran2(int16_t *idum)
{
  const int16_t IM1 = 2147483563, IM2 = 2147483399;
  const float64_t AM = (1.0/IM1);
  const int16_t IMM1 = IM1-1;
  const int16_t IA1 = 40014, IA2 = 40692, IQ1 = 53668, IQ2 = 52774;
  const int16_t IR1 = 12211, IR2 = 3791, NTAB = 32;
  const int16_t NDIV = 1+IMM1/NTAB;
  const float64_t EPS = 3.0e-16f, RNMX = 1.0f-EPS;

  int16_t j, k;
  int16_t idum2 = 123456789, iy = 0;
  int16_t iv1[NTAB];
  float64_t temp1;

  if (*idum <= 0) {
    *idum = (*idum == 0 ? 1 : -*idum);
    idum2=*idum;
    for (j=NTAB+7;j>=0;j--) {
      k=*idum/IQ1;
      *idum=IA1*(*idum-k*IQ1)-k*IR1;
      if (*idum < 0) *idum += IM1;
      if (j < NTAB) iv1[j] = *idum;
    }
    iy=iv1[0];
  }
  k=*idum/IQ1;
  *idum=IA1*(*idum-k*IQ1)-k*IR1;

  if (*idum < 0) *idum += IM1;
  k=idum2/IQ2;
  idum2=IA2*(idum2-k*IQ2)-k*IR2;
  if (idum2 < 0) idum2 += IM2;
  j=iy/NDIV;
  iy=iv1[j]-idum2;
  iv1[j] = *idum;
  if (iy < 1) iy += IMM1;
  if ((temp1=AM*iy) > RNMX) return RNMX;
  else return temp1;
}

// #################Get a random seed for ran2############################
/*! If useRandom is true, a random negative number will be return,
  otherwise a negative 16 bit integer that is always the same -123456 will be
  returned. */
/*-----------------------------------------------------------------------------
 *  getIdum : get clock count scaled as integer
 *
 *  Parameters: const bool useRandom 
 *   
 *  Return: int16_t
 *----------------------------------------------------------------------------*/
int16_t getIdum(const bool useRandom)
{
  if (useRandom)
    return -(CP0_GET(CP0_COUNT) % INT16_MAX);
  else
    return -123456;
}

/*-----------------------------------------------------------------------------
 *  setIdumRand : set seed for random or generate random int
 *
 *  Parameters: const bool useRandom 
 *   
 *  Return: int16_t
 *----------------------------------------------------------------------------*/
int16_t setIdumRand(const bool useRandom)
{
  if (useRandom)
  {
    srand(CP0_GET(CP0_COUNT) % INT16_MAX);                                      /* seed the random number generator */
    return -(CP0_GET(CP0_COUNT) % INT16_MAX);                                   /* return the seed as the first number */
  }
  else
    return -rand();                                                             /* return a random number generated */
}

/*-----------------------------------------------------------------------------
 *  gasdev : Randomly distributed Gaussian deviate with zero mean and unit variance
 *
 *  Parameters: int16_t *idum 
 *   
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t gasdev (int16_t *idum) 
{
  int16_t iset = 0;
  float64_t  gset;

  float64_t  fac, rsq, v1, v2;
  if (*idum < 0) iset = 0;
  if (iset == 0) {
    do {
      v1 = 2.0*ran2(idum)-1.0f;
      v2 = 2.0*ran2(idum)-1.0f;
      rsq = v1*v1 + v2*v2;
    } while (rsq >= 1.0f || rsq == 0.0f);

    fac = sqrt(-2.0f*log(rsq)/rsq);
    gset = v1*fac; 
    iset = 1;
    return v2*fac;
  } else {
    iset = 0;
    return gset;
  }
}

/*-----------------------------------------------------------------------------
 *  expdev : Randomly distributed Exponential deviate
 *
 *  Parameters: int16_t *idum 
 *   
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t expdev(int16_t *idum)
{
  float64_t dum;
  do dum = ran2(idum); while (dum == 0.0f);
  return -log(dum);
}


/*-----------------------------------------------------------------------------
 *  iNvLab_angle : finds a cosine of angle between vectors from pt0->pt1 and from pt0->pt2
 *
 *  Parameters: Vectr* pt1, Vectr* pt2, Vectr* pt0 
 *   
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t iNvLab_angle( Vectr* pt1, Vectr* pt2, Vectr* pt0 )
{
    float64_t dx1 = pt1->x - pt0->x;
    float64_t dy1 = pt1->y - pt0->y;
    float64_t dx2 = pt2->x - pt0->x;
    float64_t dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10f);
}
  
/* Lanczos method for real x > 0;
 * gamma=7, truncated at 1/(z+8)
 * [J. SIAM Numer. Anal, Ser. B, 1 (1964) 86]
 */
/*-----------------------------------------------------------------------------
 *  lngamma : Lanczos method for real x > 0
 *
 *  Parameters: float64_t x 
 *   
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t lngamma(float64_t x)
{

#ifdef HAVE_LGAMMA
  return lgamma(x);
#else

  // ripped from GSL (see above).
  /* coefficients for gamma=7, kmax=8  Lanczos method */
  static const float64_t lanczos_7_c[9u] = {
    0.99999999999980993227684700473478f,
    676.520368121885098567009190444019f,
    -1259.13921672240287047156078755283f,
    771.3234287776530788486528258894f,
    -176.61502916214059906584551354f,
    12.507343278686904814458936853f,
    -0.13857109526572011689554707f,
    9.984369578019570859563e-6f,
    1.50563273514931155834e-7f
  };
  float64_t Ag = lanczos_7_c[0u];
  int8_t k;
  float64_t term1,term2;
  
  x -= 1.0f;                                                                    /* Lanczos writes z! instead of Gamma(z) */

  for (k = 1; k <= 8; k++) { Ag += lanczos_7_c[k] / (x+k); }

  /* (x+0.5)*log(x+7.5) - (x+7.5) + LogRootTwoPi_ + log(Ag(x)) */
  term1 = (x + 0.5f) * log((x + 7.5f) / M_E);
  term2 = D_LOG_SQRT_2_PI + log(Ag);
  return term1 + (term2 - 7.0f);
#endif
}
// ripped from GSL (see above). Compute probability of getting a value
// k from a Poisson distribution with mean mu:
float64_t poisson(const int16_t k, const float64_t mu)
{
  float64_t lf = lngamma(k+1);
  return exp(log(mu) * k - lf - mu);
}

//computes the AUC, inspired by ROC, between two data sets
//the intended use is to compare human eye movements to a random
//model
float64_t AUC(const float64_t* model, const float64_t* rand, size_t sm, size_t sr, const float64_t step)
{
        //if ((max(model) > 1.0F) || (max(rand) > 1.0F)
        //|| (min(model) < 0.0F) || (min(rand) < 0.0F))
        //LFATAL("Vectors must be normalized");

    float64_t auc = 0.0f;
    float64_t last_tp = 1.0f;
    float64_t last_fp = 1.0f;
    float64_t thresh,fpr,tpr;
    uint8_t tp,fp;
    size_t jj;
    
    for (thresh = 0.0f; thresh <= 1.0f+step; thresh+=step)
    {
        tp = 0;
        fp = 0;
        for (jj = 0; jj < sm; ++jj)
            if (model[jj] >= thresh)
                tp++;

        for (jj = 0; jj < sr; ++jj)
            if (rand[jj] >= thresh)
                fp++;

        fpr = (float64_t)fp/(float64_t)sr;
        tpr = (float64_t)tp/(float64_t)sm;
        auc+= ( (last_fp - fpr) * (tpr) ) + ( 0.5f * (last_fp - fpr) * (last_tp - tpr) );
        last_tp = tpr;
        last_fp = fpr;
    }
    return auc;
}
/*-----------------------------------------------------------------------------
 *      inplaceAddBGnoise2 :  adds background noise 
 *                             
 *  Parameters: float32_t *src, const float32_t range, int16_t w, int16_t h
 *
 *  Return:
 *         float32_t *
 *----------------------------------------------------------------------------*/
#define BGNOISELEVEL 0.00001f                                                   // background noise level: as coeff of map full dynamic range:
float32_t * inplaceAddBGnoise2(float32_t *src, const float32_t range, int16_t w, int16_t h)
{
  int8_t j,i;
  int16_t siz = min(w, h) / 10;                                                 // do not put noise very close to image borders:
  float32_t *sptr = src + siz + siz * w;

  for (j = siz; j < h - siz; ++j)
  {
      for (i = siz; i < w - siz; ++i)
        *sptr++ += (range * (float64_t) rand() * BGNOISELEVEL);
      sptr += siz + siz;
  }
  return sptr;
}
/*-----------------------------------------------------------------------------
 *      uniformRandom :  uniform random float 
 *                             
 *  Parameters: void
 *
 *  Return:
 *         float32_t 
 *----------------------------------------------------------------------------*/
float64_t uniformRandom(void)
{
      return (float64_t) rand() / (float64_t) RAND_MAX;
}
/*-----------------------------------------------------------------------------
 *      gaussianRand :  gaussian random float 
 *                             
 *  Parameters: void
 *
 *  Return:
 *         float32_t 
 *----------------------------------------------------------------------------*/
float64_t gaussianRand(void)
{
      int16_t next_gaussian = 0;
      float64_t saved_gaussian_value=0.0f;

      float64_t fac, rsq, v1, v2;

      if (next_gaussian == 0) 
      {
        do {
          v1 = 2.0f*uniformRandom()-1.0f;
          v2 = 2.0f*uniformRandom()-1.0f;
          rsq = v1*v1+v2*v2;
        } while (rsq >= 1.0f || rsq == 0.0f);
        fac = sqrt(-2.0*log(rsq)/rsq);
        saved_gaussian_value=v1*fac;
        next_gaussian=1;
        return v2*fac;
      } 
      else 
      {
        next_gaussian=0;
        return saved_gaussian_value;
      }
}
/*-----------------------------------------------------------------------------
 *      ParticleFilter_getLikelihood :  Evaluate using a Gaussian distribution 
 *                             
 *  Parameters: const float64_t z, const float64_t X
 *
 *  Return:
 *         float32_t 
 *----------------------------------------------------------------------------*/
float64_t ParticleFilter_getLikelihood(const float64_t z, const float64_t X)
{
  float64_t val = X - z;
  float64_t sigma = 0.03f;

  return 1.0f/(sqrt(2.0f*PI) * sigma) * exp(-0.5f * (val*val / (sigma*sigma)));
}
//! Find the cosine of the angle between two vectors (p0,p1) and (p0,p2)
//  2D space is using the 3D vector but ignoring the z-plane 
float64_t AngleCosineFrom2DVectr(const Vectr p0, const Vectr p1, const Vectr p2)
{

  float64_t dx1 = p1.x - p0.x;
  float64_t dy1 = p1.y - p0.y;
  float64_t dx2 = p2.x - p0.x;
  float64_t dy2 = p2.y - p0.y;
  
  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10f);
}
/*-----------------------------------------------------------------------------
 *      Value_saturate :  Value saturate 
 *                             
 *  Parameters: float64_t A, const float64_t n
 *
 *  Return:
 *         float32_t 
 *----------------------------------------------------------------------------*/
float64_t Value_saturate(float64_t A, const float64_t n)
{
  if(A >= n) A = n;
  else A = A;
  return A;
}
 
/*-----------------------------------------------------------------------------
 *      qconjgAxB :  function compute the quaternion product conjg(qA) * qB
 *                             
 *  Parameters: const quat *pqA, const quat *pqB
 *
 *  Return:
 *         quat 
 *----------------------------------------------------------------------------*/
quat qconjgAxB(const quat *pqA, const quat *pqB)
{
  quat qProd;

  qProd.x = pqA->x * pqB->x + pqA->y * pqB->y + pqA->z * pqB->z + pqA->w * pqB->w;
  qProd.y = pqA->x * pqB->y - pqA->y * pqB->x - pqA->z * pqB->w + pqA->w * pqB->z;
  qProd.z = pqA->x * pqB->z + pqA->y * pqB->w - pqA->z * pqB->x - pqA->w * pqB->y;
  qProd.w = pqA->x * pqB->w - pqA->y * pqB->z + pqA->z * pqB->y - pqA->w * pqB->x;

  return qProd;
} 
/*-----------------------------------------------------------------------------
 *      fqAeq1 :  set a quaternion to the unit quaternion
 *                             
 *  Parameters: quat *pqA
 *
 *  Return:
 *         void 
 *----------------------------------------------------------------------------*/
void fqAeq1(quat *pqA) 
{
  pqA->x = 1.0F;
  pqA->x = pqA->x = pqA->x = 0.0F;
}
/*-----------------------------------------------------------------------------
 *      qAeqBxC :  function compute the quaternion product qA * qB
 *                             
 *  Parameters: quat *pqA, const quat *pqB, const quat *pqC
 *
 *  Return:
 *         void 
 *----------------------------------------------------------------------------*/
void qAeqBxC(quat *pqA, const quat *pqB, const quat *pqC) 
{
  pqA->x = pqB->x * pqC->x - pqB->y * pqC->y - pqB->z * pqC->z - pqB->w * pqC->w;
  pqA->y = pqB->x * pqC->y + pqB->y * pqC->x + pqB->z * pqC->w - pqB->w * pqC->z;
  pqA->z = pqB->x * pqC->z - pqB->y * pqC->w + pqB->z * pqC->x + pqB->w * pqC->y;
  pqA->w = pqB->x * pqC->w + pqB->y * pqC->z - pqB->z * pqC->y + pqB->w * pqC->x;
} 
/*-----------------------------------------------------------------------------
 *      fqAeqNormqA :  function normalizes a rotation quaternion and ensures 
 *                     q0 is non-negative
 *        
 *  Parameters: quat *pqA
 *
 *  Return:
 *         void 
 *----------------------------------------------------------------------------*/
void fqAeqNormqA(quat *pqA) 
{
  float32_t fNorm;                                                              // quaternion Norm

  fNorm = sqrt(pqA->x * pqA->x + pqA->y * pqA->y + pqA->z * pqA->z + pqA->w * pqA->w);    // calculate the quaternion Norm
  if (fNorm > CORRUPTQUAT) 
  {
    fNorm = 1.0F / fNorm;                                                       // general case
    pqA->x *= fNorm;
    pqA->y *= fNorm;
    pqA->z *= fNorm;
    pqA->w *= fNorm;
  } else 
  {
    pqA->x = 1.0F;                                                              // return with identity quaternion since the quaternion is corrupted
    pqA->y = pqA->z = pqA->w = 0.0F;
  }

  // correct a negative scalar component if the function was called with
  // negative x
  if (pqA->x < 0.0F) {
    pqA->x = -pqA->x;
    pqA->y = -pqA->y;
    pqA->z = -pqA->z;
    pqA->w = -pqA->w;
  }
}
/*-----------------------------------------------------------------------------
 *      mkvec : make vector
 *                             
 *  Parameters: float32_t x, float32_t y, float32_t z
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr mkvec(float32_t x, float32_t y, float32_t z)
{
        Vectr v;
        v.x = x; v.y = y; v.z = z;
        return v;
}
/*-----------------------------------------------------------------------------
 *      vadd : add two vectors.
 *                             
 *  Parameters: Vectr a, Vectr b
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vadd(Vectr a, Vectr b) 
{
   return mkvec(a.x + b.x, a.y + b.y, a.z + b.z);
}
/*-----------------------------------------------------------------------------
 *      vneg : negate a vector.
 *                             
 *  Parameters: Vectr v 
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vneg(Vectr v) 
{
    return mkvec(-v.x, -v.y, -v.z);
}
/*-----------------------------------------------------------------------------
 *      vsub : subtract a vector from another vector.
 *                             
 *  Parameters: Vectr a, Vectr b 
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vsub(Vectr a, Vectr b) 
{
    return vadd(a, vneg(b));
}
/*
 * matrix3.cpp  vector3.cpp
 * Copyright (C) Andrew Tridgell 2012
 * Ported by ACP Aviation (useful math functions from adupilot)
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 /*-----------------------------------------------------------------------------
 *  matrix33_det : 3x3 matrix determinate.
 *                             
 *  Parameters: mat33 a
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/
float32_t matrix33_det( mat33 a ) 
{
    return (a.m[0][0] * (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1])) + (a.m[0][1] * (a.m[1][2] * a.m[2][0] - a.m[1][0] * a.m[2][2])) + (a.m[0][2] * (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0]));
}
 /*-----------------------------------------------------------------------------
 *  matrix33_inverse : 3x3 matrix inverse.
 *                             
 *  Parameters: mat33 *inv, const mat33 a
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool matrix33_inverse(mat33 *inv, const mat33 a)
{
    float32_t d = matrix33_det(a);                                              /* determinate product */

    if ((d==0.0f) || (inv==NULL)) {
        return false;
    }

    inv->m[0][0] = (a.m[1][1] * a.m[2][2] - a.m[2][1] * a.m[1][2]) / d;
    inv->m[0][1] = (a.m[0][2] * a.m[2][1] - a.m[0][1] * a.m[2][2]) / d;
    inv->m[0][2] = (a.m[0][1] * a.m[1][2] - a.m[0][2] * a.m[1][1]) / d;
    inv->m[1][0] = (a.m[1][2] * a.m[2][0] - a.m[1][0] * a.m[2][2]) / d;
    inv->m[1][1] = (a.m[0][0] * a.m[2][2] - a.m[0][2] * a.m[2][0]) / d;
    inv->m[1][2] = (a.m[1][0] * a.m[0][2] - a.m[0][0] * a.m[1][2]) / d;
    inv->m[2][0] = (a.m[1][0] * a.m[2][1] - a.m[2][0] * a.m[1][1]) / d;
    inv->m[2][1] = (a.m[2][0] * a.m[0][1] - a.m[0][0] * a.m[2][1]) / d;
    inv->m[2][2] = (a.m[0][0] * a.m[1][1] - a.m[1][0] * a.m[0][1]) / d;

    return true;
}
 /*-----------------------------------------------------------------------------
 *  setRotationX_mat44 : set rotation X 4x4 matrix
 *                             
 *  Parameters: float64_t angle, mat44 *a
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void setRotationX_mat44(float64_t angle, mat44 *a)
{
        float64_t s = sin(angle);
        float64_t c = cos(angle);

        a->m[0u][0u]= 1.0f;
        a->m[1u][0u]= 0.0f;
        a->m[2u][0u]= 0.0f;
        a->m[3u][0u]= 0.0f;

        a->m[0u][1u]= 0.0f;
        a->m[1u][1u]= c;
        a->m[2u][1u]= s;
        a->m[3u][1u]= 0.0f;

        a->m[0u][2u]= 0.0f;
        a->m[1u][2u]= -s;
        a->m[2u][2u]= c;
        a->m[3u][2u]= 0.0f;

        a->m[0u][3u]= 0.0f;
        a->m[1u][3u]= 0.0f;
        a->m[2u][3u]= 0.0f;
        a->m[3u][3u]= 1.0f;

}
 /*-----------------------------------------------------------------------------
 *  setRotationY_mat44 : set rotation Y 4x4 matrix
 *                             
 *  Parameters: float64_t angle, mat44 *a
 *
 *  Return: void
 *----------------------------------------------------------------------------*/    
void setRotationY_mat44(float64_t angle, mat44 *a) 
{
        float64_t s = sin(angle);
        float64_t c = cos(angle);

        a->m[0u][0u]= c;
        a->m[1u][0u]= 0.0f;
        a->m[2u][0u]= -s;
        a->m[3u][0u]= 0.0f;

        a->m[0u][1u]= 0.0f;
        a->m[1u][1u]= 1.0f;
        a->m[2u][1u]= 0.0f;
        a->m[3u][1u]= 0.0f;

        a->m[0u][2u]= s;
        a->m[1u][2u]= 0.0f;
        a->m[2u][2u]= c;
        a->m[3u][2u]= 0.0f;

        a->m[0u][3u]= 0.0f;
        a->m[1u][3u]= 0.0f;
        a->m[2u][3u]= 0.0f;
        a->m[3u][3u]= 1.0f;

}
 /*-----------------------------------------------------------------------------
 *  setRotationZ_mat44 : set rotation Z 4x4 matrix
 *                             
 *  Parameters: float64_t angle, mat44 *a
 *
 *  Return: void
 *----------------------------------------------------------------------------*/ 
void setRotationZ_mat44(float64_t angle, mat44 *a) 
{
        float64_t s = sin(angle);
        float64_t c = cos(angle);

        a->m[0u][0u]= c;
        a->m[1u][0u]= s;
        a->m[2u][0u]= 0.0f;
        a->m[3u][0u]= 0.0f;

        a->m[0u][1u]= -s;
        a->m[1u][1u]= c;
        a->m[2u][1u]= 0.0f;
        a->m[3u][1u]= 0.0f;

        a->m[0u][2u]= 0.0f;
        a->m[1u][2u]= 0.0f;
        a->m[2u][2u]= 1.0f;
        a->m[3u][2u]= 0.0f;

        a->m[0u][3u]= 0.0f;
        a->m[1u][3u]= 0.0f;
        a->m[2u][3u]= 0.0f;
        a->m[3u][3u]= 1.0f;
}
 /*-----------------------------------------------------------------------------
 *  invertR_mat44 : invert 4x4 matrix
 *                             
 *  Parameters: const mat44 m, mat44 *mat
 *
 *  Return: void
 *----------------------------------------------------------------------------*/ 
void invertR_mat44(const mat44 m, mat44 *mat) 
{
        mat->m[0u][0u] = m.m[0u][0u]; 
        mat->m[1u][1u] = m.m[1u][1u]; 
        mat->m[2u][2u] = m.m[2u][2u];
        mat->m[0u][1u] = m.m[1u][0u]; 
        mat->m[1u][0u] = m.m[0u][1u];
        mat->m[0u][2u] = m.m[2u][0u]; 
        mat->m[2u][0u] = m.m[0u][2u];
        mat->m[1u][2u] = m.m[2u][1u]; 
        mat->m[2u][1u] = m.m[1u][2u];
        mat->m[0u][3u] = mat->m[1u][3u] = mat->m[2u][3u] = 0.0f;
        mat->m[3u][0u] = mat->m[3u][1u] = mat->m[3u][2u] = 0.0f; 
        mat->m[3u][3u] = 1.0f;
}
 /*-----------------------------------------------------------------------------
 *  invertRwot_mat44 : invert calculation for rotation matrix (without trans)
 *                             
 *  Parameters: mat44 *mat
 *
 *  Return: void
 *----------------------------------------------------------------------------*/ 
void invertRwot_mat44( mat44 *mat ) 
{
        float64_t tmp;
        tmp = mat->m[0u][1u]; mat->m[0u][1u] = mat->m[1u][0u]; mat->m[1u][0u] = tmp;
        tmp = mat->m[0u][2u]; mat->m[0u][2u] = mat->m[2u][0u]; mat->m[2u][0u] = tmp;
        tmp = mat->m[1u][2u]; mat->m[1u][2u] = mat->m[2u][1u]; mat->m[2u][1u] = tmp;
        // m03 = m13 = m23 = 0.0f;
        // m30 = m31 = m32 = 0.0f; 
        // m33 = 1.0f;
}
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
 /*-----------------------------------------------------------------------------
 *  Matrix33_from_euler : create a rotation matrix given some euler angles
 *                             
 *  Parameters: float32_t roll, float32_t pitch, float32_t yaw
 *
 *  Return: mat33
 *----------------------------------------------------------------------------*/ 
mat33 Matrix33_from_euler(float32_t roll, float32_t pitch, float32_t yaw)
{
    const float32_t cp = cos(pitch);
    const float32_t sp = sin(pitch);
    const float32_t sr = sin(roll);
    const float32_t cr = cos(roll);
    const float32_t sy = sin(yaw);
    const float32_t cy = cos(yaw);
    mat33 m;
    
    m.m[0][0] = cp * cy;
    m.m[0][1] = (sr * sp * cy) - (cr * sy);
    m.m[0][2] = (cr * sp * cy) + (sr * sy);
    m.m[1][0] = cp * sy;
    m.m[1][1] = (sr * sp * sy) + (cr * cy);
    m.m[1][2] = (cr * sp * sy) - (sr * cy);
    m.m[2][0] = -sp;
    m.m[2][1] = sr * cp;
    m.m[2][2]= cr * cp;
    return m;
}
/* ref : https://nghiaho.com/?page_id=846 */
/*-----------------------------------------------------------------------------
 *  matrix33_to_eulerAngle : 3x3 to euler angles vector
 *
 *
 *  Parameters: const mat33 mat, Vectr *theta
 *
 *  Return: mat33
 *----------------------------------------------------------------------------*/
void matrix33_to_eulerAngle( const mat33 mat, Vectr *theta )
{
    theta->x = atan2(mat.m[2u][1u], mat.m[2u][2u]);
    theta->y = atan2(mat.m[2u][0u], sqrt(pow(mat.m[2u][1u],2.0f) + pow(mat.m[2u][2u],2.0f)));
    theta->z = atan2(mat.m[1u][0u], mat.m[0u][0u]);          
}
 /*-----------------------------------------------------------------------------
 *  matrix44_det : 4x4 matrix determinate
 *                             
 *  Parameters: mat44 *det
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/ 
float32_t matrix44_det( mat44 *det )
{ 
    float32_t dst[4u];
        
    dst[0u] =                                                                     /* Compute adjoint: */
        + det->m[1u][1u] * det->m[2u][2u] * det->m[3u][3u]
        - det->m[1u][1u] * det->m[2u][3u] * det->m[3u][2u]
        - det->m[2u][1u] * det->m[1u][2u] * det->m[3u][3u]
        + det->m[2u][1u] * det->m[1u][3u] * det->m[3u][2u]
        + det->m[3u][1u] * det->m[1u][2u] * det->m[2u][3u]
        - det->m[3u][1u] * det->m[1u][3u] * det->m[2u][2u];

    dst[1u] =
        - det->m[1u][0u] * det->m[2u][2u] * det->m[3u][3u]
        + det->m[1u][0u] * det->m[2u][3u] * det->m[3u][2u]
        + det->m[2u][0u] * det->m[1u][2u] * det->m[3u][3u]
        - det->m[2u][0u] * det->m[1u][3u] * det->m[3u][2u]
        - det->m[3u][0u] * det->m[1u][2u] * det->m[2u][3u]
        + det->m[3u][0u] * det->m[1u][3u] * det->m[2u][2u];

    dst[2u] =
        + det->m[1u][0u] * det->m[2u][1u] * det->m[3][3]
        - det->m[1u][0u] * det->m[2u][3u] * det->m[3][1]
        - det->m[2u][0u] * det->m[1u][1u] * det->m[3][3]
        + det->m[2u][0u] * det->m[1u][3u] * det->m[3][1]
        + det->m[3u][0u] * det->m[1u][1u] * det->m[2][3]
        - det->m[3u][0u] * det->m[1u][3u] * det->m[2][1];

    dst[3u] =
        - det->m[1u][0u] * det->m[2u][1u] * det->m[3u][2u]
        + det->m[1u][0u] * det->m[2u][2u] * det->m[3u][1u]
        + det->m[2u][0u] * det->m[1u][1u] * det->m[3u][2u]
        - det->m[2u][0u] * det->m[1u][2u] * det->m[3u][1u]
        - det->m[3u][0u] * det->m[1u][1u] * det->m[2u][2u]
        + det->m[3u][0u] * det->m[1u][2u] * det->m[2u][1u];

    return (det->m[0u][0u] * dst[0u] + det->m[0u][1u] * dst[1u] + det->m[0u][2u] * dst[2u] + det->m[0u][3u] * dst[3u]);     /* Compute determinant: */
}
 /*-----------------------------------------------------------------------------
 *  matrix44_inverse : 4x4 matrix inverse
 *                             
 *  Parameters: mat44 *inv, const mat44 a
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool matrix44_inverse(mat44 *inv, const mat44 a)
{
    float32_t det;  
    if (inv==NULL) 
    {
        return false;
    }
        
    inv->m[0u][0u] =                                                            /* Compute adjoint: */
        + a.m[1u][1u] * a.m[2u][2u] * a.m[3u][3u]
        - a.m[1u][1u] * a.m[2u][3u] * a.m[3u][2u]
        - a.m[2u][1u] * a.m[1u][2u] * a.m[3u][3u]
        + a.m[2u][1u] * a.m[1u][3u] * a.m[3u][2u]
        + a.m[3u][1u] * a.m[1u][2u] * a.m[2u][3u]
        - a.m[3u][1u] * a.m[1u][3u] * a.m[2u][2u];

    inv->m[0u][1u] =
        - a.m[0u][1u] * a.m[2u][2u] * a.m[3u][3u]
        + a.m[0u][1u] * a.m[2u][3u] * a.m[3u][2u]
        + a.m[2u][1u] * a.m[0u][2u] * a.m[3u][3u]
        - a.m[2u][1u] * a.m[0u][3u] * a.m[3u][2u]
        - a.m[3u][1u] * a.m[0u][2u] * a.m[2u][3u]
        + a.m[3u][1u] * a.m[0u][3u] * a.m[2u][2u];

    inv->m[0u][2u] =
        + a.m[0u][1u] * a.m[1u][2u] * a.m[3u][3u]
        - a.m[0u][1u] * a.m[1u][3u] * a.m[3u][2u]
        - a.m[1u][1u] * a.m[0u][2u] * a.m[3u][3u]
        + a.m[1u][1u] * a.m[0u][3u] * a.m[3u][2u]
        + a.m[3u][1u] * a.m[0u][2u] * a.m[1u][3u]
        - a.m[3u][1u] * a.m[0u][3u] * a.m[1u][2u];

    inv->m[0u][3u] =
        - a.m[0u][1u] * a.m[1u][2u] * a.m[2u][3u]
        + a.m[0u][1u] * a.m[1u][3u] * a.m[2u][2u]
        + a.m[1u][1u] * a.m[0u][2u] * a.m[2u][3u]
        - a.m[1u][1u] * a.m[0u][3u] * a.m[2u][2u]
        - a.m[2u][1u] * a.m[0u][2u] * a.m[1u][3u]
        + a.m[2u][1u] * a.m[0u][3u] * a.m[1u][2u];

    inv->m[1u][0u] =
        - a.m[1u][0u] * a.m[2u][2u] * a.m[3u][3u]
        + a.m[1u][0u] * a.m[2u][3u] * a.m[3u][2u]
        + a.m[2u][0u] * a.m[1u][2u] * a.m[3u][3u]
        - a.m[2u][0u] * a.m[1u][3u] * a.m[3u][2u]
        - a.m[3u][0u] * a.m[1u][2u] * a.m[2u][3u]
        + a.m[3u][0u] * a.m[1u][3u] * a.m[2u][2u];

    inv->m[1u][1u] =
        + a.m[0u][0u] * a.m[2u][2u] * a.m[3u][3u]
        - a.m[0u][0u] * a.m[2u][3u] * a.m[3u][2u]
        - a.m[2u][0u] * a.m[0u][2u] * a.m[3u][3u]
        + a.m[2u][0u] * a.m[0u][3u] * a.m[3u][2u]
        + a.m[3u][0u] * a.m[0u][2u] * a.m[2u][3u]
        - a.m[3u][0u] * a.m[0u][3u] * a.m[2u][2u];

    inv->m[1u][2u] =
        - a.m[0u][0u] * a.m[1u][2u] * a.m[3u][3u]
        + a.m[0u][0u] * a.m[1u][3u] * a.m[3u][2u]
        + a.m[1u][0u] * a.m[0u][2u] * a.m[3u][3u]
        - a.m[1u][0u] * a.m[0u][3u] * a.m[3u][2u]
        - a.m[3u][0u] * a.m[0u][2u] * a.m[1u][3u]
        + a.m[3u][0u] * a.m[0u][3u] * a.m[1u][2u];

    inv->m[1u][3u] =
        + a.m[0u][0u] * a.m[1u][2u] * a.m[2u][3u]
        - a.m[0u][0u] * a.m[1u][3u] * a.m[2u][2u]
        - a.m[1u][0u] * a.m[0u][2u] * a.m[2u][3u]
        + a.m[1u][0u] * a.m[0u][3u] * a.m[2u][2u]
        + a.m[2u][0u] * a.m[0u][2u] * a.m[1u][3u]
        - a.m[2u][0u] * a.m[0u][3u] * a.m[1u][2u];

    inv->m[2u][0u] =
        + a.m[1u][0u] * a.m[2u][1u] * a.m[3u][3u]
        - a.m[1u][0u] * a.m[2u][3u] * a.m[3u][1u]
        - a.m[2u][0u] * a.m[1u][1u] * a.m[3u][3u]
        + a.m[2u][0u] * a.m[1u][3u] * a.m[3u][1u]
        + a.m[3u][0u] * a.m[1u][1u] * a.m[2u][3u]
        - a.m[3u][0u] * a.m[1u][3u] * a.m[2u][1u];

    inv->m[2u][1u] =
        - a.m[0u][0u] * a.m[2u][1u] * a.m[3u][3u]
        + a.m[0u][0u] * a.m[2u][3u] * a.m[3u][1u]
        + a.m[2u][0u] * a.m[0u][1u] * a.m[3u][3u]
        - a.m[2u][0u] * a.m[0u][3u] * a.m[3u][1u]
        - a.m[3u][0u] * a.m[0u][1u] * a.m[2u][3u]
        + a.m[3u][0u] * a.m[0u][3u] * a.m[2u][1u];

    inv->m[2u][2u] =
        + a.m[0u][0u] * a.m[1u][1u] * a.m[3u][3u]
        - a.m[0u][0u] * a.m[1u][3u] * a.m[3u][1u]
        - a.m[1u][0u] * a.m[0u][1u] * a.m[3u][3u]
        + a.m[1u][0u] * a.m[0u][3u] * a.m[3u][1u]
        + a.m[3u][0u] * a.m[0u][1u] * a.m[1u][3u]
        - a.m[3u][0u] * a.m[0u][3u] * a.m[1u][1u];

    inv->m[2u][3u] =
        - a.m[0u][0u] * a.m[1u][1u] * a.m[2u][3u]
        + a.m[0u][0u] * a.m[1u][3u] * a.m[2u][1u]
        + a.m[1u][0u] * a.m[0u][1u] * a.m[2u][3u]
        - a.m[1u][0u] * a.m[0u][3u] * a.m[2u][1u]
        - a.m[2u][0u] * a.m[0u][1u] * a.m[1u][3u]
        + a.m[2u][0u] * a.m[0u][3u] * a.m[1u][1u];

    inv->m[3u][0u] =
        - a.m[1u][0u] * a.m[2u][1u] * a.m[3u][2u]
        + a.m[1u][0u] * a.m[2u][2u] * a.m[3u][1u]
        + a.m[2u][0u] * a.m[1u][1u] * a.m[3u][2u]
        - a.m[2u][0u] * a.m[1u][2u] * a.m[3u][1u]
        - a.m[3u][0u] * a.m[1u][1u] * a.m[2u][2u]
        + a.m[3u][0u] * a.m[1u][2u] * a.m[2u][1u];

    inv->m[3u][1u] =
        + a.m[0u][0u] * a.m[2u][1u] * a.m[3u][2u]
        - a.m[0u][0u] * a.m[2u][2u] * a.m[3u][1u]
        - a.m[2u][0u] * a.m[0u][1u] * a.m[3u][2u]
        + a.m[2u][0u] * a.m[0u][2u] * a.m[3u][1u]
        + a.m[3u][0u] * a.m[0u][1u] * a.m[2u][2u]
        - a.m[3u][0u] * a.m[0u][2u] * a.m[2u][1u];

    inv->m[3u][2u] =
        - a.m[0u][0u] * a.m[1u][1u] * a.m[3u][2u]
        + a.m[0u][0u] * a.m[1u][2u] * a.m[3u][1u]
        + a.m[1u][0u] * a.m[0u][1u] * a.m[3u][2u]
        - a.m[1u][0u] * a.m[0u][2u] * a.m[3u][1u]
        - a.m[3u][0u] * a.m[0u][1u] * a.m[1u][2u]
        + a.m[3u][0u] * a.m[0u][2u] * a.m[1u][1u];

    inv->m[3u][3u] =
        + a.m[0u][0u] * a.m[1u][1u] * a.m[2u][2u]
        - a.m[0u][0u] * a.m[1u][2u] * a.m[2u][1u]
        - a.m[1u][0u] * a.m[0u][1u] * a.m[2u][2u]
        + a.m[1u][0u] * a.m[0u][2u] * a.m[2u][1u]
        + a.m[2u][0u] * a.m[0u][1u] * a.m[1u][2u]
        - a.m[2u][0u] * a.m[0u][2u] * a.m[1u][1u];

    det = a.m[0u][0u] * inv->m[0u][0u] + a.m[0u][1u] * inv->m[1u][0u] + a.m[0u][2u] * inv->m[2u][0u] + a.m[0u][3u] * inv->m[3u][0u];     /* Compute determinant: */
    if (det==0.0f) return false;
    det = 1.0f / det;                                                           /* Multiply adjoint with reciprocal of determinant: */

    inv->m[0u][0u] *= det;
    inv->m[0u][1u] *= det;
    inv->m[0u][2u] *= det;
    inv->m[0u][3u] *= det;
    inv->m[1u][0u] *= det;
    inv->m[1u][1u] *= det;
    inv->m[1u][2u] *= det;
    inv->m[1u][3u] *= det;
    inv->m[2u][0u] *= det;
    inv->m[2u][1u] *= det;
    inv->m[2u][2u] *= det;
    inv->m[2u][3u] *= det;
    inv->m[3u][0u] *= det;
    inv->m[3u][1u] *= det;
    inv->m[3u][2u] *= det;
    inv->m[3u][3u] *= det;
    return true;        
} 
/*-----------------------------------------------------------------------------
 *      vcross : vector cross product.
 *                             
 *  Parameters: Vectr a, Vectr b, 
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vcross(Vectr a, Vectr b) 
{
   Vectr vec;
   /* return mkvec(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x); */
   vec.x = a.y*b.z - a.z*b.y;
   vec.y = a.z*b.x - a.x*b.z;
   vec.z = a.x*b.y - a.y*b.x;
   return vec;
}       
/*-----------------------------------------------------------------------------
 *      mscl : multiply a matrix by a scalar.
 *
 *
 *  Parameters: float64_t s, mat33 a
 *
 *  Return:     mat33
 *----------------------------------------------------------------------------*/
mat33 mscl(float64_t s, mat33 a) 
{
        mat33 sa;
        int8_t i,j;
        
        for (i = 0; i < 3; ++i) 
        {
                for (j = 0; j < 3; ++j) 
                {
                        sa.m[i][j] = s * a.m[i][j];
                }
        }
        return sa;
}
/*-----------------------------------------------------------------------------
 *      m44scl : multiply a matrix by a scalar.
 *
 *
 *  Parameters: float64_t s, mat44 a
 *
 *  Return:     mat44
 *----------------------------------------------------------------------------*/
mat44 m44scl(float64_t s, mat44 a) 
{
        mat44 sa;
        int8_t i,j;
        
        for (i = 0; i < 4; ++i) 
        {
                for (j = 0; j < 4; ++j) 
                {
                        sa.m[i][j] = s * a.m[i][j];
                }
        }
        return sa;
}
/* ----------------------------------------------------------------------
     https://github.com/miyosuda/rodentia.git 
   ---------------------------------------------------------------------- */
void set_mat44( const dquat v0, const dquat v1, const dquat v2, const dquat v3, mat44 *a ) 
{
        a->m[0u][0u]=v0.x; 
        a->m[1u][0u]=v0.y; 
        a->m[2u][0u]=v0.z;
        a->m[3u][0u]=v0.w;
        a->m[0u][1u]=v1.x; 
        a->m[1u][1u]=v1.y; 
        a->m[2u][1u]=v1.z; 
        a->m[3u][1u]=v1.w;
        a->m[0u][2u]=v2.x; 
        a->m[1u][2u]=v2.y;
        a->m[2u][2u]=v2.z; 
        a->m[3u][2u]=v2.w;
        a->m[0u][3u]=v3.x;
        a->m[1u][3u]=v3.y; 
        a->m[2u][3u]=v3.z; 
        a->m[3u][3u]=v3.w;
}
 /*-----------------------------------------------------------------------------
 *  mul_mat44 : 4x4 matrix multiply
 *                             
 *  Parameters: const mat44 m0, const mat44 m1, mat44 *a
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void mul_mat44( const mat44 m0, const mat44 m1, mat44 *a ) 
{
    a->m[0u][0u]= m0.m[0u][0u]*m1.m[0u][0u] + m0.m[0u][1u]*m1.m[1u][0u] + m0.m[0u][2u]*m1.m[2u][0u] + m0.m[0u][3u]*m1.m[3u][0u];
    a->m[1u][0u]= m0.m[1u][0u]*m1.m[0u][0u] + m0.m[1u][1u]*m1.m[1u][0u] + m0.m[1u][2u]*m1.m[2u][0] + m0.m[1u][3u]*m1.m[3u][0u];
    a->m[2u][0u]= m0.m[2u][0u]*m1.m[0u][0u] + m0.m[2u][1u]*m1.m[1u][0u] + m0.m[2u][2u]*m1.m[2u][0] + m0.m[2u][3u]*m1.m[3u][0u];
    a->m[3u][0u]= m0.m[3u][0u]*m1.m[0u][0u] + m0.m[3u][1u]*m1.m[1u][0u] + m0.m[3u][2u]*m1.m[2u][0u] + m0.m[3u][3u]*m1.m[3u][0u];

    a->m[0u][1u]= m0.m[0u][0u]*m1.m[0u][1u] + m0.m[0u][1u]*m1.m[1u][1u] + m0.m[0u][2u]*m1.m[2u][1u] + m0.m[0u][3u]*m1.m[3u][1u];
    a->m[1u][1u]= m0.m[1u][0u]*m1.m[0u][1u] + m0.m[1u][1u]*m1.m[1u][1u] + m0.m[1u][2u]*m1.m[2u][1u] + m0.m[1u][3u]*m1.m[3u][1u];
    a->m[2u][1u]= m0.m[2u][0u]*m1.m[0u][1u] + m0.m[2u][1u]*m1.m[1u][1u] + m0.m[2u][2u]*m1.m[2u][1u] + m0.m[2u][3u]*m1.m[3u][1u];
    a->m[3u][1u]= m0.m[3u][0u]*m1.m[0u][1u] + m0.m[3u][1u]*m1.m[1u][1u] + m0.m[3u][2u]*m1.m[2u][1u] + m0.m[3u][3u]*m1.m[3u][1u];

    a->m[0u][2u]= m0.m[0u][0]*m1.m[0u][2u] + m0.m[0u][1u]*m1.m[1u][2u] + m0.m[0u][2u]*m1.m[2u][2u] + m0.m[0u][3u]*m1.m[3u][2u];
    a->m[1u][2u]= m0.m[1u][0]*m1.m[0u][2u] + m0.m[1u][1u]*m1.m[1u][2u] + m0.m[1u][2u]*m1.m[2u][2u] + m0.m[1u][3u]*m1.m[3u][2u];
    a->m[2u][2u]= m0.m[2u][0]*m1.m[0u][2u] + m0.m[2u][1u]*m1.m[1u][2u] + m0.m[2u][2u]*m1.m[2u][2u] + m0.m[2u][3u]*m1.m[3u][2u];
    a->m[3u][2u]= m0.m[3u][0]*m1.m[0u][2u] + m0.m[3u][1u]*m1.m[1u][2u] + m0.m[3u][2u]*m1.m[2u][2u] + m0.m[3u][3u]*m1.m[3u][2u];

    a->m[0u][3u]= m0.m[0u][0u]*m1.m[0u][3u] + m0.m[0u][1u]*m1.m[1u][3u] + m0.m[0u][2u]*m1.m[2u][3u] + m0.m[0u][3u]*m1.m[3u][3u];
    a->m[1u][3u]= m0.m[1u][0u]*m1.m[0u][3u] + m0.m[1u][1u]*m1.m[1u][3u] + m0.m[1u][2u]*m1.m[2u][3u] + m0.m[1u][3u]*m1.m[3u][3u];
    a->m[2u][3u]= m0.m[2u][0u]*m1.m[0u][3u] + m0.m[2u][1u]*m1.m[1u][3u] + m0.m[2u][2u]*m1.m[2u][3u] + m0.m[2u][3u]*m1.m[3u][3u];
    a->m[3u][3u]= m0.m[3u][0u]*m1.m[0u][3u] + m0.m[3u][1u]*m1.m[1u][3u] + m0.m[3u][2u]*m1.m[2u][3u] + m0.m[3u][3u]*m1.m[3u][3u];
}
/*-----------------------------------------------------------------------------
 *      mNscl : multiply a symetrical matrix NxN by a scalar.
 *
 *
 *  Parameters: float64_t s, float64_t * a[], float64_t * res[], uint16_t nEle
 *
 *  Return:     float64_t *
 *----------------------------------------------------------------------------*/
float64_t * mNscl(float64_t s, float64_t * a, float64_t * res, uint16_t nEle) 
{
        int8_t i,j;
        uint16_t n, size;
        
        size = nEle * nEle * sizeof(float64_t);
        n = size / sizeof(float64_t);
        
        for (i = 0; i < n; i=i+sizeof(float64_t)) 
        {
           for (j = 0; j < n; j=j+sizeof(float64_t)) 
           {
              *(res+i+j) = (*(a+i+j)) * s;
           }
        }
        return res;
}
/*-----------------------------------------------------------------------------
 *      mNdiv : divide a symetrical matrix NxN by a scalar.
 *
 *
 *  Parameters: float64_t s, float64_t * a[], float64_t * res[], uint16_t nEle
 *
 *  Return:     float64_t *
 *----------------------------------------------------------------------------*/
float64_t * mNdiv(float64_t s, float64_t * a, float64_t * res, uint16_t nEle) 
{
        int8_t i,j;
        uint16_t n, size;
        
        size = nEle * nEle * sizeof(float64_t);
        n = size / sizeof(float64_t);
        
        for (i = 0; i < n; i=i+sizeof(float64_t)) 
        {
           for (j = 0; j < n; j=j+sizeof(float64_t)) 
           {
              *(res+i+j) = (*(a+i+j)) / s;
           }
        }
        return res;
}
/*-----------------------------------------------------------------------------
 *      mNadd : add a scalar to each element of a symetrical matrix NxN by 
 *
 *
 *  Parameters: float64_t s, float64_t * a[], float64_t * res[], uint16_t nEle
 *
 *  Return:     float64_t *
 *----------------------------------------------------------------------------*/
float64_t * mNadd(float64_t s, float64_t * a, float64_t * res, uint16_t nEle) 
{
        int8_t i,j;
        uint16_t n, size;
        
        size = nEle * nEle * sizeof(float64_t);
        n = size / sizeof(float64_t);
        
        for (i = 0; i < n; i=i+sizeof(float64_t)) 
        {
           for (j = 0; j < n; j=j+sizeof(float64_t)) 
           {
              *(res+i+j) = (*(a+i+j)) + s;
           }
        }
        return res;
}
/*-----------------------------------------------------------------------------
 *      mNsub : subtract a scalar to each element of a symetrical matrix NxN  
 *
 *
 *  Parameters: float64_t s, float64_t * a[], float64_t * res[], uint16_t nEle
 *
 *  Return:     float64_t *
 *----------------------------------------------------------------------------*/
float64_t * mNsub(float64_t s, float64_t * a, float64_t * res, uint16_t nEle) 
{
        int8_t i,j;
        uint16_t n, size;
        
        size = nEle * nEle * sizeof(float64_t);
        n = size / sizeof(float64_t);
        
        for (i = 0; i < n; i=i+sizeof(float64_t)) 
        {
           for (j = 0; j < n; j=j+sizeof(float64_t)) 
           {
              *(res+i+j) = (*(a+i+j)) - s;
           }
        }
        return res;
}
/* these come from miyosuda rodentia project on github 
   https://github.com/miyosuda/rodentia/blob/master/src/geom/Matrix4f.h */
 /*-----------------------------------------------------------------------------
 *  Matrix4_setIdentity : set identity for 4x4 matrix
 *                             
 *  Parameters: none
 *
 *  Return: Matrix4d64_t
 *----------------------------------------------------------------------------*/
Matrix4d64_t Matrix4_setIdentity() 
{
        Matrix4d64_t a;
        
        a.b.x=a.c.x=a.d.x = 0.0f;
        a.a.y=a.c.y=a.d.y = 0.0f;
        a.a.z=a.b.z=a.d.z = 0.0f;
        a.a.w=a.b.w=a.c.w = 0.0f;     
        a.a.x=a.b.y=a.c.z=a.d.w = 1.0f;
        return a;
}
/*-----------------------------------------------------------------------------
 *   setRowMat4f64: set row with quat in 4x4 matrix
 *
 *  Parameters: uint16_t row, dquat *v, Matrix4d64_t *m00
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void setRowMat4f64(uint16_t row, const dquat v, Matrix4d64_t *m00) 
{
        float64_t p[16u]; 
        memcpy((void*)&p,(void*)m00,sizeof(p));
        p[row     ] = v.x;
        p[row + 4 ] = v.y;
        p[row + 8 ] = v.z;
        p[row + 12] = v.w;
        memcpy((void*)m00,(void*)&p,sizeof(p));
}

/*-----------------------------------------------------------------------------
 *   setColumnMat4f64: set column with quat in 4x4 matrix
 *
 *  Parameters: uint16_t row, dquat *v, Matrix4d64_t *m00
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void setColumnMat4f64(uint16_t column, const quat v, Matrix4d64_t *m00) 
{
        memcpy((void*)m00+(4 * column),(void*)&v,sizeof(v));
} 
/*-----------------------------------------------------------------------------
 *   getRowMat4f64: return column of 4x4 matrix as a quat
 *
 *  Parameters: uint16_t row, dquat *v, Matrix4d64_t *m00
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void getRowMat4f64(uint16_t row, dquat *v, Matrix4d64_t *m00)  
{
        float64_t p[16u];
        memcpy((void*)&p,(void*)m00,16*sizeof(float64_t));
        v->x = p[row     ];
        v->y = p[row + 4 ];
        v->z = p[row + 8 ];
        v->w = p[row + 12];
}
/*-----------------------------------------------------------------------------
 *   getColumnMat4f64: return column of 4x4 matrix as a quat
 *
 *  Parameters: uint16_t column, dquat *v, Matrix4d64_t *m00
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void getColumnMat4f64(uint16_t column, dquat *v, Matrix4d64_t *m00)  
{
        float64_t pbase[16u];

        memcpy((void*)&pbase,(void*)m00,(16*sizeof(float64_t)));       
        v->x = pbase[0u+ 4 * column];
        v->y = pbase[1u+ 4 * column];
        v->z = pbase[2u+ 4 * column];
        v->w = pbase[3u+ 4 * column];
} 
/*-----------------------------------------------------------------------------
 *   Matrix3_det: matrix 3 determinate 3x3 double precision
 *
 *  Parameters: const Matrix3f64_t a
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t Matrix3_det( const Matrix3f64_t a ) 
{
    return a.a.x * (a.b.y * a.c.z - a.b.z * a.c.y) +  a.a.y * (a.b.z * a.c.x - a.b.x * a.c.z) + a.a.z * (a.b.x * a.c.y - a.b.y * a.c.x);
}
/*-----------------------------------------------------------------------------
 *   Matrix4_det: matrix 4 determinate 4x4 double precision
 *
 *  Parameters: const Matrix4d64_t a
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t Matrix4_det( const Matrix4d64_t a )
{ 
    float64_t dst[4u];
        
    dst[0u] =                                                                     /* Compute adjoint: */
        + a.b.y * a.c.z * a.d.w
        - a.b.y * a.c.w * a.d.z
        - a.c.y * a.b.z * a.d.w
        + a.c.y * a.b.w * a.d.z
        + a.d.y * a.b.z * a.c.w
        - a.d.y * a.b.w * a.c.z;

    dst[1u] =
        - a.b.x * a.c.z * a.d.w
        + a.b.x * a.c.w * a.d.z
        + a.c.x * a.b.z * a.d.w
        - a.c.x * a.b.w * a.d.z
        - a.d.x * a.b.z * a.c.w
        + a.d.x * a.b.w * a.c.z;

    dst[2u] =
        + a.b.x * a.c.y * a.d.w
        - a.b.x * a.c.w * a.d.y
        - a.c.x * a.b.y * a.d.w
        + a.c.x * a.b.w * a.d.y
        + a.d.x * a.b.y * a.c.w
        - a.d.x * a.b.w * a.b.y;

    dst[3u] =
        - a.b.x * a.c.y * a.d.z
        + a.b.x * a.c.z * a.d.y
        + a.c.x * a.b.y * a.d.z
        - a.c.x * a.b.z * a.d.y
        - a.d.x * a.b.y * a.c.z
        + a.d.x * a.b.z * a.c.y;

    return (a.a.x * dst[0u] + a.a.y * dst[1u] + a.a.z * dst[2u] + a.a.w * dst[3u]);     /* Compute determinant: */
}
/* These converters convert from mat33 sphere to Matrix3f64 (double precision)
   consider replicating or rationalising (for speed) but this converts from mat33 
   float arrays to double precison Matrix3f64 and uses the ardupilot format 
   in double for greatest accuracy and vice versa */
/*-----------------------------------------------------------------------------
 *   Matrix3f64_to_matrix33: format convert double Matrix3f64_t 3x3 matrix to mat33
 *
 *  Parameters: mat33 *mtx, const Matrix3f64_t a
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/   
bool Matrix3f64_to_matrix33(mat33 *mtx, const Matrix3f64_t a)
{
    if (mtx==NULL) {
        return false;
    }

    mtx->m[0][0] = (float32_t)a.a.x;
    mtx->m[0][1] = (float32_t)a.a.y;
    mtx->m[0][2] = (float32_t)a.a.z;
    mtx->m[1][0] = (float32_t)a.b.x;
    mtx->m[1][1] = (float32_t)a.b.y;
    mtx->m[1][2] = (float32_t)a.b.z;
    mtx->m[2][0] = (float32_t)a.c.x;
    mtx->m[2][1] = (float32_t)a.c.y;
    mtx->m[2][2] = (float32_t)a.c.z;

    return true;
}
/*-----------------------------------------------------------------------------
 *   matrix33_to_Matrix3f64: format convert mat33 3x3 matrix to double Matrix3f64_t
 *
 *  Parameters: const mat33 mtx, Matrix3f64_t *a
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool matrix33_to_Matrix3f64(const mat33 mtx, Matrix3f64_t *a)
{
    if (a==NULL) {
        return false;
    }

    a->a.x = (float64_t)mtx.m[0][0];
    a->a.y = (float64_t)mtx.m[0][1];
    a->a.z = (float64_t)mtx.m[0][2];
    a->b.x = (float64_t)mtx.m[1][0];
    a->b.y = (float64_t)mtx.m[1][1];
    a->b.z = (float64_t)mtx.m[1][2];
    a->c.x = (float64_t)mtx.m[2][0];
    a->c.y = (float64_t)mtx.m[2][1];
    a->c.z = (float64_t)mtx.m[2][2];

    return true;
}
/*-----------------------------------------------------------------------------
 *   Quaternion_inverse: return the reverse rotation of this quaternion
 *
 *  Parameters: const dquat q
 *
 *  Return: dquat
 *----------------------------------------------------------------------------*/
dquat Quaternion_inverse(const dquat q) 
{
    dquat ret;
    ret.x = q.x;
    ret.y = -q.y;
    ret.z = -q.z;
    ret.w = -q.w;
    return ret;
}
/*-----------------------------------------------------------------------------
 *   Quaternion_length: quat length 
 *
 *  Parameters: const dquat q
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t Quaternion_length(const dquat q) 
{
    return sqrt((q.x*q.x) + (q.y*q.y) + (q.z*q.z) + (q.w*q.w));
}
/*-----------------------------------------------------------------------------
 *   Vectr_length: vector length 
 *
 *  Parameters: const dVectr q
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t Vectr_length(const dVectr q) 
{
    return sqrt((q.x*q.x) + (q.y*q.y) + (q.z*q.z));
}
// return the rotation matrix equivalent for this quaternion
// Thanks to Martin John Baker
// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
/*-----------------------------------------------------------------------------
 *   Quaternion_from_rotation_matrix: convert 3x3 matrix 
 *                                    to quaternion 
 *
 *  Parameters: const Matrix3f64_t m
 *
 *  Return: quat
 *----------------------------------------------------------------------------*/
quat Quaternion_from_rotation_matrix(const Matrix3f64_t m)
{
    const float64_t m00 = m.a.x;
    const float64_t m11 = m.b.y;
    const float64_t m22 = m.c.z;
    const float64_t m10 = m.b.x;
    const float64_t m01 = m.a.y;
    const float64_t m20 = m.c.x;
    const float64_t m02 = m.a.z;
    const float64_t m21 = m.c.y;
    const float64_t m12 = m.b.z;
    const float64_t tr = m00 + m11 + m22;
    quat q;
    float64_t S;
        
    if (tr > 0.0f) {
        S = sqrt(tr+1.0f) * 2.0f;
        q.w = 0.25f * S;
        q.x = (m21 - m12) / S;
        q.y = (m02 - m20) / S;
        q.z = (m10 - m01) / S;
    } else if ((m00 > m11) && (m00 > m22)) {
        S = sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q.w = (m21 - m12) / S;
        q.x = 0.25f * S;
        q.y = (m01 + m10) / S;
        q.z = (m02 + m20) / S;
    } else if (m11 > m22) {
        S = sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q.w = (m02 - m20) / S;
        q.x = (m01 + m10) / S;
        q.y = 0.25f * S;
        q.z = (m12 + m21) / S;
    } else {
        S = sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q.w = (m10 - m01) / S;
        q.x = (m02 + m20) / S;
        q.y = (m12 + m21) / S;
        q.z = 0.25f * S;
    }
    return q;
}
/*-----------------------------------------------------------------------------
 *   Quaternion_from_rotation_matrix2: ROTATE 3x3 matrix 
 *                                    by quaternion 
 *
 *  Parameters: const Matrix3f64_t m, quat *q
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Quaternion_from_rotation_matrix2(const Matrix3f64_t m, quat *q)
{
    const float64_t m00 = m.a.x;
    const float64_t m11 = m.b.y;
    const float64_t m22 = m.c.z;
    const float64_t m10 = m.b.x;
    const float64_t m01 = m.a.y;
    const float64_t m20 = m.c.x;
    const float64_t m02 = m.a.z;
    const float64_t m21 = m.c.y;
    const float64_t m12 = m.b.z;
    const float64_t tr = m00 + m11 + m22;
    float64_t S;

    if (q==NULL) return false;
            
    if (tr > 0.0f) {
        S = sqrt(tr+1.0f) * 2.0f;
        q->w = 0.25f * S;
        q->x = (m21 - m12) / S;
        q->y = (m02 - m20) / S;
        q->z = (m10 - m01) / S;
    } else if ((m00 > m11) && (m00 > m22)) {
        S = sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q->w = (m21 - m12) / S;
        q->x = 0.25f * S;
        q->y = (m01 + m10) / S;
        q->z = (m02 + m20) / S;
    } else if (m11 > m22) {
        S = sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q->w = (m02 - m20) / S;
        q->x = (m01 + m10) / S;
        q->y = 0.25f * S;
        q->z = (m12 + m21) / S;
    } else {
        S = sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q->w = (m10 - m01) / S;
        q->x = (m02 + m20) / S;
        q->y = (m12 + m21) / S;
        q->z = 0.25f * S;
    }
    return true;
}
/*-----------------------------------------------------------------------------
 *   Quaternion_rotation_matrix_norm: return the rotation matrix equivalent for 
 *                                    this quaternion after normalization
 *
 *  Parameters: Matrix3f64_t *m, const dquat q
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Quaternion_rotation_matrix_norm(Matrix3f64_t *m, const dquat q) 
{
    const float64_t q1q1 = q.x * q.x;
    const float64_t q1q2 = q.x * q.y;
    const float64_t q1q3 = q.x * q.z;
    const float64_t q1q4 = q.x * q.w;
    const float64_t q2q2 = q.y * q.y;
    const float64_t q2q3 = q.y * q.z;
    const float64_t q2q4 = q.y * q.w;
    const float64_t q3q3 = q.z * q.z;
    const float64_t q3q4 = q.z * q.w;
    const float64_t q4q4 = q.w * q.w;
    float64_t invs;
    
    if (m==NULL) return false;
    if ((q1q1 + q2q2 + q3q3 + q4q4) != 0.0f)
      invs = 1.0f / (q1q1 + q2q2 + q3q3 + q4q4);
    else
      invs = 0.0f;        
    m->a.x = ( q2q2 - q3q3 - q4q4 + q1q1)*invs;
    m->a.y = 2.0f*(q2q3 - q1q4)*invs;
    m->a.z = 2.0f*(q2q4 + q1q3)*invs;
    m->b.x = 2.0f*(q2q3 + q1q4)*invs;
    m->b.y = (-q2q2 + q3q3 - q4q4 + q1q1)*invs;
    m->b.z = 2.0f*(q3q4 - q1q2)*invs;
    m->c.x = 2.0f*(q2q4 - q1q3)*invs;
    m->c.y = 2.0f*(q3q4 + q1q2)*invs;
    m->c.z = (-q2q2 - q3q3 + q4q4 + q1q1)*invs;
    return true;
}
 
/*-----------------------------------------------------------------------------
 *   Quaternion_rotation_matrix : return the rotation matrix equivalent for this quaternion 
 *
 *  Parameters: Matrix3f64_t *m, const dquat q
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Quaternion_rotation_matrix(Matrix3f64_t *m, const dquat q) 
{
    const float64_t q3q3 = q.z * q.z;
    const float64_t q3q4 = q.z * q.w;
    const float64_t q2q2 = q.y * q.y;
    const float64_t q2q3 = q.y * q.z;
    const float64_t q2q4 = q.y * q.w;
    const float64_t q1q2 = q.x * q.y;
    const float64_t q1q3 = q.x * q.z;
    const float64_t q1q4 = q.x * q.w;
    const float64_t q4q4 = q.w * q.w;

    if (m==NULL) return false;
    m->a.x = 1.0f-2.0f*(q3q3 + q4q4);
    m->a.y = 2.0f*(q2q3 - q1q4);
    m->a.z = 2.0f*(q2q4 + q1q3);
    m->b.x = 2.0f*(q2q3 + q1q4);
    m->b.y = 1.0f-2.0f*(q2q2 + q4q4);
    m->b.z = 2.0f*(q3q4 - q1q2);
    m->c.x = 2.0f*(q2q4 - q1q3);
    m->c.y = 2.0f*(q3q4 + q1q2);
    m->c.z = 1.0f-2.0f*(q2q2 + q3q3);
    return true;
}
/*-----------------------------------------------------------------------------
 *  Quaternion_from_euler: create a quaternion from Euler angles 
 *
 *  Parameters: float64_t roll, float64_t pitch, float64_t yaw, dquat *q
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Quaternion_from_euler(float64_t roll, float64_t pitch, float64_t yaw, dquat *q)
{
    const float64_t cr2 = cos(roll*0.5f);
    const float64_t cp2 = cos(pitch*0.5f);
    const float64_t cy2 = cos(yaw*0.5f);
    const float64_t sr2 = sin(roll*0.5f);
    const float64_t sp2 = sin(pitch*0.5f);
    const float64_t sy2 = sin(yaw*0.5f);
    if (q==NULL) return false;
    q->x = cr2*cp2*cy2 + sr2*sp2*sy2;
    q->y = sr2*cp2*cy2 - cr2*sp2*sy2;
    q->z = cr2*sp2*cy2 + sr2*cp2*sy2;
    q->w = cr2*cp2*sy2 - sr2*sp2*cy2;
    return true;
}
/*-----------------------------------------------------------------------------
 *  Quaternion_from_axis_angle: create a quaternion from its axis-angle representation 
 *                              the axis vector must be length 1, theta is in radians
 *
 *  Parameters: const dVectr axis, float64_t theta, dquat *q
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Quaternion_from_axis_angle(const dVectr axis, float64_t theta, dquat *q)
{
    const float64_t st2 = sin(theta/2.0f);
    if (q==NULL) return false;
    if (theta==0.0f) {                                                         // axis must be a unit vector as there is no check for length
        q->x = 1.0f;
        q->y=q->z=q->w=0.0f;
    } else {
      q->x = cos(theta/2.0f);
      q->y = axis.x * st2;
      q->z = axis.y * st2;
      q->w = axis.z * st2;
    }
    return true;
}
/*-----------------------------------------------------------------------------
 *      dquatDiv : divide vector q by div 
 *
 *  Parameters: const dVectr q, float64_t div
 *
 *  Return: dquat
 *----------------------------------------------------------------------------*/
dquat dquatDiv( const dquat q, float64_t div )
{
    dquat resl;
    resl.x = q.x / div;
    resl.y = q.y / div;
    resl.z = q.z / div;
    resl.w = q.w / div;
    return resl;        
}
/*-----------------------------------------------------------------------------
 *      dvecDiv : divide vector q by div 
 *
 *  Parameters: const dVectr q, float64_t div
 *
 *  Return: dVectr
 *----------------------------------------------------------------------------*/
dVectr dvecDiv( const dVectr q, float64_t div )
{
    dVectr resl;
    resl.x = q.x / div;
    resl.y = q.y / div;
    resl.z = q.z / div;
    return resl;        
}
/*-----------------------------------------------------------------------------
 *      Quaternion_from_axis_angle2 : create a quaternion from its axis-angle representation 
 *
 *  Parameters: dVectr *v, dquat *q, const float64_t theta
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Quaternion_from_axis_angle2(dVectr *v, dquat *q, const float64_t theta)
{
    if ((q==NULL)||(v==NULL)) return false;
    if (theta==0.0f) {
        q->x = 1.0f;
        q->y=q->z=q->w=0.0f;
        return true;
    }
    *v = dvecDiv(*v,theta);
    if (Quaternion_from_axis_angle(*v,theta,q)==true)
       return true;
    else
       return false;
} 
/*-----------------------------------------------------------------------------
 *      Quaternion_from_axis_angle3 : create a quaternion from its axis-angle representation 
 *
 *  Parameters: dVectr *v, dquat *q
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Quaternion_from_axis_angle3(dVectr *v, dquat *q)
{
    const float64_t theta = Vectr_length(*v); 
    if ((q==NULL)||(v==NULL)) return false;      
    if (theta==0.0f) {
        q->x = 1.0f;
        q->y=q->z=q->w=0.0f;
        return true;
    }
    *v = dvecDiv(*v,theta);
    if (Quaternion_from_axis_angle(*v,theta,q)==true)
       return true;
    else
       return false;    
} 
/*-----------------------------------------------------------------------------
 *      Quaternion_get_euler_roll : get euler roll angle 
 *
 *  Parameters: const dquat q
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t Quaternion_get_euler_roll(const dquat q) 
{
    return (atan2(2.0f*(q.x*q.y + q.z*q.w), 1.0f - 2.0f*(q.y*q.y + q.z*q.z)));
} 
/*-----------------------------------------------------------------------------
 *      Quaternion_get_euler_pitch : get euler pitch angle 
 *
 *  Parameters: const dquat q
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t Quaternion_get_euler_pitch(const dquat q)
{
     float64_t ret;
     IKsin(ret,(2.0f*(q.x*q.z - q.w*q.y)));
     return ret;
}
/*-----------------------------------------------------------------------------
 *      Quaternion_get_euler_yaw : get euler yaw angle 
 *
 *  Parameters: const dquat q
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t Quaternion_get_euler_yaw(const dquat q) 
{
    return atan2(2.0f*(q.x*q.w + q.y*q.z), 1.0f - 2.0f*(q.z*q.z + q.w*q.w));
} 
/*-----------------------------------------------------------------------------
 *      Quaternion_mul : multiply 2 quaternion 
 *
 *  Parameters: const dquat v,const dquat q
 *
 *  Return: dquat
 *----------------------------------------------------------------------------*/
dquat Quaternion_mul(const dquat v,const dquat q)
{
    const float64_t w1 = q.x;
    const float64_t x1 = q.y;
    const float64_t y1 = q.z;
    const float64_t z1 = q.w;

    const float64_t w2 = v.x;
    const float64_t x2 = v.y;
    const float64_t y2 = v.z;
    const float64_t z2 = v.w;
    dquat ret;
        
    ret.x = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    ret.y = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    ret.z = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    ret.w = w1*z2 + x1*y2 - y1*x2 + z1*w2;

    return ret;
} 
/*-----------------------------------------------------------------------------
 *      Quaternion_div : divide 2 quaternion 
 *
 *  Parameters: const dquat q, const dquat v
 *
 *  Return: dquat
 *----------------------------------------------------------------------------*/
dquat Quaternion_div(const dquat q, const dquat v) 
{
    dquat ret;
    const float64_t quat0 = q.x;
    const float64_t quat1 = q.y;
    const float64_t quat2 = q.z;
    const float64_t quat3 = q.w;

    const float64_t rquat0 = v.x;
    const float64_t rquat1 = v.y;
    const float64_t rquat2 = v.z;
    const float64_t rquat3 = v.w;

    ret.x = (rquat0*quat0 + rquat1*quat1 + rquat2*quat2 + rquat3*quat3);
    ret.y = (rquat0*quat1 - rquat1*quat0 - rquat2*quat3 + rquat3*quat2);
    ret.z = (rquat0*quat2 + rquat1*quat3 - rquat2*quat0 - rquat3*quat1);
    ret.w = (rquat0*quat3 - rquat1*quat2 + rquat2*quat1 - rquat3*quat0);
    return ret;
}
/*-----------------------------------------------------------------------------
 *      Quaternion_rotate : rotate by the provided axis angle 
 *
 *  Parameters: const dVectr *v, dquat *q
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Quaternion_rotate(const dVectr *v, dquat *q)
{
    dquat r = {0.0f,0.0f,0.0f,0.0f};
    if ((q==NULL) || (v==NULL)) return false;
    Quaternion_from_axis_angle3(v, &r);                                          
    *q = Quaternion_mul(*q, r);
    return true;
} 
/*-----------------------------------------------------------------------------
 *      Quaternion_to_euler : create eulers from a quaternion. 
 *
 *  Parameters: float64_t *roll, float64_t *pitch, float64_t *yaw, const dquat q
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Quaternion_to_euler(float64_t *roll, float64_t *pitch, float64_t *yaw, const dquat q) 
{
    if ((roll==NULL) || ((pitch==NULL) || (yaw==NULL))) return false;
    *roll = Quaternion_get_euler_roll(q);  
    *pitch = Quaternion_get_euler_pitch(q);
    *yaw = Quaternion_get_euler_yaw(q);
    return true;
}
/*
  See http://www.atacolorado.com/eulersequences.doc 
*/
/*-----------------------------------------------------------------------------
 *      Vector3_to_euler312 : calculate Euler angles (312 convention) for the matrix. 
 *
 *  Parameters: const Matrix3f64_t m 
 *
 *  Return: dVectr   vector is returned in r, p, y order
 *----------------------------------------------------------------------------*/
dVectr Vector3_to_euler312( const Matrix3f64_t m ) 
{
    dVectr ret;
    ret.x = asin(m.c.y);
    ret.y = atan2(-m.c.x, m.c.z);
    ret.z = atan2(-m.a.y, m.b.y);
    return ret;
}
/*-----------------------------------------------------------------------------
 *      Matrix3_from_euler312 : fill the matrix from Euler angles in radians in 312 convention 
 *
 *  Parameters: float64_t roll, float64_t pitch, float64_t yaw
 *
 *  Return: Matrix3f64_t
 *----------------------------------------------------------------------------*/
Matrix3f64_t Matrix3_from_euler312(float64_t roll, float64_t pitch, float64_t yaw)
{
    const float64_t c3 = cos(pitch);
    const float64_t s3 = sin(pitch);
    const float64_t s2 = sin(roll);
    const float64_t c2 = cos(roll);
    const float64_t s1 = sin(yaw);
    const float64_t c1 = cos(yaw);
    Matrix3f64_t m;
    
    m.a.x = c1 * c3 - s1 * s2 * s3;
    m.b.y = c1 * c2;
    m.c.z = c3 * c2;
    m.a.y = -c2*s1;
    m.a.z = s3*c1 + c3*s2*s1;
    m.b.x = c3*s1 + s3*s2*c1;
    m.b.z = s1*s3 - s2*c1*c3;
    m.c.x = -s3*c2;
    m.c.y = s2;
    return m;
}
/*-----------------------------------------------------------------------------
 *      Quaternion_to_vector312 : create eulers from a quaternion 
 *
 *  Parameters: const dquat q
 *
 *  Return: dVectr
 *----------------------------------------------------------------------------*/
dVectr Quaternion_to_vector312(const dquat q)
{
    Matrix3f64_t m;
    Quaternion_rotation_matrix(&m,q);
    return Vector3_to_euler312( m );
}
/*-----------------------------------------------------------------------------
 *      Quaternion_from_rotation : create a quaternion from a given rotation 
 *
 *  Parameters: uint8_t rotation
 *
 *  Return: dquat
 *----------------------------------------------------------------------------*/
dquat Quaternion_from_rotation(uint8_t rotation)
{
    // the constants below can be calculated using the following formula:
    //     Matrix3f m_from_rot;
    //     m_from_rot.from_rotation(rotation);
    //     Quaternion q_from_m;
    //     from_rotation_matrix(m_from_rot);

    dquat q = {0.0f,0.0f,0.0f,0.0f};
        
    switch (rotation) {
    case ROTATION_NONE:
        q.x = 1.0f;
        q.y = q.z = q.w = 0;
        return q;

    case ROTATION_YAW_45:
        q.x = 0.92387956f;
        q.y = q.z = 0.0f;
        q.w = 0.38268343f;
        return q;

    case ROTATION_YAW_90:
        q.x = HALF_SQRT_2;
        q.y = q.z = 0.0f;
        q.w = HALF_SQRT_2;
        return q;

    case ROTATION_YAW_135:
        q.x = 0.38268343f;
        q.y = q.z = 0.0f;
        q.w = 0.92387956f;
        return q;

    case ROTATION_YAW_180:
        q.x = q.y = q.z = 0;
        q.w=1;
        return q;

    case ROTATION_YAW_225:
        q.x = -0.38268343f;
        q.y = q.z = 0;
        q.w = 0.92387956f;
        return q;

    case ROTATION_YAW_270:
        q.x = HALF_SQRT_2;
        q.y = q.z = 0;
        q.w = -HALF_SQRT_2;
        return q;

    case ROTATION_YAW_315:
        q.x = 0.92387956f;
        q.y = q.z = 0;
        q.w = -0.38268343f;
        return q;

    case ROTATION_ROLL_180:
        q.x = q.z = q.w = 0;
        q.y = 1;
        return q;

    case ROTATION_ROLL_180_YAW_45:
        q.x = q.w = 0;
        q.y = 0.92387956f;
        q.z = 0.38268343f;
        return q;

    case ROTATION_ROLL_180_YAW_90:
        q.x = q.w = 0;
        q.y = q.z = HALF_SQRT_2;
        return q;

    case ROTATION_ROLL_180_YAW_135:
        q.x = q.w = 0;
        q.y = 0.38268343f;
        q.z = 0.92387956f;
        return q;

    case ROTATION_PITCH_180:
        q.x = q.y = q.w = 0;
        q.z = 1;
        return q;

    case ROTATION_ROLL_180_YAW_225:
        q.x = q.w = 0;
        q.y = -0.38268343f;
        q.z = 0.92387956f;
        return q;

    case ROTATION_ROLL_180_YAW_270:
        q.x = q.w = 0;
        q.y = -HALF_SQRT_2;
        q.z = HALF_SQRT_2;
        return q;

    case ROTATION_ROLL_180_YAW_315:
        q.x = q.w = 0;
        q.y = 0.92387956f;
        q.z = -0.38268343f;
        return q;

    case ROTATION_ROLL_90:
        q.x = q.y = HALF_SQRT_2;
        q.z = q.w = 0;
        return q;

    case ROTATION_ROLL_90_YAW_45:
        q.x = 0.65328151f;
        q.y = 0.65328145f;
        q.z = q.w = 0.27059802f;
        return q;

    case ROTATION_ROLL_90_YAW_90:
        q.x = q.y = q.z = q.w = 0.5f;
        return q;

    case ROTATION_ROLL_90_YAW_135:
        q.x = q.y = 0.27059802f;
        q.z = 0.65328145f;
        q.w = 0.65328151f;
        return q;

    case ROTATION_ROLL_270:
        q.x = HALF_SQRT_2;
        q.y = -HALF_SQRT_2;
        q.z = q.w = 0;
        return q;

    case ROTATION_ROLL_270_YAW_45:
        q.x = 0.65328151f;
        q.y = -0.65328145f;
        q.z = -0.27059802f;
        q.w = 0.27059802f;
        return q;

    case ROTATION_ROLL_270_YAW_90:
        q.x = q.w = 0.5f;
        q.y = q.z = -0.5f;
        return q;

    case ROTATION_ROLL_270_YAW_135:
        q.x = 0.27059802f;
        q.y = -0.27059802f;
        q.z = -0.65328145f;
        q.w = 0.65328151f;
        return q;

    case ROTATION_PITCH_90:
        q.x = q.z = HALF_SQRT_2;
        q.y = q.w = 0;
        return q;

    case ROTATION_PITCH_270:
        q.x = HALF_SQRT_2;
        q.y = q.w = 0;
        q.z = -HALF_SQRT_2;
        return q;

    case ROTATION_PITCH_180_YAW_90:
        q.x = q.w = 0;
        q.y = -HALF_SQRT_2;
        q.z = HALF_SQRT_2;
        return q;

    case ROTATION_PITCH_180_YAW_270:
        q.x = q.w = 0;
        q.y = q.z = HALF_SQRT_2;
        return q;

    case ROTATION_ROLL_90_PITCH_90:
        q.x = q.y = q.z = -0.5f;
        q.w = 0.5f;
        return q;

    case ROTATION_ROLL_180_PITCH_90:
        q.x = q.z = 0;
        q.y = -HALF_SQRT_2;
        q.w = HALF_SQRT_2;
        return q;

    case ROTATION_ROLL_270_PITCH_90:
        q.x = q.z = q.w = 0.5f;
        q.y = -0.5f;
        return q;

    case ROTATION_ROLL_90_PITCH_180:
        q.x = q.y = 0;
        q.z = -HALF_SQRT_2;
        q.w = HALF_SQRT_2;
        return q;

    case ROTATION_ROLL_270_PITCH_180:
        q.x = q.y = 0;
        q.z = q.w = HALF_SQRT_2;
        return q;

    case ROTATION_ROLL_90_PITCH_270:
        q.x = q.y = q.w = 0.5f;
        q.z = -0.5;
        return q;

    case ROTATION_ROLL_180_PITCH_270:
        q.x = q.z = 0;
        q.y = q.w = HALF_SQRT_2;
        return q;

    case ROTATION_ROLL_270_PITCH_270:
        q.x = -0.5f;
        q.y = q.z = q.w = 0.5f;
        return q;

    case ROTATION_ROLL_90_PITCH_180_YAW_90:
        q.x = q.z = -0.5f;
        q.y = q.w = 0.5f;
        return q;

    case ROTATION_ROLL_90_YAW_270:
        q.x = q.y = -0.5f;
        q.z = q.w = 0.5f;
        return q;

    case ROTATION_ROLL_90_PITCH_68_YAW_293:
        q.x = 0.26774535f;
        q.y = 0.70698798f;
        q.z = 0.01295743f;
        q.w = -0.65445596f;
        return q;

    case ROTATION_PITCH_315:
        q.x = 0.92387956f;
        q.y = q.w = 0;
        q.z = -0.38268343f;
        return q;

    case ROTATION_ROLL_90_PITCH_315:
        q.x = 0.65328151f;
        q.y = 0.65328145f;
        q.z = -0.27059802f;
        q.w = 0.27059802f;
        return q;

    case ROTATION_PITCH_7:
        q.x = 0.99813479f;
        q.y = q.w = 0;
        q.z = 0.06104854f;
        return q;

    case ROTATION_CUSTOM:
        return q;

    case ROTATION_MAX:
        break;
    }
   return q;
} 
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
/*-----------------------------------------------------------------------------
 *      Matrix3_to_euler : calculate euler angles from a rotation matrix 
 *
 *  Parameters: const Matrix3f64_t m, float64_t *roll, float64_t *pitch, float64_t *yaw
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void Matrix3_to_euler(const Matrix3f64_t m, float64_t *roll, float64_t *pitch, float64_t *yaw) 
{
    if (pitch != NULL) {
        IKsin(*pitch,m.c.x);
        *pitch = -*pitch;
    }
    if (roll != NULL) {
        IKatan2(*roll,m.c.y,m.c.z);
    }
    if (yaw != NULL) {
        IKatan2(*yaw,m.b.x,m.a.x);
    }
} 
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
/*-----------------------------------------------------------------------------
 *      Matrix3_from_euler : create a rotation matrix given some euler angles 
 *
 *  Parameters: float64_t roll, float64_t pitch, float64_t yaw
 *
 *  Return:     Matrix3f64_t
 *----------------------------------------------------------------------------*/
Matrix3f64_t Matrix3_from_euler(float64_t roll, float64_t pitch, float64_t yaw)
{
    const float64_t cp = cos(pitch);
    const float64_t sp = sin(pitch);
    const float64_t sr = sin(roll);
    const float64_t cr = cos(roll);
    const float64_t sy = sin(yaw);
    const float64_t cy = cos(yaw);
    Matrix3f64_t m;
    
    m.a.x = cp * cy;
    m.a.y = (sr * sp * cy) - (cr * sy);
    m.a.z = (cr * sp * cy) + (sr * sy);
    m.b.x = cp * sy;
    m.b.y = (sr * sp * sy) + (cr * cy);
    m.b.z = (cr * sp * sy) - (sr * cy);
    m.c.x = -sp;
    m.c.y = sr * cp;
    m.c.z = cr * cp;
    return m;
}
/*-----------------------------------------------------------------------------
 *      matf64_dVec_mul : mul 3x3 float matrix by vector  
 *
 *  Parameters: Matrix3f64_t a, dVectr v
 *
 *  Return:     bool
 *----------------------------------------------------------------------------*/
dVectr matf64_dVec_mul(Matrix3f64_t a, dVectr v) 
{
  dVectr ret;  
  float64_t x = a.a.x * v.x + a.a.y * v.y + a.a.z * v.z;
  float64_t y = a.b.x * v.x + a.b.y * v.y + a.b.z * v.z;
  float64_t z = a.c.x * v.x + a.c.y * v.y + a.c.z * v.z;
  ret.x = x; ret.y = y; ret.z = z;
  return ret;
}
/*-----------------------------------------------------------------------------
 *      Quaternion_earth_to_body : convert a vector from earth to body frame  
 *
 *  Parameters: dVectr *v,const dquat q
 *
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool Quaternion_earth_to_body(dVectr *v, const dquat q) 
{
    Matrix3f64_t m;
    if (v==NULL) return false;
    Quaternion_rotation_matrix(&m, q);
    *v = matf64_dVec_mul(m, *v);
    return true;
}
/*-----------------------------------------------------------------------------
 *      wrap_PI : wrap value between PI
 *                             
 *  Parameters: float64_t AngleX
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t wrap_PI( float64_t AngleX )                                               
{
  float64_t Angle = FMOD2(AngleX, (2.0f*PI));
  
  if (Angle>PI)
    Angle = Angle - 2.0f*PI;

  if (Angle<-PI)
    Angle = Angle + 2.0f*PI;
    
   return Angle; 
}
/*-----------------------------------------------------------------------------
 *      Quaternion_to_axis_angle : convert this quaternion to a rotation vector   
 *                                 where the direction of the vector represents
 *                                 the axis of rotation and the length of the 
 *                                 vector represents the angle of rotation
 *  Parameters: dVectr *v,const dquat q
 *
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool Quaternion_to_axis_angle(dVectr *v,const dquat q)
{
    const float64_t l = sqrt((q.y*q.y)+(q.z*q.z)+(q.w*q.w));
    float64_t val,wrap_1;
    if (v==NULL) return false;
    v->x = q.y;
    v->y = q.z;
    v->z = q.w;
    if (l!=0.0f) {
        v->x = v->x / l;
        v->y = v->y / l;
        v->z = v->z / l;
        IKatan2(val,l,q.x);
        wrap_1 = wrap_PI(2.0f * val );
        v->x = v->x * wrap_1;
        v->y = v->y * wrap_1;
        v->z = v->z * wrap_1;
    }
    return true;
}
/*-----------------------------------------------------------------------------
 *      Vector3_rotate : ROTATE vector  
 *
 *  Parameters: uint8_t rotation, dVectr *Vect
 *
 *  Return:     dVectr*
 *----------------------------------------------------------------------------*/
dVectr* Vector3_rotate(uint8_t rotation, dVectr *Vect)
{
    float64_t tmp;
    const float64_t sin_pitch = 0.12186934340514748f;                           // sinf(pitch);
    const float64_t cos_pitch = 0.992546151641322f;                             // cosf(pitch);
    dVectr v1 = {0.0f,0.0f,0.0f};
    
    if (Vect==NULL)
    {
      return &v1; 
    }
    switch (rotation) {
    case ROTATION_NONE:
        return Vect;
    case ROTATION_YAW_45: {
        tmp = HALF_SQRT_2*(Vect->x - Vect->y);
        Vect->y = HALF_SQRT_2*(Vect->x + Vect->y);
        Vect->x = tmp;
        return Vect;
    }
    case ROTATION_YAW_90: {
        tmp = Vect->x; Vect->x = -Vect->y; Vect->y = tmp;
        return Vect;
    }
    case ROTATION_YAW_135: {
        tmp = -HALF_SQRT_2*(Vect->x + Vect->y);
        Vect->y = HALF_SQRT_2*(Vect->x - Vect->y);
        Vect->x = tmp;
        return Vect;
    }
    case ROTATION_YAW_180:
        Vect->x = -Vect->x; Vect->y = -Vect->y;
        return Vect;
    case ROTATION_YAW_225: {
        tmp = HALF_SQRT_2*(Vect->y - Vect->x);
        Vect->y = -HALF_SQRT_2*(Vect->x + Vect->y);
        Vect->x = tmp;
        return Vect;
    }
    case ROTATION_YAW_270: {
        tmp = Vect->x; Vect->x = Vect->y; Vect->y = -tmp;
        return Vect;
    }
    case ROTATION_YAW_315: {
        tmp = HALF_SQRT_2*(Vect->x + Vect->y);
        Vect->y = HALF_SQRT_2*(Vect->y - Vect->x);
        Vect->x = tmp;
        return Vect;
    }
    case ROTATION_ROLL_180: {
        Vect->y = -Vect->y; Vect->z = -Vect->z;
        return Vect;
    }
    case ROTATION_ROLL_180_YAW_45: {
        tmp = HALF_SQRT_2*(Vect->x + Vect->y);
        Vect->y = HALF_SQRT_2*(Vect->x - Vect->y);
        Vect->x = tmp; Vect->z = -Vect->z;
        return Vect;
    }
    case ROTATION_ROLL_180_YAW_90: {
        tmp = Vect->x; Vect->x = Vect->y; Vect->y = tmp; Vect->z = -Vect->z;
        return Vect;
    }
    case ROTATION_ROLL_180_YAW_135: {
        tmp = HALF_SQRT_2*(Vect->y - Vect->x);
        Vect->y = HALF_SQRT_2*(Vect->y + Vect->x);
        Vect->x = tmp; Vect->z = -Vect->z;
        return Vect;
    }
    case ROTATION_PITCH_180: {
        Vect->x = -Vect->x; Vect->z = -Vect->z;
        return Vect;
    }
    case ROTATION_ROLL_180_YAW_225: {
        tmp = -HALF_SQRT_2*(Vect->x + Vect->y);
        Vect->y =  HALF_SQRT_2*(Vect->y - Vect->x);
        Vect->x = tmp; Vect->z = -Vect->z;
        return Vect;
    }
    case ROTATION_ROLL_180_YAW_270: {
        tmp = Vect->x; Vect->x = -Vect->y; Vect->y = -tmp; Vect->z = -Vect->z;
        return Vect;
    }
    case ROTATION_ROLL_180_YAW_315: {
        tmp =  HALF_SQRT_2*(Vect->x - Vect->y);
        Vect->y = -HALF_SQRT_2*(Vect->x + Vect->y);
        Vect->x = tmp; Vect->z = -Vect->z;
        return Vect;
    }
    case ROTATION_ROLL_90: {
        tmp = Vect->z; Vect->z = Vect->y; Vect->y = -tmp;
        return Vect;
    }
    case ROTATION_ROLL_90_YAW_45: {
        tmp = Vect->z; Vect->z = Vect->y; Vect->y = -tmp;
        tmp = HALF_SQRT_2*(Vect->x - Vect->y);
        Vect->y   = HALF_SQRT_2*(Vect->x + Vect->y);
        Vect->x = tmp;
        return Vect;
    }
    case ROTATION_ROLL_90_YAW_90: {
        tmp = Vect->z; Vect->z = Vect->y; Vect->y = -tmp;
        tmp = Vect->x; Vect->x = -Vect->y; Vect->y = tmp;
        return Vect;
    }
    case ROTATION_ROLL_90_YAW_135: {
        tmp = Vect->z; Vect->z = Vect->y; Vect->y = -tmp;
        tmp = -HALF_SQRT_2*(Vect->x + Vect->y);
        Vect->y   =  HALF_SQRT_2*(Vect->x - Vect->y);
        Vect->x = tmp;
        return Vect;
    }
    case ROTATION_ROLL_270: {
        tmp = Vect->z; Vect->z = -Vect->y; Vect->y = tmp;
        return Vect;
    }
    case ROTATION_ROLL_270_YAW_45: {
        tmp = Vect->z; Vect->z = -Vect->y; Vect->y = tmp;
        tmp = HALF_SQRT_2*(Vect->x - Vect->y);
        Vect->y   = HALF_SQRT_2*(Vect->x + Vect->y);
        Vect->x = tmp;
        return Vect;
    }
    case ROTATION_ROLL_270_YAW_90: {
        tmp = Vect->z; Vect->z = -Vect->y; Vect->y = tmp;
        tmp = Vect->x; Vect->x = -Vect->y; Vect->y = tmp;
        return Vect;
    }
    case ROTATION_ROLL_270_YAW_135: {
        tmp = Vect->z; Vect->z = -Vect->y; Vect->y = tmp;
        tmp = -HALF_SQRT_2*(Vect->x + Vect->y);
        Vect->y   =  HALF_SQRT_2*(Vect->x - Vect->y);
        Vect->x = tmp;
        return Vect;
    }
    case ROTATION_PITCH_90: {
        tmp = Vect->z; Vect->z = -Vect->x; Vect->x = tmp;
        return Vect;
    }
    case ROTATION_PITCH_270: {
        tmp = Vect->z; Vect->z = Vect->x; Vect->x = -tmp;
        return Vect;
    }
    case ROTATION_PITCH_180_YAW_90: {
        Vect->z = -Vect->z;
        tmp = -Vect->x; Vect->x = -Vect->y; Vect->y = tmp;
        return Vect;
    }
    case ROTATION_PITCH_180_YAW_270: {
        Vect->x = -Vect->x; Vect->z = -Vect->z;
        tmp = Vect->x; Vect->x = Vect->y; Vect->y = -tmp;
        return Vect;
    }
    case ROTATION_ROLL_90_PITCH_90: {
        tmp = Vect->z; Vect->z = Vect->y; Vect->y = -tmp;
        tmp = Vect->z; Vect->z = -Vect->x; Vect->x = tmp;
        return Vect;
    }
    case ROTATION_ROLL_180_PITCH_90: {
        Vect->y = -Vect->y; Vect->z = -Vect->z;
        tmp = Vect->z; Vect->z = -Vect->x; Vect->x = tmp;
        return Vect;
    }
    case ROTATION_ROLL_270_PITCH_90: {
        tmp = Vect->z; Vect->z = -Vect->y; Vect->y = tmp;
        tmp = Vect->z; Vect->z = -Vect->x; Vect->x = tmp;
        return Vect;
    }
    case ROTATION_ROLL_90_PITCH_180: {
        tmp = Vect->z; Vect->z = Vect->y; Vect->y = -tmp;
        Vect->x = -Vect->x; Vect->z = -Vect->z;
        return Vect;
    }
    case ROTATION_ROLL_270_PITCH_180: {
        tmp = Vect->z; Vect->z = -Vect->y; Vect->y = tmp;
        Vect->x = -Vect->x; Vect->z = -Vect->z;
        return Vect;
    }
    case ROTATION_ROLL_90_PITCH_270: {
        tmp = Vect->z; Vect->z = Vect->y; Vect->y = -tmp;
        tmp = Vect->z; Vect->z = Vect->x; Vect->x = -tmp;
        return Vect;
    }
    case ROTATION_ROLL_180_PITCH_270: {
        Vect->y = -Vect->y; Vect->z = -Vect->z;
        tmp = Vect->z; Vect->z = Vect->x; Vect->x = -tmp;
        return Vect;
    }
    case ROTATION_ROLL_270_PITCH_270: {
        tmp = Vect->z; Vect->z = -Vect->y; Vect->y = tmp;
        tmp = Vect->z; Vect->z = Vect->x; Vect->x = -tmp;
        return Vect;
    }
    case ROTATION_ROLL_90_PITCH_180_YAW_90: {
        tmp = Vect->z; Vect->z = Vect->y; Vect->y = -tmp;
        Vect->x = -Vect->x; Vect->z = -Vect->z;
        tmp = Vect->x; Vect->x = -Vect->y; Vect->y = tmp;
        return Vect;
    }
    case ROTATION_ROLL_90_YAW_270: {
        tmp = Vect->z; Vect->z = Vect->y; Vect->y = -tmp;
        tmp = Vect->x; Vect->x = Vect->y; Vect->y = -tmp;
        return Vect;
    }
    case ROTATION_ROLL_90_PITCH_68_YAW_293: {
        Vect->x =  0.143039f * Vect->x +  0.368776f * Vect->y + -0.918446f * Vect->z;
        Vect->y = -0.332133f * Vect->x + -0.856289f * Vect->y + -0.395546f * Vect->z;
        Vect->z = -0.932324f * Vect->x +  0.361625f * Vect->y +  0.000000f * Vect->z;
        return Vect;
    }
    case ROTATION_PITCH_315: {
        tmp = HALF_SQRT_2*(Vect->x - Vect->z);
        Vect->z   = HALF_SQRT_2*(Vect->x + Vect->z);
        Vect->x = tmp;
        return Vect;
    }
    case ROTATION_ROLL_90_PITCH_315: {
        tmp = Vect->z; Vect->z = Vect->y; Vect->y = -tmp;
        tmp = HALF_SQRT_2*(Vect->x - Vect->z);
        Vect->z   = HALF_SQRT_2*(Vect->x + Vect->z);
        Vect->x = tmp;
        return Vect;
    }
    case ROTATION_PITCH_7: {
        Vect->x =  cos_pitch * Vect->x + sin_pitch * Vect->z;
        Vect->y = -sin_pitch * Vect->x + cos_pitch * Vect->z;
        return Vect;
    }
    case ROTATION_CUSTOM: 
        return Vect;                                                            // Error: caller must perform custom rotations via matrix multiplication
    case ROTATION_MAX:
        break;
    }
    return Vect;
}
/*-----------------------------------------------------------------------------
 *      Matrix99_mtranspose : transpose 9x9 matrix 
 *
 *  Parameters: float64_t m[9][9]
 *
 *  Return:     float64_t*
 *----------------------------------------------------------------------------*/
float64_t* Matrix99_mtranspose(float64_t m[9][9])
{
   float64_t transp[9][9];
   int8_t i,j;
   for (i = 0; i < 9; ++i)
   {
        for (j = 0; j < 9; ++j)
        {
              transp[i][j] = m[j][i];
        }
   }
   return &transp;
}
/*-----------------------------------------------------------------------------
 *      Matrix4_transpose : transpose 4x4 matrix 
 *
 *  Parameters: Matrix4d64_t *mat 
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Matrix4_transpose( Matrix4d64_t *mat ) 
{
        float64_t tmp;

        tmp = mat->a.y;  
        mat->a.x = mat->b.x;  
        mat->b.x = tmp;
        tmp = mat->a.z;  
        mat->a.z = mat->c.x;  
        mat->c.x = tmp;
        tmp = mat->a.w;  
        mat->a.w = mat->d.x;  
        mat->d.x = tmp;
        tmp = mat->b.z;  
        mat->b.z = mat->c.y;  
        mat->c.y = tmp;
        tmp = mat->b.w;  
        mat->b.w = mat->d.y;  
        mat->d.y = tmp;
        tmp = mat->c.w;  
        mat->c.w = mat->d.z;  
        mat->d.z = tmp;
}
/*-----------------------------------------------------------------------------
 *      set_Matrix4 : set 4 quat as a 4x4 matrix 
 *
 *  Parameters: const dquat v0, const dquat v1, const dquat v2, const dquat v3, 
 *              Matrix4d64_t *a
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void set_Matrix4( const dquat v0, const dquat v1, const dquat v2, const dquat v3, Matrix4d64_t *a ) 
{
        a->a.x= v0.x; 
        a->b.x= v0.y; 
        a->c.x= v0.z;
        a->d.x= v0.w;
        a->a.y= v1.x; 
        a->b.y= v1.y; 
        a->c.y= v1.z; 
        a->d.y= v1.w;
        a->a.z= v2.x; 
        a->b.z= v2.y;
        a->c.z= v2.z; 
        a->d.z= v2.w;
        a->a.w= v3.x;
        a->b.w= v3.y; 
        a->c.w= v3.z; 
        a->d.w= v3.w;
}
/*-----------------------------------------------------------------------------
 *      mul_Matrix4 : multiply 4x4 matrix 
 *
 *  Parameters: const Matrix4d64_t m0, const Matrix4d64_t m1, Matrix4d64_t *out
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void mul_Matrix4( const Matrix4d64_t m0, const Matrix4d64_t m1, Matrix4d64_t *out ) 
{
        out->a.x= m0.a.x*m1.a.x + m0.a.y*m1.b.x + m0.a.z*m1.c.x + m0.a.w*m1.d.x;
        out->b.x= m0.b.x*m1.a.x + m0.b.y*m1.b.x + m0.b.z*m1.c.x + m0.b.w*m1.d.x;
        out->c.x= m0.c.x*m1.a.x + m0.c.y*m1.b.x + m0.c.z*m1.c.x + m0.c.w*m1.d.x;
        out->d.x= m0.d.x*m1.a.x + m0.d.y*m1.b.x + m0.d.z*m1.c.x + m0.d.w*m1.d.x;

        out->a.y= m0.a.x*m1.a.y + m0.a.y*m1.b.y + m0.a.z*m1.c.y + m0.a.w*m1.d.y;
        out->b.y= m0.b.x*m1.a.y + m0.b.y*m1.b.y + m0.b.z*m1.c.y + m0.b.w*m1.d.y;
        out->c.y= m0.c.x*m1.a.y + m0.c.y*m1.b.y + m0.c.z*m1.c.y + m0.c.w*m1.d.y;
        out->d.y= m0.d.x*m1.a.y + m0.d.y*m1.b.y + m0.d.z*m1.c.y + m0.d.w*m1.d.y;

        out->a.z= m0.a.x*m1.a.z + m0.a.y*m1.b.z + m0.a.z*m1.c.z + m0.a.w*m1.d.z;
        out->b.z= m0.b.x*m1.a.z + m0.b.y*m1.b.z + m0.b.z*m1.c.z + m0.b.w*m1.d.z;
        out->c.z= m0.c.x*m1.a.z + m0.c.y*m1.b.z + m0.c.z*m1.c.z + m0.c.w*m1.d.z;
        out->d.z= m0.d.x*m1.a.z + m0.d.y*m1.b.z + m0.d.z*m1.c.z + m0.d.w*m1.d.z;

        out->a.w= m0.a.x*m1.a.w + m0.a.y*m1.b.w + m0.a.z*m1.c.w + m0.a.w*m1.d.w;
        out->b.w= m0.b.x*m1.a.w + m0.b.y*m1.b.w + m0.b.z*m1.c.w + m0.b.w*m1.d.w;
        out->c.w= m0.c.x*m1.a.w + m0.c.y*m1.b.w + m0.c.z*m1.c.w + m0.c.w*m1.d.w;
        out->d.w= m0.d.x*m1.a.w + m0.d.y*m1.b.w + m0.d.z*m1.c.w + m0.d.w*m1.d.w;
}
/*-----------------------------------------------------------------------------
 *      setRotationX_Matrix4 : set rotation angle X axis for 4x4 matrix 
 *
 *  Parameters: float64_t angle, Matrix4d64_t *out
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void setRotationX_Matrix4(float64_t angle, Matrix4d64_t *out)
{
        float64_t s = sin(angle);
        float64_t c = cos(angle);

        out->a.x= 1.0f;
        out->b.x= 0.0f;
        out->c.x= 0.0f;
        out->d.x= 0.0f;

        out->a.y= 0.0f;
        out->b.y= c;
        out->c.y= s;
        out->d.y= 0.0f;

        out->a.z= 0.0f;
        out->b.z= -s;
        out->c.z= c;
        out->d.z= 0.0f;

        out->a.w= 0.0f;
        out->b.w= 0.0f;
        out->c.w= 0.0f;
        out->d.w= 1.0f;

}
/*-----------------------------------------------------------------------------
 *      setRotationY_Matrix4 : set rotation angle Y axis for 4x4 matrix
 *
 *  Parameters: float64_t angle, Matrix4d64_t *out
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/    
void setRotationY_Matrix4(float64_t angle, Matrix4d64_t *out) 
{
        float64_t s = sin(angle);
        float64_t c = cos(angle);

        out->a.x= c;
        out->b.x= 0.0f;
        out->c.x= -s;
        out->d.x= 0.0f;

        out->a.y= 0.0f;
        out->b.y= 1.0f;
        out->c.y= 0.0f;
        out->d.y= 0.0f;

        out->a.z= s;
        out->b.z= 0.0f;
        out->c.z= c;
        out->d.z= 0.0f;

        out->a.w= 0.0f;
        out->b.w= 0.0f;
        out->c.w= 0.0f;
        out->d.w= 1.0f;

}
/*-----------------------------------------------------------------------------
 *      setRotationZ_Matrix4 : set rotation angle Z axis for 4x4 matrix 
 *
 *  Parameters: float64_t angle, Matrix4d64_t *out
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void setRotationZ_Matrix4(float64_t angle, Matrix4d64_t *out) 
{
        float64_t s = sin(angle);
        float64_t c = cos(angle);

        out->a.x= c;
        out->b.x= s;
        out->c.x= 0.0f;
        out->d.x= 0.0f;

        out->a.y= -s;
        out->b.y= c;
        out->c.y= 0.0f;
        out->d.y= 0.0f;

        out->a.z= 0.0f;
        out->b.z= 0.0f;
        out->c.z= 1.0f;
        out->d.z= 0.0f;

        out->a.w= 0.0f;
        out->b.w= 0.0f;
        out->c.w= 0.0f;
        out->d.w= 1.0f;
}
/*-----------------------------------------------------------------------------
 *      invertR_Matrix4 : invert calculation for rotation matrix 
 *
 *  Parameters: const Matrix4d64_t m, Matrix4d64_t *mat
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void invertR_Matrix4(const Matrix4d64_t m, Matrix4d64_t *mat) 
{
        mat->a.x = m.a.x; 
        mat->b.y = m.b.y; 
        mat->c.z = m.c.z;
        mat->a.y = m.b.x; 
        mat->b.x = m.a.y;
        mat->a.z = m.c.x; 
        mat->c.x = m.a.z;
        mat->b.z = m.c.y; 
        mat->c.y = m.b.z;
        mat->a.w = mat->b.w = mat->c.w = 0.0f;
        mat->d.x = mat->d.y = mat->d.z = 0.0f; 
        mat->d.w = 1.0f;
}
/*-----------------------------------------------------------------------------
 *      invertRwot_Matrix4 : invert calculation for rotation matrix (without trans)
 *
 *  Parameters: Matrix4d64_t *mat
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void invertRwot_Matrix4( Matrix4d64_t *mat ) 
{
        float64_t tmp;
        tmp = mat->a.y; mat->a.y = mat->b.x; mat->b.x = tmp;
        tmp = mat->a.z; mat->a.z = mat->c.x; mat->c.x = tmp;
        tmp = mat->b.z; mat->b.z = mat->c.y; mat->c.y = tmp;
        // m03 = m13 = m23 = 0.0f;
        // m30 = m31 = m32 = 0.0f; 
        // m33 = 1.0f;
}
/*-----------------------------------------------------------------------------
 *      transform_mat44 : transform 4x4 matrix by quaternion
 *
 *  Parameters: const dquat in, const mat44 m
 *
 *  Return:     dquat
 *----------------------------------------------------------------------------*/
dquat transform_mat44(const dquat in, const mat44 m )  
{
        dquat out;
        out.x = m.m[0][0] * in.x + m.m[0][1] * in.y + m.m[0][2] * in.z + m.m[0][3] * in.w;
        out.y = m.m[1][0] * in.x + m.m[1][1] * in.y + m.m[1][2] * in.z + m.m[1][3] * in.w;
        out.z = m.m[2][0] * in.x + m.m[2][1] * in.y + m.m[2][2] * in.z + m.m[2][3] * in.w;
        out.w = m.m[3][0] * in.x + m.m[3][1] * in.y + m.m[3][2] * in.z + m.m[3][3] * in.w;
        return out;
}

//! overwrite the matrix with a translation-matrix
void setTranslation_mat16(const dVectr v, float64_t* matrix) 
{
  if (matrix == NULL)
  {
  }
  else
  {
     matrix[0u] = 1.0f;  matrix[1u] = 0.0f;  matrix[2u] = 0.0f;  matrix[3u] = v.x;
     matrix[4u] = 0.0f;  matrix[5u] = 1.0f;  matrix[6u] = 0.0f;  matrix[7u] = v.y;
     matrix[8u] = 0.0f;  matrix[9u] = 0.0f;  matrix[10u] = 1.0f;  matrix[11u] = v.z;
     matrix[12u] = 0.0f;  matrix[13u] = 0.0f;  matrix[14u] = 0.0f;  matrix[15u] = 1.0f;
  }
}
//! overwrite the matrix with a translation-matrix
mat44 setTranslation_mat44(const dVectr v) 
{
  mat44 ret;        
  ret.m[0u][0u] = 1.0f;  ret.m[0u][1u] = 0.0f;  ret.m[0u][2u] = 0.0f;  ret.m[0u][3u] = v.x;
  ret.m[1u][0u] = 0.0f;  ret.m[1u][1u] = 1.0f;  ret.m[1u][2u] = 0.0f;  ret.m[1u][3u] = v.y;
  ret.m[2u][0u] = 0.0f;  ret.m[2u][1u] = 0.0f;  ret.m[2u][2u] = 1.0f;  ret.m[2u][3u] = v.z;
  ret.m[3u][0u] = 0.0f;  ret.m[3u][1u] = 0.0f;  ret.m[3u][2u] = 0.0f;  ret.m[3u][3u] = 1.0f;
  return ret;
}

//! overwrite the matrix with a translation-matrix
Matrix4d64_t setTranslation_Matrix4d64(const dVectr v) 
{
  Matrix4d64_t ret;        
  ret.a.x = 1.0f;  ret.a.y = 0.0f;  ret.a.z = 0.0f;  ret.a.w = v.x;
  ret.b.x = 0.0f;  ret.b.y = 1.0f;  ret.b.z = 0.0f;  ret.b.w = v.y;
  ret.c.x = 0.0f;  ret.c.y = 0.0f;  ret.c.z = 1.0f;  ret.c.w = v.z;
  ret.d.x = 0.0f;  ret.d.y = 0.0f;  ret.d.z = 0.0f;  ret.d.w = 1.0f;
  return ret;
}

//! overwrite the matrix with a rotation-matrix around a coordinate-axis (angle is specified in degrees)
void setRotationX_mat16(float64_t angle, float64_t *matrix) 
{
        float64_t angleRad = DEGREE_TO_RADIAN(angle);
        float64_t sinAngle = sin(angleRad);
        float64_t cosAngle = cos(angleRad);

        matrix[0u]=1.0f;  matrix[1u]=0.0f; matrix[2u]=0.0f; matrix[3u]=0.0f;
        matrix[4u]=0.0f;  matrix[5u]=cosAngle; matrix[6u]=-sinAngle; matrix[7u]=0.0f;
        matrix[8u]=0.0f;  matrix[9u]=sinAngle; matrix[10u]= cosAngle; matrix[11u]=0.0f;
        matrix[12u]=0.0f;  matrix[13u]=0.0f; matrix[14u]=0.0f; matrix[15u]=1.0f;  
}

//! overwrite the matrix with a rotation-matrix around a coordinate-axis (angle is specified in degrees)
Matrix4d64_t setRotationX_Matrix4d64(float64_t angle) 
{
        Matrix4d64_t ret;        
        float64_t angleRad = DEGREE_TO_RADIAN(angle);
        float64_t sinAngle = sin(angleRad);
        float64_t cosAngle = cos(angleRad);

        ret.a.x=1.0f;  ret.a.y=0.0f; ret.a.z=0.0f; ret.a.w=0.0f;
        ret.b.x=0.0f;  ret.b.y=cosAngle; ret.b.z=-sinAngle; ret.b.w=0.0f;
        ret.c.x=0.0f;  ret.c.y=sinAngle; ret.c.z= cosAngle; ret.c.w=0.0f;
        ret.d.x=0.0f;  ret.d.y=0.0f; ret.d.z=0.0f; ret.d.w=1.0f; 
        return ret;        
}

//! overwrite the matrix with a rotation-matrix around a coordinate-axis (angle is specified in degrees)
mat44 setRotationX_mat44_2(float64_t angle) 
{
        mat44 ret;        
        float64_t angleRad = DEGREE_TO_RADIAN(angle);
        float64_t sinAngle = sin(angleRad);
        float64_t cosAngle = cos(angleRad);

        ret.m[0u][0u]=1.0f;  ret.m[0u][1u]=0.0f; ret.m[0u][2u]=0.0f; ret.m[0u][3u]=0.0f;
        ret.m[1u][0u]=0.0f;  ret.m[1u][1u]=cosAngle; ret.m[1u][2u]=-sinAngle; ret.m[1u][3u]=0.0f;
        ret.m[2u][0u]=0.0f;  ret.m[1u][1u]=sinAngle; ret.m[2u][2u]= cosAngle; ret.m[2u][3u]=0.0f;
        ret.m[3u][0u]=0.0f;  ret.m[1u][1u]=0.0f; ret.m[3u][2u]=0.0f; ret.m[3u][3u]=1.0f; 
        return ret;        
}

//! overwrite the matrix with a rotation-matrix around a coordinate-axis (angle is specified in degrees)
void setRotationY_mat16(float64_t angle, float64_t *matrix) 
{
        float64_t angleRad = DEGREE_TO_RADIAN(angle);
        float64_t sinAngle = sin(angleRad);
        float64_t cosAngle = cos(angleRad);

        matrix[0u]=cosAngle; matrix[1u]=0.0f; matrix[2u]=sinAngle; matrix[3u]=0.0f;
        matrix[4u]=0.0f; matrix[5u]=1; matrix[6u]=0;  matrix[7u]=0.0f;
        matrix[8u]=-sinAngle; matrix[9u]=0.0f; matrix[10u]=cosAngle; matrix[11u]=0.0f;
        matrix[12u]=0.0f; matrix[13u]=0.0f;  matrix[14u]=0.0f; matrix[15u]=1.0f;   
}

//! overwrite the matrix with a rotation-matrix around a coordinate-axis (angle is specified in degrees)
Matrix4d64_t setRotationY_Matrix4d64(float64_t angle) 
{
        Matrix4d64_t ret;
        float64_t angleRad = DEGREE_TO_RADIAN(angle);
        float64_t sinAngle = sin(angleRad);
        float64_t cosAngle = cos(angleRad);

        ret.a.x=cosAngle; ret.a.y=0.0f; ret.a.z=sinAngle; ret.a.w=0.0f;
        ret.b.x=0.0f; ret.b.y=1.0f; ret.b.z=0.0f; ret.b.w=0.0f;
        ret.c.x=-sinAngle; ret.c.y=0.0f; ret.c.z=cosAngle; ret.c.w=0.0f;
        ret.d.x=0.0f; ret.d.y=0.0f; ret.d.z=0.0f; ret.d.w=1.0f; 
    return ret;        
}

//! overwrite the matrix with a rotation-matrix around a coordinate-axis (angle is specified in degrees)
mat44 setRotationY_mat44_2(float64_t angle) 
{
        mat44 ret;
        float64_t angleRad = DEGREE_TO_RADIAN(angle);
        float64_t sinAngle = sin(angleRad);
        float64_t cosAngle = cos(angleRad);

        ret.m[0u][0u]=cosAngle; ret.m[0u][1u]=0.0f; ret.m[0u][2u]=sinAngle; ret.m[0u][3u]=0.0f;
        ret.m[1u][0u]=0.0f; ret.m[1u][1u]=1.0f; ret.m[1u][2u]=0.0f; ret.m[1u][3u]=0.0f;
        ret.m[2u][0u]=-sinAngle; ret.m[2u][1u]=0.0f; ret.m[2u][2u]=cosAngle; ret.m[2u][3u]=0.0f;
        ret.m[3u][0u]=0.0f; ret.m[3u][1u]=0.0f; ret.m[3u][2u]=0.0f; ret.m[3u][3u]=1.0f; 
    return ret;        
}

//! overwrite the matrix with a rotation-matrix around a coordinate-axis (angle is specified in degrees)
void setRotationZ(float64_t angle, float64_t *matrix) 
{
        float64_t angleRad = DEGREE_TO_RADIAN(angle);
        float64_t sinAngle = sin(angleRad);
        float64_t cosAngle = cos(angleRad);

        matrix[0u]=cosAngle; matrix[1u]=-sinAngle; matrix[2u]=0.0f; matrix[3u]=0.0f;
        matrix[4u]=sinAngle; matrix[5u]= cosAngle; matrix[6u]=0.0f; matrix[7u]=0.0f;
        matrix[8u]=0.0f; matrix[9u]=0.0f; matrix[10u]=1.0f; matrix[11u]=0.0f;
        matrix[12u]=0.0f; matrix[13u]=0.0f; matrix[14u]=0.0f; matrix[15u]=1.0f;
}

//! overwrite the matrix with a rotation-matrix around a coordinate-axis (angle is specified in degrees)
Matrix4d64_t setRotationZ_Matrix4d64(float64_t angle) 
{
        Matrix4d64_t ret;
        float64_t angleRad = DEGREE_TO_RADIAN(angle);
        float64_t sinAngle = sin(angleRad);
        float64_t cosAngle = cos(angleRad);

        ret.a.x=cosAngle; ret.a.y=-sinAngle; ret.a.z=0.0f; ret.a.w=0.0f;
        ret.b.x=sinAngle; ret.b.y= cosAngle; ret.b.z=0.0f; ret.b.w=0.0f;
        ret.c.x=0.0f; ret.c.y=0.0f; ret.c.z=1.0f; ret.c.w=0.0f;
        ret.d.x=0.0f; ret.d.y=0.0f; ret.d.z=0.0f; ret.d.w=1.0f;
        return ret;
}

//! overwrite the matrix with a rotation-matrix around a coordinate-axis (angle is specified in degrees)
mat44 setRotationZ_mat44_2(float64_t angle) 
{
    mat44 ret;
    float64_t angleRad = DEGREE_TO_RADIAN(angle);
    float64_t sinAngle = sin(angleRad);
    float64_t cosAngle = cos(angleRad);

    ret.m[0u][0u]=cosAngle; ret.m[0u][1u]=-sinAngle; ret.m[0u][2u]=0.0f; ret.m[0u][3u]=0.0f;
    ret.m[1u][0u]=sinAngle; ret.m[1u][1u]= cosAngle; ret.m[1u][2u]=0.0f; ret.m[1u][3u]=0.0f;
    ret.m[2u][0u]=0.0f; ret.m[2u][1u]=0.0f; ret.m[2u][2u]=1.0f; ret.m[2u][3u]=0.0f;
    ret.m[3u][0u]=0.0f; ret.m[3u][1u]=0.0f; ret.m[3u][2u]=0.0f; ret.m[3u][3u]=1.0f;
    return ret;
}

//! overwrite the matrix with a scale-matrix
void setScale_mat16(const dVectr vec, float64_t *matrix)  
{
        matrix[0u] = vec.x; matrix[1u] = 0.0f; matrix[2u] = 0.0f; matrix[3u] = 0.0f;
        matrix[4u] = 0.0f; matrix[5u] = vec.y; matrix[6u] = 0.0f; matrix[7u] = 0.0f;
        matrix[8u] = 0.0f; matrix[9u] = 0.0f; matrix[10u] = vec.z; matrix[11u] = 0.0f;
        matrix[12u] = 0.0f; matrix[13u] = 0.0f; matrix[14u] = 0.0f; matrix[15u] = 1.0f;
}

//! overwrite the matrix with a scale-matrix
mat44 setScale_mat44(const dVectr vec)  
{
        mat44 ret;
        ret.m[0u][0u] = vec.x; ret.m[0u][1u] = 0.0f; ret.m[0u][2u] = 0.0f; ret.m[0u][3u] = 0.0f;
        ret.m[1u][0u] = 0.0f; ret.m[1u][1u] = vec.y; ret.m[1u][2u] = 0.0f; ret.m[1u][3u] = 0.0f;
        ret.m[2u][0u] = 0.0f; ret.m[2u][1u] = 0.0f; ret.m[2u][2u] = vec.z; ret.m[2u][3u] = 0.0f;
        ret.m[3u][0u] = 0.0f; ret.m[3u][1u] = 0.0f; ret.m[3u][2u] = 0.0f; ret.m[3u][3u] = 1.0f;
    return ret;
}

//! overwrite the matrix with a scale-matrix
Matrix4d64_t setScale_Matrix4d64(const dVectr vec)  
{
        Matrix4d64_t ret;
        ret.a.x = vec.x; ret.a.y = 0.0f; ret.a.z = 0.0f; ret.a.w = 0.0f;
        ret.b.x = 0.0f; ret.b.y = vec.y; ret.b.z = 0.0f; ret.b.w = 0.0f;
        ret.c.x = 0.0f; ret.c.y = 0.0f; ret.c.z = vec.z; ret.c.w = 0.0f;
        ret.d.x = 0.0f; ret.d.y = 0.0f; ret.d.z = 0.0f; ret.d.w = 1.0f;
    return ret;
}

/*-----------------------------------------------------------------------------
 *      mkdquat : construct a quaternion from its x, y, z, w elements..
 *                             
 *  Parameters: float64_t x, float64_t y, float64_t z, float64_t w
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/
dquat mkdquat(float64_t x, float64_t y, float64_t z, float64_t w) 
{
    dquat q;
    q.x = x; q.y = y; q.z = z; q.w = w;
    return q;
} 

//! transform a 3D-vector with the matrix (implicit w=1 and implicit subsequent de-homogenization)
dquat dquatFrom_matrix16(const dVectr v, const float64_t *matrix)  
{
  return (mkdquat((matrix[0u]*v.x + matrix[1u]*v.y + matrix[2u]*v.z + matrix[3u]),(matrix[4u]*v.x + matrix[5u]*v.y + matrix[6u]*v.z + matrix[7u]),(matrix[8u]*v.x + matrix[9u]*v.y + matrix[10u]*v.z + matrix[11u]), (matrix[12u]*v.x + matrix[13u]*v.y + matrix[14u]*v.z + matrix[15u])));
}

//! transform a 3D-vector with the matrix (implicit w=1 and implicit subsequent de-homogenization)
dquat dquatFrom_mat44(const dVectr v, const mat44 matrix)  
{
   return (mkdquat(matrix.m[0u][0u]*v.x + matrix.m[0u][1u]*v.y + matrix.m[0u][2u]*v.z + matrix.m[0u][3u],matrix.m[1u][0u]*v.x + matrix.m[1u][1u]*v.y + matrix.m[1u][2u]*v.z + matrix.m[1u][3u],matrix.m[2u][0u]*v.x + matrix.m[2u][1u]*v.y + matrix.m[2u][2u]*v.z + matrix.m[2u][3u], matrix.m[3u][0u]*v.x + matrix.m[3u][1u]*v.y + matrix.m[3u][2u]*v.z + matrix.m[3u][3u]));
}

//! transform a 3D-vector with the matrix (implicit w=1 and implicit subsequent de-homogenization)
dquat dquatFrom_Matrix4d64(const dVectr v, const Matrix4d64_t matrix)  
{
    return (mkdquat(matrix.a.x*v.x + matrix.a.y*v.y + matrix.a.z*v.z + matrix.a.w,matrix.b.x*v.x + matrix.b.y*v.y + matrix.b.z*v.z + matrix.b.w,matrix.c.x*v.x + matrix.c.y*v.y + matrix.c.z*v.z + matrix.c.w, matrix.d.x*v.x + matrix.d.y*v.y + matrix.d.z*v.z + matrix.d.w));
}

//! returns true if the matrix is affine (i.e., projective part is zero)
bool isAffine_matrix16(float64_t *matrix)  
{
    bool ret;
    if (matrix == NULL)
    {   ret = false; }
    else
    {
        if (((matrix[12u] <= 0.000001f) && (matrix[13u] <= 0.000001f)) && ((matrix[14u] <= 0.000001f) && (matrix[15u] <= 0.000001f)))        
           ret = true;
        else 
           ret = false;
    }
    return ret;
}
/*-----------------------------------------------------------------------------
 *      transform_Matrix4 : transform 4x4 matrix by quaternion
 *
 *  Parameters: const dquat in, const Matrix4d64_t m, dquat *out
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void transform_Matrix4(const dquat in, const Matrix4d64_t m, dquat *out  ) 
{
        out->x = m.a.x * in.x + m.a.y * in.y + m.a.z * in.z + m.a.w * in.w;
        out->y = m.b.x * in.x + m.b.y * in.y + m.b.z * in.z + m.b.w * in.w;
        out->z = m.c.x * in.x + m.c.y * in.y + m.c.z * in.z + m.c.w * in.w;
        out->w = m.d.x * in.x + m.d.y * in.y + m.d.z * in.z + m.d.w * in.w;
}
/* 
   Linear Transformation or MV Multiplication 
   ref: Presentation by Marwan Nour and Mohammed El Baker Nasser 
   American University of Beirut 
*/  
/*-----------------------------------------------------------------------------
 *   linearTransFormation_Matrix4 : linear transform of 4x4 by quat
 *
 *  Parameters: quat v, Matrix4d64_t m, quat *mv
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/ 
void linearTransFormation_Matrix4(quat v, Matrix4d64_t m, quat *mv) 
{
    mv->x = ((((((m.a.x*v.x)+m.a.y)*v.y)+m.a.z)*v.z)+m.a.w)*v.w;
    mv->y = ((((((m.b.y*v.y)+m.b.z)*v.z)+m.b.w)*v.w)+m.b.x)*v.x;
    mv->z = ((((((m.c.z*v.z)+m.c.w)*v.w)+m.c.x)*v.x)+m.c.y)*v.y;
    mv->w = ((((((m.d.w*v.w)+m.d.x)*v.x)+m.d.y)*v.y)+m.d.z)*v.z;
}
/*-----------------------------------------------------------------------------
 *   linearTransFormation2_Matrix4 : linear transform of 4x4 by quat
 *
 *  Parameters: quat v, Matrix4d64_t m
 *
 *  Return:     quat
 *----------------------------------------------------------------------------*/ 
quat linearTransFormation2_Matrix4(quat v, Matrix4d64_t m) 
{
    quat mv; 
    mv.x = ((((((m.a.x*v.x)+m.a.y)*v.y)+m.a.z)*v.z)+m.a.w)*v.w;
    mv.y = ((((((m.b.y*v.y)+m.b.z)*v.z)+m.b.w)*v.w)+m.b.x)*v.x;
    mv.z = ((((((m.c.z*v.z)+m.c.w)*v.w)+m.c.x)*v.x)+m.c.y)*v.y;
    mv.w = ((((((m.d.w*v.w)+m.d.x)*v.x)+m.d.y)*v.y)+m.d.z)*v.z;
    return mv;
}
/*-----------------------------------------------------------------------------
 *   polyApprox_sigmoid_g3 : polynomial approx of the sigmoid function
 *
 *  Parameters: float64_t x
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/ 
float64_t polyApprox_sigmoid_g3( float64_t x )
{
   return (0.5f + (1.20096f*(x/8.0f)) - (0.81562f*(pow((x/8.0f),3.0f))));
}
/*-----------------------------------------------------------------------------
 *   polyApprox_sigmoid_g5 : polynomial approx of the sigmoid function
 *
 *  Parameters: float64_t x
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t polyApprox_sigmoid_g5( float64_t x )
{
   return (0.5f + (1.53048f*(x/8.0f)) - (2.3533056f*(pow((x/8.0f),3.0f))) + (1.3511295f*(pow((x/8.0f),5.0f))));
}
/*-----------------------------------------------------------------------------
 *   polyApprox_sigmoid_g7 : polynomial approx of the sigmoid function
 *
 *  Parameters: float64_t x
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t polyApprox_sigmoid_g7( float64_t x )
{
   return (0.5f + (1.73496f*(x/8.0f)) - (4.19407f*(pow((x/8.0f),3.0f))) + (5.43402f*(pow((x/8.0f),5.0f))) + (2.50739f*(pow((x/8.0f),7.0f))));
}
/*-----------------------------------------------------------------------------
 *   matrix9x9_ctAB : multiply two 9 byte streams or 3x3 matrices k is the step of 1 or 2
 *
 *  Parameters: const mat33 ctA, mat33 *ctB, uint8_t k
 *
 *  Return: mat33
 *----------------------------------------------------------------------------*/
mat33 matrix9x9_ctAB( const mat33 ctA, mat33 *ctB, uint8_t k)
{
    mat33 ctA_res0,ctB_res0,ctA_res1,ctB_res1,ctAB;
        
    /* make ctB matrix */
    ctB->m[0u][0u] = ctA.m[2u][0u];
    ctB->m[0u][1u] = ctA.m[2u][1u];
    ctB->m[0u][2u] = ctA.m[2u][2u];        
    ctB->m[1u][0u] = ctA.m[1u][0u];
    ctB->m[1u][1u] = ctA.m[1u][1u];
    ctB->m[1u][2u] = ctA.m[1u][2u];        
    ctB->m[2u][0u] = ctA.m[0u][0u];
    ctB->m[2u][1u] = ctA.m[0u][1u];
    ctB->m[2u][2u] = ctA.m[0u][2u];
        
     /* step No.1 */
     ctA_res0.m[0u][0u] = ctA.m[0u][0u];
     ctA_res0.m[0u][1u] = ctA.m[0u][1u];
     ctA_res0.m[0u][2u] = ctA.m[0u][2u];
     ctA_res0.m[1u][0u] = ctA.m[1u][1u];
     ctA_res0.m[1u][1u] = ctA.m[1u][2u];
     ctA_res0.m[1u][2u] = ctA.m[1u][0u];        
     ctA_res0.m[2u][0u] = ctA.m[2u][2u];
     ctA_res0.m[2u][1u] = ctA.m[2u][0u];
     ctA_res0.m[2u][2u] = ctA.m[2u][1u];        

     ctB_res0.m[0u][0u] = ctB->m[0u][0u];
     ctB_res0.m[0u][1u] = ctB->m[1u][1u];
     ctB_res0.m[0u][2u] = ctB->m[2u][2u];
     ctB_res0.m[1u][0u] = ctB->m[1u][0u];
     ctB_res0.m[1u][1u] = ctB->m[2u][1u];
     ctB_res0.m[1u][2u] = ctB->m[0u][2u];        
     ctB_res0.m[2u][0u] = ctB->m[2u][0u];
     ctB_res0.m[2u][1u] = ctB->m[0u][1u];
     ctB_res0.m[2u][2u] = ctB->m[1u][2u];

    switch(k)
    {        
        
          case 1u:
          /* step No.2 k=1 */
          ctA_res1.m[0u][0u] = ctA_res0.m[0u][1u];
          ctA_res1.m[0u][1u] = ctA_res0.m[0u][2u];
          ctB_res1.m[0u][2u] = ctA_res0.m[0u][0u];
          ctB_res1.m[1u][0u] = ctA_res0.m[1u][1u];
          ctB_res1.m[1u][1u] = ctA_res0.m[1u][2u];
          ctB_res1.m[1u][2u] = ctA_res0.m[1u][0u];        
          ctB_res1.m[2u][0u] = ctA_res0.m[2u][1u];
          ctB_res1.m[2u][1u] = ctA_res0.m[2u][2u];
          ctB_res1.m[2u][2u] = ctA_res0.m[2u][0u];        

          ctB_res1.m[0u][0u] = ctB_res0.m[1u][0u];
          ctB_res1.m[0u][1u] = ctB_res0.m[1u][1u];
          ctB_res1.m[0u][2u] = ctB_res0.m[1u][2u];
          ctB_res1.m[1u][0u] = ctB_res0.m[2u][0u];
          ctB_res1.m[1u][1u] = ctB_res0.m[2u][1u];
          ctB_res1.m[1u][2u] = ctB_res0.m[2u][2u];        
          ctB_res1.m[2u][0u] = ctB_res0.m[0u][0u];
          ctB_res1.m[2u][1u] = ctB_res0.m[0u][1u];
          ctB_res1.m[2u][2u] = ctB_res0.m[0u][2u];        
        
          /* step No.3 k=1 */
          ctAB.m[0u][0u] = (ctA_res1.m[0u][0u]*ctB_res1.m[0u][0u])+(ctA_res0.m[0u][0u]*ctB_res0.m[0u][0u]);
          ctAB.m[0u][1u] = (ctA_res1.m[0u][1u]*ctB_res1.m[0u][1u])+(ctA_res0.m[0u][1u]*ctB_res0.m[0u][1u]);
          ctAB.m[0u][2u] = (ctA_res1.m[0u][2u]*ctB_res1.m[0u][2u])+(ctA_res0.m[0u][2u]*ctB_res0.m[0u][2u]);
          ctAB.m[1u][0u] = (ctA_res1.m[1u][0u]*ctB_res1.m[1u][0u])+(ctA_res0.m[1u][0u]*ctB_res0.m[1u][0u]);
          ctAB.m[1u][1u] = (ctA_res1.m[1u][1u]*ctB_res1.m[1u][1u])+(ctA_res0.m[1u][1u]*ctB_res0.m[1u][1u]);
          ctAB.m[1u][2u] = (ctA_res1.m[1u][2u]*ctB_res1.m[1u][2u])+(ctA_res0.m[1u][2u]*ctB_res0.m[1u][2u]);        
          ctAB.m[2u][0u] = (ctA_res1.m[2u][0u]*ctB_res1.m[2u][0u])+(ctA_res0.m[2u][0u]*ctB_res0.m[2u][0u]);
          ctAB.m[2u][1u] = (ctA_res1.m[2u][1u]*ctB_res1.m[2u][1u])+(ctA_res0.m[2u][1u]*ctB_res0.m[2u][1u]);
          ctAB.m[2u][2u] = (ctA_res1.m[2u][2u]*ctB_res1.m[2u][2u])+(ctA_res0.m[2u][2u]*ctB_res0.m[2u][2u]);
          break;
          
          case 2u:
          /* step No.2 k=2 */
          ctA_res1.m[0u][0u] = ctA_res0.m[0u][2u];
          ctA_res1.m[0u][1u] = ctA_res0.m[0u][0u];
          ctB_res1.m[0u][2u] = ctA_res0.m[0u][1u];
          ctB_res1.m[1u][0u] = ctA_res0.m[1u][2u];
          ctB_res1.m[1u][1u] = ctA_res0.m[1u][0u];
          ctB_res1.m[1u][2u] = ctA_res0.m[1u][1u];        
          ctB_res1.m[2u][0u] = ctA_res0.m[2u][2u];
          ctB_res1.m[2u][1u] = ctA_res0.m[2u][0u];
          ctB_res1.m[2u][2u] = ctA_res0.m[2u][1u];        

          ctB_res1.m[0u][0u] = ctB_res0.m[2u][0u];
          ctB_res1.m[0u][1u] = ctB_res0.m[2u][1u];
          ctB_res1.m[0u][2u] = ctB_res0.m[2u][2u];
          ctB_res1.m[1u][0u] = ctB_res0.m[0u][0u];
          ctB_res1.m[1u][1u] = ctB_res0.m[0u][1u];
          ctB_res1.m[1u][2u] = ctB_res0.m[0u][2u];        
          ctB_res1.m[2u][0u] = ctB_res0.m[1u][0u];
          ctB_res1.m[2u][1u] = ctB_res0.m[1u][1u];
          ctB_res1.m[2u][2u] = ctB_res0.m[1u][2u];        
        
          /* step No.3 k=2 */
          ctAB.m[0u][0u] = (ctA_res1.m[0u][0u]*ctB_res1.m[0u][0u])+(ctA_res0.m[0u][0u]*ctB_res0.m[0u][0u]);
          ctAB.m[0u][1u] = (ctA_res1.m[0u][1u]*ctB_res1.m[0u][1u])+(ctA_res0.m[0u][1u]*ctB_res0.m[0u][1u]);
          ctAB.m[0u][2u] = (ctA_res1.m[0u][2u]*ctB_res1.m[0u][2u])+(ctA_res0.m[0u][2u]*ctB_res0.m[0u][2u]);
          ctAB.m[1u][0u] = (ctA_res1.m[1u][0u]*ctB_res1.m[1u][0u])+(ctA_res0.m[1u][0u]*ctB_res0.m[1u][0u]);
          ctAB.m[1u][1u] = (ctA_res1.m[1u][1u]*ctB_res1.m[1u][1u])+(ctA_res0.m[1u][1u]*ctB_res0.m[1u][1u]);
          ctAB.m[1u][2u] = (ctA_res1.m[1u][2u]*ctB_res1.m[1u][2u])+(ctA_res0.m[1u][2u]*ctB_res0.m[1u][2u]);        
          ctAB.m[2u][0u] = (ctA_res1.m[2u][0u]*ctB_res1.m[2u][0u])+(ctA_res0.m[2u][0u]*ctB_res0.m[2u][0u]);
          ctAB.m[2u][1u] = (ctA_res1.m[2u][1u]*ctB_res1.m[2u][1u])+(ctA_res0.m[2u][1u]*ctB_res0.m[2u][1u]);
          ctAB.m[2u][2u] = (ctA_res1.m[2u][2u]*ctB_res1.m[2u][2u])+(ctA_res0.m[2u][2u]*ctB_res0.m[2u][2u]);
          break;
          
          default:
          memcpy((void*)&ctAB,0,sizeof(ctAB));
          break;
          
        }
        return ctAB;
}
/*-----------------------------------------------------------------------------
 *   matrix33_transpose : transpose 9 byte stream or 3x3 matrix ct & the return result
 *
 *  Parameters: const mat33 ct
 *
 *  Return: mat33
 *----------------------------------------------------------------------------*/
mat33 matrix33_transpose( const mat33 ct )
{
   mat33 ct_T;
   ct_T.m[0u][0u] = ct.m[0u][0u];
   ct_T.m[0u][1u] = ct.m[1u][0u];
   ct_T.m[0u][2u] = ct.m[2u][0u];
   ct_T.m[1u][0u] = ct.m[0u][1u];
   ct_T.m[1u][1u] = ct.m[1u][1u];
   ct_T.m[1u][2u] = ct.m[2u][1u];        
   ct_T.m[2u][0u] = ct.m[0u][2u];
   ct_T.m[2u][1u] = ct.m[1u][2u];
   ct_T.m[2u][2u] = ct.m[2u][2u];
   return ct_T;        
}
/*-----------------------------------------------------------------------------
 *   matrix33_transpose2 : transpose 9 byte stream or 3x3 matrix ct into ct_T
 *
 *  Parameters: const mat33 ct, mat33 *ct_T
 *
 *  Return: mat33
 *----------------------------------------------------------------------------*/
void matrix33_transpose2( const mat33 ct, mat33 *ct_T )
{
   ct_T->m[0u][0u] = ct.m[0u][0u];
   ct_T->m[0u][1u] = ct.m[1u][0u];
   ct_T->m[0u][2u] = ct.m[2u][0u];
   ct_T->m[1u][0u] = ct.m[0u][1u];
   ct_T->m[1u][1u] = ct.m[1u][1u];
   ct_T->m[1u][2u] = ct.m[2u][1u];        
   ct_T->m[2u][0u] = ct.m[0u][2u];
   ct_T->m[2u][1u] = ct.m[1u][2u];
   ct_T->m[2u][2u] = ct.m[2u][2u];
}
/*  */
/*-----------------------------------------------------------------------------
 *   horner_equation : Horner's rule for polynomial evaluation
 *
 *  Parameters: float64_t x
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/ 
float64_t horner_equation( float64_t x )
{
  int16_t i,s;
  float64_t res = 0.0f;
  float64_t coeffs[4u] = { -19.0f, 7.0f, -4.0f, 6.0f };                         // THE EQUATION IS CODED AS SUCH e.g -19 + 7x -4x2 + 6x3 
 
  s = sizeof(coeffs)/sizeof(float64_t);
  for(i=s-1; i >= 0; i--)
  {
    res = res * x + coeffs[i];
  }
  return res;
}
/*-----------------------------------------------------------------------------
 *   horners_rule : Horner's rule for polynomial evaluation
 *
 *  Parameters: float64_t *coeffs, float64_t x
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t horners_rule(float64_t *coeffs, float64_t x)
{
  int16_t i,s;
  float64_t res = 0.0f;
 
  s = sizeof(*coeffs)/sizeof(float64_t);
  for(i=s-1; i >= 0; i--)
  {
    res = res * x + coeffs[i];
  }
  return res;
}
/*!@file FeatureMatching/Line.C Line segments algs */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// Ported by ACP Aviation
// $Id: $
//
/*-----------------------------------------------------------------------------
 *      Line_rotate : rotate line by theta
 *
 *  Parameters: float64_t theta, xyline_t * p1, xyline_t * p2
 *
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool Line_rotate(float64_t theta, xyline_t *p1, xyline_t *p2)
{

  float64_t sinTheta;
  float64_t cosTheta;
  float64_t mat[2u][2u];
  float64_t x, y;
  
  if ((p1==NULL) || (p2==NULL)) return false;
  sinTheta = sin(theta);
  cosTheta = cos(theta);
  mat[0u][0u] = cosTheta;
  mat[0u][1u] = -sinTheta;
  mat[1u][0u] = sinTheta;
  mat[1u][1u] = cosTheta;

  x = p1->i, y = p1->j;
  p1->i = x*mat[0u][0u] + y*mat[0u][1u];
  p1->j = x*mat[1u][0u] + y*mat[1u][1u];

  x = p2->i, y = p2->j;
  p2->i = x*mat[0u][0u] + y*mat[0u][1u];
  p2->j = x*mat[1u][0u] + y*mat[1u][1u];
  return true;
}
/*-----------------------------------------------------------------------------
 *      Line_shear : shear line by k1
 *
 *  Parameters: xyline_t * k1, xyline_t * p1, xyline_t * p2
 *
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool Line_shear(float64_t k1, float64_t k2, xyline_t *p1, xyline_t *p2)
{
  float64_t tmpP1i;
  float64_t tmpP1j;

  float64_t tmpP2i;
  float64_t tmpP2j;

  if ((p1==NULL) || (p2==NULL)) return false;
  tmpP1i = p1->i + p1->j*k2;
  tmpP1j = p1->i*k1 + p1->j;
  tmpP2i = p2->i + p2->j*k2;
  tmpP2j = p2->i*k1 + p2->j;
  
  p1->i = tmpP1i;
  p1->j = tmpP1j;
  p2->i = tmpP2i;
  p2->j = tmpP2j;
  return true;
}
/*-----------------------------------------------------------------------------
 *      Line_trans : transfrom line by p
 *
 *  Parameters: xyline_t * p, xyline_t * p1, xyline_t * p2
 *
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool Line_trans(xyline_t * p, xyline_t * p1, xyline_t * p2)
{
  if (((p1==NULL) || (p2==NULL)) || (p==NULL)) return false;
  p1->i += p->i;
  p1->j += p->j;
  p2->i += p->i;
  p2->j += p->j; 
  return true; 
}
/*-----------------------------------------------------------------------------
 *      Line_getCenter : get center
 *
 *  Parameters: const xyline_t p1, const xyline_t p2
 *
 *  Return:     xyline_t
 *----------------------------------------------------------------------------*/
xyline_t Line_getCenter(const xyline_t p1, const xyline_t p2 )
{
  xyline_t c;

  c.i = (p1.i+p2.i)/2.0f;
  c.j = (p1.j+p2.j)/2.0f;
  return c;
}
/*-----------------------------------------------------------------------------
 *      Line_getOri : get origin
 *
 *  Parameters: const xyline_t p1, const xyline_t p2
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t Line_getOri(const xyline_t p1, const xyline_t p2)
{
  float64_t theta = atan2(p2.j-p1.j,p2.i-p1.i);
  if (theta<0.0f)
    theta += PI;
  return theta;
}
/*-----------------------------------------------------------------------------
 *      Line_getLength  : get length
 *
 *  Parameters: const xyline_t p1, const xyline_t p2
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t Line_getLength(const xyline_t p1, const xyline_t p2)
{
  float64_t dx = p2.i - p1.i;
  float64_t dy = p2.j - p1.j;
  return sqrt(dx*dx + dy*dy);
}
/*-----------------------------------------------------------------------------
 *      Line_quantize
 *
 *  Parameters: int16_t numDirections, xyline_t * p1, xyline_t * p2, int16_t *itsDirectionIdx
 *
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool Line_quantize(int16_t numDirections, xyline_t * p1, xyline_t * p2, int16_t *itsDirectionIdx )
{
  xyline_t center; 
  xyline_t minus_center;
  float64_t ori, dOri;
  
  if ((p1==NULL) || ((p2==NULL) || (itsDirectionIdx==NULL))) return false;
  
  center = Line_getCenter(*p1, *p2 );
  minus_center.i = center.i * -1.0f;
  minus_center.j = center.j * -1.0f;
  if (Line_trans(&minus_center, p1, p2)==true)
  {
     ori = Line_getOri( *p1, *p2);
     *itsDirectionIdx =  (int16_t) floor((ori  *numDirections) / (PI+1e-5f));
     dOri = (((*itsDirectionIdx)*PI)/numDirections + PI/(2.0f*numDirections)) - ori;
     if (Line_rotate(dOri, p1, p2)==true)
     {         
        if (Line_trans(&center, p1, p2)==true)
          return true;
     }
  }
  return false;
}
/*-----------------------------------------------------------------------------
 *      Line_setQuantizedDir
 *
 *  Parameters: int16_t numDirections, const xyline_t p1,const xyline_t p2, int16_t *itsDirectionIdx
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Line_setQuantizedDir(int16_t numDirections, const xyline_t p1,const xyline_t p2, int16_t *itsDirectionIdx)
{
  float64_t ori = Line_getOri( p1, p2);
  *itsDirectionIdx = (int16_t) floor((ori * numDirections) / (PI+1e-5f));
}
/*-----------------------------------------------------------------------------
 *      invert_mat4x4 : invert 4x4 matrix
 *      source :  https://github.com/niswegmann
 *
 *  Parameters: const float32_t src[16u], float32_t *dst[16u]
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void invert_mat4x4(const float32_t src[16u], float32_t *dst[16u])
{
    float32_t det;

    *dst[0] =                                                                     /* Compute adjoint: */
        + src[ 5] * src[10] * src[15]
        - src[ 5] * src[11] * src[14]
        - src[ 9] * src[ 6] * src[15]
        + src[ 9] * src[ 7] * src[14]
        + src[13] * src[ 6] * src[11]
        - src[13] * src[ 7] * src[10];

    *dst[1] =
        - src[ 1] * src[10] * src[15]
        + src[ 1] * src[11] * src[14]
        + src[ 9] * src[ 2] * src[15]
        - src[ 9] * src[ 3] * src[14]
        - src[13] * src[ 2] * src[11]
        + src[13] * src[ 3] * src[10];

    *dst[2] =
        + src[ 1] * src[ 6] * src[15]
        - src[ 1] * src[ 7] * src[14]
        - src[ 5] * src[ 2] * src[15]
        + src[ 5] * src[ 3] * src[14]
        + src[13] * src[ 2] * src[ 7]
        - src[13] * src[ 3] * src[ 6];

    *dst[3] =
        - src[ 1] * src[ 6] * src[11]
        + src[ 1] * src[ 7] * src[10]
        + src[ 5] * src[ 2] * src[11]
        - src[ 5] * src[ 3] * src[10]
        - src[ 9] * src[ 2] * src[ 7]
        + src[ 9] * src[ 3] * src[ 6];

    *dst[4] =
        - src[ 4] * src[10] * src[15]
        + src[ 4] * src[11] * src[14]
        + src[ 8] * src[ 6] * src[15]
        - src[ 8] * src[ 7] * src[14]
        - src[12] * src[ 6] * src[11]
        + src[12] * src[ 7] * src[10];

    *dst[5] =
        + src[ 0] * src[10] * src[15]
        - src[ 0] * src[11] * src[14]
        - src[ 8] * src[ 2] * src[15]
        + src[ 8] * src[ 3] * src[14]
        + src[12] * src[ 2] * src[11]
        - src[12] * src[ 3] * src[10];

    *dst[6] =
        - src[ 0] * src[ 6] * src[15]
        + src[ 0] * src[ 7] * src[14]
        + src[ 4] * src[ 2] * src[15]
        - src[ 4] * src[ 3] * src[14]
        - src[12] * src[ 2] * src[ 7]
        + src[12] * src[ 3] * src[ 6];

    *dst[7] =
        + src[ 0] * src[ 6] * src[11]
        - src[ 0] * src[ 7] * src[10]
        - src[ 4] * src[ 2] * src[11]
        + src[ 4] * src[ 3] * src[10]
        + src[ 8] * src[ 2] * src[ 7]
        - src[ 8] * src[ 3] * src[ 6];

    *dst[8] =
        + src[ 4] * src[ 9] * src[15]
        - src[ 4] * src[11] * src[13]
        - src[ 8] * src[ 5] * src[15]
        + src[ 8] * src[ 7] * src[13]
        + src[12] * src[ 5] * src[11]
        - src[12] * src[ 7] * src[ 9];

    *dst[9] =
        - src[ 0] * src[ 9] * src[15]
        + src[ 0] * src[11] * src[13]
        + src[ 8] * src[ 1] * src[15]
        - src[ 8] * src[ 3] * src[13]
        - src[12] * src[ 1] * src[11]
        + src[12] * src[ 3] * src[ 9];

    *dst[10] =
        + src[ 0] * src[ 5] * src[15]
        - src[ 0] * src[ 7] * src[13]
        - src[ 4] * src[ 1] * src[15]
        + src[ 4] * src[ 3] * src[13]
        + src[12] * src[ 1] * src[ 7]
        - src[12] * src[ 3] * src[ 5];

    *dst[11] =
        - src[ 0] * src[ 5] * src[11]
        + src[ 0] * src[ 7] * src[ 9]
        + src[ 4] * src[ 1] * src[11]
        - src[ 4] * src[ 3] * src[ 9]
        - src[ 8] * src[ 1] * src[ 7]
        + src[ 8] * src[ 3] * src[ 5];

    *dst[12] =
        - src[ 4] * src[ 9] * src[14]
        + src[ 4] * src[10] * src[13]
        + src[ 8] * src[ 5] * src[14]
        - src[ 8] * src[ 6] * src[13]
        - src[12] * src[ 5] * src[10]
        + src[12] * src[ 6] * src[ 9];

    *dst[13] =
        + src[ 0] * src[ 9] * src[14]
        - src[ 0] * src[10] * src[13]
        - src[ 8] * src[ 1] * src[14]
        + src[ 8] * src[ 2] * src[13]
        + src[12] * src[ 1] * src[10]
        - src[12] * src[ 2] * src[ 9];

    *dst[14] =
        - src[ 0] * src[ 5] * src[14]
        + src[ 0] * src[ 6] * src[13]
        + src[ 4] * src[ 1] * src[14]
        - src[ 4] * src[ 2] * src[13]
        - src[12] * src[ 1] * src[ 6]
        + src[12] * src[ 2] * src[ 5];

    *dst[15] =
        + src[ 0] * src[ 5] * src[10]
        - src[ 0] * src[ 6] * src[ 9]
        - src[ 4] * src[ 1] * src[10]
        + src[ 4] * src[ 2] * src[ 9]
        + src[ 8] * src[ 1] * src[ 6]
        - src[ 8] * src[ 2] * src[ 5];

    det = + src[0] * *dst[0] + src[1] * *dst[4] + src[2] * *dst[8] + src[3] * *dst[12];     /* Compute determinant: */

    det = 1.0f / det;                                                           /* Multiply adjoint with reciprocal of determinant: */

    *dst[ 0] *= det;
    *dst[ 1] *= det;
    *dst[ 2] *= det;
    *dst[ 3] *= det;
    *dst[ 4] *= det;
    *dst[ 5] *= det;
    *dst[ 6] *= det;
    *dst[ 7] *= det;
    *dst[ 8] *= det;
    *dst[ 9] *= det;
    *dst[10] *= det;
    *dst[11] *= det;
    *dst[12] *= det;
    *dst[13] *= det;
    *dst[14] *= det;
    *dst[15] *= det;
}
/*The MIT License (MIT)

Copyright (c) 2016 T.E.A de Souza

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */
/*-----------------------------------------------------------------------------
 *      invf : inverse 4x4 matrix
 *
 *
 *  Parameters: int16_t *i, int16_t *j, const float64_t* m
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t invf(int16_t *i, int16_t *j, const float64_t* m){
    float64_t inv;
    int16_t o = 2+(*j-*i);
    *i += 4+o;
    *j += 4-o;
    #define e(a,b) (m[ (((*j+b)%4)*4 + ((*i+a)%4)) ])
    inv = (e(1,-1) * e(0,0) * e(-1,1))
      + (e(1,1) * e(0,-1) * e(-1,0))
      + (e(-1,-1) * e(1,0) * e(0,1))
      - (e(-1,-1) * e(0,0) * e(1,1))
      - (e(-1,1) * e(0,-1) * e(1,0))
      - (e(1,-1) * e(-1,0) * e(0,1)); 
    return (o%2)?inv : -inv;
    #undef e
}
/*-----------------------------------------------------------------------------
 *      inverseMatrix4x4 : inverse 4x4 matrix
 *
 *
 *  Parameters: const float64_t *m, float64_t *out
 *
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool inverseMatrix4x4(const float64_t *m, float64_t *out){
    float64_t inv[16u],D;
    int16_t i,j,k;
    for(i=0;i<4;i++) for(j=0;j<4;j++) inv[j*4+i] = invf(&i,&j,m);
    D = 0;
    for(k=0;k<4;k++) D += m[k] * inv[k*4];
    if (D == 0) return false;
    D = 1.0f / D;
    for (i = 0; i < 16; i++)
        out[i] = inv[i] * D;
    return true;
}
/* -------- functions for machine learning --------------
        Copyright © 2017 Sergei Bugrov. All rights reserved.
                Ported to mikroE C by ACP Aviation
*/

/*  Returns the value of the sigmoid function f(x) = 1/(1 + e^-x).
        Input: m1, a vector.
        Output: 1/(1 + e^-x) for every element of the input matrix m1.
                Original Author : Sergei Bugrov

*/
void ML_sigmoid(float64_t *in, uint16_t datSize, typeOfSigmoid_e sigAlgo, float64_t *out) 
{ 
    uint16_t i;
        
    for( i = 0u; i != (datSize*8u); i=+8u ) 
    {
            switch (sigAlgo)
            {
                  case SIG_EXP_FUNC:
                  *(out+i) = 1.0f / (1.0f + exp(- *(in+i)));
                  break;
                  
                  case SIG_D_FUNC:
                  *(out+i) = *(in+i) * (1.0f - *(in+i));
                  break;
                  
                  default:
                  break;
          }
    }
}
/*  Returns the value of the relu function g(z) = max{0, z} over the set of 0.0 and the input z        
    rectified linear activation function
        Input: m1, a vector. prime if not zero then make a relu prime array i.e consist of (1.0 or 0.0)
        Output: 1/(1 + e^-x) for every element of the input matrix m1.
                Original Author : Sergei Bugrov
                Ported to mikroE C by ACP Aviation
*/
void ML_relu(const float64_t *z[], float64_t *o[], uint16_t datSize, float64_t value, uint8_t prime)
{
    uint16_t i;
        
    for( i = 0; i < datSize; ++i ) 
    {
        if (*z[i] < value)
        {
            if (prime <= 0u)
            {        
               *o[i] = value;
            }
            else
           {
               *o[i] = 0.0f;
           }
        }
        else 
        {
            if (prime <= 0u)
            {                        
               *o[i] = *z[i];
            }
            else
            {
               *o[i] = 1.0f;
            }
        }
    }
}
/*-----------------------------------------------------------------------------
 *      ML_max:  return max of a floating Point number stream
 *
 *  Parameters: float64_t *z[], uint16_t datSize    
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
float64_t ML_max(float64_t *z[], uint16_t datSize)
{
    float64_t maxFlo=0.0f;
    uint16_t i;
        
    for( i = 0; i < datSize; ++i ) 
    {
        if (*z[i] > maxFlo)
        {
           maxFlo = *z[i];
        }
    }
    return maxFlo;
}

/* ============================================================================= 
     Returns the dot product of two matrices: m1 x m2.
     Inputs:
     m1: vector, left matrix of size m1_rows x m1_columns
     m2: vector, right matrix of size m1_columns x m2_columns (the number of rows in the right matrix
     must be equal to the number of the columns in the left one)
     m1_rows: int, number of rows in the left matrix m1
     m1_columns: int, number of columns in the left matrix m1
     m2_columns: int, number of columns in the right matrix m2
     Output: vector, m1 * m2, product of two vectors m1 and m2, a matrix of size m1_rows x m2_columns
   =============================================================================  
*/
void ML_dot(const float64_t* m1[], const float64_t* m2[], const int16_t m1_rows, const int16_t m1_columns, const int16_t m2_columns, float64_t *output[]) 
{  
    int16_t row,col,k;
        
    for( row = 0; row != m1_rows; ++row ) 
    {
        for( col = 0; col != m2_columns; ++col ) 
        {
            *output[ row * m2_columns + col ] = 0.f;
            for( k = 0; k != m1_columns; ++k ) 
            {
                *output[ row * m2_columns + col ] += *m1[ row * m1_columns + k ] * *m2[ k * m2_columns + col ];
            }
        }
    }
}

/*  ============================================================================
     Returns a transpose matrix of input matrix.
     Inputs:
     m: vector, input matrix
     C: int, number of columns in the input matrix
     R: int, number of rows in the input matrix
     Output: vector, transpose matrix mT of input matrix m
   =============================================================================
*/
void ML_transpose(float64_t *m[], const int16_t C, const int16_t R, float64_t *mT[]) 
{  
    uint8_t n,i,j;
        
    for(n = 0; n != C*R; n++) 
    {
        i = n/C;
        j = n%C;
        *mT[n] = *m[R*j + i];
    }

}
/* ============================================================================ 
     Returns softmax of input matrix.
     Inputs:
     * z[] vector, input matrix
     zsize output size
     dim input cloud dimension
     Output: *out[] *foo[]
  ============================================================================*/
void ML_softmax(const float64_t* z[], float64_t *out[], float64_t *foo[], uint16_t zsize, const int16_t dim) 
{
    
    float64_t max_foo;
    uint16_t i,j;
    float64_t sum_of_elems = 0.0f;
                
    for (i = 0; i != zsize; i += dim) 
    {
        for (j = 0; j != dim; ++j) 
        {
            *foo[i] = *z[i + j];
        }
        
        max_foo = ML_max(&z,zsize);

        for (j = 0; j != dim; ++j) 
        {
            *foo[j] = exp(*foo[j] - max_foo);
        }      


        for (j = 0; j != dim; ++j) 
        {
            sum_of_elems = sum_of_elems + *foo[j];
        }
        
        for (j = 0; j != dim; ++j) 
        {
            *out[j] = (*foo[j]/sum_of_elems);
        }
    }
}
/* ============================================================================
Code for reproducing key results in the paper "Neural Shuffle-Exchange Networks - 
Sequence Processing in O(n log n) Time" 
by Karlis Freivalds, Emils Ozolinš and Agris Šostaks
Institute of Mathematics and Computer Science
University of Latvia
Raina bulvaris 29, Riga, LV-1459, Latvia
{Karlis.Freivalds, Emils.Ozolins, Agris.Sostaks}@lumii.lv
  ============================================================================= */
/* We propose a neural network analogue of the Beneš network where we replace each switch with a
learnable 2-to-2 function */
/*-----------------------------------------------------------------------------
 *      ML_do_swap_half:  swap half (swaps j or y co-ord)
 *
 *  Parameters: xyline_t *s1, xyline_t *s2    
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void ML_do_swap_half( xyline_t *s1, xyline_t *s2 )
{
    float64_t rem = s1->j; 
    s1->j = s2->j;
    s2->j = rem;
}
/*-----------------------------------------------------------------------------
 *      ML_do_swap_otherhalf:  swap other half (swaps i or x co-ord)
 *
 *  Parameters: xyline_t *s1, xyline_t *s2    
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void ML_do_swap_otherhalf( xyline_t *s1, xyline_t *s2 )
{
    float64_t rem = s1->i; 
    s1->i = s2->i;
    s2->i = rem;
}
/*-----------------------------------------------------------------------------
 *      ML_calculate_U:  calculate U
 *
 *  Parameters: ML_weight_t Wus, ML_weight_t Bu, float64_t *output    
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void ML_calculate_U( ML_weight_t Wus, ML_weight_t Bu, float64_t *output )
{
   float64_t input[4u];
   input[0u] = Wus.a.i + Bu.a.i;
   input[1u] = Wus.a.j + Bu.a.j;
   input[2u] = Wus.b.i + Bu.b.i;
   input[3u] = Wus.b.j + Bu.b.j;
   ML_sigmoid(&input, 4u, SIG_EXP_FUNC, output);
}
/*-----------------------------------------------------------------------------
 *      ML_calc_adder:  calculate the value to add
 *
 *  Parameters: xyline_t *s1, xyline_t *s2, ML_weight_t Wus, ML_weight_t Bu, float64_t *adder    
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void ML_calc_adder( xyline_t *s1, xyline_t *s2, ML_weight_t Wus, ML_weight_t Bu, float64_t *adder )
{
   float64_t output[4u];
   ML_do_swap_half( s1, s2 );
   ML_calculate_U( Wus, Bu, &output );
   adder[0u] = output[0u] * s1->i;
   adder[1u] = output[1u] * s2->i;
   adder[2u] = output[2u] * s1->j;
   adder[3u] = output[3u] * s2->j;
}
/*
In the Switch Layer, we divide the cells into adjacent non-overlapping pairs and apply Switch Unit to
each pair3
The Switch Unit is similar to Gated Recurrent Unit(GRU) but it has two inputs [s1, s2]
and two outputs [s1o, s2o]. It contains two reset gates, one for each output. The reset gate performs the
computing logic of the unit(s and tanh nonlinearities just keep the values in range) and it is important
that each output uses a separate reset gate for the unit to produce unrelated outputs. Technically,
creating the pairs is implemented as reshaping the sequence s into a twice shorter sequence where
each new cell concatenates two adjacent cells [s1, s2] along the feature dimension. The Switch Unit
is defined as follows:
This is particularly useful where a common motif may reappear throughout the entire piece
*/ 
/*-----------------------------------------------------------------------------
 *      ML_calc_Shuffle_Exc:  perform shuffle
 *
 *  Parameters: ML_weight_t W1s, ML_weight_t B1r, ML_weight_t W2s, ML_weight_t B2r,     
 *  ML_weight_t B1c, ML_weight_t B2c, ML_weight_t Wus, ML_weight_t Bu, xyline_t *s1, 
 *  xyline_t *s2, float64_t *out_s1o, float64_t *out_s2o
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void ML_calc_Shuffle_Exc( ML_weight_t W1s, ML_weight_t B1r, ML_weight_t W2s, ML_weight_t B2r, ML_weight_t B1c, ML_weight_t B2c, ML_weight_t Wus, ML_weight_t Bu, xyline_t *s1, xyline_t *s2, float64_t *out_s1o, float64_t *out_s2o )
{
   float64_t output1[4u],output2[4u],output3[4u],adder[4u],tanVal1[4u],tanVal2[4u];
   ML_weight_t s;
   int8_t cnt;

   s.a.i = s1->i;                                                               /* read the input vectors */
   s.a.j = s1->j;
   s.b.i = s2->i;
   s.b.j = s2->j;

   ML_calculate_U( W1s, B1r, &output1 );                                        /* calculate sigmoids */
   ML_calculate_U( W2s, B2r, &output2 );
   ML_calculate_U( Wus, Bu, &output3 );

   output3[0u] = 1.0f - output3[0u];
   output3[1u] = 1.0f - output3[1u];
   output3[2u] = 1.0f - output3[2u];
   output3[3u] = 1.0f - output3[3u];

   tanVal1[0u] = tanh((output1[0u] * s.a.i)+B1c.a.i)*output3[0u];               /* do tanh function and Hadamard product */
   tanVal1[1u] = tanh((output1[1u] * s.a.j)+B1c.a.j)*output3[1u];
   tanVal1[2u] = tanh((output1[2u] * s.b.i)+B1c.b.i)*output3[2u];
   tanVal1[3u] = tanh((output1[3u] * s.b.j)+B1c.b.j)*output3[3u];

   tanVal2[0u] = tanh((output2[0u] * s.a.i)+B2c.a.i)*output3[0u];
   tanVal2[1u] = tanh((output2[1u] * s.a.j)+B2c.a.j)*output3[1u];
   tanVal2[2u] = tanh((output2[2u] * s.b.i)+B2c.b.i)*output3[2u];
   tanVal2[3u] = tanh((output2[3u] * s.b.j)+B2c.b.j)*output3[3u];

   ML_calc_adder( s1, s2, Wus, Bu, &adder );                                    /* calculate the results adding the factor variable */
   for (cnt=0u; cnt<4u ; cnt++)
   {
     out_s1o[cnt] = tanVal1[cnt] + adder[cnt];
     out_s2o[cnt] = tanVal2[cnt] + adder[cnt];
   }
}
/* =====================================================================
# uncompyle6 version 3.3.3
# Python bytecode 2.7 (62211)
# Decompiled from: Python 3.7.3 (v3.7.3:ef4ec6ed12, Mar 25 2019, 21:26:53) [MSC v.1916 32 bit (Intel)]
# Embedded file name: C:\Users\jfayr\flight\aviationFormula\aviationFormula.py
# Compiled at: 2019-05-19 12:59:54
*/
/*-----------------------------------------------------------------------------
 *   AVF_gcDistance : distance between 2 2d points 
 *
 *
 *  Parameters: float32_t ilat1,float32_t ilon1,float32_t ilat2,float32_t ilon2
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/
float32_t AVF_gcDistance(float32_t ilat1,float32_t ilon1,float32_t ilat2,float32_t ilon2)
{
    float32_t lat1 = DEGREE_TO_RADIAN(ilat1);
    float32_t lon1 = DEGREE_TO_RADIAN(ilon1);
    float32_t lat2 = DEGREE_TO_RADIAN(ilat2);
    float32_t lon2 = DEGREE_TO_RADIAN(ilon2);
    return (2.0f * asin(sqrt(pow(sin((lat1 - lat2) / 2.0f),2.0f) + cos(lat1) * cos(lat2) * pow(sin((lon1 - lon2) / 2.0f),2.0f))));
}
/*-----------------------------------------------------------------------------
 *   AVF_gcDistanceNm : distance between 2 2d points 
 *
 *
 *  Parameters: float32_t lat1,float32_t lon1,float32_t lat2,float32_t lon2
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/
float32_t AVF_gcDistanceNm(float32_t lat1,float32_t lon1,float32_t lat2,float32_t lon2)
{
    return (10800.0f / PI * AVF_gcDistance(lat1, lon1, lat2, lon2));
}
/*-----------------------------------------------------------------------------
 *   AVF_gcIntermediatePoint : intermediate point between 2 2d points 
 *
 *
 *  Parameters: float32_t dlat1, float32_t dlon1, float32_t dlat2, float32_t dlon2, float32_t *f
 *
 *  Return: GPS_Position_t*
 *----------------------------------------------------------------------------*/
GPS_Position_t* AVF_gcIntermediatePoint(float32_t dlat1, float32_t dlon1, float32_t dlat2, float32_t dlon2, float32_t *f)
{
    float32_t lat1 = DEGREE_TO_RADIAN(dlat1);
    float32_t lat2 = DEGREE_TO_RADIAN(dlat2);
    float32_t lon1 = DEGREE_TO_RADIAN(dlon1);
    float32_t lon2 = DEGREE_TO_RADIAN(dlon2);
    float32_t d,A,B,x,y,z;
    GPS_Position_t posn;
        
    if (*f<0.0f)
        *f = 0.5f;                                                              /* default if negative */
    d = AVF_gcDistance(lat1, lon1, lat2, lon2);
    A = (sin((1.0f - *f) * d) / sin(d));
    B = (sin(*f * d) / sin(d));
    x = (A * cos(lat1) * cos(lon1)) + B * cos(lat2) * cos(lon2);
    y = (A * cos(lat1) * sin(lon1)) + B * cos(lat2) * sin(lon2);
    z = A * sin(lat1) + B * sin(lat2);
    posn.latitude = atan2(z, sqrt((x * x) + (y * y)));
    posn.longditude = atan2(y, x);
    return &posn;
}
/*-----------------------------------------------------------------------------
 *   AVF_calcBearing : bearing between 2 2d points 
 *
 *
 *  Parameters: float32_t ilat1,float32_t ilon1,float32_t ilat2,float32_t ilon2
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/
float32_t AVF_calcBearing(float32_t ilat1,float32_t ilon1,float32_t ilat2,float32_t ilon2)
{
    float32_t lat1 = DEGREE_TO_RADIAN(ilat1);
    float32_t lat2 = DEGREE_TO_RADIAN(ilat2);
    float32_t lon1 = DEGREE_TO_RADIAN(ilon1);
    float32_t lon2 = DEGREE_TO_RADIAN(ilon2);
    float32_t y,x;
    float32_t dLon = lon2 - lon1;
    y = sin(dLon) * cos(lat2);
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    return atan2(y, x);        
}
/*
 *        g e t N o r m  from 
 *      Copyright (C) 2007-2008 by Hans Joachim Ferreau et al. All rights reserved
 */
float64_t getNormOfStream( const float64_t* const v, int16_t n )
{
        int16_t i;
        float64_t norm = 0.0f;

        for( i=0; i<n; ++i )
                norm += v[i]*v[i];
        return sqrt( norm );
}

#if defined(ALLAN_VAR_FUNCS)
/* Copyright (C) 2017 - 2018  Stéphane Guerrier and Roberto Molinari
 *
 * This file is part of av R Methods Package
 *
 * The `av` R package is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Ported to MikroE C by ACP Aviation
 */

/*-----------------------------------------------------------------------------
 *   returnAllenVarLen : return an interger equal to the allen variance matrix size 
 *
 *
 *  Parameters: uint16_t T
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/ 
uint16_t returnAllenVarLen( uint16_t T )
{
   return floor(log10((float64_t)(T))/log10(2.0f))-1.0f;                        // Create the number of halves possible and use it to find the number of clusters
}

//' @title Compute Maximal-Overlap Allan Variance using Means
//' @description Computation of Maximal-Overlap Allan Variance
//' @param x A \code{vector} with dimensions N x 1.
//' @return av A \code{list} that contains:
//' \itemize{
//'  \item{"clusters"}{The size of the cluster}
//'  \item{"allan"}{The Allan variance}
//'  \item{"errors"}{The error associated with the variance estimation.}
//' }
//' @details
//' Given \eqn{N} equally spaced samples with averaging time \eqn{\tau = n\tau _0}{tau = n*tau_0},
//' where \eqn{n} is an integer such that \eqn{ 1 \le n \le \frac{N}{2}}{1<= n <= N/2}.
//' Therefore, \eqn{n} is able to be selected from \eqn{\left\{ {n|n < \left\lfloor {{{\log }_2}\left( N \right)} \right\rfloor } \right\}}{{n|n< floor(log2(N))}}
//' Then, \eqn{M = N - 2n} samples exist.
//' The Maximal-overlap estimator is given by:
//' \eqn{\frac{1}{{2\left( {N - 2k + 1} \right)}}\sum\limits_{t = 2k}^N {{{\left[ {{{\bar Y}_t}\left( k \right) - {{\bar Y}_{t - k}}\left( k \right)} \right]}^2}} }
//'
//' where \eqn{ {{\bar y}_t}\left( \tau  \right) = \frac{1}{\tau }\sum\limits_{i = 0}^{\tau  - 1} {{{\bar y}_{t - i}}} }.
//' @author JJB
//' @references Long-Memory Processes, the Allan Variance and Wavelets, D. B. Percival and P. Guttorp
//' @examples
//' \donttest{
//' set.seed(999)
//' N = 100000
//' white.noise = rnorm(N, 0, 2)
//' random.walk = cumsum(0.1*rnorm(N, 0, 2))
//' combined.ts = white.noise+random.walk
//' av_mat = avar_mo_cpp(combined.ts)
//' }
//' @keywords internal
// [[Rcpp::export]]   
#define DIMENSION_3D 3u
/*-----------------------------------------------------------------------------
 *   avar_mo_cpp : return an interger equal to the allen variance matrix size 
 *
 *
 *  Parameters: const Vectr **Accel,const Vectr **Gyro, const uint16_t T, const uint16_t AllanVarlen, Vectr** out[DIMENSION_3D]
 *
 *  Return: void
 *----------------------------------------------------------------------------*/ 
void avar_mo_cpp(const Vectr **Accel,const Vectr **Gyro, const uint16_t T, const uint16_t AllanVarlen, Vectr** out[DIMENSION_3D]) 
{
   int8_t dim = 0;
   uint16_t i,j,k,z;
   uint16_t tau;  
   uint16_t M;
   /* 
      Vectr yBar[T];
      Vectr gBar[T];
      Vectr av[AllanVarlen*2,3u];       Allan Variance Matrix  */
   Vectr **yBar;
   Vectr **gBar;
   Vectr **av[3u];                                                              /* Allan Variance Matrix  */
   Vectr summed = {0.0f,0.0f,0.0f};
   Vectr summed1 = {0.0f,0.0f,0.0f}; 

   yBar = (Vectr **)Malloc((sizeof(Vectr))*T);                                  /* allocate dynamic memory to the size of the data passed into the function */
   gBar = (Vectr **)Malloc((sizeof(Vectr))*T);  
   av[0u] = (Vectr **)Malloc((sizeof(Vectr))*(AllanVarlen*2u));   
   av[1u] = (Vectr **)Malloc((sizeof(Vectr))*(AllanVarlen*2u));  
   av[2u] = (Vectr **)Malloc((sizeof(Vectr))*(AllanVarlen*2u));  
         
   memset(yBar,0.0f,T);                                                         // initialise arrays to zero
   memset(gBar,0.0f,T);
   
   for (i = 1; i <= AllanVarlen; i++)
   {
     tau = ((uint16_t)pow(2.0f,((float64_t)(i))));                              // Tau

     for(j = 0; j <= T-tau; j++)
     {
        for (z = j; z <= tau+j -1; z++)
        {
            yBar[j]->x += Accel[z]->x / tau;
            yBar[j]->y += Accel[z]->y / tau;
            yBar[j]->z += Accel[z]->z / tau;
            gBar[j]->x += Gyro[z]->x / tau;
            gBar[j]->y += Gyro[z]->y / tau;
            gBar[j]->z += Gyro[z]->z / tau;
        }
     }

    M = T-2*tau;                                                                // Clusters
    for(k = 0; k < M; k++)
    {
           summed.x +=  pow(yBar[k]->x - yBar[k+tau]->x,2.0f);
           summed.y +=  pow(yBar[k]->y - yBar[k+tau]->y,2.0f);
           summed.z +=  pow(yBar[k]->z - yBar[k+tau]->z,2.0f);
           summed1.x +=  pow(gBar[k]->x - gBar[k+tau]->x,2.0f);
           summed1.y +=  pow(gBar[k]->y - gBar[k+tau]->y,2.0f);
           summed1.z +=  pow(gBar[k]->z - gBar[k+tau]->z,2.0f);
    }

     av[i-1u][0u]->x = tau;                                                     // Cluster size
     av[i-1u][1u]->x = summed.x/(2.0f*(T - 2.0f*tau + 1.0f));                   // Compute the Allan Variance estimate
     av[i-1u][2u]->x = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) );       // Compute Error
        
     av[i-1u][0u]->y = tau;                                                     // Cluster size
     av[i-1][1u]->y = summed.y/(2.0f*(T - 2.0f*tau + 1.0f));                    // Compute the Allan Variance estimate
     av[i-1u][2u]->y = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) );       // Compute Error
         
     av[i-1u][0u]->z = tau;                                                     // Cluster size
     av[i-1u][1u]->z = summed.z/(2.0f*(T - 2.0f*tau + 1.0f));                   // Compute the Allan Variance estimate
     av[i-1u][2u]->z = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) );       // Compute Error
         
     av[i-1u+AllanVarlen][0u]->x = tau;                                         // Cluster size
     av[i-1u+AllanVarlen][1u]->x = summed1.x/(2.0f*(T - 2.0f*tau + 1.0f));      // Compute the Allan Variance estimate
     av[i-1u+AllanVarlen][2u]->x = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) );       // Compute Error
        
     av[i-1u+AllanVarlen][0u]->y = tau;                                         // Cluster size
     av[i-1u+AllanVarlen][1u]->y = summed1.y/(2.0f*(T - 2.0f*tau + 1.0f));      // Compute the Allan Variance estimate
     av[i-1u+AllanVarlen][2u]->y = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) ); // Compute Error
         
     av[i-1u+AllanVarlen][0u]->z = tau;                                         // Cluster size
     av[i-1u+AllanVarlen][1u]->z = summed1.z/(2.0f*(T - 2.0f*tau + 1.0f));      // Compute the Allan Variance estimate
     av[i-1u+AllanVarlen][2u]->z = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) );  // Compute Error
  }

  out = av;                                                                     // Return as a 2D/3D vector for acceleration followed by Gyro

  Free((char*)yBar,((sizeof(Vectr))*T));                                        // free the pointers malloced at the start
  Free((char*)gBar,((sizeof(Vectr))*T));  
  for (dim=0;dim<=DIMENSION_3D;++dim)
  { 
     Free((char*)av[dim],((sizeof(Vectr))*(AllanVarlen*2u)));  
  }   
}

/*-----------------------------------------------------------------------------
 *   avar_getMaxyBarSize : return an interger equal to the allen variance matrix size 
 *
 *
 *  Parameters: uint16_t AllanVarLen, uint16_t T
 *
 *  Return: uint16_t
 *----------------------------------------------------------------------------*/
uint16_t avar_getMaxyBarSize( uint16_t AllanVarLen, uint16_t T )
{
   float64_t tau = ((uint16_t)pow(2.0f,((float64_t)(AllanVarLen))));
   return ((uint16_t)(((float64_t) T) / tau)); 
}

//' @title Compute Tau-Overlap Allan Variance
//' @description Computation of Tau-Overlap Allan Variance
//' @param x A \code{vector} with dimensions N x 1.
//' @return av A \code{matrix} that contains:
//' \itemize{
//'  \item{Col 1}{The size of the cluster}
//'  \item{Col 2}{The Allan variance}
//'  \item{Col 3}{The error associated with the variance estimation.}
//' }
//' @details
//' Given \eqn{N} equally spaced samples with averaging time \eqn{\tau = n\tau _0}{tau = n*tau_0},
//' where \eqn{n} is an integer such that \eqn{ 1 \le n \le \frac{N}{2}}{1<= n <= N/2}.
//' Therefore, \eqn{n} is able to be selected from \eqn{\left\{ {n|n < \left\lfloor {{{\log }_2}\left( N \right)} \right\rfloor } \right\}}{{n|n< floor(log2(N))}}
//' Then, a sampling of \eqn{m = \left\lfloor {\frac{{N - 1}}{n}} \right\rfloor  - 1} samples exist.
//' The tau-overlap estimator is given by:
//'
//' where \eqn{ {{\bar y}_t}\left( \tau  \right) = \frac{1}{\tau }\sum\limits_{i = 0}^{\tau  - 1} {{{\bar y}_{t - i}}} }.
//'
//' @author JJB
//' @references Long-Memory Processes, the Allan Variance and Wavelets, D. B. Percival and P. Guttorp
//' @examples
//' \donttest{
//' set.seed(999)
//' N = 100000
//' white.noise = rnorm(N, 0, 2)
//' random.walk = cumsum(0.1*rnorm(N, 0, 2))
//' combined.ts = white.noise+random.walk
//' av_mat = avar_to_cpp(combined.ts)
//' }
//' @keywords internal
// [[Rcpp::export]] 

/*-----------------------------------------------------------------------------
 *   avar_to_cpp : Compute Tau-Overlap Allan Variance 
 *
 *
 *  Parameters: Vectr **Accel, Vectr **Gyro, uint16_t T, uint16_t AllanVarlen, uint16_t maxBarLen, Vectr **out[DIMENSION_3D]
 *
 *  Return: void
 *----------------------------------------------------------------------------*/       
void avar_to_cpp(const Vectr **Accel,const Vectr **Gyro,const uint16_t T,const uint16_t AllanVarlen,const uint16_t maxBarLen, Vectr **out[DIMENSION_3D]) 
{
   uint16_t i,j,z,k;                                                            // Length of vector
   uint16_t tau;  
   uint16_t M;
   /* Vectr yBar[maxBarLen];
   Vectr gBar[maxBarLen];
   Vectr av[AllanVarlen*2,3u];      // Allan Variance Matrix */
   Vectr **gBar;
   Vectr **yBar;
   Vectr **av[DIMENSION_3D];
   Vectr summed = {0.0f,0.0f,0.0f};
   Vectr summed1 = {0.0f,0.0f,0.0f}; 
   int8_t dim = 0;

   yBar = (Vectr **)Malloc((sizeof(Vectr))*maxBarLen);                          /* allocate dynamic memory to the size of the data passed into the function */
   gBar = (Vectr **)Malloc((sizeof(Vectr))*T);  
   av[0u] = (Vectr **)Malloc((sizeof(Vectr))*(AllanVarlen*2u));   
   av[1u] = (Vectr **)Malloc((sizeof(Vectr))*(AllanVarlen*2u));  
   av[2u] = (Vectr **)Malloc((sizeof(Vectr))*(AllanVarlen*2u));  
   
   memset(yBar,0.0f,T);                                                         // initialise arrays to zero
   memset(gBar,0.0f,T);

   for (i = 1; i <= AllanVarlen; i++)
   {
     tau = ((uint16_t)pow(2.0f,((float64_t)(i))));                              // Tau

     for(j = 0; j < maxBarLen;j++)
     {
        for (z = j; z <= tau*j+tau - 1; z++)                                    // Y.Bar
        {
            yBar[j]->x += Accel[z]->x / tau;
            yBar[j]->y += Accel[z]->y / tau;
            yBar[j]->z += Accel[z]->z / tau;
            gBar[j]->x += Gyro[z]->x / tau;
            gBar[j]->y += Gyro[z]->y / tau;
            gBar[j]->z += Gyro[z]->z / tau;
        }
     }

     M = floor(((float64_t)(T/(2.0f*tau))) );                                   // Clusters
     for(k = 0; k < M; k++)
     {
         summed.x += pow(yBar[2u*k+1u]->x - yBar[2u*k]->x,2.0f);
         summed.y += pow(yBar[2u*k+1u]->y - yBar[2u*k]->y,2.0f);
         summed.z += pow(yBar[2u*k+1u]->z - yBar[2u*k]->z,2.0f);
         summed1.x += pow(gBar[2u*k+1u]->x - gBar[2u*k]->x,2.0f);
         summed1.y += pow(gBar[2u*k+1u]->y - gBar[2u*k]->y,2.0f);
         summed1.z += pow(gBar[2u*k+1u]->z - gBar[2u*k]->z,2.0f);
     }
         
     av[0u][i-1u]->x = tau;                                                     // Cluster size
     av[1u][i-1u]->x = summed.x/(2.0f*M);                                       // Compute the Allan Variance estimate
     av[2u][i-1u]->x = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) );       // Compute Error
         
     av[0u][i-1u]->y = tau;                                                     // Cluster size
     av[1u][i-1u]->y = summed.y/(2.0f*M);                                       // Compute the Allan Variance estimate
     av[2u][i-1u]->y = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) );       // Compute Error
         
     av[0u][i-1u]->z = tau;                                                     // Cluster size
     av[1u][i-1u]->z = summed.z/(2.0f*M);                                       // Compute the Allan Variance estimate
     av[2u][i-1u]->z = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) );       // Compute Error
         
     av[0u][i-1u+AllanVarlen]->x = tau;                                         // Cluster size
     av[1u][i-1u+AllanVarlen]->x = summed1.x/(2.0f*M);                          // Compute the Allan Variance estimate
     av[2u][i-1u+AllanVarlen]->x = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) ); // Compute Error
        
     av[0u][i-1u+AllanVarlen]->y = tau;                                         // Cluster size
     av[1u][i-1u+AllanVarlen]->y = summed1.y/(2.0f*M);                          // Compute the Allan Variance estimate
     av[2u][i-1u+AllanVarlen]->y = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) ); // Compute Error
         
     av[0u][i-1u+AllanVarlen]->z = tau;                                         // Cluster size
     av[1u][i-1u+AllanVarlen]->z = summed1.z/(2.0f*M);                          // Compute the Allan Variance estimate
     av[2u][i-1u+AllanVarlen]->z = 1.0f/sqrt(2.0f*( (((float64_t)(T))/tau) - 1.0f) ); // Compute Error
         
  }

  out = av;                                                                     // Return as vector for acceleration followed by vector for Gyro

  Free((char*)yBar,((sizeof(Vectr))*maxBarLen));                                // free the pointers malloced at the start
  Free((char*)gBar,((sizeof(Vectr))*T));  
  for (dim=0;dim<=DIMENSION_3D;++dim)
  { 
     Free((char*)av[dim],((sizeof(Vectr))*(AllanVarlen*2u)));  
  }  
}
#endif

/* Ported from python by Aliaksei Chapyzhenka
   By ACP Aviation
   
     The following formulas are from
 *   R. Lundin, 
 *   "A Handbook Formula for the Inductance of a Single-Layer Circular Coil", 
 *   Proc. IEEE, vol. 73, no. 9, pp. 1428-1429, Sep. 1985
 */

/* IndCalc(a, b, n) - returns the inductance of a multiturn single layer circular coil
 * a: coil radius [m] 
 * b: coil length [m] 
 * n: number of turns 
 */
/*-----------------------------------------------------------------------------
 *      Induc_f1  Inductance function 1  
 *
 *  Parameters: float64_t x
 *
 *  Return:  float64_t
 *----------------------------------------------------------------------------*/
float64_t Induc_f1( float64_t x ) 
{
   return(1.0f + x * (0.383901f + 0.017108f * x)) / (1.0f + 0.258952f * x);
}
/*-----------------------------------------------------------------------------
 *      Induc_f2  Inductance function 2  
 *
 *  Parameters: float64_t x
 *
 *  Return:  float64_t
 *----------------------------------------------------------------------------*/
float64_t Induc_f2( float64_t x ) 
{
   return(x * (0.093842f + x * (0.002029f - x * 0.000801f)));
}
/* L Inductance [nH] (Result) */  
/*-----------------------------------------------------------------------------
 *      InductanceCalc  Inductance [nH] (Result)  
 *
 *  Parameters: float64_t a, float64_t b, float64_t n
 *
 *  Return:  float64_t
 *----------------------------------------------------------------------------*/
float64_t InductanceCalc( float64_t a, float64_t b, float64_t n ) 
{
  const float64_t u0 = 0.4f * PI * 1e-06f;
  const float64_t n2 = n * n;
  const float64_t shape_ratio = 2.0f * a / b;
  const float64_t shape_ratio_pwr2 = shape_ratio * shape_ratio;
  float64_t ret;
  if (shape_ratio <= 1.0f) 
  {
    ret = u0 * n2 * PI * a * a * (Induc_f1(shape_ratio_pwr2) - (4.0f / (3.0f * PI)) * shape_ratio) / b;
  }
  else
  {
    ret = u0 * n2 * a * ((log(4.0f * shape_ratio) - 0.5f) * Induc_f1(1.0f / shape_ratio_pwr2) + Induc_f2(1.0f / shape_ratio_pwr2));
  }
  return ret;
}
/*   Inductor Q (Result)
 *   The following formula is from
 *   F. Langford-Smith (ed.),
 *   "Radiotron Designer's Handbook", 4th edition
 *   Wireless Press, 1952.
 */
/*-----------------------------------------------------------------------------
 *      InductorCoilQ  Inductor Q (Result) 
 *
 *  Parameters: float64_t a, float64_t b, float64_t f
 *
 *  Return:  float64_t
 *----------------------------------------------------------------------------*/
float64_t InductorCoilQ(float64_t a, float64_t b, float64_t f) 
{
  return(sqrt(f) / (0.069f / a + 0.054f / b));
}
/*-----------------------------------------------------------------------------
 *  quick_list:  reverse the list or quickly sort the array 
 *
 *  Parameters: int16_t *temp_array3, int8_t order 
 *              
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void quick_list(int16_t *temp_array3, int8_t order)
{
    int16_t i = 0;
    int16_t a = 0;
    int16_t step = sizeof(int16_t);

    switch(order)
    {
        case 0:                
        for (i = 0; i < sizeof(*temp_array3); i+=step)                          //Bubble sort algorithm to sort the numbers in increasing order
        {
            if (*(temp_array3+i) > *(temp_array3+(i+step)))                     //Find the smallest number
            {
                a =  *(temp_array3+i);                                          //Store smallest number
                *(temp_array3+i) = *(temp_array3+(i+step));                     //Swap smallest number with current number
                *(temp_array3+(i+step)) = a;                                    //Swap number back to array 
            }
        }
        break;
                
        case 1:
        for (i = 0; i < sizeof(*temp_array3); i+=step)                          //Bubble sort algorithm to sort the numbers in decreasing order
        {
            if (*(temp_array3+i) < *(temp_array3+(i+step)))                     //Find the largest number
            {
                a =  *(temp_array3+i);                                          //Store largest number
                *(temp_array3+i) = *(temp_array3+(i+step));                     //Swap largest number with current number
                *(temp_array3+(i+step)) = a;                                    //Swap number back to array 
            }
        }
        break;
    }
}
/*-----------------------------------------------------------------------------
 *  bubble_sort_array:  Bubble sort array 
 *
 *  Parameters: int16_t *temp_array3, int8_t order 
 *              
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void bubble_sort_array(int16_t *temp_array3, int8_t order)
{
    int16_t i = 0;
    int16_t j = 0;
    int16_t a = 0;
    int16_t step = sizeof(int16_t);

    switch(order)
    {
        case 0:                
        for (i = 0; i < sizeof(*temp_array3); i+=step)                          //Bubble sort algorithm to sort the numbers in increasing order
        {
            for (j =0; j < sizeof(*temp_array3); j+=step) 
            {
               if (*(temp_array3+i) > *(temp_array3+j) )                        //Find the smallest number
               {
                  a =  *(temp_array3+i);                                        //Store smallest number
                  *(temp_array3+i) = *(temp_array3+j);                          //Swap smallest number with current number
                  *(temp_array3+j) = a;                                         //Swap number back to array 
               }
            }
        }
        break;
                
        case 1:
        for (i = 0; i < sizeof(*temp_array3); i+=step)                          //Bubble sort algorithm to sort the numbers in decreasing order
        {
            for (j =0; j < sizeof(*temp_array3); j+=step) 
            {
               if (*(temp_array3+i) < *(temp_array3+j) )                        //Find the largest number
               {
                  a =  *(temp_array3+i);                                        //Store largest number
                  *(temp_array3+i) = *(temp_array3+j);                          //Swap largest number with current number
                  *(temp_array3+j) = a;                                         //Swap number back to array 
               }
            }
        }
        break;
    }
}

// Matlab + Octave style transfer function conversion function
// All function behaviour is very similar to octave
// ref : James Chung

/*-----------------------------------------------------------------------------
 *  swap_double:  swap double 
 *
 *  Parameters: float64_t* a, float64_t* b 
 *              
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void swap_double(float64_t* a, float64_t* b)
{
        float64_t t = *a;
        *a = *b;
        *b = t;
}

/*-----------------------------------------------------------------------------
 *  selectionSort_double:  sort double
 *
 *  Parameters: float64_t arr[], int16_t n 
 *              
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void selectionSort_double(float64_t arr[], int16_t n)
{
        int16_t i, j, min_idx;
        for (i = 0; i < n - 1; i++)
        {
                min_idx = i;
                for (j = i + 1; j < n; j++)
                        if (arr[j] < arr[min_idx])
                                min_idx = j;
                swap_double(&arr[min_idx], &arr[i]);
        }
}

/*-----------------------------------------------------------------------------
 *  selectionSortAux_double:  sort double
 *
 *  Parameters: float64_t arr[], float64_t arr2[], int16_t n 
 *              
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void selectionSortAux_double(float64_t arr[], float64_t arr2[], int16_t n)
{
        int16_t i, j, min_idx;
        for (i = 0; i < n - 1; i++)
        {
                min_idx = i;
                for (j = i + 1; j < n; j++)
                        if (arr[j] < arr[min_idx])
                                min_idx = j;
                swap_double(&arr[min_idx], &arr[i]);
                swap_double(&arr2[min_idx], &arr2[i]);
        }
}

/*-----------------------------------------------------------------------------
 *  diff_double:  difference between adjacent values in float stream
 *
 *  Parameters: float64_t *y, float64_t *f, int16_t  sz
 *              
 *  Return: (none)
 *----------------------------------------------------------------------------*/
void diff_double(float64_t *y, float64_t *f, int16_t  sz)
{
        int16_t  i;
        --sz;
        for (i = 0; i < sz; i++)
                f[i] = y[i + 1] - y[i];
}

/*-----------------------------------------------------------------------------
 *  isneg_double:  true if double stream contains a negative
 *
 *  Parameters: float64_t *y, int16_t  sz 
 *              
 *  Return: int16_t
 *----------------------------------------------------------------------------*/
int16_t isneg_double(float64_t *y, int16_t  sz)
{
        int16_t  i;
        for (i = 0; i < sz; i++)
                if (y[i] < 0) return 1;
        return 0;
}

/*
 * polygon.cpp
 * Copyright (C) Andrew Tridgell 2011  ref: Anton Babuskin
 * ported to mikroE C -- ACP aviation 
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  The point in polygon algorithm is based on:
  http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
*/


/*
  Polygon_outside(): test for a point in a polygon
      Input:   P = a point,
               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
      Return:  true if P is outside the polygon
  This does not take account of the curvature of the earth, but we
  expect that to be very small over the distances involved in the
  fence boundary
*/
uint8_t Polygon_outside(const u32point_t P, const u32point_t **V, uint16_t n)
{
    uint16_t i, j;
    int8_t dx1s, dx2s, dy1s, dy2s, m1, m2;
    int32_t dx1, dx2, dy1, dy2;
                
    uint8_t outside = true;
    for (i = 0, j = n-1; i < n; j = i++) {
        if ((V[i]->y > P.y) == (V[j]->y > P.y)) {
            continue;
        }
        dx1 = P.x - V[i]->x;
        dx2 = V[j]->x - V[i]->x;
        dy1 = P.y - V[i]->y;
        dy2 = V[j]->y - V[i]->y;
#define poly_sign(x) ((x)<0?-1:1)
        dx1s = poly_sign(dx1);
        dx2s = poly_sign(dx2);
        dy1s = poly_sign(dy1);
        dy2s = poly_sign(dy2);
        m1 = dx1s * dy2s;
        m2 = dx2s * dy1s;
        // we avoid the 64 bit multiplies if we can based on sign checks.
        if (dy2 < 0) {
            if (m1 > m2) {
                outside = !outside;
            } else if (m1 < m2) {
                continue;
            } else if ( dx1 * (int64_t)dy2 > dx2 * (int64_t)dy1 ) {
                outside = !outside;
            }
        } else {
            if (m1 < m2) {
                outside = !outside;
            } else if (m1 > m2) {
                continue;
            } else if ( dx1 * (int64_t)dy2 < dx2 * (int64_t)dy1 ) {
                outside = !outside;
            }
        }            
    }
    return outside;
}
/*
  check if a polygon is complete. 
  We consider a polygon to be complete if we have at least 4 points,
  and the first point is the same as the last point. That is the
  minimum requirement for the Polygon_outside function to work
 */
uint32_t Polygon_complete(const u32point_t **V, uint32_t n)
{
    return (n >= 4ul && V[n-1u]->x == V[0u]->x && V[n-1u]->y == V[0u]->y);
}
/* root mean squared error */
dVectr* do_rmse_double( dVectr *a, dVectr *b, uint16_t num_points )
{
    int8_t i;
    dVectr diff = {0.0f, 0.0f, 0.0f};
    dVectr avg_diff;
        
    for (i=0; i < num_points; i++)
    {
        diff.x += pow((a[i]->x - b[i]->x),2.0f);
        diff.y += pow((a[i]->y - b[i]->y),2.0f);
        diff.z += pow((a[i]->z - b[i]->z),2.0f);                
    }
    avg_diff.x = sqrt(diff.x / num_points);
    avg_diff.y = sqrt(diff.y / num_points);
    avg_diff.z = sqrt(diff.z / num_points);        
    return &avg_diff;
}
/* root mean squared error between vector points */
dVectr* do_rmse_vp_double( dVectr *a, uint16_t num_points )
{
    int8_t i;
    dVectr diff = {0.0f, 0.0f, 0.0f};
    dVectr avg_diff;
        
    for (i=0; i < num_points; i++)
    {
        diff.x += pow(abs(a[i]->x - a[i]->y),2.0f);
        diff.y += pow(abs(a[i]->y - a[i]->z),2.0f);
        diff.z += pow(abs(a[i]->z - a[i]->x),2.0f);                
    }
    avg_diff.x = sqrt(diff.x / num_points);
    avg_diff.y = sqrt(diff.y / num_points);
    avg_diff.z = sqrt(diff.z / num_points);        
    return &avg_diff;
}
/* root mean squared error between adjacent points */
dVectr* do_rmse_ap_double( dVectr *a, uint16_t num_points )
{
    int8_t i;
    dVectr diff = {0.0f, 0.0f, 0.0f};
    dVectr avg_diff;
        
    for (i=1; i < num_points; i++)
    {
        diff.x += pow((a[i-1u]->x - a[i]->x),2.0f);
        diff.y += pow((a[i-1u]->y - a[i]->y),2.0f);
        diff.z += pow((a[i-1u]->z - a[i]->z),2.0f);                
    }
    avg_diff.x = sqrt(diff.x / num_points);
    avg_diff.y = sqrt(diff.y / num_points);
    avg_diff.z = sqrt(diff.z / num_points);        
    return &avg_diff;
}
/* initialise the rmse for below */
void set_rmse_iter_double( dVectr *diff )
{
        diff->x = 0.0f;
        diff->y = 0.0f;
        diff->z = 0.0f;
}
/* root mean squared error differnce initailised before and iterated more reactive */
void do_rmse_iter_double( dVectr *a, dVectr *b, uint16_t num_points, dVectr *diff )
{
    int8_t i;
        
    for (i=0; i < num_points; i++)
    {
        diff->x += pow((a[i]->x - b[i]->x),2.0f);
        diff->y += pow((a[i]->y - b[i]->y),2.0f);
        diff->z += pow((a[i]->z - b[i]->z),2.0f);                
    }
    diff->x = sqrt(diff->x / (num_points+1u));
    diff->y = sqrt(diff->y / (num_points+1u));
    diff->z = sqrt(diff->z / (num_points+1u));        
}
/* root mean squared error differnce initailised before and iterated less reactive */
void do_rmse_retain_double( dVectr *a, dVectr *b, uint16_t num_points, dVectr *diff )
{
    int8_t i;
    dVectr diff1; 
             
    for (i=0; i < num_points; i++)
    {
        diff1.x += pow((a[i]->x - b[i]->x),2.0f);
        diff1.y += pow((a[i]->y - b[i]->y),2.0f);
        diff1.z += pow((a[i]->z - b[i]->z),2.0f);                
    }
    diff->x += sqrt(diff1.x / (num_points));
    diff->y += sqrt(diff1.y / (num_points));
    diff->z += sqrt(diff1.z / (num_points));        
    diff->x /= 2.0f;
    diff->y /= 2.0f;    
    diff->z /= 2.0f;    
}
#if defined(FFT_NOTCH_REQ)
//
//  Frequency estimation using Barry Quinn's Second Estimator
//  Adapted from following links:
//  DSP Guru site:
//  http://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak
//  Vadym Markov's adaptation:
//  https://github.com/vadymmarkov/Beethoven/blob/master/Source/Estimation/Strategies/QuinnsSecondEstimator.swift
//  https://gist.github.com/hiromorozumi/f74fd4d5592a7f79028560cb2922d05f
//
//  sampleRate ... your sample rate
//  k          ... bin index that shows biggest FFT output magnitude
//  N          ... number of FFT points
//  out[k][0]  ... real part of FFT output at bin k
//  out[k][1]  ... imaginary part of FFT output at bin k

// ...
// perform complex FFT here...
// helper function used for frequency estimation        
/*-----------------------------------------------------------------------------
 *   quinn_tau:  tau function for quinn estimation
 *
 *  Parameters: float64_t x
 *  Return: void
 *----------------------------------------------------------------------------*/        
float64_t quinn_tau(float64_t x)
{
        float64_t p1 = log(3 * pow(x, 2.0f) + 6.0f * x + 1.0f);
        float64_t part1 = x + 1.0f - sqrt(2.0f/3.0f);
        float64_t part2 = x + 1.0f + sqrt(2.0f/3.0f);
        float64_t p2 = log(part1 / part2);        
        return (1.0f/4.0f * p1 - sqrt(6.0f) / 24.0f * p2);
}
/*-----------------------------------------------------------------------------
 *   do_quinn_est:  do quinn extimator
 *
 *  Parameters: quinn_est_t *quin, float64_t sampleRate, float64_t peakPosIndex, float64_t N, float64_t **out
 *  Return: void
 *----------------------------------------------------------------------------*/
void do_quinn_est( quinn_est_t *quin, float64_t sampleRate, float64_t peakPosIndex, float64_t N, float64_t **out )
{
        
   int16_t k = peakPosIndex;                
   quin->divider = pow(out[k][0u], 2.0f) + pow(out[k][1u], 2.0f);
   quin->ap = (out[k+1u][0u] * out[k][0u] + out[k+1u][1u] * out[k][1u]) / quin->divider;
   quin->dp = -quin->ap  / (1.0f - quin->ap);
   quin->am = (out[k-1u][0u] * out[k][0u] + out[k-1u][1u] * out[k][1u]) / quin->divider;

   quin->dm = quin->am / (1.0f - quin->am);
   quin->d = ((quin->dp + quin->dm) / 2.0f) + quinn_tau((quin->dp * quin->dp)) - quinn_tau((quin->dm * quin->dm));

   if (quin->d < -0.5f) quin->d = -0.5f;                                        /* constrain d between 0.5 and -0.5 */
   else if (quin->d > 0.5f) quin->d = 0.5f;
   
   quin->adjustedBinLocation = (((float64_t)k) + quin->d);
   quin->peakFreqAdjusted = (sampleRate * quin->adjustedBinLocation / N );
   
}
// peakFreqAdjusted holds the peak frequency you want
// ...

/* FFT subroutine for WaoN with FFTW library
 * Copyright (C) 1998-2013 Kengo Ichiki <kengoichiki@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * ported by ACP Aviation as helper functions for hanning window
 * FFTW function still needs porting 
 * https://github.com/FFTW/fftw3
 */
  /* Reference: "Numerical Recipes in C" 2nd Ed.
 * by W.H.Press, S.A.Teukolsky, W.T.Vetterling, B.P.Flannery
 * (1992) Cambridge University Press.
 * ISBN 0-521-43108-5
 * Sec.13.4 - Data Windowing
 */

/*-----------------------------------------------------------------------------
 *   parzen:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/ 
float64_t parzen(int16_t i, int16_t nn)
{
  return (1.0f - fabs ((((float64_t)i)-0.5f*((float64_t)(nn-1)))/(0.5f*((float64_t)(nn+1)))));
}

/*-----------------------------------------------------------------------------
 *   welch:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t welch(int16_t i, int16_t nn)
{
  return (1.0f-((((float64_t)i)-0.5f*((float64_t)(nn-1)))/(0.5f*((float64_t)(nn+1))))*((((float64_t)i)-0.5f*((float64_t)(nn-1)))/(0.5f*((float64_t)(nn+1)))));
}

/*-----------------------------------------------------------------------------
 *   hanning:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t hanning(int16_t i, int16_t nn)
{
  return ( 0.5f * (1.0f - cos(2.0f*PI*((float64_t)i)/((float64_t)(nn-1)))) );
}

/* Reference: "Digital Filters and Signal Processing" 2nd Ed.
 * by L. B. Jackson. (1989) Kluwer Academic Publishers.
 * ISBN 0-89838-276-9
 * Sec.7.3 - Windows in Spectrum Analysis
 */
/*-----------------------------------------------------------------------------
 *   hamming:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t hamming(int16_t i, int16_t nn)
{
  return ( 0.54f - 0.46f * cos (2.0f*PI*((float64_t)i)/((float64_t)(nn-1))) );
}

/*-----------------------------------------------------------------------------
 *   blackman:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t blackman(int16_t i, int16_t nn)
{
  return ( 0.42659f - 0.49656f * cos (2.0f*PI*((float64_t)i)/((float64_t)(nn-1))) + 0.076849f * cos(4.0f*PI*((float64_t)i)/((float64_t)(nn-1))) );
}

/*-----------------------------------------------------------------------------
 *   steeper:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t steeper(int16_t i, int16_t nn)
{
  return ( 0.375f - 0.5f * cos (2.0f*PI*((float64_t)i)/((float64_t)(nn-1))) + 0.125f * cos (4.0f*PI*((float64_t)i)/((float64_t)(nn-1))) );
}

/*-----------------------------------------------------------------------------
 *   nutall:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t nutall(int16_t i, int16_t nn)
{
  return ( 0.355768f - 0.487396f * cos (2.0f*PI*((float64_t)i)/((float64_t)(nn-1))) + 0.144232f * cos(4.0f*PI*((float64_t)i)/((float64_t)(nn-1))) +  0.012604f * cos(6.0f*PI*((float64_t)i)/((float64_t)(nn-1))) );
}

/*-----------------------------------------------------------------------------
 *   blackman_nutall:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t blackman_nutall(int16_t i, int16_t nn)
{
  return ( 0.3635819f - 0.4891775f * cos (2.0f*PI*((float64_t)i)/((float64_t)(nn-1))) + 0.1365995f * cos(4.0f*PI*((float64_t)i)/((float64_t)(nn-1))) +  0.0106411f * cos(6.0f*PI*((float64_t)i)/((float64_t)(nn-1))) );
}

/*-----------------------------------------------------------------------------
 *   blackman_harris:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t blackman_harris(int16_t i, int16_t nn)
{
  return ( 0.35875f - 0.48829f * cos (2.0f*PI*((float64_t)i)/((float64_t)(nn-1))) + 0.14128f * cos(4.0f*PI*((float64_t)i)/((float64_t)(nn-1))) +  0.01168f * cos(6.0f*PI*((float64_t)i)/((float64_t)(nn-1))) );
}

/*-----------------------------------------------------------------------------
 *   flattop:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t flattop(int16_t i, int16_t nn)
{
  return ( 0.21557895f - 0.41663158f * cos (2.0f*PI*((float64_t)i)/((float64_t)(nn-1))) + 0.277263158f * cos(4.0f*PI*((float64_t)i)/((float64_t)(nn-1))) +  0.083578947f * cos(6.0f*PI*((float64_t)i)/((float64_t)(nn-1))) +  0.006947368f * cos(8.0f*PI*((float64_t)i)/((float64_t)(nn-1))) );
}

/*-----------------------------------------------------------------------------
 *   gaussian:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t gaussian(int16_t i, int16_t nn)
{
  const float64_t sigma = 0.4f;        
  return exp(0.5f*pow(((((float64_t)i) - sigma*((float64_t)(nn-1))/2.0f) / (((float64_t)(nn-1))/2.0f)),2.0f));
}

/*-----------------------------------------------------------------------------
 *   tukey:  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t tukey(int16_t i, int16_t nn)
{
  const float64_t n = ((float64_t)i);
  const float64_t N = ((float64_t)(nn-1));
  const float64_t sigma = 0.4f;        
  float64_t value = 0.0f;
  
  if ((n <= (sigma*N)/2.0f) && (n >= 0.0f))
  {
     value = ((1.0f - cos((2.0f*(PI*n))/(sigma*N)))*0.5f);
  } 
  else if ((n <= (N/2.0f)) && (n >= ((sigma*N)/2.0f)))
     value = 1.0f;
  else if (sigma == 1.0f)
  {
     value = hanning(i,nn);
  }
  return value;
}

/*-----------------------------------------------------------------------------
 *   planck_taper :  cost function
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t planck_taper(int16_t i, int16_t nn)
{
  const float64_t n = ((float64_t)i);
  const float64_t N = ((float64_t)(nn-1));
  const float64_t taper_ammnt = 0.4f;        
  float64_t value;
  
  if ((n < (taper_ammnt*N)/2.0f) && (n >= 1.0f))
  {
     value = 1.0f + pow(exp(((taper_ammnt*N)/n) - ((taper_ammnt*N)/((taper_ammnt*N)-n))),1.0f);       ;
  } 
  else if ((n <= (N/2.0f)) && (n >= (taper_ammnt*N)))
     value = 1.0f;
  else 
  {
     value = 0.0f;
  }
  return value;
}

/* apply window function to data[]
 * INPUT
 *  flag_window : 0 : no-window (default -- that is, other than 1 ~ 6)
 *                1 : parzen window
 *                2 : welch window
 *                3 : hanning window
 *                4 : hamming window
 *                5 : blackman window
 *                6 : steeper 30-dB/octave rolloff window
 *
 *  return -1 for error 
 */
/*-----------------------------------------------------------------------------
 *   windowing:  do windowing before FFT is called
 *
 *  Parameters: int16_t i, int16_t nn
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
int8_t windowing(int16_t n, const float64_t *dataV, int16_t flag_window, float64_t scale, float64_t *out)
{
  int16_t i;
  int8_t ret = 1;
  
  for (i = 0; i < n; i ++)
  {
    switch (flag_window)
    {
          case 1:                                                               // parzen window
          out[i] = dataV[i] * parzen(i, n) / scale;
          break;

          case 2:                                                               // welch window
          out[i] = dataV[i] * welch(i, n) / scale;
          break;

          case 3:                                                               // hanning window
          out[i] = dataV[i] * hanning(i, n) / scale;
          break;

          case 4:                                                               // hamming window
          out[i] = dataV[i] * hamming(i, n) / scale;
          break;

          case 5:                                                               // blackman window
          out[i] = dataV[i] * blackman(i, n) / scale;
          break;

          case 6:                                                               // steeper 30-dB/octave rolloff window
          out[i] = dataV[i] * steeper(i, n) / scale;
          break;

          case 7:
          out[i] = dataV[i] * nutall(i, n) / scale;
          break;
          
          case 8:
          out[i] = dataV[i] * blackman_nutall(i, n) / scale;
          break;
          
          case 9:
          out[i] = dataV[i] * blackman_harris(i, n) / scale;
          break;

          case 10:
          out[i] = dataV[i] * flattop(i, n) / scale;
          break;

          case 11:
          out[i] = dataV[i] * gaussian(i, n) / scale;
          break;

          case 12:
          out[i] = dataV[i] * tukey(i, n) / scale;
          break;

          case 13:
          out[i] = dataV[i] * planck_taper(i, n) / scale;
          break;
                                                  
          case 0:                                                               // square (no window)
          out[i] = dataV[i] / scale;
          break;
          
          default:
          ret = -1;
     }
  }
  return ret;
}

/* prepare window for FFT
 * INPUT
 *  n : # of samples for FFT
 *  flag_window : 0 : no-window (default -- that is, other than 1 ~ 6)
 *                1 : parzen window
 *                2 : welch window
 *                3 : hanning window
 *                4 : hamming window
 *                5 : blackman window
 *                6 : steeper 30-dB/octave rolloff window
 * OUTPUT
 *  density factor as RETURN VALUE
 */
/*-----------------------------------------------------------------------------
 *   init_den:  prepare window for FFT
 *
 *  Parameters: int16_t n, char flag_window, float64_t *den
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t init_den(int16_t n, char flag_window, float64_t *den)
{
  int16_t i;
  int8_t ret = 1;
  
  *den = 0.0f;
  for (i = 0; i < n; i ++)
  {
      switch (flag_window)
          {
            case 1: // parzen window
            *den += parzen(i, n) * parzen(i, n);
            break;

            case 2: // welch window
            *den += welch(i, n) * welch(i, n);
            break;

            case 3: // hanning window
            *den += hanning(i, n) * hanning(i, n);
            break;

            case 4: // hamming window
            *den += hamming(i, n) * hamming(i, n);
            break;

            case 5: // blackman window
            *den += blackman(i, n) * blackman(i, n);
            break;

            case 6: // steeper 30-dB/octave rolloff window
            *den += steeper(i, n) * steeper(i, n);
            break;

            case 7: // nutall window
            *den += nutall(i, n) * nutall(i, n);
            break;
            
            case 8: // blackman_nutall window
            *den += blackman_nutall(i, n) * blackman_nutall(i, n);
            break;
            
            case 9: // blackman_harris window
            *den += blackman_harris(i, n) * blackman_harris(i, n);
            break;

            case 10: // flattop window
            *den += flattop(i, n) * flattop(i, n);
            break;

            case 11: // gaussian window
            *den += gaussian(i, n) * gaussian(i, n);
            break;

            case 12: // tukey window
            *den += tukey(i, n) * tukey(i, n);
            break;

            case 13: // planck taper window
            *den += planck_taper(i, n) * planck_taper(i, n);
            break;
                        
            case 0: // square (no window)
            *den += 1.0f;
            break;
           
            default:
            ret = -1;

          }
  }

  *den *= ((float64_t)n);
  return ret;
}

/* half-complex format routines
 * Copyright (C) 2007-2013 Kengo Ichiki <kengoichiki@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

/* return angle (arg) of the complex number (freq(k),freq(len-k));
 * where (real,imag) = (cos(angle), sin(angle)).
 * INPUT
 *  len        :
 *  freq [len] :
 *  conj       : set 0 for normal case.
 *               set 1 for conjugate of the complex (freq(k),freq(len-k))
 *               that is, for (freq(k),-freq(len-k)).
 * OUTPUT
 *  amp [len/2+1] :
 *  phs [len/2+1] :
 */
#define CHECK_MALLOC(PTR, FUNC) /*check the malloc was done*/ \
if (PTR == NULL) { \
  return -1; }
  
/*-----------------------------------------------------------------------------
 *  HC_to_polar:  half-complex format routines
 *
 *  Parameters: int16_t n, char flag_window, float64_t *den
 *  Return: void
 *----------------------------------------------------------------------------*/  
void HC_to_polar(int64_t len, const float64_t *freq, int16_t conj, float64_t *amp, float64_t *phs)
{
  int16_t i;
  float64_t rl, im;

  phs[0u] = 0.0f;
  amp[0u] = sqrt(freq[0u] * freq[0u]);
  for (i = 1; i < (len+1)/2; i ++)
  {
      rl = freq[i];
      im = freq[len - i];
      amp[i] = sqrt (rl * rl + im * im);
      if (amp[i] > 0.0f) 
        {
          if (conj == 0) phs[i] = atan2(+im, rl);
          else           phs[i] = atan2(-im, rl);
        }
      else
        {
          phs [i] = 0.0f;
        }
  }
  if (len%2 == 0)
  {
      phs[len/2] = 0.0f;
      amp[len/2] = sqrt(freq[len/2] * freq[len/2]);
  }
}

/* return angle (arg) of the complex number (freq(k),freq(len-k));
 * where (real,imag) = (cos(angle), sin(angle)).
 * INPUT
 *  len        :
 *  freq [len] :
 *  conj       : set 0 for normal case.
 *               set 1 for conjugate of the complex (freq(k),freq(len-k))
 *               that is, for (freq(k),-freq(len-k)).
 *  scale      : scale factor for amp2[]
 * OUTPUT
 *  amp2 [len/2+1] := (real^2 + imag^2) / scale
 *  phs  [len/2+1] := atan2 (+imag / real) for conj==0
 *                  = atan2 (-imag / real) for conj==1
 */
/*-----------------------------------------------------------------------------
 *  HC_to_polar2:  half-complex format routines
 *
 *  Parameters: int64_t len, const float64_t *freq, int16_t conj, float64_t scale,  
 *              float64_t *amp2, float64_t *phs
 *
 *  Return: void
 *----------------------------------------------------------------------------*/  
void HC_to_polar2(int64_t len, const float64_t *freq, int16_t conj, float64_t scale,  float64_t *amp2, float64_t *phs)
{
  int16_t i;
  float64_t rl, im;

  phs[0u] = 0.0f;
  amp2[0u] = freq[0u] * freq[0u] / scale;
  for (i = 1; i < (len+1)/2; i ++)
    {
      rl = freq[i];
      im = freq[len - i];
      amp2[i] = (rl * rl + im * im)  / scale;
      if (amp2[i] > 0.0f) 
        {
          if (conj == 0) phs[i] = atan2(+im, rl);
          else           phs[i] = atan2(-im, rl);
        }
      else
        {
          phs [i] = 0.0f;
        }
    }
  if (len%2 == 0)
    {
      phs[len/2] = 0.0f;
      amp2[len/2] = freq[len/2] * freq[len/2] / scale;
    }
}

/* return power (amp2) of the complex number (freq(k),freq(len-k));
 * where (real,imag) = (cos(angle), sin(angle)).
 * INPUT
 *  len        :
 *  freq [len] :
 *  scale      : scale factor for amp2[]
 * OUTPUT
 *  amp2 [len/2+1] := (real^2 + imag^2) / scale
 */
/*-----------------------------------------------------------------------------
 *  HC_to_amp2:  half-complex format routines
 *
 *  Parameters: int64_t len, const float64_t *freq, float64_t scale, float64_t *amp2  
 *
 *  Return: void
 *----------------------------------------------------------------------------*/ 
void HC_to_amp2(int64_t len, const float64_t *freq, float64_t scale, float64_t *amp2)
{
  int16_t i;
  float64_t rl, im;

  amp2[0u] = freq[0u] * freq[0u] / scale;
  for (i = 1; i < (len+1)/2; i ++)
    {
      rl = freq [i];
      im = freq [len - i];
      amp2[i] = (rl * rl + im * im)  / scale;
    }
  if (len%2 == 0)
    {
      amp2[len/2] = freq[len/2] * freq[len/2] / scale;
    }
}

/* 
 * INPUT
 *  len           : N
 *  amp [len/2+1] :
 *  phs [len/2+1] :
 * OUTPUT
 *  freq [len] :
 * NOTE
 *  if len == even,
 *  (freq(0)     ... freq(N/2)) = (r(0)          ...      r(N/2)) and
 *  (freq(N/2+1) ... freq(N)  ) = (     i(N/2-1) ... i(1)       )
 *     where Fourier coefficient Y(k) = r(k) + i i(k).
 *     note that Y(k) = Y*(N-k) and Y(0) and Y(N/2) are real.
 *  if len == odd,
 *  (freq(0)     ... freq(N/2)) = (r(0)     ... r(N/2)) and
 *  (freq(N/2+1) ... freq(N)  ) = (i(N/2-1) ... i(1)  )
 *     note in this case that Y(0) is real but Y(N/2) is not.
 *  in either case, number of elements for the coefficients are N/2+1.
 */
/*-----------------------------------------------------------------------------
 *  polar_to_HC:  half-complex format routines
 *
 *  Parameters: int64_t len, const float64_t *amp, const float64_t *phs, int16_t conj, 
 *              float64_t *freq  
 *
 *  Return: void
 *----------------------------------------------------------------------------*/ 
void polar_to_HC(int64_t len, const float64_t *amp, const float64_t *phs, int16_t conj, float64_t *freq)
{
  int16_t i;
  float64_t rl, im;

  /* calc phase and amplitude (power) */
  freq[0u] = amp[0u];
  for (i = 1; i < (len+1)/2; i ++)
    {
      rl = + amp[i] * cos (phs[i]);
      if (conj == 0) im = + amp[i] * sin(phs[i]);
      else           im = - amp[i] * sin(phs[i]);
      freq[i] = rl;
      freq[len - i] = im;
    }
  if (len%2 == 0)
    {
      freq[len/2] = amp[len/2];
    }
}

/* convert polar to HC with the scaling in freq domain
 * INPUT
 *  len           : N
 *  amp [len/2+1] :
 *  phs [len/2+1] :
 *  scale         : integer scale factor
 *                  bin k in the input is placed in scale*k in the output
 *                  except for k=0 and k=len/2 for even len.
 * OUTPUT
 *  freq [len*scale] :
 */
/*-----------------------------------------------------------------------------
 *  polar_to_HC_scale:  convert polar to HC with the scaling in freq domain
 *
 *  Parameters: int64_t len, const float64_t *amp, const float64_t *phs, int16_t conj,  
 *              int16_t scale, float64_t *freq 
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void polar_to_HC_scale(int64_t len, const float64_t *amp, const float64_t *phs, int16_t conj, int16_t scale, float64_t *freq)
{
  int16_t i;
  float64_t rl, im;

  // zero clear
  for (i = 0; i < len*scale; i ++)
  {
      freq[i] = 0.0f;
  }

  /* calc phase and amplitude (power) */
  freq[0u] = amp[0u];
  for (i = 1; i < (len+1)/2; i ++)
  {
      rl = + amp[i] * cos(phs[i]);
      if (conj == 0) im = + amp[i] * sin(phs[i]);
      else           im = - amp[i] * sin(phs[i]);

      freq[i*scale] = rl;
      freq[len*scale - i*scale] = im;
  }
  if (len%2 == 0)
  {
      freq[len] = amp[len/2];
  }
}

/* Z = X * Y, that is,
 * (rz + i iz) = (rx1 + i ix) * (ry + i iy)
 *             = (rx1 * ry - ix * iy) + i (rx1 * iy + ix * ry)
 */
/*-----------------------------------------------------------------------------
 *  HC_mul:  
 *
 *  Parameters: int64_t len, const float64_t *x, const float64_t *y, float64_t *z  
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void HC_mul(int64_t len, const float64_t *x, const float64_t *y, float64_t *z)
{
  int16_t i;
  float64_t rx1, ix;
  float64_t ry, iy;

  /* calc phase and amplitude (power) */
  z[0u] = x[0u] * y[0u];
  for (i = 1; i < (len+1)/2; i ++)
  {
      rx1 = x[i];
      ix = x[len - i];
      ry = y[i];
      iy = y[len - i];
      z[i]       = rx1 * ry - ix * iy;
      z[len - i] = rx1 * iy + ix * ry;
  }
  if (len%2 == 0)
  {
      z[len/2] = x[len/2] * y[len/2];
  }
}

/* Z = X / Y, that is,
 * (rz + i iz) = (rx1 + i ix) / (ry + i iy)
 *             = (rx1 + i ix) * (ry - i iy) / (ry*ry + iy*iy)
 *             = (rx1*ry + ix*iy + i (ix*ry - rx1*iy)) / (ry*ry + iy*iy)
 */
/*-----------------------------------------------------------------------------
 *  HC_div:  
 *
 *  Parameters: int64_t len, const float64_t *x, const float64_t *y, float64_t *z  
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void HC_div(int64_t len, const float64_t *x, const float64_t *y, float64_t *z)
{
  int16_t i;
  float64_t rx1, ix;
  float64_t ry, iy;
  float64_t den;

  z[0u] = x[0u] / y[0u];                            /* calc phase and amplitude (power) */
  for (i = 1; i < (len+1)/2; i ++)
  {
      rx1 = x[i];
      ix = x[len - i];
      ry = y[i];
      iy = y[len - i];
      den = ry * ry + iy * iy;
      z[i]       = (rx1 * ry + ix * iy) / den;
      z[len - i] = (ix * ry - rx1 * iy) / den;
  }
  if (len%2 == 0)
  {
      z[len/2] = x[len/2] / y[len/2];
  }
}

/*-----------------------------------------------------------------------------
 *  HC_abs:  
 *
 *  Parameters: int64_t len, const float64_t *x, float64_t *z  
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void HC_abs(int64_t len, const float64_t *x, float64_t *z)
{
  int16_t i;
  float64_t rx1, ix;

  z[0u] = fabs(x[0u]);                         /* calc phase and amplitude (power) */
  for (i = 1; i < (len+1)/2; i ++)
  {
      rx1 = x[i];
      ix = x[len - i];
      z[i]       = sqrt(rx1 * rx1 + ix * ix);
      z[len - i] = 0.0f;
  }
  if (len%2 == 0)
  {
      z[len/2] = fabs(x[len/2]);
  }
}

/* NOTE: y cannot be z!
 */
/*-----------------------------------------------------------------------------
 *  HC_puckette_lock:  
 *
 *  Parameters: int64_t len, const float64_t *y, float64_t *z  
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void HC_puckette_lock(int64_t len, const float64_t *y, float64_t *z)
{
  int16_t k;

  z[0u] = y[0u];
  for (k = 1; k < (len+1)/2; k ++)
  {
    z[k]       = y[k];
    z[len - k] = y[len - k];
    if (k > 1)
        {
           z[k]       += y[k-1];
           z[len - k] += y[len - (k-1)];
        }
    if (k < (len+1)/2 - 1)
        {
           z[k]       += y[k+1];
           z[len - k] += y[len - (k+1)];
        }
  }
  if (len%2 == 0)
  {
     z[len/2] = y[len/2];
  }
}

/* Y[u_i] = X[t_i] (Y[u_{i-1}]/X[s_i]) / |Y[u_{i-1}]/X[s_i]|
 * Reference: M.Puckette (1995)
 * INPUT
 *  f_out_old[] : Y[u_{i-1}], synthesis-FFT at (i-1) step
 *  fs[]        : X[s_i], analysis-FFT at starting time of i step
 *  ft[]        : X[t_i], analysis-FFT at terminal time of i step
 *                Note: t_i - s_i = u_i - u_{i-1} = hop_out
 * OUTPUT
 *  f_out[]     : Y[u_i], synthesis-FFT at i step
 *                you can use the same point16_t f_out_old[] for this.
 */
/*-----------------------------------------------------------------------------
 *  HC_complex_phase_vocoder:  
 *
 *  Parameters: int16_t len, const float64_t *fs, const float64_t *ft, 
 *              const float64_t *f_out_old, float64_t *f_out 
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
int8_t HC_complex_phase_vocoder(int16_t len, const float64_t *fs, const float64_t *ft, const float64_t *f_out_old, float64_t *f_out)
{
  static float64_t *tmp1 = NULL;
  static float64_t *tmp2 = NULL;
  int8_t ret = 1;
  
  tmp1 = (float64_t *)Malloc(sizeof (float64_t) * len);
  tmp2 = (float64_t *)Malloc(sizeof (float64_t) * len);
  CHECK_MALLOC(tmp1, "HC_complex_phase_vocoder");
  CHECK_MALLOC(tmp2, "HC_complex_phase_vocoder");

  HC_div(len, f_out_old, fs, tmp1);     // tmp1 = Y[u_{i-1}]/X[s(i)]

  HC_abs(len, tmp1, tmp2);             // tmp2 = |Y[u_{i-1}]/X[s(i)]|

  HC_div(len, tmp1, tmp2, tmp1);      // tmp1 = (Y[u_{i-1}]/X[s_i]) / |Y[u_{i-1}]/X[s_i]|

  HC_mul(len, ft, tmp1, f_out);       // f_out = X[t_i] (Y[u_{i-1}]/X[s_i]) / |Y[u_{i-1}]/X[s_i]|
  return ret;
}

/* subtract average from the power spectrum
 * -- intend to remove non-tonal signal (such as drums, percussions)
 * INPUT
 *  n : FFT size
 *  p[(n+1)/2] : power spectrum
 *  m : number of bins to average out
 *  factor : factor * average is subtracted from the power
 *           (factor = 0.0) means no subtraction
 *           (factor = 1.0) means full subtraction of the average
 *           (factor = 2.0) means over subtraction
 * OUTPUT
 *  p[(n+1)/2] : subtracted power spectrum
 */
/*-----------------------------------------------------------------------------
 *  power_subtract_ave:  subtract average from the power spectrum
 *
 *  Parameters: int16_t n, float64_t *p, int16_t m, float64_t factor 
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t power_subtract_ave(int16_t n, float64_t *p, int16_t m, float64_t factor)
{
  int16_t nlen = n/2+1;
  int16_t i;
  int16_t k;
  int16_t nave;
  int8_t ret = 1;
  
  static float64_t *ave = NULL;

  ave = (float64_t *)Malloc((sizeof(float64_t)*nlen));
  CHECK_MALLOC(ave, "power_subtract_ave");

  for (i = 0; i < nlen; i ++) // full span
  {
      ave[i] = 0.0f;
      nave = 0;
      for (k = -m; k <= m; k ++)
      {
         if ((i + k) < 0 || (i + k) >= nlen) continue;
         ave[i] += p[i+k];
         nave= ++nave % INT16_MAX;
      }
      if (nave > 0) ave[i] /= (float64_t)nave;
  }

  for (i = 0; i < nlen; i ++) // full span
  {
      p[i] = sqrt (p[i]) - factor * sqrt(ave[i]);
      if (p[i] < 0.0f) p[i] = 0.0f;
      else             p[i] = p[i] * p[i];
  }

  Free((char*)ave,(sizeof(float64_t)*nlen));
  return ret;
}

/* octave remover
 * INPUT
 *  n : FFT size
 *  p[(n+1)/2] : power spectrum
 *  factor : factor * average is subtracted from the power
 *           (factor = 0.0) means no subtraction
 *           (factor = 1.0) means full subtraction of the average
 *           (factor = 2.0) means over subtraction
 * OUTPUT
 *  p[(n+1)/2] : subtracted power spectrum
 */
/*-----------------------------------------------------------------------------
 *  power_subtract_octave:  octave remover
 *
 *  Parameters: int16_t n, float64_t *p, float64_t factor 
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
int8_t power_subtract_octave(int16_t n, float64_t *p, float64_t factor)
{
  int16_t nlen = (n+1)/2;
  int16_t i;
  int16_t i2;
  int8_t ret = 1;
  
  static float64_t *oct = NULL;
  oct = (float64_t *)Malloc((sizeof(float64_t)*(n/2+1)));
  CHECK_MALLOC(oct, "power_subtract_octave");

  oct[0u] = p[0];
  for (i = 1; i < nlen/2+1; i ++)
  {
      i2 = i * 2;
      if (i2 >= n/2+1) break;

      oct[i2] = factor * p[i];
      if (i2-1 > 0)    oct[i2-1u] = 0.5f * factor * p[i];
      if (i2+1 < nlen) oct[i2+1u] = 0.5f * factor * p[i];
  }

  for (i = 0; i < nlen; i ++) // full span
  {
      p[i] = sqrt (p[i]) - factor * sqrt (oct[i]);
      if (p [i] < 0.0) p[i] = 0.0f;
      else             p[i] = p[i] * p[i];
  }

  Free((char*)oct,(sizeof(float64_t)*(n/2+1)));
  return ret;
}

/* apply FFT with the window and return amplitude and phase
 * this is a wrapper mainly for phase vocoder process
 * INPUT
 *  len : FFT length
 *  data[len] : data to analyze
 *  flag_window : window type
 *  plan, in[len], out[len] : for FFTW3
 *  scale : amplitude scale factor
 * OUTPUT
 *  amp[len/2+1] : amplitude multiplied by the factor "scale" above
 *  phs[len/2+1] : phase
 */
/* void apply_FFT(int16_t len, const float64_t *data, int16_t flag_window, fftw_plan plan, float64_t *in, float64_t *out, float64_t scale, float64_t *amp, float64_t *phs) */
/*-----------------------------------------------------------------------------
 *  apply_FFT:  apply FFT with the window and return amplitude and phase
 *
 *  Parameters: int16_t len, const float64_t *dataV, int16_t flag_window, 
 *  float64_t *in, float64_t *out, float64_t scale, float64_t *amp, float64_t *phs 
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void apply_FFT(int16_t len, const float64_t *dataV, int16_t flag_window, float64_t *in, float64_t *out, float64_t scale, float64_t *amp, float64_t *phs)
{
  int16_t i;

  windowing(len, dataV, flag_window, 1.0f, in);
  // TODO:: fftw_execute(plan);  FFT: in[] -> out[]
  HC_to_polar(len, out, 0, amp, phs); // freq[] -> (amp, phs)

  for (i = 0; i < (len/2)+1; i ++)    // some scaling
  {
      amp[i] /= scale;
  }
}
#endif
// Box and Betts exponential quadratic sum, equivalent to least-squares of
// f(x) = exp(-0.1*a0*x) - exp(-0.1*a1*x) - a2 * (exp(-0.1*x) - exp(-x))
// with points (1,0), (2,0), ... (10,0)
// 
// domains of a0, a1, a2 are, respectively, (0.9, 1.2), (9, 11.2), (0.9, 1.2)
// minimum: (1,10,1) -> 0

// modified boxbetts_f() from nlopt-2.3/test/testfuncs.c
// source https://github.com/eugeneai/fityk/blob/master/tests/gradient.cpp
// original author : Eugvny Cherkashin port by Air Cam Pro
//
/*-----------------------------------------------------------------------------
 *  boxbetts_f:  Box and Betts exponential quadratic sum
 *
 *  Parameters: const float64_t *a, float64_t *grad  
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t boxbetts_f(const float64_t *a, float64_t *grad)
{
    float64_t f = 0.0f;
    float64_t e0;
    float64_t e1;
    float64_t e2;
    float64_t g;
    int8_t x;

    if ((a != NULL) && (grad != NULL)) 
    {
        if (sizeof(grad) >= (sizeof(float64_t) * 3u))        
        {                        
            if (grad)
                grad[0u] = grad[1u] = grad[2u] = 0.0f;
            for (x = 1; x <= 10; ++x) 
            {
                e0 = exp(-0.1f*x*a[0u]);
                e1 = exp(-0.1f*x*a[1u]);
                e2 = exp(-0.1f*x) - exp(-1.0f*x);
                g = e0 - e1 - e2 * a[2u];
                f += g*g;
                if (grad) 
                       {
                    grad[0u] += (2.0f * g) * (-0.1f*x*e0);
                    grad[1u] += (2.0f * g) * (0.1f*x*e1);
                    grad[2u] += -(2.0f * g) * e2;
                }
            }
        }
    }
    return f;
}

//  fastest_humlik.for and humdev.for - from Bob Wells Voigt Function Page
//  http://web.archive.org/web/20100503005358/http://www.atm.ox.ac.uk/user/wells/voigt.html
//  ref : marcin wojdhr
///     Calculates the Faddeeva function
///     and partial derivatives of the Voigt function for y>=0
/// arguments:
///  x, y - Faddeeva/Voigt function arguments
///  k - voigt              -- output
///  l - Imaginary part     -- output
///  *dkdx - dVoigt/dx       -- output
///  *dkdy - dVoigt/dy       -- output
/*-----------------------------------------------------------------------------
 *  humdev:  Calculates the Faddeeva function and partial derivatives of the Voigt function for y>=0
 *
 *  Parameters: const float64_t x, const float64_t y, float64_t *k, float64_t *l,  
 *  float64_t *dkdx, float64_t *dkdy 
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void humdev(const float64_t x, const float64_t y, float64_t *k, float64_t *l, float64_t *dkdx, float64_t *dkdy)
{
    static const float64_t c[6u] = { 1.0117281f, -0.75197147f, 0.012557727f, 0.010022008f, -2.4206814e-4f, 5.0084806e-7f };
    static const float64_t s[6u] = { 1.393237f, 0.23115241f, -0.15535147f, 0.0062183662f, 9.1908299e-5f, -6.2752596e-7f };
    static const float64_t t[6u] = { 0.31424038f, 0.94778839f, 1.5976826f, 2.2795071f, 3.020637f, 3.8897249f };

    static const float64_t rrtpi = 0.56418958f; // 1/SQRT(pi)
    static const float64_t drtpi = 0.5641895835477563f; // 1/SQRT(pi)

    static float64_t a0, b1, c0, c2, d0, d1, d2, e0, e2, e4, f1, f3, f5, g0, g2, g4, g6, h0, h2, h4, h6, p0, p2, p4, p6, p8,  q1, q3, q5, q7, r0, r2, w0, w2, w4, z0, z2, z4, z6, z8, mf[6u], pf[6u], mq[6u], mt[6u], pq[6u], pt[6u], xm[6u], ym[6u], xp[6u], yp[6u];

    static float64_t old_y = -1.0f;

    static bool rgb, rgc, rgd;
    static float64_t yq, xlima, xlimb, xlimc, xlim4;

    float64_t abx = fabs(x);
    float64_t xq = abx * abx;
    float64_t d;
    float64_t u;
    float64_t dudy,dvdy;
    float64_t ypy0;
    float64_t ypy0q;
    float64_t yf1;
    float64_t yf2;
    int8_t j;        
 /*   float64_t mfq;
    float64_t pfq;       */
        
    if (y != old_y) 
    {
        old_y = y;
        rgb = true, rgc = true, rgd = true;
        yq = y * y;
        xlima = 146.7f - y;
        xlimb = 24.f - y;
        xlimc = 7.4f - y;
        xlim4 = y * 18.1f + 1.65f;
    }

    if (abx > xlima) 
    {  //  Region A
        d = (float64_t)1.0f / (xq + yq);
        d1 = d * rrtpi;
        *k = d1 * y;
        *l = d1 * x;
        d1 *= d;
        *dkdx = -d1 * (y + y) * x;
        *dkdy = d1 * (xq - yq);
    }
    else if (abx > xlimb) 
    { //  Region B
        if (rgb) 
        {
            rgb = false;
            a0 = yq + .5f;
            b1 = yq - .5f;
            d0 = a0 * a0;
            d2 = b1 + b1;
            c0 = yq * (1.f - d2) + 1.5f;
            c2 = a0 + a0;
            r0 = yq * (.25f - yq * (yq + .5f)) + .125f;
            r2 = yq * (yq + 5.f) + .25f;
        }
        d = 1.f / (d0 + xq * (d2 + xq));
        d1 = d * rrtpi;
        *k = d1 * (a0 + xq) * y;
        *l = d1 * (b1 + xq) * x;
        d1 *= d;
        *dkdx = d1 * x * y * (c0 - (c2 + xq) * (xq + xq));
        *dkdy = d1 * (r0 - xq * (r2 - xq * (b1 + xq)));
    } 
    else 
    {
        if (abx > xlimc) 
        {  // Region C
            if (rgc) 
            {
                rgc = false;
                h0 = yq * (yq * (yq * (yq + (float64_t)6.f) + (float64_t)10.5f) + (float64_t)4.5f) + (float64_t).5625f;
                h2 = yq * (yq * (yq * (float64_t)4.f + (float64_t)6.f) + (float64_t)9.f) - (float64_t)4.5f;
                h4 = (float64_t)10.5f - yq * ((float64_t)6.f - yq * (float64_t)6.f);
                h6 = yq * (float64_t)4.f - (float64_t)6.f;
                w0 = yq * (yq * (yq * (float64_t)7.f + (float64_t)27.5f) + (float64_t) 24.25f) + (float64_t)1.875f;
                w2 = yq * (yq * (float64_t)15.f + (float64_t)3.f) + (float64_t)5.25f;
                w4 = yq * (float64_t)9.f - (float64_t)4.5f;
                f1 = yq * (yq * (yq + (float64_t)4.5f) + (float64_t)5.25f) - (float64_t) 1.875f;
                f3 = (float64_t)8.25f - yq * ((float64_t)1.f - yq * (float64_t)3.f);
                f5 = yq * (float64_t)3.f - (float64_t)5.5f;
                e0 = y * (yq * (yq * (yq + (float64_t)5.5f) + (float64_t)8.25f) + (float64_t)1.875f);
                e2 = y * (yq * (yq * (float64_t)3.f + (float64_t)1.f) + (float64_t) 5.25f);
                e4 = y * (float64_t).75f * h6;
                g0 = y * (yq * (yq * (yq * (float64_t)8.f + (float64_t)36.f) + (float64_t)42.f) + (float64_t)9.f);
                g2 = y * (yq * (yq * (float64_t)24.f + (float64_t)24.f) + (float64_t)18.f);
                g4 = y * (yq * (float64_t)24.f - (float64_t)12.f);
                g6 = y * (float64_t)8.f;
            }
            u = e0 + xq * (e2 + xq * (e4 + xq * y));
            d = (float64_t)1.f / (h0 + xq * (h2 + xq * (h4 + xq * (h6 + xq))));
            *k = d * rrtpi * u;
            *l = d * rrtpi * x * (f1 + xq * (f3 + xq * (f5 + xq)));
            dudy = w0 + xq * (w2 + xq * (w4 + xq));
            dvdy = g0 + xq * (g2 + xq * (g4 + xq * g6));
            *dkdy = d * rrtpi * (dudy - d * u * dvdy);
        }

        else if (abx < (float64_t).85f) {  // Region D
            if (rgd) 
            {
                rgd = false;
                z0 = y * (y * (y * (y * (y * (y * (y * (y * (y * (y + (float64_t)13.3988f) + (float64_t)88.26741f) + (float64_t)369.1989f) + (float64_t)1074.409f) + (float64_t)2256.981f) + (float64_t)3447.629f) + (float64_t)3764.966f) + (float64_t) 2802.87f) + (float64_t)1280.829f) + (float64_t)272.1014f;
                z2 = y * (y * (y * (y * (y * (y * (y * (y * (float64_t)5.f + (float64_t)53.59518f) + (float64_t)266.2987f)  + (float64_t)793.4273f) + (float64_t)1549.675f) + (float64_t)2037.31f) + (float64_t)1758.336f) + (float64_t)902.3066f) + (float64_t)211.678f;
                z4 = y * (y * (y * (y * (y * (y * (float64_t)10.f + (float64_t)80.39278f) + (float64_t)269.2916f) + (float64_t) 479.2576f) + (float64_t)497.3014f) + (float64_t)308.1852f) + (float64_t)78.86585f;
                z6 = y * (y * (y * (y * (float64_t)10.f + (float64_t)53.59518f) + (float64_t)92.75679f) + (float64_t)55.02933f) + (float64_t)22.03523f;
                z8 = y * (y * (float64_t)5. + (float64_t)13.3988f) + (float64_t)1.49646f;
                p0 = y * (y * (y * (y * (y * (y * (y * (y * (y * (float64_t).3183291f + (float64_t)4.264678f) + (float64_t)27.93941f) + (float64_t)115.3772f) + (float64_t)328.2151f) + (float64_t)662.8097f) + (float64_t)946.897f) + (float64_t)
                        919.4955f) + (float64_t)549.3954f) + (float64_t)153.5168f;
                p2 = y * (y * (y * (y * (y * (y * (y * (float64_t)1.2733163f + (float64_t)12.79458f) + (float64_t)56.81652f) + (float64_t)139.4665f) + (float64_t)189.773) + (float64_t)124.5975f) - (float64_t)1.322256f) - (float64_t)34.16955f;
                p4 = y * (y * (y * (y * (y * (float64_t)1.9099744f + (float64_t)12.79568f) + (float64_t)29.81482f) + (float64_t)24.01655f) + (float64_t)10.46332f) + (float64_t)2.584042f;
                p6 = y * (y * (y * (float64_t)1.273316f + (float64_t)4.266322f) + (float64_t).9377051f) - (float64_t).07272979f;
                p8 = y * (float64_t).3183291f + (float64_t)5.480304e-4f;
                q1 = y * (y * (y * (y * (y * (y * (y * (y * (float64_t).3183291f + (float64_t)4.26413f) + (float64_t)27.6294f) + (float64_t)111.0528f) + (float64_t)301.3208f) + (float64_t) 557.5178f) + (float64_t)685.8378f) + (float64_t)508.2585f) + (float64_t)173.2355f;
                q3 = y * (y * (y * (y * (y * (y * (float64_t)1.273316f + (float64_t)12.79239f) + (float64_t)55.8865f) + (float64_t)130.8905f) + (float64_t)160.4013f) + (float64_t)100.7375f) + (float64_t)18.97431f;
                q5 = y * (y * (y * (y * (float64_t)1.909974f + (float64_t)12.79239f) + (float64_t)28.8848f) + (float64_t)19.83766f) + (float64_t)7.985877f;
                q7 = y * (y * (float64_t)1.273316f + (float64_t)4.26413f) + (float64_t).6276985f;
            }
            u = (p0 + xq * (p2 + xq * (p4 + xq * (p6 + xq * p8))))* 1.7724538f;
            d = 1.f / (z0 + xq * (z2 + xq * (z4 + xq * (z6 + xq * (z8 + xq)))));
            *k = d * u;
            *l = d * 1.7724538f * x * (q1 + xq * (q3 + xq * (q5 + xq * (q7 + xq * .3183291f))));
            *dkdy = 2.0f * ( x * *l + y * *k - drtpi);
        }
        else 
        {     // Use CPF12
            ypy0 = y + 1.5f;
            ypy0q = ypy0 * ypy0;
            *k = 0.f;
            *l = 0.f;
            for (j = 0; j <= 5; ++j) 
            {
                mt[j] = x - t[j];
                mq[j] = mt[j] * mt[j];
                mf[j] = (float64_t)1.f / (mq[j] + ypy0q);
                xm[j] = mf[j] * mt[j];
                ym[j] = mf[j] * ypy0;
                pt[j] = x + t[j];
                pq[j] = pt[j] * pt[j];
                pf[j] = (float64_t)1.f / (pq[j] + ypy0q);
                xp[j] = pf[j] * pt[j];
                yp[j] = pf[j] * ypy0;
                *l += c[j] * (xm[j] + xp[j]) + s[j] * (ym[j] - yp[j]);
            }
            if (abx <= xlim4) 
            {  // Humlicek CPF12 Region I
                yf1 = ypy0 + ypy0;
                yf2 = ypy0q + ypy0q;
                *dkdy = (float64_t)0.f;
                for (j = 0; j <= 5; ++j) 
                {
                    float64_t mfq = mf[j] * mf[j];
                    float64_t pfq = pf[j] * pf[j];
                    *k += c[j] * (ym[j] + yp[j]) - s[j] * (xm[j] - xp[j]);
                    *dkdy += c[j] * (mf[j] + pf[j] - yf2 * (mfq + pfq)) + s[j] * yf1 * (mt[j] * mfq - pt[j] * pfq);
                }
            } 
            else 
            {  //  Humlicek CPF12 Region II
                float64_t yp2y0 = y + (float64_t)3.;
                for (j = 0; j <= 5; ++j) 
                {
                    *k += (c[j] * (mq[j] * mf[j] - ym[j] * 1.5f) + s[j] * yp2y0 * xm[j]) / (mq[j] + 2.25f) + (c[j] * (pq[j] * pf[j] - yp[j] * 1.5f) - s[j] * yp2y0 * xp[j]) / (pq[j] + 2.25f);
                }
                *k = y * *k + exp(-xq);
                *dkdy = 2.0f * ((float64_t) x * (float64_t) *l + (float64_t) y * (float64_t) *k - drtpi);
            }
        }
        *dkdx = 2.0f * ((float64_t) y * (float64_t) *l - (float64_t) x * (float64_t) *k);
    }
}

/*-----------------------------------------------------------------------------
 *  shirley_bg:  Shirley Background correction for X-ray Photoelectron Spectroscopy.
 *
 *  Parameters: dVectr **pp, int16_t ppSize   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void shirley_bg( dVectr **pp, int16_t ppSize )
{
    const int16_t max_iter = 50f;
    const float64_t max_rdiff = 1e-6f;
    const int16_t n = ppSize;
    float64_t ya = pp[0u]->y;                                                   // lowest bg
    float64_t yb;                                                               // highest bg
    float64_t dy = yb - ya;
    float64_t rel_diff;
    int16_t i,iter;
    dVectr *B;                                                                  /* used in one plane only atm (X axis) */
    dVectr *PA;                                                                 /* used in one plane only atm (Y axis) */
    float64_t old_A = 0.0f;
    dVectr *Y;

    yb = pp[n-1u]->y;
    B = (dVectr*)Malloc((sizeof(dVectr))*ppSize);
    PA = (dVectr*)Malloc((sizeof(dVectr))*ppSize);        
    Y = (dVectr*)Malloc((sizeof(dVectr))*ppSize);        
        
    for (iter = 0; iter < ppSize ; ++iter)
    {
        B[iter]->x = ya;                                                        
        B[iter]->y = ya;                                                        /* only this plane atm */
        B[iter]->z = ya;
        PA[iter]->x = 0.0f;                                                     /* only thisd plane */
        PA[iter]->y = 0.0f;                                                     
        PA[iter]->z = 0.0f;
    }
    for (iter = 0; iter < max_iter; ++iter) 
    {
        for (i = 0; i < n; ++i)
            Y[i]->y = pp[i]->y - B[i]->y;
        for (i = 1; i < n; ++i)
            PA[i]->x = PA[i-1u]->x + (Y[i]->x + Y[i-1u]->x) / 2f * (pp[i]->x - pp[i-1u]->x);
        rel_diff = old_A != 0.f ? fabs(PA[n-1u]->x - old_A) / old_A : 1.f;
        if (rel_diff < max_rdiff)
            break;
        old_A = PA[n-1u]->x;
        for (i = 0; i < n; ++i)
            B[i]->y = ya + dy / PA[n-1u]->x * PA[i]->x;
    }
    for (i = 0; i < n; ++i)
        pp[i]->y = B[i]->y;
        
    Free((char*)B,sizeof(B));
    Free((char*)PA,sizeof(PA));
    Free((char*)Y,sizeof(Y));
}

/*-----------------------------------------------------------------------------
 *  Guess_find_hwhm:  find hwhm
 *
 *  Parameters: int16_t pos, float64_t* area, float64_t **yy_, float64_t **xx_   
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t Guess_find_hwhm(int16_t pos, float64_t* area, float64_t *yy_, float64_t *xx_, int16_t yySiz ) 
{
    const int16_t n = 3;
    int16_t left_pos = 0;
    int16_t right_pos;
    int16_t i;
    int16_t counter;
    float64_t hwhm;
    float64_t hm;

    hm = 0.5f * yy_[pos];
    right_pos = yySiz - 1;        

    counter = 0;                                                                // first we search the width of the left side of the peak
    for (i = pos; i > 0; --i) 
    {
        if (yy_[i] > hm) 
        {
            if (counter > 0)                                                    // previous point16_t had y < hm
                --counter;                                                      // compensate it, it was only fluctuation
        } 
        else 
        {
            ++counter;
            if (counter == n)                                                   // we found a point16_t below `hm', but we need to find `n' point16_ts  below `hm' to be sure that it's not a fluctuation
            {
                left_pos = i + counter;
                break;
            }
        }
    }

    counter = 0;
    for (i = pos; i < right_pos; i++)                                           // do the same for the right side
    {
        if (yy_[i] > hm) 
        {
            if (counter > 0)
                counter--;
        } 
        else 
        {
            counter++;
            if (counter == n) 
            {              
                right_pos = i - counter + 1;                                    // +1 here is int16_tentionally asymmetric with the left side
                break;
            }
        }
    }

    if (area) 
    {
        *area = 0.0f;
        for (i = left_pos; i < right_pos; ++i)
            *area += (xx_[i+1u] - xx_[i]) * (yy_[i] + yy_[i+1u]) / 2.0f;
    }

    hwhm = (xx_[right_pos] - xx_[left_pos]) / 2.0f;
    return hwhm;
    /*return ((hwhm > DBL_EPSILON) ? hwhm : DBL_EPSILON); */
}

/*-----------------------------------------------------------------------------
 *  Guess_estimate_peak_parameters:  outputs vector with: center, height, hwhm, area
 *                                   returns values corresponding to peak_traits
 *
 *  Parameters: float64_t *yy_, size_t yySiz, float64_t *sigma_, float64_t *xx_, 
 *   float64_t height_correction, float64_t width_correction 
 *
 *  Return: dVectr
 *----------------------------------------------------------------------------*/
dVectr Guess_estimate_peak_parameters( float64_t *yy_, size_t yySiz, float64_t *sigma_, float64_t *xx_, float64_t height_correction, float64_t width_correction ) 
{
    int16_t pos = -1;                                                           // find the highest point16_t, which must be higher than the previous point16_t and not lower than the next one (-> it cannot be the first/last point16_t)
    float64_t height;
    float64_t center;
    float64_t area;
    float64_t hwhm;
    dVectr resultVectr;        /* TO DO */
    int16_t i,t;
        
    if (sizeof(sigma_) < (yySiz - 1)) 
    {
        for (i = 1; i < (int16_t) yySiz - 1; ++i) 
        {
            t = (pos == -1 ? i-1 : pos);
            if (sigma_[t] * yy_[i] > sigma_[i] * yy_[t] && sigma_[i+1u] * yy_[i] >= sigma_[i] * yy_[i+1u])
                pos = i;
        }
    } 
    else 
    {
        for (i = 1; i < (int16_t) yySiz - 1; ++i) 
        {
            t = (pos == -1 ? i-1 : pos);
            if (yy_[i] > yy_[t] && yy_[i] >= yy_[i+1u])
                pos = i;
        }
    }
    if (pos == -1)
        return resultVectr;

    resultVectr.x = yy_[pos] * height_correction;
    resultVectr.y = xx_[pos];
    area = 0.0f;
    hwhm = Guess_find_hwhm(pos, &area, yy_, xx_, yySiz) * width_correction;                      
    return resultVectr;
}

/*-----------------------------------------------------------------------------
 *  Guess_estimate_linear_parameters: guess linear parameters
 *
 *  Parameters: float64_t **xx_, float64_t **yy_, size_t yySiz 
 *
 *  Return: dVectr
 *----------------------------------------------------------------------------*/
dVectr Guess_estimate_linear_parameters( float64_t *xx_, float64_t *yy_, size_t yySiz )
{
    float64_t sx = 0.0f, sy = 0.0f, sxx = 0.0f, /*syy = 0,*/ sxy = 0.0f;
    int16_t i;
    int16_t n = yySiz;
    float64_t x,y;
    dVectr vector3 = {0.0f, 0.0f, 0.0f};

    if ((xx_ != NULL) && (yy_ != NULL))
    {                
        for (i = 0; i != n; ++i) 
        {
            x = xx_[i];
            y = yy_[i];
            sx += x;
            sy += y;
            sxx += x*x;
            //syy += y*y;
            sxy += x*y;
        }
        vector3.x = (n * sxy - sx * sy) / (n * sxx - sx * sx);                  /* slope */
        vector3.y = (sy - vector3.x * sx) / n;                                  /* intercept */ 
        vector3.z = sy / n;                                                     /* avgy */  
    }
    return vector3;
}

/*-----------------------------------------------------------------------------
 *  Guess_estimate_sigmoid_parameters: ad-hoc arbitrary procedure to guess initial sigmoid parameters,
 *                                     with no theoretical justification
 *
 *  Parameters: float64_t **yy_, float64_t *xx_, float64_t upper, float64_t lower, size_t yySize 
 *
 *  Return: dqaut
 *----------------------------------------------------------------------------*/
dquat Guess_estimate_sigmoid_parameters( float64_t *xx_, float64_t *yy_, size_t yySize, float64_t upper, float64_t lower )
{
    size_t i; 
    float64_t sx = 0, sy = 0, sxx = 0, sxy = 0;
    int16_t n = 0;
    float64_t x,y,a,b;
    dquat vector4;     /* to do */

    if (yy_ != NULL)
    {                
        for (i = 0; i != yySize; ++i)                                           // fitting ax+b to linearized sigmoid
        {
            if (yy_[i] <= lower || yy_[i] >= upper)
                continue;
            x = xx_[i];

            y =  -log((upper - lower) / (yy_[i] - lower) - 1.f);                // normalizing: y ->  (y - lower) / (upper - lower) and y -> -log(1/y-1);
            sx += x;
            sy += y;
            sxx += x*x;
            sxy += x*y;
            ++n;
        }
        a = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        b = (sy - a * sx) / n;

        vector4.x = upper;
        vector4.y = lower;
        vector4.z = -b/a;
        vector4.w = 1.0f/a;
    }
    return vector4;
}

/*-----------------------------------------------------------------------------
 *  is_left_of_line: test if point p2 left of the line through p0 and p1
 *
 *  Parameters: const dVectr p0, const dVectr p1, const dVectr p2 
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool is_left_of_line( const dVectr p0, const dVectr p1, const dVectr p2 )
{ return (p1.x - p0.x)*(p2.y - p0.y) > (p2.x - p0.x)*(p1.y - p0.y); }

/*-----------------------------------------------------------------------------
 *  moser_de_bruijn_gen: Function to generate nth term of Moser-de Bruijn Sequence
 *
 *  Parameters: const int16_t n, int16_t *S
 *
 *  Return: int16_t
 *----------------------------------------------------------------------------*/
int16_t moser_de_bruijn_gen(const int16_t n, int16_t *S)
{
    int16_t i;

    S[0u] = 0;
    S[1u] = 1;
 
    for (i = 2; i <= n; i++)
    {   
        if (i % 2 == 0)         // S(2 * n) = 4 * S(n)
           S[i] = 4 * S[i / 2];
        else                   // S(2 * n + 1) = 4 * S(n) + 1
           S[i] = 4 * S[i / 2] + 1;
    }
     
    return S[n];
}
 
/*-----------------------------------------------------------------------------
 *  moser_de_bruijn_gen: Generating the first 'n' terms of Moser-de Bruijn Sequence sequence must be n+1
 *
 *  Parameters: const int16_t n, int16_t *sequence
 *
 *  Return: int16_t
 *----------------------------------------------------------------------------*/
int8_t doMoserDeBruijn(const int16_t n, int16_t *sequence)
{
    int16_t i;
    int8_t ret = -1;

    if (sequence != NULL)
    {                
        if (sizeof(sequence) >= (n+1))
        {
            for (i = 0; i < n; i++)
                sequence[i] = moser_de_bruijn_gen(i, sequence);
            ret = 1;
        }
    }
    return ret;
}
/*-----------------------------------------------------------------------------
 *  newman_conway_gen: Generating the first 'n' terms of Newman Conway Sequence sequence must be n+1
 *
 *  Parameters: const int16_t n, int16_t *sequence
 *
 *  Return: int16_t
 *----------------------------------------------------------------------------*/
int16_t newman_conway_gen(const int16_t n, int16_t *f)
{
    int16_t i;
    f[0u] = 0;
    f[1u] = 1;
    f[2u] = 1;
 
    for (i = 3; i <= n; i++)
        f[i] = f[f[i - 1]] + f[i - f[i - 1]];   
 
    return f[n];
}

/*-----------------------------------------------------------------------------
 *  doNewmanConway: Generating the first 'n' terms of Newman Conway Sequence sequence must be n+1
 *
 *  Parameters: const int16_t n, int16_t *sequence
 *
 *  Return: int16_t
 *----------------------------------------------------------------------------*/
int8_t doNewmanConway(const int16_t n, int16_t *sequence)
{
        int16_t i;
    int8_t ret = -1;

    if (sequence != NULL)
    {                
        if (sizeof(sequence) >= (n+1))
        {
            for (i = 0; i < n; i++)
                sequence[i] = newman_conway_gen(i, sequence);
            ret = 1;
        }
    }
    return ret;
}
/*-----------------------------------------------------------------------------
 *  collatz_gen: Generating the first 'n' terms of collatz Sequence sequence must be n+1
 *
 *  Parameters: const int16_t n, int16_t *sequence
 *
 *  Return: int16_t
 *----------------------------------------------------------------------------*/
int16_t collatz_gen(const int16_t n)
{
    int16_t resV;
    if (n != 1)
    {
        if (n & 1)               // If n is odd else even
            resV = 3*n + 1;
        else
            resV = n/2;
    }
    return resV;
}

/*-----------------------------------------------------------------------------
 *  doCollatz: Generating the first 'n' terms of Collatz Sequence sequence must be n+1
 *
 *  Parameters: const int16_t n, int16_t *sequence
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t doCollatz(const int16_t n, int16_t *sequence)
{
    int16_t i;
    int8_t ret = -1;

    if (sequence != NULL)
    {                
        if (sizeof(sequence) >= (n+1))
        {
            for (i = 0; i < n; i++)
                sequence[i] = collatz_gen(i);
            ret = 1;
        }
    }
    return ret;
}
 
/*-----------------------------------------------------------------------------
 *  doFareySequence: Optimized function to print Farey sequence of order n
 *
 *  Parameters: const int16_t n, float64_t *sequence
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void doFareySequence(const int16_t n, float64_t *sequence)
{

    float64_t x1 = 0.0f, y1 = 1.0f, x2 = 1.0f, y2 = n;                          // We know first two terms are 0/1 and 1/n
    float64_t x, y = 0.0f;                                                      // For next terms to be evaluated
    int32_t elem = 2;
        
    sequence[0u] = x1/y1;
    sequence[1u] = x2/y2;

    while (y != 1.0f) 
    {
        x = floor((y1 + n) / y2) * x2 - x1;                                     // Using recurrence relation to find the next term
        y = floor((y1 + n) / y2) * y2 - y1;
        sequence[elem] = x/y;                                                   // Print next term
        x1 = x2, x2 = x, y1 = y2, y2 = y;                                       // Update x1, y1, x2 and y2 for next iteration
        elem = ++elem % INT32_MAX; 
    }
}
// Transforms even the sequence 0,1,2,3,... into reasonably good random numbers 
// Challenge: improve on this in speed and "randomness"!
// This seems to pass several statistical tests, and is a bijective map (of 32-bit unsigned ints)
// https://github.com/christopherbatty/SDFGen/blob/master/util.h
//
/*-----------------------------------------------------------------------------
 *  randhash:  Transforms even the sequence 0,1,2,3,... into reasonably good random numbers
 *
 *  Parameters: uint8_t *init, uint16_t seed 
 *   
 *
 *  Return: uint16_t
 *----------------------------------------------------------------------------*/
uint16_t randhash(uint8_t *init, uint16_t seed)
{
   uint16_t i;
   if (*init == false)
   {
       srand(seed);
           *init = true;
   }
   i=(rand()^0xA3C59AC3u)*2654435769u;
   i^=(i>>16u);
   i*=2654435769u;
   i^=(i>>16u);
   i*=2654435769u;
   return i;
}
/*-----------------------------------------------------------------------------
 *  randhashdouble:  returns repeatable stateless pseudo-random number in range [a,b]
 *
 *  Parameters: uint16_t seed, uint8_t *init, float64_t a, float64_t b 
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
float64_t randhashdouble(uint16_t seed, uint8_t *init, float64_t a, float64_t b)
{
    return (b-a)*((float64_t)randhash(init,seed))/((float64_t)UINT16_MAX) + a; 
}
/*-----------------------------------------------------------------------------
 *  unrandhash:  the inverse of randhash
 *
 *  Parameters: uint16_t h
 *   
 *
 *  Return: uint16_t
 *----------------------------------------------------------------------------*/
uint16_t unrandhash(uint16_t h)
{
   h*=340573321u;
   h^=(h>>16);
   h*=340573321u;
   h^=(h>>16);
   h*=340573321u;
   h^=0xA3C59AC3u;
   return h;
}
/*
   The main function that finds shortest distances from src to
   all other vertices using Bellman-Ford algorithm.  The function
   also detects negative weight cycle
*/
/*-----------------------------------------------------------------------------
 *  doBellmanFord: Generating the first 'n' terms of Moser-de Bruijn Sequence sequence must be n+1
 *
 *  Parameters: const int16_t n, const int16_t *sequence
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t doBellmanFord(BF_Graph_t* graph, const int16_t src)
{
    int16_t Vm = graph->V;
    int16_t E = graph->E;
    int16_t u,v,i,j,weight;

    if (graph == NULL ) return -1;
    if (src >= sizeof(graph->dist)) return -1;
                
    /* Step 1: Initialize distances from src to all other vertices
       as INFINITE */
    for (i = 0; i < Vm; i++)
        graph->dist[i] = INT16_MAX;
    graph->dist[src] = 0;
  
    /* Step 2: Relax all edges |Vm| - 1 times. A simple shortest
       path from src to any other vertex can have at-most |Vm| - 1
       edges */
    for (i = 1; i <= Vm - 1; i++) 
    {
        for (j = 0; j < E; j++) 
        {
            u = graph->edge[j].src;
            v = graph->edge[j].dest;
            weight = graph->edge[j].weight;
            if (graph->dist[u] != INT16_MAX && graph->dist[u] + weight < graph->dist[v])
                graph->dist[v] = graph->dist[u] + weight;
        }
    }
  
    /* Step 3: check for negative-weight cycles.  The above step
       guarantees shortest distances if graph doesn't contain
       negative weight cycle.  If we get a shorter path, then there
       is a cycle. */
    for (i = 0; i < E; i++) 
    {
        u = graph->edge[i].src;
        v = graph->edge[i].dest;
        weight = graph->edge[i].weight;
        if (graph->dist[u] != INT16_MAX && graph->dist[u] + weight < graph->dist[v]) 
        {
            return -1;                                                          /* If negative cycle is detected, simply return */
        }
    }
  
    return 1;
}

/*-----------------------------------------------------------------------------
 *  doFloydWarshall: Floyd Warshall find distances between every pair of vertices
 *
 *  Parameters: const int16_t graph[FW_V][FW_V], int16_t **dist
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
/* dist[][] will be the output matrix
   that will finally have the shortest
   distances between every pair of vertices 
void doFloydWarshall(const int16_t graph[FW_V][FW_V], int16_t dist[FW_V][FW_V]) */
void doFloydWarshall(const int16_t graph[FW_V][FW_V], int16_t **dist)
{
    int16_t i, j, k;
 
    /* Initialize the solution matrix same
    as input graph matrix. Or we can say
    the initial values of shortest distances
    are based on shortest paths considering
    no intermediate vertex. */
    for (i = 0; i < FW_V; i++)
        for (j = 0; j < FW_V; j++)
            dist[i][j] = graph[i][j];
 
    /* Add all vertices one by one to
    the set of intermediate vertices.
    ---> Before start of an iteration,
    we have shortest distances between all
    pairs of vertices such that the
    shortest distances consider only the
    vertices in set {0, 1, 2, .. k-1} as
    intermediate vertices.
    ----> After the end of an iteration,
    vertex no. k is added to the set of
    intermediate vertices and the set becomes {0, 1, 2, ..
    k} */
    for (k = 0; k < FW_V; k++) {
        for (i = 0; i < FW_V; i++) {                                            // Pick all vertices as source one by one
            for (j = 0; j < FW_V; j++) {                                        // Pick all vertices as destination for the above picked source
                // If vertex k is on the shortest path from
                // i to j, then update the value of
                // dist[i][j]
                if (dist[i][j] > (dist[i][k] + dist[k][j])
                    && (dist[k][j] != FW_INF
                        && dist[i][k] != FW_INF))
                    dist[i][j] = dist[i][k] + dist[k][j];
            }
        }
    }
}
/*-----------------------------------------------------------------------------
 *  SieveOfEratosthenes:  list the prime numbers below n
 *
 *  Parameters: const int32_t n, int32_t *result 
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t SieveOfEratosthenes(const int32_t n, int32_t *result)
{
    int8_t ret = -1;
    int32_t p,i;
    bool *prime = NULL;
        
    if (result == NULL) return ret;
    if (sizeof(result) <= n) return ret;
        
    prime = (bool*)Malloc((n+1)*sizeof(bool));
        
    // Create a boolean array
    // "prime[0..n]" and initialize
    // all entries it as true.
    // A value in prime[i] will
    // finally be false if i is
    // Not a prime, else true.
    memset((void*)prime, true, sizeof(prime));
 
    for (p = 2; p * p <= n; p++)
    {
        // If prime[p] is not changed,
        // then it is a prime
        if (prime[p] == true)
        {
            // Update all multiples
            // of p greater than or
            // equal to the square of it
            // numbers which are multiple
            // of p and are less than p^2
            // are already been marked.
            for (i = p * p; i <= n; i += p)
                prime[i] = false;
        }
    }
 
    // Print all prime numbers
    for (p = 2; p <= n; p++)
        if (prime[p])
            result[p] = p;
                
    Free(prime,sizeof(prime));  
    return 1;
}
/*-----------------------------------------------------------------------------
 *  intlog2:  intlog
 *
 *  Parameters: int16_t x 
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
int16_t intlog2(int16_t x)
{
   int16_t exp=-1;
   while(x)
   {
      x>>=1;
      ++exp;
   }
   return exp;
}

/*-----------------------------------------------------------------------------
 *  get_barycentric:  a barycentric coordinate system is a coordinate system in 
 *                    which the location of a point is specified by reference to a simplex. 
 *                    The barycentric coordinates of a point can be interpreted as masses 
 *                    placed at the vertices of the simplex, such that the point is the 
 *                    center of mass of these masses
 *
 *  Parameters: float64_t x, int16_t *i, float64_t *f, int16_t i_low, int16_t i_high 
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void get_barycentric(float64_t x, int16_t *i, float64_t *f, int16_t i_low, int16_t i_high)
{
   float64_t s=floor(x);
   *i=(int16_t)s;
   if(*i<i_low)
   {
      *i=i_low;
      *f=0.0f;
   }
   else if(*i>i_high-2)
   {
      *i=i_high-2;
      *f=1.0f;
   }
   else
   {
      *f=(float64_t)(x-s);
   }
}

/*-----------------------------------------------------------------------------
 *  lerp:  takes 3 float parameters: one representing the value to interpolate from;  
 *         another representing the value to interpolate to and a final float  
 *         representing how far to interpolate 
 *
 *  Parameters: const float64_t value0, const float64_t value1, float64_t f 
 *   
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t lerp(const float64_t value0, const float64_t value1, float64_t f)
{ return (1-f)*value0 + f*value1; }


/*-----------------------------------------------------------------------------
 *  bilerp:  Bilinear interpolation is performed using linear interpolation   
 *           first in one direction, and then again in the other direction.  
 *
 *  Parameters: const float64_t v00, const float64_t v10, const float64_t v01,  
 *              const float64_t v11, const float64_t fx, const float64_t fy
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t bilerp(const float64_t v00, const float64_t v10, const float64_t v01, const float64_t v11, const float64_t fx, const float64_t fy)
{ 
   return lerp(lerp(v00, v10, fx), lerp(v01, v11, fx), fy);
}

/*-----------------------------------------------------------------------------
 *  trilerp:  Trilinear interpolation is a method of multivariate interpolation 
 *            on a 3-dimensional regular grid. It approximates the value of a 
 *            function at an intermediate point ( x , y , z ) within the local 
 *            axial rectangular prism linearly    
 *
 *  Parameters: const float64_t v000, const float64_t v100, const float64_t v010, 
 *              const float64_t v110, const float64_t v001, const float64_t v101, 
 *              const float64_t v011, const float64_t v111, const float64_t fx, 
 *              const float64_t fy, const float64_t fz  
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t trilerp(const float64_t v000, const float64_t v100, const float64_t v010, const float64_t v110, const float64_t v001, const float64_t v101, const float64_t v011, const float64_t v111, const float64_t fx, const float64_t fy, const float64_t fz) 
{
   return lerp(bilerp(v000, v100, v010, v110, fx, fy),  bilerp(v001, v101, v011, v111, fx, fy),  fz);
}

/*-----------------------------------------------------------------------------
 *  trilerp:  Trilinear interpolation is a method of multivariate interpolation 
 *            on a 3-dimensional regular grid. It approximates the value of a 
 *            function at an intermediate point ( x , y , z ) within the local 
 *            axial rectangular prism linearly    
 *
 *  Parameters: const float64_t v000, const float64_t v100, const float64_t v010, 
 *              const float64_t v110, const float64_t v001, const float64_t v101, 
 *              const float64_t v011, const float64_t v111, const float64_t fx, 
 *              const float64_t fy, const float64_t fz  
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t quadlerp(const float64_t v0000, const float64_t v1000, const float64_t v0100, const float64_t v1100, const float64_t v0010, const float64_t v1010, const float64_t v0110, const float64_t v1110, const float64_t v0001, const float64_t v1001, const float64_t v0101, const float64_t v1101, const float64_t v0011, const float64_t v1011, const float64_t v0111, const float64_t v1111, float64_t fx, float64_t fy, float64_t fz, float64_t ft) 
{
   return lerp(trilerp(v0000, v1000, v0100, v1100, v0010, v1010, v0110, v1110, fx, fy, fz), trilerp(v0001, v1001, v0101, v1101, v0011, v1011, v0111, v1111, fx, fy, fz), ft);
}

/*-----------------------------------------------------------------------------
 *  quadratic_bspline_weights:  f should be between 0 and 1, with f=0.5  
 *            corresponding to balanced weighting between w0 and w2  
 *
 *  Parameters: const float64_t f, float64_t *w0, float64_t *w1, float64_t *w2  
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void quadratic_bspline_weights(const float64_t f, float64_t *w0, float64_t *w1, float64_t *w2)
{
   *w0=(0.5f)*sqrt(f-1f);
   *w1=(0.75f)-sqrt(f-(0.5f));
   *w2=(0.5f)*sqrt(f);
}

/*-----------------------------------------------------------------------------
 *  cubic_interp_weights: f should be between 0 and 1   
 *
 *  Parameters: const float64_t f, float64_t *wneg1, float64_t *w0, float64_t *w1, float64_t *w2  
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void cubic_interp_weights(const float64_t f, float64_t *wneg1, float64_t *w0, float64_t *w1, float64_t *w2)
{
   float64_t f2 = f*f;
   float64_t f3 = f2*f;
   *wneg1=-(1.f/3f)*f+(1.f/2f)*f2-(1.f/6f)*f3;
   *w0=1f-f2+(1.f/2f)*(f3-f);
   *w1=f+(1.f/2f)*(f2-f3);
   *w2=(1.f/6f)*(f3-f);
}

/*-----------------------------------------------------------------------------
 *  cubic_interp: f should be between 0 and 1   
 *
 *  Parameters: const float64_t value_neg1, const float64_t value0,  
 *              const float64_t value1, const float64_t value2, float64_t f 
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t cubic_interp(const float64_t value_neg1, const float64_t value0, const float64_t value1, const float64_t value2, float64_t f)
{
   float64_t wneg1, w0, w1, w2;
   cubic_interp_weights(f, &wneg1, &w0, &w1, &w2);
   return wneg1*value_neg1 + w0*value0 + w1*value1 + w2*value2;
}

/*-----------------------------------------------------------------------------
 *  zero: zero the floating point array   
 *
 *  Parameters: float64_t *v  
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
void zero(float64_t *v)
{ 
    int16_t i;
    for(i=sizeof(v)-1; i>=0; --i) v[i]=0.0f; 
}

/*-----------------------------------------------------------------------------
 *  abs_max: return absolute max from floating point array   
 *
 *  Parameters: float64_t *v  
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t abs_max(const float64_t *v)
{
   float64_t m=0;
   int16_t i;
   
   for(i=sizeof(v)-1; i>=0; --i)
   {
      if(fabs(v[i])>m)
         m=fabs(v[i]);
   }
   return m;
}

/*-----------------------------------------------------------------------------
 *  bezlerp: lerp used with bezier function  
 *
 *  Parameters: const float64_t a,const float64_t b,const float64_t c  
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t bezlerp(const float64_t a,const float64_t b,const float64_t c)
{
        return a + (b - a) * c;
}

/*-----------------------------------------------------------------------------
 *  quadBezier: A quadratic Bézier curve is a curve created using three points. 
 *              The first and the third point define the start and the end of the curve. 
 *              The intermediate point influences the curvature of the line, 
 *              and is most of the time not on the curve 
 *
 *  Parameters: const float64_t t,const float64_t p0,const float64_t p1,const float64_t p2
 *              (iterating t draws the points in the trajectory of movment along the curve) 
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/ 
float64_t quadBezier(const float64_t t,const float64_t p0,const float64_t p1,const float64_t p2)
{
        float64_t l1 = bezlerp(p0, p1, t);
        float64_t l2 = bezlerp(p1, p2, t);
        float64_t quad = bezlerp(l1, l2, t);
        return quad;
}

/*-----------------------------------------------------------------------------
 *  cubicBezier: A cubic Bézier curve is a curve that’s created with four points. 
 *               The first and the fourth point define the start and the end of the curve. 
 *               The intermediates point once again just like the quadratic curve 
 *               influence the curvature of the line. 
 *
 *  Parameters: const float64_t t,const float64_t p0,const float64_t p1,const float64_t p2,const float64_t p3
 *              (iterating t draws the points in the trajectory of movment along the curve) 
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t cubicBezier( const float64_t t,const float64_t p0,const float64_t p1,const float64_t p2,const float64_t p3)
{
        return pow((1.0f - t),3.0f)*p0 + pow((3.0f*(1.0f - t)),2.0f)*t*p1 + pow((3.0f*(1.0f - t)*t),2.0f)*p2 + pow(t,3.0f)*p3;
}
// The MIT License
// Copyright © 2014 Inigo Quilez
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
// associated documentation files (the "Software"), to deal in the Software without restriction, 
// including without limitation the rights to use, copy, modify, merge, publish, distribute, 
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions: The above copyright notice and this permission 
// notice shall be included in all copies or substantial portions of the Software. 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
// PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR 
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


// Inverse bilinear interpolation: given four points defining a quadrilateral, compute the uv
// coordinates of any point in the plane that would give result to that point as a bilinear 
// interpolation of the four points.
//
// The problem can be solved through a quadratic equation. More information in this article:
//
// http://www.iquilezles.org/www/articles/ibilinear/ibilinear.htm


// given a point p and a quad defined by four points {a,b,c,d}, return the bilinear
// coordinates of p in the quad. Will not be in the range [0..1]^2 if the point is
// outside the quad.
/*-----------------------------------------------------------------------------
 *  cross2d : cross product x,y plane (2D) ground movement
 *                             
 *  Parameters: Vectr a, Vectr b 
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t cross2d( Vectr a, Vectr b ) { return a.x*b.y - a.y*b.x; }
/*-----------------------------------------------------------------------------
 *  dot2d : dot product x,y plane (2D) ground movement
 *                             
 *  Parameters: Vectr a, Vectr b 
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t dot2d( Vectr a, Vectr b ) { return a.x*b.x - a.y*b.y; }
/*-----------------------------------------------------------------------------
 *  invBilinear : inverse bilinear function
 *                             
 *  Parameters: Vectr p, Vectr a, Vectr b, Vectr c, Vectr d 
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr invBilinear( Vectr p, Vectr a, Vectr b, Vectr c, Vectr d )
{
    Vectr res; 
    Vectr e;
    Vectr f;
    Vectr g;
    Vectr h;
    float32_t w,ik2,v,u;
        
    float32_t k2 = cross2d( g, f );
    float32_t k1 = cross2d( e, f ) + cross2d( h, g );
    float32_t k0 = cross2d( h, e );
    e = vsub(b,a);
    f = vsub(d,a);
    g = vadd(vsub(a,b),vsub(c,d));
    h = vsub(p,a);
    
    res = mkvec(-1.0f,-1.0f,0.0f);
    if( abs(k2)<0.001f )                                                        // if edges are parallel, this is a linear equation
    {
        res = mkvec( (h.x*k1+f.x*k0)/(e.x*k1-g.x*k0), -k0/k1, 0.0f );
    }
    else                                                                        // otherwise, it's a quadratic
    {
        w = k1*k1 - 4.0f*k0*k2;
        if( w<0.0f ) return mkvec(-1.0f,-1.0f,0.0f);
        w = sqrt( w );

        ik2 = 0.5f/k2;
        v = (-k1 - w)*ik2;
        u = (h.x - f.x*v)/(e.x + g.x*v);
        
        if( u<0.0f || u>1.0f || v<0.0f || v>1.0f )
        {
           v = (-k1 + w)*ik2;
           u = (h.x - f.x*v)/(e.x + g.x*v);
        }
        res = mkvec( u, v, 0.0f );
    }
    
    return res;
}
/*-----------------------------------------------------------------------------
 *   vmul : multiply vector by a constant x
 *                             
 *  Parameters: Vectr a, float32_t x 
 *
 *  Return:     Vectr
 *----------------------------------------------------------------------------*/
Vectr vmul( Vectr a, float32_t x )
{
   Vectr res;
   res.x = a.x * x;
   res.y = a.y * x;
   res.z = a.z * x;
   return res;
}
/*-----------------------------------------------------------------------------
 *   vlength : return vector length
 *                             
 *  Parameters: Vectr v 
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t vlength( Vectr v )
{
   return sqrt((v.x*v.x)+(v.y*v.y)+(v.z*v.z));
}
/*-----------------------------------------------------------------------------
 *      sdSegment : distance to a line segment
 *                             
 *  Parameters: Vectr p, Vectr a, Vectr b 
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t sdSegment( Vectr p, Vectr a, Vectr b )
{
    p = vsub(p,a); b = vsub(b,a);
    return vlength(vsub(p,vmul(b,FMAX(0.0f,FMIN(1.0f,(dot2d(p,b)/dot2d(b,b)))))));
}
/*-----------------------------------------------------------------------------
 *  ArrayDotProduct : dot product of 2 double precision axis vector arrays
 *                             
 *  Parameters: const float64_t a[3u], const float64_t b[3u]
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
float64_t ArrayDotProduct( const float64_t a[3u], const float64_t b[3u] )
{
        return( (a[0u]*b[0u])+(a[1u]*b[1u])+(a[2u]*b[2u]) );
}
/* https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/rotation.h */
/*-----------------------------------------------------------------------------
 *  CERES_AngleAxisRotatePoint : angle axis rotate point
 *                             
 *  Parameters: const float64_t angle_axis[3], const float64_t pt[3], float64_t *result[3]
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void CERES_AngleAxisRotatePoint(float64_t *angle_axis, float64_t *pt, float64_t *result) 
{
        
  /* DCHECK_NE(pt, result) << "Inplace rotation is not supported."; */
  float64_t theta;
  float64_t costheta;
  float64_t sintheta;
  float64_t theta_inverse;

  float64_t w[3u];

  // Explicitly inlined evaluation of the cross product for
  // performance reasons.
  float64_t w_cross_pt[3u];
  float64_t tmp;        
  float64_t theta2 = ArrayDotProduct(angle_axis, angle_axis);
  
  w[0u] = angle_axis[0u] * theta_inverse;
  w[1u] = angle_axis[1u] * theta_inverse;
  w[2u] = angle_axis[2u] * theta_inverse;
  
  if (theta2 > DBL_EPSILON) 
  {
    // Away from zero, use the rodriguez formula
    //
    //   result = pt costheta +
    //            (w x pt) * sintheta +
    //            w (w . pt) (1 - costheta)
    //
    // We want to be careful to only evaluate the square root if the
    // norm of the angle_axis vector is greater than zero. Otherwise
    // we get a division by zero.
    //
    theta = sqrt(theta2);
    costheta = cos(theta);
    sintheta = sin(theta);
    theta_inverse = (1.0f) / theta;

    w[0u] = angle_axis[0u] * theta_inverse;
    w[1u] = angle_axis[1u] * theta_inverse;
    w[2u] = angle_axis[2u] * theta_inverse;

    // Explicitly inlined evaluation of the cross product for
    // performance reasons.
    w_cross_pt[0u] = w[1u] * pt[2u] - w[2u] * pt[1u];
    w_cross_pt[1u] = w[2u] * pt[0u] - w[0u] * pt[2u];
    w_cross_pt[2u] = w[0u] * pt[1u] - w[1u] * pt[0u];
 
    tmp = (w[0u] * pt[0] + w[1] * pt[1] + w[2] * pt[2]) * ((1.0f) - costheta);

    result[0u] = pt[0u] * costheta + w_cross_pt[0u] * sintheta + w[0u] * tmp;
    result[1u] = pt[1u] * costheta + w_cross_pt[1u] * sintheta + w[1u] * tmp;
    result[2u] = pt[2u] * costheta + w_cross_pt[2u] * sintheta + w[2u] * tmp;
  } 
  else 
  {
    // Near zero, the first order Taylor approximation of the rotation
    // matrix R corresponding to a vector w and angle w is
    //
    //   R = I + hat(w) * sin(theta)
    //
    // But sintheta ~ theta and theta * w = angle_axis, which gives us
    //
    //  R = I + hat(w)
    //
    // and actually performing multiplication with the point pt, gives us
    // R * pt = pt + w x pt.
    //
    // Switching to the Taylor expansion near zero provides meaningful
    // derivatives when evaluated using Jets.
    //
    // Explicitly inlined evaluation of the cross product for
    // performance reasons.
    w_cross_pt[0u] = angle_axis[1u] * pt[2u] - angle_axis[2u] * pt[1u];
    w_cross_pt[1u] = angle_axis[2u] * pt[0u] - angle_axis[0u] * pt[2u];
    w_cross_pt[2u] = angle_axis[0u] * pt[1u] - angle_axis[1u] * pt[0u];

    result[0u] = pt[0u] + w_cross_pt[0u];
    result[1u] = pt[1u] + w_cross_pt[1u];
    result[2u] = pt[2u] + w_cross_pt[2u];
  }
}
// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).  
/*-----------------------------------------------------------------------------
 *  SnavelyReprojectionError : noah snavely re-projection error
 *                             
 *  Parameters: const float64_t *camera, const float64_t *point, float64_t *residuals
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/ 
bool SnavelyReprojectionError(float64_t *camera, float64_t *point, float64_t *residuals, float64_t observed_x, float64_t observed_y )  
{
    float64_t p[3u],xp,yp,l1,l2,r2,distortion,focal,predicted_x,predicted_y;    // camera[0,1,2] are the angle-axis rotation.
    
    if ((camera == NULL) || ((point == NULL) || (residuals == NULL))) return false;
    if (((sizeof(camera) < 8u*sizeof(float64_t)) || (sizeof(point) < 3u*sizeof(float64_t))) || (sizeof(residuals) < 2u*sizeof(float64_t))) return false;
    CERES_AngleAxisRotatePoint(camera, point, &p);

    p[0u] += camera[3u];                                                        // camera[3,4,5] are the translation.
    p[1u] += camera[4u];
    p[2u] += camera[5u];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    xp = -p[0u] / p[2u];
    yp = -p[1u] / p[2u];

    l1 = camera[7u];                                                            // Apply second and fourth order radial distortion.
    l2 = camera[8u];
    r2 = xp * xp + yp * yp;
    distortion = 1.0f + r2 * (l1 + l2 * r2);

    focal = camera[6u];                                                         // Compute final projected point position.
    predicted_x = focal * distortion * xp;
    predicted_y = focal * distortion * yp;

    residuals[0u] = predicted_x - observed_x;                                  // The error is the difference between the predicted and observed position.
    residuals[1u] = predicted_y - observed_y;

    return true;
}
/*-----------------------------------------------------------------------------
 *  m3D_d_for_ETUdVectr: DECODE 3D Morton code : For loop (Early termination version)
 *
 *  Parameters: const float64_t z[4], const float64_t w[4], float64_t zw[4]
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t CERES_QuaternionProduct(const float64_t *z, const float64_t *w, float64_t *zw) 
{
  /*DCHECK_NE(z, zw) << "Inplace quaternion product is not supported.";
  DCHECK_NE(w, zw) << "Inplace quaternion product is not supported."; */
  if (((z == NULL) || (w == NULL)) || (zw == NULL)) return -1;
  if ((sizeof(z) < (4u*sizeof(float64_t))) || ((sizeof(w) < (4u*sizeof(float64_t))) || (sizeof(zw) < (4u*sizeof(float64_t))))) return -1;
  
  // clang-format off
  zw[0u] = z[0u] * w[0u] - z[1u] * w[1u] - z[2u] * w[2u] - z[3u] * w[3u];
  zw[1u] = z[0u] * w[1u] + z[1u] * w[0u] + z[2u] * w[3u] - z[3u] * w[2u];
  zw[2u] = z[0u] * w[2u] - z[1u] * w[3u] + z[2u] * w[0u] + z[3u] * w[1u];
  zw[3u] = z[0u] * w[3u] + z[1u] * w[2u] - z[2u] * w[1u] + z[3u] * w[0u];
  // clang-format on
  return 1;
}
/*-----------------------------------------------------------------------------
 *  CERES_UnitQuaternionRotatePoint: unit quaternion rotate point
 *
 *  Parameters: const float64_t q[4], const float64_t pt[3], float64_t result[3]
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t CERES_UnitQuaternionRotatePoint(const float64_t *q, const float64_t *pt, float64_t *result) 
{
  /* DCHECK_NE(pt, result) << "Inplace rotation is not supported."; */
  float64_t uv0;
  float64_t uv1;
  float64_t uv2;
  
  if (((q == NULL) || (pt == NULL)) || (result == NULL)) return -1;
  if ((sizeof(q) < (4u*sizeof(float64_t))) || ((sizeof(pt) < (3u*sizeof(float64_t))) || (sizeof(result) < (3u*sizeof(float64_t))))) return -1;
  
  // clang-format off
  uv0 = q[2u] * pt[2u] - q[3u] * pt[1u];
  uv1 = q[3u] * pt[0u] - q[1u] * pt[2u];
  uv2 = q[1u] * pt[1u] - q[2u] * pt[0u];
  uv0 += uv0;
  uv1 += uv1;
  uv2 += uv2;
  result[0u] = pt[0u] + q[0u] * uv0;
  result[1u] = pt[1u] + q[0u] * uv1;
  result[2u] = pt[2u] + q[0u] * uv2;
  result[0u] += q[2u] * uv2 - q[3u] * uv1;
  result[1u] += q[3u] * uv0 - q[1u] * uv2;
  result[2u] += q[1u] * uv1 - q[2u] * uv0;
  // clang-format on
  return 1;
}

/*-----------------------------------------------------------------------------
 *  CERES_QuaternionRotatePoint: quaternion rotate point
 *
 *  Parameters: const float64_t q[4], const float64_t pt[3], float64_t result[3]
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t CERES_QuaternionRotatePoint(const float64_t *q, const float64_t *pt, float64_t *result) 
{
  float64_t unit[4u];     // Make unit-norm version of q.
  float64_t scale;
  /* DCHECK_NE(pt, result) << "Inplace rotation is not supported."; */
  if (((q == NULL) || (pt == NULL)) || (result == NULL)) return -1;
  if ((sizeof(q) < (4u*sizeof(float64_t))) || ((sizeof(pt) < (3u*sizeof(float64_t))) || (sizeof(result) < (3u*sizeof(float64_t))))) return -1;
  
  scale = (float64_t)(1.0f) / sqrt(q[0u] * q[0u] + q[1u] * q[1u] + q[2u] * q[2u] + q[3u] * q[3u]);    // 'scale' is 1 / norm(q).

  unit[0u] = scale * q[0u];
  unit[1u] = scale * q[1u];
  unit[2u] = scale * q[2u];
  unit[3u] = scale * q[3u];

  CERES_UnitQuaternionRotatePoint(&unit, pt, result);
}
/*-----------------------------------------------------------------------------
 *  CERES_QuaternionToScaledRotation: quaternion scale to rotation
 *
 *  Parameters: const float64_t q[4], const float64_t R[3][3]
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t CERES_QuaternionToScaledRotation( const float64_t *q, float64_t **R ) 
{

  float64_t a;        // Make convenient names for elements of q.
  float64_t b;
  float64_t c;
  float64_t d;
  float64_t aa;
  float64_t ab;
  float64_t ac;
  float64_t ad;
  float64_t bb;
  float64_t bc;
  float64_t bd;
  float64_t cc;
  float64_t cd;
  float64_t dd;

  if ((q == NULL) || (R == NULL))  return -1;
  if ((sizeof(q) < (4u*sizeof(float64_t))) || (sizeof(R) < (3u*3u*sizeof(float64_t))))  return -1;
  
  // This is not to eliminate common sub-expression, but to
  // make the lines shorter so that they fit in 80 columns!
  aa = a * a;
  ab = a * b;
  ac = a * c;
  ad = a * d;
  bb = b * b;
  bc = b * c;
  bd = b * d;
  cc = c * c;
  cd = c * d;
  dd = d * d;

  a = q[0u];     // Make convenient names for elements of q.
  b = q[1u];
  c = q[2u];
  d = q[3u];
  
  R[0u][0u] = aa + bb - cc - dd;       // clang-format off
  R[0u][1u] = (2.0f) * (bc - ad);  
  R[0u][2u] = (2.0f) * (ac + bd);
  R[1u][0u] = (2.0f) * (ad + bc);  
  R[1u][1u] = aa - bb + cc - dd; 
  R[1u][2u] = (2.0f) * (cd - ab);
  R[2u][0u] = (2.0f) * (bd - ac);  
  R[2u][1u] = (2.0f) * (ab + cd);  
  R[2u][2u] = aa - bb - cc + dd;
  // clang-format on
  
  return 1;
}

/*-----------------------------------------------------------------------------
 *  CERES_QuaternionToScaledRotation: quaternion scale to rotation
 *
 *  Parameters: const float64_t q[4], const float64_t R[3][3]
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t CERES_QuaternionToRotation( const float64_t *q, float64_t **R ) 
{
  float64_t normalizer;
  int8_t i,j;

  if ((q == NULL) || (R == NULL))  return -1;
  if ((sizeof(q) < (4u*sizeof(float64_t))) || (sizeof(R) < (3u*3u*sizeof(float64_t))))  return -1;
  
  if (CERES_QuaternionToScaledRotation(q, R) != -1)
  {
      normalizer = q[0u] * q[0u] + q[1u] * q[1u] + q[2u] * q[2u] + q[3u] * q[3u];
      normalizer = (1.0f) / normalizer;

      for (i = 0; i < 3; ++i) 
      {
        for (j = 0; j < 3; ++j) 
        {
           R[i][j] *= normalizer;
        }
      }
      return 1;
  }
  return -1;
}
/*-----------------------------------------------------------------------------
 *  CERES_EulerAnglesToRotationMatrix : EulerAnglesToRotationMatrix
 *
 *  Parameters: const float64_t* euler, float64_t **R
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t CERES_EulerAnglesToRotationMatrix(  const float64_t* euler, float64_t **R ) 
{
  const float64_t kPi = 3.14159265358979323846f;

  float64_t pitch = (euler[0u] * (kPi / 180.0f));
  float64_t roll = (euler[1u] * (kPi / 180.0f));
  float64_t yaw = (euler[2u] * (kPi / 180.0f));

  float64_t c1 = cos(yaw);
  float64_t s1 = sin(yaw);
  float64_t c2 = cos(roll);
  float64_t s2 = sin(roll);
  float64_t c3 = cos(pitch);
  float64_t s3 = sin(pitch);
  
  if ((euler == NULL) || (R == NULL))  return -1;
  if ((sizeof(euler) < (3u*sizeof(float64_t))) || (sizeof(R) < (3u*3u*sizeof(float64_t))))  return -1;
  
  pitch = (euler[0u] * (kPi / 180.0f));
  roll = (euler[1u] * (kPi / 180.0f));
  yaw = (euler[2u] * (kPi / 180.0f));

  c1 = cos(yaw);
  s1 = sin(yaw);
  c2 = cos(roll);
  s2 = sin(roll);
  c3 = cos(pitch);
  s3 = sin(pitch);

  R[0u][0u] = c1 * c2;
  R[0u][1u] = -s1 * c3 + c1 * s2 * s3;
  R[0u][2u] = s1 * s3 + c1 * s2 * c3;

  R[1u][0u] = s1 * c2;
  R[1u][1u] = c1 * c3 + s1 * s2 * s3;
  R[1u][2u] = -c1 * s3 + s1 * s2 * c3;

  R[2u][0u] = -s2;
  R[2u][1u] = c2 * s3;
  R[2u][2u] = c2 * c3;
  
  return 1;
}
/*-----------------------------------------------------------------------------
 *  CERES_AngleAxisToRotationMatrix : AngleAxisToRotationMatrix
 *
 *  Parameters: const float64_t* angle_axis, float64_t** R
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t CERES_AngleAxisToRotationMatrix( const float64_t* angle_axis, float64_t** R ) 
{
  static const float64_t kOne = (1.0f);
  float64_t theta;
  float64_t wx;
  float64_t wy;
  float64_t wz;

  float64_t costheta;
  float64_t sintheta;
  float64_t theta2;
  
  if ((angle_axis == NULL) || (R == NULL))  return -1;
  if ((sizeof(angle_axis) < (3u*sizeof(float64_t))) || (sizeof(R) < (3u*3u*sizeof(float64_t))))  return -1;
  
  theta2 = ArrayDotProduct(angle_axis, angle_axis);
  if (theta2 > DBL_EPSILON) 
  {
    // We want to be careful to only evaluate the square root if the
    // norm of the angle_axis vector is greater than zero. Otherwise
    // we get a division by zero.
    theta = sqrt(theta2);
    wx = angle_axis[0u] / theta;
    wy = angle_axis[1u] / theta;
    wz = angle_axis[2u] / theta;

    costheta = cos(theta);
    sintheta = sin(theta);

    // clang-format off
    R[0u][0u] =     costheta   + wx*wx*(kOne -    costheta);
    R[1u][0u] =  wz*sintheta   + wx*wy*(kOne -    costheta);
    R[2u][0u] = -wy*sintheta   + wx*wz*(kOne -    costheta);
    R[0u][1u] =  wx*wy*(kOne - costheta)     - wz*sintheta;
    R[1u][1u] =     costheta   + wy*wy*(kOne -    costheta);
    R[2u][1u] =  wx*sintheta   + wy*wz*(kOne -    costheta);
    R[0u][2u] =  wy*sintheta   + wx*wz*(kOne -    costheta);
    R[1u][2u] = -wx*sintheta   + wy*wz*(kOne -    costheta);
    R[2u][2u] =     costheta   + wz*wz*(kOne -    costheta);
    // clang-format on
  } else {
    // Near zero, we switch to using the first order Taylor expansion.
    R[0u][0u] = kOne;
    R[1u][0u] = angle_axis[2u];
    R[2u][0u] = -angle_axis[1u];
    R[0u][1u] = -angle_axis[2u];
    R[1u][1u] = kOne;
    R[2u][1u] = angle_axis[0u];
    R[0u][2u] = angle_axis[1u];
    R[1u][2u] = -angle_axis[0u];
    R[2u][2u] = kOne;
  }
  return 1;
}

/*-----------------------------------------------------------------------------
 *  CERES_RotationMatrixToQuaternion : RotationMatrixToQuaternion
 *
 *  Parameters: const float64_t **R, float64_t *quaternion
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t CERES_RotationMatrixToQuaternion(  const float64_t **R, float64_t *quaternion ) 
{
  float64_t trace;
  float64_t t;
  int16_t i,j,k;

  if ((quaternion == NULL) || (R == NULL))  return -1;
  if ((sizeof(quaternion) < (4u*sizeof(float64_t))) || (sizeof(R) < (3u*3u*sizeof(float64_t))))  return -1;

  trace = R[0u][0u] + R[1u][1u] + R[2u][2u];
  
  if (trace >= 0.0f) 
  {
    t = sqrt(trace + (1.0f));
    quaternion[0] = (0.5f) * t;
    t = (0.5f) / t;
    quaternion[1u] = (R[2u][1u] - R[1u][2u]) * t;
    quaternion[2u] = (R[0u][2u] - R[2u][0u]) * t;
    quaternion[3u] = (R[1u][0u] - R[0u][1u]) * t;
  } else {
    i = 0;
    if (R[1u][1u] > R[0u][0u]) 
    {
      i = 1;
    }

    if (R[2u][2u] > R[i][i]) 
    {
      i = 2;
    }

    j = (i + 1) % 3;
    k = (j + 1) % 3;
    t = sqrt(R[i][i] - R[j][j] - R[k][k] + (1.0f));
    quaternion[i + 1] = (0.5f) * t;
    t = (0.5f) / t;
    quaternion[0u] = (R[k][j] - R[j][k]) * t;
    quaternion[j + 1u] = (R[j][i] + R[i][j]) * t;
    quaternion[k + 1u] = (R[k][i] + R[i][k]) * t;
  }
  return 1;
}
/*-----------------------------------------------------------------------------
 *  CERES_QuaternionToAngleAxis : QuaternionToAngleAxis
 *
 *  Parameters: const float64_t* quaternion, float64_t* angle_axis
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t CERES_QuaternionToAngleAxis(const float64_t* quaternion, float64_t* angle_axis) 
{
  float64_t q1;
  float64_t q2;
  float64_t q3;
  float64_t sin_squared_theta;
  float64_t sin_theta;
  float64_t cos_theta;
  float64_t two_theta;
  float64_t k;

  if ((quaternion == NULL) || (angle_axis == NULL))  return -1;
  if ((sizeof(quaternion) < (4u*sizeof(float64_t))) || (sizeof(angle_axis) < (3u*sizeof(float64_t))))  return -1;
  
  q1 = quaternion[1u];
  q2 = quaternion[2u];
  q3 = quaternion[3u];
  sin_squared_theta = q1 * q1 + q2 * q2 + q3 * q3;
  
  // For quaternions representing non-zero rotation, the conversion
  // is numerically stable.
  if (sin_squared_theta > (0.0f)) 
  {
    sin_theta = sqrt(sin_squared_theta);
    cos_theta = quaternion[0u];

    // If cos_theta is negative, theta is greater than pi/2, which
    // means that angle for the angle_axis vector which is 2 * theta
    // would be greater than pi.
    //
    // While this will result in the correct rotation, it does not
    // result in a normalized angle-axis vector.
    //
    // In that case we observe that 2 * theta ~ 2 * theta - 2 * pi,
    // which is equivalent saying
    //
    //   theta - pi = atan(sin(theta - pi), cos(theta - pi))
    //              = atan(-sin(theta), -cos(theta))
    //
    two_theta = (2.0f) * ((cos_theta < (0.0f)) ? atan2(-sin_theta, -cos_theta) : atan2(sin_theta, cos_theta));
    k = two_theta / sin_theta;
    angle_axis[0u] = q1 * k;
    angle_axis[1u] = q2 * k;
    angle_axis[2u] = q3 * k;
  } else {
    // For zero rotation, sqrt() will produce NaN in the derivative since
    // the argument is zero.  By approximating with a Taylor series,
    // and truncating at one term, the value and first derivatives will be
    // computed correctly when Jets are used.
    angle_axis[0u] = q1 * 2.0f;
    angle_axis[1u] = q2 * 2.0f;
    angle_axis[2u] = q3 * 2.0f;
  }
}
/*-----------------------------------------------------------------------------
 *  CERES_AngleAxisToQuaternion : AngleAxisToQuaternion
 *
 *  Parameters: const float64_t* angle_axis, float64_t* quaternion
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t CERES_AngleAxisToQuaternion(const float64_t* angle_axis, float64_t* quaternion) 
{
  float64_t theta;
  float64_t half_theta;
  float64_t k;

  float64_t a0;
  float64_t a1;
  float64_t a2;
  float64_t theta_squared;

  if ((quaternion == NULL) || (angle_axis == NULL))  return -1;
  if ((sizeof(quaternion) < (4u*sizeof(float64_t))) || (sizeof(angle_axis) < (3u*sizeof(float64_t))))  return -1;
  
  a0 = angle_axis[0u];
  a1 = angle_axis[1u];
  a2 = angle_axis[2u];
  theta_squared = a0 * a0 + a1 * a1 + a2 * a2;

  // For points not at the origin, the full conversion is numerically stable.
  if (theta_squared > (0.0f)) 
  {
    theta = sqrt(theta_squared);
    half_theta = theta * (0.5f);
    k = sin(half_theta) / theta;
    quaternion[0u] = cos(half_theta);
    quaternion[1u] = a0 * k;
    quaternion[2u] = a1 * k;
    quaternion[3u] = a2 * k;
  } else {
    // At the origin, sqrt() will produce NaN in the derivative since
    // the argument is zero.  By approximating with a Taylor series,
    // and truncating at one term, the value and first derivatives will be
    // computed correctly when Jets are used.
    quaternion[0u] = (1.0f);
    quaternion[1u] = a0 * 0.5f;
    quaternion[2u] = a1 * 0.5f;
    quaternion[3u] = a2 * 0.5f;
  }
  return 1;
}
/*-----------------------------------------------------------------------------
 *  CERES_RotationMatrixToAngleAxis : RotationMatrixToAngleAxis
 *
 *  Parameters: const float64_t **R, float64_t *angle_axis
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void CERES_RotationMatrixToAngleAxis( const float64_t **R, float64_t *angle_axis ) 
{
  float64_t quaternion[4u];
  CERES_RotationMatrixToQuaternion(R, quaternion);
  CERES_QuaternionToAngleAxis(quaternion, angle_axis);
  return;
}
/*-----------------------------------------------------------------------------
 *  KnuthRng_seed : Create seed from s0=1 s1=1 
 *
 *  Parameters: uint32_t *s0, uint32_t *s1, uint32_t *state0, uint32_t *state1 
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/
void KnuthRng_seed(uint32_t *s0, uint32_t *s1, uint32_t *state0, uint32_t *state1)
{
  if(*s0 == 0ul) *s0 = 1ul;
  if(*s1 == 0ul) *s1 = 1ul;

  *state0 = ((uint32_t)(((uint64_t)*s0) % KNUTH_M));
  *state1 = ((uint32_t)(((uint64_t)*s1) % KNUTH_M));
}

/*-----------------------------------------------------------------------------
 *  KnuthRng_get : get a new set of numbers
 *
 *  Parameters: uint32_t *state0, uint32_t *state1 
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/  
uint32_t KnuthRng_get( uint32_t *state0, uint32_t *state1 )
{
  uint32_t state2 = ((uint32_t)((KNUTH_A1*(uint64_t)*state1 + KNUTH_A2*(uint64_t)*state0) % KNUTH_M));
  *state0 = *state1;
  *state1 = state2;

  return(state2);
}
/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 *                                                                            *
 * Ported for mikroE C by ACP Aviation                                        *
 ******************************************************************************/
 /*-----------------------------------------------------------------------------
 *  Matrix3f64_Identity : set identity for 3x3 matrix
 *                             
 *  Parameters: Matrix3f64_t *a
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Matrix3f64_Identity(Matrix3f64_t *a)
{
    if (a==NULL) {
        return false;
    }

    a->a.x = (float64_t)1.0f;
    a->a.y = (float64_t)0.0f;
    a->a.z = (float64_t)0.0f;
    a->b.x = (float64_t)0.0f;
    a->b.y = (float64_t)1.0f;
    a->b.z = (float64_t)0.0f;
    a->c.x = (float64_t)0.0f;
    a->c.y = (float64_t)0.0f;
    a->c.z = (float64_t)1.0f;

    return true;
}

 /*-----------------------------------------------------------------------------
 *  matrix33_identity : set identity for 3x3 matrix
 *                             
 *  Parameters: mat33 *mtx
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool matrix33_identity(mat33 *mtx)
{
    if (mtx==NULL) {
        return false;
    }

    mtx->m[0][0] = (float32_t)1.0f;
    mtx->m[0][1] = (float32_t)0.0f;
    mtx->m[0][2] = (float32_t)0.0f;
    mtx->m[1][0] = (float32_t)0.0f;
    mtx->m[1][1] = (float32_t)1.0f;
    mtx->m[1][2] = (float32_t)0.0f;
    mtx->m[2][0] = (float32_t)0.0f;
    mtx->m[2][1] = (float32_t)0.0f;
    mtx->m[2][2] = (float32_t)1.0f;

    return true;
}

 /*-----------------------------------------------------------------------------
 *  Matrix3_setIdentity : set identity for 3x3 matrix
 *                             
 *  Parameters: none
 *
 *  Return: Matrix3f64_t
 *----------------------------------------------------------------------------*/
Matrix3f64_t Matrix3_setIdentity() 
{
        Matrix3f64_t a;
        
        a.b.x=a.c.x = 0.0f;
        a.a.y=a.c.y = 0.0f;
        a.a.z=a.b.z = 0.0f;   
        a.a.x=a.b.y=a.c.z = 1.0f;
        return a;
}

 /*-----------------------------------------------------------------------------
 * matrix33_subtract : subtract mtx = a-b
 *                             
 *  Parameters: mat33 *mtx, mat33 *a, mat33 *b
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool matrix33_subtract(mat33 *mtx, mat33 *a, mat33 *b)
{
    if (mtx==NULL) {
        return false;
    }

    mtx->m[0][0] = a->m[0][0] - b->m[0][0];
    mtx->m[0][1] = a->m[0][1] - b->m[0][1];
    mtx->m[0][2] = a->m[0][2] - b->m[0][2];
    mtx->m[1][0] = a->m[1][0] - b->m[1][0];
    mtx->m[1][1] = a->m[1][1] - b->m[1][1];
    mtx->m[1][2] = a->m[1][2] - b->m[1][2];
    mtx->m[2][0] = a->m[2][0] - b->m[2][0];
    mtx->m[2][1] = a->m[2][1] - b->m[2][1];
    mtx->m[2][2] = a->m[2][2] - b->m[2][2];

    return true;
}

 /*-----------------------------------------------------------------------------
 *  Matrix3_subtract : subtract and return a-b
 *                             
 *  Parameters: const Matrix3f64_t a, const Matrix3f64_t b
 *
 *  Return: Matrix3f64_t
 *----------------------------------------------------------------------------*/
Matrix3f64_t Matrix3_subtract( const Matrix3f64_t a, const Matrix3f64_t b )
{

    Matrix3f64_t mtx;
        
    mtx.a.x = a.a.x - b.a.x;
    mtx.a.y = a.a.y - b.a.y;
    mtx.a.z = a.a.z - b.a.z;
    mtx.b.x = a.b.x - b.b.x;
    mtx.b.y = a.b.y - b.b.y;
    mtx.b.z = a.b.z - b.b.z;
    mtx.c.x = a.c.x - b.c.x;
    mtx.c.y = a.c.y - b.c.y;
    mtx.c.z = a.c.z - b.c.z;

    return mtx;
}

 /*-----------------------------------------------------------------------------
 *  matrix33_add : subtract mtx = a+b
 *                             
 *  Parameters: mat33 *mtx, mat33 *a, mat33 *b
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool matrix33_add(mat33 *mtx, mat33 *a, mat33 *b)
{
    if (mtx==NULL) {
        return false;
    }

    mtx->m[0][0] = a->m[0][0] + b->m[0][0];
    mtx->m[0][1] = a->m[0][1] + b->m[0][1];
    mtx->m[0][2] = a->m[0][2] + b->m[0][2];
    mtx->m[1][0] = a->m[1][0] + b->m[1][0];
    mtx->m[1][1] = a->m[1][1] + b->m[1][1];
    mtx->m[1][2] = a->m[1][2] + b->m[1][2];
    mtx->m[2][0] = a->m[2][0] + b->m[2][0];
    mtx->m[2][1] = a->m[2][1] + b->m[2][1];
    mtx->m[2][2] = a->m[2][2] + b->m[2][2];

    return true;
}

 /*-----------------------------------------------------------------------------
 *  Matrix3_add : subtract and return a+b
 *                             
 *  Parameters: const Matrix3f64_t a, const Matrix3f64_t b
 *
 *  Return: Matrix3d64_t
 *----------------------------------------------------------------------------*/
Matrix3f64_t Matrix3_add( const Matrix3f64_t a, const Matrix3f64_t b )
{

    Matrix3f64_t mtx;
        
    mtx.a.x = a.a.x + b.a.x;
    mtx.a.y = a.a.y + b.a.y;
    mtx.a.z = a.a.z + b.a.z;
    mtx.b.x = a.b.x + b.b.x;
    mtx.b.y = a.b.y + b.b.y;
    mtx.b.z = a.b.z + b.b.z;
    mtx.c.x = a.c.x + b.c.x;
    mtx.c.y = a.c.y + b.c.y;
    mtx.c.z = a.c.z + b.c.z;

    return mtx;
}

 /*-----------------------------------------------------------------------------
 *  Matrix3_inverse : 3x3 matrix inverse.
 *                             
 *  Parameters: const Matrix3f64_t a, const float64_t d
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
Matrix3f64_t Matrix3_inverse(const Matrix3f64_t a, const float64_t d)
{
    Matrix3f64_t inv;
    /* float64_t d = Matrix3_det(a);  determinate product 

       format requests the caller to do the det first as we always just return result 
           if (d != 0.0f)  
           {
    */
    inv.a.x = (a.b.y * a.c.z - a.c.y * a.b.z) / d;
    inv.a.y = (a.a.z * a.c.y - a.a.y * a.c.z) / d;
    inv.a.z = (a.a.y * a.b.z - a.a.z * a.b.y) / d;
    inv.b.x = (a.b.z * a.c.x - a.b.x * a.c.z) / d;
    inv.b.y = (a.a.x * a.c.z - a.a.z * a.c.x) / d;
    inv.b.z = (a.b.x * a.a.z - a.a.x * a.b.z) / d;
    inv.c.x = (a.b.x * a.c.y - a.c.x * a.b.y) / d;
    inv.c.y = (a.c.x * a.a.y - a.a.x * a.c.y) / d;
    inv.c.z = (a.a.x * a.b.y - a.b.x * a.a.y) / d;
    /* } */

    return inv;
}

/*-----------------------------------------------------------------------------
 *  mArrayScl : multiply a matrix by a scalar.
 *
 *
 *  Parameters: float64_t s, float32_t **a
 *
 *  Return: float32_t**
 *----------------------------------------------------------------------------*/
float32_t** mArrayScl(float64_t s, float32_t a[3][3]) 
{
        float32_t sa[3u][3u];
        int8_t i,j;
        
        for (i = 0; i < 3; ++i) 
        {
                for (j = 0; j < 3; ++j) 
                {
                        sa[i][j] = s * a[i][j];
                }
        }
        return &sa;
}

/*-----------------------------------------------------------------------------
 *  Matrix3f64Scl : multiply a matrix by a scalar.
 *
 *
 *  Parameters: float64_t s, const Matrix3f64_t a
 *
 *  Return: Matrix3f64_t
 *----------------------------------------------------------------------------*/
Matrix3f64_t Matrix3f64Scl(float64_t s, const Matrix3f64_t a) 
{
        Matrix3f64_t mat;
        
        mat.a.x = s * a.a.x;
        mat.b.x = s * a.b.x;
        mat.c.x = s * a.c.x;
        mat.a.y = s * a.a.y;
        mat.b.y = s * a.b.y;
        mat.c.y = s * a.c.y;        
        mat.a.z = s * a.a.z;
        mat.b.z = s * a.b.z;
        mat.c.z = s * a.c.z;        
                
        return mat;
}

 /*-----------------------------------------------------------------------------
 *  matrix33_multiply : 3x3 matrix multiply
 *                             
 *  Parameters: mat33 *a, const mat33 m0, const mat33 m1
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool matrix33_multiply( mat33 *a, const mat33 m0, const mat33 m1  ) 
{
    if (a == NULL) return false;
    a->m[0u][0u]= m0.m[0u][0u]*m1.m[0u][0u] + m0.m[0u][1u]*m1.m[1u][0u] + m0.m[0u][2u]*m1.m[2u][0u];
    a->m[1u][0u]= m0.m[1u][0u]*m1.m[0u][0u] + m0.m[1u][1u]*m1.m[1u][0u] + m0.m[1u][2u]*m1.m[2u][0];
    a->m[2u][0u]= m0.m[2u][0u]*m1.m[0u][0u] + m0.m[2u][1u]*m1.m[1u][0u] + m0.m[2u][2u]*m1.m[2u][0];

    a->m[0u][1u]= m0.m[0u][0u]*m1.m[0u][1u] + m0.m[0u][1u]*m1.m[1u][1u] + m0.m[0u][2u]*m1.m[2u][1u];
    a->m[1u][1u]= m0.m[1u][0u]*m1.m[0u][1u] + m0.m[1u][1u]*m1.m[1u][1u] + m0.m[1u][2u]*m1.m[2u][1u];
    a->m[2u][1u]= m0.m[2u][0u]*m1.m[0u][1u] + m0.m[2u][1u]*m1.m[1u][1u] + m0.m[2u][2u]*m1.m[2u][1u];

    a->m[0u][2u]= m0.m[0u][0]*m1.m[0u][2u] + m0.m[0u][1u]*m1.m[1u][2u] + m0.m[0u][2u]*m1.m[2u][2u];
    a->m[1u][2u]= m0.m[1u][0]*m1.m[0u][2u] + m0.m[1u][1u]*m1.m[1u][2u] + m0.m[1u][2u]*m1.m[2u][2u];
    a->m[2u][2u]= m0.m[2u][0]*m1.m[0u][2u] + m0.m[2u][1u]*m1.m[1u][2u] + m0.m[2u][2u]*m1.m[2u][2u];

    return true;
}

 /*-----------------------------------------------------------------------------
 *  matrix33_multiply : 3x3 matrix multiply
 *                             
 *  Parameters: const Matrix3f64_t m0, const Matrix3f64_t m1
 *
 *  Return: Matrix3f64_t
 *----------------------------------------------------------------------------*/
Matrix3f64_t Matrix3_multiply( const Matrix3f64_t m0, const Matrix3f64_t m1  ) 
{
    Matrix3f64_t a;
        
    a.a.x= m0.a.x*m1.a.x + m0.a.y*m1.b.x + m0.a.z*m1.c.x;
    a.b.x= m0.b.x*m1.a.x + m0.b.y*m1.b.x + m0.b.z*m1.c.x;
    a.c.x= m0.c.x*m1.a.x + m0.c.y*m1.b.x + m0.c.z*m1.c.x;

    a.a.y= m0.a.x*m1.a.y + m0.a.y*m1.b.y + m0.a.z*m1.c.y;
    a.b.y= m0.b.x*m1.a.y + m0.b.y*m1.b.y + m0.b.z*m1.c.y;
    a.c.y= m0.c.x*m1.a.y + m0.c.y*m1.b.y + m0.c.z*m1.c.y;

    a.a.z= m0.a.x*m1.a.z + m0.a.y*m1.b.z + m0.a.z*m1.c.z;
    a.b.z= m0.b.x*m1.a.z + m0.b.y*m1.b.z + m0.b.z*m1.c.z;
    a.c.z= m0.c.x*m1.a.z + m0.c.y*m1.b.z + m0.c.z*m1.c.z;

    return a;
}

/*-----------------------------------------------------------------------------
 *  math_rot2cayley : rotation to cayley
 *
 *
 *  Parameters: const mat33 *R, Vectr *cayley
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t math_rot2cayleyM33( const mat33 *R, Vectr *cayley )
{
  mat33 C1;
  mat33 C2;
  mat33 C;
  mat33 ID,INV;
  int8_t ret = -1;
  
  if (matrix33_identity( &ID ) == true) 
  {
      if (matrix33_subtract( &C1, R, &ID ) == true)
      {
          if (matrix33_add( &C2, R, &ID ) == true)
          {
              if (matrix33_inverse( &INV, C2 ) == true)
              {
                   if (matrix33_multiply( &C, C1, INV ) == true) 
                   {
                       cayley->x = -C.m[1u][2u];
                       cayley->y = C.m[0u][2u];
                       cayley->z = -C.m[0u][1u];
                       ret = 1;
                   }
              }
         }
      }
  }
  return ret;
}

/*-----------------------------------------------------------------------------
 *  math_rot2cayleyF64 : rotation to cayley
 *
 *
 *  Parameters: const Matrix3f64_t R
 *
 *  Return: dVectr
 *----------------------------------------------------------------------------*/
dVectr math_rot2cayleyF64( const Matrix3f64_t R ) 
{
   dVectr cayley;
   Matrix3f64_t C,C1,C2,Iden,Inv;
   float64_t Det;

   Iden = Matrix3_setIdentity();
   C1 = Matrix3_subtract( R, Iden );
   C2 = Matrix3_add( R, Iden);
   Det = Matrix3_det( C2 );
   if (Det != 0.0f)
   {
       Inv = Matrix3_inverse( C2, Det );
       C = Matrix3_multiply( C1, Inv );
       cayley.x = -C.b.z;
       cayley.y = C.a.z;
       cayley.z = -C.a.y;
   }
   return cayley;
}
        
/*-----------------------------------------------------------------------------
 *  cayley2rot_reducedF32 : cayley to rotation reduced 
 *
 *
 *  Parameters: const float32_t *cayley
 *
 *  Return: float32_t**
 *----------------------------------------------------------------------------*/
float32_t** cayley2rot_reducedF32( const float32_t *cayley )
{
  float32_t R[3u][3u];

  R[0u][0u] = 1.0f+pow(cayley[0u],2.0f)-pow(cayley[1u],2.0f)-pow(cayley[2u],2.0f);
  R[0u][1u] = 2.0f*(cayley[0u]*cayley[1u]-cayley[2u]);
  R[0u][2u] = 2.0f*(cayley[0u]*cayley[2u]+cayley[1u]);
  R[1u][0u] = 2.0f*(cayley[0u]*cayley[1u]+cayley[2u]);
  R[1u][1u] = 1.0f-pow(cayley[0u],2.0f)+pow(cayley[1u],2.0f)-pow(cayley[2u],2.0f);
  R[1u][2u] = 2.0f*(cayley[1u]*cayley[2u]-cayley[0u]);
  R[2u][0u] = 2.0f*(cayley[0u]*cayley[2u]-cayley[1u]);
  R[2u][1u] = 2.0f*(cayley[1u]*cayley[2u]+cayley[0u]);
  R[2u][2u] = 1.0f-pow(cayley[0u],2.0f)-pow(cayley[1u],2.0f)+pow(cayley[2u],2.0f);

  return &R;
}

/*-----------------------------------------------------------------------------
 *  cayley2rot_reducedM33 : cayley to rotation reduced 
 *
 *
 *  Parameters: const float32_t *cayley
 *
 *  Return: mat33
 *----------------------------------------------------------------------------*/
mat33 cayley2rot_reducedM33( const Vectr cayley)
{
  mat33 R;

  R.m[0u][0u] = 1.0f+pow(cayley.x,2.0f)-pow(cayley.y,2.0f)-pow(cayley.z,2.0f);
  R.m[0u][1u] = 2.0f*(cayley.x*cayley.y-cayley.z);
  R.m[0u][2u] = 2.0f*(cayley.x*cayley.z+cayley.y);
  R.m[1u][0u] = 2.0f*(cayley.x*cayley.y+cayley.z);
  R.m[1u][1u] = 1.0f-pow(cayley.x,2.0f)+pow(cayley.y,2.0f)-pow(cayley.z,2.0f);
  R.m[1u][2u] = 2.0f*(cayley.y*cayley.z-cayley.x);
  R.m[2u][0u] = 2.0f*(cayley.x*cayley.z-cayley.y);
  R.m[2u][1u] = 2.0f*(cayley.y*cayley.z+cayley.x);
  R.m[2u][2u] = 1.0f-pow(cayley.x,2.0f)-pow(cayley.y,2.0f)+pow(cayley.z,2.0f);

  return R;
}

/*-----------------------------------------------------------------------------
 *  cayley2rot_reducedF64 : cayley to rotation reduced 
 *
 *
 *  Parameters: const float32_t *cayley
 *
 *  Return: Matrix3f64_t
 *----------------------------------------------------------------------------*/
Matrix3f64_t cayley2rot_reducedF64( const dVectr cayley )
{
  Matrix3f64_t a;

  a.a.x = 1.0f+pow(cayley.x,2.0f)-pow(cayley.y,2.0f)-pow(cayley.z,2.0f);
  a.a.y = 2.0f*(cayley.x*cayley.y-cayley.z);
  a.a.z = 2.0f*(cayley.x*cayley.z+cayley.y);
  a.b.x = 2.0f*(cayley.x*cayley.y+cayley.z);
  a.b.y = 1.0f-pow(cayley.x,2.0f)+pow(cayley.y,2.0f)-pow(cayley.z,2.0f);
  a.b.z = 2.0f*(cayley.y*cayley.z-cayley.x);
  a.c.x = 2.0f*(cayley.x*cayley.z-cayley.y);
  a.c.y = 2.0f*(cayley.y*cayley.z+cayley.x);
  a.c.z = 1.0f-pow(cayley.x,2.0f)-pow(cayley.y,2.0f)+pow(cayley.z,2.0f);

  return a;
}

/*-----------------------------------------------------------------------------
 *  cayley2rotF32 : cayley to rotation 
 *
 *
 *  Parameters: const float32_t *cayley
 *
 *  Return: float32_t**
 *----------------------------------------------------------------------------*/
float32_t** cayley2rotF32( const float32_t *cayley)
{
  float32_t R[3u][3u], X[3u][3u];
  float64_t scale = 1.0f+pow(cayley[0u],2.0f)+pow(cayley[1u],2.0f)+pow(cayley[2u],2.0f);
  float64_t tmp;
  
  R[0u][0u] = 1.0f+pow(cayley[0u],2.0f)-pow(cayley[1u],2.0f)-pow(cayley[2u],2.0f);
  R[0u][1u] = 2.0f*(cayley[0u]*cayley[1u]-cayley[2u]);
  R[0u][2u] = 2.0f*(cayley[0u]*cayley[2u]+cayley[1u]);
  R[1u][0u] = 2.0f*(cayley[0u]*cayley[1u]+cayley[2u]);
  R[1u][1u] = 1.0f-pow(cayley[0u],2.0f)+pow(cayley[1u],2.0f)-pow(cayley[2u],2.0f);
  R[1u][2u] = 2.0f*(cayley[1u]*cayley[2u]-cayley[0u]);
  R[2u][0u] = 2.0f*(cayley[0u]*cayley[2u]-cayley[1u]);
  R[2u][1u] = 2.0f*(cayley[1u]*cayley[2u]+cayley[0u]);
  R[2u][2u] = 1.0f-pow(cayley[0u],2.0f)-pow(cayley[1u],2.0f)+pow(cayley[2u],2.0f);
  tmp = (1.0f/scale);
  /* TODO WTF !!!!!!! &X = mArrayScl(tmp, &R); */ 
  return &X;
}

/*-----------------------------------------------------------------------------
 *  cayley2rotM33 : cayley to rotation 
 *
 *
 *  Parameters: const Vectr cayley
 *
 *  Return: mat33
 *----------------------------------------------------------------------------*/
mat33 cayley2rotM33( const Vectr cayley )
{
  mat33 R;
  float64_t scale = 1.0f+pow(cayley.x,2.0f)+pow(cayley.y,2.0f)+pow(cayley.z,2.0f);

  R.m[0u][0u] = 1.0f+pow(cayley.x,2.0f)-pow(cayley.y,2.0f)-pow(cayley.z,2.0f);
  R.m[0u][1u] = 2.0f*(cayley.x*cayley.y-cayley.z);
  R.m[0u][2u] = 2.0f*(cayley.x*cayley.z+cayley.y);
  R.m[1u][0u] = 2.0f*(cayley.x*cayley.y+cayley.z);
  R.m[1u][1u] = 1.0f-pow(cayley.x,2.0f)+pow(cayley.y,2.0f)-pow(cayley.z,2.0f);
  R.m[1u][2u] = 2.0f*(cayley.y*cayley.z-cayley.x);
  R.m[2u][0u] = 2.0f*(cayley.x*cayley.z-cayley.y);
  R.m[2u][1u] = 2.0f*(cayley.y*cayley.z+cayley.x);
  R.m[2u][2u] = 1.0f-pow(cayley.x,2.0f)-pow(cayley.y,2.0f)+pow(cayley.z,2.0f);

  R = mscl((1.0f/scale), R); 
  return R;
}

/*-----------------------------------------------------------------------------
 *  cayley2rotF64 : cayley to rotation 
 *
 *
 *  Parameters: const dVectr cayley
 *
 *  Return: Matrix3f64_t
 *----------------------------------------------------------------------------*/
Matrix3f64_t cayley2rotF64( const dVectr cayley )
{
  Matrix3f64_t R;
  float64_t scale = 1.0f+pow(cayley.x,2.0f)+pow(cayley.y,2.0f)+pow(cayley.z,2.0f);

  R.a.x = 1.0f+pow(cayley.x,2.0f)-pow(cayley.y,2.0f)-pow(cayley.z,2.0f);
  R.a.y = 2.0f*(cayley.x*cayley.y-cayley.z);
  R.a.z = 2.0f*(cayley.x*cayley.z+cayley.y);
  R.b.x = 2.0f*(cayley.x*cayley.y+cayley.z);
  R.b.y = 1.0f-pow(cayley.x,2.0f)+pow(cayley.y,2.0f)-pow(cayley.z,2.0f);
  R.b.z = 2.0f*(cayley.y*cayley.z-cayley.x);
  R.c.x = 2.0f*(cayley.x*cayley.z-cayley.y);
  R.c.y = 2.0f*(cayley.y*cayley.z+cayley.x);
  R.c.z = 1.0f-pow(cayley.x,2.0f)-pow(cayley.y,2.0f)+pow(cayley.z,2.0f);

  R = Matrix3f64Scl((1.0f/scale), R); 
  return R;
}
/*-----------------------------------------------------------------------------
 *  math_arun : arun transformation
 *
 *
 *  Parameters: mat33 V, const mat33 U
 *
 *  Return: Matrix3f64_t
 *----------------------------------------------------------------------------*/
mat33 math_arun( const mat33 V, const mat33 U )
{
  mat33 R,Vprime;
  if (matrix33_multiply( &R, V, matrix33_transpose(U)  )==true)
  {
     if (matrix33_det( R ) < 0.0f)
     {
        memcpy((void*)&Vprime, (void*)&V, sizeof(V));
        Vprime.m[2u][0u] = -Vprime.m[2u][0u];        
        Vprime.m[2u][1u] = -Vprime.m[2u][1u];        
        Vprime.m[2u][2u] = -Vprime.m[2u][2u];        
        if (matrix33_multiply( &R, Vprime, matrix33_transpose(U) )==true)
        {
            return R;
        }                        
     }
     else
     {
        return R;
     }
  }         
  memset((void*)&R,0.0f,sizeof(R));
  return R;
}
//
// https://github.com/mdarin/Moore-Penrose-inversion/blob/master/src/main.go
// W are the vector of singular values of a
// U,V are left and right orthogonal transformation matrices
/*-----------------------------------------------------------------------------
 *  moore_penrose : moore_penrose inversion (in 3D)
 *
 *
 *  Parameters: const mat33 V, const mat33 U, Vectr W
 *
 *  Return: mat33
 *----------------------------------------------------------------------------*/
mat33 moore_penrose( const mat33 V, const mat33 U, Vectr W )
{
  mat33 UT,Whash,temp,ApInv;
 
  memset((void*)&ApInv,0,sizeof(ApInv));                                        // default return a zero matrix if something is wrong  
  UT = matrix33_transpose( U );                                                 // get U transposed
  if (matrix33_identity(&Whash)==true)                                          // create identity matrix for W diagonalization
  {
       Whash.m[0u][0u] = 1.0f/W.x;                                              //inverse singular values and diagonalilze W to Whash
       Whash.m[1u][1u] = 1.0f/W.y;          
       Whash.m[2u][2u] = 1.0f/W.z;        
       if (matrix33_multiply( &temp, V, Whash  )==true)                         // make the Moore–Penrose inversion
       {
           if (matrix33_multiply( &ApInv, temp, UT  )==true) 
           {
              return ApInv;        
           }                  
       }
  }
  return ApInv;
}

/*
   https://github.com/wallenut/Jacobi-SVD/blob/master/hw2.c
   Ref: Allen Wang Yale University
*/
/*-----------------------------------------------------------------------------
 *  SVD_g_eigen : 
 *
 *
 *  Parameters: float64_t* a, int16_t n, int16_t row, int16_t col
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t SVD_g_eigen(float64_t* a, int16_t n, int16_t row, int16_t col)
{
        int16_t i;
        float64_t sum = 0.0f;
        for(i = 0; i < n; ++i)
        {
            sum += a[i * n + row] * a[i * n + col];
        }
        return sum;
}

/*-----------------------------------------------------------------------------
 *  SVD_sign : 
 *
 *
 *  Parameters: float64_t val
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/        
float64_t SVD_sign(float64_t val)
{
        if(val < 0.0f)
        {
                return -1;
        }
        return 1;
}

/*-----------------------------------------------------------------------------
 *  SVD_arrange : 
 *
 *
 *  Parameters: float64_t* a, int16_t n
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void SVD_arrange(float64_t* a, int16_t n)
{
   int16_t i;
   int16_t j;
   for (i = 0; i < n; ++i)
   {
        for (j = 0; j < n; ++j)
        {
            if (i != j)
            {
                 a[i * n + j] = 0.0f;
            }
            else
            {
                 a[i * n + j] = 1.0f;
            }
         }
   }
}

/*-----------------------------------------------------------------------------
 *  SVD_rotation : 
 *
 *
 *  Parameters: float64_t *a, int16_t n, int16_t p, int16_t q, float64_t* u
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void SVD_rotation(float64_t *a, int16_t n, int16_t p, int16_t q, float64_t* u)
{
    int16_t i;
    float64_t a_pp, a_pq, a_qq, t, c, s, temp;
    a_pp = SVD_g_eigen(a, n, p, p);
    a_pq = SVD_g_eigen(a, n, p, q);
    a_qq = SVD_g_eigen(a, n, q, q);
    t = (a_pp - a_qq) / (2 *(a_pq));
    t = SVD_sign(t) / (fabs(t) + sqrt(1.0f + t * t));
    c = 1.0f / sqrt(1.0f + t * t);
    s = (c * t);
    for (i = 0; i < n; ++i)
    {
         temp = a[i * n + p];
         a[i * n + p] = s * a[i * n + q] + c * temp;
         a[i * n + q] = c * a[i * n + q] - s * temp;

         temp = u[i * n + p];
         u[i * n + p] = s * u[i * n + q] + c * temp;
         u[i * n + q] = c * u[i * n + q] - s * temp;
    }
}

/*-----------------------------------------------------------------------------
 *  SVD_comparison : 
 *
 *
 *  Parameters: const void * a, const void * b
 *
 *  Return: int16_t
 *----------------------------------------------------------------------------
int16_t SVD_comparison(const void * a, const void * b)
{
        if ((*(singular_value*)a).value > (*(singular_value*)b).value){
                return -1;
        }
        if ((*(singular_value*)a).value == (*(singular_value*)b).value){
                return 0;
        }
        if ((*(singular_value*)a).value <  (*(singular_value*)b).value){
                return 1;
        }
        return 0;
}  
    --- not used at this moment ----------------
*/

/*-----------------------------------------------------------------------------
 *  bubble_sort_singular_value:  Perform Bubble sort on singular_value_t array 
 *
 *  Parameters: singular_value_t* *temp_array3, bubble_sort_action_e order
 *              
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void bubble_sort_singular_value(singular_value_t* temp_array3, bubble_sort_action_e order)
{
    int16_t i = 0;
    int16_t j = 0;
    singular_value_t a;
    int16_t step = sizeof(singular_value_t*);

    switch(order)
    {
        case DESCENDING_REINDEX:                
        for (i = 0; i < sizeof(temp_array3); i+=step)                          //Bubble sort algorithm to sort the numbers in increasing order (value only)
        {
            for (j =0; j < sizeof(temp_array3); j+=step) 
            {
               if (temp_array3[i]->value > temp_array3[j]->value )              //Find the smallest number
               {
                  a.value =  temp_array3[i]->value;                             //Store smallest number
                  temp_array3[i]->value = temp_array3[j]->value;                //Swap smallest number with current number
                  temp_array3[j]->value = a.value;                              //Swap number back to array 
               }
            }
        }
        break;
                
        case ASCENDING_REINDEX:
        for (i = 0; i < sizeof(temp_array3); i+=step)                           //Bubble sort algorithm to sort the numbers in decreasing order (value only)
        {
            for (j =0; j < sizeof(temp_array3); j+=step) 
            {
               if (temp_array3[i]->value < temp_array3[j]->value )              //Find the largest number
               {
                  a.value =  temp_array3[i]->value;;                            //Store largest number
                  temp_array3[i]->value = temp_array3[j]->value;                //Swap largest number with current number
                  temp_array3[j]->value = a.value;                              //Swap number back to array 
               }
            }
        }
        break;

        case DESCENDING_WITHINDEX:                
        for (i = 0; i < sizeof(temp_array3); i+=step)                           //Bubble sort algorithm to sort the numbers in increasing order (copying index and value)
        {
            for (j =0; j < sizeof(temp_array3); j+=step) 
            {
               if (temp_array3[i]->value > temp_array3[j]->value )              //Find the smallest number
               {
                  memcpy((void*)&a,(void*)&temp_array3[i],sizeof(temp_array3[i]));  //Store smallest number
                  memcpy((void*)&temp_array3[i]->value,(void*)&temp_array3[j]->value,sizeof(temp_array3[i]));  //Swap smallest number with current number
                  memcpy((void*)&temp_array3[j],(void*)&a,sizeof(a));           //Swap number back to array                                      
               }
            }
        }
        break;
                
        case ASCENDING_WITHINDEX:
        for (i = 0; i < sizeof(temp_array3); i+=step)                          //Bubble sort algorithm to sort the numbers in decreasing order
        {
            for (j =0; j < sizeof(temp_array3); j+=step) 
            {
               if (temp_array3[i]->value < temp_array3[j]->value )              //Find the largest number
               {
                  memcpy((void*)&a,(void*)&temp_array3[i],sizeof(temp_array3[i]));  //Store largest number
                  memcpy((void*)&temp_array3[i]->value,(void*)&temp_array3[j]->value,sizeof(temp_array3[i]));  //Swap largest number with current number
                  memcpy((void*)&temp_array3[j],(void*)&a,sizeof(a));           //Swap number back to array   
               }
            }
        }
        break;
    }
}

/*-----------------------------------------------------------------------------
 *  singular_value_e : 
 *
 *
 *  Parameters: float64_t * a, int16_t n, singular_value_t* temp
 *
 *  Return: void 
 *----------------------------------------------------------------------------*/
void SVD_singular_value_e(float64_t * a, int16_t n, singular_value_t* temp)
{
    int16_t i;
    int16_t j;
    float64_t sum, sing_val;
    for (j = 0; j < n; ++j)
    {
        sum = 0.0f;
        for (i = 0; i < n; ++i)
        {
            sum += (a[i * n + j] * a[i * n + j]);
        }
        sing_val = sqrt(sum);
        temp[j].value = sing_val;
        temp[j].index = j;
        for (i = 0; i < n; ++i)
        {
            a[i * n + j] /= sing_val;
        }
     }
     /* qsort(temp, n, sizeof(singular_value), comparison); TODO import qsort or fast sort */
     bubble_sort_singular_value(temp, ASCENDING_WITHINDEX);
}

/* 
   may be slow TBD: ref ZLATKO DRMAC AND KRESIMIR VESELIC (Zagreb University)
   NEW FAST AND ACCURATE JACOBI SVD ALGORITHM
*/
/*-----------------------------------------------------------------------------
 *  SVD_jacobi : the singular value decomposition (SVD) of a general dense matrix
 *
 *
 *  Parameters: float64_t* a, int16_t n, float64_t *s, float64_t *u, float64_t *v
 *
 *  Return: void 
 *----------------------------------------------------------------------------*/
void SVD_jacobi(float64_t* a, int16_t n, float64_t *s, float64_t *u, float64_t *v)
{
   int16_t i = 0;
   int16_t j = 0;
   float64_t ep = 1e-16f;
   float64_t comp;
   int16_t count = 1;

   float64_t* T;
   singular_value_t *sigma;
        
   T = Malloc(n*n*sizeof(float64_t));
   SVD_arrange(T, n);
   SVD_arrange(u, n);
   SVD_arrange(v, n);

    while (count != 0)
    {
        count = 0;
        for (i= 0; i< (n-1); ++i)
        {
            for (j=i+1; j<n; ++j)
            {
                comp = ep * (sqrt(SVD_g_eigen(a, n, i, i) * SVD_g_eigen(a, n, j, j))); 
                if (fabs(SVD_g_eigen(a, n, i, j)) > comp)    //apply rotation
                { 
                     SVD_rotation(a, n, i, j, T);
                     count = count + 1;
                }
            }
        }
    }
        
    sigma = Malloc(n * sizeof(singular_value_t));
    SVD_singular_value_e(a, n, sigma);
    for (i = 0; i < n; ++i)
    {
        for (j = 0; j < n; ++j)
        {
              u[i * n + j] = a[i * n + sigma[j].index];
              v[i * n + j] = T[i * n + sigma[j].index];
        }
        s[i] = sigma[i].value; // set s = the singular value
     }        

      Free((char*)T, (n*n*sizeof(float64_t)));
      Free((char*)sigma, (n * sizeof(singular_value_t)));
}

/* 
 * svd.c: Perform a singular value decomposition A = USV' of square matrix.
 *
 * This routine has been adapted with permission from a Pascal implementation
 * (c) 1988 J. C. Nash, "Compact numerical methods for computers", Hilger 1990.
 * The A matrix must be pre-allocated with 2n rows and n columns. On calling
 * the matrix to be decomposed is contained in the first n rows of A. On return
 * the n first rows of A contain the product US and the lower n rows contain V
 * (not V'). The S2 vector returns the square of the singular values.
 *
 * (c) Copyright 1996 by Carl Edward Rasmussen. 
 */

void do_svd(float64_t *A, float64_t *S2, const int16_t n)
{
  int16_t  i, j, k, EstColRank = n, RotCount = n, SweepCount = 0, slimit = (n<120) ? 30 : n/4;
  float64_t eps = 1e-7f, e2 = 10.0f*n*eps*eps, tol = 0.1f*eps, vt, p, x0, y0, q, r, c0, s0, d1, d2;

  for (i=0; i<n; i++) { for (j=0; j<n; j++) A[(n+i)*n + j] = 0.0; A[(n+i)*n + i] = 1.0f; }
  while (RotCount != 0 && SweepCount++ <= slimit) 
  {
    RotCount = EstColRank*(EstColRank-1)/2;
    for (j=0; j<EstColRank-1; j++)
      for (k=j+1; k<EstColRank; k++) 
      {
        p = q = r = 0.0f;
        for (i=0; i<n; i++) 
        {
          x0 = A[i*n + j]; y0 = A[i*n + k];
          p += x0*y0; q += x0*x0; r += y0*y0;
        }
        S2[j] = q; S2[k] = r;
        if (q >= r) 
        {
          if (q<=e2*S2[0] || fabs(p)<=tol*q)
            RotCount--;
          else 
          {
            p /= q; r = 1.0f-r/q; vt = sqrt(4.0f*p*p+r*r);
            c0 = sqrt(0.5f*(1.0f+r/vt)); s0 = p/(vt*c0);
            for (i=0; i<2*n; i++) 
            {
              d1 = A[i*n + j]; d2 = A[i*n + k];
              A[i*n + j] = d1*c0+d2*s0; A[i*n + k] = -d1*s0+d2*c0;
            }
          }
        } 
        else 
        {
          p /= r; q = q/r-1.0f; vt = sqrt(4.0*p*p+q*q);
          s0 = sqrt(0.5f*(1.0f-q/vt));
          if (p<0.0f) s0 = -s0;
          c0 = p/(vt*s0);
          for (i=0; i<2*n; i++) 
          {
            d1 = A[i*n + j]; d2 = A[i*n + k];
            A[i*n + j] = d1*c0+d2*s0; A[i*n + k] = -d1*s0+d2*c0;
          }
        }
      }
    while (EstColRank>2 && S2[EstColRank-1]<=S2[0]*tol+tol*tol) EstColRank--;
  }
//  if (SweepCount > slimit)
//    printf("Warning: Reached maximum number of sweeps (%d) in SVD routine...\n"
//           ,slimit);
}

 /*-----------------------------------------------------------------------------
 *  matrix33_from_doubleStream : set 3x3 matrix from a 9 value float64_t stream
 *                             
 *  Parameters: mat33 *mtx, const float64_t *strm
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool matrix33_from_doubleStream(mat33 *mtx, const float64_t *strm)
{
    if ((mtx==NULL) || ((sizeof(strm)/sizeof(float64_t)) < (sizeof(mtx)/sizeof(float32_t)))) {
        return false;
    }

    mtx->m[0u][0u] = (float32_t)strm[0u];
    mtx->m[0u][1u] = (float32_t)strm[1u];
    mtx->m[0u][2u] = (float32_t)strm[2u];
    mtx->m[1u][0u] = (float32_t)strm[3u];
    mtx->m[1u][1u] = (float32_t)strm[4u];
    mtx->m[1u][2u] = (float32_t)strm[5u];
    mtx->m[2u][0u] = (float32_t)strm[6u];
    mtx->m[2u][1u] = (float32_t)strm[7u];
    mtx->m[2u][2u] = (float32_t)strm[8u];

    return true;
}
 /*-----------------------------------------------------------------------------
 *  doubleStream_from_matrix33 : set a 9 value float64_t stream from a 3x3 matrix
 *                             
 *  Parameters: const mat33 *mtx, float64_t *strm
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool doubleStream_from_matrix33(const mat33 *mtx, float64_t *strm)
{
    if ((mtx==NULL) || ((sizeof(strm)/sizeof(float64_t)) < (sizeof(mtx)/sizeof(float32_t)))) {
        return false;
    }

    strm[0u] = (float64_t)mtx->m[0u][0u];
    strm[1u] = (float64_t)mtx->m[0u][1u];
    strm[2u] = (float64_t)mtx->m[0u][2u];
    strm[3u] = (float64_t)mtx->m[1u][0u];
    strm[4u] = (float64_t)mtx->m[1u][1u];
    strm[5u] = (float64_t)mtx->m[1u][2u];
    strm[6u] = (float64_t)mtx->m[2u][0u];
    strm[7u] = (float64_t)mtx->m[2u][1u];
    strm[8u] = (float64_t)mtx->m[2u][2u];

    return true;
}
 /*-----------------------------------------------------------------------------
 *  Vectr_from_doubleStream : set vector from a 3 value float64_t stream
 *                             
 *  Parameters: Vectr *vec, const float64_t *strm
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Vectr_from_doubleStream(Vectr *vec, const float64_t *strm)
{
    if ((vec==NULL) || (strm==NULL))
    {
        return false;
        if (((sizeof(strm)/sizeof(float64_t)) < (sizeof(vec)/sizeof(float32_t)))) 
        {
           return false;
        } 
    }

    vec->x = (float32_t)strm[0u];
    vec->y = (float32_t)strm[1u];
    vec->z = (float32_t)strm[2u];

    return true;
}
 /*-----------------------------------------------------------------------------
 *  Vectr_to_doubleStream : set vector from a 3 value float64_t stream
 *                             
 *  Parameters: const Vectr *vec, float64_t *strm
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool Vectr_to_doubleStream(const Vectr *vec, float64_t *strm)
{
    if ((vec==NULL) || (strm==NULL))
    {
        return false;
        if (((sizeof(strm)/sizeof(float64_t)) < (sizeof(vec)/sizeof(float32_t)))) 
        {
           return false;
        } 
    }

    strm[0u] = (float64_t) vec->x;
    strm[1u] = (float64_t) vec->y;
    strm[2u] = (float64_t) vec->z;

    return true;
}
/* https://github.com/mdarin/Moore-Penrose-inversion/blob/master/src/main.go
 *
 * Let A be a real m x n matrix.
 * An n x m matrix X is the pseudoinverse of A,
 * if it meets the following four conditions:
 * 1. AXA = A;
 * 2. XAX = X;
 * 3. (AX) T = AX;
 * 4. (XA) T = XA.
 *
 * The only solution that satisfies these four conditions is denoted by A # or A +.
 *
 * If the matrix A is represented as A = USVT, then (here T is the transposition)
 * A # = VSUT,
 * where S # = diag (s # [i])
 * and
 * / 1 / s [i] for s [i]> 0;
 * S # [i] = <
 * \ 0 for s [i] = 0.
 *
 * Thus, the pseudo-inverse can be obtained quite simply using the svd procedure.
 * Wilkinson Reinsch. "Reference book of algorithms in the ALGOL language. Linear algeb
 *
 */
/*-----------------------------------------------------------------------------
 *  do_moore_penrose_inv : perform SVD followed by Moore Penrose
 *                         W are the vector of singular values of a
 *                         U,V are left and right orthogonal transformation matrices
 *
 *  Parameters: mat33 V, const mat33 U
 *
 *  Return: Matrix3f64_t
 *----------------------------------------------------------------------------*/
mat33 do_moore_penrose_inv( const mat33 V, const mat33 U, Vectr W )
{
   float64_t SVDres[3u],inputVec[3],sV[9u],sU[9u];
   mat33 R;

   if ((Vectr_to_doubleStream(&W, &inputVec)==true) && ((doubleStream_from_matrix33(&V, &sV)==true) && (doubleStream_from_matrix33(&U, &sU)==true)))
   {        
      SVD_jacobi(&inputVec, 3, &SVDres, &sU, &sV);
      if ((Vectr_from_doubleStream(&W, &SVDres)==true) && ((matrix33_from_doubleStream(&V, &sV)==true) && (matrix33_from_doubleStream(&U, &sU)==true)))
      {
         R = moore_penrose( V, U, W );
      }
   }
   return R;
}
/*
    ref :- https://github.com/mpaperno/aq_flight_control/blob/next/src/math/algebra.c
    
    These are common functionality ported from AutoQuad by AirCamPro
    
    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.
    Copyright © 2011-2014  Bill Nesbitt
*/
void quatMultiply(float64_t *qr, float64_t *q1, float64_t *q2) 
{
    qr[0] = q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2] - q2[3]*q1[3];
    qr[1] = q2[0]*q1[1] + q2[1]*q1[0] - q2[2]*q1[3] + q2[3]*q1[2];
    qr[2] = q2[0]*q1[2] + q2[1]*q1[3] + q2[2]*q1[0] - q2[3]*q1[1];
    qr[3] = q2[0]*q1[3] - q2[1]*q1[2] + q2[2]*q1[1] + q2[3]*q1[0];
}
void eulerToQuatYPR(float64_t *q, float64_t yaw, float64_t pitch, float64_t roll) 
{
    float64_t cy, cp, cr;
    float64_t sy, sp, sr;

    if (q == NULL) return;
    
    //yaw *= DEG_TO_RAD * 0.5f;
    yaw *= DEGREE_TO_RADIAN(1.0f) * 0.5f;
    //pitch *= DEG_TO_RAD * 0.5f;
    pitch *= DEGREE_TO_RADIAN(1.0f) * 0.5f;
    //roll *= DEG_TO_RAD * 0.5f;
    roll *= DEGREE_TO_RADIAN(1.0f) * 0.5f;

    cy = cos(yaw);
    cp = cos(pitch);
    cr = cos(roll);

    sy = sin(yaw);
    sp = sin(pitch);
    sr = sin(roll);

    q[0] = cy*cp*cr + sy*sp*sr;
    q[1] = cy*cp*sr - sy*sp*cr;
    q[2] = cy*sp*cr + sy*cp*sr;
    q[3] = sy*cp*cr - cy*sp*sr;
}
void eulerToQuatRPY(float64_t *q, float64_t roll, float64_t pitch, float64_t yaw) 
{
    float64_t cy, cp, cr;
    float64_t sy, sp, sr;

    //yaw *= DEG_TO_RAD * 0.5f;
    //pitch *= DEG_TO_RAD * 0.5f;
    //roll *= DEG_TO_RAD * 0.5f;

    if (q == NULL) return;
    
    yaw *= DEGREE_TO_RADIAN(1.0f) * 0.5f;
    pitch *= DEGREE_TO_RADIAN(1.0f) * 0.5f;
    roll *= DEGREE_TO_RADIAN(1.0f) * 0.5f;
    
    cy = cos(yaw);
    cp = cos(pitch);
    cr = cos(roll);

    sy = sin(yaw);
    sp = sin(pitch);
    sr = sin(roll);

    q[0] = cr*cp*cy - sr*sp*sy;
    q[1] = cr*sp*sy + sr*cp*cy;
    q[2] = cr*sp*cy - sr*cp*sy;
    q[3] = cr*cp*sy + sr*sp*cy;
}
void vectorNormalize(float64_t *v, int16_t n) 
{
    float64_t t;
    int16_t i;

    if (v == NULL) return;
    t = 0.0f;
    for (i = 0; i < n; i++)
	t += v[i] * v[i];

    t = sqrt(t);

    if (t > 1e-6f) 
    {
	t = 1.0f / t;
	for (i = 0; i < n; i++)
	    v[i] *= t;
    }
    else 
    {
	for (i = 0; i < n; i++)
	    v[i] *= 0.0f;
    }
}
void nlerp(float64_t *r, const float64_t *a, const float64_t *b, const float64_t t) 
{
    float64_t dp;
    float64_t f1, f2;

    if (( r == NULL ) || (( a == NULL ) || ( b == NULL ))) return;
    
    f1 = 1.0f - t;
    f2 = t;

    dp = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);

    if (dp >= 0.0f)                                                             // rotate the shorter distance
    {
	r[0] = a[0]*f1 + b[0]*f2;
	r[1] = a[1]*f1 + b[1]*f2;
	r[2] = a[2]*f1 + b[2]*f2;
	r[3] = a[3]*f1 + b[3]*f2;
    }
    else 
    {
	r[0] = a[0]*f1 - b[0]*f2;
	r[1] = a[1]*f1 - b[1]*f2;
	r[2] = a[2]*f1 - b[2]*f2;
	r[3] = a[3]*f1 - b[3]*f2;
    }

    vectorNormalize(r, 4u);
}
#ifdef __cplusplus
}
#endif