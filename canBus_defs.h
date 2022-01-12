#ifndef _CAN_BUS_DEFS
#define _CAN_BUS_DEFS
/*
 * canBus_defs.c - CanBus protocol "native" operations.
 *
 * Written Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
 *
 * This file includes both UAVCAN and CanBus implementations
 *
 * We are working on putting in IsoBus also
 */
#include <stdint.h>                                                             // Integer type defines
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define CANBUSPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define CANBUSPACKED __attribute__((packed))                                     /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define CANBUSPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define CANBUSPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define CANBUSPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

/*
// Publication period may vary within these limits.
// It is NOT recommended to change it at run time.
 */
#define UAV_MAX_BROADCASTING_PERIOD_MS 1000u
#define UAV_MIN_BROADCASTING_PERIOD_MS 2u

/*
// If a node fails to publish this message in this amount of time, it should be considered offline.
 */
#define UAV_OFFLINE_TIMEOUT_MS 3000u

/*
// Abstract node health. (uint2_t) 2-bit
 */
#define UAV_HEALTH_OK 0u                                                        // The node is functioning properly.
#define UAV_HEALTH_WARNING 1u                                                   // A critical parameter went out of range or the node encountered a minor failure.
#define UAV_HEALTH_ERROR 2u                                                     // The node encountered a major failure.
#define UAV_HEALTH_CRITICAL 3u                                                  // The node suffered a fatal malfunction.

/*
// Current mode.  (uint3_t) 3-bit
//
// Mode OFFLINE can be actually reported by the node to explicitly inform other network
// participants that the sending node is about to shutdown. In this case other nodes will not
// have to wait OFFLINE_TIMEOUT_MS before they detect that the node is no longer available.
// Reserved values can be used in future revisions of the specification.
 */
#define UAV_MODE_OPERATIONAL 0u                                                 // Node is performing its main functions.
#define UAV_MODE_INITIALIZATION 1u                                              // Node is initializing; this mode is entered immediately after startup.
#define UAV_MODE_MAINTENANCE 2u                                                 // Node is under maintenance.
#define UAV_MODE_SOFTWARE_UPDATE 3u                                             // Node is in the process of updating its software.
#define UAV_MODE_OFFLINE 7u                                                     // Node is no longer available.

#define UAV_OPCODE_SAVE 0u                                                      // Save all parameters to non-volatile storage.
#define UAV_OPCODE_ERASE 1u                                                     // Clear the non-volatile storage; some changes may take effect

// tail byte definitions
#define UAV_DO_TOGGLE(msgNo) ~( msgNo % 2u )                                    // toggle a bit each message
#define UAV_START_TRANS (1u << 7u)                                              // start of transmission
#define UAV_STOP_TRANS (1u << 6u)                                               // end of transmission

#if defined(D_FT900)
typedef struct CANBUSPACKED {
   uint8_t startTrans : 1u;                                                     // set to 0
   uint8_t endTrans : 1u;                                                       // set to 0
   uint8_t toggle : 1u;                                                         // alternate from 1 to 0
   uint8_t transId : 5u;                                                        // from algo
} canBusTailByte_t;                                                             // tail byte type
#else
CANBUSPACKED(
typedef struct {
   uint8_t startTrans : 1u;                                                     // set to 0
   uint8_t endTrans : 1u;                                                       // set to 0
   uint8_t toggle : 1u;                                                         // alternate from 1 to 0
   uint8_t transId : 5u;                                                        // from algo
}) canBusTailByte_t;                                                            // tail byte type
#endif   

#if defined(D_FT900)
typedef struct CANBUSPACKED {
  uint8_t DataBytes[7u];                                                        // first 7 bytes of the message
  uint8_t startTrans : 1u;                                                      // set to 1
  uint8_t endTrans : 1u;                                                        // set to 1
  uint8_t toggle : 1u;                                                          // set to 0
  uint8_t transId : 5u;                                                         // from algo
} canBusSingleFrame_t;                                                         // frame for CanBus single-frame                                                            // tail byte type
#else
CANBUSPACKED(
typedef struct {
  uint8_t DataBytes[7u];                                                        // first 7 bytes of the message
  uint8_t startTrans : 1u;                                                      // set to 1
  uint8_t endTrans : 1u;                                                        // set to 1
  uint8_t toggle : 1u;                                                          // set to 0
  uint8_t transId : 5u;                                                         // from algo
}) canBusSingleFrame_t;                                                         // frame for CanBus single-frame
#endif 

#if defined(D_FT900)
typedef struct CANBUSPACKED {
  uint8_t CRC_LO;                                                               // calc from CRC-16-CCITT-FALSE
  uint8_t CRC_HI;                                                               // calc from CRC-16-CCITT-FALSE
  uint8_t DataBytes[5u];                                                        // first 5 bytes of the message
  uint8_t startTrans : 1u;                                                      // set to 1
  uint8_t endTrans : 1u;                                                        // set to 0
  uint8_t toggle : 1u;                                                          // set to 0
  uint8_t transId : 5u;                                                         // from algo
} canBusMultiFrameStart_t;                                                     // first frame for CanBus multi-frame                                                            // tail byte type
#else
CANBUSPACKED(
typedef struct {
  uint8_t CRC_LO;                                                               // calc from CRC-16-CCITT-FALSE
  uint8_t CRC_HI;                                                               // calc from CRC-16-CCITT-FALSE
  uint8_t DataBytes[5u];                                                        // first 5 bytes of the message
  uint8_t startTrans : 1u;                                                      // set to 1
  uint8_t endTrans : 1u;                                                        // set to 0
  uint8_t toggle : 1u;                                                          // set to 0
  uint8_t transId : 5u;                                                         // from algo
}) canBusMultiFrameStart_t;                                                     // first frame for CanBus multi-frame
#endif 
 
#if defined(D_FT900)
typedef struct CANBUSPACKED {
   uint8_t DataBytes[7u];                                                       // next multiples of 7 bytes
   uint8_t startTrans : 1u;                                                     // set to 0
   uint8_t endTrans : 1u;                                                       // set to 0
   uint8_t toggle : 1u;                                                         // alternate from 1 to 0
   uint8_t transId : 5u;                                                        // from algo
} canBusMultiFrameBody_t;                                                      // next frames for CanBus multi-frame                                                            // tail byte type
#else
CANBUSPACKED(
typedef struct {
   uint8_t DataBytes[7u];                                                       // next multiples of 7 bytes
   uint8_t startTrans : 1u;                                                     // set to 0
   uint8_t endTrans : 1u;                                                       // set to 0
   uint8_t toggle : 1u;                                                         // alternate from 1 to 0
   uint8_t transId : 5u;                                                        // from algo
}) canBusMultiFrameBody_t;                                                      // next frames for CanBus multi-frame
#endif  

#if defined(D_FT900)
typedef struct CANBUSPACKED {
   uint8_t DataBytes[];                                                         // last bytes under 7
   uint8_t startTrans : 1u;                                                     // set to 0
   uint8_t endTrans : 1u;                                                       // set to 1
   uint8_t toggle : 1u;                                                         // alternate from 1 to 0
   uint8_t transId : 5u;                                                        // from algo
} canBusMultiFrameEnd_t;                                                        // last frame for CanBus multi-frame                                                            // tail byte type
#else
CANBUSPACKED(
typedef struct {
   uint8_t DataBytes[];                                                         // last bytes under 7
   uint8_t startTrans : 1u;                                                     // set to 0
   uint8_t endTrans : 1u;                                                       // set to 1
   uint8_t toggle : 1u;                                                         // alternate from 1 to 0
   uint8_t transId : 5u;                                                        // from algo
}) canBusMultiFrameEnd_t;                                                       // last frame for CanBus multi-frame
#endif   

// in mikroE C  CANxSetOperationMode
//  _CAN_MODE_BITS = 0xE00000,                                                  / ? Use this to access opmode  bits ? /
// _CAN_MODE_SLEEP = 0x20                                                       / ? Disable mode. Stops the operation. ? /
// _CAN_MODE_NORMAL = 0x00                                                      / ? Normal mode. Normally, data is sent and received in this mode. ? /
// _CAN_MODE_LOOP = 0x40                                                        / ? Loopback mode. for test. Moves the message from the send buffer to the receive buffer without sending the message to the CAN bus . ? /
// _CAN_MODE_LISTEN = 0x60                                                      / ? Listen Only mode. Receive only. ? /
// _CAN_MODE_CONFIG = 0x80                                                      / ? Configuration mode. Initialize after entering this mode. ? /
//  _CAN_MODE_LISTEN_ALL  = 0x07;
typedef enum {
   AVCAN_NO_ACTION = 0u,
   AVCAN_DATA_FRAME_RCV,                                                        // receive data
   AVCAN_DATA_FRAME_SND,                                                        // send data
   AVCAN_REMOTE_FRAME_SND,                                                      // remote frame request to stop tranmitting data
   AVCAN_REMOTE_FRAME_RCV,                                                      // remote recieved this state we have stopped sending data set to _CAN_MODE_SLEEP = 0x20 / ? Disable mode. Stops the operation. ? /
   AVCAN_ERROR_FRAME_SND,                                                       // when _CAN_RX_INVALID detected
   AVCAN_ERROR_FRAME_RCV,                                                       // send an error frame if you get one
   AVCAN_OVERLOAD_FRAME_SND,                                                    // when you detect _CAN_RX_OVERFLOW
   AVCAN_OVERLOAD_FRAME_RCV,                                                    // slow down your poll transmission rate up to a minimum
   AVCAN_DBLBUFFER_FRAME_RCV,                                                   // when _CAN_RX_DBL_BUFFERED  detected (set by _CAN_CONFIG_DBL_BUFFER_ON unset by _CAN_CONFIG_DBL_BUFFER_OFF)
   AVCAN_DBLBUFFER_FRAME_SND,                                                   // send error frame then remain until all devices error frames ahev been recieved back
   AVCAN_ANONYMS_FRAME_SND,                                                     // no node id
   AVCAN_ANONYMS_FRAME_RCV                                                      // got dynamic node allocation
} canFrameClass;                                                                // defines the types of canbus messages (send state engine states)

/* Arbitrary priority values */
typedef enum {
  AVCAN_PRIORITY_HIGHEST = 0xFFFCu,
  AVCAN_PRIORITY_HIGH    = 0xFFFDu,
  AVCAN_PRIORITY_MEDIUM  = 0xFFFEu,
  AVCAN_PRIORITY_LOW     = 0xFFFFu
//  AVCAN_PRIORITY_LOWEST  = 31u                                                in mikroE C this state is not defined
} canPriorityClass;                                                             // priority defines for can message sending in mikroE

typedef enum {
  UAVCAN_INH_ALL,                                                               // no messages to be sent
  UAVCAN_INH_HIGHEST,                                                           // highest only perhaps error frames
  UAVCAN_INH_HIGH,                                                              // high and highest
  UAVCAN_INH_MEDIUM,                                                            // medium high and highest
  UAVCAN_INH_LOW                                                                // low medium high and highest all frames are to be sent
} canInhibitClass;                                                              // inhibit of various message priorities set in the global gPrioInhibit

typedef enum {
   AVCAN_HEALTH_OK = 0u,                                                        // The node is functioning properly.
   AVCAN_HEALTH_WARNING = 1u,                                                   // A critical parameter went out of range or the node encountered a minor failure.
   AVCAN_UAV_HEALTH_ERROR = 2u,                                                 // The node encountered a major failure.
   AVCAN_UAV_HEALTH_CRITICAL = 3u
} canAbstractNodeHealthClass;                                                   // defines the abstract health classification

typedef enum {
   AVCAN_MODE_OPERATIONAL = 0u,                                                 // Node is performing its main functions.
   AVCAN_MODE_INITIALIZATION = 1u,                                              // Node is initializing; this mode is entered immediately after startup.
   AVCAN_MODE_MAINTENANCE = 2u,                                                 // Node is under maintenance.
   AVCAN_MODE_SOFTWARE_UPDATE = 3u,                                             // Node is in the process of updating its software.
   AVCAN_MODE_OFFLINE = 7u
} canCurrentModeClass;                                                          // defines the current mode class

typedef union {
  uint8_t Uint8;                                                                // 8 bit unsigned
  int8_t Int8;                                                                  // 8 bit signed
  uint16_t Uint16;                                                              // 16 bit unsigned
  int16_t Int16;                                                                // 16 bit signed
  uint32_t Uint32;                                                              // 32 bit unsigned
  int32_t Int32;                                                                // 32 bit signed
  float32_t Float32;                                                            // 32 bit IEEE float
  unsigned char TextSelection[32u];                                             // text selection string separated by ;
} uavcanVal;

/* polled data message out
// Any UAVCAN node is required to publish this message periodically. */

#if defined(D_FT900)
#if defined(_CPU_BIG_ENDIAN)
typedef struct CANBUSPACKED {
   uint8_t node_id;                                                             // node id
   uint16_t data_typ_id;                                                        // data type
   uint8_t priority;                                                            // priority
} uavcanCanId_t;
#else
typedef struct CANBUSPACKED {
   uint8_t priority;                                                            // priority
   uint16_t data_typ_id;                                                        // data type
   uint8_t node_id;                                                             // node id
} uavcanCanId_t;
#endif                                                                                                                   
#else
#if defined(_CPU_BIG_ENDIAN)
CANBUSPACKED(
typedef struct {
   uint8_t node_id;                                                             // node id
   uint16_t data_typ_id;                                                        // data type
   uint8_t priority;                                                            // priority
}) uavcanCanId_t;                                                               // uavcan.protocol.CanId
#else
CANBUSPACKED(
typedef struct {
   uint8_t priority;                                                            // priority
   uint16_t data_typ_id;                                                        // data type
   uint8_t node_id;                                                             // node id
}) uavcanCanId_t;                                                               // uavcan.protocol.CanId
#endif
#endif  

#if defined(D_FT900)
typedef struct CANBUSPACKED {
   uavcanCanId_t canId;                                                         // can id
   uint8_t payload[];                                                           // payload
} uavcanBroadcast_t;                                                           // uavcan.protocol.Broadcast                                                            // tail byte type
#else
CANBUSPACKED(
typedef struct {
   uavcanCanId_t canId;                                                         // can id
   uint8_t payload[];                                                           // payload
}) uavcanBroadcast_t;                                                           // uavcan.protocol.Broadcast
#endif 

/* polled data message out
// Any UAVCAN node is required to publish this message periodically. */
#if defined(D_FT900)
typedef struct CANBUSPACKED {
   uint16_t MaxBroadPeriod;                                                     // set to MAX_BROADCASTING_PERIOD_MS
   uint16_t MinBroadPeriod;                                                     // set to MIN_BROADCASTING_PERIOD_MS
   uint16_t OfflineTimeOut;                                                     // set to OFFLINE_TIMEOUT_MS if no reply put device out poll table
   uint32_t uptime_sec;                                                         // should never overflow nodes shall detect that a remote node has restarted when this value goes backwards.
   canAbstractNodeHealthClass abstract_node_health : 2u;                        // abstrakt node health
   canCurrentModeClass current_mode : 3u;                                       // current mode class
   uint8_t submode : 3u;                                                        // currently all zero (future)
   uint16_t vendor_specific_status_code;                                        // vendor code optional
} uavcanNodeStatus_t;                                                          // uavcan.protocol.NodeStatus Default data type ID: 341                                                           // tail byte type
#else
CANBUSPACKED(
typedef struct {
   uint16_t MaxBroadPeriod;                                                     // set to MAX_BROADCASTING_PERIOD_MS
   uint16_t MinBroadPeriod;                                                     // set to MIN_BROADCASTING_PERIOD_MS
   uint16_t OfflineTimeOut;                                                     // set to OFFLINE_TIMEOUT_MS if no reply put device out poll table
   uint32_t uptime_sec;                                                         // should never overflow nodes shall detect that a remote node has restarted when this value goes backwards.
   canAbstractNodeHealthClass abstract_node_health : 2u;                        // abstrakt node health
   canCurrentModeClass current_mode : 3u;                                       // current mode class
   uint8_t submode : 3u;                                                        // currently all zero (future)
   uint16_t vendor_specific_status_code;                                        // vendor code optional
}) uavcanNodeStatus_t;                                                          // uavcan.protocol.NodeStatus Default data type ID: 341
#endif 

// Node discovery
// should be supported

// Time synchronization
// think we might need it

/* uavcan.protocol.param.GetSet - gets or sets a single configuration parameter value,
// either by name or by index
// Get or set a parameter by name or by index.  Default data type ID: 11 */
#if defined(D_FT900)
typedef struct CANBUSPACKED {
   uint16_t index : 13u;                                                        // 13-bit number
   uint16_t spare : 3u;
   uavcanVal setValue;                                                          // If set - parameter will be assigned this value, then the new value will be returned. If not set - current parameter value will be returned.
   uint8_t paramName;                                                           // numerical name [<=92]
   uavcanVal actValue;                                                          // some slaves return this rather this rather than setValue
   uint8_t name2;                                                               // [<=92] Empty name (and/or empty value) in response indicates that there is no such parameter.
} uavcanGetSet_t;                                                               // uavcan get set parameter message                                                           // tail byte type
#else
CANBUSPACKED(
typedef struct {
   uint16_t index : 13u;                                                        // 13-bit number
   uint16_t spare : 3u;
   uavcanVal setValue;                                                          // If set - parameter will be assigned this value, then the new value will be returned. If not set - current parameter value will be returned.
   uint8_t paramName;                                                           // numerical name [<=92]
   uavcanVal actValue;                                                          // some slaves return this rather this rather than setValue
   uint8_t name2;                                                               // [<=92] Empty name (and/or empty value) in response indicates that there is no such parameter.
}) uavcanGetSet_t;                                                              // uavcan get set parameter message
#endif 

/* uavcan.protocol.param.ExecuteOpcode - allows control of the node configuration, including saving the configuration
   into the non-volatile memory, or resetting the configuration to default settings. */
#if defined(D_FT900)
typedef struct CANBUSPACKED {
   uint8_t opcode;                                                              // save or erase
   int32_t argument1;                                                           // blank at present
   int16_t argument2;                                                           // blank at present
   int32_t errorcode1;                                                          // error codes (blank if status was ok=true)
   int16_t errorcode2;
   uint8_t status : 1u;                                                         // True if the operation has been performed successfully, false otherwise.
   uint8_t spare1 : 7u;
} uavcanExeOpCode_t;                                                            // uavcan execute op code message                                                           // tail byte type
#else
CANBUSPACKED(
typedef struct {
   uint8_t opcode;                                                              // save or erase
   int32_t argument1;                                                           // blank at present
   int16_t argument2;                                                           // blank at present
   int32_t errorcode1;                                                          // error codes (blank if status was ok=true)
   int16_t errorcode2;
   uint8_t status : 1u;                                                         // True if the operation has been performed successfully, false otherwise.
   uint8_t spare1 : 7u;
}) uavcanExeOpCode_t;                                                           // uavcan execute op code message
#endif 

/* ------------------ The fields common to all CCP messages --------------------
CAN Calibration Protocol (CCP), in which each CAN message sent to a CCP client shares
two common fields, each of one byte. Up to 6 additional bytes may follow, 
the interpretation of which depends on the message type stored in the first byte
------------------------------------------------------------------------------*/
#define CCP_PERM_DISCONNECT 1U                                                  // check this in the spec !!!

typedef enum ccp_types_e {
   Ccp_connect,
   Ccp_disconnect,
} ccp_types_e;                                                                  /* msg types */

#if defined(D_FT900)
typedef struct CANBUSPACKED {
 ccp_types_e msg_type;
 uint8_t sequence_no;
} ccp_common_t;                                                                /* CCP connect message */                                                           // tail byte type
#else
CANBUSPACKED(
typedef struct {
 ccp_types_e msg_type;
 uint8_t sequence_no;
}) ccp_common_t;                                                                /* CCP connect message */
#endif  

#if defined(D_FT900)
typedef struct CANBUSPACKED {
 ccp_common_t common_part;
 uint16_t station_to_connect; 
} ccp_connect_t;                                                                /* CCP connect message */                                                           // tail byte type
#else
CANBUSPACKED(
typedef struct {
 ccp_common_t common_part;
 uint16_t station_to_connect; 
}) ccp_connect_t;                                                               /* CCP connect message */
#endif   

#if defined(D_FT900)
typedef struct CANBUSPACKED {
 ccp_common_t common_part;    
 uint8_t disconnect_command;
 uint8_t pad;
 uint16_t station_to_disconnect; 
} ccp_disconnect_t;                                                             /* CCP disconnect message */                                                           // tail byte type
#else
CANBUSPACKED(
typedef struct {
 ccp_common_t common_part;    
 uint8_t disconnect_command;
 uint8_t pad;
 uint16_t station_to_disconnect;
}) ccp_disconnect_t;                                                            /* CCP disconnect message */
#endif 

typedef union {
 ccp_common_t common;
 ccp_connect_t connect;
 ccp_disconnect_t disconnect;
} ccp_message_t;


//extern volatile canInhibitClass gPrioInhibit;                                 it is this but the find declaration is then locked
extern volatile uint8_t gPrioInhibit;                                           // globally set priority level for avcan sending

extern uint64_t readUptime();
extern void Init_CAN_Port(unsigned char *Can_Init_Flags, unsigned char *Can_Send_Flags, unsigned char *Can_Rcv_Flags, char *FIFObuffers);
extern uint16_t make_float16(float32_t value);
extern int16_t publish_true_airspeed(float32_t mean, float32_t variance,uint8_t uavcan_node_id, uint8_t transfer_id);
extern int16_t publish_node_status(canAbstractNodeHealthClass health, canCurrentModeClass mode, uint16_t vendor_specific_status_code,uint8_t uavcan_node_id, uint8_t transfer_id);
extern int16_t uavcan_broadcast(canPriorityClass priority, uint16_t data_type_id, uint8_t transfer_id, const uint8_t* payload, uint16_t payload_len, uint8_t uavcan_node_id);
extern int8_t computeForwardDistance(uint8_t a, uint8_t b);
extern void process_ccp_message(ccp_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif