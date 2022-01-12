#ifndef  __jrt__
#define  __jrt__
//    jrt_lidar.h : JRT Lidar (distance sensor / parking sensor) protocol stack
//    An API container and command library for the JRT Lidar IT03M
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
// #include "defintions.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define JRTPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define JRTPACKED __attribute__((packed))                                     /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define JRTPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define JRTPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define JRTPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

// define the start commands and message id for the protocol
#define JRT_READ_MODULE_STATUS_STX 0xFAU                                        // Byte means read the module status (module status write)
#define JRT_READ_MODULE_STATUS_MSG 0x04U                                        // 2nd byte READ MODULE STATUS

#define JRT_WRITE_MODULE_STATUS_STX 0xFBU                                       // Byte means read the module status (module status read)
#define JRT_WRITE_MODULE_STATUS_MSG 0x05U                                       // 2nd byte READ MODULE STATUS

#define JRT_WRITE_START_MEASURE_STX 0xFAU                                       // Byte means read the module status (start/stop measure request)
#define JRT_WRITE_START_MEASURE_MSG 0x01U                                       // 2nd byte START MEASURE
#define JRT_WRITE_START_MEASURE_CMD 0x01U                                       // 1st byte of payload START MEASURE
#define JRT_WRITE_STOP_MEASURE_CMD 0x00U                                        // 1st byte of payload START MEASURE
#define JRT_READ_STARTSTOP_PAYLOAD_LEN 0x04U                                    // length of payload for START MEASURE
#define JRT_STARTSTOP_LEN 8U                                                    // length of the start / stop message

#define JRT_READ_STARTSTOP_MEASURE_STX 0xFBU                                    // This is the continously polled readings from the instrument (reply to start/stop request)
#define JRT_READ_STARTSTOP_MEASURE_MSG 0x02U                                    // 2nd byte START MEASURE
#define JRT_MEAS_LEN 16U                                                        // the measure message is 16 bytes long

#define JRT_READ_DATA_MEASURE_STX 0xFBU                                         // Byte means read the module status (polled reply data once started above)
#define JRT_READ_DATA_MEASURE_MSG 0x03U                                         // 2nd byte START MEASURE
#define JRT_READ_DATA_MEAS_PAY_LEN 0x0CU

// values for data_valid_ind of JRT_read_measure_t structure
#define JRT_READ_DATA_VALID_IND_OK 0U                                           // ok
#define JRT_READ_DATA_VALID_IND_OOR 1U                                          // out range
#define JRT_READ_DATA_VALID_IND_WEAK1_S1 2U                                     // weak signal (scene 1)
#define JRT_READ_DATA_VALID_IND_WEAK1_S2 3U                                     // weak signal (scene 2)
#define JRT_READ_DATA_VALID_IND_WEAK1_S3 4U                                     // weak signal (scene 3)
#define JRT_READ_DATA_VALID_IND_LO_TEMP 5U                                      // low temp
#define JRT_READ_DATA_VALID_IND_HI_TEMP 6U                                      // high temp
#define JRT_READ_DATA_VALID_IND_WEAK4 7U                                        // weak signal (scene 4)

// values for the BrdStatus of the JRT_read_module_status_read_t structure
#define JRT_READ_MODULE_ERROR1 1U                                               // input power low
#define JRT_READ_MODULE_ERROR2 2U                                               // internal error (reboot) cycle power of unit exernally to do so
#define JRT_READ_MODULE_ERROR3 3U                                               // temperature too high
#define JRT_READ_MODULE_ERROR4 4U                                               // temperature too low
#define JRT_READ_MODULE_ERROR5 5U                                               // power too high
#define JRT_READ_MODULE_ERROR6 6U                                               // internal error (reboot) cycle power of unit exernally to do so
#define JRT_READ_MODULE_ERROR7 7U                                               // internal error (reboot) cycle power of unit exernally to do so
#define JRT_READ_MODULE_ERROR8 8U                                               // internal error (reboot) cycle power of unit exernally to do so
#define JRT_READ_MODULE_ERROR9 9U                                               // internal error (reboot) cycle power of unit exernally to do so
#define JRT_READ_MODULE_ERROR10 10U                                             // internal error (reboot) cycle power of unit exernally to do so

#define JRT_DEFAULT_BAUD 115200UL                                               // Default serial baud rate
#define JRT_MAX_ERR_RESET 5u                                                    // how many consecutive bad messages we had before resetting usb port

// READ MODULE STATUS
#if defined(D_FT900)
typedef struct JRTPACKED {
        uint8_t MsgType;
        uint8_t MsgCode;
        uint8_t TransId;
        uint8_t PayLoadLen;
} JRT_read_module_status_write_t;
#else
JRTPACKED(
typedef struct {
        uint8_t MsgType;
        uint8_t MsgCode;
        uint8_t TransId;
        uint8_t PayLoadLen;
}) JRT_read_module_status_write_t;
#endif

// Read reply to READ MODULE STATUS
#if defined(D_FT900)                                                            
typedef struct JRTPACKED {
        uint8_t MsgType;
        uint8_t MsgCode;
        uint8_t TransId;
        uint8_t PayLoadLen;
        uint16_t Temp;
        uint16_t VIN;
        uint16_t Freq;
        uint16_t BrdStatus;
} JRT_read_module_status_read_t;
#else
JRTPACKED(
typedef struct {
        uint8_t MsgType;
        uint8_t MsgCode;
        uint8_t TransId;
        uint8_t PayLoadLen;
        uint16_t Temp;
        uint16_t VIN;
        uint16_t Freq;
        uint16_t BrdStatus;
}) JRT_read_module_status_read_t;
#endif

// Measure request
#if defined(D_FT900)                                                            
typedef struct JRTPACKED {
        uint8_t MsgType;
        uint8_t MsgCode;
        uint8_t TransId;
        uint8_t PayLoadLen;
        uint8_t Payload[4u];
} JRT_write_measure_t;
#else
JRTPACKED(
typedef struct {
        uint8_t MsgType;
        uint8_t MsgCode;
        uint8_t TransId;
        uint8_t PayLoadLen;
        uint8_t Payload[4u];
}) JRT_write_measure_t;                                                         // Write container or mesaure instruction request (start / stop) change payload  - reply is same struct
#endif

// Measure data reply once started with measure request
#if defined(D_FT900)                                                            
typedef struct JRTPACKED {
        uint8_t MsgType;
        uint8_t MsgCode;
        uint8_t TransId;
        uint8_t PayLoadLen;
        uint16_t data_valid_ind;
        uint16_t filter_dist;
        uint16_t current_dist;
        uint16_t temp;
        uint32_t SigRef;
} JRT_read_measure_t;
#else
JRTPACKED(
typedef struct {
        uint8_t MsgType;
        uint8_t MsgCode;
        uint8_t TransId;
        uint8_t PayLoadLen;
        uint16_t data_valid_ind;
        uint16_t filter_dist;
        uint16_t current_dist;
        uint16_t temp;
        uint32_t SigRef;
}) JRT_read_measure_t;                                                          // Read container for mesaure instruction (periodic reply)
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif                                                                          // end __jrt___ header