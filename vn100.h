#ifndef __vn100_
#define __vn100_
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define VN100PACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define VN100PACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define VN100PACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define VN100PACKED
#else                                                                           /* for MPLAB PIC32 */
  #define VN100PACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define VECTNAV_VN100_STRT 0xFAu                                                /* message sync byte */
#define VECTNAV_VN100_DATLEN 41u                                                /* data length of message */
#define VECTNAV_VN100_DATSIZ 4u                                                 /* size in bytes for data values */
#define VECTNAV_VN100_TIMEOUT 20000u                                            /* message time out */
#define VECTNAV_VN100_POLLTIME 1000u                                            /* collect again no handshake timout */

#define VECTNAV_VN100_BAUD 115200u                                              /* baud rate */

typedef enum {
   VN100_NO_DATA_READ = 0u,                                                     // Node is performing its main functions.
   VN100_PACKET_READ_SYNC = 1u,                                                 // read a data packet sync byte
   VN100_PACKET_READ_DATA = 2u,                                                 // read a data packet
   VN100_PACKET_READ_COMPLETE = 3u,                                             // message is been processed
   VN100_PACKET_PROCESS_COMPLETE = 4u,                                          // message has been processed
   VN100_DATA_CORRUPT = 5u                                                      // corrupt message has been collected
} vn100UartRcvStates;                                                           // UART receive states

// Union functions for byte to float conversions
// IMU sends data as bytes, the union functions are used to convert
// these data into other data types

// Attitude data
typedef union {float32_t f; uint8_t b[VECTNAV_VN100_DATSIZ];} vn_yaw_t;
typedef union {float32_t f; uint8_t b[VECTNAV_VN100_DATSIZ];} vn_pitch_t;
typedef union {float32_t f; uint8_t b[VECTNAV_VN100_DATSIZ];} vn_roll_t;

// Angular rates
typedef union {float32_t f; uint8_t b[VECTNAV_VN100_DATSIZ];} vn_W_x_t;
typedef union {float32_t f; uint8_t b[VECTNAV_VN100_DATSIZ];} vn_W_y_t;
typedef union {float32_t f; uint8_t b[VECTNAV_VN100_DATSIZ];} vn_W_z_t;

// Acceleration
typedef union {float32_t f; uint8_t b[VECTNAV_VN100_DATSIZ];} vn_a_x_t;
typedef union {float32_t f; uint8_t b[VECTNAV_VN100_DATSIZ];} vn_a_y_t;
typedef union {float32_t f; uint8_t b[VECTNAV_VN100_DATSIZ];} vn_a_z_t;

// Checksum
typedef union {uint16_t s; uint8_t b[2u];} vn_checksum_t;

#if defined(D_FT900)
typedef struct VN100PACKED {
   uint8_t imu_sync_detected : 1u;                                              // check if the sync byte (0xFA) is detected
   uint8_t rcvState : 3u;
   uint8_t spare : 4u;
   uint8_t in[100u];                                                            // array to save data send from the IMU
   uint16_t bytesRead;                                                          // byte counter on read buffer
   uint16_t msgFail;                                                            // bad message counter
   uint16_t collectCnt;                                                         // collection counter
   vn_yaw_t yaw;
   vn_pitch_t pitch;
   vn_roll_t roll;
   vn_W_x_t W_x;
   vn_W_y_t W_y;
   vn_W_z_t W_z;
   vn_a_x_t a_x;
   vn_a_y_t a_y;
   vn_a_z_t a_z;
   vn_checksum_t checksum;
} VN100_VectorNavRcv_t;
#else
VN100PACKED(
typedef struct {
   uint8_t imu_sync_detected : 1u;                                              // check if the sync byte (0xFA) is detected
   uint8_t rcvState : 3u;
   uint8_t spare : 4u;
   uint8_t in[100u];                                                            // array to save data send from the IMU
   uint16_t bytesRead;                                                          // byte counter on read buffer
   uint16_t msgFail;                                                            // bad message counter
   uint16_t collectCnt;                                                         // collection counter
   vn_yaw_t yaw;
   vn_pitch_t pitch;
   vn_roll_t roll;
   vn_W_x_t W_x;
   vn_W_y_t W_y;
   vn_W_z_t W_z;
   vn_a_x_t a_x;
   vn_a_y_t a_y;
   vn_a_z_t a_z;
   vn_checksum_t checksum;
}) VN100_VectorNavRcv_t;
#endif

extern void vn100_init_uart( uint8_t uartNo );
extern uint16_t calculate_imu_crc(uint8_t dataV[], uint16_t length);
extern uint8_t vn100RecvPacketNoBlock(VN100_VectorNavRcv_t* Response, uint8_t uART_no);
extern void check_sync_byte(VN100_VectorNavRcv_t *msgIn, uint8_t uART_no);
extern void read_imu_data(VN100_VectorNavRcv_t *msgIn, uint8_t uART_no);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif