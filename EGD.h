#ifndef __EGD_h
#define __EGD_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Custom library for EGD protocol
// Talks to GEC Fanuc over UDP
//
// GE Fanuc Automation and GE Drive Systems developed an Ethernet Global Data, or EGD,
// exchange for PLC and computer data in 1998. EGD uses UDP or datagram messages for fast transfer
// of up to 1400 bytes of data from a producer to one or more consumers. UDP messages have much less overhead
// that the streaming TCP connectigefcommqon used for programming or CommReq's over SRTP Ethernet.
// Like Genius® broadcast input or directed control messages, UDP messages are not acknowledged.
// They can be sent at short intervals. Chances of one or more messages being dropped are small
// on a local area network.
//  SMPTE interface library.
//  smpte.h - smpte "native" operations.
//
//  Written by (C) 2020 A C P Avaiation Walkerburn Scotland

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define EGDPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define EGDPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define EGDPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define EGDPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define EGDPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define GEF_EGD_UDP_DATA_PORT 0x4746UL                                          // Letters GF are used as port for EGD Data messages */
#define GEF_EGD_PDU_TYP_VER 0x010DU                                             // PDU type version for header structure
#define GEF_EGD_PDU_MAX_SZ 1400U                                                // PDU Max size
#define GEF_EGD_UDP_MAX_LEN 1450U                                               // UDP Packet max size

#define GEF_DEV_CPU_PLC 1U                                                      // 1=PLC, 
#define GEF_DEV_CPU_GIO 2U                                                      // 2=GIO, 
#define GEF_DEV_CPU_EIO 3U                                                      // 3=EIO

// Field        Description
// PDUTypeVersion        has a 13 in the low byte and the current version of 1 in the high byte.
// RequestID        16-bit number for each ExchangedID incremented by producer
// ProducerID        TCP/IP addresses of the sender based on EGD config, not actual address
// ExchangeID        Unique number for each Producer used to identify data, from EGD config
// TimeStamp        Two 32-bit numbers with seconds and nanoseconds since 1-Jan-1970
// Status        Message status, 1=Success, others in Table 4-3 in GFK-1541A manual
// ConfigSignature        For security use, Not implemented yet and must be set to 0

#if defined(D_FT900)
typedef struct EGDPACKED {
        uint16_t PDUTypeVersion;                                                /* Type=13 (0Dh) in low byte, version=1 in hi */
        uint16_t RequestID;                                                     /* incremented every time data produced */
        uint32_t ProducerID;                                                    /* The TCP/IP address of device sending EGD */
        uint32_t ExchangeID;                                                    /* A unique Producer number identifying the data */
        uint32_t TimeStampSec;                                                  /* Timestamp seconds since 1-Jan-1970 */
        uint32_t TimeStampNanoSec;                                              /* and number of nanoseconds in current second */
        uint32_t Status;                                                        /* In low word, upper word reserved and set to 0 */
        uint32_t ConfigSignature;                                               /* In low word, upper word reserved and set to 0*/
        uint32_t Reserved;                                                      /* word set to 0 */
        unsigned char ProductionData[GEF_EGD_PDU_MAX_SZ];                       /* PLC or I/O data to be sent as EGD */
} GEF_EGD_DATA_t;
#else
EGDPACKED(
typedef  struct {
        uint16_t PDUTypeVersion;                                                /* Type=13 (0Dh) in low byte, version=1 in hi */
        uint16_t RequestID;                                                     /* incremented every time data produced */
        uint32_t ProducerID;                                                    /* The TCP/IP address of device sending EGD */
        uint32_t ExchangeID;                                                    /* A unique Producer number identifying the data */
        uint32_t TimeStampSec;                                                  /* Timestamp seconds since 1-Jan-1970 */
        uint32_t TimeStampNanoSec;                                              /* and number of nanoseconds in current second */
        uint32_t Status;                                                        /* In low word, upper word reserved and set to 0 */
        uint32_t ConfigSignature;                                               /* In low word, upper word reserved and set to 0*/
        uint32_t Reserved;                                                      /* word set to 0 */
        unsigned char ProductionData[GEF_EGD_PDU_MAX_SZ];                       /* PLC or I/O data to be sent as EGD */
}) GEF_EGD_DATA_t;
#endif

#if defined(D_FT900)
typedef struct EGDPACKED {
   uint8_t h;
   uint8_t s;
   uint8_t l;
} GEF_EGD_EXCHANGE_STATUS_t;
#else
EGDPACKED(
typedef struct {
        uint32_t  ProducerTCPIP;                                                // return TCP/IP address for this  exchange
        uint32_t  ExchangeID;                                                   // return unique ID for this Producer
        uint32_t  ExchangeCount;                                                // sent or received, can 0 if fWrite is TRUE
        uint32_t  ErrorCount;                                                   // Timeouts for consumer, other producer errors
        uint32_t  TimeStampSec;                                                 // EGD time stamp for last message received
        uint32_t  TimeStampNanoSec;
        uint32_t  TimeTillTransferEGD;                                          // MilliSec remaining, minus if overdue;
        uint32_t  RequestID;                                                    // Producer increments when EGD message sent
        uint16_t  EnableExchange;                                               // set to 1 to enable, 0 to disable if fWrite
}) GEF_EGD_EXCHANGE_STATUS_t;
#endif

#if defined(D_FT900)
typedef struct EGDPACKED {
        uint16_t SNPMemoryType;                                                 // SNP Memory Type, %AI=10, %I=16, etc
        uint16_t StartAddress;                                                  // Addresses start at 1
        uint16_t DataLength;                                                    // In words or bits if SNPMemoryType > %AQ
} GEF_PLC_MEMORY_LIST_t;
#else
EGDPACKED(
typedef struct {
        uint16_t SNPMemoryType;                                                 // SNP Memory Type, %AI=10, %I=16, etc
        uint16_t StartAddress;                                                  // Addresses start at 1
        uint16_t DataLength;                                                    // In words or bits if SNPMemoryType > %AQ
}) GEF_PLC_MEMORY_LIST_t;
#endif

#if defined(D_FT900)
typedef struct EGDPACKED {
        uint32_t ProducerTCPIP;                                                 // If equal to computer TCP/IP, it is producer
        uint32_t ExchangeID;                                                    // Unique number for each producer from 1 to N
        uint16_t DeviceType;                                                    // 0=CPU, 1=PLC, 2=GIO, 3=EIO, etc
        uint16_t DeviceNumber;                                                  // 1 to 9999
        uint32_t ConsumerTCPIP;                                                 // Set to 0 if this computer is consuming
        uint32_t ProducerPeriod;                                                // 10 to 3600000 (1 hour) milliseconds
        uint32_t ConsumerTimeout;                                               // 10 to 3600000 millisecfor timeout, 0=none
        uint16_t DataByteLength;                                                // configured transfer length, EGD limit 1400
        uint16_t MemoryListCount;                                               // same as AddressSegment count
        uint32_t PLCStatusTypeAddress;                                          // SNP Type in upper, address in lower
        uint16_t DataBytesReceived;                                             // Set when first EGD message received
} GEF_EGD_EXCHANGE_CONFIG_t;
#else
EGDPACKED(
typedef struct {
        uint32_t ProducerTCPIP;                                                 // If equal to computer TCP/IP, it is producer
        uint32_t ExchangeID;                                                    // Unique number for each producer from 1 to N
        uint16_t DeviceType;                                                    // 0=CPU, 1=PLC, 2=GIO, 3=EIO, etc
        uint16_t DeviceNumber;                                                  // 1 to 9999
        uint32_t ConsumerTCPIP;                                                 // Set to 0 if this computer is consuming
        uint32_t ProducerPeriod;                                                // 10 to 3600000 (1 hour) milliseconds
        uint32_t ConsumerTimeout;                                               // 10 to 3600000 millisecfor timeout, 0=none
        uint16_t DataByteLength;                                                // configured transfer length, EGD limit 1400
        uint16_t MemoryListCount;                                               // same as AddressSegment count
        uint32_t PLCStatusTypeAddress;                                          // SNP Type in upper, address in lower
        uint16_t DataBytesReceived;                                             // Set when first EGD message received
}) GEF_EGD_EXCHANGE_CONFIG_t;
#endif



//  A sample GEFComm.ini exchanging 200 registers between computer and PLC every 100 milliseconds is:
// --------------------------------------------------------------------------------------------------
//  [CPU1]                            ; First computer running HostEGD
//  TCPIP = mycomputername            ; computer name or TCP/IP address
//  Produce1 = 100,R201(200)        ; 100 MS period, no status, send from R201
//  Consume1PLC1 = 300                    ; 300 MS timeout, receive R1(200) set by PLC
//  [PLC1]                            ; PLC configured for EGD
//  TCPIP = 3.1.1.4                    ; Address of PLC, can also use PLC DNS name
//  Produce1 = 100,R400,R1(200)     ; 100 MS period, Status=R400, send 200 R'2
//  Consume1CPU1 = 300,R401            ; 300 MS timeout,status=R401,receive R201(200)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif