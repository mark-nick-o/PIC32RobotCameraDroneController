//    Library for  ASN E1 31 : Transport of DMX512 over UDP
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#ifndef __ASN_LIB_INCLUDE
#define __ASN_LIB_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define ASNPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define ASNPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define ASNPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define ASNPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define ASNPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define  ACN_SDT_MULTICAST_PORT 5568u                                           /* UDP port for communication  */

#define ASN_PRE_AMBLE 0x0010u
#define ASN_PACKET_ID { 0x41u, 0x53u, 0x43u, 0x2du, 0x45u, 0x31u, 0x2eu, 0x31u, 0x37u, 0x00u, 0x00u, 0x00u }

#define VECTOR_ROOT_E131_DATA 0x00000004LU  
#define VECTOR_ROOT_E131_EXTENDED 0x00000008LU     
#define VECTOR_DMP_SET_PROPERTY 0x02u 
#define VECTOR_E131_DATA_PACKET 0x00000002LU
#define VECTOR_E131_EXTENDED_SYNCHRONIZATION 0x00000001LU
#define VECTOR_E131_EXTENDED_DISCOVERY 0x00000002LU
#define VECTOR_UNIVERSE_DISCOVERY_UNIVERSE_LIST 0x00000001LU

#define E131_E131_UNIVERSE_DISCOVERY_INTERVAL 10u                               /* seconds */
#define E131_NETWORK_DATA_LOSS_TIMEOUT 2.5f                                     /* seconds */
#define E131_DISCOVERY_UNIVERSE 64214L

// ========================== Data Packet ======================================// E1.31 Data Packet defines an outer layer PDU wrapper that specifies the sequence 
                                                                                // number of a packet and that carries a block of data. This data block contains
                                                                                // a nested PDU containing a single message of the Device Management Protocol of ANSI E1.17 [ACN] 
                                                                                // to carry DMX512-A [DMX] data. Each PDU contains a length field which equals the length of the entire

#if defined(D_FT900)
typedef struct ASNPACKED {
   uint16_t preAmble;                                                           // Preamble Size Define RLP Preamble Size 0x0010
   uint16_t postAmble;                                                          // Size RLP Post-amble Size. 0x0000
   uint8_t packetId[12u];                                                       // ACN Packet Identifier Identifies this packet as E1.17 0x41 0x53 0x43 0x2d 0x45 0x31 0x2e 0x31 0x37 0x00 0x00 0x00
   uint16_t FlagsLength;                                                        // Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
   uint32_t Vector;                                                             // Identifies RLP Data as 1.31 Protocol PDU VECTOR_ROOT_E131_DATA
   uint8_t CIDSender[16u];                                                      // CID Sender's unique ID
} ASN_RootLayer_t;
#else
ASNPACKED(
typedef struct {
   uint16_t preAmble;                                                           // Preamble Size Define RLP Preamble Size 0x0010
   uint16_t postAmble;                                                          // Size RLP Post-amble Size. 0x0000
   uint8_t packetId[12u];                                                       // ACN Packet Identifier Identifies this packet as E1.17 0x41 0x53 0x43 0x2d 0x45 0x31 0x2e 0x31 0x37 0x00 0x00 0x00
   uint16_t FlagsLength;                                                        // Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
   uint32_t Vector;                                                             // Identifies RLP Data as 1.31 Protocol PDU VECTOR_ROOT_E131_DATA
   uint8_t CIDSender[16u];                                                      // CID Sender's unique ID
}) ASN_RootLayer_t;
#endif
                                                                                // PDU, including its header and data block information
#if defined(D_FT900)
typedef struct ASNPACKED {
   uint16_t FlagsLength;                                                        // Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
   uint32_t Vector;                                                             // Identifies 1.31 data as DMP Protocol PDU VECTOR_E131_DATA_PACKET (DMX512-A [DMX] data)
   uint8_t SourceName[64u];                                                     // User Assigned Name of Source UTF-8 [UTF-8] encoded string, null-terminated
   uint8_t PriorityData;                                                        // priority if multiple sources 0-200, default of 100
   uint16_t SynchroAddr;                                                        // Universe address on which  sync packets will be sent Universe on which synchronization packets are transmitted
   uint8_t SeqNumber;                                                           // Sequence Number To detect duplicate or out of order packets
   uint8_t OptionsFlags;                                                        // Bit 7 =  Preview_Data  Bit 6 =  Stream_Terminated Bit 5 = Force_Synchronization
   uint16_t Universe;                                                           // Universe Number Identifier for a distinct stream of DMX512-A [DMX] Data
} ASN_DataFrameLayer_t;
#else
ASNPACKED(
typedef struct {
   uint16_t FlagsLength;                                                        // Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
   uint32_t Vector;                                                             // Identifies 1.31 data as DMP Protocol PDU VECTOR_E131_DATA_PACKET (DMX512-A [DMX] data)
   uint8_t SourceName[64u];                                                     // User Assigned Name of Source UTF-8 [UTF-8] encoded string, null-terminated
   uint8_t PriorityData;                                                        // priority if multiple sources 0-200, default of 100
   uint16_t SynchroAddr;                                                        // Universe address on which  sync packets will be sent Universe on which synchronization packets are transmitted
   uint8_t SeqNumber;                                                           // Sequence Number To detect duplicate or out of order packets
   uint8_t OptionsFlags;                                                        // Bit 7 =  Preview_Data  Bit 6 =  Stream_Terminated Bit 5 = Force_Synchronization
   uint16_t Universe;                                                           // Universe Number Identifier for a distinct stream of DMX512-A [DMX] Data
}) ASN_DataFrameLayer_t;
#endif

#if defined(D_FT900)
typedef struct ASNPACKED {
   uint16_t FlagsLength;                                                        // Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
   uint8_t Vector;                                                              // Identifies DMP Set Property Message PDU VECTOR_DMP_SET_PROPERTY (from [ACN-DMP] 13.2)
   uint8_t AddressDataType;                                                     // Identifies format of address and data 0xa1
   uint16_t FirstPropertyAddress;                                               // Indicates DMX512-A START Code is at DMP address 0 0x0000
   uint16_t AddressIncrement;                                                   // Indicates each property is 1 octet 0x0001
   uint16_t PropertyValCnt;                                                     // Indicates 1+ the number of slots in packet 0x0001 -- 0x0201
   uint8_t PropertyVal[513u];                                                   // DMX512-A START Code + data START Code + Data
} ASN_DMPLayer_t;
#else
ASNPACKED(
typedef struct {
   uint16_t FlagsLength;                                                        // Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
   uint8_t Vector;                                                              // Identifies DMP Set Property Message PDU VECTOR_DMP_SET_PROPERTY (from [ACN-DMP] 13.2)
   uint8_t AddressDataType;                                                     // Identifies format of address and data 0xa1
   uint16_t FirstPropertyAddress;                                               // Indicates DMX512-A START Code is at DMP address 0 0x0000
   uint16_t AddressIncrement;                                                   // Indicates each property is 1 octet 0x0001
   uint16_t PropertyValCnt;                                                     // Indicates 1+ the number of slots in packet 0x0001 -- 0x0201
   uint8_t PropertyVal[513u];                                                   // DMX512-A START Code + data START Code + Data
}) ASN_DMPLayer_t;
#endif

// ====================== Synchronization Packet ===============================// The E1.31 Synchronization Packet is used to trigger synchronization. 
                                                                                // There is no additional data payload. Note: designation of a specific 
                                                                                // universe as the synchronization address does not preclude that universe 
#if defined(D_FT900)
typedef struct ASNPACKED {
   uint16_t FlagsLength;                                                        // Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
   uint32_t Vector;                                                             // Identifies 1.31 data as DMP Protocol PDU VECTOR_E131_DATA_PACKET (DMX512-A [DMX] data)
   uint8_t SeqNumber;                                                           // Sequence Number To detect duplicate or out of order packets
   uint16_t SyncAddress;                                                        // Universe Number Universe on which synchronization packets
   uint16_t Reserved; 
} ASN_SyncFrameLayer_t;
#else
ASNPACKED(
typedef struct {
   uint16_t FlagsLength;                                                        // Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
   uint32_t Vector;                                                             // Identifies 1.31 data as DMP Protocol PDU VECTOR_E131_DATA_PACKET (DMX512-A [DMX] data)
   uint8_t SeqNumber;                                                           // Sequence Number To detect duplicate or out of order packets
   uint16_t SyncAddress;                                                        // Universe Number Universe on which synchronization packets
   uint16_t Reserved;                                                           //
}) ASN_SyncFrameLayer_t;
#endif                                                                          // from also transporting E1.31 Data.

// ====================== Universe Discovery Packet ============================// E1.31 Universe Discovery Packets shall be sent by a source to advertise
                                                                                // which E1.31 universes it is actively transmitting on. A set of packets shall
                                                                                // be sent once every Superseded E131_UNIVERSE_DISCOVERY_INTERVAL, with an indicator 
                                                                                // as to what page number in the sequence of packets this is, as well as the number of total pages 
                                                                                // of packets that are intended to be transmitted. The main payload shall consist of a 
                                                                                // Universe Discovery Layer that contains these page numbers and a sorted list E1.31 universes. 
                                                                                // This list of universes may include synchronization universes
#if defined(D_FT900)
typedef struct ASNPACKED {
   uint16_t FlagsLength;                                                        // Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
   uint32_t Vector;                                                             // Identifies 1.31 data as DMP Protocol PDU VECTOR_E131_DATA_PACKET (DMX512-A [DMX] data)
   uint8_t SourceName[64u];                                                     // utf8 name
   uint32_t Reserved;    
} ASN_UniverseFrameLayer_t;
#else
ASNPACKED(
typedef struct {
   uint16_t FlagsLength;                                                        // Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
   uint32_t Vector;                                                             // Identifies 1.31 data as DMP Protocol PDU VECTOR_E131_DATA_PACKET (DMX512-A [DMX] data)
   uint8_t SourceName[64u];                                                     // utf8 name
   uint32_t Reserved;                                                           //
}) ASN_UniverseFrameLayer_t;
#endif 

#if defined(D_FT900)
typedef struct ASNPACKED {
  uint16_t FlagsLength;                                                         //Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
  uint32_t Vector;                                                              //Identifies Universe Discovery data as universe list VECTOR_UNIVERSE_DISCOVERY_UNIVERSE_LIST
  uint8_t PagePacketNumber;                                                     //Identifier indicating which packet of N this is—page numbers start at 0.
  uint8_t LastPage;                                                             //Final Page Page number of the final page to be transmitted.
  uint16_t ListUniverses[512u];                                                 //Sorted list of up to 512 16-bit universes    
} ASN_UniverseDiscoveryLayer_t;
#else
ASNPACKED(
typedef struct {
  uint16_t FlagsLength;                                                         //Protocol flags and length Low 12 bits = PDU length  High 4 bits = 0x7
  uint32_t Vector;                                                              //Identifies Universe Discovery data as universe list VECTOR_UNIVERSE_DISCOVERY_UNIVERSE_LIST
  uint8_t PagePacketNumber;                                                     //Identifier indicating which packet of N this is—page numbers start at 0.
  uint8_t LastPage;                                                             //Final Page Page number of the final page to be transmitted.
  uint16_t ListUniverses[512u];                                                 //Sorted list of up to 512 16-bit universes
}) ASN_UniverseDiscoveryLayer_t;
#endif 

#ifdef __cplusplus
}
#endif

#endif