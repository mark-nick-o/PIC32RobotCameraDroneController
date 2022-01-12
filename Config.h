#ifndef Config_H
#define Config_H

#ifdef __cplusplus
 extern "C" {
#endif

//    config.h : General system definitions and configurations
//
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
//
//Node Config
// Ethernet II, Src: Microchi_dc:e1:67 (54:10:ec:dc:e1:67), Dst: Broadcast (ff:ff:ff:ff:ff:ff)
// Ethernet II, Src: Microchi_dc:e1:a1 (54:10:ec:dc:e1:a1), Dst: Microchi_dc:e1:67 (54:10:ec:dc:e1:67)
// ether host 54:10:ec:dc:e1:67 or ether host 54:10:ec:dc:e1:a1
#include <stdint.h>
#include "definitions.h"
#include "Struts.h"
#include "camera.h"

unsigned char Node = THIS_NODE_IS;                                              // definitions.h :: AIR 1 MASGS 2 FIRGS 3 COMGS 4 GIMJOY 5
unsigned char UART2_Test_Mode =FALSE;                                           // Do fast 1 byte for TCP testing

//unsigned char   UART2_Timeout_Enable = 0x0000;                                   // Ennable = 0x8008; Disable = 0X0000
unsigned char Server = FALSE;
unsigned char TCP = FALSE;                                                      // If TCP = 1 then TCP mode If =0 then UDP mode
unsigned char UDP_Test_Mode =FALSE;                                             // If Test 1 then ping back to PC
unsigned char TCP_Test_Mode =FALSE;                                             // If Test 1 then ping back to PC

#ifdef USE_TCP                                                                                //1 = Server Mode 0 = Client
unsigned char UART1TCP =FALSE;                                                  //1= UART1 is TCP mode                                                        //1 = Server Mode 0 = Client
//unsigned char   UART2TCP =FALSE;
unsigned char UART3TCP =FALSE;
unsigned char UART4TCP =FALSE;
unsigned char UART5TCP =FALSE;
#endif

#ifdef USE_CAN
unsigned char CAN1TCP =FALSE;
unsigned char CAN2TCP =FALSE;
unsigned char CAN1_Test_Mode =0U;                                               // If Test 1 then gen data on CAN1 (Do nothing else)
unsigned char CAN2_Test_Mode =0U;                                               // If Test 1 then gen data on CAN2 (Do nothing else)
uint16_t Can1_TX_MSG_ID;
uint16_t Can2_TX_MSG_ID;
#endif

Node This_Node;
Node Remote_Node[10u];
Node Air;                                                                       // Air COM PIC
Node MasGs;                                                                     // Master Ground station
Node FirGs;                                                                     // Fire Ground Station
#ifdef YI_CAM_USED
Node YiCam;                                                                     // Yi Cam JSON server
#endif
#ifdef SEQ_CAM_USED
Node ParrotSeq;                                                                 // Parrot sequioa camera
#endif
#if defined(MODBUS_TCP)
Node ModbusTcp;                                                                 // Modbus TCP Node
#endif

uint8_t Air_MacAddr[6u] = {0x54U, 0x10U, 0xecU, 0xdcU, 0xe1U, 0xa1U};           //  hardcoded MAC address
uint8_t Air_IpAddr[4u] = {192U, 172U,  3U, 2U };                                //  IP address
uint8_t Air_Link;                                                               //  Air COM PIC is linked

uint8_t MasGs_MacAddr[6u] = {0x54U, 0x10U, 0xecU, 0xdcU, 0xe1U, 0x67U};         //  hardcoded MAC address
uint8_t MasGs_IpAddr[4u] = {192U, 172U, 3U, 3U };                               //  IP address
uint8_t MasGs_Link;                                                             //  Master Ground Station is linked

uint8_t FirGs_MacAddr[6u] = {0x54U, 0x10U, 0xecU, 0xdcU, 0xe1U, 0x68U};         //  hardcoded MAC address
uint8_t FirGs_IpAddr[4u] = {192U, 172U, 3U, 4U };                               //  IP address
uint8_t FirGs_Link;                                                             //  Fire Ground Station is linked

uint8_t Com1Gs_MacAddr[6u] = {0x54U, 0x10U, 0xecU, 0xdcU, 0xe1U, 0x69U};        //  hardcoded MAC address
uint8_t Com1Gs_IpAddr[4u]  = {192U, 172U, 3U, 5U };                             //  IP address
uint8_t Com1Gs_Link;                                                            //  Commander 1 Ground station is linked

uint8_t GimJoy_MacAddr[6u] = {0x54U, 0x10U, 0xecU, 0xdcU, 0xe1U, 0x70U};        //  hardcoded MAC address
uint8_t GimJoy_IpAddr[4u] = {192U, 172U, 3U, 10U };                             //  IP address
uint8_t GimJoy_Link;                                                            //  Gimbal Joystick controller station is linked

//unsigned char Ardupilot_MacAddr[6u] = {0x54U, 0x10U, 0xecU, 0xddU, 0xf2U, 0x63U};  //  hardcoded MAC address
unsigned char Ardupilot_IpAddr[4u] = {192U, 172U, 3U, 11U };                    //  IP address
unsigned char Ardupilot_Link;                                                   //  Ardupilot laptop station is linked

#if defined(SEQ_CAM_USED)
//unsigned char ParrotSeq_MacAddr[6u] = {0x64U, 0xF7U, 0x76U, 0x67U, 0xe2U, 0xaeU};  //  hardcoded MAC address
unsigned char ParrotSeq_IpAddr[4u] = {192U, 172U, 3U, 12U };                    //  IP address
unsigned char ParrotSeq_Link;                                                   //  Parrot Sequoia camera is linked
#endif

#if defined(YI_CAM_USED)
//unsigned char YiCam_MacAddr[6u] = {0x99U, 0x11U, 0x45U, 0x78U, 0x98U, 0x54U};   //  hardcoded MAC address
unsigned char YiCam_IpAddr[4u] = {192U, 172U, 3U, 13U };                        //  IP address
unsigned char YiCam_Link;                                                       //  Parrot Sequoia camera is linked
#endif

#if defined(MICROSCAN_USED)
//unsigned char microScan_MacAddr[6u] = {0x96U, 0x09U, 0x11U, 0x65U, 0x66U, 0x67U};   //  hardcoded MAC address
unsigned char microScan_IpAddr[4u] = {192U, 172U, 3U, 14U };                    //  IP address
unsigned char microScan_Link;                                                   //  microscan camera is linked
#endif

#if defined(LW3_SWITCH_USED)
//unsigned char vidAudSwitch_MacAddr[6u] = {0x55U, 0x87U, 0x56U, 0x15U, 0x16U, 0xA7U};   //  hardcoded MAC address
unsigned char vidAudSwitch_IpAddr[4u] = {192U, 172U, 3U, 15U };                 //  IP address
unsigned char vidAudSwitch_Link;                                                //  lightware video / audio switch is linked
#endif

#if defined(GEF_EGD_PLC)
//unsigned char gefPLC_MacAddr[6u] = {0x88U, 0x21U, 0x09U, 0x70U, 0x50U, 0x66U};   //  hardcoded MAC address
unsigned char gefPLC_IpAddr[4u] = {192U, 172U, 3U, 16U };                       //  IP address
unsigned char gefPLC_Link;                                                      //  GEFanuc PLC is linked
#endif

#if defined(ART_NET_USED)
//unsigned char artNet_MacAddr[6u] = {0x32U, 0x10U, 0x77U, 0x75U, 0xf0U, 0xe6U};   //  hardcoded MAC address
unsigned char artNet_IpAddr[4u] = {192U, 172U, 3U, 17U };                       //  IP address
unsigned char artNet_Link;                                                      //  ArtNet4 dmx node is linked
#endif

#if defined(CBOR_COMS_USED)
//unsigned char cborM2M_MacAddr[6u] = {0x1dU, 0x1eU, 0x1aU, 0x1cU, 0x1bU, 0x1fU}; //  hardcoded MAC address
unsigned char cborM2M_IpAddr[4u] = {192U, 172U, 3U, 18U };                      //  IP address
unsigned char cborM2M_Link;                                                     //  m2m server using CBOR is linked
#endif

#if defined(GEO_JSON_USED)
//unsigned char geoJSON_MacAddr[6u] = {0x67U, 0x76U, 0x65U, 0x98U, 0x1bU, 0x33U}; //  hardcoded MAC address
unsigned char geoJSON_IpAddr[4u] = {192U, 172U, 3U, 19U };                      //  IP address
unsigned char geoJSON_Link;                                                     // geoJSON server e.g. googleEarth
#endif

#if defined(RION_USED)
//unsigned char rionObj_MacAddr[6u] = {0x76U, 0x67U, 0x11U, 0x11U, 0x11U, 0x33U}; //  hardcoded MAC address
unsigned char rionObj_IpAddr[4u] = {192U, 172U, 3U, 20U };                      //  IP address
unsigned char rionObj_Link;                                                     // raw internet object notation server e.g. ntp time
#endif

#if defined(USE_MSGPACK)
//unsigned char msgPak_MacAddr[6u] = {0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x71U};  //  hardcoded MAC address
unsigned char msgPak_IpAddr[4u] = {192U, 172U, 3U, 21U };                       //  IP address
unsigned char msgPak_Link;                                                      // message pack binary object server
#endif

#if defined(SMPTE_USED)
//unsigned char smpte_MacAddr[6u] = {0x66U, 0x77U, 0x76U, 0x67U, 0x15U, 0x74U};   //  hardcoded MAC address
unsigned char smpte_IpAddr[4u] = {192U, 172U, 3U, 22U };                        //  IP address
unsigned char smpte_Link;                                                       // smpte video stamper
#endif

#if defined(ASN_E1_USED)
//unsigned char asnE1_MacAddr[6u] = {0x77U, 0x66U, 0x53U, 0xacU, 0xdaU, 0xedU};   //  hardcoded MAC address
unsigned char asnE1_IpAddr[4u] = {192U, 172U, 3U, 23U };                        //  IP address
unsigned char asnE1_Link;                                                       // smpte video stamper
#endif

#if defined(MODBUS_TCP)
//unsigned char modTCP1_MacAddr[6u] = {0x22U, 0x22U, 0x22U, 0x06U, 0x04U, 0x03U}; //  hardcoded MAC address
unsigned char modTCP1_IpAddr[4u] = {192U, 172U, 3U, 30U };                      //  IP address
unsigned char modTCP1_Link;                                                     //  modbus tcp remote i/o 1 is linked
#endif

#if defined(UBIBOT_SERVER_USED)
//unsigned char ubiBot_MacAddr[6u] = {0x05U, 0x03U, 0xf5U, 0xe3U, 0x3aU, 0x3dU};  //  hardcoded MAC address
unsigned char ubiBot_IpAddr[4u] = {192U, 172U, 3U, 40U };                       //  IP address
unsigned char ubiBot_Link;                                                      //  uBi-Bot server is linked
#endif

unsigned char gwIpAddr[4u] = {192U, 172U,  3U, 1U };                            //  gateway (router) IP address
unsigned char ipMask[4u] = {255U, 255U,  255U, 0U };                            //  network mask (for example : 255.255.255.0)
unsigned char dnsIpAddr[4u] = {192U, 172U,  3U, 1U };                           //  DNS server IP address


//================= GLOBAL HMI SCREEN VALUES FROM XS ENCODER ===================
volatile unsigned char g_diskStatus[30u];                                       // State of the recording disk
volatile unsigned char g_ch1Status[20u];                                        // Channel 1 state i.e recording or not
volatile unsigned char g_ch2Status[20u];                                        // Channel 2 state i.e recording or not
volatile unsigned char g_ch3Status[20u];                                        // Channel 3 state i.e recording or not
volatile unsigned char g_ch4Status[20u];                                        // Channel 4 state i.e recording or not
volatile unsigned char g_sysStatus[30u];                                        // State of the recording system
volatile uint16_t g_totalSizeXS;                                                // Total size of the disk in MB
volatile uint16_t g_remSizeXS;                                                  // Remaining size of the disk in MB
volatile unsigned char g_encTime[20u];                                          // Date and Time
volatile unsigned char g_strField1[20u];                                        // String field 1 after +ok, comma
volatile unsigned char g_strField2[20u];                                        // String field 2 after +ok,, comma
volatile unsigned char g_strField3[20u];                                        // String field 3 after +ok,,, comma
volatile unsigned char g_strField4[20u];                                        // String field 4 after +ok,,,, comma
volatile unsigned char g_strField5[20u];                                        // String field 5 after +ok,,,,, comma
volatile unsigned char g_strField6[20u];                                        // String field 5 after +ok,,,,, comma
volatile unsigned char g_strField7[20u];                                        // String field 5 after +ok,,,,, comma
volatile unsigned char g_strField8[20u];                                        // String field 5 after +ok,,,,, comma
volatile int32_t g_okField1;                                                    // numeric field extracted from +ok, { $GET<cmd> }
volatile int32_t g_okField2;                                                    // numeric field extracted from +ok,, { $GET<cmd> }
volatile int32_t g_okField3;                                                    // numeric field extracted from +ok,,, { $AVGBITRATE<cmd> }
volatile int32_t g_okField4;                                                    // numeric field extracted from +ok,,,, { $AVGBITRATE<cmd> }
volatile int32_t g_okField5;                                                    // numeric field extracted from +ok,,,,, { $ENUMSTREAMS<cmd> }
volatile int32_t g_okField6;                                                    // numeric field extracted from +ok,,,,,, { $GETFRAMERATE<cmd> }
volatile int32_t g_okField7;                                                    // numeric field extracted from +ok,,,,,,, { $GETFRAMERATE<cmd> }
volatile int32_t g_okField8;                                                    // numeric field extracted from +ok,,,,,,, { $GETFRAMERATE<cmd> }
#ifdef GPS_INCLUDED2
//============== GLOBAL HMI SATELITE DATA ======================================
volatile uint8_t elevSatelData[4u];                                             // possible 4 satelite elevations
volatile uint8_t azimuthSatelData[4u];                                          // possible 4 satelite azimuth
volatile uint8_t sigStrengthSatelData[4u];                                      // possible 4 signal stgrengths
volatile uint8_t signalIDData;                                                  // signal id 1: GPGSV or GLGSV 7: GAGSV
volatile uint8_t satNumberData[4u];                                             // satelite number stored in byte
#endif
// UART CONFIG
//      UART1
#ifdef OTHER_UART
uint16_t UART1_Baud = 57600U;                                                   // Baudrate for serial 1
uint16_t UART1_from_Port = 10021U;                                              // UDP source port for UART 1
uint16_t UART1_dest_Port = 10001U;                                              // UDP Destination port for UART 1
//      UART2
/*unsigned int    UART2_Baud       = 9600;                                      // Baudrate for serial 1
unsigned int    UART2_from_Port  = 10021;                                       // UDP scource port for UART 2
unsigned int    UART2_dest_Port  = 10002;                                       // UDP Destination port for UART 2
unsigned int    UART2_ACK_Port   = 10022;
unsigned char   UART2_Packet     =FALSE;
unsigned char   U2Buff[32];
unsigned char   UART2_Process_State = CLEAR;*/
//       UART3
uint16_t UART3Baud = 57600U;                                                    // Baudrate for serial 3
uint16_t UART3fromPort = 10003U;                                                // UDP source port for UART 3
uint16_t UART3destPort = 10003U;                                                // UDP Destination port for UART 3
uint16_t UART4Baud = 57600U;                                                    // Baudrate for serial 4
uint16_t UART4fromPort = 10004U;                                                // UDP source port for UART 4
uint16_t UART4destPort = 10004U;                                                // UDP Destination port for UART 4
uint16_t UART5Baud = 57600U;                                                    // Baudrate for serial 5
uint16_t UART5fromPort = 10005U;                                                // UDP source port for UART 5
uint16_t UART5destPort = 10005U;                                                // UDP Destination port for UART 5
uint16_t UART6Baud = 57600U;                                                    // Baudrate for serial 6
uint16_t UART6fromPort = 10006U;                                                // UDP source port for UART 6
uint16_t UART6destPort = 20006U;
unsigned char U1Buff[32u];

unsigned char U3Buff[32u];
unsigned char U4Buff[32u];
unsigned char U5Buff[32u];
unsigned char U6Buff[32u];
unsigned char U1BuffInd =0u;
unsigned char U2BuffInd =0u;
unsigned char U3BuffInd =0u;
unsigned char U4BuffInd =0u;
unsigned char U5BuffInd =0u;
unsigned char U6BuffInd =0u;
#endif

// Network Port Definitions appear here                                         used in Net_Ethernet_Intern_UserUDP() switch case

#ifdef USE_CAN
//**************CAN cinfig********************
uint16_t Can1fromPort = 20041U;                                                 /* UDP source port for UART */
uint16_t Can1destPort = 20041U;
uint16_t Can2fromPort = 20042U;
uint16_t Can2destPort = 20042U;                                                 /* UDP source port for UART */

const unsigned int CAN_CONFIG_FLAGS =  _CAN_CONFIG_SAMPLE_ONCE & _CAN_CONFIG_PHSEG2_PRG_ON & _CAN_CONFIG_STD_MSG & _CAN_CONFIG_MATCH_MSG_TYPE & _CAN_CONFIG_LINE_FILTER_OFF;

//CAN // CAN Initializations constants
// Baud rate is set according to following formula
// Fbaud = Fosc/(2*N*BRP),  N = SYNC + PHSEG1 + PHSEG2 + PROPSEG = 16
// In this case Fbaud = 125000
const uint16_t SJW = 1U;
const uint16_t BRP = 32U;
const uint16_t PHSEG1 = 1U;
const uint16_t PHSEG2 = 3U;
const uint16_t PROPSEG = 5U;

// with CAN2Write
const unsigned int  Can2_Send_Flags = _CAN_TX_STD_FRAME & _CAN_TX_NO_RTR_FRAME;

char Can1_Msg_Rcvd[8u];
int16_t Can_msg_rcvd;
char Can1_Rx_Data1[8u];
char Can1_Tx_Data[8u];
char Can2_Rx_Data[8u];
char Can2_Tx_Data[8u];

uint16_t PLen;                                                                  // Packet Length
unsigned char Packet;
uint16_t Test=0U;                                                               // If Test 1 then ping back to PC
char MSG[6u];
char Can2_Msg_Rcvd[8u];
int16_t Can2_MSG_Len;
char Can_RxTx_Data1[8u];                                                        // Can TEST data
int16_t Can2_MSG_ID;

char FIFObuffers[2ul*8ul*16ul] absolute 0xA0000000UL;
#endif
// reserve space for 2 buffers with 8 messages (each message is 16 bytes)
// beggining of the buffer must be 32bit aligned

#ifdef __cplusplus
}
#endif

#endif /*Config_H*/