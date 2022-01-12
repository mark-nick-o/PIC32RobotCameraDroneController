#ifndef __MovinSign_proto_
#define __MovinSign_proto_
/* -----------------------------------------------------------------------------
   %%%%%%%%%%%%%%%%%%% Moving Sign Protocol %%%%%%%%%%%%%%%

    Written By ACP Aviation
    Allows Display of values to advertising board / Bus Display etc

   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
#include "definitions.h"
#include "stdint.h"
/*#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>  */
#include "cpu_endian_defs.h"                                                    // endian definitions

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define MOVSINPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define MOVSINPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define MOVSINPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define MOVSINPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define MOVSINPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define MOVSIN_SOH 0x01u
#define MOVSIN_STX 0x02u
#define MOVSIN_ETX 0x03u
#define MOVSIN_EOT 0x04u

#define MOVSIN_SEND_DEFAULT { 0x00000000lu, 0x00u, MOVSIN_SOH, 0x0000u, 0x0000u, MOVSIN_STX, 0x00u, 0x00u, MOVSIN_ETX, 0x00000000lu, MOVSIN_EOT }

#define MOVSIN_WRITEHEXVAL2U(A,x) do { sprintf(A,"%4x",x); } while(0)
#define MOVSIN_WRITEHEXVAL4U(A,x) do { sprintf(A,"%8x",x); } while(0)
#define MOVSIN_WRITETXTGRAPH2U(A,x) do { if (((x>=48) && (x<=57)) || ((x>=65) && (x<=90)))  sprintf(A,"%4x",(x|0xFC)); else A=0; } while(0)
#define MOVSIN_WRITETXTVAR2U(A,x) do { if (((x>=48) && (x<=57)) || ((x>=65) && (x<=86)))  sprintf(A,"%4x",(x|0xFB)); else A=0; } while(0)
#define MOVSIN_WRITEVARATTR3U(A,len,color) do { if (x>=99) sprintf(A,"%4x%2x",len,color); else A=0; } while(0)
#define MOVSIN_WRITESUBCOMMAND(A,subCmd,dat) do { sprintf(A,"%x%x",subCmd,dat); } while(0)
#define MOVSIN_FORMATDATE(A,yr,mon,day,hr,min,sec,dow) do { sprintf(A,"%4d%2d%2d%2d%2d%2d%1d",yr,mon,day,hr,min,sec,dow); } while(0)

typedef enum
{
  MovSinMdeAuto=65,                                                             // ascii "A"
  MovSinMdeFlash=66,                                                            // ascii "B"
  MovSinMdeHold=67,                                                             // ascii "C"
  MovSinMdeInterlock=68,                                                        // ascii "D"
  MovSinMdeRollDown=69,                                                         // ascii "E"
  MovSinMdeRollUp=70,                                                           // ascii "F"
  MovSinMdeRollIn=71,                                                           // ascii "G"
  MovSinMdeRollOut=72,                                                          // ascii "H"
  MovSinMdeRollLeft=73,                                                         // ascii "I"
  MovSinMdeRollRight=74,                                                        // ascii "J"
  MovSinMdeRotate=75,                                                           // ascii "K"
  MovSinMdeSlide=76,                                                            // ascii "L"
  MovSinMdeSnow=77,                                                             // ascii "M"
  MovSinMdeSparkle=78,                                                          // ascii "N"
  MovSinMdeSpray=79,                                                            // ascii "O"
  MovSinMdeStarburst=80,                                                        // ascii "P"
  MovSinMdeSwitch=81,                                                           // ascii "Q"
  MovSinMdeTwinkle=82,                                                          // ascii "R"
  MovSinMdeWipedown=83,                                                         // ascii "S"
  MovSinMdeWipeUp=84,                                                           // ascii "T"
  MovSinMdeWipeIn=85,                                                           // ascii "U"
  MovSinMdeWipeOut=86,                                                          // ascii "V"
  MovSinMdeWipeLeft=87,                                                         // ascii "W"
  MovSinMdeWipeRight=88,                                                        // ascii "X"
  MovSinMdeCycleColor=89,                                                       // ascii "Y"
  MovSinMdeClock=90                                                             // ascii "Z"
} moveSign_dispMode_e;

typedef enum
{
  MovSinMdeSMALL=0xFE40,                                                        // ascii "@"  || 0xFE
  MovSinMdeSS5=0xFE41,                                                          // ascii "A"
  MovSinMdeST5=0xFE42,                                                          // ascii "B"
  MovSinMdeWD5=0xFE43,                                                          // ascii "C"
  MovSinMdeWS5=0xFE44,                                                          // ascii "D"
  MovSinMdeSS7=0xFE45,                                                          // ascii "E"
  MovSinMdeST7=0xFE46,                                                          // ascii "F"
  MovSinMdeWD7=0xFE47,                                                          // ascii "G"
  MovSinMdeWS7=0xFE48,                                                          // ascii "H"
  MovSinMdeSDS=0xFE49,                                                          // ascii "I"
  MovSinMdeSRF=0xFE4A,                                                          // ascii "J"
  MovSinMdeSTF=0xFE4B,                                                          // ascii "K"
  MovSinMdeWDF=0xFE4C,                                                          // ascii "L"
  MovSinMdeWSF=0xFE4D,                                                          // ascii "M"
  MovSinMdeSDF=0xFE4E,                                                          // ascii "N"
  MovSinMdeSS10=0xFE4F,                                                         // ascii "O"
  MovSinMdeST10=0xFE50,                                                         // ascii "P"
  MovSinMdeWD10=0xFE51,                                                         // ascii "Q"
  MovSinMdeWS10=0xFE52,                                                         // ascii "R"
  MovSinMdeSS15=0xFE53,                                                         // ascii "S"
  MovSinMdeST15=0xFE54,                                                         // ascii "T"
  MovSinMdeWD15=0xFE55,                                                         // ascii "U"
  MovSinMdeWS15=0xFE56,                                                         // ascii "V"
  MovSinMdeSS23=0xFE57,                                                         // ascii "W"
  MovSinMdeSS31 =0xFE58                                                         // ascii "X"
} moveSign_fontMode_e;

typedef enum
{
  MovSinMdeAutoColor=0xFD41,                                                    // ascii "A"  | 0xFD
  MovSinMdeLightRed=0xFD42,                                                     // ascii "B"
  MovSinMdeLightGreen=0xFD43,                                                   // ascii "C"
  MovSinMdeRed=0xFD44,                                                          // ascii "D"
  MovSinMdeGreen=0xFD45,                                                        // ascii "E"
  MovSinMdeYellow=0xFD46,                                                       // ascii "F"
  MovSinMdeBrown=0xFD47,                                                        // ascii "G"
  MovSinMdeAmber=0xFD48,                                                        // ascii "H"
  MovSinMdeOrange=0xFD49,                                                       // ascii "I"
  MovSinMdeMixV1=0xFD4A,                                                        // ascii "J"
  MovSinMdeMixV2=0xFD4B,                                                        // ascii "K"
  MovSinMdeMixH=0xFD4C,                                                         // ascii "L"
  MovSinMdeBlack=0xFD4D                                                         // ascii "M"
} moveSign_colorMode_e;                                                         // text colors

typedef enum
{
  MovSinVarLightRed=0x42,                                                       // ascii "B"
  MovSinVarLightGreen=0x43,                                                     // ascii "C"
  MovSinVarRed=0x44,                                                            // ascii "D"
  MovSinVarGreen=0x45,                                                          // ascii "E"
  MovSinVarYellow=0x46,                                                         // ascii "F"
  MovSinVarBrown=0x47,                                                          // ascii "G"
  MovSinVarAmber=0x48,                                                          // ascii "H"
  MovSinVarOrange=0x49,                                                         // ascii "I"
} moveSign_colorVar_e;                                                          // value colors

typedef enum
{
  MovSinGraLightRed=0x42,                                                       // ascii "B"
  MovSinGraLightGreen=0x43,                                                     // ascii "C"
  MovSinGraRed=0x44,                                                            // ascii "D"
  MovSinGraGreen=0x45,                                                          // ascii "E"
  MovSinGraYellow=0x46,                                                         // ascii "F"
  MovSinGraBrown=0x47,                                                          // ascii "G"
  MovSinGraAmber=0x48,                                                          // ascii "H"
  MovSinGraOrange=0x49,                                                         // ascii "I"
  MovSinGraBlack=0x4D                                                           // ascii "M"
} moveSign_colorGraph_e;                                                        // value graphics / led sign

typedef enum
{
  MovSinMdeDt1=0xFA41,                                                          // ascii "A"  | 0xFD  hh:mm:ss
  MovSinMdeDt2=0xFA42,                                                          // ascii "B" hh:mm:ss A/PM
  MovSinMdeDt3=0xFA43,                                                          // ascii "C" hh:mm
  MovSinMdeDt4=0xFA44,                                                          // ascii "D" hh:mm A/PM
  MovSinMdeDt5=0xFA45,                                                          // ascii "E" mm/dd/yyyy
  MovSinMdeDt6=0xFA46,                                                          // ascii "F" yyyy-mm-dd
  MovSinMdeDt7=0xFA47,                                                          // ascii "G" dd.MM yyyy
  MovSinMdeDt8=0xFA48,                                                          // ascii "H" mm’dd’yyyy
  MovSinMdeDt9=0xFA49,                                                          // ascii "I" English week shortened form
  MovSinMdeDt10=0xFA4A,                                                         // ascii "J" English week full form
  MovSinMdeDt11=0xFA4B,                                                         // ascii "K" For example, “010030000130”, shows the start countdown time is 01:00:30, countdown time is 1 minute 30 second.
  MovSinMdeDt12=0xFA4C                                                          // ascii "L" For example, “051023”, shows the end date is 2005-10-23
} moveSign_timeCountDown_e;

typedef enum
{
  MovSinTempC=0xF941,                                                           // ascii "A"  degC Celcius
  MovSinTempF=0xF942                                                            // ascii "B" F Farenheit
} moveSign_tempSelect_e;

typedef enum
{
  MovSinMdeFastest=0,                                                           // fastest
  MovSinMdeFast=1,                                                              // fast
  MovSinMdeNormal=2,                                                            // normal
  MovSinMdeSlow=3,                                                              // slow
  MovSinMdeSlowest=4                                                            // slowest
} moveSign_dispSpeed_e;

typedef enum
{
  MovSinSunday=0b00000001,                                                      // show it on a sunday
  MovSinMonday=0b00000010,                                                      // show it on a monday
  MovSinTuesday=0b00000100,                                                     // show it on a tuesday
  MovSinWednesday=0b00001000,                                                   // slow it on wednesday
  MovSinThursday=0b00010000,                                                    // show it on a thursday
  MovSinFriday=0b00100000,                                                      // show it on a friday
  MovSinSaturday=0b01000000                                                     // slow it on saturday
} moveSign_showDates_e;

typedef enum
{
  MovSinAlignLeft=1,                                                            // align left
  MovSinAlignRight=2,                                                           // align right
  MovSinAlignCenter=3,                                                          // align center
} moveSign_alignment_e;

#if defined(D_FT900)
typedef struct MOVSINPACKED {
        moveSign_fontMode_e fontVal;                                            // moveSign_fontMode_e as its a uint16_t
        moveSign_colorMode_e colorVal;                                          // moveSign_colorMode_e
        uint16_t graphFile;                                                     // Graphics file name, the virtual value is ‘0’--‘9’,’A’--‘Z’ e.g. for B use WRITEGRAPH2U(mov->graphFile,"B");
        uint16_t variable;                                                      // Variable name, the virtual value is ‘0’—‘9’,’A’—‘V’, total number is 32. WRITEVAR2U(mov.variable,"3");
        moveSign_timeCountDown_e timeCountDown;                                 //  moveSign_timeCountDown_e
        moveSign_tempSelect_e temperature;                                      //  moveSign_tempSelect_e
        uint8_t enter;                                                          // set to 0x7F
        uint8_t insideSymbol;                                                   // From 0xd0 to 0xea. 26 types symbol.
        uint8_t ascii;                                                          // The available character 0X20 – 0X7e in the ASCII character string table
} movSign_TextOptions_t;
#else
MOVSINPACKED (
typedef struct {
        moveSign_fontMode_e fontVal;                                            // moveSign_fontMode_e as its a uint16_t
        moveSign_colorMode_e colorVal;                                          // moveSign_colorMode_e
        uint16_t graphFile;                                                     // Graphics file name, the virtual value is ‘0’--‘9’,’A’--‘Z’ e.g. for B use WRITEGRAPH2U(mov->graphFile,"B");
        uint16_t variable;                                                      // Variable name, the virtual value is ‘0’—‘9’,’A’—‘V’, total number is 32. WRITEVAR2U(mov.variable,"3");
        moveSign_timeCountDown_e timeCountDown;                                 //  moveSign_timeCountDown_e
        moveSign_tempSelect_e temperature;                                      //  moveSign_tempSelect_e
        uint8_t enter;                                                          // set to 0x7F
        uint8_t insideSymbol;                                                   // From 0xd0 to 0xea. 26 types symbol.
        uint8_t ascii;                                                          // The available character 0X20 – 0X7e in the ASCII character string table
}) movSign_TextOptions_t;
#endif

#if defined(D_FT900)
typedef struct MOVSINPACKED {
        uint8_t displayMode;                                                    // moveSign_dispMode_e
        uint8_t displaySpeed;                                                   // moveSign_dispSpeed_e
        uint8_t sparePause : 4u;
        uint8_t pauseTime : 4u;                                                 // pause time 0-9
        uint16_t showDate;                                                      // ASCII WRITEHEXVAL2U(moveSign_showDates_e)
        uint32_t startShowTime;                                                 // time to start e.g. 18:30 pm is ASCII 1830 = 4bytes 0x49 0x56 0x51 0x48
        uint32_t endShowTime;                                                   // time to stop e.g. 18:30 pm is ASCII 1830 = 4bytes 0x49 0x56 0x51 0x48
        uint8_t preparatave0;                                                   // always 0
        uint8_t preparatave00;                                                  // always 0
        uint8_t preparatave000;                                                 // always 0
        uint8_t spareAli : 6u;
        uint8_t alignMode : 2u;                                                 // moveSign_alignment_e 2 bit field only
        unsigned char Buf[];
} movSign_TextCmd_t;
#else
MOVSINPACKED (
typedef struct {
        uint8_t displayMode;                                                    // moveSign_dispMode_e
        uint8_t displaySpeed;                                                   // moveSign_dispSpeed_e
        uint8_t sparePause : 4u;
        uint8_t pauseTime : 4u;                                                 // pause time 0-9
        uint16_t showDate;                                                      // ASCII WRITEHEXVAL2U(moveSign_showDates_e)
        uint32_t startShowTime;                                                 // time to start e.g. 18:30 pm is ASCII 1830 = 4bytes 0x49 0x56 0x51 0x48
        uint32_t endShowTime;                                                   // time to stop e.g. 18:30 pm is ASCII 1830 = 4bytes 0x49 0x56 0x51 0x48
        uint8_t preparatave0;                                                   // always 0
        uint8_t preparatave00;                                                  // always 0
        uint8_t preparatave000;                                                 // always 0
        uint8_t spareAli : 6u;
        uint8_t alignMode : 2u;                                                 // moveSign_alignment_e 2 bit field only
        unsigned char Buf[];
}) movSign_TextCmd_t;
#endif

#if defined(D_FT900)
typedef struct MOVSINPACKED {
        uint8_t variableName;                                                   // variable name
        uint16_t attributeLen;
        uint8_t attributeColor;
        unsigned char Buf[];
} movSign_VarCmd_t;
#else
MOVSINPACKED (
typedef struct {
        uint8_t variableName;                                                   // variable name
        uint16_t attributeLen;
        uint8_t attributeColor;
        unsigned char Buf[];
}) movSign_VarCmd_t;
#endif

#if defined(D_FT900)
typedef struct MOVSINPACKED {
        uint8_t vitualValue;                                                    // graphics name
        uint16_t graphLen;
        uint8_t comma;                                                          // 0x2C or 44 ','
        uint16_t graphHt;
        uint8_t attributeColor;
        unsigned char Buf[];
} movSign_GraphCmd_t;
#else
MOVSINPACKED (
typedef struct {
        uint8_t vitualValue;                                                    // graphics name
        uint16_t graphLen;
        uint8_t comma;                                                          // 0x2C or 44 ','
        uint16_t graphHt;
        uint8_t attributeColor;
        unsigned char Buf[];
}) movSign_GraphCmd_t;
#endif

typedef enum
{
  MovSinWriteTextFile=65,                                                       // ascii "A"
  MovSinWriteVariable=67,                                                       // ascii "C"
  MovSinWriteGraphics=69,                                                       // ascii "E"
  MovSinReadSpecFunc=82,                                                        // ascii "R"
  MovSinWriteSpecFunc=87                                                        // ascii "W"
} moveSign_Cmd_e;                                                               // definiiton of the command byte

typedef enum
{
  MovSinSetClock=65,                                                            // ascii "A" set clock
  MovSinReset=66,                                                               // ascii "B" reset
  MovSinSetPassword=67,                                                         // ascii "C" set password
  MovSinDeviceNum=68,                                                           // ascii "D" device number
  MovSinCtrlDtTime=69,                                                          // ascii "E" control time
  MovSinCueVoice=74,                                                            // ascii "J" cue voice 1=on 0=off
  MovSinNeedPassword=75,                                                        // ascii "K" need password = 1 or 0 = neednt enter passsowrd
  MovSinClear=76,                                                               // ascii "L" Clear all data will delete all the display data and can’t resume.
  MovSinSetBright=80,                                                           // ascii "P" set brightness
  MovSinSetUpVoice=89,                                                          // ascii "Y" Set up key cue voice,”1”== turn on, “0”== turn off
  MovSinShowWidth=90,                                                           // ascii "Z" show LED sign width, use 2 ASCII show HEX value. ”50” is 80 dots width
  MovSinCtrlDisAll=0x4641,                                                      // ascii "FA" display mode set up display all
  MovSinCtrlDisTim=0x4654,                                                      // ascii "FT" display according to set-up time
  MovSinNeedStart=0x5A4C,                                                       // ascii "ZL" show whether need start message, ‘0’ == no need, ‘1’ == need
  MovSinShowStorLoc=0x5A4D,                                                     // ascii "ZM" show storage location , ’0’ == FLASH; ‘1’ == RAM;
  MovSinShowMulti=0x5A4E,                                                       // ascii *ZN* show single sign or multi-sign, ‘0’ == Single sign; ‘1’ == Multi-sign, use 485.
  MovSinShowSigCol=0x5A43                                                       // ascii "ZC" show sign color C’ show LED sign color, ‘0’ == MONO; ‘1’ == TRICOLOR
} moveSign_SubCmd_e;                                                            // definiiton of the sub command byte for MovSinWriteSpecFunc

#if defined(D_FT900)
typedef struct MOVSINPACKED {
        uint32_t BlankStart;                                                    // 0x00 0x00 0x00 0x00
        uint8_t BlankStart1;                                                    // 0x00
        uint8_t SOH;                                                            // MOVSIN_SOH start of header
        uint16_t sendAddrAscii;                                                 // 2 byte ASCII Sending address, appointed “FF” as pc address, “00” is as broadcast address which can’t be as sender address
        uint16_t recvAddrAscii;                                                 // Display Address as per 2byte ASCII Receiving address, “00” is broadcast address, all sign will receive data; “FF” appoint pc fixed address which is used when sign return data to pc. “?” is wildcard character, “1?” allows the signs between “10” to “1F” to receive data.
        uint8_t STX;                                                            // MOVSIN_STX start of transmission characture
        uint8_t cmdAscii;                                                       // moveSign_Cmd_e Command code, 1-byte ASCII character shows different functions. Command code Description ‘A’ Write text file command ‘C’ Write variable command ‘E’ Write graphics file command ‘W’ Write special function command ‘R’ Read special function command
        uint8_t Buf[];                                                          // data packet variable length
        uint8_t ETX;                                                            // MOVSIN_ETX End of TeXt
        uint32_t CheckSumAscii;                                                 // 4 byte ASCII
        uint8_t EOT;                                                            // MOVSIN_EOT End of Text
} movSign_Send_t;
#else
MOVSINPACKED (
typedef struct {
        uint32_t BlankStart;                                                    // 0x00 0x00 0x00 0x00
        uint8_t BlankStart1;                                                    // 0x00
        uint8_t SOH;                                                            // MOVSIN_SOH start of header
        uint16_t sendAddrAscii;                                                 // 2 byte ASCII Sending address, appointed “FF” as pc address, “00” is as broadcast address which can’t be as sender address
        uint16_t recvAddrAscii;                                                 // 2byte ASCII Receiving address, “00” is broadcast address, all sign will receive data; “FF” appoint pc fixed address which is used when sign return data to pc. “?” is wildcard character, “1?” allows the signs between “10” to “1F” to receive data.
        uint8_t STX;                                                            // MOVSIN_STX start of transmission characture
        uint8_t cmdAscii;                                                       // moveSign_Cmd_e Command code, 1-byte ASCII character shows different functions. Command code Description ‘A’ Write text file command ‘C’ Write variable command ‘E’ Write graphics file command ‘W’ Write special function command ‘R’ Read special function command
        uint8_t Buf[];                                                          // data packet variable length
        uint8_t ETX;                                                            // MOVSIN_ETX End of TeXt
        uint32_t CheckSumAscii;                                                 // 4 byte ASCII
        uint8_t EOT;                                                            // MOVSIN_EOT End of Text
}) movSign_Send_t;
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif