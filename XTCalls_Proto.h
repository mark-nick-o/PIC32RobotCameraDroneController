#ifndef __XTCalls_proto_
#define __XTCalls_proto_
/* -----------------------------------------------------------------------------
   %%%%%%%%%%%%%%%%%%% EXT CALLS PROTOCOL SHENZEN TOPSHINE TECH %%%%%%%%%%%%%%%
   
    Written By ACP Aviation
    Allows Display of values to advertising board / Bus Display etc
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
#include "definitions.h"
#include <stdint.h>
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
  #define XTCALLPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define XTCALLPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define XTCALLPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define XTCALLPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define XTCALLPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define XTCALL_START_MSG 0xA5u
#define XTCALL_ESCAPE_MSG 0xAAu
#define XTCALL_END_MSG 0xAEu

#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t StartCode;                                                      // Start code 0xa5
        uint8_t PacketType;                                                     // 0x68
        uint8_t CardType;                                                        // 0x32
        uint8_t CardID;
        uint8_t ProtocolCode;                                                   // 0X7B
        uint8_t additionConfirm;                                                // bits 0 or 1
        uint16_t PacketLen;                                                     // packet length
        uint8_t PacketNum;
        uint8_t LastPacketNum;
        uint8_t PacketData[];
        uint16_t checkSum;                                                      // 16 bit summation of the bytes
        uint8_t EndCode;                                                        // 0xae
} xtCalls_Send_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t StartCode;                                                     // Start code 0xa5
        uint8_t PacketType;                                                    // 0x68
        uint8_t CardType;                                                        // 0x32
        uint8_t CardID;
        uint8_t ProtocolCode;                                                   // 0X7B
        uint8_t additionConfirm;                                                // bits 0 or 1
        uint16_t PacketLen;                                                     // packet length
        uint8_t PacketNum;
        uint8_t LastPacketNum;
        uint8_t PacketData[];
        uint16_t checkSum;                                                      // 16 bit summation of the bytes
        uint8_t EndCode;                                                        // 0xae
}) xtCalls_Send_t;
#endif

#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t StartCode;                                                     // Start code 0xa5
        uint8_t PacketType;                                                    // 0xE8 = (0x68 | 0x80), for the app 3.2 or below return 0x68, app 3.3 or above return 0xe8 to same as other protocol
        uint8_t CardType;                                                        // 0x32
        uint8_t CardID;
        uint8_t ProtocolCode;                                                   // 0x7B
        uint8_t ReturnVal;                                                      // RR = 0x00: that successful;R R = 0x01 ~ 0xFF: that the failure error code.  (0x01: checksum error)  (0x02: packet sequence error)  (Other: to be confirmed)
        uint16_t PacketLen;                                                     // packet length
        uint8_t PacketNum;
        uint8_t LastPacketNum;
        uint8_t PacketData[];
        uint16_t checkSum;                                                      // 16 bit summation of the bytes
        uint8_t EndCode;                                                        // 0xae
} xtCalls_Rcv_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t StartCode;                                                     // Start code 0xa5
        uint16_t PacketType;                                                    // 0xE8 = (0x68 | 0x80), for the app 3.2 or below return 0x68, app 3.3 or above return 0xe8 to same as other protocol
        uint8_t CardType;                                                        // 0x32
        uint8_t CardID;
        uint8_t ProtocolCode;                                                   // 0x7B
        uint8_t ReturnVal;                                                      // RR = 0x00: that successful;R R = 0x01 ~ 0xFF: that the failure error code.  (0x01: checksum error)  (0x02: packet sequence error)  (Other: to be confirmed)
        uint16_t PacketLen;                                                     // packet length
        uint8_t PacketNum;
        uint8_t LastPacketNum;
        uint8_t PacketData[];
        uint16_t checkSum;                                                      // 16 bit summation of the bytes
        uint8_t EndCode;                                                        // 0xae
}) xtCalls_Rcv_t;
#endif

#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t NullByte;                                                       // Start code 0x00
        uint16_t Identify;                                                      // Set to "I1"?
        uint16_t width;                                                         // The width of the picture, low byte previous(little endian)
        uint16_t height;                                                        // The height of the picture,low byte previous(little endian)
        uint8_t property_red : 1u;                                              // Weather exist red data, exist when 1.
        uint8_t property_green : 1u;                                            // Weather  exist green data, exist when 1..
        uint8_t property_blue : 1u;                                             // Whether exist blue data, exist when 1.
        uint8_t property_reszero : 1u;                                          // reserved set to zero
        uint8_t property_greyscale : 4u;                                        // Gray-scale, only support 0 and 7 now.  0: 2 levels gray, Each lattic data use 1 bit.7: 256 levels gray, Each lattic data use 8 bit. Each line of the picture data is aligned by byte. As for 2 levels gray picture, when the line data is not enough for 8 bit, add 0.
        uint8_t ReservedSet0;                                                   // set 0
} xtCalls_SimplePicData_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t NullByte;                                                       // Start code 0x00
        uint16_t Identify;                                                      // Set to "I1"?
        uint16_t width;                                                         // The width of the picture, low byte previous(little endian)
        uint16_t height;                                                        // The height of the picture,low byte previous(little endian)
        uint8_t property_red : 1u;                                              // Weather exist red data, exist when 1.
        uint8_t property_green : 1u;                                            // Weather  exist green data, exist when 1..
        uint8_t property_blue : 1u;                                             // Whether exist blue data, exist when 1.
        uint8_t property_reszero : 1u;                                          // reserved set to zero
        uint8_t property_greyscale : 4u;                                        // Gray-scale, only support 0 and 7 now.  0: 2 levels gray, Each lattic data use 1 bit.7: 256 levels gray, Each lattic data use 8 bit. Each line of the picture data is aligned by byte. As for 2 levels gray picture, when the line data is not enough for 8 bit, add 0.
        uint8_t ReservedSet0;                                                   // set 0
}) xtCalls_SimplePicData_t;
#endif

#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t ColotFontSize;                                                  // Text Color and Font size
        uint8_t TextEncodingHi;                                                 // High byte of the text encoding. For single-byte characters, the value is 0.
        uint8_t TextEncodingLo;                                                 // Low byte of the text encoding. For single-byte characters, the value of its ASCII code.
} xtCalls_RichText2_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t ColotFontSize;                                                  // Text Color and Font size
        uint8_t TextEncodingHi;                                                 // High byte of the text encoding. For single-byte characters, the value is 0.
        uint8_t TextEncodingLo;                                                 // Low byte of the text encoding. For single-byte characters, the value of its ASCII code.
}) xtCalls_RichText2_t;
#endif

#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t WindowNumber;                                                   // The window should be divided into the number of valid value 1 ~ 8
        uint16_t WinXCoordXHXL;                                                 // Window x-coordinate, high byte in the former
        uint16_t WinYCoordYHYL;                                                 // Window y-coordinate, high byte in the former
        uint16_t WidthWinWHWL;                                                  // The width of the window, high byte in the former
        uint16_t HtWinHHHL;                                                     // The height of the window, high byte in the former
} xtCalls_Window_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t WindowNumber;                                                   // The window should be divided into the number of valid value 1 ~ 8
        uint16_t WinXCoordXHXL;                                                 // Window x-coordinate, high byte in the former
        uint16_t WinYCoordYHYL;                                                 // Window y-coordinate, high byte in the former
        uint16_t WidthWinWHWL;                                                  // The width of the window, high byte in the former
        uint16_t HtWinHHHL;                                                     // The height of the window, high byte in the former
}) xtCalls_Window_t;
#endif



// define the OPCODES contents of CC
#define XCAll_DIV_WINDOW 0x01u                                                  // Division of display window (area)
#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t CC;                                                             // Note This command is divided into display window (area) 0x01
        xtCalls_Window_t Win[MAX_NUM_OF_WINS];                                  // Window x-coordinate, high byte in the former
} xtCalls_WinCmd_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t CC;                                                             // Note This command is divided into display window (area) 0x01
        xtCalls_Window_t Win[MAX_NUM_OF_WINS];                                  // Window x-coordinate, high byte in the former
}) xtCalls_WinCmd_t;
#endif
#define XCAll_TXT2_WINDOW 0x02u                                                 // To send text data to a specified window
#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t CC;                                                             // Note This command is divided into display window (area) 0x02
        uint8_t winNo;                                                          // The window sequence number, valid values 0 ~ 7.
        uint8_t Mode;                                                           // valid modes are shown below e.g. XCallDraw
        uint8_t Alignment;                                                      // 0: Left-aligned 1: Horizontal center 2: Right-aligned
        uint8_t Speed;                                                          // 0-100 smaller the faster
        uint16_t StayTime;                                                      // seconds 0xffff
        xtCalls_RichText2_t stringEncoded[];                                    // Every 3 bytes to represent a character, end with 3 bytes of 0. Refer to Rich3 text
} xtCalls_writeTxt2Win_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t CC;                                                             // Note This command is divided into display window (area) 0x02
        uint8_t winNo;                                                          // The window sequence number, valid values 0 ~ 7.
        uint8_t Mode;                                                           // valid modes are shown below e.g. XCallDraw
        uint8_t Alignment;                                                      // 0: Left-aligned 1: Horizontal center 2: Right-aligned
        uint8_t Speed;                                                          // 0-100 smaller the faster
        uint16_t StayTime;                                                      // seconds 0xffff
        xtCalls_RichText2_t stringEncoded[];                                    // Every 3 bytes to represent a character, end with 3 bytes of 0. Refer to Rich3 text
}) xtCalls_writeTxt2Win_t;
#endif
#define XCAll_IMG2_WINDOW 0x03u                                                 // To send image data to the specified window
#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t CC;                                                             // Note This command is divided into display window (area) 0x03
        uint8_t winNo;                                                          // The window sequence number, valid values 0 ~ 7.
        uint8_t Mode;                                                           // valid modes are shown below e.g. XCallDraw
        uint8_t Alignment;                                                      // 0: Left-aligned 1: Horizontal center 2: Right-aligned
        uint8_t Speed;                                                          // 0-100 smaller the faster
        uint16_t StayTime;                                                      // seconds 0xffff
        uint8_t imgDataFormat;                                                  // 0x01: gif image file format 0x02: gif image file references. 0x03: picture package picture reference. 0x04: simple image format.
        uint8_t imgDispayPosnX;                                                 // X co-ordinate of image display position
        uint8_t imgDispayPosnY;                                                 // Y co-ordinate of image display position
        uint8_t imgData[];                                                      // According to "image data format" is defined to determine the meaning of the data. Image data format is 0x01: gif image file of the actual data, which contains the image width, height and other information; Image data format is 0x02: the gif image file name stored in  the control card. Image data format is 0x03: The image package file name and image number that stored in the controller. The middle separated by spaces. For example, "images.rpk 1" Image data format is 0x04?Simple picture data, see the description format.
} xtCalls_drawImg2Win_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t CC;                                                             // Note This command is divided into display window (area) 0x03
        uint8_t winNo;                                                          // The window sequence number, valid values 0 ~ 7.
        uint8_t Mode;                                                           // valid modes are shown below e.g. XCallDraw
        uint8_t Alignment;                                                      // 0: Left-aligned 1: Horizontal center 2: Right-aligned
        uint8_t Speed;                                                          // 0-100 smaller the faster
        uint16_t StayTime;                                                      // seconds 0xffff
        uint8_t imgDataFormat;                                                  // 0x01: gif image file format 0x02: gif image file references. 0x03: picture package picture reference. 0x04: simple image format.
        uint8_t imgDispayPosnX;                                                 // X co-ordinate of image display position
        uint8_t imgDispayPosnY;                                                 // Y co-ordinate of image display position
        uint8_t imgData[];                                                      // According to "image data format" is defined to determine the meaning of the data. Image data format is 0x01: gif image file of the actual data, which contains the image width, height and other information; Image data format is 0x02: the gif image file name stored in  the control card. Image data format is 0x03: The image package file name and image number that stored in the controller. The middle separated by spaces. For example, "images.rpk 1" Image data format is 0x04?Simple picture data, see the description format.
}) xtCalls_drawImg2Win_t;
#endif
#define XCAll_STXT2_WINDOW 0x04u                                                // Static text data sent to the specified window
#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t CC;                                                             // Note This command is divided into display window (area) 0x04
        uint8_t winNo;                                                          // The window sequence number, valid values 0 ~ 7.
        uint8_t dataType;                                                       // simple text 0x01
        uint8_t Alignment;                                                      // 0: Left-aligned 1: Horizontal center 2: Right-aligned
        uint8_t imgDispayPosnX;                                                 // X co-ordinate of text image display position
        uint8_t imgDispayPosnY;                                                 // Y co-ordinate of text image display position
        uint8_t imgDispayWidth;                                                 // width image text display position
        uint8_t imgDispayHeight;                                                // height of image text display position
        uint8_t font;                                                           // Bit0~3: font size Bit4~6: font style Bit7: Reserved
        uint8_t TxtColorRed;                                                    // color as RGB components
        uint8_t TxtColorGreen;
        uint8_t TxtColorBlue;
        unsigned char textMsg[];                                                // text message to display as the image
} xtCalls_statTxt2Win_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t CC;                                                             // Note This command is divided into display window (area) 0x04
        uint8_t winNo;                                                          // The window sequence number, valid values 0 ~ 7.
        uint8_t dataType;                                                       // simple text 0x01
        uint8_t Alignment;                                                      // 0: Left-aligned 1: Horizontal center 2: Right-aligned
        uint8_t imgDispayPosnX;                                                 // X co-ordinate of text image display position
        uint8_t imgDispayPosnY;                                                 // Y co-ordinate of text image display position
        uint8_t imgDispayWidth;                                                 // width image text display position
        uint8_t imgDispayHeight;                                                // height of image text display position
        uint8_t font;                                                           // Bit0~3: font size Bit4~6: font style Bit7: Reserved
        uint8_t TxtColorRed;                                                    // color as RGB components
        uint8_t TxtColorGreen;
        uint8_t TxtColorBlue;
        unsigned char textMsg[];                                                // text message to display as the image
}) xtCalls_statTxt2Win_t;
#endif
#define XCAll_CLOCK2_WINDOW 0x05                                                // To send clock data to the specified window
#define XCAll_EXIT_WINDOW 0x06                                                  // Exit show to return to play within the program
#define XCAll_SAVE_CLEAR 0x07                                                   // Save / clear the data
#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t CC;                                                             // Note This command is global 0x07
        uint8_t action;                                                         // 0x00: save data to flash. 0x01: Clear flash data
        uint16_t reserved;
} xtCalls_saveClear_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t CC;                                                             // Note This command is global 0x07
        uint8_t action;                                                         // 0x00: save data to flash. 0x01: Clear flash data
        uint16_t reserved;
}) xtCalls_saveClear_t;
#endif
#define XCAll_PLAY_PRG 0x08u                                                     // Select play stored program (single-byte)
#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t CC;                                                             // Note This command global 0x08
        uint8_t options;                                                        // Bit0: Whether to save select play message to flash. 0 not to save?1 save? Bit1~7: Reserved, set to 0
        uint8_t ProgNo;                                                         // the program to play 0 will exit mode 0-255
        uint8_t ProgNoTable[];                                                  // 0-255 The program number list,1 byte for  each program
} xtCalls_playProg_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t CC;                                                             // Note This command is global 0x08
        uint8_t options;                                                        // Bit0: Whether to save select play message to flash. 0 not to save?1 save? Bit1~7: Reserved, set to 0
        uint8_t ProgNo;                                                         // the program to play 0 will exit mode 0-255
        uint8_t ProgNoTable[];                                                  // 0-255 The program number list,1 byte for  each program
}) xtCalls_playProg_t;
#endif
#define XCAll_PLAY_DPRG 0x09u                                                    // Select play t stored program (double-byte)
#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t CC;                                                             // Note This command is global 0x09
        uint8_t options;                                                        // Bit0: Whether to save select play message to flash. 0 not to save?1 save? Bit1~7: Reserved, set to 0
        uint16_t ProgNo;                                                        // the program to play 0 will exit mode 0-512
        uint16_t ProgNoTable[];                                                 // 0-65535 The program number list,1 byte for  each program
} xtCalls_playDProg_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t CC;                                                             // Note This command is global 0x09
        uint8_t options;                                                        // Bit0: Whether to save select play message to flash. 0 not to save?1 save? Bit1~7: Reserved, set to 0
        uint16_t ProgNo;                                                        // the program to play 0 will exit mode 0-512
        uint16_t ProgNoTable[];                                                 // 0-65535 The program number list,1 byte for  each program
}) xtCalls_playDProg_t;
#endif
#define XCAll_SET_VAR 0x0a                                                      // Set variable value
#define XCAll_PLAY_SET 0x0b                                                     // Select play single stored program, and set the variable value
#define XCAll_SET_DISP_AREA 0x0c                                                // Set global display area
#define XCAll_PUSH_DATA 0x0d                                                    // Push user variable data
#define XCAll_SET_TIM 0x0e                                                      // Set timer control
#define XCAll_SET_AREA_VAL 0x0f                                                 // Set the global display area and variable values
#define XCAll_FULL_SCRN_MSG 0x11u                                               // Send full screen instant message
#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t CC;                                                             // Note This command for full area screen 0x11
        uint8_t playTime;                                                       // Value 0 means play the message until a new instruction.
        uint8_t mode;                                                           // as per mode list
        uint8_t Alignment;                                                      // 0: Left-aligned 1: Horizontal center 2: Right-aligned
        uint8_t Speed;                                                          // 0-100 smaller the faster
        uint16_t StayTime;                                                      // seconds 0xffff
        uint8_t font;                                                           // Bit0~3: font size Bit4~6: font style Bit7: Reserved
        uint8_t TxtColorRed;                                                    // color as RGB components
        uint8_t TxtColorGreen;
        uint8_t TxtColorBlue;
        unsigned char textMsg[];                                                // text message to display as the image
} xtCalls_statTxtFullScrn_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t CC;                                                             // Note This command is for full area screen 0x11
        uint8_t playTime;                                                       // Value 0 means play the message until a new instruction.
        uint8_t mode;                                                           // as per mode list
        uint8_t Alignment;                                                      // 0: Left-aligned 1: Horizontal center 2: Right-aligned
        uint8_t Speed;                                                          // 0-100 smaller the faster
        uint16_t StayTime;                                                      // seconds 0xffff
        uint8_t font;                                                           // Bit0~3: font size Bit4~6: font style Bit7: Reserved
        uint8_t TxtColorRed;                                                    // color as RGB components
        uint8_t TxtColorGreen;
        uint8_t TxtColorBlue;
        unsigned char textMsg[];                                                // text message to display as the image
}) xtCalls_statTxtFullScrn_t;
#endif
#define XCAll_PURE_TXT_WIN 0x12u                                                 // Send pure text to the specified window
#if defined(D_FT900)
typedef struct XTCALLPACKED {
        uint8_t CC;                                                             // Note This command is divided into display window (area) 0x12
        uint8_t winNo;                                                          // The window sequence number, valid values 0 ~ 7.
        uint8_t Alignment;                                                      // 0: Left-aligned 1: Horizontal center 2: Right-aligned
        uint8_t Speed;                                                          // 0-100 smaller the faster
        uint16_t StayTime;                                                      // seconds 0xffff
        uint8_t font;                                                           // Bit0~3: font size Bit4~6: font style Bit7: Reserved
        uint8_t TxtColorRed;                                                    // color as RGB components
        uint8_t TxtColorGreen;
        uint8_t TxtColorBlue;
        unsigned char textMsg[];                                                // text message to display as the image
} xtCalls_pureTxt2Win_t;
#else
XTCALLPACKED (
typedef struct {
        uint8_t CC;                                                             // Note This command is divided into display window (area) 0x12
        uint8_t winNo;                                                          // The window sequence number, valid values 0 ~ 7.
        uint8_t Alignment;                                                      // 0: Left-aligned 1: Horizontal center 2: Right-aligned
        uint8_t Speed;                                                          // 0-100 smaller the faster
        uint16_t StayTime;                                                      // seconds 0xffff
        uint8_t font;                                                           // Bit0~3: font size Bit4~6: font style Bit7: Reserved
        uint8_t TxtColorRed;                                                    // color as RGB components
        uint8_t TxtColorGreen;
        uint8_t TxtColorBlue;
        unsigned char textMsg[];                                                // text message to display as the image
}) xtCalls_pureTxt2Win_t;
#endif
#define XCAll_SEND_MULTI_DATA 0x60u                                             // Send multi data message
#define XCAll_SEND_PROG_TEMPLATE 0x81u                                          // Send program template message
#define XCAll_TOGGLE_PROG_TEMPLATE 0x82u                                        // enter or exit program template mode message
#define XCAll_QERY_PROG_STAT 0x83u                                              // query program status message
#define XCAll_DEL_PROG 0x84u                                                    // delete program status message
#define XCAll_SND_TXT2_PROG 0x85u                                               // send text to program message
#define XCAll_SND_PIC2_PROG 0x86u                                               // send picture to program message
#define XCAll_SND_CLOCK_PROG 0x87u                                               // send clock to program message
#define XCAll_SND_ALONE2_PROG 0x88u                                             // send alone to program message
#define XCAll_QERY_PROG_INFO 0x89u                                              // query to program message
#define XCAll_SET_PROG_PROP 0x8Au                                               // set program property
#define XCAll_SET_PLAY_PLAN 0x8Bu                                               // set play plan property message
#define XCAll_DEL_PLAY_PLAN 0x8Cu                                               // delete play plan property message
#define XCAll_QERY_PLAY_PLAN 0x8Du                                              // query play plan property message

typedef enum
{
  XTFONT_SZ_8=0,
  XTFONT_SZ_12=1,
  XTFONT_SZ_16=2,
  XTFONT_SZ_24=3,
  XTFONT_SZ_32=4,
  XTFONT_SZ_40=5,
  XTFONT_SZ_48=6,
  XTFONT_SZ_56=7,
} XTCAlls_FontInPixel_e;

typedef enum
{
  XTFONT_ST1=0,
  XTFONT_ST2=1,
  XTFONT_ST3=2,
  XTFONT_ST4=3,
  XTFONT_ST5=4,
  XTFONT_ST6=5,
  XTFONT_ST7=6,
  XTFONT_ST8=7,
} XTCAlls_FontStyle_e;

#define XT_TEXT_COLOR_RED (1u<<0u)
#define XT_TEXT_COLOR_GREEN (1u<<1u)
#define XT_TEXT_COLOR_YELLOW 3u
#define XT_TEXT_COLOR_BLUE (1u<<2u)
#define ZT_TEXT_COLOR_PURPLE 5u
#define XT_TEXT_COLOR_CYAN 6u
#define XT_TEXT_COLOR_WHITE 7u

#define RichTextByte1(FontPixel,Color) { (((FontPixel&0x0FU)<<4u) | (Color&0x0Fu)) }

typedef enum
{
  XT_EFFECT_CENTER=0,
  XT_EFFECT_ZOOM=1,
  XT_EFFECT_STRETCH=2,
  XT_EFFECT_TILE=3,
} XTCAlls_PicEffect_e;
#define XCallCodeEffect 0                                                       /* special effect types */
/* valid MODE types */
#define XCallDraw 0
#define XCall1Openfromleft 1
#define XCallOpenfromright 2
#define XCallOpenfromcenterHoriz 3
#define XCallOpenfromcenterVertical 4
#define XCall5Shuttervertical 5
#define XCallMovetoleft 6
#define XCallMovetoright 7
#define XCallMoveup 8
#define XCallMovedown 9
#define XCallScrollup 10
#define XCallScrolltoleft 11
#define XCallScrolltoright 12
#define XCallFlicker 13
#define XCallContinuousscrolltoleft 14
#define XCallContinuousscrolltoright 15
#define XCallShutterhorizontal 16
#define XCallClockwiseopenout 17
#define XCallAnticlockwiseopenout 18
#define XCallWindmill 19
#define XCallWindmillanticlockwise 20
#define XCallRectangleforth 21
#define XCallRectangleentad 22
#define XCallQuadrangleforth 23
#define XCallQuadrangleendtad 24
#define XCallCircleforth 25
#define XCallCircleendtad 26
#define XCallOpenoutfromleftupcorner 27
#define XCallOpenoutfromrightupcorner 28
#define XCallOpenoutfromleftbottomcorner 29
#define XCallOpenoutfromrightbottomcorner 30
#define XCallBevelopenout 31
#define XCallAntiBevelopenout 32
#define XCallEnterintofromleftupcorner 33
#define XCallEnterintofromrightupcorner 34
#define XCallEnterintofromleftbottomcorner 35
#define XCallEnterintofromlowerrightcorner 36
#define XCallBevelenterinto 37
#define XCallAntiBevelenterinto 38
#define XCallHorizontalzebracrossing 39
#define XCallVerticalzebracrossing 40
#define XCallMosaicbig 41
#define XCallMosaicsmall 42
#define XCallRadiationup 43
#define XCallRadiationdownwards 44
#define XCallAmass 45
#define XCallDrop 46
#define XCallCombinationHorizontal 47
#define XCallCombinationVertical 48
#define XCallBackout 49
#define XCallScrewingin 50
#define XCallChessboardhorizontal 51
#define XCallChessboardvertical 52
#define XCallContinuousscrollup 53
#define XCallContinuousscrolldown 54
#define XCallReserved 55
#define XCallReserved1 56
#define XCallGradualbiggerup 57
#define XCallGradualsmallerdown 58
#define XCallReserved2 59
#define XCallGradualbiggervertical 60
#define XCallFlickerhorizontal 61
#define XCallFlickervertical 62
#define XCallSnow 63
#define XCallScrolldown 64
#define XCallScrollfromlefttoright 65
#define XCallOpenoutfromtoptobottom 66
#define XCallSectorexpand 67
#define XCallReserved3 68
#define XCallZebracrossinghorizontal 69
#define XCallZebracrossingVertical 70

#define XCall_CC_Division_of_display_window_area 0x01u                      /* sub commands */
#define XCall_CC_To_send_text_data_to_a_specified_window 0x02u
#define XCall_CC_To_send_image_data_to_the_specified_window 0x03u
#define XCall_CC_Static_text_data_sent_to_the_specified_window 0x04u
#define XCall_CC_To_send_clock_data_to_the_specified_window 0x05u
#define XCall_CC_Exit_show_to_return_to_play_within_the_program 0x06u
#define XCall_CC_Save_clear_the_data 0x07u
#define XCall_CC_Select_play_stored_program_singlebyte 0x08u
#define XCall_CC_Select_play_t_stored_program_doublebyte 0x09u
#define XCall_CC_Set_variable_value 0x0au
#define XCall_CC_Select_play_single_stored_program_and_set_the_value 0x0bu
#define XCall_CC_Set_global_display_area 0x0cu
#define XCall_CC_Push_user_variable_data 0x0du
#define XCall_CC_Set_timer_control 0x0eu
#define XCall_CC_Set_the_global_display_area_and_variables 0x0fu
#define XCall_CC_Send_full_screen_instant_message 0x11u
#define XCall_CC_Send_pure_text_to_the_specified_window 0x12u
#define XCall_CC_Set_program_template_command 0x81u
#define XCall_CC_In_or_out_program_template_command 0x82u
#define XCall_CC_Query_program_template_command 0x83u
#define XCall_CC_Delete_program_command 0x84u
#define XCall_CC_Send_text_to_special_window 0x85u
#define XCall_CC_Send_picture_to_special_window 0x86u
#define XCall_CC_Clock_temperature_display_in_the_specified_window 0x87u
#define XCall_CC_Send_alone_program 0x88u
#define XCall_CC_Query_Prog_Info 0x89u
#define XCall_CC_Set_program_property 0x8au
#define XCall_CC_Set_play_plan 0x8bu
#define XCall_CC_Delete_play_plan 0x8cu
#define XCall_CC_Query_play_plan 0x8du

extern uint8_t encodeXCallPayload( uint8_t * p_message, uint8_t n_bytes);
extern uint8_t decodeXCallPayload( uint8_t const * const p_message, uint8_t n_bytes, uint8_t * o_message );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif