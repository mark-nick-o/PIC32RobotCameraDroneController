#ifndef __ALM_DEF_STRUTS__                                                    // This file is to be included if not already
#define __ALM_DEF_STRUTS__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//    AlarmDefinitons.h : to define you're alarm pages
//
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define ALARMPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define ALARMPACKED __attribute__((packed)) ALIGNED(1)                        /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define ALARMPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define ALARMPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define ALARMPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define ALARM_LINES_PER_PAGE 16                                                 // We define 16 on a page
#define ALARM_PAGE_COUNT 1                                                      // One page
#define TOTAL_NO_OF_ALARMS 104                                                  // TOTAL number of possible alarms  (max uint8_t = 255)
#define ALARM_WORD_TOTAL 8                                                      // The maximum number of alarms

#if defined(D_FT900)
typedef struct SBGCPACKED {
  unsigned char Caption[];                                                      // What it says
  uint32_t CapVisible : 1;                                                      // Can the caption be seen
  uint32_t CapSpare : 7;
  uint32_t CapColor : 24;                                                       // color of the text object

  uint32_t AckVisible : 1;
  uint32_t AckActive : 1;
  uint32_t AckSpare : 6;
  uint32_t AckColor : 24;
} AlarmLineObject_t;                                                           // the attributes of each alarm line on the page
#else
ALARMPACKED (
typedef struct{                                                                 // Structure of each alarm line object on the HMI
  unsigned char Caption[];                                                      // What it says
  uint32_t CapVisible : 1;                                                      // Can the caption be seen
  uint32_t CapSpare : 7;
  uint32_t CapColor : 24;                                                       // color of the text object
  
  uint32_t AckVisible : 1;
  uint32_t AckActive : 1;
  uint32_t AckSpare : 6;
  uint32_t AckColor : 24;
}) AlarmLineObject_t;                                                           // the attributes of each alarm line on the page
#endif

#if defined(D_FT900)
typedef struct SBGCPACKED {
   uint8_t AlarmStack[TOTAL_NO_OF_ALARMS];                                      // define it as maximum number possible of alarms
   uint8_t numObjects;                                                          // number of objects in the stack
} AlarmStack_t;                                                                 // the attributes of alarm stack 16 lines per page
#else
ALARMPACKED (
typedef struct{                                                                 // queue of most recent alarms / events in order of occurance
   uint8_t AlarmStack[TOTAL_NO_OF_ALARMS];                                      // define it as maximum number possible of alarms
   uint8_t numObjects;                                                          // number of objects in the stack
}) AlarmStack_t;                                                                // the attributes of alarm stack 16 lines per page
#endif

#if defined(D_FT900)
typedef struct SBGCPACKED {
  unsigned char Caption[ALARM_WORD_TOTAL][150];                                 // Storage of the descriptions for each alarm in the alarm word object
  uint8_t objectNo[ALARM_WORD_TOTAL];                                           // The id (tag) for each alarm
} AlarmDefinitionObject_t;                                                      // The definitions of the alarms
#else
ALARMPACKED (
typedef struct{                                                                 // Define each alarm in the alarm word with a description for the GUI
  unsigned char Caption[ALARM_WORD_TOTAL][150];                                 // Storage of the descriptions for each alarm in the alarm word object
  uint8_t objectNo[ALARM_WORD_TOTAL];                                           // The id (tag) for each alarm
}) AlarmDefinitionObject_t;                                                    // The definitions of the alarms
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif