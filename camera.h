#ifndef __camera_h
#define __camera_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//    camera.h : various camera protocol definitions
//
//
//    Version : @(#) 1.2
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
//    Support the StreamCorder and XStream API (either serial or encapsulated over UDP to the AIR COMPIC)
//
//    Rev 1.0 Advanced Micro Peripherals Serial Protocol (encoders / decoders)
//    Rev 1.1 Run Cam protocol
//    Rev 1.2 Yi Action Cam protocol
//    Rev 1.3 Yi Parrot Cam protocol
//    Rev 1.4 SubC Rayfin Cam protocol
//
#include "definitions.h"                                                        // Common data for compiler
#include "ports.h"

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define CAMPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define CAMPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define CAMPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define CAMPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define CAMPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

/* ============== Rayfin Underwater Marine Camera SubC ====================== */
#if defined(RAYFIN_WATER_CAM_USED)

#define API_RAYF_START_REC(X)  do{ strcpy(X,"\\0e\\00\\00\\00StartRecording"); } while(0)  /* Start Recording */
#define API_RAYF_STOP_REC(X)  do{ strcpy(X,"\\0d\\00\\00\\00StopRecording"); } while(0)   /* Stop Recording */
#define API_RAYF_TAK_PIC(X)  do{ strcpy(X,"\\0a\\00\\00\\00TakePicture"); } while(0)      /* Take Picture */
#define API_RAYF_TAK_STIL(X)  do{ strcpy(X,"\\08\\00\\00\\00TakeStill"); } while(0)        /* Take Still */
#define API_RAYF_STRT_CONT(X)  do{ strcpy(X,"\\09\\00\\00\\00StartContinuous"); } while(0) /* Start Continuos */
#define API_RAYF_STOP_CONT(X)  do{ strcpy(X,"\\0e\\00\\00\\00StopContinuous"); } while(0) /* Stop Continuos */
#define API_RAYF_REBOOT(X)  do{ strcpy(X,"\\06\\00\\00\\00Reboot"); } while(0)            /* Reboot */
#define API_RAYF_IS_REC(X)  do{ strcpy(X,"\\0b\\00\\00\\00IsRecording"); } while(0)       /* Is Recording */
#define API_RAYF_REC_TIM(X)  do{ strcpy(X,"\\11\\00\\00\\00RecordingDuration"); } while(0) /* Recording Duration */

#define API_RAYF_START_REC1(X)  do{ sprintf(X,"\\0f\\00\\00\\00StartRecording%c",0x0au); } while(0)  /* Start Recording */
#define API_RAYF_STOP_REC1(X)  do{ sprintf(X,"\\0e\\00\\00\\00StopRecording%c",0x0au); } while(0)   /* Stop Recording */
#define API_RAYF_TAK_PIC1(X)  do{ sprintf(X,"\\0b\\00\\00\\00TakePicture%c",0x0au); } while(0)      /* Take Picture */
#define API_RAYF_TAK_STIL1(X)  do{ sprintf(X,"\\t\\00\\00\\00TakeStill%c",0x0au); } while(0)        /* Take Still */
#define API_RAYF_STRT_CONT1(X)  do{ sprintf(X,"\\10\\00\\00\\00StartContinuous%c",0x0au); } while(0) /* Start Continuos */
#define API_RAYF_STOP_CONT1(X)  do{ sprintf(X,"\\0f\\00\\00\\00StopContinuous%c",0x0au); } while(0) /* Stop Continuos */
#define API_RAYF_REBOOT1(X)  do{ sprintf(X,"\\07\\00\\00\\00Reboot%c",0x0au); } while(0)            /* Reboot */
#define API_RAYF_IS_REC1(X)  do{ sprintf(X,"\\0c\\00\\00\\00IsRecording%c",0x0au); } while(0)       /* Is Recording */
#define API_RAYF_REC_TIM1(X)  do{ sprintf(X,"\\12\\00\\00\\00RecordingDuration%c",0x0au); } while(0) /* Recording Duration */

/* looking at the documents some show 0x0e as the start length i.e. 14 while the above shows 15 0x0f also the 0x0a LF at the end is ommitted on some
   please check it when you test it you can use the binary as a const char def to send if you want that way */
#define API_RAYF_HSTART_REC  {0x0eu,0x00u,0x00u,0x00u,0x53u,0x74u,0x61u,0x72u,0x74u,0x52u,0x65u,0x63u,0x6fu,0x72u,0x64u,0x69u,0x6eu,0x67u,0x0au}  /* Start Recording */
#define API_RAYF_HSTOP_REC  {0x0du,0x00u,0x00u,0x00u,0x53u,0x74u,0x6fu,0x70u,0x52u,0x65u,0x63u,0x6fu,0x72u,0x64u,0x69u,0x6eu,0x67u,0x0au}   /* Stop Recording */
#define API_RAYF_HTAK_PIC  {0x0au,0x00u,0x00u,0x00u,0x54u,0x61u,0x6bu,0x65u,0x50u,0x69u,0x63u,0x74u,0x75u,0x72u,0x65u,0x0au} /* Take Picture */
#define API_RAYF_HTAK_STIL  {0x08u,0x00u,0x00u,0x00u,0x54u,0x61u,0x6bu,0x65u,0x53u,0x74u,0x69u,0x6cu,0x6cu,0x0au} /* Take Still */
#define API_RAYF_HSTRT_CONT  {0x09u,0x00u,0x00u,0x00u,0x53u,0x74u,0x61u,0x72u,0x74u,0x43u,0x6fu,0x6eu,0x74u,0x69u,0x6eu,0x75u,0x6fu,0x75u,0x73u,0x0au} /* Start Continuos */
#define API_RAYF_HSTOP_CONT  {0x0eu,0x00u,0x00u,0x00u,0x53u,0x74u,0x6fu,0x70u,0x43u,0x6fu,0x6eu,0x74u,0x69u,0x6eu,0x75u,0x6fu,0x75u,0x73u,0x0au}  /* Stop Continuos */
#define API_RAYF_HREBOOT  {0x06u,0x00u,0x00u,0x00u,0x52u,0x65u,0x62u,0x6fu,0x6fu,0x74u,0x0au} /* Reboot */
#define API_RAYF_HIS_REC  {0x0bu,0x00u,0x00u,0x00u,0x49u,0x73u,0x52u,0x65u,0x63u,0x6fu,0x72u,0x64u,0x69u,0x6eu,0x67u,0x0au} /* Is Recording */
#define API_RAYF_HREC_TIM  {0x11u,0x00u,0x00u,0x00u,0x52u,0x65u,0x63u,0x6fu,0x72u,0x64u,0x69u,0x6eu,0x67u,0x44u,0x75u,0x72u,0x61u,0x74u,0x69u,0x6fu,0x6eu,0x0au} /* Recording Duration */

typedef enum { LiquidOptics=0, UltraOptics=1, ZoomLens=2 } rayfin_lens_type_e;  /* types of lens types that may be returned with LensType */

/* --------------------- Accessories ---------------------------------------- */
/* ===================== Aquorea Strobe ===================================== */
#define API_AqoR_IS_ENAB(X)  do{ strcpy(X,"\\0f\\00\\00\\00IsStrobeEnabled"); } while(0)  /* query if stobe is enabled */
#define API_AqoR_IS_ENAB1(X)  do{ sprintf(X,"\\10\\00\\00\\00IsStrobeEnabled%c",0x0au); } while(0)  /* query if stobe is enabled */
#define API_AqoR_DISABLE(X)  do{ strcpy(X,"\\0d\\00\\00\\00DisableStrobe"); } while(0)  /* set stobe to disabled state */
#define API_AqoR_DISABLE1(X)  do{ sprintf(X,"\\0e\\00\\00\\00DisableStrobe%c",0x0au); } while(0)  /* set stobe to disabled state */
#define API_AqoR_ENABLE(X)  do{ strcpy(X,"\\0d\\00\\00\\00EnableStrobe"); } while(0)  /* set stobe to enabled state */
#define API_AqoR_ENABLE1(X)  do{ sprintf(X,"\\0e\\00\\00\\00EnableStrobe%c",0x0au); } while(0)  /* set stobe to enabled state */
#define API_AqoR_LAMPON(X)  do{ strcpy(X,"\\06\\00\\00\\00LampOn"); } while(0)  /* set the lamp on */
#define API_AqoR_LAMPON1(X)  do{ sprintf(X,"\\07\\00\\00\\00LampOn%c",0x0au); } while(0)  /* set the lamp on  */
#define API_AqoR_LAMPOFF(X)  do{ strcpy(X,"\\07\\00\\00\\00LampOff"); } while(0)  /* set the lamp off */
#define API_AqoR_LAMPOFF1(X)  do{ sprintf(X,"\\08\\00\\00\\00LampOff%c",0x0au); } while(0)  /* set the lamp off  */
#define API_AqoR_Mk2_GET_BRIGHT(X)  do{ strcpy(X,"\\0e\\00\\00\\00LampBrightness"); } while(0)  /* get the brightness mk2 probe only */
#define API_AqoR_Mk2_GET_BRIGHT1(X)  do{ sprintf(X,"\\0f\\00\\00\\00LampBrightness%c",0x0au); } while(0)  /* get the brightness mk2 probe only  */
/* ------ set the brightness --------- */
#define API_AqoR_SET_BRIGHT(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    if (val>100u) val=100u; \
    sprintf(buf,"SetLampBrightness:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
#define API_AqoR_Mk2_SET_BRIGHT(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    if (val>100u) val=100u; \
    sprintf(buf,"LampBrightness:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
#define API_AqoR_Mk2_LAMP_POWER(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    if (val>100u) val=100u; \
    sprintf(buf,"LampPower:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* ===================== Expansion Output =================================== */
#define API_EXPAN_ALLBREAK_OFF(X)  do{ strcpy(X,"\\0a\\00\\00\\00BreakerOff"); } while(0)  /* all breakers off */
#define API_EXPAN_ALLBREAK_OFF1(X)  do{ sprintf(X,"\\0b\\00\\00\\00BreakerOff%c",0x0au); } while(0)  /* all breakers off */
#define API_EXPAN_ALLBREAK_ON(X)  do{ strcpy(X,"\\09\\00\\00\\00BreakerOn"); } while(0)  /* all breakers on */
#define API_EXPAN_ALLBREAK_ON1(X)  do{ sprintf(X,"\\0a\\00\\00\\00BreakerOn%c",0x0au); } while(0)  /* all breakers on */
#define API_EXPAN_BREAKER(x, id, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    if (val>=1u)  \
      sprintf(buf,"BreakerOn:%u%c",id,0x0au); \
    else \
      sprintf(buf,"BreakerOff:%u%c",id,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* ===================== Exposure =========================================== */
/* sets number of stops to perform when compensating for strobe */
#define MAX_NO_OF_EXPO_STOPS 100u
#define API_EXPO_STOPS(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    val = val % MAX_NO_OF_EXPO_STOPS;  \
    sprintf(buf,"ExposureStops:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* sets exposure offset for when in Auto Exposure */
#define MAX_EXPO_VALUE INT16_MAX
#define API_EXPO_VAL(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    val = val % MAX_EXPO_VALUE;  \
    sprintf(buf,"ExposureValue:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* sets shutter speed in nano seconds */
#define MAX_SHUT_SPD UINT32_MAX
#define API_EXPO_SHUT_SPD(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    val = val % MAX_SHUT_SPD;  \
    sprintf(buf,"ShutterSpeed:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/*  sets iSO to use when a strobe is connected */
#define MAX_STROB_ISO INT16_MAX
#define API_EXPO_STROB_ISO(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    val = val % MAX_STROB_ISO;  \
    sprintf(buf,"StrobeISO:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/*  sets shutter speed to use when I strobe is connected */
#define MAX_STROB_SHSPD UINT32_MAX
#define API_EXPO_STROB_SHSPD(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    val = val % MAX_STROB_SHSPD;  \
    sprintf(buf,"StrobeShutter:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
#define API_EXPO_AUTO_ON(X)  do{ strcpy(X,"\\0c\\00\\00\\00AutoExposure"); } while(0)  /* AutoExposure on */
#define API_EXPO_AUTO_ON1(X)  do{ sprintf(X,"\\0d\\00\\00\\00AutoExposure%c",0x0au); } while(0)  /* AutoExposure on */
#define API_EXPO_CLR_WB_PRE(X)  do{ strcpy(X,"\\18\\00\\00\\00ClearWhiteBalancePresets"); } while(0)  /* ClearWhiteBalancePresets */
#define API_EXPO_CLR_WB_PRE1(X)  do{ sprintf(X,"\\19\\00\\00\\00ClearWhiteBalancePresets%c",0x0au); } while(0)  /* ClearWhiteBalancePresets */
/* Adjust the strobe values relative to the current ISO and Shuttter with the set number of stops */
#define API_EXPO_COMPEN(X)  do{ strcpy(X,"\\0a\\00\\00\\00Compensate"); } while(0)  /* Compensate */
#define API_EXPO_COMPEN1(X)  do{ sprintf(X,"\\0b\\00\\00\\00Compensate%c",0x0au); } while(0)  /* Compensate */
/* Adjust the strobe values relative to the current ISO and Shuttter with the given number of stops */
#define API_EXPO_COMPEN_SET(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    val = val % MAX_NO_OF_EXPO_STOPS;  \
    sprintf(buf,"Compensate:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* control exposure ammount */
typedef enum { RF_EXPO_DECREASE, RF_EXPO_INCREASE, RF_EXPO_LOCK, RF_EXPO_ULOCK, RF_EXPO_MAN, RF_EXPO_AUTO, RF_MAX_NO_OF_EXPO_MODES } rayfin_expo_mode_e;  /* exposure modes up,down and lock */
#define API_EXPO_CTLR(x, e_expo_mode) \
do { \
    unsigned char buf[22u]; \
    uint16_t len; \
    memset(buf,0u,22u); \
    e_expo_mode = e_expo_mode % RF_MAX_NO_OF_EXPO_MODES;  \
    switch(e_expo_mode)  \
    {                     \
       case RF_EXPO_DECREASE: \
       sprintf(buf,"DecreaseExposureValue%c",0x0au); \
       break; \
              \
       case RF_EXPO_INCREASE: \
       sprintf(buf,"IncreaseExposureValue%c",0x0au); \
       break; \
              \
       case RF_EXPO_LOCK: \
       sprintf(buf,"LockAE%c",0x0au); \
       break; \
              \
       case RF_EXPO_ULOCK: \
       sprintf(buf,"UnlockAE%c",0x0au); \
       break; \
              \
       case RF_EXPO_MAN: \
       sprintf(buf,"ManualExposure%c",0x0au); \
       break; \
              \
       case RF_EXPO_AUTO: \
       sprintf(buf,"AutoExposure%c",0x0au); \
       break; \
              \
       default: \
       break; \
    } \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* Update white balance blue, green and red channel */
typedef enum { RF_BLUE, RF_GREEN, RF_RED, RF_MAX_NO_WHITE_CHANS } rayfin_white_chan_color_e;
#define API_EXPO_UPDATE_WHITE_CHAN(x, e_white_chan, val) \
do { \
    unsigned char buf[22u]; \
    uint16_t len; \
    memset(buf,0u,22u); \
    e_white_chan = e_white_chan % RF_MAX_NO_WHITE_CHANS;  \
    switch(e_white_chan)  \
    {                     \
       case RF_BLUE: \
       sprintf(buf,"UpdateB:%f%c",val,0x0au); \
       break; \
              \
       case RF_GREEN: \
       sprintf(buf,"UpdateG:%f%c",val,0x0au); \
       break; \
              \
       case RF_RED: \
       sprintf(buf,"UpdateR:%f%c",val,0x0au); \
       break; \
              \
       default: \
       break; \
    } \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* update exposure */
#define MAX_EXPO_UPDATE INT16_MAX;
#define API_EXPO_UPDATE(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    val = val % MAX_EXPO_UPDATE;  \
    sprintf(buf,"UpdateExposure:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* update iso */
#define MAX_ISO_UPDATE 800u
#define MIN_ISO_UPDATE 50u
#define API_ISO_UPDATE(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    if ((val >= MIN_ISO_UPDATE) && (val <= MAX_ISO_UPDATE))  \
    { \
       sprintf(buf,"UpdateISO:%u%c",val,0x0au); \
       /* find the length note the unsurity of length and end LF field so check this during test */ \
       len = strlen(buf); \
       sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
    } \
} while(0)
/* update shutter speed */
#define MAX_SHUT_SPD UINT32_MAX
#define API_SHUT_SPEED_UPDATE(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    val = val % MAX_SHUT_SPD;  \
    sprintf(buf,"UpdateShutterSpeed:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* set optical focus distance - distance of camera focus */
#define API_SET_OPT_FOCUS_DIST(x, val) \
do { \
    unsigned char buf[25u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    sprintf(buf,"OpticalFocusDistance:%f%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* set optical zoom level */
#define MAX_SHUT_SPEED INT16_MAX
#define API_SET_OPT_ZOOM_LEVEL(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    val = val % MAX_ZOOM_LEVEL;  \
    sprintf(buf,"OpticalZoomLevel:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* set the iris */
#define API_SET_IRIS(x, val) \
do { \
    unsigned char buf[10u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    sprintf(buf,"Iris:%f%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* focus to the given level */
#define API_SET_FOCUS_LVL(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    sprintf(buf,"OpticalFocus:%f%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* Focus far near and default */
typedef enum { RF_FAR, RF_NEAR, RF_DEFAULT, RF_CALI, RF_HOMING, RF_AUTO, RF_MAN, RF_MAX_NO_FOCUS_SET } rayfin_lens_modes_e;
#define API_LENS_MODES(x, e_foc_mde) \
do { \
    unsigned char buf[22u]; \
    uint16_t len; \
    memset(buf,0u,22u); \
    e_foc_mde = e_foc_mde % RF_MAX_NO_FOCUS_SET;  \
    switch(e_foc_mde)  \
    {                     \
       case RF_FAR: \
       sprintf(buf,"OpticalFocusFar%c",0x0au); \
       break; \
              \
       case RF_NEAR: \
       sprintf(buf,"OpticalFocusNear%c",0x0au); \
       break; \
              \
       case RF_DEFAULT: \
       sprintf(buf,"OpticalToDefault%c",0x0au); \
       break; \
              \
       case RF_CALI: \
       sprintf(buf,"StartCalibration%c",0x0au); \
       break; \
              \
       case RF_HOMING: \
       sprintf(buf,"StartHoming%c",0x0au); \
       break; \
              \
       case RF_AUTO: \
       sprintf(buf,"EnableAutoFocus%c",0x0au); \
       break; \
              \
       case RF_MAN: \
       sprintf(buf,"EnableManualFocus%c",0x0au); \
       break; \
              \
       default: \
       break; \
    } \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* ================== Skate Laser =========================================== */
#define API_SET_SKATE_LASER(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    if ((val % 2u)==0)  \
      sprintf(buf,"LaserOff%c",0x0au); \
    else               \
      sprintf(buf,"LaserOn%c",0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* set the laser brightness */
#define API_SET_LASER_BRIGHT(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    sprintf(buf,"LaserBrightness:%f%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* set movement */
typedef enum { RF_PAN_LEFT, RF_PAN_RIGHT, RF_TILT_UP, RF_TILT_DOWN,  RF_MAX_MOTIONS } rayfin_movements_e;
#define MAX_MOVE 100u
#define API_SET_MOVEMENT(x, e_move, val) \
do { \
    unsigned char buf[22u]; \
    uint16_t len; \
    memset(buf,0u,22u); \
    e_move = e_move % RF_MAX_MOTIONS;  \
    val = val % MAX_MOVE; \
    switch(e_move)  \
    {                     \
       case RF_PAN_LEFT: \
       sprintf(buf,"PanLeft:%f%c",val,0x0au); \
       break; \
              \
       case RF_PAN_RIGHT: \
       sprintf(buf,"PanRight:%f%c",val,0x0au); \
       break; \
              \
       case RF_TILT_UP: \
       sprintf(buf,"TiltUp:%f%c",val,0x0au); \
       break; \
              \
       case RF_TILT_DOWN: \
       sprintf(buf,"TiltDown:%f%c",val,0x0au); \
       break; \
              \
       default: \
       break; \
    } \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* hdmi HDMI converter(GPIO_61) */
#define API_SET_HDMI(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    if ((val % 2u)==0)  \
      sprintf(buf,"HDMIPowerOff%c",0x0au); \
    else               \
      sprintf(buf,"HDMIPowerOn%c",0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* video output mode */
typedef enum { RAYF_NTSC, RAYF_PAL, RF_MAX_OUT_FORMAT } rayfin_out_format_e;
#define API_SET_VID_OUT_MODE(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    if (val == RAYF_NTSC)  \
      sprintf(buf,"OutputNTSC%c",0x0au); \
    else if (val == RAYF_PAL)              \
      sprintf(buf,"OutputPAL%c",0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* GPIO_125 which sets the Chrontel composite converter */
#define API_SET_CHRONTEL(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    if (val == RAYF_NTSC)  \
      sprintf(buf,"ResetPinLow%c",0x0au); \
    else if (val == RAYF_PAL)              \
      sprintf(buf,"ResetPinHigh%c",0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* set Zoom */
#define RAYF_MAX_ZOOM 10u
#define RAYF_MIN_ZOOM 1u
#define API_SET_ZOOM(x, val) \
do { \
    unsigned char buf[8u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    if ((val >= RAYF_MIN_ZOOM) && (val <= RAYF_MAX_ZOOM))  \
      sprintf(buf,"Zoom:%u%c",val,0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* zoom in or out */
#define RAYF_ZOOM_IN 0u
#define RAYF_ZOOM_OUT 1u
#define API_ZOOM_IN_OUT(x, val) \
do { \
    unsigned char buf[20u]; \
    uint16_t len; \
    memset(buf,0u,20u); \
    if ((val % 2u)==0)  \
      sprintf(buf,"ZoomStepIn%c",0x0au); \
    else               \
      sprintf(buf,"ZoomStepOut%c",0x0au); \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* power management */
typedef enum { RF_HIBERNATE, RF_OFF_IN_10S, RF_OFF_IN_TIM, RF_REBOOT_10S, RF_REBOOT_IN_TIM, RF_MAX_POWER_ACTIONS } rayfin_power_actions_e;
#define API_RAYF_POWER(x, e_power, val) \
do { \
    unsigned char buf[22u]; \
    uint16_t len; \
    memset(buf,0u,22u); \
    e_power = e_power % RF_MAX_POWER_ACTIONS;  \
    switch(e_power)  \
    {                     \
       case RF_HIBERNATE: \
       sprintf(buf,"Hibernate:%f%c",val,0x0au); \
       break; \
              \
       case RF_OFF_IN_10S: \
       sprintf(buf,"PowerOff%c",0x0au); \
       break; \
              \
       case RF_OFF_IN_TIM: \
       sprintf(buf,"PowerOff:%f%c",val,0x0au); \
       break; \
              \
       case RF_REBOOT_10S: \
       sprintf(buf,"Reboot%c",0x0au); \
       break; \
              \
       case RF_REBOOT_IN_TIM: \
       sprintf(buf,"Reboot:%f%c",val,0x0au); \
       break; \
              \
       default: \
       break; \
    } \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* --------------- Rayfin Dive Methods and Properties ----------------------- */
/* dive management */
#if defined(D_FT900)
typedef struct CAMPACKED {
   unsigned char diveDir[40u];
   unsigned char diveProjNm[20u];
   unsigned char logNm[20u];
   uint16_t logInterval;
} RF_diveProperties_t;
#else
CAMPACKED(
typedef struct {
   unsigned char diveDir[40u];
   unsigned char diveProjNm[20u];
   unsigned char logNm[20u];
   uint16_t logInterval;
}) RF_diveProperties_t;
#endif
typedef enum { RF_SET_DIR, RF_PRO_NM, RF_LOG_NM, RF_START_DIVE, RF_SET_LOG_INTERVAL, RF_MAX_DIVE_CONF } rayfin_dive_config_e;
#define API_MANAGE_DIVE(x, e_dive, val) \
    /* e_dive is a rayfin_dive_config_e and val is a *RF_diveProperties_t */ \
do { \
    unsigned char buf[80u]; \
    uint16_t len; \
    memset(buf,0u,22u); \
    e_dive = e_dive % RF_MAX_DIVE_CONF;  \
    switch(e_dive)  \
    {                     \
       case RF_SET_DIR: \
       sprintf(buf,"ProjectDirectory:%s%c",val->diveDir,0x0au); \
       break; \
              \
       case RF_PRO_NM: \
       sprintf(buf,"ProjectName:%s%c",val->diveProjNm,0x0au); \
       break; \
              \
       case RF_LOG_NM: \
       sprintf(buf,"LogEvent:%s%c",val->logNm,0x0au); \
       break; \
              \
       case RF_START_DIVE: \
       sprintf(buf,"StartDive:%s, %s%c",val->diveDir,val->diveProjNm,0x0au); \
       break; \
              \
       case RF_SET_LOG_INTERVAL: \
       val = val % UINT16_MAX; \
       sprintf(buf,"RecordingLogInterval:%u%c",val->logInterval,0x0au); \
       break; \
              \
       default: \
       break; \
    } \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* JSON Log requester */
typedef enum { RF_GET_VID_LOG, RF_GET_STILLS_LOG, RF_GET_EVENT_LOG, RF_GET_DIVE, RF_DIVE_JSON_METHODS } rayfin_dive_jSonReport_e;
#define API_REQ_JSON_DIVE(x, e_dive) \
    /* e_dive is a rayfin_dive_jSonReport_e  */ \
do { \
    unsigned char buf[80u]; \
    uint16_t len; \
    memset(buf,0u,22u); \
    e_dive = e_dive % RF_DIVE_JSON_METHODS;  \
    switch(e_dive)  \
    {                     \
       case RF_GET_VID_LOG: \
       sprintf(buf,"GetVideoLog%c",0x0au); \
       break; \
              \
       case RF_GET_STILLS_LOG: \
       sprintf(buf,"GetStillsLog%c",0x0au); \
       break; \
              \
       case RF_GET_EVENT_LOG: \
       sprintf(buf,"GetEventLog%c",0x0au); \
       break; \
              \
       case RF_GET_DIVE: \
       sprintf(buf,"GetDive%c",0x0au); \
       break; \
              \
       default: \
       break; \
    } \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)

/* video manager */
typedef enum { RF_GET_REC_TIM, RF_GET_VID_RES, RF_GET_VID_NM, RF_GET_VID_DIR, RF_VIDEO_ACTIONS } rayfin_video_manager_e;
#define API_REQ_VIDEO_INFO(x, e_dive ) \
    /* e_dive is a rayfin_video_manager_e  */ \
do { \
    unsigned char buf[80u]; \
    uint16_t len; \
    memset(buf,0u,22u); \
    e_dive = e_dive % RF_VIDEO_ACTIONS;  \
    switch(e_dive)  \
    {                     \
       case RF_GET_REC_TIM: \
       sprintf(buf,"RecordingDuration%c",0x0au); \
       break; \
              \
       case RF_GET_VID_RES: \
       sprintf(buf,"VideoResolution%c",0x0au); \
       break; \
              \
       case RF_GET_VID_NM: \
       sprintf(buf,"VideoName%c",0x0au); \
       break; \
              \
       case RF_GET_VID_DIR: \
       sprintf(buf,"VideosDirectory%c",0x0au); \
       break; \
              \
       default: \
       break; \
    } \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* set the video properties */
#if defined(D_FT900)
typedef struct CAMPACKED {
   int16_t bitRate;
   uint8_t encCodec;
   uint8_t vidRes;
   unsigned char videoNm[20u];
   unsigned char videoDir[40u];
} RF_videoProperties_t;
#else
CAMPACKED(
typedef struct {
   int16_t bitRate;
   uint8_t encCodec;
   uint8_t vidRes;
   unsigned char videoNm[20u];
   unsigned char videoDir[40u];
}) RF_videoProperties_t;
#endif
typedef enum { RF_SET_BIT_RT, RF_ENC_TYPE, RF_VID_NM, RF_VIDDIR_NM, RF_VID_RES, RF_VID_DEFAULT, RF_MAX_VID_SET } rayfin_video_config_e;
#define API_MANAGE_VIDEO(x, e_vid, val) \
    /* e_dive is a rayfin_video_config_e and val is a *RF_videoProperties_t */ \
do { \
    unsigned char buf[80u]; \
    uint16_t len; \
    memset(buf,0u,22u); \
    e_vid = e_vid % RF_MAX_VID_SET;  \
    switch(e_vid)  \
    {                     \
       case RF_SET_BIT_RT: \
       sprintf(buf,"BitRate:%d%c",val->bitRate,0x0au); \
       break; \
              \
       case RF_ENC_TYPE: \
       sprintf(buf,"Encoder:%u%c",val->encCodec,0x0au); \
       break; \
              \
       case RF_VID_NM: \
       sprintf(buf,"VideoName:%s%c",val->videoNm,0x0au); \
       break; \
              \
       case RF_VIDDIR_NM: \
       sprintf(buf,"VideosDirectory:%s%c",val->videoDir,0x0au); \
       break; \
              \
       case RF_VID_RES: \
       sprintf(buf,"UpdateVideoResolution:%u%c",val->vidRes,0x0au); \
       break; \
              \
       case RF_VID_DEFAULT: \
       sprintf(buf,"LoadDefaults%c",0x0au); \
       break; \
              \
       default: \
       break; \
    } \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)
/* stills handler */
/* set the video properties */
#if defined(D_FT900)
typedef struct CAMPACKED {
  unsigned char burstDir[40u];
  unsigned char burstNm[40u];
  unsigned char stillsDir[40u];
  unsigned char stillNm[40u];
  uint16_t burstRt;
  uint8_t imageFormat;
  uint8_t jpegQual;
  uint8_t jpegRes;
  uint16_t maxFile;
  uint8_t rawRes;
} RF_stillsProperties_t;
#else
CAMPACKED(
typedef struct {
  unsigned char burstDir[40u];
  unsigned char burstNm[40u];
  unsigned char stillsDir[40u];
  unsigned char stillNm[40u];
  uint16_t burstRt;
  uint8_t imageFormat;
  uint8_t jpegQual;
  uint8_t jpegRes;
  uint16_t maxFile;
  uint8_t rawRes;
}) RF_stillsProperties_t;
#endif
typedef enum { RF_SET_BURST_DIR, RF_BURST_NM, RF_BURST_RT, RF_IMG_FMT, RF_JPG_QUAL, RF_JPG_RES, RF_MAX_FILE, RF_RAW_RES, RF_SET_STILLS_DIR, RF_STILL_NM, RF_DEFAULT_STILL_NM, RF_MAX_STILL_SET } rayfin_still_config_e;
#define API_MANAGE_STILLS(x, e_vid, val) \
    /* e_dive is a rayfin_video_config_e and val is a *RF_stillsProperties_t */ \
do { \
    unsigned char buf[80u]; \
    uint16_t len; \
    memset(buf,0u,22u); \
    e_vid = e_vid % RF_MAX_STILL_SET;  \
    switch(e_vid)  \
    {                     \
       case RF_SET_BURST_DIR: \
       sprintf(buf,"BurstDirectory:%s%c",val->burstDir,0x0au); \
       break; \
              \
       case RF_BURST_NM: \
       sprintf(buf,"BurstName:%s%c",val->burstNm,0x0au); \
       break; \
              \
       case RF_BURST_RT: \
       sprintf(buf,"BurstRate:%u%c",val->burstRt,0x0au); \
       break; \
              \
       case RF_IMG_FMT: \
       sprintf(buf,"ImageFormat:%u%c",val->imageFormat,0x0au); \
       break; \
              \
       case RF_JPG_QUAL: \
       sprintf(buf,"JPEGQuality:%u%c",val->jpegQual,0x0au); \
       break; \
              \
       case RF_JPG_RES: \
       sprintf(buf,"UpdateJPEGResolution:%u%c"val->jpegRes,0x0au); \
       break; \
              \
       case RF_MAX_FILE: \
       sprintf(buf,"MaxFilesPerDirectory:%u%c"val->maxFile,0x0au); \
       break; \
              \
       case RF_RAW_RES: \
       sprintf(buf,"RawResolution:%u%c"val->rawRes,0x0au); \
       break; \
              \
       case RF_SET_STILLS_DIR: \
       sprintf(buf,"StillsDirectory:%s%c",val->stillsDir,0x0au); \
       break; \
              \
       case RF_STILL_NM: \
       sprintf(buf,"StillName:%s%c",val->stillNm,0x0au); \
       break; \
              \
       case RF_DEFAULT_STILL_NM: \
       sprintf(buf,"LoadDefaults%c",0x0au); \
       break; \
              \
       default: \
       break; \
    } \
    /* find the length note the unsurity of length and end LF field so check this during test */ \
    len = strlen(buf); \
    sprintf(x,"\\%d\\00\\00\\00%s",len,buf); \
} while(0)

#endif /* Rayfin Underwater cmaera */

/* ============== AMP XS Video Encoder / Decoder ============================ */
#if (CAMERA_TYPE == XS_CAM)                                                    // Type of CAM is the AMP XS Encoder

#define XS_RAW_TO_CAM(val) ((uint16_t)(val)*10000.0f/1023.0f)                   // Conversion Macro raw 0-1023 to 0-10000 (scale for brightness etc).

#define USE_NTSC                                                                // Define here which type of input you have to the encoder (options below)
#ifdef USE_PAL_SECAM
#define MAX_FPS 25U                                                             // Number of frames per second for PAL or SECAM  (Phase Alternating Line)
#endif
#ifdef USE_NTSC
#define MAX_FPS 30U                                                             // Number of frames per second for NTSC (National television system comittee)
#endif

#define XS_MIN_BITRATE 30000U                                                   // minimum allowable bitrate
#define XS_MAX_BITRATE 20000000UL                                               // maximum allowable bitrate
#define XS_MIN_IINTERVAL 1U                                                     // minimum allowable iinterval
#define XS_MAX_IINTERVAL 300U                                                   // maximum allowable iinterval
#define XS_CMD_MAX_BYTES 50u                                                    // maximum length of xs message

                                                                                // Define the used return states for an XStream reply
#define XS_WAIT 0U                                                              // Set to wait for a reply
#define XS_AUTH 1U                                                              // auth came back
#define XS_ERROR 2U                                                             // error occurred
#define XS_DISK 3U                                                              // A Disk status reply
#define XS_SYS 4U                                                               // A System Status reply
#define XS_TIME 5U                                                              // A Current Time reply from the encoder
#define XS_OK 6U                                                                // ok came back without a field after a Comma
#define XS_OK1 7U                                                               // ok came back with a field after a Comma
#define XS_OK2 8U                                                               // ok came back with 2 fields after a Comma
#define XS_OK3 9U                                                               // ok came back with 3 fields after a Comma
#define XS_OK4 10U                                                              // ok came back with 4 fields after a Comma
#define XS_OK5 11U                                                              // ok came back with 5 fields after a Comma
#define XS_OK6 12U                                                              // ok came back with 6 fields after a Comma
#define XS_OK7 13U                                                              // ok came back with 7 fields after a Comma
#define XS_OK8 14U                                                              // ok came back with 8 fields after a Comma
#define XS_WRONG 20U                                                            // +wrong came back Channel in wrong state
#define XS_FAIL 21U                                                             // +fail came back unsuccessful command
#define XS_INVALID 22U                                                          // +invalid came back wrong parameter value
#define XS_AUTH_WAIT 23U                                                        // wait for new user and or password

#define XS_CONT_TIL_SUCC 1U                                                     // mode is continue until success dont reset send delta until serial success message cane back
#define XS_TRY_TWICE 2U                                                         // mode resets the delta on seeing it and sends the UDP datagram twice

#define XS_DELTA_MODE XS_CONT_TIL_SUCC                                          // Choose the mode

#define XS_STEP_SETHMI 21U                                                      // Varaible step defined as compled all init and set HMI variables to what was read

#define XS_CMD_REQ_STX 0x24U                                                    // Encoder response message STX start transmission char
#define XS_CMD_REPLY_STX 0x2BU                                                  // Encoder REplies with a + sign
#define XS_CMD_REQ_SEP 0x2CU                                                    // Encoder response message field separator
#define XS_CMD_REQ_ETX 0x0DU                                                    // Encoder response message end transmission char

// ========== Data Request Command Set =========================================
#define API_XS_GET(X,Y)  do { sprintf(X,"$GET%s\r",Y); } while(0)                           // $GET<cmd> where e.g cmd = "RECYCLE", "TIME"
#define API_XS_GETCMD1(X,Y,Z) do { sprintf(X,"$GET%s,%d\r",Y,Z); } while(0)               // $GET<cmd>,<channel> e.g. cmd = "HUE",1 "CONTRAST",2  "AVGBITRATE",3
#define API_XS_GETCMD2(X,Y,Z,A) do { sprintf(X,"$GET%s,%d,%s\r",Y,Z,A); } while(0)        // $GET<cmd>,<stream>,<topology>
#define API_XS_ENUMST(X,Y) do { sprintf(X,"$ENUMSTREAMS,%d\r",Y); } while(0)              // $ENUMSTREAMS,<channel> lists info on the stream
#define API_XS_NET(X,Y) do { sprintf(X,"$GETNETWORK,%d\r",Y); } while(0)                  // $GETNETWORK,<adapter>
#define API_XS_DDISK(X,Y) do { sprintf(X,"$DATADISK,%d\r",Y); } while(0)                  // $DATADISK,<id> changes the storage disk
#define API_XS_BAUD(X,Y) do { sprintf(X,"$BAUDRATE,%d\r",Y); } while(0)                   // $BAUDRATE,<id> changes baud rate (you can change this for next time, must then change you're own)
#define API_XS_TIME(X,Y) do { sprintf(X,"$TIME,%s\r",Y); } while(0)                       // $TIME,<time string>  sets the time
#define API_XS_RECYC(X,Y) do { sprintf(X,"$RECYCLE,%d\r",Y); } while(0)                   // $RECYCLE,<threshold> (set file recycling threshold)
#define API_XS_DISK(X) do { sprintf(X,"$DISKSTATUS\r"); } while(0)                        // $DISKSTATUS
#define API_XS_SYS(X) do { sprintf(X,"$SYSSTATUS\r"); } while(0)                          // $SYSSTATUS
#define API_XS_HOST(X) do { sprintf(X,"$GETHOSTNAME\r"); } while(0)                       // $GETHOSTNAME
#define API_XS_ENUMD(X) do { sprintf(X,"$ENUMDISKS\r"); } while(0)                        // $ENUMDISKS lists the available disks
#define API_XS_COMT(X) do { sprintf(X,"$COMMIT\r"); } while(0)                            // $COMMIT writes changes
#define API_XS_SHUTD(X) do { sprintf(X,"$SHUTDOWN\r"); } while(0)                         // $SHUTDOWN
#define API_XS_REBOOT(X) do { sprintf(X,"$REBOOT\r"); } while(0)                          // $REBOOT
#define API_XS_RECORD(X,Y) do { sprintf(X,"$RECORD,%d\r",Y); } while(0)                   // $RECORD,<channel> e.g. $RECORD,-1
#define API_XS_STOPRECORD(X,Y) do { sprintf(X,"$STOP,%d\r",Y); } while(0)                 // $STOP,<channel> e.g. $STOP,-1
#define API_XS_MARK(X,Y) do { sprintf(X,"$MARK,%d\r",Y); } while(0)                       // $MARK,<channel> e.g. $MARK,4
#define API_XS_ERASE(X,Y) do { sprintf(X,"$ERASE,%d\r",Y); } while(0)                     // $ERASE,<channel> e.g. $ERASE,4
#define API_XS_DATEFORMAT(X,Y,Mo,D,H,Mi,S) do { sprintf(X,"%d-%d-%d %s:%s:%s.000 ",Y,Mo,D,H,Mi,S); } while(0) // e.g. 2016-11-28 11:00:05.554

// define a word to contain the commands that are queued
#define XS_CONTRAST (1u<<0u)                                                      // contrast
#define XS_BRIGHTNESS (1u<<1u)                                                    // brightness
#define XS_HUE (1u<<2u)                                                           // hue
#define XS_SATURATION (1u<<3u)                                                    // saturation
#define XS_CLIPSIZE (1u<<4u)                                                      // clip size
#define XS_CLIPLEN (1u<<5u)                                                       // clip length
#define XS_FRAMERTE (1u<<6u)                                                      // frame rate
#define XS_BITRTE (1u<<7u)                                                        // bit rate
#define XS_INTVAL (1u<<8u)                                                        // frame interval
#define XS_SYSSTAT (1u<<9u)                                                       // system status
#define XS_DISKSTAT (1u<<10u)                                                     // disk status
#define XS_COMMIT (1u<<11u)                                                       // commit
#define XS_REBOOT (1u<<12u)                                                       // reboot
#define XS_SHUTD (1u<<13u)                                                        // shutdown
#define XS_STARTREC (1u<<14u)                                                     // start recording
#define XS_STOPREC (1u<<15u)                                                      // stop recording
#define XS_MARKREC (1u<<16u)                                                      // mark recording
#define XS_SETTIME (1u<<17u)                                                      // set encoder rtc
#define XS_RECYCLE (1u<<18u)                                                      // set recycle threshold
#define XS_ERASE (1u<<19u)                                                        // erase files
#define XS_LOGIN (1u<<20u)                                                        // authorise and login

// define states of error on AUTHENTICATION login to the encoder
#define XS_PASWD_WRONG 100U                                                     // wrong password specified
#define XS_LOGIN_INVALID 101U                                                   // wrong user
#define XS_RTSP_ERROR 102U                                                      // RTSP failure

// define the alarm states for the login failure event and other XS errors
#define XS_WRONG_PW (1u<<0u)                                                      // wrong password
#define XS_WRONG_USER (1u<<1u)                                                    // wrong user
#define XS_WRONG_REPLY (1u<<2u)                                                   // bad reply
#define XS_WRONG_STATE (1u<<3u)                                                   // Channel in wrong state
#define XS_RTSP_FAIL (1u<<4u)                                                     // RTSP set-up failure
#define XS_WRONG_TIME (1u<<5u)                                                    // Wrong date time spec specified

// define the IP Addresses for remote RTSP protocol
#define XSencoderIP "192.172.3.76"                                              // remote encoder ip address
#define XSgatewayIP "192.172.3.1"                                               // encoder gateway IP address

#endif /* XS CAM DEFS */
// =============================================================================
//
// ============ Run Cam Protocol ===============================================
//
//==============================================================================
extern struct RC_res_get_info_t RCCamRepInfo;                                   // Camera info reply information
extern struct RC_res_keyaction_t RCCamKeyConfirm;                               // Confirmation to key action requests
extern struct RC_res_handshake_t RCCamHandshakeConfirm;                         // Confirmation of handshake action
extern struct RC_res_read_settings_t RCCamRepReadSetup;                         // Read Set-up parameter request reply
extern struct RC_res_handshake_t RCCamRepWriteSetup;                            // Reply to the write settings request
extern struct RC_res_get_settings_t RCGetSettings;                              // Reply to the get settings request
extern struct RC_req_write_str_t RCText2Screen;
extern uint8_t g_RunCamSent;                                                    // Current message being sent

#define RC_CMD_MAX_BYTES 64U                                                    // Define the maximum Run Cam reply message
#define RC_MAX_TXT_LINE 60U                                                     // Maximum number of text lines

// Request command definitions
#define RC_STX_CHAR 0xCCU                                                       // Start transmission characture
#define RC_PROTO_CMD_GET_DEVICE_INFO 0x00U                                      // Request Run Cam information
#define RC_PROTO_CMD_CAMERA_CONTROL 0x01U                                       // Camera control For example: through this instruction, send an instruction to simulate the actions of power button to the camera
#define RC_PROTO_CMD_5KEY_SIMULATION_PRESS 0x02U                                // Send the Press event of the 5 key remote control to the camera
#define RC_PROTO_CMD_5KEY_SIMULATION_RELEASE 0x03U                              // Send the Release event of the 5 key remotre control to the camera
#define RC_PROTO_CMD_5KEY_CONNECTION 0x04U                                      // Send handshake events and disconnected events to the camera
#define RC_PROTO_CMD_GET_SETTINGS 0x10U                                         // get a sub settings info with special parent setting id?including setting name, type and current value(in string format)
#define RC_PROTO_CMD_READ_SETTING_DETAIL 0x11U                                  // Retrieve the detail of setting, e.g it's maybe including max value, min value and etc. This command can not be called for the setting type with Folder and Static
#define RC_PROTO_CMD_WRITE_SETTING 0x13U                                        // change the value of special setting?can't call this command with the setting type of FOLDER and INFO
#define RC_PROTO_CMD_DISP_FILL_REGION 0x20U                                     // Fill an area with a special char
#define RC_PROTO_CMD_DISP_WRITE_CHAR 0x21U                                      // Write a character at special position
#define RC_PROTO_CMD_DISP_WRITE_HORT_STRING 0x22U                               // Write a string at special horizontal position
#define RC_PROTO_CMD_DISP_WRITE_VERT_STRING 0x23U                               // Write a string at special vertical position
#define RC_PROTO_CMD_DISP_WRITE_CHARS 0x24U                                     // Write a string at special position

// Features response byte definition for CMD_GET_DEVICE_INFO
#define RC_PROTO_FEAT_SIMULATE_POWER_BUTTON (1u<<0u)                              // Simulation Click the power button
#define RC_PROTO_FEAT_SIMULATE_WIFI_BUTTON (1u<<1u)                               // Simulation Click the Wi-Fi button
#define RC_PROTO_FEAT_CHANGE_MODE (1u<<2u)                                        // Switch the device operating mode
#define RC_PROTO_FEAT_SIMULATE_5_KEY_OSD_CABLE (1u<<3u)                           // Simulation 5-key OSD remote control
#define RC_PROTO_FEAT_DEVICE_SETTINGS_ACCESS (1u<<4u)                             // Support access to device settings
#define RC_PROTO_FEAT_DISPLAYP_PORT (1u<<5u)                                      // The device is identified as a DisplayPort device by flying controller and receives the OSD data display from the flight controller
#define RC_PROTO_FEAT_START_RECORDING (1u<<6u)                                    // Control the camera to start recording video
#define RC_PROTO_FEAT_STOP_RECORDING (1u<<7u)                                     // Control the camera to stop recording video

// Action byte definition for CMD_CAMERA_CONTROL request
#define RC_PROTO_SIMULATE_WIFI_BTN 0x00U                                        // Simulation Click the Wi-Fi button
#define RC_PROTO_SIMULATE_POWER_BTN 0x01U                                       // Simulation Click the Power button
#define RC_PROTOC_CHANGE_MODE 0x02U                                             // Switch the camera mode
#define RC_PROTO_CHANGE_START_RECORDING 0x03U                                   // Control the camera to start recording
#define RC_PROTO_CHANGE_STOP_RECORDING 0x04U                                    // Control the camera to stop recording

// Action byte definition for KEY_SIMULATION_PRESS and RELEASE request
#define RC_PROTO_5KEY_SIMULATION_SET 0x01U                                      // Simulate the confirmation key of the 5 key remote control
#define RC_PROTO_5KEY_SIMULATION_LEFT 0x02U                                     // Simulate the left key of the 5 key remote control
#define RC_PROTO_5KEY_SIMULATION_RIGHT 0x03U                                    // Simulate the right key of the 5 key remote control
#define RC_PROTO_5KEY_SIMULATION_UP 0x04U                                       // Simulate the up key of the 5 key remote control
#define RC_PROTO_5KEY_SIMULATION_DOWN 0x05U                                     // Simulate the down key of the 5 key remote control

// Action Id byte definition for COMMAND_5KEY_CONNECTION request
#define RC_PROTO_5KEY_FUNCTION_OPEN 0x01U                                       // Initiate a handshake action to the camera
#define RC_PROTO_5KEY_FUNCTION_CLOSE 0x02U                                      // Initiate a disconnection action to the camera

// Macro for response to COMMAND_5KEY_CONNECTION request val=response byte 2 act=action id requested
#define RC_HANDSHAKE_OK(val,act) ((uint8_t) (((val) & 0x01u) & (((val)>>4U)==act)))

// Request byte for RCDEVICE_PROTOCOL_COMMAND_GET_SETTINGS
#define RC_PROTO_SETTINGID_DISP_CHARSET 0U                                      // This setting is store current charset of the device R/W
#define RC_PROTO_SETTINGID_DISP_COLUMNS 1U                                      // Read the number of columns displayed on the screen line R
#define RC_PROTO_SETTINGID_DISP_TV_MODE 2U                                      // Read and set the camera's TV mode(NTSC,PAL) R/W
#define RC_PROTO_SETTINGID_DISP_SDCARD_CAPACIT 3U                               // Read the camera's memory card capacity R
#define RC_PROTO_SETTINGID_DISP_REMAIN_RECORDING_TIME 4U                        // Read the remaining recording time of the camera R
#define RC_PROTO_SETTINGID_DISP_RESOLUTION 5U                                   // Read and set the camera's resolution
#define RC_PROTO_SETTINGID_DISP_CAMERA_TIME 6U                                  // Read and set the camera's time

// Setting types from a CMD_READ_SETTING_DETAIL request
#define RC_PROTO_TYPE_UINT8 0U                                                  // uint8_t
#define RC_PROTO_TYPE_INT8 1U                                                   // int8_t
#define RC_PROTO_TYPE_UINT16 2U                                                 // uint16_t
#define RC_PROTO_TYPE_INT16 3U                                                  // int16_t
#define RC_PROTO_TYPE_FLOAT 8U                                                  // float32_t
#define RC_PROTO_TYPE_TEXT_SELECTION 9U                                         // text_selections will be returned as a null-terminated string, the content is all available string in the setting, they are separated by a semicolon(;)
#define RC_PROTO_TYPE_STRING 10U                                                // max string size will be returned, it's used to limit the max size of the string when user editing it.
#define RC_PROTO_TYPE_FOLDER 11U                                                // If the setting type is FOLDER, means it contains a set of settings. This setting can't be modified, when you call Get Detail Command(0x7) with it, it will return a empty response and the error code won't be zero.
#define RC_PROTO_TYPE_INFO 12U                                                  // if the settings type is INFO?this field is same with FOLDER, can't be modified.
#define RC_PROTO_TYPE_COMMAND 13U                                               // Not designed yet (reserved)

// States of the state engine this would need to change if you chose RC_MODE_TYPE instead
// This mode allows the user to conrtol the arrows the RC_MODE_TYPE would example know these
// keystrokes and set the Brightness for example
//
#define RC_PROTO_WIFI  1U                                                       // Simulation Click the Wi-Fi button
#define RC_PROTO_POWER 2U                                                       // Simulation Click the Power button
#define RC_PROTO_MODE 3U                                                        // Switch the camera mode video,photo,osd modes available
#define RC_PROTO_RECORD 4U                                                      // Start recording
#define RC_PROTO_STOP 5U                                                        // Stop recording
#define RC_PROTO_UP 6U                                                          // up press      (
#define RC_PROTO_DOWN 7U                                                        // down press
#define RC_PROTO_LEFT 8U                                                        // left press
#define RC_PROTO_RIGHT 9U                                                       // right press
#define RC_PROTO_CONFIRM 10U                                                    // confirm press
#define RC_PROTO_DS_CHARSET 11U                                                 // r/w charset via ds command 0x10U
#define RC_PROTO_DS_COULMN 12U                                                  // r only number of columns displayed
#define RC_PROTO_DS_NTSC_PAL 13U                                                // r/w ntsc or pal  READ STATUS
#define RC_PROTO_DS_MEMORY 14U                                                  // r read camera memory card
#define RC_PROTO_DS_RECTIME 15U                                                 // r read camera recording time left
#define RC_PROTO_DS_RESOL 16U                                                   // r/w camera resolution
#define RC_PROTO_DS_TIME 17U                                                    // r/w rtc of camera
#define RC_PROTO_WRITE_TXT 18U                                                  // write a text on the screen at a position
#define RC_PROTO_GETINFO 19U                                                    // Get the current state of the RunCam camera
#define RC_PROTO_CHG_MODE 20U                                                   // WRITE change the mode to NTSC or PAL difference was found
#define RC_PROTO_UP_REL 21U                                                     // up press off     (
#define RC_PROTO_DOWN_REL 22U                                                   // down press off
#define RC_PROTO_LEFT_REL 23U                                                   // left press off
#define RC_PROTO_RIGHT_REL 24U                                                  // right press off
#define RC_PROTO_CONFIRM_REL 25U                                                // confirm press off

#define RC_MSG_OK 1U                                                            // Good message returned and copied to container structure for that type
#define RC_MSG_FAIL 2U                                                          // Malformed message returned to Run Cam port
#define RC_MSG_INCOMP 3U                                                        // message incomplete go back to collection state without reset
#define RC_MSG_UNKNO 4U                                                         // message id is undefined
#define RC_MSG_RESET 4U                                                         // message too long possibly a partial and a full message reset and collect a new one
#define RC_MSG_WAIT 0U                                                          // WAit for a new response

#define RC_MODE_PHOTO 0U                                                        // Mode of camera is photo takes still pictures
#define RC_MODE_VIDEO 1U                                                        // Mode of camera is video streams video
#define RC_MODE_OSD 2U                                                          // Node of camera is OSD must be in this state for up/down left/right and change of parameters like contrast

#define RC_PROTO_WANT_PAL 1U                                                    // value for PAL change request
#define RC_PROTO_WANT_NTSC 2U                                                   // value for NTSC change request

// =============================================================================
// ------------------------ Xiaomi Yi Action Camera ----------------------------
// =============================================================================
#define XY_HOTSPOT_NAME(X) "YDXJ_" #X                                           // macro for making default hotspot name  X corresponds to the last 7 digits of the battery compartment at the camera's back side
#define XY_PASS_DEFAULT "1234567890"                                            // default password wifi
#define XY_CUSTOM_VLC_PATH "."                                                  // default path
//#define XY_CUSTOM_IP "192.168.1.254"                                          // Custom ip for the Yi Action Server  default (for SJ4000 WIFI used)
#if defined(XY_CUSTOM_IP)
   #define XY_USE_IP XY_CUSTOM_IP                                               // Use the custom ip specified
#else
  #define XY_USE_IP "192.168.42.1"                                              // Use the default ip
#endif

#define XY_STATE_PRE_MAX_TIM 10000U                                             // Maximum time to wait for an AJAX reply from the server

// Telnet server info  (May be needed, otherwise try raw tcp open on JSON port)
// Set up configuration parameter defaults if not overridden in
// You need to create a file shown here "cd /Volumes/16GB_CLASS4 "touch enable_info_display.script" in the root directory of the SD card,
// that triggers the main OS (the one that boots before Linux) to tell Linux to run telnetd once Linux comes up.
// edit /etc/init.d/S50service to uncomment the invocation of telnetd
// Create an autoexec.ash file on the root of the SD card with content "lu_util exec telnetd -l/bin/sh"
#if defined(XY_TELNET_PORT)
   #define TELNET_PORT 23U                                                      // Unsecured Telnet port  (check cam is there nc -vvn <ip.ip.ip.ip> 23
#endif
#if !defined(XY_TELNETS_PORT)
   #define TELNETS_PORT 992U                                                    // SSL Secured Telnet port (ignored if STACK_USE_SSL_SERVER is undefined)
#endif
#if !defined(XY_MAX_TELNET_CONNECTIONS)
   #define MAX_TELNET_CONNECTIONS (3u)                                          // Maximum number of Telnet connections
#endif
#if !defined(XY_TELNET_USERNAME)
   #define TELNET_USERNAME "root"                                               // Default Telnet user name is root
#endif
#if !defined(XY_TELNET_PASSWORD)
   #define TELNET_PASSWORD ""                                                   // Default Telnet password is no password
#endif

// command replies in rval which is parsed using JSMN
#define XY_RESP_OK 0                                                            // return values in rval
#define XY_INVALID_JSON_OBJ -7                                                  // invalid JSON object
#define XY_INVALID_CMD -9                                                       // input object is not a valid command
#define XY_RESP_CRASH -101                                                      // camera crash
#define XY_RESP_INVALID -15                                                     // invalid response
#define XY_RESP_REQ_NEW_TOKEN -4                                                // request a new token (input object is empty)
#define XY_RESP_NO_SD_CARD -27                                                  // No SD Card was present
// maximum send and response bytes
#define XY_MSG_MAX_SND_LEN 100U                                                 // Maximum size request
#define XY_MSG_MAX_LEN 2000U                                                    // Maximum size reply

// Define how the jasmine parser reads the reply message (from logic related to the msg_id or top level)
#define XY_READ_ALL  0U                                                         //  read all the params and types
#define XY_READ_SPECIFIC 1U                                                     // reads the specific keys requested
//==============================================================================================================================================================
// Folder Defs         self.FileTypes = {"/":"Folder", ".ash":"Script", ".bmp":"Image", ".ico":"Image", ".jpg":"Image", ".mka":"Audio", ".mkv":"Video", "mp3":"Audio", ".mp4":"Video", ".mpg":"Video", ".png":"Image", ".txt":"Text", "wav":"Audio"}
//                self.ChunkSizes = [0.5,1,2,4,8,16,32,64,128,256]

// To initialte requests you must get the token
// Access token
// Request:
// {"msg_id":257,"token":0}
// Answer:
// { "rval": 0, "msg_id": 257, "param": 1 }
// param=1, 1 is a TOKEN, it could be any other INT (or string? or something else?)
//
// Via telnet you can test as :- (you need cyclone tcp for tcp/ip stack in pic32)
// tcp 0 0 0.0.0.0:7878 0.0.0.0:* LISTEN 745/network_message
// this lien tells you, that 7878 is open. So you have to be able to connect to it from another device. thus from windows
// telnet 192.168.42.1 7878
// and send this
// {"msg_id":257,"token":0}
#define XY_START_SESS_CMD 257U                                                  // Start Session "param"  '{"msg_id":257,"token":0}') call before 259 to start session
#define API_XY_INIT_SESS_STR ""msg_id":257,"token":0"
#define API_XY_INIT_SESSION(X)  { sprintf(X,"{\"msg_id\":267,\"token\":0, \"heartbeat\": 1}\r"); } // {"msg_id":257,"token":0}
#define API_XY_INIT_FAST "{\"msg_id\": 267,\"token\":0}"

#define XY_STOP_SESS_CMD 258U                                                   // Stop Session "param"
#define API_XY_STOP_SESSION(X,token)  { sprintf(X,"{\"msg_id\":258,\"token\":%d}\r",token); } // {"msg_id":258,"token":<id>}
#define API_XY_STOPDEF_FAST "{\"msg_id\": 258,\"token\":0}"                     // makes the string
#define API_XY_STOP_FAST(token) "{\"msg_id\": 258,\"token\":" #token "}"        // makes the string

// ======= AJAX messages to the JSON server ====================================
#define XY_FILE_CMD 1287U                                                       // sendCancelGetFile and sendCancelPutFile (String)  param sent_size
#define XY_API_FILE_CANCEL(X,token)  { sprintf(X,"{\"msg_id\":1287,\"token\":%d}\r",token); }
#define XY_API_FILE_CANCEL_FAST(token) "{\"msg_id\": 1287,\"token\":" #token "}"   // makes the string

#define XY_BITRATE_CMD 16U                                                      // ChangeBitRate (int16)
#define XY_API_BITRATE(X,token,bitr)  { sprintf(X,"{\"msg_id\":16,\"token\":%d,\"param\":%d}\r",token,bitr); }
#define XY_API_BITRATE_FAST(token,bitr) "{\"msg_id\": 16,\"token\":" #token ",\"param\":" #bitr "}"   // makes the string

#define XY_DIR_CMD 1283U                                                        // sendChangeDirectory (String)   "param"
#define XY_API_DIR(X,token,para)  { sprintf(X,"{\"msg_id\":1283,\"token\":%d,\"param\":%s}\r",token,para); }
#define XY_API_DIR_FAST(token,para) "{\"msg_id\": 1283,\"token\":" #token ",\"param\":" #para "}"   // makes the string

#define XY_CCS_CMD 770U                                                         // sendContinueCaptureStop (Null)
#define XY_API_CCS(X,token)  { sprintf(X,"{\"msg_id\":770,\"token\":%d}\r",token); }
#define XY_API_CCS_FAST(token) "{\"msg_id\": 770,\"token\":" #token "}"    // makes the string

#define XY_DEL_FILE_CMD 1281U                                                   // sendDeleteFile (string) "param"
#define XY_API_DEL_FILE(X,token,para)  { sprintf(X,"{\"msg_id\":1281,\"token\":%d,\"param\":%s}\r",token,para); }
#define XY_API_DEL_FAST(token,para) "{\"msg_id\": 1281,\"token\":" #token ",\"param\":" #para "}"   // makes the string

#define XY_RESET_CMD 259U                                                       // sendForceResetVF
#define XY_API_RESET(X,token)  { sprintf(X,"{\"msg_id\":259,\"token\":%d}\r",token); }
#define XY_API_RESET_FAST(token) "{\"msg_id\": 259,\"token\":" #token "}"    // makes the string

#define XY_SND_FMT_CMD 4U                                                       // sendFormat(String s) "param"
#define XY_API_SND_FMT(X,token,para)  { sprintf(X,"{\"msg_id\":4,\"token\":%d,\"param\":%s}\r",token,para); }
#define XY_API_FMT_FAST(token,para) "{\"msg_id\": 4,\"token\":" #token ",\"param\": \"" #para "\"}"   // makes the string

// start event for status updates
#define XY_STATUS_CMD 7U                                                        // statusUpdate() of various items may be sent automatically
#define XY_API_STATUS(X,token,type)  { sprintf(X,"{\"msg_id\":7,\"token\":%d,\"type\": \"%s\" }\r",token,type); }
#define XY_API_AUTOSTAT(X,token)  { sprintf(X,"{\"msg_id\":7,\"token\":%d }\r",token); }
#define XY_API_AUTOSTAT_FAST(token) "{\"msg_id\": 7,\"token\":" #token "}"      // without the type asks for status update event to be turned on
#define XY_API_STATUS_FAST(token,type) "{\"msg_id\": 7,\"token\":" #token ",\"type\": \"" #type "\"}"    // makes the string "type" : "battery"

// Camera config
// Request:
// {"msg_id":3,"token":1}
// Answer:
// { "rval": 0, "msg_id": 3, "param": [ { "camera_clock": "2015-04-07 02:32:29" }, { "video_standard": "NTSC" }, { "app_status": "idle" }, { "video_resolution": "1920x1080 60P 16:9" }, { "video_stamp": "off" }, { "video_quality": "S.Fine" }, { "timelapse_video": "off" }, { "capture_mode": "precise quality" }, { "photo_size": "16M (4608x3456 4:3)" }, { "photo_stamp": "off" }, { "photo_quality": "S.Fine" }, { "timelapse_photo": "60" }, { "preview_status": "on" }, { "buzzer_volume": "mute" }, { "buzzer_ring": "off" }, { "capture_default_mode": "precise quality" }, { "precise_cont_time": "60.0 sec" }, { "burst_capture_number": "7 p / s" }, { "restore_factory_settings": "on" }, { "led_mode": "all enable" }, { "dev_reboot": "on" }, { "meter_mode": "center" }, { "sd_card_status": "insert" }, { "video_output_dev_type": "tv" }, { "sw_version": "YDXJv22_1.0.7_build-20150330113749_b690_i446_s699" }, { "hw_version": "YDXJ_v22" }, { "dual_stream_status": "on" }, { "streaming_status": "off" }, { "precise_cont_capturing": "off" }, { "piv_enable": "off" }, { "auto_low_light": "on" }, { "loop_record": "off" }, { "warp_enable": "off" }, { "support_auto_low_light": "on" }, { "precise_selftime": "5s" }, { "precise_self_running": "off" }, { "auto_power_off": "5 minutes" }, { "serial_number": "xxxxx" }, { "system_mode": "capture" }, { "system_default_mode": "capture" }, { "start_wifi_while_booted": "off" }, { "quick_record_time": "0" }, { "precise_self_remain_time": "0" }, { "sdcard_need_format": "no-need" }, { "video_rotate": "off" } ] }
#define XY_GET_SET_CMD 3U                                                       // sendGetAllCurrentSetting() sendGetAllSettingOption(String s) "param"
#define XY_API_GET_SET(X,token,para)  { sprintf(X,"{\"msg_id\":3,\"token\":%d,\"param\":%s}\r",token,para); }
#define XY_API_GET_SET_FAST(token,para) "{\"msg_id\": 3,\"token\":" #token ",\"param\":" #para "}"   // makes the string
#define XY_API_GET_SET_ALL(X,token)  { sprintf(X,"{\"msg_id\":3,\"token\":%d}\r",token); }
#define XY_API_GET_SET_ALL_FAST(token) "{\"msg_id\": 3,\"token\":" #token "}"   // makes the string "msg_id":3,"token":1

#define XY_BAT_LVL_CMD 13U                                                      // sendGetBatteryLevel()
#define XY_API_BAT_LVL(X,token)  { sprintf(X,"{\"msg_id\":%13,\"token\":%d}\r",token); }
#define XY_API_BAT_LVL_FAST(token) "{\"msg_id\": 13,\"token\":" #token "}"    // makes the string

#define XY_ZOOM_INFO_CMD 15U                                                    // sendGetDigitalZoomInfo(String s)  "type"
#define XY_API_ZOOM_INFO(X,token,para)  { sprintf(X,"{\"msg_id\":15,\"token\":%d,\"type\":%s}\r",token,para); }
#define XY_API_ZOOM_INFO_FAST(token,para) "{\"msg_id\": 15 ,\"token\":" #token ",\"param\":" #para "}"   // makes the string

#define XY_SND_GET_FIL_CMD 1285U                                                // sendGetFile(String s, long l, long l1)  "param" "offset" "fetch_size"
#define XY_API_GET_FIL(X,token,para,off,fet)  { sprintf(X,"{\"msg_id\":1285\"token\":%d,\"param\":%s,\"offset\": %d,\"fetch_size\":%d}\r",token,para,off,fet); }
#define XY_API_GET_FIL_FAST(token,para,off,fet) "{\"msg_id\": 1285,\"token\":" #token ",\"param\":" #para ",\"offset\":" #off ",\"fetch_size\":" #fet "}"  // makes the string

#define XY_SND_GET_MDA_CMD 1026U                                                // sendGetMediaInfo(String s) "param"
#define XY_API_GET_MDA(X,token,para)  { sprintf(X,"{\"msg_id\":1026,\"token\":%d,\"param\":%s}\r",token,para); }
#define XY_API_GET_MDA_FAST(token,para) "{\"msg_id\": 1026 ,\"token\":" #token ",\"param\":" #para "}"   // makes the string

#define XY_SND_GET_RCT_CMD 515U                                                 // sendGetRecordTime()
#define XY_API_GET_RCT(X,token)  { sprintf(X,"{\"msg_id\":515,\"token\":%d}\r",token); }
#define XY_API_GET_RCT_FAST(token) "{\"msg_id\": 515 ,\"token\":" #token "}"   // makes the string

// gets the single setting {"msg_id":1,"type":"video_resolution","token":1}
#define XY_SND_GETSET_CMD 1U                                                    // sendGetSetting(String s) "type"
#define XY_API_SND_GETSET(X,token,type)  { sprintf(X,"{\"msg_id\":1,\"token\":%d,\"type\":%s}\r",token,type); }
#define XY_API_GETSET_FAST(token,para,type) "{\"msg_id\": 1 ,\"token\":" #token ",\"param\":" #para ",\"type\":" #type "}"  // makes the string

// shows available choices for an option e.g {"msg_id":9,"param":"video_resolution","token":1}
#define XY_SND_GET_SETO_CMD 9U                                                  // sendGetSingleSettingOptions(String s) "param" (+ "options")
#define XY_API_GET_SETO(X,token,para)  { sprintf(X,"{\"msg_id\":9,\"token\":%d,\"param\":%s}\r",token,para); }
#define XY_API_SET_OPTION(X,token,para,opt)  { sprintf(X,"{\"msg_id\":9,\"token\":%d, \"permission\": \"settable\" , \"param\":%s, \"options\": %s}\r",token,para,opt); }
#define XY_API_SET_GET_SETO_FAST(token,para) "{\"msg_id\": 9 ,\"token\":" #token ",\"param\":" #para "}" // makes the string
#define XY_API_SET_OPTION_FAST(token,para,opt) "{\"msg_id\": 9 ,\"token\":" #token ", \"permission\" : \"settable\" , \"param\":" #para ", \"options\": " #opt "}" // makes the string

#define XY_SND_GET_SPC_CMD 5U                                                   // sendGetSpace(String s) "type"
#define XY_API_GET_SPC_TOT(X,token)  { sprintf(X,"{\"msg_id\":5,\"token\":%d,\"type\": \"total\"}\r",token); }      // type = "total" or "free"
#define XY_API_GET_SPC_FREE(X,token)  { sprintf(X,"{\"msg_id\":5,\"token\":%d,\"type\": \"free\"}\r",token); }      // type = "total" or "free"
#define XY_API_GET_SPC_FAST(token,type) "{\"msg_id\": 5,\"token\":" #token ",\"type\":" #type "}"  // makes the string

#define XY_SND_GET_TH_CMD 1025U                                                 // sendGetThumb(String s, String s1)  "type" "param"
#define XY_API_SND_GET_TH(X,token,type,param)  { sprintf(X,"{\"msg_id\":1025,\"token\":%d,\"type\":%s,\"param\" %s}\r",token,type,param); }
#define XY_API_GET_TH_FAST(token,type,para) "{\"msg_id\": 1025 ,\"token\":" #token ",\"type\":" #type ",\"param\":" #para "}"  // makes the string

#define XY_SND_STATE_IDLE_CMD 0x100000dL                                        // sendIntoIdleStateMode()
#define XY_API_SND_STATE_IDLE(X,token)  { sprintf(X,"{\"msg_id\":16777229,\"token\":%d}\r",token); }
#define XY_API_SND_STATE_IDLE_FAST(token) "{\"msg_id\": 16777229 ,\"token\":" #token "}"    // makes the string

#define XY_BBD_CMD 0x100000fL                                                   // sendIsBindedBluetoothDevs()
#define XY_API_BBD(X,token)  { sprintf(X,"{\"msg_id\":16777231,\"token\":%d}\r",token); }
#define XY_API_BBD_FAST(token) "{\"msg_id\":16777231 ,\"token\":" #token "}"    // makes the string

#define XY_SND_LIST_CMD 1282U                                                   // sendListing() sendListingWithOption(String s)  option  (-D -S)  "param"
#define XY_API_SND_LIST(X,token)  { sprintf(X,"{\"msg_id\":1282,\"token\":%d}\r",token); }
#define XY_API_SND_LIST_FAST(token) "{\"msg_id\": 1282 ,\"token\":" #token "}"   // makes the string
#define XY_API_SND_LIST_OPT(X,token,para)  { sprintf(X,"{\"msg_id\":1282,\"token\":%d, -S -D "param" %s}\r",token,para); }
#define XY_API_SND_LIST_OPT_FAST(token,para) "{\"msg_id\": 1282 ,\"token\":" #token ", -S -D \"param\":" #para "}"   // makes the string

#define XY_SND_STATE_WAKE_CMD 0x100000eL                                        // sendOutIdleStateMode()
#define XY_API_SND_STATE_WAKE(X,token)  { sprintf(X,"{\"msg_id\":16777220,\"token\":%d}\r",token); }
#define XY_API_SND_STATE_WAKE_FAST(token) "{\"msg_id\": 16777220,\"token\":" #token "}"    // makes the string

#define XY_SND_PIV_CMD 32U                                                      // sendPIV()  0x100000b
#define XY_API_SND_PIV(X,token)  { sprintf(X,"\"msg_id\":32,\"token\":%d\r",token); }
#define XY_API_SND_PIV_FAST(token) "{\"msg_id\": 32 ,\"token\":" #token "}"    // makes the string

#define XY_SND_PUT_FILE_CMD 1286U                                               // sendPutFIle(File file, String s)  "md5sum" "param" "size" "offset"=0U the param can enable a script //{"msg_id":1286,"token":%s,"param":"enable_info_display.script", "offset":0, "size":0, "md5sum":"d41d8cd98f00b204e9800998ecf8427e"}}
#define XY_API_SND_PUT_FILE(X,token,md,para,sz)  { sprintf(X,"{\"msg_id\":1286,\"token\":%d, \"md5sum\" %s, \"param\":%s, \"size\" %d, \"offset\"=0}\r",token,md,para,sz); }  // mds file contains settings created and used by TestComplete, an application that allows users to test web, desktop, and mobile programs. It stores project settings that are common for testers, such as the names of project items and the scripting language.

#define XY_SND_QCK_REC_PAU_CMD 0x1000006L                                       // sendQuickRecordPause()
#define XY_API_SND_QCK_REC_PAU(X,token)  { sprintf(X,"{\"msg_id\":16777222,\"token\":%d}\r",token); }
#define XY_API_SND_QCK_REC_PAU_FAST(token) "{\"msg_id\": 16777222 ,\"token\":" #token "}"    // makes the string

#define XY_SND_QCK_REC_RES_CMD 0x1000007L                                       // sendQuickRecordResume()
#define XY_API_SND_QCK_REC_RES(X,token)  { sprintf(X,"{\"msg_id\":16777223,\"token\":%d}\r",token); }
#define XY_API_SND_QCK_REC_RES_FAST(token) "{\"msg_id\": 16777223 ,\"token\":" #token "}"    // makes the string

#define XY_SND_QCK_REC_STRT_CMD 0x1000005L                                      // sendQuickRecordStart()
#define XY_API_SND_QCK_REC_STRT(X,token)  { sprintf(X,"{\"msg_id\":16777221,\"token\":%d}\r",token); }
#define XY_API_SND_QCK_REC_STRT_FAST(token) "{\"msg_id\": 16777221 ,\"token\":" #token "}"    // makes the string

#define XY_SND_RST_WIFI_CMD 0x1000008L                                          // sendRestartWiFi()
#define XY_API_SND_RST_WIFI(X,token)  { sprintf(X,"{\"msg_id\":16777224,\"token\":%d}\r",token); }   //    example --- const char * const xy_rst_msg[32]; API_XY_SND_RST_WIFI((char *) &xy_rst_msg,20,XY_SND_RST_VF_CMD);
#define XY_API_SND_RST_WIFI_FAST(token) "{\"msg_id\": 16777224,\"token\":" #token "}"

#define XY_SND_CAP_MDE_CMD 0x100000cUL                                          // sendSetCaptureMode(String s) "param"
#define XY_API_SND_CAP_MDE(X,token,para)  { sprintf(X,"{\"msg_id\":1677228,\"token\":%d, "param" %s}\r",token,para); }
#define XY_API_SND_CAP_MDE_FAST(token) "{\"msg_id\": 1677228 ,\"token\":" #token "}"    // makes the string

#define XY_SND_SET_ZOOM_CMD 14U                                                 // sendSetDigitalZoom(String s, String s1) "type" "param"
#define XY_API_SND_SET_ZOOM(X,token,typ,para)  { sprintf(X,"{\"msg_id\":14,\"token\":%d, \"type\" %s, \"param\" %s}\r",token,typ,para); }
#define XY_API_SND_SET_FAST_ZOOM(X,token,para)  { sprintf(X,"{\"msg_id\":14,\"token\":%d, \"type\" : \"fast\" , \"param\" %s}\r",token,para); }
#define XY_API_SND_SET_ZOOM_FAST(token,typ,para) "{\"msg_id\": 14 ,\"token\":" #token ",\"type\": # typ ",\"param\":" #para "}"   // makes the string

#define XY_SND_SET_SET_CMD 2U                                                   // sendSetSetting(String s, String s1) "type" "param"
// #define XY_API_SND_SET(X,token,typ,para)  { sprintf(X,"{\"msg_id\":2,\"token\":%d, \"type\" %s, \"param\" %s}\r",token,typ,para); }
// might be needed on some systems ---- #define XY_API_SND_SET(X,token,typ,para)  { sprintf(X," \{ \"msg_id\":2\, \"token\":%d\, \"type\"\:%s\, \"param\"\:%s \} \r",token,typ,para); }
#define XY_API_SND_SET(X,token,typ,para)  { sprintf(X," { \"msg_id\":2 , \"token\":%d , \"type\":%s , \"param\":%s } \r",token,typ,para); }
#define XY_API_CONFIG_DEFAULT(X,token)  { sprintf(X," { \"msg_id\":2 , \"token\":%d , \"type\":\"restore_factory_settings\" , \"param\":\"on\"} \r",token); }
#define XY_API_SND_ON(X,token,typ)  { sprintf(X," { \"msg_id\":2 , \"token\":%d , \"type\":%s , \"param\":\"on\" } \r",token,typ); }
#define XY_API_SND_OFF(X,token,typ)  { sprintf(X," { \"msg_id\":2 , \"token\":%d , \"type\":%s , \"param\":\"off\" } \r",token,typ); }
#define XY_API_LOG_START(X,token)  { sprintf(X,"{\"msg_id\":2,\"token\":%d, \"type\" \"save_log\", \"param\" \"on\"}\r",token); }
#define XY_API_SND_SET_FAST(token,typ,para) "{\"msg_id\": 2 ,\"token\":" #token ",\"type\":" #typ ",\"param\":" #para "}"
#define XY_API_LOG_START_FAST(token) "{\"msg_id\":2,\"type\":\"save_log\",\"param\":\"on\",\"" #token"\":1}"  // {"msg_id":2,"type":"save_log","param":"on","token":1} enable log files to look you tail -f /tmp/fuse_a/firmware.avtive.log

#define XY_SND_STRT_REC_CMD 513U                                                // sendStartRecord() '{"msg_id":513,"token":%s}' %token}
#define XY_API_START_REC(X,token)  { sprintf(X,"{\"msg_id\":513\"token\":%d}\r",token); }
#define XY_API_START_REC_FAST(token) "{\"msg_id\": 513 ,\"token\":" #token "}"    // makes the string

#define XY_SND_STOP_REC_CMD 514U                                                // sendStopRecord() "msg_id":514,"token":%
#define XY_API_STOP_REC(X,token)  { sprintf(X,"{\"msg_id\":514,\"token\":%d}\r",token); }
#define XY_API_STOP_REC_FAST(token) "{\"msg_id\": 514 ,\"token\":" #token "}"    // makes the string

#define XY_SND_RST_VF_CMD 259U                                                  // sendResetVF( none_force ) '{"msg_id":259,"token":%s,"param":"none_force"}'
#define XY_API_SND_RST_VF(X,token)  { sprintf(X,"{\"msg_id\":259,\"token\":%d,\"param\":\"none_force\"}\r",token); }
#define XY_API_SND_RST_VF_FAST(token) "{\"msg_id\": 259 ,\"token\":" #token "}"   // makes the string
#define XY_API_RESET_STREAM(X,token)   { sprintf(X,"{\"msg_id\":2,\"token\":%d, \"param\" \"vf_start\"}\r",token); }
#define XY_API_RESET_STREAM_FAST(token) "{\"msg_id\": 259 ,\"token\":" #token "\"type\": \"vf_start\"}"   // makes the string

#define XY_SND_STOP_VF_CMD 260U                                                 //  sendStopVF()  Stop the stream Syncro Life  "type vf_stop ?"
#define XY_API_STOP_VF(X,token)  { sprintf(X,"{\"msg_id\":260,\"token\":%d}\r",token); }  // msg id and token
#define XY_API_STOP_VF_FAST(token) "{\"msg_id\": 260 ,\"token\":" #token "}"   // makes the string
#define XY_API_STOP_STREAM(X,token) { sprintf(X,"{\"msg_id\":260,\"token\":%d , \"type\" : \"vf_stop\" }\r",token); }   // makes the string
#define XY_API_STOP_STREAM_FAST(token) "{\"msg_id\": 260 ,\"token\":" #token "\"type\": \"vf_stop\"}"   // makes the string

// Photo capture
// Request:
// {"msg_id":769,"token":1}
// Answers:
// { "msg_id": 7, "type": "start_photo_capture" ,"param":"precise quality;off"}
// { "msg_id": 7, "type": "photo_taken" ,"param":"/tmp/fuse_d/DCIM/100MEDIA/YDXJ0047.jpg"}
// photo will be stored on SD card as DCIM/100MEDIA/YDXJ0047.jpg

#define XY_SND_TK_PHO_CMD 769U                                                  // sendTakePhoto() {"msg_id":769,"token":%s}' %token}
#define XY_API_TK_PHO(X,token)  { sprintf(X,"{\"msg_id\":769,\"token\":%d}\r",token); }
#define XY_API_TK_PHO_FAST(token) "{\"msg_id\": 769 ,\"token\":" #token "}"    // makes the string

#define XY_SND_TK_PHOMD_CMD 0x1000004L                                          // sendTakePhotoWithMode(String s) "param"
#define XY_API_SND_TK_PHOMD(X,token,para)  { sprintf(X,"{\"msg_id\":16777220,\"token\":%d, \"param\" %s}\r",token,para); }
#define XY_API_XY_SND_TK_PHOMD_FAST(token,para) "{\"msg_id\": 16777220 ,\"token\":" #token ",\"param\":" #para "}"   // makes the string

#define XY_SND_UBIND_BLUE_CMD 0x1000010L                                        // sendUnbindBluetoothDevs()
#define XY_API_UBIND_BLUE(X,token)  { sprintf(X,"{\"msg_id\":16777232,\"token\":%d}\r",token); }
#define XY_API_UBIND_BLUE_FAST(token) "{\"msg_id\": 16777232 ,\"token\":" #token "}"    // makes the string

#define XY_SND_UPGRD_CMD  0x1000003L                                            // sendUpgrade(String s) "param"
#define XY_API_SND_UPGRD(X,token,para)  { sprintf(X,"{\"msg_id\":16777219,\"token\":%d, "param" %s}\r",token,para); }
#define XY_API_SND_UPGRD_FAST(token,para) "{\"msg_id\": 16777219 ,\"token\":" #token "\"param\":" #para "}"   // makes the string

#define XY_SET_IPADDR_CMD 261U                                                  // setIPAddress() "param" dotted ip example "192.11.1.5"
#define XY_API_SET_IPADDR(X,para,token)  { sprintf(X,"{\"msg_id\":261, "param" %s, \"token\":%d}\r",para,token); }
#define XY_API_SET_IPADDR_FAST(token,para) "{\"msg_id\": 261, \"param\":" #para ",\"token\":" #token "}"   // makes the string

#define XY_UNKNOWN_CMD 1027U                                                    // param": "/tmp/fuse_d/DCIM/129MEDIA/YDXJ2007.jpg", "type": 100

#define XY_GET_LAST_PHO_CMD 16777236L
#define XY_API_GET_LAST_PHO(X,token)  { sprintf(X,"{\"msg_id\":16777236,\"token\":%d}\r",token); }  // getLastPhoto() - return the last taken picture

#define XY_DUMP_FWARE 16777242L
#define XY_API_DUMP_FWARE(X,token)  { sprintf(X,"{\"msg_id\":16777242,\"token\":%d, \"param\":'/DCIM/firmware.log' }\r",token); }  // dump firmware

// {"msg_id":3,"token":%s,"param":"%s"}
// {"msg_id":2,"type":"dev_reboot","param":"on"}
#if defined(D_FT900)
typedef struct CAMPACKED {
        int16_t token;                                                          // token value returned (this is the handle for further communication with the webserver)
        int16_t rval;                                                           // return value from the command
        int32_t msg_id;                                                         // the message id of the message it is replying to
        uint32_t size;                                                          // size returned from enquiry
        unsigned char resolution[10u];                                          // resolution string returned
        unsigned char param_str[XY_MSG_MAX_LEN];                                // container to hold all the params returned as a string
        unsigned char type_str[XY_MSG_MAX_LEN];                                 // container to hold all the types returned as a string
        unsigned char video_std[5u];                                            // video_standard NTSC or PAL
        unsigned char video_res[25u];                                           // video resolution returned
        unsigned char photo_sz[15u];                                            // photo size returned
        unsigned char burst_cap[15u];                                           // burst capture size returned
        unsigned char md5sum[MD5_MAX_LEN];                                      // returned from md5sum tag in reply string
} XY_reply_t
#else
CAMPACKED(
typedef struct {
        int16_t token;                                                          // token value returned (this is the handle for further communication with the webserver)
        int16_t rval;                                                           // return value from the command
        int32_t msg_id;                                                         // the message id of the message it is replying to
        uint32_t size;                                                          // size returned from enquiry
        unsigned char resolution[10u];                                           // resolution string returned
        unsigned char param_str[XY_MSG_MAX_LEN];                                // container to hold all the params returned as a string
        unsigned char type_str[XY_MSG_MAX_LEN];                                 // container to hold all the types returned as a string
        unsigned char video_std[5u];                                             // video_standard NTSC or PAL
        unsigned char video_res[25u];                                            // video resolution returned
        unsigned char photo_sz[15u];                                             // photo size returned
        unsigned char burst_cap[15u];                                            // burst capture size returned
        unsigned char md5sum[MD5_MAX_LEN];                                      // returned from md5sum tag in reply string
}) XY_reply_t;                                                                  // typical JSON reply container
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        unsigned char camera_clock[25u];                                         // "2015-04-07 02:32:29"
        unsigned char video_standard[6u];                                        // "NTSC"
        unsigned char app_status[15u];                                           // "idle"
        unsigned char video_resolution[25u];                                     // "1920x1080 60P 16:9"
        unsigned char video_stamp[11u];                                          // "off,
        unsigned char video_quality[10u];                                        // "S.Fine
        unsigned char timelapse_video[10u];                                      // "off
        unsigned char capture_mode[25u];                                         // "precise quality
        unsigned char photo_size[25u];                                           // "16M (4608x3456 4:3
        unsigned char photo_stamp[11u];                                          // "off
        unsigned char photo_quality[10u];                                        // "S.Fine
        unsigned char timelapse_photo[5u];                                       // "60
        unsigned char preview_status[5u];                                        // "on
        unsigned char buzzer_volume[10u];                                        // "mute
        unsigned char buzzer_ring[5u];                                           // "off
        unsigned char capture_default_mode[25u];                                 // "precise quality
        unsigned char precise_cont_time[10u];                                    // "60.0 sec
        unsigned char burst_capture_number[15u];                                 // "7 p / s
        unsigned char restore_factory_settings[5u];                              // "on
        unsigned char led_mode[20u];                                             // "all enable
        unsigned char dev_reboot[5u];                                            // "on
        unsigned char meter_mode[10u];                                           // "center
        unsigned char sd_card_status[10u];                                       // "insert
        unsigned char video_output_dev_type[10u];                                // "tv
        unsigned char sw_version[60u];                                           // "YDXJv22_1.0.7_build-20150330113749_b690_i446_s699
        unsigned char hw_version[32u];                                           // "YDXJ_v22
        unsigned char dual_stream_status[10u];                                   // "on
        unsigned char streaming_status[10u];                                     // "off
        unsigned char precise_cont_capturing[10u];                               // "off
        unsigned char piv_enable[10u];                                           // "off
        unsigned char auto_low_light[10u];                                       // "on
        unsigned char loop_record[10u];                                          // "off
        unsigned char warp_enable[10u];                                          // "off
        unsigned char support_auto_low_light[10u];                               // "on
        unsigned char precise_selftime[10u];                                     // "5s
        unsigned char precise_self_running[10u];                                 // "off
        unsigned char auto_power_off[10u];                                       // "5 minutes
        unsigned char serial_number[10u];                                        // "xxxxx
        unsigned char system_mode[15u];                                          // "capture
        unsigned char system_default_mode[15u];                                  //  "capture
        unsigned char start_wifi_while_booted[15u];                              // "off
        unsigned char quick_record_time[10u];                                    // "0
        unsigned char precise_self_remain_time[10u];                             // "0
        unsigned char sdcard_need_format[10u];                                   // "no-need
        unsigned char video_rotate[10u];                                         // "off
} XY_config_t;
#else
CAMPACKED(
typedef struct {                                                                // Typicsl replies shown below
        unsigned char camera_clock[25u];                                         // "2015-04-07 02:32:29"
        unsigned char video_standard[6u];                                        // "NTSC"
        unsigned char app_status[15u];                                           // "idle"
        unsigned char video_resolution[25u];                                     // "1920x1080 60P 16:9"
        unsigned char video_stamp[11u];                                          // "off,
        unsigned char video_quality[10u];                                        // "S.Fine
        unsigned char timelapse_video[10u];                                      // "off
        unsigned char capture_mode[25u];                                         // "precise quality
        unsigned char photo_size[25u];                                           // "16M (4608x3456 4:3
        unsigned char photo_stamp[11u];                                          // "off
        unsigned char photo_quality[10u];                                        // "S.Fine
        unsigned char timelapse_photo[5u];                                       // "60
        unsigned char preview_status[5u];                                        // "on
        unsigned char buzzer_volume[10u];                                        // "mute
        unsigned char buzzer_ring[5u];                                           // "off
        unsigned char capture_default_mode[25u];                                 // "precise quality
        unsigned char precise_cont_time[10u];                                    // "60.0 sec
        unsigned char burst_capture_number[15u];                                 // "7 p / s
        unsigned char restore_factory_settings[5u];                              // "on
        unsigned char led_mode[20u];                                             // "all enable
        unsigned char dev_reboot[5u];                                            // "on
        unsigned char meter_mode[10u];                                           // "center
        unsigned char sd_card_status[10u];                                       // "insert
        unsigned char video_output_dev_type[10u];                                // "tv
        unsigned char sw_version[60u];                                           // "YDXJv22_1.0.7_build-20150330113749_b690_i446_s699
        unsigned char hw_version[32u];                                           // "YDXJ_v22
        unsigned char dual_stream_status[10u];                                   // "on
        unsigned char streaming_status[10u];                                     // "off
        unsigned char precise_cont_capturing[10u];                               // "off
        unsigned char piv_enable[10u];                                           // "off
        unsigned char auto_low_light[10u];                                       // "on
        unsigned char loop_record[10u];                                          // "off
        unsigned char warp_enable[10u];                                          // "off
        unsigned char support_auto_low_light[10u];                               // "on
        unsigned char precise_selftime[10u];                                     // "5s
        unsigned char precise_self_running[10u];                                 // "off
        unsigned char auto_power_off[10u];                                       // "5 minutes
        unsigned char serial_number[10u];                                        // "xxxxx
        unsigned char system_mode[15u];                                          // "capture
        unsigned char system_default_mode[15u];                                  //  "capture
        unsigned char start_wifi_while_booted[15u];                              // "off
        unsigned char quick_record_time[10u];                                    // "0
        unsigned char precise_self_remain_time[10u];                             // "0
        unsigned char sdcard_need_format[10u];                                   // "no-need
        unsigned char video_rotate[10u];                                         // "off
}) XY_config_t;                                                                 // Config JSON reply container, This is a readback of configuration readback from param field in response to {"msg_id":3, "token":1}
#endif

// defines a word showing the JSON parser message tags found in the reply or the msb to show an error or zero (nothing found)
#define _XY_TOKEN (1U<<0U)
#define _XY_RVAL (1U<<1U)
#define _XY_MSG_ID (1U<<2U)
#define _XY_PARAM (1U<<3U)
#define _XY_TYPE (1U<<4U)
#define  _XY_VIDEO_STD (1U<<5U)
#define _XY_VIDEO_RES (1U<<6U)
#define _XY_PHOTO_SZ (1U<<7U)
#define _XY_BURST_CAP (1U<<8U)
#define _XY_UNEXPECT (1U<<9U)
#define _XY_MD5SUM (1U<<10U)
#define _XY_SIZE (1U<<11U)
#define _XY_RESOL (1U<<12U)
#define _XY_ERROR (1U<<15U)

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint16_t f_token  : 1u;                                                  // found token
        uint16_t f_rval  : 1u;                                                   // found rval
        uint16_t f_msg_id  : 1u;                                                 // found msg id
        uint16_t f_param  : 1u;                                                  // found param
        uint16_t f_type  : 1u;                                                   // found type
        uint16_t f_vid_std  : 1u;                                                // found video standard
        uint16_t f_vid_res  : 1u;                                                // found video resolution
        uint16_t f_photo_sz : 1u;                                                // found photo size
        uint16_t f_burst_cap  : 1u;                                              // found burst cap
        uint16_t f_unex  : 1u;                                                   // found unexpected key
        uint16_t f_md5sum : 1u;                                                  // found a md5sum key
        uint16_t f_size : 1u;                                                    // found a size key
        uint16_t f_resol : 1u;                                                   // found a resolution key
        uint16_t spare : 2u;                                                     // unused bits
        uint16_t f_error  : 1u;                                                  // error occurred
} XY_JSparse_t;
#else
CAMPACKED(
typedef struct  {
        uint16_t f_token  : 1u;                                                  // found token
        uint16_t f_rval  : 1u;                                                   // found rval
        uint16_t f_msg_id  : 1u;                                                 // found msg id
        uint16_t f_param  : 1u;                                                  // found param
        uint16_t f_type  : 1u;                                                   // found type
        uint16_t f_vid_std  : 1u;                                                // found video standard
        uint16_t f_vid_res  : 1u;                                                // found video resolution
        uint16_t f_photo_sz : 1u;                                                // found photo size
        uint16_t f_burst_cap  : 1u;                                              // found burst cap
        uint16_t f_unex  : 1u;                                                   // found unexpected key
        uint16_t f_md5sum : 1u;                                                  // found a md5sum key
        uint16_t f_size : 1u;                                                    // found a size key
        uint16_t f_resol : 1u;                                                   // found a resolution key
        uint16_t spare : 2u;                                                     // unused bits
        uint16_t f_error  : 1u;                                                  // error occurred
}) XY_JSparse_t;                                                                // Struct to the status of the json parse
#endif
// low level tcp flags for the header we need to get to this if we need nagle algorythm ???
typedef enum
{
   TCP_FLAG_FIN = 0x01U,                                                        // FIN in header message Net_Ethernet_Intern_disconnectTCP
   TCP_FLAG_SYN = 0x02U,                                                        // SYN in header message Net_Ethernet_Intern_connectTCP
   TCP_FLAG_RST = 0x04U,                                                        // RST in header
   TCP_FLAG_PSH = 0x08U,                                                        // PSH in header
   TCP_FLAG_ACK = 0x10U,                                                        // ACK in header message Net_Ethernet_Intern_startSendTCP
   TCP_FLAG_URG = 0x20U                                                         // URG in header
} TcpFlags;                                                                     // Flags for the TCP header

#if defined(D_FT900)
#ifdef _CPU_BIG_ENDIAN                                                          // If we are big endian swap the bytes around
typedef struct CAMPACKED {
   uint16_t srcPort;                                                            //0-1
   uint16_t destPort;                                                           //2-3
   uint32_t seqNum;                                                             //4-7
   uint32_t ackNum;                                                             //8-11
                                                                                // swap bytes if we are big endian
   uint8_t dataOffset : 4u;                                                      //12
   uint8_t reserved1 : 4u;
   uint8_t reserved2 : 2u;                                                       //13
   uint8_t flags : 6u;
   uint16_t window;                                                             //14-15
   uint16_t checksum;                                                           //16-17
   uint16_t urgentPointer;                                                      //18-19
   uint8_t options[];                                                           //20
} TCP_header_t;                                                                // TCP header if needed for PSH URG etc
#else
typedef struct CAMPACKED {
   uint16_t srcPort;                                                            //0-1
   uint16_t destPort;                                                           //2-3
   uint32_t seqNum;                                                             //4-7
   uint32_t ackNum;                                                             //8-11
   uint8_t reserved1 : 4u;                                                       //12
   uint8_t dataOffset : 4u;
   uint8_t flags : 6u;                                                           //13
   uint8_t reserved2 : 2u;
   uint16_t window;                                                             //14-15
   uint16_t checksum;                                                           //16-17
   uint16_t urgentPointer;                                                      //18-19
   uint8_t options[];                                                           //20
} TCP_header_t;                                                                // TCP header if needed for PSH URG etc
#endif
#else
#ifdef _CPU_BIG_ENDIAN                                                          // If we are big endian swap the bytes around
CAMPACKED(
typedef struct {
   uint16_t srcPort;                                                            //0-1
   uint16_t destPort;                                                           //2-3
   uint32_t seqNum;                                                             //4-7
   uint32_t ackNum;                                                             //8-11
                                                                                // swap bytes if we are big endian
   uint8_t dataOffset : 4u;                                                      //12
   uint8_t reserved1 : 4u;
   uint8_t reserved2 : 2u;                                                       //13
   uint8_t flags : 6u;
   uint16_t window;                                                             //14-15
   uint16_t checksum;                                                           //16-17
   uint16_t urgentPointer;                                                      //18-19
   uint8_t options[];                                                           //20
}) TCP_header_t;                                                                // TCP header if needed for PSH URG etc
#else
CAMPACKED(
typedef struct {
   uint16_t srcPort;                                                            //0-1
   uint16_t destPort;                                                           //2-3
   uint32_t seqNum;                                                             //4-7
   uint32_t ackNum;                                                             //8-11
   uint8_t reserved1 : 4u;                                                       //12
   uint8_t dataOffset : 4u;
   uint8_t flags : 6u;                                                           //13
   uint8_t reserved2 : 2u;
   uint16_t window;                                                             //14-15
   uint16_t checksum;                                                           //16-17
   uint16_t urgentPointer;                                                      //18-19
   uint8_t options[];                                                           //20
}) TCP_header_t;                                                                // TCP header if needed for PSH URG etc
#endif
#endif

#define XY_TCP_TIMEOUT 10000U                                                   // Re-try timer on TCP state lockup

#if defined(D_FT900)
typedef struct CAMPACKED {
   uint32_t tcpClosedTm;                                                        // We dont have a connection
   uint32_t tcpListenTm;                                                        // Server is listening for a SYN_ACK from the client
   uint32_t tcpSynSentTm;                                                       // You need to look for a SYN
   uint32_t tcpFinWait1Tm;                                                      // You sent a FIN
   uint32_t tcpFinWait2Tm;                                                      // You wait for a remote FIN
   uint32_t tcpRetransTm;                                                       // Asked to re-transmit
} TcpStateTimers;
#else
CAMPACKED(
typedef struct {
   uint32_t tcpClosedTm;                                                        // We dont have a connection
   uint32_t tcpListenTm;                                                        // Server is listening for a SYN_ACK from the client
   uint32_t tcpSynSentTm;                                                       // You need to look for a SYN
   uint32_t tcpFinWait1Tm;                                                      // You sent a FIN
   uint32_t tcpFinWait2Tm;                                                      // You wait for a remote FIN
   uint32_t tcpRetransTm;                                                       // Asked to re-transmit
}) TcpStateTimers;                                                              // MIB error time outs for each TCP state
#endif

// =============================================================================
// ------------------------ Parrot Sequoia Camera HTTP-API ---------------------
// =============================================================================
#define SEQ_USE_IP "192.168.47.1"                                               // fixed IP for sequioa cameraz
#define SEQ_DEF_PORT 80U                                                        // http port (no support as yet for ssl)
#define SEQ_MSG_MAX_LEN 260U                                                    // Maximum size for the sequoia messages ? To be set after test

// =============================================================================
//
// ============ AMP Protocol ===================================================
//
//==============================================================================

// ============ other commands which are sent and set from the data structs ====
// Start Recording $RECORD, SC and XS
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[7u];                                               // keyword  RECORD  0x52. 0x45, 0x43, 0x4F, 0x52, 0x44
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
} SCORD_XS_record_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[7u];                                               // keyword  RECORD  0x52. 0x45, 0x43, 0x4F, 0x52, 0x44
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
}) SCORD_XS_record_t;
#endif

// Stop Recording $STOP, SC and XS
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[5u];                                               // keyword  STOP  0x52. 0x54, 0x4F, 0x50,
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
} SCORD_XS_stop_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[5u];                                               // keyword  STOP  0x52. 0x54, 0x4F, 0x50,
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
}) SCORD_XS_stop_t;
#endif

// Delete Recording $ERASE, SC and XS
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[6u];                                               // keyword  ERASE  0x45, 0x52. 0x41 0x53 0x45
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                          //   , = 0x2C
        unsigned char EraseType[9u];                                             // ALL or NOTMARKED
        uint8_t Etx;                                                            // CR = 0x0D
} SCORD_XS_erase_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[6u];                                               // keyword  ERASE  0x45, 0x52. 0x41 0x53 0x45
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                          //   , = 0x2C
        unsigned char EraseType[9u];                                             // ALL or NOTMARKED
        uint8_t Etx;                                                            // CR = 0x0D
}) SCORD_XS_erase_t;
#endif

// Mark Recording $MARK, XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
       uint8_t Stx;                                                            // First Char $ = 0x24
       unsigned char Keyword[5u];                                               // keyword  MARK  0x4D, 0x41. 0x52 0x4B
       uint8_t Comma;                                                          //   , = 0x2C
       int8_t Channel;                                                         // Channel (-1 means all)
       uint8_t Comma2;                                                         //   , = 0x2C
       unsigned char Label[];                                                  // label for the mark
       uint8_t Etx;                                                             // CR = 0x0D
} XS_mark_recording_t;
#else
CAMPACKED(
typedef struct {
       uint8_t Stx;                                                            // First Char $ = 0x24
       unsigned char Keyword[5u];                                               // keyword  MARK  0x4D, 0x41. 0x52 0x4B
       uint8_t Comma;                                                          //   , = 0x2C
       int8_t Channel;                                                         // Channel (-1 means all)
       uint8_t Comma2;                                                         //   , = 0x2C
       unsigned char Label[];                                                  // label for the mark
       uint8_t Etx;                                                             // CR = 0x0D
}) XS_mark_recording_t;
#endif

// Set $BITRATE, XS only
#if defined(D_FT900)
#ifdef CAM_USE_ASCII
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[8u];                                               // keyword  BITRATE  0x42, 0x49, 0x54, 0x52, 0x41, 0x54, 0x45
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiBitrate[8u];                                          // the bit rate
        uint8_t Etx;                                                            // CR = 0x0D
} XS_bitrate_t;
#else
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[8u];                                               // keyword  BITRATE  0x42, 0x49, 0x54, 0x52, 0x41, 0x54, 0x45
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint32_t bitrate;                                                       // the bit rate
        uint8_t Etx;                                                            // CR = 0x0D
} XS_bitrate_t;
#endif
#else
#ifdef CAM_USE_ASCII
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[8u];                                               // keyword  BITRATE  0x42, 0x49, 0x54, 0x52, 0x41, 0x54, 0x45
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiBitrate[8u];                                          // the bit rate
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_bitrate_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[8u];                                               // keyword  BITRATE  0x42, 0x49, 0x54, 0x52, 0x41, 0x54, 0x45
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint32_t bitrate;                                                       // the bit rate
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_bitrate_t;
#endif
#endif

// Get $GETBITRATE,  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                             // keyword  GETBITRATE  0x47, 0x45, 0x54 0x42, 0x49, 0x54, 0x52, 0x41, 0x54, 0x45
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_getbitrate_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                             // keyword  GETBITRATE  0x47, 0x45, 0x54 0x42, 0x49, 0x54, 0x52, 0x41, 0x54, 0x45
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_getbitrate_t;
#endif

// set IFRAMEINTERVAL,  XS only
#if defined(D_FT900)
#ifdef CAM_USE_ASCII
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[10u];                                               // keyword  IINTERVAL   49 49 4E 54 45 52 56 41 4C
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiInterval[3u];                                         // interval time
        uint8_t Etx;                                                            // CR = 0x0D
} XS_iframeinterval_t;
#else
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[10u];                                               // keyword  IINTERVAL   49 49 4E 54 45 52 56 41 4C
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t interval;                                                      // interval time
        uint8_t Etx;                                                            // CR = 0x0D
} XS_iframeinterval_t;
#endif
#else
#ifdef CAM_USE_ASCII
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[10u];                                               // keyword  IINTERVAL   49 49 4E 54 45 52 56 41 4C
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiInterval[3u];                                         // interval time
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_iframeinterval_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[10u];                                               // keyword  IINTERVAL   49 49 4E 54 45 52 56 41 4C
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t interval;                                                      // interval time
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_iframeinterval_t;
#endif
#endif

// Get $GETBITRATE,  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[13u];                                             // keyword  GETIINTERVAL  0x47, 0x45, 0x54 0x49 0x49 0x4E 0x54 0x45 0x52 0x56 0x41 0x4C
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_getiframeinterval_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[13u];                                             // keyword  GETIINTERVAL  0x47, 0x45, 0x54 0x49 0x49 0x4E 0x54 0x45 0x52 0x56 0x41 0x4C
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_getiframeinterval_t;
#endif

// set $FRAMERATE  XS only
#if defined(D_FT900)
#ifdef CAM_USE_ASCII
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[10u];                                               // keyword  FRAMERATE
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiRate[8u];                                             // Rate
        uint8_t Comma3;                                                         //   , = 0x2C
        unsigned char asciiScalar[8u];                                           // Scalar e.g. 15,2 means 7.5 fps
        uint8_t Etx;                                                            // CR = 0x0D
} XS_framerate_t;
#else
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[10u];                                               // keyword  FRAMERATE
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Rate;                                                          // Rate
        uint8_t Comma3;                                                         //   , = 0x2C
        uint8_t Scalar;                                                         // Scalar e.g. 15,2 means 7.5 fps
        uint8_t Etx;                                                            // CR = 0x0D
} XS_framerate_t;
#endif
#else
#ifdef CAM_USE_ASCII
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[10u];                                               // keyword  FRAMERATE
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiRate[8u];                                             // Rate
        uint8_t Comma3;                                                         //   , = 0x2C
        unsigned char asciiScalar[8u];                                           // Scalar e.g. 15,2 means 7.5 fps
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_framerate_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[10u];                                               // keyword  FRAMERATE
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Rate;                                                          // Rate
        uint8_t Comma3;                                                         //   , = 0x2C
        uint8_t Scalar;                                                         // Scalar e.g. 15,2 means 7.5 fps
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_framerate_t;
#endif
#endif

// set $GETFRAMERATE  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[13u];                                               // keyword  GETFRAMERATE
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_getframerate_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[13u];                                               // keyword  GETFRAMERATE
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_getframerate_t;
#endif

// set $BRIGHTNESS  XS only
#if defined(D_FT900)
#ifdef CAM_USE_ASCII
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                              // keyword  BRIGHTNESS
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiBrightness[8u];                                       // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_brightness_t;
#else
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                              // keyword  BRIGHTNESS
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Brightness;                                                    // Brightness
        uint8_t Etx;                                                            // CR = 0x0D
} XS_brightness_t;
#endif
#else
#ifdef CAM_USE_ASCII
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                              // keyword  BRIGHTNESS
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiBrightness[8u];                                       // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_brightness_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                              // keyword  BRIGHTNESS
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Brightness;                                                    // Brightness
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_brightness_t;
#endif
#endif

// set $GETBRIGHTNESS  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[14u];                                              // keyword  GETBRIGHTNESS
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_getbrightness_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[14u];                                              // keyword  GETBRIGHTNESS
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_getbrightness_t;
#endif

// set $CONTRAST  XS only     A test STUB CAM_USE_ASCII has been added
#if defined(D_FT900)
#ifdef CAM_USE_ASCII
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  CONTRAST
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiContrast[8u];                                         // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_contrast_t;
#else
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  CONTRAST
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Contrast;                                                      // Contrast
        uint8_t Etx;                                                            // CR = 0x0D
} XS_contrast_t;
#endif
#else
#ifdef CAM_USE_ASCII
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  CONTRAST
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiContrast[8u];                                         // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_contrast_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  CONTRAST
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Contrast;                                                      // Contrast
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_contrast_t;
#endif
#endif

// set $GETCONTRAST  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[12u];                                               // keyword  GETCONTRAST
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_getcontrast_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[12u];                                               // keyword  GETCONTRAST
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_getcontrast_t;
#endif

// set $HUE  XS only
#if defined(D_FT900)
#ifdef CAM_USE_ASCII
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[4u];                                               // keyword  HUE
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiHue[8u];                                              // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_hue_t;
#else
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[4u];                                               // keyword  HUE
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t hue;                                                           // Hue
        uint8_t Etx;                                                            // CR = 0x0D
} XS_hue_t;
#endif
#else
#ifdef CAM_USE_ASCII
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[4u];                                               // keyword  HUE
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiHue[8u];                                              // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_hue_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[4u];                                               // keyword  HUE
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t hue;                                                           // Hue
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_hue_t;
#endif
#endif

// get $HUE  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[7u];                                               // keyword  GETHUE
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_gethue_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[7u];                                               // keyword  GETHUE
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_gethue_t;
#endif

// set $SATURATION  XS only
#if defined(D_FT900)
#ifdef CAM_USE_ASCII
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                             // keyword  SATURATION
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiSaturation[8u];                                      // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_saturation_t;
#else
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                             // keyword  SATURATION
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Saturation;                                                    // SAturation
        uint8_t Etx;                                                            // CR = 0x0D
} XS_saturation_t;
#endif
#else
#ifdef CAM_USE_ASCII
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                             // keyword  SATURATION
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiSaturation[8u];                                      // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_saturation_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                             // keyword  SATURATION
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Saturation;                                                    // SAturation
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_saturation_t;
#endif
#endif

// set $GETSATURATION  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[14u];                                              // keyword  GETSATURATION
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_getsaturation_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[14u];                                              // keyword  GETSATURATION
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_getsaturation_t;
#endif

// set $Clip Size  XS only
#if defined(D_FT900)
#ifdef CAM_USE_ASCII
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  CLipSize
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiClipSz[8u];                                           // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_clipsize_t;
#else
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  CLipSize
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Size;                                                           // Size
        uint8_t Etx;                                                            // CR = 0x0D
} XS_clipsize_t;
#endif
#else
#ifdef CAM_USE_ASCII
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  CLipSize
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiClipSz[8u];                                           // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_clipsize_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  CLipSize
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Size;                                                           // Size
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_clipsize_t;
#endif
#endif

// get $Clip Size  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[12u];                                               // keyword  GETCLipSize
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_getclipsize_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[12u];                                               // keyword  GETCLipSize
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_getclipsize_t;
#endif

// set $Clip length  XS only
#if defined(D_FT900)
#ifdef CAM_USE_ASCII
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[14u];                                              // keyword  CLiplength
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiClipLn[8u];                                           // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_cliplength_t;
#else
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[14u];                                              // keyword  CLiplength
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Length;                                                        // Length
        uint8_t Etx;                                                            // CR = 0x0D
} XS_cliplength_t;
#endif
#else
#ifdef CAM_USE_ASCII
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[14u];                                              // keyword  CLiplength
        uint8_t Comma;                                                          //   , = 0x2C
        unsigned char asciiChannel[2u];                                          // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        unsigned char asciiClipLn[8u];                                           // This is the ascii value incase its needed (example)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_cliplength_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[14u];                                              // keyword  CLiplength
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Comma2;                                                         //   , = 0x2C
        uint16_t Length;                                                        // Length
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_cliplength_t;
#endif
#endif

// get $Clip length  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[17u];                                              // keyword  GETCLiplength
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
} XS_getcliplength_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[17u];                                              // keyword  GETCLiplength
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Channel;                                                         // Channel (-1 means all)
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_getcliplength_t;
#endif

// set RTSP (Real Time Streaming Protocol) Port  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  RTSPPORT
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Port;                                                            // Port
        uint8_t Etx;                                                            // CR = 0x0D
} XS_rtspport_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  RTSPPORT
        uint8_t Comma;                                                          //   , = 0x2C
        int8_t Port;                                                            // Port
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_rtspport_t;
#endif

// get RTSP (Real Time Streaming Protocol) Port  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[12u];                                               // keyword  GETRTSPPORT
        uint8_t Etx;                                                            // CR = 0x0D
} XS_getrtspport_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[12u];                                               // keyword  GETRTSPPORT
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_getrtspport_t;
#endif

// change the storage disk DATADISK XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  DATADISK
        uint8_t Comma;                                                          //   , = 0x2C
        uint8_t disk_no;                                                         // disk number as returned by ENUMDISK
        uint8_t Etx;                                                            // CR = 0x0D
} XS_datadisk_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[9u];                                               // keyword  DATADISK
        uint8_t Comma;                                                          //   , = 0x2C
        uint8_t disk_no;                                                         // disk number as returned by ENUMDISK
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_datadisk_t;
#endif

// change threshold for automatic file deletion RECYCLE XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[8u];                                               // keyword  RECYCLE
        uint8_t Comma;                                                          //   , = 0x2C
        uint8_t threshold;                                                       // threshold value in percent
        uint8_t Etx;                                                            // CR = 0x0D
} XS_recycle_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[8u];                                               // keyword  RECYCLE
        uint8_t Comma;                                                          //   , = 0x2C
        uint8_t threshold;                                                       // threshold value in percent
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_recycle_t;
#endif

// configure the RTSP (Real Time Streaming Protocol) Port  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
       uint8_t Stx;                                                            // First Char $ = 0x24
       unsigned char Keyword[7u];                                               // keyword  STREAM
       uint8_t Comma;                                                          //   , = 0x2C
       int8_t Port;                                                            // Port
       uint8_t Comma2;                                                         //   , = 0x2C
       unsigned char Cast[];                                                   // UNICAST or MULTICAST
       uint8_t Comma3;                                                          //   , = 0x2C
       uint8_t enable;                                                          // 1=enable 0-disable
       uint8_t Etx;                                                             // CR = 0x0D
} XS_stream_t;
#else
CAMPACKED(
typedef struct {
       uint8_t Stx;                                                            // First Char $ = 0x24
       unsigned char Keyword[7u];                                               // keyword  STREAM
       uint8_t Comma;                                                          //   , = 0x2C
       int8_t Port;                                                            // Port
       uint8_t Comma2;                                                         //   , = 0x2C
       unsigned char Cast[];                                                   // UNICAST or MULTICAST
       uint8_t Comma3;                                                          //   , = 0x2C
       uint8_t enable;                                                          // 1=enable 0-disable
       uint8_t Etx;                                                             // CR = 0x0D
}) XS_stream_t;
#endif

// configure the Network  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
       uint8_t Stx;                                                            // First Char $ = 0x24
       unsigned char Keyword[8u];                                               // keyword  NETWORK
       uint8_t Comma;                                                          //   , = 0x2C
       int8_t Adapter;                                                         // Adapter should be 0
       uint8_t Comma2;                                                         //   , = 0x2C
       unsigned char Mode[];                                                   // STATIC or DHCP
       uint8_t Comma3;                                                          //   , = 0x2C
       unsigned char IPAddr[];                                                  // IP Address for the XStream encoder
       uint8_t Comma4;                                                          //   , = 0x2C
       unsigned char SubNet[];                                                  // Subnet
       uint8_t Comma5;                                                          //   , = 0x2C
       unsigned char Gateway[];                                                 // Gateway IP Address
       uint8_t Etx;
} XS_network_t;
#else
CAMPACKED(
typedef struct {
       uint8_t Stx;                                                            // First Char $ = 0x24
       unsigned char Keyword[8u];                                               // keyword  NETWORK
       uint8_t Comma;                                                          //   , = 0x2C
       int8_t Adapter;                                                         // Adapter should be 0
       uint8_t Comma2;                                                         //   , = 0x2C
       unsigned char Mode[];                                                   // STATIC or DHCP
       uint8_t Comma3;                                                          //   , = 0x2C
       unsigned char IPAddr[];                                                  // IP Address for the XStream encoder
       uint8_t Comma4;                                                          //   , = 0x2C
       unsigned char SubNet[];                                                  // Subnet
       uint8_t Comma5;                                                          //   , = 0x2C
       unsigned char Gateway[];                                                 // Gateway IP Address
       uint8_t Etx;                                                             // CR = 0x0D
}) XS_network_t;
#endif

// authenticate (log the usaer into the encoder, must be done first) XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
       uint8_t Stx;                                                            // First Char $ = 0x24
       unsigned char Keyword[13u];                                               // keyword  AUTHENTICATE
       uint8_t Comma;                                                          //   , = 0x2C
       unsigned char Username[];                                               // username
       uint8_t Comma1;                                                          //   , = 0x2C
       unsigned char Password[];                                                // password
       uint8_t Etx;                                                             // CR = 0x0D
} XS_login_t;
#else
CAMPACKED(
typedef struct {
       uint8_t Stx;                                                            // First Char $ = 0x24
       unsigned char Keyword[13u];                                               // keyword  AUTHENTICATE
       uint8_t Comma;                                                          //   , = 0x2C
       unsigned char Username[];                                               // username
       uint8_t Comma1;                                                          //   , = 0x2C
       unsigned char Password[];                                                // password
       uint8_t Etx;                                                             // CR = 0x0D
}) XS_login_t;
#endif

// make changes permanent after reboot COMMIT  XS only
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                              // keyword  COMMIT
        uint8_t Etx;                                                            // CR = 0x0D
} XS_commit_t;
#else
CAMPACKED(
typedef struct {
        uint8_t Stx;                                                            // First Char $ = 0x24
        unsigned char Keyword[11u];                                              // keyword  COMMIT
        uint8_t Etx;                                                            // CR = 0x0D
}) XS_commit_t;
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint32_t  current;                                                      // offset 0 current element which is sending

        uint64_t  contrast    : 3u;                                              // (contrast msg state)
        uint64_t  hue         : 3u;                                              // (hue msg state)
        uint64_t  saturation  : 3u;                                              // (saturation msg state)
        uint64_t  clipsz      : 3u;                                              // (clip size msg state)
        uint64_t  cliplen     : 3u;                                              // (clip length msg state)
        uint64_t  bright      : 3u;                                              // (brightness)
        uint64_t  framert     : 3u;                                              // (feame rate)
        uint64_t  frameint    : 3u;                                              // (frame interval msg state)
        uint64_t  bitrate     : 3u;                                              // (bit rate msg state)
        uint64_t  sysstatus   : 3u;                                              // (system status request state)
        uint64_t  diskstatus  : 3u;                                              // (disk status request state)
        uint64_t  authorise   : 3u;                                              // authorisation was required try to login
        uint64_t  commit      : 3u;                                              // commit request made
        uint64_t  reboot      : 3u;                                              // reboot request made
        uint64_t  shutdown    : 3u;                                              // shutdown request made
        uint64_t  startrecord : 3u;                                              // start recording to disk event
        uint64_t  stoprecord  : 3u;                                              // stop recording to disk event
        uint64_t  markrecord  : 3u;                                              // mark the disk recording for the chanel with the description
        uint64_t  settime     : 3u;                                              // set-up the time on the encoder
        uint64_t  recycle     : 3u;                                              // configure the file recycle threshold
        uint64_t  erasefiles  : 3u;                                              // erase the files for a specified channel
        uint64_t  loggedin    : 1u;                                              // flag to say we have logged in and are authorised
} XS_change_queue_t;                                                            // Struct to hold the request queue and the transmission status
#else
CAMPACKED(
typedef struct  {
        uint32_t  current;                                                      // offset 0 current element which is sending

        uint64_t  contrast    : 3u;                                              // (contrast msg state)
        uint64_t  hue         : 3u;                                              // (hue msg state)
        uint64_t  saturation  : 3u;                                              // (saturation msg state)
        uint64_t  clipsz      : 3u;                                              // (clip size msg state)
        uint64_t  cliplen     : 3u;                                              // (clip length msg state)
        uint64_t  bright      : 3u;                                              // (brightness)
        uint64_t  framert     : 3u;                                              // (feame rate)
        uint64_t  frameint    : 3u;                                              // (frame interval msg state)
        uint64_t  bitrate     : 3u;                                              // (bit rate msg state)
        uint64_t  sysstatus   : 3u;                                              // (system status request state)
        uint64_t  diskstatus  : 3u;                                              // (disk status request state)
        uint64_t  authorise   : 3u;                                              // authorisation was required try to login
        uint64_t  commit      : 3u;                                              // commit request made
        uint64_t  reboot      : 3u;                                              // reboot request made
        uint64_t  shutdown    : 3u;                                              // shutdown request made
        uint64_t  startrecord : 3u;                                              // start recording to disk event
        uint64_t  stoprecord  : 3u;                                              // stop recording to disk event
        uint64_t  markrecord  : 3u;                                              // mark the disk recording for the chanel with the description
        uint64_t  settime     : 3u;                                              // set-up the time on the encoder
        uint64_t  recycle     : 3u;                                              // configure the file recycle threshold
        uint64_t  erasefiles  : 3u;                                              // erase the files for a specified channel
        uint64_t  loggedin    : 1u;                                              // flag to say we have logged in and are authorised
}) XS_change_queue_t;                                                           // Struct to hold the request queue and the transmission status
#endif

// XS Channel attribute type
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint32_t Ch1;                                                           // Channel 1 value
        uint32_t Ch2;                                                           // Channel 2 value
        uint32_t Ch3;                                                           // Channel 3 value
        uint32_t Ch4;                                                           // Channel 4 value
} XS_channel_t;
#else
CAMPACKED(
typedef struct {
        uint32_t Ch1;                                                           // Channel 1 value
        uint32_t Ch2;                                                           // Channel 2 value
        uint32_t Ch3;                                                           // Channel 3 value
        uint32_t Ch4;                                                           // Channel 4 value
}) XS_channel_t;
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t numdisks;                                                       // number of disks
        unsigned char id[10u];                                                  // id of disk
        uint32_t size;                                                          // size in megabytes
        unsigned char description[10u];                                         // REMOVABLE or SATA
} XS_edisk_t;
#else
CAMPACKED(
typedef struct {
        uint8_t numdisks;                                                       // number of disks
        unsigned char id[10u];                                                  // id of disk
        uint32_t size;                                                          // size in megabytes
        unsigned char description[10u];                                         // REMOVABLE or SATA
}) XS_edisk_t;
#endif

// =============================================================================
//
// ============ Run Cam Protocol ===============================================
//
//==============================================================================

// ======= RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO =========================== Read the basic information of the camera, such as firmware version, device type, protocol version
#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Header;                                                         // header stx char  0xCC
        uint8_t CommandID;                                                      // command
        uint8_t Crc8;                                                           // check code
} RC_req_get_info_t;                                                            // request packet structure for get device information command 0x00
#else
CAMPACKED(
typedef struct {
        uint8_t Header;                                                         // header stx char  0xCC
        uint8_t CommandID;                                                      // command
        uint8_t Crc8;                                                           // check code
}) RC_req_get_info_t;                                                           // request packet structure for get device information command 0x00
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t ProtocolVersion;                                                // protocol version
        uint16_t Feature;                                                       // feature
        uint8_t Crc8;                                                           // check code
} RC_res_get_info_t;                                                           // response packet structure for get device information command 0x00
#else
CAMPACKED(
typedef struct {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t ProtocolVersion;                                                // protocol version
        uint16_t Feature;                                                       // feature
        uint8_t Crc8;                                                           // check code
}) RC_res_get_info_t;                                                           // response packet structure for get device information command 0x00
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t Action;                                                         // action
        uint8_t Crc8;                                                           // check code
} RC_req_action_handshake_t;                                                   // request packet structure for performing camera actions 0x04 or 0x03 or 0x02 or 0x01
#else
CAMPACKED(
typedef struct {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t Action;                                                         // action
        uint8_t Crc8;                                                           // check code
}) RC_req_action_handshake_t;                                                   // request packet structure for performing camera actions 0x04 or 0x03 or 0x02 or 0x01
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t Crc8;                                                           // check code
} RC_res_keyaction_t;                                                           // response packet structure for performing camera actions 0x03 (release key) or 0x02 (press key) for remote control pad (??? also for 0x01)
#else
CAMPACKED(
typedef struct {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t Crc8;                                                           // check code
}) RC_res_keyaction_t;                                                          // response packet structure for performing camera actions 0x03 (release key) or 0x02 (press key) for remote control pad (??? also for 0x01)
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t Response;                                                       // Response byte ACTIONID << 4 + 1=ok 0=fail
        uint8_t Crc8;                                                           // check code
} RC_res_handshake_t;                                                          // response packet structure for performing camera actions 0x04 (handshake)
#else
CAMPACKED(
typedef struct {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t Response;                                                       // Response byte ACTIONID << 4 + 1=ok 0=fail
        uint8_t Crc8;                                                           // check code
}) RC_res_handshake_t;                                                          // response packet structure for performing camera actions 0x04 (handshake)
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t Setting;                                                        // setting
        uint8_t ChunkIdx;                                                       // chunk index
        uint8_t Crc8;                                                           // check code
} RC_req_getread_settings_t;                                                    // request settings for 0x10 or 0x11 commands
#else
CAMPACKED(
typedef struct {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t Setting;                                                        // setting
        uint8_t ChunkIdx;                                                       // chunk index
        uint8_t Crc8;                                                           // check code
}) RC_req_getread_settings_t;                                                   // request settings for 0x10 or 0x11 commands
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t RemainingChunk;                                                 // remaining chunk
        uint8_t DataLength;                                                     // Data length
        uint8_t SettingId1;                                                     // setting id
        unsigned char SettingName1[32u];                                         // setting name string
        unsigned char ValueSetting1[10u];                                        // value
        uint8_t SettingId2;                                                     // setting id
        unsigned char SettingName2[32u];                                         // setting name string
        unsigned char ValueSetting2[10u];                                        // value
        uint8_t Crc8;                                                           // check code
} RC_res_get_settings_t;
#else
CAMPACKED(
typedef struct {
        uint8_t RemainingChunk;                                                 // remaining chunk
        uint8_t DataLength;                                                     // Data length
        uint8_t SettingId1;                                                     // setting id
        unsigned char SettingName1[32u];                                         // setting name string
        unsigned char ValueSetting1[10u];                                        // value
        uint8_t SettingId2;                                                     // setting id
        unsigned char SettingName2[32u];                                         // setting name string
        unsigned char ValueSetting2[10u];                                        // value
        uint8_t Crc8;                                                           // check code
}) RC_res_get_settings_t;
#endif

typedef union {

  uint8_t Uint8;                                                                // 8 bit unsigned
  int8_t Int8;                                                                  // 8 bit signed
  uint16_t Uint16;                                                              // 16 bit unsigned
  int16_t Int16;                                                                // 16 bit signed
  float32_t Float32;                                                            // 32 bit IEEE float
  unsigned char TextSelection[32u];                                              // text selection string separated by ;
  uint16_t StringSz;                                                            // size of string returned
  uint64_t Settings;                                                            // Settings ?
  uint64_t Info;                                                                // Info ?
  uint64_t Command;                                                             // Reserved

} RC_setting_type_t;                                                            // Union of types that may be returned as SettingType

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t RemainingChunk;                                                 // remaining chunk
        uint8_t DataLength;                                                     // Data length
        uint8_t SettingType;                                                    // setting type
        RC_setting_type_t Value;                                                // value size is assigned as maximum size for the largest member of the union
        uint16_t MinValue;                                                      // minimum value
        uint16_t MaxValue;                                                      // maximum value
        uint16_t DecPoint;                                                      // the digtal count after decimal point
        uint16_t StepSize;                                                      // the increment/decrement value when modifying the setting
        uint8_t MaxStringSize;                                                  // max size of string
        unsigned char TextSelections[32u];                                       // a null-terminated string?the content is all available string in the setting, they are separated by a semicolon(;)
        uint8_t Crc8;                                                           // check code
} RC_res_read_settings_t;                                                       // response packet structure reading back settings details
#else
CAMPACKED(
typedef struct {
        uint8_t RemainingChunk;                                                 // remaining chunk
        uint8_t DataLength;                                                     // Data length
        uint8_t SettingType;                                                    // setting type
        RC_setting_type_t Value;                                                // value size is assigned as maximum size for the largest member of the union
        uint16_t MinValue;                                                      // minimum value
        uint16_t MaxValue;                                                      // maximum value
        uint16_t DecPoint;                                                      // the digtal count after decimal point
        uint16_t StepSize;                                                      // the increment/decrement value when modifying the setting
        uint8_t MaxStringSize;                                                  // max size of string
        unsigned char TextSelections[32u];                                       // a null-terminated string?the content is all available string in the setting, they are separated by a semicolon(;)
        uint8_t Crc8;                                                           // check code
}) RC_res_read_settings_t;                                                      // response packet structure reading back settings details
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t Setting;                                                        // setting
        RC_setting_type_t Value;                                                // value
        uint8_t Crc8;                                                           // check code
} RC_req_write_settings_t;                                                      // write settings for 0x13 command
#else
CAMPACKED(
typedef struct {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t Setting;                                                        // setting
        RC_setting_type_t Value;                                                // value
        uint8_t Crc8;                                                           // check code
}) RC_req_write_settings_t;                                                     // write settings for 0x13 command
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t X;                                                              // x-axis
        uint8_t Y;                                                              // y-axis
        uint8_t Width;                                                          // width
        uint8_t Height;                                                         // height
        uint32_t SpecialChar;                                                   // the special char (bmp ??)
        uint8_t Crc8;                                                           // check code
} RC_req_fill_screen_t;                                                         // request for command 0x20
#else
CAMPACKED(
typedef struct {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t X;                                                              // x-axis
        uint8_t Y;                                                              // y-axis
        uint8_t Width;                                                          // width
        uint8_t Height;                                                         // height
        uint32_t SpecialChar;                                                   // the special char (bmp ??)
        uint8_t Crc8;                                                           // check code
}) RC_req_fill_screen_t;                                                        // request for command 0x20
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Header;                                                         // header stx char  0xCC
        uint8_t CommandID;                                                      // command
        uint8_t X;                                                              // x-axis
        uint8_t Y;                                                              // y-axis
        uint32_t SpecialChar;                                                   // the special char (bmp ??)
        uint8_t Crc8;                                                           // check code
} RC_req_write_char_t;                                                          // request for command 0x21
#else
CAMPACKED(
typedef struct {
        uint8_t Header;                                                         // header stx char  0xCC
        uint8_t CommandID;                                                      // command
        uint8_t X;                                                              // x-axis
        uint8_t Y;                                                              // y-axis
        uint32_t SpecialChar;                                                   // the special char (bmp ??)
        uint8_t Crc8;                                                           // check code
}) RC_req_write_char_t;                                                         // request for command 0x21
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t MsgLength;                                                      // strlen of message string to write
        uint8_t X;                                                              // x-axis
        uint8_t Y;                                                              // y-axis
        unsigned char MsgToWrite[60u];                                          // message to write
        uint8_t Crc8;                                                           // check code
} RC_req_write_str_t;                                                           // request for command 0x22 (horizontal) or 0x23 (vertical)
#else
CAMPACKED(
typedef struct {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t MsgLength;                                                      // strlen of message string to write
        uint8_t X;                                                              // x-axis
        uint8_t Y;                                                              // y-axis
        unsigned char MsgToWrite[60u];                                          // message to write
        uint8_t Crc8;                                                           // check code
}) RC_req_write_str_t;                                                          // request for command 0x22 (horizontal) or 0x23 (vertical)
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t MsgLength;                                                      // strlen of message string to write
        unsigned char MsgToWrite[60u];                                          // message to write
        uint8_t Crc8;                                                           // check code
} RC_req_write_chars_t;                                                         // request for command 0x24
#else
CAMPACKED(
typedef struct {
        uint8_t Header;                                                         // header stx char 0xCC
        uint8_t CommandID;                                                      // command
        uint8_t MsgLength;                                                      // strlen of message string to write
        unsigned char MsgToWrite[60u];                                          // message to write
        uint8_t Crc8;                                                           // check code
}) RC_req_write_chars_t;                                                        // request for command 0x24
#endif

#if defined(D_FT900)
#ifdef RC_MODE_TYPE                                                             // This was a structure allowing each osr mode as a selected option
typedef struct CAMPACKED {
        uint32_t  current;                                                      // offset 0 current element which is sending

        uint64_t wifi  : 3u;                                                     // Simulation Click the Wi-Fi button
        uint64_t power : 3u;                                                     // Simulation Click the Power button
        uint64_t mode  : 3u;                                                     // Switch the camera mode video,photo,osd modes available
        uint64_t record : 3u;                                                    // Start recording
        uint64_t stop : 3u;                                                      // Stop recording
        uint64_t osd_wdr : 3u;                                                   // wdr via osd
        uint64_t osd_lens : 3u;                                                  // lens via osd
        uint64_t osd_agc : 3u;                                                   // agc via osd
        uint64_t osd_bright : 3u;                                                // Brightness change via osd mode (go to osd and then use up down and confirm to reach)
        uint64_t osd_cont : 3u;                                                  // Contrast change via osd mode
        uint64_t osd_sharp : 3u;                                                 // Sharpness change via osd mode
        uint64_t osd_color : 3u;                                                 // Color gain via osd mode
        uint64_t osd_nr : 3u;                                                    // NR via osd mode
        uint64_t osd_sync : 3u;                                                  // sync via osd eng mode
        uint64_t osd_burst : 3u;                                                 // burst via osd eng mode
        uint64_t osd_ped : 3u;                                                   // pedestal via osd eng mode
        uint64_t osd_white : 3u;                                                 // white via osd eng mode
        uint64_t osd_engcolor : 3u;                                              // osd color via osd eng menu
        uint64_t osd_bg : 3u;                                                    // background via osd eng mode
        uint64_t osd_eng_ret : 3u;                                               // eng mode or return from eng mode
        uint64_t handshake : 3u;                                                 // handshake
        uint64_t sparebit1 : 1u;
        uint32_t ds_charset : 3u;                                                // r/w charset via ds command 0x10U
        uint32_t ds_column : 3u;                                                 // r only number of columns displayed
        uint32_t ds_ntsc_pal : 3u;                                               // r/w ntsc or pal
        uint32_t ds_memory : 3u;                                                 // r read camera memory card
        uint32_t ds_rectime : 3u;                                                // r read camera recording time left
        uint32_t ds_resol : 3u;                                                  // r/w camera resolution
        uint32_t ds_time : 3u;                                                   // r/w rtc of camera
        uint32_t write_txt : 3u;                                                 // write a text on the screen at a position
        uint32_t sparebits2 : 8u;

} RC_change_queue_t;                                                           // Struct to hold the request queue and the transmission status
#else                                                                           // with this structure the user users the up/down right/left and confirm to select the options like brightness to adjust
typedef struct CAMPACKED {
        uint32_t  current;                                                      // offset 0 current element which is sending

        uint64_t wifi  : 3u;                                                     // Simulation Click the Wi-Fi button
        uint64_t power : 3u;                                                     // Simulation Click the Power button
        uint64_t mode  : 3u;                                                     // Switch the camera mode video,photo,osd modes available
        uint64_t record : 3u;                                                    // Start recording
        uint64_t stop : 3u;                                                      // Stop recording
        uint64_t up : 3u;                                                        // up press      (
        uint64_t down : 3u;                                                      // down press
        uint64_t left : 3u;                                                      // left press
        uint64_t right : 3u;                                                     // right press
        uint64_t confirm : 3u;                                                   // confirm press
        uint64_t ds_charset : 3u;                                                // r/w charset via ds command 0x10U
        uint64_t ds_column : 3u;                                                 // r only number of columns displayed
        uint64_t ds_ntsc_pal : 3u;                                               // r/w ntsc or pal
        uint64_t ds_memory : 3u;                                                 // r read camera memory card
        uint64_t ds_rectime : 3u;                                                // r read camera recording time left
        uint64_t ds_resol : 3u;                                                  // r/w camera resolution
        uint64_t ds_time : 3u;                                                   // r/w rtc of camera
        uint64_t write_txt : 3u;                                                 // write a text on the screen at a position
        uint64_t getinfo : 3u;                                                   // get the current camera status
        uint64_t display_bmp : 3u;                                               // display bitmap at position
        uint64_t sparebits1 : 4u;

} RC_change_queue_t;                                                            // Struct to hold the request queue and the transmission status
#endif                                                                          // end of method define RC_MODE_TYPE
#else
#ifdef RC_MODE_TYPE                                                             // This was a structure allowing each osr mode as a selected option
CAMPACKED(
typedef struct  {
        uint32_t  current;                                                      // offset 0 current element which is sending

        uint64_t wifi  : 3u;                                                     // Simulation Click the Wi-Fi button
        uint64_t power : 3u;                                                     // Simulation Click the Power button
        uint64_t mode  : 3u;                                                     // Switch the camera mode video,photo,osd modes available
        uint64_t record : 3u;                                                    // Start recording
        uint64_t stop : 3u;                                                      // Stop recording
        uint64_t osd_wdr : 3u;                                                   // wdr via osd
        uint64_t osd_lens : 3u;                                                  // lens via osd
        uint64_t osd_agc : 3u;                                                   // agc via osd
        uint64_t osd_bright : 3u;                                                // Brightness change via osd mode (go to osd and then use up down and confirm to reach)
        uint64_t osd_cont : 3u;                                                  // Contrast change via osd mode
        uint64_t osd_sharp : 3u;                                                 // Sharpness change via osd mode
        uint64_t osd_color : 3u;                                                 // Color gain via osd mode
        uint64_t osd_nr : 3u;                                                    // NR via osd mode
        uint64_t osd_sync : 3u;                                                  // sync via osd eng mode
        uint64_t osd_burst : 3u;                                                 // burst via osd eng mode
        uint64_t osd_ped : 3u;                                                   // pedestal via osd eng mode
        uint64_t osd_white : 3u;                                                 // white via osd eng mode
        uint64_t osd_engcolor : 3u;                                              // osd color via osd eng menu
        uint64_t osd_bg : 3u;                                                    // background via osd eng mode
        uint64_t osd_eng_ret : 3u;                                               // eng mode or return from eng mode
        uint64_t handshake : 3u;                                                 // handshake
        uint64_t sparebit1 : 1u;
        uint32_t ds_charset : 3u;                                                // r/w charset via ds command 0x10U
        uint32_t ds_column : 3u;                                                 // r only number of columns displayed
        uint32_t ds_ntsc_pal : 3u;                                               // r/w ntsc or pal
        uint32_t ds_memory : 3u;                                                 // r read camera memory card
        uint32_t ds_rectime : 3u;                                                // r read camera recording time left
        uint32_t ds_resol : 3u;                                                  // r/w camera resolution
        uint32_t ds_time : 3u;                                                   // r/w rtc of camera
        uint32_t write_txt : 3u;                                                 // write a text on the screen at a position
        uint32_t sparebits2 : 8u;

}) RC_change_queue_t;                                                           // Struct to hold the request queue and the transmission status
#else                                                                           // with this structure the user users the up/down right/left and confirm to select the options like brightness to adjust
CAMPACKED(
typedef struct  {
        uint32_t  current;                                                      // offset 0 current element which is sending

        uint64_t wifi  : 3u;                                                     // Simulation Click the Wi-Fi button
        uint64_t power : 3u;                                                     // Simulation Click the Power button
        uint64_t mode  : 3u;                                                     // Switch the camera mode video,photo,osd modes available
        uint64_t record : 3u;                                                    // Start recording
        uint64_t stop : 3u;                                                      // Stop recording
        uint64_t up : 3u;                                                        // up press      (
        uint64_t down : 3u;                                                      // down press
        uint64_t left : 3u;                                                      // left press
        uint64_t right : 3u;                                                     // right press
        uint64_t confirm : 3u;                                                   // confirm press
        uint64_t ds_charset : 3u;                                                // r/w charset via ds command 0x10U
        uint64_t ds_column : 3u;                                                 // r only number of columns displayed
        uint64_t ds_ntsc_pal : 3u;                                               // r/w ntsc or pal
        uint64_t ds_memory : 3u;                                                 // r read camera memory card
        uint64_t ds_rectime : 3u;                                                // r read camera recording time left
        uint64_t ds_resol : 3u;                                                  // r/w camera resolution
        uint64_t ds_time : 3u;                                                   // r/w rtc of camera
        uint64_t write_txt : 3u;                                                 // write a text on the screen at a position
        uint64_t getinfo : 3u;                                                   // get the current camera status
        uint64_t display_bmp : 3u;                                               // display bitmap at position
        uint64_t sparebits1 : 4u;

}) RC_change_queue_t;                                                           // Struct to hold the request queue and the transmission status
#endif                                                                          // end of method define RC_MODE_TYPE
#endif

#if defined(D_FT900)
typedef struct CAMPACKED {
  uint64_t getToken : 1u;
  uint64_t batteryLeft : 1u;
  uint64_t startEvent : 1u;
  uint64_t tkPhoto : 1u;
  uint64_t setPIV : 1u;
  uint64_t getRecTime : 1u;
  uint64_t startRecord : 1u;
  uint64_t stopRecord : 1u;
  uint64_t ccsStop : 1u;
  uint64_t quickRecord : 1u;
  uint64_t quickRecPause : 1u;
  uint64_t quickRecResume : 1u;
  uint64_t resetWifi : 1u;
  uint64_t videoStd :1u;
  uint64_t videoLoopRes : 1u;
  uint64_t videoPhoRes : 1u;
  uint64_t videoLapseRes : 1u;
  uint64_t videoRes : 1u;
  uint64_t photoSize : 1u;
  uint64_t idleMode : 1u;
  uint64_t wakeUpMode: 1u;
  uint64_t logStart : 1u;
  uint64_t resetStream : 1u;
  uint64_t stopStream : 1u;
  uint64_t bindBlue : 1u;
  uint64_t unBindBlue : 1u;
  uint64_t lapseVidTime : 1u;
  uint64_t lapseVidOpt : 1u;
  uint64_t slowMotionRt : 1u;
  uint64_t burstCap : 1u;
  uint64_t preciseContTim : 1u;
  uint64_t recPhoTime : 1u;
  uint64_t loopRecDur : 1u;
  uint64_t videoQ : 1u;
  uint64_t photoQ : 1u;
  uint64_t vidStampVal : 1u;
  uint64_t phoStampVal : 1u;
  uint64_t onOffOpt : 1u;
  uint64_t vidOutDev : 1u;
  uint64_t meterOpt : 1u;
  uint64_t ledMode : 1u;
  uint64_t buzzerOpt : 1u;
  uint64_t captureMode : 1u;
  uint64_t fastZoom : 1u;
  uint64_t bitRate : 1u;
  uint64_t delFile : 1u;
  uint64_t format : 1u;
  uint64_t dir : 1u;
  uint64_t sndGetFile : 1u;
  uint64_t getMedia : 1u;
  uint64_t thumbNail : 1u;
  uint64_t enabScript : 1u;
  uint64_t getLastPho : 1u;
  uint64_t getFreeSD : 1u;
  uint64_t getTotalSD : 1u;
  uint64_t case54 : 1u;
  uint64_t case55 : 1u;
  uint64_t case56 : 1u;
  uint64_t case57 : 1u;
  uint64_t case58 : 1u;
  uint64_t case59 : 1u;
  uint64_t case60 : 1u;
  uint64_t case61 : 1u;
  uint64_t case62 : 1u;
  uint64_t case63 : 1u;
} COMGS_YiOptActive_t;                                                          // Structure to hold a bit if set then the operation needs to be sent
#else
CAMPACKED(
typedef struct  {                                                               // Set a bit on the GUI to trigger a send event to the web cam
  uint64_t getToken : 1u;
  uint64_t batteryLeft : 1u;
  uint64_t startEvent : 1u;
  uint64_t tkPhoto : 1u;
  uint64_t setPIV : 1u;
  uint64_t getRecTime : 1u;
  uint64_t startRecord : 1u;
  uint64_t stopRecord : 1u;
  uint64_t ccsStop : 1u;
  uint64_t quickRecord : 1u;
  uint64_t quickRecPause : 1u;
  uint64_t quickRecResume : 1u;
  uint64_t resetWifi : 1u;
  uint64_t videoStd :1u;
  uint64_t videoLoopRes : 1u;
  uint64_t videoPhoRes : 1u;
  uint64_t videoLapseRes : 1u;
  uint64_t videoRes : 1u;
  uint64_t photoSize : 1u;
  uint64_t idleMode : 1u;
  uint64_t wakeUpMode: 1u;
  uint64_t logStart : 1u;
  uint64_t resetStream : 1u;
  uint64_t stopStream : 1u;
  uint64_t bindBlue : 1u;
  uint64_t unBindBlue : 1u;
  uint64_t lapseVidTime : 1u;
  uint64_t lapseVidOpt : 1u;
  uint64_t slowMotionRt : 1u;
  uint64_t burstCap : 1u;
  uint64_t preciseContTim : 1u;
  uint64_t recPhoTime : 1u;
  uint64_t loopRecDur : 1u;
  uint64_t videoQ : 1u;
  uint64_t photoQ : 1u;
  uint64_t vidStampVal : 1u;
  uint64_t phoStampVal : 1u;
  uint64_t onOffOpt : 1u;
  uint64_t vidOutDev : 1u;
  uint64_t meterOpt : 1u;
  uint64_t ledMode : 1u;
  uint64_t buzzerOpt : 1u;
  uint64_t captureMode : 1u;
  uint64_t fastZoom : 1u;
  uint64_t bitRate : 1u;
  uint64_t delFile : 1u;
  uint64_t format : 1u;
  uint64_t dir : 1u;
  uint64_t sndGetFile : 1u;
  uint64_t getMedia : 1u;
  uint64_t thumbNail : 1u;
  uint64_t enabScript : 1u;
  uint64_t getLastPho : 1u;
  uint64_t getFreeSD : 1u;
  uint64_t getTotalSD : 1u;
  uint64_t case54 : 1u;
  uint64_t case55 : 1u;
  uint64_t case56 : 1u;
  uint64_t case57 : 1u;
  uint64_t case58 : 1u;
  uint64_t case59 : 1u;
  uint64_t case60 : 1u;
  uint64_t case61 : 1u;
  uint64_t case62 : 1u;
  uint64_t case63 : 1u;
}) COMGS_YiOptActive_t;                                                         // Structure to hold a bit if set then the operation needs to be sent
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif