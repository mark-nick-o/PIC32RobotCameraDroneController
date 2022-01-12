#ifndef __microscan_lib
#define __microscan_lib

// Microscan Systems Inc Vision Scape (Hawk and Mini) Camera ANPR OCR Camera
// Supports GS1 app Ids
// Version : @(#) 1.0
// Copyright (C) 2020 (A C P) Walkerburn Scotland
//

#include "definitions.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define MICAMPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define MICAMPACKED __attribute__((packed))                                 /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define MICAMPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define MICAMPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define MICAMPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define MIRSC_LIB extern

// telnet ports are 49211[1-9]
#define MIRSC_TELNET_PORT1 49211U
#define MIRSC_TELNET_PORT2 49212U
#define MIRSC_TELNET_PORT3 49213U
#define MIRSC_TELNET_PORT4 49214U
#define MIRSC_TELNET_PORT5 49215U
#define MIRSC_TELNET_PORT6 49216U

#define MiScanUART_LR 4u                                                        /* chosen UART for microscan camera connection left right */
#define MiScanUART_FB 5u                                                        /* chosen UART for microscan camera connection front back */
#define NUM_OF_PLANT_SECTION 10u                                                /* number of planting positions (trees) */
#define TreeBeaconSlot1 1u                                                      /* memory slot in camera1 which contains the tree palnting beacon fonts and configuration */
#define TreeBeaconSlot2 2u                                                      /* memory slot in camera2 which contains the tree palnting beacon fonts and configuration */

// define the strings from the tcp/ip stream  (configured in the camera job )
#define MIRSC_OCR1_STAT_STR "OCR1 STATUS:"
#define MIRSC_OCR2_STAT_STR "OCR2 STATUS:"
#define MIRSC_OCR_DATA1_STR "I DATA:"
#define MIRSC_OCR_DATA2_STR "NUM PLATE:"
#define MIRSC_DM_STAT_STR "DM STATUS:"
#define MIRSC_OCR_LR_STR "LR BEACON:"
#define MIRSC_OCR_FB_STR "FB BEACON:"
#define MIRSC_CAM1_STAT "CAM1 STAT:"
#define MIRSC_CAM2_STAT "CAM2 STAT:"

#define MIRSC_GS1_GTIN "GTIN:"
#define MIRSC_GS1_BATCH "BATCH:"
#define MIRSC_GS1_EXP_DATE "DATE:"
#define MIRSC_CENTER_POINT_LOC "CENTER POINT LOCATION:"

// serial device commands
#define MIRSC_TRIGGER "\\a\\b\\0x04C"                                           // as you define in youre trigger set-up
#define MIRSC_MAT_EXP_DAT(A,month,dat,yr) { sprintf(A,"EXP (%s) %d %d \n\r",month,dat,yr); } // e.g. EXP (FEB|APR) 12-23 1999
#define MIRSC_SET_MATCH1 "SET matchstring1 Y821PSL\xSA51XYJ\n\r"                // set matchstring1 with 2 example strings
#define MIRSC_SET_FLOAT(A,tag,val) { sprintf(A,"SET %s %f\n\r",tag,val); }      // e.g MIRSC_SET_FLOAT(Buf,"float25",64.6);
#define MIRSC_SET_INT(A,tag,val) { sprintf(A,"SET %s %d\n\r",tag,val); }        // e.g MIRSC_SET_INT(Buf,"int19",70);
#define MIRSC_SET_STR(A,tag,val) { sprintf(A,"SET %s %s\n\r",tag,val); }        // e.g MIRSC_SET_STR(Buf,"string8","matchstring");
#define MIRSC_SET_BOOL(A,tag,val) { if (val) { sprintf(A,"SET %s 1\n\r",tag); } else { sprintf(A,"SET %s 0\n\r",tag); } }   // e.g MIRSC_SET_BOOL(Buf,"bool65",1);
#define MIRSC_GET_MATCH1 "GET matchstring1\n\r"                                 // get the value of match string 1
#define MIRSC_GET(A,tag) { sprintf(A,"GET %s\n\r",tag); }                       // get the current value as a tag
#define MIRSC_GET_LEN 6U
#define MIRSC_SET_OK "!OK"                                                      // set will contain as first string
#define MIRSC_SET_FAIL "!ERROR"                                                 // set will contain as first string
#define MIRSC_GET_FAIL "!ERROR"                                                 // get will contain as first string
#define MIRSC_AUTOCAL "AUTOCAL"
#define MIRSC_VER "VERSION"
#define MIRSC_START_INS "ONLINE"                                                // start inspecting
#define MIRSC_STOP_INS "OFFLINE"                                                // stop inspecting
#define MIRSC_START_TRIG "TRIGGER"                                              // send trigger
#define MIRSC_VIRT_CH(A,chan) { sprintf(A,"vt %d",chan); }                      // Triggers an inspection by pulsing a Virtual I/O point.e.g vt 1 will return pulse VIO1. The inspection will run if it is configured to use VIO 1 as a trigger
#define MIRSC_GET_GAIN_EXP_FOCUS "QUERYAUTOCAL"                                 // get gain exposure and focus
#define MIRSC_REBOOT_NO_JOB "REBOOT -noload"                                    // reboot do not load job
#define MIRSC_DEL_ALL "JOBDELETE -all"                                          // delete all jobs
#define MIRSC_DOWNLOAD "JOBDOWNLOAD <-transfer=ymodem>"                         // down load avg job externally
#define MIRSC_MEM_PRO "MEMAVAIL -p"                                             // cpu memory
#define MIRSC_MEM_CO "MEMAVAIL -c"                                              // co-processor memory
#define MIRSC_GET_JPG "GETIMAGE <-transfer=ymodem>"                             // get jpg image file
#define MIRSC_GET_PNG "GETIMAGE <-transfer=ymodem> -format=png"                 // get png image file
#define MIRSC_REBBOT_JOB(A,slot) { sprintf(A,"JOBBOOT -slot=%d",slot); }        // boot with job in slot
#define MIRSC_LOAD_JOB(A,slot) { sprintf(A,"JOBLOAD -slot=%d",slot); }          // load job in slot
#define MIRSC_DEL_JOB(A,slot) { sprintf(A,"JOBDELETE -slot=%d",slot); }         // delete job in slot
#define MIRSC_SAVE_JOB(A,slot) { sprintf(A,"JOBSAVE -slot=%d",slot); }          // save current job to slot
#define MIRSC_LOAD_START_JOB(A,slot) { sprintf(A,"JOBLOAD -slot=%d -r",slot); } // load and start job from slot

// for configuration the mapping you do in the camera a typical mapping for the hawk is shown below
//
// ======================== std ocr engine ========================================================
#define MIRSC_CONF_STD_OCR "float25"                                            // confidence def 0.5
#define MIRSC_ENAB_MATCH_STD_OCR "bool65"                                       // enable match string
#define MIRSC_MATCH_STR_STD_OCR "string8"                                       // match string
#define MIRSC_CONN_STRENG_STD_OCR "int11"                                       // connection strength def 0 (-3 to 3) int8_t
// ======================= intellect ocr ==========================================================
#define MIRSC_SCALE_FACT_INTEL "float25"                                        // scaling factor def 1.0 (0.5 - 3)
#define MIRSC_MATCH_STR_INTEL "string8"                                         // match string
#define MIRSC_USE_DEF_FONT_INTEL "bool65"                                       // use default font
#define MIRSC_CHAR_SLANT_DET_INTEL "bool66"                                     // character slant detection
#define MIRSC_DISCD_BOUND_CHAR_INTEL "bool67"                                   // discard boundary characters
#define MIRSC_UNKNOW_CHAR_INTEL "string9"                                       // unknown character for display of unidentified characture found
#define MIRSC_MAX_CHAR_PIX_INTEL "int11"                                        // maximum character pixels def 20
#define MIRSC_MAX_HT_INTEL "int12"                                              // maximum height (0-200)
#define MIRSC_MIN_HT_INTEL "int13"                                              // minimum height (0-200)
#define MIRSC_MAX_WDTH_INTEL "int14"                                            // maximum width (0-200)
#define MIRSC_MIN_WDTH_INTEL "int15"                                            // minimum width (0-200)
#define MIRSC_CONFIDNCE_INTEL "float26"                                         // confidence def 0.5 (0-1.0)
// ======================= decode menu ==========================================================
#define MIRSC_GS1_VALIDATION "bool01"                                           // GS1 validation on/off
#define MIRSC_DECODE_MATCH "string10"                                           // decode match string
#define MIRSC_C128_SYM_LEN "int16"                                              // code 128 symbol length
// ====================== count objects =====================================================
#define MIRSC_BLOB_POLARITY "int17"                                             // blob polarity
#define MIRSC_IG_BLOB_ROI "bool68"                                              // ignore blobs that touch roi
#define MIRSC_MIN_BLOB_SI "int18"                                               // min blob sise def 40 (0-50000)
#define MIRSC_MAX_BLOB_SI "int19"                                               // max blob sise def 10000000 (10000000-2147483647)
#define MIRSC_CO_TOLERANCE "int20"                                              // tolerance 1-4
// ==================== presence absence object =============================================
#define MIRSC_PAO_MIN_PIX_TOL "int21"                                           // tolerance min pixel count def 40
#define MIRSC_PAO_MAX_PIX_TOL "int22"                                           // tolerance max pixel count def 10478576
// ==================== measure distance object =============================================
#define MIRSC_MD_MIN_TOL "float27"                                              // tolerance min 20
#define MIRSC_MD_MAX_TOL "float28"                                              // tolerance max 80
// ==================== verification settings ===============================================
#define MIRSC_APERTURE "int23"                                                  // aperture def 50%
#define MIRSC_ANGLE "string11"                                                  // angle def 90
#define MIRSC_WAVE_LEN "string12"                                               // wavelength 640

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t status : 1u;                                                    // status boolean state
        uint8_t spare : 7u;                                                     // 7 spare booelan states
        float64_t location;                                                     // location
        float64_t x;                                                            // x axis
        float64_t y;                                                            // y axis
        float64_t angle;                                                        // angle
        float64_t fit_quality;                                                  // fit quality
} MIRSC_locate_t;                                                               // locate tool message contiainer structure
#else
MICAMPACKED(
typedef struct {
        uint8_t status : 1u;                                                    // status boolean state
        uint8_t spare : 7u;                                                     // 7 spare booelan states
        float64_t location;                                                     // location
        float64_t x;                                                            // x axis
        float64_t y;                                                            // y axis
        float64_t angle;                                                        // angle
        float64_t fit_quality;                                                  // fit quality
}) MIRSC_locate_t;                                                              // locate tool message contiainer structure
#endif

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        unsigned int dec_txt[32u];                                              // decode text
        unsigned int gs1_txt[32u];                                              // gs1 text
        unsigned int sym_txt[32u];                                              // symbology text
        uint32_t cp_loc;                                                        // centre point location
} MIRSC_decode_t;                                                               // decode tool message contiainer structure
#else
MICAMPACKED(
typedef struct {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        unsigned int dec_txt[32u];                                              // decode text
        unsigned int gs1_txt[32u];                                              // gs1 text
        unsigned int sym_txt[32u];                                              // symbology text
        uint32_t cp_loc;                                                        // centre point location
}) MIRSC_decode_t;                                                              // decode tool message contiainer structure
#endif

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t status : 1u;                                                    // status
        uint8_t match : 1u;                                                     // match ok or not
        uint8_t spare : 6u;
        unsigned int ocr_txt[32u];                                              // ocr text
        int32_t num_char;                                                       // number of chars found
        float64_t min_char_conf;                                                // minimum characture confidence
        float64_t max_char_conf;                                                // maximum characture confidence
        float64_t mean_char_conf;                                               // mean characture confidence
} MIRSC_ocr_t;                                                                 // ocr tool message contiainer structure
#else
MICAMPACKED(
typedef struct {
        uint8_t status : 1u;                                                    // status
        uint8_t match : 1u;                                                     // match ok or not
        uint8_t spare : 6u;
        unsigned int ocr_txt[32u];                                              // ocr text
        int32_t num_char;                                                       // number of chars found
        float64_t min_char_conf;                                                // minimum characture confidence
        float64_t max_char_conf;                                                // maximum characture confidence
        float64_t mean_char_conf;                                               // mean characture confidence
}) MIRSC_ocr_t;                                                                 // ocr tool message contiainer structure
#endif

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        int32_t num_parts;                                                      // number of parts found
        float64_t x;                                                            // x axis
        float64_t y;                                                            // y axis
        float64_t angle;                                                        // angle
        float64_t fit_quality;                                                  // fit quality
} MIRSC_count_t;                                                                // count tool message contiainer structure
#else
MICAMPACKED(
typedef struct {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        int32_t num_parts;                                                      // number of parts found
        float64_t x;                                                            // x axis
        float64_t y;                                                            // y axis
        float64_t angle;                                                        // angle
        float64_t fit_quality;                                                  // fit quality
}) MIRSC_count_t;                                                               // count tool message contiainer structure
#endif

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        int32_t picsel_count;                                                   // picsel count
} MIRSC_pres_abs_t;                                                             // prescence / absence tool message contiainer structure
#else
MICAMPACKED(
typedef struct {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        int32_t picsel_count;                                                   // picsel count
}) MIRSC_pres_abs_t;                                                            // prescence / absence tool message contiainer structure
#endif

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        unsigned char match[64u];                                               // match string
} MIRSC_match_t;                                                               // match tool message contiainer structure
#else
MICAMPACKED(
typedef struct {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        unsigned char match[64u];                                               // match string
}) MIRSC_match_t;                                                               // match tool message contiainer structure
#endif

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        unsigned char output[64u];                                              // match string
        unsigned char str1format[64u];                                          // string 1 formatted
        unsigned char str2format[64u];                                          // string 2 formatted
} MIRSC_str_format_t;                                                           // match tool message contiainer structure 
#else
MICAMPACKED(
typedef struct {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        unsigned char output[64u];                                              // match string
        unsigned char str1format[64u];                                          // string 1 formatted
        unsigned char str2format[64u];                                          // string 2 formatted
}) MIRSC_str_format_t;                                                          // match tool message contiainer structure                                                               // match tool message contiainer structure
#endif

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        int32_t num_parts;                                                      // number of parts found
        float64_t width_meas;                                                   // width measurement
        float64_t height_meas;                                                  // height measurement
        float64_t edge_pt1;                                                     // edge point one
        float64_t width_ep1;                                                    // width measurement edge point 1
        float64_t height_ep1;                                                   // height measurement edge point 1
        float64_t edge_ln1;                                                     // edge line one
        float64_t width_el1;                                                    // width measurement edge line 1
        float64_t height_el1;                                                   // height measurement edge line 1
        float64_t angle;                                                        // angle
} MIRSC_meas_tool_t;                                                            // measure tool message contiainer structure 
#else
MICAMPACKED(
typedef struct {
        uint8_t status : 1u;                                                    // status
        uint8_t spare : 7u;
        int32_t num_parts;                                                      // number of parts found
        float64_t width_meas;                                                   // width measurement
        float64_t height_meas;                                                  // height measurement
        float64_t edge_pt1;                                                     // edge point one
        float64_t width_ep1;                                                    // width measurement edge point 1
        float64_t height_ep1;                                                   // height measurement edge point 1
        float64_t edge_ln1;                                                     // edge line one
        float64_t width_el1;                                                    // width measurement edge line 1
        float64_t height_el1;                                                   // height measurement edge line 1
        float64_t angle;                                                        // angle
}) MIRSC_meas_tool_t;                                                           // measure tool message contiainer structure                                                              // match tool message contiainer structure
#endif

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t status : 1u;                                                    // status
        uint8_t GoodStatus : 1u;
        uint8_t FairStatus : 1u;
        uint8_t PoorStatus : 1u;
        uint8_t spare : 3u;
        int32_t FinalGrade;                                                     //
        unsigned char ReportGrade[];                                            //
        unsigned char VerificationStd[];                                        //
        unsigned char SymbologyType[];                                          //
        unsigned char DecodedTxt[];                                             //
        unsigned char CalibStat[];                                              //
        int32_t DecodeGrade;                                                    //
        int32_t ContrastGrade;                                                  //
        int32_t ModulationGrade;                                                //
        int32_t ReflctMarginGrade;                                              //
        int32_t FixedPattnDamaGrade;
        int32_t AxialNonUniformGrade;
        int32_t GridnonUniformGrade;
        int32_t UnusedErrCorrGrade;
        int32_t SymbolContrastVal;
        int32_t AxialNonUniformVal;
        int32_t GridnonUniformVal;
        int32_t UnusedErrCorrVal;
        float64_t contrastUniform;                                              // contrast uniformity
        float64_t rMax;
        float64_t rMin;
        unsigned char symbolSi[];
        int32_t cellSimil;
        float64_t cellSimm;
        float64_t cellSipix;
        int32_t printGrowthX;
        int32_t printGrowthY;
} MIRSC_iso15415_tool_t;                                                        // Symbol Quality Verification Tool Outputs contiainer structure 
#else
// ISO 15415
MICAMPACKED(
typedef struct {
        uint8_t status : 1u;                                                    // status
        uint8_t GoodStatus : 1u;
        uint8_t FairStatus : 1u;
        uint8_t PoorStatus : 1u;
        uint8_t spare : 3u;
        int32_t FinalGrade;                                                     //
        unsigned char ReportGrade[];                                            //
        unsigned char VerificationStd[];                                        //
        unsigned char SymbologyType[];                                          //
        unsigned char DecodedTxt[];                                             //
        unsigned char CalibStat[];                                              //
        int32_t DecodeGrade;                                                    //
        int32_t ContrastGrade;                                                  //
        int32_t ModulationGrade;                                                //
        int32_t ReflctMarginGrade;                                              //
        int32_t FixedPattnDamaGrade;
        int32_t AxialNonUniformGrade;
        int32_t GridnonUniformGrade;
        int32_t UnusedErrCorrGrade;
        int32_t SymbolContrastVal;
        int32_t AxialNonUniformVal;
        int32_t GridnonUniformVal;
        int32_t UnusedErrCorrVal;
        float64_t contrastUniform;                                              // contrast uniformity
        float64_t rMax;
        float64_t rMin;
        unsigned char symbolSi[];
        int32_t cellSimil;
        float64_t cellSimm;
        float64_t cellSipix;
        int32_t printGrowthX;
        int32_t printGrowthY;
}) MIRSC_iso15415_tool_t;                                                       // Symbol Quality Verification Tool Outputs contiainer structure                                                             // match tool message contiainer structure
#endif

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t status : 1u;                                                    // status
        uint8_t GoodStatus : 1u;
        uint8_t FairStatus : 1u;
        uint8_t PoorStatus : 1u;
        uint8_t spare : 3u;
        int32_t FinalGrade;                                                     //
        unsigned char ReportGrade[];                                            //
        unsigned char VerificationStd[];                                        //
        unsigned char SymbologyType[];                                          //
        unsigned char DecodedTxt[];                                             //
        unsigned char CalibStat[];                                              //
        int32_t DecodeGrade;                                                    //
        int32_t ContrastGrade;                                                  //
        int32_t ModulationGrade;                                                //
        int32_t ReflctMarginGrade;                                              //
        int32_t FixedPattnDamaGrade;
        int32_t AxialNonUniformGrade;
        int32_t GridnonUniformGrade;
        int32_t UnusedErrCorrGrade;
        int32_t MinReflctGrade;
        int32_t CellContrastVal;
        int32_t AxialNonUniformVal;
        int32_t GridnonUniformVal;
        int32_t UnusedErrCorrVal;
        int32_t MinReflectVal;
        float64_t contrastUniform;                                              // contrast uniformity
        int32_t MeanLight;
        unsigned char symbolSi[];
        int32_t cellSimil;
        float64_t cellSimm;
        float64_t cellSipix;
        int32_t printGrowthX;
        int32_t printGrowthY;
} MIRSC_iso29158_tool_t;                                                        // Symbol Quality Verification Tool Outputs contiainer structure 
#else
// ISO 29158
MICAMPACKED(
typedef struct {
        uint8_t status : 1u;                                                    // status
        uint8_t GoodStatus : 1u;
        uint8_t FairStatus : 1u;
        uint8_t PoorStatus : 1u;
        uint8_t spare : 3u;
        int32_t FinalGrade;                                                     //
        unsigned char ReportGrade[];                                            //
        unsigned char VerificationStd[];                                        //
        unsigned char SymbologyType[];                                          //
        unsigned char DecodedTxt[];                                             //
        unsigned char CalibStat[];                                              //
        int32_t DecodeGrade;                                                    //
        int32_t ContrastGrade;                                                  //
        int32_t ModulationGrade;                                                //
        int32_t ReflctMarginGrade;                                              //
        int32_t FixedPattnDamaGrade;
        int32_t AxialNonUniformGrade;
        int32_t GridnonUniformGrade;
        int32_t UnusedErrCorrGrade;
        int32_t MinReflctGrade;
        int32_t CellContrastVal;
        int32_t AxialNonUniformVal;
        int32_t GridnonUniformVal;
        int32_t UnusedErrCorrVal;
        int32_t MinReflectVal;
        float64_t contrastUniform;                                              // contrast uniformity
        int32_t MeanLight;
        unsigned char symbolSi[];
        int32_t cellSimil;
        float64_t cellSimm;
        float64_t cellSipix;
        int32_t printGrowthX;
        int32_t printGrowthY;
}) MIRSC_iso29158_tool_t;                                                       // Symbol Quality Verification Tool Outputs contiainer structure                                                             // match tool message contiainer structure
#endif

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t status : 1u;                                                    // status
        uint8_t GoodStatus : 1u;
        uint8_t FairStatus : 1u;
        uint8_t PoorStatus : 1u;
        uint8_t spare : 4u;
        int32_t FinalGrade1;                                                    //
        unsigned char ReportGrade[];                                            //
        unsigned char VerificationStd[];                                        //
        unsigned char SymbologyType[];                                          //
        unsigned char DecodedTxt[];                                             //
        unsigned char CalibStat[];                                              //
        float64_t AvgFinalGrade;                                                //
        float64_t OverallGrade;                                                 //
        float64_t AvgEdgeDetermnGrade;                                          //
        float64_t AvgDecodeGrade;                                               //
        float64_t AvgContrastGrade;
        float64_t AvgMinReflectGrade;
        float64_t AvgMinEdgeContGrade;
        float64_t AvgModulGrade;
        float64_t AvgDefectsGrade;
        float64_t AvgDecodabilityGrade;
        float64_t AvgQuietGrade;
        int32_t FinalGrade[10u];
        int32_t EdgeDeterminGrade[10u];
        int32_t DecodeGrade[10u];
        int32_t ContrastGrade[10u];
        int32_t MinReflectGrade[10u];
        int32_t MinEdgeContrastGrade[10u];
        int32_t ModulationGrade[10u];
        int32_t DefectsGrade[10u];
        int32_t DecodabilityGrade[10u];
        int32_t QuietGrade[10u];
        int32_t SymbolContrastVal[10u];
        int32_t SLRmin[10u];
        int32_t SLRMax[10u];
        int32_t MinEdgeContVal[10u];
        int32_t ModulVal[10u];
        int32_t DefectsVal[10u];
        int32_t DecodabilityVal[10u];
        float64_t QuietStartVal[10u];
        float64_t QuietStopVal[10u];
        int32_t Rmin;
        int32_t RMax;
        int32_t xDimmil;
        float64_t xDimmm;
        float64_t xDimpix;
} MIRSC_iso15416_tool_t;                                                        // match tool message contiainer structure 
#else
// ISO 15416
MICAMPACKED(
typedef struct {
        uint8_t status : 1u;                                                    // status
        uint8_t GoodStatus : 1u;
        uint8_t FairStatus : 1u;
        uint8_t PoorStatus : 1u;
        uint8_t spare : 4u;
        int32_t FinalGrade1;                                                    //
        unsigned char ReportGrade[];                                            //
        unsigned char VerificationStd[];                                        //
        unsigned char SymbologyType[];                                          //
        unsigned char DecodedTxt[];                                             //
        unsigned char CalibStat[];                                              //
        float64_t AvgFinalGrade;                                                //
        float64_t OverallGrade;                                                 //
        float64_t AvgEdgeDetermnGrade;                                          //
        float64_t AvgDecodeGrade;                                               //
        float64_t AvgContrastGrade;
        float64_t AvgMinReflectGrade;
        float64_t AvgMinEdgeContGrade;
        float64_t AvgModulGrade;
        float64_t AvgDefectsGrade;
        float64_t AvgDecodabilityGrade;
        float64_t AvgQuietGrade;
        int32_t FinalGrade[10u];
        int32_t EdgeDeterminGrade[10u];
        int32_t DecodeGrade[10u];
        int32_t ContrastGrade[10u];
        int32_t MinReflectGrade[10u];
        int32_t MinEdgeContrastGrade[10u];
        int32_t ModulationGrade[10u];
        int32_t DefectsGrade[10u];
        int32_t DecodabilityGrade[10u];
        int32_t QuietGrade[10u];
        int32_t SymbolContrastVal[10u];
        int32_t SLRmin[10u];
        int32_t SLRMax[10u];
        int32_t MinEdgeContVal[10u];
        int32_t ModulVal[10u];
        int32_t DefectsVal[10u];
        int32_t DecodabilityVal[10u];
        float64_t QuietStartVal[10u];
        float64_t QuietStopVal[10u];
        int32_t Rmin;
        int32_t RMax;
        int32_t xDimmil;
        float64_t xDimmm;
        float64_t xDimpix;
}) MIRSC_iso15416_tool_t;                                                       // match tool message contiainer structure
#endif


#define MAX_MICRO_SCAN_MSG 1024U                                                // max message sise

#if defined(D_FT900)
typedef struct MICAMPACKED {
        uint8_t ocr1_stat : 1u;
        uint8_t ocr2_stat : 1u;
        uint8_t dm_stat : 1u;
        uint8_t keep_alive : 1u;
        uint8_t camera_stat_fb : 1u;
        uint8_t camera_stat_lr : 1u;
        uint8_t spare : 2u;
        unsigned char buf[MAX_MICRO_SCAN_MSG];
        int32_t dataVal;
        float64_t center_p;
        unsigned char numPlate[ANCR_MAX_MSG];
        unsigned char beaconLR[ANCR_MAX_MSG];
        unsigned char beaconFB[ANCR_MAX_MSG];
} miCrosCanConnect_t;                                                           // match tool message contiainer structure  
#else
MICAMPACKED(
typedef struct {
        uint8_t ocr1_stat : 1u;
        uint8_t ocr2_stat : 1u;
        uint8_t dm_stat : 1u;
        uint8_t keep_alive : 1u;
        uint8_t camera_stat_fb : 1u;
        uint8_t camera_stat_lr : 1u;
        uint8_t spare : 2u;
        unsigned char buf[MAX_MICRO_SCAN_MSG];
        int32_t dataVal;
        float64_t center_p;
        unsigned char numPlate[ANCR_MAX_MSG];
        unsigned char beaconLR[ANCR_MAX_MSG];
        unsigned char beaconFB[ANCR_MAX_MSG];
}) miCrosCanConnect_t;                                                          // match tool message contiainer structure                                                               // match tool message contiainer structure
#endif

typedef union {
  int32_t sint;                                                                 // 32 bit signed
  float64_t doub;                                                               // 64 bit IEEE float
  unsigned char str[64u];                                                       // text selection
  uint8_t boolean : 1u;                                                         // boolean type
  uint8_t slot : 4u;                                                            // load slot type
  uint8_t spare : 3u;
} miCrosCanGetType_u;

#if defined(D_FT900)
typedef struct MICAMPACKED {
    unsigned char buf[MAX_MICRO_SCAN_MSG];                                      // interupt receive buffer
    miCrosCanGetType_u value;                                                   // returned type to union for get
    uint8_t type;                                                               // type defined for the value to get
    uint8_t state;                                                              // state of the get request
    uint16_t tmr;                                                               // timeout reply timer
} microScanGet_t;                                                             // match tool message contiainer structure  
#else
MICAMPACKED(
typedef struct {
    unsigned char buf[MAX_MICRO_SCAN_MSG];                                      // interupt receive buffer
    miCrosCanGetType_u value;                                                   // returned type to union for get
    uint8_t type;                                                               // type defined for the value to get
    uint8_t state;                                                              // state of the get request
    uint16_t tmr;                                                               // timeout reply timer
}) microScanGet_t;                                                             // match tool message contiainer structure
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif