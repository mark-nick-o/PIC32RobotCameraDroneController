#ifndef __MMC_FILE_HANDLE_
#define __MMC_FILE_HANDLE_
#include <stdint.h>                                                            /* standard type definitions */
#include "Struts.h"                                                             /* general structures */
#include "ilda.h"                                                               /* laser control files */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define MMCFPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define MMCFPACKED __attribute__((packed))                                    /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define MMCFPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define MMCFPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define MMCFPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define MMC_FL_ATTR_RO 0xFEu                                                    /* Read Only */
#define MMC_FL_ATTR_HID 0xFDu                                                   /* Hidden */
#define MMC_FL_ATTR_SYS 0xFBu                                                   /* System */
#define MMC_FL_ATTR_VOL 0xF7u                                                   /* Volume Label */
#define MMC_FL_ATTR_SUB 0xEFu                                                   /* Subdirectory */
#define MMC_FL_ATTR_ARC 0xDFu                                                   /* Archive  */
#define MMC_FL_ATTR_DEV 0xBFu                                                   /* Device (internal use only, never found on disk) */
#define MMC_FL_ATTR_NEW 0x7Fu                                                   /* File creation flag. If file does not exist and this flag is set, a new file with specified name will be created. */

#define MMC_FILE_READ_BUFSZ 255UL                                               /* max size in bytes to read file into a char buffer */
#define MMC_INIT_DELAY 100UL                                                    /* delay this many ticks before attempting mmc init after spi configuration complete */
#define MMC_FAT_DELAY 200UL                                                     /* delay before re-tryig the FAT initialise */
#define MMC_INIT_MAX_WAIT 2000UL                                                /* delay before failing the FAT initialise process and going to slow poll mode */
#define MMC_SLOW_DELAY 20000UL                                                  /* how often we look for mmc on system with card defined but none present or fault with holder etc */
#define MMC_FAT_RESET CORE_TICK_SECOND/2UL                                      /* reset timeout on no handle error ENSURE much greater than maximum write time set to 0.5 seconds */
#define MMC_SECTOR_SZ 512U                                                      /* mmc sector size for swap file */
#define MMC_LINE_LEN 50U                                                        /* 50 char text line writer */

/* ------------------ type defines ------------------------------------------ */
typedef enum {MMC_ILDA_HEAD=0u, MMC_3D_REC = 1u, MMC_2D_REC = 2u, MMC_CLR_IDX = 3u, MMC_TXT_ONLY = 4u, MMC_SPRAY_RCP = 5u, MMC_SPRAY_RCP2=6u, MMC_TANK_RCP_DAT=7u, MMC_FIELD_DATA=8u, MMC_REALTIME_DATA=9u, MMC_RCP_REQ_DONE=10u, MMC_NUM_OF_FILE = 11u} mmc_data_typ_e;
typedef enum {MMC_SPI_INIT=0u, MMC_DELAY = 1u, MMC_FILE_INIT = 2u, MMC_CRE_SWAP=3u, MMC_GO_READY = 4u, MMC_FAT_FAIL = 5u, MMC_FAT_WAIT=6u, MMC_FAT16_READY=7u, MMC_SLOW_POLL=8u, MMC_SLOW_WAIT=9u, MMC_FORMAT_SD=10u, MMC_FORMAT_FAIL=10u, MMC_NO_SD_CARD=11u, MMC_NUM_OF_INIT = 12u} mmc_init_e;
typedef enum {MMC_SPRAY_SEARCH=0u, MMC_SPRAY_READ_RCP = 1u, MMC_SPRAY_READ_POS = 2u, MMC_SPRAY_READ_COMP=3u, MMC_NUM_SPRAY_DATA = 4u} mmc_data_read_state_e;
typedef enum {MMC_FILE_OK = 0u, MMC_INCOMPLETE_FILE = 1u, MMC_INVALID_FILE=2u,  MMC_TYPE_INVALID = 3u, MMC_NULL_POINTER = 4u } mmc_file_read_state_e;
typedef enum {MMC_CRE_NEW_FILE = 0u, MMC_SAVE_NEW_RCP = 1u, MMC_OVERWRT_RCP=2u } mmc_data_write_state_e;
typedef enum {MMC_FORMAT_OK = 0u, MMC_FORMAT_FL = 1u, MMC_SD_ERROR=255u } mmc_format_state_e;
typedef enum {MMC_NO_FILE = 0u, MMC_FILE_EXIST = 1u, MMC_NO_HANDLER=2u } mmc_assign_state_e;
typedef enum {MMC_ERROR_INIT = 0u, MMC_ERROR_TIME = 1u, MMC_ERROR_RESET=2u } mmc_error_state_e;
/* ------- structures and unions (what we can read/write as records) -------- */
/* ========================= WRITE ========================================== */
#if (defined(GPS_POS_SPRAYER) || defined(BATCH_MIXER))
typedef union {                                                                 /* union of all file write data record structures */
  ILDAHeader_t ildahead;                                                        /* ilda header magic bytes */
  ILDA_3D_Data_Entry_t data3d;                                                  /* ilda 3d data */
  ILDA_2D_Data_Entry_t data2d;                                                  /* ilda 2d data */
  ILDA_ColorIdx_t colorIdx;                                                     /* color Index data */
  char textLine[50u];                                                           /* 50 characture text line */
  spray_product_t spraymix;                                                     /* batch mixer (sprayer solution) recipe data */
  product_id_t products;                                                        /* description of products and what makes them up */
  storedRealTimeData_t realTimeData;                                            /* realtime information stored e.g. for the spray batch make-up sequence */
} mmc_file_writ_union_t;                                                        /* union of possible mmc file write record structures */
#else
typedef union {                                                                 /* union of all file write data record structures */
  ILDAHeader_t ildahead;                                                        /* ilda header magic bytes */
  ILDA_3D_Data_Entry_t data3d;                                                  /* ilda 3d data */
  ILDA_2D_Data_Entry_t data2d;                                                  /* ilda 2d data */
  ILDA_ColorIdx_t colorIdx;                                                     /* color Index data */
  char textLine[50u];                                                           /* 50 characture text line */
  storedRealTimeData_t realTimeData;                                            /* realtime information stored e.g. for the spray batch make-up sequence */
} mmc_file_writ_union_t;                                                        /* union of possible mmc file write record structures */
#endif

/* ========================= WRITE  ========================================= */
#if defined(D_FT900)
typedef struct mmc_file_writ_t MMCFPACKED {
   mmc_file_writ_union_t filerec;                                               /* file record structure for writing data is a mix of the unions above */
} mmc_file_writ_t;
#else
 MMCFPACKED (                                                                   /* mmc library file record object */
typedef struct {
   mmc_file_writ_union_t filerec;                                               /* file record structure for writing data is a mix of the unions above */
}) mmc_file_writ_t;
#endif

/* ========================= READ  ========================================== */
#if defined(D_FT900)
typedef struct mmc_file_buff_t MMCFPACKED {
   char *pBuff; // [MMC_FILE_READ_BUFSZ];                                       /* char buf to contain the charactures from the file */
   uint32_t size;                                                               /* number of charactures in the file */
} mmc_file_buff_t;
#else
 MMCFPACKED (                                                                   /* mmc library file read back object */
typedef struct {
   char *pBuff; // [MMC_FILE_READ_BUFSZ];                                       /* char buf to contain the charactures from the file */
   uint32_t size;                                                               /* number of charactures in the file */
}) mmc_file_buff_t;                                                             /* read buffer type */
#endif

#if (defined(GPS_POS_SPRAYER) || defined(BATCH_MIXER))
typedef union {                                                                 /* union of all file write data record structures */
  spray_product_t spraymix;                                                     /* batch makeup and position sprayer data */
  product_id_t products;                                                        /* description of products and what they are composed out of */
  mmc_file_buff_t rawdata;                                                      /* raw receive data buffer */
} mmc_file_read_union_t;                                                        /* union of possible mmc file write record structures */
#else
typedef union {                                                                 /* union of all file write data record structures */
  mmc_file_buff_t rawdata;                                                      /* raw receive data buffer */
} mmc_file_read_union_t;                                                        /* union of possible mmc file write record structures */
#endif

#if defined(D_FT900)
typedef struct mmc_file_buff_t MMCFPACKED {
   mmc_file_read_union_t fileread;                                              /* file record structure for readings data is a mix of the unions above */
   storedRealTimeData_t realTimeData;                                           /* realtime information stored e.g. for the spray batch make-up sequence */
} mmc_file_read_t;
#else
 MMCFPACKED (                                                                   /* mmc library file record object */
typedef struct {
   mmc_file_read_union_t fileread;                                              /* file record structure for readings data is a mix of the unions above */
   storedRealTimeData_t realTimeData;                                           /* realtime information stored e.g. for the spray batch make-up sequence */
}) mmc_file_read_t;                                                             /* file read type */
#endif

#ifdef outout
 MMCFPACKED (                                                                    /* defines each Graphical User Interface option for sprayer files */
typedef struct {
   uint8_t saveProduct : 1u;                                                    /* hmi command to save product or fields definitions recipe  */
   uint8_t delProduct : 1u;                                                     /* hmi command to delete existing product recipe  */
   uint8_t guiDerivedState : 2u;
//   mmc_data_write_state_e guiDerivedState : 2u;                                 /* derived hmi command to create new product recipe file, save new product recipe, overwrite existing product recipe  */
   uint8_t formatDsk : 1u;                                                      /* format SD card */
   uint8_t spare : 3u;
   
   uint8_t prodSelect;                                                          /* product number selected to be written */
//   mmc_data_typ_e type : 4u;
   uint8_t type : 4u;
   uint8_t spare2 : 4u;
   
}) gui_spray_data_t;                                                            /* structure to hold the gui commands and derived state of action for file administration */
#endif

/*____________________________________________________________________________*/
/* #####################---| Function Definition |---######################## */
extern uint8_t mmcOpenFileRecord( char *filename, mmc_file_read_t *file_data, mmc_data_typ_e recTyp, uint8_t prod_chosen );
extern void initMMCFileSys( );
extern void mmcCreateNewFile( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp, const rionUTCDateTimeField_t *tim );

extern void mmcOpenFileRewrite( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp );
extern void mmcOpenFileAppend( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp );
extern void mmcOpenFileApp2Pos( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp, uint32_t *posn );
extern void mmcOpenFileRead( char *filename, mmc_file_buff_t *ReadDat );
extern void mmcOpenFileAppRec( char *filename, mmc_file_read_t *file_data, mmc_file_writ_t *writ_data, uint8_t prod_chosen );
extern uint8_t mmcManageRecords( char *filename, mmc_file_read_t *file_data,  mmc_data_typ_e recTyp, mmc_file_writ_t file_write, gui_spray_data_t *guiReq, const rionUTCDateTimeField_t *tim );
//extern uint8_t mmcManageRecords( char *filename, mmc_file_read_t *file_data, mmc_data_typ_e recTyp, mmc_file_writ_t file_write );

extern mmc_init_e g_mmcStep;                                                    /* mmc initialisation steps */
extern mmc_init_e g_FATprevState;                                               /* state used to tell caller we have had our first FAT request failure */
extern int32_t g_mmcTimeDuration;                                               /* global mmc start-up timer */
extern uint32_t g_mmcTimeDurationLast;                                          /* global mmc start-up timer start point (time from reference) */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* --- End mmc file handler --- */