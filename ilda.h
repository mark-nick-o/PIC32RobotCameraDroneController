#ifndef __ILDA_FILE_HANDLE_
#define __ILDA_FILE_HANDLE_
// ILDA
// ====================================================================================================================================
// The ILDA image data transfer format (file extension .ild) was designed by the International Laser Display Association to facilitate
// the transfer of laser show content between sites and systems. It supports the storage of not just individual frames/objects but time 
// varying sequences (animations). It is a binary format employing the so-called "big endian" byte ordering
//
// A ILDA file can describe three different types of data: 2D coordinates, 3D coordinates, and index colour palettes. 
// Each data type is represented in a ILDA file by a header followed by the data. These types (sections) can appear in any order 
// except that colour palette data is supposed to precede the 2D or 3D data it applies to.
//
// In PIC32 we have no filesystem so the file would be recieved on either serial or via ethernet e.g. http
// or we can create them and send them from C code to the laser device
// A MMC FAT16 library has been added to allow file creation
//
//
//  ilda.h - ILDA file "native" operations.
//
// * Written by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define ILDAPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define ILDAPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define ILDAPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define ILDAPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define ILDAPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#if defined(D_FT900)
typedef struct ILDAPACKED {
  unsigned char signature[4u];                                                  // signature "ILDA"
  uint8_t spare[3u];                                                            // should be set to zero
  uint8_t formatTyp;                                                            // format type
  unsigned char Name[8u];                                                       // name
  unsigned char CompanyName[8u];                                                // company name
  uint16_t NumOfEntry;                                                          // For coordinates this is the number of points in the following data section, for colour palettes it is the number of entries in the palette. Using this along with the known size for the data section entries allows a parsing program to skip over sections they weren't interested.
  uint16_t CurrentFrameNum;                                                     // For files that contain a number of frames, eg: library of graphical shapes, collection of colour palettes, or an animation sequence, this is the current number. It ranges from 0 up to one the total number of frames minus 1
  uint16_t TotalFrameNum;                                                       // The total number of frames and is not used for colour palette format types. This is set to 0 in a "null header" to indicate the end of the ILDA file
  uint8_t ScannerHead;                                                          // Used for systems with multiple scanners or heads, otherwise set to 0 for the default device.
  uint8_t unused;                                                               // unused
} ILDAHeader_t;
#else
ILDAPACKED(
typedef struct {
  unsigned char signature[4u];                                                  // signature "ILDA"
  uint8_t spare[3u];                                                            // should be set to zero
  uint8_t formatTyp;                                                            // format type
  unsigned char Name[8u];                                                       // name
  unsigned char CompanyName[8u];                                                // company name
  uint16_t NumOfEntry;                                                          // For coordinates this is the number of points in the following data section, for colour palettes it is the number of entries in the palette. Using this along with the known size for the data section entries allows a parsing program to skip over sections they weren't interested.
  uint16_t CurrentFrameNum;                                                     // For files that contain a number of frames, eg: library of graphical shapes, collection of colour palettes, or an animation sequence, this is the current number. It ranges from 0 up to one the total number of frames minus 1
  uint16_t TotalFrameNum;                                                       // The total number of frames and is not used for colour palette format types. This is set to 0 in a "null header" to indicate the end of the ILDA file
  uint8_t ScannerHead;                                                          // Used for systems with multiple scanners or heads, otherwise set to 0 for the default device.
  uint8_t unused;                                                               // unused
}) ILDAHeader_t;
#endif

#define ILDA_READ_BLANKING(status) ((status & 0x40u) >> 6u);                    // For the blanking a 0 indicates the laser is on, 1 is blanked. The end point indicator is set to 1 to indicate the end. The unused bits 0 to 5 in the state byte are supposed to be set to 0.
#define ILDA_READ_LASTPOINT(status) ((status & 0x80u) >> 7u);
#define ILDA_SET_BLANKING(status,blank) ((blank << 6u) | status);
#define ILDA_SET_LAST_POINT(status,last) ((last << 7u) | status);               // the last point sequence

#define ILDA_FORMAT_3D 0u                                                       // type for the file or data entry section data is as per structs below
#define ILDA_FORMAT_2D 1u
#define ILDA_COLOR_PALETTE 2u

#if defined(D_FT900)
typedef struct ILDAPACKED {
  uint16_t x_coord;                                                             // x co-ord
  uint16_t y_coord;                                                             // y co-ord
  uint16_t z_coord;                                                             // z co-ord  set to zero for 2d
  uint8_t laserstate;                                                           // laser state
  uint8_t colorindex;                                                           // color index
} ILDA_3D_Data_Entry_t;
#else
ILDAPACKED(
typedef struct {
  uint16_t x_coord;                                                             // x co-ord
  uint16_t y_coord;                                                             // y co-ord
  uint16_t z_coord;                                                             // z co-ord  set to zero for 2d
  uint8_t laserstate;                                                           // laser state
  uint8_t colorindex;                                                           // color index
}) ILDA_3D_Data_Entry_t;                                                        // 3d data entry point (can send multiples in file set lastpoint at end
#endif

#if defined(D_FT900)
typedef struct ILDAPACKED {
  uint16_t x_coord;                                                             // x co-ord
  uint16_t y_coord;                                                             // y co-ord
  uint8_t laserstate;                                                           // laser state
  uint8_t colorindex;                                                           // color index
} ILDA_2D_Data_Entry_t;
#else
ILDAPACKED(
typedef struct {
  uint16_t x_coord;                                                             // x co-ord
  uint16_t y_coord;                                                             // y co-ord
  uint8_t laserstate;                                                           // laser state
  uint8_t colorindex;                                                           // color index
}) ILDA_2D_Data_Entry_t;                                                        // 2d data entry point (can send multiples in file set lastpoint at end in status byte)
#endif

#if defined(D_FT900)
typedef struct ILDAPACKED {
  uint8_t R_Red;                                                                // Red
  uint8_t G_Green;                                                              // Green
  uint8_t B_Blue;                                                               // Blue
} ILDA_ColorIdx_t;                                                             // for one color index entry
#else
ILDAPACKED(
typedef struct {
  uint8_t R_Red;                                                                // Red
  uint8_t G_Green;                                                              // Green
  uint8_t B_Blue;                                                               // Blue
}) ILDA_ColorIdx_t;                                                             // for one color index entry
#endif

#define SWAP_2(x) ( (((x) & 0xff) << 8) | ((uint16_t)(x) >> 8) )                // endian swaps
#define FIX_SHORT(x) (*(uint16_t *)&(x) = SWAP_2(*(uint16_t *)&(x)))

#ifdef __cplusplus
}
#endif

#endif /* ---- end ilda library ---- */