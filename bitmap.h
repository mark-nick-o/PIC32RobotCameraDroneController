#ifndef __bmpfile_
#define __bmpfile_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define BMPFILEPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define BMPFILEPACKED __attribute__((packed)) ALIGNED(1)                         /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define BMPFILEPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define BMPFILEPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define BMPFILEPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#if defined(D_FT900)
typedef struct BMPFILEPACKED {
   uint32_t filesize;
   uint16_t undefined0;
   uint16_t undefined1;
   uint32_t dataoffset;
} bmp_header_t;                                                                 /* bmp file header */
#else
BMPFILEPACKED(
typedef struct {
   uint32_t filesize;
   uint16_t undefined0;
   uint16_t undefined1;
   uint32_t dataoffset;
}) bmp_header_t;                                                                /* bmp file header */
#endif

#if defined(D_FT900)
typedef struct BMPFILEPACKED {
    uint32_t dipsize;
    int32_t  width;
    int32_t  height;
    uint16_t nplanes;
    uint16_t depth;
    uint32_t compress_type;
    uint32_t bmp_bytesz;
    int32_t  hres;
    int32_t  vres;
    uint32_t ncolors;
    uint32_t nimpcolors;
} dib_header_t;                                                                 /* bmp file dib */
#else
BMPFILEPACKED(
typedef struct {
    uint32_t dipsize;
    int32_t  width;
    int32_t  height;
    uint16_t nplanes;
    uint16_t depth;
    uint32_t compress_type;
    uint32_t bmp_bytesz;
    int32_t  hres;
    int32_t  vres;
    uint32_t ncolors;
    uint32_t nimpcolors;
}) dib_header_t;                                                                /* bmp file dib */
#endif

#if defined(D_FT900)
typedef struct BMPFILEPACKED {
    uint16_t value;
} bmp_pixel_t;                                                                  /* bmp file pixel */
#else
BMPFILEPACKED(
typedef struct {
    uint16_t value;
}) bmp_pixel_t;                                                                 /* bmp file pixel */
#endif

#if defined(D_FT900)
typedef struct BMPFILEPACKED {
    bmp_header_t  fheader;
    dib_header_t  dheader;
    bmp_pixel_t ** pixels;
} bitmap_t;                                                                     /* bmp file */
#else
BMPFILEPACKED(
typedef struct {
    bmp_header_t  fheader;
    dib_header_t  dheader;
    bmp_pixel_t ** pixels;
}) bitmap_t;                                                                    /* bmp file  */
#endif

typedef int16_t pixel_t;

#if defined(D_FT900)
typedef struct BMPFILEPACKED {
  pixel_t x;
  pixel_t y;
} picsel_p_t;                                                                     /* bmp file */
#else
BMPFILEPACKED(
typedef struct {
  pixel_t x;
  pixel_t y;
}) picsel_p_t;                                                                    /* bmp file  */
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif