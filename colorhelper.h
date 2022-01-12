#ifndef __ColorHelp_h
#define __ColorHelp_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* 
  colorhelper.h : Color helper library various codecs conversion 
                   macros and functions for color and video streams

 Written by (C) 2020 A C P Avaiation Walkerburn Scotland

 YUV format "The human eye is sensitive to changes in brightness, but insensitive 
             to changes in color."

 Thus, by suppressing chromaticity and dividing a wider band or number of 
 bits into luminance, it is more efficient with less loss A format that realizes
 transmission and compression.  */
#include <stdint.h>
#include "bitmap.h"                                                             /* where manipulation is pf bitmap image rather than video stream */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define COLORPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define COLORPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define COLORPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define COLORPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define COLORPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

typedef int16_t* cv_Mat;                                                        /* mimic of open cv type */

#if defined(D_FT900)
typedef struct COLORPACKED {
   uint8_t h;
   uint8_t s;
   uint8_t l;
} HSL_t;
#else
COLORPACKED(
typedef struct {
   uint8_t h;
   uint8_t s;
   uint8_t l;
}) HSL_t;
#endif

#if defined(D_FT900)
typedef struct STRUTPACKED {
  float32_t x;
  float32_t y;
} f32point_t;
#else
STRUTPACKED(
typedef struct {
  float32_t x;
  float32_t y;
}) f32point_t;
#endif

#if defined(D_FT900)
typedef struct STRUTPACKED {
  float64_t x;
  float64_t y;
} f64point_t;
#else
STRUTPACKED(
typedef struct {
  float64_t x;
  float64_t y;
}) f64point_t;
#endif

#if defined(D_FT900)
typedef struct STRUTPACKED {
  float32_t x;
  float32_t y;
  int16_t radius;
} f32circle_t;
#else
STRUTPACKED(
typedef struct {
  float32_t x;
  float32_t y;
  int16_t radius;
}) f32circle_t;
#endif

#if defined(D_FT900)
typedef struct STRUTPACKED {
  f32point_t nodes[2u];
} f32edge_t;
#else
STRUTPACKED(
typedef struct {
  f32point_t nodes[2u];
}) f32edge_t;
#endif

#if defined(D_FT900)
typedef struct STRUTPACKED {
  f32point_t nodes[];
  f32edge_t edges[];
  f32circle_t circle;
} f32triangle_t;
#else
STRUTPACKED(
typedef struct {
  f32point_t nodes[];
  f32edge_t edges[];
  f32circle_t circle;
}) f32triangle_t;
#endif

#if defined(D_FT900)
typedef struct STRUTPACKED {
  int16_t width;
  int16_t height;
  f32triangle_t triangles[];
} f32delaunay_t;
#else
STRUTPACKED(
typedef struct {
  int16_t width;
  int16_t height;
  f32triangle_t triangles[];
}) f32delaunay_t;
#endif

#if defined(D_FT900)
typedef struct STRUTPACKED {
  uint8_t blackWhite : 1u;
  uint8_t invert : 1u;
  uint8_t spare : 6u;
} colOptions_t;
#else
STRUTPACKED(
typedef struct {
  uint8_t blackWhite : 1u;
  uint8_t invert : 1u;
  uint8_t spare : 6u;
}) colOptions_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
   uint8_t r;
   uint8_t g;
   uint8_t b;
   uint8_t red_channel;
   uint8_t green_channel;
   uint8_t blue_channel;
   uint8_t yellow_channel;
} hue_t;
#else
COLORPACKED(
typedef struct {
   uint8_t r;
   uint8_t g;
   uint8_t b;
   uint8_t red_channel;
   uint8_t green_channel;
   uint8_t blue_channel;
   uint8_t yellow_channel;
}) hue_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
   uint8_t r;
   uint8_t g;
   uint8_t b;
} COLOUR_t;
#else
COLORPACKED(
typedef struct {
   uint8_t r;
   uint8_t g;
   uint8_t b;
}) COLOUR_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
   uint16_t xsize;
   uint16_t ysize;
   float64_t dataV[];
} image_double_t;
#else
COLORPACKED(
typedef struct {
   uint16_t xsize;
   uint16_t ysize;
   float64_t dataV[];
}) image_double_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
   float64_t i;
   float64_t j;
   float64_t x;
   float64_t y;
   float64_t lat;
   float64_t lon;
} image_coord_t;
#else
COLORPACKED(
typedef struct {
   float64_t i;
   float64_t j;
   float64_t x;
   float64_t y;
   float64_t lat;
   float64_t lon;
}) image_coord_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
  float32_t x;
  float32_t y;
  float32_t z;
} Cam2pixel_t;
#else
COLORPACKED(
typedef struct {
  float32_t x;
  float32_t y;
  float32_t z;
}) Cam2pixel_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
  int16_t rgba;
  float32_t x;
  float32_t y;
  float32_t z;
} PointXYZRGB_t;
#else
COLORPACKED(
typedef struct {
  int16_t rgba;
  float32_t x;
  float32_t y;
  float32_t z;
}) PointXYZRGB_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
  unsigned char ev_buffer[];
  float32_t fr_x;
  float32_t fr_y;
  uint64_t timestamp;
  unsigned char frame_id[];
  uint8_t timerRef : 1u;
  uint8_t spare : 7u;
} PointCloud_t;
#else
COLORPACKED(
typedef struct {
  unsigned char ev_buffer[];
  float32_t fr_x;
  float32_t fr_y;
  uint64_t timestamp;
  unsigned char frame_id[];
  uint8_t timerRef : 1u;
  uint8_t spare : 7u;
}) PointCloud_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
  Vectr points[];
  uint64_t timestamp;
  unsigned char frame_id[];
} VectrPointCloud_t;
#else
COLORPACKED(
typedef struct {
  Vectr points[];
  uint64_t timestamp;
  unsigned char frame_id[];
}) VectrPointCloud_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
  Vectr le;
  Vectr re;
} Cam_Eye_t;
#else
COLORPACKED(
typedef struct {
  Vectr le;
  Vectr re;
}) Cam_Eye_t;
#endif

/* ================== voxel 3D ========================================= */
#if defined(D_FT900)
typedef struct COLORPACKED {
   uint8_t r;
   uint8_t g;
   uint8_t b;
   uint8_t a;
} voxel_rgba_color_t;
#else
COLORPACKED(
typedef struct {
   uint8_t r;
   uint8_t g;
   uint8_t b;
   uint8_t a;
}) voxel_rgba_color_t;                                                          /* this is used in others as its rgba type */
#endif

#if defined(REQUIRE_VOXEL_CODEC)
#if defined(D_FT900)
typedef struct COLORPACKED {
  uint32_t distance;
  uint32_t weight;
  voxel_rgba_color_t color;
} TsdfVoxel_t;
#else
COLORPACKED(
typedef struct {
  uint32_t distance;
  uint32_t weight;
  voxel_rgba_color_t color;
}) TsdfVoxel_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
  uint32_t intensity;
  uint32_t weight;
} BlockIntensityVoxel_t;
#else
COLORPACKED(
typedef struct {
  uint32_t intensity;
  uint32_t weight;
}) BlockIntensityVoxel_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
    uint32_t probability_log;
    uint8_t observed;
} OccupancyVoxel_t;
#else
COLORPACKED(
typedef struct {
    uint32_t probability_log;
    uint8_t observed;
}) OccupancyVoxel_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
    int8_t x;
    int8_t y;
    int8_t z;
} Voxel_Vectr_t;
#else
COLORPACKED(
typedef struct {
    int8_t x;
    int8_t y;
    int8_t z;
}) Voxel_Vectr_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
    uint32_t distance;
    uint8_t observed : 1u;
    uint8_t hallucinated : 1u;
    uint8_t in_queue : 1u;
    uint8_t fixed : 1u;
    uint8_t state_block : 4u;
    Voxel_Vectr_t parent;
} EsdfVoxel_t;
#else
COLORPACKED(
typedef struct {
    uint32_t distance;
    uint8_t observed : 1u;
    uint8_t hallucinated : 1u;
    uint8_t in_queue : 1u;
    uint8_t fixed : 1u;
    uint8_t state_block  : 4u;
    Voxel_Vectr_t parent;
}) EsdfVoxel_t;
#endif
#endif
/* ================== canberra UAV ========================================= */
#if defined(D_FT900)
typedef struct COLORPACKED {
  uint16_t n;
  uint16_t min;
  uint16_t max;
  float32_t mean;
  float32_t m2;
  float32_t variance;
} image_stats_t;
#else
COLORPACKED(
typedef struct {                                                                
  uint16_t n;                                                                   // number of samples
  uint16_t min;                                                                 // minimum pixel value found
  uint16_t max;                                                                 // maximum pixel value found
  float32_t mean;                                                               // mean
  float32_t m2;                                                                 // running mean squared
  float32_t variance;                                                           // variance (population)
}) image_stats_t;
#endif

#define CUAV_PIXEL_SIZE 3u
#define DEBAYER_TOP_AVAIL 0x01u
#define DEBAYER_BOTTOM_AVAIL 0x02u
#define DEBAYER_LEFT_AVAIL 0x04u
#define DEBAYER_RIGHT_AVAIL 0x08u

typedef enum 
{
  RED,
  GREEN,
  BLUE
} CUAV_chan;

typedef enum 
{
  PIXOP_YUV,
  PIXOP_RGB
} CUAV_pixop_e;

#if defined(D_FT900)
typedef struct COLORPACKED {
  int16_t px;                                                                   // the pixel in question [0 - 4]
  int16_t py;
  CUAV_chan ch;                                                                 // the channel
  int16_t dx;                                                                   // the offsets relative to the src pixel in the 4x4 bayer block
  int16_t dy;
} CUAV_bayer_t;
#else
COLORPACKED(
typedef struct {
  int16_t px;                                                                   // the pixel in question [0 - 4]
  int16_t py;
  CUAV_chan ch;                                                                 // the channel
  int16_t dx;                                                                   // the offsets relative to the src pixel in the 4x4 bayer block
  int16_t dy;
}) CUAV_bayer_t;
#endif

#ifndef _MY_IMAGE_HEADER_                                                       /* ported from paparazzi */
#define _MY_IMAGE_HEADER_
/*
 * Copyright (C) 2012-2013
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
#if defined(D_FT900)
typedef struct COLORPACKED {
  int16_t seq;
  float64_t timestamp;
  unsigned char *buf;
  int16_t w;
  int16_t h;
} img_struct_t;
#else
COLORPACKED(
typedef struct {
  int16_t seq;
  float64_t timestamp;
  unsigned char *buf;
  int16_t w;
  int16_t h;
}) img_struct_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
  uint8_t base;                                                                 /* base r g b componant */
  uint8_t strength;                                                             /* strength default 96 */
  uint8_t radius;
} LensShaderColor_t;
#else
COLORPACKED(
typedef struct {
  uint8_t base;                                                                 /* base r g b componant */
  uint8_t strength;                                                             /* strength default 96 */
  uint8_t radius;
}) LensShaderColor_t;
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
  float32_t *pfloat;                                                            /* pointer to the data */     
  int16_t rows;                                                                 /* number of rows */
  int16_t cols;                                                                 /* number of columns */
} gradientMap_t;                                                                /* color difference gradient */
#else
COLORPACKED(
typedef struct {
  float32_t *pfloat;                                                            /* pointer to the data */     
  int16_t rows;                                                                 /* number of rows */
  int16_t cols;                                                                 /* number of columns */
}) gradientMap_t;                                                               /* color difference gradient */
#endif

#if defined(D_FT900)
typedef struct COLORPACKED {
  float32_t i;  
  float32_t f;     
} Cv32suf;                                                                      /* open CV type */
#else
COLORPACKED(
typedef struct {
  float32_t i;  
  float32_t f;                                                              
}) Cv32suf;                                                             
#endif


#endif /* end MY_IMAGE */

/* ========== format codecs and re-sizers =================================== */
#define CLIP(x) do{if(x < 0){x = 0;} else if(x > 255){x = 255;}} while(0)
#define MAX_BRIGHTNESS 255
/* ----------------- PAL television standard -----------------------------    */
/* YUV to RGB conversions                                                     */
#define CONVERT_YUV_R(Y, V) ((298 * (Y - 16) + 409 * (V - 128) + 128) >> 8)
#define CONVERT_YUV_G(Y, U, V) ((298 * (Y - 16) - 100 * (U - 128) - 208 * (V - 128) + 128) >> 8)
#define CONVERT_YUV_B(Y, U) ((298 * (Y - 16) + 516 * (U - 128) + 128) >> 8)
/* RGB to YUV conversions                                                     */
#define CONVERT_RGB_Yuv(R, G, B) ( (0.299f  * R) + (0.587f * G) + (0.114f * B) )
#define CONVERT_RGB_U(R, G, B) ( (-0.147f  * R) + (-0.289f * G) + (0.436f * B) )
#define CONVERT_RGB_V(R, G, B) ( (0.615f  * R) + (-0.515f * G) + (-0.100f  * B) )
/* ----------------- NTSC television standard -----------------------------   */
/* YIQ to RGB */
#define CONVERT_YIQ_R(Y, I, Q) ( Y + (0.956f * I) + (0.621f * Q) )
#define CONVERT_YIQ_G(Y, I, Q) ( Y - (0.272f * I) - (0.647f * Q) )
#define CONVERT_YIQ_B(Y, I, Q) ( Y - (1.105f * I) + (1.702f * Q) )
/*  RGB to YIQ */
#define CONVERT_RGB_Yiq(R, G, B) ( (0.289f * R) + (0.587f * G) + (0.114f * B) )   /* Note: First line Y = (0.299, 0.587, 0.144) (R,G,B) also gives pure B&W translation for RGB. */
#define CONVERT_RGB_I(R, G, B) ( (0.596f * R) - (0.274f * G) - (0.322f * B) )
#define CONVERT_RGB_Q(R, G, B) ( (0.212f * R) - (0.523f * G) + (0.311f * B) )
/* ----------------- D65 CIE XYZ Zitu --------------------------------------- */
#define CONV_ZITU_X(R,G,B) ( 0.431f*R + 0.342f*G + 0.178f*B )
#define CONV_ZITU_Y(R,G,B) ( 0.222f*R + 0.707f*G + 0.071f*B )
#define CONV_ZITU_Z(R,G,B) ( 0.020f*R + 0.130f*G + 0.939f*B )
#define CONV_ZITU_R(X,Y,Z) ( 3.063f*X -1.393f*Y -0.476f*Z )
#define CONV_ZITU_G(X,Y,Z) ( -0.969f*X + 1.876f*Y + 0.042f*Z )
#define CONV_ZITU_B(X,Y,Z) ( 0.068f*X -0.229f*Y + 1.069f*Z )
/* ---------------- CIE XYZrec601-1 (C illuminant) -------------------------- */
#define CONV_REC601_X(R,G,B) ( 0.607f*R + 0.174f*G + 0.200f*B )
#define CONV_REC601_Y(R,G,B) ( 0.299f*R + 0.587f*G + 0.114f*B )
#define CONV_REC601_Z(G,B) ( 0.066f*G + 1.116f*B )
#define CONV_REC601_R(X,Y,Z) ( 1.910f*X - 0.532f*Y - 0.288f*Z )
#define CONV_REC601_G(X,Y,Z) ( -0.985f*X + 1.999f*Y -0.028f*Z )
#define CONV_REC601_B(X,Y,Z) (  0.058f*X -0.118f*Y + 0.898f*Z )
/* ---------------- CIE XYZccir709 (D65) CIE 1931 XYZ------------------------ */
#define CONV_CCIR_X(R,G,B) ( 0.4124f*R + 0.3576f*G + 0.1805f*B )
#define CONV_CCIR_Y(R,G,B) ( 0.2126f*R + 0.7152f*G + 0.0722f*B )
#define CONV_CCIR_Z(R,G,B) ( 0.0193f*R + 0.1192f*G + 0.9505f*B )
#define CONV_CCIR_R(X,Y,Z) ( 3.241f*X - 1.537f*Y - 0.499f*Z )
#define CONV_CCIR_G(X,Y,Z) ( -0.969f*X + 1.876f*Y + 0.042f*Z )
#define CONV_CCIR_B(X,Y,Z) ( 0.056f*X - 0.204f*Y + 1.057f*Z )
/* The YCC colour space used follows that used by TIFF and JPEG (Rec 601-1) */
#define CONV_YCC_Y(R,G,B)  ( 0.2989f*R + 0.5866f*G + 0.1145f*B )
#define CONV_YCC_Cb(R,G,B) ( -0.1687f*R - 0.3312f*G + 0.5000f*B )
#define CONV_YCC_Cr(R,G,B) ( 0.5000f*R - 0.4183f*G - 0.0816f*B )
#define CONV_YCC_R(Y,Cb,Cr) (Y + 1.4022f*Cr)
#define CONV_YCC_G(Y,Cb,Cr) (Y - 0.3456f*Cb - 0.7145f*Cr)
#define CONV_YCC_B(Y,Cb,Cr) (Y + 1.7710f*Cb)
/* Kodak photo CD */
#define CONV_KODAKYCC_Y(R,G,B)  ( 0.299f*R + 0.587f*G + 0.114f*B )
#define CONV_KODAKYCC_Cb(R,G,B) ( 0.701f*R - 0.587f*G - 0.114f*B )
#define CONV_KODAKYCC_Cr(R,G,B) ( -0.299f*R - 0.587f*G + 0.886f*B )

/*  RGB to CMY */
#define CONVERT_RGB_C(R) (255 - R)
#define CONVERT_RGB_M(G) (255 - G)
#define CONVERT_RGB_Y(B) (255 - B)
/* CMY to RGB */
#define CONVERT_CMY_R(C) (255 - C)
#define CONVERT_CMY_G(M) (255 - M)
#define CONVERT_CMY_B(Y) (255 - Y)
/* RGB to Y BY RY */
#define CONVERT_RGB_Y1(R,G,B) (((0.300f * R) + (0.590f * G)) + (0.110f * B))
#define CONVERT_RGB_BY(R,G,B) (-0.300f * R)-(0.590f * G) + (0.890f * B)
#define CONVERT_RGB_RY(R,G,B) (0.700f * R)-(0.590f * G)- (0.110f * B)
/* Y BY RY to RGB */
#define CONVERT_YBYRY_R(Y,RY) (1.000f * Y) + (1.000f * RY)
#define CONVERT_YBYRY_G(Y,BY,RY) (1.000f * Y) - (0.186f * BY) - (0.508f * RY)
#define CONVERT_YBYRY_B(Y,BY) (1.000f * Y)  + (1.000f * BY)
/* avery lees jfif */
#define CONVERT_JFIF_R(Y,Cr) (Y + (1.402f*(Cr-128)))
#define CONVERT_JFIF_G(Y,Cb,Cr) (Y - (0.34414f * (Cb-128)) - (0.71414f * (Cr-128)))
#define CONVERT_JFIF_B(Y,Cb) (Y + (1.772f * (Cb-128)))
/* for vegetation analysis from RGB values */
#define XS_GREEN_IDX_ExG(R,B) ((2-R)-B)
#define XS_RED_IDX_ExR(R,G) ((1.4f*R)-G)
#define NORMALIZED_DIFF_IDX_NDI(G,R) ((G-R)/(G+R))
#define CIVE(R,G,B) (0.441f*R)-(0.811f*G)+(0.385f*B)+18.78745f
#define XSG_MINUS_XSR(R,G,B) (((2-R)-B)-((1.4f*R)-G))
/* for hough transform */
#define GR(X,Y) (d[(*surfceVal)*(Y)+bpp*(X)+((2)%bpp)])
#define GG(X,Y) (d[(*surfceVal)*(Y)+bpp*(X)+((1)%bpp)])
#define GB(X,Y) (d[(*surfceVal)*(Y)+bpp*(X)+((0)%bpp)])
#define SR(X,Y) (ht[4*tw*((Y)%th)+4*((X)%tw)+2u])
#define SG(X,Y) (ht[4*tw*((Y)%th)+4*((X)%tw)+1u])
#define SB(X,Y) (ht[4*tw*((Y)%th)+4*((X)%tw)+0u])
#define RAD(A)  (PI*((float64_t)(A))/180.0f)

#define Black 0x000000ul                                                        // Color definitions to call the actual RGB color in the program
#define White 0xFFFFFFul
#define Red 0xFF0000ul
#define Lime 0x00FF00ul
#define Blue 0x0000FFul
#define Yellow 0xFFFF00ul
#define Cyan 0x00FFFFul
#define Magenta 0xFF00FFul
#define Silver 0xC0C0C0ul
#define Gray 0x808080ul
#define Maroon 0x800000ul
#define Olive 0x808000ul
#define Green 0x008000ul
#define Purple 0x800080ul
#define Teal 0x008080ul
#define Navy 0x000080ul
#define maroon 0x800000ul
#define darkred 0x8B0000ul
#define brown 0xA52A2Aul
#define firebrick 0xB22222ul
#define crimson 0xDC143Cul
#define red 0xFF0000ul
#define tomato 0xFF6347ul
#define coral 0xFF7F50ul
#define indianred 0xCD5C5Cul
#define lightcoral 0xF08080ul
#define darksalmon 0xE9967Aul
#define salmon 0xFA8072ul
#define lightsalmon 0xFFA07Aul
#define orangered 0xFF4500ul
#define darkorange 0xFF8C00ul
#define orange 0xFFA500ul
#define gold 0xFFD700ul
#define darkgoldenrod 0xB8860Bul
#define goldenrod 0xDAA520ul
#define palegoldenrod 0xEEE8AAul
#define darkkhaki 0xBDB76Bul
#define khaki 0xF0E68Cul
#define olive 0x808000ul
#define yellow 0xFFFF00ul
#define yellowgreen 0x9ACD32ul
#define darkolivegreen 0x556B2Ful
#define olivedrab 0x6B8E23ul
#define lawngreen 0x7CFC00ul
#define chartreuse 0x7FFF00ul
#define greenyellow 0xADFF2Ful
#define darkgreen 0x006400ul
#define green 0x008000ul
#define forestgreen 0x228B22ul
#define lime 0x00FF00ul
#define limegreen 0x32CD32ul
#define lightgreen 0x90EE90ul
#define palegreen 0x98FB98ul
#define darkseagreen 0x8FBC8Ful
#define mediumspringgreen 0x00FA9Aul
#define springgreen 0x00FF7Ful
#define seagreen 0x2E8B57ul
#define mediumaquamarine 0x66CDAAul
#define mediumseagreen 0x3CB371ul
#define lightseagreen 0x20B2AAul
#define darkslategray 0x2F4F4Ful
#define teal 0x008080ul
#define darkcyan 0x008B8Bul
#define aqua 0x00FFFFul
#define cyan 0x00FFFFul
#define lightcyan 0xE0FFFFul
#define darkturquoise 0x00CED1ul
#define turquoise 0x40E0D0ul
#define mediumturquoise 0x48D1CCul
#define paleturquoise 0xAFEEEEul
#define aquamarine 0x7FFFD4ul
#define powderblue 0xB0E0E6ul
#define cadetblue 0x5F9EA0ul
#define steelblue 0x4682B4ul
#define cornflowerblue 0x6495EDul
#define deepskyblue 0x00BFFFul
#define dodgerblue 0x1E90FFul
#define lightblue 0xADD8E6ul
#define skyblue 0x87CEEBul
#define lightskyblue 0x87CEFAlul
#define midnightblue 0x191970ul
#define navy 0x000080ul
#define darkblue 0x00008Bul
#define mediumblue 0x0000CDul
#define blue 0x0000FFul
#define royalblue 0x4169E1ul
#define blueviolet 0x8A2BE2ul
#define indigo 0x4B0082ul
#define darkslateblue 0x483D8Bul
#define slateblue 0x6A5ACDul
#define mediumslateblue 0x7B68EEul
#define mediumpurple 0x9370DBul
#define darkmagenta 0x8B008Bul
#define darkviolet 0x9400D3ul
#define darkorchid 0x9932CCul
#define mediumorchid 0xBA55D3ul
#define purple 0x800080ul
#define thistle 0xD8BFD8ul
#define plum 0xDDA0DDul
#define violet 0xEE82EEul
#define orchid 0xDA70D6ul
#define mediumvioletred 0xC71585ul
#define palevioletred 0xDB7093ul
#define deeppink 0xFF1493ul
#define hotpink 0xFF69B4ul
#define lightpink 0xFFB6C1ul
#define pink 0xFFC0CBul
#define antiquewhite 0xFAEBD7ul
#define beige 0xF5F5DCul
#define bisque 0xFFE4C4ul
#define blanchedalmond 0xFFEBCDul
#define wheat 0xF5DEB3ul
#define cornsilk 0xFFF8DCul
#define lemonchiffon 0xFFFACDul
#define lightgoldenrodyellow 0xFAFAD2ul
#define lightyellow 0xFFFFE0ul
#define saddlebrown 0x8B4513ul
#define sienna 0xA0522Dul
#define chocolate 0xD2691Eul
#define peru 0xCD853Ful
#define sandybrown 0xF4A460ul
#define burlywood 0xDEB887ul
#define colortan 0xD2B48Cul
#define rosybrown 0xBC8F8Ful
#define moccasin 0xFFE4B5ul
#define navajowhite 0xFFDEADul
#define peachpuff 0xFFDAB9ul
#define mistyrose 0xFFE4E1ul
#define lavenderblush 0xFFF0F5ul
#define linen 0xFAF0E6ul
#define oldlace 0xFDF5E6ul
#define papayawhip 0xFFEFD5ul
#define seashell 0xFFF5EEul
#define mintcream 0xF5FFFAul
#define slategray 0x708090ul
#define lightslategray 0x778899ul
#define lightsteelblue 0xB0C4DEul
#define lavender 0xE6E6FAul
#define floralwhite 0xFFFAF0ul
#define aliceblue 0xF0F8FFul
#define ghostwhite 0xF8F8FFul
#define honeydew 0xF0FFF0ul
#define ivory 0xFFFFF0ul
#define azure 0xF0FFFFul
#define snow 0xFFFAFAul
#define black 0x000000ul
#define dimgray 0x696969ul
#define grey 0x808080ul
#define darkgrey 0xA9A9A9ul
#define silvercoco 0xC0C0C0ul
#define lightgray 0xD3D3D3ul
#define gainsboro 0xDCDCDCul
#define whitesmoke 0xF5F5F5ul
#define white 0xFFFFFFul

// structures
typedef enum
{
    RGB_RED = 0,
    RGB_GREEN,
    RGB_BLUE
} colorComponent_e;

#define RGB_COLOR_COMPONENT_COUNT (RGB_BLUE + 1)

struct rgbColor24bpp_s
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

typedef union
{
    struct rgbColor24bpp_s rgb;
    uint8_t raw[RGB_COLOR_COMPONENT_COUNT];
} rgbColor24bpp_t;

#define HSV_HUE_MAX 359U
#define HSV_SATURATION_MAX 255U
#define HSV_VALUE_MAX 255U

typedef enum
{
    HSV_HUE = 0,
    HSV_SATURATION,
    HSV_VALUE
} hsvColorComponent_e;

#define HSV_COLOR_COMPONENT_COUNT (HSV_VALUE + 1)

typedef struct hsvColor_s
{
    uint16_t h;                                                                 // 0 - 359
    uint8_t s;                                                                  // 0 - 255
    uint8_t v;                                                                  // 0 - 255
} hsvColor_t;
/* -------------------- global variables ------------------------------------ */

/* ================== canberra UAV port from Matthew Ridley ================= */
static const int16_t rgb_yuv_mat[3u][3u] =
{
  { 66, 129,  25},
  {-38, -74, 112},
  {112, -94, -18}
};

static const int16_t yuv_shift[3u] =
{
  16, 128, 128
};

int16_t pix_x[4u] = { 0, 1, 0, 1};
int16_t pix_y[4u] = { 0, 0, 1, 1};

CUAV_bayer_t bayertab[28u] =
{
  {0, 0, GREEN, 0, 0},                                                          // the zero offset pixels
  {1, 0, BLUE,  0, 0},
  {0, 1, RED,   0, 0},
  {1, 1, GREEN, 0, 0},

  {0, 0, RED,   0,-1},                                                          // top left
  {0, 0, RED,   0, 1},
  {0, 0, BLUE, -1, 0},
  {0, 0, BLUE,  1, 0},

  {1, 0, RED,   1, 1},                                                          // top right
  {1, 0, RED,  -1, 1},
  {1, 0, RED,  -1, 1},
  {1, 0, RED,   1,-1},
  {1, 0, GREEN, 1, 0},
  {1, 0, GREEN,-1, 0},
  {1, 0, GREEN, 0, 1},
  {1, 0, GREEN, 0,-1},

  {0, 1, BLUE,  1, 1},                                                          // bottom left
  {0, 1, BLUE, -1, 1},
  {0, 1, BLUE, -1, 1},
  {0, 1, BLUE,  1,-1},
  {0, 1, GREEN, 1, 0},
  {0, 1, GREEN,-1, 0},
  {0, 1, GREEN, 0, 1},
  {0, 1, GREEN, 0,-1},

  {1, 1, RED, -1,  0},                                                          // bottom right
  {1, 1, RED,  1,  0},
  {1, 1, BLUE, 0, -1},
  {1, 1, BLUE, 0,  1}
};
//
// Function defintions
//
/* ========== format codecs and re-sizers =================================== */
rgbColor24bpp_t* hsvToRgb24(const hsvColor_t *c);
void RGBtoHSV(float32_t fR, float32_t fG, float32_t fB, float32_t fH, float32_t fS, float32_t fV);
void RGB2HSI(const uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols);
void HSI2RGB(const uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols);
void convert_rgb_yuv( float32_t R, float32_t G, float32_t B, uint16_t *Y, uint16_t *U, uint16_t *V);
void convert_xyz_lab( float32_t *X, float32_t *Y, float32_t *Z, uint16_t *L, uint16_t *A, uint16_t *B);
void convert_cmyk_rgb( uint16_t C, uint16_t M, uint16_t Y, uint16_t K, uint16_t *R, uint16_t *G, uint16_t *B );
void convert_rgb_cmyk( uint16_t R, uint16_t G, uint16_t B, uint16_t *C, uint16_t *M, uint16_t *Y, uint16_t *K);
void NV12ToRGB(uint8_t * rgbBuffer, uint8_t * yuvBuffer, int16_t width, int16_t height);
int8_t convert_YUV_420_422( uint8_t *Cin, uint8_t *Cout, uint16_t Clen);
void NV12_YUV420P(const unsigned char* image_src, unsigned char* image_dst, int16_t image_width, int16_t image_height);
uint16_t GetSmallestValue(uint16_t x, uint16_t y, uint16_t z );
float32_t sRGBtoLinearRGB( float32_t color);
void labToLch( float32_t a, float32_t b, uint16_t *c, uint16_t *h );
uint16_t abToHue(float32_t a, float32_t b);
COLOUR_t HSL2RGB(const HSL_t c1);
HSL_t RGB2HSL(const COLOUR_t c1);
void resize_uyuv(img_struct_t* input, img_struct_t* output, int16_t downsample);
void rainbow(uint8_t n, uint8_t *r, uint8_t *g, uint8_t *b);
void Raspicam_LensShader_internal( const LensShaderColor_t *R, const LensShaderColor_t *G, const LensShaderColor_t *B, uint8_t grid[52u * 39u * 4u] );
void BGR2RGB( const uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols);
// ROS point cloud event : To complete ROS interface
PointXYZRGB_t * EventVisualizer_PointCloud( PointCloud_t *cloud ); 
float64_t EventVisualizer_PointCloud_FindFarPointOppDir( VectrPointCloud_t *inlierCloud );
int8_t EventVisualizer_PointCloud_FindLargestInd( VectrPointCloud_t *inlierCloud );
float64_t EventVisualizer_PointCloud_FindFarPoint( VectrPointCloud_t *inlierCloud );
void Pointcloud_writeGeoJSON(char *outfile, const VectrPointCloud_t *points);
/* ====== octomap octrees point cloud ===================== */
bool Pointcloud_crop(float64_t *lowerBound[3u], float64_t *upperBound[3u], const VectrPointCloud_t point);
bool Pointcloud_checkDist(float64_t thres, const VectrPointCloud_t point, uint8_t operation );
void Pointcloud_calcBBX(float64_t *lowerBound[3u], float64_t *upperBound[3u], const VectrPointCloud_t *points );
void Pointcloud_writeVrml(char *outfile, const VectrPointCloud_t *points);
/* ======== picture analysis e.g. Agriculture ========================== */
int16_t rgbToGray(uint8_t *rgb, uint8_t **gray, int16_t buffer_size);
void thresholdOtsu2( uint8_t *aStar, uint8_t clusterNo, uint8_t *calcLevel );
void thresholdOtsu( uint8_t aStar[], uint8_t clusterNo, uint8_t *calcLevel[] );
float64_t otsu(int16_t minVal,int16_t maxVal,int16_t ht,int16_t wth);
int16_t histogram_otsu(int16_t ht,int16_t wth,int16_t **dataV);
void itConv(uint8_t *buffer, int16_t buffer_size, int16_t width, int16_t *op, uint8_t **res);
int16_t convolution(uint8_t *X, int16_t *Y, int16_t c_size);
void makeOpMem(uint8_t *buffer, int16_t buffer_size, int16_t width, int16_t cindex, uint8_t *op_mem);
int16_t sobelFilter(uint8_t *rgb, uint8_t **gray, uint8_t **sobel_h_res, uint8_t **sobel_v_res, uint8_t **contour_img, int16_t width, int16_t height);
void contour(uint8_t *sobel_h, uint8_t *sobel_v, int16_t gray_size, uint8_t **contour_img);
void sobel_filtering( );
uint8_t *houghtransform(uint8_t *d, int16_t *widthVal, int16_t *hightVal, int16_t *surfceVal, int16_t bpp);
void Prewitt_Filter(bitmap_t * bitmap, int16_t swidth, int16_t ewidth);
void binary_conversion(int16_t t,int16_t ht,int16_t wth,int16_t maxval, char *filename, int16_t **dataV);
void gaussian_filter(const pixel_t *inV, pixel_t *outV, int16_t nx, int16_t ny, float32_t sigma);
void canny_conv(const pixel_t *in, pixel_t *out, const float32_t *kernel, const int16_t nx, const int16_t ny, const int16_t kn, const uint8_t normalize);
pixel_t *canny_edge_detection(const pixel_t *in, const dib_header_t *bmp_ih, const int16_t tmin, const int16_t tmax, const float32_t sigma);
void doIttiKochNiebur( hue_t *obj );
void kirsch( uint8_t *ps, uint8_t *pd, const int16_t W, const int16_t H, const int16_t step );
void ParLinTran( uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols, const int16_t x1, const int16_t x2, const float64_t y1, const float64_t y2 );
void Threshold( uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols, const int16_t nThres );
void GammaTran( uint8_t *srcimg, float64_t *outimg, const int16_t rows, const int16_t cols, float64_t gamma, float64_t comp );
void LogTran( uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols, float64_t dC );
void LinTran( uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols, float64_t dFa, float64_t dFb );
int16_t AutoThresholder_IsoData( int16_t *dataVec );
int16_t AutoThresholder_Li( int16_t *dataVectr );
float32_t PolygonUtils_getImageOffset( const float32_t xin, const float32_t  yin, const int16_t width, const int16_t height );
float32_t PolygonUtils_area( float32_t **polygon );
f32point_t PolygonUtils_centroid( float32_t **polygon );
char* utils_getAverageColor( const float32_t *c, const mat44 * p, const int16_t width, const int16_t height, const uint8_t *imageBuffer8, const colOptions_t opt, const float32_t threshold, voxel_rgba_color_t *avg);
char* utils_getColorAtPos( float32_t *pt, int16_t width, int16_t height, const uint8_t *imageBuffer8, const colOptions_t opt, const float32_t threshold, voxel_rgba_color_t *avg );
char* utils_getColor_centroid( float32_t **polygon, int16_t width, int16_t height, const uint8_t *imageBuffer8, const colOptions_t opt, const float32_t threshold, voxel_rgba_color_t *avg );
char* utils_getColor( float32_t **polygon, const mat44 * p, const int16_t width, const int16_t height, const uint8_t *imageBuffer8, const colOptions_t opt, const float32_t threshold, voxel_rgba_color_t *avg );
char* utils_getDotColor( const float32_t *pt, const float32_t r, const int16_t width, const int16_t height, const uint8_t *imageBuffer8, const colOptions_t opt, const float32_t threshold, voxel_rgba_color_t *avg );
bool delaunay_point_isEq( const f32point_t n, const f32point_t p);
bool delaunay_edge_isEq( const f32edge_t e, const f32edge_t edge);
f32edge_t delaunay_new_edge( const f32point_t p0, const f32point_t p1 );
f32triangle_t delaunay_new_triangle( const f32point_t p0, const f32point_t p1, const f32point_t p2 );
void delaunay_clear( f32delaunay_t *d);
/* ========= Spline interpolation. =========== */
static float64_t initcausal4(float64_t* c, int16_t step, int16_t n, float64_t z);
static float64_t initanticausal4(float64_t* c, int16_t step, int16_t n, float64_t z);
static float64_t initcausal(float64_t *c, int16_t n, float64_t z);
static float64_t initanticausal(float64_t *c, int16_t n, float64_t z);
static void invspline1D5(float64_t* c, int16_t step, int16_t size, float64_t* z, int16_t npoles);
static void invspline1D(float64_t *c, int16_t size, float64_t *z, int16_t npoles);
static uint8_t fill_poles(float64_t* z, int16_t order);
static uint8_t prepare_spline(image_double_t *im, int16_t order);
static void keysf(float32_t *c,float32_t t,float32_t a);
static void keysd(float64_t* c, float64_t t, float64_t a);
static void spline3f(float32_t *c,float32_t t);
static void spline3d(float64_t* c, float64_t t);
static void init_splinenf(float32_t *a,int16_t n);
static void init_splinend(float64_t* a, int16_t n);
static float32_t ipow(float32_t x, int16_t n);
static void splinenf(float32_t *c,float32_t t,float32_t *a,int16_t n);
static void splinend(float64_t *c, float64_t t, float64_t *a, int16_t n);
static uint8_t valid_image_double(image_double_t *in, int16_t x, int16_t y);
uint8_t interpolate_spline( image_double_t *im, int16_t order, float64_t x, float64_t y, float64_t *out, float64_t paramKeys);
void finvspline(float32_t *in, int16_t order, float32_t *out, int16_t width, int16_t height);
/* ======== pixsel analysis ====================== */
uint8_t getluminosity( const uint8_t r,const  uint8_t g,const  uint8_t b );
image_double_t * average_image(image_double_t *img);
int16_t white_neighbors(picsel_p_t *p, image_double_t *img);
float32_t * pixel2cam(const Cam2pixel_t pixel_coord, const float32_t f[2u],const float32_t c[2u]);
Cam2pixel_t * cam2pixel(const float32_t cam_coord[3u],const float32_t f[2u],const float32_t c[2u]);
float64_t OptimizerGlobal_get_event_score(int16_t x, int16_t y, float32_t *project_img[], float32_t metric_wsize);
void to_SingleChannel(uint8_t *src,uint8_t *dst, int16_t src_rows, int16_t src_cols);
void CalWeightingTable(float32_t *WeightTable, gradientMap_t *HGradientMap, gradientMap_t *VGradientMap);
uint8_t * interpolateRBonGpixelLocation( const uint8_t *Dst, int16_t width, int16_t height);
uint8_t * interpolateRBatBRpixelLocation( const uint8_t *Dst, const gradientMap_t *TPdiff, const float32_t *Prb );
uint8_t * interpolateGVatBRpixelLocation( const float32_t *WeightTable, const uint8_t *Dst, const gradientMap_t *TPdiff, const gradientMap_t *HCDMap, const gradientMap_t *VCDMap);
float64_t Image_distance(float64_t *img1, float64_t *img2, float32_t *weight, size_t sz);
/* ======= PX4 Optical Flow ====================== */
uint32_t PX4Flow__USAD8(uint32_t val1, uint32_t val2);
uint32_t PX4Flow__USADA8(uint32_t val1, uint32_t val2, uint32_t val3);
uint32_t PX4Flow__UHADD8(uint32_t val1, uint32_t val2);
uint32_t PX4Flow_compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size);
uint32_t PX4Flow_compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size);
uint32_t PX4Flow_compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint16_t row_size);
uint8_t PX4Flow_compute_flow(uint8_t *image1, uint8_t *image2, float32_t x_rate, float32_t y_rate, float32_t z_rate, float32_t *pixel_flow_x, float32_t *pixel_flow_y, uint32_t search_size, uint32_t image_width, uint32_t flow_feature_threshold, uint32_t flow_value_threshold);
/* ====== Machine Vision Tool Kit ================= */
void Optical_locateEyes( const float32_t pl[2u][4u], const float32_t pr[2u][4u], float32_t k, float32_t a, float32_t b, Cam_Eye_t *eye);
void Optical_findEyes( const float32_t pl[2u][4u], const float32_t pr[2u][4u], float32_t a, float32_t b, Cam_Eye_t *eye);
uint8_t * makeMedianFromByteStream( uint8_t *sptr, int16_t w, int16_t h, uint8_t dev );
float32_t * makeMedianFromFloatStream( float32_t *sptr, int16_t w, int16_t h, float32_t dev );
/* ====== canberra UAV =========================== */
void CUAV_update_stats(uint16_t x, image_stats_t* s);
void debayer_half_16u_8u_rgb(uint16_t* in_image, size_t in_stride, size_t in_width, size_t in_height, uint8_t* out_image, size_t out_stride);
void pixop_half_16u_8u_rgb(const uint16_t* in, size_t in_stride, uint8_t* out);
void rgb_to_yuv_16u_8u(const uint16_t* rgb, uint8_t* yuv);
void pixop_half_16u_8u_yuv(const uint16_t* in, size_t in_stride, uint8_t* out);
void debayer_half_16u_8u(uint16_t* in_image, size_t in_stride, size_t in_width, size_t in_height, uint8_t* out_image, size_t out_stride,  CUAV_pixop_e pixop);
void pixop_2x2_16u_8u_rgb(const uint16_t* in, size_t in_stride, uint8_t* out, size_t out_stride);
void pixop_2x2_16u_8u_yuv(const uint16_t* in, size_t in_stride, uint8_t* out, size_t out_stride);
void debayer_full_16u_8u(uint16_t* in_image, size_t in_stride, size_t in_width, size_t in_height, uint8_t* out_image, size_t out_stride, CUAV_pixop_e pixop);
/* ====== picture to co-ordinate ================ */
image_coord_t image2Coord( float64_t i, float64_t j, float64_t elevation, float64_t altitude, float64_t azimuth, float64_t fov, uint32_t imgWdth, uint32_t imgHt);
/* ====== mirror the image ====================== */
void VertMirror(uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols);
void HorMirror(uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols);
void filter_interpolate_colors(int16_t *c1, int16_t *c2, int16_t *c3, float64_t *res, float64_t t);
/* ====== voxel translations ==================== */
#if defined(REQUIRE_VOXEL_CODEC)
void TsdfVoxel_deserializeFromIntegers( uint32_t *dataV, TsdfVoxel_t **voxel );
void OccupancyVoxel_deserializeFromIntegers( uint32_t *dataV, OccupancyVoxel_t **voxel );
int8_t Voxel_deserializeDirection( const uint32_t dataV, Voxel_Vectr_t *parent_direction );
void EsdfVoxel_deserializeFromIntegers( uint32_t *dataV, EsdfVoxel_t **voxel );
void IntensityVoxel_deserializeFromIntegers( uint32_t *dataV, BlockIntensityVoxel_t **voxel );
void TsdfVoxel_serializeToIntegers( uint32_t *dataV, TsdfVoxel_t **voxel );
void OccupancyVoxel_serializeToIntegers( uint32_t *dataV, OccupancyVoxel_t **voxel );
int8_t Voxel_serializeDirection( const Voxel_Vectr_t *parent_direction, uint32_t *dataV );
void EsdfVoxel_serializeToIntegers( uint32_t *dataV, EsdfVoxel_t **voxel );
void IntensityVoxel_serializeToIntegers( uint32_t *dataV, BlockIntensityVoxel_t **voxel );
#endif
// need format converter for ------------ L* a* b* color space --------------
// this method uses the a* from the L* a* b* generated from RGB
//
/*-----------------------------------------------------------------------------
 *      thresholdOtsu2:  otsu threshold algorythm
 *      
 *
 *  Parameters: uint8_t *aStar, uint8_t clusterNo, uint8_t *calcLevel
 *  Return:   void
 *----------------------------------------------------------------------------*/
void thresholdOtsu2( uint8_t *aStar, uint8_t clusterNo, uint8_t *calcLevel )
{
    uint8_t iterate;
    uint8_t prev_aStar[2u];
    
    for (iterate=0u; iterate<clusterNo; iterate++ )
    {
       if (iterate >= 2u)                                                       /* can calculate once we have at least 3 samples */
       {
          *calcLevel = ((max(prev_aStar[0u],prev_aStar[1u]) - min(*aStar,prev_aStar[1u])) /  2U);
          calcLevel++;
       }
       if (iterate==0u)
       {
          prev_aStar[1u] = *aStar;
       }
       else
       {
          prev_aStar[0u] = prev_aStar[1u];
          prev_aStar[1u] = *aStar;
       }
       aStar++;
    }
}
/*-----------------------------------------------------------------------------
 *      thresholdOtsu:  otsu threshold algorythm
 *      
 *
 *  Parameters: uint8_t aStar[], uint8_t clusterNo, uint8_t *calcLevel[]
 *  Return:   void
 *----------------------------------------------------------------------------*/
void thresholdOtsu( uint8_t aStar[], uint8_t clusterNo, uint8_t *calcLevel[] )
{
    uint8_t iterate;

    for (iterate=0u; iterate<clusterNo; iterate++ )
    {
       if (iterate >= 2u)
       {
          *calcLevel[iterate-2u] = ((max(aStar[iterate-2u],aStar[iterate-1u]) - min(aStar[iterate],aStar[iterate-1u])) /  2U);
       }
    }
}

/*-----------------------------------------------------------------------------
 *  getluminosity:  calculates a luminosity from RGB pixsel value
 *      
 *
 *  Parameters: uint8_t r, uint8_t g, uint8_t b
 *  Return: uint8_t 
 *----------------------------------------------------------------------------*/
uint8_t getluminosity( const uint8_t r, const uint8_t g, const uint8_t b )
{
    return((r+g+b)/3u);
}
/* 
As a fourth step, over the binary image the following edge detection algorithms were applied, 
Sobel, Canny, Prewitt, Roberts, Zero Cross and Log[4]. 
As a 5fth step, Hough Transform was performed over the edge’s images generated in the previous step. 
Finally the best edges detection method was selected by visual inspection (the higher the number 
of crops row correctly identifed, the better the method is). That is, which method detect a 
higher number of crop rows correctly
*/
//
// Source below found here: http://www.kasperkamperman.com/blog/arduino/arduino-programming-hsb-to-rgb/
//
/*-----------------------------------------------------------------------------
 *  hsvToRgb24:  convert HSV to RGB24
 *      
 *
 *  Parameters: const hsvColor_t* c
 *  Return:   rgbColor24bpp_t*
 *----------------------------------------------------------------------------*/
rgbColor24bpp_t* hsvToRgb24(const hsvColor_t* c)
{

    static rgbColor24bpp_t r;
    uint16_t val = c->v;
    uint16_t sat = 255 - c->s;
    uint32_t base;
    uint16_t hue = c->h;

    if (sat == 0)
    {                                                                           // Acromatic color (gray). Hue doesn't mind.

        r.rgb.r = val;
        r.rgb.g = val;
        r.rgb.b = val;
    }
    else
    {
        base = ((255U - sat) * val) >> 8;
        switch (hue / 60U)
        {

            case 0:
            r.rgb.r = val;
            r.rgb.g = (((val - base) * hue) / 60U) + base;
            r.rgb.b = base;
            break;

            case 1:
            r.rgb.r = (((val - base) * (60U - (hue % 60U))) / 60U) + base;
            r.rgb.g = val;
            r.rgb.b = base;
            break;

            case 2:
            r.rgb.r = base;
            r.rgb.g = val;
            r.rgb.b = (((val - base) * (hue % 60U)) / 60U) + base;
            break;

            case 3:
            r.rgb.r = base;
            r.rgb.g = (((val - base) * (60U - (hue % 60U))) / 60U) + base;
            r.rgb.b = val;
            break;

            case 4:
            r.rgb.r = (((val - base) * (hue % 60U)) / 60U) + base;
            r.rgb.g = base;
            r.rgb.b = val;
            break;

            case 5:
            r.rgb.r = val;
            r.rgb.g = base;
            r.rgb.b = (((val - base) * (60U - (hue % 60U))) / 60U) + base;
            break;

        }
    }
    return &r;

}
//
// Jan Winkler <winkler@cs.uni-bremen.de>
//
/*-----------------------------------------------------------------------------
 *      RGBtoHSV:  convert RGB to HSV 
 *      
 *
 *  Parameters: float32_t fR, float32_t fG, float32_t fB, float32_t fH, float32_t fS, float32_t fV
 *  Return:   void
 *----------------------------------------------------------------------------*/
void RGBtoHSV(float32_t fR, float32_t fG, float32_t fB, float32_t fH, float32_t fS, float32_t fV)
{

  float32_t fCMax = max(max(fR, fG), fB);
  float32_t fCMin = min(min(fR, fG), fB);
  float32_t fDelta = fCMax - fCMin;

  if(fDelta > 0.0f)
  {
    if(fCMax == fR)
    {
      fH = 60f * (FMOD(((fG - fB) / fDelta), 6f));
    }
    else if(fCMax == fG)
    {
      fH = 60f * (((fB - fR) / fDelta) + 2f);
    }
    else if(fCMax == fB)
    {
      fH = 60f * (((fR - fG) / fDelta) + 4f);
    }

    if(fCMax > 0f)
    {
      fS = fDelta / fCMax;
    }
    else
    {
      fS = 0f;
    }
    fV = fCMax;
  }
  else
  {
    fH = 0f;
    fS = 0f;
    fV = fCMax;
  }
  if(fH < 0f)
  {
    fH = 360f + fH;
  }
}
 
/*-----------------------------------------------------------------------------
 *      convert_rgb_yuv:  convert RGB to YUV 
 *      
 *
 *  Parameters: float32_t R, float32_t G, float32_t B, uint16_t *Y, uint16_t *U, uint16_t *V
 *  Return:   void
 *----------------------------------------------------------------------------*/
void convert_rgb_yuv( float32_t R, float32_t G, float32_t B, uint16_t *Y, uint16_t *U, uint16_t *V)
{
   *Y = ROUND(( (0.256788f * R) + (0.504129f * G) + (0.097906f * B)),0U) +  16f;
   *U = ROUND(((-0.148223f * R) - (0.290993f * G) + (0.439216f * B)),0U) + 128f;
   *V = ROUND(((0.439216f * R) - (0.367788f * G) - (0.071427f * B)),0U) + 128f;
}
 
/*-----------------------------------------------------------------------------
 *      convert_xyz_lab:  CIE 1931 XYZ colors to CIE L*a*b* 
 *      
 *
 *  Parameters: float32_t *X, float32_t *Y, float32_t *Z, uint16_t *L, uint16_t *A, uint16_t *B
 *  Return:   void
 *----------------------------------------------------------------------------*/
void convert_xyz_lab( float32_t *X, float32_t *Y, float32_t *Z, uint16_t *L, uint16_t *A, uint16_t *B)
{
    *X = *X / 95.047f;
    *Y = *Y / 100.0f;
    *Z = *Z / 108.883f;

    if ( *X > 0.008856f) 
    {
       *X = pow(*X, (1.0f / 3.0f));
    }
    else
    {
       *X = ((*X * 7.787f) + 16.0f) / 116.0f;
    }
    if ( *Y > 0.008856f)
    {
       *Y = pow(*Y, (1.0f / 3.0f));
    }
    else
    {
       *Y = ((*Y * 7.787f) + 16.0f) / 116.0f;
    }
    if ( *Z > 0.008856f)
    {
       *Z = pow(*Z, (1.0f / 3.0f));
    }
    else
    {
       *Z = ((*Z * 7.787f) + 16.0f) / 116.0f;
    }
   *L = ROUND((116.0f * (*Y - 16.0f)),0.0f);
   *A = ROUND((500.0f * (*X - *Y)),0.0f);
   *B = ROUND((200.0f * (*Y - *Z)),0.0f);
}
 
/*-----------------------------------------------------------------------------
 *      convert_cmyk_rgb:  CMYK to RGB 
 *      
 *
 *  Parameters: uint16_t C, uint16_t M, uint16_t Y, uint16_t K, uint16_t *R, uint16_t *G, uint16_t *B
 *  Return:   void
 *----------------------------------------------------------------------------*/
void convert_cmyk_rgb( uint16_t C, uint16_t M, uint16_t Y, uint16_t K, uint16_t *R, uint16_t *G, uint16_t *B )
{
  uint16_t R1,G1,B1;
  
  R1 = 255U - C - K;
  G1 = 255U - M - K;
  B1 = 255U - Y - K;

  *R = max(0U, R1);
  *G = max(0U, G1);
  *B = max(0U, B1);
}
/**
 * Converts a and b of Lab color space to Hue of LCH color space.
 * https://stackoverflow.com/questions/53733379/conversion-of-cielab-to-cielchab-not-yielding-correct-result
 * @param  {number} a
 * @param  {number} b
 * @return {number}
 */
/*-----------------------------------------------------------------------------
 *      abToHue:  Converts a and b of Lab color space to Hue of LCH color space. 
 *      
 *
 *  Parameters: float32_t a, float32_t b
 *  Return:   uint16_t
 *----------------------------------------------------------------------------*/
uint16_t abToHue(float32_t a, float32_t b)
{
    uint16_t xBias=0u;
    
    if ((a >= 0.0f) && (b == 0.0f)) {
        return 0u;
    }
    if ((a < 0.0f) && (b == 0.0f)) {
        return 180u;
    }
    if ((a == 0.0f) && (b > 0.0f)) {
        return 90u;
    }
    if ((a == 0.0f) && (b < 0.0f)) {
        return 270u;
    }

    if ((a > 0.0f) && (b > 0.0f)) {
        xBias = 0u;
    } else if (a < 0.0f) {
        xBias = 180u;
    } else if ((a > 0.0f) && (b < 0.0f)) {
        xBias = 360u;
    }
    return ((atan(b / a)) * (180.0f / PI)) + xBias;
}
/**
 * Converts Lab color space to Luminance-Chroma-Hue color space.
 * http://www.brucelindbloom.com/index.html?Eqn_Lab_to_LCH.html
 * @param  {number[]}
 * @return {number[]}
 */
/*-----------------------------------------------------------------------------
 *      labToLch:  Converts Lab color space to Luminance-Chroma-Hue color space. 
 *      
 *
 *  Parameters: float32_t a, float32_t b, uint16_t *c, uint16_t *h
 *  Return:    void
 *----------------------------------------------------------------------------*/
void labToLch( float32_t a, float32_t b, uint16_t *c, uint16_t *h )
{
    *c = ROUND(sqrt((a * a) + (b * b)),0.0f);
    *h = abToHue(a, b);
}

/*-----------------------------------------------------------------------------
 *      GetSmallestValue:  smallest value from three integers 
 *      
 *
 *  Parameters: uint16_t x, uint16_t y, uint16_t z
 *  Return:    uint16_t
 *----------------------------------------------------------------------------*/
uint16_t GetSmallestValue(uint16_t x, uint16_t y, uint16_t z )
{
   if (y < x)
   {
      if (z < y)
        return z;
      else
        return y;
   }
   else if (z < x)
   {
      if (y < z)
        return y;
      else
        return z;
   }
   else
     return x;
}

/*-----------------------------------------------------------------------------
 *      convert_rgb_cmyk:  RGB->CMYK 
 *      
 *
 *  Parameters: uint16_t R, uint16_t G, uint16_t B, uint16_t *C, uint16_t *M, uint16_t *Y, uint16_t *K
 *  Return:    void
 *----------------------------------------------------------------------------*/
void convert_rgb_cmyk( uint16_t R, uint16_t G, uint16_t B, uint16_t *C, uint16_t *M, uint16_t *Y, uint16_t *K)
{
  //uint16_t K;
  *K = GetSmallestValue((255U - R), (255U - G), (255U - B));
  *C = 255U - R - *K;
  *M = 255U - G - *K;
  *Y = 255U - B - *K;
}

/*-----------------------------------------------------------------------------
 *      NV12ToRGB:  NV12 to RGB 
 *      
 *
 *  Parameters: uint8_t * rgbBuffer, uint8_t * yuvBuffer, int16_t width, int16_t height
 *  Return:    void
 *----------------------------------------------------------------------------*/
void NV12ToRGB(uint8_t * rgbBuffer, uint8_t * yuvBuffer, int16_t width, int16_t height)
{
    uint8_t * uvStart;
    uint8_t y[2U] = { 0U, 0U };
    uint8_t u = 0U;
    uint8_t v = 0U;
    int16_t r = 0U;
    int16_t g = 0U;
    int16_t b = 0U;
    int16_t rowCnt = 0U;
    int16_t colCnt = 0U;
    int16_t cnt = 0U;
    
    uvStart = yuvBuffer + width * height;
    for (rowCnt = 0U; rowCnt < height; rowCnt++)
    {
        for (colCnt = 0U; colCnt < width; colCnt += 2U)
        {
            u = *(uvStart + colCnt + 0U);
            v = *(uvStart + colCnt + 1U);
            for (cnt = 0U; cnt < 2U; cnt++)
            {
                y[cnt] = yuvBuffer[rowCnt * width + colCnt + cnt];
                r = CONVERT_YUV_R(y[cnt], v);
                CLIP(r);
                g = CONVERT_YUV_G(y[cnt], u, v);
                CLIP(g);
                b = CONVERT_YUV_B(y[cnt], u);
                CLIP(b);
                rgbBuffer[(rowCnt * width + colCnt + cnt) * 3U + 0U] = (uint8_t)r;
                rgbBuffer[(rowCnt * width + colCnt + cnt) * 3U + 1U] = (uint8_t)g;
                rgbBuffer[(rowCnt * width + colCnt + cnt) * 3U + 2U] = (uint8_t)b;
            }
        }
        uvStart += width * (rowCnt % 2U);
    }
}
 
/*-----------------------------------------------------------------------------
 *      convert_YUV_420_422:  4:2:0 YUV to 4:2:2 YUV 
 *      
 *
 *  Parameters: uint8_t *Cin, uint8_t *Cout, uint16_t Clen
 *  Return:    int8_t
 *----------------------------------------------------------------------------*/
int8_t convert_YUV_420_422( uint8_t *Cin, uint8_t *Cout, uint16_t Clen)
{
   uint16_t i=0U;
   
   if ((Cin==NULL) || (Cout==NULL))
      return -1;
   while (i++<=Clen)
   {
      Cout[2U*i] = Cin[i];
      Cout[2U*i+1] = min(max(((9U * (Cin[i] + Cin[i+1U]) - (Cin[i-1U] + Cin[i+2U]) + 8U) >> 4U),0U),255U);
   }
   return 0;
}

/*-----------------------------------------------------------------------------
 *      NV12_YUV420P:  nv12 to YUV420P 
 *      image_src is the source image, image_dst is the converted image
 *
 *  Parameters: const unsigned char* image_src, unsigned char* image_dst, int16_t image_width, int16_t image_height
 *  Return:    void
 *----------------------------------------------------------------------------*/
void NV12_YUV420P(const unsigned char* image_src, unsigned char* image_dst, int16_t image_width, int16_t image_height)
{
        const unsigned char* pNV;
        unsigned char* pU;
        unsigned char* pV;
        int16_t i;
        
        unsigned char* p = image_dst;
        memcpy((void *) p,(void *) image_src,(int16_t) (image_width * image_height * 3 / 2));
        p[(int16_t) (image_width * image_height * 3 / 2)] = '\0';               /* terminate the copy   */
        pNV = image_src + image_width * image_height;
        pU = p + image_width * image_height;
        pV = p + image_width * image_height + ((image_width * image_height)>>2U);
        for (i=0; i<(image_width * image_height)/2; i++)   
        {
           if ((i%2)==0) *pU++ = *(pNV + i);
           else *pV++ = *(pNV + i);
        }
}

/*
   Calculate HSL from RGB
   Hue is in degrees
   Lightness is between 0 and 1
   Saturation is between 0 and 1
*/
/*-----------------------------------------------------------------------------
 *      RGB2HSL:   rgb to hsl 
 *
 *
 *  Parameters: const COLOUR_t c1
 *  Return:    HSL_t
 *----------------------------------------------------------------------------*/
HSL_t RGB2HSL(const COLOUR_t c1)
{
   float64_t themin,themax,delta;
   HSL_t c2;

   themin = min(c1.r,min(c1.g,c1.b));
   themax = max(c1.r,max(c1.g,c1.b));
   delta = themax - themin;
   c2.l = (themin + themax) / 2.0f;
   c2.s = 0.0f;
   if (c2.l > 0.0f && c2.l < 1.0f)
      c2.s = delta / (c2.l < 0.5f ? (2.0f*c2.l) : (2.0f-2.0f*c2.l));
   c2.h = 0.0f;
   if (delta > 0.0f) {
      if (themax == c1.r && themax != c1.g)
         c2.h += (c1.g - c1.b) / delta;
      if (themax == c1.g && themax != c1.b)
         c2.h += (2.0f + (c1.b - c1.r) / delta);
      if (themax == c1.b && themax != c1.r)
         c2.h += (4.0f + (c1.r - c1.g) / delta);
      c2.h *= 60.0f;
   }
   return(c2);
}
/*
   Calculate RGB from HSL, reverse of RGB2HSL()
   Hue is in degrees
   Lightness is between 0 and 1
   Saturation is between 0 and 1
*/
/*-----------------------------------------------------------------------------
 *      HSL2RGB:  hsl to rgb 
 *
 *
 *  Parameters: HSL_t c1
 *  Return:    COLOUR_t
 *----------------------------------------------------------------------------*/
COLOUR_t HSL2RGB(const HSL_t c1)
{
   COLOUR_t c2,sat,ctmp;

   while (c1.h < 0.0f)
      c1.h += 360.0f;
   while (c1.h > 360.0f)
      c1.h -= 360.0f;

   if (c1.h < 120.0f) {
      sat.r = (120.0f - c1.h) / 60.0f;
      sat.g = c1.h / 60.0f;
      sat.b = 0.0f;
   } else if (c1.h < 240.0f) {
      sat.r = 0.0f;
      sat.g = (240.0f - c1.h) / 60.0f;
      sat.b = (c1.h - 120.0f) / 60.0f;
   } else {
      sat.r = (c1.h - 240.0f) / 60.0f;
      sat.g = 0.0f;
      sat.b = (360.0f - c1.h) / 60.0f;
   }
   sat.r = min(sat.r,1);
   sat.g = min(sat.g,1);
   sat.b = min(sat.b,1);

   ctmp.r = 2.0f * c1.s * sat.r + (1.0f - c1.s);
   ctmp.g = 2.0f * c1.s * sat.g + (1.0f - c1.s);
   ctmp.b = 2.0f * c1.s * sat.b + (1.0f - c1.s);

   if (c1.l < 0.5f) {
      c2.r = c1.l * ctmp.r;
      c2.g = c1.l * ctmp.g;
      c2.b = c1.l * ctmp.b;
   } else {
      c2.r = (1.0f - c1.l) * ctmp.r + 2.0f * c1.l - 1.0f;
      c2.g = (1.0f - c1.l) * ctmp.g + 2.0f * c1.l - 1.0f;
      c2.b = (1.0f - c1.l) * ctmp.b + 2.0f * c1.l - 1.0f;
   }

   return(c2);
}

/*-----------------------------------------------------------------------------
 *      RGB2HSI:  convert a RGB byte stream to byte stream as HSI format
 *
 *  Parameters: const uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void RGB2HSI(const uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols)
{ 
    /* p.243
    if(type2str(img.type()) != "8UC3") return false; */

    int16_t row,col;
    float64_t B,G,R,minColor,maxColor,H,S,I,cosVal,radiance;
        
    for(row = 0; row < rows; row++)
    {
        for(col = 0; col < cols; col=+3)                                        /* here we are using a byte stream of 3 bytes per RGB element */
        {
            /* cv::Vec3b pixel = img.at<cv::Vec3b>(row, col); */
            B = srcimg[0+row+col]/255.0f;
            G = srcimg[1+row+col]/255.0f;
            R = srcimg[2+row+col]/255.0f;

            minColor = min(B, G);
            minColor = min(minColor, R);
            maxColor = max(B,G);
            maxColor = max(maxColor, R);

            I = (R+G+B)/3.0f;                    //[0,1]

            // given in textbook??
            if(I < 0.078431f)
            {
                 S = 0.0f;
            }
            else if(I > 0.92f)
            {
                S = 0.0f;
            }
            else
            {
                 S = 1.0f - 3.0f*minColor/(R+G+B);
            }

            if(minColor == maxColor)
            {
                H = 0.0f;
                S = 0.0f;
            }
            else
            {
                //its unit is radiance
                cosVal = (R-G+R-B)/2.0f / sqrt((R-G)*(R-G) + (R-B)*(G-B) + DBL_EPSILON);
                radiance = (abs(cosVal-1) < DBL_EPSILON) ? 0.0f : (abs(cosVal-(-1.0f)) < DBL_EPSILON) ? PI : acos(cosVal);
                // double radiance = acos(radiance);
                H = (B <= G) ? radiance : 2.0f*PI - radiance; //[0,2.0*M_PI]
                S = 1.0f - 3.0f*minColor/(R+G+B + DBL_EPSILON); //[0,1]
            }

            //normalize
            H = H/(2.0f*PI) * 255.0f;
            S = S * 255.0f;
            I = I * 255.0f;

            //put H in R channel, S in G channel, I in B channel
            /*img.at<cv::Vec3b>(row, col) = cv::Vec3b((int)I, (int)S, (int)H);*/
            outimg[0+row+col]=(uint8_t)I;
            outimg[1+row+col]=(uint8_t)S;
            outimg[2+row+col]=(uint8_t)H;
        }
    }
}
/*-----------------------------------------------------------------------------
 *      HSI2RGB:  convert a HSI byte stream to byte stream in RGB format
 *
 *  Parameters: const uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void HSI2RGB( const uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols)
{ 
    /* p.247
    if(type2str(img.type()) != "8UC3") return false; */
    int16_t row,col;
    float64_t B,G,R,minColor,maxColor,H,S,I,cosVal,radiance;
        
    for(row = 0; row < rows; row++)
    {
        for(col = 0; col < cols; col=+3)
        {
            /* cv::Vec3b pixel = img.at<cv::Vec3b>(row, col); */
            I = ((float64_t) srcimg[0+row+col])/255.0f;                         //normalize to [0,1]
            S = ((float64_t)srcimg[1+row+col])/255.0f;                          //normalize to [0,1]
            H = (((float64_t)srcimg[2+row+col])/255.0f)*2.0f*PI;                //normalize to [0,2*PI]

            if(H >= 0.0f && H < PI*2.0f/3.0f)
            { 
                B = I * (1.0f - S);                                             //0 to 120 degrees RG sector
                R = I * (1.0f + S*cos(H)/cos(PI/3.0f - H));
                G = 3.0f*I - (R+B);
            }
            else if(H >= PI*2.0f/3.0f && H < PI*4.0f/3.0f)
            {                                                      
                H = H - PI*2.0f/3.0f;                                           //120 to 240 degrees GB sector
                R = I * (1.0f - S);
                G = I * (1.0f + S*cos(H)/cos(PI/3.0f - H));
                B = 3.0f*I - (R+G);
            }
            else if(H >= PI*4.0f/3.0f)
            { 
                H = H - PI*4.0f/3.0f;                                           //larger than 240 degrees BR sector
                G = I * (1.0f - S);
                B = I * (1.0f + S*cos(H)/cos(PI/3.0f - H));
                R = 3.0f*I - (G+B);
            }

            R *= 255.0f;
            G *= 255.0f;
            B *= 255.0f;

            /* img.at<cv::Vec3b>(row, col) = cv::Vec3b((int)B, (int)G, (int)R); */
            outimg[0+row+col]=(uint8_t)B;
            outimg[1+row+col]=(uint8_t)G;
            outimg[2+row+col]=(uint8_t)R;                        
        }
    }

}
/*-----------------------------------------------------------------------------
 *      BGR2RGB:  convert a BGR byte stream to byte stream in RGB format
 *
 *  Parameters: const uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void BGR2RGB( const uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols)
{ 
    int16_t row,col;

    for(row = 0; row < rows; row++)
    {
        for(col = 0; col < cols; col=+3)
        {
            outimg[0+row+col]=srcimg[2+row+col];
            outimg[1+row+col]=srcimg[1+row+col];
            outimg[2+row+col]=srcimg[0+row+col];                        
        }
    }

}
/*-----------------------------------------------------------------------------
 *      rgbToGray:  rgb to greyscale
 *
 *
 *  Parameters: uint8_t *rgb, uint8_t **gray, int16_t buffer_size
 *  Return:    int16_t
 *----------------------------------------------------------------------------*/
int16_t rgbToGray(uint8_t *rgb, uint8_t **gray, int16_t buffer_size)
{
   int16_t gray_size = buffer_size / 3u;                                        /* Take size for gray image and allocate memory */
   uint8_t *p_rgb = rgb;                                                        /* byte pointer for rgb pixels */
   uint8_t *p_gray = *gray;                                                     /* byte pointer for gray pixels */
   int16_t i;
   *gray = Malloc((uint32_t)(sizeof(uint8_t) * gray_size));

   for(i=0; i<gray_size; i++)
   {
       *p_gray = 0.30f*p_rgb[0] + 0.59f*p_rgb[1] + 0.11f*p_rgb[2u];             /* convert each pixel to grey scale */
       p_rgb += 3u;
       p_gray++;
   }
   Free((char*)gray,sizeof(gray));                                                     /* free the memory allocated for the gray pointer */
   return gray_size;
}

/*-----------------------------------------------------------------------------
 *      makeOpMem:  Make the operation memory for iterative convolution
 *
 *
 *  Parameters: uint8_t *buffer, int16_t buffer_size, int16_t width, int16_t cindex, uint8_t *op_mem
 *  Return:    void
 *----------------------------------------------------------------------------*/
void makeOpMem(uint8_t *buffer, int16_t buffer_size, int16_t width, int16_t cindex, uint8_t *op_mem) 
{
    int16_t bottom = cindex-width < 0;
    int16_t top = cindex+width >= buffer_size;
    int16_t left = cindex % width == 0;
    int16_t right = (cindex+1) % width == 0;

    op_mem[0u] = !bottom && !left  ? buffer[cindex-width-1u] : 0u;
    op_mem[1u] = !bottom           ? buffer[cindex-width]   : 0u;
    op_mem[2u] = !bottom && !right ? buffer[cindex-width+1u] : 0u;
    op_mem[3u] = !left             ? buffer[cindex-1]       : 0u;
    op_mem[4u] = buffer[cindex];
    op_mem[5u] = !right            ? buffer[cindex+1]       : 0u;
    op_mem[6u] = !top && !left     ? buffer[cindex+width-1u] : 0u;
    op_mem[7u] = !top              ? buffer[cindex+width]   : 0u;
    op_mem[8u] = !top && !right    ? buffer[cindex+width+1u] : 0u;
}

/*-----------------------------------------------------------------------------
 *      convolution:  Performs convolution between first two arguments
 *
 *
 *  Parameters: uint8_t *X, int16_t *Y, int16_t c_size
 *  Return:    int16_t
 *----------------------------------------------------------------------------*/
int16_t convolution(uint8_t *X, int16_t *Y, int16_t c_size)
{
    int16_t sum = 0;
    int16_t loop;
    
    for(loop=0; loop<c_size; loop++)
    {
        sum += X[loop] * Y[c_size-loop-1];
    }
    return sum;
}
#define SOBEL_OP_SIZE 64u                                                       /* ------- need to find this from library */
/*-----------------------------------------------------------------------------
 *      itConv:  iterative convolution
 *
 *
 *  Parameters: uint8_t *buffer, int16_t buffer_size, int16_t width, int16_t *op, uint8_t **res
 *  Return:    void
 *----------------------------------------------------------------------------*/
void itConv(uint8_t *buffer, int16_t buffer_size, int16_t width, int16_t *op, uint8_t **res)
{
    int16_t i;
    uint8_t op_mem[SOBEL_OP_SIZE];                                              // Temporary memory for each pixel operation
    *res = Malloc(sizeof(uint8_t) * buffer_size);                               // Allocate memory for result

    memset(op_mem, 0, SOBEL_OP_SIZE);
    for(i=0; i<buffer_size; i++)                                                // Make convolution for every pixel
    {
        makeOpMem(buffer, buffer_size, width, i, op_mem);                       // Make op_mem
        (*res)[i] = (uint8_t) abs(convolution(op_mem, op, SOBEL_OP_SIZE));         // Convolution
        /*
         * The abs function is used in here to avoid storing negative numbers
         * in a byte data type array. It wouldn't make a different if the negative
         * value was to be stored because the next time it is used the value is
         * squared.
         */
    }
    Free((char*)res,sizeof(res));                                               /* free the memory allocated */
}

/*-----------------------------------------------------------------------------
 *      contour:  Contour
 *
 *
 *  Parameters: uint8_t *sobel_h, uint8_t *sobel_v, int16_t gray_size, uint8_t **contour_img
 *  Return:    void
 *----------------------------------------------------------------------------*/
void contour(uint8_t *sobel_h, uint8_t *sobel_v, int16_t gray_size, uint8_t **contour_img) 
{
    int16_t i;
    *contour_img = Malloc(sizeof(uint8_t) *gray_size);                          /* Allocate memory for contour_img */

    for(i=0; i<gray_size; i++)                                                  /* Iterate through every pixel to calculate the contour image */
    {
        (*contour_img)[i] = (uint8_t) sqrt(pow((float64_t)sobel_h[i], 2.0f) + pow((float64_t)sobel_v[i], 2.0f));
    }
    Free((char*)*contour_img,sizeof(*contour_img));                             /* free the memory allocated */
}

/*-----------------------------------------------------------------------------
 *      sobelFilter:  Sobel Filter
 *
 *
 *  Parameters: uint8_t *rgb, uint8_t **gray, uint8_t **sobel_h_res, uint8_t **sobel_v_res, uint8_t **contour_img, int16_t width, int16_t height
 *  Return:    int16_t
 *----------------------------------------------------------------------------*/
int16_t sobelFilter(uint8_t *rgb, uint8_t **gray, uint8_t **sobel_h_res, uint8_t **sobel_v_res, uint8_t **contour_img, int16_t width, int16_t height) 
{

    int16_t sobel_h[] = {-1, 0, 1, -2, 0, 2, -1, 0, 1},
        sobel_v[] = {1, 2, 1, 0, 0, 0, -1, -2, -1};
    int16_t rgb_size = width*height*3;
    int16_t gray_size = rgbToGray(rgb, gray, rgb_size);                         /* Get gray representation of the image */
    itConv(*gray, gray_size, width, sobel_h, sobel_h_res);                      /* Make sobel operations  */
    itConv(*gray, gray_size, width, sobel_v, sobel_v_res);
    contour(*sobel_h_res, *sobel_v_res, gray_size, contour_img);                /* Calculate contour matrix */
    return gray_size;
}
//#define MAX_BRIGHTNESS 200u
/*-----------------------------------------------------------------------------
 *      sobel:  sobel filter for crop analysis
 *
 *
 *  Parameters: void
 *  Return:    void
 *----------------------------------------------------------------------------*/
void sobel_filtering( )
     /* Spatial filtering of image data */
     /* Sobel filter (horizontal differentiation */
     /* Input: image1[y][x] ---- Outout: image2[y][x] */
{
  /* Definition of Sobel filter in horizontal direction */
  int16_t weight[3u][3u] = {{ -1,  0,  1 },
                  { -2,  0,  2 },
                  { -1,  0,  1 }};
  float64_t pixel_value;
  float64_t minP, maxP;
  int16_t x, y, i, j;                                                            /* Loop variable */
  uint8_t image1[20u][20u];                                                     /* ----- check dimensions and sizes here ------- */
  uint8_t image2[20u][20u];
  uint8_t y_size1=20u,y_size2=20u;
  uint8_t x_size1=20u,x_size2=20u;

  /* Maximum values calculation after filtering*/
  minP = UINT64_MAX;                                                            /* uint64 max is big enough else find FLOAT64_MAX ? */
  maxP = -UINT64_MAX;
  for (y = 1; y < y_size1 - 1; y++) 
  {
    for (x = 1; x < x_size1 - 1; x++) 
    {
      pixel_value = 0.0f;
      for (j = -1; j <= 1; j++) 
      {
          for (i = -1; i <= 1; i++) 
          {
            pixel_value += weight[j + 1][i + 1] * image1[y + j][x + i];
          }
      }
      if (pixel_value < minP) minP = pixel_value;
      if (pixel_value > maxP) maxP = pixel_value;
    }
  }
  if ((int16_t)(maxP - minP) == 0) 
  {
    return;                                                                     /* nothing exisits exit */
  }

  x_size2 = x_size1;                                                            /* Initialization of image2[y][x] */
  y_size2 = y_size1;
  for (y = 0; y < y_size2; y++) 
  {
  
    for (x = 0; x < x_size2; x++) 
    {
      image2[y][x] = 0;
    }
  }

  for (y = 1; y < y_size1 - 1; y++)                                               /* Generation of image2 after linear transformtion */
  {
    for (x = 1; x < x_size1 - 1; x++) 
    {
      pixel_value = 0.0;
      for (j = -1; j <= 1; j++) 
      {
          for (i = -1; i <= 1; i++) 
          {
            pixel_value += weight[j + 1][i + 1] * image1[y + j][x + i];
          }
      }
      pixel_value = MAX_BRIGHTNESS * (pixel_value - minP) / (maxP - minP);
      image2[y][x] = (unsigned char)pixel_value;
    }
  }
}
/*-----------------------------------------------------------------------------
 *      otsu:  otsu for crop analysis
 *
 *
 *  Parameters: int16_t minVal,int16_t maxVal,int16_t ht,int16_t wth
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t otsu(int16_t minVal,int16_t maxVal,int16_t ht,int16_t wth)
{
        int16_t i,j,k,t,count,threshold;
        float64_t wt=0.0f,mean=0.0f,wt_bg,sum=0.0f,mean_bg,var=0.0f,var_bg;
        float64_t wt1=0.0f,mean1=0.0f,wt_fg,sum1=0.0f,mean_fg,var1=0.0f,var_fg;
        float64_t thes,temp,varience,sq,diff;
        int16_t hist[255u];                                                     /* should be global */
        
                for(i=minVal;i<=maxVal;i++)
                {
                   //wt = wt + hist[i];
                   mean=mean+i*hist[i];
                   sum = sum+hist[i];
                }
                if(sum!=0.0f)
                {
                  wt_bg = sum/(ht*wth);
                  mean_bg = mean/sum;
                  for(j=minVal;j<=maxVal;j++)
                  {
                        diff = j-mean_bg;
                        sq= pow(diff,2);
                        var =var + (sq*hist[j]);
                  }
                  var_bg=var/sum;
                  varience = wt_bg * pow(var_bg,2);
                  return (varience);
                }
                else
                  return(0.0f);
}
/*-----------------------------------------------------------------------------
 *      histogram_otsu:  otsu for crop analysis
 *
 *
 *  Parameters: int16_t ht,int16_t wth,int16_t **dataV
 *  Return:     uint16_t
 *----------------------------------------------------------------------------*/
int16_t histogram_otsu(int16_t ht,int16_t wth,int16_t **dataV)
{
        int16_t i,j,k,t,count,final_threshold;
        float64_t  threshold=0.0f;
        float64_t  wt=0.0f,mean=0.0f,wt_bg,sum=0.0f,mean_bg,var=0.0f,var_bg;
        float64_t  wt1=0.0f,mean1=0.0f,wt_fg,sum1=0.0f,mean_fg,var1=0.0f,var_fg;
        float64_t thes,temp;
        int16_t hist[255u];
        float64_t sigma[255u],sort[255u];
        
        for(k=0;k<=255;k++)
        {
           count = 0;
           for(i=0;i<ht;i++)
           {
                for(j=0;j<wth;j++)
                {
                    if(dataV[i][j]==k)
                    {
                        count=++count % INT16_MAX;
                        hist[k]=count;
                     }
                 }
            }
        }

/*        for(i=0;i<=255;i++)
        {
                printf("%d level total gray values = %d \n",i,hist[i]);
        }  */


        //calling OTSU
        for(t=0;t<254;t++)
        {
                //printf("\nLower boundary 0, higher boundary %d",t);
                var_bg = otsu(0,t,ht,wth);
                //printf("\nLower boundary %d, higher boundary 255",t);
                var_fg = otsu(t+1, 255,ht,wth);
                //printf("\n The variance background =%f varience forgraound=%f",var_bg,var_fg);
                sigma[t] = var_bg + var_fg;
                sort[t]= sigma[t];
        }
        //printf("\nThe minimum value is %f",thes);
        for (i=1;i<254;i++)
        {
             //printf("%f ",sigma[i]);
             for(j=1;j<254-i;j++)
             {
                 if(sigma[i]<sigma[j])
                 {
                      temp = sigma[i];
                      sigma[i]=sigma[j];
                      sigma[j]=temp;
                 }
             }
         }
         /* printf("\n");
         for(i=1;i<254;i++)
         {
             printf("%f ",sigma[i]);
         }*/
        thes = sigma[2];
        //printf("Threshold :%f",thes);
        for(i=1;i<=255;i++)
        {
            if(sort[i] == thes)
            threshold = i;
        }
        final_threshold = (int16_t)threshold;
        //printf("\nThe Threshold is %d\n",final_threshold);

#if defined(USE_MMC_SPI2)
        binary_conversion(final_threshold,ht,wth,255,"otsuFile.pgm",dataV);           /* write the file to the MMC file system */
#endif
        return (final_threshold);
}
/*-----------------------------------------------------------------------------
 *      houghtransform:  Hough transformation for line detection
 *
 *
 *  Parameters: uint8_t *d, int16_t *widthVal, int16_t *hightVal, int16_t *surfceVal, int16_t bpp
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t *houghtransform(uint8_t *d, int16_t *widthVal, int16_t *hightVal, int16_t *surfceVal, int16_t bpp)
{
  int16_t rho, theta, y, x, W = *widthVal, H = *hightVal;
  int16_t th = sqrt(W*W + H*H)/2.0f;
  int16_t tw = 360;
  uint8_t *ht = NULL;
  float64_t C;
  float64_t SinV;
  uint32_t totalred = 0u;
  uint32_t totalgreen = 0u;
  uint32_t totalblue = 0u;
  uint32_t totalpix = 0u;
  float64_t dy;
  float64_t dp;
  float64_t dx;
  
  memset(ht, 0, 4*th*tw);                                                       // black bg
  ht = Malloc(th*tw*4u);                                                        /* allocate memory for pointer */

  for(rho = 0; rho < th; rho++)
  {
    for(theta = 0; theta < tw/*720*/; theta++)
    {
      C = cos(RAD(theta));
      SinV = sin(RAD(theta));
      if ( theta < 45 || (theta > 135 && theta < 225) || theta > 315) {
        for(y = 0; y < H; y++) {
          dx = W/2.0 + (rho - (H/2.0-y)*SinV)/C;
          if ( dx < 0 || dx >= W ) continue;
          x = floor(dx+.5f);
          if (x == W) continue;
          totalpix++;
          totalred += GR(x, y);
          totalgreen += GG(x, y);
          totalblue += GB(x, y);
        }
      } else {
        for(x = 0; x < W; x++) {
          dy = H/2.0 - (rho - (x - W/2.0)*C)/SinV;
          if ( dy < 0 || dy >= H ) continue;
          y = floor(dy+.5);
          if (y == H) continue;
          totalpix++;
          totalred += GR(x, y);
          totalgreen += GG(x, y);
          totalblue += GB(x, y);
        }
      }
      if ( totalpix > 0 ) {
        dp = totalpix;
        SR(theta, rho) = (int16_t)(totalred/dp)   &0xffu;
        SG(theta, rho) = (int16_t)(totalgreen/dp) &0xffu;
        SB(theta, rho) = (int16_t)(totalblue/dp)  &0xffu;
      }
    }
  }

  *hightVal = th;                                                                   // sqrt(W*W+H*H)/2
  *widthVal = tw;                                                                   // 360
  *surfceVal = 4*tw;
  Free((char*)ht,sizeof(ht));
  return ht;
}

/*-----------------------------------------------------------------------------
 *      Prewitt_Filter:  prewitt edge anaysis filter
 *
 *
 *  Parameters: bitmap_t * bitmap, int16_t swidth, int16_t ewidth
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Prewitt_Filter(bitmap_t * bitmap, int16_t swidth, int16_t ewidth)
{
   bmp_pixel_t ** result;
   float32_t accum1 = 0, accum2 = 0;
   int16_t row1 = 0, col1 = 0, row, col, i, j;

   float32_t kGx[3][3] = { { -1.0f, 0, 1.0f },{ -1.0f, 0, 1.0f },{ -1.0f, 0, 1.0f } };
   float32_t kGy[3][3] = { { -1.0f, -1.0f, -1.0f },{ 0, 0, 0 },{ 1.0f, 1.0f, 1.0f } };

   if( (result = (bmp_pixel_t **) Malloc((bitmap->dheader.height)*sizeof(bmp_pixel_t *))) == NULL)
      return;

   for (i = 0; i < (bitmap->dheader.height); i++)
     if( (result[i] = (bmp_pixel_t*) Malloc((ewidth-swidth)*sizeof(bmp_pixel_t))) == NULL)
        return;

   for (row = 0; row < bitmap->dheader.height; row++)
      for (col = swidth; col < ewidth; col++)
      {
        accum1 = 0;
        accum2 = 0;
        for (i = -1; i <= 1; i++)
           for (j = -1; j <= 1; j++)
           {
                row1 = row + i >= 0 ? row + i : 0;
                col1 = col + j >= swidth  ? col + j : 0;
                row1 = row + i >= bitmap->dheader.height ? bitmap->dheader.height - 1 : row1;
                col1 = col + j >= ewidth  ? ewidth - 1  : col1;
                accum1 += bitmap->pixels[row1][col1].value * kGy[1 + i][1 + j];
                accum2 += bitmap->pixels[row1][col1].value * kGx[1 + i][1 + j];
           }
           result[row-0][col-swidth].value = (int) sqrt(pow(accum1, 2) + pow(accum2, 2));
      }

      for (row = 0; row < bitmap->dheader.height; row++)
         for (col = swidth; col < ewidth; col++)
             bitmap->pixels[row][col].value=result[row-0][col-swidth].value;

      for (i = 0; i < bitmap->dheader.height; i++)
          free((char*)result[i],sizeof(result[i]));

      free((char*)result,sizeof(result));
}
void binary_conversion(int16_t t,int16_t ht,int16_t wth,int16_t maxval, char *filename, int16_t **dataV)
{
    /* FILE *output; */
    int16_t i,j;
    /* char ot[11u]; */
    /* int16_t c;  */
    uint8_t line_len=0u;
    char file_line[20u];

    if (filename == NULL)
       return;

        /* sprintf(ot, "otsu%d.pgm",c);c++; */
        /* output = fopen(ot,"wb"); */
        if (Mmc_Fat_Assign(filename, 0u)==1u)                                   /* Find existing file and use it  */
        {
            Mmc_Fat_Rewrite();                                                  /* Opens the currently assigned file for writing. If the file is not empty its content will be erased. */
            Mmc_Fat_Write("P5\n", 3u);                                          /* write data to the assigned file */
            sprintf(file_line,"%d %d\n%d\n",wth,ht,maxval);
            Mmc_Fat_Write(file_line, strlen(file_line));                        /* write data to the assigned file */
/*            fprintf(output,"P5\n");
            fprintf(output,"%d ",wth);
            fprintf(output,"%d\n",ht);
            fprintf(output,"%d\n",maxval);   */
            for(i=0;i<ht;i++)
            {
                   for(j=0;j<wth;j++)
                   {
                      //printf("%d ",data[i][j]);
                      if(dataV[i][j]<t)
                      {
                          dataV[i][j]=0;
                      }
                      else
                      {
                          dataV[i][j]=255;
                      }
                   }
            }
            for(i=0;i<ht;i++)
            {
                   for(j=0;j<wth;j++)
                   {
                      sprintf(file_line,"%d",dataV[i][j]);
                      Mmc_Fat_Write(file_line, 2u);                                                /* write data to the assigned file */
/*                                                fwrite((void *)&dataV[i][j],sizeof(unsigned char),1,output); */
                   }
            }
        }
        /* fclose(output); */
}
// if normalize is true, map pixels to range 0..MAX_BRIGHTNESS
/*-----------------------------------------------------------------------------
 *      canny_conv:  canny edge detection filter
 *
 *
 *  Parameters: const pixel_t *in, pixel_t *out, const float32_t *kernel, 
 *  const int16_t nx, const int16_t ny, const int16_t kn, const uint8_t normalize
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void canny_conv(const pixel_t *in, pixel_t *out, const float32_t *kernel, const int16_t nx, const int16_t ny, const int16_t kn, const uint8_t normalize)
{
    // assert(kn % 2 == 1);
    // assert(nx > kn && ny > kn);
    const int16_t khalf = kn / 2;
    //float32_t min = FLT_MAX, max = -FLT_MAX;
    float32_t min = UINT64_MAX, max = -UINT64_MAX;
    int16_t m,n,j,i;
    float32_t pixel;
    size_t c;

    if (normalize)
        for (m = khalf; m < nx - khalf; m++)
            for (n = khalf; n < ny - khalf; n++)
                        {
                pixel = 0.0;
                c = 0;
                for (j = -khalf; j <= khalf; j++)
                    for (i = -khalf; i <= khalf; i++)
                                        {
                        pixel += in[(n - j) * nx + m - i] * kernel[c];
                        c++;
                    }
                if (pixel < min)
                    min = pixel;
                if (pixel > max)
                    max = pixel;
            }

    for (m = khalf; m < nx - khalf; m++)
        for (n = khalf; n < ny - khalf; n++)
                {
            pixel = 0.0;
            c = 0;
            for (j = -khalf; j <= khalf; j++)
                for (i = -khalf; i <= khalf; i++)
                                {
                    pixel += in[(n - j) * nx + m - i] * kernel[c];
                    c++;
                }

            if (normalize)
                pixel = MAX_BRIGHTNESS * (pixel - min) / (max - min);
            out[n * nx + m] = (pixel_t)pixel;
        }
}
/*
 * gaussianFilter:
 * http://www.songho.ca/dsp/cannyedge/cannyedge.html
 * determine size of kernel (odd #)
 * 0.0 <= sigma < 0.5 : 3
 * 0.5 <= sigma < 1.0 : 5
 * 1.0 <= sigma < 1.5 : 7
 * 1.5 <= sigma < 2.0 : 9
 * 2.0 <= sigma < 2.5 : 11
 * 2.5 <= sigma < 3.0 : 13 ...
 * kernelSize = 2 * int(2*sigma) + 3;
 */
 /*-----------------------------------------------------------------------------
 *      gaussian_filter:  gaussian filter
 *
 *
 *  Parameters: const pixel_t *inV, pixel_t *outV, int16_t nx, int16_t ny, float32_t sigma
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void gaussian_filter(const pixel_t *inV, pixel_t *outV, int16_t nx, int16_t ny, float32_t sigma)
{
    const int16_t n = 2 * (int16_t)(2 * sigma) + 3;
    const float32_t mean = (float32_t)floor(n / 2.0);
//    float32_t kernel[n * n];                                                  /* dont know how to do this ? */
    float32_t kernel[20u];
    int16_t i,j;
    size_t c = 0;
    
    if ((inV==NULL) || (outV==NULL))
       return;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
        {
            kernel[c] = exp(-0.5f * (pow((i - mean) / sigma, 2.0f) + pow((j - mean) / sigma, 2.0f))) / (2.0f * PI * sigma * sigma);
            c=++c % UINT16_MAX;
        }
    canny_conv(inV, outV, kernel, nx, ny, n, true);
}
/*
 * Links:
 * http://en.wikipedia.org/wiki/Canny_edge_detector
 * http://www.tomgibara.com/computer-vision/CannyEdgeDetector.java
 * http://fourier.eng.hmc.edu/e161/lectures/canny/node1.html
 * http://www.songho.ca/dsp/cannyedge/cannyedge.html
 *
 * Note: T1 and T2 are lower and upper thresholds.
 */
  /*-----------------------------------------------------------------------------
 *      canny_edge_detection:  canny filter edge detection 
 *
 *
 *  Parameters: const pixel_t *in, const dib_header_t *bmp_ih, const int16_t tmin, const int16_t tmax, const float32_t sigma
 *
 *  Return:     pixel_t *
 *----------------------------------------------------------------------------*/
pixel_t *canny_edge_detection(const pixel_t *in, const dib_header_t *bmp_ih, const int16_t tmin, const int16_t tmax, const float32_t sigma)
{
    const int16_t nx = bmp_ih->width;
    const int16_t ny = bmp_ih->height;
    int16_t i,j,k;
    size_t c;
    const float32_t Gx[] = {-1, 0, 1,
                        -2, 0, 2,
                        -1, 0, 1};

    const float32_t Gy[] = { 1, 2, 1,
                         0, 0, 0,
                        -1,-2,-1};
    int16_t nn;
    int16_t ss;
    int16_t ww;
    int16_t ee;
    int16_t nw;
    int16_t ne;
    int16_t sw;
    int16_t se;
    float32_t dir;
    int16_t *edges;
    int16_t t;
    int16_t nbs[8]; // neighbours
    int16_t nedges;

    pixel_t *G = malloc(nx * ny * sizeof(pixel_t));
    pixel_t *after_Gx = malloc(nx * ny * sizeof(pixel_t));
    pixel_t *after_Gy = malloc(nx * ny * sizeof(pixel_t));
    pixel_t *nms = malloc(nx * ny * sizeof(pixel_t));
    pixel_t *out = malloc(bmp_ih->bmp_bytesz * sizeof(pixel_t));

    if (G == NULL || after_Gx == NULL || after_Gy == NULL || nms == NULL || out == NULL)
    {
        return(out);
    }

    gaussian_filter(in, out, nx, ny, sigma);
    canny_conv(out, after_Gx, Gx, nx, ny, 3, false);
    canny_conv(out, after_Gy, Gy, nx, ny, 3, false);

    for (i = 1; i < nx - 1; i++)
        for (j = 1; j < ny - 1; j++)
        {
            c = i + nx * j;
            G[c] = abs(after_Gx[c]) + abs(after_Gy[c]);
            //G[c] = (pixel_t)hypot(after_Gx[c], after_Gy[c]);
        }

    // Non-maximum suppression, straightforward implementation.
    for (i = 1; i < nx - 1; i++)
        for (j = 1; j < ny - 1; j++)
        {
            c = i + nx * j;
            nn = c - nx;
            ss = c + nx;
            ww = c + 1;
            ee = c - 1;
            nw = nn + 1;
            ne = nn - 1;
            sw = ss + 1;
            se = ss - 1;

            /* double check fmod dir = (float32_t)(fmod(atan2(after_Gy[c], after_Gx[c]) + PI, PI) / PI) * 8; */
            dir = ((float32_t)((uint32_t)(atan2(after_Gy[c], after_Gx[c]) + PI) % (uint32_t) PI) / (float32_t) PI) * 8.0f;
            if (((dir <= 1.0f || dir > 7.0f) && G[c] > G[ee] && G[c] > G[ww]) || ((dir > 1.0f && dir <= 3.0f) && G[c] > G[nw] && G[c] > G[se]) || ((dir > 3.0f && dir <= 5.0f) && G[c] > G[nn] && G[c] > G[ss]) || ((dir > 5.0f && dir <= 7.0f) && G[c] > G[ne] && G[c] > G[sw]))
                nms[c] = G[c];
            else
                nms[c] = 0;
        }

    // used as a stack. nx*ny/2 elements should be enough.
    edges = (int16_t*) malloc(nx * ny * sizeof(pixel_t));
    memset(out, 0, sizeof(pixel_t) * nx * ny);
    memset(edges, 0, sizeof(pixel_t) * nx * ny);

    // Tracing edges with hysteresis . Non-recursive implementation.
    c = 1;
    for (j = 1; j < ny - 1; j++)
        for (i = 1; i < nx - 1; i++)
                {
            if (nms[c] >= tmax && out[c] == 0) { // trace edges
                out[c] = MAX_BRIGHTNESS;
                nedges = 1;
                edges[0] = c;
                do 
                {
                    nedges--;
                    t = edges[nedges];
                    nbs[0] = t - nx;     // nn
                    nbs[1] = t + nx;     // ss
                    nbs[2] = t + 1;      // ww
                    nbs[3] = t - 1;      // ee
                    nbs[4] = nbs[0] + 1; // nw
                    nbs[5] = nbs[0] - 1; // ne
                    nbs[6] = nbs[1] + 1; // sw
                    nbs[7] = nbs[1] - 1; // se

                    for (k = 0; k < 8; k++)
                        if (nms[nbs[k]] >= tmin && out[nbs[k]] == 0)
                        {
                            out[nbs[k]] = MAX_BRIGHTNESS;
                            edges[nedges] = nbs[k];
                            nedges++;
                        }
                } while (nedges > 0);
            }
            c++;
        }

    free((char*)after_Gx,sizeof(after_Gx));
    free((char*)after_Gy,sizeof(after_Gy));
    free((char*)G,sizeof(G));
    free((char*)nms,sizeof(nms));
    free((char*)edges,sizeof(edges));

    return out;
}

// Create four broadly tuned color channels as
// described in "A model of saliency-based visual
// attention for rapid scene analysis" by Itti, Koch, and Niebur.
//
// first divide green and blue in half to avoid
// overflow during addition
 /*-----------------------------------------------------------------------------
 *      doIttiKochNiebur:  saliency-based visual attention for rapid scene analysis
 *
 *
 *  Parameters: hue_t *obj
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void doIttiKochNiebur( hue_t *obj )
{

   // Red Channel
   // r' = r - (g + b) / 2
   obj->red_channel=obj->r-((obj->g + obj->b)/2u);                              // r - (g + b) / 2

   // Green Channel
   // g' = g - (r + b) / 2
   obj->green_channel= obj->g - ((obj->r + obj->b)/ 2u);                        // g - (r + b) / 2

   // Blue Channel
   // b' = b - (r + g) / 2
   obj->blue_channel= obj->b - ((obj->r + obj->g)/ 2u);                         // b - (r + g) / 2

   // Yellow Channel
   // y' = (r + g) / 2 - |r - g| / 2 - b
   //
   // r > g: (r + g) - (r - g) = 2g
   // r < g: (r + g) - (g - r) = 2r
   // r = g: (r + r) - (g - g) = 2r = 2g
   //
   // y' = min(r,g) - b
   obj->yellow_channel= min(obj->r,obj->g) - obj->b;
}
/**
 * Undoes gamma-correction from an RGB-encoded color.
 * https://en.wikipedia.org/wiki/SRGB#Specification_of_the_transformation
 * https://stackoverflow.com/questions/596216/formula-to-determine-brightness-of-rgb-color
 * @param  color (r g or b)
 * @return a linearized value
 */
  /*-----------------------------------------------------------------------------
 *      sRGBtoLinearRGB:  srgb to rgb
 *
 *
 *  Parameters: float32_t color
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/ 
float32_t sRGBtoLinearRGB( float32_t color)
{
    // Send this function a decimal sRGB gamma encoded color value
    // between 0.0 and 1.0, and it returns a linearized value.
    if (color <= 0.04045f) 
    {
        return (color / 12.92f);
    } 
    else 
    {
        return pow(((color + 0.055f) / 1.055f), 2.4f);
    }
}
/* ports from paparazzi project on github */
  /*-----------------------------------------------------------------------------
 *      resize_uyuv:  picture resize downsample by rate
 *
 *
 *  Parameters: img_struct_t* input, img_struct_t* output, int16_t downsample
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/ 
void resize_uyuv(img_struct_t* input, img_struct_t* output, int16_t downsample)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  int16_t pixelskip,y,x;
  
  pixelskip = downsample-1;
  for (y=0;y<output->h;y++)
  {
    for (x=0;x<output->w;x+=2)                                                  // YUYV
    {
      *dest++ = *source++;                                                      // U
      *dest++ = *source++;                                                      // Y
      source+=(pixelskip+1)*2;                                                  // now skip 3 pixels
      *dest++ = *source++;                                                      // U
      *dest++ = *source++;                                                      // V
      source+=(pixelskip-1)*2;
    }
    source += pixelskip * input->w * 2;                                         // skip 3 rows
  }
}
void grayscale_uyvy(img_struct_t* input, img_struct_t* output);
void grayscale_uyvy(img_struct_t* input, img_struct_t* output)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  int16_t y,x;
  
  source++;
  for (y=0;y<output->h;y++)
  {
    for (x=0;x<output->w;x++)                                                   // UYVY
    {
      *dest++ = 127u;                                                           // U
      *dest++ = *source;                                                        // Y
      source+=2u;
    }
  }
}
  /*-----------------------------------------------------------------------------
 *      colorfilt_uyvy:  color filter
 *
 *
 *  Parameters: img_struct_t* input, img_struct_t* output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M
 *
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
int16_t colorfilt_uyvy(img_struct_t* input, img_struct_t* output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M);
int16_t colorfilt_uyvy(img_struct_t* input, img_struct_t* output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M)
{
  int16_t cnt = 0;
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  int16_t y,x;

  for (y=0;y<output->h;y++)
  {
    for (x=0;x<output->w;x+=2)                                                  // Color Check:
    {
      if 
      (
               (dest[1u] >= y_m)
            && (dest[1u] <= y_M)
            && (dest[0u] >= u_m)
            && (dest[0u] <= u_M)
            && (dest[2u] >= v_m)
            && (dest[2u] <= v_M)
         )                                                                      // && (dest[2] > 128))
      {
        cnt=++cnt % INT16_MAX;
        // UYVY
        dest[0u] = 64u;        // U
        dest[1u] = source[1u];  // Y
        dest[2u] = 255u;        // V
        dest[3u] = source[3u];  // Y
      }
      else
      {
        // UYVY
        char u = source[0u]-127;
        u/=4;
        dest[0u] = 127;        // U
        dest[1u] = source[1];  // Y
        u = source[2]-127;
        u/=4;
        dest[2u] = 127;        // V
        dest[3u] = source[3];  // Y
      }
      dest+=4u;
      source+=4u;
    }
  }
  return cnt;
}
/*-----------------------------------------------------------------------------
 *      rainbow:  rainbow filter
 *
 *
 *  Parameters: uint8_t n, uint8_t *r, uint8_t *g, uint8_t *b
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void rainbow(uint8_t n, uint8_t *r, uint8_t *g, uint8_t *b)
{
        if (n < 255u / 3u) 
        {
          *r = n * 3u;
          *g = 255u - n * 3u;
          *b = 0u;
        } 
        else if (n < 255u / 3u * 2u) 
        {
           n -= 255u / 3u;
           *r = 255u - n * 3u;
           *g = 0u;
           *b = n * 3u;
        } 
        else
        {
           n -= 255u / 3u * 2u;
           *r = 0u;
           *g = n * 3u;
           *b = 255u - n * 3u;
        }
}

/*
 * BCFlight
 * Copyright (C) 2016 Adrien Aubry (drich)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
**/
/*-----------------------------------------------------------------------------
 *      Raspicam_LensShader_internal:  lens shader
 *
 *
 *  Parameters: const LensShaderColor_t *R, const LensShaderColor_t *G, const LensShaderColor_t *B, uint8_t grid[52u * 39u * 4u] 
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Raspicam_LensShader_internal( const LensShaderColor_t *R, const LensShaderColor_t *G, const LensShaderColor_t *B, uint8_t grid[52u * 39u * 4u] )
{
        int32_t y,x,r1,g1,b1,dist;
        float32_t dot_r,dot_g,dot_b;

        for ( y = 0; y < 39; y++ )
        {
                for ( x = 0; x < 52; x++ )
                {
                        r1 = R->base;
                        g1 = G->base;
                        b1 = B->base;
                        dist = sqrt( ( x - 52 / 2 ) * ( x - 52 / 2 ) + ( y - 39 / 2 ) * ( y - 39 / 2 ) );
                        if ( R->radius > 0 )
                        {
                                dot_r = (float32_t)max( 0, R->radius - dist ) / (float32_t)R->radius;
                                r1 = max( 0, min( 255, r1 + (int32_t)( dot_r * R->strength ) ) );
                        }
                        if ( G->radius > 0 )
                        {
                                dot_g = (float32_t)max( 0, G->radius - dist ) / (float32_t)G->radius;
                                g1 = max( 0, min( 255, g1 + (int32_t)( dot_g * G->strength ) ) );
                        }
                        if ( B->radius > 0 )
                        {
                                dot_b = (float32_t)max( 0, B->radius - dist ) / (float32_t)B->radius;
                                b1 = max( 0, min( 255, b1 + (int32_t)( dot_b * B->strength ) ) );
                        }
                        grid[ (x + y * 52) + ( 52 * 39 ) * 0 ] = r1;
                        grid[ (x + y * 52) + ( 52 * 39 ) * 1 ] = g1;
                        grid[ (x + y * 52) + ( 52 * 39 ) * 2 ] = g1;
                        grid[ (x + y * 52) + ( 52 * 39 ) * 3 ] = b1;
                }
        }
}
/* Chromatic aberration correction prototype.
Copyright (C) 2014 Victoria Rudakova <vicrucann@gmail.com>
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ported by ACP */
/*-----------------------------------------------------------------------------
 *      average_image:  average the image
 *
 *
 *  Parameters: image_double_t *img
 *
 *  Return:     image_double_t *
 *----------------------------------------------------------------------------*/
image_double_t * average_image(image_double_t *img)
{
        int16_t v,u;
        int16_t w = img->xsize;
        int16_t h = img->ysize;
        float64_t pix;
        image_double_t *img_avg = img;
        for (v = 1; v < h-1; v++)
        {
                for (u = 1; u < w-1; u++)
                {
                        pix = img->dataV[u-1+(v-1)*w] + img->dataV[u+(v-1)*w] + img->dataV[u+1+(v-1)*w] + img->dataV[u-1+v*w] + img->dataV[u+v*w] + img->dataV[u+1+v*w] + img->dataV[u-1+(v+1)*w] + img->dataV[u+(v+1)*w] + img->dataV[u+1+(v+1)*w];
                        img_avg->dataV[u+v*w] = pix/9;
                }
        }
        return img_avg;
}
/*-----------------------------------------------------------------------------
 *      white_neighbors:  look for white neighbors
 *
 *
 *  Parameters: picsel_p_t *p, image_double_t *img
 *
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
int16_t white_neighbors(picsel_p_t *p, image_double_t *img)
{
        int16_t nn = 0;
        float64_t color;
        picsel_p_t p1,p2,p3,p4;

        p1.x = p->x-1;
        p1.y = p->y;
        if (p1.x >= 0 && p1.x < img->xsize && p1.y >= 0 && p1.y < img->ysize) {
                if (img->dataV[p1.x+p1.y*img->xsize] == 255)
                        nn++; }
        else nn++;

        p2.x = p->x+1;
        p2.y = p->y;
        if (p2.x >= 0 && p2.x < img->xsize && p2.y >= 0 && p2.y < img->ysize) {
                if (img->dataV[p2.x+p2.y*img->xsize] == 255)
                        nn++; }
        else nn++;

        p3.x = p->x;
        p3.y = p->y-1;
        if (p3.x >= 0 && p3.x < img->xsize && p3.y >= 0 && p3.y < img->ysize) {
                if (img->dataV[p3.x+p3.y*img->xsize] == 255)
                        nn++; }
        else nn++;

        p4.x = p->x;
        p3.y = p->y+1;
        if (p4.x >= 0 && p4.x < img->xsize && p4.y >= 0 && p4.y < img->ysize) {
                if (img->dataV[p4.x+p4.y*img->xsize] == 255)
                        nn++; }
        else nn++;

        if (nn > 0) nn = 1;
        return nn;
}
/* Spline interpolation.
    Copyright (C) 2007 Lionel Moisan <Lionel.Moisan@parisdescartes.fr>
    Copyright (C) 2010 Pascal Monasse <monasse@imagine.enpc.fr>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
// WARNING:
// This file implements an algorithm possibly linked to the patent
//
// David Lowe  "Method and apparatus for identifying scale invariant
// features in an image and use of same for locating an object in an
// image",  U.S. Patent 6,711,293.
//
// This file is made available for the exclusive aim of serving as
// scientific tool to verify of the soundness and
// completeness of the algorithm description. Compilation,
// execution and redistribution of this file may violate exclusive
// patents rights in certain countries.
// The situation being different for every country and changing
// over time, it is your responsibility to determine which patent
// rights restrictions apply to you before you compile, use,
// modify, or redistribute this file. A patent lawyer is qualified
// to make this determination.
// If and only if they don't conflict with any patent terms, you
// can benefit from the following license terms attached to this
// file.
//
// This program is provided for scientific and educational only:
// you can use and/or modify it for these purposes, but you are
// not allowed to redistribute this work or derivative works in
// source or executable form. A license must be obtained from the
// patent right holders for any other use.
/*-----------------------------------------------------------------------------
 *      initcausal4:  
 *
 *
 *  Parameters: float64_t* c, int16_t step, int16_t n, float64_t z
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
static float64_t initcausal4(float64_t* c, int16_t step, int16_t n, float64_t z)
{
    float64_t zk,z2k,iz,sum;
    int16_t k;

    zk = z; iz = 1./z;
    z2k = pow(z,(double)n-1.);
    sum = c[0] + z2k * c[step*(n-1)];
    z2k = z2k*z2k*iz;
    for(k = 1; k <= n-2; k++) {
        sum += (zk+z2k)*c[step*k];
        zk *= z;
        z2k *= iz;
    }
    return (sum/(1.0f-zk*zk));
}
/*-----------------------------------------------------------------------------
 *      initanticausal4:  
 *
 *
 *  Parameters: float64_t* c, int16_t step, int16_t n, float64_t z
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
static float64_t initanticausal4(float64_t* c, int16_t step, int16_t n, float64_t z)
{
    return (z/(z*z-1.)) * (z*c[step*(n-2)]+c[step*(n-1)]);
}
/*-----------------------------------------------------------------------------
 *      initcausal:  
 *
 *
 *  Parameters: float64_t *c, int16_t n, float64_t z
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
static float64_t initcausal(float64_t *c, int16_t n, float64_t z)
{
  float64_t zk,z2k,iz,sum;
  int16_t k;

  zk = z; iz = 1./z;
  z2k = pow(z,(double)n-1.0f);
  sum = c[0] + z2k * c[n-1];
  z2k = z2k*z2k*iz;
  for (k=1;k<=n-2;k++) {
    sum += (zk+z2k)*c[k];
    zk *= z;
    z2k *= iz;
  }
  return (sum/(1.0f-zk*zk));
}
/*-----------------------------------------------------------------------------
 *      initanticausal:  
 *
 *
 *  Parameters: float64_t *c, int16_t n, float64_t z
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
static float64_t initanticausal(float64_t *c, int16_t n, float64_t z)
{
  return((z/(z*z-1.))*(z*c[n-2]+c[n-1]));
}
/*-----------------------------------------------------------------------------
 *      invspline1D5:  
 *
 *
 *  Parameters: float64_t* c, int16_t step, int16_t size, float64_t* z, int16_t npoles
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void invspline1D5(float64_t* c, int16_t step, int16_t size, float64_t* z, int16_t npoles)
{
    /* normalization */
    float64_t lambda=1.0;
    int16_t n,k;
    for(k = npoles-1; k >= 0; k--)
        lambda *= (1.0f-z[k])*(1.0f-1.0f/z[k]);
    for(n = size-1; n >= 0; n--)
        c[step*n] *= (float64_t)(lambda);

    for(k=0 ; k < npoles; k++) 
    {                                                                           // Loop on poles
        c[0] = (float64_t)(initcausal4(c, step, size, z[k]));                    /* forward recursion */
        for(n=1; n < size; n++)
            c[step*n] += (float64_t)(z[k]*c[step*(n-1)]);
        c[step*(size-1)] = (float64_t)(initanticausal4(c, step, size, z[k]));    /* backward recursion */
        for(n=size-2; n >= 0; n--)
            c[step*n] = (float64_t)(z[k]*(c[step*(n+1)]-c[step*n]));
    }
}
/*-----------------------------------------------------------------------------
 *      invspline1D:  
 *
 *
 *  Parameters: float64_t* c, int16_t size, float64_t* z, int16_t npoles
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void invspline1D(float64_t *c, int16_t size, float64_t *z, int16_t npoles)
{
  float64_t lambda;
  int16_t n,k;

  /* normalization */
  for (k=npoles,lambda=1.;k--;) lambda *= (1.-z[k])*(1.-1./z[k]);
  for (n=size;n--;) c[n] *= lambda;

  /*----- Loop on poles -----*/
  for (k=0;k<npoles;k++) {

    /* forward recursion */
    c[0] = initcausal(c,size,z[k]);
    for (n=1;n<size;n++)
      c[n] += z[k]*c[n-1];

    /* backwards recursion */
    c[size-1] = initanticausal(c,size,z[k]);
    for (n=size-1;n--;)
      c[n] = z[k]*(c[n+1]-c[n]);

  }
}
 
/*-----------------------------------------------------------------------------
 *      fill_poles : Put in array \a z the poles of the spline of given \a order. 
 *
 *
 *  Parameters: float64_t* z, int16_t order
 *
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
static uint8_t fill_poles(float64_t* z, int16_t order)
{
    switch(order) {
    case 1:
        break;
    case 2: z[0]=-0.17157288f;  /* sqrt(8)-3 */
        break;
    case 3: z[0]=-0.26794919f;  /* sqrt(3)-2 */
        break;
    case 4: z[0]=-0.361341f; z[1]=-0.0137254f;
        break;
    case 5: z[0]=-0.430575f; z[1]=-0.0430963f;
        break;
    case 6: z[0]=-0.488295f; z[1]=-0.0816793f; z[2]=-0.00141415f;
        break;
    case 7: z[0]=-0.53528f; z[1]=-0.122555f; z[2]=-0.00914869f;
        break;
    case 8: z[0]=-0.574687f; z[1]=-0.163035f; z[2]=-0.0236323f;
         z[3]=-0.000153821f;
         break;
    case 9: z[0]=-0.607997f; z[1]=-0.201751f; z[2]=-0.0432226f; z[3]=-0.00212131f;
        break;
    case 10: z[0]=-0.636551f; z[1]=-0.238183f; z[2]=-0.065727f;
        z[3]=-0.00752819f; z[4]=-0.0000169828f;
        break;
    case 11: z[0]=-0.661266f; z[1]=-0.27218f; z[2]=-0.0897596f; z[3]=-0.0166696f;
        z[4]=-0.000510558f;
        break;
    default:
        return false;
    }
    return true;
}
 
/*-----------------------------------------------------------------------------
 *      prepare_spline : Prepare image for cardinal spline interpolation. 
 *
 *
 *  Parameters: image_double_t *im, int16_t order
 *
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
static uint8_t prepare_spline(image_double_t *im, int16_t order)
{
    float64_t z[5];
    int16_t x,y,npoles;
    
    if(order < 3)
        return true;

    // Init poles of associated z-filter
    if(! fill_poles(z, order))
        return false;
    npoles = order/2;

        for(y = 0; y < im->ysize; y++) // Filter on lines
                invspline1D5(im->dataV+y*im->xsize, 1, im->xsize, z, npoles);
        for(x = 0; x < im->xsize; x++) // Filter on columns
                invspline1D5(im->dataV+x, 1*im->xsize, im->ysize, z, npoles);
    return true;
}

/*-----------------------------------------------------------------------------
 *      keysf : coefficients for cubic interpolant (Keys' function) 
 *
 *
 *  Parameters: float32_t *c,float32_t t,float32_t a
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void keysf(float32_t *c,float32_t t,float32_t a)
{
  float32_t t2,at1;

  t2 = t*t;
  at1 = a*t;
  c[0u] = a*t2*(1-t);
  c[1u] = (2*a+3 - (a+2)*t)*t2 - at1;
  c[2u] = ((a+2)*t - a-3)*t2 + 1;
  c[3u] = a*(t-2)*t2 + at1;
}

/* c[] = values of interpolation function at ...,t-2,t-1,t,t+1,... */

/*-----------------------------------------------------------------------------
 *      keysd : coefficients for cubic interpolant (Keys' function)
 *
 *
 *  Parameters: float32_t *c,float32_t t,float32_t a
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void keysd(float64_t* c, float64_t t, float64_t a)
{
    float64_t t2 = t*t;
    float64_t at1 = a*t;
    c[0u] = a*t2*(1.0f-t);
    c[1u] = (2.0f*a+3.0f - (a+2.0f)*t)*t2 - at1;
    c[2u] = ((a+2.0f)*t - a-3.0f)*t2 + 1.0f;
    c[3u] = a*(t-2.0f)*t2 + at1;
}

/*-----------------------------------------------------------------------------
 *      spline3f : coefficients for cubic spline
 *
 *
 *  Parameters: float32_t *c,float32_t t
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void spline3f(float32_t *c,float32_t t)
{
  float32_t tmp;

  tmp = 1-t;
  c[0u] = 0.1666666666f*t*t*t;
  c[1u] = 0.6666666666f-0.5f*tmp*tmp*(1+t);
  c[2u] = 0.6666666666f-0.5f*t*t*(2-t);
  c[3u] = 0.1666666666f*tmp*tmp*tmp;
}

/*-----------------------------------------------------------------------------
 *      spline3d : coefficients for cubic spline
 *
 *
 *  Parameters: float64_t *c,float64_t t
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void spline3d(float64_t* c, float64_t t)
{
    float64_t tmp = 1.0f-t;
    c[0u] = 0.1666666666f *t*t*t;
    c[1u] = 0.6666666666f -0.5f*tmp*tmp*(1.0f+t);
    c[2u] = 0.6666666666f -0.5f*t*t*(2.0f-t);
    c[3u] = 0.1666666666f *tmp*tmp*tmp;
}

/*-----------------------------------------------------------------------------
 *      init_splinenf : pre-computation for spline of order >3
 *
 *
 *  Parameters: float32_t *a,int16_t n
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void init_splinenf(float32_t *a,int16_t n)
{
  int16_t k;

  a[0u] = 1.;
  for (k=2;k<=n;k++) a[0u]/=(float)k;
  for (k=1;k<=n+1;k++)
    a[k] = - a[k-1] *(float32_t)(n+2-k)/(float)k;
}

/*-----------------------------------------------------------------------------
 *      init_splinend : pre-computation for spline of order >3
 *
 *
 *  Parameters: float64_t *a,int16_t n
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void init_splinend(float64_t* a, int16_t n)
{
    int16_t k;
    a[0u] = 1.;
    for(k=2; k <= n; k++)
        a[0u]/=(float64_t)k;
    for(k=1; k <= n+1; k++)
        a[k] = - a[k-1] *(n+2-k)/k;
}

/*-----------------------------------------------------------------------------
 *      ipow : fast integral power function
 *
 *
 *  Parameters: float32_t *a,int16_t n
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
static float32_t ipow(float32_t x, int16_t n)
{
    float32_t res;
    for(res = 1.; n; n>>=1) {
        if(n&1)
            res *= x;
        x *= x;
    }
    return res;
}

/*-----------------------------------------------------------------------------
 *      splinenf : coefficients for spline of order >3
 *
 *
 *  Parameters: float32_t *c,float32_t t,float32_t *a,int16_t n
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void splinenf(float32_t *c,float32_t t,float32_t *a,int16_t n)
{
  int16_t i,k;
  float32_t xn;

  memset((void *)c,0,(n+1)*sizeof(float));
  for (k=0;k<=n+1;k++) {
    xn = ipow(t+(float)k,n);
    for (i=k;i<=n;i++)
      c[i] += a[i-k]*xn;
  }
}

/*-----------------------------------------------------------------------------
 *      splinend : coefficients for spline of order >3
 *
 *
 *  Parameters: float64_t *c,float64_t t,float64_t *a,int16_t n
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void splinend(float64_t *c, float64_t t, float64_t *a, int16_t n)
{
    int16_t i,k;
    float64_t xn;
    
    memset((void*)c, 0, (n+1)*sizeof(double));
    for(k=0; k <= n+1; k++) {
        xn = ipow(t+(double)k, n);
        for(i=k; i <= n; i++)
            c[i] += a[i-k]*xn;
    }
}
/*----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 *      valid_image_double : Is pixel inside image?
 *
 *
 *  Parameters: image_double_t *in, int16_t x, int16_t y
 *
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
static uint8_t valid_image_double(image_double_t *in, int16_t x, int16_t y)
{
        return (0 <= x && x < in->xsize && 0 <= y && y < in->ysize);
}

/// Spline interpolation of given \a order of image \a im at point (x,y).
/// \a out must be an array of size the number of components.
/// Supported orders: 0(replication), 1(bilinear), -3(Keys's bicubic), 3, 5, 7,
/// 9, 11.
/// \a paramKeys is Keys's parameter, only used for order -3.
/// Success means a valid order and pixel in image.
/*-----------------------------------------------------------------------------
 *      interpolate_spline : Spline interpolation of given order of image im at point (x,y).
 *
 *
 *  Parameters: image_double_t *im, int16_t order, float64_t x, float64_t y, float64_t *out, float64_t paramKeys
 *
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t interpolate_spline( image_double_t *im, int16_t order, float64_t x, float64_t y, float64_t *out, float64_t paramKeys)
{
        float64_t  cx[12u],cy[12u];
        float64_t ak[13u];
        int xi,yi,n1,n2,dy,dx,adrs;
        float64_t p,ux,uy,f,v;
        uint8_t bInside = false;

        /* CHECK ORDER */
        if(order != 0 && order != 1 && order != -3 && order != 3 && order != 5 && order != 7 && order != 9 && order != 11)
                return false;

        if(order > 3)
                init_splinend(ak, order);

        /* INTERPOLATION */
        if(order == 0) { /* zero order interpolation (pixel replication) */
                xi = (int16_t)floor((float64_t)x);
                yi = (int16_t)floor((float64_t)y);
                bInside = valid_image_double(im, xi, yi);//im.valid(xi, yi);
                if(bInside) {
                        p = im->dataV[xi+yi*im->xsize]; //im.pixel(xi, yi);
                        *out = p;
                }
        } else { /* higher order interpolations */
                bInside = (x>=0.0f && x<=(float64_t)im->xsize && y>=0.0f && y<=(float64_t)im->ysize);
                if(bInside) {
                        x -= 0.5f; y -= 0.5f;
                        xi = (x<0)? -1: (int)x;
                        yi = (y<0)? -1: (int)y;
                        ux = x - (float64_t)xi;
                        uy = y - (float64_t)yi;
                        switch(order)  {
                        case 1: /* first order interpolation (bilinear) */
                                cx[0u] = ux; cx[1u] = 1.0f-ux;
                                cy[0u] = uy; cy[1u] = 1.0f-uy;
                                break;
                        case -3: /* third order interpolation (bicubic Keys' function) */
                                keysd(cx, ux, paramKeys);
                                keysd(cy, uy, paramKeys);
                                break;
                        case 3: /* spline of order 3 */
                                spline3d(cx, ux);
                                spline3d(cy, uy);
                                break;
                        default: /* spline of order >3 */
                                splinend(cx, ux, ak, order);
                                splinend(cy, uy, ak, order);
                                break;
                        }
                        n2 = (order==-3)? 2: (order+1)/2;
                        n1 = 1-n2;
                        /* this test saves computation time */
                        if(valid_image_double(im, xi+n1, yi+n1) && valid_image_double(im, xi+n2,yi+n2)) {
                                *out = 0.0f;
                                for(dy = n1; dy <= n2; dy++) {
                                        adrs = (xi+n1) + (yi+dy) * im->xsize;
                                        for(dx = n1; dx <= n2; dx++) {
                                                f = im->dataV[adrs];
                                                *out += cy[n2-dy]*cx[n2-dx] * f;
                                                adrs++;
                                        }
                                }

                        } else
                                *out = 0.0f;
                        for(dy = n1; dy <= n2; dy++)
                                for(dx = n1; dx <= n2; dx++) {
                                        v = 0.0f; // the image is not infinite, there is no data outside
                                        *out += cy[n2-dy]*cx[n2-dx]*v;
                                }

                }
        }
        return bInside;
}
/*-----------------------------------------------------------------------------
 *      finvspline : Inverse Spline interpolation 
 *
 *
 *  Parameters: float32_t *in, int16_t order, float32_t *out, int16_t width, int16_t height
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void finvspline(float32_t *in, int16_t order, float32_t *out, int16_t width, int16_t height)
{
  float64_t *c,*d,z[5];
  int npoles,nx,ny,x,y;

  ny = height; nx = width;
  switch (order)                                                                /* initialize poles of associated z-filter */
    {
    case 2: z[0]=-0.17157288f;                                                  /* sqrt(8)-3 */
      break;

    case 3: z[0]=-0.26794919f;                                                  /* sqrt(3)-2 */
      break;

    case 4: z[0]=-0.361341f; z[1]=-0.0137254f;
      break;

    case 5: z[0]=-0.430575f; z[1]=-0.0430963f;
      break;

    case 6: z[0]=-0.488295f; z[1]=-0.0816793f; z[2]=-0.00141415f;
      break;

    case 7: z[0]=-0.53528f; z[1]=-0.122555f; z[2]=-0.00914869f;
      break;

    case 8: z[0]=-0.574687f; z[1]=-0.163035f; z[2]=-0.0236323f; z[3]=-0.000153821f;
      break;

    case 9: z[0]=-0.607997f; z[1]=-0.201751f; z[2]=-0.0432226f; z[3]=-0.00212131f;
      break;

    case 10: z[0]=-0.636551f; z[1]=-0.238183f; z[2]=-0.065727f; z[3]=-0.00752819f;
      z[4]=-0.0000169828;
      break;

    case 11: z[0]=-0.661266f; z[1]=-0.27218f; z[2]=-0.0897596f; z[3]=-0.0166696f;
      z[4]=-0.000510558f;
      break;

     default:
     return;
    }
  npoles = order/2;

  c = (float64_t *)malloc(nx*ny*sizeof(float64_t));                                   /* initialize double array containing image */
  d = (float64_t *)malloc(nx*ny*sizeof(float64_t));
  for (x=nx*ny;x--;)
    c[x] = (float64_t)in[x];

  for (y=0;y<ny;y++)                                                            /* apply filter on lines */
    invspline1D(c+y*nx,nx,z,npoles);

  for (x=0;x<nx;x++)                                                            /* transpose */
    for (y=0;y<ny;y++)
      d[x*ny+y] = c[y*nx+x];

  for (x=0;x<nx;x++)                                                            /* apply filter on columns */
    invspline1D(d+x*ny,ny,z,npoles);

  for (x=0;x<nx;x++)                                                            /* transpose directy into image */
    for (y=0;y<ny;y++)
      out[y*nx+x] = (float32_t)(d[x*ny+y]);

  free((char*)d,(uint32_t)sizeof(d));                                           /* free array */
  free((char*)c,(uint32_t)sizeof(c));
}
/* * Compatibility for most of the publicly available 2D and 3D, single and multi-person pose estimation datasets including **[Human3.6M](http://vision.imar.ro/human3.6m/description.php), [MPII](http://human-pose.mpi-inf.mpg.de/), [MS COCO 2017](http://cocodataset.org/#home), [MuCo-3DHP](http://gvv.mpi-inf.mpg.de/projects/SingleShotMultiPerson/) and [MuPoTS-3D](http://gvv.mpi-inf.mpg.de/projects/SingleShotMultiPerson/)**.
   Human pose estimation visualization code. */
/*-----------------------------------------------------------------------------
 *      cam2pixel : convert cam type to picsel type 
 *
 *
 *  Parameters: float32_t *in, int16_t order, float32_t *out, int16_t width, int16_t height
 *
 *  Return:     Cam2pixel_t *
 *----------------------------------------------------------------------------*/
Cam2pixel_t * cam2pixel(const float32_t cam_coord[3u],const float32_t f[2u],const float32_t c[2u])
{
    Cam2pixel_t pixel;
    pixel.x = cam_coord[0u] / (cam_coord[2u] + 1e-8f) * f[0u] + c[0u];
    pixel.y = cam_coord[1u] / (cam_coord[2u] + 1e-8f) * f[1u] + c[1u];
    pixel.z = cam_coord[2u];
    return &pixel;
}
/*-----------------------------------------------------------------------------
 *      pixel2cam : convert picsel type to cam type
 *
 *
 *  Parameters: const Cam2pixel_t pixel_coord, const float32_t f[2u],const float32_t c[2u]
 *
 *  Return:     float32_t *
 *----------------------------------------------------------------------------*/
float32_t * pixel2cam(const Cam2pixel_t pixel_coord, const float32_t f[2u],const float32_t c[2u])
{
    float32_t camArr[3u];
    camArr[0u] = (pixel_coord.x - c[0u]) / f[0u] * pixel_coord.z;
    camArr[1u] = (pixel_coord.y - c[1u]) / f[1u] * pixel_coord.z;
    camArr[2u] = pixel_coord.z;
    return &camArr;
}
// better flow ROS interface
/*-----------------------------------------------------------------------------
 *      EventVisualizer_PointCloud : point cloud visualizer
 *
 *
 *  Parameters: PointCloud_t *cloud
 *
 *  Return:     PointXYZRGB_t *
 *----------------------------------------------------------------------------*/
PointXYZRGB_t * EventVisualizer_PointCloud( PointCloud_t *cloud ) 
{
    PointXYZRGB_t p;
    const uint64_t target_cloud_size = 200000ull;                               // Rviz cannot render more
    uint64_t t0;  
    int16_t color = 0xFF80DAEB;                                                 // Color: AA:RR:GG:BB
    const int16_t rate = sizeof(cloud->ev_buffer) / target_cloud_size;

    memset((void*)&p, 0, sizeof(p));                                            // initialise pointerat zero
    if (cloud != NULL)
    {
      if (!cloud->timerRef)
      {
        t0 = g_timer5_counter;
        cloud->timerRef=1u;
      }
        
      strcpy(cloud->frame_id,"/base_link");        
      if (rate > 1)                                                             // need to look at how we get the chunks of data .... && (i % rate != 0))
      {
        p.rgba = color;
        p.x = (float32_t)(240.0f - cloud->fr_x) / 200.0f;
        p.y = (float32_t) cloud->fr_y / 200.0f;
        p.z = (float32_t)(cloud->timestamp - t0) / 100.0;
      }
    }
    return &p;
    // we need to implement a ROS publish stub this->cloud_pub.publish(cloud);
}
/*
  Information taken from code originally Created by Aksel Sveier. Spring 2016
  reading RANSAC cloud
*/
/*-----------------------------------------------------------------------------
 *      EventVisualizer_PointCloud_FindFarPoint : Find point farthest away from origin
 *
 *
 *  Parameters: VectrPointCloud_t *inlierCloud
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t EventVisualizer_PointCloud_FindFarPoint( VectrPointCloud_t *inlierCloud ) 
{
    float64_t largestD = 0.0f;                                                  //Find point farthest away from origin
    float64_t thisD =0.0f;
    int8_t i;
    
    for(i = 0; i < sizeof(inlierCloud->points); i++)
    {
        thisD = sqrt(pow(inlierCloud->points[i].x , 2.0f) + pow(inlierCloud->points[i].y , 2.0f) + pow(inlierCloud->points[i].z , 2.0f));
        if(largestD < thisD)
        {
            largestD = thisD;
            /* if you need to know which cloud it is largestInd = i;  */
        }
    }
    return largestD;
}
/*-----------------------------------------------------------------------------
 *      EventVisualizer_PointCloud_FindLargestInd : Find point farthest away from origin
 *
 *
 *  Parameters: VectrPointCloud_t *inlierCloud
 *
 *  Return:     int8_t
 *----------------------------------------------------------------------------*/
int8_t EventVisualizer_PointCloud_FindLargestInd( VectrPointCloud_t *inlierCloud ) 
{
    float64_t largestD = 0.0f;                                                  //Find point farthest away from origin
    float64_t thisD =0.0f;
    int8_t i, largestInd;
    
    for(i = 0; i < sizeof(inlierCloud->points); i++)
    {
        thisD = sqrt(pow(inlierCloud->points[i].x , 2.0f) + pow(inlierCloud->points[i].y , 2.0f) + pow(inlierCloud->points[i].z , 2.0f));
        if(largestD < thisD)
        {
            largestInd = i;  
        }
    }
    return largestInd;
}
/*-----------------------------------------------------------------------------
 *      EventVisualizer_PointCloud_FindFarPointOppDir : Find the farthest point in the opposite direction.
 *      This is done by only searching in the opposite octant of the first point
 *
 *  Parameters: VectrPointCloud_t *inlierCloud
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t EventVisualizer_PointCloud_FindFarPointOppDir( VectrPointCloud_t *inlierCloud ) 
{
    float64_t smallestD = 0.0f;                                                 //Find the farthest point in the opposite direction. This is done by only searching in the opposite octant of the first point
    float64_t thisD =0.0f;
    int8_t i;
    
    int8_t largestInd = EventVisualizer_PointCloud_FindLargestInd( inlierCloud );
    for(i = 0; i < sizeof(inlierCloud->points); i++)
    {
        if((inlierCloud->points[i].x * inlierCloud->points[largestInd].x) < 0 && (inlierCloud->points[i].y * inlierCloud->points[largestInd].y) < 0 && (inlierCloud->points[i].z * inlierCloud->points[largestInd].z) < 0)
        {
            thisD = sqrt(pow(inlierCloud->points[i].x , 2.0f) + pow(inlierCloud->points[i].y , 2.0f) + pow(inlierCloud->points[i].z , 2.0f));
            if(smallestD < thisD)
            {
                smallestD = thisD;
                /* if you need to know which cloud it is smallestInd = i; */
            }
        }
    }
    return smallestD;
}

/*-----------------------------------------------------------------------------
 *      OptimizerGlobal_get_event_score : get an event score from an image
 *
 *
 *  Parameters: int16_t x, int16_t y, float32_t *project_img[], float32_t metric_wsize
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t OptimizerGlobal_get_event_score(int16_t x, int16_t y, float32_t *project_img[], float32_t metric_wsize) 
{
    // Average of nonzero
    float64_t nz_avg = 0.0f;
    uint64_t nz_avg_cnt = 0lu;
    int8_t j,i;

    for (j = x - metric_wsize / 2.0f; j <= x + metric_wsize / 2.0f; ++j) {
        for (i = y - metric_wsize / 2.0f; i <= y + metric_wsize / 2.0f; ++i) {
            if (project_img[j][i] <= 0.0f) continue;
            nz_avg_cnt ++;
            nz_avg += project_img[j][i];
        }
    }

    nz_avg = (nz_avg_cnt == 0) ? 0 : nz_avg / ((float64_t)nz_avg_cnt);
    return nz_avg;
}
/*-----------------------------------------------------------------------------
 *      to_SingleChannel : puts rgb byte stream *src to single channel byte stream output *dst
 *
 *
 *  Parameters: uint8_t *src,uint8_t *dst, int16_t src_rows, int16_t src_cols
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void to_SingleChannel(uint8_t *src,uint8_t *dst, int16_t src_rows, int16_t src_cols)
{
    int16_t j,i;
    int16_t baseJ;
    uint8_t *sptr = src;
    uint8_t *dptr = dst;
    for(i=0;i<src_rows;++i)
    {
        for(j=0;j<src_cols;++j)
        {
            baseJ = j*3;
            dptr[j] = sptr[baseJ] + sptr[baseJ+1] + sptr[baseJ+2];
        }
    }
}
/* $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
   ref : https://ieeexplore.ieee.org/document/5654327 (GRADIENT BASED THRESHOLD FREE COLOR FILTER ARRAY INTERPOLATION)
   https://raw.githubusercontent.com/RayXie29/GBTF_Color_Interpolation  (not fully implemented as need openCV port for 
   horzontal and verticle color difference gradient )   */
/*-----------------------------------------------------------------------------
 *      CalWeightingTable : calculate weighting table
 *
 *
 *  Parameters: float32_t *WeightTable, gradientMap_t *HGradientMap, gradientMap_t *VGradientMap
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void CalWeightingTable(float32_t *WeightTable, gradientMap_t *HGradientMap, gradientMap_t *VGradientMap)
{
    int16_t i,j,k,dir,a,b;
    float32_t *VGMPtr,*HGMPtr;
    int16_t NSEW[4u][2u] = { {-4,-2}, {0,-2} , {-2,-4}, {-2,0} };
    for (i = 5, k = 0; i < HGradientMap->rows - 5; ++i)
    {
        j = i%2? 6 : 5;
        for (; j < HGradientMap->cols - 5; j+=2)
        {
            for(dir=0;dir<4;++dir,++k)                                          /*  R & B pixel location */
            {
                for(a=i+NSEW[dir][0u] ; a<=i+NSEW[dir][0u]+4u ; ++a)
                {
                    *VGMPtr = VGradientMap->pfloat[a];
                    *HGMPtr = HGradientMap->pfloat[a];
                    for(b=j+NSEW[dir][1u]; b<=j+NSEW[dir][1u]+4u ;++b)
                    {
                        if(dir<2) { WeightTable[k] += VGMPtr[b]; }
                        else { WeightTable[k] += HGMPtr[b]; }
                    }
                }
                if(WeightTable[k]) { WeightTable[k] = 1.0f / (WeightTable[k]*WeightTable[k]); }
            }
        }
    }
}
/*-----------------------------------------------------------------------------
 *      interpolateGVatBRpixelLocation : Interpolate green value at blue and red pixel location
 *
 *
 *  Parameters: float32_t *WeightTable, uint8_t *Dst, gradientMap_t *TPdiff, gradientMap_t *HCDMap, gradientMap_t *VCDMap
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
uint8_t * interpolateGVatBRpixelLocation( const float32_t *WeightTable, const uint8_t *Dst, const gradientMap_t *TPdiff, const gradientMap_t *HCDMap, const gradientMap_t *VCDMap)
{   
    float32_t NSKernel[9u][9u] = { 0.0f };
    float32_t WEKernel[9u][9u] = { 0.0f };  
    float32_t Vroi[9u][9u] = { 0.0f }; 
    float32_t Hroi[9u][9u] = { 0.0f };         
    float32_t N,S,E,W,totalWeight;
    float32_t *TPPtr;
    uint8_t *DstPtr=NULL;        
    int16_t idx,i,k,j;
    int16_t channels = 3;        

    for(i=5, k=0; i<HCDMap->rows-5;++i)
    {
        j = i%2 ? 6 : 5;
        *TPPtr = TPdiff->pfloat[i-5u];
        DstPtr = ((uint8_t*)Dst+(i-5u));
        for(;j<HCDMap->cols-5; j+=2, k+=4 )
        {
            N = WeightTable[k], S = WeightTable[k+1u], E = WeightTable[k+2u] , W = WeightTable[k+3u] ;
            totalWeight = N+S+E+W;
            if(totalWeight==0.0f) { TPPtr[j-5u] = 0; }
            else
            {
                NSKernel[4u][4u] = (N+S)*0.2f;
                NSKernel[0u][4u] = NSKernel[1u][4u] = NSKernel[2u][4u] = NSKernel[3u][4u] = 0.2f*N;
                NSKernel[5u][4u] = NSKernel[6u][4u] = NSKernel[7u][4u] = NSKernel[8u][4u] = 0.2f*S;
     

                WEKernel[4u][4u] = (W+E)*0.2f;
                WEKernel[4u][0u] = WEKernel[4u][1u] = WEKernel[4u][2u] = WEKernel[4u][3u] = E*0.2f;
                WEKernel[4u][5u] = WEKernel[4u][6u] = WEKernel[4u][7u] = WEKernel[4u][8u] = W*0.2f;
     
                // cv::Mat Vroi = VCDMap(cv::Rect(j-4,i-4,9,9));    vertical color difference gradient
                // cv::Mat Hroi = HCDMap(cv::Rect(j-4,i-4,9,9));    horizontal color difference gradient

                if (sizeof(VCDMap->pfloat)>((j-4+i-4)+(9*9)))
                  memcpy((void*)&Vroi,(void*)&VCDMap->pfloat+(j-4+i-4),(9*9));                // ? check open cv Rect function !!!!!! TODO
                if (sizeof(HCDMap->pfloat)>((j-4+i-4)+(9*9)))                          
                  memcpy((void*)&Hroi,(void*)&HCDMap->pfloat+(j-4+i-4),(9*9));                // ? check Rect function !!!!!! TODO
                                
                for(idx=0;idx<9;++idx)
                {
                    TPPtr[j-5] += WEKernel[4u][idx] * Hroi[4u][idx] + NSKernel[idx][4u] * Vroi[idx][4u];
                }
                TPPtr[j-5] /= totalWeight;
            }
            if (sizeof(DstPtr)>((j-5)*channels+1))
               DstPtr[(j-5)*channels+1]= ((uint8_t)(DstPtr[(j-5)*channels+i%2+!(j%2)] + TPPtr[j-5u]));
        }
    }
    return DstPtr;
}
/*-----------------------------------------------------------------------------
 *      interpolateRBatBRpixelLocation : Interpolate R,B channel in blue and red pixel locations
 *
 *
 *  Parameters: uint8_t *Dst, gradientMap_t *TPdiff, float32_t *Prb, 
 *
 *  Return:     uint8_t *
 *----------------------------------------------------------------------------*/
uint8_t * interpolateRBatBRpixelLocation( const uint8_t *Dst, const gradientMap_t *TPdiff, const float32_t *Prb )
{
    int16_t i,j,x,y;
    uint8_t *DPtr=NULL;        
    float32_t sum;
    float32_t *PrbPtr,*TProiPtr;
    int16_t channels = 3;
    
//    cv::Mat Prb(7, 7, CV_32FC1, cv::Scalar(0));        Prb is this float stream from openCV
    for(i=3;i<TPdiff->rows-3;++i)
    {
        j = i%2 ? 4 : 3;
        DPtr = ((uint8_t*)Dst+(i-3));
        for(;j<TPdiff->cols-3;j+=2)
        {
            sum = 0;
            //cv::Mat TProi = TPdiff(cv::Rect(j-3,i-3,7,7)); check openCV library TODO:
            
            for(x=0;x<7;x+=2)
            {
                *PrbPtr = Prb[x];
                *TProiPtr = TPdiff->pfloat[x];
                for(y=0;y<7;y+=2) { sum += PrbPtr[y]* TProiPtr[y]; }
            }
            if (sizeof(DPtr)>(j-3)*channels + !(i%2)+j%2)
               DPtr[(j-3)*channels + !(i%2)+j%2u ] = ((uint8_t)(DPtr[(j-3)*channels + 1u ] - sum));
        }
    }
    return DPtr;
}  
/*-----------------------------------------------------------------------------
 *      interpolateRBonGpixelLocation : Interpolate R,B channel on G pixel
 *
 *
 *  Parameters: const uint8_t *Dst, int16_t width, int16_t height
 *
 *  Return:     uint8_t *
 *----------------------------------------------------------------------------*/
uint8_t * interpolateRBonGpixelLocation( const uint8_t *Dst, int16_t width, int16_t height)
{
    int16_t NSEW[4u][2u] = { {-1,0} , {1,0}, {0,-1}, {0,1} };
    int16_t i,j,x,y,dir;
    float32_t GBdiff,GRdiff;
    uint8_t G;
    uint8_t *DstPtr;
    const int16_t channels = 3;
        
    for(i=0;i<height;++i)
    {
        DstPtr = ((uint8_t*)Dst+i);
        j = i%2 ? 1 : 0;
        for(;j<width;j+=2)
        {
            G = DstPtr[j*channels+1];
            GBdiff=0.0f;
            GRdiff=0.0f;
            for(dir = 0; dir<4;++dir)
            {
                x = i+NSEW[dir][0u];
                y = j+NSEW[dir][1u];
                if(x>=0 && x<height && y>=0 && y<width)
                {
                    //GBdiff += Dst.at<cv::Vec3b>(x,y)[1] - Dst.at<cv::Vec3b>(x,y)[0];
                    //GRdiff += Dst.at<cv::Vec3b>(x,y)[1] - Dst.at<cv::Vec3b>(x,y)[2];
                    GBdiff += *Dst+x*(sizeof(float32_t))+y*(sizeof(float32_t))+(1*sizeof(float32_t)*3) - *Dst+x*(sizeof(float32_t))+y*(sizeof(float32_t))+(0*sizeof(float32_t)*3);
                    GRdiff += *Dst+x*(sizeof(float32_t))+y*(sizeof(float32_t))+(1*sizeof(float32_t)*3) - *Dst+x*(sizeof(float32_t))+y*(sizeof(float32_t))+(2*sizeof(float32_t)*3);
                }
            }
            DstPtr[j*channels+2u] = ((uint8_t)(G - GRdiff/4));
            DstPtr[j*channels] = ((uint8_t)(G - GBdiff/4));
        }
    }
    return DstPtr;
}
/*-----------------------------------------------------------------------------
 *      Image_distance : compares images with weight table to give factor
 *                       (factor is indication of change of state)
 *
 *  Parameters: float64_t *img1, float64_t *img2, float32_t *weight, size_t sz
 *
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t Image_distance(float64_t *img1, float64_t *img2, float32_t *weight, size_t sz)
{
  float64_t d = 0.0f;
  float64_t *ptr1 = img1;
  float64_t *ptr2 = img2;
  float32_t *w = weight;
  size_t cnt=0u;

  while(cnt <= sz) 
  {
    d += *w * (*ptr1 - *ptr2) * (*ptr1 - *ptr2);
    ptr1 ++; ptr2 ++; w ++; cnt++;
  }
  return sqrt(d);
}

/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Petri Tanskanen <tpetri@inf.ethz.ch>
 *                             Lorenz Meier <lm@inf.ethz.ch>
 *                             Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 *   Modified: Christoph Tobler <christoph@px4.io>
 *   Ported for PIC32/FT900 mikroE C by ACP Aviation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
 /*-----------------------------------------------------------------------------
 *      PX4Flow__USAD8 : computes total difference between 4 bytes (2x u32 values)
 *
 *  Parameters: uint32_t val1, uint32_t val2
 *
 *  Return:     uint32_t
 *----------------------------------------------------------------------------*/
 uint32_t PX4Flow__USAD8(uint32_t val1, uint32_t val2)
{
        uint32_t res = 0;
        uint8_t *val1_bytes = (uint8_t *)(&val1);
        uint8_t *val2_bytes = (uint8_t *)(&val2);
        int16_t v1,v2;
        int8_t i;
        
        for (i = 0; i < 4; i++) 
        {
                v1 = val1_bytes[i];
                v2 = val2_bytes[i];
                res += (uint32_t)(abs(v1 - v2));
        }

        return res;
}
 /*-----------------------------------------------------------------------------
 *      PX4Flow__USADA8 : computes total difference between 4 bytes (2x u32 values)
 *                        and sums it with val3
 *
 *  Parameters: uint32_t val1, uint32_t val2, uint32_t val3
 *
 *  Return:     uint32_t
 *----------------------------------------------------------------------------*/
uint32_t PX4Flow__USADA8(uint32_t val1, uint32_t val2, uint32_t val3)
{
        uint32_t res = val3;
        uint8_t *val1_bytes = (uint8_t *)(&val1);
        uint8_t *val2_bytes = (uint8_t *)(&val2);
        int16_t v1,v2;
        int8_t i;
        
        for (i = 0; i < 4; i++) 
        {
          v1 = val1_bytes[i];
          v2 = val2_bytes[i];
          res += (uint32_t)(abs(v1 - v2));
        }

        return res;
}
 /*-----------------------------------------------------------------------------
 *      PX4Flow__UHADD8 : computes sum between 2 bytes in a sequence of 4 and shifts
 *                        left by one bit, (divide by 2) { average byte }
 *
 *  Parameters: uint32_t val1, uint32_t val2
 *
 *  Return:     uint32_t
 *----------------------------------------------------------------------------*/
uint32_t PX4Flow__UHADD8(uint32_t val1, uint32_t val2)
{
        uint32_t res = 0;
        uint8_t *res_bytes = (uint8_t *)(&res);
        uint8_t *val1_bytes = (uint8_t *)(&val1);
        uint8_t *val2_bytes = (uint8_t *)(&val2);
        int8_t i;
        
        for (i = 0; i < 4; i++) 
        {
           res_bytes[i] = (val1_bytes[i] + val2_bytes[i]) >> 1;
        }

        return res;
}
/**
 * @brief Compute the average pixel gradient of all horizontal and vertical steps
 *
 * TODO compute_diff is not appropriate for low-light mode images
 *
 * @param image ...
 * @param offX x coordinate of upper left corner of 8x8 pattern in image
 * @param offY y coordinate of upper left corner of 8x8 pattern in image
 */
uint32_t PX4Flow_compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size)
{
        /* calculate position in image buffer */
        uint16_t off = (offY + 2u) * row_size + (offX + 2u);                    /* we calc only the 4x4 pattern  */
        uint32_t acc;
        uint32_t col1, col2, col3, col4;
        
        /* calc row diff */
        acc = PX4Flow__USAD8(*((uint32_t *) &image[off + 0u + 0u * row_size]), *((uint32_t *) &image[off + 0u + 1u * row_size]));
        acc = PX4Flow__USADA8(*((uint32_t *) &image[off + 0u + 1u * row_size]), *((uint32_t *) &image[off + 0u + 2u * row_size]), acc);
        acc = PX4Flow__USADA8(*((uint32_t *) &image[off + 0u + 2u * row_size]), *((uint32_t *) &image[off + 0u + 3u * row_size]), acc);

        /* we need to get columns */
        col1 = (image[off + 0u + 0u * row_size] << 24u) | image[off + 0u + 1u * row_size] << 16u | image[off + 0u + 2u * row_size] << 8u | image[off + 0u + 3u * row_size];
        col2 = (image[off + 1u + 0u * row_size] << 24u) | image[off + 1u + 1u * row_size] << 16u | image[off + 1u + 2u * row_size] << 8u | image[off + 1u + 3u * row_size];
        col3 = (image[off + 2u + 0u * row_size] << 24u) | image[off + 2u + 1u * row_size] << 16u | image[off + 2u + 2u * row_size] << 8u | image[off + 2u + 3u * row_size];
        col4 = (image[off + 3u + 0u * row_size] << 24u) | image[off + 3u + 1u * row_size] << 16u | image[off + 3u + 2u * row_size] << 8u | image[off + 3u + 3u * row_size];

        /* calc column diff */
        acc = PX4Flow__USADA8(col1, col2, acc);
        acc = PX4Flow__USADA8(col2, col3, acc);
        acc = PX4Flow__USADA8(col3, col4, acc);

        return acc;
}
/**
 * @brief Compute SAD distances of subpixel shift of two 8x8 pixel patterns.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 * @param acc array to store SAD distances for shift in every direction
 */
uint32_t PX4Flow_compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size)
{
        /* calculate position in image buffer */
        uint16_t off1 = off1Y * row_size + off1X;                               // image1
        uint16_t off2 = off2Y * row_size + off2X;                               // image2
        uint32_t s0, s1, s2, s3, s4, s5, s6, s7, t1, t3, t5, t7;
        uint16_t i;

        for (i = 0u; i < 8u; i++) 
        {
           acc[i] = 0u;
        }


        /*
         * calculate for each pixel in the 8x8 field with upper left corner (off1X / off1Y)
         * every iteration is one line of the 8x8 field.
         *
         *  + - + - + - + - + - + - + - + - +
         *  |   |   |   |   |   |   |   |   |
         *  + - + - + - + - + - + - + - + - +
         *
         *
         */

        for (i = 0u; i < 8u; i++) {
                /*
                 * first column of 4 pixels:
                 *
                 *  + - + - + - + - + - + - + - + - +
                 *  | x | x | x | x |   |   |   |   |
                 *  + - + - + - + - + - + - + - + - +
                 *
                 * the 8 s values are from following positions for each pixel (X):
                 *  + - + - + - +
                 *  +   5   7   +
                 *  + - + 6 + - +
                 *  +   4 X 0   +
                 *  + - + 2 + - +
                 *  +   3   1   +
                 *  + - + - + - +
                 *
                 *  variables (s1, ...) contains all 4 results (32bit -> 4 * 8bit values)
                 *
                 */

                /* compute average of two pixel values */
                s0 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 +  0u + (i + 0u) * row_size]),
                               *((uint32_t *) &image2[off2 + 1u + (i + 0u) * row_size])));
                s1 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 +  0u + (i + 1u) * row_size]),
                               *((uint32_t *) &image2[off2 + 1u + (i + 1u) * row_size])));
                s2 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 +  0u + (i + 0u) * row_size]),
                               *((uint32_t *) &image2[off2 + 0u + (i + 1u) * row_size])));
                s3 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 +  0u + (i + 1u) * row_size]),
                               *((uint32_t *) &image2[off2 - 1u + (i + 1u) * row_size])));
                s4 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 +  0u + (i + 0u) * row_size]),
                               *((uint32_t *) &image2[off2 - 1u + (i + 0u) * row_size])));
                s5 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 +  0u + (i - 1u) * row_size]),
                               *((uint32_t *) &image2[off2 - 1u + (i - 1u) * row_size])));
                s6 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 +  0u + (i + 0u) * row_size]),
                               *((uint32_t *) &image2[off2 + 0u + (i - 1u) * row_size])));
                s7 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 +  0u + (i - 1u) * row_size]),
                               *((uint32_t *) &image2[off2 + 1u + (i - 1u) * row_size])));

                /* these 4 t values are from the corners around the center pixel */
                t1 = (PX4Flow__UHADD8(s0, s1));
                t3 = (PX4Flow__UHADD8(s3, s4));
                t5 = (PX4Flow__UHADD8(s4, s5));
                t7 = (PX4Flow__UHADD8(s7, s0));

                /*
                 * finally we got all 8 subpixels (s0, t1, s2, t3, s4, t5, s6, t7):
                 *  + - + - + - +
                 *  |   |   |   |
                 *  + - 5 6 7 - +
                 *  |   4 X 0   |
                 *  + - 3 2 1 - +
                 *  |   |   |   |
                 *  + - + - + - +
                 */

                /* fill accumulation vector */
                acc[0u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 0u + i * row_size])), s0, acc[0u]);
                acc[1u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 0u + i * row_size])), t1, acc[1u]);
                acc[2u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 0u + i * row_size])), s2, acc[2u]);
                acc[3u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 0u + i * row_size])), t3, acc[3u]);
                acc[4u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 0u + i * row_size])), s4, acc[4u]);
                acc[5u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 0u + i * row_size])), t5, acc[5u]);
                acc[6u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 0u + i * row_size])), s6, acc[6u]);
                acc[7u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 0u + i * row_size])), t7, acc[7u]);

                /*
                 * same for second column of 4 pixels:
                 *
                 *  + - + - + - + - + - + - + - + - +
                 *  |   |   |   |   | x | x | x | x |
                 *  + - + - + - + - + - + - + - + - +
                 *
                 */

                s0 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 + 4u + (i + 0u) * row_size]),
                               *((uint32_t *) &image2[off2 + 5u + (i + 0u) * row_size])));
                s1 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 + 4u + (i + 1u) * row_size]),
                               *((uint32_t *) &image2[off2 + 5u + (i + 1u) * row_size])));
                s2 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i + 0u) * row_size]),
                               *((uint32_t *) &image2[off2 + 4u + (i + 1u) * row_size])));
                s3 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i + 1u) * row_size]),
                               *((uint32_t *) &image2[off2 + 3u + (i + 1u) * row_size])));
                s4 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i + 0u) * row_size]),
                               *((uint32_t *) &image2[off2 + 3u + (i + 0u) * row_size])));
                s5 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i - 1u) * row_size]),
                               *((uint32_t *) &image2[off2 + 3u + (i - 1u) * row_size])));
                s6 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i + 0u) * row_size]),
                               *((uint32_t *) &image2[off2 + 4u + (i - 1u) * row_size])));
                s7 = (PX4Flow__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i - 1u) * row_size]),
                               *((uint32_t *) &image2[off2 + 5u + (i - 1u) * row_size])));

                t1 = (PX4Flow__UHADD8(s0, s1));
                t3 = (PX4Flow__UHADD8(s3, s4));
                t5 = (PX4Flow__UHADD8(s4, s5));
                t7 = (PX4Flow__UHADD8(s7, s0));

                acc[0u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 4u + i * row_size])), s0, acc[0u]);
                acc[1u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 4u + i * row_size])), t1, acc[1u]);
                acc[2u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 4u + i * row_size])), s2, acc[2u]);
                acc[3u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 4u + i * row_size])), t3, acc[3u]);
                acc[4u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 4u + i * row_size])), s4, acc[4u]);
                acc[5u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 4u + i * row_size])), t5, acc[5u]);
                acc[6u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 4u + i * row_size])), s6, acc[6u]);
                acc[7u] = PX4Flow__USADA8((*((uint32_t *) &image1[off1 + 4u + i * row_size])), t7, acc[7u]);
        }

        return 0u;
}

/**
 * @brief Compute SAD of two 8x8 pixel windows.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 */
uint32_t PX4Flow_compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint16_t row_size)
{
        /* calculate position in image buffer */
        uint16_t off1 = off1Y * row_size + off1X; // image1
        uint16_t off2 = off2Y * row_size + off2X; // image2

        uint32_t acc;
        acc = PX4Flow__USAD8(*((uint32_t *) &image1[off1 + 0u + 0u * row_size]), *((uint32_t *) &image2[off2 + 0u + 0u * row_size]));
        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 4u + 0u * row_size]), *((uint32_t *) &image2[off2 + 4u + 0u * row_size]), acc);

        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 0u + 1u * row_size]), *((uint32_t *) &image2[off2 + 0u + 1u * row_size]), acc);
        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 4u + 1u * row_size]), *((uint32_t *) &image2[off2 + 4u + 1u * row_size]), acc);

        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 0u + 2u * row_size]), *((uint32_t *) &image2[off2 + 0u + 2u * row_size]), acc);
        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 4u + 2u * row_size]), *((uint32_t *) &image2[off2 + 4u + 2u * row_size]), acc);

        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 0u + 3u * row_size]), *((uint32_t *) &image2[off2 + 0u + 3u * row_size]), acc);
        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 4u + 3u * row_size]), *((uint32_t *) &image2[off2 + 4u + 3u * row_size]), acc);

        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 0u + 4u * row_size]), *((uint32_t *) &image2[off2 + 0u + 4u * row_size]), acc);
        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 4u + 4u * row_size]), *((uint32_t *) &image2[off2 + 4u + 4u * row_size]), acc);

        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 0u + 5u * row_size]), *((uint32_t *) &image2[off2 + 0u + 5u * row_size]), acc);
        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 4u + 5u * row_size]), *((uint32_t *) &image2[off2 + 4u + 5u * row_size]), acc);

        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 0u + 6u * row_size]), *((uint32_t *) &image2[off2 + 0u + 6u * row_size]), acc);
        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 4u + 6u * row_size]), *((uint32_t *) &image2[off2 + 4u + 6u * row_size]), acc);

        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 0u + 7u * row_size]), *((uint32_t *) &image2[off2 + 0u + 7u * row_size]), acc);
        acc = PX4Flow__USADA8(*((uint32_t *) &image1[off1 + 4u + 7u * row_size]), *((uint32_t *) &image2[off2 + 4u + 7u * row_size]), acc);

        return acc;
}
/**
 * @brief Computes pixel flow from image1 to image2
 *
 * Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1)
 * and calculates the average offset of all.
 *
 * @param image1 previous image buffer
 * @param image2 current image buffer (new)
 * @param x_rate gyro x rate
 * @param y_rate gyro y rate
 * @param z_rate gyro z rate
 *
 * @return quality of flow calculation
 */
#define PX4_TILE_SIZE        8u                                                                                // x & y tile size
#define PX4_NUM_BLOCKS        5u // x & y number of tiles to check

uint8_t PX4Flow_compute_flow(uint8_t *image1, uint8_t *image2, float32_t x_rate, float32_t y_rate, float32_t z_rate, float32_t *pixel_flow_x, float32_t *pixel_flow_y, uint32_t search_size, uint32_t image_width, uint32_t flow_feature_threshold, uint32_t flow_value_threshold)
{
        const int16_t winmin = -search_size;                                    /* constants */
        const int16_t winmax = search_size;
        const uint16_t hist_size = 2u * (winmax - winmin + 1u) + 1u;

        uint16_t pixLo = search_size + 1u;                                      /* variables */
        uint16_t pixHi = image_width - (search_size + 1u) - PX4_TILE_SIZE;
        uint16_t pixStep = (pixHi - pixLo) / PX4_NUM_BLOCKS + 1u;
        uint16_t i, j;
        uint32_t acc[8u];                                                       // subpixels
        uint16_t *histx;                                                       // counter for x shift
        uint16_t *histy;                                                       // counter for y shift
        int8_t  dirsx[64u];                                                     // shift directions in x
        int8_t  dirsy[64u];                                                     // shift directions in y
        uint8_t  subdirs[64u];                                                  // shift directions of best subpixels
        float32_t meanflowx = 0.0f;
        float32_t meanflowy = 0.0f;
        uint16_t meancount = 0u;
        float32_t histflowx = 0.0f;
        float32_t histflowy = 0.0f;
        uint32_t meancount_x, meancount_y;
        uint32_t diff,dist,temp_dist,mindist,mindir; 
        int8_t ii, jj, sumx, sumy;
        uint8_t k, hist_index_x, hist_index_y, h, qual;
        float32_t subdirx, subdiry;
        
        histx = (uint16_t*)Malloc(hist_size);                                   /* allocate pointer to array size of hist_size */
        histy = (uint16_t*)Malloc(hist_size);
        
        for (j = 0u; j < hist_size; j++) { histx[j] = 0u; histy[j] = 0u; }      /* initialize with 0 */

        /* iterate over all patterns
         */
        for (j = pixLo; j < pixHi; j += pixStep) {
                for (i = pixLo; i < pixHi; i += pixStep) {
                        diff = PX4Flow_compute_diff(image1, i, j, image_width); /* test pixel if it is suitable for flow tracking */

                        if (diff < flow_feature_threshold) {
                                continue;
                        }

                        dist = 0xFFFFFFFFul;                                    // set initial distance to "infinity"
                        sumx = 0;
                        sumy = 0;

                        for (jj = winmin; jj <= winmax; jj++) {

                                for (ii = winmin; ii <= winmax; ii++) {
                                        temp_dist = PX4Flow_compute_sad_8x8(image1, image2, i, j, i + ii, j + jj, image_width);

                                        if (temp_dist < dist) {
                                                sumx = ii;
                                                sumy = jj;
                                                dist = temp_dist;
                                        }
                                }
                        }

                        if (dist < flow_value_threshold) {                      /* acceptance SAD distance threshhold */
                                meanflowx += (float32_t) sumx;
                                meanflowy += (float32_t) sumy;

                                PX4Flow_compute_subpixel(image1, image2, i, j, i + sumx, j + sumy, acc, image_width);
                                mindist = dist;                                 // best SAD until now
                                mindir = 8u;                                    // direction 8 for no direction

                                for (k = 0u; k < 8u; k++) {
                                        if (acc[k] < mindist) {
                                                mindist = acc[k];               // SAD becomes better in direction k
                                                mindir = k;
                                        }
                                }

                                dirsx[meancount] = sumx;
                                dirsy[meancount] = sumy;
                                subdirs[meancount] = mindir;
                                meancount++;

                                hist_index_x = 2 * sumx + (winmax - winmin + 1u);  /* feed histogram filter*/

                                if (subdirs[i] == 0u || subdirs[i] == 1u || subdirs[i] == 7u) { hist_index_x += 1u; }

                                if (subdirs[i] == 3u || subdirs[i] == 4u || subdirs[i] == 5u) { hist_index_x -= 1u; }

                                hist_index_y = 2u * sumy + (winmax - winmin + 1u);

                                if (subdirs[i] == 5u || subdirs[i] == 6u || subdirs[i] == 7u) { hist_index_y -= 1u; }

                                if (subdirs[i] == 1u || subdirs[i] == 2u || subdirs[i] == 3u) { hist_index_y += 1u; }

                                histx[hist_index_x]++;
                                histy[hist_index_y]++;

                        }
                }
        }
        Free((char *)histx, hist_size);                                         /* free the memory */
        Free((char *)histy, hist_size); 
        
        if (meancount > 10u) {                                                  /* evaluate flow calculation */
                meanflowx /= meancount;
                meanflowy /= meancount;

                // int16_t maxpositionx = 0;
                // int16_t maxpositiony = 0;
                // uint16_t maxvaluex = 0;
                // uint16_t maxvaluey = 0;
                //
                // /* position of maximal histogram peek */
                // for (j = 0; j < hist_size; j++) {
                //         if (histx[j] > maxvaluex) {
                //                 maxvaluex = histx[j];
                //                 maxpositionx = j;
                //         }
                //
                //         if (histy[j] > maxvaluey) {
                //                 maxvaluey = histy[j];
                //                 maxpositiony = j;
                //         }
                // }

                /* check if there is a peak value in histogram */
                if (1) { //(histx[maxpositionx] > meancount / 6 && histy[maxpositiony] > meancount / 6)

                        meancount_x = 0u;                                       /* use average of accepted flow values */
                        meancount_y = 0u;

                        for (h = 0u; h < meancount; h++) {
                                subdirx = 0.0f;

                                if (subdirs[h] == 0u || subdirs[h] == 1u || subdirs[h] == 7u) { subdirx = 0.5f; }

                                if (subdirs[h] == 3u || subdirs[h] == 4u || subdirs[h] == 5u) { subdirx = -0.5f; }

                                histflowx += (float32_t)dirsx[h] + subdirx;
                                meancount_x++;

                                subdiry = 0.0f;

                                if (subdirs[h] == 5u || subdirs[h] == 6u || subdirs[h] == 7u) { subdiry = -0.5f; }

                                if (subdirs[h] == 1u || subdirs[h] == 2u || subdirs[h] == 3u) { subdiry = 0.5f; }

                                histflowy += (float32_t)dirsy[h] + subdiry;
                                meancount_y++;
                        }
                        histflowx /= meancount_x;
                        histflowy /= meancount_y;
                        *pixel_flow_x = histflowx;                              /* without gyro compensation */
                        *pixel_flow_y = histflowy;
                }
                else {                                                          /* no peak value in histogram */
                        *pixel_flow_x = 0.0f;
                        *pixel_flow_y = 0.0f;
                        return 0u;
                }
        }
        else {                                                                  /* no peak value in histogram */
                *pixel_flow_x = 0.0f;
                *pixel_flow_y = 0.0f;
                return 0u;
        }
               
        qual = (uint8_t)(meancount * 255u / (PX4_NUM_BLOCKS * PX4_NUM_BLOCKS));  /* calc quality */

        return qual;
}

/* This is a C port by ACP Aviaiton of the matlab function by Laurent Itti
   and part of his vision tool kit, it can be used to check camera calibration and 
   locate the right and left eye of the camera.
   
% This function is the key function that solves our calibration geometrical
% model for finding location of both eyes. this function takes in
% projection of left eye and projection of right eye on presentation screen
% relative to LEDs and returs two vectors corresponding to eye locations
% assumptions : 1. All four LEDs are on a plane arranged on the corners of a
% rectangle 2. those sides of above mentioned rectangle that connect LED #
% and LED # 2, and LED #3 and LED #4 are parallet to presentation screen 3.
% the lines connecting two eyes are parallel to presentation screen.
% 
% inputs to the function are :-
% pl : a 2x4 matrix with rows corresponding to x and y coordinate of the
% projection dots (for left eye) and column i corresponding to LED # i
% pr : a 2x4 matrix with rows corresponding to x and y coordinate of the
% projection dots (for right eye) and column i corresponding to LED # i
% k : y coordinate of bisector of two eyes
%
% returns structure Cam_Eye_t
% le : vector corresponding to location of left eye
% re : vector corresponding to location or right eye

% a is the distance between LED #1 and LED #2 in cm
% b is the distance between LED #1 and LED #4 in cm */
 /*-----------------------------------------------------------------------------
 *      Optical_locateEyes : locate positions of both camera eyes
 *                       
 *
 *  Parameters: const float32_t pl[2u][4u], const float32_t pr[2u][4u],    
 *              float32_t k, float32_t a, float32_t b, Cam_Eye_t *eye  
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Optical_locateEyes( const float32_t pl[2u][4u], const float32_t pr[2u][4u], float32_t k, float32_t a, float32_t b, Cam_Eye_t *eye)
{
   float32_t lx1 = pl[0u][0u];
   float32_t lx2 = pl[0u][1u];
   float32_t lx3 = pl[0u][2u];
   float32_t lx4 = pl[0u][3u];
   float32_t ly1 = pl[1u][0u];
   float32_t ly2 = pl[1u][1u];
   float32_t ly3 = pl[1u][1u];
   float32_t ly4 = pl[1u][3u];

   float32_t rx1 = pr[0u][0u];
   float32_t rx2 = pr[0u][1u];
   float32_t rx3 = pr[0u][2u];
   float32_t rx4 = pr[0u][3u];
   float32_t ry1 = pr[1u][0u];
   float32_t ry2 = pr[1u][1u];
   float32_t ry3 = pr[1u][1u];
   float32_t ry4 = pr[1u][3u];

   /* d=(a*(-ly1 + ry1))/(2*(a + rx1 - rx2));   what he had commented in matlab ? */
   float32_t d=(a*(-ly3 + ry3))/(2.0f*(a - rx3 + rx4));
   float32_t ecx=(rx1*rx3 - rx2*rx4)/(rx1 - rx2 + rx3 - rx4);
   float32_t ecz= sqrt(4.0f*(b*b)*((rx1 - rx2)*(rx1 - rx2))*((rx3 - rx4)*(rx3 - rx4)) - (a*a)*pow(2.0f*k*(-rx1 + rx2 - rx3 + rx4) + (rx3 - rx4)*(ly1 + ry1) +(rx1 - rx2)*(ly3 + ry3),2.0f))/(2.0f*a*sqrt(pow((rx1 - rx2 + rx3 - rx4),2.0f)));
   eye->le=mkvec(ecx,k-d,ecz);
   eye->re=mkvec(ecx,k+d,ecz);
}
 /*-----------------------------------------------------------------------------
 *      Optical_findEyes : find both camera eyes
 *                       
 *
 *  Parameters: const float32_t pl[2u][4u], const float32_t pr[2u][4u],     
 *              float32_t a, float32_t b, Cam_Eye_t *eye 
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Optical_findEyes( const float32_t pl[2u][4u], const float32_t pr[2u][4u], float32_t a, float32_t b, Cam_Eye_t *eye)
{
   float32_t lx1 = pl[0u][0u];
   float32_t lx2 = pl[0u][1u];
   float32_t lx3 = pl[0u][2u];
   float32_t lx4 = pl[0u][3u];
   float32_t ly1 = pl[1u][0u];
   float32_t ly2 = pl[1u][1u];
   float32_t ly3 = pl[1u][1u];
   float32_t ly4 = pl[1u][3u];

   float32_t rx1 = pr[0u][0u];
   float32_t rx2 = pr[0u][1u];
   float32_t rx3 = pr[0u][2u];
   float32_t rx4 = pr[0u][3u];
   float32_t ry1 = pr[1u][0u];
   float32_t ry2 = pr[1u][1u];
   float32_t ry3 = pr[1u][1u];
   float32_t ry4 = pr[1u][3u];
   
   float32_t elx = -((a*(a/(rx4 - rx3) + 1)*(lx1/(lx1 - lx2) + rx1/(rx2 - rx1)) - a*(a/(rx1 - rx2) + 1)*(lx4/(lx4 - lx3) + rx4/(rx3 - rx4)))/((a/(lx2 - lx1) - 1)*(a/(rx4 - rx3) + 1) - (a/(lx3 - lx4)- 1)*(a/(rx1 - rx2) + 1)));
   float32_t ely = -(((lx1 - lx2)*((a*(a - rx3 + rx4)*(ly1/(lx1 - lx2) - ry1/(rx1 - rx2)))/(rx3 - rx4) + (a*(a + rx1 - rx2)*(-(ly4/(lx3 - lx4)) + ry4/(rx3 - rx4)))/(rx1 - rx2)))/(a*(1 + ((lx1 - lx2)*(a - lx3 + lx4 + rx1 - rx2))/((lx3 - lx4)*(rx1 - rx2)) + (a + lx1 - lx2)/(-rx3 + rx4))));
   float32_t erx = ((a*lx4)/(lx3 - lx4) + (a*rx4)/(-rx3 + rx4) + ((-1 + a/(lx3 - lx4))*((-a)*(1 + a/(rx1 - rx2))*(lx4/(-lx3 + lx4) + rx4/(rx3 - rx4)) +  a*(lx1/(lx1 - lx2) + rx1/(-rx1 + rx2))*(1 + a/(-rx3 + rx4))))/((-(-1 + a/(lx3 - lx4)))*(1 + a/(rx1 - rx2)) + (-1 + a/(-lx1 + lx2))*(1 + a/(-rx3 + rx4))))/(1 + a/(-rx3 + rx4));
   float32_t ery=((rx1 - rx2)*(-((a*ly1)/(lx1 - lx2)) + (a*ry1)/(rx1 - rx2) - ((a + lx1 - lx2)*((a*(a - rx3 + rx4)*(ly1/(lx1 - lx2) - ry1/(rx1 - rx2)))/(rx3 - rx4) + (a*(a + rx1 - rx2)*(-(ly4/(lx3 - lx4)) + ry4/(rx3 - rx4)))/(rx1 - rx2)))/(a*(1 + ((lx1 - lx2)*(a - lx3 + lx4 + rx1 - rx2))/((lx3 - lx4)*(rx1 - rx2)) + (a + lx1 - lx2)/(-rx3 + rx4)))))/ (a + rx1 - rx2);

   float32_t y1 = (ely*(a + lx1 - lx2) - a*ly1)/(lx1 - lx2); 
   float32_t y4 = ((-ery)*(a - rx3 + rx4) + a*ry4)/(rx3 - rx4);
/* tmp = (y4 - y1) /b;
y1-y4
teta=acos((y1-y4)/b)
%tmp=0;
%sint = sqrt(1-tmp*tmp) ; */
   float32_t teta=0;
   float32_t elz = (b*(ely - ly1)*(ely - ly4)*sin(teta))/(ly4*y1 - ly1*y4 + ely*(ly1 - ly4 - y1 + y4));
   float32_t erz = (b*(ery - ry1)*(ery - ry4)*sin(teta))/(ry4*y1 - ry1*y4 + ery*(ry1 - ry4 - y1 + y4));

   eye->le=mkvec(elx,ely,elz);
   eye->re=mkvec(erx,ery,erz);
}
 /*-----------------------------------------------------------------------------
 *      makeMedianFromByteStream : make a median from the byte stream
 *                       
 *
 *  Parameters: uint8_t *sptr, int16_t w, int16_t h, uint8_t dev     
 *               
 *              
 *  Return:     uint8_t * - median stream
 *----------------------------------------------------------------------------*/
uint8_t * makeMedianFromByteStream( uint8_t *sptr, int16_t w, int16_t h, uint8_t dev )
{
  uint8_t *dptr;
  int16_t y,x;
  
  /* ++sptr;                                                                        increment one pixsel cell to start */
  for (y = 0; y < h; ++y)                                                       /* for height array elements */
    {
      if (y == 0 || y == h-1)                                                   
        for (x = 0; x < w; ++x)
            *dptr++ = *sptr++;
      else
        for (x = 1; x < w; ++x)                                                 /* starting at 2nd element, iterate through each byte until width, compare with previous and next byte in array */
          {
             if ((fabs((float64_t)*(sptr+x) - (float64_t)*(sptr+x-1u)) < dev) && (fabs((float64_t)*(sptr+x) - (float64_t)*(sptr+x+1u)) < dev)) /* rate of deviation is fine */
             {
                *dptr = *(sptr+x+1u) + *(sptr+x) + *(sptr+x+1u) / 3.0f;         /* average accross 3 cells */
             }
             else if (fabs((float64_t)*(sptr+x) - (float64_t)*(sptr+x-1u)) > dev) /* current value has deviated too much from the last one */
             {
                if (fabs((float64_t)*(sptr+x-1u) - (float64_t)*(sptr+x+1u)) < dev) /* old value and future are in deadband we got a spike */
                {
                  *(sptr+x) = *(sptr+x-1);                                      /* keep old value overwriting deviating value in input stream */
                }
                else if (fabs((float64_t)*(sptr+x) - (float64_t)*(sptr+x+1u)) < dev) /* current value and future are in deadband perhaps we are rising */
                {
                   /* --- keep value (consider is the first one a wrong choice ?) --- */
                }
                else                                                            /* all out rely on the first value */
                {
                   *(sptr+x) = *(sptr+x-1);                                     /* keep old value overwriting deviating value in input stream */
                   *(sptr+x+1u) = *(sptr+x-1);                                  /* keep old value overwriting deviating value in input stream */
                }
                *dptr = *sptr;                                                  /* write this byte to the output stream */
             }
             else if (fabs((float64_t)*(sptr+x) - (float64_t)*(sptr+x+1u)) > dev) /* the future value is too high */
             {
                if (fabs((float64_t)*(sptr+x-1u) - (float64_t)*(sptr+x+1u)) < dev) /* old value and future are in deadband we got a spike */
                {
                  *(sptr+x) = *(sptr+x-1u);                                     /* keep old value */
                  *dptr = (*(sptr+x-1u) + *(sptr+x+1u)) / 2.0f;                 /* average past value with future value as current pixsel result */
                }
                else
                {
                  *(sptr+x+1u) = *(sptr+x);                                     /* overwrite the future value as it has deviated too much */
                  *dptr = (*(sptr+x-1u) + *(sptr+x)) / 2.0f;                    /* average past value with current value as current pixsel result */
                }
             }
             ++dptr;
             /*++sptr;*/
          }
    }

  return dptr;
}
 /*-----------------------------------------------------------------------------
 *      makeMedianFromByteStream : make a median from the float stream
 *                       
 *
 *  Parameters: float32_t *sptr, int16_t w, int16_t h, float32_t dev     
 *               
 *              
 *  Return:     uint8_t * - median stream
 *----------------------------------------------------------------------------*/
float32_t * makeMedianFromFloatStream( float32_t *sptr, int16_t w, int16_t h, float32_t dev )
{
  float32_t *dptr;
  int16_t y,x;
  
  /*++sptr;                                                                        increment one pixsel cell to start */
  for (y = 0; y < h; ++y)                                                       /* for the height of the array */
    {
      if (y == 0 || y == h-1)
        for (x = 0; x < w; ++x)
            *dptr++ = *sptr++;
      else
        for (x = sizeof(float32_t); x < w; x+=sizeof(float32_t))                /* for the width of the array */
          {
             if ((fabs(*(sptr+x) - *(sptr+x-1u)) < dev) && (fabs(*(sptr+x) - *(sptr+x+1u)) < dev)) /* rate of deviation is fine */
             {
                *dptr = *(sptr+x-1u) + *(sptr+x) + *(sptr+x+1u) / 3.0f;         /* average accross 3 cells */
             }
             else if (fabs(*(sptr+x) - *(sptr+x-1u)) > dev)                     /* current value has deviated too much from the last one */
             {
                *(sptr+x) = *(sptr+x-1);                                        /* keep old value overwriting deviating value */
                *dptr = *sptr;
             }
             else if (fabs(*(sptr+x) - *(sptr+x+1u)) > dev)                     /* the future value is too high */
             {
                if (fabs(*(sptr+x-1u) - *(sptr+x+1u)) < dev)                    /* old value and future are in deadband we got a spike */
                {
                  *(sptr+x) = *(sptr+x-1u);                                     /* keep old value */
                  *dptr = (*(sptr+x-1u) + *(sptr+x+1u)) / 2.0f;                 /* average past value with future value as current pixsel result */
                }
                else
                {
                  *(sptr+x+1u) = *(sptr+x);                                     /* overwrite the future value as it has deviated too much */
                  *dptr = (*(sptr+x-1u) + *(sptr+x)) / 2.0f;                    /* average past value with current value as current pixsel result */
                }
             }
             dptr+=sizeof(float32_t);
             /* ++sptr;  */
          }
    }

  return dptr;
}

/* Copyright (c) 2014 AVBotz 
#############################################
  _____   ____ _ 
 / _ \ \ / / _` |
|  __/\ V / (_| |
 \___| \_/ \__,_|

eva - Extensible Vehicular Automaton
Copyright (C) 2014 AVBotz
Version 2014-12 Copyright (C) 2014 Luke Shimanuki

#############################################
*/
 /*-----------------------------------------------------------------------------
 *      VisionTask_generateHueMap : generates hue map from openCV matrix
 *                       
 *
 *  Parameters: cv_Mat rgb, int16_t height, int16_t width, cv_Mat hue     
 *               
 *              
 *  Return:     cv_Mat
 *----------------------------------------------------------------------------*/
cv_Mat VisionTask_generateHueMap(cv_Mat rgb, int16_t height, int16_t width, cv_Mat hue)
{
        /* this is a call to openCV library cv_Mat hue(height, width, CV_16SC1); */
        /* we are passing it here as an input to the function */
        int16_t i;
        unsigned char* rgb_data;
        int16_t* hue_data;
        unsigned char r,g,b;
        
        for (i = 0; i < height*width; i++)
        {
                rgb_data = (unsigned char*)rgb + i*3;
                hue_data = (int16_t*)hue + i*3;
                r = rgb_data[2u];
                g = rgb_data[1u];
                b = rgb_data[0u];
                
                if (r == g && g == b)
                {
                        *hue_data = -1;
                }
                if (r >= b)
                {
                        if (r >= g)
                        {
                                if (g >= b)
                                {
                                        *hue_data = 60*(g-b)/(r-b);             // red-yellow
                                }
                                else
                                {
                                        *hue_data = 360 - 60*(b-g)/(r-g);       // magenta-red
                                }
                        }
                        else
                        {
                                *hue_data = 120 - 60*(r-b)/(g-b);               // yellow-green
                        }
                }
                else
                {
                        if (b >= g)
                        {
                                if (g >= r)
                                {
                                        *hue_data = 240 - 60*(g-r)/(b-r);       // cyan-blue
                                }
                                else
                                {
                                        *hue_data = 240 + 60*(r-g)/(b-g);       // blue-magenta
                                }
                        }
                        else
                        {
                                *hue_data = 120 + 60*(b-r)/(g-r);               // green-cyan
                        }
                }
        }
        return hue;
}
/* ************************************************************************** 
   Fast cube root by Ken Turkowski
   (http://www.worldserver.com/turk/computergraphics/papers.html)
   ************************************************************************** */
 /*-----------------------------------------------------------------------------
 *      cubeRoot : cube root
 *                       
 *
 *  Parameters: float32_t value, Cv32suf *v, Cv32suf *m    
 *               
 *              
 *  Return:     float64_t
 *----------------------------------------------------------------------------*/
float64_t  cubeRoot( float32_t value, Cv32suf *v, Cv32suf *m )
{
    /* CV_INSTRUMENT_REGION(); */
    float64_t fr;
    int16_t ix, s;
    int16_t ex, shx;

    v->f = value;
    ix = (int16_t)v->i & 0x7fffffff;
    s = (int16_t)v->i & 0x80000000;
    ex = (ix >> 23u) - 127;
    shx = ex % 3;
    shx -= shx >= 0 ? 3 : 0;
    ex = (ex - shx) / 3;                                                        /* exponent of cube root */
    v->i = (ix & ((1<<23u)-1)) | ((shx + 127)<<23u);
    fr = v->f;

    /* 0.125 <= fr < 1.0 */
    /* Use quartic rational polynomial with error < 2^(-24) */
    fr = (float64_t)(((((45.2548339756803022511987494f * fr +
    192.2798368355061050458134625f) * fr +
    119.1654824285581628956914143f) * fr +
    13.43250139086239872172837314f) * fr +
    0.1636161226585754240958355063f)/
    ((((14.80884093219134573786480845f * fr +
    151.9714051044435648658557668f) * fr +
    168.5254414101568283957668343f) * fr +
    33.9905941350215598754191872f) * fr +
    1.0f));

    /* fr *= 2^ex * sign */
    m->f = value;
    v->f = (float32_t) fr;
    if ((m->i*2.0f) != 0.0f)                                                     /* what is m.i set to ? */
      v->i = (v->i + (float32_t) (ex << 23u) + (float32_t) s);
    else
      v->i = 0;
    return v->f;
}
 /*-----------------------------------------------------------------------------
 *      Pointcloud_writeGeoJSON : write point cloud as json object
 *                       
 *
 *  Parameters: char *outfile, const VectrPointCloud_t *points    
 *               
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Pointcloud_writeGeoJSON(char *outfile, const VectrPointCloud_t *points)
{
    const size_t sz = sizeof(*points);
    char charsWrit = 0;
    size_t i;
            
    outfile=strcpy(outfile,"{\n");
    outfile=strcat(outfile,"  type: \"GeometryCollection\"\,\n");
    outfile=strcat(outfile,"  geometries: [\n");
    outfile=strcat(outfile,"     {\n");
    outfile=strcat(outfile,"       type: \"MultiPoint\"\,");
    outfile=strcat(outfile,"       coordinates: [\n");
    for (i = 0; i < sz; i++)
    {
        charsWrit+=sprintf(outfile,"          [ %f, %f ],\n",outfile,points->points[i].x, points->points[i].y);
    }
    outfile=strcat(outfile,"       ]\n     }\n  ]\n}\n");
}
/*
 * ported by ACP Aviation from 
 *
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 /*-----------------------------------------------------------------------------
 *      Pointcloud_writeVrml : write point cloud as vrml scene
 *                       
 *
 *  Parameters: char *outfile, const VectrPointCloud_t *points    
 *               
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Pointcloud_writeVrml(char *outfile, const VectrPointCloud_t *points)
{
    const size_t sz = sizeof(*points);
    char charsWrit = 0;
    size_t i;
            
    outfile=strcpy(outfile,"#VRML V2.0 utf8");
    outfile=strcat(outfile,"Transform {");
    outfile=strcat(outfile,"translation 0 0 0");
    outfile=strcat(outfile,"rotation 0 0 0 0");
    outfile=strcat(outfile,"  children [");
    outfile=strcat(outfile,"     Shape{");
    outfile=strcat(outfile,"  geometry PointSet {");
    outfile=strcat(outfile,"      coord Coordinate {");
    outfile=strcat(outfile,"          point [");
    for (i = 0; i < sz; i++)
    {
        charsWrit+=sprintf(outfile,"%s\t\t%f %f %f\n",outfile,points->points[i].x, points->points[i].y, points->points[i].z);
    }
    outfile=strcat(outfile,"                 ]");
    outfile=strcat(outfile,"      }");
    outfile=strcat(outfile,"    color Color{");
    outfile=strcat(outfile,"              color [");
    for (i = 0; i < sz; i++)
    {
        outfile=strcat(outfile,"\t\t 1.0 1.0 1.0 \n");
    }
    outfile=strcat(outfile,"                 ]");
    outfile=strcat(outfile,"      }");
    outfile=strcat(outfile,"   }");
    outfile=strcat(outfile,"     }");
    outfile=strcat(outfile,"  ]");
    outfile=strcat(outfile,"}");
}
 /*-----------------------------------------------------------------------------
 *      Pointcloud_calcBBX : calculate bbx from point cloud
 *                       
 *
 *  Parameters: float64_t *lowerBound[3u], float64_t *upperBound[3u],     
 *              const VectrPointCloud_t *points 
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Pointcloud_calcBBX(float64_t *lowerBound[3u], float64_t *upperBound[3u], const VectrPointCloud_t *points )  
{
    float64_t min_x, min_y, min_z = 1e6f;
    float64_t max_x, max_y, max_z = -1e6f;
    const size_t sz = sizeof(*points);        
    float64_t x,y,z;
    size_t i;

    for (i = 0; i < sz; i++) 
    {
       x = points->points[i].x;
       y = points->points[i].y;
       z = points->points[i].z;

       if (x < min_x) min_x = x;
       if (y < min_y) min_y = y;
       if (z < min_z) min_z = z;

       if (x > max_x) max_x = x;
       if (y > max_y) max_y = y;
       if (z > max_z) max_z = z;
    }

    *lowerBound[0u] = min_x; *lowerBound[1u] = min_y; *lowerBound[2u] = min_z;
    *upperBound[0u] = max_x; *upperBound[1u] = max_y; *upperBound[2u] = max_z;
}
 /*-----------------------------------------------------------------------------
 *      Pointcloud_checkDist : check dististance of point(s) in point cloud is > thres
 *                       
 *
 *  Parameters: float64_t thres, const VectrPointCloud_t point, uint8_t operation     
 *              
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool Pointcloud_checkDist(float64_t thres, const VectrPointCloud_t point, uint8_t operation ) 
{
  switch(operation)
  {
     case 0u: /* max */
     return ( (point.points[0u].x*point.points[0u].x+point.points[0u].y*point.points[0u].y+point.points[0u].z*point.points[0u].z) > thres );
     
     case 1u: /* min */
     return ( (point.points[0u].x*point.points[0u].x+point.points[0u].y*point.points[0u].y+point.points[0u].z*point.points[0u].z) < thres ); 
     
     case 2u: /* matches */
     return ( (point.points[0u].x*point.points[0u].x+point.points[0u].y*point.points[0u].y+point.points[0u].z*point.points[0u].z) == thres );
     
     default:
     break;          
  }
  return false; 
}
 /*-----------------------------------------------------------------------------
 *      Pointcloud_crop : check that the 1st point is in the band
 *                       
 *
 *  Parameters: float64_t *lowerBound[3u], float64_t *upperBound[3u], const VectrPointCloud_t point     
 *              
 *  Return:     bool
 *----------------------------------------------------------------------------*/
bool Pointcloud_crop(float64_t *lowerBound[3u], float64_t *upperBound[3u], const VectrPointCloud_t point) 
{
      return ( (point.points[0u].x >= *lowerBound[0u]) && (point.points[0u].y >= *lowerBound[1u]) && (point.points[0u].z >= *lowerBound[2u]) && (point.points[0u].x <= *upperBound[0u]) && (point.points[0u].y <= *upperBound[1u]) && (point.points[0u].z <= *upperBound[2u]) ); 
}

 /*-----------------------------------------------------------------------------
 *      CUAV_update_stats : update statistics (iterate for each pixsel in image)
 *      ported from Matthew Ridley, March 2011                 
 *
 *  Parameters: uint16_t x, image_stats_t* s     
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void CUAV_update_stats(uint16_t x, image_stats_t* s)
{
  float32_t delta;
  if (s==NULL) return;

  s->n += 1;
  delta = (float32_t)x - s->mean;
  s->mean += delta / (float32_t)s->n;
  s->m2 += delta * ((float32_t)x - s->mean);
  s->variance = s->m2 / (float32_t)s->n;
  if (x < s->min)
  {
    s->min = x;
  }
  if (x > s->max)
  {
    s->max = x;
  }
}

 /*-----------------------------------------------------------------------------
 *      debayer_half_16u_8u_rgb : 
 *                       
 *
 *  Parameters: uint16_t* in_image, size_t in_stride, size_t in_width,      
 *              size_t in_height, uint8_t* out_image, size_t out_stride 
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void debayer_half_16u_8u_rgb(uint16_t* in_image, size_t in_stride, size_t in_width, size_t in_height, uint8_t* out_image, size_t out_stride)
{
  uint16_t* p;
  uint16_t* g0;
  uint16_t* b0;
  uint16_t* r0;
  uint16_t* g1;
  uint8_t* q;

  size_t out_x,x;
  size_t out_y,y;

  for (y = 0; y < in_height - 1; y += 2)
  {
    for (x = 0; x < in_width - 1; x += 2)
    {
      p = in_image;
      g0 = (p + in_stride * y + x);
      b0 = (p + in_stride * y + x + 1);
      r0 = (p + in_stride * (y + 1) + x);
      g1 = (p + in_stride * (y + 1) + x + 1);

      out_x = x >> 1;
      out_y = y >> 1;
      q = out_image + out_stride*CUAV_PIXEL_SIZE*out_y + out_x*CUAV_PIXEL_SIZE;

      q[0u] = (uint8_t)((*r0) >> 8u);
      q[1u] = (uint8_t)((*g0 + *g1) >> 9u);
      q[2u] = (uint8_t)((*b0) >> 8u);
    }
  }
}

 /*-----------------------------------------------------------------------------
 *      pixop_half_16u_8u_rgb : 
 *                       
 *
 *  Parameters: const uint16_t* in, size_t in_stride, uint8_t* out       
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void pixop_half_16u_8u_rgb(const uint16_t* in, size_t in_stride, uint8_t* out)
{
  uint16_t g0 = *(in);
  uint16_t b0 = *(in + 1u);
  uint16_t r0 = *(in + in_stride);
  uint16_t g1 = *(in + in_stride + 1u);

  out[0u] = (uint8_t)(r0 >> 8u);
  out[1u] = (uint8_t)(((int)g0 + (int)g1) >> 9u);
  out[2u] = (uint8_t)(b0 >> 8u);
}

 /*-----------------------------------------------------------------------------
 *      rgb_to_yuv_16u_8u : 
 *                       
 *
 *  Parameters: const uint16_t* rgb, uint8_t* yuv      
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void rgb_to_yuv_16u_8u(const uint16_t* rgb, uint8_t* yuv)
{
  int16_t i,j;
  int16_t _yuv = 0;

  for( i=0; i < 3; ++i)
  {
    _yuv = 0;
    for (j=0; j < 3; ++j)
    {
      _yuv += rgb_yuv_mat[i][j] * rgb[j];
    }
    yuv[i] = (uint8_t)(((_yuv + 32768U) >> 16U) + yuv_shift[i]);
  }
}

 /*-----------------------------------------------------------------------------
 *      pixop_half_16u_8u_yuv : 
 *                       
 *
 *  Parameters: const uint16_t* in, size_t in_stride, uint8_t* out      
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void pixop_half_16u_8u_yuv(const uint16_t* in, size_t in_stride, uint8_t* out)
{
  uint16_t rgb[3u];

  rgb[1u] = (*(in) + *(in + in_stride + 1u)) >> 1u;
  rgb[2u] = *(in + 1u);
  rgb[0u] = *(in + in_stride);

  rgb_to_yuv_16u_8u(rgb, out);
}

 /*-----------------------------------------------------------------------------
 *      debayer_half_16u_8u : 
 *                       
 *
 *  Parameters: uint16_t* in_image, size_t in_stride, size_t in_width, size_t in_height, 
 *              uint8_t* out_image, size_t out_stride,  CUAV_pixop_e pixop
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void debayer_half_16u_8u(uint16_t* in_image, size_t in_stride, size_t in_width, size_t in_height, uint8_t* out_image, size_t out_stride,  CUAV_pixop_e pixop)
{
  size_t y,x;
  uint16_t* p;
  size_t out_x,out_y;
  uint8_t* q;

  for (y = 0; (y < in_height - 1); y += 2)
  {
    for (x = 0; (x < in_width - 1); x += 2)
    {
      p = in_image + in_stride * y + x;
      out_x = x >> 1u;
      out_y = y >> 1u;

      q = out_image + out_stride*CUAV_PIXEL_SIZE*out_y + out_x*CUAV_PIXEL_SIZE;
      switch (pixop)
      {
         case PIXOP_YUV:
         pixop_half_16u_8u_yuv(p, in_stride, q);
         break;

         case PIXOP_RGB:
         pixop_half_16u_8u_rgb(p, in_stride, q);
         break;

         default:
         break;
      } 
    }
  }
}

 /*-----------------------------------------------------------------------------
 *      pixop_2x2_16u_8u_rgb :                       
 *
 *  Parameters: const uint16_t* in, size_t in_stride, uint8_t* out, size_t out_stride 
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void pixop_2x2_16u_8u_rgb(const uint16_t* in, size_t in_stride, uint8_t* out, size_t out_stride)
{
  int16_t i,r,g,b,dy,dx;
  int16_t tmp[4u][3u] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  int16_t cnt[4u][3u] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  int16_t n = sizeof(bayertab)/sizeof(CUAV_bayer_t);
  int16_t shift[5u] = {8, 8, 9, 9, 10};

  for (i=0; i < n; ++i)
  {
    int idx = bayertab[i].py << 1 | bayertab[i].px;
    tmp[idx][bayertab[i].ch] += *(in + in_stride*(bayertab[i].py + bayertab[i].dy) + bayertab[i].px + bayertab[i].dx);
    cnt[idx][bayertab[i].ch] += 1;
  }
  for (i=0; i < 4; ++i)
  {
    dy = pix_y[i];
    dx = pix_x[i];

    r = tmp[i][0u] >> shift[cnt[i][0u]];
    g = tmp[i][1u] >> shift[cnt[i][1u]];
    b = tmp[i][2u] >> shift[cnt[i][2u]];

    *(out + out_stride*dy + dx*CUAV_PIXEL_SIZE ) = (uint8_t)r;
    *(out + out_stride*dy + dx*CUAV_PIXEL_SIZE + 1) = (uint8_t)g;
    *(out + out_stride*dy + dx*CUAV_PIXEL_SIZE + 2) = (uint8_t)b;
  }
}

 /*-----------------------------------------------------------------------------
 *      pixop_2x2_16u_8u_yuv :                       
 *
 *  Parameters: const uint16_t* in, size_t in_stride, uint8_t* out, size_t out_stride 
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void pixop_2x2_16u_8u_yuv(const uint16_t* in, size_t in_stride, uint8_t* out, size_t out_stride)
{
  int16_t i,idx,dy,dx;
  int16_t tmp[4u][3u] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  int16_t cnt[4u][3u] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  int16_t n = sizeof(bayertab)/sizeof(CUAV_bayer_t);
  int16_t shift[5u] = {0, 0, 1, 1, 2};
  uint16_t rgb16[3u];
  uint8_t* yuv8;

  for (i=0; i < n; ++i)
  {
    idx = bayertab[i].py << 1 | bayertab[i].px;
    tmp[idx][bayertab[i].ch] += *(in + in_stride*(bayertab[i].py + bayertab[i].dy) + bayertab[i].px + bayertab[i].dx);
    cnt[idx][bayertab[i].ch] += 1;
  }
  for (i=0; i < 4; ++i)
  {
    dy = pix_y[i];
    dx = pix_x[i];
    rgb16[0u] = tmp[i][0u] >> shift[cnt[i][0u]];
    rgb16[1u] = tmp[i][1u] >> shift[cnt[i][1u]];
    rgb16[2u] = tmp[i][2u] >> shift[cnt[i][2u]];
    yuv8 = out + out_stride*dy + dx*CUAV_PIXEL_SIZE;
    rgb_to_yuv_16u_8u(rgb16, yuv8);
  }
}

 /*-----------------------------------------------------------------------------
 *      debayer_full_16u_8u :                       
 *
 *  Parameters: uint16_t* in_image, size_t in_stride, size_t in_width, size_t in_height, 
 *              uint8_t* out_image, size_t out_stride, pixop_2x2_16u_8u pixop
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void debayer_full_16u_8u(uint16_t* in_image, size_t in_stride, size_t in_width, size_t in_height, uint8_t* out_image, size_t out_stride, CUAV_pixop_e pixop)
{
        size_t y,x;
        uint16_t* p;
        uint8_t* q;

        for (y = 2; y < in_height - 3; y += 2)  
        {
                for (x = 2; x < in_width - 3; x += 2) 
                {
                        p = in_image + in_stride * y + x;
                        q = out_image + y*out_stride + x*CUAV_PIXEL_SIZE;
                        switch(pixop)
                        {
                           case PIXOP_YUV:                           
                              pixop_2x2_16u_8u_yuv(p, in_stride, q, out_stride);
                           break;

                           case PIXOP_RGB:                           
                              pixop_2x2_16u_8u_rgb(p, in_stride, q, out_stride);
                           break;

                           default:
                           break;
                        }
                }
        }
}
 /*-----------------------------------------------------------------------------
 *      function ported from R code by dankelley
 *      https://github.com/dankelley/camera/blob/master/proj.R                       
 *
 *      image2Coord : image to co-ordinate 
 *  Parameters: float64_t i, float64_t j, float64_t elevation, float64_t altitude, 
 *              float64_t azimuth, float64_t fov, uint32_t imgWdth, uint32_t imgHt 
 *  for example WxH 2240x1680 for image from            
 *  Return:     image_coord_t
 *----------------------------------------------------------------------------*/
image_coord_t image2Coord( float64_t i, float64_t j, float64_t elevation, float64_t altitude, float64_t azimuth, float64_t fov, uint32_t imgWdth, uint32_t imgHt)
{
    float64_t dppx = fov / imgWdth;                                             /* degrees per pixel in x and y directions (i and j indices) */
    float64_t dppy = fov / imgHt;
    float64_t theta = altitude + (j - imgHt / 2.0f) * dppy;                     /* x and y from geometry   */
    float64_t y = -elevation / tan(theta * PI / 180.0f);
    float64_t x = y * tan(dppx * (i - imgWdth / 2.0f) * PI / 180.0f);
    float64_t C = cos(-azimuth * PI / 180.0f);                                  /* rotate for azimuth */
    float64_t S = sin(-azimuth * PI / 180.0f);
    float64_t cameraLat = 44.634719170f;                                        /* set this for you're camera */
    float64_t cameraLon = -63.5717949270f; 
    float64_t R[2u][2u],rBind[2u];
    image_coord_t retVal;                                                       /* structure containing information on picture latitude and longditude */

    R[0u][0u] = C;
    R[0u][1u] = -S;
    R[1u][0u] = S;
    R[1u][1u] = C;
    rBind[0u] = x;
    rBind[1u] = y;
    
    /* === original R code ====
    R = matrix(c(C, -S, S, C), nrow=2, byrow=TRUE) 
    xy <- R %*% rbind(x, y)
    x <- xy[1,]
    y <- xy[2,]      */
    retVal.x = ((R[0u][0u] * rBind[0u]) + (R[0u][1u] * rBind[1u]));
    retVal.y = ((R[1u][0u] * rBind[0u]) + (R[1u][1u] * rBind[1u]));   
    retVal.i = i;
    retVal.j = j;

    /* predLat <- cameraLat + pred$y / 111e3
       predLon <- cameraLon + pred$x / 111e3 / cos(cameraLat * pi / 180)    
       list(i=i, j=j, x=x, y=y) */
       
    retVal.lat = cameraLat + retVal.y / 111000.0f;
    retVal.lon = cameraLon + retVal.x / 111000.0f / cos(cameraLat * PI / 180.0f);
    
    return retVal;
}           
 /*-----------------------------------------------------------------------------
 *      function ported from :-
 *      https://topic.alibabacloud.com/a/opencv-edge-font-colorreddetectionfont-reberts-sobel-prewitt-kirsch_8_8_32059025.html                      
 *
 *      kirsch : Based on the symmetry of the direction, the Kirsch operator  
 *      can process only the first four templates to obtain the maximum value.
 *      This method is the best method to calculate the pixel 
 *      gradient around the central pixel.
 * 
 *  Parameters: uint8_t *ps, uint8_t *pd, const int16_t W, const int16_t H,  
 *              const int16_t step 
 *              
 *  Return: void 
 *----------------------------------------------------------------------------*/           
void kirsch(uint8_t *ps, uint8_t *pd, const int16_t W, const int16_t H, const int16_t step) 
{
       /* dst = cvCloneImage (src);  
       cvConvert (src, srcMat);  convert the image into a matrix to process */ 
       int16_t x, y; 
       float32_t a, b, c, d; 
       float32_t p1, p2, p3, p4, p5, p6, p7, p8, p9; 
       /* uint8_t * ps = (uchar *) src-> imageData;  ps is the pointer to the input image data. 
       uint8_t * pd = (uchar *) dst-> imageData;  pd is the pointer to the output image data 
       int16_t W = dst-> width; 
       int16_t H = dst-> height; 
       int16_t step = dst-> widthStep; */ 
       for (x = 0; x <W-2; x ++)                                                /* take (x + 1, y + 1) 9 neighboring pixels Center 1 4 7 */
       {                                                                        /* 2 5 8 */
          for (y = 0; y <H-2; y ++)                                             /* 3 6 9 */ 
          {
            p1 = ps[y * step + x]; 
            p2 = ps[y * step + (x + 1)]; 
            p3 = ps[y * step + (x + 2)]; 
            p4 = ps[(y + 1) * step + x]; 
            p5 = ps[(y + 1) * step + (x + 1)]; 
            p6 = ps[(y + 1) * step + (x + 2)]; 
            p7 = ps[(y + 2) * step + x]; 
            p8 = ps[(y + 2) * step + (x + 1)]; 
            p9 = ps[(y + 2) * step + (x + 2)];                         
            a = fabs(-5.0f * p1-5.0f * p2-5.0f * p3 + 3.0f * p4 + 3.0f * p6 + 3.0f * p7 + 3.0f * p8 + 3.0f * p9 ); /* obtain (I + 1, j + 1) gray around nine points */ 
            b = fabs(3.0f * p1-5.0f * p2-5.0f * p3 + 3.0f * p4-5.0f * p6 + 3.0f * p7 + 3.0f * p8 + 3.0f * p9); /* calculate the gradient value of 4 directions */
            c = fabs(3.0f * p1 + 3.0f * p2-5.0f * p3 + 3.0f * p4-5.0f * p6 + 3.0f * p7 + 3.0f * p8-5.0f * p9); 
            d = fabs(3.0f * p1 + 3.0f * p2 + 3.0f * p3 + 3.0f * p4-5.0f * p6 + 3.0f * p7-5.0f * p8-5.0f * p9); 
            a = max(a, b);                                                      /* take the maximum value in each direction as the edge strength */ 
            a = max(a, c); 
            a = max(a, d); 
            pd [(y + 1) * step + (x + 1)] = a; 
            if (a > 127) 
            {
              pd [(y + 1) * step + (x + 1)] = 255;
            } 
            else 
            {
              pd [(y + 1) * step + (x + 1)] = 0; 
            }
         }                 
       }
}
/* 
   next 6 function ported from ref : https://github.com/keineahnung2345/digital-image-processing-cpp/blob/master/CH3_pixel_operation.cpp
*/

 /*-----------------------------------------------------------------------------
 *      LinTran :   linear transformation                    
 *
 *  Parameters: uint8_t *srcimg, uint8_t *outimg, const int16_t rows, 
 *              const int16_t cols, float64_t dFa, float64_t dFb
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/   
void LinTran( uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols, float64_t dFa, float64_t dFb )
{
    /* p.73
    if(type2str(img.type()) != "8UC1") return false; */
    int16_t i,j;
    float64_t val=0.0f;
        
    for(i = 0; i < rows; i++)
    {
        for(j = 0; j < cols; j++)
        {
            val = (float64_t) srcimg[i + j];
            val =  (val * dFa + dFb);
            outimg[i + j] = min(max((int16_t)val, 0), 255); 
        }
    }
}

 /*-----------------------------------------------------------------------------
 *      LogTran :   log transformation                    
 *
 *  Parameters: uint8_t *srcimg, uint8_t *outimg, const int16_t rows, 
 *              const int16_t cols, float64_t dC 
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void LogTran(uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols, float64_t dC)
{
    /* p.75
    if(type2str(img.type()) != "8UC1") return false; */
    int16_t i,j;
    float64_t val=0.0f;
        
    for(i = 0; i < rows; i++)
    {
        for(j = 0; j < cols; j++)
        {
            val = (float64_t) srcimg[i + j]; /*  alternate *(srcimg+i+j) */
            val = dC * log(val+1.0f);
            outimg[i + j] = min(max((int16_t)val, 0), 255); 
        }
    }
}

 /*-----------------------------------------------------------------------------
 *      LogTran :   log transformation                    
 *
 *  Parameters: uint8_t *srcimg, float64_t *outimg, const int16_t rows, 
 *              const int16_t cols, float64_t gamma, float64_t comp 
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void GammaTran(uint8_t *srcimg, float64_t *outimg, const int16_t rows, const int16_t cols, float64_t gamma, float64_t comp)
{
    /* p.79
    gamma can be 0.75, 1, 1.5...
    if(type2str(img.type()) != "8UC1") return false; */
    int16_t i,j;
    float64_t val=0.0f;
        
    for(i = 0; i < rows; i++)
    {
        for(j = 0; j < cols; j++)
        {
            val = (float64_t) srcimg[i + j];                                    /*  alternate *(srcimg+i+j) */
            val += comp;                                                        /* compensate */
            val /= 255.0f;                                                      /* normalize */
            val = pow(val, gamma);
            val *= 255.0f;                                                      /* denormalize */
            outimg[i + j] = val;
        }
    }
}

 /*-----------------------------------------------------------------------------
 *      Threshold :   apply threshold                   
 *
 *  Parameters: uint8_t *srcimg, uint8_t *outimg, const int16_t rows,  
 *              const int16_t cols, const int16_t nThres 
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void Threshold(uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols, const int16_t nThres)
{
    /*p.82
    if(type2str(img.type()) != "8UC1") return false; */
    int16_t i,j;
    float64_t val=0.0f;
        
    for(i = 0; i < rows; i++)
    {
        for(j = 0; j < cols; j++)
        {
            outimg[i + j] = ((int16_t)srcimg[i + j] < nThres) ? 0 : 255;
        }
    }
}

 /*-----------------------------------------------------------------------------
 *      ParLinTran :   apply partial linear transform                   
 *
 *  Parameters: uint8_t *srcimg, uint8_t *outimg, const int16_t rows,  
 *              const int16_t cols, const int16_t x1, const int16_t x2,  
 *              const float64_t y1, const float64_t y2 
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void ParLinTran(uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols, const int16_t x1, const int16_t x2, const float64_t y1, const float64_t y2)
{
    /*p.89
    if(type2str(img.type()) != "8UC1") return false; */
    int16_t i,j;
    float64_t val=0.0f, slope=0.0f;
        
    for(i = 0; i < rows; i++)
    {
        for(j = 0; j < cols; j++)
        {
            val = (float64_t) srcimg[i + j];
            if(val < x1)
            {
                slope = y1/x1;
                val *= slope;
            }
            else if(val < x2)
            {
                //x1 <= val < x2
                slope = (y2-y1)/(x2-x1);
                val = (val-x1) * slope + y1;
            }
            else
            {
                //val >= x2
                slope = (255-y2)/(255-x2);
                val = (val-x2) * slope + y2;
            }
            outimg[i + j] = min(max((uint8_t)val, 0), 255); 
            /* img.at<uchar>(i, j) = val; */
        }
    }
}
 /*-----------------------------------------------------------------------------
 *      HorMirror :   horizontally mirror the picture                  
 *
 *  Parameters: uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols  
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void HorMirror(uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols)
{
    int16_t r,c;                                                                
        
    for(r = 0; r < rows; r++)
    {
        for(c = 0; c < cols; c++)
        {
           outimg[r + c] = srcimg[r + (cols - c)];
        }
    }
}
 /*-----------------------------------------------------------------------------
 *      VertMirror :   vertically mirror the picture                  
 *
 *  Parameters: uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols  
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void VertMirror(uint8_t *srcimg, uint8_t *outimg, const int16_t rows, const int16_t cols)
{
    int16_t r,c;                                                                
        
    for(r = 0; r < rows; r++)
    {
        for(c = 0; c < cols; c++)
        {
           outimg[r + c] = srcimg[(rows - r)+c];
        }
    }
}

/*-----------------------------------------------------------------------------
 *  filter_interpolate_colors : filter interpolating color into rgb float
 *
 *  Parameters: int16_t *c1, int16_t *c2, int16_t *c3, float64_t *res, float64_t t 
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/  
void filter_interpolate_colors(int16_t *c1, int16_t *c2, int16_t *c3, float64_t *res, float64_t t) 
{
    float64_t r = 0.0f, g = 0.0f, b = 0.0f;
    if (t <= 0.5f)
    {
        r = (c1[0u] * (0.5f - t) * 2.0f + c2[0u] * t * 2.0f);
        g = (c1[1u] * (0.5f - t) * 2.0f + c2[1u] * t * 2.0f);
        b = (c1[2u] * (0.5f - t) * 2.0f + c2[2u] * t * 2.0f);
    }
    else
    {
        r = (c2[0u] * (1.0f - t) * 2.0f + c3[0u] * (t - 0.5f) * 2.0f);
        g = (c2[1u] * (1.0f - t) * 2.0f + c3[1u] * (t - 0.5f) * 2.0f);
        b = (c2[2u] * (1.0f - t) * 2.0f + c3[2u] * (t - 0.5f) * 2.0f);
    }
    res[0u] = FMIN(r, 255);
    res[1u] = FMIN(g, 255);
    res[2u] = FMIN(b, 255);
}
#if defined(REQUIRE_VOXEL_CODEC)
/*
   these translations ported from 
   https://github.com/ethz-asl/voxblox/blob/0ceaf9f8889754cc6edecf3ee67f942a3dbcce8e/voxblox/src/core/block.cc
*/
/*-----------------------------------------------------------------------------
 *  TsdfVoxel_deserializeFromIntegers : create voxel from stream words
 *
 *  Parameters: uint32_t *dataV, TsdfVoxel_t **voxel 
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/
void TsdfVoxel_deserializeFromIntegers( uint32_t *dataV, TsdfVoxel_t **voxel )
{
   /*
   uint32_t bytes_1;
   uint32_t bytes_2; 
   */
   uint32_t bytes_3;
   uint16_t data_idx;
   size_t eleLen = (sizeof(dataV)/sizeof(dataV[0u]));
   const size_t kNumDataPacketsPerVoxel = 3u;
   
   for (data_idx=0u;data_idx<eleLen;data_idx=+kNumDataPacketsPerVoxel)
   {
   /*           
      bytes_1 = dataV[data_idx];
      bytes_2 = dataV[data_idx + 1u];
      memcpy(&voxel[(data_idx)/3u]->distance, &bytes_1, sizeof(bytes_1));
      memcpy(&(voxel[(data_idx)/3u]->weight), &bytes_2, sizeof(bytes_2)); 
   */
      
      bytes_3 = dataV[data_idx + 2u];
      voxel[(data_idx)/3u]->distance = dataV[data_idx];
      voxel[(data_idx)/3u]->weight = dataV[data_idx + 1u];
          
      voxel[(data_idx)/3u]->color.r = ((uint8_t)(bytes_3 >> 24ul));
      voxel[(data_idx)/3u]->color.g = ((uint8_t)((bytes_3 & 0x00FF0000ul) >> 16ul));
      voxel[(data_idx)/3u]->color.b = ((uint8_t)((bytes_3 & 0x0000FF00ul) >> 8ul));
      voxel[(data_idx)/3u]->color.a = ((uint8_t)(bytes_3 & 0x000000FFul));
   } 
}
/*-----------------------------------------------------------------------------
 *  OccupancyVoxel_deserializeFromIntegers : create voxel from stream words
 *
 *  Parameters: uint32_t *dataV, OccupancyVoxel_t **voxel 
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/
void OccupancyVoxel_deserializeFromIntegers( uint32_t *dataV, OccupancyVoxel_t **voxel )
{
   /* uint32_t bytes_1; */
   uint32_t bytes_2;
   uint16_t data_idx;
   size_t eleLen = (sizeof(dataV)/sizeof(dataV[0u]));
   
   for (data_idx=0u;data_idx<eleLen;data_idx=+2u)
   {           
      /* bytes_1 = dataV[data_idx]; */
      bytes_2 = dataV[data_idx + 1u];

      /* memcpy(&voxel[(data_idx)/2u]->probability_log, &bytes_1, sizeof(bytes_1)); */

      voxel[(data_idx)/2u]->probability_log = dataV[data_idx];
      voxel[(data_idx)/2u]->observed = ((uint8_t)(bytes_2 & 0x000000FFul));
   } 
}

/*-----------------------------------------------------------------------------
 *  Voxel_deserializeDirection : create voxel from stream words
 *
 *  Parameters: const uint32_t dataV, Voxel_Vectr_t *parent_direction 
 *   
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t Voxel_deserializeDirection( const uint32_t dataV, Voxel_Vectr_t *parent_direction ) 
{
  if ((parent_direction == NULL) || (dataV == NULL))
  {
          return -1;
  }
  else
  {
     parent_direction->x = ((int8_t)max(INT8_MAX,(min(INT8_MIN,((dataV >> 24u) & 0x000000FFul)))));
     parent_direction->y = ((int8_t)max(INT8_MAX,(min(INT8_MIN,((dataV >> 16u) & 0x000000FFul)))));
     parent_direction->z = ((int8_t)max(INT8_MAX,(min(INT8_MIN,((dataV >> 8u) & 0x000000FFul)))));        
     return 1;         
  }
}

/*-----------------------------------------------------------------------------
 *  EsdfVoxel_deserializeFromIntegers : create voxel from stream words
 *
 *  Parameters: uint32_t *dataV, EsdfVoxel_t **voxel 
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/
void EsdfVoxel_deserializeFromIntegers( uint32_t *dataV, EsdfVoxel_t **voxel )
{
   /* uint32_t bytes_1; */
   uint32_t bytes_2;
   uint16_t data_idx;
   size_t eleLen = (sizeof(dataV)/sizeof(dataV[0u]));
   
   for (data_idx=0u;data_idx<eleLen;data_idx=+2u)
   {           
      /* bytes_1 = dataV[data_idx]; */
      bytes_2 = dataV[data_idx + 1u];

      /* memcpy(&voxel[(data_idx)/2u]->distance, &bytes_1, sizeof(bytes_1)); */
      voxel[(data_idx)/2u]->distance = dataV[data_idx];
          
      /* individual states 
          voxel[(data_idx)/2u]->observed = ((uint8_t)((bytes_2 & 0x00000001)==1u));
      voxel[(data_idx)/2u]->hallucinated = ((uint8_t)((bytes_2 & 0x00000002)==1u));
      voxel[(data_idx)/2u]->in_queue = ((uint8_t)((bytes_2 & 0x00000004)==1u));
      voxel[(data_idx)/2u]->fixed = ((uint8_t)((bytes_2 & 0x00000008)==1u));
      */
      voxel[(data_idx)/2u]->state_block = ((uint8_t)(bytes_2 & 0x00000015ul));

      if (Voxel_deserializeDirection(bytes_2, &voxel[(data_idx)/2u]->parent) != 1)
          break;
   }  
}

/*-----------------------------------------------------------------------------
 *  IntensityVoxel_deserializeFromIntegers : create voxel from stream words
 *
 *  Parameters: uint32_t *dataV, BlockIntensityVoxel_t **voxel 
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/
void IntensityVoxel_deserializeFromIntegers( uint32_t *dataV, BlockIntensityVoxel_t **voxel )
{
   /*
   uint32_t bytes_1;
   uint32_t bytes_2;
   */
   uint16_t data_idx;
   size_t eleLen = (sizeof(dataV)/sizeof(dataV[0u]));
   
   for (data_idx=0u;data_idx<eleLen;data_idx=+2u)
   {
/*           
      bytes_1 = dataV[data_idx];
      bytes_2 = dataV[data_idx + 1u];
      memcpy(&voxel[(data_idx )/2u]->intensity, &bytes_1, sizeof(bytes_1));
      memcpy(&(voxel[(data_idx)/2u]->weight), &bytes_2, sizeof(bytes_2));
*/
      voxel[(data_idx )/2u]->intensity = dataV[data_idx];
      voxel[(data_idx)/2u]->weight = dataV[data_idx + 1u];
   } 
}

/*-----------------------------------------------------------------------------
 *  TsdfVoxel_serializeToIntegers : create stream words from voxel data
 *
 *  Parameters: uint32_t *dataV, TsdfVoxel_t **voxel 
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/
void TsdfVoxel_serializeToIntegers( uint32_t *dataV, TsdfVoxel_t **voxel ) 
{
   size_t data_idx;
   size_t eleLen = (sizeof(voxel)/sizeof(voxel[0u]));
   
   for (data_idx=0u;data_idx<eleLen;data_idx=+3u)
   {           
      dataV[data_idx] = voxel[(data_idx)/3u]->distance;
      dataV[data_idx + 1u] = voxel[(data_idx)/3u]->weight;
      dataV[data_idx + 2u] = (((uint32_t)(voxel[(data_idx)/3u]->color.a)) | ((((uint32_t)(voxel[(data_idx)/3u]->color.b)) << 8u) | ((((uint32_t)(voxel[(data_idx)/3u]->color.g)) << 16u) | (((uint32_t)(voxel[(data_idx)/3u]->color.r)) << 24u))));
   } 
}

/*-----------------------------------------------------------------------------
 *  OccupancyVoxel_serializeToIntegers : create stream words from voxel data
 *
 *  Parameters: uint32_t *dataV, OccupancyVoxel_t **voxel 
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/
void OccupancyVoxel_serializeToIntegers( uint32_t *dataV, OccupancyVoxel_t **voxel ) 
{
   size_t data_idx;
   size_t eleLen = (sizeof(voxel)/sizeof(voxel[0u]));
   
   for (data_idx=0u;data_idx<eleLen;data_idx=+2u)
   {           
      dataV[data_idx] = voxel[(data_idx)/2u]->probability_log;
      dataV[data_idx + 1u] = voxel[(data_idx)/2u]->observed;
   } 
}

/*-----------------------------------------------------------------------------
 *  Voxel_serializeDirection : create stream words from voxel 
 *
 *  Parameters: const uint32_t dataV, Voxel_Vectr_t *parent_direction 
 *   
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t Voxel_serializeDirection( const Voxel_Vectr_t *parent_direction, uint32_t *dataV ) 
{
  if ((parent_direction == NULL) || (dataV == NULL))
  {
          return -1;
  }
  else
  {
     *dataV |= (uint32_t)((parent_direction->x) << 24u);                        /* Layout  3x8bit (int8_t) for parent (X,Y,Z) | 8bit ESDF */
     *dataV |= (uint32_t)((parent_direction->y) << 16u);
     *dataV |= (uint32_t)((parent_direction->z) << 8u);         
     return 1;         
  }
}

/*-----------------------------------------------------------------------------
 *  EsdfVoxel_serializeToIntegers : create stream words from voxel data
 *
 *  Parameters: uint32_t *dataV, EsdfVoxel_t **voxel 
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/
void EsdfVoxel_serializeToIntegers( uint32_t *dataV, EsdfVoxel_t **voxel ) 
{
   size_t data_idx;
   uint32_t bytes_2 = 0u;
   uint8_t flag_byte = 0b00000000;
   size_t eleLen = (sizeof(voxel)/sizeof(voxel[0u]));
   
   for (data_idx=0u;data_idx<eleLen;data_idx=+2u)
   {           
      dataV[data_idx] = voxel[(data_idx)/2u]->distance;

      if (Voxel_serializeDirection(&voxel[(data_idx)/2u]->parent, &bytes_2) != 1)
          break;
      flag_byte |= (uint8_t)(voxel[(data_idx)/2u]->observed ? 0b00000001 : 0b00000000);
      flag_byte |= (uint8_t)(voxel[(data_idx)/2u]->hallucinated ? 0b00000010 : 0b00000000);
      flag_byte |= (uint8_t)(voxel[(data_idx)/2u]->in_queue ? 0b00000100 : 0b00000000);
      flag_byte |= (uint8_t)(voxel[(data_idx)/2u]->fixed ? 0b00001000 : 0b00000000);

      bytes_2 |= (((uint32_t)(flag_byte)) & 0x000000FFul);
        
      dataV[data_idx + 1u] = bytes_2;
   } 
}

/*-----------------------------------------------------------------------------
 *  IntensityVoxel_serializeToIntegers : create stream words from a voxel 
 *
 *  Parameters: uint32_t *dataV, BlockIntensityVoxel_t **voxel 
 *   
 *  Return: void
 *----------------------------------------------------------------------------*/
void IntensityVoxel_serializeToIntegers( uint32_t *dataV, BlockIntensityVoxel_t **voxel )
{
   uint16_t data_idx;
   size_t eleLen = (sizeof(dataV)/sizeof(dataV[0u]));
   
   for (data_idx=0u;data_idx<eleLen;data_idx=+2u)
   {
      dataV[data_idx] = voxel[(data_idx )/2u]->intensity;
      dataV[data_idx + 1u] = voxel[(data_idx)/2u]->weight;
   } 
}
#endif /* --- voxel codec used ---- */
// Also called intermeans
// Iterative procedure based on the isodata algorithm [T.W. Ridler, S. Calvard, Picture
// thresholding using an iterative selection method, IEEE Trans. System, Man and
// Cybernetics, SMC-8 (1978) 630-632.]
// The procedure divides the image into objects and background by taking an initial threshold,
// then the averages of the pixels at or below the threshold and pixels above are computed.
// The averages of those two values are computed, the threshold is incremented and the
// process is repeated until the threshold is larger than the composite average. That is,
//  threshold = (average background + average objects)/2
// The code in ImageJ that implements this function is the getAutoThreshold() method in the ImageProcessor class.
//
// From: Tim Morris (dtm@ap.co.umist.ac.uk)
// Subject: Re: Thresholding method?
// posted to sci.image.processing on 1996/06/24
// The algorithm implemented in NIH Image sets the threshold as that grey
// value, G, for which the average of the averages of the grey values
// below and above G is equal to G. It does this by initialising G to the
// lowest sensible value and iterating:

// L = the average grey value of pixels with intensities < G
// H = the average grey value of pixels with intensities > G
// is G = (L + H)/2?
// yes => exit
// no => increment G and repeat
//
// There is a discrepancy with IJ because they are slightly different methods
int16_t AutoThresholder_IsoData( int16_t *dataVec ) 
{
        int16_t i, l, toth, totl, h, g=0;
        for (i = 1; i < sizeof(dataVec); i++){
                if (dataVec[i] > 0){
                                g = i + 1;
                                break;
                }
        }
        while (true){
                l = 0;
                totl = 0;
                for (i = 0; i < g; i++) {
                        totl = totl + dataVec[i];
                        l = l + (dataVec[i] * i);
                }
                h = 0;
                toth = 0;
                for (i = g + 1; i < sizeof(dataVec); i++){
                        toth += dataVec[i];
                        h += (dataVec[i]*i);
                }
                if (totl > 0 && toth > 0){
                        l /= totl;
                        h /= toth;
                        if (g == (int16_t) ROUNDSINT((l + h) / 2.0f))
                                break;
                }
                g++;
                if (g >sizeof(dataVec)-2){
                        return -1;
                }
        }
        return g;
}
//
// Implements Li's Minimum Cross Entropy thresholding method
// This implementation is based on the iterative version (Ref. 2) of the algorithm.
// 1) Li C.H. and Lee C.K. (1993) "Minimum Cross Entropy Thresholding"
//    Pattern Recognition, 26(4): 617-625
// 2) Li C.H. and Tam P.K.S. (1998) "An Iterative Algorithm for Minimum
//    Cross Entropy Thresholding"Pattern Recognition Letters, 18(8): 771-776
// 3) Sezgin M. and Sankur B. (2004) "Survey over Image Thresholding
//    Techniques and Quantitative Performance Evaluation" Journal of
//    Electronic Imaging, 13(1): 146-165
//    http://citeseer.ist.psu.edu/sezgin04survey.html
// Ported to ImageJ plugin by G.Landini from E Celebi's fourier_0.8 routines
//
int16_t AutoThresholder_Li( int16_t *dataVectr ) 
{

                int16_t threshold;
                int16_t ih;
                int16_t num_pixels;
                int16_t sum_back;                                               /* sum of the background pixels at a given threshold */
                int16_t sum_obj;                                                /* sum of the object pixels at a given threshold */
                int16_t num_back;                                               /* number of background pixels at a given threshold */
                int16_t num_obj;                                                /* number of object pixels at a given threshold */
                float64_t old_thresh;
                float64_t new_thresh;
                float64_t mean_back;                                            /* mean of the background pixels at a given threshold */
                float64_t mean_obj;                                             /* mean of the object pixels at a given threshold */
                float64_t mean;                                                 /* mean gray-level in the image */
                float64_t tolerance;                                            /* threshold tolerance */
                float64_t temp;

                tolerance=0.5f;
                num_pixels = 0;
                for (ih = 0; ih < sizeof(dataVectr); ih++ )
                        num_pixels += dataVectr[ih];

                mean = 0.0f;                                                    /* Calculate the mean gray-level */
                for ( ih = 0; ih < sizeof(dataVectr); ih++ ) //0 + 1?
                        mean += ih * dataVectr[ih];
                mean /= num_pixels;
                /* Initial estimate */
                new_thresh = mean;

                do{
                        old_thresh = new_thresh;
                        threshold = (int16_t) (old_thresh + 0.5f);        /* range */
                        /* Calculate the means of background and object pixels */
                        /* Background */
                        sum_back = 0;
                        num_back = 0;
                        for ( ih = 0; ih <= threshold; ih++ ) {
                                sum_back += ih * dataVectr[ih];
                                num_back += dataVectr[ih];
                        }
                        mean_back = ( num_back == 0 ? 0.0f : ( sum_back / ( float64_t ) num_back ) );
                        /* Object */
                        sum_obj = 0;
                        num_obj = 0;
                        for ( ih = threshold + 1; ih < sizeof(dataVectr); ih++ ) {
                                sum_obj += ih * dataVectr[ih];
                                num_obj += dataVectr[ih];
                        }
                        mean_obj = ( num_obj == 0 ? 0.0f : ( sum_obj / ( float64_t ) num_obj ) );

                        /* Calculate the new threshold: Equation (7) in Ref. 2 */
                        //new_thresh = simple_round ( ( mean_back - mean_obj ) / ( Math.log ( mean_back ) - Math.log ( mean_obj ) ) );
                        //simple_round ( float64_t x ) {
                        // return ( int ) ( IS_NEG ( x ) ? x - .5 : x + .5 );
                        //}
                        //
                        //#define IS_NEG( x ) ( ( x ) < -DBL_EPSILON )
                        //DBL_EPSILON = 2.220446049250313E-16
                        temp = ( mean_back - mean_obj ) / ( log ( mean_back ) - log ( mean_obj ) );

                        if (temp < -DBL_EPSILON)
                                new_thresh = (int16_t) (temp - 0.5f);
                        else
                                new_thresh = (int16_t) (temp + 0.5f);
                        /*  Stop the iterations when the difference between the
                        new and old threshold values is less than the tolerance */
                }
                while ( fabs ( new_thresh - old_thresh ) > tolerance );
                return threshold;
}
/* 
  these came from
  https://github.com/mkfreeman/triangulate/blob/master/src/ColorUtils.js
  https://github.com/mkfreeman/triangulate/blob/2032660f0dc6e04e57e629981560049528e61fe9/src/PolygonUtils.js#L19
*/

 /*-----------------------------------------------------------------------------
 *  PolygonUtils_getImageOffset : 
 *                       
 *
 *  Parameters: float32_t xin,float32_t  yin, int16_t width, int16_t height     
 *               
 *              
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t PolygonUtils_getImageOffset(const float32_t xin, const float32_t  yin, const int16_t width, const int16_t height) 
{
    float32_t x = ROUND(xin,0.0f);
    float32_t y = ROUND(yin,0.0f);
    if (x < 0)
       x = 0;
    if (y < 0)
       y = 0;
    if (x >= width)
       x = width - 1.0f;
   if (y >= height)
       y = height - 1.0f;
    return (4.0f * (x + y * width));
}

 /*-----------------------------------------------------------------------------
 *  PolygonUtils_area : 
 *                       
 *
 *  Parameters: float32_t **polygon     
 *               
 *              
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t PolygonUtils_area(float32_t **polygon) 
{
    float32_t area = 0.0f;
    size_t n = sizeof(polygon);
    size_t j = n - 1;
    size_t i;
    for (i = 0; i < n; j = i++) 
    {
        area += polygon[i][0u] * polygon[j][1u];
        area -= polygon[i][1u] * polygon[j][0u];
    }
    return (area /= 2.0f);
}

 /*-----------------------------------------------------------------------------
 *  PolygonUtils_centroid : 
 *                       
 *
 *  Parameters: float32_t **polygon     
 *               
 *              
 *  Return:     f32point_t
 *----------------------------------------------------------------------------*/        
f32point_t PolygonUtils_centroid(float32_t **polygon) 
{
    f32point_t pt;
    size_t i;
    size_t n = sizeof(polygon);
    float32_t x = 0.0f;
    float32_t y = 0.0f;
    float32_t sixA,tmp;
    size_t j = n - 1u;
    for (i = 0; i < n; ++i) 
    {
        j = ++j % INT16_MAX;
        tmp = polygon[i][0u] * polygon[j][1u] - polygon[j][0u] * polygon[i][1u];
        x += (polygon[i][0u] + polygon[j][0u]) * tmp;
        y += (polygon[i][1u] + polygon[j][1u]) * tmp;
    }
    sixA = PolygonUtils_area(polygon) * 6.0f;
    pt.x = x / sixA;
    pt.y = y / sixA;
    return pt;
}

 /*-----------------------------------------------------------------------------
 *  utils_getAverageColor : 
 *                       
 *
 *  Parameters: float32_t *c, mat33 * p, int16_t width, int16_t height,     
 *  uint8_t *imageBuffer8, colOptions_t opt, float32_t threshold, voxel_rgba_color_t *avg              
 *              
 *  Return: char* 
 *----------------------------------------------------------------------------*/        
char* utils_getAverageColor( const float32_t *c, const mat44 * p, const int16_t width, const int16_t height, const uint8_t *imageBuffer8, const colOptions_t opt, const float32_t threshold, voxel_rgba_color_t *avg) 
 {
    size_t p_length = 4;  
    size_t pp;
    char color[32u];
    float32_t r = 0.0f;
    float32_t g = 0.0f;
    float32_t b = 0.0f;
    float32_t a = 0.0f;
    float32_t y = 0.0f;
    bool test;
    float32_t offsetFlt = ROUND(PolygonUtils_getImageOffset(c[0u], c[1u], width, height),0.0f);
    uint16_t offset = ((uint16_t)offsetFlt);
    
    strncpy((char *)&color,"                                ",32u);
    if ((c==NULL) || ((p==NULL) || (imageBuffer8==NULL))) 
    {
       strcpy((char*)&color,"error");
       return color;
    }        
    if ((p_length % 2u) != 0u) p_length -= 1u;                                  /* length must be even */
    r = (((float32_t)p_length) * ((float32_t)imageBuffer8[offset]));
    g = (((float32_t)p_length) * ((float32_t)imageBuffer8[offset + 1u]));
    b = (((float32_t)p_length) * ((float32_t)imageBuffer8[offset + 2u]));
    a = (((float32_t)p_length) * ((float32_t)imageBuffer8[offset + 3u]));

    for (pp=0; pp<p_length; pp+=2u) 
    {
        offsetFlt = ROUND(PolygonUtils_getImageOffset(p->m[pp][0u], p->m[pp][1u], width, height),0.0f);
        offset = ((uint16_t)offsetFlt);
        r += ((float32_t)imageBuffer8[offset]);
        g += ((float32_t)imageBuffer8[offset + 1u]);
        b += ((float32_t)imageBuffer8[offset + 2u]);
        a += ((float32_t)imageBuffer8[offset + 3u]);
    }
    r /= (2.0f * p_length);
    g /= (2.0f * p_length);
    b /= (2.0f * p_length);
    a /= (2.0f * p_length);
    avg->r = (uint8_t)r;
    avg->g = (uint8_t)g;
    avg->b = (uint8_t)b;
    avg->a = (uint8_t)a;
        
    if (opt.blackWhite == true) 
    {
        y = 0.2126f * r + 0.7152f * g + 0.0722f * b;
        if ((opt.invert == false) && (y < threshold))
        {
                strcpy((char*)&color,"black");
        }
        else if ((opt.invert == false) && (y > threshold))
        {
                strcpy((char*)&color,"white");
        }
        else if ((opt.invert == true) && (y > threshold))
        {
                strcpy((char*)&color,"black");
        }
        else if ((opt.invert == true) && (y < threshold))
        {
                strcpy((char*)&color,"white");
        }
        else
        {
                strcpy((char*)&color,"gray ");
        }                        
    }
    else
    {
        sprintf((char*)&color,"R=%u G=%u B=%u A=%u",ROUND(r,0.0f),ROUND(g,0.0f),ROUND(b,0.0f),ROUND(a,0.0f));
    }
    return color;
}
 /*-----------------------------------------------------------------------------
 *  utils_getColorAtPos : 
 *                       
 *
 *  Parameters: float32_t *pt, int16_t width, int16_t height, const uint8_t *imageBuffer8,      
 *  const colOptions_t opt, const float32_t threshold, voxel_rgba_color_t *avg              
 *              
 *  Return: char* 
 *----------------------------------------------------------------------------*/
char* utils_getColorAtPos( float32_t *pt, int16_t width, int16_t height, const uint8_t *imageBuffer8, const colOptions_t opt, const float32_t threshold, voxel_rgba_color_t *avg ) 
{
    char color[32u];
    float32_t y;
    float32_t offsetFlt = PolygonUtils_getImageOffset(pt[0u], pt[1u], width, height);
    int16_t offset = ((uint16_t)offsetFlt);
    avg->r = imageBuffer8[offset];
    avg->g = imageBuffer8[offset + 1u];
    avg->b = imageBuffer8[offset + 2u];
    avg->a = imageBuffer8[offset + 3u];
    strncpy((char *)&color,"                                ",32u);
    if (opt.blackWhite == true)                                                 // Calculate luminence
    {
        y = 0.2126f * avg->r + 0.7152f * avg->g + 0.0722f * avg->b;
        if ((opt.invert == false) && (y < threshold))
        {
            strcpy((char*)&color,"black");
        }
        else if ((opt.invert == false) && (y > threshold))
        {
            strcpy((char*)&color,"white");
        }
        else if ((opt.invert == true) && (y > threshold))
        {
            strcpy((char*)&color,"black");
        }
        else if ((opt.invert == true) && (y < threshold))
        {
            strcpy((char*)&color,"white");
        }
        else
        {
            strcpy((char*)&color,"gray ");
        }                        
    }
    else
    {
        sprintf((char*)&color,"R=%u G=%u B=%u A=%u",avg->r,avg->g,avg->b,avg->a);
    }
    return color;
}
/*-----------------------------------------------------------------------------
 *  utils_getColor_centroid:  
 *
 *  Parameters: float32_t **polygon, int16_t width, int16_t height, const uint8_t 
 *              *imageBuffer8, const colOptions_t opt, const float32_t threshold, 
 *              voxel_rgba_color_t *avg
 *
 *  Return:     char*
 *----------------------------------------------------------------------------*/
char* utils_getColor_centroid(float32_t **polygon, int16_t width, int16_t height, const uint8_t *imageBuffer8, const colOptions_t opt, const float32_t threshold, voxel_rgba_color_t *avg ) 
{
    f32point_t f32pt;
    float32_t point[2u];
    f32pt = PolygonUtils_centroid(polygon);
    point[0u] = f32pt.x;
    point[1u] = f32pt.y;
    return utils_getColorAtPos( &point[0u], width, height, imageBuffer8, opt, threshold, avg ); 
}

/*-----------------------------------------------------------------------------
 *  utils_getColor:  
 *
 *  Parameters: float32_t **polygon, const mat44 * p, const int16_t width, 
 *              const int16_t height, const uint8_t *imageBuffer8, const colOptions_t opt, 
 *              const float32_t threshold, voxel_rgba_color_t *avg
 *
 *  Return:     char*
 *----------------------------------------------------------------------------*/
char* utils_getColor(float32_t **polygon, const mat44 * p, const int16_t width, const int16_t height, const uint8_t *imageBuffer8, const colOptions_t opt, const float32_t threshold, voxel_rgba_color_t *avg )
{
    float32_t point[2u];
    f32point_t f32pt;
    f32pt = PolygonUtils_centroid(polygon);
    point[0u] = f32pt.x;
    point[1u] = f32pt.y;        
    return utils_getAverageColor( &point[0u], p, width, height, imageBuffer8, opt, threshold, avg);
}
/*-----------------------------------------------------------------------------
 *  utils_getDotColor:  
 *
 *  Parameters: const float32_t *pt, const float32_t r, const int16_t width, const int16_t height, 
 *              const uint8_t *imageBuffer8, const colOptions_t opt, const float32_t threshold, 
 *              voxel_rgba_color_t *avg 
 *
 *  Return:     char*
 *----------------------------------------------------------------------------*/            
char* utils_getDotColor( const float32_t *pt, const float32_t r, const int16_t width, const int16_t height, const uint8_t *imageBuffer8, const colOptions_t opt, const float32_t threshold, voxel_rgba_color_t *avg ) 
{
    mat44 ptm;                                                                  /* we use 4x4 to hold four x,y co-ordinates */
    
    ptm.m[0u][0u] = pt[0u] + r;
    ptm.m[0u][1u] = pt[1u];
    ptm.m[1u][0u] = pt[0u] - r,
    ptm.m[1u][1u] = pt[1u];
    ptm.m[2u][0u] = pt[0u];
    ptm.m[2u][1u] = pt[1u] + r;
    ptm.m[3u][0u] = pt[0];
    ptm.m[3u][1u] = pt[1u] - r;

    return utils_getAverageColor(pt, &ptm, width, height, imageBuffer8, opt, threshold, avg);
}
/* 
   ported from this Golang project by ACP Aviation 
   https://github.com/esimov/triangle/blob/master/delaunay.go
*/
/*-----------------------------------------------------------------------------
 *  delaunay_point_isEq:  
 *
 *  Parameters: const f32point_t n, const f32point_t p 
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool delaunay_point_isEq(const f32point_t n, const f32point_t p) 
{
    float64_t dx = n.x - p.x;
    float64_t dy = n.y - p.y;

    if (dx < 0.0f) {
      dx = -dx;
    }
    if (dy < 0.0f) {
      dy = -dy;
    }
    if ((dx < 0.0001f) && (dy < 0.0001f)) 
    {
        return true;
    }
    return false;
}
/*-----------------------------------------------------------------------------
 *  delaunay_edge_isEq:  check if two edge are approximately equals.
 *
 *  Parameters: const f32edge_t e, const f32edge_t edge 
 *
 *  Return: bool
 *----------------------------------------------------------------------------*/
bool delaunay_edge_isEq(const f32edge_t e, const f32edge_t edge) 
{
    if (((delaunay_point_isEq(e.nodes[0u], edge.nodes[0u])) && (delaunay_point_isEq(e.nodes[1u], edge.nodes[1u]))) || ((delaunay_point_isEq(e.nodes[0u], edge.nodes[1u])) && (delaunay_point_isEq(e.nodes[1u], edge.nodes[0u]))))
    {
        return true;
    }
    return false;
}
/*-----------------------------------------------------------------------------
 *  delaunay_new_edge:  create new edge
 *
 *  Parameters: const f32point_t p0, const f32point_t p1 
 *
 *  Return: f32edge_t
 *----------------------------------------------------------------------------*/
f32edge_t delaunay_new_edge( const f32point_t p0, const f32point_t p1 )
{
   f32edge_t e;
   e.nodes[0u] = p0;
   e.nodes[1u] = p1;
   return e;
}
/*-----------------------------------------------------------------------------
 *  delaunay_new_triangle:  create new triangle
 *
 *  Parameters: const f32point_t p0, const f32point_t p1, const f32point_t p2 
 *
 *  Return: f32triangle_t
 *----------------------------------------------------------------------------*/
f32triangle_t delaunay_new_triangle( const f32point_t p0, const f32point_t p1, const f32point_t p2 )
{
   f32triangle_t t;
   float64_t m,u,s,ax,ay,bx,by,dx,dy;
   t.nodes[0u] = p0;
   t.nodes[1u] = p1;
   t.nodes[2u] = p2; 
   t.edges[0u] = delaunay_new_edge(p0,p1);
   t.edges[1u] = delaunay_new_edge(p1,p2);
   t.edges[2u] = delaunay_new_edge(p2,p0);
   
   ax = p1.x-p0.x;
   ay = p1.y-p0.y;
   bx = p2.x-p0.x;
   by = p2.y-p0.y;
   m = p1.x*p1.x - p0.x*p0.x + p1.y*p1.y - p0.y*p0.y;
   u = p2.x*p2.x - p0.x*p0.x + p2.y*p2.y - p0.y*p0.y;
   s = 1.0f / (2.0f * ((ax*by) - (ay*bx))); 

   t.circle.x = ((p2.y-p0.y)*m+(p0.y-p1.y)*u) * s;
   t.circle.y = ((p0.x-p2.x)*m+(p1.x-p0.x)*u) * s;

   dx = p0.x - t.circle.x;                                                      // Calculate the distance between the node points and the triangle circumcircle.
   dy = p0.y - t.circle.y;
   t.circle.radius = ((int16_t)(dx*dx + dy*dy));                                     // Calculate the circle radius.
    
   return t;
}
/*-----------------------------------------------------------------------------
 *  delaunay_clear:  create new delaunay
 *
 *  Parameters: f32delaunay_t *d
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void delaunay_clear(f32delaunay_t *d)
{
    f32point_t p0,p1,p2,p3;
    p0.x = 0.0f;
    p0.y = 0.0f;
    p1.x = d->width;
    p1.y = 0.0f;
    p2.x = d->width;
    p2.y = d->height;
    p3.x = 0.0f;
    p3.y = d->height;
    d->triangles[0u] = delaunay_new_triangle(p0, p1, p2);
    d->triangles[1u] = delaunay_new_triangle(p0, p2, p3);
}
float64_t geometry_slope(float64_t x0, float64_t y0, float64_t x1, float64_t y1) 
{
	return (y0 - y1) / (x0 - x1);
}

float64_t geometry_y_intercept(float64_t x0, float64_t y0, float64_t x1, float64_t y1) 
{
	return -(geometry_slope(x0, y0, x1, y1) * x0 - y0);
}

float64_t geometry_y_intercept_m(float64_t x, float64_t y, float64_t m) 
{
	return -(m * x - y);
}

/* y = mx + b */
float64_t geometry_slope_intercept(float64_t x, float64_t y, float64_t m, float64_t b) 
{
	return m * x + b;
}

/* This code was originally written by Soren Sandmann (soren.sandmann@gmail.com),
 * and permission was granted to use it for any reason. Thanks! 
 * ref :- https://github.com/cubicool/cairou/blob/master/src/cairou-spline.c
 */
 
f64point_t cairou_spline_private_before(
	f64point_t p0,
	f64point_t p1,
	f64point_t p2
) {
	f64point_t r;

	r.x = p1.x - 1.0 / 6.0 * (p2.x - p0.x);
	r.y = p1.y - 1.0 / 6.0 * (p2.y - p0.y);

	return r;
}

f64point_t cairou_spline_private_after(
	f64point_t p0,
	f64point_t p1,
	f64point_t p2
) {
	f64point_t r;

	r.x = p1.x + 1.0 / 6.0 * (p2.x - p0.x);
	r.y = p1.y + 1.0 / 6.0 * (p2.y - p0.y);

	return r;
}

f64point_t cairou_spline_private_quadratic(
	float64_t t,
	f64point_t p0,
	f64point_t p1,
	f64point_t p2
) {
	f64point_t r;

	r.x = (1.0 - t) * (1.0 - t) * p0.x + 2.0 * (1.0 - t) * t * p1.x + t * t * p2.x;
	r.y = (1.0 - t) * (1.0 - t) * p0.y + 2.0 * (1.0 - t) * t * p1.y + t * t * p2.y;

	return r;
}

#define CAIROU_GET_POINT(x) (points[(x) % n_points])

static void cairou_spline_private_get_neighbors(
	f64point_t* points,
	int16_t n_points,
	int16_t closed,
	int16_t i,
	f64point_t* this1,
	f64point_t* prev,
	f64point_t* next
) {
    float64_t t;
	f64point_t c;
	
	/* For the first and last point of a non-closed spline, we make up 'previous'
	 * and 'next' points by extrapolating a cairou_spline_private_quadratic spline. */
	if(!i) {
		if(prev) {
			if(closed) *prev = CAIROU_GET_POINT(n_points - 1);

			else {
				t = -1.0 / 4.0;

				c = cairou_spline_private_before(
					CAIROU_GET_POINT(0),
					CAIROU_GET_POINT(1),
					CAIROU_GET_POINT(2)
				);

				*prev = cairou_spline_private_quadratic(t, CAIROU_GET_POINT(0), c, CAIROU_GET_POINT(1));
			}
		}

		if(next) *next = CAIROU_GET_POINT(1);
	}

	else if(i == n_points - 1) {
		if(prev) *prev = CAIROU_GET_POINT(i - 1);

		if(next) {
			if(closed) *next = CAIROU_GET_POINT(0);

			else {
				t = 5.0 / 4.0;

				c = cairou_spline_private_after(
					CAIROU_GET_POINT(i - 2),
					CAIROU_GET_POINT(i - 1),
					CAIROU_GET_POINT(i)
				);

				*next = cairou_spline_private_quadratic(
					t,
					CAIROU_GET_POINT(n_points - 2),
					c,
					CAIROU_GET_POINT(n_points - 1)
				);
			}
		}
	}

	else {
		if(prev) *prev = CAIROU_GET_POINT(i - 1);

		if(next) *next = CAIROU_GET_POINT(i + 1);
	}

	if(this1) *this1 = CAIROU_GET_POINT(i);
}

bool cairou_append_spline(
	f64point_t* points,
	int16_t n_points,
	f64point_t *c1,
	f64point_t *c2,
	bool closed
) {
    int16_t i;
    f64point_t prevprev;
    f64point_t prev;
    f64point_t next;
    f64point_t this1;
		
    if(n_points < 3) return false;

    for(i = 1; i < n_points + closed ? 1 : 0; i++) 
    {
		cairou_spline_private_get_neighbors(
			points,
			n_points,
			closed,
			i,
			&this1,
			&prev,
			&next
		);

		cairou_spline_private_get_neighbors(
			points,
			n_points,
			closed,
			i - 1,
			NULL,
			&prevprev,
			NULL
		);

		*c1 = cairou_spline_private_after(prevprev, prev, this1);
		*c2 = cairou_spline_private_before(prev, this1, next);
	}

	return true;
}
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif   /* end of the colorHelper library */