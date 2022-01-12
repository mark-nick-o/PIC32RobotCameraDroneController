/*
 * canon.c - Canon protocol "native" operations.
 *
 * Written 1999 by Wolfgang G. Reissnegger and Werner Almesberger
 * Additions 2000 by Philippe Marzouk and Edouard Lafargue
 * USB support, 2000, by Mikael Nystroem
 *
 * This file includes both USB and serial support for the cameras
 * manufactured by Canon. These comprise all (or at least almost all)
 * of the digital models of the IXUS and PowerShot series, and EOS
 * D30, D60, and 10D. The EOS-1D and EOS-1Ds are not supported; they
 * use a FireWire (IEEE 1394) interface.
 *
 * We are working at moving serial and USB specific stuff to serial.c
 * and usb.c, keeping the common protocols/busses support in this
 * file.
 *
 * Ported by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
 */

#define _DEFAULT_SOURCE
//#include "config.h"
#include "definitions.h"

//#include <gphoto2/gphoto2.h>
//#include <gphoto2/gphoto2-port-log.h>

//#include "usb.h"
//#include "util.h"
//#include "library.h"
#include "canon.h"
   /*  OUT FOR NOW $$$ #include "types.h"  */
// #include "g_photo2.h"
//#include "serial.h"

#ifdef HAVE_LIBEXIF
#  include <libexif/exif-data.h>
#  include <libexif/exif-utils.h>
#endif

/************************************************************************
 * Camera definitions
 ************************************************************************/
/**
 * models:
 *
 * Contains list of all camera models currently supported with their
 * respective USB IDs and a serial ID String for cameras with RS232
 * serial support.
 *
 * Some cameras are sold under different names in different regions,
 * but are technically the same. We treat them the same from a
 * technical point of view. To avoid unnecessary questions from users,
 * we add the other names to the camera list after the primary name,
 * such that their camera name occurs in the list of supported
 * cameras.
 *
 * Notes:
 * - At least some serial cameras require a certain name for correct
 *   detection.
 * - Newer Canon USB cameras also support a PTP mode. See ptp2 camlib.
 * - No IEEE1394 cameras supported yet.
 * - The size limit constants aren't used properly anywhere. We should
 *   probably get rid of them altogether.
 **/

/* SL_* - size limit constants */
#define KILOBYTE  (1024U)
#define MEGABYTE  (1024U * KILOBYTE)
#define SL_THUMB  ( 100U * KILOBYTE)
#define SL_THUMB_CR2 (  10U * MEGABYTE)                                         /* same as regular image */
#define SL_PICTURE  (  10U * MEGABYTE)
#define SL_MOVIE_SMALL ( 100U * MEGABYTE)
#define SL_MOVIE_LARGE (2048U * MEGABYTE)
#define NO_USB  0

/* Models with unknown USB ID's:
  European name          North American                 Japanese             Intro date
  PowerShot A520                                                             January 2005
  Digital IXUS 40        PowerShot SD300                IXY Digital 50       September 2004
  PowerShot Pro1                                                             February 2004
                         PowerShot SD30                 IXY i zoom           August 2005
                         PowerShot A410                                      August 2005
                         PowerShot A610                                      August 2005
                         PowerShot A620                                      August 2005
  Digital IXUS 55        PowerShot SD450                                     August 2005
  Digital IXUS 750       PowerShot SD550                                     August 2005
                         PowerShot S80                                       August 2005
  Digital IXUS Wireless  PowerShot SD430                                     August 2005
  */

const struct canonCamModelData models[] = {
        /* *INDENT-OFF* */
        {"Canon:PowerShot A5",        CANON_CLASS_3,(uint16_t) NO_USB,(uint16_t) NO_USB, CAP_NON,(uint32_t) SL_MOVIE_SMALL,(uint32_t) SL_THUMB,(uint32_t) SL_PICTURE, "DE300 Canon Inc."},
        {"Canon:PowerShot A5 Zoom",        CANON_CLASS_3,        (uint16_t) NO_USB, (uint16_t) NO_USB, CAP_NON, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE, "Canon PowerShot A5 Zoom"},
        {"Canon:PowerShot A50",                CANON_CLASS_1,        (uint16_t) NO_USB, (uint16_t) NO_USB, CAP_NON, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE, "Canon PowerShot A50"},
        {"Canon:PowerShot Pro70",        CANON_CLASS_2,        (uint16_t) NO_USB, (uint16_t) NO_USB, CAP_NON, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE, "Canon PowerShot Pro70"},
        {"Canon:PowerShot S10",                CANON_CLASS_0,        (uint16_t) 0x04A9, (uint16_t) 0x3041, CAP_NON, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE, "Canon PowerShot S10"},
        /* 3042 is a scanner, so it will never be added here. */
        {"Canon:PowerShot S20",         CANON_CLASS_0,  (uint16_t) 0x04A9, 0x3043, CAP_NON, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE, "Canon PowerShot S20"},
        {"Canon:EOS D30",               CANON_CLASS_4,  (uint16_t) 0x04A9, 0x3044, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot S100 (2000)", CANON_CLASS_0,  (uint16_t) 0x04A9, 0x3045, CAP_NON, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:IXY DIGITAL",           CANON_CLASS_0,  (uint16_t) 0x04A9, 0x3046, CAP_NON, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS",          CANON_CLASS_0,  (uint16_t) 0x04A9, 0x3047, CAP_NON, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot G1",          CANON_CLASS_0,  (uint16_t) 0x04A9, 0x3048, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE, "Canon PowerShot G1"},
        {"Canon:PowerShot Pro90 IS",    CANON_CLASS_0,  (uint16_t) 0x04A9, 0x3049, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE, "Canon PowerShot Pro90 IS"},
        {"Canon:IXY DIGITAL 300",       CANON_CLASS_1,  (uint16_t) 0x04A9, 0x304B, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot S300",        CANON_CLASS_1,  (uint16_t) 0x04A9, 0x304C, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS 300",      CANON_CLASS_1,  (uint16_t) 0x04A9, 0x304D, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A20",         CANON_CLASS_1,  (uint16_t) 0x04A9, 0x304E, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A10",         CANON_CLASS_1,  (uint16_t) 0x04A9, 0x304F, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* Mac OS includes this as a valid ID; don't know which camera model --swestin */
        {"Canon:PowerShot unknown 1",   CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3050, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* Canon IXY DIGITAL 200 here? */
        {"Canon:PowerShot S110 (2001)", CANON_CLASS_0,  (uint16_t) 0x04A9, 0x3051, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS v",        CANON_CLASS_0,  (uint16_t) 0x04A9, 0x3052, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot G2",          CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3055, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot S40",         CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3056, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot S30",         CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3057, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A40",         CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3058, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A30",         CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3059, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* 305a is the ZR50 Digital Camcorder in USB Mass Storage mode. */
        /* 305b is the ZR45MC Digital Camcorder in USB Mass Storage mode. */
        /* 305c is in MacOS Info.plist, but I don't know what it is --swestin. */
        {"Canon:PowerShot unknown 2",   CANON_CLASS_1,  (uint16_t) 0x04A9, 0x305c, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:EOS D60",               CANON_CLASS_4,  (uint16_t) 0x04A9, 0x3060, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A100",        CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3061, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A200",        CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3062, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot S200",        CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3065, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS v2",       CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3065, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot S330",        CANON_CLASS_1,  (uint16_t)0x04A9, 0x3066, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS 330",      CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3066, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* 3067  MV550i Digital Video Camera */
        /* Reported at http://www.linux-usb.org/usb.ids, we have 306E. */
        /*{"Canon:PowerShot G3",        CANON_CLASS_1,  0x04A9, 0x3069, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE, NULL},*/
        /* 306a is in MacOS Info.plist, but I don't know what it is --swestin. */
        {"Canon:Digital unknown 3",     CANON_CLASS_1,  (uint16_t) 0x04A9, 0x306a, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Optura 200 MC",         CANON_CLASS_1,  (uint16_t) 0x04A9, 0x306B, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:MVX2i",                 CANON_CLASS_1,  (uint16_t) 0x04A9, 0x306B, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:IXY DV M",              CANON_CLASS_1,  (uint16_t) 0x04A9, 0x306B, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot S45 (normal mode)",   CANON_CLASS_5,  (uint16_t) 0x04A9, 0x306C, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* 0x306D is S45 in PTP mode */
        {"Canon:PowerShot G3 (normal mode)",    CANON_CLASS_5,  (uint16_t) 0x04A9, 0x306E, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* 0x306F is G3 in PTP mode */
        {"Canon:PowerShot S230 (normal mode)",  CANON_CLASS_4,  (uint16_t) 0x04A9, 0x3070, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS v3 (normal mode)", CANON_CLASS_4,  (uint16_t) 0x04A9, 0x3070, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* 0x3071 is S230/IXUS v3 in PTP mode */
        /* Following cameras share the ID for PTP and Canon modes */
#if 0 /* served better by ptp2 driver */
        {"Canon:PowerShot SD100 (normal mode)", CANON_CLASS_5,  (uint16_t) 0x04A9, 0x3072, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS II (normal mode)", CANON_CLASS_5,  (uint16_t) 0x04A9, 0x3072, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A70",         CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3073, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A60",         CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3074, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS 400",      CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3075, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot S400",        CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3075, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* End of shared ID's */
        {"Canon:PowerShot A300",        CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3076, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* S50 also shares ID for PTP and Canon modes */
        {"Canon:PowerShot S50 (normal mode)",   CANON_CLASS_4,  (uint16_t) 0x04A9, 0x3077, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
#endif
        {"Canon:ZR70MC (normal mode)",      CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3078, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* 3079 is listed for MV650i, probably in PTP mode. */
        {"Canon:MV650i (normal mode)",  CANON_CLASS_1,  (uint16_t) 0x04A9, 0x307a, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* 307b is listed for MV630i, probably in PTP mode. */
        {"Canon:MV630i (normal mode)",  CANON_CLASS_1,  (uint16_t) 0x04A9, 0x307c, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* 307f is Optura 20/MVX150i in PTP mode, someone told, but it seems not correct. Specify it here too. */
        {"Canon:Optura 20", CANON_CLASS_1,  (uint16_t) 0x04A9, 0x307f, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Optura 20 (normal mode)", CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3080, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:MVX150i (normal mode)", CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3080, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* Sighted at
         * <http://www.qbik.ch/usb/devices/showdescr.php?id=2232>. */
        {"Canon:MVX100i",               CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3081, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Optura 10",             CANON_CLASS_1,  (uint16_t) 0x04A9, 0x3082, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:EOS 10D",               CANON_CLASS_4,  (uint16_t) 0x04A9, 0x3083, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:EOS 300D (normal mode)", CANON_CLASS_4, (uint16_t) 0x04A9, 0x3084, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:EOS Digital Rebel (normal mode)",CANON_CLASS_4, (uint16_t) 0x04A9, 0x3084, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:EOS Kiss Digital (normal mode)",CANON_CLASS_4,  (uint16_t) 0x04A9, 0x3084, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
#if 0
        /* PS G5 uses the same ProductID for PTP and Canon, with protocol autodetection */
        /* Use PTP driver, as this driver had broken reports - Marcus*/
        {"Canon:PowerShot G5 (normal mode)", CANON_CLASS_5,(uint16_t) 0x04A9, 0x3085, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
#endif
        /* Elura 50 camcorder is 0x3087 in PTP mode; 3088 in Canon mode? */
        {"Canon:Elura 50 (normal mode)",  CANON_CLASS_1,(uint16_t) 0x04A9, 0x3088, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* Optura Xi/MVX 3i/FV M1 uses 308d in PTP mode; 308e in Canon mode? */
        {"Canon:Optura Xi (normal mode)",  CANON_CLASS_1,(uint16_t) 0x04A9, 0x308e, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:MVX 3i (normal mode)",  CANON_CLASS_1,(uint16_t) 0x04A9, 0x308e, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:FV M1 (normal mode)",  CANON_CLASS_1,(uint16_t) 0x04A9, 0x308e, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* Optura 300/MVX 10i/IXY DV M2 video camera uses 3093 in USB Mass Storage mode. */
        /* Optura 300/MVX 10i/IXY DV M2 video camera uses 3095 in PTP mode; 3096 in Canon mode? */
        {"Canon:Optura 300 (normal mode)",  CANON_CLASS_1,(uint16_t) 0x04A9, 0x3096, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:MVX 10i (normal mode)",  CANON_CLASS_1,(uint16_t) 0x04A9, 0x3096, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:IXY DV M2 (normal mode)",  CANON_CLASS_1,(uint16_t) 0x04A9, 0x3096, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* 0x3099 is the EOS 300D/Digital Rebel in PTP mode */
        /* A80 seems to share the ID for PTP and Canon modes */
#if 0 /* served better by PTP2 driver, especially for capture */
        {"Canon:PowerShot A80 (normal mode)",   CANON_CLASS_1,(uint16_t) 0x04A9, 0x309A, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
#endif
        /* 0x309b is the SD10 Digital ELPH/Digital IXUS i/IXY Digital L
           in PTP mode; will it work in Canon mode? */
        {"Canon:PowerShot SD10 Digital ELPH (normal mode)",   CANON_CLASS_1,(uint16_t) 0x04A9, 0x309B, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS i (normal mode)",  CANON_CLASS_1,(uint16_t)  0x04A9, 0x309B, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot IXY Digital L (normal mode)", CANON_CLASS_1,(uint16_t) 0x04A9, 0x309B, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
#if 0
/* reportedly not working ... comment out for now - Marcus */
        /* Product ID shared between PTP and Canon mode */
        {"Canon:PowerShot S1 IS (normal mode)", CANON_CLASS_5,(uint16_t) 0x04A9, 0x309C, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
#endif
        /* 30a0 is ZR90/MV750i camcorder */
        /* 30a8 is Elura 60E/MVX200i camcorder in PTP mode; don't know
         * if it will respond in native mode at the same product
         * ID. */
        /* 30a9 is Optura 40/MVX25i camcorder, reported working at
         * <http://www.qbik.ch/usb/devices/showdescr.php?id=2700>. Seems
         * to share ID with PTP. */
        {"Canon:Optura 40 (normal mode)", CANON_CLASS_1,  0x04A9, 0x30A9, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:MVX25i (normal mode)", CANON_CLASS_1,  0x04A9, 0x30A9, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
#if 0 /* Handled in ptp2, better for capture etc. -Marcus */
        /* Another block of cameras that share the ID for PTP and Canon modes */
        {"Canon:PowerShot S70 (normal mode)",   CANON_CLASS_5,  (uint16_t) 0x04A9, 0x30b1, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot S60 (normal mode)",   CANON_CLASS_5,  (uint16_t) 0x04A9, 0x30b2, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot G6 (normal mode)",    CANON_CLASS_5,  (uint16_t) 0x04A9, 0x30b3, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS 500 (normal mode)",CANON_CLASS_5,  (uint16_t) 0x04A9, 0x30b4, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot S500 Digital ELPH (normal mode)",CANON_CLASS_5,(uint16_t) 0x04A9, 0x30b4, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:IXY Digital 500 (normal mode)", CANON_CLASS_5,(uint16_t)0x04A9, 0x30b4, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A75",                 CANON_CLASS_1,(uint16_t)0x04A9, 0x30b5, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot SD110 Digital ELPH",  CANON_CLASS_1,(uint16_t)0x04A9, 0x30b6, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS IIs",              CANON_CLASS_1,(uint16_t)0x04A9, 0x30b6, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A400",                CANON_CLASS_5,(uint16_t)0x04A9, 0x30b7, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A310",                CANON_CLASS_5,(uint16_t)0x04A9, 0x30b8, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A85 (normal mode)",   CANON_CLASS_5,(uint16_t)0x04A9, 0x30b9, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot S410 Digital ELPH (normal mode)", CANON_CLASS_5,(uint16_t)0x04A9, 0x30ba, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS 430 (normal mode)",CANON_CLASS_5,(uint16_t) 0x04A9, 0x30ba, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:IXY Digital 430 (normal mode)", CANON_CLASS_5,(uint16_t) 0x04A9, 0x30ba, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A95 (normal mode)",   CANON_CLASS_5,(uint16_t) 0x04A9, 0x30bb, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* 0x30bf is PowerShot SD300/Digital IXUS 40 in PTP mode */
#endif
        /* Another block of cameras that share the ID for PTP and Canon modes */
        /* keep these enabled as the PTP variant does not support capture */
        {"Canon:PowerShot SD200 (normal mode)", CANON_CLASS_6,(uint16_t)0x04A9, 0x30c0, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS 30 (normal mode)", CANON_CLASS_6,(uint16_t)  0x04A9, 0x30c0, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:IXY Digital 40 (normal mode)",  CANON_CLASS_6,(uint16_t)  0x04A9, 0x30c0, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
#if 0
        {"Canon:PowerShot SD400 (normal mode)", CANON_CLASS_4,(uint16_t)  0x04A9, 0x30c1, CAP_NON, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS 50 (normal mode)", CANON_CLASS_4,(uint16_t)  0x04A9, 0x30c1, CAP_NON, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:IXY Digital 55 (normal mode)",  CANON_CLASS_4,(uint16_t)  0x04A9, 0x30c1, CAP_NON, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:PowerShot A510 (normal mode)",  CANON_CLASS_1,(uint16_t)  0x04A9, 0x30c2, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* End of shared ID's */
#endif
        {"Canon:PowerShot SD20 (normal mode)",  CANON_CLASS_5,(uint16_t)  0x04A9, 0x30c4, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS i5 (normal mode)", CANON_CLASS_5,(uint16_t)  0x04A9, 0x30c4, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:IXY Digital L2 (normal mode)",  CANON_CLASS_5,(uint16_t)  0x04A9, 0x30c4, CAP_SUP, SL_MOVIE_SMALL, SL_THUMB, SL_PICTURE,(char *) NULL},
        /* Is 0x30e9 EOS 1D Mark II in Canon mode? */
        /* 0x30ea is EOS 1D Mark II in PTP mode */
        {"Canon:EOS 20D (normal mode)",         CANON_CLASS_6,(uint16_t)  0x04A9, 0x30eb, CAP_EXP, SL_MOVIE_LARGE, SL_THUMB_CR2, SL_PICTURE,(char *) NULL},
        /* 0x30ec is EOS 20D in PTP mode */
        {"Canon:EOS 350D (normal mode)",                CANON_CLASS_6,(uint16_t)  0x04A9, 0x30ee, CAP_EXP, SL_MOVIE_LARGE, SL_THUMB_CR2, SL_PICTURE,(char *) NULL},
        {"Canon:Digital Rebel XT (normal mode)",                CANON_CLASS_6,(uint16_t)  0x04A9, 0x30ee, CAP_EXP, SL_MOVIE_LARGE, SL_THUMB_CR2, SL_PICTURE,(char *) NULL},
        {"Canon:EOS Kiss Digital N (normal mode)",              CANON_CLASS_6,(uint16_t)  0x04A9, 0x30ee, CAP_EXP, SL_MOVIE_LARGE, SL_THUMB_CR2, SL_PICTURE,(char *) NULL},
        /* 30ef is EOS 350D/Digital Rebel XT/EOS Kiss Digital N in PTP mode. */
        {"Canon:EOS 5D (normal mode)",          CANON_CLASS_6,(uint16_t) 0x04A9, 0x3101, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
#if 0
/* reportedly not working ... comment out for now - Marcus */
        {"Canon:PowerShot S2 IS (normal mode)",  CANON_CLASS_1,(uint16_t) 0x04A9, 0x30f0, CAP_EXP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
#endif

#if 0
/* reportedly not working either. failing with lock keys - Marcus */
        {"Canon:PowerShot SD500 (normal mode)",  CANON_CLASS_5,(uint16_t) 0x04A9, 0x30f2, CAP_NON, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:Digital IXUS 700 (normal mode)",  CANON_CLASS_5,(uint16_t) 0x04A9, 0x30f2, CAP_NON, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
        {"Canon:IXY Digital 600 (normal mode)",  CANON_CLASS_5,(uint16_t) 0x04A9, 0x30f2, CAP_NON, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
#endif

        /* I was not able to make A620 work with CLASS 5, 1 and 6. -Marcus */
#if 0
        /* also reported as not working. */
        {"Canon:PowerShot A610 (normal mode)",   CANON_CLASS_5,(uint16_t) 0x04A9, 0x30fd, CAP_SUP, SL_MOVIE_LARGE, SL_THUMB, SL_PICTURE,(char *) NULL},
#endif
        {(char *)NULL, CANON_CLASS_NONE, (uint16_t) 0x0000U,(uint16_t) 0x0000U, CAP_NON, 0U, 0U, 0U,(char *) NULL}
        /* *INDENT-ON* */
};