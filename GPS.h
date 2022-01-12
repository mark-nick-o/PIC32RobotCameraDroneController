#ifndef __GPS_CMD__                                                             // This file is to be included if not already
#define __GPS_CMD__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//    GPS.h : NMEA UBLOX FURANO GPS protocol definitions
//
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
// NMEA talker ID's
// (GPS, SBAS, QZSS) = GP
// GLONASS = GL
// Galileo = GA
// BeiDou = GB
// Any = GN
#define GP 2u                                                                   /* Types of satelite  */
#define GL 3u
#define GA 4u
#define GN 5u

#define FURANO 1u
#define UBLOX 2u
#define QUECTEL 3u

#define TYPE_GPS UBLOX                                                          /* define the type of GPS */
#define SAT_TYPE GL                                                             /* define the Satelite Types */

#if (((TYPE_GPS == FURANO) && defined(GPS_INCLUDED)) && !defined(GPS_INCLUDED2))
#error "Furano : GPS is not supported for this option"
#elif (((TYPE_GPS == QUECTEL) && !defined(GPS_INCLUDED)) && defined(GPS_INCLUDED2))
#error "Quectel : GPS is not supported for this option"
#endif

 // RMC
#if SAT_TYPE == GL
#define GLQ_RMC(X, Y)  { sprintf(X,"$%sGLQ,RMC", Y); }                          // Poll a standard message (if the current Talker ID is GL) e.g $EIGLQ,RMC*3A<CR><LF>
#elif SAT_TYPE == GN
#define GNQ_RMC(X, Y)  { sprintf(X,"$%sGNQ,RMC", Y); }                          // Poll a standard message (if the current Talker ID is GN)
#elif  SAT_TYPE == GP
#define GPQ_RMC(X, Y)  { sprintf(X,"$%sGPQ,RMC", Y); }                          // Poll a standard message (if the current Talker ID is GP)
#elif  SAT_TYPE == GB
#define GBQ_RMC(X, Y)  { sprintf(X,"$%sGBQ,RMC", Y); }                          // Poll a standard message (if the current Talker ID is GB)
#elif  SAT_TYPE == GA
#define GAQ_RMC(X, Y)  { sprintf(X,"$%sGAQ,RMC", Y); }                          // Poll a standard message (if the current Talker ID is GA)
#endif

// GLL
#if SAT_TYPE == GL
#define GLQ_GLL(X, Y)  { sprintf(X,"$%sGLQ,GLL", Y); }                          // Poll a standard message (if the current Talker ID is GL) e.g $EIGLQ,RMC*3A<CR><LF>
#elif SAT_TYPE == GN
#define GNQ_GLL(X, Y)  { sprintf(X,"$%sGNQ,GLL", Y); }                          // Poll a standard message (if the current Talker ID is GN)
#elif  SAT_TYPE == GP
#define GPQ_GLL(X, Y)  { sprintf(X,"$%sGPQ,GLL", Y); }                          // Poll a standard message (if the current Talker ID is GP)
#elif  SAT_TYPE == GB
#define GBQ_GLL(X, Y)  { sprintf(X,"$%sGBQ,GLL", Y); }                          // Poll a standard message (if the current Talker ID is GB)
#elif  SAT_TYPE == GA
#define GAQ_GLL(X, Y)  { sprintf(X,"$%sGAQ,GLL", Y); }                          // Poll a standard message (if the current Talker ID is GA)
#endif

// GSV
#if SAT_TYPE == GL
#define GLQ_GSV(X, Y)  { sprintf(X,"$%sGLQ,GSV", Y); }                          // Poll a standard message (if the current Talker ID is GL) e.g $EIGLQ,RMC*3A<CR><LF>
#elif SAT_TYPE == GN
#define GNQ_GSV(X, Y)  { sprintf(X,"$%sGNQ,GSV", Y); }                          // Poll a standard message (if the current Talker ID is GN)
#elif  SAT_TYPE == GP
#define GPQ_GSV(X, Y)  { sprintf(X,"$%sGPQ,GSV", Y); }                          // Poll a standard message (if the current Talker ID is GP)
#elif  SAT_TYPE == GB
#define GBQ_GSV(X, Y)  { sprintf(X,"$%sGBQ,GSV", Y); }                          // Poll a standard message (if the current Talker ID is GB)
#elif  SAT_TYPE == GA
#define GAQ_GSV(X, Y)  { sprintf(X,"$%sGAQ,GSV", Y); }                          // Poll a standard message (if the current Talker ID is GA)
#endif

// VTG
#if SAT_TYPE == GL
#define GLQ_VTG(X, Y)  { sprintf(X,"$%sGLQ,VTG", Y); }                          // Poll a standard message (if the current Talker ID is GL) e.g $EIGLQ,RMC*3A<CR><LF>
#elif SAT_TYPE == GN
#define GNQ_VTG(X, Y)  { sprintf(X,"$%sGNQ,VTG", Y); }                          // Poll a standard message (if the current Talker ID is GN)
#elif  SAT_TYPE == GP
#define GPQ_VTG(X, Y)  { sprintf(X,"$%sGPQ,VTG", Y); }                          // Poll a standard message (if the current Talker ID is GP)
#elif  SAT_TYPE == GB
#define GBQ_VTG(X, Y)  { sprintf(X,"$%sGBQ,VTG", Y); }                          // Poll a standard message (if the current Talker ID is GB)
#elif  SAT_TYPE == GA
#define GAQ_VTG(X, Y)  { sprintf(X,"$%sGAQ,VTG", Y); }                          // Poll a standard message (if the current Talker ID is GA)
#endif

// GNS
#if SAT_TYPE == GL
#define GLQ_GNS(X, Y)  { sprintf(X,"$%sGLQ,GNS", Y); }                          // Poll a standard message (if the current Talker ID is GL) e.g $EIGLQ,RMC*3A<CR><LF>
#elif SAT_TYPE == GN
#define GNQ_GNS(X, Y)  { sprintf(X,"$%sGNQ,GNS", Y); }                          // Poll a standard message (if the current Talker ID is GN)
#elif  SAT_TYPE == GP
#define GPQ_GNS(X, Y)  { sprintf(X,"$%sGPQ,GNS", Y); }                          // Poll a standard message (if the current Talker ID is GP)
#elif  SAT_TYPE == GB
#define GBQ_GNS(X, Y)  { sprintf(X,"$%sGBQ,GNS", Y); }                          // Poll a standard message (if the current Talker ID is GB)
#elif SAT_TYPE == GA
#define GAQ_GNS(X, Y)  { sprintf(X,"$%sGAQ,GNS", Y); }                          // Poll a standard message (if the current Talker ID is GA)
#endif

#define ADD_CHECKSUM_GPS(X, Y)  { sprintf(X,"%s*%2.2X\n\r", Y); }               // Write the checksum in hex and complete the message

#if TYPE_GPS == UBLOX                                                           //============ UBLOX ==================================================
// proprietary messages from UBLOX
#define UB_SER_PORT_DDC 0                                                       // choice of serial port
#define UB_SER_PORT_UART1 1
#define UB_SER_PORT_UART2 2
#define UB_SER_PORT_USB 3
#define UB_SER_PORT_SPI 4
#define UBLOX_BAUD 19200                                                        // baud rate
#define UBLOX_MSGID 40
#define USE_UART1                                                               // define here what port you are using for below set-up

#ifdef USE_DDC
#define PUBX_UBLOX_CONFIG(X)  { sprintf(X,"$PUBX,41,UB_SER_PORT_DDC,0007,0003,UBLOX_BAUD,0"); }      // PUBX (manufacturer UBLOX configMsg=41
#define PUBX_UBLOX_RATE_CMD(X,Y)  { sprintf(X,"$PUBX,UBLOX_MSGID,Y,2,0,0,0,0,0",Y); } // X is string 1s on DDC id=40
#elseif USE_UART1
#define PUBX_UBLOX_CONFIG(X)  { sprintf(X,"$PUBX,41,UB_SER_PORT_UART1,0007,0003,UBLOX_BAUD,0"); }      // PUBX (manufacturer UBLOX configMsg=41
#define PUBX_UBLOX_RATE_CMD(X,Y)  { sprintf(X,"$PUBX,UBLOX_MSGID,Y,0,2,0,0,0,0",Y); } // X is string 1s on UART1 id=40 Y=msgType e.g. RMC,GLL,GSV
#elseif USE_UART2
#define PUBX_UBLOX_CONFIG(X)  { sprintf(X,"$PUBX,41,UB_SER_PORT_UART2,0007,0003,UBLOX_BAUD,0"); }      // PUBX (manufacturer UBLOX configMsg=41
#define PUBX_UBLOX_RATE_CMD(X,Y)  { sprintf(X,"$PUBX,UBLOX_MSGID,Y,0,0,2,0,0,0",Y); } // X is string 1s on UART2 id=40 Y=msgType e.g. RMC,GLL,GSV
#elseif USE_USB
#define PUBX_UBLOX_CONFIG(X)  { sprintf(X,"$PUBX,41,UB_SER_PORT_USB,0007,0003,UBLOX_BAUD,0"); }      // PUBX (manufacturer UBLOX configMsg=41
#define PUBX_UBLOX_RATE_CMD(X,Y)  { sprintf(X,"$PUBX,UBLOX_MSGID,Y,0,0,0,2,0,0",Y); } // X is string 1s on USB id=40 Y=msgType e.g. RMC,GLL,GSV
#elseif USE_SPI
#define PUBX_UBLOX_CONFIG(X)  { sprintf(X,"$PUBX,41,UB_SER_PORT_SPI,0007,0003,UBLOX_BAUD,0"); }      // PUBX (manufacturer UBLOX configMsg=41
#define PUBX_UBLOX_RATE_CMD(X,Y)  { sprintf(X,"$PUBX,UBLOX_MSGID,Y,0,0,0,0,2,0",Y); } // X is string 1s on USB id=40 Y=msgType e.g. RMC,GLL,GSV
#endif

#endif                                                                          //========== END_UBLOX ===============================================

#if TYPE_GPS == FURANO                                                          //========== FURANO ==================================================
#define FUR_REC 2
#define FUR_OFF 0

#define FUR_NONE 0                                                              // SBAS or QZAS
#define FUR_SBAS 1
#define FUR_POS 2
#define FUR_QZAS 3
#define FUR_QZASCORR 4

#define Talker1 "AUTO"                                                          // Choose the talker id
#define Talker2 "LEGACYGP"
#define Talker3 "GN"

#define RESTART_HOT "HOT"                                                       // Re-start mode
#define RESTART_COLD "COLD"
#define RESTART_WARM "WARM"
#define RESTART_FACTORY "FACTORY"

// Backup Modes available Can be ORED as multiples e.g. (FUR_BKGCLK | FUR_BKDEFLS) (backs up GCLK and DEFLS)
//
#define FUR_FLCLR 0x00                                                          //Clear the data stored in FLASH
#define FUR_BKGCLK  0x01                                                        //Back up GCLK command
#define FUR_BKDEFLS 0x02                                                        //Back up DEFLS command
#define FUR_BKTIMEALIGN 0x04                                                    //Back up TIMEALIGN command
#define FUR_BKRES1 0x08                                                         //Reserved
#define FUR_BKFIXMASK 0x10                                                      //Back up FIXMASK command
#define FUR_BKGNSS 0x20                                                         //Back up GNSS command
#define FUR_BKPPS 0x40                                                          //Back up PPS command
#define FUR_BKRES2 0x80                                                         //Reserved
#define FUR_BKNLOSMASK 0x100                                                    //Back up NLOSMASK command
#define FUR_BKSURVEY 0x200                                                      //Back up SURVEY command [*1]
#define FUR_BKHOSET 0x400                                                       //Back up HOSET command

#if SAT_TYPE == GP
#define API_FURANO_CONFIG(X)  { sprintf(X,"$PERDAPI,GNSS,Talker1,FUR_REC,FUR_OFF,FUR_OFF,FUR_REC,FUR_QZASCORR"); }      // API satelite choice GP (GPS)
#elif SAT_TYPE == GL
#define API_FURANO_CONFIG(X)  { sprintf(X,"$PERDAPI,GNSS,Talker1,FUR_OFF,FUR_REC,FUR_OFF,FUR_REC,FUR_QZASCORR"); }      // API satelite choice GLONASS
#elif SAT_TYPE == GA
#define API_FURANO_CONFIG(X)  { sprintf(X,"$PERDAPI,GNSS,Talker1,FUR_OFF,FUR_OFF,FUR_REC,FUR_REC,FUR_QZASCORR"); }      // API satelite choice Galileo
#elif SAT_TYPE == GN
#define API_FURANO_CONFIG(X)  { sprintf(X,"$PERDAPI,GNSS,Talker1,FUR_REC,FUR_REC,FUR_REC,FUR_REC,FUR_QZASCORR"); }      // API satelite choice all
#endif

#define API_FURANO_RESET(X,Y)  { sprintf(X,"$PERDAPI,RESTART,%s",Y); }          // GPS reset  Y=Restart type requested
#define API_FURANO_BACKFLASH(X,Y)  { sprintf(X,"$PERDAPI,FLASHBACKUP,%s",Y); }  // GPS backup  Y=Backup Mode Requested
#define API_FURANO_UART1(X,Y)  { sprintf(X,"$PERDCFG,UART1,%d",Y); }            // UART1  Y=Baud rate
#define API_FURANO_DEFL(X,Y)  { sprintf(X,"$PERDAPI,DEFLS,%d",Y); }             // Default leap setting Y=time in secs
#define API_FURANO_PHASESKIP(X,Y)  { sprintf(X,"$PERDAPI,PHASESKIP,%d",Y); }    // phase skip Y=phase skip flag
#define API_FURANO_EXTSYNC(X,Y,Z)  { sprintf(X,"$PERDAPI,EXTSYNC,%d,%6.6d",Y,Z); }    // external sync Y,Z mode,delay set
#define API_FURANO_ANTSET_OFF(X)  { sprintf(X,"$PERDAPI,ANTSET,0"); }           // antenna power off
#define API_FURANO_ANTSET_ON(X)  { sprintf(X,"$PERDAPI,ANTSET,1"); }            // antenna power on
#define API_FURANO_SWVERSION_ON(X)  { sprintf(X,"$PERDSYS,VERSION"); }          // request s/w version
#define API_FURANO_ANTENNA(X,Y)  { sprintf(X,"$PERDSYS,ANTSEL,%s",Y); }         // select antenna
#define API_FURANO_TIMEALIGN(X,Y)  { sprintf(X,"$PERDAPI,TIMEALIGN,%1.1d",Y); } // timealign
#define API_FURANO_CROUT(X,Y,Z)  { sprintf(X,"$PERDAPI,CROUT,%s,%d",Y,Z); }     // CR Sentance output setting
#define API_FURANO_NMEAOUT(X,Y,Z)  { sprintf(X,"$PERDCFG,NMEAOUT,%s,%d",Y,Z); } // NMEA Sentance output setting Y type is GGA, GLL, GNS, GSA, GSV, RMC, VTG, ZDA, ALL Z is time interval for sentance in seconds
#define API_FURANO_TIME(X,Y,Z,A)  { sprintf(X,"$PERDAPI,TIMEZONE,0,%d,%d,%d",Y,Z,A); }  // Y Z A == time h:m:s
#define API_FURANO_NLOS(X,Y,Z,A,B)  { sprintf(X,"$PERDAPI,NLOSMASK,0,%1.1d,%4.4d,%2.2d %4.4d",Y,Z,A,B); }  // NLOS satelite elimination command
#define API_FURANO_MODESET(X,Y,Z,A)  { sprintf(X,"$PERDAPI,MODESET,0,%1.1d,%6.6d,%6.6d",Y,Z,A); }  // Y Z A == Lock port,Coarse lock threshold, Phase skip threshold
#define API_FURANO_TAD(X,Y,Z,A,B,C,D)  { sprintf(X,"$PERDAPI,TIME,%d%d%d,%d,%d,%d",Y,Z,A,B,C,D); } // YZA time h:m:s B = day, C = month, D = year
#define API_FURANO_FIXMASK(X,Y,Z,A,B,C,D,E)  { sprintf(X,"$PERDAPI,FIXMASK,USER,%d,0,%d,0,%8.8X,%6.6X,%9.9X,%2.2X,%5.5X",Y,Z,A,B,C,D,E); } // Y=elev mask Z = SNRmask, ABCDE = GPS,Glonass,Galileo,QZSS,SBAS
#define API_FURANO_HOSET(X,Y,Z,A,B,C)  { sprintf(X,"$PERDAPI,HOSET,USER,0,%d,%7.7d,%6.6d,%3.3d,%4.4d,0,0",Y,Z,A,B,C); } // Y=elev mask Z = SNRmask, ABCDE = GPS,Glonass,Galileo,QZSS,SBAS
#define API_FURANO_SURVEY(X,Y,Z,A,B,C,D)  { sprintf(X,"$PERDAPI,SURVEY,%d,%3.3d,%5.5d,%2.7f,%2.7f,%2.7f",Y,Z,A,B,C,D); }
#define API_FURANO_GCLK(X,Y,Z,A)  { sprintf(X,"$PERDAPI,GCLK,%1.1d,%8.8d,%2.2d,0",Y,Z,A); }

#endif                                                                          //========== END_FURANO ===============================================

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif                                                                          // end of including  __GPS_CMD__