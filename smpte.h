#ifndef SMPTE_LIB
#define SMPTE_LIB

#ifdef __cplusplus
 extern "C" {
#endif

//  SMPTE interface library.
//  smpte.h - smpte "native" operations.
//
//  Written by (C) 2020 A C P Avaiation Walkerburn Scotland
//
//  via udp message http://www.orangepi-dmx.org/orange-pi-smpte-timecode-ltc-reader-converter
//
#define SMPTE_UDP_PORT 0x5443U                                                  // UDP Port for SMPTE

#define SMPTE_START(A) { strcpy(A,"ltc!start"); }                               // Starts the TimeCode increments from start_*  (see configuration)
#define SMPTE_STOP(A) { strcpy(A,"ltc!stop"); }                                 // Stops the TimeCode increments. The TimeCode is still outputting.
#define SMPTE_RESUME(A) { strcpy(A,"ltc!resume"); }                             // Continue the TimeCode increments from the 'stop' command.

#define SMPTE_TIMECODE_START(A,B,C,D,E) { if (((((B>=0U) && (B<=24U)) && ((C>=0) && (C<=59U))) && ((D>=0U) && (D<=59U))) && ((E>=0U) && (E<=29U))) \
                                         sprintf(A,"ltc!start#%d:%d:%d.%d\r\n",B,C,D,E }; }    // Sets the start TimeCode.

#define SMPTE_TIMECODE_STOP(A,B,C,D,E) { if (((((B>=0U) && (B<=24U)) && ((C>=0) && (C<=59U))) && ((D>=0U) && (D<=59U))) && ((E>=0U) && (E<=29U))) \
                                         sprintf(A,"ltc!stop#%d:%d:%d.%d\r\n",B,C,D,E }; }    // Sets the stop TimeCode.

#define SMPTE_RATE(A,B) {  if(((B>=24U) && (B<=25U)) || ((B>=29U) && (B<=30U)))  \
                           sprintf(A, "ltc!rate#%d\r\n"); }                     //  Sets the rate of the TimeCode. Valid values: 24, 25, 29 and 30.

#define SMPTE_TIMECODE_RUNNING(A,B,C,D,E) { if (((((B>=0U) && (B<=24U)) && ((C>=0) && (C<=59U))) && ((D>=0U) && (D<=59U))) && ((E>=0U) && (E<=29U)))  \
                                         sprintf(A,"ltc!start!%d:%d:%d.%d\r\n",B,C,D,E }; }    // Sets the running TimeCode.

#define SMPTE_TIMECODE_JUMP(A,B,C,D,E) { if (((((B>=0U) && (B<=24U)) && ((C>=0) && (C<=59U))) && ((D>=0U) && (D<=59U))) && ((E>=0U) && (E<=29U)))  \
                                         sprintf(A,"ltc!start@%d:%d:%d.%d\r\n",B,C,D,E }; }    // Set and jump to the start TimeCode, but stop the TimeCode incrementing/decrementing

#define SMPTE_DIRECTION(A) { sprintf(A,"ltc!direction%d\r\n"); }                // Sets the direction. Valid values: forward and backward

#define SMPTE_PITCH {  if ((B>=100) && (B<=100))                               \
                           sprintf(A, "ltc!pitch#%d\r\n"); }                    // Sets the pitch. Valid values are -100 <= ppp >= 100 (integers).

#ifdef __cplusplus
}
#endif

#endif