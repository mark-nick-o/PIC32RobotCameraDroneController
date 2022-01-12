#ifndef _MIDI_Defs_H                                                            // if not defined then define (include once)
#define _MIDI_Defs_H
//    Midi protocol definitions
//
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define MIDIPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define MIDIPACKED __attribute__((packed))                                 /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define MIDIPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define MIDIPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define MIDIPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

// Midi identity replies
#define MIDI_SYSEx_ID 0xF0u                                                     /* SysEx */
#define MIDI_NONREAL_ID 0x7Eu                                                   /* Non-Realtime */
#define MIDI_TARGET_ID 0x7Fu                                                    /* ID of target device (7F - "All Call") */
#define MIDI_GEN_INFO_ID 0x06u                                                  /* Sub-ID#1 - General Information */
#define MIDI_ID_REPLY_ID 0x02u                                                  /* Sub-ID#2 - Identity Reply */
#define MIDI_MANU_ID 0x7Du                                                      /* Manufacturer's ID: 7D - Educational Use */
#define MIDI_FAMILY_ID 0xB404l                                                  /* Family code */
#define MIDI_MODEL_NO_ID 0x32D2l                                                /* Model number */
#define MIDI_VERSION_ID 0x01000000ll,                                           /* Version number */
#define MIDI_SYSEX_END_ID 0xF7u                                                 /* End of SysEx automatically appended */

typedef union {
   uint8_t SysEx;
   uint8_t NonRealTime;
   uint8_t TargetId;
   uint8_t GenInfo;
   uint8_t IdReply;
   uint8_t ManuId;
   uint16_t FamilyId;
   uint16_t ModelNo;
   uint32_t VersionNo;
   uint8_t SysExEnd;
} Midi_identity_reply_t;                                                        // union to hold the defines as above

#define MIDI_SET_STATUS_BYTE(A) (A | 0x80u)                                     // set most significant bit
#define MIDI_SET_DATA_BYTE(A) (A & 0x7Fu)                                       // unset most significant bit

// channel messages
#define MIDI_NOTE_OFF_BYTE(A) (A | 0x80u)                                       // note on status byte D1=Notenumber D2=Velocity
#define MIDI_NOTE_ON_BYTE(A) (A | 0x90u)                                        // note off status byte D1=Notenumber D2=Velocity
#define MIDI_POLY_AFTER_BYTE(A) (A | 0xA0u)                                     // polyphonic aftertouch status byte D1=Notenumber D2=Pressure
#define MIDI_CONTROL_CHG_BYTE(A) (A | 0xB0u)                                    // control change D1=Controller D2=number Data
#define MIDI_PROG_CHG_BYTE(A) (A | 0xC0u)                                       // program change D1=Program number
#define MIDI_CHAN_AFTER_BYTE(A) (A | 0xD0u)                                     // channel aftertouch D1=Pressure
#define MIDI_PITCH_WHEEL_BYTE(A) (A | 0xE0u)                                    // Pitch wheel D1=MSB D2=LSB

// note definitions
#define MIDI_OCTAVE_MINUS_2 0u                                                  // add zero for -2 e.g. MIDI_NODE_C(MIDI_OCTAVE_MINUS_2) = C-2
#define MIDI_NOTE_C(octave) (0u+octave)
#define MIDI_NOTE_Csh(octave) (1u+octave)
#define MIDI_NOTE_D(octave) (2u+octave)
#define MIDI_NOTE_Dsh(octave) (3u+octave)
#define MIDI_NOTE_E(octave) (4u+octave)
#define MIDI_NOTE_F(octave) (5u+octave)
#define MIDI_NOTE_Fsh(octave) (6u+octave)
#define MIDI_NOTE_G(octave) (7u+octave)
#define MIDI_NOTE_Gsh(octave) (8u+octave)
#define MIDI_NOTE_A(octave) (9u+octave)
#define MIDI_NOTE_Bfl(octave) (10u+octave)
#define MIDI_NOTE_B(octave) (11u+octave)
#define MIDI_OCTAVE_MINUS_1 12u                                                 // add zero for -1 e.g. MIDI_NODE_C(MIDI_OCTAVE_MINUS_1) = C-1
#define MIDI_OCTAVE_0 24u                                                       // add zero for -0 e.g. MIDI_NODE_C(MIDI_OCTAVE_0) = C-0
#define MIDI_OCTAVE_1 36u
#define MIDI_OCTAVE_2 48u
#define MIDI_OCTAVE_3 60u                                                       // middle C C-3
#define MIDI_OCTAVE_4 72u
#define MIDI_OCTAVE_5 84u
#define MIDI_OCTAVE_6 96u
#define MIDI_OCTAVE_7 108u
#define MIDI_OCTAVE_8 120u                                                      // C-8

// control change
#define MIDI_BANK_SELECT 0u
#define MIDI_MOD_WHEEL 1u
#define MIDI_BREATH 2u
#define MIDI_FOOT_CTRL 4u
#define MIDI_PORTAMENTO 5u
#define MIDI_DATA_ENT_SLID 6u
#define MIDI_MAIN_VOL 7u
#define MIDI_BALANCE 8u
#define MIDI_PAN 0x0Au
#define MIDI_EXPRESSION 0x0Bu
#define MIDI_SUSTAIN 0x40u
#define MIDI_PORT_ONOFF 0x41u
#define MIDI_SOSTENUTO 0x42u
#define MIDI_SOFT_PEDAL 0x43u
#define MIDI_LEGATO 0x44u
#define MIDI_PORT_CTRL 0x54u

// sound controllers
#define MIDI_SOUND_VAR 0x46u                                                    // Sound variation
#define MIDI_EXT_EFF_DEPTH 0x5Bu                                                // External effects depth
#define MIDI_TIMBRE_HC 0x47u                                                    // Timbre/harmonic content
#define MIDI_TREM_DEPTH 0x5Cu                                                   // Tremolo depth
#define MIDI_REL_TIME 0x48u                                                     // Release time
#define MIDI_CHOR_DEP 0x5Du                                                     // Chorus depth
#define MIDI_ATTACK_TIM 0x49u                                                   // Attack time
#define MIDI_DETUNE_DEP 0x5Eu                                                   // Detune depth
#define MIDI_BRIGHT 0x4Au                                                       // Brightness
#define MIDI_PHASE_DEP 0x5Fu                                                    // Phase depth

// rpn messages
#define MIDI_RPN_PITCH_BEND_CORS 0x0000lul                                      // Pitch bend sensitivity
#define MIDI_RPN_PITCH_BEND_FINE 0x0001lul                                      // Fine tuning
#define MIDI_COR_TUNING 0x0002lul                                               // Coarse tuning
#define MIDI_TUN_PROG 0x0003lul                                                 // Tuning program select
#define MIDI_TUN_BANK 0x0004lul                                                 // Tuning bank select
#define MIDI_RPN_CANCEL 0x7F7Flul                                               // Cancels RPN or NRPN

// channel mode 0xB(n)
#define MIDI_ALL_SOUNDS_OFF 0x7800lul                                           // all sounds off
#define MIDI_RESET_CMODE 0x797Flul
#define MIDI_ALL_NOTES_OFF 0x7B00lul                                            // all notes off
#define MIDI_OMNI_OFF 0x7C00lul
#define MIDI_OMNI_ON 0x7D00lul
#define MIDI_POLY_ON1 0x7F00lul
#define MIDI_POLY_ON2 0x7Eul                                                    // preceded by channel id

#define MIDI_FRAME_MTC 0xF1u
#define MIDI_SONG_POINTER 0xF2u
#define MIDI_SONG_SELECT 0xF3u
#define MIDI_TUNE_REQUEST 0xF6u
#define MIDI_TIMING_CLOCK 0xF8u
#define MIDI_START 0xFAu
#define MIDI_CONTINUE 0xFBu
#define MIDI_STOP 0xFCu
#define MIDI_ACT_SENS 0xFEu
#define MIDI_RESET_MTC 0xFFu

// sysx
#define MIDI_MANUID_YAMAHA 0x43u
#define MIDI_MANUID_ROLAND 0x41u
#define MIDI_MANUID_AKAI 0x47u

MIDIPACKED(
typedef struct {
  uint8_t MSBStatus : 1u;                                                       // status bit 1=status 2=data
  uint8_t Type : 3u;                                                            // 3 bit type
  uint8_t Channel : 4u;                                                         // channel
  uint8_t MSBData1 : 1u;                                                        // status bit 1=status 2=data
  uint8_t Data1 : 7u;                                                           // 1st data byte
  uint8_t MSBData2 : 1u;                                                        // status bit 1=status 2=data
  uint8_t Data2 : 7u;                                                           // 2nd data byte
}) MidiMessage_t;

// MIDI Time Code (MTC)
// A supplement to the MIDI specification defines a standard whereby the timing information
// in hours, minutes, seconds, and, frames) found in SMPTE timecode can carried by a MIDI connection.
// In this way, a suitably equipped MIDI sequencer may be synchronised to an absolute timing reference.
// This standard is called MIDI Time Code or MTC.
#define MIDI_STX 0xF0U                                                          // start transmission char
#define MIDI_ETX 0xF7U                                                          // end transmission char
#define MIDI_MANUF_ID 0x7FU                                                     // manufacturer is set to real time universal
#define MIDI_CHAN 0x7FU                                                         // channel is set to global braodcast
#define MIDI_CMD_MTC 0x01U                                                      // ccommand for MTC

#define MIDI_MTC_BYTE1_24 0U                                                    // 24 fps
#define MIDI_MTC_BYTE1_25 1U                                                    // 25 fps
#define MIDI_MTC_BYTE1_29 2U                                                    // 29 fps
#define MIDI_MTC_BYTE1_30 3U                                                    // 30 fps
typedef struct {                                                                // a value of 1 means active 0 inactive 2 unknown
      uint8_t stx;
      uint8_t manu;
      uint8_t chan;
      uint8_t cmd;
      uint8_t byte1;
      uint8_t hrs;
      uint8_t mins;
      uint8_t secs;
      uint8_t frame;
      uint8_t etx;
} SMPTE_midi_mtc_t;
// Message is fixed as below you need to set the smpteMTCMsg.hrs etc.....
SMPTE_midi_mtc_t smpteMTCMsg = { MIDI_STX, MIDI_MANUF_ID, MIDI_CHAN, MIDI_CMD_MTC, MIDI_MTC_BYTE1_30, 0U, 0U, 0U, 0U, MIDI_ETX };

#ifdef __cplusplus
}
#endif

#endif