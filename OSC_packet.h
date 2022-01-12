#ifndef __OSC_PAK_H_
#define __OSC_PAK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "definitions.h"

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define OSCPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define OSCPACKED __attribute__((packed))                                     /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define OSCPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define OSCPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define OSCPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

typedef enum {
    TRUE_TYPE_TAG = 'T',
    FALSE_TYPE_TAG = 'F',
    NIL_TYPE_TAG = 'N',
    INFINITUM_TYPE_TAG = 'I',
    INT32_TYPE_TAG = 'i',
    FLOAT_TYPE_TAG = 'f',
    CHAR_TYPE_TAG = 'c',
    RGBA_COLOR_TYPE_TAG = 'r',
    MIDI_MESSAGE_TYPE_TAG = 'm',
    INT64_TYPE_TAG = 'h',
    TIME_TAG_TYPE_TAG = 't',
    DOUBLE_TYPE_TAG = 'd',
    STRING_TYPE_TAG = 's',
    SYMBOL_TYPE_TAG = 'S',
    BLOB_TYPE_TAG = 'b',
    ARRAY_BEGIN_TYPE_TAG = '[',
    ARRAY_END_TYPE_TAG = ']'
} OSC_TypeTagValues_e;

#if (defined(D_FT900) || defined(__TI_ARM__))
typedef struct OSCPACKED {
    char *dataV;
    size_t size;
} OSC_OutPacketStrm_t;
#else
OSCPACKED(
typedef struct {
    char *dataV;
    size_t size;
}) blob_t;
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))
typedef struct OSCPACKED {
    uint8_t messageIsInProgress_ : 1u;
    uint8_t spare : 7u;
    char *data_;
    char *end_;
    char *typeTagsCurrent_;                                                     // stored in reverse order
    char *messageCursor_; 
    char *argumentCurrent_;   
    uint32_t *elementSizePtr_;                                                  // elementSizePtr_ has two special values: 0 indicates that a bundle isn't open, and elementSizePtr_==data_ indicates that a bundle is open but that it doesn't have a size slot (ie the outermost bundle)
} OSC_OutPacketStrm_t;
#else
OSCPACKED(
typedef struct {
    uint8_t messageIsInProgress_ : 1u;
    uint8_t spare : 7u;
    char *data_;
    char *end_;
    char *typeTagsCurrent_;                                                     // stored in reverse order
    char *messageCursor_; 
    char *argumentCurrent_;   
    uint32_t *elementSizePtr_;                                                  // elementSizePtr_ has two special values: 0 indicates that a bundle isn't open, and elementSizePtr_==data_ indicates that a bundle is open but that it doesn't have a size slot (ie the outermost bundle)
}) OSC_OutPacketStrm_t;
#endif

// ================= Function definitions ======================================
extern void OSC_FromUInt32( char *p, uint32_t x );
extern void OSC_FromInt64( char *p, int64_t x );
extern void OSC_FromUInt64( char *p, uint64_t x );
extern void OSC_FromFloat( char *p, float32_t x );
extern void OSC_FromDouble( char *p, float64_t x );
extern size_t OSC_RoundUp4( size_t x );
extern char* OutboundPacketStream_BeginElement( char *beginPtr, OSC_OutPacketStrm_t *osc );
extern void OutboundPacketStream_EndElement( char *endPtr, OSC_OutPacketStrm_t *osc  );
extern uint8_t OutboundPacketStream_ElementSizeSlotRequired( const OSC_OutPacketStrm_t osc );
extern void OutboundPacketStream_Clear( OSC_OutPacketStrm_t *osc );
extern size_t OutboundPacketStream_Capacity( const OSC_OutPacketStrm_t osc );
extern size_t OutboundPacketStream_Size( const OSC_OutPacketStrm_t osc );
extern uint8_t OutboundPacketStream_CheckForAvailableBundleSpace( const OSC_OutPacketStrm_t osc );
extern uint8_t OutboundPacketStream_CheckForAvailableMessageSpace( const char *addressPattern, const OSC_OutPacketStrm_t osc );
extern uint8_t OutboundPacketStream_CheckForAvailableArgumentSpace( size_t argumentLength, const OSC_OutPacketStrm_t osc );
extern void OutboundPacketStream_BundleInitiator( uint64_t timeTag, OSC_OutPacketStrm_t *osc );
extern void OutboundPacketStream_BeginMessage( OSC_OutPacketStrm_t *osc, const char *addressPattern );
extern void OutboundPacketStream_Add_int32( OSC_OutPacketStrm_t *osc, int32_t rhs );
extern void OutboundPacketStream_Add_NilType( OSC_OutPacketStrm_t *osc );
extern void OutboundPacketStream_Add_InfinitumType( OSC_OutPacketStrm_t *osc );
extern void OutboundPacketStream_Add_bool( OSC_OutPacketStrm_t *osc, int8_t rhs );
extern void OutboundPacketStream_Add_char( OSC_OutPacketStrm_t *osc, int32_t rhs );
extern void OutboundPacketStream_Add_RgbaColor( OSC_OutPacketStrm_t *osc, int32_t rhs );
extern void OutboundPacketStream_Add_MidiMessage( OSC_OutPacketStrm_t *osc, uint32_t rhs );
extern void OutboundPacketStream_Add_int64( OSC_OutPacketStrm_t *osc, int64_t rhs );
extern void OutboundPacketStream_Add_TimeTag( OSC_OutPacketStrm_t *osc, uint64_t rhs );
extern void OutboundPacketStream_Add_float( OSC_OutPacketStrm_t *osc, float32_t rhs );
extern void OutboundPacketStream_Add_double( OSC_OutPacketStrm_t *osc, float64_t rhs );
extern void OutboundPacketStream_Add_Blob( OSC_OutPacketStrm_t *osc, const blob_t rhs );
extern void OutboundPacketStream_Add_Symbol( OSC_OutPacketStrm_t *osc, char * rhs );
extern void OutboundPacketStream_Add_String( OSC_OutPacketStrm_t *osc, char * rhs );
extern void OutboundPacketStream_Add_ArrayInitiator( OSC_OutPacketStrm_t *osc );
extern void OutboundPacketStream_Add_ArrayTerminator( OSC_OutPacketStrm_t *osc );
extern void OutboundPacketStream_MessageTerminator( OSC_OutPacketStrm_t *osc );   

#ifdef __cplusplus
}
#endif /* __cplusplus */
 
#endif