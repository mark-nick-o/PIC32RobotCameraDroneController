/*
        oscpack -- Open Sound Control (OSC) packet manipulation library
        http://www.rossbencina.com/code/oscpack

        Copyright (c) 2004-2013 Ross Bencina <rossb@audiomulch.com>
        Ported to mikroE C by ACP Aviation code came from kougaku
        https://github.com/kougaku/RealSenseOSC/releases
        
        Permission is hereby granted, free of charge, to any person obtaining
        a copy of this software and associated documentation files
        (the "Software"), to deal in the Software without restriction,
        including without limitation the rights to use, copy, modify, merge,
        publish, distribute, sublicense, and/or sell copies of the Software,
        and to permit persons to whom the Software is furnished to do so,
        subject to the following conditions:

        The above copyright notice and this permission notice shall be
        included in all copies or substantial portions of the Software.

        THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
        EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
        MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
        IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
        ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
        CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
        WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
        The text above constitutes the entire oscpack license; however, 
        the oscpack developer(s) also make the following non-binding requests:

        Any person wishing to distribute modifications to the Software is
        requested to send the modifications to the original developer so that
        they can be incorporated into the canonical version. It is also 
        requested that these non-binding requests be included whenever the
        above license is reproduced.
*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include "definitions.h"
#include "OSC_packet.h"

#define OSC_HOST_LITTLE_ENDIAN 

// ================= Function definitions ======================================
void OSC_FromUInt32( char *p, uint32_t x );
void OSC_FromInt64( char *p, int64_t x );
void OSC_FromUInt64( char *p, uint64_t x );
void OSC_FromFloat( char *p, float32_t x );
void OSC_FromDouble( char *p, float64_t x );
size_t OSC_RoundUp4( size_t x );
char* OutboundPacketStream_BeginElement( char *beginPtr, OSC_OutPacketStrm_t *osc );
void OutboundPacketStream_EndElement( char *endPtr, OSC_OutPacketStrm_t *osc  );
uint8_t OutboundPacketStream_ElementSizeSlotRequired( const OSC_OutPacketStrm_t osc );
void OutboundPacketStream_Clear( OSC_OutPacketStrm_t *osc );
size_t OutboundPacketStream_Capacity( const OSC_OutPacketStrm_t osc );
size_t OutboundPacketStream_Size( const OSC_OutPacketStrm_t osc );
uint8_t OutboundPacketStream_CheckForAvailableBundleSpace( const OSC_OutPacketStrm_t osc );
uint8_t OutboundPacketStream_CheckForAvailableMessageSpace( const char *addressPattern, const OSC_OutPacketStrm_t osc );
uint8_t OutboundPacketStream_CheckForAvailableArgumentSpace( size_t argumentLength, const OSC_OutPacketStrm_t osc );
void OutboundPacketStream_BundleInitiator( uint64_t timeTag, OSC_OutPacketStrm_t *osc );
void OutboundPacketStream_BeginMessage( OSC_OutPacketStrm_t *osc, const char *addressPattern );
void OutboundPacketStream_Add_int32( OSC_OutPacketStrm_t *osc, int32_t rhs );
void OutboundPacketStream_Add_NilType( OSC_OutPacketStrm_t *osc );
void OutboundPacketStream_Add_InfinitumType( OSC_OutPacketStrm_t *osc );
void OutboundPacketStream_Add_bool( OSC_OutPacketStrm_t *osc, int8_t rhs );
void OutboundPacketStream_Add_char( OSC_OutPacketStrm_t *osc, int32_t rhs );
void OutboundPacketStream_Add_RgbaColor( OSC_OutPacketStrm_t *osc, int32_t rhs );
void OutboundPacketStream_Add_MidiMessage( OSC_OutPacketStrm_t *osc, uint32_t rhs );
void OutboundPacketStream_Add_int64( OSC_OutPacketStrm_t *osc, int64_t rhs );
void OutboundPacketStream_Add_TimeTag( OSC_OutPacketStrm_t *osc, uint64_t rhs );
void OutboundPacketStream_Add_float( OSC_OutPacketStrm_t *osc, float32_t rhs );
void OutboundPacketStream_Add_double( OSC_OutPacketStrm_t *osc, float64_t rhs );
void OutboundPacketStream_Add_Blob( OSC_OutPacketStrm_t *osc, const blob_t rhs );
void OutboundPacketStream_Add_Symbol( OSC_OutPacketStrm_t *osc, char * rhs );
void OutboundPacketStream_Add_String( OSC_OutPacketStrm_t *osc, char * rhs );
void OutboundPacketStream_Add_ArrayInitiator( OSC_OutPacketStrm_t *osc );
void OutboundPacketStream_Add_ArrayTerminator( OSC_OutPacketStrm_t *osc );
void OutboundPacketStream_MessageTerminator( OSC_OutPacketStrm_t *osc );

/*-----------------------------------------------------------------------------
 *      OSC_FromInt32:  from int32_t
 *      
 *  Parameters: char *p, int32_t x
 *  Return:   void
 *----------------------------------------------------------------------------*/
void OSC_FromInt32( char *p, int32_t x )
{
#ifdef OSC_HOST_LITTLE_ENDIAN
    union{
        int32_t i;
        char c[4u];
    } u;

    u.i = x;

    p[3u] = u.c[0u];
    p[2u] = u.c[1u];
    p[1u] = u.c[2u];
    p[0u] = u.c[3u];
#else
    *p = x;
#endif
}
/*-----------------------------------------------------------------------------
 *      OSC_FromUInt32:  from uint32_t
 *      
 *  Parameters: char *p, uint32_t x
 *  Return:   void
 *----------------------------------------------------------------------------*/
void OSC_FromUInt32( char *p, uint32_t x )
{
#ifdef OSC_HOST_LITTLE_ENDIAN
    union{
        uint32_t i;
        char c[4u];
    } u;

    u.i = x;

    p[3u] = u.c[0u];
    p[2u] = u.c[1u];
    p[1u] = u.c[2u];
    p[0u] = u.c[3u];
#else
    *p = x;
#endif
}
/*-----------------------------------------------------------------------------
 *      OSC_FromInt64:  from int64_t
 *      
 *  Parameters: char *p, int64_t x
 *  Return:   void
 *----------------------------------------------------------------------------*/
void OSC_FromInt64( char *p, int64_t x )
{
#ifdef OSC_HOST_LITTLE_ENDIAN
    union{
        int64_t i;
        char c[8u];
    } u;

    u.i = x;

    p[7u] = u.c[0u];
    p[6u] = u.c[1u];
    p[5u] = u.c[2u];
    p[4u] = u.c[3u];
    p[3u] = u.c[4u];
    p[2u] = u.c[5u];
    p[1u] = u.c[6u];
    p[0u] = u.c[7u];
#else
    *p = x;
#endif
}
/*-----------------------------------------------------------------------------
 *      OSC_FromUInt64:  from uint64_t
 *      
 *  Parameters: char *p, uint64_t x
 *  Return:   void
 *----------------------------------------------------------------------------*/
void OSC_FromUInt64( char *p, uint64_t x )
{
#ifdef OSC_HOST_LITTLE_ENDIAN
    union{
        uint64_t i;
        char c[8u];
    } u;

    u.i = x;

    p[7u] = u.c[0u];
    p[6u] = u.c[1u];
    p[5u] = u.c[2u];
    p[4u] = u.c[3u];
    p[3u] = u.c[4u];
    p[2u] = u.c[5u];
    p[1u] = u.c[6u];
    p[0u] = u.c[7u];
#else
    *p = x;
#endif
}
/*-----------------------------------------------------------------------------
 *      OSC_FromFloat:  from float
 *      
 *  Parameters: char *p, float32_t x
 *  Return:   void
 *----------------------------------------------------------------------------*/
void OSC_FromFloat( char *p, float32_t x )
{
#ifdef OSC_HOST_LITTLE_ENDIAN
    union{
        float32_t f;
        char c[4u];
    } u;
    u.f = x;

    p[3u] = u.c[0u];
    p[2u] = u.c[1u];
    p[1u] = u.c[2u];
    p[0u] = u.c[3u];
#else
    *p = x;
#endif
}
/*-----------------------------------------------------------------------------
 *      OSC_FromDouble:  from double
 *      
 *  Parameters: char *p, float64_t x
 *  Return:   void
 *----------------------------------------------------------------------------*/
void OSC_FromDouble( char *p, float64_t x )
{
#ifdef OSC_HOST_LITTLE_ENDIAN
    union{
        float64_t f;
        char c[4u];
    } u;
    u.f = x;

    p[7u] = u.c[0u];
    p[6u] = u.c[1u];
    p[5u] = u.c[2u];
    p[4u] = u.c[3u];
    p[3u] = u.c[4u];
    p[2u] = u.c[5u];
    p[1u] = u.c[6u];
    p[0u] = u.c[7u];
#else
    *p = x;
#endif
}
/*-----------------------------------------------------------------------------
 *      OSC_RoundUp4:  round up to the next highest multiple of 4. 
 *                     unless x is already a multiple of 4
 *
 *  Parameters: size_t x
 *  Return:   size_t
 *----------------------------------------------------------------------------*/
size_t OSC_RoundUp4( size_t x ) 
{
    return (x + 3u) & ~((size_t)0x03u);
}
/*-----------------------------------------------------------------------------
 *      OutboundPacketStream_BeginElement:  begin element 
 *
 *  Parameters: char *beginPtr, OSC_OutPacketStrm_t *osc
 *  Return:   char*
 *----------------------------------------------------------------------------*/
char* OutboundPacketStream_BeginElement( char *beginPtr, OSC_OutPacketStrm_t *osc )
{
    if( osc->elementSizePtr_ == 0 )
    {
        osc->elementSizePtr_ = (uint32_t*)(osc->data_);
        return beginPtr;
    }
    else
    {
        beginPtr = (char*)((char*)(osc->elementSizePtr_) - (char*)osc->data_);  // store an offset to the old element size ptr in the element size slot we store an offset rather than the actual pointer to be 64 bit clean.
        osc->elementSizePtr_ = (uint32_t*)(beginPtr);
        return beginPtr + 4;
    }
}
/*-----------------------------------------------------------------------------
 *      OutboundPacketStream_EndElement:  end element 
 *
 *  Parameters: char *endPtr, OSC_OutPacketStrm_t *osc
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_EndElement( char *endPtr, OSC_OutPacketStrm_t *osc  )
{
    char * d;
    uint32_t elementSize;
    uint32_t *previousElementSizePtr;
        
    if (osc==NULL) return;

    if( osc->elementSizePtr_ == (uint32_t*)(osc->data_) )
    {
        osc->elementSizePtr_ = 0;
    }
    else
    {
        // while building an element, an offset to the containing element's
        // size slot is stored in the elements size slot (or a ptr to data_
        // if there is no containing element). We retrieve that here
        previousElementSizePtr =  (uint32_t*)(osc->data_ + *osc->elementSizePtr_);

        // then we store the element size in the slot. note that the element
        // size does not include the size slot, hence the - 4 below.

        d = (char*)((char*)endPtr - (char*)(osc->elementSizePtr_));
        // assert( d >= 4 && d <= 0x7FFFFFFF ); // assume packets smaller than 2Gb

        elementSize = (uint32_t)(d - 4);
        OSC_FromUInt32( (char*)(osc->elementSizePtr_), elementSize );

        osc->elementSizePtr_ = previousElementSizePtr;                          // finally, we reset the element size ptr to the containing element
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_ElementSizeSlotRequired:  slot size 
 *
 *  Parameters: const OSC_OutPacketStrm_t osc
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t OutboundPacketStream_ElementSizeSlotRequired( const OSC_OutPacketStrm_t osc ) 
{
    return (osc.elementSizePtr_ != 0);
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Clear:  clear stream 
 *
 *  Parameters: OSC_OutPacketStrm_t osc
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Clear( OSC_OutPacketStrm_t *osc )
{
    osc->typeTagsCurrent_ = osc->end_;
    osc->messageCursor_ = osc->data_;
    osc->argumentCurrent_ = osc->data_;
    osc->elementSizePtr_ = 0;
    osc->messageIsInProgress_ = false;
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Capacity:  show capacity used
 *
 *  Parameters: const OSC_OutPacketStrm_t osc
 *  Return: void
 *----------------------------------------------------------------------------*/
size_t OutboundPacketStream_Capacity( const OSC_OutPacketStrm_t osc )
{
    return osc.end_ - osc.data_;
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Size:  show size
 *
 *  Parameters: const OSC_OutPacketStrm_t osc
 *  Return: size_t
 *----------------------------------------------------------------------------*/
size_t OutboundPacketStream_Size( const OSC_OutPacketStrm_t osc ) 
{
    size_t result = osc.argumentCurrent_ - osc.data_;
    if( osc.messageIsInProgress_ ) 
    {
        result += OSC_RoundUp4( (osc.end_ - osc.typeTagsCurrent_) + 2 );        // account for the length of the type tag string. the total type tag includes an initial comma, plus at least one terminating \0
    }
    return result;
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_CheckForAvailableBundleSpace:  show bundle space
 *
 *  Parameters: const OSC_OutPacketStrm_t osc
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t OutboundPacketStream_CheckForAvailableBundleSpace( const OSC_OutPacketStrm_t osc )
{
    size_t required = OutboundPacketStream_Size(osc) + ((OutboundPacketStream_ElementSizeSlotRequired(osc))?4:0) + 16;
    return ( required < OutboundPacketStream_Capacity(osc) ); 
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_CheckForAvailableMessageSpace:  check available message space
 *
 *  Parameters: const char *addressPattern, const OSC_OutPacketStrm_t osc
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t OutboundPacketStream_CheckForAvailableMessageSpace( const char *addressPattern, const OSC_OutPacketStrm_t osc )
{
    size_t required = OutboundPacketStream_Size(osc) + (size_t)((OutboundPacketStream_ElementSizeSlotRequired(osc))?4:0) + OSC_RoundUp4(strlen((char*)addressPattern) + 1) + 4;       // plus 4 for at least four bytes of type tag
    return ( required < OutboundPacketStream_Capacity(osc) ); 
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_CheckForAvailableArgumentSpace:  check available arg space
 *
 *  Parameters: size_t argumentLength, const OSC_OutPacketStrm_t osc
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t OutboundPacketStream_CheckForAvailableArgumentSpace( size_t argumentLength, const OSC_OutPacketStrm_t osc )
{
    size_t required = (osc.argumentCurrent_ - osc.data_) + argumentLength + OSC_RoundUp4( (osc.end_ - osc.typeTagsCurrent_) + 3 );     // plus three for extra type tag, comma and null terminator
    return ( required < OutboundPacketStream_Capacity(osc) ); 
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_BundleInitiator:  bundle initialtor
 *
 *  Parameters: uint64_t timeTag, OSC_OutPacketStrm_t *osc
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_BundleInitiator( uint64_t timeTag, OSC_OutPacketStrm_t *osc )
{
   if ( osc == NULL ) 
   { /* misra */ }
   else
   {        
        if( osc->messageIsInProgress_==false )
        {
                if (OutboundPacketStream_CheckForAvailableBundleSpace( *osc )==true)
                {        
                   osc->messageCursor_ = OutboundPacketStream_BeginElement( osc->messageCursor_, osc );
                   memcpy( (void*)osc->messageCursor_,(void*) "#bundle\0", 8 );
                   OSC_FromUInt64( osc->messageCursor_ + 8, timeTag );
                   osc->messageCursor_ += 16;
                   osc->argumentCurrent_ = osc->messageCursor_;
                }
        }
    }                 
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_BeginMessage:  begin message
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, const char *addressPattern
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_BeginMessage( OSC_OutPacketStrm_t *osc, const char *addressPattern )
{
    size_t rhsLength = strlen((char*)addressPattern);
    size_t i;
    if ( osc == NULL ) 
    { /* misra */ }
    else
    {        
        if( osc->messageIsInProgress_==false )
        {
            if (OutboundPacketStream_CheckForAvailableMessageSpace( addressPattern, *osc )==true)
            {
                osc->messageCursor_ = OutboundPacketStream_BeginElement( osc->messageCursor_, osc );
                strcpy(osc->messageCursor_, (char*)addressPattern);
                osc->messageCursor_ += rhsLength + 1;

                i = rhsLength + 1;                                              // zero pad to 4-byte boundary
                while( i & 0x3u )
                {
                   *osc->messageCursor_++ = '\0';
                   i=++i;
                }
                osc->argumentCurrent_ = osc->messageCursor_;
                osc->typeTagsCurrent_ = osc->end_;
                osc->messageIsInProgress_ = true;
            }
        }
   }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_int32:  add int32
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, int32_t rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_int32( OSC_OutPacketStrm_t *osc, int32_t rhs )
{
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 4, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = INT32_TYPE_TAG;
          OSC_FromInt32( osc->argumentCurrent_, rhs );
          osc->argumentCurrent_ += 4;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_NilType:  add nil type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_NilType( OSC_OutPacketStrm_t *osc )
{
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 0, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = NIL_TYPE_TAG;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_InfinitumType:  add infintum type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_InfinitumType( OSC_OutPacketStrm_t *osc )
{
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 0, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = INFINITUM_TYPE_TAG;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_bool:  add boolean type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, int8_t rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_bool( OSC_OutPacketStrm_t *osc, int8_t rhs )
{
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 0, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = (char)((rhs) ? TRUE_TYPE_TAG : FALSE_TYPE_TAG);
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_char:  add char type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, int32_t rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_char( OSC_OutPacketStrm_t *osc, int32_t rhs )
{
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 4, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = CHAR_TYPE_TAG;
          OSC_FromInt32( osc->argumentCurrent_, rhs );
          osc->argumentCurrent_ += 4;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_RgbaColor:  add RGB type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, int32_t rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_RgbaColor( OSC_OutPacketStrm_t *osc, int32_t rhs )
{
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 4, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = RGBA_COLOR_TYPE_TAG;
          OSC_FromInt32( osc->argumentCurrent_, rhs );
          osc->argumentCurrent_ += 4;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_MidiMessage:  add midi type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, int32_t rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_MidiMessage( OSC_OutPacketStrm_t *osc, uint32_t rhs )
{
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 4, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = MIDI_MESSAGE_TYPE_TAG;
          OSC_FromUInt32( osc->argumentCurrent_, rhs );
          osc->argumentCurrent_ += 4;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_int64:  add int64_t type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, int64_t rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_int64( OSC_OutPacketStrm_t *osc, int64_t rhs )
{
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 8, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = INT64_TYPE_TAG;
          OSC_FromInt64( osc->argumentCurrent_, rhs );
          osc->argumentCurrent_ += 8;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_TimeTag:  add time tag type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, uint64_t rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_TimeTag( OSC_OutPacketStrm_t *osc, uint64_t rhs )
{
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 8, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = TIME_TAG_TYPE_TAG;
          OSC_FromUInt64( osc->argumentCurrent_, rhs );
          osc->argumentCurrent_ += 8;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_float:  add float type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, float32_t rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_float( OSC_OutPacketStrm_t *osc, float32_t rhs )
{        
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 4, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = FLOAT_TYPE_TAG;
          OSC_FromFloat( osc->argumentCurrent_, rhs );
          osc->argumentCurrent_ += 4;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_double:  add double type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, float64_t rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_double( OSC_OutPacketStrm_t *osc, float64_t rhs )
{        
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 8, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = DOUBLE_TYPE_TAG;
          OSC_FromDouble( osc->argumentCurrent_, rhs );
          osc->argumentCurrent_ += 8;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_Blob:  add blob type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, const blob_t rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_Blob( OSC_OutPacketStrm_t *osc, const blob_t rhs )
{
    uint64_t i = rhs.size;
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 4 + OSC_RoundUp4(rhs.size), *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = BLOB_TYPE_TAG;
          OSC_FromUInt32( osc->argumentCurrent_, rhs.size );
          osc->argumentCurrent_ += 4;
          memcpy( (void*)osc->argumentCurrent_,(void*) rhs.dataV, rhs.size );
          osc->argumentCurrent_ += rhs.size;
          while( i & 0x3u )
          {
              *osc->argumentCurrent_++ = '\0';
              i=++i;
          }
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_Symbol:  add symbol type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, char * rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_Symbol( OSC_OutPacketStrm_t *osc, char * rhs )
{
    const size_t rhsLength = strlen(rhs);
    size_t i = rhsLength + 1;
        
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( rhsLength+1, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = SYMBOL_TYPE_TAG;
          strcpy(osc->argumentCurrent_, rhs);
          osc->argumentCurrent_ += rhsLength + 1;
          while( i & 0x3u )                                                     // zero pad to 4-byte boundary
          {
             *osc->argumentCurrent_++ = '\0';
             i=++i;
          }
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_String:  add string type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc, char * rhs
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_String( OSC_OutPacketStrm_t *osc, char * rhs )
{
    const size_t rhsLength = strlen(rhs);
    size_t i = rhsLength + 1;
        
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( rhsLength+1, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = STRING_TYPE_TAG;
          strcpy(osc->argumentCurrent_, rhs);
          osc->argumentCurrent_ += rhsLength + 1;

          while( i & 0x3u )                                                     // zero pad to 4-byte boundary
          {
             *osc->argumentCurrent_++ = '\0';
             i=++i;
          }
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_ArrayInitiator:  add array type
 *
 *  Parameters: OSC_OutPacketStrm_t *osc
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_ArrayInitiator( OSC_OutPacketStrm_t *osc )
{
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 0, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = ARRAY_BEGIN_TYPE_TAG;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_Add_ArrayTerminator:  add array terminator
 *
 *  Parameters: OSC_OutPacketStrm_t *osc
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_Add_ArrayTerminator( OSC_OutPacketStrm_t *osc )
{
    if (osc == NULL)
    { /* for misra */ }
    else
    {        
       if (OutboundPacketStream_CheckForAvailableArgumentSpace( 0, *osc )==true)
       {
          *(--osc->typeTagsCurrent_) = ARRAY_END_TYPE_TAG;
       }
    }
}
/*-----------------------------------------------------------------------------
 *   OutboundPacketStream_MessageTerminator:  add message terminator
 *
 *  Parameters: OSC_OutPacketStrm_t *osc
 *  Return: void
 *----------------------------------------------------------------------------*/
void OutboundPacketStream_MessageTerminator( OSC_OutPacketStrm_t *osc )
{
    char *tempTypeTags;
    size_t typeTagsCount = osc->end_ - osc->typeTagsCurrent_;
    size_t typeTagSlotSize;
    size_t argumentsSize;
    size_t i;
    char *p;
        
    if( osc->messageIsInProgress_==false )
    {
       if( typeTagsCount )
       {
           tempTypeTags = (char*)Malloc(typeTagsCount);
           memcpy( (void*)tempTypeTags, (void*)osc->typeTagsCurrent_, typeTagsCount );

           typeTagSlotSize = OSC_RoundUp4( typeTagsCount + 2 );                 // slot size includes comma and null terminator
           argumentsSize = osc->argumentCurrent_ - osc->messageCursor_;
           memmove( osc->messageCursor_ + typeTagSlotSize, osc->messageCursor_, argumentsSize );
           osc->messageCursor_[0u] = ',';
                   
           for( i=0; i < typeTagsCount; ++i )                                   // copy type tags in reverse (really forward) order
              osc->messageCursor_[i+1] = tempTypeTags[ (typeTagsCount-1) - i ];
              
           Free(tempTypeTags,sizeof(tempTypeTags));                  
           p = osc->messageCursor_ + 1 + typeTagsCount;
           for( i=0; i < (typeTagSlotSize - (typeTagsCount + 1)); ++i )
              *p++ = '\0';
                   
            osc->typeTagsCurrent_ = osc->end_;
            osc->messageCursor_ += typeTagSlotSize + argumentsSize;             // advance messageCursor_ for next message

       }
       else
       {
           memcpy( (void*) osc->messageCursor_, (void*) ",\0\0\0", 4 );         // send an empty type tags string
           osc->messageCursor_ += 4;                                            // advance messageCursor_ for next message
       }
    }

    osc->argumentCurrent_ = osc->messageCursor_;
    OutboundPacketStream_EndElement( osc->messageCursor_, osc );
    osc->messageIsInProgress_ = false;
}
#ifdef __cplusplus
}
#endif /* __cplusplus */