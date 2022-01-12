#ifndef __bison_h
#define __bison_h
/*****************************************************************************
 * bson-decoder.h                                                            *
 *                                                                           *
 * This file contains implementations for a C BSON decoder.  It supports     *
 * documents packed together in a single file or stream.                     *
 *                                                                           *
 *                                                                           *
 *   Authors: Wolfgang Richter <wolf@cs.cmu.edu>                             *
 *   Ported for pic ft90 by ACP Aviation from the c program only             *
 *                                                                           *
 *   Copyright 2013-2014 Carnegie Mellon University                          *
 *                                                                           *
 *   Licensed under the Apache License, Version 2.0 (the "License");         *
 *   you may not use this file except in compliance with the License.        *
 *   You may obtain a copy of the License at                                 *
 *                                                                           *
 *       http://www.apache.org/licenses/LICENSE-2.0                          *
 *                                                                           *
 *   Unless required by applicable law or agreed to in writing, software     *
 *   distributed under the License is distributed on an "AS IS" BASIS,       *
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.*
 *   See the License for the specific language governing permissions and     *
 *   limitations under the License.                                          *
 *****************************************************************************/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define BISONPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define BISONPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define BISONPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define BISONPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define BISONPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define MAX_BISON_CHUNK 255u                                                    // set this to the maximum bison chunk you are reading

typedef enum {BSON_DOUBLE = 0u, BSON_STRING = 1u, BSON_EMBEDDED_DOCUMENT=2u, BSON_ARRAY=3u, BSON_BINARY=4u, BSON_UNDEFINED=5u, BSON_OBJECTID=6u, BSON_BOOLEAN=7u, BSON_UTC_DATETIME=8u, BSON_NULL=9u, BSON_REGEX=10u, BSON_DBPOINTER=11u, BSON_JS=12u, BSON_SYMBOL=13u, BSON_JS_CODE=14u, BSON_INT32=15u, BSON_TIMESTAMP=16u, BSON_INT64=16u, BSON_MIN=17u, BSON_MAX=18u } bison_type_e;

#if defined(D_FT900)
typedef struct BISONPACKED {
  int32_t size;                                                                // size of bison chunk
  uint16_t position;                                                            // pointer position
  unsigned char buffer[MAX_BISON_CHUNK];                                        // max bison chunk
} bson_info_t;                                                                 // bison information structure
#else
BISONPACKED(
typedef struct {
  int32_t size;                                                                // size of bison chunk
  uint16_t position;                                                            // pointer position
  unsigned char buffer[MAX_BISON_CHUNK];                                        // max bison chunk
}) bson_info_t;                                                                 // bison information structure
#endif

#if defined(D_FT900)
typedef struct BISONPACKED {
  uint16_t type;
  uint32_t key;
  int32_t size;
  unsigned char dataV[MAX_BISON_CHUNK];
  uint8_t subtype;
} bson_kv_t;
#else
BISONPACKED(
typedef struct {
  uint16_t type;
  uint32_t key;
  int32_t size;
  unsigned char dataV[MAX_BISON_CHUNK];
  uint8_t subtype;
}) bson_kv_t;
#endif

/*____________________________________________________________________________*/
/* #####################---| Function Definition |---######################## */
int16_t deserialize_document(bson_info_t* bson_info, bson_kv_t* kv);
int16_t deserialize_string(bson_info_t* bson_info, bson_kv_t* kv);
int16_t deserialize_cstring(bson_info_t* bson_info, bson_kv_t* kv);
int16_t bson_deserialize(bson_info_t* bson_info,bson_kv_t* value,bson_kv_t* value2);

/*-----------------------------------------------------------------------------
 *      deserialize_document:  parse bison document
 *
 *
 *  Parameters: bson_info_t* bson_info, bson_kv_t* kv
 *
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
int16_t deserialize_document(bson_info_t* bson_info, bson_kv_t* kv)
{
    if (bson_info == NULL || bson_info->buffer == NULL || kv == NULL)
        return 0;

    kv->size = *((int32_t*) &(bson_info->buffer[bson_info->position]));
//    kv->dataV = &(bson_info->buffer[bson_info->position]);
    memcpy((void*)kv->dataV,(void*) &(bson_info->buffer[bson_info->position]), kv->size);
    
    bson_info->position += kv->size;
    bson_info->size -= kv->size;
    return 1;
}

/*-----------------------------------------------------------------------------
 *      deserialize_string:  parse bison string
 *
 *
 *  Parameters: bson_info_t* bson_info, bson_kv_t* kv
 *
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
int16_t deserialize_string(bson_info_t* bson_info, bson_kv_t* kv)
{

    if (bson_info == NULL || bson_info->buffer == NULL || kv == NULL)
        return 0u;

    kv->size = *((int32_t*) &(bson_info->buffer[bson_info->position])) - 1;
    bson_info->position += 4;
    bson_info->size -= 4;
//    kv->data = &(bson_info->buffer[bson_info->position]);
    memcpy((void*)kv->dataV,(void*) &(bson_info->buffer[bson_info->position]), kv->size);
    
    bson_info->position += kv->size + 1;
    bson_info->size -= kv->size + 1;

    if (bson_info->position > bson_info->size)
        return 0;

    return 1;
}
/*-----------------------------------------------------------------------------
 *      deserialize_cstring:  parse bison cstring
 *
 *
 *  Parameters: bson_info_t* bson_info, bson_kv_t* kv
 *
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
int16_t deserialize_cstring(bson_info_t* bson_info, bson_kv_t* kv)
{
    if (bson_info == NULL || bson_info->buffer == NULL || kv == NULL)
        return 0;

//    kv->dataV = &(bson_info->buffer[bson_info->position]);
    memcpy((void*)kv->dataV,(void*) &(bson_info->buffer[bson_info->position]), kv->size);
    
    bson_info->position += strlen(kv->dataV) + 1;
    bson_info->size -= strlen(kv->dataV) + 1;

    if (bson_info->position > bson_info->size)
        return 0;

    return 1;
}
/*-----------------------------------------------------------------------------
 *      bson_deserialize:  parse bison chunk
 *
 *
 *  Parameters: bson_info_t* bson_info,bson_kv_t* value,bson_kv_t* value2
 *
 *  Return:     int16_t
 *----------------------------------------------------------------------------*/
int16_t bson_deserialize(bson_info_t* bson_info,bson_kv_t* value,bson_kv_t* value2)
{
    if (bson_info->size < 1)
        return 0;

    if (bson_info->size == 1)
    {
//        assert(bson_info->buffer[bson_info->position] == 0);
        bson_info->position++;
        bson_info->size--;
        return 0;
    }

    value->type = bson_info->buffer[bson_info->position];
    bson_info->position++;
    bson_info->size--;

    deserialize_cstring(bson_info, value);
    value->key = (uint32_t) value->dataV;
    value->size = 0;

    switch(value->type)
    {
        case BSON_DOUBLE:
        value->size = 8;
        memcpy((void*)value->dataV,(void*) &(bson_info->buffer[bson_info->position]), value->size);
        //value->data = &(bson_info->buffer[bson_info->position]);
        value->size = 8;
        bson_info->position += 8;
        bson_info->size -= 8;
        break;

        case BSON_STRING:
        deserialize_string(bson_info, value);
        break;

        case BSON_EMBEDDED_DOCUMENT:
        deserialize_document(bson_info, value2);
        break;

        case BSON_ARRAY:
        deserialize_document(bson_info, value2);
        break;

        case BSON_BINARY:
        value->size = *((int32_t *) (&bson_info->buffer[bson_info->position]));
        bson_info->position += 4;
        bson_info->size -= 4;
        value->subtype = *((uint8_t *) (&bson_info->buffer[bson_info->position]));
        bson_info->position += 1;
        bson_info->size -= 1;
        //value->data = &(bson_info->buffer[bson_info->position]);
        memcpy((void*)value->dataV,(void*) &(bson_info->buffer[bson_info->position]), value->size);
        bson_info->position += value->size;
        bson_info->size -= value->size;
        break;

        case BSON_UNDEFINED:
        /* finished */
        break;

        case BSON_OBJECTID:
        value->size = 12;
        memcpy((void*)value->dataV,(void*) &(bson_info->buffer[bson_info->position]), value->size);
        //value->data = &(bson_info->buffer[bson_info->position]);
        bson_info->position += 12;
        bson_info->size -= 12;
        break;

        case BSON_BOOLEAN:
        value->size = 1;
        memcpy((void*)value->dataV,(void*) &(bson_info->buffer[bson_info->position]), value->size);
        //value->data = &(bson_info->buffer[bson_info->position]);
        value->size = 1;
        bson_info->position += 1;
        bson_info->size -= 1;
        break;

        case BSON_UTC_DATETIME:
        value->size = 8;
        memcpy((void*)value->dataV,(void*) &(bson_info->buffer[bson_info->position]), value->size);
        //value->data = &(bson_info->buffer[bson_info->position]);
        bson_info->position += 8;
        bson_info->size -= 8;
        break;

        case BSON_NULL:
        /* finished */
        break;

        case BSON_REGEX:
        value2->key = value->key;
        deserialize_cstring(bson_info, value);
        deserialize_cstring(bson_info, value2);
        break;

        case BSON_DBPOINTER:
        value2->key = value->key;
        deserialize_string(bson_info, value);
        value2->size = 12;
        memcpy((void*)value2->dataV,(void*) &(bson_info->buffer[bson_info->position]), value2->size);
        //value2->data = &(bson_info->buffer[bson_info->position]);
        bson_info->position += 12;
        bson_info->size -= 12;
        break;

        case BSON_JS:
        deserialize_string(bson_info, value);
        break;

        case BSON_SYMBOL:
        deserialize_string(bson_info, value);
        break;

        case BSON_JS_CODE:
        bson_info->position += 4;                                               /* skip int32_t size */
        bson_info->size -= 4;                                                   /* skip int32_t size */
        deserialize_string(bson_info, value);
        value2->key = value->key;
        deserialize_document(bson_info, value2);
        break;

        case BSON_INT32:
        value->size = 4;
        memcpy((void*)value->dataV,(void*) &(bson_info->buffer[bson_info->position]), value->size);
        //value->data = &(bson_info->buffer[bson_info->position]);
        //value->size = 4;
        bson_info->position += 4;
        bson_info->size -= 4;
        break;

        case BSON_TIMESTAMP:
        value->size = 8;
        memcpy((void*)value->dataV,(void*) &(bson_info->buffer[bson_info->position]), value->size);
        //value->data = &(bson_info->buffer[bson_info->position]);
        bson_info->position += 8;
        bson_info->size -= 8;
        break;

        case BSON_INT64:
        value->size = 8;
        //value->data = &(bson_info->buffer[bson_info->position]);
        memcpy((void*)value->dataV,(void*) &(bson_info->buffer[bson_info->position]), value->size);
        bson_info->position += 8;
        bson_info->size -= 8;
        break;

        case BSON_MIN:
         /* finished */
        break;

        case BSON_MAX:
        /* finished */
        break;

        default:
        return -1;
    };

    return 1;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif