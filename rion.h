#ifndef __rion_h
#define __rion_h
//   rion.h : Raw Internet Object Notation
//
//   Version : @(#) 1.0
//   Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#include <stdint.h>
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define RIONPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define RIONPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define RIONPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define RIONPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define RIONPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define RION_BOOL_NULL 0u                                                       // define a Tiny as a boolean
#define RION_BOOL_TRUE 1u
#define RION_BOOL_FALSE 2u

typedef enum {
  RION_Bool_Null = 0u,
  RION_Bool_True = 1u,
  RION_Bool_False = 2u
} rionBooleanClass;                                                             // boolean class

typedef enum {
  RION_Bytes = 0u,
  RION_Boolean = 1u,
  RION_Int64Pos = 2u,
  RION_Int64Neg = 3u,
  RION_Float = 4u,
  RION_UTF8 = 5u,
  RION_UTF8Short = 6u,
  RION_UTCDateTime = 7u,
  RION_Res1 = 8u,
  RION_Res2 = 9u,
  RION_Array = 10u,
  RION_Table = 11u,
  RION_Object = 12u,
  RION_Key = 13u,                                                               // e.g. 0x0D for key
  RION_KeyShort = 14u,
  RION_Extended = 15u
} rionCoreIonFieldTypeClass;                                                    // ion field class

typedef enum {
  RION_Complex_Type_Id = 1u,                                                    // encoding Extended Normal Contains a longer complex type id, e.g. a Java class name
  RION_Copy = 2u,                                                               // encoding Extended Short Represents a reference to an RION field located earlier in the same RION data. Used e.g. to represent an object reference to another object, so circular object references can be represented
  RION_Reference_Back = 3u,                                                     // encoding Extended Short Represents a reference to an RION field located earlier in the same RION data. Used e.g. to represent an object reference to another object, so circular object references can be represented
  RION_Reference_Forward = 4u,                                                  // encoding Extended Short Represents a reference to an RION field located later in the same RION data. Used e.g. to represent an object reference to another object, so circular object references can be represented.
  RION_Cache_Reference = 5u,                                                    // encoding Extended Normal Represents a reference to an RION field located in the cache of the other party communicating via the same network connection. Intended to be used in conjunction with IAP
  RION_Cache_Reference_Short = 6u,                                              // encoding Extended Short Represents a short reference (key <= 15 bytes) to an RION field located in the cache of the other party communicating via the same network connection. Intended to be used in conjunction with IAP
  RION_UTC_Time = 7u                                                            // encoding Extended Short Contains a time of day in UTC time (no time zones), or a duration
} rionExtendedFieldTypeClass;                                                   // extended field class

typedef enum {
  RION_Normal,                                                                  // ion field[4]len[4] len[1..15] data[]
  RION_Short,                                                                   // ion field[4]len[4] len[15]
  RION_Tiny,                                                                    // ion field[4]len[4]
  RION_Extended_Normal,                                                         // ion field[4]len[4] type[1..2] len[1..15] data[]
  RION_Extended_Short,                                                          // ion field[4]len[4] type[1..2] len[1..15]
  RION_Extended_Tiny                                                            // ion field[4]len[4] type[1..2]
} rionEncodingVariationClass;                                                   // encoding variation class

typedef union {
  float32_t float32;                                                            // 32 bit
  float64_t float64;                                                            // 64 bit
} rionFloatTypeClass;                                                           // float type union either type is defined as RION float

#ifdef BIG_ENDIAN

#if defined(D_FT900)
typedef struct RIONPACKED {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits
   uint8_t length[15u];                                                         // A range of 0 to 15 length bytes can represent a field length range of 0 to 2^120 bytes
   uint8_t valueBytes[];                                                        // a RION field can contain values that are up to 2^120 bytes long.
} rionBytesField_t;
#else
RIONPACKED(
typedef struct {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits
   uint8_t length[15u];                                                         // A range of 0 to 15 length bytes can represent a field length range of 0 to 2^120 bytes
   uint8_t valueBytes[];                                                        // a RION field can contain values that are up to 2^120 bytes long.
}) rionBytesField_t;                                                            // rion bytes field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // type RION_Boolean
   rionBooleanClass bool_val : 4u;                                              // boolean type 4 MSB bits
} rionBooleanField_t;
#else
RIONPACKED(
typedef struct {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // type RION_Boolean
   rionBooleanClass bool_val : 4u;                                              // boolean type 4 MSB bits
}) rionBooleanField_t;                                                          // rion boolean field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_Int64Pos
   uint64_t number;
} rionInt64Field_t;
#else
RIONPACKED(
typedef struct {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_Int64Pos
   uint64_t number;
}) rionInt64Field_t;                                                            // rion int64 field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_Float
   rionFloatTypeClass number;
} rionFloatField_t;
#else
RIONPACKED(
typedef struct {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_Float
   rionFloatTypeClass number;
}) rionFloatField_t;                                                            // rion float field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTF8
   unsigned char utf8[ ];                                                       // string of utf8 charactures
} rionKeyorUTF8Field_t;
#else
RIONPACKED(
typedef struct {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTF8
   unsigned char utf8[ ];                                                       // string of utf8 charactures
}) rionKeyorUTF8Field_t;                                                        // rion utf8 or key field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTF8Short
   unsigned char utf8[15u];                                                     // string of utf8 Short charactures
} rionKeyorUTF8ShortField_t;
#else
RIONPACKED(
typedef struct {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTF8Short
   unsigned char utf8[15u];                                                     // string of utf8 Short charactures
}) rionKeyorUTF8ShortField_t;                                                   // rion utf8 or key Short field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTCDateTime
   uint16_t year;                                                               // utc date time
   uint8_t month;                                                               // utc date time
   uint8_t day;                                                                 // utc date time
   uint8_t hour;                                                                // utc date time
   uint8_t min;                                                                 // utc date time
   uint8_t sec;                                                                 // utc date time
   uint16_t millis;                                                             // milliseconds 16bit
   uint64_t micros : 24u;                                                       // microseconds 24bit
   uint64_t nanos : 32u;                                                        // nanoseconds 32bit
   uint64_t spare : 8u;                                                         // 8bits spare at end of structure
} rionUTCDateTimeField_t;
#else
RIONPACKED(
typedef struct {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTCDateTime
   uint16_t year;                                                               // utc date time
   uint8_t month;                                                               // utc date time
   uint8_t day;                                                                 // utc date time
   uint8_t hour;                                                                // utc date time
   uint8_t min;                                                                 // utc date time
   uint8_t sec;                                                                 // utc date time
   uint16_t millis;                                                             // milliseconds 16bit
   uint64_t micros : 24u;                                                       // microseconds 24bit
   uint64_t nanos : 32u;                                                        // nanoseconds 32bit
   uint64_t spare : 8u;                                                         // 8bits spare at end of structure
}) rionUTCDateTimeField_t;                                                      // rion utc date time field
#endif


typedef union {
  rionKeyorUTF8Field_t Key;                                                     // utf8 encoded key
  rionKeyorUTF8ShortField_t shortKey;                                           // utf8 short encoded key
} rionColumnKeyTypeClass;                                                       // either key is used from this union

#if defined(D_FT900)
typedef struct RIONPACKED {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass table_type : 4u;                                   // field type 4 MSB bits  RION_Table
   uint8_t len_of_field : 4u;                                                   // Length of field - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass int64_type : 4u;                                   // field type 4 MSB bits  RION_Int64
   int64_t num_of_rows;                                                         // number of rows
   rionColumnKeyTypeClass column_key[];                                         // column key number = number of rows
   rionInt64Field_t dataVals[];
} rionTableField_t;
#else
RIONPACKED(
typedef struct {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass table_type : 4u;                                   // field type 4 MSB bits  RION_Table
   uint8_t len_of_field : 4u;                                                   // Length of field - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass int64_type : 4u;                                   // field type 4 MSB bits  RION_Int64
   int64_t num_of_rows;                                                         // number of rows
   rionColumnKeyTypeClass column_key[];                                         // column key number = number of rows
   rionInt64Field_t dataVals[];
}) rionTableField_t;                                                            // rion table field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
  rionKeyorUTF8ShortField_t keyShort;                                           // key
  rionInt64Field_t int64;                                                       // int64_t
  rionTableField_t table;                                                       // table
} rionObject_t;
#else
RIONPACKED(
typedef struct {
  rionKeyorUTF8ShortField_t keyShort;                                           // key
  rionInt64Field_t int64;                                                       // int64_t
  rionTableField_t table;                                                       // table
}) rionObject_t;                                                                // rion object example
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass object_type : 4u;                                  // field type 4 MSB bits  RION_Object
   rionObject_t object[];                                                       // object
} rionObjectField_t;
#else
RIONPACKED(
typedef struct {
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass object_type : 4u;                                  // field type 4 MSB bits  RION_Object
   rionObject_t object[];                                                       // object
}) rionObjectField_t;                                                           // rion object field
#endif

#else

#if defined(D_FT900)
typedef struct RIONPACKED {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   uint8_t length[15u];                                                         // A range of 0 to 15 length bytes can represent a field length range of 0 to 2^120 bytes
   uint8_t valueBytes[];                                                        // a RION field can contain values that are up to 2^120 bytes long.
} rionBytesField_t;
#else
RIONPACKED(
typedef struct {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   uint8_t length[15u];                                                         // A range of 0 to 15 length bytes can represent a field length range of 0 to 2^120 bytes
   uint8_t valueBytes[];                                                        // a RION field can contain values that are up to 2^120 bytes long.
}) rionBytesField_t;                                                            // rion bytes field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   rionBooleanClass bool_val : 4u;                                              // boolean val 4 MSB bits
   rionCoreIonFieldTypeClass field_type : 4u;                                   // RION_Boolean
} rionBooleanField_t;
#else
RIONPACKED(
typedef struct {
   rionBooleanClass bool_val : 4u;                                              // boolean val 4 MSB bits
   rionCoreIonFieldTypeClass field_type : 4u;                                   // RION_Boolean
}) rionBooleanField_t;                                                          // rion boolean field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits RION_Int64Pos
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   uint64_t number;
} rionInt64Field_t;
#else
RIONPACKED(
typedef struct {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits RION_Int64Pos
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   uint64_t number;
}) rionInt64Field_t;                                                            // rion int64 field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_Float
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionFloatTypeClass number;
} rionFloatField_t;
#else
RIONPACKED(
typedef struct {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_Float
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionFloatTypeClass number;
}) rionFloatField_t;                                                            // rion float field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTF8
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   unsigned char utf8[ ];                                                       // string of utf8 charactures
} rionKeyorUTF8Field_t;
#else
RIONPACKED(
typedef struct {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTF8
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   unsigned char utf8[ ];                                                       // string of utf8 charactures
}) rionKeyorUTF8Field_t;                                                       // rion utf8 or Key field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTF8Short
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   unsigned char utf8[15u];                                                     // string of utf8 charactures
} rionKeyorUTF8ShortField_t;
#else
RIONPACKED(
typedef struct {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTF8Short
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   unsigned char utf8[15u];                                                     // string of utf8 charactures
}) rionKeyorUTF8ShortField_t;                                                   // rion key or utf8 short field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTCDateTime
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   uint16_t year;                                                               // utc date time
   uint8_t month;                                                               // utc date time
   uint8_t day;                                                                 // utc date time
   uint8_t hour;                                                                // utc date time
   uint8_t min;                                                                 // utc date time
   uint8_t sec;                                                                 // utc date time
   uint16_t millis;                                                             // milliseconds 16bit
   uint64_t micros : 24u;                                                       // microseconds 24bit
   uint64_t nanos : 32u;                                                        // nanoseconds 32bit
   uint64_t spare : 8u;                                                         // 8bits spare at end of structure
} rionUTCDateTimeField_t;
#else
RIONPACKED(
typedef struct {
   rionCoreIonFieldTypeClass field_type : 4u;                                   // field type 4 MSB bits  RION_UTCDateTime
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   uint16_t year;                                                               // utc date time
   uint8_t month;                                                               // utc date time
   uint8_t day;                                                                 // utc date time
   uint8_t hour;                                                                // utc date time
   uint8_t min;                                                                 // utc date time
   uint8_t sec;                                                                 // utc date time
   uint16_t millis;                                                             // milliseconds 16bit
   uint64_t micros : 24u;                                                       // microseconds 24bit
   uint64_t nanos : 32u;                                                        // nanoseconds 32bit
   uint64_t spare : 8u;                                                         // 8bits spare at end of structure
}) rionUTCDateTimeField_t;                                                      // rion utc date time field
#endif

typedef union {
  rionKeyorUTF8Field_t Key;
  rionKeyorUTF8ShortField_t shortKey;
} rionColumnKeyTypeClass;                                                       // either key type is used in the table type

#if defined(D_FT900)
typedef struct RIONPACKED {
   rionCoreIonFieldTypeClass table_type : 4u;                                   // field type 4 MSB bits  RION_Table
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass int64_type : 4u;                                   // field type 4 MSB bits  RION_Int64
   uint8_t len_of_field : 4u;                                                     // Length of field
   int64_t num_of_rows;                                                         // number of rows
   rionColumnKeyTypeClass column_key[];                                         // column key number = number of rows
   rionInt64Field_t dataVals[];
} rionTableField_t;
#else
RIONPACKED(
typedef struct {
   rionCoreIonFieldTypeClass table_type : 4u;                                   // field type 4 MSB bits  RION_Table
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionCoreIonFieldTypeClass int64_type : 4u;                                   // field type 4 MSB bits  RION_Int64
   uint8_t len_of_field : 4u;                                                     // Length of field
   int64_t num_of_rows;                                                         // number of rows
   rionColumnKeyTypeClass column_key[];                                         // column key number = number of rows
   rionInt64Field_t dataVals[];
}) rionTableField_t;                                                            // rion table field
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
  rionKeyorUTF8ShortField_t keyShort;                                           // key
  rionInt64Field_t int64;                                                       // int64_t
  rionTableField_t table;                                                       // table
} rionObject_t;
#else
RIONPACKED(
typedef struct {
  rionKeyorUTF8ShortField_t keyShort;                                           // key
  rionInt64Field_t int64;                                                       // int64_t
  rionTableField_t table;                                                       // table
}) rionObject_t;                                                                // rion object example
#endif

#if defined(D_FT900)
typedef struct RIONPACKED {
   rionCoreIonFieldTypeClass object_type : 4u;                                  // field type 4 MSB bits  RION_Object
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionObject_t object[];                                                       // object
} rionObjectField_t;
#else
RIONPACKED(
typedef struct {
   rionCoreIonFieldTypeClass object_type : 4u;                                  // field type 4 MSB bits  RION_Object
   uint8_t len_of_len : 4u;                                                     // Length of length - number of length bytes (4 LSB bits)
   rionObject_t object[];                                                       // object
}) rionObjectField_t;                                                           // rion object field
#endif

#endif

#ifdef __cplusplus
}
#endif

#endif