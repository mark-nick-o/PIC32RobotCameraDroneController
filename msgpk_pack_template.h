#ifndef MSGPACK_PACK_H
#define MSGPACK_PACK_H
/*
 * MessagePack packing routine template
 *
 * Copyright (C) 2008-2010 FURUHASHI Sadayuki
 * Ported : Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
 *
 *    Distributed under the Boost Software License, Version 1.0.
 *    (See accompanying file LICENSE_1_0.txt or copy at
 *    http://www.boost.org/LICENSE_1_0.txt)
 */

/**
 * @defgroup msgpack_pack Serializer
 * @ingroup msgpack
 * @{
 */
 
#ifdef __cplusplus
extern "C" {
#endif

#ifndef size_t
#define size_t int32_t
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define MSGPKPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define MSGPKPACKED __attribute__((packed))                                 /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define MSGPKPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define MSGPKPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define MSGPKPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif
 
MSGPKPACKED(
typedef struct msgpack_packer_write
{
    void* dataV;
    const char* buf;
    size_t len;
}) msgpack_packer_write;

typedef int16_t (*msgpack_packer_write_t);

MSGPKPACKED(
typedef struct msgpack_packer 
{
    void* dataV;
    msgpack_packer_write callback;
}) msgpack_packer;

MSGPKPACKED(
typedef struct msgpack_timestamp
{
    uint32_t tv_nsec;
    uint32_t tv_sec;
}) msgpack_timestamp;

#if defined(_CPU_BIG_ENDIAN)                                                    /* set the library endian from the MCU definition */
#define MSGPACK_ENDIAN_BIG_BYTE
#else
#define MSGPACK_ENDIAN_LITTLE_BYTE
#endif

#if defined(MSGPACK_ENDIAN_LITTLE_BYTE)                                         /* little endian take LSB */
#define TAKE8_8(d)  ((uint8_t*)&d)[0u]                                          /* LSB from a 8bit word */
#define TAKE8_16(d) ((uint8_t*)&d)[0u]                                          /* LSB from a 16bit word */
#define TAKE8_32(d) ((uint8_t*)&d)[0u]                                          /* LSB from a 32bit word */
#define TAKE8_64(d) ((uint8_t*)&d)[0u]                                          /* LSB from a 64bit word */
#elif defined(MSGPACK_ENDIAN_BIG_BYTE)                                                   /* big endian take MSB */
#define TAKE8_8(d)  ((uint8_t*)&d)[0u]                                          /* MSB from a 8bit word */
#define TAKE8_16(d) ((uint8_t*)&d)[1u]                                          /* MSB from a 16bit word */
#define TAKE8_32(d) ((uint8_t*)&d)[3u]                                          /* MSB from a 32bit word */
#define TAKE8_64(d) ((uint8_t*)&d)[7u]                                          /* MSB from a 64bit word */
#else
#error msgpack-c supports only big endian and little endian
#endif


#define msgpack_pack_inline_func msgpack_pack                                   // define the wrapper name as per object.c

#ifndef msgpack_pack_inline_func
#error msgpack_pack_inline_func template is not defined
#endif

#define msgpack_pack_user msgpack_packer*                                       // define the message packer as the struct we use

#ifndef msgpack_pack_user
#error msgpack_pack_user type is not defined
#endif

#define msgpack_pack_append_buffer memcpy                                       /* alias msgpack_pack_append_buffer as memcpy */

#ifndef msgpack_pack_append_buffer
#error msgpack_pack_append_buffer callback is not defined
#endif

//#if defined(_MSC_VER)
//#   pragma warning(push)
//#   pragma warning(disable : 4204)   /* nonstandard extension used: non-constant aggregate initializer */
//#endif
#if defined(_CPU_BIG_ENDIAN)                                                    /* if we are big endian numbers */
#define _msgpack_store16(p, a) \
   ((uint8_t *)(p))[0u] = ((uint16_t)(a) >> 8u) & 0xFFU, \
   ((uint8_t *)(p))[1u] = ((uint16_t)(a) >> 0u) & 0xFFU
#define _msgpack_store32(p, a) \
   ((uint8_t *)(p))[0u] = ((uint32_t)(a) >> 24u) & 0xFFU, \
   ((uint8_t *)(p))[1u] = ((uint32_t)(a) >> 16u) & 0xFFU, \
   ((uint8_t *)(p))[2u] = ((uint32_t)(a) >> 8u) & 0xFFU, \
   ((uint8_t *)(p))[3u] = ((uint32_t)(a) >> 0u) & 0xFFU
#define _msgpack_store64(p, a) \
   ((uint8_t *)(p))[0u] = ((uint64_t)(a) >> 56u) & 0xFFU, \
   ((uint8_t *)(p))[1u] = ((uint64_t)(a) >> 48u) & 0xFFU, \
   ((uint8_t *)(p))[2u] = ((uint64_t)(a) >> 40u) & 0xFFU, \
   ((uint8_t *)(p))[3u] = ((uint64_t)(a) >> 32u) & 0xFFU, \
   ((uint8_t *)(p))[4u] = ((uint64_t)(a) >> 24u) & 0xFFU, \
   ((uint8_t *)(p))[5u] = ((uint64_t)(a) >> 16u) & 0xFFU, \
   ((uint8_t *)(p))[6u] = ((uint64_t)(a) >> 8u) & 0xFFU, \
   ((uint8_t *)(p))[7u] = ((uint64_t)(a) >> 0u) & 0xFFU
#else                                                                           /* if we are little endian numbers */
#define _msgpack_store16(p, a) \
   ((uint8_t *)(p))[0u] = ((uint16_t)(a) >> 0u) & 0xFFU, \
   ((uint8_t *)(p))[1u] = ((uint16_t)(a) >> 8u) & 0xFFU
#define _msgpack_store32(p, a) \
   ((uint8_t *)(p))[0u] = ((uint32_t)(a) >> 0u) & 0xFFU, \
   ((uint8_t *)(p))[1u] = ((uint32_t)(a) >> 8u) & 0xFFU, \
   ((uint8_t *)(p))[2u] = ((uint32_t)(a) >> 16u) & 0xFFU, \
   ((uint8_t *)(p))[3u] = ((uint32_t)(a) >> 24u) & 0xFFU
#define _msgpack_store64(p, a) \
   ((uint8_t *)(p))[0u] = ((uint64_t)(a) >> 0u) & 0xFFU, \
   ((uint8_t *)(p))[1u] = ((uint64_t)(a) >> 8u) & 0xFFU, \
   ((uint8_t *)(p))[2u] = ((uint64_t)(a) >> 16u) & 0xFFU, \
   ((uint8_t *)(p))[3u] = ((uint64_t)(a) >> 24u) & 0xFFU, \
   ((uint8_t *)(p))[4u] = ((uint64_t)(a) >> 32u) & 0xFFU, \
   ((uint8_t *)(p))[5u] = ((uint64_t)(a) >> 40u) & 0xFFU, \
   ((uint8_t *)(p))[6u] = ((uint64_t)(a) >> 48u) & 0xFFU, \
   ((uint8_t *)(p))[7u] = ((uint64_t)(a) >> 56u) & 0xFFU
#endif

/*
 * Integer
 */

#define msgpack_pack_real_uint8(x, d) \
do { \
    unsigned char buf[2u]; \
    if(d < (1<<7)) { \
        /* fixnum */ \
        msgpack_pack_append_buffer(x, &TAKE8_8(d), 1); \
    } else { \
        /* unsigned 8 */ \
        /* not work buf[2] = {0xcc, TAKE8_8(d)}; */ \
        buf[0u] = 0xCCU;  \
        buf[1u] = TAKE8_8(d); \
        msgpack_pack_append_buffer(x, buf, 2); \
    } \
} while(0)

void f_msgpack_pack_real_uint8(msgpack_pack_user x, int64_t d)                  /* brought out to function to test first */
{
    unsigned char buf[2u];
    if(d < (1u<<7u)) 
    {
        /* fixnum */
        msgpack_pack_append_buffer(x, &TAKE8_8(d), 1);
    } 
    else 
    {
        /* unsigned 8 */
        /* not allowed buf[2u] = {0xccu, TAKE8_8(d)}; */
        buf[0u] = 0xCCU;
        buf[1u] = TAKE8_8(d);
        msgpack_pack_append_buffer(x, buf, 2);
    }
}

#define msgpack_pack_real_uint16(x, d) \
do { \
    unsigned char buf[3u]; \
    if(d < (1u<<7u)) { \
        /* fixnum */ \
        msgpack_pack_append_buffer(x, &TAKE8_16(d), 1); \
    } else if(d < (1u<<8u)) { \
        /* unsigned 8 */ \
        /* unsigned char buf[2] = {0xcc, TAKE8_16(d)}; \ doesnt work */ \
        buf[0u] = 0xCCU;      \
        buf[1u] = TAKE8_16(d); \
        msgpack_pack_append_buffer(x, buf, 2); \
    } else { \
        /* unsigned 16 */ \
        /* buf[0] = 0xcd; _msgpack_store16(&buf[1], (uint16_t)d); \ no work */ \
        buf[0u] = 0xCDU; \
        _msgpack_store16(&buf[1], (uint16_t)d); \
        msgpack_pack_append_buffer(x, buf, 3); \
    } \
} while(0)

#define msgpack_pack_real_uint32(x, d) \
do { \
    unsigned char buf[5u]; \
    if(d < (1UL<<8UL)) { \
        if(d < (1UL<<7UL)) { \
            /* fixnum */ \
            msgpack_pack_append_buffer(x, &TAKE8_32(d), 1); \
        } else { \
            /* unsigned 8 */ \
            /* unsigned char buf[2] = {0xcc, TAKE8_32(d)}; \ no work */ \
            buf[0u] = 0xCCU; \
            buf[1u] = TAKE8_32(d); \
            msgpack_pack_append_buffer(x, buf, 2); \
        } \
    } else { \
        if(d < (1UL<<16UL)) { \
            /* unsigned 16 */ \
            /* buf[0] = 0xcd; _msgpack_store16(&buf[1], (uint16_t)d); \ */ \
            buf[0u] = 0xCDU; \
            buf[1u] = _msgpack_store16(&buf[1u], (uint16_t)d); \
            msgpack_pack_append_buffer(x, buf, 3); \
        } else { \
            /* unsigned 32 */ \
            /* buf[0] = 0xce; _msgpack_store32(&buf[1], (uint32_t)d); \ */ \
            buf[0u] = 0xCEU; \
            buf[1u] = _msgpack_store32(&buf[1u], (uint32_t)d); \
            msgpack_pack_append_buffer(x, buf, 5); \
        } \
    } \
} while(0)

#define msgpack_pack_real_uint64(x, d) \
do { \
    unsigned char buf[9u]; \
    if(d < (1ULL<<8ULL)) { \
        if(d < (1ULL<<7ULL)) { \
            /* fixnum */ \
            msgpack_pack_append_buffer(x, &TAKE8_64(d), 1); \
        } else { \
            /* unsigned 8 */ \
            /* unsigned char buf[2] = {0xcc, TAKE8_64(d)}; \ no work */ \
            buf[0u] = 0xCCU; \
            buf[1u] = TAKE8_64(d); \
            msgpack_pack_append_buffer(x, buf, 2); \
        } \
    } else { \
        if(d < (1ULL<<16ULL)) { \
            /* unsigned 16 */ \
            /* buf[0] = 0xcd; _msgpack_store16(&buf[1], (uint16_t)d); \ no work */ \
            buf[0u] = 0xCDU; \
            buf[1u] = _msgpack_store16(&buf[1], (uint16_t)d); \
            msgpack_pack_append_buffer(x, buf, 3); \
        } else if(d < (1ULL<<32ULL)) { \
            /* unsigned 32 */ \
            /* buf[0] = 0xce; _msgpack_store32(&buf[1], (uint32_t)d); \ no work */ \
            buf[0u] = 0xCEU; \
            buf[1u] = _msgpack_store32(&buf[1],(uint32_t) d); \
            msgpack_pack_append_buffer(x, buf, 5); \
        } else { \
            /* unsigned 64 */ \
            /* buf[0] = 0xcf; _msgpack_store64(&buf[1], d); \ no work */ \
            buf[0u] = 0xCFU; \
            buf[1u] = _msgpack_store64(&buf[1], d); \
            msgpack_pack_append_buffer(x, buf, 9); \
        } \
    } \
} while(0)

#define msgpack_pack_real_int8(x, d) \
do { \
    unsigned char buf[2u]; \
    if(d < -(1<<5)) { \
        /* signed 8 */ \
        buf[0u] = 0xd0u; \
        buf[1u] = TAKE8_8(d); \
        msgpack_pack_append_buffer(x, buf, 2); \
    } else { \
        /* fixnum */ \
        msgpack_pack_append_buffer(x, &TAKE8_8(d), 1); \
    } \
} while(0)

void f_msgpack_pack_real_int8(msgpack_pack_user x, int64_t d)                   /* brought out to function to test first */
{
    unsigned char buf[2u];
    if(d < -(1<<5)) {
        /* signed 8 */
        buf[0u] = 0xd0u;
        buf[1u] = TAKE8_8(d);
        msgpack_pack_append_buffer(x, buf, 2);
    } else {
        /* fixnum */
        msgpack_pack_append_buffer(x, &TAKE8_8(d), 1);
    }
}

#define msgpack_pack_real_int16(x, d) \
do { \
    unsigned char buf[3u]; \
    if(d < -(1<<5)) { \
        if(d < -(1<<7)) { \
            /* signed 16 */ \
            buf[0u] = 0xd1u;  \
            _msgpack_store16(&buf[1u], (int16_t)d); \
            msgpack_pack_append_buffer(x, buf, 3); \
        } else { \
            /* signed 8 */ \
            buf[0u] = 0xd0u; \
            buf[1u] = TAKE8_16(d); \
            msgpack_pack_append_buffer(x, buf, 2); \
        } \
    } else if(d < (1<<7)) { \
        /* fixnum */ \
        msgpack_pack_append_buffer(x, &TAKE8_16(d), 1); \
    } else { \
        if(d < (1<<8)) { \
            /* unsigned 8 */ \
            buf[0u] = 0xccu; \
            buf[1u] = TAKE8_16(d); \
            msgpack_pack_append_buffer(x, buf, 2); \
        } else { \
            /* unsigned 16 */ \
            buf[0u] = 0xcdu; \
            _msgpack_store16(&buf[1u], (uint16_t)d); \
            msgpack_pack_append_buffer(x, buf, 3); \
        } \
    } \
} while(0)

void f_msgpack_pack_real_int16(msgpack_pack_user x, int64_t d)
 {
    unsigned char buf[3u];
    if(d < -(1<<5)) {
        if(d < -(1<<7)) {
            /* signed 16 */
            buf[0u] = 0xd1u;
            _msgpack_store16(&buf[1u], (int16_t)d);
            msgpack_pack_append_buffer(x, buf, 3);
        } else {
            /* signed 8 */
            buf[0u] = 0xd0u;
            buf[1u] = TAKE8_16(d);
            msgpack_pack_append_buffer(x, buf, 2);
        }
    } else if(d < (1<<7)) {
        /* fixnum */
        msgpack_pack_append_buffer(x, &TAKE8_16(d), 1);
    } else {
        if(d < (1<<8)) {
            /* unsigned 8 */
            buf[0u] = 0xccu;
            buf[1u] = TAKE8_16(d);
            msgpack_pack_append_buffer(x, buf, 2);
        } else {
            /* unsigned 16 */
            buf[0u] = 0xcdu;
            _msgpack_store16(&buf[1u], (uint16_t)d);
            msgpack_pack_append_buffer(x, buf, 3);
        }
    }
}

#define msgpack_pack_real_int32(x, d) \
do { \
    unsigned char buf[5u]; \
    if(d < -(1<<5)) { \
        if(d < -(1<<15)) { \
            /* signed 32 */ \
            buf[0u] = 0xd2u; \
            _msgpack_store32(&buf[1u], (int32_t)d); \
            msgpack_pack_append_buffer(x, buf, 5); \
        } else if(d < -(1<<7)) { \
            /* signed 16 */ \
            buf[0u] = 0xd1u; \
            _msgpack_store16(&buf[1u], (int16_t)d); \
            msgpack_pack_append_buffer(x, buf, 3); \
        } else { \
            /* signed 8 */ \
            buf[0u] = 0xd0u; \
            buf[1u] = TAKE8_32(d); \
            msgpack_pack_append_buffer(x, buf, 2); \
        } \
    } else if(d < (1<<7)) { \
        /* fixnum */ \
        msgpack_pack_append_buffer(x, &TAKE8_32(d), 1); \
    } else { \
        if(d < (1<<8)) { \
            /* unsigned 8 */ \
            buf[0u] = 0xccu; \
            buf[1u] = TAKE8_32(d); \
            msgpack_pack_append_buffer(x, buf, 2); \
        } else if(d < (1<<16)) { \
            /* unsigned 16 */ \
            buf[0u] = 0xcdu; \
            _msgpack_store16(&buf[1u], (uint16_t)d); \
            msgpack_pack_append_buffer(x, buf, 3); \
        } else { \
            /* unsigned 32 */ \
            buf[0u] = 0xce; \
            _msgpack_store32(&buf[1u], (uint32_t)d); \
            msgpack_pack_append_buffer(x, buf, 5); \
        } \
    } \
} while(0)

#define msgpack_pack_real_int64(x, d) \
do { \
    unsigned char buf[9u]; \
    if(d < -(1LL<<5)) { \
        if(d < -(1LL<<15)) { \
            if(d < -(1LL<<31)) { \
                /* signed 64 */ \
                buf[0u] = 0xd3u; \
                 _msgpack_store64(&buf[1], d); \
                msgpack_pack_append_buffer(x, buf, 9); \
            } else { \
                /* signed 32 */ \
                buf[0u] = 0xd2u; \
                _msgpack_store32(&buf[1u], (int32_t)d); \
                msgpack_pack_append_buffer(x, buf, 5); \
            } \
        } else { \
            if(d < -(1<<7)) { \
                /* signed 16 */ \
                buf[0u] = 0xd1u; \
                 _msgpack_store16(&buf[1u], (int16_t)d); \
                msgpack_pack_append_buffer(x, buf, 3); \
            } else { \
                /* signed 8 */ \
                buf[0u] = 0xd0u; \
                buf[1u] = TAKE8_64(d); \
                msgpack_pack_append_buffer(x, buf, 2); \
            } \
        } \
    } else if(d < (1<<7)) { \
        /* fixnum */ \
        msgpack_pack_append_buffer(x, &TAKE8_64(d), 1); \
    } else { \
        if(d < (1LL<<16)) { \
            if(d < (1<<8)) { \
                /* unsigned 8 */ \
                buf[0u] = 0xccu; \
                buf[1u] = TAKE8_64(d); \
                msgpack_pack_append_buffer(x, buf, 2); \
            } else { \
                /* unsigned 16 */ \
                buf[0u] = 0xcdu;  \
                buf[1u] = _msgpack_store16(&buf[1u], (uint16_t)d); \
                msgpack_pack_append_buffer(x, buf, 3); \
            } \
        } else { \
            if(d < (1LL<<32)) { \
                /* unsigned 32 */ \
                buf[0u] = 0xceu; \
                 _msgpack_store32(&buf[1u], (uint32_t)d); \
                msgpack_pack_append_buffer(x, buf, 5); \
            } else { \
                /* unsigned 64 */ \
                buf[0u] = 0xcfu; \
                _msgpack_store64(&buf[1u], d); \
                msgpack_pack_append_buffer(x, buf, 9); \
            } \
        } \
    } \
} while(0)


/*#define msgpack_pack_inline_func_fixint msgpack_pack */

/*#ifdef msgpack_pack_inline_func_fixint */

void msgpack_pack_inline_func_fixint_uint8(msgpack_pack_user x, uint8_t d)
{
    unsigned char buf[2u] = {0xccu, 0x00u};
    buf[1u] = TAKE8_8(d);
    msgpack_pack_append_buffer(x, buf, 2);
}

void msgpack_pack_inline_func_fixint_uint16(msgpack_pack_user x, uint16_t d)
{
    unsigned char buf[3u];
    buf[0u] = 0xcdu;
    _msgpack_store16(&buf[1u], d);
    msgpack_pack_append_buffer(x, buf, 3);
}

void msgpack_pack_inline_func_fixint_uint32(msgpack_pack_user x, uint32_t d)
{
    unsigned char buf[5u];
    buf[0u] = 0xceu;
    _msgpack_store32(&buf[1u], d);
    msgpack_pack_append_buffer(x, buf, 5);
}

void msgpack_pack_inline_func_fixint_uint64(msgpack_pack_user x, uint64_t d)
{
    unsigned char buf[9];
    buf[0u] = 0xcfu;
    _msgpack_store64(&buf[1u], d);
    msgpack_pack_append_buffer(x, buf, 9);
}

void msgpack_pack_inline_func_fixint_int8(msgpack_pack_user x, int8_t d)
{
    unsigned char buf[2u] = {0xd0u, 0x00u };
    buf[1u]=TAKE8_8(d);
    msgpack_pack_append_buffer(x, buf, 2);
}

void msgpack_pack_inline_func_fixint_int16(msgpack_pack_user x, int16_t d)
{
    unsigned char buf[3u];
    buf[0u] = 0xd1u;
    _msgpack_store16(&buf[1u], d);
    msgpack_pack_append_buffer(x, buf, 3);
}

void msgpack_pack_inline_func_fixint_int32(msgpack_pack_user x, int32_t d)
{
    unsigned char buf[5u];
    buf[0u] = 0xd2;
    _msgpack_store32(&buf[1u], d);
    msgpack_pack_append_buffer(x, buf, 5);
}

void msgpack_pack_inline_func_fixint_int64(msgpack_pack_user x, int64_t d)
{
    unsigned char buf[9u];
    buf[0u] = 0xd3u;
    _msgpack_store64(&buf[1u], d);
    msgpack_pack_append_buffer(x, buf, 9);
}


/*#undef msgpack_pack_inline_func_fixint */
/* #endif                                                                       msgpack_pack_inline_func_fixint */


void msgpack_pack_inline_func_uint8(msgpack_pack_user x, uint8_t d)
{
    msgpack_pack_real_uint8(x, d);
}


void msgpack_pack_inline_func_uint16(msgpack_pack_user x, uint16_t d)
{
    msgpack_pack_real_uint16(x, d);
}

void msgpack_pack_inline_func_uint32(msgpack_pack_user x, uint32_t d)
{
    msgpack_pack_real_uint32(x, d);
}

void msgpack_pack_inline_func_uint64(msgpack_pack_user x, uint64_t d)
{
    msgpack_pack_real_uint64(x, d);
}

void msgpack_pack_inline_func_int8(msgpack_pack_user x, int8_t d)
{
    msgpack_pack_real_int8(x, d);
}

void msgpack_pack_inline_func_int16(msgpack_pack_user x, int16_t d)
{
    msgpack_pack_real_int16(x, d);
}

void msgpack_pack_inline_func_int32(msgpack_pack_user x, int32_t d)
{
    msgpack_pack_real_int32(x, d);
}

void msgpack_pack_inline_func_int64(msgpack_pack_user x, int64_t d)
{
    msgpack_pack_real_int64(x, d);
}

void msgpack_pack_inline_func_char(msgpack_pack_user x, char d)
{
#define CHAR_MIN -1                                                             /* int8_t as function passes char which is signed char */

#if defined(CHAR_MIN)
#if CHAR_MIN < 0
        msgpack_pack_real_int8(x, d);
#else
        msgpack_pack_real_uint8(x, d);
#endif
#else
#error CHAR_MIN is not defined
#endif
}

/* msgpack_pack_inline_func_char(msgpack_pack_user x, signed char d)
{
    msgpack_pack_real_int8(x, d);
}  was already in above */

void msgpack_pack_inline_func_unsigned_char(msgpack_pack_user x, unsigned char d)
{
    msgpack_pack_real_uint8(x, d);
}

#ifdef msgpack_pack_inline_func_cint

void msgpack_pack_inline_func_cint_short(msgpack_pack_user x, short int d)      /* not allowed in misra but here for this function use only */
{
#define SIZEOF_SHORT 2u                                                         /* 16 bit is a short */
#if defined(SIZEOF_SHORT)
#if SIZEOF_SHORT == 2
    msgpack_pack_real_int16(x, d);
#elif SIZEOF_SHORT == 4
    msgpack_pack_real_int32(x, d);
#else
    msgpack_pack_real_int64(x, d);
#endif

#elif defined(SHRT_MAX)
#if SHRT_MAX == 0x7fff
    msgpack_pack_real_int16(x, d);
#elif SHRT_MAX == 0x7fffffff
    msgpack_pack_real_int32(x, d);
#else
    msgpack_pack_real_int64(x, d);
#endif

#else
if(sizeof(short) == 2) {
    msgpack_pack_real_int16(x, d);
} else if(sizeof(short) == 4) {
    msgpack_pack_real_int32(x, d);
} else {
    msgpack_pack_real_int64(x, d);
}
#endif
}

void msgpack_pack_inline_func_cint_int(msgpack_pack_user x, int d)
{
#define SIZEOF_INT 2
#define UINT_MAX ((uint16_t) (~0U))                                             /* 0xFFFF   */
#define INT_MAX ((int16_t)(UINT_MAX>>1u))                                       /* 0x7FFF */

#if defined(SIZEOF_INT)
#if SIZEOF_INT == 2
    msgpack_pack_real_int16(x, d);
#elif SIZEOF_INT == 4
    msgpack_pack_real_int32(x, d);
#else
    msgpack_pack_real_int64(x, d);
#endif

#elif defined(INT_MAX)
#if INT_MAX == 0x7fff
    msgpack_pack_real_int16(x, d);
#elif INT_MAX == 0x7fffffff
    msgpack_pack_real_int32(x, d);
#else
    msgpack_pack_real_int64(x, d);
#endif

#else
if(sizeof(int) == 2) {
    msgpack_pack_real_int16(x, d);
} else if(sizeof(int) == 4) {
    msgpack_pack_real_int32(x, d);
} else {
    msgpack_pack_real_int64(x, d);
}
#endif
}

void msgpack_pack_inline_func_cint_long(msgpack_pack_user x, long int d)        /* again not misra but okay if we need these functions */
{

#define ULONG_MAX ((uint32_t) (~0L))                                            /* 0xFFFFFFFF */
#define LONG_MAX ((int32_t)(ULONG_MAX>>1u))                                     /* 0x7FFFFFFF */

#if defined(SIZEOF_LONG)                                                        /* not defined */
#if SIZEOF_LONG == 2L
    msgpack_pack_real_int16(x, d);
#elif SIZEOF_LONG == 4L
    msgpack_pack_real_int32(x, d);
#else
    msgpack_pack_real_int64(x, d);
#endif

#elif defined(LONG_MAX)
#if LONG_MAX == 0x7fffL
    msgpack_pack_real_int16(x, d);
#elif LONG_MAX == 0x7fffffffL                                                   /* defined */
    msgpack_pack_real_int32(x, d);
#else
    msgpack_pack_real_int64(x, d);
#endif

#else
if(sizeof(long) == 2L) {
    msgpack_pack_real_int16(x, d);
} else if(sizeof(long) == 4L) {
    msgpack_pack_real_int32(x, d);
} else {
    msgpack_pack_real_int64(x, d);
}
#endif
}

void msgpack_pack_inline_func_cint_long_long(msgpack_pack_user x, int64_t d)
{
#define ULLONG_MAX ((uint64_t) (~0LL))                                          /* 0xFFFFFFFFFFFFFFFF */
#define LLONG_MAX ((int64_t)(ULLONG_MAX>>1u))                                   /* 0x7FFFFFFFFFFFFFFF */

#if defined(SIZEOF_LONG_LONG)
#if SIZEOF_LONG_LONG == 2LL
    msgpack_pack_real_int16(x, d);
#elif SIZEOF_LONG_LONG == 4LL
    msgpack_pack_real_int32(x, d);
#else
    msgpack_pack_real_int64(x, d);
#endif

#elif defined(LLONG_MAX)
#if LLONG_MAX == 0x7fffLL
    msgpack_pack_real_int16(x, d);
#elif LLONG_MAX == 0x7fffffffLL
    msgpack_pack_real_int32(x, d);
#else
    msgpack_pack_real_int64(x, d);
#endif

#else
if(sizeof(uint64_t) == 2LL) {
    msgpack_pack_real_int16(x, d);
} else if(sizeof(uint64_t) == 4LL) {
    msgpack_pack_real_int32(x, d);
} else {
    msgpack_pack_real_int64(x, d);
}
#endif
}

void msgpack_pack_inline_func_cint_ushort(msgpack_pack_user x, unsigned short int d) /* not misra only this conversion function if needed in library */
{
#define USHRT_MAX UINT16_MAX

#if defined(SIZEOF_SHORT)
#if SIZEOF_SHORT == 2U
    msgpack_pack_real_uint16(x, d);
#elif SIZEOF_SHORT == 4U
    msgpack_pack_real_uint32(x, d);
#else
    msgpack_pack_real_uint64(x, d);
#endif

#elif defined(USHRT_MAX)
#if USHRT_MAX == 0xffffU
    msgpack_pack_real_uint16(x, d);
#elif USHRT_MAX == 0xffffffffU
    msgpack_pack_real_uint32(x, d);
#else
    msgpack_pack_real_uint64(x, d);
#endif

#else
if(sizeof(uint16_t) == 2u) {
    msgpack_pack_real_uint16(x, d);
} else if(sizeof(uint16_t) == 4u) {
    msgpack_pack_real_uint32(x, d);
} else {
    msgpack_pack_real_uint64(x, d);
}
#endif
}

void msgpack_pack_inline_func_cint_uint16_t(msgpack_pack_user x, uint16_t d)
{
#define UINT_MAX UINT16_MAX

#if defined(SIZEOF_INT)
#if SIZEOF_INT == 2U
    msgpack_pack_real_uint16(x, d);
#elif SIZEOF_INT == 4U
    msgpack_pack_real_uint32(x, d);
#else
    msgpack_pack_real_uint64(x, d);
#endif

#elif defined(UINT_MAX)
#if UINT_MAX == 0xffffU
    msgpack_pack_real_uint16(x, d);
#elif UINT_MAX == 0xffffffffU
    msgpack_pack_real_uint32(x, d);
#else
    msgpack_pack_real_uint64(x, d);
#endif

#else
if(sizeof(unsigned int) == 2U) {
    msgpack_pack_real_uint16(x, d);
} else if(sizeof(unsigned int) == 4U) {
    msgpack_pack_real_uint32(x, d);
} else {
    msgpack_pack_real_uint64(x, d);
}
#endif
}

void msgpack_pack_inline_func_cint_ulong(msgpack_pack_user x, unsigned long d)  /* not misra but here for these functions if enabled with the define */
{
#define ULONG_MAX UINT32_MAX

#if defined(SIZEOF_LONG)
#if SIZEOF_LONG == 2L
    msgpack_pack_real_uint16(x, d);
#elif SIZEOF_LONG == 4L
    msgpack_pack_real_uint32(x, d);
#else
    msgpack_pack_real_uint64(x, d);
#endif

#elif defined(ULONG_MAX)
#if ULONG_MAX == 0xffffL
    msgpack_pack_real_uint16(x, d);
#elif ULONG_MAX == 0xffffffffL
    msgpack_pack_real_uint32(x, d);
#else
    msgpack_pack_real_uint64(x, d);
#endif

#else
if(sizeof(unsigned long) == 2L) {
    msgpack_pack_real_uint16(x, d);
} else if(sizeof(unsigned long) == 4L) {
    msgpack_pack_real_uint32(x, d);
} else {
    msgpack_pack_real_uint64(x, d);
}
#endif
}

void msgpack_pack_inline_func_cint_ulonglong(msgpack_pack_user x, unsigned long long d)  /* not misra but here for these functions if enabled with the define */
{
#define ULLONG_MAX UINT64_MAX

#if defined(SIZEOF_LONG_LONG)
#if SIZEOF_LONG_LONG == 2LL
    msgpack_pack_real_uint16(x, d);
#elif SIZEOF_LONG_LONG == 4LL
    msgpack_pack_real_uint32(x, d);
#else
    msgpack_pack_real_uint64(x, d);
#endif

#elif defined(ULLONG_MAX)
#if ULLONG_MAX == 0xffffLL
    msgpack_pack_real_uint16(x, d);
#elif ULLONG_MAX == 0xffffffffLL
    msgpack_pack_real_uint32(x, d);
#else
    msgpack_pack_real_uint64(x, d);
#endif

#else
if(sizeof(uint64_t) == 2LL) {
    msgpack_pack_real_uint16(x, d);
} else if(sizeof(uint64_t) == 4LL) {
    msgpack_pack_real_uint32(x, d);
} else {
    msgpack_pack_real_uint64(x, d);
}
#endif
}

#undef msgpack_pack_inline_func_cint
#endif                                                                          /* end of msgpack_pack_inline_func_cint branch for use */

/*
 * Float
 */
void msgpack_pack_inline_func_float32(msgpack_pack_user x, float32_t d)
{
    unsigned char buf[5u];
    union { float32_t f; uint32_t i; } mem;                                     /* this union is to convert float to int */
    mem.f = d;
    buf[0u] = 0xcau; _msgpack_store32(&buf[1u], mem.i);
    msgpack_pack_append_buffer(x, buf, 5);
}

#define TARGET_OS_IPHONE                                                        /* as we use the IEEE 64 bit represntation */
void msgpack_pack_inline_func_float64(msgpack_pack_user x, float64_t d)
{
    unsigned char buf[9u];
    union { float64_t f; uint64_t i; } mem;                                     /* this union is to convert double to long   */
    mem.f = d;
    buf[0u] = 0xcbu;
#if defined(TARGET_OS_IPHONE)                                                   /* ok then use the IEEE 64 bit representation */

#elif defined(__arm__) && !(__ARM_EABI__)                                       /* arm-oabi */
    // https://github.com/msgpack/msgpack-perl/pull/1
    mem.i = (mem.i & 0xFFFFFFFFUL) << 32UL | (mem.i >> 32UL);
#endif
    _msgpack_store64(&buf[1u], mem.i);
    msgpack_pack_append_buffer(x, buf, 9);
}
#undef TARGET_OS_IPHONE                                                         /* it was used here for branch */

/*
 * Nil
 */
void msgpack_pack_inline_func_nil(msgpack_pack_user x)
{
    static const unsigned char d = 0xc0u;
    msgpack_pack_append_buffer((void*)x,(void*) d,(int16_t) 1);
}

/*
 * Boolean
 */
void msgpack_pack_inline_func_true(msgpack_pack_user x)
{
    static const unsigned char d = 0xc3u;
    msgpack_pack_append_buffer((void*)x,(void*) d,(int16_t) 1);
}

void msgpack_pack_inline_func_false(msgpack_pack_user x)
{
    static const unsigned char d = 0xc2u;
    msgpack_pack_append_buffer((void*)x,(void*) d,(int16_t) 1);
}

/*
 * Array
 */
void msgpack_pack_inline_func_array(msgpack_pack_user x, size_t n)
{
    unsigned char buf[5u];
    unsigned char d;
    
    if(n < 16u) {
        d = 0x90u | (uint8_t)n;
        msgpack_pack_append_buffer((void*)x,(void*) d,(int16_t) 1);
    } else if(n < 65536u) {
        buf[0u] = 0xdcu; 
        _msgpack_store16(&buf[1u], (uint16_t)n);
        msgpack_pack_append_buffer(x, buf, 3);
    } else {
        buf[0u] = 0xddu;
         _msgpack_store32(&buf[1u], (uint32_t)n);
        msgpack_pack_append_buffer(x, buf, 5);
    }
}

/*
 * Map
 */
void msgpack_pack_inline_func_map(msgpack_pack_user x, size_t n)
{
    unsigned char buf[5u];
    unsigned char d;
    if(n < 16u) {
        d = 0x80u | (uint8_t)n;
        msgpack_pack_append_buffer(x, &TAKE8_8(d), 1);
    } else if(n < 65536u) {
        buf[0u] = 0xdeu; 
        _msgpack_store16(&buf[1u], (uint16_t)n);
        msgpack_pack_append_buffer(x, buf, 3);
    } else {
        buf[0u] = 0xdfu;
        _msgpack_store32(&buf[1u], (uint32_t)n);
        msgpack_pack_append_buffer(x, buf, 5);
    }
}

/*
 * Str
 */
void msgpack_pack_inline_func_str(msgpack_pack_user x, size_t l)
{
    unsigned char buf[5u];
    unsigned char d;
    
    if(l < 32u) {
        d = 0xa0u | (uint8_t)l;
        msgpack_pack_append_buffer(x, &TAKE8_8(d), 1);
    } else if(l < 256u) {
        buf[0u] = 0xd9u; 
        buf[1u] = (uint8_t)l;
        msgpack_pack_append_buffer(x, buf, 2);
    } else if(l < 65536u) {
        buf[0u] = 0xdau; 
        _msgpack_store16(&buf[1u], (uint16_t)l);
        msgpack_pack_append_buffer(x, buf, 3);
    } else {
        buf[0u] = 0xdbu; 
        _msgpack_store32(&buf[1u], (uint32_t)l);
        msgpack_pack_append_buffer(x, buf, 5);
    }
}

void msgpack_pack_inline_func_str_body(msgpack_pack_user x, const void* b, size_t l)
{
    msgpack_pack_append_buffer((void*)x, (void*)b,(int16_t) l);
}

/*
 * Raw (V4)
 */
void msgpack_pack_inline_func_v4raw(msgpack_pack_user x, size_t l)
{
    unsigned char buf[5u];
    unsigned char d;
    
    if(l < 32u) {
        d = 0xa0u | (uint8_t)l;
        msgpack_pack_append_buffer(x, &TAKE8_8(d), 1);
    } else if(l < 65536u) {
        buf[0u] = 0xdau; _msgpack_store16(&buf[1u], (uint16_t)l);
        msgpack_pack_append_buffer(x, buf, 3);
    } else {
        buf[0u] = 0xdbu; _msgpack_store32(&buf[1u], (uint32_t)l);
        msgpack_pack_append_buffer(x, buf, 5);
    }
}

void msgpack_pack_inline_func_v4raw_body(msgpack_pack_user x, const void* b, size_t l)
{
    msgpack_pack_append_buffer((void*)x, (void*)b,(int16_t) l);
}

/*
 * Bin
 */
void msgpack_pack_inline_func_bin(msgpack_pack_user x, size_t l)
{
    unsigned char buf[5u];
    if(l < 256u) {
        buf[0] = 0xc4u; 
        buf[1u] = (uint8_t)l;
        msgpack_pack_append_buffer(x, buf, 2);
    } else if(l < 65536u) {
        buf[0] = 0xc5u; 
        _msgpack_store16(&buf[1u], (uint16_t)l);
        msgpack_pack_append_buffer(x, buf, 3);
    } else {
        buf[0] = 0xc6u; 
        _msgpack_store32(&buf[1u], (uint32_t)l);
        msgpack_pack_append_buffer(x, buf, 5);
    }
}


void msgpack_pack_inline_func_bin_body(msgpack_pack_user x, const void* b, size_t l)
{
    msgpack_pack_append_buffer((void*)x, (void*)b,(int16_t) l);
}

/*
 * Ext
 */
void msgpack_pack_inline_func_ext(msgpack_pack_user x, size_t l, int8_t type)
{
    unsigned char buf[6u];
    
    switch(l) {
    case 1u: {
        buf[0u] = 0xd4u;
        buf[1u] = (unsigned char)type;
        msgpack_pack_append_buffer(x, buf, 2);
    } break;
    case 2u: {
        buf[0u] = 0xd5u;
        buf[1u] = (unsigned char)type;
        msgpack_pack_append_buffer(x, buf, 2);
    } break;
    case 4u: {
        buf[0u] = 0xd6u;
        buf[1u] = (unsigned char)type;
        msgpack_pack_append_buffer(x, buf, 2);
    } break;
    case 8u: {
        buf[0u] = 0xd7u;
        buf[1u] = (unsigned char)type;
        msgpack_pack_append_buffer(x, buf, 2);
    } break;
    case 16u: {
        buf[0u] = 0xd8u;
        buf[1u] = (unsigned char)type;
        msgpack_pack_append_buffer(x, buf, 2);
    } break;
    default:
        if(l < 256u) {
            buf[0u] = 0xc7u;
            buf[1u] = (unsigned char)l;
            buf[2u] = (unsigned char)type;
            msgpack_pack_append_buffer(x, buf, 3);
        } else if(l < 65536u) {
            buf[0u] = 0xc8u;
            _msgpack_store16(&buf[1u], l);
            buf[3u] = (unsigned char)type;
            msgpack_pack_append_buffer(x, buf, 4);
        } else {
            buf[0u] = 0xc9u;
            _msgpack_store32(&buf[1u], l);
            buf[5u] = (unsigned char)type;
            msgpack_pack_append_buffer(x, buf, 6);
        }
        break;
    }
}


void msgpack_pack_inline_func_ext_body(msgpack_pack_user x, const void* b, size_t l)
{
    msgpack_pack_append_buffer((void*)x, (void*)b,(int16_t) l);
}

void msgpack_pack_inline_func_timestamp(msgpack_pack_user x, const msgpack_timestamp* d)
{
    char buf[12u];
    uint32_t data32;
    uint64_t data64;
    
    if ((((int64_t)d->tv_sec) >> 34u) == 0u) 
    {
        data64 = ((uint64_t) d->tv_nsec << 34u) | (uint64_t)d->tv_sec;
        if ((data64 & 0xffffffff00000000LL) == 0LL)   
        {
            /* timestamp 32 */
            data32 = (uint32_t)data64;
            /* msgpack_pack_ext(x, 4, -1); */
            msgpack_pack_inline_func_ext(x, 4, -1);
            _msgpack_store32(buf, data32);
            msgpack_pack_append_buffer(x, buf, 4);
        } else 
        {
            /* timestamp 64 */
            /* msgpack_pack_ext(x, 8u, -1); */
            msgpack_pack_inline_func_ext(x, 8, -1);
            _msgpack_store64(buf, data64);
            msgpack_pack_append_buffer(x, buf, 8);
        }
    } 
    else  
    {
        /* timestamp 96 */
        _msgpack_store32(&buf[0u], d->tv_nsec);
        _msgpack_store64(&buf[4u], d->tv_sec);
        /* msgpack_pack_ext(x, 12, -1); */
        msgpack_pack_inline_func_ext(x, 12, -1);
        msgpack_pack_append_buffer(x, buf, 12);
    }
}
/* #endif */


#undef msgpack_pack_inline_func
#undef msgpack_pack_user
#undef msgpack_pack_append_buffer

#undef TAKE8_8
#undef TAKE8_16
#undef TAKE8_32
#undef TAKE8_64

#undef msgpack_pack_real_uint8
#undef msgpack_pack_real_uint16
#undef msgpack_pack_real_uint32
#undef msgpack_pack_real_uint64
#undef msgpack_pack_real_int8
#undef msgpack_pack_real_int16
#undef msgpack_pack_real_int32
#undef msgpack_pack_real_int64

#ifdef __cplusplus
}
#endif

#endif                                                                          /* end library */