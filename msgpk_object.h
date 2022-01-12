#ifndef MSGPACK_OBJECT_H
#define MSGPACK_OBJECT_H
/*
 * MessagePack for C dynamic typing routine
 *
 * Copyright (C) 2008-2009 FURUHASHI Sadayuki
 * Ported : Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
 *
 */

#include <stdint.h>
#include "definitions.h"
#include "msgpk_pack_template.h"
#ifndef bool
#define bool uint8_t
#endif
#ifdef __cplusplus
extern "C" {
#endif

#ifndef size_t
#define size_t int32_t
#endif

/**
 * @defgroup msgpack_object Dynamically typed object
 * @ingroup msgpack
 * @{
 */
typedef enum {
    MSGPACK_OBJECT_NIL                  = 0x00,
    MSGPACK_OBJECT_BOOLEAN              = 0x01,
    MSGPACK_OBJECT_POSITIVE_INTEGER     = 0x02,
    MSGPACK_OBJECT_NEGATIVE_INTEGER     = 0x03,
    MSGPACK_OBJECT_FLOAT32              = 0x0a,
    MSGPACK_OBJECT_FLOAT64              = 0x04,
    MSGPACK_OBJECT_FLOAT                = 0x04,
#if defined(MSGPACK_USE_LEGACY_NAME_AS_FLOAT)
    MSGPACK_OBJECT_DOUBLE               = MSGPACK_OBJECT_FLOAT,                 /* obsolete */
#endif                                                                          /* MSGPACK_USE_LEGACY_NAME_AS_FLOAT */
    MSGPACK_OBJECT_STR                  = 0x05,
    MSGPACK_OBJECT_ARRAY                = 0x06,
    MSGPACK_OBJECT_MAP                  = 0x07,
    MSGPACK_OBJECT_BIN                  = 0x08,
    MSGPACK_OBJECT_EXT                  = 0x09
} msgpack_object_type;


struct msgpack_object;
struct msgpack_object_kv;

MSGPKPACKED(
typedef struct {
    uint32_t size;
    struct msgpack_object* ptr;
}) msgpack_object_array;

MSGPKPACKED(
typedef struct {
    uint32_t size;
    struct msgpack_object_kv* ptr;
}) msgpack_object_map;

MSGPKPACKED(
typedef struct {
    uint32_t size;
    const char* ptr;
}) msgpack_object_str;

MSGPKPACKED(
typedef struct {
    uint32_t size;
    const char* ptr;
}) msgpack_object_bin;

MSGPKPACKED(
typedef struct {
    int8_t type;
    uint32_t size;
    const char* ptr;
}) msgpack_object_ext;

typedef union {
    bool boolean;
    uint64_t u64;
    int64_t  i64;
#if defined(MSGPACK_USE_LEGACY_NAME_AS_FLOAT)
    float64_t dec;                                                              /* obsolete*/
#endif                                                                          /* MSGPACK_USE_LEGACY_NAME_AS_FLOAT */
    float64_t f64;
    msgpack_object_array array;
    msgpack_object_map map;
    msgpack_object_str str;
    msgpack_object_bin bin;
    msgpack_object_ext ext;
} msgpack_object_union;

MSGPKPACKED(
typedef struct msgpack_object {
    msgpack_object_type type;
    msgpack_object_union via;
}) msgpack_object;

MSGPKPACKED(
typedef struct msgpack_object_kv {
    msgpack_object key;
    msgpack_object val;
}) msgpack_object_kv;

/*   #if !defined(_KERNEL_MODE)
//   MSGPACK_DLLEXPORT
//   void msgpack_object_print(FILE* out, msgpack_object o);
//   #endif */
/* int16_t msgpack_pack_object(msgpack_packer* pk, msgpack_object d);  */

/* MSGPACK_DLLEXPORT */
/* int msgpack_object_print_buffer(char *buffer, size_t buffer_size, msgpack_object o); */

/* MSGPACK_DLLEXPORT */
/* bool msgpack_object_equal(const msgpack_object x, const msgpack_object y);  */

/** @} */
extern int16_t msgpack_pack_object(msgpack_packer* pk, msgpack_object d);
extern int16_t msgpack_pack_object1(msgpack_packer* pk, msgpack_object *d);

#ifdef __cplusplus
}
#endif

#endif /* msgpack/object.h */