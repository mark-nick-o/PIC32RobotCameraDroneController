#ifndef _enCoders_H
#define _enCoders_H
/**
 *
 * @section License
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Copyright (C) 2010-2019 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneCrypto Open.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 1.9.6
 *
 * ported by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
 **/
#include "definitions.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef char_t
#define char_t char
#endif
#ifndef size_t
#define size_t int32_t
#endif
#ifndef error_t
#define error_t int16_t                                                         // define error type used in cyclone
#endif
#ifndef uint_t
#define uint_t uint16_t
#endif
#ifndef bool_t
#define bool_t uint8_t
#endif
#define int_t int16_t
#define NO_ERROR 0u                                                             // error handlers for cyclone
#define ERROR_INVALID_PARAMETER 1u
#define ERROR_INVALID_LENGTH 2u
#define ERROR_INVALID_SYNTAX 3u
#define ERROR_BUFFER_OVERFLOW 4u
#define ERROR_WRONG_STATE 5u
#define ERROR_TIMEOUT 6u
#define ERROR_WOULD_BLOCK 7u
#define ERROR_END_OF_STREAM 8u
#define ERROR_INVALID_VERSION 9u
#define ERROR_INVALID_CHARACTER 10u
#define ERROR_INVALID_TAG 11u
#define ERROR_WRONG_ENCODING 12u
#define ERROR_INVALID_CLASS 13u
#define ERROR_INVALID_TYPE 14u
#define ERROR_WRONG_IDENTIFIER 15u
#define ERROR_NOT_IMPLEMENTED 16u
#define ERROR_OUT_OF_MEMORY 17u

// ------------------ asn1 -----------------------------------------------------

#define ASN1_TAG_NUMBER_MASK        0x1Fu                                       //Tag number mask

#define ASN1_ENCODING_MASK          0x20u                                       //ASN.1 encoding
#define ASN1_ENCODING_PRIMITIVE     0x00u
#define ASN1_ENCODING_CONSTRUCTED   0x20u

#define ASN1_CLASS_MASK             0xC0u                                       //ASN.1 class
#define ASN1_CLASS_UNIVERSAL        0x00u
#define ASN1_CLASS_APPLICATION      0x40u
#define ASN1_CLASS_CONTEXT_SPECIFIC 0x80u
#define ASN1_CLASS_PRIVATE          0xC0u

/**
 * @brief ASN.1 data types
 **/
typedef enum
{
   ASN1_TYPE_BOOLEAN           = 1,
   ASN1_TYPE_INTEGER           = 2,
   ASN1_TYPE_BIT_STRING        = 3,
   ASN1_TYPE_OCTET_STRING      = 4,
   ASN1_TYPE_NULL              = 5,
   ASN1_TYPE_OBJECT_IDENTIFIER = 6,
   ASN1_TYPE_OBJECT_DESCRIPTOR = 7,
   ASN1_TYPE_EXTERNAL          = 8,
   ASN1_TYPE_REAL              = 9,
   ASN1_TYPE_ENUMERATED        = 10,
   ASN1_TYPE_UTF8_STRING       = 12,
   ASN1_TYPE_SEQUENCE          = 16,
   ASN1_TYPE_SET               = 17,
   ASN1_TYPE_NUMERIC_STRING    = 18,
   ASN1_TYPE_PRINTABLE_STRING  = 19,
   ASN1_TYPE_TELETEX_STRING    = 20,
   ASN1_TYPE_VIDEOTEX_STRING   = 21,
   ASN1_TYPE_IA5_STRING        = 22,
   ASN1_TYPE_UTC_TIME          = 23,
   ASN1_TYPE_GENERALIZED_TIME  = 24,
   ASN1_TYPE_GRAPHIC_STRING    = 25,
   ASN1_TYPE_VISIBLE_STRING    = 26,
   ASN1_TYPE_GENERAL_STRING    = 27,
   ASN1_TYPE_UNIVERSAL_STRING  = 28,
   ASN1_TYPE_BMP_STRING        = 30,
} Asn1Type;

/**
 * @brief ASN.1 tag
 **/
typedef struct
{
   bool_t constructed;
   uint_t objClass;
   uint_t objType;
   size_t length;
   const uint8_t *value;
   size_t totalLength;
} Asn1Tag;

/**
 * @brief Arbitrary precision integer
 **/
typedef struct
{
   int_t sign;
   uint_t size;
   uint_t *dataV;
} Mpi;
/**
 * @brief MPI import/export format
 **/
typedef enum
{
   MPI_FORMAT_LITTLE_ENDIAN = 0u,
   MPI_FORMAT_BIG_ENDIAN    = 1u
} MpiFormat;

//Size of the sub data type
#define MPI_INT_SIZE sizeof(uint_t)

//Dependencies
//#include "core/crypto.h"

#define HUFFMAN_TABLE_SIZE 257                                                  // 256 characters plus EOF
typedef struct huffmanTable_s {
    uint8_t  codeLen;
    uint16_t hCode;
} huffmanTable_t;

typedef struct huffmanState_s {
    uint16_t bytesWritten;
    uint8_t *outByte;
    uint16_t outBufLen;
    uint8_t outBit;
} huffmanState_t;

extern const huffmanTable_t huffmanTable[HUFFMAN_TABLE_SIZE];

struct huffmanInfo_s {
    uint16_t uncompressedByteCount;
};

#define SSIZE_MAX INT16_MAX

// ------------------ Radix64 encoding related functions -----------------------
extern void radix64Encode(const void *inputV, size_t inputLen, char_t *outputV, size_t *outputLen);
extern error_t radix64Decode(const char_t *input, size_t inputLen, void *outputV, size_t *outputLen);
// ------------------ Base64 encoding related functions ------------------------
extern void base64Encode(const void *input, size_t inputLen, char_t *output, size_t *outputLen);
extern error_t base64Decode(const char_t *input, size_t inputLen, void *output,size_t *outputLen);
// ------------------ Base64url encoding related functions ---------------------
extern void base64urlEncode(const void *input, size_t inputLen, char_t *output, size_t *outputLen);
extern error_t base64urlDecode(const char_t *input, size_t inputLen, void *output, size_t *outputLen);
// ------------------ ASN.1 related functions ----------------------------------
extern error_t asn1ReadTag(const uint8_t *dataV, size_t length, Asn1Tag *tag);
extern error_t asn1ReadSequence(const uint8_t *dataV, size_t length, Asn1Tag *tag);
extern error_t asn1ReadOctetString(const uint8_t *dataV, size_t length, Asn1Tag *tag);
extern error_t asn1ReadOid(const uint8_t *dataV, size_t length, Asn1Tag *tag);
extern error_t asn1ReadBoolean(const uint8_t *dataV, size_t length, Asn1Tag *tag, bool_t *value);
extern error_t asn1ReadInt32(const uint8_t *dataV, size_t length, Asn1Tag *tag, int32_t *value);
extern error_t mpiGrow(Mpi *r, uint8_t size);
extern error_t asn1ReadMpi(const uint8_t *dataV, size_t length, Asn1Tag *tag, Mpi *value);
extern error_t asn1WriteTag(Asn1Tag *tag, bool_t reverse, uint8_t *dataV, size_t *written);
extern error_t asn1WriteInt32(int32_t value, bool_t reverse, uint8_t *dataV,size_t *written);
extern uint8_t mpiGetBitLength(const Mpi *a);
extern uint8_t mpiGetByteLength(const Mpi *a);
extern error_t mpiExport(const Mpi *a, uint8_t *dataV, uint8_t length, MpiFormat format);
extern error_t asn1WriteMpi(const Mpi *value, bool_t reverse, uint8_t *dataV, size_t *written);
extern error_t asn1CheckTag(const Asn1Tag *tag, bool_t constructed, uint_t objClass, uint_t objType);
extern error_t asn1CheckOid(const Asn1Tag *tag, const uint8_t *oid, size_t length);
extern error_t asn1DumpObject(const uint8_t *dataV, size_t length, uint_t level);
extern int16_t oidComp(const uint8_t *oid1, size_t oidLen1, const uint8_t *oid2, size_t oidLen2);
/* ---------------- weigand display ----------------------------------------- */
extern void ConvertWiegand(char* CodBinLeitura, char* CartaoWiegand );
/* ---------------- zigZag -------------------------------------------------- */
extern uint32_t zigzagEncode(int32_t value);
/* ---------------- huffman ------------------------------------------------- */
extern int16_t huffmanEncodeBuf(uint8_t *outBuf, int16_t outBufLen, const uint8_t *inBuf, int16_t inLen, const huffmanTable_t *huffmanTable);
extern int16_t huffmanEncodeBufStreaming(huffmanState_t *state, const uint8_t *inBuf, int16_t inLen, const huffmanTable_t *huffmanTable);
/* ---------------- vigener cipher ------------------------------------------ */
extern void vigenersCipherEncode( char* cipherText, char* keyword, char* message  );
extern void vigenersCipherDecode( char* cipherText, char* keyword, char* message  );
/* ---------------- rot13 cipher -------------------------------------------- */
extern void rot13CipherEncode( char* cipherText, char* keyword, char* message  );      
extern void rot13CipherDecode( char* cipherText, char* keyword, char* message  );
/* ---------------- utf 8 / 16 ---------------------------------------------- */
extern int16_t utf8_encode(int32_t codepoint, char *buffer, size_t *size);             //utf8
extern size_t utf8_check_first(char byte);
extern size_t utf8_check_full(const char *buffer, size_t size, int32_t *codepoint);
extern const char *utf8_iterate(const char *buffer, size_t bufsize, int32_t *codepoint);
extern int16_t utf8_check_string(const char *string, size_t length);
extern int16_t encode_utf8(uint8_t  *out, uint32_t in);
extern int16_t decode_utf8(uint32_t *out, const uint8_t *in);
extern int16_t decode_utf16(uint32_t *out, const uint16_t *in);
extern int16_t encode_utf16(uint16_t *out, uint32_t in);
extern size_t utf16_to_utf32(uint32_t  *out, const uint16_t *in, size_t len);
extern size_t utf16_to_utf8(uint8_t *out, const uint16_t *in, size_t len);
extern size_t utf32_to_utf16(uint16_t *out, const uint32_t *in,  size_t len);
extern size_t utf32_to_utf8(uint8_t *out, const uint32_t *in, size_t len);
extern size_t utf8_to_utf16(uint16_t *out, const uint8_t *in, size_t len);
extern size_t utf8_to_utf32(uint32_t *out, const uint8_t *in, size_t len);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif