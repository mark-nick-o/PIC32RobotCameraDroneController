/**
 * @file encoders.c
 * @brief Radix64 encoding scheme
 *
 * @section License
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Copyright (C) 2010-2019 Oryx Embedded SARL. All rights reserved.
 * Ported by Copyright (C) 2020 A C P Avaiation 
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
 **/

#define TRACE_LEVEL CRYPTO_TRACE_LEVEL                                          //Switch to the appropriate trace level

//Dependencies
//#include "core/crypto.h"
#include "definitions.h"
#include "encoders.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void radix64Encode(const void *inputV, size_t inputLen, char_t *outputV, size_t *outputLen);   //Radix64 encoding related functions
error_t radix64Decode(const char_t *input, size_t inputLen, void *outputV, size_t *outputLen);
void base64Encode(const void *input, size_t inputLen, char_t *output, size_t *outputLen);     //Base64 encoding related functions
error_t base64Decode(const char_t *input, size_t inputLen, void *output,size_t *outputLen);
void base64urlEncode(const void *input, size_t inputLen, char_t *output, size_t *outputLen);  //Base64url encoding related functions
error_t base64urlDecode(const char_t *input, size_t inputLen, void *output, size_t *outputLen);
error_t asn1ReadTag(const uint8_t *dataV, size_t length, Asn1Tag *tag);                      //ASN.1 related functions      
error_t asn1ReadSequence(const uint8_t *dataV, size_t length, Asn1Tag *tag);
error_t asn1ReadOctetString(const uint8_t *dataV, size_t length, Asn1Tag *tag);
error_t asn1ReadOid(const uint8_t *dataV, size_t length, Asn1Tag *tag);
error_t asn1ReadBoolean(const uint8_t *dataV, size_t length, Asn1Tag *tag, bool_t *value);
error_t asn1ReadInt32(const uint8_t *dataV, size_t length, Asn1Tag *tag, int32_t *value);
error_t mpiGrow(Mpi *r, uint8_t size);
error_t asn1ReadMpi(const uint8_t *dataV, size_t length, Asn1Tag *tag, Mpi *value);
error_t asn1WriteTag(Asn1Tag *tag, bool_t reverse, uint8_t *dataV, size_t *written);
error_t asn1WriteInt32(int32_t value, bool_t reverse, uint8_t *dataV,size_t *written);
uint8_t mpiGetBitLength(const Mpi *a);
uint8_t mpiGetByteLength(const Mpi *a);
error_t mpiExport(const Mpi *a, uint8_t *dataV, uint8_t length, MpiFormat format);
error_t asn1WriteMpi(const Mpi *value, bool_t reverse, uint8_t *dataV, size_t *written);
error_t asn1CheckTag(const Asn1Tag *tag, bool_t constructed, uint_t objClass, uint_t objType);
error_t asn1CheckOid(const Asn1Tag *tag, const uint8_t *oid, size_t length);
error_t asn1DumpObject(const uint8_t *dataV, size_t length, uint_t level);
int_t oidComp(const uint8_t *oid1, size_t oidLen1, const uint8_t *oid2, size_t oidLen2);
void ConvertWiegand(char* CodBinLeitura, char* CartaoWiegand );                 //weigand
uint32_t zigzagEncode(int32_t value);                                           //zigzag
int16_t huffmanEncodeBufStreaming(huffmanState_t *state, const uint8_t *inBuf, int16_t inLen, const huffmanTable_t *huffmanTable);  //huffman
int16_t huffmanEncodeBuf(uint8_t *outBuf, int16_t outBufLen, const uint8_t *inBuf, int16_t inLen, const huffmanTable_t *huffmanTable);
int16_t utf8_encode(int32_t codepoint, char *buffer, size_t *size);             //utf8
size_t utf8_check_first(char byte);
size_t utf8_check_full(const char *buffer, size_t size, int32_t *codepoint);
const char *utf8_iterate(const char *buffer, size_t bufsize, int32_t *codepoint);
int16_t utf8_check_string(const char *string, size_t length);
int16_t encode_utf8(uint8_t  *out, uint32_t in);
int16_t decode_utf8(uint32_t *out, const uint8_t *in);
int16_t decode_utf16(uint32_t *out, const uint16_t *in);                        //utf16
int16_t encode_utf16(uint16_t *out, uint32_t in);
size_t utf16_to_utf32(uint32_t  *out, const uint16_t *in, size_t len);          //utf conversion
size_t utf16_to_utf8(uint8_t *out, const uint16_t *in, size_t len);
size_t utf32_to_utf16(uint16_t *out, const uint32_t *in,  size_t len);
size_t utf32_to_utf8(uint8_t *out, const uint32_t *in, size_t len);
size_t utf8_to_utf16(uint16_t *out, const uint8_t *in, size_t len);
size_t utf8_to_utf32(uint32_t *out, const uint8_t *in, size_t len);
void vigenersCipherEncode( char* cipherText, char* keyword, char* message  );   //vigeners
void vigenersCipherDecode( char* cipherText, char* keyword, char* message  );
void rot13CipherEncode( char* cipherText, char* keyword, char* message  );      //rot13
void rot13CipherDecode( char* cipherText, char* keyword, char* message  );

const huffmanTable_t huffmanTable[HUFFMAN_TABLE_SIZE] = {
//   Len    Code       Char Bitcode
    {  2, 0xC000u }, // 0x00 11
    {  3, 0xA000u }, // 0x01 101
    {  4, 0x9000u }, // 0x02 1001
    {  5, 0x8800u }, // 0x03 10001
    {  5, 0x8000u }, // 0x04 10000
    {  6, 0x7400u }, // 0x05 011101
    {  6, 0x7000u }, // 0x06 011100
    {  6, 0x6C00u }, // 0x07 011011
    {  6, 0x6800u }, // 0x08 011010
    {  7, 0x6200u }, // 0x09 0110001
    {  7, 0x6000u }, // 0x0A 0110000
    {  7, 0x5E00u }, // 0x0B 0101111
    {  7, 0x5C00u }, // 0x0C 0101110
    {  7, 0x5A00u }, // 0x0D 0101101
    {  7, 0x5800u }, // 0x0E 0101100
    {  7, 0x5600u }, // 0x0F 0101011
    {  6, 0x6400u }, // 0x10 011001
    {  7, 0x5400u }, // 0x11 0101010
    {  7, 0x5200u }, // 0x12 0101001
    {  8, 0x5100u }, // 0x13 01010001
    {  8, 0x5000u }, // 0x14 01010000
    {  8, 0x4F00u }, // 0x15 01001111
    {  8, 0x4E00u }, // 0x16 01001110
    {  8, 0x4D00u }, // 0x17 01001101
    {  8, 0x4C00u }, // 0x18 01001100
    {  8, 0x4B00u }, // 0x19 01001011
    {  8, 0x4A00u }, // 0x1A 01001010
    {  8, 0x4900u }, // 0x1B 01001001
    {  8, 0x4800u }, // 0x1C 01001000
    {  8, 0x4700u }, // 0x1D 01000111
    {  8, 0x4600u }, // 0x1E 01000110
    {  8, 0x4500u }, // 0x1F 01000101
    {  8, 0x4400u }, // 0x20 01000100
    {  8, 0x4300u }, // 0x21 01000011
    {  8, 0x4200u }, // 0x22 01000010
    {  8, 0x4100u }, // 0x23 01000001
    {  8, 0x4000u }, // 0x24 01000000
    {  9, 0x3C80u }, // 0x25 001111001
    {  9, 0x3C00u }, // 0x26 001111000
    {  9, 0x3B80u }, // 0x27 001110111
    {  9, 0x3B00u }, // 0x28 001110110
    {  9, 0x3A80u }, // 0x29 001110101
    {  9, 0x3A00u }, // 0x2A 001110100
    {  9, 0x3980u }, // 0x2B 001110011
    {  9, 0x3900u }, // 0x2C 001110010
    {  9, 0x3880u }, // 0x2D 001110001
    {  9, 0x3800u }, // 0x2E 001110000
    {  9, 0x3780u }, // 0x2F 001101111
    {  8, 0x3F00u }, // 0x30 00111111
    {  9, 0x3700u }, // 0x31 001101110
    {  9, 0x3680u }, // 0x32 001101101
    {  9, 0x3600u }, // 0x33 001101100
    {  9, 0x3580u }, // 0x34 001101011
    {  9, 0x3500u }, // 0x35 001101010
    {  9, 0x3480u }, // 0x36 001101001
    {  9, 0x3400u }, // 0x37 001101000
    {  9, 0x3380u }, // 0x38 001100111
    {  9, 0x3300u }, // 0x39 001100110
    {  9, 0x3280u }, // 0x3A 001100101
    {  9, 0x3200u }, // 0x3B 001100100
    {  9, 0x3180u }, // 0x3C 001100011
    {  9, 0x3100u }, // 0x3D 001100010
    {  9, 0x3080u }, // 0x3E 001100001
    {  9, 0x3000u }, // 0x3F 001100000
    {  8, 0x3E00u }, // 0x40 00111110
    {  9, 0x2F80u }, // 0x41 001011111
    {  9, 0x2F00u }, // 0x42 001011110
    {  9, 0x2E80u }, // 0x43 001011101
    {  9, 0x2E00u }, // 0x44 001011100
    {  9, 0x2D80u }, // 0x45 001011011
    {  9, 0x2D00u }, // 0x46 001011010
    {  9, 0x2C80u }, // 0x47 001011001
    {  9, 0x2C00u }, // 0x48 001011000
    {  9, 0x2B80u }, // 0x49 001010111
    { 10, 0x27C0u }, // 0x4A 0010011111
    { 10, 0x2780u }, // 0x4B 0010011110
    {  9, 0x2B00u }, // 0x4C 001010110
    { 10, 0x2740u }, // 0x4D 0010011101
    { 10, 0x2700u }, // 0x4E 0010011100
    {  9, 0x2A80u }, // 0x4F 001010101
    {  5, 0x7800u }, // 0x50 01111
    {  9, 0x2A00u }, // 0x51 001010100
    { 10, 0x26C0u }, // 0x52 0010011011
    { 10, 0x2680u }, // 0x53 0010011010
    { 10, 0x2640u }, // 0x54 0010011001
    { 10, 0x2600u }, // 0x55 0010011000
    { 10, 0x25C0u }, // 0x56 0010010111
    { 10, 0x2580u }, // 0x57 0010010110
    { 10, 0x2540u }, // 0x58 0010010101
    { 10, 0x2500u }, // 0x59 0010010100
    { 10, 0x24C0u }, // 0x5A 0010010011
    { 10, 0x2480u }, // 0x5B 0010010010
    { 10, 0x2440u }, // 0x5C 0010010001
    { 10, 0x2400u }, // 0x5D 0010010000
    { 10, 0x23C0u }, // 0x5E 0010001111
    { 10, 0x2380u }, // 0x5F 0010001110
    { 10, 0x2340u }, // 0x60 0010001101
    { 10, 0x2300u }, // 0x61 0010001100
    { 10, 0x22C0u }, // 0x62 0010001011
    { 10, 0x2280u }, // 0x63 0010001010
    { 10, 0x2240u }, // 0x64 0010001001
    { 10, 0x2200u }, // 0x65 0010001000
    { 10, 0x21C0u }, // 0x66 0010000111
    { 10, 0x2180u }, // 0x67 0010000110
    { 10, 0x2140u }, // 0x68 0010000101
    { 10, 0x2100u }, // 0x69 0010000100
    { 10, 0x20C0u }, // 0x6A 0010000011
    { 10, 0x2080u }, // 0x6B 0010000010
    { 10, 0x2040u }, // 0x6C 0010000001
    { 10, 0x2000u }, // 0x6D 0010000000
    { 10, 0x1FC0u }, // 0x6E 0001111111
    { 10, 0x1F80u }, // 0x6F 0001111110
    { 10, 0x1F40u }, // 0x70 0001111101
    { 10, 0x1F00u }, // 0x71 0001111100
    { 10, 0x1EC0u }, // 0x72 0001111011
    { 10, 0x1E80u }, // 0x73 0001111010
    { 10, 0x1E40u }, // 0x74 0001111001
    { 10, 0x1E00u }, // 0x75 0001111000
    { 10, 0x1DC0u }, // 0x76 0001110111
    { 10, 0x1D80u }, // 0x77 0001110110
    { 10, 0x1D40u }, // 0x78 0001110101
    { 10, 0x1D00u }, // 0x79 0001110100
    { 10, 0x1CC0u }, // 0x7A 0001110011
    { 10, 0x1C80u }, // 0x7B 0001110010
    { 10, 0x1C40u }, // 0x7C 0001110001
    { 10, 0x1C00u }, // 0x7D 0001110000
    { 10, 0x1BC0u }, // 0x7E 0001101111
    { 10, 0x1B80u }, // 0x7F 0001101110
    {  9, 0x2980u }, // 0x80 001010011
    { 10, 0x1B40u }, // 0x81 0001101101
    { 10, 0x1B00u }, // 0x82 0001101100
    { 10, 0x1AC0u }, // 0x83 0001101011
    { 10, 0x1A80u }, // 0x84 0001101010
    { 10, 0x1A40u }, // 0x85 0001101001
    { 10, 0x1A00u }, // 0x86 0001101000
    { 10, 0x19C0u }, // 0x87 0001100111
    { 10, 0x1980u }, // 0x88 0001100110
    { 10, 0x1940u }, // 0x89 0001100101
    { 10, 0x1900u }, // 0x8A 0001100100
    { 10, 0x18C0u }, // 0x8B 0001100011
    { 10, 0x1880u }, // 0x8C 0001100010
    { 10, 0x1840u }, // 0x8D 0001100001
    { 10, 0x1800u }, // 0x8E 0001100000
    { 10, 0x17C0u }, // 0x8F 0001011111
    { 10, 0x1780u }, // 0x90 0001011110
    { 10, 0x1740u }, // 0x91 0001011101
    { 10, 0x1700u }, // 0x92 0001011100
    { 10, 0x16C0u }, // 0x93 0001011011
    { 10, 0x1680u }, // 0x94 0001011010
    { 10, 0x1640u }, // 0x95 0001011001
    { 10, 0x1600u }, // 0x96 0001011000
    { 10, 0x15C0u }, // 0x97 0001010111
    { 10, 0x1580u }, // 0x98 0001010110
    { 10, 0x1540u }, // 0x99 0001010101
    { 10, 0x1500u }, // 0x9A 0001010100
    { 10, 0x14C0u }, // 0x9B 0001010011
    { 10, 0x1480u }, // 0x9C 0001010010
    { 10, 0x1440u }, // 0x9D 0001010001
    { 10, 0x1400u }, // 0x9E 0001010000
    { 10, 0x13C0u }, // 0x9F 0001001111
    { 10, 0x1380u }, // 0xA0 0001001110
    { 10, 0x1340u }, // 0xA1 0001001101
    { 10, 0x1300u }, // 0xA2 0001001100
    { 10, 0x12C0u }, // 0xA3 0001001011
    { 10, 0x1280u }, // 0xA4 0001001010
    { 10, 0x1240u }, // 0xA5 0001001001
    { 10, 0x1200u }, // 0xA6 0001001000
    { 10, 0x11C0u }, // 0xA7 0001000111
    { 10, 0x1180u }, // 0xA8 0001000110
    { 10, 0x1140u }, // 0xA9 0001000101
    { 10, 0x1100u }, // 0xAA 0001000100
    { 10, 0x10C0u }, // 0xAB 0001000011
    { 10, 0x1080u }, // 0xAC 0001000010
    { 10, 0x1040u }, // 0xAD 0001000001
    { 10, 0x1000u }, // 0xAE 0001000000
    { 10, 0x0FC0u }, // 0xAF 0000111111
    { 10, 0x0F80u }, // 0xB0 0000111110
    { 10, 0x0F40u }, // 0xB1 0000111101
    { 10, 0x0F00u }, // 0xB2 0000111100
    { 10, 0x0EC0u }, // 0xB3 0000111011
    { 10, 0x0E80u }, // 0xB4 0000111010
    { 10, 0x0E40u }, // 0xB5 0000111001
    { 10, 0x0E00u }, // 0xB6 0000111000
    { 10, 0x0DC0u }, // 0xB7 0000110111
    { 10, 0x0D80u }, // 0xB8 0000110110
    { 10, 0x0D40u }, // 0xB9 0000110101
    { 10, 0x0D00u }, // 0xBA 0000110100
    { 10, 0x0CC0u }, // 0xBB 0000110011
    { 10, 0x0C80u }, // 0xBC 0000110010
    { 10, 0x0C40u }, // 0xBD 0000110001
    { 10, 0x0C00u }, // 0xBE 0000110000
    { 10, 0x0BC0u }, // 0xBF 0000101111
    { 10, 0x0B80u }, // 0xC0 0000101110
    { 10, 0x0B40u }, // 0xC1 0000101101
    { 10, 0x0B00u }, // 0xC2 0000101100
    { 10, 0x0AC0u }, // 0xC3 0000101011
    { 10, 0x0A80u }, // 0xC4 0000101010
    { 10, 0x0A40u }, // 0xC5 0000101001
    { 10, 0x0A00u }, // 0xC6 0000101000
    { 10, 0x09C0u }, // 0xC7 0000100111
    { 10, 0x0980u }, // 0xC8 0000100110
    { 10, 0x0940u }, // 0xC9 0000100101
    { 10, 0x0900u }, // 0xCA 0000100100
    { 10, 0x08C0u }, // 0xCB 0000100011
    { 10, 0x0880u }, // 0xCC 0000100010
    { 10, 0x0840u }, // 0xCD 0000100001
    { 10, 0x0800u }, // 0xCE 0000100000
    { 10, 0x07C0u }, // 0xCF 0000011111
    { 10, 0x0780u }, // 0xD0 0000011110
    { 10, 0x0740u }, // 0xD1 0000011101
    { 10, 0x0700u }, // 0xD2 0000011100
    { 10, 0x06C0u }, // 0xD3 0000011011
    { 10, 0x0680u }, // 0xD4 0000011010
    { 11, 0x0320u }, // 0xD5 00000011001
    { 10, 0x0640u }, // 0xD6 0000011001
    { 10, 0x0600u }, // 0xD7 0000011000
    { 10, 0x05C0u }, // 0xD8 0000010111
    { 10, 0x0580u }, // 0xD9 0000010110
    { 10, 0x0540u }, // 0xDA 0000010101
    { 10, 0x0500u }, // 0xDB 0000010100
    { 10, 0x04C0u }, // 0xDC 0000010011
    { 11, 0x0300u }, // 0xDD 00000011000
    { 10, 0x0480u }, // 0xDE 0000010010
    { 10, 0x0440u }, // 0xDF 0000010001
    {  9, 0x2900u }, // 0xE0 001010010
    { 10, 0x0400u }, // 0xE1 0000010000
    { 10, 0x03C0u }, // 0xE2 0000001111
    { 11, 0x02E0u }, // 0xE3 00000010111
    { 10, 0x0380u }, // 0xE4 0000001110
    { 11, 0x02C0u }, // 0xE5 00000010110
    { 11, 0x02A0u }, // 0xE6 00000010101
    { 11, 0x0280u }, // 0xE7 00000010100
    { 11, 0x0260u }, // 0xE8 00000010011
    { 11, 0x0240u }, // 0xE9 00000010010
    { 11, 0x0220u }, // 0xEA 00000010001
    { 11, 0x0200u }, // 0xEB 00000010000
    { 11, 0x01E0u }, // 0xEC 00000001111
    { 11, 0x01C0u }, // 0xED 00000001110
    { 11, 0x01A0u }, // 0xEE 00000001101
    { 10, 0x0340u }, // 0xEF 0000001101
    {  8, 0x3D00u }, // 0xF0 00111101
    {  9, 0x2880u }, // 0xF1 001010001
    { 11, 0x0180u }, // 0xF2 00000001100
    { 11, 0x0160u }, // 0xF3 00000001011
    { 11, 0x0140u }, // 0xF4 00000001010
    { 11, 0x0120u }, // 0xF5 00000001001
    { 11, 0x0100u }, // 0xF6 00000001000
    { 11, 0x00E0u }, // 0xF7 00000000111
    { 11, 0x00C0u }, // 0xF8 00000000110
    { 12, 0x0010u }, // 0xF9 000000000001
    { 11, 0x00A0u }, // 0xFA 00000000101
    { 11, 0x0080u }, // 0xFB 00000000100
    { 11, 0x0060u }, // 0xFC 00000000011
    { 11, 0x0040u }, // 0xFD 00000000010
    { 11, 0x0020u }, // 0xFE 00000000001
    {  9, 0x2800u }, // 0xFF 001010000
    { 12, 0x0000u }, // EOF  000000000000
};

#if (RADIX64_SUPPORT == ENABLED)                                                //Check crypto library configuration

//Radix64 encoding table
static const char_t radix64EncTable[64] =
{
   '.', '/', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N',
   'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd',
   'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't',
   'u', 'v', 'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
};

//Radix64 decoding table
static const uint8_t radix64DecTable[128] =
{
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x01,
   0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
   0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A,
   0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
#endif


#if (BASE64_SUPPORT == ENABLED)                                                 //Check crypto library configuration

//Base64 encoding table
static const char_t base64EncTable[64] =
{
   'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
   'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
   'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
   'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/'
};

//Base64 decoding table
static const uint8_t base64DecTable[128] =
{
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3E, 0xFF, 0xFF, 0xFF, 0x3F,
   0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
   0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
   0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
#endif
//Check crypto library configuration
#if (BASE64URL_SUPPORT == ENABLED)
//Base64url encoding table
static const char_t base64urlEncTable[64] =
{
   'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
   'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
   'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
   'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '-', '_'
};

//Base64url decoding table
static const uint8_t base64urlDecTable[128] =
{
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3E, 0xFF, 0xFF,
   0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
   0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F,
   0xFF, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
   0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
#endif
#if (RADIX64_SUPPORT == ENABLED)
/**
 * @brief Radix64 encoding algorithm
 * @param[in] input Input data to encode
 * @param[in] inputLen Length of the data to encode
 * @param[out] output NULL-terminated string encoded with Radix64 algorithm
 * @param[out] outputLen Length of the encoded string (optional parameter)
 **/
void radix64Encode(const void *inputV, size_t inputLen, char_t *outputV, size_t *outputLen)
{
   size_t n;
   uint8_t a;
   uint8_t b;
   uint8_t c;
   uint8_t d;
   const uint8_t *p;

   //Point to the first byte of the input data
   p = (const uint8_t *) inputV;

   //Divide the input stream into blocks of 3 bytes
   n = inputLen / 3;

   //A full encoding quantum is always completed at the end of a quantity
   if(inputLen == (n * 3 + 1))
   {
      //The final quantum of encoding input is exactly 8 bits
      if(inputV != NULL && outputV != NULL)
      {
         //Read input data
         a = (p[n * 3] & 0xFC) >> 2;
         b = (p[n * 3] & 0x03) << 4;

         //The final unit of encoded output will be two characters
         outputV[n * 4] = radix64EncTable[a];
         outputV[n * 4 + 1] = radix64EncTable[b];
         outputV[n * 4 + 2] = '\0';
      }

      //Length of the encoded string (excluding the terminating NULL)
      if(outputLen != NULL)
      {
         *outputLen = n * 4 + 2;
      }
   }
   else if(inputLen == (n * 3 + 2))
   {
      //The final quantum of encoding input is exactly 16 bits
      if(inputV != NULL && outputV != NULL)
      {
         //Read input data
         a = (p[n * 3] & 0xFC) >> 2;
         b = ((p[n * 3] & 0x03) << 4) | ((p[n * 3 + 1] & 0xF0) >> 4);
         c = (p[n * 3 + 1] & 0x0F) << 2;

         //The final unit of encoded output will be three characters followed
         //by one "=" padding character
         outputV[n * 4] = radix64EncTable[a];
         outputV[n * 4 + 1] = radix64EncTable[b];
         outputV[n * 4 + 2] = radix64EncTable[c];
         outputV[n * 4 + 3] = '\0';
      }

      //Length of the encoded string (excluding the terminating NULL)
      if(outputLen != NULL)
      {
         *outputLen = n * 4 + 3;
      }
   }
   else
   {
      //The final quantum of encoding input is an integral multiple of 24 bits
      if(outputV != NULL)
      {
         //The final unit of encoded output will be an integral multiple of 4
         //characters
         outputV[n * 4] = '\0';
      }

      //Length of the encoded string (excluding the terminating NULL)
      if(outputLen != NULL)
      {
         *outputLen = n * 4;
      }
   }

   //If the output parameter is NULL, then the function calculates the
   //length of the resulting Radix64 string without copying any data
   if(inputV != NULL && outputV != NULL)
   {
      //The input data is processed block by block
      while(n-- > 0)
      {
         //Read input data
         a = (p[n * 3] & 0xFC) >> 2;
         b = ((p[n * 3] & 0x03) << 4) | ((p[n * 3 + 1] & 0xF0) >> 4);
         c = ((p[n * 3 + 1] & 0x0F) << 2) | ((p[n * 3 + 2] & 0xC0) >> 6);
         d = p[n * 3 + 2] & 0x3F;

         //Map each 3-byte block to 4 printable characters using the Radix64
         //character set
         outputV[n * 4] = radix64EncTable[a];
         outputV[n * 4 + 1] = radix64EncTable[b];
         outputV[n * 4 + 2] = radix64EncTable[c];
         outputV[n * 4 + 3] = radix64EncTable[d];
      }
   }
}

/**
 * @brief Radix64 decoding algorithm
 * @param[in] input Radix64-encoded string
 * @param[in] inputLen Length of the encoded string
 * @param[out] output Resulting decoded data
 * @param[out] outputLen Length of the decoded data
 * @return Error code
 **/
error_t radix64Decode(const char_t *inputV, size_t inputLen, void *output, size_t *outputLen)
{
   error_t error;
   uint32_t value;
   uint_t c;
   size_t i;
   size_t n;
   uint8_t *p;

   //Check parameters
   if(inputV == NULL && inputLen != 0)
      return ERROR_INVALID_PARAMETER;
   if(outputLen == NULL)
      return ERROR_INVALID_PARAMETER;

   //Check the length of the input string
   if((inputLen % 4) == 1)
      return ERROR_INVALID_LENGTH;

   //Initialize status code
   error = NO_ERROR;

   //Point to the buffer where to write the decoded data
   p = (uint8_t *) output;

   //Initialize variables
   n = 0;
   value = 0;

   //Process the Radix64-encoded string
   for(i = 0; i < inputLen && !error; i++)
   {
      //Get current character
      c = (uint_t) inputV[i];

      //Check the value of the current character
      if(c < 128 && radix64DecTable[c] < 64)
      {
         //Decode the current character
         value = (value << 6) | radix64DecTable[c];

         //Divide the input stream into blocks of 4 characters
         if((i % 4) == 3)
         {
            //Map each 4-character block to 3 bytes
            if(p != NULL)
            {
               p[n] = (value >> 16) & 0xFF;
               p[n + 1] = (value >> 8) & 0xFF;
               p[n + 2] = value & 0xFF;
            }

            //Adjust the length of the decoded data
            n += 3;
            //Decode next block
            value = 0;
         }
      }
      else
      {
         //Implementations must reject the encoded data if it contains
         //characters outside the base alphabet
         error = ERROR_INVALID_CHARACTER;
      }
   }

   //Check status code
   if(!error)
   {
      //All trailing pad characters are omitted in Radix64
      if((inputLen % 4) == 2)
      {
         //The last block contains only 1 byte
         if(p != NULL)
         {
            //Decode the last byte
            p[n] = (value >> 4) & 0xFF;
         }

         //Adjust the length of the decoded data
         n++;
      }
      else if((inputLen % 4) == 3)
      {
         //The last block contains only 2 bytes
         if(p != NULL)
         {
            //Decode the last two bytes
            p[n] = (value >> 10) & 0xFF;
            p[n + 1] = (value >> 2) & 0xFF;
         }

         //Adjust the length of the decoded data
         n += 2;
      }
      else
      {
         //No pad characters in this case
      }
   }

   //Total number of bytes that have been written
   *outputLen = n;

   //Return status code
   return error;
}

#endif
//Check crypto library configuration
#if (BASE64_SUPPORT == ENABLED)
/**
 * @brief Base64 encoding algorithm
 * @param[in] input Input data to encode
 * @param[in] inputLen Length of the data to encode
 * @param[out] output NULL-terminated string encoded with Base64 algorithm
 * @param[out] outputLen Length of the encoded string (optional parameter)
 **/
void base64Encode(const void *input, size_t inputLen, char_t *output, size_t *outputLen)
{
   size_t n;
   uint8_t a;
   uint8_t b;
   uint8_t c;
   uint8_t d;
   const uint8_t *p;

   //Point to the first byte of the input data
   p = (const uint8_t *) input;

   //Divide the input stream into blocks of 3 bytes
   n = inputLen / 3;

   //A full encoding quantum is always completed at the end of a quantity
   if(inputLen == (n * 3 + 1))
   {
      //The final quantum of encoding input is exactly 8 bits
      if(input != NULL && output != NULL)
      {
         //Read input data
         a = (p[n * 3] & 0xFC) >> 2;
         b = (p[n * 3] & 0x03) << 4;

         //The final unit of encoded output will be two characters followed
         //by two "=" padding characters
         output[n * 4] = base64EncTable[a];
         output[n * 4 + 1] = base64EncTable[b];
         output[n * 4 + 2] = '=';
         output[n * 4 + 3] = '=';
         output[n * 4 + 4] = '\0';
      }

      //Length of the encoded string (excluding the terminating NULL)
      if(outputLen != NULL)
      {
         *outputLen = n * 4 + 4;
      }
   }
   else if(inputLen == (n * 3 + 2))
   {
      //The final quantum of encoding input is exactly 16 bits
      if(input != NULL && output != NULL)
      {
         //Read input data
         a = (p[n * 3] & 0xFC) >> 2;
         b = ((p[n * 3] & 0x03) << 4) | ((p[n * 3 + 1] & 0xF0) >> 4);
         c = (p[n * 3 + 1] & 0x0F) << 2;

         //The final unit of encoded output will be three characters followed
         //by one "=" padding character
         output[n * 4] = base64EncTable[a];
         output[n * 4 + 1] = base64EncTable[b];
         output[n * 4 + 2] = base64EncTable[c];
         output[n * 4 + 3] = '=';
         output[n * 4 + 4] = '\0';
      }

      //Length of the encoded string (excluding the terminating NULL)
      if(outputLen != NULL)
      {
         *outputLen = n * 4 + 4;
      }
   }
   else
   {
      //The final quantum of encoding input is an integral multiple of 24 bits
      if(output != NULL)
      {
         //The final unit of encoded output will be an integral multiple of 4
         //characters with no "=" padding
         output[n * 4] = '\0';
      }

      //Length of the encoded string (excluding the terminating NULL)
      if(outputLen != NULL)
      {
         *outputLen = n * 4;
      }
   }

   //If the output parameter is NULL, then the function calculates the
   //length of the resulting Base64 string without copying any data
   if(input != NULL && output != NULL)
   {
      //The input data is processed block by block
      while(n-- > 0)
      {
         //Read input data
         a = (p[n * 3] & 0xFC) >> 2;
         b = ((p[n * 3] & 0x03) << 4) | ((p[n * 3 + 1] & 0xF0) >> 4);
         c = ((p[n * 3 + 1] & 0x0F) << 2) | ((p[n * 3 + 2] & 0xC0) >> 6);
         d = p[n * 3 + 2] & 0x3F;

         //Map each 3-byte block to 4 printable characters using the Base64
         //character set
         output[n * 4] = base64EncTable[a];
         output[n * 4 + 1] = base64EncTable[b];
         output[n * 4 + 2] = base64EncTable[c];
         output[n * 4 + 3] = base64EncTable[d];
      }
   }
}

/**
 * @brief Base64 decoding algorithm
 * @param[in] input Base64-encoded string
 * @param[in] inputLen Length of the encoded string
 * @param[out] output Resulting decoded data
 * @param[out] outputLen Length of the decoded data
 * @return Error code
 **/
error_t base64Decode(const char_t *input, size_t inputLen, void *output, size_t *outputLen)
{
   error_t error;
   uint32_t value;
   uint_t c;
   size_t i;
   size_t j;
   size_t n;
   size_t padLen;
   uint8_t *p;

   //Check parameters
   if(input == NULL && inputLen != 0)
      return ERROR_INVALID_PARAMETER;
   if(outputLen == NULL)
      return ERROR_INVALID_PARAMETER;

   //Initialize status code
   error = NO_ERROR;

   //Point to the buffer where to write the decoded data
   p = (uint8_t *) output;

   //Initialize variables
   j = 0;
   n = 0;
   value = 0;
   padLen = 0;

   //Process the Base64-encoded string
   for(i = 0; i < inputLen && !error; i++)
   {
      //Get current character
      c = (uint_t) input[i];

      //Check the value of the current character
      if(c == '\r' || c == '\n')
      {
         //CR and LF characters should be ignored
      }
      else if(c == '=')
      {
         //Increment the number of pad characters
         padLen++;
      }
      else if(c < 128 && base64DecTable[c] < 64 && padLen == 0)
      {
         //Decode the current character
         value = (value << 6) | base64DecTable[c];

         //Divide the input stream into blocks of 4 characters
         if(++j == 4)
         {
            //Map each 4-character block to 3 bytes
            if(p != NULL)
            {
               p[n] = (value >> 16) & 0xFF;
               p[n + 1] = (value >> 8) & 0xFF;
               p[n + 2] = value & 0xFF;
            }

            //Adjust the length of the decoded data
            n += 3;

            //Decode next block
            j = 0;
            value = 0;
         }
      }
      else
      {
         //Implementations must reject the encoded data if it contains
         //characters outside the base alphabet (refer to RFC 4648,
         //section 3.3)
         error = ERROR_INVALID_CHARACTER;
      }
   }

   //Check status code
   if(!error)
   {
      //Check the number of pad characters
      if(padLen == 0 && j == 0)
      {
         //No pad characters in this case
      }
      else if(padLen == 1 && j == 3)
      {
         //The "=" sequence indicates that the last block contains only 2 bytes
         if(p != NULL)
         {
            //Decode the last two bytes
            p[n] = (value >> 10) & 0xFF;
            p[n + 1] = (value >> 2) & 0xFF;
         }

         //Adjust the length of the decoded data
         n += 2;
      }
      else if(padLen == 2 && j == 2)
      {
         //The "==" sequence indicates that the last block contains only 1 byte
         if(p != NULL)
         {
            //Decode the last byte
            p[n] = (value >> 4) & 0xFF;
         }

         //Adjust the length of the decoded data
         n++;
         //Skip trailing pad characters
         i++;
      }
      else
      {
         //The length of the input string must be a multiple of 4
         error = ERROR_INVALID_LENGTH;
      }
   }

   //Total number of bytes that have been written
   *outputLen = n;

   //Return status code
   return error;
}

#endif
//Check crypto library configuration
#if (BASE64URL_SUPPORT == ENABLED)
/**
 * @brief Base64url encoding algorithm
 * @param[in] input Input data to encode
 * @param[in] inputLen Length of the data to encode
 * @param[out] output NULL-terminated string encoded with Base64url algorithm
 * @param[out] outputLen Length of the encoded string (optional parameter)
 **/
void base64urlEncode(const void *input, size_t inputLen, char_t *output, size_t *outputLen)
{
   size_t n;
   uint8_t a;
   uint8_t b;
   uint8_t c;
   uint8_t d;
   const uint8_t *p;

   //Point to the first byte of the input data
   p = (const uint8_t *) input;

   //Divide the input stream into blocks of 3 bytes
   n = inputLen / 3;

   //A full encoding quantum is always completed at the end of a quantity
   if(inputLen == (n * 3 + 1))
   {
      //The final quantum of encoding input is exactly 8 bits
      if(input != NULL && output != NULL)
      {
         //Read input data
         a = (p[n * 3] & 0xFC) >> 2;
         b = (p[n * 3] & 0x03) << 4;

         //The final unit of encoded output will be two characters
         output[n * 4] = base64urlEncTable[a];
         output[n * 4 + 1] = base64urlEncTable[b];
         output[n * 4 + 2] = '\0';
      }

      //Length of the encoded string (excluding the terminating NULL)
      if(outputLen != NULL)
      {
         *outputLen = n * 4 + 2;
      }
   }
   else if(inputLen == (n * 3 + 2))
   {
      //The final quantum of encoding input is exactly 16 bits
      if(input != NULL && output != NULL)
      {
         //Read input data
         a = (p[n * 3] & 0xFC) >> 2;
         b = ((p[n * 3] & 0x03) << 4) | ((p[n * 3 + 1] & 0xF0) >> 4);
         c = (p[n * 3 + 1] & 0x0F) << 2;

         //The final unit of encoded output will be three characters followed
         //by one "=" padding character
         output[n * 4] = base64urlEncTable[a];
         output[n * 4 + 1] = base64urlEncTable[b];
         output[n * 4 + 2] = base64urlEncTable[c];
         output[n * 4 + 3] = '\0';
      }

      //Length of the encoded string (excluding the terminating NULL)
      if(outputLen != NULL)
      {
         *outputLen = n * 4 + 3;
      }
   }
   else
   {
      //The final quantum of encoding input is an integral multiple of 24 bits
      if(output != NULL)
      {
         //The final unit of encoded output will be an integral multiple of 4
         //characters
         output[n * 4] = '\0';
      }

      //Length of the encoded string (excluding the terminating NULL)
      if(outputLen != NULL)
      {
         *outputLen = n * 4;
      }
   }

   //If the output parameter is NULL, then the function calculates the
   //length of the resulting Base64url string without copying any data
   if(input != NULL && output != NULL)
   {
      //The input data is processed block by block
      while(n-- > 0)
      {
         //Read input data
         a = (p[n * 3] & 0xFC) >> 2;
         b = ((p[n * 3] & 0x03) << 4) | ((p[n * 3 + 1] & 0xF0) >> 4);
         c = ((p[n * 3 + 1] & 0x0F) << 2) | ((p[n * 3 + 2] & 0xC0) >> 6);
         d = p[n * 3 + 2] & 0x3F;

         //Map each 3-byte block to 4 printable characters using the Base64url
         //character set
         output[n * 4] = base64urlEncTable[a];
         output[n * 4 + 1] = base64urlEncTable[b];
         output[n * 4 + 2] = base64urlEncTable[c];
         output[n * 4 + 3] = base64urlEncTable[d];
      }
   }
}

/**
 * @brief Base64url decoding algorithm
 * @param[in] input Base64url-encoded string
 * @param[in] inputLen Length of the encoded string
 * @param[out] output Resulting decoded data
 * @param[out] outputLen Length of the decoded data
 * @return Error code
 **/
error_t base64urlDecode(const char_t *input, size_t inputLen, void *output,
   size_t *outputLen)
{
   error_t error;
   uint32_t value;
   uint_t c;
   size_t i;
   size_t n;
   uint8_t *p;

   //Check parameters
   if(input == NULL && inputLen != 0)
      return ERROR_INVALID_PARAMETER;
   if(outputLen == NULL)
      return ERROR_INVALID_PARAMETER;

   //Check the length of the input string
   if((inputLen % 4) == 1)
      return ERROR_INVALID_LENGTH;

   //Initialize status code
   error = NO_ERROR;

   //Point to the buffer where to write the decoded data
   p = (uint8_t *) output;

   //Initialize variables
   n = 0;
   value = 0;

   //Process the Base64url-encoded string
   for(i = 0; i < inputLen && !error; i++)
   {
      //Get current character
      c = (uint_t) input[i];

      //Check the value of the current character
      if(c < 128 && base64urlDecTable[c] < 64)
      {
         //Decode the current character
         value = (value << 6) | base64urlDecTable[c];

         //Divide the input stream into blocks of 4 characters
         if((i % 4) == 3)
         {
            //Map each 4-character block to 3 bytes
            if(p != NULL)
            {
               p[n] = (value >> 16) & 0xFF;
               p[n + 1] = (value >> 8) & 0xFF;
               p[n + 2] = value & 0xFF;
            }

            //Adjust the length of the decoded data
            n += 3;
            //Decode next block
            value = 0;
         }
      }
      else
      {
         //Implementations must reject the encoded data if it contains
         //characters outside the base alphabet
         error = ERROR_INVALID_CHARACTER;
      }
   }

   //Check status code
   if(!error)
   {
      //All trailing pad characters are omitted in Base64url
      if((inputLen % 4) == 2)
      {
         //The last block contains only 1 byte
         if(p != NULL)
         {
            //Decode the last byte
            p[n] = (value >> 4) & 0xFF;
         }

         //Adjust the length of the decoded data
         n++;
      }
      else if((inputLen % 4) == 3)
      {
         //The last block contains only 2 bytes
         if(p != NULL)
         {
            //Decode the last two bytes
            p[n] = (value >> 10) & 0xFF;
            p[n + 1] = (value >> 2) & 0xFF;
         }

         //Adjust the length of the decoded data
         n += 2;
      }
      else
      {
         //No pad characters in this case
      }
   }

   //Total number of bytes that have been written
   *outputLen = n;

   //Return status code
   return error;
}

#endif
//Check crypto library configuration
#if defined(ASN1_SUPPORT)
/**
 * @brief Read an ASN.1 tag from the input stream
 * @param[in] data Input stream where to read the tag
 * @param[in] length Number of bytes available in the input stream
 * @param[out] tag Structure describing the ASN.1 tag
 * @return Error code
 **/
error_t asn1ReadTag(const uint8_t *dataV, size_t length, Asn1Tag *tag)
{
   uint_t i;
   uint_t n;

   //Make sure the identifier octet is present
   if(length == 0)
      return ERROR_INVALID_TAG;

   //Save the class of the ASN.1 tag
   tag->objClass = dataV[0] & ASN1_CLASS_MASK;
   //Primitive or constructed encoding?
   tag->constructed = (dataV[0] & ASN1_ENCODING_CONSTRUCTED) ? TRUE : FALSE;

   //Check the tag number
   if((dataV[0u] & ASN1_TAG_NUMBER_MASK) < 31u)
   {
      //Tag number is in the range 0 to 30
      tag->objType = dataV[0] & ASN1_TAG_NUMBER_MASK;
      //Point to the tag length field
      i = 1;
   }
   else
   {
      //If the tag number is greater than or equal to 31,
      //the subsequent octets will encode the tag number
      tag->objType = 0;

      //Decode the tag number
      for(i = 1; ; i++)
      {
         //The field cannot exceed 5 bytes
         if(i > (sizeof(tag->objType) + 1))
            return ERROR_INVALID_TAG;
         //Insufficient number of bytes to decode the tag number?
         if(!(length - i))
            return ERROR_INVALID_TAG;

         //Update the tag number with bits 7 to 1
         tag->objType = (tag->objType << 7) | (dataV[i] & 0x7F);

         //Bit 8 shall be set unless it is the last octet
         if(!(dataV[i] & 0x80))
            break;
      }
      //Point to the tag length field
      i++;
   }

   //Insufficient number of bytes to decode the tag length?
   if(!(length - i))
      return ERROR_INVALID_TAG;

   //Short form is used?
   if(dataV[i] < 128)
   {
      //Bits 7 to 1 encode the number of bytes in the contents
      tag->length = dataV[i];
      //Point to the contents of the tag
      i++;
   }
   //Long form is used?
   else if(dataV[i] > 128 && dataV[i] < 255)
   {
      //Bits 7 to 1 encode the number of octets in the length field
      n = dataV[i] & 0x7F;

      //The field cannot exceed 4 bytes
      if(n > sizeof(tag->length))
         return ERROR_INVALID_TAG;
      //Insufficient number of bytes to decode the tag length?
      if((length - i) < n)
         return ERROR_INVALID_TAG;

      //Clear the tag length
      tag->length = 0;

      //Read the subsequent octets
      for(i++; n > 0; n--)
      {
         tag->length = (tag->length << 8) | dataV[i++];
      }
   }
   //Indefinite form is used?
   else
   {
      //Indefinite form is not supported
      return ERROR_INVALID_TAG;
   }

   //Save the pointer to the tag contents
   tag->value = dataV + i;
   //Check the length of tag
   if((length - i) < tag->length)
      return ERROR_INVALID_TAG;

   //Total length occupied by the ASN.1 tag in the input stream
   tag->totalLength = i + tag->length;
   //ASN.1 tag successfully decoded
   return NO_ERROR;
}


/**
 * @brief Read an ASN.1 sequence from the input stream
 * @param[in] data Input stream where to read the tag
 * @param[in] length Number of bytes available in the input stream
 * @param[out] tag Structure describing the ASN.1 tag
 * @return Error code
 **/
error_t asn1ReadSequence(const uint8_t *dataV, size_t length, Asn1Tag *tag)
{
   error_t error;

   //Read ASN.1 tag
   error = asn1ReadTag(dataV, length, tag);

   //Check status code
   if(!error)
   {
      //Enforce encoding, class and type
      error = asn1CheckTag(tag, TRUE, ASN1_CLASS_UNIVERSAL, ASN1_TYPE_SEQUENCE);
   }

   //Return status code
   return error;
}


/**
 * @brief Read an octet string from the input stream
 * @param[in] data Input stream where to read the tag
 * @param[in] length Number of bytes available in the input stream
 * @param[out] tag Structure describing the ASN.1 tag
 * @return Error code
 **/
error_t asn1ReadOctetString(const uint8_t *dataV, size_t length, Asn1Tag *tag)
{
   error_t error;

   //Read ASN.1 tag
   error = asn1ReadTag(dataV, length, tag);

   //Check status code
   if(!error)
   {
      //Enforce encoding, class and type
      error = asn1CheckTag(tag, FALSE, ASN1_CLASS_UNIVERSAL, ASN1_TYPE_OCTET_STRING);
   }

   //Return status code
   return error;
}


/**
 * @brief Read an object identifier from the input stream
 * @param[in] data Input stream where to read the tag
 * @param[in] length Number of bytes available in the input stream
 * @param[out] tag Structure describing the ASN.1 tag
 * @return Error code
 **/
error_t asn1ReadOid(const uint8_t *dataV, size_t length, Asn1Tag *tag)
{
   error_t error;

   //Read ASN.1 tag
   error = asn1ReadTag(dataV, length, tag);

   //Check status code
   if(!error)
   {
      //Enforce encoding, class and type
      error = asn1CheckTag(tag, FALSE, ASN1_CLASS_UNIVERSAL, ASN1_TYPE_OBJECT_IDENTIFIER);
   }

   //Return status code
   return error;
}

/**
 * @brief Read a boolean from the input stream
 * @param[in] data Input stream where to read the tag
 * @param[in] length Number of bytes available in the input stream
 * @param[out] tag Structure describing the ASN.1 tag
 * @param[out] value Boolean value
 * @return Error code
 **/
error_t asn1ReadBoolean(const uint8_t *dataV, size_t length, Asn1Tag *tag, bool_t *value)
{
   error_t error;

   //Read ASN.1 tag
   error = asn1ReadTag(dataV, length, tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Enforce encoding, class and type
   error = asn1CheckTag(tag, FALSE, ASN1_CLASS_UNIVERSAL, ASN1_TYPE_BOOLEAN);
   //Invalid tag?
   if(error)
      return error;

   //Make sure the length of the boolean is valid
   if(tag->length != 1)
      return ERROR_INVALID_LENGTH;

   //Read the value of the boolean
   *value = tag->value[0] ? TRUE : FALSE;

   //ASN.1 tag successfully decoded
   return NO_ERROR;
}


/**
 * @brief Read a 32-bit integer from the input stream
 * @param[in] data Input stream where to read the tag
 * @param[in] length Number of bytes available in the input stream
 * @param[out] tag Structure describing the ASN.1 tag
 * @param[out] value Integer value
 * @return Error code
 **/
error_t asn1ReadInt32(const uint8_t *dataV, size_t length, Asn1Tag *tag, int32_t *value)
{
   error_t error;
   size_t i;

   //Read ASN.1 tag
   error = asn1ReadTag(dataV, length, tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Enforce encoding, class and type
   error = asn1CheckTag(tag, FALSE, ASN1_CLASS_UNIVERSAL, ASN1_TYPE_INTEGER);
   //Invalid tag?
   if(error)
      return error;

   //The contents shall consist of one or more octets
   if(tag->length < 1 || tag->length > 4)
      return ERROR_INVALID_TAG;

   //The contents octets shall be a two's complement binary
   //number equal to the integer value
   *value = (tag->value[0] & 0x80) ? -1 : 0;

   //Process contents octets
   for(i = 0; i < tag->length; i++)
   {
      //Rotate left operation
      *value <<= 8;
      //Reconstruct integer value
      *value |= tag->value[i];
   }

   //ASN.1 tag successfully decoded
   return NO_ERROR;
}
/**
 * @brief Adjust the size of multiple precision integer
 * @param[in,out] r A multiple precision integer whose size is to be increased
 * @param[in] size Desired size in words
 * @return Error code
 **/
error_t mpiGrow(Mpi *r, uint8_t size)
{
   uint16_t *dataV;

   size = max(size, 1);                                                         //Ensure the parameter is valid

   if(r->size >= size)                                                          //Check the current size
      return NO_ERROR;

   dataV = (uint16_t*) Malloc(size * MPI_INT_SIZE);                               //Allocate a memory buffer
   if(dataV == NULL)                                                            //Failed to allocate memory?
      return ERROR_OUT_OF_MEMORY;
   memset((void*)dataV,(void*) 0,(int16_t) size * MPI_INT_SIZE);                //Clear buffer contents

   if(r->size > 0)                                                              //Any data to copy?
   {
      memcpy((void*)dataV,(void*) r->dataV,(int16_t) r->size * MPI_INT_SIZE);    //Copy original data
      //Free(r->data,r->size * MPI_INT_SIZE);                                   Free previously allocated memory
      Free((char*)dataV,sizeof(dataV));                                                // frees memory block from the Heap allocated by Malloc, pointed to by the dataV pointer
   }
   r->size = size;                                                              //Update the size of the multiple precision integer
   r->dataV = dataV;

   return NO_ERROR;                                                             //Successful operation
}
/**
 * @brief Octet string to integer conversion
 *
 * Converts an octet string to a non-negative integer
 *
 * @param[out] r Non-negative integer resulting from the conversion
 * @param[in] data Octet string to be converted
 * @param[in] length Length of the octet string
 * @param[in] format Input format
 * @return Error code
 **/
error_t mpiImport(Mpi *r, const uint8_t *dataV, uint_t length, MpiFormat format)
{
   error_t error=0u;
   uint_t i;

   //Check input format
   if(format == MPI_FORMAT_LITTLE_ENDIAN)
   {
      //Skip trailing zeroes
      while(length > 0u && dataV[length - 1] == 0u)
      {
         length--;
      }

      //Ajust the size of the multiple precision integer
      error = mpiGrow(r, (length + MPI_INT_SIZE - 1u) / MPI_INT_SIZE);
      
      //Check status code
      if(!error)
      {
         //Clear the contents of the multiple precision integer
         memset(r->dataV, 0u, r->size * MPI_INT_SIZE);
         //Set sign
         r->sign = 1u;

         //Import data
         for(i = 0u; i < length; i++, dataV++)
         {
            r->dataV[i / MPI_INT_SIZE] |= *dataV << ((i % MPI_INT_SIZE) * 8u);
         }
      }
   }
   else if(format == MPI_FORMAT_BIG_ENDIAN)
   {
      //Skip leading zeroes
      while(length > 1u && *dataV == 0u)
      {
         dataV++;
         length--;
      }

      //Ajust the size of the multiple precision integer
      // ====================== TO PUT IN ====================================== error = mpiGrow(r, (length + MPI_INT_SIZE - 1u) / MPI_INT_SIZE);

      //Check status code
      if(!error)
      {
         //Clear the contents of the multiple precision integer
         memset(r->dataV, 0u, r->size * MPI_INT_SIZE);
         //Set sign
         r->sign = 1;

         //Start from the least significant byte
         dataV += length - 1u;

         //Import data
         for(i = 0u; i < length; i++, dataV--)
         {
            r->dataV[i / MPI_INT_SIZE] |= *dataV << ((i % MPI_INT_SIZE) * 8u);
         }
      }
   }
   else
   {
      //Report an error
      error = ERROR_INVALID_PARAMETER;
   }

   //Return status code
   return error;
}

/**
 * @brief Read a multiple-precision integer from the input stream
 * @param[in] data Input stream where to read the tag
 * @param[in] length Number of bytes available in the input stream
 * @param[out] tag Structure describing the ASN.1 tag
 * @param[out] value Integer value
 * @return Error code
 **/
error_t asn1ReadMpi(const uint8_t *dataV, size_t length, Asn1Tag *tag, Mpi *value)
{
   error_t error;

   //Read ASN.1 tag
   error = asn1ReadTag(dataV, length, tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Enforce encoding, class and type
   error = asn1CheckTag(tag, FALSE, ASN1_CLASS_UNIVERSAL, ASN1_TYPE_INTEGER);
   //Invalid tag?
   if(error)
      return error;

   //Negative integer?
   if(tag->length > 0 && (tag->value[0] & 0x80) != 0)
      return ERROR_INVALID_SYNTAX;

   //Convert the octet string to a multiple precision integer
   error = mpiImport(value, tag->value, tag->length, MPI_FORMAT_BIG_ENDIAN);

   //Return status code
   return error;
}


/**
 * @brief Write an ASN.1 tag
 * @param[in] tag Structure describing the ASN.1 tag
 * @param[in] reverse Use reverse encoding
 * @param[out] data Output stream where to write the tag (optional parameter)
 * @param[out] written Number of bytes written to the output stream (optional parameter)
 * @return Error code
 **/
error_t asn1WriteTag(Asn1Tag *tag, bool_t reverse, uint8_t *dataV, size_t *written)
{
   size_t i;
   size_t m;
   size_t n;

   //Compute the number of octets that are necessary to encode the tag number
   if(tag->objType < 31)
      m = 0;
   else if(tag->objType < 128)
      m = 1;
   else if(tag->objType < 16384)
      m = 2;
   else if(tag->objType < 2097152)
      m = 3;
   else if(tag->objType < 268435456)
      m = 4;
   else
      m = 5;

   //Compute the number of octets that are necessary to encode the length field
   if(tag->length < 128)
      n = 0;
   else if(tag->length < 256)
      n = 1;
   else if(tag->length < 65536)
      n = 2;
   else if(tag->length < 16777216)
      n = 3;
   else
      n = 4;

   //Valid output stream?
   if(dataV != NULL)
   {
      //Use reverse encoding?
      if(reverse)
      {
         //Any data to copy?
         if(tag->value != NULL && tag->length > 0)
         {
            //Make room for the data
            dataV -= tag->length;
            //Copy data
            memmove((void*)dataV,(void*)tag->value, tag->length);
         }

         //Move backward
         dataV -= m + n + 2;
      }
      else
      {
         //Any data to copy?
         if(tag->value != NULL && tag->length > 0)
         {
            //Copy data
            memmove((void*)dataV + m + n + 2u,(void*)tag->value, tag->length);
         }
      }

      //Save the class of the ASN.1 tag
      dataV[0] = tag->objClass;

      //Primitive or constructed encoding?
      if(tag->constructed)
         dataV[0] |= ASN1_ENCODING_CONSTRUCTED;

      //Encode the tag number
      if(m == 0)
      {
         //Tag number is in the range 0 to 30
         dataV[0] |= tag->objType;
      }
      else
      {
         //The tag number is greater than or equal to 31
         dataV[0] |= ASN1_TAG_NUMBER_MASK;

         //The subsequent octets will encode the tag number
         for(i = 0; i < m; i++)
         {
            //Bits 7 to 1 encode the tag number
            dataV[m - i] = (tag->objType >> (i * 7)) & 0x7F;

            //Bit 8 of each octet shall be set to one unless it is the
            //last octet of the identifier octets
            if(i != 0)
               dataV[m - i] |= 0x80;
         }
      }

      //Encode the length field
      if(n == 0)
      {
         //Use short form encoding
         dataV[1 + m] = tag->length & 0x7F;
      }
      else
      {
         //Bits 7 to 1 encode the number of octets in the length field
         dataV[1 + m] = 0x80 | (n & 0x7F);

         //The subsequent octets will encode the length field
         for(i = 0; i < n; i++)
         {
            dataV[1 + m + n - i] = (tag->length >> (i * 8)) & 0xFF;
         }
      }
   }

   //Total length occupied by the ASN.1 tag
   tag->totalLength = tag->length + m + n + 2;

   //The last parameter is optional
   if(written != NULL)
   {
      //Number of bytes written to the output stream
      *written = m + n + 2;

      //Any data copied?
      if(tag->value != NULL)
         *written += tag->length;
   }

   //Successful processing
   return NO_ERROR;
}


/**
 * @brief Write a 32-bit integer to the output stream
 * @param[in] value Integer value
 * @param[in] reverse Use reverse encoding
 * @param[out] data Output stream where to write the tag (optional parameter)
 * @param[out] written Number of bytes written to the output stream
 * @return Error code
 **/
error_t asn1WriteInt32(int32_t value, bool_t reverse, uint8_t *dataV, size_t *written)
{
   size_t i;
   size_t n;
   uint16_t msb;

   //An integer value is always encoded in the smallest possible number of
   //octets
   for(n = 4; n > 1; n--)
   {
      //Retrieve the upper 9 bits
      msb = (value >> (n * 8 - 9)) & 0x01FF;

      //The upper 9 bits shall not have the same value (all 0 or all 1)
      if(msb != 0x0000 && msb != 0x01FF)
         break;
   }

   //Valid output stream?
   if(dataV != NULL)
   {
      //Use reverse encoding?
      if(reverse)
         dataV -= n + 2;

      //Write tag type
      dataV[0] = ASN1_CLASS_UNIVERSAL | ASN1_TYPE_INTEGER;
      //Write tag length
      dataV[1] = n & 0xFF;

      //Write contents octets
      for(i = 0; i < n; i++)
      {
         dataV[1 + n - i] = (value >> (i * 8)) & 0xFF;
      }
   }

   //Number of bytes written to the output stream
   if(written != NULL)
      *written = n + 2;

   //Successful processing
   return NO_ERROR;
}
/**
 * @brief Get the actual length in bits
 * @param[in] a Pointer to a multiple precision integer
 * @return The actual bit count
 **/
uint8_t mpiGetBitLength(const Mpi *a)
{
   uint8_t n;
   uint32_t m;

   //Check whether the specified multiple precision integer is empty
   if(a->size == 0)
      return 0;

   //Start from the most significant word
   for(n = a->size - 1; n > 0; n--)
   {
      //Loop as long as the current word is zero
      if(a->dataV[n] != 0)
         break;
   }

   //Get the current word
   m = a->dataV[n];
   //Convert the length to a bit count
   n *= MPI_INT_SIZE * 8;

   //Adjust the bit count
   for(; m != 0; m >>= 1)
   {
      n++;
   }

   //Return the actual length in bits
   return n;
}
/**
 * @brief Get the actual length in bytes
 * @param[in] a Pointer to a multiple precision integer
 * @return The actual byte count
 **/
uint8_t mpiGetByteLength(const Mpi *a)
{
   uint8_t n;
   uint32_t m;

   //Check whether the specified multiple precision integer is empty
   if(a->size == 0)
      return 0;

   //Start from the most significant word
   for(n = a->size - 1; n > 0; n--)
   {
      //Loop as long as the current word is zero
      if(a->dataV[n] != 0)
         break;
   }

   //Get the current word
   m = a->dataV[n];
   //Convert the length to a byte count
   n *= MPI_INT_SIZE;

   //Adjust the byte count
   for(; m != 0; m >>= 8)
   {
      n++;
   }

   //Return the actual length in bytes
   return n;
}
/**
 * @brief Integer to octet string conversion
 *
 * Converts an integer to an octet string of a specified length
 *
 * @param[in] a Non-negative integer to be converted
 * @param[out] data Octet string resulting from the conversion
 * @param[in] length Intended length of the resulting octet string
 * @param[in] format Output format
 * @return Error code
 **/
error_t mpiExport(const Mpi *a, uint8_t *dataV, uint8_t length, MpiFormat format)
{
   uint8_t i;
   uint8_t n;
   error_t error;

   //Initialize status code
   error = NO_ERROR;

   //Check input format
   if(format == MPI_FORMAT_LITTLE_ENDIAN)
   {
      //Get the actual length in bytes
      n = mpiGetByteLength(a);

      //Make sure the output buffer is large enough
      if(n <= length)
      {
         //Clear output buffer
         memset((void*)dataV,(void*) 0,(int16_t) length);

         //Export data
         for(i = 0; i < n; i++, dataV++)
         {
            *dataV = a->dataV[i / MPI_INT_SIZE] >> ((i % MPI_INT_SIZE) * 8);
         }
      }
      else
      {
         //Report an error
         error = ERROR_INVALID_LENGTH;
      }
   }
   else if(format == MPI_FORMAT_BIG_ENDIAN)
   {
      //Get the actual length in bytes
      n = mpiGetByteLength(a);

      //Make sure the output buffer is large enough
      if(n <= length)
      {
         //Clear output buffer
         memset((void*)dataV,(void*) 0,(int16_t) length);

         //Point to the least significant word
         dataV += length - 1;

         //Export data
         for(i = 0; i < n; i++, dataV--)
         {
            *dataV = a->dataV[i / MPI_INT_SIZE] >> ((i % MPI_INT_SIZE) * 8);
         }
      }
      else
      {
         //Report an error
         error = ERROR_INVALID_LENGTH;
      }
   }
   else
   {
      //Report an error
      error = ERROR_INVALID_PARAMETER;
   }

   //Return status code
   return error;
}
/**
 * @brief Write a multiple-precision integer from the output stream
 * @param[in] value Integer value
 * @param[in] reverse Use reverse encoding
 * @param[out] data Output stream where to write the tag (optional parameter)
 * @param[out] written Number of bytes written to the output stream
 * @return Error code
 **/
error_t asn1WriteMpi(const Mpi *value, bool_t reverse, uint8_t *dataV, size_t *written)
{
   error_t error;
   size_t n;
   Asn1Tag tag;

   //Retrieve the length of the multiple precision integer
   n = mpiGetBitLength(value);

   //An integer value is always encoded in the smallest possible number of
   //octets
   n = (n / 8) + 1;

   //Valid output stream?
   if(dataV != NULL)
   {
      //Use reverse encoding?
      if(reverse)
         dataV -= n;

      //The value of the multiple precision integer is encoded MSB first
      error = mpiExport(value, dataV, n, MPI_FORMAT_BIG_ENDIAN);
      //Any error to report?
      if(error)
         return error;
   }

   //The integer is encapsulated within an ASN.1 structure
   tag.constructed = FALSE;
   tag.objClass = ASN1_CLASS_UNIVERSAL;
   tag.objType = ASN1_TYPE_INTEGER;
   tag.length = n;
   tag.value = dataV;

   //Compute the length of the corresponding ASN.1 structure
   error = asn1WriteTag(&tag, FALSE, dataV, NULL);
   //Any error to report?
   if(error)
      return error;

   //Number of bytes written to the output stream
   if(written != NULL)
      *written = tag.totalLength;

   //Successful processing
   return NO_ERROR;
}

/**
 * @brief Enforce the type of a specified tag
 * @param[in] tag Pointer to an ASN.1 tag
 * @param[in] constructed Expected encoding (TRUE for constructed, FALSE
 *   for primitive)
 * @param[in] objClass Expected tag class
 * @param[in] objType Expected tag type
 * @return Error code
 **/
error_t asn1CheckTag(const Asn1Tag *tag, bool_t constructed, uint_t objClass, uint_t objType)
{
   //Check encoding
   if(tag->constructed != constructed)
      return ERROR_WRONG_ENCODING;
   //Enforce class
   if(tag->objClass != objClass)
      return ERROR_INVALID_CLASS;
   //Enforce type
   if(tag->objType != objType)
      return ERROR_INVALID_TYPE;

   //The tag matches all the criteria
   return NO_ERROR;
}
/**
 * @brief Compare object identifiers
 * @param[in] oid1 Pointer the first OID
 * @param[in] oidLen1 Length of the first OID, in bytes
 * @param[in] oid2 Pointer the second OID
 * @param[in] oidLen2 Length of the second OID, in bytes
 * @return Comparison result
 * @retval 0 Objects identifiers are equal
 * @retval -1 The first OID lexicographically precedes the second OID
 * @retval 1 The second OID lexicographically precedes the first OID
 **/
int_t oidComp(const uint8_t *oid1, size_t oidLen1, const uint8_t *oid2, size_t oidLen2)
{
   size_t i;

   //Perform lexicographical comparison
   for(i = 0; i < oidLen1 && i < oidLen2; i++)
   {
      //Compare current byte
      if(oid1[i] < oid2[i])
         return -1;
      else if(oid1[i] > oid2[i])
         return 1;
   }

   //Compare length
   if(oidLen1 < oidLen2)
      return -1;
   else if(oidLen1 > oidLen2)
      return 1;

   //Object identifiers are equal
   return 0;
}

/**
 * @brief Check ASN.1 tag against a specified OID
 * @param[in] tag Pointer to an ASN.1 tag
 * @param[in] oid Expected object identifier (OID)
 * @param[in] length Length of the OID
 * @return Error code
 **/
error_t asn1CheckOid(const Asn1Tag *tag, const uint8_t *oid, size_t length)
{
   error_t error;

   //Enforce encoding, class and type
   error = asn1CheckTag(tag, FALSE, ASN1_CLASS_UNIVERSAL, ASN1_TYPE_OBJECT_IDENTIFIER);
   //Any error to report?
   if(error)
      return error;

   //Compare OID against the specified value
   if(oidComp(tag->value, tag->length, oid, length))
     return ERROR_WRONG_IDENTIFIER;

   //The tag matches all the criteria
   return NO_ERROR;
}


/**
 * @brief Display an ASN.1 data object
 * @param[in] data Pointer to the ASN.1 object to dump
 * @param[in] length Length of the ASN.1 object
 * @param[in] level Current level of recursion (this parameter shall be set to 0)
 * @return Error code
 **/

error_t asn1DumpObject(const uint8_t *dataV, size_t length, uint_t level)
{
//Check debugging level
#if (TRACE_LEVEL >= TRACE_LEVEL_DEBUG)
   error_t error;
   uint_t i;
   Asn1Tag tag;

   //ASN.1 universal types
   static const char_t *label[32] =
   {
      "[0]",
      "BOOLEAN",
      "INTEGER",
      "BIT STRING",
      "OCTET STRING",
      "NULL",
      "OBJECT IDENTIFIER",
      "OBJECT DESCRIPTOR",
      "EXTERNAL",
      "REAL",
      "ENUMERATED",
      "[11]",
      "UTF8 STRING",
      "[13]",
      "[14]",
      "[15]",
      "SEQUENCE",
      "SET",
      "NUMERIC STRING",
      "PRINTABLE STRING",
      "TELETEX STRING",
      "VIDEOTEX STRING",
      "IA5 STRING",
      "UTC TIME",
      "GENERALIZED TIME",
      "GRAPHIC STRING",
      "VISIBLE STRING",
      "GENERAL STRING",
      "UNIVERSAL STRING",
      "[29]",
      "BMP STRING",
      "[31]"
   };

   //Prefix used to format the structure
   static const char_t *prefix[10] =
   {
      "",
      "  ",
      "    ",
      "      ",
      "        ",
      "          ",
      "            ",
      "              ",
      "                ",
      "                  "
   };

   //Parse ASN.1 object
   while(length > 0)
   {
      //Decode current ASN.1 tag
      error = asn1ReadTag(dataV, length, &tag);
      //Decoding failed?
      if(error)
         return error;

      //Point to the next field
      dataV += tag.totalLength;
      length -= tag.totalLength;

      //Dump tag number, tag class, and contents length fields
      if(tag.objType < 32 && (tag.objClass & ASN1_CLASS_MASK) == ASN1_CLASS_UNIVERSAL)
         TRACE_DEBUG("%s%s (%" PRIuSIZE " bytes)\r\n", prefix[level], label[tag.objType], tag.length);
      else
         TRACE_DEBUG("%s[%u] (%" PRIuSIZE " bytes)\r\n", prefix[level], tag.objType, tag.length);

      //Constructed type?
      if(tag.constructed)
      {
         //Check whether the maximum level of recursion is reached
         if(level < 8)
         {
            //Recursive decoding of the ASN.1 tag
            error = asn1DumpObject(tag.value, tag.length, level + 1);
            //Decoding failed?
            if(error)
               return error;
         }
         else
         {
            //If the maximum level of recursion is reached, then dump contents
            TRACE_DEBUG_ARRAY(prefix[level + 1], tag.value, tag.length);
         }
      }
      //Primitive type?
      else
      {
         //Check the type of the current tag
         switch(tag.objType)
         {
         //OID?
         case ASN1_TYPE_OBJECT_IDENTIFIER:
            //Append prefix
            TRACE_DEBUG(prefix[level + 1]);
            //Print OID
            TRACE_DEBUG("%s", oidToString(tag.value, tag.length, NULL, 0));
            //Add a line feed
            TRACE_DEBUG("\r\n");
            break;

         //String?
         case ASN1_TYPE_UTF8_STRING:
         case ASN1_TYPE_NUMERIC_STRING:
         case ASN1_TYPE_PRINTABLE_STRING:
         case ASN1_TYPE_TELETEX_STRING:
         case ASN1_TYPE_VIDEOTEX_STRING:
         case ASN1_TYPE_IA5_STRING:
         case ASN1_TYPE_GRAPHIC_STRING:
         case ASN1_TYPE_VISIBLE_STRING:
         case ASN1_TYPE_GENERAL_STRING:
         case ASN1_TYPE_UNIVERSAL_STRING:
         case ASN1_TYPE_BMP_STRING:
            //Append prefix
            TRACE_DEBUG("%s", prefix[level + 1]);

            //Dump the entire string
            for(i = 0; i < tag.length; i++)
            {
               TRACE_DEBUG("%c", tag.value[i]);
            }

            //Add a line feed
            TRACE_DEBUG("\r\n");
            break;

         //UTC time?
         case ASN1_TYPE_UTC_TIME:
            //Check length
            if(tag.length != 13)
               return ERROR_WRONG_ENCODING;
            //The encoding shall terminate with a "Z"
            if(tag.value[tag.length - 1] != 'Z')
               return ERROR_WRONG_ENCODING;

            //Append prefix
            TRACE_DEBUG("%s", prefix[level + 1]);
            //Display date
            TRACE_DEBUG("%c%c/%c%c/%c%c ", tag.value[0], tag.value[1],
               tag.value[2], tag.value[3], tag.value[4], tag.value[5]);
            //Display time
            TRACE_DEBUG("%c%c:%c%c:%c%c", tag.value[6], tag.value[7],
               tag.value[8], tag.value[9], tag.value[10], tag.value[11]);
            //Add a line feed
            TRACE_DEBUG("\r\n");
            break;

         //Generalized time?
         case ASN1_TYPE_GENERALIZED_TIME:
            //Check length
            if(tag.length != 15)
               return ERROR_WRONG_ENCODING;
            //The encoding shall terminate with a "Z"
            if(tag.value[tag.length - 1] != 'Z')
               return ERROR_WRONG_ENCODING;

            //Append prefix
            TRACE_DEBUG("%s", prefix[level + 1]);
            //Display date
            TRACE_DEBUG("%c%c%c%c/%c%c/%c%c ", tag.value[0], tag.value[1], tag.value[2],
               tag.value[3], tag.value[4], tag.value[5], tag.value[6], tag.value[7]);
            //Display time
            TRACE_DEBUG("%c%c:%c%c:%c%c", tag.value[8], tag.value[9],
               tag.value[10], tag.value[11], tag.value[12], tag.value[13]);
            //Add a line feed
            TRACE_DEBUG("\r\n");
            break;

         //Any other type?
         default:
            //Dump the contents of the tag
            TRACE_DEBUG_ARRAY(prefix[level + 1], tag.value, tag.length);
            break;
         }
      }
   }
#endif

   //ASN.1 object successfully decoded
   return NO_ERROR;
}

//
// Args:- CodBinLeitura (input char stream)
//        CartaoWiegand (output char stream)
// Example :-
//            Lcd_Cmd(_LCD_CLEAR) ;
//            Lcd_Out(1, 1, "Codigo Cartao:" ) ;
//            Lcd_Out(2, 1, CartaoWiegand) ;
//
// CodBinLeitura is created like this from on to off state transitions :-
//
//          if (PORTB.RB6 == 0 && disparo == 0){
//             CodBinLeitura[cont] = '0';
//             cont++;
//             delay_ms(1);
//             disparo = 1;
//          }
//          if (PORTB.RB7 == 0 && disparo == 0){
//             CodBinLeitura [cont] = '1';
//             cont ++;
//             delay_ms(1);
//             disparo = 1;
//          }
//          if (PORTB.RB6 == 1 && PORTB.RB7 == 1){
//             disparo = 0;
//          }
//
//  RB6 and RB7 are inputs onto the PIC MCU  where display is attached
//
void ConvertWiegand(char* CodBinLeitura, char* CartaoWiegand )
{
      int32_t FacilityDecimal = 0,
              CartaoDecimal = 0;
      int16_t  i;
      int8_t valLen;
      char WiegandFacility[8u],
           WiegandCartao[16u],
           AuxFacility[12u],
           AuxCartao[12u];

      if (strlen(CodBinLeitura)<=23)                                            // input not long enough
      {
         return;
      }
      
      for (i=0;i<8;i++){
          WiegandFacility[i] = CodBinLeitura[i+1];                              // first 8 bits from CodBinLeitura input to function are copied to Wiegand Facility
      }
      for (i=0;i<16;i++){
          WiegandCartao[i] =  CodBinLeitura[i+9];                               // next 16 bits from CodBinLeitura input to function are copied to Wiegand Cartao
      }
      for (i=0;i<8;i++){                                                        // convert the 8 bit bináry input into decimal and write the number to facility decimal
          if (WiegandFacility[i] == '1')
          {
             FacilityDecimal = FacilityDecimal + ldexp (1,7-i);                 // pow(2.,15.-j);
          }
      }
      for (i=0;i<16;i++){                                                       // convert the 16 bit bináry input into decimal and write the number to cartao decimal
          if (WiegandCartao[i] == '1')
          {
             CartaoDecimal = CartaoDecimal + ldexp (1,15-i);                    // pow(2.,15.-j);
          }
      }
      // LongIntToStrWithZeros(FacilityDecimal, AuxFacility);
      if (FacilityDecimal>=100)                                                 // string to be made 11 long e.g. 00000000255
      {
         valLen = 3;                                                            // 3 chars long
      }
      else if ((FacilityDecimal<100) && (FacilityDecimal>=10))                  // between 100 and 10
      {
         valLen = 2;                                                            // 2 chars long
      }
      else if ((FacilityDecimal<10) && (FacilityDecimal>=1))                    // between 10 and 1
      {
         valLen = 1;                                                            // 1 char long
      }
      else
      {
         valLen = 0;                                                            // no number
      }
      for (i=0;i<12-valLen;i++) {
         AuxFacility[i] = '0';                                                  // pad the zeros
      }
      AuxFacility[i] = '\0';                                                    // terminate the string
      sprintf(AuxFacility,"%s%d",AuxFacility,FacilityDecimal);                  // add the number

                                                                                
      // LongIntToStrWithZeros(CartaoDecimal, AuxCartao);
      if (CartaoDecimal>=10000)                                                 // string to be made 11 long e.g. 00000065535
      {
         valLen = 5;                                                            // 5 chars long
      }
      else if ((CartaoDecimal<10000) && (CartaoDecimal>=1000))                  // between 1000 and 10000
      {
         valLen = 4;                                                            // 4 chars long
      }
      else if ((CartaoDecimal>=100) && (CartaoDecimal<1000))                    // between 100 and 1000
      {
         valLen = 3;                                                            // 3 chars long
      }
      else if ((CartaoDecimal<100) && (CartaoDecimal>=10))                      // between 10 and 100
      {
         valLen = 2;                                                            // two chars long
      }
      else if ((CartaoDecimal<10) && (CartaoDecimal>=1))                        // between 1 and 10
      {
         valLen = 1;                                                            // 1 char long
      }
      else                                                                      // zero
      {
         valLen = 0;                                                            // no number
      }
      for (i=0;i<12-valLen;i++) {
         AuxCartao[i] = '0';                                                    // pad the zeros
      }
      sprintf(AuxCartao,"%s%d",AuxCartao,CartaoDecimal);                        // tab the decimal number to the 00 paded string

                                                                                
      for (i=0;i<6;i++){
          CartaoWiegand[i] = '0';                                               // pad 6 zeros on Cartao Wiegand
      }
      for (i=0;i<3;i++){                                                        // now tag auxfacility to the cartao wiegand string
          CartaoWiegand[i+6] = AuxFacility[i+8];                                // now copy the last 3 chars offset by 8 in AuxFacility into Cartao Wiegand offset by 6
      }
      for (i=0;i<6;i++){                                                        // now tag auxcartao to the cartao wiegand string
          CartaoWiegand[i+9] = AuxCartao[i+6];                                  // now copy the last 6 chars offset by 6 in AuxCatao into Cartao Wiegand offset by 9
      }                                                                         // Cartao wiegand is complete
}

/**
 * ZigZag encoding maps all values of a signed integer into those of an unsigned integer in such
 * a way that numbers of small absolute value correspond to small integers in the result.
 *
 * (Compared to just casting a signed to an unsigned which creates huge resulting numbers for
 * small negative integers).
 */
uint32_t zigzagEncode(int32_t value)
{
    return (uint32_t)((value << 1u) ^ (value >> 31u));
}
/*-----------------------------------------------------------------------------
 *   huffmanEncodeBuf():  huffman encoding of buffer
 *
 *  Parameters: uint8_t *outBuf, int16_t outBufLen, const uint8_t *inBuf,  
 *              int16_t inLen, const huffmanTable_t *huffmanTable
 *
 *  Return:     int16_t  _ERR_PARSE_ERROR_ or _ERR_OK_
 *             
 *----------------------------------------------------------------------------*/
int16_t huffmanEncodeBuf(uint8_t *outBuf, int16_t outBufLen, const uint8_t *inBuf, int16_t inLen, const huffmanTable_t *huffmanTable)
{
    int16_t ret = 0;
    int16_t huffCodeLen;
    uint16_t huffCode;
    uint16_t testBit;
    int16_t ii;
    int16_t jj;
    uint8_t outBit = 0x80u;
    uint8_t *outByte = NULL;

    if (((outBuf == NULL) || (inBuf == NULL)) || (huffmanTable == NULL))
    { /* for misra */ }
    else
    {
       outByte = outBuf;
    
       for (ii = 0; ii < inLen; ++ii)
       {
           huffCodeLen = huffmanTable[*inBuf]->codeLen;
           huffCode = huffmanTable[*inBuf]->hCode;
           ++inBuf;
           testBit = 0x8000u;
           for (jj = 0; jj < huffCodeLen; ++jj)
           {
               if (huffCode & testBit)
               {
                   *outByte |= outBit;
               }
               testBit >>= 1;
               outBit >>= 1;
               if (outBit == 0)
               {
                   outBit = 0x80u;
                   ++outByte;
                   *outByte = 0;
                   ret=++ret % INT16_MAX;
               }
               if (ret >= outBufLen && ii < inLen - 1 && jj < huffCodeLen - 1)
               {
                   ret=-1;
               }
           }
        }
        if ((outBit != 0x80)&&(ret != -1))
        {
           ret=++ret % INT16_MAX;                                               // ensure last character in output buffer is counted
        }
    }
    return ret;
}
/*-----------------------------------------------------------------------------
 *   huffmanEncodeBufStreaming():  huffman encoding
 *
 *  Parameters: huffmanState_t *state, const uint8_t *inBuf, int16_t inLen, 
 *              const huffmanTable_t *huffmanTable
 *
 *  Return:     int16_t  _ERR_PARSE_ERROR_ or _ERR_OK_
 *             
 *----------------------------------------------------------------------------*/
int16_t huffmanEncodeBufStreaming(huffmanState_t *state, const uint8_t *inBuf, int16_t inLen, const huffmanTable_t *huffmanTable)
{
    uint8_t *savedOutBytePtr = state->outByte;
    uint8_t savedOutByte = *savedOutBytePtr;
    uint8_t *pos = NULL;
    int16_t huffCodeLen;
    uint16_t huffCode;
    uint16_t testBit;
    int16_t jj;
    int16_t ret=0;
    uint8_t end = 0u;

    if (((state == NULL) || (inBuf == NULL)) || (huffmanTable == NULL))         /* guard null pointer */
    { /* for misra */ }
    else
    {
       for (pos = (uint8_t *)inBuf, end = (uint8_t )inBuf + (uint8_t )inLen; (uint8_t)pos < end; ++pos)                 /* for the encoder byte stream */
       {
           huffCodeLen = huffmanTable[*pos]->codeLen;                           /* look up the length from the huffman table for that char */
           huffCode = huffmanTable[*pos]->hCode;                                /* look up the code for that char read from the stream */
           testBit = 0x8000u;
           for (jj = 0; jj < huffCodeLen; ++jj)
           {
              if (huffCode & testBit)                                           /* MSB is set */
              {
                 *state->outByte |= state->outBit;                              /* or with the outBit */
              }
              testBit >>= 1;                                                    /* shift to the left */
              state->outBit >>= 1;
              if (state->outBit == 0)                                           /* gone to zero */
              {
                 state->outBit = 0x80;                                          /* reset to 80 */
                 ++state->outByte;
                 *state->outByte = 0;
                 ++state->bytesWritten;
              }
              if (state->bytesWritten >= state->outBufLen && ((uint8_t)pos < end - 1 || jj < huffCodeLen - 1)) // if buffer is filled and we haven't finished compressing
              {
                 *savedOutBytePtr = savedOutByte;                               // restore savedOutByte
                 ret=-1;
              }
           }
        }
    }
    return ret;
}
#endif

/*
   Ported from 
   https://github.com/Bownairo/ctrulib/blob/master/libctru/source/util/utf/decode_utf8.c
*/ 

/*-----------------------------------------------------------------------------
 *  encode_utf16:  utf16 encode
 *
 *  Parameters: uint16_t *out, uint32_t in
 *
 *  Return: int16_t
 *             
 *----------------------------------------------------------------------------*/
int16_t encode_utf16(uint16_t *out, uint32_t in)
{
  if(in < 0x10000)
  {
    if(out != NULL)
      *out++ = in;
    return 1;
  }
  else if(in < 0x110000)
  {
    if(out != NULL)
    {
      *out++ = (in >> 10) + 0xD7C0;
      *out++ = (in & 0x3FF) + 0xDC00;
    }
    return 2;
  }

  return -1;
}

/*-----------------------------------------------------------------------------
 *  decode_utf16:  utf16 decode
 *
 *  Parameters: uint32_t *out, const uint16_t *in
 *
 *  Return: int16_t
 *             
 *----------------------------------------------------------------------------*/
int16_t decode_utf16(uint32_t *out, const uint16_t *in)
{
  uint16_t code1, code2;

  code1 = *in++;
  if(code1 >= 0xD800 && code1 < 0xDC00)
  {
    /* surrogate pair */
    code2 = *in++;
    if(code2 >= 0xDC00 && code2 < 0xE000)
    {
      *out = (code1 << 10) + code2 - 0x35FDC00;
      return 2;
    }

    return -1;
  }

  *out = code1;
  return 1;
}

/*-----------------------------------------------------------------------------
 *  decode_utf8:  utf8 decode
 *
 *  Parameters: uint32_t *out, const uint8_t *in
 *
 *  Return: int16_t
 *             
 *----------------------------------------------------------------------------*/
int16_t decode_utf8(uint32_t *out, const uint8_t *in)
{
  uint8_t code1, code2, code3, code4;

  code1 = *in++;
  if(code1 < 0x80)
  {
    /* 1-byte sequence */
    *out = code1;
    return 1;
  }
  else if(code1 < 0xC2)
  {
    return -1;
  }
  else if(code1 < 0xE0)
  {
    /* 2-byte sequence */
    code2 = *in++;
    if((code2 & 0xC0) != 0x80)
    {
      return -1;
    }

    *out = (code1 << 6) + code2 - 0x3080;
    return 2;
  }
  else if(code1 < 0xF0)
  {
    /* 3-byte sequence */
    code2 = *in++;
    if((code2 & 0xC0) != 0x80)
    {
      return -1;
    }
    if(code1 == 0xE0 && code2 < 0xA0)
    {
      return -1;
    }

    code3 = *in++;
    if((code3 & 0xC0) != 0x80)
    {
      return -1;
    }

    *out = (code1 << 12) + (code2 << 6) + code3 - 0xE2080;
    return 3;
  }
  else if(code1 < 0xF5)
  {
    /* 4-byte sequence */
    code2 = *in++;
    if((code2 & 0xC0) != 0x80)
    {
      return -1;
    }
    if(code1 == 0xF0 && code2 < 0x90)
    {
      return -1;
    }
    if(code1 == 0xF4 && code2 >= 0x90)
    {
      return -1;
    }

    code3 = *in++;
    if((code3 & 0xC0) != 0x80)
    {
      return -1;
    }

    code4 = *in++;
    if((code4 & 0xC0) != 0x80)
    {
      return -1;
    }

    *out = (code1 << 18) + (code2 << 12) + (code3 << 6) + code4 - 0x3C82080;
    return 4;
  }

  return -1;
}
/*
 * Copyright (c) 2009-2016 Petri Lehtinen <petri@digip.org>
 */
/*-----------------------------------------------------------------------------
 *  utf8_encode():  utf8 encoding
 *
 *  Parameters: int32_t codepoint, char *buffer, size_t *size
 *
 *  Return:     int16_t  -1 (error) or 0
 *             
 *----------------------------------------------------------------------------*/
int16_t utf8_encode(int32_t codepoint, char *buffer, size_t *size) 
{
    if (codepoint < 0)
        return -1;
    else if (codepoint < 0x80) {
        buffer[0] = (char)codepoint;
        *size = 1;
    } else if (codepoint < 0x800) {
        buffer[0] = 0xC0 + ((codepoint & 0x7C0) >> 6);
        buffer[1] = 0x80 + ((codepoint & 0x03F));
        *size = 2;
    } else if (codepoint < 0x10000) {
        buffer[0] = 0xE0 + ((codepoint & 0xF000) >> 12);
        buffer[1] = 0x80 + ((codepoint & 0x0FC0) >> 6);
        buffer[2] = 0x80 + ((codepoint & 0x003F));
        *size = 3;
    } else if (codepoint <= 0x10FFFF) {
        buffer[0] = 0xF0 + ((codepoint & 0x1C0000) >> 18);
        buffer[1] = 0x80 + ((codepoint & 0x03F000) >> 12);
        buffer[2] = 0x80 + ((codepoint & 0x000FC0) >> 6);
        buffer[3] = 0x80 + ((codepoint & 0x00003F));
        *size = 4;
    } else
        return -1;

    return 0;
}
/*-----------------------------------------------------------------------------
 *  utf8_check_first():  utf8 encoding
 *
 *  Parameters: char byte
 *
 *  Return: size_t
 *             
 *----------------------------------------------------------------------------*/
size_t utf8_check_first(char byte) 
{
    unsigned char u = (unsigned char)byte;

    if (u < 0x80)
        return 1;

    if (0x80 <= u && u <= 0xBF) {
        /* second, third or fourth byte of a multi-byte
           sequence, i.e. a "continuation byte" */
        return 0;
    } else if (u == 0xC0 || u == 0xC1) {
        /* overlong encoding of an ASCII byte */
        return 0;
    } else if (0xC2 <= u && u <= 0xDF) {
        /* 2-byte sequence */
        return 2;
    }

    else if (0xE0 <= u && u <= 0xEF) {
        /* 3-byte sequence */
        return 3;
    } else if (0xF0 <= u && u <= 0xF4) {
        /* 4-byte sequence */
        return 4;
    } else { /* u >= 0xF5 */
        /* Restricted (start of 4-, 5- or 6-byte sequence) or invalid
           UTF-8 */
        return 0;
    }
}
/*-----------------------------------------------------------------------------
 *  utf8_check_full():  utf8 encoding
 *
 *  Parameters: const char *buffer, size_t size, int32_t *codepoint
 *
 *  Return: size_t
 *             
 *----------------------------------------------------------------------------*/
size_t utf8_check_full(const char *buffer, size_t size, int32_t *codepoint) 
{
    size_t i;
    int32_t value = 0;
    unsigned char u = (unsigned char)buffer[0];

    if (size == 2) {
        value = u & 0x1F;
    } else if (size == 3) {
        value = u & 0xF;
    } else if (size == 4) {
        value = u & 0x7;
    } else
        return 0;

    for (i = 1; i < size; i++) {
        u = (unsigned char)buffer[i];

        if (u < 0x80 || u > 0xBF) {
            /* not a continuation byte */
            return 0;
        }

        value = (value << 6) + (u & 0x3F);
    }

    if (value > 0x10FFFF) {
        /* not in Unicode range */
        return 0;
    }

    else if (0xD800 <= value && value <= 0xDFFF) {
        /* invalid code point (UTF-16 surrogate halves) */
        return 0;
    }

    else if ((size == 2 && value < 0x80) || (size == 3 && value < 0x800) ||
             (size == 4 && value < 0x10000)) {
        /* overlong encoding */
        return 0;
    }

    if (codepoint)
        *codepoint = value;

    return 1;
}
/*-----------------------------------------------------------------------------
 *  utf8_iterate():  utf8 encoding
 *
 *  Parameters: const char *buffer, size_t bufsize, int32_t *codepoint
 *
 *  Return: const char *
 *             
 *----------------------------------------------------------------------------*/
const char *utf8_iterate(const char *buffer, size_t bufsize, int32_t *codepoint)
{
    size_t count;
    int32_t value;

    if (!bufsize)
        return buffer;

    count = utf8_check_first(buffer[0]);
    if (count <= 0)
        return NULL;

    if (count == 1)
        value = (unsigned char)buffer[0];
    else {
        if (count > bufsize || !utf8_check_full(buffer, count, &value))
            return NULL;
    }

    if (codepoint)
        *codepoint = value;

    return buffer + count;
}
/*-----------------------------------------------------------------------------
 *  utf8_check_string():  utf8 encoding
 *
 *  Parameters: const char *string, size_t length
 *
 *  Return: int16_t
 *             
 *----------------------------------------------------------------------------*/
int16_t utf8_check_string(const char *string, size_t length) 
{
    size_t i;

    for (i = 0; i < length; i++) {
        size_t count = utf8_check_first(string[i]);
        if (count == 0)
            return 0;
        else if (count > 1) {
            if (count > length - i)
                return 0;

            if (!utf8_check_full(&string[i], count, NULL))
                return 0;

            i += count - 1;
        }
    }

    return 1;
}
/*-----------------------------------------------------------------------------
 *  encode_utf8:  encode utf8
 *
 *  Parameters: uint8_t  *out, uint32_t in
 *
 *  Return: int16_t
 *             
 *----------------------------------------------------------------------------*/
int16_t encode_utf8(uint8_t  *out, uint32_t in)
{
  if(in < 0x80)
  {
    if(out != NULL)
      *out++ = in;
    return 1;
  }
  else if(in < 0x800)
  {
    if(out != NULL)
    {
      *out++ = (in >> 6) + 0xC0;
      *out++ = (in & 0x3F) + 0x80;
    }
    return 2;
  }
  else if(in < 0x10000)
  {
    if(out != NULL)
    {
      *out++ = (in >> 12) + 0xE0;
      *out++ = ((in >> 6) & 0x3F) + 0x80;
      *out++ = (in & 0x3F) + 0x80;
    }
    return 3;
  }
  else if(in < 0x110000)
  {
    if(out != NULL)
    {
      *out++ = (in >> 18) + 0xF0;
      *out++ = ((in >> 12) & 0x3F) + 0x80;
      *out++ = ((in >> 6) & 0x3F) + 0x80;
      *out++ = (in & 0x3F) + 0x80;
    }
    return 4;
  }

  return -1;
}
/*-----------------------------------------------------------------------------
 *  utf16_to_utf32:  utf16 to utf32
 *
 *  Parameters: uint32_t  *out, const uint16_t *in, size_t len
 *
 *  Return: size_t
 *             
 *----------------------------------------------------------------------------*/
size_t utf16_to_utf32(uint32_t  *out, const uint16_t *in, size_t len)
{
  size_t  rc = 0;
  int16_t  units;
  uint32_t code1;

  do
  {
    units = decode_utf16(&code1, in); 
    if(units == -1)
      return -1;

    if(code1 > 0)
    {
      in += units;

      if(out != NULL)
      {
        if(rc < len)
          *out++ = code1;
      }

      if(SSIZE_MAX - 1 >= rc)
        ++rc;
      else
        return -1;
    }
  } while(code1 > 0);

  return rc;
}

/*-----------------------------------------------------------------------------
 *  utf16_to_utf8:  utf16 to utf8
 *
 *  Parameters: uint32_t  *out, const uint16_t *in, size_t len
 *
 *  Return: size_t
 *             
 *----------------------------------------------------------------------------*/
size_t utf16_to_utf8(uint8_t *out, const uint16_t *in, size_t len)
{
  size_t  rc = 0;
  int16_t  units;
  uint32_t code1;
  uint8_t  encoded[4];

  do
  {
    units = decode_utf16(&code1, in); 
    if(units == -1)
      return -1;

    if(code1 > 0)
    {
      in += units;

      units = encode_utf8(encoded, code1);
      if(units == -1)
        return -1;

      if(out != NULL)
      {
        if(rc + units <= len)
        {
          *out++ = encoded[0];
          if(units > 1)
            *out++ = encoded[1];
          if(units > 2)
            *out++ = encoded[2];
          if(units > 3)
            *out++ = encoded[3];
        }
      }

      if(SSIZE_MAX - units >= rc)
        rc += units;
      else
        return -1;
    }
  } while(code1 > 0);

  return rc;
}

/*-----------------------------------------------------------------------------
 *  utf32_to_utf16:  utf32 to utf16
 *
 *  Parameters: uint32_t  *out, const uint16_t *in, size_t len
 *
 *  Return: size_t
 *             
 *----------------------------------------------------------------------------*/
size_t utf32_to_utf16(uint16_t *out, const uint32_t *in,  size_t len)
{
  size_t  rc = 0;
  int16_t  units;
  uint16_t encoded[2];

  while(*in > 0)
  {
    units = encode_utf16(encoded, *in++);
    if(units == -1)
      return -1;

    if(out != NULL)
    {
      if(rc + units <= len)
      {
        *out++ = encoded[0];
        if(units > 1)
          *out++ = encoded[1];
      }
    }

    if(SSIZE_MAX - units >= rc)
      rc += units;
    else
      return -1;
  }

  return rc;
}

/*-----------------------------------------------------------------------------
 *  utf32_to_utf8:  utf32 to utf8
 *
 *  Parameters: uint32_t  *out, const uint16_t *in, size_t len
 *
 *  Return: size_t
 *             
 *----------------------------------------------------------------------------*/
size_t utf32_to_utf8(uint8_t *out, const uint32_t *in, size_t len)
{
  size_t  rc = 0;
  int16_t  units;
  uint8_t  encoded[4];

  while(*in > 0)
  {
    units = encode_utf8(encoded, *in++);
    if(units == -1)
      return -1;

    if(out != NULL)
    {
      if(rc + units <= len)
      {
        *out++ = encoded[0];
        if(units > 1)
          *out++ = encoded[1];
        if(units > 2)
          *out++ = encoded[2];
        if(units > 3)
          *out++ = encoded[3];
      }
    }

    if(SSIZE_MAX - units >= rc)
      rc += units;
    else
      return -1;
  }

  return rc;
}

/*-----------------------------------------------------------------------------
 *  utf8_to_utf16:  utf8 to utf16
 *
 *  Parameters: uint32_t  *out, const uint16_t *in, size_t len
 *
 *  Return: size_t
 *             
 *----------------------------------------------------------------------------*/
size_t utf8_to_utf16(uint16_t *out, const uint8_t *in, size_t len)
{
  size_t  rc = 0;
  int16_t  units;
  uint32_t code1;
  uint16_t encoded[2];

  do
  {
    units = decode_utf8(&code1, in);
    if(units == -1)
      return -1;

    if(code1 > 0)
    {
      in += units;

      units = encode_utf16(encoded, code1);
      if(units == -1)
        return -1;

      if(out != NULL)
      {
        if(rc + units <= len)
        {
          *out++ = encoded[0];
          if(units > 1)
            *out++ = encoded[1];
        }
      }

      if(SSIZE_MAX - units >= rc)
        rc += units;
      else
        return -1;
    }
  } while(code1 > 0);

  return rc;
}

/*-----------------------------------------------------------------------------
 *  utf8_to_utf32:  utf8 to utf32
 *
 *  Parameters: uint32_t  *out, const uint16_t *in, size_t len
 *
 *  Return: size_t
 *             
 *----------------------------------------------------------------------------*/
size_t utf8_to_utf32(uint32_t *out, const uint8_t *in, size_t len)
{
  size_t  rc = 0;
  int16_t  units;
  uint32_t code1;

  do
  {
    units = decode_utf8(&code1, in);
    if(units == -1)
      return -1;

    if(code1 > 0)
    {
      in += units;

      if(out != NULL)
      {
        if(rc < len)
          *out++ = code1;
      }

      if(SSIZE_MAX - 1 >= rc)
        ++rc;
      else
        return -1;
    }
  } while(code1 > 0);

  return rc;
}
/*
 *  vigeners cipher ref :- https://golangify.com/shifr-vizhenera
 *  Vasile Buldumac vasile.buldumac@ati.utm.md
 *  Ported from Golang to mikroE C ACP Aviation
*/

/*-----------------------------------------------------------------------------
 *  vigenersCipherEncode:  vigeners cipher encode
 *
 *  Parameters: char* cipherText, char* keyword, char* message
 *
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/
void vigenersCipherEncode( char* cipherText, char* keyword, char* message  )
{
   uint8_t c;
   uint16_t keyIndex = 0;
   size_t i;
   uint8_t k;
      
   for (i = 0; i < strlen(cipherText); i++)
   {
      if (c >= 'A' && c <= 'Z')                                                 // A=0, B=1, ... Z=25
      {
           c = cipherText[i] - 'A';
           k = keyword[keyIndex] - 'A';
 
           c = (c-k+26)%26 + 'A';                                               // encrypted letter - key letter
           message[i] = c;
 
           keyIndex++;                                                          // increment keyIndex in range of keyword
           keyIndex %= strlen(keyword);
       }
       else if (c >= 'a' && c <= 'z') 
       {
           c = cipherText[i] - 'a';
           k = keyword[keyIndex] - 'a';
 
           c = (c-k+26)%26 + 'a';                                               // encrypted letter - key letter
           message[i] = c;
 
           keyIndex++;                                                          // increment keyIndex in range of keyword
           keyIndex %= strlen(keyword);		   
       }
   }
}

/*-----------------------------------------------------------------------------
 *  vigenersCipherDecode:  vieners cipher decode
 *
 *  Parameters: char* cipherText, char* keyword, char* message
 *
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/
void vigenersCipherDecode( char* cipherText, char* keyword, char* message  )
{
   uint8_t c;
   uint16_t keyIndex = 0;
   size_t i;
   uint8_t k;
      
   for (i = 0; i < strlen(message); i++) 
   {
      c = message[i];
      if (c >= 'A' && c <= 'Z') 
      {
        c -= 'A';                                                               // A=0, B=1, ... Z=25
        k = keyword[keyIndex] - 'A';
 
        c = (c+k)%26 + 'A';                                                     // encrypted letter - key letter
 
        keyIndex++;                                                             // increment keyIndex in range of keyword
        keyIndex %= strlen(keyword);
      }
      else if (c >= 'a' && c <= 'z') 
      {
        c -= 'a';                                                               // A=0, B=1, ... Z=25
        k = keyword[keyIndex] - 'a';
 
        c = (c+k)%26 + 'a';                                                     // encrypted letter - key letter
 
        keyIndex++;                                                             // increment keyIndex in range of keyword
        keyIndex %= strlen(keyword);      
      }
      cipherText[i] = c;
   }
}

/*-----------------------------------------------------------------------------
 *  rot13CipherEncode:  rot13 cipher encode
 *
 *  Parameters: char* cipherText, char* keyword, char* message
 *
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/
void rot13CipherEncode( char* cipherText, char* keyword, char* message  )
{
   uint8_t c;
   size_t i;
      
   for (i = 0; i < strlen(cipherText); i++)
   {
	c = cipherText[i];
        if (c >= 'a' && c <= 'z') 
        {
           c = c + 13;
           if (c > 'z') 
           {
              c = c - 26;
           }
        } 
        else if (c >= 'A' && c <= 'Z') 
        {
            c = c + 13;
            if (c > 'Z') 
            {
               c = c - 26;
            }
        }
        message[i] = c;
    }
}

/*-----------------------------------------------------------------------------
 *  rot13CipherDecode:  rot13 cipher decode
 *
 *  Parameters: char* cipherText, char* keyword, char* message
 *
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/
void rot13CipherDecode( char* cipherText, char* keyword, char* message  )
{
   uint8_t c;
   size_t i;
      
   for (i = 0; i < strlen(message); i++) 
   {
      c = message[i];
      if (c >= 'a' && c <= 'z') 
      {
        c -= 13;
        if (c < 'a') 
        {
            c += 26;
        }
      }
      else if (c >= 'A' && c <= 'Z') 
      {
        c -= 13;
        if (c < 'A') 
        {
            c += 26;
        }
      }
      cipherText[i] = c;
   }
}
#ifdef __cplusplus
}
#endif