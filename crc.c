/*
 */
/*  @file crc.c  *   * @brief Compact CRC generator for embedded systems, with brute force and table-  * driven algorithm options.
 */
/*  Supports CRC-CCITT, CRC-16, and CRC-32 standards.  *  * @par
 */
/*
 */
/*  Ported for mikroE C Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
 */
/*           CRC-8 CRC-8_DVB ARC Modbus Ascii / RTU added
 */
/*
 */
/*  * COPYRIGHT NOTICE: (c) 2000, 2018 Michael Barr.  This software is placed in the   * public domain and may be used for any purpose.
 */
/*  However, this notice must not  * be changed or removed.  No warranty is expressed or implied by the publication  * or distribution of this source code.  
 */
/*  Ported and added to by ACP Aviation 
 */
#ifndef CRC_C
#define CRC_C

#include "definitions.h"
#include <stdint.h>
#include "crc.h"
 /* Algorithmic parameters based on CRC elections made in crc.h. //
 */
#define BITS_PER_BYTE 8u
#define WIDTH  (BITS_PER_BYTE * sizeof(crc_t))
#define TOPBIT (1 << (WIDTH - 1u))

/* Allocate storage for the byte-wide CRC lookup table. //
 */
#define CRC_TABLE_SIZE 256u
static crc_t g_crc_table[CRC_TABLE_SIZE];

/* for UAVCAN
 */
static uint16_t g_ccit16_table[CRC_TABLE_SIZE];

/* Further algorithmic configuration to support the selected CRC standard.
 */
/*
 */
#if defined(CRC_CCITT)

#define POLYNOMIAL             ((crc_t) 0x1021u)
#define INITIAL_REMAINDER      ((crc_t) 0xFFFFu)
#define FINAL_XOR_VALUE        ((crc_t) 0x0000u)
#define REFLECT_DATA(X)        (X)
#define REFLECT_REMAINDER(X)   (X)

#elif defined(CRC_16)

#define POLYNOMIAL             ((crc_t) 0x8005u)
#define INITIAL_REMAINDER      ((crc_t) 0x0000u)
#define FINAL_XOR_VALUE        ((crc_t) 0x0000u)
#define REFLECT_DATA(X)        ((uint8_t) reflect((X), BITS_PER_BYTE))
#define REFLECT_REMAINDER(X)   ((crc_t) reflect((X), WIDTH))

#elif defined(CRC_32)

#define POLYNOMIAL             ((crc_t) 0x04C11DB7UL)
#define INITIAL_REMAINDER      ((crc_t) 0xFFFFFFFFUL)
#define FINAL_XOR_VALUE        ((crc_t) 0xFFFFFFFFUL)
#define REFLECT_DATA(X)        ((uint8_t) reflect((X), BITS_PER_BYTE))
#define REFLECT_REMAINDER(X)   ((crc_t) reflect((X), WIDTH))

#endif

#define USE_CRC_FAST                                                            /* If defined then use the 32 bit CRC fast computation otherwise use slow
 */

uint32_t reflect(uint32_t dataD, uint8_t n_bits);
void crc_init (void);
crc_t crc_slow (uint8_t const * const p_message, uint8_t n_bytes);
crc_t crc_fast (uint8_t const * const p_message, uint8_t n_bytes);
void compute_hashtable_fast32X();
void compute_hashtable_fast32( unsigned char *addData );
uint8_t crc8(uint8_t const *dataX, int32_t length);
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);
uint8_t crc8_dvb_s2_update(uint8_t crc, const void *dataX, uint32_t length);
uint16_t crc16_arc_calculate(uint16_t length, uint8_t const * const p_message);
uint16_t crc16_arc_fast(uint16_t crc, unsigned char const *buffer, uint16_t len);
uint16_t usMBCRC16( unsigned char * pucFrame, uint16_t usLen );
uint8_t usMBAsciiLRC(unsigned char *auchMsg, uint16_t usLen);
void crc_ccit16_init(void);
uint16_t crc_ccit16_fast(uint8_t const * const p_message, uint8_t n_bytes);
uint16_t RobotisServo_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

uint16_t const crc16arc_table[256u] = {                                          /* Table for CRC 16 ARC FAST
 */
        0x0000u, 0xC0C1U, 0xC181U, 0x0140U, 0xC301U, 0x03C0U, 0x0280U, 0xC241U,
        0xC601u, 0x06C0u, 0x0780u, 0xC741u, 0x0500u, 0xC5C1u, 0xC481u, 0x0440u,
        0xCC01u, 0x0CC0u, 0x0D80u, 0xCD41u, 0x0F00u, 0xCFC1u, 0xCE81u, 0x0E40u,
        0x0A00u, 0xCAC1u, 0xCB81u, 0x0B40u, 0xC901u, 0x09C0u, 0x0880u, 0xC841u,
        0xD801u, 0x18C0u, 0x1980u, 0xD941u, 0x1B00u, 0xDBC1u, 0xDA81u, 0x1A40u,
        0x1E00u, 0xDEC1u, 0xDF81u, 0x1F40u, 0xDD01u, 0x1DC0u, 0x1C80u, 0xDC41u,
        0x1400u, 0xD4C1u, 0xD581u, 0x1540u, 0xD701u, 0x17C0u, 0x1680u, 0xD641u,
        0xD201u, 0x12C0u, 0x1380u, 0xD341u, 0x1100u, 0xD1C1u, 0xD081u, 0x1040u,
        0xF001u, 0x30C0u, 0x3180u, 0xF141u, 0x3300u, 0xF3C1u, 0xF281u, 0x3240u,
        0x3600u, 0xF6C1u, 0xF781u, 0x3740u, 0xF501u, 0x35C0u, 0x3480u, 0xF441u,
        0x3C00u, 0xFCC1u, 0xFD81u, 0x3D40u, 0xFF01u, 0x3FC0u, 0x3E80u, 0xFE41u,
        0xFA01u, 0x3AC0u, 0x3B80u, 0xFB41u, 0x3900u, 0xF9C1u, 0xF881u, 0x3840u,
        0x2800u, 0xE8C1u, 0xE981u, 0x2940u, 0xEB01u, 0x2BC0u, 0x2A80u, 0xEA41u,
        0xEE01u, 0x2EC0u, 0x2F80u, 0xEF41u, 0x2D00u, 0xEDC1u, 0xEC81u, 0x2C40u,
        0xE401u, 0x24C0u, 0x2580u, 0xE541u, 0x2700u, 0xE7C1u, 0xE681u, 0x2640u,
        0x2200u, 0xE2C1u, 0xE381u, 0x2340u, 0xE101u, 0x21C0u, 0x2080u, 0xE041u,
        0xA001u, 0x60C0u, 0x6180u, 0xA141u, 0x6300u, 0xA3C1u, 0xA281u, 0x6240u,
        0x6600u, 0xA6C1u, 0xA781u, 0x6740u, 0xA501u, 0x65C0u, 0x6480u, 0xA441u,
        0x6C00u, 0xACC1u, 0xAD81u, 0x6D40u, 0xAF01u, 0x6FC0u, 0x6E80u, 0xAE41u,
        0xAA01u, 0x6AC0u, 0x6B80u, 0xAB41u, 0x6900u, 0xA9C1u, 0xA881u, 0x6840u,
        0x7800u, 0xB8C1u, 0xB981u, 0x7940u, 0xBB01u, 0x7BC0u, 0x7A80u, 0xBA41u,
        0xBE01u, 0x7EC0u, 0x7F80u, 0xBF41u, 0x7D00u, 0xBDC1u, 0xBC81u, 0x7C40u,
        0xB401u, 0x74C0u, 0x7580u, 0xB541u, 0x7700u, 0xB7C1u, 0xB681u, 0x7640u,
        0x7200u, 0xB2C1u, 0xB381u, 0x7340u, 0xB101u, 0x71C0u, 0x7080u, 0xB041u,
        0x5000u, 0x90C1u, 0x9181u, 0x5140u, 0x9301u, 0x53C0u, 0x5280u, 0x9241u,
        0x9601u, 0x56C0u, 0x5780u, 0x9741u, 0x5500u, 0x95C1u, 0x9481u, 0x5440u,
        0x9C01u, 0x5CC0u, 0x5D80u, 0x9D41u, 0x5F00u, 0x9FC1u, 0x9E81u, 0x5E40u,
        0x5A00u, 0x9AC1u, 0x9B81u, 0x5B40u, 0x9901u, 0x59C0u, 0x5880u, 0x9841u,
        0x8801u, 0x48C0u, 0x4980u, 0x8941u, 0x4B00u, 0x8BC1u, 0x8A81u, 0x4A40u,
        0x4E00u, 0x8EC1u, 0x8F81u, 0x4F40u, 0x8D01u, 0x4DC0u, 0x4C80u, 0x8C41u,
        0x4400u, 0x84C1u, 0x8581u, 0x4540u, 0x8701u, 0x47C0u, 0x4680u, 0x8641u,
        0x8201u, 0x42C0u, 0x4380u, 0x8341u, 0x4100u, 0x81C1u, 0x8081u, 0x4040u
};
static const unsigned char aucCRCHi[] = {                                       /* for modbus RTU fast
 */
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u,
    0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u
};

static const unsigned char aucCRCLo[] = {                                       /* for modbus RTU fast
 */
    0x00u, 0xC0u, 0xC1u, 0x01u, 0xC3u, 0x03u, 0x02u, 0xC2u, 0xC6u, 0x06u, 0x07u, 0xC7u,
    0x05u, 0xC5u, 0xC4u, 0x04u, 0xCCu, 0x0Cu, 0x0Du, 0xCDu, 0x0Fu, 0xCFu, 0xCEu, 0x0Eu,
    0x0Au, 0xCAu, 0xCBu, 0x0Bu, 0xC9u, 0x09u, 0x08u, 0xC8u, 0xD8u, 0x18u, 0x19u, 0xD9u,
    0x1Bu, 0xDBu, 0xDAu, 0x1Au, 0x1Eu, 0xDEu, 0xDFu, 0x1Fu, 0xDDu, 0x1Du, 0x1Cu, 0xDCu,
    0x14u, 0xD4u, 0xD5u, 0x15u, 0xD7u, 0x17u, 0x16u, 0xD6u, 0xD2u, 0x12u, 0x13u, 0xD3u,
    0x11u, 0xD1u, 0xD0u, 0x10u, 0xF0u, 0x30u, 0x31u, 0xF1u, 0x33u, 0xF3u, 0xF2u, 0x32u,
    0x36u, 0xF6u, 0xF7u, 0x37u, 0xF5u, 0x35u, 0x34u, 0xF4u, 0x3Cu, 0xFCu, 0xFDu, 0x3Du,
    0xFFu, 0x3Fu, 0x3Eu, 0xFEu, 0xFAu, 0x3Au, 0x3Bu, 0xFBu, 0x39u, 0xF9u, 0xF8u, 0x38u,
    0x28u, 0xE8u, 0xE9u, 0x29u, 0xEBu, 0x2Bu, 0x2Au, 0xEAu, 0xEEu, 0x2Eu, 0x2Fu, 0xEFu,
    0x2Du, 0xEDu, 0xECu, 0x2Cu, 0xE4u, 0x24u, 0x25u, 0xE5u, 0x27u, 0xE7u, 0xE6u, 0x26u,
    0x22u, 0xE2u, 0xE3u, 0x23u, 0xE1u, 0x21u, 0x20u, 0xE0u, 0xA0u, 0x60u, 0x61u, 0xA1u,
    0x63u, 0xA3u, 0xA2u, 0x62u, 0x66u, 0xA6u, 0xA7u, 0x67u, 0xA5u, 0x65u, 0x64u, 0xA4u,
    0x6Cu, 0xACu, 0xADu, 0x6Du, 0xAFu, 0x6Fu, 0x6Eu, 0xAEu, 0xAAu, 0x6Au, 0x6Bu, 0xABu,
    0x69u, 0xA9u, 0xA8u, 0x68u, 0x78u, 0xB8u, 0xB9u, 0x79u, 0xBBu, 0x7Bu, 0x7Au, 0xBAu,
    0xBEu, 0x7Eu, 0x7Fu, 0xBFu, 0x7Du, 0xBDu, 0xBCu, 0x7Cu, 0xB4u, 0x74u, 0x75u, 0xB5u,
    0x77u, 0xB7u, 0xB6u, 0x76u, 0x72u, 0xB2u, 0xB3u, 0x73u, 0xB1u, 0x71u, 0x70u, 0xB0u,
    0x50u, 0x90u, 0x91u, 0x51u, 0x93u, 0x53u, 0x52u, 0x92u, 0x96u, 0x56u, 0x57u, 0x97u,
    0x55u, 0x95u, 0x94u, 0x54u, 0x9Cu, 0x5Cu, 0x5Du, 0x9Du, 0x5Fu, 0x9Fu, 0x9Eu, 0x5Eu,
    0x5Au, 0x9Au, 0x9Bu, 0x5Bu, 0x99u, 0x59u, 0x58u, 0x98u, 0x88u, 0x48u, 0x49u, 0x89u,
    0x4Bu, 0x8Bu, 0x8Au, 0x4Au, 0x4Eu, 0x8Eu, 0x8Fu, 0x4Fu, 0x8Du, 0x4Du, 0x4Cu, 0x8Cu,
    0x44u, 0x84u, 0x85u, 0x45u, 0x87u, 0x47u, 0x46u, 0x86u, 0x82u, 0x42u, 0x43u, 0x83u,
    0x41u, 0x81u, 0x80u, 0x40u
};

/*-----------------------------------------------------------------------------
 *      reflect:  Reflect calculation
 *
 *  Parameters: uint32_t dataD, uint8_t n_bits
 *
 *  Return:     uint32_t
 *----------------------------------------------------------------------------*/
uint32_t reflect(uint32_t dataD, uint8_t n_bits)
{
   uint32_t reflection = 0x00000000UL;
   uint8_t bitcount;

   /* NOTE: For efficiency sake, n_bits is not verified to be <= 32. */

   /* Reflect the data about the center bit. */
   for (bitcount = 0u; (bitcount < n_bits); bitcount++)
   {

       if (dataD & 0x01u)
       {
           reflection |= (1u << ((n_bits - 1u) - bitcount));                    /* If the LSB bit is set, set the reflection of it. */
       }

       dataD = (dataD >> 1u);
   }

  return (reflection);

}  /* reflect() */

/*-----------------------------------------------------------------------------
 *      crc_init:  Initialize the table for the chosen CRC when using fast method
 *                 (noramlly you do it at boot-up)
 *
 *  Parameters: void
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void crc_init(void)
{
   crc_t dividend;
   crc_t remainder;
   int bitcounter;
   
   for (dividend = 0; dividend < CRC_TABLE_SIZE; dividend++)                    /* Compute the remainder of each possible dividend.
 */
   {
       remainder = dividend << (WIDTH - BITS_PER_BYTE);                         /* Start with the dividend followed by zeros.
 */

       for (bitcounter = BITS_PER_BYTE; (bitcounter > 0); bitcounter--)         /* Perform modulo-2 division, a bit at a time.
 */
       {
           if (remainder & TOPBIT)                                              /* Try to divide the current data bit.
 */
           {
               remainder = (remainder << 1) ^ POLYNOMIAL;
           }
           else
           {
               remainder = (remainder << 1);
           }
   }
   
   g_crc_table[dividend] = remainder;                                            /* Store the result into the table.
 */
  }
}  /* crc_init() */

/*-----------------------------------------------------------------------------
 *      crc_slow:  Compute the chosen CRC from the define slow (math calc)
 *
 *  185 instructions per byte of message data
 *
 *  Parameters: uint8_t const * const p_message, uint8_t n_bytes
 *
 *  Return:     crc_t - as chosen by the type define
 *----------------------------------------------------------------------------*/
crc_t crc_slow(uint8_t const * const p_message, uint8_t n_bytes)
{
   crc_t remainder = INITIAL_REMAINDER;
   uint8_t byte;
   uint8_t bitcount;

   if (p_message != NULL)
   {
      for (byte = 0u; (byte < n_bytes); byte++)                                 /* Perform modulo-2 division, one byte at a time. */
      {
        remainder ^= (REFLECT_DATA(p_message[byte]) << (WIDTH - BITS_PER_BYTE)); /* Bring the next byte into the remainder. */
        for (bitcount = BITS_PER_BYTE; bitcount > 0u; bitcount--)               /* Perform modulo-2 division, one bit at a time. */
        {
           if (remainder & TOPBIT)                                              /* Try to divide the current data bit. */
           {
               remainder = (remainder << 1u) ^ POLYNOMIAL;
           }
           else
           {
               remainder = (remainder << 1u);
           }
        }
     }
   }
   
   return (REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);                     /* The final remainder is the CRC result.
 */

}  /* crc_slow() */

/*-----------------------------------------------------------------------------
 *      crc_fast:  Compute the chosen CRC from the define fast (lookup method)
 *
 *  36 instructions per byte of message data (needs to call crc_init(); at boot
 *
 *  Parameters: uint8_t const * const p_message, uint8_t n_bytes
 *
 *  Return:     crc_t - as chosen by the type define
 *----------------------------------------------------------------------------*/
crc_t crc_fast(uint8_t const * const p_message, uint8_t n_bytes)
{
   crc_t remainder = INITIAL_REMAINDER;
   uint8_t byte;
   uint8_t dataZ;
   
   if (p_message != NULL)
   {
      for (byte = 0u;(byte < n_bytes); byte++)                                  /* Divide the message by the polynomial, a byte at a time. */
      {
         dataZ = (REFLECT_DATA(p_message[byte])) ^ (remainder >> (WIDTH - BITS_PER_BYTE));
         remainder = g_crc_table[dataZ] ^ (remainder << BITS_PER_BYTE);
      }
   }

   return (REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);                     /* The final remainder is the CRC.
 */
}


/*-----------------------------------------------------------------------------
 *      compute_hashtable_fast32X:  Compute the 32 bit CRC and enter in hash table
 *                                 which for netowork masks in DMA
 *  Function is passed the MAC Address of the destination address as AddData
 *
 *  Compute the 32 bit CRC and enter in hash table (this is a test function for 
 *  testing hence X prefix)
 *  Parameters: none data is fixed at 123456789
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void compute_hashtable_fast32X()
{
   unsigned char  test[] = "123456789";
   crc_t crc32Result;                                                           /* 32bit CRC computation
 */
   uint8_t byteRep;                                                             /* byte representation of 32 bit CRC for hash table
 */

   ETHRXFCSET = (0x8000 | ETHRXFCSET);                                          /* Enable Ethernet Filtering
 */
#ifdef USE_CRC_FAST
   crc_init();                                                                  /* initialise look up table
 */
   crc32Result=crc_fast( test, strlen(test));                                   /* calculate fast CRC32 bit
 */
#else
   crc32Result=crc_slow( test, strlen(test));                                   /* calculate fast CRC32 bit
 */
#endif
   byteRep = (uint8_t) ((crc32Result & INT32_TO_BYTE_MASK) >> INT32_TO_BYTE_MOVE);
                                                                                /* calculate a byte from the 32bit CRC
 */
   if (byteRep <= 63)
   {
     ETHHT0 = ((uint64_t) pow(2,byteRep) | ETHHT0);                             /* Set the relevant bit in hash table 1
 */
   }
   else if ((byteRep >=64) && (byteRep <= 127))
   {
    ETHHT1 = ((uint64_t) pow(2,(byteRep >> 64)) | ETHHT1);                      /* Set the relevant bit in hash table 2
 */
   }

}

/*-----------------------------------------------------------------------------
 *      compute_hashtable_fast32:  Compute the 32 bit CRC and enter in hash table
 *                                 which for netowork masks in DMA
 *  Function is passed the MAC Address of the destination address as AddData
 *
 *
 *  Parameters: unsigned char *addData
 *  Return:     void
 *----------------------------------------------------------------------------*/
void compute_hashtable_fast32( unsigned char *addData )
{
   /*unsigned char  test[] = "123456789";
 */
   crc_t crc32Result;                                                           /* 32bit CRC computation
 */
   uint8_t byteRep;                                                             /* byte representation of 32 bit CRC for hash table
 */

   ETHRXFCSET = (0x8000 | ETHRXFCSET);                                          /* Enable Ethernet Filtering
 */
#ifdef USE_CRC_FAST
   crc_init();                                                                  /* initialise look up table
 */
   crc32Result=crc_fast( addData, strlen(addData) );                            /* calculate fast CRC32 bit
 */
#else
   crc32Result=crc_slow( addData, strlen(addData) );                            /* calculate slow CRC32 bit
 */
#endif
   byteRep = (uint8_t) ((crc32Result & INT32_TO_BYTE_MASK) >> INT32_TO_BYTE_MOVE);
                                                                                /* calculate a byte from the 32bit CRC
 */
   if (byteRep <= 63)
   {
     ETHHT0 = ((uint64_t) pow(2,byteRep) | ETHHT0);                             /* Set the relevant bit in hash table 1
 */
   }
   else if ((byteRep >=64) && (byteRep <= 127))
   {
    ETHHT1 = ((uint64_t) pow(2,(byteRep >> 64)) | ETHHT1);                      /* Set the relevant bit in hash table 2
 */
   }

}

/*-----------------------------------------------------------------------------
 *      crc8:  8bit CRC
 *
 *
 *  Parameters: uint8_t const *dataX, int32_t length
 *  Return:     uint8_t the function returns the CRC as a type unsigned char
 *----------------------------------------------------------------------------*/
uint8_t crc8(uint8_t const *dataX, int32_t length)                              /* CRC 8 Calculation
 */
{
   unsigned char crc = 0x00U;
   unsigned char extract;
   unsigned char sum;
   int16_t i;
   uint8_t tempI;

   if (dataX == NULL)
   {
      crc = 0u;
   }
   else
   {
      for(i=0;i<length;i++)
      {
         extract = *dataX;
         for (tempI = 8u; tempI; tempI--)
         {
            sum = (crc ^ extract) & 0x01U;
            crc >>= 1U;
            if (sum)
               crc ^= 0x8CU;
            extract >>= 1U;
         }
         dataX++;
      }
   }
   return crc;
}

/*-----------------------------------------------------------------------------
 *      crc8_dvb_s2:  8bit DVB CRC
 *      CRC for RunCam
 *
 *  Parameters: uint8_t crc, unsigned char a
 *  Return:     uint8_t the function returns the CRC as a type unsigned char
 *----------------------------------------------------------------------------*/
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)                               /* CRC8 DVB S2 algorythm used for RunCam */
{
   int8_t ii;
   
   crc ^= a;
   for (ii = 0U; ii < 8U; ++ii)
   {
      if (crc & 0x80U)
      {
         crc = (crc << 1) ^ 0xD5U;
      }
      else 
      {
         crc = crc << 1u;
      }
    }
    return crc;
}

/*-----------------------------------------------------------------------------
 *      crc8_dvb_s2_update:  8bit DVB CRC
 *      CRC for RunCam
 *
 *  Parameters: uint8_t crc, const void *dataX, uint32_t length
 *  Return:     uint8_t the function returns the CRC as a type unsigned char
 *----------------------------------------------------------------------------*/
uint8_t crc8_dvb_s2_update(uint8_t crc, const void *dataX, uint32_t length)     /* CRC8 DVB-T iteration
 */
{

    const uint8_t *p = (const uint8_t *) dataX;                                 /* type cast the unsigned char* to an int to allow for iteration */
    const uint8_t *pend = p + (uint8_t) length;
    
    if (dataX == NULL)
    {
      crc = 0u;
    }
    else
    {
       for (; p != pend; p++)                                                      /* to the length of data */
       {
           crc = crc8_dvb_s2(crc, (unsigned char) *p);                             /* calculate the iterative crc */
       }
    }

    return crc;

}

/*-----------------------------------------------------------------------------
 *      crc16_arc_calculate:  arc 16 CRC slow
 *      CRC for SimpleBGC version > 2.688 : Think this is type CRC16_ARC
 *
 *  Parameters: uint16_t length, uint8_t const * const p_message
 *  Return:     uint16_t the function returns the CRC as a type unsigned short int
 *----------------------------------------------------------------------------*/
uint16_t crc16_arc_calculate(uint16_t length, uint8_t const * const p_message)
{
  uint16_t counter;
  uint16_t polynom = 0x8005U;                                                   /* 0x8005 poly and crc[0][1] = 0 is ARC implementation */
  uint16_t crc_register = 0U;                                                   /* Initial value is a zero */
  uint8_t shift_register;
  uint8_t data_bit, crc_bit;
  
  if (p_message == NULL)
  {
     crc_register = 0u;
  }
  else
  {
     for (counter = 0u; counter < length; counter++)
     {
        for (shift_register = 0x01U; shift_register > 0x00U; shift_register <<= 1U)
        {
           data_bit = (p_message[counter] & shift_register) ? 1U : 0U;
           crc_bit = crc_register >> 15U;
           crc_register <<= 1U;
           if (data_bit != crc_bit) crc_register ^= polynom;
        }
     }
  }
  return crc_register;                                                          /* Return the calculated CRC
 */
}

/*-----------------------------------------------------------------------------
 *      crc16_arc_fast:  arc 16 CRC fast
 *
 *
 *  Parameters: uint16_t crc, unsigned char const *buffer, uint16_t len
 *  Return:     uint16_t the function returns the CRC as a type unsigned short int
 *----------------------------------------------------------------------------*/
uint16_t crc16_arc_fast(uint16_t crc, unsigned char const *buffer, uint16_t len)
{
   if (buffer == NULL)
   {
      crc = 0u;
   }
   else
   {
      while (len--)
         crc = (uint16_t)((crc >> 8U)^(crc16arc_table[(crc^(*buffer++))&0xffu]));
   }
   return crc;
}

/*-----------------------------------------------------------------------------
 *      usMBCRC16:  modbus RTU CRC
 *
 *
 *  Parameters: unsigned char * pucFrame, uint16_t usLen
 *  Return:     uint16_t the function returns the CRC as a type unsigned short int
 *----------------------------------------------------------------------------*/
uint16_t usMBCRC16( unsigned char * pucFrame, uint16_t usLen )
{
    unsigned char ucCRCHi = 0xFFU;
    unsigned char ucCRCLo = 0xFFU;
    int16_t iIndex;

    if (pucFrame == NULL)
    {
       ucCRCHi = 0u;
       ucCRCLo = 0u;
    }
    else
    {
       while( usLen-- )
       {
          iIndex = ucCRCLo ^ *( pucFrame++ );
          ucCRCLo = ( unsigned char)( ucCRCHi ^ aucCRCHi[iIndex] );
          ucCRCHi = aucCRCLo[iIndex];
       }
    }
    return ( uint16_t )( ucCRCHi << 8U | ucCRCLo );
}

/*-----------------------------------------------------------------------------
 *      usMBAsciiLRC:  modbus ascii LRC
 *
 *
 *  Parameters: unsigned char *auchMsg, uint16_t usLen
 *  Return:     uint8_t the function returns the LRC as a type unsigned char
 *----------------------------------------------------------------------------*/
uint8_t usMBAsciiLRC(unsigned char *auchMsg, uint16_t usLen)
{
   uint8_t uchLRC = 0U;                                                         /* LRC char initialized */

   if (auchMsg != NULL)
   {
      while( usLen-- )
      {
        uchLRC += *auchMsg++;                                                   /* add buffer byte without carry */
      }
   }

   return ( (uint8_t) (-((char)uchLRC)) );                                      /* return twos complement */
}
/* repeat for CCIT-16 becasue we need it as well for UAVCAN bus multi-send
 */
/* Initialise the CRC used with the fast function (noramlly you do it at boot-up)
 */
/*-----------------------------------------------------------------------------
 *      crc_ccit16_init:  initialise lookup table for CCIT-16 crc
 *
 *  Fast implementation means (needs to call crc_init(); at boot
 *
 *  Parameters: void
 *  Return:     void
 *----------------------------------------------------------------------------*/
#define CCIT16_POLYNOMIAL ((int16_t) 0x1021u)
#define CCIT16_INITIAL_REMAINDER ((int16_t) 0xFFFFu)
#define CCIT16_FINAL_XOR_VALUE ((int16_t) 0x0000u)
#define CCIT16_REFLECT_DATA(X) (X)
#define CCIT16_REFLECT_REMAINDER(X) (X)
void crc_ccit16_init(void)
{
   int16_t dividend;
   int16_t remainder;
   int16_t bitcounter;

   for (dividend = 0; dividend < CRC_TABLE_SIZE; dividend++)                    /* Compute the remainder of each possible dividend.
 */
   {
       remainder = dividend << (WIDTH - BITS_PER_BYTE);                         /* Start with the dividend followed by zeros.
 */

       for (bitcounter = BITS_PER_BYTE; (bitcounter > 0); bitcounter--)         /* Perform modulo-2 division, a bit at a time.
 */
       {
           if (remainder & TOPBIT)                                              /* Try to divide the current data bit.
 */
           {
               remainder = (remainder << 1) ^ CCIT16_POLYNOMIAL;
           }
           else
           {
               remainder = (remainder << 1);
           }
   }

   g_ccit16_table[dividend] = remainder;                                            /* Store the result into the table.
 */
  }
}  /* crc_init() */

/*-----------------------------------------------------------------------------
 *      crc_ccit16_fast:  CCIT-16 crc calculation separate for UAVCAN
 *
 *  Fast implementation means (needs to call crc_init(); at boot
 *
 *  Parameters: uint8_t const * const p_message, uint8_t n_bytes
 *  Return:     uint16_t
 *----------------------------------------------------------------------------*/
uint16_t crc_ccit16_fast(uint8_t const * const p_message, uint8_t n_bytes)
{
   uint16_t remainder = CCIT16_INITIAL_REMAINDER;
   uint8_t byte;
   uint8_t dataZ;

   if (p_message != NULL)
   {
      for (byte = 0u;(byte < n_bytes); byte++)                                  /* Divide the message by the polynomial, a byte at a time. */
      {
         dataZ = (CCIT16_REFLECT_DATA(p_message[byte])) ^ (remainder >> (WIDTH - BITS_PER_BYTE));
         remainder = g_ccit16_table[dataZ] ^ (remainder << BITS_PER_BYTE);
      }
   }

   return (CCIT16_REFLECT_REMAINDER(remainder) ^ CCIT16_FINAL_XOR_VALUE);       /* The final remainder is the CRC.
 */
}
/*-----------------------------------------------------------------------------
 *      RobotisServo_crc:  crc for Robotis Servo
 *
 *  Parameters: uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size
 *  Return:     uint16_t
 *----------------------------------------------------------------------------*/
uint16_t RobotisServo_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t i,j;
    static const uint16_t robocrc_table[256] = {0x0000,
                                            0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                                            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
                                            0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
                                            0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
                                            0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
                                            0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
                                            0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
                                            0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
                                            0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                                            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
                                            0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
                                            0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
                                            0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
                                            0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
                                            0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
                                            0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
                                            0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                                            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
                                            0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
                                            0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
                                            0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
                                            0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
                                            0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
                                            0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
                                            0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                                            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
                                            0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
                                            0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
                                            0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
                                            0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
                                            0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
                                            0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
                                            0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                                            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
                                            0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
                                            0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
                                            0x820D, 0x8207, 0x0202 };

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
        crc_accum = (crc_accum << 8) ^ robocrc_table[i];
    }

    return crc_accum;
}
#endif