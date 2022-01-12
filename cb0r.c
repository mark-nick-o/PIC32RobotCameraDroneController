// by jeremie miller - 2015-2017
// public domain UNLICENSE, contributions/improvements welcome via github at https://github.com/quartzjer/cb0r
//
// CBOR
// RFC 7049 Concise Binary Object Representation
// “The Concise Binary Object Representation (CBOR) is a data format whose design goals include the possibility 
// of extremely small code size, fairly small message size, and extensibility without the need for version negotiation.”
//
// Internet of Things (IoT) is the term used to describe the increasing number of connected embedded systems used in a wide variety of applications. 
// M2M (Machine to Machine) and constrained nodes are also common terms. 
// The distinction between the expressions is some what fuzzy but inbroad terms both IoT and M2M are networked systems 
// which are comprised partly or fully out of constrained nodes.
// changed for mikroE C by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#include "cb0r.h"
#include "definitions.h"
#include <stdint.h>
#include "cpu_endian_defs.h"
#ifndef gc_events_H                                                             // if we dont have gc_events just add this global timer 
extern volatile uint64_t g_timer5_counter;                                      // Global Timer Counter volatile as globally written in interrupt
#endif

#if defined(ADV_ENCPT_STD_USED)
//Common interface for PRNG algorithms
const PrngAlgo_t yarrowPrngAlgo =
{
   "Yarrow",
   sizeof(YarrowContext_t),
   (PrngAlgoInit) yarrowInit,
   (PrngAlgoRelease) yarrowRelease,
   (PrngAlgoSeed) yarrowSeed,
   (PrngAlgoAddEntropy) yarrowAddEntropy,
   (PrngAlgoRead) yarrowRead
};
#endif

// unhelpful legacy GCC warning noise for syntax used in cb0r()
//#pragma GCC diagnostic ignored "-Wunknown-pragmas"
//#pragma GCC diagnostic ignored "-Wpragmas"
//#pragma GCC diagnostic ignored "-Winitializer-overrides"
//#pragma GCC diagnostic ignored "-Woverride-init"
//C++ guard

// low-level, pass raw CBOR bytes via start/stop pointers, returns end pointer (== stop if complete)
// can skip a given number of items and then will fill optional result w/ the current item
uint8_t *cb0r(uint8_t *start, uint8_t *stop, uint32_t skip, cb0r_t result);

// safer high-level wrapper to read raw CBOR
bool cb0r_read(uint8_t *in, uint32_t len, cb0r_t result);

// fetch an item from the given array (or map), zero-index
bool cb0r_get(cb0r_t array, uint32_t index, cb0r_t result);

// find and fetch the value of a given key from the map, number/bytes args only used for some types
bool cb0r_find(cb0r_t map, cb0r_e type, uint64_t number, uint8_t *bytes, cb0r_t result);

// convenience method to write a header given a type and optional number (length/count/value), returns bytes written to out (max 9)
uint8_t cb0r_write(uint8_t *out, cb0r_e type, uint64_t number);

// simple wrapper to return a contained value start/length
uint8_t *cb0r_value(cb0r_t data);
uint32_t cb0r_vlen(cb0r_t data);

/* support these cipher modes */
uint8_t cbcEncrypt(const CipherAlgo_t *cipher, void *context, uint8_t *ivXOR, const uint8_t *p, uint8_t *c, size_t length );
uint8_t cbcDecrypt(const CipherAlgo_t *cipher, void *context, uint8_t *ivXOR, const uint8_t *c, uint8_t *p, size_t length);
uint8_t ecbEncrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *p, uint8_t *c, size_t length);
uint8_t ecbDecrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *c, uint8_t *p, size_t length);
uint8_t ofbEncrypt(const CipherAlgo_t *cipher, void *context, uint8_t s,  uint8_t *ivXOR, const uint8_t *p, uint8_t *c, size_t length);
uint8_t ofbDecrypt(const CipherAlgo_t *cipher, void *context, uint8_t s, uint8_t *ivXOR, const uint8_t *c, uint8_t *p, size_t length);
uint8_t xtsInit(XtsContext_t *context, const CipherAlgo_t *cipherAlgo, const void *key, size_t keyLen);
uint8_t xtsEncrypt(XtsContext_t *context, const uint8_t *i, const uint8_t *p, uint8_t *c, size_t length);
uint8_t xtsDecrypt(XtsContext_t *context, const uint8_t *i, const uint8_t *c, uint8_t *p, size_t length);
/* CCM is a block cipher mode of encryption which does both con?dentiality and integrity protection it is ued with AES cipher in OSCoAP */
void ccmIncCounter(uint8_t *x, size_t n);
void ccmXorBlock(uint8_t *x, const uint8_t *a, const uint8_t *b, size_t n);
uint8_t ccmDecrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen, const uint8_t *c, uint8_t *p, size_t length, const uint8_t *t, size_t tLen);
uint8_t ccmEncrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen, const uint8_t *p, uint8_t *c, size_t length, uint8_t *t, size_t tLen);

#if defined(ALGO_CHA_CHA)
uint8_t chachaInit(ChachaContext_t *context, uint8_t nr, const uint8_t *key, size_t keyLen, const uint8_t *nonce, size_t nonceLen);
void chachaProcessBlock(ChachaContext_t *context);
void chachaCipher(ChachaContext_t *context, const uint8_t *input, uint8_t *output, size_t length);
#elif defined(ALGO_SEED)
uint8_t seedInit(SeedContext_t *context, const uint8_t *key, size_t keyLen);
void seedEncryptBlock(SeedContext_t *context, const uint8_t *input, uint8_t *output);
void seedDecryptBlock(SeedContext_t *context, const uint8_t *input, uint8_t *output);
#elif defined(ALGO_CAMELLIA)
uint8_t camelliaInit(CamelliaContext_t *context, const uint8_t *key, size_t keyLen);
void camelliaEncryptBlock(CamelliaContext_t *context, const uint8_t *input, uint8_t *output);
void camelliaDecryptBlock(CamelliaContext_t *context, const uint8_t *input, uint8_t *output);
#elif defined(ADV_ENCPT_STD_USED)
uint8_t aesInit(AesContext_t *context, const uint8_t *key, size_t keyLen);
void aesEncryptBlock(AesContext_t *context, const uint8_t *input, uint8_t *output);
void aesDecryptBlock(AesContext_t *context, const uint8_t *input, uint8_t *output);
#elif defined(ALGO_ARIA)
uint8_t ariaInit(AriaContext_t *context, const uint8_t *key, size_t keyLen);
void ariaEncryptBlock(AriaContext_t *context, const uint8_t *input, uint8_t *output);
void ariaDecryptBlock(AriaContext_t *context, const uint8_t *input, uint8_t *output);
#elif defined(ALGO_SALSA20)
void salsa20ProcessBlock(const uint8_t *input, uint8_t *output, uint8_t nr);
#elif defined(ALGO_DES)
uint8_t desInit(DesContext_t *context, const uint8_t *key, size_t keyLen);
void desEncryptBlock(DesContext_t *context, const uint8_t *input, uint8_t *output);
void desDecryptBlock(DesContext_t *context, const uint8_t *input, uint8_t *output);
#endif /* end algoryhtm types */
void poly1305Init(Poly1305Context_t *context, const uint8_t *key);
void poly1305Update(Poly1305Context_t *context, const void *dataV, size_t length);
void poly1305Final(Poly1305Context_t *context, uint8_t *tag);
void poly1305ProcessBlock(Poly1305Context_t *context);
#if defined(ALGO_CHA_CHA)
uint8_t chacha20Poly1305Decrypt(const uint8_t *k, size_t kLen, const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen,  const uint8_t *c, uint8_t *p, size_t length, const uint8_t *t, size_t tLen);
uint8_t chacha20Poly1305Encrypt(const uint8_t *k, size_t kLen,const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen, const uint8_t *p, uint8_t *c, size_t length, uint8_t *t, size_t tLen);
#endif /* end chacha 20 poly 1305 */
#ifdef MAX_CIPHER_BLOCK_SIZE                                                    /* we are using a ciper compatible with cmac */
void cmacReset(CmacContext_t *context);
void cmacMul(uint8_t *x, const uint8_t *a, size_t n, uint8_t rb);
uint8_t cmacInit(CmacContext_t *context, const CipherAlgo_t *cipher, const void *key, size_t keyLen);
void cmacXorBlock(uint8_t *x, const uint8_t *a, const uint8_t *b, size_t n);
void cmacUpdate(CmacContext_t *context, const void *dataV, size_t dataLen);
uint8_t cmacFinal(CmacContext_t *context, uint8_t *mac, size_t macLen);
#endif  /* end of cmac */
#if defined(IOT_SHA_256)
void sha256Init(Sha256Context_t *context);
void sha256ProcessBlock(Sha256Context_t *context);
void sha256FinalRaw(Sha256Context_t *context, uint8_t *digest);
void sha256Update(Sha256Context_t *context, const void *dataV, size_t length);
void sha256Final(Sha256Context_t *context, uint8_t *digest);
#endif /* end SHA256 hash selection */
#if defined(ADV_ENCPT_STD_USED)                                                 /* needs ciper to be AES */
uint8_t yarrowInit(YarrowContext_t *context);
void yarrowRelease(YarrowContext_t *context);
uint8_t osCreateMutex(YarrowContext_t *context);
uint8_t osDeleteMutex(YarrowContext_t *context);
void yarrowFastReseed(YarrowContext_t *context);
void yarrowGenerateBlock(YarrowContext_t *context, uint8_t *output);
uint8_t yarrowSeed(YarrowContext_t *context, const uint8_t *input, size_t length);
uint8_t yarrowRead(YarrowContext_t *context, uint8_t *output, size_t length);
void yarrowSlowReseed(YarrowContext_t *context);
uint8_t yarrowAddEntropy(YarrowContext_t *context, uint8_t source,  const uint8_t *input, size_t length, size_t entropy);
uint8_t yarrowBegin(YarrowContext_t yarrowContext, uint8_t seed[32u]);
void AddRoundKey(uint8_t round, aes_state_t* state, const uint8_t* RoundKey);
void SubBytes(aes_state_t* state);
void ShiftRows(aes_state_t* state);
uint8_t xtime(uint8_t x);
void MixColumns(aes_state_t* state);
void Cipher(aes_state_t* state, const uint8_t* RoundKey);
void AES_CTR_xcrypt_buffer(AES_ctx_t* ctx, uint8_t* buf, uint32_t length);
void XorWithIv(uint8_t* buf, const uint8_t* Iv);
void AES_CBC_encrypt_buffer(AES_ctx_t *ctx, uint8_t* buf, uint32_t length);
void InvMixColumns(aes_state_t* state);
void InvShiftRows(aes_state_t* state);
void InvSubBytes(aes_state_t* state);
void InvCipher(aes_state_t* state, const uint8_t* RoundKey);
void AES_CBC_decrypt_buffer(AES_ctx_t* ctx, uint8_t* buf,  uint32_t length);
void AES_ECB_encrypt(const AES_ctx_t* ctx, uint8_t* buf);
void AES_ECB_decrypt(const AES_ctx_t* ctx, uint8_t* buf);
#endif /* end of yarrow prng */

#if defined(USE_SHA_512)
uint64_t SHA512_Ch(uint64_t x, uint64_t y, uint64_t z);
uint64_t SHA512_Maj(uint64_t x, uint64_t y, uint64_t z);
uint64_t SHA512_SigmaB0(uint64_t x);
uint64_t SHA512_SigmaB1(uint64_t x);
uint64_t SHA512_sigmaL0(uint64_t x);
uint64_t SHA512_sigmaL1(uint64_t x);
void SHA512_Round(uint64_t a, uint64_t b, uint64_t c, uint64_t* d, uint64_t e, uint64_t f, uint64_t g, uint64_t* h, uint64_t k, uint64_t w );
void SHA512_Initialize(uint64_t* s);
uint64_t SHA512_ReadBE64(const unsigned char* ptr);
void SHA512_Transform(uint64_t* s, const unsigned char* chunk);
void CSHA512_Write(unsigned char* dataV, unsigned char* buf, size_t len, size_t *bytes, uint64_t* s);
void SHA512_WriteBE64(unsigned char* ptr, uint64_t x);
void CSHA512_Finalize(unsigned char hash[SHA512_OUTPUT_SIZE], unsigned char* buf, size_t *bytes, uint64_t* s);
#endif

// start at bin, returns end pointer (== stop if complete), either skips items or extracts result of current item
/*-----------------------------------------------------------------------------
 *      cb0r():  perform cbor
 *
 *  Parameters: uint8_t *start, uint8_t *stop, uint32_t skip, cb0r_t result
 *
 *  Return: uint8_t *
 *             
 *----------------------------------------------------------------------------*/
uint8_t *cb0r(uint8_t *start, uint8_t *stop, uint32_t skip, cb0r_t result)
{

  cb0r_e cbType = CB0R_ERR;
  uint8_t size = 0u;
  uint32_t count = 0u;
  uint8_t *end = start + 1u;
  
  if ((start==NULL) || (stop==NULL))                                            /* check FOR null pointers */
  {
     return 0u;
  }
  
#ifdef jump_tables_work                                                         // jump tables dont work in mikroE C

  // type byte is fully unrolled for structure only      THIS STRUT DEFINES THE JUMP TABLE ADDRESSES
  //static const void *go[] RODATA_SEGMENT_CONSTANT =
  static void *go[] =
  {
    //                                                                          [0x00 ... 0xff] =
       &&l_ebad,
    // first 5 bits
    //                                                                          [0x00 ... 0x17] =
        &&l_int,
    // Major type 1 CB0R_INT
    //[0x18] =
        &&l_int1, //
        // [0x19] =
        &&l_int2,
        //[0x1a] =
        &&l_int4,
    //        [0x1b] =
        &&l_int8,
    //                                                                          [0x20 ... 0x37] =
        &&l_int,
    // Major type 2 CB0R_NEG
    // [0x38] =
        &&l_int1,
        // [0x39] =
        &&l_int2,
        // [0x3a] =
        &&l_int4,
    //        [0x3b] =
        &&l_int8,
    //                                                                          [0x40 ... 0x57] =
        &&l_byte,
    // Major type 3 CB0R_BYTE
    // [0x58] =
        &&l_byte1,
        // [0x59] =
        &&l_byte2,
        // [0x5a] =
        &&l_byte4,
        //[0x5b] =
        &&l_ebig,
    // [0x5f] =
        &&l_until,
    // Major type 4 CB0R_UTF8
    //                                                                          [0x60 ... 0x77] =
        &&l_byte,
    // [0x78] =
        &&l_byte1,
        // [0x79] =
        &&l_byte2,
        // [0x7a] =
        &&l_byte4,
        // [0x7b] =
        &&l_ebig,
    // [0x7f] =
        &&l_until,
    // Major type 5 CB0R_ARRAY
    //                                                                          [0x80 ... 0x97] =
        &&l_array,
    // [0x98] =
        &&l_array1,
        // [0x99] =
        &&l_array2,
        // [0x9a] =
        &&l_array4,
        //[0x9b] =
        &&l_ebig,
    // [0x9f] =
        &&l_until,
    // Major type 6 CB0R_MAP
    //                                                                          [0xa0 ... 0xb7] =
        &&l_array,
    // [0xb8] =
        &&l_array1,
        // [0xb9] =
        &&l_array2,
        //[0xba] =
        &&l_array4,
        // [0xbb] =
        &&l_ebig,
    // [0xbf] =
        &&l_until,
    // Major type 7 CB0R_TAG
    //                                                                          [0xc0 ... 0xd7] =
        &&l_tag,
    // [0xd8] =
        &&l_tag1,
        //[0xd9] =
        &&l_tag2,
        //[0xda] =
        &&l_tag4,
        // [0xdb] =
        &&l_tag8,
    // Major type 8 CB0R_SIMPLE / CB0R_FLOAT
    //                                                                          [0xe0 ... 0xf7] =
        &&l_int,
    //[0xf8] =
        &&l_int1,
        // [0xf9] =
        &&l_int2,
        //[0xfa] =
        &&l_int4,
        // [0xfb] =
        &&l_int8,
    //[0xff] =
        &&l_break
  };
#endif

  if(end > stop) {
    if(result) result->type = CB0R_ERR;
    return stop;
  }

#ifdef jump_tables_work
  goto *go[*start];                                                             // ======= DOESNT SEEM TO WORK in mikroE C ============
#else
  switch(*start)
  {
       case 0u:
       goto l_ebad;
       break;
    // first 5 bits
    //                                                                          [0x00 ... 0x17] =
       case 1u:
       goto l_int;
       break;
       
    // Major type 1 CB0R_INT
    //[0x18] =
       case 2u:
       goto l_int1;
       break;
        // [0x19] =
        
       case 3u:
       goto l_int2;
       break;
       
       case 4u:
        //[0x1a] =
       goto l_int4;
       break;
       
    //        [0x1b] =
       case 5u:
       goto l_int8;
       break;
    //                                                                          [0x20 ... 0x37] =
       case 6u:
       goto l_int;
       break;
    // Major type 2 CB0R_NEG
    // [0x38] =
       case 7u:
       goto l_int1;
       break;
        // [0x39] =
       case 8u:
       goto l_int2;
       break;
        // [0x3a] =
       case 9u:
       goto l_int4;
       break;
    //        [0x3b] =
       case 10u:
       goto l_int8;
       break;
    //                                                                          [0x40 ... 0x57] =
       case 11u:
       goto l_byte;
       break;
    // Major type 3 CB0R_BYTE
    // [0x58] =
       case 12u:
       goto l_byte1;
       break;
        // [0x59] =
       case 13u:
       goto l_byte2;
       break;
        // [0x5a] =
       case 14u:
       goto l_byte4;
       break;
        //[0x5b] =
       case 15u:
       goto l_ebig;
       break;
    // [0x5f] =
       case 16u:
       goto l_until;
       break;
    // Major type 4 CB0R_UTF8
    //                                                                          [0x60 ... 0x77] =
       case 17u:
       goto l_byte;
       break;
    // [0x78] =
       case 18u:
       goto l_byte1;
       break;
        // [0x79] =
       case 19u:
       goto l_byte2;
       break;
        // [0x7a] =
       case 20u:
       goto l_byte4;
        // [0x7b] =
       case 21u:
       goto l_ebig;
       break;
    // [0x7f] =
       case 22u:
       goto l_until;
       break;
    // Major type 5 CB0R_ARRAY
    //                                                                          [0x80 ... 0x97] =
       case 23u:
       goto l_array;
       break;
    // [0x98] =
       case 24u:
       goto l_array1;
       break;
        // [0x99] =
       case 25u:
       goto l_array2;
       break;
        // [0x9a] =
       case 26u:
       goto l_array4;
       break;
        //[0x9b] =
       case 27u:
       goto l_ebig;
       break;
    // [0x9f] =
       case 28u:
       goto l_until;
       break;
    // Major type 6 CB0R_MAP
    //                                                                          [0xa0 ... 0xb7] =
       case 29u:
       goto l_array;
       break;
    // [0xb8] =
       case 30u:
       goto l_array1;
       break;
        // [0xb9] =
       case 31u:
       goto l_array2;
       break;
        //[0xba] =
       case 32u:
       goto l_array4;
       break;
        // [0xbb] =
       case 33u:
       goto l_ebig;
       break;
    // [0xbf] =
       case 34u:
       goto l_until;
       break;
    // Major type 7 CB0R_TAG
    //                                                                          [0xc0 ... 0xd7] =
       case 35u:
       goto l_tag;
       break;
    // [0xd8] =
       case 36u:
       goto l_tag1;
       break;
        //[0xd9] =
       case 37u:
       goto l_tag2;
       break;
        //[0xda] =
       case 38u:
       goto l_tag4;
       break;
        // [0xdb] =
       case 39u:
       goto l_tag8;
       break;
    // Major type 8 CB0R_SIMPLE / CB0R_FLOAT
    //                                                                          [0xe0 ... 0xf7] =
       case 40u:
       goto l_int;
       break;
    //[0xf8] =
       case 41u:
       goto l_int1;
       break;
        // [0xf9] =
       case 42u:
       goto l_int2;
       break;
        //[0xfa] =
       case 43u:
       goto l_int4;
       break;
        // [0xfb] =
       case 44u:
       goto l_int8;
       break;
    //[0xff] =
       case 45u:
       goto l_break;
       break;
       
       default:
       cbType = CB0R_ERR;
       goto l_fail;
       break;
  }
#endif
  // all types using integer structure
  l_int8:
    end += 4u;
  l_int4:
    end += 2u;
  l_int2:
    end += 1u;
  l_int1:
    end += 1u;
  l_int:
    goto l_finish;
  // bytes and string structures
  l_byte4:
    size = 2u;
    end += (uint32_t)(start[1u]) << 24u;
    end += (uint32_t)(start[2u]) << 16u;
  l_byte2:
    size += 1u;
    end += (uint32_t)(start[size]) << 8u;
  l_byte1:
    size += 1u;
    end += start[size] + size;
    goto l_finish;
  l_byte:
    end += (start[0u] & 0x1fu);
    goto l_finish;

  // array and map structures
  l_array4:
    size = 2u;
    count += (uint32_t)(start[1u]) << 24u;
    count += (uint32_t)(start[2u]) << 16u;
  l_array2:
    size += 1u;
    count += (uint32_t)(start[size]) << 8u;
  l_array1:
    size += 1u;
    count += start[size];
    goto l_skip;
  l_array:
    count = (start[0u] & 0x1fu);
    goto l_skip;

  // skip fixed count of items in an array/map
  l_skip:

    if(count) {
      // double map for actual count
      if(start[0u] & 0x20u) count <<= 1;
      end = cb0r(start+size+1u,stop,count-1u,NULL);
    }else{
      end += size;
    }
    goto l_finish;

  // cross between l_int and l_array
  l_tag8:
    size = 4u;

  l_tag4:
    size += 2u;

  l_tag2:
    size += 1u;

  l_tag1:
    size += 1u;

  l_tag:
    // tag is like an array of 1, just grabs next item
    end = cb0r(start+size+1u,stop,0u,NULL);
    goto l_finish;

  // indefinite length wrapper
  l_until:
    count = CB0R_STREAM;
    end = cb0r(start+1u,stop,count,NULL);
    goto l_finish;

  l_break: {
    if(skip == CB0R_STREAM) return end;
    goto l_eparse;
  }

  l_ebad:
    cbType = CB0R_EBAD;
    goto l_fail;

  l_eparse:
    cbType = CB0R_EPARSE;
    goto l_fail;

  l_ebig:
    cbType = CB0R_EBIG;
    goto l_fail;

  l_fail: // all errors
    skip = 0u;

  l_finish: // only first 7 types
    // warning cbType = ((start[0u] >> 5u) & UINT8_MAX);                        gives warning hence long code below for enum type warning
    switch (((start[0u] >> 5u) & UINT8_MAX))
    {
        case CB0R_INT:
        cbType = CB0R_INT;
        break;
  
        case CB0R_NEG:
        cbType = CB0R_NEG;
        break;
  
        case CB0R_BYTE:
        cbType = CB0R_NEG;
        break;
        
        case CB0R_UTF8:
        cbType = CB0R_UTF8;
        break;
  
        case CB0R_ARRAY:
        cbType = CB0R_ARRAY;
        break;

        case CB0R_MAP:
        cbType = CB0R_MAP;
        break;
  
        case CB0R_TAG:
        cbType = CB0R_TAG;
        break;
  
        case CB0R_SIMPLE:
        cbType = CB0R_SIMPLE;
        break;

        case CB0R_TAGS:                                                         // if(type > CB0R_TAGS && type < CB0R_SIMPLES)
        cbType = CB0R_TAGS;
        break;
  
        case CB0R_DATETIME:
        cbType = CB0R_DATETIME;
        break;
  
        case CB0R_EPOCH:
        cbType = CB0R_EPOCH;
        break;
  
        case CB0R_BIGNUM:
        cbType = CB0R_BIGNUM;
        break;
  
        case CB0R_BIGNEG:
        cbType = CB0R_BIGNEG;
        break;
  
        case CB0R_FRACTION:
        cbType = CB0R_FRACTION;
        break;
 
        case CB0R_BIGFLOAT:
        cbType = CB0R_BIGFLOAT;
        break;
  
        case CB0R_BASE64URL:
        cbType = CB0R_BASE64URL;
        break;

        case CB0R_BASE64:
        cbType = CB0R_BASE64;
        break;

        case CB0R_HEX:
        cbType = CB0R_HEX;
        break;

        case CB0R_DATA:
        cbType = CB0R_DATA;
        break;

        case CB0R_SIMPLES:                                                      // expand standard simple values
        cbType = CB0R_SIMPLES;
        break;

        case CB0R_FALSE:
        cbType = CB0R_FALSE;
        break;

        case CB0R_TRUE:
        cbType = CB0R_TRUE;
        break;

        case CB0R_NULL:
        cbType = CB0R_NULL;
        break;

        case CB0R_UNDEF:
        cbType = CB0R_UNDEF;
        break;

        case CB0R_FLOAT:
        cbType = CB0R_FLOAT;
        break;

        case CB0R_ERR:                                                          // if(type >= CB0R_ERR) ...
        cbType = CB0R_ERR;
        break;

        case CB0R_EPARSE:                                                       // invalid structure
        cbType = CB0R_EPARSE;
        break;

        case CB0R_EBAD:                                                         // invalid type byte
        cbType = CB0R_EBAD;
        break;

        case CB0R_EBIG:                                                         // unsupported size item
        cbType = CB0R_EBIG;
        break;

        case CB0R_EMAX:
        cbType = CB0R_EMAX;
        break;

        default:
        break;
   }
  // done done, extract value if result requested
  if(!skip)
  {
    if(!result) return end;
    result->start = start;
    result->end = end;
    result->type = cbType;
    result->value = 0u;
    switch(cbType)
    {
      case CB0R_INT:
      case CB0R_NEG:
        size = end - (start + 1);
      case CB0R_TAG: {
        switch(size)
        {
          case 8u:
            result->value |= (uint64_t)(start[size - 7u]) << 56u;
            result->value |= (uint64_t)(start[size - 6u]) << 48u;
            result->value |= (uint64_t)(start[size - 5u]) << 40u;
            result->value |= (uint64_t)(start[size - 4u]) << 32u;

          case 4u:
            result->value |= (uint32_t)(start[size - 3u]) << 24u;
            result->value |= (uint32_t)(start[size - 2u]) << 16u;

          case 2u:
            result->value |= (uint32_t)(start[size - 1u]) << 8u;

          case 1u:
            result->value |= start[size];
            break;

          case 0u:
            result->value = start[0u] & 0x1fu;
            if(cbType == CB0R_TAG) switch(result->value)
            {
              case 0u: result->type = CB0R_DATETIME; break;
              case 1u: result->type = CB0R_EPOCH; break;
              case 2u: result->type = CB0R_BIGNUM; break;
              case 3u: result->type = CB0R_BIGNEG; break;
              case 4u: result->type = CB0R_FRACTION; break;
              case 5u: result->type = CB0R_BIGFLOAT; break;
              case 21u: result->type = CB0R_BASE64URL; break;
              case 22u: result->type = CB0R_BASE64; break;
              case 23u: result->type = CB0R_HEX; break;
              case 24u: result->type = CB0R_DATA; break;
            }
        }
      } break;

      case CB0R_BYTE:
      case CB0R_UTF8: {
        if(count == CB0R_STREAM) result->count = count;
        else result->length = end - (start + 1u);
      } break;

      case CB0R_ARRAY:
      case CB0R_MAP: {
        result->count = count;
      } break;


      case CB0R_SIMPLE: {
        result->value = (start[0u] & 0x1fu);
        switch(result->value)
        {
          case 20u: result->type = CB0R_FALSE; break;
          case 21u: result->type = CB0R_TRUE; break;
          case 22u: result->type = CB0R_NULL; break;
          case 23u: result->type = CB0R_UNDEF; break;
          case 24u:
            if(start[1u] >= 32u) result->value = start[1u];
            else result->type = CB0R_EBAD;
            break;
          case 25u:
            result->type = CB0R_FLOAT;
            result->length = 2u;
            break;

          case 26u:
            result->type = CB0R_FLOAT;
            result->length = 4u;
            break;

          case 27u:
            result->type = CB0R_FLOAT;
            result->length = 8u;
            break;
        }
      } break;

      default: {
        if(result->type < CB0R_ERR) result->type = CB0R_ERR;
      }
    }
    result->header = size + 1u;
    return end;
  }

  // max means indefinite mode skip
  if(skip != CB0R_STREAM) skip--;
  else if(result) result->count++;

  // tail recurse while skipping to not stack bloat
  return cb0r(end, stop, skip, result);
}

// safer high-level wrapper to read raw CBOR
bool cb0r_read(uint8_t *in, uint32_t len, cb0r_t result)
{
  if (in == NULL) return false;
  if(!in || !len || !result) return false;
  cb0r(in, in+len, 0u, result);
  if(result->type >= CB0R_ERR) return false;
  return true;
}

// fetch a given item from an array (or map), 0 index
bool cb0r_get(cb0r_t array, uint32_t index, cb0r_t result)
{
  if(!array || !result) return false;
  if(array->type != CB0R_ARRAY && array->type != CB0R_MAP) return false;
  cb0r(array->start+array->header, array->end, index, result);
  if(result->type >= CB0R_ERR) return false;
  return true;
}

// get the value of a given key from a map, number/bytes only used for some types
bool cb0r_find(cb0r_t map, cb0r_e type, uint64_t number, uint8_t *bytes, cb0r_t result)
{
  uint32_t i;
  if (bytes == NULL) return false;
  if(!map || !result) return false;
  if(map->type != CB0R_MAP) return false;

  for(i = 0u; i < map->length * 2u; i += 2u) 
  {
    if(!cb0r_get(map, i, result)) return false;
    if(result->type != type) continue;
    // either number compare or number+bytes compare
    switch(type) {
      case CB0R_INT:

      case CB0R_NEG:

      case CB0R_SIMPLE:

      case CB0R_DATETIME:

      case CB0R_EPOCH:

      case CB0R_BIGNUM:

      case CB0R_BIGNEG:

      case CB0R_FRACTION:

      case CB0R_BIGFLOAT:

      case CB0R_BASE64URL:

      case CB0R_BASE64:

      case CB0R_HEX:

      case CB0R_DATA:

      case CB0R_FALSE:

      case CB0R_TRUE:

      case CB0R_NULL:

      case CB0R_UNDEF:
        if(number == result->value) break;
        continue;

      case CB0R_BYTE:

      case CB0R_UTF8:

      case CB0R_FLOAT:
        // compare value by given length
        if(number == result->length && memcmp(bytes, result->start + result->header, number) == 0u) break;
        continue;

      case CB0R_MAP:

      case CB0R_ARRAY:

      case CB0R_TAG:
        // compare value by parsed byte length
        if(number == (uint64_t)(result->end - (result->start + result->header)) && memcmp(bytes, result->start + result->header, number) == 0u) break;
        continue;

      default:
        continue;
    }

    // key matched
    if(!cb0r_get(map, i+1u, result)) return false;
    return true;
  }

  return false;
}

// simple wrappers to return start and length
uint8_t *cb0r_value(cb0r_t dataV)
{
  if(!dataV) return NULL;
  return dataV->start + dataV->header;
}

uint32_t cb0r_vlen(cb0r_t dataV)
{
  if(!dataV) return 0u;
  return dataV->end - cb0r_value(dataV);
}

// defined everywhere but OSX, copied from https://gist.github.com/yinyin/2027912
#ifdef __APPLE__
#include <libkern/OSByteOrder.h>
#define htobe16(x) OSSwapHostToBigInt16(x)
#define htole16(x) OSSwapHostToLittleInt16(x)
#define be16toh(x) OSSwapBigToHostInt16(x)
#define le16toh(x) OSSwapLittleToHostInt16(x)
#define htobe32(x) OSSwapHostToBigInt32(x)
#define htole32(x) OSSwapHostToLittleInt32(x)
#define be32toh(x) OSSwapBigToHostInt32(x)
#define le32toh(x) OSSwapLittleToHostInt32(x)
#define htobe64(x) OSSwapHostToBigInt64(x)
#define htole64(x) OSSwapHostToLittleInt64(x)
#define be64toh(x) OSSwapBigToHostInt64(x)
#define le64toh(x) OSSwapLittleToHostInt64(x)
#else
//                                               #include "cpu_endian.h"
#endif /* __APPLE__ */

/*-----------------------------------------------------------------------------
 *      cb0r_write():  write a cBOR type
 *
 *  Parameters: uint8_t *out, cb0r_e type, uint64_t number
 *
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t cb0r_write(uint8_t *out, cb0r_e type, uint64_t number)
{
  if (out == NULL) return 0u;
  if(type >= CB0R_ERR) return 0u;

  switch(type) {                                                                // built-in types

    case CB0R_DATETIME: number = 0u; break;

    case CB0R_EPOCH: number = 1u; break;

    case CB0R_BIGNUM: number = 2u; break;

    case CB0R_BIGNEG: number = 3u; break;

    case CB0R_FRACTION: number = 4u; break;

    case CB0R_BIGFLOAT: number = 5u; break;

    case CB0R_BASE64URL: number = 21u; break;

    case CB0R_BASE64: number = 22u; break;

    case CB0R_HEX: number = 23u; break;

    case CB0R_DATA: number = 24u; break;

    case CB0R_FALSE: number = 25u; break;

    case CB0R_TRUE: number = 20u; break;

    case CB0R_NULL: number = 21u; break;

    case CB0R_UNDEF: number = 22u; break;

    case CB0R_FLOAT: {                                                          // incoming number is size of float

      if(number == 2u) number = 25u;

      else if(number == 4u) number = 26u;

      else if(number == 8u) number = 27u;

      else return 0u;
    }

    default:;
  }

  out[0u] = type << 5u;

  if(number <= 23u) {
    out[0u] |= number;
    return 1u;
  }

  if(number >= UINT32_MAX) {
    out[0u] |= 27u;
    number = htobe64(number);
    memcpy(out + 1u, &number, 8u);
    out[9u] = '\0';                                                             // terminate the copy
    return 9u;
  }

  if(number > UINT16_MAX) {
    out[0u] |= 26u;
    number = htobe32(number);
    memcpy(out + 1u, &number, 4u);
    out[5u] = '\0';                                                             // terminate the copy
    return 5u;
  }

  if(number >= UINT8_MAX) {
    out[0u] |= 25u;
    number = htobe16(number);
    memcpy(out + 1u, &number, 2u);
    out[3u] = '\0';                                                             // terminate the copy
    return 3u;
  }

  out[0u] |= 24u;
  out[1u] = number;
  return 2u;
}

/**
 * @brief CBC encryption
 * @param[in] cipher Cipher algorithm
 * @param[in] context Cipher algorithm context
 * @param[in,out] iv Initialization vector
 * @param[in] p Plaintext to be encrypted
 * @param[out] c Ciphertext resulting from the encryption
 * @param[in] length Total number of data bytes to be encrypted
 * @return Error code
 **/
uint8_t cbcEncrypt(const CipherAlgo_t *cipher, void *context, uint8_t *ivXOR, const uint8_t *p, uint8_t *c, size_t length)
{
   size_t i;
   
   if (((cipher==NULL) || (ivXOR==NULL)) || ((p==NULL) || (c==NULL)))           // null pointer passed then return
     return 0u;
     
   while(length >= cipher->blockSize)                                           //CBC mode operates in a block-by-block fashion
   {
      for(i = 0; i < cipher->blockSize; i++)                                    //XOR input block with IV contents
      {
         c[i] = p[i] ^ ivXOR[i];
      }

      cipher->encryptBlock(context, c, c);                                      // Encrypt the current block based upon the output of the previous encryption
      //cryptoMemcpy(iv, c, cipher->blockSize);
      memcpy((void*)ivXOR,(void*)c,cipher->blockSize);                          // Update IV with output block contents

      p += cipher->blockSize;                                                   // Next block
      c += cipher->blockSize;
      length -= cipher->blockSize;
   }
   if(length != 0u)                                                             // The plaintext must be a multiple of the block size
      return 0u;
   return 1u;                                                                   // Successful encryption
}

/**
 * @brief CBC decryption
 * @param[in] cipher Cipher algorithm
 * @param[in] context Cipher algorithm context
 * @param[in,out] iv Initialization vector
 * @param[in] c Ciphertext to be decrypted
 * @param[out] p Plaintext resulting from the decryption
 * @param[in] length Total number of data bytes to be decrypted
 * @return Error code
 **/
uint8_t cbcDecrypt(const CipherAlgo_t *cipher, void *context, uint8_t *ivXOR, const uint8_t *c, uint8_t *p, size_t length)
{
   size_t i;
   uint8_t t[16u];

   if (((cipher==NULL) || (ivXOR==NULL)) || ((p==NULL) || (c==NULL)))           // null pointer passed then return
     return 0u;
     
   while(length >= cipher->blockSize)                                           // CBC mode operates in a block-by-block fashion
   {
      memcpy((void*)t,(void*)c, cipher->blockSize);                             // Save input block
      cipher->decryptBlock(context, c, p);                                      // Decrypt the current block
      for(i = 0; i < cipher->blockSize; i++)                                    // XOR output block with IV contents
      {
         p[i] ^= ivXOR[i];
      }
      memcpy((void*)ivXOR,(void*)t, cipher->blockSize);                         // Update IV with input block contents
      c += cipher->blockSize;                                                   // Next block
      p += cipher->blockSize;
      length -= cipher->blockSize;
   }
   if(length != 0u)                                                             // The ciphertext must be a multiple of the block size
      return 0u;
   return 1u;                                                                   // Successful encryption
}

/**
 * @brief ECB encryption
 * @param[in] cipher Cipher algorithm
 * @param[in] context Cipher algorithm context
 * @param[in] p Plaintext to be encrypted
 * @param[out] c Ciphertext resulting from the encryption
 * @param[in] length Total number of data bytes to be encrypted
 * @return Error code
 **/
uint8_t ecbEncrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *p, uint8_t *c, size_t length)
{
   if (((cipher==NULL) || (context==NULL)) || ((p==NULL) || (c==NULL)))         // null pointer passed then return
     return 0u;
     
   while(length >= cipher->blockSize)                                           // ECB mode operates in a block-by-block fashion
   {
      cipher->encryptBlock(context, p, c);                                      // Encrypt current block
      p += cipher->blockSize;                                                   // Next block
      c += cipher->blockSize;
      length -= cipher->blockSize;
   }
   if(length != 0u)                                                             // The plaintext must be a multiple of the block size
      return 0u;
   return 1u;                                                                   // Successful encryption
}

/**
 * @brief ECB decryption
 * @param[in] cipher Cipher algorithm
 * @param[in] context Cipher algorithm context
 * @param[in] c Ciphertext to be decrypted
 * @param[out] p Plaintext resulting from the decryption
 * @param[in] length Total number of data bytes to be decrypted
 * @return Error code
 **/
uint8_t ecbDecrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *c, uint8_t *p, size_t length)
{
   if (((cipher==NULL) || (context==NULL)) || ((p==NULL) || (c==NULL)))         // null pointer passed then return
     return 0u;
     
   while(length >= cipher->blockSize)                                           // ECB mode operates in a block-by-block fashion
   {
      cipher->decryptBlock(context, c, p);                                      // Decrypt current block
      c += cipher->blockSize;                                                   // Next block
      p += cipher->blockSize;
      length -= cipher->blockSize;
   }
   if(length != 0u)                                                             // The ciphertext must be a multiple of the block size
      return 0u;
   return 1u;                                                                   // Successful encryption
}

/**
 * @brief OFB encryption
 * @param[in] cipher Cipher algorithm
 * @param[in] context Cipher algorithm context
 * @param[in] s Size of the plaintext and ciphertext segments
 * @param[in,out] iv Initialization vector
 * @param[in] p Plaintext to be encrypted
 * @param[out] c Ciphertext resulting from the encryption
 * @param[in] length Total number of data bytes to be encrypted
 * @return Error code
 **/
uint8_t ofbEncrypt(const CipherAlgo_t *cipher, void *context, uint8_t s,  uint8_t *ivXOR, const uint8_t *p, uint8_t *c, size_t length)
{
   size_t i;
   size_t n;
   uint8_t o[16u];

   if ((((cipher==NULL) || (context==NULL)) || ((p==NULL) || (c==NULL))) || (ivXOR==NULL))  // null pointer passed then return
     return 0u;
     
   if((s % 8u) != 0u)                                                           //The parameter must be a multiple of 8
      return 0u;
   s = s / 8u;                                                                  //Determine the size, in bytes, of the plaintext and ciphertext segments
   if(s < 1u || s > cipher->blockSize)                                          //Check the resulting value
      return 0u;
   while(length > 0u)                                                           //Process each plaintext segment
   {
      n = min(length, s);                                                       //Compute the number of bytes to process at a time
      cipher->encryptBlock(context, ivXOR, o);                                  //Compute O(j) = CIPH(I(j))
      for(i = 0u; i < n; i++)                                                   //Compute C(j) = P(j) XOR MSB(O(j))
      {
         c[i] = p[i] ^ o[i];
      }
      memmove((void*)ivXOR,(void*)ivXOR + s,(int16_t)cipher->blockSize - s);    //Compute I(j+1) = LSB(I(j)) | O(j)
      memcpy((void*)ivXOR + cipher->blockSize - s,(void*) o,(int16_t) s);
      p += n;                                                                   //Next block
      c += n;
      length -= n;
   }
   return 1u;                                                                   //Successful encryption
}
/**
 * @brief OFB decryption
 * @param[in] cipher Cipher algorithm
 * @param[in] context Cipher algorithm context
 * @param[in] s Size of the plaintext and ciphertext segments
 * @param[in,out] iv Initialization vector
 * @param[in] c Ciphertext to be decrypted
 * @param[out] p Plaintext resulting from the decryption
 * @param[in] length Total number of data bytes to be decrypted
 * @return Error code
 **/
uint8_t ofbDecrypt(const CipherAlgo_t *cipher, void *context, uint8_t s, uint8_t *ivXOR, const uint8_t *c, uint8_t *p, size_t length)
{
   size_t i;
   size_t n;
   uint8_t o[16u];

   if ((((cipher==NULL) || (context==NULL)) || ((p==NULL) || (c==NULL))) || (ivXOR==NULL)) // null pointer passed then return
     return 0u;
     
   if((s % 8u) != 0u)                                                           // The parameter must be a multiple of 8
      return 0u;
   s = s / 8u;                                                                  // Determine the size, in bytes, of the plaintext and ciphertext segments
   if(s < 1u || s > cipher->blockSize)                                          // Check the resulting value
      return 0u;
   while(length > 0u)                                                           //Process each ciphertext segment
   {
      n = min(length, s);                                                       //Compute the number of bytes to process at a time
      cipher->encryptBlock(context, ivXOR, o);                                  //Compute O(j) = CIPH(I(j))
      for(i = 0u; i < n; i++)                                                   //Compute P(j) = C(j) XOR MSB(O(j))
      {
         p[i] = c[i] ^ o[i];
      }
      memmove((void*) ivXOR,(void*) ivXOR + s,(int16_t)cipher->blockSize - s);  //Compute I(j+1) = LSB(I(j)) | O(j)
      memcpy((void*) ivXOR + cipher->blockSize - s,(void*) o,(int16_t) s);
      c += n;                                                                   //Next block
      p += n;
      length -= n;
   }
   return 1u;                                                                   //Successful encryption
}

/**
 * @brief Initialize XTS context
 * @param[in] context Pointer to the XTS context
 * @param[in] cipherAlgo Cipher algorithm
 * @param[in] key Pointer to the key
 * @param[in] keyLen Length of the key
 * @return Error code
 **/
uint8_t xtsInit(XtsContext_t *context, const CipherAlgo_t *cipherAlgo, const void *key, size_t keyLen)
{
   uint8_t error;
   const uint8_t *k1;
   const uint8_t *k2;

   if (((context==NULL) || (cipherAlgo==NULL)) || (key==NULL))                  // null pointer passed then return
      return 0u;
      
   if(cipherAlgo->type != CIPHER_ALGO_TYPE_BLOCK || cipherAlgo->blockSize != 16u)    //XTS supports only symmetric block ciphers whose block size is 128 bits
      return 0u;

   if(keyLen != 32u && keyLen != 64u)                                           //Invalid key length?
      return 0u;
   context->cipherAlgo = cipherAlgo;                                            //Cipher algorithm used to perform XTS encryption/decryption
   k1 = (uint8_t *) key;                                                        //The key is parsed as a concatenation of 2 fields of equal size called K1 and K2
   k2 = (uint8_t *) key + (keyLen / 2u);
   error = cipherAlgo->init(context->cipherContext1, k1, keyLen / 2u);          //Initialize first cipher context using K1

   if(error)                                                                    //Any error to report?
      return error;
   error = cipherAlgo->init(context->cipherContext2, k2, keyLen / 2u);          //Initialize second cipher context using K2
   if(error)                                                                    //Any error to report?
      return error;
   return 1u;                                                                   //Successful initialization
}
/**
 * @brief Multiplication by x in GF(2^128)
 * @param[out] x Pointer to the output block
 * @param[out] a Pointer to the input block
 **/
void xtsMul(uint8_t *x, const uint8_t *a)
{
   size_t i;
   uint8_t c;

   if ((x==NULL) || (a==NULL))                                                  // null pointer passed the return
      return;

   c = a[15u] >> 7u;                                                            //Save the value of the most significant bit
   for(i = 15u; i > 0u; i--)                                                    //The multiplication of a polynomial by x in GF(2^128) corresponds to a shift of indices
   {
      x[i] = (a[i] << 1u) | (a[i - 1u] >> 7u);
   }
   x[0u] = a[0u] << 1u;                                                         //Shift the first byte of the block
   x[0u] ^= 0x87u & ~(c - 1u);                                                  //If the highest term of the result is equal to one, then perform reduction
}
/**
 * @brief XOR operation
 * @param[out] x Block resulting from the XOR operation
 * @param[in] a First input block
 * @param[in] b Second input block
 **/
void xtsXorBlock(uint8_t *x, const uint8_t *a, const uint8_t *b)
{
   size_t i;

   if (((x==NULL) || (a==NULL)) || (b==NULL))
     return;
   for(i = 0u; i < 16u; i++)                                                    // Perform XOR operation
   {
      x[i] = a[i] ^ b[i];
   }
}
/**
 * @brief Encrypt a data unit using XTS
 * @param[in] context Pointer to the XTS context
 * @param[in] i Value of the 128-bit tweak
 * @param[in] p Pointer to the data unit to be encrypted (plaintext)
 * @param[out] c Pointer to the resulting data unit (ciphertext)
 * @param[in] length Length of the data unit, in bytes
 * @return Error code
 **/
uint8_t xtsEncrypt(XtsContext_t *context, const uint8_t *i, const uint8_t *p, uint8_t *c, size_t length)
{
   uint8_t t[16u];
   uint8_t x[16u];

   if ((((context==NULL) || (i==NULL)) || ((p==NULL) || (c==NULL)))  || (length < 16u))   //The data unit size shall be at least 128 bits
      return 0u;
   context->cipherAlgo->encryptBlock(context->cipherContext2, i, t);            // Encrypt the tweak using K2
   while(length >= 16u)                                                         // XTS mode operates in a block-by-block fashion
   {
      xtsXorBlock(x, p, t);                                                     // Merge the tweak into the input block
      context->cipherAlgo->encryptBlock(context->cipherContext1, x, x);         // Encrypt the block using K1
      xtsXorBlock(c, x, t);                                                     // Merge the tweak into the output block
      xtsMul(t, t);                                                             // Multiply T by x in GF(2^128)
      p += 16u;                                                                 // Next block
      c += 16u;
      length -= 16u;
   }
   if(length > 0u)                                                              // Any partial block?
   {
      memcpy((void*)c,(void*) c - 16u,(int16_t) length);                        // Copy the final ciphertext bytes
      memcpy((void*)x,(void*) p,(int16_t) length);                              // Copy the final plaintext bytes
      memcpy((void*)x + length,(void*) c + length - 16u,(int16_t) 16u - length); // Steal ciphertext to complete the block
      xtsXorBlock(x, x, t);                                                     // Merge the tweak into the input block
      context->cipherAlgo->encryptBlock(context->cipherContext1, x, x);         // Encrypt the final block using K1
      xtsXorBlock(c - 16u, x, t);                                               // Merge the tweak into the output block
   }
   return 1u;                                                                   // Successful processing
}


/**
 * @brief Decrypt a data unit using XTS
 * @param[in] context Pointer to the XTS context
 * @param[in] i Value of the 128-bit tweak
 * @param[in] c Pointer to the data unit to be decrypted (ciphertext)
 * @param[out] p Pointer to the resulting data unit (plaintext)
 * @param[in] length Length of the data unit, in bytes
 * @return Error code
 **/
uint8_t xtsDecrypt(XtsContext_t *context, const uint8_t *i, const uint8_t *c, uint8_t *p, size_t length)
{
   uint8_t t[16u];
   uint8_t x[16u];
   uint8_t tt[16u];
  
   if ((((context==NULL) || (i==NULL)) || ((p==NULL) || (c==NULL)))  || (length < 16u))   //The data unit size shall be at least 128 bits
      return 0u;

   context->cipherAlgo->encryptBlock(context->cipherContext2, i, t);            // Encrypt the tweak using K2
   while(length >= 32u)                                                         //XTS mode operates in a block-by-block fashion
   {
      xtsXorBlock(x, c, t);                                                     //Merge the tweak into the input block
      context->cipherAlgo->decryptBlock(context->cipherContext1, x, x);         //Decrypt the block using K1
      xtsXorBlock(p, x, t);                                                     //Merge the tweak into the output block
      xtsMul(t, t);                                                             //Multiply T by x in GF(2^128)
      c += 16u;                                                                 //Next block
      p += 16u;
      length -= 16u;
   }
   if(length > 16u)                                                             //Any partial block?
   {
      xtsMul(tt, t);                                                            //Multiply T by x in GF(2^128)
      xtsXorBlock(x, c, tt);                                                    //Merge the tweak into the input block
      context->cipherAlgo->decryptBlock(context->cipherContext1, x, x);         //Decrypt the next-to-last block using K1
      xtsXorBlock(p, x, tt);                                                    //Merge the tweak into the output block
      length -= 16u;                                                            //Retrieve the length of the final block
      memcpy((void*)p + 16u,(void*) p,(int16_t) length);                        //Copy the final plaintext bytes
      memcpy((void*)x,(void*) c + 16u,(int16_t) length);                        //Copy the final ciphertext bytes
      memcpy((void*)x + length,(void*) p + length,(int16_t) 16u - length);      //Steal ciphertext to complete the block
   }
   else
   {
      memcpy((void*)x,(void*) c,(int16_t) 16u);                                 //The last block contains exactly 128 bits
   }
   xtsXorBlock(x, x, t);                                                        //Merge the tweak into the input block
   context->cipherAlgo->decryptBlock(context->cipherContext1, x, x);            //Decrypt the final block using K1
   xtsXorBlock(p, x, t);                                                        //Merge the tweak into the output block
   return 1u;                                                                   //Successful processing
}

#if defined(ALGO_CHA_CHA)
/**
 * @brief Initialize ChaCha context using the supplied key and nonce
 * @param[in] context Pointer to the ChaCha context to initialize
 * @param[in] nr Number of rounds to be applied (8, 12 or 20)
 * @param[in] key Pointer to the key
 * @param[in] keyLen Length of the key, in bytes (16 or 32)
 * @param[in] nonce Pointer to the nonce
 * @param[in] nonceLen Length of the nonce, in bytes (8 or 12)
 * @return Error code
 **/
uint8_t chachaInit(ChachaContext_t *context, uint8_t nr, const uint8_t *key, size_t keyLen, const uint8_t *nonce, size_t nonceLen)
{
   uint32_t *w;

   if(context == NULL || key == NULL || nonce == NULL)                          //Check parameters
      return 0u;

   if(nr != 8u && nr != 12u && nr != 20u)                                       //The number of rounds must be 8, 12 or 20
      return 0u;

   context->nr = nr;                                                            //Save the number of rounds to be applied

   w = context->state;                                                          //Point to the state

   if(keyLen == 16)                                                             //Check the length of the key
   {
      w[0u] = 0x61707865ul;                                                       //The first four input words are constants
      w[1u] = 0x3120646Eul;
      w[2u] = 0x79622D36ul;
      w[3u] = 0x6B206574ul;

      //Input words 4 through 7 are taken from the 128-bit key, by reading
      //the bytes in little-endian order, in 4-byte chunks
      w[4u] = LOAD32LE(key);
      w[5u] = LOAD32LE(key + 4);
      w[6u] = LOAD32LE(key + 8);
      w[7u] = LOAD32LE(key + 12);

      //Input words 8 through 11 are taken from the 128-bit key, again by
      //reading the bytes in little-endian order, in 4-byte chunks
      w[8u] = LOAD32LE(key);
      w[9u] = LOAD32LE(key + 4u);
      w[10u] = LOAD32LE(key + 8u);
      w[11u] = LOAD32LE(key + 12u);
   }
   else if(keyLen == 32u)
   {
      w[0u] = 0x61707865ul;                                                     //The first four input words are constants
      w[1u] = 0x3320646Eul;
      w[2u] = 0x79622D32ul;
      w[3u] = 0x6B206574ul;

      //Input words 4 through 11 are taken from the 256-bit key, by reading
      //the bytes in little-endian order, in 4-byte chunks
      w[4u] = LOAD32LE(key);
      w[5u] = LOAD32LE(key + 4u);
      w[6u] = LOAD32LE(key + 8u);
      w[7u] = LOAD32LE(key + 12u);
      w[8u] = LOAD32LE(key + 16u);
      w[9u] = LOAD32LE(key + 20u);
      w[10u] = LOAD32LE(key + 24u);
      w[11u] = LOAD32LE(key + 28u);
   }
   else
   {
      return 0u;                                                                //Invalid key length
   }

   if(nonceLen == 8u)                                                           //Check the length of the nonce
   {
      //Input words 12 and 13 are a block counter, with word 12
      //overflowing into word 13
      w[12u] = 0;
      w[13u] = 0;

      //Input words 14 and 15 are taken from an 64-bit nonce, by reading
      //the bytes in little-endian order, in 4-byte chunks
      w[14u] = LOAD32LE(nonce);
      w[15u] = LOAD32LE(nonce + 4u);
   }
   else if(nonceLen == 12u)
   {
      w[12u] = 0u;                                                              //Input word 12 is a block counter
      //Input words 13 to 15 are taken from an 96-bit nonce, by reading
      //the bytes in little-endian order, in 4-byte chunks
      w[13u] = LOAD32LE(nonce);
      w[14u] = LOAD32LE(nonce + 4u);
      w[15u] = LOAD32LE(nonce + 8u);
   }
   else
   {
      return 0u;                                                                //Invalid nonce length
   }
   context->pos = 0u;                                                           //The keystream block is empty
   return 1u;                                                                   //No error to report
}
/**
 * @brief Encrypt/decrypt data with the ChaCha algorithm
 * @param[in] context Pointer to the ChaCha context
 * @param[in] input Pointer to the data to encrypt/decrypt (optional)
 * @param[in] output Pointer to the resulting data (optional)
 * @param[in] length Number of bytes to be processed
 **/
void chachaCipher(ChachaContext_t *context, const uint8_t *input, uint8_t *output, size_t length)
{
   uint8_t i;
   uint8_t n;
   uint8_t *k;

   //Encryption loop
   while(length > 0)
   {
      //Check whether a new keystream block must be generated
      if(context->pos == 0 || context->pos >= 64)
      {
         //ChaCha successively calls the ChaCha block function, with the same key
         //and nonce, and with successively increasing block counter parameters
         chachaProcessBlock(context);

         //Increment block counter
         context->state[12]++;

         //Propagate the carry if necessary
         if(context->state[12] == 0)
         {
            context->state[13]++;
         }

         //Rewind to the beginning of the keystream block
         context->pos = 0;
      }

      //Compute the number of bytes to encrypt/decrypt at a time
      n = min(length, 64 - context->pos);

      //Valid output pointer?
      if(output != NULL)
      {
         //Point to the keystream
         k = (uint8_t *) context->block + context->pos;

         //Valid input pointer?
         if(input != NULL)
         {
            //XOR the input data with the keystream
            for(i = 0; i < n; i++)
            {
               output[i] = input[i] ^ k[i];
            }

            //Advance input pointer
            input += n;
         }
         else
         {
            //Output the keystream
            for(i = 0; i < n; i++)
            {
               output[i] = k[i];
            }
         }

         //Advance output pointer
         output += n;
      }

      //Current position in the keystream block
      context->pos += n;
      //Remaining bytes to process
      length -= n;
   }
}
/**
 * @brief Generate a keystream block
 * @param[in] context Pointer to the ChaCha context
 **/
void chachaProcessBlock(ChachaContext_t *context)
{
   uint8_t i;
   uint32_t *w = NULL;

   if (context==NULL) return;
   
   //Point to the working state
   w = (uint32_t *) context->block;

   //Copy the state to the working state
   for(i = 0; i < 16; i++)
   {
      w[i] = context->state[i];
   }

   //ChaCha runs 8, 12 or 20 rounds, alternating between column rounds and
   //diagonal rounds
   for(i = 0; i < context->nr; i += 2)
   {
      //The column rounds apply the quarter-round function to the four
      //columns, from left to right
      CHACHA_QUARTER_ROUND(w[0], w[4], w[8], w[12]);
      CHACHA_QUARTER_ROUND(w[1], w[5], w[9], w[13]);
      CHACHA_QUARTER_ROUND(w[2], w[6], w[10], w[14]);
      CHACHA_QUARTER_ROUND(w[3], w[7], w[11], w[15]);

      //The diagonal rounds apply the quarter-round function to the top-left,
      //bottom-right diagonal, followed by the pattern shifted one place to
      //the right, for three more quarter-rounds
      CHACHA_QUARTER_ROUND(w[0], w[5], w[10], w[15]);
      CHACHA_QUARTER_ROUND(w[1], w[6], w[11], w[12]);
      CHACHA_QUARTER_ROUND(w[2], w[7], w[8], w[13]);
      CHACHA_QUARTER_ROUND(w[3], w[4], w[9], w[14]);
   }

   //Add the original input words to the output words
   for(i = 0; i < 16; i++)
   {
      w[i] += context->state[i];
   }

   //Serialize the result by sequencing the words one-by-one in little-endian
   //order
   for(i = 0; i < 16; i++)
   {
      w[i] = htole32(w[i]);
   }
}

#elif defined(ALGO_SEED)
//Key schedule constants
static const uint32_t kc[16] =
{
   0x9E3779B9, 0x3C6EF373, 0x78DDE6E6, 0xF1BBCDCC,
   0xE3779B99, 0xC6EF3733, 0x8DDE6E67, 0x1BBCDCCF,
   0x3779B99E, 0x6EF3733C, 0xDDE6E678, 0xBBCDCCF1,
   0x779B99E3, 0xEF3733C6, 0xDE6E678D, 0xBCDCCF1B
};
//S-Box SS0
static const uint32_t ss00[256] =
{
   0x2989A1A8, 0x05858184, 0x16C6D2D4, 0x13C3D3D0, 0x14445054, 0x1D0D111C, 0x2C8CA0AC, 0x25052124,
   0x1D4D515C, 0x03434340, 0x18081018, 0x1E0E121C, 0x11415150, 0x3CCCF0FC, 0x0ACAC2C8, 0x23436360,
   0x28082028, 0x04444044, 0x20002020, 0x1D8D919C, 0x20C0E0E0, 0x22C2E2E0, 0x08C8C0C8, 0x17071314,
   0x2585A1A4, 0x0F8F838C, 0x03030300, 0x3B4B7378, 0x3B8BB3B8, 0x13031310, 0x12C2D2D0, 0x2ECEE2EC,
   0x30407070, 0x0C8C808C, 0x3F0F333C, 0x2888A0A8, 0x32023230, 0x1DCDD1DC, 0x36C6F2F4, 0x34447074,
   0x2CCCE0EC, 0x15859194, 0x0B0B0308, 0x17475354, 0x1C4C505C, 0x1B4B5358, 0x3D8DB1BC, 0x01010100,
   0x24042024, 0x1C0C101C, 0x33437370, 0x18889098, 0x10001010, 0x0CCCC0CC, 0x32C2F2F0, 0x19C9D1D8,
   0x2C0C202C, 0x27C7E3E4, 0x32427270, 0x03838380, 0x1B8B9398, 0x11C1D1D0, 0x06868284, 0x09C9C1C8,
   0x20406060, 0x10405050, 0x2383A3A0, 0x2BCBE3E8, 0x0D0D010C, 0x3686B2B4, 0x1E8E929C, 0x0F4F434C,
   0x3787B3B4, 0x1A4A5258, 0x06C6C2C4, 0x38487078, 0x2686A2A4, 0x12021210, 0x2F8FA3AC, 0x15C5D1D4,
   0x21416160, 0x03C3C3C0, 0x3484B0B4, 0x01414140, 0x12425250, 0x3D4D717C, 0x0D8D818C, 0x08080008,
   0x1F0F131C, 0x19899198, 0x00000000, 0x19091118, 0x04040004, 0x13435350, 0x37C7F3F4, 0x21C1E1E0,
   0x3DCDF1FC, 0x36467274, 0x2F0F232C, 0x27072324, 0x3080B0B0, 0x0B8B8388, 0x0E0E020C, 0x2B8BA3A8,
   0x2282A2A0, 0x2E4E626C, 0x13839390, 0x0D4D414C, 0x29496168, 0x3C4C707C, 0x09090108, 0x0A0A0208,
   0x3F8FB3BC, 0x2FCFE3EC, 0x33C3F3F0, 0x05C5C1C4, 0x07878384, 0x14041014, 0x3ECEF2FC, 0x24446064,
   0x1ECED2DC, 0x2E0E222C, 0x0B4B4348, 0x1A0A1218, 0x06060204, 0x21012120, 0x2B4B6368, 0x26466264,
   0x02020200, 0x35C5F1F4, 0x12829290, 0x0A8A8288, 0x0C0C000C, 0x3383B3B0, 0x3E4E727C, 0x10C0D0D0,
   0x3A4A7278, 0x07474344, 0x16869294, 0x25C5E1E4, 0x26062224, 0x00808080, 0x2D8DA1AC, 0x1FCFD3DC,
   0x2181A1A0, 0x30003030, 0x37073334, 0x2E8EA2AC, 0x36063234, 0x15051114, 0x22022220, 0x38083038,
   0x34C4F0F4, 0x2787A3A4, 0x05454144, 0x0C4C404C, 0x01818180, 0x29C9E1E8, 0x04848084, 0x17879394,
   0x35053134, 0x0BCBC3C8, 0x0ECEC2CC, 0x3C0C303C, 0x31417170, 0x11011110, 0x07C7C3C4, 0x09898188,
   0x35457174, 0x3BCBF3F8, 0x1ACAD2D8, 0x38C8F0F8, 0x14849094, 0x19495158, 0x02828280, 0x04C4C0C4,
   0x3FCFF3FC, 0x09494148, 0x39093138, 0x27476364, 0x00C0C0C0, 0x0FCFC3CC, 0x17C7D3D4, 0x3888B0B8,
   0x0F0F030C, 0x0E8E828C, 0x02424240, 0x23032320, 0x11819190, 0x2C4C606C, 0x1BCBD3D8, 0x2484A0A4,
   0x34043034, 0x31C1F1F0, 0x08484048, 0x02C2C2C0, 0x2F4F636C, 0x3D0D313C, 0x2D0D212C, 0x00404040,
   0x3E8EB2BC, 0x3E0E323C, 0x3C8CB0BC, 0x01C1C1C0, 0x2A8AA2A8, 0x3A8AB2B8, 0x0E4E424C, 0x15455154,
   0x3B0B3338, 0x1CCCD0DC, 0x28486068, 0x3F4F737C, 0x1C8C909C, 0x18C8D0D8, 0x0A4A4248, 0x16465254,
   0x37477374, 0x2080A0A0, 0x2DCDE1EC, 0x06464244, 0x3585B1B4, 0x2B0B2328, 0x25456164, 0x3ACAF2F8,
   0x23C3E3E0, 0x3989B1B8, 0x3181B1B0, 0x1F8F939C, 0x1E4E525C, 0x39C9F1F8, 0x26C6E2E4, 0x3282B2B0,
   0x31013130, 0x2ACAE2E8, 0x2D4D616C, 0x1F4F535C, 0x24C4E0E4, 0x30C0F0F0, 0x0DCDC1CC, 0x08888088,
   0x16061214, 0x3A0A3238, 0x18485058, 0x14C4D0D4, 0x22426260, 0x29092128, 0x07070304, 0x33033330,
   0x28C8E0E8, 0x1B0B1318, 0x05050104, 0x39497178, 0x10809090, 0x2A4A6268, 0x2A0A2228, 0x1A8A9298
};
//S-Box SS1
static const uint32_t ss1[256] =
{
   0x38380830, 0xE828C8E0, 0x2C2D0D21, 0xA42686A2, 0xCC0FCFC3, 0xDC1ECED2, 0xB03383B3, 0xB83888B0,
   0xAC2F8FA3, 0x60204060, 0x54154551, 0xC407C7C3, 0x44044440, 0x6C2F4F63, 0x682B4B63, 0x581B4B53,
   0xC003C3C3, 0x60224262, 0x30330333, 0xB43585B1, 0x28290921, 0xA02080A0, 0xE022C2E2, 0xA42787A3,
   0xD013C3D3, 0x90118191, 0x10110111, 0x04060602, 0x1C1C0C10, 0xBC3C8CB0, 0x34360632, 0x480B4B43,
   0xEC2FCFE3, 0x88088880, 0x6C2C4C60, 0xA82888A0, 0x14170713, 0xC404C4C0, 0x14160612, 0xF434C4F0,
   0xC002C2C2, 0x44054541, 0xE021C1E1, 0xD416C6D2, 0x3C3F0F33, 0x3C3D0D31, 0x8C0E8E82, 0x98188890,
   0x28280820, 0x4C0E4E42, 0xF436C6F2, 0x3C3E0E32, 0xA42585A1, 0xF839C9F1, 0x0C0D0D01, 0xDC1FCFD3,
   0xD818C8D0, 0x282B0B23, 0x64264662, 0x783A4A72, 0x24270723, 0x2C2F0F23, 0xF031C1F1, 0x70324272,
   0x40024242, 0xD414C4D0, 0x40014141, 0xC000C0C0, 0x70334373, 0x64274763, 0xAC2C8CA0, 0x880B8B83,
   0xF437C7F3, 0xAC2D8DA1, 0x80008080, 0x1C1F0F13, 0xC80ACAC2, 0x2C2C0C20, 0xA82A8AA2, 0x34340430,
   0xD012C2D2, 0x080B0B03, 0xEC2ECEE2, 0xE829C9E1, 0x5C1D4D51, 0x94148490, 0x18180810, 0xF838C8F0,
   0x54174753, 0xAC2E8EA2, 0x08080800, 0xC405C5C1, 0x10130313, 0xCC0DCDC1, 0x84068682, 0xB83989B1,
   0xFC3FCFF3, 0x7C3D4D71, 0xC001C1C1, 0x30310131, 0xF435C5F1, 0x880A8A82, 0x682A4A62, 0xB03181B1,
   0xD011C1D1, 0x20200020, 0xD417C7D3, 0x00020202, 0x20220222, 0x04040400, 0x68284860, 0x70314171,
   0x04070703, 0xD81BCBD3, 0x9C1D8D91, 0x98198991, 0x60214161, 0xBC3E8EB2, 0xE426C6E2, 0x58194951,
   0xDC1DCDD1, 0x50114151, 0x90108090, 0xDC1CCCD0, 0x981A8A92, 0xA02383A3, 0xA82B8BA3, 0xD010C0D0,
   0x80018181, 0x0C0F0F03, 0x44074743, 0x181A0A12, 0xE023C3E3, 0xEC2CCCE0, 0x8C0D8D81, 0xBC3F8FB3,
   0x94168692, 0x783B4B73, 0x5C1C4C50, 0xA02282A2, 0xA02181A1, 0x60234363, 0x20230323, 0x4C0D4D41,
   0xC808C8C0, 0x9C1E8E92, 0x9C1C8C90, 0x383A0A32, 0x0C0C0C00, 0x2C2E0E22, 0xB83A8AB2, 0x6C2E4E62,
   0x9C1F8F93, 0x581A4A52, 0xF032C2F2, 0x90128292, 0xF033C3F3, 0x48094941, 0x78384870, 0xCC0CCCC0,
   0x14150511, 0xF83BCBF3, 0x70304070, 0x74354571, 0x7C3F4F73, 0x34350531, 0x10100010, 0x00030303,
   0x64244460, 0x6C2D4D61, 0xC406C6C2, 0x74344470, 0xD415C5D1, 0xB43484B0, 0xE82ACAE2, 0x08090901,
   0x74364672, 0x18190911, 0xFC3ECEF2, 0x40004040, 0x10120212, 0xE020C0E0, 0xBC3D8DB1, 0x04050501,
   0xF83ACAF2, 0x00010101, 0xF030C0F0, 0x282A0A22, 0x5C1E4E52, 0xA82989A1, 0x54164652, 0x40034343,
   0x84058581, 0x14140410, 0x88098981, 0x981B8B93, 0xB03080B0, 0xE425C5E1, 0x48084840, 0x78394971,
   0x94178793, 0xFC3CCCF0, 0x1C1E0E12, 0x80028282, 0x20210121, 0x8C0C8C80, 0x181B0B13, 0x5C1F4F53,
   0x74374773, 0x54144450, 0xB03282B2, 0x1C1D0D11, 0x24250521, 0x4C0F4F43, 0x00000000, 0x44064642,
   0xEC2DCDE1, 0x58184850, 0x50124252, 0xE82BCBE3, 0x7C3E4E72, 0xD81ACAD2, 0xC809C9C1, 0xFC3DCDF1,
   0x30300030, 0x94158591, 0x64254561, 0x3C3C0C30, 0xB43686B2, 0xE424C4E0, 0xB83B8BB3, 0x7C3C4C70,
   0x0C0E0E02, 0x50104050, 0x38390931, 0x24260622, 0x30320232, 0x84048480, 0x68294961, 0x90138393,
   0x34370733, 0xE427C7E3, 0x24240420, 0xA42484A0, 0xC80BCBC3, 0x50134353, 0x080A0A02, 0x84078783,
   0xD819C9D1, 0x4C0C4C40, 0x80038383, 0x8C0F8F83, 0xCC0ECEC2, 0x383B0B33, 0x480A4A42, 0xB43787B3
};
//S-Box SS2
static const uint32_t ss2[256] =
{
   0xA1A82989, 0x81840585, 0xD2D416C6, 0xD3D013C3, 0x50541444, 0x111C1D0D, 0xA0AC2C8C, 0x21242505,
   0x515C1D4D, 0x43400343, 0x10181808, 0x121C1E0E, 0x51501141, 0xF0FC3CCC, 0xC2C80ACA, 0x63602343,
   0x20282808, 0x40440444, 0x20202000, 0x919C1D8D, 0xE0E020C0, 0xE2E022C2, 0xC0C808C8, 0x13141707,
   0xA1A42585, 0x838C0F8F, 0x03000303, 0x73783B4B, 0xB3B83B8B, 0x13101303, 0xD2D012C2, 0xE2EC2ECE,
   0x70703040, 0x808C0C8C, 0x333C3F0F, 0xA0A82888, 0x32303202, 0xD1DC1DCD, 0xF2F436C6, 0x70743444,
   0xE0EC2CCC, 0x91941585, 0x03080B0B, 0x53541747, 0x505C1C4C, 0x53581B4B, 0xB1BC3D8D, 0x01000101,
   0x20242404, 0x101C1C0C, 0x73703343, 0x90981888, 0x10101000, 0xC0CC0CCC, 0xF2F032C2, 0xD1D819C9,
   0x202C2C0C, 0xE3E427C7, 0x72703242, 0x83800383, 0x93981B8B, 0xD1D011C1, 0x82840686, 0xC1C809C9,
   0x60602040, 0x50501040, 0xA3A02383, 0xE3E82BCB, 0x010C0D0D, 0xB2B43686, 0x929C1E8E, 0x434C0F4F,
   0xB3B43787, 0x52581A4A, 0xC2C406C6, 0x70783848, 0xA2A42686, 0x12101202, 0xA3AC2F8F, 0xD1D415C5,
   0x61602141, 0xC3C003C3, 0xB0B43484, 0x41400141, 0x52501242, 0x717C3D4D, 0x818C0D8D, 0x00080808,
   0x131C1F0F, 0x91981989, 0x00000000, 0x11181909, 0x00040404, 0x53501343, 0xF3F437C7, 0xE1E021C1,
   0xF1FC3DCD, 0x72743646, 0x232C2F0F, 0x23242707, 0xB0B03080, 0x83880B8B, 0x020C0E0E, 0xA3A82B8B,
   0xA2A02282, 0x626C2E4E, 0x93901383, 0x414C0D4D, 0x61682949, 0x707C3C4C, 0x01080909, 0x02080A0A,
   0xB3BC3F8F, 0xE3EC2FCF, 0xF3F033C3, 0xC1C405C5, 0x83840787, 0x10141404, 0xF2FC3ECE, 0x60642444,
   0xD2DC1ECE, 0x222C2E0E, 0x43480B4B, 0x12181A0A, 0x02040606, 0x21202101, 0x63682B4B, 0x62642646,
   0x02000202, 0xF1F435C5, 0x92901282, 0x82880A8A, 0x000C0C0C, 0xB3B03383, 0x727C3E4E, 0xD0D010C0,
   0x72783A4A, 0x43440747, 0x92941686, 0xE1E425C5, 0x22242606, 0x80800080, 0xA1AC2D8D, 0xD3DC1FCF,
   0xA1A02181, 0x30303000, 0x33343707, 0xA2AC2E8E, 0x32343606, 0x11141505, 0x22202202, 0x30383808,
   0xF0F434C4, 0xA3A42787, 0x41440545, 0x404C0C4C, 0x81800181, 0xE1E829C9, 0x80840484, 0x93941787,
   0x31343505, 0xC3C80BCB, 0xC2CC0ECE, 0x303C3C0C, 0x71703141, 0x11101101, 0xC3C407C7, 0x81880989,
   0x71743545, 0xF3F83BCB, 0xD2D81ACA, 0xF0F838C8, 0x90941484, 0x51581949, 0x82800282, 0xC0C404C4,
   0xF3FC3FCF, 0x41480949, 0x31383909, 0x63642747, 0xC0C000C0, 0xC3CC0FCF, 0xD3D417C7, 0xB0B83888,
   0x030C0F0F, 0x828C0E8E, 0x42400242, 0x23202303, 0x91901181, 0x606C2C4C, 0xD3D81BCB, 0xA0A42484,
   0x30343404, 0xF1F031C1, 0x40480848, 0xC2C002C2, 0x636C2F4F, 0x313C3D0D, 0x212C2D0D, 0x40400040,
   0xB2BC3E8E, 0x323C3E0E, 0xB0BC3C8C, 0xC1C001C1, 0xA2A82A8A, 0xB2B83A8A, 0x424C0E4E, 0x51541545,
   0x33383B0B, 0xD0DC1CCC, 0x60682848, 0x737C3F4F, 0x909C1C8C, 0xD0D818C8, 0x42480A4A, 0x52541646,
   0x73743747, 0xA0A02080, 0xE1EC2DCD, 0x42440646, 0xB1B43585, 0x23282B0B, 0x61642545, 0xF2F83ACA,
   0xE3E023C3, 0xB1B83989, 0xB1B03181, 0x939C1F8F, 0x525C1E4E, 0xF1F839C9, 0xE2E426C6, 0xB2B03282,
   0x31303101, 0xE2E82ACA, 0x616C2D4D, 0x535C1F4F, 0xE0E424C4, 0xF0F030C0, 0xC1CC0DCD, 0x80880888,
   0x12141606, 0x32383A0A, 0x50581848, 0xD0D414C4, 0x62602242, 0x21282909, 0x03040707, 0x33303303,
   0xE0E828C8, 0x13181B0B, 0x01040505, 0x71783949, 0x90901080, 0x62682A4A, 0x22282A0A, 0x92981A8A
};
//S-Box SS3
static const uint32_t ss3[256] =
{
   0x08303838, 0xC8E0E828, 0x0D212C2D, 0x86A2A426, 0xCFC3CC0F, 0xCED2DC1E, 0x83B3B033, 0x88B0B838,
   0x8FA3AC2F, 0x40606020, 0x45515415, 0xC7C3C407, 0x44404404, 0x4F636C2F, 0x4B63682B, 0x4B53581B,
   0xC3C3C003, 0x42626022, 0x03333033, 0x85B1B435, 0x09212829, 0x80A0A020, 0xC2E2E022, 0x87A3A427,
   0xC3D3D013, 0x81919011, 0x01111011, 0x06020406, 0x0C101C1C, 0x8CB0BC3C, 0x06323436, 0x4B43480B,
   0xCFE3EC2F, 0x88808808, 0x4C606C2C, 0x88A0A828, 0x07131417, 0xC4C0C404, 0x06121416, 0xC4F0F434,
   0xC2C2C002, 0x45414405, 0xC1E1E021, 0xC6D2D416, 0x0F333C3F, 0x0D313C3D, 0x8E828C0E, 0x88909818,
   0x08202828, 0x4E424C0E, 0xC6F2F436, 0x0E323C3E, 0x85A1A425, 0xC9F1F839, 0x0D010C0D, 0xCFD3DC1F,
   0xC8D0D818, 0x0B23282B, 0x46626426, 0x4A72783A, 0x07232427, 0x0F232C2F, 0xC1F1F031, 0x42727032,
   0x42424002, 0xC4D0D414, 0x41414001, 0xC0C0C000, 0x43737033, 0x47636427, 0x8CA0AC2C, 0x8B83880B,
   0xC7F3F437, 0x8DA1AC2D, 0x80808000, 0x0F131C1F, 0xCAC2C80A, 0x0C202C2C, 0x8AA2A82A, 0x04303434,
   0xC2D2D012, 0x0B03080B, 0xCEE2EC2E, 0xC9E1E829, 0x4D515C1D, 0x84909414, 0x08101818, 0xC8F0F838,
   0x47535417, 0x8EA2AC2E, 0x08000808, 0xC5C1C405, 0x03131013, 0xCDC1CC0D, 0x86828406, 0x89B1B839,
   0xCFF3FC3F, 0x4D717C3D, 0xC1C1C001, 0x01313031, 0xC5F1F435, 0x8A82880A, 0x4A62682A, 0x81B1B031,
   0xC1D1D011, 0x00202020, 0xC7D3D417, 0x02020002, 0x02222022, 0x04000404, 0x48606828, 0x41717031,
   0x07030407, 0xCBD3D81B, 0x8D919C1D, 0x89919819, 0x41616021, 0x8EB2BC3E, 0xC6E2E426, 0x49515819,
   0xCDD1DC1D, 0x41515011, 0x80909010, 0xCCD0DC1C, 0x8A92981A, 0x83A3A023, 0x8BA3A82B, 0xC0D0D010,
   0x81818001, 0x0F030C0F, 0x47434407, 0x0A12181A, 0xC3E3E023, 0xCCE0EC2C, 0x8D818C0D, 0x8FB3BC3F,
   0x86929416, 0x4B73783B, 0x4C505C1C, 0x82A2A022, 0x81A1A021, 0x43636023, 0x03232023, 0x4D414C0D,
   0xC8C0C808, 0x8E929C1E, 0x8C909C1C, 0x0A32383A, 0x0C000C0C, 0x0E222C2E, 0x8AB2B83A, 0x4E626C2E,
   0x8F939C1F, 0x4A52581A, 0xC2F2F032, 0x82929012, 0xC3F3F033, 0x49414809, 0x48707838, 0xCCC0CC0C,
   0x05111415, 0xCBF3F83B, 0x40707030, 0x45717435, 0x4F737C3F, 0x05313435, 0x00101010, 0x03030003,
   0x44606424, 0x4D616C2D, 0xC6C2C406, 0x44707434, 0xC5D1D415, 0x84B0B434, 0xCAE2E82A, 0x09010809,
   0x46727436, 0x09111819, 0xCEF2FC3E, 0x40404000, 0x02121012, 0xC0E0E020, 0x8DB1BC3D, 0x05010405,
   0xCAF2F83A, 0x01010001, 0xC0F0F030, 0x0A22282A, 0x4E525C1E, 0x89A1A829, 0x46525416, 0x43434003,
   0x85818405, 0x04101414, 0x89818809, 0x8B93981B, 0x80B0B030, 0xC5E1E425, 0x48404808, 0x49717839,
   0x87939417, 0xCCF0FC3C, 0x0E121C1E, 0x82828002, 0x01212021, 0x8C808C0C, 0x0B13181B, 0x4F535C1F,
   0x47737437, 0x44505414, 0x82B2B032, 0x0D111C1D, 0x05212425, 0x4F434C0F, 0x00000000, 0x46424406,
   0xCDE1EC2D, 0x48505818, 0x42525012, 0xCBE3E82B, 0x4E727C3E, 0xCAD2D81A, 0xC9C1C809, 0xCDF1FC3D,
   0x00303030, 0x85919415, 0x45616425, 0x0C303C3C, 0x86B2B436, 0xC4E0E424, 0x8BB3B83B, 0x4C707C3C,
   0x0E020C0E, 0x40505010, 0x09313839, 0x06222426, 0x02323032, 0x84808404, 0x49616829, 0x83939013,
   0x07333437, 0xC7E3E427, 0x04202424, 0x84A0A424, 0xCBC3C80B, 0x43535013, 0x0A02080A, 0x87838407,
   0xC9D1D819, 0x4C404C0C, 0x83838003, 0x8F838C0F, 0xCEC2CC0E, 0x0B33383B, 0x4A42480A, 0x87B3B437
};
//Common interface for encryption algorithms
const CipherAlgo_t seedCipherAlgo =
{
   "SEED",
   sizeof(SeedContext_t),
   CIPHER_ALGO_TYPE_BLOCK,
   SEED_BLOCK_SIZE,
   (CipherAlgoInit) seedInit,
   //NULL,
   //NULL,
   (CipherAlgoEncryptBlock) seedEncryptBlock,
   (CipherAlgoDecryptBlock) seedDecryptBlock
};
/**
 * @brief Initialize a SEED context using the supplied key
 * @param[in] context Pointer to the SEED context to initialize
 * @param[in] key Pointer to the key
 * @param[in] keyLen Length of the key
 * @return Error code
 **/
uint8_t seedInit(SeedContext_t *context, const uint8_t *key, size_t keyLen)
{
   uint8_t i;
   uint32_t t;
   uint32_t key0;
   uint32_t key1;
   uint32_t key2;
   uint32_t key3;

   //Check parameters
   if(context == NULL || key == NULL)
      return 0u;

   //Invalid key length?
   if(keyLen != 16)
      return 0u;

   //The 128-bit input key is divided into four 32-bit blocks
   key0 = LOAD32BE(key + 0);
   key1 = LOAD32BE(key + 4);
   key2 = LOAD32BE(key + 8);
   key3 = LOAD32BE(key + 12);

   //Apply 16 rounds
   for(i = 0; i < 16; i++)
   {
      t = key0 + key2 - kc[i];
      context->ks[2 * i] = G(t);
      t = key1 - key3 + kc[i];
      context->ks[2 * i + 1] = G(t);

      //Odd round?
      if((i % 2) != 0)
      {
         t = (key3 << 8) | (key2 >> 24);
         key2 = (key2 << 8) | (key3 >> 24);
         key3 = t;
      }
      //Even round?
      else
      {
         t = (key1 >> 8) | (key0 << 24);
         key0 = (key0 >> 8) | (key1 << 24);
         key1 = t;
      }
   }

   //No error to report
   return 1u;
}


/**
 * @brief Encrypt a 16-byte block using SEED algorithm
 * @param[in] context Pointer to the SEED context
 * @param[in] input Plaintext block to encrypt
 * @param[out] output Ciphertext block resulting from encryption
 **/
void seedEncryptBlock(SeedContext_t *context, const uint8_t *input, uint8_t *output)
{
   uint8_t i;
   uint32_t temp0;
   uint32_t temp1;
   uint32_t *ks;
   uint32_t left0, left1, right0, right1;
    
   if ((context == NULL) || ((input == NULL) || (output == NULL)))
      return;
      
   //The 128-bit input is divided into two 64-bit blocks (L and R)
   left0 = LOAD32BE(input + 0);
   left1 = LOAD32BE(input + 4);
   right0 = LOAD32BE(input + 8);
   right1 = LOAD32BE(input + 12);

   //The key schedule must be applied in ascending order
   ks = context->ks;

   //Perform 16 rounds
   for(i = 0; i < 16; i++)
   {
      //Apply function F
      F(ks[0u], ks[1u], right0, right1, temp0, temp1);

      temp0 ^= left0;
      temp1 ^= left1;
      left0 = right0;
      left1 = right1;
      right0 = temp0;
      right1 = temp1;

      //Advance current location in key schedule
      ks += 2;
   }

   //The resulting value is the ciphertext
   STORE32BE(right0, output + 0);
   STORE32BE(right1, output + 4);
   STORE32BE(left0, output + 8);
   STORE32BE(left1, output + 12);
}

/**
 * @brief Decrypt a 16-byte block using SEED algorithm
 * @param[in] context Pointer to the SEED context
 * @param[in] input Ciphertext block to decrypt
 * @param[out] output Plaintext block resulting from decryption
 **/
void seedDecryptBlock(SeedContext_t *context, const uint8_t *input, uint8_t *output)
{
   uint8_t i;
   uint32_t temp0;
   uint32_t temp1;
   uint32_t *ks;
   uint32_t left0, left1, right0, right1;
    
   if ((context == NULL) || ((input == NULL) || (output == NULL)))
      return;
      
   //The 128-bit input is divided into two 64-bit blocks (L and R)
   left0 = LOAD32BE(input + 0);
   left1 = LOAD32BE(input + 4);
   right0 = LOAD32BE(input + 8);
   right1 = LOAD32BE(input + 12);

   //The key schedule must be applied in reverse order
   ks = context->ks + 30;

   //Perform 16 rounds
   for(i = 0u; i < 16u; i++)
   {
      //Apply function F
      F(ks[0], ks[1], right0, right1, temp0, temp1);

      temp0 ^= left0;
      temp1 ^= left1;
      left0 = right0;
      left1 = right1;
      right0 = temp0;
      right1 = temp1;

      //Advance current location in key schedule
      ks -= 2;
   }

   //The resulting value is the plaintext
   STORE32BE(right0, output + 0);
   STORE32BE(right1, output + 4);
   STORE32BE(left0, output + 8);
   STORE32BE(left1, output + 12);
}
#elif defined(ALGO_CAMELLIA)
//Key schedule for 128-bit key
static const CamelliaSubkey_t ks1[] =
{
   {0,  KL, 0,  L},  //kw1
   {2,  KL, 0,  R},  //kw2
   {4,  KA, 0,  L},  //k1
   {6,  KA, 0,  R},  //k2
   {8,  KL, 15, L},  //k3
   {10, KL, 15, R},  //k4
   {12, KA, 15, L},  //k5
   {14, KA, 15, R},  //k6
   {16, KA, 30, L},  //ke1
   {18, KA, 30, R},  //ke2
   {20, KL, 45, L},  //k7
   {22, KL, 45, R},  //k8
   {24, KA, 45, L},  //k9
   {26, KL, 60, R},  //k10
   {28, KA, 60, L},  //k11
   {30, KA, 60, R},  //k12
   {32, KL, 77, L},  //ke3
   {34, KL, 77, R},  //ke4
   {36, KL, 94, L},  //k13
   {38, KL, 94, R},  //k14
   {40, KA, 94, L},  //k15
   {42, KA, 94, R},  //k16
   {44, KL, 111, L}, //k17
   {46, KL, 111, R}, //k18
   {48, KA, 111, L}, //kw3
   {50, KA, 111, R}, //kw4
};

//Key schedule for 192 and 256-bit keys
static const CamelliaSubkey_t ks2[] =
{
   {0,  KL, 0,  L},  //kw1
   {2,  KL, 0,  R},  //k2
   {4,  KB, 0,  L},  //k1
   {6,  KB, 0,  R},  //k2
   {8,  KR, 15, L},  //k3
   {10, KR, 15, R},  //k4
   {12, KA, 15, L},  //k5
   {14, KA, 15, R},  //k6
   {16, KR, 30, L},  //ke1
   {18, KR, 30, R},  //ke2
   {20, KB, 30, L},  //k7
   {22, KB, 30, R},  //k8
   {24, KL, 45, L},  //k9
   {26, KL, 45, R},  //k10
   {28, KA, 45, L},  //k11
   {30, KA, 45, R},  //k12
   {32, KL, 60, L},  //ke3
   {34, KL, 60, R},  //ke4
   {36, KR, 60, L},  //k13
   {38, KR, 60, R},  //k14
   {40, KB, 60, L},  //k15
   {42, KB, 60, R},  //k16
   {44, KL, 77, L},  //k17
   {46, KL, 77, R},  //k18
   {48, KA, 77, L},  //ke5
   {50, KA, 77, R},  //ke6
   {52, KR, 94, L},  //k19
   {54, KR, 94, R},  //k20
   {56, KA, 94, L},  //k21
   {58, KA, 94, R},  //k22
   {60, KL, 111, L}, //k23
   {62, KL, 111, R}, //k24
   {64, KB, 111, L}, //kw3
   {66, KB, 111, R}, //kw4
};

//Key schedule constants
static const uint32_t sigma[12u] =
{
   0xA09E667Ful, 0x3BCC908Bul,
   0xB67AE858ul, 0x4CAA73B2ul,
   0xC6EF372Ful, 0xE94F82BEul,
   0x54FF53A5ul, 0xF1D36F1Cul,
   0x10E527FAul, 0xDE682D1Dul,
   0xB05688C2ul, 0xB3E6C1FDul
};

//Substitution table 1
static const uint8_t sbox1[256u] =
{
   0x70, 0x82, 0x2C, 0xEC, 0xB3, 0x27, 0xC0, 0xE5, 0xE4, 0x85, 0x57, 0x35, 0xEA, 0x0C, 0xAE, 0x41,
   0x23, 0xEF, 0x6B, 0x93, 0x45, 0x19, 0xA5, 0x21, 0xED, 0x0E, 0x4F, 0x4E, 0x1D, 0x65, 0x92, 0xBD,
   0x86, 0xB8, 0xAF, 0x8F, 0x7C, 0xEB, 0x1F, 0xCE, 0x3E, 0x30, 0xDC, 0x5F, 0x5E, 0xC5, 0x0B, 0x1A,
   0xA6, 0xE1, 0x39, 0xCA, 0xD5, 0x47, 0x5D, 0x3D, 0xD9, 0x01, 0x5A, 0xD6, 0x51, 0x56, 0x6C, 0x4D,
   0x8B, 0x0D, 0x9A, 0x66, 0xFB, 0xCC, 0xB0, 0x2D, 0x74, 0x12, 0x2B, 0x20, 0xF0, 0xB1, 0x84, 0x99,
   0xDF, 0x4C, 0xCB, 0xC2, 0x34, 0x7E, 0x76, 0x05, 0x6D, 0xB7, 0xA9, 0x31, 0xD1, 0x17, 0x04, 0xD7,
   0x14, 0x58, 0x3A, 0x61, 0xDE, 0x1B, 0x11, 0x1C, 0x32, 0x0F, 0x9C, 0x16, 0x53, 0x18, 0xF2, 0x22,
   0xFE, 0x44, 0xCF, 0xB2, 0xC3, 0xB5, 0x7A, 0x91, 0x24, 0x08, 0xE8, 0xA8, 0x60, 0xFC, 0x69, 0x50,
   0xAA, 0xD0, 0xA0, 0x7D, 0xA1, 0x89, 0x62, 0x97, 0x54, 0x5B, 0x1E, 0x95, 0xE0, 0xFF, 0x64, 0xD2,
   0x10, 0xC4, 0x00, 0x48, 0xA3, 0xF7, 0x75, 0xDB, 0x8A, 0x03, 0xE6, 0xDA, 0x09, 0x3F, 0xDD, 0x94,
   0x87, 0x5C, 0x83, 0x02, 0xCD, 0x4A, 0x90, 0x33, 0x73, 0x67, 0xF6, 0xF3, 0x9D, 0x7F, 0xBF, 0xE2,
   0x52, 0x9B, 0xD8, 0x26, 0xC8, 0x37, 0xC6, 0x3B, 0x81, 0x96, 0x6F, 0x4B, 0x13, 0xBE, 0x63, 0x2E,
   0xE9, 0x79, 0xA7, 0x8C, 0x9F, 0x6E, 0xBC, 0x8E, 0x29, 0xF5, 0xF9, 0xB6, 0x2F, 0xFD, 0xB4, 0x59,
   0x78, 0x98, 0x06, 0x6A, 0xE7, 0x46, 0x71, 0xBA, 0xD4, 0x25, 0xAB, 0x42, 0x88, 0xA2, 0x8D, 0xFA,
   0x72, 0x07, 0xB9, 0x55, 0xF8, 0xEE, 0xAC, 0x0A, 0x36, 0x49, 0x2A, 0x68, 0x3C, 0x38, 0xF1, 0xA4,
   0x40, 0x28, 0xD3, 0x7B, 0xBB, 0xC9, 0x43, 0xC1, 0x15, 0xE3, 0xAD, 0xF4, 0x77, 0xC7, 0x80, 0x9E
};

//Substitution table 2
static const uint8_t sbox2[256] =
{
   0xE0, 0x05, 0x58, 0xD9, 0x67, 0x4E, 0x81, 0xCB, 0xC9, 0x0B, 0xAE, 0x6A, 0xD5, 0x18, 0x5D, 0x82,
   0x46, 0xDF, 0xD6, 0x27, 0x8A, 0x32, 0x4B, 0x42, 0xDB, 0x1C, 0x9E, 0x9C, 0x3A, 0xCA, 0x25, 0x7B,
   0x0D, 0x71, 0x5F, 0x1F, 0xF8, 0xD7, 0x3E, 0x9D, 0x7C, 0x60, 0xB9, 0xBE, 0xBC, 0x8B, 0x16, 0x34,
   0x4D, 0xC3, 0x72, 0x95, 0xAB, 0x8E, 0xBA, 0x7A, 0xB3, 0x02, 0xB4, 0xAD, 0xA2, 0xAC, 0xD8, 0x9A,
   0x17, 0x1A, 0x35, 0xCC, 0xF7, 0x99, 0x61, 0x5A, 0xE8, 0x24, 0x56, 0x40, 0xE1, 0x63, 0x09, 0x33,
   0xBF, 0x98, 0x97, 0x85, 0x68, 0xFC, 0xEC, 0x0A, 0xDA, 0x6F, 0x53, 0x62, 0xA3, 0x2E, 0x08, 0xAF,
   0x28, 0xB0, 0x74, 0xC2, 0xBD, 0x36, 0x22, 0x38, 0x64, 0x1E, 0x39, 0x2C, 0xA6, 0x30, 0xE5, 0x44,
   0xFD, 0x88, 0x9F, 0x65, 0x87, 0x6B, 0xF4, 0x23, 0x48, 0x10, 0xD1, 0x51, 0xC0, 0xF9, 0xD2, 0xA0,
   0x55, 0xA1, 0x41, 0xFA, 0x43, 0x13, 0xC4, 0x2F, 0xA8, 0xB6, 0x3C, 0x2B, 0xC1, 0xFF, 0xC8, 0xA5,
   0x20, 0x89, 0x00, 0x90, 0x47, 0xEF, 0xEA, 0xB7, 0x15, 0x06, 0xCD, 0xB5, 0x12, 0x7E, 0xBB, 0x29,
   0x0F, 0xB8, 0x07, 0x04, 0x9B, 0x94, 0x21, 0x66, 0xE6, 0xCE, 0xED, 0xE7, 0x3B, 0xFE, 0x7F, 0xC5,
   0xA4, 0x37, 0xB1, 0x4C, 0x91, 0x6E, 0x8D, 0x76, 0x03, 0x2D, 0xDE, 0x96, 0x26, 0x7D, 0xC6, 0x5C,
   0xD3, 0xF2, 0x4F, 0x19, 0x3F, 0xDC, 0x79, 0x1D, 0x52, 0xEB, 0xF3, 0x6D, 0x5E, 0xFB, 0x69, 0xB2,
   0xF0, 0x31, 0x0C, 0xD4, 0xCF, 0x8C, 0xE2, 0x75, 0xA9, 0x4A, 0x57, 0x84, 0x11, 0x45, 0x1B, 0xF5,
   0xE4, 0x0E, 0x73, 0xAA, 0xF1, 0xDD, 0x59, 0x14, 0x6C, 0x92, 0x54, 0xD0, 0x78, 0x70, 0xE3, 0x49,
   0x80, 0x50, 0xA7, 0xF6, 0x77, 0x93, 0x86, 0x83, 0x2A, 0xC7, 0x5B, 0xE9, 0xEE, 0x8F, 0x01, 0x3D
};

//Substitution table 3
static const uint8_t sbox3[256u] =
{
   0x38, 0x41, 0x16, 0x76, 0xD9, 0x93, 0x60, 0xF2, 0x72, 0xC2, 0xAB, 0x9A, 0x75, 0x06, 0x57, 0xA0,
   0x91, 0xF7, 0xB5, 0xC9, 0xA2, 0x8C, 0xD2, 0x90, 0xF6, 0x07, 0xA7, 0x27, 0x8E, 0xB2, 0x49, 0xDE,
   0x43, 0x5C, 0xD7, 0xC7, 0x3E, 0xF5, 0x8F, 0x67, 0x1F, 0x18, 0x6E, 0xAF, 0x2F, 0xE2, 0x85, 0x0D,
   0x53, 0xF0, 0x9C, 0x65, 0xEA, 0xA3, 0xAE, 0x9E, 0xEC, 0x80, 0x2D, 0x6B, 0xA8, 0x2B, 0x36, 0xA6,
   0xC5, 0x86, 0x4D, 0x33, 0xFD, 0x66, 0x58, 0x96, 0x3A, 0x09, 0x95, 0x10, 0x78, 0xD8, 0x42, 0xCC,
   0xEF, 0x26, 0xE5, 0x61, 0x1A, 0x3F, 0x3B, 0x82, 0xB6, 0xDB, 0xD4, 0x98, 0xE8, 0x8B, 0x02, 0xEB,
   0x0A, 0x2C, 0x1D, 0xB0, 0x6F, 0x8D, 0x88, 0x0E, 0x19, 0x87, 0x4E, 0x0B, 0xA9, 0x0C, 0x79, 0x11,
   0x7F, 0x22, 0xE7, 0x59, 0xE1, 0xDA, 0x3D, 0xC8, 0x12, 0x04, 0x74, 0x54, 0x30, 0x7E, 0xB4, 0x28,
   0x55, 0x68, 0x50, 0xBE, 0xD0, 0xC4, 0x31, 0xCB, 0x2A, 0xAD, 0x0F, 0xCA, 0x70, 0xFF, 0x32, 0x69,
   0x08, 0x62, 0x00, 0x24, 0xD1, 0xFB, 0xBA, 0xED, 0x45, 0x81, 0x73, 0x6D, 0x84, 0x9F, 0xEE, 0x4A,
   0xC3, 0x2E, 0xC1, 0x01, 0xE6, 0x25, 0x48, 0x99, 0xB9, 0xB3, 0x7B, 0xF9, 0xCE, 0xBF, 0xDF, 0x71,
   0x29, 0xCD, 0x6C, 0x13, 0x64, 0x9B, 0x63, 0x9D, 0xC0, 0x4B, 0xB7, 0xA5, 0x89, 0x5F, 0xB1, 0x17,
   0xF4, 0xBC, 0xD3, 0x46, 0xCF, 0x37, 0x5E, 0x47, 0x94, 0xFA, 0xFC, 0x5B, 0x97, 0xFE, 0x5A, 0xAC,
   0x3C, 0x4C, 0x03, 0x35, 0xF3, 0x23, 0xB8, 0x5D, 0x6A, 0x92, 0xD5, 0x21, 0x44, 0x51, 0xC6, 0x7D,
   0x39, 0x83, 0xDC, 0xAA, 0x7C, 0x77, 0x56, 0x05, 0x1B, 0xA4, 0x15, 0x34, 0x1E, 0x1C, 0xF8, 0x52,
   0x20, 0x14, 0xE9, 0xBD, 0xDD, 0xE4, 0xA1, 0xE0, 0x8A, 0xF1, 0xD6, 0x7A, 0xBB, 0xE3, 0x40, 0x4F
};

//Substitution table 4
static const uint8_t sbox4[256u] =
{
   0x70, 0x2C, 0xB3, 0xC0, 0xE4, 0x57, 0xEA, 0xAE, 0x23, 0x6B, 0x45, 0xA5, 0xED, 0x4F, 0x1D, 0x92,
   0x86, 0xAF, 0x7C, 0x1F, 0x3E, 0xDC, 0x5E, 0x0B, 0xA6, 0x39, 0xD5, 0x5D, 0xD9, 0x5A, 0x51, 0x6C,
   0x8B, 0x9A, 0xFB, 0xB0, 0x74, 0x2B, 0xF0, 0x84, 0xDF, 0xCB, 0x34, 0x76, 0x6D, 0xA9, 0xD1, 0x04,
   0x14, 0x3A, 0xDE, 0x11, 0x32, 0x9C, 0x53, 0xF2, 0xFE, 0xCF, 0xC3, 0x7A, 0x24, 0xE8, 0x60, 0x69,
   0xAA, 0xA0, 0xA1, 0x62, 0x54, 0x1E, 0xE0, 0x64, 0x10, 0x00, 0xA3, 0x75, 0x8A, 0xE6, 0x09, 0xDD,
   0x87, 0x83, 0xCD, 0x90, 0x73, 0xF6, 0x9D, 0xBF, 0x52, 0xD8, 0xC8, 0xC6, 0x81, 0x6F, 0x13, 0x63,
   0xE9, 0xA7, 0x9F, 0xBC, 0x29, 0xF9, 0x2F, 0xB4, 0x78, 0x06, 0xE7, 0x71, 0xD4, 0xAB, 0x88, 0x8D,
   0x72, 0xB9, 0xF8, 0xAC, 0x36, 0x2A, 0x3C, 0xF1, 0x40, 0xD3, 0xBB, 0x43, 0x15, 0xAD, 0x77, 0x80,
   0x82, 0xEC, 0x27, 0xE5, 0x85, 0x35, 0x0C, 0x41, 0xEF, 0x93, 0x19, 0x21, 0x0E, 0x4E, 0x65, 0xBD,
   0xB8, 0x8F, 0xEB, 0xCE, 0x30, 0x5F, 0xC5, 0x1A, 0xE1, 0xCA, 0x47, 0x3D, 0x01, 0xD6, 0x56, 0x4D,
   0x0D, 0x66, 0xCC, 0x2D, 0x12, 0x20, 0xB1, 0x99, 0x4C, 0xC2, 0x7E, 0x05, 0xB7, 0x31, 0x17, 0xD7,
   0x58, 0x61, 0x1B, 0x1C, 0x0F, 0x16, 0x18, 0x22, 0x44, 0xB2, 0xB5, 0x91, 0x08, 0xA8, 0xFC, 0x50,
   0xD0, 0x7D, 0x89, 0x97, 0x5B, 0x95, 0xFF, 0xD2, 0xC4, 0x48, 0xF7, 0xDB, 0x03, 0xDA, 0x3F, 0x94,
   0x5C, 0x02, 0x4A, 0x33, 0x67, 0xF3, 0x7F, 0xE2, 0x9B, 0x26, 0x37, 0x3B, 0x96, 0x4B, 0xBE, 0x2E,
   0x79, 0x8C, 0x6E, 0x8E, 0xF5, 0xB6, 0xFD, 0x59, 0x98, 0x6A, 0x46, 0xBA, 0x25, 0x42, 0xA2, 0xFA,
   0x07, 0x55, 0xEE, 0x0A, 0x49, 0x68, 0x38, 0xA4, 0x28, 0x7B, 0xC9, 0xC1, 0xE3, 0xF4, 0xC7, 0x9E
};

//Common interface for encryption algorithms
const CipherAlgo_t camelliaCipherAlgo =
{
   "CAMELLIA",
   sizeof(CamelliaContext_t),
   CIPHER_ALGO_TYPE_BLOCK,
   CAMELLIA_BLOCK_SIZE,
   (CipherAlgoInit) camelliaInit,
//   NULL,
//   NULL,
   (CipherAlgoEncryptBlock) camelliaEncryptBlock,
   (CipherAlgoDecryptBlock) camelliaDecryptBlock
};
/**
 * @brief Initialize a Camellia context using the supplied key
 * @param[in] context Pointer to the Camellia context to initialize
 * @param[in] key Pointer to the key
 * @param[in] keyLen Length of the key
 * @return Error code
 **/
uint8_t camelliaInit(CamelliaContext_t *context, const uint8_t *key, size_t keyLen)
{
   uint8_t i;
   uint32_t temp1;
   uint32_t temp2;
   uint32_t *k;
   const CamelliaSubkey_t *p;
   uint8_t n;
   uint8_t m;

   if(context == NULL || key == NULL)                                           //Check parameters
      return 0u;

   if(keyLen == 16)                                                             //Check the length of the key
   {
      context->nr = 18;                                                         //18 rounds are required for 128-bit key
   }
   else if(keyLen == 24 || keyLen == 32)
   {
      context->nr = 24;                                                         //24 rounds are required for 192 and 256-bit keys
   }
   else
   {
      return 2u;                                                                //Report an error
   }
   k = context->k;                                                              //Point to KA, KB, KL and KR
   memset((void*)k,(void*) 0,(int16_t) 64);                                     //Clear key contents
   memcpy((void*)k,(void*) key,(int16_t) keyLen);                               //Save the supplied secret key
   if(keyLen == 24)                                                             //192-bit keys require special processing
   {
      k[KR + 2] = ~k[KR + 0];                                                   //Form a 256-bit key
      k[KR + 3] = ~k[KR + 1];
   }
   for(i = 0; i < 4; i++)                                                       //XOR KL and KR before applying the rounds
   {
      k[KL + i] = betoh32(k[KL + i]);
      k[KR + i] = betoh32(k[KR + i]);
      k[KB + i] = k[KL + i] ^ k[KR + i];
   }
   for(i = 0; i < 6; i++)                                                       //Generate the 128-bit keys KA and KB
   {
      CAMELLIA_ROUND(k[KB + 0], k[KB + 1], k[KB + 2], k[KB + 3], sigma[2 * i], sigma[2 * i + 1]);       //Apply round function
      if(i == 1)                                                                //The 2nd round requires special processing
      {
         k[KB + 0] ^= k[KL + 0];                                                //The result is XORed with KL
         k[KB + 1] ^= k[KL + 1];
         k[KB + 2] ^= k[KL + 2];
         k[KB + 3] ^= k[KL + 3];
      }
      else if(i == 3)                                                           //The 4th round requires special processing
      {
         memcpy((void*)k + KA,(void*) k + KB,(int16_t) 16);                     //Save KA after the 4th round
         k[KB + 0] ^= k[KR + 0];                                                //The result is XORed with KR
         k[KB + 1] ^= k[KR + 1];
         k[KB + 2] ^= k[KR + 2];
         k[KB + 3] ^= k[KR + 3];
      }
   }
   if(keyLen == 16)                                                             //The key schedule depends on the length of key
   {
      i = arraysize(ks1);                                                       //Key schedule for 128-bit key
      p = ks1;
   }
   else
   {
      i = arraysize(ks2);                                                       //Key schedule for 192 and 256-bit keys
      p = ks2;
   }
   while(i > 0)                                                                 //Generate subkeys
   {
      n = (p->shift + p->position) / 32;                                        //Calculate the shift count
      m = (p->shift + p->position) % 32;
      k = context->k + p->key;                                                  //Point to KL, KR, KA or KB
      if(m == 0)                                                                //Generate the current subkey
      {
         context->ks[p->index] = k[n % 4];
         context->ks[p->index + 1] = k[(n + 1) % 4];
      }
      else
      {
         context->ks[p->index] = (k[n % 4] << m) | (k[(n + 1) % 4] >> (32 - m));
         context->ks[p->index + 1] = (k[(n + 1) % 4] << m) | (k[(n + 2) % 4] >> (32 - m));
      }
      p++;                                                                      //Next subkey
      i--;
   }
   return 1u;                                                                   //No error to report
}
/**
 * @brief Encrypt a 16-byte block using Camellia algorithm
 * @param[in] context Pointer to the Camellia context
 * @param[in] input Plaintext block to encrypt
 * @param[out] output Ciphertext block resulting from encryption
 **/
void camelliaEncryptBlock(CamelliaContext_t *context, const uint8_t *input, uint8_t *output)
{
   uint8_t i;
   uint32_t temp1;
   uint32_t temp2;
   uint32_t *ks;
   uint32_t left1 = LOAD32BE(input + 0);                                        //The plaintext is separated into two parts (L and R)
   uint32_t left2 = LOAD32BE(input + 4);
   uint32_t right1 = LOAD32BE(input + 8);
   uint32_t right2 = LOAD32BE(input + 12);
   ks = context->ks;                                                            //The key schedule must be applied in ascending order
   left1 ^= ks[0u];                                                              //XOR plaintext with kw1 and kw2
   left2 ^= ks[1u];
   right1 ^= ks[2u];
   right2 ^= ks[3u];
   ks += 4;                                                                     //Advance current location in key schedule
   for(i = context->nr; i > 0u; i--)                                             //Apply round function 18 or 24 times depending on the key length
   {
      CAMELLIA_ROUND(left1, left2, right1, right2, ks[0u], ks[1u]);             //Apply round function
      ks += 2;                                                                  //Advance current location in key schedule
      if(i == 7 || i == 13 || i == 19)                                          //6th, 12th and 18th rounds require special processing
      {
         CAMELLIA_FL(left1, left2, ks[0], ks[1])                                //Apply FL-function
         CAMELLIA_INV_FL(right1, right2, ks[2], ks[3])                          //Apply inverse FL-function
         ks += 4;                                                               //Advance current location in key schedule
      }
   }
   right1 ^= ks[0];                                                             //XOR operation with kw3 and kw4
   right2 ^= ks[1];
   left1 ^= ks[2];
   left2 ^= ks[3];
   STORE32BE(right1, output + 0);                                               //The resulting value is the ciphertext
   STORE32BE(right2, output + 4);
   STORE32BE(left1, output + 8);
   STORE32BE(left2, output + 12);
}
/**
 * @brief Decrypt a 16-byte block using Camellia algorithm
 * @param[in] context Pointer to the Camellia context
 * @param[in] input Ciphertext block to decrypt
 * @param[out] output Plaintext block resulting from decryption
 **/
void camelliaDecryptBlock(CamelliaContext_t *context, const uint8_t *input, uint8_t *output)
{
   uint8_t i;
   uint32_t temp1;
   uint32_t temp2;
   uint32_t *ks;

   uint32_t right1 = LOAD32BE(input + 0);                                       //The ciphertext is separated into two parts (L and R)
   uint32_t right2 = LOAD32BE(input + 4);
   uint32_t left1 = LOAD32BE(input + 8);
   uint32_t left2 = LOAD32BE(input + 12);

   ks = (context->nr == 18) ? (context->ks + 48) : (context->ks + 64);          //The key schedule must be applied in reverse order

   right1 ^= ks[0];                                                             //XOR ciphertext with kw3 and kw4
   right2 ^= ks[1];
   left1 ^= ks[2];
   left2 ^= ks[3];

   for(i = context->nr; i > 0; i--)                                             //Apply round function 18 or 24 times depending on the key length
   {
      ks -= 2;                                                                  //Update current location in key schedule
      CAMELLIA_ROUND(right1, right2, left1, left2, ks[0], ks[1]);               //Apply round function

      if(i == 7 || i == 13 || i == 19)                                          //6th, 12th and 18th rounds require special processing
      {
         ks -= 4;                                                               //Update current location in key schedule
         CAMELLIA_FL(right1, right2, ks[2], ks[3])                              //Apply FL-function
         CAMELLIA_INV_FL(left1, left2, ks[0], ks[1])                            //Apply inverse FL-function
      }
   }
   ks -= 4;                                                                     //Update current location in key schedule
   left1 ^= ks[0];                                                              //XOR operation with kw1 and kw2
   left2 ^= ks[1];
   right1 ^= ks[2];
   right2 ^= ks[3];
   STORE32BE(left1, output + 0);                                                //The resulting value is the plaintext
   STORE32BE(left2, output + 4);
   STORE32BE(right1, output + 8);
   STORE32BE(right2, output + 12);
}
#elif defined(ADV_ENCPT_STD_USED)
//Substitution table used by encryption algorithm (S-box)
static const uint8_t sbox[256] =
{
   0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5, 0x30, 0x01, 0x67, 0x2B, 0xFE, 0xD7, 0xAB, 0x76,
   0xCA, 0x82, 0xC9, 0x7D, 0xFA, 0x59, 0x47, 0xF0, 0xAD, 0xD4, 0xA2, 0xAF, 0x9C, 0xA4, 0x72, 0xC0,
   0xB7, 0xFD, 0x93, 0x26, 0x36, 0x3F, 0xF7, 0xCC, 0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
   0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A, 0x07, 0x12, 0x80, 0xE2, 0xEB, 0x27, 0xB2, 0x75,
   0x09, 0x83, 0x2C, 0x1A, 0x1B, 0x6E, 0x5A, 0xA0, 0x52, 0x3B, 0xD6, 0xB3, 0x29, 0xE3, 0x2F, 0x84,
   0x53, 0xD1, 0x00, 0xED, 0x20, 0xFC, 0xB1, 0x5B, 0x6A, 0xCB, 0xBE, 0x39, 0x4A, 0x4C, 0x58, 0xCF,
   0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85, 0x45, 0xF9, 0x02, 0x7F, 0x50, 0x3C, 0x9F, 0xA8,
   0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5, 0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2,
   0xCD, 0x0C, 0x13, 0xEC, 0x5F, 0x97, 0x44, 0x17, 0xC4, 0xA7, 0x7E, 0x3D, 0x64, 0x5D, 0x19, 0x73,
   0x60, 0x81, 0x4F, 0xDC, 0x22, 0x2A, 0x90, 0x88, 0x46, 0xEE, 0xB8, 0x14, 0xDE, 0x5E, 0x0B, 0xDB,
   0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C, 0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79,
   0xE7, 0xC8, 0x37, 0x6D, 0x8D, 0xD5, 0x4E, 0xA9, 0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
   0xBA, 0x78, 0x25, 0x2E, 0x1C, 0xA6, 0xB4, 0xC6, 0xE8, 0xDD, 0x74, 0x1F, 0x4B, 0xBD, 0x8B, 0x8A,
   0x70, 0x3E, 0xB5, 0x66, 0x48, 0x03, 0xF6, 0x0E, 0x61, 0x35, 0x57, 0xB9, 0x86, 0xC1, 0x1D, 0x9E,
   0xE1, 0xF8, 0x98, 0x11, 0x69, 0xD9, 0x8E, 0x94, 0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
   0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68, 0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16
};

//Substitution table used by decryption algorithm (inverse S-box)
static const uint8_t isbox[256] =
{
   0x52, 0x09, 0x6A, 0xD5, 0x30, 0x36, 0xA5, 0x38, 0xBF, 0x40, 0xA3, 0x9E, 0x81, 0xF3, 0xD7, 0xFB,
   0x7C, 0xE3, 0x39, 0x82, 0x9B, 0x2F, 0xFF, 0x87, 0x34, 0x8E, 0x43, 0x44, 0xC4, 0xDE, 0xE9, 0xCB,
   0x54, 0x7B, 0x94, 0x32, 0xA6, 0xC2, 0x23, 0x3D, 0xEE, 0x4C, 0x95, 0x0B, 0x42, 0xFA, 0xC3, 0x4E,
   0x08, 0x2E, 0xA1, 0x66, 0x28, 0xD9, 0x24, 0xB2, 0x76, 0x5B, 0xA2, 0x49, 0x6D, 0x8B, 0xD1, 0x25,
   0x72, 0xF8, 0xF6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xD4, 0xA4, 0x5C, 0xCC, 0x5D, 0x65, 0xB6, 0x92,
   0x6C, 0x70, 0x48, 0x50, 0xFD, 0xED, 0xB9, 0xDA, 0x5E, 0x15, 0x46, 0x57, 0xA7, 0x8D, 0x9D, 0x84,
   0x90, 0xD8, 0xAB, 0x00, 0x8C, 0xBC, 0xD3, 0x0A, 0xF7, 0xE4, 0x58, 0x05, 0xB8, 0xB3, 0x45, 0x06,
   0xD0, 0x2C, 0x1E, 0x8F, 0xCA, 0x3F, 0x0F, 0x02, 0xC1, 0xAF, 0xBD, 0x03, 0x01, 0x13, 0x8A, 0x6B,
   0x3A, 0x91, 0x11, 0x41, 0x4F, 0x67, 0xDC, 0xEA, 0x97, 0xF2, 0xCF, 0xCE, 0xF0, 0xB4, 0xE6, 0x73,
   0x96, 0xAC, 0x74, 0x22, 0xE7, 0xAD, 0x35, 0x85, 0xE2, 0xF9, 0x37, 0xE8, 0x1C, 0x75, 0xDF, 0x6E,
   0x47, 0xF1, 0x1A, 0x71, 0x1D, 0x29, 0xC5, 0x89, 0x6F, 0xB7, 0x62, 0x0E, 0xAA, 0x18, 0xBE, 0x1B,
   0xFC, 0x56, 0x3E, 0x4B, 0xC6, 0xD2, 0x79, 0x20, 0x9A, 0xDB, 0xC0, 0xFE, 0x78, 0xCD, 0x5A, 0xF4,
   0x1F, 0xDD, 0xA8, 0x33, 0x88, 0x07, 0xC7, 0x31, 0xB1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xEC, 0x5F,
   0x60, 0x51, 0x7F, 0xA9, 0x19, 0xB5, 0x4A, 0x0D, 0x2D, 0xE5, 0x7A, 0x9F, 0x93, 0xC9, 0x9C, 0xEF,
   0xA0, 0xE0, 0x3B, 0x4D, 0xAE, 0x2A, 0xF5, 0xB0, 0xC8, 0xEB, 0xBB, 0x3C, 0x83, 0x53, 0x99, 0x61,
   0x17, 0x2B, 0x04, 0x7E, 0xBA, 0x77, 0xD6, 0x26, 0xE1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0C, 0x7D
};

//Precalculated table (encryption)
static const uint32_t te[256] =
{
   0xA56363C6, 0x847C7CF8, 0x997777EE, 0x8D7B7BF6, 0x0DF2F2FF, 0xBD6B6BD6, 0xB16F6FDE, 0x54C5C591,
   0x50303060, 0x03010102, 0xA96767CE, 0x7D2B2B56, 0x19FEFEE7, 0x62D7D7B5, 0xE6ABAB4D, 0x9A7676EC,
   0x45CACA8F, 0x9D82821F, 0x40C9C989, 0x877D7DFA, 0x15FAFAEF, 0xEB5959B2, 0xC947478E, 0x0BF0F0FB,
   0xECADAD41, 0x67D4D4B3, 0xFDA2A25F, 0xEAAFAF45, 0xBF9C9C23, 0xF7A4A453, 0x967272E4, 0x5BC0C09B,
   0xC2B7B775, 0x1CFDFDE1, 0xAE93933D, 0x6A26264C, 0x5A36366C, 0x413F3F7E, 0x02F7F7F5, 0x4FCCCC83,
   0x5C343468, 0xF4A5A551, 0x34E5E5D1, 0x08F1F1F9, 0x937171E2, 0x73D8D8AB, 0x53313162, 0x3F15152A,
   0x0C040408, 0x52C7C795, 0x65232346, 0x5EC3C39D, 0x28181830, 0xA1969637, 0x0F05050A, 0xB59A9A2F,
   0x0907070E, 0x36121224, 0x9B80801B, 0x3DE2E2DF, 0x26EBEBCD, 0x6927274E, 0xCDB2B27F, 0x9F7575EA,
   0x1B090912, 0x9E83831D, 0x742C2C58, 0x2E1A1A34, 0x2D1B1B36, 0xB26E6EDC, 0xEE5A5AB4, 0xFBA0A05B,
   0xF65252A4, 0x4D3B3B76, 0x61D6D6B7, 0xCEB3B37D, 0x7B292952, 0x3EE3E3DD, 0x712F2F5E, 0x97848413,
   0xF55353A6, 0x68D1D1B9, 0x00000000, 0x2CEDEDC1, 0x60202040, 0x1FFCFCE3, 0xC8B1B179, 0xED5B5BB6,
   0xBE6A6AD4, 0x46CBCB8D, 0xD9BEBE67, 0x4B393972, 0xDE4A4A94, 0xD44C4C98, 0xE85858B0, 0x4ACFCF85,
   0x6BD0D0BB, 0x2AEFEFC5, 0xE5AAAA4F, 0x16FBFBED, 0xC5434386, 0xD74D4D9A, 0x55333366, 0x94858511,
   0xCF45458A, 0x10F9F9E9, 0x06020204, 0x817F7FFE, 0xF05050A0, 0x443C3C78, 0xBA9F9F25, 0xE3A8A84B,
   0xF35151A2, 0xFEA3A35D, 0xC0404080, 0x8A8F8F05, 0xAD92923F, 0xBC9D9D21, 0x48383870, 0x04F5F5F1,
   0xDFBCBC63, 0xC1B6B677, 0x75DADAAF, 0x63212142, 0x30101020, 0x1AFFFFE5, 0x0EF3F3FD, 0x6DD2D2BF,
   0x4CCDCD81, 0x140C0C18, 0x35131326, 0x2FECECC3, 0xE15F5FBE, 0xA2979735, 0xCC444488, 0x3917172E,
   0x57C4C493, 0xF2A7A755, 0x827E7EFC, 0x473D3D7A, 0xAC6464C8, 0xE75D5DBA, 0x2B191932, 0x957373E6,
   0xA06060C0, 0x98818119, 0xD14F4F9E, 0x7FDCDCA3, 0x66222244, 0x7E2A2A54, 0xAB90903B, 0x8388880B,
   0xCA46468C, 0x29EEEEC7, 0xD3B8B86B, 0x3C141428, 0x79DEDEA7, 0xE25E5EBC, 0x1D0B0B16, 0x76DBDBAD,
   0x3BE0E0DB, 0x56323264, 0x4E3A3A74, 0x1E0A0A14, 0xDB494992, 0x0A06060C, 0x6C242448, 0xE45C5CB8,
   0x5DC2C29F, 0x6ED3D3BD, 0xEFACAC43, 0xA66262C4, 0xA8919139, 0xA4959531, 0x37E4E4D3, 0x8B7979F2,
   0x32E7E7D5, 0x43C8C88B, 0x5937376E, 0xB76D6DDA, 0x8C8D8D01, 0x64D5D5B1, 0xD24E4E9C, 0xE0A9A949,
   0xB46C6CD8, 0xFA5656AC, 0x07F4F4F3, 0x25EAEACF, 0xAF6565CA, 0x8E7A7AF4, 0xE9AEAE47, 0x18080810,
   0xD5BABA6F, 0x887878F0, 0x6F25254A, 0x722E2E5C, 0x241C1C38, 0xF1A6A657, 0xC7B4B473, 0x51C6C697,
   0x23E8E8CB, 0x7CDDDDA1, 0x9C7474E8, 0x211F1F3E, 0xDD4B4B96, 0xDCBDBD61, 0x868B8B0D, 0x858A8A0F,
   0x907070E0, 0x423E3E7C, 0xC4B5B571, 0xAA6666CC, 0xD8484890, 0x05030306, 0x01F6F6F7, 0x120E0E1C,
   0xA36161C2, 0x5F35356A, 0xF95757AE, 0xD0B9B969, 0x91868617, 0x58C1C199, 0x271D1D3A, 0xB99E9E27,
   0x38E1E1D9, 0x13F8F8EB, 0xB398982B, 0x33111122, 0xBB6969D2, 0x70D9D9A9, 0x898E8E07, 0xA7949433,
   0xB69B9B2D, 0x221E1E3C, 0x92878715, 0x20E9E9C9, 0x49CECE87, 0xFF5555AA, 0x78282850, 0x7ADFDFA5,
   0x8F8C8C03, 0xF8A1A159, 0x80898909, 0x170D0D1A, 0xDABFBF65, 0x31E6E6D7, 0xC6424284, 0xB86868D0,
   0xC3414182, 0xB0999929, 0x772D2D5A, 0x110F0F1E, 0xCBB0B07B, 0xFC5454A8, 0xD6BBBB6D, 0x3A16162C
};

//Precalculated table (decryption)
static const uint32_t td[256] =
{
   0x50A7F451, 0x5365417E, 0xC3A4171A, 0x965E273A, 0xCB6BAB3B, 0xF1459D1F, 0xAB58FAAC, 0x9303E34B,
   0x55FA3020, 0xF66D76AD, 0x9176CC88, 0x254C02F5, 0xFCD7E54F, 0xD7CB2AC5, 0x80443526, 0x8FA362B5,
   0x495AB1DE, 0x671BBA25, 0x980EEA45, 0xE1C0FE5D, 0x02752FC3, 0x12F04C81, 0xA397468D, 0xC6F9D36B,
   0xE75F8F03, 0x959C9215, 0xEB7A6DBF, 0xDA595295, 0x2D83BED4, 0xD3217458, 0x2969E049, 0x44C8C98E,
   0x6A89C275, 0x78798EF4, 0x6B3E5899, 0xDD71B927, 0xB64FE1BE, 0x17AD88F0, 0x66AC20C9, 0xB43ACE7D,
   0x184ADF63, 0x82311AE5, 0x60335197, 0x457F5362, 0xE07764B1, 0x84AE6BBB, 0x1CA081FE, 0x942B08F9,
   0x58684870, 0x19FD458F, 0x876CDE94, 0xB7F87B52, 0x23D373AB, 0xE2024B72, 0x578F1FE3, 0x2AAB5566,
   0x0728EBB2, 0x03C2B52F, 0x9A7BC586, 0xA50837D3, 0xF2872830, 0xB2A5BF23, 0xBA6A0302, 0x5C8216ED,
   0x2B1CCF8A, 0x92B479A7, 0xF0F207F3, 0xA1E2694E, 0xCDF4DA65, 0xD5BE0506, 0x1F6234D1, 0x8AFEA6C4,
   0x9D532E34, 0xA055F3A2, 0x32E18A05, 0x75EBF6A4, 0x39EC830B, 0xAAEF6040, 0x069F715E, 0x51106EBD,
   0xF98A213E, 0x3D06DD96, 0xAE053EDD, 0x46BDE64D, 0xB58D5491, 0x055DC471, 0x6FD40604, 0xFF155060,
   0x24FB9819, 0x97E9BDD6, 0xCC434089, 0x779ED967, 0xBD42E8B0, 0x888B8907, 0x385B19E7, 0xDBEEC879,
   0x470A7CA1, 0xE90F427C, 0xC91E84F8, 0x00000000, 0x83868009, 0x48ED2B32, 0xAC70111E, 0x4E725A6C,
   0xFBFF0EFD, 0x5638850F, 0x1ED5AE3D, 0x27392D36, 0x64D90F0A, 0x21A65C68, 0xD1545B9B, 0x3A2E3624,
   0xB1670A0C, 0x0FE75793, 0xD296EEB4, 0x9E919B1B, 0x4FC5C080, 0xA220DC61, 0x694B775A, 0x161A121C,
   0x0ABA93E2, 0xE52AA0C0, 0x43E0223C, 0x1D171B12, 0x0B0D090E, 0xADC78BF2, 0xB9A8B62D, 0xC8A91E14,
   0x8519F157, 0x4C0775AF, 0xBBDD99EE, 0xFD607FA3, 0x9F2601F7, 0xBCF5725C, 0xC53B6644, 0x347EFB5B,
   0x7629438B, 0xDCC623CB, 0x68FCEDB6, 0x63F1E4B8, 0xCADC31D7, 0x10856342, 0x40229713, 0x2011C684,
   0x7D244A85, 0xF83DBBD2, 0x1132F9AE, 0x6DA129C7, 0x4B2F9E1D, 0xF330B2DC, 0xEC52860D, 0xD0E3C177,
   0x6C16B32B, 0x99B970A9, 0xFA489411, 0x2264E947, 0xC48CFCA8, 0x1A3FF0A0, 0xD82C7D56, 0xEF903322,
   0xC74E4987, 0xC1D138D9, 0xFEA2CA8C, 0x360BD498, 0xCF81F5A6, 0x28DE7AA5, 0x268EB7DA, 0xA4BFAD3F,
   0xE49D3A2C, 0x0D927850, 0x9BCC5F6A, 0x62467E54, 0xC2138DF6, 0xE8B8D890, 0x5EF7392E, 0xF5AFC382,
   0xBE805D9F, 0x7C93D069, 0xA92DD56F, 0xB31225CF, 0x3B99ACC8, 0xA77D1810, 0x6E639CE8, 0x7BBB3BDB,
   0x097826CD, 0xF418596E, 0x01B79AEC, 0xA89A4F83, 0x656E95E6, 0x7EE6FFAA, 0x08CFBC21, 0xE6E815EF,
   0xD99BE7BA, 0xCE366F4A, 0xD4099FEA, 0xD67CB029, 0xAFB2A431, 0x31233F2A, 0x3094A5C6, 0xC066A235,
   0x37BC4E74, 0xA6CA82FC, 0xB0D090E0, 0x15D8A733, 0x4A9804F1, 0xF7DAEC41, 0x0E50CD7F, 0x2FF69117,
   0x8DD64D76, 0x4DB0EF43, 0x544DAACC, 0xDF0496E4, 0xE3B5D19E, 0x1B886A4C, 0xB81F2CC1, 0x7F516546,
   0x04EA5E9D, 0x5D358C01, 0x737487FA, 0x2E410BFB, 0x5A1D67B3, 0x52D2DB92, 0x335610E9, 0x1347D66D,
   0x8C61D79A, 0x7A0CA137, 0x8E14F859, 0x893C13EB, 0xEE27A9CE, 0x35C961B7, 0xEDE51CE1, 0x3CB1477A,
   0x59DFD29C, 0x3F73F255, 0x79CE1418, 0xBF37C773, 0xEACDF753, 0x5BAAFD5F, 0x146F3DDF, 0x86DB4478,
   0x81F3AFCA, 0x3EC468B9, 0x2C342438, 0x5F40A3C2, 0x72C31D16, 0x0C25E2BC, 0x8B493C28, 0x41950DFF,
   0x7101A839, 0xDEB30C08, 0x9CE4B4D8, 0x90C15664, 0x6184CB7B, 0x70B632D5, 0x745C6C48, 0x4257B8D0
};

//Round constant word array
static const uint32_t rcon1[11] =
{
   0x00000000,
   0x00000001,
   0x00000002,
   0x00000004,
   0x00000008,
   0x00000010,
   0x00000020,
   0x00000040,
   0x00000080,
   0x0000001B,
   0x00000036
};

//Common interface for encryption algorithms
const CipherAlgo_t aesCipherAlgo =
{
   "AES",
   sizeof(AesContext_t),
   CIPHER_ALGO_TYPE_BLOCK,
   AES_BLOCK_SIZE,
   (CipherAlgoInit) aesInit,
   //NULL,
   //NULL,
   (CipherAlgoEncryptBlock) aesEncryptBlock,
   (CipherAlgoDecryptBlock) aesDecryptBlock
};
/**
 * @brief Key expansion
 * @param[in] context Pointer to the AES context to initialize
 * @param[in] key Pointer to the key
 * @param[in] keyLen Length of the key
 * @return Error code
 **/
uint8_t aesInit(AesContext_t *context, const uint8_t *key, size_t keyLen)
{
   uint8_t i;
   uint32_t temp;
   size_t keyScheduleSize;

   //Check parameters
   if(context == NULL || key == NULL)
      return 0u;

   //Check the length of the key
   if(keyLen == 16)                                                             // AES128
   {
      context->nr = 10;                                                         //10 rounds are required for 128-bit key
   }
   else if(keyLen == 24)                                                        // AES192
   {
      context->nr = 12;                                                         //12 rounds are required for 192-bit key
   }
   else if(keyLen == 32)                                                        // AES256
   {
      context->nr = 14;                                                         //14 rounds are required for 256-bit key
   }
   else
   {
      return 2u;                                                                //Report an error
   }

   keyLen /= 4;                                                                 //Determine the number of 32-bit words in the key

   for(i = 0u; i < keyLen; i++)                                                  //Copy the original key
   {
      context->ek[i] = LOAD32LE(key + (i * 4u));
   }

   keyScheduleSize = 4 * (context->nr + 1);                                     //The size of the key schedule depends on the number of rounds

   for(i = keyLen; i < keyScheduleSize; i++)                                    //Generate the key schedule (encryption)
   {
      temp = context->ek[i - 1];                                                //Save previous word
      if((i % keyLen) == 0u)                                                     //Apply transformation
      {
         context->ek[i] = sbox[(temp >> 8u) & 0xFF];
         context->ek[i] |= (sbox[(temp >> 16) & 0xFF] << 8);
         context->ek[i] |= (sbox[(temp >> 24) & 0xFF] << 16);
         context->ek[i] |= (sbox[temp & 0xFF] << 24);
         context->ek[i] ^= rcon1[i / keyLen];
      }
      else if(keyLen > 6 && (i % keyLen) == 4)
      {
         context->ek[i] = sbox[temp & 0xFF];
         context->ek[i] |= (sbox[(temp >> 8) & 0xFF] << 8);
         context->ek[i] |= (sbox[(temp >> 16) & 0xFF] << 16);
         context->ek[i] |= (sbox[(temp >> 24) & 0xFF] << 24);
      }
      else
      {
         context->ek[i] = temp;
      }

      context->ek[i] ^= context->ek[i - keyLen];                                //Update the key schedule
   }

   for(i = 0; i < keyScheduleSize; i++)                                         //Generate the key schedule (decryption)
   {
      //Apply the InvMixColumns transformation to all round keys
      //but the first and the last
      if(i < 4 || i >= (keyScheduleSize - 4))
      {
         context->dk[i] = context->ek[i];
      }
      else
      {
         context->dk[i] = td[sbox[context->ek[i] & 0xFFu]];
         temp = td[sbox[(context->ek[i] >> 8u) & 0xFFu]];
         context->dk[i] ^= ROL32(temp, 8u);
         temp = td[sbox[(context->ek[i] >> 16u) & 0xFFu]];
         context->dk[i] ^= ROL32(temp, 16u);
         temp = td[sbox[(context->ek[i] >> 24u) & 0xFFu]];
         context->dk[i] ^= ROL32(temp, 24u);
      }
   }

   //No error to report
   return 1u;
}


/**
 * @brief Encrypt a 16-byte block using AES algorithm
 * @param[in] context Pointer to the AES context
 * @param[in] input Plaintext block to encrypt
 * @param[out] output Ciphertext block resulting from encryption
 **/
void aesEncryptBlock(AesContext_t *context, const uint8_t *input, uint8_t *output)
{
   uint8_t i;
   uint32_t s0;
   uint32_t s1;
   uint32_t s2;
   uint32_t s3;
   uint32_t t0;
   uint32_t t1;
   uint32_t t2;
   uint32_t t3;
   uint32_t temp;

   //Copy the plaintext to the state array
   s0 = LOAD32LE(input + 0);
   s1 = LOAD32LE(input + 4);
   s2 = LOAD32LE(input + 8);
   s3 = LOAD32LE(input + 12);

   //Initial round key addition
   s0 ^= context->ek[0];
   s1 ^= context->ek[1];
   s2 ^= context->ek[2];
   s3 ^= context->ek[3];

   //The number of rounds depends on the key length
   for(i = 1; i < context->nr; i++)
   {
      //Apply round function
      t0 = te[s0 & 0xFF];
      temp = te[(s1 >> 8) & 0xFF];
      t0 ^= ROL32(temp, 8);
      temp = te[(s2 >> 16) & 0xFF];
      t0 ^= ROL32(temp, 16);
      temp = te[(s3 >> 24) & 0xFF];
      t0 ^= ROL32(temp, 24);

      t1 = te[s1 & 0xFF];
      temp = te[(s2 >> 8) & 0xFF];
      t1 ^= ROL32(temp, 8);
      temp = te[(s3 >> 16) & 0xFF];
      t1 ^= ROL32(temp, 16);
      temp = te[(s0 >> 24) & 0xFF];
      t1 ^= ROL32(temp, 24);

      t2 = te[s2 & 0xFF];
      temp = te[(s3 >> 8) & 0xFF];
      t2 ^= ROL32(temp, 8);
      temp = te[(s0 >> 16) & 0xFF];
      t2 ^= ROL32(temp, 16);
      temp = te[(s1 >> 24) & 0xFF];
      t2 ^= ROL32(temp, 24);

      t3 = te[s3 & 0xFF];
      temp = te[(s0 >> 8) & 0xFF];
      t3 ^= ROL32(temp, 8);
      temp = te[(s1 >> 16) & 0xFF];
      t3 ^= ROL32(temp, 16);
      temp = te[(s2 >> 24) & 0xFF];
      t3 ^= ROL32(temp, 24);

      //Round key addition
      s0 = t0 ^ context->ek[i * 4];
      s1 = t1 ^ context->ek[i * 4 + 1];
      s2 = t2 ^ context->ek[i * 4 + 2];
      s3 = t3 ^ context->ek[i * 4 + 3];
   }

   //The last round differs slightly from the first rounds
   t0 = sbox[s0 & 0xFF];
   t0 |= sbox[(s1 >> 8) & 0xFF] << 8;
   t0 |= sbox[(s2 >> 16) & 0xFF] << 16;
   t0 |= sbox[(s3 >> 24) & 0xFF] << 24;

   t1 = sbox[s1 & 0xFF];
   t1 |= sbox[(s2 >> 8) & 0xFF] << 8;
   t1 |= sbox[(s3 >> 16) & 0xFF] << 16;
   t1 |= sbox[(s0 >> 24) & 0xFF] << 24;

   t2 = sbox[s2 & 0xFF];
   t2 |= sbox[(s3 >> 8) & 0xFF] << 8;
   t2 |= sbox[(s0 >> 16) & 0xFF] << 16;
   t2 |= sbox[(s1 >> 24) & 0xFF] << 24;

   t3 = sbox[s3 & 0xFF];
   t3 |= sbox[(s0 >> 8) & 0xFF] << 8;
   t3 |= sbox[(s1 >> 16) & 0xFF] << 16;
   t3 |= sbox[(s2 >> 24) & 0xFF] << 24;

   //Last round key addition
   s0 = t0 ^ context->ek[context->nr * 4];
   s1 = t1 ^ context->ek[context->nr * 4 + 1];
   s2 = t2 ^ context->ek[context->nr * 4 + 2];
   s3 = t3 ^ context->ek[context->nr * 4 + 3];

   //The final state is then copied to the output
   STORE32LE(s0, output + 0);
   STORE32LE(s1, output + 4);
   STORE32LE(s2, output + 8);
   STORE32LE(s3, output + 12);
}


/**
 * @brief Decrypt a 16-byte block using AES algorithm
 * @param[in] context Pointer to the AES context
 * @param[in] input Ciphertext block to decrypt
 * @param[out] output Plaintext block resulting from decryption
 **/
void aesDecryptBlock(AesContext_t *context, const uint8_t *input, uint8_t *output)
{
   uint8_t i;
   uint32_t s0;
   uint32_t s1;
   uint32_t s2;
   uint32_t s3;
   uint32_t t0;
   uint32_t t1;
   uint32_t t2;
   uint32_t t3;
   uint32_t temp;

   //Copy the ciphertext to the state array
   s0 = LOAD32LE(input + 0);
   s1 = LOAD32LE(input + 4);
   s2 = LOAD32LE(input + 8);
   s3 = LOAD32LE(input + 12);

   //Initial round key addition
   s0 ^= context->dk[context->nr * 4];
   s1 ^= context->dk[context->nr * 4 + 1];
   s2 ^= context->dk[context->nr * 4 + 2];
   s3 ^= context->dk[context->nr * 4 + 3];

   //The number of rounds depends on the key length
   for(i = context->nr - 1; i >= 1; i--)
   {
      //Apply round function
      t0 = td[s0 & 0xFF];
      temp = td[(s3 >> 8) & 0xFF];
      t0 ^= ROL32(temp, 8);
      temp = td[(s2 >> 16) & 0xFF];
      t0 ^= ROL32(temp, 16);
      temp = td[(s1 >> 24) & 0xFF];
      t0 ^= ROL32(temp, 24);

      t1 = td[s1 & 0xFF];
      temp = td[(s0 >> 8) & 0xFF];
      t1 ^= ROL32(temp, 8);
      temp = td[(s3 >> 16) & 0xFF];
      t1 ^= ROL32(temp, 16);
      temp = td[(s2 >> 24) & 0xFF];
      t1 ^= ROL32(temp, 24);

      t2 = td[s2 & 0xFF];
      temp = td[(s1 >> 8) & 0xFF];
      t2 ^= ROL32(temp, 8);
      temp = td[(s0 >> 16) & 0xFF];
      t2 ^= ROL32(temp, 16);
      temp = td[(s3 >> 24) & 0xFF];
      t2 ^= ROL32(temp, 24);

      t3 = td[s3 & 0xFF];
      temp = td[(s2 >> 8) & 0xFF];
      t3 ^= ROL32(temp, 8);
      temp = td[(s1 >> 16) & 0xFF];
      t3 ^= ROL32(temp, 16);
      temp = td[(s0 >> 24) & 0xFF];
      t3 ^= ROL32(temp, 24);

      //Round key addition
      s0 = t0 ^ context->dk[i * 4];
      s1 = t1 ^ context->dk[i * 4 + 1];
      s2 = t2 ^ context->dk[i * 4 + 2];
      s3 = t3 ^ context->dk[i * 4 + 3];
   }

   //The last round differs slightly from the first rounds
   t0 = isbox[s0 & 0xFF];
   t0 |= isbox[(s3 >> 8) & 0xFF] << 8;
   t0 |= isbox[(s2 >> 16) & 0xFF] << 16;
   t0 |= isbox[(s1 >> 24) & 0xFF] << 24;

   t1 = isbox[s1 & 0xFF];
   t1 |= isbox[(s0 >> 8) & 0xFF] << 8;
   t1 |= isbox[(s3 >> 16) & 0xFF] << 16;
   t1 |= isbox[(s2 >> 24) & 0xFF] << 24;

   t2 = isbox[s2 & 0xFF];
   t2 |= isbox[(s1 >> 8) & 0xFF] << 8;
   t2 |= isbox[(s0 >> 16) & 0xFF] << 16;
   t2 |= isbox[(s3 >> 24) & 0xFF] << 24;

   t3 = isbox[s3 & 0xFF];
   t3 |= isbox[(s2 >> 8) & 0xFF] << 8;
   t3 |= isbox[(s1 >> 16) & 0xFF] << 16;
   t3 |= isbox[(s0 >> 24) & 0xFF] << 24;

   //Last round key addition
   s0 = t0 ^ context->dk[0];
   s1 = t1 ^ context->dk[1];
   s2 = t2 ^ context->dk[2];
   s3 = t3 ^ context->dk[3];

   //The final state is then copied to the output
   STORE32LE(s0, output + 0);
   STORE32LE(s1, output + 4);
   STORE32LE(s2, output + 8);
   STORE32LE(s3, output + 12);
}
#elif defined(ALGO_ARIA)
//S-box 1
static const uint8_t sb1[256] =
{
   0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5, 0x30, 0x01, 0x67, 0x2B, 0xFE, 0xD7, 0xAB, 0x76,
   0xCA, 0x82, 0xC9, 0x7D, 0xFA, 0x59, 0x47, 0xF0, 0xAD, 0xD4, 0xA2, 0xAF, 0x9C, 0xA4, 0x72, 0xC0,
   0xB7, 0xFD, 0x93, 0x26, 0x36, 0x3F, 0xF7, 0xCC, 0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
   0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A, 0x07, 0x12, 0x80, 0xE2, 0xEB, 0x27, 0xB2, 0x75,
   0x09, 0x83, 0x2C, 0x1A, 0x1B, 0x6E, 0x5A, 0xA0, 0x52, 0x3B, 0xD6, 0xB3, 0x29, 0xE3, 0x2F, 0x84,
   0x53, 0xD1, 0x00, 0xED, 0x20, 0xFC, 0xB1, 0x5B, 0x6A, 0xCB, 0xBE, 0x39, 0x4A, 0x4C, 0x58, 0xCF,
   0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85, 0x45, 0xF9, 0x02, 0x7F, 0x50, 0x3C, 0x9F, 0xA8,
   0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5, 0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2,
   0xCD, 0x0C, 0x13, 0xEC, 0x5F, 0x97, 0x44, 0x17, 0xC4, 0xA7, 0x7E, 0x3D, 0x64, 0x5D, 0x19, 0x73,
   0x60, 0x81, 0x4F, 0xDC, 0x22, 0x2A, 0x90, 0x88, 0x46, 0xEE, 0xB8, 0x14, 0xDE, 0x5E, 0x0B, 0xDB,
   0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C, 0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79,
   0xE7, 0xC8, 0x37, 0x6D, 0x8D, 0xD5, 0x4E, 0xA9, 0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
   0xBA, 0x78, 0x25, 0x2E, 0x1C, 0xA6, 0xB4, 0xC6, 0xE8, 0xDD, 0x74, 0x1F, 0x4B, 0xBD, 0x8B, 0x8A,
   0x70, 0x3E, 0xB5, 0x66, 0x48, 0x03, 0xF6, 0x0E, 0x61, 0x35, 0x57, 0xB9, 0x86, 0xC1, 0x1D, 0x9E,
   0xE1, 0xF8, 0x98, 0x11, 0x69, 0xD9, 0x8E, 0x94, 0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
   0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68, 0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16
};

//S-box 2
static const uint8_t sb2[256] =
{
   0xE2, 0x4E, 0x54, 0xFC, 0x94, 0xC2, 0x4A, 0xCC, 0x62, 0x0D, 0x6A, 0x46, 0x3C, 0x4D, 0x8B, 0xD1,
   0x5E, 0xFA, 0x64, 0xCB, 0xB4, 0x97, 0xBE, 0x2B, 0xBC, 0x77, 0x2E, 0x03, 0xD3, 0x19, 0x59, 0xC1,
   0x1D, 0x06, 0x41, 0x6B, 0x55, 0xF0, 0x99, 0x69, 0xEA, 0x9C, 0x18, 0xAE, 0x63, 0xDF, 0xE7, 0xBB,
   0x00, 0x73, 0x66, 0xFB, 0x96, 0x4C, 0x85, 0xE4, 0x3A, 0x09, 0x45, 0xAA, 0x0F, 0xEE, 0x10, 0xEB,
   0x2D, 0x7F, 0xF4, 0x29, 0xAC, 0xCF, 0xAD, 0x91, 0x8D, 0x78, 0xC8, 0x95, 0xF9, 0x2F, 0xCE, 0xCD,
   0x08, 0x7A, 0x88, 0x38, 0x5C, 0x83, 0x2A, 0x28, 0x47, 0xDB, 0xB8, 0xC7, 0x93, 0xA4, 0x12, 0x53,
   0xFF, 0x87, 0x0E, 0x31, 0x36, 0x21, 0x58, 0x48, 0x01, 0x8E, 0x37, 0x74, 0x32, 0xCA, 0xE9, 0xB1,
   0xB7, 0xAB, 0x0C, 0xD7, 0xC4, 0x56, 0x42, 0x26, 0x07, 0x98, 0x60, 0xD9, 0xB6, 0xB9, 0x11, 0x40,
   0xEC, 0x20, 0x8C, 0xBD, 0xA0, 0xC9, 0x84, 0x04, 0x49, 0x23, 0xF1, 0x4F, 0x50, 0x1F, 0x13, 0xDC,
   0xD8, 0xC0, 0x9E, 0x57, 0xE3, 0xC3, 0x7B, 0x65, 0x3B, 0x02, 0x8F, 0x3E, 0xE8, 0x25, 0x92, 0xE5,
   0x15, 0xDD, 0xFD, 0x17, 0xA9, 0xBF, 0xD4, 0x9A, 0x7E, 0xC5, 0x39, 0x67, 0xFE, 0x76, 0x9D, 0x43,
   0xA7, 0xE1, 0xD0, 0xF5, 0x68, 0xF2, 0x1B, 0x34, 0x70, 0x05, 0xA3, 0x8A, 0xD5, 0x79, 0x86, 0xA8,
   0x30, 0xC6, 0x51, 0x4B, 0x1E, 0xA6, 0x27, 0xF6, 0x35, 0xD2, 0x6E, 0x24, 0x16, 0x82, 0x5F, 0xDA,
   0xE6, 0x75, 0xA2, 0xEF, 0x2C, 0xB2, 0x1C, 0x9F, 0x5D, 0x6F, 0x80, 0x0A, 0x72, 0x44, 0x9B, 0x6C,
   0x90, 0x0B, 0x5B, 0x33, 0x7D, 0x5A, 0x52, 0xF3, 0x61, 0xA1, 0xF7, 0xB0, 0xD6, 0x3F, 0x7C, 0x6D,
   0xED, 0x14, 0xE0, 0xA5, 0x3D, 0x22, 0xB3, 0xF8, 0x89, 0xDE, 0x71, 0x1A, 0xAF, 0xBA, 0xB5, 0x81
};

//S-box 3
static const uint8_t sb3[256] =
{
   0x52, 0x09, 0x6A, 0xD5, 0x30, 0x36, 0xA5, 0x38, 0xBF, 0x40, 0xA3, 0x9E, 0x81, 0xF3, 0xD7, 0xFB,
   0x7C, 0xE3, 0x39, 0x82, 0x9B, 0x2F, 0xFF, 0x87, 0x34, 0x8E, 0x43, 0x44, 0xC4, 0xDE, 0xE9, 0xCB,
   0x54, 0x7B, 0x94, 0x32, 0xA6, 0xC2, 0x23, 0x3D, 0xEE, 0x4C, 0x95, 0x0B, 0x42, 0xFA, 0xC3, 0x4E,
   0x08, 0x2E, 0xA1, 0x66, 0x28, 0xD9, 0x24, 0xB2, 0x76, 0x5B, 0xA2, 0x49, 0x6D, 0x8B, 0xD1, 0x25,
   0x72, 0xF8, 0xF6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xD4, 0xA4, 0x5C, 0xCC, 0x5D, 0x65, 0xB6, 0x92,
   0x6C, 0x70, 0x48, 0x50, 0xFD, 0xED, 0xB9, 0xDA, 0x5E, 0x15, 0x46, 0x57, 0xA7, 0x8D, 0x9D, 0x84,
   0x90, 0xD8, 0xAB, 0x00, 0x8C, 0xBC, 0xD3, 0x0A, 0xF7, 0xE4, 0x58, 0x05, 0xB8, 0xB3, 0x45, 0x06,
   0xD0, 0x2C, 0x1E, 0x8F, 0xCA, 0x3F, 0x0F, 0x02, 0xC1, 0xAF, 0xBD, 0x03, 0x01, 0x13, 0x8A, 0x6B,
   0x3A, 0x91, 0x11, 0x41, 0x4F, 0x67, 0xDC, 0xEA, 0x97, 0xF2, 0xCF, 0xCE, 0xF0, 0xB4, 0xE6, 0x73,
   0x96, 0xAC, 0x74, 0x22, 0xE7, 0xAD, 0x35, 0x85, 0xE2, 0xF9, 0x37, 0xE8, 0x1C, 0x75, 0xDF, 0x6E,
   0x47, 0xF1, 0x1A, 0x71, 0x1D, 0x29, 0xC5, 0x89, 0x6F, 0xB7, 0x62, 0x0E, 0xAA, 0x18, 0xBE, 0x1B,
   0xFC, 0x56, 0x3E, 0x4B, 0xC6, 0xD2, 0x79, 0x20, 0x9A, 0xDB, 0xC0, 0xFE, 0x78, 0xCD, 0x5A, 0xF4,
   0x1F, 0xDD, 0xA8, 0x33, 0x88, 0x07, 0xC7, 0x31, 0xB1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xEC, 0x5F,
   0x60, 0x51, 0x7F, 0xA9, 0x19, 0xB5, 0x4A, 0x0D, 0x2D, 0xE5, 0x7A, 0x9F, 0x93, 0xC9, 0x9C, 0xEF,
   0xA0, 0xE0, 0x3B, 0x4D, 0xAE, 0x2A, 0xF5, 0xB0, 0xC8, 0xEB, 0xBB, 0x3C, 0x83, 0x53, 0x99, 0x61,
   0x17, 0x2B, 0x04, 0x7E, 0xBA, 0x77, 0xD6, 0x26, 0xE1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0C, 0x7D
};

//S-box 4
static const uint8_t sb4[256] =
{
   0x30, 0x68, 0x99, 0x1B, 0x87, 0xB9, 0x21, 0x78, 0x50, 0x39, 0xDB, 0xE1, 0x72, 0x09, 0x62, 0x3C,
   0x3E, 0x7E, 0x5E, 0x8E, 0xF1, 0xA0, 0xCC, 0xA3, 0x2A, 0x1D, 0xFB, 0xB6, 0xD6, 0x20, 0xC4, 0x8D,
   0x81, 0x65, 0xF5, 0x89, 0xCB, 0x9D, 0x77, 0xC6, 0x57, 0x43, 0x56, 0x17, 0xD4, 0x40, 0x1A, 0x4D,
   0xC0, 0x63, 0x6C, 0xE3, 0xB7, 0xC8, 0x64, 0x6A, 0x53, 0xAA, 0x38, 0x98, 0x0C, 0xF4, 0x9B, 0xED,
   0x7F, 0x22, 0x76, 0xAF, 0xDD, 0x3A, 0x0B, 0x58, 0x67, 0x88, 0x06, 0xC3, 0x35, 0x0D, 0x01, 0x8B,
   0x8C, 0xC2, 0xE6, 0x5F, 0x02, 0x24, 0x75, 0x93, 0x66, 0x1E, 0xE5, 0xE2, 0x54, 0xD8, 0x10, 0xCE,
   0x7A, 0xE8, 0x08, 0x2C, 0x12, 0x97, 0x32, 0xAB, 0xB4, 0x27, 0x0A, 0x23, 0xDF, 0xEF, 0xCA, 0xD9,
   0xB8, 0xFA, 0xDC, 0x31, 0x6B, 0xD1, 0xAD, 0x19, 0x49, 0xBD, 0x51, 0x96, 0xEE, 0xE4, 0xA8, 0x41,
   0xDA, 0xFF, 0xCD, 0x55, 0x86, 0x36, 0xBE, 0x61, 0x52, 0xF8, 0xBB, 0x0E, 0x82, 0x48, 0x69, 0x9A,
   0xE0, 0x47, 0x9E, 0x5C, 0x04, 0x4B, 0x34, 0x15, 0x79, 0x26, 0xA7, 0xDE, 0x29, 0xAE, 0x92, 0xD7,
   0x84, 0xE9, 0xD2, 0xBA, 0x5D, 0xF3, 0xC5, 0xB0, 0xBF, 0xA4, 0x3B, 0x71, 0x44, 0x46, 0x2B, 0xFC,
   0xEB, 0x6F, 0xD5, 0xF6, 0x14, 0xFE, 0x7C, 0x70, 0x5A, 0x7D, 0xFD, 0x2F, 0x18, 0x83, 0x16, 0xA5,
   0x91, 0x1F, 0x05, 0x95, 0x74, 0xA9, 0xC1, 0x5B, 0x4A, 0x85, 0x6D, 0x13, 0x07, 0x4F, 0x4E, 0x45,
   0xB2, 0x0F, 0xC9, 0x1C, 0xA6, 0xBC, 0xEC, 0x73, 0x90, 0x7B, 0xCF, 0x59, 0x8F, 0xA1, 0xF9, 0x2D,
   0xF2, 0xB1, 0x00, 0x94, 0x37, 0x9F, 0xD0, 0x2E, 0x9C, 0x6E, 0x28, 0x3F, 0x80, 0xF0, 0x3D, 0xD3,
   0x25, 0x8A, 0xB5, 0xE7, 0x42, 0xB3, 0xC7, 0xEA, 0xF7, 0x4C, 0x11, 0x33, 0x03, 0xA2, 0xAC, 0x60
};

//Key scheduling constants
static const uint32_t c[12] =
{
   BETOH32(0x517CC1B7), BETOH32(0x27220A94), BETOH32(0xFE13ABE8), BETOH32(0xFA9A6EE0),
   BETOH32(0x6DB14ACC), BETOH32(0x9E21C820), BETOH32(0xFF28B1D5), BETOH32(0xEF5DE2B0),
   BETOH32(0xDB92371D), BETOH32(0x2126E970), BETOH32(0x03249775), BETOH32(0x04E8C90E)
};

//Common interface for encryption algorithms
const CipherAlgo_t ariaCipherAlgo =
{
   "ARIA",
   sizeof(AriaContext_t),
   CIPHER_ALGO_TYPE_BLOCK,
   ARIA_BLOCK_SIZE,
   (CipherAlgoInit) ariaInit,
//   NULL,
//   NULL,
   (CipherAlgoEncryptBlock) ariaEncryptBlock,
   (CipherAlgoDecryptBlock) ariaDecryptBlock
};


/**
 * @brief Odd round function
 * @param[in,out] d 128-bit string
 * @param[in] rk 128-bit string
 **/
static void OF(uint32_t *d, const uint32_t *rk)
{
   uint32_t t[4u];

   XOR128(d, rk);                                                               //XOR D with RK
   SL1(t, d);                                                                   //Substitution layer SL1
   A(d, t);                                                                     //Diffusion layer
}


/**
 * @brief Even round function
 * @param[in,out] d 128-bit string
 * @param[in] rk 128-bit string
 **/
static void EF(uint32_t *d, const uint32_t *rk)
{
   uint32_t t[4u];

   XOR128(d, rk);                                                               //XOR D with RK
   SL2(t, d);                                                                   //Substitution layer SL2
   A(d, t);                                                                     //Diffusion layer
}


/**
 * @brief Initialize a ARIA context using the supplied key
 * @param[in] context Pointer to the ARIA context to initialize
 * @param[in] key Pointer to the key
 * @param[in] keyLen Length of the key
 * @return Error code
 **/
uint8_t ariaInit(AriaContext_t *context, const uint8_t *key, size_t keyLen)
{
   uint8_t i;
   uint32_t *ek;
   uint32_t *dk;
   const uint32_t *ck1;
   const uint32_t *ck2;
   const uint32_t *ck3;
   uint32_t w[16];


   if(context == NULL || key == NULL)                                           //Check parameters
      return 0u;

    *ek= 0ul;

   if(keyLen == 16)                                                             //Check the length of the master key
   {
      ck1 = c + 0;                                                              //Select the relevant constants
      ck2 = c + 4;
      ck3 = c + 8;
      context->nr = 12;                                                         //The number of rounds depends on the size of the master key
   }
   else if(keyLen == 24)
   {
      ck1 = c + 4;                                                              //Select the relevant constants
      ck2 = c + 8;
      ck3 = c + 0;
      context->nr = 14;                                                         //The number of rounds depends on the size of the master key
   }
   else if(keyLen == 32)
   {
      ck1 = c + 8;                                                              //Select the relevant constants
      ck2 = c + 0;
      ck3 = c + 4;
      context->nr = 16;                                                         //The number of rounds depends on the size of the master key
   }
   else
   {
      return 2u;                                                                //Report an error
   }

   memset((void*)w,(void*) 0, sizeof(w));                                       //Compute 128-bit values KL and KR
   memcpy((void*)w,(void*)key,(int16_t) keyLen);

   MOV128(w + 8, w + 4);                                                        //Save KR...
   MOV128(w + 4, w + 0);                                                        //Compute intermediate values W0, W1, W2, and W3
   OF(w + 4, ck1);
   XOR128(w + 4, w + 8);

   MOV128(w + 8, w + 4);
   EF(w + 8, ck2);
   XOR128(w + 8, w + 0);

   MOV128(w + 12, w + 8);
   OF(w + 12, ck3);
   XOR128(w + 12, w + 4);

   for(i = 0; i < 16; i++)                                                      //Convert from big-endian byte order to host byte order
   {
      w[i] = betoh32(w[i]);
   }
   ek = context->ek;                                                            //Point to the encryption round keys
   ROL128(ek + 0, w + 4, 109);                                                  //Compute ek1, ..., ek17 as follow
   XOR128(ek + 0, w + 0);
   ROL128(ek + 4, w + 8, 109);
   XOR128(ek + 4, w + 4);
   ROL128(ek + 8, w + 12, 109);
   XOR128(ek + 8, w + 8);
   ROL128(ek + 12, w + 0, 109);
   XOR128(ek + 12, w + 12);
   ROL128(ek + 16, w + 4, 97);
   XOR128(ek + 16, w + 0);
   ROL128(ek + 20, w + 8, 97);
   XOR128(ek + 20, w + 4);
   ROL128(ek + 24, w + 12, 97);
   XOR128(ek + 24, w + 8);
   ROL128(ek + 28, w + 0, 97);
   XOR128(ek + 28, w + 12);
   ROL128(ek + 32, w + 4, 61);
   XOR128(ek + 32, w + 0);
   ROL128(ek + 36, w + 8, 61);
   XOR128(ek + 36, w + 4);
   ROL128(ek + 40, w + 12, 61);
   XOR128(ek + 40, w + 8);
   ROL128(ek + 44, w + 0, 61);
   XOR128(ek + 44, w + 12);
   ROL128(ek + 48, w + 4, 31);
   XOR128(ek + 48, w + 0);
   ROL128(ek + 52, w + 8, 31);
   XOR128(ek + 52, w + 4);
   ROL128(ek + 56, w + 12, 31);
   XOR128(ek + 56, w + 8);
   ROL128(ek + 60, w + 0, 31);
   XOR128(ek + 60, w + 12);
   ROL128(ek + 64, w + 4, 19);
   XOR128(ek + 64, w + 0);
   for(i = 0; i < 68; i++)                                                      //Convert from host byte order to big-endian byte order
   {
      ek[i] = htobe32(ek[i]);
   }
   dk = context->dk;                                                            //Decryption round keys are derived from the encryption round keys
   MOV128(dk + 0, ek + context->nr * 4);                                        //Compute dk1
   for(i = 1; i < context->nr; i++)                                             //Compute dk2, ..., dk(n)
   {
      A(dk + i * 4, ek + (context->nr - i) * 4);
   }
   MOV128(dk + i * 4, ek + 0);                                                  //Compute dk(n + 1)
   return 1u;                                                                   //No error to report
}

/**
 * @brief Encrypt a 16-byte block using ARIA algorithm
 * @param[in] context Pointer to the ARIA context
 * @param[in] input Plaintext block to encrypt
 * @param[out] output Ciphertext block resulting from encryption
 **/
void ariaEncryptBlock(AriaContext_t *context, const uint8_t *input, uint8_t *output)
{
   uint32_t *ek;
   uint32_t p[4];
   uint32_t q[4];

   memcpy((void*)p,(void*) input,(int16_t) ARIA_BLOCK_SIZE);                    //Copy the plaintext to the buffer

   ek = context->ek;                                                            //Point to the encryption round keys
   OF(p, ek + 0);                                                               //Apply 11 rounds
   EF(p, ek + 4);
   OF(p, ek + 8);
   EF(p, ek + 12);
   OF(p, ek + 16);
   EF(p, ek + 20);
   OF(p, ek + 24);
   EF(p, ek + 28);
   OF(p, ek + 32);
   EF(p, ek + 36);
   OF(p, ek + 40);

   if(context->nr == 12)                                                        //128-bit master keys require a total of 12 rounds
   {
      XOR128(p, ek + 44);
      SL2(q, p);
      XOR128(q, ek + 48);
   }
   else if(context->nr == 14)                                                   //192-bit master keys require a total of 14 rounds
   {
      EF(p, ek + 44);
      OF(p, ek + 48);
      XOR128(p, ek + 52);
      SL2(q, p);
      XOR128(q, ek + 56);
   }
   else                                                                         //256-bit master keys require a total of 16 rounds
   {
      EF(p, ek + 44);
      OF(p, ek + 48);
      EF(p, ek + 52);
      OF(p, ek + 56);
      XOR128(p, ek + 60);
      SL2(q, p);
      XOR128(q, ek + 64);
   }

   memcpy((void*)output,(void*) q,(int16_t) ARIA_BLOCK_SIZE);                   //Copy the resulting ciphertext from the buffer
}


/**
 * @brief Decrypt a 16-byte block using ARIA algorithm
 * @param[in] context Pointer to the ARIA context
 * @param[in] input Ciphertext block to decrypt
 * @param[out] output Plaintext block resulting from decryption
 **/
void ariaDecryptBlock(AriaContext_t *context, const uint8_t *input, uint8_t *output)
{
   uint32_t *dk;
   uint32_t p[4];
   uint32_t q[4];

   memcpy((void*)p,(void*) input,(int16_t) ARIA_BLOCK_SIZE);                    //Copy the ciphertext to the buffer
   dk = context->dk;                                                            //Point to the decryption round keys
   OF(p, dk + 0);                                                               //Apply 11 rounds
   EF(p, dk + 4);
   OF(p, dk + 8);
   EF(p, dk + 12);
   OF(p, dk + 16);
   EF(p, dk + 20);
   OF(p, dk + 24);
   EF(p, dk + 28);
   OF(p, dk + 32);
   EF(p, dk + 36);
   OF(p, dk + 40);

   if(context->nr == 12)                                                        //128-bit master keys require a total of 12 rounds
   {
      XOR128(p, dk + 44);
      SL2(q, p);
      XOR128(q, dk + 48);
   }
   else if(context->nr == 14)                                                   //192-bit master keys require a total of 14 rounds
   {
      EF(p, dk + 44);
      OF(p, dk + 48);
      XOR128(p, dk + 52);
      SL2(q, p);
      XOR128(q, dk + 56);
   }
   else                                                                         //256-bit master keys require a total of 16 rounds
   {
      EF(p, dk + 44);
      OF(p, dk + 48);
      EF(p, dk + 52);
      OF(p, dk + 56);
      XOR128(p, dk + 60);
      SL2(q, p);
      XOR128(q, dk + 64);
   }
   memcpy((void*)output,(void*) q,(int16_t) ARIA_BLOCK_SIZE);                   //The resulting value is the plaintext
}
#elif defined(ALGO_SALSA20)
/**
 * @brief Salsa20 core function
 * @param[in] input Pointer to the 64-octet input block
 * @param[out] output Pointer to the 64-octet output block
 * @param[in] nr Number of rounds to be applied (8, 12 or 20)
 **/
void salsa20ProcessBlock(const uint8_t *input, uint8_t *output, uint8_t nr)
{
   uint8_t i;
   uint32_t x[16];

   for(i = 0; i < 16; i++)                                                      //Copy the input words to the working state
   {
      x[i] = LOAD32LE(input + i * 4);
   }

   for(i = 0; i < nr; i += 2)                                                   //The Salsa20 core function alternates between column rounds and row rounds
   {
      //The column round function modifies the columns of the matrix in parallel
      //by feeding a permutation of each column through the quarter round function
      SALSA20_QUARTER_ROUND(x[0], x[4], x[8], x[12]);
      SALSA20_QUARTER_ROUND(x[5], x[9], x[13], x[1]);
      SALSA20_QUARTER_ROUND(x[10], x[14], x[2], x[6]);
      SALSA20_QUARTER_ROUND(x[15], x[3], x[7], x[11]);

      //The row round function modifies the rows of the matrix in parallel by
      //feeding a permutation of each row through the quarter round function
      SALSA20_QUARTER_ROUND(x[0], x[1], x[2], x[3]);
      SALSA20_QUARTER_ROUND(x[5], x[6], x[7], x[4]);
      SALSA20_QUARTER_ROUND(x[10], x[11], x[8], x[9]);
      SALSA20_QUARTER_ROUND(x[15], x[12], x[13], x[14]);
   }

   for(i = 0; i < 16; i++)                                                      //Add the original input words to the output words
   {
      x[i] += LOAD32LE(input + i * 4);
      STORE32LE(x[i], output + i * 4);
   }
}
#elif defined(ALGO_DES)
//Selection function 1
static const uint32_t sp1[64] =
{
   0x01010400, 0x00000000, 0x00010000, 0x01010404, 0x01010004, 0x00010404, 0x00000004, 0x00010000,
   0x00000400, 0x01010400, 0x01010404, 0x00000400, 0x01000404, 0x01010004, 0x01000000, 0x00000004,
   0x00000404, 0x01000400, 0x01000400, 0x00010400, 0x00010400, 0x01010000, 0x01010000, 0x01000404,
   0x00010004, 0x01000004, 0x01000004, 0x00010004, 0x00000000, 0x00000404, 0x00010404, 0x01000000,
   0x00010000, 0x01010404, 0x00000004, 0x01010000, 0x01010400, 0x01000000, 0x01000000, 0x00000400,
   0x01010004, 0x00010000, 0x00010400, 0x01000004, 0x00000400, 0x00000004, 0x01000404, 0x00010404,
   0x01010404, 0x00010004, 0x01010000, 0x01000404, 0x01000004, 0x00000404, 0x00010404, 0x01010400,
   0x00000404, 0x01000400, 0x01000400, 0x00000000, 0x00010004, 0x00010400, 0x00000000, 0x01010004
};

//Selection function 2
static const uint32_t sp2[64] =
{
   0x80108020, 0x80008000, 0x00008000, 0x00108020, 0x00100000, 0x00000020, 0x80100020, 0x80008020,
   0x80000020, 0x80108020, 0x80108000, 0x80000000, 0x80008000, 0x00100000, 0x00000020, 0x80100020,
   0x00108000, 0x00100020, 0x80008020, 0x00000000, 0x80000000, 0x00008000, 0x00108020, 0x80100000,
   0x00100020, 0x80000020, 0x00000000, 0x00108000, 0x00008020, 0x80108000, 0x80100000, 0x00008020,
   0x00000000, 0x00108020, 0x80100020, 0x00100000, 0x80008020, 0x80100000, 0x80108000, 0x00008000,
   0x80100000, 0x80008000, 0x00000020, 0x80108020, 0x00108020, 0x00000020, 0x00008000, 0x80000000,
   0x00008020, 0x80108000, 0x00100000, 0x80000020, 0x00100020, 0x80008020, 0x80000020, 0x00100020,
   0x00108000, 0x00000000, 0x80008000, 0x00008020, 0x80000000, 0x80100020, 0x80108020, 0x00108000
};

//Selection function 3
static const uint32_t sp3[64] =
{
   0x00000208, 0x08020200, 0x00000000, 0x08020008, 0x08000200, 0x00000000, 0x00020208, 0x08000200,
   0x00020008, 0x08000008, 0x08000008, 0x00020000, 0x08020208, 0x00020008, 0x08020000, 0x00000208,
   0x08000000, 0x00000008, 0x08020200, 0x00000200, 0x00020200, 0x08020000, 0x08020008, 0x00020208,
   0x08000208, 0x00020200, 0x00020000, 0x08000208, 0x00000008, 0x08020208, 0x00000200, 0x08000000,
   0x08020200, 0x08000000, 0x00020008, 0x00000208, 0x00020000, 0x08020200, 0x08000200, 0x00000000,
   0x00000200, 0x00020008, 0x08020208, 0x08000200, 0x08000008, 0x00000200, 0x00000000, 0x08020008,
   0x08000208, 0x00020000, 0x08000000, 0x08020208, 0x00000008, 0x00020208, 0x00020200, 0x08000008,
   0x08020000, 0x08000208, 0x00000208, 0x08020000, 0x00020208, 0x00000008, 0x08020008, 0x00020200
};

//Selection function 4
static const uint32_t sp4[64] =
{
   0x00802001, 0x00002081, 0x00002081, 0x00000080, 0x00802080, 0x00800081, 0x00800001, 0x00002001,
   0x00000000, 0x00802000, 0x00802000, 0x00802081, 0x00000081, 0x00000000, 0x00800080, 0x00800001,
   0x00000001, 0x00002000, 0x00800000, 0x00802001, 0x00000080, 0x00800000, 0x00002001, 0x00002080,
   0x00800081, 0x00000001, 0x00002080, 0x00800080, 0x00002000, 0x00802080, 0x00802081, 0x00000081,
   0x00800080, 0x00800001, 0x00802000, 0x00802081, 0x00000081, 0x00000000, 0x00000000, 0x00802000,
   0x00002080, 0x00800080, 0x00800081, 0x00000001, 0x00802001, 0x00002081, 0x00002081, 0x00000080,
   0x00802081, 0x00000081, 0x00000001, 0x00002000, 0x00800001, 0x00002001, 0x00802080, 0x00800081,
   0x00002001, 0x00002080, 0x00800000, 0x00802001, 0x00000080, 0x00800000, 0x00002000, 0x00802080
};

//Selection function 5
static const uint32_t sp5[64] =
{
   0x00000100, 0x02080100, 0x02080000, 0x42000100, 0x00080000, 0x00000100, 0x40000000, 0x02080000,
   0x40080100, 0x00080000, 0x02000100, 0x40080100, 0x42000100, 0x42080000, 0x00080100, 0x40000000,
   0x02000000, 0x40080000, 0x40080000, 0x00000000, 0x40000100, 0x42080100, 0x42080100, 0x02000100,
   0x42080000, 0x40000100, 0x00000000, 0x42000000, 0x02080100, 0x02000000, 0x42000000, 0x00080100,
   0x00080000, 0x42000100, 0x00000100, 0x02000000, 0x40000000, 0x02080000, 0x42000100, 0x40080100,
   0x02000100, 0x40000000, 0x42080000, 0x02080100, 0x40080100, 0x00000100, 0x02000000, 0x42080000,
   0x42080100, 0x00080100, 0x42000000, 0x42080100, 0x02080000, 0x00000000, 0x40080000, 0x42000000,
   0x00080100, 0x02000100, 0x40000100, 0x00080000, 0x00000000, 0x40080000, 0x02080100, 0x40000100
};

//Selection function 6
static const uint32_t sp6[64] =
{
   0x20000010, 0x20400000, 0x00004000, 0x20404010, 0x20400000, 0x00000010, 0x20404010, 0x00400000,
   0x20004000, 0x00404010, 0x00400000, 0x20000010, 0x00400010, 0x20004000, 0x20000000, 0x00004010,
   0x00000000, 0x00400010, 0x20004010, 0x00004000, 0x00404000, 0x20004010, 0x00000010, 0x20400010,
   0x20400010, 0x00000000, 0x00404010, 0x20404000, 0x00004010, 0x00404000, 0x20404000, 0x20000000,
   0x20004000, 0x00000010, 0x20400010, 0x00404000, 0x20404010, 0x00400000, 0x00004010, 0x20000010,
   0x00400000, 0x20004000, 0x20000000, 0x00004010, 0x20000010, 0x20404010, 0x00404000, 0x20400000,
   0x00404010, 0x20404000, 0x00000000, 0x20400010, 0x00000010, 0x00004000, 0x20400000, 0x00404010,
   0x00004000, 0x00400010, 0x20004010, 0x00000000, 0x20404000, 0x20000000, 0x00400010, 0x20004010
};

//Selection function 7
static const uint32_t sp7[64] =
{
   0x00200000, 0x04200002, 0x04000802, 0x00000000, 0x00000800, 0x04000802, 0x00200802, 0x04200800,
   0x04200802, 0x00200000, 0x00000000, 0x04000002, 0x00000002, 0x04000000, 0x04200002, 0x00000802,
   0x04000800, 0x00200802, 0x00200002, 0x04000800, 0x04000002, 0x04200000, 0x04200800, 0x00200002,
   0x04200000, 0x00000800, 0x00000802, 0x04200802, 0x00200800, 0x00000002, 0x04000000, 0x00200800,
   0x04000000, 0x00200800, 0x00200000, 0x04000802, 0x04000802, 0x04200002, 0x04200002, 0x00000002,
   0x00200002, 0x04000000, 0x04000800, 0x00200000, 0x04200800, 0x00000802, 0x00200802, 0x04200800,
   0x00000802, 0x04000002, 0x04200802, 0x04200000, 0x00200800, 0x00000000, 0x00000002, 0x04200802,
   0x00000000, 0x00200802, 0x04200000, 0x00000800, 0x04000002, 0x04000800, 0x00000800, 0x00200002
};

//Selection function 8
static const uint32_t sp8[64] =
{
   0x10001040, 0x00001000, 0x00040000, 0x10041040, 0x10000000, 0x10001040, 0x00000040, 0x10000000,
   0x00040040, 0x10040000, 0x10041040, 0x00041000, 0x10041000, 0x00041040, 0x00001000, 0x00000040,
   0x10040000, 0x10000040, 0x10001000, 0x00001040, 0x00041000, 0x00040040, 0x10040040, 0x10041000,
   0x00001040, 0x00000000, 0x00000000, 0x10040040, 0x10000040, 0x10001000, 0x00041040, 0x00040000,
   0x00041040, 0x00040000, 0x10041000, 0x00001000, 0x00000040, 0x10040040, 0x00001000, 0x00041040,
   0x10001000, 0x00000040, 0x10000040, 0x10040000, 0x10040040, 0x10000000, 0x00040000, 0x10001040,
   0x00000000, 0x10041040, 0x00040040, 0x10000040, 0x10040000, 0x10001000, 0x10001040, 0x00000000,
   0x10041040, 0x00041000, 0x00041000, 0x00001040, 0x00001040, 0x00040040, 0x10000000, 0x10041000
};

//Common interface for encryption algorithms
/**
 * @brief DES algorithm context
 **/
typedef struct
{
   uint32_t ks[32u];
} DesContext_t;

const CipherAlgo_t desCipherAlgo =
{
   "DES",
   sizeof(DesContext_t),
   CIPHER_ALGO_TYPE_BLOCK,
   DES_BLOCK_SIZE,
   (CipherAlgoInit) desInit,
//   NULL,
//   NULL,
   (CipherAlgoEncryptBlock) desEncryptBlock,
   (CipherAlgoDecryptBlock) desDecryptBlock
};
/**
 * @brief Initialize a DES context using the supplied key
 * @param[in] context Pointer to the DES context to initialize
 * @param[in] key Pointer to the key
 * @param[in] keyLen Length of the key (must be set to 8)
 * @return Error code
 **/
uint8_t desInit(DesContext_t *context, const uint8_t *key, size_t keyLen)
{
   uint8_t i;
   uint32_t c;
   uint32_t d;

   if(context == NULL || key == NULL)                                           //Check parameters
      return 0u;

   if(keyLen != 8)                                                              //Invalid key length?
      return 2u;
   c = LOAD32BE(key + 0);                                                       //Copy the key
   d = LOAD32BE(key + 4);
   DES_PC1(c, d);                                                               //Permuted choice 1

   for(i = 0; i < 16; i++)                                                      //Generate the key schedule
   {
      if(i == 0 || i == 1 || i == 8 || i == 15)                                 //Individual blocks are shifted left
      {
         c = ROL28(c, 1);
         d = ROL28(d, 1);
      }
      else
      {
         c = ROL28(c, 2);
         d = ROL28(d, 2);
      }
      context->ks[2 * i] =                                                      //Permuted choice 2
         ((c << 4)  & 0x24000000) | ((c << 28) & 0x10000000) |
         ((c << 14) & 0x08000000) | ((c << 18) & 0x02080000) |
         ((c << 6)  & 0x01000000) | ((c << 9)  & 0x00200000) |
         ((c >> 1)  & 0x00100000) | ((c << 10) & 0x00040000) |
         ((c << 2)  & 0x00020000) | ((c >> 10) & 0x00010000) |
         ((d >> 13) & 0x00002000) | ((d >> 4)  & 0x00001000) |
         ((d << 6)  & 0x00000800) | ((d >> 1)  & 0x00000400) |
         ((d >> 14) & 0x00000200) | ((d)       & 0x00000100) |
         ((d >> 5)  & 0x00000020) | ((d >> 10) & 0x00000010) |
         ((d >> 3)  & 0x00000008) | ((d >> 18) & 0x00000004) |
         ((d >> 26) & 0x00000002) | ((d >> 24) & 0x00000001);

      context->ks[2 * i + 1] =
         ((c << 15) & 0x20000000) | ((c << 17) & 0x10000000) |
         ((c << 10) & 0x08000000) | ((c << 22) & 0x04000000) |
         ((c >> 2)  & 0x02000000) | ((c << 1)  & 0x01000000) |
         ((c << 16) & 0x00200000) | ((c << 11) & 0x00100000) |
         ((c << 3)  & 0x00080000) | ((c >> 6)  & 0x00040000) |
         ((c << 15) & 0x00020000) | ((c >> 4)  & 0x00010000) |
         ((d >> 2)  & 0x00002000) | ((d << 8)  & 0x00001000) |
         ((d >> 14) & 0x00000808) | ((d >> 9)  & 0x00000400) |
         ((d)       & 0x00000200) | ((d << 7)  & 0x00000100) |
         ((d >> 7)  & 0x00000020) | ((d >> 3)  & 0x00000011) |
         ((d << 2)  & 0x00000004) | ((d >> 21) & 0x00000002);
   }
   return 1u;                                                                   //No error to report
}


/**
 * @brief Encrypt a 8-byte block using DES algorithm
 * @param[in] context Pointer to the DES context
 * @param[in] input Plaintext block to encrypt
 * @param[out] output Ciphertext block resulting from encryption
 **/
void desEncryptBlock(DesContext_t *context, const uint8_t *input, uint8_t *output)
{
   uint8_t i;
   uint32_t left;
   uint32_t right;
   uint32_t temp;

   uint32_t *ks = context->ks;                                                  //Key schedule

   left = LOAD32BE(input + 0);                                                  //Copy the plaintext from the input buffer
   right = LOAD32BE(input + 4);

   DES_IP(left, right);                                                         //Initial permutation

   for(i = 0; i < 16; i++, ks += 2)                                             //16 rounds of computation are needed
   {
      DES_ROUND(left, right, ks);
   }
   DES_FP(right, left);                                                         //Inverse IP permutation
   STORE32BE(right, output + 0);                                                //Copy the resulting ciphertext
   STORE32BE(left, output + 4);
}
/**
 * @brief Decrypt a 8-byte block using DES algorithm
 * @param[in] context Pointer to the DES context
 * @param[in] input Ciphertext block to decrypt
 * @param[out] output Plaintext block resulting from decryption
 **/
void desDecryptBlock(DesContext_t *context, const uint8_t *input, uint8_t *output)
{
   uint8_t i;
   uint32_t left;
   uint32_t right;
   uint32_t temp;

   uint32_t *ks = context->ks + 30;                                             //Keys in the key schedule must be applied in reverse order
   left = LOAD32BE(input + 0);                                                  //Copy the ciphertext from the input buffer
   right = LOAD32BE(input + 4);
   DES_IP(left, right);                                                         //Initial permutation

   for(i = 0; i < 16; i++, ks -= 2)                                             //16 rounds of computation are needed
   {
      DES_ROUND(left, right, ks);
   }
   DES_FP(right, left);                                                         //Inverse IP permutation
   STORE32BE(right, output + 0);                                                //Copy the resulting plaintext
   STORE32BE(left, output + 4);
}
#endif /* end choose algo */
/**
 * @brief Increment counter block
 * @param[in,out] x Pointer to the counter block
 * @param[in] n Size in bytes of the specific part of the block to be incremented
 **/
void ccmIncCounter(uint8_t *x, size_t n)
{
   size_t i;

   if (x==NULL) return;
   
   //The function increments the right-most bytes of the block. The remaining
   //left-most bytes remain unchanged
   for(i = 0; i < n; i++)
   {
      if(++(x[15 - i]) != 0)                                                    //Increment the current byte and propagate the carry if necessary
      {
         break;
      }
   }
}
/**
 * @brief XOR operation
 * @param[out] x Block resulting from the XOR operation
 * @param[in] a First block
 * @param[in] b Second block
 * @param[in] n Size of the block
 **/
void ccmXorBlock(uint8_t *x, const uint8_t *a, const uint8_t *b, size_t n)
{
   size_t i;

   if ((x==NULL)||((a==NULL)||(b==NULL))) return;

   for(i = 0; i < n; i++)                                                       //Perform XOR operation
   {
      x[i] = a[i] ^ b[i];
   }
}
/**
 * @brief Authenticated decryption using CCM
 * @param[in] cipher Cipher algorithm
 * @param[in] context Cipher algorithm context
 * @param[in] n Nonce
 * @param[in] nLen Length of the nonce
 * @param[in] a Additional authenticated data
 * @param[in] aLen Length of the additional data
 * @param[in] c Ciphertext to be decrypted
 * @param[out] p Plaintext resulting from the decryption
 * @param[in] length Total number of data bytes to be decrypted
 * @param[in] t MAC to be verified
 * @param[in] tLen Length of the MAC
 * @return Error code
 **/
uint8_t ccmDecrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen, const uint8_t *c, uint8_t *p, size_t length, const uint8_t *t, size_t tLen)
{
   uint8_t mask;
   size_t m;
   size_t q;
   size_t qLen;
   uint8_t b[16];
   uint8_t y[16];
   uint8_t r[16];
   uint8_t s[16];

   if(cipher == NULL || context == NULL)                                        //Check parameters
      return 0u;

   if(cipher->type != CIPHER_ALGO_TYPE_BLOCK || cipher->blockSize != 16)        //CCM supports only symmetric block ciphers whose block size is 128 bits
      return 0u;

   if(nLen < 7 || nLen > 13)                                                    //Check the length of the nonce
      return 2u;

   if(tLen < 4 || tLen > 16 || (tLen % 2) != 0)                                 //Check the length of the MAC
      return 2u;

   q = length;                                                                  //Q is the bit string representation of the octet length of C
   qLen = 15 - nLen;                                                            //Compute the octet length of Q

   b[0] = (aLen > 0) ? 0x40 : 0x00;                                             //Format the leading octet of the first block
   b[0] |= ((tLen - 2) / 2) << 3;                                               //Encode the octet length of T
   b[0] |= qLen - 1;                                                            //Encode the octet length of Q

   memcpy((void*)b + 1,(void*) n,(int16_t) nLen);                               //Copy the nonce
   for(m = 0; m < qLen; m++, q >>= 8)                                           //Encode the length field Q
   {
      b[15 - m] = q & 0xFF;
   }

   if(q != 0)                                                                   //Invalid length?
      return 2u;
   cipher->encryptBlock(context, b, y);                                         //Set Y(0) = CIPH(B(0))

   if(aLen > 0)                                                                 //Any additional data?
   {
      memset((void*)b,(void*) 0,(int16_t) 16);                                  //Format the associated data
      if(aLen < 0xFF00)                                                         //Check the length of the associated data string
      {

         STORE16BE(aLen, b);                                                    //The length is encoded as 2 octets
         m = min(aLen, 16 - 2);                                                 //Number of bytes to copy
         memcpy((void*)b + 2,(void*) a,(int16_t) m);                            //Concatenate the associated data A
      }
      else
      {
         b[0] = 0xFF;                                                           //The length is encoded as 6 octets
         b[1] = 0xFE;
         STORE32BE(aLen, b + 2);                                                //MSB is stored first
         m = min(aLen, 16 - 6);                                                 //Number of bytes to copy
         memcpy((void*)b + 6,(void*) a,(int16_t) m);                            //Concatenate the associated data A
      }
      ccmXorBlock(y, b, y, 16);                                                 //XOR B(1) with Y(0)
      cipher->encryptBlock(context, y, y);                                      //Compute Y(1) = CIPH(B(1) ^ Y(0))
      aLen -= m;                                                                //Number of remaining data bytes
      a += m;

      while(aLen > 0)                                                           //Process the remaining data bytes
      {
         m = min(aLen, 16);                                                     //Associated data are processed in a block-by-block fashion
         ccmXorBlock(y, a, y, m);                                               //XOR B(i) with Y(i-1)
         cipher->encryptBlock(context, y, y);                                   //Compute Y(i) = CIPH(B(i) ^ Y(i-1))
         aLen -= m;                                                             //Next block
         a += m;
      }
   }

   b[0] = (uint8_t) (qLen - 1);                                                 //Format CTR(0)
   memcpy((void*)b + 1,(void*) n,(int16_t) nLen);                               //Copy the nonce
   memset((void*)b + 1 + nLen,(void*) 0,(int16_t) qLen);                        //Initialize counter value
   cipher->encryptBlock(context, b, s);                                         //Compute S(0) = CIPH(CTR(0))
   memcpy((void*)r,(void*) s,(int16_t) tLen);                                   //Save MSB(S(0))

   while(length > 0)                                                            //Decrypt ciphertext
   {
      m = min(length, 16);                                                      //The decryption operates in a block-by-block fashion
      ccmIncCounter(b, qLen);                                                   //Increment counter
      cipher->encryptBlock(context, b, s);                                      //Compute S(i) = CIPH(CTR(i))
      ccmXorBlock(p, c, s, m);                                                  //Compute B(i) = C(i) XOR S(i)
      ccmXorBlock(y, p, y, m);                                                  //XOR B(i) with Y(i-1)
      cipher->encryptBlock(context, y, y);                                      //Compute Y(i) = CIPH(B(i) ^ Y(i-1))
      length -= m;                                                              //Next block
      c += m;
      p += m;
   }
   ccmXorBlock(r, r, y, tLen);                                                  //Compute MAC

   //The calculated tag is bitwise compared to the received tag. The message
   //is authenticated if and only if the tags match
   for(mask = 0, m = 0; m < tLen; m++)
   {
      mask |= r[m] ^ t[m];
   }

   return (mask == 0) ? 1u : 3u;                                                //Return status code
}
/**
 * @brief Authenticated encryption using CCM
 * @param[in] cipher Cipher algorithm
 * @param[in] context Cipher algorithm context
 * @param[in] n Nonce
 * @param[in] nLen Length of the nonce
 * @param[in] a Additional authenticated data
 * @param[in] aLen Length of the additional data
 * @param[in] p Plaintext to be encrypted
 * @param[out] c Ciphertext resulting from the encryption
 * @param[in] length Total number of data bytes to be encrypted
 * @param[out] t MAC resulting from the encryption process
 * @param[in] tLen Length of the MAC
 * @return Error code
 **/
uint8_t ccmEncrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen, const uint8_t *p, uint8_t *c, size_t length, uint8_t *t, size_t tLen)
{
   size_t m;
   size_t q;
   size_t qLen;
   uint8_t b[16];
   uint8_t y[16];
   uint8_t s[16];

   if(cipher == NULL || context == NULL)                                        //Check parameters
      return 0u;

   if(cipher->type != CIPHER_ALGO_TYPE_BLOCK || cipher->blockSize != 16)        //CCM supports only symmetric block ciphers whose block size is 128 bits
      return 0u;

   if(nLen < 7 || nLen > 13)                                                    //Check the length of the nonce
      return 2u;

   if(tLen < 4 || tLen > 16 || (tLen % 2) != 0)                                 //Check the length of the MAC
      return 2u;

   q = length;                                                                  //Q is the bit string representation of the octet length of P
   qLen = 15 - nLen;                                                            //Compute the octet length of Q
   b[0] = (aLen > 0) ? 0x40 : 0x00;                                             //Format the leading octet of the first block
   b[0] |= ((tLen - 2) / 2) << 3;                                               //Encode the octet length of T
   b[0] |= qLen - 1;                                                            //Encode the octet length of Q

   memcpy((void*)b + 1,(void*) n,(int16_t) nLen);                               //Copy the nonce

   for(m = 0; m < qLen; m++, q >>= 8)                                           //Encode the length field Q
   {
      b[15 - m] = q & 0xFF;
   }

   if(q != 0)                                                                   //Invalid length?
      return 2u;

   cipher->encryptBlock(context, b, y);                                         //Set Y(0) = CIPH(B(0))

   if(aLen > 0)                                                                 //Any additional data?
   {
      memset((void*)b,(void*) 0,(int16_t) 16);                                  //Format the associated data
      if(aLen < 0xFF00)                                                         //Check the length of the associated data string
      {
         STORE16BE(aLen, b);                                                    //The length is encoded as 2 octets
         m = min(aLen, 16 - 2);                                                 //Number of bytes to copy
         memcpy((void*)b + 2,(void*) a,(int16_t) m);                            //Concatenate the associated data A
      }
      else
      {
         b[0] = 0xFF;                                                           //The length is encoded as 6 octets
         b[1] = 0xFE;
         STORE32BE(aLen, b + 2);                                                //MSB is stored first
         m = min(aLen, 16 - 6);                                                 //Number of bytes to copy
         memcpy((void*)b + 6,(void*) a,(int16_t) m);                            //Concatenate the associated data A
      }
      ccmXorBlock(y, b, y, 16);                                                 //XOR B(1) with Y(0)
      cipher->encryptBlock(context, y, y);                                      //Compute Y(1) = CIPH(B(1) ^ Y(0))
      aLen -= m;                                                                //Number of remaining data bytes
      a += m;
      while(aLen > 0)                                                           //Process the remaining data bytes
      {
         m = min(aLen, 16);                                                     //Associated data are processed in a block-by-block fashion
         ccmXorBlock(y, a, y, m);                                               //XOR B(i) with Y(i-1)
         cipher->encryptBlock(context, y, y);                                   //Compute Y(i) = CIPH(B(i) ^ Y(i-1))
         aLen -= m;                                                             //Next block
         a += m;
      }
   }
   b[0] = (uint8_t) (qLen - 1);                                                 //Format CTR(0)
   memcpy((void*)b + 1,(void*) n,(int16_t) nLen);                               //Copy the nonce
   memset((void*)b + 1 + nLen,(void*) 0,(int16_t) qLen);                        //Initialize counter value
   cipher->encryptBlock(context, b, s);                                         //Compute S(0) = CIPH(CTR(0))
   memcpy((void*)t,(void*) s,(int16_t) tLen);                                   //Save MSB(S(0))

   while(length > 0)                                                            //Encrypt plaintext
   {
      m = min(length, 16);                                                      //The encryption operates in a block-by-block fashion
      ccmXorBlock(y, p, y, m);                                                  //XOR B(i) with Y(i-1)
      cipher->encryptBlock(context, y, y);                                      //Compute Y(i) = CIPH(B(i) ^ Y(i-1))
      ccmIncCounter(b, qLen);                                                   //Increment counter
      cipher->encryptBlock(context, b, s);                                      //Compute S(i) = CIPH(CTR(i))
      ccmXorBlock(c, p, s, m);                                                  //Compute C(i) = B(i) XOR S(i)

      length -= m;                                                              //Next block
      p += m;
      c += m;
   }

   ccmXorBlock(t, t, y, tLen);                                                  //Compute MAC
   return 1u;                                                                   //Successful encryption
}
/**
 * @brief Initialize Poly1305 message-authentication code computation
 * @param[in] context Pointer to the Poly1305 context to initialize
 * @param[in] key Pointer to the 256-bit key
 **/
void poly1305Init(Poly1305Context_t *context, const uint8_t *key)
{
   if ((context==NULL) || (key==NULL)) return;

   //The 256-bit key is partitioned into two parts, called r and s
   context->r[0] = LOAD32LE(key);
   context->r[1] = LOAD32LE(key + 4);
   context->r[2] = LOAD32LE(key + 8);
   context->r[3] = LOAD32LE(key + 12);
   context->s[0] = LOAD32LE(key + 16);
   context->s[1] = LOAD32LE(key + 20);
   context->s[2] = LOAD32LE(key + 24);
   context->s[3] = LOAD32LE(key + 28);

   //Certain bits of r are required to be 0
   context->r[0] &= 0x0FFFFFFF;
   context->r[1] &= 0x0FFFFFFC;
   context->r[2] &= 0x0FFFFFFC;
   context->r[3] &= 0x0FFFFFFC;

   //The accumulator is set to zero
   context->a[0] = 0;
   context->a[1] = 0;
   context->a[2] = 0;
   context->a[3] = 0;
   context->a[4] = 0;
   context->a[5] = 0;
   context->a[6] = 0;
   context->a[7] = 0;

   //Number of bytes in the buffer
   context->size = 0;
}
/**
 * @brief Update Poly1305 message-authentication code computation
 * @param[in] context Pointer to the Poly1305 context
 * @param[in] data Pointer to the input message
 * @param[in] length Length of the input message
 **/
void poly1305Update(Poly1305Context_t *context, const void *dataV, size_t length)
{
   size_t n;

   if ((context==NULL) || (dataV==NULL)) return;

   //Process the incoming data
   while(length > 0)
   {
      //The buffer can hold at most 16 bytes
      n = min(length, 16 - context->size);

      //Copy the data to the buffer
      memcpy((void*)context->buffer + context->size,(void*) dataV,(int16_t) n);

      //Update the Poly1305 context
      context->size += n;
      //Advance the data pointer
      dataV = (uint8_t *) dataV + n;
      //Remaining bytes to process
      length -= n;

      //Process message in 16-byte blocks
      if(context->size == 16)
      {
         //Transform the 16-byte block
         poly1305ProcessBlock(context);
         //Empty the buffer
         context->size = 0;
      }
   }
}
/**
 * @brief Finalize Poly1305 message-authentication code computation
 * @param[in] context Pointer to the Poly1305 context
 * @param[out] tag Calculated message-authentication code
 **/
void poly1305Final(Poly1305Context_t *context, uint8_t *tag)
{
   uint32_t mask;
   uint32_t b[4];

   if ((context==NULL) || (tag==NULL)) return;

   //Process the last block
   if(context->size != 0)
      poly1305ProcessBlock(context);

   //Save the accumulator
   b[0] = context->a[0] & 0xFFFFFFFF;
   b[1] = context->a[1] & 0xFFFFFFFF;
   b[2] = context->a[2] & 0xFFFFFFFF;
   b[3] = context->a[3] & 0xFFFFFFFF;

   //Compute a + 5
   context->a[0] += 5;

   //Propagate the carry
   context->a[1] += context->a[0] >> 32;
   context->a[2] += context->a[1] >> 32;
   context->a[3] += context->a[2] >> 32;
   context->a[4] += context->a[3] >> 32;

   //If (a + 5) >= 2^130, form a mask with the value 0x00000000. Else,
   //form a mask with the value 0xffffffff
   mask = ((context->a[4] & 0x04) >> 2) - 1;

   //Select between ((a - (2^130 - 5)) % 2^128) and (a % 2^128)
   context->a[0] = (context->a[0] & ~mask) | (b[0] & mask);
   context->a[1] = (context->a[1] & ~mask) | (b[1] & mask);
   context->a[2] = (context->a[2] & ~mask) | (b[2] & mask);
   context->a[3] = (context->a[3] & ~mask) | (b[3] & mask);

   //Finally, the value of the secret key s is added to the accumulator
   context->a[0] += context->s[0];
   context->a[1] += context->s[1];
   context->a[2] += context->s[2];
   context->a[3] += context->s[3];

   //Propagate the carry
   context->a[1] += context->a[0] >> 32;
   context->a[2] += context->a[1] >> 32;
   context->a[3] += context->a[2] >> 32;
   context->a[4] += context->a[3] >> 32;

   //We only consider the least significant bits
   b[0] = context->a[0] & 0xFFFFFFFF;
   b[1] = context->a[1] & 0xFFFFFFFF;
   b[2] = context->a[2] & 0xFFFFFFFF;
   b[3] = context->a[3] & 0xFFFFFFFF;

   //The result is serialized as a little-endian number, producing
   //the 16 byte tag
   STORE32LE(b[0], tag);
   STORE32LE(b[1], tag + 4);
   STORE32LE(b[2], tag + 8);
   STORE32LE(b[3], tag + 12);

   //Clear the accumulator
   context->a[0] = 0;
   context->a[1] = 0;
   context->a[2] = 0;
   context->a[3] = 0;
   context->a[4] = 0;
   context->a[5] = 0;
   context->a[6] = 0;
   context->a[7] = 0;

   //Clear r and s
   context->r[0] = 0;
   context->r[1] = 0;
   context->r[2] = 0;
   context->r[3] = 0;
   context->s[0] = 0;
   context->s[1] = 0;
   context->s[2] = 0;
   context->s[3] = 0;
}
/**
 * @brief Process message in 16-byte blocks
 * @param[in] context Pointer to the Poly1305 context
 **/
void poly1305ProcessBlock(Poly1305Context_t *context)
{
   uint32_t a[5];
   uint32_t r[4];
   uint8_t n;

   if (context==NULL) return;

   //Retrieve the length of the last block
   n = context->size;

   //Add one bit beyond the number of octets. For a 16-byte block,
   //this is equivalent to adding 2^128 to the number. For the shorter
   //block, it can be 2^120, 2^112, or any power of two that is evenly
   //divisible by 8, all the way down to 2^8
   context->buffer[n++] = 0x01;

   //If the resulting block is not 17 bytes long (the last block),
   //pad it with zeros
   while(n < 17)
      context->buffer[n++] = 0x00;

   //Read the block as a little-endian number
   a[0] = LOAD32LE(context->buffer);
   a[1] = LOAD32LE(context->buffer + 4);
   a[2] = LOAD32LE(context->buffer + 8);
   a[3] = LOAD32LE(context->buffer + 12);
   a[4] = context->buffer[16];

   //Add this number to the accumulator
   context->a[0] += a[0];
   context->a[1] += a[1];
   context->a[2] += a[2];
   context->a[3] += a[3];
   context->a[4] += a[4];

   //Propagate the carry
   context->a[1] += context->a[0] >> 32;
   context->a[2] += context->a[1] >> 32;
   context->a[3] += context->a[2] >> 32;
   context->a[4] += context->a[3] >> 32;

   //We only consider the least significant bits
   a[0] = context->a[0] & 0xFFFFFFFF;
   a[1] = context->a[1] & 0xFFFFFFFF;
   a[2] = context->a[2] & 0xFFFFFFFF;
   a[3] = context->a[3] & 0xFFFFFFFF;
   a[4] = context->a[4] & 0xFFFFFFFF;

   //Copy r
   r[0] = context->r[0];
   r[1] = context->r[1];
   r[2] = context->r[2];
   r[3] = context->r[3];

   //Multiply the accumulator by r
   context->a[0] = (uint64_t) a[0] * r[0];
   context->a[1] = (uint64_t) a[0] * r[1] + (uint64_t) a[1] * r[0];
   context->a[2] = (uint64_t) a[0] * r[2] + (uint64_t) a[1] * r[1] + (uint64_t) a[2] * r[0];
   context->a[3] = (uint64_t) a[0] * r[3] + (uint64_t) a[1] * r[2] + (uint64_t) a[2] * r[1] + (uint64_t) a[3] * r[0];
   context->a[4] = (uint64_t) a[1] * r[3] + (uint64_t) a[2] * r[2] + (uint64_t) a[3] * r[1] + (uint64_t) a[4] * r[0];
   context->a[5] = (uint64_t) a[2] * r[3] + (uint64_t) a[3] * r[2] + (uint64_t) a[4] * r[1];
   context->a[6] = (uint64_t) a[3] * r[3] + (uint64_t) a[4] * r[2];
   context->a[7] = (uint64_t) a[4] * r[3];

   //Propagate the carry
   context->a[1] += context->a[0] >> 32;
   context->a[2] += context->a[1] >> 32;
   context->a[3] += context->a[2] >> 32;
   context->a[4] += context->a[3] >> 32;
   context->a[5] += context->a[4] >> 32;
   context->a[6] += context->a[5] >> 32;
   context->a[7] += context->a[6] >> 32;

   //Save the high part of the accumulator
   a[0] = context->a[4] & 0xFFFFFFFC;
   a[1] = context->a[5] & 0xFFFFFFFF;
   a[2] = context->a[6] & 0xFFFFFFFF;
   a[3] = context->a[7] & 0xFFFFFFFF;

   //We only consider the least significant bits
   context->a[0] &= 0xFFFFFFFF;
   context->a[1] &= 0xFFFFFFFF;
   context->a[2] &= 0xFFFFFFFF;
   context->a[3] &= 0xFFFFFFFF;
   context->a[4] &= 0x00000003;

   //Perform fast modular reduction (first pass)
   context->a[0] += a[0];
   context->a[0] += (a[0] >> 2) | (a[1] << 30);
   context->a[1] += a[1];
   context->a[1] += (a[1] >> 2) | (a[2] << 30);
   context->a[2] += a[2];
   context->a[2] += (a[2] >> 2) | (a[3] << 30);
   context->a[3] += a[3];
   context->a[3] += (a[3] >> 2);

   //Propagate the carry
   context->a[1] += context->a[0] >> 32;
   context->a[2] += context->a[1] >> 32;
   context->a[3] += context->a[2] >> 32;
   context->a[4] += context->a[3] >> 32;

   //Save the high part of the accumulator
   a[0] = context->a[4] & 0xFFFFFFFC;

   //We only consider the least significant bits
   context->a[0] &= 0xFFFFFFFF;
   context->a[1] &= 0xFFFFFFFF;
   context->a[2] &= 0xFFFFFFFF;
   context->a[3] &= 0xFFFFFFFF;
   context->a[4] &= 0x00000003;

   //Perform fast modular reduction (second pass)
   context->a[0] += a[0];
   context->a[0] += a[0] >> 2;

   //Propagate the carry
   context->a[1] += context->a[0] >> 32;
   context->a[2] += context->a[1] >> 32;
   context->a[3] += context->a[2] >> 32;
   context->a[4] += context->a[3] >> 32;

   //We only consider the least significant bits
   context->a[0] &= 0xFFFFFFFF;
   context->a[1] &= 0xFFFFFFFF;
   context->a[2] &= 0xFFFFFFFF;
   context->a[3] &= 0xFFFFFFFF;
   context->a[4] &= 0x00000003;
}
#if defined(ALGO_CHA_CHA)
/**
 * @brief Authenticated encryption using ChaCha20Poly1305
 * @param[in] k key
 * @param[in] kLen Length of the key
 * @param[in] n Nonce
 * @param[in] nLen Length of the nonce
 * @param[in] a Additional authenticated data
 * @param[in] aLen Length of the additional data
 * @param[in] p Plaintext to be encrypted
 * @param[out] c Ciphertext resulting from the encryption
 * @param[in] length Total number of data bytes to be encrypted
 * @param[out] t MAC resulting from the encryption process
 * @param[in] tLen Length of the MAC
 * @return Error code
 **/
uint8_t chacha20Poly1305Encrypt(const uint8_t *k, size_t kLen,const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen, const uint8_t *p, uint8_t *c, size_t length, uint8_t *t, size_t tLen)
{
   uint8_t error;
   size_t paddingLen;
   ChachaContext_t chachaContext;
   Poly1305Context_t poly1305Context;
   uint8_t temp[32];

   //Check the length of the message-authentication code
   if ((tLen != 16) || ((k==NULL) || ((n==NULL) || ((a==NULL) || ((p==NULL) || ((c==NULL) || (t==NULL)))))))
      return 0u;

   //Initialize ChaCha20 context
   error = chachaInit(&chachaContext, 20, k, kLen, n, nLen);
   //Any error to report?
   if(error)
      return error;

   //First, a Poly1305 one-time key is generated from the 256-bit key
   //and nonce
   chachaCipher(&chachaContext, NULL, temp, 32);

   //The other 256 bits of the Chacha20 block are discarded
   chachaCipher(&chachaContext, NULL, NULL, 32);

   //Next, the ChaCha20 encryption function is called to encrypt the
   //plaintext, using the same key and nonce
   chachaCipher(&chachaContext, p, c, length);

   //Initialize the Poly1305 function with the key calculated above
   poly1305Init(&poly1305Context, temp);

   //Compute MAC over the AAD
   poly1305Update(&poly1305Context, a, aLen);

   //If the length of the AAD is not an integral multiple of 16 bytes,
   //then padding is required
   if((aLen % 16) != 0)
   {
      //Compute the number of padding bytes
      paddingLen = 16 - (aLen % 16);

      //The padding is up to 15 zero bytes, and it brings the total length
      //so far to an integral multiple of 16
      memset((void*)temp,(void*) 0,(int16_t) paddingLen);

      //Compute MAC over the padding
      poly1305Update(&poly1305Context, temp, paddingLen);
   }

   //Compute MAC over the ciphertext
   poly1305Update(&poly1305Context, c, length);

   //If the length of the ciphertext is not an integral multiple of 16 bytes,
   //then padding is required
   if((length % 16) != 0)
   {
      //Compute the number of padding bytes
      paddingLen = 16 - (length % 16);

      //The padding is up to 15 zero bytes, and it brings the total length
      //so far to an integral multiple of 16
      memset((void*)temp,(void*) 0,(int16_t) paddingLen);

      //Compute MAC over the padding
      poly1305Update(&poly1305Context, temp, paddingLen);
   }

   //Encode the length of the AAD as a 64-bit little-endian integer
   STORE64LE(aLen, temp);
   //Compute MAC over the length field
   poly1305Update(&poly1305Context, temp, sizeof(uint64_t));

   //Encode the length of the ciphertext as a 64-bit little-endian integer
   STORE64LE(length, temp);
   //Compute MAC over the length field
   poly1305Update(&poly1305Context, temp, sizeof(uint64_t));

   //Compute message-authentication code
   poly1305Final(&poly1305Context, t);

   //Successful encryption
   return 1u;
}

/**
 * @brief Authenticated decryption using ChaCha20Poly1305
 * @param[in] k key
 * @param[in] kLen Length of the key
 * @param[in] n Nonce
 * @param[in] nLen Length of the nonce
 * @param[in] a Additional authenticated data
 * @param[in] aLen Length of the additional data
 * @param[in] c Ciphertext to be decrypted
 * @param[out] p Plaintext resulting from the decryption
 * @param[in] length Total number of data bytes to be decrypted
 * @param[in] t MAC to be verified
 * @param[in] tLen Length of the MAC
 * @return Error code
 **/
uint8_t chacha20Poly1305Decrypt(const uint8_t *k, size_t kLen, const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen,  const uint8_t *c, uint8_t *p, size_t length, const uint8_t *t, size_t tLen)
{
   uint8_t error;
   uint8_t mask;
   size_t i;
   size_t paddingLen;
   ChachaContext_t chachaContext;
   Poly1305Context_t poly1305Context;
   uint8_t temp[32];

   //Check the length of the message-authentication code
   if ((tLen != 16) || ((k==NULL) || ((n==NULL) || ((a==NULL) || ((p==NULL) || ((c==NULL) || (t==NULL)))))))
      return 0u;

   //Initialize ChaCha20 context
   error = chachaInit(&chachaContext, 20, k, kLen, n, nLen);
   //Any error to report?
   if(error)
      return error;

   //First, a Poly1305 one-time key is generated from the 256-bit key
   //and nonce
   chachaCipher(&chachaContext, NULL, temp, 32);

   //The other 256 bits of the Chacha20 block are discarded
   chachaCipher(&chachaContext, NULL, NULL, 32);

   //Initialize the Poly1305 function with the key calculated above
   poly1305Init(&poly1305Context, temp);

   //Compute MAC over the AAD
   poly1305Update(&poly1305Context, a, aLen);

   //If the length of the AAD is not an integral multiple of 16 bytes,
   //then padding is required
   if((aLen % 16) != 0)
   {
      //Compute the number of padding bytes
      paddingLen = 16 - (aLen % 16);

      //The padding is up to 15 zero bytes, and it brings the total length
      //so far to an integral multiple of 16
      memset((void*)temp,(void*) 0,(int16_t) paddingLen);

      //Compute MAC over the padding
      poly1305Update(&poly1305Context, temp, paddingLen);
   }

   //Compute MAC over the ciphertext
   poly1305Update(&poly1305Context, c, length);

   //If the length of the ciphertext is not an integral multiple of 16 bytes,
   //then padding is required
   if((length % 16) != 0)
   {
      //Compute the number of padding bytes
      paddingLen = 16 - (length % 16);

      //The padding is up to 15 zero bytes, and it brings the total length
      //so far to an integral multiple of 16
      memset((void*)temp,(void*) 0,(int16_t) paddingLen);

      //Compute MAC over the padding
      poly1305Update(&poly1305Context, temp, paddingLen);
   }

   //Encode the length of the AAD as a 64-bit little-endian integer
   STORE64LE(aLen, temp);
   //Compute MAC over the length field
   poly1305Update(&poly1305Context, temp, sizeof(uint64_t));

   //Encode the length of the ciphertext as a 64-bit little-endian integer
   STORE64LE(length, temp);
   //Compute MAC over the length field
   poly1305Update(&poly1305Context, temp, sizeof(uint64_t));

   //Compute message-authentication code
   poly1305Final(&poly1305Context, temp);

   //Finally, we decrypt the ciphertext
   chachaCipher(&chachaContext, c, p, length);

   //The calculated tag is bitwise compared to the received tag. The
   //message is authenticated if and only if the tags match
   for(mask = 0, i = 0; i < tLen; i++)
   {
      mask |= temp[i] ^ t[i];
   }

   //Return status code
   return (mask == 0) ? 1u : 2u;
}
#endif /* _________ needs chacha ____________ */
#ifdef MAX_CIPHER_BLOCK_SIZE                                                    // the cipher chosen can be used with CMAC
/**
 * @brief Reset CMAC context
 * @param[in] context Pointer to the CMAC context
 **/
void cmacReset(CmacContext_t *context)
{
   //Clear input buffer
   memset((void*)context->buffer,(void*) 0,(int16_t) context->cipher->blockSize);
   //Number of bytes in the buffer
   context->bufferLength = 0;

   //Initialize MAC value
   memset((void*)context->mac,(void*) 0,(int16_t) context->cipher->blockSize);
}
/**
 * @brief Multiplication by x in GF(2^128)
 * @param[out] x Pointer to the output block
 * @param[out] a Pointer to the input block
 * @param[in] n Size of the block, in bytes
 * @param[in] rb Representation of the irreducible binary polynomial
 **/
void cmacMul(uint8_t *x, const uint8_t *a, size_t n, uint8_t rb)
{
   size_t i;
   uint8_t c;

   //Save the value of the most significant bit
   c = a[0] >> 7;

   //The multiplication of a polynomial by x in GF(2^128) corresponds to a
   //shift of indices
   for(i = 0; i < (n - 1); i++)
   {
      x[i] = (a[i] << 1) | (a[i + 1] >> 7);
   }

   //Shift the last byte of the block to the left
   x[i] = a[i] << 1;

   //If the highest term of the result is equal to one, then perform reduction
   x[i] ^= rb & ~(c - 1);
}
/**
 * @brief Initialize CMAC calculation
 * @param[in] context Pointer to the CMAC context to initialize
 * @param[in] cipher Cipher algorithm used to compute CMAC
 * @param[in] key Pointer to the secret key
 * @param[in] keyLen Length of the secret key
 * @return Error code
 **/
uint8_t cmacInit(CmacContext_t *context, const CipherAlgo_t *cipher, const void *key, size_t keyLen)
{
   uint8_t error;
   uint8_t rb;

   //CMAC supports only block ciphers whose block size is 64 or 128 bits
   if((cipher->type != CIPHER_ALGO_TYPE_BLOCK) || ((context==NULL) || (cipher==NULL)))
      return 0u;

   //Rb is completely determined by the number of bits in a block
   if(cipher->blockSize == 8)
   {
      //If b = 64, then Rb = 11011
      rb = 0x1B;
   }
   else if(cipher->blockSize == 16)
   {
      //If b = 128, then Rb = 10000111
      rb = 0x87;
   }
   else
   {
      //Invalid block size
      return 0u;
   }

   //Cipher algorithm used to compute CMAC
   context->cipher = cipher;

   //Initialize cipher context
   error = cipher->init(context->cipherContext, key, keyLen);
   //Any error to report?
   if(error)
      return error;

   //Let L = 0
   memset((void*)context->buffer,(void*) 0,(int16_t) cipher->blockSize);
   //Compute L = CIPH(L)
   cipher->encryptBlock(context->cipherContext, context->buffer, context->buffer);

   //The subkey K1 is obtained by multiplying L by x in GF(2^b)
   cmacMul(context->k1, context->buffer, cipher->blockSize, rb);
   //The subkey K2 is obtained by multiplying L by x^2 in GF(2^b)
   cmacMul(context->k2, context->k1, cipher->blockSize, rb);

   //Reset CMAC context
   cmacReset(context);

   //Successful initialization
   return 1u;
}
/**
 * @brief XOR operation
 * @param[out] x Block resulting from the XOR operation
 * @param[in] a First input block
 * @param[in] b Second input block
 * @param[in] n Size of the block, in bytes
 **/
void cmacXorBlock(uint8_t *x, const uint8_t *a, const uint8_t *b, size_t n)
{
   size_t i;

   //Perform XOR operation
   for(i = 0; i < n; i++)
   {
      x[i] = a[i] ^ b[i];
   }
}
/**
 * @brief Update the CMAC context with a portion of the message being hashed
 * @param[in] context Pointer to the CMAC context
 * @param[in] data Pointer to the input data
 * @param[in] dataLen Length of the buffer
 **/
void cmacUpdate(CmacContext_t *context, const void *dataV, size_t dataLen)
{
   size_t n;

   //Process the incoming data
   while(dataLen > 0)
   {
      //Process message block by block
      if(context->bufferLength == context->cipher->blockSize)
      {
         //XOR M(i) with C(i-1)
         cmacXorBlock(context->buffer, context->buffer, context->mac,
            context->cipher->blockSize);

         //Compute C(i) = CIPH(M(i) ^ C(i-1))
         context->cipher->encryptBlock(context->cipherContext, context->buffer, context->mac);

         //Empty the buffer
         context->bufferLength = 0;
      }

      //The message is partitioned into complete blocks
      n = min(dataLen, context->cipher->blockSize - context->bufferLength);

      //Copy the data to the buffer
      memcpy((void*)context->buffer + context->bufferLength,(void*) dataV,(int16_t) n);
      //Update the length of the buffer
      context->bufferLength += n;

      //Advance the data pointer
      dataV = (uint8_t *) dataV + n;
      //Remaining bytes to process
      dataLen -= n;
   }
}
/**
 * @brief Finish the CMAC calculation
 * @param[in] context Pointer to the CMAC context
 * @param[out] mac Calculated MAC value
 * @param[in] macLen Expected length of the MAC
 * @return Error code
 **/
uint8_t cmacFinal(CmacContext_t *context, uint8_t *mac, size_t macLen)
{
   //Check the length of the MAC
   if(macLen < 1 || macLen > context->cipher->blockSize)
      return 0u;

   //Check whether the last block Mn is complete
   if(context->bufferLength >= context->cipher->blockSize)
   {
      //The final block M(n) is XOR-ed with the first subkey K1
      cmacXorBlock(context->buffer, context->buffer, context->k1, context->cipher->blockSize);
   }
   else
   {
      //Append padding string
      context->buffer[context->bufferLength++] = 0x80;

      //Append the minimum number of zeroes to form a complete block
      while(context->bufferLength < context->cipher->blockSize)
         context->buffer[context->bufferLength++] = 0x00;

      //The final block M(n) is XOR-ed with the second subkey K2
      cmacXorBlock(context->buffer, context->buffer, context->k2, context->cipher->blockSize);
   }

   //XOR M(n) with C(n-1)
   cmacXorBlock(context->buffer, context->buffer, context->mac, context->cipher->blockSize);

   //Compute T = CIPH(M(n) ^ C(n-1))
   context->cipher->encryptBlock(context->cipherContext, context->buffer, context->mac);

   //Copy the resulting MAC value
   if(mac != NULL)
   {
      //It is possible to truncate the MAC. The result of the truncation
      //should be taken in most significant bits first order
      memcpy((void*)mac,(void*) context->mac,(int16_t) macLen);
   }

   //Successful processing
   return 1u;
}
#endif /* ________ a ciper was chosen that can be used with cmac _________ */
#if defined(IOT_SHA_256)
//SHA-256 padding
static const uint8_t padding[64u] =
{
   0x80u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u
};
//SHA-256 constants
static const uint32_t k[64u] =
{
   0x428A2F98ul, 0x71374491ul, 0xB5C0FBCFul, 0xE9B5DBA5ul, 0x3956C25Bul, 0x59F111F1ul, 0x923F82A4ul, 0xAB1C5ED5ul,
   0xD807AA98ul, 0x12835B01ul, 0x243185BEul, 0x550C7DC3ul, 0x72BE5D74ul, 0x80DEB1FEul, 0x9BDC06A7ul, 0xC19BF174ul,
   0xE49B69C1ul, 0xEFBE4786ul, 0x0FC19DC6ul, 0x240CA1CCul, 0x2DE92C6Ful, 0x4A7484AAul, 0x5CB0A9DCul, 0x76F988DAul,
   0x983E5152ul, 0xA831C66Dul, 0xB00327C8ul, 0xBF597FC7ul, 0xC6E00BF3ul, 0xD5A79147ul, 0x06CA6351ul, 0x14292967ul,
   0x27B70A85ul, 0x2E1B2138ul, 0x4D2C6DFCul, 0x53380D13ul, 0x650A7354ul, 0x766A0ABBul, 0x81C2C92Eul, 0x92722C85ul,
   0xA2BFE8A1ul, 0xA81A664Bul, 0xC24B8B70ul, 0xC76C51A3ul, 0xD192E819ul, 0xD6990624ul, 0xF40E3585ul, 0x106AA070ul,
   0x19A4C116ul, 0x1E376C08ul, 0x2748774Cul, 0x34B0BCB5ul, 0x391C0CB3ul, 0x4ED8AA4Aul, 0x5B9CCA4Ful, 0x682E6FF3ul,
   0x748F82EEul, 0x78A5636Ful, 0x84C87814ul, 0x8CC70208ul, 0x90BEFFFAul, 0xA4506CEBul, 0xBEF9A3F7ul, 0xC67178F2ul
};
//SHA-256 object identifier (2.16.840.1.101.3.4.2.1)
const uint8_t sha256Oid[9] = {0x60u, 0x86u, 0x48u, 0x01u, 0x65u, 0x03u, 0x04u, 0x02u, 0x01u};
//Common interface for hash algorithms
const HashAlgo_t sha256HashAlgo =
{
   "SHA-256",
   sha256Oid,
   sizeof(sha256Oid),
   sizeof(Sha256Context_t),
   SHA256_BLOCK_SIZE,
   SHA256_DIGEST_SIZE,
   SHA256_MIN_PAD_SIZE,
   TRUE,
//   (HashAlgoCompute) sha256Compute,
//   (HashAlgoInit) sha256Init,
//   (HashAlgoUpdate) sha256Update,
//   (HashAlgoFinal) sha256Final,
//   (HashAlgoFinalRaw) sha256FinalRaw
};
/**
 * @brief Initialize SHA-256 message digest context
 * @param[in] context Pointer to the SHA-256 context to initialize
 **/
void sha256Init(Sha256Context_t *context)
{
   context->h[0u] = 0x6A09E667ul;                                               //Set initial hash value
   context->h[1u] = 0xBB67AE85ul;
   context->h[2u] = 0x3C6EF372ul;
   context->h[3u] = 0xA54FF53Aul;
   context->h[4u] = 0x510E527Ful;
   context->h[5u] = 0x9B05688Cul;
   context->h[6u] = 0x1F83D9ABul;
   context->h[7u] = 0x5BE0CD19ul;

   context->size = 0u;                                                          //Number of bytes in the buffer
   context->totalSize = 0u;                                                     //Total length of the message
}
/**
 * @brief Process message in 16-word blocks
 * @param[in] context Pointer to the SHA-256 context
 **/
void sha256ProcessBlock(Sha256Context_t *context)
{
   uint8_t t;
   uint32_t temp1;
   uint32_t temp2;

   uint32_t a = context->h[0u];                                                 //Initialize the 8 working registers
   uint32_t b = context->h[1u];
   uint32_t c = context->h[2u];
   uint32_t d = context->h[3u];
   uint32_t e = context->h[4u];
   uint32_t f = context->h[5u];
   uint32_t g = context->h[6u];
   uint32_t h = context->h[7u];
   uint32_t *w = context->w;                                                    //Process message in 16-word blocks

   for(t = 0u; t < 16u; t++)                                                    //Convert from big-endian byte order to host byte order
   {
      w[t] = betoh32(w[t]);
   }
   for(t = 0u; t < 64u; t++)                                                    //SHA-256 hash computation (alternate method)
   {
      if(t >= 16u)                                                              //Prepare the message schedule
         W(t) += SIGMA4(W(t + 14u)) + W(t + 9u) + SIGMA3(W(t + 1u));
      temp1 = h + SIGMA2(e) + CH(e, f, g) + k[t] + W(t);                        //Calculate T1 and T2
      temp2 = SIGMA1(a) + MAJ(a, b, c);
      h = g;                                                                    //Update the working registers
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
   }
   context->h[0u] += a;                                                          //Update the hash value
   context->h[1u] += b;
   context->h[2u] += c;
   context->h[3u] += d;
   context->h[4u] += e;
   context->h[5u] += f;
   context->h[6u] += g;
   context->h[7u] += h;
}
/**
 * @brief Finish the SHA-256 message digest (no padding is added)
 * @param[in] context Pointer to the SHA-256 context
 * @param[out] digest Calculated digest
 **/
void sha256FinalRaw(Sha256Context_t *context, uint8_t *digest)
{
   uint8_t i;

   for(i = 0u; i < 8u; i++)                                                     //Convert from host byte order to big-endian byte order
   {
      context->h[i] = htobe32(context->h[i]);
   }
   memcpy((void*)digest,(void*) context->digest,(int16_t) SHA256_DIGEST_SIZE);  //Copy the resulting digest
   for(i = 0u; i < 8u; i++)                                                     //Convert from big-endian byte order to host byte order
   {
      context->h[i] = betoh32(context->h[i]);
   }
}
/**
 * @brief Update the SHA-256 context with a portion of the message being hashed
 * @param[in] context Pointer to the SHA-256 context
 * @param[in] data Pointer to the buffer being hashed
 * @param[in] length Length of the buffer
 **/
void sha256Update(Sha256Context_t *context, const void *dataV, size_t length)
{
   size_t n;

   while(length > 0u)                                                           //Process the incoming data
   {
      n = min(length, 64u - context->size);                                     //The buffer can hold at most 64 bytes
      memcpy((void*)context->buffer + context->size,(void*) dataV,(int16_t) n); //Copy the data to the buffer
      context->size += n;                                                       //Update the SHA-256 context
      context->totalSize += n;
      dataV = (uint8_t *) dataV + n;                                            //Advance the data pointer
      length -= n;                                                              //Remaining bytes to process
      if(context->size == 64u)                                                   //Process message in 16-word blocks
      {
         sha256ProcessBlock(context);                                           //Transform the 16-word block
         context->size = 0u;                                                    //Empty the buffer
      }
   }
}
/**
 * @brief Finish the SHA-256 message digest
 * @param[in] context Pointer to the SHA-256 context
 * @param[out] digest Calculated digest (optional parameter)
 **/
void sha256Final(Sha256Context_t *context, uint8_t *digest)
{
   uint8_t i;
   size_t paddingSize;
   uint64_t totalSize;

   totalSize = context->totalSize * 8u;                                         //Length of the original message (before padding)

   if(context->size < 56u)                                                      //Pad the message so that its length is congruent to 56 modulo 64
      paddingSize = 56u - context->size;
   else
      paddingSize = 64u + 56u - context->size;

   sha256Update(context, padding, paddingSize);                                 //Append padding

   context->w[14u] = htobe32((uint32_t) (totalSize >> 32));                     //Append the length of the original message
   context->w[15u] = htobe32((uint32_t) totalSize);

   sha256ProcessBlock(context);                                                 //Calculate the message digest

   for(i = 0u; i < 8u; i++)                                                     //Convert from host byte order to big-endian byte order
   {
      context->h[i] = htobe32(context->h[i]);
   }

   if(digest != NULL)                                                           //Copy the resulting digest
      memcpy((void*)digest,(void*) context->digest,(int16_t) SHA256_DIGEST_SIZE);
}
#endif   /* end sha256 hash function */
#if defined(ADV_ENCPT_STD_USED)                                                 /* yarrow needs AES as the ciper */
/*-----------------------------------------------------------------------------
 *      osDeleteMutex():  create a simulated mutex
 *
 *  Parameters: YarrowContext_t *context
 *
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t osCreateMutex(YarrowContext_t *context)
{
    if (context==NULL) return 0u;
    
    if (context->mutex==PRNG_COMPLETE)
    {
        context->mutex=PRNG_START;
    }
    return context->mutex;
}
/*-----------------------------------------------------------------------------
 *      osDeleteMutex():  clear simulated mutex
 *
 *  Parameters: YarrowContext_t *context
 *
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t osDeleteMutex(YarrowContext_t *context)
{
    if (context==NULL) return 0u;

    if (context->mutex==PRNG_START)
    {
        context->mutex=PRNG_COMPLETE;
    }
    return context->mutex;
}
/*-----------------------------------------------------------------------------
 *      yarrowInit():  initialise the yarrow psudo random number generator
 *
 *  Parameters: YarrowContext_t *context
 *
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t yarrowInit(YarrowContext_t *context)
{

   if (context==NULL) return 0u;
   
   memset((void*)context,(void*) 0,(int16_t) sizeof(YarrowContext_t));          //Clear PRNG state
   if(osCreateMutex(context)!=PRNG_START)                                       //Create a mutex to prevent simultaneous access to the PRNG state
   {
      return 3u;                                                                //Failed to create mutex
   }

   sha256Init(&context->fastPool);                                              //Initialize hash contexts
   sha256Init(&context->slowPool);

   context->ready = FALSE;                                                      //The PRNG is not ready to generate random data

   return 1u;                                                                   //Successful initialization
}
/**
 * @brief Release PRNG context
 * @param[in] context Pointer to the PRNG context
 **/

void yarrowRelease(YarrowContext_t *context)
{
   osDeleteMutex(context);                                                      //Release previously allocated resources
   memset((void*)context,(void*) 0,(int16_t) sizeof(YarrowContext_t));          //Clear PRNG state
}
/**
 * @brief Reseed from the fast pool
 * @param[in] context Pointer to the PRNG context
 **/
void yarrowFastReseed(YarrowContext_t *context)
{
   size_t i;

   if ( context != NULL )
   {
      //Reseeding from the fast pool use the current key and the hash of all
      //inputs to the fast pool since the last reseed, to generate a new key
      sha256Update(&context->fastPool, context->key, sizeof(context->key));
      sha256Final(&context->fastPool, context->key);

      aesInit(&context->cipherContext, context->key, sizeof(context->key));        //Set the new key

      //Define the new value of the counter
      memset((void*)context->counter,(void*) 0,(int16_t) sizeof(context->counter));
      aesEncryptBlock(&context->cipherContext, context->counter, context->counter);

      //Reset the hash context
      sha256Init(&context->fastPool);

      //The entropy estimates for the fast pool are all reset to zero
      for(i = 0; i < YARROW_N; i++)
      {
        context->fastPoolEntropy[i] = 0u;
      }

      //The PRNG is ready to generate random data
      context->ready = TRUE;
   } else { /* for misra */ }
}
/**
 * @brief Generate a random block of data
 * @param[in] context Pointer to the PRNG context
 * @param[out] output Buffer where to store the output block
 **/
void yarrowGenerateBlock(YarrowContext_t *context, uint8_t *output)
{
   int16_t i;

   if (context != NULL)
   {
      aesEncryptBlock(&context->cipherContext, context->counter, output);       //Encrypt counter block
      for(i = AES_BLOCK_SIZE - 1; i >= 0; i--)                                  //Increment counter value
      {
         if(++(context->counter[i]) != 0u)                                      //Increment the current byte and propagate the carry if necessary
            break;
      }
   } else { /* for misra */ }
}
/**
 * @brief Seed the PRNG state
 * @param[in] context Pointer to the PRNG context
 * @param[in] input Pointer to the input data
 * @param[in] length Length of the input data
 * @return Error code
 **/
uint8_t yarrowSeed(YarrowContext_t *context, const uint8_t *input, size_t length)
{
   if ((length < sizeof(context->key)) || (context == NULL))                    //Check parameters
      return 0u;

   sha256Update(&context->fastPool, input, length);                             //Add entropy to the fast pool
   yarrowFastReseed(context);                                                   //Reseed from the fast pool
   return 1u;                                                                   //Successful processing
}
/**
 * @brief Read random data
 * @param[in] context Pointer to the PRNG context
 * @param[out] output Buffer where to store the output data
 * @param[in] length Desired length in bytes
 * @return Error code
 **/
uint8_t yarrowRead(YarrowContext_t *context, uint8_t *output, size_t length)
{
   size_t n;
   uint8_t buffer[AES_BLOCK_SIZE];

   if (context == NULL)
      return 0u;
   else if(!context->ready)                                                     //Make sure that the PRNG has been properly seeded
      return 4u;

   //Acquire exclusive access to the PRNG state
   //osAcquireMutex(&context->mutex);

   while(length > 0)                                                            //Generate random data in a block-by-block fashion
   {
      n = min(length, AES_BLOCK_SIZE);                                          //Number of bytes to process at a time

      yarrowGenerateBlock(context, buffer);                                     //Generate a random block
      memcpy((void*)output,(void*) buffer,(int16_t) n);                         //Copy data to the output buffer

      context->blockCount++;                                                    //We keep track of how many blocks we have output

      output += n;                                                              //Next block
      length -= n;
   }

   if(context->blockCount >= YARROW_PG)                                         //Apply generator gate?
   {
      yarrowGenerateBlock(context, context->key);                               //Generate some random bytes
      aesInit(&context->cipherContext, context->key, sizeof(context->key));     //Use them as the new key
      context->blockCount = 0;                                                  //Reset block counter
   }

   //Release exclusive access to the PRNG state
   //osReleaseMutex(&context->mutex);

   return 1u;                                                                   //Successful processing
}
/**
 * @brief Reseed from the slow pool
 * @param[in] context Pointer to the PRNG context
 **/

void yarrowSlowReseed(YarrowContext_t *context)
{
   size_t i;

   if (context == NULL)
     return;
     
   sha256Final(&context->fastPool, NULL);                                       //Compute the hash of all inputs to the fast pool

   //Reseeding from the slow pool use the current key, the hash of all inputs to the
   //fast pool and the hash of all inputs to the slow pool, to generate a new key
   sha256Update(&context->slowPool, context->key, sizeof(context->key));
   sha256Update(&context->slowPool, context->fastPool.digest, SHA256_DIGEST_SIZE);
   sha256Final(&context->slowPool, context->key);

   aesInit(&context->cipherContext, context->key, sizeof(context->key));        //Set the new key

   memset((void*)context->counter,(void*) 0,(int16_t) sizeof(context->counter));     //Define the new value of the counter
   aesEncryptBlock(&context->cipherContext, context->counter, context->counter);

   sha256Init(&context->fastPool);                                              //Reset the hash contexts
   sha256Init(&context->slowPool);

   for(i = 0; i < YARROW_N; i++)                                                //The entropy estimates for both pools are reset to zero
   {
      context->fastPoolEntropy[i] = 0u;
      context->slowPoolEntropy[i] = 0u;
   }

   context->ready = TRUE;                                                       //The PRNG is ready to generate random data
}
/**
 * @brief Add entropy to the PRNG state
 * @param[in] context Pointer to the PRNG context
 * @param[in] source Entropy source identifier
 * @param[in] input Pointer to the input data
 * @param[in] length Length of the input data
 * @param[in] entropy Actual number of bits of entropy
 * @return Error code
 **/
uint8_t yarrowAddEntropy(YarrowContext_t *context, uint8_t source,  const uint8_t *input, size_t length, size_t entropy)
{
   uint8_t i;
   uint8_t k;
     
   if ((source >= YARROW_N) || ((context == NULL) || (input == NULL)))          //Check parameters
      return 0u;

   //Acquire exclusive access to the PRNG state
   //osAcquireMutex(&context->mutex);

   if(context->currentPool[source] == YARROW_FAST_POOL_ID)                      //Entropy from samples are collected into two pools
   {
      sha256Update(&context->fastPool,(void*) input, length);                   //Each pool contains running hash of all inputs since last reseed
      context->fastPoolEntropy[source] += entropy;                              //Estimate the amount of entropy we have collected thus far

      if(context->fastPoolEntropy[source] >= YARROW_FAST_THRESHOLD)             //Reseed when any source estimate reaches 100 bits
         yarrowFastReseed(context);

      context->currentPool[source] = YARROW_SLOW_POOL_ID;                       //The samples from each source alternate between the two pools
   }
   else
   {
      sha256Update(&context->slowPool, input, length);                          //Each pool contains running hash of all inputs since last reseed
      context->slowPoolEntropy[source] += entropy;                              //Estimate the amount of entropy we have collected thus far

      if(context->slowPoolEntropy[source] >= YARROW_SLOW_THRESHOLD)             //Prevent overflows while adding up the entropy estimate
         context->slowPoolEntropy[source] = YARROW_SLOW_THRESHOLD;

      for(k = 0, i = 0; i < YARROW_N; i++)                                      //At least two different sources must be over 160 bits in the slow pool before the slow pool reseeds
      {
         if(context->slowPoolEntropy[i] >= YARROW_SLOW_THRESHOLD)               //Check whether the current source has hit the threshold
            k++;
      }

      if(k >= YARROW_K)                                                         //Reseed from the slow pool?
         yarrowSlowReseed(context);

      context->currentPool[source] = YARROW_FAST_POOL_ID;                       //The samples from each source alternate between the two pools
   }

   //Release exclusive access to the PRNG state
   //osReleaseMutex(&context->mutex);

   return 1u;                                                                   //Successful processing
}
/*-----------------------------------------------------------------------------
 *      yarrowBegin():  start the yarrow psudo random number generator
 *
 *  Parameters: YarrowContext_t yarrowContext, uint8_t seed[32u]
 *
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t yarrowBegin(YarrowContext_t yarrowContext, uint8_t seed[32u])
{
   uint64_t value1;
   uint64_t value2;
   uint8_t i;
   uint8_t error;

   //RNGCON |= _RNGCON_TRNGEN_MASK;       //Enable TRNG
   srand(g_timer5_counter % UINT16_MAX);                                        // start the random generator from here srand(9u);

   for(i = 0; i < 32; i += 8)                                                   //Generate a random seed
   {
      //Wait for the RNG to contain a valid data
      //while(RNGCNT < 64);

      value2 = (rand()<<48u) | (rand()<<32u) | (rand()<<16u) | rand();          //Get 64-bit random value
      value1 = (rand()<<48u) | (rand()<<32u) | (rand()<<16u) | rand();

      seed[i] = value1 & 0xFFu;                                                 //Copy random value
      seed[i + 1u] = (value1 >> 8u) & 0xFFu;
      seed[i + 2u] = (value1 >> 16u) & 0xFFu;
      seed[i + 3u] = (value1 >> 24u) & 0xFFu;
      seed[i + 4u] = value2 & 0xFFu;
      seed[i + 5u] = (value2 >> 8u) & 0xFFu;
      seed[i + 6u] = (value2 >> 16u) & 0xFFu;
      seed[i + 7u] = (value2 >> 24u) & 0xFFu;
   }
   error = yarrowInit(&yarrowContext);                                          //PRNG initialization
   if(error==1u)                                                                //init was successful
   {
     error = yarrowSeed(&yarrowContext, seed, sizeof(seed));                    //Properly seed the PRNG
   }
   return error;

}
// This function adds the round key to state.
// The round key is added to the state by an XOR function.
void AddRoundKey(uint8_t round, aes_state_t* state, const uint8_t* RoundKey)
{
  uint8_t i,j;
  
  if ((state == NULL) || (RoundKey == NULL))
     return;
     
  for (i = 0u; i < 4u; ++i)
  {
    for (j = 0u; j < 4u; ++j)
    {
      (*state)[i][j] ^= RoundKey[(round * AES_Nb * 4u) + (i * AES_Nb) + j];
    }
  }
}
// The SubBytes Function Substitutes the values in the
// state matrix with values in an S-box.
static void SubBytes(aes_state_t* state)
{
  uint8_t i, j;
  
  if (state == NULL) 
     return;
     
  for (i = 0u; i < 4u; ++i)
  {
    for (j = 0u; j < 4u; ++j)
    {
      (*state)[j][i] = sbox[((*state)[j][i])];
    }
  }
}
// The ShiftRows() function shifts the rows in the state to the left.
// Each row is shifted with different offset.
// Offset = Row number. So the first row is not shifted.
static void ShiftRows(aes_state_t* state)
{
  uint8_t temp;

  if (state == NULL) 
     return;
     
  temp = (*state)[0u][1u];                                                      // Rotate first row 1 columns to left
  (*state)[0u][1u] = (*state)[1u][1u];
  (*state)[1u][1u] = (*state)[2u][1u];
  (*state)[2u][1u] = (*state)[3u][1u];
  (*state)[3u][1u] = temp;

  temp = (*state)[0u][2u];                                                      // Rotate second row 2 columns to left
  (*state)[0u][2u] = (*state)[2u][2u];
  (*state)[2u][2u] = temp;

  temp = (*state)[1u][2u];
  (*state)[1u][2u] = (*state)[3u][2u];
  (*state)[3u][2u] = temp;

  temp = (*state)[0u][3u];                                                      // Rotate third row 3 columns to left
  (*state)[0u][3u] = (*state)[3u][3u];
  (*state)[3u][3u] = (*state)[2u][3u];
  (*state)[2u][3u] = (*state)[1u][3u];
  (*state)[1u][3u] = temp;
}
/*-----------------------------------------------------------------------------
 *      xtime(): convert byte format 
 *
 *  Parameters: uint8_t x
 *
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
static uint8_t xtime(uint8_t x)
{
  return ((x<<1) ^ (((x>>7u) & 1u) * 0x1bu));
}
 
/*-----------------------------------------------------------------------------
 *      MixColumns(): MixColumns function mixes the columns of the state matrix
 *
 *  Parameters: aes_state_t* state
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void MixColumns(aes_state_t* state)
{
  uint8_t i;
  uint8_t Tmp, Tm, t;
  
  if (state == NULL) 
     return;
     
  for (i = 0u; i < 4u; ++i)
  {
    t   = (*state)[i][0u];
    Tmp = (*state)[i][0u] ^ (*state)[i][1u] ^ (*state)[i][2u] ^ (*state)[i][3u] ;
    Tm  = (*state)[i][0u] ^ (*state)[i][1u] ; Tm = xtime(Tm);  (*state)[i][0u] ^= Tm ^ Tmp ;
    Tm  = (*state)[i][1u] ^ (*state)[i][2u] ; Tm = xtime(Tm);  (*state)[i][1u] ^= Tm ^ Tmp ;
    Tm  = (*state)[i][2u] ^ (*state)[i][3u] ; Tm = xtime(Tm);  (*state)[i][2u] ^= Tm ^ Tmp ;
    Tm  = (*state)[i][3u] ^ t ;              Tm = xtime(Tm);  (*state)[i][3u] ^= Tm ^ Tmp ;
  }
} 
/*-----------------------------------------------------------------------------
 *      Cipher(): Cipher is the main function that encrypts the PlainText.
 *
 *  Parameters: aes_state_t* state, const uint8_t* RoundKey
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void Cipher(aes_state_t* state, const uint8_t* RoundKey)
{
  uint8_t round = 0;

  if ((state == NULL) || (RoundKey == NULL)) 
     return;
     
  AddRoundKey(0, state, RoundKey);                                              // Add the First round key to the state before starting the rounds.

  // There will be Nr rounds.
  // The first Nr-1 rounds are identical.
  // These Nr rounds are executed in the loop below.
  // Last one without MixColumns()
  for (round = 1u; ; ++round)
  {
    SubBytes(state);
    ShiftRows(state);
    if (round == AES_Nr) {
      break;
    }
    MixColumns(state);
    AddRoundKey(round, state, RoundKey);
  }

  AddRoundKey(AES_Nr, state, RoundKey);                                         // Add round key to last round
}
/* ============== CTR ================================================== */
/* Symmetrical operation: same function for encrypting as for decrypting. Note any IV/nonce should never be reused with the same key */
/*-----------------------------------------------------------------------------
 *      AES_CTR_xcrypt_buffer(): Symmetrical operation: same function for encrypting as for decrypting. 
 *      Note any IV/nonce should never be reused with the same key
 *
 *  Parameters: AES_ctx_t* ctx, uint8_t* buf, uint32_t length
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void AES_CTR_xcrypt_buffer(AES_ctx_t* ctx, uint8_t* buf, uint32_t length)
{
  uint8_t buffer[AES_BLOCKLEN];
  uint8_t i;
  int16_t bi;
  
  if ((ctx == NULL) || (buf == NULL)) 
     return;
     
  for (i = 0u, bi = AES_BLOCKLEN; i < length; ++i, ++bi)
  {
    if (bi == AES_BLOCKLEN)                                                     /* we need to regen xor compliment in buffer */
    {
      memcpy(buffer, ctx->Iv, AES_BLOCKLEN);
      Cipher((aes_state_t*)buffer,ctx->RoundKey);

      /* Increment Iv and handle overflow */
      for (bi = (AES_BLOCKLEN - 1); bi >= 0; --bi)
      {
        if (ctx->Iv[bi] == 255u)                                                /* inc will overflow */
        {
          ctx->Iv[bi] = 0u;
          continue;
        }
        ctx->Iv[bi] += 1u;
        break;
      }
      bi = 0;
    }

    buf[i] = (buf[i] ^ buffer[bi]);
  }
}
/* =================== CBC ============================== */
/*-----------------------------------------------------------------------------
 *      XorWithIv(): Xor with the Iv key 
 *      Note any IV/nonce should never be reused with the same key
 *
 *  Parameters: uint8_t* buf, const uint8_t* Iv
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void XorWithIv(uint8_t* buf, const uint8_t* Iv)
{
  uint8_t i;
  
  if ((Iv == NULL) || (buf == NULL)) 
     return;
     
  for (i = 0; i < AES_BLOCKLEN; ++i)                                            /* The block in AES is always 128bit no matter the key size */
  {
    buf[i] ^= Iv[i];
  }
}
/*-----------------------------------------------------------------------------
 *      AES_CBC_encrypt_buffer(): AES CBC encrpytion of buffer 
 *
 *  Parameters: AES_ctx_t *ctx, uint8_t* buf, uint32_t length
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void AES_CBC_encrypt_buffer(AES_ctx_t *ctx, uint8_t* buf, uint32_t length)
{
  uintptr_t i;
  uint8_t *Iv;
  
  if ((ctx == NULL) || (buf == NULL)) 
     return;
  Iv = ctx->Iv;
  
  for (i = 0; i < length; i += AES_BLOCKLEN)
  {
    XorWithIv(buf, Iv);
    Cipher((aes_state_t*)buf, ctx->RoundKey);
    Iv = buf;
    buf += AES_BLOCKLEN;
  }

  memcpy(ctx->Iv, Iv, AES_BLOCKLEN);                                            /* store Iv in ctx for next call */
}
// Multiply is used to multiply numbers in the field GF(2^8)
// Note: The last call to xtime() is unneeded, but often ends up generating a smaller binary
//       The compiler seems to be able to vectorize the operation better this way.
//       See https://github.com/kokke/tiny-AES-c/pull/34
#if MULTIPLY_AS_A_FUNCTION
static uint8_t AES_Multiply(uint8_t x, uint8_t y)
{
  return (((y & 1) * x) ^
       ((y>>1 & 1) * xtime(x)) ^
       ((y>>2 & 1) * xtime(xtime(x))) ^
       ((y>>3 & 1) * xtime(xtime(xtime(x)))) ^
       ((y>>4 & 1) * xtime(xtime(xtime(xtime(x))))));                           /* this last call to xtime() can be omitted */
  }
#else
#define AES_Multiply(x, y)                                \
      (  ((y & 1) * x) ^                              \
      ((y>>1 & 1) * xtime(x)) ^                       \
      ((y>>2 & 1) * xtime(xtime(x))) ^                \
      ((y>>3 & 1) * xtime(xtime(xtime(x)))) ^         \
      ((y>>4 & 1) * xtime(xtime(xtime(xtime(x))))))   \

#endif
// MixColumns function mixes the columns of the state matrix.
// The method used to multiply may be difficult to understand for the inexperienced.
// Please use the references to gain more information.
static void InvMixColumns(aes_state_t* state)
{
  int16_t i;
  uint8_t a, b, c, d;
  
  if (state == NULL)  
     return;
     
  for (i = 0; i < 4; ++i)
  {
    a = (*state)[i][0u];
    b = (*state)[i][1u];
    c = (*state)[i][2u];
    d = (*state)[i][3u];

    (*state)[i][0u] = AES_Multiply(a, 0x0eu) ^ AES_Multiply(b, 0x0bu) ^ AES_Multiply(c, 0x0du) ^ AES_Multiply(d, 0x09u);
    (*state)[i][1u] = AES_Multiply(a, 0x09u) ^ AES_Multiply(b, 0x0eu) ^ AES_Multiply(c, 0x0bu) ^ AES_Multiply(d, 0x0du);
    (*state)[i][2u] = AES_Multiply(a, 0x0du) ^ AES_Multiply(b, 0x09u) ^ AES_Multiply(c, 0x0eu) ^ AES_Multiply(d, 0x0bu);
    (*state)[i][3u] = AES_Multiply(a, 0x0bu) ^ AES_Multiply(b, 0x0du) ^ AES_Multiply(c, 0x09u) ^ AES_Multiply(d, 0x0eu);
  }
}
/*-----------------------------------------------------------------------------
 *      InvShiftRows(): invert shift rows hash function 
 *
 *  Parameters: aes_state_t* state
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void InvShiftRows(aes_state_t* state)
{
  uint8_t temp;

  if (state == NULL)  
     return;
     
  temp = (*state)[3u][1u];                                                      // Rotate first row 1 columns to right
  (*state)[3u][1u] = (*state)[2u][1u];
  (*state)[2u][1u] = (*state)[1u][1u];
  (*state)[1u][1u] = (*state)[0u][1u];
  (*state)[0u][1u] = temp;

  temp = (*state)[0u][2u];                                                      // Rotate second row 2 columns to right
  (*state)[0u][2u] = (*state)[2u][2u];
  (*state)[2u][2u] = temp;

  temp = (*state)[1u][2u];
  (*state)[1u][2] = (*state)[3u][2u];
  (*state)[3u][2] = temp;

  temp = (*state)[0u][3u];                                                      // Rotate third row 3 columns to right
  (*state)[0u][3u] = (*state)[1u][3u];
  (*state)[1u][3u] = (*state)[2u][3u];
  (*state)[2u][3u] = (*state)[3u][3u];
  (*state)[3u][3u] = temp;
}

/*-----------------------------------------------------------------------------
 *      InvSubBytes(): The SubBytes Function Substitutes the values in the  
 *                     state matrix with values in an S-box.
 *  Parameters: aes_state_t* state
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void InvSubBytes(aes_state_t* state)
{
  uint8_t i, j;
  
  if (state == NULL)  
     return;
     
  for (i = 0u; i < 4u; ++i)
  {
    for (j = 0u; j < 4u; ++j)
    {
      (*state)[j][i] = isbox[((*state)[j][i])];
    }
  }
}
/*-----------------------------------------------------------------------------
 *      InvCipher(): invert Cipher  
 *
 *  Parameters: aes_state_t* state, const uint8_t* RoundKey
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void InvCipher(aes_state_t* state, const uint8_t* RoundKey)
{
  uint8_t round = 0u;

  if ((state == NULL) || (RoundKey == NULL)) 
     return;
       
  AddRoundKey(AES_Nr, state, RoundKey);                                         // Add the First round key to the state before starting the rounds.

  // There will be Nr rounds.
  // The first Nr-1 rounds are identical.
  // These Nr rounds are executed in the loop below.
  // Last one without InvMixColumn()
  for (round = (AES_Nr - 1u); ; --round)
  {
    InvShiftRows(state);
    InvSubBytes(state);
    AddRoundKey(round, state, RoundKey);
    if (round == 0u) {
      break;
    }
    InvMixColumns(state);
  }

}
/*-----------------------------------------------------------------------------
 *      AES_CBC_decrypt_buffer(): AES CBC decrypt  
 *
 *  Parameters: AES_ctx_t* ctx, uint8_t* buf,  uint32_t length
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void AES_CBC_decrypt_buffer(AES_ctx_t* ctx, uint8_t* buf,  uint32_t length)
{
  uintptr_t i;
  uint8_t storeNextIv[AES_BLOCKLEN];
  
  if ((ctx == NULL) || (buf == NULL)) 
     return;
     
  for (i = 0u; i < length; i += AES_BLOCKLEN)
  {
    memcpy(storeNextIv, buf, AES_BLOCKLEN);
    InvCipher((aes_state_t*)buf, ctx->RoundKey);
    XorWithIv(buf, ctx->Iv);
    memcpy(ctx->Iv, storeNextIv, AES_BLOCKLEN);
    buf += AES_BLOCKLEN;
  }
}

/* ---------------- ECB AES ----------------------------------------------- */
/*-----------------------------------------------------------------------------
 *      AES_ECB_encrypt(): AES CBC decrypt  
 *
 *  Parameters: const AES_ctx_t* ctx, uint8_t* buf
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void AES_ECB_encrypt(const AES_ctx_t* ctx, uint8_t* buf)
{
  if ((ctx == NULL) || (buf == NULL)) 
  { /* for misra */ }
  else
  {
     Cipher((aes_state_t*)buf, ctx->RoundKey);                                  /* The next function call encrypts the PlainText with the Key using AES algorithm. */
  }
}
/*-----------------------------------------------------------------------------
 *      AES_ECB_decrypt(): AES ECB decrypt  
 *
 *  Parameters: const AES_ctx_t* ctx, uint8_t* buf
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void AES_ECB_decrypt(const AES_ctx_t* ctx, uint8_t* buf)
{
  if ((ctx == NULL) || (buf == NULL)) 
  { /* for misra */ }
  else
  {
     InvCipher((aes_state_t*)buf, ctx->RoundKey);                               /* The next function call decrypts the PlainText with the Key using AES algorithm. */
  }
}

#endif /* _________ end YARROW PsuedoRandomNumGen and AES ________________ */

/* https://github.com/AristodamusAdairs/Penetrationcoin/blob/main/src/crypto/sha512.cpp */
/* ported by ACP Aviation                                                               */

// Copyright (c) 2014 The Bitcoin Core developers
// Distributed under the MIT software license, see the accompanying
// file COPYING or http://www.opensource.org/licenses/mit-license.php.

///Internal SHA-512 implementation.

#if defined(USE_SHA_512)

/* ================ functions used locally only ============================  */
uint64_t SHA512_Ch(uint64_t x, uint64_t y, uint64_t z) { return z ^ (x & (y ^ z)); }
uint64_t SHA512_Maj(uint64_t x, uint64_t y, uint64_t z) { return (x & y) | (z & (x | y)); }
uint64_t SHA512_SigmaB0(uint64_t x) { return (x >> 28 | x << 36) ^ (x >> 34 | x << 30) ^ (x >> 39 | x << 25); }
uint64_t SHA512_SigmaB1(uint64_t x) { return (x >> 14 | x << 50) ^ (x >> 18 | x << 46) ^ (x >> 41 | x << 23); }
uint64_t SHA512_sigmaL0(uint64_t x) { return (x >> 1 | x << 63) ^ (x >> 8 | x << 56) ^ (x >> 7); }
uint64_t SHA512_sigmaL1(uint64_t x) { return (x >> 19 | x << 45) ^ (x >> 61 | x << 3) ^ (x >> 6); }

/* ============== external functions for SHA512 ============================  */
/*-----------------------------------------------------------------------------
 *  SHA512_Round():  One Round of SHA-512.
 *
 *  Parameters: uint64_t a, uint64_t b, uint64_t c, uint64_t* d, uint64_t e, 
 *              uint64_t f, uint64_t g, uint64_t* h, uint64_t k, uint64_t w
 *
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/
void SHA512_Round(uint64_t a, uint64_t b, uint64_t c, uint64_t* d, uint64_t e, uint64_t f, uint64_t g, uint64_t* h, uint64_t k, uint64_t w )
{
    uint64_t t1 = *h + SHA512_SigmaB1(e) + SHA512_Ch(e, f, g) + k + w;
    uint64_t t2 = SHA512_SigmaB0(a) + SHA512_Maj(a, b, c);
    *d += t1;
    *h = t1 + t2;
}

/** Initialize SHA-256 state. */
/*-----------------------------------------------------------------------------
 *  SHA512_Initialize():  sha512 initialize 
 *
 *  Parameters: uint64_t* s
 *
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/
void SHA512_Initialize(uint64_t* s)
{
    s[0] = 0x6a09e667f3bcc908ull;
    s[1] = 0xbb67ae8584caa73bull;
    s[2] = 0x3c6ef372fe94f82bull;
    s[3] = 0xa54ff53a5f1d36f1ull;
    s[4] = 0x510e527fade682d1ull;
    s[5] = 0x9b05688c2b3e6c1full;
    s[6] = 0x1f83d9abfb41bd6bull;
    s[7] = 0x5be0cd19137e2179ull;
}

/*-----------------------------------------------------------------------------
 *  SHA512_ReadBE64():  sha512 read big endian 64 bit  
 *
 *  Parameters: const unsigned char* ptr
 *
 *  Return: uint64_t
 *             
 *----------------------------------------------------------------------------*/
uint64_t SHA512_ReadBE64(const unsigned char* ptr)
{
    uint64_t x;
    memcpy((char*)&x, (char*)ptr, 8);
    return betoh64(x);
}

/** Perform one SHA-512 transformation, processing a 128-byte chunk. */
/*-----------------------------------------------------------------------------
 *  SHA512_Transform():  sha512 write 
 *
 *  Parameters: uint64_t* s, const unsigned char* chunk
 *
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/
void SHA512_Transform(uint64_t* s, const unsigned char* chunk)
{
    uint64_t a = s[0], b = s[1], c = s[2], d = s[3], e = s[4], f = s[5], g = s[6], h = s[7];
    uint64_t w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15;

    SHA512_Round(a, b, c, &d, e, f, g, &h, 0x428a2f98d728ae22ull, w0 = SHA512_ReadBE64(chunk + 0));
    SHA512_Round(h, a, b, &c, d, e, f, &g, 0x7137449123ef65cdull, w1 = SHA512_ReadBE64(chunk + 8));
    SHA512_Round(g, h, a, &b, c, d, e, &f, 0xb5c0fbcfec4d3b2full, w2 = SHA512_ReadBE64(chunk + 16));
    SHA512_Round(f, g, h, &a, b, c, d, &e, 0xe9b5dba58189dbbcull, w3 = SHA512_ReadBE64(chunk + 24));
    SHA512_Round(e, f, g, &h, a, b, c, &d, 0x3956c25bf348b538ull, w4 = SHA512_ReadBE64(chunk + 32));
    SHA512_Round(d, e, f, &g, h, a, b, &c, 0x59f111f1b605d019ull, w5 = SHA512_ReadBE64(chunk + 40));
    SHA512_Round(c, d, e, &f, g, h, a, &b, 0x923f82a4af194f9bull, w6 = SHA512_ReadBE64(chunk + 48));
    SHA512_Round(b, c, d, &e, f, g, h, &a, 0xab1c5ed5da6d8118ull, w7 = SHA512_ReadBE64(chunk + 56));
    SHA512_Round(a, b, c, &d, e, f, g, &h, 0xd807aa98a3030242ull, w8 = SHA512_ReadBE64(chunk + 64));
    SHA512_Round(h, a, b, &c, d, e, f, &g, 0x12835b0145706fbeull, w9 = SHA512_ReadBE64(chunk + 72));
    SHA512_Round(g, h, a, &b, c, d, e, &f, 0x243185be4ee4b28cull, w10 = SHA512_ReadBE64(chunk + 80));
    SHA512_Round(f, g, h, &a, b, c, d, &e, 0x550c7dc3d5ffb4e2ull, w11 = SHA512_ReadBE64(chunk + 88));
    SHA512_Round(e, f, g, &h, a, b, c, &d, 0x72be5d74f27b896full, w12 = SHA512_ReadBE64(chunk + 96));
    SHA512_Round(d, e, f, &g, h, a, b, &c, 0x80deb1fe3b1696b1ull, w13 = SHA512_ReadBE64(chunk + 104));
    SHA512_Round(c, d, e, &f, g, h, a, &b, 0x9bdc06a725c71235ull, w14 = SHA512_ReadBE64(chunk + 112));
    SHA512_Round(b, c, d, &e, f, g, h, &a, 0xc19bf174cf692694ull, w15 = SHA512_ReadBE64(chunk + 120));

    SHA512_Round(a, b, c, &d, e, f, g, &h, 0xe49b69c19ef14ad2ull, w0 += SHA512_sigmaL1(w14) + w9 + SHA512_sigmaL0(w1));
    SHA512_Round(h, a, b, &c, d, e, f, &g, 0xefbe4786384f25e3ull, w1 += SHA512_sigmaL1(w15) + w10 + SHA512_sigmaL0(w2));
    SHA512_Round(g, h, a, &b, c, d, e, &f, 0x0fc19dc68b8cd5b5ull, w2 += SHA512_sigmaL1(w0) + w11 + SHA512_sigmaL0(w3));
    SHA512_Round(f, g, h, &a, b, c, d, &e, 0x240ca1cc77ac9c65ull, w3 += SHA512_sigmaL1(w1) + w12 + SHA512_sigmaL0(w4));
    SHA512_Round(e, f, g, &h, a, b, c, &d, 0x2de92c6f592b0275ull, w4 += SHA512_sigmaL1(w2) + w13 + SHA512_sigmaL0(w5));
    SHA512_Round(d, e, f, &g, h, a, b, &c, 0x4a7484aa6ea6e483ull, w5 += SHA512_sigmaL1(w3) + w14 + SHA512_sigmaL0(w6));
    SHA512_Round(c, d, e, &f, g, h, a, &b, 0x5cb0a9dcbd41fbd4ull, w6 += SHA512_sigmaL1(w4) + w15 + SHA512_sigmaL0(w7));
    SHA512_Round(b, c, d, &e, f, g, h, &a, 0x76f988da831153b5ull, w7 += SHA512_sigmaL1(w5) + w0 + SHA512_sigmaL0(w8));
    SHA512_Round(a, b, c, &d, e, f, g, &h, 0x983e5152ee66dfabull, w8 += SHA512_sigmaL1(w6) + w1 + SHA512_sigmaL0(w9));
    SHA512_Round(h, a, b, &c, d, e, f, &g, 0xa831c66d2db43210ull, w9 += SHA512_sigmaL1(w7) + w2 + SHA512_sigmaL0(w10));
    SHA512_Round(g, h, a, &b, c, d, e, &f, 0xb00327c898fb213full, w10 += SHA512_sigmaL1(w8) + w3 + SHA512_sigmaL0(w11));
    SHA512_Round(f, g, h, &a, b, c, d, &e, 0xbf597fc7beef0ee4ull, w11 += SHA512_sigmaL1(w9) + w4 + SHA512_sigmaL0(w12));
    SHA512_Round(e, f, g, &h, a, b, c, &d, 0xc6e00bf33da88fc2ull, w12 += SHA512_sigmaL1(w10) + w5 + SHA512_sigmaL0(w13));
    SHA512_Round(d, e, f, &g, h, a, b, &c, 0xd5a79147930aa725ull, w13 += SHA512_sigmaL1(w11) + w6 + SHA512_sigmaL0(w14));
    SHA512_Round(c, d, e, &f, g, h, a, &b, 0x06ca6351e003826full, w14 += SHA512_sigmaL1(w12) + w7 + SHA512_sigmaL0(w15));
    SHA512_Round(b, c, d, &e, f, g, h, &a, 0x142929670a0e6e70ull, w15 += SHA512_sigmaL1(w13) + w8 + SHA512_sigmaL0(w0));

    SHA512_Round(a, b, c, &d, e, f, g, &h, 0x27b70a8546d22ffcull, w0 += SHA512_sigmaL1(w14) + w9 + SHA512_sigmaL0(w1));
    SHA512_Round(h, a, b, &c, d, e, f, &g, 0x2e1b21385c26c926ull, w1 += SHA512_sigmaL1(w15) + w10 + SHA512_sigmaL0(w2));
    SHA512_Round(g, h, a, &b, c, d, e, &f, 0x4d2c6dfc5ac42aedull, w2 += SHA512_sigmaL1(w0) + w11 + SHA512_sigmaL0(w3));
    SHA512_Round(f, g, h, &a, b, c, d, &e, 0x53380d139d95b3dfull, w3 += SHA512_sigmaL1(w1) + w12 + SHA512_sigmaL0(w4));
    SHA512_Round(e, f, g, &h, a, b, c, &d, 0x650a73548baf63deull, w4 += SHA512_sigmaL1(w2) + w13 + SHA512_sigmaL0(w5));
    SHA512_Round(d, e, f, &g, h, a, b, &c, 0x766a0abb3c77b2a8ull, w5 += SHA512_sigmaL1(w3) + w14 + SHA512_sigmaL0(w6));
    SHA512_Round(c, d, e, &f, g, h, a, &b, 0x81c2c92e47edaee6ull, w6 += SHA512_sigmaL1(w4) + w15 + SHA512_sigmaL0(w7));
    SHA512_Round(b, c, d, &e, f, g, h, &a, 0x92722c851482353bull, w7 += SHA512_sigmaL1(w5) + w0 + SHA512_sigmaL0(w8));
    SHA512_Round(a, b, c, &d, e, f, g, &h, 0xa2bfe8a14cf10364ull, w8 += SHA512_sigmaL1(w6) + w1 + SHA512_sigmaL0(w9));
    SHA512_Round(h, a, b, &c, d, e, f, &g, 0xa81a664bbc423001ull, w9 += SHA512_sigmaL1(w7) + w2 + SHA512_sigmaL0(w10));
    SHA512_Round(g, h, a, &b, c, d, e, &f, 0xc24b8b70d0f89791ull, w10 += SHA512_sigmaL1(w8) + w3 + SHA512_sigmaL0(w11));
    SHA512_Round(f, g, h, &a, b, c, d, &e, 0xc76c51a30654be30ull, w11 += SHA512_sigmaL1(w9) + w4 + SHA512_sigmaL0(w12));
    SHA512_Round(e, f, g, &h, a, b, c, &d, 0xd192e819d6ef5218ull, w12 += SHA512_sigmaL1(w10) + w5 + SHA512_sigmaL0(w13));
    SHA512_Round(d, e, f, &g, h, a, b, &c, 0xd69906245565a910ull, w13 += SHA512_sigmaL1(w11) + w6 + SHA512_sigmaL0(w14));
    SHA512_Round(c, d, e, &f, g, h, a, &b, 0xf40e35855771202aull, w14 += SHA512_sigmaL1(w12) + w7 + SHA512_sigmaL0(w15));
    SHA512_Round(b, c, d, &e, f, g, h, &a, 0x106aa07032bbd1b8ull, w15 += SHA512_sigmaL1(w13) + w8 + SHA512_sigmaL0(w0));

    SHA512_Round(a, b, c, &d, e, f, g, &h, 0x19a4c116b8d2d0c8ull, w0 += SHA512_sigmaL1(w14) + w9 + SHA512_sigmaL0(w1));
    SHA512_Round(h, a, b, &c, d, e, f, &g, 0x1e376c085141ab53ull, w1 += SHA512_sigmaL1(w15) + w10 + SHA512_sigmaL0(w2));
    SHA512_Round(g, h, a, &b, c, d, e, &f, 0x2748774cdf8eeb99ull, w2 += SHA512_sigmaL1(w0) + w11 + SHA512_sigmaL0(w3));
    SHA512_Round(f, g, h, &a, b, c, d, &e, 0x34b0bcb5e19b48a8ull, w3 += SHA512_sigmaL1(w1) + w12 + SHA512_sigmaL0(w4));
    SHA512_Round(e, f, g, &h, a, b, c, &d, 0x391c0cb3c5c95a63ull, w4 += SHA512_sigmaL1(w2) + w13 + SHA512_sigmaL0(w5));
    SHA512_Round(d, e, f, &g, h, a, b, &c, 0x4ed8aa4ae3418acbull, w5 += SHA512_sigmaL1(w3) + w14 + SHA512_sigmaL0(w6));
    SHA512_Round(c, d, e, &f, g, h, a, &b, 0x5b9cca4f7763e373ull, w6 += SHA512_sigmaL1(w4) + w15 + SHA512_sigmaL0(w7));
    SHA512_Round(b, c, d, &e, f, g, h, &a, 0x682e6ff3d6b2b8a3ull, w7 += SHA512_sigmaL1(w5) + w0 + SHA512_sigmaL0(w8));
    SHA512_Round(a, b, c, &d, e, f, g, &h, 0x748f82ee5defb2fcull, w8 += SHA512_sigmaL1(w6) + w1 + SHA512_sigmaL0(w9));
    SHA512_Round(h, a, b, &c, d, e, f, &g, 0x78a5636f43172f60ull, w9 += SHA512_sigmaL1(w7) + w2 + SHA512_sigmaL0(w10));
    SHA512_Round(g, h, a, &b, c, d, e, &f, 0x84c87814a1f0ab72ull, w10 += SHA512_sigmaL1(w8) + w3 + SHA512_sigmaL0(w11));
    SHA512_Round(f, g, h, &a, b, c, d, &e, 0x8cc702081a6439ecull, w11 += SHA512_sigmaL1(w9) + w4 + SHA512_sigmaL0(w12));
    SHA512_Round(e, f, g, &h, a, b, c, &d, 0x90befffa23631e28ull, w12 += SHA512_sigmaL1(w10) + w5 + SHA512_sigmaL0(w13));
    SHA512_Round(d, e, f, &g, h, a, b, &c, 0xa4506cebde82bde9ull, w13 += SHA512_sigmaL1(w11) + w6 + SHA512_sigmaL0(w14));
    SHA512_Round(c, d, e, &f, g, h, a, &b, 0xbef9a3f7b2c67915ull, w14 += SHA512_sigmaL1(w12) + w7 + SHA512_sigmaL0(w15));
    SHA512_Round(b, c, d, &e, f, g, h, &a, 0xc67178f2e372532bull, w15 += SHA512_sigmaL1(w13) + w8 + SHA512_sigmaL0(w0));

    SHA512_Round(a, b, c, &d, e, f, g, &h, 0xca273eceea26619cull, w0 += SHA512_sigmaL1(w14) + w9 + SHA512_sigmaL0(w1));
    SHA512_Round(h, a, b, &c, d, e, f, &g, 0xd186b8c721c0c207ull, w1 += SHA512_sigmaL1(w15) + w10 + SHA512_sigmaL0(w2));
    SHA512_Round(g, h, a, &b, c, d, e, &f, 0xeada7dd6cde0eb1eull, w2 += SHA512_sigmaL1(w0) + w11 + SHA512_sigmaL0(w3));
    SHA512_Round(f, g, h, &a, b, c, d, &e, 0xf57d4f7fee6ed178ull, w3 += SHA512_sigmaL1(w1) + w12 + SHA512_sigmaL0(w4));
    SHA512_Round(e, f, g, &h, a, b, c, &d, 0x06f067aa72176fbaull, w4 += SHA512_sigmaL1(w2) + w13 + SHA512_sigmaL0(w5));
    SHA512_Round(d, e, f, &g, h, a, b, &c, 0x0a637dc5a2c898a6ull, w5 += SHA512_sigmaL1(w3) + w14 + SHA512_sigmaL0(w6));
    SHA512_Round(c, d, e, &f, g, h, a, &b, 0x113f9804bef90daeull, w6 += SHA512_sigmaL1(w4) + w15 + SHA512_sigmaL0(w7));
    SHA512_Round(b, c, d, &e, f, g, h, &a, 0x1b710b35131c471bull, w7 += SHA512_sigmaL1(w5) + w0 + SHA512_sigmaL0(w8));
    SHA512_Round(a, b, c, &d, e, f, g, &h, 0x28db77f523047d84ull, w8 += SHA512_sigmaL1(w6) + w1 + SHA512_sigmaL0(w9));
    SHA512_Round(h, a, b, &c, d, e, f, &g, 0x32caab7b40c72493ull, w9 += SHA512_sigmaL1(w7) + w2 + SHA512_sigmaL0(w10));
    SHA512_Round(g, h, a, &b, c, d, e, &f, 0x3c9ebe0a15c9bebcull, w10 += SHA512_sigmaL1(w8) + w3 + SHA512_sigmaL0(w11));
    SHA512_Round(f, g, h, &a, b, c, d, &e, 0x431d67c49c100d4cull, w11 += SHA512_sigmaL1(w9) + w4 + SHA512_sigmaL0(w12));
    SHA512_Round(e, f, g, &h, a, b, c, &d, 0x4cc5d4becb3e42b6ull, w12 += SHA512_sigmaL1(w10) + w5 + SHA512_sigmaL0(w13));
    SHA512_Round(d, e, f, &g, h, a, b, &c, 0x597f299cfc657e2aull, w13 += SHA512_sigmaL1(w11) + w6 + SHA512_sigmaL0(w14));
    SHA512_Round(c, d, e, &f, g, h, a, &b, 0x5fcb6fab3ad6faecull, w14 + SHA512_sigmaL1(w12) + w7 + SHA512_sigmaL0(w15));
    SHA512_Round(b, c, d, &e, f, g, h, &a, 0x6c44198c4a475817ull, w15 + SHA512_sigmaL1(w13) + w8 + SHA512_sigmaL0(w0));

    s[0] += a;
    s[1] += b;
    s[2] += c;
    s[3] += d;
    s[4] += e;
    s[5] += f;
    s[6] += g;
    s[7] += h;
}

/*-----------------------------------------------------------------------------
 *  CSHA512_Write():  sha512 write 
 *
 *  Parameters: unsigned char* dataV, unsigned char* buf, size_t len, size_t *bytes
 *              uint64_t* s
 *
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/
void CSHA512_Write(unsigned char* dataV, unsigned char* buf, size_t len, size_t *bytes, uint64_t* s)
{
    const unsigned char* end = dataV + len;
    size_t bufsize = *bytes % 128;
    if (bufsize && bufsize + len >= 128) {
        // Fill the buffer, and process it.
        memcpy(buf + bufsize, dataV, 128 - bufsize);
        *bytes += 128 - bufsize;
        dataV += 128 - bufsize;
        SHA512_Transform(s, buf);
        bufsize = 0;
    }
    while (end >= dataV + 128) {
        // Process full chunks directly from the source.
        SHA512_Transform(s, dataV);
        dataV += 128;
        *bytes += 128;
    }
    if (end > dataV) {
        // Fill the buffer with what remains.
        memcpy(buf + bufsize, dataV, end - dataV);
        *bytes += end - dataV;
    }
}

/*-----------------------------------------------------------------------------
 *  SHA512_WriteBE64():  sha512 write big endian 64bit
 *
 *  Parameters: unsigned char* ptr, uint64_t x
 *
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/
void SHA512_WriteBE64(unsigned char* ptr, uint64_t x)
{
    uint64_t v = htobe64(x);   // cpu_endian_defs.h LOAD64BE(p)
    memcpy(ptr, (char*)&v, 8);
}

/*-----------------------------------------------------------------------------
 *  CSHA512_Finalize():  sha512 finalize
 *
 *  Parameters: unsigned char hash[SHA512_OUTPUT_SIZE], unsigned char* buf, size_t *bytes
 *              uint64_t* s
 * 
 *  Return: void
 *             
 *----------------------------------------------------------------------------*/
void CSHA512_Finalize(unsigned char hash[SHA512_OUTPUT_SIZE], unsigned char* buf, size_t *bytes, uint64_t* s)
{
    static const unsigned char pad[128] = {0x80};
    unsigned char sizedesc[16] = {0x00};
    size_t lenCalc;
    SHA512_WriteBE64(sizedesc + 8, *bytes << 3);
    lenCalc = 1 + ((239 - (*bytes % 128)) % 128);
    CSHA512_Write((unsigned char*)&pad, buf, lenCalc, bytes, s);
    CSHA512_Write(&sizedesc, buf, 16, bytes, s);
    SHA512_WriteBE64(hash, s[0]);
    SHA512_WriteBE64(hash + 8, s[1]);
    SHA512_WriteBE64(hash + 16, s[2]);
    SHA512_WriteBE64(hash + 24, s[3]);
    SHA512_WriteBE64(hash + 32, s[4]);
    SHA512_WriteBE64(hash + 40, s[5]);
    SHA512_WriteBE64(hash + 48, s[6]);
    SHA512_WriteBE64(hash + 56, s[7]);
}
#endif    /* --- end SHA512 --- */