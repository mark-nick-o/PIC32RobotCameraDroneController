// by jeremie miller - 2015,2016
// public domain UNLICENSE, contributions/improvements welcome via github at https://github.com/quartzjer/cb0r
// Ported : Copyright (C) 2020 A C P Avaiation Walkerburn Scotland

//#include <stdlib.h>
//#include <stdbool.h>
//#include <string.h>
#ifndef cb0r_h
#define cb0r_h

#include <stdint.h>
#include "definitions.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define CBORPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define CBORPACKED __attribute__((packed))                                     /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define CBORPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define CBORPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define CBORPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#ifndef size_t
#define size_t int32_t
#endif

#ifndef RODATA_SEGMENT_CONSTANT
#define RODATA_SEGMENT_CONSTANT
#endif

#if defined(USE_SHA_512)
#define SHA512_OUTPUT_SIZE 64
#endif

#ifndef bool
#define bool uint8_t
#endif
// ->count value for indefinite length items
#define CB0R_STREAM UINT32_MAX

// flattened result types
typedef enum {
  CB0R_INT = 0u  ,
  CB0R_NEG      ,
  CB0R_BYTE     ,
  CB0R_UTF8     ,
  CB0R_ARRAY    ,
  CB0R_MAP      ,
  // "unassigned" tag/simple values are this base type, rest expanded below
  CB0R_TAG      ,
  CB0R_SIMPLE   ,

  // expand standard base tags
  CB0R_TAGS = 8u , // if(type > CB0R_TAGS && type < CB0R_SIMPLES)
  CB0R_DATETIME ,
  CB0R_EPOCH    ,
  CB0R_BIGNUM   ,
  CB0R_BIGNEG   ,
  CB0R_FRACTION ,
  CB0R_BIGFLOAT ,
  CB0R_BASE64URL,
  CB0R_BASE64   ,
  CB0R_HEX      ,
  CB0R_DATA     ,

  // expand standard simple values
  CB0R_SIMPLES = 24,
  CB0R_FALSE    ,
  CB0R_TRUE     ,
  CB0R_NULL     ,
  CB0R_UNDEF    ,
  CB0R_FLOAT    ,
  // if(type >= CB0R_ERR) ...
  CB0R_ERR = 224u,
  CB0R_EPARSE   , // invalid structure
  CB0R_EBAD     , // invalid type byte
  CB0R_EBIG     , // unsupported size item
  CB0R_EMAX
} cb0r_e;

// this struct is for representing parsed data only and retains pointers to the original bytes, it's not designed for composing types from scratch
typedef struct cb0r_s
{
  uint8_t *start;
  uint8_t *end;
  union {                                                                       // different names/aliases for context readability based on type
    uint64_t length;
    uint64_t count;
    uint64_t value;
  };
  cb0r_e type:8;
  uint8_t header;                                                               // size of the type header bytes, start+head points to type's contents (if any)
} cb0r_s, *cb0r_t;

// Application Layer COAP
// Application layer DTLS (optional) )
// Transport layer UDP
// Network layer IPv6
// Network layer 6LoWPAN
// Physical layer IEEE 802.15.4
// Figure 3.1: Constrained REST stack (CoAP)
//Maximum size of CoAP messages

#define COAP_PORT 5683u                                                         //CoAP port number
#define COAPS_PORT 5684u                                                        //DTLS-secured CoAP port number
#define COAP_MAX_TOKEN_LEN 8u                                                   //Maximum acceptable length for tokens
#define COAP_PAYLOAD_MARKER 0xFFU                                               //CoAP payload marker value
#define COAP_CODE(c, d) ((((c) & 0x07U) << 5U) | ((d) & 0x1FU))                 //CoAP code definition
#define COAP_GET_CODE_CLASS(code) (((code) >> 5U) & 0x07U)                      //Get code class
#define COAP_GET_CODE_SUBCLASS(code) ((code) & 0x1FU)                           //Get code subclass

#define COAP_OPT_DELTA_8_BITS 13u                                               //Option delta encoding
#define COAP_OPT_DELTA_16_BITS 14u
#define COAP_OPT_DELTA_RESERVED 15u
#define COAP_OPT_DELTA_MINUS_8_BITS 13u
#define COAP_OPT_DELTA_MINUS_16_BITS 269u
#define COAP_OPT_LEN_8_BITS 13u                                                 //Option length encoding
#define COAP_OPT_LEN_16_BITS 14u
#define COAP_OPT_LEN_RESERVED 15u
#define COAP_OPT_LEN_MINUS_8_BITS 13u
#define COAP_OPT_LEN_MINUS_16_BITS 269u
#define COAP_DEFAULT_MAX_AGE 60u                                                //Default Max-Age option value

#define COAP_IS_OPTION_CRITICAL(num) (((num) & 0x01U) ? TRUE : FALSE)           //Test whether an option is critical
#define COAP_IS_OPTION_UNSAFE(num) (((num) & 0x02U) ? TRUE : FALSE)             //Test whether an option is unsafe to forward
#define COAP_SET_BLOCK_NUM(value, n) value = ((value) & 0x0FU) | ((n) << 4U)    //Set block number
#define COAP_SET_BLOCK_M(value, m) value = ((value) & ~0x08U) | (((m) << 3U) & 0x08U) //Set More flag
#define COAP_SET_BLOCK_SZX(value, s) value = ((value) & ~0x07U) | ((s) & 0x07U) //Set block size
#define COAP_GET_BLOCK_NUM(value) ((value) >> 4U)                               //Get block number
#define COAP_GET_BLOCK_M(value) (((value) >> 3U) & 0x01U)                       //Get More flag
#define COAP_GET_BLOCK_SZX(value) ((value) & 0x07U)                             //Get block size
#define COAP_GET_BLOCK_SIZE(value) (16U << ((value) & 0x07U))                   //Get block size (in bytes)
#define COAP_GET_BLOCK_POS(value) (((value) & ~0x0FU) << ((value) & 0x07U))     //Get block position from the beginning of the resource (in bytes)

/**
 * @brief CoAP options
 **/
typedef enum
{
   COAP_OPT_IF_MATCH       = 1,                                                 //RFC 7252
   COAP_OPT_URI_HOST       = 3,                                                 //RFC 7252
   COAP_OPT_ETAG           = 4,                                                 //RFC 7252
   COAP_OPT_IF_NONE_MATCH  = 5,                                                 //RFC 7252
   COAP_OPT_OBSERVE        = 6,                                                 //RFC 7641
   COAP_OPT_URI_PORT       = 7,                                                 //RFC 7252
   COAP_OPT_LOCATION_PATH  = 8,                                                 //RFC 7252
   COAP_OPT_URI_PATH       = 11,                                                //RFC 7252
   COAP_OPT_CONTENT_FORMAT = 12,                                                //RFC 7252
   COAP_OPT_MAX_AGE        = 14,                                                //RFC 7252
   COAP_OPT_URI_QUERY      = 15,                                                //RFC 7252
   COAP_OPT_ACCEPT         = 17,                                                //RFC 7252
   COAP_OPT_LOCATION_QUERY = 20,                                                //RFC 7252
   COAP_OPT_BLOCK2         = 23,                                                //RFC 7959
   COAP_OPT_BLOCK1         = 27,                                                //RFC 7959
   COAP_OPT_SIZE2          = 28,                                                //RFC 7959
   COAP_OPT_PROXY_URI      = 35,                                                //RFC 7252
   COAP_OPT_PROXY_SCHEME   = 39,                                                //RFC 7252
   COAP_OPT_SIZE1          = 60,                                                //RFC 7252
   COAP_OPT_NO_RESPONSE    = 258                                                //RFC 7967
} CoapOptionNumber_e;

/**
 * @brief CoAP option formats
 **/
typedef enum
{
   COAP_OPT_FORMAT_EMPTY  = 0,                                                  ///<Zero-length sequence of bytes
   COAP_OPT_FORMAT_OPAQUE = 1,                                                  ///<Opaque sequence of bytes
   COAP_OPT_FORMAT_UINT   = 2,                                                  ///<Non-negative integer
   COAP_OPT_FORMAT_STRING = 3                                                   ///<UTF-8 string
} CoapOptionFormat_e;

/**
 * @brief Observe option
 **/
typedef enum
{
   COAP_OBSERVE_REGISTER    = 0,
   COAP_OBSERVE_DEREGISTER  = 1
} CoapObserveOption_e;

/**
 * @brief Content-Format option
 **/
typedef enum
{
   COAP_CONTENT_FORMAT_TEXT_PLAIN       = 0,
   COAP_CONTENT_FORMAT_APP_LINK_FORMAT  = 40,
   COAP_CONTENT_FORMAT_APP_XML          = 41,
   COAP_CONTENT_FORMAT_APP_OCTET_STREAM = 42,
   COAP_CONTENT_FORMAT_APP_EXI          = 47,
   COAP_CONTENT_FORMAT_APP_JSON         = 50
} CoapContentFormat_e;

/**
 * @brief Block size parameter
 **/
typedef enum
{
   COAP_BLOCK_SIZE_16       = 0,
   COAP_BLOCK_SIZE_32       = 1,
   COAP_BLOCK_SIZE_64       = 2,
   COAP_BLOCK_SIZE_128      = 3,
   COAP_BLOCK_SIZE_256      = 4,
   COAP_BLOCK_SIZE_512      = 5,
   COAP_BLOCK_SIZE_1024     = 6,
   COAP_BLOCK_SIZE_RESERVED = 7,
} CoapBlockSize_e;

/**
 * @brief CoAP option
 **/
typedef struct
{
   uint16_t delta;
   uint16_t number;
   size_t length;
   const uint8_t *value;
} CoapOption_t;

/**
 * @brief Encryption algorithm type
 **/
typedef enum
{
   CIPHER_ALGO_TYPE_STREAM = 0,
   CIPHER_ALGO_TYPE_BLOCK  = 1
} CipherAlgoType_e;

/**
 * @brief Cipher operation modes
 **/
typedef enum
{
   CIPHER_MODE_NULL              = 0,
   CIPHER_MODE_STREAM            = 1,
   CIPHER_MODE_ECB               = 2,
   CIPHER_MODE_CBC               = 3,
   CIPHER_MODE_CFB               = 4,
   CIPHER_MODE_OFB               = 5,
   CIPHER_MODE_CTR               = 6,
   CIPHER_MODE_CCM               = 7,
   CIPHER_MODE_GCM               = 8,
   CIPHER_MODE_CHACHA20_POLY1305 = 9,
} CipherMode_e;

//Common API for encryption algorithms
typedef uint8_t (*CipherAlgoInit)(void *context, const uint8_t *key, size_t keyLen);
//typedef void (*CipherAlgoEncryptStream)(void *context, const uint8_t *input, uint8_t *output, size_t length);
//typedef void (*CipherAlgoDecryptStream)(void *context, const uint8_t *input, uint8_t *output, size_t length);
typedef void (*CipherAlgoEncryptBlock)(void *context, const uint8_t *input, uint8_t *output);
typedef void (*CipherAlgoDecryptBlock)(void *context, const uint8_t *input, uint8_t *output);
/**
 * @brief Common interface for encryption algorithms
 **/
typedef struct
{
   const char *name;
   size_t contextSize;
   CipherAlgoType_e type;
   size_t blockSize;
   CipherAlgoInit init;
   //CipherAlgoEncryptStream encryptStream;
   //CipherAlgoDecryptStream decryptStream;
   CipherAlgoEncryptBlock encryptBlock;
   CipherAlgoDecryptBlock decryptBlock;
} CipherAlgo_t;

/* ------------------ AES ------------------------------------------------ */
#define AES_BLOCKLEN 16 // Block length in bytes - AES is 128b block only

/* ----------------  selection ------------------------------------------- */
#define AES128 1u
//#define AES192 1
//#define AES256 1

#if defined(AES256) && (AES256 == 1)
    #define AES_KEYLEN 32u
    #define AES_keyExpSize 240u
    #define AES_Nk 8u
    #define AES_Nr 14u
#elif defined(AES192) && (AES192 == 1)
    #define AES_KEYLEN 24u
    #define AES_keyExpSize 208u
    #define AES_Nk 6u
    #define AES_Nr 12u
#else
    #define AES_KEYLEN 16u   // Key length in bytes
    #define AES_keyExpSize 176u
    #define AES_Nk 4u        // The number of 32 bit words in a key.
    #define AES_Nr 10u       // The number of rounds in AES Cipher.
#endif


typedef uint8_t aes_state_t[4u][4u];                                            // state - array holding the intermediate results during decryption.

#define AES_Nb 4                                                                // The number of columns comprising a state in AES. This is a constant in AES. Value=4

typedef struct
{
  uint8_t RoundKey[AES_keyExpSize];
  uint8_t Iv[AES_BLOCKLEN];
} AES_ctx_t;

#define MAX_CIPHER_CONTEXT_SIZE 80u                                             /* depends on the algryhtm chosen */
typedef struct
{
   const CipherAlgo_t *cipherAlgo;
   uint8_t cipherContext1[MAX_CIPHER_CONTEXT_SIZE];
   uint8_t cipherContext2[MAX_CIPHER_CONTEXT_SIZE];
} XtsContext_t;

/**
 * @brief CoAP option parameters
 **/
typedef struct
{
   uint16_t number;                                                             ///<Option number
   uint8_t critical;                                                            ///<Critical property
   uint8_t unsafe;                                                              ///<Unsafe property
   uint8_t noCacheKey;                                                          ///<NoCacheKey property
   uint8_t repeatable;                                                          ///<Repeatable option
   const char *name;                                                            ///<Option name
   CoapOptionFormat_e format;                                                   ///<Option format
   uint16_t minLength;                                                          ///<Minimum acceptable length
   uint16_t maxLength;                                                          ///<Maximum acceptable length
} CoapOptionParameters_t;

/**
 * @brief CoAP version numbers
 **/
typedef enum {
   COAP_VERSION_1 = 1                                                           ///<CoAP version 1
} CoapProtocolLevel_e;

/**
 * @brief CoAP transport protocols
 **/
typedef enum {
   COAP_TRANSPORT_PROTOCOL_UDP  = 1,                                            ///<UDP protocol
   COAP_TRANSPORT_PROTOCOL_DTLS = 2                                             ///<DTLS protocol
} CoapTransportProtocol_e;

/**
 * @brief CoAP message types
 **/
typedef enum
{
   COAP_TYPE_CON = 0,                                                           ///<Confirmable message
   COAP_TYPE_NON = 1,                                                           ///<Non-confirmable message
   COAP_TYPE_ACK = 2,                                                           ///<Acknowledgment message
   COAP_TYPE_RST = 3                                                            ///<Reset message
} CoapMessageType_e;

/**
 * @brief CoAP code classes
 **/
typedef enum
{
   COAP_CODE_CLASS_SUCCESS      = 2,
   COAP_CODE_CLASS_CLIENT_ERROR = 4,
   COAP_CODE_CLASS_SERVER_ERROR = 5
} CoapCodeClass_e;

/**
 * @brief CoAP method and response codes
 **/
typedef enum
{
   COAP_CODE_EMPTY                        = COAP_CODE(0, 0),
   COAP_CODE_GET                          = COAP_CODE(0, 1),
   COAP_CODE_POST                         = COAP_CODE(0, 2),
   COAP_CODE_PUT                          = COAP_CODE(0, 3),
   COAP_CODE_DELETE                       = COAP_CODE(0, 4),
   COAP_CODE_FETCH                        = COAP_CODE(0, 5),
   COAP_CODE_PATCH                        = COAP_CODE(0, 6),
   COAP_CODE_IPATCH                       = COAP_CODE(0, 7),
   COAP_CODE_CREATED                      = COAP_CODE(2, 1),
   COAP_CODE_DELETED                      = COAP_CODE(2, 2),
   COAP_CODE_VALID                        = COAP_CODE(2, 3),
   COAP_CODE_CHANGED                      = COAP_CODE(2, 4),
   COAP_CODE_CONTENT                      = COAP_CODE(2, 5),
   COAP_CODE_CONTINUE                     = COAP_CODE(2, 31),
   COAP_CODE_BAD_REQUEST                  = COAP_CODE(4, 0),
   COAP_CODE_UNAUTHOZED                   = COAP_CODE(4, 1),
   COAP_CODE_BAD_OPTION                   = COAP_CODE(4, 2),
   COAP_CODE_FORBIDDEN                    = COAP_CODE(4, 3),
   COAP_CODE_NOT_FOUND                    = COAP_CODE(4, 4),
   COAP_CODE_METHOD_NOT_ALLOWED           = COAP_CODE(4, 5),
   COAP_CODE_NOT_ACCEPTABLE               = COAP_CODE(4, 6),
   COAP_CODE_REQUEST_ENTITY_INCOMPLETE    = COAP_CODE(4, 8),
   COAP_CODE_CONFLICT                     = COAP_CODE(4, 9),
   COAP_CODE_PRECONDITION_FAILED          = COAP_CODE(4, 12),
   COAP_CODE_REQUEST_ENTITY_TO_LARGE      = COAP_CODE(4, 13),
   COAP_CODE_UNSUPPORTED_CONTENT_FORMAT   = COAP_CODE(4, 15),
   COAP_CODE_UNPROCESSABLE_ENTITY         = COAP_CODE(4, 22),
   COAP_CODE_INTERNAL_SERVER              = COAP_CODE(5, 0),
   COAP_CODE_NOT_IMPLEMENTED              = COAP_CODE(5, 1),
   COAP_CODE_BAD_GATEWAY                  = COAP_CODE(5, 2),
   COAP_CODE_SERVICE_UNAVAILABLE          = COAP_CODE(5, 3),
   COAP_CODE_GATEWAY_TIMEOUT              = COAP_CODE(5, 4),
   COAP_CODE_PROXYING_NOT_SUPPORTED       = COAP_CODE(5, 5),
   COAP_CODE_CSM                          = COAP_CODE(7, 1),
   COAP_CODE_PING                         = COAP_CODE(7, 2),
   COAP_CODE_PONG                         = COAP_CODE(7, 3),
   COAP_CODE_RELEASE                      = COAP_CODE(7, 4),
   COAP_CODE_ABORT                        = COAP_CODE(7, 5)
} CoapCode_e;

#ifndef COAP_MAX_MSG_SIZE
#define COAP_MAX_MSG_SIZE 1024u
#endif

typedef struct {
#ifdef _CPU_BIG_ENDIAN
   uint8_t delta : 4u;                                                          //0
   uint8_t length : 4u;
#else
   uint8_t length : 4u;                                                         //0
   uint8_t delta : 4u;
#endif
} CoapOptionHeader_t;                                                           // alternate byte swapped option field type check if 32 bit or 8bit or may vary ?

#ifdef _CPU_BIG_ENDIAN
//CBORPACKED (
typedef struct{
  uint8_t version : 2u;                                                         // Version (Ver): CoAP version, currently 01.
  uint8_t type : 2u;                                                            // Type (T): Keeps track of whether a message is con?rmable, non-con?rmable or a con?rmation
  uint8_t tkl : 4u;                                                             // Token Length (TKL): Indicates Length of token ?eld, 0-8 bytes (4bit bytes)
  uint8_t codeReq;                                                              // Code: Request/Responsecode. Analogue to HTTP codes,e.g. GETorPOST
  uint16_t messageId;                                                           // Message ID: Used for duplication detection and acknowledgement handling.
  uint8_t token[];                                                              // Token: Used for Request-Response mapping. (shows 32bits long)
  uint32_t options;                                                             // Options: An ordered list of options related to the message. For example options specifying how a proxy should handle messages and what resource that is requested.
  uint8_t flags;                                                                // 0xFF 0b11111111
  uint8_t payload[COAP_MAX_MSG_SIZE];                                           // Payload: The payload of the message
 } COAP_Message_t;                                                              // COAP message type
#else
typedef struct{
  uint8_t tkl : 4u;                                                             // Token Length (TKL): Indicates Length of token ?eld, 0-8 bytes (4bit bytes)
  uint8_t type : 2u;                                                            // Type (T): Keeps track of whether a message is con?rmable, non-con?rmable or a con?rmation
  uint8_t version : 2u;                                                         // Version (Ver): CoAP version, currently 01.
  uint8_t codeReq;                                                              // Code: Request/Responsecode. Analogue to HTTP codes,e.g. GETorPOST
  uint16_t messageId;                                                           // Message ID: Used for duplication detection and acknowledgement handling.
  uint8_t token[];                                                              // Token: Used for Request-Response mapping. (shows 32bits long)
  uint32_t options;                                                             // Options: An ordered list of options related to the message. For example options specifying how a proxy should handle messages and what resource that is requested.
  uint8_t flags;                                                                // 0xFF 0b11111111
  uint8_t payload[COAP_MAX_MSG_SIZE];                                           // Payload: The payload of the message
 } COAP_Message_t;                                                              // COAP message type
#endif

#define OSCoAP_Cipertext "[Content-Format=TEXT/PLAIN,0xFF,Payload=\"hello\"]"
#define OSCoAP_CiperEncoded 0x2cd41d5c67003455351a120e5d1069

#define CBOR_OSCoAP_ENAB                                                        // if you want to enable OSCoAP rather than plain COAP
#if defined(CBOR_OSCoAP_ENAB)
//CBORPACKED (
typedef struct{
 unsigned char cipertextStart[64u];                                             // Cipertext= [Content-Format=TEXT/PLAIN,0xFF,Payload="hello"]
 uint8_t Cipertext_enc[32u];
// }) OSCoAP_Cipertext_Obj_t;                                                     // cipertext object
} OSCoAP_Cipertext_Obj_t;                                                       // cipertext object

//CBORPACKED (
typedef struct{
  uint32_t coseTag;
  uint8_t coseArray;
  uint8_t bytes1;
  uint8_t map;
  uint8_t keySeqNo;
  uint8_t bytes2;
  uint8_t value;
  uint8_t map2;
  uint8_t Cipertext_enc[32u];
// }) OSCoAP_COSE_Obj_t;                                                          // COSE object used in OSCoAP preparation
 } OSCoAP_COSE_Obj_t;                                                           // COSE object used in OSCoAP preparation
#endif

/**
 * @brief Poly1305 context
 **/
typedef struct
{
   uint32_t r[4u];
   uint32_t s[4u];
   uint64_t a[8u];
   uint8_t buffer[17u];
   size_t size;
} Poly1305Context_t;

#if defined(ALGO_CHA_CHA)
#define CHACHA_QUARTER_ROUND(a, b, c, d) \
{ /* ChaCha quarter-round function */ \
   a += b; \
   d ^= a; \
   d = ROL32(d, 16); \
   c += d; \
   b ^= c; \
   b = ROL32(b, 12); \
   a += b; \
   d ^= a; \
   d = ROL32(d, 8); \
   c += d; \
   b ^= c; \
   b = ROL32(b, 7); \
}
/**
 * @brief ChaCha algorithm context
 **/
typedef struct
{
   uint8_t nr;
   uint32_t state[16u];
   uint32_t block[16u];
   size_t pos;
} ChachaContext_t;
#elif defined(ALGO_SEED)
#define SEED_BLOCK_SIZE 16u                                                     //SEED block size
#define SEED_CIPHER_ALGO (&seedCipherAlgo)                                      //Common interface for encryption algorithms
#define MAX_CIPHER_BLOCK_SIZE SEED_BLOCK_SIZE
/**
 * @brief SEED algorithm context
 **/
typedef struct
{
   uint32_t ks[32u];
} SeedContext_t;
extern const CipherAlgo_t seedCipherAlgo;
/* Function G */
#define G(x) (ss00[(x) & 0xFF] ^ ss1[((x) >> 8) & 0xFF] ^ ss2[((x) >> 16) & 0xFF] ^ ss3[((x) >> 24) & 0xFF])
/* Round function F */
#define F(k0, k1, r0, r1, t0, t1) \
{ \
   t1 = (r0 ^ k0) ^ (r1 ^ k1); \
   t1 = G(t1); \
   t0 = t1 + (r0 ^ k0); \
   t0 = G(t0); \
   t1 += t0; \
   t1 = G(t1); \
   t0 += t1; \
}
#elif defined(ALGO_CAMELLIA)
#define CAMELLIA_BLOCK_SIZE 16u                                                 //Camellia block size
#define CAMELLIA_CIPHER_ALGO (&camelliaCipherAlgo)                              //Common interface for encryption algorithms
#define MAX_CIPHER_BLOCK_SIZE CAMELLIA_BLOCK_SIZE
/**
 * @brief Structure describing subkey generation
 **/
typedef struct
{
   uint8_t index;
   uint8_t key;
   uint8_t shift;
   uint8_t position;
} CamelliaSubkey_t;
/**
 * @brief Camellia algorithm context
 **/
typedef struct
{
   uint8_t nr;
   uint32_t k[16];
   uint32_t ks[68];
} CamelliaContext_t;
extern const CipherAlgo_t camelliaCipherAlgo;                                   //Camellia related constants
//Camellia round function
#define CAMELLIA_ROUND(left1, left2, right1, right2, k1, k2) \
{ \
   temp1 = left1 ^ k1; \
   temp2 = left2 ^ k2; \
   CAMELLIA_S(temp1, temp2); \
   CAMELLIA_P(temp1, temp2); \
   temp1 ^= right2; \
   temp2 ^= right1; \
   right1 = left1; \
   right2 = left2; \
   left1 = temp2; \
   left2 = temp1; \
}
//F-function
#define CAMELLIA_F(xl, xr, kl, kr) \
{ \
   xl = xl ^ kl; \
   xl = xr ^ kr; \
   CAMELLIA_S(xl, xr); \
   CAMELLIA_P(xl, xr); \
}
//FL-function
#define CAMELLIA_FL(xl, xr, kl, kr) \
{ \
   temp1 = (xl & kl); \
   xr ^= ROL32(temp1, 1); \
   xl ^= (xr | kr); \
}
//Inverse FL-function
#define CAMELLIA_INV_FL(yl, yr, kl, kr) \
{ \
   yl ^= (yr | kr); \
   temp1 = (yl & kl); \
   yr ^= ROL32(temp1, 1); \
}
//S-function
#define CAMELLIA_S(zl, zr) \
{ \
   zl = (sbox1[(zl >> 24) & 0xFF] << 24) | (sbox2[(zl >> 16) & 0xFF] << 16) | \
      (sbox3[(zl >> 8) & 0xFF] << 8) | sbox4[zl & 0xFF]; \
   zr = (sbox2[(zr >> 24) & 0xFF] << 24) | (sbox3[(zr >> 16) & 0xFF] << 16) | \
      (sbox4[(zr >> 8) & 0xFF] << 8) | sbox1[zr & 0xFF]; \
}
//P-function
#define CAMELLIA_P(zl, zr) \
{ \
   zl ^= ROL32(zr, 8); \
   zr ^= ROL32(zl, 16); \
   zl ^= ROR32(zr, 8); \
   zr ^= ROR32(zl, 8); \
}
//Key schedule related constants
#define KL 0
#define KR 4
#define KA 8
#define KB 12
#define L  0
#define R  64
#elif defined(ADV_ENCPT_STD_USED)                                               // -------------------------- AES Advanced Encryption Standard ----------------
#define AES_BLOCK_SIZE 16u                                                      //AES block size
#define AES_CIPHER_ALGO (&aesCipherAlgo)                                        //Common interface for encryption algorithms
#define MAX_CIPHER_BLOCK_SIZE AES_BLOCK_SIZE
/**
 * @brief AES algorithm context
 **/
typedef struct
{
   uint8_t nr;
   uint32_t ek[60u];
   uint32_t dk[60u];
} AesContext_t;
extern const CipherAlgo_t aesCipherAlgo;                                        //AES related constants
#elif defined(ALGO_ARIA)
#define ARIA_BLOCK_SIZE 16u                                                     //ARIA block size
#define ARIA_CIPHER_ALGO (&ariaCipherAlgo)                                      //Common interface for encryption algorithms
#define MAX_CIPHER_BLOCK_SIZE ARIA_BLOCK_SIZE

//Move operation
#define MOV128(b, a) \
{ \
   (b)[0] = (a)[0]; \
   (b)[1] = (a)[1]; \
   (b)[2] = (a)[2]; \
   (b)[3] = (a)[3]; \
}

//XOR operation
#define XOR128(b, a) \
{ \
   (b)[0] ^= (a)[0]; \
   (b)[1] ^= (a)[1]; \
   (b)[2] ^= (a)[2]; \
   (b)[3] ^= (a)[3]; \
}

//Rotate left operation
#define ROL128(b, a, n) \
{ \
   (b)[0] = ((a)[((n) / 32 + 0) % 4u] << ((n) % 32u)) | ((a)[((n) / 32u + 1u) % 4u] >> (32u - ((n) % 32u))); \
   (b)[1] = ((a)[((n) / 32 + 1) % 4u] << ((n) % 32u)) | ((a)[((n) / 32u + 2u) % 4u] >> (32u - ((n) % 32u))); \
   (b)[2] = ((a)[((n) / 32 + 2) % 4u] << ((n) % 32u)) | ((a)[((n) / 32u + 3u) % 4u] >> (32u - ((n) % 32u))); \
   (b)[3] = ((a)[((n) / 32 + 3) % 4u] << ((n) % 32u)) | ((a)[((n) / 32u + 0u) % 4u] >> (32u - ((n) % 32u))); \
}

//Substitution layer SL1
#define SL1(b, a) \
{ \
   uint8_t *x = (uint8_t *) (a); \
   uint8_t *y = (uint8_t *) (b); \
   y[0] = sb1[x[0]]; \
   y[1] = sb2[x[1]]; \
   y[2] = sb3[x[2]]; \
   y[3] = sb4[x[3]]; \
   y[4] = sb1[x[4]]; \
   y[5] = sb2[x[5]]; \
   y[6] = sb3[x[6]]; \
   y[7] = sb4[x[7]]; \
   y[8] = sb1[x[8]]; \
   y[9] = sb2[x[9]]; \
   y[10] = sb3[x[10]]; \
   y[11] = sb4[x[11]]; \
   y[12] = sb1[x[12]]; \
   y[13] = sb2[x[13]]; \
   y[14] = sb3[x[14]]; \
   y[15] = sb4[x[15]]; \
}

//Substitution layer SL2
#define SL2(b, a) \
{ \
   uint8_t *x = (uint8_t *) (a); \
   uint8_t *y = (uint8_t *) (b); \
   y[0] = sb3[x[0]]; \
   y[1] = sb4[x[1]]; \
   y[2] = sb1[x[2]]; \
   y[3] = sb2[x[3]]; \
   y[4] = sb3[x[4]]; \
   y[5] = sb4[x[5]]; \
   y[6] = sb1[x[6]]; \
   y[7] = sb2[x[7]]; \
   y[8] = sb3[x[8]]; \
   y[9] = sb4[x[9]]; \
   y[10] = sb1[x[10]]; \
   y[11] = sb2[x[11]]; \
   y[12] = sb3[x[12]]; \
   y[13] = sb4[x[13]]; \
   y[14] = sb1[x[14]]; \
   y[15] = sb2[x[15]]; \
}

//Diffusion layer
#define A(b, a) \
{ \
   uint8_t *x = (uint8_t *) (a); \
   uint8_t *y = (uint8_t *) (b); \
   y[0] = x[3] ^ x[4] ^ x[6] ^ x[8] ^ x[9] ^ x[13] ^ x[14]; \
   y[1] = x[2] ^ x[5] ^ x[7] ^ x[8] ^ x[9] ^ x[12] ^ x[15]; \
   y[2] = x[1] ^ x[4] ^ x[6] ^ x[10] ^ x[11] ^ x[12] ^ x[15]; \
   y[3] = x[0] ^ x[5] ^ x[7] ^ x[10] ^ x[11] ^ x[13] ^ x[14]; \
   y[4] = x[0] ^ x[2] ^ x[5] ^ x[8] ^ x[11] ^ x[14] ^ x[15]; \
   y[5] = x[1] ^ x[3] ^ x[4] ^ x[9] ^ x[10] ^ x[14] ^ x[15]; \
   y[6] = x[0] ^ x[2] ^ x[7] ^ x[9] ^ x[10] ^ x[12] ^ x[13]; \
   y[7] = x[1] ^ x[3] ^ x[6] ^ x[8] ^ x[11] ^ x[12] ^ x[13]; \
   y[8] = x[0] ^ x[1] ^ x[4] ^ x[7] ^ x[10] ^ x[13] ^ x[15]; \
   y[9] = x[0] ^ x[1] ^ x[5] ^ x[6] ^ x[11] ^ x[12] ^ x[14]; \
   y[10] = x[2] ^ x[3] ^ x[5] ^ x[6] ^ x[8] ^ x[13] ^ x[15]; \
   y[11] = x[2] ^ x[3] ^ x[4] ^ x[7] ^ x[9] ^ x[12] ^ x[14]; \
   y[12] = x[1] ^ x[2] ^ x[6] ^ x[7] ^ x[9] ^ x[11] ^ x[12]; \
   y[13] = x[0] ^ x[3] ^ x[6] ^ x[7] ^ x[8] ^ x[10] ^ x[13]; \
   y[14] = x[0] ^ x[3] ^ x[4] ^ x[5] ^ x[9] ^ x[11] ^ x[14]; \
   y[15] = x[1] ^ x[2] ^ x[4] ^ x[5] ^ x[8] ^ x[10] ^ x[15]; \
}
/**
 * @brief ARIA algorithm context
 **/
typedef struct
{
   uint8_t nr;
   uint32_t k[16u];
   uint32_t ek[68u];
   uint32_t dk[68u];
} AriaContext_t;
extern const CipherAlgo_t ariaCipherAlgo;                                       //ARIA related constants
#elif defined(ALGO_SALSA20)
//Salsa20 quarter-round function
#define SALSA20_QUARTER_ROUND(a, b, c, d) \
{ \
   b ^= ROL32(a + d, 7); \
   c ^= ROL32(b + a, 9); \
   d ^= ROL32(c + b, 13); \
   a ^= ROL32(d + c, 18); \
}
#elif defined(ALGO_DES)
#define DES_BLOCK_SIZE 8u                                                       //DES block size
#define DES_CIPHER_ALGO (&desCipherAlgo)                                        //Common interface for encryption algorithms
#define MAX_CIPHER_BLOCK_SIZE DES_BLOCK_SIZE

//Rotate left operation
#define ROL28(a, n) ((((a) << (n)) | ((a) >> (28 - (n)))) & 0x0FFFFFFF)

//Initial permutation
#define DES_IP(left, right) \
{ \
   temp = ((left >> 4) ^ right) & 0x0F0F0F0F; \
   right ^= temp; \
   left ^= temp << 4; \
   temp = ((left >> 16) ^ right) & 0x0000FFFF; \
   right ^= temp; \
   left ^= temp << 16; \
   temp = ((right >> 2) ^ left) & 0x33333333; \
   left ^= temp; \
   right ^= temp << 2; \
   temp = ((right >> 8) ^ left) & 0x00FF00FF; \
   left ^= temp; \
   right ^= temp << 8; \
   temp = ((left >> 1) ^ right) & 0x55555555; \
   right ^= temp; \
   left ^= temp << 1; \
   left = ROL32(left, 1); \
   right = ROL32(right, 1); \
}

//Final permutation
#define DES_FP(left, right) \
{ \
   left = ROR32(left, 1); \
   right = ROR32(right, 1); \
   temp = ((left >> 1) ^ right) & 0x55555555; \
   right ^= temp; \
   left ^= temp << 1; \
   temp = ((right >> 8) ^ left) & 0x00FF00FF; \
   left ^= temp; \
   right ^= temp << 8; \
   temp = ((right >> 2) ^ left) & 0x33333333; \
   left ^= temp; \
   right ^= temp << 2; \
   temp = ((left >> 16) ^ right) & 0x0000FFFF; \
   right ^= temp; \
   left ^= temp << 16; \
   temp = ((left >> 4) ^ right) & 0x0F0F0F0F; \
   right ^= temp; \
   left ^= temp << 4; \
}

//DES round
#define DES_ROUND(left, right, ks) \
{ \
   temp = right ^ *(ks); \
   left ^= sp2[(temp >> 24) & 0x3F]; \
   left ^= sp4[(temp >> 16) & 0x3F]; \
   left ^= sp6[(temp >> 8) & 0x3F]; \
   left ^= sp8[temp & 0x3F]; \
   temp = ROR32(right, 4) ^ *(ks + 1); \
   left ^= sp1[(temp >> 24) & 0x3F]; \
   left ^= sp3[(temp >> 16) & 0x3F]; \
   left ^= sp5[(temp >> 8) & 0x3F]; \
   left ^= sp7[temp & 0x3F]; \
   temp = right; \
   right = left; \
   left = temp; \
}

//Permuted choice 1
#define DES_PC1(left, right) \
{ \
   uint32_t temp; \
   temp = ((left >> 4) ^ right) & 0x0F0F0F0F; \
   right ^= temp; \
   left ^= (temp << 4); \
   temp = ((right >> 16) ^ left) & 0x0000FFFF; \
   left ^= temp; \
   right ^= (temp << 16); \
   temp = ((left >> 2) ^ right) & 0x33333333; \
   right ^= temp; \
   left ^= (temp << 2); \
   temp = ((right >> 16) ^ left) & 0x0000FFFF; \
   left ^= temp; \
   right ^= (temp << 16); \
   temp = ((left >> 1) ^ right) & 0x55555555; \
   right ^= temp; \
   left ^= (temp << 1); \
   temp = ((right >> 8) ^ left) & 0x00FF00FF; \
   left ^= temp; \
   right ^= (temp << 8); \
   temp = ((left >> 1) ^ right) & 0x55555555; \
   right ^= temp; \
   left ^= (temp << 1); \
   temp = (left << 8) | ((right >> 20) & 0x000000F0); \
   left = ((right << 20) & 0x0FF00000); \
   left |= ((right << 4) & 0x000FF000); \
   left |= ((right >> 12) & 0x00000FF0); \
   left |= ((right >> 28) & 0x0000000F); \
   right = temp >> 4; \
}

extern const CipherAlgo_t desCipherAlgo;                                        //DES related constants
#endif /* end of cipher macro choice */

#ifdef MAX_CIPHER_BLOCK_SIZE                                                    // the cipher chosen can be used with CMAC
/**
 * @brief CMAC algorithm context
 **/
typedef struct
{
   const CipherAlgo_t *cipher;
   uint8_t cipherContext[MAX_CIPHER_CONTEXT_SIZE];
   uint8_t k1[MAX_CIPHER_BLOCK_SIZE];
   uint8_t k2[MAX_CIPHER_BLOCK_SIZE];
   uint8_t buffer[MAX_CIPHER_BLOCK_SIZE];
   size_t bufferLength;
   uint8_t mac[MAX_CIPHER_BLOCK_SIZE];
} CmacContext_t;
#endif

typedef uint8_t (*HashAlgoCompute)(const void *dataV, size_t length, uint8_t *digest); //Common API for hash algorithms
typedef void (*HashAlgoInit)(void *context);
typedef void (*HashAlgoUpdate)(void *context, const void *dataV, size_t length);
typedef void (*HashAlgoFinal)(void *context, uint8_t *digest);
typedef void (*HashAlgoFinalRaw)(void *context, uint8_t *digest);
/**
 * @brief Common interface for hash algorithms
 **/
typedef struct
{
   const char *name;
   const uint8_t *oid;
   size_t oidSize;
   size_t contextSize;
   size_t blockSize;
   size_t digestSize;
   size_t minPadSize;
   uint8_t bigEndian : 1u;
   uint8_t spare : 7u;
   HashAlgoCompute compute;
   HashAlgoInit init;
   HashAlgoUpdate update;
   HashAlgoFinal final;
   HashAlgoFinalRaw finalRaw;
} HashAlgo_t;
extern const uint8_t sha256Oid[9];                                              //SHA-256 related constants
extern const HashAlgo_t sha256HashAlgo;
#define SHA256_HASH_ALGO (&sha256HashAlgo)
#if defined(IOT_SHA_256)
#define SHA256_BLOCK_SIZE 64                                                    //SHA-256 block size
#define SHA256_DIGEST_SIZE 32                                                   //SHA-256 digest size
#define SHA256_MIN_PAD_SIZE 9                                                   //Minimum length of the padding string
#define SHA256_OID sha256Oid                                                    //SHA-256 algorithm object identifier
#define SHA256_HASH_ALGO (&sha256HashAlgo)                                      //Common interface for hash algorithms
#define W(t) w[(t) & 0x0F]                                                      //Macro to access the workspace as a circular buffer
#define CH(x, y, z) (((x) & (y)) | (~(x) & (z)))                                //SHA-256 auxiliary functions
#define MAJ(x, y, z) (((x) & (y)) | ((x) & (z)) | ((y) & (z)))
#define SIGMA1(x) (ROR32(x, 2) ^ ROR32(x, 13) ^ ROR32(x, 22))
#define SIGMA2(x) (ROR32(x, 6) ^ ROR32(x, 11) ^ ROR32(x, 25))
#define SIGMA3(x) (ROR32(x, 7) ^ ROR32(x, 18) ^ (x >> 3))
#define SIGMA4(x) (ROR32(x, 17) ^ ROR32(x, 19) ^ (x >> 10))
/**
 * @brief SHA-256 algorithm context
 **/
typedef struct
{
   union
   {
      uint32_t h[8u];
      uint8_t digest[32u];
   };
   union
   {
      uint32_t w[16u];
      uint8_t buffer[64u];
   };
   size_t size;
   uint64_t totalSize;
} Sha256Context_t;
extern const uint8_t sha256Oid[9];                                              //SHA-256 related constants
extern const HashAlgo_t sha256HashAlgo;
#endif    /* end sha256 */
#if defined(ADV_ENCPT_STD_USED)                                                 /* to use yarrow we need aes */
#define YARROW_PRNG_ALGO (&yarrowPrngAlgo)                                      //Common interface for PRNG algorithms
#define YARROW_FAST_POOL_ID 0u                                                  //Pool identifiers
#define YARROW_SLOW_POOL_ID 1u
#define YARROW_N 3u                                                             //Yarrow PRNG parameters
#define YARROW_K 2u
#define YARROW_PG 10u
#define YARROW_FAST_THRESHOLD 100u
#define YARROW_SLOW_THRESHOLD 160u
/**
 * @brief CoAP option formats
 **/
typedef enum
{
   PRNG_COMPLETE  = 0u,                                                         ///resource is complete and free to be taken
   PRNG_START = 1u,                                                             ///resource has been booked to this task and has started
   PRNG_QUE   = 2u,                                                             ///booking is in the queue to get this resource
} YarrowPrngMutex_e;
/**
 * @brief Yarrow PRNG context
 **/
typedef struct
{
//   OsMutex mutex;                    //Mutex to prevent simultaneous access to the PRNG state
   uint8_t ready : 1u;                                                          //This flag tells whether the PRNG has been properly seeded
   uint8_t mutex : 7u;                                                          // mutex state variable
   uint16_t currentPool[YARROW_N];                                              //Current pool identifier
   Sha256Context_t fastPool;                                                    //Fast pool
   size_t fastPoolEntropy[YARROW_N];                                            //Entropy estimation (fast pool)
   Sha256Context_t slowPool;                                                    //Slow pool
   size_t slowPoolEntropy[YARROW_N];                                            //Entropy estimation (slow pool)
   AesContext_t cipherContext;                                                  //Cipher context
   uint8_t key[32];                                                             //Current key
   uint8_t counter[16];                                                         //Counter block
   size_t blockCount;                                                           //Number of blocks that have been generated
} YarrowContext_t;

//Common API for pseudo-random number generators
typedef uint8_t (*PrngAlgoInit)(void *context);
typedef void (*PrngAlgoRelease)(void *context);
typedef uint8_t (*PrngAlgoSeed)(void *context, const uint8_t *input, size_t length);
typedef uint8_t (*PrngAlgoAddEntropy)(void *context, uint16_t source, const uint8_t *input, size_t length, size_t entropy);
typedef uint8_t (*PrngAlgoRead)(void *context, uint8_t *output, size_t length);
/**
 * @brief Common interface for pseudo-random number generators
 **/
typedef struct
{
   const char *name;
   size_t contextSize;
   PrngAlgoInit init;
   PrngAlgoRelease release;
   PrngAlgoSeed seed;
   PrngAlgoAddEntropy addEntropy;
   PrngAlgoRead read;
} PrngAlgo_t;
extern const PrngAlgo_t yarrowPrngAlgo;                                         //Yarrow related constants

#endif /* _____end AES Yarrow______ */


// low-level, pass raw CBOR bytes via start/stop pointers, returns end pointer (== stop if complete)
// can skip a given number of items and then will fill optional result w/ the current item
extern uint8_t *cb0r(uint8_t *start, uint8_t *stop, uint32_t skip, cb0r_t result);

// safer high-level wrapper to read raw CBOR
extern bool cb0r_read(uint8_t *in, uint32_t len, cb0r_t result);

// fetch an item from the given array (or map), zero-index
extern bool cb0r_get(cb0r_t array, uint32_t index, cb0r_t result);

// find and fetch the value of a given key from the map, number/bytes args only used for some types
extern bool cb0r_find(cb0r_t map, cb0r_e type, uint64_t number, uint8_t *bytes, cb0r_t result);

// convenience method to write a header given a type and optional number (length/count/value), returns bytes written to out (max 9)
extern uint8_t cb0r_write(uint8_t *out, cb0r_e type, uint64_t number);

// simple wrapper to return a contained value start/length
extern uint8_t *cb0r_value(cb0r_t data);
extern uint32_t cb0r_vlen(cb0r_t data);

/* support these cipher modes */
extern uint8_t cbcEncrypt(const CipherAlgo_t *cipher, void *context, uint8_t *ivXOR, const uint8_t *p, uint8_t *c, size_t length );
extern uint8_t cbcDecrypt(const CipherAlgo_t *cipher, void *context, uint8_t *ivXOR, const uint8_t *c, uint8_t *p, size_t length);
extern uint8_t ecbEncrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *p, uint8_t *c, size_t length);
extern uint8_t ecbDecrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *c, uint8_t *p, size_t length);
extern uint8_t ofbEncrypt(const CipherAlgo_t *cipher, void *context, uint8_t s,  uint8_t *ivXOR, const uint8_t *p, uint8_t *c, size_t length);
extern uint8_t ofbDecrypt(const CipherAlgo_t *cipher, void *context, uint8_t s, uint8_t *ivXOR, const uint8_t *c, uint8_t *p, size_t length);
extern uint8_t xtsInit(XtsContext_t *context, const CipherAlgo_t *cipherAlgo, const void *key, size_t keyLen);
extern uint8_t xtsEncrypt(XtsContext_t *context, const uint8_t *i, const uint8_t *p, uint8_t *c, size_t length);
extern uint8_t xtsDecrypt(XtsContext_t *context, const uint8_t *i, const uint8_t *c, uint8_t *p, size_t length);
/* CCM is a block cipher mode of encryption which does both con?dentiality and integrity protection it is ued with AES cipher in OSCoAP */
extern void ccmIncCounter(uint8_t *x, size_t n);
extern void ccmXorBlock(uint8_t *x, const uint8_t *a, const uint8_t *b, size_t n);
extern uint8_t ccmDecrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen, const uint8_t *c, uint8_t *p, size_t length, const uint8_t *t, size_t tLen);
extern uint8_t ccmEncrypt(const CipherAlgo_t *cipher, void *context, const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen, const uint8_t *p, uint8_t *c, size_t length, uint8_t *t, size_t tLen);

#if defined(ALGO_CHA_CHA)
extern uint8_t chachaInit(ChachaContext_t *context, uint8_t nr, const uint8_t *key, size_t keyLen, const uint8_t *nonce, size_t nonceLen);
extern void chachaProcessBlock(ChachaContext_t *context);
extern void chachaCipher(ChachaContext_t *context, const uint8_t *input, uint8_t *output, size_t length);
#elif defined(ALGO_SEED)
extern uint8_t seedInit(SeedContext_t *context, const uint8_t *key, size_t keyLen);
extern void seedEncryptBlock(SeedContext_t *context, const uint8_t *input, uint8_t *output);
extern void seedDecryptBlock(SeedContext_t *context, const uint8_t *input, uint8_t *output);
#elif defined(ALGO_CAMELLIA)
extern uint8_t camelliaInit(CamelliaContext_t *context, const uint8_t *key, size_t keyLen);
extern void camelliaEncryptBlock(CamelliaContext_t *context, const uint8_t *input, uint8_t *output);
extern void camelliaDecryptBlock(CamelliaContext_t *context, const uint8_t *input, uint8_t *output);
#elif defined(ADV_ENCPT_STD_USED)
extern uint8_t aesInit(AesContext_t *context, const uint8_t *key, size_t keyLen);
extern void aesEncryptBlock(AesContext_t *context, const uint8_t *input, uint8_t *output);
extern void aesDecryptBlock(AesContext_t *context, const uint8_t *input, uint8_t *output);
#elif defined(ALGO_ARIA)
extern uint8_t ariaInit(AriaContext_t *context, const uint8_t *key, size_t keyLen);
extern void ariaEncryptBlock(AriaContext_t *context, const uint8_t *input, uint8_t *output);
extern void ariaDecryptBlock(AriaContext_t *context, const uint8_t *input, uint8_t *output);
#elif defined(ALGO_SALSA20)
extern void salsa20ProcessBlock(const uint8_t *input, uint8_t *output, uint8_t nr);
#elif defined(ALGO_DES)
extern uint8_t desInit(DesContext_t *context, const uint8_t *key, size_t keyLen);
extern void desEncryptBlock(DesContext_t *context, const uint8_t *input, uint8_t *output);
extern void desDecryptBlock(DesContext_t *context, const uint8_t *input, uint8_t *output);
#endif /* end algoryhtm types */
extern void poly1305Init(Poly1305Context_t *context, const uint8_t *key);
extern void poly1305Update(Poly1305Context_t *context, const void *dataV, size_t length);
extern void poly1305Final(Poly1305Context_t *context, uint8_t *tag);
extern void poly1305ProcessBlock(Poly1305Context_t *context);
#if defined(ALGO_CHA_CHA)
extern uint8_t chacha20Poly1305Decrypt(const uint8_t *k, size_t kLen, const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen,  const uint8_t *c, uint8_t *p, size_t length, const uint8_t *t, size_t tLen);
extern uint8_t chacha20Poly1305Encrypt(const uint8_t *k, size_t kLen,const uint8_t *n, size_t nLen, const uint8_t *a, size_t aLen, const uint8_t *p, uint8_t *c, size_t length, uint8_t *t, size_t tLen);
#endif /* end chacha 20 poly 1305 */
#ifdef MAX_CIPHER_BLOCK_SIZE                                                    /* we are using a ciper compatible with cmac */
extern void cmacReset(CmacContext_t *context);
extern void cmacMul(uint8_t *x, const uint8_t *a, size_t n, uint8_t rb);
extern uint8_t cmacInit(CmacContext_t *context, const CipherAlgo_t *cipher, const void *key, size_t keyLen);
extern void cmacXorBlock(uint8_t *x, const uint8_t *a, const uint8_t *b, size_t n);
extern void cmacUpdate(CmacContext_t *context, const void *dataV, size_t dataLen);
extern uint8_t cmacFinal(CmacContext_t *context, uint8_t *mac, size_t macLen);
#endif  /* end of cmac */
#if defined(IOT_SHA_256)
extern void sha256Init(Sha256Context_t *context);
extern void sha256ProcessBlock(Sha256Context_t *context);
extern void sha256FinalRaw(Sha256Context_t *context, uint8_t *digest);
extern void sha256Update(Sha256Context_t *context, const void *dataV, size_t length);
extern void sha256Final(Sha256Context_t *context, uint8_t *digest);
#endif /* end SHA256 hash selection */
#if defined(ADV_ENCPT_STD_USED)                                                 /* needs ciper to be AES */
extern uint8_t yarrowInit(YarrowContext_t *context);
extern void yarrowRelease(YarrowContext_t *context);
extern uint8_t osCreateMutex(YarrowContext_t *context);
extern uint8_t osDeleteMutex(YarrowContext_t *context);
extern void yarrowFastReseed(YarrowContext_t *context);
extern void yarrowGenerateBlock(YarrowContext_t *context, uint8_t *output);
extern uint8_t yarrowSeed(YarrowContext_t *context, const uint8_t *input, size_t length);
extern uint8_t yarrowRead(YarrowContext_t *context, uint8_t *output, size_t length);
extern void yarrowSlowReseed(YarrowContext_t *context);
extern uint8_t yarrowAddEntropy(YarrowContext_t *context, uint8_t source,  const uint8_t *input, size_t length, size_t entropy);
extern uint8_t yarrowBegin(YarrowContext_t yarrowContext, uint8_t seed[32u]);
extern void AddRoundKey(uint8_t round, aes_state_t* state, const uint8_t* RoundKey);
extern  void SubBytes(aes_state_t* state);
extern  void ShiftRows(aes_state_t* state);
extern  uint8_t xtime(uint8_t x);
extern  void MixColumns(aes_state_t* state);
extern  void Cipher(aes_state_t* state, const uint8_t* RoundKey);
extern  void AES_CTR_xcrypt_buffer(AES_ctx_t* ctx, uint8_t* buf, uint32_t length);
extern  void XorWithIv(uint8_t* buf, const uint8_t* Iv);
extern  void AES_CBC_encrypt_buffer(AES_ctx_t *ctx, uint8_t* buf, uint32_t length);
extern  void InvMixColumns(aes_state_t* state);
extern  void InvShiftRows(aes_state_t* state);
extern  void InvSubBytes(aes_state_t* state);
extern  void InvCipher(aes_state_t* state, const uint8_t* RoundKey);
extern void AES_CBC_decrypt_buffer(AES_ctx_t* ctx, uint8_t* buf,  uint32_t length);
extern void AES_ECB_encrypt(const AES_ctx_t* ctx, uint8_t* buf);
extern void AES_ECB_decrypt(const AES_ctx_t* ctx, uint8_t* buf);
#endif /* end of yarrow prng */

#if defined(USE_SHA_512)
extern uint64_t SHA512_Ch(uint64_t x, uint64_t y, uint64_t z);
extern uint64_t SHA512_Maj(uint64_t x, uint64_t y, uint64_t z);
extern uint64_t SHA512_SigmaB0(uint64_t x);
extern uint64_t SHA512_SigmaB1(uint64_t x);
extern uint64_t SHA512_sigmaL0(uint64_t x);
extern uint64_t SHA512_sigmaL1(uint64_t x);
extern void SHA512_Round(uint64_t a, uint64_t b, uint64_t c, uint64_t* d, uint64_t e, uint64_t f, uint64_t g, uint64_t* h, uint64_t k, uint64_t w );
extern void SHA512_Initialize(uint64_t* s);
extern uint64_t SHA512_ReadBE64(const unsigned char* ptr);
extern void SHA512_Transform(uint64_t* s, const unsigned char* chunk);
extern void CSHA512_Write(unsigned char* dataV, unsigned char* buf, size_t len, size_t *bytes, uint64_t* s);
extern void SHA512_WriteBE64(unsigned char* ptr, uint64_t x);
extern void CSHA512_Finalize(unsigned char hash[SHA512_OUTPUT_SIZE], unsigned char* buf, size_t *bytes, uint64_t* s);
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif


#endif // cb0r_h