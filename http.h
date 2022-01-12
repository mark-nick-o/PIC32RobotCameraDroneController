//  Header file to support http client functions
//
//  Contains some sections
//  Copyright (C) 2010-2019 Oryx Embedded SARL. All rights reserved.
//  Copyright (c) 2011-2012 Yaroslav Stavnichiy <yarosla@gmail.com> httppress.c
//
//  Which is part of CycloneTCP Open.
//  * Compiled & Ported : Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#ifndef _HTTP_H
#define _HTTP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define HTTPPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define HTTPPACKED __attribute__((packed))                                     /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define HTTPPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define HTTPPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define HTTPPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define HTTP_PORT 80U                                                           //HTTP port number
#define HTTPS_PORT 443U                                                         //HTTPS port number (HTTP over TLS)
#define HTTP_LIB static                                                         // static or external

/**
 * @brief HTTP version numbers
 **/
typedef enum
{
   HTTP_VERSION_0_9 = 0x0009u,
   HTTP_VERSION_1_0 = 0x0100u,
   HTTP_VERSION_1_1 = 0x0101u
} HttpVersion;

/**
 * @brief HTTP authentication schemes
 **/
typedef enum
{
   HTTP_AUTH_MODE_NONE   = 0u,
   HTTP_AUTH_MODE_BASIC  = 1u,
   HTTP_AUTH_MODE_DIGEST = 2u
} HttpAuthMode;

/**
 * @brief Quality of protection (digest authentication)
 **/
typedef enum
{
   HTTP_AUTH_QOP_NONE     = 0u,
   HTTP_AUTH_QOP_AUTH     = 1u,
   HTTP_AUTH_QOP_AUTH_INT = 2u
} HttpAuthQop;

/**
 * @brief Flags used by I/O functions
 **/
typedef enum
{
   HTTP_FLAG_WAIT_ALL   = 0x0800u,
   HTTP_FLAG_BREAK_CHAR = 0x1000u,
   HTTP_FLAG_BREAK_CRLF = 0x100Au,
   HTTP_FLAG_NO_DELAY   = 0x4000u,
   HTTP_FLAG_DELAY      = 0x8000u
} HttpFlags;

/**
 * @brief HTTP request states
 */
typedef enum
{
   HTTP_REQ_STATE_INIT                = 0u,
   HTTP_REQ_STATE_FORMAT_HEADER       = 1u,
   HTTP_REQ_STATE_SEND_HEADER         = 2u,
   HTTP_REQ_STATE_SEND_BODY           = 3u,
   HTTP_REQ_STATE_SEND_CHUNK_SIZE     = 4u,
   HTTP_REQ_STATE_SEND_CHUNK_DATA     = 5u,
   HTTP_REQ_STATE_FORMAT_TRAILER      = 6u,
   HTTP_REQ_STATE_SEND_TRAILER        = 7u,
   HTTP_REQ_STATE_RECEIVE_STATUS_LINE = 8u,
   HTTP_REQ_STATE_RECEIVE_HEADER      = 9u,
   HTTP_REQ_STATE_PARSE_HEADER        = 10u,
   HTTP_REQ_STATE_RECEIVE_BODY        = 11u,
   HTTP_REQ_STATE_RECEIVE_CHUNK_SIZE  = 12u,
   HTTP_REQ_STATE_RECEIVE_CHUNK_DATA  = 13u,
   HTTP_REQ_STATE_RECEIVE_TRAILER     = 14u,
   HTTP_REQ_STATE_PARSE_TRAILER       = 15u,
   HTTP_REQ_STATE_COMPLETE            = 16u
} HttpRequestState;

typedef enum {
     C_CONNECTING, C_HANDSHAKING, C_WRITING, C_READING_HEADERS, C_READING_BODY, C_READ_COMPLETED
} nx_connection_state;

/**
 * @brief HTTP character sets
 */
typedef enum
{
   HTTP_CHARSET_OCTET    = 0x0001u,
   HTTP_CHARSET_CTL      = 0x0002u,
   HTTP_CHARSET_LWS      = 0x0004u,
   HTTP_CHARSET_ALPHA    = 0x0008u,
   HTTP_CHARSET_DIGIT    = 0x0010u,
   HTTP_CHARSET_HEX      = 0x0020u,
   HTTP_CHARSET_VCHAR    = 0x0040u,
   HTTP_CHARSET_TCHAR    = 0x0080u,
   HTTP_CHARSET_TEXT     = 0x0100u,
   HTTP_CHARSET_OBS_TEXT = 0x0200u
} HttpCharset;

/**
 * @brief HTTP client states
 */
typedef enum
{
   HTTP_CLIENT_STATE_DISCONNECTED  = 0u,
   HTTP_CLIENT_STATE_CONNECTING    = 1u,
   HTTP_CLIENT_STATE_CONNECTED     = 2u,
   HTTP_CLIENT_STATE_DISCONNECTING = 3u
} HttpClientState;

typedef enum {
         CDS_CR1=-2, 
         CDS_LF1=-1, 
         CDS_SIZE=0, 
         CDS_LF2, 
         CDS_DATA
} HttpChunkDecStateCode;

#define HTTP_CLIENT_MAX_USERNAME_LEN 32U
#define HTTP_CLIENT_MAX_PASSWORD_LEN 32U
#define HTTP_CLIENT_MAX_REALM_LEN 32U

#ifndef HTTP_CLIENT_CNONCE_SIZE
   #define HTTP_CLIENT_CNONCE_SIZE 16                                           //Cnonce size
#elif (HTTP_CLIENT_CNONCE_SIZE < 1)
   #error HTTP_CLIENT_CNONCE_SIZE parameter is not valid
#endif

#ifndef HTTP_CLIENT_MAX_NONCE_LEN                                               //Maximum length of the nonce
   #define HTTP_CLIENT_MAX_NONCE_LEN 64
#elif (HTTP_CLIENT_MAX_NONCE_LEN < 1)
   #error HTTP_CLIENT_MAX_NONCE_LEN parameter is not valid
#endif

#ifndef HTTP_CLIENT_MAX_OPAQUE_LEN                                              //Maximum length of the opaque parameter
   #define HTTP_CLIENT_MAX_OPAQUE_LEN 64
#elif (HTTP_CLIENT_MAX_OPAQUE_LEN < 1)
   #error HTTP_CLIENT_MAX_OPAQUE_LEN parameter is not valid
#endif

#if defined(D_FT900)
typedef struct HTTPPACKED {
   HttpAuthMode mode;                                                           ///<HTTP authentication mode
   unsigned char username[HTTP_CLIENT_MAX_USERNAME_LEN + 1u];                   ///<User name
   unsigned char password[HTTP_CLIENT_MAX_PASSWORD_LEN + 1u];                   //<Password
   unsigned char realm[HTTP_CLIENT_MAX_REALM_LEN + 1u];                         ///<Realm
   HttpAuthQop qop;                                                             ///<Quality of protection
   unsigned char nonce[HTTP_CLIENT_MAX_NONCE_LEN + 1];                          ///<Nonce value
   unsigned char cnonce[HTTP_CLIENT_CNONCE_SIZE * 2 + 1];                       ///<Cnonce value
   unsigned char opaque[HTTP_CLIENT_MAX_OPAQUE_LEN + 1];                        ///<Opaque parameter opaque field sent from server during authentication
   uint32_t nc;                                                                 // nonce count
   uint8_t algorithm;
   uint8_t stale;
   uint8_t init;                                                                // init for random number nonce generation
} HttpClientAuthParams_t;
#else
HTTPPACKED(
typedef struct {
   HttpAuthMode mode;                                                           ///<HTTP authentication mode
   unsigned char username[HTTP_CLIENT_MAX_USERNAME_LEN + 1u];                   ///<User name
   unsigned char password[HTTP_CLIENT_MAX_PASSWORD_LEN + 1u];                   //<Password
   unsigned char realm[HTTP_CLIENT_MAX_REALM_LEN + 1u];                         ///<Realm
   HttpAuthQop qop;                                                             ///<Quality of protection
   unsigned char nonce[HTTP_CLIENT_MAX_NONCE_LEN + 1];                          ///<Nonce value
   unsigned char cnonce[HTTP_CLIENT_CNONCE_SIZE * 2 + 1];                       ///<Cnonce value
   unsigned char opaque[HTTP_CLIENT_MAX_OPAQUE_LEN + 1];                        ///<Opaque parameter opaque field sent from server during authentication
   uint32_t nc;                                                                 // nonce count
   uint8_t algorithm;
   uint8_t stale;
   uint8_t init;                                                                // init for random number nonce generation
}) HttpClientAuthParams_t;
#endif 


/**
 * @brief WWW-Authenticate header field
 **/

typedef struct
{
   HttpAuthMode mode;                                                           ///<Authentication scheme
   const char *realm;                                                           ///<Realm
   size_t realmLen;                                                             ///<Length of the realm
#if (HTTP_CLIENT_DIGEST_AUTH_SUPPORT == ENABLED)
   HttpAuthQop qop;                                                             ///<Quality of protection
   //const HashAlgo *algorithm;    ///<Digest algorithm
   const char *nonce;                                                           ///<Nonce value
   size_t nonceLen;                                                             ///<Length of the nonce value
   const char *opaque;                                                          ///<Opaque parameter
   size_t opaqueLen;                                                            ///<Length of the opaque parameter
   uint8_t stale;                                                               ///<Stale flag
#endif
} HttpWwwAuthenticateHeader_t;

#define HTTP_CLIENT_MAX_METHOD_LEN 8U
#define HTTP_CLIENT_BUFFER_SIZE 2048U

#if defined(D_FT900)
typedef struct HTTPPACKED {
   HttpClientState state;                                                       ///<HTTP client state
   HttpVersion version;                                                         ///<HTTP protocol version
   uint8_t serverIpAddr[4u];
   uint16_t serverPort;
   unsigned char socket1;                                                       // a value indicating the state of the socket being used
//   SOCKET_Intern_Dsc *socket;
   uint8_t socket;
//#if (HTTP_CLIENT_TLS_SUPPORT == ENABLED)
//   TlsContext *tlsContext;                                                      ///<TLS context
//   TlsSessionState tlsSession;                                                  ///<TLS session state
//   HttpClientTlsInitCallback tlsInitCallback;                                   ///<TLS initialization callback function
//#endif
   HttpClientAuthParams_t authParams;                                           ///<HTTP username and password required
   HttpRequestState requestState;                                               ///<HTTP request state
   unsigned char method1[HTTP_CLIENT_MAX_METHOD_LEN + 1u];                       ///<HTTP request method
   uint8_t keepAlive : 1u;                                                       ///<HTTP persistent connection
   uint8_t chunkedEncoding : 1u;                                                 ///<Chunked transfer encoding
   uint8_t spare : 6u;
   unsigned char buffer[HTTP_CLIENT_BUFFER_SIZE + 1u];                           ///<Memory buffer for input/output operations
   size_t bufferLen;                                                            ///<Length of the buffer, in bytes
   size_t bufferPos;                                                            ///<Current position in the buffer
   size_t bodyLen;                                                              ///<Length of the body, in bytes
   size_t bodyPos;                                                              ///<Current position in the body
   uint8_t statusCode;                                                          ///<HTTP status code
   uint64_t timestamp;                                                          ///store for the start of the timer count
   uint64_t timeout;                                                            ///exceeds this counter then message timed out
} HttpClientContext_t;
#else
HTTPPACKED(
typedef struct {
   HttpClientState state;                                                       ///<HTTP client state
   HttpVersion version;                                                         ///<HTTP protocol version
   uint8_t serverIpAddr[4u];
   uint16_t serverPort;
   unsigned char socket1;                                                       // a value indicating the state of the socket being used
//   SOCKET_Intern_Dsc *socket;
   uint8_t socket;
//#if (HTTP_CLIENT_TLS_SUPPORT == ENABLED)
//   TlsContext *tlsContext;                                                      ///<TLS context
//   TlsSessionState tlsSession;                                                  ///<TLS session state
//   HttpClientTlsInitCallback tlsInitCallback;                                   ///<TLS initialization callback function
//#endif
   HttpClientAuthParams_t authParams;                                           ///<HTTP username and password required
   HttpRequestState requestState;                                               ///<HTTP request state
   unsigned char method1[HTTP_CLIENT_MAX_METHOD_LEN + 1u];                       ///<HTTP request method
   uint8_t keepAlive : 1u;                                                       ///<HTTP persistent connection
   uint8_t chunkedEncoding : 1u;                                                 ///<Chunked transfer encoding
   uint8_t spare : 6u;
   unsigned char buffer[HTTP_CLIENT_BUFFER_SIZE + 1u];                           ///<Memory buffer for input/output operations
   size_t bufferLen;                                                            ///<Length of the buffer, in bytes
   size_t bufferPos;                                                            ///<Current position in the buffer
   size_t bodyLen;                                                              ///<Length of the body, in bytes
   size_t bodyPos;                                                              ///<Current position in the body
   uint8_t statusCode;                                                          ///<HTTP status code
   uint64_t timestamp;                                                          ///store for the start of the timer count
   uint64_t timeout;                                                            ///exceeds this counter then message timed out
}) HttpClientContext_t;
#endif  

#define MEM_GUARD 128U
#define NX_CONNECTION_BUF 2048U

#if defined(D_FT900)
typedef struct HTTPPACKED {
  int16_t num_connections;
  int16_t num_requests;
  int16_t num_threads;
  int16_t progress_step;
  // struct addrinfo *saddr;                                                    for linux
  const char* uri_path;
  const char* uri_host;
  const char* ssl_cipher_priority;
  char request_data[4096];
  int16_t request_length;
//#ifdef WITH_SSL                                                                 // We would need to think about this as it doesnt work in the PIC32
//  gnutls_certificate_credentials_t ssl_cred;
//  gnutls_priority_t priority_cache;
//#endif
  int8_t keep_alive:1;
  int8_t secure:1;
  int8_t quiet:1;
  int8_t spare1:5;
  char padding1[MEM_GUARD];                                                    // guard from false sharing
  int16_t request_counter;
  char padding2[MEM_GUARD];
} HttpConfig_t;                                                                // http configuration structure
#else
HTTPPACKED(
typedef struct {
  int16_t num_connections;
  int16_t num_requests;
  int16_t num_threads;
  int16_t progress_step;
  // struct addrinfo *saddr;                                                    for linux
  const char* uri_path;
  const char* uri_host;
  const char* ssl_cipher_priority;
  char request_data[4096];
  int16_t request_length;
//#ifdef WITH_SSL                                                                 // We would need to think about this as it doesnt work in the PIC32
//  gnutls_certificate_credentials_t ssl_cred;
//  gnutls_priority_t priority_cache;
//#endif
  int8_t keep_alive:1;
  int8_t secure:1;
  int8_t quiet:1;
  int8_t spare1:5;
  char padding1[MEM_GUARD];                                                    // guard from false sharing
  int16_t request_counter;
  char padding2[MEM_GUARD];
}) HttpConfig_t;                                                                // http configuration structure
#endif 

#if defined(D_FT900)
typedef struct HTTPPACKED {
  HttpChunkDecStateCode state;
  uint8_t final_chunk:1;
  uint8_t monitor_only:1;
  uint8_t spare:6;
  int64_t chunk_bytes_left;
} HttpChunkDecode_t;
#else
HTTPPACKED(
typedef struct {
  HttpChunkDecStateCode state;
  uint8_t final_chunk:1;
  uint8_t monitor_only:1;
  uint8_t spare:6;
  int64_t chunk_bytes_left;
}) HttpChunkDecode_t;
#endif 

#if defined(D_FT900)
typedef struct HTTPPACKED {
//  struct ev_loop* loop;
//  struct thread_config* tdata;
  int8_t fd;
//  ev_io watch_read;
//  ev_io watch_write;
//  ev_tstamp last_activity;
  HttpChunkDecode_t cdstate;
//#ifdef WITH_SSL
//  gnutls_session_t session;
//#endif
  char ip_addr[15u];
  uint16_t rcv_pt;
  uint16_t dst_pt;
  int16_t write_pos;
  int16_t read_pos;
  int16_t bytes_to_read;
  int16_t bytes_received;
  int8_t alive_count;
  int8_t success_count;
  int8_t keep_alive:1u;
  int8_t chunked:1u;
  int8_t done:1u;
  int8_t secure:1u;
  int8_t spare:4u;
  char buf[NX_CONNECTION_BUF];
  char* body_ptr;
  nx_connection_state state;
  uint16_t fail_counter;
} HttpConnect_t;
#else
HTTPPACKED(
typedef struct {
//  struct ev_loop* loop;
//  struct thread_config* tdata;
  int8_t fd;
//  ev_io watch_read;
//  ev_io watch_write;
//  ev_tstamp last_activity;
  HttpChunkDecode_t cdstate;
//#ifdef WITH_SSL
//  gnutls_session_t session;
//#endif
  char ip_addr[15u];
  uint16_t rcv_pt;
  uint16_t dst_pt;
  int16_t write_pos;
  int16_t read_pos;
  int16_t bytes_to_read;
  int16_t bytes_received;
  int8_t alive_count;
  int8_t success_count;
  int8_t keep_alive:1u;
  int8_t chunked:1u;
  int8_t done:1u;
  int8_t secure:1u;
  int8_t spare:4u;
  char buf[NX_CONNECTION_BUF];
  char* body_ptr;
  nx_connection_state state;
  uint16_t fail_counter;
}) HttpConnect_t;
#endif  

#define HTTP_MAKE_GET(c)  { sprintf(c->request_data,  \
           "\"GET %s HTTP/1.1\r\n\"                                             \
           \"Host: %s\r\n\"                                                     \
           \"Connection: %s\r\n\"                                               \
           \"\r\n\",                                                            \
           ?\"keep-alive\":"close\" ",c.uri_path,c.uri_host,c.keep_alive        \
          );  }                                                                 \

#define HTTP_MAKE_PUT(c)  { sprintf(c.request_data,  \
           "\"PUT %s HTTP/1.1\r\n\"                                             \
           \"Host: %s\r\n\"                                                     \
           \"Connection: %s\r\n\"                                               \
           \"\r\n\",                                                            \
           ?\"keep-alive\":"close\" ",c.uri_path,c.uri_host,c.keep_alive        \
          );  }                                                                 \

int16_t decode_chunked_stream(HttpChunkDecode_t * decoder_state, char* buf, int16_t * buf_len);
char* find_end_of_http_headers(char* buf, int len, char** start_of_body);
void parse_headers(HttpConnect_t* conn);
int16_t parse_uri(const char* uri, HttpConfig_t *config);

//static int16_t decode_chunked_stream(nxweb_chunked_decoder_state_t * decoder_state, char* buf, int16_t * buf_len)
/*******************************************************************************
* Function Name: decode_chunked_stream
********************************************************************************
* Summary:
* decodes a tcp/ip chunked stream
* Parameters:
* HttpChunkDecode_t * decoder_state, char* buf, int16_t * buf_len
* Return:
*  int16_t : -1 on error 1 final chunk complete otherwise 0
*
*******************************************************************************/
HTTP_LIB int16_t decode_chunked_stream(HttpChunkDecode_t * decoder_state, char* buf, int16_t * buf_len)
{
  char* p=buf;
  char* d=buf;
  char* end=buf+*buf_len;
  char c;
  
  if ((decoder_state == NULL) || ((buf == NULL) || (buf_len == NULL)))
     return -1;
     
  while (p<end) 
  {
    c=*p;
    switch (decoder_state->state) 
    {
      case CDS_DATA:
        if (end-p>=decoder_state->chunk_bytes_left) 
        {
          p+=decoder_state->chunk_bytes_left;
          decoder_state->chunk_bytes_left=0;
          decoder_state->state=CDS_CR1;
          d=p;
          break;
        }
        else 
        {
          decoder_state->chunk_bytes_left-=(end-p);
          if (!decoder_state->monitor_only) *buf_len=(end-buf);
          return 0;
        }
      case CDS_CR1:
        if (c!='\r') return -1;
        p++;
        decoder_state->state=CDS_LF1;
        break;
      case CDS_LF1:
        if (c!='\n') return -1;
        if (decoder_state->final_chunk) 
        {
          if (!decoder_state->monitor_only) *buf_len=(d-buf);
          return 1;
        }
        p++;
        decoder_state->state=CDS_SIZE;
        break;
      case CDS_SIZE:                                                            // read digits until CR2
        if (c=='\r') 
        {
          if (!decoder_state->chunk_bytes_left) 
          {
            decoder_state->final_chunk=1;                                       // terminator found
          }
          p++;
          decoder_state->state=CDS_LF2;
        }
        else 
        {
          if (c>='0' && c<='9') c-='0';
          else if (c>='A' && c<='F') c=c-'A'+10;
          else if (c>='a' && c<='f') c=c-'a'+10;
          else return -1;
          decoder_state->chunk_bytes_left=(decoder_state->chunk_bytes_left<<4)+c;
          p++;
        }
        break;
      case CDS_LF2:
        if (c!='\n') return -1;
        p++;
        if (!decoder_state->monitor_only) 
        {
          memmove(d, p, end-p);
          end-=(p-d);
          p=d;
        }
        decoder_state->state=CDS_DATA;
        break;
    }
  }
  if (!decoder_state->monitor_only) *buf_len=(d-buf);
  return 0;
}
/*******************************************************************************
* Function Name: find_end_of_http_headers
********************************************************************************
* Summary:
* returns where the http headers end
* Parameters:
* char* buf, int16_t len, char** start_of_body
* Return:
*  char* where header ends or 0 on error
*
*******************************************************************************/
HTTP_LIB char* find_end_of_http_headers(char* buf, int16_t len, char** start_of_body)
{
  char* p;
  
  if (buf == NULL)
    return '\0';
  if (len<4) return 0;
  for (p=memchr(buf+3, '\n', len-3); p; p=memchr(p+1, '\n', len-(p-buf)-1)) 
  {
    if (*(p-1)=='\n') { *start_of_body=p+1; return p-1; }
    if (*(p-3)=='\r' && *(p-2)=='\n' && *(p-1)=='\r') { *start_of_body=p+1; return p-3; }
  }
  return 0;
}
/*******************************************************************************
* Function Name: parse_headers
********************************************************************************
* Summary:
* parse the header to determine the correct http version
* Parameters:
* HttpConnect_t* conn
* Return:
*  nothing sets the keepalive bit of the structure
*
*******************************************************************************/
HTTP_LIB void parse_headers(HttpConnect_t* conn)
{
  char *p;

  if (conn == NULL)                                                             /* passed a null object then exit */
     return;

  *(conn->body_ptr-1)='\0';
  conn->keep_alive=!strncmp(conn->buf, "HTTP/1.1", 8u);
  conn->bytes_to_read=-1;
  for (p=strchr(conn->buf, '\n'); p; p=strchr(p, '\n'))                         /* for each line */
  {
    p++;
    if (!strncmp(p, "Content-Length:", 15))                                     /* read content length */
    {
      p+=15;
      while (*p==' ' || *p=='\t') p++;
      conn->bytes_to_read=atoi(p);
    }
    else if (!strncmp(p, "Transfer-Encoding:", 18))                             /* read transfer encoding */
    {
      p+=18;
      while (*p==' ' || *p=='\t') p++;
      conn->chunked=!strncmp(p, "chunked", 7);                                  /* read chunked */
    }
    else if (!strncmp(p, "Connection:", 11)) 
    {
      p+=11;
      while (*p==' ' || *p=='\t') p++;
      conn->keep_alive=!strncmp(p, "keep-alive", 10);                           /* read keep alive */
    }
  }

  if (conn->chunked) 
  {
    conn->bytes_to_read=-1;
    memset(&conn->cdstate, 0, sizeof(conn->cdstate));
    conn->cdstate.monitor_only=1;
  }

  conn->bytes_received=conn->read_pos-(conn->body_ptr-conn->buf);               // what already read
}
/*******************************************************************************
* Function Name: parse_uri
********************************************************************************
* Summary:
* parse the uri
* Parameters:
* const char* uri, HttpConfig_t *config
* Return:
*  int16_t  0=success -1=error
*
*******************************************************************************/
HTTP_LIB int16_t parse_uri(const char* uri, HttpConfig_t *config)
{
  static char host_buf[1024U];
  const char* p;

  if ((uri == NULL) || (config == NULL))
     return -1;
     
  p=strchr((char*)uri,'/');
  if (!strncmp((char*)uri, "http://", 7)) uri+=7;
#ifdef WITH_SSL                                                                 // not currently supported
  else if (!strncmp(uri, "https://", 8)) { uri+=8; config->secure=1; }
#endif
  else return -1;

  if (!p) 
  {
    config->uri_host=uri;
    config->uri_path="/";
    return 0;
  }
  if ((p - uri)>sizeof(host_buf)-1) return -1;
  strncpy((char*)host_buf,(char*) uri,(int) (p - uri));
  host_buf[(p - uri)]='\0';
  config->uri_host=host_buf;
  config->uri_path=p;
  return 0;
}

#define ERR_AGAIN -2
#define ERR_ERROR -1 
#define ERR_RDCLOSED -3

#define HTTP_NO_READ_NOT_CHUNK (1u<<4u)                                         // didnt read anything and not chunked
#define HTTP_NO_READ_CHUNK (1u<<5u)                                             // didnt read anything and chunked
#define HTTP_HEAD_NO_BODY (1u<<6u)                                              // read head no body
#define HTTP_NO_CHUNK (1u<<7u)                                                  // no chunk decode
#define HTTP_HEAD_CHUNK (1u<<8u)                                                // header and chunk
#define HTTP_INVALID_HEAD (1u<<9u)                                              // not a valid header structure
#define HTTP_BODY_EMPTY (1u<<10u)                                               // no body
#define HTTP_BODY_NO_CHUNK (1u<<11u)                                            // body no chunk
#define HTTP_BODY_CHUNK (1u<<12u)                                               // body chunk
#define HTTP_BODY_READ (1u<<13u)                                                // body read
#define READ_CHUNK_HEAD (1u<<14u)                                               // read chunked head
#define HTTP_UNKNOWN (1u<<15u)                                                  // unknown
#define error_t int16_t                                                         // define error type used in cyclone
#define char_t char

#define NO_ERROR 0u                                                             // error handlers for http
#define ERROR_INVALID_PARAMETER 1u
#define ERROR_INVALID_LENGTH 2u
#define ERROR_INVALID_SYNTAX 3u
#define ERROR_BUFFER_OVERFLOW 4u
#define ERROR_WRONG_STATE 5u
#define ERROR_TIMEOUT 6u
#define ERROR_WOULD_BLOCK 7u
#define ERROR_END_OF_STREAM 8u
#define ERROR_INVALID_VERSION 9u

#define HTTP09_GET_REQ "GET / HTTP/0.9"
#define HTTP09_PUT_REQ "PUT / HTTP/0.9"
#define HTTP09_POST_REQ "POST / HTTP/0.9"
#define HTTP10_GET_REQ "GET / HTTP/1.0"
#define HTTP10_PUT_REQ "PUT / HTTP/1.0"
#define HTTP10_POST_REQ "POST / HTTP/1.0"
#define HTTP10_DELETE_REQ "DELETE / HTTP/1.0"
#define HTTP11_GET_REQ "GET / HTTP/1.1"                                         // get request for current version of html
#define HTTP11_PUT_REQ "PUT / HTTP/1.1"                                         // put request for current version of html
#define HTTP11_POST_REQ "POST / HTTP/1.1"                                       // post request for current version of html
#define HTTP11_DELETE_REQ "DELETE / HTTP/1.1"                                   // delete request for current version of html

#if (HTTP_CLIENT_SHA512_256_SUPPORT == ENABLED)
   #define HTTP_CLIENT_MAX_RESPONSE_LEN 64u
#elif (HTTP_CLIENT_SHA256_SUPPORT == ENABLED)
   #define HTTP_CLIENT_MAX_RESPONSE_LEN 64u
#else
   #define HTTP_CLIENT_MAX_RESPONSE_LEN 32u
#endif

#ifdef __cplusplus
}
#endif

#endif