#ifndef _UbiDotLib_H_
#define _UbiDotLib_H_
/* =============================================================================
Copyright (c) 2013-2018 Ubidots.
Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:
The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Developed and maintained by Jose Garcia Reyes and Cristian Arrieta Pacheco 
for IoT Services Inc

@jotathebest at github: https://github.com/jotathebest
@crisap94 at github: https://github.com/crisap94

ported modiifed and compiled for pic32 by ACP Aviation
==============================================================================*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define UBIDOTPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define UBIDOTPACKED __attribute__((packed))                                 /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define UBIDOTPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define UBIDOTPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define UBIDOTPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

/* ------------------ UbiDots Connection Info -------------------------- */
#define UBIDOT_POST(X,URL) do{ sprintf(X,"POST %s HTTP/1.1",URL); }while(0)
#define UBIDOT_EDU_HOST "Host : things.ubidots.com"                             /* UBIDOTS IOT education test server */
#define UBIDOT_INDUS_HOST "Host : industrial.api.ubidots.com"                   /* UBIDOTS IOT industrial information and position server */
#define JSON_CONTEXT "Context-Type : application/json"
#define HTTP_CONTEXT_LEN(X,Y) do{ sprintf(X,"Context Length : %d",Y); }while(0)
#define UBIDOTS_INDUSTRIAL_IP "169.55.61.243"
#define UBIDOTS_USER_AGENT = "UbidotsSIM900\/4.0.1";
#define UBIDOTS_HTTP_PORT 80u
#define UBIDOTS_HTTPS_PORT 443u
#define UBIDOTS_TCP_PORT 9012u
#define UBIDOTS_TCPS_PORT 9812u
#define _connectionTimeout 5000u
#define _maxConnectionAttempts = 20;
#define UBI_ERROR_VALUE = -3.4028235E+8f;
#define MIN_UBI_PAYLOAD_HEAD_LEN 100u                                           /* minimum message header length */
#define MIN_UBI_PAYLOAD_ITEM_LEN 20u                                            /* minimum message value length */

#define UBI_CONTEXT_MAX_VALUES 100u                                             /* define you number of contexts and points per upload structure */
#define UBI_POINT_MAX_VALUES 100u

/* @device_label, [Required]. The device label which contains the variable to retrieve values from.
   @variable_label, [Required]. The variable label to retrieve values from. */
#if defined(D_FT900)
typedef struct UBIDOTPACKED {
  char *key_label;
  char *key_value;
} ContextUbi_t;
#else
UBIDOTPACKED(
typedef struct {
  char *key_label;
  char *key_value;
}) ContextUbi_t;
#endif

/* @variable_label, [Required]. The label of the variable where the dot will be stored.
   @value, [Required]. The value of the dot.
   @context, [Optional]. The dot's context.
   @dot_timestamp_seconds, [Optional]. The dot's timestamp in seconds.
   @dot_timestamp_millis, [Optional]. The dot's timestamp number of milliseconds. If the timestamp's milliseconds values is not set, the seconds will be multplied by 1000. */
#if defined(D_FT900)
typedef struct UBIDOTPACKED {
  const char *variable_label;
  char *dot_context;
  float32_t dot_value;
  uint32_t dot_timestamp_seconds;
  uint16_t dot_timestamp_millis;
} ubiValue_t;
#else
UBIDOTPACKED(
typedef struct {
  const char *variable_label;
  char *dot_context;
  float32_t dot_value;
  uint32_t dot_timestamp_seconds;
  uint16_t dot_timestamp_millis;
}) ubiValue_t;
#endif

typedef enum { UBI_HTTP, UBI_TCP, UBI_UDP } IotProtocol_e;

/* =====================================================================================================================
 @token, [Required]. Your Ubidots unique account [TOKEN](http://help.ubidots.com/user-guides/find-your-token-from-your-ubidots-account).
 @apn, [Required]. Your simcard operator APN setting, usually it's an URL.
 @apnUser, [Required]. Your APN user set by the Simcard operator, some operators don't use an user to authenticate.
 @apnPass, [Required].  Your APN password set by the Simcard operator, some operators don't use a password to authenticate.
 @server, [Optional], [Options] = [`UBI_INDUSTRIAL`, `UBI_EDUCATIONAL`], [Default] = `UBI_INDUSTRIAL`. The server to send data, set `UBI_EDUCATIONAL` if your account is educational type.
 @iot_protocol, [Optional],[Default] = IotProtocol_e. The IoT protocol that you will use to send or retrieve data.     
   ====================================================================================================================== */
#if defined(D_FT900)
typedef struct UBIDOTPACKED {
  const char *UbiServer;
  const char *UbiApn;
  const char* user_agent;
  const char* _token;
  const char *device_label;
  const char *device_name;
  uint8_t _current_value;
  uint8_t _current_context;
  IotProtocol_e ubiType;
} UbiProtocolHandler_t;
#else
UBIDOTPACKED(
typedef struct {
  const char *UbiServer;
  const char *UbiApn;
  const char* user_agent;
  const char* _token;
  const char *device_label;
  const char *device_name;
  uint8_t _current_value;
  uint8_t _current_context;
  IotProtocol_e ubiType;
}) UbiProtocolHandler_t;
#endif
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif