/*
 * Copyright (c) 2004-2013 Sergey Lyubka <valenok@gmail.com>
 * Copyright (c) 2018 Cesanta Software Limited
 * Also contains some code from nxjson
 * Copyright (c) 2013 Yaroslav Stavnichiy <yarosla@gmail.com>
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Ported for mikroE C by ACP Avaiation  (in progress)
 * Added functionality for Sequoia Cam http:// get or put
 *
 * JSON is a natural choice if the client is a web browser because web browsers 
 * have built-in support for parsing JSON to JavaScript objects. But JSON is also 
 * often used as data format between backend services despite not being the fastest, 
 * most compact or most flexible and expressive data format you could use for that purpose. 
 *
 * Second, we will look at RION against ProtoBuf, MessagePack and CBOR which are all binary data formats.
 * Since RION is a binary data format it is more fair to benchmark RION against these data formats than JSON. 
 * For Protobuf we will use Google's Protocol Buffers implementation. For MessagePack and CBOR we will use Jackson's implementations
 */
#define FROZEN_API extern
 
#define _CRT_SECURE_NO_WARNINGS                                                 /* Disable deprecation warning in VS2005+ */

#include "definitions.h"
#include "p32mx795f512l.h"                                                      // we are using the PIC32 used to disable file functions

#include "frozen.h"
#include "http.h"
#include <stdint.h>
#include "gc_events.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(WEAK)
#if (defined(__GNUC__) || defined(__TI_COMPILER_VERSION__)) && !defined(_WIN32)
#define WEAK __attribute__((weak))
#else
#define WEAK
#endif
#endif

#ifdef _WIN32
#undef snprintf
#undef vsnprintf
#define snprintf cs_win_snprintf
#define vsnprintf cs_win_vsnprintf
int cs_win_snprintf(char *str, size_t size, const char *format, ...);
int cs_win_vsnprintf(char *str, size_t size, const char *format, va_list ap);

//#if _MSC_VER >= 1700
#include "stdint.h"
//#else
//typedef _int64 int64_t;
//typedef unsigned _int64 uint64_t;
//#endif
#ifdef PRId64
#undef PRId64
#endif
#define PRId64 "I64d"
#ifdef PRIu64
#undef PRIu64
#endif
#define PRIu64 "I64u"
#else /* _WIN32 */
/* <inttypes.h> wants this for C++ */
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>
#endif /* _WIN32 */
#ifndef INT64_FMT
#define INT64_FMT PRId64
#endif
#ifndef UINT64_FMT
#define UINT64_FMT PRIu64
#endif

#ifndef va_copy
#define va_copy(x, y) x = y
#endif

#ifndef JSON_ENABLE_ARRAY
#define JSON_ENABLE_ARRAY 1
#endif

struct frozen {
  const char *end;
  const char *cur;
  const char *cur_name;
  size_t cur_name_len;

  /* For callback API */
  char path[JSON_MAX_PATH_LEN];
  size_t path_len;
  void *callback_data;
  json_walk_callback_t callback;
};

typedef struct {
  const char *ptr;
  size_t path_len;
} fstate2_t;

fstate2_t fstate;

//    variable arguments macros

#ifndef   _STDARG

typedef void *   va_list[1];
#define   va_start(ap, parmn)   *ap = (char *)&parmn + sizeof(parmn)
#define   va_copy(dst, src) ((void)((dst) = (src)))
#define   va_arg(ap, type)   (*(*(type **)ap)++)
#define   _STDARG

#endif   /* STDARG */
/*******************************************************************************
* Function Name: json_append_to_path
********************************************************************************
* Summary:
* append to path
* Parameters:
* struct frozen *f, const char *str, int size
* Return:
*  int
*
*******************************************************************************/
static int json_append_to_path(struct frozen *f, const char *str, int16_t size)
{
  int16_t n = f->path_len;
  int16_t left = sizeof(f->path) - n - 1;
  if (size > left) size = left;
  memcpy((void*) f->path + n,(void*) str, size);
  f->path[n + size] = '\0';
  f->path_len += size;
  return n;
}

/*******************************************************************************
* Function Name: SET_STATE
********************************************************************************
* Summary:
* set state
* Parameters:
* struct frozen *fr,const char *ptr,const char *str,int len
* Return:
*  nothing
*
*******************************************************************************/
#if (MCU_NAME == PIC32MX795F512L)                                               // from definitions.h
static void SET_STATE(struct frozen *fr,const char *ptr,const char *str,int16_t len) 
{

  fstate.ptr = ptr;
  fstate.path_len = fr->path_len;
  // fstate = {(ptr), (fr)->path_len};
  json_append_to_path((fr),(str),(len));
}

typedef struct {                                                                // a value of 1 means active 0 inactive 2 unknown
      int16_t value;
      int16_t len;
      int16_t tok;
} json_token;                                                                   // structure holding a json token
json_token t;

/*-----------------------------------------------------------------------------
 *      CALL_BACK : Callback
 *
 *  Parameters: struct frozen *fr,int tok,int value,int len
 *
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
static void CALL_BACK(struct frozen *fr, int16_t tok, int16_t value, int16_t len)
{
//  do {
        if ((fr)->callback &&
        ((fr)->path_len == 0 || (fr)->path[(fr)->path_len - 1] != '.'))
        {
         //struct json_token t = {(value), (int) (len), (tok)};
            t.value=value;
            t.len=len;
            t.tok=tok;
      
         /* Call the callback with the given value and current name */
           (fr)->callback((fr)->callback_data, (fr)->cur_name, (fr)->cur_name_len, (fr)->path, &t);

         /* Reset the name */
           (fr)->cur_name = NULL;
           (fr)->cur_name_len = 0;
        }
//     } while (0)
}
#else
#define SET_STATE(fr, ptr, str, len)              \
  fstate = {(ptr), (fr)->path_len}; \
  json_append_to_path((fr), (str), (len));

/* Call the callback with the given value and current name */
#define CALL_BACK(fr, tok, value, len)                                        \
  do {                                                                        \
    if ((fr)->callback &&                                                     \
        ((fr)->path_len == 0 || (fr)->path[(fr)->path_len - 1] != '.'))       \
    {                                                                         \
      struct json_token t = {(value), (int) (len), (tok)};                    \
                                                                              \
                \
      (fr)->callback((fr)->callback_data, (fr)->cur_name, (fr)->cur_name_len, \
                     (fr)->path, &t);                                         \
                                                                              \
      /* Reset the name */                                                    \
      (fr)->cur_name = NULL;                                                  \
      (fr)->cur_name_len = 0;                                                 \
    }                                                                         \
  } while (0)
  
#endif
// Returns true if c is a printable ASCII character.  We test the
// value of c directly instead of calling isprint(), which is buggy on
// Windows Mobile.
#undef isprint
static int16_t isprint(char c)
{
  return 0x20U <= c && c <= 0x7EU;
}

/*-----------------------------------------------------------------------------
 *      json_truncate_path : null terminate path string in frozen structure
 *
 *  Parameters: struct frozen *f, size_t len
 *
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
static void json_truncate_path(struct frozen *f, size_t len) 
{
  f->path_len = len;
  f->path[len] = '\0';
}

/*-----------------------------------------------------------------------------
 *      json_parse_object : parse JSON object in frozen structure
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
static int json_parse_object(struct frozen *f);
static int json_parse_value(struct frozen *f);

#define EXPECT(cond, err_code)      \
  do {                              \
    if (!(cond)) return (err_code); \
  } while (0)


#define TRY(expr)          \
  do {                     \
    int16_t _n = expr;         \
    if (_n < 0) return _n; \
  } while (0)

#define END_OF_STRING (-1)

/*-----------------------------------------------------------------------------
 *      json_left :
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        integer representing lenght from end to current
 *----------------------------------------------------------------------------*/
static int16_t json_left(const struct frozen *f)
{
  return f->end - f->cur;
}

/*-----------------------------------------------------------------------------
 *      json_isspace :
 *
 *  Parameters: int ch
 *
 *  Return:
 *        integer which is 1 if space found
 *----------------------------------------------------------------------------*/
static int16_t json_isspace(int16_t ch) 
{
  return ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n';
}

/*-----------------------------------------------------------------------------
 *      json_skip_whitespaces : skip the spaces
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        none
 *----------------------------------------------------------------------------*/
static void json_skip_whitespaces(struct frozen *f) {
  while (f->cur < f->end && json_isspace(*f->cur)) f->cur++;
}

/*-----------------------------------------------------------------------------
 *      json_cur :
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        int pointer to current or end of string
 *----------------------------------------------------------------------------*/
static int16_t json_cur(struct frozen *f) 
{
  json_skip_whitespaces(f);
  return f->cur >= f->end ? END_OF_STRING : *(unsigned char *) f->cur;
}

/*-----------------------------------------------------------------------------
 *      json_test_and_skip :
 *
 *  Parameters: struct frozen *f,  int expected
 *
 *  Return:
 *        int 0=ok else JSON_STRING_INCOMPLETE or JSON_STRING_INVALID
 *----------------------------------------------------------------------------*/
static int16_t json_test_and_skip(struct frozen *f, int16_t expected)
{
  int16_t ch = json_cur(f);
  if (ch == expected) {
    f->cur++;
    return 0;
  }
  return ch == END_OF_STRING ? JSON_STRING_INCOMPLETE : JSON_STRING_INVALID;
}

/*-----------------------------------------------------------------------------
 *      json_isalpha :
 *
 *  Parameters: int ch
 *
 *  Return:
 *        int 1=alpha numeric character
 *----------------------------------------------------------------------------*/
static int16_t json_isalpha(int16_t ch) 
{
  return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z');
}

/*-----------------------------------------------------------------------------
 *      json_isdigit :
 *
 *  Parameters: int ch
 *
 *  Return:
 *        int 1=char is a digital number
 *----------------------------------------------------------------------------*/
static int16_t json_isdigit(int16_t ch) 
{
  return ch >= '0' && ch <= '9';
}

/*-----------------------------------------------------------------------------
 *      json_isxdigit :
 *
 *  Parameters: int ch
 *
 *  Return:
 *        int 1=if the character is a valid hexadecimal number
 *----------------------------------------------------------------------------*/
static int json_isxdigit(int ch) {

  return json_isdigit(ch) || (ch >= 'a' && ch <= 'f') ||
         (ch >= 'A' && ch <= 'F');
}

/*-----------------------------------------------------------------------------
 *      json_get_escape_len :
 *
 *  Parameters: int ch
 *
 *  Return:
 *        int 1=ok else JSON_STRING_INVALID or JSON_STRING_INCOMPLETE
 *----------------------------------------------------------------------------*/
static int16_t json_get_escape_len(const char *s, int16_t len) 
{

  switch (*s) {

    case 'u':
      return len < 6 ? JSON_STRING_INCOMPLETE
                     : json_isxdigit(s[1u]) && json_isxdigit(s[2u]) &&
                               json_isxdigit(s[3u]) && json_isxdigit(s[4u])
                           ? 5
                           : JSON_STRING_INVALID;

    case '"':
    case '\\':
    case '/':
    case 'b':
    case 'f':
    case 'n':
    case 'r':
    case 't':
      return len < 2 ? JSON_STRING_INCOMPLETE : 1;
    default:
      return JSON_STRING_INVALID;
  }
}

/* identifier = letter { letter | digit | '_' } */
/*-----------------------------------------------------------------------------
 *      json_parse_identifier :
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        int = 0
 *----------------------------------------------------------------------------*/
static int16_t json_parse_identifier(struct frozen *f) 
{
  EXPECT(json_isalpha(json_cur(f)), JSON_STRING_INVALID);
  {
    SET_STATE(f, f->cur, "", 0);
    while (f->cur < f->end && (*f->cur == '_' || json_isalpha(*f->cur) || json_isdigit(*f->cur)))
    {
      f->cur++;
    }
    json_truncate_path(f, fstate.path_len);
#if (MCU_NAME == PIC32MX795F512L)
    CALL_BACK(f, JSON_TYPE_STRING,(int16_t) fstate.ptr,(int16_t) f->cur - (int16_t) fstate.ptr);
#else
    CALL_BACK(f, JSON_TYPE_STRING, fstate.ptr, f->cur - fstate.ptr);
#endif
  }
  return 0;
}

/*-----------------------------------------------------------------------------
 *      json_get_utf8_char_len :
 *
 *  Parameters: unsigned char ch
 *
 *  Return:
 *        utf char length
 *----------------------------------------------------------------------------*/
static int16_t json_get_utf8_char_len(unsigned char ch) 
{
  if ((ch & 0x80u) == 0u) return 1;
  switch (ch & 0xf0u) 
  {
    case 0xf0u:
      return 4;

    case 0xe0u:
      return 3;

    default:
      return 2;
  }
}

/*-----------------------------------------------------------------------------
 *      json_parse_string :
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        int = 0 or JSON_STRING_INCOMPLETE
 *----------------------------------------------------------------------------*/
/* string = '"' { quoted_printable_chars } '"' */
static int16_t json_parse_string(struct frozen *f)
{
  int16_t n, ch = 0, len = 0;
  TRY(json_test_and_skip(f, '"'));
  {
    SET_STATE(f, f->cur, "", 0);
    for (; f->cur < f->end; f->cur += len) 
    {
      ch = *(unsigned char *) f->cur;
      len = json_get_utf8_char_len((unsigned char) ch);
      EXPECT(ch >= 32 && len > 0, JSON_STRING_INVALID);                         /* No control chars */
      EXPECT(len <= json_left(f), JSON_STRING_INCOMPLETE);
      if (ch == '\\') 
      {
        EXPECT((n = json_get_escape_len(f->cur + 1, json_left(f))) > 0, n);
        len += n;
      } 
      else if (ch == '"') 
      {
        json_truncate_path(f, fstate.path_len);
#if (MCU_NAME == PIC32MX795F512L)
        CALL_BACK(f, JSON_TYPE_STRING,(int16_t) fstate.ptr,(int16_t) f->cur - (int16_t) fstate.ptr);
#else
        CALL_BACK(f, JSON_TYPE_STRING, fstate.ptr, f->cur - fstate.ptr);
#endif
        f->cur++;
        break;
      };
    }
  }
  return ch == '"' ? 0 : JSON_STRING_INCOMPLETE;
}

/* number = [ '-' ] digit+ [ '.' digit+ ] [ ['e'|'E'] ['+'|'-'] digit+ ] */
/*-----------------------------------------------------------------------------
 *      json_parse_number :
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        int = 0 or JSON_STRING_INCOMPLETE or JSON_STRING_INVALID
 *----------------------------------------------------------------------------*/
static int16_t json_parse_number(struct frozen *f)
{

  int16_t ch = json_cur(f);

  SET_STATE(f, f->cur, "", 0);
  if (ch == '-') f->cur++;
  EXPECT(f->cur < f->end, JSON_STRING_INCOMPLETE);

  if (f->cur + 1 < f->end && f->cur[0u] == '0' && f->cur[1u] == 'x')
  {
    f->cur += 2;
    EXPECT(f->cur < f->end, JSON_STRING_INCOMPLETE);
    EXPECT(json_isxdigit(f->cur[0u]), JSON_STRING_INVALID);
    while (f->cur < f->end && json_isxdigit(f->cur[0u])) f->cur++;
  } 
  else 
  {
    EXPECT(json_isdigit(f->cur[0u]), JSON_STRING_INVALID);

    while (f->cur < f->end && json_isdigit(f->cur[0u])) f->cur++;
    if (f->cur < f->end && f->cur[0u] == '.')
    {
      f->cur++;
      EXPECT(f->cur < f->end, JSON_STRING_INCOMPLETE);
      EXPECT(json_isdigit(f->cur[0u]), JSON_STRING_INVALID);
      while (f->cur < f->end && json_isdigit(f->cur[0u])) f->cur++;
    }

    if (f->cur < f->end && (f->cur[0] == 'e' || f->cur[0] == 'E')) 
    {
      f->cur++;
      EXPECT(f->cur < f->end, JSON_STRING_INCOMPLETE);
      if ((f->cur[0] == '+' || f->cur[0u] == '-')) f->cur++;
      EXPECT(f->cur < f->end, JSON_STRING_INCOMPLETE);
      EXPECT(json_isdigit(f->cur[0u]), JSON_STRING_INVALID);
      while (f->cur < f->end && json_isdigit(f->cur[0u])) f->cur++;
    }
  }
  json_truncate_path(f, fstate.path_len);
#if (MCU_NAME == PIC32MX795F512L)
  CALL_BACK(f, JSON_TYPE_NUMBER,(int16_t) fstate.ptr,(int16_t) f->cur - (int16_t) fstate.ptr);
#else
  CALL_BACK(f, JSON_TYPE_NUMBER, fstate.ptr, f->cur - fstate.ptr);
#endif
  return 0;
}

#if JSON_ENABLE_ARRAY
/* array = '[' [ value { ',' value } ] ']' */
/*-----------------------------------------------------------------------------
 *      json_parse_array :
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        int = 0
 *----------------------------------------------------------------------------*/
static int16_t json_parse_array(struct frozen *f)
{

  int16_t i = 0, current_path_len;
  char buf[20u];

  CALL_BACK(f, JSON_TYPE_ARRAY_START, NULL, 0);

  TRY(json_test_and_skip(f, '['));
  {
    {
      SET_STATE(f, f->cur - 1, "", 0);

      while (json_cur(f) != ']') 
      {
        snprintf(buf, sizeof(buf), "[%d]", i);
        i++;
        current_path_len = json_append_to_path(f, buf, strlen(buf));
        f->cur_name = f->path + strlen(f->path) - strlen(buf) + 1;              /*opening brace*/
        f->cur_name_len = strlen(buf) - 2;                                      /*braces*/

        TRY(json_parse_value(f));
        json_truncate_path(f, current_path_len);
        if (json_cur(f) == ',') f->cur++;
      }
      TRY(json_test_and_skip(f, ']'));
      json_truncate_path(f, fstate.path_len);
#if (MCU_NAME == PIC32MX795F512L)
      CALL_BACK(f, JSON_TYPE_ARRAY_END,(int16_t) fstate.ptr,(int16_t) f->cur - (int16_t) fstate.ptr);
#else
      CALL_BACK(f, JSON_TYPE_ARRAY_END, fstate.ptr, f->cur - fstate.ptr);
#endif
    }
  }
  return 0;
}
#endif /* JSON_ENABLE_ARRAY */

/*-----------------------------------------------------------------------------
 *      json_expect :
 *
 *  Parameters: struct frozen *f, const char *s, int len, enum json_token_type tok_type
 *
 *  Return:
 *        int = 0 or JSON_STRING_INCOMPLETE or JSON_STRING_INVALID
 *----------------------------------------------------------------------------*/
static int16_t json_expect(struct frozen *f, const char *s, int16_t len, enum json_token_type tok_type) 
{
  int16_t i, n = json_left(f);

  SET_STATE(f, f->cur, "", 0);
  for (i = 0; i < len; i++) 
  {
    if (i >= n) return JSON_STRING_INCOMPLETE;
    if (f->cur[i] != s[i]) return JSON_STRING_INVALID;
  }
  f->cur += len;
  json_truncate_path(f, fstate.path_len);
#if (MCU_NAME == PIC32MX795F512L)
  CALL_BACK(f,(int16_t) tok_type,(int16_t) fstate.ptr,(int16_t) f->cur - (int16_t) fstate.ptr);
#else
  CALL_BACK(f, tok_type, fstate.ptr, f->cur - fstate.ptr);
#endif
  return 0;
}

/* value = 'null' | 'true' | 'false' | number | string | array | object */
/*-----------------------------------------------------------------------------
 *      json_parse_value :
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        int = 0 or JSON_STRING_INCOMPLETE or JSON_STRING_INVALID
 *----------------------------------------------------------------------------*/
static int16_t json_parse_value(struct frozen *f) 
{
  int16_t ch = json_cur(f);

  switch (ch) 
  {
    case '"':
      TRY(json_parse_string(f));
      break;

    case '{':
      TRY(json_parse_object(f));
      break;

#if JSON_ENABLE_ARRAY
    case '[':
      TRY(json_parse_array(f));
      break;
#endif

    case 'n':
      TRY(json_expect(f, "null", 4U, JSON_TYPE_NULL));
      break;

    case 't':
      TRY(json_expect(f, "true", 4U, JSON_TYPE_TRUE));
      break;

    case 'f':
      TRY(json_expect(f, "false", 5U, JSON_TYPE_FALSE));
      break;

    case '-':

    case '0':

    case '1':

    case '2':

    case '3':

    case '4':

    case '5':

    case '6':

    case '7':

    case '8':

    case '9':
      TRY(json_parse_number(f));

      break;

    default:
      return ch == END_OF_STRING ? JSON_STRING_INCOMPLETE : JSON_STRING_INVALID;
  }
  return 0;
}

/* key = identifier | string */
/*-----------------------------------------------------------------------------
 *      json_parse_key :
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        int = 0 or JSON_STRING_INCOMPLETE or JSON_STRING_INVALID
 *----------------------------------------------------------------------------*/
static int16_t json_parse_key(struct frozen *f) 
{
  int16_t ch = json_cur(f);

  if (json_isalpha(ch)) 
  {
    TRY(json_parse_identifier(f));
  } 
  else if (ch == '"') 
  {
    TRY(json_parse_string(f));
  } 
  else 
  {
    return ch == END_OF_STRING ? JSON_STRING_INCOMPLETE : JSON_STRING_INVALID;
  }
  return 0;
}

/* pair = key ':' value */
/*-----------------------------------------------------------------------------
 *      json_parse_pair :
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        int = 0 or JSON_STRING_INCOMPLETE or JSON_STRING_INVALID
 *----------------------------------------------------------------------------*/
static int16_t json_parse_pair(struct frozen *f)
{
  int16_t current_path_len;
  const char *tok;
  json_skip_whitespaces(f);
  tok = f->cur;
  TRY(json_parse_key(f));
  {
    f->cur_name = *tok == '"' ? tok + 1 : tok;
    f->cur_name_len = *tok == '"' ? f->cur - tok - 2 : f->cur - tok;
    current_path_len = json_append_to_path(f, f->cur_name, f->cur_name_len);
  }

  TRY(json_test_and_skip(f, ':'));
  TRY(json_parse_value(f));
  json_truncate_path(f, current_path_len);
  return 0;
}

/* object = '{' pair { ',' pair } '}' */
/*-----------------------------------------------------------------------------
 *      json_parse_object :
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        int = 0 or JSON_STRING_INCOMPLETE or JSON_STRING_INVALID
 *----------------------------------------------------------------------------*/
static int16_t json_parse_object(struct frozen *f)
{
  CALL_BACK(f, JSON_TYPE_OBJECT_START, NULL, 0);
  TRY(json_test_and_skip(f, '{'));
  {
    SET_STATE(f, f->cur - 1, ".", 1);
    while (json_cur(f) != '}') 
    {
      TRY(json_parse_pair(f));
      if (json_cur(f) == ',') f->cur++;
    }

    TRY(json_test_and_skip(f, '}'));
    json_truncate_path(f, fstate.path_len);
#if (MCU_NAME == PIC32MX795F512L)
    CALL_BACK(f, JSON_TYPE_OBJECT_END,(int) fstate.ptr,(int) f->cur - (int) fstate.ptr);
#else
    CALL_BACK(f, JSON_TYPE_OBJECT_END, fstate.ptr, f->cur - fstate.ptr);
#endif
  }
  return 0;
}

/*-----------------------------------------------------------------------------
 *      json_doit :
 *
 *  Parameters: struct frozen *f
 *
 *  Return:
 *        int = 0 or JSON_STRING_INCOMPLETE or JSON_STRING_INVALID
 *----------------------------------------------------------------------------*/
static int16_t json_doit(struct frozen *f)
{
  if (f->cur == 0 || f->end < f->cur) return JSON_STRING_INVALID;
  if (f->end == f->cur) return JSON_STRING_INCOMPLETE;
  return json_parse_value(f);
}

/*-----------------------------------------------------------------------------
 *      json_escape :
 *
 *  Parameters: struct frozen *f const char *p, size_t len
 *
 *  Return:
 *        int = n number of chars in out
 *----------------------------------------------------------------------------*/
int json_escape(struct json_out *out, const char *p, size_t len) WEAK;
int json_escape(struct json_out *out, const char *p, size_t len) {

  size_t i, cl, n = 0;

  const char *hex_digits = "0123456789abcdef";
  const char *specials = "btnvfr";

  for (i = 0; i < len; i++) 
  {
    unsigned char ch = ((unsigned char *) p)[i];
    if (ch == '"' || ch == '\\') {
      n += out->printer(out, "\\", 1);
      n += out->printer(out, p + i, 1);
    } else if (ch >= '\b' && ch <= '\r') {
      n += out->printer(out, "\\", 1);
      n += out->printer(out, &specials[ch - '\b'], 1);
    } else if (isprint(ch)) {
      n += out->printer(out, p + i, 1);
    } else if ((cl = json_get_utf8_char_len(ch)) == 1) {
      n += out->printer(out, "\\u00", 4);
      n += out->printer(out, &hex_digits[(ch >> 4u) % 0xfu], 1);
      n += out->printer(out, &hex_digits[ch % 0xfu], 1);
    } else {
      n += out->printer(out, p + i, cl);
      i += cl - 1;
    }
  }
  return n;
}

/*-----------------------------------------------------------------------------
 *      json_printer_buf :
 *
 *  Parameters: struct frozen *f const char *p, size_t len
 *
 *  Return:
 *        int = max(len length of string or avail (size of buffer))
 *----------------------------------------------------------------------------*/
int16_t json_printer_buf(struct json_out *out, const char *buf, size_t len) WEAK;
int16_t json_printer_buf(struct json_out *out, const char *buf, size_t len) 
{
  size_t avail = out->u.buf.size - out->u.buf.len;
  size_t n = len < avail ? len : avail;
  memcpy((void*) out->u.buf.buf + out->u.buf.len,(void *) buf, n);
  out->u.buf.len += n;

  if (out->u.buf.size > 0) {
    size_t idx = out->u.buf.len;
    if (idx >= out->u.buf.size) idx = out->u.buf.size - 1;
    out->u.buf.buf[idx] = '\0';
  }
  return len;
}

#ifndef __32MX795F512L_H                                                        // dont include file functions with the pic32
int16_t json_printer_file(struct json_out *out, const char *buf, size_t len) WEAK;
int16_t json_printer_file(struct json_out *out, const char *buf, size_t len) 
{
  return fwrite(buf, 1, len, out->u.fp);                                        // CANT FILE WRITE IN PIC32
}
#endif

#if defined(JSON_ENABLE_BASE64)
static int16_t b64idx(int16_t c)
{
  if (c < 26) {
    return c + 'A';
  } else if (c < 52) {
    return c - 26 + 'a';
  } else if (c < 62) {
    return c - 52 + '0';
  } else {
    return c == 62 ? '+' : '/';
  }
}

static int16_t b64rev(int16_t c)
{
  if (c >= 'A' && c <= 'Z') {
    return c - 'A';
  } else if (c >= 'a' && c <= 'z') {
    return c + 26 - 'a';
  } else if (c >= '0' && c <= '9') {
    return c + 52 - '0';
  } else if (c == '+') {
    return 62;
  } else if (c == '/') {
    return 63;
  } else {
    return 64;
  }
}

static int16_t b64enc(struct json_out *out, const unsigned char *p, int16_t n) 
{

  char buf[4u];
  int16_t i, len = 0;
  int16_t a, b, c;
  
  for (i = 0; i < n; i += 3) 
  {
    a = p[i];
    b = i + 1 < n ? p[i + 1] : 0;
    c = i + 2 < n ? p[i + 2] : 0;
    buf[0u] = b64idx(a >> 2u);
    buf[1u] = b64idx((a & 3) << 4u | (b >> 4u));
    buf[2u] = b64idx((b & 15) << 2u | (c >> 6u));
    buf[3u] = b64idx(c & 63);
    if (i + 1 >= n) buf[2u] = '=';
    if (i + 2 >= n) buf[3u] = '=';
    len += out->printer(out, buf, sizeof(buf));
  }
  return len;
}

static int16_t b64dec(const char *src, int16_t n, char *dst) {

  const char *end = src + n;
  int16_t len = 0;
  int16_t a, b, c, d;

  while (src + 3 < end) 
  {
    a = b64rev(src[0u]);
    b = b64rev(src[1u]);
    c = b64rev(src[2u]);
    d = b64rev(src[3u]);
    dst[len++] = (a << 2u) | (b >> 4u);
    if (src[2u] != '=')
    {
      dst[len++] = (b << 4u) | (c >> 2u);
      if (src[3u] != '=')
      {
        dst[len++] = (c << 6u) | d;
      }
    }
    src += 4;
  }
  return len;
}

#endif /* JSON_ENABLE_BASE64 */
/*-----------------------------------------------------------------------------
 *      hexdec :
 *
 *  Parameters: const char *s
 *
 *  Return:
 *        char stream = hex code representing the int
 *----------------------------------------------------------------------------*/
static unsigned char hexdec(const char *s) 
{

#define HEXTOI(x) (x >= '0' && x <= '9' ? x - '0' : x - 'W')

  int16_t a = tolower(*(const unsigned char *) s);
  int16_t b = tolower(*(const unsigned char *) (s + 1));
  return (HEXTOI(a) << 4u) | HEXTOI(b);
}

#if !(MCU_NAME == PIC32MX795F512L)
int16_t json_vprintf(struct json_out *out, const char *fmt, va_list xap) WEAK;
int16_t json_vprintf(struct json_out *out, const char *fmt, va_list xap) {

  int16_t len = 0;
  const char *quote = "\"", *null = "null";
  va_list ap;
  va_copy(ap, xap);
  char buf[21u];
  size_t skip;

  while (*fmt != '\0') 
  {
    if (strchr(":, \r\n\t[]{}\"", *fmt) != NULL) 
    {
      len += out->printer(out, fmt, 1);
      fmt++;
    } 
    else if (fmt[0u] == '%')
    {
      skip = 2;
      if (fmt[1] == 'l' && fmt[2] == 'l' && (fmt[3] == 'd' || fmt[3] == 'u')) 
      {
        int64_t val = va_arg(ap, int64_t);
        const char *fmt2 = fmt[3] == 'u' ? "%" UINT64_FMT : "%" INT64_FMT;
        snprintf(buf, sizeof(buf), fmt2, val);
        len += out->printer(out, buf, strlen(buf));
        skip += 2;
      } 
      else if (fmt[1] == 'z' && fmt[2] == 'u') 
      {
        size_t val = va_arg(ap, size_t);
        snprintf(buf, sizeof(buf), "%lu", (unsigned long) val);
        len += out->printer(out, buf, strlen(buf));
        skip += 1;
      }
      else if (fmt[1] == 'M') 
      {
        json_printf_callback_t f = va_arg(ap, json_printf_callback_t);
        len += f(out, &ap);
      } 
      else if (fmt[1] == 'B') 
      {
        int val = va_arg(ap, int);
        const char *str = val ? "true" : "false";
        len += out->printer(out, str, strlen(str));
      } 
      else if (fmt[1] == 'H') 
      {
#if defined(JSON_ENABLE_HEX)
        const char *hex = "0123456789abcdef";
        int i, n = va_arg(ap, int);
        const unsigned char *p = va_arg(ap, const unsigned char *);
        len += out->printer(out, quote, 1);

        for (i = 0; i < n; i++) {
          len += out->printer(out, &hex[(p[i] >> 4) & 0xf], 1);
          len += out->printer(out, &hex[p[i] & 0xf], 1);
        }
        len += out->printer(out, quote, 1);
#endif /* JSON_ENABLE_HEX */
      }
      else if (fmt[1] == 'V') 
      {
#if defined(JSON_ENABLE_BASE64)
        const unsigned char *p = va_arg(ap, const unsigned char *);
        int n = va_arg(ap, int);
        len += out->printer(out, quote, 1);
        len += b64enc(out, p, n);
        len += out->printer(out, quote, 1);
#endif /* JSON_ENABLE_BASE64 */
      } 
      else if (fmt[1] == 'Q' || (fmt[1] == '.' && fmt[2] == '*' && fmt[3] == 'Q'))
      {
        size_t l = 0;
        const char *p;

        if (fmt[1] == '.') {
          l = (size_t) va_arg(ap, int);
          skip += 2;
        }
        p = va_arg(ap, char *);

        if (p == NULL) {
          len += out->printer(out, null, 4);
        } else {
          if (fmt[1] == 'Q') {
            l = strlen(p);
          }
          len += out->printer(out, quote, 1);
          len += json_escape(out, p, l);
          len += out->printer(out, quote, 1);
        }
      } 
      else 
      {
        /*
         * we delegate printing to the system printf.
         * The goal here is to delegate all modifiers parsing to the system
         * printf, as you can see below we still have to parse the format
         * types.
         *
         * Currently, %s with strings longer than 20 chars will require
         * double-buffering (an auxiliary buffer will be allocated from heap).
         * TODO(dfrank): reimplement %s and %.*s in order to avoid that.
         */
        const char *end_of_format_specifier = "sdfFeEgGlhuIcx.*-0123456789";
        int n = strspn(fmt + 1, end_of_format_specifier);
        char *pbuf = buf;
        int need_len, size = sizeof(buf);
        char fmt2[20];
        va_list ap_copy;
        strncpy(fmt2, fmt,
                n + 1 > (int) sizeof(fmt2) ? sizeof(fmt2) : (size_t) n + 1);
        fmt2[n + 1] = '\0';
        va_copy(ap_copy, ap);
        need_len = vsnprintf(pbuf, size, fmt2, ap_copy);
        va_end(ap_copy);

        if (need_len < 0) 
        {
          /*
           * Windows & eCos vsnprintf implementation return -1 on overflow
           * instead of needed size.
           */
          pbuf = NULL;
          while (need_len < 0) {
            free(pbuf);
            size *= 2;
            if ((pbuf = (char *) malloc(size)) == NULL) break;
            va_copy(ap_copy, ap);
            need_len = vsnprintf(pbuf, size, fmt2, ap_copy);
            va_end(ap_copy);
          }
        } 
        else if (need_len >= (int) sizeof(buf)) 
        {
          /*
           * resulting string doesn't fit into a stack-allocated buffer `buf`,
           * so we need to allocate a new buffer from heap and use it
           */
          if ((pbuf = (char *) malloc(need_len + 1)) != NULL) 
          {
            va_copy(ap_copy, ap);
            vsnprintf(pbuf, need_len + 1, fmt2, ap_copy);
            va_end(ap_copy);
          }
        }

        if (pbuf == NULL) 
        {
          buf[0] = '\0';
          pbuf = buf;
        }

        /*
         * however we need to parse the type ourselves in order to advance
         * the va_list by the correct amount; there is no portable way to
         * inherit the advancement made by vprintf.
         * 32-bit (linux or windows) passes va_list by value.
         */
        if ((n + 1 == (int) strlen("%" PRId64) && strcmp(fmt2, "%" PRId64) == 0) || (n + 1 == (int) strlen("%" PRIu64) && strcmp(fmt2, "%" PRIu64) == 0)) 
        {
          (void) va_arg(ap, int64_t);
        } 
        else if (strcmp(fmt2, "%.*s") == 0) 
        {
          (void) va_arg(ap, int);
          (void) va_arg(ap, char *);
        } else 
        {
          switch (fmt2[n]) 
          {
            case 'u':
            case 'd':
              (void) va_arg(ap, int);
              break;
            case 'g':
            case 'f':
              (void) va_arg(ap, double);
              break;
            case 'p':
              (void) va_arg(ap, void *);
              break;
            default:
              /* many types are promoted to int */
              (void) va_arg(ap, int);
          }
        }

        len += out->printer(out, pbuf, strlen(pbuf));
        skip = n + 1;

        /* If buffer was allocated from heap, free it */
        if (pbuf != buf) 
        {
          free(pbuf);
          pbuf = NULL;
        }
      }
      fmt += skip;
    } 
    else if (*fmt == '_' || json_isalpha(*fmt)) 
    {
      len += out->printer(out, quote, 1);
      while (*fmt == '_' || json_isalpha(*fmt) || json_isdigit(*fmt)) 
      {
        len += out->printer(out, fmt, 1);
        fmt++;
      }
      len += out->printer(out, quote, 1);
    } 
    else 
    {
      len += out->printer(out, fmt, 1);
      fmt++;
    }
  }
  va_end(ap);

  return len;
}
#else
/*-----------------------------------------------------------------------------
 *      json_vprintf :
 *
 *  Parameters: struct json_out *out, const char *fmt, va_list xap
 *
 *  Return:
 *        int
 *----------------------------------------------------------------------------*/
int16_t json_vprintf(struct json_out *out, const char *fmt, va_list xap ) WEAK;
int16_t json_vprintf(struct json_out *out, const char *fmt, va_list xap ) 
{
  int16_t len = 0;
  const char *quote = "\"", *null = "null";
  va_list ap;
  va_list ap_copy;
  char *str = NULL;
  // va_start(ap,xap);                                                          // issue va start with the elipsis  use xap ?
//  char *pbuf;                                                                 // not currently used

  memcpy((void*)ap,(void*)xap,sizeof(va_list));                                 // instead of va_copy
  //va_copy(ap, xap);
  while (*fmt != '\0') {
    if (strchr(":, \r\n\t[]{}\"", *fmt) != NULL) {
      len += out->printer(out, fmt, 1);
      fmt++;
    } else if (fmt[0] == '%') {
      char buf[21];
      size_t skip = 2;

      if (fmt[1] == 'l' && fmt[2] == 'l' && (fmt[3] == 'd' || fmt[3] == 'u')) {
        int64_t val = va_arg(ap, int64_t);
        const char *fmt2 = fmt[3] == 'u' ? "%" UINT64_FMT : "%" INT64_FMT;
        snprintf(buf, sizeof(buf), fmt2, val);
        len += out->printer(out, buf, strlen(buf));
        skip += 2;
      } else if (fmt[1] == 'z' && fmt[2] == 'u') {
        size_t val = va_arg(ap, size_t);
        snprintf(buf, sizeof(buf), "%lu", (unsigned long) val);
        len += out->printer(out, buf, strlen(buf));
        skip += 1;
      } else if (fmt[1] == 'M') {
        json_printf_callback_t f = va_arg(ap, json_printf_callback_t);
        len += f(out, &ap);
      } else if (fmt[1] == 'B') {
        int val = va_arg(ap, int);
        if (val==1)
          strcpy(str,"true");
        else
          strcpy(str,"false");
        //const char *str = val ? "true" : "false";
        len += out->printer(out, str, strlen(str));
      } else if (fmt[1] == 'H') {
#if defined(JSON_ENABLE_HEX)
        const char *hex = "0123456789abcdef";
        int i, n = va_arg(ap, int);
        const unsigned char *p = va_arg(ap, const unsigned char *);
        len += out->printer(out, quote, 1);
        for (i = 0; i < n; i++) {
          len += out->printer(out, &hex[(p[i] >> 4) & 0xf], 1);
          len += out->printer(out, &hex[p[i] & 0xf], 1);
        }
        len += out->printer(out, quote, 1);
#endif /* JSON_ENABLE_HEX */
      } else if (fmt[1] == 'V') {
#if defined(JSON_ENABLE_BASE64)
        const unsigned char *p = va_arg(ap, const unsigned char *);
        int n = va_arg(ap, int);
        len += out->printer(out, quote, 1);
        len += b64enc(out, p, n);
        len += out->printer(out, quote, 1);
#endif /* JSON_ENABLE_BASE64 */
      } 
      else if (fmt[1] == 'Q' ||
                 (fmt[1] == '.' && fmt[2] == '*' && fmt[3] == 'Q')) 
      {
        size_t l = 0;
        const char *p;

        if (fmt[1] == '.') 
        {
          l = (size_t) va_arg(ap, int);
          skip += 2;
        }

        p = va_arg(ap, char *);

        if (p == NULL) {
          len += out->printer(out, null, 4);
        } 
        else 
        {
          if (fmt[1] == 'Q') 
          {
            l = strlen((char*) p);
          }
          len += out->printer(out, quote, 1);
          len += json_escape(out, p, l);
          len += out->printer(out, quote, 1);
        }
      }
      else
      {
        /*
         * we delegate printing to the system printf.
         * The goal here is to delegate all modifiers parsing to the system
         * printf, as you can see below we still have to parse the format
         * types.
         *
         * Currently, %s with strings longer than 20 chars will require
         * double-buffering (an auxiliary buffer will be allocated from heap).
         * TODO(dfrank): reimplement %s and %.*s in order to avoid that.
         */
        const char *end_of_format_specifier = "sdfFeEgGlhuIcx.*-0123456789";
        int n = strspn((char*) (fmt + 1), (char*) end_of_format_specifier);
        char *pbuf = buf;
        int need_len, size = sizeof(buf);
        char fmt2[20];
        // va_start(ap_copy);
        // va_copy(ap_copy, ap);
        memcpy((void*)ap_copy,(void*)ap,sizeof(va_list));                       // instead of va_copy
        // va_list ap_copy;  not used any more
        strncpy((char *) fmt2,(char *) fmt, (int) (n + 1 > (int) sizeof(fmt2) ? sizeof(fmt2) : (size_t) n + 1));
        fmt2[n + 1] = '\0';

        // va_copy(ap_copy, ap);
        need_len = vsnprintf(pbuf, size, fmt2, ap_copy);
       // va_end(xap);

        if (need_len < 0)
        {
          /*
           * Windows & eCos vsnprintf implementation return -1 on overflow
           * instead of needed size.
           */
          pbuf = NULL;
          while (need_len < 0)
          {
            Free(pbuf,sizeof(pbuf));
            size *= 2;
            if ((pbuf = (char *) Malloc(size)) == NULL) break;
//            va_copy(xap, ap);
            memcpy((void*)ap_copy,(void*)ap,sizeof(va_list));                   // instead of va_copy
            need_len = vsnprintf(pbuf, size, fmt2, ap_copy);
//            va_end(xap);
          }
        }
        else if (need_len >= (int) sizeof(buf))
        {
          /*
           * resulting string doesn't fit into a stack-allocated buffer `buf`,
           * so we need to allocate a new buffer from heap and use it
           */
          if ((pbuf = (char *) Malloc(need_len + 1)) != NULL) {
              memcpy((void*)ap_copy,(void*)ap,sizeof(va_list));                 // instead of va_copy
              vsnprintf(pbuf, need_len + 1, fmt2, ap_copy);
//            va_end(ap);
          }
        }
        if (pbuf == NULL) {
          buf[0] = '\0';
          pbuf = buf;
        }
        /*
         * however we need to parse the type ourselves in order to advance
         * the va_list by the correct amount; there is no portable way to
         * inherit the advancement made by vprintf.
         * 32-bit (linux or windows) passes va_list by value.
         */
        if ((n + 1 == (int) strlen("%" PRId64) &&
             strcmp(fmt2, "%" PRId64) == 0) ||
            (n + 1 == (int) strlen("%" PRIu64) &&
             strcmp(fmt2, "%" PRIu64) == 0)) {
          (void) va_arg(ap, int64_t);
        } else if (strcmp(fmt2, "%.*s") == 0) {
          (void) va_arg(ap, int);
          (void) va_arg(ap, char *);
        } else {
          switch (fmt2[n]) {
            case 'u':

            case 'd':
              (void) va_arg(ap, int);
              break;

            case 'g':

            case 'f':
              (void) va_arg(ap, double);
              break;

            case 'p':
              (void) va_arg(ap, void *);
              break;

            default:
              /* many types are promoted to int */
              (void) va_arg(ap, int);
          }
        }

        len += out->printer(out, pbuf, strlen(pbuf));
        skip = n + 1;

        /* If buffer was allocated from heap, free it */
       if (pbuf != buf) {
#if (MCU_NAME == PIC32MX795F512L)
          Free(pbuf,sizeof(pbuf));
#else
          free(pbuf);
#endif
          pbuf = NULL;
        }
      }

      fmt += skip;

    } else if (*fmt == '_' || json_isalpha(*fmt)) {
      len += out->printer(out, quote, 1);
      while (*fmt == '_' || json_isalpha(*fmt) || json_isdigit(*fmt)) {
        len += out->printer(out, fmt, 1);
        fmt++;
      }

      len += out->printer(out, quote, 1);
    } else {
      len += out->printer(out, fmt, 1);
      fmt++;
    }
  }
//  va_end(ap);
  return len;
}
#endif

/*-----------------------------------------------------------------------------
 *      json_printf :
 *
 *  Parameters: struct json_out *out, const char *fmt, ...
 *
 *  Return:
 *        int number of chars written to out
 *----------------------------------------------------------------------------*/
int16_t json_printf(struct json_out *out, const char *fmt, ...) WEAK;
int16_t json_printf(struct json_out *out, const char *fmt, ...) 
{
  int16_t n;
  va_list ap;
  va_start(ap, fmt);
  n = json_vprintf(out, fmt, ap);
//  va_end(ap);
  return n;
}

/*-----------------------------------------------------------------------------
 *      json_printf_array :
 *
 *  Parameters: struct json_out *out, va_list *ap
 *
 *  Return:
 *        int len = array length
 *----------------------------------------------------------------------------*/
int16_t json_printf_array(struct json_out *out, va_list *ap) WEAK;
int16_t json_printf_array(struct json_out *out, va_list ap) {
  int16_t len = 0;
  char *arr;
  size_t i, arr_size;
  size_t elem_size;
  const char *fmt;
  union
  {
    int64_t i;
    double d;
  } val;
    
#ifdef WEAK
  arr = va_arg(ap, char *);
  arr_size = va_arg(ap, size_t);
  elem_size = va_arg(ap, size_t);
  fmt = va_arg(ap, char *);
#else
  arr = va_arg(*ap, char *);
  arr_size = va_arg(*ap, size_t);
  elem_size = va_arg(*ap, size_t);
  fmt = va_arg(*ap, char *);
#endif

  len += json_printf(out, "[", 1);

  for (i = 0; arr != NULL && i < arr_size / elem_size; i++) 
  {
    memcpy(&val, arr + i * elem_size, elem_size > sizeof(val) ? sizeof(val) : elem_size);
    if (i > 0) len += json_printf(out, ", ");
#if (MCU_NAME == PIC32MX795F512L)
    if (strpbrk((char*) fmt,(char*) "efg") != NULL) 
    {
#else
    if (strpbrk(fmt, "efg") != NULL)
    {
#endif
        len += json_printf(out, fmt, val.d);
     }
     else
     {
          len += json_printf(out, fmt, val.i);
     }
   }
   len += json_printf(out, "]", 1);
   return len;
}

#ifdef _WIN32 
#if !(MCU_NAME == PIC32MX795F512L)
int16_t cs_win_vsnprintf(char *str, size_t size, const char *format, va_list ap) WEAK;
int16_t cs_win_vsnprintf(char *str, size_t size, const char *format, va_list ap)
{
  int16_t res = _vsnprintf(str, size, format, ap);
  va_end(ap);

  if (res >= size) {
    str[size - 1] = '\0';
  }

  return res;
}

int16_t cs_win_snprintf(char *str, size_t size, const char *format, ...) WEAK;
int16_t cs_win_snprintf(char *str, size_t size, const char *format, ...)
{
  int res;

  va_list ap;
  va_start(ap, format);
  res = vsnprintf(str, size, format, ap);

//  va_end(ap);
  return res;
}
#endif /* MCU_NAME */
#endif /* _WIN32 */

/*-----------------------------------------------------------------------------
 *      json_walk :
 *
 *  Parameters: const char *json_string, int json_string_length,
 *              json_walk_callback_t callback, void *callback_data
 *  Return:
 *        int = frozen.cur - json_string (size)
 *----------------------------------------------------------------------------*/
int16_t json_walk(const char *json_string, int16_t json_string_length, json_walk_callback_t callback, void *callback_data) WEAK;

int16_t json_walk(const char *json_string, int json_string_length, json_walk_callback_t callback, void *callback_data) 
{
  struct frozen frozen;
  memset(&frozen, 0, sizeof(frozen));
  frozen.end = json_string + json_string_length;
  frozen.cur = json_string;
  frozen.callback_data = callback_data;
  frozen.callback = callback;

  TRY(json_doit(&frozen));
  return frozen.cur - json_string;
}

struct scan_array_info 
{
  int16_t found;
  char path[JSON_MAX_PATH_LEN];
  struct json_token *token;
};

/*-----------------------------------------------------------------------------
 *      json_scanf_array_elem_cb :
 *
 *  Parameters: void *callback_data, const char *name, size_t name_len, const char *path,
 *              const struct json_token *token
 *  Return:
 *        nothing
 *----------------------------------------------------------------------------*/
static void json_scanf_array_elem_cb(void *callback_data, const char *name, size_t name_len, const char *path, const struct json_token *token) 
{
  struct scan_array_info *info = (struct scan_array_info *) callback_data;
  (void) name;
  (void) name_len;
#if (MCU_NAME == PIC32MX795F512L)
  if (strcmp((char*) path,(char*) info->path) == 0) 
  {
#else
  if (strcmp(path, info->path) == 0) 
  {
#endif
     *info->token = *token;
     info->found = 1;
  }
}

/*-----------------------------------------------------------------------------
 *      json_scanf_array_elem :
 *
 *  Parameters: const char *s, int len, const char *path, int idx,
 *              struct json_token *token
 *  Return:
 *       int : token length or -1 if not found
 *----------------------------------------------------------------------------*/
int16_t json_scanf_array_elem(const char *s, int16_t len, const char *path, int16_t idx, struct json_token *token) WEAK;
int16_t json_scanf_array_elem(const char *s, int16_t len, const char *path, int16_t idx, struct json_token *token) 
{
  struct scan_array_info info;
  info.token = token;
  info.found = 0;
  memset(token, 0, sizeof(*token));
  snprintf(info.path, sizeof(info.path), "%s[%d]", path, idx);
  json_walk(s, len, json_scanf_array_elem_cb, &info);
  return info.found ? token->len : -1;
}

//#if MCU_NAME == PIUC32MX795F512L
//typedef struct {
//  int num_conversions;
//  char *path;
//  const char *fmt;
//  void *target;
//  void *user_data;
//  int type;
//} json_scanf_info;
//#else
struct json_scanf_info 
{
  int16_t num_conversions;
  char *path;
  const char *fmt;
  void *target;
  void *user_data;
  int16_t type;
};
//#endif

/*-----------------------------------------------------------------------------
 *      json_unescape :
 *
 *  Parameters: const char *src, int slen, char *dst, int dlen
 *
 *  Return:
 *       int : dst - orig_dst (length) or JSON_STRING_INCOMPLETE or 
 *                   JSON_STRING_INVALID
 *----------------------------------------------------------------------------*/
int16_t json_unescape(const char *src, int16_t slen, char *dst, int16_t dlen) WEAK;
int16_t json_unescape(const char *src, int16_t slen, char *dst, int16_t dlen) 
{
  char *send = (char *) src + slen, *dend = dst + dlen, *orig_dst = dst, *p;
  const char *esc1 = "\"\\/bfnrt", *esc2 = "\"\\/\b\f\n\r\t";

  while (src < send) {
    if (*src == '\\') {
      if (++src >= send) return JSON_STRING_INCOMPLETE;
      if (*src == 'u') {
#if (MCU_NAME == PIC32MX795F512L)
        if (((uint8_t)send) - ((uint8_t)src) < 5U) return JSON_STRING_INCOMPLETE;
#else
        if (send - src < 5) return JSON_STRING_INCOMPLETE;
#endif
        /* Here we go: this is a \u.... escape. Process simple one-byte chars */
        if (src[1] == '0' && src[2] == '0') {
          if (dst < dend) *dst = hexdec(src + 3);                               /* This is \u00xx character from the ASCII range */
          src += 4;
        } else {
          return JSON_STRING_INVALID;                                           /* Complex \uXXXX escapes drag utf8 lib... Do it at some stage */
        }
#if (MCU_NAME == PIC32MX795F512L)
      } else if ((p = (char *) strchr((char*)esc1,(char) *src)) != NULL) {
        if (dst < dend) *dst = esc2[(((uint8_t)p) - ((uint8_t)esc1))];
#else
      } else if ((p = (char *) strchr(esc1, *src)) != NULL) {
        if (dst < dend) *dst = esc2[p - esc1];
#endif
      } else {
        return JSON_STRING_INVALID;
      }
    } else {
      if (dst < dend) *dst = *src;
    }

    dst++;
    src++;
  }
  return dst - orig_dst;
}

/*-----------------------------------------------------------------------------
 *      json_scanf_cb :
 *
 *  Parameters: void *callback_data, const char *name, size_t name_len, const char *path,
 *              const struct json_token *token
 *  Return:
 *       nothing
 *
 *----------------------------------------------------------------------------*/
static void json_scanf_cb(void *callback_data, const char *name, size_t name_len, const char *path, const struct json_token *token)
{
  struct json_scanf_info *info = (struct json_scanf_info *) callback_data;

  char buf[32u];                                                                /* Must be enough to hold numbers */
  (void) name;
  (void) name_len;
  //info->num_conversions=0U;                                                      initialise at zero

  if (token->ptr == NULL) 
  {
    /*
     * We're not interested here in the events for which we have no value;
     * namely, JSON_TYPE_OBJECT_START and JSON_TYPE_ARRAY_START
     */
    return;
  }
  
#if (MCU_NAME == PIC32MX795F512L)
  if (strcmp((char*)path,(char*)info->path) != 0) 
  {
#else
  if (strcmp(path, info->path) != 0) 
  {
#endif
     return;                                                                    /* It's not the path we're looking for, so, just ignore this callback */
  }

  switch (info->type) 
  {
    case 'B':
      info->num_conversions=++info->num_conversions % INT16_MAX;
#if (MCU_NAME == PIC32MX795F512L)
      switch (sizeof(uint8_t))
      {
        case sizeof(char):
          *(char *) info->target = (token->type == JSON_TYPE_TRUE ? 1 : 0);
          break;

        case sizeof(int):
          *(int *) info->target = (token->type == JSON_TYPE_TRUE ? 1 : 0);
          break;

        case sizeof(uint8_t):
          *(uint8_t *) info->target = (token->type == JSON_TYPE_TRUE ? 1 : 0);
          break;
          
        default:
        /* should never be here */
        return;
      }
#else
      switch (sizeof(bool)) 
      {
        case sizeof(char):
          *(char *) info->target = (token->type == JSON_TYPE_TRUE ? 1 : 0);
          break;

        case sizeof(int):
          *(int *) info->target = (token->type == JSON_TYPE_TRUE ? 1 : 0);
          break;

        default:
          /* should never be here */
          // abort();
          break;
      }
#endif
      break;

    case 'M': {
#if (MCU_NAME == PIC32MX795F512L)
      typedef union {
        void *p;
        json_scanner_t f;
      } Mtype_t;
      Mtype_t u;
      // u = {info->target};
#else
      union {
        void *p;
        json_scanner_t f;
      } u = {info->target};
#endif
    info->num_conversions++;
#if (MCU_NAME == PIC32MX795F512L)
     u.f.str = token->ptr;
     u.f.len = token->len;
     u.f.user_data = info->user_data;
#else
      u.f(token->ptr, token->len, info->user_data);
#endif
      break;

    }

    case 'Q': {
      char **dst = (char **) info->target;
      if (token->type == JSON_TYPE_NULL) {
        *dst = NULL;
      } else {
        int unescaped_len = json_unescape(token->ptr, token->len, NULL, 0);
        if (unescaped_len >= 0 &&
            (*dst = (char *) Malloc(unescaped_len + 1)) != NULL) {
          info->num_conversions++;
          if (json_unescape(token->ptr, token->len, *dst, unescaped_len) ==
              unescaped_len) {
            (*dst)[unescaped_len] = '\0';
          } else {
#if (MCU_NAME == PIC32MX795F512L)
            Free(*dst,sizeof(*dst));
#else
            free(*dst);
#endif
            *dst = NULL;
          }
        }
      }
      break;

    }

    case 'H': {

#if defined(JSON_ENABLE_HEX)

      char **dst = (char **) info->user_data;
      int i, len = token->len / 2;
      *(int *) info->target = len;
      if ((*dst = (char *) Malloc(len + 1)) != NULL) {
        for (i = 0; i < len; i++) {
          (*dst)[i] = hexdec(token->ptr + 2 * i);
        }

        (*dst)[len] = '\0';
        info->num_conversions=++info->num_conversions % INT16_MAX;
      }
#endif /* JSON_ENABLE_HEX */
      break;
    }

    case 'V': {
#if defined(JSON_ENABLE_BASE64)
      char **dst = (char **) info->target;
      int len = token->len * 4 / 3 + 2;
      if ((*dst = (char *) Malloc(len + 1)) != NULL) {
        int n = b64dec(token->ptr, token->len, *dst);
        (*dst)[n] = '\0';
        *(int *) info->user_data = n;
        info->num_conversions++;
      }
#endif /* JSON_ENABLE_BASE64 */
      break;
    }

    case 'T':
      info->num_conversions++;
      *(struct json_token *) info->target = *token;
      break;

    default:
      if (token->len >= (int) sizeof(buf)) break;
      /* Before converting, copy into tmp buffer in order to 0-terminate it */
#if (MCU_NAME == PIC32MX795F512L)
      memcpy((void*) buf,(void*) token->ptr, token->len);
#else
      memcpy(buf, token->ptr, token->len);
#endif
      buf[token->len] = '\0';
      /* NB: Use of base 0 for %d, %ld, %u and %lu is intentional. */
      if (info->fmt[1] == 'd' || (info->fmt[1] == 'l' && info->fmt[2] == 'd') ||
          info->fmt[1] == 'i') {
        char *endptr = NULL;
        int32_t r = strtol(buf, &endptr, 0 /* base */);
        if (*endptr == '\0') {
          if (info->fmt[1] == 'l') {
            *((int32_t *) info->target) = r;
          } else {
            *((int *) info->target) = (int) r;
          }
          info->num_conversions++;
        }
      } else if (info->fmt[1] == 'u' ||
                 (info->fmt[1] == 'l' && info->fmt[2] == 'u')) {

        char *endptr = NULL;
        uint32_t r = strtoul(buf, &endptr, 0 /* base */);
        if (*endptr == '\0') {
          if (info->fmt[1] == 'l') {
            *((uint32_t *) info->target) = r;
          } else {
            *((uint32_t *) info->target) = (uint32_t) r;
          }
          info->num_conversions++;
        }
      } else {
#if !(MCU_NAME == PIC32MX795F512L)
#if !JSON_MINIMAL
      info->num_conversions += sscanf(buf, info->fmt, info->target);            // $$$ consider re-writing for PIC32 no sscanf function exists
#endif
#endif
      }
      break;
  }
}

/*-----------------------------------------------------------------------------
 *      json_vscanf :
 *
 *  Parameters: const char *s, int len, const char *fmt, va_list ap
 *
 *  Return:
 *       int info.num_conversions
 *
 *----------------------------------------------------------------------------*/
int16_t json_vscanf(const char *s, int16_t len, const char *fmt, va_list ap) WEAK;
int16_t json_vscanf(const char *s, int16_t len, const char *fmt, va_list ap) 
{
  char path[JSON_MAX_PATH_LEN] = "", fmtbuf[20u];
  int16_t i = 0;
  char *p = NULL;
#if (MCU_NAME == PIC32MX795F512L)
  struct json_scanf_info info;
  info.num_conversions = 0U;
  info.path = path;
  info.fmt = fmtbuf;
  info.target = NULL;
  info.user_data = NULL;
  info.type = 0U;
#else
  struct json_scanf_info info = {0, path, fmtbuf, NULL, NULL, 0};
#endif

  while (fmt[i] != '\0') 
  {
    if (fmt[i] == '{') 
    {
      strcat(path, ".");
      i++;
    } 
    else if (fmt[i] == '}') 
    {
      if ((p = strrchr(path, '.')) != NULL) *p = '\0';
      i++;
    } 
    else if (fmt[i] == '%') 
    {
      info.target = va_arg(ap, void *);
      info.type = fmt[i + 1];
      switch (fmt[i + 1]) 
      {
        case 'M':

        case 'V':

        case 'H':
          info.user_data = va_arg(ap, void *);
        /* FALLTHROUGH */

        case 'B':

        case 'Q':

        case 'T':
          i += 2;
          break;

        default: 
        {
          const char *delims = ", \t\r\n]}";
#if (MCU_NAME == PIC32MX795F512L)
          int16_t conv_len = (int16_t) strcspn((char*)fmt + i + 1,(char*) delims) + 1;
          memcpy((void*)fmtbuf,(void*) fmt + i, conv_len);
#else
          int16_t conv_len = strcspn(fmt + i + 1, delims) + 1;
          memcpy(fmtbuf, fmt + i, conv_len);
#endif
          fmtbuf[conv_len] = '\0';
          i += conv_len;
#if (MCU_NAME == PIC32MX795F512L)
          i += strspn((char*)fmt + i,(char*) delims);
#else
          i += strspn(fmt + i, delims);
#endif
          break;
        }
      }

      json_walk(s, len, json_scanf_cb, &info);
    } 
    else if (json_isalpha(fmt[i]) || json_get_utf8_char_len(fmt[i]) > 1) 
    {
      char *pe;
      const char *delims = ": \r\n\t";
#if (MCU_NAME == PIC32MX795F512L)
     int16_t key_len = (int16_t) strcspn((char*) &fmt[i],(char*) delims);
#else
     int16_t key_len = strcspn(&fmt[i], delims);
#endif
      if ((p = strrchr(path, '.')) != NULL) p[1] = '\0';
      pe = path + strlen(path);
#if (MCU_NAME == PIC32MX795F512L)
      memcpy((void *)pe,(void *) fmt + i, key_len);
#else
      memcpy(pe,fmt + i, key_len);
#endif
      pe[key_len] = '\0';
#if (MCU_NAME == PIC32MX795F512L)
      i += key_len + (int) strspn((char*)fmt + i + key_len,(char*) delims);
#else
      i += key_len + strspn((char*)fmt + i + key_len,delims);
#endif
    } 
    else 
    {
      i=++i % INT16_MAX;
    }
  }
  return info.num_conversions;
}
/*******************************************************************************
* Function Name: json_scanf
********************************************************************************
* Summary:
* scanf for JSON string
* Parameters:
* const char *str, int len, const char *fmt, ...
* Return:
*  int
*
*******************************************************************************/
int16_t json_scanf(const char *str, int len, const char *fmt, ...) WEAK;
int16_t json_scanf(const char *str, int len, const char *fmt, ...) 
{
  int16_t result;
  va_list ap;
  va_start(ap, fmt);

  result = json_vscanf(str, len, fmt, ap);
#if !(MCU_NAME == PIC32MX795F512L)
  va_end(ap);
#endif
  return result;
}
/*******************************************************************************
* Function Name: json_vfprintf
********************************************************************************
* Summary:
* fprintf for JSON string  (N/A to PIC32)
* Parameters:
* const char *file_name, const char *fmt, va_list ap
* Return:
*  int
*
*******************************************************************************/
#if !(MCU_NAME == PIC32MX795F512L)                                              // no files in PIC32
int16_t json_vfprintf(const char *file_name, const char *fmt, va_list ap) WEAK;
int16_t json_vfprintf(const char *file_name, const char *fmt, va_list ap)
{
  int16_t res = -1;

  FILE *fp = fopen(file_name, "wb");
  if (fp != NULL) {
    struct json_out out = JSON_OUT_FILE(fp);
    res = json_vprintf(&out, fmt, ap);
    fputc('\n', fp);
    fclose(fp);
  }
  return res;

}
/*******************************************************************************
* Function Name: json_fprintf
********************************************************************************
* Summary:
* fprintf for JSON string  (N/A to PIC32)
* Parameters:
* const char *file_name, const char *fmt, ...
* Return:
*  int
*
*******************************************************************************/
int16_t json_fprintf(const char *file_name, const char *fmt, ...) WEAK;
int16_t json_fprintf(const char *file_name, const char *fmt, ...) 
{
  int16_t result;
  va_list ap;
  va_start(ap, fmt);

  result = json_vfprintf(file_name, fmt, ap);
  va_end(ap);
  return result;
}
/*******************************************************************************
* Function Name: json_fread
********************************************************************************
* Summary:
* fread for JSON string  (N/A to PIC32)
* Parameters:
* const char *path
* Return:
*  char *
*
*******************************************************************************/
char *json_fread(const char *path) WEAK;
char *json_fread(const char *path) 
{
  FILE *fp;
  char *data = NULL;

  if ((fp = fopen(path, "rb")) == NULL) 
  {
  } 
  else if (fseek(fp, 0, SEEK_END) != 0) 
  {
    fclose(fp);
  } 
  else 
  {
    int32_t size = ftell(fp);
    if (size > 0 && (data = (char *) malloc(size + 1)) != NULL) 
    {
      fseek(fp, 0, SEEK_SET);                                                   /* Some platforms might not have rewind(), Oo */
      if (fread(data, 1, size, fp) != (size_t) size) 
      {
        free(data);
        data = NULL;
      } 
      else 
      {
        data[size] = '\0';
      }
    }
    fclose(fp);
  }
  return data;

}
#endif

struct json_setf_data 
{
  const char *json_path;
  const char *base;                                                             /* Pointer to the source JSON string */
  int16_t matched;                                                              /* Matched part of json_path */
  int16_t pos;                                                                  /* Offset of the mutated value begin */
  int16_t end;                                                                  /* Offset of the mutated value end */
  int16_t prev;                                                                 /* Offset of the previous token end */
};

/*-----------------------------------------------------------------------------
 *      get_matched_prefix_len :
 *
 *  Parameters: const char *s1, const char *s2
 *
 *  Return:
 *       int i=number of matched prefix
 *
 *----------------------------------------------------------------------------*/
static int16_t get_matched_prefix_len(const char *s1, const char *s2) 
{
  int16_t i = 0;
  while (s1[i] && s2[i] && s1[i] == s2[i]) i++;
  return i;

}

/*-----------------------------------------------------------------------------
 *      json_vsetf_cb :
 *
 *  Parameters: void *userdata, const char *name, size_t name_len,
 *              const char *path, const struct json_token *t
 *  Return:
 *       nothing
 *
 *----------------------------------------------------------------------------*/
static void json_vsetf_cb(void *userdata, const char *name, size_t name_len, const char *path, const struct json_token *t) 
{

  int16_t off1;
  int16_t len;
  
//#if (MCU_NAME == PIC32MX795F512L)
//  struct json_setf_data *dataV = NULL;
//  if (sizeof(userdata) <= sizeof(dataV))
//     memcpy((void *) dataV,userdata,sizeof(userdata));
//#else
  struct json_setf_data *dataV = (struct json_setf_data *) userdata;
//#endif

  len = get_matched_prefix_len(path, dataV->json_path);

  if (t->ptr == NULL) return;
  off1 = t->ptr - dataV->base;
  if (len > dataV->matched) dataV->matched = len;

  /*
   * If there is no exact path match, set the mutation position to tbe end
   * of the object or array
   */
  if (len < dataV->matched && dataV->pos == 0 &&
      (t->type == JSON_TYPE_OBJECT_END || t->type == JSON_TYPE_ARRAY_END)) {
    dataV->pos = dataV->end = dataV->prev;
  }

  /* Exact path match. Set mutation position to the value of this token */
#if (MCU_NAME == PIC32MX795F512L)
  if (strcmp((char*)path,(char*) dataV->json_path) == 0 && t->type != JSON_TYPE_OBJECT_START &&
#else
  if (strcmp(path, dataV->json_path) == 0 && t->type != JSON_TYPE_OBJECT_START &&
#endif
      t->type != JSON_TYPE_ARRAY_START) {
    dataV->pos = off1;
    dataV->end = off1 + t->len;
  }

  /*
   * For deletion, we need to know where the previous value ends, because
   * we don't know where matched value key starts.
   * When the mutation position is not yet set, remember each value end.
   * When the mutation position is already set, but it is at the beginning
   * of the object/array, we catch the end of the object/array and see
   * whether the object/array start is closer then previously stored prev.
   */
  if (dataV->pos == 0U) {
    dataV->prev = off1 + t->len;                                                /* pos is not yet set */
  } else if ((t->ptr[0U] == '[' || t->ptr[0U] == '{') && off1 + 1U < dataV->pos &&
             off1 + 1U > dataV->prev) {
    dataV->prev = off1 + 1U;
  }

  (void) name;
  (void) name_len;
}

/*-----------------------------------------------------------------------------
 *      json_vsetf :
 *
 *  Parameters: const char *s, int len, struct json_out *out1,
 *              const char *json_path, const char *json_fmt, va_list ap
 *  Return:
 *       int =1 if dataV.end > dataV.pos else 0
 *
 *----------------------------------------------------------------------------*/
int16_t json_vsetf(const char *s, int len, struct json_out *out1, const char *json_path, const char *json_fmt, va_list ap) WEAK;
int16_t json_vsetf(const char *s, int len, struct json_out *out1, const char *json_path, const char *json_fmt, va_list ap) 
{
  struct json_setf_data dataV;
  memset(&dataV, 0, sizeof(dataV));
  dataV.json_path = json_path;
  dataV.base = s;
  dataV.end = len;
  json_walk(s, len, json_vsetf_cb, &dataV);
  if (json_fmt == NULL) 
  {
    json_printf(out1, "%.*s", dataV.prev, s);                                   /* Deletion codepath */
    if (s[dataV.prev - 1] == '{' || s[dataV.prev - 1] == '[') 
    {                                                                           /* Trim comma after the value that begins at object/array start */
      int16_t i = dataV.end;
      while (i < len && json_isspace(s[i])) i++;
      if (s[i] == ',') dataV.end = i + 1;                                       /* Point after comma */
    }
    json_printf(out1, "%.*s", len - dataV.end, s + dataV.end);
  } 
  else 
  {
    int16_t n, off = dataV.matched, depth = 0;                                  /* Modification codepath */
    json_printf(out1, "%.*s", dataV.pos, s);                                    /* Print the unchanged beginning */
#if (MCU_NAME == PIC32MX795F512L)
    while ((n = (int) strcspn((char*)&json_path[off],(char*) ".[")) > 0)        /* Add missing keys */
    {
#else
    while ((n = strcspn(&json_path[off], ".[")) > 0)                            /* Add missing keys */
    {
#endif
      if (s[dataV.prev - 1] != '{' && s[dataV.prev - 1] != '[' && depth == 0) 
      {
        json_printf(out1, ",");
      }
      if (off > 0 && json_path[off - 1] != '.') break;
      json_printf(out1, "%.*Q:", n, json_path + off);
      off += n;
      if (json_path[off] != '\0') 
      {
        json_printf(out1, "%c", json_path[off] == '.' ? '{' : '[');
        depth++;
        off++;
      }
    }
    json_vprintf(out1, json_fmt, ap);                                           /* Print the new value */

    for (; off > dataV.matched; off--)                                          /* Close brackets/braces of the added missing keys */
    {
      int16_t ch = json_path[off];
      const char *p = ch == '.' ? "}" : ch == '[' ? "]" : "";
      json_printf(out1, "%s", p);
    }
    json_printf(out1, "%.*s", len - dataV.end, s + dataV.end);                  /* Print the rest of the unchanged string */
  }
  return dataV.end > dataV.pos ? 1 : 0;
}

int json_setf(const char *s, int len, struct json_out *out1, const char *json_path, const char *json_fmt, ...) WEAK;
int json_setf(const char *s, int len, struct json_out *out1, const char *json_path, const char *json_fmt, ...) 
{
  int16_t result;
  va_list ap;
  va_start(ap, json_fmt);
  result = json_vsetf(s, len, out1, json_path, json_fmt, ap);
#if !(MCU_NAME == PIC32MX795F512L)
  va_end(ap);
#endif
  return result;
}

struct prettify_data 
{
  struct json_out *out;
  int16_t level;
  int16_t last_token;
};

/*-----------------------------------------------------------------------------
 *      indent :
 *
 *  Parameters: struct json_out *out, int level
 *
 *  Return:
 *       nothing
 *
 *----------------------------------------------------------------------------*/
static void indent(struct json_out *out, int16_t level)
{
  while (level-- > 0) out->printer(out, "  ", 2);
}

/*-----------------------------------------------------------------------------
 *      print_key :
 *
 *  Parameters: struct prettify_data *pd, const char *path,
 *              const char *name, int name_len
 *  Return:
 *       nothing
 *
 *----------------------------------------------------------------------------*/
static void print_key(struct prettify_data *pd, const char *path, const char *name, int16_t name_len)
{
  if (pd->last_token != JSON_TYPE_INVALID && pd->last_token != JSON_TYPE_ARRAY_START && pd->last_token != JSON_TYPE_OBJECT_START) 
  {
    pd->out->printer(pd->out, ",", 1);
  }

  if (path[0] != '\0') pd->out->printer(pd->out, "\n", 1);
  indent(pd->out, pd->level);
#if (MCU_NAME == PIC32MX795F512L)
  if (path[0] != '\0' && path[strlen((char*)path) - 1] != ']') 
  {
#else
  if (path[0] != '\0' && path[strlen(path) - 1] != ']') 
  {
#endif
    pd->out->printer(pd->out, "\"", 1);
    pd->out->printer(pd->out, name, (int) name_len);
    pd->out->printer(pd->out, "\"", 1);
    pd->out->printer(pd->out, ": ", 2);
  }
}

/*-----------------------------------------------------------------------------
 *      prettify_cb :
 *
 *  Parameters: void *userdata, const char *name, size_t name_len,
 *              const char *path, const struct json_token *t
 *  Return:
 *       nothing
 *
 *----------------------------------------------------------------------------*/
static void prettify_cb(void *userdata, const char *name, size_t name_len, const char *path, const struct json_token *t) 
{
  struct prettify_data *pd = (struct prettify_data *) userdata;
  switch (t->type) 
  {
    case JSON_TYPE_OBJECT_START:
    case JSON_TYPE_ARRAY_START:
      print_key(pd, path, name, name_len);
      pd->out->printer(pd->out, t->type == JSON_TYPE_ARRAY_START ? "[" : "{",
                       1);
      pd->level++;
      break;

    case JSON_TYPE_OBJECT_END:

    case JSON_TYPE_ARRAY_END:
      pd->level--;
      if (pd->last_token != JSON_TYPE_INVALID && pd->last_token != JSON_TYPE_ARRAY_START && pd->last_token != JSON_TYPE_OBJECT_START) 
      {
        pd->out->printer(pd->out, "\n", 1);
        indent(pd->out, pd->level);
      }
      pd->out->printer(pd->out, t->type == JSON_TYPE_ARRAY_END ? "]" : "}", 1);
      break;

    case JSON_TYPE_NUMBER:

    case JSON_TYPE_NULL:

    case JSON_TYPE_TRUE:

    case JSON_TYPE_FALSE:

    case JSON_TYPE_STRING:
      print_key(pd, path, name, name_len);
      if (t->type == JSON_TYPE_STRING) pd->out->printer(pd->out, "\"", 1);
      pd->out->printer(pd->out, t->ptr, t->len);
      if (t->type == JSON_TYPE_STRING) pd->out->printer(pd->out, "\"", 1);
      break;

    default:
      break;
  }
  pd->last_token = t->type;
}

/*-----------------------------------------------------------------------------
 *      json_prettify :
 *
 *  Parameters: const char *s, int len, struct json_out *out
 *
 *  Return:
 *       int number of processed bytes or -ve error code
 *
 *----------------------------------------------------------------------------*/
int16_t json_prettify(const char *s, int16_t len, struct json_out *out) WEAK;
int16_t json_prettify(const char *s, int16_t len, struct json_out *out) 
{
#if !(MCU_NAME == PIC32MX795F512L)
  struct prettify_data pd = {out, 0U, JSON_TYPE_INVALID};
#else
  struct prettify_data pd;
  pd.out = out;
  pd.level = 0U;
  pd.last_token = JSON_TYPE_INVALID;
#endif
  return json_walk(s, len, prettify_cb, &pd);
}

#if !(MCU_NAME == PIC32MX795F512L)
int16_t json_prettify_file(const char *file_name) WEAK;
int16_t json_prettify_file(const char *file_name) 
{
  int16_t res = -1;
  char *s = json_fread(file_name);

  FILE *fp;
  if (s != NULL && (fp = fopen(file_name, "wb")) != NULL) 
  {
    struct json_out out = JSON_OUT_FILE(fp);
    res = json_prettify(s, strlen(s), &out);
    if (res < 0) 
    {
      /* On error, restore the old content */
      fclose(fp);
      fp = fopen(file_name, "wb");
      fseek(fp, 0, SEEK_SET);
      fwrite(s, 1, strlen(s), fp);
    } else {
      fputc('\n', fp);
    }
    fclose(fp);
  }
  free(s);
  return res;
}
#endif

struct next_data 
{
  void *handle;                                                                 // Passed handle. Changed if a next entry is found
  const char *path;                                                             // Path to the iterated object/array
  int16_t path_len;                                                             // Path length - optimisation
  int16_t found;                                                                // Non-0 if found the next entry
  struct json_token *key;                                                       // Object's key
  struct json_token *val;                                                       // Object's value
  int16_t *idx;                                                                 // Array index
};

/*******************************************************************************
* Function Name: next_set_key
********************************************************************************
* Summary:
* next set key for JSON string
* Parameters:
* struct next_data *d, const char *name, int name_len, int is_array
* Return:
*  nothing
*
*******************************************************************************/
static void next_set_key(struct next_data *d, const char *name, int16_t name_len, int16_t is_array) 
{
  if (is_array)
  {
    /* Array. Set index and reset key  */
    if (d->key != NULL) 
    {
      d->key->len = 0;
      d->key->ptr = NULL;
    }
#if (MCU_NAME == PIC32MX795F512L)
    if (d->idx != NULL) *d->idx = atoi((char *)name);
#else
    if (d->idx != NULL) *d->idx = atoi(name);
#endif
  } 
  else 
  {
    /* Object. Set key and make index -1 */
    if (d->key != NULL) 
    {
      d->key->ptr = name;
      d->key->len = name_len;
    }
    if (d->idx != NULL) *d->idx = -1;
  }
}
/*******************************************************************************
* Function Name: json_next_cb
********************************************************************************
* Summary:
* set next callback for JSON string
* Parameters:
* void *userdata, const char *name, size_t name_len, const char *path, const struct json_token *t
* Return:
*  nothing
*
*******************************************************************************/
static void json_next_cb(void *userdata, const char *name, size_t name_len, const char *path, const struct json_token *t) 
{
  struct next_data *d = (struct next_data *) userdata;
  const char *p = path + d->path_len;

  if (d->found) return;

#if (MCU_NAME == PIC32MX795F512L)
  if (d->path_len >= (int) strlen((char*)path)) return;
  if (strncmp((char*)d->path,(char *) path,(char) d->path_len) != 0) return;
  if (strchr((char*)p + 1,(char) '.') != NULL) return;                          /* More nested objects - skip */
  if (strchr((char*)p + 1,(char) '[') != NULL) return;                          /* Ditto for arrays */
#else
  if (d->path_len >= (int16_t) strlen(path)) return;
  if (strncmp(d->path,path,d->path_len) != 0) return;
  if (strchr(p + 1,'.') != NULL) return;                                        /* More nested objects - skip */
  if (strchr(p + 1,'[') != NULL) return;                                        /* Ditto for arrays */
#endif

  if (t->type == JSON_TYPE_OBJECT_START || t->type == JSON_TYPE_ARRAY_START)    // {OBJECT,ARRAY}_END types do not pass name, _START does. Save key.
  {
    next_set_key(d, name, name_len, p[0u] == '[');
  } 
  else if (d->handle == NULL || d->handle < (void *) t->ptr) 
  {
    if (t->type != JSON_TYPE_OBJECT_END && t->type != JSON_TYPE_ARRAY_END) 
    {
      next_set_key(d, name, name_len, p[0u] == '[');
    }
    if (d->val != NULL) *d->val = *t;
    d->handle = (void *) t->ptr;
    d->found = 1;
  }
}
/*******************************************************************************
* Function Name: json_next
********************************************************************************
* Summary:
* get next handle for JSON string or NULL if no more
* Parameters:
* const char *s, int len, void *handle, const char *path, struct json_token *key, struct json_token *val, int *i
* Return:
*  void *
*
*******************************************************************************/
static void *json_next(const char *s, int len, void *handle, const char *path, struct json_token *key, struct json_token *val, int16_t *i) 
{
  struct json_token tmpval, *v = val == NULL ? &tmpval : val;
  struct json_token tmpkey, *k = key == NULL ? &tmpkey : key;
  int16_t tmpidx, *pidx = i == NULL ? &tmpidx : i;

  struct next_data dataV;
#if !(MCU_NAME == PIC32MX795F512L)
  dataV = {handle, path, (int16_t) strlen(path), 0, k, v, pidx};
#else
  dataV.handle = handle;
  dataV.path = path;
  dataV.path_len = (int16_t) strlen((char*)path);
  dataV.found = 0;
  dataV.key = k;
  dataV.val = v;
  dataV.idx = pidx;
#endif
  json_walk(s, len, json_next_cb, &dataV);
  return dataV.found ? dataV.handle : NULL;
}
/*******************************************************************************
* Function Name: json_next_key
********************************************************************************
* Summary:
* get next key for JSON string or NULL if no more
* Parameters:
* const char *s, int len, void *handle, const char *path struct json_token *key, struct json_token *val
* Return:
*  void *
*
*******************************************************************************/
void *json_next_key(const char *s, int16_t len, void *handle, const char *path, struct json_token *key, struct json_token *val) WEAK;
void *json_next_key(const char *s, int16_t len, void *handle, const char *path, struct json_token *key, struct json_token *val)
{
  return json_next(s, len, handle, path, key, val, NULL);
}
/*******************************************************************************
* Function Name: json_next_elem
********************************************************************************
* Summary:
* get next element for JSON string or NULL if no more
* Parameters:
* const char *s, int len, void *handle, const char *path, int *idx, struct json_token *val
* Return:
*  void *
*
*******************************************************************************/
void *json_next_elem(const char *s, int16_t len, void *handle, const char *path, int16_t *idx, struct json_token *val) WEAK;
void *json_next_elem(const char *s, int16_t len, void *handle, const char *path, int16_t *idx, struct json_token *val) 
{
  return json_next(s, len, handle, path, NULL, val, idx);
}
/*******************************************************************************
* Function Name: json_sprinter
********************************************************************************
* Summary:
* sprintf for JSON
* Parameters:
* struct json_out *out, const char *str, size_t len
* Return:
*  int
*
*******************************************************************************/
static int16_t json_sprinter(struct json_out *out, const char *str, size_t len)
{
  size_t old_len = out->u.buf.buf == NULL ? 0 : strlen(out->u.buf.buf);
  size_t new_len = len + old_len;
#if !(MCU_NAME == PIC32MX795F512L)
  char *p = (char *) realloc(out->u.buf.buf, new_len + 1);
#else
  char *p = (char *) Malloc((unsigned long)out->u.buf.buf);                     // char *p = (char *) realloc(out->u.buf.buf, new_len + 1);
#endif
  if (p != NULL) 
  {
#if (MCU_NAME == PIC32MX795F512L)
    memcpy((void *) p + old_len,(void *) str,(int) len);
#else
    memcpy(p + old_len,str,len);
#endif
    p[new_len] = '\0';
    out->u.buf.buf = p;
  }
#if !(MCU_NAME == PIC32MX795F512L)
  free(p);
#else
  Free(p, sizeof(p));
#endif
  return len;
}
/*******************************************************************************
* Function Name: json_vasprintf
********************************************************************************
* Summary:
* printf for JSON
* Parameters:
* const char *fmt, va_list ap
* Return:
*  int
*
*******************************************************************************/
char *json_vasprintf(const char *fmt, va_list ap) WEAK;
char *json_vasprintf(const char *fmt, va_list ap) 
{
  struct json_out out;
  memset(&out, 0, sizeof(out));
  out.printer = json_sprinter;
  json_vprintf(&out, fmt, ap);
  return out.u.buf.buf;
}
/*******************************************************************************
* Function Name: json_asprintf
********************************************************************************
* Summary:
* printf for JSON
* Parameters:
* const char *fmt, ...
* Return:
*  int
*
*******************************************************************************/
char *json_asprintf(const char *fmt, ...) WEAK;
char *json_asprintf(const char *fmt, ...) 
{
  char *result = NULL;
  va_list ap;
  va_start(ap, fmt);
  result = json_vasprintf(fmt, ap);
  //va_end(ap);
  return result;
}

/* Modified and compiled from
 *
 * HappyHTTP - a simple HTTP library
 * Version 0.1
 *
 * Contains code from Copyright (c) 2006 Ben Campbell and
 * Copyright (c) 2013 Yaroslav Stavnichiy <yarosla@gmail.com> NXJSON
 */
/*******************************************************************************
* Function Name: unicode_to_utf8
********************************************************************************
* Summary:
* convert unicode to utf8
* Parameters:
* unsigned int codepoint, char* p, char** endp
* Return:
*  int
*  code from http://stackoverflow.com/a/4609989/697313
*******************************************************************************/
static int16_t unicode_to_utf8(uint32_t codepoint, char* p, char** endp)
{
  if (p == NULL)
    return -1;
  if (codepoint<0x80UL) *p++=codepoint;
  else if (codepoint<0x800UL) *p++=192+codepoint/64, *p++=128+codepoint%64;
  else if (codepoint-0xd800UL<0x800UL) return 0;                                // surrogate must have been treated earlier
  else if (codepoint<0x10000UL) *p++=224+codepoint/4096UL, *p++=128+codepoint/64UL%64, *p++=128+codepoint%64;
  else if (codepoint<0x110000UL) *p++=240+codepoint/262144UL, *p++=128+codepoint/4096UL%64, *p++=128+codepoint/64UL%64, *p++=128+codepoint%64;
  else return 0;                                                                // error
  *endp=p;
  return 1;
}
/*******************************************************************************
* Function Name: hex_val
********************************************************************************
* Summary:
* return int for hex value
* Parameters:
* char
* Return:
*  int
*
*******************************************************************************/
static inline int16_t hex_val(char c)
{
  if (c>='0' && c<='9') return c-'0';
  if (c>='a' && c<='f') return c-'a'+10;
  if (c>='A' && c<='F') return c-'A'+10;
  return -1;
}
/*******************************************************************************
* Function Name: CharToHex
********************************************************************************
* Summary:
* Convert utf32 to utf8
* Parameters:
* unsigned char c, char * hexBuf
* Return:
*  nothing
*
*******************************************************************************/
static void CharToHex(unsigned char c, char * hexBuf)
{
    const char * hexchar = "0123456789ABCDEF";
    hexBuf[0u] = hexchar[c >> 4u];
    hexBuf[1u] = hexchar[c & 0x0FUL];
}


#define JSON_INV_UNI_C_ESC -1                                                   // invalid unicode escape
#define JSON_INV_UNI_C_SUR -2                                                   // invalid unicode surrogate
#define JSON_INV_CODE_POINT -3                                                  // invalid code point
#define JSON_NO_CLOSE_QUOTE -4                                                  // no close quote
/*******************************************************************************
* Function Name: unescape_string
********************************************************************************
* Summary:
* Convert from unicode
* Parameters:
* char* s, char** end, nx_json_unicode_encoder encoder
* Return:
*  char* (char stream)
*
*******************************************************************************/
static char* unescape_string(char* s, char** end, nx_json_unicode_encoder encoder) 
{
  char* p=s;
  char* d=s;
  char c;
  uint32_t codepoint;
  uint32_t codepoint2;
  char* ps;
  int16_t h1, h2, h3, h4;
  
  if (p == NULL)
    return (char*) JSON_INV_CODE_POINT;
    
  while ((c=*p++)) 
  {
    if (c=='"') 
    {
      *d='\0';
      *end=p;
      return s;                                                                 // return the char stream
    }
    else if (c=='\\') 
    {
      switch (*p) 
      {
        case '\\':
        case '/':
        case '"':
          *d++=*p++;
          break;
        case 'b':
          *d++='\b'; p++;
          break;
        case 'f':
          *d++='\f'; p++;
          break;
        case 'n':
          *d++='\n'; p++;
          break;
        case 'r':
          *d++='\r'; p++;
          break;
        case 't':
          *d++='\t'; p++;
          break;
        case 'u':                                                               // unicode
          if (!encoder) 
          {
            *d++=c;                                                             // leave untouched
            break;
          }
          ps=p-1;
          if ((h1=hex_val(p[1u]))<0 || (h2=hex_val(p[2u]))<0 || (h3=hex_val(p[3u]))<0 || (h4=hex_val(p[4u]))<0)
          {
            return (char*) JSON_INV_UNI_C_ESC;                                  // Invalid unicode escape
          }
          codepoint=h1<<12u|h2<<8u|h3<<4u|h4;
          if ((codepoint & 0xfc00UL)==0xd800UL)                                 // high surrogate; need one more unicode to succeed
          {
            p+=6;
            if (p[-1]!='\\' || *p!='u' || (h1=hex_val(p[1u]))<0 || (h2=hex_val(p[2u]))<0 || (h3=hex_val(p[3u]))<0 || (h4=hex_val(p[4u]))<0)
            {
              return (char*) JSON_INV_UNI_C_SUR;                                // invalid unicode surrogate
            }
            codepoint2=h1<<12u|h2<<8u|h3<<4u|h4;
            if ((codepoint2 & 0xfc00UL)!=0xdc00UL)
            {
              return (char*) JSON_INV_UNI_C_SUR;                                // invalid unicode surrogate
            }
            codepoint=0x10000UL+((codepoint-0xd800UL)<<10u)+(codepoint2-0xdc00UL);
          }
          if (!encoder(codepoint, d, &d)) 
          {
            return (char*) JSON_INV_CODE_POINT;                                 // invalid code point
          }
          p+=5;
          break;
        default:
          *d++=c;                                                               // leave untouched
          break;
      }
    }
    else {
      *d++=c;
    }
  }
  return (char*) JSON_NO_CLOSE_QUOTE;                                           // no closing quote for string
}
/*******************************************************************************
* Function Name: Utf32toUtf8
********************************************************************************
* Summary:
* Convert utf32 to utf8
* Parameters:
* unsigned int codepoint, char * utf8Buf
* Return:
*  nothing
*
*******************************************************************************/
static void Utf32toUtf8(uint32_t codepoint, char * utf8Buf)
{
    if (utf8Buf == NULL)
        return;
        
    if (codepoint < 0x80UL) {
        utf8Buf[0u] = (char) codepoint;
        utf8Buf[1u] = 0;
    } else if (codepoint < 0x0800UL) {
        utf8Buf[0u] = (char) ((codepoint >> 6) | 0xC0);
        utf8Buf[1u] = (char) ((codepoint & 0x3F) | 0x80);
        utf8Buf[2u] = 0;
    } else if (codepoint < 0x10000UL) {
        utf8Buf[0u] = (char) ((codepoint >> 12u) | 0xE0U);
        utf8Buf[1u] = (char) (((codepoint >> 6u) & 0x3FU) | 0x80U);
        utf8Buf[2u] = (char) ((codepoint & 0x3FU) | 0x80U);
        utf8Buf[3u] = 0;
    } else if (codepoint < 0x200000UL) {
        utf8Buf[0u] =(char)((codepoint >> 18u) | 0xF0U);
        utf8Buf[1u] =(char)(((codepoint >> 12u) & 0x3FU) | 0x80U);
        utf8Buf[2u] =(char)(((codepoint >> 6u) & 0x3FU) | 0x80U);
        utf8Buf[3u] =(char)((codepoint & 0x3FU) | 0x80U);
        utf8Buf[4u] = 0;
    } else {
        utf8Buf[0u] = '?';
        utf8Buf[1u] = 0;
    }
}

/*******************************************************************************
* Function Name: charToUpperCase
********************************************************************************
* Summary:
* Convert a char to lupper case  - instead of toupper();
* Parameters:
* char character
* Return:
*  char
*
*******************************************************************************/
char charToUpperCase(char character)
{
  char toReturn;

  toReturn = character;
  if(character == 'a'){
    toReturn = 'A';
  }else if(character == 'b'){
    toReturn = 'B';
  }else if(character == 'c'){
    toReturn = 'C';
  }else if(character == 'd'){
    toReturn = 'D';
  }else if(character == 'e'){
    toReturn = 'E';
  }else if(character == 'f'){
    toReturn = 'F';
  }else if(character == 'g'){
    toReturn = 'G';
  }else if(character == 'h'){
    toReturn = 'H';
  }else if(character == 'i'){
    toReturn = 'I';
  }else if(character == 'j'){
    toReturn = 'J';
  }else if(character == 'k'){
    toReturn = 'K';
  }else if(character == 'l'){
    toReturn = 'L';
  }else if(character == 'm'){
    toReturn = 'M';
  }else if(character == 'n'){
    toReturn = 'N';
  }else if(character == 'o'){
    toReturn = 'O';
  }else if(character == 'p'){
    toReturn = 'P';
  }else if(character == 'q'){
    toReturn = 'Q';
  }else if(character == 'r'){
    toReturn = 'R';
  }else if(character == 's'){
    toReturn = 'S';
  }else if(character == 't'){
    toReturn = 'T';
  }else if(character == 'u'){
    toReturn = 'U';
  }else if(character == 'v'){
    toReturn = 'V';
  }else if(character == 'w'){
    toReturn = 'W';
  }else if(character == 'x'){
    toReturn = 'X';
  }else if(character == 'y'){
    toReturn = 'Y';
  }else if(character == 'z'){
    toReturn = 'Z';
  }
  return toReturn;
}
/*******************************************************************************
* Function Name: charToLowerCase
********************************************************************************
* Summary:
* Convert a char to lower case  - instead of tolower();
* Parameters:
* char character
* Return:
*  char
*
*******************************************************************************/
char charToLowerCase(char character)
{
  char toReturn;

  toReturn = character;
  if(character == 'A'){
    toReturn = 'a';
  }else if(character == 'B'){
    toReturn = 'b';
  }else if(character == 'C'){
    toReturn = 'c';
  }else if(character == 'D'){
    toReturn = 'd';
  }else if(character == 'E'){
    toReturn = 'e';
  }else if(character == 'F'){
    toReturn = 'f';
  }else if(character == 'G'){
    toReturn = 'g';
  }else if(character == 'H'){
    toReturn = 'h';
  }else if(character == 'I'){
    toReturn = 'i';
  }else if(character == 'J'){
    toReturn = 'j';
  }else if(character == 'K'){
    toReturn = 'k';
  }else if(character == 'L'){
    toReturn = 'l';
  }else if(character == 'M'){
    toReturn = 'm';
  }else if(character == 'N'){
    toReturn = 'n';
  }else if(character == 'O'){
    toReturn = 'o';
  }else if(character == 'P'){
    toReturn = 'p';
  }else if(character == 'Q'){
    toReturn = 'q';
  }else if(character == 'R'){
    toReturn = 'r';
  }else if(character == 'S'){
    toReturn = 's';
  }else if(character == 'T'){
    toReturn = 't';
  }else if(character == 'U'){
    toReturn = 'u';
  }else if(character == 'V'){
    toReturn = 'v';
  }else if(character == 'W'){
    toReturn = 'w';
  }else if(character == 'X'){
    toReturn = 'x';
  }else if(character == 'Y'){
    toReturn = 'y';
  }else if(character == 'Z'){
    toReturn = 'z';
  }

  return toReturn;
}
/*******************************************************************************
* Function Name: str2Upper
********************************************************************************
* Summary:
* Convert a string to upper case instead of toupper();
* Parameters:
* char* str
* Return:
*  string
*
*******************************************************************************/
char* str2Upper(char* str)
{
    char * p = str;
    if (NULL == str)
      return '\0';

    while(p != '\0')
    {
        *p=charToUpperCase(*p);
        ++p;
    }
    return p;
}
/*******************************************************************************
* Function Name: str2Lower
********************************************************************************
* Summary:
* Convert a string to lower case  - instead of tolower();
* Parameters:
* char* str
* Return:
*  string
*
*******************************************************************************/
char* str2Lower(char* str)
{
    char * p = str;
    if (NULL == str)
      return '\0';

    while(p != '\0')
    {
        *p=charToLowerCase(*p);
        ++p;
    }
    return str;
}
/*******************************************************************************
* Function Name: http_putheader
********************************************************************************
* Summary:
* puts http header
* Parameters:
* const char* header, int numericvalue const char* strvalue, uint8_t typ
* Return:
*  nothing
*
*******************************************************************************/
static void http_putheader( const char* header, int16_t numericvalue, const char* strvalue, uint8_t typ )
{
        char buf[32u];
        
        if (header == NULL)                                                     /* allows strvalue to be a NULL when called - Misra ? */
           return;
           
        switch (typ)
        {
           case 0u:
           sprintf( buf, "%d", numericvalue );
           break;
           
           case 1u:
           sprintf( buf, "%s", strvalue );
           break;
           
           default:
           return;
        }

        sprintf( (char*) header, "%s%s",(char*) header,(char *) buf );
}
/*******************************************************************************
* Function Name: http_putrequest
********************************************************************************
* Summary:
* puts http
* Parameters:
* const char* method, const char* url
* Return:
*  nothing
*
*******************************************************************************/
static void http_putrequest( const char* method, const char* url )                     // method is GET or PUT url is xxx.com
{
        //m_State = REQ_STARTED;

        char req[ 512U ];
        
        if ((method == NULL) || (url == NULL))
           return;
           
        sprintf( req, "%s %s HTTP/1.1", method, url );
        //m_Buffer.push_back( req );

        // http_putheader( "Host", m_Host.c_str() );                               // required for HTTP1.1  ???????????????????????????

        http_putheader((const char*) "Accept-Encoding",0U, (const char*) "identity", 1U);                          // don't want any fancy encodings please

        // Push a new response onto the queue
        // Response *r = new Response( method, *this );
        // m_Outstanding.push_back( r );
}
/*******************************************************************************
* Function Name: http_request
********************************************************************************
* Summary:
* puts http
* Parameters:
* const char* method, const char* url const char* headers[] const unsigned char* body int bodysize
* Return:
*  nothing
*
*******************************************************************************/
static void http_request( const char* method,                                   // GET or PUT
        const char* url,                                                        // e.g.
        const char* headers[],
        const unsigned char* body,
        int16_t bodysize )
{
        uint8_t gotcontentlength = false;                                       // already in headers?
        const char* name;
        const char* value;

        if (((url == NULL) || (headers == NULL)) || (body == NULL))             /* assumes none can be NULL */
            return;
            
        // check headers for content-length
        // TODO: check for "Host" and "Accept-Encoding" too
        // and avoid adding them ourselves in putrequest()
        if( headers )
        {
           const char** h = headers;
           while( *h )
           {
              name = *h++;
              value = *h++;
              //assert( value != 0 );        // name with no value!

             name=str2Lower((char*) name);
             if( 0U==strcmp((char*) name,(char *) "content-length" ) )
                gotcontentlength = true;
           }
        }

        http_putrequest( method, url );

        if( body && !gotcontentlength )
           http_putheader((const char*) "Content-Length",(int16_t) bodysize,(const char*)  NULL, 0U );

        if( headers )
        {
           const char** h = headers;
           while( *h )
           {
              const char* name = *h++;
              const char* value = *h++;
              http_putheader( name,(int) value, (const char*) NULL, 0U );
           }
        }
//        endheaders();
//        if( body )
//           send( body, bodysize );

}
/*******************************************************************************
* Function Name: AkiyamaTanigawaAlgorithm
********************************************************************************
* Summary:
* AkiyamaTanigawaAlgorithm for computing bernoulli number
* Parameters:
* uint64_t n
* Return:
* uint64_t
*
*******************************************************************************/
static uint64_t AkiyamaTanigawaAlgorithm(uint64_t n)
{
  uint64_t m, j, B;
  uint64_t *A;
  size_t ALength;

  A = (uint64_t*)malloc(sizeof(uint64_t) * (n + 1.0f));
  ALength = n + 1.0f;

  for(m = 0.0f; m <= n; m = m + 1.0f){
    A[(int)(m)] = 1.0f/(m + 1.0f);
    for(j = m; j >= 1.0f; j = j - 1.0f){
      A[(int)(j - 1.0f)] = j*(A[(int)(j - 1.0f)] - A[(int)(j)]);
    }
  }
  B = A[0u];
  free((char*)A,sizeof(uint64_t));
  return B;
}