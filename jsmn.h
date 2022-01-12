/*
 * MIT License
 *
 * Copyright (c) 2010 Serge Zaitsev
 * Ported by (C) 2020 A C P Avaiation Walkerburn Scotland
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef JSMN_H
#define JSMN_H

#include <stddef.h>

#ifdef __cplusplus

extern "C" {

#endif

#ifdef JSMN_STATIC
#define JSMN_API static
#else
#define JSMN_API extern
#endif

/**
 * JSON type identifier. Basic types are:
 *         o Object
 *         o Array
 *         o String
 *         o Other primitive: number, boolean (true/false) or null
 */
typedef enum {
  JSMN_UNDEFINED = 0,
  JSMN_OBJECT = 1,
  JSMN_ARRAY = 2,
  JSMN_STRING = 3,
  JSMN_PRIMITIVE = 4
} jsmntype_t;

enum jsmnerr {
  JSMN_ERROR_NOMEM = -1,                                                        /* Not enough tokens were provided */
  JSMN_ERROR_INVAL = -2,                                                        /* Invalid character inside JSON string */
  JSMN_ERROR_PART = -3                                                          /* The string is not a full JSON packet, more bytes expected */
};

/**
 * JSON token description.
 * type                type (object, array, string etc.)
 * start        start position in JSON data string
 * end                end position in JSON data string
 */
typedef struct {

  jsmntype_t type;
  int start;
  int end;
  int size;
#ifdef JSMN_PARENT_LINKS
  int parent;
#endif
} jsmntok_t;

/**
 * JSON parser. Contains an array of token blocks available. Also stores
 * the string being parsed now and current position in that string.
 */
typedef struct {
  unsigned int pos;     /* offset in the JSON string */
  unsigned int toknext; /* next token to allocate */
  int toksuper;         /* superior token node, e.g. parent object or array */
} jsmn_parser;

/**
 * Create JSON parser over an array of tokens
 */
JSMN_API void jsmn_init(jsmn_parser *parser);

/**
 * Run JSON parser. It parses a JSON data string into and array of tokens, each
 * describing
 * a single JSON object.
 */

JSMN_API int jsmn_parse(jsmn_parser *parser, const char *js, const size_t len,
                        jsmntok_t *tokens, const unsigned int num_tokens);

//int16_t geoJSON_parse_reply(unsigned char *JSON_STRING, geoCoord_Square_t *pReply);
//static int8_t parseGeoJSonArrObj(char* p, geoCoord_Square_t *pReply);

//#ifndef JSMN_HEADER

/**
 * Allocates a fresh unused token from the token pool.
 */
/*******************************************************************************
* Function Name: jsmn_alloc_token
********************************************************************************
* Summary:
*  Allocates a fresh unused token from the token pool.
* Parameters:
* jsmn_parser *parser
* jsmntok_t *tokens
* const size_t num_tokens
* Return:
*  jsmntok_t
*
*******************************************************************************/
static jsmntok_t *jsmn_alloc_token(jsmn_parser *parser, jsmntok_t *tokens,

                                   const size_t num_tokens) {

  jsmntok_t *tok;

  if (parser->toknext >= num_tokens) {

    return NULL;

  }

  tok = &tokens[parser->toknext++];

  tok->start = tok->end = -1;

  tok->size = 0;

#ifdef JSMN_PARENT_LINKS

  tok->parent = -1;

#endif

  return tok;

}

/*******************************************************************************
* Function Name: jsmn_fill_token
********************************************************************************
* Summary:
*  Fills token type and boundaries.
* Parameters:
* jsmntok_t *tokens
* const jsmntype_t type
* const int start
* const int end
* Return:
*  void
*
*******************************************************************************/
static void jsmn_fill_token(jsmntok_t *token, const jsmntype_t type,

                            const int start, const int end) {

  token->type = type;
  token->start = start;
  token->end = end;
  token->size = 0;

}

/*******************************************************************************
* Function Name: jsmn_parse_primitive
********************************************************************************
* Summary:
*  Fills next available token with JSON primitive.
* Parameters:
*  jsmn_parser *parser
* const char *js
* const size_t len
* jsmntok_t *tokens
* const unsigned int num_tokens
* Return:
*  int16_t
*
*******************************************************************************/
static int jsmn_parse_primitive(jsmn_parser *parser, const char *js,
                                const size_t len, jsmntok_t *tokens,
                                const size_t num_tokens) {

  jsmntok_t *token;
  int start;
  start = parser->pos;

  for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) 
  {
    switch (js[parser->pos]) {

#ifndef JSMN_STRICT

    /* In strict mode primitive must be followed by "," or "}" or "]" */

    case ':':

#endif

    case '\t':

    case '\r':

    case '\n':

    case ' ':

    case ',':

    case ']':

    case '}':

      goto found;

    default:
                   /* to quiet a warning from gcc*/
      break;
    }

    if (js[parser->pos] < 32 || js[parser->pos] >= 127) {
      parser->pos = start;
      return JSMN_ERROR_INVAL;
    }
  }

#ifdef JSMN_STRICT

  /* In strict mode primitive must be followed by a comma/object/array */

  parser->pos = start;
  return JSMN_ERROR_PART;

#endif



found:

  if (tokens == NULL) {
    parser->pos--;
    return 0;
  }

  token = jsmn_alloc_token(parser, tokens, num_tokens);
  if (token == NULL) {
    parser->pos = start;
    return JSMN_ERROR_NOMEM;
  }

  jsmn_fill_token(token, JSMN_PRIMITIVE, start, parser->pos);

#ifdef JSMN_PARENT_LINKS

  token->parent = parser->toksuper;

#endif

  parser->pos--;
  return 0;
}

/*******************************************************************************
* Function Name: jsmn_parse_string
********************************************************************************
* Summary:
*  Fills next token with JSON string.
* Parameters:
*  jsmn_parser *parser
* const char *js
* const size_t len
* jsmntok_t *tokens
* const unsigned int num_tokens
* Return:
*  int16_t
*
*******************************************************************************/
static int jsmn_parse_string(jsmn_parser *parser, const char *js,
                             const size_t len, jsmntok_t *tokens,
                             const size_t num_tokens) {

  jsmntok_t *token;
  int start = parser->pos;
  parser->pos++;

  /* Skip starting quote */
  for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
    char c = js[parser->pos];
    /* Quote: end of string */
    if (c == '\"') {
      if (tokens == NULL) {
        return 0;
      }

      token = jsmn_alloc_token(parser, tokens, num_tokens);

      if (token == NULL) {
        parser->pos = start;
        return JSMN_ERROR_NOMEM;
      }
      jsmn_fill_token(token, JSMN_STRING, start + 1, parser->pos);

#ifdef JSMN_PARENT_LINKS
      token->parent = parser->toksuper;
#endif
      return 0;
    }
    /* Backslash: Quoted symbol expected */
    if (c == '\\' && parser->pos + 1 < len) {
      int i;
      parser->pos++;
      switch (js[parser->pos]) {
      /* Allowed escaped symbols */

      case '\"':

      case '/':

      case '\\':

      case 'b':

      case 'f':

      case 'r':

      case 'n':

      case 't':
        break;
      /* Allows escaped symbol \uXXXX */

      case 'u':
        parser->pos++;
        for (i = 0; i < 4 && parser->pos < len && js[parser->pos] != '\0';
             i++) {
          /* If it isn't a hex character we have an error */
          if (!((js[parser->pos] >= 48 && js[parser->pos] <= 57) ||   /* 0-9 */
                (js[parser->pos] >= 65 && js[parser->pos] <= 70) ||   /* A-F */
                (js[parser->pos] >= 97 && js[parser->pos] <= 102))) { /* a-f */
            parser->pos = start;
            return JSMN_ERROR_INVAL;
          }

          parser->pos++;
        }

        parser->pos--;
        break;

      /* Unexpected symbol */
      default:

        parser->pos = start;
        return JSMN_ERROR_INVAL;
      }
    }
  }
  parser->pos = start;
  return JSMN_ERROR_PART;
}

/*******************************************************************************
* Function Name: jsmn_parse
********************************************************************************
* Summary:
*  Parse JSON string and fill tokens.
* Parameters:
*  jsmn_parser *parser
* const char *js
* const size_t len
* jsmntok_t *tokens
* const unsigned int num_tokens
* Return:
*  int16_t
*
*******************************************************************************/
JSMN_API int jsmn_parse(jsmn_parser *parser, const char *js, const size_t len, jsmntok_t *tokens, const unsigned int num_tokens) 
{

  int r;
  int i;
  jsmntok_t *token;
  int count = parser->toknext;

  for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
    char c;
    jsmntype_t type;
    c = js[parser->pos];

    switch (c) {

    case '{':

    case '[':

      count++;

      if (tokens == NULL) {

        break;

      }

      token = jsmn_alloc_token(parser, tokens, num_tokens);

      if (token == NULL) {

        return JSMN_ERROR_NOMEM;

      }

      if (parser->toksuper != -1) {

        jsmntok_t *t = &tokens[parser->toksuper];

#ifdef JSMN_STRICT
        /* In strict mode an object or array can't become a key */
        if (t->type == JSMN_OBJECT) {
          return JSMN_ERROR_INVAL;
        }
#endif
        t->size++;
#ifdef JSMN_PARENT_LINKS
        token->parent = parser->toksuper;
#endif
      }

      //token->type = (c == '{' ? JSMN_OBJECT : JSMN_ARRAY);
      if (c == '{')
      {
         token->type = JSMN_OBJECT;
      }
      else
      {
         token->type = JSMN_ARRAY;
      }
      token->start = parser->pos;
      parser->toksuper = parser->toknext - 1;
      break;

    case '}':

    case ']':
      if (tokens == NULL) {
        break;
      }

      // type = (c == '}' ? JSMN_OBJECT : JSMN_ARRAY);
      if (c == '}')
      {
         type = JSMN_OBJECT;
      }
      else
      {
         type = JSMN_ARRAY;
      }

#ifdef JSMN_PARENT_LINKS

      if (parser->toknext < 1) {
        return JSMN_ERROR_INVAL;
      }

      token = &tokens[parser->toknext - 1];

      for (;;) {
        if (token->start != -1 && token->end == -1) {
          if (token->type != type) {
            return JSMN_ERROR_INVAL;
          }
          token->end = parser->pos + 1;
          parser->toksuper = token->parent;
          break;
        }

        if (token->parent == -1) {
          if (token->type != type || parser->toksuper == -1) {
            return JSMN_ERROR_INVAL;
          }
          break;
        }
        token = &tokens[token->parent];
      }
#else
      for (i = parser->toknext - 1; i >= 0; i--) {
        token = &tokens[i];
        if (token->start != -1 && token->end == -1) {
          if (token->type != type) {
            return JSMN_ERROR_INVAL;
          }
          parser->toksuper = -1;
          token->end = parser->pos + 1;
          break;
        }
      }

      /* Error if unmatched closing bracket */
      if (i == -1) {
        return JSMN_ERROR_INVAL;
      }

      for (; i >= 0; i--) {
        token = &tokens[i];
        if (token->start != -1 && token->end == -1) {
          parser->toksuper = i;
          break;
        }
      }
#endif
      break;

    case '\"':
      r = jsmn_parse_string(parser, js, len, tokens, num_tokens);
      if (r < 0) {
        return r;
      }
      count++;
      if (parser->toksuper != -1 && tokens != NULL) {
        tokens[parser->toksuper].size++;
      }
      break;

    case '\t':

    case '\r':

    case '\n':

    case ' ':
      break;

    case ':':
      parser->toksuper = parser->toknext - 1;
      break;

    case ',':
      if (tokens != NULL && parser->toksuper != -1 &&
          tokens[parser->toksuper].type != JSMN_ARRAY &&
          tokens[parser->toksuper].type != JSMN_OBJECT) {
#ifdef JSMN_PARENT_LINKS
        parser->toksuper = tokens[parser->toksuper].parent;
#else
        for (i = parser->toknext - 1; i >= 0; i--) {
          if (tokens[i].type == JSMN_ARRAY || tokens[i].type == JSMN_OBJECT) {
            if (tokens[i].start != -1 && tokens[i].end == -1) {
              parser->toksuper = i;
              break;
            }
          }
        }
#endif
      }
      break;
#ifdef JSMN_STRICT
    /* In strict mode primitives are: numbers and booleans */
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

    case 't':

    case 'f':

    case 'n':
      /* And they must not be keys of the object */
      if (tokens != NULL && parser->toksuper != -1) {
        const jsmntok_t *t = &tokens[parser->toksuper];
        if (t->type == JSMN_OBJECT ||
            (t->type == JSMN_STRING && t->size != 0)) {
          return JSMN_ERROR_INVAL;
        }
      }
#else
    /* In non-strict mode every unquoted value is a primitive */
    default:
#endif
      r = jsmn_parse_primitive(parser, js, len, tokens, num_tokens);
      if (r < 0) {
        return r;
      }
      count++;
      if (parser->toksuper != -1 && tokens != NULL) {
        tokens[parser->toksuper].size++;
      }
      break;

#ifdef JSMN_STRICT
    /* Unexpected char in strict mode */
    default:
      return JSMN_ERROR_INVAL;
#endif
    }
  }

  if (tokens != NULL) {
    for (i = parser->toknext - 1; i >= 0; i--) {
      /* Unmatched opened object or array */
      if (tokens[i].start != -1 && tokens[i].end == -1) {
        return JSMN_ERROR_PART;
      }
    }
  }
  return count;
}

/*******************************************************************************
* Function Name: jsmn_init
********************************************************************************
* Summary:
*  Creates a new parser based over a given buffer with an array of tokens available.
* Parameters:
*  jsmn_parser *parser
* Return:
*  void
*
*******************************************************************************/
JSMN_API void jsmn_init(jsmn_parser *parser) {
  parser->pos = 0;
  parser->toknext = 0;
  parser->toksuper = -1;
}
 /*******************************************************************************
* Function Name: removeSubstr
********************************************************************************
* Summary:
*  Remove a char from a string
* Parameters:
*  char *string - string to work on
*  char *sub - char or string to remove from above string
* Return:
*  nothing (void)
*
*******************************************************************************/
JSMN_API void removeSubstr(char *string,char *sub)
{
    char *match = string;
    int len = strlen(sub);
    while ((match = strstr(match, sub)))
    {
        *match = '\0';
        strcat(string, match+len);
        match++;
    }
}
 /*******************************************************************************
* Function Name: stringFilter
********************************************************************************
* Summary:
*  Remove ac or b from string (example filter for processing)
* Parameters:
*  char *str
* Return:
*  nothing (void)
*
*******************************************************************************/
JSMN_API void stringFilter(char *str)
{
    int n = strlen(str);

    int i = -1;                                                                 // previous character
    int j = 0;                                                                  // current character

    while (j < n)
    {
        if (j < n-1 && str[j] == 'a' && str[j+1] == 'c')                        /* check if current and next character forms ac */
            j += 2;
        else if (str[j] == 'b')                                                 /* if current character is b */
            j++;
        else if (i >= 0 && str[j] == 'c' && str[i] == 'a')                      /* if current char is 'c && last char in output is 'a' so delete both */
            i--,j++;
        else
            str[++i] = str[j++];                                                /* else copy curr char to output string */
    }
    str[++i] = '\0';
}
 /*******************************************************************************
* Function Name: RemoveCharFromString
********************************************************************************
* Summary:
*  Remove every occurance of a char from a string
* Parameters:
*  unsigned char * p
*  char c
* Return:
*  nothing (void)
*
*******************************************************************************/
JSMN_API void RemoveCharFromString( unsigned char * p, char c)
{
   unsigned char * pDest;
   if (NULL == p)
      return;
   pDest = p;

   while (*p)
   {
      if (*p != c)
            *pDest++ = *p;
          p++;
   }
   *pDest = '\0';
}
/*******************************************************************************
* Function Name: SubCharFromString
********************************************************************************
* Summary:
* Substitute a char c in a string with sub
* Parameters:
* unsigned char * p
* char c
* char sub
* Return:
*  nothing (void)
*
*******************************************************************************/
JSMN_API void SubCharFromString( unsigned char * p, char c, char sub)
{
   unsigned char * pDest;
   if (NULL == p)
      return;
   pDest = p;

   while (*p)
   {
      if (*p == c)
        *pDest = sub;
      else
        *pDest++ = *p;
      p++;
   }
   *pDest = '\0';
}



/*******************************************************************************
* Function Name: jsoneq
********************************************************************************
* Summary:
*  matches json string with string keyword s and puts location in tok
* Parameters:
*  unsigned char *json
*  jsmntok_t *tok
*  unsigned char *s
* Return:
*  0 = ok -1 no match
*
*******************************************************************************/
JSMN_API int jsoneq(unsigned char *json, jsmntok_t *tok, unsigned char *s)
{
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start && strncmp(json + tok->start, s, tok->end - tok->start) == 0)
  {
    return 0;
  }
  return -1;
}

#if defined(YI_CAM_USED)                                                        // ================== Yi Action Cam used ===============================
/*******************************************************************************
* Function Name: xy_parse_reply
********************************************************************************
* Summary:
*  Parse the reply from a JSON reply message buffer
*  This has been modified for the tokens found in the Xiaomi Yi Action Camera
* Parameters:
*  unsigned char *JSON_STRING - string sent over tcp
*  XY_reply_t *pReply - reply structure
*  uint8_t parse_typ - all or individual word
* Return:
*  int16_t as per XY_JSparse_t
*
*******************************************************************************/
JSMN_API int16_t xy_parse_reply(unsigned char *JSON_STRING, XY_reply_t *pReply, uint8_t parse_typ, XY_config_t *XYConf)
{
  int i;
  int r;
  int j;
  int destValue;                                                                // returned integer (return code)
  unsigned char destString[32u];                                                // string to hold the values after the keys
  jsmn_parser p;                                                                // jasmine parser type
  jsmntok_t t[128u];                                                            /* We expect no more than 128 tokens */
  uint8_t tokenIsParam=false;                                                   // initialise it as false set to true if we get param from a 257 cmd
  int16_t tagsFound=0U;

   unsigned char testString[100u];                                              // string to write into when checking configuration reply
   uint8_t collect=0U;                                                          // collection step for configuration read

  if (pReply == NULL || XYConf == NULL)                                         // null passed return an error
        return tagsFound;
        
  jsmn_init(&p);
  // r = jsmn_parse( &p, JSON_STRING, strlen(JSON_STRING), t, sizeof(t) / sizeof(t[0]) );
  r = jsmn_parse( &p, (const char *)JSON_STRING, strlen(JSON_STRING), t, sizeof(t) / sizeof(t[0]) );
  
  if (r < 0)
  {
    tagsFound |= _XY_ERROR;
    return tagsFound;                                                           //printf("Failed to parse JSON: %d\n", r);
  }

  if (r < 1 || t[0].type != JSMN_OBJECT)                                        /* Assume the top-level element is an object */
  {
    tagsFound |= _XY_ERROR;
    return tagsFound;                                                           //printf("Object expected\n");
  }

  for (i = 1; i < r; i++)                                                       /* Loop over all keys of the root object */
  {
     if (jsoneq(JSON_STRING, &t[i], "token") == 0)                               // string was token then return its value
     {
        /* We may use strndup() to fetch string value */
        //printf("- User: %.*s\n", t[i + 1].end - t[i + 1].start,JSON_STRING + t[i + 1].start);  //

        // printf("%.*s: ", t[i].end - t[i].start,JSON_STRING + t[i].start);                      // token name only
        // printf("%.*s \n", t[i+1].end - t[i+1].start,JSON_STRING + t[i+1].start);               // value name only
        // strncpy(destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);            // copy only the value to a string
        // destValue=atoi(destString)+6U;                                                          // convert and add 6
        // printf("%s\n", JSON_STRING + t[i].start);                                              // prints all read (dump)
        // printf("value %d\n", destValue);                                                       // prints number
        // strncpy(destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        destValue=atoi((char*) &destString);                                    // convert the string to an integer
        tagsFound |= _XY_TOKEN;
        pReply->token = destValue;                                              // put the token reply number into the token field
        i++;
     }
     else if (jsoneq(JSON_STRING, &t[i], "rval") == 0)                          // string was rval (function code return value) then return its value
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        destValue=atoi((char*) &destString);                                    // convert the string to an integer
        tagsFound |= _XY_RVAL;
        pReply->rval = destValue;
        i++;
     }
     else if (jsoneq(JSON_STRING, &t[i], "md5sum") == 0)                        // string was md5sum then return its value
     {
        strncpy((char*) pReply->md5sum, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into the md5sum
        pReply->md5sum[t[i+1].end - t[i+1].start] = '\0';                       // Properly terminate the string with a NULL character
        tagsFound |= _XY_MD5SUM;                                                // return that an md5sum was found
        i++;
     }
     else if (jsoneq(JSON_STRING, &t[i], "size") == 0)                          // string was size then return its value
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        destValue=atol((char*) &destString);                                    // convert the string to an integer
        tagsFound |= _XY_SIZE;
        pReply->size = destValue;
        i++;
     }
     else if (jsoneq(JSON_STRING, &t[i], "resolution") == 0)                    // string was resolution then return its value
     {
        strncpy((char*) pReply->resolution, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into the md5sum
        pReply->resolution[t[i+1].end - t[i+1].start] = '\0';                   // Properly terminate the string with a NULL character
        tagsFound |= _XY_RESOL;                                                 // return that an resolution was found
        i++;
     }
     else if (jsoneq(JSON_STRING, &t[i], "msg_id") == 0)                        // string was msg_id (the message we are responding to) then return its value
     {
        tagsFound |= _XY_MSG_ID;
        strncpy((char *) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        destValue=atol((char *) &destString);                                   // convert the string to an integer
        pReply->msg_id = destValue;
        if ((pReply->msg_id) == 257U)                                           // The start-up string reply is :: { "rval": 0, "msg_id": 257, "param": 1 } where param is the token
           tokenIsParam=true;
        else if ((((pReply->msg_id) == 7U) || ((pReply->msg_id) == 9U)) || ((pReply->msg_id) == 5U))        // Status updates are found or a read options was requested or sd card space
           parse_typ = XY_READ_ALL;                                             // look for param and type replies
        else if (((pReply->msg_id) == 1U) || ((pReply->msg_id) == 2U))          // Get a single setting (type) found
           parse_typ = XY_READ_SPECIFIC;                                        // look for param and type replies
        i++;
     }
     else if (jsoneq(JSON_STRING, &t[i], "param") == 0)                          // string was msg_id (the message we are responding to) then return its value
     {
        if (tokenIsParam==true)                                                 // its the respomse to a start-up cmd 257
        {
           strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
           destString[t[i+1].end - t[i+1].start] = '\0';                        // Properly terminate the string with a NULL character
           destValue=atoi((char*) &destString);                                 // convert the string to an integer
           tagsFound |= _XY_TOKEN;
           pReply->token = destValue;
           tokenIsParam=false;
           i++;
        }
        else if (parse_typ == XY_READ_ALL)                                       // otherwise read the param responses to here
        {
          tagsFound |= _XY_PARAM;
          strncpy((char *) &pReply->param_str, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);  // copy all the parameters into the holding structure
          pReply->param_str[t[i+1].end - t[i+1].start] = '\0';                  // Properly terminate the string with a NULL character
          // - Param: [{"camera_clock":"2018-07-18 22:46:16"},{"video_standard":"NTSC"}]
          i++;
        }
    }
    else if ((jsoneq(JSON_STRING, &t[i], "type") == 0) && (parse_typ == XY_READ_ALL))    // string was type
    {
       /* We may additionally check if the value is either "true" or "false" */
       //printf("- Admin: %.*s\n", t[i + 1].end - t[i + 1].start,JSON_STRING + t[i + 1].start);
       tagsFound |= _XY_TYPE;
       strncpy((char *) &pReply->type_str, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);  // copy all the parameters into the holding structure
       pReply->type_str[t[i+1].end - t[i+1].start] = '\0';                      // Properly terminate the string with a NULL character
       i++;
    }
    else if ((jsoneq(JSON_STRING, &t[i], "video_standard") == 0) && (parse_typ == XY_READ_SPECIFIC))                // string was video standards
    {
       tagsFound |= _XY_VIDEO_STD;                                              // set the corresponding bit to return
       strncpy((char *) &pReply->video_std, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);  // copy all the video standard into the holding structure
       pReply->video_std[t[i+1].end - t[i+1].start] = '\0';                      // Properly terminate the string with a NULL character
       i++;
    }
    else if (((((jsoneq(JSON_STRING, &t[i], "video_resolution") == 0) || (jsoneq(JSON_STRING, &t[i], "video_loop_resolution") == 0)) || (jsoneq(JSON_STRING, &t[i], "timelapse_video_resolution") == 0)) || (jsoneq(JSON_STRING, &t[i], "video_photo_resolution") == 0)) && (parse_typ == XY_READ_SPECIFIC))             // string was video resolution
    {
       tagsFound |= _XY_VIDEO_RES;                                              // set the corresponding bit to return
       strncpy((char *) &pReply->video_res, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);  // copy all the videoresolution into the holding structure
       pReply->video_res[t[i+1].end - t[i+1].start] = '\0';                     // Properly terminate the string with a NULL character
       i++;
    }
    else if ((jsoneq(JSON_STRING, &t[i], "photo_size") == 0) && (parse_typ == XY_READ_SPECIFIC))                      // string was photo size string
    {
       tagsFound |= _XY_PHOTO_SZ;                                               // set the corresponding bit to return
       strncpy((char *) &pReply->photo_sz, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);  // copy all the video standard into the holding structure
       pReply->photo_sz[t[i+1].end - t[i+1].start] = '\0';                      // Properly terminate the string with a NULL character
       i++;
    }
    else if ((jsoneq(JSON_STRING, &t[i], "burst_capture_number") == 0) && (parse_typ == XY_READ_SPECIFIC))            // string was burst capture number string
    {
       tagsFound |= _XY_BURST_CAP;                                              // set the corresponding bit to return
       strncpy((char *) &pReply->burst_cap, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);  // copy all the video standard into the holding structure
       pReply->burst_cap[t[i+1].end - t[i+1].start] = '\0';                      // Properly terminate the string with a NULL character
       i++;
    }
    else if (jsoneq(JSON_STRING, &t[i], "groups") == 0)                         // left as a group example from serge original code dont think Yi responds like this
    {
       //printf("- Groups:\n");
       if (t[i + 1].type != JSMN_ARRAY)
       {
         continue;                                                              /* We expect groups to be an array of strings */
       }
       for (j = 0; j < t[i + 1].size; j++)
       {
         jsmntok_t *g = &t[i + j + 2];
         switch (collect)                                                       // collect each config tag value and put into the config structure
         {
            case 1U:
            //printf("clock  * %.*s\r", g->end - g->start, JSON_STRING + g->start);
            strncpy((char*)XYConf->camera_clock , (char *) JSON_STRING + g->start, g->end - g->start);
            XYConf->camera_clock[g->end - g->start] = '\0';                     // Properly terminate the string with a NULL character
            break;

            case 2U:
            //printf("res  * %.*s\r", g->end - g->start, JSON_STRING + g->start);
            strncpy((char*)XYConf->video_resolution , (char *) JSON_STRING + g->start, g->end - g->start);
            XYConf->video_resolution[g->end - g->start] = '\0';                 // Properly terminate the string with a NULL character
            break;

            case 3U:
            //printf("quality  * %.*s\r", g->end - g->start, JSON_STRING + g->start);
            strncpy((char*)XYConf->video_quality , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->video_quality[g->end - g->start] = '\0';                    // Properly terminate the string with a NULL character
            break;

            case 4U:
            //printf("photo size  * %.*s\r", g->end - g->start, JSON_STRING + g->start);
            strncpy((char*)XYConf->photo_size , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->photo_size[g->end - g->start] = '\0';                       // Properly terminate the string with a NULL character
            break;

            case 5U:
            //printf("video_standard  * %.*s\r", g->end - g->start, JSON_STRING + g->start);
            strncpy((char*)XYConf->video_standard , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->video_standard[g->end - g->start] = '\0';                   // Properly terminate the string with a NULL character
            break;

            case 6U:
            //printf("app_status  * %.*s\r", g->end - g->start, JSON_STRING + g->start);
            strncpy((char*)XYConf->app_status , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->app_status[g->end - g->start] = '\0';                       // Properly terminate the string with a NULL character
            break;

            case 7U:
            //printf("video_stamp  * %.*s\r", g->end - g->start, JSON_STRING + g->start);
            strncpy((char*)XYConf->video_stamp , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->video_stamp[g->end - g->start] = '\0';                      // Properly terminate the string with a NULL character
            break;

            case 8U:
            strncpy((char*)XYConf->timelapse_video , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->timelapse_video[g->end - g->start] = '\0';                   // Properly terminate the string with a NULL character
            break;

            case 9U:
            strncpy((char*)XYConf->capture_mode , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->capture_mode[g->end - g->start] = '\0';                     // Properly terminate the string with a NULL character
            break;

            case 10U:
            strncpy((char*)XYConf->timelapse_photo , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->timelapse_photo[g->end - g->start] = '\0';                  // Properly terminate the string with a NULL character
            break;

            case 11U:
            strncpy((char*)XYConf->preview_status , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->preview_status[g->end - g->start] = '\0';                   // Properly terminate the string with a NULL character
            break;

            case 12U:
            strncpy((char*)XYConf->buzzer_volume , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->buzzer_volume[g->end - g->start] = '\0';                    // Properly terminate the string with a NULL character
            break;

            case 13U:
            strncpy((char*)XYConf->buzzer_ring , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->buzzer_ring[g->end - g->start] = '\0';                      // Properly terminate the string with a NULL character
            break;

            case 14U:
            strncpy((char*)XYConf->capture_default_mode , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->capture_default_mode[g->end - g->start] = '\0';             // Properly terminate the string with a NULL character
            break;

            case 15U:
            strncpy((char*)XYConf->precise_cont_time , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->precise_cont_time[g->end - g->start] = '\0';                // Properly terminate the string with a NULL character
            break;

            case 16U:
            strncpy((char*)XYConf->burst_capture_number , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->burst_capture_number[g->end - g->start] = '\0';             // Properly terminate the string with a NULL character
            break;

            case 17U:
            strncpy((char*)XYConf->restore_factory_settings , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->restore_factory_settings[g->end - g->start] = '\0';         // Properly terminate the string with a NULL character
            break;

            case 18U:
            strncpy((char*)XYConf->led_mode , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->led_mode[g->end - g->start] = '\0';                         // Properly terminate the string with a NULL character
            break;

            case 19U:
            strncpy((char*)XYConf->dev_reboot , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->dev_reboot[g->end - g->start] = '\0';                       // Properly terminate the string with a NULL character
            break;

            case 20U:
            strncpy((char*)XYConf->meter_mode , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->meter_mode[g->end - g->start] = '\0';                       // Properly terminate the string with a NULL character
            break;

            case 21U:
            strncpy((char*)XYConf->sd_card_status , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->sd_card_status[g->end - g->start] = '\0';                   // Properly terminate the string with a NULL character
            break;

            case 22U:
            strncpy((char*)XYConf->video_output_dev_type , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->video_output_dev_type[g->end - g->start] = '\0';            // Properly terminate the string with a NULL character
            break;

            case 23U:
            strncpy((char*)XYConf->sw_version , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->sw_version[g->end - g->start] = '\0';                       // Properly terminate the string with a NULL character
            break;

            case 24U:
            strncpy((char*)XYConf->hw_version , (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->hw_version[g->end - g->start] = '\0';                       // Properly terminate the string with a NULL character
            break;

            case 25U:
            strncpy((char*)XYConf->dual_stream_status, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->dual_stream_status[g->end - g->start] = '\0';               // Properly terminate the string with a NULL character
            break;

            case 26U:
            strncpy((char*)XYConf->streaming_status, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->streaming_status[g->end - g->start] = '\0';                 // Properly terminate the string with a NULL character
            break;

            case 27U:
            strncpy((char*)XYConf->precise_cont_capturing, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->precise_cont_capturing[g->end - g->start] = '\0';           // Properly terminate the string with a NULL character
            break;

            case 28U:
            strncpy((char*)XYConf->piv_enable, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->piv_enable[g->end - g->start] = '\0';                       // Properly terminate the string with a NULL character
            break;

            case 29U:
            strncpy((char*)XYConf->auto_low_light, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->auto_low_light[g->end - g->start] = '\0';                   // Properly terminate the string with a NULL character
            break;

            case 30U:
            strncpy((char*)XYConf->loop_record, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->loop_record[g->end - g->start] = '\0';                      // Properly terminate the string with a NULL character
            break;

            case 31U:
            strncpy((char*)XYConf->warp_enable, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->warp_enable[g->end - g->start] = '\0';                      // Properly terminate the string with a NULL character
            break;

            case 32U:
            strncpy((char*)XYConf->support_auto_low_light, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->support_auto_low_light[g->end - g->start] = '\0';           // Properly terminate the string with a NULL character
            break;

            case 33U:
            strncpy((char*)XYConf->precise_selftime, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->precise_selftime[g->end - g->start] = '\0';                 // Properly terminate the string with a NULL character
            break;

            case 34U:
            strncpy((char*)XYConf->precise_self_running, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->precise_self_running[g->end - g->start] = '\0';             // Properly terminate the string with a NULL character
            break;

            case 35U:
            strncpy((char*)XYConf->auto_power_off, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->auto_power_off[g->end - g->start] = '\0';                   // Properly terminate the string with a NULL character
            break;

            case 36U:
            strncpy((char*)XYConf->serial_number, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->serial_number[g->end - g->start] = '\0';                    // Properly terminate the string with a NULL character
            break;

            case 37U:
            strncpy((char*)XYConf->system_mode, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->system_mode[g->end - g->start] = '\0';                      // Properly terminate the string with a NULL character
            break;

            case 38U:
            strncpy((char*)XYConf->system_default_mode, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->system_default_mode[g->end - g->start] = '\0';              // Properly terminate the string with a NULL character
            break;

            case 39U:
            strncpy((char*)XYConf->start_wifi_while_booted, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->start_wifi_while_booted[g->end - g->start] = '\0';          // Properly terminate the string with a NULL character
            break;

            case 40U:
            strncpy((char*)XYConf->quick_record_time, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->quick_record_time[g->end - g->start] = '\0';                // Properly terminate the string with a NULL character
            break;

            case 41U:
            strncpy((char*)XYConf->precise_self_remain_time, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->precise_self_remain_time[g->end - g->start] = '\0';         // Properly terminate the string with a NULL character
            break;

            case 42U:
            strncpy((char*)XYConf->sdcard_need_format, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->sdcard_need_format[g->end - g->start] = '\0';               // Properly terminate the string with a NULL character
            break;

            case 43U:
            strncpy((char*)XYConf->video_rotate, (char * ) JSON_STRING + g->start, g->end - g->start);
            XYConf->video_rotate[g->end - g->start] = '\0';                     // Properly terminate the string with a NULL character
            break;

            default:
            break;
         }
         strncpy((char*)testString , (char * ) JSON_STRING + g->start, g->end - g->start);
         testString[g->end - g->start]='\0';

         if (!strcmp((char*) testString, (char*) "video_resolution"))
         {
           collect=2U;
         }
         else if (!strcmp((char*) testString, (char*) "camera_clock"))
         {
           collect=1U;
         }
         else if (!strcmp((char*) testString, (char*) "video_quality"))
         {
           collect=3U;
         }
         else if (!strcmp((char*) testString, (char*) "photo_size"))
         {
           collect=4U;
         }
         else if (!strcmp((char*) testString, (char*) "video_standard"))
         {
           collect=5U;
         }
         else if (!strcmp((char*) testString, (char*) "app_status"))
         {
           collect=6U;
         }
         else if (!strcmp((char*) testString, (char*) "video_stamp"))
         {
           collect=7U;
         }
         else if (!strcmp((char*) testString, (char*) "timelapse_video"))
         {
           collect=8U;
         }
         else if (!strcmp((char*) testString, (char*) "capture_mode"))
         {
           collect=9U;
         }
         else if (!strcmp((char*) testString, (char*) "timelapse_photo"))
         {
           collect=10U;
         }
         else if (!strcmp((char*) testString, (char*) "preview_status"))
         {
           collect=11U;
         }
         else if (!strcmp((char*) testString, (char*) "buzzer_volume"))
         {
           collect=12U;
         }
         else if (!strcmp((char*) testString, (char*) "buzzer_ring"))
         {
           collect=13U;
         }
         else if (!strcmp((char*) testString, (char*) "capture_default_mode"))
         {
           collect=14U;
         }
         else if (!strcmp((char*) testString, (char*) "precise_cont_time"))
         {
           collect=15U;
         }
         else if (!strcmp((char*) testString, (char*) "burst_capture_number"))
         {
           collect=16U;
         }
         else if (!strcmp((char*) testString, (char*) "restore_factory_settings"))
         {
           collect=17U;
         }
         else if (!strcmp((char*) testString, (char*) "led_mode"))
         {
           collect=18U;
         }
         else if (!strcmp((char*) testString, (char*) "dev_reboot"))
         {
           collect=19U;
         }
         else if (!strcmp((char*) testString, (char*) "meter_mode"))
         {
           collect=20U;
         }
         else if (!strcmp((char*) testString, (char*) "sd_card_status"))
         {
           collect=21U;
         }
         else if (!strcmp((char*) testString, (char*) "video_output_dev_type"))
         {
           collect=22U;
         }
         else if (!strcmp((char*) testString, (char*) "sw_version"))
         {
           collect=23U;
         }
         else if (!strcmp((char*) testString, (char*) "hw_version"))
         {
           collect=24U;
         }
         else if (!strcmp((char*) testString, (char*) "dual_stream_status"))
         {
           collect=25U;
         }
         else if (!strcmp((char*) testString, (char*) "streaming_status"))
         {
           collect=26U;
         }
         else if (!strcmp((char*) testString, (char*) "precise_cont_capturing"))
         {
           collect=27U;
         }
         else if (!strcmp((char*) testString, (char*) "piv_enable"))
         {
           collect=28U;
         }
         else if (!strcmp((char*) testString, (char*) "auto_low_light"))
         {
           collect=29U;
         }
         else if (!strcmp((char*) testString, (char*) "loop_record"))
         {
           collect=30U;
         }
         else if (!strcmp((char*) testString, (char*) "warp_enable"))
         {
           collect=31U;
         }
         else if (!strcmp((char*) testString, (char*) "support_auto_low_light"))
         {
           collect=32U;
         }
         else if (!strcmp((char*) testString, (char*) "precise_selftime"))
         {
           collect=33U;
         }
         else if (!strcmp((char*) testString, (char*) "precise_self_running"))
         {
           collect=34U;
         }
         else if (!strcmp((char*) testString, (char*) "auto_power_off"))
         {
           collect=35U;
         }
         else if (!strcmp((char*) testString, (char*) "serial_number"))
         {
           collect=36U;
         }
         else if (!strcmp((char*) testString, (char*) "system_mode"))
         {
           collect=37U;
         }
         else if (!strcmp((char*) testString, (char*) "system_default_mode"))
         {
           collect=38U;
         }
         else if (!strcmp((char*) testString, (char*) "start_wifi_while_booted"))
         {
           collect=39U;
         }
         else if (!strcmp((char*) testString, (char*) "quick_record_time"))
         {
           collect=40U;
         }
         else if (!strcmp((char*) testString, (char*) "precise_self_remain_time"))
         {
           collect=41U;
         }
         else if (!strcmp((char*) testString, (char*) "sdcard_need_format"))
         {
           collect=42U;
         }
         else if (!strcmp((char*) testString, (char*) "video_rotate"))
         {
           collect=43U;
         }
         else
         {
           collect=0U;
         }
         //printf("  * %.*s\n", g->end - g->start, JSON_STRING + g->start);
       }
       i += t[i + 1].size + 1;
     }
     else
     {
        tagsFound |= _XY_UNEXPECT;                                              // set the corresponding bit to return                                                           // Set a bit to say we found unexpected tags
       //printf("Unexpected key: %.*s\n", t[i].end - t[i].start,JSON_STRING + t[i].start);
     }
  }
  return tagsFound;                                                             // Returns 0 if nothing found otherwise bit set for each tag found

}

/*******************************************************************************
* Function Name: ReportYiCamtoHMI
********************************************************************************
* Summary:
*  Move current reply values to those on the HMI
*
* Parameters:
*  XY_JSparse_t *parsedReply - tags found
*  COMGS_YiDig_t *hmiVal - HMI values
*  XY_reply_t *parsedValues - current response string
* Return:
*  nothing (void)
*
*******************************************************************************/
JSMN_API int8_t ReportYiCamtoHMI(XY_JSparse_t *parsedReply, COMGS_YiDig_t *hmiVal, XY_reply_t *parsedValues )
{
   if (((parsedReply == NULL) || (hmiVal == NULL)) || (parsedValues == NULL))        // null passed return an error
        return -1;
   if (((parsedReply->f_msg_id) && (parsedReply->f_type) && (parsedReply->f_param)) && (parsedValues->rval==0))     // reply had no errors and contained msg, type and param
   {
      removeSubstr((char*) &parsedValues->type_str, "\"");                      // remove any " quote from the type_str replies
      switch (parsedValues->msg_id)
      {
              case XY_STATUS_CMD:                                               // 7u A Status update message
              if ((!strcmp((char*) &parsedValues->type_str,"battery")) || (!strcmp((char*) &parsedValues->type_str,"adapter")))     // battery
              {
                  removeSubstr((char*) &parsedValues->param_str, "\"");
                  hmiVal->Battpercent=atoi(parsedValues->param_str);
              }
              else if (!strcmp((char*) &parsedValues->type_str,"adapter_status"))    // cable connection
              {
                  removeSubstr((char*) &parsedValues->param_str, "\"");
                  hmiVal->CableConnection=atoi(parsedValues->param_str);
              }
              else if (!strcmp((char*) &parsedValues->type_str,"switch_to_rec_mode"))   // recording mode
              {
                  hmiVal->VidOrPho=0U;
              }
              else if (!strcmp((char*) &parsedValues->type_str,"switch_to_cap_modee"))
              {
                 hmiVal->VidOrPho=1U;
              }
              else if (!strcmp((char*) &parsedValues->type_str,"photo_taken"))      // photo taken
              {
                 strcpy((char*) &hmiVal->LastPicName, (char*) &parsedValues->param_str);
              }
              else if (!strcmp((char*) &parsedValues->type_str,"sd_card_status"))        // sd card state
              {
                 removeSubstr((char*) &parsedValues->param_str, "\"");
                 hmiVal->SDCardStatus=(!strcmp((char*) &parsedValues->param_str,"insert"));
              }
              else if (!strcmp((char*) &parsedValues->type_str,"CARD_REMOVED"))
              {
                 hmiVal->SDCardStatus=0U;
              }
              break;

              case XY_START_SESS_CMD:                                           // 257u Get Token message
              hmiVal->Token=atoi(parsedValues->param_str);
              break;

              case XY_SND_GET_TH_CMD:                                           // get thumbnail request
              removeSubstr((char*) &parsedValues->md5sum, "'");                 // remove the quotes
              removeSubstr((char*) &parsedValues->md5sum, "\"");                // remove the quotes documentation is either ?????
              if (parsedReply->f_md5sum)                                        // a new md5sum was returned
                 strcpy(hmiVal->md5sum,parsedValues->md5sum);                   // save the md5sum returned
              else
                 strcpy(hmiVal->md5sum,"getFileMd5 error\r");                   // return no md5sum returned

              if (parsedReply->f_size)                                          // a new size was returned
                 hmiVal->size = parsedValues->size;                             // save the size returned to the hmi
              else
                 hmiVal->size = 0U;                                             // make file size returned zero
              break;

              case XY_SND_GET_MDA_CMD:                                          // get media info request
              if (parsedReply->f_size)                                          // a new size was returned
                 hmiVal->size = parsedValues->size;                             // save the size returned to the hmi
              else
                 hmiVal->size = 0U;                                             // make file size returned zero

              removeSubstr((char*) &parsedValues->resolution, "\"");            // remove the quotes
              if (parsedReply->f_resol)                                         // a new md5sum was returned
                 strcpy(hmiVal->resolution,parsedValues->resolution);           // save the md5sum returned
              else
                 strcpy(hmiVal->resolution,"error\r");                          // return no resolution returned
              break;

             case XY_GET_LAST_PHO_CMD:                                          // what was last photo info request
             removeSubstr((char*) &parsedValues->param_str, "\"");               // remove the quotes
             if ((strlen(parsedValues->param_str) <= sizeof(hmiVal->LastPicName)) && parsedReply->f_param)
                strcpy(hmiVal->LastPicName,parsedValues->param_str);           // then copy param returned to the file name string
             else
                strcpy(hmiVal->LastPicName,"invalid\r");
             break;

             //case XY_SND_GET_SPC_CMD:                                           // a call for SD Card space
             //removeSubstr((char*) &parsedValues->param_str, "\"");              // remove the quotes
             //if ((parsedReply->f_param) && (XYRequest==(52u+1u)))               // request plus one
             //{
             //   hmiVal->free_space = atol(parsedValues->param_str);             // put the free space on the SD card in bytes into the display register
             //}
             //else if ((parsedReply->f_param) && (XYRequest==(0u)))              // request plus one
             //{
             //   hmiVal->total_space = atol(parsedValues->param_str);            // put the total space on the SD card in bytes into the display register
             //}
             //break;

             //case XY_SND_GET_SPC_CMD:                                         // 5u SD Card Space free message now done in XY_Check_Return_Code can be free or total
             //removeSubstr((char*) &parsedValues->param_str, "\"");
             //hmiVal->SDCardSpace=atol(parsedValues->param_str);
             //break;

             default:                                                           // Not known yet
             break;
       }
   }
   else if (parsedReply->f_rval)                                                // if an rval was returned then write it back to the active structure
   {
       hmiVal->rval=parsedValues->rval;                                         // set the current rval to the one found in the reply
   }
   return 1;
}
#endif /* JSMN_HEADER */

#if defined(GEO_JSON_USED)
#define POLY_NUM_OF_END_BRACKETS 7u
#define LS_NUM_OF_END_BRACKETS 5u
#define GEO_JSON_POINT 1u
#define GEO_JSON_POLYGON 2u
#define GEO_JSON_LINESTRING 3u
#define GEO_JSON_MULTI_LINESTRING 4u
#define GEO_JSON_MULTI_POLYGON 5u
static int8_t parseGeoJSonArrObj(char* p, geoCoord_Square_t *pReply)
{
    uint8_t idx=0u;
    uint8_t endCount=1u;
    uint8_t field=1u;
    uint8_t beginParse=0u;
    uint8_t offset=0u;

   // no string or unsupported types
   if (((!p) || (pReply->geoTyp==GEO_JSON_MULTI_LINESTRING)) || (pReply->geoTyp==GEO_JSON_MULTI_POLYGON)) return 0u;
            
    while (1u)                                                                  // forever unless you get a return (complete or empty)
    {
        switch (*p)
        {
            case '\0':                                                          // null char
            return 0u;                                                          // error should be ended

            case ' ': case '\t': case '\n': case '\r':                          // skip any of these
            case '{':  case '}':                                                // also these
            p++;                                                                // increment counter pointer in string until its max the roll over (shouldnt count so high)
            break;

            case ',':                                                           // finalise field
            p++;
            offset=++offset % 10u;                                              // count the next field in the structure until the max (allow up to 4 values in array)
            break;

            case '[':                                                           // start reading the array values
            p++;
            beginParse=1u;
            break;

            case ']':                                                           // end the read if we have the right ammount of end brackets ]
            if (((endCount >= LS_NUM_OF_END_BRACKETS) && (pReply->geoTyp==GEO_JSON_LINESTRING)) || ((endCount >= POLY_NUM_OF_END_BRACKETS) && (pReply->geoTyp==GEO_JSON_POLYGON)))
            {
               offset=8u;                                                       // last value
            }
            else if (pReply->geoTyp==GEO_JSON_POINT)
            {
               offset=8u;                                                       // last value
            }
            endCount=++endCount % 10u;                                          // count ] brackets up to 10 (more than max)
            p++;
                 break;

            default:                                                            // any other characture
            if (!p) return 0u;                                                  // no more chars
            if (beginParse==1u)
            {
               field=offset+field;                                              // field plus the offset
               switch(field)
               {
                   case 1u:                                                     // collect the string
                   pReply->geoJSONCoord[idx] = *p;
                   //p=p++;
                   idx=++idx % UINT8_MAX;
                   break;

                   case 2u:                                                     // write the first value
                   pReply->geoJSONCoord[idx] = '\0';
                   pReply->long1 = atof(&pReply->geoJSONCoord);
                   field=1u;
                   idx=0u;
                   break;

                   case 3u:                                                     // write the second value
                   pReply->geoJSONCoord[idx] = '\0';
                   pReply->lat1 = atof(&pReply->geoJSONCoord);
                   field=1u;
                   idx=0u;
                   break;

                   case 4u:                                                     // write the third value
                   pReply->geoJSONCoord[idx] = '\0';
                   pReply->long2 = atof(&pReply->geoJSONCoord);
                   field=1u;
                   idx=0u;
                   break;

                   case 5u:                                                     // write the fouth value
                   pReply->geoJSONCoord[idx] = '\0';
                   pReply->lat2 = atof(&pReply->geoJSONCoord);
                   field=1u;
                   idx=0u;
                   break;

                   case 6u:                                                     // write the fifth value
                   pReply->geoJSONCoord[idx] = '\0';
                   pReply->long3 = atof(&pReply->geoJSONCoord);
                   field=1u;
                   idx=0u;
                   break;

                   case 7u:                                                     // write the sixth value
                   pReply->geoJSONCoord[idx] = '\0';
                   pReply->lat3 = atof(&pReply->geoJSONCoord);
                   field=1u;
                   idx=0u;
                   break;

                   case 8u:                                                     // write the seventh value
                   pReply->geoJSONCoord[idx] = '\0';
                   pReply->long4 = atof(&pReply->geoJSONCoord);
                   field=1u;
                   idx=0u;
                   break;

                   case 9u:                                                     // write the eigth value
                   pReply->geoJSONCoord[idx] = '\0';
                   pReply->lat4 = atof(&pReply->geoJSONCoord);
                   field=1u;
                   idx=0u;
                   return (1u);
                   break;

                   default:                                                     // shouldnt be here
                   return(0u);
                   break;
                }
            }
            p++;                                                                // advance to next char in JSON array string
            break;
        }
    }
}
JSMN_API int8_t geoJSON_parse_reply(unsigned char *JSON_STRING, geoCoord_Square_t *pReply)
{
  int16_t i;
  int16_t r;
  //int16_t j;
  unsigned char destString[32u];                                                // string to hold the values after the keys
  jsmn_parser p;                                                                // jasmine parser type
  jsmntok_t t[128u];                                                            /* We expect no more than 128 tokens */
  int16_t tagsFound=0U;
  uint8_t collect=0U;                                                           // collection step for configuration read

  if (pReply == NULL )                                                          // null passed return an error
    return tagsFound;

  jsmn_init(&p);
  // r = jsmn_parse( &p, JSON_STRING, strlen(JSON_STRING), t, sizeof(t) / sizeof(t[0]) );
  r = jsmn_parse( &p, (const char *)JSON_STRING, strlen(JSON_STRING), t, sizeof(t) / sizeof(t[0]) );

  if (r < 0)
  {
    tagsFound |= _XY_ERROR;
    return tagsFound;                                                           //printf("Failed to parse JSON: %d\n", r);
  }

  if (r < 1 || t[0].type != JSMN_OBJECT)                                        /* Assume the top-level element is an object */
  {
    tagsFound |= _XY_ERROR;
    return tagsFound;                                                           //printf("Object expected\n");
  }

  for (i = 1; i < r; i++)                                                       /* Loop over all keys of the root object */
  {
     if (jsoneq(JSON_STRING, &t[i], "type") == 0)                               // string was type then check its type for the next parse
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the type into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (!strcmp((char*) &destString,"Point"))                               // if its a single point type
        {
           pReply->geoTyp=GEO_JSON_POINT;
        }
        else if (!strcmp((char*) &destString,"Polygon"))                        // if its a polygon point type (here we support a rectangle)
        {
           pReply->geoTyp=GEO_JSON_POLYGON;
        }
        else if (!strcmp((char*) &destString,"LineString"))                     // if its a line string type
        {
           pReply->geoTyp=GEO_JSON_LINESTRING;
        }
        else if (!strcmp((char*) &destString,"MultiLineString"))                // if its a multiline string type
        {
            pReply->geoTyp=GEO_JSON_MULTI_LINESTRING;
        }
        else if (!strcmp((char*) &destString,"MultiPolygon"))                   // if its a multi polygon point type
        {
            pReply->geoTyp=GEO_JSON_MULTI_POLYGON;
        }
        i++;
     }
     else if (jsoneq(JSON_STRING, &t[i], "coordinates") == 0)                   // look for the properties key
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        return(parseGeoJSonArrObj(destString, pReply));                         // now get the data from the array
     }
  }
  return tagsFound;                                                             // Returns 0 if nothing found otherwise bit set for each tag found

}
#endif                                                                          // end======================== geoJSON ========================

#if (defined(UBIBOT_DEVICE_USED) || defined(UBIBOT_SERVER_USED))

#define _uBi_ERROR (1u<<0u)                                // error was found
#define _uBi_DATE (1u<<1u)                                 // date associated with the values
#define _uBi_DEV_VAL1 (1u<<2u)                             // for device the value was found
#define _uBi_SVR_AVG1 (1u<<2u)                             // for server the average was returned
#define _uBi_SVR_SD1 (1u<<4u)                              // for server the standard deviation was returned
#define _uBi_SVR_SUM1 (1u<<3u)                             // for server sum was returned
#define _uBi_SVR_COUNT1 (1u<<5u)                           // for server count was returned
#define _uBi_SVR_MAX1 (1u<<6u)                             // for server max was returned
#define _uBi_SVR_MIN1 (1u<<7u)                             // for server min was returned

#define UBI_DEVICE_MSG 0u                                  // device message
#define UBI_SERVER_MSG 1u                                  // server message

JSMN_API uint64_t uBiJSON_parse_reply(unsigned char *JSON_STRING, uBiReadFieldData_t *pReply)
{
  int8_t i;                                                                     // loop iterators
  int8_t j;
  int16_t r;                                                                    // return from jasmn parse calls
  int16_t ret1;
  unsigned char destString[32u];                                                // string to hold the values after the keys
  jsmn_parser p;                                                                // jasmine parser type first parse
  jsmn_parser q;                                                                // jasmine parser type second parse
  jsmntok_t t[128u];                                                            /* We expect no more than 128 tokens */
  jsmntok_t token1[128u];                                                       /* We expect no more than 128 tokens */
  int16_t tagsFound=0U;                                                         // return what we found in the parse

  if (pReply == NULL )                                                          // null passed return an error
    return tagsFound;

  jsmn_init(&p);
  r = jsmn_parse( &p, (const char *)JSON_STRING, strlen(JSON_STRING), t, sizeof(t) / sizeof(t[0u]) );

  if (r < 0)
  {
    tagsFound |= _uBi_ERROR;
    return tagsFound;                                                           //printf("Failed to parse JSON: %d\n", r);
  }

  if (r < 1 || t[0].type != JSMN_OBJECT)                                        /* Assume the top-level element is an object */
  {
    tagsFound |= _uBi_ERROR;
    return tagsFound;                                                           //printf("Object expected\n");
  }

  for (i = 1; i < r; i++)                                                       /* Loop over all keys of the root object */
  {
     if (jsoneq(JSON_STRING, &t[i], "created_at") == 0)                         // string was type then check its type for the next parse
     {
        strncpy((char*) pReply->ubiDate, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the created at into destString
        pReply->ubiDate[t[i+1].end - t[i+1].start] = '\0';                      // Properly terminate the string with a NULL character
        i++;
     }
     else if (jsoneq(JSON_STRING, &t[i], pReply->field1Desc) == 0)              // look for the description assocaited with field1 (field1 or temp or Temperature etc)
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (pReply->jSonTyp == UBI_DEVICE_MSG)                                  // This is a device message
        {
           pReply->temperature_celcius.deviceVal = atof(&destString);           // now convert to float the returned string from the parse
        }
        else if (pReply->jSonTyp == UBI_SERVER_MSG)
        {
           jsmn_init(&q);
           ret1 = jsmn_parse( &q, (const char *)JSON_STRING, strlen(JSON_STRING), token1, sizeof(token1) / sizeof(token1[0u]) );
           if (r < 0)
           {
              tagsFound |= _uBi_ERROR;
              return tagsFound;                                                 //printf("Failed to parse JSON: %d\n", r);
           }
           for (j = 1; j < ret1; j++)                                           /* Loop over all keys of the root object */
           {
              if (jsoneq(JSON_STRING, &token1[j], "sum") == 0)                  // string was type then check its type for the next parse
              {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->temperature_celcius.serverVal.sum = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "avg") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->temperature_celcius.serverVal.avg = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "count") == 0)          // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->temperature_celcius.serverVal.count = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "sd") == 0)             // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->temperature_celcius.serverVal.sd = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "max") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->temperature_celcius.serverVal.max = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "min") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->temperature_celcius.serverVal.min = atof(&destString);
               }
           }
        }
     }
     else if (jsoneq(JSON_STRING, &t[i], pReply->field2Desc) == 0)              // look for the description assocaited with field2 (field2 or Humidity etc)
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (pReply->jSonTyp == UBI_DEVICE_MSG)                                  // This is a device message
        {
           pReply->humidity.deviceVal = atof(&destString);                      // now convert to float the returned string from the parse
        }
        else if (pReply->jSonTyp == UBI_SERVER_MSG)
        {
           jsmn_init(&q);
           ret1 = jsmn_parse( &q, (const char *)JSON_STRING, strlen(JSON_STRING), token1, sizeof(token1) / sizeof(token1[0u]) );
           if (r < 0)
           {
              tagsFound |= _uBi_ERROR;
              return tagsFound;                                                 //printf("Failed to parse JSON: %d\n", r);
           }
           for (j = 1; j < ret1; j++)                                           /* Loop over all keys of the root object */
           {
              if (jsoneq(JSON_STRING, &token1[j], "sum") == 0)                  // string was type then check its type for the next parse
              {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->humidity.serverVal.sum = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "avg") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->humidity.serverVal.avg = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "count") == 0)          // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->humidity.serverVal.count = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "sd") == 0)             // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->humidity.serverVal.sd = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "max") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->humidity.serverVal.max = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "min") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->humidity.serverVal.min = atof(&destString);
               }
           }
        }
     }
     else if (jsoneq(JSON_STRING, &t[i], pReply->field3Desc) == 0)              // look for the description assocaited with field2 (field2 or Humidity etc)
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (pReply->jSonTyp == UBI_DEVICE_MSG)                                  // This is a device message
        {
           pReply->light_lux.deviceVal = atof(&destString);                     // now convert to float the returned string from the parse
        }
        else if (pReply->jSonTyp == UBI_SERVER_MSG)
        {
           jsmn_init(&q);
           ret1 = jsmn_parse( &q, (const char *)JSON_STRING, strlen(JSON_STRING), token1, sizeof(token1) / sizeof(token1[0u]) );
           if (r < 0)
           {
              tagsFound |= _uBi_ERROR;
              return tagsFound;                                                 //printf("Failed to parse JSON: %d\n", r);
           }
           for (j = 1; j < ret1; j++)                                           /* Loop over all keys of the root object */
           {
              if (jsoneq(JSON_STRING, &token1[j], "sum") == 0)                  // string was type then check its type for the next parse
              {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->light_lux.serverVal.sum = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "avg") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->light_lux.serverVal.avg = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "count") == 0)          // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->light_lux.serverVal.count = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "sd") == 0)             // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->light_lux.serverVal.sd = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "max") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->light_lux.serverVal.max = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "min") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->light_lux.serverVal.min = atof(&destString);
               }
           }
        }
     }
     else if (jsoneq(JSON_STRING, &t[i], pReply->field4Desc) == 0)              // look for the description assocaited with field4 (field4 or voltage etc)
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (pReply->jSonTyp == UBI_DEVICE_MSG)                                  // This is a device message
        {
           pReply->voltage.deviceVal = atof(&destString);                       // now convert to float the returned string from the parse
        }
        else if (pReply->jSonTyp == UBI_SERVER_MSG)
        {
           jsmn_init(&q);
           ret1 = jsmn_parse( &q, (const char *)JSON_STRING, strlen(JSON_STRING), token1, sizeof(token1) / sizeof(token1[0u]) );
           if (r < 0)
           {
              tagsFound |= _uBi_ERROR;
              return tagsFound;                                                 //printf("Failed to parse JSON: %d\n", r);
           }
           for (j = 1; j < ret1; j++)                                           /* Loop over all keys of the root object */
           {
              if (jsoneq(JSON_STRING, &token1[j], "sum") == 0)                  // string was type then check its type for the next parse
              {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->voltage.serverVal.sum = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "avg") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->voltage.serverVal.avg = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "count") == 0)          // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->voltage.serverVal.count = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "sd") == 0)             // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->voltage.serverVal.sd = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "max") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->voltage.serverVal.max = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "min") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->voltage.serverVal.min = atof(&destString);
               }
           }
        }
     }
     else if (jsoneq(JSON_STRING, &t[i], pReply->field5Desc) == 0)              // look for the description assocaited with field4 (field4 or voltage etc)
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (pReply->jSonTyp == UBI_DEVICE_MSG)                                  // This is a device message
        {
           pReply->voltage.deviceVal = atof(&destString);                       // now convert to float the returned string from the parse
        }
        else if (pReply->jSonTyp == UBI_SERVER_MSG)
        {
           jsmn_init(&q);
           ret1 = jsmn_parse( &q, (const char *)JSON_STRING, strlen(JSON_STRING), token1, sizeof(token1) / sizeof(token1[0u]) );
           if (r < 0)
           {
              tagsFound |= _uBi_ERROR;
              return tagsFound;                                                 //printf("Failed to parse JSON: %d\n", r);
           }
           for (j = 1; j < ret1; j++)                                           /* Loop over all keys of the root object */
           {
              if (jsoneq(JSON_STRING, &token1[j], "sum") == 0)                  // string was type then check its type for the next parse
              {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->vibration.serverVal.sum = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "avg") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->vibration.serverVal.avg = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "count") == 0)          // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->vibration.serverVal.count = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "sd") == 0)             // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->vibration.serverVal.sd = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "max") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->vibration.serverVal.max = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "min") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->vibration.serverVal.min = atof(&destString);
               }
           }
        }
     }
     else if (jsoneq(JSON_STRING, &t[i], pReply->field6Desc) == 0)              // look for the description assocaited with field4 (field4 or voltage etc)
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (pReply->jSonTyp == UBI_DEVICE_MSG)                                  // This is a device message
        {
           pReply->voltage.deviceVal = atof(&destString);                       // now convert to float the returned string from the parse
        }
        else if (pReply->jSonTyp == UBI_SERVER_MSG)
        {
           jsmn_init(&q);
           ret1 = jsmn_parse( &q, (const char *)JSON_STRING, strlen(JSON_STRING), token1, sizeof(token1) / sizeof(token1[0u]) );
           if (r < 0)
           {
              tagsFound |= _uBi_ERROR;
              return tagsFound;                                                 //printf("Failed to parse JSON: %d\n", r);
           }
           for (j = 1; j < ret1; j++)                                           /* Loop over all keys of the root object */
           {
              if (jsoneq(JSON_STRING, &token1[j], "sum") == 0)                  // string was type then check its type for the next parse
              {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->knocks.serverVal.sum = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "avg") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->knocks.serverVal.avg = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "count") == 0)          // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->knocks.serverVal.count = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "sd") == 0)             // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->knocks.serverVal.sd = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "max") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->knocks.serverVal.max = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "min") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->knocks.serverVal.min = atof(&destString);
               }
           }
        }
     }
     else if (jsoneq(JSON_STRING, &t[i], pReply->field7Desc) == 0)              // look for the description assocaited with field4 (field4 or voltage etc)
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (pReply->jSonTyp == UBI_DEVICE_MSG)                                  // This is a device message
        {
           pReply->magnetic_switch.deviceVal = atof(&destString);               // now convert to float the returned string from the parse
        }
        else if (pReply->jSonTyp == UBI_SERVER_MSG)
        {
           jsmn_init(&q);
           ret1 = jsmn_parse( &q, (const char *)JSON_STRING, strlen(JSON_STRING), token1, sizeof(token1) / sizeof(token1[0u]) );
           if (r < 0)
           {
              tagsFound |= _uBi_ERROR;
              return tagsFound;                                                 //printf("Failed to parse JSON: %d\n", r);
           }
           for (j = 1; j < ret1; j++)                                           /* Loop over all keys of the root object */
           {
              if (jsoneq(JSON_STRING, &token1[j], "sum") == 0)                  // string was type then check its type for the next parse
              {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->magnetic_switch.serverVal.sum = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "avg") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->magnetic_switch.serverVal.avg = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "count") == 0)          // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->magnetic_switch.serverVal.count = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "sd") == 0)             // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->magnetic_switch.serverVal.sd = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "max") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->magnetic_switch.serverVal.max = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "min") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->magnetic_switch.serverVal.min = atof(&destString);
               }
           }
        }
     }
     else if (jsoneq(JSON_STRING, &t[i], pReply->field8Desc) == 0)              // look for the description assocaited with field4 (field4 or voltage etc)
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (pReply->jSonTyp == UBI_DEVICE_MSG)                                  // This is a device message
        {
           pReply->soil_absolute_moisture.deviceVal = atof(&destString);        // now convert to float the returned string from the parse
        }
        else if (pReply->jSonTyp == UBI_SERVER_MSG)
        {
           jsmn_init(&q);
           ret1 = jsmn_parse( &q, (const char *)JSON_STRING, strlen(JSON_STRING), token1, sizeof(token1) / sizeof(token1[0u]) );
           if (r < 0)
           {
              tagsFound |= _uBi_ERROR;
              return tagsFound;                                                 //printf("Failed to parse JSON: %d\n", r);
           }
           for (j = 1; j < ret1; j++)                                           /* Loop over all keys of the root object */
           {
              if (jsoneq(JSON_STRING, &token1[j], "sum") == 0)                  // string was type then check its type for the next parse
              {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->soil_absolute_moisture.serverVal.sum = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "avg") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->soil_absolute_moisture.serverVal.avg = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "count") == 0)          // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->soil_absolute_moisture.serverVal.count = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "sd") == 0)             // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->soil_absolute_moisture.serverVal.sd = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "max") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->soil_absolute_moisture.serverVal.max = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "min") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->soil_absolute_moisture.serverVal.min = atof(&destString);
               }
           }
        }
     }
     else if (jsoneq(JSON_STRING, &t[i], pReply->field9Desc) == 0)              // look for the description assocaited with field4 (field4 or voltage etc)
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (pReply->jSonTyp == UBI_DEVICE_MSG)                                  // This is a device message
        {
           pReply->ext_temp_probe_celcuis.deviceVal = atof(&destString);        // now convert to float the returned string from the parse
        }
        else if (pReply->jSonTyp == UBI_SERVER_MSG)
        {
           jsmn_init(&q);
           ret1 = jsmn_parse( &q, (const char *)JSON_STRING, strlen(JSON_STRING), token1, sizeof(token1) / sizeof(token1[0u]) );
           if (r < 0)
           {
              tagsFound |= _uBi_ERROR;
              return tagsFound;                                                 //printf("Failed to parse JSON: %d\n", r);
           }
           for (j = 1; j < ret1; j++)                                           /* Loop over all keys of the root object */
           {
              if (jsoneq(JSON_STRING, &token1[j], "sum") == 0)                  // string was type then check its type for the next parse
              {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->ext_temp_probe_celcuis.serverVal.sum = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "avg") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->ext_temp_probe_celcuis.serverVal.avg = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "count") == 0)          // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->ext_temp_probe_celcuis.serverVal.count = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "sd") == 0)             // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->ext_temp_probe_celcuis.serverVal.sd = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "max") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->ext_temp_probe_celcuis.serverVal.max = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "min") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->ext_temp_probe_celcuis.serverVal.min = atof(&destString);
               }
           }
        }
     }
     else if (jsoneq(JSON_STRING, &t[i], pReply->field10Desc) == 0)              // look for the description assocaited with field10 (field4 or voltage etc)
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (pReply->jSonTyp == UBI_DEVICE_MSG)                                  // This is a device message
        {
           pReply->reed_sensor.deviceVal = atof(&destString);                   // now convert to float the returned string from the parse
        }
        else if (pReply->jSonTyp == UBI_SERVER_MSG)
        {
           jsmn_init(&q);
           ret1 = jsmn_parse( &q, (const char *)JSON_STRING, strlen(JSON_STRING), token1, sizeof(token1) / sizeof(token1[0u]) );
           if (r < 0)
           {
              tagsFound |= _uBi_ERROR;
              return tagsFound;                                                 //printf("Failed to parse JSON: %d\n", r);
           }
           for (j = 1; j < ret1; j++)                                           /* Loop over all keys of the root object */
           {
              if (jsoneq(JSON_STRING, &token1[j], "sum") == 0)                  // string was type then check its type for the next parse
              {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->reed_sensor.serverVal.sum = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "avg") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->reed_sensor.serverVal.avg = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "count") == 0)          // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->reed_sensor.serverVal.count = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "sd") == 0)             // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->reed_sensor.serverVal.sd = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "max") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->reed_sensor.serverVal.max = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "min") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->reed_sensor.serverVal.min = atof(&destString);
               }
           }
        }
     }
     else if (jsoneq(JSON_STRING, &t[i], pReply->field11Desc) == 0)              // look for the description assocaited with field10 (field4 or voltage etc)
     {
        strncpy((char*) &destString, JSON_STRING + t[i+1].start, t[i+1].end - t[i+1].start);     // copy the value after the token into destString
        destString[t[i+1].end - t[i+1].start] = '\0';                           // Properly terminate the string with a NULL character
        if (pReply->jSonTyp == UBI_DEVICE_MSG)                                  // This is a device message
        {
           pReply->wifi_rssi.deviceVal = atof(&destString);                     // now convert to float the returned string from the parse
        }
        else if (pReply->jSonTyp == UBI_SERVER_MSG)                             //
        {
           jsmn_init(&q);
           ret1 = jsmn_parse( &q, (const char *)JSON_STRING, strlen(JSON_STRING), token1, sizeof(token1) / sizeof(token1[0u]) );
           if (r < 0)
           {
              tagsFound |= _uBi_ERROR;
              return tagsFound;                                                 //printf("Failed to parse JSON: %d\n", r);
           }
           for (j = 1; j < ret1; j++)                                           /* Loop over all keys of the root object */
           {
              if (jsoneq(JSON_STRING, &token1[j], "sum") == 0)                  // string was type then check its type for the next parse
              {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->wifi_rssi.serverVal.sum = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "avg") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->wifi_rssi.serverVal.avg = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "count") == 0)          // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->wifi_rssi.serverVal.count = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "sd") == 0)             // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->wifi_rssi.serverVal.sd = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "max") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->wifi_rssi.serverVal.max = atof(&destString);
               }
               else if (jsoneq(JSON_STRING, &token1[j], "min") == 0)            // string was type then check its type for the next parse
               {
                  strncpy((char*) &destString, JSON_STRING + token1[j+1].start, token1[j+1].end - token1[j+1].start);     // copy the value after the created at into destString
                  destString[token1[j+1].end - token1[j+1].start] = '\0';       // Properly terminate the string with a NULL character
                  j++;
                  pReply->wifi_rssi.serverVal.min = atof(&destString);
               }
           }
        }
     }
  }
  return tagsFound;                                                             // Returns 0 if nothing found otherwise bit set for each tag found

}
#endif

#ifdef __cplusplus

}

#endif



#endif /* JSMN_H */