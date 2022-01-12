#ifndef ZREPSTR_H
#define ZREPSTR_H

#define ZREPSTR_API extern
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "lwLW3.h"

/*! Copyright (c) 2016-2019, Fehmi Noyan ISI fnoyanisi@yahoo.com
 * \brief Replaces a string with another one.
 *
 * \details Every occurance of the string given by the argument \p x contained
 *  in the first argument \p str is replaced by the string given by the
 * argument \p y.
 *
 * \param str pointer to the input string
 * \param x a NULL terminated string that is to be replaced
 * \param y a NULL terminated string that will replace the string pointed
 * by \p x
 *
 * \b Example
 * \code{.c}
 *
 * int main() {
 *      char str[] = "free as in free speech, not as in free beer";
 *      printf("%s\\n",zrepstr(str,"free","KLMN"));
 *      return 0;
 * }
 * \endcode
 *
 * \b Output
 * \code{.unparsed}
 * KLMN as in KLMN speech, not as in KLMN beer
 * \endcode
 *
 * \returns zrepstr function returns a pointer to the resulting string.
 *
 */

ZREPSTR_API char * zrepstr(char *str, const char *x, const char *y)
{
        char *p = str;
        int16_t tlen;
        int16_t clen;
        int16_t ylen;
        tlen = strlen(str);                                                        /* total length of the buffer */
        ylen = strlen((char *) y);                                                /* clen - copy length */
        
        while ((p = strstr(str,(char *) x))!= NULL)
        {
                /* litte bit of pointer arithmetic
                 *
                 *                     tlen       (str + tlen)
                 *   |-----------------------|
                 *   |                 |     |           |
                 *  str................p.............(p + ylen)
                 *   |<--------------->|<---->
                 *         (p - str)     (str + tlen - p)
                 */

                clen = ((p - str + ylen) <= tlen) ? ylen : (str + tlen - p);
                p = strncpy(p,(char *) y, clen);
                p += clen;
        }
        return str;
}

/* replace every occurance of string x with string y */
/*******************************************************************************
* Function Name: zstring_replace_str
********************************************************************************
* Summary:
* Replace word x with word y throughout str
* Parameters:
* char *str, const char *x, const char *y
* Return:
*  string
*
*******************************************************************************/
ZREPSTR_API char *zstring_replace_str(char *str, const char *x, const char *y)
{
    char *tmp_str = str, *tmp_x = (char *) x, *dummy_ptr = tmp_x, *tmp_y = (char *) y;
    int len_str=0, len_y=0, len_x=0;

    for(; *tmp_y; ++len_y, ++tmp_y)                                             /* string length */
        ;

    for(; *tmp_str; ++len_str, ++tmp_str)
        ;

    for(; *tmp_x; ++len_x, ++tmp_x)
        ;

    if (len_y >= len_str)                                                       /* Bounds check */
        return str;

    tmp_y = (char *) y;                                                         /* reset tmp pointers */
    tmp_x = (char *) x;

    for (tmp_str = str ; *tmp_str; ++tmp_str)
        if(*tmp_str == *tmp_x) {

            for (dummy_ptr=tmp_str; *dummy_ptr == *tmp_x; ++tmp_x, ++dummy_ptr) /* save tmp_str */
                if (*(tmp_x+1) == '\0' && ((dummy_ptr-str+len_y) < len_str)){
                /* Reached end of x, we got something to replace then!
                * Copy y only if there is enough room for it
                */
                    for(tmp_y= (char *) y; *tmp_y; ++tmp_y, ++tmp_str)
                        *tmp_str = *tmp_y;
            }

        tmp_x = (char *) x;                                                     /* reset tmp_x */
        }

    return str;
}

/*******************************************************************************
* Function Name: replace_char
********************************************************************************
* Summary:
* Substitute a char in, in a string str with char out
* Parameters:
* char* str, char in, char out
* Return:
*  string
*
*******************************************************************************/
ZREPSTR_API char* replace_char(char* str, char in, char out)
{
    char * p = str;
    if (NULL == str)
      return '\0';

    while(p != '\0')
    {
        if(*p == in)
            *p == out;
        ++p;
    }

    return str;
}
/*******************************************************************************
* Function Name: char_pos_in_string
********************************************************************************
* Summary:
* return the first or last position of a char in a srtring
* Parameters:
* char* str, char in, int8_t type (type=0=first type=1=last)
* Return:
*  int16_t position or -1 on error
*
*******************************************************************************/
ZREPSTR_API int16_t char_pos_in_string(char* str, char in, int8_t type)
{
   char *position_ptr;
   if (type == 0U)
   {                                                            // first
      position_ptr = strchr(str, in);
      return( (position_ptr == NULL ? -1 : position_ptr - str));
   }
   else if (type == 1U)
   {
      position_ptr = strrchr(str, in);
      return ((position_ptr == NULL ? -1 : position_ptr - str));
   }
   else
   {
      return -1;
   }
}

/*******************************************************************************
* Function Name: checkLW3Reply
********************************************************************************
* Summary:
* Checks LW3 reply message against a LW3 request message
* types are defined below as are the characters that preceed to be removed
* Parameters:
* char* strReplyLW3, char * strReqLW3, uint8_t subTyp, uint8_t remChar
* Return:
*  0 when they match
*
*******************************************************************************/
ZREPSTR_API int16_t checkLW3Reply(char* strReplyLW3, char * strReqLW3, uint8_t subTyp, uint8_t remChar)
{
  char * pSubStr;
  int16_t posOfChar;

  switch (subTyp)
  {
       case LW3_CALL_OBJ_TYP:                                                   // A successful method execution
       pSubStr=zrepstr(strReplyLW3,"CALL","mO");
       break;

       case LW3_SET_RW_TYP:                                                     // RW property that was set
       pSubStr=zrepstr(strReplyLW3,"SET","pW");
       break;

       case LW3_GET_RW_TYP:                                                     // RW property that was read
       pSubStr=zrepstr(strReplyLW3,"GET","pW");
       break;

       case LW3_GET_RO_TYP:                                                     // RO property that was read
       pSubStr=zrepstr(strReplyLW3,"GET","pR");
       break;

       default:
       break;
  }
  switch (remChar)
  {
        case LW3_BRACKET:                                                       // remove the bracket from the sent string
        posOfChar=char_pos_in_string(strReqLW3, '(', 0U);                       // find the start bracket
        strReqLW3[posOfChar] = '\0';                                            // terminate the string
        break;

        case LW3_EQUALS:                                                        // remove the equals from the reply
        posOfChar=char_pos_in_string(pSubStr, '=', 0U);                         // find the equals sign
        pSubStr[posOfChar] = '\0';                                              // terminate the string
        break;

        default:
        break;

  }
  return(strcmp(pSubStr,strReqLW3));                                            // returns zero for match
}
/*******************************************************************************
* Function Name: checkLW3ErrorCode
********************************************************************************
* Summary:
* Checks LW3 error code returned in the reply packet
*
* Parameters:
* char* strReplyLW3
* Return:
*  uint8_t code representing the error replied
*
*******************************************************************************/

ZREPSTR_API uint8_t checkLW3ErrorCode(char* strReplyLW3)
{
  int16_t posOfChar;
  
  posOfChar=char_pos_in_string(strReplyLW3,' ', 0U);
  strReplyLW3[posOfChar] = '\0';

  if (!strcmp(strReplyLW3,LW3_RESP_ERROR))
      return LW3_RESP_ERR_CODE;
  else if (!strcmp(strReplyLW3,LW3_RESP_FAIL))
      return LW3_RESP_FAIL_CODE;
  else if (!strcmp(strReplyLW3,LW3_RESP_MAN))
      return LW3_RESP_MAN_CODE;
  else if (!strcmp(strReplyLW3,LW3_RESP_NODE_ERR))
      return LW3_RESP_ERR_NODE_CODE;
  else if (!strcmp(strReplyLW3,LW3_RESP_NODE))
      return LW3_RESP_NODE_CODE;
  else if (!strcmp(strReplyLW3,LW3_RESP_METHOD))
      return LW3_RESP_METHOD_CODE;
  else if (!strcmp(strReplyLW3,LW3_RESP_CHNG))
      return LW3_RESP_CHG_CODE;
  else if (!strcmp(strReplyLW3,LW3_RESP_SUBS))
      return LW3_RESP_SUBS_CODE;
  else
      return LW3_RESP_NOMATCH_CODE;

}
/*******************************************************************************
* Function Name: getLW3Values
********************************************************************************
* Summary:
* Gets the values returned with an LW3 call
*
* Parameters:
* char* strReplyLW3, uint8_t delimeter, char *stringVals[]
* Return:
*  0 for error or the number of values found
*
*******************************************************************************/

ZREPSTR_API uint8_t getLW3Values(char* strReplyLW3, uint8_t delimeter, char *stringVals[])
{
  int16_t posOfChar;
  unsigned char listOfVals[30U];
  char* token;
  char delim;
  uint8_t noOfVals=0U;

  posOfChar=char_pos_in_string(strReplyLW3,'=', 0U);
  if ((strlen(strReplyLW3)-posOfChar) <= sizeof(listOfVals))
  {
    strncpy(listOfVals,strReplyLW3+posOfChar,strlen(strReplyLW3)-posOfChar);    // creates a string with each value separated by a delimeter
    listOfVals[strlen(strReplyLW3)-posOfChar] = '\0';                           // Properly terminate the string with a NULL character
  }
  else
  {
    return 0U;
  }
  
  switch (delimeter)
  {
    case 1u:                                                                    // ; is the delimter
    delim=';';
    break;

    case 2u:                                                                    // , is the delimter
    delim=',';
    break;

    case 0u:                                                                    // no delimeter
    strcpy(stringVals[0U],listOfVals);
    return 1U;
    
    default:                                                                    // not supported as yet
    return 0U;
  }
  token = strtok(listOfVals,(char *) delim);                                    // split by the delimter

  while ((token != NULL) && (noOfVals <= LW3_MAX_TOKENS))                       // for each token
  {
     noOfVals = ++noOfVals % UINT8_MAX;                                         // increment counter
     strcpy(stringVals[noOfVals-1U],token);                                     // copy to string array the value as a string
     token = strtok(NULL,(char *) delim);                                       // split by delimeter for the next string
  }

  return noOfVals;

}

/*******************************************************************************
* Function Name: checkLW3StatusWord
********************************************************************************
* Summary:
* Checks a LW3 status word returned
*
* Parameters:
* char* strReplyLW3
* Return:
*  0 for error or 1 for the status object being successfully updated
*
*******************************************************************************/

ZREPSTR_API uint8_t checkLW3StatusWord(char* strReplyLW3, LW3Status_t *statWord)
{
   char firstLetter;
   unsigned char wordVal[8U];
   uint32_t uintVal;
   uint8_t connectStat;

   firstLetter=strReplyLW3[0];

   switch (firstLetter)
   {
        case 'T':
        statWord->muted = 0U;
        statWord->locked = 0U;
        break;

        case 'L':
        statWord->muted = 0U;
        statWord->locked = 1U;
        break;

        case 'M':
        statWord->muted = 1U;
        statWord->locked = 0U;
        break;

        case 'U':
        statWord->muted = 1U;
        statWord->locked = 1U;
        break;

        default:
        return 0U;
   }

   if ((strlen(strReplyLW3)-1U) <= sizeof(wordVal))
   {
      strcpy(wordVal,strReplyLW3+1U);
      uintVal=hex2int( wordVal ) & 0x000FU;
          connectStat=uintVal & 0x0003U;
          switch (connectStat)
          {
                 case 3u:
                 statWord->connect=1U;
                 break;

                 case 2u:
                 statWord->connect=0U;
                 break;

                 default:
                 statWord->connect=2U;
                 break;
          }
          connectStat=((uintVal & 0x000CU) >> 2U);
          switch (connectStat)
          {
                 case 3u:
                 statWord->signal=1U;
                 break;

                 case 2u:
                 statWord->signal=0U;
                 break;

                 default:
                 statWord->signal=2U;
                 break;
          }
          return 1U;
   }
   else
   {
      return 0U;
   }

}
#endif