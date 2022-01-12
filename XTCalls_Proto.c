#include "definitions.h"
#include <stdint.h>
#include "XTCalls_Proto.h"
/*******************************************************************************
* Project Name:      XTCAlls display driver
* Version:           1.00
*
* Related Hardware:  SHENZHEN TOP SHINE ELECTRONIC CO., LTD. 2015.5.4 Version V2.0
*
********************************************************************************
* Theory of Operation:
*
* Writes to display a text or graphic multiple windows
*
********************************************************************************
* Copyright (2020), ACP Aviation
********************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Air Cam Pro LTD (ACP Aviation)
* ACP hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* ACP source code and derivative works for the sole purpose of creating
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with ACP.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* ACP reserves the right to make changes to the Software without notice.
* ACP does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. ACP does
* not authorize its products for use as critical components in any products
* where a malfunction or failure may reasonably be expected to result in
* significant injury or death ("High Risk Product"). By including ACP
* product in a High Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies ACP against all
* liability. Use of this Software may be limited by and subject to the
* applicable ACP license agreement.
*******************************************************************************/

uint8_t encodeXCallPayload( uint8_t * p_message, uint8_t n_bytes);
uint8_t decodeXCallPayload( uint8_t const * const p_message, uint8_t n_bytes, uint8_t * o_message );

/*-----------------------------------------------------------------------------
 *      encodeXCallPayload:  Encodes payload to have 0xAA infront of stx or etc or esc
 +       within the payload.
 *
 *
 *  Parameters: uint8_t const * const p_message, uint8_t n_bytes
 *  Return:     uint8_t new message length
 *----------------------------------------------------------------------------*/
uint8_t encodeXCallPayload( uint8_t * p_message, uint8_t n_bytes)
{
    uint8_t byte=0u;
    uint8_t posInMsg = 0u;
    uint8_t retVal = 0u;

    if (p_message == NULL)
    {
           retVal=0u;
    }
    else
    {
          for (byte = 0u;(byte < n_bytes); byte++)                              /* for each byte in the byte stream */
          {
             posInMsg = posInMsg + byte;
             switch (p_message[posInMsg])
             {
                   case 0xA5u:                                                  /* we got a STX then encode the data */
                   p_message[posInMsg] = 0xAAu;
                   posInMsg = ++posInMsg % UINT8_MAX;
                   p_message[posInMsg]= 0x05u;
                   break;

                   case 0xAEu:                                                  /* we got a ESC then encode the data */
                   p_message[posInMsg] = 0xAAu;
                   posInMsg = ++posInMsg % UINT8_MAX;
                   p_message[posInMsg]= 0x0Eu;
                   break;

                   case 0xAAu:                                                  /* we got a ETX then encode the data */
                   p_message[posInMsg] = 0xAAu;
                   posInMsg = ++posInMsg % UINT8_MAX;
                   p_message[posInMsg]= 0x0Au;
                   break;

                   default:
                   break;
              }
          }
          retVal = posInMsg;                                                    /* return new message length */
    }
    return retVal;
}

/*-----------------------------------------------------------------------------
 *      decodeXCallPayload:  Parse payload to have 0xAA infront removed and 0xA0 added
 +       within the payload.
 *
 *
 *  Parameters: uint8_t const * const p_message, uint8_t n_bytes
 *  Return:     uint8_t new message length
 *----------------------------------------------------------------------------*/
uint8_t decodeXCallPayload( uint8_t const * const p_message, uint8_t n_bytes, uint8_t * o_message )
{
    uint8_t byte=0u;
    uint8_t posInMsg = 0u;
    uint8_t add_A0 = false;
    uint8_t retVal = 0u;
    
    if (p_message == NULL)
    {
      retVal=0u;
    }
    else
    {
       for (byte = 0u;(byte < n_bytes); byte++)                                 /* for each byte recieved */
       {
           switch (p_message[posInMsg])
           {
              case 0xAAu:                                                       /* data was encoded then strip and convert the next characture */
              add_A0 = true;
              break;

              default:
              if (add_A0 == true)
              {
                  o_message[posInMsg] = p_message[byte] | 0xA0u;                /* convert characture in byte stream back to original */
                  add_A0 = false;
              }
              else
              {
                  o_message[posInMsg] = p_message[byte];                        /* read byte stream as normal */
              }
              posInMsg = ++posInMsg % UINT8_MAX;                                /* increment position in message recieved output array */
              break;
           }
       }
       retVal = posInMsg;
    }
    return retVal;
}