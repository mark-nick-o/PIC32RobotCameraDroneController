#include "definitions.h"
#if defined(J1939_USED)
//    J1939 ================== CAnBus functions ==================================
//
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
//
#include "J1939_CAN.h"
#include <stdint.h>
/*#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> */
#ifndef bool
#define bool uint8_t
#endif
#ifndef size_t
#define size_t uint32_t
#endif

#ifdef _CPU_BIG_ENDIAN
#define HOST_IS_BIG_ENDIAN
#endif
#include "cpu_endian_defs.h"                                                    // endian definitions

#include "gc_events.h"
/* #include "mmc_file_handler.h" */

/* %%%%%%%%%%%%%%%%%%%%%% Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  */

void Init_J1939_Port( unsigned char *Can_Init_Flags, unsigned char *Can_Send_Flags, unsigned char *Can_Rcv_Flags, char *FIFObuffers);
void j1939GetBuf( int16_t index, const unsigned char* buf, j1939_Data_Type_e j1939Type, float64_t precision, float64_t offset, float64_t def, float64_t *value );
void CanIdToj1939(uint32_t id, j1939_Control_t * j1939Obj);
void j1939GetBuff( int16_t *index, const unsigned char* buf, j1939_Data_Type_e j1939Type, float64_t precision, uint32_t maxVal, j1939_data_obj_t *value );
void j1939SetBuff( int16_t *index, const unsigned char* buf, j1939_Data_Type_e j1939Type, float64_t precision, uint32_t maxVal, j1939_data_obj_t *value );
uint32_t j1939toCanID(j1939_Control_t * j1939ConObj);
#if defined(LINAK_ACTUATOR_USED)
uint32_t WriteJ1939LinakGenMsg(uint8_t PortNo, j1939_Control_t * j1939ConObj);
uint8_t ParseJ1939LinakStatMsg( linak_status_t *J1939Obj, unsigned char *j1939RcvBuf );
#endif
#if defined(APEM_JOYSTCK_USED)
int8_t ReadApemJoyEJM1( CAN_j1939_ejm1_t *JoyValue, uint8_t offset );
int8_t ReadApemJoyBJM1( CAN_j1939_bjm1_t *JoyValue, uint8_t offset );
#endif
uint8_t readJ1939CANBusMsg( j1939_Control_t * j1939Obj, int8_t PortNo);
uint8_t PollJ1939CANBusMsg( j1939_Control_t * j1939Obj, int8_t PortNo );

 /*-----------------------------------------------------------------------------
 *      Init_J1939_Port():  J1939 message port initialization
 *
 *  Parameters: unsigned char *Can_Init_Flags, unsigned char *Can_Send_Flags, 
 *              unsigned char *Can_Rcv_Flags, char *FIFObuffers
 *
 *  Return:     nothing
 *
 *----------------------------------------------------------------------------*/
void Init_J1939_Port(unsigned char *Can_Init_Flags, unsigned char *Can_Send_Flags, unsigned char *Can_Rcv_Flags, char *FIFObuffers)
{
  Can_Init_Flags = 0u;                                                          //
  Can_Send_Flags = 0u;                                                          // clear flags
  Can_Rcv_Flags  = 0u;                                                          //

  *Can_Send_Flags = _CAN_TX_PRIORITY_0 &                                         //form value to be used
                   _CAN_TX_XTD_FRAME &                                          // with CANWrite
                   _CAN_TX_NO_RTR_FRAME;

  *Can_Init_Flags = _CAN_CONFIG_SAMPLE_THRICE &                                  // form value to be used
                   _CAN_CONFIG_PHSEG2_PRG_ON &                                  // with CANInit
                   _CAN_CONFIG_XTD_MSG &                                        //_CAN_CONFIG_DBL_BUFFER_ON &
                   _CAN_CONFIG_STD_MSG &
                   _CAN_CONFIG_LINE_FILTER_OFF;

  CAN2Initialize(1,4,5,3,1,*Can_Init_Flags);                                      //'OSC 40Mhz 500Kb/Sec
   // set CONFIGURATION mode
  CAN2SetOperationMode(_CAN_MODE_CONFIG,0xFFU);
  CAN2SetMask(_CAN_MASK_1,-1,_CAN_CONFIG_STD_MSG);                              // set all mask1 bits to ones
  CAN2SetMask(_CAN_MASK_2,-1,_CAN_CONFIG_STD_MSG);                              // set all mask2 bits to ones
  CAN2AssignBuffer(FIFObuffers);                                                //assign the buffers
  //configure rx fifo
  CAN2ConfigureFIFO(0u, 8u,_CAN_FIFO_RX & _CAN_FULL_MESSAGE);                   //RX buffer 8 messages deep
  //configure tx fifo
  CAN2ConfigureFIFO(1u, 8u,_CAN_FIFO_TX & _CAN_TX_PRIORITY_3 & _CAN_TX_NO_RTR_FRAME);// TX buffer 8 messages deep

  CAN2SetOperationMode(_CAN_MODE_NORMAL,0xFFu);                                 // set NORMAL mode
}

#if defined(APEM_JOYSTCK_USED)
 /*-----------------------------------------------------------------------------
 *      ReadApemJoyEJM1():  Read APEM Joystick EJM1 output J1939 message
 *
 *  Parameters: CAN_j1939_ejm1_t *JoyValue,  uint8_t offset
 *
 *  Return:     int8_t : -1 fault 
 *                       0 full copy 
 *                       >0 partial copy of returned number of bytes
 *
 *----------------------------------------------------------------------------*/
int8_t ReadApemJoyEJM1( CAN_j1939_ejm1_t *JoyValue, uint8_t offset )
{
   uint16_t msgFeedback=0u;
   uint16_t data_len=0u;
   uint16_t rx_flags = 0u;
   int8_t returnCode = -1;
   char dataValues[8u] = { 0u,0u,0u,0u,0u,0u,0u,0u };
   
   msgFeedback = CAN1Read(&JoyValue->PGN, dataValues, &data_len, &rx_flags);
   
   if (((msgFeedback == 0xFFFFU) && !(rx_flags & _CAN_RX_INVALID_MSG)) && !(rx_flags & _CAN_RX_OVERFLOW))
   {
      if (JoyValue->PGN == J1939_EJM1_JOY_PGN)                                     /* its the right PGM number ? */
      {
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            memcpy((void*)&JoyValue->priority,(void*) dataValues, 8u);          /* copy the 8 bytes in the frame */
            returnCode=0;
         }
         else
         {
            data_len = data_len % (sizeof(CAN_j1939_ejm1_t) - 4u);              /* ensure we havent returned an out of range stupid value for length */
#if defined(HOST_IS_BIG_ENDIAN)
            dataValues[2u] = LOAD16BE(dataValues+2u);                           /* if not work do a load16be then a store16le from a temporary variable */
            dataValues[4u] = LOAD16BE(dataValues+4u);
            dataValues[6u] = LOAD16BE(dataValues+6u);
#endif
            memcpy((void*)(&JoyValue->priority+offset),(void*) dataValues, data_len);  /* copy the received bytes in the frame */
            returnCode=(int8_t)data_len;                                        /* return the number of bytes we received in the partial */
         }
      }
   }
   return returnCode;
}

 /*-----------------------------------------------------------------------------
 *      ReadApemJoyBJM1():  Read APEM Joystick BJM1 output J1939 message
 *
 *  Parameters: CAN_j1939_bjm1_t *JoyValue,  uint8_t offset
 *
 *  Return:     int8_t : -1 fault
 *                       0 full copy
 *                       >0 partial copy of returned number of bytes
 *
 *----------------------------------------------------------------------------*/
int8_t ReadApemJoyBJM1( CAN_j1939_bjm1_t *JoyValue, uint8_t offset )
{
   uint16_t msgFeedback=0u;
   uint16_t data_len=0u;
   uint16_t rx_flags = 0u;
   int8_t returnCode = -1;
   char dataValues[8u] = { 0u,0u,0u,0u,0u,0u,0u,0u };

   msgFeedback = CAN1Read(&JoyValue->PGN, dataValues, &data_len, &rx_flags);
   if (((msgFeedback == 0xFFFFU) && !(rx_flags & _CAN_RX_INVALID_MSG)) && !(rx_flags & _CAN_RX_OVERFLOW))
   {
      if (JoyValue->PGN == J1939_BJM1_JOY_PGN)                                  /* its the right PGM number ? */
      {
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            memcpy((void*)&JoyValue->priority,(void*) dataValues, 8u);          /* copy the 8 bytes in the frame */
            returnCode=0;
         }
         else
         {
            data_len = data_len % (sizeof(CAN_j1939_ejm1_t) - 4u);              /* ensure we havent returned an out of range stupid value for length */
            memcpy((void*)(&JoyValue->priority+offset),(void*) dataValues, data_len);  /* copy the received bytes in the frame */
            returnCode=(int8_t)data_len;                                        /* return the number of bytes we received in the partial */
         }
      }
   }
   return returnCode;
}
#endif

 /*-----------------------------------------------------------------------------
 *      j1939GetBuf():  Get J1939 message
 *
 *  Parameters: int16_t index, const unsigned char* buf, j1939_Data_Type_e j1939Type, 
 *  float64_t precision, float64_t offset, float64_t def, float64_t *value
 *
 *  Return:     void
 *
 *----------------------------------------------------------------------------*/
void j1939GetBuf(int16_t index, const unsigned char* buf, j1939_Data_Type_e j1939Type, float64_t precision, float64_t offset, float64_t def, float64_t *value )
{

    if (buf==NULL)
      return;

    switch(j1939Type)
    {
        case typ2ByteInt:
        memcpy((void*)value,(void*)&buf[index], 2);
        index=index + 2;
        break;

        case typ2ByteUInt:
        memcpy((void*)value,(void*)&buf[index], 2);
#if defined(HOST_IS_BIG_ENDIAN)
        *value = SWAPINT16(*value);
#endif
        index=index + 2;
        break;

        case typ3ByteUInt:
        memcpy((void*)value,(void*)&buf[index], 3);
#if defined(HOST_IS_BIG_ENDIAN)
        *value = SWAPINT32(*value);
#endif
        index=index + 3;
        break;

        case typ4ByteUInt:
        memcpy((void*)value,(void*)&buf[index], 4);
#if defined(HOST_IS_BIG_ENDIAN)
        *value = SWAPINT32(*value);
#endif
        index=index + 4;
        break;

        case typ8ByteUInt:
        memcpy((void*)value,(void*)&buf[index], 8);
#if defined(HOST_IS_BIG_ENDIAN)
        *value = SWAPINT64(*value);
#endif
        index=index + 8;
        break;

        case typ1ByteDouble:
        memcpy((void*)value,(void*)&buf[index], 1);
        if (*value ==0x7f)
           *value =def;
        else
            *value= (*value + offset) * precision;
        index=index + 1;
        break;

        case typ1ByteUDouble:
        memcpy((void*)value,(void*)&buf[index], 1);
        if (*value==0xffU)
           *value=def;
        else
           *value= (*value + offset) * precision;
        index=index + 1;
        break;

        case typ2ByteDouble:
        memcpy((void*)value,(void*)&buf[index], 2);
        if (*value==0x7fff)
           *value=def;
        else
           *value= (*value + offset) * precision;
        index=index + 2;
        break;

        case typ2ByteUDouble:
        memcpy((void*)value,(void*)&buf[index], 2);
        if (*value==0xffffU)
          *value=def;
        else
          *value= (*value + offset) * precision;
#if defined(HOST_IS_BIG_ENDIAN)
       *value = SWAPINT16(*value);
#endif
        index=index + 2;
        break;

        case typ8ByteDouble:
        memcpy((void*)value,(void*)&buf[index], 8);
        if (*value==0x7fffffffffffffffLL)
           *value=def;
        else
           *value= (*value + offset) * precision;
        index=index + 8;
        break;

        case typ3ByteDouble:
        memcpy((void*)value,(void*)&buf[index], 3);
        if (*value==0x007fffffL)
            *value=def;
        else
           *value= (*value + offset) * precision;
        index=index + 3;
        break;

        case typ4ByteDouble:
        memcpy((void*)value,(void*)&buf[index], 4);
        if (*value==0x7fffffffL)
           *value=def;
        else
           *value= (*value + offset) * precision;
        index=index + 4;
        break;

        case typ4ByteUDouble:
        memcpy((void*)value,(void*)&buf[index], 4);
        if (*value==0xffffffffL)
           *value=def;
        else
           *value= (*value + offset) * precision;
#if defined(HOST_IS_BIG_ENDIAN)
        *value = SWAPINT32(*value);
#endif
        index=index + 4;
        break;

        case typ1ByteUint:
        memcpy((void*)value,(void*)&buf[index], 1);
        index=index + 1;
        break;

        default:
        break;
   }
}

/* ----------------------------------------------------------------------------
 *  j1939GetBuff : Get the buffer with the value at the precision given
 *  param : int16_t index, const unsigned char* buf, N2K_Data_Type_e n2kType,
 *         float64_t precision, float64_t def, n2k_data_obj_t *value
 *  return : void
 * -------------------------------------------------------------------------- */
void j1939GetBuff(int16_t *index, const unsigned char* buf, j1939_Data_Type_e j1939Type, float64_t precision, uint32_t maxVal, j1939_data_obj_t *value )
{

    if (((buf==NULL) || (value==NULL)) || (index==NULL))
      return;
    if (*index > sizeof(buf))
      return;

    switch(j1939Type)
    {
        case typ2ByteInt:
        memcpy((void*)&value->j1939field.ty2ByteInt,(void*)&buf[*index], 2u);
        *index=*index + 2;
        break;

        case typ2ByteUInt:
        memcpy((void*)&value->j1939field.ty2ByteUInt,(void*)&buf[*index], 2u);
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty2ByteUInt = SWAPINT16(value->j1939field.ty2ByteUInt);
#endif
        *index=*index + 2;
        break;

        case typ3ByteUInt:
        memcpy((void*)&value->j1939field.ty3ByteUInt,(void*)&buf[*index], 3u);
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty3ByteUInt = SWAPINT32(value->j1939field.ty3ByteUInt);
#endif
        *index=*index + 3;
        break;

        case typ4ByteUInt:
        memcpy((void*)&value->j1939field.ty4ByteUInt,(void*)&buf[*index], 4u);
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty4ByteUInt = SWAPINT32(value->j1939field.ty4ByteUInt);
#endif
        *index=*index + 4;
        break;

        case typ8ByteUInt:
        memcpy((void*)&value->j1939field.ty8ByteUInt,(void*)&buf[*index], 8u);
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty8ByteUInt = SWAPINT64(value->j1939field.ty8ByteUInt);
#endif
        *index=*index + 8;
        break;

        case typ1ByteDouble:
        memcpy((void*)&value->j1939field.ty1ByteDouble,(void*)&buf[*index], 1u);
        if (value->j1939field.ty1ByteDouble==0x7f)
           value->j1939field.ty1ByteDouble=maxVal;
        else
            value->j1939field.ty1ByteDouble= value->j1939field.ty1ByteDouble * precision;
        *index=*index + 1;
        break;

        case typ1ByteUDoubleLinakVal:
        memcpy((void*)&value->j1939field.ty1ByteUDouble,(void*)&buf[*index], 1u);
        if (buf[*index] <= maxVal)
        {
            value->j1939field.ty1ByteUDouble= value->j1939field.ty1ByteUDouble * precision;
        }
        *index=*index + 1;
        break;

        case typ1ByteUDouble:
        memcpy((void*)&value->j1939field.ty1ByteUDouble,(void*)&buf[*index], 1u);
        if (value->j1939field.ty1ByteUDouble==0xffU)
           value->j1939field.ty1ByteUDouble=maxVal;
        else
           value->j1939field.ty1ByteUDouble= value->j1939field.ty1ByteUDouble * precision;
        *index=*index + 1;
        break;

        case typ2ByteDouble:
        memcpy((void*)&value->j1939field.ty2ByteDouble,(void*)&buf[*index], 2u);
        if (value->j1939field.ty2ByteDouble==0x7fff)
           value->j1939field.ty2ByteDouble=maxVal;
        else
           value->j1939field.ty2ByteDouble= value->j1939field.ty2ByteDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty2ByteDouble = SWAPINT16(value->j1939field.ty2ByteDouble);
#endif
        *index=*index + 2;
        break;

        case typ2ByteUDouble:
        memcpy((void*)&value->j1939field.ty2ByteUDouble,(void*)&buf[*index], 2u);
        if (value->j1939field.ty2ByteUDouble==0xffffU)
          value->j1939field.ty2ByteUDouble=maxVal;
        else
          value->j1939field.ty2ByteUDouble= value->j1939field.ty2ByteUDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
       value->j1939field.ty2ByteUDouble = SWAPINT16(value->j1939field.ty2ByteUDouble);
#endif
        *index=*index + 2;
        break;

        case typ2ByteDoubleLinakVal:
        memcpy((void*)&value->j1939field.ty2ByteUDouble,(void*)&buf[*index], 2u);
        if (value->j1939field.ty2ByteUDouble <= maxVal)
        {
          value->j1939field.ty2ByteUDouble= value->j1939field.ty2ByteUDouble * precision;
        }
#if defined(HOST_IS_BIG_ENDIAN)
       value->j1939field.ty2ByteUDouble = SWAPINT16(value->j1939field.ty2ByteUDouble);
#endif
        *index=*index + 2;
        break;

        case typ8ByteDouble:
        memcpy((void*)&value->j1939field.ty8ByteDouble,(void*)&buf[*index], 8u);
        if (value->j1939field.ty8ByteDouble==0x7fffffffffffffffLL)
           value->j1939field.ty8ByteDouble=maxVal;
        else
           value->j1939field.ty8ByteDouble= value->j1939field.ty8ByteDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty8ByteDouble = SWAPINT64(value->j1939field.ty8ByteDouble);
#endif
        *index=*index + 8;
        break;

        case typ3ByteDouble:
        memcpy((void*)&value->j1939field.ty3ByteDouble,(void*)&buf[*index], 3u);
        if (value->j1939field.ty3ByteDouble==0x007fffffL)
           value->j1939field.ty3ByteDouble=maxVal;
        else
           value->j1939field.ty3ByteDouble= value->j1939field.ty3ByteDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty3ByteDouble = SWAPINT32(value->j1939field.ty3ByteDouble);
#endif
        *index=*index + 3;
        break;

        case typ4ByteDouble:
        memcpy((void*)&value->j1939field.ty4ByteDouble,(void*)&buf[*index], 4u);
        if (value->j1939field.ty4ByteDouble==0x7fffffffL)
           value->j1939field.ty4ByteDouble=maxVal;
        else
            value->j1939field.ty4ByteDouble= value->j1939field.ty4ByteDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty4ByteDouble = SWAPINT32(value->j1939field.ty4ByteDouble);
#endif
        *index=*index + 4;
        break;

        case typ4ByteUDouble:
        memcpy((void*)&value->j1939field.ty4ByteUDouble,(void*)&buf[*index], 4u);
        if (value->j1939field.ty4ByteUDouble==0xffffffffL)
           value->j1939field.ty4ByteUDouble=maxVal;
        else
           value->j1939field.ty4ByteUDouble= value->j1939field.ty4ByteUDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty4ByteUDouble = SWAPINT32(value->j1939field.ty4ByteUDouble);
#endif
        *index=*index + 4;
        break;

        case typ1ByteUint:
        memcpy((void*)&value->j1939field.ty1ByteUInt,(void*)&buf[*index], 1u);
        *index=*index + 1;
        break;

        default:
        break;
   }

}

/* ----------------------------------------------------------------------------
 *  j1939SetBuff : Set the buffer with the value at the precision given
 *  param : int16_t index, const unsigned char* buf, N2K_Data_Type_e n2kType,
 *         float64_t precision, float64_t def, n2k_data_obj_t *value
 *  return : void
 * -------------------------------------------------------------------------- */
void j1939SetBuff(int16_t *index, const unsigned char* buf, j1939_Data_Type_e j1939Type, float64_t precision, uint32_t maxVal, j1939_data_obj_t *value )
{

    float64_t fp=0.0f;
    int64_t fpll=0;
    int64_t vll=0;
    int32_t v10=0;
    int16_t v9=0;
    int8_t v8=0;
    uint8_t wayLen=0u;

    if (((buf==NULL) || (value==NULL)) || (index==NULL))
      return;
    if (*index > sizeof(buf))
      return;

    switch(j1939Type)
    {
        case typ2ByteInt:
        memcpy((void*)&buf[*index], (void*)&value->j1939field.ty2ByteInt, 2u);
        *index=*index + 2;
        break;

        case typ2ByteUInt:
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty2ByteUInt = SWAPINT16(value->j1939field.ty2ByteUInt);
#endif
        memcpy((void*)&buf[*index], (void*)&value->j1939field.ty2ByteUInt, 2u);
        *index=*index + 2;
        break;

        case typ3ByteUInt:
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty3ByteUInt = SWAPINT32(value->j1939field.ty3ByteUInt);
#endif
        memcpy((void*)&buf[*index], (void*)&value->j1939field.ty3ByteUInt, 3u);
        *index=*index + 3;
        break;

        case typ4ByteUInt:
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty4ByteUInt = SWAPINT32(value->j1939field.ty4ByteUInt);
#endif
        memcpy((void*)&buf[*index], (void*)&value->j1939field.ty4ByteUInt, 4u);
        *index=*index + 4;
        break;

        case typ8ByteUInt:
#if defined(HOST_IS_BIG_ENDIAN)
        value->j1939field.ty8ByteUInt = SWAPINT64(value->j1939field.ty8ByteUInt);
#endif
        memcpy((void*)&buf[*index], (void*)&value->j1939field.ty8ByteUInt, 8u);
        *index=*index + 8;
        break;

        case typ1ByteDouble:
        fp = value->j1939field.ty1ByteDouble * precision;
        v8 = ((uint8_t)ROUND(fp,0));
        memcpy((void*)&buf[*index], (void*) &v8, 1u);
        *index=*index + 1;
        break;

        case typ1ByteUDouble:
        fp = value->j1939field.ty1ByteUDouble * precision;
        v8 = ((uint8_t)ROUND(fp,0));
        memcpy((void*)&buf[*index], (void*) &v8, 1u);
        *index=*index + 1;
        break;

        case typ1ByteUDoubleLinakVal:
        if (value->j1939field.ty1ByteUDouble >= maxVal)
        {
           v8 = ((uint8_t)ROUND(fp,0));
        }
        else
        {
           fp = value->j1939field.ty1ByteUDouble * precision;
           if (fp <= maxVal)
           {
             v8 = ((uint8_t)ROUND(fp,0));
           }
           else
           {
             v8 = maxVal;
           }
        }
        memcpy((void*)&buf[*index], (void*) &v8, 1u);
        *index=*index + 1;
        break;

        case typ2ByteDouble:
        fp = value->j1939field.ty2ByteDouble * precision;
        v9 = ((uint16_t)ROUND(fp,0));
#if defined(HOST_IS_BIG_ENDIAN)
        v9 = SWAPINT16(v9);
#endif
        memcpy((void*)&buf[*index], (void*) &v9, 2u);
        *index=*index + 2;
        break;

        case typ2ByteDoubleLinakVal:
        if (value->j1939field.ty2ByteDouble > maxVal)
        {
            v9 = ((uint16_t)ROUND(fp,0));
        }
        else
        {
           fp = value->j1939field.ty2ByteDouble * precision;
           v9 = ((uint16_t)ROUND(fp,0)); 
           v9 = v9 % maxVal;
        }
#if defined(HOST_IS_BIG_ENDIAN)
        v9 = SWAPINT16(v9);
#endif
        memcpy((void*)&buf[*index], (void*) &v9, 2u);
        *index=*index + 2;
        break;

        case typ2ByteUDouble:
        fp = value->j1939field.ty2ByteUDouble * precision;
        v9 = ((uint16_t)ROUND(fp,0));
#if defined(HOST_IS_BIG_ENDIAN)
        v9 = SWAPINT16(v9);
#endif
        memcpy((void*)&buf[*index], (void*) &v9, 2u);
        *index=*index + 2;
        break;

        case typ8ByteDouble:
        fp=precision*1e6f;
        fpll=(int64_t)(1.0f*fp);
        vll=value->j1939field.ty8ByteDouble*1e6f;
        vll*=fpll % UINT64_MAX;
#if defined(HOST_IS_BIG_ENDIAN)
        vll = SWAPINT64(vll);
#endif
        memcpy((void*)&buf[*index], (void*) &vll, 8u);
        *index=*index + 8;
        break;

        case typ3ByteDouble:
        fp = value->j1939field.ty3ByteDouble * precision;
        v10 = ((uint32_t)ROUND(fp,0));
#if defined(HOST_IS_BIG_ENDIAN)
        v10 = SWAPINT32(v10);
#endif
        memcpy((void*)&buf[*index], (void*) &v10, 3u);
        *index=*index + 3;
        break;

        case typ4ByteDouble:
        fp = value->j1939field.ty4ByteDouble * precision;
        v10 = ((uint32_t)ROUND(fp,0));
#if defined(HOST_IS_BIG_ENDIAN)
        v10 = SWAPINT32(v10);
#endif
        memcpy((void*)&buf[*index], (void*) &v10, 4u);
        *index=*index + 4;
        break;

        case typ4ByteUDouble:
        fp = value->j1939field.ty4ByteUDouble * precision;
        v10 = ((uint32_t)ROUND(fp,0));
#if defined(HOST_IS_BIG_ENDIAN)
        v10 = SWAPINT32(v10);
#endif
        memcpy((void*)&buf[*index], (void*) &v10, 4u);
        *index=*index + 4;
        break;

        case typ1ByteUint:
        memcpy((void*)&buf[*index], (void*)&value->j1939field.ty1ByteUInt, 1u);
        *index=*index + 1;
        break;

        default:
        break;
   }

}
/* ----------------------------------------------------------------------------
 *  j1939toCanID : make can id
 *  param : j1939_Control_t * j1939ConObj
 *
 *  return : uint32_t
 * -------------------------------------------------------------------------- */
uint32_t j1939toCanID(j1939_Control_t * j1939ConObj)
{
  uint8_t CanIdPF = (uint8_t) (j1939ConObj->PGN >> 8u);

  if (CanIdPF < 240u)                                                           // PDU1 format
  {
     if ( (j1939ConObj->PGN & 0xfful) != 0ul ) return 0ul;                          // for PDU1 format PGN lowest byte has to be 0 for the destination.
     return ( ((uint32_t)(j1939ConObj->priority & 0x7u))<<26u | j1939ConObj->PGN<<8ul | ((uint32_t)j1939ConObj->dst)<<8u | (uint32_t)j1939ConObj->src);
  }
  else                                                                          // PDU2 format
  {
     return ( ((uint32_t)(j1939ConObj->priority & 0x7u))<<26u | j1939ConObj->PGN<<8ul | (uint32_t)j1939ConObj->src);
  }
}

#if defined(LINAK_ACTUATOR_USED)
//******************** Write Linak General Message ******************************
/* ----------------------------------------------------------------------------
 *  WriteJ1939LinakGenMsg : Write Linak General Message
 *  param : uint8_t PortNo, j1939_Control_t * j1939ConObj
 *
 *  return : uint32_t 0xFFFFLU on success
 * -------------------------------------------------------------------------- */
uint32_t WriteJ1939LinakGenMsg(uint8_t PortNo, j1939_Control_t * j1939ConObj)
{
  j1939_data_obj_t j1939Value;
  int16_t Index=0;
  uint32_t msgFeedback=0LU;
  uint32_t canId;

  j1939Value.j1939field.ty2ByteUDouble = j1939ConObj->linakCommand.LinakCtrlObj.position;
  j1939SetBuff(&Index, &j1939ConObj->linakCommand.DataBuf, typ2ByteDoubleLinakVal, 10.0f, LINAK_POS_MAX, &j1939Value );
  j1939Value.j1939field.ty1ByteUDouble = j1939ConObj->linakCommand.LinakCtrlObj.current;;
  j1939SetBuff(&Index, &j1939ConObj->linakCommand.DataBuf, typ1ByteUDoubleLinakVal, 0.004f, LINAK_CURRENT_MAX, &j1939Value );
  j1939Value.j1939field.ty1ByteUDouble = j1939ConObj->linakCommand.LinakCtrlObj.speed;
  j1939SetBuff(&Index, &j1939ConObj->linakCommand.DataBuf, typ1ByteUDoubleLinakVal, 2.0f, LINAK_SPEED_MAX, &j1939Value );
  j1939Value.j1939field.ty1ByteUDouble = j1939ConObj->linakCommand.LinakCtrlObj.softStart;
  j1939SetBuff(&Index, &j1939ConObj->linakCommand.DataBuf, typ1ByteUDoubleLinakVal, 0.02f, LINAK_SOFT_MAX, &j1939Value );
  j1939Value.j1939field.ty1ByteUDouble = j1939ConObj->linakCommand.LinakCtrlObj.softStop;
  j1939SetBuff(&Index, &j1939ConObj->linakCommand.DataBuf, typ1ByteUDoubleLinakVal, 0.02f, LINAK_SOFT_MAX, &j1939Value );
  j1939Value.j1939field.ty1ByteUInt = 0xFFU;
  j1939SetBuff(&Index, &j1939ConObj->linakCommand.DataBuf, typ1ByteUint, 0.02f, 0u, &j1939Value );
  j1939Value.j1939field.ty1ByteUInt = 0xFFU;
  j1939SetBuff(&Index, &j1939ConObj->linakCommand.DataBuf, typ1ByteUint, 0.02f, 0u, &j1939Value );

   canId = j1939toCanID( j1939ConObj );                                         /* create the can id */
   switch (PortNo)                                                              /* read which ever can port was specified */
   {
      case 1u:
      msgFeedback = CAN1Write(canId, &j1939ConObj->linakCommand.DataBuf, 8u, (uint16_t) ((j1939ConObj->priority & _CAN_TX_STD_FRAME) & _CAN_TX_NO_RTR_FRAME));         /* read port 1 */
      break;

      case 2u:
      msgFeedback = CAN2Write(canId, &j1939ConObj->linakCommand.DataBuf, 8u, (uint16_t) ((j1939ConObj->priority & _CAN_TX_STD_FRAME) & _CAN_TX_NO_RTR_FRAME));         /* read port 2 */
      break;

      default:
      break;
   }
  return msgFeedback;
}

//******************** Parse Linak Status Message ******************************
/* ----------------------------------------------------------------------------
 *  ParseJ1939LinakStatMsg : Parse Linak Status Message
 *  param : linak_status_t *J1939Obj, unsigned char *j1939RcvBuf
 *
 *  return : uint8_t
 * -------------------------------------------------------------------------- */
uint8_t ParseJ1939LinakStatMsg(linak_status_t *J1939Obj, unsigned char *j1939RcvBuf)
{
  j1939_data_obj_t j1939Value;
  int16_t Index=0;

  j1939GetBuff(&Index, j1939RcvBuf, typ2ByteDoubleLinakVal, 0.1f, LINAK_POS_MAX, &j1939Value );
  J1939Obj->positionLost = (j1939Value.j1939field.ty2ByteUInt == LINAK_POSN_LOST);
  J1939Obj->position= j1939Value.j1939field.ty2ByteUDouble;
  j1939GetBuff(&Index, j1939RcvBuf, typ1ByteUDoubleLinakVal, 250.0f, LINAK_CURRENT_MAX, &j1939Value );
  J1939Obj->errorInCurrentMeas = (j1939Value.j1939field.ty1ByteUInt == LINAK_CURR_MEAS_ERR);
  J1939Obj->current= j1939Value.j1939field.ty1ByteUDouble;
  j1939GetBuff(&Index, j1939RcvBuf, typ1ByteUint, 0.0f, 0u, &j1939Value );
  J1939Obj->ESSinb1= j1939Value.j1939field.ty1ByteUInt;
  j1939GetBuff(&Index, j1939RcvBuf, typ1ByteUint, 0.0f, 0u, &j1939Value );
  J1939Obj->error= j1939Value.j1939field.ty1ByteUInt;
  j1939GetBuff(&Index, j1939RcvBuf, typ1ByteUDoubleLinakVal, 0.1f, LINAK_SPEED_MAX, &j1939Value );
  J1939Obj->speed= j1939Value.j1939field.ty2ByteUDouble;
  j1939GetBuff(&Index, j1939RcvBuf, typ1ByteUint, 0.0f, 0u, &j1939Value );
  J1939Obj->Byte6FF= j1939Value.j1939field.ty1ByteUInt;
  j1939GetBuff(&Index, j1939RcvBuf, typ1ByteUint, 0.0f, 0u, &j1939Value );
  J1939Obj->inputLevel1= j1939Value.j1939field.ty1ByteUInt;
  return true;
}
#endif /* end Linak */

/* ----------------------------------------------------------------------------
 *  CanIdToj1939 : get src and pgn from canid returned
 *  param : uint32_t id, j1939_Control_t * j1939Obj
 *
 *  return : void
 * -------------------------------------------------------------------------- */
void CanIdToj1939(uint32_t id, j1939_Control_t * j1939Obj)
{
  uint8_t CanIdPF = (uint8_t) ((id >> 16u) % UINT8_MAX);
  uint8_t CanIdPS = (uint8_t) ((id >> 8u) % UINT8_MAX);
  uint8_t CanIdDP = (uint8_t) ((id >> 24u) & 1u);

    j1939Obj->src = (uint8_t) id >> 0;
    j1939Obj->priority = (uint8_t) ((id >> 26) & 0x7);

    if (CanIdPF < 240u)
    {
        j1939Obj->dst = CanIdPS;                                                 /* PDU1 format, the PS contains the destination address */
        j1939Obj->PGN = (((uint32_t)CanIdDP) << 16u) | (((uint32_t)CanIdPF) << 8u);
    }
    else
    {
        j1939Obj->dst = 0xffu;                                                   /* PDU2 format, the destination is implied global and the PGN is extended */
        j1939Obj->PGN = (((uint32_t)CanIdDP) << 16u) | (((uint32_t)CanIdPF) << 8u) | (uint32_t)CanIdPS;
    }
}
/* ----------------------------------------------------------------------------
 *  readJ1939CANBusMsg : read can bus write read for every device
 *  param : j1939_Control_t * j1939Obj, int8_t PortNo
 *
 *  return : uint8_t
 * -------------------------------------------------------------------------- */
uint8_t readJ1939CANBusMsg( j1939_Control_t * j1939Obj, int8_t PortNo)
{
   uint16_t msgFeedBack=0u;
   uint16_t data_len=0u;
   uint16_t rx_flags = 0u;
   int8_t returnCode = -1;
   uint32_t canId=0ul;
   char dataValues[8u] = { 0u,0u,0u,0u,0u,0u,0u,0u };                           /* array to read the can packet into */


   switch (PortNo)                                                              /* read which ever can port was specified */
   {
      case 1u:
      msgFeedBack = CAN1Read(&canId, dataValues, &data_len, &rx_flags);         /* read port 1 */
      break;

      case 2u:
      msgFeedBack = CAN2Read(&canId, dataValues, &data_len, &rx_flags);         /* read port 2 */
      break;

      default:
      break;
   }

   if ((((msgFeedBack == 0xFFFFU) && !(rx_flags & _CAN_RX_INVALID_MSG)) && !(rx_flags & _CAN_RX_OVERFLOW)) && (data_len <= 8u)) /* can frame recieved without an error */
   {
      CanIdToj1939(canId, j1939Obj);                                            /* get the PGN and source address from the can message */
      switch (j1939Obj->src)                                                    /* chack the source address */
      {
          case actNo1LftHndAddr:                                                /* =========== LINAK ACTUATOR NO.1 =================== */
          switch (j1939Obj->PGN)                                                /* look at the PGN Number received */
          {
             case LINAK_GEN_STAT_PGN:                                           /* ----------- LINAK ACTUATOR FRAME TO RECEIVE --------------------- */
             if (!(rx_flags & _CAN_RX_XTD_FRAME))                               /* not extended frame */
             {
               memset((void*)&j1939Obj->linakStatus.DataBuf,(void*) 0u, data_len);  /* copy the 8 bytes (data_len bytes received) in the frame */
               memcpy((void*)&j1939Obj->linakStatus.DataBuf,(void*) dataValues, data_len);  /* copy the 8 bytes (data_len bytes received) in the frame */
               returnCode=ParseJ1939LinakStatMsg(&j1939Obj->linakStatus.LinakStatObj, (unsigned char *) &j1939Obj->linakStatus.DataBuf); /* parse buffer and set variables in the display structure */
             }
             break;

             default:                                                            /* unknown pgn for this device you may wish to add other pgns for this address */
             break;
          }
          break;

          case actNo2LftHndAddr:                                                /* =========== LINAK ACTUATOR NO.2 =================== */
          switch (j1939Obj->PGN)                                                /* look at the PGN Number received */
          {
             case LINAK_GEN_STAT_PGN:                                           /* ----------- LINAK ACTUATOR FRAME TO RECEIVE --------------------- */
             if (!(rx_flags & _CAN_RX_XTD_FRAME))                               /* not extended frame */
             {
               memset((void*)&j1939Obj->linakStatus.DataBuf,(void*) 0u, data_len);  /* copy the 8 bytes (data_len bytes received) in the frame */
               memcpy((void*)&j1939Obj->linakStatus.DataBuf,(void*) dataValues, data_len);  /* copy the 8 bytes (data_len bytes received) in the frame */
               returnCode=ParseJ1939LinakStatMsg(&j1939Obj->linakStatus.LinakStatObj, (unsigned char *) &j1939Obj->linakStatus.DataBuf); /* parse buffer and set variables in the display structure */
             }
             break;

             default:                                                            /* unknown pgn for this device you may wish to add other pgns for this address */
             break;
          }
          break;
          
          case actNo1RghtHndAddr:                                               /* =========== LINAK ACTUATOR NO.3 =================== */
          switch (j1939Obj->PGN)                                                /* look at the PGN Number received */
          {
             case LINAK_GEN_STAT_PGN:                                           /* ----------- LINAK ACTUATOR FRAME TO RECEIVE --------------------- */
             if (!(rx_flags & _CAN_RX_XTD_FRAME))                               /* not extended frame */
             {
               memset((void*)&j1939Obj->linakStatus.DataBuf,(void*) 0u, data_len);  /* copy the 8 bytes (data_len bytes received) in the frame */
               memcpy((void*)&j1939Obj->linakStatus.DataBuf,(void*) dataValues, data_len);  /* copy the 8 bytes (data_len bytes received) in the frame */
               returnCode=ParseJ1939LinakStatMsg(&j1939Obj->linakStatus.LinakStatObj, (unsigned char *) &j1939Obj->linakStatus.DataBuf); /* parse buffer and set variables in the display structure */
             }
             break;

             default:                                                            /* unknown pgn for this device you may wish to add other pgns for this address */
             break;
          }
          break;
          
          case actNo2RghtHndAddr:                                               /* =========== LINAK ACTUATOR NO.1 =================== */
          switch (j1939Obj->PGN)                                                /* look at the PGN Number received */
          {
             case LINAK_GEN_STAT_PGN:                                           /* ----------- LINAK ACTUATOR FRAME TO RECEIVE --------------------- */
             if (!(rx_flags & _CAN_RX_XTD_FRAME))                               /* not extended frame */
             {
               memset((void*)&j1939Obj->linakStatus.DataBuf,(void*) 0u, data_len);  /* copy the 8 bytes (data_len bytes received) in the frame */
               memcpy((void*)&j1939Obj->linakStatus.DataBuf,(void*) dataValues, data_len);  /* copy the 8 bytes (data_len bytes received) in the frame */
               returnCode=ParseJ1939LinakStatMsg(&j1939Obj->linakStatus.LinakStatObj, (unsigned char *) &j1939Obj->linakStatus.DataBuf); /* parse buffer and set variables in the display structure */
             }
             break;

             default:                                                            /* unknown pgn for this device you may wish to add other pgns for this address */
             break;
          }
          break;
          
          default:                                                              /* no more devices are supported here */
          break;
       }
   }

   return returnCode;

}
/* ----------------------------------------------------------------------------
 *  PollJ1939CANBusMsg : poll can bus write read for every device
 *  param : j1939_Control_t * j1939Obj, int8_t PortNo
 *
 *  return : uint8_t
 * -------------------------------------------------------------------------- */
uint8_t PollJ1939CANBusMsg( j1939_Control_t * j1939Obj, int8_t PortNo )
{
   int32_t ticks2Now;

   switch(j1939Obj->linakCommand.State)
   {
        case LIN_NEW_FWD_TO_SEND:                                               /* position request has been sent to move the actuator in forward direction */
        if (WriteJ1939LinakGenMsg( PortNo, j1939Obj )==0xFFFFLU)
        {
           j1939Obj->linakCommand.prevState = j1939Obj->linakCommand.State;
           j1939Obj->linakStatus.timeRef = CP0_GET(CP0_COUNT);                  /* get new time reference to wait for the next time */
           j1939Obj->linakCommand.State = LIN_READ_FIELD_STATUS;
        }
        break;

        case LIN_NEW_BWD_TO_SEND:                                               /* position request has been sent to move the actuator in backward direction */
        if (WriteJ1939LinakGenMsg( PortNo, j1939Obj )==0xFFFFLU)
        {
           j1939Obj->linakCommand.prevState = j1939Obj->linakCommand.State;
           j1939Obj->linakStatus.timeRef = CP0_GET(CP0_COUNT);                  /* get new time reference to wait for the next time */
           j1939Obj->linakCommand.State = LIN_READ_FIELD_STATUS;
        }
        break;

        case LIN_NEW_PARAMS:                                                    /* change of parameters has been requested to the actuator */
        if (WriteJ1939LinakGenMsg( PortNo, j1939Obj )==0xFFFFLU)
        {
           j1939Obj->linakCommand.State = LIN_READ_FIELD_STATUS;
        }
        break;
        
        case LIN_READ_FIELD_STATUS:
        if (readJ1939CANBusMsg( j1939Obj, PortNo ))                             /* message was received */
        {
           /* reached commanded position */
           if ((j1939Obj->linakStatus.LinakStatObj.position >= j1939Obj->linakCommand.LinakCtrlObj.position) || ((LINAK_POS_MAX_F <= j1939Obj->linakCommand.LinakCtrlObj.position) && (j1939Obj->linakStatus.LinakStatObj.position >= LINAK_POS_MAX_F)))
           {
              j1939Obj->linakCommand.State = LIN_SEND_STOP;
           }
           else if (j1939Obj->linakStatus.LinakStatObj.Overcurrentb3 == 1u)     /* got an overcurrent go opposite way ref page 37 */
           {
                switch( j1939Obj->linakCommand.prevState )
                {
                   case LIN_NEW_FWD_TO_SEND:
                   j1939Obj->linakCommand.State = LIN_RUN_TO_IN;
                   break;

                   case LIN_NEW_BWD_TO_SEND:
                   j1939Obj->linakCommand.State = LIN_RUN_TO_OUT;
                   break;

                   case LIN_RUN_TO_IN:
                   j1939Obj->linakCommand.State = LIN_RUN_TO_OUT;
                   break;

                   case LIN_RUN_TO_OUT:
                   j1939Obj->linakCommand.State = LIN_RUN_TO_IN;
                   break;
                   
                   default:                                                     /* not known or STOP  sent already */
                   if (j1939Obj->linakStatus.LinakStatObj.error != 0u)          /* got an error */
                   {
                      j1939Obj->linakCommand.State = LIN_ACTUATOR_ERROR;
                   }
                   break;
                }
            }
            else if (j1939Obj->linakStatus.LinakStatObj.error != 0u)            /* got an error */
            {
                j1939Obj->linakCommand.State = LIN_ACTUATOR_ERROR;
            }
            else
            {
                calculateTick2Now( &ticks2Now, &j1939Obj->linakStatus.timeRef );/* get the tick count up til now */
                if (ticks2Now >= LINNAK_NOT_POSN_TIMOT)                         /* took too long getting to position so try the previous command state again */
                {
                     j1939Obj->linakCommand.State = j1939Obj->linakCommand.prevState;
                }
            }
        }
        else
        {
            calculateTick2Now( &ticks2Now, &j1939Obj->linakStatus.timeRef );    /* get the tick count up til now */
            if (ticks2Now >= LINNAK_NOT_RESP_TIMOT)                             /* took too long getting a reply so try the previous command state again */
            {
               j1939Obj->linakCommand.State = j1939Obj->linakCommand.prevState;
            }
        }
        break;

        case LIN_SEND_STOP:
        j1939Obj->linakCommand.LinakCtrlObj.position = LINAK_STOP_ACTUATOR;
        if (WriteJ1939LinakGenMsg( PortNo, j1939Obj )==0xFFFFLU)
        {
          j1939Obj->linakCommand.prevState = j1939Obj->linakCommand.State;
          j1939Obj->linakStatus.timeRef = CP0_GET(CP0_COUNT);                   /* get new time reference to wait for the next time */
          j1939Obj->linakCommand.State = LIN_READ_FIELD_STATUS;
        }
        break;

        case LIN_RUN_TO_IN:
        j1939Obj->linakCommand.LinakCtrlObj.position = LINAK_RUN_2_IN;
        if (WriteJ1939LinakGenMsg( PortNo, j1939Obj )==0xFFFFLU)
        {
          j1939Obj->linakCommand.prevState = j1939Obj->linakCommand.State;
          j1939Obj->linakStatus.timeRef = CP0_GET(CP0_COUNT);                   /* get new time reference to wait for the next time */
          j1939Obj->linakCommand.State = LIN_READ_FIELD_STATUS;
        }
        break;

        case LIN_RUN_TO_OUT:
        j1939Obj->linakCommand.LinakCtrlObj.position = LINAK_RUN_2_OUT;
        if (WriteJ1939LinakGenMsg( PortNo, j1939Obj )==0xFFFFLU)
        {
           j1939Obj->linakCommand.prevState = j1939Obj->linakCommand.State;
           j1939Obj->linakStatus.timeRef = CP0_GET(CP0_COUNT);                  /* get new time reference to wait for the next time */
           j1939Obj->linakCommand.State = LIN_READ_FIELD_STATUS;
        }
        break;

        case LIN_ACTUATOR_ERROR:
        j1939Obj->linakCommand.LinakCtrlObj.position = LINAK_CLR_ERROR_CODE;
        if (WriteJ1939LinakGenMsg( PortNo, j1939Obj )==0xFFFFLU)
        {
           j1939Obj->linakCommand.prevState = j1939Obj->linakCommand.State;
           j1939Obj->linakStatus.timeRef = CP0_GET(CP0_COUNT);                  /* get new time reference to wait for the next time */
           j1939Obj->linakCommand.State = LIN_READ_FIELD_STATUS;
        }
        break;

        default:
        break;

   }
   return 0u;
}
#endif