#ifdef __cplusplus
extern "C"
{
#endif

// Contains generic templates which are used to produce jump table functions
// for each camera option
// This is a generic library and is multiply included to create functions
// from defintions and these prototypes
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#if defined(VISCA_GET_ON_OFF_STATE)                                             // Call to get the on/off state groups
/*-----------------------------------------------------------------------------
 *      SONY_SND_INQ_FUNC():  Send state inquiry sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t = sending index
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_INQ_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_INQ;                                    // send inquiry message as defined by top level
     sendSonyMessage( &sonyMsg,VISCA_SND_MSG_LEN );
     return 1u;
}
/*-----------------------------------------------------------------------------
 *      SONY_RCV_STATE_FUNC():  Receive sony state request message
 *
 *  Parameters: none
 *  Return:     uint8_t = state found
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_RCV_STATE_FUNC( )
{
     if (viscaSerial.State == VISCA_PACKET_IN_BUFFER)                           // UART interrupt has just read a reply packet of data
     {
        SONY_rcv_t sonyMsgExp = SONY_MSG_RCV_ON;                                // compare the message to the ON state reply
        SONY_rcv_t sonyMsgRcv;
        memcpy(&sonyMsgRcv,viscaSerial.Buffer,VISCA_REPLY_MSG_LEN);             // copy the message from the interrupt buffer to the receive container
        if(!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp,VISCA_REPLY_MSG_LEN))  // message received matches that expected for on state
        {
           viscaSerial.State = VISCA_PACKET_READ_COMPLETE;                      // we have processed the reply tell the caller we are complete via the global state
           return 1u;                                                           // camera is ON
        }
        else
        {
           SONY_rcv_t sonyMsgExp = SONY_MSG_RCV_OFF;
           if(!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp,VISCA_REPLY_MSG_LEN)) // message received matches that expected for off state
           {
              viscaSerial.State = VISCA_PACKET_READ_COMPLETE;                   // we have processed the reply
              return 0u;                                                        // camera is OFF
           }
           else
           {
              viscaSerial.State = VISCA_PACKET_READ_ERROR;                      // we have processed the reply tell the caller we have not understood and keep looking
           }
        }
     }
     //return (!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp,sizeof(SONY_rcv_t)));
}
#else
#if defined(VISCA_GET_MULTI_STATE)                                              // Call to get the multi-state state groups
/*-----------------------------------------------------------------------------
 *      SONY_RCV_MULTI_STATE_FUNC():  receive sony multi state request message
 *
 *  Parameters: none
 *  Return:     uint8_t = state found
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_RCV_MULTI_STATE_FUNC( )
{
     uint8_t eachCheckState=VISCA_NO_OF_STATE_REPLIES;                          // the number of states you have
     SONY_rcv_t sonyMsgRcv;
     SONY_rcv_t sonyMsgExp6 = SONY_MSG_RCV_6;
     SONY_rcv_t sonyMsgExp5 = SONY_MSG_RCV_5;
     SONY_rcv_t sonyMsgExp4 = SONY_MSG_RCV_4;
     SONY_rcv_t sonyMsgExp3 = SONY_MSG_RCV_3;
     SONY_rcv_t sonyMsgExp2 = SONY_MSG_RCV_2;
     SONY_rcv_t sonyMsgExp1 = SONY_MSG_RCV_1;

     if (viscaSerial.State == VISCA_PACKET_IN_BUFFER)                           // UART interrupt has just read a reply packet of data
     {
        switch (eachCheckState)                                                 // cascade each check until one matches each possible reply or not
        {
           case 6u:
           memcpy(&sonyMsgRcv,viscaSerial.Buffer,VISCA_REPLY_MSG_LEN);          // copy the message from the interrupt buffer to the receive container
           if(!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp6,VISCA_REPLY_MSG_LEN))  // message received matches that expected for on state
           {
             viscaSerial.State = VISCA_PACKET_READ_COMPLETE;                    // we have processed the reply tell the caller we are complete via the global state
             return 1u;                                                         // state match item 1
           }
           else
           {
              eachCheckState=--eachCheckState;                                  // goto next case
           }

           case 5u:
           memcpy(&sonyMsgRcv,viscaSerial.Buffer,VISCA_REPLY_MSG_LEN);          // copy the message from the interrupt buffer to the receive container
           if(!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp5,VISCA_REPLY_MSG_LEN))  // message received matches that expected for on state
           {
             viscaSerial.State = VISCA_PACKET_READ_COMPLETE;                    // we have processed the reply tell the caller we are complete via the global state
             return 1u;                                                         // state match item 1
           }
           else
           {
              eachCheckState=--eachCheckState;                                  // goto next case
           }

           case 4u:
           memcpy(&sonyMsgRcv,viscaSerial.Buffer,VISCA_REPLY_MSG_LEN);          // copy the message from the interrupt buffer to the receive container
           if(!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp4,VISCA_REPLY_MSG_LEN))  // message received matches that expected for on state
           {
             viscaSerial.State = VISCA_PACKET_READ_COMPLETE;                    // we have processed the reply tell the caller we are complete via the global state
             return 1u;                                                         // state match item 1
           }
           else
           {
              eachCheckState=--eachCheckState;                                  // goto next case
           }

           case 3u:
           memcpy(&sonyMsgRcv,viscaSerial.Buffer,VISCA_REPLY_MSG_LEN);          // copy the message from the interrupt buffer to the receive container
           if(!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp3,VISCA_REPLY_MSG_LEN))  // message received matches that expected for on state
           {
             viscaSerial.State = VISCA_PACKET_READ_COMPLETE;                    // we have processed the reply tell the caller we are complete via the global state
             return 1u;                                                         // state match item 1
           }
           else
           {
              eachCheckState=--eachCheckState;                                  // goto next case
           }
           //break;

           case 2u:
           memcpy(&sonyMsgRcv,viscaSerial.Buffer,VISCA_REPLY_MSG_LEN);          // copy the message from the interrupt buffer to the receive container
           if(!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp2,VISCA_REPLY_MSG_LEN))  // message received matches that expected for on state
           {
             viscaSerial.State = VISCA_PACKET_READ_COMPLETE;                    // we have processed the reply tell the caller we are complete via the global state
             return 1u;                                                         // state match item 1
           }
           else
           {
              eachCheckState=--eachCheckState;                                  // goto next case
           }

           case 1u:
           memcpy(&sonyMsgRcv,viscaSerial.Buffer,VISCA_REPLY_MSG_LEN);          // copy the message from the interrupt buffer to the receive container
           if(!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp1,VISCA_REPLY_MSG_LEN))  // message received matches that expected for on state
           {
             viscaSerial.State = VISCA_PACKET_READ_COMPLETE;                    // we have processed the reply tell the caller we are complete via the global state
             return 1u;                                                         // state match item 1
           }
           else
           {
              eachCheckState=--eachCheckState;                                  // goto next case
           }

           default:                                                             // No state match
           viscaSerial.State = VISCA_PACKET_READ_ERROR;                         // we have processed the reply tell the caller we have not understood and keep looking
           break;

        }
        return(0u);
     }
     //return (!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp,sizeof(SONY_rcv_t)));
}
#else
#if ((SONY_OPTION_SET == VISCA_HAS_BOTH) || (SONY_OPTION_SET == VISCA_HAS_NO_ON_OFF))
/*-----------------------------------------------------------------------------
 *      SONY_SND_RESET_FUNC():  Send sony reset message
 *
 *  Parameters: (none)
 *  Return:     uint8_t (jump table corresponding position)
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_RESET_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_RESET;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 3u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_UP_FUNC():  Send sony up message
 *
 *  Parameters: (none)
 *  Return:     uint8_t (jump table corresponding position)
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_UP_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_UP;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 4u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_DOWN_FUNC():  Send sony down message
 *
 *  Parameters: (none)
 *  Return:     uint8_t (jump table corresponding position)
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_DOWN_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_DOWN;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 5u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_DIRECT_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_DIRECT_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_DIRECT;
     sonyMsg.mSndByte6 = g_visca_p;
     sonyMsg.mSndByte7 = g_visca_q;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 6u;
}
#endif
#if ((SONY_OPTION_SET == VISCA_HAS_BOTH) || (SONY_OPTION_SET == VISCA_HAS_ON_OFF))
/*-----------------------------------------------------------------------------
 *      SONY_SND_ON_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_ON_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_ON;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 1u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_OFF_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_OFF_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_OFF;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 2u;
}
#endif
#if defined(SONY_WANT_TOGGLE)
/*-----------------------------------------------------------------------------
 *      SONY_SND_TOGGLE_FUNC():  Send sony toggle state message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_TOGGLE_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_TOGGLE;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 3u;
}
#endif
#if defined(SONY_ZOOM_USED)
/*-----------------------------------------------------------------------------
 *      SONY_SND_STOP_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_STOP_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_STOP;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 1u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_TELE_STD_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_TELE_STD_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_TELE_STD;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 2u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_WIDE_STD_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_WIDE_STD_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_WIDE_STD;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 3u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_WIDE_VAR_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_WIDE_VAR_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_WIDE_VAR;
     sonyMsg.mSndByte4 = g_p1+0x20u;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 4u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_TELE_VAR_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_TELE_VAR_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_TELE_VAR;
     sonyMsg.mSndByte4 = g_p2+0x30u;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 5u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_DIRECT_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_DIRECT_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_DIRECT;
     sonyMsg.mSndByte4 = g_visca_p;                                             // zoom
     sonyMsg.mSndByte5 = g_visca_q;
     sonyMsg.mSndByte6 = g_visca_r;
     sonyMsg.mSndByte7 = g_visca_s;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 6u;
}
#endif
#if defined(SONY_FOCUS_USED)
/*-----------------------------------------------------------------------------
 *      SONY_SND_STOP_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_STOP_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_STOP;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 1u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_TELE_STD_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_TELE_STD_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_TELE_STD;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 2u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_WIDE_STD_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_WIDE_STD_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_WIDE_STD;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 3u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_1PSH_AF_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_1PSH_AF_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_1PSH_AF;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 4u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_DIRECT2_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_DIRECT2_FUNC( )
{
     SONY_send_long_t sonyMsg = SONY_MSG_SND_DIRECT2;
     sonyMsg.mSndByte4 = g_visca_p;                                             // zoom
     sonyMsg.mSndByte5 = g_visca_q;
     sonyMsg.mSndByte6 = g_visca_r;
     sonyMsg.mSndByte7 = g_visca_s;
     sonyMsg.mSndByte8 = g_visca_p1;                                            // focus
     sonyMsg.mSndByte9 = g_visca_q1;
     sonyMsg.mSndByte10 = g_visca_r1;
     sonyMsg.mSndByte11 = g_visca_s1;
     sendExtSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 5u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_DIRECT_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_DIRECT_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_DIRECT;
     sonyMsg.mSndByte4 = g_visca_p1;                                            // focus
     sonyMsg.mSndByte5 = g_visca_q1;
     sonyMsg.mSndByte6 = g_visca_r1;
     sonyMsg.mSndByte7 = g_visca_s1;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 6u;
}
#endif
#if defined(SONY_CAM_VID_SYS_USED) || defined(SONY_CAM_WB_USED) || defined(SONY_CAM_AE_USED) \
|| defined(SONY_CAM_MEM_USED) || defined(SONY_CAM_PAN_TILTDRIVE_USED) || defined(SONY_CAM_MISC_USED)
/*-----------------------------------------------------------------------------
 *      SONY_SND_1_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_1_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_1;
#if defined(SONY_CAM_MEM_USED)
     sonyMsg.mSndByte5 = g_visca_p1;
#endif
#if defined(SONY_CAM_VIS_SYS_USED)
     sonyMsg.mSndByte7 = g_visca_p1;
#endif
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
#endif
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 1u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_2_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_2_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_2;
#if defined(SONY_CAM_MEM_USED)
     sonyMsg.mSndByte5 = g_visca_p1;
#endif
#if defined(SONY_CAM_VIS_SYS_USED)
     sonyMsg.mSndByte4 = g_visca_q1;
#endif
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
#endif
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 2u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_3_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_3_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_3;
#if defined(SONY_CAM_MEM_USED)
     sonyMsg.mSndByte5 = g_visca_p1;
#endif
#if defined(SONY_CAM_VIS_SYS_USED)
     sonyMsg.mSndByte4 = g_visca_r1;
#endif
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
#endif
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 3u;
}
#endif

#if ((defined(SONY_CAM_WB_USED) || defined(SONY_CAM_VID_SYS_USED)) || defined(SONY_CAM_PAN_TILTDRIVE_USED))
/*-----------------------------------------------------------------------------
 *      SONY_SND_4_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_4_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_4;
#if defined(SONY_CAM_VIS_SYS_USED)
     sonyMsg.mSndByte5 = g_visca_s1;
#endif
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
#endif
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 4u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_5_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_5_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_5;
#if defined(SONY_CAM_VIS_SYS_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
     sonyMsg.mSndByte6 = g_visca_r1;
     sonyMsg.mSndByte7 = g_visca_s1;
#endif
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
#endif
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 5u;
}
#endif
#if defined(SONY_CAM_WB_USED) || defined(SONY_CAM_PAN_TILTDRIVE_USED)
/*-----------------------------------------------------------------------------
 *      SONY_SND_6_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_6_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_6;
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
#endif
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 6u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_7_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_7_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_7;
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
#endif
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 7u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_8_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_8_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_8;
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
#endif
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 8u;
}
#endif
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
/*-----------------------------------------------------------------------------
 *      SONY_SND_9_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_9_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_9;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 9u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_10_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_10_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_10;
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
#endif
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 10u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_11_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_11_FUNC( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_11;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 11u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_12_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_12_FUNC( )
{
     SONY_send_long_t sonyMsg = SONY_MSG_SND_12;
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
     sonyMsg.mSndByte6 = (g_visca_y >> 24u) & 0xFFu;
     sonyMsg.mSndByte7 = (g_visca_y >> 16u) & 0xFFu;
     sonyMsg.mSndByte8 = (g_visca_y >> 8u) & 0xFFu;
     sonyMsg.mSndByte9 = g_visca_y & 0xFFu;
     sonyMsg.mSndByte10 = (g_visca_z >> 24u) & 0xFFu;
     sonyMsg.mSndByte11 = (g_visca_z >> 16u) & 0xFFu;
     sonyMsg.mSndByte12 = (g_visca_z >> 8u) & 0xFFu;
     sonyMsg.mSndByte13 = g_visca_z & 0xFFu;
#endif
     sendExtSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 12u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_13_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_13_FUNC( )
{
     SONY_send_long_t sonyMsg = SONY_MSG_SND_13;
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte4 = g_visca_p1;
     sonyMsg.mSndByte5 = g_visca_q1;
     sonyMsg.mSndByte6 = (g_visca_y >> 24u) & 0xFFu;
     sonyMsg.mSndByte7 = (g_visca_y >> 16u) & 0xFFu;
     sonyMsg.mSndByte8 = (g_visca_y >> 8u) & 0xFFu;
     sonyMsg.mSndByte9 = g_visca_y & 0xFFu;
     sonyMsg.mSndByte10 = (g_visca_z >> 24u) & 0xFFu;
     sonyMsg.mSndByte11 = (g_visca_z >> 16u) & 0xFFu;
     sonyMsg.mSndByte12 = (g_visca_z >> 8u) & 0xFFu;
     sonyMsg.mSndByte13 = g_visca_z & 0xFFu;
#endif
     sendExtSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 13u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_14_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_14_FUNC( )
{
     SONY_send_long_t sonyMsg = SONY_MSG_SND_14;
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte5 = g_visca_p1;
     sonyMsg.mSndByte6 = (g_visca_y >> 24u) & 0xFFu;
     sonyMsg.mSndByte7 = (g_visca_y >> 16u) & 0xFFu;
     sonyMsg.mSndByte8 = (g_visca_y >> 8u) & 0xFFu;
     sonyMsg.mSndByte9 = g_visca_y & 0xFFu;
     sonyMsg.mSndByte10 = (g_visca_z >> 24u) & 0xFFu;
     sonyMsg.mSndByte11 = (g_visca_z >> 16u) & 0xFFu;
     sonyMsg.mSndByte12 = (g_visca_z >> 8u) & 0xFFu;
     sonyMsg.mSndByte13 = g_visca_z & 0xFFu;
#endif
     sendExtSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 14u;
}
/*-----------------------------------------------------------------------------
 *      SONY_SND_15_FUNC():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t SONY_SND_15_FUNC( )
{
     SONY_send_long_t sonyMsg = SONY_MSG_SND_15;
#if defined(SONY_CAM_PAN_TILTDRIVE_USED)
     sonyMsg.mSndByte5 = g_visca_p1;
     sonyMsg.mSndByte6 = (g_visca_y >> 24u) & 0xFFu;
     sonyMsg.mSndByte7 = (g_visca_y >> 16u) & 0xFFu;
     sonyMsg.mSndByte8 = (g_visca_y >> 8u) & 0xFFu;
     sonyMsg.mSndByte9 = g_visca_y & 0xFFu;
     sonyMsg.mSndByte10 = (g_visca_z >> 24u) & 0xFFu;
     sonyMsg.mSndByte11 = (g_visca_z >> 16u) & 0xFFu;
     sonyMsg.mSndByte12 = (g_visca_z >> 8u) & 0xFFu;
     sonyMsg.mSndByte13 = g_visca_z & 0xFFu;
#endif
     sendExtSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 15u;
}
#endif // on off state requests
#endif // multistate requests
#endif // else

#ifdef __cplusplus
} /* extern "C" */
#endif