#ifndef __microscan_func
#define __microscan_func

#ifdef __cplusplus
extern "C"
{
#endif

/*
    microscan_funcs.h : mircroscan functions for mircoscan camera


    Version : @(#) 1.0
    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
*/
int8_t parse_microScan( miCrosCanConnect_t* conn );
void getMicroscanValue( unsigned char * nameOfTag, uint8_t portNo, microScanGet_t *getReq );
void setMicroScanValue( unsigned char * nameOfTag, uint8_t portNo, microScanGet_t *getReq );
uint8_t setupMiScCamLRFB( microScanGet_t *getRequest );
void moveAccording2pArea( const ocr_match_select_t plantingArea[NUM_OF_PLANT_SECTION], miCrosCanConnect_t connCameraOut, ocr_movement_obj_t *moveAction );



/*******************************************************************************
* Function Name: parse_mircoScan
********************************************************************************
* Summary:
*  Parse the serial command from the microscan ocr camera
*
* Parameters:
*   uBiConnect_t* conn
*
* Return:
* return if fail is -1 (0xFFu) otherwise the number of strings it matched (updated)
* ==============================================================================
*
*******************************************************************************/
MIRSC_LIB int8_t parse_microScan( miCrosCanConnect_t* conn )
{
  char *p;                                                                      // create char pointer for iteration
  int8_t pos=0u;                                                                // position in string counter
  int8_t retStatus=-1;                                                          // initialise return code to error
  int8_t numMatch=0U;

  conn->keep_alive=!strncmp(conn->buf, "OCR", 3u);                              // message must start with an OCR
  if (!conn->keep_alive) return retStatus;                                      // no then return error
  for (p=strchr(conn->buf, '\n'); p; p=strchr(p, '\n'))                         // for each line
  {
    p++;                                                                        // increment to next char
    if (!strncmp(p, MIRSC_OCR1_STAT_STR, strlen(MIRSC_OCR1_STAT_STR)))
    {
      p+=strlen(MIRSC_OCR1_STAT_STR);                                            // skip command bytes
      numMatch=++numMatch % INT8_MAX;
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespecae
      conn->ocr1_stat=strncmp(p, "1", 1u);                                       // reads bool for ocr status
    }
    if (!strncmp(p, MIRSC_OCR2_STAT_STR, strlen(MIRSC_OCR2_STAT_STR)))
    {
      p+=strlen(MIRSC_OCR2_STAT_STR);                                            // skip command bytes
      numMatch=++numMatch % INT8_MAX;
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespecae
      conn->ocr2_stat=strncmp(p, "1", 1u);                                       // reads bool for ocr status
    }
    if (!strncmp(p, MIRSC_CAM1_STAT, strlen(MIRSC_CAM1_STAT)))
    {
      p+=strlen(MIRSC_CAM1_STAT);                                                // skip command bytes
      numMatch=++numMatch % INT8_MAX;
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespecae
      conn->camera_stat_lr=strncmp(p, "1", 1u);                                 // reads bool for left right camera status
    }
    if (!strncmp(p, MIRSC_CAM2_STAT, strlen(MIRSC_CAM2_STAT)))
    {
      p+=strlen(MIRSC_CAM2_STAT);                                                // skip command bytes
      numMatch=++numMatch % INT8_MAX;
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespecae
      conn->camera_stat_fb=strncmp(p, "1", 1u);                                 // reads bool for front back camera status
    }
    if (!strncmp(p, MIRSC_DM_STAT_STR, strlen(MIRSC_DM_STAT_STR)))
    {
      p+=strlen(MIRSC_DM_STAT_STR);                                             // skip command bytes
      numMatch=++numMatch % INT8_MAX;
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
      conn->dm_stat=strncmp(p, "1", 1u);                                        // reads bool for dm_stat
    }
    if (!strncmp(p, MIRSC_OCR_DATA1_STR, strlen(MIRSC_OCR_DATA1_STR)))
    {
      p+=strlen(MIRSC_OCR_DATA1_STR);                                           // skip command bytes
      numMatch=++numMatch % INT8_MAX;
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
      conn->dataVal=atol(p);                                                    // reads an sint32_t dataVal
    }
    if (!strncmp(p, MIRSC_CENTER_POINT_LOC, strlen(MIRSC_CENTER_POINT_LOC)))
    {
      p+=strlen(MIRSC_CENTER_POINT_LOC);                                        // skip command bytes
      numMatch=++numMatch % INT8_MAX;
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
      conn->center_p=atof(p);                                                   // reads an float64_t into center point location
    }
    if (!strncmp(p, MIRSC_OCR_DATA2_STR, strlen(MIRSC_OCR_DATA2_STR)))
    {
      p+=strlen(MIRSC_OCR_DATA2_STR);                                           // skip command bytes
      numMatch=++numMatch % INT8_MAX;
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
      if (pos>=ANCR_MAX_MSG) return 0u;                                         // string buffer overflow
      conn->numPlate[pos]=*p;                                                   // reads string into the number plate field
      pos=++pos % UINT8_MAX;
    }
    if (!strncmp(p, MIRSC_OCR_LR_STR, strlen(MIRSC_OCR_LR_STR)))
    {
      p+=strlen(MIRSC_OCR_LR_STR);                                              // skip command bytes
      numMatch=++numMatch % INT8_MAX;
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
      if (pos>=ANCR_MAX_MSG) return 0u;                                         // string buffer overflow
      conn->beaconLR[pos]=*p;                                                   // reads string into the left right beacon field
      pos=++pos % UINT8_MAX;
    }
    if (!strncmp(p, MIRSC_OCR_FB_STR, strlen(MIRSC_OCR_FB_STR)))
    {
      p+=strlen(MIRSC_OCR_FB_STR);                                              // skip command bytes
      numMatch=++numMatch % INT8_MAX;
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
      if (pos>=ANCR_MAX_MSG) return 0u;                                         // string buffer overflow
      conn->beaconFB[pos]=*p;                                                   // reads string into the front back beacon field
      pos=++pos % UINT8_MAX;
    }
    if (!strncmp(p, "Connection:", 11u))                                        // connection status message
    {
      p+=11u;                                                                   // skip command bytes
      numMatch=++numMatch % INT8_MAX;
      while (*p==' ' || *p=='\t') p++;                                          // skip whitespace and tabs
      conn->keep_alive=!strncmp(p, "1", 1u);                                    // set keep alive true
    }
  }
  return numMatch;                                                              // returns a byte which is a bitmap representing the type found
}

// state engine
#define MIRSC_CMD_SEND 0u
#define MIRSC_CMD_SENT 1u
#define MIRSC_CMD_REPLY 2u
#define MIRSC_CMD_COMPLETE 3u
#define MIRSC_CMD_FAIL 4u
#define MIRSC_MAX_RETRY_EXCEED 5u
#define MIRSC_CMD_UNKNOWN 6u
// fail retry time
#define MIRSC_RETRY_TIME 1000U
#define MIRSC_MAX_NUM_RETRY 5u
// data types
#define MIRSC_LOAD_CONF 4U
#define MIRSC_BOOL 3U
#define MIRSC_SINT 2U
#define MIRSC_DOUBLE 1U
#define MIRSC_STRING 0U

/*******************************************************************************
* Function Name: getMicroscanValue
********************************************************************************
* Summary:
*  Read a value from microscan on serial (iterative function)
*
* Parameters:
*   unsigned char * nameOfTag, uint8_t portNo, microScanGet *getReq
*
* Return:
* return nothing
* ==============================================================================
*
* example below of use (function iterative receives on interrupt which sets MIRSC_CMD_REPLY)
*
* microScanGet_t getRequest;
* getRequest.type=MIRSC_SINT;
* getRequest.state=MIRSC_CMD_SEND;
* if ((getRequest.state!=MIRSC_CMD_COMPLETE)||(getRequest.state!=MIRSC_CMD_FAIL))
*    getMicroScanValue("int_Value_3",4u,&getRequest);
*******************************************************************************/
MIRSC_LIB void getMicroscanValue( unsigned char * nameOfTag, uint8_t portNo, microScanGet_t *getReq )
{
   char *Buf[64u];
   uint8_t noOfRetry=0u;

   switch(getReq->state)                                                        // the state of the request
   {
      case MIRSC_CMD_SEND:                                                      // message to send send it
      MIRSC_GET((char*)Buf,nameOfTag);                                          // get with the tag specified
      Buf[MIRSC_GET_LEN+strlen(nameOfTag)]='\0';                                // terminate string
          switch ( portNo )                                                     // choose the serial port and send it
          {
             case 1u:
             Uart1_write_text( (char*) Buf );                                   // write to the chosen port
             break;

             case 2u:
             Uart2_write_text( (char*) Buf );
             break;

             case 3u:
             Uart3_write_text( (char*) Buf );
             break;

             case 4u:
             Uart4_write_text( (char*) Buf );
             break;

             case 5u:
             Uart5_write_text( (char*) Buf );
             break;

             case 6u:
             Uart6_write_text( (char*) Buf );
             break;

             default:
             Uart6_write_text( (char*) Buf );
             break;
      }
      getReq->tmr=0u;
      getReq->state=MIRSC_CMD_SENT;                                             // sent the request now wait for reply or timeour
      break;

      case MIRSC_CMD_SENT:                                                      // sent awaiting reply
      getReq->tmr=++getReq->tmr % UINT16_MAX;
      if (getReq->tmr>=MIRSC_RETRY_TIME)                                        // timeout with no response back
      {
         noOfRetry=++noOfRetry % UINT8_MAX;                                     // count how many times we tried, after max, wait for acknowledge from hmi
         getReq->tmr=0u;
         if (noOfRetry>=MIRSC_MAX_NUM_RETRY)
           getReq->state=MIRSC_MAX_RETRY_EXCEED;                                // retry exceeded await top end decision
         else
           getReq->state = MIRSC_CMD_SEND;                                      // re-send the request no response in time period
      }
      break;
     //-----------------------
      case MIRSC_CMD_REPLY:                                                     // interrupt messge back received
      if (strncmp(getReq->buf,MIRSC_SET_FAIL,6u))                               // reply wasnt !ERROR
      {
         switch(getReq->type)
         {
            case MIRSC_SINT:                                                    // signed int 32
            getReq->value.sint=atoi(getReq->buf);
            break;

            case MIRSC_DOUBLE:                                                  // double float64_t
            getReq->value.doub=atof(getReq->buf);
            break;

            case MIRSC_STRING:                                                  // string
            strcpy(getReq->value.str,getReq->buf);
            break;

            case MIRSC_BOOL:                                                    // boolean
            getReq->value.boolean=(!strcmp(getReq->buf,"1"));
            break;

            default:                                                            // choose string then
            getReq->type=MIRSC_STRING;
            break;
        }
        getReq->state=MIRSC_CMD_COMPLETE;                                       // complete await next request from HMI
      }
      else
      {
        getReq->state=MIRSC_CMD_FAIL;
      }
      break;

      case MIRSC_MAX_RETRY_EXCEED:                                              // do nothing unless removed and acknowledged
      break;

      case MIRSC_CMD_FAIL:                                                      // do nothing unless removed and acknowledged
      break;

      case MIRSC_CMD_COMPLETE:                                                  // do nothing unless removed and acknowledged
      break;

      default:                                                                  // unknown go to send
      getReq->state=MIRSC_CMD_SEND;
      break;
   }
}

/*******************************************************************************
* Function Name: setMicroscanValue
********************************************************************************
* Summary:
*  Set a value in microscan on serial (iterative function)
*
* Parameters:
*   unsigned char * nameOfTag, uint8_t portNo, microScanGet *getReq
*
* Return:
* return nothing
* ==============================================================================
*
* example below of use (function iterative receives on interrupt which sets MIRSC_CMD_REPLY)
*
* microScanGet_t getRequest;
* getRequest.type=MIRSC_SINT;
* getRequest.state=MIRSC_CMD_SEND;
* getRequest.value.sint = 893456l;
* if ((getRequest.state!=MIRSC_CMD_COMPLETE)||(getRequest.state!=MIRSC_CMD_FAIL))
*    setMicroScanValue("int_Value_3",4u,&getRequest);
*******************************************************************************/
MIRSC_LIB void setMicroScanValue( unsigned char * nameOfTag, uint8_t portNo, microScanGet_t *getReq )
{
   char *Buf[64u];
   uint8_t noOfRetry=0u;
   unsigned char* token;                                                        // what is extracted from the , field
   uint8_t fieldNo=0u;

   if ((getReq == NULL) && (nameOfTag==NULL))                                   // check for NULL pointers
   {
       return;
   }
   
   switch(getReq->state)                                                        // the state of the request
   {
      case MIRSC_CMD_SEND:                                                      // message to send send it
      switch(getReq->type)
      {
         case MIRSC_SINT:                                                       // signed int 32
         MIRSC_SET_INT((char*)Buf,nameOfTag,getReq->value.sint);
         break;

         case MIRSC_DOUBLE:                                                     // double float64_t
         MIRSC_SET_FLOAT((char*)Buf,nameOfTag,getReq->value.doub);
         break;

         case MIRSC_STRING:                                                     // string
         MIRSC_SET_STR((char*)Buf,nameOfTag,getReq->value.str);
         break;

         case MIRSC_BOOL:                                                       // boolean
         MIRSC_SET_BOOL((char*)Buf,nameOfTag,getReq->value.boolean);
         break;
         
         case MIRSC_LOAD_CONF:
         MIRSC_LOAD_JOB((char*)Buf,getReq->value.slot);                         /* load the configuration for this application from memory */
         break;
         
         default:                                                               // choose string then
         getReq->type=MIRSC_STRING;
         break;
      }
      switch ( portNo )                                                         // choose the serial port and send it
      {
         case 1u:
         Uart1_write_text((char*) Buf );
         break;

         case 2u:
         Uart2_write_text((char*) Buf );
         break;

         case 3u:
         Uart3_write_text((char*) Buf );
         break;

         case 4u:
         Uart4_write_text((char*) Buf );
         break;

         case 5u:
         Uart5_write_text((char*) Buf );
         break;

         case 6u:
         Uart6_write_text((char*) Buf );
         break;

         default:
         Uart6_write_text((char*) Buf );
         break;
      }
      getReq->tmr=0u;
      getReq->state=MIRSC_CMD_SENT;                                             // sent the request now wait for reply or timeour
      break;

      case MIRSC_CMD_SENT:                                                      // sent awaiting reply
      if (getReq->type == MIRSC_LOAD_CONF)                                      // configuration doesnt give reply ? just complete it
      {
          getReq->state=MIRSC_CMD_COMPLETE;                                     // complete await next request from HMI
      }
      getReq->tmr=++getReq->tmr % UINT16_MAX;
      if (getReq->tmr>=MIRSC_RETRY_TIME)
      {
        noOfRetry=++noOfRetry % UINT8_MAX;
        getReq->tmr=0u;
        if (noOfRetry>=MIRSC_MAX_NUM_RETRY)
           getReq->state=MIRSC_MAX_RETRY_EXCEED;                                // retry exceeded await top end decision
         else
           getReq->state = MIRSC_CMD_SEND;                                      // re-send the request no response in time period
      }
      break;

      case MIRSC_CMD_UNKNOWN:                                            // unkown wait for another interrupt
      getReq->state=MIRSC_CMD_SENT;                                             // wait for a new reply
      break;

      case MIRSC_CMD_REPLY:                                                     // interrupt messge back received
      if (strncmp(getReq->buf,MIRSC_SET_FAIL,6u))                               // reply wasnt !ERROR
      {
         if (!strncmp(getReq->buf,MIRSC_SET_OK,3u))                             // reply was !OK
         {
            token = strtok(getReq->buf, " ");                                   // Returns first token that was preceeding the comma
            if (token == NULL) getReq->state=MIRSC_CMD_UNKNOWN;                 // unknown response log it and come back
            while (token != NULL)                                               // read each delimetered field in the string
            {
               switch (fieldNo)
               {
                  case 0u:                                                      // 1st case !OK
                  token = strtok(NULL, " ");                                    // split next field separated by space
                  fieldNo=++fieldNo % UINT8_MAX;
                  break;

                  case 1u:                                                      // 2nd case SET
                  if (!strncmp(getReq->buf,"SET",3u))
                  {
                    token = strtok(NULL, " ");                                  // split next field separated by space
                    fieldNo=++fieldNo % UINT8_MAX;
                  }
                  else
                  {
                     getReq->state=MIRSC_CMD_UNKNOWN;                           // log unknown and look for next message
                     token = NULL;
                  }
                  break;

                  case 2u:                                                      // 3rd case tagname
                  if (!strncmp(getReq->buf,nameOfTag,strlen(nameOfTag)))
                  {
                      getReq->state=MIRSC_CMD_COMPLETE;                          // complete await next request from HMI
                  }
                  else
                  {
                      getReq->state=MIRSC_CMD_UNKNOWN;                          // unknown response log it and come back
                  }
                  token = NULL;
                  break;

                  default:
                  token = strtok(NULL, " ");                                    // split next field separated by space
                  fieldNo=2u;
                  break;
              }
          }
        }
        else
        {
           getReq->state=MIRSC_CMD_UNKNOWN;                                      // unknown response log it and come back
        }
    }
    else
    {
        getReq->state=MIRSC_CMD_FAIL;                                           // unknown response
    }
    break;

    case MIRSC_MAX_RETRY_EXCEED:                                                // do nothing unless removed and acknowledged
    break;

    case MIRSC_CMD_FAIL:                                                        // do nothing unless removed and acknowledged
    break;

    case MIRSC_CMD_COMPLETE:                                                    // do nothing unless removed and acknowledged
    break;

    default:                                                                    // unknown go to send
    getReq->state=MIRSC_CMD_SEND;
    break;
  }
}
/*******************************************************************************
* Function Name: setupMiScCamLRFB
********************************************************************************
* Summary:
*  Set up microscan cameras front and back
*
* Parameters:
*   microScanGet_t *getRequest
*
* Return:
* return uint8_t  MIRSC_CMD_COMPLETE or MIRSC_CMD_FAIL
* ==============================================================================
*
* example below of use (function iterative receives on interrupt which sets MIRSC_CMD_REPLY)
*
*  microScanGet_t getRequest;
*  setupMiScCamLRFB( &getRequest );
*
*******************************************************************************/
MIRSC_LIB uint8_t setupMiScCamLRFB( microScanGet_t *getRequest )
{

/* ===================== set up camera No.1 LR =============================== */
        getRequest->type=MIRSC_LOAD_CONF;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.slot = TreeBeaconSlot1;
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))  /* this check is disabled in the command for now but left in case you should check error code (do during test) */
        {
          setMicroScanValue("anything",MiScanUART_LR,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
        getRequest->type=MIRSC_SINT;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.sint = 200;
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))
        {
          setMicroScanValue(MIRSC_MAX_HT_INTEL,MiScanUART_LR,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
        getRequest->type=MIRSC_SINT;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.sint = 200;
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))
        {
          setMicroScanValue(MIRSC_MAX_WDTH_INTEL,MiScanUART_LR,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
           getRequest->type=MIRSC_SINT;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.sint = 0;
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))
        {
          setMicroScanValue(MIRSC_MIN_HT_INTEL,MiScanUART_LR,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
        getRequest->type=MIRSC_SINT;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.sint = 0;
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))
        {
          setMicroScanValue(MIRSC_MIN_WDTH_INTEL,MiScanUART_LR,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
           getRequest->type=MIRSC_DOUBLE;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.sint = 0.3f;                                          /* relativey wide confidence on the match set as you wish */
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))
        {
          setMicroScanValue(MIRSC_CONFIDNCE_INTEL,MiScanUART_LR,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
/* ===================== set up camera No.2 FB =============================== */
        getRequest->type=MIRSC_LOAD_CONF;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.slot = TreeBeaconSlot2;
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))
        {
          setMicroScanValue("anything",MiScanUART_FB,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
        getRequest->type=MIRSC_SINT;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.sint = 200;
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))
        {
          setMicroScanValue(MIRSC_MAX_HT_INTEL,MiScanUART_FB,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
        getRequest->type=MIRSC_SINT;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.sint = 200;
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))
        {
          setMicroScanValue(MIRSC_MAX_WDTH_INTEL,MiScanUART_FB,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
           getRequest->type=MIRSC_SINT;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.sint = 0;
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))
        {
          setMicroScanValue(MIRSC_MIN_HT_INTEL,MiScanUART_FB,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
        getRequest->type=MIRSC_SINT;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.sint = 0;
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))
        {
          setMicroScanValue(MIRSC_MIN_WDTH_INTEL,MiScanUART_FB,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
           getRequest->type=MIRSC_DOUBLE;
        getRequest->state=MIRSC_CMD_SEND;
        getRequest->value.sint = 0.3f;                                          /* relativey wide confidence on the match set as you wish */
        while ((getRequest->state!=MIRSC_CMD_COMPLETE)||(getRequest->state!=MIRSC_CMD_FAIL))
        {
          setMicroScanValue(MIRSC_CONFIDNCE_INTEL,MiScanUART_FB,getRequest);
        }
        if (getRequest->state==MIRSC_CMD_FAIL) return MIRSC_CMD_FAIL;           /* if we got an !ERROR response then return an error */
        return MIRSC_CMD_COMPLETE;                                              /* complete without error */
}

MIRSC_LIB void moveAccording2pArea( const ocr_match_select_t plantingArea[NUM_OF_PLANT_SECTION], miCrosCanConnect_t connCameraOut, ocr_movement_obj_t *moveAction )
{
    unsigned char strLeftRight[15u];                                            /* string read by left or right camera */
    unsigned char strFrontBack[15u];                                            /* string read by front or back camera */
    uint8_t area;

    if (moveAction==NULL)
      return;
                                                                                /* read the camera output data either from tcp or serial as per set-up defintions */
#if defined(MICROSCAN_OUT_TCP)
    memcpy((void*) &connCameraOut.buf,(void*) &MiCroScaN_DATA.TCP_Buffer, MiCroScaN_DATA.TCP_Buffer_Len);
    parse_microScan( &connCameraOut );
    if ((connCameraOut.ocr1_stat==true) && (connCameraOut.camera_stat_lr==true))
    strcpy(strLeftRight,&connCameraOut.beaconLR);
    if ((connCameraOut.ocr2_stat==true) && (connCameraOut.camera_stat_fb==true))
    strcpy(strLeftRight,&connCameraOut.beaconFB);
#elif defined(MICROSCAN_OUT_UART)
    memcpy((void*) &connCameraOut.buf,(void*) &MiCroScaN_SER_LR.Buffer, MiCroScaN_DATA.Len);
    parse_microScan( &connCameraOut );
    if ((connCameraOut.ocr1_stat==true) && (connCameraOut.camera_stat_lr==true))
    strcpy(strLeftRight,&connCameraOut.beaconLR);
    memcpy((void*) &connCameraOut.buf,(void*) &MiCroScaN_SER_FB.Buffer, MiCroScaN_DATA.Len);
    parse_microScan( &connCameraOut );
    if ((connCameraOut.ocr2_stat==true) && (connCameraOut.camera_stat_fb==true))
    strcpy(strFrontBack,&connCameraOut.beaconFB);
#endif

        for (area=0u;area<NUM_OF_PLANT_SECTION;area++)                          /* for each area defined as a tree plant search for the matching signs or positions */
        {
                switch(plantingArea[area].numMatchActiv)                        /* check the type of infomation on co-oridnates for each plant marker (location in area) */
                {
                        case OCR_MATCH_2LRFB:                                   /* ocr is matching left/right and front/back markers */
                        if ((!strcmp((char*)plantingArea[area].ocrRiLeMatchStr,(char*)strLeftRight) || (moveAction->matchLefRig==true)) && (!strcmp((char*)plantingArea[area].ocrFrBaMatchStr,(char*)strFrontBack) || (moveAction->matchFwdBack==true)))
                        {
                                area= NUM_OF_PLANT_SECTION;
                                moveAction->fwdBack= __roStop;
                                moveAction->leftRig= __roStop;
                                moveAction->matchLefRig= false;
                                moveAction->matchFwdBack = false;
                        }
                        else if ((!strcmp((char*)plantingArea[area].ocrRiLeMatchStr,(char*)strLeftRight) || (moveAction->matchLefRig==true)) && strcmp((char*)plantingArea[area].ocrFrBaMatchStr,(char*)strFrontBack))
                        {
                                moveAction->fwdBack= __roStop;
                                moveAction->leftRig= Rig;
                                if ((moveAction->RightStop == true) && (moveAction->LeftStop == false))          /* gone too far right left go left to look for marker */
                                {
                                   moveAction->leftRig= Lef;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == false))         /* gone too far left now go right to look for the marker */
                                {
                                   moveAction->leftRig= Rig;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == true))
                                {
                                   moveAction->leftRig= __roStop;                   /* malfunction stop */
                                }
                                if (moveAction->matchLefRig==false)
                                {
                                   moveAction->turnby90= true;
                                }
                                else
                                {
                                   moveAction->turnby90= false;
                                }
                                moveAction->matchLefRig= true;
                        }
                        else if (strcmp((char*)plantingArea[area].ocrRiLeMatchStr,(char*)strLeftRight) && (!strcmp((char*)plantingArea[area].ocrFrBaMatchStr,(char*)strFrontBack) || (moveAction->matchFwdBack==true)))
                        {
                                moveAction->fwdBack= fwd;
                                moveAction->leftRig= __roStop;
                                if ((moveAction->FwdStop == true) && (moveAction->RevStop == false))
                                {
                                   moveAction->fwdBack= Back;                                            /* gone far enough forward so go back to see if you can find the marker */
                                }
                                else if ((moveAction->RevStop == true) && (moveAction->FwdStop == false))
                                {
                                   moveAction->fwdBack= fwd;                                             /* gone far enough back so now go forward to look for the marker */
                                }
                                else if ((moveAction->FwdStop == true) && (moveAction->RevStop == true))
                                {
                                   moveAction->fwdBack= __roStop;                   /* malfunction stop */
                                }
                                if (moveAction->matchFwdBack==false)
                                {
                                   moveAction->turnby90= true;
                                }
                                else
                                {
                                   moveAction->turnby90= false;
                                }
                                moveAction->matchFwdBack = true;
                        }
                        else { /* just for sanity */ }
                        break;

                        case OCR_MATCH_2LR_TOP1ST:   /* ocr is matching 2 left/ right indicators the top one in the structure is 1st */
                        if (!strcmp((char*)plantingArea[area].ocrRiLeMatchStr,(char*)strLeftRight) || (moveAction->matchLefRig==false))
                        {
                                moveAction->fwdBack= __roStop;
                                moveAction->leftRig= Rig;
                                if ((moveAction->RightStop == true) && (moveAction->LeftStop == false))          /* gone too far right left go left to look for marker */
                                {
                                   moveAction->leftRig= Lef;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == false))         /* gone too far left now go right to look for the marker */
                                {
                                   moveAction->leftRig= Rig;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == true))
                                {
                                   moveAction->leftRig= __roStop;                   /* malfunction stop */
                                }
                                if (moveAction->matchLefRig==false)
                                {
                                   moveAction->turnby90= true;
                                }
                                else
                                {
                                   moveAction->turnby90= false;
                                }
                                moveAction->matchLefRig= true;
                        }
                        else if (!strcmp((char*)plantingArea[area].ocrFrBaMatchStr,(char*)strLeftRight) || (moveAction->matchLefRig==true))
                        {
                                moveAction->fwdBack= __roStop;
                                moveAction->leftRig= __roStop;
                                if ((moveAction->RightStop == true) && (moveAction->LeftStop == false))          /* gone too far right left go left to look for marker */
                                {
                                   moveAction->leftRig= Lef;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == false))         /* gone too far left now go right to look for the marker */
                                {
                                   moveAction->leftRig= Rig;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == true))
                                {
                                   moveAction->leftRig= __roStop;                   /* malfunction stop */
                                }
                        }
                        else { /* just for sanity */ }
                        break;

                        case OCR_MATCH_2LR_BOT1ST:   /* ocr is matching 2 left/ right indicators the top one in the structure is 1st */
                        if (!strcmp((char*)plantingArea[area].ocrRiLeMatchStr,(char*)strLeftRight) || (moveAction->matchLefRig==true))
                        {
                                moveAction->fwdBack= __roStop;
                                moveAction->leftRig= Rig;
                                if ((moveAction->RightStop == true) && (moveAction->LeftStop == false))          /* gone too far right left go left to look for marker */
                                {
                                   moveAction->leftRig= Lef;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == false))         /* gone too far left now go right to look for the marker */
                                {
                                   moveAction->leftRig= Rig;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == true))
                                {
                                   moveAction->leftRig= __roStop;                   /* malfunction stop */
                                }
                                if (moveAction->matchLefRig==false)
                                {
                                   moveAction->turnby90= true;
                                }
                                else
                                {
                                   moveAction->turnby90= false;
                                }
                                moveAction->matchLefRig= true;
                        }
                        else if (!strcmp((char*)plantingArea[area].ocrFrBaMatchStr,(char*)strLeftRight) || (moveAction->matchLefRig==false))
                        {
                                moveAction->fwdBack= __roStop;
                                moveAction->leftRig= __roStop;
                                if ((moveAction->RightStop == true) && (moveAction->LeftStop == false))          /* gone too far right left go left to look for marker */
                                {
                                   moveAction->leftRig= Lef;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == false))         /* gone too far left now go right to look for the marker */
                                {
                                   moveAction->leftRig= Rig;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == true))
                                {
                                   moveAction->leftRig= __roStop;                   /* malfunction stop */
                                }
                        }
                        else { /* just for sanity */ }
                        break;

                        case OCR_MATCH_2LR_ANYORDER:                    /* ocr is matching left/right and front/back markers */
                        if ((!strcmp((char*)plantingArea[area].ocrRiLeMatchStr,(char*)strLeftRight) || (moveAction->matchLefRig==true)) && (!strcmp((char*)plantingArea[area].ocrFrBaMatchStr,(char*)strLeftRight) || (moveAction->matchFwdBack==true)))
                        {
                                area= NUM_OF_PLANT_SECTION;
                                moveAction->fwdBack= __roStop;
                                moveAction->leftRig= __roStop;
                                moveAction->matchLefRig= false;
                                moveAction->matchFwdBack = false;
                        }
                        else if ((!strcmp((char*)plantingArea[area].ocrRiLeMatchStr,(char*)strLeftRight) || (moveAction->matchLefRig==true)) && strcmp((char*)plantingArea[area].ocrFrBaMatchStr,(char*)strLeftRight))
                        {
                                moveAction->fwdBack= __roStop;
                                moveAction->leftRig= Rig;
                                if ((moveAction->RightStop == true) && (moveAction->LeftStop == false))          /* gone too far right left go left to look for marker */
                                {
                                   moveAction->leftRig= Lef;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == false))         /* gone too far left now go right to look for the marker */
                                {
                                   moveAction->leftRig= Rig;
                                }
                                else if ((moveAction->LeftStop == true) && (moveAction->RightStop == true))
                                {
                                   moveAction->leftRig= __roStop;                   /* malfunction stop */
                                }
                                if (moveAction->matchLefRig==false)
                                {
                                   moveAction->turnby90= true;
                                }
                                else
                                {
                                   moveAction->turnby90= false;
                                }
                                moveAction->matchLefRig= true;
                        }
                        else if (strcmp((char*)plantingArea[area].ocrRiLeMatchStr,(char*)strLeftRight) && (!strcmp((char*)plantingArea[area].ocrFrBaMatchStr,(char*)strLeftRight) || (moveAction->matchFwdBack==true)))
                        {
                                moveAction->fwdBack= fwd;
                                moveAction->leftRig= __roStop;
                                if ((moveAction->FwdStop == true) && (moveAction->RevStop == false))
                                {
                                   moveAction->fwdBack= Back;                   /* gone far enough forward so go back to see if you can find the marker */
                                }
                                else if ((moveAction->RevStop == true) && (moveAction->FwdStop == false))
                                {
                                   moveAction->fwdBack= fwd;                    /* gone far enough back so now go forward to look for the marker */
                                }
                                else if ((moveAction->FwdStop == true) && (moveAction->RevStop == true))
                                {
                                   moveAction->fwdBack= __roStop;                   /* malfunction stop */
                                }
                                if (moveAction->matchFwdBack==false)
                                {
                                   moveAction->turnby90= true;
                                }
                                else
                                {
                                   moveAction->turnby90= false;
                                }
                                moveAction->matchFwdBack = true;
                        }
                        else { /* just for sanity */ }
                        break;

                        default:
                        break;
           }
     }
}

#ifdef __cplusplus
}
#endif

#endif