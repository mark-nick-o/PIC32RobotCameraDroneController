#ifndef __GSM_Modem_
#define __GSM_Modem_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define GSMPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define GSMPACKED __attribute__((packed))                                     /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define GSMPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define GSMPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define GSMPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define GSM_MODEM_RETRY_TIM 2000ul                                              /* time in ticks before we resend the command to the modem */

/* for SMS text message send */
#define MODEM_CMD1 "AT+CMGF=1"                                                  // to intialise the modem
#define MODEM_CMD2 "AT+CMGS="                                                   // start the sms text

#define MODEM_CMD_AT "AT"                                                       // to intialise the modem step1
#define MODEM_REPLY_AT "OK"                                                     // reply to intialise the modem step1
#define MODEM_CMD_CMGF "AT+CMGF=1"                                              // to intialise the modem step2
#define MODEM_REPLY_CMGF "OK"                                                   // reply to intialise the modem step2
#define MODEM_CMD_CNMI "AT+CNMI=1,1,0,0,0"                                      // to intialise the modem step3
#define MODEM_REPLY_CNMI "OK"                                                   // reply to intialise the modem step3
#define MODEM_CMD_CPMS "AT+CPMS=\"SM\""                                         // to intialise the modem step4
#define MODEM_REPLY_CPMS "+CPMS:"                                               // reply to intialise the modem step4
/* ===== reading message ========= */
#define MODEM_NEW_SMS "+CMTI:"                                                  // new sms arrived look for the index
#define MODEM_SMSREQMSG(A,index) do { sprintf(A,"AT+CMGR=%d",index); } while(0)    // request the message at the index recieved
#define MODEM_REPLY_CMGR "+CMGR:"                                               // reply to getting message of index recieved 

/* ===== writing message ========= */
#define MODEM_SMSTXTDIAL(A,telNo) do { sprintf(A,"AT+CMGS=\"%s\"\r\n",telNo); } while(0)    // write the telephone number and make a call
#define MODEM_ENTER 0x0Du                                                       // at end of each line
#define MODEM_QUOTE 0x22u                                                       // to quote around the recipients number
#define MODEM_CTLZ 0x1Au                                                        // used to end the text message send <CTRZ>
#define MODEM_SMSTXTEND(A,msg) do { sprintf(A,"%s %c\r\n",msg,MODEM_CTLZ); } while(0)    // end text portion of the message

#define MODEM_REPLY_ERASE "OK"
#define MODEM_SMSERASE(demande,index) do { sprintf(demande, "AT+CMGD=%d\r\n",index); } while(0) // AT+CMGD=1,4  --> erase all messages
#define MODEM_SMSERASEALL(demande) do { sprintf(demande, "AT+CMGD=1,4\r\n"); } while(0)  // AT+CMGD=1,4  --> erase all messages

/* for GPRS connection and http post for example */
#define MODEM_GPRS_INIT "AT"                                                    // reply is OK or ERROR
#define MODEM_GPRS_RETURN "OK"                                                  // standard reply
#define MODEM_GPRS_CMD1 "AT+CIPSHUT"                                            // reply OK or ERROR reset connection
#define MODEM_GPRS_CMD2 "AT+CIPHUX=0"                                           // reply OK or ERROR prepare simple connection
#define MODEM_GPRS_CMD3 "AT+CSTT=\"lte.avantel.com.co\",\"\",\"\""              // reply OK or ERROR change for you're apn
#define MODEM_GPRS_Q1 "AT+COQMIN=1,0,0,0,0,0"                                   // QOS quality of service ok
#define MODEM_GPRS_Q2 "AT+COQREQ=1,0,0,3,0,0"                                   // ok
#define MODEM_GPRS_Q3 "AT+CMGF=1"                                               // to intialise the modem  ok
#define MODEM_GPRS_Q4 "AT+CMMI=1,1,2,1,0"                                       // ok
#define MODEM_GPRS_L1 "AT+CIPSTATUS"                                            // analyse the status Now loop this forever until START either START or ERROR
#define MODEM_CIPSTATUS_RETURN "IP INITIAL"
#define MODEM_CIP_START_RETURN "START"
#define MODEM_CIP_ACT_RETURN "GPRSACT"
#define MODEM_GPRS_L2 "AT+CIICR"                                                // OK or ERROR
#define MODEM_GPRS_L3 "AT+CIPSR"                                                // reply is . or ERROR
#define MODEM_GPRS_L3_REPLY "."
#define MODEM_GPRS_START1 " \"AT+CIPSTART=\" \"TCP\" \",\"  \"things.ubidots.com\" \"80\" " // change for your url and port
#define MODEM_GPRS_SEND "AT+CIPSEND"                                            // sends data
// example now send a http post for example to this serial port
// POST url HHTP/1.1 Host : things.ubidots.com Context-Type : application/json Context-Length : 38
#define MODEM_GRPS_SEND_END 0x1Au                                               // used to end the text message send
#define MODEM_GRPS_CLOSE "AT+CIPCLOSE"                                          // end and close the gprs tcp socket

/* define your connection details */
#define MY_GPRS_APN "web.colombiamovil.com.co"                                  /* change to youre GPRS APN */
#define MY_GPRS_USER "mrX"
#define MT_GPRS_PASSWORD "Xrm"

typedef enum {
  _INIT = 0,
  _AT = 1,
  _CMGF        = 2,
  _CNMI        = 3,
  _CPMS        = 4,
 _FINCONF = 5,
 _RECEPTION_SMS = 6,
 _SUPPR_SMS = 7,
 _NEW_SMS_PARSED = 8
} smsRcvState_e;                                                                /* modem receiver states */

typedef enum {
  _NOSEND = 0,
  _DIALING = 1,
  _SENDING = 2,
  _READY_TO_SEND = 3
} smsSndState_e;                                                                /* modem receiver state */

#if defined(D_FT900)
typedef struct GSMPACKED {
  char info_SMS_recu[20u];
  char demande_lecture[20u];                                                    /* modem command out to the serial port uart connected to the modem */
  char reponse_attendue[10u];                                                   /* chars recieved back in relation to the modem command */
  char buffer[450u];                                                            /* interrupt raw chars recieved from modem */
  uint8_t wait_reponse : 1u; 
  uint8_t GSM_reponse : 1u;
  uint8_t wait_prompt : 3u;
  uint8_t new_send : 1u;
  uint8_t stepInit : 1u;
  uint8_t spare : 1u;
  uint8_t fct_en_cours;                                                         /* modem state machine */
  int16_t index_msg;
  char flag[11u];
  char expediteur[12u];                                                         /* senders telephone number */
  char receiver[12u];                                                           /* reeceivers telephone number */
  char dateheure[20u];
  char dataV[160u];
  char dataS[165u];                                                             /* tert to send */
  int32_t ticksVal;
  uint32_t ticksRef;
  int32_t sntTicksVal;
  uint32_t sntTicksRef;
} sms_rcv_t;
#else
GSMPACKED(
typedef struct {
  char info_SMS_recu[20u];
  char demande_lecture[20u];                                                    /* modem command out to the serial port uart connected to the modem */
  char reponse_attendue[10u];                                                   /* chars recieved back in relation to the modem command */
  char buffer[450u];                                                            /* interrupt raw chars recieved from modem */
  uint8_t wait_reponse : 1u; 
  uint8_t GSM_reponse : 1u;
  uint8_t wait_prompt : 3u;
  uint8_t new_send : 1u;
  uint8_t stepInit : 1u;
  uint8_t spare : 1u;
  uint8_t fct_en_cours;                                                         /* modem state machine */
  int16_t index_msg;
  char flag[11u];
  char expediteur[12u];                                                         /* senders telephone number */
  char receiver[12u];                                                           /* reeceivers telephone number */
  char dateheure[20u];
  char dataV[160u];                                                             /* incoming text */
  char dataS[165u];                                                             /* tert to send */
  int32_t ticksVal;                                                             /* receiver initialise timers */
  uint32_t ticksRef;
  int32_t sntTicksVal;                                                          /* sender timers */
  uint32_t sntTicksRef;
}) sms_rcv_t;
#endif

typedef enum {
  _BEGIN = 0,
  _GPRS_AT = 1,
  _GPRS1 = 2,
  _GPRS2 = 3,
  _GPRS_L1 = 4,
 _GPRS3 = 5,
 _GPRSQ1 = 6,
 _GPRSQ2 = 7,
 _GPRSQ3 = 8,
 _GPRSQ4 = 9,
 _GPRSStart = 10,
 _GPRS_L2 = 11,
 _GPRS_Act = 12,
 _GPRS_Delay1 = 13,
 _GPRS_L3 = 14,
 _GPRS_READY = 15 
} gprsModemState_e;                                                                /* GPRS set-up states */

#if defined(D_FT900)
typedef struct GSMPACKED {
  char demande_lecture[20u];                                                    /* modem command out to the serial port uart connected to the modem */
  char reponse_attendue[10u];                                                   /* chars recieved back in relation to the modem command */
  char buffer[450u];                                                            /* interrupt raw chars recieved from GPRS connection over modem serial */
  uint8_t wait_reponse : 1u; 
  uint8_t GPRS_reponse : 1u;
  uint8_t stepInit : 1u;
  uint8_t spare : 5u;
  uint8_t fct_en_cours;                                                         /* modem state machine */
  int32_t ticksVal;
  uint32_t ticksRef;
} gprs_rcv_t;
#else
GSMPACKED(
typedef struct {
  char demande_lecture[20u];                                                    /* modem command out to the serial port uart connected to the modem */
  char reponse_attendue[10u];                                                   /* chars recieved back in relation to the modem command */
  char buffer[450u];                                                            /* interrupt raw chars recieved from GPRS connection over modem serial */
  uint8_t wait_reponse : 1u; 
  uint8_t GPRS_reponse : 1u;
  uint8_t stepInit : 1u;
  uint8_t spare : 5u;
  uint8_t fct_en_cours;                                                         /* modem state machine */
  int32_t ticksVal;
  uint32_t ticksRef;
}) gprs_rcv_t;
#endif

// Return the position of the ct string from the message cs
int16_t str_istr(const char *cs, const char *ct)
{
   int16_t index = -1;
   char *ptr_pos;
   
   if (cs != NULL && ct != NULL)
   {
      ptr_pos = NULL;

      ptr_pos = strstr((char*)cs,(char*)ct);
      if (ptr_pos != NULL)
      {
         index = ((int16_t)ptr_pos) - ((int16_t)cs);
      }
   }
   return index;
}

// Function indicating the presense of chaine_a_trouver (string you want to look for) in chaine_source (message)
int16_t locate(char chaine_a_trouver[], sms_rcv_t *smsBuf)
{
        int16_t i=0, indice_dbt = -1;
        char buffer[20u];

        if (strstr(&smsBuf->buffer, chaine_a_trouver) != NULL)
        {
           if (strcmp(chaine_a_trouver, "+CMTI:") == 0)
           {
                if((indice_dbt = str_istr(&smsBuf->buffer, "+CMTI:")) == -1)
                     return 0;
                else
                {
                   for (i = 0; i < 15; i++)
                   {
                     buffer[i] = smsBuf->buffer[i+indice_dbt];
                   }
                   strcpy((char*)&smsBuf->info_SMS_recu,(char*)&buffer);
                }
            }
            return 1;
        }
        else
             return 0;
}

int16_t recherche_caractere(char* chaine, char caractere, int16_t nb_occ)
{
        int16_t nb_trouve = 0, i=0, longueur_chaine = strlen(chaine);

        while(i<longueur_chaine && nb_trouve < nb_occ)
        {
           if(chaine[i] == caractere)
           {
              nb_trouve=++nb_trouve % INT16_MAX;
           }
           i=++i % INT16_MAX;
        }

        if (i<=longueur_chaine)
            return i-1;
        else
            return -1;
}

// Fonction d'extraction a partir de la chaine source de la ssous-chaine comprise entre char_dbt et char_fin, stockée dans destination
void Extraction(char source[], char char_dbt, int16_t nb_occurrence1, int16_t decalage1, char char_fin, int16_t nb_occurrence2, int16_t decalage2, char destination[])
{
        int16_t indice_debut, indice_fin, i, rch1, rch2;

          rch1 = recherche_caractere(source, char_dbt, nb_occurrence1);

        if(rch1 == -1)
                strncpy(destination, "0", 1);
        else
        {

          indice_debut = rch1 + decalage1;
          for(i=0; i<strlen(source); i++)
                source[i] = source[i+indice_debut];

          rch2 = recherche_caractere(source, char_fin, nb_occurrence2);

          if(rch2 == -1)
                  strncpy(destination, "0", 1);
          else
          {
                  indice_fin = rch2 + decalage2;
                  strncpy(destination, source, indice_fin);
                  destination[indice_fin] = '\0';
          }
        }
}

void sendCmd2Modem( int16_t handle, sms_rcv_t *smsBuf )
{
   switch( handle )
   {
      case 1u:
      if (smsBuf->wait_prompt = _SENDING)
         UART1_Write_Text( smsBuf->dataS );      
      else
         UART1_Write_Text( smsBuf->demande_lecture );
      break;
      
      case 2u:
      if (smsBuf->wait_prompt = _SENDING)
         UART2_Write_Text( smsBuf->dataS );      
      else
         UART2_Write_Text( smsBuf->demande_lecture );
      break;
      
      case 3u:
      if (smsBuf->wait_prompt = _SENDING)
         UART3_Write_Text( smsBuf->dataS );      
      else
         UART3_Write_Text( smsBuf->demande_lecture );
      break;
      
      case 4u:
      if (smsBuf->wait_prompt = _SENDING)
         UART4_Write_Text( smsBuf->dataS );      
      else
         UART4_Write_Text( smsBuf->demande_lecture );
      break;
      
      case 5u:
      if (smsBuf->wait_prompt = _SENDING)
         UART5_Write_Text( smsBuf->dataS );      
      else
         UART5_Write_Text( smsBuf->demande_lecture );
      break;
      
      case 6u:
      if (smsBuf->wait_prompt = _SENDING)
         UART6_Write_Text( smsBuf->dataS );      
      else
         UART6_Write_Text( smsBuf->demande_lecture );
      break;
      
      default:
      break;
   }
   if (!((smsBuf->wait_prompt == _DIALING) || (smsBuf->wait_prompt == _SENDING))) /* separate state engine for dialling out */
   {
         smsBuf->wait_reponse = 1u;
   }
}

void sendCmdGPRSModem( int16_t handle, gprs_rcv_t *smsBuf )
{
   switch( handle )
   {
      case 1u:
         UART1_Write_Text( smsBuf->demande_lecture );
      break;
      
      case 2u:
         UART2_Write_Text( smsBuf->demande_lecture );
      break;
      
      case 3u:
         UART3_Write_Text( smsBuf->demande_lecture );
      break;
      
      case 4u:
         UART4_Write_Text( smsBuf->demande_lecture );
      break;
      
      case 5u:
         UART5_Write_Text( smsBuf->demande_lecture );
      break;
      
      case 6u:
         UART6_Write_Text( smsBuf->demande_lecture );
      break;
      
      default:
      break;
   }
   smsBuf->wait_reponse = 1u;
}
// Fonction to ask for the SMS message content of the index recieved
void Recuperation_SMS(int16_t handle, sms_rcv_t *smsBuf)
{
        char chaine_extraite[4u];

        Extraction(smsBuf->info_SMS_recu, ',', 1, 1, '\0', 1, 0, chaine_extraite);
        smsBuf->index_msg = atoi(chaine_extraite);
        MODEM_SMSREQMSG(&smsBuf->demande_lecture,smsBuf->index_msg);
        strcpy(&smsBuf->reponse_attendue, MODEM_REPLY_CMGR);
        smsBuf->fct_en_cours = _RECEPTION_SMS;
        sendCmd2Modem( handle, smsBuf );
        smsBuf->ticksRef = CP0_GET(CP0_COUNT);
}

// Strips the phone number, flags content, datetime from the recieve buffer reponse_module
void Remplissage_SMS(sms_rcv_t *smsBuf)
{
        Extraction(&smsBuf->buffer, '"', 1, 1, '"', 1, 0, smsBuf->flag);         /* message flags */
        Extraction(&smsBuf->buffer, '"', 2, 1, '"', 1, 0, smsBuf->expediteur);   /* senders phone number */
         Extraction(&smsBuf->buffer, '"', 2, 1, '"', 1, 0, smsBuf->dateheure);    /* date and time */
        Extraction(&smsBuf->buffer, '"', 1, 3, '\0', 1, -3, smsBuf->dataV);      /* message content */
}

// fonction de traitement des données recues sur l'UART
void SMS_reponse(int16_t handle, sms_rcv_t *smsBuf, char reponse_module[])
{
        if(locate("+CMTI:", smsBuf) == 1)                               /* received a new sms message into smsBuf->buffer (which is filled by interrupt */
        {
                Recuperation_SMS(handle, smsBuf);
        }
        else if ((smsBuf->wait_prompt == _READY_TO_SEND) && (smsBuf->new_send == 1)) /* modem is ready and state is changed by the sender to write an sms */
        {
            MODEM_SMSTXTDIAL(&smsBuf->demande_lecture,smsBuf->receiver);        /* dial the number */
            smsBuf->wait_prompt = _DIALING;
            sendCmd2Modem( handle, smsBuf );
            smsBuf->sntTicksRef = CP0_GET(CP0_COUNT);
        }
        else if (smsBuf->wait_prompt == _DIALING)                               /* ready to send after dial */
        {
                calculateTick2Now( &smsBuf->sntTicksVal, &smsBuf->sntTicksRef );  /* calc new tick to now for the retry timer */ 
                if (locate(">", smsBuf) == 1)                                   /* dial ready */
                {
                   MODEM_SMSTXTEND(smsBuf->dataS,smsBuf->dataS);                /* send the message */
                    smsBuf->wait_prompt = _SENDING;
                   sendCmd2Modem( handle, smsBuf );
                     smsBuf->new_send = 0u;                                       /* complete */
                   smsBuf->wait_prompt = _READY_TO_SEND;                        /* go back to send ready state */
                }
                else
                {
                   if (smsBuf->sntTicksVal >= GSM_MODEM_RETRY_TIM)              /* timeout */
                      smsBuf->wait_prompt = _READY_TO_SEND;                     /* send again */
                }
        }
        else if (smsBuf->wait_reponse == 1u)                                    /* we sent something or recived something and wait for its reply */
        {
                if(locate(smsBuf->reponse_attendue, smsBuf) == 1)               /* look for expected modem command to come back and set response flag */
                {
                   smsBuf->GSM_reponse = 1;
                   smsBuf->wait_reponse = 0;
                     memset((void*)&smsBuf->demande_lecture,0,sizeof(smsBuf->demande_lecture));  /* blank send string for next send */
                   memset((void*)&smsBuf->reponse_attendue,0,sizeof(smsBuf->reponse_attendue));  /* blank response for next send */
                }
                else
                   smsBuf->GSM_reponse = 0;                                     /* wait for timeout before re-sending command */

                switch(smsBuf->fct_en_cours)                                    /* state engine */
                {
                        case _AT :                                              /* ====== init modem ====== */
                        calculateTick2Now( &smsBuf->ticksVal, &smsBuf->ticksRef );  /* calc new tick to now for the retry timer */         
                        if(smsBuf->GSM_reponse == 1)
                        {
                           strcpy(&smsBuf->demande_lecture,MODEM_CMD_CMGF);
                           strcpy(&smsBuf->reponse_attendue,MODEM_REPLY_CMGF);
                           sendCmd2Modem( handle, smsBuf );
                           smsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           smsBuf->fct_en_cours = _CMGF;                        
                        }
                        else
                        {
                           if (( smsBuf->ticksVal >= GSM_MODEM_RETRY_TIM ) || (smsBuf->stepInit == 0))
                           {
                             strcpy(&smsBuf->demande_lecture,MODEM_CMD_AT);
                             strcpy(&smsBuf->reponse_attendue,MODEM_REPLY_AT);
                             sendCmd2Modem( handle, smsBuf );
                             smsBuf->ticksRef = CP0_GET(CP0_COUNT);
                             smsBuf->stepInit = 1u;
                           }
                        }
                        break;

                        case _CMGF :                                            /* ====== 1st Step of SMS ====== */
                        calculateTick2Now( &smsBuf->ticksVal, &smsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                        if(smsBuf->GSM_reponse == 1)
                        {
                           strcpy(&smsBuf->demande_lecture,MODEM_CMD_CNMI);
                           strcpy(&smsBuf->reponse_attendue,MODEM_REPLY_CNMI);
                           sendCmd2Modem( handle, smsBuf );
                           smsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           smsBuf->fct_en_cours = _CNMI;                        
                        }
                        else
                        {
                           if ( smsBuf->ticksVal >= GSM_MODEM_RETRY_TIM )
                           {
                              strcpy(&smsBuf->demande_lecture,MODEM_CMD_CMGF);
                              strcpy(&smsBuf->reponse_attendue,MODEM_REPLY_CMGF);
                              sendCmd2Modem( handle, smsBuf );
                           }
                        }
                        break;
                        

                        case _CNMI :                                            /* ====== 2nd Step of SMS ====== */
                        calculateTick2Now( &smsBuf->ticksVal, &smsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                        if(smsBuf->GSM_reponse == 1)
                        {
                           strcpy(&smsBuf->demande_lecture,MODEM_CMD_CPMS);
                           strcpy(&smsBuf->reponse_attendue,MODEM_REPLY_CPMS);        
                           sendCmd2Modem( handle, smsBuf );
                           smsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           smsBuf->fct_en_cours = _CPMS;                                                   
                        }
                        else
                        {
                           if ( smsBuf->ticksVal >= GSM_MODEM_RETRY_TIM )
                           {
                              strcpy(&smsBuf->demande_lecture,MODEM_CMD_CNMI);
                              strcpy(&smsBuf->reponse_attendue,MODEM_REPLY_CNMI);
                              sendCmd2Modem( handle, smsBuf );
                           }        
                        }
                        break;                        
                        
                        case _CPMS :                                            /* ====== 3rd Step of SMS set-up ====== */
                        calculateTick2Now( &smsBuf->ticksVal, &smsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                        if(smsBuf->GSM_reponse == 1)
                        {
                           smsBuf->wait_reponse = 0u;                                 /* ===== SMS modem set-up is complete ==== */
                           smsBuf->wait_prompt = _READY_TO_SEND;                /* ===== enable send engine ======= */
                        }
                        else
                        {
                           if ( smsBuf->ticksVal >= GSM_MODEM_RETRY_TIM )
                           {
                              strcpy(&smsBuf->demande_lecture,MODEM_CMD_CPMS);
                              strcpy(&smsBuf->reponse_attendue,MODEM_REPLY_CPMS);
                              sendCmd2Modem( handle, smsBuf );
                           }                
                        }
                        break;
                        
                        case _SUPPR_SMS : 
                        if(smsBuf->GSM_reponse == 1)
                        {
                        
                        }                        
                        break;
                        
                        case _FINCONF :
                        break;

                        case _RECEPTION_SMS :                                   /* ==== 1st part SMS from index recieved at interrupt === */
                        calculateTick2Now( &smsBuf->ticksVal, &smsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                        if(smsBuf->GSM_reponse == 1)
                        {
                           Remplissage_SMS(smsBuf);                                /* ==== 2nd part is parse the packet into phone number etc === */
                           smsBuf->wait_reponse = 0u;                                 /* ===== SMS read of incoming packet is complete ==== */
                           smsBuf->fct_en_cours = _NEW_SMS_PARSED;                
                        }
                        else
                        {
                           if ( smsBuf->ticksVal >= GSM_MODEM_RETRY_TIM )
                           {
                               Recuperation_SMS(handle, smsBuf);                /* request the index number */
                           }                
                        }
                        break;
                                        
                        default : break;
                }
        }
        else
        {
           if (smsBuf->fct_en_cours == _AT)                                     /* initial send of AT command to the modem */
           {
                strcpy(&smsBuf->demande_lecture,MODEM_CMD_AT);
                strcpy(&smsBuf->reponse_attendue,MODEM_REPLY_AT);
                sendCmd2Modem( handle, smsBuf );
                smsBuf->ticksRef = CP0_GET(CP0_COUNT);
                smsBuf->stepInit = 1u;           
           }
        }

}


// fonction for control of the GPRS modem
void GPRS_reponse(int16_t handle, gprs_rcv_t *gprsBuf )
{

        if (gprsBuf->wait_reponse == 1u)                                        /* we sent something or recived something and wait for its reply */
        {
                if(strstr(&gprsBuf->reponse_attendue, &gprsBuf->buffer) != 0)             /* look for expected modem command to come back and set response flag */
                {
                   gprsBuf->GPRS_reponse = 1;
                   gprsBuf->wait_reponse = 0;
                   memset((void*)&gprsBuf->demande_lecture,0,sizeof(gprsBuf->demande_lecture));  /* blank send string for next send */
                   memset((void*)&gprsBuf->reponse_attendue,0,sizeof(gprsBuf->reponse_attendue));  /* blank response for next send */
                }
                else
                   gprsBuf->GPRS_reponse = 0;                                   /* wait for timeout before re-sending command */

                switch(gprsBuf->fct_en_cours)                                   /* state engine */
                {
                        case _GPRS_AT :                                              /* ====== init modem ====== */
                        calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */         
                        if(gprsBuf->GPRS_reponse == 1)
                        {
                           strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_CMD1);
                           strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);
                           sendCmdGPRSModem( handle, gprsBuf );
                           gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           gprsBuf->fct_en_cours = _GPRS1;                        
                        }
                        else
                        {
                           if (( gprsBuf->ticksVal >= 2000 ) || (gprsBuf->stepInit == 0))
                           {
                             strcpy(&gprsBuf->demande_lecture,MODEM_CMD_AT);
                             strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);
                             sendCmdGPRSModem( handle, gprsBuf );
                             gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                             gprsBuf->stepInit = 1u;
                           }
                        }
                        break;

                        case _GPRS1 :                                            /* ====== 1st Step of GPRS ====== */
                        calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                        if(gprsBuf->GPRS_reponse == 1)
                        {
                           strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_CMD2);
                           strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);
                           sendCmdGPRSModem( handle, gprsBuf );
                           gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           gprsBuf->fct_en_cours = _GPRS2;                        
                        }
                        else
                        {
                           if ( gprsBuf->ticksVal >= 2000 )
                           {
                                                     gprsBuf->fct_en_cours = _GPRS_AT; 
                             gprsBuf->stepInit = 0u;
                           }
                        }
                        break;
                        

                        case _GPRS2 :                                            /* ====== 2nd Step of GPRS ====== */
                        calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                        if(gprsBuf->GPRS_reponse == 1)
                        {
                           strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_L1);
                           strcpy(&gprsBuf->reponse_attendue,MODEM_CIPSTATUS_RETURN);        
                           sendCmdGPRSModem( handle, gprsBuf );
                           gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           gprsBuf->fct_en_cours = _GPRS_L1;                                                   
                        }
                        else
                        {
                           if ( gprsBuf->ticksVal >= 4000 )
                           {
                             gprsBuf->fct_en_cours = _GPRS_AT; 
                             gprsBuf->stepInit = 0u;
                           }        
                        }
                        break;

                        case _GPRS_L1 :                                            /* ====== try to conenct to the provider of service ====== */
                        calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                        if(gprsBuf->GPRS_reponse == 1)
                        {
                           strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_CMD3);
                           strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);        
                           sendCmdGPRSModem( handle, gprsBuf );
                           gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           gprsBuf->fct_en_cours = _GPRS3;                                                   
                        }
                        else
                        {
                           if ( gprsBuf->ticksVal >= 500 )
                           {
                                                     gprsBuf->fct_en_cours = _GPRS_AT; 
                             gprsBuf->stepInit = 0u;
                           }        
                        }
                        break;

                        case _GPRS3 :                                           
                        calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                        if(gprsBuf->GPRS_reponse == 1)
                        {
                           strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_Q1);
                           strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);        
                           sendCmdGPRSModem( handle, gprsBuf );
                           gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           gprsBuf->fct_en_cours = _GPRSQ1;                                                   
                        }
                        else
                        {
                           if ( gprsBuf->ticksVal >= 30000 )
                           {
                              gprsBuf->fct_en_cours = _GPRS_AT; 
                              gprsBuf->stepInit = 0u;
                           }        
                        }
                        break;                        
                        
                        case _GPRSQ1 :                                        
                        calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                        if(gprsBuf->GPRS_reponse == 1)
                        {
                           strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_Q2);
                           strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);        
                           sendCmdGPRSModem( handle, gprsBuf );
                           gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           gprsBuf->fct_en_cours = _GPRSQ2;   
                        }
                        else
                        {
                           if ( gprsBuf->ticksVal >= 500 )
                           {
                              strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_Q1);
                              strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);
                              sendCmdGPRSModem( handle, gprsBuf );
                           }                
                        }
                        break;

                       case _GPRSQ2 :                                           
                       calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                       if(gprsBuf->GPRS_reponse == 1)
                       {
                           strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_Q3);
                           strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);        
                           sendCmdGPRSModem( handle, gprsBuf );
                           gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           gprsBuf->fct_en_cours = _GPRSQ3;   
                       }
                       else
                       {
                           if ( gprsBuf->ticksVal >= 500 )
                           {
                              strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_Q2);
                              strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);
                              sendCmdGPRSModem( handle, gprsBuf );
                           }                
                        }
                        break;

                       case _GPRSQ3 :                                            
                       calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                       if(gprsBuf->GPRS_reponse == 1)
                       {
                           strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_Q4);
                           strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);        
                           sendCmdGPRSModem( handle, gprsBuf );
                           gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           gprsBuf->fct_en_cours = _GPRSQ4;   
                       }
                       else
                       {
                           if ( gprsBuf->ticksVal >= 500 )
                           {
                              strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_Q3);
                              strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);
                              sendCmdGPRSModem( handle, gprsBuf );
                           }                
                       }
                       break;        

                       case _GPRSQ4 :                                           
                       calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                       if(gprsBuf->GPRS_reponse == 1)
                       {
                           strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_L1);
                           strcpy(&gprsBuf->reponse_attendue,MODEM_CIP_START_RETURN);        
                           sendCmdGPRSModem( handle, gprsBuf );
                           gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           gprsBuf->fct_en_cours = _GPRSStart;   
                       }
                       else
                       {
                           if ( gprsBuf->ticksVal >= 500 )
                           {
                              strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_Q4);
                              strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);
                              sendCmdGPRSModem( handle, gprsBuf );
                           }                
                       }
                       break;        

                       case _GPRSStart :                                          
                       calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                       if(gprsBuf->GPRS_reponse == 1)
                       {
                           strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_L2);
                           strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_RETURN);        
                           sendCmdGPRSModem( handle, gprsBuf );
                           gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           gprsBuf->fct_en_cours = _GPRS_L2;   
                       }
                       else
                       {
                           if ( gprsBuf->ticksVal >= 3000 )
                           {
                              strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_L1);
                              strcpy(&gprsBuf->reponse_attendue,MODEM_CIP_START_RETURN);
                              sendCmdGPRSModem( handle, gprsBuf );
                           }                
                       }
                       break;        

                       case _GPRS_L2 :                                          
                       calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                       if(gprsBuf->GPRS_reponse == 1)
                       {
                           strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_L1);
                           strcpy(&gprsBuf->reponse_attendue,MODEM_CIP_ACT_RETURN);        
                           sendCmdGPRSModem( handle, gprsBuf );
                           gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                           gprsBuf->fct_en_cours = _GPRS_Act;   
                       }
                       else
                       {
                           if ( gprsBuf->ticksVal >= 30000 )
                           {
                              gprsBuf->fct_en_cours = _GPRS_AT; 
                              gprsBuf->stepInit = 0u;
                           }             
                       }
                       break;        

                       case _GPRS_Act :                                          
                       calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                       if(gprsBuf->GPRS_reponse == 1)
                       {
                           gprsBuf->fct_en_cours = _GPRS_Delay1;   
                       }
                       else
                       {
                           if ( gprsBuf->ticksVal >= 4000 )
                           {
                              strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_L1);
                              strcpy(&gprsBuf->reponse_attendue,MODEM_CIP_ACT_RETURN);
                              sendCmdGPRSModem( handle, gprsBuf );
                           }             
                       }
                       break;
                                                
                       case _GPRS_Delay1 :                                            
                       calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                       if ( gprsBuf->ticksVal >= 2000 )
                       {
                          strcpy(&gprsBuf->demande_lecture,MODEM_GPRS_L3);
                          strcpy(&gprsBuf->reponse_attendue,MODEM_GPRS_L3_REPLY);
                          sendCmdGPRSModem( handle, gprsBuf );
                          gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                          gprsBuf->fct_en_cours = _GPRS_L3;  
                       }             
                       break;

                       case _GPRS_L3 :                                           
                       calculateTick2Now( &gprsBuf->ticksVal, &gprsBuf->ticksRef );  /* calc new tick to now for the retry timer */ 
                       if(gprsBuf->GPRS_reponse == 1)
                       {
                           gprsBuf->wait_reponse = 0u;                          /* ===== GPRS modem set-up is complete ==== */
                           gprsBuf->fct_en_cours = _GPRS_READY;                 /* ===== enable send engine to communicate with internet ======= */
                       }
                       else
                       {
                           if ( gprsBuf->ticksVal >= 10000 )
                           {
                             gprsBuf->fct_en_cours = _GPRS_AT; 
                             gprsBuf->stepInit = 0u;
                           }                
                       }
                       break;                       
                                       
                       default : break;
                }
        }
        else
        {
           if (gprsBuf->fct_en_cours == _GPRS_AT)                               /* initial send of AT command to the modem */
           {
                strcpy(&gprsBuf->demande_lecture,MODEM_CMD_AT);
                strcpy(&gprsBuf->reponse_attendue,MODEM_REPLY_AT);
                sendCmdGPRSModem( handle, gprsBuf );
                gprsBuf->ticksRef = CP0_GET(CP0_COUNT);
                gprsBuf->stepInit = 1u;           
           }
        }

}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif