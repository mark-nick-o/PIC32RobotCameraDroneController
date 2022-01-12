#include "definitions.h"
#if defined(MARINE_PROTO_USED)
/*
 * Marine Communication Functions
 * Copyright (C) 2020 ACP Aviation includes information from sources in the header
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#include "Actisense_NGT1.h"
#include <stdint.h>
#include "Struts.h"
#include "gc_events.h"

/* #include <string.h>
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

#if defined(ACTISENSE_USED)
static int16_t debug = 0;
static int16_t verbose = 0;
static int16_t readonly = 0;
static int16_t writeonly = 0;
static int16_t passthru = 0;
static uint32_t timeout = 0;
static int16_t outputCommands = 0;
static bool isFile;
static unsigned char buf[500u];
static unsigned char *head = buf;
#endif /* end actisense */

/* ================== functions ============================================  */
float64_t correctWindSpeedForHeal(float64_t v, float64_t angleOfHeal);
#if defined(ACTISENSE_USED)
void messageReceived(const uint8_t *msg, size_t msgLen);
bool readNGT1Byte(unsigned char c);
void n2kMessageReceived(const uint8_t *msg, size_t msgLen);
void ngtMessageReceived(const uint8_t *msg, size_t msgLen);
void writeMessage(int16_t handle, unsigned char command, const unsigned char *cmd, const size_t len);
#endif
/* %%%%%%%%%%%%%%%%%%%%%% NMEA-2000 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   */
#if defined(NMEA2000_USED)
void n2kGetBuf(int16_t *index, const unsigned char* buf, N2K_Data_Type_e n2kType, float64_t precision, float64_t def, n2k_data_obj_t *value );
uint8_t parseMaretronFFM100(maretron_flow_t *MaretronFloObj, unsigned char *n2kRcvBuf);
uint8_t writeMaretronFFM100(maretron_flow_t *MaretronFloObj, unsigned char *n2kSndBuf);
uint8_t writeMaretronFQM100(maretron_FluidVol_t *MaretronFloObj, unsigned char *n2kSndBuf);
uint8_t parseMaretronFQM100(maretron_FluidVol_t *MaretronFloObj, unsigned char *n2kRcvBuf);
uint8_t parseMaretronTMP100(maretron_temp_t *MaretronTempObj, unsigned char *n2kRcvBuf);
uint8_t writeMaretronTMP100(maretron_temp_t *MaretronTempObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kSysTime(n2k_time_t *N2kSysTimObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kSysTime(n2k_time_t *N2kSysTimObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kDCDetail(n2k_dc_detail_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kDCDetail(n2k_dc_detail_t *N2kObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kChargerStat(n2k_charger_stat_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kChargerStat(n2k_charger_stat_t *N2kObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kBatConf(n2k_bat_conf_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kBatConf(n2k_bat_conf_t *N2kObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kLatLong(n2k_lat_lon_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kLatLong(n2k_lat_lon_t *N2kObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kGNSSPositionData(n2k_gnss_pos_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kGNSSPositionData(n2k_gnss_pos_t *N2kObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kDateTimLocalOff(n2k_dateTim_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kDistLog(n2k_distLog_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kRateOfTurn(n2k_rateOfturn_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kAttitude(n2k_attitude_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kAttitude(n2k_attitude_t *N2kObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kMagVar(n2k_magVar_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kMagVar(n2k_magVar_t *N2kObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kFluidLvl(n2k_FluidLvl_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kLeeway(n2k_Leeway_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kBoatSpeed(n2k_BoatSpeed_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kWaterDepth(n2k_WaterDepth_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kCogSog(n2k_CogSog_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kCogSog(n2k_CogSog_t *N2kObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kGNSSDOP(n2k_GNSSDOP_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kGNSSDOP(n2k_GNSSDOP_t *N2kObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kAISRepClassA129038(n2k_AISRepClassA129038_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kAISRepClassB129039(n2k_AISRepClassB129039_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kAISSARAirRep129798(n2k_AISSARAirRep129798_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kWindSpeed(n2k_WindSpeed_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kWindSpeed(n2k_WindSpeed_t *N2kObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kEngRap(n2k_engineRapid_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kEngDyn(n2k_engineDynamics_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kTransmissDyn(n2k_transmissDyn_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kTripEngDyn(n2k_EngTripDyn_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kNavInfo(n2k_NavInfo_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kRudInfo(n2k_Rudder_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kNavInfo(n2k_NavInfo_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kVessHead(n2k_vesselHeading_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kBatStat(n2k_bat_stat_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t WriteN2kBatStat(n2k_bat_stat_t *N2kObj, unsigned char *n2kSndBuf);
uint8_t ParseN2kCrossTrakError(n2k_CrossTrak_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kOutsideEnv(n2k_OutEnv_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kEnvPar(n2k_EnvPar_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kTempPar(n2k_Temperature_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kHumPar(n2k_Humidity_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kSetPressPar(n2k_SetPressure_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kExtTempPar(n2k_Temperature_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kTrimTab(n2k_TrimTab_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kStatClassBPartA(n2k_AISStaticClassBB129810_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kStatClassBPartB(n2k_AISStaticClassBA129809_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kStatClassAPartA(n2k_AISStaticData129794_t *N2kObj, unsigned char *n2kRcvBuf);
uint8_t ParseN2kWaypointList(n2k_WayPointList_t *N2kObj, unsigned char *n2kRcvBuf);
void n2kSetBuf(int16_t *index, const unsigned char* buf, N2K_Data_Type_e n2kType, float64_t precision, float64_t def, n2k_data_obj_t *value );
uint32_t N2ktoCanID(N2K_Header_t * n2kHead);
void CanIdToN2k(uint32_t id, N2K_Receiver_t * n2kHead);
bool IsISOSndMessage(uint32_t PGN);
bool IsISORcvMessage(uint32_t PGN);
bool IsSingleFrameSystemMessage(uint32_t PGN);
bool IsFastPacketSystemMessage(uint32_t PGN);
bool IsDefaultSingleFrameMessage(uint32_t PGN);
bool IsMandatoryFastPacketMessage(uint32_t PGN);
bool IsDefaultFastPacketMessage(uint32_t PGN);
uint8_t IsFastPacket( uint32_t PGN );
uint8_t SendMsgN2kOnCan(const N2K_Header_t *N2kMsg, int8_t PortNo, int16_t DeviceIndex);
uint8_t RcvN2kCANBusMsg( N2K_Receiver_t * n2kHead);
/* %%%%%%%%%%%%%%%%%%%%%% ISO11783 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   */
uint16_t getCanIdFromISO11783Bits(uint16_t prio, uint16_t pgn, uint16_t src, uint16_t dst);
void getISO11783BitsFromCanId(uint16_t id, uint16_t *prio, uint16_t *pgn, uint16_t *src, uint16_t *dst);
#endif /* NMEA2000 used */

/* ------------------- N2K and Seasmart ------------------------------------ */
#if (defined(NMEA2000_USED) && defined(SEASMART_USED))
size_t N2kToSeasmart(const N2K_RawMessage_t *msg, uint32_t timestamp, char *buffer, size_t size);
bool SeasmartToN2k(const char *buffer, uint32_t *timestamp, N2K_RawMessage_t *msg);
#endif /* NMEA Seasmart */

/* %%%%%%%%%%%%%%%%%%%%%% SEASMART %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   */
#if defined(SEASMART_USED)
int16_t appendByte(char *s, uint8_t byte1);
int16_t append2Bytes(char *s, uint16_t i);
int16_t appendWord(char *s, uint32_t i);
uint8_t nmea_compute_checksum(const char *sentence);
int8_t readNHexByte(const char *s, uint16_t n, uint32_t *value);
#endif

/* %%%%%%%%%%%%%%%%%%%%%% YDWG JSON %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   */
#if defined(YDWG_JSON_USED)
int16_t getJSONValue(const char *message, const char *fieldName, char *value, size_t len);
void jSonGSA(SerialNMEA0183Object_t *msg183, int src, const char *msg);
void jSonGLL(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg);
void jSonRudder(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg);
void jSonDistanceTraveled(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg);
void jSonWaterTemperature(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg);
void jSonWaterSpeed(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg);
void jSonWaterDepth(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg);
void jSonWindData(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg);
void jSonVesselHeading( SerialNMEA0183Object_t *msg183, int16_t src, const char *msg);
void jSonVTG(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg);
void readJSON(SerialNMEA0183Object_t *msg183, const char *msg);
#endif

/* %%%%%%%%%%%%%%%%%%%%%% NMEA 0183 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
#if defined(NMEA0183_USED)
void nmea0183CreateMessage(SerialNMEA0183Object_t *msg183, int16_t src, const char *format, ...);
#endif

#if defined(ACTISENSE_USED)
static void ngtMessageReceived(const uint8_t *msg, size_t msgLen)
{
  size_t i;
  char line[1000u];
  char *p;
  char dateStr[N2K_DATE_LENGTH];                                                    /* needs to be linked to our actual date string */

  if (msg==NULL)
     return;

  if (msgLen < 12)
  {
    return;                                                                     /* Ignore short msg len */
  }

  sprintf(line, "%s,%u,%u,%u,%u,%u", dateStr, 0u, ACTISENSE_BEM + msg[0u], 0u, 0u, (uint16_t) msgLen - 1u);
  p = line + strlen(line);
  for (i = 1; i < msgLen && p < line + sizeof(line) - 5u; i++)
  {
    sprintf(p, ",%02x", msg[i]);
    p += 3;
  }
  *p++ = 0;

  UART4_Write_Text(line);                                                       /* assumes UART4 for NMEA */
  /* puts(line);
  fflush(stdout); */
}

static void n2kMessageReceived(const uint8_t *msg, size_t msgLen)
{
  uint16_t prio, src, dst;
  uint16_t pgn;
  size_t i;
  uint16_t id;
  uint16_t len;
  /* uint16_t dataV[8u];  */
  char line[800u];
  char *p;
  char dateStr[N2K_DATE_LENGTH];                                                    /* connect this to our date string */

  if (msg==NULL)
    return;

  if (msgLen < 11u)
  {
    return;                                                                     /* Ignoring N2K message - too short */
  }
  prio = msg[0u];
  pgn  = (uint16_t) msg[1u] + 256u * ((uint16_t) msg[2u] + 256u * (uint16_t) msg[3u]);
  dst  = msg[4u];
  src  = msg[5u];
  /* Skip the timestamp logged by the NGT-1-A in bytes 6-9 */
  len = msg[10u];

  if (len > 223u)
  {
    return;                                                                     /* Ignoring N2K message - too long  */
  }
  p = line;

  memset(p,0u,sizeof(line));
  sprintf(p, "%s,%u,%u,%u,%u,%u", dateStr, prio, pgn, src, dst, len);
  p += strlen(line);

  len += 11u;
  for (i = 11u; i < len; i++)
  {
    /*snprintf(p, line + sizeof(line) - p, ",%02x", msg[i]); */
    sprintf(p,",%02x", msg[i]);
    p += strlen(p);
  }
  UART4_Write_Text(line);                                                       /* assumes UART4 for NMEA */
  /* puts(line); */
  /* fflush(stdout);  */
}

static void messageReceived(const uint8_t *msg, size_t msgLen)
{
  uint8_t command;
  uint8_t checksum = 0u;
  uint8_t *payload;
  uint8_t payloadLen;
  size_t i;

  if (msg==NULL)
    return;

  if (msgLen < 3)
  {
    return;                                                                     /* Ignore short command */
  }

  for (i = 0; i < msgLen; i++)
  {
    checksum += msg[i];
  }
  if (checksum)
  {
    return;                                                                     /* Ignoring message with invalid checksum */
  }

  command = msg[0u];                                                            /* first byte is command */
  payloadLen = msg[1u];                                                         /* next byte is length */

  if (command == N2K_MSG_RECEIVED)                                              /* if it was a N2k NMEA2000 message */
  {
    n2kMessageReceived(msg + 2u, payloadLen);
  }
  else if (command == NGT_MSG_RECEIVED)                                         /* if it was a NGT NMEA0183 message */
  {
    ngtMessageReceived(msg + 2u, payloadLen);
  }
}
/**
 * Handle a byte coming in from the NGT1.
 * This is called for each char recieved in the interrupt stream
 * when set to true to interrupt stream read state is set to true (complete)
 *
 */
static bool readNGT1Byte(unsigned char c)
{
  Acti_MSG_State_e state = MSG_START;
  static bool startEscape = false;
  static bool noEscape = false;

  if (state == MSG_START)
  {
    if ((c == ESC) && isFile)
    {
      noEscape = true;
    }
  }

  if (state == MSG_ESCAPE)
  {
    if (c == ETX)
    {
      messageReceived(buf, head - buf);
      head  = buf;
      state = MSG_START;
    }
    else if (c == STX)
    {
      head  = buf;
      state = MSG_MESSAGE;
    }
    else if ((c == DLE) || ((c == ESC) && isFile) || noEscape)
    {
      *head++ = c;
      state   = MSG_MESSAGE;
    }
    else
    {
      state = MSG_START;                                                        /* DLE followed by unexpected char %02X, ignore message */
    }
  }
  else if (state == MSG_MESSAGE)
  {
    if (c == DLE)
    {
      state = MSG_ESCAPE;
    }
    else if (isFile && (c == ESC) && !noEscape)
    {
      state = MSG_ESCAPE;
    }
    else
    {
      *head++ = c;
    }
  }
  else
  {
    if (c == DLE)
    {
      state = MSG_ESCAPE;
    }
  }

  return state == MSG_START;
}

/*
 * Wrap the PGN or NGT message and send to NGT
 */
void writeMessage(int16_t handle, unsigned char command, const unsigned char *cmd, const size_t len)
{
  unsigned char bst[255u];
  unsigned char *b = bst;
  unsigned char *r = bst;
  unsigned char crc;
  /* int16_t retryCount = 5;  */
  int16_t needs_written = b - bst;
  int16_t written;
  int16_t i;
  uint8_t byte;

  *b++ = DLE;                                                                   /* start the message */
  *b++ = STX;
  *b++ = command;
  crc  = command;
  *b++ = len;
  if (len == DLE)
  {
    *b++ = DLE;
  }

  for (i = 0; i < len; i++)
  {
    if (cmd[i] == DLE)
    {
      *b++ = DLE;
    }
    *b++ = cmd[i];
    crc += (unsigned char) cmd[i];
  }

  crc += i;

  crc = 256u - (int16_t) crc;
  if (crc == DLE)
  {
    *b++ = DLE;
  }
  *b++ = crc;
  *b++ = DLE;
  *b++ = ETX;
  needs_written = b - bst;

  for(byte=0;byte<needs_written;byte++)                                         /* for each byte in the message */
  {
    UART5_Write(bst[byte]);                                                     /* assumes UART5 for actisense */
  }

}
#endif /* actisense */

#if defined(SEASMART_USED)
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SEASMART %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
/* Some private helper functions to generate hex-serialized NMEA messages */
static const char *hex = "0123456789ABCDEF";

int16_t appendByte(char *s, uint8_t byte1)
{
  if (s == NULL) return 0;
  s[0] = hex[byte1 >> 4u];
  s[1] = hex[byte1 & 0xfu];
  return 2;
}

int16_t append2Bytes(char *s, uint16_t i)
{
  if (s == NULL) return 0;
  appendByte(s, i >> 8u);
  appendByte(s + 2, i & 0xffu);
  return 4;
}

int16_t appendWord(char *s, uint32_t i)
{
  if (s == NULL) return 0;
  append2Bytes(s, i >> 16u);
  append2Bytes(s + 4u, i & 0xfffful);
  return 8;
}

uint8_t nmea_compute_checksum(const char *sentence)
{
  int16_t i = 1;                                                                /* Skip the $ at the beginning */
  int16_t checksum = 0;

  if (sentence == 0)
  {
    return 0;
  }

  while (sentence[i] != '*')
  {
    checksum ^= sentence[i];
    i=++i %INT16_MAX;
  }
  return checksum;
}

size_t N2kToSeasmart(const N2K_RawMessage_t *msg, uint32_t timestamp, char *buffer, size_t size)
{
  size_t pcdin_sentence_length = 6+1+6+1+8+1+2+1+msg->len*2+1+2 + 1;
  int16_t i;
  char *s = NULL;

  if (((size < pcdin_sentence_length) || (buffer == NULL)) || (msg == NULL))
  {
    return 0;
  }

  *s = (char) buffer;

  strcpy(s, "$PCDIN,");
  s += 7;
  s += appendByte(s, msg->pgn >> 16);
  s += append2Bytes(s, msg->pgn & 0xffff);
  *s++ = ',';
  s += appendWord(s, timestamp);
  *s++ = ',';
  s += appendByte(s, msg->src);
  *s++ = ',';

  for (i = 0; i < msg->len; i++)
  {
    s += appendByte(s, msg->dataV[i]);
  }

  *s++ = '*';
  s += appendByte(s, nmea_compute_checksum(buffer));
  *s = 0;

  return (size_t)(s - buffer);
}

/*
 * Attemts to read n bytes in hexadecimal from input string to value.
 *
 * Returns true if successful, false otherwise.
 */
#define SEASMART_MAX_SIZE 80u
int8_t readNHexByte(const char *s, uint16_t n, uint32_t *value)
{

  uint16_t i;
  //char sNumber[2*n + 1];
  char sNumber[SEASMART_MAX_SIZE];

  if (strlen((char*)s) < (int16_t)(2*n))
  {
    return -1;
  }

  for (i = 0; i < (2*n); i++)
  {
    if (!isxdigit(s[i]))
        {
      return -1;
    }
  }

  strncpy((char*)sNumber,(char*) s,(int16_t) sizeof(sNumber));
  sNumber[sizeof(sNumber) - 1] = 0;

  *value = strtol(sNumber, 0, 16);
  return true;
}
#define SEASMART_MAX_DATA_LEN 100u
bool SeasmartToN2k(const char *buffer, uint32_t *timestamp, N2K_RawMessage_t *msg)
{

  const char *s = buffer;
  uint32_t pgnHigh=0LU;
  uint32_t pgnLow=0LU;
  uint32_t checksum=0LU;
  uint32_t source=0LU;
  int16_t dataLen = 0;
  int16_t i;
  uint32_t byte1=0LU;

  if (buffer == NULL)
  {
     return false;
  }
  if (strncmp((char*)"$PCDIN,",(char*) s,(char) 6) != 0)
  {
    return false;
  }
  s += 7;
  if (!readNHexByte(s,  1, &pgnHigh))
  {
    return false;
  }
  s += 2;
  if (!readNHexByte(s, 2, &pgnLow))
  {
    return false;
  }
  s += 5;
  msg->pgn = (pgnHigh << 16) + pgnLow;

  if (!readNHexByte(s, 4, timestamp))
  {
    return false;
  }
  s += 9;
  if (!readNHexByte(s, 1, &source))
  {
    return false;
  }
  msg->src = source;
  s += 3;


  while (s[dataLen] != 0 && s[dataLen] != '*')
  {
    dataLen++;
  }
  if (dataLen % 2 != 0)
  {
    return false;
  }
  dataLen /= 2;

  msg->len = dataLen;
  if (msg->len > SEASMART_MAX_DATA_LEN)
  {
    return false;
  }
  for (i = 0; i < dataLen; i++)
  {
    if (!readNHexByte(s, 1, &byte1))
    {
      return false;
    }
    msg->dataV[i] = byte1;
    s += 2;
  }
  s += 1;
  if (!readNHexByte(s, 1, &checksum))                                           // Skip the terminating '*' which marks beginning of checksum
  {
    return false;
  }

  if (checksum != nmea_compute_checksum(buffer))
  {
    return false;
  }

  return true;
}
#endif /* seasmart */

#if defined(NMEA2000_USED)
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NMEA 2000 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
//*****************************************************************************
void CanIdToN2k(uint32_t id, N2K_Receiver_t * n2kHead)
{
  uint8_t CanIdPF = (uint8_t) ((id >> 16u) % UINT8_MAX);
  uint8_t CanIdPS = (uint8_t) ((id >> 8u) % UINT8_MAX);
  uint8_t CanIdDP = (uint8_t) ((id >> 24u) & 1u);

    n2kHead->src = (uint8_t) id >> 0;
    n2kHead->priority = (uint8_t) ((id >> 26) & 0x7);

    if (CanIdPF < 240u)
    {
        n2kHead->dst = CanIdPS;                                                 /* PDU1 format, the PS contains the destination address */
        n2kHead->PGN = (((uint32_t)CanIdDP) << 16u) | (((uint32_t)CanIdPF) << 8u);
    }
    else
    {
        n2kHead->dst = 0xffu;                                                   /* PDU2 format, the destination is implied global and the PGN is extended */
        n2kHead->PGN = (((uint32_t)CanIdDP) << 16u) | (((uint32_t)CanIdPF) << 8u) | (uint32_t)CanIdPS;
    }
}

//*****************************************************************************
uint32_t N2ktoCanID(N2K_Header_t * n2kHead)
{
  uint8_t CanIdPF = (uint8_t) (n2kHead->PGN >> 8u);

  if (CanIdPF < 240u)                                                           // PDU1 format
  {
     if ( (n2kHead->PGN & 0xfful) != 0ul ) return 0ul;                          // for PDU1 format PGN lowest byte has to be 0 for the destination.
     return ( ((uint32_t)(n2kHead->priority & 0x7u))<<26u | n2kHead->PGN<<8ul | ((uint32_t)n2kHead->dst)<<8u | (uint32_t)n2kHead->src);
  }
  else                                                                          // PDU2 format
  {
     return ( ((uint32_t)(n2kHead->priority & 0x7u))<<26u | n2kHead->PGN<<8ul | (uint32_t)n2kHead->src);
  }
}

void n2kGetBuf(int16_t *index, const unsigned char* buf, N2K_Data_Type_e n2kType, float64_t precision, float64_t def, n2k_data_obj_t *value )
{
    uint8_t WayLen=0u;

    if (((buf==NULL) || (value==NULL)) || (index==NULL))
      return;
    if (*index > sizeof(buf))
      return;

    switch(n2kType)
    {
        case typ2ByteInt:
        memcpy((void*)&value->n2kfield.ty2ByteInt,(void*)&buf[*index], 2u);
        *index=*index + 2;
        break;

        case typ2ByteUInt:
        memcpy((void*)&value->n2kfield.ty2ByteUInt,(void*)&buf[*index], 2u);
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty2ByteUInt = SWAPINT16(value->n2kfield.ty2ByteUInt);
#endif
        *index=*index + 2;
        break;

        case typ3ByteUInt:
        memcpy((void*)&value->n2kfield.ty3ByteUInt,(void*)&buf[*index], 3u);
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty3ByteUInt = SWAPINT32(value->n2kfield.ty3ByteUInt);
#endif
        *index=*index + 3;
        break;

        case typ4ByteUInt:
        memcpy((void*)&value->n2kfield.ty4ByteUInt,(void*)&buf[*index], 4u);
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty4ByteUInt = SWAPINT32(value->n2kfield.ty4ByteUInt);
#endif
        *index=*index + 4;
        break;

        case typ8ByteUInt:
        memcpy((void*)&value->n2kfield.ty8ByteUInt,(void*)&buf[*index], 8u);
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty8ByteUInt = SWAPINT64(value->n2kfield.ty8ByteUInt);
#endif
        *index=*index + 8;
        break;

        case typ1ByteDouble:
        memcpy((void*)&value->n2kfield.ty1ByteDouble,(void*)&buf[*index], 1u);
        if (value->n2kfield.ty1ByteDouble==0x7f)
           value->n2kfield.ty1ByteDouble=def;
        else
            value->n2kfield.ty1ByteDouble= value->n2kfield.ty1ByteDouble * precision;
        *index=*index + 1;
        break;

        case typ1ByteUDouble:
        memcpy((void*)&value->n2kfield.ty1ByteUDouble,(void*)&buf[*index], 1u);
        if (value->n2kfield.ty1ByteUDouble==0xffU)
           value->n2kfield.ty1ByteUDouble=def;
        else
           value->n2kfield.ty1ByteUDouble= value->n2kfield.ty1ByteUDouble * precision;
        *index=*index + 1;
        break;

        case typ2ByteDouble:
        memcpy((void*)&value->n2kfield.ty2ByteDouble,(void*)&buf[*index], 2u);
        if (value->n2kfield.ty2ByteDouble==0x7fff)
           value->n2kfield.ty2ByteDouble=def;
        else
           value->n2kfield.ty2ByteDouble= value->n2kfield.ty2ByteDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty2ByteDouble = SWAPINT16(value->n2kfield.ty2ByteDouble);
#endif
        *index=*index + 2;
        break;

        case typ2ByteUDouble:
        memcpy((void*)&value->n2kfield.ty2ByteUDouble,(void*)&buf[*index], 2u);
        if (value->n2kfield.ty2ByteUDouble==0xffffU)
          value->n2kfield.ty2ByteUDouble=def;
        else
          value->n2kfield.ty2ByteUDouble= value->n2kfield.ty2ByteUDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
       value->n2kfield.ty2ByteUDouble = SWAPINT16(value->n2kfield.ty2ByteUDouble);
#endif
        *index=*index + 2;
        break;

        case typ8ByteDouble:
        memcpy((void*)&value->n2kfield.ty8ByteDouble,(void*)&buf[*index], 8u);
        if (value->n2kfield.ty8ByteDouble==0x7fffffffffffffffLL)
           value->n2kfield.ty8ByteDouble=def;
        else
           value->n2kfield.ty8ByteDouble= value->n2kfield.ty8ByteDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty8ByteDouble = SWAPINT64(value->n2kfield.ty8ByteDouble);
#endif
        *index=*index + 8;
        break;

        case typ3ByteDouble:
        memcpy((void*)&value->n2kfield.ty3ByteDouble,(void*)&buf[*index], 3u);
        if (value->n2kfield.ty3ByteDouble==0x007fffffL)
                  value->n2kfield.ty3ByteDouble=def;
        else
           value->n2kfield.ty3ByteDouble= value->n2kfield.ty3ByteDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty3ByteDouble = SWAPINT32(value->n2kfield.ty3ByteDouble);
#endif
        *index=*index + 3;
        break;

        case typ4ByteDouble:
        memcpy((void*)&value->n2kfield.ty4ByteDouble,(void*)&buf[*index], 4u);
        if (value->n2kfield.ty4ByteDouble==0x7fffffffL)
           value->n2kfield.ty4ByteDouble=def;
        else
            value->n2kfield.ty4ByteDouble= value->n2kfield.ty4ByteDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty4ByteDouble = SWAPINT32(value->n2kfield.ty4ByteDouble);
#endif
        *index=*index + 4;
        break;

        case typ4ByteUDouble:
        memcpy((void*)&value->n2kfield.ty4ByteUDouble,(void*)&buf[*index], 4u);
        if (value->n2kfield.ty4ByteUDouble==0xffffffffL)
           value->n2kfield.ty4ByteUDouble=def;
        else
           value->n2kfield.ty4ByteUDouble= value->n2kfield.ty4ByteUDouble * precision;
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty4ByteUDouble = SWAPINT32(value->n2kfield.ty4ByteUDouble);
#endif
        *index=*index + 4;
        break;

        case typ1ByteUint:
        memcpy((void*)&value->n2kfield.ty1ByteUInt,(void*)&buf[*index], 1u);
        *index=*index + 1;
        break;

        case typ7ByteStr:
        memcpy((void*)&value->n2kfield.ty7ByteStr,(void*)&buf[*index], 7u);
        *index=*index + 7;
        break;

        case typ20ByteStr:
        memcpy((void*)&value->n2kfield.ty20ByteStr,(void*)&buf[*index], 20u);
        *index=*index + 20;
        break;

        case typWayPoint:
        memcpy((void*)&value->n2kfield.ty1ByteUInt,(void*)&buf[*index], 1u);
        WayLen = (value->n2kfield.ty1ByteUInt-2u);                              /* length of string is minus length byte and 0x01 start byte */
        *index=*index + 2;                                                      /* skip the 0x01u start byte */
        memcpy((void*)&value->n2kfield.tyWayPoint,(void*)&buf[*index], WayLen);
        *index=*index + WayLen;
        break;

        default:
        break;
   }

}

uint8_t parseMaretronFFM100(maretron_flow_t *MaretronFloObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

  /* n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );*/
  /* MaretronFloObj->PGN= n2kvalue.n2kfield.ty4ByteUInt; */
  /* if (MaretronFloObj->PGN!=MARETRON_FFM100_FRate_PGN) return false; */
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  MaretronFloObj->proprieter= n2kValue.n2kfield.ty2ByteUInt;
  if (MaretronFloObj->proprieter!=MaretronProprietary) return 0u;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  MaretronFloObj->SID= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  MaretronFloObj->FlowRateInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  MaretronFloObj->FluidType= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ3ByteDouble, 0.1f, 0.0f, &n2kValue );
  MaretronFloObj->FluidFlowRate= n2kValue.n2kfield.ty3ByteDouble;

  return true;
}

uint8_t writeMaretronFFM100(maretron_flow_t *MaretronFloObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  MaretronFloObj->PGN = MARETRON_FFM100_FRate_PGN;
  n2kvalue.n2kfield.ty4ByteUInt = MARETRON_FFM100_FRate_PGN;  */
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  MaretronFloObj->proprieter = MaretronProprietary;
  n2kValue.n2kfield.ty2ByteUInt = MaretronFloObj->proprieter;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = MaretronFloObj->SID;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = MaretronFloObj->FlowRateInstance;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = MaretronFloObj->FluidType;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty3ByteDouble = MaretronFloObj->FluidFlowRate;
  n2kSetBuf(&Index, n2kSndBuf, typ3ByteDouble, 0.1f, 0.0f, &n2kValue );
  return true;
}

uint8_t parseMaretronFQM100(maretron_FluidVol_t *MaretronFloObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

  /* n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  MaretronFloObj->PGN= n2kvalue.n2kfield.ty4ByteUInt;
  if (MaretronFloObj->PGN!=MARETRON_FFM100_FTv_PGN) return false;  */
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  MaretronFloObj->proprieter= n2kValue.n2kfield.ty2ByteUInt;
  if (MaretronFloObj->proprieter!=MaretronProprietary) return 0u;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  MaretronFloObj->SID= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  MaretronFloObj->VolumeInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  MaretronFloObj->FluidType= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ3ByteDouble, 1.0f, 0.0f, &n2kValue );
  MaretronFloObj->TripVolume= n2kValue.n2kfield.ty3ByteDouble;
  return true;
}

uint8_t writeMaretronFQM100(maretron_FluidVol_t *MaretronFloObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

 /* MaretronFloObj->PGN = MARETRON_FFM100_FTv_PGN;
  n2kvalue.n2kfield.ty4ByteUInt = MaretronFloObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );  */
  MaretronFloObj->proprieter = MaretronProprietary;
  n2kValue.n2kfield.ty2ByteUInt = MaretronFloObj->proprieter;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = MaretronFloObj->SID;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = MaretronFloObj->VolumeInstance;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = MaretronFloObj->FluidType;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty3ByteDouble = MaretronFloObj->TripVolume;
  n2kSetBuf(&Index, n2kSndBuf, typ3ByteDouble, 1.0f, 0.0f, &n2kValue );
  return true;
}


uint8_t parseMaretronTMP100(maretron_temp_t *MaretronTempObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  MaretronTempObj->PGN= n2kvalue.n2kfield.ty4ByteUInt;
  if (MaretronTempObj->PGN!=MARETRON_TMP100_PGN) return false;  */
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  MaretronTempObj->proprieter= n2kValue.n2kfield.ty2ByteUInt;
  if (MaretronTempObj->proprieter!=MaretronProprietary) return 0u;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  MaretronTempObj->SID= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  MaretronTempObj->TempInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  MaretronTempObj->TempSource= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  MaretronTempObj->ActualTemperature= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  MaretronTempObj->SetTemperature= n2kValue.n2kfield.ty2ByteUDouble;
  return true;
}

uint8_t writeMaretronTMP100(maretron_temp_t *MaretronTempObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  MaretronTempObj->PGN = MARETRON_TMP100_PGN;
  n2kValue.n2kfield.ty4ByteUInt = MaretronTempObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );  */
  MaretronTempObj->proprieter = MaretronProprietary;
  n2kValue.n2kfield.ty2ByteUInt = MaretronTempObj->proprieter;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = MaretronTempObj->SID;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = MaretronTempObj->TempInstance;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = MaretronTempObj->TempSource;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUDouble = MaretronTempObj->ActualTemperature;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUDouble = MaretronTempObj->SetTemperature;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  return true;
}

//******************** Parse System Time ******************************
uint8_t ParseN2kSysTime(n2k_time_t *N2kSysTimObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

  /* n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );        PIC Can function already handles it in the ID
  N2kSysTimObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kSysTimObj->PGN!=N2K_SYSTIM_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kSysTimObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kSysTimObj->SID= n2kValue.n2kfield.ty1ByteUInt;    */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kSysTimObj->TimeSource= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kSysTimObj->SystemDate= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 0.0001f, 0.0f, &n2kValue );
  N2kSysTimObj->SystemTime= n2kValue.n2kfield.ty4ByteDouble;
  return true;
}
//******************** Write System Time ******************************
uint8_t WriteN2kSysTime(n2k_time_t *N2kSysTimObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kSysTimObj->PGN= N2K_SYSTIM_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kSysTimObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kSysTimObj->priority= 3u;
  n2kValue.n2kfield.ty1ByteUInt = N2kSysTimObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kSysTimObj->SID;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );  */
  n2kValue.n2kfield.ty1ByteUInt = N2kSysTimObj->TimeSource;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUInt = N2kSysTimObj->SystemDate;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty4ByteDouble = N2kSysTimObj->SystemTime;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteDouble, 0.0001f, 0.0f, &n2kValue );
  return true;
}

//******************** Parse DC Detail ******************************
uint8_t ParseN2kDCDetail(n2k_dc_detail_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_DCDETAIL_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->DCInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->DCType= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->StateOfCharge= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->StateOfHealth= n2kValue.n2kfield.ty1ByteUInt;

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 60.0f, 0.0f, &n2kValue );
  N2kObj->TimeRemaining= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.001f, 0.0f, &n2kValue );
  N2kObj->RippleVoltage= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 3600.0f, 0.0f, &n2kValue );
  N2kObj->Capacity= n2kValue.n2kfield.ty2ByteUDouble;
  return true;
}

//******************** Parse DC Detail ******************************
uint8_t WriteN2kDCDetail(n2k_dc_detail_t *N2kObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kObj->PGN = N2K_DCDETAIL_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= 6u;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->SID;      */

  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->DCInstance;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->DCType;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->StateOfCharge;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->StateOfHealth;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUDouble = N2kObj->TimeRemaining;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUDouble, 60.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUDouble = N2kObj->RippleVoltage;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUDouble, 0.001f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUDouble = N2kObj->Capacity;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUDouble, 3600.0f, 0.0f, &n2kValue );
  return true;
}

//******************** Parse Charger Status ******************************
uint8_t ParseN2kChargerStat(n2k_charger_stat_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_CHARGERSTAT_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->Instance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->ChargeState= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->Enabled= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 60.0f, 0.0f, &n2kValue );
  N2kObj->EqualizationTimeRemaining= n2kValue.n2kfield.ty2ByteUDouble;
  return true;
}
//******************** Parse Charger Status ******************************
uint8_t WriteN2kChargerStat(n2k_charger_stat_t *N2kObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kObj->PGN = N2K_CHARGERSTAT_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );

  N2kObj->priority = 6u;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->SID;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );  */

  n2kValue.n2kfield.ty1ByteUInt = N2kObj->Instance;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->ChargeState;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->Enabled;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUDouble = N2kObj->EqualizationTimeRemaining;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUDouble, 60.0f, 0.0f, &n2kValue );
  return true;
}
//******************** Battery Status ******************************
uint8_t ParseN2kBatStat(n2k_bat_stat_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_CHARGERSTAT_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;     */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->BatInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->BatteryVoltage= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->BatteryCurrent= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->BatteryTemperature= n2kValue.n2kfield.ty2ByteUDouble;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;      */
  return true;
}
//******************** Parse Charger Status ******************************
uint8_t WriteN2kBatStat(n2k_bat_stat_t *N2kObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kObj->PGN = N2K_CHARGERSTAT_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );

  N2kObj->priority = 6u;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );  */

  n2kValue.n2kfield.ty1ByteUInt = N2kObj->BatInstance;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->BatteryVoltage;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->BatteryCurrent;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.1f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUDouble = N2kObj->BatteryTemperature;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );

/*  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->SID;    */
  return true;
}

//******************** Battery Configuration Status ****************************
uint8_t ParseN2kBatConf(n2k_bat_conf_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_BATSTAT_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->BatInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->BatType= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->BatNominalVoltage= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 3600.0f, 0.0f, &n2kValue );
  N2kObj->BatCapacity= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->BatTemperatureCoefficient= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUDouble, 0.002f, 0.0f, &n2kValue );
  N2kObj->PeukertExponent= n2kValue.n2kfield.ty1ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->ChargeEfficiencyFactor= n2kValue.n2kfield.ty1ByteUInt;
  return true;
}

//******************** Battery Configuration Status ****************************
uint8_t WriteN2kBatConf(n2k_bat_conf_t *N2kObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kObj->PGN = N2K_BATSTAT_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority = 6u;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->SID;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );   */

  n2kValue.n2kfield.ty1ByteUInt = N2kObj->BatInstance;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->BatType;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->BatNominalVoltage;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUDouble = N2kObj->BatCapacity;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUDouble, 3600.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->BatTemperatureCoefficient;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUDouble = N2kObj->PeukertExponent;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUDouble, 0.002f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->ChargeEfficiencyFactor;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  return true;
}

//******************** Lattitude / Longditude ****************************
uint8_t ParseN2kLatLong(n2k_lat_lon_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_LATLONRAPID_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUDouble, 1e-7f, 0.0f, &n2kValue );
  N2kObj->Latitude= n2kValue.n2kfield.ty4ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUDouble, 1e-7f, 0.0f, &n2kValue );
  N2kObj->Longitude= n2kValue.n2kfield.ty4ByteUDouble;
  return true;
}

//******************** Lattitude / Longditude ****************************
uint8_t WriteN2kLatLong(n2k_lat_lon_t *N2kObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kObj->PGN = N2K_LATLONRAPID_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= 2u;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= 1234u;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->SID;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );  */

  n2kValue.n2kfield.ty4ByteUDouble = N2kObj->Latitude;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUDouble, 1e-7f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty4ByteUDouble = N2kObj->Longitude;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUDouble, 1e-7f, 0.0f, &n2kValue );
  return true;
}

//******************** GNSS Position Data ****************************
uint8_t ParseN2kGNSSPositionData(n2k_gnss_pos_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_GNSSPOS_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->DaysSince1970= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->SecondsSinceMidnight= n2kValue.n2kfield.ty4ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ8ByteDouble, 1e-16f, 0.0f, &n2kValue );
  N2kObj->Latitude= n2kValue.n2kfield.ty8ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ8ByteDouble, 1e-16f, 0.0f, &n2kValue );
  N2kObj->Longitude= n2kValue.n2kfield.ty8ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ8ByteDouble, 1e-6f, 0.0f, &n2kValue );
  N2kObj->Altitude= n2kValue.n2kfield.ty8ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->GNSStype= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->Integrity= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->nSatellites= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->HDOP= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->PDOP= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->GeoidalSeparation= n2kValue.n2kfield.ty4ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->nReferenceStations= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->ReferenceStationType= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->AgeOfCorrection= n2kValue.n2kfield.ty2ByteUDouble;
  return true;
}

//******************** GNSS Position Data ****************************
uint8_t WriteN2kGNSSPositionData(n2k_gnss_pos_t *N2kObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kObj->PGN= N2K_GNSSPOS_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority = 3;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->SID;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );    */

  n2kValue.n2kfield.ty2ByteUInt= N2kObj->DaysSince1970;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty4ByteUDouble = N2kObj->SecondsSinceMidnight;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUDouble, 0.0001f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty8ByteDouble = N2kObj->Latitude;
  n2kSetBuf(&Index, n2kSndBuf, typ8ByteDouble, 1e-16f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty8ByteDouble = N2kObj->Longitude;
  n2kSetBuf(&Index, n2kSndBuf, typ8ByteDouble, 1e-16f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty8ByteDouble = N2kObj->Altitude;
  n2kSetBuf(&Index, n2kSndBuf, typ8ByteDouble, 1e-6f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->GNSStype;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->Integrity;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->nSatellites;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->HDOP;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->PDOP;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty4ByteDouble = N2kObj->GeoidalSeparation;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteDouble, 0.01f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->nReferenceStations;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUInt = N2kObj->ReferenceStationType;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUDouble = N2kObj->AgeOfCorrection;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  return true;
}

//******************** Date Time and local offset ****************************
uint8_t ParseN2kDateTimLocalOff(n2k_dateTim_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_DATETIM_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->DaysSince1970= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->SecondsSinceMidnight= n2kValue.n2kfield.ty4ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->LocalOffset= n2kValue.n2kfield.ty2ByteInt;
  return true;
}

//******************** Dist Log ****************************
uint8_t ParseN2kDistLog(n2k_distLog_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_DISTLOG_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->DaysSince1970= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->SecondsSinceMidnight= n2kValue.n2kfield.ty4ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->Log= n2kValue.n2kfield.ty4ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->TripLog= n2kValue.n2kfield.ty4ByteUInt;
  return true;
}

//******************** Rate Of turn ****************************
uint8_t ParseN2kRateOfTurn(n2k_rateOfturn_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_RATEOFTURN_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 3.125E-08f, 0.0f, &n2kValue );
  N2kObj->RateOfTurn= n2kValue.n2kfield.ty4ByteDouble;
  return true;
}

//******************** Attitude ****************************
uint8_t ParseN2kAttitude(n2k_attitude_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_ATTITUDE_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->Yaw= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->Pitch= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->Roll= n2kValue.n2kfield.ty2ByteDouble;
  return true;
}

//******************** Attitude ****************************
uint8_t WriteN2kAttitude(n2k_attitude_t *N2kObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kObj->PGN = N2K_ATTITUDE_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= 3u;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->SID; */

  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->Yaw;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->Pitch;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->Roll;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  return true;
}

//******************** Magnetic variation ****************************
uint8_t ParseN2kMagVar(n2k_magVar_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_MAGVAR_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0000f, 0.0f, &n2kValue );
  N2kObj->DaysSince1970= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->Variation= n2kValue.n2kfield.ty2ByteDouble;
  return true;
}

//******************** Magnetic variation ****************************
uint8_t WriteN2kMagVar(n2k_magVar_t *N2kObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kObj->PGN= N2K_MAGVAR_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority = 6u;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->SID; */

  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUInt = N2kObj->DaysSince1970;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUInt, 0.0000f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->Variation;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  return true;
}

//******************** Fluid Level ****************************
uint8_t ParseN2kFluidLvl(n2k_FluidLvl_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_FLUIDLVL_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  not in this reply ?? */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0000f, 0.0f, &n2kValue );
  N2kObj->Instance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.004f, 0.0f, &n2kValue );
  N2kObj->Level= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->Capacity= n2kValue.n2kfield.ty4ByteDouble;
  return true;
}

//******************** Leeway ****************************
uint8_t ParseN2kLeeway(n2k_Leeway_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_LEEWAY_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->Leeway= n2kValue.n2kfield.ty2ByteDouble;
  return true;
}

//******************** Boat Speed ****************************
uint8_t ParseN2kBoatSpeed(n2k_BoatSpeed_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_BOATSPEED_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->WaterReferenced= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->GroundReferenced= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SWRT= n2kValue.n2kfield.ty1ByteUInt;
  return true;
}

//******************** Water Depth ****************************
uint8_t ParseN2kWaterDepth(n2k_WaterDepth_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_WATERDEPTH_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->DepthBelowTransducer= n2kValue.n2kfield.ty4ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.001f, 0.0f, &n2kValue );
  N2kObj->Offset= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteDouble, 10.0f, 0.0f, &n2kValue );
  N2kObj->Range= n2kValue.n2kfield.ty1ByteDouble;
  return true;
}

//******************** COG and SOG  ****************************
uint8_t ParseN2kCogSog(n2k_CogSog_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_COGSOG_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->HeadingRef= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->COG= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->SOG= n2kValue.n2kfield.ty1ByteDouble;
  return true;
}

//******************** COG and SOG  ****************************
uint8_t WriteN2kCogSog(n2k_CogSog_t *N2kObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kObj->PGN= N2K_COGSOG_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= 2u;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->SID;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );  */

  n2kValue.n2kfield.ty1ByteUInt = N2kObj->HeadingRef;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->COG;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteDouble = N2kObj->SOG;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->byte1=0xFFLU;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->byte1;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->byte2=0xFFLU;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->byte2;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  return true;
}

//******************** GNSS DOP (dilution of precision)  ***********************
uint8_t ParseN2kGNSSDOP(n2k_GNSSDOP_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_GNSSDOP_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->ActualMode= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->HDOP= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->VDOP= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->TDOP= n2kValue.n2kfield.ty2ByteDouble;
  return true;
}

//******************** GNSS DOP (dilution of precision)  ***********************
uint8_t WriteN2kGNSSDOP(n2k_GNSSDOP_t *N2kObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kObj->PGN= N2K_GNSSDOP_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= 3u;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->SID;  */
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->ActualMode;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->HDOP;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->VDOP;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteDouble = N2kObj->TDOP;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  return true;
}

//******************** AIS position report (class A 129038)  *******************
uint8_t ParseN2kAISRepClassA129038(n2k_AISRepClassA129038_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_AISCLASSA_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->MessageID= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.00f, 0.0f, &n2kValue );
  N2kObj->UserID= n2kValue.n2kfield.ty4ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 1e-07f, 0.0f, &n2kValue );
  N2kObj->Longditude= n2kValue.n2kfield.ty4ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 1e-07f, 0.0f, &n2kValue );
  N2kObj->Latitude= n2kValue.n2kfield.ty4ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->Accuracy= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 1e-04f, 0.0f, &n2kValue );
  N2kObj->COG= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->SOG= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.00f, 0.0f, &n2kValue );
  N2kObj->CommStat= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.00f, 0.0f, &n2kValue );
  N2kObj->TransInfo= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 1e-04f, 0.0f, &n2kValue );
  N2kObj->Heading= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 3.125E-05f, 0.0f, &n2kValue );
  N2kObj->ROT= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->NavStatus= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->Reserved= n2kValue.n2kfield.ty1ByteUInt;

  return true;
}

//******************** Air Craft Position  *******************
uint8_t ParseN2kAISSARAirRep129798(n2k_AISSARAirRep129798_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_AISCLASSA_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->MessageID= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ8ByteDouble, 1e-6f, 0.0f, &n2kValue );
  N2kObj->Altitude= n2kValue.n2kfield.ty8ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->SOG= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->PositionAccuracy= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 1e-07f, 0.0f, &n2kValue );
  N2kObj->Longitude= n2kValue.n2kfield.ty4ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 1e-07f, 0.0f, &n2kValue );
  N2kObj->Latitude= n2kValue.n2kfield.ty4ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 1e-04f, 0.0f, &n2kValue );
  N2kObj->COG= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.00f, 0.0f, &n2kValue );
  N2kObj->Timestamp= n2kValue.n2kfield.ty4ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.00f, 0.0f, &n2kValue );
  N2kObj->RegionalReserve= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->DTE= n2kValue.n2kfield.ty1ByteUInt;
  return true;
}

//******************** AIS position report (class B 129039)  *******************
uint8_t ParseN2kAISRepClassB129039(n2k_AISRepClassB129039_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_AISCLASSB_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;      */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->MessageID= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.00f, 0.0f, &n2kValue );
  N2kObj->UserID= n2kValue.n2kfield.ty4ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 1e-07f, 0.0f, &n2kValue );
  N2kObj->Longditude= n2kValue.n2kfield.ty4ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 1e-07f, 0.0f, &n2kValue );
  N2kObj->Latitude= n2kValue.n2kfield.ty4ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->Accuracy= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 1e-04f, 0.0f, &n2kValue );
  N2kObj->COG= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->SOG= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.00f, 0.0f, &n2kValue );
  N2kObj->CommStat= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.00f, 0.0f, &n2kValue );
  N2kObj->TransInfo= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 1e-04f, 0.0f, &n2kValue );
  N2kObj->Heading= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->RegionalApp= n2kValue.n2kfield.ty2ByteUInt;
  return true;
}

//******************** WindSpeed  ****************************
uint8_t ParseN2kWindSpeed(n2k_WindSpeed_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_COGSOG_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;   */

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->WindSpeed= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->WindAngle= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->WindReference= n2kValue.n2kfield.ty1ByteUInt;
  return true;
}

//******************** WindSpeed  ****************************
uint8_t WriteN2kWindSpeed(n2k_WindSpeed_t *N2kObj, unsigned char *n2kSndBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  N2kObj->PGN= N2K_COGSOG_PGN;
  n2kValue.n2kfield.ty4ByteUInt = N2kObj->PGN;
  n2kSetBuf(&Index, n2kSndBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority = 3u;
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->priority;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->SID;    */
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUDouble = N2kObj->WindSpeed;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty2ByteUDouble = N2kObj->WindAngle;
  n2kSetBuf(&Index, n2kSndBuf, typ2ByteUDouble, 0.0001f, 0.0f, &n2kValue );
  n2kValue.n2kfield.ty1ByteUInt = N2kObj->WindReference;
  n2kSetBuf(&Index, n2kSndBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  return true;
}

//******************** Engine Rapid  ****************************
uint8_t ParseN2kEngRap(n2k_engineRapid_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_ENGRAP_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;   */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->EngineInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.25f, 0.0f, &n2kValue );
  N2kObj->EngineSpeed= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 100.0f, 0.0f, &n2kValue );
  N2kObj->EngineBoostPressure= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->EngineTiltTrim= n2kValue.n2kfield.ty1ByteUInt;
  return true;
}

//******************** Engine Dynamics  ****************************
uint8_t ParseN2kEngDyn(n2k_engineDynamics_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_ENGDYN_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->EngineInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 100.0f, 0.0f, &n2kValue );
  N2kObj->EngineOilPress= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->EngineOilTemp= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->EngineCoolantTemp= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->AltenatorVoltage= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->FuelRate= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUDouble, 1.0f, 0.0f, &n2kValue );
  N2kObj->EngineHours= n2kValue.n2kfield.ty4ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 100.0f, 0.0f, &n2kValue );
  N2kObj->EngineCoolantPress= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 100.0f, 0.0f, &n2kValue );
  N2kObj->EngineFuelPress= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->Reserved= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->Status1= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->Status2= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->EngineLoad= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->EngineTorque= n2kValue.n2kfield.ty1ByteUInt;
  return true;
}

//******************** Transmission Dynamics ****************************
uint8_t ParseN2kTransmissDyn(n2k_transmissDyn_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_TRANSDYN_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->EngineInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->TransmissionGear= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 100.0f, 0.0f, &n2kValue );
  N2kObj->OilPressure= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->OilTemperature= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->DiscreteStatus1= n2kValue.n2kfield.ty1ByteUInt;
  return true;
}

//******************** Trip Eng Dynamics ****************************
uint8_t ParseN2kTripEngDyn(n2k_EngTripDyn_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_TRIPENG_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;    */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->EngineInstance= n2kValue.n2kfield.ty1ByteUInt;

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 1.0f, 0.0f, &n2kValue );
  N2kObj->TripFuelUsed= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->FuelRateAverage= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->FuelRateEconomy= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->InstantaneousFuelEconomy= n2kValue.n2kfield.ty2ByteUDouble;
  return true;
}

//******************** Navigation Information ****************************
uint8_t ParseN2kNavInfo(n2k_NavInfo_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_NAV_INFO_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->DistanceToWaypoint= n2kValue.n2kfield.ty4ByteUDouble;

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->BearingReference= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->ETATime= n2kValue.n2kfield.ty4ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->ETADate= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->BearingOriginToDestinationWaypoint= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->BearingPositionToDestinationWaypoint= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.000f, 0.0f, &n2kValue );
  N2kObj->OriginWaypointNumber= n2kValue.n2kfield.ty4ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.000f, 0.0f, &n2kValue );
  N2kObj->DestinationWaypointNumber= n2kValue.n2kfield.ty4ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 1e-07f, 0.0f, &n2kValue );
  N2kObj->DestinationLatitude= n2kValue.n2kfield.ty4ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 1e-07f, 0.0f, &n2kValue );
  N2kObj->DestinationLongitude= n2kValue.n2kfield.ty4ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->WaypointClosingVelocity= n2kValue.n2kfield.ty2ByteDouble;
  return true;
}

//******************** Rudder Information ****************************
uint8_t ParseN2kRudInfo(n2k_Rudder_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_RUDDER_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->Instance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->RudderDirectionOrder= n2kValue.n2kfield.ty1ByteUInt;

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->AngleOrder= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->RudderPosition= n2kValue.n2kfield.ty2ByteDouble;
  return true;
}

//******************** Vessel Heading ****************************
uint8_t ParseN2kVessHead(n2k_vesselHeading_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_VESSEL_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;    */

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->Heading= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->Deviation= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->Variation= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->ref= n2kValue.n2kfield.ty1ByteUInt;
  return true;
}

//******************** Cross Track Error ****************************
uint8_t ParseN2kCrossTrakError(n2k_CrossTrak_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_CROSSTRAK_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 1.0f, 0.0f, &n2kValue );
  N2kObj->XTEMode= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->XTE= n2kValue.n2kfield.ty4ByteDouble;
  return true;
}

//******************** Outside Environment Parameters **************************
uint8_t ParseN2kOutsideEnv(n2k_OutEnv_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_OUTENV_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;    */

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->WaterTemperature= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->OutsideAmbientAirTemperature= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 100.0f, 0.0f, &n2kValue );
  N2kObj->AtmosphericPressure= n2kValue.n2kfield.ty2ByteUDouble;
  return true;
}

//******************** Environment Parameters **************************
uint8_t ParseN2kEnvPar(n2k_EnvPar_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_ENV_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;     */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->TempSource= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->Temperature= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.004f, 0.0f, &n2kValue );
  N2kObj->Humidity= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 100.0f, 0.0f, &n2kValue );
  N2kObj->AtmosphericPressure= n2kValue.n2kfield.ty2ByteUDouble;
  return true;
}

//******************** Temperature Parameters **************************
uint8_t ParseN2kTempPar(n2k_Temperature_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_TEMP_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->TempInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->TempSource= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->ActualTemperature= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->SetTemperature= n2kValue.n2kfield.ty2ByteUDouble;
  return true;
}

//******************** Extended Temperature Parameters *************************
uint8_t ParseN2kExtTempPar(n2k_Temperature_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_TEMPEXT_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->TempInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->TempSource= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ3ByteDouble, 0.001f, 0.0f, &n2kValue );
  N2kObj->ActualTemperature= n2kValue.n2kfield.ty3ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->SetTemperature= n2kValue.n2kfield.ty2ByteDouble;
  return true;
}

//******************** Humidity Parameters **************************
uint8_t ParseN2kHumPar(n2k_Humidity_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_HUMID_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;   */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->HumidityInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->HumiditySource= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.004f, 0.0f, &n2kValue );
  N2kObj->ActualHumidity= n2kValue.n2kfield.ty2ByteDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteDouble, 0.004f, 0.0f, &n2kValue );
  N2kObj->SetHumidity= n2kValue.n2kfield.ty2ByteDouble;
  return true;
}

//******************** Pressure Parameters **************************
uint8_t ParseN2kPressPar(n2k_Pressure_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_PRESS_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;     */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->PressureInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->PressureSource= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->ActualPressure= n2kValue.n2kfield.ty4ByteUDouble;
  return true;
}

//******************** Set Pressure Parameters **************************
uint8_t ParseN2kSetPressPar(n2k_SetPressure_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_SETPRESS_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;  */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->PressureInstance= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->PressureSource= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->SetPressure= n2kValue.n2kfield.ty4ByteUDouble;
  return true;
}

//******************** Trim Tab **************************
uint8_t ParseN2kTrimTab(n2k_TrimTab_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_SETPRESS_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->SID= n2kValue.n2kfield.ty1ByteUInt;     */

  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->PortTrimTab= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->StbdTrimTab= n2kValue.n2kfield.ty1ByteUInt;

  return true;
}

//******************** AIS Static Data Class A message Part A **************************
uint8_t ParseN2kStatClassAPartA(n2k_AISStaticData129794_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_AISCLASSBA_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->MessageID= n2kValue.n2kfield.ty1ByteUInt;  */
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->Repeat= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->UserID= n2kValue.n2kfield.ty4ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->IMOnumber= n2kValue.n2kfield.ty4ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ7ByteStr, 0.0f, 0.0f, &n2kValue );
  memcpy((void*)&N2kObj->Callsign,(void*)&n2kValue.n2kfield.ty7ByteStr,7u);
  n2kGetBuf(&Index, n2kRcvBuf, typ20ByteStr, 0.0f, 0.0f, &n2kValue );
  memcpy((void*)&N2kObj->Name,(void*)&n2kValue.n2kfield.ty20ByteStr,20u);
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->VesselType= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->Length= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->Beam= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->PosRefStbd= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->PosRefBow= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->ETAdate= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUDouble, 0.0001f, 0.0f, &n2kValue );
  N2kObj->PosRefBow= n2kValue.n2kfield.ty4ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.01f, 0.0f, &n2kValue );
  N2kObj->Draught= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ20ByteStr, 0.0f, 0.0f, &n2kValue );
  memcpy((void*)&N2kObj->Destination,(void*)&n2kValue.n2kfield.ty20ByteStr,20u);
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->AISversion= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->AISTransieverInfo= n2kValue.n2kfield.ty1ByteUInt;
  return true;
}

//******************** Static Class B Part A **************************
uint8_t ParseN2kStatClassBPartA(n2k_AISStaticClassBA129809_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_AISCLASSBA_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->MessageID= n2kValue.n2kfield.ty1ByteUInt;     */

  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->UserID= n2kValue.n2kfield.ty4ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ20ByteStr, 0.0f, 0.0f, &n2kValue );
  memcpy((void*)&N2kObj->Name,(void*)&n2kValue.n2kfield.ty20ByteStr,20u);
  return true;
}

//******************** Static Class B Part B **************************
uint8_t ParseN2kStatClassBPartB(n2k_AISStaticClassBB129810_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_AISCLASSBB_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->MessageID= n2kValue.n2kfield.ty1ByteUInt; */

  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->UserID= n2kValue.n2kfield.ty4ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->VesselType= n2kValue.n2kfield.ty1ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ7ByteStr, 0.0f, 0.0f, &n2kValue );
  memcpy((void*)&N2kObj->VesselType,(void*)&n2kValue.n2kfield.ty7ByteStr,7u);
  n2kGetBuf(&Index, n2kRcvBuf, typ7ByteStr, 0.0f, 0.0f, &n2kValue );
  memcpy((void*)&N2kObj->Callsign,(void*)&n2kValue.n2kfield.ty7ByteStr,7u);
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->Length= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->Beam= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->PosRefStbd= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUDouble, 0.1f, 0.0f, &n2kValue );
  N2kObj->PosRefBow= n2kValue.n2kfield.ty2ByteUDouble;
  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->MothershipID= n2kValue.n2kfield.ty4ByteUInt;
  return true;
}
//******************** Waypoint List **************************
uint8_t ParseN2kWaypointList(n2k_WayPointList_t *N2kObj, unsigned char *n2kRcvBuf)
{
  n2k_data_obj_t n2kValue;
  int16_t Index=0;
  uint8_t itemNo=0u;

/*  n2kGetBuf(&Index, n2kRcvBuf, typ4ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->PGN= n2kValue.n2kfield.ty4ByteUInt;
  if (N2kObj->PGN!=N2K_WAYPOINT_PGN) return false;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->priority= n2kValue.n2kfield.ty1ByteUInt;   */
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->Start= n2kValue.n2kfield.ty2ByteUInt;

  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->NumOfItems= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->Database= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
  N2kObj->Route= n2kValue.n2kfield.ty2ByteUInt;
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->NavDirection= n2kValue.n2kfield.ty1ByteUInt;

  n2kGetBuf(&Index, n2kRcvBuf, typWayPoint, 0.0f, 0.0f, &n2kValue );
  if (strlen(n2kValue.n2kfield.tyWayPoint)>=1u)
  {
    memcpy((void*)&N2kObj->routeName,(void*)&n2kValue.n2kfield.tyWayPoint,strlen(n2kValue.n2kfield.tyWayPoint));
  }
  else
  {
    memset((void*)&N2kObj->routeName,0u,1u);
  }
  n2kGetBuf(&Index, n2kRcvBuf, typ1ByteUint, 0.0f, 0.0f, &n2kValue );
  N2kObj->endName= n2kValue.n2kfield.ty1ByteUInt;
  for (itemNo=0;itemNo<N2kObj->NumOfItems;itemNo++)                             /* for each item in the waypoint list */
  {
     n2kGetBuf(&Index, n2kRcvBuf, typ2ByteUInt, 0.0f, 0.0f, &n2kValue );
     N2kObj->waypoint[itemNo]->ID= n2kValue.n2kfield.ty2ByteUInt;
     n2kGetBuf(&Index, n2kRcvBuf, typWayPoint, 0.0f, 0.0f, &n2kValue );
     if (strlen(n2kValue.n2kfield.tyWayPoint)>=1u)
     {
        memcpy((void*)&N2kObj->waypoint[itemNo]->routeName,(void*)&n2kValue.n2kfield.tyWayPoint,strlen(n2kValue.n2kfield.tyWayPoint));
     }
     else
     {
        memset((void*)&N2kObj->waypoint[itemNo]->routeName,0u,1u);
     }
     n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 1e-07f, 0.0f, &n2kValue );
     N2kObj->waypoint[itemNo]->Latitude= n2kValue.n2kfield.ty4ByteDouble;
     n2kGetBuf(&Index, n2kRcvBuf, typ4ByteDouble, 1e-07f, 0.0f, &n2kValue );
     N2kObj->waypoint[itemNo]->Longitude= n2kValue.n2kfield.ty4ByteDouble;
  }
  return true;
}

/* ----------------------------------------------------------------------------
 *  n2kSetBuf : Set the buffer with the value at the precision given
 *  param : int16_t index, const unsigned char* buf, N2K_Data_Type_e n2kType,
 *         float64_t precision, float64_t def, n2k_data_obj_t *value
 *  return : void
 * -------------------------------------------------------------------------- */
void n2kSetBuf(int16_t *index, const unsigned char* buf, N2K_Data_Type_e n2kType, float64_t precision, float64_t def, n2k_data_obj_t *value )
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

    switch(n2kType)
    {
        case typ2ByteInt:
        memcpy((void*)&buf[*index], (void*)&value->n2kfield.ty2ByteInt, 2u);
        *index=*index + 2;
        break;

        case typ2ByteUInt:
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty2ByteUInt = SWAPINT16(value->n2kfield.ty2ByteUInt);
#endif
        memcpy((void*)&buf[*index], (void*)&value->n2kfield.ty2ByteUInt, 2u);
        *index=*index + 2;
        break;

        case typ3ByteUInt:
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty3ByteUInt = SWAPINT32(value->n2kfield.ty3ByteUInt);
#endif
        memcpy((void*)&buf[*index], (void*)&value->n2kfield.ty3ByteUInt, 3u);
        *index=*index + 3;
        break;

        case typ4ByteUInt:
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty4ByteUInt = SWAPINT32(value->n2kfield.ty4ByteUInt);
#endif
        memcpy((void*)&buf[*index], (void*)&value->n2kfield.ty4ByteUInt, 4u);
        *index=*index + 4;
        break;

        case typ8ByteUInt:
#if defined(HOST_IS_BIG_ENDIAN)
        value->n2kfield.ty8ByteUInt = SWAPINT64(value->n2kfield.ty8ByteUInt);
#endif
        memcpy((void*)&buf[*index], (void*)&value->n2kfield.ty8ByteUInt, 8u);
        *index=*index + 8;
        break;

        case typ1ByteDouble:
        fp = value->n2kfield.ty1ByteDouble / precision;
        v8 = ((uint8_t)ROUND(fp,0)); //% UINT8_MAX;
        memcpy((void*)&buf[*index], (void*) &v8, 1u);
        *index=*index + 1;
        break;

        case typ1ByteUDouble:
        fp = value->n2kfield.ty1ByteUDouble / precision;
        v8 = ((uint8_t)ROUND(fp,0)); //% UINT8_MAX;
        memcpy((void*)&buf[*index], (void*) &v8, 1u);
        *index=*index + 1;
        break;

        case typ2ByteDouble:
        fp = value->n2kfield.ty2ByteDouble / precision;
        v9 = ((uint16_t)ROUND(fp,0)); //% UINT16_MAX;
#if defined(HOST_IS_BIG_ENDIAN)
        v9 = SWAPINT16(v9);
#endif
        memcpy((void*)&buf[*index], (void*) &v9, 2u);
        *index=*index + 2;
        break;

        case typ2ByteUDouble:
        fp = value->n2kfield.ty2ByteUDouble / precision;
        v9 = ((uint16_t)ROUND(fp,0)); //% UINT16_MAX;;
#if defined(HOST_IS_BIG_ENDIAN)
        v9 = SWAPINT16(v9);
#endif
        memcpy((void*)&buf[*index], (void*) &v9, 2u);
        *index=*index + 2;
        break;

        case typ8ByteDouble:
        fp=precision*1e6f;
        fpll=(int64_t)(1.0f/fp);
        vll=value->n2kfield.ty8ByteDouble*1e6f;
        vll*=fpll % UINT64_MAX;
#if defined(HOST_IS_BIG_ENDIAN)
        vll = SWAPINT64(vll);
#endif
        memcpy((void*)&buf[*index], (void*) &vll, 8u);
        *index=*index + 8;
        break;

        case typ3ByteDouble:
        fp = value->n2kfield.ty3ByteDouble / precision;
        v10 = ((uint32_t)ROUND(fp,0)); // % UINT32_MAX;
#if defined(HOST_IS_BIG_ENDIAN)
        v10 = SWAPINT32(v10);
#endif
        memcpy((void*)&buf[*index], (void*) &v10, 3u);
        *index=*index + 3;
        break;

        case typ4ByteDouble:
        fp = value->n2kfield.ty4ByteDouble / precision;
        v10 = ((uint32_t)ROUND(fp,0)); //% UINT32_MAX;
#if defined(HOST_IS_BIG_ENDIAN)
        v10 = SWAPINT32(v10);
#endif
        memcpy((void*)&buf[*index], (void*) &v10, 4u);
        *index=*index + 4;
        break;

        case typ4ByteUDouble:
        fp = value->n2kfield.ty4ByteUDouble / precision;
        v10 = ((uint32_t)ROUND(fp,0)); //% UINT32_MAX;
#if defined(HOST_IS_BIG_ENDIAN)
        v10 = SWAPINT32(v10);
#endif
        memcpy((void*)&buf[*index], (void*) &v10, 4u);
        *index=*index + 4;
        break;

        case typ1ByteUint:
        memcpy((void*)&buf[*index], (void*)&value->n2kfield.ty1ByteUInt, 1u);
        *index=*index + 1;
        break;

        case typ7ByteStr:
        memcpy((void*)&buf[*index], (void*)&value->n2kfield.ty7ByteStr, 7u);
        *index=*index + 7;
        break;

        case typ20ByteStr:
        memcpy((void*)&buf[*index], (void*)&value->n2kfield.ty7ByteStr, 20u);
        *index=*index + 20;
        break;

        case typWayPoint:
        wayLen = (strlen(value->n2kfield.tyWayPoint)+2u);                       /* length is string length plus two */
        if (wayLen == 2u)                                                       /* if string is a NULL */
        {
           wayLen = 0x03u;                                                      /* len is length field plus 0x01 plus 0x0 = 3 */
        }
        memcpy((void*)&buf[*index],(void*)wayLen,1u);                           /* add the length field */
        *index=*index + 1;
        wayLen = 0x01u;
        memcpy((void*)&buf[*index],(void*)wayLen,1u);                           /* add the 0x01 start byte */
        *index=*index + 1;
        if (wayLen == 0x03u)
        {
           memset((void*)&buf[*index],0u,1u);                                   /* send a NULL byte */
           *index=*index + 1;
        }
        else
        {
           memcpy((void*)&buf[*index],(void*)&value->n2kfield.tyWayPoint, strlen(value->n2kfield.tyWayPoint));  /* add the waypoint string */
           *index=*index + strlen(value->n2kfield.tyWayPoint);                  /* advance the pointer by the string length */
        }
        break;

        default:
        break;
   }

}

/* ----------------------------------------------------------------------------
 *  IsISOSndMessage : checks pgn validity
 *  param : uint32_t PGN
 *         
 *  return : bool
 * -------------------------------------------------------------------------- */
bool IsISOSndMessage(uint32_t PGN)
{
                                  uint8_t retCode=false;
                                  switch (PGN)
                                  {
                                      case  59392LU:                            /* ISO Acknowledgement, pri=6, period=NA */
                                      retCode=true;
                                      break;

                                      case  59904LU:                            /* ISO Request, pri=6, period=NA */
                                      retCode=true;
                                      break;

                                      case  60928LU:                            /* ISO Address Claim, pri=6, period=NA */
                                      retCode=true;
                                      break;

                                      case  126208LU:                           /* NMEA Request/Command/Acknowledge group function, pri=3, period=NA */
                                      retCode=true;
                                      break;

                                      case  126464LU:                           /* PGN List (Transmit and Receive), pri=6, period=NA */
                                      retCode=true;
                                      break;

                                      case  126993L:                            /* Heartbeat, pri=7, period=60000 */
                                      retCode=true;
                                      break;

                                      case 126996L:                             /* Product information, pri=6, period=NA */
                                      retCode=true;
                                      break;

                                      case 126998L:                             /* Configuration information, pri=6, period=NA */
                                      retCode=true;
                                      break;

                                      default:
                                      retCode=false;
                                      break;
                                  }
                                  return retCode;
}
/* ----------------------------------------------------------------------------
 *  IsISORcvMessage : checks pgn validity on recieve end
 *  param : uint32_t PGN
 *         
 *  return : bool
 * -------------------------------------------------------------------------- */
bool IsISORcvMessage(uint32_t PGN)
{
                                  uint8_t retCode=false;
                                  switch (PGN)
                                  {
                                      case 59392LU:                             /* ISO Acknowledgement, pri=6, period=NA */
                                      retCode=true;
                                      break;

                                      case 59904LU:                             /* ISO Request, pri=6, period=NA */
                                      retCode=true;
                                      break;

                                      case TP_DT:                               /* Multi packet data transfer, TP.DT */
                                      retCode=true;
                                      break;

                                      case TP_CM:                               /* Multi packet connection management, TP.CM */
                                      retCode=true;
                                      break;

                                      case 60928LU:                             /* ISO Address Claim */
                                      retCode=true;
                                      break;

                                      case 65240LU:                             /* Commanded Address */
                                      retCode=true;
                                      break;

                                      case 126208LU:                            /* NMEA Request/Command/Acknowledge group function */
                                      retCode=true;
                                      break;

                                      case 126998LU:                            /* Configuration information, pri=6, period=NA */
                                      retCode=true;
                                      break;

                                      default:
                                      retCode=false;
                                      break;
                                  }
                                  return retCode;
}
/* ----------------------------------------------------------------------------
 *  IsSingleFrameSystemMessage : checks pgn validity on SingleFrameSystemMessage
 *  param : uint32_t PGN
 *         
 *  return : bool
 * -------------------------------------------------------------------------- */
bool IsSingleFrameSystemMessage(uint32_t PGN)
{
                                  uint8_t retCode=false;
                                  switch (PGN)
                                  {
                                      case 59392LU:                              /* ISO Acknowledgement */
                                      retCode=true;
                                      break;

                                      case TP_DT:                               /* Multi packet data transfer, TP.DT */
                                      retCode=true;
                                      break;

                                      case TP_CM:                               /* Multi packet connection management, TP.CM */
                                      retCode=true;
                                      break;

                                      case 59904LU:                             /* ISO Request */
                                      retCode=true;
                                      break;

                                      case 60928LU:                             /* ISO Address Claim */
                                      retCode=true;
                                      break;

                                      default:
                                      retCode=false;
                                      break;
                                  }
                                  return retCode;
}
/* ----------------------------------------------------------------------------
 *  IsFastPacketSystemMessage : checks pgn validity on FastPacketSystemMessage
 *  param : uint32_t PGN
 *         
 *  return : bool
 * -------------------------------------------------------------------------- */
bool IsFastPacketSystemMessage(uint32_t PGN)
{
                                  uint8_t retCode=false;
                                  switch (PGN)
                                  {
                                      case  65240LU:                            /* Commanded Address*/
                                      retCode=true;
                                      break;

                                      case 126208LU:                            /* NMEA Request/Command/Acknowledge group function */
                                      retCode=true;
                                      break;

                                      default:
                                      retCode=false;
                                      break;
                                  }
                                  return retCode;
}
/* ----------------------------------------------------------------------------
 *  IsDefaultSingleFrameMessage : checks pgn validity on DefaultSingleFrameMessage
 *  param : uint32_t PGN
 *         
 *  return : bool
 * -------------------------------------------------------------------------- */
bool IsDefaultSingleFrameMessage(uint32_t PGN)
{
                                  uint8_t retCode=false;
                                  switch (PGN)
                                  {
                                      case N2K_SYSTIM_PGN:                      // System date/time, pri=3, period=1000
                                      retCode=true;
                                      break;

                                      case 126993LU:                            // Heartbeat, pri=7, period=60000
                                      retCode=true;
                                      break;

                                      case 127245LU:                            // Rudder, pri=2, period=100
                                      retCode=true;
                                      break;

                                      case 127250LU:                            // Vessel Heading, pri=2, period=100
                                      retCode=true;
                                      break;

                                      case 127251LU:                            // Rate of Turn, pri=2, period=100
                                      retCode=true;
                                      break;

                                      case 127257LU:                            // Attitude, pri=3, period=1000
                                      retCode=true;
                                      break;

                                      case 127488LU:                            // Engine parameters rapid, rapid Update, pri=2, period=100
                                      retCode=true;
                                      break;

                                      case 127493LU:                            // Transmission parameters: dynamic, pri=2, period=100
                                      retCode=true;
                                      break;

                                      case 127501LU:                            // Binary status report, pri=3, period=NA
                                      retCode=true;
                                      break;

                                      case 127505LU:                            // Fluid level, pri=6, period=2500
                                      retCode=true;
                                      break;

                                      case 127508LU:                            // Battery Status, pri=6, period=1500
                                      retCode=true;
                                      break;

                                      case 128259LU:                            // Boat speed, pri=2, period=1000
                                      retCode=true;
                                      break;

                                      case 128267LU:                            // Water depth, pri=3, period=1000
                                      retCode=true;
                                      break;

                                      case 129025LU:                            // Lat/lon rapid, pri=2, period=100
                                      retCode=true;
                                      break;

                                      case 129026LU:                            // COG SOG rapid, pri=2, period=250
                                      retCode=true;
                                      break;

                                      case 129283LU:                            // Cross Track Error, pri=3, period=1000
                                      retCode=true;
                                      break;

                                      case 130306LU:                            // Wind Speed, pri=2, period=100
                                      retCode=true;
                                      break;

                                      case 130310LU:                            // Outside Environmental parameters, pri=5, period=500
                                      retCode=true;
                                      break;

                                      case 130311LU:                            // Environmental parameters, pri=5, period=500
                                      retCode=true;
                                      break;

                                      case 130312LU:                            // Temperature, pri=2, period=2000
                                      retCode=true;
                                      break;

                                      case 130314LU:                            // Pressure, pri=2, period=2000
                                      retCode=true;
                                      break;

                                      case 130316LU:                            // Temperature extended range, pri=2, period=NA
                                      retCode=true;
                                      break;

                                      case 130576LU:                            // Small Craft Status (Trim Tab position), pri=2, period=NA
                                      retCode=true;
                                      break;

                                      default:
                                      retCode=false;
                                      break;
                                  }
                                  return retCode;
}
/* ----------------------------------------------------------------------------
 *  IsMandatoryFastPacketMessage : checks pgn validity on MandatoryFastPacketMessage
 *  param : uint32_t PGN
 *         
 *  return : bool
 * -------------------------------------------------------------------------- */
bool IsMandatoryFastPacketMessage(uint32_t PGN)
{
                                  uint8_t retCode=false;
                                  switch (PGN)
                                  {
                                      case 126464LU:                            // PGN List (Transmit and Receive), pri=6, period=NA
                                      retCode=true;
                                      break;

                                      case 126996LU:                            // Product information, pri=6, period=NA
                                      retCode=true;
                                      break;

                                      case 126998LU:                            // Configuration information, pri=6, period=NA
                                      retCode=true;
                                      break;

                                      default:
                                      retCode=false;
                                      break;
                                  }
                                  return retCode;
}

bool IsDefaultFastPacketMessage(uint32_t PGN)
{
                                  uint8_t retCode=false;
                                  switch (PGN)
                                  {
                                      case 127237LU:                            // Heading/Track control, pri=2, period=250
                                      retCode=true;
                                      break;

                                      case 127489LU:                            // Engine parameters dynamic, pri=2, period=500
                                      retCode=true;
                                      break;

                                      case 127496LU:                            // Trip fuel consumption, vessel, pri=5, period=1000
                                      retCode=true;
                                      break;

                                      case 127497LU:                            // Trip fuel consumption, engine, pri=5, period=1000
                                      retCode=true;
                                      break;

                                      case 127498LU:                            // Engine parameters static, pri=5, period=NA
                                      retCode=true;
                                      break;

                                      case 127503LU:                            // AC Input Status, pri=6, period=1500
                                      retCode=true;
                                      break;

                                      case 127504LU:                            // AC Output Status, pri=6, period=1500
                                      retCode=true;
                                      break;

                                      case 127506LU:                            // DC Detailed status, pri=6, period=1500
                                      retCode=true;
                                      break;

                                      case 127507LU:                            // Charger status, pri=6, period=1500
                                      retCode=true;
                                      break;

                                      case 127509LU:                            // Inverter status, pri=6, period=1500
                                      retCode=true;
                                      break;

                                      case 127510LU:                            // Charger configuration status, pri=6, period=NA
                                      retCode=true;
                                      break;

                                      case 127511LU:                            // Inverter Configuration Status, pri=6, period=NA
                                      retCode=true;
                                      break;

                                      case 127512LU:                            // AGS configuration status, pri=6, period=NA
                                      retCode=true;
                                      break;

                                      case 127513LU:                            // Battery configuration status, pri=6, period=NA
                                      retCode=true;
                                      break;

                                      case 127514LU:                            // AGS Status, pri=6, period=1500
                                      retCode=true;
                                      break;

                                      case 128275LU:                            // Distance log, pri=6, period=1000
                                      retCode=true;
                                      break;

                                      case 129029LU:                            // GNSS Position Data, pri=3, period=1000
                                      retCode=true;
                                      break;

                                      case 129038LU:                            // AIS Class A Position Report, pri=4, period=NA
                                      retCode=true;
                                      break;

                                      case 129039LU:                            // AIS Class B Position Report, pri=4, period=NA
                                      retCode=true;
                                      break;

                                      case 129284LU:                            // Navigation info, pri=3, period=1000
                                      retCode=true;
                                      break;

                                      case 129285LU:                            // Waypoint list, pri=3, period=NA
                                      retCode=true;
                                      break;

                                      case 129540LU:                            // GNSS Sats in View, pri=6, period=1000
                                      retCode=true;
                                      break;

                                      case 129794LU:                            // AIS Class A Static data, pri=6, period=NA
                                      retCode=true;
                                      break;

                                      case 129802LU:                            // AIS Safety Related Broadcast Message, pri=5, period=NA
                                      retCode=true;
                                      break;

                                      case 129809LU:                            // AIS Class B Static Data: Part A, pri=6, period=NA
                                      retCode=true;
                                      break;

                                      case 129810LU:                            // AIS Class B Static Data Part B, pri=6, period=NA
                                      retCode=true;
                                      break;

                                      case 130074LU:                            // Waypoint list, pri=7, period=NA
                                      retCode=true;
                                      break;

                                      default:
                                      retCode=false;
                                      break;
                                  }
                                  return retCode;
}
//*****************************************************************************
// Identifies a NMEA fast Packet
//
uint8_t IsFastPacket( uint32_t PGN )
{
  uint8_t ItIsFastPacket=false;

  ItIsFastPacket= IsDefaultFastPacketMessage(PGN);
  ItIsFastPacket= IsMandatoryFastPacketMessage(PGN);
  ItIsFastPacket= IsFastPacketSystemMessage(PGN);
  return ItIsFastPacket;
}

//*****************************************************************************
// Sends message to N2k bus
//
#define N2K_MAX_CAN_ADDR 251u
#define N2kPGNIsoAddressClaim 12345
uint8_t SendMsgN2kOnCan(const N2K_Header_t *N2kMsg, int8_t PortNo, int16_t DeviceIndex)
{

  uint8_t result=false;                                                         /* default returns not done */
  uint32_t canId=0ul;
  unsigned char temp[8u] = {0u,0u,0u,0u,0u,0u,0u,0u};

  int16_t cur=0;
  int16_t frames=(N2kMsg->DataLen>6u ? (N2kMsg->DataLen-6-1)/7+1+1 : 1 );
  int16_t Order=0;                                                              // to be checked out ..... GetSequenceCounter(N2kMsg.PGN,DeviceIndex)<<5;
  int16_t i;
  int16_t j;

  //  if ( DeviceIndex>=DeviceCount) return result;
  //  N2kMsg.CheckDestination();
  //  if (DeviceIndex>=0) { N2kMsg.ForceSource(Devices[DeviceIndex].N2kSource); } else { DeviceIndex=0; }

  if (( N2kMsg->src > N2K_MAX_CAN_ADDR && N2kMsg->PGN!=N2kPGNIsoAddressClaim ) || (N2kMsg->PGN==0ul)) return false; // CAN bus address range is 0-251. Anyway allow ISO address claim mgs.

  canId=N2ktoCanID(N2kMsg);

  if ( canId==0 )                                                               // PGN validity - N2ktoCanID returns 0 for invalid PGN
  {
    return result;
  }

  // if (N2kMode==N2km_ListenOnly) return false;  Do not send anything on listen only mode
  // Enable once you support this one .... if ( IsAddressClaimStarted(DeviceIndex) && N2kMsg.PGN!=N2kPGNIsoAddressClaim ) return false;

  if ((N2kMsg->DataLen<=8u) && !IsFastPacket(N2kMsg->PGN))                      // We can send single frame
  {
    // result=SendFrame(canId, N2kMsg.DataLen, N2kMsg.Data,false);
    switch (PortNo)
    {
       case 1:
       result=CAN1Write(canId, (unsigned char*) N2kMsg->DataBuf, N2kMsg->DataLen, (uint16_t) ((N2kMsg->priority & _CAN_TX_STD_FRAME) & _CAN_TX_NO_RTR_FRAME));
       break;

       case 2:
       result=CAN2Write(canId, (unsigned char*) N2kMsg->DataBuf, N2kMsg->DataLen, (uint16_t) ((N2kMsg->priority & _CAN_TX_STD_FRAME) & _CAN_TX_NO_RTR_FRAME));
       break;

       default:
       break;
    }
  }
  else                                                                          // Send it as fast packet in multiple frames
  {
    for (i = 0; i<frames && result; i++)
    {
       temp[0u] = i | Order;                                                    //frame counter
       if (i==0)
       {
          temp[1u] = N2kMsg->DataLen;                                           //total bytes in fast packet
          for (j = 2; j<8; j++)                                                 //send the first 6 bytes
          {
             temp[j]=N2kMsg->DataBuf[cur];
             cur=++cur % INT16_MAX;
          }
        }
        else
        {
            j=1;                                                                // start a new packet
            for (; j<8 && cur < N2kMsg->DataLen; j++)                           //send the next 7 data bytes
            {
                temp[j]=N2kMsg->DataBuf[cur];
                cur=++cur % INT16_MAX;
            }
            for (; j<8; j++)                                                    // stuff the end if we have finished up to 8 bytes
            {
                temp[j]=0xffu;
            }
        }
        //      result=SendFrame(canId, 8, temp, true);
        switch (PortNo)
        {
            case 1:
            result=CAN1Write(canId, (unsigned char*) N2kMsg->DataBuf, 8u, (uint16_t) ((N2kMsg->priority & _CAN_TX_XTD_FRAME) & _CAN_TX_NO_RTR_FRAME));
            break;

            case 2:
            result=CAN2Write(canId, (unsigned char*) N2kMsg->DataBuf, 8u, (uint16_t) ((N2kMsg->priority & _CAN_TX_XTD_FRAME) & _CAN_TX_NO_RTR_FRAME));
            break;

           default:
           break;
        }
     }
  }
  return result;
}

uint8_t RcvN2kCANBusMsg( N2K_Receiver_t * n2kHead, int8_t PortNo )
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
      CanIdToN2k(canId, n2kHead);
      switch (n2kHead->PGN)                                                     /* look at the PGN Number received */
      {
         case MARETRON_FFM100_FRate_PGN:                                        /* ----------- MARETRON FLOW RATE FRAME TO RECEIVE --------------------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            memset((void*)&n2kHead->FFM100FRate.DataBuf,(void*) 0u, data_len);  /* copy the 8 bytes (data_len bytes received) in the frame */
            memcpy((void*)&n2kHead->FFM100FRate.DataBuf,(void*) dataValues, data_len);  /* copy the 8 bytes (data_len bytes received) in the frame */
            parseMaretronFFM100(&n2kHead->FFM100FRate.MaretronFloObj, (unsigned char *) &n2kHead->FFM100FRate.DataBuf); /* parse buffer and set variables in the display structure */
         }
         else                                                                   /* it is extended frame */
         {
            if ( n2kHead->FFM100FRate->offset == 0u )                           /* havent written any charactures to the buffer its the first in the sequence */
            {
              memset((void*)&n2kHead->FFM100FRate.DataBuf,(void*) 0u, sizeof(n2kHead->FFM100FRate.DataBuf));  /* clean the buffer for restart */
              n2kHead->FFM100FRate->timePeriod = -1;                            /* START a timer */
              calculateTime2Now( &n2kHead->FFM100FRate.timePeriod, &n2kHead->FFM100FRate.timeRef ); /* initialise reference time to now */
            }
            if (sizeof(n2kHead->FFM100FRate.DataBuf) >= (n2kHead->FFM100FRate.offset + data_len)) /* check message size fits our container structure */
            {
               memcpy((void*)(&n2kHead->FFM100FRate.DataBuf[n2kHead->FFM100FRate.offset]),(void*) dataValues, data_len);  /* copy the received bytes in the frame */
            }
            else if (sizeof(n2kHead->FFM100FRate.DataBuf) >= data_len)         /* so long as it fits collect a new frame with the data */
            {
               memcpy((void*)(&n2kHead->FFM100FRate.DataBuf[0u]),(void*) dataValues, data_len);
               n2kHead->FFM100FRate.offset = 0u;                               /* we started again as we counted past the end of the buffer */
               n2kHead->FFM100FRate.timePeriod = -1;                           /* START a timer */
               calculateTime2Now( &n2kHead->FFM100FRate.timePeriod, &n2kHead->FFM100FRate.timeRef ); /* initialise reference time to now */
            }
            else { /* just for sanity */ }
            if (n2kHead->FFM100FRate->offset == 0u)                             /* new collection */
            {
               if (n2kHead->FFM100FRate.DataBuf[0u] == sizeof(n2kHead->FFM100FRate.MaretronFloObj)) /* required length is valid then its the START of a good message */
               {
                  n2kHead->FFM100FRate.State = N2K_COLLECTING;                 /* set the state to collecting so you can time out if it doesnt get the full frame in time limit */
               }
               else
               {
                  data_len = 0u;                                                /* reset the message if its invalid */
               }
            }
            n2kHead->FFM100FRate.offset = n2kHead->FFM100FRate.offset + data_len;  /* increment offset register by the number of bytes just received */
            if (n2kHead->FFM100FRate.offset >= n2kHead->FFM100FRate.DataBuf[0u])   /* chars received are equal to total message required data len  */
            {
               parseMaretronFFM100(&n2kHead->FFM100FRate.MaretronFloObj, (unsigned char *) &n2kHead->FFM100FRate.DataBuf); /* parse buffer and set variables in the display structure */
               n2kHead->FFM100FRate.offset = 0u;                                /* reset the chars received on this message counter */
               n2kHead->FFM100FRate.State = N2K_COLLECTED;                      /* set frame collection state to completed */
            }
         }
         break;

         case MARETRON_FFM100_FTv_PGN:                                          /* ---------- Maretron fluid volume report --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            parseMaretronFQM100(&n2kHead->FFM100FVol, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case MARETRON_TMP100_PGN:                                              /* ---------- Maretron temperature sensor --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            parseMaretronTMP100(&n2kHead->TMP100Tmp.MaretronTempObj, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else
         {
            if ( n2kHead->TMP100Tmp.offset == 0u )                             /* havent written any charactures to the buffer its the first in the sequence */
            {
              memset((void*)&n2kHead->TMP100Tmp.DataBuf,(void*) 0u, sizeof(n2kHead->TMP100Tmp.DataBuf));  /* clean the buffer for restart */
              n2kHead->TMP100Tmp.timePeriod = -1;                              /* START a timer */
              calculateTime2Now( &n2kHead->TMP100Tmp.timePeriod, &n2kHead->TMP100Tmp.timeRef ); /* initialise reference time to now */
            }
            if (sizeof(n2kHead->TMP100Tmp.DataBuf) >= (n2kHead->TMP100Tmp.offset + data_len)) /* check message size fits our container structure */
            {
               memcpy((void*)(&n2kHead->TMP100Tmp.DataBuf[n2kHead->TMP100Tmp.offset]),(void*) dataValues, data_len);  /* copy the received bytes in the frame */
            }
            else if (sizeof(n2kHead->TMP100Tmp.DataBuf) >= data_len)           /* so long as it fits collect a new frame with the data */
            {
               memcpy((void*)(&n2kHead->TMP100Tmp.DataBuf[0u]),(void*) dataValues, data_len);
               n2kHead->TMP100Tmp.offset = 0u;                                 /* we started again as we counted past the end of the buffer */
               n2kHead->TMP100Tmp.timePeriod = -1;                             /* START a timer */
               calculateTime2Now( &n2kHead->TMP100Tmp.timePeriod, &n2kHead->TMP100Tmp.timeRef ); /* initialise reference time to now */
            }
            else { /* just for sanity */ }
            if (n2kHead->TMP100Tmp.offset == 0u)                               /* new collection */
            {
               if (n2kHead->TMP100Tmp.DataBuf[0u] == sizeof(n2kHead->TMP100Tmp.MaretronTempObj)) /* required length is valid then its the START of a good message */
               {
                  n2kHead->TMP100Tmp.State = N2K_COLLECTING;                   /* set the state to collecting so you can time out if it doesnt get the full frame in time limit */
               }
               else
               {
                  data_len = 0u;                                                /* reset the message if its invalid */
               }
            }
            n2kHead->TMP100Tmp.offset = n2kHead->TMP100Tmp.offset + data_len; /* increment offset register by the number of bytes just received */
            if (n2kHead->TMP100Tmp.offset >= n2kHead->TMP100Tmp.DataBuf[0u])  /* chars received are equal to total message required data len  */
            {
               parseMaretronTMP100(&n2kHead->TMP100Tmp.MaretronTempObj, (unsigned char *) &n2kHead->TMP100Tmp.DataBuf); /* parse buffer and set variables in the display structure */
               n2kHead->TMP100Tmp.offset = 0u;                                 /* reset the chars received on this message counter */
               n2kHead->TMP100Tmp.State = N2K_COLLECTED;                       /* set frame collection state to completed */
            }
         }
         break;

         case N2K_SYSTIM_PGN:                                                   /* ---------- System Time Update --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kSysTime(&n2kHead->TimeObj, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_VESSEL_PGN:                                                   /* ---------- Vessel Heading Message --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kVessHead(&n2kHead->vessHead, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_RATEOFTURN_PGN:                                               /* ---------- Rate of Turn --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kRateOfTurn(&n2kHead->rateOfTurn, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_ATTITUDE_PGN:                                                 /* ---------- Attitude --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kAttitude(&n2kHead->attitude, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_TRANSDYN_PGN:                                                 /* ---------- Transmission Dynamics --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kTransmissDyn(&n2kHead->transDynam, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_ENGRAP_PGN:                                                   /* ---------- Engine Rapid --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kEngRap(&n2kHead->engRapid, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_MAGVAR_PGN:                                                   /* ---------- Magnetic Variation --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kMagVar(&n2kHead->magnetVar, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_TRIPENG_PGN:                                                   /* ---------- Engine Trip --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kTripEngDyn(&n2kHead->engTripDyn, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_FLUIDLVL_PGN:                                                   /* ---------- Fluid Level --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kFluidLvl(&n2kHead->fluidLvl, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_DCDETAIL_PGN:                                                 /* ---------- DC Detail --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kDCDetail(&n2kHead->dcDetail, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_CHARGERSTAT_PGN:                                              /* ---------- Charger Status --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kChargerStat(&n2kHead->chargerStat, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_LATLONRAPID_PGN:                                              /* ---------- Latitude Longditude Rapid --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kLatLong(&n2kHead->latLonPos, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_BATSTAT_PGN:                                                  /* ---------- Battery Status --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kBatStat(&n2kHead->batStatus, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_LEEWAY_PGN:                                                   /* ---------- Leeway Angle --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kLeeway(&n2kHead->leeWay, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_BOATSPEED_PGN:                                                /* ---------- Boat Speed --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kBoatSpeed(&n2kHead->boatSpeed, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_COGSOG_PGN:                                                   /* ---------- Course over Ground Speed over Ground --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kCogSog(&n2kHead->cogSog, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_GNSSPOS_PGN:                                                  /* ---------- GNSS Position Information --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kGNSSPositionData(&n2kHead->gnssPos.GNSSPosObj, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else
         {
            if ( n2kHead->gnssPos->offset == 0u )                             /* havent written any charactures to the buffer its the first in the sequence */
            {
              memset((void*)&n2kHead->gnssPos.DataBuf,(void*) 0u, sizeof(n2kHead->gnssPos.DataBuf));  /* clean the buffer for restart */
              n2kHead->gnssPos.timePeriod = -1;                              /* START a timer */
              calculateTime2Now( &n2kHead->gnssPos.timePeriod, &n2kHead->gnssPos.timeRef ); /* initialise reference time to now */
            }
            if (sizeof(n2kHead->gnssPos.DataBuf) >= (n2kHead->gnssPos.offset + data_len)) /* check message size fits our container structure */
            {
               memcpy((void*)(&n2kHead->gnssPos.DataBuf[n2kHead->gnssPos.offset]),(void*) dataValues, data_len);  /* copy the received bytes in the frame */
            }
            else if (sizeof(n2kHead->gnssPos.DataBuf) >= data_len)           /* so long as it fits collect a new frame with the data */
            {
               memcpy((void*)(&n2kHead->gnssPos.DataBuf[0u]),(void*) dataValues, data_len);
               n2kHead->gnssPos.offset = 0u;                                 /* we started again as we counted past the end of the buffer */
               n2kHead->gnssPos.timePeriod = -1;                             /* START a timer */
               calculateTime2Now( &n2kHead->gnssPos.timePeriod, &n2kHead->gnssPos.timeRef ); /* initialise reference time to now */
            }
            else { /* just for sanity */ }
            if (n2kHead->gnssPos.offset == 0u)                               /* new collection */
            {
               if (n2kHead->gnssPos.DataBuf[0u] == sizeof(n2kHead->gnssPos.GNSSPosObj)) /* required length is valid then its the START of a good message */
               {
                  n2kHead->gnssPos.State = N2K_COLLECTING;                   /* set the state to collecting so you can time out if it doesnt get the full frame in time limit */
               }
               else
               {
                  data_len = 0u;                                                /* reset the message if its invalid */
               }
            }
            n2kHead->gnssPos.offset = n2kHead->gnssPos.offset + data_len; /* increment offset register by the number of bytes just received */
            if (n2kHead->gnssPos.offset >= n2kHead->gnssPos.DataBuf[0u])  /* chars received are equal to total message required data len  */
            {
               ParseN2kGNSSPositionData(&n2kHead->gnssPos.GNSSPosObj, (unsigned char *) &n2kHead->gnssPos.DataBuf); /* parse buffer and set variables in the display structure */
               n2kHead->gnssPos.offset = 0u;                                 /* reset the chars received on this message counter */
               n2kHead->gnssPos.State = N2K_COLLECTED;                       /* set frame collection state to completed */
            }
         }
         break;

         case N2K_DATETIM_PGN:                                                  /* ---------- Date and Time --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kDateTimLocalOff(&n2kHead->dateDate, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_GNSSDOP_PGN:                                                  /* ---------- GNSS DOP --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kGNSSDOP(&n2kHead->gnssDop, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_AISCLASSA_PGN:                                                /* ---------- AIS Position Class A --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kAISRepClassA129038(&n2kHead->aisClassA.posReport, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else
         {
            if ( n2kHead->aisClassA.offset == 0u )                             /* havent written any charactures to the buffer its the first in the sequence */
            {
              memset((void*)&n2kHead->aisClassA.DataBuf,(void*) 0u, sizeof(n2kHead->aisClassA.DataBuf));  /* clean the buffer for restart */
              n2kHead->aisClassA->timePeriod = -1;                              /* START a timer */
              calculateTime2Now( &n2kHead->aisClassA.timePeriod, &n2kHead->aisClassA.timeRef ); /* initialise reference time to now */
            }
            if (sizeof(n2kHead->aisClassA.DataBuf) >= (n2kHead->aisClassA.offset + data_len)) /* check message size fits our container structure */
            {
               memcpy((void*)(&n2kHead->aisClassA.DataBuf[n2kHead->aisClassA->offset]),(void*) dataValues, data_len);  /* copy the received bytes in the frame */
            }
            else if (sizeof(n2kHead->aisClassA.DataBuf) >= data_len)           /* so long as it fits collect a new frame with the data */
            {
               memcpy((void*)(&n2kHead->aisClassA.DataBuf[0u]),(void*) dataValues, data_len);
               n2kHead->aisClassA.offset = 0u;                                 /* we started again as we counted past the end of the buffer */
               n2kHead->aisClassA.timePeriod = -1;                             /* START a timer */
               calculateTime2Now( &n2kHead->aisClassA.timePeriod, &n2kHead->aisClassA.timeRef ); /* initialise reference time to now */
            }
            else { /* just for sanity */ }
            if (n2kHead->aisClassA.offset == 0u)                               /* new collection */
            {
               if (n2kHead->aisClassA.DataBuf[0u] == sizeof(n2kHead->aisClassA.posReport)) /* required length is valid then its the START of a good message */
               {
                  n2kHead->aisClassA.State = N2K_COLLECTING;                   /* set the state to collecting so you can time out if it doesnt get the full frame in time limit */
               }
               else
               {
                  data_len = 0u;                                                /* reset the message if its invalid */
               }
            }
            n2kHead->aisClassA.offset = n2kHead->aisClassA.offset + data_len; /* increment offset register by the number of bytes just received */
            if (n2kHead->aisClassA.offset >= n2kHead->aisClassA.DataBuf[0u])  /* chars received are equal to total message required data len  */
            {
               ParseN2kAISRepClassA129038(&n2kHead->aisClassA.posReport, (unsigned char *) &n2kHead->aisClassA.DataBuf); /* parse buffer and set variables in the display structure */
               n2kHead->aisClassA.offset = 0u;                                 /* reset the chars received on this message counter */
               n2kHead->aisClassA.State = N2K_COLLECTED;                       /* set frame collection state to completed */
            }
         }
         break;

         case N2K_SAR_AIRPOS_PGN:                                               /* ---------- Air Craft Position Class A --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kAISSARAirRep129798(&n2kHead->airCraft.posReport, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else
         {
            if ( n2kHead->airCraft.offset == 0u )                             /* havent written any charactures to the buffer its the first in the sequence */
            {
              memset((void*)&n2kHead->airCraft.DataBuf,(void*) 0u, sizeof(n2kHead->airCraft.DataBuf));  /* clean the buffer for restart */
              n2kHead->airCraft.timePeriod = -1;                              /* START a timer */
              calculateTime2Now( &n2kHead->airCraft.timePeriod, &n2kHead->airCraft.timeRef ); /* initialise reference time to now */
            }
            if (sizeof(n2kHead->airCraft.DataBuf) >= (n2kHead->airCraft.offset + data_len)) /* check message size fits our container structure */
            {
               memcpy((void*)(&n2kHead->airCraft.DataBuf[n2kHead->airCraft->offset]),(void*) dataValues, data_len);  /* copy the received bytes in the frame */
            }
            else if (sizeof(n2kHead->airCraft.DataBuf) >= data_len)           /* so long as it fits collect a new frame with the data */
            {
               memcpy((void*)(&n2kHead->airCraft.DataBuf[0u]),(void*) dataValues, data_len);
               n2kHead->airCraft.offset = 0u;                                 /* we started again as we counted past the end of the buffer */
               n2kHead->airCraft.timePeriod = -1;                             /* START a timer */
               calculateTime2Now( &n2kHead->airCraft.timePeriod, &n2kHead->airCraft.timeRef ); /* initialise reference time to now */
            }
            else { /* just for sanity */ }
            if (n2kHead->airCraft->offset == 0u)                               /* new collection */
            {
               if (n2kHead->airCraft.DataBuf[0u] == sizeof(n2kHead->airCraft.posReport)) /* required length is valid then its the START of a good message */
               {
                  n2kHead->airCraft.State = N2K_COLLECTING;                   /* set the state to collecting so you can time out if it doesnt get the full frame in time limit */
               }
               else
               {
                  data_len = 0u;                                                /* reset the message if its invalid */
               }
            }
            n2kHead->airCraft.offset = n2kHead->airCraft.offset + data_len; /* increment offset register by the number of bytes just received */
            if (n2kHead->airCraft.offset >= n2kHead->airCraft.DataBuf[0u])  /* chars received are equal to total message required data len  */
            {
               ParseN2kAISSARAirRep129798(&n2kHead->airCraft.posReport, (unsigned char *) &n2kHead->airCraft.DataBuf); /* parse buffer and set variables in the display structure */
               n2kHead->airCraft.offset = 0u;                                 /* reset the chars received on this message counter */
               n2kHead->airCraft.State = N2K_COLLECTED;                       /* set frame collection state to completed */
            }
         }
         break;

         case N2K_BATCONF_PGN:                                                  /* ---------- Battery Configuration --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kBatConf(&n2kHead->battery.config, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else
         {
            if ( n2kHead->battery.offset == 0u )                               /* havent written any charactures to the buffer its the first in the sequence */
            {
              memset((void*)&n2kHead->battery.DataBuf,(void*) 0u, sizeof(n2kHead->battery.DataBuf));  /* clean the buffer for restart */
              n2kHead->battery.timePeriod = -1;                                /* START a timer */
              calculateTime2Now( &n2kHead->battery.timePeriod, &n2kHead->battery.timeRef ); /* initialise reference time to now */
            }
            if (sizeof(n2kHead->battery.DataBuf) >= (n2kHead->battery.offset + data_len)) /* check message size fits our container structure */
            {
               memcpy((void*)(&n2kHead->battery->DataBuf[n2kHead->battery.offset]),(void*) dataValues, data_len);  /* copy the received bytes in the frame */
            }
            else if (sizeof(n2kHead->battery.DataBuf) >= data_len)             /* so long as it fits collect a new frame with the data */
            {
               memcpy((void*)(&n2kHead->battery.DataBuf[0u]),(void*) dataValues, data_len);
               n2kHead->battery.offset = 0u;                                   /* we started again as we counted past the end of the buffer */
               n2kHead->battery.timePeriod = -1;                               /* START a timer */
               calculateTime2Now( &n2kHead->battery.timePeriod, &n2kHead->battery.timeRef ); /* initialise reference time to now */
            }
            else { /* just for sanity */ }
            if (n2kHead->battery.offset == 0u)                                 /* new collection */
            {
               if (n2kHead->battery.DataBuf[0u] == sizeof(n2kHead->battery.config)) /* required length is valid then its the START of a good message */
               {
                  n2kHead->battery.State = N2K_COLLECTING;                     /* set the state to collecting so you can time out if it doesnt get the full frame in time limit */
               }
               else
               {
                  data_len = 0u;                                                /* reset the message if its invalid */
               }
            }
            n2kHead->battery.offset = n2kHead->battery.offset + data_len;     /* increment offset register by the number of bytes just received */
            if (n2kHead->battery.offset >= n2kHead->battery.DataBuf[0u])      /* chars received are equal to total message required data len  */
            {
               ParseN2kBatConf(&n2kHead->battery.config, (unsigned char *) &n2kHead->battery.DataBuf); /* parse buffer and set variables in the display structure */
               n2kHead->battery.offset = 0u;                                   /* reset the chars received on this message counter */
               n2kHead->battery.State = N2K_COLLECTED;                         /* set frame collection state to completed */
            }
         }
         break;

         case N2K_WINDSPEED_PGN:                                                /* ---------- WindSpeed --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kWindSpeed(&n2kHead->windSpeed, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_CROSSTRAK_PGN:                                                /* ---------- Cross track --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kCrossTrakError(&n2kHead->crossTrkErr, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_OUTENV_PGN:                                                   /* ---------- Outside Environment --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kOutsideEnv(&n2kHead->outsideEnv, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_ENV_PGN:                                                      /* ---------- Environmental Parameters --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kEnvPar(&n2kHead->environ, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_TEMP_PGN:                                                      /* ---------- Temperature Parameters --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kTempPar(&n2kHead->temperat, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_HUMID_PGN:                                                    /* ---------- Humidity --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kHumPar(&n2kHead->humid, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_PRESS_PGN:                                                    /* ---------- Pressure --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kPressPar(&n2kHead->press, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_SETPRESS_PGN:                                                 /* ---------- Set Pressure --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kSetPressPar(&n2kHead->setPress, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_TRIMTAB_PGN:                                                  /* ---------- Trim Tab Position --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kTrimTab(&n2kHead->smallCraftStat, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_RUDDER_PGN:                                                   /* ---------- Rudder Angle --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kRudInfo(&n2kHead->rudder, (unsigned char *) dataValues);     /* parse buffer and set variables in the display structure */
         }
         else { /* multi frame not supported for this PGN */ }
         break;

         case N2K_NAV_INFO_PGN:                                                  /* ---------- Navigation Information --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kNavInfo(&n2kHead->navInfo.navigData, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else
         {
            if ( n2kHead->navInfo.offset == 0u )                               /* havent written any charactures to the buffer its the first in the sequence */
            {
              memset((void*)&n2kHead->navInfo.DataBuf,(void*) 0u, sizeof(n2kHead->navInfo.DataBuf));  /* clean the buffer for restart */
              n2kHead->navInfo.timePeriod = -1;                                /* START a timer */
              calculateTime2Now( &n2kHead->navInfo.timePeriod, &n2kHead->navInfo.timeRef ); /* initialise reference time to now */
            }
            if (sizeof(n2kHead->navInfo.DataBuf) >= (n2kHead->navInfo.offset + data_len)) /* check message size fits our container structure */
            {
               memcpy((void*)(&n2kHead->navInfo.DataBuf[n2kHead->navInfo.offset]),(void*) dataValues, data_len);  /* copy the received bytes in the frame */
            }
            else if (sizeof(n2kHead->navInfo.DataBuf) >= data_len)             /* so long as it fits collect a new frame with the data */
            {
               memcpy((void*)(&n2kHead->navInfo.DataBuf[0u]),(void*) dataValues, data_len);
               n2kHead->navInfo.offset = 0u;                                   /* we started again as we counted past the end of the buffer */
               n2kHead->navInfo.timePeriod = -1;                               /* START a timer */
               calculateTime2Now( &n2kHead->navInfo.timePeriod, &n2kHead->navInfo.timeRef ); /* initialise reference time to now */
            }
            else { /* just for sanity */ }
            if (n2kHead->navInfo.offset == 0u)                                 /* new collection */
            {
               if (n2kHead->navInfo.DataBuf[0u] == sizeof(n2kHead->navInfo.navigData)) /* required length is valid then its the START of a good message */
               {
                  n2kHead->navInfo.State = N2K_COLLECTING;                     /* set the state to collecting so you can time out if it doesnt get the full frame in time limit */
               }
               else
               {
                  data_len = 0u;                                                /* reset the message if its invalid */
               }
            }
            n2kHead->navInfo.offset = n2kHead->navInfo.offset + data_len;     /* increment offset register by the number of bytes just received */
            if (n2kHead->navInfo.offset >= n2kHead->navInfo.DataBuf[0u])      /* chars received are equal to total message required data len  */
            {
               ParseN2kNavInfo(&n2kHead->navInfo.navigData, (unsigned char *) &n2kHead->navInfo.DataBuf); /* parse buffer and set variables in the display structure */
               n2kHead->navInfo.offset = 0u;                                   /* reset the chars received on this message counter */
               n2kHead->navInfo.State = N2K_COLLECTED;                         /* set frame collection state to completed */
            }
         }
         break;

         case N2K_ENGDYN_PGN:                                                   /* ---------- Engine Dynamics --------- */
         if (!(rx_flags & _CAN_RX_XTD_FRAME))                                   /* not extended frame */
         {
            ParseN2kEngDyn(&n2kHead->engDynamics.engDynObj, (unsigned char *) dataValues); /* parse buffer and set variables in the display structure */
         }
         else
         {
            if ( n2kHead->engDynamics.offset == 0u )                               /* havent written any charactures to the buffer its the first in the sequence */
            {
              memset((void*)&n2kHead->engDynamics.DataBuf,(void*) 0u, sizeof(n2kHead->engDynamics.DataBuf));  /* clean the buffer for restart */
              n2kHead->engDynamics.timePeriod = -1;                                /* START a timer */
              calculateTime2Now( &n2kHead->engDynamics.timePeriod, &n2kHead->engDynamics.timeRef ); /* initialise reference time to now */
            }
            if (sizeof(n2kHead->engDynamics.DataBuf) >= (n2kHead->engDynamics.offset + data_len)) /* check message size fits our container structure */
            {
               memcpy((void*)(&n2kHead->engDynamics.DataBuf[n2kHead->engDynamics->offset]),(void*) dataValues, data_len);  /* copy the received bytes in the frame */
            }
            else if (sizeof(n2kHead->engDynamics.DataBuf) >= data_len)         /* so long as it fits collect a new frame with the data */
            {
               memcpy((void*)(&n2kHead->engDynamics.DataBuf[0u]),(void*) dataValues, data_len);
               n2kHead->engDynamics.offset = 0u;                               /* we started again as we counted past the end of the buffer */
               n2kHead->engDynamics.timePeriod = -1;                           /* START a timer */
               calculateTime2Now( &n2kHead->engDynamics.timePeriod, &n2kHead->engDynamics.timeRef ); /* initialise reference time to now */
            }
            else { /* just for sanity */ }
            if (n2kHead->engDynamics.offset == 0u)                             /* new collection */
            {
               if (n2kHead->engDynamics.DataBuf[0u] == sizeof(n2kHead->engDynamics.engDynObj)) /* required length is valid then its the START of a good message */
               {
                  n2kHead->engDynamics.State = N2K_COLLECTING;                 /* set the state to collecting so you can time out if it doesnt get the full frame in time limit */
               }
               else
               {
                  data_len = 0u;                                                /* reset the message if its invalid */
               }
            }
            n2kHead->engDynamics.offset = n2kHead->engDynamics.offset + data_len;     /* increment offset register by the number of bytes just received */
            if (n2kHead->engDynamics.offset >= n2kHead->engDynamics.DataBuf[0u])      /* chars received are equal to total message required data len  */
            {
               ParseN2kEngDyn(&n2kHead->engDynamics.engDynObj, (unsigned char *) &n2kHead->engDynamics.DataBuf); /* parse buffer and set variables in the display structure */
               n2kHead->engDynamics.offset = 0u;                               /* reset the chars received on this message counter */
               n2kHead->engDynamics.State = N2K_COLLECTED;                     /* set frame collection state to completed */
            }
         }
         break;
      }
   }

  /* reset any chunk message collections if they time-out before the full frame was received */
  if (n2kHead->FFM100FRate.State == N2K_COLLECTING)                            /* collecting and not got the full frame yet */
  {
      calculateTime2Now( &n2kHead->FFM100FRate.timePeriod, &n2kHead->FFM100FRate.timeRef ); /* check the timer value and if it expires reset the offset for new collection */
      if ( n2kHead->FFM100FRate.timePeriod > N2K_MAX_RCV_TIME )                /* timed out waiting for full frame to be recieved */
      {
          n2kHead->FFM100FRate.offset = 0u;                                    /* reset the chars received on this message counter */
          n2kHead->FFM100FRate.State = N2K_INCOMPLETE_FRAME;                   /* set state of this collection to incomplete frame */
      }
   }
  if (n2kHead->TMP100Tmp.State == N2K_COLLECTING)                              /* collecting and not got the full frame yet */
  {
      calculateTime2Now( &n2kHead->TMP100Tmp.timePeriod, &n2kHead->TMP100Tmp.timeRef ); /* check the timer value and if it expires reset the offset for new collection */
      if ( n2kHead->TMP100Tmp.timePeriod > N2K_MAX_RCV_TIME )                  /* timed out waiting for full frame to be recieved */
      {
          n2kHead->TMP100Tmp.offset = 0u;                                      /* reset the chars received on this message counter */
          n2kHead->TMP100Tmp.State = N2K_INCOMPLETE_FRAME;                     /* set state of this collection to incomplete frame */
      }
   }
  if (n2kHead->gnssPos.State == N2K_COLLECTING)                                /* collecting and not got the full frame yet */
  {
      calculateTime2Now( &n2kHead->gnssPos.timePeriod, &n2kHead->gnssPos.timeRef ); /* check the timer value and if it expires reset the offset for new collection */
      if ( n2kHead->gnssPos.timePeriod > N2K_MAX_RCV_TIME )                    /* timed out waiting for full frame to be recieved */
      {
          n2kHead->gnssPos.offset = 0u;                                        /* reset the chars received on this message counter */
          n2kHead->gnssPos.State = N2K_INCOMPLETE_FRAME;                       /* set state of this collection to incomplete frame */
      }
   }
  if (n2kHead->aisClassA.State == N2K_COLLECTING)                              /* collecting and not got the full frame yet */
  {
      calculateTime2Now( &n2kHead->aisClassA.timePeriod, &n2kHead->aisClassA.timeRef ); /* check the timer value and if it expires reset the offset for new collection */
      if ( n2kHead->aisClassA.timePeriod > N2K_MAX_RCV_TIME )                  /* timed out waiting for full frame to be recieved */
      {
          n2kHead->aisClassA.offset = 0u;                                      /* reset the chars received on this message counter */
          n2kHead->aisClassA.State = N2K_INCOMPLETE_FRAME;                     /* set state of this collection to incomplete frame */
      }
   }
  if (n2kHead->airCraft.State == N2K_COLLECTING)                               /* collecting and not got the full frame yet */
  {
      calculateTime2Now( &n2kHead->airCraft.timePeriod, &n2kHead->airCraft.timeRef ); /* check the timer value and if it expires reset the offset for new collection */
      if ( n2kHead->airCraft.timePeriod > N2K_MAX_RCV_TIME )                   /* timed out waiting for full frame to be recieved */
      {
          n2kHead->airCraft.offset = 0u;                                       /* reset the chars received on this message counter */
          n2kHead->airCraft.State = N2K_INCOMPLETE_FRAME;                      /* set state of this collection to incomplete frame */
      }
   }
  if (n2kHead->battery.State == N2K_COLLECTING)                                /* collecting and not got the full frame yet */
  {
      calculateTime2Now( &n2kHead->battery.timePeriod, &n2kHead->battery.timeRef ); /* check the timer value and if it expires reset the offset for new collection */
      if ( n2kHead->battery.timePeriod > N2K_MAX_RCV_TIME )                    /* timed out waiting for full frame to be recieved */
      {
          n2kHead->battery.offset = 0u;                                        /* reset the chars received on this message counter */
          n2kHead->battery.State = N2K_INCOMPLETE_FRAME;                       /* set state of this collection to incomplete frame */
      }
   }
  if (n2kHead->navInfo.State == N2K_COLLECTING)                                /* collecting and not got the full frame yet */
  {
      calculateTime2Now( &n2kHead->navInfo.timePeriod, &n2kHead->navInfo.timeRef ); /* check the timer value and if it expires reset the offset for new collection */
      if ( n2kHead->navInfo.timePeriod > N2K_MAX_RCV_TIME )                    /* timed out waiting for full frame to be recieved */
      {
          n2kHead->navInfo.offset = 0u;                                        /* reset the chars received on this message counter */
          n2kHead->navInfo.State = N2K_INCOMPLETE_FRAME;                       /* set state of this collection to incomplete frame */
      }
   }
  if (n2kHead->engDynamics.State == N2K_COLLECTING)                            /* collecting and not got the full frame yet */
  {
      calculateTime2Now( &n2kHead->engDynamics.timePeriod, &n2kHead->engDynamics.timeRef ); /* check the timer value and if it expires reset the offset for new collection */
      if ( n2kHead->engDynamics.timePeriod > N2K_MAX_RCV_TIME )                /* timed out waiting for full frame to be recieved */
      {
          n2kHead->engDynamics.offset = 0u;                                    /* reset the chars received on this message counter */
          n2kHead->engDynamics.State = N2K_INCOMPLETE_FRAME;                   /* set state of this collection to incomplete frame */
      }
   }
   return returnCode;
}
/* --------------- Type of NMEA2000 message definitions ----------- */
void getISO11783BitsFromCanId(uint16_t id, uint16_t *prio, uint16_t *pgn, uint16_t *src, uint16_t *dst)
{
  unsigned char PF = (unsigned char) (id >> 16u);
  unsigned char PS = (unsigned char) (id >> 8u);
  unsigned char DP = (unsigned char) (id >> 24u) & 1u;

  if (src)
  {
    *src = (unsigned char) id >> 0u;
  }
  if (prio)
  {
    *prio = (unsigned char) ((id >> 26u) & 0x7u);
  }

  if (PF < 240u)
  {
    if (dst)                                                                    /* PDU1 format, the PS contains the destination address */
    {
      *dst = PS;
    }
    if (pgn)
    {
      *pgn = (DP << 16u) + (PF << 8u);
    }
  }
  else
  {
    if (dst)                                                                    /* PDU2 format, the destination is implied global and the PGN is extended */
    {
      *dst = 0xffu;
    }
    if (pgn)
    {
      *pgn = (DP << 16u) + (PF << 8u) + PS;
    }
  }
}

/*
  This does the opposite from getISO11783BitsFromCanId: given n2k fields produces the extended frame CAN id
*/
uint16_t getCanIdFromISO11783Bits(uint16_t prio, uint16_t pgn, uint16_t src, uint16_t dst)
{
  uint16_t canId  = (src & 0xffU) | 0x80000000LLU;                              // src bits are the lowest ones of the CAN ID. Also set the highest bit to 1 as n2k uses only extended frames (EFF bit).

  uint16_t PF = (pgn >> 8U) & 0xffU;

  if (PF < 240U)
  {
    canId |= (dst & 0xffU) << 8U;                                                 // PDU 1
    canId |= (pgn << 8U);
  }
  else
  {
    canId |= pgn << 8U;                                                         // PDU 2
  }
  canId |= prio << 26U;

  return canId;
}
#endif /* NMEA 2000 */

#if defined(YDWG_JSON_USED)
/* ******************** JSON parse ************************************
*/
#define MMSI_LENGTH sizeof("244060807")
#define LAT_LENGTH sizeof("-123.1234567890")
#define LON_LENGTH sizeof("-123.1234567890")
#define ANGLE_LENGTH sizeof("-123.999")
#define SPEED_LENGTH sizeof("-12345.999")
#define OTHER_LENGTH (20)
static void removeChar(char *str, char garbage)
{
  char *src, *dst;

  for (src = dst = str; *src; src++)
  {
    *dst = *src;
    if (*dst != garbage)
    {
      dst++;
    }
  }
  *dst = 0;
}
/*
 * Retrieve a value out of a JSON styled message.
 */
int16_t getJSONValue(const char *message, const char *fieldName, char *value, size_t len)
{
  const char *loc = message + 1;
  size_t fieldLen = strlen((char*)fieldName);
  uint32_t n;

  if ((message==NULL) || (fieldName==NULL))
     return -1;

  for (;;)
  {
    loc = strstr((char*)loc,(char*) fieldName);
    if (!loc)
    {
      return 0;
    }
    if (loc[-1] == '"' && loc[fieldLen] == '"' && loc[fieldLen + 1] == ':')
    {
      break;
    }
    loc += fieldLen;
  }

  loc += fieldLen + 2;                                                          /* field has been found */

  while (isspace(*loc))
  {
    loc++;
  }

  if (*loc != '"')
  {
    while ((isdigit(*loc) || *loc == '.' || *loc == '-' || *loc == 'E' || *loc == 'e' || *loc == '+') && len > 1)
    {
      *value++ = *loc++;
      len--;
    }
    *value = 0;
    return 1;
  }

  loc++;                                                                        /* field is string */

  while (len > 1)
  {
    if (*loc == '\\')
    {
      loc++;
      switch (*loc)
      {
        case 'b':
          *value++ = '\b';
          loc++;
          break;
        case 'f':
          *value++ = '\f';
          loc++;
          break;
        case 'n':
          *value++ = '\n';
          loc++;
          break;
        case 'r':
          *value++ = '\r';
          loc++;
          break;
        case 't':
          *value++ = '\t';
          loc++;
          break;
        case 'u':
        {
/* double check====          sscanf(loc, "%4x", &n); */
          n=atol((char*)loc);
          loc += 4;
          *value++ = n & 255;                                                   // We're single byte, forget about the high byte...
        }
        break;

        default:
          *value++ = *loc++;
      }
    }
    else if (*loc == '"')
    {
      break;
    }
    else
    {
      *value++ = *loc++;
    }
    len--;
  }
  *value = 0;
  return 1;
}
static float64_t convert2kCoordinateToNMEA0183(const char *coordinateString, const char *hemispheres, char *hemisphere)
{
  float64_t coordinate;
  float64_t degrees;
  float64_t result;

  coordinate = atof((char*)coordinateString);
  if (coordinate < 0)
  {
    *hemisphere = hemispheres[1];
    coordinate  = coordinate * -1.;
  }
  else
  {
    *hemisphere = hemispheres[0u];
  }

  degrees = floor(coordinate);
  result  = degrees * 100.0f + (coordinate - degrees) * 60.0f;
  return result;
}
/* === HDG - Heading - Deviation & Variation ===

------------------------------------------------------------------------------
        1   2   3 4   5 6
        |   |   | |   | |
 $--HDG,x.x,x.x,a,x.x,a*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Magnetic Sensor heading in degrees
2. Magnetic Deviation, degrees
3. Magnetic Deviation direction, E = Easterly, W = Westerly
4. Magnetic Variation degrees
5. Magnetic Variation direction, E = Easterly, W = Westerly
6. Checksum


=== HDM - Heading - Magnetic ===

Vessel heading in degrees with respect to magnetic north produced by
any device or system producing magnetic heading.

------------------------------------------------------------------------------
        1   2 3
        |   | |
 $--HDM,x.x,M*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Heading Degrees, magnetic
2. M = magnetic
3. Checksum

=== HDT - Heading - True ===

Actual vessel heading in degrees true produced by any device or system
producing true heading.

------------------------------------------------------------------------------
        1   2 3
        |   | |
 $--HDT,x.x,T*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Heading Degrees, true
2. T = True
3. Checksum

  JSON looks like this :-

 * {"timestamp":"2010-09-12-10:57:41.217","prio":"2","src":"36","dst":"255","pgn":"127250","description":"Vessel
 Heading","fields":{"SID":"116","Heading":"10.1","Deviation":"0.0","Variation":"0.0","Reference":"Magnetic"}}

*/
//static void nmea0183VesselHeading(StringBuffer *msg183, int16_t src, const char *msg)
void jSonVesselHeading( SerialNMEA0183Object_t *msg183, int16_t src, const char *msg)
{
  char headingString[30u];
  char deviationString[30u];
  char variationString[30u];
  char referenceString[30u];
  float64_t dev;
  float64_t var;

  if (getJSONValue(msg, "Heading", headingString, sizeof(headingString)) && getJSONValue(msg, "Reference", referenceString, sizeof(referenceString)))
  {
    if (getJSONValue(msg, "Deviation", deviationString, sizeof(deviationString)) && getJSONValue(msg, "Variation", variationString, sizeof(variationString)) && strcmp(referenceString, "Magnetic") == 0)
    {
      /* Enough info for HDG message */
      //dev = strtod(deviationString, 0);
      //var = strtod(variationString, 0);
      dev=atof(deviationString);
      var=atof(variationString);

      nmea0183CreateMessage(msg183,src,"HDG,%s,%04.1f,%c,%04.1f,%c",headingString,fabs(dev),((dev < 0.0) ? 'W' : 'E'),fabs(var),((var < 0.0) ? 'W' : 'E'));
    }
    else if (strcmp(referenceString, "True") == 0)
    {
      nmea0183CreateMessage(msg183, src, "HDT,%s,T", headingString);
    }
    else if (strcmp(referenceString, "Magnetic") == 0)
    {
      nmea0183CreateMessage(msg183, src, "HDM,%s,M", headingString);
    }
  }
}

/*
=== MWV - Wind Speed and Angle ===

------------------------------------------------------------------------------
        1   2 3   4 5 6
        |   | |   | | |
 $--MWV,x.x,a,x.x,a,a*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Wind Angle, 0 to 360 degrees
2. Reference, R = Relative, T = True
3. Wind Speed
4. Wind Speed Units, K/M/N
5. Active (A) or invalid (V)
6. Checksum

 * {"timestamp":"2010-09-12-11:00:20.269","prio":"2","src":"13","dst":"255","pgn":"130306","description":"Wind Data","fields":{"Wind
 Speed":"5.00","Wind Angle":"308.8","Reference":"Apparent"}}

*/

//static void nmea0183WindData(StringBuffer *msg183, int src, const char *msg)
void jSonWindData(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg)
{
  char speedString[30u];
  char angleString[30u];
  char referenceString[30u];
  float64_t speed=0.0f;

  if (getJSONValue(msg, "Wind Speed", speedString, sizeof(speedString))
      && getJSONValue(msg, "Wind Angle", angleString, sizeof(angleString))
      && getJSONValue(msg, "Reference", referenceString, sizeof(referenceString)))
  {
    //double speed = strtod(speedString, 0);
    speed = atof(speedString);

    if (strcmp(referenceString, "True") == 0)
    {
      nmea0183CreateMessage(msg183, src, "MWV,%s,T,%.1f,K,A", angleString, SPEED_M_S_TO_KMH(speed));
      nmea0183CreateMessage(msg183, src, "MWD,,T,%s,M,%.1f,N,%.1f,M", angleString, SPEED_M_S_TO_KNOTS(speed), speed);
    }
    else if (strcmp(referenceString, "Apparent") == 0)
    {
      nmea0183CreateMessage(msg183, src, "MWV,%s,R,%.1f,K,A", angleString, SPEED_M_S_TO_KMH(speed));
    }
  }
}

/*
=== DBK - Depth Below Keel ===

------------------------------------------------------------------------------
        1   2 3   4 5   6 7
        |   | |   | |   | |
 $--DBK,x.x,f,x.x,M,x.x,F*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Depth, feet
2. f = feet
3. Depth, meters
4. M = meters
5. Depth, Fathoms
6. F = Fathoms
7. Checksum

=== DBS - Depth Below Surface ===

------------------------------------------------------------------------------
        1   2 3   4 5   6 7
        |   | |   | |   | |
 $--DBS,x.x,f,x.x,M,x.x,F*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Depth, feet
2. f = feet
3. Depth, meters
4. M = meters
5. Depth, Fathoms
6. F = Fathoms
7. Checksum

=== DBT - Depth below transducer ===

------------------------------------------------------------------------------
        1   2 3   4 5   6 7
        |   | |   | |   | |
 $--DBT,x.x,f,x.x,M,x.x,F*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Depth, feet
2. f = feet
3. Depth, meters
4. M = meters
5. Depth, Fathoms
6. F = Fathoms
7. Checksum

 * {"timestamp":"2012-12-01-12:53:19.929","prio":"3","src":"35","dst":"255","pgn":"128267","description":"Water
Depth","fields":{"SID":"70","Depth":"0.63","Offset":"0.500"}}
 */
//static void nmea0183WaterDepth(StringBuffer *msg183, int src, const char *msg)
void jSonWaterDepth(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg)
{
  char depthString[30u];
  char offsetString[30u];
  float64_t off=0.0f;
  float64_t dep=0.0f;

  if (getJSONValue(msg, "Depth", depthString, sizeof(depthString))
      && getJSONValue(msg, "Offset", offsetString, sizeof(offsetString)))
  {
    off = atof(offsetString);
    dep = atof(depthString);
    nmea0183CreateMessage(msg183, src, "DPT,%04.1f,%04.1f", dep, off);
  }
}

/*

=== VHW - Water speed and heading ===
------------------------------------------------------------------------------
        1   2 3   4 5   6 7   8 9
        |   | |   | |   | |   | |
 $--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Degress True
2. T = True
3. Degrees Magnetic
4. M = Magnetic
5. Knots (speed of vessel relative to the water)
6. N = Knots
7. Kilometers (speed of vessel relative to the water)
8. K = Kilometers
9. Checksum

 * {"timestamp":"2015-12-07-21:51:11.381","prio":"2","src":"4","dst":"255","pgn":"128259","description":"Speed","fields":{"Speed
 Water Referenced":0.30}}

*/
//static void nmea0183WaterSpeed(StringBuffer *msg183, int src, const char *msg)
void jSonWaterSpeed(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg)
{
  char speedString[30];
  float64_t speed=0.0f;

  if (getJSONValue(msg, "Speed Water Referenced", speedString, sizeof(speedString)))
  {
    speed = atof(speedString);
    nmea0183CreateMessage(msg183, src, "VHW,,T,,M,%04.1f,N,%04.1f,K", SPEED_M_S_TO_KNOTS(speed), SPEED_M_S_TO_KMH(speed));
  }
}


/*

=== MTW - Mean Temperature of Water ===
------------------------------------------------------------------------------
$--MTW,x.x,C*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:
1. Degrees
2. Unit of Measurement, Celcius
3. Checksum

YDWG string :-
{"timestamp":"2018-10-16T22:25:25.181","prio":5,"src":35,"dst":255,"pgn":130311,"description":"Environmental Parameters","fields":{"Temperature Source":"Sea Temperature","Temperature":13.39}}

*/
//static void nmea0183WaterTemperature(StringBuffer *msg183, int src, const char *msg)
void jSonWaterTemperature(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg)
{
  char temperatureString[30u];
  char sourceString[30u];
  float64_t temp = 0.0f;

  if (getJSONValue(msg, "Temperature Source", sourceString, sizeof(sourceString)) && (strcmp(sourceString, "Sea Temperature") == 0)
      && getJSONValue(msg, "Temperature", temperatureString, sizeof(temperatureString)))
  {
    temp = atof(temperatureString);
    nmea0183CreateMessage(msg183, src, "MTW,%04.1f,C", TEMP_K_TO_C(temp));
  }
}

/*
VLW - Distance Traveled through Water
------------------------------------------------------------------------------
       1   2 3   4 5
       |   | |   | |
$--VLW,x.x,N,x.x,N*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:
1. Total cumulative distance
2. N = Nautical Miles
3. Distance since Reset
4. N = Nautical Miles
5. Checksum

 * {"timestamp":"2016-04-20T21:03:57.631Z","prio":6,"src":35,"dst":255,"pgn":128275,"description":"Distance
 Log","fields":{"Log":57688,"Trip Log":57688}}

*/
//static void nmea0183DistanceTraveled(StringBuffer *msg183, int src, const char *msg)
void jSonDistanceTraveled(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg)
{
  char logString[30u];
  char tripString[30u];
  float64_t total=0.0f;
  float64_t trip=0.0f;

  if (getJSONValue(msg, "Log", logString, sizeof(logString)) && getJSONValue(msg, "Trip Log", tripString, sizeof(tripString)))
  {
    total = atof(logString);
    trip  = atof(tripString);

    nmea0183CreateMessage(msg183, src, "VLW,%.1f,N,%.1f,N", DIST_M_TO_NM(total), DIST_M_TO_NM(trip));
  }
}

/*
=== RSA - Rudder Sensor Angle ===
------------------------------------------------------------------------------
        1   2 3   4 5
        |   | |   | |
 $--RSA,x.x,A,x.x,A*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Starboard (or single) rudder sensor, "-" means Turn To Port
2. Status, A means data is valid
3. Port rudder sensor
4. Status, A means data is valid
5. Checksum

 * {"timestamp":"2015-12-09-21:53:47.497","prio":"2","src":"1","dst":"255","pgn":"127245","description":"Rudder","fields":{"Angle
 Order":-0.0,"Position":6.8}}

*/
void jSonRudder(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg)
{
  char positionString[30u];
  float64_t pos = 0.0f;

  if (getJSONValue(msg, "Position", positionString, sizeof(positionString)))
  {
    pos = atof(positionString);
    nmea0183CreateMessage(msg183, src, "RSA,%04.1f,A,,F", -pos);
  }
}
/*
=== GSA - GPS DOP and active satellites
This is one of the sentences commonly emitted by GPS units.

        1 2 3                        14 15  16  17  18
        | | |                         |  |   |   |   |
 $--GSA,a,a,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x.x,x.x,x.x*hh<CR><LF>
Field Number:
1. Selection mode: M=Manual, forced to operate in 2D or 3D, A=Automatic, 3D/2D
2. Mode (1 = no fix, 2 = 2D fix, 3 = 3D fix)
3. ID of 1st satellite used for fix
4. ID of 2nd satellite used for fix
5. ID of 3rd satellite used for fix
6. ID of 4th satellite used for fix
7. ID of 5th satellite used for fix
8. ID of 6th satellite used for fix
9. ID of 7th satellite used for fix
10. ID of 8th satellite used for fix
11. ID of 9th satellite used for fix
12. ID of 10th satellite used for fix
13. ID of 11th satellite used for fix
14. ID of 12th satellite used for fix
15. PDOP
16. HDOP
17. VDOP
18. Checksum

{"timestamp":"2015-12-11T17:30:46.573Z","prio":6,"src":2,"dst":255,"pgn":129539,"description":"GNSS
DOPs","fields":{"SID":177,"Desired Mode":"3D","Actual Mode":"3D","HDOP":0.97,"VDOP":1.57,"TDOP":327.67}}
*/

void jSonGSA(SerialNMEA0183Object_t *msg183, int src, const char *msg)
{
  char modeString[OTHER_LENGTH] = "";
  char pdopString[OTHER_LENGTH] = "";
  char hdopString[OTHER_LENGTH] = "";
  char vdopString[OTHER_LENGTH] = "";

  getJSONValue(msg, "Actual Mode", modeString, sizeof(modeString));
  getJSONValue(msg, "PDOP", pdopString, sizeof(pdopString));
  getJSONValue(msg, "HDOP", hdopString, sizeof(hdopString));
  getJSONValue(msg, "VDOP", vdopString, sizeof(vdopString));

  modeString[1u] = 0;                                                           // Abbreviate string, or still empty if not in N2K PGN

  nmea0183CreateMessage(msg183, src, "GSA,M,%s,,,,,,,,,,,,,%s,%s,%s", modeString, pdopString, hdopString, vdopString);
}
/*
 * {"timestamp":"2019-02-02T19:45:02.051Z","prio":2,"src":43,"dst":255,"pgn":129025,"description":"Position, Rapid
 Update","fields":{"Latitude":37.8670000,"Longitude":-122.3150000}}

*/
/*
=== GLL - Geographic Position - Latitude/Longitude ===

This is one of the sentences commonly emitted by GPS units.

        1       2 3        4 5         6 7   8
        |       | |        | |         | |   |
 $--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,a,m,*hh<CR><LF>

Field Number:

1. Latitude
2. N or S (North or South)
3. Longitude
4. E or W (East or West)
5. Universal Time Coordinated (UTC)
6. Status A - Data Valid, V - Data Invalid
7. FAA mode indicator (NMEA 2.3 and later)
8. Checksum

{"timestamp":"2015-12-11T19:59:22.399Z","prio":2,"src":2,"dst":255,"pgn":129025,"description":"Position, Rapid
Update","fields":{"Latitude":36.1571104,"Longitude":-5.3561568}}
{"timestamp":"2015-12-11T20:01:19.010Z","prio":3,"src":2,"dst":255,"pgn":129029,"description":"GNSS Position
Data","fields":{"SID":10,"Date":"2015.12.11", "Time": "20:01:19","Latitude":36.1571168,"Longitude":-5.3561616,"GNSS
type":"GPS+SBAS/WAAS","Method":"GNSS fix","Integrity":"Safe","Number of SVs":12,"HDOP":0.86,"PDOP":1.68,"Geoidal
Separation":-0.01,"Reference Station ID":4087}} $GPGLL,3609.42711,N,00521.36949,W,200015.00,A,D*72
*/

void jSonGLL(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg)
{
  char latString[LAT_LENGTH];
  char lonString[LON_LENGTH];

  if (getJSONValue(msg, "Latitude", latString, sizeof(latString)) && getJSONValue(msg, "Longitude", lonString, sizeof(lonString)))
  {
    char timeString[OTHER_LENGTH] = "";
    char latHemisphere;
    char lonHemisphere;
    float64_t latitude  = convert2kCoordinateToNMEA0183(latString, "NS", &latHemisphere);
    float64_t longitude = convert2kCoordinateToNMEA0183(lonString, "EW", &lonHemisphere);

    if (getJSONValue(msg, "Time", timeString, sizeof(timeString)))
    {
      removeChar(timeString, ':');
    }

    nmea0183CreateMessage(msg183, src, "GLL,%.4f,%c,%.4f,%c,%s,A,D", latitude, latHemisphere, longitude, lonHemisphere, timeString);
  }
}
/*
=== VTG - Track made good and Ground speed ===
This is one of the sentences commonly emitted by GPS units.

         1  2  3  4  5  6  7  8 9   10
         |  |  |  |  |  |  |  | |   |
 $--VTG,x.x,T,x.x,M,x.x,N,x.x,K,m,*hh<CR><LF>

Field Number:
1. Track Degrees
2. T = True
3. Track Degrees
4. M = Magnetic
5. Speed Knots
6. N = Knots
7. Speed Kilometers Per Hour
8. K = Kilometers Per Hour
9. FAA mode indicator (NMEA 2.3 and later)
10. Checksum

{"timestamp":"2015-12-10T22:19:45.330Z","prio":2,"src":2,"dst":255,"pgn":129026,"description":"COG & SOG, Rapid
Update","fields":{"SID":9,"COG Reference":"True","COG":0.0,"SOG":0.00}} $GPVTG,,T,,M,0.150,N,0.278,K,D*2F<0x0D><0x0A>
*/
void jSonVTG(SerialNMEA0183Object_t *msg183, int16_t src, const char *msg)
{
  char sogString[SPEED_LENGTH];
  char cogString[ANGLE_LENGTH];
  float64_t cog;
  float64_t sog;

  if (getJSONValue(msg, "SOG", sogString, sizeof(sogString)) && getJSONValue(msg, "COG", cogString, sizeof(cogString)))
  {
    sog = atof(sogString);
    cog = atof(cogString);
    nmea0183CreateMessage( msg183, src, "VTG,%s,T,,M,%04.3f,N,%04.3f,K", cogString, SPEED_M_S_TO_KNOTS(sog), SPEED_M_S_TO_KMH(sog));
  }
}
/*
    Read a message from YDWG - JSON string and write to NMEA0183
*/
void readJSON(SerialNMEA0183Object_t *msg183, const char *msg)
{
  char str[20u];
  int16_t prn=0;
  int16_t src=0;
  int16_t rateType=0;

  if (!getJSONValue(msg, "pgn", str, sizeof(str)))
  {
    return;
  }
  prn = atoi(str);

  switch (prn)
  {
    case PGN_VESSEL_HEADING:
      rateType = RATE_VESSEL_HEADING;
      break;
    case PGN_WIND_DATA:
      rateType = RATE_WIND_DATA;
      break;
    case PGN_WATER_DEPTH:
      rateType = RATE_WATER_DEPTH;
      break;
    case PGN_WATER_SPEED:
      rateType = RATE_WATER_SPEED;
      break;
    case PGN_ENVIRONMENTAL:
      rateType = RATE_ENVIRONMENTAL;
      break;
    case PGN_DISTANCE_LOG:
      rateType = RATE_DISTANCE_LOG;
      break;
    case PGN_RUDDER:
      rateType = RATE_RUDDER;
      break;
    case PGN_SOG_COG:
      rateType = RATE_GPS_SPEED;
      break;
    case PGN_GPS_DOP:
      rateType = RATE_GPS_DOP;
      break;
    case PGN_POSITION:
    case PGN_POSITION_RAPID:
      rateType = RATE_GPS_POSITION;
      break;
    case PGN_AIS_A:
    case PGN_AIS_B:
    case PGN_AIS_4:
    case PGN_AIS_5:
    case PGN_AIS_9:
    case PGN_AIS_12:
    case PGN_AIS_14:
    case PGN_AIS_19:
    case PGN_AIS_21:
    case PGN_AIS_24A:
    case PGN_AIS_24B:
      rateType = RATE_NO_LIMIT;
      break;
    default:
      return;
  }

/*  if (!getJSONValue(msg, "src", str, sizeof(str)))
  {
    return;
  }
  src = atoi(str);
  if (srcFilter && !matchFilter(src, srcFilter))
  {
    return;
  }

  logDebug("NMEA passed filter for prn %d src %d\n", src, prn);

  if (rateLimit && rateType != RATE_NO_LIMIT)
  {
    int64_t now = epoch();

    if (rateLimitPassed[src][rateType] > (now - 1000L))
    {
      logDebug("Ratelimit for prn %d src %d not reached\n", src, prn);
      return;
    }
    rateLimitPassed[src][rateType] = now;
    logDebug("Ratelimit passed for prn %d src %d\n", src, prn);
  } */

  switch (prn)
  {
    case PGN_VESSEL_HEADING:
      jSonVesselHeading(msg183, src, msg);
      break;
    case PGN_WIND_DATA:
      jSonWindData(msg183, src, msg);
      break;
    case PGN_WATER_DEPTH:
      jSonWaterDepth(msg183, src, msg);
      break;
    case PGN_WATER_SPEED:
      jSonWaterSpeed(msg183, src, msg);
      break;
    case PGN_ENVIRONMENTAL:
      jSonWaterTemperature(msg183, src, msg);
      break;
    case PGN_DISTANCE_LOG:
      jSonDistanceTraveled(msg183, src, msg);
      break;
    case PGN_RUDDER:
      jSonRudder(msg183, src, msg);
      break;
    case PGN_SOG_COG:
      jSonVTG(msg183, src, msg);
      break;
    case PGN_GPS_DOP:
      jSonGSA(msg183, src, msg);
      break;
    case PGN_POSITION:
    case PGN_POSITION_RAPID:
      jSonGLL(msg183, src, msg);
      break;
    case PGN_AIS_A:
    case PGN_AIS_B:
    case PGN_AIS_4:
    case PGN_AIS_5:
    case PGN_AIS_9:
    case PGN_AIS_12:
    case PGN_AIS_14:
    case PGN_AIS_19:
    case PGN_AIS_21:
    case PGN_AIS_24A:
    case PGN_AIS_24B:
      //nmea0183AIVDM(msg183, src, msg);
      break;
    default:
      return;
  }
}
#endif /* YDWG JSON */




/*
 * Table 1 - Mapping of ISO 11783 into CAN's Arbitration and Control Fields 29 Bit Identifiers
CAN   ISO 11783 Bit Number
SOF     SOF*      1
ID 28   P 3       2
ID 27   P 2       3
ID 26   P 1       4
ID 23   R 1       5
ID 24   DP        6
ID 23   PF 8      7
ID 22   PF 7      8
ID 21   PF 6      9
ID 20   PF 5     10
ID 19   PF 4     11
ID 18   PF 3     12
SRR (r) SRR*     13
IDE (r) IDE*     14
ID 17   PF 2     15
ID 16   PF 1     16
ID 13   PS 8     17
ID 14   PS 7     18
ID 13   PS 6     19
ID 12   PS 5     20
ID 11   PS 4     21
ID 10   PS 3     22
ID 9    PS 2     23
ID 8    PS 1     24
ID 7    SA 8     25
ID 6    SA 7     26
ID 3    SA 6     27
ID 4    SA 5     28
ID 3    SA 4     29
ID 2    SA 3     30
ID 1    SA 2     31
ID 0    SA 1     32
RTR (x) RTR*     33
r 1     r 1*     34
r 0     r 0*     35
DLC 4   DLC 4    36
DLC 3   DLC 3    37
DLC 2   DLC 2    38
DLC 1   DLC 1    39
Notes:
SOF - Start of Frame Bit P# - ISO 11783 Priority Bit #n
ID## - Identifier Bit #n R# - ISO 11783 Reserved Bit #n
SRR - Substitute Remote Request SA# - ISO 11783 Source Address Bit #n
RTR - Remote Transmission Request Bit DP - ISO 11783 Data Page
IDE - Identifier Extension Bit PF# - ISO 11783 PDU Format Bit #n
r# - CAN Reserved Bit #n PS# - ISO 11783 PDU Specific Bit #n
DLC# - Data Length Code Bit #n *CAN Defined Bit, Unchanged in ISO 11783
(d) - dominant bit 1 Required format of proprietary 11 bit identifiers
(r) - recessive bit
*/


#if defined(NMEA0183_USED)
/* %%%%%%%%%%%%%%%%%%%%%%%%%% NMEA0183 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
/*
 * Which PGNs do we care and know about for now?
 *
 * NMEA 0183 information from the excellent reference at
 * www.catb.org/gpsd/NMEA.txt
 *
 * PGN 127250 "Vessel Heading" -> $xxHDG
 * PGN 130306 "Wind Data"      -> $xxMWV
 * PGN 128267 "Water Depth"    -> $xxDBK/DBS/DBT
 * PGN 128267 "Water Speed"    -> $xxVHW
 * PGN 127245 "Rudder"         -> $xxRSA
 * PGN 130311 "Environmental Parameters - water temperature" -> $xxMTW
 * PGN 128275 "Distance Log"   -> $xxVLW

 * Some others are in gps_ais.c file
 * PGN 129026 "Track made good and Ground speed"           -> $xxVTG
 * PGN 129539 "GPS DOP"                                    -> $xxGSA
 * PGN 129025 or 129029 "GPS Position"                     -> $xxGLL
 * PGN 129038 "Class A Position Report"                    -> !AIVDM
 * PGN 129039 "AIS Class B Position Report"                -> !AIVDM
 * PGN 129040 "AIS Class B Extended Position Report"       -> !AIVDM
 * PGN 129041 "AIS Aids to Navigation (AtoN) Report"       -> !AIVDM
 * PGN 129793 "AIS UTC and Date Report"                    -> !AIVDM
 * PGN 129794 "AIS Class A Static and Voyage Related Data" -> !AIVDM
 * PGN 129798 "AIS SAR Aircraft Position Report"           -> !AIVDM   PGN incomplete
 * PGN 129801 "AIS Addressed Safety Related Message"       -> !AIVDM
 * PGN 129802 "AIS Safety Related Broadcast Message"       -> !AIVDM   PGN incomplete
 * PGN 129809 "AIS Class B "CS" Static Data Report, Part A"-> !AIVDM
 * PGN 129810 "AIS Class B "CS" Static Data Report, Part B"-> !AIVDM
 * Typical output of these from analyzer:
 * {"timestamp":"2010-09-12-10:57:41.217","prio":"2","src":"36","dst":"255","pgn":"127250","description":"Vessel
 Heading","fields":{"SID":"116","Heading":"10.1","Deviation":"0.0","Variation":"0.0","Reference":"Magnetic"}}
 * {"timestamp":"2010-09-12-11:00:20.269","prio":"2","src":"13","dst":"255","pgn":"130306","description":"Wind Data","fields":{"Wind
 Speed":"5.00","Wind Angle":"308.8","Reference":"Apparent"}}
 * {"timestamp":"2012-12-01-12:53:19.929","prio":"3","src":"35","dst":"255","pgn":"128267","description":"Water
 Depth","fields":{"SID":"70","Depth":"0.63","Offset":"0.500"}}
 * {"timestamp":"2015-12-07-21:51:11.381","prio":"2","src":"4","dst":"255","pgn":"128259","description":"Speed","fields":{"Speed
 Water Referenced":0.30}}
 * {"timestamp":"2015-12-09-21:53:47.497","prio":"2","src":"1","dst":"255","pgn":"127245","description":"Rudder","fields":{"Angle
 Order":-0.0,"Position":6.8}}
 * {"timestamp":"2015-12-11T17:56:55.755Z","prio":6,"src":2,"dst":255,"pgn":129539,"description":"GNSS
 DOPs","fields":{"SID":239,"Desired Mode":"3D","Actual Mode":"3D","HDOP":1.21,"VDOP":1.83,"TDOP":327.67}}
 * {"timestamp":"2016-04-14T20:27:02.303Z","prio":5,"src":35,"dst":255,"pgn":130311,"description":"Environmental
 Parameters","fields":{"SID":222,"Temperature Source":"Sea Temperature","Temperature":17.16}}
 * {"timestamp":"2016-04-20T21:03:57.631Z","prio":6,"src":35,"dst":255,"pgn":128275,"description":"Distance
 Log","fields":{"Log":57688,"Trip Log":57688}}
 * {"timestamp":"2019-02-02T19:45:02.051Z","prio":2,"src":43,"dst":255,"pgn":129025,"description":"Position, Rapid
 Update","fields":{"Latitude":37.8670000,"Longitude":-122.3150000}}
 */
void nmea0183CreateMessage(SerialNMEA0183Object_t *msg183, int16_t src, const char *format, ...)
{
  uint16_t chk;
  size_t i;
  char first, second;
  va_list ap;

  va_start(ap, format);

  // Convert the 8 bit value 'n' into a valid NMEA0183 style sender.
  // The first implementation sent out a 2 digit hexadecimal number,
  // but that throws some implementations of receivers off as they
  // cannot handle numeric senders. So now we produce a 2 character
  // code with the src value 0-255 translated into
  // A..Q A..P with A representing 0, B representing 1, etc.
  // P is skipped for the initial letter, as that represents 'proprietary'
  // and OpenCPN does not allow this.

  first  = 'A' + ((src >> 4) & 0xf);
  second = 'A' + ((src) &0xf);
  if (first >= 'P')
  {
    first++;
  }
  i = msg183->Len;                                                              // Prepare for calculation of checksum

  //if (src > 255)
    //sbAppendFormat(msg183, "!%c%c", first, second);
  //else
    //sbAppendFormat(msg183, "$%c%c", first, second);
  //sbAppendFormatV(msg183, format, ap);
  if (src > 255)
     sprintf((char*)&msg183->Buffer,"%s!%c%c",(char*)&msg183->Buffer,first,second);
  else
     sprintf((char*)&msg183->Buffer,"%s$%c%c",(char*)&msg183->Buffer,first,second);
  sprintf((char*)&msg183->Buffer,format,ap);
  //va_end(ap);

  chk = 0;
  for (i++; i < msg183->Len; i++)
  {
    chk ^= (uint16_t) msg183->Buffer[i];
  }
  //sbAppendFormat(msg183, "*%02X\r\n", chk);
  sprintf((char*)&msg183->Buffer,"%s*%02X\r\n",(char*)&msg183->Buffer,chk);
  //logDebug("nmea0183 = %s", sbGet(msg183));
}

#endif /* NMEA0183 */

// correct for angle of heal, this is determined by experiment. Anenomiters have a very non linear
// behavior. Ultrasound sensors tend to follow a much more idealised model. The idealised model
// assumes the annenomiter follows a cosine rule. This function corrects the annenomiter reading
// to match the cosine rule, first by correcting to the horizontal and then
// correcting to give the masthead speed in the masthead co-ordinates.From
// see http://www.dewi.de/dewi/fileadmin/pdf/publications/Publikations/S09_2.pdf, figure 9 A100LM-ME1
// which appears to be close to most marine annenomiters, above
float64_t correctWindSpeedForHeal(float64_t v, float64_t angleOfHeal)
{
      if ( angleOfHeal > 0.174533f)                                              // > 10 degress + 3%
      {
        return v*1.03f;
      }
      else if ( angleOfHeal > 0.139626f)                                        // >8 degrees +2%
      {
        return v*1.02f;
      }
      else if ( angleOfHeal > 0.10472f )                                        // 6 degrees +1%
      {
        return v*1.01f;
      }
      return v*cos(angleOfHeal);                                                // now the speed is corrected for angle of heal reative horizontal. apply the cosine rule to correct for angle of heal.
}

#endif /* MARINE_PROTO */