//-------------------------------------------------------------------------------------------------
// The LWNX protocol is a binary based protocol for reading and writing data to LightWare devices.
// Taken from Robert Gowans original library
// https://github.com/LightWare-Optoelectronics/sf40-samples/tree/master/src
//
//-------------------------------------------------------------------------------------------------
// =============== Ported to PIC32 device miKroE C by ACP Aviation ===============================
//
//
//    LwNx.c : protocol functions
//
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#include <stdint.h>                                                             // standard type defintions
#include"lwNx_defs.h"                                                           // definitions of the protocol
#include "crc.h"                                                                // cyclic redundancy check

#ifdef lwserial                                                                 // lwserial means original library not pic32 version being used. use definitions.h
#pragma once
#endif

#ifdef lwserial
#include "common.h"
#endif

//----------------------------------------------------------------------------------------------------------------------------------
// Helper utilities.
//----------------------------------------------------------------------------------------------------------------------------------
// Create a CRC-16-CCITT 0x1021 hash of the specified data.
//extern uint16_t lwnxCreateCrc(uint8_t* Data, uint16_t Size);

// Breaks an integer firmware version into Major, Minor, and Patch.
void lwnxConvertFirmwareVersionToStr(uint32_t Version, char* String);
//----------------------------------------------------------------------------------------------------------------------------------
// LWNX protocol implementation.
//----------------------------------------------------------------------------------------------------------------------------------
// Prepare a response packet for a new incoming response.
void lwnxInitResponsePacket(lwResponsePacket* Response);

#ifdef lwserial                                                                 // for linux or windows device
// Waits to receive a packet of specific command id.
// Does not return until a response is received or a timeout occurs.
uint8_t lwnxRecvPacket(lwSerialPort* Serial, uint8_t CommandId, lwResponsePacket* Response, uint32_t TimeoutMs);

// Returns true if full packet was received, otherwise finishes immediately and returns false while waiting for more data.
uint8_t lwnxRecvPacketNoBlock(lwSerialPort* Serial, uint8_t CommandId, lwResponsePacket* Response);

// Composes and sends a packet.
void lwnxSendPacketBytes( lwSerialPort* Serial, uint8_t CommandId, uint8_t Write, uint8_t* Data, uint32_t DataSize );

// Handle both the sending and receving of a command.
// Does not return until a response is received or all retries have expired.
uint8_t lwnxHandleManagedCmd(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize, bool Write = false, uint8_t* WriteData = NULL, uint32_t WriteSize = 0);

//----------------------------------------------------------------------------------------------------------------------------------
// Command functions.
//----------------------------------------------------------------------------------------------------------------------------------
// Issue read commands.
uint8_t lwnxCmdReadInt8(lwSerialPort* Serial, uint8_t CommandId, int8_t* Response);
uint8_t lwnxCmdReadInt16(lwSerialPort* Serial, uint8_t CommandId, int16_t* Response);
uint8_t lwnxCmdReadInt32(lwSerialPort* Serial, uint8_t CommandId, int32_t* Response);
uint8_t lwnxCmdReadUInt8(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response);
uint8_t lwnxCmdReadUInt16(lwSerialPort* Serial, uint8_t CommandId, uint16_t* Response);
uint8_t lwnxCmdReadUInt32(lwSerialPort* Serial, uint8_t CommandId, uint32_t* Response);
uint8_t lwnxCmdReadString(lwSerialPort* Serial, uint8_t CommandId, char* Response);
uint8_t lwnxCmdReadData(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize);

// Issue write commands.
uint8_t lwnxCmdWriteInt8(lwSerialPort* Serial, uint8_t CommandId, int8_t Value);
uint8_t lwnxCmdWriteInt16(lwSerialPort* Serial, uint8_t CommandId, int16_t Value);
uint8_t lwnxCmdWriteInt32(lwSerialPort* Serial, uint8_t CommandId, int32_t Value);
uint8_t lwnxCmdWriteUInt8(lwSerialPort* Serial, uint8_t CommandId, uint8_t Value);
uint8_t lwnxCmdWriteUInt16(lwSerialPort* Serial, uint8_t CommandId, uint16_t Value);
uint8_t lwnxCmdWriteUInt32(lwSerialPort* Serial, uint8_t CommandId, uint32_t Value);
uint8_t lwnxCmdWriteString(lwSerialPort* Serial, uint8_t CommandId, char* String);
uint8_t lwnxCmdWriteData(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Data, uint32_t DataSize);
#else                                                                           // for pic32 processor

// Waits to receive a packet of specific command id.
// Does not return until a response is received or a timeout occurs.
uint8_t lwnxRecvPacket(uint8_t CommandId, lwResponsePacket* Response, uint32_t TimeoutTicks, uint8_t uART_no);

// Returns true if full packet was received, otherwise finishes immediately and returns false while waiting for more data.
uint8_t lwnxRecvPacketNoBlock(uint8_t CommandId, lwResponsePacket* Response, uint8_t uART_no);

// Composes and sends a packet.
void lwnxSendPacketBytes(uint8_t CommandId, uint8_t Write, uint8_t* Data, uint32_t DataSize, uint8_t uART_no );

// Handle both the sending and receving of a command.
// Does not return until a response is received or all retries have expired.
uint8_t lwnxHandleManagedCmd(uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize, uint8_t Write, uint8_t* WriteData, uint32_t WriteSize, uint8_t uART_no);

//----------------------------------------------------------------------------------------------------------------------------------
// Command functions.
//----------------------------------------------------------------------------------------------------------------------------------
// Issue read commands.
uint8_t lwnxCmdReadInt8(uint8_t CommandId, int8_t* Response, uint8_t uART_no);
uint8_t lwnxCmdReadInt16(uint8_t CommandId, int16_t* Response, uint8_t uART_no);
uint8_t lwnxCmdReadInt32(uint8_t CommandId, int32_t* Response, uint8_t uART_no);
uint8_t lwnxCmdReadUInt8(uint8_t CommandId, uint8_t* Response, uint8_t uART_no);
uint8_t lwnxCmdReadUInt16(uint8_t CommandId, uint16_t* Response, uint8_t uART_no);
uint8_t lwnxCmdReadUInt32(uint8_t CommandId, uint32_t* Response, uint8_t uART_no);
uint8_t lwnxCmdReadString(uint8_t CommandId, char* Response, uint8_t uART_no);
uint8_t lwnxCmdReadData(uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize, uint8_t uART_no);

// Issue write commands.
uint8_t lwnxCmdWriteInt8(uint8_t CommandId, int8_t Value, uint8_t uART_no);
uint8_t lwnxCmdWriteInt16(uint8_t CommandId, int16_t Value, uint8_t uART_no);
uint8_t lwnxCmdWriteInt32(uint8_t CommandId, int32_t Value, uint8_t uART_no);
uint8_t lwnxCmdWriteUInt8(uint8_t CommandId, uint8_t Value, uint8_t uART_no);
uint8_t lwnxCmdWriteUInt16(uint8_t CommandId, uint16_t Value, uint8_t uART_no);
uint8_t lwnxCmdWriteUInt32(uint8_t CommandId, uint32_t Value, uint8_t uART_no);
uint8_t lwnxCmdWriteString(uint8_t CommandId, char* String, uint8_t uART_no);
uint8_t lwnxCmdWriteData(uint8_t CommandId, uint8_t* Data, uint32_t DataSize, uint8_t uART_no);
#endif
//----------------------------------------------------------------------------------------------------------------------------------
// Helper utilities.
//----------------------------------------------------------------------------------------------------------------------------------
// Create a CRC-16-CCITT 0x1021 hash of the specified data.
uint16_t lwnxCreateCrc(uint8_t* Data, uint16_t Size);

// Breaks an integer firmware version into Major, Minor, and Patch.
void lwnxConvertFirmwareVersionToStr(uint32_t Version, char* String);
//----------------------------------------------------------------------------------------------------------------------------------

// LWNX protocol implementation.

//----------------------------------------------------------------------------------------------------------------------------------
// Prepare a response packet for a new incoming response.
void lwnxInitResponsePacket(lwResponsePacket* Response);

// ================== Functions ================================================
/*-----------------------------------------------------------------------------
 *      lwnxCreateCrc :  Calculate the CRC
 *  Parameters: uint8_t* Data, uint16_t Size
 *  Return:
 *         uint16_t : Cyclic redundancy word 16 bit
 *----------------------------------------------------------------------------*/
uint16_t lwnxCreateCrc(uint8_t* Data, uint16_t Size)
{
        uint16_t crc = 0U;
        uint32_t i=1U;
        uint16_t code1=0U;

        for (i = 0U; i < Size; ++i)
        {
           code1 = (crc >> 8U);
           code1 ^= Data[i];
           code1 ^= (code1 >> 4U);
           crc = (crc << 8U);
           crc ^= code1;
           code1 = (code1 << 5U);
           crc ^= code1;
           code1 = (code1 << 7U);
           crc ^= code1;
        }
        return crc;
}
/*-----------------------------------------------------------------------------
 *      lwnxConvertFirmwareVersionToStr :  Initializes the response structure
 *  Parameters: uint32_t Version, char* String
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void lwnxConvertFirmwareVersionToStr(uint32_t Version, char* String)
{
        uint32_t major = (Version >> 16U) & 0xFFU;
        uint32_t minor = (Version >> 8U) & 0xFFU;
        uint32_t patch = (Version >> 0U) & 0xFFU;
        sprintf(String, "%d.%d.%d", major, minor, patch);
}

/*-----------------------------------------------------------------------------
 *      lwnxInitResponsePacket :  Initializes the response structure
 *  Parameters: lwResponsePacket* Response
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void lwnxInitResponsePacket(lwResponsePacket* Response)
{
        Response->size = 0U;
        Response->payloadSize = 0U;
        Response->parseState = 0U;
}

/*-----------------------------------------------------------------------------
 *      lwnxParseData :  Parses the incoming characture data and puts into
 *  response structure, setting state when full message recieved
 *  Parameters: lwResponsePacket* Response, uint8_t Data1
 *  Return:     1 when a successful packet has been recieved
 *              0 when collecting
 *              2 wrong crc
 *              3 packet too long
 *----------------------------------------------------------------------------*/
uint8_t lwnxParseData(lwResponsePacket* Response, uint8_t Data1)
{
   uint16_t crc;
   uint16_t verifyCrc;
   
        if (Response->parseState == 0U)                                         /* look for a start byte */
        {                                                                       // hasnt seen the message yet
          if (Data1 == PACKET_START_BYTE) 
          {                                                                     // stx is found 0xAA
             Response->parseState = 1U;
             Response->dataVal[0U] = PACKET_START_BYTE;
           }
        } 
        else if (Response->parseState == 1U)                                    // read the flags
        {
            Response->parseState = 2U;
            Response->dataVal[1U] = Data1;                                      // flags byte
        } 
        else if (Response->parseState == 2U) 
        {                                                                       // next step read the length
           Response->parseState = 3U;                                           // go to payloadstep
           Response->dataVal[2U] = Data1;                                       // read the next byte
           Response->payloadSize = (Response->dataVal[1U] | (Response->dataVal[2U] << 8U)) >> 6U;      // make the length byte
           Response->payloadSize += 2U;                                         // add two bytes to the payload size
           Response->size = 3U;
           if (Response->payloadSize > 1019U)
           {                                                                    // if payload length read is greater than maximum
              Response->parseState = 0U;                                        // reset the message processed state
              return 3U;                                                        // return with an error code debug = printf("Packet too long\n");
           }
        } 
        else if (Response->parseState == 3U)                                    // read payload step
        {
           Response->dataVal[Response->size++] = Data1;                         // read in the payload and increment the index
           if (--Response->payloadSize == 0U)                                   // subtract from the size and read next unless its zero
           {
              crc = Response->dataVal[Response->size - 2U] | (Response->dataVal[Response->size - 1U] << 8U); // make the crc that was read in
              verifyCrc = lwnxCreateCrc(Response->dataVal, Response->size - 2U);     // now calculate the the crc
              Response->parseState = 0U;                                        // now reset the parse state to the start for a new message
              if (crc == verifyCrc)                                             // packet matches then return it
              {
                 //Response->parseState = 0U;
                 return 1U;                                                     // packet is good and complete return 1
              }
              else
              {
                 //Response->parseState = 0U;
                 return 2U;                                                     // packet has wrong crc return error printf("Packet has invalid CRC\n");
              }
           }
        }
        return 0U;                                                              // packet is incomplete
}

/*-----------------------------------------------------------------------------
 *      lwnxRecvPacketNoBlock :  Receives and parses a message from lwnx protocol
 *  until the requested message id matches
 *  Parameters: uint8_t CommandId, lwResponsePacket* Response, uint8_t uART_no
 *  Return:     1 when the char matches the message id
 *----------------------------------------------------------------------------*/
#ifdef lwserial                                                                 // if not mikroE PIC32
uint8_t lwnxRecvPacketNoBlock(lwSerialPort* Serial, uint8_t CommandId, lwResponsePacket* Response)
#else
uint8_t lwnxRecvPacketNoBlock(uint8_t CommandId, lwResponsePacket* Response, uint8_t uART_no)
#endif
{
        uint8_t byte = 0U;
        int8_t cmdId = 0U;
#ifndef lwserial                                                                // if mikroE pic32
        int32_t bytesRead;
        switch (uART_no)                                                        // read from the chosen UART
        {
           case 1u:
           if (UART1_Data_Ready())
           {
              byte = UART1_Read();
              bytesRead = 1U;
           }
           break;

           case 2u:
           if (UART2_Data_Ready())
           {
              byte = UART2_Read();
              bytesRead = 1U;
           }
           break;

           case 3u:
           if (UART3_Data_Ready())
           {
              byte = UART3_Read();
              bytesRead = 1U;
           }
           break;

           case 4u:
           if (UART4_Data_Ready())
           {
              byte = UART4_Read();
              bytesRead = 1U;
           }
           break;

           case 5u:
           if (UART5_Data_Ready())
           {
              byte = UART5_Read();
              bytesRead = 1U;
           }
           break;

           case 6u:
           if (UART6_Data_Ready())
           {
              byte = UART6_Read();
              bytesRead = 1U;
           }
           break;
           
           default:                                                             // not a known port
           bytesRead=0U;
           break;
     }
#else
        int32_t bytesRead = Serial->readData(&byte, 1U);                        // serial read of byte
#endif
        if (bytesRead != 0U) {
                if (lwnxParseData(Response, byte)) {
                        cmdId = Response->dataVal[3U];
                        if (cmdId == CommandId) {
                                return 1U;                                      // return 1 when it matches the command id you want
                        }
                }
        }
        return 0U;                                                              // not the command you want yet
}

/*-----------------------------------------------------------------------------
 *      lwnxRecvPacket :  Recieves packets of lwnx
 *  Parameters: uint8_t CommandId, lwResponsePacket* Response, uint32_t TimeoutMs
 *  Return:
 *         uint16_t : Cyclic redundancy word 16 bit
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxRecvPacket(lwSerialPort* Serial, uint8_t CommandId, lwResponsePacket* Response, uint32_t TimeoutMs)
#else
uint8_t lwnxRecvPacket(uint8_t CommandId, lwResponsePacket* Response, uint32_t TimeoutTicks, uint8_t uART_no)
#endif
{

#ifdef lwserial
        uint32_t timeoutTime = platformGetMillisecond() + TimeoutMs;
#else
        uint32_t timeoutTime=0U;                                                // here we are using a tick counter rather than a global interrupt ms (can use a global timer interrupt)
#endif
        uint8_t byte;
        int32_t bytesRead;
        int8_t cmdId;
        lwnxInitResponsePacket(Response);                                       // initialise the response structures
        
#ifdef lwserial
        while (platformGetMillisecond() < timeoutTime)                          // wait for timeout
        {
#else
        while (timeoutTime<=TimeoutTicks)                                       // in PIC32 better interrupt drive but here we read for a finite time
        {
#endif
           byte = 0U;
           bytesRead = 0U;
#ifdef lwserial
           while ((bytesRead = Serial->readData(&byte, 1U)) > 0U) {
#else
           switch (uART_no)                                                     // read from the chosen UART
           {
               case 1u:
               if (UART1_Data_Ready())                                          // data on the port
               {
                   byte = UART1_Read();                                         // read it
                   bytesRead = ++bytesRead % UINT32_MAX;                        // increment the byte count
               }
               break;

               case 2u:
               if (UART2_Data_Ready())
               {
                   byte = UART2_Read();
                   bytesRead = ++bytesRead % UINT32_MAX;
               }
               break;

               case 3u:
               if (UART3_Data_Ready())
               {
                   byte = UART3_Read();
                   bytesRead = ++bytesRead % UINT32_MAX;
               }
               break;

               case 4u:
               if (UART4_Data_Ready())
               {
                  byte = UART4_Read();
                  bytesRead = ++bytesRead % UINT32_MAX;
               }
               break;

               case 5u:
               if (UART5_Data_Ready())
               {
                  byte = UART5_Read();
                  bytesRead = ++bytesRead % UINT32_MAX;
               }
               break;

               case 6u:
               if (UART6_Data_Ready())
               {
                  byte = UART6_Read();
                  bytesRead = ++bytesRead % UINT32_MAX;
               }
               break;

               default:                                                         // not a known port
               bytesRead=0U;
               break;
            }
            if (bytesRead>=1U) {                                                // You read something
#endif
                if (lwnxParseData(Response, byte)) {                            // collect the byte into the response structure
                   cmdId = Response->dataVal[3U];                               // command id is byte 3
                   // printf("Got packet: %d\n", cmdId);
                   // printf("Recv ");
                   // printHexDebug(Response->data, Response->size);
                   if (cmdId == CommandId) {
                      return 1U;                                                // matches request message id then return with a 1
                   }
                }
            }
            timeoutTime = ++timeoutTime % UINT32_MAX;                           // increment the timed read counter
        }
        return 0U;                                                              // else return 0
}

/*-----------------------------------------------------------------------------
 *      lwnxSendPacketBytes :  Sends packets of lwnx
 *  Parameters: uint8_t CommandId, uint8_t Write, uint8_t* Data, uint32_t DataSize, uint8_t uART_no
 *  Return:
 *         nothing
 *----------------------------------------------------------------------------*/
#ifdef lwserial
void lwnxSendPacketBytes(lwSerialPort* Serial, uint8_t CommandId, uint8_t Write, uint8_t* Data, uint32_t DataSize)
#else
void lwnxSendPacketBytes(uint8_t CommandId, uint8_t Write, uint8_t* Data, uint32_t DataSize, uint8_t uART_no )
#endif
{
        uint8_t buffer[1024U];
        uint32_t payloadLength = 1U + DataSize;
        uint16_t flags = (payloadLength << 6U) | (Write & 0x1U);
        uint16_t crc;
        
        buffer[0] = PACKET_START_BYTE;                                          // Start byte.
        buffer[1] = ((uint8_t*)&flags)[0U];                                     // Flags low.
        buffer[2] = ((uint8_t*)&flags)[1U];                                     // Flags high.
        buffer[3] = CommandId;                                                  // Payload: Command ID.

        memcpy(buffer + 4U, Data, DataSize);                                    // Payload: Data.
        crc = lwnxCreateCrc(buffer, 4U + DataSize);
        buffer[4U + DataSize] = ((uint8_t*)&crc)[0U];                           // Checksum low.
        buffer[5U + DataSize] = ((uint8_t*)&crc)[1U];                           // Checksum high.
#ifdef lwserial
       Serial->writeData(buffer, 6U + DataSize);
#else
       memset((void *) buffer[6U + DataSize],(void *) '\0',sizeof(char));       // Null terminate the Buffer for serial
       switch (uART_no)
       {
            case 1u:
            UART1_Write_Text(buffer);                                           // Send it to serial UART 1
            break;
            
            case 2u:
            UART2_Write_Text(buffer);                                           // Send it to serial UART 2
            break;
            
            case 3u:
            UART3_Write_Text(buffer);                                           // Send it to serial UART 3
            break;

            case 4u:
            UART4_Write_Text(buffer);                                           // Send it to serial UART 4
            break;
            
            case 5u:
            UART5_Write_Text(buffer);                                           // Send it to serial UART 5
            break;

            case 6u:
            UART6_Write_Text(buffer);                                           // Send it to serial UART 6
            break;
            
            default:                                                            // invalid ignore
            break;
       }
#endif

}

/*-----------------------------------------------------------------------------
 *      lwnxHandleManagedCmd :  Sends packets of lwnx
 *  Parameters: uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize, uint8_t WriteBool, uint8_t* WriteData, uint32_t WriteSize
 *  Return:
 *         nothing
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxHandleManagedCmd(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize, bool Write, uint8_t* WriteData, uint32_t WriteSize)
#else
uint8_t lwnxHandleManagedCmd(uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize, uint8_t WriteBool, uint8_t* WriteData, uint32_t WriteSize, uint8_t uART_no)
#endif
{

        int32_t attempts = PACKET_RETRIES;
        lwResponsePacket response1;
        
        lwnxInitResponsePacket(&response1);
        #ifdef lwserial
        while (attempts--) {
           lwnxSendPacketBytes(Serial, CommandId, Write, WriteData, WriteSize);     // write from linux or windows
        #else
        while (attempts--) {                                                    // send it
           lwnxSendPacketBytes(CommandId, WriteBool, WriteData, WriteSize, uART_no);  // write from the UART which has been defined as the port
        #endif

        #ifdef lwserial
            if (lwnxRecvPacket(Serial, CommandId, &response1, PACKET_TIMEOUT))   // wait for response
            {
               memcpy(Response, response1.data + 4u, ResponseSize);
        #else
            if (lwnxRecvPacket(CommandId, &response1, PACKET_TIMEOUT, uART_no))   // wait for response
            //if (lwnxRecvPacket(CommandId, &response, 2000U, uART_no))          // lwnxRecvPacket(uint8_t CommandId, lwResponsePacket* Response, uint32_t TimeoutTicks, uint8_t uART_no)
            {
                memcpy(Response, response1.dataVal + 4u, ResponseSize);
        #endif
                Response[ResponseSize] = '\0';
                return 1U;
            }
        }
        return 0U;
}


/*-----------------------------------------------------------------------------
 *      lwnxCmdReadInt8 :  Reads int8_t
 *  Parameters: uint8_t uint8_t CommandId, int8_t* Response, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdReadInt8(lwSerialPort* Serial, uint8_t CommandId, int8_t* Response)
#else
uint8_t lwnxCmdReadInt8(uint8_t CommandId, int8_t* Response, uint8_t uART_no )
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 1U,false, NULL, 0U);
#else
   return lwnxHandleManagedCmd(CommandId, (uint8_t*)Response, 1U, false, NULL, 0U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdReadInt16 :  Reads int16_t
 *  Parameters: uint8_t CommandId, int16_t* Response, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdReadInt16(lwSerialPort* Serial, uint8_t CommandId, int16_t* Response)
#else
uint8_t lwnxCmdReadInt16(uint8_t CommandId, int16_t* Response, uint8_t uART_no)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 2U, false, NULL, 0U);
#else
   return lwnxHandleManagedCmd(CommandId, (uint8_t*)Response, 2U, false, NULL, 0U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdReadInt32 :  Reads int32_t
 *  Parameters: uint8_t uint8_t CommandId, int32_t* Response, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdReadInt32(lwSerialPort* Serial, uint8_t CommandId, int32_t* Response)
#else
uint8_t lwnxCmdReadInt32(uint8_t CommandId, int32_t* Response, uint8_t uART_no)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 4U, false, NULL, 0U);
#else
   return lwnxHandleManagedCmd(CommandId, (uint8_t*)Response, 4U, false, NULL, 0U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdReadUInt8 :  Reads uint8_t
 *  Parameters: uint8_t CommandId, uint8_t* Response, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdReadUInt8(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response)
#else
uint8_t lwnxCmdReadUInt8(uint8_t CommandId, uint8_t* Response, uint8_t uART_no)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 1U, false, NULL, 0U);
#else
   return lwnxHandleManagedCmd(CommandId, (uint8_t*)Response, 1U, false, NULL, 0U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdReadUInt16 :  Reads uint16_t
 *  Parameters: uint8_t CommandId, uint16_t* Response, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdReadUInt16(lwSerialPort* Serial, uint8_t CommandId, uint16_t* Response)
#else
uint8_t lwnxCmdReadUInt16(uint8_t CommandId, uint16_t* Response, uint8_t uART_no)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 2U, false, NULL, 0U);
#else
   return lwnxHandleManagedCmd(CommandId, (uint8_t*)Response, 2U, false, NULL, 0U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdReadUInt32 :  Reads uint32_t
 *  Parameters: uint8_t CommandId, uint32_t* Response, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdReadUInt32(lwSerialPort* Serial, uint8_t CommandId, uint32_t* Response)
#else
uint8_t lwnxCmdReadUInt32(uint8_t CommandId, uint32_t* Response, uint8_t uART_no)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 4U, false, NULL, 0U);
#else
   return lwnxHandleManagedCmd(CommandId, (uint8_t*)Response, 4U, false, NULL, 0U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdReadString :  Reads string
 *  Parameters: uint8_t CommandId, char* Response, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdReadString(lwSerialPort* Serial, uint8_t CommandId, char* Response)
#else
uint8_t lwnxCmdReadString(uint8_t CommandId, char* Response, uint8_t uART_no)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 16U, false, NULL, 0U);
#else
   return lwnxHandleManagedCmd(CommandId, (uint8_t*)Response, 16U, false, NULL, 0U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdReadData :  Reads data
 *  Parameters: uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdReadData(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize)
#else
uint8_t lwnxCmdReadData(uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize, uint8_t uART_no)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, Response, ResponseSize, false, NULL, 0U);
#else
   return lwnxHandleManagedCmd(CommandId, Response, ResponseSize, false, NULL, 0U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdWriteInt8 :  Write int8_t
 *  Parameters: uint8_t CommandId, int8_t Value, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdWriteInt8(lwSerialPort* Serial, uint8_t CommandId, int8_t Value)
#else
uint8_t lwnxCmdWriteInt8(uint8_t CommandId, int8_t Value, uint8_t uART_no)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0U, true, (uint8_t*)&Value, 1U);
#else
   return lwnxHandleManagedCmd(CommandId, NULL, 0U, true, (uint8_t*)&Value, 1U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdWriteInt16 :  Write int16_t
 *  Parameters: uint8_t CommandId, int16_t Value, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdWriteInt16(lwSerialPort* Serial, uint8_t CommandId, int16_t Value)
#else
uint8_t lwnxCmdWriteInt16(uint8_t CommandId, int16_t Value, uint8_t uART_no)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0U, true, (uint8_t*)&Value, 2U);
#else
   return lwnxHandleManagedCmd(CommandId, NULL, 0U, true, (uint8_t*)&Value, 2U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdWriteInt32 :  Write int32_t
 *  Parameters: uint8_t CommandId, int32_t Value, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifndef lwserial
uint8_t lwnxCmdWriteInt32(uint8_t CommandId, int32_t Value, uint8_t uART_no)
#else
uint8_t lwnxCmdWriteInt32(lwSerialPort* Serial, uint8_t CommandId, int32_t Value)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0U, true, (uint8_t*)&Value, 4U);
#else
   return lwnxHandleManagedCmd(CommandId, NULL, 0U, true, (uint8_t*)&Value, 4U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdWriteUInt8 :  Write uint8_t
 *  Parameters: uint8_t CommandId, uint8_t Value, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifndef lwserial
uint8_t lwnxCmdWriteUInt8(uint8_t CommandId, uint8_t Value, uint8_t uART_no)
#else
uint8_t lwnxCmdWriteUInt8(lwSerialPort* Serial, uint8_t CommandId, uint8_t Value)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0U, true, (uint8_t*)&Value, 1U);
#else
   return lwnxHandleManagedCmd(CommandId, NULL, 0U, true, (uint8_t*)&Value, 1U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdWriteUInt16 :  Write uint16_t
 *  Parameters: uint8_t CommandId, uint16_t Value, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifndef lwserial
uint8_t lwnxCmdWriteUInt16(uint8_t CommandId, uint16_t Value, uint8_t uART_no)
#else
uint8_t lwnxCmdWriteUInt16(lwSerialPort* Serial, uint8_t CommandId, uint16_t Value)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0U, true, (uint8_t*)&Value, 2U);
#else
   return lwnxHandleManagedCmd(CommandId, NULL, 0U, true, (uint8_t*)&Value, 2U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdWriteUInt32 :  Write uint32_t
 *  Parameters: uint8_t CommandId, uint32_t Value, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdWriteUInt32(lwSerialPort* Serial, uint8_t CommandId, uint32_t Value)
#else
uint8_t lwnxCmdWriteUInt32(uint8_t CommandId, uint32_t Value, uint8_t uART_no)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0U, true, (uint8_t*)&Value, 4U);
#else
   return lwnxHandleManagedCmd(CommandId, NULL, 0U, true, (uint8_t*)&Value, 4U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdWriteString :  Write string
 *  Parameters: lwSerialPort* Serial, uint8_t CommandId, char* String
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifdef lwserial
uint8_t lwnxCmdWriteString(lwSerialPort* Serial, uint8_t CommandId, char* String)
#else
uint8_t lwnxCmdWriteString(uint8_t CommandId, char* String, uint8_t uART_no)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0U, true, (uint8_t*)String, 16U);
#else
   return lwnxHandleManagedCmd(CommandId, NULL, 0U, true, (uint8_t*)String, 16U, uART_no);
#endif
}

/*-----------------------------------------------------------------------------
 *      lwnxCmdWriteData :  Write data
 *  Parameters: uint8_t CommandId, uint8_t* Data, uint32_t DataSize, uint8_t uART_no
 *  Return:
 *         uint8_t
 *----------------------------------------------------------------------------*/
#ifndef lwserial
uint8_t lwnxCmdWriteData(uint8_t CommandId, uint8_t* Data, uint32_t DataSize, uint8_t uART_no)
#else
uint8_t lwnxCmdWriteData(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Data, uint32_t DataSize)
#endif
{
#ifdef lwserial
   return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0U, true, Data, DataSize);
#else
   return lwnxHandleManagedCmd(CommandId, NULL, 0U, true, Data, DataSize, uART_no);
#endif
}