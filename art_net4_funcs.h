//  art_net4_funcs.h - ILDA file "native" operations.
//
// * Written by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#ifndef ARTNET4_FUNCS
#define ARTNET4_FUNCS
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

uint16_t rdmCalculateChecksum(uint8_t* dataVal, uint8_t length);
void dmxPackDataWithout_sc(uint8_t *dataStrm, uint16_t length, artnet_dmx_t *artDmxPkt);
void composeDmX( uint8_t bKey, artnet_dmx_t *artDmxPkt, uint8_t u8DMX512Role, uint8_t u8DMX512DimValue );

//  void dmxSendDataWithout_sc(uint8_t *data, uint16_t length, artnet_dmx_t *artDmxPkt);
 /*-----------------------------------------------------------------------------
 *      rdmCalculateChecksum():  calculate the checksum for rdm
 *
 *  Parameters: uint8_t a, uint8_t b
 *
 *  Return:     uint16_t : calculated checksum
 *----------------------------------------------------------------------------*/
uint16_t rdmCalculateChecksum(uint8_t* dataVal, uint8_t length)
{
    uint16_t checksum = 0u;
    int16_t i;

    if (dataVal == NULL)                                                        // null stream return
      return 0u;
      
    for (i = 0u; i < length; ++i)                                               // calculate checksum
    {
        checksum += *dataVal;
        ++dataVal;
    }
    return checksum;
}

#define DMX512_START_CODE 0U
#define DMX512_ROLE_NONE 0U
#define DMX512_ROLE_MASTER 1U
#define DMX512_ROLE_SLAVE 2U
/*******************************************************************************
* Function Name: dmxPackDataWithout_sc
********************************************************************************
* Summary:
*   compose DMX message using buffer without start code
* Parameters:
*  uint8_t *dataStrm, uint16_t length, artnet_dmx_t *artDmxPkt
*
* Return:
*  None.
*
*******************************************************************************/
void dmxPackDataWithout_sc(uint8_t *dataStrm, uint16_t length, artnet_dmx_t *artDmxPkt)
{
        artDmxPkt->dataV[0] = DMX512_START_CODE;
        memcpy((void*)artDmxPkt->dataV[1],(void*) dataStrm, length);
        artDmxPkt->dataV[length] = '\0';
}
#define SEG_CHAR_A  0X10U                                                       // Segment definitions
#define SEG_CHAR_B  0X11U
#define SEG_CHAR_C  0X12U
#define SEG_CHAR_D  0X13U
#define SEG_CHAR_E  0X14U
#define SEG_CHAR_F  0X15U
#define SEG_CHAR_G  0X16U
#define SEG_CHAR_H  0X17U
#define SEG_CHAR_L  0X18U
#define SEG_CHAR_M  0X19U
#define SEG_CHAR_N  0X1AU
#define SEG_CHAR_O  0X1BU
#define SEG_CHAR_P  0X1CU
#define SEG_CHAR_Q  0X1DU
#define SEG_CHAR_R  0X1EU
#define SEG_CHAR_S  0X1FU
#define SEG_CHAR_T  0X20U
#define SEG_CHAR_U  0X21U
#define SEG_CHAR_V  0X22U
#define SEG_CHAR_W  0X23U
#define SEG_CHAR_X  0X24U
#define SEG_CHAR_Y  0X25U
#define SEG_CHAR_Z  0X26U
#define SEG_CHAR_NULL 0XFFU
#define DMX_KEY_MASTER 1U
#define DMX_KEY_CHANNEL_SELECT 2U
#define DMX_KEY_UP 3U
#define DMX_KEY_DOWN 4U
#define DMX_KEY_LEFT 5U
#define DMX_KEY_RIGHT 6U
#define DMX512_SEGMENT_DISPLAY_CHANNEL_BASE  0//100

/*******************************************************************************
* Function Name: composeDmX
********************************************************************************
* Summary:
*   compose DMX message
* Parameters:
*  uint8_t bKey,artnet_dmx_t *artDmxPkt. uint8_t u8DMX512Role uint8_t u8DMX512DimValue
*
* Return:
*  None.
*
*******************************************************************************/
void composeDmX( uint8_t bKey, artnet_dmx_t *artDmxPkt, uint8_t u8DMX512Role, uint8_t u8DMX512DimValue, uint8_t chan )
{
   uint8_t u8DMX512TransChannel,u8DMX512RecChannel,u87SegRam[2u],Temp;          //  if you are sending via UART use this we are using artnet  u8DMX512TransData[DMX512_MAX_ARRAY];
                                                                                //  like wise change artnet_dmx_t *artDmxPkt with ASN_DMPLayer_t for transport using ASN
   u8DMX512TransChannel = chan;
   u8DMX512RecChannel = chan;

   // not sure yet if i need to transmit SeqRam ??
   switch(bKey)
    {
        case DMX_KEY_MASTER:
            if(u8DMX512Role != DMX512_ROLE_MASTER)
            {
                u8DMX512Role = DMX512_ROLE_MASTER;
                u8DMX512TransChannel = 1u;
                u87SegRam[0u] = SEG_CHAR_H;
                u87SegRam[1u] = u8DMX512TransChannel;
                artDmxPkt->dataV[0u] = DMX512_START_CODE;
                for(Temp = 1; Temp < (DMX512_MAX_ARRAY + 1u); Temp++)
                {
                    artDmxPkt->dataV[Temp] = Temp;
                }
//                DMX512_SendInit();
//                DMX512_SendData();
                // u8DMX512SendTriggerEnable = 1;
                //configure UART as transmitter
            }
            else
            {
                u8DMX512Role = DMX512_ROLE_SLAVE;
                u87SegRam[0u] = SEG_CHAR_S;
                u87SegRam[1u] = SEG_CHAR_NULL;
//                u8DMX512SendTriggerEnable = 0;
                u8DMX512RecChannel = 1u;
                //configure UART as receiver
//                DMX512_ReceiveInit();
            }
            break;

        case DMX_KEY_CHANNEL_SELECT:
            if(u8DMX512Role == DMX512_ROLE_MASTER)
            {
                u8DMX512TransChannel++;
                if(u8DMX512TransChannel > 9u)
                {
                    u8DMX512TransChannel = 1u;
                }
                u87SegRam[0u] = SEG_CHAR_H;
                u87SegRam[1u] = u8DMX512TransChannel;
                // LED_On(u8DMX512TransData[u8DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]);
            }
            else if(u8DMX512Role == DMX512_ROLE_SLAVE)
            {
                u8DMX512RecChannel++;
                if(u8DMX512RecChannel > 9u)
                {
                    u8DMX512RecChannel = 1u;
                }
                u87SegRam[0u] = SEG_CHAR_S;
                u87SegRam[1u] = u8DMX512RecChannel;
            }
            break;

        case DMX_KEY_UP:                                                        // select the channel upwards
            if(u8DMX512Role == DMX512_ROLE_MASTER)
            {
                u87SegRam[0u] = SEG_CHAR_H;
                u87SegRam[1u] = u8DMX512TransChannel;
                //u8DMX512TransData[u8DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]++;
                artDmxPkt->dataV[u8DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]++;
                //LED_On(u8DMX512TransData[u8DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]);
            }
            break;

        case DMX_KEY_DOWN:                                                      // select the channel downwards
            if(u8DMX512Role == DMX512_ROLE_MASTER)
            {
                u87SegRam[0u] = SEG_CHAR_H;
                u87SegRam[1u] = u8DMX512TransChannel;
                //u8DMX512TransData[u8DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]--;
                artDmxPkt->dataV[u8DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]--;
                //LED_On(u8DMX512TransData[u8DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]);
            }
            break;

        case DMX_KEY_LEFT:                                                      // intensity down
           u8DMX512DimValue=u8DMX512DimValue - 1U;
           artDmxPkt->dataV[0u] = DMX512_START_CODE;
           artDmxPkt->dataV[1u] = u8DMX512TransChannel;
           artDmxPkt->dataV[2u] = u8DMX512DimValue;
           break;

        case DMX_KEY_RIGHT:                                                     // intensity up
           u8DMX512DimValue=++u8DMX512DimValue % UINT8_MAX;
           artDmxPkt->dataV[0u] = DMX512_START_CODE;
           artDmxPkt->dataV[1u] = u8DMX512TransChannel;
           artDmxPkt->dataV[2u] = u8DMX512DimValue;
           break;

        default:
            break;
    }
}

#ifdef __cplusplus
}
#endif

#endif