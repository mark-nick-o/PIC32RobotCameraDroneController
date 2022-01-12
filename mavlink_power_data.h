#ifndef __cmd_power_data_mav__
#define __cmd_power_data_mav__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "mavlink_packet_parser.h"                                              /* general function for parsing the packet of data recieved */
#include "canBus_defs.h"
#include "linear_float_conversions.h"
#if (MAV_VER == 1)                                                              // ==== MAVLINK ver 1.0 =============
#include "msg1_power_board.h"
#include "msg1_sys_status.h"
#else
#include "msg2_sys_status.h"
#endif
#include "gc_events.h"

/* 
   convert power data read from flight controler over MAVLINK to linear16 data
   that can be sent over AUVCAN to a PMBUS power management chip
*/
#define FROM16CAN1LE(a, p) \
   ((uint8_t *)(p))[0u] = ((uint16_t)(a) >> 0u) & 0xFFU, \
   ((uint8_t *)(p))[1u] = ((uint16_t)(a) >> 8u) & 0xFFU                         /* unaligned 16-bit integer (little-endian encoding) */
#define FROM16CAN2LE(a, p) \
   ((uint8_t *)(p))[2u] = ((uint16_t)(a) >> 0u) & 0xFFU, \
   ((uint8_t *)(p))[3u] = ((uint16_t)(a) >> 8u) & 0xFFU
#define FROM16CAN3LE(a, p) \
   ((uint8_t *)(p))[4u] = ((uint16_t)(a) >> 0u) & 0xFFU, \
   ((uint8_t *)(p))[5u] = ((uint16_t)(a) >> 8u) & 0xFFU
   
#define FROM32CAN1LE(a, p) \
   ((uint8_t *)(p))[0u] = ((uint32_t)(a) >> 0u) & 0xFFU, \
   ((uint8_t *)(p))[1u] = ((uint32_t)(a) >> 8u) & 0xFFU, \
   ((uint8_t *)(p))[2u] = ((uint32_t)(a) >> 16u) & 0xFFU, \
   ((uint8_t *)(p))[3u] = ((uint32_t)(a) >> 24u) & 0xFFU                        /* unaligned 32-bit integer (little-endian encoding) */

#define FROM16CAN1BE(a, p) \
   ((uint8_t *)(p))[0u] = ((uint16_t)(a) >> 8u) & 0xFFU, \
   ((uint8_t *)(p))[1u] = ((uint16_t)(a) >> 0u) & 0xFFU                         /* unaligned 16-bit integer (big-endian encoding) */
#define FROM16CAN2BE(a, p) \
   ((uint8_t *)(p))[2u] = ((uint16_t)(a) >> 8u) & 0xFFU, \
   ((uint8_t *)(p))[3u] = ((uint16_t)(a) >> 0u) & 0xFFU
#define FROM16CAN3BE(a, p) \
   ((uint8_t *)(p))[4u] = ((uint16_t)(a) >> 8u) & 0xFFU, \
   ((uint8_t *)(p))[5u] = ((uint16_t)(a) >> 0u) & 0xFFU
#define FROM32CAN1BE(a, p) \
   ((uint8_t *)(p))[0u] = ((uint32_t)(a) >> 24u) & 0xFFU, \
   ((uint8_t *)(p))[1u] = ((uint32_t)(a) >> 16u) & 0xFFU, \
   ((uint8_t *)(p))[2u] = ((uint32_t)(a) >> 8u) & 0xFFU, \
   ((uint8_t *)(p))[3u] = ((uint32_t)(a) >> 0u) & 0xFFU                         /* unaligned 32-bit integer (big-endian encoding) */

#define BATTERY_CAPACITY 2500.0f                                                /* define here the battery capacity */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define POWERPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define POWERPACKED __attribute__((packed))                                 /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define POWERPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define POWERPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define POWERPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#if defined(D_FT900)
typedef struct POWERPACKED {
   float32_t prev_vbat_raw;
   float32_t vbat_raw;
   float32_t prev_vbat;
   uint8_t only_once : 1u;
   uint8_t spareflags : 7u;
} Batt_filter_t;
#else
POWERPACKED(
typedef struct {
   float32_t prev_vbat_raw;
   float32_t vbat_raw;
   float32_t prev_vbat;
   uint8_t only_once : 1u;
   uint8_t spareflags : 7u;
}) Batt_filter_t;
#endif

#if defined(D_FT900)
typedef struct POWERPACKED {
    float32_t CurrentTotal;
    float32_t BatteryLevel;
    float32_t mvInput;
    uint8_t CellsCount;
    int64_t dt;
    uint64_t secsRef;
    uint8_t CurrentSensor : 1u;
    uint8_t isInitialised : 1u;
    uint8_t batLow : 1u;
    uint8_t batLeftInit : 1u;
    uint8_t spare : 4u;
    float32_t sumBatVolt;
    uint8_t sumCount;
    float32_t calcVoltLeft;
} power_t;
#else
POWERPACKED(
typedef struct {
    float32_t CurrentTotal;
    float32_t BatteryLevel;
    float32_t mvInput;
    uint8_t CellsCount;
    int64_t dt;
    uint64_t secsRef;
    uint8_t CurrentSensor : 1u;
    uint8_t isInitialised : 1u;
    uint8_t batLow : 1u;
    uint8_t batLeftInit : 1u;
    uint8_t spare : 4u;
    float32_t sumBatVolt;
    uint8_t sumCount;
    float32_t calcVoltLeft;
}) power_t;
#endif

void convPowerBoardVoltsforPMBusLE( const mavlink_sens_power_board_t *mcuPBData, canBusSingleFrame_t *canPMBusController );
void convPowerBoardAmpsforPMBusLE( const mavlink_sens_power_board_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1, canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameEnd_t *canPMBusFrame3 );
void convPowerBoardVoltsforPMBusBE( const mavlink_sens_power_board_t *mcuPBData, canBusSingleFrame_t *canPMBusController );
void convPowerBoardAmpsforPMBusBE( const mavlink_sens_power_board_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1, canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameEnd_t *canPMBusFrame3 );
void convBatteryPackforPMBusLE( const mavlink_sens_batmon_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1, canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameBody_t *canPMBusFrame3, canBusMultiFrameBody_t *canPMBusFrame4, canBusMultiFrameEnd_t *canPMBusFrame4 );
void convBatteryPackforPMBusBE( const mavlink_sens_batmon_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1, canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameBody_t *canPMBusFrame3, canBusMultiFrameBody_t *canPMBusFrame4, canBusMultiFrameEnd_t *canPMBusFrame5 );
int8_t getBatteryDataFromMAV( const mavlink_message_t* mavMsg, mavlink_sens_batmon_t *mcuPBData );
int8_t getPowerBoardDataFromMAV( const mavlink_message_t* mavMsg, mavlink_sens_power_board_t *mcuPBData );
int8_t convPMBusforMAVPowerBoardVoltsLE( mavlink_sens_power_board_t *mcuPBData, const canBusSingleFrame_t *canPMBusController );
int8_t convPMBusforMAVPowerBoardVoltsBE( mavlink_sens_power_board_t *mcuPBData, const canBusSingleFrame_t *canPMBusController );
int8_t convPMBusforMAVPowerBoardAmpsLE( mavlink_sens_power_board_t *mcuPBData, const canBusMultiFrameStart_t *canPMBusFrame1, const canBusMultiFrameBody_t *canPMBusFrame2, const canBusMultiFrameEnd_t *canPMBusFrame3 );
int8_t convPMBusforMAVPowerBoardAmpsBE( mavlink_sens_power_board_t *mcuPBData, const canBusMultiFrameStart_t *canPMBusFrame1, const canBusMultiFrameBody_t *canPMBusFrame2, const canBusMultiFrameEnd_t *canPMBusFrame3 );
int8_t convPMBusforMAVBatteryPackLE( mavlink_sens_batmon_t *mcuPBData,const canBusMultiFrameStart_t *canPMBusFrame1,const canBusMultiFrameBody_t *canPMBusFrame2,const canBusMultiFrameBody_t *canPMBusFrame3,const canBusMultiFrameBody_t *canPMBusFrame4,const canBusMultiFrameEnd_t *canPMBusFrame5 );
int8_t convPMBusforMAVBatteryPackBE( mavlink_sens_batmon_t *mcuPBData,const canBusMultiFrameStart_t *canPMBusFrame1,const canBusMultiFrameBody_t *canPMBusFrame2,const canBusMultiFrameBody_t *canPMBusFrame3,const canBusMultiFrameBody_t *canPMBusFrame4,const canBusMultiFrameEnd_t *canPMBusFrame5 );
void SendPowerBoardMAVLinkMessage( mavlink_sens_power_board_t *mcuPBData, uint8_t target_sys, uint8_t target_comp );
void SendBatteryMAVLinkMessage( mavlink_sens_batmon_t *mcuPBData, uint8_t target_sys, uint8_t target_comp );
float32_t AP_BattMonitor_Bebop_Comp(const uint16_t *rpm, float32_t vbat_raw);
float32_t AP_BattMonitor_Bebop_Percent(float32_t vbat);
float32_t AP_BattMonitor_Bebop_filter_volts(Batt_filter_t *batFilt);
void pack_pwr_info_to_mavlnk( uint8_t target_sys, uint8_t target_comp, power_t *battery, uint8_t measBatVolt, uint8_t measBatCurnt, const uint16_t *rpm, mavlink_sys_status_t *mavSysSta );
void sendSysStatusMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_sys_status_t *mavSysSta );
void convL16CypressMv2RawVolts( uint16_t linear16Val, Batt_filter_t *battFilt );
void filterVoltsInit( Batt_filter_t *battFilt );
void calcPowerVals( power_t *pow, float32_t current, float32_t mVBat, uint8_t mCellsCount  );
void remainBattVoltage( power_t *battery );

/*-----------------------------------------------------------------------------
 *      AP_BattMonitor_Bebop_comp():  compensate battery for flying craft
 *                                    Brushless DC Motor compensation
 *                                    (file is for bebop)
 *
 *  Parameters: const uint16_t *rpm, float32_t vbat_raw
 *
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
#define BLDC_MOTORS_NUM 10u                                                     /* number of brushless DC motors */
float32_t AP_BattMonitor_Bebop_Comp(const uint16_t *rpm, float32_t vbat_raw)
{
    static const float32_t bat_comp_polynomial_coeffs[5u] = {
    -1.2471059149657287e-16f,
    3.2072883440944087e-12f,
    -3.3012241016211356e-08f,
    1.4612693130825659e-04f,
    -1.9236755589522961e-01f
    };

    float32_t vbat, res;
    size_t i, j;

    vbat = vbat_raw;
    for (i = 0; i < BLDC_MOTORS_NUM; i++)
    {
        res = 0;
        for (j = 0; j < sizeof(bat_comp_polynomial_coeffs); j++)
            res = res * rpm[i] + bat_comp_polynomial_coeffs[j];
        vbat -= res;
    }
    return vbat;
}
/*-----------------------------------------------------------------------------
 *      AP_BattMonitor_Bebop_Percent():  Calculate percent from lookup table
 *                                       table to match battery
 *
 *  Parameters: float32_t vbat
 *
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t AP_BattMonitor_Bebop_Percent(float32_t vbat)
{
    float32_t percent = 0.0f;
    int16_t i;

    /* battery percent lookup table */
    static const struct {
    float32_t voltage;
    float32_t percent;
    } bat_lut[] = {
    #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
    {9.5f, 0.0f},
    {11.04f, 5.0f},
    {11.11f, 10.0f},
    {11.21f, 15.0f},
    {11.3f, 25.0f},
    {11.4f, 45.0f},
    {11.6f, 55.0f},
    {11.9f, 79.0f},
    {12.02f, 84.0f},
    {12.11f, 88.0f},
    {12.19f, 91.0f},
    {12.26f, 94.0f},
    {12.35f, 96.0f},
    {12.45f, 98.0f},
    {12.5f, 100.0f}
    #else                                                                       // bebop
    {10.50f, 0.0f},
    {10.741699f, 2.6063901f},
    {10.835779f, 5.1693798f},
    {10.867705f, 7.7323696f},
    {10.900651f, 10.295359f},
    {11.008754f, 20.547318f},
    {11.148267f, 38.488246f},
    {11.322504f, 53.866185f},
    {11.505738f, 66.681133f},
    {11.746556f, 79.496082f},
    {12.110226f, 94.874021f},
    {12.3f, 100.0f }
    #endif
    };
    size_t sizeLookup = sizeof(bat_lut) - 1u;
    
    if (vbat <= bat_lut[0u].voltage)
    {
        percent = 0.0f;
    } 
    else if (vbat >= bat_lut[sizeLookup].voltage)
    {
        percent = 100.0f;
    } 
    else 
    {
        i = 0;
        while (vbat >= bat_lut[i].voltage)
            i++;
        percent += bat_lut[i - 1].percent + (vbat - bat_lut[i - 1].voltage) * (bat_lut[i].percent - bat_lut[i - 1].percent) / (bat_lut[i].voltage - bat_lut[i - 1].voltage);
    }

    return percent;
}
/*-----------------------------------------------------------------------------
 *      AP_BattMonitor_Bebop_filter_volts():  filter volts
 *
 *  Parameters: Batt_filter_t *batFilt
 *
 *  Return:     float32_t
 *----------------------------------------------------------------------------*/
float32_t AP_BattMonitor_Bebop_filter_volts(Batt_filter_t *batFilt)
{
    static const float32_t a[2u] = {
        1.0f, -9.9686333183343789e-01f
    };
    static const float32_t b[2u] = {
        1.5683340832810533e-03f, 1.5683340832810533e-03f
    };
    float32_t vbat;

    if (batFilt->only_once==0u)                                                    /* on first time reset filter with first raw value */
    {
        vbat = batFilt->vbat_raw;
        batFilt->prev_vbat_raw = batFilt->vbat_raw;
        batFilt->prev_vbat = batFilt->vbat_raw;
        batFilt->only_once = 1u;
    } 
    else  if (batFilt->vbat_raw > 0.0f)
    {
        vbat = b[0u] * batFilt->vbat_raw + b[1u] * batFilt->prev_vbat_raw - a[1u] * batFilt->prev_vbat;     /*  1st order filter */
        batFilt->prev_vbat_raw = batFilt->vbat_raw;
        batFilt->prev_vbat = vbat;
    } 
    else 
    {
        vbat = batFilt->prev_vbat;
    }
    return vbat;
}
/*-----------------------------------------------------------------------------
 *      calcPowerVals():  calculate the power statistics
 *
 *  Parameters: power_t *pow, float32_t current, float32_t mVBat,
 *
 * modified from code by :- Copyright (C) 2016 Adrien Aubry (drich) BCFlight
 *
 *  Return:     nil
 *----------------------------------------------------------------------------*/
void calcPowerVals( power_t *pow, float32_t current, float32_t mVBat )
{
   if ( pow != NULL )
   {
      if ((pow->isInitialised == false) && (pow->CurrentSensor == 1u))
      {
         pow->dt = -1;
         calculateTimeDiff( &pow->dt, &pow->secsRef );
         pow->isInitialised = true;
         pow->BatteryLevel = ( mVBat / (float32_t)pow->CellsCount - 3.7f ) / ( 4.2f - 3.7f );
      }
      else if ( pow->CurrentSensor == 0u )                                      /*  No current sensor  */
      {
         pow->BatteryLevel = ( mVBat / (float32_t)pow->CellsCount - 3.7f ) / ( 4.2f - 3.7f );
      }
      else
      {
         calculateTimeDiff( &pow->dt, &pow->secsRef );
         pow->CurrentTotal += current * pow->dt / 3600.0f;                      /* mAh for GCS */
         pow->BatteryLevel = 1.0f - ( pow->CurrentTotal * 1000.0f ) / BATTERY_CAPACITY;
      }
      pow->BatteryLevel = FMAX( 0.0f, FMIN( 1.0f, pow->BatteryLevel ) );
      pow->batLow = ( mVBat < 3.365f );
   }
}
/*-----------------------------------------------------------------------------
 *      remainBattVoltage():  Calculates remaining voltage on batteries.
 *
 *  Parameters: power_t *battery
 *
 * modified from code by :- Maxim Vovenko  Copyright (c) 2019 AVBotz
 *
 *  Return:     nil
 *----------------------------------------------------------------------------*/
void remainBattVoltage( power_t *battery )
{
    const uint8_t num_samples = 10u;
    float32_t cal_constant = 23.88349514563107f;

    if (battery == NULL)
    { /* for misra */ }
    else
    {
       if (battery->batLeftInit == false)                                       /* initialise */
       {
          battery->sumBatVolt = battery->mvInput;
          battery->sumCount = 1u;
          battery->batLeftInit = true;
       }
       else if (battery->sumCount >= num_samples)                               /* sampled enough do the calc */
       {
          battery->calcVoltLeft = battery->sumBatVolt/battery->sumCount*5.0f/1024.0f*cal_constant;
          battery->sumCount = 0u;
          battery->sumBatVolt = 0.0f;
       }
       else                                                                     /* keep taking a volatge sample */
       {
          battery->sumBatVolt = battery->sumBatVolt + battery->mvInput;
          battery->sumCount = ++battery->sumCount % UINT8_MAX;
       }
    }
}
/*-----------------------------------------------------------------------------
 *      sendSysStatusMavlink():  wrap and send system status message
 *                               from client to server
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, mavlink_sys_status_t *mavSysSta
 *       
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void sendSysStatusMavlink( uint8_t target_sys, uint8_t target_comp, mavlink_sys_status_t *mavSysSta )
{
  
#ifdef MAV_LIB_GENERATED

  uint8_t buf[MAVLINK_MSG_ID_SYS_STATUS_LEN ];                     /* buffer containing the message to send */
  uint16_t len;
  mavlink_message_t message;
  
  if (mavSysSta == NULL)                                                     /* if null was passed return */
    return;

  len = mavlink_msg_sys_status_encode(target_sys, target_comp, &message, mavSysSta);  /* encode */  

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_SYS_STATUS_LEN  );                    // Send message you write to your serial UART_Write
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_SYS_STATUS_LEN  );                       // Send message you write to your UDP Port
#endif
}

/*-----------------------------------------------------------------------------
 *  pack_pwr_info_to_mavlnk():  packs battery power information into a mavlink sys status
 *                                  object            
 *
 *  Parameters: uint8_t target_sys, uint8_t target_comp, power_t *battery, uint8_t measBatVolt,  
 *              uint8_t measBatCurnt, const uint16_t *rpm, mavlink_sys_status_t *mavSysSta
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
 void pack_pwr_info_to_mavlnk( uint8_t target_sys, uint8_t target_comp, power_t *battery, uint8_t measBatVolt, uint8_t measBatCurnt, const uint16_t *rpm, mavlink_sys_status_t *mavSysSta )
 {
         battery->mvInput = measBatVolt;
         mavSysSta->current_battery = measBatCurnt;        
         
         if  (battery->batLeftInit == true)
         {         
           remainBattVoltage( battery );
           mavSysSta->voltage_battery = AP_BattMonitor_Bebop_Comp(rpm, battery->calcVoltLeft);
         }
         else
         {
           remainBattVoltage( battery );
           mavSysSta->voltage_battery = AP_BattMonitor_Bebop_Comp(rpm, measBatVolt);
         }
         calcPowerVals( battery, measBatCurnt, mavSysSta->voltage_battery );
         mavSysSta->battery_remaining = (battery->BatteryLevel*100.0f); 
         sendSysStatusMavlink( target_sys, target_comp, mavSysSta );
 }
/*-----------------------------------------------------------------------------
 *      convPowerVoltsBoardforPMBusLE():  get power board data from mavlink message
 *                                       (little endian)
 *  Parameters: const mavlink_sens_power_board_t *mcuPBData, canBusSingleFrame_t *canPMBusController
 *
 *
 *  Return:     nil
 *----------------------------------------------------------------------------*/
void convPowerBoardVoltsforPMBusLE( const mavlink_sens_power_board_t *mcuPBData, canBusSingleFrame_t *canPMBusController )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */

     if ((mcuPBData == NULL) || (canPMBusController == NULL))
     {
        return;
     }
     
     canPMBusController->startTrans=1u;                                         /* set the tail bytes */
     canPMBusController->toggle=0u;
     canPMBusController->endTrans=1u;
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_system_volt);  /* convert to linear 16 from IEEE float */
     FROM16CAN1LE(linear16Val,canPMBusController->DataBytes);                   /* put the bytes in little endian order */
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_servo_volt);
     FROM16CAN2LE(linear16Val,canPMBusController->DataBytes);
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_digital_volt);
     FROM16CAN3LE(linear16Val,canPMBusController->DataBytes);
}
/*-----------------------------------------------------------------------------
 *      convPowerVoltsBoardforPMBusBE():  get power board data from mavlink message
 *                                       (big endian)
 *  Parameters: const mavlink_sens_power_board_t *mcuPBData, canBusSingleFrame_t *canPMBusController
 *
 *
 *  Return:     nil
 *----------------------------------------------------------------------------*/
void convPowerVoltsBoardforPMBusBE( const mavlink_sens_power_board_t *mcuPBData, canBusSingleFrame_t *canPMBusController )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */

     if ((mcuPBData == NULL) || (canPMBusController == NULL))
     {
        return;
     }
     
     canPMBusController->startTrans=1u;                                         /* set the tail bytes */
     canPMBusController->toggle=0u;
     canPMBusController->endTrans=1u;
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_system_volt);  /* convert to linear 16 from IEEE float */
     FROM16CAN1BE(linear16Val,canPMBusController->DataBytes);                   /* put the bytes in big endian order */
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_servo_volt);
     FROM16CAN2BE(linear16Val,canPMBusController->DataBytes);
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_digital_volt);
     FROM16CAN3BE(linear16Val,canPMBusController->DataBytes);
}
/*-----------------------------------------------------------------------------
 *      convPowerBoardAmpsforPMBusLE():  get power board data from mavlink message
 *                                       (little endian)
 *  Parameters: const mavlink_sens_power_board_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1,
 *  canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameBody_t *canPMBusFrame3,
 *
 *
 *  Return:     nil
 *----------------------------------------------------------------------------*/
void convPowerBoardAmpsforPMBusLE( const mavlink_sens_power_board_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1, canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameEnd_t *canPMBusFrame3 )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */

     if ((((mcuPBData == NULL) || (canPMBusFrame1 == NULL)) || (canPMBusFrame2 == NULL)) || (canPMBusFrame3 == NULL))
     {
         return;                                                                /* Null passed return without action */
     }
     
     canPMBusFrame1->startTrans=1u;                                             /* set the tail bytes */
     canPMBusFrame1->toggle=0u;
     canPMBusFrame1->endTrans=0u;
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_mot_l_amp);/* convert to linear 16 from IEEE float */
     FROM16CAN1LE(linear16Val,canPMBusFrame1->DataBytes);                       /* put the bytes in little endian order */
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_mot_r_amp);
     FROM16CAN2LE(linear16Val,canPMBusFrame1->DataBytes);
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_analog_amp);
     FROM16CAN3LE(linear16Val,canPMBusFrame1->DataBytes);

     canPMBusFrame2->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame2->toggle=1u;
     canPMBusFrame2->endTrans=0u;
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_digital_amp);/* convert to linear 16 from IEEE float */
     FROM16CAN1LE(linear16Val,canPMBusFrame2->DataBytes);                       /* put the bytes in little endian order */
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_ext_amp);
     FROM16CAN2LE(linear16Val,canPMBusFrame2->DataBytes);
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_aux_amp);
     FROM16CAN3LE(linear16Val,canPMBusFrame2->DataBytes);

     canPMBusFrame3->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame3->toggle=0u;
     canPMBusFrame3->endTrans=1u;
     canPMBusFrame3->DataBytes[1u] = mcuPBData->pwr_brd_led_status;
     canPMBusFrame3->DataBytes[0u] = mcuPBData->pwr_brd_status;
}
/*-----------------------------------------------------------------------------
 *      convPowerBoardAmpsforPMBusBE():  get power board data from mavlink message
 *                                       (big endian)
 *  Parameters: const mavlink_sens_power_board_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1,
 *  canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameBody_t *canPMBusFrame3,
 *
 *
 *  Return:     nil
 *----------------------------------------------------------------------------*/
void convPowerBoardAmpsforPMBusBE( const mavlink_sens_power_board_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1, canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameEnd_t *canPMBusFrame3 )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */

     if ((((mcuPBData == NULL) || (canPMBusFrame1 == NULL)) || (canPMBusFrame2 == NULL)) || (canPMBusFrame3 == NULL))
     {
         return;                                                                /* Null passed return without action */
     }
     
     canPMBusFrame1->startTrans=1u;                                             /* set the tail bytes */
     canPMBusFrame1->toggle=0u;
     canPMBusFrame1->endTrans=0u;
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_mot_l_amp); /* convert to linear 16 from IEEE float */
     FROM16CAN1BE(linear16Val,canPMBusFrame1->DataBytes);                       /* put the bytes in big endian order */
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_mot_r_amp);
     FROM16CAN2BE(linear16Val,canPMBusFrame1->DataBytes);
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_analog_amp);
     FROM16CAN3BE(linear16Val,canPMBusFrame1->DataBytes);
     
     canPMBusFrame2->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame2->toggle=1u;
     canPMBusFrame2->endTrans=0u;
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_digital_amp); /* convert to linear 16 from IEEE float */
     FROM16CAN1BE(linear16Val,canPMBusFrame2->DataBytes);                       /* put the bytes in big endian order */
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_ext_amp);
     FROM16CAN2BE(linear16Val,canPMBusFrame2->DataBytes);
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->pwr_brd_aux_amp);
     FROM16CAN3BE(linear16Val,canPMBusFrame2->DataBytes);
     
     canPMBusFrame3->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame3->toggle=0u;
     canPMBusFrame3->endTrans=1u;
     canPMBusFrame3->DataBytes[0u] = mcuPBData->pwr_brd_led_status;
     canPMBusFrame3->DataBytes[1u] = mcuPBData->pwr_brd_status;
     
}
/*-----------------------------------------------------------------------------
 *      convBatteryPackforPMBusLE():  get power board data from mavlink message
 *                                       (little endian)
 *  Parameters: const mavlink_sens_batmon_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1, 
 *  canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameBody_t *canPMBusFrame3, 
 *  canBusMultiFrameBody_t *canPMBusFrame4, canBusMultiFrameEnd_t *canPMBusFrame5
 *
 *
 *  Return:     nil
 *----------------------------------------------------------------------------*/
void convBatteryPackforPMBusLE( const mavlink_sens_batmon_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1, canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameBody_t *canPMBusFrame3, canBusMultiFrameBody_t *canPMBusFrame4, canBusMultiFrameEnd_t *canPMBusFrame5 )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */

     if ((((((mcuPBData == NULL) || (canPMBusFrame1 == NULL)) || (canPMBusFrame2 == NULL)) || (canPMBusFrame3 == NULL)) || (canPMBusFrame4 == NULL)) || (canPMBusFrame5 == NULL))
     {
         return;                                                                /* Null passed return without action */
     }
     
     canPMBusFrame1->startTrans=1u;                                             /* set the tail bytes */
     canPMBusFrame1->toggle=0u;
     canPMBusFrame1->endTrans=0u;
     FROM16CAN1LE(mcuPBData->cellvoltage1,canPMBusFrame1->DataBytes);           /* put the bytes in little endian order */
     FROM16CAN2LE(mcuPBData->cellvoltage2,canPMBusFrame1->DataBytes);
     FROM16CAN3LE(mcuPBData->cellvoltage3,canPMBusFrame1->DataBytes);

     canPMBusFrame2->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame2->toggle=1u;
     canPMBusFrame2->endTrans=0u;
     FROM16CAN1LE(mcuPBData->cellvoltage4,canPMBusFrame2->DataBytes);
     FROM16CAN2LE(mcuPBData->cellvoltage5,canPMBusFrame2->DataBytes);
     FROM16CAN3LE(mcuPBData->cellvoltage6,canPMBusFrame2->DataBytes);
     
     canPMBusFrame3->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame3->toggle=0u;
     canPMBusFrame3->endTrans=0u;
     FROM16CAN1LE(mcuPBData->current,canPMBusFrame3->DataBytes);
     FROM16CAN2LE(mcuPBData->voltage,canPMBusFrame3->DataBytes);
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->temperature);
     FROM16CAN3LE(linear16Val,canPMBusFrame3->DataBytes);

     canPMBusFrame4->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame4->toggle=1u;
     canPMBusFrame4->endTrans=0u;
     FROM16CAN3LE(mcuPBData->batterystatus,canPMBusFrame4->DataBytes);
     FROM32CAN1LE(mcuPBData->safetystatus,canPMBusFrame4->DataBytes);
     
     canPMBusFrame5->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame5->toggle=0u;
     canPMBusFrame5->endTrans=1u;
     FROM32CAN1LE(mcuPBData->operationstatus,canPMBusFrame5->DataBytes);
}
/*-----------------------------------------------------------------------------
 *      convBatteryPackforPMBusBE():  get power board data from mavlink message
 *                                       (big endian)
 *  Parameters: const mavlink_sens_batmon_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1, 
 *  canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameBody_t *canPMBusFrame3, 
 *  canBusMultiFrameBody_t *canPMBusFrame4, canBusMultiFrameEnd_t *canPMBusFrame5
 *
 *
 *  Return:     nil
 *----------------------------------------------------------------------------*/
void convBatteryPackforPMBusBE( const mavlink_sens_batmon_t *mcuPBData, canBusMultiFrameStart_t *canPMBusFrame1, canBusMultiFrameBody_t *canPMBusFrame2, canBusMultiFrameBody_t *canPMBusFrame3, canBusMultiFrameBody_t *canPMBusFrame4, canBusMultiFrameEnd_t *canPMBusFrame5 )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */

     if ((((((mcuPBData == NULL) || (canPMBusFrame1 == NULL)) || (canPMBusFrame2 == NULL)) || (canPMBusFrame3 == NULL)) || (canPMBusFrame4 == NULL)) || (canPMBusFrame5 == NULL))
     {
         return;                                                                /* Null passed return without action */
     }
     
     canPMBusFrame1->startTrans=1u;                                             /* set the tail bytes */
     canPMBusFrame1->toggle=0u;
     canPMBusFrame1->endTrans=0u;
     FROM16CAN1BE(mcuPBData->cellvoltage1,canPMBusFrame1->DataBytes);           /* put the bytes in big endian order */
     FROM16CAN2BE(mcuPBData->cellvoltage2,canPMBusFrame1->DataBytes);
     FROM16CAN3BE(mcuPBData->cellvoltage3,canPMBusFrame1->DataBytes);

     canPMBusFrame2->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame2->toggle=1u;
     canPMBusFrame2->endTrans=0u;
     FROM16CAN1BE(mcuPBData->cellvoltage4,canPMBusFrame2->DataBytes);           /* put the bytes in big endian order */
     FROM16CAN2BE(mcuPBData->cellvoltage5,canPMBusFrame2->DataBytes);
     FROM16CAN3BE(mcuPBData->cellvoltage6,canPMBusFrame2->DataBytes);

     canPMBusFrame3->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame3->toggle=0u;
     canPMBusFrame3->endTrans=0u;
     FROM16CAN1BE(mcuPBData->current,canPMBusFrame3->DataBytes);                /* put the bytes in big endian order */
     FROM16CAN2BE(mcuPBData->voltage,canPMBusFrame3->DataBytes);
     linear16Val=linear16Val=float_to_L16((float64_t) mcuPBData->temperature);
     FROM16CAN3BE(linear16Val,canPMBusFrame3->DataBytes);

     canPMBusFrame4->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame4->toggle=1u;
     canPMBusFrame4->endTrans=0u;
     FROM16CAN3BE(mcuPBData->batterystatus,canPMBusFrame4->DataBytes);          /* put the bytes in big endian order */
     FROM32CAN1BE(mcuPBData->safetystatus,canPMBusFrame4->DataBytes);

     canPMBusFrame5->startTrans=0u;                                             /* set the tail bytes */
     canPMBusFrame5->toggle=0u;
     canPMBusFrame5->endTrans=1u;
     FROM32CAN1BE(mcuPBData->operationstatus,canPMBusFrame5->DataBytes);        /* put the bytes in big endian order */
}
/*-----------------------------------------------------------------------------
 *      getPowerBoardDataFromMAV():  get power board data from mavlink message
 *
 *  Parameters: const mavlink_message_t* mavMsg, mavlink_sens_power_board_t *mcuPBData
 *
 *
 *  Return:     int8_t 1=function ok
 *----------------------------------------------------------------------------*/
int8_t getPowerBoardDataFromMAV( const mavlink_message_t* mavMsg, mavlink_sens_power_board_t *mcuPBData )
{

   if ((mavMsg == NULL) || (mcuPBData == NULL))                                 /* passed either as a NULL object then return immediately */
   {
      return -1;
   }

   mavlink_msg_sens_power_board_decode( mavMsg, mcuPBData );                    /* decode MAVLINK received packet to power board data structure */
   return 1;

}
/*-----------------------------------------------------------------------------
 *      getBatteryDataFromMAV():  get battery data from mavlink message
 *                                       (little endian)
 *  Parameters: const mavlink_message_t* mavMsg, mavlink_sens_batmon_t *mcuPBData
 *
 *
 *  Return:     int8_t 1=function ok
 *----------------------------------------------------------------------------*/
int8_t getBatteryDataFromMAV( const mavlink_message_t* mavMsg, mavlink_sens_batmon_t *mcuPBData )
{

   if ((mavMsg == NULL) || (mcuPBData == NULL))                                 /* passed either as a NULL object then return immediately */
   {
      return -1;
   }

   mavlink_msg_sens_batmon_decode( mavMsg, mcuPBData );                         /* decode MAVLINK received packet to battery pack data structure */
   return 1;

}
/*-----------------------------------------------------------------------------
 *      convPMBusforMAVPowerBoardVoltsLE():  converts linear16 data into mavlink board volts
 *                                       (little endian)
 *  Parameters: mavlink_sens_power_board_t *mcuPBData, const canBusSingleFrame_t *canPMBusController
 *
 *
 *  Return:     int8_t 1=all frames transmitted
 *----------------------------------------------------------------------------*/
int8_t convPMBusforMAVPowerBoardVoltsLE( mavlink_sens_power_board_t *mcuPBData, const canBusSingleFrame_t *canPMBusController )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */
     int8_t exponent = ROUND8INT(pow(2.0f,(float64_t) DEVICE_LTC3880));          /* calculate the exponent we are using the LTC3880 device here */
     
     if ((mcuPBData == NULL) || (canPMBusController == NULL))
     {
        return 0u;
     }
     
     if (((canPMBusController->startTrans==1u) && (canPMBusController->toggle==0u)) && (canPMBusController->endTrans==1u)) /* check its a single packet message */
     {
        linear16Val=(((canPMBusController->DataBytes[0u]<<8u)&0xFFU) || (canPMBusController->DataBytes[1u]&0xFFU));  /* make the interger16 which represents the linear16 float */
        mcuPBData->pwr_brd_system_volt=(float32_t)L16_to_float(exponent, linear16Val);        /* convert the linear 16 to a float64_t and typecast to the float32 used in MAVLINK */
        linear16Val=(((canPMBusController->DataBytes[2u]<<8u)&0xFFU) || (canPMBusController->DataBytes[3u]&0xFFU));
        mcuPBData->pwr_brd_servo_volt=(float32_t)L16_to_float(exponent, linear16Val);
        linear16Val=(((canPMBusController->DataBytes[4u]<<8u)&0xFFU) || (canPMBusController->DataBytes[5u]&0xFFU));
        mcuPBData->pwr_brd_digital_volt=(float32_t)L16_to_float(exponent, linear16Val);
        return 1;
     }
     else
     {
        return -1;
     }
}
/*-----------------------------------------------------------------------------
 *      filterVoltsInit():  initializes battery filter object
 *
 *
 *  Parameters: Batt_filter_t *battFilt
 *
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void filterVoltsInit( Batt_filter_t *battFilt )
{
     memset((void*)battFilt,0u,sizeof(Batt_filter_t));
}
/*-----------------------------------------------------------------------------
 *      convL16CypressMv2RawVolts():  converts linear16 mV data into raw board volts
 *                                    it is placed into the filter object for filtering
 *
 *  Parameters: uint16_t linear16Val, Batt_filter_t *battFilt
 *
 *
 *  Return:     int8_t 1=all frames transmitted
 *----------------------------------------------------------------------------*/
void convL16CypressMv2RawVolts( uint16_t linear16Val, Batt_filter_t *battFilt )
{
     int8_t exponent = ROUND8INT(pow(2.0f,(float64_t) DEVICE_LTC3880));         /* calculate the exponent we are using the LTC3880 device here */
     battFilt->vbat_raw = ((float32_t)L16_to_float(exponent, linear16Val)/1000.0f);        /* convert the linear 16 to a float64_t and typecast to the float32 used in MAVLINK */
}
/*-----------------------------------------------------------------------------
 *      convPMBusforMAVPowerBoardVoltsBE():  converts linear16 data into mavlink board volts
 *                                       (big endian)
 *  Parameters: mavlink_sens_power_board_t *mcuPBData, const canBusSingleFrame_t *canPMBusController
 *
 *
 *  Return:     int8_t 1=all frames transmitted
 *----------------------------------------------------------------------------*/
int8_t convPMBusforMAVPowerBoardVoltsBE( mavlink_sens_power_board_t *mcuPBData, const canBusSingleFrame_t *canPMBusController )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */
     int8_t exponent = ROUND8INT(pow(2.0f,(float64_t) DEVICE_LTC3880));          /* calculate the exponent we are using the LTC3880 device here */
     if ((mcuPBData == NULL) || (canPMBusController == NULL))
     {
        return 0u;
     }
     
     if (((canPMBusController->startTrans==1u) && (canPMBusController->toggle==0u)) && (canPMBusController->endTrans==1u)) /* check its a single packet message */
     {
        linear16Val=(((canPMBusController->DataBytes[1u]<<8u)&0xFFU) || (canPMBusController->DataBytes[0u]&0xFFU));  /* make the interger16 which represents the linear16 float */
        mcuPBData->pwr_brd_system_volt=(float32_t)L16_to_float(exponent, linear16Val);        /* convert the linear 16 to a float64_t and typecast to the float32 used in MAVLINK */
        linear16Val=(((canPMBusController->DataBytes[3u]<<8u)&0xFFU) || (canPMBusController->DataBytes[2u]&0xFFU));
        mcuPBData->pwr_brd_servo_volt=(float32_t)L16_to_float(exponent, linear16Val);
        linear16Val=(((canPMBusController->DataBytes[5u]<<8u)&0xFFU) || (canPMBusController->DataBytes[4u]&0xFFU));
        mcuPBData->pwr_brd_digital_volt=(float32_t)L16_to_float(exponent, linear16Val);
        return 1;
     }
     else
     {
        return -1;
     }
}
/*-----------------------------------------------------------------------------
 *      convPMBusforMAVPowerBoardAmpsLE():  converts linear16 data into mavlink battery
 *                                       (little endian)
 *  Parameters: mavlink_sens_power_board_t *mcuPBData,const canBusMultiFrameStart_t *canPMBusFrame1,
 *  const canBusMultiFrameBody_t *canPMBusFrame2,const canBusMultiFrameEnd_t *canPMBusFrame3
 *
 *
 *  Return:     int8_t 1=all 3 frames transmitted
 *----------------------------------------------------------------------------*/
int8_t convPMBusforMAVPowerBoardAmpsLE( mavlink_sens_power_board_t *mcuPBData,const canBusMultiFrameStart_t *canPMBusFrame1,const canBusMultiFrameBody_t *canPMBusFrame2,const canBusMultiFrameEnd_t *canPMBusFrame3 )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */
     int8_t exponent = ROUND8INT(pow(2.0f,(float64_t) DEVICE_LTC3880));          /* calculate the exponent we are using the LTC3880 device here */
     int8_t fRet=0;

     if ((((mcuPBData == NULL) || (canPMBusFrame1 == NULL)) || (canPMBusFrame2 == NULL)) || (canPMBusFrame3 == NULL))
     {
         return fRet;                                                           /* Null passed return without action */
     }
     
     if (((canPMBusFrame1->startTrans==1u) && (canPMBusFrame1->toggle==0u)) && (canPMBusFrame1->endTrans==0u))
     {
        linear16Val=(((canPMBusFrame1->DataBytes[1u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[0u]&0xFFU));
        mcuPBData->pwr_brd_mot_l_amp=((float32_t)L16_to_float((uint8_t)exponent, linear16Val));
        linear16Val=(((canPMBusFrame1->DataBytes[3u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[2u]&0xFFU));
        mcuPBData->pwr_brd_mot_r_amp=(float32_t)L16_to_float(exponent, linear16Val);
        linear16Val=(((canPMBusFrame1->DataBytes[5u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[4u]&0xFFU));
        mcuPBData->pwr_brd_analog_amp=(float32_t)L16_to_float(exponent, linear16Val);
        fRet=++fRet % INT8_MAX;
     }
     
     if (((canPMBusFrame2->startTrans==0u) && (canPMBusFrame2->toggle==1u)) && (canPMBusFrame2->endTrans==0u))
     {
        linear16Val=(((canPMBusFrame2->DataBytes[1u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[0u]&0xFFU));
        mcuPBData->pwr_brd_digital_amp=(float32_t)L16_to_float(exponent, linear16Val);
        linear16Val=(((canPMBusFrame2->DataBytes[3u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[2u]&0xFFU));
        mcuPBData->pwr_brd_ext_amp=(float32_t)L16_to_float(exponent, linear16Val);
        linear16Val=(((canPMBusFrame2->DataBytes[5u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[4u]&0xFFU));
        mcuPBData->pwr_brd_aux_amp=(float32_t)L16_to_float(exponent, linear16Val);
        fRet=++fRet % INT8_MAX;
     }

     if (((canPMBusFrame3->startTrans==0u) && (canPMBusFrame3->toggle==0u)) && (canPMBusFrame3->endTrans==1u))
     {
        mcuPBData->pwr_brd_led_status = canPMBusFrame3->DataBytes[1u];
        mcuPBData->pwr_brd_status = canPMBusFrame3->DataBytes[0u];
        fRet=++fRet % INT8_MAX;
     }
     
     if (fRet>=3)                                                               /* all 3 frames had correct tail bits in tail byte */
       return 1;
     else
       return -1;
}
/*-----------------------------------------------------------------------------
 *      convPMBusforMAVPowerBoardAmpsBE():  converts linear16 data into mavlink battery
 *                                       (big endian)
 *  Parameters: mavlink_sens_power_board_t *mcuPBData,const canBusMultiFrameStart_t *canPMBusFrame1,
 *  const canBusMultiFrameBody_t *canPMBusFrame2,const canBusMultiFrameEnd_t *canPMBusFrame3
 *
 *
 *  Return:     int8_t 1=all 3 frames transmitted
 *----------------------------------------------------------------------------*/
int8_t convPMBusforMAVPowerBoardAmpsBE( mavlink_sens_power_board_t *mcuPBData,const canBusMultiFrameStart_t *canPMBusFrame1,const canBusMultiFrameBody_t *canPMBusFrame2,const canBusMultiFrameEnd_t *canPMBusFrame3 )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */
     int8_t exponent = ROUND8INT(pow(2.0f,(float64_t) DEVICE_LTC3880));         /* calculate the exponent we are using the LTC3880 device here */
     int8_t fRet=0;

     if ((((mcuPBData == NULL) || (canPMBusFrame1 == NULL)) || (canPMBusFrame2 == NULL)) || (canPMBusFrame3 == NULL))
     {
         return fRet;                                                           /* Null passed return without action */
     }
     
     if (((canPMBusFrame1->startTrans==1u) && (canPMBusFrame1->toggle==0u)) && (canPMBusFrame1->endTrans==0u))
     {
        linear16Val=(((canPMBusFrame1->DataBytes[0u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[1u]&0xFFU));
        mcuPBData->pwr_brd_mot_l_amp=((float32_t)L16_to_float(exponent, linear16Val));
        linear16Val=(((canPMBusFrame1->DataBytes[2u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[3u]&0xFFU));
        mcuPBData->pwr_brd_mot_r_amp=(float32_t)L16_to_float(exponent, linear16Val);
        linear16Val=(((canPMBusFrame1->DataBytes[4u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[5u]&0xFFU));
        mcuPBData->pwr_brd_analog_amp=(float32_t)L16_to_float(exponent, linear16Val);
        fRet=++fRet % INT8_MAX;
     }

     if (((canPMBusFrame2->startTrans==0u) && (canPMBusFrame2->toggle==1u)) && (canPMBusFrame2->endTrans==0u))
     {
        linear16Val=(((canPMBusFrame2->DataBytes[0u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[1u]&0xFFU));
        mcuPBData->pwr_brd_digital_amp=(float32_t)L16_to_float(exponent, linear16Val);
        linear16Val=(((canPMBusFrame2->DataBytes[2u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[3u]&0xFFU));
        mcuPBData->pwr_brd_ext_amp=(float32_t)L16_to_float(exponent, linear16Val);
        linear16Val=(((canPMBusFrame2->DataBytes[4u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[5u]&0xFFU));
        mcuPBData->pwr_brd_aux_amp=(float32_t)L16_to_float(exponent, linear16Val);
        fRet=++fRet % INT8_MAX;
     }

     if (((canPMBusFrame3->startTrans==0u) && (canPMBusFrame3->toggle==0u)) && (canPMBusFrame3->endTrans==1u))
     {
        mcuPBData->pwr_brd_led_status = canPMBusFrame3->DataBytes[0u];
        mcuPBData->pwr_brd_status = canPMBusFrame3->DataBytes[1u];
        fRet=++fRet % INT8_MAX;
     }
     
     if (fRet>=3)                                                               /* all 3 frames had correct tail bits in tail byte */
       return 1;
     else
       return -1;
}
/*-----------------------------------------------------------------------------
 *      convPMBusforMAVBatteryPackLE():  converts linear16 data into mavlink battery
 *                                       (little endian endian)
 *  Parameters: mavlink_sens_batmon_t *mcuPBData,const canBusMultiFrameStart_t *canPMBusFrame1,
 *  const canBusMultiFrameBody_t *canPMBusFrame2,const canBusMultiFrameBody_t *canPMBusFrame3,
 *  const canBusMultiFrameBody_t *canPMBusFrame4,const canBusMultiFrameEnd_t *canPMBusFrame5
 *
 *
 *  Return:     int8_t 1=all 5 frames transmitted
 *----------------------------------------------------------------------------*/
int8_t convPMBusforMAVBatteryPackLE( mavlink_sens_batmon_t *mcuPBData,const canBusMultiFrameStart_t *canPMBusFrame1,const canBusMultiFrameBody_t *canPMBusFrame2,const canBusMultiFrameBody_t *canPMBusFrame3,const canBusMultiFrameBody_t *canPMBusFrame4,const canBusMultiFrameEnd_t *canPMBusFrame5 )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */
     int8_t exponent = ROUND8INT(pow(2.0f,(float64_t) DEVICE_LTC3880));          /* calculate the exponent we are using the LTC3880 device here */
     int8_t fRet=0;

     if ((((((mcuPBData == NULL) || (canPMBusFrame1 == NULL)) || (canPMBusFrame2 == NULL)) || (canPMBusFrame3 == NULL)) || (canPMBusFrame4 == NULL)) || (canPMBusFrame5 == NULL))
     {
         return fRet;                                                           /* Null passed return without action */
     }
     
     if (((canPMBusFrame1->startTrans==1u) && (canPMBusFrame1->toggle==0u)) && (canPMBusFrame1->endTrans==0u))
     {
        mcuPBData->cellvoltage1=(((canPMBusFrame1->DataBytes[0u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[1u]&0xFFU));
        mcuPBData->cellvoltage2=(((canPMBusFrame1->DataBytes[2u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[3u]&0xFFU));
        mcuPBData->cellvoltage3=(((canPMBusFrame1->DataBytes[4u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[5u]&0xFFU));
        fRet=++fRet % INT8_MAX;
     }
     
     if (((canPMBusFrame2->startTrans==0u) && (canPMBusFrame2->toggle==1u)) && (canPMBusFrame2->endTrans==0u))
     {
        mcuPBData->cellvoltage4=(((canPMBusFrame2->DataBytes[0u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[1u]&0xFFU));
        mcuPBData->cellvoltage5=(((canPMBusFrame2->DataBytes[2u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[3u]&0xFFU));
        mcuPBData->cellvoltage6=(((canPMBusFrame2->DataBytes[4u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[5u]&0xFFU));
        fRet=++fRet % INT8_MAX;
     }

     if (((canPMBusFrame3->startTrans==0u) && (canPMBusFrame3->toggle==0u)) && (canPMBusFrame3->endTrans==0u))
     {
        mcuPBData->current=(((canPMBusFrame3->DataBytes[0u]<<8u)&0xFFU) || (canPMBusFrame3->DataBytes[1u]&0xFFU));
        mcuPBData->voltage=(((canPMBusFrame3->DataBytes[2u]<<8u)&0xFFU) || (canPMBusFrame3->DataBytes[3u]&0xFFU));
        linear16Val=(((canPMBusFrame3->DataBytes[4u]<<8u)&0xFFU) || (canPMBusFrame3->DataBytes[5u]&0xFFU));
        mcuPBData->temperature=(float32_t)L16_to_float(exponent, linear16Val);
        fRet=++fRet % INT8_MAX;
     }

     if (((canPMBusFrame4->startTrans==0u) && (canPMBusFrame4->toggle==1u)) && (canPMBusFrame4->endTrans==0u))
     {
        mcuPBData->batterystatus=(((canPMBusFrame4->DataBytes[0u]<<8u)&0xFFU) || (canPMBusFrame4->DataBytes[1u]&0xFFU));
        mcuPBData->safetystatus=((((canPMBusFrame4->DataBytes[2u]<<24u)&0xFFU) || ((canPMBusFrame4->DataBytes[3u]<<16u)&0xFFU)) || (((canPMBusFrame4->DataBytes[4u]<<8u)&0xFFU) || ((canPMBusFrame4->DataBytes[5u]<<0u)&0xFFU)));
        fRet=++fRet % INT8_MAX;
     }
     
     if (((canPMBusFrame5->startTrans==0u) && (canPMBusFrame5->toggle==0u)) && (canPMBusFrame5->endTrans==1u))
     {
        mcuPBData->operationstatus=((((canPMBusFrame5->DataBytes[0u]<<24u)&0xFFU) || ((canPMBusFrame5->DataBytes[1u]<<16u)&0xFFU)) || (((canPMBusFrame5->DataBytes[2u]<<8u)&0xFFU) || ((canPMBusFrame5->DataBytes[3u]<<0u)&0xFFU)));
        fRet=++fRet % INT8_MAX;
     }
     
     if (fRet>=5)                                                               /* all 5 frames had correct tail bits in tail byte */
       return 1;                                                                /* return with success */
     else
       return -1;                                                               /* return an error */
     
}
/*-----------------------------------------------------------------------------
 *      convPMBusforMAVBatteryPackBE():  converts linear16 data into mavlink battery
 *                                       (big endian)
 *  Parameters: mavlink_sens_batmon_t *mcuPBData,const canBusMultiFrameStart_t *canPMBusFrame1,
 *  const canBusMultiFrameBody_t *canPMBusFrame2,const canBusMultiFrameBody_t *canPMBusFrame3,
 *  const canBusMultiFrameBody_t *canPMBusFrame4,const canBusMultiFrameEnd_t *canPMBusFrame5
 *
 *
 *  Return:     int8_t 1=all 5 frames transmitted
 *----------------------------------------------------------------------------*/
int8_t convPMBusforMAVBatteryPackBE( mavlink_sens_batmon_t *mcuPBData,const canBusMultiFrameStart_t *canPMBusFrame1,const canBusMultiFrameBody_t *canPMBusFrame2,const canBusMultiFrameBody_t *canPMBusFrame3,const canBusMultiFrameBody_t *canPMBusFrame4,const canBusMultiFrameEnd_t *canPMBusFrame5 )
{
     uint16_t linear16Val;                                                      /* store for a linear 16 represented real number */
     int8_t exponent = ROUND8INT(pow(2.0f,(float64_t) DEVICE_LTC3880));          /* calculate the exponent we are using the LTC3880 device here */
     int8_t fRet=0;

     if ((((((mcuPBData == NULL) || (canPMBusFrame1 == NULL)) || (canPMBusFrame2 == NULL)) || (canPMBusFrame3 == NULL)) || (canPMBusFrame4 == NULL)) || (canPMBusFrame5 == NULL))
     {
         return fRet;                                                           /* Null passed return without action */
     }
     
     if (((canPMBusFrame1->startTrans==1u) && (canPMBusFrame1->toggle==0u)) && (canPMBusFrame1->endTrans==0u))  /* check tail byte bits are as expected */
     {
        mcuPBData->cellvoltage1=(((canPMBusFrame1->DataBytes[1u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[0u]&0xFFU));  /* convert the byte stream to the data */
        mcuPBData->cellvoltage2=(((canPMBusFrame1->DataBytes[3u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[2u]&0xFFU));
        mcuPBData->cellvoltage3=(((canPMBusFrame1->DataBytes[5u]<<8u)&0xFFU) || (canPMBusFrame1->DataBytes[4u]&0xFFU));
        fRet=++fRet % INT8_MAX;
     }

     if (((canPMBusFrame2->startTrans==0u) && (canPMBusFrame2->toggle==1u)) && (canPMBusFrame2->endTrans==0u)) /* check tail byte bits are as expected */
     {
        mcuPBData->cellvoltage4=(((canPMBusFrame2->DataBytes[1u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[0u]&0xFFU)); /* convert the byte stream to the data */
        mcuPBData->cellvoltage5=(((canPMBusFrame2->DataBytes[3u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[2u]&0xFFU));
        mcuPBData->cellvoltage6=(((canPMBusFrame2->DataBytes[5u]<<8u)&0xFFU) || (canPMBusFrame2->DataBytes[4u]&0xFFU));
        fRet=++fRet % INT8_MAX;
     }

     if (((canPMBusFrame3->startTrans==0u) && (canPMBusFrame3->toggle==0u)) && (canPMBusFrame3->endTrans==0u)) /* check tail byte bits are as expected */
     {
        mcuPBData->current=(((canPMBusFrame3->DataBytes[1u]<<8u)&0xFFU) || (canPMBusFrame3->DataBytes[0u]&0xFFU));  /* convert the byte stream to the data */
        mcuPBData->voltage=(((canPMBusFrame3->DataBytes[3u]<<8u)&0xFFU) || (canPMBusFrame3->DataBytes[2u]&0xFFU));
        linear16Val=(((canPMBusFrame3->DataBytes[5u]<<8u)&0xFFU) || (canPMBusFrame3->DataBytes[4u]&0xFFU));
        mcuPBData->temperature=(float32_t)L16_to_float(exponent, linear16Val);
        fRet=++fRet % INT8_MAX;
     }

     if (((canPMBusFrame4->startTrans==0u) && (canPMBusFrame4->toggle==1u)) && (canPMBusFrame4->endTrans==0u)) /* check tail byte bits are as expected */
     {
        mcuPBData->batterystatus=(((canPMBusFrame4->DataBytes[1u]<<8u)&0xFFU) || (canPMBusFrame4->DataBytes[0u]&0xFFU));  /* convert the byte stream to the data */
        mcuPBData->safetystatus=((((canPMBusFrame4->DataBytes[5u]<<24u)&0xFFU) || ((canPMBusFrame4->DataBytes[4u]<<16u)&0xFFU)) || (((canPMBusFrame4->DataBytes[3u]<<8u)&0xFFU) || ((canPMBusFrame4->DataBytes[2u]<<0u)&0xFFU)));
        fRet=++fRet % INT8_MAX;
     }

     if (((canPMBusFrame5->startTrans==0u) && (canPMBusFrame5->toggle==0u)) && (canPMBusFrame5->endTrans==1u)) /* check tail byte bits are as expected */
     {
        mcuPBData->operationstatus=((((canPMBusFrame5->DataBytes[3u]<<24u)&0xFFU) || ((canPMBusFrame5->DataBytes[2u]<<16u)&0xFFU)) || (((canPMBusFrame5->DataBytes[1u]<<8u)&0xFFU) || ((canPMBusFrame5->DataBytes[0u]<<0u)&0xFFU))); /* convert the byte stream to the data */
        fRet=++fRet % INT8_MAX;
     }

     if (fRet>=5)                                                               /* all 5 frames had correct tail bits in tail byte */
       return 1;                                                                /* return with success */
     else
       return -1;                                                               /* return an error */

}
/*-----------------------------------------------------------------------------
 *      SendPowerBoardMAVLinkMessage():  send mavlink message for battery monitoring
 *
 *  Parameters: mavlink_sens_power_board_t *mcuPBData, uint8_t target_sys, uint8_t target_comp
 *
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendPowerBoardMAVLinkMessage( mavlink_sens_power_board_t *mcuPBData, uint8_t target_sys, uint8_t target_comp )
{

 /*
 * @brief Pack a power board message
 *
 * @param conf mavlink_sens_power_board_t power board information structure
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED

  uint8_t buf[MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN];                             /* buffer containing the message to send */

  mavlink_message_t message;
  
  if (mcuPBData==NULL)                                                          /* if null was passed return */
    return;
    
  mcuPBData->timestamp    = getMAVUptime();                                     /* populate the header of the transmission packet with a timestamp */

  len = mavlink_msg_sens_power_board_encode(target_sys, target_comp, &message, mcuPBData);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

#if defined(SERIAL_MAVLINK)                                                     // Send message you write 
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_SENS_BATMON_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_SENS_BATMON_LEN );
#endif

}
/*-----------------------------------------------------------------------------
 *      SendBatteryMAVLinkMessage():  send mavlink message for battery monitoring
 *
 *  Parameters: mavlink_sens_batmon_t *mcuPBData, uint8_t target_sys, uint8_t target_comp
 *
 *
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void SendBatteryMAVLinkMessage( mavlink_sens_batmon_t *mcuPBData, uint8_t target_sys, uint8_t target_comp )
{

 /*
 * @brief Pack a battery pack message
 *
 * @param mavlink_sens_batmon_t battery pack information structure
 * @param target_sys uint8_t target system
 * @param target_comp uint8_t target component
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  uint16_t len=0u;

#ifdef MAV_LIB_GENERATED

  uint8_t buf[MAVLINK_MSG_ID_SENS_BATMON_LEN];                                  /* buffer containing the message to send */

  mavlink_message_t message;

  if (mcuPBData==NULL)                                                          /* if null was passed return */
    return;

  mcuPBData->batmon_timestamp    = getMAVUptime();                              /* populate the header of the transmission packet with a timestamp */

  len = mavlink_msg_sens_batmon_encode(target_sys, target_comp, &message, mcuPBData);  /* encode */

  len = mavlink_msg_to_send_buffer(buf, &message);                              /* Copy the message to send buffer when buf is uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]; */
#endif

  // Send message you write to your serial UART_Write
  //fcboardSerial.write(buf, len);  void UARTx_Write(unsigned _data);
#if defined(SERIAL_MAVLINK)
  sendMavlinkSerial( &buf, MAVLINK_MSG_ID_SENS_BATMON_LEN );
#elif defined(USE_MAVLINK)
  sendMavlinkUDP( &buf, MAVLINK_MSG_ID_SENS_BATMON_LEN );
#endif
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif