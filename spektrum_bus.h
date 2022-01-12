// Copyright 2013 by Horizon Hobby, Inc.
// All Rights Reserved Worldwide.
// This header file may be incorporated into non-Horizon
// products.

// ------------- read over i2c -------------------------------------
// byte 1 = i2c address
// bytes X = as defined in each structure

#ifndef SPEKTRUM_TELEMETRY_H
#define SPEKTRUM_TELEMETRY_H
#include <stdint.h>
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define SPEKPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define SPEKPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define SPEKPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define SPEKPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define SPEKPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

// typedefs for this library
#define UINT8 uint8_t
#define UINT16 uint16_t
#define INT16 int16_t
#define UINT32 uint32_t
#define INT32 int32_t

// Assigned I2C Addresses and Device Types //
#define TELE_DEVICE_NODATA (0x00u)                                               // No data in packet
#define TELE_DEVICE_VOLTAGE (0x01u)                                              // High-Voltage sensor (INTERNAL)
#define TELE_DEVICE_TEMPERATURE (0x02u)                                          // Temperature Sensor (INTERNAL)
#define TELE_DEVICE_RSV_03 (0x03u)                                               // Reserved
#define TELE_DEVICE_RSV_04 (0x04u)                                               // Reserved
#define TELE_DEVICE_RSV_05 (0x05u)                                               // Reserved
#define TELE_DEVICE_RSV_06 (0x06u)                                               // Reserved
#define TELE_DEVICE_RSV_07 (0x07u)                                               // Reserved
#define TELE_DEVICE_RSV_08 (0x08u)                                               // Reserved
#define TELE_DEVICE_RSV_09 (0x09u)                                               // Reserved
#define TELE_DEVICE_PBOX (0x0Au)                                                 // PowerBox
#define TELE_DEVICE_AIRSPEED (0x11u)                                             // Air Speed
#define TELE_DEVICE_ALTITUDE (0x12u)                                             // Altitude
#define TELE_DEVICE_GMETER (0x14u)                                               // GForce
#define TELE_DEVICE_JETCAT (0x15u)                                               // JetCat interface
#define TELE_DEVICE_GPS_LOC (0x16u)                                              // GPS Location Data
#define TELE_DEVICE_GPS_STATS (0x17u)                                            // GPS Status
#define TELE_DEVICE_ENERGY_DUAL (0x18u)                                          // Dual Coulomb counter
#define TELE_DEVICE_JETCAT_2 (0x19u)                                             // JetCat interface, msg 2
#define TELE_DEVICE_GYRO (0x1Au)                                                 // 3-axis gyro
#define TELE_DEVICE_ATTMAG (0x1Bu)                                               // Attitude and Magnetic Compass
#define TELE_DEVICE_AS3X_LEGACYGAIN (0x1Fu)                                      // Active AS3X Gains for legacy mode
#define TELE_DEVICE_ESC (0x20u)                                                  // ESC
#define TELE_DEVICE_FUEL (0x22u)                                                 // Fuel Flow Meter
// DO NOT USE (0x30) // Reserved for internal use
// DO NOT USE (0x32) // Reserved for internal use
#define TELE_DEVICE_MAH (0x34u)                                                  // Battery Gauge (mAh)
#define TELE_DEVICE_DIGITAL_AIR (0x36u)                                          // Digital Inputs & Tank Pressure
#define TELE_DEVICE_STRAIN (0x38u)                                               // Thrust/Strain Gauge
#define TELE_DEVICE_LIPOMON (0x3Au)                                              // Cell Monitor (LiPo taps)
#define TELE_DEVICE_VARIO_S (0x40u)                                              // Vario
#define TELE_DEVICE_RSV_43 (0x43u)                                               // Reserved
#define TELE_DEVICE_USER_16SU (0x50u)                                            // User-Def, STRU_TELE_USER_16SU
#define TELE_DEVICE_USER_16SU32U (0x52u)                                         // User-Def, STRU_TELE_USER_16SU32U
#define TELE_DEVICE_USER_16SU32S (0x54u)                                         // User-Def, STRU_TELE_USER_16SU32S
#define TELE_DEVICE_USER_16U32SU (0x56u)                                         // User-Def, STRU_TELE_USER_16U32SU
#define TELE_DEVICE_RSV_60 (0x60u)                                               // Reserved
#define TELE_DEVICE_RSV_68 (0x68u)                                               // Reserved
#define TELE_DEVICE_RSV_69 (0x69u)                                               // Reserved
#define TELE_DEVICE_RSV_6A (0x6Au)                                               // Reserved
#define TELE_DEVICE_RSV_6B (0x6Bu)                                               // Reserved
#define TELE_DEVICE_RSV_6C (0x6Cu)                                               // Reserved
#define TELE_DEVICE_RSV_6D (0x6Du)                                               // Reserved
#define TELE_DEVICE_RSV_6E (0x6Eu)                                               // Reserved
#define TELE_DEVICE_RSV_6F (0x6Fu)                                               // Reserved
#define TELE_DEVICE_RSV_70 (0x70u)                                               // Reserved
#define TELE_DEVICE_FRAMEDATA (0x7Du)                                            // Transmitter frame data
#define TELE_DEVICE_RPM (0x7Eu)                                                  // RPM sensor
#define TELE_DEVICE_QOS (0x7Fu)                                                  // RxV + flight log data
#define TELE_DEVICE_MAX (0x7Fu)                                                  // Last address available
#define TELE_DEVICE_SHORTRANGE (0x80u)                                           // Data is from a TM1100


 // =================== Message Data Structures ====================== //

typedef enum {
  SPEK_ELE_SPEED_CONT = 0x20,                                                   /* electronic speed control object */
  SPEK_FUEL_CAP_FLOW = 0x22,                                                    /* fuel capacity and flow */
  SPEK_BAT_CAP_CHG = 0x34,                                                      /* battery capacity and charge  */
  SPEK_PRESSURE = 0x36,                                                         /* pressure  */
  SPEK_STRAIN_GAUGE = 0x38,                                                     /* strain gauge  */
  SPEK_LIPO_CELL = 0x3A,                                                        /* Lipo Cell  */
  SPEK_THIRD_PARTYUS16 = 0x50,                                                  /* Third part unsigned and signed 16 bit data  */
  SPEK_THIRD_PARTYU16_32 = 0x52,                                                /* Third part only unsigned 16 and 32 bit data  */
  SPEK_THIRD_PARTYS16_32 = 0x54,                                                /* Third part only signed 16 and 32 bit data  */
  SPEK_THIRD_PARTYSU16_US32 = 0x56,                                             /* Third part only unsigned 16 and 32 bit and signed and unsigned data  */
  SPEK_POWERBOX = 0x7D                                                          /* Powerbox */
} SPEKObjType_e;

#define SPEK_RAW_RPM 655340ul
#define SPEK_SCALED_RPM 10u
#define SPEK_THROT_MAX 127u

#if defined(D_FT900)
typedef struct SPEKPACKED {
UINT8 identifier;                                                               // Source device = 0x20
UINT8 sID;                                                                      // Secondary ID
UINT16 RPM;                                                                     // RPM, 10RPM (0-655340 RPM).     0xFFFF --> "No data"
UINT16 voltsInput;                                                              // Volts, 0.01v (0-655.34V).      0xFFFF --> "No data"
UINT16 tempFET;                                                                 // Temperature, 0.1C (0-999.8C)   0xFFFF --> "No data"
UINT16 currentMotor;                                                            // Current, 10mA (0-655.34A).     0xFFFF --> "No data"
UINT16 tempBEC;                                                                 // Temperature, 0.1C (0-999.8C)   0x7FFF --> "No data"
UINT8 currentBEC;                                                               // BEC Current, 100mA (0-25.4A).  0xFF ----> "No data"
UINT8 voltsBEC;                                                                 // BEC Volts, 0.05V (0-12.70V).   0xFF ----> "No data"
UINT8 throttle;                                                                 // 0.5% (0-127%).                 0xFF ----> "No data"
UINT8 powerOut;                                                                 // Power Output, 0.5% (0-127%).   0xFF ----> "No data"
} STRU_TELE_ESC_t;
#else
 //  Electronic Speed Control //
SPEKPACKED (
typedef struct {
UINT8 identifier;                                                               // Source device = 0x20
UINT8 sID;                                                                      // Secondary ID
UINT16 RPM;                                                                     // RPM, 10RPM (0-655340 RPM).     0xFFFF --> "No data"
UINT16 voltsInput;                                                              // Volts, 0.01v (0-655.34V).      0xFFFF --> "No data"
UINT16 tempFET;                                                                 // Temperature, 0.1C (0-999.8C)   0xFFFF --> "No data"
UINT16 currentMotor;                                                            // Current, 10mA (0-655.34A).     0xFFFF --> "No data"
UINT16 tempBEC;                                                                 // Temperature, 0.1C (0-999.8C)   0x7FFF --> "No data"
UINT8 currentBEC;                                                               // BEC Current, 100mA (0-25.4A).  0xFF ----> "No data"
UINT8 voltsBEC;                                                                 // BEC Volts, 0.05V (0-12.70V).   0xFF ----> "No data"
UINT8 throttle;                                                                 // 0.5% (0-127%).                 0xFF ----> "No data"
UINT8 powerOut;                                                                 // Power Output, 0.5% (0-127%).   0xFF ----> "No data"
}) STRU_TELE_ESC_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 id;                                                                      // Source device = 0x22
 UINT8 sID;                                                                     // Secondary ID
 UINT16 fuelConsumed_A;                                                         // Integrated fuel consumption, 0.1mL
 UINT16 flowRate_A;                                                             // Instantaneous consumption, 0.01mL/min
 UINT16 temp_A;                                                                 // Temperature, 0.1C (0-655.34C)
 UINT16 fuelConsumed_B;                                                         // Integrated fuel consumption, 0.1mL
 UINT16 flowRate_B;                                                             // Instantaneous consumption, 0.01mL/min
 UINT16 temp_B;                                                                 // Temperature, 0.1C (0-655.34C)
 UINT16 spare;                                                                  // Not used
} STRU_TELE_FUEL_t;
#else
 //  Electronic Speed Control //
 //  (Liquid) Fuel Flow/Capacity (Two Tanks/Engines) //
SPEKPACKED (
 typedef struct {
 UINT8 id;                                                                      // Source device = 0x22
 UINT8 sID;                                                                     // Secondary ID
 UINT16 fuelConsumed_A;                                                         // Integrated fuel consumption, 0.1mL
 UINT16 flowRate_A;                                                             // Instantaneous consumption, 0.01mL/min
 UINT16 temp_A;                                                                 // Temperature, 0.1C (0-655.34C)
 UINT16 fuelConsumed_B;                                                         // Integrated fuel consumption, 0.1mL
 UINT16 flowRate_B;                                                             // Instantaneous consumption, 0.01mL/min
 UINT16 temp_B;                                                                 // Temperature, 0.1C (0-655.34C)
 UINT16 spare;                                                                  // Not used
 }) STRU_TELE_FUEL_t;
#endif


#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 id;                                                                      // Source device = 0x34
 UINT8 sID;                                                                     // Secondary ID
 INT16 current_A;                                                               // Instantaneous current, 0.1A (0-3276.8A)
 INT16 chargeUsed_A;                                                            // Integrated mAh used, 1mAh (0-32.766Ah)
 UINT16 temp_A;                                                                 // Temperature, 0.1C (0-150.0C,        // 0x7FFF indicates not populated)
 INT16 current_B;                                                               // Instantaneous current, 0.1A (0-6553.4A)
 INT16 chargeUsed_B;                                                            // Integrated mAh used, 1mAh (0-65.534Ah)
 UINT16 temp_B;                                                                 // Temperature, 0.1C (0-150.0C,        // 0x7FFF indicates not populated)
 UINT16 spare;                                                                  // Not used
} STRU_TELE_MAH_t;
#else
 //  Battery Current/Capacity (Dual Batteries) //
 SPEKPACKED (
 typedef struct {
 UINT8 id;                                                                      // Source device = 0x34
 UINT8 sID;                                                                     // Secondary ID
 INT16 current_A;                                                               // Instantaneous current, 0.1A (0-3276.8A)
 INT16 chargeUsed_A;                                                            // Integrated mAh used, 1mAh (0-32.766Ah)
 UINT16 temp_A;                                                                 // Temperature, 0.1C (0-150.0C,        // 0x7FFF indicates not populated)
 INT16 current_B;                                                               // Instantaneous current, 0.1A (0-6553.4A)
 INT16 chargeUsed_B;                                                            // Integrated mAh used, 1mAh (0-65.534Ah)
 UINT16 temp_B;                                                                 // Temperature, 0.1C (0-150.0C,        // 0x7FFF indicates not populated)
 UINT16 spare;                                                                  // Not used
 }) STRU_TELE_MAH_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 id;                                                                      // Source device = 0x36
 UINT8 sID;                                                                     // Secondary ID
 UINT16 digital;                                                                // Digital inputs (bit per input)
 UINT16 pressure;                                                               // Tank pressure, 0.1PSI (0-6553.4PSI)
} STRU_TELE_DIGITAL_AIR_t;
#else
 //  Digital Input Status (Retract Status) and Tank Pressure //
 SPEKPACKED (
 typedef struct {
 UINT8 id;                                                                      // Source device = 0x36
 UINT8 sID;                                                                     // Secondary ID
 UINT16 digital;                                                                // Digital inputs (bit per input)
 UINT16 pressure;                                                               // Tank pressure, 0.1PSI (0-6553.4PSI)
 }) STRU_TELE_DIGITAL_AIR_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 id;                                                                      // Source device = 0x38
 UINT8 sID;                                                                     // Secondary ID
 UINT16 strain_A;                                                               // Strain sensor A
 UINT16 strain_B;                                                               // Strain sensor B
 UINT16 strain_C;                                                               // Strain sensor D
 UINT16 strain_D;                                                               // Strain sensor C
} STRU_TELE_STRAIN_t;
#else
 //  Thrust/Strain Gauge //
 SPEKPACKED (
 typedef struct {
 UINT8 id;                                                                      // Source device = 0x38
 UINT8 sID;                                                                     // Secondary ID
 UINT16 strain_A;                                                               // Strain sensor A
 UINT16 strain_B;                                                               // Strain sensor B
 UINT16 strain_C;                                                               // Strain sensor D
 UINT16 strain_D;                                                               // Strain sensor C
 }) STRU_TELE_STRAIN_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 id;                                                                      // Source device = 0x3A
 UINT8 sID;                                                                     // Secondary ID
 UINT16 cell[6u];                                                               // Voltage across cell 1    cell_2,   // Voltage across cell 2    cell_3,   // Voltage across cell 3    cell_4,   // Voltage across cell 4    cell_5,   // Voltage across cell 5    cell_6;   // Voltage across cell 6
 UINT16 temp;                                                                   // Temperature, 0.1C (0-655.34C)
} STRU_TELE_LIPOMON_t;
#else
 //  LiPo Cell Monitor //
 SPEKPACKED (
 typedef struct {
 UINT8 id;                                                                      // Source device = 0x3A
 UINT8 sID;                                                                     // Secondary ID
 UINT16 cell[6u];                                                               // Voltage across cell 1    cell_2,   // Voltage across cell 2    cell_3,   // Voltage across cell 3    cell_4,   // Voltage across cell 4    cell_5,   // Voltage across cell 5    cell_6;   // Voltage across cell 6
 UINT16 temp;                                                                   // Temperature, 0.1C (0-655.34C)
 }) STRU_TELE_LIPOMON_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 id;                                                                      // Source device = 0x50
 UINT8 sID;                                                                     // Secondary ID
 INT16 sField[3u];                                                              // Signed 16-bit data fields
 UINT16 uField[4u];                                                             // Unsigned 16-bit data fields    uField2,    uField3,    uField4;
} STRU_TELE_USER_16SU_t;
#else
 //  THIRD-PARTY 16-BIT DATA SIGNED/UNSIGNED //
 SPEKPACKED (
 typedef struct {
 UINT8 id;                                                                      // Source device = 0x50
 UINT8 sID;                                                                     // Secondary ID
 INT16 sField[3u];                                                              // Signed 16-bit data fields
 UINT16 uField[4u];                                                             // Unsigned 16-bit data fields    uField2,    uField3,    uField4;
 }) STRU_TELE_USER_16SU_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 id;                                                                      // Source device = 0x52
 UINT8 sID;                                                                     // Secondary ID
 INT16 sField1[2u];                                                             // Signed 16-bit data fields    sField2;
 UINT16 uField[3u];                                                             // Unsigned 16-bit data fields    uField2,    uField3;
 UINT32 u32Field;                                                               // Unsigned 32-bit data field
} STRU_TELE_USER52_t;
#else
 //  THIRD-PARTY 16-BIT SIGNED/UNSIGNED AND 32-BIT UNSIGNED //
 SPEKPACKED (
 typedef struct {
 UINT8 id;                                                                      // Source device = 0x52
 UINT8 sID;                                                                     // Secondary ID
 INT16 sField1[2u];                                                             // Signed 16-bit data fields    sField2;
 UINT16 uField[3u];                                                             // Unsigned 16-bit data fields    uField2,    uField3;
 UINT32 u32Field;                                                               // Unsigned 32-bit data field
 }) STRU_TELE_USER52_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 id;                                                                      // Source device = 0x54
 UINT8 sID;                                                                     // Secondary ID
 INT16 sField[2u];                                                              // Signed 16-bit data fields    sField2;
 UINT16 uField1[3u];                                                            // Unsigned 16-bit data fields    uField2,    uField3;
 INT32 u32Field;                                                                // Signed 32-bit data field
} STRU_TELE_USER54_t;
#else
 //  THIRD-PARTY 16-BIT SIGNED/UNSIGNED AND 32-BIT SIGNED //
 SPEKPACKED (
 typedef struct {
 UINT8 id;                                                                      // Source device = 0x54
 UINT8 sID;                                                                     // Secondary ID
 INT16 sField[2u];                                                              // Signed 16-bit data fields    sField2;
 UINT16 uField1[3u];                                                            // Unsigned 16-bit data fields    uField2,    uField3;
 INT32 u32Field;                                                                // Signed 32-bit data field
 }) STRU_TELE_USER54_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 id;                                                                      // Source device = 0x56
 UINT8 sID;                                                                     // Secondary ID
 UINT16 uField1;                                                                // Unsigned 16-bit data field
 INT32 u32Field;                                                                // Signed 32-bit data field
 INT32 u32Field1[2u];                                                           // Signed 32-bit data fields
 UINT32 u32Field2[2u];                                                          // Unsigned 32-bit data fields
} STRU_TELE_USER56_t;
#else
 //  THIRD-PARTY 16-BIT UNSIGNED AND 32-BIT SIGNED/UNSIGNED //
 SPEKPACKED (
 typedef struct {
 UINT8 id;                                                                      // Source device = 0x56
 UINT8 sID;                                                                     // Secondary ID
 UINT16 uField1;                                                                // Unsigned 16-bit data field
 INT32 u32Field;                                                                // Signed 32-bit data field
 INT32 u32Field1[2u];                                                           // Signed 32-bit data fields
 UINT32 u32Field2[2u];                                                          // Unsigned 32-bit data fields
 }) STRU_TELE_USER56_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 identifier;                                                              // Source device = 0x7D
 UINT8 sID;                                                                     // Secondary ID
 UINT16 volt1;                                                                  // Volts, 0v01v
 UINT16 volt2;                                                                  // Volts, 0.01v
 UINT16 capacity1;                                                              // mAh, 1mAh
 UINT16 capacity2;                                                              // mAh, 1mAh
 UINT16 spare16_1;
 UINT16 spare16_2;
 UINT8 spare;
 UINT8 alarms;                                                                  // Alarm bitmask (see below)
} STRU_TELE_POWERBOX_t;
#else
//  POWERBOX //
 SPEKPACKED (
 typedef struct {
 UINT8 identifier;                                                              // Source device = 0x7D
 UINT8 sID;                                                                     // Secondary ID
 UINT16 volt1;                                                                  // Volts, 0v01v
 UINT16 volt2;                                                                  // Volts, 0.01v
 UINT16 capacity1;                                                              // mAh, 1mAh
 UINT16 capacity2;                                                              // mAh, 1mAh
 UINT16 spare16_1;
 UINT16 spare16_2;
 UINT8 spare;
 UINT8 alarms;                                                                  // Alarm bitmask (see below)
 }) STRU_TELE_POWERBOX_t;
#endif

#define TELE_PBOX_ALARM_VOLTAGE_1  (0x01u)
#define TELE_PBOX_ALARM_VOLTAGE_2  (0x02u)
#define TELE_PBOX_ALARM_CAPACITY_1 (0x04u)
#define TELE_PBOX_ALARM_CAPACITY_2 (0x08u)
#define TELE_PBOX_ALARM_RPM  (0x10u)
#define TELE_PBOX_ALARM_TEMPERATURE (0x20u)
#define TELE_PBOX_ALARM_RESERVED_1 (0x40u)
#define TELE_PBOX_ALARM_RESERVED_2 (0x80u)

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 id;                                                                      // Source device = 0x18
 UINT8 sID;                                                                     // Secondary ID
 INT16 current_A;                                                               // Instantaneous current, 0.01A (0-328.7A)
 INT16 chargeUsed_A;                                                            // Integrated mAh used, 0.1mAh (0-3276.6mAh)
 UINT16 volts_A;                                                                // Voltage, 0.01VC (0-16.00V)
 INT16 current_B;                                                               // Instantaneous current, 0.1A (0-3276.8A)
 INT16 chargeUsed_B;                                                            // Integrated mAh used, 1mAh (0-32.766Ah)
 UINT16 volts_B;                                                                // Voltage, 0.01VC (0-16.00V)
 UINT16 spare;                                                                  // Not used
} STRU_TELE_ENERGY_DUAL_t;
#else
 //  DUAL ENERGY //
 SPEKPACKED (
 typedef struct {
 UINT8 id;                                                                      // Source device = 0x18
 UINT8 sID;                                                                     // Secondary ID
 INT16 current_A;                                                               // Instantaneous current, 0.01A (0-328.7A)
 INT16 chargeUsed_A;                                                            // Integrated mAh used, 0.1mAh (0-3276.6mAh)
 UINT16 volts_A;                                                                // Voltage, 0.01VC (0-16.00V)
 INT16 current_B;                                                               // Instantaneous current, 0.1A (0-3276.8A)
 INT16 chargeUsed_B;                                                            // Integrated mAh used, 1mAh (0-32.766Ah)
 UINT16 volts_B;                                                                // Voltage, 0.01VC (0-16.00V)
 UINT16 spare;                                                                  // Not used
 }) STRU_TELE_ENERGY_DUAL_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 identifier;                                                              // Source device = 0x03
 UINT8 sID;                                                                     // Secondary ID
 INT16 current;                                                                 // Range: +/- 150A            // Resolution: 300A/2048 = 0.196791 A/tick
 INT16 dummy;                                                                   // TBD
} STRU_TELE_IHIGH_t;
#else
 //  HIGH-CURRENT //
 SPEKPACKED (
 typedef struct {
 UINT8 identifier;                                                              // Source device = 0x03
 UINT8 sID;                                                                     // Secondary ID
 INT16 current;                                                                 // Range: +/- 150A            // Resolution: 300A/2048 = 0.196791 A/tick
 INT16 dummy;                                                                   // TBD
 }) STRU_TELE_IHIGH_t;
#endif

#define IHIGH_RESOLUTION_FACTOR  ((FP32)(0.196791f))

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 identifier;                                                              // Source device = 0x40
 UINT8 sID;                                                                     // Secondary ID
 INT16 altitude;                                                                // .1m increments
 INT16 delta_0250ms;                                                            // delta last 250ms, 0.1m/s increments
 INT16 delta_0500ms;                                                            // delta last 500ms, 0.1m/s increments
 INT16 delta_1000ms;                                                            // delta last 1.0 seconds
 INT16 delta_1500ms;                                                            // delta last 1.5 seconds
 INT16 delta_2000ms;                                                            // delta last 2.0 seconds
 INT16 delta_3000ms;                                                            // delta last 3.0 seconds
} STRU_TELE_VARIO_S_t;
#else
 //  VARIO-S //
 SPEKPACKED (
 typedef struct {
 UINT8 identifier;                                                              // Source device = 0x40
 UINT8 sID;                                                                     // Secondary ID
 INT16 altitude;                                                                // .1m increments
 INT16 delta_0250ms;                                                            // delta last 250ms, 0.1m/s increments
 INT16 delta_0500ms;                                                            // delta last 500ms, 0.1m/s increments
 INT16 delta_1000ms;                                                            // delta last 1.0 seconds
 INT16 delta_1500ms;                                                            // delta last 1.5 seconds
 INT16 delta_2000ms;                                                            // delta last 2.0 seconds
 INT16 delta_3000ms;                                                            // delta last 3.0 seconds
 }) STRU_TELE_VARIO_S_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 identifier;
 UINT8 sID;                                                                     // Secondary ID
 INT16 altitude;                                                                // .1m increments
 INT16 maxAltitude;                                                             // .1m increments
} STRU_TELE_ALT_t;
#else
 //  ALTIMETER //
 SPEKPACKED (
 typedef struct {
 UINT8 identifier;
 UINT8 sID;                                                                     // Secondary ID
 INT16 altitude;                                                                // .1m increments
 INT16 maxAltitude;                                                             // .1m increments
 }) STRU_TELE_ALT_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 identifier;
 UINT8 sID;                                                                     // Secondary ID
 UINT16 airspeed;                                                               // 1 km/h increments
 UINT16 maxAirspeed;                                                            // 1 km/h increments
} STRU_TELE_SPEED_t;
#else
 //  AIRSPEED //
 SPEKPACKED (
 typedef struct {
 UINT8 identifier;
 UINT8 sID;                                                                     // Secondary ID
 UINT16 airspeed;                                                               // 1 km/h increments
 UINT16 maxAirspeed;                                                            // 1 km/h increments
 }) STRU_TELE_SPEED_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 identifier;                                                              // Source device = 0x14
 UINT8 sID;                                                                     // Secondary ID
 INT16 GForceX;                                                                 // force is reported as .01G increments
 INT16 GForceY;                                                                 // Range = +/-4000 (+/- 40G) in Pro model
 INT16 GForceZ;                                                                 // Range = +/-800 (+/- 8G) in Standard model
 INT16 maxGForceX;                                                              // abs(max G X-axis)   FORE/AFT
 INT16 maxGForceY;                                                              // abs (max G Y-axis)  LEFT/RIGHT
 INT16 maxGForceZ;                                                              // max G Z-axis        WING SPAR LOAD
 INT16 minGForceZ;                                                              // min G Z-axis        WING SPAR LOAD
} STRU_TELE_G_METER_t;
#else
 //  GFORCE //
 SPEKPACKED (
 typedef struct {
 UINT8 identifier;                                                              // Source device = 0x14
 UINT8 sID;                                                                     // Secondary ID
 INT16 GForceX;                                                                 // force is reported as .01G increments
 INT16 GForceY;                                                                 // Range = +/-4000 (+/- 40G) in Pro model
 INT16 GForceZ;                                                                 // Range = +/-800 (+/- 8G) in Standard model
 INT16 maxGForceX;                                                              // abs(max G X-axis)   FORE/AFT
 INT16 maxGForceY;                                                              // abs (max G Y-axis)  LEFT/RIGHT
 INT16 maxGForceZ;                                                              // max G Z-axis        WING SPAR LOAD
 INT16 minGForceZ;                                                              // min G Z-axis        WING SPAR LOAD
 }) STRU_TELE_G_METER_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
 UINT8 identifier;                                                              // Source device = 0x15
 UINT8 sID;                                                                     // Secondary ID
 UINT8 status;                                                                  // See table below
 UINT8 throttle;                                                                // (BCD) xx Percent
 UINT16 packVoltage;                                                            // (BCD) xx.yy
 UINT16 pumpVoltage;                                                            // (BCD) xx.yy
 UINT32 RPM;                                                                    // (BCD)
 UINT16 EGT;                                                                    // (BCD) Temperature, Celsius
 UINT8 offCondition;                                                            // (BCD) See table below
 UINT8 spare;
} STRU_TELE_JETCAT_t;
#else
 //  JETCAT/TURBINE //
 SPEKPACKED (
 typedef struct {
 UINT8 identifier;                                                              // Source device = 0x15
 UINT8 sID;                                                                     // Secondary ID
 UINT8 status;                                                                  // See table below
 UINT8 throttle;                                                                // (BCD) xx Percent
 UINT16 packVoltage;                                                            // (BCD) xx.yy
 UINT16 pumpVoltage;                                                            // (BCD) xx.yy
 UINT32 RPM;                                                                    // (BCD)
 UINT16 EGT;                                                                    // (BCD) Temperature, Celsius
 UINT8 offCondition;                                                            // (BCD) See table below
 UINT8 spare;
 }) STRU_TELE_JETCAT_t;
#endif

enum JETCAT_ECU_TURBINE_STATE {                                                 // ECU Status definitions
JETCAT_ECU_STATE_OFF = 0x00,
JETCAT_ECU_STATE_WAIT_for_RPM = 0x01,                                           // (Stby/Start)
JETCAT_ECU_STATE_Ignite = 0x02,
JETCAT_ECU_STATE_Accelerate = 0x03,
JETCAT_ECU_STATE_Stabilise = 0x04,
JETCAT_ECU_STATE_Learn_HI = 0x05,
JETCAT_ECU_STATE_Learn_LO = 0x06,
JETCAT_ECU_STATE_UNDEFINED = 0x07,
JETCAT_ECU_STATE_Slow_Down = 0x08,
JETCAT_ECU_STATE_Manual = 0x09,
JETCAT_ECU_STATE_AutoOff = 0x10,
JETCAT_ECU_STATE_Run = 0x11,                                                    // (reg.)
JETCAT_ECU_STATE_Accleleration_delay = 0x12,
JETCAT_ECU_STATE_SpeedReg = 0x13,                                               // (Speed Ctrl)
JETCAT_ECU_STATE_Two_Shaft_Regulate = 0x14,                                     // (only for secondary shaft)
JETCAT_ECU_STATE_PreHeat1 = 0x15,
JETCAT_ECU_STATE_PreHeat2 = 0x16,
JETCAT_ECU_STATE_MainFStart = 0x17,
JETCAT_ECU_STATE_NotUsed = 0x18,
JETCAT_ECU_STATE_KeroFullOn = 0x19,                                             // undefined states 0x1A-0x1F
EVOJET_ECU_STATE_off = 0x20,
EVOJET_ECU_STATE_ignt = 0x21,
EVOJET_ECU_STATE_acce = 0x22,
EVOJET_ECU_STATE_run = 0x23,
EVOJET_ECU_STATE_cal = 0x24,
EVOJET_ECU_STATE_cool = 0x25,
EVOJET_ECU_STATE_fire = 0x26,
EVOJET_ECU_STATE_glow = 0x27,
EVOJET_ECU_STATE_heat = 0x28,
EVOJET_ECU_STATE_idle = 0x29,
EVOJET_ECU_STATE_rel = 0x2B,
EVOJET_ECU_STATE_spin = 0x2C,
EVOJET_ECU_STATE_stop = 0x2D,                                                   // undefined states 0x2E-0x2F
HORNET_ECU_STATE_OFF = 0x30,
HORNET_ECU_STATE_SLOWDOWN = 0x31,
HORNET_ECU_STATE_COOL_DOWN = 0x32,
HORNET_ECU_STATE_AUTO = 0x33,
HORNET_ECU_STATE_AUTO_HC = 0x34,
HORNET_ECU_STATE_BURNER_ON = 0x35,
HORNET_ECU_STATE_CAL_IDLE = 0x36u,
HORNET_ECU_STATE_CALIBRATE = 0x37u,
HORNET_ECU_STATE_DEV_DELAY = 0x38u,
HORNET_ECU_STATE_EMERGENCY = 0x39u,
HORNET_ECU_STATE_FUEL_HEAT = 0x3Au,
HORNET_ECU_STATE_FUEL_IGNITE = 0x3Bu,
HORNET_ECU_STATE_GO_IDLE = 0x3C,
HORNET_ECU_STATE_PROP_IGNITE = 0x3D,
HORNET_ECU_STATE_RAMP_DELAY = 0x3E,
HORNET_ECU_STATE_RAMP_UP = 0x3F,
HORNET_ECU_STATE_STANDBY = 0x40,
HORNET_ECU_STATE_STEADY = 0x41,
HORNET_ECU_STATE_WAIT_ACC = 0x42,
HORNET_ECU_STATE_ERROR = 0x43,                                                  // undefined states 0x44-0x4F
XICOY_ECU_STATE_Temp_High = 0x50,
XICOY_ECU_STATE_Trim_Low = 0x51,
XICOY_ECU_STATE_Set_Idle = 0x52,
XICOY_ECU_STATE_Ready = 0x53,
XICOY_ECU_STATE_Ignition = 0x54,
XICOY_ECU_STATE_Fuel_Ramp = 0x55,
XICOY_ECU_STATE_Glow_Test = 0x56,
XICOY_ECU_STATE_Running = 0x57,
XICOY_ECU_STATE_Stop = 0x58,
XICOY_ECU_STATE_Flameout = 0x59,
XICOY_ECU_STATE_Speed_Low = 0x5A,
XICOY_ECU_STATE_Cooling = 0x5B,
XICOY_ECU_STATE_Igniter_Bad = 0x5C,
XICOY_ECU_STATE_Starter_F = 0x5D,
XICOY_ECU_STATE_Weak_Fuel = 0x5E,
XICOY_ECU_STATE_Start_On = 0x5F,
XICOY_ECU_STATE_Pre_Heat = 0x60,
XICOY_ECU_STATE_Battery = 0x61,
XICOY_ECU_STATE_Time_Out = 0x62,
XICOY_ECU_STATE_Overload = 0x63,
XICOY_ECU_STATE_Igniter_Fail = 0x64,
XICOY_ECU_STATE_Burner_On = 0x65,
XICOY_ECU_STATE_Starting = 0x66,
XICOY_ECU_STATE_SwitchOver = 0x67,
XICOY_ECU_STATE_Cal_Pump = 0x68,
XICOY_ECU_STATE_Pump_Limit = 0x69,
XICOY_ECU_STATE_No_Engine = 0x6A,
XICOY_ECU_STATE_Pwr_Boost = 0x6B,
XICOY_ECU_STATE_Run_Idle = 0x6C,
XICOY_ECU_STATE_Run_Max = 0x6D,
TURBINE_ECU_MAX_STATE = 0x74 };

enum JETCAT_ECU_OFF_CONDITIONS {                                                // ECU off conditions. Valid only when the ECUStatus = JETCAT_ECU_STATE_OFF
JETCAT_ECU_OFF_No_Off_Condition_defined = 0,
JETCAT_ECU_OFF_Shut_down_via_RC,
JETCAT_ECU_OFF_Overtemperature,
JETCAT_ECU_OFF_Ignition_timeout,
JETCAT_ECU_OFF_Acceleration_time_out,
JETCAT_ECU_OFF_Acceleration_too_slow,
JETCAT_ECU_OFF_Over_RPM,
JETCAT_ECU_OFF_Low_Rpm_Off,
JETCAT_ECU_OFF_Low_Battery,
JETCAT_ECU_OFF_Auto_Off,
JETCAT_ECU_OFF_Low_temperature_Off,
JETCAT_ECU_OFF_Hi_Temp_Off,
JETCAT_ECU_OFF_Glow_Plug_defective,
JETCAT_ECU_OFF_Watch_Dog_Timer,
JETCAT_ECU_OFF_Fail_Safe_Off,
JETCAT_ECU_OFF_Manual_Off,                                                      // (via GSU)
JETCAT_ECU_OFF_Power_fail,                                                      // (Battery fail)
JETCAT_ECU_OFF_Temp_Sensor_fail,                                                // (only during startup)
JETCAT_ECU_OFF_Fuel_fail,
JETCAT_ECU_OFF_Prop_fail,
JETCAT_ECU_OFF_2nd_Engine_fail,
JETCAT_ECU_OFF_2nd_Engine_Diff_Too_High,
JETCAT_ECU_OFF_2nd_Engine_No_Comm,
JETCAT_ECU_MAX_OFF_COND
};

#if defined(D_FT900)
typedef struct SPEKPACKED {
UINT8 identifier;                                                               // Source device = 0x19
UINT8 sID;                                                                      // Secondary ID
UINT16 FuelFlowRateMLMin;                                                       // (BCD) mL per Minute
UINT32 RestFuelVolumeInTankML;                                                  // (BCD) mL remaining in tank  // 8 bytes left
} STRU_TELE_JETCAT2_t;
#else
SPEKPACKED (
typedef struct {
UINT8 identifier;                                                               // Source device = 0x19
UINT8 sID;                                                                      // Secondary ID
UINT16 FuelFlowRateMLMin;                                                       // (BCD) mL per Minute
UINT32 RestFuelVolumeInTankML;                                                  // (BCD) mL remaining in tank  // 8 bytes left
}) STRU_TELE_JETCAT2_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
  UINT8 identifier;                                                             // Source device = 0x16
  UINT8 sID;                                                                    // Secondary ID
  UINT16 altitudeLow;                                                           // BCD, meters, format 3.1 (Low bits of alt)
  UINT32 latitude;                                                              // BCD, format 4.4,        // Degrees * 100 + minutes, < 100 degrees 
  UINT32 longitude;                                                             // BCD, format 4.4,        // Degrees * 100 + minutes, flag --> > 99deg
  UINT16 course;                                                                // BCD, 3.1
  UINT8 HDOP;                                                                   // BCD, format 1.1
  UINT8 GPSflags;                                                               // see definitions below
} STRU_TELE_GPS_LOC_t;
#else
 //  GPS //
 SPEKPACKED (
typedef struct {
  UINT8 identifier;                                                             // Source device = 0x16
  UINT8 sID;                                                                    // Secondary ID
  UINT16 altitudeLow;                                                           // BCD, meters, format 3.1 (Low bits of alt)
  UINT32 latitude;                                                              // BCD, format 4.4,        // Degrees * 100 + minutes, < 100 degrees 
  UINT32 longitude;                                                             // BCD, format 4.4,        // Degrees * 100 + minutes, flag --> > 99deg
  UINT16 course;                                                                // BCD, 3.1
  UINT8 HDOP;                                                                   // BCD, format 1.1
  UINT8 GPSflags;                                                               // see definitions below
}) STRU_TELE_GPS_LOC_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
  UINT8 identifier;                                                             // Source device = 0x17
  UINT8 sID;                                                                    // Secondary ID
  UINT16 speed;                                                                 // BCD, knots, format 3.1
  UINT32 UTC;                                                                   // BCD, format HH:MM:SS.S, format 6.1
  UINT8 numSats;                                                                // BCD, 0-99
  UINT8 altitudeHigh;                                                           // BCD, meters, format 2.0 (High bits alt)
} STRU_TELE_GPS_STAT_t;
#else
SPEKPACKED (
typedef struct {
  UINT8 identifier;                                                             // Source device = 0x17
  UINT8 sID;                                                                    // Secondary ID
  UINT16 speed;                                                                 // BCD, knots, format 3.1
  UINT32 UTC;                                                                   // BCD, format HH:MM:SS.S, format 6.1
  UINT8 numSats;                                                                // BCD, 0-99
  UINT8 altitudeHigh;                                                           // BCD, meters, format 2.0 (High bits alt)
}) STRU_TELE_GPS_STAT_t;
#endif

// GPS flags definitions:
#define GPS_INFO_FLAGS_IS_NORTH_BIT (0)
#define GPS_INFO_FLAGS_IS_NORTH  (1 << GPS_INFO_FLAGS_IS_NORTH_BIT)
#define GPS_INFO_FLAGS_IS_EAST_BIT (1)
#define GPS_INFO_FLAGS_IS_EAST  (1 << GPS_INFO_FLAGS_IS_EAST_BIT)
#define GPS_INFO_FLAGS_LONG_GREATER_99_BIT (2)
#define GPS_INFO_FLAGS_LONG_GREATER_99  (1 << GPS_INFO_FLAGS_LONG_GREATER_99_BIT)
#define GPS_INFO_FLAGS_GPS_FIX_VALID_BIT (3)
#define GPS_INFO_FLAGS_GPS_FIX_VALID (1 << GPS_INFO_FLAGS_GPS_FIX_VALID_BIT)
#define GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT (4)
#define GPS_INFO_FLAGS_GPS_DATA_RECEIVED (1 << GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT)
#define GPS_INFO_FLAGS_3D_FIX_BIT  (5)
#define GPS_INFO_FLAGS_3D_FIX  (1 << GPS_INFO_FLAGS_3D_FIX_BIT)
#define GPS_INFO_FLAGS_NEGATIVE_ALT_BIT (7)
#define GPS_INFO_FLAGS_NEGATIVE_ALT (1 << GPS_INFO_FLAGS_NEGATIVE_ALT_BIT)

#if defined(D_FT900)
typedef struct SPEKPACKED {
   UINT8  identifier;                                                           // Source device = 0x1B
   UINT8  sID;                                                                  // Secondary ID
   INT16  attRoll;                                                              // Attitude, 3 axes.  Roll is a        // rotation about the X Axis of         // the vehicle using the RHR.
   INT16  attPitch;                                                             // Units are 0.1 deg - Pitch is a        // rotation about the Y Axis of the        // vehicle using the RHR.
   INT16  attYaw;                                                               // Yaw is a rotation about the Z         // Axis of the vehicle using the RHR.
   INT16  magX;                                                                 // Magnetic Compass, 3 axes
   INT16  magY;                                                                 // Units are TBD
   INT16  magZ;                                                                 //
} STRU_TELE_ATTMAG_t;
#else
 //  ATTITUDE & MAG COMPASS //
 SPEKPACKED (
typedef struct {
   UINT8  identifier;                                                           // Source device = 0x1B
   UINT8  sID;                                                                  // Secondary ID
   INT16  attRoll;                                                              // Attitude, 3 axes.  Roll is a        // rotation about the X Axis of         // the vehicle using the RHR.
   INT16  attPitch;                                                             // Units are 0.1 deg - Pitch is a        // rotation about the Y Axis of the        // vehicle using the RHR.
   INT16  attYaw;                                                               // Yaw is a rotation about the Z         // Axis of the vehicle using the RHR.
   INT16  magX;                                                                 // Magnetic Compass, 3 axes
   INT16  magY;                                                                 // Units are TBD
   INT16  magZ;                                                                 //
}) STRU_TELE_ATTMAG_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
   UINT8 identifier;                                                            // Source device = 0x7D
   UINT8 sID;                                                                   // Secondary ID
   UINT16 chanData[7u];                                                         // Channel Data array
} STRU_TELE_FRAMEDATA_t;
#else
 //  Transmitter Frame Data //
 SPEKPACKED (
 typedef struct {
   UINT8 identifier;                                                            // Source device = 0x7D
   UINT8 sID;                                                                   // Secondary ID
   UINT16 chanData[7u];                                                         // Channel Data array
 }) STRU_TELE_FRAMEDATA_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
  UINT8 identifier;                                                             // Source device = 0x7E
  UINT8 sID;                                                                    // Secondary ID
  UINT16 microseconds;                                                          // microseconds between pulse leading edges
  UINT16 volts;                                                                 // 0.01V increments
  INT16 temperature;                                                            // degrees F
} STRU_TELE_RPM_t;
#else
//  RPM/Volts/Temperature //
SPEKPACKED (
typedef struct {
  UINT8 identifier;                                                             // Source device = 0x7E
  UINT8 sID;                                                                    // Secondary ID
  UINT16 microseconds;                                                          // microseconds between pulse leading edges
  UINT16 volts;                                                                 // 0.01V increments
  INT16 temperature;                                                            // degrees F
}) STRU_TELE_RPM_t;
#endif

#if defined(D_FT900)
typedef struct SPEKPACKED {
  UINT8 identifier;                                                             // Source device = 0x7F
  UINT8 sID;                                                                    // Secondary ID
  UINT16 A;
  UINT16 B;
  UINT16 L;
  UINT16 R;
  UINT16 F;                                                                     //   F = fades
  UINT16 H;                                                                     //   H = holds
  UINT16 rxVoltage;                                                             // Volts, 0.01V increments rxV = 0xFFFF
} STRU_TELE_QOS_t;
#else
//  QoS DATA // // NOTE:  AR6410-series send: //   id = 7F //   sID = 0 //   A = 0 //   B = 0 //   L = 0 //   R = 0   //
SPEKPACKED (
typedef struct {
  UINT8 identifier;                                                             // Source device = 0x7F
  UINT8 sID;                                                                    // Secondary ID
  UINT16 A;
  UINT16 B;
  UINT16 L;
  UINT16 R;
  UINT16 F;                                                                     //   F = fades
  UINT16 H;                                                                     //   H = holds
  UINT16 rxVoltage;                                                             // Volts, 0.01V increments rxV = 0xFFFF
}) STRU_TELE_QOS_t;
#endif

//  UNION OF ALL DEVICE MESSAGES //
typedef union {
UINT16  raw[8u];
STRU_TELE_QOS_t qos;
STRU_TELE_RPM_t rpm;
STRU_TELE_FRAMEDATA_t frame;
STRU_TELE_ALT_t alt;
STRU_TELE_SPEED_t speed;
STRU_TELE_ENERGY_DUAL_t eDual;
STRU_TELE_VARIO_S_t varioS;
STRU_TELE_G_METER_t accel;
STRU_TELE_JETCAT_t jetcat;
STRU_TELE_JETCAT2_t jetcat2;
STRU_TELE_GPS_LOC_t gpsloc;
STRU_TELE_GPS_STAT_t gpsstat;
STRU_TELE_GYRO_t gyro;
STRU_TELE_ATTMAG_t attMag;
STRU_TELE_POWERBOX_t powerBox;
STRU_TELE_ESC_t escGeneric;
STRU_TELE_FUEL_t fuel;
STRU_TELE_MAH_t mAh;
STRU_TELE_DIGITAL_AIR_t digAir;
STRU_TELE_STRAIN_t strain;
STRU_TELE_LIPOMON_t lipomon;
STRU_TELE_USER_16SU_t user_16SU;
STRU_TELE_USER54_t user54;        // STRU_TELE_USER54_16U32SU_t
STRU_TELE_USER56_t user56;
STRU_TELE_USER52_t user52;
} UN_TELEMETRY_t;                                                               // All telemetry messages

typedef enum
{
  SBUS_RCV_MSG_IDLE,
  SBUS_RCV_MSG_INCOMING,
  SBUS_MSG_RECEIVED,
  SBUS_RCV_MSG_PROCESS
} Sbus_Rcv_State_t;

//############################################################################
// --------------- Function definitions -------------------------------------
//############################################################################
void SbusParser(unsigned char udr, uint8_t  *ptr, unsigned char sBusBuffer[25u], uint8_t *state, uint8_t siz );
void ProcessSBus(uint8_t *SenderOkay, uint8_t *NewPpmData, uint8_t *NewSBusData, int16_t * PPM_diff[MAX_RC_IN], int16_t * PPM_in[PPM_IN_MAX], uint8_t *sBusBuffer[25u], uint8_t *state);
void MlinkParser(unsigned char udr, uint8_t *SpektrumTimer, uint8_t *MlinkData[MAX_LEN_MLINK], uint8_t *NewMlinkData, int8_t *state, int8_t *lenght );
void s_update(uint8_t channel, int16_t value, int16_t *PPM_diff[MAX_RC_IN], int16_t *PPM_in[PPM_IN_MAX], uint8_t SenderOkay );
void ProcessMlinkData(uint8_t *NewMlinkData, uint8_t *NewPpmData, uint8_t *MlinkData[MAX_LEN_MLINK], int16_t * PPM_diff[MAX_RC_IN], int16_t * PPM_in[PPM_IN_MAX], uint8_t *SenderOkay, uint8_t *Channels );
void SpekbusProcess( uint8_t *state, uint8_t siz, unsigned char sBusBuffer[25u], STRU_TELE_LIPOMON_t *lipo, STRU_TELE_G_METER_t *gforce );

//############################################################################
// Is called by the uart RX interrupt
//############################################################################
 /*-----------------------------------------------------------------------------
 *      SbusParser : sbus parser
 *                       
 *
 *  Parameters: unsigned char udr, uint8_t  *ptr, uint8_t *sBusBuffer[25u],   
 *              uint8_t *state, uint8_t siz 
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void SbusParser(unsigned char udr, uint8_t  *ptr, uint8_t *sBusBuffer[25u], uint8_t *state, uint8_t siz )
{
  if ((*state == SBUS_RCV_MSG_PROCESS) || (*state == SBUS_MSG_RECEIVED))
  {
     /* consider disable interupt during process or store data elsewhere ? */
  }
  else
  {
        if (udr == 0x0fu)                                                       // wait for the start
        {
                *ptr = 0;
        }
        else
        {
                if((*ptr=++*ptr) == siz)                                        // last byte
                {
                     *state = SBUS_MSG_RECEIVED;
                     asm "DI";                                                  // disable interrupt until processed
                }
                else
                {
                     *state = SBUS_MSG_INCOMING;
                     *sBusBuffer[*ptr] = udr;                                   // collect all bytes                        
                }                                        
        }
   }                
}
/*#######################################################################################
Decodes the sbus protocol
#######################################################################################*/
 /*-----------------------------------------------------------------------------
 *      ProcessSBus : process sbus
 *                       
 *
 *  Parameters: uint8_t *SenderOkay, uint8_t *NewPpmData, uint8_t *NewSBusData,  
 *              int16_t * PPM_diff[MAX_RC_IN], int16_t * PPM_in[PPM_IN_MAX], 
 *              uint8_t *sBusBuffer[25u], uint8_t *state
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void ProcessSBus(uint8_t *SenderOkay, uint8_t *NewPpmData, uint8_t *NewSBusData, int16_t * PPM_diff[MAX_RC_IN], int16_t * PPM_in[PPM_IN_MAX], uint8_t *sBusBuffer[25u], uint8_t *state)
{
 uint8_t load = 0, bitmask8 = 1, sbyte = 2, i, index = 1, process;
 uint16_t bitmask11 = 256;
 int16_t signal = 0,tmp;

 if (*state != SBUS_MSG_RECEIVED) return;
 else
        *state = SBUS_RCV_MSG_PROCESS;

 if(!(sBusBuffer[23u] & 4u))                                                         // This Bit contains the 'Signal loss'
 {
    /*TIMSK1 &= ~_BV(ICIE1); // disable PPM-Input
    if(EE_Parameter.FailsafeChannel == 0 || PPM_in[EE_Parameter.FailsafeChannel] < 100)  // forces Failsafe if the receiver doesn't have 'signal loss' on Failsafe
    {
     if(SenderOkay < 200) SenderOkay += 20; else SenderOkay = 200;
    }*/ 
    signal = sBusBuffer[1u];
    if(!load--) { process = (16*11 - 8); load = 2;} else process = (4*11 - 8);  // lowers the processor load 
    for(i = 0; i < process; i++)                                                // collect the single bits
    {
        if(sBusBuffer[sbyte] & bitmask8) signal |= bitmask11;
        bitmask8 *= 2u; 
        if(!bitmask8)
        {
             bitmask8 = 1u;
            sbyte++;
        }
        bitmask11 *= 2u;
        if(bitmask11 == 2048u)
        {
           bitmask11 = 1;
           signal = (signal-1024) / 5;                                          // the resolution is higher than required
           tmp = (3 * (*PPM_in[index]) + signal) / 4; 
           if(tmp > signal+1) tmp--; else
           if(tmp < signal-1) tmp++;
           if(*SenderOkay >= 195)  *PPM_diff[index] = ((tmp - *PPM_in[index]) / 3) * 3;
           else *PPM_diff[index] = 0;
           *PPM_in[index] = tmp;
           signal = 0;
           index++;                                                             // next channel
        }
    }
    *NewPpmData = 0;                                                            // Null bedeutet: Neue Daten
 }
 *NewSBusData = 0;
 *state = SBUS_RCV_MSG_IDLE;
} 

#if defined(USE_MLINK)
//############################################################################
// MULTIPLEX Servo protocol SRXL16 & SRXL12 
//############################################################################
#define MAX_LEN_MLINK 36u
#define MAX_RC_IN 6+12+3+4                                                      // 16 chan 12 ser 3 stage 4 reserved
#define PPM_IN_MAX 31

//############################################################################
// Is called by the uart RX interrupt
// UDR contains the received byte which U(N)RXREG (N)=port number
// The top level caller decrements the timer to timeout the messsage 
//############################################################################
 /*-----------------------------------------------------------------------------
 *      MlinkParser : mlink parser
 *                       
 *
 *  Parameters: unsigned char udr, uint8_t *SpektrumTimer, uint8_t *MlinkData[MAX_LEN_MLINK], 
 *              uint8_t *NewMlinkData, int8_t *state, int8_t *lenght
 *  Return:     void
 *----------------------------------------------------------------------------*/
void MlinkParser(unsigned char udr, uint8_t *SpektrumTimer, uint8_t *MlinkData[MAX_LEN_MLINK], uint8_t *NewMlinkData, int8_t *state, int8_t *lenght )
{
 if(!*SpektrumTimer)                                                            // Timeout-> block finished  (counted down outside routine or intialised)
 {
    if(udr == 0xA1u) *lenght = 12 * 2 + 2;                                      // 12 channels plus CRC
    else if(udr == 0xA2u) *lenght = 16 * 2 + 2;                                 // 16 channels plus CRC
    else *lenght = -1;
   *state = 0;                                                                  // ready to receive a new message
 }
 else
 {
    if(*state < *lenght) 
    {
         *MlinkData[*state++] = udr;
         if(*state == *lenght)                                                  // last Byte received
         {
            *NewMlinkData = *lenght - 2;                                        // without CRC
            *lenght = 0;
         }
     } 
  }
  if (*lenght > 0) *SpektrumTimer = 100u;                                       // when new message came 10ms Timeout resets so long as we get a char on interrupt
  else *SpektrumTimer = 0u;                                                     // look for a new start
}
 /*-----------------------------------------------------------------------------
 *      s_update : Channel-Diff numbercrunching and finally assign new stickvalue to PPM_in
 *                       
 *
 *  Parameters: uint8_t channel, int16_t value, int16_t *PPM_diff[MAX_RC_IN],  
 *              int16_t *PPM_in[PPM_IN_MAX], uint8_t SenderOkay
 *  Return:     void
 *----------------------------------------------------------------------------*/
void s_update(uint8_t channel, int16_t value, int16_t *PPM_diff[MAX_RC_IN], int16_t *PPM_in[PPM_IN_MAX], uint8_t SenderOkay )  // Channel-Diff numbercrunching and finally assign new stickvalue to PPM_in
{
        if (SenderOkay >= 180) *PPM_diff[channel] = ((value - *PPM_in[channel]) / 3) * 3;
        else *PPM_diff[channel] = 0;
        *PPM_in[channel] = value;
}
 /*-----------------------------------------------------------------------------
 *      ProcessMlinkData : process mlink
 *                       
 *
 *  Parameters: uint8_t *NewMlinkData, uint8_t *NewPpmData, uint8_t *MlinkData[MAX_LEN_MLINK],   
 *              int16_t * PPM_diff[MAX_RC_IN], int16_t * PPM_in[PPM_IN_MAX], 
 *              uint8_t *SenderOkay, uint8_t *Channels
 *  Return:     void
 *----------------------------------------------------------------------------*/
void ProcessMlinkData(uint8_t *NewMlinkData, uint8_t *NewPpmData, uint8_t *MlinkData[MAX_LEN_MLINK], int16_t * PPM_diff[MAX_RC_IN], int16_t * PPM_in[PPM_IN_MAX], uint8_t *SenderOkay, uint8_t *Channels )
{
    uint8_t i = 0;
    uint16_t tmp;
    while(i < *NewMlinkData)
    {
         tmp = (uint16_t) *MlinkData[i] * 256 + *MlinkData[i + 1];
         tmp /= 13;
         i += 2;
         s_update(i/2, (int16_t) tmp - 156, PPM_diff, PPM_in, *SenderOkay );     // copies the values into the Channel-Data and calculates the PPM_Diff
         *SenderOkay = 220;  
    }
   *Channels = i/2 + 1;
   *NewPpmData = 0;                                                             // Null bedeutet: Neue Daten
   *NewMlinkData = 0;
}
#endif  /* ============= eNd MLINK ========== */
 /*-----------------------------------------------------------------------------
 *      SpekbusProcess : example spekbus processor (e.g. battry monitor and force meter)
 *                       
 *
 *  Parameters: uint8_t *state, uint8_t siz, unsigned char sBusBuffer[25u],   
 *              STRU_TELE_LIPOMON_t *lipo, STRU_TELE_G_METER_t *gforce  
 *              
 *  Return:     void
 *----------------------------------------------------------------------------*/
void SpekbusProcess( uint8_t *state, uint8_t siz, unsigned char sBusBuffer[25u], STRU_TELE_LIPOMON_t *lipo, STRU_TELE_G_METER_t *gforce )
{
  if (*state == SBUS_MSG_RECEIVED)
  {
     *state = SBUS_RCV_MSG_PROCESS;
     if (siz == sizeof(STRU_TELE_LIPOMON_t))
     {
        memcpy((void*)lipo,(void*)&sBusBuffer,siz);
     }
     else if (siz == sizeof(STRU_TELE_G_METER_t))
     {
        memcpy((void*)gforce,(void*)&sBusBuffer,siz);
     }
     *state = SBUS_RCV_MSG_IDLE;     
     asm "EI";                                                                     // re-enable interrupt for another message to come in
  }
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif   /* ============= eNd SPEKTRUM ========== */