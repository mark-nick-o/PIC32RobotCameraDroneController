#ifndef _CAPACITY_H
#define _CAPACITY_H

#include <stdlib.h>
#include "definitions.h"                                                        // global defines

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define MK_CAP_PACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define MK_CAP_PACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define MK_CAP_PACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define MK_CAP_PACKED
#else                                                                           /* for MPLAB PIC32 */
  #define MK_CAP_PACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define TWI_STATE_MOTOR_TX 0u
#define TWI_STATE_MOTOR_RX 5u
//#define TWI_STATE_GYRO_OFFSET_TX	18

/*
extern volatile uint8_t twi_state, ReadBlSize;
extern volatile uint8_t motor_write;
extern volatile uint8_t motor_read;
extern volatile uint8_t I2C_TransferActive;
extern uint8_t Max_I2C_Packets;
extern uint8_t MissingMotor;
*/

#define MAX_MOTORS 16u
#define MOTOR_STATE_PRESENT_MASK 0x80u
#define MOTOR_STATE_ERROR_MASK	0x7Fu

// Motor[x].Version
#define MOTOR_STATE_NEW_PROTOCOL_MASK 0x01u
#define MOTOR_STATE_FAST_MODE 0x02u
#define MOTOR_STATE_BL30 0x04u                                                  // extended Current measurement -> 200 = 20A    201 = 21A    255 = 75A (20+55)

#define BLFLAG_TX_COMPLETE 0x01u
#define BLFLAG_READ_VERSION 0x02u

/* 
extern volatile uint8_t BLFlags; 
*/

#define BL_READMODE_STATUS 0u
#define BL_READMODE_CONFIG 16u

#if defined(D_FT900)
typedef struct MK_CAP_PACKED {
	uint8_t Version;			                                // the version of the BL (0 = old)
	uint8_t SetPoint; 			                                // written by attitude controller
	uint8_t SetPointLowerBits;	                                        // for higher Resolution of new BLs
	uint8_t State;    			                                // 7 bit for I2C error counter, highest bit indicates if motor is present
	uint8_t ReadMode;			                                // select data to read
	                                                                        // the following bytes must be exactly in that order!
	uint8_t Current;  			                                // in 0.1 A steps, read back from BL
	uint8_t MaxPWM;   			                                // read back from BL -> is less than 255 if BL is in current limit, not running (250) or starting (40)
	uint8_t Temperature;		                                        // old BL-Ctrl will return a 255 here, the new version the temp. in °C
	uint8_t RPM;				                                // Raw value for RPM
	uint8_t reserved1;			                                // Voltage (BL3) or mAh (BL2)
	uint8_t Voltage;			                                // in 0.1V (BL3 is limited to 255, BL2 is only low-byte)
	uint8_t SlaveI2cError;		                                        // BL2 & BL3
	uint8_t VersionMajor;		                                        // BL2 & BL3
	uint8_t VersionMinor;		                                        // BL2 & BL3
	uint8_t NotReadyCnt; 		                                        // Counts up is the Motor is not ready during flight -> MotorRestart etc.
} MotorData_t;
#else
MK_CAP_PACKED(
typedef struct  {
	uint8_t Version;			                                // the version of the BL (0 = old)
	uint8_t SetPoint; 			                                // written by attitude controller
	uint8_t SetPointLowerBits;	                                        // for higher Resolution of new BLs
	uint8_t State;    			                                // 7 bit for I2C error counter, highest bit indicates if motor is present
	uint8_t ReadMode;			                                // select data to read
	                                                                        // the following bytes must be exactly in that order!
	uint8_t Current;  			                                // in 0.1 A steps, read back from BL
	uint8_t MaxPWM;   			                                // read back from BL -> is less than 255 if BL is in current limit, not running (250) or starting (40)
	uint8_t Temperature;		                                        // old BL-Ctrl will return a 255 here, the new version the temp. in °C
	uint8_t RPM;				                                // Raw value for RPM
	uint8_t reserved1;			                                // Voltage (BL3) or mAh (BL2)
	uint8_t Voltage;			                                // in 0.1V (BL3 is limited to 255, BL2 is only low-byte)
	uint8_t SlaveI2cError;		                                        // BL2 & BL3
	uint8_t VersionMajor;		                                        // BL2 & BL3
	uint8_t VersionMinor;		                                        // BL2 & BL3
	uint8_t NotReadyCnt; 		                                        // Counts up is the Motor is not ready during flight -> MotorRestart etc.
}) MotorData_t;                                                                
#endif

extern MotorData_t Motor[MAX_MOTORS];                                           // global structure contining the Motor data

#if defined(D_FT900)
typedef struct MK_CAP_PACKED {
   uint16_t ActualCurrent;                                                      // in 0.1A Steps
   uint16_t ActualPower;                                                        // in 0.1W
   uint16_t UsedCapacity;                                                       // in mAh
   uint16_t MinOfMaxPWM;	                                                // BL Power Limit
} MkCapacity_t;
#else
MK_CAP_PACKED(
typedef struct  {
   uint16_t ActualCurrent;                                                      // in 0.1A Steps
   uint16_t ActualPower;                                                        // in 0.1W
   uint16_t UsedCapacity;                                                       // in mAh
   uint16_t MinOfMaxPWM;	                                                // BL Power Limit
}) MkCapacity_t;                                                                
#endif

#define STICK_GAIN 4u
#define ACC_AMPLIFY 6u
#define HEIGHT_CONTROL_STICKTHRESHOLD 15u
#define SERVO_FS_TIME 10u                                                       // in Seconds

// FC_StatusFlags
#define FC_STATUS_MOTOR_RUN 0x01u
#define FC_STATUS_FLY 0x02u
#define FC_STATUS_CALIBRATE 0x04u
#define FC_STATUS_START 0x08u
#define FC_STATUS_EMERGENCY_LANDING 0x10u
#define FC_STATUS_LOWBAT 0x20u
#define FC_STATUS_VARIO_TRIM_UP 0x40u
#define FC_STATUS_VARIO_TRIM_DOWN 0x80u

// FC_StatusFlags2 
#define FC_STATUS2_CAREFREE 0x01u
#define FC_STATUS2_ALTITUDE_CONTROL 0x02u
#define FC_STATUS2_RC_FAILSAVE_ACTIVE 0x04u
#define FC_STATUS2_OUT1_ACTIVE 0x08u
#define FC_STATUS2_OUT2_ACTIVE 0x10u
#define FC_STATUS2_WAIT_FOR_TAKEOFF 0x20u                                       // Motor Running, but still on the ground
#define FC_STATUS2_AUTO_STARTING 0x40u
#define FC_STATUS2_AUTO_LANDING 0x80u

// FC_StatusFlags3 
#define FC_STATUS3_REDUNDANCE_AKTIVE 0x01u
#define FC_STATUS3_BOAT	0x02u
#define FC_STATUS3_REDUNDANCE_ERROR 0x04u
#define FC_STATUS3_REDUNDANCE_TEST 0x08u

//NC_To_FC_Flags
#define NC_TO_FC_FLYING_RANGE  0x01u
#define NC_TO_FC_EMERGENCY_LANDING 0x02u                                        // Forces a landing
#define NC_TO_FC_AUTOSTART 0x04u
#define NC_TO_FC_FAILSAFE_LANDING 0x08u                                         // moves Servos into FS-Position
#define NC_TO_FC_SIMULATION_ACTIVE 0x10u                                        // don't start motors

extern uint8_t FC_StatusFlags3;
extern MkCapacity_t Capacity;
extern uint32_t UBat;                                                           // TODO:: needs to read in via Analog input

void Capacity_Init(void);
void Capacity_Update(void);

#ifdef __cplusplus
}
#endif

#endif //_CAPACITY_H