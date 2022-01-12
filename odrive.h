#ifndef __odrive_h
#define __odrive_h

#ifdef __cplusplus
extern "C"
{
#endif

// Odrive communication interface
// Writen : Copyright (C) 2020 A C P Avaiation Walkerburn Scotland

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define ODRIVEPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define ODRIVEPACKED __attribute__((packed))                                     /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define ODRIVEPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define ODRIVEPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define ODRIVEPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

// If you have a realtime controller that is streaming setpoints and tracking a trajectory, use the p command.
// p for position
// motor is the motor number, 0 or 1.
// position is the desired position, in encoder counts.
// velocity_ff is the velocity feed-forward term, in counts/s (optional).
// current_ff is the current feed-forward term, in A (optional).
#define ODRIVE_API_SetPositionStr(X, i_motor_number, f_position, f_velocity_feedforward, f_current_feedforward )  { sprintf(X,"p %d %f %f %f\n",i_motor_number,f_position, f_velocity_feedforward, f_current_feedforward); }
// For basic use where you send one setpoint at at a time, use the q command
// q for position
// motor is the motor number, 0 or 1.
// position is the desired position, in encoder counts.
// velocity_lim is the velocity limit, in counts/s (optional).
// current_lim is the current limit, in A (optional).
#define ODRIVE_API_SetPositionBasic(X, i_motor_number, f_position, f_velocity_lim, f_current_lim )  { sprintf(X,"q %d %f %f %f\n",i_motor_number,f_position, f_velocity_lim, f_current_lim); }

// v for velocity
// motor is the motor number, 0 or 1.
// velocity is the desired velocity in counts/s.
// current_ff is the current feed-forward term, in A (optional).
// Example: v 0 1000 0
// Note that if you don’t know what feed-forward is or what it’s used for, simply omit it.
#define ODRIVE_API_SetVelocity(X, i_motor_number, f_velocity, f_current_feedforward )  { sprintf(X,"v %d %f %f\n",i_motor_number, f_velocity, f_current_feedforward); }

// Motor Current command
// c motor current
// c for current
// motor is the motor number, 0 or 1.
// current is the desired current in A.
// This command updates the watchdog timer for the motor.
#define ODRIVE_API_SetCurrent(X, i_motor_number, f_current )  { sprintf(X,"c %d %f\n",i_motor_number, f_current); }

// f_position is the goal position, in encoder counts.
#define ODRIVE_API_TrapezoidalMove(X, i_motor_number, f_position )  { sprintf(X,"t %d %f\n",i_motor_number, f_position); }

// set gains
#define ODRIVE_API_SetGains(X,kp_t,kd_t,kp_g,kd_g) { sprintf(X,"Set gains to: %f %f %f %f \n", kp_t, kd_t, kp_g, kd_g); }
// set current limit
#define ODRIVE_API_SetCurLim(X,mtr_number,c_lim) { sprintf(X,"w axis%d.motor.config.current_lim  %f \n",mtr_number,c_lim); }
// get velocity
#define ODRIVE_API_GetVelocity(X, i_motor_number)  { sprintf(X,"r axis %d .encoder.vel_estimate\n",i_motor_number); }   // reply is a single float32_t
// get motor state
#define ODRIVE_API_GetState(X, i_axis)  { sprintf(X,"w axis %d .requested_state \n",i_axis); }   // reply is like "r axis i_axis .current_state\n"
// get vbus volts
#define ODRIVE_API_GetVBusVolts(X) { strcpy(X,"r vbus_voltage\n"); }
// set property
#define ODRIVE_API_SetProperty(X, prop, val)  { sprintf(X,"w %s %f\n",prop,val); }
// get property
#define ODRIVE_API_GetProperty(X, prop)  { sprintf(X,"r %s n",prop); }

enum AxisState_t {
    AXIS_STATE_UNDEFINED = 0U,                                                  //<! will fall through to idle
    AXIS_STATE_IDLE = 1U,                                                       //<! disable PWM and do nothing
    AXIS_STATE_STARTUP_SEQUENCE = 2U,                                           //<! the actual sequence is defined by the config.startup_... flags
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3U,                                  //<! run all calibration procedures, then idle
    AXIS_STATE_MOTOR_CALIBRATION = 4U,                                          //<! run motor calibration
    AXIS_STATE_SENSORLESS_CONTROL = 5U,                                         //<! run sensorless control
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6U,                                       //<! run encoder index search
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7U,                                 //<! run encoder offset calibration
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8U                                         //<! run closed loop control
};
#define arSz(array) (sizeof(array) / sizeof((array)[0]))

/* Stream format
   The stream based format is just a wrapper for the packet format. */
#if defined(D_FT900)
typedef struct ODRIVEPACKED {
      uint8_t Stx;                                                              // STX 0xAA
      uint8_t Length;                                                           // length
      uint8_t CRC8;                                                             // CRC8 of above
      uint16_t DataV[];                                                         // Data Stream
      uint16_t checksum;                                                        // checksum 16 bit
} ODRIVE_SndPkt_t;                                                              // Odrive output object
#else
ODRIVEPACKED(
typedef struct {
      uint8_t Stx;                                                              // STX 0xAA
      uint8_t Length;                                                           // length
      uint8_t CRC8;                                                             // CRC8 of above
      uint16_t DataV[];                                                         // Data Stream
      uint16_t checksum;                                                        // checksum 16 bit
}) ODRIVE_SndPkt_t;                                                             // Odrive output object
#endif  

#define ODRIVE_STX 0xAAu
#define MAX_ODRIVE_MSG_LEN 32u

#ifdef __cplusplus
}
#endif

#endif