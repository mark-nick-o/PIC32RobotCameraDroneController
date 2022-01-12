#ifndef Pid_h
#define Pid_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//    pid.h : P_roportional I_ntergral D_erivative control
//
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#include <stdint.h>                                                             // Interger type defines
#include "definitions.h"
#include "filter_types.h"
//#include "io.h"

typedef uint32_t time_t;

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define PIDPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define PIDPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define PIDPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define PIDPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define PIDPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define PID_NONE 0U
#define PID_POSITION 1U
#define PID_SPEED 2U

//#define INT32_MAX (long)0x7fffffff
//#define INT32_MIN (long)0x80000000

#define PID_TUNE_STATE_IDLE   0U                                                /* PID tuning state machine for gimbal */
#define PID_TUNE_STATE_START  1U
#define PID_TUNE_STATE_WAIT   2U
#define PID_TUNE_STATE_LOOP   3U
#define PID_TUNE_STATE_END    4U

// auto tune methods  (they are specified in the tuner object as parameters to znMode attribute)
#define ZNModeBasicPID 1U
#define ZNModeLessOvershoot 2U
#define ZNModeNoOvershoot 3U
#define tyreus_luyben 4U
#define ciancone_marlin 5U
#define pessen_integral 6U
#define kettle_brewer 7U
#define PCUnderdampedPID 8U
#define PCCriticaldampedPID 9U
#define PCOverdampedPID 10U
#define BuczOvershootPID 11U
#define BuczSettlePID 12U
#define Chau1PID 13U
#define Chau2PID 14U

#define Viteckova2DOF_P(K,T) (0.784f/(K*T))                                     // 2 DOF PID Ostrava Uni Viteckova and Vitececk 2 degree of freedom controller
#define Viteckova2DOF_T 3.732f
#define Viteckova2DOF_D(T) (0.263f*T)

typedef enum {
  pid_Type_Der1 = 0u,                                                           /* derivative calculation 1 standard */
  pid_Type_Der2,                                                                /* derivative calculation 2 as per calc by (C) 2015 Jesus Ruben Santa Anna Zamudio */
  pid_Type_ArduPiDer3,                                                          /* derivative calculation 3 as per calc by ArduPilot Andy Piper */
  pid_Type_craZyFlie                                                            /* derivative calculation 4 as per calc by craZyFlie team lpf2 filter */
} PIDTypes_e;

// Low pass filter cut frequency for derivative calculation.
//
// 20 Hz because anything over that is probably noise, see
// http://en.wikipedia.org/wiki/Low-pass_filter.
//
#define PID_fCut 20u
#define PID_samplingRate 20u                                                    // for lpf

#if defined(D_FT900)
typedef struct PIDPACKED {

      uint8_t mode_man : 1u;                                                    // bit for manual mode
      uint8_t mode_rem : 1u;                                                    // remote setpoint mode
      uint8_t mode_trk : 1u;                                                    // track mode
      uint8_t mode_hld : 1u;                                                    // hold mode
      uint8_t mode_init : 1u;                                                   // mode initialise
      uint8_t mode_bmp : 1u;                                                    // remote mode bumpless
      uint8_t mode_began : 1u;                                                  // first run of the PID (time initialise)
      uint8_t mode_reverse : 1u;                                                // set true for reverse action

      uint8_t thrust_tilt_comp : 1u;                                            /* pid is controlling thrust with tilt compensation enabled */
      uint8_t spare_bits1 : 7u;
      
      int32_t setpoint;                                                         // local setpoint
      int32_t rem_setpoint;                                                     // remote setpoint
      int32_t feedback;                                                         // feedback (measurement)
      float32_t kp;                                                             // proportional band
      float32_t ki;                                                             // gain
      float32_t kd;                                                             // derivative
      float32_t last_derivative;                                                // used when derivative correction is applied
      float32_t filter_derivative;                                              // calculated filter derivative
      int32_t lastError;                                                        // last error
      uint64_t lasttime;                                                        // time time it ran
      uint32_t output;                                                          // calculated pid output as uint32_t
      float32_t fout;                                                           // calculated pid output as float32_t
      uint32_t outMax;                                                          // maximum allowed pid output
      uint32_t outMin;                                                          // minimum allowed pid output
      uint32_t initVal;                                                         // feedback for output to start at when init flag is set on
      int32_t lastFeedBack;                                                     // last input value since last time read for creating diffInput
      PIDTypes_e type;                                                          // type of algorythm chosen for this loop controller
      uint8_t cutoffFreq;                                                       // cut off frequency forthe filter typically 20Hz
      lpf2pData dFilter;                                                        // filter for D term when using crazyflie model for derivative compensation
      int32_t integral;                                                         // internal store for the integral term (each PID has its own calcualtion)
      int32_t lastIntegral;                                                     // saves the last known intergral
} pidBlock_t;
#else
PIDPACKED(
typedef struct {

      uint8_t mode_man : 1u;                                                    // bit for manual mode
      uint8_t mode_rem : 1u;                                                    // remote setpoint mode
      uint8_t mode_trk : 1u;                                                    // track mode
      uint8_t mode_hld : 1u;                                                    // hold mode
      uint8_t mode_init : 1u;                                                   // mode initialise
      uint8_t mode_bmp : 1u;                                                    // remote mode bumpless
      uint8_t mode_began : 1u;                                                  // first run of the PID (time initialise)
      uint8_t mode_reverse : 1u;                                                // set true for reverse action

      uint8_t thrust_tilt_comp : 1u;                                            /* pid is controlling thrust with tilt compensation enabled */
      uint8_t spare_bits1 : 7u;
      
      int32_t setpoint;                                                         // local setpoint
      int32_t rem_setpoint;                                                     // remote setpoint
      int32_t feedback;                                                         // feedback (measurement)
      float32_t kp;                                                             // proportional band
      float32_t ki;                                                             // gain
      float32_t kd;                                                             // derivative
      float32_t last_derivative;                                                // used when derivative correction is applied
      float32_t filter_derivative;                                              // calculated filter derivative
      int32_t lastError;                                                        // last error
      uint64_t lasttime;                                                        // time time it ran
      uint32_t output;                                                          // calculated pid output as uint32_t
      float32_t fout;                                                           // calculated pid output as float32_t
      uint32_t outMax;                                                          // maximum allowed pid output
      uint32_t outMin;                                                          // minimum allowed pid output
      uint32_t initVal;                                                         // feedback for output to start at when init flag is set on
      int32_t lastFeedBack;                                                     // last input value since last time read for creating diffInput
      PIDTypes_e type;                                                          // type of algorythm chosen for this loop controller
      uint8_t cutoffFreq;                                                       // cut off frequency forthe filter typically 20Hz
      lpf2pData dFilter;                                                        // filter for D term when using crazyflie model for derivative compensation
      int32_t integral;                                                         // internal store for the integral term (each PID has its own calcualtion)
      int32_t lastIntegral;                                                     // saves the last known intergral
}) pidBlock_t;
#endif

#if defined(D_FT900)
typedef struct PIDPACKED {
        uint64_t timeLast;                                                      // Timestamp when the data was computed
        int64_t Current_time;                                                   // current time elapsed
        float64_t noiseBand;                                                    // only the |SP - PV| upper this threhold, the relay output change 
        int8_t controlType;                                                     // 1 : PID  0: PI
        int16_t nLookBack;                                                      // must > 3 
        float64_t oStep;                                                        // PWM for Output step, Output = mean + step  or Output = mean - step
        float64_t outputStart;                                                  //  mean value for Output
        float64_t *input;       
        float64_t *output;
        float64_t *setpoint;                                                    // PV  CV SP                
        uint64_t peak1;
        uint64_t peak2;
        uint16_t lastTimeAuto;                                                  // peak Value (upper and lower)                
        uint16_t sampleTime;                                                    // Calculate periods
        int16_t peakType;                                                       // upper or lower mark
        float64_t lastInputs[10u];                
        float64_t peaks[10u];                
        int16_t peakCount;                                                      // peaks count
        uint8_t isMax : 1u;
        uint8_t isMin : 1u;                                                     // mark for judge peak or not
        uint8_t running : 1u;                                                   // mark for auto process is running
        uint8_t justchanged : 1u;        
        uint8_t justevaled : 1u;                                                // mark for auto end
        uint8_t sparebits : 3u;
        float64_t absMax;
        float64_t absMin;                                                       // peak        Value absolute
        float64_t Ku;
        float64_t Pu;
} pideAutoTuner_t;
#else
PIDPACKED(
typedef struct {
        uint64_t timeLast;                                                      // Timestamp when the data was computed
        int64_t Current_time;                                                   // current time elapsed
        float64_t noiseBand;                                                    // only the |SP - PV| upper this threhold, the relay output change 
        int8_t controlType;                                                     // 1 : PID  0: PI
        int16_t nLookBack;                                                      // must > 3 
        float64_t oStep;                                                        // PWM for Output step, Output = mean + step  or Output = mean - step
        float64_t outputStart;                                                  //  mean value for Output
        float64_t *input;       
        float64_t *output;
        float64_t *setpoint;                                                    // PV  CV SP                
        uint64_t peak1;
        uint64_t peak2;
        uint16_t lastTimeAuto;                                                  // peak Value (upper and lower)                
        uint16_t sampleTime;                                                    // Calculate periods
        int16_t peakType;                                                       // upper or lower mark
        float64_t lastInputs[10u];                
        float64_t peaks[10u];                
        int16_t peakCount;                                                      // peaks count
        uint8_t isMax : 1u;
        uint8_t isMin : 1u;                                                     // mark for judge peak or not
        uint8_t running : 1u;                                                   // mark for auto process is running
        uint8_t justchanged : 1u;        
        uint8_t justevaled : 1u;                                                // mark for auto end
        uint8_t sparebits : 3u;
        float64_t absMax;
        float64_t absMin;                                                       // peak        Value absolute
        float64_t Ku;
        float64_t Pu;
}) pideAutoTuner_t;                                                             // from JiangTingjia(kyzy_duck@163.com)
#endif

typedef enum {
        SPRAY_NO_ACTION = 0u,
        SPRAY_SET_ON,
        SPRAY_SET_OFF,
        SPRAY_GO_DOWN,
        SPRAY_GO_UP,
        SPRAY_SET_VAL,
        SPRAY_RESET,
        NUMBER_OF_SPRAY_STATES
} pidSprayerStateClass;                                                         // defines the states of the pid sprayer

typedef enum {
        SPRAY_REMOTE_GPS = 0u,                                                  // default control for the sprayer
        SPRAY_GUI_CONNECTED,                                                    // values (setpoints) from the hmi / gui
        SPRAY_PUSHBTN_POT,                                                      // physical pushbutton and potentiometer panel interface
        SPRAY_JSON_CONNECT_IN,                                                  // java app communication json object
        SPRAY_COAP_CONNECT_IN,                                                  // m2m app with cbor / coAp
        SPRAY_UBIBOT_IN,                                                        // moisture / humidity / pH auto control
        SPRAY_MODBUS_CONNECT_IN,                                                // driven over modbus inputs (controls)
        SPRAY_GEC_FANUC_CONNECT_IN,                                             // from GEC Fanuc
        SPRAY_HTTP_MESSAGE,                                                     // webservice remote
        NUMBER_OF_SPRAY_INPUT_TYPES
} pidInputRouteClass;                                                           // defines the possible input method to the sprayer object setpoint

typedef enum {
        SPRAY_PWM_OUTPUT = 0u,                                                  // default control for the sprayer pulse width modulation
        SPRAY_AOT_OUTPUT,                                                       // voltage / mA output
        SPRAY_UBIBOT_OUT,                                                       // moisture / humidity / pH auto control
        SPRAY_MODBUS_CONNECT_OUT,                                               // driven over modbus outputs (inverter drive)
        SPRAY_GEC_FANUC_CONNECT_OUT,                                            // from GEC Fanuc
        SPRAY_ODRIVE_MESSAGE_OUT,                                               // odrive gait drive
        SPRAY_COAP_CONNECT_OUT,                                                 // m2m app with cbor / coAp
        LIGHT_ASN_CONNECT_OUT,                                                  // light controls on ASN / dmx512
        LIGHT_ARTNET_CONNECT_OUT,                                               // lights lasers etc on artnet / dmx512
        LIGHT_DALI_CONNECT_OUT,                                                 // lights on dali interface
        NUMBER_OF_SPRAY_OUTPUT_TYPES
} pidOutputRouteClass;                                                          // defines the possible output method to the pid object output


#if defined(D_FT900)
typedef struct PIDPACKED {
     int32_t pidValue;                                                          // hmi manual setpoint or output to the pid controller
     pidSprayerStateClass sysState:8u;                                          // requested system state for the PID
     pidInputRouteClass sourceToPid:8u;                                         // source type to the PID sprayer setpoints
     pidOutputRouteClass sourceFromPid:8u;                                      // source from the PID
} interfacePIDObj_t;
#else
PIDPACKED(
typedef struct {
     int32_t pidValue;                                                          // hmi manual setpoint or output to the pid controller
     pidSprayerStateClass sysState:8u;                                          // requested system state for the PID
     pidInputRouteClass sourceToPid:8u;                                         // source type to the PID sprayer setpoints
     pidOutputRouteClass sourceFromPid:8u;                                      // source from the PID
}) interfacePIDObj_t;
#endif

#if defined(D_FT900)
typedef struct PIDPACKED {
     uint8_t init : 1u;                                                         // init = false, init done = true
     uint8_t znMode : 7u;                                                       // tuner object mode
     uint8_t cycles;                                                            // number of iterative cycles to do
     float32_t targetInputValue;                                                // target value
} pidTuner_t;
#else
PIDPACKED(
typedef struct {
     uint8_t init : 1u;                                                         // init = false, init done = true
     uint8_t znMode : 7u;                                                       // tuner object mode
     uint8_t cycles;                                                            // number of iterative cycles to do
     float32_t targetInputValue;                                                // target value
}) pidTuner_t;
#endif

    PIDTypes_e PIDtype;
    int16_t scale;
    int32_t integral;
    int32_t lastError;
    int32_t integralMax=100u;
    int32_t integralMin=0u;
    int32_t iTermMax;
    int32_t iTermMin;
    int32_t pwmPeriod;
    time_t sampleLastTime;
    int8_t lastDir;
    int32_t lastWidth;

    /* tuning parameter */
    uint8_t tuneState;
    int16_t tuneInput;
    int16_t lastTuneInput;
    uint16_t tuneBaseInput;
    uint16_t tuneStep;
    int32_t tuneSetpoint;
    int32_t tuneLastFeedback;
    int8_t tuneDirChange;                                                       //0: initial, 1: go up, -1 go down
    int32_t tuneMaxSum;
    int32_t tuneMinSum;
    uint8_t tuneMaxCount;
    uint8_t tuneMinCount;
    uint8_t tuneNoiseband;
    time_t tuneStartTime;
    time_t tuneEndTime;
    time_t tuneWaitTime;
    
    uint8_t computePID( pidBlock_t *pidObj );                                   // the PID routine

#define ATTITUDE_UPDATE_RATE 2000UL                                             /* how often we want to update the attitude calculations */
#define POS_UPDATE_RATE 5000UL                                                  /* how often we want to do the position update */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif