#ifndef devIC_PR_TE 
#define devIC_PR_TE 

#include "definitions.h"
#include "gc_events.h"
/*#include "mmc_file_handler.h"*/
#include "spi_chip_sel.h"
#if defined(MPU6050_ACC_GYRO)
#include "mpu6050_acc_gyro.h"                                                   /* i2c registers for reading the accelerometer data from the MPU6050 */
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define DEVICEICPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define DEVICEICPACKED __attribute__((packed))                                 /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define DEVICEICPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define DEVICEICPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define DEVICEICPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#ifndef bool
#define bool uint8_t
#endif

#ifndef devIC_PR_TE
#define devIC_PR_TE static
#endif
#define QMP6988_I2C_ADDR (0x70U)
#define QMP6988_DEFAULT_CHIP_ID        (0x5cU)
#define QMP6988_CHIP_ID_REG (0xD1U)                                             /* Chip ID Register */

#define QMP6988_IO_SETUP_REG (0xF5U)
#define QMP6988_SET_IIR_REG (0xF1U)
#define QMP6988_CTRL_MEAS_REG (0xF4U)
#define QMP6988_COE_B00_1_REG (0xA0U)
#define QMP6988_PRESSURE_MSB_REG (0xF7U)                                        /* Pressure MSB Register */
#define QMP6988_PRESSURE_LSB_REG (0xF8U)                                        /* Pressure LSB Register */
#define QMP6988_PRESSURE_XLSB_REG (0xF9U)                                       /* Pressure XLSB Register */
#define QMP6988_TEMPERATURE_MSB_REG (0xFAU)                                     /* Temperature MSB Reg */
#define QMP6988_TEMPERATURE_LSB_REG (0xFBU)                                     /* Temperature LSB Reg */
#define QMP6988_TEMPERATURE_XLSB_REG (0xFCU)                                    /* Temperature XLSB Reg */
#define QMP6988_DATA_FRAME_SIZE 6U
#define QMP6988_FORCED_MODE (0x01U)
#define QMP6988_PWR_SAMPLE_MODE (0x7BU)

#define QMP6988_OVERSAMP_SKIPPED (0x00U)
#define QMP6988_OVERSAMP_1X (0x01U)
#define QMP6988_OVERSAMP_2X (0x02U)
#define QMP6988_OVERSAMP_4X (0x03U)
#define QMP6988_OVERSAMP_8X (0x04U)
#define QMP6988_OVERSAMP_16X (0x05U)

// configure pressure and temperature oversampling, forced sampling mode
#define QMP6988_PRESSURE_OSR (QMP6988_OVERSAMP_8X)
#define QMP6988_TEMPERATURE_OSR (QMP6988_OVERSAMP_1X)
#define QMP6988_MODE (QMP6988_PRESSURE_OSR << 2 | QMP6988_TEMPERATURE_OSR << 5 | QMP6988_FORCED_MODE)

#define T_INIT_MAX (20U)
#define T_MEASURE_PER_OSRS_MAX (37U)
#define T_SETUP_PRESSURE_MAX (10U)

#define SENSOR_STATE_START_INIT 0u                                                 /* states of the sensor for the barometer */
#define SENSOR_STATE_SET_IIR 1u
#define SENSOR_STATE_READ_OTP 2u
#define SENSOR_STATE_SET_SAMPTIM 3u
#define SENSOR_STATE_START_BARO 4u
#define SENSOR_STATE_GET_BARO 5u
#define SENSOR_STATE_CALC_BARO 6u
#define SENSOR_STATE_CALC_DONE 7u
#define SENSOR_STATE_WRONG_CHIP 8u
#define SENSOR_STATE_CALIB_MIN1 10u
#define SENSOR_STATE_CALIB_MIN2 11u
#define SENSOR_STATE_CALIB_MAX1 12u
#define SENSOR_STATE_CALIB_MAX2 13u

#define FBM320_I2C_ADDR 0xA0u
#define FBM320_REG_ID   0x6Bu
#define FBM320_REG_DATA 0xF6u
#define FBM320_REG_CMD  0xF4u
#define FBM320_CMD_READ_T 0x2Eu
#define FBM320_CMD_READ_P 0xF4u
#define FBM320_WHOAMI 0x42u
#define FBM320_PRESSURE_MSB_REG 0xF6u
#define FBM320_DATA_FRAME_SIZE 3u
#define BARO_NUM_OF_ITERS 5u

// Measurement range registers for Keller
#define KELLER_PRANGE_MIN_MSB 0x13u
#define KELLER_PRANGE_MIN_LSB 0x14u
#define KELLER_PRANGE_MAX_MSB 0x15u
#define KELLER_PRANGE_MAX_LSB 0x16u

// write to this address to start pressure measurement for Keller
#define KELLER_REQUEST_MEASUREMENT 0xACu
#define KELLER_DATA_FRAME_SIZE 5u
#define KELLER_REQUEST_VAL 0x01u

// =================== HMC6352Driver is a Driver for a HMC6352 digital compass
#define HMC6352ADDR 0x21u
#define HMC_DAT_REQ "A"
#define HMC6352_DATA_FRAME_SIZE 2
#define HMC6352_SAMPLED_ENOUGH 64u

// ====== Define Structures to contain the i2c data ============================

   
typedef enum {
    BUSTYPE_NONE = 0,
    BUSTYPE_I2C,
    BUSTYPE_SPI,
    BUSTYPE_MPU_SLAVE                                                           // Slave I2C on SPI master
} busType_e;

typedef enum I2CDevice {
    I2CINVALID = -1,
    I2CDEV_1   = 0,                                                             /* available i2c */
    I2CDEV_2,
    I2CDEV_3,
    I2CDEV_4,
    I2CDEV_5,
} I2CDevice_e;

typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1   = 0,
    SPIDEV_2,
    SPIDEV_3,
    SPIDEV_4
} SPIDevice_e;

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
            I2CDevice_e device;
            uint8_t address;
} deviceI2C_s;
#else
DEVICEICPACKED(
typedef struct {
            I2CDevice_e device;
            uint8_t address;
}) deviceI2C_s;                                                       // qmp6988
#endif

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
            SPIDevice_e device;
            uint8_t address;
} deviceSPI_s;
#else
DEVICEICPACKED(
typedef struct {
            SPIDevice_e device;
            uint8_t address;
}) deviceSPI_s;                                                       // qmp6988
#endif

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    busType_e bustype;
    union {
        deviceI2C_s i2c;
        deviceSPI_s spi;
    } busdev_u;
} busDevice_t;
#else
DEVICEICPACKED(
typedef struct {
    busType_e bustype;
    union {
        deviceI2C_s i2c;
        deviceSPI_s spi;
    } busdev_u;
}) busDevice_t;                                                       // qmp6988
#endif

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    union {
        float32_t f;
        int32_t i;
        uint32_t u;
    } baroFloatConvert_u;
} baroFloatConvert_t;
#else
DEVICEICPACKED(
typedef struct {
    union {
        float32_t f;
        int32_t i;
        uint32_t u;
    } baroFloatConvert_u;
}) baroFloatConvert_t;                                                       // qmp6988
#endif

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    float64_t Coe_a0;
    float64_t Coe_a1;
    float64_t Coe_a2;
    float64_t Coe_b00;
    float64_t Coe_bt1;
    float64_t Coe_bt2;
    float64_t Coe_bp1;
    float64_t Coe_b11;
    float64_t Coe_bp2;
    float64_t Coe_b12;
    float64_t Coe_b21;
    float64_t Coe_bp3;
} qmp6988_calib_param_t;
#else
DEVICEICPACKED(
typedef struct {
    float64_t Coe_a0;
    float64_t Coe_a1;
    float64_t Coe_a2;
    float64_t Coe_b00;
    float64_t Coe_bt1;
    float64_t Coe_bt2;
    float64_t Coe_bp1;
    float64_t Coe_b11;
    float64_t Coe_bp2;
    float64_t Coe_b12;
    float64_t Coe_b21;
    float64_t Coe_bp3;
}) qmp6988_calib_param_t;                                                       // qmp6988
#endif

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    uint32_t C0;
    uint32_t C1;
    uint32_t C2;
    uint32_t C3;
    uint32_t C4;
    uint32_t C5;
    uint32_t C6;
    uint32_t C7;
    uint32_t C8;
    uint32_t C9;
    uint32_t C10;
    uint32_t C11;
    uint32_t C12;
} fbm320_calibration_t;
#else
DEVICEICPACKED(
typedef struct {
    uint32_t C0;
    uint32_t C1;
    uint32_t C2;
    uint32_t C3;
    uint32_t C4;
    uint32_t C5;
    uint32_t C6;
    uint32_t C7;
    uint32_t C8;
    uint32_t C9;
    uint32_t C10;
    uint32_t C11;
    uint32_t C12;
}) fbm320_calibration_t;
#endif

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    union {
       float32_t p_min;
       uint32_t cal_data;
    } kellcal_min;
    union {
       float32_t p_max;
       uint32_t cal_data;
    } kellcal_max;
    uint8_t calIntegrity;
} keller_calibration_t;
#else
DEVICEICPACKED(
typedef struct {
    union {
       float32_t p_min;
       uint32_t cal_data;
    } kellcal_min;
    union {
       float32_t p_max;
       uint32_t cal_data;
    } kellcal_max;
    uint8_t calIntegrity;
}) keller_calibration_t;
#endif

//struct baroDev_s;

//typedef void (*baroOpFuncPtr)(struct baroDev_s *baro);                          // baro start operation
//typedef void (*baroCalculateFuncPtr)(float64_t *pressure, float64_t *temperature); // baro calculation (filled params are pressure and temperature)

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    busDevice_t busdev;
    uint16_t ut_delay;
    uint16_t up_delay;
    uint16_t otp_delay;
    uint16_t sam_delay;
    uint16_t start_delay;
    int32_t dev_up;                                                             /* uncompressed pressure */
    int32_t dev_ut;                                                             /* uncompressed temperature */
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t busState;
    uint8_t busState1;
    float64_t pressure_sum;
    float64_t temperature_sum;
    uint64_t num_of_sums;
} baroDev_t;
#else
DEVICEICPACKED(
typedef struct {
    busDevice_t busdev;
    uint16_t ut_delay;
    uint16_t up_delay;
    uint16_t otp_delay;
    uint16_t sam_delay;
    uint16_t start_delay;
    int32_t dev_up;                                                             /* uncompressed pressure */
    int32_t dev_ut;                                                             /* uncompressed temperature */
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t busState;
    uint8_t busState1;
    float64_t pressure_sum;
    float64_t temperature_sum;
    uint64_t num_of_sums;
}) baroDev_t;
#endif

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    float32_t BaroAlt;
    int32_t baroTemperature;                                                    // Use temperature for telemetry
    int32_t baroPressure;                                                       // Use pressure for telemetry
    float32_t baro_noise_lpf;
    int16_t baro_cf_vel;
    int16_t baro_cf_alt;
    float32_t baroGroundAltitude;
    uint8_t cali_cycles;
    int32_t savedGroundPressure;
} baro_t;
#else
DEVICEICPACKED(
typedef struct {
    float32_t BaroAlt;
    int32_t baroTemperature;                                                    // Use temperature for telemetry
    int32_t baroPressure;                                                       // Use pressure for telemetry
    float32_t baro_noise_lpf;
    int16_t baro_cf_vel;
    int16_t baro_cf_alt;
    float32_t baroGroundAltitude;
    uint8_t cali_cycles;
    int32_t savedGroundPressure;
}) baro_t;
#endif

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    busDevice_t busdev;
    float32_t heading;
    float32_t heading_sum;
    uint32_t num_of_sums;
    float32_t avg_heading;
} compass_t;
#else
DEVICEICPACKED(
typedef struct {
    busDevice_t busdev;
    float32_t heading;
    float32_t heading_sum;
    uint32_t num_of_sums;
    float32_t avg_heading;
}) compass_t;
#endif

/* ========== Lidar Lite =================================================== */
#if defined(LIDAR_LITESTART_USED)

#define LIDDARLITEADDR 0x62u
#define LL_DATA1 0x00u
#define LL_DATA2 0x04u
#define LL_DATA3 0x0Fu
#define LL_DATA4 0x10u
#define LL_DATA_FRAME_SIZE 2u
#define LL_SAMPLED_ENOUGH 64u
#define LL_DELAY1 20000u
#define LL_DELAY2 40000u

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    busDevice_t busdev;
    float32_t distance;
    float32_t distance_sum;
    uint32_t num_of_sums;
    float32_t avg_distance;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
} distance_t;
#else
DEVICEICPACKED(
typedef struct {
    busDevice_t busdev;
    float32_t distance;
    float32_t distance_sum;
    uint32_t num_of_sums;
    float32_t avg_distance;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
}) distance_t;
#endif

#endif /* end lidar litestart */
/* =================== BH1750 Lux Sensor IC on I2C ========================= */
#define BH1750_ADDR 0x23u

#define BH1750_POWER_DOWN 0x00u
#define BH1750_POWER_ON 0x01u
#define BH1750_RESET 0x07u
#define BH1750_CONTINUOUS_HIGH_RES_MODE 0x10u
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2 0x11u
#define BH1750_CONTINUOUS_LOW_RES_MODE 0x13u
#define BH1750_ONE_TIME_HIGH_RES_MODE 0x20u
#define BH1750_ONE_TIME_HIGH_RES_MODE_2 0x21u
#define BH1750_ONE_TIME_LOW_RES_MODE 0x23u
#define BH1750_DATA_FRAME_SIZE 2u
#define BH1750_SAMPLED_ENOUGH 64u
#define BH1750_DELAY1 20000lu
#define BH1750_DELAY2 20000lu

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    busDevice_t busdev;
    float32_t lux;
    float32_t lux_sum;
    uint32_t num_of_sums;
    float32_t avg_lux;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
} lux_t;
#else
DEVICEICPACKED(
typedef struct {
    busDevice_t busdev;
    float32_t lux;
    float32_t lux_sum;
    uint32_t num_of_sums;
    float32_t avg_lux;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
}) lux_t;
#endif

/* ====================== IAQ2000 Co2 monitor chip ========================= */
#if defined(WANT_IAQ2000_Co2)
// Based on AppliedSensor iAQ-2000 Interface Description, Version PA1, 2009
// 2012-04-01 by Peteris Skorovs <pskorovs@gmail.com>
#define IAQ2000_ADDRESS  0x5Au
#define IAQ2000_DEFAULT_ADDRESS IAQ2000_ADDRESS

#define IAQ2000_RA_DATA1 0x00u
#define IAQ2000_RA_DATA2 0x01u
#define IAQ2000_SAMPLED_ENOUGH 64u

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    busDevice_t busdev;
    uint16_t Iaqpred;                                                           /* predicted co2 conc */
    uint32_t Iaqpred_sum;
    uint16_t num_of_sums;
    uint16_t avg_Iaqpred;
    uint8_t Iaqstatus;
    uint16_t Iaqtvoc;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
} iaq2000_t;
#else
DEVICEICPACKED(
typedef struct {
    busDevice_t busdev;
    uint16_t Iaqpred;                                                           /* preedicted co2 conc */
    uint32_t Iaqpred_sum;
    uint16_t num_of_sums;
    uint16_t avg_Iaqpred;
    uint8_t Iaqstatus;
    uint16_t Iaqtvoc;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
}) iaq2000_t;
#endif

#endif        /* end co2 monitor ic2 module */

// I2Cdev library collection - HTU21D I2C device class header file
// Based on MEAS HTU21D HPC199_2 HTU321(F) datasheet, October 2013
// 2016-03-24 by https://github.com/eadf
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2016-03-24 - initial release
//     oct 2000 ported for pic32 
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2016 Eadf, Jeff Rowberg
Ported to mikroE C PIC32/FT900 by ACP Aviation

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#if defined(HTU21D_HUMID_USED)

#define HTU21D_DEFAULT_ADDRESS     0x40u
#define HTU21D_RA_TEMPERATURE      0xE3u
#define HTU21D_RA_HUMIDITY         0xE5u
#define HTU21D_RESET               0xFEu
#define HTU21D_WRITE_USER_REGISTER 0xE6u
#define HTU21D_READ_USER_REGISTER  0xE7u
#define HTU21D_SAMPLED_ENOUGH 32u

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    busDevice_t busdev;
    float32_t temperat;                                                          
    float32_t temperat_sum;
    uint16_t num_of_sums;
    float32_t avg_temperat;
    float32_t humid;                                                          
    float32_t humid_sum;
    uint16_t num_of_sums1;
    float32_t avg_humid;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
} htu21d_t;
#else
DEVICEICPACKED(
typedef struct {
    busDevice_t busdev;
    float32_t temperat;                                                          
    float32_t temperat_sum;
    uint16_t num_of_sums;
    float32_t avg_temperat;
    float32_t humid;                                                          
    float32_t humid_sum;
    uint16_t num_of_sums1;
    float32_t avg_humid;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
}) htu21d_t;
#endif

#endif /* end ht21d */

/* ====================== Power measurement IC ============================= */
// INA3221Driver is a driver for the Texas Instruments INA3221 device. The INA3221 is a three-channel
// current and bus voltage monitor with an I2C and SMBUS compatible interface.
//
// INA3221 data sheet and specifications can be found at http://www.ti.com/product/INA3221
//
// This module was tested with SwitchDoc Labs INA3221 breakout board found at http://www.switchdoc.com/
//
#define        INA3221Address 0x40u                                                    // 1000000 (A0+A1=GND)
#define        INA3221Read 0x01u
#define        INA3221RegConfig 0x00u                                                  // CONFIG REGISTER (R/W)
#define        INA3221ConfigReset 0x8000u                                              // Reset Bit
#define        INA3221ConfigEnableChan1 0x4000u                                        // Enable INA3221 Channel 1
#define        INA3221ConfigEnableChan2 0x2000u                                        // Enable INA3221 Channel 2
#define        INA3221ConfigEnableChan3 0x1000u                                        // Enable INA3221 Channel 3
#define        INA3221ConfigAvg2 0x0800u                                               // AVG Samples Bit 2 - See table 3 spec
#define        INA3221ConfigAvg1 0x0400u                                               // AVG Samples Bit 1 - See table 3 spec
#define        INA3221ConfigAvg0 0x0200u                                               // AVG Samples Bit 0 - See table 3 spec
#define        INA3221ConfigVBusCT2 0x0100u                                            // VBUS bit 2 Conversion time - See table 4 spec
#define        INA3221ConfigVBusCT1 0x0080u                                            // VBUS bit 1 Conversion time - See table 4 spec
#define        INA3221ConfigVBusCT0 0x0040u                                            // VBUS bit 0 Conversion time - See table 4 spec
#define        INA3221ConfigVShCT2 0x0020u                                             // Vshunt bit 2 Conversion time - See table 5 spec
#define        INA3221ConfigVShCT1 0x0010u                                             // Vshunt bit 1 Conversion time - See table 5 spec
#define        INA3221ConfigVShCT0 0x0008u                                             // Vshunt bit 0 Conversion time - See table 5 spec
#define        INA3221ConfigMode2 0x0004u                                              // Operating Mode bit 2 - See table 6 spec
#define        INA3221ConfigMode1 0x0002u                                              // Operating Mode bit 1 - See table 6 spec
#define        INA3221ConfigMode0 0x0001u                                              // Operating Mode bit 0 - See table 6 spec
#define        INA3221RegShuntVoltage1 0x01u                                           // SHUNT VOLTAGE REGISTER (R)
#define        INA3221RegBusVoltage1 0x02u                                             // BUS VOLTAGE REGISTER (R)
#define        INA3221ShuntResistorValue 0.1f                                          // default shunt resistor value of 0.1 Ohm

typedef enum { INA3221Channel1=1, INA3221Channel2=2, INA3221Channel3=3, INA3221_NUM_OF_CHAN } INA3221Channel_e;
#define INA3221_DATA_FRAME_SIZE 4u
#define INA3221_SAMPLED_ENOUGH 64u

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    busDevice_t busdev;
    float32_t mAcurrent;
    float32_t mAcurrent_sum;
    float32_t avg_mAcurrent;
    float32_t avg_mVoltsLoad;
    float32_t mVoltsBus;
    float32_t mVoltsBus_sum;
    float32_t avg_mVoltsBus;
    float32_t mVoltsShunt;
    float32_t mVoltsShunt_sum;
    float32_t avg_mVoltsShunt;
    float32_t peak_current;
    uint32_t num_of_sums;
    uint32_t num_of_sums1;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
} powerIV_IC_t;
#else
DEVICEICPACKED(
typedef struct {
    busDevice_t busdev;
    float32_t mAcurrent;
    float32_t mAcurrent_sum;
    float32_t avg_mAcurrent;
    float32_t avg_mVoltsLoad;
    float32_t mVoltsBus;
    float32_t mVoltsBus_sum;
    float32_t avg_mVoltsBus;
    float32_t mVoltsShunt;
    float32_t mVoltsShunt_sum;
    float32_t avg_mVoltsShunt;
    float32_t peak_current;
    uint32_t num_of_sums;
    uint32_t num_of_sums1;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
}) powerIV_IC_t;
#endif

#if defined(PAW_OPTICAL_FLOW_USED)
/* ====================== optical flow ===================================== */
typedef enum {
        PAW_Product_ID = 0x00,
        PAW_Revision_ID = 0x01,
        PAW_Motion = 0x02,
        PAW_Delta_X_L = 0x03,
        PAW_Delta_X_H = 0x04,
        PAW_Delta_Y_L = 0x05,
        PAW_Delta_Y_H = 0x06,
        PAW_Squal = 0x07,
        PAW_RawData_Sum = 0x08,
        PAW_Maximum_RawData = 0x09,
        PAW_Minimum_RawData = 0x0A,
        PAW_Shutter_Lower = 0x0B,
        PAW_Shutter_Upper = 0x0C,
        PAW_Observation = 0x15,
        PAW_Motion_Burst = 0x16,
        PAW_Power_Up_Reset = 0x3A,
        PAW_Resolution = 0x4E,
        PAW_Reset = 0x5A,
        PAW_Inverse_Product = 0x5F
} PAW3902_Registers_e;                                                          /* control registers */

typedef enum 
{
        PAW3902_Mode_Bright,
        PAW3902_Mode_LowLight,
        PAW3902_Mode_SuperLowLight,
        PAW3902_Mode_FrameCapture        
} PAW_Modes_e;                                                                  /* mode of PAW */

#define PAW3902_CHIP_PRODUCT_ID 0x49u                                           /* product ID returned from probe function shared with the PMW3901 */
#define PAW3902_REVISION_ID 0x01u
#define PAW3902_INV_PROD_ID 0xB6u

typedef enum
{
  PAW_ResetReq,
  PAW_ResetInit,
  PAW_ResetDone
} resetState_e;

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    busDevice_t busdev;
    uint8_t mode;
    uint8_t modeBeforeCapture;
    uint8_t retVal;
    float64_t resolution;
    uint32_t schedOnInterval;
    uint32_t _flow_dt_sum_tick;
    uint32_t _task_sched_sum_tick;
    uint32_t _frame_count_since_last;
    float32_t _flow_sum_x;
    float32_t _flow_sum_y;
    int16_t _bright_to_low_counter;
    int16_t _low_to_bright_counter;
    int16_t _low_to_superlow_counter;
    int16_t _superlow_to_low_counter;
    uint32_t TickRefUpdate;
    uint8_t modeChangeRequest : 1u;
    uint8_t dataRequest : 1u;
    uint8_t resetRequest : 2u;
    uint8_t start2Low : 1u;
    uint8_t start2SuperLow : 1u;
    uint8_t start2Bright : 1u;
    uint8_t spare : 1u;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
    uint8_t Motion;
    uint8_t Observation;
    uint8_t Delta_X_L;
    uint8_t Delta_X_H;
    uint8_t Delta_Y_L;
    uint8_t Delta_Y_H;
    uint8_t SQUAL;
    uint8_t RawData_Sum;
    uint8_t Maximum_RawData;
    uint8_t Minimum_RawData;
    uint8_t Shutter_Upper;
    uint8_t Shutter_Lower;
    uint8_t product_ID;
    uint8_t revision_ID;
    uint8_t inverse_product_ID;
    uint8_t iterations;
    uint8_t kk;
} optical_flow_PAW3902JF_t;
#else
DEVICEICPACKED(
typedef struct {
    busDevice_t busdev;
    uint8_t mode;
    uint8_t modeBeforeCapture;
    uint8_t retVal;
    float64_t resolution;
    uint32_t schedOnInterval;
    uint32_t _flow_dt_sum_tick;
    uint32_t _task_sched_sum_tick;
    uint32_t _frame_count_since_last;
    float32_t _flow_sum_x;
    float32_t _flow_sum_y;
    int16_t _bright_to_low_counter;
    int16_t _low_to_bright_counter;
    int16_t _low_to_superlow_counter;
    int16_t _superlow_to_low_counter;
    uint32_t TickRefUpdate;
    uint8_t modeChangeRequest : 1u;
    uint8_t dataRequest : 1u;
    uint8_t resetRequest : 2u;
    uint8_t start2Low : 1u;
    uint8_t start2SuperLow : 1u;
    uint8_t start2Bright : 1u;
    uint8_t spare : 1u;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
    uint8_t Motion;
    uint8_t Observation;
    uint8_t Delta_X_L;
    uint8_t Delta_X_H;
    uint8_t Delta_Y_L;
    uint8_t Delta_Y_H;
    uint8_t SQUAL;
    uint8_t RawData_Sum;
    uint8_t Maximum_RawData;
    uint8_t Minimum_RawData;
    uint8_t Shutter_Upper;
    uint8_t Shutter_Lower;
    uint8_t product_ID;
    uint8_t revision_ID;
    uint8_t inverse_product_ID;
    uint8_t iterations;
    uint8_t kk;
}) optical_flow_PAW3902JF_t;
#endif

#endif /* -- end PAW optical flow */

#if defined(PX4_OPTICAL_FLOW_USED)
/*
 * Copyright (c) 2014 by Laurent Eschenauer <laurent@eschenauer.be>
 * Ported to PIC32 / FT900 by ACP Aviation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 */
// 7 Bit I2C Address of the Flow Module: Default 0x42 (user selectable bits 0,1,2) 
#define PX4FLOW_ADDRESS 0x42u

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    uint16_t frame_count;                                                       // counts created I2C frames
    int16_t pixel_flow_x_sum;                                                   // accumulated x flow in pixels*10 since last I2C frame
    int16_t pixel_flow_y_sum;                                                   // accumulated y flow in pixels*10 since last I2C frame
    int16_t flow_comp_m_x;                                                      // x velocity*1000 in meters / timestep
    int16_t flow_comp_m_y;                                                      // y velocity*1000 in meters / timestep
    int16_t qual;                                                               // Optical flow quality / confidence 0: bad, 255: maximum quality
    int16_t gyro_x_rate;                                                        //gyro x rate
    int16_t gyro_y_rate;                                                        //gyro y rate
    int16_t gyro_z_rate;                                                        //gyro z rate
    uint8_t gyro_range;                                                         // gyro range
    uint8_t sonar_timestamp;                                                    // timestep in milliseconds between I2C frames
    int16_t ground_distance;                                                    // Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
} px4_flow_frame_t;
#else
DEVICEICPACKED(
typedef struct {
    uint16_t frame_count;                                                       // counts created I2C frames
    int16_t pixel_flow_x_sum;                                                   // accumulated x flow in pixels*10 since last I2C frame
    int16_t pixel_flow_y_sum;                                                   // accumulated y flow in pixels*10 since last I2C frame
    int16_t flow_comp_m_x;                                                      // x velocity*1000 in meters / timestep
    int16_t flow_comp_m_y;                                                      // y velocity*1000 in meters / timestep
    int16_t qual;                                                               // Optical flow quality / confidence 0: bad, 255: maximum quality
    int16_t gyro_x_rate;                                                        //gyro x rate
    int16_t gyro_y_rate;                                                        //gyro y rate
    int16_t gyro_z_rate;                                                        //gyro z rate
    uint8_t gyro_range;                                                         // gyro range
    uint8_t sonar_timestamp;                                                    // timestep in milliseconds between I2C frames
    int16_t ground_distance;                                                    // Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
}) px4_flow_frame_t;
#endif

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    uint16_t frame_count_since_last_readout;                                    //number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;                                              //accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;                                              //accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;                                               //accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_y_rate_integral;                                               //accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_z_rate_integral;                                               //accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] 
    uint32_t integration_timespan;                                              //accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;                                                   // time since last sonar update [microseconds]
    int16_t ground_distance;                                                    // Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;                                                   // Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;                                                            // averaged quality of accumulated flow values [0:bad quality;255: max quality]
} px4_integral_frame_t;
#else
DEVICEICPACKED(
typedef struct {
    uint16_t frame_count_since_last_readout;                                    //number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;                                              //accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;                                              //accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;                                               //accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_y_rate_integral;                                               //accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_z_rate_integral;                                               //accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] 
    uint32_t integration_timespan;                                              //accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;                                                   // time since last sonar update [microseconds]
    int16_t ground_distance;                                                    // Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;                                                   // Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;                                                            // averaged quality of accumulated flow values [0:bad quality;255: max quality]
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
}) px4_integral_frame_t;
#endif

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
   px4_integral_frame_t iframe;
   px4_flow_frame_t flow;
   int32_t ticksVal;
   uint32_t ticksRef;
   uint8_t state;   
} px4_optical_flow_frame_t;
#else
DEVICEICPACKED(
typedef struct {
   px4_integral_frame_t iframe;
   px4_flow_frame_t flow;
   int32_t ticksVal;
   uint32_t ticksRef;
   uint8_t state; 
}) px4_optical_flow_frame_t;
#endif

typedef enum
{
   REQUEST_FLOW,
   WAIT_REQ_FLOW,
   GET_FLOW_DATA,
   WAIT_IDLE_FOR_HANDSHAKE,
   REQUEST_INTGL,
   WAIT_REQ_INTGL,
   GET_INTGL_DATA,
   NUM_OF_PX_FLOW_STATES
} px4_opt_flow_state_e;

#endif /* end px4 opt flo */

#if defined(MPU6050_ACC_GYRO)
#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    busDevice_t busdev;
    uint8_t mode;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
    uint8_t buffer[MPU6050_MAX_RCV_LENGTH];
} mpu6050_Invensense_t;
#else
DEVICEICPACKED(
typedef struct {
    busDevice_t busdev;
    uint8_t mode;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
    uint8_t buffer[MPU6050_MAX_RCV_LENGTH];
}) mpu6050_Invensense_t;
#endif
#endif /* --- end mp6050 gyro acc --- */

#if defined(CCS811_AIR_QUAL)                                                    // https://ams.com/documents/20143/36005/CCS811_DS000459_6-00.pdf/c7091525-c7e5-37ac-eedb-b6c6828b0dcf#page=16

#define        CCS811DriveModeIdle 0x00u 
#define CCS811DriveMode 0x00u
#define CCS811DriveMode1Sec 0x01u
#define CCS811DriveMode10Sec 0x02u
#define CCS811DriveMode60Sec 0x03u
#define CCS811DriveMode250MS 0x04u

        //DefaultAddress is the default I2C address for the ccs811
#define ccs811DefaultAddress = 0x5A

        //Registers, all definitions have been taken from the datasheet
        //Single byte read only register which indicates if a device is active, if new data is available or if an error occurred.
#define ccs811RegStatus 0x00u
        //This is Single byte register, which is used to enable sensor drive mode and interrupts.
#define ccs811RegMeasMode 0x01u
        //This multi-byte read only register contains the calculated eCO2 (ppm) and eTVOC (ppb) values followed by the STATUS register, ERROR_ID register and the RAW_DATA register.
#define ccs811RegAlgResultData 0x02u
        //Two byte read only register which contains the latest readings from the sensor.
#define ccs811RegRawData 0x03u
        //A multi-byte register that can be written with the current Humidity and Temperature values if known.
#define ccs811RegEnvData 0x05u
        //Register that holds the NTC value used for temperature calcualtions
#define ccs811RegNtc 0x06u
        //Asserting the SW_RESET will restart the CCS811 in Boot mode to enable new application firmware to be downloaded.
#define ccs811RegSwReset 0xFFu
        //Single byte read only register which holds the HW ID which is 0x81 for this family of CCS81x devices.
#define ccs811RegHwID 0x20u
        //Single byte read only register that contains the hardware version. The value is 0x1X
#define ccs811RegHwVersion 0x21u
        //Two byte read only register which contain the version of the firmware bootloader stored in the CCS811 in the format Major.Minor.Trivial
#define ccs811RegFwBootVersion 0x23u
        //Two byte read only register which contain the version of the firmware application stored in the CCS811 in the format Major.Minor.Trivial
#define ccs811RegFwAppVersion 0x24u
        //To change the mode of the CCS811 from Boot mode to running the application, a single byte write of 0xF4 is required.
#define ccs811RegAppStart 0xF4u

        // The hardware ID code
#define ccs811HwIDCode 0x81u
#define ccs811ResetSequence {0x11u, 0xE5u, 0x72u, 0x8Au}

#define ccsntcResistanceValue 100000.0f                                         /* sets reistor value used in the temperature calculations. resistor must be placed between pin 4 and pin 8 of the chip Recommended resistance value is 100,000 */

#if defined(D_FT900)
typedef struct DEVICEICPACKED {
    busDevice_t busdev;
    uint8_t mode;
    uint8_t confSamRate : 3u;
    uint8_t intThresh : 1u;
    uint8_t intDataRdy : 1u;
    uint8_t driveMode : 1u;
    uint8_t spareConf : 2u;
    uint8_t HasError :1u; 
    uint8_t DataReady :1u; 
    uint8_t AppValid :1u; 
    uint8_t FwMode :1u;
    uint8_t sparebits : 4u;  
    uint16_t eco2;
    uint16_t tvoC;
    float32_t temperat;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
    uint8_t buf[8u];
} CCS811Status_t;
#else
DEVICEICPACKED(
typedef struct {
    busDevice_t busdev;
    uint8_t mode;
    uint8_t confSamRate : 3u;
    uint8_t intThresh : 1u;
    uint8_t intDataRdy : 1u;
    uint8_t driveMode : 1u;
    uint8_t spareConf : 2u;
    uint8_t HasError :1u; 
    uint8_t DataReady :1u; 
    uint8_t AppValid :1u; 
    uint8_t FwMode :1u;
    uint8_t sparebits : 4u;  
    uint16_t eco2;
    uint16_t tvoC;
    float32_t temperat;
    int32_t ticksVal;
    uint32_t ticksRef;
    uint8_t state;
    uint8_t buf[8u];
}) CCS811Status_t;
#endif

#endif  /* -- end air qual monitoring chip -- */

/* ============== PMLPS Poor mans local positioning system =========== */
#define PMLPS_I2C_MASTER_FREQ_HZ  400000lu
#define PMLPS_PACKET_SIZE 12u
#define PMLPS_PKT_QSIZE  64u
/*____________________________________________________________________________*/
/* #####################---| Function Definition |---######################## */
bool busWriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t dataV);
bool busReadRegisterBuffer(const busDevice_t *busdev, uint8_t reg, uint8_t *dataV, uint8_t length);
bool busInit(const busDevice_t *busdev);
uint8_t write_i2c_module1(uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat);
uint8_t write_i2c_module2(uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat);
uint8_t write_i2c_module3(uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat);
uint8_t write_i2c_module4(uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat);
uint8_t write_i2c_module5(uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat);
uint8_t write_spi_module1(uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat);
uint8_t write_spi_module2(uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat);
uint8_t write_spi_module3(uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat);
uint8_t write_spi_module4(uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat);
uint8_t read_i2c_module5(uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length);
uint8_t read_i2c_module4(uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length);
uint8_t read_i2c_module3(uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length);
uint8_t read_i2c_module2(uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length);
uint8_t read_i2c_module1(uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length);
uint8_t read_spi_module1(uint8_t deviceAddress, unsigned char reg, unsigned char *dataV, uint16_t len);
uint8_t read_spi_module2(uint8_t deviceAddress, unsigned char reg, unsigned char *dataV, uint16_t len);
uint8_t read_spi_module3(uint8_t deviceAddress, unsigned char reg, unsigned char *dataV, uint16_t len);
uint8_t read_spi_module4(uint8_t deviceAddress, unsigned char reg, unsigned char *dataV, uint16_t len);
void spi4_initial();
void spi3_initial();
void spi2_initial();
void spi1_initial();
/* ======== QMP6988 Barometer ==================== */
#if defined(QMP6988_BARO_USED)
bool qmp6988Detect(baroDev_t *baro, qmp6988_calib_param_t *qmp6988_cal, float64_t *press, float64_t *temper ); /* main collector for qmp6988 */
void qmp6988_calculate(float64_t *pressure, float64_t *temperature, const qmp6988_calib_param_t *qmp6988_cal, const baroDev_t *baro);
float64_t qmp6988_compensate_T(int32_t adc_T, const qmp6988_calib_param_t *qmp6988_cal);
void qmp6988_get_up(baroDev_t *baro);
void qmp6988_start_up(baroDev_t *baro);
#endif
/* ======== FBM320 Barometer ==================== */
#if defined(FBM320_BARO_USED)
bool Baro_FBM320read_cali(fbm320_calibration_t *calibration, const baroDev_t *baro);
void Baro_FBM320calc_PT(int32_t UT, int32_t UP, int32_t *pressure, int32_t *temperature, const fbm320_calibration_t *cal);
void Baro_FBM320StartStream( baroDev_t *baro );
void Baro_FBM320GetData( baroDev_t *baro, const fbm320_calibration_t *cal );
void Baro_FBM320InitStart( fbm320_calibration_t *calibration, baroDev_t *baro, float64_t *press, float64_t *temper ); /* main collector for FBM320 */
#endif
void Baro_CalcTempAndPress( baroDev_t *baro, float64_t *pressure, float64_t *temperature );
/* ======== Keller Barometer ==================== */
#if defined(KELLER_BARO_USED)
void Baro_KellerGetData( baroDev_t *baro );
void Baro_KellerCalcul( baroDev_t *baro, float64_t *pressure, float64_t *temperature, const keller_calibration_t *cal );
#endif
uint32_t baroCalculateAltitude( baro_t *baro, float64_t baroPressure );
void performBaroCalibrationCycle( float64_t baroPressure, baro_t *baro );
void baroSetCalibrationCycles( uint16_t calibrationCyclesRequired, baro_t *baro );
/* ======== HMC6352 Compass ==================== */
#if defined(HMC6352_COMPASS_USED)
void Compass_HMC6352Start( compass_t *compass );
void Compass_HMC6352GetData( compass_t *compass );
#endif
/* ======== LiteStart Liddar ================= */
#if defined(LIDAR_LITESTART_USED)
void Lidar_LiteStart( distance_t *dist );
void Lidar_LiteGetData( distance_t *dist );
#endif
/* ====== BH1750 Lux Sensor ================= */
#if defined(BH1750_LUX_USED)
void Lux_BH1750Start( lux_t *luxobj );
void Lux_BH1750GetData( lux_t *luxobj );
#endif
/* ====== INA3221 Power Monitor ================= */
#if defined(INA3221_POWER_USED)
void INA3221_Init( powerIV_IC_t *vi );
void INA3221_GetShuntVoltCurrent( INA3221Channel_e channel, powerIV_IC_t *vi );
void INA3221_GetBusVoltage( INA3221Channel_e channel, powerIV_IC_t *vi );
void INA3221_GetLoadVoltage( powerIV_IC_t *vi );
#endif
/* ====== PAW3902 Optical Flow Sensor ================= */
#if defined(PAW_OPTICAL_FLOW_USED)
uint8_t PAW3902_modeBright( optical_flow_PAW3902JF_t *dev );
uint8_t PAW3902_modeLowLight( optical_flow_PAW3902JF_t *dev );
uint8_t PAW3902_modeSuperLowLight( optical_flow_PAW3902JF_t *dev );
uint8_t PAW3902_reset( optical_flow_PAW3902JF_t *dev );
uint8_t PAW3902_changeMode( optical_flow_PAW3902JF_t *dev, uint8_t newMode );
uint8_t PAW3902_getUpdate( optical_flow_PAW3902JF_t *dev );
uint8_t PAW3902_enterFrameCaptureMode( optical_flow_PAW3902JF_t *dev );
uint8_t PAW3902_enterFrameCaptureModeWithDelay( optical_flow_PAW3902JF_t *dev );
uint8_t PAW3902_captureFrame(uint8_t * frameArray, optical_flow_PAW3902JF_t *dev);
uint8_t PAW3902_exitFrameCaptureModeWithDelay( optical_flow_PAW3902JF_t *dev );
uint8_t PAW3902_checkID( optical_flow_PAW3902JF_t *dev );
void paw3902_spi_init( const busDevice_t *busdev );
uint8_t PAW3902_RunImpl( optical_flow_PAW3902JF_t *dev, optical_flow_report_t *report );
#endif
/* ====== IAQ2000 cO2 Sensor ================= */
#if defined(WANT_IAQ2000_Co2)
devIC_PR_TE void IAQ_2000Start( iaq2000_t *iaQobj );
devIC_PR_TE void IAQ_2000Update( iaq2000_t *iaQobj );
#endif
/*-----------------------------------------------------------------------------
 *      read_i2c_module1():  read the data from i2c bus Number 1
 *
 *  Parameters: uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t read_i2c_module1(uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length)
{
  uint8_t i2cDevByte = 0u;
  uint8_t dataAdrStart = 0u;
  uint8_t ret=0u;

  if ((dat == NULL) || (((uint8_t)sizeof(dat)) < length))
  {  /* just for sanity */
  }
  else
  {
     dataAdrStart = dataAddress;
     while(length>0u)
     {
        if (!I2C1_Start())                                                      // issue I2C start signal
        {
           i2cDevByte = ((deviceAddress<<1u)+0u);
           if(!I2C1_Write(i2cDevByte))                                          // send byte via I2C  (device address + W)
           {
              if (!I2C1_Write(dataAddress))                                     // send byte (data address)
              {
                 if (!I2C1_Restart())                                           // issue I2C signal repeated start
                 {
                    i2cDevByte = ((deviceAddress<<1u)+1u);
                    if (!I2C1_Write(i2cDevByte))                                // send byte (device address + R)
                    {
                       dat[dataAddress-dataAdrStart] = I2C1_Read(1u);           // Read the data (NO acknowledge)
                       I2C1_Stop();
                       dataAddress=++dataAddress % UINT8_MAX;
                       length = length - 1u;
                       ret = 1u;
                    }
                 }
              }
           }
        }
     }
  }
  return ret;
}
/*-----------------------------------------------------------------------------
 *      read_i2c_module2():  read the data from i2c bus Number 2
 *
 *  Parameters: uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t read_i2c_module2(uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length)
{
  uint8_t i2cDevByte = 0u;
  uint8_t dataAdrStart = 0u;
  uint8_t ret = 0u;
  
  if ((dat == NULL) || (((uint8_t)sizeof(dat)) < length))
  {  /* just for sanity */
  }
  else
  {
     dataAdrStart = dataAddress;
     while(length>0u)
     {
        if (!I2C2_Start())                                                      // issue I2C start signal
        {
           i2cDevByte = ((deviceAddress<<1u)+0u);
           if(!I2C2_Write(i2cDevByte))                                          // send byte via I2C  (device address + W)
           {
              if (!I2C2_Write(dataAddress))                                     // send byte (data address)
              {
                 if (!I2C2_Restart())                                           // issue I2C signal repeated start
                 {
                    i2cDevByte = ((deviceAddress<<1u)+1u);
                    if (!I2C2_Write(i2cDevByte))                                // send byte (device address + R)
                    {
                       dat[dataAddress-dataAdrStart] = I2C2_Read(1u);           // Read the data (NO acknowledge)
                       I2C2_Stop();
                       dataAddress=++dataAddress % UINT8_MAX;
                       length = length - 1u;
                       ret = 1u;
                    }
                 }
              }
           }
        }
     }
  }
  return ret;
}
/*-----------------------------------------------------------------------------
 *      read_i2c_module3():  read the data from i2c bus Number 3
 *
 *  Parameters: uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t read_i2c_module3(uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length)
{
  uint8_t i2cDevByte = 0u;
  uint8_t dataAdrStart = 0u;
  uint8_t ret = 0u;

  if ((dat == NULL) || (((uint8_t)sizeof(dat)) < length))
  {  /* just for sanity */
  }
  else
  {
     dataAdrStart = dataAddress;
     while(length>0u)
     {
        if (!I2C3_Start())                                                      // issue I2C start signal
        {
           i2cDevByte = ((deviceAddress<<1u)+0u);
           if(!I2C3_Write(i2cDevByte))                                          // send byte via I2C  (device address + W)
           {
              if (!I2C3_Write(dataAddress))                                     // send byte (data address)
              {
                 if (!I2C3_Restart())                                           // issue I2C signal repeated start
                 {
                    i2cDevByte = ((deviceAddress<<1u)+1u);
                    if (!I2C3_Write(i2cDevByte))                                // send byte (device address + R)
                    {
                       dat[dataAddress-dataAdrStart] = I2C3_Read(1u);           // Read the data (NO acknowledge)
                       I2C3_Stop();
                       dataAddress=++dataAddress % UINT8_MAX;
                       length = length - 1u;
                       ret = 1u;
                    }
                 }
              }
           }
        }
     }
  }
  return ret;
}
/*-----------------------------------------------------------------------------
 *      read_i2c_module4():  read the data from i2c bus Number 4
 *
 *  Parameters: uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t read_i2c_module4(uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length)
{
  uint8_t i2cDevByte = 0u;
  uint8_t dataAdrStart = 0u;
  uint8_t ret = 0u;
  
  if ((dat == NULL) || (((uint8_t)sizeof(dat)) < length))
  {  /* just for sanity */
  }
  else
  {
     dataAdrStart = dataAddress;
     while(length>0u)
     {
        if (!I2C4_Start())                                                      // issue I2C start signal
        {
           i2cDevByte = ((deviceAddress<<1u)+0u);
           if(!I2C4_Write(i2cDevByte))                                          // send byte via I2C  (device address + W)
           {
              if (!I2C4_Write(dataAddress))                                     // send byte (data address)
              {
                 if (!I2C4_Restart())                                           // issue I2C signal repeated start
                 {
                    i2cDevByte = ((deviceAddress<<1u)+1u);
                    if (!I2C4_Write(i2cDevByte))                                // send byte (device address + R)
                    {
                       dat[dataAddress-dataAdrStart] = I2C4_Read(1u);           // Read the data (NO acknowledge)
                       I2C4_Stop();
                       dataAddress=++dataAddress % UINT8_MAX;
                       length = length - 1u;
                       ret = 1u;
                    }
                 }
              }
           }
        }
     }
  }
  return ret;
}
/*-----------------------------------------------------------------------------
 *      read_i2c_module5():  read the data from i2c bus Number 5
 *
 *  Parameters: uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t read_i2c_module5(uint8_t deviceAddress, uint8_t dataAddress, uint8_t *dat, uint8_t length)
{
  uint8_t i2cDevByte = 0u;
  uint8_t dataAdrStart = 0u;
  uint8_t ret = 0u;

  if ((dat == NULL) || (((uint8_t)sizeof(dat)) < length))
  {  /* just for sanity */
  }
  else
  {
     dataAdrStart = dataAddress;
     while(length>0u)
     {
        if (!I2C5_Start())                                                      // issue I2C start signal
        {
           i2cDevByte = ((deviceAddress<<1u)+0u);
           if(!I2C5_Write(i2cDevByte))                                          // send byte via I2C  (device address + W)
           {
              if (!I2C5_Write(dataAddress))                                     // send byte (data address)
              {
                 if (!I2C5_Restart())                                           // issue I2C signal repeated start
                 {
                    i2cDevByte = ((deviceAddress<<1u)+1u);
                    if (!I2C5_Write(i2cDevByte))                                // send byte (device address + R)
                    {
                       dat[dataAddress-dataAdrStart] = I2C5_Read(1u);           // Read the data (NO acknowledge)
                       I2C5_Stop();
                       dataAddress=++dataAddress % UINT8_MAX;
                       length = length - 1u;
                       ret = 1u;
                    }
                 }
              }
           }
        }
     }
  }
  return ret;
}
/*-----------------------------------------------------------------------------
 *      read_spi_module1():  read the data from spi bus Number 1
 *
 *  Parameters: uint8_t deviceAddress, unsigned char reg, unsigned char dataV[], uint16_t len
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
uint8_t read_spi_module1(uint8_t deviceAddress, unsigned char reg, unsigned char *dataV, uint16_t len)
{
   uint16_t char2rcv;
   char buffr=0;
   uint8_t ret=0u;
   
   if ((sizeof(dataV) >= len) && (dataV != NULL))
   {
      reg |= 0x80u;                                                             /* set the read bit (as per the accelerometer’s protocol) */
      if(len >= 1u)
      {
         reg |= 0x40u;                                                          /* set the address auto inc. bit (as per the accelerometer’s protocol) } */
         Chip_Select1 = 0;                                                      /* bring CS low to activate SPI */
         SPI1_Write((uint32_t)reg);
         for(char2rcv = 0u; char2rcv != len; ++char2rcv)
         {
            dataV[char2rcv] = SPI1_Read(buffr);                                 /* read data from spi  */
         }
         Chip_Select1 = 1;                                                      /* bring CS high to de-activate SPI */
         ret=1u;
      }
   }
   else { /*just for sanity */ }
   return ret;
}

/*-----------------------------------------------------------------------------
 *      read_spi_module2():  read the data from spi bus Number 2
 *
 *  Parameters: uint8_t deviceAddress, unsigned char reg, unsigned char dataV[], uint16_t len
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
uint8_t read_spi_module2(uint8_t deviceAddress, unsigned char reg, unsigned char *dataV, uint16_t len)
{
   uint16_t char2rcv;
   char buffr=0;
   uint8_t ret=0u;

   if ((sizeof(dataV) >= len) && (dataV != NULL))
   {
      reg |= 0x80u;                                                             /* set the read bit (as per the accelerometer’s protocol) */
      if(len >= 1u)
      {
         reg |= 0x40u;                                                          /* set the address auto inc. bit (as per the accelerometer’s protocol) } */
         Chip_Select2 = 0;                                                      /* bring CS low to activate SPI */
         SPI2_Write((uint32_t)reg);
         for(char2rcv = 0u; char2rcv != len; ++char2rcv)
         {
            dataV[char2rcv] = SPI2_Read(buffr);                                 /* read data from spi  */
         }
         Chip_Select2 = 1;                                                      /* bring CS high to de-activate SPI */
         ret=1u;
      }
   }
   else { /*just for sanity */ }
   return ret;
}

/*-----------------------------------------------------------------------------
 *      read_spi_module3():  read the data from spi bus Number 3
 *
 *  Parameters: uint8_t deviceAddress, unsigned char reg, unsigned char dataV[], uint16_t len
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
uint8_t read_spi_module3(uint8_t deviceAddress, unsigned char reg, unsigned char *dataV, uint16_t len)
{
   uint16_t char2rcv;
   char buffr=0;
   uint8_t ret=0u;

   if ((sizeof(dataV) >= len) && (dataV != NULL))
   {
      reg |= 0x80u;                                                             /* set the read bit (as per the accelerometer’s protocol) */
      if(len >= 1u)
      {
         reg |= 0x40u;                                                          /* set the address auto inc. bit (as per the accelerometer’s protocol) } */
         Chip_Select3 = 0;                                                      /* bring CS low to activate SPI */
         SPI3_Write((uint32_t)reg);
         for(char2rcv = 0u; char2rcv != len; ++char2rcv)
         {
            dataV[char2rcv] = SPI3_Read(buffr);                                 /* read data from spi  */
         }
         Chip_Select3 = 1;                                                      /* bring CS high to de-activate SPI */
         ret = 1u;
      }
   }
   else { /*just for sanity */ }
   return ret;
}

/*-----------------------------------------------------------------------------
 *      read_spi_module4():  read the data from spi bus Number 4
 *
 *  Parameters: uint8_t deviceAddress, unsigned char reg, unsigned char dataV[], uint16_t len
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
uint8_t read_spi_module4(uint8_t deviceAddress, unsigned char reg, unsigned char *dataV, uint16_t len)
{
   uint16_t char2rcv;
   char buffr=0;
   uint8_t ret=0u;

   if ((sizeof(dataV) >= len) && (dataV != NULL))
   {
      reg |= 0x80u;                                                             /* set the read bit (as per the accelerometer’s protocol) */
      if(len >= 1u)
      {
         reg |= 0x40u;                                                          /* set the address auto inc. bit (as per the accelerometer’s protocol) } */
         Chip_Select4 = 0;                                                      /* bring CS low to activate SPI */
         SPI4_Write((uint32_t)reg);
         for(char2rcv = 0u; char2rcv != len; ++char2rcv)
         {
            dataV[char2rcv] = SPI4_Read(buffr);                                 /* read data from spi  */
         }
         Chip_Select4 = 1;                                                      /* bring CS high to de-activate SPI */
         ret = 1u;
      }
   }
   else { /*just for sanity */ }
   return ret;
}
/*-----------------------------------------------------------------------------
 *      busReadRegisterBuffer():  read the data from i2c or spi bus on any allowed number
 *
 *  Parameters: const busDevice_t *busdev, uint8_t reg, uint8_t *dataV, uint8_t length
 *  Return:     (bool)
 *----------------------------------------------------------------------------*/
devIC_PR_TE  bool busReadRegisterBuffer(const busDevice_t *busdev, uint8_t reg, uint8_t *dataV, uint8_t length)
{
    bool ret=false;
    
    if ((busdev != NULL) && (dataV != NULL))
    {
       switch (busdev->bustype)
       {
           case BUSTYPE_SPI:
           switch (busdev->busdev_u.spi.device)
           {
              case SPIDEV_1:
              ret=read_spi_module1(busdev->busdev_u.spi.address, reg, dataV, length);
              break;

              case SPIDEV_2:
              ret=read_spi_module2(busdev->busdev_u.spi.address, reg, dataV, length);
              break;

              case SPIDEV_3:
              ret=read_spi_module3(busdev->busdev_u.spi.address, reg, dataV, length);
              break;

              case SPIDEV_4:
              ret=read_spi_module4(busdev->busdev_u.spi.address, reg, dataV, length);
              break;
           
              default:
              break;
           }
           break;
        
           case BUSTYPE_I2C:
           switch (busdev->busdev_u.i2c.device)
           {
              case I2CDEV_1:
              ret=read_i2c_module1(busdev->busdev_u.i2c.address, reg, dataV, length);
              break;

              case I2CDEV_2:
              ret=read_i2c_module2(busdev->busdev_u.i2c.address, reg, dataV, length);
              break;

              case I2CDEV_3:
              ret=read_i2c_module3(busdev->busdev_u.i2c.address, reg, dataV, length);
              break;

              case I2CDEV_4:
              ret=read_i2c_module4(busdev->busdev_u.i2c.address, reg, dataV, length);
              break;

              case I2CDEV_5:
              ret=read_i2c_module5(busdev->busdev_u.i2c.address, reg, dataV, length);
              break;

              default:
              break;
           }
           break;
        
           default:
           break;
        }
    }
    else { /* just for misra */ }
    return true;
}

/*-----------------------------------------------------------------------------
 *      write_i2c_module1():  write a byte on i2c bus module number 1
 *
 *  Parameters: uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t write_i2c_module1(uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat)
{
  uint8_t temp;
  uint8_t i2cDevByte;                                                           // Create the header byte from Device Address + Read(1) Write(0)
  uint8_t ret=0u;
  
  if(!I2C1_Start())                                                             // Send the start sequence
  {                                                                             // issue I2C start signal
    i2cDevByte = ((deviceAddress<<1u)+0u);                                      // device address with 0 RW=0
    if(!I2C1_Write(i2cDevByte))                                                 // send byte via I2C (device address <<1 + 0 )  Send the I2C address of the slave with the R/W bit low (even address)
    {
       temp = (uint8_t) (dataAddress >> 8u);                                     // saving higher order address to temp
       if(!I2C1_Write(temp))                                                    // sending higher order address Send the internal register number you want to write to
       {
         if(!I2C1_Write((uint8_t)(dataAddress & 0x0Fu)))                         // sending lower order address
         {
           if(!I2C1_Write(dat))                                                 // send data (data to be written)   Send the data byte
           {
            I2C1_Stop();                                                        // issue I2C stop signal
            ret=1u;                                                             // success
           }
         }
         else
         {
           ret=0u;                                                              /* error (just for sanity we need the branch for misra) */
         }
      }
      else
      {
        ret=0u;                                                                 /* error (just for sanity we need the branch for misra) */
      }
    }
    else
    {
       ret=0u;                                                                  /* error (just for sanity we need the branch for misra) */
    }
  }
  else
  {
     ret=0u;                                                                    /* error (just for sanity we need the branch for misra) */
  }
  return ret;
}

/*-----------------------------------------------------------------------------
 *      write_i2c_module2():  write a byte on i2c bus module number 2
 *
 *  Parameters: uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t write_i2c_module2(uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat)
{
  uint8_t temp;
  uint8_t i2cDevByte;                                                           // Create the header byte from Device Address + Read(1) Write(0)
  uint8_t ret=0u;

  if(!I2C2_Start())                                                             // Send the start sequence
  {                                                                             // issue I2C start signal
    i2cDevByte =  ((deviceAddress<<1u)+0u);                                     // device address with 0 RW=0
    if(!I2C2_Write(i2cDevByte))                                                  // send byte via I2C (device address <<1 + 0 )  Send the I2C address of the slave with the R/W bit low (even address)
    {
       temp = (uint8_t) (dataAddress >> 8u);                                     // saving higher order address to temp
       if(!I2C2_Write(temp))                                                    // sending higher order address Send the internal register number you want to write to
       {
         if(!I2C2_Write((uint8_t)(dataAddress & 0x0Fu)))                         // sending lower order address
         {
           if(!I2C2_Write(dat))                                                 // send data (data to be written)   Send the data byte
           {
            I2C2_Stop();                                                        // issue I2C stop signal
            ret=1u;                                                             // success
           }
         }
         else
         {
           ret=0u;                                                              /* error (just for sanity we need the branch for misra) */
         }
      }
      else
      {
        ret=0u;                                                                 /* error (just for sanity we need the branch for misra) */
      }
    }
    else
    {
       ret=0u;                                                                  /* error (just for sanity we need the branch for misra) */
    }
  }
  else
  {
     ret=0u;                                                                    /* error (just for sanity we need the branch for misra) */
  }
  return ret;
}
/*-----------------------------------------------------------------------------
 *      write_i2c_module3():  write a byte on i2c bus module number 3
 *
 *  Parameters: uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t write_i2c_module3(uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat)
{
  uint8_t temp;
  uint8_t i2cDevByte;                                                           // Create the header byte from Device Address + Read(1) Write(0)
  uint8_t ret=0u;

  if(!I2C3_Start())                                                             // Send the start sequence
  {                                                                             // issue I2C start signal
    i2cDevByte =  ((deviceAddress<<1u)+0u);                            // device address with 0 RW=0
    if(!I2C3_Write(i2cDevByte))                                                  // send byte via I2C (device address <<1 + 0 )  Send the I2C address of the slave with the R/W bit low (even address)
    {
       temp = (uint8_t) (dataAddress >> 8u);                                     // saving higher order address to temp
       if(!I2C3_Write(temp))                                                    // sending higher order address Send the internal register number you want to write to
       {
         if(!I2C3_Write((uint8_t)(dataAddress & 0x0Fu)))                         // sending lower order address
         {
           if(!I2C3_Write(dat))                                                 // send data (data to be written)   Send the data byte
           {
            I2C3_Stop();                                                        // issue I2C stop signal
            ret=1u;                                                             // success
           }
         }
         else
         {
           ret=0u;                                                              /* error (just for sanity we need the branch for misra) */
         }
      }
      else
      {
        ret=0u;                                                                 /* error (just for sanity we need the branch for misra) */
      }
    }
    else
    {
       ret=0u;                                                                  /* error (just for sanity we need the branch for misra) */
    }
  }
  else
  {
     ret=0u;                                                                    /* error (just for sanity we need the branch for misra) */
  }
  return ret;
}

/*-----------------------------------------------------------------------------
 *      write_i2c_module4():  write a byte on i2c bus module number 4
 *
 *  Parameters: uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t write_i2c_module4(uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat)
{
  uint8_t temp;
  uint8_t i2cDevByte;                                                           // Create the header byte from Device Address + Read(1) Write(0)
  uint8_t ret=0u;

  if(!I2C4_Start())                                                             // Send the start sequence
  {                                                                             // issue I2C start signal
    i2cDevByte = (uint8_t) ((deviceAddress<<1u)+0u);                            // device address with 0 RW=0
    if(!I2C4_Write(i2cDevByte))                                                  // send byte via I2C (device address <<1 + 0 )  Send the I2C address of the slave with the R/W bit low (even address)
    {
       temp = (uint8_t) (dataAddress >> 8u);                                     // saving higher order address to temp
       if(!I2C4_Write(temp))                                                    // sending higher order address Send the internal register number you want to write to
       {
         if(!I2C4_Write((uint8_t)(dataAddress & 0x0Fu)))                         // sending lower order address
         {
           if(!I2C4_Write(dat))                                                 // send data (data to be written)   Send the data byte
           {
            I2C4_Stop();                                                        // issue I2C stop signal
            ret=1u;                                                             // success
           }
         }
         else
         {
           ret=0u;                                                              /* error (just for sanity we need the branch for misra) */
         }
      }
      else
      {
        ret=0u;                                                                 /* error (just for sanity we need the branch for misra) */
      }
    }
    else
    {
       ret=0u;                                                                  /* error (just for sanity we need the branch for misra) */
    }
  }
  else
  {
     ret=0u;                                                                    /* error (just for sanity we need the branch for misra) */
  }
  return ret;
}
/*-----------------------------------------------------------------------------
 *      write_i2c_module5():  write a byte on i2c bus module number 5
 *
 *  Parameters: uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat
 *  Return:     (uint8_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t write_i2c_module5(uint8_t deviceAddress, uint16_t dataAddress, uint8_t dat)
{
  uint8_t temp;
  uint8_t i2cDevByte;                                                           // Create the header byte from Device Address + Read(1) Write(0)
  uint8_t ret=0u;

  if(!I2C3_Start())                                                             // Send the start sequence
  {                                                                             // issue I2C start signal
    i2cDevByte = (uint8_t) ((deviceAddress<<1u)+0u);                            // device address with 0 RW=0
    if(!I2C5_Write(i2cDevByte))                                                  // send byte via I2C (device address <<1 + 0 )  Send the I2C address of the slave with the R/W bit low (even address)
    {
       temp = (uint8_t) (dataAddress >> 8u);                                     // saving higher order address to temp
       if(!I2C5_Write(temp))                                                    // sending higher order address Send the internal register number you want to write to
       {
         if(!I2C5_Write((uint8_t)(dataAddress & 0x0Fu)))                         // sending lower order address
         {
           if(!I2C5_Write(dat))                                                 // send data (data to be written)   Send the data byte
           {
            I2C5_Stop();                                                        // issue I2C stop signal
            ret=1u;                                                             // success
           }
         }
         else
         {
           ret=0u;                                                              /* error (just for sanity we need the branch for misra) */
         }
      }
      else
      {
        ret=0u;                                                                 /* error (just for sanity we need the branch for misra) */
      }
    }
    else
    {
       ret=0u;                                                                  /* error (just for sanity we need the branch for misra) */
    }
  }
  else
  {
     ret=0u;                                                                    /* error (just for sanity we need the branch for misra) */
  }
  return ret;
}
/*-----------------------------------------------------------------------------
 *      write_spi_module1():  write to register the data specified on spi bus 1
 *
 *  Parameters: uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t write_spi_module1(uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat)
{
    Chip_Select1 = 0;                                                           /* bring CS low to activate SPI */
    SPI1_Write(dataAddress);
    SPI1_Write(dat);
    Chip_Select1 = 1;                                                           /* bring CS high to de-activate SPI */
    return 1u;
}
/*-----------------------------------------------------------------------------
 *      write_spi_module2():  write to register the data specified on spi bus 2
 *
 *  Parameters: uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t write_spi_module2(uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat)
{
    Chip_Select2 = 0;                                                           /* bring CS low to activate SPI */
    SPI2_Write(dataAddress);
    SPI2_Write(dat);
    Chip_Select2 = 1;                                                           /* bring CS high to de-activate SPI */
    return 1u;
}
/*-----------------------------------------------------------------------------
 *      write_spi_module3():  write to register the data specified on spi bus 3
 *
 *  Parameters: uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t write_spi_module3(uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat)
{
    Chip_Select3 = 0;                                                           /* bring CS low to activate SPI */
    SPI3_Write(dataAddress);
    SPI3_Write(dat);
    Chip_Select3 = 1;                                                           /* bring CS high to de-activate SPI */
    return 1u;
}
/*-----------------------------------------------------------------------------
 *      write_spi_module4():  write to register the data specified on spi bus 4
 *
 *  Parameters: uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
uint8_t write_spi_module4(uint8_t deviceAddress, uint8_t dataAddress, uint8_t dat)
{
    Chip_Select4 = 0;                                                           /* bring CS low to activate SPI */
    SPI4_Write(dataAddress);
    SPI4_Write(dat);
    Chip_Select4 = 1;                                                           /* bring CS high to de-activate SPI */
    return 1u;
}
/*-----------------------------------------------------------------------------
 *      busWriteRegister():  write a byte on i2c or spi bus any allowed module
 *
 *  Parameters: const busDevice_t *busdev, uint8_t reg, uint8_t dataV
 *  Return:     (bool)
 *----------------------------------------------------------------------------*/
devIC_PR_TE bool busWriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t dataV)
{
    uint8_t ret=false;
    
    if (busdev == NULL)
    {
       /* for misra */
    }
    else
    {
       switch (busdev->bustype)
       {
          case BUSTYPE_SPI:
          switch (busdev->busdev_u.spi.device)
          {
             case SPIDEV_1:
             ret=write_spi_module1(busdev->busdev_u.spi.address, reg, dataV);   /* address not used you might want to use it internally */
             break;

             case SPIDEV_2:
             ret=write_spi_module2(busdev->busdev_u.spi.address, reg, dataV);
             break;

             case SPIDEV_3:
             ret=write_spi_module3(busdev->busdev_u.spi.address, reg, dataV);
             break;

             case SPIDEV_4:
             ret=write_spi_module4(busdev->busdev_u.spi.address, reg, dataV);
             break;

             default:
             break;
          }
          break;

          case BUSTYPE_I2C:
          switch (busdev->busdev_u.i2c.device)
          {
             case I2CDEV_1:
             ret=write_i2c_module1(busdev->busdev_u.i2c.address, reg, dataV);
             break;

             case I2CDEV_2:
             ret=write_i2c_module2(busdev->busdev_u.i2c.address, reg, dataV);
             break;
           
             case I2CDEV_3:
             ret=write_i2c_module3(busdev->busdev_u.i2c.address, reg, dataV);
             break;
           
             case I2CDEV_4:
             ret=write_i2c_module4(busdev->busdev_u.i2c.address, reg, dataV);
             break;
           
             case I2CDEV_5:
             ret=write_i2c_module5(busdev->busdev_u.i2c.address, reg, dataV);
             break;

             default:
             break;
          }
          break;

          default:
          break;
       }
    }
    return ret;
}
/*-----------------------------------------------------------------------------
 *      spi1_initial():  initialze the spi bus number 1
 *
 *  Parameters:
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
void spi1_initial()
{
   Chip_Select1 = 1;                                                            // Deselect SPi preiferal
   Chip_Select_Direction1 = 0;                                                  // Set CS# pin as Output
   SPI1_Init();                                                                 /* initialise SPI link 1 */
   SPI1CON = 0u;                                                                /* turn off the SPI module and reset it */
   SPI1BUF;                                                                     /* clear the rx buffer by reading from it  */
   SPI1BRG = 0x3u;                                                              /* baud rate to 10MHz [SPI4BRG = (80000000/(2*desired))-1]  */
   SPI1STATbits.SPIROV = 0;                                                     /* clear the overflow bit  */
   SPI1CONbits.CKE = 1;                                                         /* data changes when clock goes from active to inactive (high to low since CKP is 0) */
   SPI1CONbits.MSTEN = 1;                                                       /* master operation  */
   SPI1CONbits.ON = 1;                                                          /* turn on SPI 1  */
}
/*-----------------------------------------------------------------------------
 *      spi2_initial():  initialze the spi bus number 2
 *
 *  Parameters:
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
void spi2_initial()
{
   Chip_Select2 = 1;                                                            // Deselect SPi preiferal
   Chip_Select_Direction2 = 0;                                                  // Set CS# pin as Output
   SPI1_Init();                                                                 /* initialise SPI link 1 */
   SPI2CON = 0u;                                                                /* turn off the SPI module and reset it */
   SPI2BUF;                                                                     /* clear the rx buffer by reading from it  */
   SPI2BRG = 0x3u;                                                              /* baud rate to 10MHz [SPI4BRG = (80000000/(2*desired))-1]  */
   SPI2STATbits.SPIROV = 0;                                                     /* clear the overflow bit  */
   SPI2CONbits.CKE = 1;                                                         /* data changes when clock goes from active to inactive (high to low since CKP is 0) */
   SPI2CONbits.MSTEN = 1;                                                       /* master operation  */
   SPI2CONbits.ON = 1;                                                          /* turn on SPI 2  */
}
/*-----------------------------------------------------------------------------
 *      spi3_initial():  initialze the spi bus number 3
 *
 *  Parameters:
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
void spi3_initial()
{
   Chip_Select3 = 1;                                                            // Deselect SPi preiferal
   Chip_Select_Direction3 = 0;                                                  // Set CS# pin as Output
   SPI3_Init();                                                                 /* initialise SPI link 1 */
   SPI3CON = 0u;                                                                /* turn off the SPI module and reset it */
   SPI3BUF;                                                                     /* clear the rx buffer by reading from it  */
   SPI3BRG = 0x3u;                                                              /* baud rate to 10MHz [SPI4BRG = (80000000/(2*desired))-1]  */
   SPI3STATbits.SPIROV = 0;                                                     /* clear the overflow bit  */
   SPI3CONbits.CKE = 1;                                                         /* data changes when clock goes from active to inactive (high to low since CKP is 0) */
   SPI3CONbits.MSTEN = 1;                                                       /* master operation  */
   SPI3CONbits.ON = 1;                                                          /* turn on SPI 3  */
}
/*-----------------------------------------------------------------------------
 *      spi4_initial():  initialze the spi bus number 4
 *
 *  Parameters:
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
void spi4_initial()
{
   Chip_Select4 = 1;                                                            // Deselect SPi preiferal
   Chip_Select_Direction4 = 0;                                                  // Set CS# pin as Output
   SPI4_Init();                                                                 /* initialise SPI link 4 */
   SPI4CON = 0u;                                                                /* turn off the SPI module and reset it */
   SPI4BUF;                                                                     /* clear the rx buffer by reading from it  */
   SPI4BRG = 0x3u;                                                              /* baud rate to 10MHz [SPI4BRG = (80000000/(2*desired))-1]  */
   SPI4STATbits.SPIROV = 0;                                                     /* clear the overflow bit  */
   SPI4CONbits.CKE = 1;                                                         /* data changes when clock goes from active to inactive (high to low since CKP is 0) */
   SPI4CONbits.MSTEN = 1;                                                       /* master operation  */
   SPI4CONbits.ON = 1;                                                          /* turn on SPI 4  */
}
/*-----------------------------------------------------------------------------
 *      busInit():  initialze the spi or i2c bus on the allowed module
 *
 *  Parameters: const busDevice_t *busdev
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE bool busInit(const busDevice_t *busdev)
{
    switch (busdev->bustype)
    {
        case BUSTYPE_SPI:
        switch (busdev->busdev_u.spi.device)
        {
           case SPIDEV_1:
           spi1_initial();
           break;

           case SPIDEV_2:
           spi2_initial();
           break;

           case SPIDEV_3:
           spi3_initial();
           break;

           case SPIDEV_4:
           spi4_initial();
           break;

           default:
           break;
        }
        break;

        case BUSTYPE_I2C:
        switch (busdev->busdev_u.i2c.device)
        {
           case I2CDEV_1:
           I2C1_Init(100000UL);                                                 // bus set @ 100Khz
           break;

           case I2CDEV_2:
           I2C2_Init(100000UL);                                                 // bus set @ 100Khz
           break;

           case I2CDEV_3:
           I2C3_Init(100000UL);                                                 // bus set @ 100Khz
           break;

           case I2CDEV_4:
           I2C4_Init(100000UL);                                                 // bus set @ 100Khz
           break;

           case I2CDEV_5:
           I2C5_Init(100000UL);                                                 // bus set @ 100Khz
           break;

           default:
           break;
        }
        break;

        default:
        break;
    }
    return true;
}
#if defined(QMP6988_BARO_USED)
/*-----------------------------------------------------------------------------
 *      qmp6988_start_up():  start up the qmp6988 measurement stream
 *
 *  Parameters: baroDev_t *baro
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void qmp6988_start_up(baroDev_t *baro)
{
    if (baro == NULL)
    { /* for sanity */ }
    else
    {
       busWriteRegister(&baro->busdev, QMP6988_CTRL_MEAS_REG, QMP6988_PWR_SAMPLE_MODE);  // start measurement
    }
}
/*-----------------------------------------------------------------------------
 *      qmp6988_get_up():  get up from the qmp6988 measurement stream
 *
 *  Parameters: baroDev_t *baro
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void qmp6988_get_up(baroDev_t *baro)
{
    uint8_t dataV[QMP6988_DATA_FRAME_SIZE];

    if (baro == NULL)
    { /* for misra */ }
    else
    {
       busReadRegisterBuffer(&baro->busdev, QMP6988_PRESSURE_MSB_REG, dataV, QMP6988_DATA_FRAME_SIZE);  // read data from sensor
       baro->dev_up = (int32_t)((((uint32_t)(dataV[0u])) << 16u) | (((uint32_t)(dataV[1u])) << 8u) | ((uint32_t)dataV[2u] ));
       baro->dev_ut = (int32_t)((((uint32_t)(dataV[3u])) << 16u) | (((uint32_t)(dataV[4u])) << 8u) | ((uint32_t)dataV[5u]));
    }
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
/*-----------------------------------------------------------------------------
 *      qmp6988_compensate_T():  qmp6988 temperature compensation for pressure
 *
 *  Parameters: int32_t adc_T, const qmp6988_calib_param_t *qmp6988_cal
 *  Return:     (float64_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE float64_t qmp6988_compensate_T(int32_t adc_T, const qmp6988_calib_param_t *qmp6988_cal)
{
    int32_t var1;
    float64_t T=0.0f;

    if (qmp6988_cal == NULL)
    { /* for misra */ }
    else
    {
      var1=adc_T-1024*1024*8;
      T= qmp6988_cal->Coe_a0+qmp6988_cal->Coe_a1*var1+qmp6988_cal->Coe_a2*var1*var1;
    }
    return T;
}
/*-----------------------------------------------------------------------------
 *      qmp6988_calculate():  qmp6988 calculate the actual pressure and temperature
 *
 *  Parameters: float64_t *pressure, float64_t *temperature, const qmp6988_calib_param_t *qmp6988_cal, const baroDev_t *baro
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void qmp6988_calculate(float64_t *pressure, float64_t *temperature, const qmp6988_calib_param_t *qmp6988_cal, const baroDev_t *baro)
{
    int32_t Dp=0;
    float64_t tr=0.0f;

    if ((baro == NULL) || (qmp6988_cal == NULL))
    {   /* for misra */ }
    else
    {
       tr = qmp6988_compensate_T(baro->dev_ut,qmp6988_cal);
       Dp = baro->dev_up - 1024*1024*8;
       *pressure = qmp6988_cal->Coe_b00+qmp6988_cal->Coe_bt1*tr+qmp6988_cal->Coe_bp1*Dp+qmp6988_cal->Coe_b11*tr*Dp+qmp6988_cal->Coe_bt2*tr*tr+qmp6988_cal->Coe_bp2*Dp*Dp+qmp6988_cal->Coe_b12*Dp*tr*tr+qmp6988_cal->Coe_b21*Dp*Dp*tr+qmp6988_cal->Coe_bp3*Dp*Dp*Dp;
       *temperature = tr/256;
    }
}
/*-----------------------------------------------------------------------------
 *      qmp6988Detect():  qmp6988 get calibration co-efffients start-up measure and
 *                        calculate final result, wait for handshake from caller 
 *                        to re-collect the next data item
 *
 *  Parameters: baroDev_t *baro, qmp6988_calib_param_t *qmp6988_cal, float64_t *press, float64_t *temper
 *  Return:     (bool)
 *----------------------------------------------------------------------------*/
devIC_PR_TE bool qmp6988Detect(baroDev_t *baro, qmp6988_calib_param_t *qmp6988_cal, float64_t *press, float64_t *temper )
{
    uint8_t qmp6988_chip_id = 0u;
    uint8_t databuf[25u] = {0u};
    int16_t Coe_a0_;
    int16_t Coe_a1_;
    int16_t Coe_a2_;
    int16_t Coe_b00_;
    int16_t Coe_bt1_;
    int16_t Coe_bt2_;
    int16_t Coe_bp1_;
    int16_t Coe_b11_;
    int16_t Coe_bp2_;
    int16_t Coe_b12_;
    int16_t Coe_b21_;
    int16_t Coe_bp3_;
    uint16_t lb=0,hb=0;
    uint32_t lw=0,hw=0,temp1,temp2;

    busDevice_t *busdev = &baro->busdev;
    bool defaultAddressApplied = false;

    busInit(busdev);                                                            /* initialise the bus */

    switch(baro->busState)
    {
       case SENSOR_STATE_START_INIT:                                               /* ======= init state get id =========== */
       if ((busdev->bustype == BUSTYPE_I2C) && (busdev->busdev_u.i2c.address == 0))
       {
           busdev->busdev_u.i2c.address = QMP6988_I2C_ADDR;
           defaultAddressApplied = true;
       }
       busReadRegisterBuffer(busdev, QMP6988_CHIP_ID_REG, &qmp6988_chip_id, 1u);   /* read Chip Id */
       if (qmp6988_chip_id != QMP6988_DEFAULT_CHIP_ID)
       {
           if (defaultAddressApplied)
           {
              busdev->busdev_u.i2c.address = 0;
           }
           baro->busState = SENSOR_STATE_WRONG_CHIP;
           break;
       }
       else
       {
         baro->ticksVal = -1;
         calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                 /* initialise the tick reference */
         baro->busState = SENSOR_STATE_SET_IIR;
       }
       /* break;     */

       case SENSOR_STATE_SET_IIR:                                                  /* ======= set IIR =========== */
       busWriteRegister(busdev, QMP6988_SET_IIR_REG, 0x05u);                    // SetIIR
       baro->busState = SENSOR_STATE_SET_IIR;
       /* break;    */

       case SENSOR_STATE_READ_OTP:                                                 /* ======= set OTP =========== */
       calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                   /* get the ticks from now to the tick reference */
       if ( baro->ticksVal >= baro->otp_delay )                                 /* we have waited for the delay */
       {
          busReadRegisterBuffer(busdev, QMP6988_COE_B00_1_REG, databuf, 25u);   //read OTP
          hw = databuf[0u];                                                     //algo OTP
          lw =  databuf[1u];
          temp1 = (hw<<12u) | (lw<<4u);
          hb = databuf[2u];
          lb = databuf[3u];
          Coe_bt1_ = (int16_t)((hb<<8u) | lb);
          hb = databuf[4u];
          lb = databuf[5u];
          Coe_bt2_ = (int16_t)((hb<<8u) | lb);
          hb = databuf[6u];
          lb = databuf[7u];
          Coe_bp1_ = (int16_t)((hb<<8u) | lb);
          hb = databuf[8u];
          lb = databuf[9u];
          Coe_b11_ = (int16_t)((hb<<8u) | lb);
          hb = databuf[10u];
          lb = databuf[11u];
          Coe_bp2_ = (int16_t)((hb<<8u) | lb);
          hb = databuf[12u];
          lb = databuf[13u];
          Coe_b12_ = (int16_t)((hb<<8u) | lb);
          hb = databuf[14u];
          lb = databuf[15u];
          Coe_b21_ = (int16_t)((hb<<8u) | lb);
          hb = databuf[16u];
          lb = databuf[17u];
          Coe_bp3_ = (int16_t)((hb<<8u) | lb);
          hw = databuf[18u];
          lw = databuf[19u];
          temp2 = (hw<<12u) | (lw<<4u);
          hb = databuf[20u];
          lb = databuf[21u];
          Coe_a1_ = (int16_t)((hb<<8u) | lb);
          hb = databuf[22u];
          lb = databuf[23u];
          Coe_a2_ = (int16_t)((hb<<8u) | lb);
          hb = databuf[24u];
          temp1 = temp1|((hb&0xf0u)>>4u);
          if(temp1&0x80000LU)
            Coe_b00_ = ((int16_t)temp1 - (int16_t)0x100000LU);
          else
            Coe_b00_ = temp1;
          temp2 = temp2|(hb&0x0fu);
          if(temp2&0x80000LU)
                 Coe_a0_  = ((int16_t)temp2 - (int16_t)0x100000LU);
          else
             Coe_a0_ = temp2;
          qmp6988_cal->Coe_a0=(float64_t)Coe_a0_/16.0f;
          qmp6988_cal->Coe_a1=(-6.30E-03f)+(4.30E-04f)*(float64_t)(Coe_a1_/INT16_MAX);
          qmp6988_cal->Coe_a2=(-1.9E-11f)+(1.2E-10f)*(float64_t)(Coe_a2_/INT16_MAX);
          qmp6988_cal->Coe_b00 = Coe_b00_/16.0f;
          qmp6988_cal->Coe_bt1 = (1.00E-01f)+(9.10E-02f)*(float64_t)(Coe_bt1_/INT16_MAX);
          qmp6988_cal->Coe_bt2= (1.20E-08f)+(1.20E-06f)*(float64_t)(Coe_bt2_/INT16_MAX);
          qmp6988_cal->Coe_bp1 = (3.30E-02f)+(1.90E-02f)*(float64_t)(Coe_bp1_/INT16_MAX);
          qmp6988_cal->Coe_b11= (2.10E-07f)+(1.40E-07f)*(float64_t)(Coe_b11_/INT16_MAX);
          qmp6988_cal->Coe_bp2 = (-6.30E-10f)+(3.50E-10f)*(float64_t)(Coe_bp2_/INT16_MAX);
          qmp6988_cal->Coe_b12= (2.90E-13f)+(7.60E-13f)*(float64_t)(Coe_b12_/INT16_MAX);
          qmp6988_cal->Coe_b21 = (2.10E-15f)+(1.20E-14f)*(float64_t)(Coe_b21_/INT16_MAX);
          qmp6988_cal->Coe_bp3= (1.30E-16f)+(7.90E-17f)*(float64_t)(Coe_bp3_/INT16_MAX);
          baro->ticksVal = -1;
          calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                /* initialise the tick reference */
          baro->busState = SENSOR_STATE_SET_SAMPTIM;
       }
       break;

       case SENSOR_STATE_SET_SAMPTIM:                                              /* ======= set sampletime =========== */
       calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                   /* calc new tick to now */
       if ( baro->ticksVal >= baro->sam_delay )                                 /* we have waited for the delay */
       {
          busWriteRegister(busdev, QMP6988_CTRL_MEAS_REG, QMP6988_PWR_SAMPLE_MODE);   // Set power mode and sample times
          baro->ticksVal = -1;
          calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                /* initialise the tick reference */
          baro->busState = SENSOR_STATE_START_BARO;
       }
       break;

       case SENSOR_STATE_START_BARO:                                               /* ======= start reading =========== */
       calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                   /* calc new tick to now */
       if ( baro->ticksVal >= baro->start_delay )                               /* we have waited for the delay */
       {
          qmp6988_start_up( baro );                                             // only _up part is executed, and gets both temperature and pressure
          baro->ticksVal = -1;
          calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                /* initialise the tick reference */
          baro->busState = SENSOR_STATE_GET_BARO;
          baro->up_delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1u << QMP6988_TEMPERATURE_OSR) >> 1u) + ((1u << QMP6988_PRESSURE_OSR) >> 1u)) + (QMP6988_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0u) + 15u) / 16u) * 1000u;
       }
       break;

       case SENSOR_STATE_GET_BARO:                                                 /* ======= get readings =========== */
       calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                   /* calc new tick to now */
       if ( baro->ticksVal >= baro->up_delay )                                  /* we have waited for the delay */
       {
           qmp6988_get_up( baro );
           baro->busState = SENSOR_STATE_CALC_BARO;
       }
       break;

       case SENSOR_STATE_CALC_BARO:                                                /* ======= calculate values from readings =========== */
       qmp6988_calculate( press, temper, qmp6988_cal, baro );
       baro->busState = SENSOR_STATE_CALC_DONE;
       break;

       case SENSOR_STATE_CALC_DONE:                                                /* reading has been completed wait for handshake */
       break;

       case SENSOR_STATE_WRONG_CHIP:                                               /* chip seems wrong or corrupt */
       break;

       default:
       break;
    }

    return true;
}
#endif /* QMP6988 baro */
#if defined(FBM320_BARO_USED)
/*-----------------------------------------------------------------------------
 *      Baro_FBM320InitStart():  start the stream with new sensor readings collect
 *                               calculate and wait for handshake to collect the next one
 *
 *  Parameters: fbm320_calibration_t *calibration, baroDev_t *baro, float64_t *press, float64_t *temper
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Baro_FBM320InitStart( fbm320_calibration_t *calibration, baroDev_t *baro, float64_t *press, float64_t *temper )
{
    busDevice_t *busdev = &baro->busdev;
    bool defaultAddressApplied = false;
    uint8_t FBM320_chip_id = 0u;

    busInit(busdev);                                                            /* initialise the bus */

    switch(baro->busState)
    {
       case SENSOR_STATE_START_INIT:                                               /* ======= init state get id =========== */
       if ((busdev->bustype == BUSTYPE_I2C) && (busdev->busdev_u.i2c.address == 0))
       {
           busdev->busdev_u.i2c.address = FBM320_I2C_ADDR;
           defaultAddressApplied = true;
       }
       busReadRegisterBuffer(busdev, FBM320_REG_ID, &FBM320_chip_id, 1u);       /* read Chip Id */
       if (FBM320_chip_id != FBM320_WHOAMI)
       {
           if (defaultAddressApplied)
           {
              busdev->busdev_u.i2c.address = 0;
           }
           baro->busState = SENSOR_STATE_WRONG_CHIP;
           break;
       }
       else
       {
         baro->ticksVal = -1;
         calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                 /* initialise the tick reference */
         baro->busState = SENSOR_STATE_READ_OTP;
       }

       case SENSOR_STATE_READ_OTP:                                                 /* ======= set OTP =========== */
       calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                   /* get the ticks from now to the tick reference */
       if ( baro->ticksVal >= baro->otp_delay )                                 /* we have waited for the delay */
       {
          Baro_FBM320read_cali(calibration, baro);
          baro->ticksVal = -1;
          calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                /* initialise the tick reference */
          baro->busState = SENSOR_STATE_SET_SAMPTIM;
       }

       case SENSOR_STATE_SET_SAMPTIM:
       calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                   /* calc new tick to now */
       if ( baro->ticksVal >= baro->sam_delay )                                 /* we have waited for the delay */
       {
          busWriteRegister(busdev, FBM320_REG_CMD, FBM320_CMD_READ_T);          // Set power mode and sample times
          baro->ticksVal = -1;
          calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                /* initialise the tick reference */
          baro->busState = SENSOR_STATE_START_BARO;
       }
       break;

       case SENSOR_STATE_START_BARO:                                               /* ======= start reading =========== */
       calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                   /* calc new tick to now */
       if ( baro->ticksVal >= baro->start_delay )                               /* we have waited for the delay */
       {
          Baro_FBM320StartStream( baro );
          baro->ticksVal = -1;
          calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                /* initialise the tick reference */
          baro->busState = SENSOR_STATE_GET_BARO;
          baro->up_delay = 100u;                                                 /* start-up delay and read frequency */
       }
       break;

       case SENSOR_STATE_GET_BARO:                                                 /* ======= get readings =========== */
       calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                   /* calc new tick to now or since periodic call */
       if ( baro->ticksVal >= baro->up_delay )                                  /* we have waited for the delay */
       {
           Baro_FBM320GetData( baro, calibration );                             /* get up and ut */
           baro->ticksVal = -1;
           calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                /* initialise the tick reference */
           if (baro->num_of_sums >= BARO_NUM_OF_ITERS)                          /* we want to now get the avaerage and update the data vision */
           {
              baro->busState = SENSOR_STATE_CALC_BARO;
           }
       }
       break;

       case SENSOR_STATE_CALC_BARO:                                                /* ======= calculate values from readings =========== */
       Baro_CalcTempAndPress( baro, press, temper );
       baro->busState = SENSOR_STATE_CALC_DONE;
       break;

       case SENSOR_STATE_CALC_DONE:                                                /* reading has been completed wait for handshake */
       break;

       case SENSOR_STATE_WRONG_CHIP:                                               /* chip seems wrong or corrupt */
       break;

       default:
       break;
    }
}
 /*-----------------------------------------------------------------------------
 *      Baro_FBM320read_cali():  qmp6988 read calibration data
 *
 *  Parameters: fbm320_calibration_t *calibration, const baroDev_t *baro
 *  Return:     (bool)
 *----------------------------------------------------------------------------*/
devIC_PR_TE bool Baro_FBM320read_cali(fbm320_calibration_t *calibration, const baroDev_t *baro)
{
    uint8_t tmp[2u];
    uint16_t R[10u];
    uint8_t i=0;

    if ((calibration == NULL) || (baro == NULL))
    { 
       return false; 
    }
    else
    {
       for (i=0u; i<9u; i++)
       {
           if (!busReadRegisterBuffer(&baro->busdev, 0xAAu+(i*2), &tmp[0u], 1u))
           {
               return false;
           }
           if (!busReadRegisterBuffer(&baro->busdev, 0xAB+(i*2), &tmp[1u], 1u))
           {
               return false;
           }
           R[i] = ((uint8_t)tmp[0] << 8 | tmp[1]);
       }
       if (!busReadRegisterBuffer(&baro->busdev, 0xA4u, &tmp[0u], 1u))
       {
          return false;
       }
       if (!busReadRegisterBuffer(&baro->busdev, 0xF1u, &tmp[1u], 1u))
       {
         return false;
       }
       R[9u] = ((uint8_t)tmp[0u] << 8u) | tmp[1u];
       calibration->C0 = R[0u] >> 4u;                                              /*    Use R0~R9 calculate C0~C12 of FBM320-02    */
       calibration->C1 = ((R[1u] & 0xFF00) >> 5u) | (R[2u] & 7u);
       calibration->C2 = ((R[1u] & 0xFFu) << 1u) | (R[4u] & 1u);
       calibration->C3 = R[2u] >> 3u;
       calibration->C4 = ((uint32_t)R[3u] << 2u) | (R[0u] & 3u);
       calibration->C5 = R[4u] >> 1u;
       calibration->C6 = R[5u] >> 3u;
       calibration->C7 = ((uint32_t)R[6u] << 3u) | (R[5u] & 7u);
       calibration->C8 = R[7u] >> 3u;
       calibration->C9 = R[8u] >> 2u;
       calibration->C10 = ((R[9u] & 0xFF00u) >> 6u) | (R[8u] & 3u);
       calibration->C11 = R[9u] & 0xFFu;
       calibration->C12 = ((R[0u] & 0x0Cu) << 1u) | (R[7u] & 7u);
    }
    return true;
}

 /*-----------------------------------------------------------------------------
 *      Baro_FBM320calc_PT():  calculate corrected pressure and temperature
 *
 *  Parameters: int32_t UT, int32_t UP, int32_t *pressure, int32_t *temperature, const fbm320_calibration_t *cal
 *  Return:     (bool)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Baro_FBM320calc_PT(int32_t UT, int32_t UP, int32_t *pressure, int32_t *temperature, const fbm320_calibration_t *cal)
{
    int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;

    if (cal == NULL)
    { /* for misra */ }
    else
    {
       DT  = ((UT - 8388608) >> 4) + (cal->C0 << 4);
       X01 = (cal->C1 + 4459) * DT >> 1;
       X02 = ((((cal->C2 - 256) * DT) >> 14) * DT) >> 4;
       X03 = (((((cal->C3 * DT) >> 18) * DT) >> 18) * DT);
       *temperature = ((2500 << 15) - X01 - X02 - X03) >> 15;
       DT2 = (X01 + X02 + X03) >> 12;
       X11 = ((cal->C5 - 4443) * DT2);
       X12 = (((cal->C6 * DT2) >> 16) * DT2) >> 2;
       X13 = ((X11 + X12) >> 10) + ((cal->C4 + 120586) << 4);
       X21 = ((cal->C8 + 7180) * DT2) >> 10;
       X22 = (((cal->C9 * DT2) >> 17) * DT2) >> 12;
       X23 = (X22 >= X21) ? (X22 - X21) : (X21 - X22);
       X24 = (X23 >> 11) * (cal->C7 + 166426);
       X25 = ((X23 & 0x7FF) * (cal->C7 + 166426)) >> 11;
       X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 11) + cal->C7 + 166426) : (((X24 + X25) >> 11) + cal->C7 + 166426);
       PP1 = ((UP - 8388608) - X13) >> 3;
       PP2 = (X26 >> 11) * PP1;
       PP3 = ((X26 & 0x7FF) * PP1) >> 11;
       PP4 = (PP2 + PP3) >> 10;
       CF  = (2097152 + cal->C12 * DT2) >> 3;
       X31 = (((CF * cal->C10) >> 17) * PP4) >> 2;
       X32 = (((((CF * cal->C11) >> 15) * PP4) >> 18) * PP4);
       *pressure = ((X31 + X32) >> 15) + PP4 + 99880;
    }
}
/*-----------------------------------------------------------------------------
 *      Baro_FBM320StartStream():  start the stream with new sensor readings
 *
 *  Parameters: baroDev_t *baro
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Baro_FBM320StartStream( baroDev_t *baro )
{
    if (baro == NULL)
    { /* for misra */ }
    else
    {
      baro->pressure_sum = 0.0f;
      baro->temperature_sum = 0.0f;                                             // sum and convert to degrees
      baro->num_of_sums = 0lu;
      baro->busState1 = 0u;
      busWriteRegister(&baro->busdev, FBM320_REG_CMD, FBM320_CMD_READ_T);       // Set power mode and sample times
      busWriteRegister(&baro->busdev, FBM320_REG_CMD, FBM320_CMD_READ_P);       // Set power mode and sample times
    }
}

 /*-----------------------------------------------------------------------------
 *      Baro_FBM320GetData():  acumulate a new sensor reading
 *
 *  Parameters: baroDev_t *baro, const fbm320_calibration_t *cal
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Baro_FBM320GetData( baroDev_t *baro, const fbm320_calibration_t *cal )
{
    uint8_t buf[FBM320_DATA_FRAME_SIZE];
    int32_t value;
    int32_t pressure, temperature;

    if ((baro == NULL) || (cal == NULL))
    { /* for misra */ }
    else
    {
       busReadRegisterBuffer(&baro->busdev, FBM320_PRESSURE_MSB_REG, buf, FBM320_DATA_FRAME_SIZE);  // read data from sensor
       value = ((uint32_t)buf[0u] << 16u) | ((uint32_t)buf[1u] << 8u) | (uint32_t)buf[2u];
       if (baro->busState1 == 0)
       {
           baro->dev_ut = value;
       }
       else
      {
          Baro_FBM320calc_PT(baro->dev_ut, value, &pressure, &temperature, cal);
/*        if (pressure_ok(pressure))
        {                      */
          baro->pressure_sum += pressure;
          baro->temperature_sum += temperature*0.01f;                         // sum and convert to degrees
          baro->num_of_sums=++baro->num_of_sums % UINT64_MAX;
/*        }     */
       }
       baro->busState1 = ++baro->busState1 % UINT8_MAX;
       if (baro->busState1 >= 5u)
       {
          busWriteRegister(&baro->busdev, FBM320_REG_CMD, FBM320_CMD_READ_T);   // Set power mode and sample times
          baro->busState1 = 0u;
        }
        else
        {
           busWriteRegister(&baro->busdev, FBM320_REG_CMD, FBM320_CMD_READ_P);   // Set power mode and sample times
        }
    }
}
#endif /* end FBM320 */
/*-----------------------------------------------------------------------------
 *      Baro_CalcTempAndPress():  calculate and use the FBM320 data as an avaerage then reset the barometer object
 *
 *  Parameters: baroDev_t *baro, float64_t *pressure, float64_t *temperature
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Baro_CalcTempAndPress( baroDev_t *baro, float64_t *pressure, float64_t *temperature )
{
    if (baro == NULL)
    { /* for misra */ }
    else
    {
       *pressure = baro->pressure_sum / baro->num_of_sums;
       *temperature = baro->temperature_sum / baro->num_of_sums;
       baro->pressure_sum = 0u;
       baro->temperature_sum = 0u;
       baro->num_of_sums = 0u;
    }
}

#if defined(KELLER_BARO_USED)
/*-----------------------------------------------------------------------------
 *      Baro_KellerStart():  start a new sensor reading stream
 *
 *  Parameters: baroDev_t *baro
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Baro_KellerStart( baroDev_t *baro )
{
    if (baro == NULL)
    { /* for misra */ }
    else
    {
       baro->pressure_sum = 0u;
       baro->temperature_sum = 0u;
       baro->num_of_sums = 0u;
       busWriteRegister(&baro->busdev, KELLER_REQUEST_MEASUREMENT, KELLER_REQUEST_VAL);   // Set power mode and sample times
    }
}
/*-----------------------------------------------------------------------------
 *      Baro_KellerGetData():  acumulate a new sensor reading
 *
 *  Parameters: baroDev_t *baro
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Baro_KellerGetData( baroDev_t *baro )
{
    uint8_t buf[KELLER_DATA_FRAME_SIZE];
    uint8_t status;

    if (baro == NULL)
    { /* for misra */ }
    else
    {
       busReadRegisterBuffer(&baro->busdev, 0u, buf, KELLER_DATA_FRAME_SIZE);   // read data from sensor 5 bytes

       status = buf[0u];                                                        /* TODO check the status word and branch if error */
       baro->dev_up = (buf[1u] << 8u) | buf[2u];
       baro->dev_ut = (buf[3u] << 8u) | buf[4u];
    
       if (baro->dev_up>0)
       {
           baro->pressure_sum += baro->dev_up;
           baro->temperature_sum += baro->dev_ut;
           baro->num_of_sums=++baro->num_of_sums % UINT64_MAX;
       }
       busWriteRegister(&baro->busdev, KELLER_REQUEST_MEASUREMENT, KELLER_REQUEST_VAL);   // Set power mode and sample times
    }
}
/*-----------------------------------------------------------------------------
 *      Baro_KellerCalcul(): calculate and use the Keller data as an avaerage then reset the barometer object
 *
 *  Parameters: baroDev_t *baro, float64_t *pressure, float64_t *temperature, const keller_calibration_t *cal
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Baro_KellerCalcul( baroDev_t *baro, float64_t *pressure, float64_t *temperature, const keller_calibration_t *cal )
{
    float64_t pressure1;
    float64_t temperature1;
    
    if (((baro == NULL) || (baro->num_of_sums == 0ULL)) || (cal == NULL))
    { /* for misra */ }
    else
    {
       pressure1 = baro->pressure_sum / baro->num_of_sums;
       temperature1 = baro->temperature_sum / baro->num_of_sums;
       // per datasheet
       if (cal->calIntegrity==1u)                                               /* calibration integrity is good so do compensation */
       {
          pressure1 = (pressure1 - 16384.0f) * (cal->kellcal_max.p_max - cal->kellcal_min.p_min) / 32768.0f + cal->kellcal_min.p_min;
       }
       pressure1 *= 100000.0f;                                                  // bar -> Pascal
       *pressure = pressure1 + 101300.0f;                                       // MSL pressure offset
       //*temperature = ((float64_t)((ROUND(temperature1,0u) >> 4u) - 24.0f) * 0.05f - 50.0f;
       *temperature = ((temperature1 / pow(2u,4u)) - 24.0f) * 0.05f - 50.0f;
       baro->pressure_sum = 0u;
       baro->temperature_sum = 0u;
       baro->num_of_sums = 0u;
    }
}
/*-----------------------------------------------------------------------------
 *      Baro_KellerInitStart(): initialize the keller probe read and calculate the nwait for handshake
 *                              to start collecting the next reading
 *
 *  Parameters: keller_calibration_t *calibration, baroDev_t *baro, float64_t *press, float64_t *temper
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Baro_KellerInitStart( keller_calibration_t *calibration, baroDev_t *baro, float64_t *press, float64_t *temper )
{
    busDevice_t *busdev = &baro->busdev;
    uint16_t calTemp1;
    uint8_t tmp[3u];
    /* uint8_t KELLER_chip_id = 0u;  not in code maybe we need to add this */

    if ((calibration == NULL) || (baro == NULL))
    { /* for misra */ }
    else
    {
       busInit(busdev);                                                         /* initialise the bus */

       switch(baro->busState)
       {
          case SENSOR_STATE_START_INIT:                                            /* ======= init state get id =========== */
          baro->otp_delay = 10u;                                                /* states driver needs to wait between commands */
          baro->sam_delay = 10u;                                                /* states driver needs to wait between commands */
          baro->ticksVal = -1;
          calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                 /* initialise the tick reference */
          baro->busState = SENSOR_STATE_CALIB_MIN1;

          case SENSOR_STATE_CALIB_MIN1:                                            /* ======= read calibration part No.1 =========== */
          calculateTick2Now( &baro->ticksVal, &baro->ticksRef );                /* get the ticks from now to the tick reference */
          if ( baro->ticksVal >= baro->otp_delay )                              /* we have waited for the delay */
          {
             if (busReadRegisterBuffer(busdev, KELLER_PRANGE_MIN_MSB, &tmp[0u], 3u))
             {
                calibration->calIntegrity &= tmp[0u];
                calTemp1 = (tmp[1u] << 8u) | tmp[2u];
             }
             baro->ticksVal = -1;
             calculateTick2Now( &baro->ticksVal, &baro->ticksRef );             /* initialise the tick reference */
             baro->busState = SENSOR_STATE_CALIB_MIN2;
           }

           case SENSOR_STATE_CALIB_MIN2:
           calculateTick2Now( &baro->ticksVal, &baro->ticksRef );               /* calc new tick to now */
           if ( baro->ticksVal >= baro->sam_delay )                             /* we have waited for the delay */
           {
              if (busReadRegisterBuffer(&baro->busdev, KELLER_PRANGE_MIN_LSB, &tmp[0u], 3u))
              {
                 calibration->calIntegrity &= tmp[0u];
                 calibration->kellcal_min.cal_data =  (calTemp1 << 16u) | (tmp[1u] << 8u) | tmp[2u];
              }
              baro->ticksVal = -1;
              calculateTick2Now( &baro->ticksVal, &baro->ticksRef );            /* initialise the tick reference */
              baro->busState = SENSOR_STATE_CALIB_MAX1;
           }
           break;
       
           case SENSOR_STATE_CALIB_MAX1:                                           /* ======= read calibration part No.1 =========== */
           calculateTick2Now( &baro->ticksVal, &baro->ticksRef );               /* get the ticks from now to the tick reference */
           if ( baro->ticksVal >= baro->otp_delay )                             /* we have waited for the delay */
           {
              if (busReadRegisterBuffer(&baro->busdev, KELLER_PRANGE_MAX_MSB, &tmp[0u], 3u))
              {
                 calibration->calIntegrity &= tmp[0u];
                 calTemp1 = (tmp[1u] << 8u) | tmp[2u];
              }
              baro->ticksVal = -1;
              calculateTick2Now( &baro->ticksVal, &baro->ticksRef );            /* initialise the tick reference */
              baro->busState = SENSOR_STATE_CALIB_MAX2;
            }

            case SENSOR_STATE_CALIB_MAX2:
            calculateTick2Now( &baro->ticksVal, &baro->ticksRef );              /* calc new tick to now */
            if ( baro->ticksVal >= baro->sam_delay )                            /* we have waited for the delay */
            {
               if (busReadRegisterBuffer(&baro->busdev, KELLER_PRANGE_MAX_LSB, &tmp[0u], 3u))
               {
                  calibration->calIntegrity &= tmp[0u];
                  calibration->kellcal_max.cal_data =  (calTemp1 << 16u) | (tmp[1u] << 8u) | tmp[2u];
               }
               baro->ticksVal = -1;
               calculateTick2Now( &baro->ticksVal, &baro->ticksRef );           /* initialise the tick reference */
               baro->busState = SENSOR_STATE_START_BARO;
            }
            break;

            case SENSOR_STATE_START_BARO:                                          /* ======= start reading =========== */
            calculateTick2Now( &baro->ticksVal, &baro->ticksRef );              /* calc new tick to now */
            if ( baro->ticksVal >= baro->start_delay )                          /* we have waited for the delay */
            {
               Baro_KellerStart( baro );
               baro->ticksVal = -1;
               calculateTick2Now( &baro->ticksVal, &baro->ticksRef );           /* initialise the tick reference */
               baro->busState = SENSOR_STATE_GET_BARO;
               baro->up_delay = 100u;                                           /* start-up delay and read frequency */
             }
             break;

             case SENSOR_STATE_GET_BARO:                                           /* ======= get readings =========== */
             calculateTick2Now( &baro->ticksVal, &baro->ticksRef );             /* calc new tick to now or since periodic call */
             if ( baro->ticksVal >= baro->up_delay )                            /* we have waited for the delay */
             {
                Baro_KellerGetData( baro );                                     /* get up and ut */
                baro->ticksVal = -1;
                calculateTick2Now( &baro->ticksVal, &baro->ticksRef );          /* initialise the tick reference */
                if (baro->num_of_sums >= BARO_NUM_OF_ITERS)                     /* we want to now get the avaerage and update the data vision */
                {
                   baro->busState = SENSOR_STATE_CALC_BARO;
                }
             }
             break;

             case SENSOR_STATE_CALC_BARO:                                          /* ======= calculate values from readings =========== */
             Baro_KellerCalcul( baro, press, temper, calibration );
             baro->busState = SENSOR_STATE_CALC_DONE;
             break;

             case SENSOR_STATE_CALC_DONE:                                          /* reading has been completed wait for handshake */
             break;

             case SENSOR_STATE_WRONG_CHIP:                                         /* chip seems wrong or corrupt */
             break;

             default:
             break;
        }
    }
}
#endif /* end keller */
/*-----------------------------------------------------------------------------
 *      baroCalculateAltitude(): calculate altitude from the pressure
 *
 *  Parameters: baro_t *baro, float64_t baroPressure
 *  Return:     (uint32_t)
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint32_t baroCalculateAltitude( baro_t *baro, float64_t baroPressure )
{
    baroFloatConvert_t BaroAlt_tmp, BaroAlt_tmp1;                               /* incase you want the integer representaion to send over comms */
    uint32_t ret=0;
    
    if (baro == NULL)
    { /*just for misra */ }
    else
    {
       /* calculates height from ground via baro readings
          see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140 */
       BaroAlt_tmp.baroFloatConvert_u.f  = ((1.0f - pow(baroPressure / 101325.0f, 0.190295f)) * 4433000.0f);
       BaroAlt_tmp.baroFloatConvert_u.f -= baro->baroGroundAltitude;
       BaroAlt_tmp1.baroFloatConvert_u.f = ((float32_t)baro->BaroAlt) * baro->baro_noise_lpf + BaroAlt_tmp.baroFloatConvert_u.f * (1.0f - baro->baro_noise_lpf); // additional LPF to reduce baro noise
       baro->BaroAlt = BaroAlt_tmp1.baroFloatConvert_u.f;                       /* change to u.i if you want the integer representation for coms */
       ret = BaroAlt_tmp1.baroFloatConvert_u.u;                                 /* returns the unsigned integer value for the float32_t altitude calculation */
    }
    return ret;
}
/*-----------------------------------------------------------------------------
 *      baroSetCalibrationCycles(): set the number of calibration cycles
 *
 *  Parameters: uint16_t calibrationCyclesRequired, baro_t *baro
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void baroSetCalibrationCycles( uint16_t calibrationCyclesRequired, baro_t *baro )
{
    if (baro == NULL)
    { /*just for misra */ }
    else
    {
      baro->cali_cycles = calibrationCyclesRequired;
      baro->savedGroundPressure = 0u;
      baro->baro_noise_lpf = 600;
      baro->baro_cf_vel = 985;
      baro->baro_cf_alt = 965;
    }
}
/*-----------------------------------------------------------------------------
 *      performBaroCalibrationCycle(): perform calibration cycle
 *
 *  Parameters: float64_t baroPressure, baro_t *baro
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void performBaroCalibrationCycle( float64_t baroPressure, baro_t *baro )
{
    int32_t baroGroundPressure = 8*101325;
    
    if (baro == NULL)
    { /*just for misra */ }
    else
    {
       baroGroundPressure -= baroGroundPressure / 8;
       baroGroundPressure += baroPressure;
       baro->baroGroundAltitude = (1.0f - pow((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;

       if (baroGroundPressure == baro->savedGroundPressure)                     /* stability then finish the calibration */
          baro->cali_cycles = 0;
       else
       {
          baro->cali_cycles--;
          baro->savedGroundPressure = baroGroundPressure;
       }
    }
}
#if defined(HMC6352_COMPASS_USED)
/*-----------------------------------------------------------------------------
 *      Compass_HMC6352Start():  start a new sensor reading stream
 *
 *  Parameters: Compass_t *compass
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Compass_HMC6352Start( compass_t *compass )
{
    if (compass == NULL)
    { /* for misra */ }
    else
    {
       compass->heading = 0.0f;
       compass->heading_sum = 0.0f;
       compass->num_of_sums = 0lu;
       compass->avg_heading = 0.0f;
       if ((compass->busdev.bustype == BUSTYPE_I2C) && (compass->busdev.busdev_u.i2c.address == 0))
       {
           compass->busdev.busdev_u.i2c.address = HMC6352ADDR;
       }
       busWriteRegister(&compass->busdev, (uint8_t)HMC6352ADDR, (uint8_t)HMC_DAT_REQ);   // Set power mode and sample times
    }
}
/*-----------------------------------------------------------------------------
 *      Compass_HMC6352GetData():  acumulate a new sensor reading
 *
 *  Parameters: compass_t *compass
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Compass_HMC6352GetData( compass_t *compass )
{
    uint8_t buf[HMC6352_DATA_FRAME_SIZE];
    //uint8_t status;

    if (compass == NULL)
    { /* for misra */ }
    else
    {
       busWriteRegister(&compass->busdev, (uint8_t) HMC6352ADDR,(uint8_t) HMC_DAT_REQ);   // Set power mode and sample times

       busReadRegisterBuffer(&compass->busdev, 0u, buf, HMC6352_DATA_FRAME_SIZE);   // read data from sensor 5 bytes

       compass->heading = ((float32_t)(((uint16_t)buf[1u]) | ((uint16_t)buf[0u])<<8u)) / 10.0f;

       if (compass->heading>0)
       {
           compass->heading_sum += compass->heading;
           compass->num_of_sums=++compass->num_of_sums % UINT32_MAX;
           if (compass->num_of_sums>HMC6352_SAMPLED_ENOUGH)
           {
              compass->avg_heading = compass->heading_sum / compass->num_of_sums;
              compass->num_of_sums = 0u;
           }
       }
    }
}
#endif
/* ======================== Liddar Lite =================================== */
#if defined(LIDAR_LITESTART_USED)
/*-----------------------------------------------------------------------------
 *      Lidar_LiteStart():  start a new sensor reading stream
 *
 *  Parameters: dist_t *dist
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Lidar_LiteStart( distance_t *dist )
{
    if (dist == NULL)
    { /* for misra */ }
    else
    {
       dist->distance = 0.0f;
       dist->distance_sum = 0.0f;
       dist->num_of_sums = 0lu;
       dist->avg_distance = 0.0f;
       if ((dist->busdev.bustype == BUSTYPE_I2C) && (dist->busdev.busdev_u.i2c.address == 0))
       {
           dist->busdev.busdev_u.i2c.address = LIDDARLITEADDR;
       }
       busWriteRegister(&dist->busdev, LIDDARLITEADDR, LL_DATA1);   // Set power mode and sample times
    }
}
/*-----------------------------------------------------------------------------
 *      Lidar_LiteGetData():  acumulate a new sensor reading
 *
 *  Parameters: dist_t *dist
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Lidar_LiteGetData( distance_t *dist )
{
    uint8_t buf[LL_DATA_FRAME_SIZE*2u];
    //uint8_t status;

    if (dist == NULL)
    { /* for misra */ }
    else
    {
        switch(dist->state)
        {
        
          case 0u:
          busWriteRegister(&dist->busdev, LIDDARLITEADDR, LL_DATA1);
          dist->state = 1u;
          break;

          case 1u:
          busWriteRegister(&dist->busdev, LIDDARLITEADDR, LL_DATA2);
          dist->ticksVal = -1;
          calculateTick2Now( &dist->ticksVal, &dist->ticksRef );                 /* initialise the tick reference */
          dist->state = 2u;
          break;

          case 2u:
          calculateTick2Now( &dist->ticksVal, &dist->ticksRef );
          if (dist->ticksVal > LL_DELAY1 )
          {
            busWriteRegister(&dist->busdev, LIDDARLITEADDR, LL_DATA3);
            dist->state = 3u;
          }
          break;

          case 3u:
          busReadRegisterBuffer(&dist->busdev, 0u, &buf[0], LL_DATA_FRAME_SIZE);
          dist->state = 4u;
          break;

         case 4u:
          busWriteRegister(&dist->busdev, LIDDARLITEADDR, LL_DATA4);
          dist->state = 5u;
          break;

          case 5u:
          busReadRegisterBuffer(&dist->busdev, 0u, &buf[2u], LL_DATA_FRAME_SIZE);
          dist->distance = (((uint16_t)(buf[0u]) & 0xff) << 8u) | ((uint16_t)(buf[2u]) & 0xff);
          dist->state = 6u;
          break;

          case 6u:
          if (dist->distance>0)
          {
             dist->distance_sum += dist->distance;
             dist->num_of_sums=++dist->num_of_sums % UINT32_MAX;
             if (dist->num_of_sums>LL_SAMPLED_ENOUGH)
             {
                dist->avg_distance = dist->distance_sum / dist->num_of_sums;
                dist->num_of_sums = 0u;
             }
          }
          dist->state = 7u;
          break;

          case 7u:
          calculateTick2Now( &dist->ticksVal, &dist->ticksRef );
          if (dist->ticksVal > LL_DELAY2 )
          {
             dist->state = 0u;
          }
          break;
      }

    }
}
#endif
#if defined(BH1750_LUX_USED)
/* ================== BH1750 Lux Sensor IC =================================== */
/*-----------------------------------------------------------------------------
 *      Lux_BH1750Start():  start a new sensor reading stream
 *
 *  Parameters: lux_t *luxobj
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Lux_BH1750Start( lux_t *luxobj )
{
    if (luxobj == NULL)
    { /* for misra */ }
    else
    {
       luxobj->lux = 0.0f;
       luxobj->lux_sum = 0.0f;
       luxobj->num_of_sums = 0lu;
       luxobj->avg_lux = 0.0f;
       if ((luxobj->busdev.bustype == BUSTYPE_I2C) && (luxobj->busdev.busdev_u.i2c.address == 0))
       {
           luxobj->busdev.busdev_u.i2c.address = BH1750_ADDR;
       }
       busWriteRegister(&luxobj->busdev, BH1750_ADDR, BH1750_POWER_ON);   // Set power mode and sample times
    }
}
/*-----------------------------------------------------------------------------
 *      Lux_BH1750GetData():  acumulate a new sensor reading
 *
 *  Parameters: luxobj_t *luxobj
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void Lux_BH1750GetData( lux_t *luxobj )
{
    uint8_t buf[BH1750_DATA_FRAME_SIZE];
    //uint8_t status;

    if (luxobj == NULL)
    { /* for misra */ }
    else
    {
      switch(luxobj->state)
      {
          case 0u:
          luxobj->ticksVal = -1;
          calculateTick2Now( &luxobj->ticksVal, &luxobj->ticksRef );            /* initialise the tick reference */
          luxobj->state = 1u;
          break;

          case 1u:
          calculateTick2Now( &luxobj->ticksVal, &luxobj->ticksRef );
          if (luxobj->ticksVal > BH1750_DELAY1 )
          {
            luxobj->ticksRef = CP0_GET(CP0_COUNT);
            luxobj->state = 2u;
          }
          break;

          case 2u:
          busWriteRegister(&luxobj->busdev, BH1750_ADDR, BH1750_CONTINUOUS_HIGH_RES_MODE);
          luxobj->state = 3u;
          break;

          case 3u:
          busReadRegisterBuffer(&luxobj->busdev, 0u, &buf[0u], BH1750_DATA_FRAME_SIZE);
          luxobj->state = 4u;
          break;

          case 5u:
          luxobj->lux = (float32_t)((((uint16_t)(buf[0u]) & 0xff) << 8u) | ((uint16_t)(buf[1u]) & 0xff)) / 1.2f;
          luxobj->state = 6u;
          break;

          case 6u:
          if (luxobj->lux>0)
          {
             luxobj->lux_sum += luxobj->lux;
             luxobj->num_of_sums=++luxobj->num_of_sums % UINT32_MAX;
             if (luxobj->num_of_sums>BH1750_SAMPLED_ENOUGH)
             {
                luxobj->avg_lux = luxobj->lux_sum / luxobj->num_of_sums;
                luxobj->lux_sum = 0u;
                luxobj->num_of_sums = 0u;
             }
          }
          luxobj->state = 7u;
          break;

          case 7u:
          calculateTick2Now( &luxobj->ticksVal, &luxobj->ticksRef );
          if (luxobj->ticksVal > BH1750_DELAY2 )
          {
             luxobj->ticksRef = CP0_GET(CP0_COUNT);
             luxobj->state = 3u;
          }
          break;
       }

    }
}
#endif  /* end if lux */
#if defined(HTU21D_HUMID_USED)
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2016 Eadf, Jeff Rowberg 
ported to pic32 mikroE C by ACP aviation */
/*-----------------------------------------------------------------------------
 *      HUM_htu21dStart():  start a new sensor reading stream
 *
 *  Parameters: htu21d_t *huMobj
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void HUM_htu21dStart( htu21d_t *huMobj )
{
    if (huMobj == NULL)
    { /* for misra */ }
    else
    {
       huMobj->temperat = 0.0f;                                                 
       huMobj->temperat_sum = 0.0f;
       huMobj->num_of_sums = 0lu;
       huMobj->avg_temperat = 0.0f;
       huMobj->humid = 0.0f;                                                 
       huMobj->humid_sum = 0.0f;
       huMobj->num_of_sums1 = 0lu;
       huMobj->avg_humid = 0.0f;
       if ((huMobj->busdev.bustype == BUSTYPE_I2C) && (huMobj->busdev.busdev_u.i2c.address == 0))
       {
           huMobj->busdev.busdev_u.i2c.address = HTU21D_DEFAULT_ADDRESS;
       }
    }
}

/*-----------------------------------------------------------------------------
 *      HUM_htu21dUpdate():  update humidity monitor ic information
 *
 *  Parameters: htu21d_t *huMobj
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void HUM_htu21dUpdate( htu21d_t *huMobj )
{
   uint8_t buf[9u];
   uint16_t u16ReadonI2C;
   
   switch (huMobj->state)
   {
      case 0u:
      HUM_htu21dStart( huMobj );
      break;
                  
      case 1u:
      busReadRegisterBuffer(&huMobj->busdev, HTU21D_READ_USER_REGISTER, &buf[0u], 1u); 
      if (buf[0] == 0x2u) huMobj->state = 2u;
      break;

      case 2u:
      busReadRegisterBuffer(&huMobj->busdev, HTU21D_RA_TEMPERATURE, &buf[0u], 2u); 
      memcpy((void*)&u16ReadonI2C,(void*)&buf,sizeof(u16ReadonI2C));
      huMobj->temperat = ((float32_t)(u16ReadonI2C&0xFFFCu))*175.72f/65536.0f-46.85f;
      huMobj->state = 3u;
      break;
           
      case 3u:
      busReadRegisterBuffer(&huMobj->busdev, HTU21D_RA_HUMIDITY, &buf[0u], 2u); 
      memcpy((void*)&u16ReadonI2C,(void*)&buf,sizeof(u16ReadonI2C));
      huMobj->humid = ((float32_t)(u16ReadonI2C&0xFFFCu))*125.0f/65536.0f-6.0f;
      huMobj->state = 4u;
      break;           

      case 4u:
      if (huMobj->temperat > 0)
      {
         huMobj->temperat_sum += huMobj->temperat;
         huMobj->num_of_sums=++huMobj->num_of_sums % UINT32_MAX;
         if (huMobj->num_of_sums > HTU21D_SAMPLED_ENOUGH)
         {
            huMobj->avg_temperat = huMobj->temperat_sum / huMobj->num_of_sums;
            huMobj->temperat_sum = 0u;
            huMobj->num_of_sums = 0u;
         }
       }
       if (huMobj->humid > 0)
       {
          huMobj->humid_sum += huMobj->humid;
          huMobj->num_of_sums1=++huMobj->num_of_sums1 % UINT32_MAX;
          if (huMobj->num_of_sums1 > HTU21D_SAMPLED_ENOUGH)
          {
            huMobj->avg_humid = huMobj->humid_sum / huMobj->num_of_sums1;
            huMobj->humid_sum = 0u;
            huMobj->num_of_sums1 = 0u;
          }
       }
       huMobj->state = 2u;
       break;
                  
       default:
       break;
   }          
}

#endif /* htu21d */

#if defined(WANT_IAQ2000_Co2)
// I2Cdev library collection - iAQ-2000 I2C device class
// Based on AppliedSensor iAQ-2000 Interface Description, Version PA1, 2009
// 2012-04-01 by Peteris Skorovs <pskorovs@gmail.com>
//
// This I2C device library is using (and submitted as a part of) Jeff Rowberg's I2Cdevlib library,
// which should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-04-01 - initial release
//     2012-11-08 - added TVoc and Status
//     Oct 2000 : ported to pic32 FT900 mikroE C by ACP Aviation

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Peteris Skorovs, Jeff Rowberg
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */
/*-----------------------------------------------------------------------------
 *      IAQ_2000Start():  start a new sensor reading stream
 *
 *  Parameters: iaq2000_t *iaQobj
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void IAQ_2000Start( iaq2000_t *iaQobj )
{
    if (iaQobj == NULL)
    { /* for misra */ }
    else
    {
       iaQobj->Iaqpred = 0.0f;                                                  /* preedicted co2 conc */
       iaQobj->Iaqpred_sum = 0.0f;
       iaQobj->num_of_sums = 0lu;
       iaQobj->avg_Iaqpred = 0.0f;
       if ((iaQobj->busdev.bustype == BUSTYPE_I2C) && (iaQobj->busdev.busdev_u.i2c.address == 0))
       {
           iaQobj->busdev.busdev_u.i2c.address = IAQ2000_DEFAULT_ADDRESS;
       }
    }
}
/*-----------------------------------------------------------------------------
 *      IAQ_2000Update():  update co2 monitor ic information
 *
 *  Parameters: iaq2000_t *iaQobj
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void IAQ_2000Update( iaq2000_t *iaQobj )
{
   uint8_t buf[9u];
   
   switch (iaQobj->state)
   {
          case 0u:
          IAQ_2000Start( iaQobj );
          break;
                  
          case 1u:
          busReadRegisterBuffer(&iaQobj->busdev, IAQ2000_RA_DATA1, &buf[0u], 9u); /* read bytes from the DATA0 to DATA8 registers and bit-shifting them into a 16-bit value  */
          iaQobj->state = 2u;
          break;

          case 2u:
          iaQobj->Iaqpred = ((buf[0u] << 8u) | buf[1u]);                        /* read bytes from the DATA0 AND DATA1 registers and bit-shifting them into a 16-bit value */
          iaQobj->state = 3u;
          break;

          case 3u:
          if (iaQobj->Iaqpred>0)
          {
             iaQobj->Iaqpred_sum += iaQobj->Iaqpred;
             iaQobj->num_of_sums=++iaQobj->num_of_sums % UINT32_MAX;
             if (iaQobj->num_of_sums>IAQ2000_SAMPLED_ENOUGH)
             {
                iaQobj->avg_Iaqpred = iaQobj->Iaqpred_sum / iaQobj->num_of_sums;
                iaQobj->Iaqpred_sum = 0u;
                iaQobj->num_of_sums = 0u;
             }
          }
          iaQobj->state = 4u;
          break;
                  
          case 4u:
          iaQobj->Iaqstatus = buf[2u];                                          /* read bytes from the DATA2 register  */
          iaQobj->state = 5u;
          break;
                  
          case 5u:
          iaQobj->Iaqtvoc = ((buf[7u] << 8u) | buf[8u]);                        /* read bytes from the DATA7 AND DATA8 registers and bit-shifting them into a 16-bit value */
          iaQobj->state = 1u;
          break;
                  
          default:
          break;
   }          
}
#endif /* iaq2000 co2 monitor */

#if defined(INA3221_POWER_USED)
/* ======================== Power IC ======================================= */
/*-----------------------------------------------------------------------------
 *      INA3221_Init():  power current and volts IC set-up
 *
 *  Parameters: powerIV_IC_t *iv
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void INA3221_Init( powerIV_IC_t *vi )
{
    if (vi == NULL)
    { /* for misra */ }
    else
    {
       vi->mAcurrent = 0.0f;
       vi->mVoltsBus = 0.0f;
       vi->mVoltsShunt = 0.0f;
       vi->num_of_sums = 0lu;
       vi->avg_mAcurrent = 0.0f;
       vi->avg_mVoltsLoad = 0.0f;
       vi->avg_mVoltsBus = 0.0f;
       vi->avg_mVoltsShunt = 0.0f;
       vi->mAcurrent_sum = 0.0f;
       vi->mVoltsBus_sum = 0.0f;
       vi->mVoltsShunt_sum = 0.0f;
       if ((vi->busdev.bustype == BUSTYPE_I2C) && (vi->busdev.busdev_u.i2c.address == 0))
       {
           vi->busdev.busdev_u.i2c.address = INA3221Address;
       }
       busWriteRegister(&vi->busdev, INA3221RegConfig, (INA3221ConfigEnableChan1 | INA3221ConfigEnableChan2 | INA3221ConfigEnableChan3 | INA3221ConfigAvg1 | INA3221ConfigVBusCT2 | INA3221ConfigVShCT2 | INA3221ConfigMode2 | INA3221ConfigMode1 | INA3221ConfigMode0));   // Set power mode and sample times
    }
}
/*-----------------------------------------------------------------------------
 *      INA3221_GetBusVoltage():  acumulate a new sensor reading
 *
 *  Parameters: INA3221Channel_e channel, powerIV_IC_t *vi
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void INA3221_GetBusVoltage( INA3221Channel_e channel, powerIV_IC_t *vi )
{
    uint8_t buf[INA3221_DATA_FRAME_SIZE];
    uint32_t tempRead1;

    if (vi == NULL)
    { /* for misra */ }
    else
    {
       busReadRegisterBuffer(&vi->busdev,(INA3221RegBusVoltage1 + (((uint8_t)(channel)-1u)*2u)), buf, INA3221_DATA_FRAME_SIZE);   // read data from sensor 5 bytes
       memcpy((void*)&tempRead1,(void*)buf,INA3221_DATA_FRAME_SIZE);
       if (tempRead1 > 0x7FFFul)
       {
         tempRead1 -= 0x10000ul;
       }
       vi->mVoltsBus = ((float32_t)tempRead1) * 0.001f;

       if (vi->mVoltsBus>0.0f)
       {
           vi->mVoltsBus_sum += vi->mVoltsBus;
           vi->num_of_sums=++vi->num_of_sums % UINT32_MAX;
           if (vi->num_of_sums>INA3221_SAMPLED_ENOUGH)
           {
              vi->avg_mVoltsBus= vi->mVoltsBus_sum / vi->num_of_sums;
              vi->num_of_sums = 0u;
           }
       }
    }
}
/*-----------------------------------------------------------------------------
 *      INA3221_GetShuntVoltCurrent():  acumulate a new sensor reading
 *
 *  Parameters: INA3221Channel_e channel, powerIV_IC_t *vi
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void INA3221_GetShuntVoltCurrent( INA3221Channel_e channel, powerIV_IC_t *vi )
{
    uint8_t buf[INA3221_DATA_FRAME_SIZE];
    uint32_t tempRead1;

    if (vi == NULL)
    { /* for misra */ }
    else
    {

       busReadRegisterBuffer(&vi->busdev, (INA3221RegShuntVoltage1 + (((uint8_t)(channel)-1u)*2u)), buf, INA3221_DATA_FRAME_SIZE);   // read data from sensor 5 bytes

       memcpy((void*)&tempRead1,(void*)buf,INA3221_DATA_FRAME_SIZE);
       if (tempRead1 > 0x7FFFul)
       {
           tempRead1 -= 0x10000ul;
       }
       vi->mVoltsShunt = ((float32_t)tempRead1) * 0.005f;
       vi->mAcurrent = vi->mVoltsShunt / INA3221ShuntResistorValue;

       if (vi->mVoltsShunt>0.0f)
       {
           vi->mVoltsShunt_sum += vi->mVoltsShunt;
           vi->mAcurrent_sum += vi->mAcurrent;
           vi->num_of_sums1=++vi->num_of_sums1 % UINT32_MAX;
           if (vi->num_of_sums1>INA3221_SAMPLED_ENOUGH)
           {
              vi->avg_mVoltsShunt= vi->mVoltsShunt_sum / vi->num_of_sums1;
              vi->avg_mAcurrent= vi->mAcurrent_sum / vi->num_of_sums1;
              if (vi->peak_current < vi->avg_mAcurrent)
              {
                 vi->peak_current = vi->avg_mAcurrent;
              }
              vi->num_of_sums1 = 0u;
           }
       }
    }
}
/*-----------------------------------------------------------------------------
 *      INA3221_GetLoadVoltage():  calculate load volatge reading
 *
 *  Parameters: powerIV_IC_t *vi
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void INA3221_GetLoadVoltage( powerIV_IC_t *vi )
{
        vi->avg_mVoltsLoad = vi->avg_mVoltsBus + (vi->avg_mVoltsShunt / 1000.0f);
}
#endif /* INA3221 power IC */

#if defined(PAW_OPTICAL_FLOW_USED)
/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *   Copyright (c) 2019 Tlera Corporation Created by Kris Winer
 *   Ported to mikroE C Pic32/FT900 by ACP Aviation
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#define PAW3902_SPI_BUS_SPEED (2000000L)                                        // 2MHz
/*-----------------------------------------------------------------------------
 *      PAW3902_modeBright():  Mode 0: Bright (126 fps) 60 Lux typical
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_modeBright( optical_flow_PAW3902JF_t *dev )
{

   uint8_t fRet = false;
      
   switch(dev->state)                                                           // set performance optimization registers
   {

        case 0u:
        if (dev->modeChangeRequest=true) dev->state = 1u;
        break;
        
        case 1u:
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x55u, 0x01u);
        busWriteRegister(&dev->busdev, 0x50u, 0x07u);
        busWriteRegister(&dev->busdev, 0x7fu, 0x0eu);
        busWriteRegister(&dev->busdev, 0x43u, 0x10u);
        
        busWriteRegister(&dev->busdev, 0x48u, 0x02u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x51u, 0x7bu);
        busWriteRegister(&dev->busdev, 0x50u, 0x00u);
        busWriteRegister(&dev->busdev, 0x55u, 0x00u);
        
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x61u, 0xADu);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x03u);
        busWriteRegister(&dev->busdev, 0x40u, 0x00u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x05u);
        busWriteRegister(&dev->busdev, 0x41u, 0xB3u);
        busWriteRegister(&dev->busdev, 0x43u, 0xF1u);
        busWriteRegister(&dev->busdev, 0x45u, 0x14u);
        busWriteRegister(&dev->busdev, 0x5Fu, 0x34u);
        busWriteRegister(&dev->busdev, 0x7Bu, 0x08u);
        busWriteRegister(&dev->busdev, 0x5eu, 0x34u);

        busWriteRegister(&dev->busdev, 0x5bu, 0x32u);
        busWriteRegister(&dev->busdev, 0x6du, 0x32u);
        busWriteRegister(&dev->busdev, 0x45u, 0x17u);
        busWriteRegister(&dev->busdev, 0x70u, 0xe5u);
        busWriteRegister(&dev->busdev, 0x71u, 0xe5u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x06u);
        busWriteRegister(&dev->busdev, 0x44u, 0x1Bu);
        busWriteRegister(&dev->busdev, 0x40u, 0xBFu);
        busWriteRegister(&dev->busdev, 0x4Eu, 0x3Fu);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x08u);
        busWriteRegister(&dev->busdev, 0x66u, 0x44u);
        busWriteRegister(&dev->busdev, 0x65u, 0x20u);
        busWriteRegister(&dev->busdev, 0x6au, 0x3au);
        busWriteRegister(&dev->busdev, 0x61u, 0x05u);
        busWriteRegister(&dev->busdev, 0x62u, 0x05u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x09u);
        busWriteRegister(&dev->busdev, 0x4Fu, 0xAFu);
        busWriteRegister(&dev->busdev, 0x48u, 0x80u);
        
        busWriteRegister(&dev->busdev, 0x49u, 0x80u);
        busWriteRegister(&dev->busdev, 0x57u, 0x77u);
        busWriteRegister(&dev->busdev, 0x5Fu, 0x40u);
        busWriteRegister(&dev->busdev, 0x60u, 0x78u);
        busWriteRegister(&dev->busdev, 0x61u, 0x78u);
        busWriteRegister(&dev->busdev, 0x62u, 0x08u);
        busWriteRegister(&dev->busdev, 0x63u, 0x50u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x0Au);
        busWriteRegister(&dev->busdev, 0x45u, 0x60u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x4Du, 0x11u);
        busWriteRegister(&dev->busdev, 0x55u, 0x80u);
        busWriteRegister(&dev->busdev, 0x74u, 0x21u);
        busWriteRegister(&dev->busdev, 0x75u, 0x1Fu);
        busWriteRegister(&dev->busdev, 0x4Au, 0x78u);
        
        busWriteRegister(&dev->busdev, 0x4Bu, 0x78u);
        busWriteRegister(&dev->busdev, 0x44u, 0x08u);
        busWriteRegister(&dev->busdev, 0x45u, 0x50u);
        busWriteRegister(&dev->busdev, 0x64u, 0xFEu);
        busWriteRegister(&dev->busdev, 0x65u, 0x1Fu);
        busWriteRegister(&dev->busdev, 0x72u, 0x0Au);
        busWriteRegister(&dev->busdev, 0x73u, 0x00u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x14u);
        busWriteRegister(&dev->busdev, 0x44u, 0x84u);
        busWriteRegister(&dev->busdev, 0x65u, 0x47u);
        busWriteRegister(&dev->busdev, 0x66u, 0x18u);
        busWriteRegister(&dev->busdev, 0x63u, 0x70u);
        busWriteRegister(&dev->busdev, 0x6fu, 0x2cu);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x15u);
        busWriteRegister(&dev->busdev, 0x48u, 0x48u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x07u);
        busWriteRegister(&dev->busdev, 0x41u, 0x0Du);
        busWriteRegister(&dev->busdev, 0x43u, 0x14u);
        busWriteRegister(&dev->busdev, 0x4Bu, 0x0Eu);
        busWriteRegister(&dev->busdev, 0x45u, 0x0Fu);
        busWriteRegister(&dev->busdev, 0x44u, 0x42u);
        busWriteRegister(&dev->busdev, 0x4Cu, 0x80u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x10u);
        busWriteRegister(&dev->busdev, 0x5Bu, 0x03u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x07u);
        busWriteRegister(&dev->busdev, 0x40u, 0x41u);
        dev->ticksRef = CP0_GET(CP0_COUNT);
        dev->state = 2u;
        break;
        
        case 2u:                                                                //usleep(10_ms);  delay 10ms
        calculateTick2Now(&dev->ticksVal,&dev->ticksRef);
        if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))  dev->state = 3u;
        break;

        case 3u:
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x32u, 0x00u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x07u);
        busWriteRegister(&dev->busdev, 0x40u, 0x40u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x06u);
        busWriteRegister(&dev->busdev, 0x68u, 0x70u);
        busWriteRegister(&dev->busdev, 0x69u, 0x01u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x0Du);
        busWriteRegister(&dev->busdev, 0x48u, 0xC0u);
        busWriteRegister(&dev->busdev, 0x6Fu, 0xD5u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x5Bu, 0xA0u);
        busWriteRegister(&dev->busdev, 0x4Eu, 0xA8u);
        busWriteRegister(&dev->busdev, 0x5Au, 0x50u);
        busWriteRegister(&dev->busdev, 0x40u, 0x80u);
        busWriteRegister(&dev->busdev, 0x73u, 0x1fu);
        dev->ticksRef = CP0_GET(CP0_COUNT);
        dev->state = 4u;
        break;

        case 4u:                                                                //usleep(10_ms);  delay 10ms
        calculateTick2Now(&dev->ticksVal,&dev->ticksRef);
        if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))  dev->state = 5u;
        break;        

        case 5u:
        busWriteRegister(&dev->busdev, 0x73u, 0x00u);
        fRet = true;
        dev->state = 0u;
        break;
        
        default:
        dev->state = 0u;
        break;
   }
   return fRet;
}
 
/*-----------------------------------------------------------------------------
 *      PAW3902_modeLowLight():  Mode 1: Low Light (126 fps) 30 Lux typical
 *                               low light and low speed motion tracking
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_modeLowLight( optical_flow_PAW3902JF_t *dev )
{

   uint8_t fRet = false;
      
   switch(dev->state)                                                           // set performance optimization registers
   {
        case 0u:
        if (dev->modeChangeRequest=true) dev->state = 1u;
        break;
        
        case 1u:
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x55u, 0x01u);
        busWriteRegister(&dev->busdev, 0x50u, 0x07u);
        busWriteRegister(&dev->busdev, 0x7fu, 0x0eu);
        busWriteRegister(&dev->busdev, 0x43u, 0x10u);
        busWriteRegister(&dev->busdev, 0x48u, 0x02u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x51u, 0x7bu);
        busWriteRegister(&dev->busdev, 0x50u, 0x00u);
        busWriteRegister(&dev->busdev, 0x55u, 0x00u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x61u, 0xADu);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x03u);
        busWriteRegister(&dev->busdev, 0x40u, 0x00u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x05u);
        busWriteRegister(&dev->busdev, 0x41u, 0xB3u);
        busWriteRegister(&dev->busdev, 0x43u, 0xF1u);
        busWriteRegister(&dev->busdev, 0x45u, 0x14u);
        busWriteRegister(&dev->busdev, 0x5Fu, 0x34u);
        busWriteRegister(&dev->busdev, 0x7Bu, 0x08u);
        busWriteRegister(&dev->busdev, 0x5eu, 0x34u);

        busWriteRegister(&dev->busdev, 0x5bu, 0x65u);
        busWriteRegister(&dev->busdev, 0x6du, 0x65u);
        busWriteRegister(&dev->busdev, 0x45u, 0x17u);
        busWriteRegister(&dev->busdev, 0x70u, 0xe5u);
        busWriteRegister(&dev->busdev, 0x71u, 0xe5u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x06u);
        busWriteRegister(&dev->busdev, 0x44u, 0x1Bu);
        busWriteRegister(&dev->busdev, 0x40u, 0xBFu);
        busWriteRegister(&dev->busdev, 0x4Eu, 0x3Fu);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x08u);
        busWriteRegister(&dev->busdev, 0x66u, 0x44u);
        busWriteRegister(&dev->busdev, 0x65u, 0x20u);
        busWriteRegister(&dev->busdev, 0x6au, 0x3au);
        busWriteRegister(&dev->busdev, 0x61u, 0x05u);
        busWriteRegister(&dev->busdev, 0x62u, 0x05u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x09u);
        busWriteRegister(&dev->busdev, 0x4Fu, 0xAFu);
        busWriteRegister(&dev->busdev, 0x48u, 0x80u);
        busWriteRegister(&dev->busdev, 0x49u, 0x80u);
        busWriteRegister(&dev->busdev, 0x57u, 0x77u);
        busWriteRegister(&dev->busdev, 0x5Fu, 0x40u);
        busWriteRegister(&dev->busdev, 0x60u, 0x78u);
        busWriteRegister(&dev->busdev, 0x61u, 0x78u);
        busWriteRegister(&dev->busdev, 0x62u, 0x08u);
        busWriteRegister(&dev->busdev, 0x63u, 0x50u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x0Au);
        busWriteRegister(&dev->busdev, 0x45u, 0x60u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x4Du, 0x11u);
        busWriteRegister(&dev->busdev, 0x55u, 0x80u);
        busWriteRegister(&dev->busdev, 0x74u, 0x21u);
        busWriteRegister(&dev->busdev, 0x75u, 0x1Fu);
        busWriteRegister(&dev->busdev, 0x4Au, 0x78u);
        busWriteRegister(&dev->busdev, 0x4Bu, 0x78u);
        busWriteRegister(&dev->busdev, 0x44u, 0x08u);
        busWriteRegister(&dev->busdev, 0x45u, 0x50u);
        busWriteRegister(&dev->busdev, 0x64u, 0xFEu);
        busWriteRegister(&dev->busdev, 0x65u, 0x1Fu);
        busWriteRegister(&dev->busdev, 0x72u, 0x0Au);
        busWriteRegister(&dev->busdev, 0x73u, 0x00u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x14u);
        busWriteRegister(&dev->busdev, 0x44u, 0x84u);
        busWriteRegister(&dev->busdev, 0x65u, 0x47u);
        busWriteRegister(&dev->busdev, 0x66u, 0x18u);
        busWriteRegister(&dev->busdev, 0x63u, 0x70u);
        busWriteRegister(&dev->busdev, 0x6fu, 0x2cu);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x15u);
        busWriteRegister(&dev->busdev, 0x48u, 0x48u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x07u);
        busWriteRegister(&dev->busdev, 0x41u, 0x0Du);
        busWriteRegister(&dev->busdev, 0x43u, 0x14u);
        busWriteRegister(&dev->busdev, 0x4Bu, 0x0Eu);
        busWriteRegister(&dev->busdev, 0x45u, 0x0Fu);
        busWriteRegister(&dev->busdev, 0x44u, 0x42u);
        busWriteRegister(&dev->busdev, 0x4Cu, 0x80u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x10u);
        busWriteRegister(&dev->busdev, 0x5Bu, 0x03u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x07u);
        busWriteRegister(&dev->busdev, 0x40u, 0x41u);
        dev->ticksRef = CP0_GET(CP0_COUNT);
        dev->state = 2u;
        break;
        
        case 2u:                                                                //usleep(10_ms);  delay 10ms
        calculateTick2Now(&dev->ticksVal,&dev->ticksRef);
        if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))  dev->state = 3u;
        break;

        case 3u:
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x32u, 0x00u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x07u);
        busWriteRegister(&dev->busdev, 0x40u, 0x40u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x06u);
        busWriteRegister(&dev->busdev, 0x68u, 0x70u);
        busWriteRegister(&dev->busdev, 0x69u, 0x01u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x0Du);
        busWriteRegister(&dev->busdev, 0x48u, 0xC0u);
        busWriteRegister(&dev->busdev, 0x6Fu, 0xD5u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x5Bu, 0xA0u);
        busWriteRegister(&dev->busdev, 0x4Eu, 0xA8u);
        busWriteRegister(&dev->busdev, 0x5Au, 0x50u);
        busWriteRegister(&dev->busdev, 0x40u, 0x80u);
        busWriteRegister(&dev->busdev, 0x73u, 0x1fu);
        dev->ticksRef = CP0_GET(CP0_COUNT);
        dev->state = 4u;
        break;

        case 4u:                                                                //usleep(10_ms);  delay 10ms
        calculateTick2Now(&dev->ticksVal,&dev->ticksRef);
        if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))  dev->state = 5u;
        break;        

        case 5u:
        busWriteRegister(&dev->busdev, 0x73u, 0x00u);
        fRet = true;
        dev->state = 0u;
        break;
        
        default:
        dev->state = 0u;
        break;
   }
   return fRet;
}
 
/*-----------------------------------------------------------------------------
 *      PAW3902_modeSuperLowLight():  Mode 2: Super Low Light (50 fps) 9 Lux typical
 *                                    super low light and low speed motion tracking
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_modeSuperLowLight( optical_flow_PAW3902JF_t *dev )
{

   uint8_t fRet = false;
      
   switch(dev->state)                                                           // set performance optimization registers
   {
        case 0u:
        if (dev->modeChangeRequest=true) dev->state = 1u;
        break;
        
        case 1u:
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x55u, 0x01u);
        busWriteRegister(&dev->busdev, 0x50u, 0x07u);
        busWriteRegister(&dev->busdev, 0x7fu, 0x0eu);
        busWriteRegister(&dev->busdev, 0x43u, 0x10u);
        
        busWriteRegister(&dev->busdev, 0x48u, 0x04u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x51u, 0x7bu);
        busWriteRegister(&dev->busdev, 0x50u, 0x00u);
        busWriteRegister(&dev->busdev, 0x55u, 0x00u);
        
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x61u, 0xADu);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x03u);
        busWriteRegister(&dev->busdev, 0x40u, 0x00u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x05u);
        busWriteRegister(&dev->busdev, 0x41u, 0xB3u);
        busWriteRegister(&dev->busdev, 0x43u, 0xF1u);
        busWriteRegister(&dev->busdev, 0x45u, 0x14u);
        busWriteRegister(&dev->busdev, 0x5Fu, 0x34u);
        busWriteRegister(&dev->busdev, 0x7Bu, 0x08u);
        busWriteRegister(&dev->busdev, 0x5eu, 0x34u);

        busWriteRegister(&dev->busdev, 0x5bu, 0x32u);
        busWriteRegister(&dev->busdev, 0x6du, 0x32u);
        busWriteRegister(&dev->busdev, 0x45u, 0x17u);
        busWriteRegister(&dev->busdev, 0x70u, 0xe5u);
        busWriteRegister(&dev->busdev, 0x71u, 0xe5u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x06u);
        busWriteRegister(&dev->busdev, 0x44u, 0x1Bu);
        busWriteRegister(&dev->busdev, 0x40u, 0xBFu);
        busWriteRegister(&dev->busdev, 0x4Eu, 0x3Fu);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x08u);
        busWriteRegister(&dev->busdev, 0x66u, 0x44u);
        busWriteRegister(&dev->busdev, 0x65u, 0x20u);
        busWriteRegister(&dev->busdev, 0x6au, 0x3au);
        busWriteRegister(&dev->busdev, 0x61u, 0x05u);
        
        busWriteRegister(&dev->busdev, 0x62u, 0x05u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x09u);
        busWriteRegister(&dev->busdev, 0x4Fu, 0xAFu);
        busWriteRegister(&dev->busdev, 0x48u, 0x80u);
        busWriteRegister(&dev->busdev, 0x49u, 0x80u);
        busWriteRegister(&dev->busdev, 0x57u, 0x77u);
        busWriteRegister(&dev->busdev, 0x5Fu, 0x40u);
        busWriteRegister(&dev->busdev, 0x60u, 0x78u);
        busWriteRegister(&dev->busdev, 0x61u, 0x78u);
        busWriteRegister(&dev->busdev, 0x62u, 0x08u);
        busWriteRegister(&dev->busdev, 0x63u, 0x50u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x0Au);
        
        busWriteRegister(&dev->busdev, 0x45u, 0x60u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x4Du, 0x11u);
        busWriteRegister(&dev->busdev, 0x55u, 0x80u);
        busWriteRegister(&dev->busdev, 0x74u, 0x21u);
        busWriteRegister(&dev->busdev, 0x75u, 0x1Fu);
        busWriteRegister(&dev->busdev, 0x4Au, 0x78u);
        busWriteRegister(&dev->busdev, 0x4Bu, 0x78u);
        busWriteRegister(&dev->busdev, 0x44u, 0x08u);
        busWriteRegister(&dev->busdev, 0x45u, 0x50u);
        
        busWriteRegister(&dev->busdev, 0x64u, 0xCEu);
        busWriteRegister(&dev->busdev, 0x65u, 0x0Bu);
        busWriteRegister(&dev->busdev, 0x72u, 0x0Au);
        busWriteRegister(&dev->busdev, 0x73u, 0x00u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x14u);
        busWriteRegister(&dev->busdev, 0x44u, 0x84u);
        
        busWriteRegister(&dev->busdev, 0x65u, 0x67u);
        busWriteRegister(&dev->busdev, 0x66u, 0x18u);
        busWriteRegister(&dev->busdev, 0x63u, 0x70u);
        busWriteRegister(&dev->busdev, 0x6fu, 0x2cu);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x15u);
        busWriteRegister(&dev->busdev, 0x48u, 0x48u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x07u);
        busWriteRegister(&dev->busdev, 0x41u, 0x0Du);
        busWriteRegister(&dev->busdev, 0x43u, 0x14u);
        busWriteRegister(&dev->busdev, 0x4Bu, 0x0Eu);
        
        busWriteRegister(&dev->busdev, 0x45u, 0x0Fu);
        busWriteRegister(&dev->busdev, 0x44u, 0x42u);
        busWriteRegister(&dev->busdev, 0x4Cu, 0x80u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x10u);
        busWriteRegister(&dev->busdev, 0x5Bu, 0x02u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x07u);
        busWriteRegister(&dev->busdev, 0x40u, 0x41u);
        dev->ticksRef = CP0_GET(CP0_COUNT);
        dev->state = 2u;
        break;
        
        case 2u:                                                                //usleep(10_ms);  delay 10ms
        calculateTick2Now(&dev->ticksVal,&dev->ticksRef);
        if (dev->ticksVal > (0.25f*CPU_TICKS_PER_SECOND))  dev->state = 3u;
        break;

        case 3u:
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x32u, 0x44u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x07u);
        busWriteRegister(&dev->busdev, 0x40u, 0x40u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x06u);
        busWriteRegister(&dev->busdev, 0x68u, 0x40u);
        busWriteRegister(&dev->busdev, 0x69u, 0x02u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x0Du);
        busWriteRegister(&dev->busdev, 0x48u, 0xC0u);
        busWriteRegister(&dev->busdev, 0x6Fu, 0xD5u);
        busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
        busWriteRegister(&dev->busdev, 0x5Bu, 0xA0u);
        busWriteRegister(&dev->busdev, 0x4Eu, 0xA8u);
        busWriteRegister(&dev->busdev, 0x5Au, 0x50u);
        busWriteRegister(&dev->busdev, 0x40u, 0x80u);
        busWriteRegister(&dev->busdev, 0x73u, 0x0Bu);
        dev->ticksRef = CP0_GET(CP0_COUNT);
        dev->state = 4u;
        break;

        case 4u:                                                                //usleep(10_ms);  delay 10ms
        calculateTick2Now(&dev->ticksVal,&dev->ticksRef);
        if (dev->ticksVal > (0.25f*CPU_TICKS_PER_SECOND))  dev->state = 5u;
        break;        

        case 5u:
        busWriteRegister(&dev->busdev, 0x73u, 0x00u);
        fRet = true;
        dev->state = 0u;
        break;
        
        default:
        dev->state = 0u;
        break;
   }
   return fRet;
}
 
/*-----------------------------------------------------------------------------
 *      PAW3902_reset():  Power on reset
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_reset( optical_flow_PAW3902JF_t *dev )
{
   uint8_t retVal=false; 

   switch(dev->state)                                                           // set performance optimization registers
   {
        case 0u:
        if (dev->resetRequest == PAW_ResetReq) dev->state = 6u;
        break;
        
        case 6u:
        busWriteRegister(&dev->busdev, PAW_Power_Up_Reset, 0x5Au);
        dev->ticksRef = CP0_GET(CP0_COUNT);
        dev->state = 7u;
        break;
        
        case 7u:                                                                // usleep(5000);
        calculateTick2Now(&dev->ticksVal,&dev->ticksRef);
        if (dev->ticksVal > (5.0f*CPU_TICKS_PER_SECOND))  dev->state = 8u;
        break;         

        case 8u:                                                                // Read from registers 0x02, 0x03, 0x04, 0x05 and 0x06 one time regardless of the motion state
        busReadRegisterBuffer(&dev->busdev, PAW_Motion, &dev->Motion, 1u);
        busReadRegisterBuffer(&dev->busdev, PAW_Delta_X_L, &dev->Delta_X_L, 1u);
        busReadRegisterBuffer(&dev->busdev, PAW_Delta_X_H, &dev->Delta_X_H, 1u);
        busReadRegisterBuffer(&dev->busdev, PAW_Delta_Y_L, &dev->Delta_Y_L, 1u);
        busReadRegisterBuffer(&dev->busdev, PAW_Delta_Y_H, &dev->Delta_Y_H, 1u);
        dev->ticksRef = CP0_GET(CP0_COUNT);
        dev->state = 9u;        
        break;
        
        case 9u:                                                                // usleep(1000);
        calculateTick2Now(&dev->ticksVal,&dev->ticksRef);
        if (dev->ticksVal > (1.0f*CPU_TICKS_PER_SECOND))
        {
          retVal = true;
          dev->state = 0u;
        }  
        break;         

   }
   return retVal;
}
/*-----------------------------------------------------------------------------
 *      PAW3902_changeMode():  change the light mode
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev, uint8_t newMode
 *  Return:     uint8_t
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_changeMode( optical_flow_PAW3902JF_t *dev, uint8_t newMode )
{
        uint8_t regVal;
        uint8_t returnVal=false;
        //uint8_t resetDone;
        
        if (newMode != dev->mode)                                               /* change of mode occurred */
        {
           dev->resetRequest = PAW_ResetReq;
           dev->mode = newMode;
        }

        //ScheduleClear();
        switch ( dev->resetRequest )
        {
           case PAW_ResetReq:
           if ( PAW3902_reset( dev ) == true ) dev->resetRequest = PAW_ResetInit;
           break;
                    
           case PAW_ResetInit:
           switch (dev->mode) 
           {
                case PAW3902_Mode_Bright:
                if (PAW3902_modeBright( dev ) == true)
                {
                   if (busReadRegisterBuffer(&dev->busdev, PAW_Resolution, &regVal, 1u) == true)
                   {
                      dev->resolution = (( ((float64_t)(regVal & 0xA8u)) + 1.0f) * (50.0f / 8450.0f));
                      returnVal = true;
                      dev->resetRequest = PAW_ResetDone;
                   }
                }
                dev->schedOnInterval = PAW_SAMPLE_INTERVAL_MODE_0;
                break;

                case PAW3902_Mode_LowLight:
                if (PAW3902_modeLowLight( dev ) == true)
                {
                   if (busReadRegisterBuffer(&dev->busdev, PAW_Resolution, &regVal, 1u) == true)
                   {
                      dev->resolution = (( ((float64_t)(regVal & 0xA8u)) + 1.0f) * (50.0f / 8450.0f));
                      returnVal = true;
                      dev->resetRequest = PAW_ResetDone;
                   }
                }
                dev->schedOnInterval = PAW_SAMPLE_INTERVAL_MODE_1;
                break;

                case PAW3902_Mode_SuperLowLight:
                if (PAW3902_modeSuperLowLight( dev ) == true)
                {
                   if (busReadRegisterBuffer(&dev->busdev, PAW_Resolution, &regVal, 1u) == true)
                   {
                      dev->resolution = (( ((float64_t)(regVal & 0xA8u)) + 1.0f) * (50.0f / 8450.0f));
                      returnVal = true;
                      dev->resetRequest = PAW_ResetDone;
                   }
                }
                dev->schedOnInterval = PAW_SAMPLE_INTERVAL_MODE_2;
                break;
                        
                default:
                break;
            }
            break;

            case PAW_ResetDone:
            // Approximate Resolution = (Register Value + 1) * (50 / 8450) ˜ 0.6% of data point in Figure 19
            // The maximum register value is 0xA8. The minimum register value is 0.
            if (busReadRegisterBuffer(&dev->busdev, PAW_Resolution, &regVal, 1u) == true)
            {
                 dev->resolution = (( ((float64_t)(regVal & 0xA8u)) + 1.0f) * (50.0f / 8450.0f));
                 returnVal = true;
            }
            break;
                                          
            default:
            break;
        }

        return returnVal;                                                       /* this iterative function returns true when it completes all actions */ 
}
/*-----------------------------------------------------------------------------
 *      PAW3902_getUpdate():  get update of the values
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev
 *  Return:     uint8_t : true when successful
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_getUpdate( optical_flow_PAW3902JF_t *dev )
{
    uint8_t retVal = false;
    
    if ((dev->resetRequest == PAW_ResetDone) && (dev->state == 0u)) dev->state = 9u;
    if (dev->state == 9u)
    {        
        if (busReadRegisterBuffer(&dev->busdev, PAW_Motion, &dev->Motion, 1u) == true)
            if (busReadRegisterBuffer(&dev->busdev, PAW_Observation, &dev->Observation, 1u) == true)
                if (busReadRegisterBuffer(&dev->busdev, PAW_Delta_X_L, &dev->Delta_X_L, 1u) == true)
                   if (busReadRegisterBuffer(&dev->busdev, PAW_Delta_X_H, &dev->Delta_X_H, 1u) == true)        
                      if (busReadRegisterBuffer(&dev->busdev, PAW_Delta_Y_L, &dev->Delta_Y_L, 1u) == true)
                         if (busReadRegisterBuffer(&dev->busdev, PAW_Delta_Y_H, &dev->Delta_Y_H, 1u) == true)
                            if (busReadRegisterBuffer(&dev->busdev, PAW_Squal, &dev->SQUAL, 1u) == true)
                               if (busReadRegisterBuffer(&dev->busdev, PAW_RawData_Sum, &dev->RawData_Sum, 1u) == true)
                                 if (busReadRegisterBuffer(&dev->busdev, PAW_Maximum_RawData, &dev->Maximum_RawData, 1u) == true)                                                                           
                                   if (busReadRegisterBuffer(&dev->busdev, PAW_Minimum_RawData, &dev->Minimum_RawData, 1u) == true)
                                      if (busReadRegisterBuffer(&dev->busdev, PAW_Shutter_Upper, &dev->Shutter_Upper, 1u) == true)
                                          if (busReadRegisterBuffer(&dev->busdev, PAW_Shutter_Lower, &dev->Shutter_Lower, 1u) == true)
                                          {
                                             dev->state = 0u; 
                                             dev->iterations = ++dev->iterations % UINT8_MAX;;                                                                                       
                                             retVal = true;
                                          }  
    }
    return retVal;
}
/*-----------------------------------------------------------------------------
 *      PAW3902_enterFrameCaptureMode():  enter frame capture mode
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev
 *  Return:     uint8_t true once iteration has completed its 
 *                      sequence of operations and timers
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_enterFrameCaptureMode( optical_flow_PAW3902JF_t *dev )
{
  switch (dev->retVal)
  {
      case 0u:
      if (dev->mode == PAW3902_Mode_SuperLowLight)                                  /* make sure not in superlowlight mode for frame capture */
      {
         dev->retVal = 100u;
         dev->modeBeforeCapture = PAW3902_Mode_SuperLowLight;
      }
      else
      {
         dev->modeBeforeCapture = dev->mode;
         dev->retVal = 1u;
      }
      break;
      
      case 100u:
      dev->retVal = PAW3902_changeMode( dev, PAW3902_Mode_LowLight );
      break;
      
      case 1u:
      dev->mode = PAW3902_Mode_FrameCapture;                                     /* disable any light mode changeover while in capture mode */          
      busWriteRegister(&dev->busdev, 0x7Fu, 0x07u);
      busWriteRegister(&dev->busdev, 0x41u, 0x1Du);
      busWriteRegister(&dev->busdev, 0x4Cu, 0x00u);  
      busWriteRegister(&dev->busdev, 0x7Fu, 0x08u);  
      busWriteRegister(&dev->busdev, 0x6Au, 0x38u);  
      busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);    
      busWriteRegister(&dev->busdev, 0x55u, 0x04u);    
      busWriteRegister(&dev->busdev, 0x40u, 0x80u); 
      busWriteRegister(&dev->busdev, 0x4Du, 0x11u); 
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = 2u;
      break;
      
      case 2u:
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
        dev->retVal = true;                                                     /* return true for completion and set the state to read the values */
      }
      break;
      
      default:
      break;      
  }        
  return dev->retVal;
}

/*-----------------------------------------------------------------------------
 *      PAW3902_enterFrameCaptureModeWithDelay():  enter frame capture mode
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev
 *  Return:     uint8_t true once iteration has completed its 
 *                      sequence of operations and timers
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_enterFrameCaptureModeWithDelay( optical_flow_PAW3902JF_t *dev )
{

  switch (dev->retVal)
  {
      case 0u:                                                                  /* START sequence */
      if (dev->mode == PAW3902_Mode_SuperLowLight)                              /* make sure not in superlowlight mode for frame capture */
      {
         dev->modeBeforeCapture = PAW3902_Mode_SuperLowLight;
         dev->retVal = 100u;
      }
      else
      {
         dev->modeBeforeCapture = dev->mode;                                    /* store last known mode */
         dev->retVal = 1u;
      }
      break;

      case 100u:                                                                /* change light intensity and wait until its completed before going to step 1 */
      dev->retVal = PAW3902_changeMode( dev, PAW3902_Mode_LowLight );
                  
      case 1u:                                                                  /* mode is okay now STARt the capture mode initialise sequence */ 
      dev->mode = PAW3902_Mode_FrameCapture;                                    /* disable any light mode changeover while in capture mode */
      busWriteRegister(&dev->busdev, 0x7Fu, 0x07u);
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;
      
      case 2u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;

      case 3u:
      busWriteRegister(&dev->busdev, 0x41u, 0x1Du);
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;      

      case 4u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;

      case 5u:
      busWriteRegister(&dev->busdev, 0x4Cu, 0x00u); 
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;      

      case 6u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;
      
      case 7u:
      busWriteRegister(&dev->busdev, 0x7Fu, 0x08u);  
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break; 
      
      case 8u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;

      case 9u:
      busWriteRegister(&dev->busdev, 0x6Au, 0x38u);  
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;      

      case 10u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break; 
      
      case 11u:
      busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);     
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break; 
      
      case 12u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;
      
      case 13u:
      busWriteRegister(&dev->busdev, 0x55u, 0x04u);        
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;  

      case 14u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;
      
      case 15u:
      busWriteRegister(&dev->busdev, 0x40u, 0x80u);       
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break; 
      
      case 16u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break; 
      
      case 17u:
      busWriteRegister(&dev->busdev, 0x4Du, 0x11u);      
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;
      
      case 18u:
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
        dev->retVal = true;                                                     /* signal completion to the caller the iteration has totally completed */
      }
      break;
            
      default:
      break;
                                                  
  }        
  return dev->retVal;
}
/*-----------------------------------------------------------------------------
 *      PAW3902_captureFrame():  capture a frame 
 *
 *  Parameters: uint8_t * frameArray, optical_flow_PAW3902JF_t *dev
 *  Return:     uint8_t true once iteration has completed its 
 *                      sequence of operations and timers
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_captureFrame(uint8_t * frameArray, optical_flow_PAW3902JF_t *dev)
{
  uint8_t rawDataUpper = 0u, rawDataLower = 0u, ii, jj;

  switch (dev->retVal)
  {
      case 1u:                                                                  /* mode is ready to capture */
      busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;
      
      case 2u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break; 

      case 3u:
      busWriteRegister(&dev->busdev, 0x58u, 0xFFu);
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;

      case 4u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;
      
      case 5u:              
      for(ii = 0u; ii < 35u; ii++)
      {
         for(jj = 0u; jj < 35u; jj++)
         {
            if (busReadRegisterBuffer(&dev->busdev, 0x58u, &rawDataUpper, 1u) == true)
            {
               while( (rawDataUpper & 0xC0u) != 0x40u ) { if (busReadRegisterBuffer(&dev->busdev, 0x58u, &rawDataUpper, 1u) == true){} } // wait for upper six bits of raw data to be valid
               if (busReadRegisterBuffer(&dev->busdev, 0x58u, &rawDataLower, 1u) == true)
               {
                   while( (rawDataLower & 0xC0u) != 0x80u ) { if (busReadRegisterBuffer(&dev->busdev, 0x58u, &rawDataLower, 1u) == true){} } // wait for lower two bits of raw data to be valid
                   frameArray[ii*35u + jj] = (rawDataUpper & 0x3F) << 2u | (rawDataLower & 0x0C) >> 2u;
               }
               else
               {
                  dev->retVal = 3u;                                             /* re-start by writing last mode write again */
               }
            }
            else
            {
               dev->retVal = 3u;            
            }
         }
      }
      dev->retVal = 1u;                                                         /* set mode back to capture */
      break;
      
      default:
      break;
  }
  return dev->retVal;
}

/*-----------------------------------------------------------------------------
 *      PAW3902_exitFrameCaptureModeeWithDelay():  enter frame capture mode
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev
 *  Return:     uint8_t true once iteration has completed its 
 *                      sequence of operations and timers
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_exitFrameCaptureModeWithDelay( optical_flow_PAW3902JF_t *dev )
{

  dev->retVal = 1u;

  switch (dev->retVal)
  {
      case 1u:
      busWriteRegister(&dev->busdev, 0x7Fu, 0x00u);
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;
      
      case 2u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;

      case 3u:
      busWriteRegister(&dev->busdev, 0x4Du, 0x11u);
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;      

      case 4u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;

      case 5u:
      busWriteRegister(&dev->busdev, 0x40, 0x80); 
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;      

      case 6u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;
      
      case 7u:
      busWriteRegister(&dev->busdev, 0x55u, 0x80u);  
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break; 
      
      case 8u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;

      case 9u:
      busWriteRegister(&dev->busdev, 0x7Fu, 0x08u);  
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;      

      case 10u:                                                                 /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break; 
      
      case 11u:
      busWriteRegister(&dev->busdev, 0x6Au, 0x18u );     
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break; 
      
      case 12u:                                                                 /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;
      
      case 13u:
      busWriteRegister(&dev->busdev, 0x7Fu, 0x07u );        
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;  

      case 14u:                                                                 /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break;
      
      case 15u:
      busWriteRegister(&dev->busdev, 0x41u, 0x0Du );       
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break; 
      
      case 16u:                                                                 /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break; 
      
      case 17u:
      busWriteRegister(&dev->busdev, 0x4Cu, 0x80u );      
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;

      case 18u:                                                                  /* wait step */
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
         dev->retVal = ++dev->retVal % UINT8_MAX;
      }
      break; 

      case 19u:
      busWriteRegister(&dev->busdev, 0x7Fu, 0x00u );      
      dev->ticksRef = CP0_GET(CP0_COUNT);
      dev->retVal = ++dev->retVal % UINT8_MAX;
      break;
      
      case 20u:
      calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
      if (dev->ticksVal > (0.1f*CPU_TICKS_PER_SECOND))
      {
        dev->retVal = true;                                                     /* signal completion to the caller the iteration has totally completed */
        dev->mode = dev->modeBeforeCapture;
     }
      break;
      
      default:
      break;
                                                  
  }        
  return dev->retVal;
}
/*-----------------------------------------------------------------------------
 *      PAW3902_checkID():  check product id
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev
 *  Return:     uint8_t :: true success
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_checkID( optical_flow_PAW3902JF_t *dev )
{
  uint8_t rCode = 0u;
  if (dev != NULL)
  {
     if (busReadRegisterBuffer(&dev->busdev, PAW_Product_ID, &dev->product_ID, 1u) == true)
     {
        if (busReadRegisterBuffer(&dev->busdev, PAW_Revision_ID , &dev->revision_ID, 1u) == true) 
        {
           if (busReadRegisterBuffer(&dev->busdev, PAW_Inverse_Product , &dev->inverse_product_ID, 1u) == true)
           {          
              rCode = ((dev->revision_ID != PAW3902_REVISION_ID) || ((dev->product_ID != PAW3902_CHIP_PRODUCT_ID) || (dev->inverse_product_ID != PAW3902_INV_PROD_ID))) ? 0u : 1u;
           }
        }
     }
  }
  else { /* for misra */ }
  return rCode;
}
/*-----------------------------------------------------------------------------
 *      paw3902_spi_init():  initialze the spi bus number bus_num for PAW sensor
 *
 *  Parameters: const busDevice_t *busdev
 *  Return:     uint8_t :: true success
 *----------------------------------------------------------------------------*/
devIC_PR_TE void paw3902_spi_init( const busDevice_t *busdev )
{
   if ((busdev == NULL) || (busdev->bustype != BUSTYPE_SPI))
   { /* for misra */ }
   else
   {           
      switch ((busdev->busdev_u.spi.device))
      {  
         case SPIDEV_1:   
         Chip_Select1 = 1;                                                      // Deselect SPi preiferal
         Chip_Select_Direction1 = 0;                                            // Set CS# pin as Output
         SPI1_Init_Advanced(_SPI_MASTER, _SPI_8_BIT, (CPU_CLOCK_SPEED/PAW3902_SPI_BUS_SPEED), _SPI_SS_DISABLE, _SPI_DATA_SAMPLE_MIDDLE, _SPI_CLK_IDLE_HIGH, _SPI_ACTIVE_2_IDLE);                                                                 /* initialise SPI link 4 */
         SPI1CON = 0u;                                                          /* turn off the SPI module and reset it */
         SPI1BUF;                                                               /* clear the rx buffer by reading from it  */
         SPI1STATbits.SPIROV = 0;                                               /* clear the overflow bit  */
         SPI1CONbits.CKE = 1;                                                   /* data changes when clock goes from active to inactive (high to low since CKP is 0) */
         SPI1CONbits.MSTEN = 1;                                                 /* master operation  */
         SPI1CONbits.ON = 1;                                                    /* turn on SPI 1  */
         break;  
           
         case SPIDEV_2:   
         Chip_Select2 = 1;                                                      // Deselect SPi preiferal
         Chip_Select_Direction2 = 0;                                            // Set CS# pin as Output
         SPI2_Init_Advanced(_SPI_MASTER, _SPI_8_BIT, (CPU_CLOCK_SPEED/PAW3902_SPI_BUS_SPEED), _SPI_SS_DISABLE, _SPI_DATA_SAMPLE_MIDDLE, _SPI_CLK_IDLE_HIGH, _SPI_ACTIVE_2_IDLE);                                                                 /* initialise SPI link 4 */
         SPI2CON = 0u;                                                          /* turn off the SPI module and reset it */
         SPI2BUF;                                                               /* clear the rx buffer by reading from it  */
         SPI2STATbits.SPIROV = 0;                                               /* clear the overflow bit  */
         SPI2CONbits.CKE = 1;                                                   /* data changes when clock goes from active to inactive (high to low since CKP is 0) */
         SPI2CONbits.MSTEN = 1;                                                 /* master operation  */
         SPI2CONbits.ON = 1;                                                    /* turn on SPI 2  */
         break;  
           
         case SPIDEV_3:   
         Chip_Select3 = 1;                                                      // Deselect SPi preiferal
         Chip_Select_Direction3 = 0;                                            // Set CS# pin as Output
         SPI3_Init_Advanced(_SPI_MASTER, _SPI_8_BIT, (CPU_CLOCK_SPEED/PAW3902_SPI_BUS_SPEED), _SPI_SS_DISABLE, _SPI_DATA_SAMPLE_MIDDLE, _SPI_CLK_IDLE_HIGH, _SPI_ACTIVE_2_IDLE);                                                                 /* initialise SPI link 4 */
         SPI3CON = 0u;                                                          /* turn off the SPI module and reset it */
         SPI3BUF;                                                               /* clear the rx buffer by reading from it  */
         SPI3STATbits.SPIROV = 0;                                               /* clear the overflow bit  */
         SPI3CONbits.CKE = 1;                                                   /* data changes when clock goes from active to inactive (high to low since CKP is 0) */
         SPI3CONbits.MSTEN = 1;                                                 /* master operation  */
         SPI3CONbits.ON = 1;                                                    /* turn on SPI 3  */
         break;  
           
         case SPIDEV_4:   
         Chip_Select4 = 1;                                                      // Deselect SPi preiferal
         Chip_Select_Direction4 = 0;                                            // Set CS# pin as Output
         SPI4_Init_Advanced(_SPI_MASTER, _SPI_8_BIT, (CPU_CLOCK_SPEED/PAW3902_SPI_BUS_SPEED), _SPI_SS_DISABLE, _SPI_DATA_SAMPLE_MIDDLE, _SPI_CLK_IDLE_HIGH, _SPI_ACTIVE_2_IDLE);                                                                 /* initialise SPI link 4 */
         SPI4CON = 0u;                                                          /* turn off the SPI module and reset it */
         SPI4BUF;                                                               /* clear the rx buffer by reading from it  */
         SPI4STATbits.SPIROV = 0;                                               /* clear the overflow bit  */
         SPI4CONbits.CKE = 1;                                                   /* data changes when clock goes from active to inactive (high to low since CKP is 0) */
         SPI4CONbits.MSTEN = 1;                                                 /* master operation  */
         SPI4CONbits.ON = 1;                                                    /* turn on SPI 4  */
         break;  
           
         default:
         break;
     } 
  }
}
/*-----------------------------------------------------------------------------
 *      PAW3902_doFrameCapture():  perform frame capture TODO : send the frames ?
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev , uint8_t *frameArray
 *  Return:     uint8_t :: true success
 *----------------------------------------------------------------------------*/
uint8_t PAW3902_doFrameCapture( optical_flow_PAW3902JF_t *dev , uint8_t *frameArray )
{
    if ((dev->resetRequest == PAW_ResetDone) && (dev->state == 0u)) dev->state = 10u;
        
    switch(dev->state)
    {        
       case 10u:
       dev->ticksRef = CP0_GET(CP0_COUNT);
       dev->state = ++dev->state % UINT8_MAX;
       break;

       case 11u:
       calculateTick2Now(&dev->ticksVal, &dev->ticksRef); 
       if (dev->ticksVal > (4.0f*CPU_TICKS_PER_SECOND))
       {
          dev->state = ++dev->state % UINT8_MAX;                                                     
       }
       break;

       case 12u:
       if (PAW3902_enterFrameCaptureModeWithDelay( dev ) == true)
       {
          dev->state = ++dev->state % UINT8_MAX;                   
       }
       break;

       case 13u:
       dev->kk = 0u;
       dev->state = ++dev->state % UINT8_MAX;

       case 14u:
       if (PAW3902_captureFrame(frameArray, dev) == true)
       {
          dev->kk = ++dev->kk % UINT8_MAX;
          dev->state = ++dev->state % UINT8_MAX;                  
       }
       break;

       case 15u:
       if (dev->kk >= 5u)
       {
          dev->iterations = 0;
          dev->state = 0;
       } 
       else
       {
          dev->state = 14u;                
       }
       break;
           
       default:
       break;
       
    }
    return (dev->state == 0u);
}
/*-----------------------------------------------------------------------------
 *      PAW3902_RunImpl():  run implementation and report out to ros structure
 *
 *  Parameters: optical_flow_PAW3902JF_t *dev, optical_flow_report_t *report, uint8_t *frameArray
 *  Return:     uint8_t :: true success
 *----------------------------------------------------------------------------*/
devIC_PR_TE uint8_t PAW3902_RunImpl( optical_flow_PAW3902JF_t *dev, optical_flow_report_t *report, uint8_t *frameArray )
{
        uint8_t complet=false;
        int32_t dt_flow; 
        const int16_t delta_x_raw = ((int16_t)dev->Delta_X_H << 8u) | dev->Delta_X_L;
        const int16_t delta_y_raw = ((int16_t)dev->Delta_Y_H << 8u) | dev->Delta_Y_L;
        const uint16_t shutter = (dev->Shutter_Upper << 8u) | dev->Shutter_Lower;
        
        calculateTickDiff( &dt_flow, &dev->TickRefUpdate );                     /* set initial at initialisation */
        
        dev->_flow_dt_sum_tick += dt_flow; 
        dev->_task_sched_sum_tick += dt_flow;
        
        /* Check SQUAL & Shutter values
           To suppress false motion reports, discard Delta X and Delta Y values if the SQUAL and Shutter values meet the condition
           Bright Mode,                        SQUAL < 0x19, Shutter = 0x1FF0
           Low Light Mode,                SQUAL < 0x46, Shutter = 0x1FF0
           Super Low Light Mode,        SQUAL < 0x55, Shutter = 0x0BC0          */

        if (((dev->SQUAL < 0x19u) && (shutter >= 0x0BC0u)) || (dev == NULL))    /* out of range values, inside unacceptable change repitition time 5 secs, or NULL pointer */
        {
           return complet;
        }

        switch (dev->mode) 
        {
            case PAW3902_Mode_Bright:
                if (dev->start2Low == true)                                     /* latch the iterative changeover function once condition has been detected */ 
                {
                   if (PAW3902_modeLowLight( dev ) == true)
                   {
                      dev->_bright_to_low_counter = 0;
                      dev->start2Low = false;                                   /* clear the transition action */
                      dev->mode = PAW3902_Mode_LowLight;                        /* clear the counter so if it comes back and remains bad needs to re-count full number of consequtive errors */
                   }                                                        
                }
                else if ((shutter >= 0x1FFE) && (dev->RawData_Sum < 0x3Cu)) 
                {
                   if ((dev->_bright_to_low_counter >= 10) && (dev->_flow_dt_sum_tick >= (5.0f*CPU_TICKS_PER_SECOND)))      /* AND valid for 10 consecutive frames Bright -> LowLight and not more oftern than every 5 seconds do a changeover  */
                   { 
                      dev->start2Low = true;                                    /* latch the transition action */
                   }
                   else
                   {
                      dev->_bright_to_low_counter = ++dev->_bright_to_low_counter % INT16_MAX;
                   }
                }                        
                else 
                {
                   dev->_bright_to_low_counter = 0;
                }
                break;

            case PAW3902_Mode_LowLight:
                if (dev->start2SuperLow == true)                                /* latched iterative changeover function once condition has been detected */ 
                {
                   if (PAW3902_modeSuperLowLight( dev ) == true)
                   {
                      dev->_low_to_superlow_counter = 0;
                      dev->start2SuperLow = false;                              /* clear the transition action */
                      dev->mode = PAW3902_Mode_SuperLowLight;                   /* clear the counter so if it comes back and remains bad needs to re-count full number of consequtive errors */
                   }                                                        
                }
                else if (dev->start2Bright == true)                             /* latch the iterative changeover function once condition has been detected */ 
                {
                   if (PAW3902_modeBright( dev ) == true)
                   {
                       dev->_low_to_bright_counter = 0;
                       dev->start2Bright = false;                               /* clear the transition action */
                       dev->mode = PAW3902_Mode_Bright;                         /* clear the counter so if it comes back and remains bad needs to re-count full number of consequtive errors */
                   }                                                        
                }
                else if ((shutter >= 0x1FFEu) && (dev->RawData_Sum < 0x5Au)) 
                {
                   dev->_low_to_bright_counter = 0;

                   if ((dev->_low_to_superlow_counter >= 10) || (dev->_flow_dt_sum_tick < (5.0f*CPU_TICKS_PER_SECOND)))  /* AND valid for 10 consecutive frames LowLight -> SuperLowLight */
                   { 
                      dev->start2SuperLow = true;
                   }
                   else
                   {
                      dev->_low_to_superlow_counter = ++dev->_low_to_superlow_counter % INT16_MAX;
                   }
                } 
                else if ((shutter < 0x0BB8u)) 
                {
                    dev->_low_to_superlow_counter = 0;

                    if ((dev->_low_to_bright_counter >= 10) || (dev->_flow_dt_sum_tick < (5.0f*CPU_TICKS_PER_SECOND))) /* AND valid for 10 consecutive frames LowLight -> Bright */
                    { 
                       dev->start2Bright = true;                                 
                    }
                    else
                    {
                       dev->_low_to_bright_counter = ++dev->_low_to_bright_counter % INT16_MAX;
                    }
                }
                break;

            case PAW3902_Mode_SuperLowLight:                                           /* SuperLowLight -> LowLight */
                if (dev->start2Low == true)                                     /* latch the iterative changeover function once condition has been detected */ 
                {
                    if (PAW3902_modeLowLight( dev ) == true)
                    {
                       dev->_superlow_to_low_counter = 0;
                       dev->start2Low = false;                                  /* clear the transition action */
                       dev->mode = PAW3902_Mode_LowLight;                       /* clear the counter so if it comes back and remains bad needs to re-count full number of consequtive errors */
                    }                                                        
                }
                else if ((shutter < 0x03E8u)) 
                {
                    if ((dev->_superlow_to_low_counter >= 10) && (dev->_flow_dt_sum_tick > (5.0f*CPU_TICKS_PER_SECOND))) // AND valid for 10 consecutive frames
                    { 
                        dev->start2Low = true;                                  /* latch the transition action */
                    }
                    else
                    {
                        dev->_superlow_to_low_counter = ++dev->_superlow_to_low_counter % INT16_MAX;
                    }
                }
                else if (shutter < 0x01F4u)                                     // Page 35 switching scheme Drop out of superlowlight mode as soon as the Shutter less than 500
                {
                   dev->start2Low = true;                                       /* latch the transition action */                
                } 
                else 
                {
                   dev->_superlow_to_low_counter = 0;
                }
                break;
                
                default:
                break;
        }
   
        dev->_flow_sum_x += delta_x_raw;
        dev->_flow_sum_y += delta_y_raw;

        dev->_frame_count_since_last=++dev->_frame_count_since_last % UINT16_MAX;
        report->timestamp = dev->TickRefUpdate;                                 /* ======== set the output object =====================   */

        report->pixel_flow_x_integral = dev->_flow_sum_x / 500.0f;              /* proportional factor + convert from pixels to radians  */
        report->pixel_flow_y_integral = dev->_flow_sum_y / 500.0f;              /* proportional factor + convert from pixels to radians  */

        // report->quaterni = mkquat(_yaw_rotation, report->pixel_flow_x_integral, report->pixel_flow_x_integral, 0.0f);          // rotate measurements in yaw from sensor frame to body frame according to parameter SENS_FLOW_ROT

        report->frame_count_since_last_readout = dev->_frame_count_since_last;
        report->integration_timespan = (dev->_flow_dt_sum_tick / CPU_TICKS_PER_SECOND) * 1000.0f;        // microseconds

        report->sensor_id = 0;
        report->quality = dev->SQUAL;

        report->gyro_x_rate_integral = NaN;                                     /* No gyro on this board */
        report->gyro_y_rate_integral = NaN;
        report->gyro_z_rate_integral = NaN;

        /* set (conservative) specs according to datasheet */
        report->max_flow_rate = 5.0f;                                           /* Datasheet: 7.4 rad/s  */
        report->min_ground_distance = 0.08f;                                    /* Datasheet: 80mm  */
        report->max_ground_distance = 30.0f;                                    /* Datasheet: infinity */

        // _optical_flow_pub.publish(report);
        if (dev->_task_sched_sum_tick >= dev->schedOnInterval)                  /* schedule an update if needed */
        {
           if (dev->iterations >= 25u)                                          /* for every 25 iterations do 5 frame captures and send */
           {
              if (PAW3902_doFrameCapture( dev, frameArray ) == true) dev->_task_sched_sum_tick = 0LU;           
           }
           else
           {
              if (PAW3902_getUpdate( dev ) == true) dev->_task_sched_sum_tick = 0LU;
           }
        }
        dev->_flow_dt_sum_tick = 0;                                             /* reset */
        dev->_flow_sum_x = 0;
        dev->_flow_sum_y = 0;
        dev->_frame_count_since_last = 0;
        complet = true;
        
        return complet;
}
#endif /* PAW3902 optical flow used */

#if defined(MPU6050_ACC_GYRO)
// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
// 11/28/2014 by Marton Sebok <sebokmarton@gmail.com>
//
// Changelog:
//     2014-11-28 - ported to PIC18 peripheral library from Arduino code
//     2020-10-05 - ported to PIC32 and FT900 by ACP Aviation
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg
Copyright (c) 2014 Marton Sebok
Copyright (c) 2017 Daichou
Copyright (c) 2020 AirCamPro
 *
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#define MasterWriteI2C( byte ) I2C1_Write( byte )
#define I2CSTATbits I2C1STATbits
#define I2CCONbits I2C1CONbits
#define StartI2C() I2C1_Start()
#define StopI2C() I2C1_Stop()
#define MasterReadI2C() I2C1_Read(_I2C_ACK)
#define MasterReadI2CNACK() I2C1_Read(_I2C_NACK)
#define MI2CIF I2C1MIF 
#define MI2CIE I2C1MIE

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
devIC_PR_TE bool I2Cdev_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* dataV) 
{
    uint8_t i;
                
    IFS0bits.MI2CIF = 0;                                                        /* clear i2c master event flag */
    IEC0bits.MI2CIE = 0;                                                        /* clear i2c master event enable */

    /*Master Start*/
    StartI2C();

    while(I2CCONbits.SEN);                                                      /* Wait util Start sequence is completed */
    IFS0bits.MI2CIF = 0;                                                        /* Clear interrupt flag */

    /* Master send AD+W */
    /* Write Slave address (Write)*/
    MasterWriteI2C(devAddr << 1 | 0x00);
    /* Wait till address is transmitted */
    while(I2CSTATbits.TBF);                                                     // 8 clock cycles

    /*Slave send ACK*/
    while(!IFS0bits.MI2CIF);                                                    // Wait for 9th clock cycle
    IFS0bits.MI2CIF = 0;                                                        // Clear interrupt flag
    while(I2CSTATbits.ACKSTAT);

    /*Master send RA*/
    /* Write Slave address (Write)*/
    MasterWriteI2C(regAddr);
    /* Wait till address is transmitted */
    while(I2CSTATbits.TBF);                                                     // 8 clock cycles

    /*Slave send ACK*/
    while(!IFS0bits.MI2CIF);                                                    // Wait for 9th clock cycle
    IFS0bits.MI2CIF = 0;                                                        // Clear interrupt flag
    while(I2CSTATbits.ACKSTAT);

    /*Master send data*/
    /* Transmit string of data */
    //MasterputsI2C(data);
    for (i = 0 ; i < length ; i++)
    {
        MasterWriteI2C(dataV[i]);
        /* Wait till address is transmitted */
        while(I2CSTATbits.TBF);                                                 // 8 clock cycles

        /*Slave send ACK*/
        while(!IFS0bits.MI2CIF);                                                // Wait for 9th clock cycle
        IFS0bits.MI2CIF = 0;                                                    // Clear interrupt flag
    }
    StopI2C();

    while(I2CCONbits.PEN);                                                      /* Wait till stop sequence is completed */

    return true;
}
/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
devIC_PR_TE bool I2Cdev_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t dataV) 
{
    return I2Cdev_writeBytes(devAddr, regAddr, 1, &dataV);
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
devIC_PR_TE bool I2Cdev_writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* dataV) 
{
    uint8_t OneByte[100u];
    int16_t i;
    for (i = 0 ; i < length ; i++)
    {
        OneByte[i*2] = dataV[i]>>8u;
        OneByte[i*2+1] = dataV[i] & 0XFFu;
    }
    I2Cdev_writeBytes(devAddr,regAddr,length*2u,OneByte);
    return true;
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
devIC_PR_TE bool I2Cdev_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t dataV) 
{
    return I2Cdev_writeWords(devAddr, regAddr, 1u, &dataV);
}
/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @return Number of bytes read (-1 indicates failure)
 */
devIC_PR_TE int8_t I2Cdev_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *dataV) 
{
    int8_t i;
    /* Clear interrupt flag */
    IFS0bits.MI2CIF = 0;                                                        /* clear i2c master event flag */
    IEC0bits.MI2CIE = 0;                                                        /* clear i2c master event enable */

    StartI2C();                                                                 /*master Start*/

    while(I2CCONbits.SEN);                                                      /* Wait till Start sequence is completed */

    /*master send AD+W*/
    MasterWriteI2C(devAddr << 1 | 0x00);                                        /* Write Slave Address (Write)*/

    /* Wait until address is transmitted */
    while(I2CSTATbits.TBF);                                                     // 8 clock cycles

    /*Slave send Ack*/
    while(!IFS0bits.MI2CIF);                                                    // Wait for 9th clock cycle
    IFS0bits.MI2CIF = 0;                                                        // Clear interrupt flag
    //  while(I2CSTATbits.ACKSTAT);
    /*  Master send RA */
    /*  Write Register Address */
    MasterWriteI2C(regAddr);

    /* Wait until address is transmitted */
    while(I2CSTATbits.TBF);                                                     // 8 clock cycles

    /*Slave send ACK*/
    while(!IFS0bits.MI2CIF);                                                    // Wait for 9th clock cycle
    IFS0bits.MI2CIF = 0;                                                        // Clear interrupt flag
    while(I2CSTATbits.ACKSTAT);

    StopI2C();                                                                  /* Master Pause */

    while(I2CCONbits.PEN);                                                      /* Wait till stop sequence is completed */

    StartI2C();                                                                 /* Master Start */
    while(I2CCONbits.SEN);                                                      /* Wait till Start sequence is completed */

    /*Master send AD+R*/
    MasterWriteI2C(devAddr << 1 | 0x01);                                        /* Write Slave Address (Read)*/
    /* Wait until address is transmitted */
    while(I2CSTATbits.TBF);                                                     // 8 clock cycles

    /*Slave send Ack*/
    while(!IFS0bits.MI2CIF);                                                    // Wait for 9th clock cycle
    IFS0bits.MI2CIF = 0;                                                        // Clear interrupt flag
    while(I2CSTATbits.ACKSTAT);

    /*Slave send DATA*/
    //uint16_t flag = MastergetsI2C(length,data,I2C_DATA_WAIT);

    /*Slave send NACK*/
    //MastergetsI2C(length,data,100);
    //NotAckI2C();

    dataV[0] = MasterReadI2C();                                                 // I2C1_Read(_I2C_ACK);
    for (i = 1 ; i < length ; i++ )
    {
        while(I2CCONbits.ACKEN == 1);                                           // AckI2C();
        if (i < length)
           dataV[i] = MasterReadI2C();
        else
           dataV[i] = MasterReadI2CNACK();
    }
    while(I2CCONbits.ACKEN == 1);                                               // NotAckI2C();

    StopI2C();                                                                  /*Master Pause*/

    while(I2CCONbits.PEN);                                                      /* Wait till stop sequence is completed */

    return length;
}
/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @return Status of read operation (true = success)
 */
devIC_PR_TE int8_t I2Cdev_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *dataV) 
{
    return I2Cdev_readBytes(devAddr, regAddr, 1, dataV);
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @return Number of words read (-1 indicates failure)
 */
devIC_PR_TE int8_t I2Cdev_readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *dataV) 
{
    uint8_t Onebyte[100u];
    int8_t i;
    I2Cdev_readBytes(devAddr,regAddr,length*2,Onebyte);
    for (i = 0 ; i < length ; i++ )
    {
        dataV[i] = Onebyte[i*2u] << 8u | Onebyte[i*2u+1u];
    }
    return length;
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @return Status of read operation (true = success)
 */
devIC_PR_TE int8_t I2Cdev_readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *dataV) 
{
    return I2Cdev_readWords(devAddr, regAddr, 1, dataV);
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
devIC_PR_TE int8_t I2Cdev_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *dataV) 
{
    uint8_t b;
    uint8_t count = I2Cdev_readByte(devAddr, regAddr, &b);
    *dataV = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
devIC_PR_TE int8_t I2Cdev_readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *dataV) 
{
    uint16_t b;
    uint8_t count = I2Cdev_readWord(devAddr, regAddr, &b);
    *dataV = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (true = success)
 */
devIC_PR_TE int8_t I2Cdev_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *dataV) 
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b, mask;
    if ((count = I2Cdev_readByte(devAddr, regAddr, &b)) != 0) 
    {
        mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *dataV = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
devIC_PR_TE int8_t I2Cdev_readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *dataV) 
{
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w,mask;
    if ((count = I2Cdev_readWord(devAddr, regAddr, &w)) != 0) 
    {
        mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *dataV = w;
    }
    return count;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
devIC_PR_TE bool I2Cdev_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t dataV) 
{
    uint8_t b;
    I2Cdev_readByte(devAddr, regAddr, &b);
    b = (dataV != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return I2Cdev_writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
devIC_PR_TE bool I2Cdev_writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t dataV) 
{
    uint16_t w;
    I2Cdev_readWord(devAddr, regAddr, &w);
    w = (dataV != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return I2Cdev_writeWord(devAddr, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
devIC_PR_TE bool I2Cdev_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t dataV) 
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b,mask;
    if (I2Cdev_readByte(devAddr, regAddr, &b) != 0) 
    {
        mask = ((1 << length) - 1) << (bitStart - length + 1);
        dataV <<= (bitStart - length + 1);                                      // shift data into correct position
        dataV &= mask;                                                          // zero all non-important bits in data
        b &= ~(mask);                                                           // zero all important bits in existing byte
        b |= dataV;                                                             // combine data with existing byte
        return I2Cdev_writeByte(devAddr, regAddr, b);
    } 
    else 
    {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
devIC_PR_TE bool I2Cdev_writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t dataV) 
{
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w,mask;
    if (I2Cdev_readWord(devAddr, regAddr, &w) != 0) 
    {
        mask = ((1 << length) - 1) << (bitStart - length + 1);
        dataV <<= (bitStart - length + 1);                                      // shift data into correct position
        dataV &= mask;                                                          // zero all non-important bits in data
        w &= ~(mask);                                                           // zero all important bits in existing word
        w |= dataV;                                                             // combine data with existing word
        return I2Cdev_writeWord(devAddr, regAddr, w);
    } 
    else 
    {
        return false;
    }
}
/* =================== MPU6050 Invensense =================================== */

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
devIC_PR_TE void MPU6050_setClockSource(uint8_t source, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}
/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
devIC_PR_TE void MPU6050_setFullScaleGyroRange(uint8_t range, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}
devIC_PR_TE void mpu6050SetGyroXSelfTest(bool enabled, mpu6050_Invensense_t *mpu6050)
{
  I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, enabled);
}

devIC_PR_TE void mpu6050SetGyroYSelfTest(bool enabled, mpu6050_Invensense_t *mpu6050)
{
  I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_YG_ST_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, enabled);
}

devIC_PR_TE void mpu6050SetGyroZSelfTest(bool enabled, mpu6050_Invensense_t *mpu6050)
{
  I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_ZG_ST_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, enabled);
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
devIC_PR_TE void MPU6050_setFullScaleAccelRange(uint8_t range, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
devIC_PR_TE void MPU6050_setSleepEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}
/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
devIC_PR_TE void MPU6050_initialize( mpu6050_Invensense_t *mpu6050 ) 
{
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO,mpu6050);
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_250,mpu6050);
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2,mpu6050);
    MPU6050_setSleepEnabled(false,mpu6050);                                     // thanks to Jack Elston for pointing this one out!
}

// ========= WHO_AM_I register ===========================

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100, 0x34).
 * @return Device ID (6 bits only! should be 0x34)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
devIC_PR_TE uint8_t MPU6050_getDeviceID( mpu6050_Invensense_t *mpu6050 ) 
{
    uint8_t buffer[MPU6050_WHO_AM_I_LENGTH];
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, buffer);
    return buffer[0u];
}
/** Set Device ID.
 * Write a new ID into the WHO_AM_I register (no idea why this should ever be
 * necessary though).
 * @param id New device ID to set.
 * @see getDeviceID()
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
devIC_PR_TE void MPU6050_setDeviceID( uint8_t id, mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id);
}
/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
devIC_PR_TE bool MPU6050_testConnection( mpu6050_Invensense_t *mpu6050 ) 
{
    return MPU6050_getDeviceID( mpu6050 ) == 0x34u;
}

// AUX_VDDIO register (InvenSense demo code calls this RA_*G_OFFS_TC)

/** Get the auxiliary I2C supply voltage level.
 * When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
 * 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
 * the MPU-6000, which does not have a VLOGIC pin.
 * @return I2C supply voltage level (0=VLOGIC, 1=VDD)
 */
devIC_PR_TE uint8_t MPU6050_getAuxVDDIOLevel( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set the auxiliary I2C supply voltage level.
 * When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
 * 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
 * the MPU-6000, which does not have a VLOGIC pin.
 * @param level I2C supply voltage level (0=VLOGIC, 1=VDD)
 */
devIC_PR_TE void MPU6050_setAuxVDDIOLevel( uint8_t level, mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, level);
}

// SMPLRT_DIV register

/** Get gyroscope output rate divider.
 * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
 * Motion detection, and Free Fall detection are all based on the Sample Rate.
 * The Sample Rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26).
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
 * of the MPU-6000/MPU-6050 Product Specification document.
 *
 * @return Current sample rate
 * @see MPU6050_RA_SMPLRT_DIV
 */
devIC_PR_TE uint8_t MPU6050_getRate( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_SMPLRT_DIV, mpu6050->buffer);
    return mpu6050->buffer[0];
}
/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
devIC_PR_TE void MPU6050_setRate(uint8_t rate, mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_SMPLRT_DIV, rate);
}

// CONFIG register

/** Get external FSYNC configuration.
 * Configures the external Frame Synchronization (FSYNC) pin sampling. An
 * external signal connected to the FSYNC pin can be sampled by configuring
 * EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
 * strobes may be captured. The latched FSYNC signal will be sampled at the
 * Sampling Rate, as defined in register 25. After sampling, the latch will
 * reset to the current FSYNC signal state.
 *
 * The sampled value will be reported in place of the least significant bit in
 * a sensor data register determined by the value of EXT_SYNC_SET according to
 * the following table.
 *
 * <pre>
 * EXT_SYNC_SET | FSYNC Bit Location
 * -------------+-------------------
 * 0            | Input disabled
 * 1            | TEMP_OUT_L[0]
 * 2            | GYRO_XOUT_L[0]
 * 3            | GYRO_YOUT_L[0]
 * 4            | GYRO_ZOUT_L[0]
 * 5            | ACCEL_XOUT_L[0]
 * 6            | ACCEL_YOUT_L[0]
 * 7            | ACCEL_ZOUT_L[0]
 * </pre>
 *
 * @return FSYNC configuration value
 */
devIC_PR_TE uint8_t MPU6050_getExternalFrameSync( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
devIC_PR_TE void MPU6050_setExternalFrameSync( uint8_t sync, mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}
/** Get digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @return DLFP configuration
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
devIC_PR_TE uint8_t MPU6050_getDLPFMode( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
devIC_PR_TE void MPU6050_setDLPFMode(uint8_t mode, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
devIC_PR_TE uint8_t MPU6050_getFullScaleGyroRange( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0];
}


// ACCEL_CONFIG register

/** Get self-test enabled setting for accelerometer X axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
devIC_PR_TE bool MPU6050_getAccelXSelfTest( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get self-test enabled setting for accelerometer X axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
devIC_PR_TE void MPU6050_setAccelXSelfTest(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, enabled);
}
/** Get self-test enabled value for accelerometer Y axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
devIC_PR_TE bool MPU6050_getAccelYSelfTest( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get self-test enabled value for accelerometer Y axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
devIC_PR_TE void MPU6050_setAccelYSelfTest(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, enabled);
}
/** Get self-test enabled value for accelerometer Z axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
devIC_PR_TE bool MPU6050_getAccelZSelfTest( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set self-test enabled value for accelerometer Z axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
devIC_PR_TE void MPU6050_setAccelZSelfTest(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, enabled);
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
devIC_PR_TE uint8_t MPU6050_getFullScaleAccelRange( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}

/** Get the high-pass filter configuration.
 * The DHPF is a filter module in the path leading to motion detectors (Free
 * Fall, Motion threshold, and Zero Motion). The high pass filter output is not
 * available to the data registers (see Figure in Section 8 of the MPU-6000/
 * MPU-6050 Product Specification document).
 *
 * The high pass filter has three modes:
 *
 * <pre>
 *    Reset: The filter output settles to zero within one sample. This
 *           effectively disables the high pass filter. This mode may be toggled
 *           to quickly settle the filter.
 *
 *    On:    The high pass filter will pass signals above the cut off frequency.
 *
 *    Hold:  When triggered, the filter holds the present sample. The filter
 *           output will be the difference between the input sample and the held
 *           sample.
 * </pre>
 *
 * <pre>
 * ACCEL_HPF | Filter Mode | Cut-off Frequency
 * ----------+-------------+------------------
 * 0         | Reset       | None
 * 1         | On          | 5Hz
 * 2         | On          | 2.5Hz
 * 3         | On          | 1.25Hz
 * 4         | On          | 0.63Hz
 * 7         | Hold        | None
 * </pre>
 *
 * @return Current high-pass filter configuration
 * @see MPU6050_DHPF_RESET
 * @see MPU6050_RA_ACCEL_CONFIG
 */
devIC_PR_TE uint8_t MPU6050_getDHPFMode( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set the high-pass filter configuration.
 * @param bandwidth New high-pass filter configuration
 * @see setDHPFMode()
 * @see MPU6050_DHPF_RESET
 * @see MPU6050_RA_ACCEL_CONFIG
 */
devIC_PR_TE void MPU6050_setDHPFMode(uint8_t bandwidth, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH, bandwidth);
}
// FF_THR register

/** Get free-fall event acceleration threshold.
 * This register configures the detection threshold for Free Fall event
 * detection. The unit of FF_THR is 1LSB = 2mg. Free Fall is detected when the
 * absolute value of the accelerometer measurements for the three axes are each
 * less than the detection threshold. This condition increments the Free Fall
 * duration counter (Register 30). The Free Fall interrupt is triggered when the
 * Free Fall duration counter reaches the time specified in FF_DUR.
 *
 * For more details on the Free Fall detection interrupt, see Section 8.2 of the
 * MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
 * 58 of this document.
 *
 * @return Current free-fall acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_FF_THR
 */
devIC_PR_TE uint8_t MPU6050_getFreefallDetectionThreshold(mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FF_THR, mpu6050->buffer);
    return mpu6050->buffer[0];
}
/** Get free-fall event acceleration threshold.
 * @param threshold New free-fall acceleration threshold value (LSB = 2mg)
 * @see getFreefallDetectionThreshold()
 * @see MPU6050_RA_FF_THR
 */
devIC_PR_TE void MPU6050_setFreefallDetectionThreshold(uint8_t threshold, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FF_THR, threshold);
}
// FF_DUR register

/** Get free-fall event duration threshold.
 * This register configures the duration counter threshold for Free Fall event
 * detection. The duration counter ticks at 1kHz, therefore FF_DUR has a unit
 * of 1 LSB = 1 ms.
 *
 * The Free Fall duration counter increments while the absolute value of the
 * accelerometer measurements are each less than the detection threshold
 * (Register 29). The Free Fall interrupt is triggered when the Free Fall
 * duration counter reaches the time specified in this register.
 *
 * For more details on the Free Fall detection interrupt, see Section 8.2 of
 * the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
 * and 58 of this document.
 *
 * @return Current free-fall duration threshold value (LSB = 1ms)
 * @see MPU6050_RA_FF_DUR
 */
devIC_PR_TE uint8_t MPU6050_getFreefallDetectionDuration( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FF_DUR, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get free-fall event duration threshold.
 * @param duration New free-fall duration threshold value (LSB = 1ms)
 * @see getFreefallDetectionDuration()
 * @see MPU6050_RA_FF_DUR
 */
devIC_PR_TE void MPU6050_setFreefallDetectionDuration(uint8_t duration, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FF_DUR, duration);
}

// MOT_THR register

/** Get motion detection event acceleration threshold.
 * This register configures the detection threshold for Motion interrupt
 * generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the
 * absolute value of any of the accelerometer measurements exceeds this Motion
 * detection threshold. This condition increments the Motion detection duration
 * counter (Register 32). The Motion detection interrupt is triggered when the
 * Motion Detection counter reaches the time count specified in MOT_DUR
 * (Register 32).
 *
 * The Motion interrupt will indicate the axis and polarity of detected motion
 * in MOT_DETECT_STATUS (Register 97).
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
 * 58 of this document.
 *
 * @return Current motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_MOT_THR
 */
devIC_PR_TE uint8_t MPU6050_getMotionDetectionThreshold( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_THR, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
devIC_PR_TE void MPU6050_setMotionDetectionThreshold(uint8_t threshold, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_THR, threshold);
}

// MOT_DUR register

/** Get motion detection event duration threshold.
 * This register configures the duration counter threshold for Motion interrupt
 * generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit
 * of 1LSB = 1ms. The Motion detection duration counter increments when the
 * absolute value of any of the accelerometer measurements exceeds the Motion
 * detection threshold (Register 31). The Motion detection interrupt is
 * triggered when the Motion detection counter reaches the time count specified
 * in this register.
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document.
 *
 * @return Current motion detection duration threshold value (LSB = 1ms)
 * @see MPU6050_RA_MOT_DUR
 */
devIC_PR_TE uint8_t MPU6050_getMotionDetectionDuration( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DUR, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
devIC_PR_TE void MPU6050_setMotionDetectionDuration(uint8_t duration, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DUR, duration);
}

// ZRMOT_THR register

/** Get zero motion detection event acceleration threshold.
 * This register configures the detection threshold for Zero Motion interrupt
 * generation. The unit of ZRMOT_THR is 1LSB = 2mg. Zero Motion is detected when
 * the absolute value of the accelerometer measurements for the 3 axes are each
 * less than the detection threshold. This condition increments the Zero Motion
 * duration counter (Register 34). The Zero Motion interrupt is triggered when
 * the Zero Motion duration counter reaches the time count specified in
 * ZRMOT_DUR (Register 34).
 *
 * Unlike Free Fall or Motion detection, Zero Motion detection triggers an
 * interrupt both when Zero Motion is first detected and when Zero Motion is no
 * longer detected.
 *
 * When a zero motion event is detected, a Zero Motion Status will be indicated
 * in the MOT_DETECT_STATUS register (Register 97). When a motion-to-zero-motion
 * condition is detected, the status bit is set to 1. When a zero-motion-to-
 * motion condition is detected, the status bit is set to 0.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
 * and 58 of this document.
 *
 * @return Current zero motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_ZRMOT_THR
 */
devIC_PR_TE uint8_t MPU6050_getZeroMotionDetectionThreshold( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ZRMOT_THR, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
devIC_PR_TE void MPU6050_setZeroMotionDetectionThreshold(uint8_t threshold, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ZRMOT_THR, threshold);
}

// ZRMOT_DUR register

/** Get zero motion detection event duration threshold.
 * This register configures the duration counter threshold for Zero Motion
 * interrupt generation. The duration counter ticks at 16 Hz, therefore
 * ZRMOT_DUR has a unit of 1 LSB = 64 ms. The Zero Motion duration counter
 * increments while the absolute value of the accelerometer measurements are
 * each less than the detection threshold (Register 33). The Zero Motion
 * interrupt is triggered when the Zero Motion duration counter reaches the time
 * count specified in this register.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document, as well as Registers 56
 * and 58 of this document.
 *
 * @return Current zero motion detection duration threshold value (LSB = 64ms)
 * @see MPU6050_RA_ZRMOT_DUR
 */
devIC_PR_TE uint8_t MPU6050_getZeroMotionDetectionDuration( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ZRMOT_DUR, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
devIC_PR_TE void MPU6050_setZeroMotionDetectionDuration( uint8_t duration, mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ZRMOT_DUR, duration);
}

// FIFO_EN register

/** Get temperature FIFO enabled value.
 * When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and
 * 66) to be written into the FIFO mpu6050.buffer.
 * @return Current temperature FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE bool MPU6050_getTempFIFOEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set temperature FIFO enabled value.
 * @param enabled New temperature FIFO enabled value
 * @see getTempFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE void MPU6050_setTempFIFOEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, enabled);
}
/** Get gyroscope X-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
 * 68) to be written into the FIFO mpu6050.buffer.
 * @return Current gyroscope X-axis FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE bool MPU6050_getXGyroFIFOEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set gyroscope X-axis FIFO enabled value.
 * @param enabled New gyroscope X-axis FIFO enabled value
 * @see getXGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE void MPU6050_setXGyroFIFOEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, enabled);
}
/** Get gyroscope Y-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
 * 70) to be written into the FIFO mpu6050.buffer.
 * @return Current gyroscope Y-axis FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE bool MPU6050_getYGyroFIFOEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set gyroscope Y-axis FIFO enabled value.
 * @param enabled New gyroscope Y-axis FIFO enabled value
 * @see getYGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE void MPU6050_setYGyroFIFOEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, enabled);
}
/** Get gyroscope Z-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
 * 72) to be written into the FIFO mpu6050.buffer.
 * @return Current gyroscope Z-axis FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE bool MPU6050_getZGyroFIFOEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set gyroscope Z-axis FIFO enabled value.
 * @param enabled New gyroscope Z-axis FIFO enabled value
 * @see getZGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE void MPU6050_setZGyroFIFOEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, enabled);
}
/** Get accelerometer FIFO enabled value.
 * When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
 * ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
 * written into the FIFO mpu6050.buffer.
 * @return Current accelerometer FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE bool MPU6050_getAccelFIFOEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set accelerometer FIFO enabled value.
 * @param enabled New accelerometer FIFO enabled value
 * @see getAccelFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE void MPU6050_setAccelFIFOEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, enabled);
}
/** Get Slave 2 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 2 to be written into the FIFO mpu6050.buffer.
 * @return Current Slave 2 FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE bool MPU6050_getSlave2FIFOEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Slave 2 FIFO enabled value.
 * @param enabled New Slave 2 FIFO enabled value
 * @see getSlave2FIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE void MPU6050_setSlave2FIFOEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, enabled);
}
/** Get Slave 1 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 1 to be written into the FIFO mpu6050.buffer.
 * @return Current Slave 1 FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE bool MPU6050_getSlave1FIFOEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Slave 1 FIFO enabled value.
 * @param enabled New Slave 1 FIFO enabled value
 * @see getSlave1FIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE void MPU6050_setSlave1FIFOEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, enabled);
}
/** Get Slave 0 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 0 to be written into the FIFO mpu6050.buffer.
 * @return Current Slave 0 FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE bool MPU6050_getSlave0FIFOEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Slave 0 FIFO enabled value.
 * @param enabled New Slave 0 FIFO enabled value
 * @see getSlave0FIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
devIC_PR_TE void MPU6050_setSlave0FIFOEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, enabled);
}

// I2C_SLV* registers (Slave 0-3)

/** Get the I2C address of the specified slave (0-3).
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * In read mode, the result of the read is placed in the lowest available 
 * EXT_SENS_DATA register. For further information regarding the allocation of
 * read results, please refer to the EXT_SENS_DATA register description
 * (Registers 73 - 96).
 *
 * The MPU-6050 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions (getSlave4* and setSlave4*).
 *
 * I2C data transactions are performed at the Sample Rate, as defined in
 * Register 25. The user is responsible for ensuring that I2C data transactions
 * to and from each enabled Slave can be completed within a single period of the
 * Sample Rate.
 *
 * The I2C slave access rate can be reduced relative to the Sample Rate. This
 * reduced access rate is determined by I2C_MST_DLY (Register 52). Whether a
 * slave's access rate is reduced relative to the Sample Rate is determined by
 * I2C_MST_DELAY_CTRL (Register 103).
 *
 * The processing order for the slaves is fixed. The sequence followed for
 * processing the slaves is Slave 0, Slave 1, Slave 2, Slave 3 and Slave 4. If a
 * particular Slave is disabled it will be skipped.
 *
 * Each slave can either be accessed at the sample rate or at a reduced sample
 * rate. In a case where some slaves are accessed at the Sample Rate and some
 * slaves are accessed at the reduced rate, the sequence of accessing the slaves
 * (Slave 0 to Slave 4) is still followed. However, the reduced rate slaves will
 * be skipped if their access rate dictates that they should not be accessed
 * during that particular cycle. For further information regarding the reduced
 * access rate, please refer to Register 52. Whether a slave is accessed at the
 * Sample Rate or at the reduced rate is determined by the Delay Enable bits in
 * Register 103.
 *
 * @param num Slave number (0-3)
 * @return Current address for specified slave
 * @see MPU6050_RA_I2C_SLV0_ADDR
 */
devIC_PR_TE uint8_t MPU6050_getSlaveAddress(uint8_t num, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return 0;
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_ADDR + num*3, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU6050_RA_I2C_SLV0_ADDR
 */
devIC_PR_TE void MPU6050_setSlaveAddress(uint8_t num, uint8_t address, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return;
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_ADDR + num*3, address);
}
/** Get the active internal register for the specified slave (0-3).
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * The MPU-6050 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions.
 *
 * @param num Slave number (0-3)
 * @return Current active register for specified slave
 * @see MPU6050_RA_I2C_SLV0_REG
 */
devIC_PR_TE uint8_t MPU6050_getSlaveRegister(uint8_t num, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return 0;
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_REG + num*3, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set the active internal register for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param reg New active register for specified slave
 * @see getSlaveRegister()
 * @see MPU6050_RA_I2C_SLV0_REG
 */
devIC_PR_TE void MPU6050_setSlaveRegister(uint8_t num, uint8_t reg, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return;
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_REG + num*3, reg);
}
/** Get the enabled value for the specified slave (0-3).
 * When set to 1, this bit enables Slave 0 for data transfer operations. When
 * cleared to 0, this bit disables Slave 0 from data transfer operations.
 * @param num Slave number (0-3)
 * @return Current enabled value for specified slave
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
devIC_PR_TE bool MPU6050_getSlaveEnabled(uint8_t num, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return 0;
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_CTRL + num*3, MPU6050_I2C_SLV_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set the enabled value for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New enabled value for specified slave
 * @see getSlaveEnabled()
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
devIC_PR_TE void MPU6050_setSlaveEnabled(uint8_t num, bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return;
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_CTRL + num*3, MPU6050_I2C_SLV_EN_BIT, enabled);
}
/** Get word pair byte-swapping enabled for the specified slave (0-3).
 * When set to 1, this bit enables byte swapping. When byte swapping is enabled,
 * the high and low bytes of a word pair are swapped. Please refer to
 * I2C_SLV0_GRP for the pairing convention of the word pairs. When cleared to 0,
 * bytes transferred to and from Slave 0 will be written to EXT_SENS_DATA
 * registers in the order they were transferred.
 *
 * @param num Slave number (0-3)
 * @return Current word pair byte-swapping enabled value for specified slave
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
devIC_PR_TE bool MPU6050_getSlaveWordByteSwap(uint8_t num, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return 0;
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_CTRL + num*3, MPU6050_I2C_SLV_BYTE_SW_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set word pair byte-swapping enabled for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New word pair byte-swapping enabled value for specified slave
 * @see getSlaveWordByteSwap()
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
devIC_PR_TE void MPU6050_setSlaveWordByteSwap(uint8_t num, bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return;
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_CTRL + num*3, MPU6050_I2C_SLV_BYTE_SW_BIT, enabled);
}
/** Get write mode for the specified slave (0-3).
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @param num Slave number (0-3)
 * @return Current write mode for specified slave (0 = register address + data, 1 = data only)
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
devIC_PR_TE bool MPU6050_getSlaveWriteMode(uint8_t num, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return 0;
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_CTRL + num*3, MPU6050_I2C_SLV_REG_DIS_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set write mode for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param mode New write mode for specified slave (0 = register address + data, 1 = data only)
 * @see getSlaveWriteMode()
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
devIC_PR_TE void MPU6050_setSlaveWriteMode(uint8_t num, bool mode, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return;
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_CTRL + num*3, MPU6050_I2C_SLV_REG_DIS_BIT, mode);
}
/** Get word pair grouping order offset for the specified slave (0-3).
 * This sets specifies the grouping order of word pairs received from registers.
 * When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc (even,
 * then odd register addresses) are paired to form a word. When set to 1, bytes
 * from register addresses are paired 1 and 2, 3 and 4, etc. (odd, then even
 * register addresses) are paired to form a word.
 *
 * @param num Slave number (0-3)
 * @return Current word pair grouping order offset for specified slave
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
devIC_PR_TE bool MPU6050_getSlaveWordGroupOffset(uint8_t num, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return 0;
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_CTRL + num*3, MPU6050_I2C_SLV_GRP_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set word pair grouping order offset for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New word pair grouping order offset for specified slave
 * @see getSlaveWordGroupOffset()
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
devIC_PR_TE void MPU6050_setSlaveWordGroupOffset(uint8_t num, bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return;
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_CTRL + num*3, MPU6050_I2C_SLV_GRP_BIT, enabled);
}
/** Get number of bytes to read for the specified slave (0-3).
 * Specifies the number of bytes transferred to and from Slave 0. Clearing this
 * bit to 0 is equivalent to disabling the register by writing 0 to I2C_SLV0_EN.
 * @param num Slave number (0-3)
 * @return Number of bytes to read for specified slave
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
devIC_PR_TE uint8_t MPU6050_getSlaveDataLength(uint8_t num, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return 0;
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_CTRL + num*3, MPU6050_I2C_SLV_LEN_BIT, MPU6050_I2C_SLV_LEN_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set number of bytes to read for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param length Number of bytes to read for specified slave
 * @see getSlaveDataLength()
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
devIC_PR_TE void MPU6050_setSlaveDataLength(uint8_t num, uint8_t length, mpu6050_Invensense_t *mpu6050) 
{
    if (num > 3) return;
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_CTRL + num*3, MPU6050_I2C_SLV_LEN_BIT, MPU6050_I2C_SLV_LEN_LENGTH, length);
}

// I2C_SLV* registers (Slave 4)

/** Get the I2C address of Slave 4.
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * @return Current address for Slave 4
 * @see getSlaveAddress()
 * @see MPU6050_RA_I2C_SLV4_ADDR
 */
devIC_PR_TE uint8_t MPU6050_getSlave4Address( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_ADDR, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set the I2C address of Slave 4.
 * @param address New address for Slave 4
 * @see getSlave4Address()
 * @see MPU6050_RA_I2C_SLV4_ADDR
 */
devIC_PR_TE void MPU6050_setSlave4Address(uint8_t address, mpu6050_Invensense_t *mpu6050)
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_ADDR, address);
}
/** Get the active internal register for the Slave 4.
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * @return Current active register for Slave 4
 * @see MPU6050_RA_I2C_SLV4_REG
 */
devIC_PR_TE uint8_t MPU6050_getSlave4Register( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_REG, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set the active internal register for Slave 4.
 * @param reg New active register for Slave 4
 * @see getSlave4Register()
 * @see MPU6050_RA_I2C_SLV4_REG
 */
devIC_PR_TE void MPU6050_setSlave4Register(uint8_t reg, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_REG, reg);
}
/** Set new byte to write to Slave 4.
 * This register stores the data to be written into the Slave 4. If I2C_SLV4_RW
 * is set 1 (set to read), this register has no effect.
 * @param data New byte to write to Slave 4
 * @see MPU6050_RA_I2C_SLV4_DO
 */
devIC_PR_TE void MPU6050_setSlave4OutputByte(uint8_t dataV, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_DO, dataV);
}
/** Get the enabled value for the Slave 4.
 * When set to 1, this bit enables Slave 4 for data transfer operations. When
 * cleared to 0, this bit disables Slave 4 from data transfer operations.
 * @return Current enabled value for Slave 4
 * @see MPU6050_RA_I2C_SLV4_CTRL
 */
devIC_PR_TE bool MPU6050_getSlave4Enabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set the enabled value for Slave 4.
 * @param enabled New enabled value for Slave 4
 * @see getSlave4Enabled()
 * @see MPU6050_RA_I2C_SLV4_CTRL
 */
devIC_PR_TE void MPU6050_setSlave4Enabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT, enabled );
}
/** Get the enabled value for Slave 4 transaction interrupts.
 * When set to 1, this bit enables the generation of an interrupt signal upon
 * completion of a Slave 4 transaction. When cleared to 0, this bit disables the
 * generation of an interrupt signal upon completion of a Slave 4 transaction.
 * The interrupt status can be observed in Register 54.
 *
 * @return Current enabled value for Slave 4 transaction interrupts.
 * @see MPU6050_RA_I2C_SLV4_CTRL
 */
devIC_PR_TE bool MPU6050_getSlave4InterruptEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set the enabled value for Slave 4 transaction interrupts.
 * @param enabled New enabled value for Slave 4 transaction interrupts.
 * @see getSlave4InterruptEnabled()
 * @see MPU6050_RA_I2C_SLV4_CTRL
 */
devIC_PR_TE void MPU6050_setSlave4InterruptEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, enabled);
}
/** Get write mode for Slave 4.
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @return Current write mode for Slave 4 (0 = register address + data, 1 = data only)
 * @see MPU6050_RA_I2C_SLV4_CTRL
 */
devIC_PR_TE bool MPU6050_getSlave4WriteMode( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_REG_DIS_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set write mode for the Slave 4.
 * @param mode New write mode for Slave 4 (0 = register address + data, 1 = data only)
 * @see getSlave4WriteMode()
 * @see MPU6050_RA_I2C_SLV4_CTRL
 */
devIC_PR_TE void MPU6050_setSlave4WriteMode(bool mode, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_REG_DIS_BIT, mode);
}
/** Get Slave 4 master delay value.
 * This configures the reduced access rate of I2C slaves relative to the Sample
 * Rate. When a slave's access rate is decreased relative to the Sample Rate,
 * the slave is accessed every:
 *
 *     1 / (1 + I2C_MST_DLY) samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register 25) and
 * DLPF_CFG (register 26). Whether a slave's access rate is reduced relative to
 * the Sample Rate is determined by I2C_MST_DELAY_CTRL (register 103). For
 * further information regarding the Sample Rate, please refer to register 25.
 *
 * @return Current Slave 4 master delay value
 * @see MPU6050_RA_I2C_SLV4_CTRL
 */
devIC_PR_TE uint8_t MPU6050_getSlave4MasterDelay( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT, MPU6050_I2C_SLV4_MST_DLY_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Slave 4 master delay value.
 * @param delay New Slave 4 master delay value
 * @see getSlave4MasterDelay()
 * @see MPU6050_RA_I2C_SLV4_CTRL
 */
devIC_PR_TE void MPU6050_setSlave4MasterDelay(uint8_t delay, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT, MPU6050_I2C_SLV4_MST_DLY_LENGTH, delay);
}
/** Get last available byte read from Slave 4.
 * This register stores the data read from Slave 4. This field is populated
 * after a read transaction.
 * @return Last available byte read from to Slave 4
 * @see MPU6050_RA_I2C_SLV4_DI
 */
devIC_PR_TE uint8_t MPU6050_getSlate4InputByte( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV4_DI, mpu6050->buffer);
    return mpu6050->buffer[0u];
}

// I2C_MST_STATUS register

/** Get FSYNC interrupt status.
 * This bit reflects the status of the FSYNC interrupt from an external device
 * into the MPU-60X0. This is used as a way to pass an external interrupt
 * through the MPU-60X0 to the host application processor. When set to 1, this
 * bit will cause an interrupt if FSYNC_INT_EN is asserted in INT_PIN_CFG
 * (Register 55).
 * @return FSYNC interrupt status
 * @see MPU6050_RA_I2C_MST_STATUS
 */
devIC_PR_TE bool MPU6050_getPassthroughStatus( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_PASS_THROUGH_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Slave 4 transaction done status.
 * Automatically sets to 1 when a Slave 4 transaction has completed. This
 * triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register
 * (Register 56) is asserted and if the SLV_4_DONE_INT bit is asserted in the
 * I2C_SLV4_CTRL register (Register 52).
 * @return Slave 4 transaction done status
 * @see MPU6050_RA_I2C_MST_STATUS
 */
devIC_PR_TE bool MPU6050_getSlave4IsDone( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_DONE_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get master arbitration lost status.
 * This bit automatically sets to 1 when the I2C Master has lost arbitration of
 * the auxiliary I2C bus (an error condition). This triggers an interrupt if the
 * I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Master arbitration lost status
 * @see MPU6050_RA_I2C_MST_STATUS
 */
devIC_PR_TE bool MPU6050_getLostArbitration( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_LOST_ARB_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Slave 4 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 4. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 4 NACK interrupt status
 * @see MPU6050_RA_I2C_MST_STATUS
 */
devIC_PR_TE bool MPU6050_getSlave4Nack( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_NACK_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Slave 3 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 3. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 3 NACK interrupt status
 * @see MPU6050_RA_I2C_MST_STATUS
 */
devIC_PR_TE bool MPU6050_getSlave3Nack( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV3_NACK_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Slave 2 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 2. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 2 NACK interrupt status
 * @see MPU6050_RA_I2C_MST_STATUS
 */
devIC_PR_TE bool MPU6050_getSlave2Nack( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV2_NACK_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Slave 1 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 1. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 1 NACK interrupt status
 * @see MPU6050_RA_I2C_MST_STATUS
 */
devIC_PR_TE bool MPU6050_getSlave1Nack( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV1_NACK_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Slave 0 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 0. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 0 NACK interrupt status
 * @see MPU6050_RA_I2C_MST_STATUS
 */
devIC_PR_TE bool MPU6050_getSlave0Nack( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV0_NACK_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}

// INT_PIN_CFG register

/** Get interrupt logic level mode.
 * Will be set 0 for active-high, 1 for active-low.
 * @return Current interrupt mode (0=active-high, 1=active-low)
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_LEVEL_BIT
 */
devIC_PR_TE bool MPU6050_getInterruptMode( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set interrupt logic level mode.
 * @param mode New interrupt mode (0=active-high, 1=active-low)
 * @see getInterruptMode()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_LEVEL_BIT
 */
devIC_PR_TE void MPU6050_setInterruptMode(bool mode, mpu6050_Invensense_t *mpu6050) 
{
   I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, mode);
}
/** Get interrupt drive mode.
 * Will be set 0 for push-pull, 1 for open-drain.
 * @return Current interrupt drive mode (0=push-pull, 1=open-drain)
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_OPEN_BIT
 */
devIC_PR_TE bool MPU6050_getInterruptDrive( mpu6050_Invensense_t *mpu6050 )
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set interrupt drive mode.
 * @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
 * @see getInterruptDrive()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_OPEN_BIT
 */
devIC_PR_TE void MPU6050_setInterruptDrive(bool drive, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, drive);
}
/** Get interrupt latch mode.
 * Will be set 0 for 50us-pulse, 1 for latch-until-int-cleared.
 * @return Current latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_LATCH_INT_EN_BIT
 */
devIC_PR_TE bool MPU6050_getInterruptLatch( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set interrupt latch mode.
 * @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see getInterruptLatch()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_LATCH_INT_EN_BIT
 */
devIC_PR_TE void MPU6050_setInterruptLatch(bool latch, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, latch);
}
/** Get interrupt latch clear mode.
 * Will be set 0 for status-read-only, 1 for any-register-read.
 * @return Current latch clear mode (0=status-read-only, 1=any-register-read)
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_RD_CLEAR_BIT
 */
devIC_PR_TE bool MPU6050_getInterruptLatchClear( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set interrupt latch clear mode.
 * @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
 * @see getInterruptLatchClear()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_RD_CLEAR_BIT
 */
devIC_PR_TE void MPU6050_setInterruptLatchClear(bool clear, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}
/** Get FSYNC interrupt logic level mode.
 * @return Current FSYNC interrupt mode (0=active-high, 1=active-low)
 * @see getFSyncInterruptMode()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT
 */
devIC_PR_TE bool MPU6050_getFSyncInterruptLevel( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set FSYNC interrupt logic level mode.
 * @param mode New FSYNC interrupt mode (0=active-high, 1=active-low)
 * @see getFSyncInterruptMode()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT
 */
devIC_PR_TE void MPU6050_setFSyncInterruptLevel(bool level, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, level);
}
/** Get FSYNC pin interrupt enabled setting.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled setting
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_FSYNC_INT_EN_BIT
 */
devIC_PR_TE bool MPU6050_getFSyncInterruptEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set FSYNC pin interrupt enabled setting.
 * @param enabled New FSYNC pin interrupt enabled setting
 * @see getFSyncInterruptEnabled()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_FSYNC_INT_EN_BIT
 */
devIC_PR_TE void MPU6050_setFSyncInterruptEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, enabled);
}
/** Get I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @return Current I2C bypass enabled status
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_I2C_BYPASS_EN_BIT
 */
devIC_PR_TE bool MPU6050_getI2CBypassEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @param enabled New I2C bypass enabled status
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_I2C_BYPASS_EN_BIT
 */
devIC_PR_TE void MPU6050_setI2CBypassEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/** Get reference clock output enabled status.
 * When this bit is equal to 1, a reference clock output is provided at the
 * CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
 * further information regarding CLKOUT, please refer to the MPU-60X0 Product
 * Specification document.
 * @return Current reference clock output enabled status
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_CLKOUT_EN_BIT
 */
devIC_PR_TE bool MPU6050_getClockOutputEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set reference clock output enabled status.
 * When this bit is equal to 1, a reference clock output is provided at the
 * CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
 * further information regarding CLKOUT, please refer to the MPU-60X0 Product
 * Specification document.
 * @param enabled New reference clock output enabled status
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_CLKOUT_EN_BIT
 */
devIC_PR_TE void MPU6050_setClockOutputEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, enabled);
}

// INT_ENABLE register

/** Get full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit will be
 * set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
devIC_PR_TE uint8_t MPU6050_getIntEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
devIC_PR_TE void MPU6050_setIntEnabled(uint8_t enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, enabled);
}
/** Get Free Fall interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
devIC_PR_TE bool MPU6050_getIntFreefallEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Free Fall interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
devIC_PR_TE void MPU6050_setIntFreefallEnabled(bool enabled, mpu6050_Invensense_t *mpu6050)
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT, enabled);
}
/** Get Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_MOT_BIT
 **/
devIC_PR_TE bool MPU6050_getIntMotionEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntMotionEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_MOT_BIT
 **/
devIC_PR_TE void MPU6050_setIntMotionEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, enabled);
}
/** Get Zero Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_ZMOT_BIT
 **/
devIC_PR_TE bool MPU6050_getIntZeroMotionEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Zero Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntZeroMotionEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_ZMOT_BIT
 **/
devIC_PR_TE void MPU6050_setIntZeroMotionEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, enabled);
}
/** Get FIFO Buffer Overflow interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FIFO_OFLOW_BIT
 **/
devIC_PR_TE bool MPU6050_getIntFIFOBufferOverflowEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set FIFO Buffer Overflow interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFIFOBufferOverflowEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FIFO_OFLOW_BIT
 **/
devIC_PR_TE void MPU6050_setIntFIFOBufferOverflowEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}
/** Get I2C Master interrupt enabled status.
 * This enables any of the I2C Master interrupt sources to generate an
 * interrupt. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_I2C_MST_INT_BIT
 **/
devIC_PR_TE bool MPU6050_getIntI2CMasterEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set I2C Master interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntI2CMasterEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_I2C_MST_INT_BIT
 **/
devIC_PR_TE void MPU6050_setIntI2CMasterEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, enabled);
}
/** Get Data Ready interrupt enabled setting.
 * This event occurs each time a write operation to all of the sensor registers
 * has been completed. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_DATA_RDY_BIT
 */
devIC_PR_TE bool MPU6050_getIntDataReadyEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Data Ready interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see MPU6050_RA_INT_CFG
 * @see MPU6050_INTERRUPT_DATA_RDY_BIT
 */
devIC_PR_TE void MPU6050_setIntDataReadyEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, enabled);
}

// INT_STATUS register

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
devIC_PR_TE uint8_t MPU6050_getIntStatus( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_STATUS, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Free Fall interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_FF_BIT
 */
devIC_PR_TE bool MPU6050_getIntFreefallStatus( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_FF_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Motion Detection interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_MOT_BIT
 */
devIC_PR_TE bool MPU6050_getIntMotionStatus( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_MOT_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Zero Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Zero Motion Detection interrupt has
 * been generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_ZMOT_BIT
 */
devIC_PR_TE bool MPU6050_getIntZeroMotionStatus( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_ZMOT_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get FIFO Buffer Overflow interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_FIFO_OFLOW_BIT
 */
devIC_PR_TE bool MPU6050_getIntFIFOBufferOverflowStatus( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get I2C Master interrupt status.
 * This bit automatically sets to 1 when an I2C Master interrupt has been
 * generated. For a list of I2C Master interrupts, please refer to Register 54.
 * The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_I2C_MST_INT_BIT
 */
devIC_PR_TE bool MPU6050_getIntI2CMasterStatus( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_I2C_MST_INT_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Data Ready interrupt status.
 * This bit automatically sets to 1 when a Data Ready interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_DATA_RDY_BIT
 */
devIC_PR_TE bool MPU6050_getIntDataReadyStatus( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}

// ACCEL_*OUT_* registers
/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
devIC_PR_TE void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_XOUT_H, 14, mpu6050->buffer);
    *ax = (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
    *ay = (((int16_t)mpu6050->buffer[2u]) << 8u) | mpu6050->buffer[3u];
    *az = (((int16_t)mpu6050->buffer[4u]) << 8u) | mpu6050->buffer[5u];
    *gx = (((int16_t)mpu6050->buffer[8u]) << 8u) | mpu6050->buffer[9u];
    *gy = (((int16_t)mpu6050->buffer[10u]) << 8u) | mpu6050->buffer[11u];
    *gz = (((int16_t)mpu6050->buffer[12u]) << 8u) | mpu6050->buffer[13u];
}
/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * FUNCTION NOT FULLY IMPLEMENTED YET.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @param mx 16-bit signed integer container for magnetometer X-axis value
 * @param my 16-bit signed integer container for magnetometer Y-axis value
 * @param mz 16-bit signed integer container for magnetometer Z-axis value
 * @see getMotion6()
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
devIC_PR_TE void MPU6050_getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz, mpu6050_Invensense_t *mpu6050) 
{
    MPU6050_getMotion6(ax, ay, az, gx, gy, gz, mpu6050);
    // TODO: magnetometer integration
}

/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see MPU6050_RA_GYRO_XOUT_H
 */
devIC_PR_TE void MPU6050_getAcceleration(int16_t* x, int16_t* y, int16_t* z, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_XOUT_H, 6, mpu6050->buffer);
    *x = (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
    *y = (((int16_t)mpu6050->buffer[2u]) << 8u) | mpu6050->buffer[3u];
    *z = (((int16_t)mpu6050->buffer[4u]) << 8u) | mpu6050->buffer[5u];
}
/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
devIC_PR_TE int16_t MPU6050_getAccelerationX( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_XOUT_H, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}
/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_YOUT_H
 */
devIC_PR_TE int16_t MPU6050_getAccelerationY( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_YOUT_H, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}
/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_ZOUT_H
 */
devIC_PR_TE int16_t MPU6050_getAccelerationZ( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ACCEL_ZOUT_H, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}

// TEMP_OUT_* registers

/** Get current internal temperature.
 * @return Temperature reading in 16-bit 2's complement format
 * @see MPU6050_RA_TEMP_OUT_H
 */
devIC_PR_TE int16_t MPU6050_getTemperature( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_TEMP_OUT_H, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}

// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
devIC_PR_TE void MPU6050_getRotation(int16_t* x, int16_t* y, int16_t* z, mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_GYRO_XOUT_H, 6, mpu6050->buffer);
    *x = (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
    *y = (((int16_t)mpu6050->buffer[2u]) << 8u) | mpu6050->buffer[3u];
    *z = (((int16_t)mpu6050->buffer[4u]) << 8u) | mpu6050->buffer[5u];
}
/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
devIC_PR_TE int16_t MPU6050_getRotationX( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_GYRO_XOUT_H, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}
/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_YOUT_H
 */
devIC_PR_TE int16_t MPU6050_getRotationY( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_GYRO_YOUT_H, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}
/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_ZOUT_H
 */
devIC_PR_TE int16_t MPU6050_getRotationZ( mpu6050_Invensense_t *mpu6050 )
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_GYRO_ZOUT_H, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}

// EXT_SENS_DATA_* registers

/** Read single byte from external sensor data register.
 * These registers store data read from external sensors by the Slave 0, 1, 2,
 * and 3 on the auxiliary I2C interface. Data read by Slave 4 is stored in
 * I2C_SLV4_DI (Register 53).
 *
 * External sensor data is written to these registers at the Sample Rate as
 * defined in Register 25. This access rate can be reduced by using the Slave
 * Delay Enable registers (Register 103).
 *
 * External sensor data registers, along with the gyroscope measurement
 * registers, accelerometer measurement registers, and temperature measurement
 * registers, are composed of two sets of registers: an internal register set
 * and a user-facing read register set.
 *
 * The data within the external sensors' internal register set is always updated
 * at the Sample Rate (or the reduced access rate) whenever the serial interface
 * is idle. This guarantees that a burst read of sensor registers will read
 * measurements from the same sampling instant. Note that if burst reads are not
 * used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Data is placed in these external sensor data registers according to
 * I2C_SLV0_CTRL, I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39,
 * 42, 45, and 48). When more than zero bytes are read (I2C_SLVx_LEN > 0) from
 * an enabled slave (I2C_SLVx_EN = 1), the slave is read at the Sample Rate (as
 * defined in Register 25) or delayed rate (if specified in Register 52 and
 * 103). During each Sample cycle, slave reads are performed in order of Slave
 * number. If all slaves are enabled with more than zero bytes to be read, the
 * order will be Slave 0, followed by Slave 1, Slave 2, and Slave 3.
 *
 * Each enabled slave will have EXT_SENS_DATA registers associated with it by
 * number of bytes read (I2C_SLVx_LEN) in order of slave number, starting from
 * EXT_SENS_DATA_00. Note that this means enabling or disabling a slave may
 * change the higher numbered slaves' associated registers. Furthermore, if
 * fewer total bytes are being read from the external sensors as a result of
 * such a change, then the data remaining in the registers which no longer have
 * an associated slave device (i.e. high numbered registers) will remain in
 * these previously allocated registers unless reset.
 *
 * If the sum of the read lengths of all SLVx transactions exceed the number of
 * available EXT_SENS_DATA registers, the excess bytes will be dropped. There
 * are 24 EXT_SENS_DATA registers and hence the total read lengths between all
 * the slaves cannot be greater than 24 or some bytes will be lost.
 *
 * Note: Slave 4's behavior is distinct from that of Slaves 0-3. For further
 * information regarding the characteristics of Slave 4, please refer to
 * Registers 49 to 53.
 *
 * EXAMPLE:
 * Suppose that Slave 0 is enabled with 4 bytes to be read (I2C_SLV0_EN = 1 and
 * I2C_SLV0_LEN = 4) while Slave 1 is enabled with 2 bytes to be read so that
 * I2C_SLV1_EN = 1 and I2C_SLV1_LEN = 2. In such a situation, EXT_SENS_DATA _00
 * through _03 will be associated with Slave 0, while EXT_SENS_DATA _04 and 05
 * will be associated with Slave 1. If Slave 2 is enabled as well, registers
 * starting from EXT_SENS_DATA_06 will be allocated to Slave 2.
 *
 * If Slave 2 is disabled while Slave 3 is enabled in this same situation, then
 * registers starting from EXT_SENS_DATA_06 will be allocated to Slave 3
 * instead.
 *
 * REGISTER ALLOCATION FOR DYNAMIC DISABLE VS. NORMAL DISABLE:
 * If a slave is disabled at any time, the space initially allocated to the
 * slave in the EXT_SENS_DATA register, will remain associated with that slave.
 * This is to avoid dynamic adjustment of the register allocation.
 *
 * The allocation of the EXT_SENS_DATA registers is recomputed only when (1) all
 * slaves are disabled, or (2) the I2C_MST_RST bit is set (Register 106).
 *
 * This above is also true if one of the slaves gets NACKed and stops
 * functioning.
 *
 * @param position Starting position (0-23)
 * @return Byte read from register
 */
devIC_PR_TE uint8_t MPU6050_getExternalSensorByte(int16_t position, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_EXT_SENS_DATA_00 + position, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Read word (2 bytes) from external sensor data registers.
 * @param position Starting position (0-21)
 * @return Word read from register
 * @see getExternalSensorByte()
 */
devIC_PR_TE uint16_t MPU6050_getExternalSensorWord(int16_t position, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_EXT_SENS_DATA_00 + position, 2, mpu6050->buffer);
    return (((uint16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}
/** Read double word (4 bytes) from external sensor data registers.
 * @param position Starting position (0-20)
 * @return Double word read from registers
 * @see getExternalSensorByte()
 */
devIC_PR_TE uint32_t MPU6050_getExternalSensorDWord(int16_t position, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_EXT_SENS_DATA_00 + position, 4, mpu6050->buffer);
    return (((uint32_t)mpu6050->buffer[0u]) << 24u) | (((uint32_t)mpu6050->buffer[1u]) << 16u) | (((uint16_t)mpu6050->buffer[2u]) << 8u) | mpu6050->buffer[3u];
}

// MOT_DETECT_STATUS register

/** Get X-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_XNEG_BIT
 */
devIC_PR_TE bool MPU6050_getXNegMotionDetected( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XNEG_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get X-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_XPOS_BIT
 */
devIC_PR_TE bool MPU6050_getXPosMotionDetected( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XPOS_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Y-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_YNEG_BIT
 */
devIC_PR_TE bool MPU6050_getYNegMotionDetected( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YNEG_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Y-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_YPOS_BIT
 */
devIC_PR_TE bool MPU6050_getYPosMotionDetected( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YPOS_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Z-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_ZNEG_BIT
 */
devIC_PR_TE bool MPU6050_getZNegMotionDetected( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZNEG_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get Z-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_ZPOS_BIT
 */
devIC_PR_TE bool MPU6050_getZPosMotionDetected( mpu6050_Invensense_t *mpu6050 )
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZPOS_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Get zero motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_ZRMOT_BIT
 */
devIC_PR_TE bool MPU6050_getZeroMotionDetected( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZRMOT_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}

// I2C_SLV*_DO register

/** Write byte to Data Output container for specified slave.
 * This register holds the output data written into Slave when Slave is set to
 * write mode. For further information regarding Slave control, please
 * refer to Registers 37 to 39 and immediately following.
 * @param num Slave number (0-3)
 * @param data Byte to write
 * @see MPU6050_RA_I2C_SLV0_DO
 */
devIC_PR_TE void MPU6050_setSlaveOutputByte(uint8_t num, uint8_t dataV, mpu6050_Invensense_t *mpu6050 ) 
{
    if (num > 3) return;
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_SLV0_DO + num, dataV);
}

// I2C_MST_DELAY_CTRL register

/** Get external data shadow delay enabled status.
 * This register is used to specify the timing of external sensor data
 * shadowing. When DELAY_ES_SHADOW is set to 1, shadowing of external
 * sensor data is delayed until all data has been received.
 * @return Current external data shadow delay enabled status.
 * @see MPU6050_RA_I2C_MST_DELAY_CTRL
 * @see MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT
 */
devIC_PR_TE bool MPU6050_getExternalShadowDelayEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_DELAY_CTRL, MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set external data shadow delay enabled status.
 * @param enabled New external data shadow delay enabled status.
 * @see getExternalShadowDelayEnabled()
 * @see MPU6050_RA_I2C_MST_DELAY_CTRL
 * @see MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT
 */
devIC_PR_TE void MPU6050_setExternalShadowDelayEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_DELAY_CTRL, MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}
/** Get slave delay enabled status.
 * When a particular slave delay is enabled, the rate of access for the that
 * slave device is reduced. When a slave's access rate is decreased relative to
 * the Sample Rate, the slave is accessed every:
 *
 *     1 / (1 + I2C_MST_DLY) Samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register  * 25)
 * and DLPF_CFG (register 26).
 *
 * For further information regarding I2C_MST_DLY, please refer to register 52.
 * For further information regarding the Sample Rate, please refer to register 25.
 *
 * @param num Slave number (0-4)
 * @return Current slave delay enabled status.
 * @see MPU6050_RA_I2C_MST_DELAY_CTRL
 * @see MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */
devIC_PR_TE bool MPU6050_getSlaveDelayEnabled(uint8_t num, mpu6050_Invensense_t *mpu6050) 
{
    // MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT is 4, SLV3 is 3, etc.
    if (num > 4) return 0;
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_DELAY_CTRL, num, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set slave delay enabled status.
 * @param num Slave number (0-4)
 * @param enabled New slave delay enabled status.
 * @see MPU6050_RA_I2C_MST_DELAY_CTRL
 * @see MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */
devIC_PR_TE void MPU6050_setSlaveDelayEnabled(uint8_t num, bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_I2C_MST_DELAY_CTRL, num, enabled);
}

// SIGNAL_PATH_RESET register

/** Reset gyroscope signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU6050_RA_SIGNAL_PATH_RESET
 * @see MPU6050_PATHRESET_GYRO_RESET_BIT
 */
devIC_PR_TE void MPU6050_resetGyroscopePath( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_GYRO_RESET_BIT, true);
}
/** Reset accelerometer signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU6050_RA_SIGNAL_PATH_RESET
 * @see MPU6050_PATHRESET_ACCEL_RESET_BIT
 */
devIC_PR_TE void MPU6050_resetAccelerometerPath( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_ACCEL_RESET_BIT, true);
}
/** Reset temperature sensor signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU6050_RA_SIGNAL_PATH_RESET
 * @see MPU6050_PATHRESET_TEMP_RESET_BIT
 */
devIC_PR_TE void MPU6050_resetTemperaturePath( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_TEMP_RESET_BIT, true);
}

// MOT_DETECT_CTRL register

/** Get accelerometer power-on delay.
 * The accelerometer data path provides samples to the sensor registers, Motion
 * detection, Zero Motion detection, and Free Fall detection modules. The
 * signal path contains filters which must be flushed on wake-up with new
 * samples before the detection modules begin operations. The default wake-up
 * delay, of 4ms can be lengthened by up to 3ms. This additional delay is
 * specified in ACCEL_ON_DELAY in units of 1 LSB = 1 ms. The user may select
 * any value above zero unless instructed otherwise by InvenSense. Please refer
 * to Section 8 of the MPU-6000/MPU-6050 Product Specification document for
 * further information regarding the detection modules.
 * @return Current accelerometer power-on delay
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_ACCEL_ON_DELAY_BIT
 */
devIC_PR_TE uint8_t MPU6050_getAccelerometerPowerOnDelay( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_ON_DELAY_BIT, MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set accelerometer power-on delay.
 * @param delay New accelerometer power-on delay (0-3)
 * @see getAccelerometerPowerOnDelay()
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_ACCEL_ON_DELAY_BIT
 */
devIC_PR_TE void MPU6050_setAccelerometerPowerOnDelay(uint8_t delay, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_ON_DELAY_BIT, MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}
/** Get Free Fall detection counter decrement configuration.
 * Detection is registered by the Free Fall detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring FF_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * FF_COUNT | Counter Decrement
 * ---------+------------------
 * 0        | Reset
 * 1        | 1
 * 2        | 2
 * 3        | 4
 * </pre>
 *
 * When FF_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Free Fall detection,
 * please refer to Registers 29 to 32.
 *
 * @return Current decrement configuration
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_FF_COUNT_BIT
 */
devIC_PR_TE uint8_t MPU6050_getFreefallDetectionCounterDecrement( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT, MPU6050_DETECT_FF_COUNT_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Free Fall detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getFreefallDetectionCounterDecrement()
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_FF_COUNT_BIT
 */
devIC_PR_TE void MPU6050_setFreefallDetectionCounterDecrement(uint8_t decrement, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT, MPU6050_DETECT_FF_COUNT_LENGTH, decrement);
}
/** Get Motion detection counter decrement configuration.
 * Detection is registered by the Motion detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring MOT_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * MOT_COUNT | Counter Decrement
 * ----------+------------------
 * 0         | Reset
 * 1         | 1
 * 2         | 2
 * 3         | 4
 * </pre>
 *
 * When MOT_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Motion detection,
 * please refer to Registers 29 to 32.
 *
 */
devIC_PR_TE uint8_t MPU6050_getMotionDetectionCounterDecrement( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT, MPU6050_DETECT_MOT_COUNT_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Motion detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getMotionDetectionCounterDecrement()
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_MOT_COUNT_BIT
 */
devIC_PR_TE void MPU6050_setMotionDetectionCounterDecrement(uint8_t decrement, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT, MPU6050_DETECT_MOT_COUNT_LENGTH, decrement);
}


// USER_CTRL register

/** Get FIFO enabled status.
 * When this bit is set to 0, the FIFO mpu6050.buffer is disabled. The FIFO mpu6050.buffer
 * cannot be written to or read from while disabled. The FIFO mpu6050.buffer's state
 * does not change unless the MPU-60X0 is power cycled.
 * @return Current FIFO enabled status
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
devIC_PR_TE bool MPU6050_getFIFOEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0];
}
/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
devIC_PR_TE void MPU6050_setFIFOEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}
/** Get I2C Master Mode enabled status.
 * When this mode is enabled, the MPU-60X0 acts as the I2C Master to the
 * external sensor slave devices on the auxiliary I2C bus. When this bit is
 * cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically
 * driven by the primary I2C bus (SDA and SCL). This is a precondition to
 * enabling Bypass Mode. For further information regarding Bypass Mode, please
 * refer to Register 55.
 * @return Current I2C Master Mode enabled status
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
 */
devIC_PR_TE bool MPU6050_getI2CMasterModeEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
 */
devIC_PR_TE void MPU6050_setI2CMasterModeEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}
/** Switch from I2C to SPI mode (MPU-6000 only)
 * If this is set, the primary SPI interface will be enabled in place of the
 * disabled primary I2C interface.
 */
devIC_PR_TE void MPU6050_switchSPIEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_IF_DIS_BIT, enabled);
}
/** Reset the FIFO.
 * This bit resets the FIFO mpu6050.buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
devIC_PR_TE void MPU6050_resetFIFO( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}
/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_RESET_BIT
 */
devIC_PR_TE void MPU6050_resetI2CMaster( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, true);
}
/** Reset all sensor registers and signal paths.
 * When set to 1, this bit resets the signal paths for all sensors (gyroscopes,
 * accelerometers, and temperature sensor). This operation will also clear the
 * sensor registers. This bit automatically clears to 0 after the reset has been
 * triggered.
 *
 * When resetting only the signal path (and not the sensor registers), please
 * use Register 104, SIGNAL_PATH_RESET.
 *
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_SIG_COND_RESET_BIT
 */
devIC_PR_TE void MPU6050_resetSensors( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_SIG_COND_RESET_BIT, true);
}

// PWR_MGMT_1 register

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
devIC_PR_TE void MPU6050_reset( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}
/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
devIC_PR_TE bool MPU6050_getSleepEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}

/** Get wake cycle enabled status.
 * When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
 * between sleep mode and waking up to take a single sample of data from active
 * sensors at a rate determined by LP_WAKE_CTRL (register 108).
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CYCLE_BIT
 */
devIC_PR_TE bool MPU6050_getWakeCycleEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set wake cycle enabled status.
 * @param enabled New sleep mode enabled status
 * @see getWakeCycleEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CYCLE_BIT
 */
devIC_PR_TE void MPU6050_setWakeCycleEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, enabled);
}
/** Get temperature sensor enabled status.
 * Control the usage of the internal temperature sensor.
 *
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @return Current temperature sensor enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_TEMP_DIS_BIT
 */
devIC_PR_TE bool MPU6050_getTempSensorEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u] == 0;                                            // 1 is actually disabled here
}
/** Set temperature sensor enabled status.
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @param enabled New temperature sensor enabled status
 * @see getTempSensorEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_TEMP_DIS_BIT
 */
devIC_PR_TE void MPU6050_setTempSensorEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    // 1 is actually disabled here
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, !enabled);
}
/** Get clock source setting.
 * @return Current clock source setting
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
devIC_PR_TE uint8_t MPU6050_getClockSource( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}

// PWR_MGMT_2 register

/** Get wake frequency in Accel-Only Low Power Mode.
 * The MPU-60X0 can be put into Accerlerometer Only Low Power Mode by setting
 * PWRSEL to 1 in the Power Management 1 register (Register 107). In this mode,
 * the device will power off all devices except for the primary I2C interface,
 * waking only the accelerometer at fixed intervals to take a single
 * measurement. The frequency of wake-ups can be configured with LP_WAKE_CTRL
 * as shown below:
 *
 * <pre>
 * LP_WAKE_CTRL | Wake-up Frequency
 * -------------+------------------
 * 0            | 1.25 Hz
 * 1            | 2.5 Hz
 * 2            | 5 Hz
 * 3            | 10 Hz
 * <pre>
 *
 * For further information regarding the MPU-60X0's power modes, please refer to
 * Register 107.
 *
 * @return Current wake frequency
 * @see MPU6050_RA_PWR_MGMT_2
 */
devIC_PR_TE uint8_t MPU6050_getWakeFrequency( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT, MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set wake frequency in Accel-Only Low Power Mode.
 * @param frequency New wake frequency
 * @see MPU6050_RA_PWR_MGMT_2
 */
devIC_PR_TE void MPU6050_setWakeFrequency(uint8_t frequency, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT, MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}

/** Get X-axis accelerometer standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XA_BIT
 */
devIC_PR_TE bool MPU6050_getStandbyXAccelEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set X-axis accelerometer standby enabled status.
 * @param New X-axis standby enabled status
 * @see getStandbyXAccelEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XA_BIT
 */
devIC_PR_TE void MPU6050_setStandbyXAccelEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, enabled);
}
/** Get Y-axis accelerometer standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YA_BIT
 */
devIC_PR_TE bool MPU6050_getStandbyYAccelEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Y-axis accelerometer standby enabled status.
 * @param New Y-axis standby enabled status
 * @see getStandbyYAccelEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YA_BIT
 */
devIC_PR_TE void MPU6050_setStandbyYAccelEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, enabled);
}
/** Get Z-axis accelerometer standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZA_BIT
 */
devIC_PR_TE bool MPU6050_getStandbyZAccelEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Z-axis accelerometer standby enabled status.
 * @param New Z-axis standby enabled status
 * @see getStandbyZAccelEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZA_BIT
 */
devIC_PR_TE void MPU6050_setStandbyZAccelEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, enabled);
}
/** Get X-axis gyroscope standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XG_BIT
 */
devIC_PR_TE bool MPU6050_getStandbyXGyroEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set X-axis gyroscope standby enabled status.
 * @param New X-axis standby enabled status
 * @see getStandbyXGyroEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XG_BIT
 */
devIC_PR_TE void MPU6050_setStandbyXGyroEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, enabled);
}
/** Get Y-axis gyroscope standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YG_BIT
 */
devIC_PR_TE bool MPU6050_getStandbyYGyroEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Y-axis gyroscope standby enabled status.
 * @param New Y-axis standby enabled status
 * @see getStandbyYGyroEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YG_BIT
 */
devIC_PR_TE void MPU6050_setStandbyYGyroEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, enabled);
}
/** Get Z-axis gyroscope standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZG_BIT
 */
devIC_PR_TE bool MPU6050_getStandbyZGyroEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
/** Set Z-axis gyroscope standby enabled status.
 * @param New Z-axis standby enabled status
 * @see getStandbyZGyroEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZG_BIT
 */
devIC_PR_TE void MPU6050_setStandbyZGyroEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, enabled);
}

// FIFO_COUNT* registers

/** Get current FIFO mpu6050.buffer size.
 * This value indicates the number of bytes stored in the FIFO mpu6050.buffer. This
 * number is in turn the number of bytes that can be read from the FIFO mpu6050.buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO mpu6050.buffer size
 */
devIC_PR_TE uint16_t MPU6050_getFIFOCount( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_COUNTH, 2, mpu6050->buffer);
    return (((uint16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}

// FIFO_R_W register

/** Get byte from FIFO mpu6050.buffer.
 * This register is used to read and write data from the FIFO mpu6050.buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO mpu6050.buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO mpu6050.buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO mpu6050.buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO mpu6050.buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO mpu6050.buffer is not read when
 * empty.
 *
 * @return Byte from FIFO mpu6050.buffer
 */
devIC_PR_TE uint8_t MPU6050_getFIFOByte( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_R_W, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_getFIFOBytes(uint8_t *dataV, uint8_t length, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_R_W, length, dataV);
}
/** Write byte to FIFO mpu6050.buffer.
 * @see getFIFOByte()
 * @see MPU6050_RA_FIFO_R_W
 */
devIC_PR_TE void MPU6050_setFIFOByte(uint8_t dataV, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_FIFO_R_W, dataV);
}

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register
devIC_PR_TE uint8_t MPU6050_getOTPBankValid( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_setOTPBankValid(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}
devIC_PR_TE int8_t MPU6050_getXGyroOffsetTC( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_setXGyroOffsetTC(int8_t offset, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// YG_OFFS_TC register
devIC_PR_TE int8_t MPU6050_getYGyroOffsetTC( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_setYGyroOffsetTC(int8_t offset, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register
devIC_PR_TE int8_t MPU6050_getZGyroOffsetTC( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_setZGyroOffsetTC(int8_t offset, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBits(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// X_FINE_GAIN register
devIC_PR_TE int8_t MPU6050_getXFineGain( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_X_FINE_GAIN, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_setXFineGain(int8_t gain, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_X_FINE_GAIN, gain);
}

// Y_FINE_GAIN register
devIC_PR_TE int8_t MPU6050_getYFineGain( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_Y_FINE_GAIN, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_setYFineGain(int8_t gain, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_Y_FINE_GAIN, gain);
}

// Z_FINE_GAIN register
devIC_PR_TE int8_t MPU6050_getZFineGain( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_Z_FINE_GAIN, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_setZFineGain(int8_t gain, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_Z_FINE_GAIN, gain);
}


// XA_OFFS_* registers
devIC_PR_TE int16_t MPU6050_getXAccelOffset( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_XA_OFFS_H, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}
devIC_PR_TE void MPU6050_setXAccelOffset(int16_t offset, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeWord(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_XA_OFFS_H, offset);
}

// YA_OFFS_* register
devIC_PR_TE int16_t MPU6050_getYAccelOffset( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_YA_OFFS_H, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}
devIC_PR_TE void MPU6050_setYAccelOffset(int16_t offset, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeWord(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_YA_OFFS_H, offset);
}

// ZA_OFFS_* register
devIC_PR_TE int16_t MPU6050_getZAccelOffset( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ZA_OFFS_H, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}
devIC_PR_TE void MPU6050_setZAccelOffset(int16_t offset, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeWord(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ZA_OFFS_H, offset);
}

// XG_OFFS_USR* registers
devIC_PR_TE int16_t MPU6050_getXGyroOffset( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_XG_OFFS_USRH, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}
devIC_PR_TE void MPU6050_setXGyroOffset(int16_t offset, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeWord(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_XG_OFFS_USRH, offset);
}

// YG_OFFS_USR* register
devIC_PR_TE int16_t MPU6050_getYGyroOffset( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_YG_OFFS_USRH, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}
devIC_PR_TE void MPU6050_setYGyroOffset(int16_t offset, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeWord(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_YG_OFFS_USRH, offset);
}

// ZG_OFFS_USR* register
devIC_PR_TE int16_t MPU6050_getZGyroOffset( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ZG_OFFS_USRH, 2, mpu6050->buffer);
    return (((int16_t)mpu6050->buffer[0u]) << 8u) | mpu6050->buffer[1u];
}
devIC_PR_TE void MPU6050_setZGyroOffset(int16_t offset, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeWord(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_ZG_OFFS_USRH, offset);
}

// INT_ENABLE register (DMP functions)

devIC_PR_TE bool MPU6050_getIntPLLReadyEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_setIntPLLReadyEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, enabled);
}
devIC_PR_TE bool MPU6050_getIntDMPEnabled( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_setIntDMPEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, enabled);
}

// DMP_INT_STATUS
devIC_PR_TE bool MPU6050_getDMPInt5Status( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_5_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE bool MPU6050_getDMPInt4Status( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_4_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE bool MPU6050_getDMPInt3Status( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_3_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE bool MPU6050_getDMPInt2Status( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_2_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE bool MPU6050_getDMPInt1Status( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_1_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE bool MPU6050_getDMPInt0Status( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_0_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
// INT_STATUS register (DMP functions)
devIC_PR_TE bool MPU6050_getIntPLLReadyStatus( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE bool MPU6050_getIntDMPStatus( mpu6050_Invensense_t *mpu6050 ) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DMP_INT_BIT, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
// USER_CTRL register (DMP functions)

devIC_PR_TE bool MPU6050_getDMPEnabled(mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_readBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, mpu6050->buffer);
    return mpu6050->buffer[0];
}
devIC_PR_TE void MPU6050_setDMPEnabled(bool enabled, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}
devIC_PR_TE void MPU6050_resetDMP(mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeBit(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);
}
// BANK_SEL register
devIC_PR_TE void MPU6050_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank, mpu6050_Invensense_t *mpu6050) 
{
    bank &= 0x1F;
    if (userBank) bank |= 0x20u;
    if (prefetchEnabled) bank |= 0x40u;
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_BANK_SEL, bank);
}
// MEM_START_ADDR register
devIC_PR_TE void MPU6050_setMemoryStartAddress(uint8_t address, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MEM_START_ADDR, address);
}
// MEM_R_W register
devIC_PR_TE uint8_t MPU6050_readMemoryByte(mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MEM_R_W, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_writeMemoryByte(uint8_t dataV, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MEM_R_W, dataV);
}
devIC_PR_TE void MPU6050_readMemoryBlock(uint8_t *dataV, uint16_t dataSize, uint8_t bank, uint8_t address, mpu6050_Invensense_t *mpu6050) 
{
    uint8_t chunkSize;
    uint16_t i;
    MPU6050_setMemoryBank(bank, false, false,mpu6050);
    MPU6050_setMemoryStartAddress(address,mpu6050);
    for (i = 0; i < dataSize;) 
    {
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;                              // determine correct chunk size according to bank position and data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;                 // make sure we don't go past the data size
        if (chunkSize > 256 - address) chunkSize = 256 - address;               // make sure this chunk doesn't go past the bank boundary (256 bytes)
        I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MEM_R_W, chunkSize, dataV + i);          // read the chunk of data as specified        
        i += chunkSize;                                                         // increase byte index by [chunkSize]
        address += chunkSize;                                                   // uint8_t automatically wraps to 0 at 256
        if (i < dataSize) 
        {                                                                       // if we aren't done, update bank (if necessary) and address
            if (address == 0) bank++;
            MPU6050_setMemoryBank(bank, false, false,mpu6050);
            MPU6050_setMemoryStartAddress(address,mpu6050);
        }
    }
}
// DMP_CFG_1 register
devIC_PR_TE uint8_t MPU6050_getDMPConfig1( mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_DMP_CFG_1, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_setDMPConfig1(uint8_t config, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_DMP_CFG_1, config);
}

// DMP_CFG_2 register
devIC_PR_TE uint8_t MPU6050_getDMPConfig2(mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_readByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_DMP_CFG_2, mpu6050->buffer);
    return mpu6050->buffer[0u];
}
devIC_PR_TE void MPU6050_setDMPConfig2(uint8_t config, mpu6050_Invensense_t *mpu6050) 
{
    I2Cdev_writeByte(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_DMP_CFG_2, config);
}
devIC_PR_TE bool MPU6050_writeMemoryBlock(const uint8_t *dataV, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem, mpu6050_Invensense_t *mpu6050) 
{
    uint8_t chunkSize;
    uint8_t *verifyBuffer;
    uint8_t *progBuffer;
    uint16_t i;
    //uint8_t j;
    MPU6050_setMemoryBank(bank, false, false, mpu6050);
    MPU6050_setMemoryStartAddress(address, mpu6050);
    if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    if (useProgMem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) 
    {
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;                              // determine correct chunk size according to bank position and data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;                 // make sure we don't go past the data size
        if (chunkSize > 256 - address) chunkSize = 256 - address;               // make sure this chunk doesn't go past the bank boundary (256 bytes)
        
//        if (useProgMem) 
//        {
           // for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(dataV + i + j);  write the chunk of data as specified
//        } else {
        progBuffer = (uint8_t *)dataV + i;                                  // write the chunk of data as specified
//        }
        I2Cdev_writeBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

        if (verify && verifyBuffer)                                             // verify data if needed
        {
            MPU6050_setMemoryBank(bank, false, false, mpu6050);
            MPU6050_setMemoryStartAddress(address, mpu6050);
            I2Cdev_readBytes(mpu6050->busdev.busdev_u.i2c.address, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) 
            {
                Free(verifyBuffer,sizeof(verifyBuffer));
                if (useProgMem) Free(progBuffer,sizeof(progBuffer));
                return false; 
            }
        }
        i += chunkSize;                                                         // increase byte index by [chunkSize]
        address += chunkSize;                                                   // uint8_t automatically wraps to 0 at 256

        if (i < dataSize)                                                       // if we aren't done, update bank (if necessary) and address
        {
            if (address == 0) bank++;
            MPU6050_setMemoryBank(bank, false, false, mpu6050);
            MPU6050_setMemoryStartAddress(address, mpu6050);
        }
    }
    if (verify) Free(verifyBuffer,sizeof(verifyBuffer));
    if (useProgMem) Free(progBuffer,sizeof(progBuffer));
    return true;
}
/** Evaluate the values from a MPU6050 self test.
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
devIC_PR_TE bool mpu6050EvaluateSelfTest(float32_t low, float32_t high, float32_t value)
{
  if ((value < low) || (value > high)) { return false; } else { return true; }
}
/** Do a MPU6050 self test.
 * @return True if self test passed, false otherwise
 */
devIC_PR_TE bool mpu6050SelfTest( mpu6050_Invensense_t *mpu6050 )
{
  const uint8_t no2discard = 20u; 
  bool testStatus = false;
  int16_t axi16, ayi16, azi16;
  int16_t gxi16, gyi16, gzi16;
  float32_t axf, ayf, azf;
  float32_t gxf, gyf, gzf;
  float32_t axfTst, ayfTst, azfTst;
  float32_t gxfTst, gyfTst, gzfTst;
  float32_t axfDiff, ayfDiff, azfDiff;
  float32_t gxfDiff, gyfDiff, gzfDiff;
  float32_t gRange, aRange;
  uint32_t scrap;

  aRange = MPU6050_getFullScaleAccelRange( mpu6050 );
  gRange = MPU6050_getFullScaleGyroRange( mpu6050 );

  if (mpu6050->state == 0u)
  {
     scrap = 0u;
     mpu6050->ticksRef = CP0_GET(CP0_COUNT);
     mpu6050->state = 1u;
  }
  if (scrap < no2discard)                                                       // First values after startup can be read as zero. Scrap a couple to be sure.
  {
    MPU6050_getMotion6(&axi16, &ayi16, &azi16, &gxi16, &gyi16, &gzi16, mpu6050);
    calculateTick2Now(&mpu6050->ticksVal,&mpu6050->ticksRef);
    if (mpu6050->ticksVal > MPU6050_SELFTST_DLY_1)  
    {
       scrap=++scrap % UINT32_MAX;
       mpu6050->ticksRef = CP0_GET(CP0_COUNT);
       return testStatus;
    }
    else
    {
       return testStatus;
    }
  }
  mpu6050->state = 2u;
  gxf = gxi16 * gRange;                                                         // First measurement
  gyf = gyi16 * gRange;
  gzf = gzi16 * gRange;
  axf = axi16 * aRange;
  ayf = ayi16 * aRange;
  azf = azi16 * aRange;

  mpu6050SetGyroXSelfTest(true,mpu6050);                                        // Enable self test
  mpu6050SetGyroYSelfTest(true,mpu6050);
  mpu6050SetGyroZSelfTest(true,mpu6050);
  MPU6050_setAccelXSelfTest(true,mpu6050);
  MPU6050_setAccelYSelfTest(true,mpu6050);
  MPU6050_setAccelZSelfTest(true,mpu6050);

  calculateTick2Now(&mpu6050->ticksVal,&mpu6050->ticksRef);                     // Wait for self test to take effect
  if ((mpu6050->ticksVal > MPU6050_SELFTST_DLY_2) && (mpu6050->state == 2u))  
  {
     MPU6050_getMotion6(&axi16, &ayi16, &azi16, &gxi16, &gyi16, &gzi16, mpu6050);       // Take second measurement
     gxfTst = gxi16 * gRange;
     gyfTst = gyi16 * gRange;
     gzfTst = gzi16 * gRange;
     axfTst = axi16 * aRange;
     ayfTst = ayi16 * aRange;
     azfTst = azi16 * aRange;

     mpu6050SetGyroXSelfTest(false,mpu6050);                                    // Disable self test
     mpu6050SetGyroYSelfTest(false,mpu6050);
     mpu6050SetGyroZSelfTest(false,mpu6050);
     MPU6050_setAccelXSelfTest(false,mpu6050);
     MPU6050_setAccelYSelfTest(false,mpu6050);
     MPU6050_setAccelZSelfTest(false,mpu6050);

     gxfDiff = gxfTst - gxf;                                                    // Calculate difference
     gyfDiff = gyfTst - gyf;
     gzfDiff = gzfTst - gzf;
     axfDiff = axfTst - axf;
     ayfDiff = ayfTst - ayf;
     azfDiff = azfTst - azf;

     // ====== Check result ========
     if (mpu6050EvaluateSelfTest(MPU6050_ST_GYRO_LOW, MPU6050_ST_GYRO_HIGH, gxfDiff) && mpu6050EvaluateSelfTest(-MPU6050_ST_GYRO_HIGH, -MPU6050_ST_GYRO_LOW, gyfDiff) &&  mpu6050EvaluateSelfTest(MPU6050_ST_GYRO_LOW, MPU6050_ST_GYRO_HIGH, gzfDiff) && mpu6050EvaluateSelfTest(MPU6050_ST_ACCEL_LOW, MPU6050_ST_ACCEL_HIGH, axfDiff) && mpu6050EvaluateSelfTest(MPU6050_ST_ACCEL_LOW, MPU6050_ST_ACCEL_HIGH, ayfDiff) && mpu6050EvaluateSelfTest(MPU6050_ST_ACCEL_LOW, MPU6050_ST_ACCEL_HIGH, azfDiff))
     {
        testStatus = true;
     }
     else
     {
        testStatus = false;
     }
     mpu6050->state = 0u;
     return testStatus; 
   }
}
/**
 * The software is provided "as is", without any warranty of any kind.
 * Feel free to edit it if needed.
 *
 * @author lobodol <grobodol@gmail.com>
 */
 
 /**
 * Configure gyro and accelerometer precision as following:
 *  - accelerometer: ±8g
 *  - gyro: ±500°/s
 *
 * @see https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *
 * This is an laternative set-up to MPU6050_initialize which is using different
 * settings and clock source, this is as recommened above and with power save (wake-up) 
 * set to enabled
 *
 */
/*-----------------------------------------------------------------------------
 *      setupMpu6050Registers():  set-up the mpu6050 as above
 *
 *  Parameters: mpu6050_Invensense_t *mpu6050
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
 devIC_PR_TE void setupMpu6050Registers( mpu6050_Invensense_t *mpu6050 )
 {
     switch (mpu6050->state)
     {
         case 0u:
         MPU6050_setClockSource(MPU6050_CLK_SEL_INT_OSC, mpu6050);              /* select the clock as internal oscillator */
         MPU6050_setWakeFrequency(MPU6050_LP_WAKE_CTRL_5, mpu6050);             /* configure wake-up @ 5hz using command MPU6050_RA_PWR_MGMT_2 */
         MPU6050_setSleepEnabled(false, mpu6050);                               /* wake device disable sleep mode */
         MPU6050_setWakeCycleEnabled(true, mpu6050);                            /* enable wake up mode using MPU6050_RA_PWR_MGMT_1 */
         MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500, mpu6050);           /* Configure the gyro's sensitivity using MPU6050_RA_GYRO_CONFIG Apply the desired configuration to the register : ±500°/s*/
         MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_8, mpu6050);           /* Configure the acceleromter's sensitivity using MPU6050_RA_ACCEL_CONFIG Apply the desired configuration to the register : ±8g MPU6050_ACCEL_FS_8 */
         MPU6050_setDLPFMode(MPU6050_DLPF_BW_42, mpu6050);                      /* Configure low pass filter using MPU6050_RA_CONFIG Set Digital Low Pass Filter about ~43Hz */
         break;
         
         default:
         break;
     }
 }
/*-----------------------------------------------------------------------------
 *      doMpu6050Reset():  reset the mpu6050
 *
 *  Parameters: mpu6050_Invensense_t *mpu6050
 *  Return:     (void)
 *----------------------------------------------------------------------------*/
devIC_PR_TE void doMpu6050Reset( mpu6050_Invensense_t *mpu6050 )
 {
     mpu6050->mode = mpu6050->state;                                            /* remember the current state */
     mpu6050->state = 100u;                                                     /* doing reset */
     switch (mpu6050->state)
     {
         case 100u:
         MPU6050_reset( mpu6050 );
         mpu6050->ticksRef = CP0_GET(CP0_COUNT);
         mpu6050->state = 101u;
         break;

         case 101u:
         calculateTick2Now(&mpu6050->ticksVal,&mpu6050->ticksRef);
         if (mpu6050->ticksVal > (0.05f * CPU_TICKS_PER_SECOND)) mpu6050->state = mpu6050->mode;
         break;
                  
         default:
         break;
     }
 }
#endif  /* end mpu6050 */
#if defined(CCS811_AIR_QUAL)
//GetStatus returns the current status of the device
devIC_PR_TE void CCS811DriverGetStatus( CCS811Status_t *ccs ) 
{
        uint8_t readByte;
        I2Cdev_readBytes(ccs->busdev.busdev_u.i2c.address, ccs811RegStatus, 1u, &readByte);
        ccs->HasError = readByte & 0x01u;
        ccs->DataReady = (readByte >> 3u) & 0x01u;
        ccs->AppValid = (readByte >> 4u) & 0x01u;
        ccs->FwMode = (readByte >> 7u) & 0x01u;
}

//GetFirmwareBootVersion returns the bootloader version
devIC_PR_TE uint16_t GetFirmwareBootVersion( CCS811Status_t *ccs ) 
{
        uint16_t readWord;
        I2Cdev_readBytes(ccs->busdev.busdev_u.i2c.address, ccs811RegFwBootVersion, 2u, ccs->buf);
        readWord = ((uint16_t)(ccs->buf[0u]) << 8u) | (uint16_t)(ccs->buf[1u]);
        return readWord;
}

//GetHardwareVersion returns the hardware version of the device in the form of 0x1X
devIC_PR_TE uint8_t GetHardwareVersion( CCS811Status_t *ccs ) 
{
        uint8_t readByte;
        I2Cdev_readBytes(ccs->busdev.busdev_u.i2c.address, ccs811RegHwVersion, 1u, &readByte);
        return readByte;
}

//GetFirmwareAppVersion returns the app code version
devIC_PR_TE uint16_t GetFirmwareAppVersion( CCS811Status_t *ccs ) 
{
        uint16_t readWord;
        I2Cdev_readBytes(ccs->busdev.busdev_u.i2c.address, ccs811RegFwAppVersion, 2u, ccs->buf);
        readWord = ((uint16_t)(ccs->buf[0u]) << 8u) | (uint16_t)(ccs->buf[1u]);
        return readWord;
}

//GetTemperature returns the device temperature in celcius.
//If you do not have an NTC resistor installed, this function should not be called
devIC_PR_TE float32_t GetTemperature( CCS811Status_t *ccs ) 
{
     float32_t vref, vrntc, rntc, ntcTemp;
     
     I2Cdev_readBytes(ccs->busdev.busdev_u.i2c.address, ccs811RegNtc, 4u, ccs->buf);
     vref = (((uint16_t)(ccs->buf[0u]) << 8u) | (uint16_t)(ccs->buf[1u]));
     vrntc = (((uint16_t)(ccs->buf[2u]) << 8u) | (uint16_t)(ccs->buf[3u]));
     rntc = ((vrntc * ccsntcResistanceValue) / vref);
     ntcTemp = (float32_t)(log((float64_t)(rntc / 10000.0f)));
     ntcTemp /= 3380.0f;
     ntcTemp += 1.0f / (25.0f + 273.15f);
     ntcTemp = 1.0f / ntcTemp;
     ntcTemp -= 273.15f;                
     return ntcTemp;
}

//GetGasData returns the data for the gas sensor.
//eco2 is returned in ppm and tvoc is returned in ppb
devIC_PR_TE void GetGasData( CCS811Status_t *ccs ) 
{
    I2Cdev_readBytes(ccs->busdev.busdev_u.i2c.address, ccs811RegAlgResultData, 4u, ccs->buf);        

    // Bit masks defined by https://ams.com/documents/20143/36005/CCS811_AN000369_2-00.pdf/25d0db9a-92b9-fa7f-362c-a7a4d1e292be#page=14
    ccs->eco2 = ((uint16_t)(ccs->buf[0u]) << 8u) | (uint16_t)(ccs->buf[1u]);
    ccs->tvoC = ((uint16_t)(ccs->buf[2u]) << 8u) | (uint16_t)(ccs->buf[3u]);
}

//ResetDevice does a software reset of the device. After this operation is done,
//the user must start the app code before the sensor can take any measurements
devIC_PR_TE void ccs811_reset( CCS811Status_t *ccs )
{
        uint8_t dataV[4u] = ccs811ResetSequence;
        I2Cdev_writeBytes( ccs->busdev.busdev_u.i2c.address, ccs811RegSwReset, 4, &dataV);
}

//startApp starts the app code in the device. This operation has to be done after a
//software reset to start taking sensor measurements.
devIC_PR_TE void ccs811_start( CCS811Status_t *ccs )
{
        I2Cdev_writeBytes( ccs->busdev.busdev_u.i2c.address, ccs811RegAppStart, 0, ccs->buf);
}
// GetMeasMode returns the measurement mode
devIC_PR_TE uint8_t * doConfigMode( CCS811Status_t ccs )
{
        uint8_t val;
        val = (ccs.intThresh << 2u) | (ccs.intDataRdy << 3u) | (ccs.driveMode << 4u) | ccs.confSamRate;
        return &val;
}
// START sampling at 10sec rate
devIC_PR_TE void startSample10Sec( CCS811Status_t *ccs )
{
        ccs->confSamRate = CCS811DriveMode10Sec;
        I2Cdev_writeBytes( ccs->busdev.busdev_u.i2c.address, ccs811RegMeasMode, 1, doConfigMode(*ccs));
}
#endif
/* ==== pmlps poor mans local posn === */
uint16_t pmlps_get_leu16(char v[], int16_t idx)
{
    uint8_t *u = (void *) v;
    return (uint16_t)((u[2u*idx+1]) << 8u) | u[2u*idx];
}

void pmlps_set_leu16(char v[], int16_t idx, uint16_t val)
{
    uint8_t *u = (void *) v;
    u[2u*idx] = val & 0xffu;
    u[2u*idx+1u] = val >> 8u;
}

#if defined(PX4_OPTICAL_FLOW_USED)
/* ======= px4 optical flow used ====== */
uint8_t PX4Flow_update( px4_optical_flow_frame_t *frame)
{
  uint8_t * dataV;        
  uint8_t complet=false;
  
  switch(frame->state)
  {
     case REQUEST_FLOW:                                                         // send 0x0 to PX4FLOW module and receive back 22 Bytes data            
     I2Cdev_readBytes(PX4FLOW_ADDRESS, 0x00u, 22u, dataV);                      // request 22 bytes from the module
     frame->ticksRef = CP0_GET(CP0_COUNT);
     frame->state=++frame->state % NUM_OF_PX_FLOW_STATES;
     break;
         
     case WAIT_REQ_FLOW:
     calculateTick2Now(&frame->ticksVal, &frame->ticksRef);
     if (frame->ticksVal >= (0.25*CPU_TICKS_PER_SECOND))
     {
        frame->state=++frame->state % NUM_OF_PX_FLOW_STATES; 
     }
     break;
         
     case GET_FLOW_DATA:                                                        // read the data
     frame->flow.frame_count       = dataV[0u] | (dataV[1u] << 8u);
     frame->flow.pixel_flow_x_sum  = dataV[2u] | (dataV[3u] << 8u);
     frame->flow.pixel_flow_y_sum  = dataV[4u] | (dataV[5u] << 8u);
     frame->flow.flow_comp_m_x     = dataV[6u] | (dataV[7u] << 8u);
     frame->flow.flow_comp_m_y     = dataV[8u] | (dataV[9u] << 8u);
     frame->flow.qual              = dataV[10u] | (dataV[11u] << 8u);
     frame->flow.gyro_x_rate       = dataV[12u] | (dataV[13u] << 8u);
     frame->flow.gyro_y_rate       = dataV[14u] | (dataV[15u] << 8u);
     frame->flow.gyro_z_rate       = dataV[16u] | (dataV[17u] << 8u);
     frame->flow.gyro_range        = dataV[18u];
     frame->flow.sonar_timestamp   = dataV[19u];
     frame->flow.ground_distance   = dataV[20u] | (dataV[21u] << 8u);
     frame->state=++frame->state % NUM_OF_PX_FLOW_STATES;
     complet = true;
     break;
     
     case WAIT_IDLE_FOR_HANDSHAKE:                                              /* wait for handshake */
     calculateTick2Now(&frame->ticksVal, &frame->ticksRef);
     if (frame->ticksVal >= (3.0*CPU_TICKS_PER_SECOND))                         /* or no handshake then collect the integration data */
     {
        frame->state=++frame->state % NUM_OF_PX_FLOW_STATES; 
     }
     break;
     
     case REQUEST_INTGL:
     I2Cdev_readBytes(PX4FLOW_ADDRESS, 0x16u, 26u, dataV);                      // request 26 bytes from the module
     frame->ticksRef = CP0_GET(CP0_COUNT);
     frame->state=++frame->state % NUM_OF_PX_FLOW_STATES;
     break;
     
     case WAIT_REQ_INTGL:
     calculateTick2Now(&frame->ticksVal, &frame->ticksRef);
     if (frame->ticksVal >= (0.25*CPU_TICKS_PER_SECOND))
     {
        frame->state=++frame->state % NUM_OF_PX_FLOW_STATES; 
     }
     break;
     
     case GET_INTGL_DATA:
     frame->iframe.frame_count_since_last_readout = dataV[0u] | (dataV[1u] << 8u);
     frame->iframe.pixel_flow_x_integral  = dataV[2u] | (dataV[3u] << 8u);
     frame->iframe.pixel_flow_y_integral  = dataV[4u] | (dataV[5u] << 8u);
     frame->iframe.gyro_x_rate_integral   = dataV[6u] | (dataV[7u] << 8u);
     frame->iframe.gyro_y_rate_integral   = dataV[8u] | (dataV[9u] << 8u);
     frame->iframe.gyro_z_rate_integral   = dataV[10u] | (dataV[11u] << 8u);
     frame->iframe.integration_timespan   = dataV[12u] | ((dataV[13u] << 8u) | ((dataV[14u] << 16u) | (dataV[15u] << 24u)));
     frame->iframe.sonar_timestamp        = dataV[16u] | ((dataV[17u] << 8u) | ((dataV[18u] << 16u) | (dataV[19u] << 24u)));
     frame->iframe.ground_distance        = dataV[20u] | (dataV[21u] << 8u);
     frame->iframe.gyro_temperature       = dataV[22u] | (dataV[23u] << 8u);
     frame->iframe.quality                = dataV[24u];
     /* dataV[25u] described unused byte at present spare */
     frame->state=++frame->state % NUM_OF_PX_FLOW_STATES;
     complet = true;
     break;
     
     default:
     frame->state = 0u;
     break;
  }

  return complet;
}
#endif /* end px4 optical flow */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif