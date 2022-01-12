#ifndef BoschSens680_h
#define BoschSens680_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//    BoschSensorBME680.h : Bosch Sensor on I2c bus for reading temperatue humidity pressure
//
//    can also be read from gimbal if connected on i2c using i2c read write containers and simpleBGC messages
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define SENSORPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define SENSORPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define SENSORPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define SENSORPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define SENSORPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define BOSCH_PR_HU extern

// Temperature mesaurement data address definitions
#define TEMP_par_t1_ub 0xE9u
#define TEMP_par_t1_lb 0xEAu
#define TEMP_par_t2_ub 0x8Au
#define TEMP_par_t2_lb 0x8Bu
#define TEMP_par_t3 0x8Cu
#define TEMP_temp_adc_1 0x24u                                                    // bits 7:4 xlsb 3:0
#define TEMP_temp_adc1_2 0x23u                                                   // 8 bit lsb 11:4
#define TEMP_temp_adc1_3 0x22u                                                   // 8 bit msb 19:12

#define BITS7_4(v) (uint8_t) (((v>>4u)&7u)                                        // to return bits 7:4 from result

// Temperature compensation calculations
#define TEMP_var1(x,y)  ((int32_t) (x>>3u)) + ((int32_t) (y<<1u))                 // x = temp_adc value, y = par_t1 value
#define TEMP_var2(x,y)  ((x * (int32_t) y) >> 11u)                               // x = var1, y = par_t2
#define TEMP_var3(x,y) (((((x>>1u) * (x>>1u)) >> 12u) * (((int32_t) y) << 4u)) >> 14u)   // x = var1, y = par_t3
#define TEMP_tfine(x,y)  (x+y)                                                  // x = var2 y = var3
#define TEMP_temp_comp(x) (((x * 5u) + 128u) >> 8u)                             // x = tfine COMPENSATED TEMPERATURE OUTPUT

// Pressure measurement data address definitions
#define PRESS_PAR_p1_ub 0x8Eu
#define PRESS_PAR_p1_lb 0x8Fu
#define PRESS_PAR_p2_ub 0x90u
#define PRESS_PAR_p2_lb 0x91u
#define PRESS_PAR_p3 0x92u
#define PRESS_PAR_p4_ub 0x94u
#define PRESS_PAR_p4_lb 0x95u
#define PRESS_PAR_p5_ub 0x96u
#define PRESS_PAR_p5_lb 0x97u
#define PRESS_PAR_p6 0x99u
#define PRESS_PAR_p7 0x98u
#define PRESS_PAR_p8_ub 0x9Cu
#define PRESS_PAR_p8_lb 0x9Du
#define PRESS_PAR_p9_ub 0x9Eu
#define PRESS_PAR_p9_lb 0x9Fu
#define PRESS_PAR_p10 0xA0u
#define PRESS_press_adc_1 0x21u                                                  // bits 7:4 xlsb 3:0
#define PRESS_press_adc_2 0x20u                                                  // 8 bit lsb 11:4
#define PRESS_press_adc_3 0x1Fu                                                  // 8 bit msb 19:12

// Pressure calculations pcom_final = compensated pressure in Pascals  Range should be == 300-1100 hPa
#define PRESS_var1(x) ((int32_t) (x>>1)) - 64000L                               // x = tfine
#define PRESS_var2(x,y) (((((x>>2) * (x>>2)) >> 11) * (int32_t) y) >> 2)        // x = var1, y = par_p6
#define PRESS_var3(x,y,z) (x + ((y * ((int32_t) z)) <<1))                       // x = var2, y = var1, z = par_p5
#define PRESS_var4(x,y) (x>>2) + (((int32_t) y) << 16)                          // x = var3, y = par_p4
#define PRESS_var5(x,y,z) (((((x>>2)*(x>>2) >> 13) * ((int32_t)y << 5)) + (((int32_t) z * x) >> 1))>>18)  // x = var1, y = par_p3 z = par_p2
#define PRESS_var6(x,y) (((32768L + x) * (int32_t)y) >> 15)                     // x = var5 y = par_p1
#define PRESS_presscomp(x,y) ((uint32_t) ((1048576UL - x) - (y >> 12UL)) *(uint32_t)3125UL)  // x = pressraw, y = var4
#define PRESS_pcom_hi(x,y) ((x / (uint32_t) y) << 1UL)                          // x = pressccomp y = var6 Run when x >= (1<<30)
#define PRESS_pcom_lo(x,y) ((x<<1) / y)                                         // x = pressccomp y = var6 Run when x < (1<<30)
#define PRESS_var7(x,y) ((((int32_t)  x) * (int32_t)((y >> 3)*(y >> 3) >> 13)) >> 12) // x = par_p9 y = pcom (either high or low)
#define PRESS_var8(x,y) x*y                                                     // x = pcom y = par_p8
#define PRESS_var9(x,y) ((((((int32_t)(x>>8)) * ((int32_t)(x>>8))) * ((int32_t)(x>>8))) * (int32_t)y) >> 13)   // x = pcom, y = par_p10
#define PRESS_pcom_final(x,y,z,a,b) ((((int32_t)x) + (y + z + a + ((int32_t)(b<<7))))>>4) // x = pcom, y = var7, z = var8, a = var9, b = par_p7

#define HPA_TO_inHG(x) (((float32_t) x)*0.0295299830714f)                       // inHG
#define HPA_TO_mmHG(x) (((float32_t) x)*0.750061561303f)                        // mmHG
#define HPA_TO_TORR(x) (((float32_t) x)*0.750061673821f)                        // Torr

// Humidity measurement data address definitions
#define HUM_PAR_p1_ub 0xE2u                                                      // bits 7:4
#define HUM_PAR_p1_lb 0xE3u
#define HUM_PAR_p2_ub 0xE0u                                                      // bits 7:4 manual says E2 but expect should be E0
#define HUM_PAR_p2_lb 0xE1u
#define HUM_PAR_p3 0xE4u
#define HUM_PAR_p4 0xE5u
#define HUM_PAR_p5 0xE6u
#define HUM_PAR_p6 0xE7u
#define HUM_PAR_p7 0xE8u
#define HUM_PAR_adc_lb 0x26u                                                     // 7:0
#define HUM_PAR_adc_ub 0x25u                                                     // 15:8

// Pressure calculations pcom_final = compensated pressure in Pascals  Range should be == 300-1100 hPa
#define HUM_var1(a,x,y,z) (a+((((float64_t)x)*16.0f) + ((((float64_t)y)/2.0f) * z)))
// x=hum_adc , y=par_h1 , z=par_h2 , a=temp_comp
#define HUM_var2(x,y,z,a,b) (x * (((((float64_t)y) / 262144.0f) * (1.0f + ((((float64_t)z) / 16384.0f) * a))) + ((((float64_t)b) / 1048576.0f) * a * a)))
// x= var1, y= par_h2, z=par_h4, a=temp_comp, b=par_h5
#define HUM_var3(x) (((float64_t) x) / 16384.0f)                                // x = par_h6
#define HUM_var4(x) (((float64_t) x) / 2097152.0f)                              // x = par_h7
#define HUM_comp(a,b,c,d)  a + ((b + (c * d)) * a * a)                          // a = var2, b = var3, c = var4, d = temp_comp

// SFR Control Registers
#define SEN_ctrl_meas 0x74u                                                      // Mode selection
#define SEN_Sleep 0u                                                             // Sleep mode
#define SEN_Forced 1u                                                            // Forced mode

#ifndef SPI_MODE                                                                // we are either i2c or SPI
#define SEN_reset 0xE0u                                                         // Reset device for i2c
#else
#define SEN_reset 0x60u                                                         // Reset device for SPI
#endif
#define SEN_reset_chr 0xB6u                                                     // send this to SFR data register to reset

#define SEN_ctrl_hum 0x72u                                                      // humidity mode oversampling control
#define SEN_ctrl_hum_skip 0                                                     // skips with output as 0x8000
#define SEN_ctrl_hum_os1 0b001                                                  // oversampling 1
#define SEN_ctrl_hum_os2 0b010                                                  // oversampling 2
#define SEN_ctrl_hum_os4 0b011                                                  // oversampling 4
#define SEN_ctrl_hum_os8 0b100                                                  // oversampling 8
#define SEN_ctrl_hum_os16 0b101                                                 // oversampling 16

#define SEN_ctrl_temp 0x74u                                                     // temperature mode oversampling control
#define SEN_ctrl_temp_skip 0                                                    // skips with output as 0x8000
#define SEN_ctrl_temp_os1 (0b001<<5)                                            // oversampling 1
#define SEN_ctrl_temp_os2 (0b010<<5)                                            // oversampling 2
#define SEN_ctrl_temp_os4 (0b011<<5)                                            // oversampling 4
#define SEN_ctrl_temp_os8 (0b100<<5)                                            // oversampling 8
#define SEN_ctrl_temp_os16 (0b101<<5)                                           // oversampling 16

#define SEN_ctrl_press 0x74u                                                    // pressure mode oversampling control
#define SEN_ctrl_press_skip 0                                                   // skips with output as 0x8000
#define SEN_ctrl_press_os1 (0b001<<2)                                           // oversampling 1
#define SEN_ctrl_press_os2 (0b010<<2)                                           // oversampling 2
#define SEN_ctrl_press_os4 (0b011<<2)                                           // oversampling 4
#define SEN_ctrl_press_os8 (0b100<<2)                                           // oversampling 8
#define SEN_ctrl_press_os16 (0b101<<2)                                          // oversampling 16

#define SEN_IIR_config 0x75u                                                    // IIR Filter control for temp and press not humidity
#define SEN_IIR_config_skip 0                                                   // skips with output as 0x8000
#define SEN_IIR_config_f1 (0b001<<2)                                            // filter 1
#define SEN_IIR_config_f3 (0b010<<2)                                            // filter 3
#define SEN_IIR_config_f7 (0b011<<2)                                            // filter 7
#define SEN_IIR_config_f15 (0b100<<2)                                           // filter 15
#define SEN_IIR_config_f31 (0b101<<2)                                           // filter 31
#define SEN_IIR_config_f63 (0b110<<2)                                           // filter 63
#define SEN_IIR_config_f127 (0b111<<2)                                          // filter 127

#define SEN_HEAT_ctrl_gas 0x70u                                                 // control state of current injected to heater
#define SEN_HEAT_OFF (1<<3)                                                     // turns heat off
#define SEN_HEAT_ON 0                                                           // turns heat on

#define SEN_meas_status 0x1Du                                                   // measurement status
#define SEN_meas_active (1<<5)                                                  // measurement is active this bit is set do not read
                                                                                // meas_active & result == 1 (transferring data do not read)
                                                                                // meas_active & result == 0 (ready to read, transfer complete)
#define MAX_WAIT_FOR_I2CREADY 200u                                              // maximum wait for the above signal to be 0 ready to read

// ====== Define Structures to contain the i2c data ============================


#if defined(D_FT900)
typedef struct SENSORPACKED {
      uint8_t par_t1_ub;                                                        // par_t1 : temperature MSB
      uint8_t par_t1_lb;                                                        // par_t1 : temperature LSB
      uint8_t par_t2_ub;                                                        // par_t2 : temperature MSB
      uint8_t par_t2_lb;                                                        // par_t2 : temperature LSB
      uint8_t par_t3_lb;                                                        // par_t3 : temperature LSB
      uint8_t temp_adc1;                                                        // first byte temp adc
      uint8_t temp_adc2;                                                        // second byte temp adc
      uint32_t temp_adc3 : 8u;                                                  // third byte temp adc
      uint32_t temp_adc : 24u;                                                   // temp adc formed as a number
      int32_t t_fine;                                                           // compensation used in temp & pressure calculation
      float32_t temp_comp;                                                      // compensated temperature in celcuis
} SEN_temp_t;                                                                  // Sensor temperature structure
#else
SENSORPACKED(
typedef struct {
      uint8_t par_t1_ub;                                                        // par_t1 : temperature MSB
      uint8_t par_t1_lb;                                                        // par_t1 : temperature LSB
      uint8_t par_t2_ub;                                                        // par_t2 : temperature MSB
      uint8_t par_t2_lb;                                                        // par_t2 : temperature LSB
      uint8_t par_t3_lb;                                                        // par_t3 : temperature LSB
      uint8_t temp_adc1;                                                        // first byte temp adc
      uint8_t temp_adc2;                                                        // second byte temp adc
      uint32_t temp_adc3 : 8u;                                                  // third byte temp adc
      uint32_t temp_adc : 24u;                                                   // temp adc formed as a number
      int32_t t_fine;                                                           // compensation used in temp & pressure calculation
      float32_t temp_comp;                                                      // compensated temperature in celcuis
}) SEN_temp_t;                                                                  // Sensor temperature structure
#endif

#if defined(D_FT900)
typedef struct SENSORPACKED {
      uint8_t par_p1_ub;                                                        // par_p1 : pressure MSB
      uint8_t par_p1_lb;                                                        // par_p1 : pressure LSB
      uint8_t par_p2_ub;                                                        // par_p2 : pressure MSB
      uint8_t par_p2_lb;                                                        // par_p2 : pressure LSB
      uint8_t par_p3_lb;                                                        // par_p3 : pressure LSB
      uint8_t par_p4_ub;                                                        // par_p4 : pressure MSB
      uint8_t par_p4_lb;                                                        // par_p4 : pressure LSB
      uint8_t par_p5_ub;                                                        // par_p5 : pressure MSB
      uint8_t par_p5_lb;                                                        // par_p5 : pressure LSB
      uint8_t par_p6_lb;                                                        // par_p6 : pressure LSB
      uint8_t par_p7_lb;                                                        // par_p7 : pressure LSB
      uint8_t par_p8_ub;                                                        // par_p8 : pressure MSB
      uint8_t par_p8_lb;                                                        // par_p8 : pressure LSB
      uint8_t par_p9_ub;                                                        // par_p9 : pressure MSB
      uint8_t par_p9_lb;                                                        // par_p9 : pressure LSB
      uint8_t par_p10_lb;                                                       // par_p10 : pressure LSB
      uint8_t press_adc1;                                                       // first byte pressure adc
      uint8_t press_adc2;                                                       // second byte pressure adc
      uint32_t press_adc3 : 8u;                                                 // third byte pressure adc
      uint32_t press_adc : 24u;                                                 // 20 bit pressure from ADC
      float32_t press_comp;                                                     // compensated pressure  range 300-1100 hPa
} SEN_press_t;
#else
SENSORPACKED(
typedef struct {
      uint8_t par_p1_ub;                                                        // par_p1 : pressure MSB
      uint8_t par_p1_lb;                                                        // par_p1 : pressure LSB
      uint8_t par_p2_ub;                                                        // par_p2 : pressure MSB
      uint8_t par_p2_lb;                                                        // par_p2 : pressure LSB
      uint8_t par_p3_lb;                                                        // par_p3 : pressure LSB
      uint8_t par_p4_ub;                                                        // par_p4 : pressure MSB
      uint8_t par_p4_lb;                                                        // par_p4 : pressure LSB
      uint8_t par_p5_ub;                                                        // par_p5 : pressure MSB
      uint8_t par_p5_lb;                                                        // par_p5 : pressure LSB
      uint8_t par_p6_lb;                                                        // par_p6 : pressure LSB
      uint8_t par_p7_lb;                                                        // par_p7 : pressure LSB
      uint8_t par_p8_ub;                                                        // par_p8 : pressure MSB
      uint8_t par_p8_lb;                                                        // par_p8 : pressure LSB
      uint8_t par_p9_ub;                                                        // par_p9 : pressure MSB
      uint8_t par_p9_lb;                                                        // par_p9 : pressure LSB
      uint8_t par_p10_lb;                                                       // par_p10 : pressure LSB
      uint8_t press_adc1;                                                       // first byte pressure adc
      uint8_t press_adc2;                                                       // second byte pressure adc
      uint32_t press_adc3 : 8u;                                                 // third byte pressure adc
      uint32_t press_adc : 24u;                                                 // 20 bit pressure from ADC
      float32_t press_comp;                                                     // compensated pressure  range 300-1100 hPa
}) SEN_press_t;                                                                 // sensor pressure structure
#endif

#if defined(D_FT900)
typedef struct SENSORPACKED {
      uint8_t hum_p1_ub;                                                        // par_h1 : humidity MSB
      uint8_t hum_p1_lb;                                                        // par_h1 : humidity LSB
      uint8_t hum_p2_ub;                                                        // par_h2 : humidity MSB
      uint8_t hum_p2_lb;                                                        // par_h2 : humidity LSB
      uint8_t hum_p3_lb;                                                        // par_h3 : humidity LSB
      uint8_t hum_p4_lb;                                                        // par_h4 : humidity LSB
      uint8_t hum_p5_lb;                                                        // par_h5 : humidity LSB
      uint8_t hum_p6_lb;                                                        // par_h6 : humidity LSB
      uint8_t hum_p7_lb;                                                        // par_h7 : humidity LSB
      uint8_t hum_adc1;                                                         // first byte hum adc
      uint8_t hum_adc2;                                                         // second byte hum adc
      uint16_t hum_adc;                                                         // 16 bit raw ADC value
      float32_t hum_comp;                                                       // compensated humidity in %
} SEN_humid_t;                                                                  // Sensor humidity structure
#else
SENSORPACKED(
typedef struct {
      uint8_t hum_p1_ub;                                                        // par_h1 : humidity MSB
      uint8_t hum_p1_lb;                                                        // par_h1 : humidity LSB
      uint8_t hum_p2_ub;                                                        // par_h2 : humidity MSB
      uint8_t hum_p2_lb;                                                        // par_h2 : humidity LSB
      uint8_t hum_p3_lb;                                                        // par_h3 : humidity LSB
      uint8_t hum_p4_lb;                                                        // par_h4 : humidity LSB
      uint8_t hum_p5_lb;                                                        // par_h5 : humidity LSB
      uint8_t hum_p6_lb;                                                        // par_h6 : humidity LSB
      uint8_t hum_p7_lb;                                                        // par_h7 : humidity LSB
      uint8_t hum_adc1;                                                         // first byte hum adc
      uint8_t hum_adc2;                                                         // second byte hum adc
      uint16_t hum_adc;                                                         // 16 bit raw ADC value
      float32_t hum_comp;                                                       // compensated humidity in %
}) SEN_humid_t;                                                                 // Sensor humidity structure
#endif

/* =================== FUNCTIONS ============================================ */
void i2c_init( uint8_t portNo );
uint8_t read_single_i2c(uint16_t deviceAddress, uint16_t dataAddress, uint8_t valRead);
int8_t read_temp_i2c(uint16_t deviceAddress, SEN_temp_t *tempRead);
int8_t read_press_i2c(uint16_t deviceAddress, SEN_press_t *pressRead, const SEN_temp_t *tempObj);
int8_t read_humid_i2c(uint16_t deviceAddress, SEN_humid_t *humidRead, const SEN_temp_t *tempObj);
uint8_t write_i2c(uint16_t deviceAddress, uint16_t dataAddress, uint8_t dat);
uint8_t InitTPHSensor(uint16_t deviceAddress);
uint8_t ResetTPHSensor(uint16_t deviceAddress);

/*-----------------------------------------------------------------------------
 *      i2c_init:  init i2c bus
 *      
 *
 *  Parameters: uint8_t portNo
 *  Return:   void
 *----------------------------------------------------------------------------*/
BOSCH_PR_HU void i2c_init( uint8_t portNo )
{
    switch (portNo)
    {
       case 1u:
       I2C1_Init(100000UL);                                                     // bus set @ 100Khz
       break;
       
       case 2u:
       I2C2_Init(100000UL);
       break;
       
       case 3u:
       I2C3_Init(100000UL);
       break;
       
       case 4u:
       I2C4_Init(100000UL);
       break;
       
       case 5u:
       I2C5_Init(100000UL);
       break;
       
       default:
       break;
    }
}

// read_i2c ( deviceAddress = 0x35 dataAddress = 0xAA valRead = value returned )
// returns 0=success 1=error
//
// This is a little more complicated - but not too much more. Before reading data from the slave device, 
// you must tell it which of its internal addresses you want to read. So a read of the slave actually starts off by writing to it. 
// This is the same as when you want to write to it: You send the start sequence, the I2C address of the slave with the R/W bit low (even address) 
// and the internal register number you want to write to. Now you send another start sequence (sometimes called a restart) and the I2C address again - 
// this time with the read bit set. You then read as many data bytes as you wish and terminate the transaction with a stop sequence.
//
/*-----------------------------------------------------------------------------
 *      read_single_i2c:  read single value on i2c bus
 *      
 *
 *  Parameters: uint16_t deviceAddress, uint16_t dataAddress, uint8_t valRead
 *  Return:   uint8_t
 *----------------------------------------------------------------------------*/
BOSCH_PR_HU uint8_t read_single_i2c(uint16_t deviceAddress, uint16_t dataAddress, uint8_t valRead)
{
  uint16_t temp;
  uint8_t i2cDevByte;                                                           // Create the header byte from Device Address + Read(1) Write(0)
  uint16_t returnCode;                                                          // returnCode from function calls
  
  returnCode=I2C1_Start();                                                      //  Issue I2C start sequence signal     MASTER START
                                                                                //  This will alert all the slave devices on the bus that a transaction is starting and they should listen in incase it is for them
  i2cDevByte = ((deviceAddress<<1u)+0u);
  if(!I2C1_Write(i2cDevByte))                                                   // send byte via I2C (device address << 1 + 0)
  {
     temp = (uint8_t) (dataAddress >> 8u);                                       // saving higher order address to temp
     if(!I2C1_Write(temp))                                                      // sending higher order address
     {
       if(I2C1_Write((uint8_t) (dataAddress & 0x0Fu)))                          // sending lower order address
       {
          I2C1_Restart();                                                       // issue I2C signal repeated start go to a read loop
       }
       else
       {
          return(0u);
       }
     }
     else
     {
        return(0u);
     }
  }
  else
  {
     return(0u);
  }
  
  i2cDevByte = (uint8_t) ((deviceAddress<<1u)+1u);                                // send byte via I2C (device address << 1 + 1)  read state
  if(!I2C1_Write(i2cDevByte))                                                   // send byte (device address + R)
  {
      //if(I2C1_Write((uint8_t) (dataAddress & 0x0F)))                           // sending lower order address
      //{
         valRead = I2C1_Read(0u);                                               // Read the data (NO acknowledge)
         I2C1_Stop();                                                           // Send the stop sequence
         return(1u);
      //}
      //else
      //{
      //   return(0);
      //}
  }
  else
  {
    return(0u);
  }

}

// read_i2c ( deviceAddress = 0x35 dataAddress = 0xAA valRead = value returned )
// returns 0=success 1=error -1=busy
//
//  Read the temperature data --------------------------------------------------
//
// This is a little more complicated - but not too much more. Before reading data from the slave device,
// you must tell it which of its internal addresses you want to read. So a read of the slave actually starts off by writing to it.
// This is the same as when you want to write to it: You send the start sequence, the I2C address of the slave with the R/W bit low (even address)
// and the internal register number you want to write to. Now you send another start sequence (sometimes called a restart) and the I2C address again -
// this time with the read bit set. You then read as many data bytes as you wish and terminate the transaction with a stop sequence.
//
/*-----------------------------------------------------------------------------
 *      read_temp_i2c:  Read the temperature data on i2c bus
 *      
 *
 *  Parameters: uint16_t deviceAddress, SEN_temp_t *tempRead
 *  Return:   int8_t
 *----------------------------------------------------------------------------*/
BOSCH_PR_HU int8_t read_temp_i2c(uint16_t deviceAddress, SEN_temp_t *tempRead)
{
  uint16_t temp=0u;
  uint8_t i2cDevByte;                                                           // Create the header byte from Device Address + Read(1) Write(0)
  int8_t cntWrite;                                                              // Countdown waiting for ready
  uint8_t cyclRead;
  int32_t var1,var2,var3;
  
  uint16_t dataAddress=TEMP_temp_adc1_3;                                        // Temperature data address for raw count data from the ADC

  for (cyclRead=1u;cyclRead<=3u;cyclRead++)                                       // The regsiters are split use 3 cycles to read them
  {
     if(!read_single_i2c(deviceAddress,SEN_meas_status,temp))                   // check the mesurement status
     {
         return(0);                                                             // cant get status of chip writing data
     }
     cntWrite= MAX_WAIT_FOR_I2CREADY;
     while ((temp & SEN_meas_active) && (cntWrite >= 0))                        // wait until timeout or status no longer transferring into read area
     {
       if(!read_single_i2c(deviceAddress,SEN_meas_status,temp))                 // check the mesurement status
       {
         return(0);                                                             // cant get status of chip writing data
       }
       cntWrite--;
     }
     if (cntWrite < 0)
     {
        return(-1);                                                              // did not get a ready signal (consider a reset)
     }
     if(!I2C1_Start())                                                          //  Issue I2C start sequence signal     MASTER START
     {
         return(0);
     }
                                                                                 //  This will alert all the slave devices on the bus that a transaction is starting and they should listen in incase it is for them
     switch (cyclRead)
     {
       case 2:
         dataAddress=TEMP_par_t2_ub;                                            // change the data address to par2 and 3 bank
         break;
       case 3:
         dataAddress=TEMP_par_t1_ub;                                            // change the data address to the par 1 bank
         break;
     }
     i2cDevByte = ((deviceAddress<<1u)+0u);
     if(!I2C1_Write(i2cDevByte))                                                // send byte via I2C (device address << 1 + 0)
     {
        temp = (uint8_t) (dataAddress >> 8u);                                    // saving higher order address to temp
        if(!I2C1_Write(temp))                                                   // sending higher order address
        {
          if(I2C1_Write((uint8_t) (dataAddress & 0x0Fu)))                        // sending lower order address
          {
             I2C1_Restart();                                                    // issue I2C signal repeated start go to a read loop
          }
          else
          {
             return(0);
          }
        }
        else
        {
           return(0);
         }
     }
     else
     {
       return(0);
     }

     i2cDevByte = (uint8_t) ((deviceAddress<<1u)+1u);                             // send byte via I2C (device address << 1 + 1)  read state
     if(!I2C1_Write(i2cDevByte))                                                // send byte (device address + R)
     {
        switch (cyclRead)
        {
           case 1u:
             tempRead->temp_adc3 = I2C1_Read(1u);                               // Read the data (acknowledge) ACK
             tempRead->temp_adc2 = I2C1_Read(1u);                               // Read the data (acknowledge) ACK
             tempRead->temp_adc1 = I2C1_Read(0u);                               // Read the data (NO acknowledge) NACK
             I2C1_Stop();                                                       // Send the stop i2c sequence
             tempRead->temp_adc = (tempRead->temp_adc3<<12) | ((tempRead->temp_adc2<<4) | tempRead->temp_adc1);  // make the 20 bit register
             break;
           case 2u:
             tempRead->par_t2_ub = I2C1_Read(1u);                               // Read the data (acknowledge) ACK
             tempRead->par_t2_lb = I2C1_Read(1u);                               // Read the data (acknowledge) ACK
             tempRead->par_t3_lb = I2C1_Read(0u);                               // Read the data (NO acknowledge) NACK
             I2C1_Stop();                                                       // Send the stop i2c sequence
             break;
           case 3u:
             tempRead->par_t1_ub = I2C1_Read(1u);                               // Read the data (acknowledge)     ACK
             tempRead->par_t1_lb = I2C1_Read(0u);                               // Read the data (No acknowledge)  NACK
             I2C1_Stop();                                                       // Send the stop i2c sequence
             var1 = TEMP_var1(tempRead->temp_adc,((tempRead->par_t1_ub<<8u)|tempRead->par_t1_lb));
             var2 = TEMP_var2(var1,((tempRead->par_t2_ub<<8u)|tempRead->par_t2_lb));
             var3 = TEMP_var3(var1,tempRead->par_t3_lb);
             tempRead->t_fine = TEMP_tfine(var2,var3);
             tempRead->temp_comp = (float32_t) TEMP_temp_comp(tempRead->t_fine);            // write the compensated calculated temperature
             return(1);                                                         // COMPLETE without error
             break;
        }
     }
     else
     {
       return(0);
     }
  }
}

// read_i2c ( deviceAddress = 0x35 dataAddress = 0xAA valRead = value returned )
// returns 0=success 1=error -1=busy
//
//  Read the pressure data --------------------------------------------------
//
// This is a little more complicated - but not too much more. Before reading data from the slave device,
// you must tell it which of its internal addresses you want to read. So a read of the slave actually starts off by writing to it.
// This is the same as when you want to write to it: You send the start sequence, the I2C address of the slave with the R/W bit low (even address)
// and the internal register number you want to write to. Now you send another start sequence (sometimes called a restart) and the I2C address again -
// this time with the read bit set. You then read as many data bytes as you wish and terminate the transaction with a stop sequence.
//
/*-----------------------------------------------------------------------------
 *      read_press_i2c:  Read the pressure data on i2c bus
 *      
 *
 *  Parameters: uint16_t deviceAddress, SEN_press_t *pressRead, const SEN_temp_t *tempObj
 *  Return:   int8_t
 *----------------------------------------------------------------------------*/
BOSCH_PR_HU int8_t read_press_i2c(uint16_t deviceAddress, SEN_press_t *pressRead, const SEN_temp_t *tempObj)
{
  uint16_t temp=0u;
  uint8_t i2cDevByte;                                                           // Create the header byte from Device Address + Read(1) Write(0)
  int8_t cntWrite;                                                              // Countdown waiting for ready
  uint8_t cyclRead;
  int32_t var1,var2,var3,var4,var5,var6,var7,var8,var9;
  int32_t presscomp;

  uint16_t dataAddress=PRESS_press_adc_3;                                       // 1st Pressure data address for raw count data from the ADC

  if ((pressRead == NULL) || (tempObj == NULL))
      return -1;
      
  for (cyclRead=1u;cyclRead<=4u;cyclRead++)                                       // The regsiters are split use 3 cycles to read them
  {
     if(!read_single_i2c(deviceAddress,SEN_meas_status,temp))                   // check the mesurement status
     {
         return(0);                                                             // cant get status of chip writing data
     }
     cntWrite= MAX_WAIT_FOR_I2CREADY;
     while ((temp & SEN_meas_active) && (cntWrite >= 0))                        // wait until timeout or status no longer transferring into read area
     {
       if(!read_single_i2c(deviceAddress,SEN_meas_status,temp))                 // check the mesurement status
       {
         return(0);                                                             // cant get status of chip writing data
       }
       cntWrite--;
     }
     if (cntWrite < 0)
     {
        return(-1);                                                              // did not get a ready signal (consider a reset)
     }
     if(!I2C1_Start())                                                          //  Issue I2C start sequence signal     MASTER START
     {
         return(0);
     }
                                                                                 //  This will alert all the slave devices on the bus that a transaction is starting and they should listen in incase it is for them
     switch (cyclRead)
     {
       case 2:
         dataAddress=PRESS_PAR_p1_ub;                                           // change the data address to start at par1 to par7
         break;
       case 3:
         dataAddress=PRESS_PAR_p8_ub;                                           // change the data address to the par 8 bank
         break;
       case 4:
         dataAddress=PRESS_PAR_p10;                                             // change the data address to par 10 bank
         break;
     }
     i2cDevByte = ((deviceAddress<<1u)+0u);
     if(!I2C1_Write(i2cDevByte))                                                // send byte via I2C (device address << 1 + 0)
     {
        temp = (uint8_t) (dataAddress >> 8u);                                    // saving higher order address to temp
        if(!I2C1_Write(temp))                                                   // sending higher order address
        {
          if(I2C1_Write((uint8_t) (dataAddress & 0x0Fu)))                        // sending lower order address
          {
             I2C1_Restart();                                                    // issue I2C signal repeated start go to a read loop
          }
          else
          {
             return(0);
          }
        }
        else
        {
           return(0);
         }
     }
     else
     {
       return(0);
     }

     i2cDevByte = (uint8_t) ((deviceAddress<<1u)+1u);                             // send byte via I2C (device address << 1 + 1)  read state
     if(!I2C1_Write(i2cDevByte))                                                // send byte (device address + R)
     {
        switch (cyclRead)
        {
           case 1u:
             pressRead->press_adc3 = I2C1_Read(1u);                             // Read the data (acknowledge) ACK
             pressRead->press_adc2 = I2C1_Read(1u);                             // Read the data (acknowledge) ACK
             pressRead->press_adc1 = I2C1_Read(0u);                             // Read the data (NO acknowledge) NACK
             I2C1_Stop();                                                       // Send the stop i2c sequence
             pressRead->press_adc = (pressRead->press_adc3<<12) | ((pressRead->press_adc2<<4) | pressRead->press_adc1);  // make the 20 bit register
             break;
           case 2u:
             pressRead->par_p1_ub = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             pressRead->par_p1_lb = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             pressRead->par_p2_ub = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             pressRead->par_p2_lb = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             pressRead->par_p3_lb = I2C1_Read(1u);                                 // Read the data (acknowledge) ACK
             pressRead->par_p4_ub = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             pressRead->par_p4_lb = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             pressRead->par_p5_ub = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             pressRead->par_p5_lb = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             pressRead->par_p6_lb = I2C1_Read(1u);                                 // Read the data (acknowledge) ACK
             pressRead->par_p7_lb = I2C1_Read(0u);                                 // Read the data (Not acknowledge) NACK
             I2C1_Stop();                                                       // Send the stop i2c sequence
             break;
           case 3u:
             pressRead->par_p8_ub = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             pressRead->par_p8_lb = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             pressRead->par_p9_ub = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             pressRead->par_p9_lb = I2C1_Read(0u);                              // Read the data (NO acknowledge) NACK
             I2C1_Stop();                                                       // Send the stop i2c sequence
             break;
           case 4u:
             pressRead->par_p10_lb = I2C1_Read(0u);                              // Read the data (NO acknowledge) NACK
             var1 = PRESS_var1(tempObj->t_fine);
             var2 = PRESS_var2(var1,pressRead->par_p6_lb);
             var3 = PRESS_var3(var2,var1, ((pressRead->par_p5_ub << 8u) | pressRead->par_p5_lb));
             var4 = PRESS_var4(var3, ((pressRead->par_p4_ub << 8u) | pressRead->par_p4_lb));
             var5 = PRESS_var5(var1,((pressRead->par_p2_ub << 8u) | pressRead->par_p2_lb),pressRead->par_p3_lb);
             var6 = PRESS_var6(var5,((pressRead->par_p1_ub << 8u) | pressRead->par_p1_lb));
             presscomp = PRESS_presscomp(pressRead->press_adc,var4);
             if (presscomp >= (1u<<30u))
             {
                presscomp = PRESS_pcom_hi(presscomp,var6);
             }
             else
             {
                presscomp = PRESS_pcom_lo(presscomp,var6);
             }
             var7 = PRESS_var7(((pressRead->par_p9_ub << 8u) | pressRead->par_p9_lb),presscomp);
             var8 = PRESS_var8(presscomp,((pressRead->par_p8_ub << 8u) | pressRead->par_p8_lb));
             var9 = PRESS_var9(presscomp,pressRead->par_p10_lb);
             pressRead->press_comp = (float32_t) PRESS_pcom_final(presscomp,var7,var8,var6,pressRead->par_p7_lb);   // compensated pressure reading hPa
             return(1);                                                         // COMPLETE without error
             break;
        }
     }
     else
     {
       return(0);
     }
  }
}

// read_i2c ( deviceAddress = 0x35 dataAddress = 0xAA valRead = value returned )
// returns 0=success 1=error -1=busy
//
//  Read the humidity data --------------------------------------------------
//
// This is a little more complicated - but not too much more. Before reading data from the slave device,
// you must tell it which of its internal addresses you want to read. So a read of the slave actually starts off by writing to it.
// This is the same as when you want to write to it: You send the start sequence, the I2C address of the slave with the R/W bit low (even address)
// and the internal register number you want to write to. Now you send another start sequence (sometimes called a restart) and the I2C address again -
// this time with the read bit set. You then read as many data bytes as you wish and terminate the transaction with a stop sequence.
//
/*-----------------------------------------------------------------------------
 *      read_humid_i2c:  Read the humidity data on i2c bus
 *      
 *
 *  Parameters: uint16_t deviceAddress, SEN_humid_t *humidRead, const SEN_temp_t *tempObj
 *  Return:   int8_t
 *----------------------------------------------------------------------------*/
BOSCH_PR_HU int8_t read_humid_i2c(uint16_t deviceAddress, SEN_humid_t *humidRead, const SEN_temp_t *tempObj)
{
  uint16_t temp=0u;
  uint8_t i2cDevByte;                                                           // Create the header byte from Device Address + Read(1) Write(0)
  int8_t cntWrite;                                                              // Countdown waiting for ready
  uint8_t cyclRead;
  int32_t var1,var2,var3,var4;

  uint16_t dataAddress=HUM_PAR_adc_ub;                                          // 1st humidity data address for raw count data from the ADC

  if ((tempObj == NULL) || (humidRead == NULL))
     return -1;
     
  for (cyclRead=1u;cyclRead<=2u;cyclRead++)                                       // The regsiters are split use 3 cycles to read them
  {
     if(!read_single_i2c(deviceAddress,SEN_meas_status,temp))                   // check the mesurement status
     {
         return(0);                                                             // cant get status of chip writing data
     }
     cntWrite= MAX_WAIT_FOR_I2CREADY;
     while ((temp & SEN_meas_active) && (cntWrite >= 0))                        // wait until timeout or status no longer transferring into read area
     {
       if(!read_single_i2c(deviceAddress,SEN_meas_status,temp))                 // check the mesurement status
       {
         return(0);                                                             // cant get status of chip writing data
       }
       cntWrite--;
     }
     if (cntWrite < 0)
     {
        return(-1);                                                              // did not get a ready signal (consider a reset)
     }
     if(!I2C1_Start())                                                          //  Issue I2C start sequence signal     MASTER START
     {
         return(0);
     }
                                                                                 //  This will alert all the slave devices on the bus that a transaction is starting and they should listen in incase it is for them
     switch (cyclRead)
     {
       case 2:
       dataAddress=HUM_PAR_p2_ub;                                             // change the data address to start at par1 to end pars
       break;
       
       default:
       break;
     }
     i2cDevByte = ((deviceAddress<<1u)+0u);
     if(!I2C1_Write(i2cDevByte))                                                // send byte via I2C (device address << 1 + 0)
     {
        temp = (uint8_t) (dataAddress >> 8u);                                   // saving higher order address to temp
        if(!I2C1_Write(temp))                                                   // sending higher order address
        {
          if(I2C1_Write((uint8_t) (dataAddress & 0x0Fu)))                       // sending lower order address
          {
             I2C1_Restart();                                                    // issue I2C signal repeated start go to a read loop
          }
          else
          {
             return(0);
          }
        }
        else
        {
           return(0);
         }
     }
     else
     {
       return(0);
     }

     i2cDevByte = (uint8_t) ((deviceAddress<<1u)+1u);                           // send byte via I2C (device address << 1 + 1)  read state
     if(!I2C1_Write(i2cDevByte))                                                // send byte (device address + R)
     {
        switch (cyclRead)
        {
           case 1u:
             humidRead->hum_adc2 = I2C1_Read(1u);                             // Read the data (acknowledge) ACK
             humidRead->hum_adc1 = I2C1_Read(0u);                             // Read the data (NO acknowledge) NACK
             I2C1_Stop();                                                       // Send the stop i2c sequence
             humidRead->hum_adc = ((humidRead->hum_adc2<<8) | humidRead->hum_adc1);  // make the 16 bit register
             break;
           case 2u:
             humidRead->hum_p2_ub = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             humidRead->hum_p2_lb = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             humidRead->hum_p1_ub = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             humidRead->hum_p1_lb = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             humidRead->hum_p3_lb = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             humidRead->hum_p4_lb = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             humidRead->hum_p5_lb = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             humidRead->hum_p6_lb = I2C1_Read(1u);                              // Read the data (acknowledge) ACK
             humidRead->hum_p7_lb = I2C1_Read(0u);                              // Read the data (No acknowledge) NACK
             I2C1_Stop();                                                       // Send the stop i2c sequence
             var1 = HUM_var1(humidRead->hum_adc,((humidRead->hum_p1_ub<<8u)|humidRead->hum_p1_lb),((humidRead->hum_p2_ub<<8u)|humidRead->hum_p2_lb),tempObj->temp_comp);
             var2 = HUM_var2(((humidRead->hum_p1_ub<<8u)|humidRead->hum_p1_lb),((humidRead->hum_p2_ub<<8u)|humidRead->hum_p2_lb),humidRead->hum_p4_lb,tempObj->temp_comp,humidRead->hum_p5_lb);
             var3 = HUM_var3(humidRead->hum_p6_lb);
             var4 = HUM_var4(humidRead->hum_p7_lb);
             humidRead->hum_comp = HUM_comp(((humidRead->hum_p2_ub<<8u)|humidRead->hum_p2_lb),humidRead->hum_p3_lb,humidRead->hum_p4_lb,tempObj->temp_comp);
             return(1);
             break;
        }
     }
     else
     {
       return(0);
     }
  }
}

//
// write_i2c ( deviceAddress = 0x35 dataAddress = 0xAA data = 23145 )
//
/*-----------------------------------------------------------------------------
 *      write_i2c:  write data on i2c bus
 *      
 *
 *  Parameters: uint16_t deviceAddress, uint16_t dataAddress, uint8_t dat
 *  Return:   uint8_t
 *----------------------------------------------------------------------------*/
BOSCH_PR_HU uint8_t write_i2c(uint16_t deviceAddress, uint16_t dataAddress, uint8_t dat)
{
  uint8_t temp;
  uint8_t i2cDevByte;                                                           // Create the header byte from Device Address + Read(1) Write(0)
  
  if(!I2C1_Start())                                                             // Send the start sequence
  {                                                                             // issue I2C start signal
    i2cDevByte = (uint8_t) ((deviceAddress<<1u)+0u);                            // device address with 0 RW=0
    if(!I2C1_Write(i2cDevByte))                                                  // send byte via I2C (device address <<1 + 0 )  Send the I2C address of the slave with the R/W bit low (even address)
    {
       temp = (uint8_t) (dataAddress >> 8u);                                     // saving higher order address to temp
       if(!I2C1_Write(temp))                                                    // sending higher order address Send the internal register number you want to write to
       {
         if(!I2C1_Write((uint8_t)(dataAddress & 0x0Fu)))                         // sending lower order address
         {
           if(!I2C1_Write(dat))                                                 // send data (data to be written)   Send the data byte
           {
            I2C1_Stop();                                                        // issue I2C stop signal
            return(1u);                                                         // success
           }
         }
         else
         {
           return(0u);                                                           // error
         }
      }
      else
      {
        return(0u);                                                              // error
      }
    }
    else
    {
       return(0u);                                                               // error
    }
  }
  else
  {
     return(0u);                                                                 // error
  }
}

/*-----------------------------------------------------------------------------
 *      InitTPHSensor:  Initialise the Sensor for IIR Filters and oversampling
 *      
 *
 *  Parameters: uint16_t deviceAddress
 *  Return:   uint8_t
 *----------------------------------------------------------------------------*/
BOSCH_PR_HU uint8_t InitTPHSensor(uint16_t deviceAddress)
{
   if (write_i2c(deviceAddress, SEN_IIR_config, SEN_IIR_config_f3))             // IIR config
   {
      if (write_i2c(deviceAddress, SEN_ctrl_press, SEN_ctrl_press_os2))         // oversampling for pressure
      {
         if (write_i2c(deviceAddress, SEN_ctrl_temp, SEN_ctrl_temp_os2))        // oversampling for temperature
         {
            if (write_i2c(deviceAddress, SEN_ctrl_hum, SEN_ctrl_hum_os2))       // oversampling for humidity
            {
               return(1u);                                                      // success
            }
         }
      }
   }
   return(0u);                                                                  // return failure
}

/*-----------------------------------------------------------------------------
 *      ResetTPHSensor:  Reset the TPH SEnsor
 *      
 *
 *  Parameters: uint16_t deviceAddress
 *  Return:   uint8_t
 *----------------------------------------------------------------------------*/
BOSCH_PR_HU uint8_t ResetTPHSensor(uint16_t deviceAddress)
{
  return(write_i2c(deviceAddress, SEN_reset, SEN_reset_chr));                   // send a reset word to the reset sFr (special function register)
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif