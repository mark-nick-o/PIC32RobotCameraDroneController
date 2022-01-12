#ifndef __mpu6050_acc_gyro_
#define __mpu6050_acc_gyro_
/*    Version : @(#) 1.0
      Copyright (C) 2020 A C P Avaiation Walkerburn Scotland                 */
/* The MPU-6050 is the world’s first integrated 6-axis MotionTracking device */
/* i2c for reading gyro data from MPU6050 invensense                         */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MPU6050_ADDR = 0x68u
#define ACCEL_XOUT_H 0x3Bu                                                      /* X_H which is MSB */
#define ACCEL_XOUT_L 0x3Cu                                                      /* X_L which is LSB */
#define ACCEL_YOUT_H 0x3Du                                                      /* Y_H which is MSB */
#define ACCEL_YOUT_L 0x3Eu                                                      /* Y_L which is LSB */
#define ACCEL_ZOUT_H 0x3Fu                                                      /* Z_H which is MSB */
#define ACCEL_ZOUT_L 0x40u                                                      /* Z_L which is LSB */
#define TEMP_OUT_H 0x41u                                                        /* T_H which is MSB */
#define TEMP_OUT_L 0x42u                                                        /* T_L which is LSB */

#define Axyz_val(MSB,LSB)  (((uint16_t)MSB) << 8u) | LSB                        /* make the readings into a uint16_t */

#define WA_Rest(Racc,w1,Rgyro,w2) = ((Racc * w1) + (Rgyro * w2 )) / (w1 + w2)   /* weighted average calc using ACCELEROMETER AND GYRO */

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Copyright (c) 2014 Marton Sebok */

#define MPU6050_ADDRESS_AD0_LOW  0x68u                                          // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH 0x69u                                          // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_XG_OFFS_TC 0x00u                                             //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC 0x01u                                             //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC 0x02u                                             //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN 0x03u                                            //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN 0x04u                                            //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN 0x05u                                            //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H 0x06u                                              //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC 0x07u
#define MPU6050_RA_YA_OFFS_H 0x08u                                              //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC 0x09u
#define MPU6050_RA_ZA_OFFS_H 0x0Au                                              //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC 0x0Bu
#define MPU6050_RA_XG_OFFS_USRH 0x13u                                           //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL 0x14u
#define MPU6050_RA_YG_OFFS_USRH 0x15u                                           //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL 0x16u
#define MPU6050_RA_ZG_OFFS_USRH 0x17u                                           //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL 0x18u
#define MPU6050_RA_SMPLRT_DIV 0x19u
#define MPU6050_RA_CONFIG  0x1Au
#define MPU6050_RA_GYRO_CONFIG  0x1Bu
#define MPU6050_RA_ACCEL_CONFIG 0x1Cu
#define MPU6050_RA_FF_THR 0x1Du
#define MPU6050_RA_FF_DUR 0x1Eu
#define MPU6050_RA_MOT_THR 0x1Fu
#define MPU6050_RA_MOT_DUR 0x20u
#define MPU6050_RA_ZRMOT_THR 0x21u
#define MPU6050_RA_ZRMOT_DUR 0x22u
#define MPU6050_RA_FIFO_EN 0x23u
#define MPU6050_RA_I2C_MST_CTRL 0x24u
#define MPU6050_RA_I2C_SLV0_ADDR 0x25u
#define MPU6050_RA_I2C_SLV0_REG  0x26u
#define MPU6050_RA_I2C_SLV0_CTRL 0x27u
#define MPU6050_RA_I2C_SLV1_ADDR 0x28u
#define MPU6050_RA_I2C_SLV1_REG  0x29u
#define MPU6050_RA_I2C_SLV1_CTRL 0x2Au
#define MPU6050_RA_I2C_SLV2_ADDR 0x2Bu
#define MPU6050_RA_I2C_SLV2_REG 0x2Cu
#define MPU6050_RA_I2C_SLV2_CTRL 0x2Du
#define MPU6050_RA_I2C_SLV3_ADDR 0x2Eu
#define MPU6050_RA_I2C_SLV3_REG 0x2Fu
#define MPU6050_RA_I2C_SLV3_CTRL 0x30u
#define MPU6050_RA_I2C_SLV4_ADDR 0x31u
#define MPU6050_RA_I2C_SLV4_REG 0x32u
#define MPU6050_RA_I2C_SLV4_DO 0x33u
#define MPU6050_RA_I2C_SLV4_CTRL 0x34u
#define MPU6050_RA_I2C_SLV4_DI 0x35u
#define MPU6050_RA_I2C_MST_STATUS 0x36u
#define MPU6050_RA_INT_PIN_CFG 0x37u
#define MPU6050_RA_INT_ENABLE 0x38u
#define MPU6050_RA_DMP_INT_STATUS 0x39u
#define MPU6050_RA_INT_STATUS 0x3Au
#define MPU6050_RA_ACCEL_XOUT_H 0x3Bu
#define MPU6050_RA_ACCEL_XOUT_L 0x3Cu
#define MPU6050_RA_ACCEL_YOUT_H 0x3Du
#define MPU6050_RA_ACCEL_YOUT_L 0x3Eu
#define MPU6050_RA_ACCEL_ZOUT_H 0x3Fu
#define MPU6050_RA_ACCEL_ZOUT_L 0x40u
#define MPU6050_RA_TEMP_OUT_H 0x41u
#define MPU6050_RA_TEMP_OUT_L  0x42u
#define MPU6050_RA_GYRO_XOUT_H 0x43u
#define MPU6050_RA_GYRO_XOUT_L 0x44u
#define MPU6050_RA_GYRO_YOUT_H 0x45u
#define MPU6050_RA_GYRO_YOUT_L 0x46u
#define MPU6050_RA_GYRO_ZOUT_H 0x47u
#define MPU6050_RA_GYRO_ZOUT_L 0x48u
#define MPU6050_RA_EXT_SENS_DATA_00 0x49u
#define MPU6050_RA_EXT_SENS_DATA_01 0x4Au
#define MPU6050_RA_EXT_SENS_DATA_02 0x4Bu
#define MPU6050_RA_EXT_SENS_DATA_03 0x4Cu
#define MPU6050_RA_EXT_SENS_DATA_04 0x4Du
#define MPU6050_RA_EXT_SENS_DATA_05 0x4Eu
#define MPU6050_RA_EXT_SENS_DATA_06 0x4Fu
#define MPU6050_RA_EXT_SENS_DATA_07 0x50u
#define MPU6050_RA_EXT_SENS_DATA_08 0x51u
#define MPU6050_RA_EXT_SENS_DATA_09 0x52u
#define MPU6050_RA_EXT_SENS_DATA_10 0x53u
#define MPU6050_RA_EXT_SENS_DATA_11 0x54u
#define MPU6050_RA_EXT_SENS_DATA_12 0x55u
#define MPU6050_RA_EXT_SENS_DATA_13 0x56u
#define MPU6050_RA_EXT_SENS_DATA_14 0x57u
#define MPU6050_RA_EXT_SENS_DATA_15 0x58u
#define MPU6050_RA_EXT_SENS_DATA_16 0x59u
#define MPU6050_RA_EXT_SENS_DATA_17 0x5Au
#define MPU6050_RA_EXT_SENS_DATA_18 0x5Bu
#define MPU6050_RA_EXT_SENS_DATA_19 0x5Cu
#define MPU6050_RA_EXT_SENS_DATA_20 0x5Du
#define MPU6050_RA_EXT_SENS_DATA_21 0x5Eu
#define MPU6050_RA_EXT_SENS_DATA_22 0x5Fu
#define MPU6050_RA_EXT_SENS_DATA_23 0x60u
#define MPU6050_RA_MOT_DETECT_STATUS 0x61u
#define MPU6050_RA_I2C_SLV0_DO 0x63u
#define MPU6050_RA_I2C_SLV1_DO 0x64u
#define MPU6050_RA_I2C_SLV2_DO 0x65u
#define MPU6050_RA_I2C_SLV3_DO 0x66u
#define MPU6050_RA_I2C_MST_DELAY_CTRL 0x67u
#define MPU6050_RA_SIGNAL_PATH_RESET 0x68u
#define MPU6050_RA_MOT_DETECT_CTRL 0x69u
#define MPU6050_RA_USER_CTRL 0x6Au
#define MPU6050_RA_PWR_MGMT_1 0x6Bu
#define MPU6050_RA_PWR_MGMT_2 0x6Cu
#define MPU6050_RA_BANK_SEL 0x6Du
#define MPU6050_RA_MEM_START_ADDR 0x6Eu
#define MPU6050_RA_MEM_R_W 0x6Fu
#define MPU6050_RA_DMP_CFG_1 0x70u
#define MPU6050_RA_DMP_CFG_2 0x71u
#define MPU6050_RA_FIFO_COUNTH 0x72u
#define MPU6050_RA_FIFO_COUNTL 0x73u
#define MPU6050_RA_FIFO_R_W 0x74u
#define MPU6050_RA_WHO_AM_I 0x75u

#define MPU6050_TC_PWR_MODE_BIT 7u
#define MPU6050_TC_OFFSET_BIT 6u
#define MPU6050_TC_OFFSET_LENGTH 6u
#define MPU6050_TC_OTP_BNK_VLD_BIT 0u

#define MPU6050_VDDIO_LEVEL_VLOGIC 0u
#define MPU6050_VDDIO_LEVEL_VDD 1u

#define MPU6050_CFG_EXT_SYNC_SET_BIT 5u
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3u
#define MPU6050_CFG_DLPF_CFG_BIT 2u                                          
#define MPU6050_CFG_DLPF_CFG_LENGTH 3u

#define MPU6050_EXT_SYNC_DISABLED 0x0u
#define MPU6050_EXT_SYNC_TEMP_OUT_L 0x1u
#define MPU6050_EXT_SYNC_GYRO_XOUT_L 0x2u
#define MPU6050_EXT_SYNC_GYRO_YOUT_L 0x3u
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L 0x4u
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L 0x5u
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L 0x6u
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L 0x7u

#define MPU6050_DLPF_BW_256 0x00u                                               // Set Digital Low Pass Filter about ~256Hz
#define MPU6050_DLPF_BW_188 0x01u                                               // Set Digital Low Pass Filter about ~188Hz
#define MPU6050_DLPF_BW_98 0x02u                                                // Set Digital Low Pass Filter about ~98Hz
#define MPU6050_DLPF_BW_42 0x03u                                                // Set Digital Low Pass Filter about ~43Hz
#define MPU6050_DLPF_BW_20 0x04u                                                // Set Digital Low Pass Filter about ~20Hz
#define MPU6050_DLPF_BW_10 0x05u                                                // Set Digital Low Pass Filter about ~10Hz
#define MPU6050_DLPF_BW_5 0x06u                                                 // Set Digital Low Pass Filter about ~5Hz

#define MPU6050_GCONFIG_XG_ST_BIT 7u
#define MPU6050_GCONFIG_YG_ST_BIT 6u
#define MPU6050_GCONFIG_ZG_ST_BIT 5u
#define MPU6050_GCONFIG_FS_SEL_BIT 4u
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2u

#define MPU6050_SELFTST_DLY_1 200u
#define MPU6050_SELFTST_DLY_2 20000u

#define MPU6050_ST_GYRO_LOW 10.0f                                               // deg/s
#define MPU6050_ST_GYRO_HIGH 105.0f                                             // deg/s
#define MPU6050_ST_ACCEL_LOW 0.300f                                             // G
#define MPU6050_ST_ACCEL_HIGH 0.950f                                            // G

#define MPU6050_GYRO_FS_250 0x00u
#define MPU6050_GYRO_FS_500 0x01u
#define MPU6050_GYRO_FS_1000 0x02u
#define MPU6050_GYRO_FS_2000 0x03u

#define MPU6050_ACONFIG_XA_ST_BIT 7u
#define MPU6050_ACONFIG_YA_ST_BIT 6u
#define MPU6050_ACONFIG_ZA_ST_BIT 5u
#define MPU6050_ACONFIG_AFS_SEL_BIT 4u
#define MPU6050_ACONFIG_AFS_SEL_LENGTH 2u
#define MPU6050_ACONFIG_ACCEL_HPF_BIT 2u
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH 3u

#define MPU6050_ACCEL_FS_2  0x00u
#define MPU6050_ACCEL_FS_4  0x01u
#define MPU6050_ACCEL_FS_8  0x02u
#define MPU6050_ACCEL_FS_16 0x03u

#define MPU6050_DHPF_RESET 0x00u
#define MPU6050_DHPF_5 0x01u
#define MPU6050_DHPF_2P5 0x02u
#define MPU6050_DHPF_1P25 0x03u
#define MPU6050_DHPF_0P63 0x04u
#define MPU6050_DHPF_HOLD 0x07u

#define MPU6050_TEMP_FIFO_EN_BIT 7u
#define MPU6050_XG_FIFO_EN_BIT 6u
#define MPU6050_YG_FIFO_EN_BIT 5u
#define MPU6050_ZG_FIFO_EN_BIT 4u
#define MPU6050_ACCEL_FIFO_EN_BIT 3u
#define MPU6050_SLV2_FIFO_EN_BIT 2u
#define MPU6050_SLV1_FIFO_EN_BIT 1u
#define MPU6050_SLV0_FIFO_EN_BIT 0u

#define MPU6050_MULT_MST_EN_BIT 7u
#define MPU6050_WAIT_FOR_ES_BIT 6u
#define MPU6050_SLV_3_FIFO_EN_BIT 5u
#define MPU6050_I2C_MST_P_NSR_BIT 4u
#define MPU6050_I2C_MST_CLK_BIT 3u
#define MPU6050_I2C_MST_CLK_LENGTH  4u

#define MPU6050_CLOCK_DIV_348 0x0u
#define MPU6050_CLOCK_DIV_333 0x1u
#define MPU6050_CLOCK_DIV_320 0x2u
#define MPU6050_CLOCK_DIV_308 0x3u
#define MPU6050_CLOCK_DIV_296 0x4u
#define MPU6050_CLOCK_DIV_286 0x5u
#define MPU6050_CLOCK_DIV_276 0x6u
#define MPU6050_CLOCK_DIV_267 0x7u
#define MPU6050_CLOCK_DIV_258 0x8u
#define MPU6050_CLOCK_DIV_500 0x9u
#define MPU6050_CLOCK_DIV_471 0xAu
#define MPU6050_CLOCK_DIV_444 0xBu
#define MPU6050_CLOCK_DIV_421 0xCu
#define MPU6050_CLOCK_DIV_400 0xDu
#define MPU6050_CLOCK_DIV_381 0xEu
#define MPU6050_CLOCK_DIV_364 0xFu

#define MPU6050_I2C_SLV_RW_BIT      7u
#define MPU6050_I2C_SLV_ADDR_BIT    6u
#define MPU6050_I2C_SLV_ADDR_LENGTH 7u
#define MPU6050_I2C_SLV_EN_BIT      7u
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6u
#define MPU6050_I2C_SLV_REG_DIS_BIT 5u
#define MPU6050_I2C_SLV_GRP_BIT     4u
#define MPU6050_I2C_SLV_LEN_BIT     3u
#define MPU6050_I2C_SLV_LEN_LENGTH  4u

#define MPU6050_I2C_SLV4_RW_BIT 7u
#define MPU6050_I2C_SLV4_ADDR_BIT 6u
#define MPU6050_I2C_SLV4_ADDR_LENGTH 7u
#define MPU6050_I2C_SLV4_EN_BIT 7u
#define MPU6050_I2C_SLV4_INT_EN_BIT  6u
#define MPU6050_I2C_SLV4_REG_DIS_BIT 5u
#define MPU6050_I2C_SLV4_MST_DLY_BIT 4u
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5u

#define MPU6050_MST_PASS_THROUGH_BIT 7u
#define MPU6050_MST_I2C_SLV4_DONE_BIT 6u
#define MPU6050_MST_I2C_LOST_ARB_BIT 5u
#define MPU6050_MST_I2C_SLV4_NACK_BIT 4u
#define MPU6050_MST_I2C_SLV3_NACK_BIT 3u
#define MPU6050_MST_I2C_SLV2_NACK_BIT 2u
#define MPU6050_MST_I2C_SLV1_NACK_BIT 1u
#define MPU6050_MST_I2C_SLV0_NACK_BIT 0u

#define MPU6050_INTCFG_INT_LEVEL_BIT 7u
#define MPU6050_INTCFG_INT_OPEN_BIT 6u
#define MPU6050_INTCFG_LATCH_INT_EN_BIT 5u
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT 4u
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT 3u
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT 2u
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT 1u
#define MPU6050_INTCFG_CLKOUT_EN_BIT 0u

#define MPU6050_INTMODE_ACTIVEHIGH 0x00u
#define MPU6050_INTMODE_ACTIVELOW 0x01u

#define MPU6050_INTDRV_PUSHPULL 0x00u
#define MPU6050_INTDRV_OPENDRAIN 0x01u

#define MPU6050_INTLATCH_50USPULSE 0x00u
#define MPU6050_INTLATCH_WAITCLEAR 0x01u

#define MPU6050_INTCLEAR_STATUSREAD 0x00u
#define MPU6050_INTCLEAR_ANYREAD 0x01u

#define MPU6050_INTERRUPT_FF_BIT 7u
#define MPU6050_INTERRUPT_MOT_BIT 6u
#define MPU6050_INTERRUPT_ZMOT_BIT 5u
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT  4u
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT 3u
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT 2u
#define MPU6050_INTERRUPT_DMP_INT_BIT 1u
#define MPU6050_INTERRUPT_DATA_RDY_BIT 0u

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT 5u
#define MPU6050_DMPINT_4_BIT 4u
#define MPU6050_DMPINT_3_BIT 3u
#define MPU6050_DMPINT_2_BIT 2u
#define MPU6050_DMPINT_1_BIT 1u
#define MPU6050_DMPINT_0_BIT 0u

#define MPU6050_MOTION_MOT_XNEG_BIT 7u
#define MPU6050_MOTION_MOT_XPOS_BIT 6u
#define MPU6050_MOTION_MOT_YNEG_BIT 5u
#define MPU6050_MOTION_MOT_YPOS_BIT 4u
#define MPU6050_MOTION_MOT_ZNEG_BIT 3u
#define MPU6050_MOTION_MOT_ZPOS_BIT 2u
#define MPU6050_MOTION_MOT_ZRMOT_BIT  0u

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT 7u
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT 4u
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT 3u
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT 2u
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT 1u
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT 0u

#define MPU6050_PATHRESET_GYRO_RESET_BIT 2u
#define MPU6050_PATHRESET_ACCEL_RESET_BIT 1u
#define MPU6050_PATHRESET_TEMP_RESET_BIT 0u

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT 5u
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH 2u
#define MPU6050_DETECT_FF_COUNT_BIT 3u
#define MPU6050_DETECT_FF_COUNT_LENGTH 2u
#define MPU6050_DETECT_MOT_COUNT_BIT 1u
#define MPU6050_DETECT_MOT_COUNT_LENGTH 2u

#define MPU6050_DETECT_DECREMENT_RESET 0x0u
#define MPU6050_DETECT_DECREMENT_1 0x1u
#define MPU6050_DETECT_DECREMENT_2 0x2u
#define MPU6050_DETECT_DECREMENT_4 0x3u

#define MPU6050_USERCTRL_DMP_EN_BIT 7u
#define MPU6050_USERCTRL_FIFO_EN_BIT 6u
#define MPU6050_USERCTRL_I2C_MST_EN_BIT 5u
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT 4u
#define MPU6050_USERCTRL_DMP_RESET_BIT 3u
#define MPU6050_USERCTRL_FIFO_RESET_BIT 2u
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT  1u
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT 0u

#define MPU6050_PWR1_DEVICE_RESET_BIT 7u
#define MPU6050_PWR1_SLEEP_BIT 6u
#define MPU6050_PWR1_CYCLE_BIT 5u
#define MPU6050_PWR1_TEMP_DIS_BIT 3u
#define MPU6050_PWR1_CLKSEL_BIT 2u
#define MPU6050_PWR1_CLKSEL_LENGTH 3u

#define MPU6050_CLOCK_INTERNAL   0x00u                                          /* same as below use which you like */
#define MPU6050_CLOCK_PLL_XGYRO  0x01u
#define MPU6050_CLOCK_PLL_YGYRO  0x02u
#define MPU6050_CLOCK_PLL_ZGYRO  0x03u
#define MPU6050_CLOCK_PLL_EXT32K 0x04u
#define MPU6050_CLOCK_PLL_EXT19M 0x05u
#define MPU6050_CLOCK_KEEP_RESET 0x07u

#define MPU6050_CLK_SEL_INT_OSC 0u                                              /* internal oscillator */
#define MPU6050_CLK_SEL_PLL_X 1u                                                /* PLL with X Gyro reference */
#define MPU6050_CLK_SEL_PLL_Y 2u                                                /* PLL with Y Gyro reference */
#define MPU6050_CLK_SEL_PLL_Z 3u                                                /* PLL with Z Gyro reference */
#define MPU6050_CLK_SEL_PLL_32 4u                                               /* PLL with external 32.768kHz reference */
#define MPU6050_CLK_SEL_PLL_19 5u                                               /* PLL with external 19.2MHz reference */
#define MPU6050_CLK_SEL_STOP 7u                                                 /* Stops the clock and keeps the timing generator in reset */

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT 7u
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH 2u
#define MPU6050_PWR2_STBY_XA_BIT 5u
#define MPU6050_PWR2_STBY_YA_BIT 4u
#define MPU6050_PWR2_STBY_ZA_BIT 3u
#define MPU6050_PWR2_STBY_XG_BIT 2u
#define MPU6050_PWR2_STBY_YG_BIT 1u
#define MPU6050_PWR2_STBY_ZG_BIT 0u

#define MPU6050_WAKE_FREQ_1P25 0x0u
#define MPU6050_WAKE_FREQ_2P5 0x1u
#define MPU6050_WAKE_FREQ_5 0x2u
#define MPU6050_WAKE_FREQ_10 0x3u

#define MPU6050_BANKSEL_PRFTCH_EN_BIT 6u
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT 5u
#define MPU6050_BANKSEL_MEM_SEL_BIT 4u
#define MPU6050_BANKSEL_MEM_SEL_LENGTH 5u

#define MPU6050_WHO_AM_I_BIT 6u
#define MPU6050_WHO_AM_I_LENGTH 6u

#define MPU6050_DMP_MEMORY_BANKS 8u
#define MPU6050_DMP_MEMORY_BANK_SIZE 256u
#define MPU6050_DMP_MEMORY_CHUNK_SIZE 16u

#define MPU6050_MAX_RCV_LENGTH 8u

#define MPU6050_LP_WAKE_CTRL_1 0u                                               /* wake up frequency 1.25 Hz */
#define MPU6050_LP_WAKE_CTRL_2 1u                                               /* wake up frequency 2.5 Hz */
#define MPU6050_LP_WAKE_CTRL_5 2u                                               /* wake up frequency 5.0 Hz */
#define MPU6050_LP_WAKE_CTRL_10 3u                                              /* wake up frequency 10.0 Hz */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif