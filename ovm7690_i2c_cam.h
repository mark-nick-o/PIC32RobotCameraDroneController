#ifndef __ovm7690_lib
#define __ovm7690_lib
/*-----------------------------------------------------------------------------
 * Name:    CAM.c
 * Purpose: Digital camera driver
 * Note(s):
 *-----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2011 Keil - An ARM Company. All rights reserved.
 * Ported by ACP Aviation - At present left DMA and DCMI part commented out
 *----------------------------------------------------------------------------*/

/* #include <stm32f4xx.h>                  STM32F4xx Definitions              */
/* #include "I2C.h" */
/* #include "CAM.h" */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* I2C Camera address */
#define CAM_I2C_ADDR 0x21U                                                      /* 7-bit accelerator I2C address      */

/* DMA Source and destination addresses TBD for PIC32 */
#define DCMI_SRC_ADDR   0x50050028UL
#define LCD_DST_ADDR    0x6C000002UL

/* Local bitmasks and settings definitions TBD for PIC32 */
#define PH_MODER_MASK  (GPIO_MODER_MODER8       | GPIO_MODER_MODER9       | GPIO_MODER_MODER10      | \
                        GPIO_MODER_MODER11      | GPIO_MODER_MODER12      | GPIO_MODER_MODER14      )
#define PH_MODER_SET   (GPIO_MODER_MODER8_1     | GPIO_MODER_MODER9_1     | GPIO_MODER_MODER10_1    | \
                        GPIO_MODER_MODER11_1    | GPIO_MODER_MODER12_1    | GPIO_MODER_MODER14_1    )
#define PH_OTYPER_MASK (GPIO_OTYPER_OT_8        | GPIO_OTYPER_OT_9        | GPIO_OTYPER_OT_10       | \
                        GPIO_OTYPER_OT_11       | GPIO_OTYPER_OT_12       | GPIO_OTYPER_OT_14       )
#define PH_OSPEEDR_SET (GPIO_OSPEEDER_OSPEEDR8  | GPIO_OSPEEDER_OSPEEDR9  | GPIO_OSPEEDER_OSPEEDR10 | \
                        GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR14 )
#define PH_PUPDR_MASK  (GPIO_PUPDR_PUPDR8       | GPIO_PUPDR_PUPDR9       | GPIO_PUPDR_PUPDR10      | \
                        GPIO_PUPDR_PUPDR11      | GPIO_PUPDR_PUPDR12      | GPIO_PUPDR_PUPDR14      )
#define PH_PUPDR_SET   (GPIO_PUPDR_PUPDR8_0     | GPIO_PUPDR_PUPDR9_0     | GPIO_PUPDR_PUPDR10_0    | \
                        GPIO_PUPDR_PUPDR11_0    | GPIO_PUPDR_PUPDR12_0    | GPIO_PUPDR_PUPDR14_0    )

#define PI_MODER_MASK  (GPIO_MODER_MODER4       | GPIO_MODER_MODER5       | GPIO_MODER_MODER6       | GPIO_MODER_MODER7      )
#define PI_MODER_SET   (GPIO_MODER_MODER4_1     | GPIO_MODER_MODER5_1     | GPIO_MODER_MODER6_1     | GPIO_MODER_MODER7_1    )
#define PI_OTYPER_MASK (GPIO_OTYPER_OT_4        | GPIO_OTYPER_OT_5        | GPIO_OTYPER_OT_6        | GPIO_OTYPER_OT_7       )
#define PI_OSPEEDR_SET (GPIO_OSPEEDER_OSPEEDR4  | GPIO_OSPEEDER_OSPEEDR5  | GPIO_OSPEEDER_OSPEEDR6  | GPIO_OSPEEDER_OSPEEDR7 )
#define PI_PUPDR_MASK  (GPIO_PUPDR_PUPDR4       | GPIO_PUPDR_PUPDR5       | GPIO_PUPDR_PUPDR6       | GPIO_PUPDR_PUPDR7      )
#define PI_PUPDR_SET   (GPIO_PUPDR_PUPDR4_0     | GPIO_PUPDR_PUPDR5_0     | GPIO_PUPDR_PUPDR6_0     | GPIO_PUPDR_PUPDR7_0    )

/* Calculate array size */
#define ARR_SZ(x) (sizeof (x) / sizeof(x[0]))


typedef struct 
{
  uint8_t Addr;
  uint8_t Val;
} REG_VAL_t;                                                                      /* Register value */

/* OmniVision recommended settings based on OVM7690 Setting V2.2              */
/* Modified for RGB QVGA settings                                             */
REG_VAL_t CAM_RegInit[] = {
  {0x0EU, 0x00U},                                                               /* Normal operation mode              */
  {0x0CU, 0x46U},
  {0x81U, 0xFFU},
  {0x21U, 0x44U},
  {0x16U, 0x03U},
  {0x39U, 0x80U},
  {0x1EU, 0xB1U},

  /* Format */
  {0x12U, 0x06U},                                                               /* Output format control: RGB         */
  {0x82U, 0x03U},
  {0xD0U, 0x48U},
  {0x80U, 0x7FU},
  {0x3EU, 0x30U},
  {0x22U, 0x00U},

  /* Resolution */
  {0x17U, 0x69U},                                                               /* Horizontal window start point      */
  {0x18U, 0xA4U},                                                               /* Horizontal senzor size             */
  {0x19U, 0x0CU},                                                               /* Vertical Window start line         */
  {0x1AU, 0xF6U},                                                               /* Vertical sensor size               */

  {0xC8U, 0x02U},
  {0xC9U, 0x80U},
  {0xCAU, 0x01U},
  {0xCBU, 0xE0U},
  {0xCCU, 0x02U},
  {0xCDU, 0x80U},
  {0xCEU, 0x01U},
  {0xCFU, 0xE0U},

  {0x85U, 0x90U},                                                               /* Lens Correction */
  {0x86U, 0x00U},
  {0x87U, 0x00U},
  {0x88U, 0x10U},
  {0x89U, 0x30U},
  {0x8AU, 0x29U},
  {0x8BU, 0x26U},

  {0xBBU, 0x80U},                                                               /* Color Matrix */
  {0xBCU, 0x62U},
  {0xBDU, 0x1EU},
  {0xBEU, 0x26U},
  {0xBFU, 0x7BU},
  {0xC0U, 0xACU},
  {0xC1U, 0x1EU},

  {0xB7U, 0x05U},                                                               /* Edge + Denoise */
  {0xB8U, 0x09U},
  {0xB9U, 0x00U},
  {0xBAU, 0x18U},

  {0x5AU, 0x4AU},                                                               /* UVAdjust */
  {0x5BU, 0x9FU},
  {0x5CU, 0x48U},
  {0x5DU, 0x32U},

  {0x24U, 0x78U},                                                               /* AEC/AGC target */
  {0x25U, 0x68U},
  {0x26U, 0xB3U},

  {0xA3U, 0x0BU},                                                               /* Gamma */
  {0xA4U, 0x15U},
  {0xA5U, 0x2AU},
  {0xA6U, 0x51U},
  {0xA7U, 0x63U},
  {0xA8U, 0x74U},
  {0xA9U, 0x83U},
  {0xAAU, 0x91U},
  {0xABU, 0x9EU},
  {0xACU, 0xAAU},
  {0xADU, 0xBEU},
  {0xAEU, 0xCEU},
  {0xAFU, 0xE5U},
  {0xB0U, 0xF3U},
  {0xB1U, 0xFBU},
  {0xB2U, 0x06U},

  {0x8CU, 0x5DU},                                                               /* Advance */
  {0x8DU, 0x11U},
  {0x8EU, 0x12U},
  {0x8FU, 0x11U},
  {0x90U, 0x50U},
  {0x91U, 0x22U},
  {0x92U, 0xD1U},
  {0x93U, 0xA7U},
  {0x94U, 0x23U},
  {0x95U, 0x3BU},
  {0x96U, 0xFFU},
  {0x97U, 0x00U},
  {0x98U, 0x4AU},
  {0x99U, 0x46U},
  {0x9AU, 0x3DU},
  {0x9BU, 0x3AU},
  {0x9CU, 0xF0U},
  {0x9DU, 0xF0U},
  {0x9EU, 0xF0U},
  {0x9FU, 0xFFU},
  {0xA0U, 0x56U},
  {0xA1U, 0x55U},
  {0xA2U, 0x13U},

  {0x50U, 0x9AU},                                                               /* General Control */
  {0x51U, 0x80U},
  {0x21U, 0x23U},

  {0x14U, 0x29U},
  {0x13U, 0xF7U},
  {0x11U, 0x01U},

  {0x0EU, 0x00U}
};

/* Camera module register settings for QVGA/RGB565 capture.                   */
REG_VAL_t CAM_RegSz[] = {
  {0xC8U, 0x02U},
  {0xC9U, 0x80U},
  {0xCAU, 0x01U},
  {0xCBU, 0xE0U},
  {0xCCU, 0x01U},
  {0xCdU, 0x40U},
  {0xCEU, 0x00U},
  {0xCFU, 0xF0U},
};

#define I2C_CAM_LIB static

/* ================= Prototypes ============================================= */
I2C_CAM_LIB void CAM_SetQVGA(void);
I2C_CAM_LIB uint32_t CAM_WrReg(uint8_t reg, uint8_t byte);
I2C_CAM_LIB uint32_t CAM_RdReg(uint8_t reg);
I2C_CAM_LIB void CAM_Init(void);
I2C_CAM_LIB uint32_t CAM_On(void);
I2C_CAM_LIB uint32_t CAM_Off(void);
I2C_CAM_LIB uint32_t CAM_Test(uint32_t mode);

/*-----------------------------------------------------------------------------
 *      CAM_WrReg:  Write a value to camera register
 *
 *  Parameters:  reg - register address
 *               num - number of bytes to write
 *               val - value to be written
 *
 *  Return: 0 on success, nonzero on error
 *----------------------------------------------------------------------------*/
uint32_t CAM_WrReg(uint8_t reg, uint8_t byte)
{
  uint32_t err = 0UL;

  if (!write_i2c(CAM_I2C_ADDR, reg, byte))
  {
    err |= (1UL << 31UL);
  }
  return (err);
}


/*-----------------------------------------------------------------------------
 *      CAM_RdReg:  Read a value from camera register
 *
 *  Parameters:  reg - register address
 *
 *  Return: 8-bit register value, >8-bit value on error
 *----------------------------------------------------------------------------*/
uint32_t CAM_RdReg(uint8_t reg)
{
  uint32_t err = 0UL;
  uint8_t  val = 0U;

  if (!read_single_i2c(CAM_I2C_ADDR, reg, val))
  {
    err |= (1UL << 31UL);
  }
  return (err | val);
}
/*-----------------------------------------------------------------------------
 *      CAM_Init:  Initialize digital camera interface
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void CAM_Init(void)
{
  uint32_t val;

  CAM_SetQVGA ();                                                               /* Configure camera size */

  val = CAM_RdReg(0x6Fu) & ~(1u << 7u);
  CAM_WrReg(0x6Fu, val);

  /* Enable GPIOA, GPIOH and GPIOI clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                  RCC_AHB1ENR_GPIOHEN |
                  RCC_AHB1ENR_GPIOIEN ;

  RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN; */

  /* Configure PA6: push-pull output, 100 MHz, with pull-up, AF is DCMI
  GPIOA->MODER   &= ~GPIO_MODER_MODER6;
  GPIOA->MODER   |=  GPIO_MODER_MODER6_1;
  GPIOA->OTYPER  &=  GPIO_OTYPER_OT_6;
  GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR6;
  GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPDR6;
  GPIOA->PUPDR   |=  GPIO_PUPDR_PUPDR6_0;
  GPIOA->AFR[0]  &= ~0x0F000000;
  GPIOA->AFR[0]  |=  0x0D000000;  */

  /* Configure PH8, PH9, PH10, PH11, PH12, PH14                               */
  /* Pins are push-pull outputs, 100MHz, with pull-up, AF is DCMI
  GPIOH->MODER   &= ~PH_MODER_MASK;
  GPIOH->MODER   |=  PH_MODER_SET;
  GPIOH->OTYPER  &= ~PH_OTYPER_MASK;
  GPIOH->OSPEEDR |=  PH_OSPEEDR_SET;
  GPIOH->PUPDR   &= ~PH_PUPDR_MASK;
  GPIOH->PUPDR   |=  PH_PUPDR_SET;
  GPIOH->AFR[1]  &= ~0x0F0FFFFF;
  GPIOH->AFR[1]  |=  0x0D0DDDDD; */

  /* Configure PI4, PI5, PI6, PI7                                             */
  /* Pins are push-pull outputs, 100MHz, with pull-up, AF is DCMI
  GPIOI->MODER   &= ~PI_MODER_MASK;
  GPIOI->MODER   |=  PI_MODER_SET;
  GPIOI->OTYPER  &= ~PI_OTYPER_MASK;
  GPIOI->OSPEEDR |=  PI_OSPEEDR_SET;
  GPIOI->PUPDR   &= ~PI_PUPDR_MASK;
  GPIOI->PUPDR   |=  PI_PUPDR_SET;
  GPIOI->AFR[0]  &= ~0xFFFF0000;
  GPIOI->AFR[0]  |=  0xDDDD0000; */

  /* Configure DCMI peripheral
  DCMI->CR &= ~(DCMI_CR_ENABLE | DCMI_CR_CAPTURE);  */

  /* DCMI->CR  =  DCMI_CR_PCKPOL |         Rising polarity                    */
  /*             DCMI_CR_VSPOL;            VS Polarity high                   */
                                        /* HS Polarity low                    */
                                        /* Continuous mode                    */
                                        /* Hardware sync                      */
                                        /* Capture rate all frames            */
                                        /* Extended data mode 8b              */
  /* Configure DMA
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;      Enable DMA2 clock                  */

  /* DMA2_Stream1->CR = DMA_SxCR_CHSEL_0 |  Select channel 1                   */
                /*     DMA_SxCR_MSIZE_0 |  Memory data size is half-word      */
                /*     DMA_SxCR_PSIZE_1 | Peripheral data size is word       */
                /*     DMA_SxCR_PL_1    |  DMA priority high                  */
                /*     DMA_SxCR_CIRC    ;  Curcular mode enabled              */

  /* DMA2_Stream1->FCR  = DMA_SxFCR_FTH  |  FIFO threshold: full FIFO          */
  /*                     DMA_SxFCR_DMDIS;  Direct mode disabled               */
  /* DMA2_Stream1->NDTR = 1;                Transfer 1 byte                    */
  /* DMA2_Stream1->PAR  = DCMI_SRC_ADDR;    Set DCMI data source address       */
  /* DMA2_Stream1->M0AR = LCD_DST_ADDR;    Set LCD data destination address   */
}

/*-----------------------------------------------------------------------------
 *      CAM_SetQVGA:  Configure display size to QVGA (240*320)
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
static void CAM_SetQVGA(void)
{
  uint32_t i;

  for (i = 0UL; i < ARR_SZ(CAM_RegInit); i++)
  {
    CAM_WrReg (CAM_RegInit[i].Addr, CAM_RegInit[i].Val);
  }

  for (i = 0UL; i < ARR_SZ(CAM_RegSz); i++)
  {
    CAM_WrReg (CAM_RegSz[i].Addr, CAM_RegSz[i].Val);
  }
}


/*-----------------------------------------------------------------------------
 *      CAM_On:  Turn camera ON
 *
 *  Parameters: (none)
 *  Return:     0 on success, nonzero otherwise
 *----------------------------------------------------------------------------*/
uint32_t CAM_On(void)
{
  uint32_t val;

  val  = CAM_RdReg(0x0EU);
  val &= ~(1UL << 3UL);
  CAM_WrReg (0x0EU, val);                                                       /* Put camera into normal mode        */

  /* DMA2_Stream1->CR |= DMA_SxCR_EN;       Enable DMA                         */
  /* DCMI->CR |= DCMI_CR_ENABLE;            Enable DCMI                        */
  /* DCMI->CR |= DCMI_CR_CAPTURE;           Start image capture                */
  return (0UL);
}


/*-----------------------------------------------------------------------------
 *      CAM_On:  Turn camera OFF
 *
 *  Parameters: (none)
 *  Return:     0 on success, nonzero otherwise
 *----------------------------------------------------------------------------*/
uint32_t CAM_Off(void)
{
  uint32_t val;

  /* DMA2_Stream1->CR &= ~DMA_SxCR_EN;      Disable DMA                        */
  /* DCMI->CR &= ~DCMI_CR_CAPTURE;          Disable image capture              */
  /* DCMI->CR &= ~DCMI_CR_ENABLE;           Disable DCMI                       */

  val = CAM_RdReg (0x0EU);
  CAM_WrReg(0x0EU, val | (1U << 3U));                                           /* Put camera into sleep mode         */
  return (0UL);
}

/*-----------------------------------------------------------------------------
 *      CAM_Test:  Enable camera test mode
 *
 *  Parameters: mode - type of test pattern
 *  Return:     0 on success, nonzero otherwise
 *----------------------------------------------------------------------------*/
#define TEST_PAT_OFF 0u
uint32_t CAM_Test(uint32_t mode)
{
  uint32_t val;

  val = CAM_RdReg(0x0CU);
  if (mode == TEST_PAT_OFF)
  {
    val &= ~1UL;                                                                /* Color bar OFF                      */
  }
  else
  {
    val |= 1UL;                                                                 /* Color bar enable                   */
  }
  CAM_WrReg(0x0CU, val);
  return (0UL);
}

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif                                                                          /* end of library */