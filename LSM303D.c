#include "definitions.h"
#if defined(LSM303M_USED)

#include <stdint.h>
#include "Struts.h"
#include "LSM303D.h"
#include "LSM303_Chip_Sel.h"

void lsm_acc_write_register(unsigned char reg, unsigned char dataV);
void lsm_acc_setup();
void lsm_acc_read_register(unsigned char reg, unsigned char dataV[], uint16_t len);
void get_lsm303Data(lsm303_data_t *readData);

/*-----------------------------------------------------------------------------
 *      lsm_acc_write_register():  write to register the data specified on spi bus
 *
 *  Parameters: unsigned char reg, unsigned char dataV
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void lsm_acc_write_register(unsigned char reg, unsigned char dataV)
{    
    Chip_Select4 = 0;                                                           /* bring CS low to activate SPI */
    SPI4_Write(reg);
    SPI4_Write(dataV);
    Chip_Select4 = 1;                                                           /* complete the command */
}
/*-----------------------------------------------------------------------------
 *      lsm_acc_setup():  set up spi bus and instrument
 *
 *  Parameters: unsigned char reg, unsigned char dataV[], uint16_t len
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void lsm_acc_setup()
{
   // setup the accelerometer, using SPI 4
   TRISBbits.TRISB8 = 0;
   Chip_Select4 = 1;
   /* Master - SPI4, pins are: SDI4(F4), SDO4(F5), SCK4(B14).
      we manually control SS4 as a digital output (B8)
      since the PIC is just starting, we know that spi is off. We rely on defaults here
      setup SPI4 */
   SPI4_Init();                                                                 /* initialise SPI link 4 */
   SPI4CON = 0u;                                                                /* turn off the SPI module and reset it */
   SPI4BUF;                                                                     /* clear the rx buffer by reading from it  */
   SPI4BRG = 0x3u;                                                              /* baud rate to 10MHz [SPI4BRG = (80000000/(2*desired))-1]  */
   SPI4STATbits.SPIROV = 0;                                                     /* clear the overflow bit  */
   SPI4CONbits.CKE = 1;                                                         /* data changes when clock goes from active to inactive (high to low since CKP is 0) */
   SPI4CONbits.MSTEN = 1;                                                       /* master operation  */
   SPI4CONbits.ON = 1;                                                          /* turn on SPI 4  */

   lsm_acc_write_register(CTRL1, 0xAFU);                                        /* set the accelerometer data rate to 1600 Hz. Do not update until we read values  */
   lsm_acc_write_register(CTRL5, 0xF0U);                                        /* 50 Hz magnetometer, high resolution, temperature sensor on */
   lsm_acc_write_register(CTRL7, 0x0U);                                         /* enable continuous reading of the magnetometer */
}
/*-----------------------------------------------------------------------------
 *      lsm_acc_read_register():  read the data from spi bus
 *
 *  Parameters: unsigned char reg, unsigned char dataV[], uint16_t len
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void lsm_acc_read_register(unsigned char reg, unsigned char dataV[], uint16_t len)
{
   uint16_t char2rcv;
   reg |= 0x80u;                                                                /* set the read bit (as per the accelerometer’s protocol) */
   if(len >= 1u)
   {
     reg |= 0x40u;                                                              /* set the address auto inc. bit (as per the accelerometer’s protocol) } */
     Chip_Select4 = 0;
     SPI4_Write((uint32_t)reg);
     for(char2rcv = 0u; char2rcv != len; ++char2rcv)
     {
        dataV[char2rcv] = SPI4_Read(0);                                         /* read data from spi  */
     }
     Chip_Select4= 1;
   }
}
/*-----------------------------------------------------------------------------
 *      get_lsm303Data():  get the accelerometer compass and temperature
 *
 *  Parameters: lsm303_data_t *readData
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void get_lsm303Data(lsm303_data_t *readData)
{
   lsm_acc_read_register(OUT_X_L_A, (unsigned char *)readData->accels,6u);
   lsm_acc_read_register(OUT_X_L_M, (unsigned char *)readData->mags, 6u);
   lsm_acc_read_register(TEMP_OUT_LO,(unsigned char *)readData->temp,2u);
}

#endif