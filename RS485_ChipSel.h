// ====== RS485 Pins ===========================================================
#ifndef __CHIP_SEL_RS485
#define __CHIP_SEL_RS485

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

sbit RS485_rxtx_pin at RF2_bit;                                                 // set transcieve pin
sbit RS485_rxtx_pin_direction at TRISF2_bit;                                    // set transcieve pin direction

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif