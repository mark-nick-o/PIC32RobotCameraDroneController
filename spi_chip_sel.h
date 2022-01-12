// ====== SPI Pins =============================================================
#ifndef __CHIP_SEL_SPI
#define __CHIP_SEL_SPI

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if defined(CYPRESS_2_4GHZ_USB)
sbit Chip_Select1 at LATG0_bit;
sbit Chip_Select_Direction1 at TRISG0_bit;
sbit Chip_Select2 at LATG7_bit;
sbit Chip_Select_Direction2 at TRISG7_bit;
sbit Chip_Select3 at LATE0_bit;
sbit Chip_Select_Direction3 at TRISE0_bit;
#ifndef LSM303M_USED
sbit Chip_Select4 at LATF0_bit;
sbit Chip_Select_Direction4 at TRISF0_bit;
#endif
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif