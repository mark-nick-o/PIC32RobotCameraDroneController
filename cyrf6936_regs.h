/*
 * Copyright (C) 2013 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 * Ported by ACP Aviation for mikro E 'C' for PIC32
 */

/**
 * @file peripherals/cyrf6936_regs.h
 * Register defines for the CYRF6936 2.4GHz radio chip
 */

#ifndef CYRF6936_REGS_H
#define CYRF6936_REGS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* The SPI interface defines */
typedef enum {
  CYRF_CHANNEL            = 0x00,
  CYRF_TX_LENGTH          = 0x01,
  CYRF_TX_CTRL            = 0x02,
  CYRF_TX_CFG             = 0x03,
  CYRF_TX_IRQ_STATUS      = 0x04,
  CYRF_RX_CTRL            = 0x05,
  CYRF_RX_CFG             = 0x06,
  CYRF_RX_IRQ_STATUS      = 0x07,
  CYRF_RX_STATUS          = 0x08,
  CYRF_RX_COUNT           = 0x09,
  CYRF_RX_LENGTH          = 0x0A,
  CYRF_PWR_CTRL           = 0x0B,
  CYRF_XTAL_CTRL          = 0x0C,
  CYRF_IO_CFG             = 0x0D,
  CYRF_GPIO_CTRL          = 0x0E,
  CYRF_XACT_CFG           = 0x0F,
  CYRF_FRAMING_CFG        = 0x10,
  CYRF_DATA32_THOLD       = 0x11,
  CYRF_DATA64_THOLD       = 0x12,
  CYRF_RSSI               = 0x13,
  CYRF_EOP_CTRL           = 0x14,
  CYRF_CRC_SEED_LSB       = 0x15,
  CYRF_CRC_SEED_MSB       = 0x16,
  CYRF_TX_CRC_LSB         = 0x17,
  CYRF_TX_CRC_MSB         = 0x18,
  CYRF_RX_CRC_LSB         = 0x19,
  CYRF_RX_CRC_MSB         = 0x1A,
  CYRF_TX_OFFSET_LSB      = 0x1B,
  CYRF_TX_OFFSET_MSB      = 0x1C,
  CYRF_MODE_OVERRIDE      = 0x1D,
  CYRF_RX_OVERRIDE        = 0x1E,
  CYRF_TX_OVERRIDE        = 0x1F,
  CYRF_TX_BUFFER          = 0x20,
  CYRF_RX_BUFFER          = 0x21,
  CYRF_SOP_CODE           = 0x22,
  CYRF_DATA_CODE          = 0x23,
  CYRF_PREAMBLE           = 0x24,
  CYRF_MFG_ID             = 0x25,
  CYRF_XTAL_CFG           = 0x26,
  CYRF_CLK_OFFSET         = 0x27,
  CYRF_CLK_EN             = 0x28,
  CYRF_RX_ABORT           = 0x29,
  CYRF_AUTO_CAL_TIME      = 0x32,
  CYRF_AUTO_CAL_OFFSET    = 0x35,
  CYRF_ANALOG_CTRL        = 0x39,
} cyrf_interface_e;
#define CYRF_DIR                (1u<<7u)                                        /**< Bit for enabling writing */

// CYRF_MODE_OVERRIDE
#define CYRF_RST                (1u<<0u)

// CYRF_CLK_EN
#define CYRF_RXF                (1u<<1u)

// CYRF_XACT_CFG
typedef enum {
  CYRF_MODE_SLEEP     = (0x0 << 2),
  CYRF_MODE_IDLE      = (0x1 << 2),
  CYRF_MODE_SYNTH_TX  = (0x2 << 2),
  CYRF_MODE_SYNTH_RX  = (0x3 << 2),
  CYRF_MODE_RX        = (0x4 << 2),
} cyrf_mode_e;

#define CYRF_FRC_END            (1u<<5u)
#define CYRF_ACK_EN             (1u<<7u)

// CYRF_IO_CFG
#define CYRF_IRQ_GPIO           (1u<<0u)
#define CYRF_SPI_3PIN           (1u<<1u)
#define CYRF_PACTL_GPIO         (1u<<2u)
#define CYRF_PACTL_OD           (1u<<3u)
#define CYRF_XOUT_OD            (1u<<4u)
#define CYRF_MISO_OD            (1u<<5u)
#define CYRF_IRQ_POL            (1u<<6u)
#define CYRF_IRQ_OD             (1u<<7u)

// CYRF_FRAMING_CFG
#define CYRF_LEN_EN             (1u<<5u)
#define CYRF_SOP_LEN            (1u<<6u)
#define CYRF_SOP_EN             (1u<<7u)

// CYRF_RX_STATUS
typedef enum {
  CYRF_RX_DATA_MODE_GFSK  = 0x00,
  CYRF_RX_DATA_MODE_8DR   = 0x01,
  CYRF_RX_DATA_MODE_DDR   = 0x10,
  CYRF_RX_DATA_MODE_NV    = 0x11,
} cyrf_rx_data_mode_e;

#define CYRF_RX_CODE            (1u<<2u)
#define CYRF_BAD_CRC            (1u<<3u)
#define CYRF_CRC0               (1u<<4u)
#define CYRF_EOP_ERR            (1u<<5u)
#define CYRF_PKT_ERR            (1u<<6u)
#define CYRF_RX_ACK             (1u<<7u)

// CYRF_TX_IRQ_STATUS
#define CYRF_TXE_IRQ            (1u<<0u)
#define CYRF_TXC_IRQ            (1u<<1u)
#define CYRF_TXBERR_IRQ         (1u<<2u)
#define CYRF_TXB0_IRQ           (1u<<3u)
#define CYRF_TXB8_IRQ           (1u<<4u)
#define CYRF_TXB15_IRQ          (1u<<5u)
#define CYRF_LV_IRQ             (1u<<6u)
#define CYRF_OS_IRQ             (1u<<7u)

// CYRF_RX_IRQ_STATUS
#define CYRF_RXE_IRQ            (1u<<0u)
#define CYRF_RXC_IRQ            (1u<<1u)
#define CYRF_RXBERR_IRQ         (1u<<2u)
#define CYRF_RXB1_IRQ           (1u<<3u)
#define CYRF_RXB8_IRQ           (1u<<4u)
#define CYRF_RXB16_IRQ          (1u<<5u)
#define CYRF_SOPDET_IRQ         (1u<<6u)
#define CYRF_RXOW_IRQ           (1u<<7u)

// CYRF_TX_CTRL
#define CYRF_TXE_IRQEN          (1u<<0u)
#define CYRF_TXC_IRQEN          (1u<<1u)
#define CYRF_TXBERR_IRQEN       (1u<<2u)
#define CYRF_TXB0_IRQEN         (1u<<3u)
#define CYRF_TXB8_IRQEN         (1u<<4u)
#define CYRF_TXB15_IRQEN        (1u<<5u)
#define CYRF_TX_CLR             (1u<<6u)
#define CYRF_TX_GO              (1u<<7u)

// CYRF_RX_CTRL
#define CYRF_RXE_IRQEN          (1u<<0u)
#define CYRF_RXC_IRQEN          (1u<<1u)
#define CYRF_RXBERR_IRQEN       (1u<<2u)
#define CYRF_RXB1_IRQEN         (1u<<3u)
#define CYRF_RXB8_IRQEN         (1u<<4u)
#define CYRF_RXB16_IRQEN        (1u<<5u)
#define CYRF_RSVD               (1u<<6u)
#define CYRF_RX_GO              (1u<<7u)

// CYRF_RX_OVERRIDE
#define CYRF_ACE                (1u<<1u)
#define CYRF_DIS_RXCRC          (1u<<2u)
#define CYRF_DIS_CRC0           (1u<<3u)
#define CYRF_FRC_RXDR           (1u<<4u)
#define CYRF_MAN_RXACK          (1u<<5u)
#define CYRF_RXTX_DLY           (1u<<6u)
#define CYRF_ACK_RX             (1u<<7u)

// CYRF_TX_OVERRIDE
#define CYRF_TX_INV             (1u<<0u)
#define CYRF_DIS_TXCRC          (1u<<2u)
#define CYRF_OVRD_ACK           (1u<<3u)
#define CYRF_MAN_TXACK          (1u<<4u)
#define CYRF_FRC_PRE            (1u<<6u)
#define CYRF_ACK_TX             (1u<<7u)

// CYRF_RX_CFG
#define CYRF_VLD_EN             (1u<<0u)
#define CYRF_RXOW_EN            (1u<<1u)
#define CYRF_FAST_TURN_EN       (1u<<3u)
#define CYRF_HILO               (1u<<4u)
#define CYRF_ATT                (1u<<5u)
#define CYRF_LNA                (1u<<6u)
#define CYRF_AGC_EN             (1u<<7u)

typedef enum {                                                                  // CYRF_TX_CFG
  CYRF_PA_M35     = 0x0,
  CYRF_PA_M30     = 0x1,
  CYRF_PA_M24     = 0x2,
  CYRF_PA_M18     = 0x3,
  CYRF_PA_M13     = 0x4,
  CYRF_PA_M5      = 0x5,
  CYRF_PA_0       = 0x6,
  CYRF_PA_4       = 0x7,
} cyrf_pa_e;

typedef enum {
  CYRF_DATA_MODE_GFSK = (0x0 << 3u),
  CYRF_DATA_MODE_8DR  = (0x1 << 3u),
  CYRF_DATA_MODE_DDR  = (0x2 << 3u),
  CYRF_DATA_MODE_SDR  = (0x3 << 3u),
} cyrf_data_mode_e;

#define CYRF_DATA_CODE_LENGTH   (1u<<5u)

#define CYRF6936_INIT_DLY1 1000u
#define CYRF6936_INIT_DLY2 1000u

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // CYRF6936_REGS_H