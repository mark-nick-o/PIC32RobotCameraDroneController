#ifndef CYRF6936_H
#define CYRF6936_H
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
 * Ported by ACP Aviation for mikro E 'C' for Microchip PIC32 MCU's
 */

/**
 * @file peripherals/cyrf6936.h
 * Driver for the cyrf6936 2.4GHz radio chip
 */
#include "definitions.h"
#include "cyrf6936_regs.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define CYRF6936PACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define CYRF6936PACKED __attribute__((packed))                                 /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define CYRF6936PACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define CYRF6936PACKED
#else                                                                           /* for MPLAB PIC32 */
  #define CYRF6936PACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define CYRF6936_MAX_BUFFER 80u                                                 /**< The max buffer size in the cyrf6936 structure */

/** SPI transaction queue length.
 * Number of transactions that can be queued.
 */
#ifndef SPI_TRANSACTION_QUEUE_LEN
#define SPI_TRANSACTION_QUEUE_LEN 8u
#endif

typedef enum {
  SPIMaster = _SPI_MASTER,
  SPISlave = _SPI_SLAVE
} SPIMode_e;

/** SPI slave selection behavior options.
 * Default operation should be SelectUnselected, but some peripherals
 * might need some special control.
 * Use non-default control only if you know what you're doing.
 */
typedef enum {
  SPISelectUnselect = _SPI_SS_ENABLE,                                           ///< slave is selected before transaction and unselected after
///<  SPISelect,                                                                     slave is selected before transaction but not unselected
///<  SPIUnselect,                                                                   slave is not selected but unselected after transaction
  SPINoSelect = _SPI_SS_DISABLE                                                 ///< slave is not selected nor unselected
} SPISlaveSelect_e;

/** SPI peripheral status.
 */
typedef enum {
  SPIIdle,
  SPIRunning
} SPIStatus_e;

typedef enum {
  SPIMSBFirst,
  SPILSBFirst
} SPIBitOrder_e;

typedef enum {
  SPIMiddleSample = _SPI_DATA_SAMPLE_MIDDLE,
  SPIEndSample = _SPI_DATA_SAMPLE_END
} SPIPhase_e;

/** Peripheral clock divider.
 * Defines the SPI baudrate
 */
typedef enum {
  SPIDiv2 = 2,
  SPIDiv4 = 4,
  SPIDiv8 = 8,
  SPIDiv16 = 16,
  SPIDiv32 = 32,
  SPIDiv64 = 64,
  SPIDiv128 = 128,
  SPIDiv256 = 256
} SPIClockDiv_e;

/** SPI CPHA (clock phase) options.
 * Control whether data line is sampled
 * at first or second edge of clock signal.
 */
typedef enum {
  SPICphaEdge1 = _SPI_ACTIVE_2_IDLE,                                            ///< CPHA = 0
  SPICphaEdge2 = _SPI_IDLE_2_ACTIVE                                             ///< CPHA = 1
} SPIClockPhase_e;

/** SPI slave selection options.
 * determines whether the Slave Select (SS) pin is used in communication. Valid in the Slave Mode only.
 */
typedef enum {
  SPISSPinUsed = _SPI_SS_ENABLE,                                                ///< SS Pin Used
  SPISSPinNotUsed = _SPI_SS_DISABLE                                             ///< SS Pin not used in slave
} SPISSPin_e;

/** SPI CPOL (clock polarity) options.
 * Control whether clock line is held
 * low or high in idle state.
 */
typedef enum {
  SPICpolIdleLow = _SPI_CLK_IDLE_LOW,                                           ///< CPOL = 0
  SPICpolIdleHigh = _SPI_CLK_IDLE_HIGH                                          ///< CPOL = 1
} SPIClockPolarity_e;

/** SPI data word size of transfer.
 */
typedef enum {
  SPIDss8bit = _SPI_8_BIT,
  SPIDss16bit = _SPI_16_BIT,
  SPIDss32bit = _SPI_32_BIT
} SPIDataSizeSelect_e;

/** SPI transaction status.
 */
typedef enum {
  SPITransPending,
  SPITransRunning,
  SPITransSuccess,
  SPITransFailed,
  SPITransDone
} SPITransactionStatus_e;

/** SPI Callback function.
 * If not NULL (or 0), call function (with transaction as parameter)
 * before or after transaction, e.g to allow execution of hardware specific actions
 */
///< struct spi_transaction_t;
///< typedef void (*SPICallback)(spi_transaction_t *trans);

/** SPI transaction structure.
 * - Use this structure to store a request of SPI transaction
 *   and submit it using #spi_submit function
 * - The input/output buffers needs to be created separately
 * - Take care of pointing input_buf/ouput_buf correctly
 * - input_length and output_length can be different, the larger number
 *   of the two specifies the total number of exchanged words,
 * - if input_length is larger than output length,
 *   0 is sent for the remaining words
 */
#if defined(D_FT900)
typedef struct CYRF6936PACKED {
  volatile uint8_t *input_buf;                                                  ///< pointer to receive buffer for DMA
  volatile uint8_t *output_buf;                                                 ///< pointer to transmit buffer for DMA
  uint16_t input_length;                                                        ///< number of data words to read
  uint16_t output_length;                                                       ///< number of data words to write
  uint8_t slave_idx;                                                            ///< slave id: #SPI_SLAVE0 to #SPI_SLAVE4
  SPISlaveSelect_e select;                                                      ///< slave selection behavior
  SPIClockPolarity_e cpol;                                                      ///< clock polarity control
  SPIClockPhase_e cpha;                                                         ///< clock phase control
  SPIDataSizeSelect_e dss;                                                      ///< data transfer word size
  SPIBitOrder_e bitorder;                                                       ///< MSB/LSB order
  SPIPhase_e phase;                                                             ///< phase _SPI_DATA_SAMPLE_MIDDLE or _SPI_DATA_SAMPLE_END
  SPIClockDiv_e cdiv;                                                           ///< prescaler of main clock to use as SPI clock
  ///<SPICallback before_cb;                                                         NULL or function called before the transaction
  ///<SPICallback after_cb;                                                         ///< NULL or function called after the transaction
  volatile SPITransactionStatus_e status;
} spi_transaction_t;                                                            /* SPI transaction type */
#else
CYRF6936PACKED(
typedef struct {
  volatile uint8_t *input_buf;                                                  ///< pointer to receive buffer for DMA
  volatile uint8_t *output_buf;                                                 ///< pointer to transmit buffer for DMA
  uint16_t input_length;                                                        ///< number of data words to read
  uint16_t output_length;                                                       ///< number of data words to write
  uint8_t slave_idx;                                                            ///< slave id: #SPI_SLAVE0 to #SPI_SLAVE4
  SPISlaveSelect_e select;                                                      ///< slave selection behavior
  SPIClockPolarity_e cpol;                                                      ///< clock polarity control
  SPIClockPhase_e cpha;                                                         ///< clock phase control
  SPIDataSizeSelect_e dss;                                                      ///< data transfer word size
  SPIBitOrder_e bitorder;                                                       ///< MSB/LSB order
  SPIPhase_e phase;                                                             ///< phase _SPI_DATA_SAMPLE_MIDDLE or _SPI_DATA_SAMPLE_END
  SPIClockDiv_e cdiv;                                                           ///< prescaler of main clock to use as SPI clock
  ///<SPICallback before_cb;                                                        ///< NULL or function called before the transaction
  ///<SPICallback after_cb;                                                         ///< NULL or function called after the transaction
  volatile SPITransactionStatus_e status;
}) spi_transaction_t;                                                           /* SPI transaction type */
#endif

/* The structure for the cyrf6936 chip that handles all the buffers and requests */
#if defined(D_FT900)
typedef struct CYRF6936PACKED {
  spi_transaction_t *trans[SPI_TRANSACTION_QUEUE_LEN];                          /** circular buffer holding transactions */
  uint8_t trans_insert_idx;
  uint8_t trans_extract_idx;
  volatile SPIStatus_e status;                                                  /** internal state of the peripheral */
  volatile uint8_t tx_idx_buf;
  volatile uint8_t rx_idx_buf;
  void *reg_addr;
  void *init_struct;
  SPIMode_e mode;
  volatile uint8_t suspend;                                                     /** control for stop/resume of the fifo */
  uint8_t spiDevNo;                                                             /* actual physical device we are using for this SPI */
} spi_periph_t;                                                                 /* SPI periferal type */
#else
CYRF6936PACKED(
typedef struct {
  spi_transaction_t *trans[SPI_TRANSACTION_QUEUE_LEN];                          /** circular buffer holding transactions */
  uint8_t trans_insert_idx;
  uint8_t trans_extract_idx;
  volatile SPIStatus_e status;                                                  /** internal state of the peripheral */
  volatile uint8_t tx_idx_buf;
  volatile uint8_t rx_idx_buf;
  void *reg_addr;
  void *init_struct;
  SPIMode_e mode;
  volatile uint8_t suspend;                                                     /** control for stop/resume of the fifo */
  uint8_t spiDevNo;                                                             /* actual physical device we are using for this SPI */
}) spi_periph_t;                                                                /* SPI periferal type */
#endif

/* The different statuses the cyrf6936 chip can be in */
typedef enum {
  CYRF6936_UNINIT,                                                              /**< The chip isn't initialized */
  CYRF6936_WAIT_FOR_ENABLE,                                                     /**< The chip is waiting after enabling the SPI bus */
  CYRF6936_GET_MFG_ID,                                                          /**< The chip is busy with getting the manufacturer ID */
  CYRF6936_IDLE,                                                                /**< The chip is idle and can be used */
  CYRF6936_MULTIWRITE,                                                          /**< The chip is writing multiple registers */
  CYRF6936_DATA_CODE,                                                           /**< The chip is writing a data code */
  CYRF6936_CHAN_SOP_DATA_CRC,                                                   /**< The chip is setting the channel, SOP code, DATA code and the CRC seed */
  CYRF6936_RX_IRQ_STATUS_PACKET,                                                /**< The chip is getting the receive irq status, receive status and the receive packet */
  CYRF6936_SEND                                                                 /**< The chip is busy sending a packet */
} Cyrf6936Status_e;

#if defined(D_FT900)
typedef struct CYRF6936PACKED {
  spi_periph_t *spi_p;                                                          /* The SPI peripheral for the connection */
  spi_transaction_t spi_t;                                                      /*  The SPI transaction used for the writing and reading of registers */
  volatile Cyrf6936Status_e status;                                        /**< The status of the CYRF6936 chip */
  uint8_t input_buf[17u];                                                       /**< The input buffer for the SPI transaction */
  uint8_t output_buf[17u];                                                      /**< The output buffer for the SPI transaction */
  uint8_t buffer[CYRF6936_MAX_BUFFER];                                          /**< The buffer used to write/read multiple registers */
  uint8_t buffer_length;                                                        /**< The length of the buffer used for MULTIWRITE */
  uint8_t buffer_idx;                                                           /**< The index of the buffer used for MULTIWRITE and used as sub-status for other statuses */
  uint8_t has_irq : 1u;                                                         /**< When the CYRF6936 is done reading the irq */
  uint8_t spare : 7u;  
  uint8_t mfg_id[6u];                                                           /**< The manufacturer id of the CYRF6936 chip */
  uint8_t tx_irq_status;                                                        /**< The last send interrupt status */
  uint8_t rx_irq_status;                                                        /**< The last receive interrupt status */
  uint8_t rx_status;                                                            /**< The last receive status */
  uint8_t rx_count;                                                             /**< The length of the received packet */
  uint8_t rx_packet[16u];                                                       /**< The last received packet */
  uint8_t initStat;                                                             /* initialisation state */
  uint32_t tickLast;                                                            /* store of the last time reference in cpu ticks from the co-processor */
} Cyrf6936_t;                                                                  // CYRF Wifi chip
#else
CYRF6936PACKED(
typedef struct {
  spi_periph_t *spi_p;                                                          /* The SPI peripheral for the connection */
  spi_transaction_t spi_t;                                                      /*  The SPI transaction used for the writing and reading of registers */
  volatile Cyrf6936Status_e status;                                        /**< The status of the CYRF6936 chip */
  uint8_t input_buf[17u];                                                       /**< The input buffer for the SPI transaction */
  uint8_t output_buf[17u];                                                      /**< The output buffer for the SPI transaction */
  uint8_t buffer[CYRF6936_MAX_BUFFER];                                          /**< The buffer used to write/read multiple registers */
  uint8_t buffer_length;                                                        /**< The length of the buffer used for MULTIWRITE */
  uint8_t buffer_idx;                                                           /**< The index of the buffer used for MULTIWRITE and used as sub-status for other statuses */
  uint8_t has_irq : 1u;                                                         /**< When the CYRF6936 is done reading the irq */
  uint8_t spare : 7u;  
  uint8_t mfg_id[6u];                                                           /**< The manufacturer id of the CYRF6936 chip */
  uint8_t tx_irq_status;                                                        /**< The last send interrupt status */
  uint8_t rx_irq_status;                                                        /**< The last receive interrupt status */
  uint8_t rx_status;                                                            /**< The last receive status */
  uint8_t rx_count;                                                             /**< The length of the received packet */
  uint8_t rx_packet[16u];                                                       /**< The last received packet */
  uint8_t initStat;                                                             /* initialisation state */
  uint32_t tickLast;                                                            /* store of the last time reference in cpu ticks from the co-processor */
}) Cyrf6936_t;                                                                  // CYRF Wifi chip
#endif

extern sbit Chip_Select1;
extern sbit Chip_Select_Direction1;
extern sbit Chip_Select2;
extern sbit Chip_Select_Direction2;
extern sbit Chip_Select3;
extern sbit Chip_Select_Direction3;
extern sbit Chip_Select4;
extern sbit Chip_Select_Direction4;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif