#include <stdint.h>
#include "definitions.h"
#if defined(CYPS_24GHZ_USB)  

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Copyright (C) 2013 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi. Ported and modified by ACP Aviation
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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/**
 * @file peripherals/cyrf6936.c
 * Driver for the cyrf6936 2.4GHz radio chip
 */
#include "cyrf6936_regs.h"
#include "cyrf6936.h"
#include "gc_events.h"
/*#include "mmc_file_handler.h"*/
#include "spi_chip_sel.h"
#include "deviceIC.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* ------------------ Function Definition --------------------------------- */
uint8_t cyrf6936_write_block(Cyrf6936_t *cyrf, uint8_t addr, uint8_t dataV[], uint8_t length);
void cyrf6936_init(Cyrf6936_t *cyrf, spi_periph_t *spi_p, const uint8_t slave_idx);
uint8_t cyrf6936_read_block(Cyrf6936_t *cyrf, const uint8_t addr, const uint8_t length);
uint8_t cyrf6936_write_chan_sop_data_crc(Cyrf6936_t *cyrf, const uint8_t chan, const uint8_t sop_code[], const uint8_t data_code[], const uint16_t crc_seed);
uint8_t cyrf6936_send_packet(Cyrf6936_t *cyrf, const uint8_t dataV[], const uint8_t length);
uint8_t cyrf6936_read_rx_irq_status_packet(Cyrf6936_t *cyrf);
void cyrf6936_event(Cyrf6936_t *cyrf);

/*-----------------------------------------------------------------------------
 *  cyrf6936_write_block(): Write multiple bytes to a register
 *                                      
 *  Parameters: Cyrf6936_t *cyrf, uint8_t addr, uint8_t dataV[], uint8_t length 
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t cyrf6936_write_block(Cyrf6936_t *cyrf, uint8_t addr, uint8_t dataV[], uint8_t length)
{
  uint8_t i;

  if (cyrf->spi_t.status != SPITransDone)                                       /* Check if there is already a SPI transaction busy */
  {
    return FALSE;
  }

  cyrf->spi_t.output_length = length + 1;                                       /* Set the buffer and commit the transaction */
  cyrf->spi_t.input_length = 0;
  cyrf->output_buf[0] = addr | CYRF_DIR;                                        /* set the address and write bit the first write we make */

  for (i = 0; i < length; i++)                                                  /* Copy the data (we use the buffer here for debug only if too slow remove this buffer */
  {
    cyrf->output_buf[i + 1] = dataV[i];
  }

  switch (cyrf->spi_p->spiDevNo)                                                /* Submit the transaction */
  {
       case SPIDEV_1:
       Chip_Select1 = 0;                                                        /* bring CS low to activate SPI */
       for (i = 0; i < cyrf->spi_t.output_length; i++)
       {
           SPI1_Write(cyrf->output_buf[i]);
       }
       Chip_Select1 = 1;                                                        /* bring CS high to de-activate SPI */
       break;

       case SPIDEV_2:
       Chip_Select2 = 0;                                                        /* bring CS low to activate SPI */
       for (i = 0; i < cyrf->spi_t.output_length; i++)
       {
           SPI2_Write(cyrf->output_buf[i]);
       }
       Chip_Select2 = 1;                                                        /* bring CS high to de-activate SPI */
       break;

       case SPIDEV_3:
       Chip_Select3 = 0;                                                        /* bring CS low to activate SPI */
       for (i = 0; i < cyrf->spi_t.output_length; i++)
       {
           SPI3_Write(cyrf->output_buf[i]);
       }
       Chip_Select3 = 1;                                                        /* bring CS high to de-activate SPI */
       break;

#ifndef LSM303M_USED
       case SPIDEV_4:
       Chip_Select4 = 0;                                                        /* bring CS low to activate SPI */
       for (i = 0; i < cyrf->spi_t.output_length; i++)
       {
           SPI4_Write(cyrf->output_buf[i]);
       }
       Chip_Select4 = 1;                                                        /* bring CS high to de-activate SPI */
       break;
#endif
       
  }

  /*return spi_submit(cyrf->spi_p, &(cyrf->spi_t));*/
}

/*-----------------------------------------------------------------------------
 *  cyrf6936_init(): Initializing the cyrf chip 
 *                                      
 *  Parameters: Cyrf6936_t *cyrf, spi_periph_t *spi_p, const uint8_t slave_idx 
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
void cyrf6936_init(Cyrf6936_t *cyrf, spi_periph_t *spi_p, const uint8_t slave_idx)
{
   int32_t ticks2Now;
   uint8_t sendThis = 0xFFu;                                                    /* data to send */
   uint8_t numBytes2Send = 1u;                                                  /* number of bytes to send */

   switch (cyrf->initStat)
   {
      case 0u:                                                                  /* Set spi_peripheral and the status */
      cyrf->spi_p = spi_p;
      cyrf->status = CYRF6936_UNINIT;
      cyrf->initStat = 1;
      cyrf->has_irq = FALSE;
      break;

      case 1u:
      switch (cyrf->status)
      {
         case CYRF6936_UNINIT:                                                  /* Set the spi transaction */
         cyrf->spi_t.cpol = SPICpolIdleLow;
         cyrf->spi_t.cpha = SPICphaEdge1;
         cyrf->spi_t.dss = SPIDss8bit;
         cyrf->spi_t.bitorder = SPIMSBFirst;
         cyrf->spi_t.cdiv = SPIDiv64;
         cyrf->spi_t.input_length = 0;
         cyrf->spi_t.output_length = 0;
         cyrf->spi_t.input_buf = cyrf->input_buf;
         cyrf->spi_t.output_buf = cyrf->output_buf;
         cyrf->spi_t.slave_idx = slave_idx;
         cyrf->spi_t.select = SPISelectUnselect;
         cyrf->spi_t.phase = SPIEndSample;
         switch (cyrf->spi_p->spiDevNo)
         {
              case SPIDEV_1:
              Chip_Select1 = 1;                                                 /* bring CS high to de-activate SPI */
              Chip_Select_Direction1 = 0;                                       // Set CS# pin as Output
              SPI1_Init_Advanced(cyrf->spi_p->mode, cyrf->spi_t.dss, cyrf->spi_t.cdiv, cyrf->spi_t.select, cyrf->spi_t.phase, cyrf->spi_t.cpol, cyrf->spi_t.cpha);
              SPI1CON = 0u;                                                     /* turn off the SPI module and reset it */
              SPI1BUF;                                                          /* clear the rx buffer by reading from it  */
              SPI1STATbits.SPIROV = 0;                                          /* clear the overflow bit  */
              break;

              case SPIDEV_2:
              Chip_Select2 = 1;                                                 /* bring CS high to de-activate SPI */
              Chip_Select_Direction2 = 0;                                       // Set CS# pin as Output
              SPI2_Init_Advanced(cyrf->spi_p->mode, cyrf->spi_t.dss, cyrf->spi_t.cdiv, cyrf->spi_t.select, cyrf->spi_t.phase, cyrf->spi_t.cpol, cyrf->spi_t.cpha);
              SPI2CON = 0u;                                                     /* turn off the SPI module and reset it */
              SPI2BUF;                                                          /* clear the rx buffer by reading from it  */
              SPI2STATbits.SPIROV = 0;                                          /* clear the overflow bit  */
              break;

              case SPIDEV_3:
              Chip_Select3 = 1;                                                 /* bring CS high to de-activate SPI */
              Chip_Select_Direction3 = 0;                                       // Set CS# pin as Output
              SPI3_Init_Advanced(cyrf->spi_p->mode, cyrf->spi_t.dss, cyrf->spi_t.cdiv, cyrf->spi_t.select, cyrf->spi_t.phase, cyrf->spi_t.cpol, cyrf->spi_t.cpha);
              Chip_Select3 = 1;                                                 /* bring CS high to de-activate SPI */
              SPI3CON = 0u;                                                     /* turn off the SPI module and reset it */
              SPI3BUF;                                                          /* clear the rx buffer by reading from it  */
              SPI3STATbits.SPIROV = 0;                                          /* clear the overflow bit  */
              break;

#ifndef LSM303M_USED
              case SPIDEV_4:
              Chip_Select4 = 1;                                                 /* bring CS high to de-activate SPI */
              Chip_Select_Direction4 = 0;                                       // Set CS# pin as Output
              SPI4_Init_Advanced(cyrf->spi_p->mode, cyrf->spi_t.dss, cyrf->spi_t.cdiv, cyrf->spi_t.select, cyrf->spi_t.phase, cyrf->spi_t.cpol, cyrf->spi_t.cpha);
              Chip_Select4 = 1;                                                 /* bring CS high to de-activate SPI */
              SPI4CON = 0u;                                                     /* turn off the SPI module and reset it */
              SPI4BUF;                                                          /* clear the rx buffer by reading from it  */
              SPI4STATbits.SPIROV = 0;                                          /* clear the overflow bit  */
              break;
#endif
              default:
              break;
         }
         ticks2Now = -1;                                                        /* on first pass initialise the timer */
         calculateTick2Now( &ticks2Now, &cyrf->tickLast );                      /* ticksNow also initialised at zero */
         cyrf->spi_t.status = SPITransDone;
         break;

         case SPITransDone:
         calculateTick2Now( &ticks2Now, &cyrf->tickLast );                      /* Reset the CYRF6936 chip (busy waiting) */
         if (ticks2Now >= CYRF6936_INIT_DLY1)
         {
            switch (cyrf->spi_p->spiDevNo)
            {
               case SPIDEV_1:
               SPI1CONbits.ON = 1;                                              /* turn on SPI 1  */
               Chip_Select1 = 0;                                                /* bring CS low to activate SPI */
               break;

               case SPIDEV_2:
               SPI2CONbits.ON = 1;                                              /* turn on SPI 2  */
               Chip_Select2 = 0;                                                /* bring CS high to de-activate SPI */
               break;

               case SPIDEV_3:
               SPI3CONbits.ON = 1;                                              /* turn on SPI 3  */
               Chip_Select3 = 0;                                                /* bring CS high to de-activate SPI */
               break;

#ifndef LSM303M_USED
               case SPIDEV_4:
               SPI4CONbits.ON = 1;                                              /* turn on SPI 4  */
               Chip_Select4 = 0;                                                /* bring CS high to de-activate SPI */
               break;
#endif

               default:
               break;
            }
            cyrf->tickLast = CP0_GET(CP0_COUNT);                                 /* get new time reference to wait for the next time */
            cyrf->status = CYRF6936_WAIT_FOR_ENABLE;
         }
         break;

         case CYRF6936_WAIT_FOR_ENABLE:
         calculateTick2Now( &ticks2Now, &cyrf->tickLast );
         if (ticks2Now >= CYRF6936_INIT_DLY2)
         {
            cyrf->status = CYRF6936_GET_MFG_ID;                                 /* Get the MFG ID */
         }
         break;

         case CYRF6936_GET_MFG_ID:
         cyrf->buffer_idx = 0u;
         cyrf6936_write_block(cyrf, (uint8_t) CYRF_MFG_ID, &sendThis, numBytes2Send);
         cyrf->initStat = 0u;
         break;
     }

     default:
     cyrf->initStat = 0u;
     break;
  }
}

/*-----------------------------------------------------------------------------
 *  cyrf6936_write_chan_sop_data_crc(): Set the channel, SOP code,  
 *                                      DATA code and the CRC seed 
 *  Parameters: Cyrf6936_t *cyrf, const uint8_t chan, const uint8_t sop_code[], 
 *              const uint8_t data_code[], const uint16_t crc_seed
 *
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t cyrf6936_write_chan_sop_data_crc(Cyrf6936_t *cyrf, const uint8_t chan, const uint8_t sop_code[], const uint8_t data_code[], const uint16_t crc_seed)
{
  uint8_t i;

  if (cyrf->status != CYRF6936_IDLE)                                            /* Check if the cyrf6936 isn't busy */
  {
    return false;
  }

  cyrf->status = CYRF6936_CHAN_SOP_DATA_CRC;                                    /* Set the status  */

  cyrf->buffer[0u] = crc_seed & 0xFFu;                                          /* Copy the CRC  */
  cyrf->buffer[1u] = (crc_seed >> 8u) & 0xFFu;

  for (i = 0u; i < 8u; i++)                                                     /* Copy the SOP code  */
  {
    cyrf->buffer[i + 2u] = sop_code[i];
  }

  for (i = 0u; i < 16u; i++)                                                    /* Copy the DATA code */
  {
    cyrf->buffer[i + 10u] = data_code[i];
  }
  cyrf->buffer[26u] = chan;                                                     /* Copy the channel  */

  cyrf->buffer_idx = 0;                                                         /* Try to write the CRC LSB (not sure why as its in the handler but it was in original so maybe needs this twice ???? */
  cyrf6936_write_block(cyrf,(uint8_t) CYRF_CRC_SEED_LSB, &cyrf->buffer[0u],1u);          /* write a byte to the SPI module ???? the 26 bytes from the buffer ???  */
  cyrf->status = CYRF6936_CHAN_SOP_DATA_CRC;                                    /* now call the handler */
  
  return true;
}

/*-----------------------------------------------------------------------------
 *  cyrf6936_read_block(): Read multiple bytes from a register 
 *                                       
 *  Parameters: Cyrf6936_t *cyrf, const uint8_t addr, const uint8_t length
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t cyrf6936_read_block(Cyrf6936_t *cyrf, const uint8_t addr, const uint8_t length)
{
  uint16_t char2rcv;
  char buffr=0;
  uint8_t ret=0u;

  if (cyrf->spi_t.status != SPITransDone)
  {
    return FALSE;
  }

  /* Set the buffer and commit the transaction */
  cyrf->spi_t.output_length = 1;
  cyrf->spi_t.input_length = length + 1;
  cyrf->output_buf[0] = addr;

  // Submit the transaction
  if ((sizeof(cyrf->input_buf) >= cyrf->spi_t.input_length) && (cyrf != NULL))
  {
      cyrf->output_buf[0u] |= CYRF_DIR;                                         /* set the read bit (as per the cyrf protocol) */
      if(cyrf->spi_t.input_length >= 1u)
      {
         /* reg |= 0x40u;                                                       set the address auto inc. bit (not in protocol) } */
        switch (cyrf->spi_p->spiDevNo)
        {
           case SPIDEV_1:
           Chip_Select1 = 0;                                                    /* bring CS low to activate SPI */
           SPI1_Write((uint32_t)cyrf->output_buf[0u]);
           for(char2rcv = 0u; char2rcv != cyrf->spi_t.input_length; ++char2rcv)
           {
              cyrf->input_buf[char2rcv] = SPI1_Read(buffr);                     /* read data from spi  */
           }
           Chip_Select1 = 1;                                                    /* bring CS high to de-activate SPI */
           break;

           case SPIDEV_2:
           Chip_Select2 = 0;                                                    /* bring CS low to activate SPI */
           SPI2_Write((uint32_t)cyrf->output_buf[0u]);
           for(char2rcv = 0u; char2rcv != cyrf->spi_t.input_length; ++char2rcv)
           {
              cyrf->input_buf[char2rcv] = SPI2_Read(buffr);                     /* read data from spi  */
           }
           Chip_Select2 = 1;                                                    /* bring CS high to de-activate SPI */
           break;
           
           case SPIDEV_3:
           Chip_Select3 = 0;                                                    /* bring CS low to activate SPI */
           SPI3_Write((uint32_t)cyrf->output_buf[0u]);
           for(char2rcv = 0u; char2rcv != cyrf->spi_t.input_length; ++char2rcv)
           {
              cyrf->input_buf[char2rcv] = SPI3_Read(buffr);                     /* read data from spi  */
           }
           Chip_Select3 = 1;                                                    /* bring CS high to de-activate SPI */
           break;
#ifndef LSM303M_USED           
           case SPIDEV_4:
           Chip_Select4 = 0;                                                    /* bring CS low to activate SPI */
           SPI4_Write((uint32_t)cyrf->output_buf[0u]);
           for(char2rcv = 0u; char2rcv != cyrf->spi_t.input_length; ++char2rcv)
           {
              cyrf->input_buf[char2rcv] = SPI4_Read(buffr);                     /* read data from spi  */
           }
           Chip_Select4 = 1;                                                    /* bring CS high to de-activate SPI */
           break;
#endif           
           default:
           break;
         }
         ret = 1u;
      }
   }
   else { /*just for sanity */ }
   return ret;
}

/*-----------------------------------------------------------------------------
 *  cyrf6936_read_rx_irq_status_packet(): Read the RX IRQ status register, 
 *                                        the rx status register and the rx packet
 *  Parameters: Cyrf6936_t *cyrf
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t cyrf6936_read_rx_irq_status_packet(Cyrf6936_t *cyrf)
{

  if (cyrf->status != CYRF6936_IDLE)                                            /* Check if the cyrf6936 isn't busy */
  {
    return false;
  }
  cyrf->status = CYRF6936_RX_IRQ_STATUS_PACKET;                                 /* Set the status */

  cyrf->buffer_idx = 0;                                                         /* Try to read the RX status */
  cyrf6936_read_block(cyrf, CYRF_RX_IRQ_STATUS, 1u);

  return true;
}

/*-----------------------------------------------------------------------------
 *  cyrf6936_send_packet(): Send a packet with a certain length 
 *                     
 *  Parameters: Cyrf6936_t *cyrf, const uint8_t dataV[], const uint8_t length
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t cyrf6936_send_packet(Cyrf6936_t *cyrf, const uint8_t dataV[], const uint8_t length)
{
  int16_t i;

  if (cyrf->status != CYRF6936_IDLE)                                            /* Check if the cyrf6936 isn't busy */
  {
    return false;
  }

  cyrf->buffer[0u] = length;
  for (i = 0u; i < length; i++) 
  {
    cyrf->buffer[i + 1u] = dataV[i];
  }
  cyrf->buffer_idx = 0;                                                         /* Try to set the packet length */
  cyrf6936_write_block(cyrf, (uint8_t) CYRF_TX_LENGTH, &cyrf->buffer[0u],1u);
  cyrf->status = CYRF6936_SEND;                                                 /* Set the status */
  return true;
}

/*-----------------------------------------------------------------------------
 *  cyrf6936_event():  The on event call for the CYRF6936 chip (may be 
 *                     called in Interrupt timed poll)
 *  Parameters: Cyrf6936_t *cyrf
 *  Return: void
 *----------------------------------------------------------------------------*/
void cyrf6936_event(Cyrf6936_t *cyrf)
{
  int16_t i;
  uint8_t writeThis;

  //if (cyrf->status == CYRF6936_UNINIT)
  if (cyrf->status < CYRF6936_GET_MFG_ID)                                       // Check if cyrf is initialized
  {
    return;
  }

  if (cyrf->spi_t.status == SPITransPending || cyrf->spi_t.status == SPITransRunning)   // Check if there is still a transaction in progress
  {
    return;
  }

  switch (cyrf->status)                                                         /* Check the status of the cyrf */
  {

    case CYRF6936_GET_MFG_ID:                                                   /* Getting the MFG id */
    if (cyrf->spi_t.status != SPITransFailed)                                   // When the last transaction isn't failed send the next
    {
        cyrf->buffer_idx++;
    }
    cyrf->spi_t.status = SPITransDone;

    switch (cyrf->buffer_idx)                                                   // Switch for the different states
    {
        case 0:
        writeThis = 0xFFu;
        cyrf6936_write_block(cyrf, (uint8_t) CYRF_MFG_ID, &writeThis, 1u);
        break;
        
        case 1:
        cyrf6936_read_block(cyrf, CYRF_MFG_ID, 6u);
        break;
        
        case 2:                                                                 // Copy the MFG id
        for (i = 0; i < 6; i++)
        {
            cyrf->mfg_id[i] = cyrf->input_buf[i + 1];
        }
        writeThis = 0x00u;
        cyrf6936_write_block(cyrf, (uint8_t) CYRF_MFG_ID, &writeThis, 1u);
        break;
        
        default:
        cyrf->status = CYRF6936_IDLE;
        break;
    }
    break;

    case CYRF6936_MULTIWRITE:                                                   /* Do a multi write */
    if (cyrf->spi_t.status != SPITransFailed)                                   // When the last transaction isn't failed send the next
    {
        cyrf->buffer_idx++;
    }
    cyrf->spi_t.status = SPITransDone;
    if (cyrf->buffer_idx == cyrf->buffer_length)                                // When we are done writing
    {
        cyrf->status = CYRF6936_IDLE;
        break;
    }
    // Write the next register from the buffer
    // this was what was originally here,,, not sure what he meant it gives a compiler warning !! ------ cyrf6936_write_block(cyrf,((uint8_t ( *)[2u])cyrf->buffer)[cyrf->buffer_idx][0u], ((uint8_t ( *)[2u])cyrf->buffer)[cyrf->buffer_idx][1u],1u);
    cyrf6936_write_block(cyrf,(uint8_t)cyrf->buffer[cyrf->buffer_idx], (uint8_t *)cyrf->buffer[cyrf->buffer_idx+1u],1u);
    break;

    case CYRF6936_DATA_CODE:                                                    /* Do a write of the data code */
    break;

    case CYRF6936_CHAN_SOP_DATA_CRC:                                            /* Do a write of channel, sop, data and crc */
    if (cyrf->spi_t.status != SPITransFailed)                                   // When the last transaction isn't failed send the next
    {
      cyrf->buffer_idx++;
    }
    cyrf->spi_t.status = SPITransDone;

    switch (cyrf->buffer_idx)                                                   // Switch for the different states
    {
        case 0u:                                                                // Write the CRC LSB
        cyrf6936_write_block(cyrf, (uint8_t)CYRF_CRC_SEED_LSB, &cyrf->buffer[0u], 1u);
        break;
        
        case 1u:                                                                // Write the CRC MSB
        cyrf6936_write_block(cyrf, (uint8_t)CYRF_CRC_SEED_MSB, &cyrf->buffer[1u], 1u);
        break;
        
        case 2u:                                                                // Write the SOP code
        cyrf6936_write_block(cyrf, (uint8_t)CYRF_SOP_CODE, &(cyrf->buffer[2u]), 8u);
        break;
        
        case 3u:                                                                // Write the DATA code
        cyrf6936_write_block(cyrf, (uint8_t)CYRF_DATA_CODE, &(cyrf->buffer[10u]), 16u);
        break;
        
        case 4u:                                                                // Write the Channel
        cyrf6936_write_block(cyrf, (uint8_t)CYRF_CHANNEL, &cyrf->buffer[26u], 1u);
        break;
        
        default:
        cyrf->status = CYRF6936_IDLE;
        break;
    }
    break;

    /* Do a read of the receive irq status, receive status and the receive packet */
    case CYRF6936_RX_IRQ_STATUS_PACKET:
    if (cyrf->spi_t.status != SPITransFailed)                                   // When the last transaction isn't failed send the next
    {
       cyrf->buffer_idx++;
    }
    cyrf->spi_t.status = SPITransDone;

    switch (cyrf->buffer_idx)       // Switch for the different states
    {
        case 0: // Read the receive IRQ status
        cyrf6936_read_block(cyrf, CYRF_RX_IRQ_STATUS, 1u);
        break;

        case 1: // Read the send IRQ status
        cyrf->rx_irq_status = cyrf->input_buf[1u];
        cyrf6936_read_block(cyrf, CYRF_TX_IRQ_STATUS, 1u);
        break;
        
        case 2: // Read the receive status
        cyrf->tx_irq_status = cyrf->input_buf[1u];
        cyrf6936_read_block(cyrf, CYRF_RX_STATUS, 1u);
        break;
        
        case 3: // Set the packet length
        cyrf->rx_status = cyrf->input_buf[1];
        cyrf6936_read_block(cyrf, CYRF_RX_COUNT, 1u);
        break;
        
        case 4: // Read the receive packet
        cyrf->rx_count = cyrf->input_buf[1];
        cyrf6936_read_block(cyrf, CYRF_RX_BUFFER, 16u);
        break;
        
        default:
        for (i = 0; i < 16; i++)                                                // Copy the receive packet
        {
            cyrf->rx_packet[i] = cyrf->input_buf[i + 1u];
        }
        cyrf->has_irq = TRUE;
        cyrf->status = CYRF6936_IDLE;
        break;
    }
    break;

    case CYRF6936_SEND:                                                         /* The CYRF6936 is busy sending a packet */
    if (cyrf->spi_t.status != SPITransFailed)                                   // When the last transaction isn't failed send the next
    {
      cyrf->buffer_idx++;
    }
    cyrf->spi_t.status = SPITransDone;

    switch (cyrf->buffer_idx)                                                   // Switch for the different states
    {
        case 0: // Set the packet length
        cyrf6936_write_block(cyrf, (uint8_t) CYRF_TX_LENGTH, &cyrf->buffer[0],1u);
        break;
        
        case 1: // Clear the TX buffer
        writeThis = CYRF_TX_CLR;
        cyrf6936_write_block(cyrf, (uint8_t) CYRF_TX_CTRL, &writeThis,1u);
        break;
        
        case 2: // Write the send packet
        cyrf6936_write_block(cyrf, (uint8_t) CYRF_TX_BUFFER, &cyrf->buffer[1u], 16u);
        break;
        
        case 3: // Send the packet
        writeThis = CYRF_TX_GO | CYRF_TXC_IRQEN | CYRF_TXE_IRQEN;
        cyrf6936_write_block(cyrf, (uint8_t) CYRF_TX_CTRL, &writeThis, 1u);
        break;
        
        default:
        cyrf->status = CYRF6936_IDLE;
        break;
    }
    break;

    default:                                                                    /* This should not happen */
    break;
  }
}

/*-----------------------------------------------------------------------------
 *  cyrf6936_multi_write():  Write to multiple registers one byte
 *
 *  Parameters: Cyrf6936_t *cyrf, uint8_t dataV[][2u], uint8_t length
 *  Return: uint8_t
 *----------------------------------------------------------------------------*/
uint8_t cyrf6936_multi_write(Cyrf6936_t *cyrf, uint8_t dataV[][2u], uint8_t length)
{
  uint8_t i;

  if (cyrf->status != CYRF6936_IDLE)                                            /* Check if the cyrf6936 isn't busy */
  {
    return FALSE;
  }
  cyrf->buffer_length = length;                                                 /* Set the multi write */
  cyrf->buffer_idx = 0u;

  for (i = 0u; i < length; i++)                                                 // Copy the buffer
  {
    cyrf->buffer[i * 2u] = dataV[i][0u];
    cyrf->buffer[i * 2u + 1u] = dataV[i][1u];
  }

  if (length > 0u)                                                              /* Write the first regiter */
  {
    cyrf6936_write_block(cyrf, dataV[0u][0u], &dataV[0u][1u], 1u);
  }
  cyrf->status = CYRF6936_MULTIWRITE;                                           // Set the status
  return TRUE;
}
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif 