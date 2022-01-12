#ifndef __pic32MX795F512_flash_mem__
#define __pic32MX795F512_flash_mem__

#include "definitions.h"
#include "io.h"                                                                 /* for time differnce function */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* physical memory definitions */
#define PIC32_FLASH_START_ADDR 0x9d001000L                                      /* define the start address for the memory */
#define PIC32_FLASH_MAX_DATA_OFFSET 4096L                                       /* the number of bytes we can store from the start address in flash (4 pages)*/
#define PIC32_BYTES_PER_PAGE 512L                                               /* number of bytes per page 128 words or 512 bytes */
#define PIC32_WORDS_PER_PAGE (PIC32_BYTES_PER_PAGE / 4.0L)                      /* number of words per page */
#define PIC32_WORD_LEN 4L                                                       /* pic32 word length 4 bytes long */
#define ST_REC_TIM_LEN (PIC32_WORD_LEN*2L)                                      /* length of each streaming time record in bytes */

/* data storage address definitions */
#define FLASH_ADDR_OFFSET_ST1_REC_TIM 0L                                        /* start address for storing in flash the stream 1 64 bit recording time */
#define FLASH_ADDR_OFFSET_ST2_REC_TIM FLASH_ADDR_OFFSET_ST1_REC_TIM+ST_REC_TIM_LEN  /* start address for storing in flash the stream 2 64 bit recording time */
#define FLASH_ADDR_OFFSET_ST3_REC_TIM FLASH_ADDR_OFFSET_ST2_REC_TIM+ST_REC_TIM_LEN  /* start address for storing in flash the stream 3 64 bit recording time */
#define FLASH_ADDR_OFFSET_ST4_REC_TIM FLASH_ADDR_OFFSET_ST3_REC_TIM+ST_REC_TIM_LEN  /* start address for storing in flash the stream 4 64 bit recording time */
#define ST_REC_TIM_ITEMS 4L                                                     /* number of items we wrote */
#define FLASH_ADDR_OFFSET_ST_CRC FLASH_ADDR_OFFSET_ST4_REC_TIM+ST_REC_TIM_LEN   /* start address for the 32bit CRC */

/* functionality defines */
#define FLASH_NEED_DELAY                                                        /* feature to delay the flash read for a page check */
#if defined(FLASH_NEED_DELAY)
#define FLASH_DELAY_MS 50L                                                      /* delay between read back of each word when reading back data written by the page write */
#endif

#define FLASH64LE(a, p, o) \
   ((uint8_t *)(p))[0u+o] = ((uint64_t)(a) >> 0L) & 0xFFL, \
   ((uint8_t *)(p))[1u+o] = ((uint64_t)(a) >> 8L) & 0xFFL, \
   ((uint8_t *)(p))[2u+o] = ((uint64_t)(a) >> 16L) & 0xFFL, \
   ((uint8_t *)(p))[3u+o] = ((uint64_t)(a) >> 24L) & 0xFFL, \
   ((uint8_t *)(p))[4u+o] = ((uint64_t)(a) >> 32L) & 0xFFL, \
   ((uint8_t *)(p))[5u+o] = ((uint64_t)(a) >> 40L) & 0xFFL, \
   ((uint8_t *)(p))[6u+o] = ((uint64_t)(a) >> 48L) & 0xFFL, \
   ((uint8_t *)(p))[7u+o] = ((uint64_t)(a) >> 56L) & 0xFFL                      /* Store unaligned 64-bit integer (little-endian encoding) */

#define FLASH64BE(a, p, o) \
   ((uint8_t *)(p))[0u+o] = ((uint64_t)(a) >> 56L) & 0xFFL, \
   ((uint8_t *)(p))[1u+o] = ((uint64_t)(a) >> 48L) & 0xFFL, \
   ((uint8_t *)(p))[2u+o] = ((uint64_t)(a) >> 40L) & 0xFFL, \
   ((uint8_t *)(p))[3u+o] = ((uint64_t)(a) >> 32L) & 0xFFL, \
   ((uint8_t *)(p))[4u+o] = ((uint64_t)(a) >> 24L) & 0xFFL, \
   ((uint8_t *)(p))[5u+o] = ((uint64_t)(a) >> 16L) & 0xFFL, \
   ((uint8_t *)(p))[6u+o] = ((uint64_t)(a) >> 8L) & 0xFFL, \
   ((uint8_t *)(p))[7u+o] = ((uint64_t)(a) >> 0L) & 0xFFL                       /* Store unaligned 64-bit integer (big-endian encoding) */

#define FREAD32LE(p,o) ( \
   ((uint32_t)(((uint8_t *)(p))[0+o]) << 0u) | \
   ((uint32_t)(((uint8_t *)(p))[1+o]) << 8u) | \
   ((uint32_t)(((uint8_t *)(p))[2+o]) << 16u) | \
   ((uint32_t)(((uint8_t *)(p))[3+o]) << 24u))                                  /* Load unaligned 32-bit integer (little-endian encoding) */

#define FREAD32BE(p,o) ( \
   ((uint32_t)(((uint8_t *)(p))[0+o]) << 24u) | \
   ((uint32_t)(((uint8_t *)(p))[1+o]) << 16u) | \
   ((uint32_t)(((uint8_t *)(p))[2+o]) << 8u) | \
   ((uint32_t)(((uint8_t *)(p))[3+o]) << 0u))                                   /* Load unaligned 32-bit integer (big-endian encoding) */
   
#define FREAD64LE(p,o) ( \
   ((uint64_t)(((uint8_t *)(p))[0u+o]) << 0u) | \
   ((uint64_t)(((uint8_t *)(p))[1u+o]) << 8u) | \
   ((uint64_t)(((uint8_t *)(p))[2u+o]) << 16u) | \
   ((uint64_t)(((uint8_t *)(p))[3u+o]) << 24u) | \
   ((uint64_t)(((uint8_t *)(p))[4u+o]) << 32u) | \
   ((uint64_t)(((uint8_t *)(p))[5u+o]) << 40u) | \
   ((uint64_t)(((uint8_t *)(p))[6u+o]) << 48u) | \
   ((uint64_t)(((uint8_t *)(p))[7u+o]) << 56u))                                 /* Load unaligned 64-bit number from flash (little-endian encoding) */


#define FREAD64BE(p,o) ( \
   ((uint64_t)(((uint8_t *)(p))[0u+o]) << 56u) | \
   ((uint64_t)(((uint8_t *)(p))[1u+o]) << 48u) | \
   ((uint64_t)(((uint8_t *)(p))[2u+o]) << 40u) | \
   ((uint64_t)(((uint8_t *)(p))[3u+o]) << 32u) | \
   ((uint64_t)(((uint8_t *)(p))[4u+o]) << 24u) | \
   ((uint64_t)(((uint8_t *)(p))[5u+o]) << 16u) | \
   ((uint64_t)(((uint8_t *)(p))[6u+o]) << 8u) | \
   ((uint64_t)(((uint8_t *)(p))[7u+o]) << 0u))                                  /* Load unaligned 64-bit integer (big-endian encoding) */
   
int8_t WriteWordToFlash( uint32_t offset, uint32_t wdata );                     /* write a single 32 bit word to flash memory */
#if defined(FLASH_NEED_DELAY)
int8_t WritePageToFlash( uint32_t offset, uint32_t *rdata, int64_t TimeDuration, uint64_t TimeDurationLast ); /* write a page and delay for every word read back but do not hang the CPU, to initialise this iterative function initially set TimeDuration to -1 in code */
#else
int8_t WritePageToFlash( uint32_t offset, uint32_t *rdata);                     /* write a page to flash and read back immediately */
#endif
int8_t ErasePageFromFlash( uint32_t offset );                                   /* erase a page 128 words from the flash memory */
int8_t saveEncoderRecordingTimetoFlash();                                       /* write the encoder recording times to the flash memory called by a polled timer as often as you need to store values */
int8_t readEncoderRecordingTimefromFlash();                                     /* read the encoder recording times from the flash memory and recall them into the global volatiles upon boot-up */

int8_t WriteWordToFlash( uint32_t offset, uint32_t wdata )
{
   uint32_t *pAddr;                                                             /* pointer to the flash memory address */
   
   if (offset >= PIC32_FLASH_MAX_DATA_OFFSET)                                   /* out of flash storage range address sepcified then exit function */
      return 0;
      
   Flash_Write_Word((uint32_t) PIC32_FLASH_START_ADDR + offset, wdata);         /* write the data to the address in flash memory so it can be recalled on reboot */
   asm nop;                                                                     /* delay 1 tick */
   pAddr = ((uint32_t*) (PIC32_FLASH_START_ADDR + offset));                     /* point to the flash memory address we wrote to */
   
   if (wdata == *pAddr)                                                         /* check it wrote correctly */
   {
     return 1;                                                                  /* success */
   }
   else
   {
     return -1;                                                                 /* failure */
   }
}


#if defined(FLASH_NEED_DELAY)
int8_t WritePageToFlash( uint32_t offset, uint32_t *rdata, int64_t TimeDuration, uint64_t TimeDurationLast ) /* to initialise set TimeDuration to -1 in code */
#else
int8_t WritePageToFlash( uint32_t offset, uint32_t *rdata)
#endif
{
   uint32_t *pAddr;                                                             /* pointer to the flash memory address */
   int8_t word=0u;
   
   if (offset+PIC32_BYTES_PER_PAGE >= PIC32_FLASH_MAX_DATA_OFFSET)              /* out of flash storage range address sepcified then exit function */
      return 0;

   Flash_Write_Row((uint32_t) PIC32_FLASH_START_ADDR + offset,(void*) rdata);   /* write the 128 32-bit words or 512 bytes of page data to the start address */
   asm nop;                                                                     /* delay 1 tick */
   pAddr = ((uint32_t*) (PIC32_FLASH_START_ADDR + offset));                     /* point to the flash memory address we wrote to */


#if defined(FLASH_NEED_DELAY)
  if (word < PIC32_WORDS_PER_PAGE)                                              /* have you checked all words on the page */
  {
    if (*rdata != *pAddr)                                                       /* mismatch between value and what was written to the flash memory */
    {
       return -1;                                                               /* return an error */
    }
    calculateTimeDiff( &TimeDuration, &TimeDurationLast );                        /* calculate the time difference since the last iterative call */
    if (TimeDuration >= FLASH_DELAY_MS)
    {
       word = ++word % INT32_MAX;                                               /* increment the word counter to the next item */
       pAddr++;                                                                 /* iterate to read the next word after FLASH_DELAY_MS */
       rdata++;
    }
  }
  else                                                                          /* we have now read back all the words */
  {
     return 1;                                                                  // return that the iteration has completed */
  }
#else
  for (word=0; word<PIC32_WORDS_PER_PAGE; word++)                               /* foreach word on the page */
  {
    if (*rdata != *pAddr)                                                       /* mismatch between value and what was written to the flash memory */
    {
       return -1;                                                               /* return an error */
    }
    pAddr++;                                                                    /* increment address pointer */
    rdata++;                                                                    /* increment the data stream */
    /* Delay_ms(50); not allowed in realtime if needed iterate function and delay via a tick counter */
  }
  
  return 1;                                                                     /* return success */
#endif
}

int8_t ErasePageFromFlash( uint32_t offset )
{

   if (offset+PIC32_BYTES_PER_PAGE >= PIC32_FLASH_MAX_DATA_OFFSET)              /* out of flash storage range address sepcified then exit function */
      return 0;

   Flash_Erase_Page((uint32_t) PIC32_FLASH_START_ADDR + offset);                /* erase 1 page (128 words 512 bytes) starting at the address plus offset */
   
   return 1;
}

#if ((CAMERA_TYPE == XS_CAM) && defined(CAMERA_TYPE))
int8_t saveEncoderRecordingTimetoFlash()
{
   int8_t returnCode=0;
   uint8_t byteStreamArray[FLASH_ADDR_OFFSET_ST_CRC+PIC32_WORD_LEN];            /* 8 bytes for each of the 4 floats  and then 4 bytes for the checksum */
   uint32_t crcResult;
   
   uint32_t wdata = (((uint32_t) g_XSVideoChan.videoRecCh1) & 0xFFFFL) ;        /* write the first data to flash and check its validity */
   returnCode=WriteWordToFlash( FLASH_ADDR_OFFSET_ST1_REC_TIM, wdata );
   if (returnCode == 1)                                                         /* written the data okay */
   {
      asm nop;
      wdata = ((((uint32_t) g_XSVideoChan.videoRecCh1)>>32L) & 0xFFFFL);
      returnCode=WriteWordToFlash( FLASH_ADDR_OFFSET_ST1_REC_TIM+PIC32_WORD_LEN, wdata );
      if (returnCode != 1)
         return returnCode;                                                     /* error writing data then exit */
   }
   else
   {
      return returnCode;                                                        /* error writing data then exit */
   }
   
   wdata = (((uint32_t) g_XSVideoChan.videoRecCh2) & 0xFFFFL) ;                 /* write the second data to flash and check its validity */
   returnCode=WriteWordToFlash( FLASH_ADDR_OFFSET_ST2_REC_TIM, wdata );
   if (returnCode == 1)
   {
      asm nop;
      wdata = ((((uint32_t) g_XSVideoChan.videoRecCh2)>>32L) & 0xFFFFL);
      returnCode=WriteWordToFlash( FLASH_ADDR_OFFSET_ST2_REC_TIM+PIC32_WORD_LEN, wdata );
      if (returnCode != 1)
         return returnCode;
   }
   else
   {
      return returnCode;
   }
   
   wdata = (((uint32_t) g_XSVideoChan.videoRecCh3) & 0xFFFFL) ;                 /* write the third data to flash and check its validity */
   returnCode=WriteWordToFlash( FLASH_ADDR_OFFSET_ST3_REC_TIM, wdata );
   if (returnCode == 1)
   {
      asm nop;
      wdata = ((((uint32_t) g_XSVideoChan.videoRecCh3)>>32L) & 0xFFFFL);
      returnCode=WriteWordToFlash( FLASH_ADDR_OFFSET_ST3_REC_TIM+PIC32_WORD_LEN, wdata );
      if (returnCode != 1)
         return returnCode;
   }
   else
   {
      return returnCode;
   }
   
   wdata = (((uint32_t) g_XSVideoChan.videoRecCh4) & 0xFFFFL) ;                 /* write the fourth data to flash and check its validity */
   returnCode=WriteWordToFlash( FLASH_ADDR_OFFSET_ST4_REC_TIM, wdata );
   if (returnCode == 1)
   {
      asm nop;
      wdata = ((((uint32_t) g_XSVideoChan.videoRecCh4)>>32L) & 0xFFFFL);
      returnCode=WriteWordToFlash( FLASH_ADDR_OFFSET_ST4_REC_TIM+PIC32_WORD_LEN, wdata );
      if (returnCode != 1)
         return returnCode;
   }
   else
   {
      return returnCode;
   }
   
#if defined(_CPU_BIG_ENDIAN)
   FLASH64BE(g_XSVideoChan.videoRecCh1,byteStreamArray,FLASH_ADDR_OFFSET_ST1_REC_TIM); /* unpack the 64 bit floating point recording time into a byte stream */
   FLASH64BE(g_XSVideoChan.videoRecCh2,byteStreamArray,FLASH_ADDR_OFFSET_ST2_REC_TIM);
   FLASH64BE(g_XSVideoChan.videoRecCh3,byteStreamArray,FLASH_ADDR_OFFSET_ST3_REC_TIM);
   FLASH64BE(g_XSVideoChan.videoRecCh4,byteStreamArray,FLASH_ADDR_OFFSET_ST4_REC_TIM);
#else
   FLASH64LE(g_XSVideoChan.videoRecCh1,byteStreamArray,FLASH_ADDR_OFFSET_ST1_REC_TIM);
   FLASH64LE(g_XSVideoChan.videoRecCh2,byteStreamArray,FLASH_ADDR_OFFSET_ST2_REC_TIM);
   FLASH64LE(g_XSVideoChan.videoRecCh3,byteStreamArray,FLASH_ADDR_OFFSET_ST3_REC_TIM);
   FLASH64LE(g_XSVideoChan.videoRecCh4,byteStreamArray,FLASH_ADDR_OFFSET_ST4_REC_TIM);
#endif
   crcResult=crc_slow(&byteStreamArray,(uint8_t)(ST_REC_TIM_ITEMS*ST_REC_TIM_LEN));  /* calculate the 32 bit CRC from the byte stream */
   return(WriteWordToFlash( FLASH_ADDR_OFFSET_ST_CRC, crcResult ));             /* write the CRC to flash and return what we got from the flash write if we got this far*/
   
}

int8_t readEncoderRecordingTimefromFlash()
{
   int8_t returnCode=0;
   uint8_t byteStreamArray[FLASH_ADDR_OFFSET_ST_CRC+PIC32_WORD_LEN];
   uint32_t crcResult;
   uint32_t crcStoredInFlash;
   int8_t byte;
   uint8_t *pAddr;
   
   pAddr = ((uint8_t*) (PIC32_FLASH_START_ADDR + FLASH_ADDR_OFFSET_ST1_REC_TIM)); /* point to the address in flash memory as a byte stream */
   
   for (byte=0; byte<(FLASH_ADDR_OFFSET_ST_CRC+PIC32_WORD_LEN); byte++)         /* foreach byte in the flash memory we want */
   {
      byteStreamArray[byte] = *pAddr;                                           /* read each byte from the flash to the byteStreamArray */
      pAddr++;                                                                  /* increment address pointer in flash to next byte value */
   }
   crcResult=crc_slow(&byteStreamArray,(uint8_t)(ST_REC_TIM_ITEMS*ST_REC_TIM_LEN));  /* calculate the 32 bit CRC from the byte stream from the flash minus the CRC */
#if defined(_CPU_BIG_ENDIAN)                                                    /* choose big endian or little endian */
   crcStoredInFlash = FREAD32BE(byteStreamArray,FLASH_ADDR_OFFSET_ST_CRC);      /* read 32 bits from the start of the CRC address into the stored variable big endian */
#else
   crcStoredInFlash = FREAD32LE(byteStreamArray,FLASH_ADDR_OFFSET_ST_CRC);      /* read 32 bits from the start of the CRC address into the stored variable little endian */
#endif
   if (crcStoredInFlash == crcResult)                                           /* the CRC stored matched the one we calculated so the data from flash is good */
   {
#if defined(_CPU_BIG_ENDIAN)                                                    /* choose big endian or little endian */
      g_XSVideoChan.videoRecCh1=FREAD64BE(byteStreamArray,FLASH_ADDR_OFFSET_ST1_REC_TIM);  /* set each video channel recording time to the previous value stored from flash in big endian */
      g_XSVideoChan.videoRecCh2=FREAD64BE(byteStreamArray,FLASH_ADDR_OFFSET_ST2_REC_TIM);
      g_XSVideoChan.videoRecCh3=FREAD64BE(byteStreamArray,FLASH_ADDR_OFFSET_ST3_REC_TIM);
      g_XSVideoChan.videoRecCh4=FREAD64BE(byteStreamArray,FLASH_ADDR_OFFSET_ST4_REC_TIM);
      returnCode=1;                                                             /* success */
#else
      g_XSVideoChan.videoRecCh1=FREAD64LE(byteStreamArray,FLASH_ADDR_OFFSET_ST1_REC_TIM);  /* set each video channel recording time to the previous value stored from flash in little endian */
      g_XSVideoChan.videoRecCh2=FREAD64LE(byteStreamArray,FLASH_ADDR_OFFSET_ST2_REC_TIM);
      g_XSVideoChan.videoRecCh3=FREAD64LE(byteStreamArray,FLASH_ADDR_OFFSET_ST3_REC_TIM);
      g_XSVideoChan.videoRecCh4=FREAD64LE(byteStreamArray,FLASH_ADDR_OFFSET_ST4_REC_TIM);
      returnCode=1;                                                             /* success */
#endif
   }
   return returnCode;
}
#endif /* end xs encoder function includes */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end the library */