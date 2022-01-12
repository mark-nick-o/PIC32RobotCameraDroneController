#include "definitions.h"
#if defined(NV_FLASH_USED)

#include <stdint.h>
#include "nv_flash.h"
#include "p32mx795f512l.h"

//#include <xc.h>
//#include <sys/kmem.h>                                                 // macros for converting between physical and virtual addresses
#define OP_ERASE_PAGE 4U                                                        // erase page operation, per NVMCONbits.NVMOP specification
#define OP_WRITE_WORD 1U                                                        // write word operation, per NVMCONbits.NVMOP specification
// Making the array const and initializing it to 0 ensures that the linker will
// store it in flash memory.
// Since one page is erased at a time, array must be a multiple of PAGE_SIZE bytes long.
// The aligned attribute ensures that the page falls at an address divisible by 4096.

//static const uint32_t nv_buf[PAGE_WORDS] __attribute__ ((__aligned__(PAGE_SIZE))); // = {0};
const unsigned long nv_buf[PAGE_WORDS] __attribute__ ((__aligned__(PAGE_SIZE)));
static void nvflash_op(uint8_t op, NVRAM_State_e nv_State);                     // perform a flash operation (op is NVMOP)
void nvflash_erase(NVRAM_State_e nv_state);
void nvflash_write_word(uint32_t indexV, uint32_t dataV, NVRAM_State_e nv_state);
uint32_t nvflash_read_word(uint32_t indexV);
/*-----------------------------------------------------------------------------
 *      nvflash_op(): perform a flash operation (op is NVMOP)  
 *
 *  Parameters: uint8_t op, NVRAM_State_e nv_State
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
static void nvflash_op(uint8_t op, NVRAM_State_e nv_State)                      // perform a flash operation (op is NVMOP)
{
   uint32_t ie=0UL;                                                             //= __builtin_disable_interrupts();
   switch (nv_State)
   {
       case NV_IDLE:                                                            // -------- No Action -----------------
       break;

       case NV_WRITE_START:                                                     // -------- Start Action --------------
       ie= DI();
       //NVMCONbits.NVMOP = op;                                              store the operation
       NVMCON = (((NVMCON & ~(_NVMCON_NVMOP_MASK<<_NVMCON_NVMOP_POSITION)) | ((op &_NVMCON_NVMOP_MASK) << _NVMCON_NVMOP_POSITION)) | (1u<<_NVMCON_WREN_POSITION)); // NVMOP=op store the operation WREN=1 enable writes to the WR bit
       //NVMCONbits.WREN = 1;                                                   // enable writes to the WR bit
       NVMKEY = 0xAA996655UL;                                                   // unlock step 1
       NVMKEY = 0x556699AAUL;                                                   // unlock step 2
       NVMCONSET = 0x8000U;                                                     // set the WR bit to begin the operation
       nv_State = NV_WRITE_WAIT;
       break;

       case NV_WRITE_WAIT:                                                      // -------- Wait for Action -------------
       if (((NVMCON&_NVMCON_WR_MASK)>>_NVMCON_WR_POSITION)==0u)
          nv_State = NV_WRITE_COMPLETE;                                          // now clear the WREN bit
       break;

       case NV_WRITE_COMPLETE:                                                  // ------- Complete and re-enable interrupts
       //NVMCONbits.WREN = 0;                                                   // disables writes to the WR bit
       NVMCON = (NVMCON & ~(1u<<_NVMCON_WREN_POSITION));
       if (ie & 0x1u)                                                           // re-enable interrupts if they had been disabled
       {
         asm ei;                                                                //__builtin_enable_interrupts();
         //asm ei;
       }
       nv_State = NV_IDLE;
       break;
   }

//}
}
/*-----------------------------------------------------------------------------
 *      nvflash_erase(): flash erase  
 *
 *  Parameters: NVRAM_State_e nv_state
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void nvflash_erase(NVRAM_State_e nv_state)
{                                                                               // erase the flash buffer. resets the memory to ones
        //NVMADDR = KVA_TO_PA(nv_buf);                                          // use the physical address of the buffer
        nvflash_op(OP_ERASE_PAGE,nv_state);
}
/*-----------------------------------------------------------------------------
 *      nvflash_write_word(): write word to flash   
 *
 *  Parameters: uint32_t indexV, uint32_t dataV, NVRAM_State_e nv_state
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void nvflash_write_word(uint32_t indexV, uint32_t dataV, NVRAM_State_e nv_state)  // writes a word to flash
{
        //NVMADDR = KVA_TO_PA((const uint32_t)(nv_buf + indexV));                                // physical address of flash to write to
        NVMDATA = dataV;                                                        // data to write
        nvflash_op(OP_WRITE_WORD, nv_state);
}
/*-----------------------------------------------------------------------------
 *      nvflash_read_word(): read word from flash  
 *
 *  Parameters: uint32_t indexV
 *
 *  Return:     uint32_t
 *----------------------------------------------------------------------------*/
uint32_t nvflash_read_word(uint32_t indexV)                                     // read a word from flash
{
        return nv_buf[indexV];
}
#endif