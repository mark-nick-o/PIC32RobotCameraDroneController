#ifndef FLASH__H__
#define FLASH__H__                                                      // the flash module allocates a page of flash and provides read/write accesss
#include <stdint.h>
#define PAGE_SIZE 4096u                                                 // size of a page, in bytes
#define PAGE_WORDS (PAGE_SIZE/4u)                                       // size of a page, in 4-byte words

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef enum
{
  NV_IDLE,
  NV_WRITE_START,
  NV_WRITE_WAIT,
  NV_WRITE_COMPLETE
} NVRAM_State_e;

// erases the flash page by setting all bits to 1s
extern void nvflash_erase(NVRAM_State_e nv_state);
// writes the 0s of a 4-byte word
extern void nvflash_write_word(uint32_t indexV, uint32_t dataV, NVRAM_State_e nv_state);
// reads a word from flash unsigned
extern uint32_t nvflash_read_word(uint32_t index);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif