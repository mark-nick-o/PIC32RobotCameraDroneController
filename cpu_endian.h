#ifndef _CPU_ENDIAN_H
#define _CPU_ENDIAN_H

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//Byte order conversion functions
extern uint16_t swapInt16func(uint16_t value);
extern uint32_t swapInt32func(uint32_t value);
extern uint64_t swapInt64func(uint64_t value);

//Bit reversal functions
extern uint8_t reverseInt4(uint8_t value);
extern uint8_t reverseInt8(uint8_t value);
extern uint16_t reverseInt16(uint16_t value);
extern uint32_t reverseInt32(uint32_t value);
extern uint64_t reverseInt64(uint64_t value);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif