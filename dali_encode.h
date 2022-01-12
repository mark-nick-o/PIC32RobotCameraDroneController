#ifndef __DALI_ENCODE__
#define __DALI_ENCODE__
//  dali_encode.h - dali "native" operations.
//
// * Ported by (C) 2020 A C P Avaiation Walkerburn Scotland
//
#include <stdint.h>
#include "dali_codes.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef uint16_t word;                                                          // these are used by the dali libraries you could consider move to definitions.h
typedef unsigned char  byte;
// typedef byte bool;

#define _ERR_WRONG_ADDRESS_    -1
#define _ERR_OK_                0
#define _ERR_WRONG_COMMAND_    -2
#define _ERR_RESERVED_COMMAND_ -3
#define _MODE_SIMPLE_          40
#define _MODE_REPEAT_TWICE_    41
#define _MODE_QUERY_           42

#define dali_command_initialize_broadcast(output) dali_special_command(output, INITIALIZE, 0xFF)
#define dali_command_randomize(output) dali_special_command(output, RANDOMIZE, 0)
#define dali_command_terminate(output) dali_special_command(output, TERMINATE, 0)
#define dali_command_off(output,address) dali_slave_command(output,address,0x00)
#define MAX_BUFFER_LENGTH 64
#define _ERR_PARSE_ERROR_ -253
#define _ERR_BUFFER_FULL_ -254
#define _ERR_PARAMETER_MISSING_ -255
#define _ERR_UNIMPLEMENTED_ -256
#define _ERR_ACK 9000
#define _ERR_NACK 9001
#define pgm_read_byte(a) a

typedef enum {RANDOMIZE, INITIALIZE, TERMINATE, COMPARE, WITHDRAW, PROGRAM_SHORT_ADDRESS, VERIFY_SHORT_ADDRESS, QUERY_SHORT_ADDRESS, STORE_DTR, SEARCH_ADDRESS_H, SEARCH_ADDRESS_M, SEARCH_ADDRESS_L, PHYSICAL_SELECTION, ENABLE_DEVICE_TYPE} special_command_type;

extern unsigned char arc_device[64u];
extern unsigned char arc_group[16u];

extern void dali_init_device();
extern void dali_init_group();
extern int16_t dali_slave_direct_arc(word *output, byte address, byte brightness);
extern int16_t dali_group_direct_arc(word *output, byte address, byte brightness);
extern int16_t dali_broadcast_direct_arc(word* output, byte brightness);
extern int16_t dali_slave_command(word *output, byte address, byte command);
extern int16_t dali_group_command(word *output, byte address, byte command);
extern int16_t dali_broadcast_command(word* output, byte command);
extern int16_t dali_slave_command_with_param(word *output, byte address, byte command, byte param);
extern int16_t dali_group_command_with_param(word *output, byte address, byte command, byte param);
extern int16_t dali_broadcast_command_with_param(word* output, byte command, byte param);
extern int16_t dali_special_command(word *output, special_command_type command, byte dataVal);

char nibble_to_ascii(uint8_t nibble);
int16_t decode_command_to_frame(const char* token, word* output);
int16_t parse_int(const char* string, int16_t* integer);

#ifdef __cplusplus
}
#endif

#endif