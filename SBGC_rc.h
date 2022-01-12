/*
  SBGC_rc.h :  SimpleBGC Serial API  library - RC (Remote Control) definitions

  More info: http://www.basecamelectronics.com/serialapi/

  Copyright (c) 2014-2015 Aleksei Moskalenko
  Ported and expanded by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
  
  All rights reserved.
        See license info in the SBGC.h
*/
#ifndef  __SBGC_rc__
#define  __SBGC_rc__

#ifdef __cplusplus
 extern "C" {
#endif

// RC channels used in the SBGC controller
#define SBGC_RC_NUM_CHANNELS 6U // ROLL, PITCH, YAW, CMD, EXT_ROLL, EXT_PITCH

// Hardware RC inputs, as labeled on the board
#define SBGC_RC_INPUT_NO 0U
#define SBGC_RC_INPUT_ROLL 1U
#define SBGC_RC_INPUT_PITCH 2U
#define SBGC_RC_INPUT_EXT_ROLL 3U
#define SBGC_RC_INPUT_EXT_PITCH 4U
#define SBGC_RC_INPUT_YAW 5U                                                    // not connected in 1.0 board

// Analog inputs (board v.3.0)   Not relevant in PIC this is for STM32
#define SBGC_RC_INPUT_ADC1 33U
#define SBGC_RC_INPUT_ADC2 34U
#define SBGC_RC_INPUT_ADC3 35U

// Bit indicates input is in analog mode
#define SBGC_RC_INPUT_ANALOG_BIT (1u<<5u) // 32

// Bit indicates input is a virtual channel
#define SBGC_RC_INPUT_VIRT_BIT (1u<<6u) // 64

// Bit indicates input is a API virtual channel
#define SBGC_RC_INPUT_API_VIRT_BIT (1u<<7u) // 128

// Mask to separate input channel number from input mode
#define SBGC_RC_INPUT_CH_MASK ((1u<<0u) | (1u<<1u) | (1u<<2u) | (1u<<3u) | (1u<<4u))
#define SBGC_RC_INPUT_MODE_MASK ((1u<<5u) | (1u<<6u) | (1u<<7u))

// Number of virtual channels for RC serial input (s-bus, spektrum, Sum-PPM)
#define SBGC_VIRT_NUM_CHANNELS 32U

// Number of virtual channels for API serial input
#define SBGC_API_VIRT_NUM_CHANNELS 32U

// Normal range of RC signal. Real signal may go outside this range
#define SBGC_RC_MIN_VAL -500
#define SBGC_RC_MAX_VAL 500

// Value to encode 'RC no signal'
#define SBGC_RC_UNDEF -10000                                                    // Low speed mode
#define SBGC_RC_HIGH_UNDEF -32767                                               // High speed mode

#ifdef __cplusplus
}
#endif

#endif // __SBGC_rc__