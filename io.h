#ifndef  __io__
#define  __io__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//    io.h : PIC32 input / output / port definitions
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
// Set up the IO PORTS for analogue input (AIN) digital input (DIN) and LED output (DOT)

/* -------- Interrupt Vector Table definition -------------------------------
   IPC0,12 are the interrupt priority SFRs
   IEC0,1,2 are 32 bit configurations to set the interrupts
   IFS0,1,2 are 32 bit representations of there state triggers */

#define COMPARE0_ENAB (1u<<0u)
#define INT0_ENAB (1u<<3u)
#define INT1_ENAB (1u<<7u)
#define INT2_ENAB (1u<<11u)
#define INT3_ENAB (1u<<15u)
#define INT4_ENAB (1u<<19u)
#define CN_ENAB (1u<<0u)

#define PRIO_COMPARE_MASK ~(0x0000001FUL << 0u)                                 /* set the mask to remove the bits before we set them according to prio and sub-prio */
#define SET_PRIO_COMPARE0_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC0=(((IPC0 & PRIO_COMPARE_MASK) | (prio<<2u)) | (subprio)); IEC0=(IEC0 | COMPARE0_ENAB); IFS0=(IFS0 & ~COMPARE0_ENAB); } }while(0)
#define CLR_PRIO_COMPARE0_INTER(prio,subprio) do{ IEC0=(IEC0 & ~(COMPARE0_ENAB)); IFS0=(IFS0 & ~COMPARE0_ENAB); } }while(0)

#define PRIO_CN_MASK ~(0x0000001FUL << 16u)                                     /* set the mask to remove the bits before we set them according to prio and sub-prio */
#define SET_PRIO_CN_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {asm di; IPC6=(((IPC6 & PRIO_CN_MASK) | (prio<<18u)) | (subprio<<16u)); IEC1=(IEC1 | CN_ENAB); IFS1=(IFS1 & ~CN_ENAB); asm ei; } }while(0)
#define CLR_PRIO_CN_INTER(prio,subprio) do{ asm di; IEC1=(IEC1 & ~(CN_ENAB)); IFS1=(IFS1 & ~CN_ENAB); asm ei; } }while(0)

#define PRIO_IO_MASK ~(0x0000001FUL << 24u)
#define SET_PRIO_IO0_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC0=(((IPC0 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u)); IEC0=(IEC0 | INT0_ENAB); IFS0=(IFS0 & ~INT0_ENAB); } }while(0)
#define CLR_PRIO_IO0_INTER(prio,subprio) do{ IEC0=(IEC0 & ~(INT0_ENAB)); IFS0=(IFS0 & ~INT0_ENAB); }while(0)
#define SET_PRIO_IO1_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC1=(((IPC1 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u)); IEC0=(IEC0 | INT1_ENAB); IFS0=(IFS0 & ~INT1_ENAB); } }while(0)
#define CLR_PRIO_IO1_INTER(prio,subprio) do{ IEC0=(IEC0 & ~(INT0_ENAB)); IFS0=(IFS0 & ~INT1_ENAB); }while(0)
#define SET_PRIO_IO2_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC2=(((IPC2 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u)); IEC0=(IEC0 | INT2_ENAB); IFS0=(IFS0 & ~INT2_ENAB); } }while(0)
#define CLR_PRIO_IO2_INTER(prio,subprio) do{ IEC0=(IEC0 & ~(INT0_ENAB)); IFS0=(IFS0 & ~INT2_ENAB); }while(0)
#define SET_PRIO_IO3_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC3=(((IPC3 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u)); IEC0=(IEC0 | INT3_ENAB); IFS0=(IFS0 & ~INT3_ENAB); } }while(0)
#define CLR_PRIO_IO3_INTER(prio,subprio) do{ IEC0=(IEC0 & ~(INT0_ENAB)); IFS0=(IFS0 & ~INT3_ENAB); }while(0)
#define SET_PRIO_IO4_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC4=(((IPC4 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u)); IEC0=(IEC0 | INT4_ENAB); IFS0=(IFS0 & ~INT4_ENAB); } }while(0)
#define CLR_PRIO_IO4_INTER(prio,subprio) do{ IEC0=(IEC0 & ~(INT0_ENAB)); IFS0=(IFS0 & ~INT4_ENAB); }while(0)


#define SET_PRIO_SPI1RCV_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC5=(((IPC5 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u));} }while(0)
#define SET_PRIO_ADC1CONV_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC6=(((IPC6 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u));} }while(0)
#define SET_PRIO_SPI2RCV_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC7=(((IPC7 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u));} }while(0)
#define SET_PRIO_RTCC_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC8=(((IPC8 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u));} }while(0)
#define SET_PRIO_DMA3_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC9=(((IPC9 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u));} }while(0)
#define SET_PRIO_DMA7_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC10=(((IPC10 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u));} }while(0)
#define SET_PRIO_CAN2_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC11=(((IPC11 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u));} }while(0)
#define SET_PRIO_UART5_INTER(prio,subprio) do{ if((prio<=7u)&&(subprio<=3u)) {IPC12=(((IPC12 & PRIO_IO_MASK) | (prio<<26u)) | (subprio<<24u));} }while(0)
// IPC0,12 are the interrupt priority SFRs
// IEC0,1,2 are 32 bit configurations to set the interrupts
// IFS0,1,2 are 32 bit representations of there state triggers

#define DINPORT1 PORTA                                                          // We will assign PORTA to DIN
#if defined(ENCODER_HELPER)                                                     // encoder is using PORTA and PORTB as 10 motor encoder inputs
#define DINPORT2 PORTB
#else
#define AINPORT1 PORTB                                                          // We will assing PORTB to the AIN
#endif
#if ((defined(SBGC_GIMBAL_JOY) || defined(SBGC_GIMBAL_HMI)) || defined(BATCH_MIXER))   // joystick and HMI have analog inputs for control values to be set
#define IOCONFIG 0xFFC0u                                                        // Port B 0-5 ANI (Analog) 6-15 (Digital Change Notification Pins CN Interrupt )
#else
#define IOCONFIG 0xFFFFu                                                        // All Banks digital
#endif
#if defined(SBGC_GIMBAL_JOY)
#define DOTPORT1 LATC                                                           // PORTC We will assign PORTC to the 1st DOT bank used in Joystick
#endif
#if (defined(SBGC_GIMBAL_HMI) || defined(EXTENDED_ARM))
#define DOTPORT2 LATD                                                           // We Wiill assign PORTD to 2nd DOT bank used in ComGS
#endif
#define DINPORT3 PORTE                                                          // we shall assign PORTE as an extra Digital input port
#define DOTPORT3 LATF                                                           // PORTF
#define DINPORT4 PORTG                                                          // PORTG

#define DINTRIS1 TRISA                                                          // We will assign TRISA to DIN
#define INVDIN1 LATAINV                                                         // this word defines which inputs on portA (rail1) are inverted
#if defined(ENCODER_HELPER)
#define DINTRIS2 TRISB
#define INVDIN2 LATBINV                                                         // this word defines which inputs on portB (rail2) are inverted
#else
#define AINTRIS1 TRISB                                                          // We will assing TRISB to the AIN
#endif
#define DOTTRIS1 TRISC                                                          // We will assign TRISC to the 1st DOT bank
#define INVDOT1 LATCINV
#define DOT1ODRAIN ODCCSET
#define DOTTRIS2 TRISD                                                          // We Wiill assign TRISD to 2nd DOT bank
#define INVDOT2 LATDINV
#define DOT2ODRAIN ODCDSET
#define DINTRIS3 TRISE                                                          // We will assign TRISE to 3rd DIN bank used by extended arm
#define INVDIN3 LATEINV                                                         // this word defines which inputs on portE (rail3) are inverted
#define DOTTRIS3 TRISF                                                          // We will assign TRISF to 3rd DOT bank used by extended arm
#define INVDOT3 LATFINV
#define DOT3ODRAIN ODCFSET
#define DINTRIS4 TRISG
#define INVDIN4 LATGINV
#define CN_PULL_UP_CONFIG CNPUE                                                 /* set the bit to true to activate internal pull up resistor on CN6-CN18 inputs on PORTB */
#define ESD_INPUT_PU1 (1u<<6u)                                                  /* example safety cut out which needs pull up resistor on input CN6 */
#define ESD_INPUT_PU2 (1u<<7u)                                                  /* example safety cut out which needs pull up resistor on input CN7 */

#if defined(USE_MAVLINK)                                                        // apply macro to set the relays from a mavlink command
#define MAV_SET_RELAY(x) do{ if (x<=7u) {PORTC = PORTC | (1u<<x);} else if (x<=15u) {PORTD = PORTD | (1u<<x);} else { PORTC=0x0Fu; PORTD=0x0Fu; } }while(0)
#define MAV_CLR_RELAY(x) do{ if (x<=7u) {PORTC = PORTC & ~(1u<<x);} else if (x<=15u) {PORTD = PORTD & ~(1u<<x);} else { PORTC=0u; PORTD=0u; } }while(0)
#define MAV_SET_BANK(x) do{ if (x==1u) {PORTC = PORTC | (x & 0x0Fu);} else if (x==2u) {PORTD = PORTD | (x & 0x0Fu);} else if  (x==3u) {PORTC = PORTC & ~(x & 0x0Fu);} else if (x-=4u) {PORTD = PORTD & ~(x & 0x0Fu);} else { PORTC=0x0Fu; PORTD=0x0Fu; } }while(0)
#define MAV_UNIT_OP1_SEQ_STEP(x) do{ if (x==1u) { PORTC = 0b11110000; PORTD = 0b00001111; } else if (x==2u) { PORTC = 0b01010101; PORTD = 0b10101010; } else if (x==3u) { PORTC = 0b11001100; PORTD = 0b01010101; } else { PORTC = 0b10101111; PORTD = 0b11000011; } }while(0)
#define MAV_UNIT_OP2_SEQ_STEP(x) do{ if (x==1u) { PORTC = 0b11010100; PORTD = 0b01101001; } else if (x==2u) { PORTC = 0b01000101; PORTD = 0b11101010; } else if (x==3u) { PORTC = 0b11101100; PORTD = 0b01010111; } else { PORTC = 0b10100111; PORTD = 0b00110011; } }while(0)
#endif

// Define the digital input switches
#if defined(SBGC_GIMBAL_JOY)
#define di1SpeedAngle DINPORT1.B0                                               // Flag TRUE=Speed  FALSE=Angle
#define di2PosHalt DINPORT1.B1                                                  // Flag command FALSE=active or TRUE=HALT
#define di3Reset DINPORT1.B2                                                    // Issue a Reset to the device.
#define di4RCMode DINPORT1.B3                                                   // Select RC Mode (for speed)
#define di5VCMode DINPORT1.B4                                                   // Select VC Mode  (for speed).
#define di6FastSelect DINPORT1.B5                                               // Select Fast mode for ANGLE or SPEED.
#endif

// Assign the Analog inputs to each axis
#if defined(SBGC_GIMBAL_JOY)
#define X_AXIS_IN 1U                                                            /* define each pin carrying analog signal */
#define Y_AXIS_IN 2U
#define Z_AXIS_IN 3U
#if (((X_AXIS_IN == Y_AXIS_IN) || (Y_AXIS_IN == Z_AXIS_IN)) || (Z_AXIS_IN == X_AXIS_IN))
#error "Each input pin for the pitch,roll,yaw must be independant in the gimbal controller configuration (currently at least 2 pins are the same)"
#endif
#endif

#if defined(BATCH_MIXER)
#define LevelInChan 4U                                                          /* define the pin carrying the analog for the level measurement signal */
#if ((((LevelInChan <= X_AXIS_IN) || (LevelInChan <= Y_AXIS_IN)) || (LevelInChan <= Z_AXIS_IN)) && defined(SBGC_GIMBAL_JOY))
#error "Cannot assign the LevelInChan to the same as one of the gimbal axis on on a pin before it"
#endif
#define BATCH_LEVL_MIN 0.0f
#define BATCH_LEVL_MAX 100.0f                                                   /* tank level maximum level */
#define timedOn DINPORT1.B6                                                     /* assign a spare digital input channel for manual timed override switch for use when level instrument fault */

#if defined(EXTENDED_ARM)
#define extArmEndStop DINPORT3.B1                                               /* extend arm end stop proximity switch at extended end */
#define extArmRevStop DINPORT3.B2                                               /* extend arm end stop proximity switch at retracted end  */
#define extArmMidStop DINPORT3.B0                                               /* extend arm end stop proximity switch at middle position */
#define EXT_ARM_NOT_AT_STOP 0u                                                  /* position of i/o */
#define EXT_ARM_MID_POS 1u
#define EXT_ARM_FULL_POS 2u
#define EXT_ARM_RET_POS 4u
#define EXT_ARM_ERR_POS 3u
#define ARM_START_DLY 30                                                        /* delay time before we check wrong direction sensor as it could start here */
#endif

#define DeviceState DOTPORT3                                                    /* where pneumatic valve control harness is wired, joystick controller uses DOT1 and can be on the same MCU as the batcher system */
#define Valve1Open (1u<<0u)                                                     /* first addition corse air valve */
#define Valve2Open (1u<<1u)                                                     /* first addition fine air valve */
#define Valve3Open (1u<<2u)                                                     /* second addition corse air valve */
#define Valve4Open (1u<<3u)                                                     /* second addition fine air valve */
#define Valve5Open (1u<<4u)                                                     /* dosing sprayer air valve */
#define Mixer1Run (1u<<5u)                                                      /* tank mixer */
#define Vib1Start (1u<<6u)                                                      /* solid mixing vibrator or pneumatic conveying system */
#define batchSpare1 (1u<<7u)                                                    /* batch system spare DOT */
#define AuxDevState DOTPORT2                                                    /* need to mod joystick if in same system */
#define extArmFwd (1u<<4u)                                                      /* extend the arm forwards */
#define extArmRev (1u<<5u)                                                      /* extend the arm in reverse direction */
#define hydLevlUp (1u<<7u)                                                      /* hyraulic level system control up */
#define hydLevlDwn (1u<<6u)                                                     /* hyraulic level system control down */

#define BANK1_IO_ESD_MASK (Mixer1Run | ((Valve1Open | Valve2Open) | (Valve3Open | Valve4Open)))  // selects the bits which constitute the esd trip outputs from bank 1 of relay outs

#endif  /* end batch mixer */

// State Engine Reported to the DOT LED on DOTPORT1
#define JoyReset 0xE1U                                                          // AND to Set bit 1
#define JoyAngle 0xE2U                                                          // AND to set bit 2
#define JoySpeed 0xE4U                                                          // AND to set bit 3
#define JoyOff 0xE8U                                                            // AND to set bit 4
#define JoySend 0xF0U                                                           // AND to set bit 5
#define GSCMotorSend 32U
#define GSCPIDAutotune 64U
#define JoyClear 0xE0U                                                          // AND to Reset the bits 1-5 of the Status word (Joystick)

// State Engine poll requests for data DOT LED on DOTPORT2
#define GetExtAngles 0x2U                                                       // get angles LED DOTPORT2 0
#define GetAngles 0x1U                                                          // get angles ext LED DOTPORT2 1
#define GetRealtime4 0x4U                                                       // get realtime data 4 LED DOTPORT2 2
#define Clear3LED 0xF8U                                                         // clear the 1st 3 LED's on DOTPORT2
#define GSCMenuSend 0x8U                                                        // Menu Command exceute LED
#define GSCAdjvar 0x10U                                                         // Adjust variables execute LED
#define GSCGetvar 0x20U                                                         // Get variables request LED
#define GSCImu 0x40U                                                            // Select IMU request LED
#define GSCProSet 0x80U                                                         // Select Profile and load / save / erase it LED
#define GSCConSet 0x80U                                                         // Share the same LED as above we ran out of spare

#define BTN_STATE_RELEASED 0U                                                   // Push button States
#define BTN_STATE_PRESSED 1U
#define BTN_BOUNCE_THRESHOLD_MS 3UL                                              // interval for button de-bouncer threshold (in multiples of 100ms)

// AD1CON1 bits
#define IO_AD_ENABLE (1ul<<15ul)                                                  // bit to set in AD1CON1 for enable
#define IO_SIDL_ENABLE (1ul<<13ul)                                                // enable stop on IDLE mode for the ADC
#define IO_FORM_SF16 (3ul<<8ul)
#define IO_FORM_F16 (2ul<<8ul)
#define IO_FORM_SI16 (1ul<<8ul)
#define IO_FORM_I16 0ul
#define IO_FORM_SF32 (7ul<<8ul)
#define IO_FORM_F32 (6ul<<8ul)
#define IO_FORM_SI32 (5ul<<8ul)
#define IO_FORM_I32 (4ul<<8ul)
#define IO_FORM_CLRASAM (1ul<<4ul)
#define IO_FORM_ASAM (1ul<<2ul)                                                 /* set this bit true to Use auto sampling. Sampling starts after the last conversion is finished */
#define IO_FORM_MSAM ~(1ul<<2ul)                                                /* set this bit false to Use manual sampling. Sampling begins when the user sets AD1CON1bits.SAMP. */
/* 1=The SHA is sampling 0=The SHA is holding. */
#define IO_FORM_SAMP (1ul<<1ul)                                                 /*  When auto sampling is disabled (AD1CON1bits.ASAM=0), set this bit to initiate sampling. When using manual conversion (AD1CON1bits.SSRC=0) clear this bit to zero to start conversion */
#define IO_FORM_NO_SAMP 0xFFFDUL
#define IO_AD_STATE 0x0001UL                                                    /* and with AD1CON1 to get state 1 The analog-to-digital conversion is finished. or 0 The ADC is either pending or has not begun. */

#define IO_CH0SB_15 (0b1111<<24)                                                // bits to set in AD1CHS for enable positive input select MUXB
#define IO_CH0SB_14 (0b1110<<24)                                                // bits to set in AD1CHS for enable
#define IO_CH0SB_13 (0b1101<<24)                                                // bits to set in AD1CHS for enable
#define IO_CH0SB_12 (0b1100<<24)                                                // bits to set in AD1CHS for enable
#define IO_CH0SB_11 (0b1011<<24)                                                // bits to set in AD1CHS for enable
#define IO_CH0SB_10 (0b1010<<24)                                                // bits to set in AD1CHS for enable
#define IO_CH0SB_9 (0b1001<<24)                                                 // bits to set in AD1CHS for enable
#define IO_CH0SB_8 (0b1000<<24)                                                 // bits to set in AD1CHS for enable
#define IO_CH0SB_7 (0b0111<<24)                                                 // bits to set in AD1CHS for enable
#define IO_CH0SB_6 (0b0110<<24)                                                 // bits to set in AD1CHS for enable
#define IO_CH0SB_5 (0b0101<<24)                                                 // bits to set in AD1CHS for enable
#define IO_CH0SB_4 (0b0100<<24)                                                 // bits to set in AD1CHS for enable
#define IO_CH0SB_3 (0b0011<<24)                                                 // bits to set in AD1CHS for enable
#define IO_CH0SB_2 (0b0010<<24)                                                 // bits to set in AD1CHS for enable
#define IO_CH0SB_1 (0b1001<<24)                                                 // bits to set in AD1CHS for enable
#define IO_CH0SB_0 0                                                            // bits to set in AD1CHS for enable

/* AD1CON2 settings (rest as default) choose the analog volt positive and negative and set to scan according to AD1CSSL word */
#define ANI_VOLT_INT_REF_0V do{ AD1CON2=((AD1CON2 & (0b000<<13u)) | (1u<<10u)); }while(0)     /* Use the internal references:VREFH is 3.3 V and VREFL is 0 V */
#define ANI_VOLT_EXT_REF_0V do{ AD1CON2=(((AD1CON2 & (0b000<<13u)) | (0b001<<13u)) | (1u<<10u)); }while(0) /* Use an external referencefor VREFH and an internal referencefor VREFL: VREFH is the voltage on the VREF+ pin and VREFL is 0 V. */
#define ANI_VOLT_INT_REF do{ AD1CON2=(((AD1CON2 & (0b000<<13u)) | (0b010<<13u)) | (1u<<10u)); }while(0)  /* Use an internal referencefor VREFH and an external referencefor VREFL: VREFH is 3.3 V and VREFL is the voltage on the VREF- pin */
#define ANI_VOLT_EXT_REF do{ AD1CON2=(((AD1CON2 & (0b000<<13u)) | (0b011<<13u)) | (1u<<10u)); }while(0)  /* Use external references:VREFH is the voltage on the VREF+ pin and VREFL is the voltage on the VREF- pin */

// AD1CON3 timing of the ADC minimum tad_ns = 75 ns
#define AIN_TAD_NS(tad_ns)  (tad_ns / 25u)-1u                                   /* calculates the ADCS from the tad in ns min val is 75 */
#define AIN_TAD_SAM 5u                                                          /* ADC sample frequency in multiples of the above tads */

// ADCHS1 settings
#define SET_INVERT_ADC do{ AD1CHS=((1u<<23u) | AD1CHS); }while(0)               /* when set The negative input is the pin AN1. */
#define CLR_INVERT_ADC do{ AD1CHS=(~(1u<<23u) & AD1CHS); }while(0)              /* when clear instruction is issued The negative input is VREFL. (normal) */
#define IO_CH0SA_PIN(pin) do{ AD1CHS=((AD1CHS & ~(0b1111<<16u)) | (pin<<16u));   }while(0)   /* pin number 1-15 AD1CHS for enable */

#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
#if defined(SBGC_GIMBAL_JOY)                                                    /* rule is that gimbal io comes first */
#if ((Z_AXIS_IN < Y_AXIS_IN) && (Z_AXIS_IN < X_AXIS_IN))
#define START_MAN_AIN_PIN Z_AXIS_IN                                             /* z axis is start of the manual sampling AIN pin (gimbal is first) */
#elif ((Y_AXIS_IN < X_AXIS_IN) && (Y_AXIS_IN < Z_AXIS_IN))
#define START_MAN_AIN_PIN Y_AXIS_IN                                             /* y axis start of the manual sampling AIN pin (gimbal is first) */
#else
#define START_MAN_AIN_PIN X_AXIS_IN                                             /* x axis start of the manual sampling AIN pin (gimbal is first) */
#endif /* end choose pin order */
#else
#define START_MAN_AIN_PIN LevelInChan
#endif /* end choose feature */
#endif /* end choose method */

#if ((AIN_ADC_METHOD == ADC_AUTO_COLLECT) || (AIN_ADC_METHOD == ADC_MAN_COLLECT))
#if (defined(SBGC_GIMBAL_JOY) && defined(BATCH_MIXER))
#define NUM_OF_AINS 4u                                                          /* number of consequtive AINs to read */
#elif defined(BATCH_MIXER)
#define NUM_OF_AINS 1u
#elif defined(SBGC_GIMBAL_JOY)
#define NUM_OF_AINS 3u
#endif /* end of function enable count */
#endif /* end of adc method branch */

#define go_left (1u<<0u)                                                        /* defintions of craft movement digital outputs */
#define go_right (1u<<1u)
#define reverse_LR (1u<<2u)
#define go_fwd (1u<<0u)
#define go_back (1u<<1u)
#define reverse_FB (1u<<2u)

// macros to drive I/O
#define SET_DOT_BIT_ON(PORT_BYTE,BITNO)  do ( PORT_BYTE |=  (1u<<BITNO) } while(0)
#define SET_DOT_BIT_OFF(PORT_BYTE,BITNO) do ( PORT_BYTE &= ~(1u<<BITNO) } while(0)
#define SET_DOT_BIT_TOGGLE(PORT_BYTE,BITNO) do ( PORT_BYTE ^=  (1u<<BITNO) } while(0)

// Choose Oscillator
#define IO_FRC_OSC 0                                                            // internal fast RC oscillator
#define IO_FRCPLL_OSC 0b1                                                       // FRC PLL OSC
#define IO_PO_OSC 0b01                                                          // PO OSC
#define IO_POPLL_OSC 0b11                                                       // PO PLL OSC
#define IO_SOSC_OSC 0b100                                                       // SO OSC
#define IO_LPRC_OSC 0b101                                                       // LPRC OSC
#define IO_FRC16_OSC 0b110                                                      // FRC div by 16 OSC
#define IO_FRCDIV_OSC 0b111                                                     // FRC div by FRCDIV bits

#define RAW_MAX 1024u                                                            // 10 bit ADC maxiumum raw counts

typedef struct {
        uint8_t  state;                                                         // current state
        uint8_t trigger_state;                                                  // de-bounced state
        uint64_t last_time_ms;                                                  // Time out of the latch
} IO_btn_state_t;                                                               // This is a debounced button type for a DI key press

/* --------------- function definitions ------------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif