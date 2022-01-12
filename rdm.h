#ifndef _RDM_CODES_H_
#define _RDM_CODES_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************/
/* Entertainment Services Technology Association (ESTA)          */
/* ANSI E1.20 Remote Device Management (RDM) over DMX512 Networks*/
/*****************************************************************/
/*                                                               */
/*                          RDM.h                                */
/*                                                               */
/*****************************************************************/
/* Appendix A Defines for the RDM Protocol.                      */
/* Publish date: 3/31/2006                                       */
/*****************************************************************/
/* Compiled by: Scott M. Blair   8/18/2006                       */
/* Updated 10/11/2011: Adding E1.20-2010 and E1.37-1 defines.    */
/*****************************************************************/
/* For updates see: http://www.rdmprotocol.org                   */
/*****************************************************************/
/* Copyright 2006,2011 Litespeed Design                          */
/* Ported : Copyright (C) 2020 A C P Avaiation Scotland          */
/*****************************************************************/
/* Permission to use, copy, modify, and distribute this software */
/* is freely granted, provided that this notice is preserved.    */
/*****************************************************************/

/* Protocol version. */
#define E120_PROTOCOL_VERSION                             0x0100u

/* RDM START CODE (Slot 0)                                                                                                     */
#define        E120_SC_RDM                                0xCCu

/* RDM Protocol Data Structure ID's (Slot 1)                                                                                   */
#define E120_SC_SUB_MESSAGE                               0x01u
/* Broadcast Device UID's                                                                                                      */
#define E120_BROADCAST_ALL_DEVICES_ID                     0xFFFFFFFFFFFFlu   /* (Broadcast all Manufacturers)                    */
//#define ALL_DEVICES_ID                                                                 0xmmmmFFFFFFFF   /* (Specific Manufacturer ID 0xmmmm)                                              */
#define E120_SUB_DEVICE_ALL_CALL                          0xFFFFu

/********************************************************/
/* Table A-1: RDM Command Classes (Slot 20)             */
/********************************************************/
#define E120_DISCOVERY_COMMAND                            0x10u
#define E120_DISCOVERY_COMMAND_RESPONSE                   0x11u
#define E120_GET_COMMAND                                  0x20u
#define E120_GET_COMMAND_RESPONSE                         0x21u
#define E120_SET_COMMAND                                  0x30u
#define E120_SET_COMMAND_RESPONSE                         0x31u

/********************************************************/
/* Table A-2: RDM Response Type (Slot 16)               */
/********************************************************/
#define E120_RESPONSE_TYPE_ACK                            0x00u
#define E120_RESPONSE_TYPE_ACK_TIMER                      0x01u
#define E120_RESPONSE_TYPE_NACK_REASON                    0x02u   /* See Table A-17                                              */
#define E120_RESPONSE_TYPE_ACK_OVERFLOW                   0x03u   /* Additional Response Data available beyond single response length.*/

/********************************************************/
/* Table A-3: RDM Parameter ID's (Slots 21-22)          */
/********************************************************/
/* Category - Network Management   */
#define E120_DISC_UNIQUE_BRANCH                           0x0001u
#define E120_DISC_MUTE                                    0x0002u
#define E120_DISC_UN_MUTE                                 0x0003u
#define E120_PROXIED_DEVICES                              0x0010u
#define E120_PROXIED_DEVICE_COUNT                         0x0011u
#define E120_COMMS_STATUS                                 0x0015u

/* Category - Status Collection    */
#define E120_QUEUED_MESSAGE                               0x0020u /* See Table A-4                                              */
#define E120_STATUS_MESSAGES                              0x0030u /* See Table A-4                                              */
#define E120_STATUS_ID_DESCRIPTION                        0x0031u
#define E120_CLEAR_STATUS_ID                              0x0032u
#define E120_SUB_DEVICE_STATUS_REPORT_THRESHOLD           0x0033u /* See Table A-4                                              */

/* Category - RDM Information      */
#define E120_SUPPORTED_PARAMETERS                         0x0050u               /* Support required only if supporting Parameters beyond the minimum required set.*/
#define E120_PARAMETER_DESCRIPTION                        0x0051u               /* Support required for Manufacturer-Specific PIDs exposed in SUPPORTED_PARAMETERS message */

/* Category - Product Information  */
#define E120_DEVICE_INFO                                  0x0060u
#define E120_PRODUCT_DETAIL_ID_LIST                       0x0070u
#define E120_DEVICE_MODEL_DESCRIPTION                     0x0080u
#define E120_MANUFACTURER_LABEL                           0x0081u
#define E120_DEVICE_LABEL                                 0x0082u
#define E120_FACTORY_DEFAULTS                             0x0090u
#define E120_LANGUAGE_CAPABILITIES                        0x00A0u
#define E120_LANGUAGE                                     0x00B0u
#define E120_SOFTWARE_VERSION_LABEL                       0x00C0u
#define E120_BOOT_SOFTWARE_VERSION_ID                     0x00C1u
#define E120_BOOT_SOFTWARE_VERSION_LABEL                  0x00C2u

/* Category - DMX512 Setup         */
#define E120_DMX_PERSONALITY                              0x00E0u
#define E120_DMX_PERSONALITY_DESCRIPTION                  0x00E1u
#define E120_DMX_START_ADDRESS                            0x00F0u               /* Support required if device uses a DMX512 Slot.             */
#define E120_SLOT_INFO                                    0x0120u
#define E120_SLOT_DESCRIPTION                             0x0121u
#define E120_DEFAULT_SLOT_VALUE                           0x0122u
#define E137_1_DMX_BLOCK_ADDRESS                          0x0140u               /* Defined in ANSI E1.37-1 document                           */
#define E137_1_DMX_FAIL_MODE                              0x0141u               /* Defined in ANSI E1.37-1 document                           */
#define E137_1_DMX_STARTUP_MODE                           0x0142u               /* Defined in ANSI E1.37-1 document                           */

/* Category - Sensors              */
#define E120_SENSOR_DEFINITION                            0x0200u
#define E120_SENSOR_VALUE                                 0x0201u
#define E120_RECORD_SENSORS                               0x0202u

/* Category - Dimmer Settings      */
#define E137_1_DIMMER_INFO                                0x0340u
#define E137_1_MINIMUM_LEVEL                              0x0341u
#define E137_1_MAXIMUM_LEVEL                              0x0342u
#define E137_1_CURVE                                      0x0343u
#define E137_1_CURVE_DESCRIPTION                          0x0344u /* Support required if CURVE is supported                     */
#define E137_1_OUTPUT_RESPONSE_TIME                       0x0345u
#define E137_1_OUTPUT_RESPONSE_TIME_DESCRIPTION           0x0346u /* Support required if OUTPUT_RESPONSE_TIME is supported      */
#define E137_1_MODULATION_FREQUENCY                       0x0347u
#define E137_1_MODULATION_FREQUENCY_DESCRIPTION           0x0348u /* Support required if MODULATION_FREQUENCY is supported      */

/* Category - Power/Lamp Settings  */
#define E120_DEVICE_HOURS                                 0x0400u
#define E120_LAMP_HOURS                                   0x0401u
#define E120_LAMP_STRIKES                                 0x0402u
#define E120_LAMP_STATE                                   0x0403u /* See Table A-8                                              */
#define E120_LAMP_ON_MODE                                 0x0404u /* See Table A-9                                              */
#define E120_DEVICE_POWER_CYCLES                          0x0405u
#define E137_1_BURN_IN                                    0x0440u /* Defined in ANSI E1.37-1                                    */

/* Category - Display Settings     */
#define E120_DISPLAY_INVERT                               0x0500u
#define E120_DISPLAY_LEVEL                                0x0501u

/* Category - Configuration        */
#define E120_PAN_INVERT                                   0x0600u
#define E120_TILT_INVERT                                  0x0601u
#define E120_PAN_TILT_SWAP                                0x0602u
#define E120_REAL_TIME_CLOCK                              0x0603u
#define E137_1_LOCK_PIN                                   0x0640u /* Defined in ANSI E1.37-1                                    */
#define E137_1_LOCK_STATE                                 0x0641u /* Defined in ANSI E1.37-1                                    */
#define E137_1_LOCK_STATE_DESCRIPTION                     0x0642u /* Support required if MODULATION_FREQUENCY is supported      */

/* Category - Control              */
#define E120_IDENTIFY_DEVICE                              0x1000u
#define E120_RESET_DEVICE                                 0x1001u
#define E120_POWER_STATE                                  0x1010u /* See Table A-11                                              */
#define E120_PERFORM_SELFTEST                             0x1020u /* See Table A-10                                              */
#define E120_SELF_TEST_DESCRIPTION                        0x1021u
#define E120_CAPTURE_PRESET                               0x1030u
#define E120_PRESET_PLAYBACK                              0x1031u /* See Table A-7                                               */
#define E137_1_IDENTIFY_MODE                              0x1040u /* Defined in ANSI E1.37-1                                     */
#define E137_1_PRESET_INFO                                0x1041u /* Defined in ANSI E1.37-1                                     */
#define E137_1_PRESET_STATUS                              0x1042u /* Defined in ANSI E1.37-1                                     */
#define E137_1_PRESET_MERGEMODE                           0x1043u /* See E1.37-1 Table A-3                                       */
#define E137_1_POWER_ON_SELF_TEST                         0x1044u /* Defined in ANSI E1.37-1                                     */

/* ESTA Reserved Future RDM Development                   0x7FE0-
                                                          0x7FFF
   Manufacturer-Specific PIDs                             0x8000-
                                                          0xFFDF
   ESTA Reserved Future RDM Development
                                                          0xFFE0-
                                                          0xFFFF
*/

/*****************************************************************/
/* Discovery Mute/Un-Mute Messages Control Field. See Table 7-3. */
/*****************************************************************/
#define E120_CONTROL_PROXIED_DEVICE                       0x0008u
#define E120_CONTROL_BOOT_LOADER                          0x0004u
#define E120_CONTROL_SUB_DEVICE                           0x0002u
#define E120_CONTROL_MANAGED_PROXY                        0x0001u

/********************************************************/
/* Table A-4: Status Type Defines                       */
/********************************************************/
#define E120_STATUS_NONE                                  0x00u   /* Not allowed for use with GET: QUEUED_MESSAGE                */
#define E120_STATUS_GET_LAST_MESSAGE                      0x01u
#define E120_STATUS_ADVISORY                              0x02u
#define E120_STATUS_WARNING                               0x03u
#define E120_STATUS_ERROR                                 0x04u
#define E120_STATUS_ADVISORY_CLEARED                      0x12u  /* Added in E1.20-2010 version                                  */
#define E120_STATUS_WARNING_CLEARED                       0x13u  /* Added in E1.20-2010 version                                  */
#define E120_STATUS_ERROR_CLEARED                         0x14u  /* Added in E1.20-2010 version                                  */

/********************************************************/
/* Table A-5: Product Category Defines                  */
/********************************************************/
#define E120_PRODUCT_CATEGORY_NOT_DECLARED                0x0000u

/* Fixtures - intended as source of illumination See Note 1                                                                     */
#define E120_PRODUCT_CATEGORY_FIXTURE                     0x0100u /* No Fine Category declared                                   */
#define E120_PRODUCT_CATEGORY_FIXTURE_FIXED               0x0101u /* No pan / tilt / focus style functions                       */
#define E120_PRODUCT_CATEGORY_FIXTURE_MOVING_YOKE         0x0102u
#define E120_PRODUCT_CATEGORY_FIXTURE_MOVING_MIRROR       0x0103u
#define E120_PRODUCT_CATEGORY_FIXTURE_OTHER               0x01FFu /* For example, focus but no pan/tilt.                         */

/* Fixture Accessories - add-ons to fixtures or projectors                                                                      */
#define E120_PRODUCT_CATEGORY_FIXTURE_ACCESSORY           0x0200u /* No Fine Category declared.                                  */
#define E120_PRODUCT_CATEGORY_FIXTURE_ACCESSORY_COLOR     0x0201u /* Scrollers / Color Changers                                  */
#define E120_PRODUCT_CATEGORY_FIXTURE_ACCESSORY_YOKE      0x0202u /* Yoke add-on                                                 */
#define E120_PRODUCT_CATEGORY_FIXTURE_ACCESSORY_MIRROR    0x0203u /* Moving mirror add-on                                        */
#define E120_PRODUCT_CATEGORY_FIXTURE_ACCESSORY_EFFECT    0x0204u /* Effects Discs                                               */
#define E120_PRODUCT_CATEGORY_FIXTURE_ACCESSORY_BEAM      0x0205u /* Gobo Rotators /Iris / Shutters / Dousers/ Beam modifiers.   */
#define E120_PRODUCT_CATEGORY_FIXTURE_ACCESSORY_OTHER     0x02FFu

/* Projectors - light source capable of producing realistic images from another media i.e Video / Slide / Oil Wheel / Film */
#define E120_PRODUCT_CATEGORY_PROJECTOR                   0x0300u /* No Fine Category declared.                                  */
#define E120_PRODUCT_CATEGORY_PROJECTOR_FIXED             0x0301u /* No pan / tilt functions.                                    */
#define E120_PRODUCT_CATEGORY_PROJECTOR_MOVING_YOKE       0x0302u
#define E120_PRODUCT_CATEGORY_PROJECTOR_MOVING_MIRROR     0x0303u
#define E120_PRODUCT_CATEGORY_PROJECTOR_OTHER             0x03FFu



/* Atmospheric Effect - earth/wind/fire                                                                                         */
#define E120_PRODUCT_CATEGORY_ATMOSPHERIC                 0x0400u /* No Fine Category declared.                                  */
#define E120_PRODUCT_CATEGORY_ATMOSPHERIC_EFFECT          0x0401u /* Fogger / Hazer / Flame, etc.                                */
#define E120_PRODUCT_CATEGORY_ATMOSPHERIC_PYRO            0x0402u /* See Note 2.                                                 */
#define E120_PRODUCT_CATEGORY_ATMOSPHERIC_OTHER           0x04FFu


/* Intensity Control (specifically Dimming equipment)                                                                           */
#define E120_PRODUCT_CATEGORY_DIMMER                      0x0500u /* No Fine Category declared.                                  */
#define E120_PRODUCT_CATEGORY_DIMMER_AC_INCANDESCENT      0x0501u /* AC > 50VAC                                                  */
#define E120_PRODUCT_CATEGORY_DIMMER_AC_FLUORESCENT       0x0502u
#define E120_PRODUCT_CATEGORY_DIMMER_AC_COLDCATHODE       0x0503u /* High Voltage outputs such as Neon or other cold cathode.    */
#define E120_PRODUCT_CATEGORY_DIMMER_AC_NONDIM            0x0504u /* Non-Dim module in dimmer rack.                              */
#define E120_PRODUCT_CATEGORY_DIMMER_AC_ELV               0x0505u /* AC <= 50V such as 12/24V AC Low voltage lamps.              */
#define E120_PRODUCT_CATEGORY_DIMMER_AC_OTHER             0x0506u
#define E120_PRODUCT_CATEGORY_DIMMER_DC_LEVEL             0x0507u /* Variable DC level output.                                   */
#define E120_PRODUCT_CATEGORY_DIMMER_DC_PWM               0x0508u /* Chopped (PWM) output.                                       */
#define E120_PRODUCT_CATEGORY_DIMMER_CS_LED               0x0509u /* Specialized LED dimmer.                                     */
#define E120_PRODUCT_CATEGORY_DIMMER_OTHER                0x05FFu

/* Power Control (other than Dimming equipment)                                                                                 */
#define E120_PRODUCT_CATEGORY_POWER                       0x0600u /* No Fine Category declared.                                  */
#define E120_PRODUCT_CATEGORY_POWER_CONTROL               0x0601u /* Contactor racks, other forms of Power Controllers.          */
#define E120_PRODUCT_CATEGORY_POWER_SOURCE                0x0602u /* Generators                                                  */
#define E120_PRODUCT_CATEGORY_POWER_OTHER                 0x06FFu

/* Scenic Drive - including motorized effects unrelated to light source.                                                        */
#define E120_PRODUCT_CATEGORY_SCENIC                      0x0700u /* No Fine Category declared                                   */
#define E120_PRODUCT_CATEGORY_SCENIC_DRIVE                0x0701u /* Rotators / Kabuki drops, etc. See Note 2.                   */
#define E120_PRODUCT_CATEGORY_SCENIC_OTHER                0x07FFu

/* DMX Infrastructure, conversion and interfaces                                                                                */
#define E120_PRODUCT_CATEGORY_DATA                        0x0800u /* No Fine Category declared.                                  */
#define E120_PRODUCT_CATEGORY_DATA_DISTRIBUTION           0x0801u /* Splitters/repeaters/Ethernet products used to distribute DMX*/
#define E120_PRODUCT_CATEGORY_DATA_CONVERSION             0x0802u /* Protocol Conversion analog decoders.                        */
#define E120_PRODUCT_CATEGORY_DATA_OTHER                  0x08FFu

/* Audio-Visual Equipment                                                                                                       */
#define E120_PRODUCT_CATEGORY_AV                          0x0900u /* No Fine Category declared.                                  */
#define E120_PRODUCT_CATEGORY_AV_AUDIO                    0x0901u /* Audio controller or device.                                 */
#define E120_PRODUCT_CATEGORY_AV_VIDEO                    0x0902u /* Video controller or display device.                         */
#define E120_PRODUCT_CATEGORY_AV_OTHER                    0x09FFu

/* Parameter Monitoring Equipment See Note 3.                                                                                   */
#define E120_PRODUCT_CATEGORY_MONITOR                     0x0A00u /* No Fine Category declared.                                  */
#define E120_PRODUCT_CATEGORY_MONITOR_ACLINEPOWER         0x0A01u /* Product that monitors AC line voltage, current or power.    */
#define E120_PRODUCT_CATEGORY_MONITOR_DCPOWER             0x0A02u /* Product that monitors DC line voltage, current or power.    */
#define E120_PRODUCT_CATEGORY_MONITOR_ENVIRONMENTAL       0x0A03u /* Temperature or other environmental parameter.               */
#define E120_PRODUCT_CATEGORY_MONITOR_OTHER               0x0AFFu

/* Controllers, Backup devices                                                                                                  */
#define E120_PRODUCT_CATEGORY_CONTROL                     0x7000u /* No Fine Category declared.                                  */
#define E120_PRODUCT_CATEGORY_CONTROL_CONTROLLER          0x7001u
#define E120_PRODUCT_CATEGORY_CONTROL_BACKUPDEVICE        0x7002u
#define E120_PRODUCT_CATEGORY_CONTROL_OTHER               0x70FFu

/* Test Equipment                                                                                                               */
#define E120_PRODUCT_CATEGORY_TEST                        0x7100u /* No Fine Category declared.                                  */
#define E120_PRODUCT_CATEGORY_TEST_EQUIPMENT              0x7101u
#define E120_PRODUCT_CATEGORY_TEST_EQUIPMENT_OTHER        0x71FFu

/* Miscellaneous                                                                                                                */
#define E120_PRODUCT_CATEGORY_OTHER                       0x7FFFu /* For devices that aren't described within this table.        */

/* Manufacturer Specific Categories                       0x8000 -
                                                          0xDFFF                                                                */

/********************************************************/
/* Table A-6: Product Detail Defines                    */
/********************************************************/
#define E120_PRODUCT_DETAIL_NOT_DECLARED                  0x0000u

/* Generally applied to fixtures                                                                                                */
#define E120_PRODUCT_DETAIL_ARC                           0x0001u
#define E120_PRODUCT_DETAIL_METAL_HALIDE                  0x0002u
#define E120_PRODUCT_DETAIL_INCANDESCENT                  0x0003u
#define E120_PRODUCT_DETAIL_LED                           0x0004u
#define E120_PRODUCT_DETAIL_FLUORESCENT                   0x0005u
#define E120_PRODUCT_DETAIL_COLDCATHODE                   0x0006u  /*includes Neon/Argon                                         */
#define E120_PRODUCT_DETAIL_ELECTROLUMINESCENT            0x0007u
#define E120_PRODUCT_DETAIL_LASER                         0x0008u
#define E120_PRODUCT_DETAIL_FLASHTUBE                     0x0009u /* Strobes or other flashtubes                                 */

/* Generally applied to fixture accessories                                                                                     */
#define E120_PRODUCT_DETAIL_COLORSCROLLER                 0x0100u
#define E120_PRODUCT_DETAIL_COLORWHEEL                    0x0101u
#define E120_PRODUCT_DETAIL_COLORCHANGE                   0x0102u /* Semaphore or other type                                     */
#define E120_PRODUCT_DETAIL_IRIS_DOUSER                   0x0103u
#define E120_PRODUCT_DETAIL_DIMMING_SHUTTER               0x0104u
#define E120_PRODUCT_DETAIL_PROFILE_SHUTTER               0x0105u /* hard-edge beam masking                                      */
#define E120_PRODUCT_DETAIL_BARNDOOR_SHUTTER              0x0106u /* soft-edge beam masking                                      */
#define E120_PRODUCT_DETAIL_EFFECTS_DISC                  0x0107u
#define E120_PRODUCT_DETAIL_GOBO_ROTATOR                  0x0108u

/* Generally applied to Projectors                                                                                              */
#define E120_PRODUCT_DETAIL_VIDEO                         0x0200u
#define E120_PRODUCT_DETAIL_SLIDE                         0x0201u
#define E120_PRODUCT_DETAIL_FILM                          0x0202u
#define E120_PRODUCT_DETAIL_OILWHEEL                      0x0203u
#define E120_PRODUCT_DETAIL_LCDGATE                       0x0204u

/* Generally applied to Atmospheric Effects                                                                                     */
#define E120_PRODUCT_DETAIL_FOGGER_GLYCOL                 0x0300u /* Glycol/Glycerin hazer                                       */
#define E120_PRODUCT_DETAIL_FOGGER_MINERALOIL             0x0301u /* White Mineral oil hazer                                     */
#define E120_PRODUCT_DETAIL_FOGGER_WATER                  0x0302u /* Water hazer                                                 */
#define E120_PRODUCT_DETAIL_CO2                           0x0303u /* Dry Ice/Carbon Dioxide based                                */
#define E120_PRODUCT_DETAIL_LN2                           0x0304u /* Nitrogen based                                              */
#define E120_PRODUCT_DETAIL_BUBBLE                        0x0305u /* including foam                                              */
#define E120_PRODUCT_DETAIL_FLAME_PROPANE                 0x0306u
#define E120_PRODUCT_DETAIL_FLAME_OTHER                   0x0307u
#define E120_PRODUCT_DETAIL_OLEFACTORY_STIMULATOR         0x0308u /* Scents                                                      */
#define E120_PRODUCT_DETAIL_SNOW                          0x0309u
#define E120_PRODUCT_DETAIL_WATER_JET                     0x030Au /* Fountain controls etc                                       */
#define E120_PRODUCT_DETAIL_WIND                          0x030Bu /* Air Mover                                                   */
#define E120_PRODUCT_DETAIL_CONFETTI                      0x030Cu
#define E120_PRODUCT_DETAIL_HAZARD                        0x030Du /* Any form of pyrotechnic control or device.                  */

/* Generally applied to Dimmers/Power controllers See Note 1                                                                    */
#define E120_PRODUCT_DETAIL_PHASE_CONTROL                 0x0400u
#define E120_PRODUCT_DETAIL_REVERSE_PHASE_CONTROL         0x0401u /* includes FET/IGBT                                           */
#define E120_PRODUCT_DETAIL_SINE                          0x0402u
#define E120_PRODUCT_DETAIL_PWM                           0x0403u
#define E120_PRODUCT_DETAIL_DC                            0x0404u /* Variable voltage                                            */
#define E120_PRODUCT_DETAIL_HFBALLAST                     0x0405u /* for Fluorescent                                             */
#define E120_PRODUCT_DETAIL_HFHV_NEONBALLAST              0x0406u /* for Neon/Argon and other coldcathode.                       */
#define E120_PRODUCT_DETAIL_HFHV_EL                       0x0407u /* for Electroluminscent                                       */
#define E120_PRODUCT_DETAIL_MHR_BALLAST                   0x0408u /* for Metal Halide                                            */
#define E120_PRODUCT_DETAIL_BITANGLE_MODULATION           0x0409u
#define E120_PRODUCT_DETAIL_FREQUENCY_MODULATION          0x040Au
#define E120_PRODUCT_DETAIL_HIGHFREQUENCY_12V             0x040Bu /* as commonly used with MR16 lamps                            */
#define E120_PRODUCT_DETAIL_RELAY_MECHANICAL              0x040Cu /* See Note 1                                                  */
#define E120_PRODUCT_DETAIL_RELAY_ELECTRONIC              0x040Du /* See Note 1, Note 2                                          */
#define E120_PRODUCT_DETAIL_SWITCH_ELECTRONIC             0x040Eu /* See Note 1, Note 2                                          */
#define E120_PRODUCT_DETAIL_CONTACTOR                     0x040Fu /* See Note 1                                                  */

/* Generally applied to Scenic drive                                                                                            */
#define E120_PRODUCT_DETAIL_MIRRORBALL_ROTATOR            0x0500u
#define E120_PRODUCT_DETAIL_OTHER_ROTATOR                 0x0501u /* includes turntables                                         */
#define E120_PRODUCT_DETAIL_KABUKI_DROP                   0x0502u
#define E120_PRODUCT_DETAIL_CURTAIN                       0x0503u /* flown or traveller                                          */
#define E120_PRODUCT_DETAIL_LINESET                       0x0504u
#define E120_PRODUCT_DETAIL_MOTOR_CONTROL                 0x0505u
#define E120_PRODUCT_DETAIL_DAMPER_CONTROL                0x0506u /* HVAC Damper                                                 */

/* Generally applied to Data Distribution                                                                                       */
#define E120_PRODUCT_DETAIL_SPLITTER                      0x0600u /* Includes buffers/repeaters                                  */
#define E120_PRODUCT_DETAIL_ETHERNET_NODE                 0x0601u /* DMX512 to/from Ethernet                                     */
#define E120_PRODUCT_DETAIL_MERGE                         0x0602u /* DMX512 combiner                                             */
#define E120_PRODUCT_DETAIL_DATAPATCH                     0x0603u /* Electronic Datalink Patch                                   */
#define E120_PRODUCT_DETAIL_WIRELESS_LINK                 0x0604u /* radio/infrared                                              */

/* Generally applied to Data Conversion and Interfaces                                                                          */
#define E120_PRODUCT_DETAIL_PROTOCOL_CONVERTER            0x0701u /* D54/AMX192/Non DMX serial links, etc to/from DMX512         */
#define E120_PRODUCT_DETAIL_ANALOG_DEMULTIPLEX            0x0702u /* DMX to DC voltage                                           */
#define E120_PRODUCT_DETAIL_ANALOG_MULTIPLEX              0x0703u /* DC Voltage to DMX                                           */
#define E120_PRODUCT_DETAIL_SWITCH_PANEL                  0x0704u /* Pushbuttons to DMX or polled using RDM                      */

/* Generally applied to Audio or Video (AV) devices                                                                             */
#define E120_PRODUCT_DETAIL_ROUTER                        0x0800u /* Switching device                                            */
#define E120_PRODUCT_DETAIL_FADER                         0x0801u /* Single channel                                              */
#define E120_PRODUCT_DETAIL_MIXER                         0x0802u /* Multi-channel                                               */

/* Generally applied to Controllers, Backup devices and Test Equipment                                                          */
#define E120_PRODUCT_DETAIL_CHANGEOVER_MANUAL            0x0900u /* requires manual intervention to assume control of DMX line   */
#define E120_PRODUCT_DETAIL_CHANGEOVER_AUTO              0x0901u /* may automatically assume control of DMX line                 */
#define E120_PRODUCT_DETAIL_TEST                         0x0902u /* test equipment                                               */

/* Could be applied to any category                                                                                             */
#define E120_PRODUCT_DETAIL_GFI_RCD                      0x0A00u /* device includes GFI/RCD trip                                 */
#define E120_PRODUCT_DETAIL_BATTERY                      0x0A01u /* device is battery operated                                   */
#define E120_PRODUCT_DETAIL_CONTROLLABLE_BREAKER         0x0A02u
#define E120_PRODUCT_DETAIL_OTHER                        0x7FFFu /* for use where the Manufacturer believes that none of the
                                                                   defined details apply.                                            */
/* Manufacturer Specific Types                           0x8000-
                                                         0xDFFF                                                                 */
/* Note 1: Products intended for switching 50V AC / 120V DC or greater should be declared with a
           Product Category of PRODUCT_CATEGORY_POWER_CONTROL.
           Products only suitable for extra low voltage switching (typically up to 50VAC / 30VDC) at currents
           less than 1 ampere should be declared with a Product Category of PRODUCT_CATEGORY_DATA_CONVERSION.
           Please refer to GET: DEVICE_INFO and Table A-5 for an explanation of Product Category declaration.
   Note 2: Products with TTL, MOSFET or Open Collector Transistor Outputs or similar non-isolated electronic
           outputs should be declared as PRODUCT_DETAIL_SWITCH_ELECTRONIC. Use of PRODUCT_DETAIL_RELAY_ELECTRONIC
           shall be restricted to devices whereby the switched circuits are electrically isolated from the control signals.     */

/********************************************************/
/* Table A-7: Preset Playback Defines                   */
/********************************************************/
#define E120_PRESET_PLAYBACK_OFF                         0x0000u /* Returns to Normal DMX512 Input                               */
#define E120_PRESET_PLAYBACK_ALL                         0xFFFFu /* Plays Scenes in Sequence if supported.                       */
/*      E120_PRESET_PLAYBACK_SCENE                       0x0001-
                                                         0xFFFE    Plays individual Scene #                                     */

/********************************************************/
/* Table A-8: Lamp State Defines                        */
/********************************************************/
#define E120_LAMP_OFF                                    0x00u   /* No demonstrable light output                                 */
#define E120_LAMP_ON                                     0x01u
#define E120_LAMP_STRIKE                                 0x02u   /* Arc-Lamp ignite                                              */
#define E120_LAMP_STANDBY                                0x03u   /* Arc-Lamp Reduced Power Mode                                  */
#define E120_LAMP_NOT_PRESENT                            0x04u   /* Lamp not installed                                           */
#define E120_LAMP_ERROR                                  0x7Fu
/* Manufacturer-Specific States                          0x80-
                                                         0xDF                                                                   */

/********************************************************/
/* Table A-9: Lamp On Mode Defines                      */
/********************************************************/
#define E120_LAMP_ON_MODE_OFF                            0x00u   /* Lamp Stays off until directly instructed to Strike.          */
#define E120_LAMP_ON_MODE_DMX                            0x01u   /* Lamp Strikes upon receiving a DMX512 signal.                 */
#define E120_LAMP_ON_MODE_ON                             0x02u   /* Lamp Strikes automatically at Power-up.                      */
#define E120_LAMP_ON_MODE_AFTER_CAL                      0x03u   /* Lamp Strikes after Calibration or Homing procedure.          */
/* Manufacturer-Specific Modes                           0x80-
                                                         0xDF                                                                   */

/********************************************************/
/* Table A-10: Self Test Defines                        */
/********************************************************/
#define E120_SELF_TEST_OFF                               0x00u   /* Turns Self Tests Off                                         */
/* Manufacturer Tests                                    0x01-
                                                         0xFE      Various Manufacturer Self Tests                              */
#define E120_SELF_TEST_ALL                               0xFFu   /* Self Test All, if applicable                                 */

/********************************************************/
/* Table A-11: Power State Defines                      */
/********************************************************/
#define E120_POWER_STATE_FULL_OFF                        0x00u   /* Completely disengages power to device. Device can no longer respond. */
#define E120_POWER_STATE_SHUTDOWN                        0x01u   /* Reduced power mode, may require device reset to return to
                                                                   normal operation. Device still responds to messages.         */
#define E120_POWER_STATE_STANDBY                         0x02u   /* Reduced power mode. Device can return to NORMAL without a
                                                                   reset. Device still responds to messages.                    */
#define E120_POWER_STATE_NORMAL                          0xFFu   /* Normal Operating Mode.                                       */


/********************************************************/
/* Table A-12: Sensor Type Defines                      */
/********************************************************/
#define E120_SENS_TEMPERATURE                            0x00u
#define E120_SENS_VOLTAGE                                0x01u
#define E120_SENS_CURRENT                                0x02u
#define E120_SENS_FREQUENCY                              0x03u
#define E120_SENS_RESISTANCE                             0x04u   /* Eg: Cable resistance                                         */
#define E120_SENS_POWER                                  0x05u
#define E120_SENS_MASS                                   0x06u   /* Eg: Truss load Cell                                          */
#define E120_SENS_LENGTH                                 0x07u
#define E120_SENS_AREA                                   0x08u
#define E120_SENS_VOLUME                                 0x09u   /* Eg: Smoke Fluid                                              */
#define E120_SENS_DENSITY                                0x0Au
#define E120_SENS_VELOCITY                               0x0Bu
#define E120_SENS_ACCELERATION                           0x0Cu
#define E120_SENS_FORCE                                  0x0Du
#define E120_SENS_ENERGY                                 0x0Eu
#define E120_SENS_PRESSURE                               0x0Fu
#define E120_SENS_TIME                                   0x10u
#define E120_SENS_ANGLE                                  0x11u
#define E120_SENS_POSITION_X                             0x12u   /* E.g.: Lamp position on Truss                                 */
#define E120_SENS_POSITION_Y                             0x13u
#define E120_SENS_POSITION_Z                             0x14u
#define E120_SENS_ANGULAR_VELOCITY                       0x15u   /* E.g.: Wind speed                                             */
#define E120_SENS_LUMINOUS_INTENSITY                     0x16u
#define E120_SENS_LUMINOUS_FLUX                          0x17u
#define E120_SENS_ILLUMINANCE                            0x18u
#define E120_SENS_CHROMINANCE_RED                        0x19u
#define E120_SENS_CHROMINANCE_GREEN                      0x1Au
#define E120_SENS_CHROMINANCE_BLUE                       0x1Bu
#define E120_SENS_CONTACTS                               0x1Cu   /* E.g.: Switch inputs.                                         */
#define E120_SENS_MEMORY                                 0x1Du   /* E.g.: ROM Size                                               */
#define E120_SENS_ITEMS                                  0x1Eu   /* E.g.: Scroller gel frames.                                   */
#define E120_SENS_HUMIDITY                               0x1Fu
#define E120_SENS_COUNTER_16BIT                          0x20u
#define E120_SENS_OTHER                                  0x7Fu
/* Manufacturer-Specific Sensors                         0x80-
                                                         0xFF                                                                   */

/********************************************************/

/* Table A-13: Sensor Unit Defines                      */

/********************************************************/
#define E120_UNITS_NONE                                  0x00u   /* CONTACTS                                                     */
#define E120_UNITS_CENTIGRADE                            0x01u   /* TEMPERATURE                                                        */
#define E120_UNITS_VOLTS_DC                              0x02u   /* VOLTAGE                                                        */
#define E120_UNITS_VOLTS_AC_PEAK                         0x03u   /* VOLTAGE                                                      */
#define E120_UNITS_VOLTS_AC_RMS                          0x04u   /* VOLTAGE                                                      */
#define E120_UNITS_AMPERE_DC                             0x05u   /* CURRENT                                                        */
#define E120_UNITS_AMPERE_AC_PEAK                        0x06u   /* CURRENT                                                        */
#define E120_UNITS_AMPERE_AC_RMS                         0x07u   /* CURRENT                                                      */
#define E120_UNITS_HERTZ                                 0x08u   /* FREQUENCY / ANGULAR_VELOCITY                                 */
#define E120_UNITS_OHM                                   0x09u   /* RESISTANCE                                                        */
#define E120_UNITS_WATT                                  0x0Au   /* POWER                                                        */
#define E120_UNITS_KILOGRAM                              0x0Bu   /* MASS                                                         */
#define E120_UNITS_METERS                                0x0Cu   /* LENGTH / POSITION                                                */
#define E120_UNITS_METERS_SQUARED                        0x0Du   /* AREA                                                                */
#define E120_UNITS_METERS_CUBED                          0x0Eu   /* VOLUME                                                       */
#define E120_UNITS_KILOGRAMMES_PER_METER_CUBED           0x0Fu   /* DENSITY                                                      */
#define E120_UNITS_METERS_PER_SECOND                     0x10u   /* VELOCITY                                                        */
#define E120_UNITS_METERS_PER_SECOND_SQUARED             0x11u   /* ACCELERATION                                                        */
#define E120_UNITS_NEWTON                                0x12u   /* FORCE                                                        */
#define E120_UNITS_JOULE                                 0x13u   /* ENERGY                                                        */
#define E120_UNITS_PASCAL                                0x14u   /* PRESSURE                                                        */
#define E120_UNITS_SECOND                                0x15u   /* TIME                                                         */
#define E120_UNITS_DEGREE                                0x16u   /* ANGLE                                                        */
#define E120_UNITS_STERADIAN                             0x17u   /* ANGLE                                                        */
#define E120_UNITS_CANDELA                               0x18u   /* LUMINOUS_INTENSITY                                           */
#define E120_UNITS_LUMEN                                 0x19u   /* LUMINOUS_FLUX                                                */
#define E120_UNITS_LUX                                   0x1Au   /* ILLUMINANCE                                                        */
#define E120_UNITS_IRE                                   0x1Bu   /* CHROMINANCE                                                  */
#define E120_UNITS_BYTE                                  0x1Cu   /* MEMORY                                                        */
/* Manufacturer-Specific Units                           0x80-

                                                                          0xFF                                                                                    */
/********************************************************/

/* Table A-14: Sensor Unit Prefix Defines               */

/********************************************************/
#define E120_PREFIX_NONE                                 0x00u   /* Multiply by 1                                                */
#define E120_PREFIX_DECI                                 0x01u   /* Multiply by 10-1                                                    */
#define E120_PREFIX_CENTI                                0x02u   /* Multiply by 10-2                                                    */
#define E120_PREFIX_MILLI                                0x03u   /* Multiply by 10-3                                                    */
#define E120_PREFIX_MICRO                                0x04u   /* Multiply by 10-6                                                    */
#define E120_PREFIX_NANO                                 0x05u   /* Multiply by 10-9                                                    */
#define E120_PREFIX_PICO                                 0x06u   /* Multiply by 10-12                                                */
#define E120_PREFIX_FEMTO                                0x07u   /* Multiply by 10-15                                                */
#define E120_PREFIX_ATTO                                 0x08u   /* Multiply by 10-18                                                */
#define E120_PREFIX_ZEPTO                                0x09u   /* Multiply by 10-21                                                */
#define E120_PREFIX_YOCTO                                0x0Au   /* Multiply by 10-24                                                */
#define E120_PREFIX_DECA                                 0x11u   /* Multiply by 10+1                                                    */
#define E120_PREFIX_HECTO                                0x12u   /* Multiply by 10+2                                                    */
#define E120_PREFIX_KILO                                 0x13u   /* Multiply by 10+3                                                    */
#define E120_PREFIX_MEGA                                 0x14u   /* Multiply by 10+6                                                    */
#define E120_PREFIX_GIGA                                 0x15u   /* Multiply by 10+9                                                    */
#define E120_PREFIX_TERA                                 0x16u   /* Multiply by 10+12                                                */
#define E120_PREFIX_PETA                                 0x17u   /* Multiply by 10+15                                                */
#define E120_PREFIX_EXA                                  0x18u   /* Multiply by 10+18                                                */
#define E120_PREFIX_ZETTA                                0x19u   /* Multiply by 10+21                                                */
#define E120_PREFIX_YOTTA                                0x1Au   /* Multiply by 10+24                                                */

/********************************************************/

/* Table A-15: Data Type Defines                        */

/********************************************************/
#define E120_DS_NOT_DEFINED                              0x00u   /* Data type is not defined                                     */
#define E120_DS_BIT_FIELD                                0x01u   /* Data is bit packed                                                        */
#define E120_DS_ASCII                                    0x02u   /* Data is a string                                                                    */
#define E120_DS_UNSIGNED_BYTE                            0x03u   /* Data is an array of unsigned bytes                           */
#define E120_DS_SIGNED_BYTE                              0x04u   /* Data is an array of signed bytes                             */
#define E120_DS_UNSIGNED_WORD                            0x05u   /* Data is an array of unsigned 16-bit words                        */
#define E120_DS_SIGNED_WORD                              0x06u   /* Data is an array of signed 16-bit words                            */
#define E120_DS_UNSIGNED_DWORD                           0x07u   /* Data is an array of unsigned 32-bit words                        */
#define E120_DS_SIGNED_DWORD                             0x08u   /* Data is an array of signed 32-bit words                                                */
/* Manufacturer-Specific Data Types                                          0x80-                                                                  */
/*                                                       0xDF                                                                        */

/********************************************************/
/* Table A-16: Parameter Desc. Command Class Defines    */
/********************************************************/
#define E120_CC_GET                                      0x01u   /* PID supports GET only                                        */
#define E120_CC_SET                                      0x02u   /* PID supports SET only                                        */
#define E120_CC_GET_SET                                  0x03u   /* PID supports GET & SET                                       */

/********************************************************/

/* Table A-17: Response NACK Reason Code Defines        */

/********************************************************/
#define E120_NR_UNKNOWN_PID                              0x0000u /* The responder cannot comply with request because the message
                                                                   is not implemented in responder.                             */
#define E120_NR_FORMAT_ERROR                             0x0001u /* The responder cannot interpret request as controller data
                                                                   was not formatted correctly.                                 */
#define E120_NR_HARDWARE_FAULT                           0x0002u /* The responder cannot comply due to an internal hardware fault*/
#define E120_NR_PROXY_REJECT                             0x0003u /* Proxy is not the RDM line master and cannot comply with message.*/
#define E120_NR_WRITE_PROTECT                            0x0004u /* SET Command normally allowed but being blocked currently.    */
#define E120_NR_UNSUPPORTED_COMMAND_CLASS                0x0005u /* Not valid for Command Class attempted. May be used where
                                                                   GET allowed but SET is not supported.                        */
#define E120_NR_DATA_OUT_OF_RANGE                        0x0006u /* Value for given Parameter out of allowable range or
                                                                   not supported.                                               */
#define E120_NR_BUFFER_FULL                              0x0007u /* Buffer or Queue space currently has no free space to store data. */
#define E120_NR_PACKET_SIZE_UNSUPPORTED                  0x0008u /* Incoming message exceeds buffer capacity.                    */
#define E120_NR_SUB_DEVICE_OUT_OF_RANGE                  0x0009u /* Sub-Device is out of range or unknown.                       */
#define E120_NR_PROXY_BUFFER_FULL                        0x000Au /* Proxy buffer is full and can not store any more Queued       */
                                                                /* Message or Status Message responses.                         */

/********************************************************************************************************************************/
/********************************************************************************************************************************/
/* ANSI E1.37-1 DEFINES                                                                                                         */
/********************************************************************************************************************************/

/********************************************************************************************************************************/
/********************************************************/

/* E1.37-1 Table A-2: Preset Programmed Defines         */

/********************************************************/
#define E137_1_PRESET_NOT_PROGRAMMED                     0x00u /* Preset Scene not programmed.                                   */
#define E137_1_PRESET_PROGRAMMED                         0x01u /* Preset Scene programmed.                                       */
#define E137_1_PRESET_PROGRAMMED_READ_ONLY               0x02u /* Preset Scene read-only, factory programmed.                    */

/********************************************************/

/* E1.37-1 Table A-3: Merge Mode Defines                */

/********************************************************/
#define E137_1_MERGEMODE_DEFAULT                         0x00u /* Preset overrides DMX512 default behavior as defined in         */
                                                              /* E1.20 PRESET_PLAYBACK                                          */
#define E137_1_MERGEMODE_HTP                             0x01u /* Highest Takes Precedence on slot by slot basis                 */
#define E137_1_MERGEMODE_LTP                             0x02u /* Latest Takes Precedence from Preset or DMX512 on slot by slot  */
#define E137_1_MERGEMODE_DMX_ONLY                        0x03u /* DMX512 only, Preset ignored                                    */
#define E137_1_MERGEMODE_OTHER                           0xFFu /* Other (undefined) merge mode                                   */

#ifdef __cplusplus
}
#endif

#endif