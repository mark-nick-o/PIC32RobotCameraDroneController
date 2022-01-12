#ifndef PSLR_H
#define PSLR_H
/*
    pkTriggerCord
    Copyright (C) 2011-2019 Andras Salamon <andras.salamon@melda.info>
     * Ported : Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
     
    Remote control of Pentax DSLR cameras.

    Support for K200D added by Jens Dreyer <jens.dreyer@udo.edu> 04/2011
    Support for K-r added by Vincenc Podobnik <vincenc.podobnik@gmail.com> 06/2011
    Support for K-30 added by Camilo Polymeris <cpolymeris@gmail.com> 09/2012
    Support for K-01 added by Ethan Queen <ethanqueen@gmail.com> 01/2013
    Support for K-3 added by Tao Wang <twang2218@gmail.com> 01/2016
    based on:
    PK-Remote
    Remote control of Pentax DSLR cameras.
    Copyright (C) 2008 Pontus Lidman <pontus@lysator.liu.se>
    PK-Remote for Windows
    Copyright (C) 2010 Tomasz Kos

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU General Public License
    and GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifdef __cplusplus
extern "C"
{
#endif

//#include "pslr_enum.h"
// #include "pslr_scsi.h"
// #include "pslr_model.h"

#define PSLR_LIGHT_METER_AE_LOCK 0x8

typedef enum {
    PSLR_OK = 0,
    PSLR_DEVICE_ERROR,
    PSLR_SCSI_ERROR,
    PSLR_COMMAND_ERROR,
    PSLR_READ_ERROR,
    PSLR_NO_MEMORY,
    PSLR_PARAM,                 /* Invalid parameters to API */
    PSLR_ERROR_MAX
} pslr_result;                                                                  // when using scsi over udp

typedef enum {
    PSLR_COLOR_SPACE_SRGB,
    PSLR_COLOR_SPACE_ADOBERGB,
    PSLR_COLOR_SPACE_MAX
} pslr_color_space_t;

typedef enum {
    PSLR_AF_MODE_MF,
    PSLR_AF_MODE_AF_S,
    PSLR_AF_MODE_AF_C,
    PSLR_AF_MODE_AF_A,
    PSLR_AF_MODE_MAX
} pslr_af_mode_t;

typedef enum {
    PSLR_AE_METERING_MULTI,
    PSLR_AE_METERING_CENTER,
    PSLR_AE_METERING_SPOT,
    PSLR_AE_METERING_MAX
} pslr_ae_metering_t;

typedef enum {
    PSLR_FLASH_MODE_MANUAL = 0,
    PSLR_FLASH_MODE_MANUAL_REDEYE = 1,
    PSLR_FLASH_MODE_SLOW = 2,
    PSLR_FLASH_MODE_SLOW_REDEYE = 3,
    PSLR_FLASH_MODE_TRAILING_CURTAIN = 4,
    PSLR_FLASH_MODE_AUTO = 5,
    PSLR_FLASH_MODE_AUTO_REDEYE = 6,
    /* 7 not used */
    PSLR_FLASH_MODE_WIRELESS = 8,
    PSLR_FLASH_MODE_MAX = 9
} pslr_flash_mode_t;

typedef enum {
    PSLR_DRIVE_MODE_SINGLE,
    PSLR_DRIVE_MODE_CONTINUOUS_HI,
    PSLR_DRIVE_MODE_SELF_TIMER_12,
    PSLR_DRIVE_MODE_SELF_TIMER_2,
    PSLR_DRIVE_MODE_REMOTE,
    PSLR_DRIVE_MODE_REMOTE_3,
    PSLR_DRIVE_MODE_CONTINUOUS_LO,
    PSLR_DRIVE_MODE_MAX
} pslr_drive_mode_t;

typedef enum {
    PSLR_AF_POINT_SEL_AUTO_5,
    PSLR_AF_POINT_SEL_SELECT,
    PSLR_AF_POINT_SEL_SPOT,
    PSLR_AF_POINT_SEL_AUTO_11, /* maybe not for all cameras */
    PSLR_AF_POINT_SEL_EXPANDED, /* only for newer */
    PSLR_AF_POINT_SEL_MAX
} pslr_af_point_sel_t;

typedef enum {
    PSLR_AF11_POINT_TOP_LEFT  = 0x01,
    PSLR_AF11_POINT_TOP_MID   = 0x2,
    PSLR_AF11_POINT_TOP_RIGHT = 0x4,
    PSLR_AF11_POINT_FAR_LEFT  = 0x8,
    PSLR_AF11_POINT_MID_LEFT  = 0x10,
    PSLR_AF11_POINT_MID_MID   = 0x20,
    PSLR_AF11_POINT_MID_RIGHT = 0x40,
    PSLR_AF11_POINT_FAR_RIGHT = 0x80,
    PSLR_AF11_POINT_BOT_LEFT  = 0x100,
    PSLR_AF11_POINT_BOT_MID   = 0x200,
    PSLR_AF11_POINT_BOT_RIGHT = 0x400
} pslr_af11_point_t;

typedef enum {
    PSLR_JPEG_IMAGE_TONE_NONE = -1,
    PSLR_JPEG_IMAGE_TONE_NATURAL,
    PSLR_JPEG_IMAGE_TONE_BRIGHT,
    PSLR_JPEG_IMAGE_TONE_PORTRAIT,
    PSLR_JPEG_IMAGE_TONE_LANDSCAPE,
    PSLR_JPEG_IMAGE_TONE_VIBRANT,
    PSLR_JPEG_IMAGE_TONE_MONOCHROME,
    PSLR_JPEG_IMAGE_TONE_MUTED,
    PSLR_JPEG_IMAGE_TONE_REVERSAL_FILM,
    PSLR_JPEG_IMAGE_TONE_BLEACH_BYPASS,
    PSLR_JPEG_IMAGE_TONE_RADIANT,
    PSLR_JPEG_IMAGE_TONE_CROSS_PROCESSING,
    PSLR_JPEG_IMAGE_TONE_FLAT,
    PSLR_JPEG_IMAGE_TONE_AUTO,
    PSLR_JPEG_IMAGE_TONE_MAX
} pslr_jpeg_image_tone_t;

typedef enum {
    PSLR_WHITE_BALANCE_MODE_AUTO,
    PSLR_WHITE_BALANCE_MODE_DAYLIGHT,
    PSLR_WHITE_BALANCE_MODE_SHADE,
    PSLR_WHITE_BALANCE_MODE_CLOUDY,
    PSLR_WHITE_BALANCE_MODE_FLUORESCENT_DAYLIGHT_COLOR,
    PSLR_WHITE_BALANCE_MODE_FLUORESCENT_DAYLIGHT_WHITE,
    PSLR_WHITE_BALANCE_MODE_FLUORESCENT_COOL_WHITE,
    PSLR_WHITE_BALANCE_MODE_TUNGSTEN,
    PSLR_WHITE_BALANCE_MODE_FLASH,
    PSLR_WHITE_BALANCE_MODE_MANUAL,
    PSLR_WHITE_BALANCE_MODE_MANUAL_2,
    PSLR_WHITE_BALANCE_MODE_MANUAL_3,
    PSLR_WHITE_BALANCE_MODE_KELVIN_1,
    PSLR_WHITE_BALANCE_MODE_KELVIN_2,
    PSLR_WHITE_BALANCE_MODE_KELVIN_3,
    PSLR_WHITE_BALANCE_MODE_FLUORESCENT_WARM_WHITE,
    PSLR_WHITE_BALANCE_MODE_CTE,
    PSLR_WHITE_BALANCE_MODE_MULTI_AUTO,
    PSLR_WHITE_BALANCE_MODE_MAX
} pslr_white_balance_mode_t;

typedef enum {
    PSLR_CUSTOM_EV_STEPS_1_2,
    PSLR_CUSTOM_EV_STEPS_1_3,
    PSLR_CUSTOM_EV_STEPS_MAX
} pslr_custom_ev_steps_t;

typedef enum {
    PSLR_CUSTOM_SENSITIVITY_STEPS_1EV,
    PSLR_CUSTOM_SENSITIVITY_STEPS_AS_EV,
    PSLR_CUSTOM_SENSITIVITY_STEPS_MAX
} pslr_custom_sensitivity_steps_t;

typedef enum {
    PSLR_IMAGE_FORMAT_JPEG,
    PSLR_IMAGE_FORMAT_RAW,
    PSLR_IMAGE_FORMAT_RAW_PLUS,
    PSLR_IMAGE_FORMAT_MAX
} pslr_image_format_t;

typedef enum {
    PSLR_RAW_FORMAT_PEF,
    PSLR_RAW_FORMAT_DNG,
    PSLR_RAW_FORMAT_MAX
} pslr_raw_format_t;

typedef enum {
    PSLR_SCENE_MODE_NONE,
    PSLR_SCENE_MODE_HISPEED,
    PSLR_SCENE_MODE_DOF,
    PSLR_SCENE_MODE_MTF,
    PSLR_SCENE_MODE_STANDARD,
    PSLR_SCENE_MODE_PORTRAIT,
    PSLR_SCENE_MODE_LANDSCAPE,
    PSLR_SCENE_MODE_MACRO,
    PSLR_SCENE_MODE_SPORT,
    PSLR_SCENE_MODE_NIGHTSCENEPORTRAIT,
    PSLR_SCENE_MODE_NOFLASH,
    PSLR_SCENE_MODE_NIGHTSCENE,
    PSLR_SCENE_MODE_SURFANDSNOW,
    PSLR_SCENE_MODE_TEXT,
    PSLR_SCENE_MODE_SUNSET,
    PSLR_SCENE_MODE_KIDS,
    PSLR_SCENE_MODE_PET,
    PSLR_SCENE_MODE_CANDLELIGHT,
    PSLR_SCENE_MODE_MUSEUM,
    PSLR_SCENE_MODE_19,
    PSLR_SCENE_MODE_FOOD,
    PSLR_SCENE_MODE_STAGE,
    PSLR_SCENE_MODE_NIGHTSNAP,
    PSLR_SCENE_MODE_SWALLOWDOF,
    PSLR_SCENE_MODE_24,
    PSLR_SCENE_MODE_NIGHTSCENEHDR,
    PSLR_SCENE_MODE_BLUESKY,
    PSLR_SCENE_MODE_FOREST,
    PSLR_SCENE_MODE_28,
    PSLR_SCENE_MODE_BLACKLIGHTSILHOUETTE,
    PSLR_SCENE_MODE_MAX
} pslr_scene_mode_t;

typedef enum {
    PSLR_BUF_PEF,
    PSLR_BUF_DNG,
    PSLR_BUF_JPEG_MAX,
    PSLR_BUF_JPEG_MAX_M1,
    PSLR_BUF_JPEG_MAX_M2,
    PSLR_BUF_JPEG_MAX_M3,
    PSLR_BUF_PREVIEW = 8,
    PSLR_BUF_THUMBNAIL = 9 // 7 works also
} pslr_buffer_type;

typedef enum {
    USER_FILE_FORMAT_PEF,
    USER_FILE_FORMAT_DNG,
    USER_FILE_FORMAT_JPEG,
    USER_FILE_FORMAT_MAX
} user_file_format;

typedef struct {
    user_file_format uff;
    const char *file_format_name;
    const char *extension;
} user_file_format_t;

extern user_file_format_t file_formats[3];

user_file_format_t *get_file_format_t( user_file_format uff );

// OFF-AUTO: Off-Auto-Aperture
typedef enum {
    PSLR_EXPOSURE_MODE_P = 0,
    PSLR_EXPOSURE_MODE_GREEN = 1,
//    PSLR_EXPOSURE_MODE_HYP = 2,
//    PSLR_EXPOSURE_MODE_AUTO_PICT = 1,
    PSLR_EXPOSURE_MODE_GREEN1 = 3, //maybe 1 is AUTO_PICT
    PSLR_EXPOSURE_MODE_TV = 4,
    PSLR_EXPOSURE_MODE_AV = 5,
//    PSLR_EXPOSURE_MODE_TV_SHIFT = 6, //?
//    PSLR_EXPOSURE_MODE_AV_SHIFT = 7, //?
    PSLR_EXPOSURE_MODE_M = 8,
    PSLR_EXPOSURE_MODE_B = 9,
    PSLR_EXPOSURE_MODE_AV_OFFAUTO = 10,
    PSLR_EXPOSURE_MODE_M_OFFAUTO = 11,
    PSLR_EXPOSURE_MODE_B_OFFAUTO = 12,
    PSLR_EXPOSURE_MODE_TAV = 13, // ?
    PSLR_EXPOSURE_MODE_SV = 15,
    PSLR_EXPOSURE_MODE_X = 16, // ?
    PSLR_EXPOSURE_MODE_MAX = 17
} pslr_exposure_mode_t;

typedef enum {
    PSLR_GUI_EXPOSURE_MODE_GREEN,
    PSLR_GUI_EXPOSURE_MODE_P,
    PSLR_GUI_EXPOSURE_MODE_SV,
    PSLR_GUI_EXPOSURE_MODE_TV,
    PSLR_GUI_EXPOSURE_MODE_AV,
    PSLR_GUI_EXPOSURE_MODE_TAV,
    PSLR_GUI_EXPOSURE_MODE_M,
    PSLR_GUI_EXPOSURE_MODE_B,
    PSLR_GUI_EXPOSURE_MODE_X,
    PSLR_GUI_EXPOSURE_MODE_MAX
} pslr_gui_exposure_mode_t;

typedef void *pslr_handle_t;

typedef struct {
    uint32_t a;
    uint32_t b;
    uint32_t addr;
    uint32_t length;
} pslr_buffer_segment_info;

typedef void (*pslr_progress_callback_t)(uint32_t current, uint32_t total);

#define MAX_RESOLUTION_SIZE 4
#define MAX_STATUS_BUF_SIZE 456
#define SETTINGS_BUFFER_SIZE 1024
#define MAX_SEGMENTS 4

typedef struct ipslr_handle ipslr_handle_t;

typedef struct {
    int32_t nom;
    int32_t denom;
} pslr_rational_t;

typedef struct {
    uint16_t bufmask;
    uint32_t current_iso;
    pslr_rational_t current_shutter_speed;
    pslr_rational_t current_aperture;
    pslr_rational_t lens_max_aperture;
    pslr_rational_t lens_min_aperture;
    pslr_rational_t set_shutter_speed;
    pslr_rational_t set_aperture;
    pslr_rational_t max_shutter_speed;
    uint32_t auto_bracket_mode;
    pslr_rational_t auto_bracket_ev;
    uint32_t auto_bracket_picture_count;
    uint32_t auto_bracket_picture_counter;
    uint32_t fixed_iso;
    uint32_t jpeg_resolution;
    uint32_t jpeg_saturation;
    uint32_t jpeg_quality;
    uint32_t jpeg_contrast;
    uint32_t jpeg_sharpness;
    uint32_t jpeg_image_tone;
    uint32_t jpeg_hue;
    pslr_rational_t zoom;
    int32_t focus;
    uint32_t image_format;
    uint32_t raw_format;
    uint32_t light_meter_flags;
    pslr_rational_t ec;
    uint32_t custom_ev_steps;
    uint32_t custom_sensitivity_steps;
    uint32_t exposure_mode;
    uint32_t scene_mode;
    uint32_t user_mode_flag;
    uint32_t ae_metering_mode;
    uint32_t af_mode;
    uint32_t af_point_select;
    uint32_t selected_af_point;
    uint32_t focused_af_point;
    uint32_t auto_iso_min;
    uint32_t auto_iso_max;
    uint32_t drive_mode;
    uint32_t shake_reduction;
    uint32_t white_balance_mode;
    uint32_t white_balance_adjust_mg;
    uint32_t white_balance_adjust_ba;
    uint32_t flash_mode;
    int32_t flash_exposure_compensation; /* 1/256 */
    int32_t manual_mode_ev; /* 1/10 */
    uint32_t color_space;
    uint32_t lens_id1;
    uint32_t lens_id2;
    uint32_t battery_1;
    uint32_t battery_2;
    uint32_t battery_3;
    uint32_t battery_4;
} pslr_status;

typedef enum {
    PSLR_SETTING_STATUS_UNKNOWN,
    PSLR_SETTING_STATUS_READ,
    PSLR_SETTING_STATUS_HARDWIRED,
    PSLR_SETTING_STATUS_NA
} pslr_setting_status_t;

typedef struct {
    pslr_setting_status_t pslr_setting_status;
    uint8_t value : 1u;
    uint8_t spare1 : 7u;
} pslr_bool_setting;

typedef struct {
    pslr_setting_status_t pslr_setting_status;
    uint16_t value;
} pslr_uint16_setting;

typedef struct {
    pslr_bool_setting one_push_bracketing;
    pslr_bool_setting bulb_mode_press_press;
    pslr_bool_setting bulb_timer;
    pslr_uint16_setting bulb_timer_sec;
    pslr_bool_setting using_aperture_ring;
    pslr_bool_setting shake_reduction;
    pslr_bool_setting astrotracer;
    pslr_uint16_setting astrotracer_timer_sec;
    pslr_bool_setting horizon_correction;
    pslr_bool_setting remote_bulb_mode_press_press;
} pslr_settings;

typedef struct {
    const char *name;
    uint32_t address;
    const char *value;
    const char *type;
} pslr_setting_def_t;

typedef void (*ipslr_status_parse_t)(ipslr_handle_t *p, pslr_status *status);

typedef struct {
    uint32_t id;                                                                // Pentax model ID
    const char *name;                                                           // name
    uint8_t old_scsi_command : 1u;                                              // true for *ist cameras, false for the newer cameras
    uint8_t old_bulb_mode : 1u;                                                 // true for older cameras
    uint8_t need_exposure_mode_conversion : 1u;                                 // is exposure_mode_conversion required
    uint8_t bufmask_command : 1u;                                               // true if bufmask determined by calling command 0x02 0x00
    uint8_t bufmask_single : 1u;                                                // true if buffer cannot handle multiple images
    uint8_t is_little_endian : 1u;                                              // whether the return value should be parsed as little-endian
    uint8_t spare2 : 2;
    int status_buffer_size;                                                     // status buffer size in bytes
    int max_jpeg_stars;                                                         // maximum jpeg stars
    int jpeg_resolutions[MAX_RESOLUTION_SIZE];                                  // jpeg resolution table
    int jpeg_property_levels;                                                   // 5 [-2, 2] or 7 [-3,3] or 9 [-4,4]
    int fastest_shutter_speed;                                                  // fastest shutter speed denominator
    int base_iso_min;                                                           // base iso minimum
    int base_iso_max;                                                           // base iso maximum
    int extended_iso_min;                                                       // extended iso minimum
    int extended_iso_max;                                                       // extended iso maximum
    pslr_jpeg_image_tone_t max_supported_image_tone;                            // last supported jpeg image tone
    uint8_t has_jpeg_hue : 1;                                                   // camera has jpeg hue setting
    uint8_t spare1 : 7;
    int af_point_num;                                                           // number of AF points
    ipslr_status_parse_t status_parser_function;                                // parse function for status buffer
} ipslr_model_info_t;

typedef struct {
    uint32_t offset;
    uint32_t addr;
    uint32_t length;
} ipslr_segment_t;

struct ipslr_handle {
    // FDTYPE fd;
    uint16_t fd;
    pslr_status status;
    pslr_settings settings;
    uint32_t id;
    ipslr_model_info_t *model;
    ipslr_segment_t segments[MAX_SEGMENTS];
    uint32_t segment_count;
    uint32_t offset;
    uint8_t status_buffer[MAX_STATUS_BUF_SIZE];
    uint8_t settings_buffer[SETTINGS_BUFFER_SIZE];
};

#ifdef enable_funcs                                                             // not implemented yet
void sleep_sec(double sec);
pslr_handle_t pslr_init(char *model, char *device);
int16_t pslr_connect(pslr_handle_t h);
int16_t pslr_disconnect(pslr_handle_t h);
int16_t pslr_shutdown(pslr_handle_t h);
const char *pslr_model(uint32_t id);
int16_t pslr_shutter(pslr_handle_t h);
int16_t pslr_focus(pslr_handle_t h);
int16_t pslr_get_status(pslr_handle_t h, pslr_status *sbuf);
int16_t pslr_get_status_buffer(pslr_handle_t h, uint8_t *st_buf);
int16_t pslr_get_settings(pslr_handle_t h, pslr_settings *ps);
int16_t pslr_get_settings_json(pslr_handle_t h, pslr_settings *ps);
int16_t pslr_get_settings_buffer(pslr_handle_t h, uint8_t *st_buf);
char *collect_status_info( pslr_handle_t h, pslr_status status );
char *collect_settings_info( pslr_handle_t h, pslr_settings settings );
int16_t pslr_get_buffer(pslr_handle_t h, int bufno, pslr_buffer_type type, int resolution, uint8_t **pdata, uint32_t *pdatalen);
int16_t pslr_set_progress_callback(pslr_handle_t h, pslr_progress_callback_t cb, uintptr_t user_data);
int16_t pslr_set_shutter(pslr_handle_t h, pslr_rational_t value);
int16_t pslr_set_aperture(pslr_handle_t h, pslr_rational_t value);
int16_t pslr_set_iso(pslr_handle_t h, uint32_t value, uint32_t auto_min_value, uint32_t auto_max_value);
int16_t pslr_set_ec(pslr_handle_t h, pslr_rational_t value);
int16_t pslr_set_white_balance(pslr_handle_t h, pslr_white_balance_mode_t wb_mode);
int16_t pslr_set_white_balance_adjustment(pslr_handle_t h, pslr_white_balance_mode_t wb_mode, uint32_t wbadj_mg, uint32_t wbadj_ba);
int16_t pslr_set_flash_mode(pslr_handle_t h, pslr_flash_mode_t value);
int16_t pslr_set_flash_exposure_compensation(pslr_handle_t h, pslr_rational_t value);
int16_t pslr_set_drive_mode(pslr_handle_t h, pslr_drive_mode_t drive_mode);
int16_t pslr_set_af_mode(pslr_handle_t h, pslr_af_mode_t af_mode);
int16_t pslr_set_af_point_sel(pslr_handle_t h, pslr_af_point_sel_t af_point_sel);
int16_t pslr_set_ae_metering_mode(pslr_handle_t h, pslr_ae_metering_t ae_metering_mode);
int16_t pslr_set_color_space(pslr_handle_t h, pslr_color_space_t color_space);
int16_t pslr_set_jpeg_stars(pslr_handle_t h, int jpeg_stars);
int16_t pslr_set_jpeg_resolution(pslr_handle_t h, int megapixel);
int16_t pslr_set_jpeg_image_tone(pslr_handle_t h, pslr_jpeg_image_tone_t image_mode);
int16_t pslr_set_jpeg_sharpness(pslr_handle_t h, int32_t sharpness);
int16_t pslr_set_jpeg_contrast(pslr_handle_t h, int32_t contrast);
int16_t pslr_set_jpeg_saturation(pslr_handle_t h, int32_t saturation);
int16_t pslr_set_jpeg_hue(pslr_handle_t h, int32_t hue);
int16_t pslr_set_image_format(pslr_handle_t h, pslr_image_format_t format);
int16_t pslr_set_raw_format(pslr_handle_t h, pslr_raw_format_t format);
int16_t pslr_set_user_file_format(pslr_handle_t h, user_file_format uff);
user_file_format get_user_file_format( pslr_status *st );
int16_t pslr_delete_buffer(pslr_handle_t h, int bufno);
int16_t pslr_green_button(pslr_handle_t h);
int16_t pslr_button_test(pslr_handle_t h, int bno, int arg);
int16_t pslr_ae_lock(pslr_handle_t h, bool lock);
int16_t pslr_dust_removal(pslr_handle_t h);
int16_t pslr_bulb(pslr_handle_t h, bool on );
int16_t pslr_buffer_open(pslr_handle_t h, int bufno, pslr_buffer_type type, int resolution);
uint32_t pslr_buffer_read(pslr_handle_t h, uint8_t *buf, uint32_t size);
uint32_t pslr_fullmemory_read(pslr_handle_t h, uint8_t *buf, uint32_t offset, uint32_t size);
void pslr_buffer_close(pslr_handle_t h);
uint32_t pslr_buffer_get_size(pslr_handle_t h);
int16_t pslr_set_exposure_mode(pslr_handle_t h, pslr_exposure_mode_t mode);
int16_t pslr_select_af_point(pslr_handle_t h, uint32_t point);
const char *pslr_camera_name(pslr_handle_t h);
int16_t pslr_get_model_max_jpeg_stars(pslr_handle_t h);
int16_t pslr_get_model_jpeg_property_levels(pslr_handle_t h);
int16_t pslr_get_model_status_buffer_size(pslr_handle_t h);
int16_t pslr_get_model_fastest_shutter_speed(pslr_handle_t h);
int16_t pslr_get_model_base_iso_min(pslr_handle_t h);
int16_t pslr_get_model_base_iso_max(pslr_handle_t h);
int16_t pslr_get_model_extended_iso_min(pslr_handle_t h);
int16_t pslr_get_model_extended_iso_max(pslr_handle_t h);
int16_t *pslr_get_model_jpeg_resolutions(pslr_handle_t h);
bool pslr_get_model_only_limited(pslr_handle_t h);
bool pslr_get_model_has_jpeg_hue(pslr_handle_t h);
bool pslr_get_model_need_exposure_conversion(pslr_handle_t h);
pslr_jpeg_image_tone_t pslr_get_model_max_supported_image_tone(pslr_handle_t h);
bool pslr_get_model_has_settings_parser(pslr_handle_t h);
int16_t pslr_get_model_af_point_num(pslr_handle_t h);
bool pslr_get_model_old_bulb_mode(pslr_handle_t h);
bool pslr_get_model_bufmask_single(pslr_handle_t h);
pslr_buffer_type pslr_get_jpeg_buffer_type(pslr_handle_t h, int quality);
int16_t pslr_get_jpeg_resolution(pslr_handle_t h, int hwres);
int16_t pslr_read_datetime(pslr_handle_t *h, int *year, int *month, int *day, int *hour, int *min, int *sec);
int16_t pslr_read_dspinfo(pslr_handle_t *h, char *firmware);
int16_t pslr_read_setting(pslr_handle_t *h, int offset, uint32_t *value);
int16_t pslr_write_setting(pslr_handle_t *h, int offset, uint32_t value);
int16_t pslr_write_setting_by_name(pslr_handle_t *h, char *name, uint32_t value);
bool pslr_has_setting_by_name(pslr_handle_t *h, char *name);
int16_t pslr_read_settings(pslr_handle_t *h);
pslr_gui_exposure_mode_t exposure_mode_conversion( pslr_exposure_mode_t exp );
char *format_rational( pslr_rational_t rational, char * fmt );
int16_t pslr_test( pslr_handle_t h, bool cmd9_wrap, int subcommand, int argnum,  int arg1, int arg2, int arg3, int arg4);
char *copyright(void);
void write_debug( const char* message, ... );
int16_t debug_onoff(ipslr_handle_t *p, char debug_mode);
#endif // enablefuncs

#ifdef __cplusplus
}
#endif

#endif