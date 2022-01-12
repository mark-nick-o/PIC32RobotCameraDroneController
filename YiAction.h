#ifndef __Yi_c
#define __Yi_c

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//
// YiAction.h : Talk to Yi Action Camera via JSON objects ajax messages
//
// Written by (C) 2020 A C P Avaiation Walkerburn Scotland
//
// Macro to give options for video photo sizes and quality
// Use the enumerated type to get the corresponding string definitions
// Refers to method invented by Stefan Ram
//
#define vNAMES C(video_standard)C(NTSC)C(PAL)                                   // Enumerated type options
#define C(x) x,                                                                 // enum type def
#define dNAMES D(VIDRESPARAM_1)D(VIDRESPARAM_2)D(VIDRESPARAM_3)D(VIDRESPARAM_4)D(NTSC_1920_1080_60P)D(NTSC_1920_1080_30P)D(NTSC_1280_960_60P)D(NTSC_1280_720_60P)D(NTSC_1280_720_120P)D(NTSC_848_480_240P)D(PAL_1920_1080_48P)D(PAL_1920_1080_24P)D(PAL_1920_960_48P)D(PAL_1920_720_48P)  // {"msg_id":9,"param":"video_resolution","token":1} options listed after
#define NO_OF_VID_RES_TYPE 4U                                                   // Where you have more than one type then specify
#define XY_NO_OF_NTSC 6U                                                        // Number of NTSC parameters above
#define D(x) x,
#define eNAMES E(photo_sz)E(R16MP)E(R13MP)E(R8MP)E(R5MP)
#define E(x) x,
#define fNAMES F(vid_qual)F(photo_qual)F(SUPER_FINE)F(FINE)F(NORMAL)
#define F(x) x,
#define hNAMES H(video_stamp_val)H(photo_stamp_val)H(OFF)H(DATE)H(TIME)H(DATE_TIME)
#define H(x) x,
#define bNAMES B(param_burst_cap_no)B(p3_s)B(p5_s)B(p10_s)B(p10_2s)B(p10_3s)B(p20_2s)B(p30_3s)B(p30_6s)   //  msg_id=9 param=id0 + options
#define B(x) x,
#define aNAMES A(precise_cont_time)A(pct3s)A(pct10s)A(pct15s)A(pct20s)A(pct30s) //  msg_id=9 param=id0 + options
#define A(x) x,
#define jNAMES J(slow_motion_rate)J(smr2)J(smr4)J(smr8)                         //  msg_id=9 param=id0 + options
#define J(x) x,
#define kNAMES K(timelapse_video)K(tvhalf)K(tv1)K(tv2)K(tv5)K(tv10)K(tv30)K(tv60)K(tvon)K(tvoff)     //  msg_id=9 param=id0 + options
#define K(x) x,
#define lNAMES L(timelapse_video_duration)L(tvdoff)L(tvd6s)L(tvd8s)L(tvd10s)L(tvd20s)L(tvd30s)L(tvd60s)L(tvd120s)     //  msg_id=9 param=id0 + options
#define L(x) x,
#define mNAMES M(record_photo_time)M(rpt5)M(rpt10)M(rpt30)M(rpt60)              //  msg_id=9 param=id0 + options
#define M(x) x,
#define nNAMES N(loop_rec_duration)N(lrd5min)N(lrd20mins)N(lrd60mins)N(lrd120mins)N(lrdmax)  //  msg_id=9 param=id0 + options
#define N(x) x,
#define oNAMES O(video_stamp)O(photo_stamp)O(preview_status)O(buzzer_ring)O(restore_factory_settings)O(dev_reboot)O(dual_stream_status)O(streaming_status)O(precise_cont_capturing)O(piv_enable)O(support_auto_low_light)O(precise_self_running)O(start_wifi_while_booted)O(video_rotate)O(iq_eis_enable)O(protune)O(screen_auto_lock)O(dewarp_support_status)O(eis_support_status)O(video_volume_set)O(wifi_country_editable)O(rec_audio_support)O(support_iso)O(support_wb)O(support_sharpness)O(led_mode)O(auto_power_off)O(ev_enable)O(el_warp_correction)O(el_stabilization)O(el_self_timer)O(std_def_video)O(stream_while_record)O(el_auto_power_off)O(el_image_rotation)O(auto_low_light)O(loop_record)O(warp_enable)O(emergency_file_backup)O(osd_enable)
#define O(x) x,
#define pNAMES P(video_output_dev_type)P(hdmi)P(tv)P(no_out_dev)
#define P(x) x,
#define qNAMES Q(meter_mode)Q(center)Q(average)Q(spot)
#define Q(x) x,
#define rNAMES R(led_mode_val)R(all_enable)R(all_disable)R(status_enable)
#define R(x) x,
#define sNAMES S(buzzer_volume)S(high)S(low)S(mute)
#define S(x) x,
#define tNAMES T(capture_mode)T(capture_default_mode)T(precise_quality)T(precise_quality_cont)T(burst_quality)T(precise_self_quality)
#define T(x) x,
enum xy_vid_std_e { vNAMES MAX_VID_STD };                                            // Enumerated Types requested
enum xy_vid_res_e { dNAMES MAX_VID_RES };
enum xy_photo_sz_e { eNAMES MAX_PHOTO_SZ };
enum xy_quality_e { fNAMES MAX_QUALITY };
enum xy_stamp_e { hNAMES MAX_STAMP };
enum xy_burst_e { bNAMES MAX_BURST };
enum xy_cont_time_e { aNAMES MAX_CONT_TIME };
enum xy_slowmot_e { jNAMES MAX_SLOWMOT };
enum xy_timelap_e { kNAMES MAX_TIMELAP };
enum xy_tlpvid_e { lNAMES MAX_TLPVID };
enum xy_rectim_e { mNAMES MAX_RECTIM };
enum xy_looptim_e { nNAMES MAX_LOOPTIM };
enum xy_onofftypes_e { oNAMES MAX_ONOFF };
enum xy_vidoutdev_e { pNAMES MAX_VIDOUT };
enum xy_meter_e { qNAMES MAX_METER };
enum xy_led_e { rNAMES MAX_LED };
enum xy_buzzer_e { sNAMES MAX_BUZZER };
enum xy_capmode_e { tNAMES MAX_CAP_MODE };
#undef C                                                                        // undef to allow re-definition of string
#undef D
#undef E
#undef F
#undef H
#undef B
#undef A
#undef J
#undef K
#undef L
#undef M
#undef N
#undef O
#undef P
#undef Q
#undef R
#undef S
#undef T
#undef dNAMES                                                                   // Actual JSON string parameters
#undef eNAMES
#undef fNAMES
#undef hNAMES
#undef bNAMES
#undef aNAMES
#undef jNAMES
#undef kNAMES
#undef lNAMES
#undef mNAMES
#undef nNAMES
#undef oNAMES
#undef pNAMES
#undef qNAMES
#undef rNAMES
#undef sNAMES
#undef tNAMES
#define dNAMES D("video_loop_resolution")D("video_photo_resolution")D("timelapse_video_resolution")D("video_resolution")D("1920x1080 60P 16:9")D("1920x1080 30P 16:9")D("1280x960 60P 4:3")D("1280x720 60P 16:9")D("1280x720 120P 16:9")D("848x480 240P 16:9")D("848x480 240P 16:9")D("1920x1080 24P 16:9")D("1280x960 48P 4:3")D("1280x720 48P 16:9")
#define eNAMES E("photo_size")E("16M (4608x3456 4:3)")E("13M (4128x3096 4:3)")E("8M (3264x2448 4:3)")E("5M (2560x1920 4:3)")
#define fNAMES F("video_quality")F("photo_quality")F("S.Fine")F("Fine")F("Normal")
#define hNAMES H("video_stamp")H("photo_stamp")H("off")H("date")H("time")H("date/time")
#define bNAMES B("burst_capture_number")B("3 p / s")B("5 p / s")B("10 p / s")B("10 p / 2s")B("10 p / 3s")B("20 p / 2s")B("30 p / 3s")B("30 p / 6s")
#define aNAMES A("precise_cont_time")A("3s")A("10s")A("15s")A("20s")A("30s")
#define jNAMES J("slow_motion_rate")J("2")J("4")J("8")                          //  msg_id=9 param=id0 + options
#define kNAMES K("timelapse_video")K("0.5")K("1")K("2")K("5")K("10")K("30")K("60")K("on")K("off")     //  msg_id=9 param=id0 + options
#define lNAMES L("timelapse_video_duration")L("off")L("6s")L("8s")L("10s")L("20s")L("30s")L("60s")L("120s")
#define mNAMES M("record_photo_time")M("5")M("10")M("30")M("60")
#define nNAMES N("loop_rec_duration")N("5 minutes")N("20 minutes")N("60 minutes")N("120 minutes")N("max")
#define oNAMES O("video_stamp")O("photo_stamp")O("preview_status")O("buzzer_ring")O("restore_factory_settings")O("dev_reboot")O("dual_stream_status")O("streaming_status")O("precise_cont_capturing")O("piv_enable")O("support_auto_low_light")O("precise_self_running")O("start_wifi_while_booted")O("video_rotate")O("iq_eis_enable")O("protune")O("screen_auto_lock")O("dewarp_support_status")O("eis_support_status")O("video_volume_set")O("wifi_country_editable")O("rec_audio_support")O("support_iso")O("support_wb")O("support_sharpness")O("led_mode")O("auto_power_off")O("ev_enable")O("el_warp_correction")O("el_stabilization")O("el_self_timer")O("std_def_video")O("stream_while_record")O("el_auto_power_off")O("el_image_rotation")O("auto_low_light")O("loop_record")O("warp_enable")O("emergency_file_backup")O("osd_enable")
#define pNAMES P("video_output_dev_type")P("hdmi")P("tv")P("off")
#define qNAMES Q("meter_mode")Q("center")Q("average")Q("spot")
#define rNAMES R("led_mode")R("all enable")R("all disable")R("status enable")
#define sNAMES S("buzzer_volume")S("high")S("low")S("mute")
#define tNAMES T("capture_mode")T("capture_default_mode")T("precise quality")T("precise quality cont.")T("burst quality")T("precise self quality")
#define C(x) #x,                                                                // string concat
#define D(x) #x,
#define E(x) #x,
#define F(x) #x,
#define H(x) #x,
#define B(x) #x,
#define A(x) #x,
#define J(x) #x,
#define K(x) #x,
#define L(x) #x,
#define M(x) #x,
#define N(x) #x,
#define O(x) #x,
#define P(x) #x,
#define Q(x) #x,
#define R(x) #x,
#define S(x) #x,
#define T(x) #x,
const char * const xy_video_standard[] = { vNAMES };                            // Array of string definitions as per enum
const char * const xy_video_res[] = { dNAMES };                                 // as per string re-def
const char * const xy_photo_sz[] = { eNAMES };                                  // Array of photo sizes
const char * const xy_vidpho_qual[] = { fNAMES };                               // Array of video quality
const char * const xy_vidpho_stamp[] = { hNAMES };                              // Array of video stamps
const char * const xy_burstcap_opt[] = { bNAMES };                              // Array of bust capture options id=0 = param
const char * const xy_conttime_opt[] = { aNAMES };                              // Array of precise cont TIME options options id=0 = param
const char * const xy_slowmot_opt[] = { jNAMES };                               // Array of slow motion options options id=0 = param
const char * const xy_timelap_opt[] = { kNAMES };                               // Array of TIME lapse options options id=0 = param
const char * const xy_tlpvid_opt[] = { lNAMES };                                // Array of TIME lapse duration options options id=0 = param
const char * const xy_rectim_opt[] = { mNAMES };                                // Array of photo record time options options id=0 = param
const char * const xy_looptim_opt[] = { nNAMES };                               // Array of loop duration options options id=0 = param
const char * const xy_onofftypes_opt[] = { oNAMES };                            // Array of parameters that have "on" or "off" states
const char * const xy_vidoutdev_opt[] = { pNAMES };                             // Array of video output devices
const char * const xy_meter_opt[] = { qNAMES };                                 // Array of meter modes
const char * const xy_led_opt[] = { rNAMES };                                   // Array of led modes
const char * const xy_buzzer_opt[] = { sNAMES };                                // Array of buzzer loadness modes
const char * const xy_capmode_opt[] = { tNAMES };                               // Array of capture modes

// ============================================================================================================================================================
//
// The usage of these macros and enumerated types which produce strings is shown below
//
// printf( "\"%s\": %s %s: %s %s: %s %s: %s %s: %s\r", xy_video_standard[ video_standard ],xy_video_standard[ NTSC ],xy_video_res[TYPE_NAME],xy_video_res[ NTSC_848_480_240P ],xy_photo_sz[photo_sz],xy_photo_sz[R5MP],xy_vidpho_qual[TYPE_NAME3],xy_vidpho_qual[SUPER_FINE],xy_vidpho_stamp[TYPE_NAME4],xy_vidpho_stamp[DATE]);
//  printf( "There are %d video standards and %d video resolutions and %d photo sizes and %d qualities %d stamps\n", COUNT1-1,COUNT2-1,COUNT3-1,COUNT4-2,COUNT5-2 ); }
//
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif