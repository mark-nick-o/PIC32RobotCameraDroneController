#ifndef __sony_visca_camera_h
#define __sony_visca_camera_h
//______________________________________________________________________________
//
//    Support the Sony VISCA protocol either over serial or UDP/TCP
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//______________________________________________________________________________

#include "definitions.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __GNUC__                                                                 // pack the structures so as not to waste memory
  #define SOCAMPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#else
  #define SOCAMPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define SONY_LIB extern
#define jump_table                                                              // if you use jump table or not

#define VISCA_UDP_PORT 52381U                                                   // UDP port to use for Sony Visca
#define VISCA_CAM_ADDR 1u                                                       // camera address
#define VISCA_MAX_MSG_LEN 16U                                                   // max visca message length
#define VISCA_MAX_MEMORIES 9u                                                   // has nine memory stores
#define VISCA_MAX_PAN_SPEED 0x18u
#define VISCA_MAX_TILT_SPEED 0x14u

#define VISCA_SCALE_PQ_TO_P(NUM) ((uint8_t)((NUM >> 4u) & (0x0Fu)))             // gives first number p i.e. 0x34 p=3
#define VISCA_SCALE_PQ_TO_Q(NUM) ((uint8_t)(NUM & 0x0Fu))                       // gives the second number i.e. 0x34 q=4
#define VISCA_SCALE_PQRS_TO_P(NUM) ((uint8_t)((NUM >> 0x0Cu) & (0x0Fu)))        // gives first number p i.e. 3412 p=3
#define VISCA_SCALE_PQRS_TO_Q(NUM) ((uint8_t)((NUM >> 0x08u) & (0x0Fu)))        // gives second number p i.e. 3412 q=4
#define VISCA_SCALE_PQRS_TO_R(NUM) ((uint8_t)((NUM >> 0x04u) & (0x0Fu)))        // gives third number p i.e. 3412 r=1
#define VISCA_SCALE_PQRS_TO_S(NUM) ((uint8_t)(NUM & 0x0Fu))                     // gives the fourth number i.e. 3412 s=2
#define VISCA_PQ_TO_SCALE(p,q,NUM) { NUM=((p<<4u)+q); }                         // NUM=pq
#define VISCA_PQRS_TO_SCALE(p,q,r,s,NUM) { NUM=(((s+(r<<4u))+(q<<8u))+(p<<12u)); }    // NUM=pqrs

// ============================== Message Definition ===========================
//
// =============================================================================
//Ack/completion messages
//Command packet        Note
//z0 41 FF        Returned when the command is accepted (Ack)
//z0 51 FF        Returned when the command has been executed (completion).
#define VISCA_MSG_LEN 3u
#define VISCA_SND_ACK(A) { ((A+8u)<<4U),0x41u,0xffu }
#define VISCA_RET_ACK(A) { ((A+8u)<<4U),0x51u,0xffu }
// ================ CommandCancel ==============================================
#define VISCA_CAM_CMD_CANCEL(A) { (0x80u+A),0x21u,0xFFu }                       // 8x 21 FF

//Error messages
//Description        Command Packet        Note
//Syntax Error        z0 60 02 FF        Returned when the command format is different or when a command with illegal command parameters is accepted.
//Command Not Executable        z0 61 41 FF        Returned when a command cannot be executed due to current conditions. For example, when commands controlling the focus manually are received during auto focus.
#undef VISCA_MSG_LEN
#define VISCA_MSG_LEN 4u                                                        // =============== FOUR LONG ============================================
#define VISCA_RET_SYNTX_ERR(A) { ((A+8u)<<4U),0x60u,0x02u,0xFFu }
#define VISCA_RET_NOT_EXE(A) { ((A+8u)<<4U),0x61u,0x41u,0xffu }
// ================ AddressSet Broadcast =======================================
#define VISCA_CAM_ADDR_SET { 0x88u,0x30u,0x01u,0xFFu }                          // 88 30 01 FF
//================ Control commands
//VISCA control commands allow you to customize your LUMiO 12x settings and issue movement commands.
// ================ IF_Clear        Broadcast  =================================
#undef VISCA_MSG_LEN
#define VISCA_MSG_LEN 5u                                                        // ================ FIVE LONG ============================================
#define VISCA_CAM_IF_CLR { 0x88u,0x01u,0x00u,0x01u,0xFFu }                      // 88 01 00 01 FF
// =============== CAM_Power ===================================================
#undef VISCA_MSG_LEN
#define VISCA_MSG_LEN 6u                                                        // ==================== SIX LONG =========================================
#define VISCA_CAM_PWR_ON(A) { (0x80u+A),0x01u,0x04u,0x00u,0x02u,0xFFu }         // 8x 01 04 00 02 FF
// Cam Power Off
#define VISCA_CAM_PWR_OFF(A) { (0x80u+A),0x01u,0x04u,0x00u,0x03u,0xFFu }        // 8x 01 04 00 03 FF
// ================ CAM_Zoom ===================================================
#define VISCA_CAM_ZOOM_STOP(A) { (0x80u+A),0x01u,0x04u,0x07u,0x00u,0xFFu }      // Stop 8x 01 04 07 00 FF
#define VISCA_CAM_ZOOM_TELE_STD(A) { (0x80u+A),0x01u,0x04u,0x07u,0x02u,0xFFu }  // Tele(Standard) 8x 01 04 07 02 FF
#define VISCA_CAM_ZOOM_WIDE_STD(A) { (0x80u+A),0x01u,0x04u,0x07u,0x03u,0xFFu }  // Wide(Standard) 8x 01 04 07 03 FF
#define VISCA_CAM_ZOOM_TELE_VAR(A,p) { (0x80u+A),0x01u,0x04u,0x07u,(0x20u+p),0xFFu }  // Tele(Variable) 8x 01 04 07 2p FF p = 0(low)~7(high)
#define VISCA_CAM_ZOOM_WIDE_VAR(A,p) { (0x80u+A),0x01u,0x04u,0x07u,(0x30u+p),0xFFu }   // Wide(Variable) 8x 01 04 07 3p FF = 0(low)~7(high
// ================= CAM_Focus =================================================
#define VISCA_CAM_FOCUS_STOP(A) { (0x80u+A),0x01u,0x04u,0x08u,0x00u,0xFFu }     // Stop 8x 01 04 08 00 FF
// CAM_Focus
#define VISCA_CAM_FOCUS_FAR_STD(A) { (0x80u+A),0x01u,0x04u,0x08u,0x02u,0xFFu }  // Far(Standard) 8x 01 04 08 02 FF
// CAM_Focus
#define VISCA_CAM_FOCUS_NEAR_STD(A) { (0x80u+A),0x01u,0x04u,0x08u,0x03u,0xFFu } // Near(Standard) 8x 01 04 08 03 FF
// CAM_Focus
#define VISCA_CAM_FOCUS_1PSH_AF(A) { (0x80u+A),0x01u,0x04u,0x18u,0x01u,0xFFu }  // One Push AF 8x 01 04 18 01 FF
// =============== CAM_WB ======================================================
#define VISCA_CAM_WB_AUTO(A) { (0x80u+A),0x01u,0x04u,0x35u,0x00u,0xFFu }        // Auto 8x 01 04 35 00 FF
// CAM_WB
#define VISCA_CAM_WB_INDOOR(A) { (0x80u+A),0x01u,0x04u,0x35u,0x01u,0xFFu }      // Indoor 8x 01 04 35 01 FF
// CAM_WB
#define VISCA_CAM_WB_OUTDOOR(A) { (0x80u+A),0x01u,0x04u,0x35u,0x02u,0xFFu }     // Outdoor 8x 01 04 35 02 FF
// CAM_WB
#define VISCA_CAM_WB_1PSH(A) { (0x80u+A),0x01u,0x04u,0x35u,0x03u,0xFFu }        // OnePush 8x 01 04 35 03 FF
// CAM_WB
#define VISCA_CAM_WB_MAN(A) { (0x80u+A),0x01u,0x04u,0x35u,0x05u,0xFFu }         // Manual 8x 01 04 35 05 FF
// CAM_WB
#define VISCA_CAM_WB_OUT_AUTO(A) { (0x80u+A),0x01u,0x04u,0x35u,0x06u,0xFFu }    // Outdoor Auto 8x 01 04 35 06 FF
// CAM_WB
#define VISCA_CAM_WB_SOD_LMP_AUTO(A) { (0x80u+A),0x01u,0x04u,0x35u,0x07u,0xFFu }// Sodium Lamp Auto 8x 01 04 35 07 FF
// CAM_WB
#define VISCA_CAM_WB_SOD_AUTO(A) { (0x80u+A),0x01u,0x04u,0x35u,0x08u,0xFFu }    // Sodium Auto 8x 01 04 35 08 FF
// =============== CAM_RGain ===================================================
#define VISCA_CAM_RGAIN_RESET(A) { (0x80u+A),0x01u,0x04u,0x03u,0x00u,0xFFu }    // Reset 8x 01 04 03 00 FF
// CAM_RGain
#define VISCA_CAM_RGAIN_UP(A) { (0x80u+A),0x01u,0x04u,0x03u,0x02u,0xFFu }       // Up 8x 01 04 03 02 FF
// CAM_RGain
#define VISCA_CAM_RGAIN_DOWN(A) { (0x80u+A),0x01u,0x04u,0x03u,0x03u,0xFFu }     // Down 8x 01 04 03 03 FF
// ============== CAM_Bgain ====================================================
#define VISCA_CAM_BGAIN_RESET(A) { (0x80u+A),0x01u,0x04u,0x04u,0x00u,0xFFu }    // Reset 8x 01 04 04 00 FF
// CAM_BGain
#define VISCA_CAM_BGAIN_UP(A) { (0x80u+A),0x01u,0x04u,0x04u,0x02u,0xFFu }       // Up 8x 01 04 04 02 FF
// CAM_BGain
#define VISCA_CAM_BGAIN_DOWN(A) { (0x80u+A),0x01u,0x04u,0x04u,0x03u,0xFFu }     // Down 8x 01 04 04 03 FF
// ============= CAM_Shutter ===================================================
#define VISCA_CAM_SHUT_RESET(A) { (0x80u+A),0x01u,0x04u,0x0Au,0x00u,0xFFu }     // Reset 8x 01 04 0A 00 FF
// CAM_Shutter
#define VISCA_CAM_SHUT_UP(A) { (0x80u+A),0x01u,0x04u,0x0Au,0x02u,0xFFu }        // Up 8x 01 04 0A 02 FF
// CAM_Shutter
#define VISCA_CAM_SHUT_DOWN(A) { (0x80u+A),0x01u,0x04u,0x0Au,0x03u,0xFFu }      // Down 8x 01 04 0A 03 FF
// ============ CAM_Iris =======================================================
#define VISCA_CAM_IRIS_RESET(A) { (0x80u+A),0x01u,0x04u,0x0Bu,0x00u,0xFFu }     // Reset 8x 01 04 0B 00 FF
// CAM_Iris
#define VISCA_CAM_IRIS_UP(A) { (0x80u+A),0x01u,0x04u,0x0Bu,0x02u,0xFFu }        // Up 8x 01 04 0B 02 FF
// CAM_Iris
#define VISCA_CAM_IRIS_DOWN(A) { (0x80u+A),0x01u,0x04u,0x0Bu,0x03u,0xFFu }      // Down 8x 01 04 0B 03 FF
// ============ CAM_Gain =======================================================
#define VISCA_CAM_GAIN_RESET(A) { (0x80u+A),0x01u,0x04u,0x0Cu,0x00u,0xFFu }     // Reset 8x 01 04 0C 00 FF
// CAM_Gain
#define VISCA_CAM_GAIN_UP(A) { (0x80u+A),0x01u,0x04u,0x0Cu,0x02u,0xFFu }        // Up 8x 01 04 0C 02 FF
// CAM_Gain
#define VISCA_CAM_GAIN_DOWN(A) { (0x80u+A),0x01u,0x04u,0x0Cu,0x03u,0xFFu }      // Down 8x 01 04 0C 03 FF
// =========== CAM_Bright ======================================================
#define VISCA_CAM_BRIGHT_RESET(A) { (0x80u+A),0x01u,0x04u,0x0Du,0x00u,0xFFu }   // Reset 8x 01 04 0D 00 FF
// CAM_Bright
#define VISCA_CAM_BRIGHT_UP(A) { (0x80u+A),0x01u,0x04u,0x0Du,0x02u,0xFFu }      // Up 8x 01 04 0D 02 FF
// CAM_Bright
#define VISCA_CAM_BRIGHT_DOWN(A) { (0x80u+A),0x01u,0x04u,0x0Du,0x03u,0xFFu }    // Down 8x 01 04 0D 03 FF
// ========== CAM_Aperture =====================================================
#define VISCA_CAM_AP_RESET(A) { (0x80u+A),0x01u,0x04u,0x02u,0x00u,0xFFu }       // Reset 8x 01 04 02 00 FF
// CAM_Aperture
#define VISCA_CAM_AP_UP(A) { (0x80u+A),0x01u,0x04u,0x02u,0x02u,0xFFu }          // Up 8x 01 04 02 02 FF
// CAM_Aperture
#define VISCA_CAM_AP_DOWN(A) { (0x80u+A),0x01u,0x04u,0x02u,0x03u,0xFFu }        // Down 8x 01 04 02 03 FF
// ========== CAM_ExpComp ======================================================
#define VISCA_CAM_EXP_RESET(A) { (0x80u+A),0x01u,0x04u,0x0Eu,0x00u,0xFFu }      // Reset 8x 01 04 0E 00 FF
// CAM_ExpComp
#define VISCA_CAM_EXP_UP(A) { (0x80u+A),0x01u,0x04u,0x0Eu,0x02u,0xFFu }         // Up 8x 01 04 0E 02 FF
// CAM_ExpComp
#define VISCA_CAM_EXP_DOWN(A) { (0x80u+A),0x01u,0x04u,0x0Eu,0x03u,0xFFu }       // Down 8x 01 04 0E 03 FF
// CAM_ExpComp
#define VISCA_CAM_EXP_DIRECT(A,p,q) { (0x80u+A),0x01u,0x04u,0x4Eu,0x00u,0x00u,p,q,0xFFu } // Direct 8x 01 04 4E 00 00 0p 0q FF
// CAM_ExpComp
#define VISCA_CAM_EXP_ON(A) { (0x80u+A),0x01u,0x04u,0x3Eu,0x02u,0xFFu }         // On 8x 01 04 3E 02 FF
// CAM_ExpComp
#define VISCA_CAM_EXP_OFF(A) { (0x80u+A),0x01u,0x04u,0x3Eu,0x03u,0xFFu }        // Off 8x 01 04 3E 03 FF
// ========== CAM_BackLight ====================================================
#define VISCA_CAM_BK_LT_ON(A) { (0x80u+A),0x01u,0x04u,0x33u,0x02u,0xFFu }       // On 8x 01 04 33 02 FF
// CAM_BackLight
#define VISCA_CAM_BK_LT_OFF(A) { (0x80u+A),0x01u,0x04u,0x33u,0x03u,0xFFu }      // Off 8x 01 04 33 03 FF
// ========== CAM_LR_Reverse ===================================================
#define VISCA_CAM_LR_REV_ON(A) { (0x80u+A),0x01u,0x04u,0x61u,0x02u,0xFFu }      // On 8x 01 04 61 02 FF
// CAM_LR_Reverse
#define VISCA_CAM_LR_REV_OFF(A) { (0x80u+A),0x01u,0x04u,0x61u,0x03u,0xFFu }     // Off 8x 01 04 61 03 FF
// ========== CAM_PictureFlip ==================================================
#define VISCA_CAM_PFLIP_ON(A) { (0x80u+A),0x01u,0x04u,0x66u,0x02u,0xFFu }       // On 8x 01 04 66 02 FF
// CAM_PictureFlip
#define VISCA_CAM_PFLIP_OFF(A) { (0x80u+A),0x01u,0x04u,0x66u,0x03u,0xFFu }      // Off 8x 01 04 66 03 FF
// ========= CAM_MountMode =====================================================
#define VISCA_CAM_MM_UP(A) { (0x80u+A),0x01u,0x04u,0xA4u,0x02u,0xFFu }          // UP 8x 01 04 A4 02 FF
// CAM_MountMode
#define VISCA_CAM_MM_DOWN(A) { (0x80u+A),0x01u,0x04u,0xA4u,0x03u,0xFFu }        // DOWN 8x 01 04 A4 03 FF
// ========= IR_Transfer =======================================================
#define VISCA_CAM_IRT_ON(A) { (0x80u+A),0x01u,0x06u,0x1Au,0x02u,0xFFu }         // On 8x 01 06 1A 02 FF
// IR_Transfer
#define VISCA_CAM_IRT_OFF(A) { (0x80u+A),0x01u,0x06u,0x1Au,0x03u,0xFFu }        // Off 8x 01 06 1A 03 FF
// ========= IR_Receive ========================================================
#define VISCA_CAM_IRR_TOGGLE(A) { (0x80u+A),0x01u,0x06u,0x08u,0x10u,0xFFu }      // On/Off 8x 01 06 08 10 FF
// IR_Receive
#define VISCA_CAM_IRR_ON(A) { (0x80u+A),0x01u,0x06u,0x08u,0x02u,0xFFu }          // On 8x 01 06 08 02 FF
// IR_Receive
#define VISCA_CAM_IRR_OFF(A) { (0x80u+A),0x01u,0x06u,0x08u,0x03u,0xFFu }         // Off 8x 01 06 08 03 FF
// ========= FLICK =============================================================
#define VISCA_CAM_FLICK_50Hz { 0x81u,0x01u,0x04u,0x23u,0x01u,0xFFu }            // 50Hz 81 01 04 23 01 FF
// FLICK
#define VISCA_CAM_FLICK_60Hz { 0x81u,0x01u,0x04u,0x23u,0x02u,0xFFu }            // 60Hz 81 01 04 23 02 FF
// ========== CAM_2D Noise reduction ===========================================
#define VISCA_CAM_2D_NOISE_DIRECT(A,p) { (0x80u+A),0x01u,0x04u,0x53u,p,0xFFu }  // 8x 01 04 53 0p FF
// =========== CAM_3D Noise reduction ==========================================
#define VISCA_CAM_3D_NOISE_DIRECT(A,p) { (0x80u+A),0x01u,0x04u,0x54u,p,0xFFu }  // 8x 01 04 54 0p FF
// ========= Cam_FREEZE ========================================================
#define VISCA_CAM_FREEZE_ON(A) { (0x80u+A),0x01u,0x04u,0x62u,0x02u,0xFFu }      // On 81 01 04 62 02 FF
//   un-freeze immediate
#define VISCA_CAM_FREEZE_OFF(A) { (0x80u+A),0x01u,0x04u,0x62u,0x03u,0xFFu }     // Off 81 01 04 62 03 FF
//  freeze when running preset
#define VISCA_CAM_PRE_FREEZE_ON(A) { (0x80u+A),0x01u,0x04u,0x62u,0x22u,0xFFu }  // On 81 01 04 62 22 FF
//   Freeze Off When Running Preset
#define VISCA_CAM_PRE_FREEZE_OFF(A) { (0x80u+A),0x01u,0x04u,0x62u,0x23u,0xFFu } // Off 81 01 04 62 23 FF
// ============================= Cam AE ========================================
#define VISCA_CAM_AE_FULL_AUTO(A) { (0x80u+A),0x01u,0x04u,0x39u,0x00u,0xFFu }   // Full Auto 8x 01 04 39 00 FF
#define VISCA_CAM_AE_MAN(A) { (0x80u+A),0x01u,0x04u,0x39u,0x03u,0xFFu }         // Manual 8x 01 04 39 03 FF
#define VISCA_CAM_AE_BRIGHT(A) { (0x80u+A),0x01u,0x04u,0x39u,0x0Du,0xFFu }      // Bright 8x 01 04 39 0D FF

// ========= Misc Direct Functions =============================================
#undef VISCA_MSG_LEN
#define VISCA_MSG_LEN 7u                                                        // ==================== SEVEN LONG =========================================
// video system set 8x 01 06 35 00 pp FF
#define VISCA_CAM_VID_SYS_SET_DIRECT(A,B) { (0x80u+A),0x01u,0x06u,0x35u,0x00u,B,0xFFu }    // where B is enum type visa_vid_sys_e
// ============================= Cam Memory ====================================
#define VISCA_CAM_MEM_RESET(A,p) { (0x80u+A),0x01u,0x04u,0x3Fu,0x00u,p,0xFFu }  // Reset 8x 01 04 3F 00 0p FF
#define VISCA_CAM_MEM_SET(A,p) { (0x80u+A),0x01u,0x04u,0x3Fu,0x01u,p,0xFFu }    // Set 8x 01 04 3F 01 0p FF
#define VISCA_CAM_MEM_RECALL(A,p) { (0x80u+A),0x01u,0x04u,0x3Fu,0x02u,p,0xFFu } // Recall 8x 01 04 3F 02 0p FF

#undef VISCA_MSG_LEN
#define VISCA_MSG_LEN 8u                                                        // ==================== EIGHT LONG =========================================
// ========= IR_ReceiveReturn ==================================================
#define VISCA_CAM_IRRR_ON(A) { (0x80u+A),0x01u,0x7Du,0x01u,0x03u,0x00u,0x00u,0xFFu }  // On 8x 01 7D 01 03 00 00 FF
// IR_ReceiveReturn
#define VISCA_CAM_IRRR_OFF(A) { (0x80u+A),0x01u,0x7Du,0x01u,0x13u,0x00u,0x00u,0xFFu } // Off 8x 01 7D 01 13 00 00 FF

// ================ CAM_Zoom Direct ============================================
#undef VISCA_MSG_LEN
#define VISCA_MSG_LEN 9u                                                        // ==================== NINE LONG ==========================================
#define VISCA_CAM_ZOOM_DIRECT(A) { (0x80u+A),0x01u,0x04u,0x47u,0x00u,0x00u,0x00u,0x00u,0xFFu }            // Direct 8x 01 04 47 0p 0q 0r 0s FF pqrs: Zoom Position (0(wide) ~0x4000(tele))
// =============== CAM_Focus Direct ============================================
#define VISCA_CAM_FOCUS_DIRECT(A,p,q,r,s) { (0x80u+A),0x01u,0x04u,0x48u,p,q,r,s,0xFFu } // Direct 8x 01 04 48 0p 0q 0r 0s FF pqrs: Focus Position
// =============== CAM_RGain Direct ============================================
#define VISCA_CAM_RGAIN_DIRECT(A,p,q) { (0x80u+A),0x01u,0x04u,0x43u,0x00u,0x00u,p,q,0xFFu } // Direct 8x 01 04 43 00 00 0p 0q FF
// ============== CAM_BGain Direct =============================================
#define VISCA_CAM_BGAIN_DIRECT(A,p,q) { (0x80u+A),0x01u,0x04u,0x44u,0x00u,0x00u,p,q,0xFFu } // 8x 01 04 44 00 00 0p 0q FF
// ============== CAM_Shutter ==================================================
#define VISCA_CAM_SHUT_DIRECT(A,p,q) { (0x80u+A),0x01u,0x04u,0x4Au,0x00u,0x00u,p,q,0xFFu } // Direct 8x 01 04 4A 00 00 0p 0q FF
// ============== CAM_Iris Direct ==============================================
#define VISCA_CAM_IRIS_DIRECT(A,p,q) { (0x80u+A),0x01u,0x04u,0x4Bu,0x00u,0x00u,p,q,0xFFu } // Direct 8x 01 04 4B 00 00 0p 0q FF
// ============== CAM_Gain =====================================================
#define VISCA_CAM_GAIN_DIRECT(A,p,q) { (0x80u+A),0x01u,0x04u,0x4Cu,0x00u,0x00u,p,q,0xFFu } // Direct 8x 01 04 4C 00 00 0p 0q FF
// ============== CAM_Bright Direct ============================================
#define VISCA_CAM_BRIGHT_DIRECT(A,p,q) { (0x80u+A),0x01u,0x04u,0x4Du,0x00u,0x00u,p,q,0xFFu } // Direct 8x 01 04 4D 00 00 0p 0q FF
// ============== CAM_Aperture Direct ==========================================
#define VISCA_CAM_AP_DIRECT(A,p,q) { (0x80u+A),0x01u,0x04u,0x42u,0x00u,0x00u,p,q,0xFFu } // Direct 8x 01 04 42 00 00 0p 0q FF
// ========= IR_ReceiveReturn Direct ===========================================
#define VISCA_CAM_IRRR_ON(A) { (0x80u+A),0x01u,0x7Du,0x01u,0x03u,0x00u,0x00u,0xFFu }  // On 8x 01 7D 01 03 00 00 FF
// ========= IR_ReceiveReturn Direct ===========================================
#define VISCA_CAM_IRRR_OFF(A) { (0x80u+A),0x01u,0x7Du,0x01u,0x13u,0x00u,0x00u,0xFFu } // Off 8x 01 7D 01 13 00 00 FF
// ============== CAM_ColorGain Direct =========================================
#define VISCA_CAM_COL_GAIN_DIRECT(A,p) { (0x80u+A),0x01u,0x04u,0x49u,0x00u,0x00u,0x00u,p,0xFFu }   // where p is (0~0x0E) 8x 01 04 49 00 00 00 0p FF
// ============== CAM_IDWrite Direct ===========================================
#define VISCA_CAM_ID_WRITE_DIRECT(A,p,q,r,s) { (0x80u+A),0x01u,0x04u,0x22u,p,q,r,s,0xFFu } // 8x 01 04 22 0p 0q 0r 0s FF where 0xfedc p=f q=e r=d s=c
// ========= Pan_tiltDrive =====================================================
// up 8x 01 06 01 VV WW 03 01 FF
#define VISCA_CAM_PAN_TILTDRIVE_UP(A,VV,WW) { (0x80u+A),0x01u,0x06u,0x01u,VV,WW,0x03u,0x01u,0xFFu }  // VV: Pan speed 0x01 (low speed) to 0x18 (high speed)
// down 8x 01 06 01 VV WW 03 02 FF
#define VISCA_CAM_PAN_TILTDRIVE_DOWN(A,VV,WW) { (0x80u+A),0x01u,0x06u,0x01u,VV,WW,0x03u,0x02u,0xFFu }  // WW: Tilt speed 0x01 (low speed) to 0x14 (high speed)
// Left 8x 01 06 01 VV WW 01 03 FF
#define VISCA_CAM_PAN_TILTDRIVE_LEFT(A,VV,WW) { (0x80u+A),0x01u,0x06u,0x01u,VV,WW,0x01u,0x03u,0xFFu }
// Right 8x 01 06 01 VV WW 02 03 FF
#define VISCA_CAM_PAN_TILTDRIVE_RIGHT(A,VV,WW) { (0x80u+A),0x01u,0x06u,0x01u,VV,WW,0x02u,0x03u,0xFFu }
// UpLeft 8x 01 06 01 VV WW 01 01 FF
#define VISCA_CAM_PAN_TILTDRIVE_UP_LEFT(A,VV,WW) { (0x80u+A),0x01u,0x06u,0x01u,VV,WW,0x01u,0x01u,0xFFu }
// Upright 8x 01 06 01 VV WW 02 01 FF
#define VISCA_CAM_PAN_TILTDRIVE_UP_RIGHT(A,VV,WW) { (0x80u+A),0x01u,0x06u,0x01u,VV,WW,0x02u,0x01u,0xFFu }
// DownLeft 8x 01 06 01 VV WW 01 02 FF
#define VISCA_CAM_PAN_TILTDRIVE_DOWN_LEFT(A,VV,WW) { (0x80u+A),0x01u,0x06u,0x01u,VV,WW,0x01u,0x02u,0xFFu }
// DownRight 8x 01 06 01 VV WW 02 02 FF
#define VISCA_CAM_PAN_TILTDRIVE_DOWN_RIGHT(A,VV,WW) { (0x80u+A),0x01u,0x06u,0x01u,VV,WW,0x02u,0x02u,0xFFu }
// Stop 8x 01 06 01 VV WW 03 03 FF
#define VISCA_CAM_PAN_TILTDRIVE_STOP(A,VV,WW) { (0x80u+A),0x01u,0x06u,0x01u,VV,WW,0x03u,0x03u,0xFFu }
// Home 8x 01 06 04 FF
#define VISCA_CAM_PAN_TILTDRIVE_HOME(A) { (0x80u+A),0x01u,0x06u,0x04u,0xFFu }
// Reset 8x 01 06 05 FF
#define VISCA_CAM_PAN_TILTDRIVE_RESET(A) { (0x80u+A),0x01u,0x06u,0x05u,0xFFu }

// ================= CAM_ZoomFocus Direct ======================================
#undef VISCA_MSG_LEN
#define VISCA_MSG_LEN 13u                                                       // ================= THIRTEEN LONG =========================================
#define VISCA_CAM_ZOOM_FOCUS_DIRECT(A,p,q,r,s,t,u,v,w) { (0x80u+A),0x01u,0x04u,0x47u,p,q,r,s,t,u,v,w,0xFFu }   // Direct 8x 01 04 47 0p 0q 0r 0s 0t 0u 0v 0w FF pqrs: Focus Position

// ================= CAM_ZoomFocus Direct ======================================
#undef VISCA_MSG_LEN
#define VISCA_MSG_LEN 15u                                                       // ================= FIFTEEN LONG =========================================
// AbsolutePosition 8x 01 06 02 VV WW 0Y 0Y 0Y 0Y 0Z 0Z 0Z 0Z FF
#define VISCA_CAM_PAN_TILTDRIVE_ABS_POS(A,VV,WW) { (0x80u+A),0x01u,0x06u,0x02u,VV,WW,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0xFFu }
// RelativePosition 8x 01 06 03 VV WW 0Y 0Y 0Y 0Y 0Z 0Z 0Z 0Z FF
#define VISCA_CAM_PAN_TILTDRIVE_REL_POS(A,VV,WW) { (0x80u+A),0x01u,0x06u,0x03u,VV,WW,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0xFFu }
// ======================== Pan-tiltLimitSet ===================================
// Set 8x 01 06 07 00 0W 0Y 0Y 0Y 0Y 0Z 0Z 0Z 0Z FF
#define VISCA_CAM_PAN_TILTDRIVE_LIM_SET(A,WW) { (0x80u+A),0x01u,0x06u,0x07u,0x00u,WW,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0xFFu }
// CLear 8x 01 06 07 01 0W 07 0F 0F 0F 07 0F 0F 0F FF
#define VISCA_CAM_PAN_TILTDRIVE_LIM_CLR(A,WW) { (0x80u+A),0x01u,0x06u,0x07u,0x00u,WW,0x07u,0x0Fu,0x0Fu,0x0Fu,0x07u,0x0Fu,0x0Fu,0x0Fu,0xFFu }

typedef enum {
  SONY_INIT = 0u,
  SONY_REQ_SENT,
  SONY_NO_REPLY,
  SONY_REP_OK,
  SONY_REP_FAIL
} sony_msg_state_e;

typedef enum {
  SONY_NO_OPT = 0u,
  SONY_RGAIN,
  SONY_BGAIN,
  SONY_SHUT,
  SONY_IRIS,
  SONY_GAIN,
  SONY_BRIGHT,
  SONY_AP,
  SONY_EXPCOMP,
  SONY_BK_LT,
  SONY_LR_REV,
  SONY_PFLIP,
  SONY_IRT,
  SONY_IRRR,
  SONY_IRR,
  SONY_FLICK,
  SONY_MM,
  SONY_PWR,
  SONY_ZOOM,
  SONY_FOCUS,
  SONY_CAMWB,
  SONY_CAMAE,
  SONY_CAMMEM,
  SONY_VIDSYS,
  SONY_FREEZE,
  SONY_PANTILTDRIVE,
  SONY_MISC
} sony_option_state_e;                                                          // each gui group for the camera, the index buttons run each function from jump table

typedef enum {
  SV_1080P60 = 1u,
  SV_1080P50,
  SV_1080I60,
  SV_1080I50,
  SV_1080P30,
  SV_1080P25,
  SV_720P60,
  SV_720P50,
  SV_720P30,
  SV_720P25,
  SV_1600_900_60HZ,                                                             // usb outputs start here
  SV_1440_900_60HZ,
  SV_1366_768_60HZ,
  SV_1280_800_60HZ,
  SV_1024_768_60HZ,
  SV_800_600_60HZ,
  SV_800_600_30HZ,
  SV_640_480_60HZ,
  SV_640_480_30HZ,
  NUM_OF_VIDEO_SYS                                                              // number of video systems
} visa_vid_sys_e;

typedef enum {
        ss1_10000 = 21u,
        ss1_6000 = 20u,
        ss1_4000 = 19u,
        ss1_3000 = 18u,
        ss1_2000 = 17u,
        ss1_1500 = 16u,
        ss1_1000 = 15u,
        ss1_725 = 14u,
        ss1_500 = 13u,
        ss1_350 = 12u,
        ss1_250 = 11u,
        ss1_180 = 10u,
        ss1_125 = 9u,
        ss1_100 = 8u,
        ss1_90 = 7u,
        ss1_60 = 6u,
        ss1_30 = 5u,
        ss1_15 = 4u,
        ss1_8 = 3u,
        ss1_4 = 2u,
        ss1_2 = 1u,
        ss1_1 = 0u
} visa_shutter_spd_e;                                                           // shutter speed settings

typedef enum {
        iris_close = 0u,
        iris_f32 = 1u,
        iris_f28 = 2u,
        iris_f24 = 3u,
        iris_f22 = 4u,
        iris_f18 = 5u,
        iris_f14 = 6u,
        iris_f11 = 7u,
        iris_f9_6 = 8u,
        iris_f6_8 = 9u,
        iris_f5_6 = 10u,
        iris_f4_8 = 11u,
        iris_f4_0 = 12u,
        iris_f3_4 = 13u,
        iris_f2_8 = 14u,
        iris_f2_4 = 15u,
        iris_f2_0 = 16u,
        iris_f1_8 = 17u
} visa_iris_e;                                                                  // iris settings

typedef enum {
        gain_0dB = 0u,
        gain_2dB = 1u,
        gain_4dB = 2u,
        gain_6dB = 3u,
        gain_8dB = 4u,
        gain_10dB = 5u,
        gain_12dB = 6u,
        gain_14dB = 7u,
        gain_16dB = 8u,
        gain_18dB = 9u,
        gain_20dB = 10u,
        gain_22dB = 11u,
        gain_24dB = 12u,
        gain_26dB = 13u,
        gain_28dB = 14u
} visa_gain_e;                                                                  // gain settings

typedef enum {
        zoom_x1_wide = 0x0u,
        zoom_x1_2 = 0x8D0u,
        zoom_x1_5 = 0x1194u,
        zoom_x2 = 0x1A58u,
        zoom_x3 = 0x2610u,
        zoom_x5 = 0x31D4u,
        zoom_x5_8 = 0x34BCu,
        zoom_x6_9 = 0x37A4u,
        zoom_x8_2 = 0x3A98u,
        zoom_x9_9 = 0x3D8Cu,
        zoom_x11_8_tele = 0x4000u
} visa_zoom_e;                                                                  // zoom settings

typedef enum {
        pan_minus_170 = 0xF670u,
        pan_minus_135 = 0xF868u,
        pan_minus_90 = 0xFAF0u,
        pan_minus_45 = 0xFD78u,
        pan_0 = 0x0000u,
        pan_plus_45 = 0x0288u,
        pan_plus_90 = 0x0510u,
        pan_plus_135 = 0x0798u,
        pan_plus_170 = 0x0990u,
        tilt_minus_30 = 0xFE50u,
        tilt_0 = 0x0000u,
        tilt_plus_30 = 0x01B0u,
        tilt_plus_60 = 0x0360u,
        tilt_plus_90 = 0x0510u
} visa_pan_tilt_e;                                                              // pan tilt settings

SOCAMPACKED(
typedef struct {
        uint8_t stx;                                                            // start transmission
        uint8_t mSndByte1;                                                      // message send byte 1
        uint8_t mSndByte2;                                                      // message send byte 2
        uint8_t mSndByte3;                                                      // message send byte 3
        uint8_t mSndByte4;                                                      // message send byte 4
        uint8_t mSndByte5;                                                      // message send byte 5
        uint8_t mSndByte6;                                                      // message send byte 6
        uint8_t mSndByte7;                                                      // message send byte 7
        uint8_t mSndByte8;                                                      // message send byte 8
}) SONY_send_t;                                                                 // send message contiainer structure

SOCAMPACKED(
typedef struct {
        uint8_t stx;                                                            // start transmission
        uint8_t mSndByte1;                                                      // message send byte 1
        uint8_t mSndByte2;                                                      // message send byte 2
        uint8_t mSndByte3;                                                      // message send byte 3
        uint8_t mSndByte4;                                                      // message send byte 4
        uint8_t mSndByte5;                                                      // message send byte 5
        uint8_t mSndByte6;                                                      // message send byte 6
        uint8_t mSndByte7;                                                      // message send byte 7
        uint8_t mSndByte8;                                                      // message send byte 8
        uint8_t mSndByte9;                                                      // message send byte 9
        uint8_t mSndByte10;                                                     // message send byte 10
        uint8_t mSndByte11;                                                     // message send byte 11
        uint8_t mSndByte12;                                                     // message send byte 12
        uint8_t mSndByte13;                                                     // message send byte 13
        uint8_t mSndByte14;                                                     // message send byte 14
}) SONY_send_long_t;                                                            // send message contiainer structure for extended length datagrams

SOCAMPACKED(
typedef struct {
        uint8_t stx;                                                            // start transmission
        uint8_t mRcvByte1;                                                      // message receive byte 1
        uint8_t mRcvByte2;                                                      // message receive byte 2
        uint8_t mRcvByte3;                                                      // message receive byte 3
        uint8_t mRcvByte4;                                                      // message receive byte 4
        uint8_t mRcvByte5;                                                      // message receive byte 5
        uint8_t mRcvByte6;                                                      // message receive byte 6
        uint8_t mRcvByte7;                                                      // message receive byte 7
        uint8_t mRcvByte8;                                                      // message receive byte 8
}) SONY_rcv_t;                                                                  // receive message contiainer structure

SOCAMPACKED(
typedef struct {
        uint8_t jumpTblIdx;                                                     // jump table index
        sony_option_state_e opt_sel:8u;                                         // group option selection
        sony_msg_state_e msgStat:8u;                                            // message request state
        uint16_t tmr;                                                           // tick count on request
}) SONY_hmi_t;                                                                  // receive message contianer structure

typedef enum {
        NO_ACTION = 0u,                                                         // null function
        SET_ON,                                                                 // set option to on
        SET_OFF,                                                                // set option to off
        DO_RESET,                                                               // perform reset on option
        GO_UP,                                                                  // increase option value
        GO_DOWN,                                                                // decrease option value
        SET_VALUE,                                                              // set value to that specified for direct option
        MAX_NUM_OF_JUMP_STATES                                                  // number of states used
} visa_jump_states_e;                                                           // standard jump table state engine

#define SONY_MAX_REQ_CMD MAX_NUM_OF_JUMP_STATES                                 // number of jump table entries

// function definitions
void sendSonyMessage( SONY_send_t *sonyMsgOut );
uint8_t sonySndRgainReset( );
uint8_t sonySndBgainReset( );
uint8_t sonySndShutReset( );
uint8_t sonySndIrisReset( );
uint8_t sonySndGainReset( );
uint8_t sonySndBrightReset( );
uint8_t sonySndApReset( );
uint8_t sonySndExpReset( );
//uint8_t sonySndPwrInq();
void sonyRcv1( unsigned char *replyBuf, SONY_hmi_t *hmi );
//uint8_t sonyRcvCamPowerOn( );
//uint8_t sonyRcvCamPowerState( );
//uint8_t sonyRcvCamPowerOff( );
uint8_t sonySndNoState();                                                       // dummy function do nothing
uint8_t sonyRcv12( );
uint8_t sonyRcv13( );
uint8_t sonyRcv14( );
uint8_t sonyRcv15( );
uint8_t sonyRcv16( );
uint8_t sonySndRgainUp( );
uint8_t sonySndBgainUp( );
uint8_t sonySndShutUp( );
uint8_t sonySndIrisUp( );
uint8_t sonySndGainUp( );
uint8_t sonySndBrightUp( );
uint8_t sonySndApUp( );
uint8_t sonySndExpUp( );
void sonyRcv2( unsigned char *replyBuf, SONY_hmi_t *hmi );
uint8_t sonySndRgainDown( );
uint8_t sonySndBgainDown( );
uint8_t sonySndShutDown( );
uint8_t sonySndIrisDown( );
uint8_t sonySndGainDown( );
uint8_t sonySndBrightDown( );
uint8_t sonySndApDown( );
uint8_t sonySndExpDown( );
void sonyRcv3( unsigned char *replyBuf, SONY_hmi_t *hmi );
uint8_t sonySndRgainDirect( );
uint8_t sonySndBgainDirect( );
uint8_t sonySndShutDirect( );
uint8_t sonySndIrisDirect( );
uint8_t sonySndGainDirect( );
uint8_t sonySndBrightDirect( );
uint8_t sonySndApDirect( );
uint8_t sonySndExpDirect( );
uint8_t sonySndExpOn( );
uint8_t sonySndExpOff( );
uint8_t sonySndBkLtOn( );
uint8_t sonySndBkLtOff( );
uint8_t sonySndLrRevOn( );
uint8_t sonySndLrRevOff( );
uint8_t sonySndPFlipOn( );
uint8_t sonySndPFlipOff( );
uint8_t sonySndIrtOn( );
uint8_t sonySndIrtOff( );
uint8_t sonySndIrrrOn( );
uint8_t sonySndIrrrOff( );
uint8_t sonySndIrrOn( );
uint8_t sonySndIrrOff( );
uint8_t sonySndPwrOn( );
uint8_t sonySndPwrOff( );
uint8_t sonySndFlick50Hz( );
uint8_t sonySndFlick60Hz( );
uint8_t sonySndIrrrToggle();
uint8_t sonySndMMUp( );
uint8_t sonySndMMDown( );
void sonyRcv4( unsigned char *replyBuf, SONY_hmi_t *hmi );
uint8_t sonySnd5( );
void sonyRcv5( unsigned char *replyBuf, SONY_hmi_t *hmi );
uint8_t sonySnd6( );
uint8_t sonySndZoomStop( );
uint8_t sonySndZoomTeleStd( );
uint8_t sonySndZoomTeleVar( );
uint8_t sonySndZoomWideStd( );
uint8_t sonySndZoomWideVar( );
uint8_t sonySndZoomDirect( );
uint8_t sonySndFocusStop();
uint8_t sonySndFocusFarStd();
uint8_t sonySndFocusNearStd();
uint8_t sonySndFocusOnePushAF();
uint8_t sonySndZoomFocusDirect();
uint8_t sonySndFocusDirect();
uint8_t SonySndCamWBAuto();
uint8_t SonySndCamWBIndoor();
uint8_t SonySndCamWBOutdoor();
uint8_t SonySndCamWBOnePush();
uint8_t SonySndCamWBOneMan();
uint8_t SonySndCamWBOutdoorAuto();
uint8_t SonySndCamWBSodiumLampAuto();
uint8_t SonySndCamWBSodiumAuto();
uint8_t SonySndCamAEFullAuto();
uint8_t SonySndCamAEMan();
uint8_t SonySndCamAEBright();
uint8_t SonySndCamMemReset();
uint8_t SonySndCamMemSet();
uint8_t SonySndCamMemRecall();
uint8_t SonySndVidSysColorGain();
uint8_t SonySndVidSys2DNoiseRed();
uint8_t SonySndVidSys3DNoiseRed();
uint8_t SonySndCamVidSysSet();
uint8_t SonySndCamVidSysIDWrite();
uint8_t sonySndFreezeOn();
uint8_t sonySndFreezeOff();
uint8_t sonySndPreFreezeOn();
uint8_t sonySndPreFreezeOff();
uint8_t SonySndCamPanTiltDriveUp();
uint8_t SonySndCamPanTiltDriveDown();
uint8_t SonySndCamPanTiltDriveLeft();
uint8_t SonySndCamPanTiltDriveRight();
uint8_t SonySndCamPanTiltDriveDownRight();
uint8_t SonySndCamPanTiltDriveDownLeft();
uint8_t SonySndCamPanTiltDriveUpLeft();
uint8_t SonySndCamPanTiltDriveUpRight();
uint8_t SonySndCamPanTiltDriveHome();
uint8_t SonySndCamPanTiltDriveStop();
uint8_t SonySndCamPanTiltDriveReset();
uint8_t SonySndCamPanTiltDriveAbsPos();
uint8_t SonySndCamPanTiltDriveRelPos();
uint8_t SonySndCamPanTiltDriveLimSet();
uint8_t SonySndCamPanTiltDriveLimClr();
uint8_t SonySndCamCmdCancel();
uint8_t SonySndCamIFClr();
uint8_t SonySndAddrSet();
void sonyRcv6( unsigned char *replyBuf, SONY_hmi_t *hmi );
void chooseSendFunction( uint8_t *hmiVal );
void hmiSendFunction( SONY_hmi_t *hmiVal );
void hmiRcvFunction( unsigned char *rBuf, SONY_hmi_t *hmiVal );
// inquiry functions
uint8_t sonySndPwrInq();
uint8_t sonyRcvCamPowerState();
uint8_t sonySndFocusInq();
uint8_t sonyRcvCamFocusState();
uint8_t sonySndExposureInq();
uint8_t sonyRcvCamExposureState();
uint8_t sonySndMenuInq();
uint8_t sonyRcvCamMenuState();
uint8_t sonySndLRRevInq();
uint8_t sonyRcvCamLRRevState();
uint8_t sonySndPicFlipInq();
uint8_t sonyRcvCamPicFlipState();
uint8_t sonySndIRTransInq();
uint8_t sonyRcvCamIRTransState();
uint8_t sonySndIRRecvInq();
uint8_t sonyRcvCamIRRecvState();
uint8_t sonySndWBModelInq();
uint8_t sonyRcvWBModelState();
uint8_t sonySndIRRecvInq();
uint8_t sonyRcvIRRecvState();
uint8_t sonySndAEModeInq();
uint8_t sonyRcvAEModeState();

// jump table definitions
#ifdef jump_table
// table is STD GUI OPTION as below then functions normally (0=no action 1=on 2=off 3=reset 4=up 5=down 6=direct) 7 states in normal engine
static uint8_t (* const go[7u] )(void) = { sonySndNoState, sonySndPwrInq, sonySndRgainUp, sonySndRgainDown, sonySndRgainDirect, sonySnd5, sonySnd6 };          // function jump table for send
static uint8_t (* const rgain[7u] )(void) = { sonySndNoState, sonySndNoState, sonySndNoState, sonySndRgainReset, sonySndRgainUp, sonySndRgainDown, sonySndRgainDirect };          // function jump table for send
static uint8_t (* const bgain[7u] )(void) = { sonySndNoState, sonySndNoState, sonySndNoState, sonySndBgainReset, sonySndBgainUp, sonySndBgainDown, sonySndBgainDirect };
static uint8_t (* const shut[7u] )(void) = { sonySndNoState, sonySndNoState, sonySndNoState, sonySndShutReset, sonySndShutUp, sonySndShutDown, sonySndShutDirect };
static uint8_t (* const iris[7u] )(void) = { sonySndNoState, sonySndNoState, sonySndNoState, sonySndIrisReset, sonySndIrisUp, sonySndIrisDown, sonySndIrisDirect };
static uint8_t (* const gain[7u] )(void) = { sonySndNoState, sonySndNoState, sonySndNoState, sonySndGainReset, sonySndGainUp, sonySndGainDown, sonySndGainDirect };
static uint8_t (* const ap[7u] )(void) = { sonySndNoState, sonySndNoState, sonySndNoState, sonySndApReset, sonySndApUp, sonySndApDown, sonySndApDirect };
static uint8_t (* const zoom[7u] )(void) = { sonySndNoState, sonySndZoomStop, sonySndZoomTeleStd, sonySndZoomTeleVar, sonySndZoomWideStd, sonySndZoomWideVar, sonySndZoomDirect };
static uint8_t (* const focus[7u] )(void) = { sonySndNoState, sonySndFocusStop, sonySndFocusFarStd, sonySndFocusNearStd, sonySndFocusOnePushAF, sonySndZoomFocusDirect, sonySndFocusDirect };
static uint8_t (* const bklt[3u] )(void) = { sonySndNoState, sonySndBkLtOn, sonySndBkLtOff  };
static uint8_t (* const lrrev[3u] )(void) = { sonySndNoState, sonySndLrRevOn, sonySndLrRevOff  };
static uint8_t (* const pflip[3u] )(void) = { sonySndNoState, sonySndPFlipOn, sonySndPFlipOff  };
static uint8_t (* const irt[3u] )(void) = { sonySndNoState, sonySndIrtOn, sonySndIrtOff  };
static uint8_t (* const misc[4u] )(void) = { sonySndNoState, SonySndCamCmdCancel, SonySndCamIFClr, SonySndAddrSet  };
static uint8_t (* const irrr[4u] )(void) = { sonySndNoState, sonySndIrrrOn, sonySndIrrrOff, sonySndIrrrToggle  };
static uint8_t (* const camwb[9u] )(void) = { sonySndNoState, SonySndCamWBAuto, SonySndCamWBIndoor, SonySndCamWBOutdoor, SonySndCamWBOnePush, SonySndCamWBOneMan, SonySndCamWBOutdoorAuto, SonySndCamWBSodiumLampAuto, SonySndCamWBSodiumAuto   };
static uint8_t (* const irr[3u] )(void) = { sonySndNoState, sonySndIrrOn, sonySndIrrOff  };
static uint8_t (* const pantiltdrive[16u] )(void) = { sonySndNoState, SonySndCamPanTiltDriveUp, SonySndCamPanTiltDriveDown, SonySndCamPanTiltDriveLeft, SonySndCamPanTiltDriveRight, SonySndCamPanTiltDriveDownRight, SonySndCamPanTiltDriveDownLeft, SonySndCamPanTiltDriveUpLeft, SonySndCamPanTiltDriveUpRight, SonySndCamPanTiltDriveHome, SonySndCamPanTiltDriveStop, SonySndCamPanTiltDriveReset, SonySndCamPanTiltDriveAbsPos, SonySndCamPanTiltDriveRelPos, SonySndCamPanTiltDriveLimSet, SonySndCamPanTiltDriveLimClr };
static uint8_t (* const pwr[3u] )(void) = { sonySndNoState, sonySndPwrOn, sonySndPwrOff  };
static uint8_t (* const freeze[5u] )(void) = { sonySndNoState, sonySndFreezeOn, sonySndFreezeOff, sonySndPreFreezeOn, sonySndPreFreezeOff  };
static uint8_t (* const camae[4u] )(void) = { sonySndNoState, SonySndCamAEFullAuto, SonySndCamAEMan, SonySndCamAEBright  };
static uint8_t (* const mem[4u] )(void) = { sonySndNoState, SonySndCamMemReset, SonySndCamMemSet, SonySndCamMemRecall  };
static uint8_t (* const vidsys[6u] )(void) = { sonySndNoState, SonySndVidSysColorGain, SonySndVidSys2DNoiseRed, SonySndVidSys3DNoiseRed, SonySndCamVidSysSet, SonySndCamVidSysIDWrite };
static uint8_t (* const flick[3u] )(void) = { sonySndNoState, sonySndFlick50Hz, sonySndFlick60Hz  };
static uint8_t (* const mm[6u] )(void) = { sonySndNoState, sonySndNoState, sonySndNoState, sonySndNoState, sonySndMMUp, sonySndMMDown  };
static uint8_t (* const expcomp[7u] )(void) = { sonySndNoState, sonySndExpReset, sonySndExpUp, sonySndExpDown, sonySndExpDirect, sonySndExpOn, sonySndExpOff };
static uint8_t (* const bright[7u] )(void) = { sonySndNoState, sonySndNoState, sonySndNoState, sonySndBrightReset, sonySndBrightUp, sonySndBrightDown, sonySndBrightDirect };
static uint8_t (* const recv[7u] )(void) = { sonySndNoState, sonyRcvCamPowerState, sonyRcv12, sonyRcv13, sonyRcv14, sonyRcv15, sonyRcv16 };                          // function jump table for receive
#endif

#define HMI_REQ_NONE 0U                                                         // null feature select
#define HMI_REQ_ON 1U                                                           // request to on
#define HMI_REQ_OFF 2U                                                          // request to off
#define HMI_REQ_RESET 3U                                                        // reset
#define HMI_REQ_UP 4U                                                           // up
#define HMI_REQ_DOWN 5U                                                         // down
#define HMI_REQ_DIRECT 6U                                                       // down

// define you're list of commands here for the sequencer array
//
#define SONY_MSG_SND_RESET { 0xBBU, 0x01U, 0x34U, 0x56U, 0xDFU, 0xF4U }
#undef SONY_MSG_SND_RESET

// ================== Inquiry messages and responses ===========================
//
// ------------------ Power Status ---------------------------------------------
#define SONY_SND_CAM_PWR_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x00u, 0xFFu }         // 8x 09 04 00 FF
#define SONY_RCV_CAM_PWR_ON(A) { 8u+A, 0x50u, 0x02u, 0xFFu, 0x00u }             // y0 50 02 FF on
#define SONY_RCV_CAM_PWR_OFF(A) { 8u+A, 0x50u, 0x03u, 0xFFu, 0x00u }            // y0 50 03 FF off
// ----------------- CAM_ZoomPosInq --------------------------------------------
#define SONY_SND_CAM_ZOOM_POS_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x47u, 0xFFu }    // 8x 09 04 47 FF
#define SONY_RCV_CAM_ZOOM_POS(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 0p 0q 0r 0s FF (pqrs = zoom position)
// ----------------- CAM_FocusModeInq ------------------------------------------
#define SONY_SND_CAM_FOCUS_MODE_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x38u, 0xFFu }  // 8x 09 04 38 FF
#define SONY_RCV_CAM_FOCUS_MODE_AUT(A) { 8u+A, 0x50u, 0x02u, 0xFFu, 0x00u }     // y0 50 02 FF        Auto Focus
#define SONY_RCV_CAM_FOCUS_MODE_MAN(A) { 8u+A, 0x50u, 0x03u, 0xFFu, 0x00u }     // y0 50 03 FF        Manual Focus
// ----------------- CAM_FocusPosInq ------------------------------------------
#define SONY_SND_CAM_FOCUS_POS_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x48u, 0xFFu }    // 8x 09 04 48 FF
#define SONY_RCV_CAM_FOCUS_POS(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 0p 0q 0r 0s FF (pqrs = focus position)
// ----------------- CAM_WBModeInq ------------------------------------------
#define SONY_SND_CAM_WB_MODEL_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x35u, 0xFFu }    // 8x 09 04 35 FF
#define SONY_RCV_CAM_WB_MODEL_AUTO(A) { 8u+A, 0x50u, 0x00u, 0xFFu, 0x00u }      // y0 50 00 FF        Auto
#define SONY_RCV_CAM_WB_MODEL_INDOOR(A) { 8u+A, 0x50u, 0x01u, 0xFFu, 0x00u }    // y0 50 01 FF        Indoor mode
#define SONY_RCV_CAM_WB_MODEL_OUTDOOR(A) { 8u+A, 0x50u, 0x02u, 0xFFu, 0x00u }   // y0 50 02 FF        Outdoor mode
#define SONY_RCV_CAM_WB_MODEL_1PSH(A) { 8u+A, 0x50u, 0x03u, 0xFFu, 0x00u }      // y0 50 03 FF        OnePush mode
#define SONY_RCV_CAM_WB_MODEL_ATW(A) { 8u+A, 0x50u, 0x04u, 0xFFu, 0x00u }       // y0 50 04 FF        ATW
#define SONY_RCV_CAM_WB_MODEL_MAN(A) { 8u+A, 0x50u, 0x05u, 0xFFu, 0x00u }       // y0 50 05 FF        Manual
// ----------------- CAM_RGainInq ---------------------------------------------
#define SONY_SND_CAM_RGAIN_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x43u, 0xFFu }       // 8x 09 04 43 FF
#define SONY_RCV_CAM_RGAIN_POS(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 00 00 0p 0q FF (pq = rgain)
// ----------------- CAM_BGainInq ----------------------------------------------
#define SONY_SND_CAM_BGAIN_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x44u, 0xFFu }       // 8x 09 04 44 FF
#define SONY_RCV_CAM_BGAIN_POS(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 00 00 0p 0q FF (pq = bgain)
// ----------------- CAM_AEModeInq ---------------------------------------------
#define SONY_SND_CAM_AEMODE_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x39u, 0xFFu }      // 8x 09 04 39 FF
#define SONY_RCV_CAM_AEMODE_AUTO(A) { 8u+A, 0x50u, 0x00u, 0xFFu, 0x00u }        // y0 50 00 FF        Auto
#define SONY_RCV_CAM_AEMODE_SHUT(A) { 8u+A, 0x50u, 0x0Au, 0xFFu, 0x00u }        // y0 50 0A FF        Shutter mode
#define SONY_RCV_CAM_AEMODE_IRIS(A) { 8u+A, 0x50u, 0x0Bu, 0xFFu, 0x00u }        // y0 50 0B FF  Iris Priority
#define SONY_RCV_CAM_AEMODE_BRIGHT(A) { 8u+A, 0x50u, 0x0Du, 0xFFu, 0x00u }      // y0 50 0D FF  Bright
#define SONY_RCV_CAM_AEMODE_MAN(A) { 8u+A, 0x50u, 0x03u, 0xFFu, 0x00u }         // y0 50 03 FF        Manual
// ----------------- CAM_Shutter Pos Inq ---------------------------------------
#define SONY_SND_CAM_SHUT_POS_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x4Au, 0xFFu }    // 8x 09 04 4A FF
#define SONY_RCV_CAM_SHUT_POS(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 00 00 0p 0q FF
// ----------------- CAM_Iris Pos Inq ---------------------------------------
#define SONY_SND_CAM_IRIS_POS_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x4Bu, 0xFFu }    // 8x 09 04 4B FF
#define SONY_RCV_CAM_IRIS_POS(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 00 00 0p 0q FF
// ----------------- CAM_GainPosiInq ---------------------------------------
#define SONY_SND_CAM_GAIN_POS_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x4Bu, 0xFFu }    // 8x 09 04 4B FF
#define SONY_RCV_CAM_GAIN_POS(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 00 00 0p 0q FF
// ----------------- CAM_ BrightPosiInq ---------------------------------------
#define SONY_SND_CAM_BRIGHT_POS_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x4Du, 0xFFu }  // 8x 09 04 4D FF
#define SONY_RCV_CAM_BRIGHT_POS(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 00 00 0p 0q FF
// ----------------- CAM_ExpCompPosInq ---------------------------------------
#define SONY_SND_CAM_EXPCOMP_POS_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x4Eu, 0xFFu } // 8x 09 04 4E FF
#define SONY_RCV_CAM_EXPCOMP_POS(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 00 00 0p 0q FF
// ----------------- CAM_ApertureInq ---------------------------------------
#define SONY_SND_CAM_AP_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x42u, 0xFFu }          // 8x 09 04 42 FF
#define SONY_RCV_CAM_AP(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 00 00 0p 0q FF
// ----------------- CAM_MemoryInq ---------------------------------------
#define SONY_SND_CAM_MEM_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x3Fu, 0xFFu }          // 8x 09 04 3F FF
#define SONY_RCV_CAM_MEM(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 00 00 0p 0q FF
// ----------------- Pan-tiltMaxSpeedInq ---------------------------------------
#define SONY_SND_CAM_PAN_TILT_SPD_INQ(A) { 0x80u+A, 0x09u, 0x06u, 0x11u, 0xFFu } // 8x 09 06 11 FF
#define SONY_RCV_CAM_PAN_TILT_SPD(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 ww zz FF w-pan z-tilt
// ----------------- Pan-tiltPosInq ---------------------------------------
#define SONY_SND_CAM_PAN_TILT_POS_INQ(A) { 0x80u+A, 0x09u, 0x06u, 0x12u, 0xFFu } // 8x 09 06 12 FF
#define SONY_RCV_CAM_PAN_TILT_POS(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }   // y0 50 ww zz FF w-pan z-tilt
// ----------------- CAM_ExpCompModeInq ---------------------------------------
#define SONY_SND_CAM_EXP_COMP_MODE_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x3Eu, 0xFFu } // 8x 09 04 3E FF
#define SONY_RCV_CAM_EXP_COMP_MODE_ON(A) { 8u+A, 0x50u, 0x02u, 0xFFu }          // y0 50 02 FF
#define SONY_RCV_CAM_EXP_COMP_MODE_OFF(A) { 8u+A, 0x50u, 0x03u, 0xFFu }         // y0 50 03 FF
// ----------------- SYS_MenuModeInq ---------------------------------------
#define SONY_SND_CAM_SYS_MENU_MODE_INQ(A) { 0x80u+A, 0x09u, 0x06u, 0x06u, 0xFFu } // 8x 09 06 06 FF
#define SONY_RCV_CAM_SYS_MENU_MODE_ON(A) { 8u+A, 0x50u, 0x02u, 0xFFu }          // y0 50 02 FF
#define SONY_RCV_CAM_SYS_MENU_MODE_OFF(A) { 8u+A, 0x50u, 0x03u, 0xFFu }         // y0 50 03 FF
// ----------------- CAM_LR_ReverseInq ---------------------------------------
#define SONY_SND_CAM_LR_REV_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x61u, 0xFFu }      // 8x 09 04 61 FF
#define SONY_RCV_CAM_LR_REV_ON(A) { 8u+A, 0x50u, 0x02u, 0xFFu }                 // y0 50 02 FF
#define SONY_RCV_CAM_LR_REV_OFF(A) { 8u+A, 0x50u, 0x03u, 0xFFu }                // y0 50 03 FF
// ----------------- CAM_PictureFlipInq ---------------------------------------
#define SONY_SND_CAM_PIC_FLIP_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x66u, 0xFFu }    // 8x 09 04 66 FF
#define SONY_RCV_CAM_PIC_FLIP_ON(A) { 8u+A, 0x50u, 0x02u, 0xFFu }               // y0 50 02 FF
#define SONY_RCV_CAM_PIC_FLIP_OFF(A) { 8u+A, 0x50u, 0x03u, 0xFFu }              // y0 50 03 FF
// ----------------- CAM_IR_TransferInq ---------------------------------------
#define SONY_SND_CAM_IR_TRANS_INQ(A) { 0x80u+A, 0x09u, 0x06u, 0x1Au, 0xFFu }    // 8x 09 06 1A FF
#define SONY_RCV_CAM_IR_TRANS_ON(A) { 8u+A, 0x50u, 0x02u, 0xFFu }               // y0 50 02 FF
#define SONY_RCV_CAM_IR_TRANS_OFF(A) { 8u+A, 0x50u, 0x03u, 0xFFu }              // y0 50 03 FF
// ----------------- CAM_IR_ReceiveInq ---------------------------------------
#define SONY_SND_CAM_IR_RECV_INQ(A) { 0x80u+A, 0x09u, 0x06u, 0x08u, 0xFFu }     // 8x 09 06 08 FF
#define SONY_RCV_CAM_IR_RECV_ON(A) { 8u+A, 0x50u, 0x02u, 0xFFu }                // y0 50 02 FF
#define SONY_RCV_CAM_IR_RECV_OFF(A) { 8u+A, 0x50u, 0x03u, 0xFFu }               // y0 50 03 FF
// ----------------- CAM_IR_ReceiveReturnInq -----------------------------------
#define SONY_IR_RECV_RET_INQ(A) { 0x80u+A, 0x09u, 0x06u, 0x08u, 0xFFu }         // 8x 09 06 08 FF
#define SONY_IR_RECV_RET_ON_OFF(A) { 8u+A, 0x07u, 0x07Du, 0x01u, 0x04u, 0x00u, 0xFFu }  // y0 07 7D 01 04 00 FF
#define SONY_IR_RECV_RET_ZOOM(A) { 8u+A, 0x07u, 0x07Du, 0x01u, 0x04u, 0x07u, 0xFFu }    // y0 07 7D 01 04 07 FF
#define SONY_IR_RECV_RET_AF_ON_OFF(A) { 8u+A, 0x07u, 0x07Du, 0x01u, 0x04u, 0x38u, 0xFFu }  // y0 07 7D 01 04 38 FF
#define SONY_IR_RECV_RET_BACKLT(A) { 8u+A, 0x07u, 0x07Du, 0x01u, 0x04u, 0x33u, 0xFFu }    // y0 07 7D 01 04 33 FF
#define SONY_IR_RECV_RET_MEMORY(A) { 8u+A, 0x07u, 0x07Du, 0x01u, 0x04u, 0x3Fu, 0xFFu }    // y0 07 7D 01 04 3F FF
#define SONY_IR_RECV_RET_PAN_TILTDRV(A) { 8u+A, 0x07u, 0x07Du, 0x01u, 0x06u, 0x01u, 0xFFu } // y0 07 7D 01 06 01 FF
// ----------------- VideoSystemInq --------------------------------------------
#define SONY_SND_CAM_VID_SYS_INQ(A) { 0x80u+A, 0x09u, 0x06u, 0x23u, 0xFFu }     // 8x 09 06 23 FF
#define SONY_RCV_CAM_VID_SYS(A) { 8u+A, 0x50u, 0x00u, 0xFFu }                   // y0 50 pp FF pp: 0~18 Video format
// ----------------- CAM_IDInq -------------------------------------------------
#define SONY_SND_CAM_VID_ID_INQ(A) { 0x80u+A, 0x09u, 0x04u, 0x22u, 0xFFu }      // 8x 09 04 22 FF
#define SONY_RCV_CAM_VID_ID(A) { 8u+A, 0x50u, 0x00u, 0xFFu, 0x00 }              // y0 50 pp FF pp:
// ----------------- CAM_VersionInq -------------------------------------------------
#define SONY_SND_CAM_VER_INQ(A) { 0x80u+A, 0x09u, 0x00u, 0x02u, 0xFFu }         // 8x 09 00 02 FF
#define SONY_RCV_CAM_VID_VER(A) { 8u+A, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu }  // y0 50 ab cd mn pq rs tu vw FF



//#define SONY_MSG_RCV_1 { 0xBBU, 0x01U, 0xF7U, 0x76U, 0x67U, 0xE9U }           // y0 50 02 FF
//#define SONY_MSG_SND_2 { 0xBBU, 0x01U, 0x67U, 0x76U, 0x30U, 0x74U }
#define SONY_MSG_RCV_2 { 0xBBU, 0x33U, 0x99U, 0x11U, 0x73U, 0xCCU }
//#define SONY_MSG_SND_3 { 0xBFU, 0x02U, 0x44U, 0x90U, 0x92U, 0xDDU }
#define SONY_MSG_RCV_3 { 0xBBU, 0x33U, 0x79U, 0x11U, 0x63U, 0xCCU }
//#define SONY_MSG_SND_4 { 0xB6U, 0x07U, 0x67U, 0x76U, 0x30U, 0x74U }
#define SONY_MSG_RCV_4 { 0x7BU, 0x53U, 0x99U, 0x01U, 0x73U, 0xCCU }
//#define SONY_MSG_SND_5 { 0xB5U, 0x02U, 0x44U, 0x80U, 0x92U, 0x85U }
#define SONY_MSG_RCV_5 { 0x3BU, 0x33U, 0x44U, 0x01U, 0x63U, 0xC8U }
//#define SONY_MSG_SND_6 { 0x44U, 0x02U, 0x97U, 0x80U, 0x92U, 0x95U }
#define SONY_MSG_RCV_6 { 0x37U, 0x33U, 0x44U, 0x20U, 0x41U, 0xC7U }

uint8_t sonySndNoState( )
{
    return 0u;
}

uint8_t sonyRcv12( )
{
    return 0;
}
uint8_t sonyRcv13( )
{
    return 0;
}
uint8_t sonyRcv14( )
{
    return 0;
}
uint8_t sonyRcv15( )
{
    return 0;
}
uint8_t sonyRcv16( )
{
    return 0;
}

/*-----------------------------------------------------------------------------
 *      sendSonyMessage():  Send sony message
 *
 *  Parameters: SONY_send_t *sonyMsgOut
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB void sendSonyMessage( SONY_send_t *sonyMsgOut, uint8_t msgLen )
{
   unsigned char buf[sizeof(SONY_send_t)];
   memcpy(buf,sonyMsgOut,msgLen);
   memset((void *) buf[msgLen],(void *) '\0',sizeof(char));
   Uart5_write_text( buf );
}
/*-----------------------------------------------------------------------------
 *      sendExtSonyMessage():  Send extended length sony message
 *
 *  Parameters: SONY_send_long_t *sonyMsgOut
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB void sendExtSonyMessage( SONY_send_long_t *sonyMsgOut, uint8_t msgLen )
{
   unsigned char buf[VISCA_MAX_MSG_LEN];
   memcpy(buf,sonyMsgOut,msgLen);
   memset((void *) buf[msgLen],(void *) '\0',sizeof(char));
   Uart5_write_text( buf );
}

uint8_t hmi_tele_var=0x0F;                                                      // variables written to / from hmi
uint8_t hmi_wide_var=0x08;
uint16_t hmi_rgain=0x4F;
uint32_t hmi_zoom=0xEA67;
uint32_t hmi_focus=0xFF76;
uint8_t hmi_mem_num = 5u;
uint8_t hmi_color_gain=5u;
uint8_t hmi_2d_noise=0u;
uint8_t hmi_3d_noise=0u;
uint8_t hmi_vid_sys_sel=SV_720P60;
uint8_t hmi_pan_speed=0x10u;
uint8_t hmi_pan_tilt=0x0Au;
uint8_t g_visca_p,g_visca_q,g_visca_r,g_visca_s;
uint8_t g_visca_p1,g_visca_q1,g_visca_r1,g_visca_s1;
uint32_t g_visca_y,g_visca_z;
uint8_t g_p1,g_p2;

// =============== Define the keys each option has =============================
#define VISCA_HAS_NO_ON_OFF 1U                                                  // option has no on/off button
#define VISCA_HAS_BOTH 2U                                                       // option has on/off capability
#define VISCA_HAS_ON_OFF 3U                                                     // option has only on/off button
// =============== CAM Zoom ====================================================
#define SONY_ZOOM_USED                                                          // use the zoom template
#define SONY_MSG_SND_STOP VISCA_CAM_ZOOM_STOP(VISCA_CAM_ADDR)                   // stop message
#define SONY_SND_STOP_FUNC sonySndZoomStop                                      // stop function
#define SONY_MSG_SND_TELE_STD VISCA_CAM_ZOOM_TELE_STD(VISCA_CAM_ADDR)           // tele std message
#define SONY_SND_TELE_STD_FUNC sonySndZoomTeleStd                               // tele std function
#define SONY_MSG_SND_WIDE_STD VISCA_CAM_ZOOM_WIDE_STD(VISCA_CAM_ADDR)           // wide std message
#define SONY_SND_WIDE_STD_FUNC sonySndZoomWideStd                               // wide std function
#define SONY_MSG_SND_WIDE_VAR VISCA_CAM_ZOOM_WIDE_VAR(VISCA_CAM_ADDR,3u)        // wide var message
#define SONY_SND_WIDE_VAR_FUNC sonySndZoomWideVar                               // wide var function
#define SONY_MSG_SND_TELE_VAR VISCA_CAM_ZOOM_TELE_VAR(VISCA_CAM_ADDR,5u)        // tele var message
#define SONY_SND_TELE_VAR_FUNC sonySndZoomTeleVar                               // tele var function
#define SONY_MSG_SND_DIRECT VISCA_CAM_ZOOM_DIRECT(VISCA_CAM_ADDR)               // direct message
#define SONY_SND_DIRECT_FUNC sonySndZoomDirect                                  // direct function
#include "visca_generic.h"                                                      // generic functions for above actions
#undef SONY_ZOOM_USED                                                           // end using zoom template
// =============== CAM Focus ===================================================
#define SONY_FOCUS_USED                                                         // use the focus template
#undef SONY_MSG_SND_STOP                                                        // undefine for re-use
#undef SONY_SND_STOP_FUNC
#undef SONY_MSG_SND_TELE_STD
#undef SONY_SND_TELE_STD_FUNC
#undef SONY_MSG_SND_WIDE_STD
#undef SONY_SND_WIDE_STD_FUNC
#undef SONY_MSG_SND_DIRECT
#undef SONY_SND_DIRECT_FUNC
#define SONY_MSG_SND_STOP VISCA_CAM_FOCUS_STOP(VISCA_CAM_ADDR)                  // stop message
#define SONY_SND_STOP_FUNC sonySndFocusStop                                     // stop function
#define SONY_MSG_SND_TELE_STD VISCA_CAM_FOCUS_FAR_STD(VISCA_CAM_ADDR)           // far std message
#define SONY_SND_TELE_STD_FUNC sonySndFocusFarStd                               // far std function
#define SONY_MSG_SND_WIDE_STD VISCA_CAM_FOCUS_NEAR_STD(VISCA_CAM_ADDR)          // near std message
#define SONY_SND_WIDE_STD_FUNC sonySndFocusNearStd                              // near std function
#define SONY_MSG_SND_1PSH_AF VISCA_CAM_FOCUS_1PSH_AF(VISCA_CAM_ADDR)            // one push af message
#define SONY_SND_1PSH_AF_FUNC sonySndFocusOnePushAF                             // one push af function
#define SONY_MSG_SND_DIRECT2 VISCA_CAM_ZOOM_FOCUS_DIRECT(VISCA_CAM_ADDR,0x40u,0u,0u,0u,0u,0u,0u,0u)  // zoom focus direct message
#define SONY_SND_DIRECT2_FUNC sonySndZoomFocusDirect                            // zoom focus direct function
#define SONY_MSG_SND_DIRECT VISCA_CAM_FOCUS_DIRECT(VISCA_CAM_ADDR,0x40u,0u,0u,0u)  // direct message
#define SONY_SND_DIRECT_FUNC sonySndFocusDirect                                 // direct function
#include "visca_generic.h"                                                      // generic functions for above actions
#undef SONY_FOCUS_USED                                                          // end using focus template
// ================= PanTilt Options ===========================================
#define SONY_CAM_PAN_TILTDRIVE_USED                                             // using sony pan tilt drive commands
#define SONY_MSG_SND_1 VISCA_CAM_PAN_TILTDRIVE_UP(VISCA_CAM_ADDR,0u,0u)         // Up 8x 01 06 01 VV WW 03 01 FF
#define SONY_SND_1_FUNC SonySndCamPanTiltDriveUp
#define SONY_MSG_SND_2 VISCA_CAM_PAN_TILTDRIVE_DOWN(VISCA_CAM_ADDR,0u,0u)       // Down 8x 01 06 01 VV WW 03 02 FF
#define SONY_SND_2_FUNC SonySndCamPanTiltDriveDown
#define SONY_MSG_SND_3 VISCA_CAM_PAN_TILTDRIVE_LEFT(VISCA_CAM_ADDR,0u,0u)       // Left 8x 01 06 01 VV WW 01 03 FF
#define SONY_SND_3_FUNC SonySndCamPanTiltDriveLeft
#define SONY_MSG_SND_4 VISCA_CAM_PAN_TILTDRIVE_RIGHT(VISCA_CAM_ADDR,0u,0u)      // Right 8x 01 06 01 VV WW 02 03 FF
#define SONY_SND_4_FUNC SonySndCamPanTiltDriveRight
#define SONY_MSG_SND_5 VISCA_CAM_PAN_TILTDRIVE_DOWN_RIGHT(VISCA_CAM_ADDR,0u,0u) // DownRight 8x 01 06 01 VV WW 02 02 FF
#define SONY_SND_5_FUNC SonySndCamPanTiltDriveDownRight
#define SONY_MSG_SND_6 VISCA_CAM_PAN_TILTDRIVE_DOWN_LEFT(VISCA_CAM_ADDR,0u,0u)  // DownLeft 8x 01 06 01 VV WW 01 02 FF
#define SONY_SND_6_FUNC SonySndCamPanTiltDriveDownLeft
#define SONY_MSG_SND_7 VISCA_CAM_PAN_TILTDRIVE_UP_LEFT(VISCA_CAM_ADDR,0u,0u)    // UpLeft 8x 01 06 01 VV WW 01 01 FF
#define SONY_SND_7_FUNC SonySndCamPanTiltDriveUpLeft
#define SONY_MSG_SND_8 VISCA_CAM_PAN_TILTDRIVE_UP_RIGHT(VISCA_CAM_ADDR,0u,0u)   // UpRight 8x 01 06 01 VV WW 02 01 FF
#define SONY_SND_8_FUNC SonySndCamPanTiltDriveUpRight
#define SONY_MSG_SND_9 VISCA_CAM_PAN_TILTDRIVE_HOME(VISCA_CAM_ADDR)             // Home 8x 01 06 04 FF
#define SONY_SND_9_FUNC SonySndCamPanTiltDriveHome
#define SONY_MSG_SND_10 VISCA_CAM_PAN_TILTDRIVE_STOP(VISCA_CAM_ADDR,0u,0u)      // Stop 8x 01 06 01 VV WW 03 03 FF
#define SONY_SND_10_FUNC SonySndCamPanTiltDriveStop
#define SONY_MSG_SND_11 VISCA_CAM_PAN_TILTDRIVE_RESET(VISCA_CAM_ADDR)           // Reset 8x 01 06 05 FF
#define SONY_SND_11_FUNC SonySndCamPanTiltDriveReset
#define SONY_MSG_SND_12 VISCA_CAM_PAN_TILTDRIVE_ABS_POS(VISCA_CAM_ADDR,0u,0u)   // Absolute Position 8x 01 06 02 VV WW 0Y 0Y 0Y 0Y 0Z 0Z 0Z 0Z FF
#define SONY_SND_12_FUNC SonySndCamPanTiltDriveAbsPos
#define SONY_MSG_SND_13 VISCA_CAM_PAN_TILTDRIVE_REL_POS(VISCA_CAM_ADDR,0u,0u)   // Relative Position 8x 01 06 03 VV WW 0Y 0Y 0Y 0Y 0Z 0Z 0Z 0Z FF
#define SONY_SND_13_FUNC SonySndCamPanTiltDriveRelPos
#define SONY_MSG_SND_14 VISCA_CAM_PAN_TILTDRIVE_LIM_SET(VISCA_CAM_ADDR,0u)      // Limit Set 8x 01 06 07 00 0W 0Y 0Y 0Y 0Y 0Z 0Z 0Z 0Z FF
#define SONY_SND_14_FUNC SonySndCamPanTiltDriveLimSet
#define SONY_MSG_SND_15 VISCA_CAM_PAN_TILTDRIVE_LIM_CLR(VISCA_CAM_ADDR,0u)      // Limit Clear 8x 01 06 07 01 0W 07 0F 0F 0F 07 0F 0F 0F FF
#define SONY_SND_15_FUNC SonySndCamPanTiltDriveLimClr
#include "visca_generic.h"                                                      // generic functions for above actions
#undef SONY_CAM_PAN_TILTDRIVE_USED
// ================= Cam WB ====================================================
#undef SONY_MSG_SND_1                                                           // un-define for re-use
#undef SONY_MSG_SND_2
#undef SONY_MSG_SND_3
#undef SONY_MSG_SND_4                                                           // un-define for re-use
#undef SONY_MSG_SND_5
#undef SONY_MSG_SND_6
#undef SONY_MSG_SND_7                                                           // un-define for re-use
#undef SONY_MSG_SND_8
#undef SONY_SND_1_FUNC
#undef SONY_SND_2_FUNC
#undef SONY_SND_3_FUNC
#undef SONY_SND_4_FUNC
#undef SONY_SND_5_FUNC
#undef SONY_SND_6_FUNC
#undef SONY_SND_7_FUNC
#undef SONY_SND_8_FUNC
#define SONY_CAM_WB_USED                                                        // using cam wb comands
#define SONY_MSG_SND_1 VISCA_CAM_WB_AUTO(VISCA_CAM_ADDR)                        // Auto 8x 01 04 35 00 FF
#define SONY_SND_1_FUNC SonySndCamWBAuto
#define SONY_MSG_SND_2 VISCA_CAM_WB_INDOOR(VISCA_CAM_ADDR)                      // Indoor 8x 01 04 35 01 FF
#define SONY_SND_2_FUNC SonySndCamWBIndoor
#define SONY_MSG_SND_3 VISCA_CAM_WB_OUTDOOR(VISCA_CAM_ADDR)                     // Outdoor 8x 01 04 35 02 FF
#define SONY_SND_3_FUNC SonySndCamWBOutdoor
#define SONY_MSG_SND_4 VISCA_CAM_WB_1PSH(VISCA_CAM_ADDR)                        // OnePush 8x 01 04 35 03 FF
#define SONY_SND_4_FUNC SonySndCamWBOnePush
#define SONY_MSG_SND_5 VISCA_CAM_WB_MAN(VISCA_CAM_ADDR)                         // Manual 8x 01 04 35 05 FF
#define SONY_SND_5_FUNC SonySndCamWBOneMan
#define SONY_MSG_SND_6 VISCA_CAM_WB_OUT_AUTO(VISCA_CAM_ADDR)                    // Outdoor Auto 8x 01 04 35 06 FF
#define SONY_SND_6_FUNC SonySndCamWBOutdoorAuto
#define SONY_MSG_SND_7 VISCA_CAM_WB_SOD_LMP_AUTO(VISCA_CAM_ADDR)                // Sodium Lamp Auto 8x 01 04 35 07 FF
#define SONY_SND_7_FUNC SonySndCamWBSodiumLampAuto
#define SONY_MSG_SND_8 VISCA_CAM_WB_SOD_AUTO(VISCA_CAM_ADDR)                    // Sodium Auto 8x 01 04 35 08 FF
#define SONY_SND_8_FUNC SonySndCamWBSodiumAuto
#include "visca_generic.h"                                                      // generic functions for above actions
#undef SONY_CAM_WB_USED
// ================= Cam AE ====================================================
#define SONY_CAM_AE_USED                                                        // using the sony ae cam
#undef SONY_MSG_SND_1                                                           // un-define for re-use
#undef SONY_MSG_SND_2
#undef SONY_MSG_SND_3
#undef SONY_SND_1_FUNC
#undef SONY_SND_2_FUNC
#undef SONY_SND_3_FUNC
#define SONY_MSG_SND_1 VISCA_CAM_AE_FULL_AUTO(VISCA_CAM_ADDR)                   // 8x 01 04 39 00 FF
#define SONY_SND_1_FUNC SonySndCamAEFullAuto
#define SONY_MSG_SND_2 VISCA_CAM_AE_MAN(VISCA_CAM_ADDR)                         // 8x 01 04 39 03 FF
#define SONY_SND_2_FUNC SonySndCamAEMan
#define SONY_MSG_SND_3 VISCA_CAM_AE_BRIGHT(VISCA_CAM_ADDR)                      // 8x 01 04 39 0D FF
#define SONY_SND_3_FUNC SonySndCamAEBright
#include "visca_generic.h"
#undef SONY_CAM_AE_USED
// ================= Cam Memory ================================================
#define SONY_CAM_MEM_USED                                                       // using the sony memory commands
#undef SONY_MSG_SND_1                                                           // un-define for re-use
#undef SONY_MSG_SND_2
#undef SONY_MSG_SND_3
#undef SONY_SND_1_FUNC
#undef SONY_SND_2_FUNC
#undef SONY_SND_3_FUNC
#define SONY_MSG_SND_1 VISCA_CAM_MEM_RESET(VISCA_CAM_ADDR,0u)                   // 8x 01 04 3F 00 0p FF
#define SONY_SND_1_FUNC SonySndCamMemReset
#define SONY_MSG_SND_2 VISCA_CAM_MEM_SET(VISCA_CAM_ADDR,0u)                     // 8x 01 04 3F 01 0p FF
#define SONY_SND_2_FUNC SonySndCamMemSet
#define SONY_MSG_SND_3 VISCA_CAM_MEM_RECALL(VISCA_CAM_ADDR,0u)                  // 8x 01 04 3F 02 0p FF
#define SONY_SND_3_FUNC SonySndCamMemRecall
#include "visca_generic.h"
#undef SONY_CAM_MEM_USED
// ================= Cam Misc ================================================
#define SONY_CAM_MISC_USED                                                      // using the sony miscelaneous commands
#undef SONY_MSG_SND_1                                                           // un-define for re-use
#undef SONY_MSG_SND_2
#undef SONY_MSG_SND_3
#undef SONY_SND_1_FUNC
#undef SONY_SND_2_FUNC
#undef SONY_SND_3_FUNC
#define SONY_MSG_SND_1 VISCA_CAM_ADDR_SET                                       // 88 30 01 FF
#define SONY_SND_1_FUNC SonySndAddrSet
#define SONY_MSG_SND_2 VISCA_CAM_IF_CLR                                         // 88 01 00 01 FF
#define SONY_SND_2_FUNC SonySndCamIFClr
#define SONY_MSG_SND_3 VISCA_CAM_CMD_CANCEL(VISCA_CAM_ADDR)                     // 8x 21 FF
#define SONY_SND_3_FUNC SonySndCamCmdCancel
#include "visca_generic.h"
#undef SONY_CAM_MISC_USED
// ================= Cam Video System ==========================================
#define SONY_CAM_VID_SYS_USED                                                   // using the sony video system controls
#undef SONY_MSG_SND_1                                                           // un-define for re-use
#undef SONY_MSG_SND_2
#undef SONY_MSG_SND_3
#undef SONY_MSG_SND_4
#undef SONY_MSG_SND_5
#undef SONY_SND_1_FUNC
#undef SONY_SND_2_FUNC
#undef SONY_SND_3_FUNC
#undef SONY_SND_4_FUNC
#undef SONY_SND_5_FUNC
#define SONY_MSG_SND_1 VISCA_CAM_MEM_RESET(VISCA_CAM_ADDR,0u)                   // 8x 01 04 3F 00 0p FF
#define SONY_SND_1_FUNC SonySndVidSysColorGain
#define SONY_MSG_SND_2 VISCA_CAM_MEM_SET(VISCA_CAM_ADDR,0u)                     // 8x 01 04 3F 01 0p FF
#define SONY_SND_2_FUNC SonySndVidSys2DNoiseRed
#define SONY_MSG_SND_3 VISCA_CAM_MEM_RECALL(VISCA_CAM_ADDR,0u)                  // 8x 01 04 3F 02 0p FF
#define SONY_SND_3_FUNC SonySndVidSys3DNoiseRed
#define SONY_MSG_SND_4 VISCA_CAM_VID_SYS_SET_DIRECT(VISCA_CAM_ADDR,0u)          // video system select 8x 01 06 35 00 pp FF
#define SONY_SND_4_FUNC SonySndCamVidSysSet
#define SONY_MSG_SND_5 VISCA_CAM_ID_WRITE_DIRECT(VISCA_CAM_ADDR,0u,0u,0u,0u)    // idwrite (environment associated with preset load) 8x 01 04 22 0p 0q 0r 0s FF
#define SONY_SND_5_FUNC SonySndCamVidSysIDWrite
#define FUNC_IS_IDWRITE 6u                                                      // position of idwrite in jump table for parameter passing
#include "visca_generic.h"                                                      // generic functions for above actions
#undef SONY_CAM_VID_SYS_USED
// ================ RGAIN Option ===============================================
#undef SONY_MSG_SND_DIRECT
#undef SONY_SND_DIRECT_FUNC
#define SONY_OPTION_SET VISCA_HAS_NO_ON_OFF                                     // has no on/off capability
#define SONY_MSG_SND_RESET VISCA_CAM_RGAIN_RESET(VISCA_CAM_ADDR)                // reset message
#define SONY_SND_RESET_FUNC sonySndRgainReset                                   // reset function
#define SONY_MSG_SND_UP VISCA_CAM_RGAIN_UP(VISCA_CAM_ADDR)                      // up message
#define SONY_SND_UP_FUNC sonySndRgainUp                                         // up function
#define SONY_MSG_SND_DOWN VISCA_CAM_RGAIN_DOWN(VISCA_CAM_ADDR)                  // down message
#define SONY_SND_DOWN_FUNC sonySndRgainDown                                     // down function
#define SONY_MSG_SND_DIRECT VISCA_CAM_RGAIN_DIRECT(VISCA_CAM_ADDR,3u,5u)        // direct default message
#define SONY_SND_DIRECT_FUNC sonySndRgainDirect                                 // direct function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ BGAIN Option ===============================================
#undef SONY_MSG_SND_RESET
#undef SONY_SND_RESET_FUNC
#undef SONY_MSG_SND_UP
#undef SONY_SND_UP_FUNC
#undef SONY_MSG_SND_DOWN
#undef SONY_SND_DOWN_FUNC
#undef SONY_MSG_SND_DIRECT
#undef SONY_SND_DIRECT_FUNC
//#undef SONY_OPTION_SET                                                        ----- un comment if moved
//#define SONY_OPTION_SET VISCA_HAS_NO_ON_OFF                                     // has no on/off capability
#define SONY_MSG_SND_RESET VISCA_CAM_BGAIN_RESET(VISCA_CAM_ADDR)                // reset message
#define SONY_SND_RESET_FUNC sonySndBgainReset                                   // reset function
#define SONY_MSG_SND_UP VISCA_CAM_BGAIN_UP(VISCA_CAM_ADDR)                      // up message
#define SONY_SND_UP_FUNC sonySndBgainUp                                         // up function
#define SONY_MSG_SND_DOWN VISCA_CAM_BGAIN_DOWN(VISCA_CAM_ADDR)                  // down message
#define SONY_SND_DOWN_FUNC sonySndBgainDown                                     // down function
#define SONY_MSG_SND_DIRECT VISCA_CAM_BGAIN_DIRECT(VISCA_CAM_ADDR,3u,5u)        // direct default message
#define SONY_SND_DIRECT_FUNC sonySndBgainDirect                                 // direct function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM Shutter Option =========================================
#undef SONY_MSG_SND_RESET
#undef SONY_SND_RESET_FUNC
#undef SONY_MSG_SND_UP
#undef SONY_SND_UP_FUNC
#undef SONY_MSG_SND_DOWN
#undef SONY_SND_DOWN_FUNC
#undef SONY_MSG_SND_DIRECT
#undef SONY_SND_DIRECT_FUNC
//#undef SONY_OPTION_SET
//#define SONY_OPTION_SET VISCA_HAS_NO_ON_OFF                                     // has no on/off capability
#define SONY_MSG_SND_RESET VISCA_CAM_SHUT_RESET(VISCA_CAM_ADDR)                 // reset message
#define SONY_SND_RESET_FUNC sonySndShutReset                                    // reset function
#define SONY_MSG_SND_UP VISCA_CAM_SHUT_UP(VISCA_CAM_ADDR)                       // up message
#define SONY_SND_UP_FUNC sonySndShutUp                                          // up function
#define SONY_MSG_SND_DOWN VISCA_CAM_SHUT_DOWN(VISCA_CAM_ADDR)                   // down message
#define SONY_SND_DOWN_FUNC sonySndShutDown                                      // down function
#define SONY_MSG_SND_DIRECT VISCA_CAM_SHUT_DIRECT(VISCA_CAM_ADDR,3u,5u)         // direct default message
#define SONY_SND_DIRECT_FUNC sonySndShutDirect                                  // direct function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM Iris Option =========================================
#undef SONY_MSG_SND_RESET
#undef SONY_SND_RESET_FUNC
#undef SONY_MSG_SND_UP
#undef SONY_SND_UP_FUNC
#undef SONY_MSG_SND_DOWN
#undef SONY_SND_DOWN_FUNC
#undef SONY_MSG_SND_DIRECT
#undef SONY_SND_DIRECT_FUNC
//#undef SONY_OPTION_SET
//#define SONY_OPTION_SET VISCA_HAS_NO_ON_OFF                                     // has no on/off capability
#define SONY_MSG_SND_RESET VISCA_CAM_IRIS_RESET(VISCA_CAM_ADDR)                 // reset message
#define SONY_SND_RESET_FUNC sonySndIrisReset                                    // reset function
#define SONY_MSG_SND_UP VISCA_CAM_IRIS_UP(VISCA_CAM_ADDR)                       // up message
#define SONY_SND_UP_FUNC sonySndIrisUp                                          // up function
#define SONY_MSG_SND_DOWN VISCA_CAM_IRIS_DOWN(VISCA_CAM_ADDR)                   // down message
#define SONY_SND_DOWN_FUNC sonySndIrisDown                                      // down function
#define SONY_MSG_SND_DIRECT VISCA_CAM_IRIS_DIRECT(VISCA_CAM_ADDR,3u,5u)         // direct default message
#define SONY_SND_DIRECT_FUNC sonySndIrisDirect                                  // direct function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM Gain Option =========================================
#undef SONY_MSG_SND_RESET
#undef SONY_SND_RESET_FUNC
#undef SONY_MSG_SND_UP
#undef SONY_SND_UP_FUNC
#undef SONY_MSG_SND_DOWN
#undef SONY_SND_DOWN_FUNC
#undef SONY_MSG_SND_DIRECT
#undef SONY_SND_DIRECT_FUNC
//#undef SONY_OPTION_SET
//#define SONY_OPTION_SET VISCA_HAS_NO_ON_OFF                                     // has no on/off capability
#define SONY_MSG_SND_RESET VISCA_CAM_GAIN_RESET(VISCA_CAM_ADDR)                 // reset message
#define SONY_SND_RESET_FUNC sonySndGainReset                                    // reset function
#define SONY_MSG_SND_UP VISCA_CAM_GAIN_UP(VISCA_CAM_ADDR)                       // up message
#define SONY_SND_UP_FUNC sonySndGainUp                                          // up function
#define SONY_MSG_SND_DOWN VISCA_CAM_GAIN_DOWN(VISCA_CAM_ADDR)                   // down message
#define SONY_SND_DOWN_FUNC sonySndGainDown                                      // down function
#define SONY_MSG_SND_DIRECT VISCA_CAM_GAIN_DIRECT(VISCA_CAM_ADDR,3u,5u)         // direct default message
#define SONY_SND_DIRECT_FUNC sonySndGainDirect                                  // direct function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM Bright Option =========================================
#undef SONY_MSG_SND_RESET
#undef SONY_SND_RESET_FUNC
#undef SONY_MSG_SND_UP
#undef SONY_SND_UP_FUNC
#undef SONY_MSG_SND_DOWN
#undef SONY_SND_DOWN_FUNC
#undef SONY_MSG_SND_DIRECT
#undef SONY_SND_DIRECT_FUNC
//#undef SONY_OPTION_SET
//#define SONY_OPTION_SET VISCA_HAS_NO_ON_OFF                                     // has no on/off capability
#define SONY_MSG_SND_RESET VISCA_CAM_BRIGHT_RESET(VISCA_CAM_ADDR)               // reset message
#define SONY_SND_RESET_FUNC sonySndBrightReset                                  // reset function
#define SONY_MSG_SND_UP VISCA_CAM_BRIGHT_UP(VISCA_CAM_ADDR)                     // up message
#define SONY_SND_UP_FUNC sonySndBrightUp                                        // up function
#define SONY_MSG_SND_DOWN VISCA_CAM_BRIGHT_DOWN(VISCA_CAM_ADDR)                 // down message
#define SONY_SND_DOWN_FUNC sonySndBrightDown                                    // down function
#define SONY_MSG_SND_DIRECT VISCA_CAM_BRIGHT_DIRECT(VISCA_CAM_ADDR,3u,5u)       // direct default message
#define SONY_SND_DIRECT_FUNC sonySndBrightDirect                                // direct function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM Aperture Option ========================================
#undef SONY_MSG_SND_RESET
#undef SONY_SND_RESET_FUNC
#undef SONY_MSG_SND_UP
#undef SONY_SND_UP_FUNC
#undef SONY_MSG_SND_DOWN
#undef SONY_SND_DOWN_FUNC
#undef SONY_MSG_SND_DIRECT
#undef SONY_SND_DIRECT_FUNC
//#undef SONY_OPTION_SET
//#define SONY_OPTION_SET VISCA_HAS_NO_ON_OFF                                     // has no on/off capability
#define SONY_MSG_SND_RESET VISCA_CAM_AP_RESET(VISCA_CAM_ADDR)                   // reset message
#define SONY_SND_RESET_FUNC sonySndApReset                                      // reset function
#define SONY_MSG_SND_UP VISCA_CAM_AP_UP(VISCA_CAM_ADDR)                         // up message
#define SONY_SND_UP_FUNC sonySndApUp                                            // up function
#define SONY_MSG_SND_DOWN VISCA_CAM_AP_DOWN(VISCA_CAM_ADDR)                     // down message
#define SONY_SND_DOWN_FUNC sonySndApDown                                        // down function
#define SONY_MSG_SND_DIRECT VISCA_CAM_AP_DIRECT(VISCA_CAM_ADDR,3u,5u)           // direct default message
#define SONY_SND_DIRECT_FUNC sonySndApDirect                                    // direct function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM ExpComp Option ========================================
#undef SONY_MSG_SND_RESET
#undef SONY_SND_RESET_FUNC
#undef SONY_MSG_SND_UP
#undef SONY_SND_UP_FUNC
#undef SONY_MSG_SND_DOWN
#undef SONY_SND_DOWN_FUNC
#undef SONY_MSG_SND_DIRECT
#undef SONY_SND_DIRECT_FUNC
#undef SONY_OPTION_SET                                                          // change the option set
#define SONY_OPTION_SET VISCA_HAS_BOTH                                          // has on/off capability as well
#define SONY_MSG_SND_RESET VISCA_CAM_EXP_RESET(VISCA_CAM_ADDR)                  // reset message
#define SONY_SND_RESET_FUNC sonySndExpReset                                     // reset function
#define SONY_MSG_SND_UP VISCA_CAM_EXP_UP(VISCA_CAM_ADDR)                        // up message
#define SONY_SND_UP_FUNC sonySndExpUp                                           // up function
#define SONY_MSG_SND_DOWN VISCA_CAM_EXP_DOWN(VISCA_CAM_ADDR)                    // down message
#define SONY_SND_DOWN_FUNC sonySndExpDown                                       // down function
#define SONY_MSG_SND_DIRECT VISCA_CAM_EXP_DIRECT(VISCA_CAM_ADDR,3u,5u)          // direct default message
#define SONY_SND_DIRECT_FUNC sonySndExpDirect                                   // direct function
#define SONY_MSG_SND_ON VISCA_CAM_EXP_ON(VISCA_CAM_ADDR)                        // on message
#define SONY_SND_ON_FUNC sonySndExpOn                                           // on function
#define SONY_MSG_SND_OFF VISCA_CAM_EXP_OFF(VISCA_CAM_ADDR)                      // on message
#define SONY_SND_OFF_FUNC sonySndExpOff                                         // on function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM_BackLight Option ========================================
#undef SONY_OPTION_SET                                                          // change the option set
#define SONY_OPTION_SET VISCA_HAS_ON_OFF                                        // has only on/off capability
#undef SONY_MSG_SND_ON
#undef SONY_SND_ON_FUNC
#undef SONY_MSG_SND_OFF
#undef SONY_SND_OFF_FUNC
#define SONY_MSG_SND_ON VISCA_CAM_BK_LT_ON(VISCA_CAM_ADDR)                      // on message
#define SONY_SND_ON_FUNC sonySndBkLtOn                                          // on function
#define SONY_MSG_SND_OFF VISCA_CAM_BK_LT_OFF(VISCA_CAM_ADDR)                    // on message
#define SONY_SND_OFF_FUNC sonySndBkLtOff                                        // on function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM_LR_Reverse Option ========================================
//#undef SONY_OPTION_SET                                                          // change the option set
//#define SONY_OPTION_SET VISCA_HAS_ON_OFF                                        // has only on/off capability
#undef SONY_MSG_SND_ON
#undef SONY_SND_ON_FUNC
#undef SONY_MSG_SND_OFF
#undef SONY_SND_OFF_FUNC
#define SONY_MSG_SND_ON VISCA_CAM_LR_REV_ON(VISCA_CAM_ADDR)                     // on message
#define SONY_SND_ON_FUNC sonySndLrRevOn                                         // on function
#define SONY_MSG_SND_OFF VISCA_CAM_LR_REV_OFF(VISCA_CAM_ADDR)                   // on message
#define SONY_SND_OFF_FUNC sonySndLrRevOff                                       // on function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM_Picture Flip Option ====================================
//#undef SONY_OPTION_SET                                                          // change the option set
//#define SONY_OPTION_SET VISCA_HAS_ON_OFF                                        // has only on/off capability
#undef SONY_MSG_SND_ON
#undef SONY_SND_ON_FUNC
#undef SONY_MSG_SND_OFF
#undef SONY_SND_OFF_FUNC
#define SONY_MSG_SND_ON VISCA_CAM_PFLIP_ON(VISCA_CAM_ADDR)                      // on message
#define SONY_SND_ON_FUNC sonySndPFlipOn                                         // on function
#define SONY_MSG_SND_OFF VISCA_CAM_PFLIP_OFF(VISCA_CAM_ADDR)                    // on message
#define SONY_SND_OFF_FUNC sonySndPFlipOff                                       // on function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM_IR Transfer Option ====================================
//#undef SONY_OPTION_SET                                                          // change the option set
//#define SONY_OPTION_SET VISCA_HAS_ON_OFF                                        // has only on/off capability
#undef SONY_MSG_SND_ON
#undef SONY_SND_ON_FUNC
#undef SONY_MSG_SND_OFF
#undef SONY_SND_OFF_FUNC
#define SONY_MSG_SND_ON VISCA_CAM_IRT_ON(VISCA_CAM_ADDR)                        // on message
#define SONY_SND_ON_FUNC sonySndIrtOn                                           // on function
#define SONY_MSG_SND_OFF VISCA_CAM_IRT_OFF(VISCA_CAM_ADDR)                      // on message
#define SONY_SND_OFF_FUNC sonySndIrtOff                                         // on function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM_IR Receive Return Option ===============================
//#undef SONY_OPTION_SET                                                          // change the option set
//#define SONY_OPTION_SET VISCA_HAS_ON_OFF                                        // has only on/off capability
#undef SONY_MSG_SND_ON
#undef SONY_SND_ON_FUNC
#undef SONY_MSG_SND_OFF
#undef SONY_SND_OFF_FUNC
#define SONY_MSG_SND_ON VISCA_CAM_IRRR_ON(VISCA_CAM_ADDR)                       // on message
#define SONY_SND_ON_FUNC sonySndIrrrOn                                          // on function
#define SONY_MSG_SND_OFF VISCA_CAM_IRRR_OFF(VISCA_CAM_ADDR)                     // on message
#define SONY_SND_OFF_FUNC sonySndIrrrOff                                        // on function
#define SONY_WANT_TOGGLE                                                        // turn on the use of the toggle function
#define SONY_MSG_SND_TOGGLE VISCA_CAM_IRR_TOGGLE(VISCA_CAM_ADDR)                // toggle message
#define SONY_SND_TOGGLE_FUNC sonySndIrrrToggle                                  // toggle function
#include "visca_generic.h"                                                      // generic functions for above actions
#undef SONY_WANT_TOGGLE                                                         // turn off the include of toggle function
// ================ CAM_IR Receive Option ===============================
//#undef SONY_OPTION_SET                                                          // change the option set
//#define SONY_OPTION_SET VISCA_HAS_ON_OFF                                        // has only on/off capability
#undef SONY_MSG_SND_ON
#undef SONY_SND_ON_FUNC
#undef SONY_MSG_SND_OFF
#undef SONY_SND_OFF_FUNC
#define SONY_MSG_SND_ON VISCA_CAM_IRR_ON(VISCA_CAM_ADDR)                        // on message
#define SONY_SND_ON_FUNC sonySndIrrOn                                           // on function
#define SONY_MSG_SND_OFF VISCA_CAM_IRR_OFF(VISCA_CAM_ADDR)                      // on message
#define SONY_SND_OFF_FUNC sonySndIrrOff                                         // on function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM_Flick Option ===============================
//#undef SONY_OPTION_SET                                                          // change the option set
//#define SONY_OPTION_SET VISCA_HAS_ON_OFF                                        // has only on/off capability
#undef SONY_MSG_SND_ON
#undef SONY_SND_ON_FUNC
#undef SONY_MSG_SND_OFF
#undef SONY_SND_OFF_FUNC
#define SONY_MSG_SND_ON VISCA_CAM_FLICK_50Hz                                    // 50hz message
#define SONY_SND_ON_FUNC sonySndFlick50Hz                                       // 50hz function
#define SONY_MSG_SND_OFF VISCA_CAM_FLICK_60Hz                                   // 60hz message
#define SONY_SND_OFF_FUNC sonySndFlick60Hz                                      // 60hz function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM_Freeze Mode Option ===============================
//#undef SONY_OPTION_SET                                                          // change the option set
//#define SONY_OPTION_SET VISCA_HAS_ON_OFF                                        // has only on/off capability
#undef SONY_MSG_SND_ON
#undef SONY_SND_ON_FUNC
#undef SONY_MSG_SND_OFF
#undef SONY_SND_OFF_FUNC
#define SONY_MSG_SND_ON VISCA_CAM_FREEZE_ON(VISCA_CAM_ADDR)                     // freeze on message
#define SONY_SND_ON_FUNC sonySndFreezeOn                                        // freeze on function
#define SONY_MSG_SND_OFF VISCA_CAM_FREEZE_OFF(VISCA_CAM_ADDR)                   // freeze off message
#define SONY_SND_OFF_FUNC sonySndFreezeOff                                      // freeze off function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM_Preset Freeze Mode Option ==============================
//#undef SONY_OPTION_SET                                                          // change the option set
//#define SONY_OPTION_SET VISCA_HAS_ON_OFF                                        // has only on/off capability
#undef SONY_MSG_SND_ON
#undef SONY_SND_ON_FUNC
#undef SONY_MSG_SND_OFF
#undef SONY_SND_OFF_FUNC
#define SONY_MSG_SND_ON VISCA_CAM_PRE_FREEZE_ON(VISCA_CAM_ADDR)                 // preset freeze on message
#define SONY_SND_ON_FUNC sonySndPreFreezeOn                                     // preset freeze on function
#define SONY_MSG_SND_OFF VISCA_CAM_PRE_FREEZE_OFF(VISCA_CAM_ADDR)               // preset freeze off message
#define SONY_SND_OFF_FUNC sonySndPreFreezeOff                                   // preset freeze off function
#include "visca_generic.h"                                                      // preset generic functions for above actions
// ================ CAM_Mount Mode Option ===============================
//#undef SONY_OPTION_SET                                                          // change the option set
//#define SONY_OPTION_SET VISCA_HAS_ON_OFF                                        // has only on/off capability
#undef SONY_MSG_SND_ON
#undef SONY_SND_ON_FUNC
#undef SONY_MSG_SND_OFF
#undef SONY_SND_OFF_FUNC
#define SONY_MSG_SND_ON VISCA_CAM_MM_UP(VISCA_CAM_ADDR)                         // mount mode up message
#define SONY_SND_ON_FUNC sonySndMMUp                                            // mount mode up function
#define SONY_MSG_SND_OFF VISCA_CAM_MM_DOWN(VISCA_CAM_ADDR)                      // mount mode down message
#define SONY_SND_OFF_FUNC sonySndMMDown                                         // mount mode down function
#include "visca_generic.h"                                                      // generic functions for above actions
// ================ CAM_Power Mode Option ===============================
//#undef SONY_OPTION_SET                                                          // change the option set
//#define SONY_OPTION_SET VISCA_HAS_ON_OFF                                        // has only on/off capability
#undef SONY_MSG_SND_ON
#undef SONY_SND_ON_FUNC
#undef SONY_MSG_SND_OFF
#undef SONY_SND_OFF_FUNC
#define SONY_MSG_SND_ON VISCA_CAM_PWR_ON(VISCA_CAM_ADDR)                        // power up message
#define SONY_SND_ON_FUNC sonySndPwrOn                                           // power up function
#define SONY_MSG_SND_OFF VISCA_CAM_PWR_OFF(VISCA_CAM_ADDR)                      // power down message
#define SONY_SND_OFF_FUNC sonySndPwrOff                                         // power down function
#include "visca_generic.h"                                                      // generic functions for above actions

// --------------------- Visca On/Off State Inquiries --------------------------
//#undef SONY_OPTION_SET
#define VISCA_GET_ON_OFF_STATE
#undef SONY_MSG_SND_RESET
#undef SONY_SND_RESET_FUNC
#define SONY_MSG_SND_INQ SONY_SND_CAM_PWR_INQ(VISCA_CAM_ADDR)
#define VISCA_SND_MSG_LEN 5u
#define SONY_SND_INQ_FUNC sonySndPwrInq                                         // ====== power inquiry
#define SONY_RCV_STATE_FUNC sonyRcvCamPowerState                                // ======== receive function definitions
#define SONY_MSG_RCV_ON SONY_RCV_CAM_PWR_ON(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_OFF SONY_RCV_CAM_PWR_OFF(VISCA_CAM_ADDR)
#define VISCA_REPLY_MSG_LEN 5u
#include "visca_generic.h"

#undef SONY_MSG_SND_INQ                                                         // send inquiry defs
#undef SONY_SND_INQ_FUNC
#undef SONY_MSG_RCV_ON                                                          // receive inquiry defs
#undef SONY_MSG_RCV_OFF
#undef SONY_RCV_STATE_FUNC
#define SONY_MSG_SND_INQ SONY_SND_CAM_FOCUS_MODE_INQ(VISCA_CAM_ADDR)
//#define VISCA_SND_MSG_LEN 5u
#define SONY_SND_INQ_FUNC sonySndFocusInq                                       // ====== focus inquiry
#define SONY_RCV_STATE_FUNC sonyRcvCamFocusState                                // ======== receive function definitions
#define SONY_MSG_RCV_ON SONY_RCV_CAM_FOCUS_MODE_AUT(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_OFF SONY_RCV_CAM_FOCUS_MODE_MAN(VISCA_CAM_ADDR)
//#define VISCA_REPLY_MSG_LEN 5u
#include "visca_generic.h"

#undef SONY_MSG_SND_INQ                                                         // send inquiry defs
#undef SONY_SND_INQ_FUNC
#undef SONY_MSG_RCV_ON                                                          // receive inquiry defs
#undef SONY_MSG_RCV_OFF
#undef SONY_RCV_STATE_FUNC
#define SONY_MSG_SND_INQ SONY_SND_CAM_EXP_COMP_MODE_INQ(VISCA_CAM_ADDR)
//#define VISCA_SND_MSG_LEN 5u
#define SONY_SND_INQ_FUNC sonySndExposureInq                                    // ====== exposure inquiry
#define SONY_RCV_STATE_FUNC sonyRcvCamExposureState                             // ======== receive function definitions
#define SONY_MSG_RCV_ON SONY_RCV_CAM_EXP_COMP_MODE_ON(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_OFF SONY_RCV_CAM_EXP_COMP_MODE_OFF(VISCA_CAM_ADDR)
//#define VISCA_REPLY_MSG_LEN 5u
#include "visca_generic.h"

#undef SONY_MSG_SND_INQ                                                         // send inquiry defs
#undef SONY_SND_INQ_FUNC
#undef SONY_MSG_RCV_ON                                                          // receive inquiry defs
#undef SONY_MSG_RCV_OFF
#undef SONY_RCV_STATE_FUNC
#define SONY_MSG_SND_INQ SONY_SND_CAM_SYS_MENU_MODE_INQ(VISCA_CAM_ADDR)
//#define VISCA_SND_MSG_LEN 5u
#define SONY_SND_INQ_FUNC sonySndMenuInq                                        // ====== menu inquiry
#define SONY_RCV_STATE_FUNC sonyRcvCamMenuState                                 // ======== receive function definitions
#define SONY_MSG_RCV_ON SONY_RCV_CAM_SYS_MENU_MODE_ON(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_OFF SONY_RCV_CAM_SYS_MENU_MODE_OFF(VISCA_CAM_ADDR)
//#define VISCA_REPLY_MSG_LEN 5u
#include "visca_generic.h"

#undef SONY_MSG_SND_INQ                                                         // send inquiry defs
#undef SONY_SND_INQ_FUNC
#undef SONY_MSG_RCV_ON                                                          // receive inquiry defs
#undef SONY_MSG_RCV_OFF
#undef SONY_RCV_STATE_FUNC
#define SONY_MSG_SND_INQ SONY_SND_CAM_LR_REV_INQ(VISCA_CAM_ADDR)
//#define VISCA_SND_MSG_LEN 5u
#define SONY_SND_INQ_FUNC sonySndLRRevInq                                       // ====== lr rev inquiry
#define SONY_RCV_STATE_FUNC sonyRcvCamLRRevState                                // ======== receive function definitions
#define SONY_MSG_RCV_ON SONY_RCV_CAM_LR_REV_ON(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_OFF SONY_RCV_CAM_LR_REV_OFF(VISCA_CAM_ADDR)
//#define VISCA_REPLY_MSG_LEN 5u
#include "visca_generic.h"

#undef SONY_MSG_SND_INQ                                                         // send inquiry defs
#undef SONY_SND_INQ_FUNC
#undef SONY_MSG_RCV_ON                                                          // receive inquiry defs
#undef SONY_MSG_RCV_OFF
#undef SONY_RCV_STATE_FUNC
#define SONY_MSG_SND_INQ SONY_SND_CAM_PIC_FLIP_INQ(VISCA_CAM_ADDR)
//#define VISCA_SND_MSG_LEN 5u
#define SONY_SND_INQ_FUNC sonySndPicFlipInq                                     // ====== pic flip inquiry
#define SONY_RCV_STATE_FUNC sonyRcvCamPicFlipState                              // ======== receive function definitions
#define SONY_MSG_RCV_ON SONY_RCV_CAM_PIC_FLIP_ON(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_OFF SONY_RCV_CAM_PIC_FLIP_OFF(VISCA_CAM_ADDR)
//#define VISCA_REPLY_MSG_LEN 5u
#include "visca_generic.h"

#undef SONY_MSG_SND_INQ                                                         // send inquiry defs
#undef SONY_SND_INQ_FUNC
#undef SONY_MSG_RCV_ON                                                          // receive inquiry defs
#undef SONY_MSG_RCV_OFF
#undef SONY_RCV_STATE_FUNC
#define SONY_MSG_SND_INQ SONY_SND_CAM_IR_TRANS_INQ(VISCA_CAM_ADDR)
//#define VISCA_SND_MSG_LEN 5u
#define SONY_SND_INQ_FUNC sonySndIRTransInq                                     // ====== ir trans inquiry
#define SONY_RCV_STATE_FUNC sonyRcvCamIRTransState                              // ======== receive function definitions
#define SONY_MSG_RCV_ON SONY_RCV_CAM_IR_TRANS_ON(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_OFF SONY_RCV_CAM_IR_TRANS_OFF(VISCA_CAM_ADDR)
//#define VISCA_REPLY_MSG_LEN 5u
#include "visca_generic.h"

#undef SONY_MSG_SND_INQ                                                         // send inquiry defs
#undef SONY_SND_INQ_FUNC
#undef SONY_MSG_RCV_ON                                                          // receive inquiry defs
#undef SONY_MSG_RCV_OFF
#undef SONY_RCV_STATE_FUNC
#define SONY_MSG_SND_INQ SONY_SND_CAM_IR_RECV_INQ(VISCA_CAM_ADDR)
//#define VISCA_SND_MSG_LEN 5u
#define SONY_SND_INQ_FUNC sonySndIRRecvInq                                      // ====== ir receive inquiry
#define SONY_RCV_STATE_FUNC sonyRcvCamIRRecvState                               // ======== receive function definitions
#define SONY_MSG_RCV_ON SONY_RCV_CAM_IR_RECV_ON(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_OFF SONY_RCV_CAM_IR_RECV_OFF(VISCA_CAM_ADDR)
//#define VISCA_REPLY_MSG_LEN 5u
#include "visca_generic.h"

#undef SONY_MSG_SND_INQ
#undef VISCA_GET_ON_OFF_STATE
#define VISCA_GET_MULTI_STATE                                                   // now the replies are multi-state rather than on/off type
#undef SONY_SND_INQ_FUNC
#undef SONY_MSG_RCV_1
#undef SONY_MSG_RCV_2
#undef SONY_MSG_RCV_3
#undef SONY_MSG_RCV_4
#undef SONY_MSG_RCV_5
#undef SONY_MSG_RCV_6
#undef VISCA_REPLY_MSG_LEN
#define VISCA_REPLY_MSG_LEN 5u                                                  // reply length
#define SONY_SND_INQ_FUNC sonySndWBModelInq
#define SONY_RCV_MULTI_STATE_FUNC sonyRcvWBModelState
#define SONY_MSG_SND_INQ SONY_SND_CAM_WB_MODEL_INQ(VISCA_CAM_ADDR)              // inquiry
#define SONY_MSG_RCV_1 SONY_RCV_CAM_WB_MODEL_AUTO(VISCA_CAM_ADDR)               // responses
#define SONY_MSG_RCV_2 SONY_RCV_CAM_WB_MODEL_INDOOR(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_3 SONY_RCV_CAM_WB_MODEL_OUTDOOR(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_4 SONY_RCV_CAM_WB_MODEL_1PSH(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_5 SONY_RCV_CAM_WB_MODEL_ATW(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_6 SONY_RCV_CAM_WB_MODEL_MAN(VISCA_CAM_ADDR)
#define VISCA_NO_OF_STATE_REPLIES 6u                                            // 6 possible responses
#include "visca_generic.h"

#undef SONY_SND_INQ_FUNC
#undef SONY_RCV_MULTI_STATE_FUNC
#undef SONY_MSG_SND_INQ
#undef SONY_MSG_RCV_1
#undef SONY_MSG_RCV_2
#undef SONY_MSG_RCV_3
#undef SONY_MSG_RCV_4
#undef SONY_MSG_RCV_5
#undef SONY_MSG_RCV_6
#undef VISCA_REPLY_MSG_LEN
#define VISCA_REPLY_MSG_LEN 7u                                                  // reply length
#define SONY_SND_INQ_FUNC sonySndIRRecvInq
#define SONY_RCV_MULTI_STATE_FUNC sonyRcvIRRecvState
#define SONY_MSG_SND_INQ SONY_SND_CAM_IR_RECV_RET_INQ(VISCA_CAM_ADDR)           // inquiry
#define SONY_MSG_RCV_1 SONY_IR_RECV_RET_ON_OFF(VISCA_CAM_ADDR)                  // responses
#define SONY_MSG_RCV_2 SONY_IR_RECV_RET_ZOOM(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_3 SONY_IR_RECV_RET_AF_ON_OFF(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_4 SONY_IR_RECV_RET_BACKLT(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_5 SONY_IR_RECV_RET_MEMORY(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_6 SONY_IR_RECV_RET_PAN_TILTDRV(VISCA_CAM_ADDR)
#include "visca_generic.h"

#undef VISCA_NO_OF_STATE_REPLIES
#undef SONY_SND_INQ_FUNC
#undef SONY_RCV_MULTI_STATE_FUNC
#undef SONY_MSG_SND_INQ
#undef SONY_MSG_RCV_1
#undef SONY_MSG_RCV_2
#undef SONY_MSG_RCV_3
#undef SONY_MSG_RCV_4
#undef SONY_MSG_RCV_5
#undef SONY_MSG_RCV_6
#undef VISCA_REPLY_MSG_LEN
#define VISCA_REPLY_MSG_LEN 5u                                                  // reply length
#define SONY_SND_INQ_FUNC sonySndAEModeInq
#define SONY_RCV_MULTI_STATE_FUNC sonyRcvAEModeState
#define SONY_MSG_SND_INQ SONY_SND_CAM_AEMODE_INQ(VISCA_CAM_ADDR)                // inquiry
#define SONY_MSG_RCV_1 SONY_RCV_CAM_AEMODE_AUTO(VISCA_CAM_ADDR)                 // responses
#define SONY_MSG_RCV_2 SONY_RCV_CAM_AEMODE_SHUT(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_3 SONY_RCV_CAM_AEMODE_IRIS(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_4 SONY_RCV_CAM_AEMODE_BRIGHT(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_5 SONY_RCV_CAM_AEMODE_MAN(VISCA_CAM_ADDR)
#define SONY_MSG_RCV_6 SONY_RCV_CAM_AEMODE_MAN(VISCA_CAM_ADDR)
#define VISCA_NO_OF_STATE_REPLIES 5u                                            // now only 5 response
#include "visca_generic.h"

/*-----------------------------------------------------------------------------
 *      sonyRcv2():  Send sony message
 *
 *  Parameters: unsigned char *replyBuf, SONY_hmi_t *hmi
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB void sonyRcv2( unsigned char *replyBuf, SONY_hmi_t *hmi )
{
  if (hmi->msgStat == SONY_REQ_SENT)
  {
     SONY_rcv_t sonyMsgExp = SONY_MSG_RCV_2;
     SONY_rcv_t sonyMsgRcv;
     memcpy(&sonyMsgRcv,replyBuf,VISCA_MAX_MSG_LEN);
     if (!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp,sizeof(SONY_rcv_t)))
     {
       hmi->msgStat = SONY_REP_OK;
     }
     else
     {
       hmi->msgStat = SONY_REP_FAIL;
     }
  }
}

/*-----------------------------------------------------------------------------
 *      sonyRcv3():  Send sony message
 *
 *  Parameters: unsigned char *replyBuf, SONY_hmi_t *hmi
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB void sonyRcv3( unsigned char *replyBuf, SONY_hmi_t *hmi )
{
  if (hmi->msgStat == SONY_REQ_SENT)
  {
     SONY_rcv_t sonyMsgExp = SONY_MSG_RCV_3;
         SONY_rcv_t sonyMsgRcv;
         memcpy(&sonyMsgRcv,replyBuf,VISCA_MAX_MSG_LEN);
         if (!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp,sizeof(SONY_rcv_t)))
         {
            hmi->msgStat = SONY_REP_OK;
         }
         else
         {
            hmi->msgStat = SONY_REP_FAIL;
         }
  }
}
/*-----------------------------------------------------------------------------
 *      sonySnd4():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t sonySnd4( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_4;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 1;
}
/*-----------------------------------------------------------------------------
 *      sonyRcv4():  Send sony message
 *
 *  Parameters: unsigned char *replyBuf, SONY_hmi_t *hmi
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB void sonyRcv4( unsigned char *replyBuf, SONY_hmi_t *hmi )
{
  if (hmi->msgStat == SONY_REQ_SENT)
  {
     SONY_rcv_t sonyMsgExp = SONY_MSG_RCV_4;
         SONY_rcv_t sonyMsgRcv;
         memcpy(&sonyMsgRcv,replyBuf,VISCA_MAX_MSG_LEN);
         if (!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp,sizeof(SONY_rcv_t)))
         {
            hmi->msgStat = SONY_REP_OK;
         }
         else
         {
            hmi->msgStat = SONY_REP_FAIL;
         }
  }
}
/*-----------------------------------------------------------------------------
 *      sonySnd5():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t sonySnd5( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_5;
         sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
         return 1;
}
/*-----------------------------------------------------------------------------
 *      sonyRcv5():  Send sony message
 *
 *  Parameters: unsigned char *replyBuf, SONY_hmi_t *hmi
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB void sonyRcv5( unsigned char *replyBuf, SONY_hmi_t *hmi )
{
  if (hmi->msgStat == SONY_REQ_SENT)
  {
     SONY_rcv_t sonyMsgExp = SONY_MSG_RCV_5;
         SONY_rcv_t sonyMsgRcv;
         memcpy(&sonyMsgRcv,replyBuf,VISCA_MAX_MSG_LEN);
         if (!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp,sizeof(SONY_rcv_t)))
         {
            hmi->msgStat = SONY_REP_OK;
         }
         else
         {
            hmi->msgStat = SONY_REP_FAIL;
         }
  }
}
/*-----------------------------------------------------------------------------
 *      sonySnd6():  Send sony message
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB uint8_t sonySnd6( )
{
     SONY_send_t sonyMsg = SONY_MSG_SND_6;
     sendSonyMessage( &sonyMsg,VISCA_MSG_LEN );
     return 1;
}

/*-----------------------------------------------------------------------------
 *      sonyRcv5():  Send sony message
 *
 *  Parameters: unsigned char *replyBuf, SONY_hmi_t *hmi
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB void sonyRcv6( unsigned char *replyBuf, SONY_hmi_t *hmi )
{
  if (hmi->msgStat == SONY_REQ_SENT)
  {
     SONY_rcv_t sonyMsgExp = SONY_MSG_RCV_6;
     SONY_rcv_t sonyMsgRcv;
     memcpy(&sonyMsgRcv,replyBuf,VISCA_MAX_MSG_LEN);
     if (!memcmp((void*)&sonyMsgRcv,(void*)&sonyMsgExp,sizeof(SONY_rcv_t)))
     {
        hmi->msgStat = SONY_REP_OK;
     }
     else
     {
        hmi->msgStat = SONY_REP_FAIL;
     }
  }
}
/*-----------------------------------------------------------------------------
 *      chooseSendFunction():  Select binary message and send out of serial port
 *
 *  Parameters: uint8_t *hmiVal
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB void chooseSendFunction( uint8_t *hmiVal )
{
#ifdef jump_table
    //uint8_t retCode;
    if ((*hmiVal=0u)&&(*hmiVal<=SONY_MAX_REQ_CMD))
    {
       //retCode=
       *go[ *hmiVal ];
    }
#else
    switch (hmiVal)
    {
      case HMI_REQ_ON:
          sonySnd1();
          break;

          case HMI_REQ_OFF:
          sonySnd2();
          break;

          case HMI_REQ_RESET:
          sonySnd3();
          break;

          case HMI_REQ_UP:
          sonySnd3();
          break;

          case HMI_REQ_DOWN:
          sonySnd3();
          break;

          case HMI_REQ_DIRECT:
          sonySnd3();
          break;

          default:
          break;
    }
#endif
}
/*-----------------------------------------------------------------------------
 *      hmiSendFunction():  Hmi send function from hmi selction choose binary message and
 *                          send it out of serial report state back as sent
 *  Parameters: uint8_t *hmiVal
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB void hmiSendFunction( SONY_hmi_t *hmiVal )
{
#ifdef jump_table
     hmiVal->jumpTblIdx=hmiVal->jumpTblIdx % (SONY_MAX_REQ_CMD+1u);             // bound the jump table index to that of the enum type do not exceed
    //if ((hmiVal->jumpTblIdx>=0u)&&(hmiVal->jumpTblIdx<=SONY_MAX_REQ_CMD))     --- no need to check it
    //{
        switch(hmiVal->opt_sel)                                                 // HMI selects the rpup option as shown
        {
            case SONY_RGAIN:
            g_visca_p=VISCA_SCALE_PQ_TO_P(hmi_rgain);
            g_visca_q=VISCA_SCALE_PQ_TO_Q(hmi_rgain);
            if (hmiVal->jumpTblIdx < (sizeof(rgain) / sizeof(*rgain)))
            *rgain[ hmiVal->jumpTblIdx ];                                       // Call Rgain functions
            break;

            case SONY_BGAIN:
            if (hmiVal->jumpTblIdx < (sizeof(bgain) / sizeof(*bgain)))
            *bgain[ hmiVal->jumpTblIdx ];                                       // Call Bgain functions
            break;

            case SONY_SHUT:
            if (hmiVal->jumpTblIdx < (sizeof(shut) / sizeof(*shut)))
            *shut[ hmiVal->jumpTblIdx ];                                        // Call Shutter functions
            break;

            case SONY_IRIS:
            if (hmiVal->jumpTblIdx < (sizeof(iris) / sizeof(*iris)))
            *iris[ hmiVal->jumpTblIdx ];                                        // Call Iris functions
            break;

            case SONY_GAIN:
            if (hmiVal->jumpTblIdx < (sizeof(gain) / sizeof(*gain)))
            *gain[ hmiVal->jumpTblIdx ];                                        // Call Gain functions
            break;

           case SONY_BRIGHT:
            if (hmiVal->jumpTblIdx < (sizeof(bright) / sizeof(*bright)))
            *bright[ hmiVal->jumpTblIdx ];                                      // Call Brightness functions
            break;

            case SONY_AP:
            if (hmiVal->jumpTblIdx < (sizeof(ap) / sizeof(*ap)))
            *ap[ hmiVal->jumpTblIdx ];                                          // Call Aperture functions
            break;

            case SONY_EXPCOMP:
            if (hmiVal->jumpTblIdx < (sizeof(expcomp) / sizeof(*expcomp)))
            *expcomp[ hmiVal->jumpTblIdx ];                                     // Call Exposure compensation functions
            break;

            case SONY_BK_LT:
            if (hmiVal->jumpTblIdx < (sizeof(bklt) / sizeof(*bklt)))
            *bklt[ hmiVal->jumpTblIdx ];                                        // Call back light control functions
            break;

            case SONY_LR_REV:
            if (hmiVal->jumpTblIdx < (sizeof(lrrev) / sizeof(*lrrev)))
            *lrrev[ hmiVal->jumpTblIdx ];                                       // Call lr reverse control functions
            break;

            case SONY_PFLIP:
            if (hmiVal->jumpTblIdx < (sizeof(pflip) / sizeof(*pflip)))
            *pflip[ hmiVal->jumpTblIdx ];                                       // Call picture flip functions
            break;

            case SONY_IRT:
            if (hmiVal->jumpTblIdx < (sizeof(irt) / sizeof(*irt)))
            *irt[ hmiVal->jumpTblIdx ];                                         // Call infa red transfer control functions
            break;

            case SONY_IRRR:
            if (hmiVal->jumpTblIdx < (sizeof(irrr) / sizeof(*irrr)))
            *irrr[ hmiVal->jumpTblIdx ];                                         // Call infa red receive return functions
            break;

            case SONY_IRR:
            if (hmiVal->jumpTblIdx < (sizeof(irr) / sizeof(*irr)))
            *irr[ hmiVal->jumpTblIdx ];                                         // Call infa red receive control functions
            break;

            case SONY_FLICK:
            if (hmiVal->jumpTblIdx < (sizeof(flick) / sizeof(*flick)))
            *flick[ hmiVal->jumpTblIdx ];                                       // Call flick control functions
            break;

            case SONY_MM:
            if (hmiVal->jumpTblIdx < (sizeof(mm) / sizeof(*mm)))
            *mm[ hmiVal->jumpTblIdx ];                                          // Call mount mode functions
            break;

            case SONY_PWR:
            if (hmiVal->jumpTblIdx < (sizeof(pwr) / sizeof(*pwr)))
            *pwr[ hmiVal->jumpTblIdx ];                                         // Call power mode functions
            break;

            case SONY_ZOOM:
            g_p1=hmi_tele_var % 8u;                                             // range 0-7
            g_p2=hmi_wide_var % 8u;                                             // range 0-7
            g_visca_p=VISCA_SCALE_PQRS_TO_P(hmi_zoom);
            g_visca_q=VISCA_SCALE_PQRS_TO_Q(hmi_zoom);
            g_visca_r=VISCA_SCALE_PQRS_TO_R(hmi_zoom);
            g_visca_s=VISCA_SCALE_PQRS_TO_S(hmi_zoom);
            if (hmiVal->jumpTblIdx < (sizeof(zoom) / sizeof(*zoom)))
            *zoom[ hmiVal->jumpTblIdx ];                                        // Call zoom functions
            break;

            case SONY_FOCUS:
            g_visca_p1=VISCA_SCALE_PQRS_TO_P(hmi_focus);                        // convert focus value from GUI
            g_visca_q1=VISCA_SCALE_PQRS_TO_Q(hmi_focus);
            g_visca_r1=VISCA_SCALE_PQRS_TO_R(hmi_focus);
            g_visca_s1=VISCA_SCALE_PQRS_TO_S(hmi_focus);
            g_visca_p=VISCA_SCALE_PQRS_TO_P(hmi_zoom);                          // convert zoom value from GUI
            g_visca_q=VISCA_SCALE_PQRS_TO_Q(hmi_zoom);
            g_visca_r=VISCA_SCALE_PQRS_TO_R(hmi_zoom);
            g_visca_s=VISCA_SCALE_PQRS_TO_S(hmi_zoom);
            if (hmiVal->jumpTblIdx < (sizeof(zoom) / sizeof(*zoom)))
            *focus[ hmiVal->jumpTblIdx ];                                       // Call focus functions
            break;

            case SONY_CAMWB:
            if (hmiVal->jumpTblIdx < (sizeof(camwb) / sizeof(*camwb)))
            *camwb[ hmiVal->jumpTblIdx ];                                       // Call cam wb functions

            case SONY_CAMAE:
            if (hmiVal->jumpTblIdx < (sizeof(camae) / sizeof(*camae)))
            *camae[ hmiVal->jumpTblIdx ];                                       // Call cam ae functions

            case SONY_CAMMEM:
            g_visca_p1=hmi_mem_num % (VISCA_MAX_MEMORIES+1);                    // read hmi and limit to maximum number of memories
            if (hmiVal->jumpTblIdx < (sizeof(mem) / sizeof(*mem)))
            *mem[ hmiVal->jumpTblIdx ];                                         // Call cam memory management functions

            case SONY_VIDSYS:
            if ( hmiVal->jumpTblIdx != FUNC_IS_IDWRITE)                         // Not idWrite selection which is p1,p2,p3,p4 direct from hmi
            {
               g_visca_p1=hmi_color_gain % (0x0Eu+1u);                          // read hmi and limit to maximum number of color gain settings
               g_visca_q1=hmi_2d_noise % (0x05u+1u);                            // read hmi and limit to maximum number of 2d noise settings
               g_visca_r1=hmi_3d_noise % (0x03u+1u);                            // read hmi and limit to maximum number of 3d noise settings
               g_visca_s1=hmi_vid_sys_sel % NUM_OF_VIDEO_SYS;                   // read hmi and limit to maximum number of video systems
            }
            if (hmiVal->jumpTblIdx < (sizeof(vidsys) / sizeof(*vidsys)))
            *vidsys[ hmiVal->jumpTblIdx ];                                      // Call cam vidoe system functions

            case SONY_FREEZE:
            if (hmiVal->jumpTblIdx < (sizeof(freeze) / sizeof(*freeze)))
            *freeze[ hmiVal->jumpTblIdx ];                                      // Call freeze functions

            case SONY_PANTILTDRIVE:
            g_visca_p1=hmi_pan_speed % (VISCA_MAX_PAN_SPEED+1u);
            g_visca_q1=hmi_pan_tilt % (VISCA_MAX_TILT_SPEED+1u);
            if (hmiVal->jumpTblIdx < (sizeof(pantiltdrive) / sizeof(*pantiltdrive)))
            *pantiltdrive[ hmiVal->jumpTblIdx ];                                // call pan tilt drive functions

            case SONY_MISC:
            if (hmiVal->jumpTblIdx < (sizeof(misc) / sizeof(*misc)))
            *misc[ hmiVal->jumpTblIdx ];                                        // call miscelaneous camera functions

            default:
            break;
        }
    //}
#else
    switch (hmiVal->jumpTblIdx)
    {
         case HMI_REQ_ON:
         sonySnd1();
         break;

         case HMI_REQ_OFF:
         sonySnd2();
         break;

         case HMI_REQ_RESET:
         sonySnd3();
         break;

         case HMI_REQ_UP:
         sonySnd4();
         break;

         case HMI_REQ_DOWN:
         sonySnd5();
         break;

         case HMI_REQ_DIRECT:
         sonySnd6();
         break;

         default:
         break;
      }
#endif
      hmiVal->msgStat = SONY_REQ_SENT;
      hmiVal->tmr=0U;
}
/*-----------------------------------------------------------------------------
 *      hmiRcvFunction():  Hmi receive function check reply
 *  Parameters: uint8_t *hmiVal
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
SONY_LIB void hmiRcvFunction( unsigned char *rBuf, SONY_hmi_t *hmiVal )
{
  if (hmiVal->msgStat == SONY_REQ_SENT)
  {
#ifdef jump_table
    if ((hmiVal->jumpTblIdx>=0u)&&(hmiVal->jumpTblIdx<=SONY_MAX_REQ_CMD))
    {
       if (((uint8_t)*recv[ hmiVal->jumpTblIdx ])==1u)
       {
          hmiVal->msgStat = SONY_REP_OK;
       }
       else
       {
          hmiVal->msgStat = SONY_REP_FAIL;
       }
    }
#else
      switch (hmiVal->jumpTblIdx)
      {
         case HMI_REQ_ON:
         sonyRcv1( rBuf, hmiVal );
         break;

         case HMI_REQ_OFF:
         sonyRcv2( rBuf, hmiVal );
         break;

         case HMI_REQ_RESET:
         sonyRcv3( rBuf, hmiVal );
         break;

         case HMI_REQ_UP:
         sonyRcv4( rBuf, hmiVal );
         break;

         case HMI_REQ_DOWN:
         sonyRcv5( rBuf, hmiVal );
         break;

         case HMI_REQ_DIRECT:
         sonyRcv6( rBuf, hmiVal );
         break;

         default:
         break;
      }
#endif
      hmiVal->tmr=0U;
   }
}
//VISCA_SCALE_TO_P_PQ(hmi_rgain,g_visca_p);
//g_visca_q=VISCA_SCALE_TO_Q_PQ(hmi_rgain);
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif