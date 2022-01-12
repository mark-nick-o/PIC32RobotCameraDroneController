#ifndef  __ports__
#define  __ports__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// SBGC ports
#define SBGC_From_Port 20001U                                                   // SBGC gimbal over UDP controller ports
#define SBGC_Dest_Port 20002U                                                   // UDP receive port
#define SBGC_Ack_Port 20003U                                                    // UDP Ack port

// XS AMP Encoder Ports
#define XS_From_Port 20004U                                                     // XStream encoder message over UDP ports
#define XS_Dest_Port 20005U                                                     // UDP receive port
#define XS_Ack_Port 20006U                                                      // UDP Ack port
#define XS_RTSP_Port 554U                                                       // Encoder RTSP Port

// Run Cam Encoder ports
#define RC_From_Port 20007U                                                     // RunCam Camera message over UDP ports
#define RC_Dest_Port 20008U                                                     // Run Cam UDP receive port
#define RC_Ack_Port 20009U                                                      // Run Cam ACK port

// Yi Action Server
#define XY_DEF_PORT 7878U                                                       // remote control port, use JSON
#define XY_DATA_PORT 8787U                                                      // data port
#define XY_RTSP_PORT 554U                                                       // rtsp port
#define XY_WEB_PORT 80U                                                         // rtsp://192.168.42.1:80 is stream to web browser for files http://192.168.42.1/DCIM/100MEDIA/
#define XY_TELNET_PORT                                                          // Yi Action uses normal telnet for connection

// EGD Data protocol
#define EGD_PRO_CONS_PORT 18246U                                                // producer/consumer service port
#define EGD_CMD_PORT 7937U                                                      // command service port on UDP

// Mavlink over UDP
#define MAV_UDP_OUT 14540U
#define MAV_UDP_IN 14550U

// Rayfin Camera Ports
#define API_RAYF_DST_PORT 8888U                                                 /* destination port either TCP or UDP */
#define API_RAYF_SRC_PORT 61219U                                                /* source port either TCP or UDP */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif