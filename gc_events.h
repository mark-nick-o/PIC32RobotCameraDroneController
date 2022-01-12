#ifndef gc_events_H
#define gc_events_H
//    gc_events.h : General Event declarations
//
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
//

#ifdef __cplusplus
 extern "C" {
#endif

#include "Struts.h"
#include <stdint.h>
#include "definitions.h"
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
#include "SBGC.h"                                                               // SBGC general
#include "SBGC_rc.h"                                                            // SBGC rc commands
#include "SBGC_COMMAND.h"                                                       // SBGC commands
#include "SBGC_PARSER.h"                                                        // SBGC parser (mostly for aduino) some general lengths etc
#include "SBGC_cmd_helpers.h"                                                   // SBGC all message conatiners
#include "SBGC_adj_vars.h"                                                      // SBGC variables
#endif
#include "camera.h"
#include "io.h"
//#include "lwNx.h"

#if defined(JRT_LIDDAR_USED)
#include "jrt_lidar.h"
#endif

#if defined(VOLZ_PROTOCOL)
#include "volz_protocol.h"
#endif

#if defined(ROBOTIS_PROTOCOL)
#include "robotis.h"
#endif

#if defined(TRAMP_PROTOCOL)
#include "tramp_protocol.h"
#endif

#include "__NetEthInternal.h"

#ifdef RION_USED
#include "rion.h"
extern rionUTCDateTimeField_t  g_timeDate;                                      // global time using rion object
#endif

#if defined(UBIDOT_USED)
#ifndef HTTP_USED
#define HTTP_USED
#endif
#include "UbiDot.h"
#endif

#if defined(AIRMAP_USED)
#include "airmap.h"
#ifndef HTTP_USED
#define HTTP_USED
#ifndef PROTOBUF_USED                                                           /* we cant not have protobuf if we need airmap */
#define PROTOBUF_USED
#endif
#endif
#endif

#if defined(HTTP_USED)
#include "http.h"                                                               // include http helpers
#endif

#define ETHER_ENABLE_MAC_FILTER                                                 // enables MAC Address filtering
#define ETHER_USE_DEFINED_MAC                                                   // use the hardcoded MAC address rather than that in config.h

#define MAX_NUM_BAD_UDP 10                                                      // Reset if you get this many bad UDP sends

#define Ethernet_HALFDUPLEX     0U
#define Ethernet_FULLDUPLEX     1U

#if defined(USE_RS485)
extern sfr atomic sbit RS485_rxtx_pin;
extern sfr atomic sbit RS485_rxtx_pin_direction;
#endif

#ifndef bool
#define bool uint8_t
#endif

/* Solves the all-pairs shortest path
   problem using Floyd Warshall algorithm
   Number of vertices in the graph  */
#define FW_V 4
/* Define Infinite as a large enough
   value.This value will be used for
   vertices not connected to each other */
#define FW_INF INT16_MAX

// ================= Global Functions used =====================================
/*  extern uint16_t Net_Ethernet_Intern_UserUDP(UDP_Intern_Dsc *udpDsc); 
    extern void Net_Ethernet_Intern_UserTCP(SOCKET_Intern_Dsc *used_tcp_socket); */
extern void calculateTickDiff( int32_t *TimeDuration, uint32_t *TimeDurationLast );
extern void calculateTick2Now( int32_t *TimeDuration, uint32_t *TimeDurationLast );
extern void calculateTime2Tick( uint32_t *TimeDuration, uint32_t *TimeDurationLast, uint32_t *TimeDurationCurrent );
extern void setBit(uint16_t* mask, uint16_t n);
extern void eraseBit(uint16_t* mask, uint16_t n);
extern uint16_t isSet(uint16_t mask, uint16_t n);
extern uint16_t onesCount(uint16_t mask);
extern uint16_t zerosCount(uint16_t mask);
extern uint8_t debounce_button( IO_btn_state_t *btn, uint8_t new_state, uint64_t TimeThreshold );
extern void calculateTimeDiff( int64_t *TimeDuration, uint64_t *TimeDurationLast );
extern void calculateTime2Now( int64_t *TimeDuration, uint64_t *TimeDurationLast );
extern void calculateHrs2Now( uint8_t *TimeDuration, uint8_t *TimeDurationLast );
extern void calculateMins2Now( uint8_t *TimeDuration, uint8_t *TimeDurationLast );
extern uint8_t mov_avg( mov_avg_data_t *accStruct );
extern int16_t setRandRange(int16_t min, int16_t max);
extern void bubble_sort_array(int16_t *temp_array3, int8_t order);
extern void quick_list(int16_t *temp_array3, int8_t order);
extern void swap_double(float64_t* a, float64_t* b);
extern void selectionSort_double(float64_t arr[], int16_t n);
extern void selectionSortAux_double(float64_t arr[], float64_t arr2[], int16_t n);
extern void diff_double(float64_t *y, float64_t *f, int16_t  sz);
extern int16_t isneg_double(float64_t *y, int16_t  sz);
#ifdef TIMER1_NEEDED                                                            // if you use Ethernet IP
extern void Init_Timer1();                                                      // Initialisation routine for Timer 1
#endif
#ifdef TIMER2_NEEDED
extern void Init_Timer2();                                                      // Initialisation routine for Timer 2
extern void Init_Timer2_3();                                                    // Initialisation routine for Timer 2_3
#endif
extern void Init_Timer5();                                                      // Initialisation routine for Timer 5
//extern void Init_PHYPins();                                                     // Initialise physical pins
extern void Init_UART( uint16_t parityValue );                                  // Initialise and set-up UART
extern void Init_MCU();                                                         // Initialise the MCU
extern void Init_Ether();                                                       // Initialise ethernet
extern void Status(int16_t address, int16_t state);                             // Report Status to uart5
extern uint16_t CRC16_Calc( const unsigned char *Packet, uint16_t Packet_length); // Do CRC16 UDP CRC calculation to check if packet okay
extern void Init_Node();                                                        // Initialise Nodes
extern void boot_arp();                                                         // Do Arp and wait for response
#if defined(WEATHER_STAT)
extern void RstDavisWeatherStation( uint8_t uartNo );                           // START communciation to the davis weather station
extern void RstKestrelWeatherStation( );                                        // START communciation to the kestrel weather station
extern int16_t getKestrelValues( char* msg, float32_t* values );                // get values as float from csv list from kestrel
extern void decodeKestrel( uint8_t *packet, WeatherStat_t *kesDat );            // decode values from kestrel and scale them
extern void decodeDavisNet( uint8_t *packet, WeatherStat_t *davDat, uint8_t portNo );  // decode values from davis and scale them
#endif
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
extern uint8_t SBGC_process_UDP_InBuffer( const SBGC_cmd_board_info_t *boardinf );    // Function to process the incoming UDP datagram from SBGC Gimbal
extern uint8_t ReadSBGC(const unsigned char *pdata, uint16_t psz, uint16_t arrPosition);     // Read the data portion of the UDP datagram and get SBGC data.
extern uint8_t check_payload(uint16_t buffStartPos,const SBGC_cmd_board_info_t *boardinf );
#endif
#if (CAMERA_TYPE == XS_CAM)
extern int8_t XS_process_UDP_InBuffer();                                        // Function to process the incoming UDP datagram from XSStream encoder
extern int8_t XS_process_SERIAL_InBuffer();                                     // Function to process the incoming UDP datagram from XSStream encoder
#endif                                                                          // ================== AMP XS Encoder / Decoder =====================
#ifdef RUN_CAM_USED
extern uint8_t ReadRunCam( const unsigned char *pdata, uint16_t psz );          // Function to process the incoming UDP datagram from RunCam Camera
#endif
#if defined(GEF_EGD_PLC)
extern uint8_t ReadEGD( const unsigned char *pdata, uint16_t psz );             // process the EGD data received
#endif
#if defined(ART_NET_USED)
extern uint8_t ReadArtNet( const unsigned char *pdata, uint16_t psz );          // process the ArtNet data received
#endif
uint8_t sizeToChar( const unsigned char* stringX, unsigned char val );          // the size of a string (arg.1) up to the specified char (argument No.2)
extern uint8_t checksum( const unsigned char *fdata, uint16_t sz );             // A Function to calculate a checksum for a struct
extern uint8_t DataChangeDetected( uint16_t dataVal, uint16_t dataLast, uint8_t delta);
extern float32_t Scale_Raw_Value( int16_t raw_min, int16_t raw_max, float32_t scale_min, float32_t scale_max, int16_t raw_value);
extern uint16_t Scale_Float_Value( float32_t raw_min, float32_t raw_max, uint16_t scale_min, uint16_t scale_max, float32_t float_value);
extern void InitEther();
extern void Test_CRC();
extern char ARP(unsigned char *NodeIPAddr);
extern unsigned char Pay_checksum(const unsigned char *pdata, uint16_t sz);
extern void StartTimeout();
extern uint8_t CheckTimout();
extern void StopTimer1();
extern void StartTCPCommunication();                                            // Initialise the TCP Stack
extern void do_rmse_retain_double( dVectr *a, dVectr *b, uint16_t num_points, dVectr *diff );
extern void do_rmse_iter_double( dVectr *a, dVectr *b, uint16_t num_points, dVectr *diff );
extern void set_rmse_iter_double( dVectr *diff );
extern dVectr* do_rmse_ap_double( dVectr *a, uint16_t num_points );
extern dVectr* do_rmse_vp_double( dVectr *a, uint16_t num_points );
extern dVectr* do_rmse_double( dVectr *a, dVectr *b, uint16_t num_points );
extern float64_t boxbetts_f(const float64_t *a, float64_t *grad);
extern void humdev(const float64_t x, const float64_t y, float64_t *k, float64_t *l, float64_t *dkdx, float64_t *dkdy);
extern void shirley_bg( dVectr **pp, int16_t ppSize );
extern float64_t Guess_find_hwhm(int16_t pos, float64_t* area, float64_t *yy_, float64_t *xx_, int16_t yySiz );
extern dVectr Guess_estimate_peak_parameters( float64_t *yy_, size_t yySiz, float64_t *sigma_, float64_t *xx_, float64_t height_correction, float64_t width_correction );
extern dVectr Guess_estimate_linear_parameters( float64_t *xx_, float64_t *yy_, size_t yySiz );
extern dquat Guess_estimate_sigmoid_parameters( float64_t *xx_, float64_t *yy_, size_t yySize, float64_t upper, float64_t lower );
extern bool is_left_of_line( const dVectr p0, const dVectr p1, const dVectr p2 );
extern int16_t moser_de_bruijn_gen(const int16_t n, int16_t *S);                /* moser de-bruijn seuence for creating a unique encoded sending sequence */
extern int8_t doMoserDeBruijn(const int16_t n, int16_t *sequence);
extern int8_t doBellmanFord(BF_Graph_t* graph, const int16_t src);              /* bellman ford - alternative dijsktra */
extern void doFloydWarshall(const int16_t graph[FW_V][FW_V], int16_t **dist);   /* ffloyd warshall */
extern int8_t SieveOfEratosthenes(const int32_t n, int32_t *result);            /* list prime numbers */
extern int16_t newman_conway_gen(const int16_t n, int16_t *f);
extern int8_t doNewmanConway(const int16_t n, int16_t *sequence);
extern int16_t collatz_gen(const int16_t n);
extern int8_t doCollatz(const int16_t n, int16_t *sequence);
extern void doFareySequence(const int16_t n, float64_t *sequence);
extern uint16_t randhash(uint8_t *init, uint16_t seed);                         /* generate random encoded seeds */
extern uint16_t unrandhash(uint16_t h);
extern float64_t randhashdouble(uint16_t seed, uint8_t *init, float64_t a, float64_t b);
extern int16_t intlog2(int16_t x);
extern void get_barycentric(float64_t x, int16_t *i, float64_t *f, int16_t i_low, int16_t i_high);
extern float64_t lerp(const float64_t value0, const float64_t value1, float64_t f);
extern float64_t bilerp(const float64_t v00, const float64_t v10, const float64_t v01, const float64_t v11, const float64_t fx, const float64_t fy);
extern float64_t trilerp(const float64_t v000, const float64_t v100, const float64_t v010, const float64_t v110, const float64_t v001, const float64_t v101, const float64_t v011, const float64_t v111, const float64_t fx, const float64_t fy, const float64_t fz);
extern float64_t quadlerp(const float64_t v0000, const float64_t v1000, const float64_t v0100, const float64_t v1100, const float64_t v0010, const float64_t v1010, const float64_t v0110, const float64_t v1110, const float64_t v0001, const float64_t v1001, const float64_t v0101, const float64_t v1101, const float64_t v0011, const float64_t v1011, const float64_t v0111, const float64_t v1111, float64_t fx, float64_t fy, float64_t fz, float64_t ft);
extern void quadratic_bspline_weights(const float64_t f, float64_t *w0, float64_t *w1, float64_t *w2);
extern void cubic_interp_weights(const float64_t f, float64_t *wneg1, float64_t *w0, float64_t *w1, float64_t *w2);
extern float64_t cubic_interp(const float64_t value_neg1, const float64_t value0, const float64_t value1, const float64_t value2, float64_t f);
extern void zero(float64_t *v);
extern float64_t abs_max(const float64_t *v);
extern float64_t bezlerp(const float64_t a,const float64_t b,const float64_t c);
extern float64_t quadBezier(const float64_t t,const float64_t p0,const float64_t p1,const float64_t p2);
extern float64_t cubicBezier( const float64_t t,const float64_t p0,const float64_t p1,const float64_t p2,const float64_t p3);
extern float32_t cross2d( Vectr a, Vectr b );
extern float32_t dot2d( Vectr a, Vectr b );
extern Vectr invBilinear( Vectr p, Vectr a, Vectr b, Vectr c, Vectr d );
extern float32_t sdSegment( Vectr p, Vectr a, Vectr b );
extern float32_t vlength( Vectr v );
extern Vectr vmul( Vectr a, float32_t x );
extern void CERES_AngleAxisRotatePoint(float64_t *angle_axis, float64_t *pt, float64_t *result);
extern bool SnavelyReprojectionError(float64_t *camera, float64_t *point, float64_t *residuals, float64_t observed_x, float64_t observed_y );
extern int8_t CERES_QuaternionProduct(const float64_t *z, const float64_t *w, float64_t *zw);
extern int8_t CERES_AngleAxisToQuaternion(const float64_t* angle_axis, float64_t* quaternion);
extern int8_t CERES_QuaternionToAngleAxis(const float64_t* quaternion, float64_t* angle_axis);
extern int8_t CERES_AngleAxisToRotationMatrix( const float64_t* angle_axis, float64_t** R );
extern int8_t CERES_UnitQuaternionRotatePoint(const float64_t *q, const float64_t *pt, float64_t *result);
extern int8_t CERES_QuaternionRotatePoint(const float64_t *q, const float64_t *pt, float64_t *result);
extern int8_t CERES_QuaternionToScaledRotation( const float64_t *q, float64_t **R );
extern int8_t CERES_QuaternionToRotation( const float64_t *q, float64_t **R );
extern int8_t CERES_EulerAnglesToRotationMatrix(  const float64_t* euler, float64_t **R );
extern int8_t CERES_RotationMatrixToQuaternion(  const float64_t **R, float64_t *quaternion );
extern void CERES_RotationMatrixToAngleAxis( const float64_t **R, float64_t *angle_axis );
extern void KnuthRng_seed(uint32_t *s0, uint32_t *s1, uint32_t *state0, uint32_t *state1);
extern uint32_t KnuthRng_get( uint32_t *state0, uint32_t *state1 );
/* ------ matrix rotation maths ------------------------------------------- */
extern bool Vectr_to_doubleStream(const Vectr *vec, float64_t *strm);
extern bool Vectr_from_doubleStream(Vectr *vec, const float64_t *strm);
extern bool doubleStream_from_matrix33(const mat33 *mtx, float64_t *strm);
extern bool matrix33_from_doubleStream(mat33 *mtx, const float64_t *strm);
extern void SVD_jacobi(float64_t* a, int16_t n, float64_t *s, float64_t *u, float64_t *v);
extern void SVD_singular_value_e(float64_t * a, int16_t n, singular_value_t* temp);
extern void bubble_sort_singular_value(singular_value_t* temp_array3, bubble_sort_action_e order);
extern void SVD_rotation(float64_t *a, int16_t n, int16_t p, int16_t q, float64_t* u);
extern void SVD_arrange(float64_t* a, int16_t n);
extern float64_t SVD_sign(float64_t val);
extern float64_t SVD_g_eigen(float64_t* a, int16_t n, int16_t row, int16_t col);
extern mat33 moore_penrose( const mat33 V, const mat33 U, Vectr W );
extern mat33 math_arun( const mat33 V, const mat33 U );
extern Matrix3f64_t cayley2rotF64( const dVectr cayley );
extern mat33 cayley2rotM33( const Vectr cayley );
extern Matrix3f64_t cayley2rot_reducedF64( const dVectr cayley );
extern mat33 cayley2rot_reducedM33( const Vectr cayley);
extern float32_t** cayley2rot_reducedF32( const float32_t *cayley );
extern dVectr math_rot2cayleyF64( const Matrix3f64_t R );
extern int8_t math_rot2cayleyM33( const mat33 *R, Vectr *cayley );
extern Matrix3f64_t Matrix3_multiply( const Matrix3f64_t m0, const Matrix3f64_t m1  );
extern bool matrix33_multiply( mat33 *a, const mat33 m0, const mat33 m1  );
extern Matrix3f64_t Matrix3f64Scl(float64_t s, const Matrix3f64_t a);
extern float32_t** mArrayScl(float64_t s, float32_t a[3][3]);
extern Matrix3f64_t Matrix3_inverse(const Matrix3f64_t a, const float64_t d);
extern Matrix3f64_t Matrix3_add( const Matrix3f64_t a, const Matrix3f64_t b );
extern bool matrix33_add(mat33 *mtx, mat33 *a, mat33 *b);
extern Matrix3f64_t Matrix3_subtract( const Matrix3f64_t a, const Matrix3f64_t b );
extern bool matrix33_subtract(mat33 *mtx, mat33 *a, mat33 *b);
extern Matrix3f64_t Matrix3_setIdentity();
extern bool matrix33_identity(mat33 *mtx);
extern bool Matrix3f64_Identity(Matrix3f64_t *a);
extern mat33 do_moore_penrose_inv( const mat33 V, const mat33 U, Vectr W );
#if defined(FFT_NOTCH_REQ)
extern float64_t quinn_tau(float64_t x);
extern void do_quinn_est( quinn_est_t *quin, float64_t sampleRate, float64_t peakPosIndex, float64_t N, float64_t **out );
extern float64_t parzen(int16_t i, int16_t nn);
extern float64_t welch(int16_t i, int16_t nn);
extern float64_t hanning(int16_t i, int16_t nn);
extern float64_t hamming(int16_t i, int16_t nn);
extern float64_t blackman(int16_t i, int16_t nn);
extern float64_t steeper(int16_t i, int16_t nn);
extern float64_t planck_taper(int16_t i, int16_t nn);
extern float64_t gaussian(int16_t i, int16_t nn);
extern float64_t tukey(int16_t i, int16_t nn);
extern float64_t flattop(int16_t i, int16_t nn);
extern float64_t blackman_harris(int16_t i, int16_t nn);
extern float64_t blackman_nutall(int16_t i, int16_t nn);
extern float64_t nutall(int16_t i, int16_t nn);
extern int8_t power_subtract_octave(int16_t n, float64_t *p, float64_t factor);
extern int8_t power_subtract_ave(int16_t n, float64_t *p, int16_t m, float64_t factor);
extern int8_t HC_complex_phase_vocoder(int16_t len, const float64_t *fs, const float64_t *ft, const float64_t *f_out_old, float64_t *f_out);
extern void HC_puckette_lock(int64_t len, const float64_t *y, float64_t *z);
extern void HC_abs(int64_t len, const float64_t *x, float64_t *z);
extern void HC_div(int64_t len, const float64_t *x, const float64_t *y, float64_t *z);
extern void HC_mul(int64_t len, const float64_t *x, const float64_t *y, float64_t *z);
extern void polar_to_HC_scale(int64_t len, const float64_t *amp, const float64_t *phs, int16_t conj, int16_t scale, float64_t *freq);
extern void polar_to_HC(int64_t len, const float64_t *amp, const float64_t *phs, int16_t conj, float64_t *freq);
extern void HC_to_amp2(int64_t len, const float64_t *freq, float64_t scale, float64_t *amp2);
extern void HC_to_polar2(int64_t len, const float64_t *freq, int16_t conj, float64_t scale,  float64_t *amp2, float64_t *phs);
extern void HC_to_polar(int64_t len, const float64_t *freq, int16_t conj, float64_t *amp, float64_t *phs);
extern int8_t init_den(int16_t n, char flag_window, float64_t *den);
extern int8_t windowing(int16_t n, const float64_t *dataV, int16_t flag_window, float64_t scale, float64_t *out);
extern void apply_FFT(int16_t len, const float64_t *dataV, int16_t flag_window, float64_t *in, float64_t *out, float64_t scale, float64_t *amp, float64_t *phs);
#endif
#if defined(YI_CAM_USED)
extern uint8_t XY_OpenTCPSocketRawJson( SOCKET_Intern_Dsc **sock1 );            // Opens RAW tcp socket for JSON communication for Yi Action Cam
extern uint8_t XY_CloseTCPSocketRawJson( SOCKET_Intern_Dsc *sock1 );            // Closes RAW tcp socket for JSON communication for Yi Action Cam
extern void TCPStateCheck( SOCKET_Intern_Dsc *used_tcp_socket);                 // Check the State of the YiCam socket
extern void resetXYActions();
#endif
#if defined(SEQ_CAM_USED)
extern uint8_t SEQ_OpenTCPSocketHttpJson( SOCKET_Intern_Dsc **sock1 );          // Opens http api tcp socket for JSON communication for Sequoia Cam
extern uint8_t SEQ_CloseTCPSocketHttpJson( SOCKET_Intern_Dsc *sock1 );          // Closes http api tcp socket for JSON communication for Sequoia Cam
#endif
#if defined(HTTP_USED)
extern uint8_t airMapaddPress(float32_t *press,  AirMap_Barometer_t *amBaro, rionUTCDateTimeField_t *syncedTime );
extern uint8_t UbiTCP_checkIpAddress( char *ipAddress );
extern float32_t checkHttpRespUbiDots( HttpConnect_t *conn, HttpClientContext_t *context );
extern uint8_t UbiDotaddContext(char *key_label, char *key_value, UbiProtocolHandler_t *proto, ContextUbi_t *_context);
extern uint8_t UbiDotaddDot(char *key_label, float32_t *dot_value, UbiProtocolHandler_t *proto, ubiValue_t *_dots, rionUTCDateTimeField_t *syncedTime );
extern void UbiDot_floatToChar(char *str_value, float32_t value);
extern void UbiDotbuildAnyPayload(char *payload, UbiProtocolHandler_t *ubiDots, ubiValue_t *_dots);
extern int16_t http_read_cb( HttpConnect_t* conn, int revents, SOCKET_Intern_Dsc *sock1  );   // http chunk reader
extern error_t httpClientShutdownConnection(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
extern error_t httpClientReceiveData(HttpClientContext_t *context, void *dataV, size_t size, size_t *received, uint8_t flags);
extern void httpClientChangeState(HttpClientContext_t *context, HttpClientState newState);
extern error_t httpClientDisconnect(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
extern error_t httpClientCreateRequest(HttpClientContext_t *context);
extern error_t httpClientSetMethod(HttpClientContext_t *context, const char_t *method);
extern error_t httpClientSetUri(HttpClientContext_t *context, const char_t *uri);
extern error_t httpClientAddQueryParam(HttpClientContext_t *context, const char_t *name, const char_t *value);
extern error_t httpClientAddHeaderField(HttpClientContext_t *context, const char_t *name, const char_t *value);
extern error_t httpClientWriteHeader(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
extern error_t httpClientWriteBody(HttpClientContext_t *context, const void *dataV, size_t length, size_t *written, uint8_t flags, SOCKET_Intern_Dsc *socket);
extern error_t httpClientReadHeader(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
extern uint16_t httpClientGetStatus(HttpClientContext_t *context);
extern const char_t *httpClientGetHeaderField(HttpClientContext_t *context, const char_t *name);
extern error_t httpClientReadBody(HttpClientContext_t *context, void *dataV, size_t size, size_t *received, uint8_t flags);
extern error_t httpClientReadTrailer(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
extern error_t httpClientCloseBody(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
extern error_t httpClientSetVersion(HttpClientContext_t *context, HttpVersion version);
extern error_t httpCheckCharset(const char_t *s, size_t length, uint8_t charset);
extern void httpClientChangeRequestState(HttpClientContext_t *context, HttpRequestState newState);
extern error_t httpClientParseHeaderField(HttpClientContext_t *context, char_t *line, size_t length);
extern error_t httpClientParseConnectionField(HttpClientContext_t *context, const char_t *value);
extern int16_t isprintable(char c);
extern error_t httpClientParseTransferEncodingField(HttpClientContext_t *context, const char_t *value);
extern error_t httpClientParseContentLengthField(HttpClientContext_t *context, const char_t *value);
extern error_t httpClientParseStatusLine(HttpClientContext_t *context, char_t *line, size_t length);
extern error_t httpClientCheckTimeout(HttpClientContext_t *context);
extern error_t httpClientFormatRequestHeader(HttpClientContext_t *context);
extern error_t httpClientFormatChunkSize(HttpClientContext_t *context, size_t length);
extern error_t httpClientParseChunkSize(HttpClientContext_t *context, char_t *line, size_t length);
extern error_t httpClientWriteTrailer(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
extern error_t httpClientSendData(HttpClientContext_t *context, const void *dataV, size_t length, size_t *written, uint8_t flags, SOCKET_Intern_Dsc *socket);
extern error_t httpClientConnect(HttpClientContext_t *context, uint8_t *serverIpAddr, uint16_t serverPort);
extern void httpClientCloseConnection(HttpClientContext_t *context, SOCKET_Intern_Dsc *socket);
extern error_t httpClientFormatAuthorizationField(HttpClientContext_t *context);
extern void httpClientInitAuthParams(HttpClientAuthParams_t *authParams);
#endif
#if defined(JRT_LIDDAR_USED)
extern void initUSB( USBObject_t *USBObj );
extern void ReadUSB_JRT( JRT_read_measure_t *distMeas, USBObject_t *USBObj, JRT_write_measure_t *deviceState );
#endif
#ifdef DO_COMPIC
extern char do_UART_to_UDP();
#endif
#if (!defined(SERIAL_MAVLINK) && defined(USE_MAVLINK))
extern void mavInitUDP();
extern void chooseMavlinkSink( mavSinkClass mavSink );
#endif
#if defined(NAVSTIK_USED)
extern float64_t NavStik_getRawValues( char* inpStr, navstickWordOffset_e offset  );
extern void NavStik_getAllValues( char* inpStr, navstik_object_t *nav, navstickUseCalib_e *mode  );
#endif
#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
extern ADCStates_e readManualADC( uint8_t pinNo, uint16_t *rawValADC );
#endif
#if defined(BATCH_MIXER)
#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
extern void makeUpDoseTank( uint16_t *valADC, batchSeq1Recipe_t *rcpDat, mov_avg_data_t *movAvg   );
#else
extern void makeUpDoseTank( batchSeq1Recipe_t *rcpDat, mov_avg_data_t *movAvg  );
#endif
#endif /* end dose tank */
extern void updateRTOfromRTC( );
extern void initRTOfromNTP( );
extern void checkValve( device_t *dev );
extern void checkMotor( device_t *dev );
extern void checkPulseIntegrity( rotat_sens_obj_t *risingEdge1Obj, rotat_sens_obj_t *fallingEdge1Obj, rotat_sens_obj_t *risingEdge2Obj, rotat_sens_obj_t *fallingEdge2Obj, pulse_integrity_t *puls );
extern unsigned char *strTrimWhitespace(unsigned char *s);
extern void strRemoveTrailingSpace(unsigned char *s);
extern int32_t strtol(const char *nptr, char **endptr, register int16_t base);
extern uint32_t strtoul(const char *nptr, char **endptr, register int16_t base);
extern void moveVehicle( ocr_movement_obj_t *move, uint8_t *output );

// External Calls protocol
extern uint16_t XtCalls_checksum1(const unsigned char *pdata, uint16_t sz);
// convert 00:00:00 recording time timespec to mavlink
extern void getTimeFromString( mavDelay_t *clock, unsigned char *inputString );
// from sexagesimal
extern float64_t valueFromSexagesimal( char *inputString, char *delim );

// dsHot600 motor
#if defined(DSHOT_MOTOR)
extern void writedShotOut(uint8_t index, float32_t value, dShot_motor_t *motor );
extern void dShotOutputOne(dShot_motor_t *motor);
#endif
/* =============== Servos ================================================== */
#if defined(ROBOTIS_PROTOCOL)
extern void RobotisServo_add_stuffing(uint8_t *packet);
extern void Robotis_send_packet(uint8_t *txpacket);
extern void Robotis_send_command(uint8_t id, uint16_t reg, uint32_t value, uint8_t len);
#endif
#if defined(VOLZ_PROTOCOL)
extern void Volz_send_value( VolZ_t *vObj );
#endif
#if defined(TRAMP_PROTOCOL)
extern void trampFrameGetSettings(trampFrame_t *frame);
extern void trampFrameSetFrequency(trampFrame_t *frame, const uint16_t frequency);
extern void trampFrameSetPower(trampFrame_t *frame, const uint16_t power);
extern void trampFrameSetActiveState(trampFrame_t *frame, const uint8_t active);
extern uint8_t trampParseResponseBuffer(trampSettings_t *settings, const uint8_t *buffer, size_t bufferLen);
#endif
#if defined(KIN_SCUL)
extern void setKineticSculpture( int8_t servoNo, float32_t float_value, uint8_t uartPort );
#endif
#if defined(VIDEORAY_M5_USED)
extern uint32_t vm5_crc_slow(uint8_t const * const p_message, uint8_t n_bytes);
extern void vm5_static_thrust_msg(vm5_object_t *vm5);
#endif
#if defined(AVNav_USED)
extern int16_t SerialMbed_writeCommand(char prefix, int16_t num, int8_t uartNo);
#endif
#if defined(TILT_SERVO1_USED)
extern void servo1_write_uart(int16_t comport, uint8_t *buf, uint8_t len  );
extern void pop_queue( servo1_t *servo );
extern void push_queue( servo1_t *servo, servo1_request_e request );
extern void servo1_move(int16_t comport, int16_t ID, int16_t angle, int16_t rpm, servo1_t *servo);
extern void servo1_read_angle(int16_t comport, int16_t ID, servo1_t *servo);
extern void servo1_read_speed(int16_t comport, int16_t ID, servo1_t *servo);
extern void servo1_read_ret_dly_time(int16_t comport, int16_t ID, servo1_t *servo);
extern void servo1_read_voltage(int16_t comport, int16_t ID, servo1_t *servo);
extern void servo1_read_temp(int16_t comport, int16_t ID, servo1_t *servo); 
#endif
/* Air Core Inductor Calculator */
extern float64_t Induc_f1( float64_t x );
extern float64_t Induc_f2( float64_t x );
extern float64_t InductanceCalc( float64_t a, float64_t b, float64_t n );
extern float64_t InductorCoilQ( float64_t a, float64_t b, float64_t f );

/* ========== motion capture (noise algos) ================================= */
#if defined(TF_DISTORT)
extern void whiteGaussInitSeed( int16_t *seedIterator );
extern void whiteGaussianNoise(float64_t * n1, float64_t * n2);
extern float32_t generateGaussianNoise(float32_t mu, float32_t variance);
extern float64_t uniformNoise( const float64_t mag );
extern Vicon_update( const float64_t dt, float64_t tau_ );
#endif

#if defined(NAVILOCK_USED)
extern void navilockInterruptStateManager( navilock_t *nav );
#endif

/* ========== vision tool kit ======= */
extern float64_t ran2(int16_t *idum);
extern int16_t getIdum(const bool useRandom);
extern int16_t setIdumRand(const bool useRandom);
extern float64_t gasdev (int16_t *idum);
extern float64_t expdev(int16_t *idum);
extern float64_t iNvLab_angle( Vectr* pt1, Vectr* pt2, Vectr* pt0 );
extern float64_t lngamma(float64_t x);
extern float64_t poisson(const int16_t k, const float64_t mu);
extern float64_t AUC(const float64_t* model, const float64_t* rand, size_t sm, size_t sr, const float64_t step);
extern float32_t * inplaceAddBGnoise2(float32_t *src, const float32_t range, int16_t w, int16_t h);
extern float64_t uniformRandom(void);
extern float64_t gaussianRand(void);
extern float64_t ParticleFilter_getLikelihood(const float64_t z, const float64_t X);
extern float64_t AngleCosineFrom2DVectr(const Vectr p0, const Vectr p1, const Vectr p2);
extern float64_t Value_saturate(float64_t A, const float64_t n);
/* quaternion */
extern Vectr mkvec(float32_t x, float32_t y, float32_t z);
extern Vectr vadd(Vectr a, Vectr b); 
extern Vectr vcross(Vectr a, Vectr b);
extern Vectr vneg(Vectr v);
extern Vectr vsub(Vectr a, Vectr b);
extern void fqAeqNormqA(quat *pqA);
extern void qAeqBxC(quat *pqA, const quat *pqB, const quat *pqC);
extern void fqAeq1(quat *pqA);
extern quat qconjgAxB(const quat *pqA, const quat *pqB);
extern float32_t matrix33_dot( mat33 a );
extern bool matrix33_inverse(mat33 *inv, const mat33 a);
extern float64_t wrap_PI( float64_t AngleX ); 
extern mat33 mscl(float64_t s, mat33 a);
extern bool matrix44_inverse(mat44 *inv, const mat44 a);
extern mat44 m44scl(float64_t s, mat44 a);
extern mat33 Matrix33_from_euler(float32_t roll, float32_t pitch, float32_t yaw);
extern void matrix33_to_eulerAngle( const mat33 mat, Vectr *theta );
extern float64_t Matrix4_det( const Matrix4d64_t a );
extern bool inverseMatrix4x4(const float64_t *m, float64_t *out);
extern float64_t invf(int16_t *i, int16_t *j, const float64_t* m);
extern void Line_setQuantizedDir(int16_t numDirections, const xyline_t p1,const xyline_t p2, int16_t *itsDirectionIdx);
extern bool Line_quantize(int16_t numDirections, xyline_t * p1, xyline_t * p2, int16_t *itsDirectionIdx );
extern float64_t Line_getLength(const xyline_t p1, const xyline_t p2);
extern float64_t Line_getOri(const xyline_t p1, const xyline_t p2);
extern xyline_t Line_getCenter(const xyline_t p1, const xyline_t p2 );
extern bool Line_trans(xyline_t * p, xyline_t * p1, xyline_t * p2);
extern bool Line_shear(float64_t k1, float64_t k2, xyline_t *p1, xyline_t *p2);
extern bool Line_rotate(float64_t theta, xyline_t *p1, xyline_t *p2);
extern float64_t* Matrix99_mtranspose(float64_t m[9][9]);
extern dVectr* Vector3_rotate(uint8_t rotation, dVectr *Vect);
extern bool Quaternion_to_axis_angle(dVectr *v,const dquat q);
extern bool Quaternion_earth_to_body(dVectr *v, const dquat q);
extern dVectr matf64_dVec_mul(Matrix3f64_t a, dVectr v);
extern Matrix3f64_t Matrix3_from_euler(float64_t roll, float64_t pitch, float64_t yaw);
extern void Matrix3_to_euler(const Matrix3f64_t m, float64_t *roll, float64_t *pitch, float64_t *yaw);
extern dquat Quaternion_from_rotation(uint8_t rotation);
extern dVectr Quaternion_to_vector312(const dquat q);
extern Matrix3f64_t Matrix3_from_euler312(float64_t roll, float64_t pitch, float64_t yaw);
extern dVectr Vector3_to_euler312( const Matrix3f64_t m );
extern bool Quaternion_to_euler(float64_t *roll, float64_t *pitch, float64_t *yaw, const dquat q);
extern bool Quaternion_rotate(const dVectr *v, dquat *q);
extern dquat Quaternion_div(const dquat q, const dquat v);
extern dquat Quaternion_mul(const dquat v,const dquat q);
extern float64_t Quaternion_get_euler_yaw(const dquat q);
extern float64_t Quaternion_get_euler_pitch(const dquat q);
extern float64_t Quaternion_get_euler_roll(const dquat q);
extern bool Quaternion_from_axis_angle3(dVectr *v, dquat *q);
extern bool Quaternion_from_axis_angle2(dVectr *v, dquat *q, const float64_t theta);
extern dVectr dvecDiv( const dVectr q, float64_t div );
extern dquat dquatDiv( const dquat q, float64_t div );
extern bool Quaternion_from_axis_angle(const dVectr axis, float64_t theta, dquat *q);
extern bool Quaternion_from_euler(float64_t roll, float64_t pitch, float64_t yaw, dquat *q);
extern bool Quaternion_rotation_matrix(Matrix3f64_t *m, const dquat q);
extern bool Quaternion_rotation_matrix_norm(Matrix3f64_t *m, const dquat q);
extern bool Quaternion_from_rotation_matrix2(const Matrix3f64_t m, quat *q);
extern quat Quaternion_from_rotation_matrix(const Matrix3f64_t m);
extern float64_t Vectr_length(const dVectr q);
extern float64_t Quaternion_length(const dquat q);
extern dquat Quaternion_inverse(const dquat q);
extern bool matrix33_to_Matrix3f64(const mat33 mtx, Matrix3f64_t *a);
extern bool Matrix3f64_to_matrix33(mat33 *mtx, const Matrix3f64_t a);
extern float64_t Matrix3_det( const Matrix3f64_t a );
extern float64_t * mNsub(float64_t s, float64_t * a, float64_t * res, uint16_t nEle);
extern float64_t * mNadd(float64_t s, float64_t * a, float64_t * res, uint16_t nEle);
extern float64_t * mNdiv(float64_t s, float64_t * a, float64_t * res, uint16_t nEle);
extern float64_t * mNscl(float64_t s, float64_t * a, float64_t * res, uint16_t nEle);
extern Matrix4d64_t Matrix4_setIdentity();
extern void getRowMat4f64(uint16_t row, dquat *v, Matrix4d64_t *m00);
extern void getColumnMat4f64(uint16_t column, dquat *v, Matrix4d64_t *m00);
extern void setColumnMat4f64(uint16_t column, const quat v, Matrix4d64_t *m00);
extern void setRowMat4f64(uint16_t row, const dquat v, Matrix4d64_t *m00);
extern void Matrix4_transpose( Matrix4d64_t *mat );
extern void set_mat44( const dquat v0, const dquat v1, const dquat v2, const dquat v3, mat44 *a );
extern void set_Matrix4( const dquat v0, const dquat v1, const dquat v2, const dquat v3, Matrix4d64_t *a );
extern void mul_mat44( const mat44 m0, const mat44 m1, mat44 *a );
extern void mul_Matrix4( const Matrix4d64_t m0, const Matrix4d64_t m1, Matrix4d64_t *out );
extern void setRotationZ_Matrix4(float64_t angle, Matrix4d64_t *out);
extern void setRotationY_Matrix4(float64_t angle, Matrix4d64_t *out);
extern void setRotationX_Matrix4(float64_t angle, Matrix4d64_t *out);
extern void setRotationX_mat44(float64_t angle, mat44 *a);
extern void setRotationY_mat44(float64_t angle, mat44 *a);
extern void setRotationZ_mat44(float64_t angle, mat44 *a);
extern void invertR_Matrix4(const Matrix4d64_t m, Matrix4d64_t *mat);
extern void invertR_mat44(const mat44 m, mat44 *mat);
extern void transform_Matrix4(const dquat in, const mat44 m, dquat *out  );
extern dquat transform_mat44(const dquat in, const mat44 m );
extern dquat mkdquat(float64_t x, float64_t y, float64_t z, float64_t w);
extern void setTranslation_mat16(const dVectr v, float64_t* matrix);
extern mat44 setTranslation_mat44(const dVectr v);
extern Matrix4d64_t setTranslation_Matrix4d64(const dVectr v);
extern void setRotationX_mat16(float64_t angle, float64_t *matrix);
extern Matrix4d64_t setRotationX_Matrix4d64(float64_t angle);
extern mat44 setRotationX_mat44_2(float64_t angle);
extern void setRotationY_mat16(float64_t angle, float64_t *matrix);
extern Matrix4d64_t setRotationY_Matrix4d64(float64_t angle);
extern mat44 setRotationY_mat44_2(float64_t angle);
extern void setRotationZ_matrix16(float64_t angle, float64_t *matrix);
extern Matrix4d64_t setRotationZ_Matrix4d64(float64_t angle);
extern mat44 setRotationZ_mat44_2(float64_t angle);
extern void setScale_mat16(const dVectr vec, float64_t *matrix);
extern mat44 setScale_mat44(const dVectr vec);
extern Matrix4d64_t setScale_Matrix4d64(const dVectr vec);
extern dquat dquatFrom_matrix16(const dVectr v, const float64_t *matrix);
extern dquat dquatFrom_mat44(const dVectr v, const mat44 matrix);
extern dquat dquatFrom_Matrix4d64(const dVectr v, const Matrix4d64_t matrix);
extern bool isAffine_matrix16(float64_t *matrix);
extern float64_t getNormOfStream( const float64_t* v, int16_t n );
extern uint8_t Polygon_outside(const u32point_t P, const u32point_t **V, uint16_t n);
extern uint32_t Polygon_complete(const u32point_t **V, uint32_t n);
extern float32_t AVF_gcDistance(float32_t ilat1,float32_t ilon1,float32_t ilat2,float32_t ilon2);
extern float32_t AVF_gcDistanceNm(float32_t lat1,float32_t lon1,float32_t lat2,float32_t lon2);
extern GPS_Position_t* AVF_gcIntermediatePoint(float32_t dlat1, float32_t dlon1, float32_t dlat2, float32_t dlon2, float32_t *f);
extern float32_t AVF_calcBearing(float32_t ilat1,float32_t ilon1,float32_t ilat2,float32_t ilon2);
/* =============== machine learning ======================================= */
extern void ML_softmax(const float64_t* z[], float64_t *out[], float64_t *foo[], uint16_t zsize, const int16_t dim);
extern void ML_transpose(float64_t *m[], const int16_t C, const int16_t R, float64_t *mT[]);
extern void ML_dot(const float64_t* m1[], const float64_t* m2[], const int16_t m1_rows, const int16_t m1_columns, const int16_t m2_columns, float64_t *output[]);
extern float64_t ML_max(float64_t *z[], uint16_t datSize);
extern void ML_relu(const float64_t *z[], float64_t *o[], uint16_t datSize, float64_t value, uint8_t prime);
extern void ML_sigmoid(float64_t *in, uint16_t datSize, typeOfSigmoid_e sigAlgo, float64_t *out);
extern void linearTransFormation_Matrix4(quat v, Matrix4d64_t m, quat *mv);
extern quat linearTransFormation2_Matrix4(quat v, Matrix4d64_t m);
extern float64_t polyApprox_sigmoid_g3( float64_t x );
extern float64_t polyApprox_sigmoid_g5( float64_t x );
extern float64_t polyApprox_sigmoid_g7( float64_t x );
extern mat33 matrix9x9_ctAB( const mat33 ctA, mat33 *ctB, uint8_t k);
extern mat33 matrix33_transpose( const mat33 ct );
extern void matrix33_transpose2( const mat33 ct, mat33 *ct_T );
extern float64_t horners_rule(float64_t *coeffs, float64_t x);
extern float64_t horner_equation( float64_t x );
/* qauternion maths */
extern void quatMultiply(float64_t *qr, float64_t *q1, float64_t *q2);
extern void eulerToQuatYPR(float64_t *q, float64_t yaw, float64_t pitch, float64_t roll);
extern void eulerToQuatRPY(float64_t *q, float64_t roll, float64_t pitch, float64_t yaw);
extern void vectorNormalize(float64_t *v, int16_t n);
extern void nlerp(float64_t *r, const float64_t *a, const float64_t *b, const float64_t t);
#if defined(ALLAN_VAR_FUNCS)
#define DIMENSION_3D 3u                                                         /* dimension of the output result Vector array */
extern uint16_t returnAllenVarLen( const uint16_t T );
extern void avar_mo_cpp(const Vectr **Accel,const Vectr **Gyro, const uint16_t T, const uint16_t AllanVarlen, Vectr** out[DIMENSION_3D]);
extern uint16_t avar_getMaxyBarSize( const uint16_t AllanVarLen, const uint16_t T );
extern void avar_to_cpp(const Vectr **Accel, const Vectr **Gyro, const uint16_t T, const uint16_t AllanVarlen, const uint16_t maxBarLen, Vectr **out[DIMENSION_3D]); 
#endif

// ============= External variables used =======================================
#if (AIN_ADC_METHOD == ADC_MAN_COLLECT)
extern volatile ADCStates_e g_adcState;                                         /* global state for the ADC  */
#endif

#ifdef USE_CAN
extern void Init_Can();                                                         /* Initialise CAN Bus ? */
extern void do_CAN_to_UDP();
extern void Can2_Test();
extern void bitWrite(char *x, char n, char value);
extern char bitRead(char *x, char n);
extern uint16_t Can1_RX_MSG_ID;
extern uint16_t Can1_TX_MSG_ID;
extern uint16_t Can2_RX_MSG_ID;
extern uint16_t Can2_TX_MSG_ID;
extern uint16_t Can1_Rcv_Flags;
extern uint16_t Can2_Rcv_Flags;
#endif

extern uint8_t EtherConnState;
extern uint8_t Ether_Link_Present;
extern uint16_t UDP_CRC_Reg;
extern uint16_t UDP_CRC_Good_Reg;
extern char Node;
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
extern uint8_t UART2_Test_Mode;
#endif
//extern unsigned char Pay_checksum();

// =============== Communication Objects =======================================
#ifdef UART1_INTERUPT                                                           // serial UART1 (for test purposes)
extern SerialObject_t UART1;                                                    // Define UART1 as UART object.
#endif
#ifdef UART2_INTERUPT
extern SerialSBGCObject_t UART2;                                                // External declarations for Serial Communication direct to gimbal simpleBGC
#endif
#ifdef UART4_INTERUPT
extern SerialObject_t UART4;                                                    // External declarations for Serial Communication direct to AMP Camera Encoder
#endif
#ifdef UART5_INTERUPT                                                           // read the Run Cam camera on serial UART5 (for test purposes)
extern SerialObject_t UART5;                                                    // Define UART5 as UART object.
#endif
#ifdef UART6_INTERUPT
extern SerialLwNxObject_t UART6;                                                // External declarations for Serial Communication for liddar lwnx protocol
//extern lwResponsePacket LwNxResponse;                                           // LightWare liddar packet and status
#endif
#ifdef UBIBOT_DEVICE_USED
extern SerialObject_t UbiSerial;                                                // Define UbiSerial as UART object.
#endif
#ifdef SONY_VISCA_PROTO_USED
extern SerialObject_t viscaSerial;                                              // Define viscaSerial as UART object.
#endif
#ifdef JRT_LIDDAR_USED
extern USBObject_t JRTSerial;                                                   // Define JRTSerial as UART object
extern JRT_write_measure_t distSensIT03Conf;                                    /* Chengdu JRT meter IT03 distance sensor measurement configuration object */
extern JRT_read_measure_t distSensIT03Meas;                                     /* Chengdu JRT meter IT03 distance sensor measurement reading */
#endif
#if defined(SERIAL_MAVLINK)
extern SerialMavObject_t MavLinkBuf;                                            /* Define MAVLink as UART object  */
#elif defined(USE_MAVLINK)                                                      /* then it must be over UDP */
extern EthMavlinkUDPObject_t MavLinkBuf;                                        /* Define MAVLinkBuf as UDP object */
#endif
#if defined(NAVILOCK_USED)
extern navilock_t g_NavObject;                                                  // navilock object
#endif

//#if defined(ENCODER_HELPER)
//extern
//#endif
#if (CAMERA_TYPE == XS_CAM)
extern EthXSUDPObject_t XS_DATA;                                                // External declarations for Ethernet Communication to COMPIC (encoder)
#endif
#if defined(SBGC_GIMBAL_HMI) || defined(SBGC_GIMBAL_JOY)
extern EthSBGCUDPObject_t SBGC_DATA;                                            // External declarations for Ethernet Communication to COMPIC (gimbal)
extern uint8_t g_SBGC_Stx_Char;                                                 // STX char for SimpleBGC
extern SBGC_progress_auto_pid_t pidreadbk;                                      // Container for the PID readback message
extern unsigned char *ptr_pidreadbk;                                            // Define the pointer to the container struct
extern SBGC_cmd_get_angles_t getangles;                                         // Conatiner for the GET_ANGLES readback message
extern unsigned char *ptr_getangles;                                            // Define the pointer to the container struct
extern SBGC_cmd_get_angles_ext_t getanglesext;                                  // Conatiner for the GET_ANGLES_EXT readback message
extern unsigned char *ptr_getanglesext;                                         // Define the pointer to the container struct
extern SBGC_cmd_realtime_data_3 realdata3;                                      // Conatiner for the REALTIME_DATA_3 readback message
extern unsigned char *ptr_realdata3;                                            // Define the pointer to the container struct
extern SBGC_cmd_realtime_data_4 realdata4;                                      // Conatiner for the REALTIME_DATA_4 readback message
extern unsigned char *ptr_realdata4;                                            // Define the pointer to the container struct
extern SBGC_cmd_confirmation_t cmdconf;                                         // Conatiner for the CMD_CONFIRM readback message
extern unsigned char *ptr_cmdconf;                                              // Define the pointer to the container struct
extern SBGC_cmd_error_t cmderror;                                               // Conatiner for the CMD_ERROR readback message
extern unsigned char *ptr_cmderror;                                             // Define the pointer to the container struct
extern SBGC_cmd_setget_adj_vars_val_t getvar_rd;                                // Container for the CMD_SET_ADJ_VARS_VAL command in response to above (7 values deep)
extern SBGC_cmd_board_info_t boardinforb;                                       // Container for BOARD_INFO readback
extern unsigned char *ptr_bi;                                                   // Define the pointer to the container struct
extern SBGC_cmd_read_params_3_t readparam3;                                     // Container for config param read/write
extern unsigned char *ptr_rp3;                                                  // Define the pointer to the container struct
extern SBGC_cmd_script_debug_t scriptstate;                                     // returned script state
extern SBGC_cmd_event_t eventCmd;                                               // Container for the CMD_EVENT which is a readback
extern unsigned char *ptr_eve;                                                  // Define the pointer to the container struct
extern SBGC_cmd_realtime_data_custom_reply_t realtimedatacust;                  // Container for real time data custom
#endif
#ifdef RUN_CAM_USED
extern EthRunCUDPObject_t RunC_DATA;                                            // External declarations for Ethernet Communication to COMPIC (run cam)
#endif
#ifdef GEF_EGD_PLC
extern EthEGDUDPObject_t EGD_DATA;                                              // Define struct to store EGD Data from GEC Fanuc PLC
#endif
#if defined(ART_NET_USED)
extern EthArtNetUDPObject_t artNet_DATA;                                        // Define struct to store artnet4 protocol data
#endif
#if defined(CBOR_COMS_USED)
//extern SOCKET_Intern_Dsc *CBORSocket;                                         // TCP ip socket for cbor
extern EthCoAPUDPObject_t CBOR_DATA;                                            // UDP data for cbor
#endif

#if defined(YI_CAM_USED)
extern SOCKET_Intern_Dsc *YiSocket;                                             // TCP ip socket
extern uint8_t g_YiCamReqState;                                                 // State of YiCam write messages
extern COMGS_YiOptActive_t hmiReqActive;                                        // structure to hold requests for each send message
extern uint8_t XYRequest;                                                       // Current button request (HMI Action Queue)
extern EthTCPObject_t YiCam_DATA;                                               // External declarations for Ethernet Communication to Yi Action Cam JSON server
extern TcpStateTimers YiTimer;                                                  // State timeouts for each TCP Step for the Socket used for Yi Cam
#endif
#if defined(SEQ_CAM_USED)
extern SOCKET_Intern_Dsc *SeqCamSocket;                                         // TCP ip socket for sequioa camera TCP/ip
extern EthTCPObject_t SeqCam_DATA;                                              // Parrot sequioa camera interface receieve / send packet buffer
extern TcpStateTimers SeqTimer;                                                 // State timeouts for each TCP Step for the Socket used for Yi Cam
extern uint8_t g_SeqCamReqState;                                                // State of Sequioa Cam write messages
#endif
#if defined(MODBUS_TCP)
extern SOCKET_Intern_Dsc *MbusSocket;                                           // TCP ip socket for modbus TCP/ip
extern EthTCPObject_t Modbus_DATA;                                              // modbus tcp receive/send Packet from/to slaves
#endif
#if defined(LW3_SWITCH_USED)
extern SOCKET_Intern_Dsc *Lw3Socket;                                            // TCP ip socket for LW3 over TCP/ip
extern EthTCPObject_t LW3_DATA;                                                 // LW3 tcp receive/send Packet from/to slaves
#endif
#if defined(MICROSCAN_USED)
#if defined(MICROSCAN_OUT_TCP)
extern SOCKET_Intern_Dsc *MiCroScanSocket;                                      // TCP ip socket for microscan ocr camera
extern EthTCPObject_t MiCroScaN_DATA;                                           // modbus tcp receive/send Packet from/to camera
#elif defined(MICROSCAN_OUT_UART)
extern SerialObject_t MiCroScaN_SER_LR;                                         // left right camera
extern SerialObject_t MiCroScaN_SER_FB;                                         // front back camera
#endif /* end microscan camera mode */
#endif /* end microscan camera include */

#if defined(USE_TCP_CAN)
extern SOCKET_Intern_Dsc *tcpCanSocket;                                         // TCP ip socket for can over TCP/ip
#endif
#if defined(REMOTE_TCP_AMP_XY_CAM)
extern SOCKET_Intern_Dsc *remAmpSocket;                                         // TCP ip socket for remote amp encoder/decoder socket
#endif
#if defined(REMOTE_TCP_RUN_CAM)
extern SOCKET_Intern_Dsc *remRunCamSocket;                                      // TCP ip socket for run cam camera
#endif
#if defined(REMOTE_TCP_SBGC)
extern SOCKET_Intern_Dsc *remSBGCSocket;                                        // TCP ip socket for sbgc gimbal
#endif
#if defined(JSON_COMS_USED)
extern SOCKET_Intern_Dsc *jSONSocket;                                           // TCP ip socket for json
#endif
#if defined(PENTAX_CAM_USED)
extern SOCKET_Intern_Dsc *pentaxSocket;                                         // TCP ip socket for pentax camera
#endif

// ============== Define the nodes =============================================
extern Node Air;                                                                // Air COM PIC
extern Node MasGs;                                                              // Master ground station
extern Node FirGs;                                                              // Fire ground station
extern Node This_Node;                                                          // Node set up using Init_Node()

extern unsigned char SYN;
extern unsigned char bad_ether;

// ================ Declare the global volatiles used in interrupt =============
extern volatile uint8_t g_start_timer5;                                         // Global signal to atart the timer
extern volatile uint64_t g_timer5_counter;                                      // Global Timer Counter volatile as globally written in interrupt

#ifdef UART4_INTERUPT
extern uint8_t g_extended;                                                      // Camera encoder reply message was extended type
#endif
extern Net_Ethernet_Intern_arpCacheStruct gRemoteUnit;                          // REmote Unit MAC Address
#ifdef GPS_INCLUDED2
extern GPS_Info_t g_posDataGPS;                                                 // GPS position data for UBLOX or futano type
extern unsigned char navStatGPS[50u];                                           // navigation Status description
extern unsigned char positionMode[20u];                                         // description of the position mode to be used with checksum for validity
#endif

#if (CAMERA_TYPE == XS_CAM)
//================= GLOBAL HMI SCREEN VALUES FROM XS ENCODER ===================
extern unsigned char g_diskStatus[30u];                                         // State of the recording disk as a string
extern unsigned char g_ch1Status[20u];                                          // Channel 1 state i.e recording or not
extern unsigned char g_ch2Status[20u];                                          // Channel 2 state i.e recording or not
extern unsigned char g_ch3Status[20u];                                          // Channel 3 state i.e recording or not
extern unsigned char g_ch4Status[20u];                                          // Channel 4 state i.e recording or not
extern unsigned char g_sysStatus[30u];                                          // State of the recording system
extern uint16_t g_totalSizeXS;                                                  // Total size of the disk in MB
extern uint16_t g_remSizeXS;                                                    // Remaining size of the disk in MB
extern unsigned char g_encTime[20u];                                            // Date and Time
extern unsigned char g_strField1[20u];                                          // String field 1 after +ok, comma
extern unsigned char g_strField2[20u];                                          // String field 2 after +ok,, comma
extern unsigned char g_strField3[20u];                                          // String field 3 after +ok,,, comma
extern unsigned char g_strField4[20u];                                          // String field 4 after +ok,,,, comma
extern unsigned char g_strField5[20u];                                          // String field 5 after +ok,,,,, comma
extern int32_t g_okField1;                                                      // numeric field extracted from +ok, { $GET<cmd> }
extern int32_t g_okField2;                                                      // numeric field extracted from +ok,, { $GET<cmd> }
extern int32_t g_okField3;                                                      // numeric field extracted from +ok,,, { $AVGBITRATE<cmd> }
extern int32_t g_okField4;                                                      // numeric field extracted from +ok,,,, { $AVGBITRATE<cmd> }
extern int32_t g_okField5;                                                      // numeric field extracted from +ok,,,,, { $ENUMSTREAMS<cmd> }
extern int32_t g_okField6;                                                      // numeric field extracted from +ok,,,,,,
extern int32_t g_okField7;                                                      // numeric field extracted from +ok,,,,,,,
extern int32_t g_okField8;                                                      // numeric field extracted from +ok,,,,,,,,
#endif

#ifdef GPS_INCLUDED2
//================ GLOBAL SATELITE INFO FROM GPS ===============================
extern uint8_t elevSatelData[4u];                                               // possible 4 satelite elevations
extern uint8_t azimuthSatelData[4u];                                            // possible 4 satelite azimuth
extern uint8_t sigStrengthSatelData[4u];                                        // possible 4 signal stgrengths
extern uint8_t signalIDData;                                                    // signal id 1: GPGSV or GLGSV 7: GAGSV
extern uint8_t satNumberData[4u];                                               // satelite number stored in byte
#endif
extern unsigned char g_XYtcpBuffer[XY_MSG_MAX_SND_LEN];                         // Buffer to send JSON requests to the webserver

#if defined(BATCH_MIXER)
extern volatile batch_steps_e g_batchState;                                     /* initialise the batch timers and start */
extern int64_t elapsedTime;                                                     /* elapsed time in sequence */
extern volatile uint64_t lastBatchTime;                                         /* last stored timer seconds */
#endif

extern volatile uint8_t g_hmiResetESD1;                                         /* external ESD */

//extern H_B Heart;                                                               // Heart beat type

typedef enum  {
    ROTATION_NONE                = 0,
    ROTATION_YAW_45              = 1,
    ROTATION_YAW_90              = 2,
    ROTATION_YAW_135             = 3,
    ROTATION_YAW_180             = 4,
    ROTATION_YAW_225             = 5,
    ROTATION_YAW_270             = 6,
    ROTATION_YAW_315             = 7,
    ROTATION_ROLL_180            = 8,
    ROTATION_ROLL_180_YAW_45     = 9,
    ROTATION_ROLL_180_YAW_90     = 10,
    ROTATION_ROLL_180_YAW_135    = 11,
    ROTATION_PITCH_180           = 12,
    ROTATION_ROLL_180_YAW_225    = 13,
    ROTATION_ROLL_180_YAW_270    = 14,
    ROTATION_ROLL_180_YAW_315    = 15,
    ROTATION_ROLL_90             = 16,
    ROTATION_ROLL_90_YAW_45      = 17,
    ROTATION_ROLL_90_YAW_90      = 18,
    ROTATION_ROLL_90_YAW_135     = 19,
    ROTATION_ROLL_270            = 20,
    ROTATION_ROLL_270_YAW_45     = 21,
    ROTATION_ROLL_270_YAW_90     = 22,
    ROTATION_ROLL_270_YAW_135    = 23,
    ROTATION_PITCH_90            = 24,
    ROTATION_PITCH_270           = 25,
    ROTATION_PITCH_180_YAW_90    = 26,
    ROTATION_PITCH_180_YAW_270   = 27,
    ROTATION_ROLL_90_PITCH_90    = 28,
    ROTATION_ROLL_180_PITCH_90   = 29,
    ROTATION_ROLL_270_PITCH_90   = 30,
    ROTATION_ROLL_90_PITCH_180   = 31,
    ROTATION_ROLL_270_PITCH_180  = 32,
    ROTATION_ROLL_90_PITCH_270   = 33,
    ROTATION_ROLL_180_PITCH_270  = 34,
    ROTATION_ROLL_270_PITCH_270  = 35,
    ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
    ROTATION_ROLL_90_YAW_270     = 37,
    ROTATION_ROLL_90_PITCH_68_YAW_293 = 38,
    ROTATION_PITCH_315           = 39,
    ROTATION_ROLL_90_PITCH_315   = 40,
    ROTATION_PITCH_7             = 41,
    ///////////////////////////////////////////////////////////////////////
    // Do not add more rotations without checking that there is not a conflict
    // with the MAVLink spec. MAV_SENSOR_ORIENTATION is expected to match our
    // list of rotations here. If a new rotation is added it needs to be added
    // to the MAVLink messages as well.
    ///////////////////////////////////////////////////////////////////////
    ROTATION_MAX,
    ROTATION_CUSTOM              = 100,
} Rotation_e;
// maximum rotation that will be used for auto-detection
#define ROTATION_MAX_AUTO_ROTATION ROTATION_ROLL_90_PITCH_315

// definitions used by quaterion and vector3f
#define HALF_SQRT_2 0.70710678118654757f

#ifdef __cplusplus
}
#endif

#endif //Transceiver_events_H