//
//   01112019 : Initial release support the following modes over UDP
//
//   SBGC :: (request) SBGC_CMD_BEEP_SOUND SBGC_CMD_MOTORS SBGC_CMD_PID SBGC_CMD_EXECUTE_MENU SBGC_CMD_SET_ADJ_VARS_VAL SBGC_CMD_GET_ADJ_VARS_VAL
//           SBGC_CMD_SELECT_IMU_3 SBGC_CMD_CONTROL SBGC_CMD_RESET SBGC_CONTROL_MODE_RC SBGC_CMD_API_VIRT_CH_CONTROL
//           (polled)  SBGC_CMD_GET_ANGLES SBGC_CMD_GET_ANGLES_EXT SBGC_CMD_REALTIME_DATA_4
//           (confirmed)  SBGC_CMD_CALIB_ACC SBGC_CMD_CALIB_GYRO SBGC_CMD_CALIB_MAG SBGC_CMD_BOOT_MODE_3 SBGC_CMD_CALIB_BAT SBGC_CMD_TRIGGER_PIN
//                        SBGC_CMD_MOTORS_ON: SBGC_CMD_SET_ADJ_VARS_VAL SBGC_CMD_AUTO_PID SBGC_CMD_I2C_WRITE_REG_BUF SBGC_CMD_WRITE_EXTERNAL_DATA
//                        SBGC_CMD_WRITE_ADJ_VARS_CFG SBGC_CMD_WRITE_FILE SBGC_CMD_CONTROL SBGC_CMD_CONTROL_CONFIG
//
//   encoder ::   start-up XStreamLogin  XStreamNetwork XStreamConf  XStreamRTSP XStreamDisk XStreamBitRate XStreamIInterval XStreamFrameRate
//                realtime XStreamContrast XStreamBrightness XStreamHue XStreamSaturation XStreamClipsize XStreamCliplength XStreamFrameRate XStreamBitRate
//                XStreamIInterval XSDiskStatus XSSysStatus
//
//   04122019 : Support library for JRT lidar bosch BME680
//   05112019 : Added CMD_EVENT and CMD_DATA_STREAM_INTERVAL and also to read serial UART2 data as well as UDP frames
//   06112019 : Support for various firmware revisions of gimbal and handling of error code messages as alarms to the GUI
//   07112019 : Support for PID Auto tune both versions < v2.7b0 and > v2.7b0 also made all buffers SBGC_CMD_MAX_BYTES
//   08112019 : Support for Filter configuration
//   11112019 : Support for configuration CMD_READ_PARAMS, CMD_READ_PARAMS_3 CMD_READ_PARAMS_EXT CMD_READ_PARAMS_EXT2 CMD_READ_PARAMS_EXT3
//   18112019 : Inclusion of helper routines for moving avaerage, quartenion, axis angle etc from the AHRS IMU
//   21122019 : Support of non-linear calibration for the Analogue input from the joystick
//   25112019 : Support for Axis movement library robot helper e.g. legs, pid controller and tuning position and speed controllers (not for gimbal)
//   26112019 : Support for scripting on the gimbal (automation of gimbal movement) and motor encoder calibration message
//   27112019 : Added Event Alarming for motors
//   28112019 : Added Bosch BME680 library. and further functions for Mahony and Madgewick
//   15112019 : Added support for GPS Furano and UBLOX which allows actions to be made to a PIC in the air on position like change PID rate or open contact
//              Added support for simpleBGC Gimbal scripting to be executed from a PIC32
//              Added Alarm queue and alarm message defintion for Alarms within a group or graphic object
//  17122019 :  Added XS Stream support for serial (testing without COM PIC) and $SYSSTATUS & $DISKSTATUS commands as well as $TIME requests
//  2020-01-07 : Read back settings to HMI from XS Encoder
//  2020-01-08 : Allow settings for XS Encoder to be per channel rather than all channels
//  2020-01-13 : Re-issue login for XS if we get a +AUTH response and alarm if invalid and incremented Net_Ethernet_Intern_userTimerSec every second for arp
//  2020-01-14 : Record Stop Commit Mark and REboot for XS Encoder supported
//  2020-01-15 : Support Date and Time function to XS Encoder and writing seconds since 1970 as a uint32_t
//  10012020 :  Supported CRC16 for new version (VER >= 2680U) SimpleBGC
//  20012020 :  Added support for Run Cam Camera
//  29012020 :  Added support for EGD GEC Fanuc PLC interface and setting deadbands for XS Encoder
//  03022020 :  Added support for Yi Action Cam JSON server communication via Jasmine library
//  04022020 :  Added support for TCP/IP messaging
//  17022020 : Added support to do various Yi Action Cam actions and config and readback group tag in jasmine for entire JSON configuration objects
//  18022020 : Implementing support libraries for Lidar Lightwave SF40 and LiddarTech Vu8 LWNX protocol and Modbus protocol libraries
//  20022020 : Modbus library extended to each variant ascii, rtu, tcp, udp, bin with all command codes and additional for leddartech support
//  21022020 : Color conversion codecs NV12_YUV420P RGB HSV YUV CMY CMYK Y,BY,RY YCbCr
//  27022020 : example for SF40 liddar library to read point data and calculate angle, also added library for lightwave LW3 protocol for Switches
//  02032020 : Sequoia library support via frozen library for JSON / Ajax
//  09032020 : Support for DMX ArtNet4 and Dali lighting protocols
//  10032020 : Added Odrive to go with Robo helper library as well as routines for forward kinematics
//  11032020 : Tidy up Pelco-D PTZ added, endian helper and http support for authenticate
//  23032020 : implemented microscan ANPR OCR camera interface
//  30032020 : sony visca protocol for thermal imaging camera
//  25092020 : update added numerous i2c spi devices, extra controllers displays codecs cameras protocols such as MAVLINK, J1939, AUVCAN, NMEA2000
//             GSM Modem, LwnX liddar, LW3 video switch, frozen json composer, smpte library, artnet4 library, dali lighting library, rdm,
//             Parrot Sequoia agriculture camera, pelco d ptz, http, canon, pentax, cbor, ciphers, encryption keys, geo json, xml parser, ubiBot agriculture 
//             ilda laser show creation, asn e1, rion, spektrum bus, linear11 conversion, linear16 conversion, power calculations, terrain follower, i2c ovm7690 camera
//             various dc motors and servos, bison, ntp, actisense ngt, ydwg, xt calls protocol, airmap, ubidot, message pack, protobuf, various AHRS filters
//             various positon and pose estiamtion algorthyms, 2.4ghz usb radio on spi, moving sign protocol, optical flow via mavlink ros bridge eth zurich,
//             various flight controllers and robot positon controls, various algoryhtms for manipulating picture data e.g. sobel filter, chromatic aberation, spline interpolation
//             support of NMEA2000 protocol, J1939 joystick, actuator, keyboard, pressure sensor, mellanger controller, indi controller,
// 24062021  : Added a lot of things since last updated (sorry) many maths functions, machine learning optimisation (random forrest, linear, logistic regression), robotics like ceres, AHRS compensations such as justa
//             allen variance, genetic evolution, optical flow, mitsubishi PLC SLMP, Ethernet IP, Kawasaki Robot, Cognex Pose Camera, morton encoding, genetic evolution,
//             yandex disk, vn100, univariate smooth, sound processing, sDa, OSC protocol, mavlink, naive bayes, tramp, vicon, displays moving sign protocol, XTCalls Protoool
//             various servos (voltz etc), linak actuators joystick and various canbus on J1939, cyrf bluetooth radio, ubidot, airmap, protobuf, message pack,
//             levenburg marquardt, navilock, voxel,                                          