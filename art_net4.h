// ART NET 4 - DMX512 over UDP
//
// includes code from Copyright (C) 2004-2006 Simon Newton
// upgraded to artnet4 by ACP Aviation and features added
//
// Can be used to control alsorts of lighting and video e.g. Robo spot motion cam
// various smoke or fire machines and lighting flash
//
#ifndef ARTNET4_LIB
#define ARTNET4_LIB

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "definitions.h"

#define ARTNET4_UDP_PORT 0x1936U                                                /* UDP rcv/tx port  */
#define ARTNET4_MAX_LEN 512U

/*
 * libartnet error codes
 */
typedef enum {
  ARTNET_EOK = 0,
  ARTNET_ENET = -1,                                                             /* network error      */
  ARTNET_EMEM = -2,                                                             /* memory error */
  ARTNET_EARG = -3,                                                             /* argument error */
  ARTNET_ESTATE = -4,                                                           /* state error  */
  ARTNET_EACTION = -5,                                                          /* invalid action */
} artpar_e;
/*
 * The maximum ports per node built into the ArtNet protocol.
 * This is always 4. Don't change it unless you really know what your doing
 */
enum { ARTNET_MAX_PORTS = 4u };
/**
 * The length of the short name field. Always 18
 */
enum { ARTNET_SHORT_NAME_LENGTH = 18u };
/**
 * The length of the long name field. Always 64
 */
enum { ARTNET_LONG_NAME_LENGTH = 64u };
/**
 * The length of the report field. Always 64
 */
enum { ARTNET_REPORT_LENGTH = 64u };
/**
 * The length of the DMX field. Always 512
 */
enum { ARTNET_DMX_LENGTH = 512u };
/*
 * Number of bytes in a RDM UID
 */
enum { ARTNET_RDM_UID_WIDTH = 6u };
/*
 * Length of the hardware address
 */
enum { ARTNET_MAC_SIZE = 6u };
/*
 * Length of the ESTA field
 */
enum { ARTNET_ESTA_SIZE = 2u };
/*
 * Length of the IP field
 */
enum { ARTNET_IP_SIZE = 4u };
/**
 * An enum for setting the behaviour of a port.
 * Ports can either input data (DMX -> ArtNet) or
 * output (ArtNet -> DMX) data.
 */

typedef enum {
  ARTNET_ENABLE_INPUT = 0x40u,                                                  /**< Enables the input for this port */
  ARTNET_ENABLE_OUTPUT = 0x80u                                                  /**< Enables the output for this port */
} artnet_port_settings_t;

typedef enum {
  ARTNET_PC_NONE = 0x00u,
  ARTNET_PC_CANCEL = 0x01u,
  ARTNET_PC_LED_NORMAL = 0x02u,
  ARTNET_PC_LED_MUTE = 0x03u,
  ARTNET_PC_LED_LOCATE = 0x04u,
  ARTNET_PC_RESET = 0x05u,
  ARTNET_PC_MERGE_LTP_O = 0x10u,
  ARTNET_PC_MERGE_LTP_1 = 0x11u,
  ARTNET_PC_MERGE_LTP_2 = 0x12u,
  ARTNET_PC_MERGE_LTP_3 = 0x13u,
  ARTNET_PC_MERGE_HTP_0 = 0x50u,
  ARTNET_PC_MERGE_HTP_1 = 0x51u,
  ARTNET_PC_MERGE_HTP_2 = 0x52u,
  ARTNET_PC_MERGE_HTP_3 = 0x53u,
  ARTNET_PC_CLR_0 = 0x93u,
  ARTNET_PC_CLR_1 = 0x93u,
  ARTNET_PC_CLR_2 = 0x93u,
  ARTNET_PC_CLR_3 = 0x93u,
} artnet_port_command_t;

/*
 * An enum for the type of data transmitted on a port.
 * As far as I know, only DMX-512 is supported
 */
typedef enum  {
  ARTNET_PORT_DMX = 0x00u,                                                      /**< Data is DMX-512 */
  ARTNET_PORT_MIDI = 0x01u,                                                     /**< Data is MIDI */
  ARTNET_PORT_AVAB = 0x02u,                                                     /**< Data is Avab */
  ARTNET_PORT_CMX = 0x03u,                                                      /**< Data is Colortran CMX */
  ARTNET_PORT_ADB = 0x04u,                                                      /**< Data is ABD 62.5 */
  ARTNET_PORT_ARTNET = 0x05u                                                    /**< Data is ArtNet */
} artnet_port_data_code;

typedef enum  {
  ARTNET_FIRMWARE_BLOCKGOOD = 0x00u,
  ARTNET_FIRMWARE_ALLGOOD = 0x01u,
  ARTNET_FIRMWARE_FAIL = 0xffu,
} artnet_firmware_status_code;                                                  /* defines the status of the firmware transfer */

typedef enum  {
  ARTNET_TOD_FULL = 0x00u,
  ARTNET_TOD_FLUSH = 0x01u,
} artnet_tod_command_code;                                                      /* tod actions */

/**
 * An enum for refering to a particular input or output port.
 */
typedef enum {
  ARTNET_INPUT_PORT = 1u,                                                       /**< The input port */
  ARTNET_OUTPUT_PORT,                                                           /**< The output port */
} artnet_port_dir_t;
/*

 * Enum describing the type of node
 */
typedef enum {
  ARTNET_SRV,                                                                   /**< An ArtNet server (transmitts DMX data) */
  ARTNET_NODE,                                                                  /**< An ArtNet node   (dmx reciever) */
  ARTNET_MSRV,                                                                  /**< A Media Server */
  ARTNET_ROUTE,                                                                 /**< No Effect currently */
  ARTNET_BACKUP,                                                                /**< No Effect currently */
  ARTNET_RAW                                                                    /**< Raw Node - used for diagnostics */
} artnet_node_type;

/*
 * Enum for the talk-to-me value
 * These values can be &'ed togeather, so for example to set private replies
 * and auto replying use :
 *   (ARTNET_TTM_PRIVATE & ARTNET_TTM_AUTO)
 */
typedef enum {
  ARTNET_TTM_DEFAULT = 0xFFu,                                                   /**< default, ArtPollReplies are broadcast, and nodes won't send a ArtPollReply when conditions change */
  ARTNET_TTM_PRIVATE = 0xFEu,                                                   /**< ArtPollReplies aren't broadcast */
  ARTNET_TTM_AUTO = 0xFDu                                                       /**< ArtPollReplies are send when node conditions change */
} artnet_ttm_value_t;

/**
 * Enums for the application defined handlers
 */
typedef enum {
  ARTNET_RECV_HANDLER,                                                          /**< Called on reciept of any ArtNet packet */
  ARTNET_SEND_HANDLER,                                                          /**< Called on transmission of any ArtNet packet */
  ARTNET_POLL_HANDLER,                                                          /**< Called on reciept of an ArtPoll packet */
  ARTNET_REPLY_HANDLER,                                                         /**< Called on reciept of an ArtPollReply packet */
  ARTNET_DMX_HANDLER,                                                           /**< Called on reciept of an ArtDMX packet */
  ARTNET_ADDRESS_HANDLER,                                                       /**< Called on reciept of an ArtAddress packet */
  ARTNET_INPUT_HANDLER,                                                         /**< Called on reciept of an ArtInput packet */
  ARTNET_TOD_REQUEST_HANDLER,                                                   /**< Called on reciept of an ArtTodRequest packet */
  ARTNET_TOD_DATA_HANDLER,                                                      /**< Called on reciept of an ArtTodData packet */
  ARTNET_TOD_CONTROL_HANDLER,                                                   /**< Called on reciept of an ArtTodControl packet */
  ARTNET_RDM_HANDLER,                                                           /**< Called on reciept of an ArtRdm packet */
  ARTNET_IPPROG_HANDLER,                                                        /**< Called on reciept of an ArtIPProg packet */
  ARTNET_FIRMWARE_HANDLER,                                                      /**< Called on reciept of an ArtFirmware packet */
  ARTNET_FIRMWARE_REPLY_HANDLER,                                                /**< Called on reciept of an ArtFirmwareReply packet */
} artnet_handler_name_t;

/*
 * Describes a remote ArtNet node that has been discovered
 */
typedef struct artnet_node_entry_s {

  uint8_t ip[ARTNET_IP_SIZE];                                                   /**< The IP address, Network byte ordered*/
  int16_t ver;                                                                  /**< The firmware version */
  int16_t sub;                                                                  /**< The subnet address */
  int16_t oem;                                                                  /**< The OEM value */
  uint8_t ubea;                                                                 /**< The UBEA version */
  uint8_t status;
  uint8_t etsaman[ARTNET_ESTA_SIZE];                                            /**< The ESTA Manufacturer code */
  uint8_t shortname[ARTNET_SHORT_NAME_LENGTH];                                  /**< The short node name */
  uint8_t longname[ARTNET_LONG_NAME_LENGTH];                                    /**< The long node name */
  uint8_t nodereport[ARTNET_REPORT_LENGTH];                                     /**< The node report */
  int16_t numbports;                                                            /**< The number of ports */
  uint8_t porttypes[ARTNET_MAX_PORTS];                                          /**< The type of ports */
  uint8_t goodinput[ARTNET_MAX_PORTS];
  uint8_t goodoutput[ARTNET_MAX_PORTS];
  uint8_t swin[ARTNET_MAX_PORTS];
  uint8_t swout[ARTNET_MAX_PORTS];
  uint8_t swvideo;
  uint8_t swmacro;
  uint8_t swremote;
  uint8_t style;
  uint8_t mac[ARTNET_MAC_SIZE];                                                 /**< The MAC address of the node */
} artnet_node_entry_t;

typedef artnet_node_entry_t *artnet_node_entry;                                 /** A pointer to an artnet_node_entry_t */

typedef struct {
  char short_name[ARTNET_SHORT_NAME_LENGTH];
  char long_name[ARTNET_LONG_NAME_LENGTH];
  uint8_t subnet;
  uint8_t in_ports[ARTNET_MAX_PORTS];
  uint8_t out_ports[ARTNET_MAX_PORTS];
} artnet_node_config_t;

enum { ARTNET_MAX_RDM_ADCOUNT = 32u };

enum { ARTNET_MAX_UID_COUNT = 200u };

enum { ARTNET_MAX_RDM_DATA = 512u };                                            /* according to the rdm spec, this should be 278 bytes   we'll set to 512 here, the firmware datagram is still bigger */

enum { ARTNET_FIRMWARE_SIZE = 512u };

#if (MCU_NAME == PIC32MX795F512L)                                               /* 32 bit enums not allowed */
#define ARTNET_POLL 0x2000ul                                                    // This is an ArtPoll packet, no other data is contained in this UDP packet.
#define ARTNET_REPLY 0x2100ul                                                   // This is an ArtPollReply Packet. It contains device status information.
#define ARTNET_OpDiagData 0x2300ul                                              // Diagnostics and data logging packet
#define ARTNET_OpCommand 0x2400ul                                               // Used to send text based parameter commands.
#define ARTNET_DMX 0x5000ul                                                     // This is an ArtDmx data packet. It contains zero start code DMX512 information for a single Universe
#define ARTNET_OpNzs 0x5100ul                                                   // This is an ArtNzs data packet. It contains non-zero start code (except RDM) DMX512 information for a single Universe
#define ARTNET_OpSync 0x5200ul                                                  // This is an ArtSync data packet. It is used to force synchronous transfer of ArtDmx packets to a node’s output
#define ARTNET_ADDRESS 0x6000ul                                                 // This is an ArtAddress packet. It contains remote programming information for a Node.
#define ARTNET_INPUT 0x7000ul                                                   // This is an ArtInput packet. It contains enable – disable data for DMX inputs
#define ARTNET_TODREQUEST 0x8000ul                                              // This is an ArtTodRequest packet. It is used to request a Table of Devices (ToD) for RDM discovery.
#define ARTNET_TODDATA 0x8100ul                                                 // This is an ArtTodData packet. It is used to send a Table of Devices (ToD) for RDM discovery.
#define ARTNET_TODCONTROL 0x8200ul                                              // This is an ArtTodControl packet. It is used to send RDM discovery control messages
#define ARTNET_RDM 0x8300ul                                                     // This is an ArtRdm packet. It is used to send all non discovery RDM messages.
#define ARTNET_OpRdmSub 0x8400ul                                                // This is an ArtRdmSub packet. It is used to send compressed, RDM Sub-Device data.
#define ARTNET_VIDEOSTEUP 0xa010ul                                              // This is an ArtVideoSetup packet. It contains video screen setup information for nodes that implement the extended video features
#define ARTNET_VIDEOPALETTE 0xa020ul                                            // This is an ArtVideoPalette packet. It contains colour palette setup information for nodes that implement the extended video features.
#define ARTNET_VIDEODATA 0xa040ul                                               // This is an ArtVideoData packet. It contains display data for nodes that implement the extended video features.
#define ARTNET_MACMASTER 0xf000ul                                               // This packet is deprecated
#define ARTNET_MACSLAVE 0xf100ul                                                // This packet is deprecated
#define ARTNET_FIRMWAREMASTER 0xf200ul                                          // This is an ArtFirmwareMaster packet. It is used to upload new firmware or firmware extensions to the Node
#define ARTNET_FIRMWAREREPLY 0xf300ul                                           // This is an ArtFirmwareReply packet. It is returned by the node to acknowledge receipt of an ArtFirmwareMaster packet or ArtFileTnMaster packet.
#define ARTNET_OpFileTnMaster 0xf400ul                                          // Uploads file to the node
#define ARTNET_OpFileFnMaster 0xf500ul                                          // Downloads user file from node.
#define ARTNET_OpFileFnReply 0xf600ul                                           // Server to Node acknowledge for download packets.
#define ARTNET_IPPROG 0xf800ul                                                  // This is an ArtIpProg packet. It is used to reprogramme the IP address and Mask of the Node.
#define ARTNET_IPREPLY 0xf900ul                                                 // This is an ArtIpProgReply packet. It is returned by the node to acknowledge receipt of an ArtIpProg packet.
#define ARTNET_MEDIA 0x9000ul                                                   // This is an ArtMedia packet. It is Unicast by a Media Server and acted upon by a Controller.
#define ARTNET_OpMediaPatch 0x9100ul                                            // This is an ArtMediaPatch packet. It is Unicast by a Controller and acted upon by a Media Server
#define ARTNET_MEDIAPATCH 0x9200ul                                              // This is an ArtMediaControl packet. It is Unicast by a Controller and acted upon by a Media Server
#define ARTNET_MEDIACONTROLREPLY 0x9300ul                                       // This is an ArtMediaControlReply packet. It is Unicast by a Media Server and acted upon by a Controller.
#define ARTNET_OpTimeCode 0x9700ul                                              // This is an ArtTimeCode packet. It is used to transport time code over the network.
#define ARTNET_OpTimeSync 0x9800ul                                              // Used to synchronise real time date and clock
#define ARTNET_OpTrigger 0x9900ul                                               // Used to send trigger macros
#define ARTNET_OpDirectory 0x9a00ul                                             // Requests a node's file list
#define ARTNET_OpDirectoryReply 0x9b00ul                                        // Replies to OpDirectory with file list
#else
enum artnet_packet_type_e {                                                     /* these are the valid .OpCode variables */
  ARTNET_POLL = 0x2000ul,                                                       // This is an ArtPoll packet, no other data is contained in this UDP packet.
  ARTNET_REPLY = 0x2100ul,                                                      // This is an ArtPollReply Packet. It contains device status information.
  ARTNET_OpDiagData = 0x2300ul,                                                 // Diagnostics and data logging packet
  ARTNET_OpCommand = 0x2400ul,                                                  // Used to send text based parameter commands.
  ARTNET_DMX = 0x5000ul,                                                        // This is an ArtDmx data packet. It contains zero start code DMX512 information for a single Universe
  ARTNET_OpNzs = 0x5100ul,                                                      // This is an ArtNzs data packet. It contains non-zero start code (except RDM) DMX512 information for a single Universe
  ARTNET_OpSync = 0x5200ul,                                                     // This is an ArtSync data packet. It is used to force synchronous transfer of ArtDmx packets to a node’s output
  ARTNET_ADDRESS = 0x6000ul,                                                    // This is an ArtAddress packet. It contains remote programming information for a Node.
  ARTNET_INPUT = 0x7000ul,                                                      // This is an ArtInput packet. It contains enable – disable data for DMX inputs
  ARTNET_TODREQUEST = 0x8000ul,                                                 // This is an ArtTodRequest packet. It is used to request a Table of Devices (ToD) for RDM discovery.
  ARTNET_TODDATA = 0x8100ul,                                                    // This is an ArtTodData packet. It is used to send a Table of Devices (ToD) for RDM discovery.
  ARTNET_TODCONTROL = 0x8200ul,                                                 // This is an ArtTodControl packet. It is used to send RDM discovery control messages
  ARTNET_RDM = 0x8300ul,                                                        // This is an ArtRdm packet. It is used to send all non discovery RDM messages.
  ARTNET_OpRdmSub = 0x8400ul,                                                   // This is an ArtRdmSub packet. It is used to send compressed, RDM Sub-Device data.
  ARTNET_VIDEOSTEUP = 0xa010ul,                                                 // This is an ArtVideoSetup packet. It contains video screen setup information for nodes that implement the extended video features
  ARTNET_VIDEOPALETTE = 0xa020ul,                                               // This is an ArtVideoPalette packet. It contains colour palette setup information for nodes that implement the extended video features.
  ARTNET_VIDEODATA = 0xa040ul,                                                  // This is an ArtVideoData packet. It contains display data for nodes that implement the extended video features.
  ARTNET_MACMASTER = 0xf000ul,                                                  // This packet is deprecated
  ARTNET_MACSLAVE = 0xf100ul,                                                   // This packet is deprecated
  ARTNET_FIRMWAREMASTER = 0xf200ul,                                             // This is an ArtFirmwareMaster packet. It is used to upload new firmware or firmware extensions to the Node
  ARTNET_FIRMWAREREPLY = 0xf300ul,                                              // This is an ArtFirmwareReply packet. It is returned by the node to acknowledge receipt of an ArtFirmwareMaster packet or ArtFileTnMaster packet.
  ARTNET_OpFileTnMaster = 0xf400ul,                                             // Uploads file to the node
  ARTNET_OpFileFnMaster = 0xf500ul,                                             // Downloads user file from node.
  ARTNET_OpFileFnReply = 0xf600ul,                                              // Server to Node acknowledge for download packets.
  ARTNET_IPPROG = 0xf800ul,                                                     // This is an ArtIpProg packet. It is used to reprogramme the IP address and Mask of the Node.
  ARTNET_IPREPLY = 0xf900ul,                                                    // This is an ArtIpProgReply packet. It is returned by the node to acknowledge receipt of an ArtIpProg packet.
  ARTNET_MEDIA = 0x9000ul,                                                      // This is an ArtMedia packet. It is Unicast by a Media Server and acted upon by a Controller.
  ARTNET_OpMediaPatch = 0x9100ul,                                               // This is an ArtMediaPatch packet. It is Unicast by a Controller and acted upon by a Media Server
  ARTNET_MEDIAPATCH = 0x9200ul,                                                 // This is an ArtMediaControl packet. It is Unicast by a Controller and acted upon by a Media Server
  ARTNET_MEDIACONTROLREPLY = 0x9300ul,                                          // This is an ArtMediaControlReply packet. It is Unicast by a Media Server and acted upon by a Controller.
  ARTNET_OpTimeCode = 0x9700ul,                                                 // This is an ArtTimeCode packet. It is used to transport time code over the network.
  ARTNET_OpTimeSync = 0x9800ul,                                                 // Used to synchronise real time date and clock
  ARTNET_OpTrigger = 0x9900ul,                                                  // Used to send trigger macros
  ARTNET_OpDirectory = 0x9a00ul,                                                // Requests a node's file list
  ARTNET_OpDirectoryReply = 0x9b00ul                                            // Replies to OpDirectory with file list
}__attribute__((packed));
#endif

typedef enum artnet_packet_type_e artnet_packet_type_t;

struct artnet_poll_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // The OpCode defines the class of data following ArtPoll within this UDP packet. Transmitted low byte first. See Table 1 for the OpCode listing. Set to OpPoll.
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number.
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14. Controllers should ignore communication with nodes using a protocol version lower than 14.
  uint8_t  ttm;                                                                 // 7-5 Unused, transmit as zero, do not test upon receipt. 4 0 = Enable VLC transmission. 1 = Disable VLC transmission. 3 0 = Diagnostics messages are broadcast. (if bit 2).  1 = Diagnostics messages are unicast. (if bit 2). 2 0 = Do not send me diagnostics messages. 1 = Send me diagnostics messages. 1 0 = Only send ArtPollReply in response to an ArtPoll or ArtAddress. 1 = Send ArtPollReply whenever Node conditions change. This selection allows the Controller to be informed of changes without the need to continuously poll. 0 0 = Deprecated.
  uint8_t  pad;                                                                 // priority
} __attribute__((packed));

typedef struct artnet_poll_s artnet_poll_t;

#ifdef USE_OLD_VERSION
struct artnet_reply_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpPollReply Transmitted low byte firs
  uint8_t  ip[4u];                                                              // ip
  uint16_t port;                                                                // The Port is always 0x1936 Transmitted low byte first.
  uint8_t  verH;                                                                // High byte of Node’s firmware revision number. The Controller should only use this field to decide if a firmware update should proceed. The convention is that a higher number is a more recent release of firmware
  uint8_t  ver;                                                                 // Low byte of Node’s firmware revision number.
  uint8_t  subH;                                                                // Bits 14-8 of the 15 bit Port-Address are encoded into the bottom 7 bits of this field. This is used in combination with SubSwitch and SwIn[] or SwOut[] to produce the full universe address.
  uint8_t  sub;                                                                 // Bits 7-4 of the 15 bit Port-Address are encoded into the bottom 4 bits of this field. This is used in combination with NetSwitch and SwIn[] or SwOut[] to produce the full universe address
  uint8_t  oemH;                                                                // The high byte of the Oem value
  uint8_t  oem;                                                                 // The low byte of the Oem value.  The Oem word describes the equipment vendor and the feature set available. Bit 15 high indicates extended features available
  uint8_t  ubea;                                                                // This field contains the firmware version of the User Bios Extension Area (UBEA). If the UBEA is not programmed, this field contains zero
  uint8_t  status;                                                              // General Status register containing bit fields
  uint8_t  etsaman[2u];                                                         // The ESTA manufacturer code. These codes are used to represent equipment manufacturer. They are assigned by ESTA. This field can be interpreted as two ASCII bytes representing the manufacturer initials
  uint8_t  shortname[ARTNET_SHORT_NAME_LENGTH];                                 // The array represents a null terminated short name for the Node. The Controller uses the ArtAddress packet to program this string. Max length is 17 characters plus the null. This is a fixed length field, although the string it contains can be shorter than the field.
  uint8_t  longname[ARTNET_LONG_NAME_LENGTH];                                   // The array represents a null terminated long name for the Node. The Controller uses the ArtAddress packet to program this string. Max length is 63 characters plus the null. This is a fixed length field, although the string it contains can be shorter than the field.
  uint8_t  nodereport[ARTNET_REPORT_LENGTH];                                    // The array is a textual report of the Node’s operating status or operational errors. It is primarily intended for ‘engineering’ data rather than ‘end user’ data. The field is formatted as: “#xxxx [yyyy..] zzzzz…” xxxx is a hex status code as defined in Table 3. yyyy is a decimal counter that increments every time the Node sends an ArtPollResponse. This allows the controller to monitor event changes in the Node.  zzzz is an English text string defining the status. This is a fixed length field, although the string it contains can be shorter than the field.
  uint8_t  numbportsH;                                                          // The high byte of the word describing the number of input or output ports. The high byte is for future expansion and is currently zero.
  uint8_t  numbports;                                                           // The low byte of the word describing the number of input or output ports. If number of inputs is not equal to number of outputs, the largest value is taken. Zero is a legal value if no input or output ports are implemented. The maximum value is 4. Nodes can ignore this field as the information is implicit in PortTypes[].
  uint8_t  porttypes[ARTNET_MAX_PORTS];                                         // This array defines the operation and protocol of each channel. (A product with 4 inputs and 4 outputs would report 0xc0, 0xc0, 0xc0, 0xc0). The array length is fixed, independent of the number of inputs or outputs physically available on the Node
  uint8_t  goodinput[ARTNET_MAX_PORTS];                                         // input status of the node
  uint8_t  goodoutput[ARTNET_MAX_PORTS];                                        // output status of the node
  uint8_t  swin[ARTNET_MAX_PORTS];                                              // Bits 3-0 of the 15 bit Port-Address for each of the 4 possible input ports are encoded into the low nibble
  uint8_t  swout[ARTNET_MAX_PORTS];                                             // Bits 3-0 of the 15 bit Port-Address for each of the 4 possible output ports are encoded into the low nibble
  uint8_t  swvideo;                                                             // Set to 00 when video display is showing local data. Set to 01 when video is showing ethernet data. The field is now deprecate
  uint8_t  swmacro;                                                             // If the Node supports macro key inputs, this byte represents the trigger values. The Node is responsible for ‘debouncing’ inputs. When the ArtPollReply is set to transmit automatically, (TalkToMe Bit 1), the ArtPollReply will be sent on both key down and key up events. However, the Controller should not assume that only one bit position has changed. The Macro inputs are used for remote event triggering or cueing.
  uint8_t  swremote;                                                            // If the Node supports remote trigger inputs, this byte represents the trigger values. The Node is responsible for ‘debouncing’ inputs. When the ArtPollReply is set to transmit automatically, (TalkToMe Bit 1), the ArtPollReply will be sent on both key down and key up events. However, the Controller should not assume that only one bit position has changed. The Remote inputs are used for remote event triggering or cueing
  uint8_t  sp1;                                                                 // spares
  uint8_t  sp2;
  uint8_t  sp3;
  uint8_t  style;                                                               // The Style code defines the equipment style of the device. See Table 4 for current Style codes
  uint8_t  mac[ARTNET_MAC_SIZE];                                                // mac address
  uint8_t  filler[32u];                                                         // Transmit as zero. For future expansion.
} __attribute__((packed));
#else
struct artnet_reply_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpPollReply Transmitted low byte firs
  uint8_t  ip[4u];                                                              // ip
  uint16_t port;                                                                // The Port is always 0x1936 Transmitted low byte first.
  uint8_t  verH;                                                                // High byte of Node’s firmware revision number. The Controller should only use this field to decide if a firmware update should proceed. The convention is that a higher number is a more recent release of firmware
  uint8_t  ver;                                                                 // Low byte of Node’s firmware revision number.
  uint8_t  subH;                                                                // Bits 14-8 of the 15 bit Port-Address are encoded into the bottom 7 bits of this field. This is used in combination with SubSwitch and SwIn[] or SwOut[] to produce the full universe address.
  uint8_t  sub;                                                                 // Bits 7-4 of the 15 bit Port-Address are encoded into the bottom 4 bits of this field. This is used in combination with NetSwitch and SwIn[] or SwOut[] to produce the full universe address
  uint8_t  oemH;                                                                // The high byte of the Oem value
  uint8_t  oem;                                                                 // The low byte of the Oem value.  The Oem word describes the equipment vendor and the feature set available. Bit 15 high indicates extended features available
  uint8_t  ubea;                                                                // This field contains the firmware version of the User Bios Extension Area (UBEA). If the UBEA is not programmed, this field contains zero
  uint8_t  status;                                                              // General Status register containing bit fields
  uint8_t  etsaman[2u];                                                         // The ESTA manufacturer code. These codes are used to represent equipment manufacturer. They are assigned by ESTA. This field can be interpreted as two ASCII bytes representing the manufacturer initials
  uint8_t  shortname[ARTNET_SHORT_NAME_LENGTH];                                 // The array represents a null terminated short name for the Node. The Controller uses the ArtAddress packet to program this string. Max length is 17 characters plus the null. This is a fixed length field, although the string it contains can be shorter than the field.
  uint8_t  longname[ARTNET_LONG_NAME_LENGTH];                                   // The array represents a null terminated long name for the Node. The Controller uses the ArtAddress packet to program this string. Max length is 63 characters plus the null. This is a fixed length field, although the string it contains can be shorter than the field.
  uint8_t  nodereport[ARTNET_REPORT_LENGTH];                                    // The array is a textual report of the Node’s operating status or operational errors. It is primarily intended for ‘engineering’ data rather than ‘end user’ data. The field is formatted as: “#xxxx [yyyy..] zzzzz…” xxxx is a hex status code as defined in Table 3. yyyy is a decimal counter that increments every time the Node sends an ArtPollResponse. This allows the controller to monitor event changes in the Node.  zzzz is an English text string defining the status. This is a fixed length field, although the string it contains can be shorter than the field.
  uint8_t  numbportsH;                                                          // The high byte of the word describing the number of input or output ports. The high byte is for future expansion and is currently zero.
  uint8_t  numbports;                                                           // The low byte of the word describing the number of input or output ports. If number of inputs is not equal to number of outputs, the largest value is taken. Zero is a legal value if no input or output ports are implemented. The maximum value is 4. Nodes can ignore this field as the information is implicit in PortTypes[].
  uint8_t  porttypes[ARTNET_MAX_PORTS];                                         // This array defines the operation and protocol of each channel. (A product with 4 inputs and 4 outputs would report 0xc0, 0xc0, 0xc0, 0xc0). The array length is fixed, independent of the number of inputs or outputs physically available on the Node
  uint8_t  goodinput[ARTNET_MAX_PORTS];                                         // input status of the node
  uint8_t  goodoutput[ARTNET_MAX_PORTS];                                        // output status of the node
  uint8_t  swin[ARTNET_MAX_PORTS];                                              // Bits 3-0 of the 15 bit Port-Address for each of the 4 possible input ports are encoded into the low nibble
  uint8_t  swout[ARTNET_MAX_PORTS];                                             // Bits 3-0 of the 15 bit Port-Address for each of the 4 possible output ports are encoded into the low nibble
  uint8_t  swvideo;                                                             // Set to 00 when video display is showing local data. Set to 01 when video is showing ethernet data. The field is now deprecate
  uint8_t  swmacro;                                                             // If the Node supports macro key inputs, this byte represents the trigger values. The Node is responsible for ‘debouncing’ inputs. When the ArtPollReply is set to transmit automatically, (TalkToMe Bit 1), the ArtPollReply will be sent on both key down and key up events. However, the Controller should not assume that only one bit position has changed. The Macro inputs are used for remote event triggering or cueing.
  uint8_t  swremote;                                                            // If the Node supports remote trigger inputs, this byte represents the trigger values. The Node is responsible for ‘debouncing’ inputs. When the ArtPollReply is set to transmit automatically, (TalkToMe Bit 1), the ArtPollReply will be sent on both key down and key up events. However, the Controller should not assume that only one bit position has changed. The Remote inputs are used for remote event triggering or cueing
  uint8_t  sp1;                                                                 // spares
  uint8_t  sp2;
  uint8_t  sp3;
  uint8_t  style;                                                               // The Style code defines the equipment style of the device. See Table 4 for current Style codes
  uint8_t  mac[ARTNET_MAC_SIZE];                                                // mac address
  uint8_t  BindIp[4u];                                                          // If this unit is part of a larger or modular product, this is the IP of the root device
  uint8_t  BindIndex;                                                           // This number represents the order of bound devices. A lower number means closer to root device. A value of 1 means root device
  uint8_t  Status2;                                                             // product support word
  uint8_t  filler[26u];                                                         // Transmit as zero. For future expansion.
} __attribute__((packed));
#endif

// status 1
#define ARTNET_STAT1_UBEA (1<<0)                                                // Set - UBEA present
#define ARTNET_STAT1_RDM (1<<1)                                                 // Set - RDM capable
#define ARTNET_STAT1_ROM (1<<2)                                                 // Set - booted from ROM otherwise flash
#define ARTNET_STAT1_PA_FRONT(x) { ((x>>4U) & 0x01U) }                          // Set - port address programmed on front panel
#define ARTNET_STAT1_PA_WEBB(x) { ((x>>4U) & 0x02U) }                           // Set - port address programmed on web browser
#define ARTNET_STAT1_IND_FRONT(x) { ((x>>6U) & 0x01U) }                         // Set - Indicators in Locate / Identify mode
#define ARTNET_STAT1_IND_MUTE(x) { ((x>>6U) & 0x02U) }                          // Set - Indicators in Mute Mode.
#define ARTNET_STAT1_IND_NORM(x) { ((x>>6U) & 0x03U) }                          // Set - Indicators in Normal Mode.

// port types
#define ARTNET_PORT_DMX512 0b000000                                             // DMX512
#define ARTNET_PORT_MIDI 0b000001                                               // Midi
#define ARTNET_PORT_AVAB 0b000010                                               // Avab
#define ARTNET_PORT_CMX 0b000011                                                // ColorTran CMX
#define ARTNET_PORT_ADB62 0b000100                                              // ADB 62.5
#define ARTNET_PORT_ARTNET 0b000101                                             // ARTNET
#define ARTNET_PORT_INPUT_ON (1u<<6u)                                           // Set if this channel can input onto the Art-Net Network.
#define ARTNET_PORT_OUTPUT_ON (1u<<7u)                                          // Set is this channel can output data from the Art-Net Network.

// good input word
#define ARTNET_IN_RCV_ERR (1u<<2u)                                              // Set – Receive errors detected.
#define ARTNET_IN_DISABL (1u<<3u)                                               // Set – Input is disabled.
#define ARTNET_IN_DMX_TEST1 (1u<<4u)                                            // Set – Channel includes DMX512 text packets
#define ARTNET_IN_DMX_SIP (1u<<5u)                                              // Set – Channel includes DMX512 SIP’s
#define ARTNET_IN_DMX_TEST2 (1u<<6u)                                            // Set – Channel includes DMX512 text packets
#define ARTNET_IN_DMX_RCV (1u<<7u)                                              // Set – Data received.

// good output word
#define ARTNET_OUT_sACN_ART (1u<<0u)                                            // Set – Output is selected to transmit sACN. Clr - ArtNet
#define ARTNET_OUT_MM_ATP (1u<<1u)                                              // Set – Merget mode is ATP
#define ARTNET_OUT_DMX_SHORT (1u<<2u)                                           // Set – DMX short detected
#define ARTNET_OUT_MERGE_ART (1u<<3u)                                           // Set – Output is merging ArtNet data.
#define ARTNET_OUT_DMX_TEST1 (1u<<4u)                                           // Set – Channel includes DMX512 text packets.
#define ARTNET_OUT_DMX_SIP (1u<<5u)                                             // Set – Channel includes DMX512 SIP’s.
#define ARTNET_OUT_DMX_TEST2 (1u<<6u)                                           // Set – Channel includes DMX512 text packets.
#define ARTNET_OUT_DMX_DATA (1u<<7u)                                            // Set – Data is being transmitted.

// status 2 word
#define ARTNET_STATUS2_WEB_BROWSER (1u<<0u)                                     // bit 1 set product supports web browser
#define ARTNET_STATUS2_CONF_DHCP (1u<<1u)                                       // bit 1 set product configured DHCP
#define ARTNET_STATUS2_SUP_DHCP (1u<<2u)                                        // bit 1 set product supports DHCP
#define ARTNET_STATUS2_VER (1u<<3u)                                             // bit 1 set 15 bit port address ARtnet3 or 4 otherwise Artnet2
#define ARTNET_STATUS2_sACN (1u<<4u)                                            // bit 1 set can switch between Artnet and sACN
#define ARTNET_STATUS2_SQUK (1u<<5u)                                            // bit 1 set squawking.

typedef struct artnet_reply_s artnet_reply_t;

#if (MCU_NAME == PIC32MX795F512L)                                               /* 32 bit enums not allowed */
#define ARTNET_RcDebug 0x0000ul,                                                /* Booted in debug mode (Only used in development) */
#define ARTNET_RcPowerOk 0x0001ul,                                              /* Power On Tests successful                       */
#define ARTNET_RcPowerFail 0x0002ul,                                            /* Hardware tests failed at Power On               */
#define ARTNET_RcSocketWr1 0x0003ul,                                            /* Last UDP from Node failed due to truncated length,  Most likely caused by a collision. */
#define ARTNET_RcParseFail 0x0004ul,                                            /* Unable to identify last UDP transmission. Check OpCode and packet length. */
#define ARTNET_RcUdpFail 0x0005ul,                                              /* Unable to open Udp Socket in last transmission attempt */
#define ARTNET_RcShNameOk 0x0006ul,                                             /* Confirms that Short Name programming via ArtAddress, was successful. */
#define ARTNET_RcLoNameOk 0x0007ul,                                             /* Confirms that Long Name programming via ArtAddress, was successful.  */
#define ARTNET_RcDmxError 0x0008ul,                                             /* DMX512 receive errors detected.   */
#define ARTNET_RcDmxUdpFull 0x0009ul,                                           /* Ran out of internal DMX transmit buffers. */
#define ARTNET_RcDmxRxFull 0x000Aul,                                            /* Ran out of internal DMX receive buffers.  */
#define ARTNET_RcSwitchErr 0x000Bul,                                            /* Rx Universe switches conflict.  */
#define ARTNET_RcConfigErr 0x000Cul,                                            /* DMX512 receive errors detected. */
#define ARTNET_RcDmxShort 0x000Dul,                                             /* DMX output short detected. See GoodOutput field. */
#define ARTNET_RcFirmwareFail 0x000Eul,                                         /* Last attempt to upload new firmware failed       */
#define ARTNET_RcUserFail 0x000Ful,                                             /* User changed switch settings when address locked by remote programming. User changes ignored */
#define ARTNET_RcFactoryRes 0x0010ul                                            /* Factory reset has occurred  */
#else
enum ARTNET_node_report_e {                                                     // these are the valid .NodeReportCode variables
  ARTNET_RcDebug = 0x0000ul,                                                    // Booted in debug mode (Only used in development)
  ARTNET_RcPowerOk = 0x0001ul,                                                  // Power On Tests successful
  ARTNET_RcPowerFail = 0x0002ul,                                                // Hardware tests failed at Power On
  ARTNET_RcSocketWr1 = 0x0003ul,                                                // Last UDP from Node failed due to truncated length,  Most likely caused by a collision.
  ARTNET_RcParseFail= 0x0004ul,                                                 // Unable to identify last UDP transmission. Check OpCode and packet length.
  ARTNET_RcUdpFail = 0x0005ul,                                                  // Unable to open Udp Socket in last transmission attempt
  ARTNET_RcShNameOk = 0x0006ul,                                                 // Confirms that Short Name programming via ArtAddress, was successful.
  ARTNET_RcLoNameOk = 0x0007ul,                                                 // Confirms that Long Name programming via ArtAddress, was successful.
  ARTNET_RcDmxError = 0x0008ul,                                                 // DMX512 receive errors detected.
  ARTNET_RcDmxUdpFull = 0x0009ul,                                               // Ran out of internal DMX transmit buffers.
  ARTNET_RcDmxRxFull = 0x000Aul,                                                // Ran out of internal DMX receive buffers.
  ARTNET_RcSwitchErr = 0x000Bul,                                                // Rx Universe switches conflict.
  ARTNET_RcConfigErr = 0x000Cul,                                                // DMX512 receive errors detected.
  ARTNET_RcDmxShort = 0x000Dul,                                                 // DMX output short detected. See GoodOutput field.
  ARTNET_RcFirmwareFail = 0x000Eul,                                             // Last attempt to upload new firmware failed
  ARTNET_RcUserFail = 0x000Ful,                                                 // User changed switch settings when address locked by remote programming. User changes ignored
  ARTNET_RcFactoryRes = 0x0010ul                                                // Factory reset has occurred
}__attribute__((packed));
#endif

// The following table details the Style codes. The Style code defines the general functionality of a Controller. 
// The Style code is returned in ArtPollReply.
enum ARTNET_style_code_e {                                                      // these are the valid .style code variables
  ARTNET_StNode = 0x00u,                                                        // A DMX to / from Art-Net device
  ARTNET_StController = 0x01u,                                                  // A lighting console.
  ARTNET_StMedia = 0x02u,                                                       // A Media Server
  ARTNET_StRoute = 0x03u,                                                       // A network routing device
  ARTNET_StBackup= 0x04u,                                                       // backup device
  ARTNET_StConfig = 0x05u,                                                      // A configuration or diagnostic tool.
  ARTNET_StVisual = 0x06u                                                       // A visualiser.
}__attribute__((packed));

struct artnet_ipprog_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t OpCode;                                                              // OpIpProg Transmitted low byte first
  uint8_t  ProVerH;                                                             // High byte of the Art-Net protocol revision number.
  uint8_t  ProVer;                                                              // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  Filler1;                                                             // Pad length to match ArtPoll
  uint8_t  Filler2;                                                             // Pad length to match ArtPoll
  uint8_t  Command;                                                             // Action this packet as below in Commnd
  uint8_t  Filler4;                                                             // Set to zero. Pads data structure for word alignment
  uint8_t  ProgIpHi;                                                            // IP Address to be programmed into Node if enabled by Command Field
  uint8_t  ProgIp2;
  uint8_t  ProgIp1;
  uint8_t  ProgIpLo;
  uint8_t  ProgSmHi;                                                            // Subnet mask to be programmed into Node if enabled by Command Field
  uint8_t  ProgSm2;
  uint8_t  ProgSm1;
  uint8_t  ProgSmLo;
  uint8_t  ProgPortHi;
  uint8_t  ProgPortLo;
  uint8_t  Spare1;
  uint8_t  Spare2;
  uint8_t  Spare3;
  uint8_t  Spare4;
  uint8_t  Spare5;
  uint8_t  Spare6;
  uint8_t  Spare7;
  uint8_t  Spare8;
} __attribute__((packed));

typedef struct artnet_ipprog_s artnet_ipprog_t;

#define ARTNET_IPPRO_CMD_PP (1u<<0u)                                            // prog port
#define ARTNET_IPPRO_CMD_PSM (1u<<1u)                                           // program subnet mask
#define ARTNET_IPPRO_CMD_IPA (1u<<2u)                                           // Program IP Address
#define ARTNET_IPPRO_CMD_DEF (1u<<3u)                                           // Set as default
#define ARTNET_IPPRO_CMD_DHCP (1u<<6u)                                          // Set to enable DHCP
#define ARTNET_IPPRO_CMD_PRG (1u<<7u)                                           // Set to enable any programming.

struct artnet_ipprog_reply_s {
  uint8_t id[8u];                                                               // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t  OpCode;                                                             // OpIpProgReply Transmitted low byte first.
  uint8_t  ProVerH;                                                             // High byte of the Art-Net protocol revision number.
  uint8_t  ProVer;                                                              // Low byte of the Art-Net protocol revision number. (14)
  uint8_t  Filler1;
  uint8_t  Filler2;
  uint8_t  Filler3;
  uint8_t  Filler4;
  uint8_t  ProgIpHi;                                                            // IP Address of Node.
  uint8_t  ProgIp2;
  uint8_t  ProgIp1;
  uint8_t  ProgIpLo;
  uint8_t  ProgSmHi;                                                            // subnet mask of the node
  uint8_t  ProgSm2;
  uint8_t  ProgSm1;
  uint8_t  ProgSmLo;
  uint8_t  ProgPortHi;                                                          // depricated
  uint8_t  ProgPortLo;
  uint8_t  Status;                                                              // Bit 6 DHCP enabled.
  uint8_t  Spare2;
  uint8_t  Spare3;
  uint8_t  Spare4;
  uint8_t  Spare5;
  uint8_t  Spare6;
  uint8_t  Spare7;
  uint8_t  Spare8;
} __attribute__((packed));

typedef struct artnet_ipprog_reply_s artnet_ipprog_reply_t;

struct artnet_address_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpAddress  Transmitted low byte first.
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number.
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  NetSwitch;                                                           // Bits 14-8 of the 15 bit Port-Address are encoded into the bottom 7 bits of this field. This is used in combination with SubSwitch and SwIn[] or SwOut[] to produce the full universe address.  This value is ignored unless bit 7 is high. i.e. to program a  value 0x07, send the value as 0x87. Send 0x00 to reset this value to the physical switch setting. Use value 0x7f for no change
  uint8_t  BindIndex;                                                           // The BindIndex defines the bound node which originated this packet and is used to uniquely identify the bound node when identical IP addresses are in use.  This number represents the order of bound devices. A lower number means closer to root device. A value of 1 means root device
  uint8_t  shortname[ARTNET_SHORT_NAME_LENGTH];                                 // The array represents a null terminated short name for the Node. The Controller uses the ArtAddress packet to program this string. Max length is 17 characters plus the null. The Node will ignore this value if the string is null. This is a fixed length field, although the string it contains can be shorter than the field.
  uint8_t  longname[ARTNET_LONG_NAME_LENGTH];                                   // The array represents a null terminated long name for the Node. The Controller uses the ArtAddress packet to program this string. Max length is 63 characters plus the null. The Node will ignore this value if the string is null. This is a fixed length field, although the string it contains can be shorter than the field.
  uint8_t  swin[ARTNET_MAX_PORTS];                                              // Bits 3-0 of the 15 bit Port-Address for a given input port are encoded into the bottom 4 bits of this field. This is used in combination with NetSwitch and SubSwitch to produce the full universe address.  This value is ignored unless bit 7 is high. i.e. to program a  value 0x07, send the value as 0x87. Send 0x00 to reset this value to the physical switch setting. Use value 0x7f for no change
  uint8_t  swout[ARTNET_MAX_PORTS];                                             // Bits 3-0 of the 15 bit Port-Address for a given output port are encoded into the bottom 4 bits of this field This is used in combination with NetSwitch and SubSwitch to produce the full universe address.  This value is ignored unless bit 7 is high. i.e. to program a  value 0x07, send the value as 0x87. Send 0x00 to reset this value to the physical switch setting. Use value 0x7f for no change
  uint8_t  subnet;                                                              // Bits 7-4 of the 15 bit Port-Address are encoded into the bottom 4 bits of this field. This is used in combination with NetSwitch and SwIn[] or SwOut[] to produce the full universe address.  This value is ignored unless bit 7 is high. i.e. to program a  value 0x07, send the value as 0x87. Send 0x00 to reset this value to the physical switch setting. Use value 0x7f for no change.
  uint8_t  swvideo;
  uint8_t  command;                                                             // command as below ARTNET_ADDR_CMD_(x)
} __attribute__((packed));

typedef struct artnet_address_s artnet_address_t;

#define ARTNET_ADDR_CMD_AcCancelMerge 0x01U                                     // If Node is currently in merge mode, cancel merge mode upon receipt of next ArtDmx packet. See discussion of merge mode operation
#define ARTNET_ADDR_CMD_AcLedNormal 0x02U                                       // The front panel indicators of the Node operate normally.
#define ARTNET_ADDR_CMD_AcLedMute 0x03U                                         // The front panel indicators of the Node are disabled and switched off.
#define ARTNET_ADDR_CMD_AcLedLocate 0x04U                                       // Rapid flashing of the Node’s front panel indicators. It is intended as an outlet identifier for large installations.
#define ARTNET_ADDR_CMD_AcResetRxFlags 0x05U                                    // Resets the Node’s Sip, Text, Test and data error flags. If an output short is being flagged, forces the test to re-run.
#define ARTNET_ADDR_CMD_AcMergeLtp0 0x10U                                       // Set DMX Port 0 to Merge in LTP mode
#define ARTNET_ADDR_CMD_AcMergeLtp1 0x11U                                       // Set DMX Port 1 to Merge in LTP mode
#define ARTNET_ADDR_CMD_AcMergeLtp2 0x12U                                       // Set DMX Port 2 to Merge in LTP mode
#define ARTNET_ADDR_CMD_AcMergeLtp3 0x13U                                       // Set DMX Port 3 to Merge in LTP mode
#define ARTNET_ADDR_CMD_AcMergeHtp0 0x50U                                       // Set DMX Port 0 to Merge in HTP (default) mode
#define ARTNET_ADDR_CMD_AcMergeHtp1 0x51U                                       // Set DMX Port 1 to Merge in HTP (default) mode
#define ARTNET_ADDR_CMD_AcMergeHtp2 0x52U                                       // Set DMX Port 2 to Merge in HTP (default) mode
#define ARTNET_ADDR_CMD_AcMergeHtp3 0x53U                                       // Set DMX Port 3 to Merge in HTP (default) mode
#define ARTNET_ADDR_CMD_AcArtNetSel0 0x60U                                      // Set DMX Port 0 to output both DMX512 and RDM packets from the Art-Net protocol (default)
#define ARTNET_ADDR_CMD_AcArtNetSel1 0x61U                                      // Set DMX Port 1 to output both DMX512 and RDM packets from the Art-Net protocol (default)
#define ARTNET_ADDR_CMD_AcArtNetSel2 0x62U                                      // Set DMX Port 2 to output both DMX512 and RDM packets from the Art-Net protocol (default)
#define ARTNET_ADDR_CMD_AcArtNetSel3 0x63U                                      // Set DMX Port 3 to output both DMX512 and RDM packets from the Art-Net protocol (default)
#define ARTNET_ADDR_CMD_AcAcnSel0 0x70U                                         // Set DMX Port 0 to output DMX512 data from the sACN protocol and RDM data from the Art-Net protocol.
#define ARTNET_ADDR_CMD_AcAcnSel1 0x71U                                         // Set DMX Port 1 to output DMX512 data from the sACN protocol and RDM data from the Art-Net protocol.
#define ARTNET_ADDR_CMD_AcAcnSel2 0x72U                                         // Set DMX Port 2 to output DMX512 data from the sACN protocol and RDM data from the Art-Net protocol.
#define ARTNET_ADDR_CMD_AcAcnSel3 0x73U                                         // Set DMX Port 3 to output DMX512 data from the sACN protocol and RDM data from the Art-Net protocol.
#define ARTNET_ADDR_CMD_AcClearOp0 0x90U                                        // Clear DMX Output buffer for Port 0
#define ARTNET_ADDR_CMD_AcClearOp1 0x91U                                        // Clear DMX Output buffer for Port 1
#define ARTNET_ADDR_CMD_AcClearOp2 0x92U                                        // Clear DMX Output buffer for Port 2
#define ARTNET_ADDR_CMD_AcClearOp3 0x93U                                        // Clear DMX Output buffer for Port 3

// ArtDmx is the data packet used to transfer DMX512 data. The format is identical 
// for Node to Controller, Node to Node and Controller to Node.
struct artnet_dmx_s {
  uint8_t  id[8u];
  uint16_t opCode;
  uint8_t  verH;
  uint8_t  ver;
  uint8_t  sequence;                                                            // The sequence number is used to ensure that ArtDmx packets are used in the correct order. When Art-Net is carried over a medium such as the Internet, it is possible that ArtDmx packets will reach the receiver out of order. This field is incremented in the range 0x01 to 0xff to allow the receiving node to resequence packets. The Sequence field is set to 0x00 to disable this feature
  uint8_t  physical;                                                            // The physical input port from which DMX512 data was input. This field is for information only. Use Universe for data routing
  uint16_t  universe;                                                           // Port-Address to which this packet is destined.
  uint8_t  lengthHi;                                                            // The length of the DMX512 data array. This value should be an even number in the range 2 – 512.  It represents the number of DMX512 channels encoded in packet. NB: Products which convert Art-Net to DMX512 may opt to always send 512 channels
  uint8_t  length;                                                              // Low Byte of above
  uint8_t  dataV[ARTNET_DMX_LENGTH];                                            // A variable length array of DMX512 lighting data.
} __attribute__((packed));

typedef struct artnet_dmx_s artnet_dmx_t;

// At power on or reset a node shall operate in non-synchronous mode. 
// This means that ArtDmx packets will be immediately processed and output.
// When a node receives an ArtSync packet it should transfer to synchronous operation. 
// This means that received ArtDmx packets will be buffered and output when the next ArtSync is received.
// In order to allow transition between synchronous and non-synchronous modes, a node shall time out 
// to non-synchronous operation if an ArtSync is not received for 4 seconds or more.
struct artnet_sync_s {
  uint8_t  id[8u];
  uint16_t opCode;
  uint8_t  verH;
  uint8_t  ver;
  uint8_t  aux1;                                                                // transmit as zero
  uint8_t  aux2;                                                                // transmit as zero
} __attribute__((packed));

typedef struct artnet_sync_s artnet_sync_t;

// A Controller or monitoring device on the network can enable or disable individual
// DMX512 inputs on any of the network nodes. This allows the Controller to directly control 
// network traffic and ensures that unused inputs are disabled and therefore not wasting bandwidth.
struct artnet_input_s {
  uint8_t id[8u];                                                                // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t  opCode;                                                             // OpInput  Transmitted low byte first
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  filler1;                                                             // space filler
  uint8_t  BindIndex;                                                           // The BindIndex defines the bound node which originated this packet and is used to uniquely identify the bound node when identical IP addresses are in use. This number represents the order of bound devices. A lower number means closer to root device. A value of 1 means root device
  uint8_t  numbportsH;                                                          // The high byte of the word describing the number of input or output ports. The high byte is for future expansion and is currently zero
  uint8_t  numbports;                                                           // The low byte of the word describing the number of input or output ports. If number of inputs is not equal to number of outputs, the largest value is taken. The maximum value is 4
  uint8_t  input[ARTNET_MAX_PORTS];                                             // This array defines input disable status of each channel. (Example = 0x01, 0x00, 0x01, 0x00  to disable first and third inputs)
} __attribute__((packed));

typedef struct artnet_input_s artnet_input_t;

// This packet is used to request the Table of RDM Devices (TOD). 
// A Node receiving this packet must not interpret it as forcing full discovery. 
// Full discovery is only initiated at power on or when an ArtTodControl.AtcFlush is received. 
// The response is ArtTodData.
struct artnet_todrequest_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpTodRequest.  Transmitted low byte first
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  filler1;
  uint8_t  filler2;
  uint8_t  spare1;
  uint8_t  spare2;
  uint8_t  spare3;
  uint8_t  spare4;
  uint8_t  spare5;
  uint8_t  spare6;
  uint8_t  spare7;
  uint8_t  Net;                                                                 // The top 7 bits of the 15 bit Port-Address of Nodes that must respond to this packet.
  uint8_t  command;                                                             // 0x00 send the full tod
  uint8_t  adCount;                                                             // The number of entries in Address that are used. Max value is 32.
  uint8_t  address[ARTNET_MAX_RDM_ADCOUNT];                                     // This array defines the low byte of the PortAddress of the Output Gateway nodes that must respond to this packet. The high nibble is the Sub-Net switch. The low nibble corresponds to the Universe. This is combined with the 'Net' field above to form the 15 bit address
} __attribute__((packed));

typedef struct artnet_todrequest_s artnet_todrequest_t;

struct artnet_toddata_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x0
  uint16_t opCode;                                                              // OpTodData.  Transmitted low byte first
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  rdmVer;                                                              // Art-Net Devices that only support RDM DRAFT V1.0 set field to 0x00. Devices that support RDM STANDARD V1.0 set field to 0x01.
  uint8_t  port;                                                                // Physical Port. Range 1-4.
  uint8_t  spare1;
  uint8_t  spare2;
  uint8_t  spare3;
  uint8_t  spare4;
  uint8_t  spare5;
  uint8_t  spare6;
  uint8_t  BindIndex;                                                           // The BindIndex defines the bound node which originated this packet. In combination with Port and Source IP address, it uniquely identifies the sender. This must match the BindIndex field in ArtPollReply. This number represents the order of bound devices. A lower number means closer to root device. A value of 1 means root device.
  uint8_t  Net;                                                                 // The top 7 bits of the Port-Address of the Output Gateway DMX Port that generated this packet
  uint8_t  cmdRes;                                                              // 0x00 TodFull or 0x01 TodNak
  uint8_t  address;                                                             // The low 8 bits of the Port-Address of the Output Gateway DMX Port that generated this packet. The high nibble is the Sub-Net switch. The low nibble corresponds to the Universe
  uint8_t  uidTotalHi;                                                          // The total number of RDM devices discovered by this Universe.
  uint8_t  uidTotal;                                                            // low byte of above
  uint8_t  blockCount;                                                          // The index number of this packet. When UidTotal exceeds 200, multiple ArtTodData packets are used. BlockCount is set to zero for the first packet, and incremented for each subsequent packet containing
  uint8_t  uidCount;                                                            // The number of UIDs encoded in this packet. This is the index of the following array.
  uint8_t  tod[ARTNET_MAX_UID_COUNT][ARTNET_RDM_UID_WIDTH];                     // RDM UID array
} __attribute__((packed));

typedef struct artnet_toddata_s artnet_toddata_t;

//  firware upload
struct artnet_firmware_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpFirmwareMaster.  Transmitted low byte first
  uint8_t  verH;
  uint8_t  ver;
  uint8_t  filler1;
  uint8_t  filler2;
  uint8_t  type;                                                                // type of firmware upgrade as defined below
  uint8_t  blockId;                                                             // Counts the consecutive blocks of firmware upload. Starting at 0x00 for the FirmFirst or UbeaFirst packet.
  uint8_t  length[4];                                                           // total number of words (Int16) in the firmware upload plus the firmware header size. Eg a 32K word upload plus 530 words of header information == 0x00008212. This value is also the file size (in words) of the file to be uploaded
  uint8_t  spare[20];
  uint16_t  dataV[ARTNET_FIRMWARE_SIZE ];                                       // This array contains the firmware or UBEA data block. The order is hi byte first. The interpretation of this data is manufacturer specific. Final packet should be null packed if less than 512 bytes needed.
} __attribute__((packed));

typedef struct artnet_firmware_s artnet_firmware_t;

#define ARTNET_FW_TYPE_FirmFirst 0x00U                                          // The first packet of a firmware upload.
#define ARTNET_FW_TYPE_FirmCont 0x01U                                           // The continuation packet of a firmware upload.
#define ARTNET_FW_TYPE_FirmLast 0x02U                                           // The last packet of a firmware upload.
#define ARTNET_FW_TYPE_UbeaFirst 0x03U                                          // The first packet of a UBEA upload.
#define ARTNET_FW_TYPE_UbeaCont 0x04U                                           // The continuation packet of a UBEA upload.
#define ARTNET_FW_TYPE_UbeaLast 0x05U                                           // The last packet of a UBEA upload.

struct artnet_todcontrol_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpTodControl.  Transmitted low byte first.
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  filler1;
  uint8_t  filler2;
  uint8_t  spare1;
  uint8_t  spare2;
  uint8_t  spare3;
  uint8_t  spare4;
  uint8_t  spare5;
  uint8_t  spare6;
  uint8_t  spare7;
  uint8_t  Net;                                                                 // The top 7 bits of the Port-Address of the Output Gateway DMX Port that should action this command.
  uint8_t  cmd;                                                                 // AtcNone=0 AtcFlush=1 The node flushes its TOD and instigates full discovery
  uint8_t  address;                                                             // The low byte of the 15 bit Port-Address of the DMX Port that should action this command
} __attribute__((packed));

typedef struct artnet_todcontrol_s artnet_todcontrol_t;

// ---------------------- RDM (RemoteDeviceManagement) -------------------------
struct rdm_init_s {
  char *manufacturerLabel;
  uint16_t deviceModelId;
  char *deviceModel;
  uint16_t footprint;
  // uint16_t personalityCount;
  // RDMPERSONALITY *personalities;
  uint16_t  additionalCommandsLength;
  uint16_t  *additionalCommands;
}__attribute__((packed));                                                       // struct RDMINIT

typedef struct rdm_init_s rdm_init_t;

// The RDMDATA structure (length = 24+data) is used by all GET/SET RDM commands.
// The maximum permitted data length according to the spec is 231 bytes.
#define RDM_DATA_MAX 231U
struct rdm_Data_s {
  uint8_t     StartCode;                                                        // Start Code 0xCC for RDM
  uint8_t     SubStartCode;                                                     // Start Code 0x01 for RDM
  uint8_t     Length;                                                           // packet length
  uint8_t     DestID[6u];
  uint8_t     SourceID[6u];
  uint8_t     _TransNo;                                                         // transaction number, not checked
  uint8_t     ResponseType;                                                     // ResponseType
  uint8_t     _unknown;                                                         // I don't know, ignore this
  uint16_t SubDev;                                                              // sub device number (root = 0)
  uint8_t     CmdClass;                                                         // command class
  uint16_t Parameter;                                                                // parameter ID
  uint8_t     DataLength;                                                       // parameter data length in bytes
  uint8_t     Data[RDM_DATA_MAX];                                               // data byte field
} __attribute__((packed));                                                      // struct RDMDATA

typedef struct rdm_Data_s rdm_Data_t;

// The packet structure used to gate the Remote Device Management (RDM) protocol across Art-Net.
struct artnet_rdm_s {
  uint8_t id[8u];                                                               // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t  opCode;                                                             // OpRdm.  Transmitted low byte first.
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number.
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  rdmVer;                                                              // Art-Net Devices that only support RDM DRAFT V1.0 set field to 0x00. Devices that support RDM STANDARD V1.0 set field to 0x01.
  uint8_t  filler2;
  uint8_t  spare1;
  uint8_t  spare2;
  uint8_t  spare3;
  uint8_t  spare4;
  uint8_t  spare5;
  uint8_t  spare6;
  uint8_t  spare7;
  uint8_t  Net;                                                                 // The top 7 bits of 15 bit Port-Address that should action this command.
  uint8_t  cmd;                                                                 // ArProcess=0 Process RDM Packet.
  uint8_t  address;                                                             // The low 8 bits of the Port-Address that should action this command
  uint8_t  dataV[ARTNET_MAX_RDM_DATA];                                          // RDM data packet excluding DMX start code
} __attribute__((packed));

typedef struct artnet_rdm_sub_s artnet_rdm_t;

struct artnet_rdm_sub_s {
  uint8_t id[8u];                                                               // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t  opCode;                                                             // OpRdmSub.  Transmitted low byte first.
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number.
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  rdmVer;                                                              // Art-Net Devices that only support RDM DRAFT V1.0 set field to 0x00. Devices that support RDM STANDARD V1.0 set field to 0x01.
  uint8_t  filler2;
  uint8_t  uid[6u];                                                             // UID of target RDM device
  uint8_t  spare1;
  uint8_t  CommandClass;                                                        // As per RDM specification. This field defines whether this is a Get, Set, GetResponse, SetResponse.
  uint16_t ParameterId;                                                         // As per RDM specification. This field defines the type of parameter contained in this packet. Bigendian.
  uint16_t  SubDevice;                                                          // Defines the first device information contained in packet. This follows the RDM convention that 0 = root device and 1 = first subdevice. Big-endian.
  uint16_t  SubCount;                                                           // The number of sub devices packed into packet. Zero is illegal. Big-endian.
  uint8_t  spare2;
  uint8_t  spare3;
  uint8_t  spare4;
  uint8_t  spare5;
  uint8_t  cmd;                                                                 // ArProcess=0 Process RDM Packet.
  uint16_t  dataV[];                                                            // Packed 16-bit big-endian data. The size of the data array is defined by the contents of CommandClass and SubCount:
} __attribute__((packed));

typedef struct artnet_rdm_sub_s artnet_rdm_sub_t;

// ArtDiagData is a general purpose packet that allows a node or controller to send diagnostics data for display.
struct artnet_diag_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpFirmwareReply.  Transmitted low byte first
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number.
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  filler1;
  uint8_t  Priority;                                                            // The priority of this diagnostic data
  uint8_t  filler2;
  uint8_t  filler3;
  uint8_t  LengthHi;                                                            // length of text array below Hi byte.
  uint8_t  LengthLo;                                                            // length of text array below Lo byte.
  uint8_t  dataV[512u];                                                         // ASCII text array, null terminated. Max length is 512 bytes including the null terminator
} __attribute__((packed));
typedef struct artnet_diag_s artnet_diag_t;

struct artnet_firmware_reply_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpFirmwareReply.  Transmitted low byte first
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number.
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  filler1;
  uint8_t  filler2;
  uint8_t  type;                                                                // Defines the packet contents as follows. Codes are used for both firmware and UBEA.
  uint8_t  spare[21u];
} __attribute__((packed));
typedef struct artnet_firmware_reply_s artnet_firmware_reply_t;

#define ARTNET_FW_TYPE_FirmBlockGood 0x00U                                      // Packet received successfully
#define ARTNET_FW_TYPE_FirmAllGood 0x01U                                        // All packets received okay
#define ARTNET_FW_TYPE_FirmFail 0xFFU                                           // The last packet of a firmware upload failed

// ArtTimeCode allows time code to be transported over the network. 
// The data format is compatible with both longitudinal time code and MIDI time code. 
// The four key types of Film, EBU, Drop Frame and SMPTE are also encoded.
struct artnet_tc_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpFirmwareReply.  Transmitted low byte first
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number.
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  filler1;
  uint8_t  filler2;
  uint8_t  frame;                                                               // Frames time. 0 – 29 depending on mode.
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  uint8_t  type;                                                                // as per type def below
} __attribute__((packed));
typedef struct artnet_tc_s artnet_tc_t;

#define ARTNET_TC_TYPE_Film 0x00U                                               // Film 24fps
#define ARTNET_TC_TYPE_EBU 0x01U                                                // EBU  25fps
#define ARTNET_TC_TYPE_DF 0x02U                                                 // DF 29.7fps
#define ARTNET_TC_TYPE_SMPTE 0x03U                                              // SMPYE 30fps

// The ArtCommand packet is used to send property set style commands. 
// The packet can be unicast or broadcast, the decision being application specific.
struct artnet_cmd_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpFirmwareReply.  Transmitted low byte first
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number.
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t  EstaManHi;                                                           // The ESTA manufacturer code. These codes are used to represent equipment manufacturer. They are assigned by ESTA. This field can be interpreted as two ASCII bytes representing the manufacturer initials.
  uint8_t  EstaManLo;
  uint8_t  LengthHi;                                                            // The length of the text array below. High Byte.
  uint8_t LengthLo;                                                             // low byte
  uint8_t dataV[512u];                                                          // ascii command string max length 512
} __attribute__((packed));
typedef struct artnet_cmd_s artnet_cmd_t;

// The ArtCommand packet is used to send property set style commands.
// The packet can be unicast or broadcast, the decision being application specific.
struct artnet_trigger_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpFirmwareReply.  Transmitted low byte first
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number.
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t filler1;
  uint8_t filler2;
  uint8_t  OemCodeHi;                                                           // The manufacturer code (high byte) of nodes that shall accept this trigger.
  uint8_t  OemCodeLo;
  uint8_t  Key;                                                                 // The Trigger Key.
  uint8_t SubKey;                                                               // The Trigger Sub Key.
  uint8_t dataV[512u];                                                          // The interpretation of the payload is defined by the Key
} __attribute__((packed));
typedef struct artnet_trigger_s artnet_trigger_t;
#define ARTNET_TRIGGER_KeyAscii 0x00U                                           // The SubKey field contains an ASCII character which the receiving device should process as if it were a keyboard press. (Payload not used)
#define ARTNET_TRIGGER_KeyMacro 0x01U                                           // The SubKey field contains the number of a Macro which the receiving device should execute. (Payload not used).
#define ARTNET_TRIGGER_KeySoft 0x02U                                            // The SubKey field contains a soft-key number which the receiving device should process as if it were a soft-key keyboard press. (Payload not used).
#define ARTNET_TRIGGER_KeyShow 0x03U                                            // The SubKey field contains the number of a Show which the receiving device should run. (Payload not used).

// ArtNzs is the data packet used to transfer DMX512 data with non-zero start codes 
// (except RDM). The format is identical for Node to Controller, Node to Node and Controller to Node.
struct artnet_nzs_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpFirmwareReply.  Transmitted low byte first
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number.
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t sequence;                                                             // The sequence number is used to ensure that ArtNzs packets are used in the correct order. When Art-Net is carried over a medium such as the Internet, it is possible that ArtNzs packet The Sequence field is set to 0x00 to disable this feature
  uint8_t StartCode;                                                            // The DMX512 start code of this packet. Must not be Zero or RDM.
  uint8_t  SubUni;                                                              // The low byte of the 15 bit Port-Address to which this packet is destined.
  uint8_t  Net;                                                                 // The top 7 bits of the 15 bit Port-Address to which this packet is destined.
  uint8_t  LengthHi;                                                            // The length of the data array. This value should be a number in the range 1 – 512.  It represents the number of DMX512 channels encoded in packet.  High Byte.
  uint8_t Length;                                                               // Low Byte of above.
  uint8_t dataV[512u];                                                          // Variable length array of DMX lighting data
} __attribute__((packed));
typedef struct artnet_nzs_s artnet_nzs_t;

// ArtVlc is a specific implementation of the ArtNzs packet which is used for the 
// transfer of VLC (Visible Light Communication) data over Art-Net. 
//( The packet’s payload can also be used to transfer VLC over a DMX512 physical layer).
struct vlc_packet_s {
  uint8_t  ManIdHi;                                                             // ManIdHiInt8 - 41 Magic number used to identify this packet
  uint8_t ManIdLo;                                                              // ManIdLoInt8 - 4C Magic number used to identify this packet
  uint8_t SubCode;                                                              // ManIdLoInt8 - 45 Magic number used to identify this packet
  uint8_t Flags;                                                                // Flags as below
  uint8_t TransHi;                                                              // TransHi Int8 - The transaction number is a 16-bit value which allows VLC transactions to be synchronised. A value of 0 indicates the first packet in a transaction. A value of ffff16 indicates the final packet in the transaction. All other packets contain consecutive numbers which increment on each packet and roll over to 1 at fffe16
  uint8_t TransLo;                                                              // lo byte of above
  uint8_t SlotAddrHi;                                                           // SlotAddrHi Int8 - The slot number, range 1-512, of the device to which this packet is directed. A value 0f 0 indicates that all devices attached to this packet’s Port-Address should accept the packet
  uint8_t SlotAddrLo;                                                           // lo byte of above
  uint8_t PayCountHi;                                                           // PayCountHi Int8 - The 16-bit payload size in the range 0 to 48010
  uint8_t PayCountLo;                                                           // lo byte of above
  uint8_t PayCheckHi;                                                           // PayCheckHi Int8 - The 16-bit unsigned additive checksum of the data in the payload
  uint8_t PayCheckLo;                                                           // lo byte of above
  uint8_t Spare1;                                                               // tx as 0x00
  uint8_t VlcDepth;                                                             // VlcDepth Int8 - The 8-bit VLC modulation depth expressed as a percentage in the range 1 to 100. A value of 0 indicates that the transmitter should use its default value
  uint8_t VlcFreqHi;                                                            // The 16-bit modulation frequency of the VLC transmitter expressed in Hz. A value of 0 indicates that the transmitter should use its default value
  uint8_t VlcFreqLo;                                                            // lo byte as above
  uint8_t VlcModHi;                                                             // VlcModHi Int8 - The 16-bit modulation type number that the transmitter should use to transmit VLC. 000016 – Use transmitter default
  uint8_t VlcModLo;                                                             // lo byte as above
  uint8_t PayLangHi;                                                            // PayLangHi Int8 - The 16-bit payload language code. Currently registered values:
  uint8_t PayLangLo;                                                            // low byte of above
  uint8_t BeacRepHi;                                                            // BeacRepHi Int8 - The 16-bit beacon mode repeat frequency. If Flags.Beacon is set, this 16-bit value indicates the frequency in Hertz at which the VLC packet should be repeated.  000016 – Use transmitter default.
  uint8_t BeacRepLo;                                                            // low byte for above
  uint8_t VLCData[];                                                            // Variable length array of VLC data
} __attribute__((packed));
typedef struct vlc_packet_s vlc_packet_t;

#define ARTNET_VLC_FLAG_BEACON (1u<<5u)                                         // If set, the transmitter should continuously repeat transmission of this packet until another is received. If clear, the transmitter should transmit this packet once.
#define ARTNET_VLC_FLAG_REPLY (1u<<6u)                                          // If set this is a reply packet that is in response to the request sent with matching number in the transaction number: TransHi/Lo. If clear this is not a reply.
#define ARTNET_VLC_FLAG_IEEE (1u<<7u)                                           // If set, data in the payload area shall be interpreted as IEEE VLC data. If clear, PayLanguage defines the payload contents
#define ARTNET_VLC_LANG_URL 0U                                                  // BeaconURL – Payload contains a simple text string representing a URL.
#define ARTNET_VLC_LANG_TXT 1U                                                  // BeaconText – Payload contains a simple ASCII text message

struct artnet_vlc_s {
  uint8_t  id[8u];                                                              // Array of 8 characters, the final character is a null termination. Value = ‘A’ ‘r’ ‘t’ ‘-‘ ‘N’ ‘e’ ‘t’ 0x00
  uint16_t opCode;                                                              // OpFirmwareReply.  Transmitted low byte first
  uint8_t  verH;                                                                // High byte of the Art-Net protocol revision number.
  uint8_t  ver;                                                                 // Low byte of the Art-Net protocol revision number. Current value 14
  uint8_t sequence;                                                             // The sequence number is used to ensure that ArtNzs packets are used in the correct order. When Art-Net is carried over a medium such as the Internet, it is possible that ArtNzs packet The Sequence field is set to 0x00 to disable this feature
  uint8_t StartCode;                                                            // The DMX512 start code of this packet. Must not be Zero or RDM.
  uint8_t  SubUni;                                                              // The low byte of the 15 bit Port-Address to which this packet is destined.
  uint8_t  Net;                                                                 // The top 7 bits of the 15 bit Port-Address to which this packet is destined.
  uint8_t  LengthHi;                                                            // The length of the data array. This value should be a number in the range 1 – 512.  It represents the number of DMX512 channels encoded in packet.  High Byte.
  uint8_t Length;                                                               // Low Byte of above.

  vlc_packet_t VLCData[];                                                         // Variable length array of VLC data
} __attribute__((packed));
typedef struct artnet_vlc_s artnet_vlc_t;

typedef union {
  artnet_poll_t ap;
  artnet_reply_t ar;
  artnet_ipprog_t aip;
  artnet_address_t addr;
  artnet_dmx_t admx;
  artnet_input_t ainput;
  artnet_todrequest_t todreq;
  artnet_toddata_t toddata;
  artnet_firmware_t firmware;
  artnet_firmware_reply_t firmwarer;
  artnet_todcontrol_t todcontrol;
  artnet_rdm_t rdm;
  artnet_rdm_sub_t rdm_sub;
  artnet_diag_t diag;
  artnet_tc_t timecode;
  artnet_cmd_t command;
  artnet_trigger_t trigger;
  artnet_nzs_t nzs;
  artnet_vlc_t vlc;
} artnet_packet_union_t;                                                        /* union of all artnet packets */

typedef struct {
  int length;
  struct in_addr from;
  struct in_addr to;
  artnet_packet_type_t type;
  artnet_packet_union_t dataV;
} artnet_packet_t;                                                              /* a packet, containing data, length, type and a src/dst address */

typedef artnet_packet_t *artnet_packet;

// Open Pixel Control OPC
// Open Pixel Control (OPC) is a simple protocol for controlling arrays of RGB lights, such as Total Control Lighting LEDs.

// Set 8-bit pixel colours (command 0): The data block contains 8-bit RGB values: three bytes in red, green, blue 
// order for each pixel to set. If the data block has length 3*n, then the first n pixels of the specified channel are set. 
// All other pixels are unaffected and retain their current colour values. If the data length is not a multiple of 3, 
// or there is data for more pixels than are present, the extra data is ignored. (Because the maximum data length is 65535, 
// this command can control a maximum of 21845 pixels per channel, or a maximum of 5570475 pixels on all 255 channels.)
#define OPC_CMD_SET_8bitColor 0U
// Set 16-bit pixel colours (command 2): The data block contains 16-bit RGB values: six bytes for each pixel to set, 
// consisting of three 16-bit words in red, green, blue order with each two-byte word in big-endian order. 
// If the data block has length 6*n, then the first n pixels of the specified channel are set. 
// All other pixels are unaffected and retain their current colour values. 
// If the data length is not a multiple of 6, or there is data for more pixels than are present, 
// the extra data is ignored. (Because the maximum data length is 65535, this command can control a maximum of 10922 pixels per channel,
// or a maximum of 2785110 pixels on all 255 channels.)
#define OPC_CMD_SET_16bitColor 2U
// System exclusive (command 255): Command 0xff is used to send a message that is specific to a particular device or software system. 
// The data block should begin with a two-byte system ID; designers of that system are then free to define any message format for the rest of the data block.
#define OPC_CMD_SET_SYSX 0xFFU

#define OPC_STREAM_SYNC_LENGTH 4u
#define OPC_STREAM_SYNC_DATA ((uint8_t *) "\0xf0u\0xcau\0x71u\0x2eu")
#define OPC_MAX_SINKS 64u                                                       /* Maximum number of OPC sinks or sources allowed */
#define OPC_MAX_SOURCES 64u
#define OPC_MAX_PIXELS_PER_MESSAGE ((1u << 16u) / 3u)                           /* Maximum number of pixels in one message */

struct opc_data_s {
  uint8_t channel;                                                              // channel
  uint8_t command;                                                              // command as above
  uint16_t length;                                                              // length
  uint8_t dataV[];                                                              // data bytes
} __attribute__((packed));
typedef struct opc_data_s opc_data_t;

// DMX-512  (250 kbaud - 512 channels) Standard USITT DMX-512
#define DMX512_STX_LIGHT 0x00U                                                  // start transmission for lighting  Default Null Start Code for Dimmers per DMX512 & DMX512/1990
#define DMX512_STX_MSB_DPT  0x01U                                               // Most significant Byte of double precision transmission or Soundlight
#define DMX512_STX_FP_16BITW  0x021U                                            // Following packet is 256 16-Bi words in Lo byte/Hi byte order
#define DMX512_STX_R_A_Gray 0x03U                                               // manufacturer specific RA gray or white rabbit co
#define DMX512_STX_Checksum 0x04U                                               // T_Recursive
#define DMX512_STX_Answerback 0x05U                                             // T_Recursive
#define DMX512_STX_16bitLow 0x06U                                               // T_Recursive
#define DMX512_STX_COMP_DATA 0x07U                                              // T_Recursive
#define DMX512_STX_COMP_16BIT 0x08U                                             // T_Recursive
#define DMX512_STX_EntTech 0x09U                                                // Entertainment Technology
#define DMX512_STX_MODE_LIGHT 0x0AU                                             // Mode Lighting Second universe of 512 channels on one data link
#define DMX512_STX_GODDARD 0x0BU                                                // Goddard Design
#define DMX512_STX_SGM 0x0CU                                                    // SGM
#define DMX512_STX_ENG_ART 0x0DU                                                // Engineering Arts
#define DMX512_STX_CETronics 0x0EU                                              // CE Tronics
#define DMX512_STX_MORPH 0x0FU                                                  // Morpheus Lights
#define DMX512_STX_MPAS1 0xBBU                                                  // Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
#define DMX512_STX_MPAS2 0xCBU                                                  // Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
#define DMX512_STX_MPAS3 0xDEU                                                  // Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
#define DMX512_STX_MPAS4 0xDFU                                                  // Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
#define DMX512_STX_MPAS5 0xE0U                                                  // Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
#define DMX512_STX_MPAS6 0xEDh                                                  // Download Dimmer Information Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
#define DMX512_STX_AVO_DCS 0xFFU                                                // AVO Dimmer Curve Select
#define DMX512_STX_DCD 0xEHU                                                    // e:cue control GmbH Device configuration data
#define DMX512_STX_NSI_DCD 0xE1U                                                // NSI colortran Dim/Non-Dim Control
#define DMX512_STX_NSI_ENR 0xE0U                                                // NSI colortran ENR Mode Control
#define DMX512_STX_EDC_PRIO 0xDDU                                               // Alternate start code DD is for use in transmitting per channel priority for use in merging streams in multi-source DMX applications. Priorities will range from 0 at the low end, which means do not use the data in the corresponding slot, to 200, which means use this data over any slot data supplied with a priority between 0 and 199. Values above 200 are reserved for future use.
#define DMX512_STX_E1_SIP 0xD0U                                                 // E1 system info packet
#define DMX512_STX_E1_RDM 0xCCU                                                 // E1 E1.20 (RDM) start code
#define DMX512_STX_SUN_SPECIAL 0xAAU                                            // Sun
#define DMX512_STX_PLASA_SPECIAL 0x92U                                          // BSR E1.45 Alternate START Code
#define DMX512_STX_E1_MAN_ID 0x91U                                              // 2-byte Manufacturer ID serves as an identifier that the data following in that packet is proprietary to that entity and should be ignored by all others
#define DMX512_STX_PLASA_UTF8 0x90U                                             // utf8 packet
#define DMX512_STX_MP_SPECIAL 0x8BU                                             // Martin Professional A/S
#define DMX512_STX_CLAY_PAKY_SPECIAL 0x8AU                                      // CLAY PAKY S.p.A
#define DMX512_STX_ANYTRON_SYNC 0x83U                                           // To synchronise both the memory contents and the internal clocks of lighting control equipment. Min packet length 24 bytes. Max 512.
#define DMX512_STX_WYBRON_SPECIAL 0x57U                                         // Wybron, Inc.
#define DMX512_STX_E1_TEST 0x55U                                                // E1 Test
#define DMX512_STX_LP_SPECIAL 0x50U                                             // LightProcessor Ltd
#define DMX512_STX_OSCAR_BACKUP 0x4FU                                           // Oscar Lighting AB Backup States
#define DMX512_STX_AVO_SPECIAL 0x4DU                                            // Avolites Ltd.Proprietary function with ART2000 products
#define DMX512_STX_LUX_STX 0x4CU                                                // 4Ch is the START Code used for OpenDMX messages, a protocol developed for use on LUX Italia products, and published by them for royalty-free use by anyone. Visit the LUX Italia website (http://www.luxitalia.eu) for a copy of the specification.
#define DMX512_STX_ENFIS_SPECIAL 0x48U                                          // ASC is used for passing proprietary data for applications such as factory test, configuration, and software update.
#define DMX512_STX_GVA_CONFIG 0x47U                                             // Manufacturer-specific configuration data and remote control
#define DMX512_STX_COEMAR_SPECIAL 0x44U                                         // Coemar Spa
#define DMX512_STX_CITY_SPECIAL 0x43U                                           // Purpose: firmware updates and product configuration
#define DMX512_STX_LSC_SPECIAL 0x42U                                            // Proprietary remote peripheral control
#define DMX512_STX_MICROLITE_SPECIAL 0x41U                                      // mircolite
#define DMX512_STX_SAND_SPECIAL 0x3FU                                           // SAND
#define DMX512_STX_AVAB_SPECIAL 0x3EU                                           // AVAB spsecific info
#define DMX512_STX_AVAB_SMART 0x3DU                                             // AVAB Smart 16 Bit Format
#define DMX512_STX_AVAB_INTERNAL 0x3CU                                          // AVAB Internal Functions
#define DMX512_STX_TIR_SPECIAL 0x35U                                            // Programmable DMX512-based LED controllers. Alternate START code used to specify various operating parameters for DMX512 network and standalone operation. Min.frames: 15; Max frames: 40
#define DMX512_STX_TESI_SPECIAL 0x33U                                           // The start code's purpose is to send/receive application-specific information and execute product software update
#define DMX512_STX_PR_FIRMWARE 0x30U                                            // The Start Code purpose for now is to be able to perform firmware updates to our products. In the future we might add more functions to it.
#define DMX512_STX_JOHN_SPECIAL 0x2AU                                           // johnson
#define DMX512_STX_HES_SPECIAL 0x26U                                            // high end systems
#define DMX512_STX_GDS_SPECIAL 0x22U                                            // gds
#define DMX512_STX_ELETRO_SPECIAL 0x21U                                         // eletro lab
#define DMX512_STX_RJULIAT_SPECIAL 0x1EU                                        // Robert Juliat Used to update old products which do not have RDM capabilities but still supported. Also, to modify remotely some factory settings.
#define DMX512_STX_DROSS_SPECIAL 0x1DU                                          // Dangeross Design
#define DMX512_STX_KLH_RTC 0x1CU                                                // The alternate byte is to provide real-time updates for triple precision data ( either 20 or 24 bit ) for use with Photon Cannon project. This will allow the use of the standard to control 100 lamp fixtures at greater than 75 Hz update rate
#define DMX512_STX_ESCAPE_SPECIAL 0x1BU                                         // The purpose is to toggle inner program of the receiver. If the Start code is 0, the machine answers with Program 1. If the Start code is 27, the machine answers with Program 2. Rest of the DMX trame would remain exactly the same as in USITT description. The purpose for us is to control different types of machines with the same DMX values (Program 2) using a switch box sending Start Code 27 and predetermined DMX values. The program run by Start Code 0 would be adapted to a fader control.
#define DMX512_STX_IT_SPECIAL 0x1AU                                             // integrated theatre
#define DMX512_STX_HUB_SPECIAL 0x19U                                            // hubbell
#define DMX512_STX_AND_SPECIAL 0x18U                                            // andera
#define DMX512_STX_E1_TEXT 0x17U                                                // E1 ANSI E1.11 Text Packet
#define DMX512_STX_AL_TEXT 0x17U                                                // Artistic License Text Packet (matches use in ANSI E1.11)
#define DMX512_STX_PER_SPECIAL 0x16U                                            // Peradise We build specialFx and moving set parts. The startcode will be used to identify data that is used for tacticle feedback from devices (Position, status, errors, etc)
#define DMX512_STX_CDCA_CONFIG 0x15U                                            // Firmware update and configuration info
#define DMX512_STX_SSI_SPECIAL 0x14U                                            // We are implementing a message-based protocol that is optimized for safe and secure motor control, embedded with lighting control. A unique Start Code is the ideal way to identify this alternate data type on the DMX network, while keeping packets as short as possible.
#define DMX512_STX_ZERO88_SPECIAL 0x13U                                         // zero 88
#define DMX512_STX_BJA_FIRMWARE 0x12U                                           // For updating the firmware in my equipment and to control it (Reset). My packet size will between the 3 and 256 bytes.
#define DMX512_STX_TBS_SPECIAL 0x11U                                            // Tokyo Broadcast systems
#define DMX512_STX_ADB_SPECIAL 0x10U                                            // ADB
#define DMX512_STX_ELDO_FIRM 0xDOU                                              // eldo LED Configuration, firmware updates and standalone configuration. Framelength to vary from 12 to 512 bytes.

#define DMX512_TXT_PACKET 0x17U                                                 // text packet
#define DMX512_RDM_PKT 0xCCU                                                    // remote device management packet
#define DMX512_SYS_INFO 0xCFU                                                   // system information packet

#define DMX512_ADDR_RED(A) { A }
#define DMX512_ADDR_GREEN(A) { A+1 }
#define DMX512_ADDR_BLUE(A) { A+2 }

#define DMX512_MAX_ARRAY 512U                                                   // define max size of matrix for DMX512 (max channels)

// DMX-1024 (500 kbaud - 1024 channels) Completely non standard, but usefull ;)
// DMX-2048 (1000 kbaud - 2048 channels) Used by manufacturers as DMX1000K, DMX-4x or DMX-1M ???
/* DMX Channel-Attribute Allocations
   for ADB CC/Warp  (channel map)      */
#define ADB_CCWARP_ColorFrame 1U                                                /* Colour / Frame */
#define ADB_CCWARP_Fan 2U                                                       /* Fan  */
#define ADB_CCWARP_Reset 3U                                                     /* Reset*/
/* DMX Channel-Attribute Allocations
   for ADB MD/Warp     */
#define ADB_MDWARP_Shut8 1U                                                     /* Shutter 8 bit */
#define ADB_MDWARP_Shut16 2U                                                    /* Shutter 16 bit */
/* DMX Channel-Attribute Allocations
   for ADB Combo Warp    */
#define ADB_COWARP_GelPos 1U                                                    /* Gel position   */
#define ADB_COWARP_GelSpeed 2U                                                  /* Gel Speed      */
#define ADB_COWARP_Fan 3U                                                       /* Fan Speed    */
#define ADB_COWARP_Shut8 4U                                                     /* Shutter 8 bit  */
#define ADB_COWARP_Shut16 5U                                                    /* Shutter 16 bit  */

#ifdef __cplusplus
}
#endif

#endif