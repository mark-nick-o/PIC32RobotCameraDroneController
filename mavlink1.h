#ifndef MAVLINK1_H
#define MAVLINK1_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//#pragma once
/** @file
 *  @brief MAVLink comm protocol built from common.xml
 *  @see http://mavlink.org
 */
#define MAVLINK_PRIMARY_XML_IDX 1u

#ifndef MAVLINK_STX
#define MAVLINK_STX 254u
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 1
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

#ifndef MAVLINK_COMMAND_24BIT
#define MAVLINK_COMMAND_24BIT 0
#endif

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#if defined(MAVLINK_USE_CONVENIENCE_FUNCTIONS)
#define MAVLINK_SEND_UART_BYTES                                                 /* system allows muliple bytes to be sent */
#define MAV_SYS_ID 1u                                                           /* youre system ID */
#define MAV_COMP_ID 2u                                                          /* youre component ID */
#endif   /* end mav convienience sending functions */

//#include "version.h"
#include "mavlink1_common.h"

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // MAVLINK_H