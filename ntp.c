/* -----------------------------------------------------------------------------
   %%%%%%%%%%%%%%%%%%% NTP client for time sync to a ntp server %%%%%%%%%%%%%%%

    Written By ACP Aviation

   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
#include <stdint.h>
#include "definitions.h"
#include "ntp.h"

/*-----------------------------------------------------------------------------
 *      requestNtp() :  Send Request to ntp server for time
 *
 *  Parameters: (none)
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
void requestNtp()
{
  ntp_packet_t ntpPacket = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };     // Create and zero out the packet. All 48 bytes worth.
  memset( &ntpPacket, 0, sizeof( ntp_packet_t ) );
  ntpPacket.li_vn_mode = 0x1Bu;                                                 // Represents 27 in base 10 or 00011011 in base 2.
}