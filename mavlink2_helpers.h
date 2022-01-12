//#pragma once

//#include "string.h"
#include "mavlink_crc.h"
#include "mavlink2_msg_types.h"
#include "mavlink_conversions.h"
#include "gc_events.h"
//#include <stdio.h>

#ifndef MAVLINK2_HELPER__
#define MAVLINK2_HELPER__

//#endif
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "mavlink2_sha256.h"

//#ifdef MAVLINK_USE_CXX_NAMESPACE
//namespace mavlink {
//#endif

/*
 * Internal function to give access to the channel status for each channel
 */
#ifndef MAVLINK_GET_CHANNEL_STATUS
MAVLINK_HELPER mavlink_status_t* mavlink_get_channel_status(uint8_t chan)
{
#ifdef MAVLINK_EXTERNAL_RX_STATUS
        // No m_mavlink_status array defined in function,
        // has to be defined externally
#else
        static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
#endif
        return &m_mavlink_status[chan];
}
#endif

/*
 * Internal function to give access to the channel buffer for each channel
 */
#ifndef MAVLINK_GET_CHANNEL_BUFFER
MAVLINK_HELPER mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan)
{

#ifdef MAVLINK_EXTERNAL_RX_BUFFER
        // No m_mavlink_buffer array defined in function,
        // has to be defined externally
#else
        static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
#endif
        return &m_mavlink_buffer[chan];
}
#endif // MAVLINK_GET_CHANNEL_BUFFER

/**
 * @brief Reset the status of a channel.
 */
MAVLINK_HELPER void mavlink_reset_channel_status(uint8_t chan)
{
        mavlink_status_t *status = mavlink_get_channel_status(chan);
        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
}

/**
 * @brief create a signature block for a packet
 */
MAVLINK_HELPER uint8_t mavlink_sign_packet(mavlink_signing_t *signing,
                                           uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN],
                                           const uint8_t *header, uint8_t header_len,
                                           const uint8_t *packet, uint8_t packet_len,
                                           const uint8_t crc[2u])
{


        mavlink_sha256_ctx ctx;
        union {
            uint64_t t64;
            uint8_t t8[8u];
        } tstamp;
        if (signing == NULL || !(signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING)) 
        {
            return 0u;
        }
        signature[0u] = signing->link_id;
        tstamp.t64 = signing->timestamp;
        memcpy(&signature[1u], tstamp.t8, 6);
        signing->timestamp=++signing->timestamp % UINT64_MAX;

        mavlink_sha256_init(&ctx);
        mavlink_sha256_update(&ctx, signing->secret_key, sizeof(signing->secret_key));
        mavlink_sha256_update(&ctx, header, header_len);
        mavlink_sha256_update(&ctx, packet, packet_len);
        mavlink_sha256_update(&ctx, crc, 2);
        mavlink_sha256_update(&ctx, signature, 7);
        mavlink_sha256_final_48(&ctx, &signature[7]);

        return MAVLINK_SIGNATURE_BLOCK_LEN;
}

/**
 * @brief Trim payload of any trailing zero-populated bytes (MAVLink 2 only).
 *
 * @param payload Serialised payload buffer.
 * @param length Length of full-width payload buffer.
 * @return Length of payload after zero-filled bytes are trimmed.
 */
MAVLINK_HELPER uint8_t _mav_trim_payload(const char *payload, uint8_t length)
{
        while (length > 1u && payload[length-1] == 0u) 
        {
                length--;
        }
        return length;
}

/**
 * @brief check a signature block for a packet
 */
MAVLINK_HELPER bool mavlink_signature_check(mavlink_signing_t *signing, mavlink_signing_streams_t *signing_streams, const mavlink_message_t *msg)
{
        const uint8_t *p = (const uint8_t *)&msg->magic;
        const uint8_t *psig = msg->signature;
        const uint8_t *incoming_signature = psig+7u;                            /* should it not be 6 ? */
        mavlink_sha256_ctx ctx;
        uint8_t signature[6u];
        uint16_t i;
        union tstamp
        {
            uint64_t t64;
            uint8_t t8[8u];
        } tstamp;
        uint8_t link_id;
        union tstamp last_tstamp;

        if (signing == NULL) 
        {
           return true;
        }
        mavlink_sha256_init(&ctx);
        mavlink_sha256_update(&ctx, signing->secret_key, sizeof(signing->secret_key));
        mavlink_sha256_update(&ctx, p, MAVLINK_CORE_HEADER_LEN+1+msg->len);
        mavlink_sha256_update(&ctx, msg->ck, 2u);
        mavlink_sha256_update(&ctx, psig, 1+6u);
        mavlink_sha256_final_48(&ctx, signature);
        if (memcmp((void*) signature,(void*) incoming_signature,(int16_t) 6u) != 0u)
        {
           return false;
        }

        link_id = psig[0u];                                                     // now check timestamp
        tstamp.t64 = 0u;
        memcpy((void*) tstamp.t8,(void*) psig+1u,(int16_t) 6u);
        if (signing_streams == NULL) 
        {
           return false;
        }

        for (i=0u; i<signing_streams->num_signing_streams; i++)                  // find stream
        {
           if (msg->sysid == signing_streams->stream[i].sysid && msg->compid == signing_streams->stream[i].compid && link_id == signing_streams->stream[i].link_id)
           {
               break;
           }
        }
        if (i == signing_streams->num_signing_streams) 
        {
           if (signing_streams->num_signing_streams >= MAVLINK_MAX_SIGNING_STREAMS)
           {
               return false;                                                    // over max number of streams
           }

           if (tstamp.t64 + 6000LL*1000LL < signing->timestamp)                 // new stream. Only accept if timestamp is not more than 1 minute old
           {
               return false;
           }
           signing_streams->stream[i].sysid = msg->sysid;                       // add new stream
           signing_streams->stream[i].compid = msg->compid;
           signing_streams->stream[i].link_id = link_id;
           signing_streams->num_signing_streams=++signing_streams->num_signing_streams % UINT16_MAX;
        } 
        else 
        {
            last_tstamp.t64 = 0LL;                                              // union tstamp last_tstamp;
            memcpy(last_tstamp.t8, signing_streams->stream[i].timestamp_bytes, 6u);
            if (tstamp.t64 <= last_tstamp.t64)
            {
                return false;                                                   // repeating old timestamp
            }
        }

        memcpy((void*)signing_streams->stream[i].timestamp_bytes,(void*) psig+1,(int16_t) 6u);         // remember last timestamp

        if (tstamp.t64 > signing->timestamp)                                    // our next timestamp must be at least this timestamp
        {
            signing->timestamp = tstamp.t64;
        }
        return true;
}


/**
 * @brief Finalize a MAVLink message with channel assignment
 *
 * This function calculates the checksum and sets length and aircraft id correctly.
 * It assumes that the message id and the payload are already correctly set. This function
 * can also be used if the message header has already been written before (as in mavlink_msg_xxx_pack
 * instead of mavlink_msg_xxx_pack_headerless), it just introduces little extra overhead.
 *
 * @param msg Message to finalize
 * @param system_id Id of the sending (this) system, 1-127
 * @param length Message length
 */
MAVLINK_HELPER uint16_t mavlink_finalize_message_buffer(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                                                      mavlink_status_t* status, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
        bool mavlink1 = (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0u;
        bool signing =         (!mavlink1) && status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING);
        uint8_t signature_len = signing? MAVLINK_SIGNATURE_BLOCK_LEN : 0u;
        uint8_t header_len = MAVLINK_CORE_HEADER_LEN+1u;
        uint8_t buf[MAVLINK_CORE_HEADER_LEN+1u];
        uint16_t checksum;
        
        if (mavlink1) {
                msg->magic = MAVLINK_STX_MAVLINK1;
                header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN+1u;
        } else {
                msg->magic = MAVLINK_STX;
        }
        msg->len = mavlink1?min_length:_mav_trim_payload(_MAV_PAYLOAD(msg), length);
        msg->sysid = system_id;
        msg->compid = component_id;
        msg->incompat_flags = 0u;
        if (signing) {
                msg->incompat_flags |= MAVLINK_IFLAG_SIGNED;
        }
        msg->compat_flags = 0u;
        msg->seq = status->current_tx_seq;
        status->current_tx_seq = status->current_tx_seq + 1u;

        // form the header as a byte array for the crc
        buf[0u] = msg->magic;
        buf[1u] = msg->len;
        if (mavlink1) 
        {
                buf[2u] = msg->seq;
                buf[3u] = msg->sysid;
                buf[4u] = msg->compid;
                buf[5u] = msg->msgid & 0xFFU;
        } 
        else 
        {
                buf[2u] = msg->incompat_flags;
                buf[3u] = msg->compat_flags;
                buf[4u] = msg->seq;
                buf[5u] = msg->sysid;
                buf[6u] = msg->compid;
                buf[7u] = msg->msgid & 0xFFU;
                buf[8u] = (msg->msgid >> 8U) & 0xFFU;
                buf[9u] = (msg->msgid >> 16U) & 0xFFU;
        }

        checksum = crc_calculate(&buf[1u], header_len-1);
        crc_accumulate_buffer(&checksum, _MAV_PAYLOAD(msg), msg->len);
        crc_accumulate(crc_extra, &checksum);
        mavlink_ck_a(msg) = (uint8_t)(checksum & 0xFFU);
        mavlink_ck_b(msg) = (uint8_t)(checksum >> 8U);

        msg->checksum = checksum;

        if (signing) 
        {
            mavlink_sign_packet(status->signing,
                                msg->signature,
                                (const uint8_t *)buf, header_len,
                                (const uint8_t *)_MAV_PAYLOAD(msg), msg->len,
                                (const uint8_t *)_MAV_PAYLOAD(msg)+(uint16_t)msg->len);
        }

        return msg->len + header_len + 2u + signature_len;
}

MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                                                      uint8_t chan, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
        mavlink_status_t *status = mavlink_get_channel_status(chan);
        return mavlink_finalize_message_buffer(msg, system_id, component_id, status, min_length, length, crc_extra);
}

/**
 * @brief Finalize a MAVLink message with MAVLINK_COMM_0 as default channel
 */
MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                                                 uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
    return mavlink_finalize_message_chan(msg, system_id, component_id, MAVLINK_COMM_0, min_length, length, crc_extra);
}

static inline void _mav_parse_error(mavlink_status_t *status)
{
    status->parse_error=++status->parse_error % UINT8_MAX;
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len);

/**
 * @brief Finalize a MAVLink message with channel assignment and send
 */
MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint32_t msgid, const char *packet, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
        uint16_t checksum;
        uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
        uint8_t ck[2u];
        mavlink_status_t *status = mavlink_get_channel_status(chan);
        uint8_t header_len = MAVLINK_CORE_HEADER_LEN;
        uint8_t signature_len = 0u;
        uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
        bool mavlink1 = (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0u;
        bool signing =  (!mavlink1) && status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING);
        uint8_t incompat_flags = 0u;
        mavlink_system_t mavlink_system;                                        // make this a volatile global if you want it
        
        mavlink_system.sysid = MAV_SYS_ID;                                      // make this a volatile global if you want it or use this global define
        mavlink_system.compid = MAV_COMP_ID;
        
        if (mavlink1) 
        {
            length = min_length;
            if (msgid > 255u)
            {
                _mav_parse_error(status);                                       // can't send 16 bit messages
                return;
            }
            header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN;
            buf[0u] = MAVLINK_STX_MAVLINK1;
            buf[1u] = length;
            buf[2u] = status->current_tx_seq;
            buf[3u] = mavlink_system.sysid;
            buf[4u] = mavlink_system.compid;
            buf[5u] = msgid & 0xFFU;
        } 
        else 
        {
            if (signing) 
            {
                incompat_flags |= MAVLINK_IFLAG_SIGNED;
            }
            length = _mav_trim_payload(packet, length);
            buf[0u] = MAVLINK_STX;
            buf[1u] = length;
            buf[2u] = incompat_flags;
            buf[3u] = 0u;                                                       /* compat_flags  */
            buf[4u] = status->current_tx_seq;
            buf[5u] = mavlink_system.sysid;
            buf[6u] = mavlink_system.compid;
            buf[7u] = msgid & 0xFFU;
            buf[8u] = (msgid >> 8U) & 0xFFU;
            buf[9u] = (msgid >> 16U) & 0xFFU;
        }
        status->current_tx_seq=++status->current_tx_seq % UINT8_MAX;
        checksum = crc_calculate((const uint8_t*)&buf[1u], header_len);
        crc_accumulate_buffer(&checksum, packet, length);
        crc_accumulate(crc_extra, &checksum);
        ck[0u] = (uint8_t)(checksum & 0xFFU);
        ck[1u] = (uint8_t)(checksum >> 8U);

        if (signing) 
        {
           /* possibly add a signature  */
           signature_len = mavlink_sign_packet(status->signing, signature, buf, header_len+1,(const uint8_t *)packet, length, ck);
        }

        MAVLINK_START_UART_SEND(chan, header_len + 3u + (uint16_t)length + (uint16_t)signature_len);
        _mavlink_send_uart(chan, (const char *)buf, header_len+1);
        _mavlink_send_uart(chan, packet, length);
        _mavlink_send_uart(chan, (const char *)ck, 2u);
        if (signature_len != 0u) 
        {
           _mavlink_send_uart(chan, (const char *)signature, signature_len);
        }
        MAVLINK_END_UART_SEND(chan, header_len + 3u + (uint16_t)length + (uint16_t)signature_len);
}

/**
 * @brief re-send a message over a uart channel
 * this is more stack efficient than re-marshalling the message
 * If the message is signed then the original signature is also sent
 */
MAVLINK_HELPER void _mavlink_resend_uart(mavlink_channel_t chan, const mavlink_message_t *msg)
{
        uint8_t ck[2u];
        uint8_t buf[MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1u];                     /* mav version 1.0 */
        uint8_t header_len;
        uint8_t signature_len;
        uint8_t buf2[MAVLINK_CORE_HEADER_LEN + 1u];                             /* mav version 2.0 */
        
        ck[0u] = (uint8_t)(msg->checksum & 0xFFU);
        ck[1u] = (uint8_t)(msg->checksum >> 8U);
        // XXX use the right sequence here

        if (msg->magic == MAVLINK_STX_MAVLINK1) 
        {
            header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1U;
            signature_len = 0u;
            MAVLINK_START_UART_SEND(chan, header_len + msg->len + 2U + signature_len);
            // we can't send the structure directly as it has extra mavlink2 elements in it
            buf[0u] = msg->magic;
            buf[1u] = msg->len;
            buf[2u] = msg->seq;
            buf[3u] = msg->sysid;
            buf[4u] = msg->compid;
            buf[5u] = msg->msgid & 0xFFU;
            _mavlink_send_uart(chan, (const char*)buf, header_len);
        } 
        else 
        {
            header_len = MAVLINK_CORE_HEADER_LEN + 1u;
            signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED)?MAVLINK_SIGNATURE_BLOCK_LEN:0u;
            MAVLINK_START_UART_SEND(chan, header_len + msg->len + 2u + signature_len);
            buf2[0u] = msg->magic;
            buf2[1u] = msg->len;
            buf2[2u] = msg->incompat_flags;
            buf2[3u] = msg->compat_flags;
            buf2[4u] = msg->seq;
            buf2[5u] = msg->sysid;
            buf2[6u] = msg->compid;
            buf2[7u] = msg->msgid & 0xFFU;
            buf2[8u] = (msg->msgid >> 8U) & 0xFFU;
            buf2[9u] = (msg->msgid >> 16U) & 0xFFU;
            _mavlink_send_uart(chan, (const char *)buf2, header_len);
        }
        _mavlink_send_uart(chan, _MAV_PAYLOAD(msg), msg->len);
        _mavlink_send_uart(chan, (const char *)ck, 2U);
        if (signature_len != 0U) 
        {
            _mavlink_send_uart(chan, (const char *)msg->signature, MAVLINK_SIGNATURE_BLOCK_LEN);
        }
        MAVLINK_END_UART_SEND(chan, header_len + msg->len + 2u + signature_len);
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

/**
 * @brief Pack a message to send it over a serial byte stream
 */
MAVLINK_HELPER uint16_t mavlink_msg_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg)
{
        uint8_t signature_len, header_len;
        uint8_t *ck;
        uint8_t length = msg->len;

        if (msg->magic == MAVLINK_STX_MAVLINK1) 
        {
                signature_len = 0u;
                header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN;
                buf[0u] = msg->magic;
                buf[1u] = length;
                buf[2u] = msg->seq;
                buf[3u] = msg->sysid;
                buf[4u] = msg->compid;
                buf[5u] = msg->msgid & 0xFFU;
                memcpy((void*) &buf[6u],(void*) _MAV_PAYLOAD(msg),(int16_t) msg->len);
                ck = buf + header_len + 1 + (uint16_t)msg->len;
        } 
        else 
        {
                length = _mav_trim_payload(_MAV_PAYLOAD(msg), length);
                header_len = MAVLINK_CORE_HEADER_LEN;
                buf[0u] = msg->magic;
                buf[1u] = length;
                buf[2u] = msg->incompat_flags;
                buf[3u] = msg->compat_flags;
                buf[4u] = msg->seq;
                buf[5u] = msg->sysid;
                buf[6u] = msg->compid;
                buf[7u] = msg->msgid & 0xFFU;
                buf[8u] = (msg->msgid >> 8U) & 0xFFU;
                buf[9u] = (msg->msgid >> 16U) & 0xFFU;
                memcpy((void*) &buf[10u],(void*) _MAV_PAYLOAD(msg),(int16_t) length);
                ck = buf + header_len + 1u + (uint16_t)length;
                signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED)?MAVLINK_SIGNATURE_BLOCK_LEN:0u;
        }
        ck[0u] = (uint8_t)(msg->checksum & 0xFFU);
        ck[1u] = (uint8_t)(msg->checksum >> 8U);
        if (signature_len > 0u) 
        {
            memcpy(&ck[2u], msg->signature, signature_len);
        }

        return header_len + 1u + 2u + (uint16_t)length + (uint16_t)signature_len;
}

union __mavlink_bitfield 
{
        uint8_t uint8;
        int8_t int8;
        uint16_t uint16;
        int16_t int16;
        uint32_t uint32;
        int32_t int32;
};

MAVLINK_HELPER void mavlink_start_checksum(mavlink_message_t* msg)
{
        uint16_t crcTmp = 0u;
        crc_init(&crcTmp);
        msg->checksum = crcTmp;
}

MAVLINK_HELPER void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c)
{
        uint16_t checksum = msg->checksum;
        crc_accumulate(c, &checksum);
        msg->checksum = checksum;
}

/*
  return the crc_entry value for a msgid
*/
#ifndef MAVLINK_GET_MSG_ENTRY
MAVLINK_HELPER const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t msgid)
{
        static const mavlink_msg_entry_t mavlink_message_crcs[] = MAVLINK_MESSAGE_CRCS;
        /*
          use a bisection search to find the right entry. A perfect hash may be better
          Note that this assumes the table is sorted by msgid
        */
        uint32_t low=0u, high=sizeof(mavlink_message_crcs)/sizeof(mavlink_message_crcs[0u]);
        uint32_t mid;
        
        while (low < high) 
        {
            mid = (low+1+high)/2u;
            if (msgid < mavlink_message_crcs[mid].msgid) 
            {
                high = mid-1u;
                continue;
            }
            if (msgid > mavlink_message_crcs[mid].msgid) 
            {
                low = mid;
                continue;
            }
            low = mid;
            break;
        }
        if (mavlink_message_crcs[low].msgid != msgid) 
        {
            return NULL;                                                        // msgid is not in the table
        }
        return &mavlink_message_crcs[low];
}
#endif // MAVLINK_GET_MSG_ENTRY

/*
  return the crc_extra value for a message
*/
MAVLINK_HELPER uint8_t mavlink_get_crc_extra(const mavlink_message_t *msg)
{
        const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
        return e?e->crc_extra:0u;
}

/*
  return the min message length
*/
#define MAVLINK_HAVE_MIN_MESSAGE_LENGTH
MAVLINK_HELPER uint8_t mavlink_min_message_length(const mavlink_message_t *msg)
{
        const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
        return e?e->min_msg_len:0u;
}

/*
  return the max message length (including extensions)
*/
#define MAVLINK_HAVE_MAX_MESSAGE_LENGTH
MAVLINK_HELPER uint8_t mavlink_max_message_length(const mavlink_message_t *msg)
{
        const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
        return e?e->max_msg_len:0u;
}

/**
 * This is a variant of mavlink_frame_char() but with caller supplied
 * parsing buffers. It is useful when you want to create a MAVLink
 * parser in a library that doesn't use any global variables
 *
 * @param rxmsg    parsing message buffer
 * @param status   parsing status buffer
 * @param c        The char to parse
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC
 *
 */
MAVLINK_HELPER uint8_t mavlink_frame_char_buffer(mavlink_message_t* rxmsg,
                                                 mavlink_status_t* status,
                                                 uint8_t c,
                                                 mavlink_message_t* r_message,
                                                 mavlink_status_t* r_mavlink_status)
{
        /* Enable this option to check the length of each message.
           This allows invalid messages to be caught much sooner. Use if the transmission
           medium is prone to missing (or extra) characters (e.g. a radio that fades in
           and out). Only use if the channel will only contain messages types listed in
           the headers.
        */
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
#ifndef MAVLINK_MESSAGE_LENGTH
        static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
#define MAVLINK_MESSAGE_LENGTH(msgid) mavlink_message_lengths[msgid]
#endif
#endif

        int16_t bufferIndex = 0u;
        const mavlink_msg_entry_t *e;
        uint8_t crc_extra;
        bool sig_ok;

        status->msg_received = MAVLINK_FRAMING_INCOMPLETE;

        switch (status->parse_state)
        {
        case MAVLINK_PARSE_STATE_UNINIT:
        case MAVLINK_PARSE_STATE_IDLE:
                if (c == MAVLINK_STX)
                {
                        status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
                        rxmsg->len = 0u;
                        rxmsg->magic = c;
                        status->flags &= ~MAVLINK_STATUS_FLAG_IN_MAVLINK1;
                        mavlink_start_checksum(rxmsg);
                } else if (c == MAVLINK_STX_MAVLINK1)
                {
                        status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
                        rxmsg->len = 0u;
                        rxmsg->magic = c;
                        status->flags |= MAVLINK_STATUS_FLAG_IN_MAVLINK1;
                        mavlink_start_checksum(rxmsg);
                }
                break;

        case MAVLINK_PARSE_STATE_GOT_STX:
                if (status->msg_received
/* Support shorter buffers than the
   default maximum packet size */
#if (MAVLINK_MAX_PAYLOAD_LEN < 255)
                    || c > MAVLINK_MAX_PAYLOAD_LEN
#endif
                    )
                {
                    status->buffer_overrun++;
                    _mav_parse_error(status);
                    status->msg_received = 0u;
                    status->parse_state = MAVLINK_PARSE_STATE_IDLE;
                }
                else
                {
                     rxmsg->len = c;                                            // NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
                     status->packet_idx = 0u;
                     mavlink_update_checksum(rxmsg, c);
                     if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) 
                     {
                        rxmsg->incompat_flags = 0u;
                        rxmsg->compat_flags = 0u;
                        status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
                     } 
                     else 
                     {
                         status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
                     }
                }
                break;

        case MAVLINK_PARSE_STATE_GOT_LENGTH:
                rxmsg->incompat_flags = c;
                if ((rxmsg->incompat_flags & ~MAVLINK_IFLAG_MASK) != 0u)
                {
                    _mav_parse_error(status);                                   // message includes an incompatible feature flag
                    status->msg_received = 0u;
                    status->parse_state = MAVLINK_PARSE_STATE_IDLE;
                    break;
                }
                mavlink_update_checksum(rxmsg, c);
                status->parse_state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS;
                break;

        case MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:
                rxmsg->compat_flags = c;
                mavlink_update_checksum(rxmsg, c);
                status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
                break;

        case MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:
                rxmsg->seq = c;
                mavlink_update_checksum(rxmsg, c);
                status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
                break;

        case MAVLINK_PARSE_STATE_GOT_SEQ:
                rxmsg->sysid = c;
                mavlink_update_checksum(rxmsg, c);
                status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
                break;

        case MAVLINK_PARSE_STATE_GOT_SYSID:
                rxmsg->compid = c;
                mavlink_update_checksum(rxmsg, c);
                status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
                break;

        case MAVLINK_PARSE_STATE_GOT_COMPID:
                rxmsg->msgid = c;
                mavlink_update_checksum(rxmsg, c);
                if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) 
                {
                    if(rxmsg->len > 0u)
                    {
                        status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
                    } 
                    else 
                    {
                        status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
                    }
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
                    if (rxmsg->len != MAVLINK_MESSAGE_LENGTH(rxmsg->msgid))
                    {
                        _mav_parse_error(status);
                        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
                        break;
                    }
#endif
                } 
                else 
                {
                    status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID1;
                }
                break;

        case MAVLINK_PARSE_STATE_GOT_MSGID1:
                rxmsg->msgid |= c<<8u;
                mavlink_update_checksum(rxmsg, c);
                status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID2;
                break;

        case MAVLINK_PARSE_STATE_GOT_MSGID2:
                rxmsg->msgid |= ((uint32_t)c)<<16u;
                mavlink_update_checksum(rxmsg, c);
                if(rxmsg->len > 0u)
                {
                   status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
                } 
                else 
                {
                   status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
                }
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
                if (rxmsg->len != MAVLINK_MESSAGE_LENGTH(rxmsg->msgid))
                {
                    _mav_parse_error(status);
                    status->parse_state = MAVLINK_PARSE_STATE_IDLE;
                    break;
                }
#endif
                break;

        case MAVLINK_PARSE_STATE_GOT_MSGID3:
                _MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;
                mavlink_update_checksum(rxmsg, c);
                if (status->packet_idx == rxmsg->len)
                {
                   status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
                }
                break;

        case MAVLINK_PARSE_STATE_GOT_PAYLOAD: 
        {
                e = mavlink_get_msg_entry(rxmsg->msgid);
                crc_extra = e?e->crc_extra:0u;
                mavlink_update_checksum(rxmsg, crc_extra);
                if (c != (rxmsg->checksum & 0xFFU)) 
                {
                    status->parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC1;
                } 
                else 
                {
                    status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
                }
                rxmsg->ck[0u] = c;

                // zero-fill the packet to cope with short incoming packets
                if (e && status->packet_idx < e->max_msg_len) 
                {
                    memset(&_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx], 0u, e->max_msg_len - status->packet_idx);
                }
                break;
        }

        case MAVLINK_PARSE_STATE_GOT_CRC1:
        case MAVLINK_PARSE_STATE_GOT_BAD_CRC1:
                if (status->parse_state == MAVLINK_PARSE_STATE_GOT_BAD_CRC1 || c != (rxmsg->checksum >> 8u)) 
                {
                   status->msg_received = MAVLINK_FRAMING_BAD_CRC;              // got a bad CRC message
                } 
                else 
                {
                    status->msg_received = MAVLINK_FRAMING_OK;                  // Successfully got message
                }
                rxmsg->ck[1u] = c;

                if (rxmsg->incompat_flags & MAVLINK_IFLAG_SIGNED) 
                {
                    status->parse_state = MAVLINK_PARSE_STATE_SIGNATURE_WAIT;
                    status->signature_wait = MAVLINK_SIGNATURE_BLOCK_LEN;

                    // If the CRC is already wrong, don't overwrite msg_received,
                    // otherwise we can end up with garbage flagged as valid.
                    if (status->msg_received != MAVLINK_FRAMING_BAD_CRC) 
                    {
                        status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
                    }
                } 
                else 
                {
                    if (status->signing &&
                        (status->signing->accept_unsigned_callback == NULL ||
                         !status->signing->accept_unsigned_callback(status, rxmsg->msgid))) {

                                // If the CRC is already wrong, don't overwrite msg_received.
                                if (status->msg_received != MAVLINK_FRAMING_BAD_CRC) 
                                {
                                   status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;
                                }
                        }
                        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
                        if (r_message != NULL) 
                        {
                            memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
                        }
                }
                break;
        case MAVLINK_PARSE_STATE_SIGNATURE_WAIT:
                rxmsg->signature[MAVLINK_SIGNATURE_BLOCK_LEN-status->signature_wait] = c;
                status->signature_wait--;
                if (status->signature_wait == 0u)
                {
                        // we have the whole signature, check it is OK
                        sig_ok = mavlink_signature_check(status->signing, status->signing_streams, rxmsg);
                        if (!sig_ok &&
                           (status->signing->accept_unsigned_callback &&
                           status->signing->accept_unsigned_callback(status, rxmsg->msgid))) {
                           // accepted via application level override
                           sig_ok = true;
                        }
                        if (sig_ok) 
                        {
                           status->msg_received = MAVLINK_FRAMING_OK;
                        } 
                        else 
                        {
                           status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;
                        }
                        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
                        if (r_message !=NULL) 
                        {
                           memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
                        }
                }
                break;
        }

        bufferIndex=++bufferIndex % INT16_MAX;
        // If a message has been sucessfully decoded, check index
        if (status->msg_received == MAVLINK_FRAMING_OK)
        {
                //while(status->current_seq != rxmsg->seq)
                //{
                //        status->packet_rx_drop_count++;
                //               status->current_seq++;
                //}
                status->current_rx_seq = rxmsg->seq;

                if (status->packet_rx_success_count == 0u)                      // Initial condition: If no packet has been received so far, drop count is undefined
                   status->packet_rx_drop_count = 0u;

                status->packet_rx_success_count=++status->packet_rx_success_count % UINT16_MAX;  // Count this packet as received
        }

       if (r_message != NULL) 
       {
           r_message->len = rxmsg->len;                                         // Provide visibility on how far we are into current msg
       }
       if (r_mavlink_status != NULL) 
       {
           r_mavlink_status->parse_state = status->parse_state;
           r_mavlink_status->packet_idx = status->packet_idx;
           r_mavlink_status->current_rx_seq = status->current_rx_seq+1u;
           r_mavlink_status->packet_rx_success_count = status->packet_rx_success_count;
           r_mavlink_status->packet_rx_drop_count = status->parse_error;
           r_mavlink_status->flags = status->flags;
       }
       status->parse_error = 0u;

        if (status->msg_received == MAVLINK_FRAMING_BAD_CRC) 
        {
            /*
              the CRC came out wrong. We now need to overwrite the
              msg CRC with the one on the wire so that if the
              caller decides to forward the message anyway that
              mavlink_msg_to_send_buffer() won't overwrite the
              checksum
            */
            if (r_message != NULL) 
            {
                r_message->checksum = rxmsg->ck[0u] | (rxmsg->ck[1u]<<8u);
            }
        }
        return status->msg_received;
}

/**
 * This is a convenience function which handles the complete MAVLink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. This function will return 0, 1 or
 * 2 (MAVLINK_FRAMING_INCOMPLETE, MAVLINK_FRAMING_OK or MAVLINK_FRAMING_BAD_CRC)
 *
 * Messages are parsed into an internal buffer (one for each channel). When a complete
 * message is received it is copies into *r_message and the channel's status is
 * copied into *r_mavlink_status.
 *
 * @param chan     ID of the channel to be parsed.
 *                 A channel is not a physical message channel like a serial port, but a logical partition of
 *                 the communication streams. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * @param c        The char to parse
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <mavlink.h>
 *
 * mavlink_status_t status;
 * mavlink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (mavlink_frame_char(chan, byte, &msg, &status) != MAVLINK_FRAMING_INCOMPLETE)
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
 *     }
 * }
 *
 *
 * @endcode
 */
MAVLINK_HELPER uint8_t mavlink_frame_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
    return mavlink_frame_char_buffer(mavlink_get_channel_buffer(chan),  mavlink_get_channel_status(chan), c, r_message, r_mavlink_status);
}

/**
 * Set the protocol version
 */
MAVLINK_HELPER void mavlink_set_proto_version(uint8_t chan, uint16_t version)
{
        mavlink_status_t *status = mavlink_get_channel_status(chan);
        if (version > 1u)
        {
            status->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
        } 
        else 
        {
            status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        }
}

/**
 * Get the protocol version
 *
 * @return 1 for v1, 2 for v2
 */
MAVLINK_HELPER uint16_t mavlink_get_proto_version(uint8_t chan)
{
        mavlink_status_t *status = mavlink_get_channel_status(chan);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) > 0u)
        {
            return 1u;
        } 
        else 
        {
            return 2u;
        }
}

/**
 * This is a convenience function which handles the complete MAVLink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. This function will return 0 or 1.
 *
 * Messages are parsed into an internal buffer (one for each channel). When a complete
 * message is received it is copies into *r_message and the channel's status is
 * copied into *r_mavlink_status.
 *
 * @param chan     ID of the channel to be parsed.
 *                 A channel is not a physical message channel like a serial port, but a logical partition of
 *                 the communication streams. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * @param c        The char to parse
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded or bad CRC, 1 on good message and CRC
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <mavlink.h>
 *
 * mavlink_status_t status;
 * mavlink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (mavlink_parse_char(chan, byte, &msg, &status))
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
 *     }
 * }
 *
 *
 * @endcode
 */
MAVLINK_HELPER uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
    uint8_t msg_received = mavlink_frame_char(chan, c, r_message, r_mavlink_status);
    mavlink_message_t* rxmsg;
    mavlink_status_t* status;
    
    if (msg_received == MAVLINK_FRAMING_BAD_CRC || msg_received == MAVLINK_FRAMING_BAD_SIGNATURE)
    {
        rxmsg = mavlink_get_channel_buffer(chan);                           // we got a bad CRC. Treat as a parse failure
        status = mavlink_get_channel_status(chan);
        _mav_parse_error(status);
        status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
        if (c == MAVLINK_STX)
        {
          status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
          rxmsg->len = 0u;
          mavlink_start_checksum(rxmsg);
        }
        return 0u;
    }
    return msg_received;
}

/**
 * @brief Put a bitfield of length 1-32 bit into the buffer
 *
 * @param b the value to add, will be encoded in the bitfield
 * @param bits number of bits to use to encode b, e.g. 1 for boolean, 2, 3, etc.
 * @param packet_index the position in the packet (the index of the first byte to use)
 * @param bit_index the position in the byte (the index of the first bit to use)
 * @param buffer packet buffer to write into
 * @return new position of the last used byte in the buffer
 */
MAVLINK_HELPER uint8_t put_bitfield_n_by_index(int32_t b, uint8_t bits, uint8_t packet_index, uint8_t bit_index, uint8_t* r_bit_index, uint8_t* buffer)
{
        uint16_t bits_remain = bits;
        int32_t v;                                                              // Transform number into network order
        uint8_t i_bit_index, i_byte_index, curr_bits_n;
#if MAVLINK_NEED_BYTE_SWAP
        union 
        {
          int32_t i;
          uint8_t b[4u];
        } bin, bout;
        bin.i = b;
        bout.b[0u] = bin.b[3u];
        bout.b[1u] = bin.b[2u];
        bout.b[2u] = bin.b[1u];
        bout.b[3u] = bin.b[0u];
        v = bout.i;
#else
        v = b;
#endif

        // buffer in
        // 01100000 01000000 00000000 11110001
        // buffer out
        // 11110001 00000000 01000000 01100000

        // Existing partly filled byte (four free slots)
        // 0111xxxx

        // Mask n free bits
        // 00001111 = 2^0 + 2^1 + 2^2 + 2^3 = 2^n - 1
        // = ((uint32_t)(1 << n)) - 1; // = 2^n - 1

        // Shift n bits into the right position
        // out = in >> n;

        // Mask and shift bytes
        i_bit_index = bit_index;
        i_byte_index = packet_index;
        if (bit_index > 0u)
        {
            // If bits were available at start, they were available
            // in the byte before the current index
            i_byte_index--;
        }

        while (bits_remain > 0u)                                                // While bits have not been packed yet
        {
                // Bits still have to be packed
                // there can be more than 8 bits, so
                // we might have to pack them into more than one byte

                // First pack everything we can into the current 'open' byte
                //curr_bits_n = bits_remain << 3; // Equals  bits_remain mod 8
                //FIXME
                if (bits_remain <= (uint8_t)(8u - i_bit_index))
                {
                    curr_bits_n = (uint8_t)bits_remain;                         // Enough space
                }
                else
                {
                    curr_bits_n = (8 - i_bit_index);
                }

                // Pack these n bits into the current byte
                // Mask out whatever was at that position with ones (xxx11111)
                buffer[i_byte_index] &= (0xFFU >> (8U - curr_bits_n));
                
                buffer[i_byte_index] |= ((0x00u << curr_bits_n) & v);           // Put content to this position, by masking out the non-used part

                i_bit_index += curr_bits_n;                                     // Increment the bit index

                bits_remain -= curr_bits_n;                                     // Now proceed to the next byte, if necessary
                if (bits_remain > 0u)
                {
                   i_byte_index=++i_byte_index % UINT8_MAX;                     /* Offer another 8 bits / one byte */
                   i_bit_index = 0u;
                }
        }

        *r_bit_index = i_bit_index;

        if (i_bit_index != 7u) i_byte_index=++i_byte_index % UINT8_MAX;         // If a partly filled byte is present, mark this as consumed
        return i_byte_index - packet_index;
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

// To make MAVLink work on your MCU, define comm_send_ch() if you wish
// to send 1 byte at a time, or MAVLINK_SEND_UART_BYTES() to send a
// whole packet at a time

/*

#include "mavlink_types.h"

void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
        uart0_transmit(ch);
    }
    if (chan == MAVLINK_COMM_1)
    {
            uart1_transmit(ch);
    }
}
 */

MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len)
{
#ifdef MAVLINK_SEND_UART_BYTES
 uint8_t returnUDP=0u;
        /* this is the more efficient approach, if the platform
           defines it */
        //MAVLINK_SEND_UART_BYTES(chan, (const uint8_t *)buf, len);
        memset((void *) buf[len],(void *) '\0',sizeof(char));
        switch(chan)
        {
#if defined(SERIAL_MAVLINK)                                                     /* this is the serial port or a relay port for a UDP frame */
           case 1u:
           UART1_Write_Text((unsigned char*)buf);
           break;

           case 2u:
           UART2_Write_Text((unsigned char*)buf);
           break;

           case 3u:
           UART3_Write_Text((unsigned char*)buf);
           break;

           case 4u:
           UART4_Write_Text((unsigned char*)buf);
           break;
           
           case 5u:
           UART5_Write_Text((unsigned char*)buf);
           break;

           case 6u:
           UART6_Write_Text((unsigned char*)buf);
           break;
           
           default:
           UART6_Write_Text((unsigned char*)buf);
           break;

#else
           case 7u:
           returnUDP = Net_Ethernet_Intern_sendUDP(MavLinkBuf.RemoteIpAddr, MavLinkBuf.From_Port, MavLinkBuf.Dest_Port,(unsigned char*)buf, len);
           break;
           
           default:
           returnUDP = Net_Ethernet_Intern_sendUDP(MavLinkBuf.RemoteIpAddr, MavLinkBuf.From_Port, MavLinkBuf.Dest_Port,(unsigned char*)buf, len);
           break;
#endif  /* end serial or UDP */

         }
#else
        /* fallback to one byte at a time */
        uint16_t i;
        for (i = 0u; i < len; i++)
        {
           switch(chan)
           {
              case 1u:
              UART1_Write((uint8_t)buf[i]);
              break;
              
              case 2u:
              UART2_Write((uint8_t)buf[i]);
              break;
              
              case 3u:
              UART3_Write((uint8_t)buf[i]);
              break;
              
              case 4u:
              UART4_Write((uint8_t)buf[i]);
              break;
              
              case 5u:
              UART5_Write((uint8_t)buf[i]);
              break;
              
              case 6u:
              UART6_Write((uint8_t)buf[i]);
              break;
              
              default:                                                          /* there is no udp send for this option of sending a char at a time */
              UART6_Write((uint8_t)buf[i]);
              break;
           }
        }
#endif  /* single or multi-char write */
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

//#ifdef MAVLINK_USE_CXX_NAMESPACE
//} // namespace mavlink
//#endif
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // end library