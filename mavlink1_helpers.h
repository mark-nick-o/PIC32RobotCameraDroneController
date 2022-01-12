#ifndef MAVLINK1_HELPER__
#define MAVLINK1_HELPER__
/* #endif  */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifndef  _MAVLINK_HELPERS_H_
#define  _MAVLINK_HELPERS_H_

/* #include "string.h" */
#include "mavlink_crc.h"
#include "mavlink1_msg_types.h"
#include "mavlink_conversions.h"

#ifndef MAVLINK_HELPER
#define MAVLINK_HELPER
#endif

/*
 * Internal function to give access to the channel status for each channel
 */
#ifndef MAVLINK_GET_CHANNEL_STATUS
MAVLINK_HELPER mavlink_status_t* mavlink_get_channel_status(uint8_t chan)
{
#ifdef MAVLINK_EXTERNAL_RX_STATUS
        /* No m_mavlink_status array defined in function,
           has to be defined externally */
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
        /* -------------- No m_mavlink_buffer array defined in function,
                          has to be defined externally ---------------------- */
#else
        static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
#endif
        return &m_mavlink_buffer[chan];
}
#endif

/**
 * @brief Reset the status of a channel.
 */
MAVLINK_HELPER void mavlink_reset_channel_status(uint8_t chan)
{
        mavlink_status_t *status = mavlink_get_channel_status(chan);
        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
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
#if MAVLINK_CRC_EXTRA
MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                                                      uint8_t chan, uint8_t min_length, uint8_t length, uint8_t crc_extra)
#else
MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, uint8_t chan, uint8_t length)
#endif
{
        (void)min_length;                                                       /* This is only used for the v2 protocol and we silence it here */
        msg->magic = MAVLINK_STX;                                               /* This code part is the same for all messages; */
        msg->len = length;
        msg->sysid = system_id;
        msg->compid = component_id;
        msg->seq = mavlink_get_channel_status(chan)->current_tx_seq;            /* One sequence number per channel */
        mavlink_get_channel_status(chan)->current_tx_seq = mavlink_get_channel_status(chan)->current_tx_seq+1;
        msg->checksum = crc_calculate(((const uint8_t*)(msg)) + 3u, MAVLINK_CORE_HEADER_LEN);
        crc_accumulate_buffer(&msg->checksum, _MAV_PAYLOAD(msg), msg->len);
#if MAVLINK_CRC_EXTRA
        crc_accumulate(crc_extra, &msg->checksum);
#endif
        mavlink_ck_a(msg) = (uint8_t)(msg->checksum & 0xFFu);
        mavlink_ck_b(msg) = (uint8_t)(msg->checksum >> 8u);

        return length + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}


/**
 * @brief Finalize a MAVLink message with MAVLINK_COMM_0 as default channel
 */
#if MAVLINK_CRC_EXTRA
MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                                                 uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
    return mavlink_finalize_message_chan(msg, system_id, component_id, MAVLINK_COMM_0, min_length, length, crc_extra);
}
#else
MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                                                 uint8_t length)
{
        return mavlink_finalize_message_chan(msg, system_id, component_id, MAVLINK_COMM_0, length);
}
#endif

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len);

/**
 * @brief Finalize a MAVLink message with channel assignment and send
 */
#if MAVLINK_CRC_EXTRA
MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint8_t msgid, const char *packet, uint8_t min_length, uint8_t length, uint8_t crc_extra)
#else
MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint8_t msgid, const char *packet, uint8_t length)
#endif
{
        uint16_t checksum;
        uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
        uint8_t ck[2u];
        mavlink_status_t *status = mavlink_get_channel_status(chan);
        mavlink_system_t mavlink_system;                                        // make this a volatile global if you want it

        mavlink_system.sysid = MAV_SYS_ID;                                      // make this a volatile global if you want to keep changing leave as global define
        mavlink_system.compid = MAV_COMP_ID;
        
        buf[0u] = MAVLINK_STX;
        buf[1u] = length;
        buf[2u] = status->current_tx_seq;
        buf[3u] = mavlink_system.sysid;
        buf[4u] = mavlink_system.compid;
        buf[5u] = msgid;

        status->current_tx_seq++;
        checksum = crc_calculate((const uint8_t*)&buf[1u], MAVLINK_CORE_HEADER_LEN);
        crc_accumulate_buffer(&checksum, packet, length);
#if MAVLINK_CRC_EXTRA
        crc_accumulate(crc_extra, &checksum);
#endif
        ck[0u] = (uint8_t)(checksum & 0xFFu);
        ck[1u] = (uint8_t)(checksum >> 8u);

        MAVLINK_START_UART_SEND(chan, MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)length);
        _mavlink_send_uart(chan, (const char *)buf, MAVLINK_NUM_HEADER_BYTES);
        _mavlink_send_uart(chan, packet, length);
        _mavlink_send_uart(chan, (const char *)ck, 2);
        MAVLINK_END_UART_SEND(chan, MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)length);
}

/**
 * @brief re-send a message over a uart channel
 * this is more stack efficient than re-marshalling the message
 */
MAVLINK_HELPER void _mavlink_resend_uart(mavlink_channel_t chan, const mavlink_message_t *msg)
{
        uint8_t ck[2u];

        ck[0u] = (uint8_t)(msg->checksum & 0xFFu);
        ck[1u] = (uint8_t)(msg->checksum >> 8u);
        MAVLINK_START_UART_SEND(chan, MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len);  /* XXX use the right sequence here */
        _mavlink_send_uart(chan, (const char *)&msg->magic, MAVLINK_NUM_HEADER_BYTES);
        _mavlink_send_uart(chan, _MAV_PAYLOAD(msg), msg->len);
        _mavlink_send_uart(chan, (const char *)ck, 2);
        MAVLINK_END_UART_SEND(chan, MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len);
}
#endif /* MAVLINK_USE_CONVENIENCE_FUNCTIONS  */

/**
 * @brief Pack a message to send it over a serial byte stream
 */
MAVLINK_HELPER uint16_t mavlink_msg_to_send_buffer(uint8_t *buffer, const mavlink_message_t *msg)
{
        uint8_t *ck;
        memcpy((void*)buffer, (void *)&msg->magic,(int16_t) (MAVLINK_NUM_HEADER_BYTES + msg->len));

        ck = buffer + (MAVLINK_NUM_HEADER_BYTES + (uint16_t)msg->len);

        ck[0u] = (uint8_t)(msg->checksum & 0xFFu);
        ck[1u] = (uint8_t)(msg->checksum >> 8u);

        return MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)msg->len;
}

union __mavlink_bitfield {
        uint8_t uint8;
        int8_t int8;
        uint16_t uint16;
        int16_t int16;
        uint32_t uint32;
        int32_t int32;
};


MAVLINK_HELPER void mavlink_start_checksum(mavlink_message_t* msg)
{
        crc_init(&msg->checksum);
}

MAVLINK_HELPER void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c)
{
        crc_accumulate(c, &msg->checksum);
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
 */
MAVLINK_HELPER uint8_t mavlink_frame_char_buffer(mavlink_message_t* rxmsg, mavlink_status_t* status, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
        /*
          default message crc function. You can override this per-system to
          put this data in a different memory segment
        */
#if MAVLINK_CRC_EXTRA
#ifndef MAVLINK_MESSAGE_CRC
        static const uint8_t mavlink_message_crcs[256u] = MAVLINK_MESSAGE_CRCS;
#define MAVLINK_MESSAGE_CRC(msgid) mavlink_message_crcs[msgid]
#endif
#endif

        /* Enable this option to check the length of each message.
           This allows invalid messages to be caught much sooner. Use if the transmission
           medium is prone to missing (or extra) characters (e.g. a radio that fades in
           and out). Only use if the channel will only contain messages types listed in
           the headers.
        */
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
#ifndef MAVLINK_MESSAGE_LENGTH
        static const uint8_t mavlink_message_lengths[256u] = MAVLINK_MESSAGE_LENGTHS;
#define MAVLINK_MESSAGE_LENGTH(msgid) mavlink_message_lengths[msgid]
#endif
#endif

        int16_t bufferIndex = 0;

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
                    mavlink_start_checksum(rxmsg);
                }
                break;

        case MAVLINK_PARSE_STATE_GOT_STX:
                if (status->msg_received
#if (MAVLINK_MAX_PAYLOAD_LEN < 255)                                             /* Support shorter buffers than the default maximum packet size */
                    || c > MAVLINK_MAX_PAYLOAD_LEN
#endif
                   )
                {
                   status->buffer_overrun++;
                   status->parse_error=++status->parse_error % UINT8_MAX;
                   status->msg_received = 0u;
                   status->parse_state = MAVLINK_PARSE_STATE_IDLE;
                }
                else
                {
                   rxmsg->len = c;                                              /* NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2  */
                   status->packet_idx = 0u;
                   mavlink_update_checksum(rxmsg, c);
                   status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
                }
                break;

        case MAVLINK_PARSE_STATE_GOT_LENGTH:
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
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
                if (rxmsg->len != MAVLINK_MESSAGE_LENGTH(c))
                {
                   status->parse_error=++status->parse_error % UINT8_MAX;
                   status->parse_state = MAVLINK_PARSE_STATE_IDLE;
                   break;
                }
#endif
                rxmsg->msgid = c;
                mavlink_update_checksum(rxmsg, c);
                if (rxmsg->len == 0u)
                {
                   status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
                }
                else
                {
                   status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID;
                }
                break;

        case MAVLINK_PARSE_STATE_GOT_MSGID:
                _MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;
                mavlink_update_checksum(rxmsg, c);
                if (status->packet_idx == rxmsg->len)
                {
                   status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
                }
                break;

        case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
#if MAVLINK_CRC_EXTRA
                mavlink_update_checksum(rxmsg, MAVLINK_MESSAGE_CRC(rxmsg->msgid));
#endif
                if (c != (rxmsg->checksum & 0xFFu)) 
                {
                   status->parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC1;
                } 
                else 
                {
                   status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
                }
                _MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx] = (char)c;
                break;

        case MAVLINK_PARSE_STATE_GOT_CRC1:
        case MAVLINK_PARSE_STATE_GOT_BAD_CRC1:
                if (status->parse_state == MAVLINK_PARSE_STATE_GOT_BAD_CRC1 || c != (rxmsg->checksum >> 8u)) 
                {
                    status->msg_received = MAVLINK_FRAMING_BAD_CRC;             /* got a bad CRC message */
                }
                else
                {
                    status->msg_received = MAVLINK_FRAMING_OK;                  /* Successfully got message */
                }
                status->parse_state = MAVLINK_PARSE_STATE_IDLE;
                _MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx+1u] = (char)c;
                memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
                break;
        }

        bufferIndex=++bufferIndex % INT16_MAX;

        if (status->msg_received == MAVLINK_FRAMING_OK)                         /* If a message has been sucessfully decoded, check index */
        {
                /* while(status->current_seq != rxmsg->seq)
                //{
                //        status->packet_rx_drop_count++;
                //               status->current_seq++;
                //} */
                status->current_rx_seq = rxmsg->seq;
                if (status->packet_rx_success_count == 0u) status->packet_rx_drop_count = 0u; /* Initial condition: If no packet has been received so far, drop count is undefined */
                status->packet_rx_success_count=++status->packet_rx_success_count % UINT16_MAX;  /* Count this packet as received */
        }

        r_message->len = rxmsg->len;                                            /* Provide visibility on how far we are into current msg  */
        r_mavlink_status->parse_state = status->parse_state;
        r_mavlink_status->packet_idx = status->packet_idx;
        r_mavlink_status->current_rx_seq = status->current_rx_seq+1u;
        r_mavlink_status->packet_rx_success_count = status->packet_rx_success_count;
        r_mavlink_status->packet_rx_drop_count = status->parse_error;
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
            r_message->checksum = _MAV_PAYLOAD(rxmsg)[status->packet_idx] | (_MAV_PAYLOAD(rxmsg)[status->packet_idx+1]<<8u);
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
 * @param r_message NULL if no message could be decoded, the message data else
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
    return mavlink_frame_char_buffer(mavlink_get_channel_buffer(chan), mavlink_get_channel_status(chan), c, r_message, r_mavlink_status);
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
 * @param r_message NULL if no message could be decoded, otherwise the message data.
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
    if (msg_received == MAVLINK_FRAMING_BAD_CRC) {
            mavlink_message_t* rxmsg = mavlink_get_channel_buffer(chan);        /* we got a bad CRC. Treat as a parse failure */
            mavlink_status_t* status = mavlink_get_channel_status(chan);
            status->parse_error=++status->parse_error % UINT8_MAX;
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

        int32_t v;                                                              /* Transform number into network order */
        uint8_t i_bit_index, i_byte_index, curr_bits_n;
#if MAVLINK_NEED_BYTE_SWAP
        union {
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
        /* buffer in
         01100000 01000000 00000000 11110001
         buffer out
         11110001 00000000 01000000 01100000
         Existing partly filled byte (four free slots)
         0111xxxx
         Mask n free bits
         00001111 = 2^0 + 2^1 + 2^2 + 2^3 = 2^n - 1
         = ((uint32_t)(1 << n)) - 1; 
         = 2^n - 1
         Shift n bits into the right position
         out = in >> n;       */

        i_bit_index = bit_index;                                                /* Mask and shift bytes */
        i_byte_index = packet_index;
        if (bit_index > 0u)
        {
                /* If bits were available at start, they were available
                   in the byte before the current index */
                i_byte_index--;
        }
        while (bits_remain > 0u)                                                /* While bits have not been packed yet */
        {
                /* Bits still have to be packed
                 there can be more than 8 bits, so
                 we might have to pack them into more than one byte

                 First pack everything we can into the current 'open' byte
                 curr_bits_n = bits_remain << 3; // Equals  bits_remain mod 8
                 FIXME */
                if (bits_remain <= (uint8_t)(8u - i_bit_index))
                {
                    curr_bits_n = (uint8_t)bits_remain;                         /* Enough space  */
                }
                else
                {
                    curr_bits_n = (8u - i_bit_index);
                }
                buffer[i_byte_index] &= (0xFFu >> (8u - curr_bits_n));          /* Pack these n bits into the current byte Mask out whatever was at that position with ones (xxx11111) */
                buffer[i_byte_index] |= ((0x00u << curr_bits_n) & v);           /* Put content to this position, by masking out the non-used part */
                i_bit_index += curr_bits_n;                                     /* Increment the bit index */
                bits_remain -= curr_bits_n;                                     /* Now proceed to the next byte, if necessary */
                if (bits_remain > 0u)
                {
                    i_byte_index=++i_byte_index % UINT8_MAX;                    /* Offer another 8 bits / one byte */
                    i_bit_index = 0u;
                }
        }

        *r_bit_index = i_bit_index;
        
        if (i_bit_index != 7u) i_byte_index=++i_byte_index % UINT8_MAX;         /* If a partly filled byte is present, mark this as consumed  */
        return i_byte_index - packet_index;
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

/* To make MAVLink work on your MCU, define comm_send_ch() if you wish
// to send 1 byte at a time, or MAVLINK_SEND_UART_BYTES() to send a
// whole packet at a time     */

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
   
        //MAVLINK_SEND_UART_BYTES(chan, (const uint8_t *)buf, len);               /* this is the more efficient approach, if the platform defines it */
        memset((void *) buf[len],(void *) '\0',sizeof(char));
        switch(chan)
        {
#if defined(SERIAL_MAVLINK)
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
           returnUDP = Net_Ethernet_Intern_sendUDP(MavLinkBuf.RemoteIpAddr, MavLinkBuf.From_Port, MavLinkBuf.Dest_Port, (unsigned char*)buf, len);
           break;
           
           default:
           returnUDP = Net_Ethernet_Intern_sendUDP(MavLinkBuf.RemoteIpAddr, MavLinkBuf.From_Port, MavLinkBuf.Dest_Port, (unsigned char*)buf, len);
           break;
#endif /* end serial or UDP */
         }
#else
        uint16_t i;                                                             /* fallback to one byte at a time */
        for (i = 0u; i < len; i++) 
        {
            //comm_send_ch(chan, (uint8_t)buf[i]);
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

              default:
              UART6_Write((uint8_t)buf[i]);
              break;
           }
        }
#endif /* single or multi-char write */
}
#endif /* MAVLINK_USE_CONVENIENCE_FUNCTIONS  */

#endif /* _MAVLINK_HELPERS_H_ */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end library     */