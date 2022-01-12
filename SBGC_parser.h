/*
   SBGC_parser.h : SimpleBGC Serial API  library - input data parser
        More info: http://www.basecamelectronics.com/serialapi/

  Copyright (c) 2014-2015 Aleksei Moskalenko
  Ported and expanded by Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
  
  All rights reserved.
        See license info in the SBGC.h
*/

#define __NO_PARSER                                                             // We are not able to use the c++ parser
#ifdef __NO_PARSER                                                              // In the PIC we can not use the parser so only do this....
#ifndef  __SBGC_parser__
#define  __SBGC_parser__

#ifdef __cplusplus
 extern "C" {
#endif

#include "inttypes.h"
#include <stdint.h>
#define SBGC_CMD_START_BYTE '>'                                                 // STX character for SBGC protocol.
#define SBGC_CMD_START_BYTE_V2 '$'                                              // STX character for SBGC protocol. v2 2.68b0 or greater
#define SBGC_START_PAYLOAD_POS 4u                                                // Fixed position in the SBGC message for payload i.e. 4 bytes after STX at byte 0
#define SBGC_PAYLOAD_LENGTH_POS 2u                                               // Fixed position in the SBGC message for payload length
#define SBGC_LEN_OF_HEADER 4u                                                    // Length of the header in bytes

#include "SBGC_command.h"

#ifdef __cplusplus
}
#endif

#endif                                                                          // end __SBGC_parser__
#endif                                                                          // end NO_PARSER

#ifndef  __NO_PARSER                                                            // @171019 take the parser out as it will not work c++
#ifndef  __SBGC_parser__
#define  __SBGC_parser__                                                        // @171019 take the parser out as it will not work c++

#ifdef __cplusplus
 extern "C" {
#endif

#include <inttypes.h>
//#include "inttypes.h"

#include <string.h>
#include "SBGC_command.h"

#define SBGC_CMD_START_BYTE '>'

typedef enum {
        PARSER_NO_ERROR=0,
        PARSER_ERROR_PROTOCOL=1,
        PARSER_ERROR_WRONG_CMD_SIZE=2,
        PARSER_ERROR_BUFFER_IS_FULL=3,
        PARSER_ERROR_WRONG_DATA_SIZE=4,
} SBGC_parser_errors;

/*
 * Abstract class that implements general primitive reading/writing from stream.
 */
class SBGC_IOStream {
public:
    // Methods need to be implemented
    virtual uint16_t getBytesAvailable() = 0u;
    virtual uint8_t readByte() = 0u;
    virtual void writeByte(uint8_t b) = 0u;

    uint16_t readWord() {
        return (uint16_t)readByte() + ((uint16_t)readByte()<<8u);
    }

#ifdef SYS_LITTLE_ENDIAN
        // Optimization for little-endian machines only!
    inline void readWordArr(int16_t *arr, uint8_t size) {
                readBuf(arr, (size*2u));
        }
#else
    void readWordArr(int16_t *arr, uint8_t size) {
            for(uint8_t i=0u; i<size; i++) {
                    arr[i] = readWord();
            }
    }
#endif

    int32_t readLong() {
        return (int32_t)readByte() + ((int32_t)readByte()<<8) + ((int32_t)readByte()<<16) + ((int32_t)readByte()<<24);
    }

    void readBuf(void* buf, uint8_t size) {
        for(uint8_t i = 0u; i < size; i++) {
            ((uint8_t*)buf)[i] = readByte();
        }
    }

    float readFloat() {
        float f;
        readBuf(&f, sizeof(float));
        return f;
    }

    void skipBytes(uint8_t size) {
        while(size-- > 0u) {
            readByte();
        }
    }

    void writeWord(int16_t w) {
        writeByte(w); // low
        writeByte(w>>8); // high
    }

#ifdef SYS_LITTLE_ENDIAN
    // Optimization for little-endian machines only!
    inline void writeWordArr(int16_t *arr, uint8_t size) {
                writeBuf(arr, (size*2u));
        }
#else
    void writeWordArr(int16_t *arr, uint8_t size) {
        for(uint8_t i=0u; i<size; i++) {
            writeWord(arr[i]);
        }
    }
#endif

    void writeLong(int32_t dw) {
        writeWord(dw); // low word
        writeWord(dw>>16); // high word
    }

    void writeFloat(float f) {
        writeBuf(&f, sizeof(float));
    }

    void writeBuf(const void* buf, uint8_t size) {
        for(uint8_t i=0u; i<size; i++) {
            writeByte(((uint8_t*)buf)[i]);
        }
    }

    void writeString(const char* str) {
        uint8_t len = strlen(str);
        writeByte(len);
        writeBuf(str, len);
    }

    void writeEmptyBuf(uint8_t size) {
        while(size-- > 0u) {
            writeByte(0u);
        }
    }
};

/* Class to manipulate command data */
class SerialCommand : public SBGC_IOStream {
public:

        uint8_t pos;
        uint8_t id;
        uint8_t data[SBGC_CMD_DATA_SIZE];
        uint8_t len;

        /* Check if limit reached after reading data buffer */
        inline uint8_t checkLimit() {
                return len == pos;
        }

        inline uint16_t getBytesAvailable() {
                return len - pos;
        }

        void init(uint8_t _id) {
                id = _id;
                len = 0u;
                pos = 0u;
        }

        inline void reset() {
                len = 0u;
                pos = 0u;
        }

        uint8_t readByte() {
                if(pos < len) {
                        return data[pos++];
                } else {
                        pos++;
                        return 0u;
                }
        }

        void writeByte(uint8_t b)  {
                if(len < sizeof(data)) {
                        data[len++] = b;
                }
        }

        static inline void init_checksum(uint8_t &checksum) {
                checksum = 0u;
        }

        static inline void update_checksum(uint8_t &checksum, uint8_t byte) {
                checksum+= byte;
        }
};

/* Need to be implemented in the main code */
class SBGC_ComObj {
public:
        // Return the number of bytes received in input buffer
        virtual uint16_t getBytesAvailable() = 0u;

        // Read byte from the input stream
        virtual uint8_t readByte() = 0u;

        // Write byte to the output stream
        virtual void writeByte(uint8_t b) = 0u;

        // Return the space available in the output buffer. Return 0xFFFF if unknown.
        virtual uint16_t getOutEmptySpace() = 0u;
};

/* Optimized version of a parser, that does not require a separate buffer for maintain the parsed data */
class SBGC_Parser {
        SBGC_ComObj *com_obj;
        enum {STATE_WAIT, STATE_GOT_MARKER, STATE_GOT_ID, STATE_GOT_LEN, STATE_GOT_HEADER, STATE_GOT_DATA } state;
        uint16_t len;
        uint8_t checksum;
        uint16_t parser_error_count;

public:
    SerialCommand in_cmd; // received command is stored here
        inline void init(SBGC_ComObj *_com_obj) {
                com_obj = _com_obj;
            state = STATE_WAIT;
                parser_error_count = 0u;
        }
        inline void onParseError(uint8_t error = PARSER_ERROR_PROTOCOL) {
                parser_error_count++;
        }

        /*
         * Parses next character in a stream.
         * Returns 1 if command successfully parsed, 0 otherwise.
         * Calls  onParseError() in case of errors
         */
        inline uint8_t process_char(uint8_t c) {
                switch(state) {
                        case STATE_WAIT:
                                if(c == SBGC_CMD_START_BYTE) {
                                        state = STATE_GOT_MARKER;
                                } else {
                                        //onParseError();
                                }
                                break;

                        case STATE_GOT_MARKER:
                                in_cmd.init(c); // got command id
                                state = STATE_GOT_ID;
                                break;

                        case STATE_GOT_ID:
                                len = c;
                                state = STATE_GOT_LEN;
                                break;

                        case STATE_GOT_LEN:
                                if (c == (uint8_t)(in_cmd.id + len) && len <= sizeof(in_cmd.data)) { // checksum and size ok
                                        SerialCommand::init_checksum(checksum);
                                        state = len == 0u ? STATE_GOT_DATA : STATE_GOT_HEADER;
                                } else {
                                        onParseError();
                                        state = STATE_WAIT;
                                }
                                break;

                        case STATE_GOT_HEADER:
                                in_cmd.data[in_cmd.len++] = c;
                                SerialCommand::update_checksum(checksum, c);
                                if(in_cmd.len == len) state = STATE_GOT_DATA;
                                break;

                        case STATE_GOT_DATA:
                                state = STATE_WAIT;
                                if(c == checksum) { // checksum ok
                                        return 1u;
                                } else {
                                        onParseError();
                                }
                               break;
                }
                return 0u;
        }

        /* Parse and fill SerialCommand object SBGC_Parser.in_cmd;
         * Returns 1 if command is parsed, 0 otherwise.
         */
        inline int8_t read_cmd() {
                while(com_obj->getBytesAvailable()) {
                        if(process_char(com_obj->readByte())) {
                                return 1;
                        }
                }
                return 0;
        }

        /*
         * Formats and writes command to serial port.
         * If wait=1, waits even if output buffer is full.
         * If wait=0, writes only if output buffer has enough space to accept this command.
         * Returns 0 on success, PARSER_ERR_xx on fail.
         */
        uint8_t send_command(uint8_t cmd_id, void *data, uint16_t size, uint8_t wait = 1u) {
                if(com_obj != NULL && size <= (SBGC_CMD_MAX_BYTES - SBGC_CMD_NON_PAYLOAD_BYTES)) {
                        if(wait || com_obj->getOutEmptySpace() >= size + SBGC_CMD_NON_PAYLOAD_BYTES) {
                                com_obj->writeByte(SBGC_CMD_START_BYTE); // protocol-specific start marker
                                com_obj->writeByte(cmd_id); // command id
                                com_obj->writeByte(size); // data body length
                                com_obj->writeByte(cmd_id + size); // header checksum
                                // Write data
                                uint8_t checksum;
                                SerialCommand::init_checksum(checksum);
                                for(uint8_t i = 0u; i < size; i++) {
                                        com_obj->writeByte(((uint8_t*)data)[i]);
                                        SerialCommand::update_checksum(checksum, ((uint8_t*)data)[i]);
                                }
                                com_obj->writeByte(checksum); // data checksum
                                return 0u;
                        } else {
                                return PARSER_ERROR_BUFFER_IS_FULL;
                        }
                } else {
                        return PARSER_ERROR_WRONG_CMD_SIZE;
                }
        }
        /*
        * Send command from the SerialCommand object to serial port. See send_command() description.
        */
        inline uint8_t send_cmd(SerialCommand &cmd, uint8_t wait = 1) {
                return send_command(cmd.id, cmd.data, cmd.len, wait);
        }

        /*
        * Get parsing errors counter
        */
        inline uint16_t get_parse_error_count() { return parser_error_count; }

        /*
        * Resets the state of a parser
        */
        inline void reset() {
                state = STATE_WAIT;
        }

        /*
        * Returns the space available in the output buffer
        */
        inline uint16_t get_out_empty_space() {
                return com_obj != NULL ? com_obj->getOutEmptySpace() : 0u;
        }
};

#ifdef __cplusplus
}
#endif

#endif //__SBGC_parser__
#endif //                                                                       end of taking out __NO_PARSER