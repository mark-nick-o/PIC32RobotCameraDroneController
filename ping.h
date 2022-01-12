#define PING_MAX_DATA_SIZE 32U

//Size of the internal buffer
//#define PING_BUFFER_SIZE (sizeof(IcmpEchoMessage) + PING_MAX_DATA_SIZE)
//typedef struct
//{
//   NetInterface *interface;
//   Socket *socket;
//   size_t dataPayloadSize;
//   uint16_t identifier;
//   uint16_t sequenceNumber;
//   systime_t timestamp;
//   systime_t timeout;
//  systime_t rtt;
//   uint8_t buffer[PING_BUFFER_SIZE];
//} PingContext;                                                                  // ping structure