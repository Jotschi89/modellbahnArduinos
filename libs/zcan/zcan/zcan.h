#ifndef ZCAN_H
#define ZCAN_H

#include <SPI.h>
#include <mcp2515.h>

// MASK0 -> RXF0, RXF1
// MASK1 -> RXF2, RXF3, RXF4, RXF5
#define MASK_GROUP (0b00011111000000000000000000000000)
#define FILTER_SYSTEM (0b00010000000000000000000000000000)
#define FILTER_ZUBEHOER (0b00010001000000000000000000000000)
#define FILTER_FAHRZEUGE (0b00010010000000000000000000000000)
#define FILTER_DATA (0b00010111000000000000000000000000)
#define FILTER_INFO (0b00011000000000000000000000000000)
#define FILTER_NETWORK (0b00011010000000000000000000000000)


enum ZCAN_GROUP {
  SYSTEM = 0,
  ZUBEHOER = 1,
  FAHRZEUGE = 2,
  RESERVED = 3,
  RCS = 4,
  CONFIG = 5,
  TRACK_CFG = 6,
  DATA = 7,
  INFO = 8,
  FREE = 9,
  NETWORK = 10
};

enum ZCAN_MODE {
  REQUEST = 0,
  COMMAND = 1,
  EVENT = 2,
  ACKNOWLEDGE = 3
};

struct zcan_message {
    ZCAN_GROUP group;
    uint8_t command;
    ZCAN_MODE mode;
    uint16_t networkId;
    uint8_t dataLength;
    uint8_t data[8];
};


ZCAN_GROUP getZCanGroup(uint32_t can_id);
uint8_t getZCanCommand(uint32_t can_id);
ZCAN_MODE getZCanMode(uint32_t can_id);
uint16_t getZCanNetworkId(uint32_t can_id);
uint32_t createZCanId(ZCAN_GROUP group, uint8_t command, ZCAN_MODE mode, uint16_t networkId);
zcan_message toZCanMessage(can_frame canMsg);
can_frame toCanFrame(zcan_message zcan);
void print_zcanMsg(zcan_message zcanMsg);

#endif
