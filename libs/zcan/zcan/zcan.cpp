#include "zcan.h"

ZCAN_GROUP getZCanGroup(uint32_t can_id) {
  return static_cast<ZCAN_GROUP>(can_id >> 24 & 0b1111);
}

uint8_t getZCanCommand(uint32_t can_id) {
  return can_id >> 18 & 0b111111;  
}

ZCAN_MODE getZCanMode(uint32_t can_id) {
  return static_cast<ZCAN_MODE>(can_id >> 16 & 0b11);  
}

uint16_t getZCanNetworkId(uint32_t can_id) {
  return can_id & 0xFFFF;
}

uint32_t createZCanId(ZCAN_GROUP group, uint8_t command, ZCAN_MODE mode, uint16_t networkId) {
  uint32_t raw = 0;
  raw += (static_cast<uint32_t>(1) << 31);  // dont know why this is needed ...
  raw += (static_cast<uint32_t>(1) << 28);
  raw += (static_cast<uint32_t>(group) << 24);
  raw += (static_cast<uint32_t>(command) << 18);
  raw += (static_cast<uint32_t>(mode) << 16);
  raw += networkId;
  return raw;
}

zcan_message toZCanMessage(can_frame canMsg) {
  struct zcan_message zcan;
  zcan.group = getZCanGroup(canMsg.can_id);
  zcan.command = getZCanCommand(canMsg.can_id);
  zcan.mode = getZCanMode(canMsg.can_id);
  zcan.networkId = getZCanNetworkId(canMsg.can_id);
  zcan.dataLength = canMsg.can_dlc;
  memcpy(zcan.data, canMsg.data, canMsg.can_dlc);
  return zcan;
}

can_frame toCanFrame(zcan_message zcan) {
  struct can_frame canMsg;
  canMsg.can_id = createZCanId(zcan.group, zcan.command, zcan.mode, zcan.networkId);
  canMsg.can_dlc = zcan.dataLength;
  memcpy(canMsg.data, zcan.data, zcan.dataLength);
  return canMsg;
}

void print_zcanMsg(zcan_message zcanMsg) { 
    Serial.print(zcanMsg.group); 
    Serial.print(" "); 

    Serial.print(zcanMsg.command); 
    Serial.print(" "); 

    Serial.print(zcanMsg.mode); 
    Serial.print(" "); 

    Serial.print(zcanMsg.networkId); 
    Serial.print(" | ");
   
    for (int i = 0; i<zcanMsg.dataLength; i++)  {  // print the data
      Serial.print(zcanMsg.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();   
}
