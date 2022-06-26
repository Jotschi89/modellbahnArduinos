#include <SPI.h>
#include <mcp2515.h>
#include <zcan.h>

// PIN Belegung 
// CAN-Module -> Arduino Mega 2560
// Int -> 2
// SCK -> 52
// SI  -> 51
// SO  -> 50
// CS  -> 53
// GND -> GND
// VCC -> +5V 

// todo: set NETWORK_ID and ZUBEHOER_NID
#define NETWORK_ID ()
#define ZUBEHOER_NID ()

MCP2515 mcp2515(53); // SPI CS Pin
struct can_frame canMsg;

// todo: edit this function as needed
void sendCan(ZCAN_MODE mode, uint8_t dataLength, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7) {
  struct zcan_message zcantest;
  zcantest.group = ZCAN_GROUP::ZUBEHOER;
  zcantest.command = 6;
  zcantest.mode = mode;
  zcantest.networkId = NETWORK_ID;
  zcantest.dataLength = dataLength + 2;
  zcantest.data[0] = ZUBEHOER_NID & 0xff;;
  zcantest.data[1] = (ZUBEHOER_NID >> 8);
  zcantest.data[2] = data2;
  zcantest.data[3] = data3;
  zcantest.data[4] = data4;
  zcantest.data[5] = data5;
  zcantest.data[6] = data6;
  zcantest.data[7] = data7;
  mcp2515.sendMessage(&toCanFrame(zcantest));
}

void setup() {
  SPI.begin();
  Serial.begin(115200);

  mcp2515.reset();                          
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setConfigMode();
  // MASK0 -> RXF0, RXF1; MASK1 -> RXF2, RXF3, RXF4, RXF5
  mcp2515.setFilterMask(MCP2515::MASK0, true, MASK_GROUP);
  // todo: adapte filters to usecase
  mcp2515.setFilter(MCP2515::RXF0, true, FILTER_ZUBEHOER);
  mcp2515.setNormalMode(); 
}

void loop(){
  // poll CAN message
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    zcan_message zcanMsg = toZCanMessage(canMsg);   
    uint16_t nid = zcanMsg.data[0] | zcanMsg.data[1] << 8;
    if (nid == ZUBEHOER_NID) {
      print_zcanMsg(zcanMsg);  // debug print CAN
      if (zcanMsg.mode == REQUEST) {
        // todo
      }
      if (zcanMsg.mode == COMMAND) {
        // todo
      }
      // todo: remove dummy send
      sendCan(ZCAN_MODE::ACKNOWLEDGE, 4, 0x12, 0x0F, 0x14, 0, 0, 0);
    }
  }

  
}
