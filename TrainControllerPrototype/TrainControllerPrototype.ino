#include <SPI.h>
#include <mcp2515.h>
#include <zcan.h>
#include <math.h>

// PIN Belegung 
// CAN-Module -> Arduino Mega 2560
// Int -> 2
// SCK -> 52
// SI  -> 51
// SO  -> 50
// CS  -> 53
// GND -> GND
// VCC -> +5V 

#define NETWORK_ID (40999)   // todo: set NETWORK_ID to non-prototype values

MCP2515 mcp2515(53); // SPI CS Pin
struct can_frame canMsg;

/* PIN DEFINITIONS */

// Connection a series of 74HC595 chips (serial-to-parallel converter)
#define SERIALIZED_LATCH_PIN 3 // ST_CP
#define SERIALIZED_CLOCK_PIN 4 // SH_CP
#define SERIALIZED_DATA_PIN 7 // DS

#define BUTTON_DOWN_PIN 5
#define BUTTON_UP_PIN 6

#define ROTARY_CLOCK_PIN 11 // CLK
#define ROTARY_DATA_PIN 12 // DT
#define ROTARY_BUTTON_PIN 13 // SW


/* CONFIGURATION */

#define SPEED_STEP 5
#define SPEED_MAX 100
#define LOCO_AMOUNT 128


/* CONSTANTS */

/* seven-segment display digit mapping
 * COMMON ANODE (0 = segment on, 1 = segment off)
 * bit       (LSB)0,1,2,3,4,5,6(MSB)
 * Mapped to      a,b,c,d,e,f,g of seven-segment display
 */
const byte digit2SevenSegment[12] = {
  B1000000, // 0
  B1111001, // 1
  B0100100, // 2
  B0110000, // 3
  B0011001, // 4
  B0010010, // 5
  B0000010, // 6
  B1111000, // 7
  B0000000, // 8
  B0010000, // 9
  B0111111, // '-'
  B1111111  // empty
};
/* two-segment display digit mapping
 * (which is a seven-segment display with only some segments connected in HW)
 * bit 0 => b&c ('1'); bit 1 => g ('-');
 */
const byte digit2TwoSegment[13] = {
  B11, // 0 - same as empty
  B10, // 1
  B00, // 2 (2-9 should not occur => light up all available segments to signal error)
  B00, // 3
  B00, // 4
  B00, // 5
  B00, // 6
  B00, // 7
  B00, // 8
  B00, // 9
  B01, // '-'
  B11, // empty
  B00  // '-1'
};

#define TYPE_UNSIGNED 0x01
#define TYPE_SIGNED 0x02


/* SERIALIZED OUTPUT */
#define SERIALIZED_BYTE_SIZE 4
byte serializedData[SERIALIZED_BYTE_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF};

/* INPUT STATES */
byte buttonDownState = HIGH;
byte buttonUpState = HIGH;

byte rotaryClockState = HIGH;
byte rotaryButtonState = HIGH;

/* CONTROL STATES */
int8_t speed[LOCO_AMOUNT];
byte locoSelect = 60;

void requestCanLokSpeed(byte lokID) {
  struct zcan_message zcantest;
  zcantest.group = ZCAN_GROUP::FAHRZEUGE;
  zcantest.command = 2;  // 2 -> speed command
  zcantest.mode = ZCAN_MODE::REQUEST;
  zcantest.networkId = NETWORK_ID;
  zcantest.dataLength = 4;
  zcantest.data[0] = lokID;
  zcantest.data[1] = 0;
  zcantest.data[2] = 0;
  zcantest.data[3] = 0;
  mcp2515.sendMessage(&toCanFrame(zcantest));
}

void sendCanLokSpeed(byte lokID, short speedValue) {
  struct zcan_message zcantest;
  zcantest.group = ZCAN_GROUP::FAHRZEUGE;
  zcantest.command = 2;  // 2 -> speed command
  zcantest.mode = ZCAN_MODE::COMMAND;
  zcantest.networkId = NETWORK_ID;
  zcantest.dataLength = 4;
  zcantest.data[0] = lokID;
  zcantest.data[1] = 0;

  bool vor = true;
  if (speedValue < 0) {
    vor = false;
  }
  byte dirByte = B00001100;
  speedValue = abs(speedValue * 7.88);
  zcantest.data[2] = speedValue % 256;
  zcantest.data[3] = ((speedValue >> 8) & B00000011) | (vor ? 0 : dirByte);
  mcp2515.sendMessage(&toCanFrame(zcantest));
}

void setup() {
  pinMode(SERIALIZED_LATCH_PIN, OUTPUT);
  pinMode(SERIALIZED_CLOCK_PIN, OUTPUT);
  pinMode(SERIALIZED_DATA_PIN, OUTPUT);

  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);

  pinMode(ROTARY_CLOCK_PIN, INPUT);
  pinMode(ROTARY_DATA_PIN, INPUT);
  pinMode(ROTARY_BUTTON_PIN, INPUT_PULLUP);

  rotaryClockState = digitalRead(ROTARY_CLOCK_PIN);

  for (byte i = 0; i < LOCO_AMOUNT; i++) {
    speed[i] = 0;    
  }

  flushDisplays();

  // CAN BUS
  SPI.begin();
  Serial.begin(115200);

  mcp2515.reset();                          
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setConfigMode();
  // MASK0 -> RXF0, RXF1; MASK1 -> RXF2, RXF3, RXF4, RXF5
  mcp2515.setFilterMask(MCP2515::MASK0, true, MASK_GROUP);
  mcp2515.setFilter(MCP2515::RXF0, true, FILTER_FAHRZEUGE);
  mcp2515.setNormalMode(); 
}

void loop() {
  // LOCO SELECT
  byte buttonDown = digitalRead(BUTTON_DOWN_PIN);
  if (buttonDown != buttonDownState) {
    if (buttonDown == LOW) {
      if (locoSelect > 0) {
        locoSelect--;
      } else {
        locoSelect = LOCO_AMOUNT - 1;
      }
      requestCanLokSpeed(locoSelect);
      flushDisplays();
    }
    // debounce button press & button release
    delay(1);
  }
  buttonDownState = buttonDown;
  
  byte buttonUp = digitalRead(BUTTON_UP_PIN);
  if (buttonUp != buttonUpState) {
    if (buttonUp == LOW) {
      if (locoSelect < LOCO_AMOUNT - 1) {
        locoSelect++;
      } else {
        locoSelect = 0;
      }
      requestCanLokSpeed(locoSelect);
      flushDisplays();
    }
    delay(1);
  }
  buttonUpState = buttonUp;

  // SPEED KNOB
  byte rotaryClock = digitalRead(ROTARY_CLOCK_PIN);
  if (rotaryClock != rotaryClockState) {
    if (rotaryClock == LOW) {
      if (digitalRead(ROTARY_DATA_PIN) == LOW) {
        // go UP
        speed[locoSelect] += SPEED_STEP;
        if (speed[locoSelect] > SPEED_MAX) {
          speed[locoSelect] = SPEED_MAX;
        } else if(speed[locoSelect] > 0 && speed[locoSelect] < SPEED_STEP) {
          // coming from negative - let's not step over 0
          speed[locoSelect] = 0;
        }
      } else {
        // go DOWN
        speed[locoSelect] -= SPEED_STEP;
        if (speed[locoSelect] < -SPEED_MAX) {
          speed[locoSelect] = -SPEED_MAX;
        } else if(speed[locoSelect] < 0 && speed[locoSelect] > -SPEED_STEP) {
          // coming from positive - let's not step over 0
          speed[locoSelect] = 0;
        }
      }
      sendCanLokSpeed(locoSelect, speed[locoSelect]);
      flushDisplays();
    }
    delay(4);
  }
  rotaryClockState = rotaryClock;
  
  byte rotaryButton = digitalRead(ROTARY_BUTTON_PIN);
  if (rotaryButton != rotaryButtonState) {
    if (rotaryButton == LOW) {
      speed[locoSelect] = 0;
      sendCanLokSpeed(locoSelect, speed[locoSelect]);
      flushDisplays();
    }
    delay(1);
  }
  rotaryButtonState = rotaryButton;

  // count numbers up/down
  /*for (byte number = 0;; number++) {
    setThreeDigitDisplay(number, 0);
    setThreeDigitDisplay(255 - number, 16);
    flushSerialized();
    delay(100);
  }//*/

  // poll CAN message
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    zcan_message zcanMsg = toZCanMessage(canMsg);
    if (zcanMsg.mode == ACKNOWLEDGE || (zcanMsg.mode == COMMAND && zcanMsg.networkId != NETWORK_ID)) {
      byte lokID = zcanMsg.data[0];  // lokid
      byte dirByte = B00001100;
      
      short speedValue = zcanMsg.data[2] + ((zcanMsg.data[3] & B00000011) << 8);
      speedValue = round(speedValue / 7.88);
      bool vor = (zcanMsg.data[3] & dirByte) == 0;
      if (!vor) {
        speedValue = -speedValue;
      }
      speed[lokID] = speedValue;
      if (locoSelect == lokID) {
        flushDisplays();
      }
    }
  }
}


void flushDisplays() {
  setThreeDigitDisplay(speed[locoSelect], 0, TYPE_SIGNED);
  setThreeDigitDisplay(locoSelect, 16, TYPE_UNSIGNED);
  flushSerialized();
}

void setSerializedBytes(byte data[], byte bitPosition, byte bitLength) {
  byte offset = bitPosition;
  for (byte i = 0; i < bitLength; i++) {
    if (bitRead(data[i / 8], i % 8) == 0x01) {
      bitSet(serializedData[offset / 8], offset % 8);
    } else {
      bitClear(serializedData[offset / 8], offset % 8);
    }
    offset++;
  }
}

void setSerialized(byte data, byte bitPosition, byte bitLength) {
  byte offset = bitPosition;
  for (byte i = 0; i < bitLength; i++) {
    if (bitRead(data, i) == 0x01) {
      bitSet(serializedData[offset / 8], offset % 8);
    } else {
      bitClear(serializedData[offset / 8], offset % 8);
    }
    offset++;
  }
}

void flushSerialized() {
  writeSerializedData(serializedData, SERIALIZED_BYTE_SIZE);
}

void writeSerializedData(byte data[], byte arraySize) {
  for (byte i = arraySize; i > 0; i--) {
    shiftOut(SERIALIZED_DATA_PIN, SERIALIZED_CLOCK_PIN, MSBFIRST, data[i - 1]);
  }
  digitalWrite(SERIALIZED_LATCH_PIN, HIGH);
  digitalWrite(SERIALIZED_LATCH_PIN, LOW);
}

void setThreeDigitDisplay(byte number, byte bitPosition, byte type) {
  byte one;
  byte ten;
  byte hundred;
  bool isNegative = false;

  if (type == TYPE_SIGNED) {
    // dealing with actual signed, but type given is unsigned - that's why this looks a bit strange
    isNegative = number > 127;
    if (isNegative) {
      // revert two's complement
      number = ~number + 1;
    }
  }

  if (number > 199) {
    // set '-'
    one = ten = hundred = 10;
  } else {
    one = number % 10;
    if (number < 10) {
      // set empty
      ten = 11;
    } else {
      ten = (number / 10) % 10;
    }
    if (isNegative) {
      if (number > 99) {
        // set '-1'
        hundred = 12;
      } else {
        // set '-';
        hundred = 10;
      }
    } else {
      // Note: 0 is displayed as empty (see digit2TwoSegment array)
      hundred = number / 100;
    }
  }

  setSerialized(digit2SevenSegment[one], bitPosition, 7);
  setSerialized(digit2SevenSegment[ten], bitPosition + 7, 7);
  setSerialized(digit2TwoSegment[hundred], bitPosition + 14, 2);
}
