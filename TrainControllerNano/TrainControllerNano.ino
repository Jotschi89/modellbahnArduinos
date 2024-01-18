#include <SPI.h>
#include <mcp2515.h>
#include <zcan.h>
#include <math.h>
#include <ButtonDebounce.h>

// PIN Belegung 
// CAN-Module -> Arduino NANO
// Int -> 2
// SCK -> 13
// SI  -> 11
// SO  -> 12
// CS  -> 10
// GND -> GND
// VCC -> +5V 

#define NETWORK_ID (40111)   // train controller 1

MCP2515 mcp2515(10); // SPI CS Pin
struct can_frame canMsg;

/* PIN DEFINITIONS */

// Connection a series of 74HC595 chips (serial-to-parallel converter)
#define SERIALIZED_LATCH_PIN A4 // ST_CP
#define SERIALIZED_CLOCK_PIN A3 // SH_CP
#define SERIALIZED_DATA_PIN A5 // DS

#define PREV_LOCO_PIN A1
#define NEXT_LOCO_PIN A0

#define ROTARY_CLOCK_PIN 5 // CLK, is the primary output pulse used to determine the amount of rotation. Each time the knob is turned in either direction by just one detent (click), the ‘CLK’ output goes through one cycle of going HIGH and then LOW.
#define ROTARY_DATA_PIN 4 // DT, is similar to CLK output, but it lags behind CLK by a 90° phase shift. This output is used to determine the direction of rotation.
#define ROTARY_BUTTON_PIN 3 // SW

#define FUNC_BUTTON_AMOUNT 4
const byte FUNC_BUTTON_PINS[FUNC_BUTTON_AMOUNT] = {6, 7, 8, 9};


/* CONFIGURATION */

#define MAX_SPEED_STEP 5
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

enum BUTTON {
  B_FUNC_1 = 0,
  B_FUNC_2 = 1,
  B_FUNC_3 = 2,
  B_FUNC_4 = 3,
  B_PREV_LOCO = 4,
  B_NEXT_LOCO = 5,
  B_SPEED_ZERO = 6
};

// buttons
#define BUTTON_DEBOUNCE_MS 50
ButtonDebounce buttons[] = {
  ButtonDebounce(FUNC_BUTTON_PINS[0], BUTTON_DEBOUNCE_MS),
  ButtonDebounce(FUNC_BUTTON_PINS[1], BUTTON_DEBOUNCE_MS),
  ButtonDebounce(FUNC_BUTTON_PINS[2], BUTTON_DEBOUNCE_MS),
  ButtonDebounce(FUNC_BUTTON_PINS[3], BUTTON_DEBOUNCE_MS),
  ButtonDebounce(PREV_LOCO_PIN, BUTTON_DEBOUNCE_MS),
  ButtonDebounce(NEXT_LOCO_PIN, BUTTON_DEBOUNCE_MS),
  ButtonDebounce(ROTARY_BUTTON_PIN, BUTTON_DEBOUNCE_MS),
};

byte rotaryClockState = HIGH;
unsigned long roataryClockChangedTimer = 0;

/* SERIALIZED OUTPUT */
#define SERIALIZED_BYTE_SIZE 5
byte serializedData[SERIALIZED_BYTE_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0x00};

/* CONTROL STATES */
byte locoSelect = 0;
int8_t speed[LOCO_AMOUNT];
bool locoKnown[LOCO_AMOUNT];
uint8_t locoFunctionStateLow[LOCO_AMOUNT];
uint8_t locoFunctionStateHigh[LOCO_AMOUNT];
byte funcButtonLights[FUNC_BUTTON_AMOUNT] = {LOW, LOW, LOW, LOW};
bool fDisplays = false;

int8_t toDCCSpeed(short canSpeed) {
  return round(canSpeed / 8.119);
}

short toCanSpeed(int8_t dccSpeed) {
   return ceil(abs(dccSpeed) * 8.119);
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

void flushDisplays() {
  setThreeDigitDisplay(speed[locoSelect], 0, TYPE_SIGNED);
  setThreeDigitDisplay(locoSelect, 16, TYPE_UNSIGNED);

  // Func Button Lights
  setSerialized(funcButtonLights[0], 36, 1);
  setSerialized(funcButtonLights[1], 34, 1);
  setSerialized(funcButtonLights[2], 38, 1);
  setSerialized(funcButtonLights[3], 33, 1);
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

void writeSerializedData(byte data[], byte arraySize) {
  for (byte i = arraySize; i > 0; i--) {
    shiftOut(SERIALIZED_DATA_PIN, SERIALIZED_CLOCK_PIN, MSBFIRST, data[i - 1]);
  }
  digitalWrite(SERIALIZED_LATCH_PIN, HIGH);
  digitalWrite(SERIALIZED_LATCH_PIN, LOW);
}

void flushSerialized() {
  writeSerializedData(serializedData, SERIALIZED_BYTE_SIZE);
}

void activateLok(byte lokID) {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::FAHRZEUGE;
  zcanMessage.command = 16;  //  16 -> lok aktivieren
  zcanMessage.mode = ZCAN_MODE::COMMAND;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 3;
  zcanMessage.data[0] = lokID;
  zcanMessage.data[1] = 0;
  zcanMessage.data[2] = 16;
  
  mcp2515.sendMessage(&toCanFrame(zcanMessage));  
}

void requestCanLokSpeed(byte lokID) {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::FAHRZEUGE;
  zcanMessage.command = 2;  // 2 -> speed command
  zcanMessage.mode = ZCAN_MODE::REQUEST;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 4;
  zcanMessage.data[0] = lokID;
  zcanMessage.data[1] = 0;
  zcanMessage.data[2] = 0;
  zcanMessage.data[3] = 0;
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void requestCanLokFunctionState(byte lokID) {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::FAHRZEUGE;
  zcanMessage.command = 3;  // 3 -> fahrzeug schalten command
  zcanMessage.mode = ZCAN_MODE::REQUEST;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 4;
  zcanMessage.data[0] = lokID;
  zcanMessage.data[1] = 0;
  zcanMessage.data[2] = 0;
  zcanMessage.data[3] = 0;
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void sendCanLokSpeed(byte lokID, int8_t dccSpeed) {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::FAHRZEUGE;
  zcanMessage.command = 2;  // 2 -> speed command
  zcanMessage.mode = ZCAN_MODE::COMMAND;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 4;
  zcanMessage.data[0] = lokID;
  zcanMessage.data[1] = 0;

  bool vor = true;
  if (dccSpeed < 0) {
    vor = false;
  }
  byte dirByte = B00001100;
  short canSpeed = toCanSpeed(dccSpeed);
  zcanMessage.data[2] = canSpeed % 256;
  zcanMessage.data[3] = ((canSpeed >> 8) & B00000011) | (vor ? 0 : dirByte);
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void sendCanLokFunctionState(byte lokID, byte buttonNr, bool state) {
  // buttonNr in range 0-3
  if (state) {
    bitSet(locoFunctionStateLow[lokID], buttonNr);
  } else {
    bitClear(locoFunctionStateLow[lokID], buttonNr);
  }
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::FAHRZEUGE;
  zcanMessage.command = 3;  //  3 -> fahrzeug schalten command
  zcanMessage.mode = ZCAN_MODE::COMMAND;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 6;
  zcanMessage.data[0] = lokID;
  zcanMessage.data[1] = 0;
  zcanMessage.data[2] = locoFunctionStateLow[lokID];
  zcanMessage.data[3] = locoFunctionStateHigh[lokID];
  zcanMessage.data[4] = 0;
  zcanMessage.data[5] = 0;
  
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void funcButtonLightUpdate() {
  for (byte i = 0; i < FUNC_BUTTON_AMOUNT; i++) {
    funcButtonLights[i] = bitRead(locoFunctionStateLow[locoSelect], i);
  }
}

void funcButtonDown(short buttonNr) {
  if (buttonNr < 2 && funcButtonLights[buttonNr] == HIGH) {
    sendCanLokFunctionState(locoSelect, buttonNr, LOW);
  } else {
    sendCanLokFunctionState(locoSelect, buttonNr, HIGH);
  }
}

void funcButtonUp(short buttonNr) {
  if (buttonNr >= 2) {
    sendCanLokFunctionState(locoSelect, buttonNr, LOW);
  }
}

void prevLocoButtonChanged(const int state) {
  if (state == LOW) {
    do {
      if (locoSelect == 0) {
        locoSelect = LOCO_AMOUNT - 1;
      } else {
        locoSelect--;
      }
    } while(locoKnown[locoSelect] == false);
    requestCanLokSpeed(locoSelect);
    requestCanLokFunctionState(locoSelect);
    activateLok(locoSelect);
    fDisplays = true;
  }
}

void nextLocoButtonChanged(const int state) {
  if (state == LOW) {
    do {
      if (locoSelect == LOCO_AMOUNT - 1) {
        locoSelect = 0;
      } else {
        locoSelect++;
      }
    } while(locoKnown[locoSelect] == false);
    requestCanLokSpeed(locoSelect);
    requestCanLokFunctionState(locoSelect);
    activateLok(locoSelect);
    fDisplays = true;
  }
}

void functionButtonChanged(const int state, byte buttonNr) {
  if (state == LOW) {
    funcButtonDown(buttonNr);
  } else if (state == HIGH) {
    funcButtonUp(buttonNr);
  }
  funcButtonLightUpdate();
  fDisplays = true;
}

void functionButton1Changed(const int state) {
  functionButtonChanged(state, 0);
}

void functionButton2Changed(const int state) {
  functionButtonChanged(state, 1);
}

void functionButton3Changed(const int state) {
  functionButtonChanged(state, 2);
}

void functionButton4Changed(const int state) {
  functionButtonChanged(state, 3);
}

void zeroButtonChanged(const int state) {
  if (state == LOW) {
    speed[locoSelect] = 0;
    sendCanLokSpeed(locoSelect, speed[locoSelect]);
    fDisplays = true;
  }
}

void setup() {
  pinMode(SERIALIZED_LATCH_PIN, OUTPUT);
  pinMode(SERIALIZED_CLOCK_PIN, OUTPUT);
  pinMode(SERIALIZED_DATA_PIN, OUTPUT);

  pinMode(PREV_LOCO_PIN, INPUT_PULLUP);
  pinMode(NEXT_LOCO_PIN, INPUT_PULLUP);

  pinMode(ROTARY_CLOCK_PIN, INPUT);
  pinMode(ROTARY_DATA_PIN, INPUT);
  pinMode(ROTARY_BUTTON_PIN, INPUT_PULLUP);

  for (byte i = 0; i < FUNC_BUTTON_AMOUNT; i++) {
    pinMode(FUNC_BUTTON_PINS[i], INPUT_PULLUP);
  }

  buttons[BUTTON::B_FUNC_1].setCallback(functionButton1Changed);
  buttons[BUTTON::B_FUNC_2].setCallback(functionButton2Changed);
  buttons[BUTTON::B_FUNC_3].setCallback(functionButton3Changed);
  buttons[BUTTON::B_FUNC_4].setCallback(functionButton4Changed);
  buttons[BUTTON::B_PREV_LOCO].setCallback(prevLocoButtonChanged);
  buttons[BUTTON::B_NEXT_LOCO].setCallback(nextLocoButtonChanged);
  buttons[BUTTON::B_SPEED_ZERO].setCallback(zeroButtonChanged);

  rotaryClockState = digitalRead(ROTARY_CLOCK_PIN);

  for (byte i = 0; i < LOCO_AMOUNT; i++) {
    speed[i] = 0;  
    locoFunctionStateLow[i] = 0;  
    locoFunctionStateHigh[i] = 0;  
    locoKnown[i] = false;
  }
  locoKnown[0] = true;

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
  unsigned long t = millis();
  fDisplays = false;

  buttons[BUTTON::B_SPEED_ZERO].update();
  buttons[BUTTON::B_PREV_LOCO].update();
  buttons[BUTTON::B_NEXT_LOCO].update();
  buttons[BUTTON::B_FUNC_1].update();
  buttons[BUTTON::B_FUNC_2].update();
  buttons[BUTTON::B_FUNC_3].update();
  buttons[BUTTON::B_FUNC_4].update();

  // SPEED KNOB
  byte rotaryClock = digitalRead(ROTARY_CLOCK_PIN);
  // prevent spam of speed messages by including a small waiting time
  if (rotaryClock != rotaryClockState && t - roataryClockChangedTimer > 5) {
    int8_t currentSpeedStep = MAX_SPEED_STEP;
    if (speed[locoSelect] < 4 && speed[locoSelect] > -4) {
      currentSpeedStep = 1;
    } else if (speed[locoSelect] < 10 && speed[locoSelect] > -10) {
      // if the update was shortly before, increase the step size
      currentSpeedStep = 2;
    }
    roataryClockChangedTimer = t;
    rotaryClockState = rotaryClock;
    if (digitalRead(ROTARY_DATA_PIN) != rotaryClock) {
      // go UP
      speed[locoSelect] = speed[locoSelect] + currentSpeedStep;
      if (speed[locoSelect] > SPEED_MAX) {
        speed[locoSelect] = SPEED_MAX;
      } else if(speed[locoSelect] > 0 && speed[locoSelect] < currentSpeedStep) {
        // coming from negative - let's not step over 0
        speed[locoSelect] = 0;
      }
    } else {
      // go DOWN
      speed[locoSelect] = speed[locoSelect] - currentSpeedStep;
      if (speed[locoSelect] < -SPEED_MAX) {
        speed[locoSelect] = -SPEED_MAX;
      } else if(speed[locoSelect] < 0 && speed[locoSelect] > -currentSpeedStep) {
        // coming from positive - let's not step over 0
        speed[locoSelect] = 0;
      }
    }
    sendCanLokSpeed(locoSelect, speed[locoSelect]);
    fDisplays = true;
  }

  // poll CAN message
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    zcan_message zcanMsg = toZCanMessage(canMsg);
    if (zcanMsg.mode == ACKNOWLEDGE || (zcanMsg.mode == COMMAND && zcanMsg.networkId != NETWORK_ID)) {
      byte lokID = zcanMsg.data[0];  // lokid
      locoKnown[lokID] = true;
      // speed message
      if (zcanMsg.command == 2) {
        byte dirByte = B00001100;
        
        short speedValue = zcanMsg.data[2] + ((zcanMsg.data[3] & B00000011) << 8);
        speedValue = toDCCSpeed(speedValue);
        bool vor = (zcanMsg.data[3] & dirByte) == 0;
        if (!vor) {
          speedValue = -speedValue;
        }
        speed[lokID] = speedValue;
      }
      // function state message
      else if (zcanMsg.command == 3) {               
        locoFunctionStateLow[lokID] = zcanMsg.data[2];
        locoFunctionStateHigh[lokID] = zcanMsg.data[3];
      }
      if (locoSelect == lokID) {
        funcButtonLightUpdate();
        fDisplays = true;
      }
    }
  }

  if (fDisplays) {
    flushDisplays();
  }
}
