#include <SPI.h>
#include <TimerOne.h>
#include <mcp2515.h>
#include <zcan.h>

// PIN Belegung CAN-Modul -> Arduino Mega 2560
// Int -> 2
// SCK -> 52
// SI  -> 51
// SO  -> 50
// CS  -> 53
// GND -> GND
// VCC -> +5V 

// Schrittmotor Pins
#define DIR_PIN (10)  
#define PULS_PIN (11)
#define PULS_PIN_PMW (1) // = 11 on Mega 2560

// Laser Pins
#define LASER_BOTTOM_PIN (5) // lila
#define LASER_LEFT_PIN (6)   // orange
#define LASER_RIGHT_PIN (7)  // green

// Relay Pins
#define RELAY_0_PIN (30)  // not conntected
#define RELAY_1_PIN (31)
#define RELAY_2_PIN (32)
#define RELAY_3_PIN (33)
#define RELAY_4_PIN (34)
#define RELAY_5_PIN (35)
#define RELAY_6_PIN (36)
#define RELAY_7_PIN (37)
#define RELAY_8_PIN (38)
#define RELAY_9_PIN (39)
#define RELAY_10_PIN (40)
#define RELAY_11_PIN (41)

// time in ms after laser has to be free, to change the state to open
#define BLOCKED_DELAY_MS (1500)  

//  CONFIG
// stepper motor Jumper Config: 
//  SW1 - ON
//  SW2 - ON
//  SW3 - OFF
//  SW4 - OFF
//  SW5 - ON   // SW5-SS8 -> 12800 steps pro Drehung
//  SW6 - OFF  // 5mm pro Drehung
//  SW7 - OFF  // 25600 steps pro cm
//  SW8 - ON   // 

#define MICROSECONDS_PER_CYCLE (50)

// in steps
#define STEPS_E0 (-21000)
#define STEPS_E1 (69500)
#define STEPS_E2 (296500)
#define STEPS_E3 (517500)
#define STEPS_E4 (734500)
#define STEPS_E5 (961000)
#define STEPS_E6 (1185000)
#define STEPS_E7 (1402500)
#define STEPS_E8 (1631000)
#define STEPS_E9 (1850500)
#define STEPS_E10 (2072000)
#define STEPS_E11 (2292500)

// stepper motor state
volatile long actSteps;
volatile long goalSteps;
boolean up;  // true -> up, false -> down

// CAN Params
#define NETWORK_ID (40200)
#define ZUBEHOER_NID (1000)
#define PORT (1)

MCP2515 mcp2515(53); // SPI CS Pin
struct can_frame canMsg;

// state model
enum VITRINE_STATE {
  START = 0,
  LEVEL_ZERO = 1,
  MOVING = 2,
  STOPPED = 3,
  OPEN = 4,
  BLOCKED = 5,
  DETECTING = 6
};

VITRINE_STATE state;
uint8_t level;    // target ebene
uint8_t actLevel;
boolean detectingQueued;

unsigned long laserFreeTimestamp = 0;

// detection mode
#define DETECTION_TIME_MS (3000)
uint8_t detectionLevel = 0;
unsigned long detectionTimestamp = 0;


boolean isBottomLaserBlocked() {
  if (digitalRead(LASER_BOTTOM_PIN) == LOW) {
    return true;
  }
  return false;  
}

boolean isLaserBlocked() {
  if (digitalRead(LASER_LEFT_PIN) == LOW) {
    return true;
  }
  if (digitalRead(LASER_RIGHT_PIN) == LOW) {
    return true;
  }
  return false;
}

uint8_t toByteValue() {
  uint8_t result;
  if (state == VITRINE_STATE::DETECTING) {
    result = detectionLevel * 16 + state;
  } else {
    result = level * 16 + state;
  }
  return result;
}

uint8_t levelFromByteValue(uint8_t value) {
  uint8_t result = value >> 4 & 0b1111;
  return result;
}

VITRINE_STATE stateFromByteValue(uint8_t value) {
  VITRINE_STATE result = static_cast<VITRINE_STATE>(value & 0b1111);
  return result;
}

void sendCan(ZCAN_MODE mode) {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::ZUBEHOER;
  zcanMessage.command = 4;  // Command.PORT4
  zcanMessage.mode = mode;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 4;
  zcanMessage.data[0] = ZUBEHOER_NID & 0xff;;
  zcanMessage.data[1] = (ZUBEHOER_NID >> 8);
  zcanMessage.data[2] = PORT;
  zcanMessage.data[3] = toByteValue();
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

long goalStepsFromLevel() {
  if (level == 1) {
    return STEPS_E1;
  } else if (level == 2) {
    return STEPS_E2;
  } else if (level == 3) {
    return STEPS_E3;
  } else if (level == 4) {
    return STEPS_E4;
  } else if (level == 5) {
    return STEPS_E5;
  } else if (level == 6) {
    return STEPS_E6;
  } else if (level == 7) {
    return STEPS_E7;
  } else if (level == 8) {
    return STEPS_E8;
  } else if (level == 9) {
    return STEPS_E9;
  } else if (level == 10) {
    return STEPS_E10;
  } else if (level == 11) {
    return STEPS_E11;
  }
  return STEPS_E0;
}

int relayPinFromLevel(uint8_t l) {
  if (l == 1) {
    return RELAY_1_PIN;
  } else if (l == 2) {
    return RELAY_2_PIN;
  } else if (l == 3) {
    return RELAY_3_PIN;
  } else if (l == 4) {
    return RELAY_4_PIN;
  } else if (l == 5) {
    return RELAY_5_PIN;
  } else if (l == 6) {
    return RELAY_6_PIN;
  } else if (l == 7) {
    return RELAY_7_PIN;
  } else if (l == 8) {
    return RELAY_8_PIN;
  } else if (l == 9) {
    return RELAY_9_PIN;
  } else if (l == 10) {
    return RELAY_10_PIN;
  } else if (l == 11) {
    return RELAY_11_PIN;
  }
  return RELAY_0_PIN;
}

void disableAllRelays() {
  digitalWrite(RELAY_0_PIN, HIGH);
  digitalWrite(RELAY_1_PIN, HIGH);
  digitalWrite(RELAY_2_PIN, HIGH);
  digitalWrite(RELAY_3_PIN, HIGH);  
  digitalWrite(RELAY_4_PIN, HIGH);
  digitalWrite(RELAY_5_PIN, HIGH);
  digitalWrite(RELAY_6_PIN, HIGH);
  digitalWrite(RELAY_7_PIN, HIGH);  
  digitalWrite(RELAY_8_PIN, HIGH);
  digitalWrite(RELAY_9_PIN, HIGH);
  digitalWrite(RELAY_10_PIN, HIGH);
  digitalWrite(RELAY_11_PIN, HIGH);  
}

void printState() {
  Serial.print("level: ");
  Serial.print(level);
  Serial.print(" state: ");
  Serial.print(state);
  Serial.println();
}

void printStateUpdate(VITRINE_STATE newState) {
  if (state != newState) {
    Serial.print("State: ");
    Serial.print(state);
    Serial.print(" -> ");
    Serial.print(newState);
    Serial.println();
  }
}

void updateSteps() {
  if (up) {
    actSteps++;
  } else {
    actSteps--;
  }
  if (actSteps == goalSteps) {
    stopMotor();
  }
}

void startMotor() {
  Timer1.pwm(PULS_PIN_PMW, 512);
  Timer1.attachInterrupt(updateSteps);
}

void stopMotor() {
  Timer1.disablePwm(PULS_PIN_PMW);
  Timer1.detachInterrupt();
}

void updateState(VITRINE_STATE newState);

void nextDetectionLevel() {
  disableAllRelays();
  detectionLevel++;
  if (detectionLevel > 11) {
    // end detection mode
    detectionTimestamp = 0;
    detectionLevel = 0;
    digitalWrite(relayPinFromLevel(actLevel), LOW);
    updateState(VITRINE_STATE::OPEN); 
  } else {
    detectionTimestamp = millis();
    digitalWrite(relayPinFromLevel(detectionLevel), LOW);
  }
}

void updateState(VITRINE_STATE newState) {
  // printStateUpdate(newState);

  noInterrupts();
  long actStepsCopy = actSteps;
  interrupts();
  
  state = newState;
  if (state == VITRINE_STATE::START && !isBottomLaserBlocked()) {
    stopMotor();
    actSteps = 0;
    updateState(VITRINE_STATE::LEVEL_ZERO);
  } else if (state == VITRINE_STATE::LEVEL_ZERO && actStepsCopy != goalSteps) {
    startMotor();
    updateState(VITRINE_STATE::MOVING);
  } else if (state == VITRINE_STATE::MOVING && actStepsCopy == goalSteps) {
    if (level == 0) {
      updateState(VITRINE_STATE::LEVEL_ZERO);
    } else {
      actLevel = level;
      digitalWrite(relayPinFromLevel(actLevel), LOW);
      updateState(VITRINE_STATE::OPEN);
    }
  } else if (state == VITRINE_STATE::MOVING && actStepsCopy < goalSteps && !up) {
    up = true;
    digitalWrite(DIR_PIN, HIGH);
  } else if (state == VITRINE_STATE::MOVING && actStepsCopy > goalSteps && up) {
    up = false;
    digitalWrite(DIR_PIN, LOW);
  } else if (state == VITRINE_STATE::OPEN && isLaserBlocked()) {
    updateState(VITRINE_STATE::BLOCKED);
  } else if (state == VITRINE_STATE::OPEN && detectingQueued) {
    detectingQueued = false;
    updateState(VITRINE_STATE::DETECTING);
  } else if (state == VITRINE_STATE::OPEN && actStepsCopy != goalSteps) {
    disableAllRelays();
    actLevel = 0;
    startMotor();
    updateState(VITRINE_STATE::MOVING);
  } else if (state == VITRINE_STATE::BLOCKED && !isLaserBlocked()) {
    if (laserFreeTimestamp == 0) {
      laserFreeTimestamp = millis();
    } else if (millis() - laserFreeTimestamp > BLOCKED_DELAY_MS) {
      updateState(VITRINE_STATE::OPEN);
    }
  } else if (state == VITRINE_STATE::BLOCKED && isLaserBlocked()) {
    if (laserFreeTimestamp != 0) {
      laserFreeTimestamp = 0;
    }
  } else if (state == VITRINE_STATE::DETECTING && detectionLevel == 0 && detectionTimestamp == 0) {
    disableAllRelays();
    detectionTimestamp = millis();
  }   
}

void computeCommand(int levelReceived, VITRINE_STATE stateReceived) {
  if (stateReceived == VITRINE_STATE::MOVING) {
    // level of command only relevant when state moving set
    level = levelReceived;
    goalSteps = goalStepsFromLevel();
    // allowed states from wich can be switched to moving
    if (state == VITRINE_STATE::STOPPED || state == VITRINE_STATE::OPEN || state == VITRINE_STATE::LEVEL_ZERO) {
      disableAllRelays();
      actLevel = 0;
      startMotor();
      updateState(VITRINE_STATE::MOVING);
    }
  } else if (stateReceived == VITRINE_STATE::STOPPED) {
    // can only be stopped when moving
    if (state == VITRINE_STATE::MOVING) {
      stopMotor();
      updateState(VITRINE_STATE::STOPPED);
    }
  } else if (stateReceived == VITRINE_STATE::DETECTING) {
    detectingQueued = true;
  }
}

void setup() {
  // Serial.begin(115200);
  
  // init Pins
  pinMode(RELAY_0_PIN, INPUT_PULLUP);
  pinMode(RELAY_1_PIN, INPUT_PULLUP);
  pinMode(RELAY_2_PIN, INPUT_PULLUP);
  pinMode(RELAY_3_PIN, INPUT_PULLUP);
  pinMode(RELAY_4_PIN, INPUT_PULLUP);
  pinMode(RELAY_5_PIN, INPUT_PULLUP);
  pinMode(RELAY_6_PIN, INPUT_PULLUP);
  pinMode(RELAY_7_PIN, INPUT_PULLUP);  
  pinMode(RELAY_8_PIN, INPUT_PULLUP);
  pinMode(RELAY_9_PIN, INPUT_PULLUP);
  pinMode(RELAY_10_PIN, INPUT_PULLUP);
  pinMode(RELAY_11_PIN, INPUT_PULLUP);  
  
  pinMode(RELAY_0_PIN, OUTPUT);
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  pinMode(RELAY_3_PIN, OUTPUT);
  pinMode(RELAY_4_PIN, OUTPUT);
  pinMode(RELAY_5_PIN, OUTPUT);
  pinMode(RELAY_6_PIN, OUTPUT);
  pinMode(RELAY_7_PIN, OUTPUT);  
  pinMode(RELAY_8_PIN, OUTPUT);
  pinMode(RELAY_9_PIN, OUTPUT);
  pinMode(RELAY_10_PIN, OUTPUT);
  pinMode(RELAY_11_PIN, OUTPUT);  
  
  pinMode(PULS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  pinMode(LASER_BOTTOM_PIN, INPUT);
  pinMode(LASER_LEFT_PIN, INPUT);
  pinMode(LASER_RIGHT_PIN, INPUT);
  
  // init state
  level = 1;
  actLevel = 0;
  state = VITRINE_STATE::START;
  detectingQueued = false;
 
  // init CAN
  SPI.begin();
  mcp2515.reset();                          
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setConfigMode();
  mcp2515.setFilterMask(MCP2515::MASK0, true, MASK_GROUP);
  mcp2515.setFilter(MCP2515::RXF0, true, FILTER_ZUBEHOER);
  mcp2515.setNormalMode();

  // init schrittmotor
  actSteps = 0;
  goalSteps = goalStepsFromLevel();
  up = true;
  
  // start moving up 
  digitalWrite(DIR_PIN, HIGH);
  Timer1.initialize(MICROSECONDS_PER_CYCLE);
  Timer1.pwm(PULS_PIN_PMW, 512); 
}

void loop(){
  VITRINE_STATE stateBefore = state;
  uint8_t levelBefore = level;
  uint8_t detectionLevelBefor = detectionLevel;

  // poll CAN message
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    
    zcan_message zcanMsg = toZCanMessage(canMsg);   
    uint16_t nid = zcanMsg.data[0] | zcanMsg.data[1] << 8;
    
    if (nid == ZUBEHOER_NID && zcanMsg.data[2] == PORT && zcanMsg.command == 4) {
      // print_zcanMsg(zcanMsg);  // debug print CAN
      
      // answere request with current state
      if (zcanMsg.mode == ZCAN_MODE::REQUEST) {
        sendCan(ZCAN_MODE::EVENT);
      }
      
      // compute command
      if (zcanMsg.mode == ZCAN_MODE::COMMAND) {
        int levelReceived = levelFromByteValue(zcanMsg.data[3]);
        VITRINE_STATE stateReceived = stateFromByteValue(zcanMsg.data[3]);
        computeCommand(levelReceived, stateReceived);
      }
    }
  }

  // when in detection mode
  if (detectionTimestamp > 0) {
    // after detection time switch to next level
    if (millis() - detectionTimestamp > DETECTION_TIME_MS) {
      nextDetectionLevel();
    }
  }
  
  // check programatic state changes
  updateState(state);

  if (stateBefore != state || levelBefore != level || detectionLevelBefor != detectionLevel) {
    sendCan(ZCAN_MODE::EVENT);
    // printState();
  }
}
