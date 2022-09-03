#include <SPI.h>
#include <mcp2515.h>
#include <zcan.h>
#include <TimerOne.h>

// PIN Belegung 
// CAN-Module -> Arduino NANO
// Int -> 2
// SCK -> 13
// SI  -> 11
// SO  -> 12
// CS  -> 10
// GND -> GND
// VCC -> +5V 

#define NETWORK_ID (40101)
#define VITRINE_ZUBEHOER_NID (1000)
#define COMPUTER_NID (40100)
#define Z21_NID (49200)

#define LED_Z21_RED (1)
#define LED_Z21_BLUE (0)
#define LED_PC_OFFLINE (3)
#define LED_ENTGLEISUNG (4)
#define LED_BRUECKE (5)
#define LED_KOLLISION (6)
#define LED_VITRINE_ZUFAHRT (7)
#define LED_LAN_OFFLINE (8)
#define BUZZER (9)
#define LED_MANUEL (14)     // A0
#define LED_ASSISTENT (15)  // A1
#define LED_AUTO (16)       // A2

enum FAHRMODUS {
  MANUEL = 0,
  ASSISTENT = 64,
  AUTO = 128
};

enum BUTTON {
  B_MANUEL = 0,
  B_ASSISTENT = 1,
  B_AUTO = 2,
  RESET = 3,
  NOTAUS = 4
};

enum ERROR_MESSAGE {
  PC_OFFLINE_ERROR = 1,
  LAN_OFFLINE_ERROR = 2,
  ENTGLEISUNG = 4,  
  BRUECKE_OFFEN = 8,    
  KOLLISION_WARNUNG = 16,   
  VITRINEN_ZUFAHRT_WARNUNG = 32,     
};

// CAN
MCP2515 mcp2515(10); // SPI CS Pin
struct can_frame canMsg;

// buzzer timer / state
unsigned long buzzerAus = 0;
byte buzzerCounter = 0;
int buzzerMS = 0;
byte buzzerPitch = 0;
bool analogOutON = false;

// blue z21 button state
unsigned long lastBlueZ21Blink = 0;
bool blueZ21State = false;

// button stablisation state
BUTTON stableButton = BUTTON::NOTAUS;
int stableButtonCounter = 0;
unsigned long buttonPressedTimer = 0;

// computer alive timer
unsigned long lastComputerAlive = 0;

// bruecken
int brueckenConnectionCounter = 0;
unsigned long brueckeSchnitt = 0;

// state model
FAHRMODUS fahrmodus = -1;
int errorState = 0;
bool gleisStromAus = true;
bool kurzschluss = false;


void playBuzzerSound(int ms, int count, int pitch) {
  buzzerMS = ms;
  buzzerAus = millis() + buzzerMS;
  buzzerCounter = count;
  buzzerPitch = pitch;
}

void sendReset() {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::SYSTEM;
  zcanMessage.command = 0;
  zcanMessage.mode = ZCAN_MODE::COMMAND;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 4;
  zcanMessage.data[0] = 0;
  zcanMessage.data[1] = 0;
  zcanMessage.data[2] = 0;
  zcanMessage.data[3] = 1;
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void sendNotaus() {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::SYSTEM;
  zcanMessage.command = 0;
  zcanMessage.mode = ZCAN_MODE::COMMAND;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 4;
  zcanMessage.data[0] = 0;
  zcanMessage.data[1] = 0;
  zcanMessage.data[2] = 0;
  zcanMessage.data[3] = 4;
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void sendNotausVitrine() {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::ZUBEHOER;
  zcanMessage.command = 4;
  zcanMessage.mode = ZCAN_MODE::COMMAND;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 4;
  zcanMessage.data[0] = VITRINE_ZUBEHOER_NID & 0xff;;
  zcanMessage.data[1] = (VITRINE_ZUBEHOER_NID >> 8);
  zcanMessage.data[2] = 1;
  zcanMessage.data[3] = 3;  // VITRINE_STATE::STOPPED  & (ebene 0)
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void sendStatus() {
   struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::INFO;
  zcanMessage.command = 0;
  zcanMessage.mode = ZCAN_MODE::EVENT;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 1;
  // data byte bits: 
  // [Fahrmodus.AUTO, Fahrmodus.ASSISTENT, VITRINEN_ZUFAHRT_WARNUNG, KOLLISION_WARNUNG, BRUECKE_OFFEN, ENTGLEISUNG, LAN_OFFLINE_ERROR, CAN/PC_OFFLINE_ERROR]
  zcanMessage.data[0] = errorState + fahrmodus;
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void updateFahrmodus(FAHRMODUS newModus) {
  if (fahrmodus == newModus) {
    return;
  }
  fahrmodus = newModus;
  if (fahrmodus == FAHRMODUS::MANUEL) {
    playBuzzerSound(250, 3, 50);
    digitalWrite(LED_MANUEL, HIGH);
    digitalWrite(LED_ASSISTENT, LOW);
    digitalWrite(LED_AUTO, LOW);
  } else if (fahrmodus == FAHRMODUS::ASSISTENT) {
    digitalWrite(LED_MANUEL, LOW);
    digitalWrite(LED_ASSISTENT, HIGH);
    digitalWrite(LED_AUTO, LOW);
  } else if (fahrmodus == FAHRMODUS::AUTO) {
    digitalWrite(LED_MANUEL, LOW);
    digitalWrite(LED_ASSISTENT, LOW);
    digitalWrite(LED_AUTO, HIGH);     
  }
}

bool readErrorState(ERROR_MESSAGE errorMsg) {
  return (errorMsg & errorState) > 0;
}

void setErroState(ERROR_MESSAGE errorMsg, bool state) {
  bool oldState = readErrorState(errorMsg);
  if (state) {
    errorState = errorState | errorMsg;
  } else {
    errorState = errorState & ~errorMsg;
  }
  if (oldState != state) {    
    if (errorMsg == ERROR_MESSAGE::PC_OFFLINE_ERROR) {
      if (state) {
        playBuzzerSound(250, 1, 50);
        if (fahrmodus != FAHRMODUS::MANUEL) {
          sendNotaus();
          updateFahrmodus(FAHRMODUS::MANUEL);
          sendStatus();
        }
        digitalWrite(LED_PC_OFFLINE, HIGH);
      } else {
        digitalWrite(LED_PC_OFFLINE, LOW);
        sendStatus();
      }
    } else if (errorMsg == ERROR_MESSAGE::LAN_OFFLINE_ERROR) {
      if (state) {
        playBuzzerSound(250, 1, 50);
        digitalWrite(LED_LAN_OFFLINE, HIGH);
      } else {
        digitalWrite(LED_LAN_OFFLINE, LOW);
      }
    } else if (errorMsg == ERROR_MESSAGE::ENTGLEISUNG) {
      if (state) {
        playBuzzerSound(250, 1, 50);
        digitalWrite(LED_ENTGLEISUNG, HIGH);
      } else {
        digitalWrite(LED_ENTGLEISUNG, LOW);
      }
    } else if (errorMsg == ERROR_MESSAGE::BRUECKE_OFFEN) {
      if (state) {
        sendNotaus();
        playBuzzerSound(250, 1, 50);
        digitalWrite(LED_BRUECKE, HIGH);
      } else {
        digitalWrite(LED_BRUECKE, LOW);
      }
      sendStatus();
    } else if (errorMsg == ERROR_MESSAGE::KOLLISION_WARNUNG) {
      if (state) {
        playBuzzerSound(150, 2, 50);
        digitalWrite(LED_KOLLISION, HIGH);
      } else {
        digitalWrite(LED_KOLLISION, LOW);
      }      
    } else if (errorMsg == ERROR_MESSAGE::VITRINEN_ZUFAHRT_WARNUNG) {
      if (state) {
        playBuzzerSound(150, 2, 50);
        digitalWrite(LED_VITRINE_ZUFAHRT, HIGH);
      } else {
        digitalWrite(LED_VITRINE_ZUFAHRT, LOW);
      }  
    }
  }
}

void computePCStatusByte(byte statusByte) {
  FAHRMODUS newFahrmodus = (statusByte & 0b11000000);
  updateFahrmodus(newFahrmodus);

  setErroState(ERROR_MESSAGE::VITRINEN_ZUFAHRT_WARNUNG, (statusByte & 0b00100000) > 0);
  setErroState(ERROR_MESSAGE::KOLLISION_WARNUNG,        (statusByte & 0b00010000) > 0);
  setErroState(ERROR_MESSAGE::ENTGLEISUNG,              (statusByte & 0b00000100) > 0);
  setErroState(ERROR_MESSAGE::LAN_OFFLINE_ERROR,        (statusByte & 0b00000010) > 0);      
}

void computeZ21StatusByte(byte statusByte) {
  bool newGleisStromAus = (statusByte & 0b10000001) > 0;
  bool newKurzschluss = (statusByte & 0b00100000) > 0;
  if (newGleisStromAus != gleisStromAus || newKurzschluss != kurzschluss) {
    gleisStromAus = newGleisStromAus;
    kurzschluss = newKurzschluss;
    if (kurzschluss) {
      playBuzzerSound(400, 1, 50);
    }
    digitalWrite(LED_Z21_RED, kurzschluss ? HIGH : LOW);
  }
}

void computeButtonInput(BUTTON button, unsigned long t) {
  if (stableButton == button) {
    stableButtonCounter++;
  } else {
    stableButton = button;
    stableButtonCounter = 0;
  }
  if (stableButtonCounter > 4 && t - buttonPressedTimer > 200) {
    buttonPressedTimer = t;
    if (button == BUTTON::NOTAUS) {
      playBuzzerSound(50, 1, 50);
      sendNotausVitrine();
      sendNotaus();
    } else if (button == BUTTON::RESET) {
      playBuzzerSound(50, 1, 50);
      sendReset();
    } else if (button == BUTTON::B_MANUEL) {
      playBuzzerSound(50, 1, 50);
      updateFahrmodus(FAHRMODUS::MANUEL);
      sendStatus();
    } else if (button == BUTTON::B_ASSISTENT) {
      playBuzzerSound(50, 1, 50);
      updateFahrmodus(FAHRMODUS::ASSISTENT);
      sendStatus();
    } else if (button == BUTTON::B_AUTO) {
      playBuzzerSound(50, 1, 50);
      updateFahrmodus(FAHRMODUS::AUTO);
      sendStatus();
    }
  }
}

void setup() {
  //Serial.begin(115200);

  // init CAN
  SPI.begin();
  mcp2515.reset();                          
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setConfigMode();
  // MASK0 -> RXF0, RXF1; MASK1 -> RXF2, RXF3, RXF4, RXF5
  mcp2515.setFilterMask(MCP2515::MASK0, true, MASK_GROUP);
  mcp2515.setFilter(MCP2515::RXF0, true, FILTER_INFO);
  mcp2515.setFilter(MCP2515::RXF1, true, FILTER_SYSTEM);
  mcp2515.setNormalMode(); 

  // init PINS
  pinMode(LED_Z21_RED, OUTPUT); 
  pinMode(LED_Z21_BLUE, OUTPUT); 
  pinMode(LED_PC_OFFLINE, OUTPUT); 
  pinMode(LED_ENTGLEISUNG, OUTPUT); 
  pinMode(LED_BRUECKE, OUTPUT); 
  pinMode(LED_KOLLISION, OUTPUT); 
  pinMode(LED_VITRINE_ZUFAHRT, OUTPUT); 
  pinMode(LED_LAN_OFFLINE, OUTPUT);   
  pinMode(LED_MANUEL, OUTPUT); 
  pinMode(LED_ASSISTENT, OUTPUT); 
  pinMode(LED_AUTO, OUTPUT); 

  pinMode(BUZZER, OUTPUT);

  pinMode(A4, INPUT_PULLUP);
  //pinMode(A6, INPUT_PULLUP);

  digitalWrite(LED_Z21_RED, LOW); 
  digitalWrite(LED_Z21_BLUE, LOW); 
  digitalWrite(LED_PC_OFFLINE, LOW); 
  digitalWrite(LED_ENTGLEISUNG, LOW); 
  digitalWrite(LED_BRUECKE, LOW); 
  digitalWrite(LED_KOLLISION, LOW); 
  digitalWrite(LED_VITRINE_ZUFAHRT, LOW); 
  digitalWrite(LED_LAN_OFFLINE, LOW);   
  digitalWrite(LED_MANUEL, LOW); 
  digitalWrite(LED_ASSISTENT, LOW); 
  digitalWrite(LED_AUTO, LOW); 
}

void loop(){
  unsigned long t = millis();
  int sensorValue = analogRead(A4);
  if (sensorValue > 350 && sensorValue < 390) {
    // Manuell
    computeButtonInput(BUTTON::B_MANUEL, t);
  } else if (sensorValue > 440 &&  sensorValue < 490) {
    // Assistant
    computeButtonInput(BUTTON::B_ASSISTENT, t);
  } else if (sensorValue > 530 && sensorValue < 590) {
    // Auto
    computeButtonInput(BUTTON::B_AUTO, t);
  } else if (sensorValue > 680 && sensorValue < 750) {
    // Reset
    computeButtonInput(BUTTON::RESET, t);
  } else if (sensorValue > 770) {
    // Notaus
    computeButtonInput(BUTTON::NOTAUS, t);
  }

  // poll CAN message
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    zcan_message zcanMsg = toZCanMessage(canMsg);   
    // INFO message
    if (zcanMsg.group == ZCAN_GROUP::INFO && 
        zcanMsg.command == 0 &&
        zcanMsg.mode == EVENT) {
        if (zcanMsg.networkId == COMPUTER_NID) {
          // keep alive message from computer
          byte statusByte = zcanMsg.data[0];
          computePCStatusByte(statusByte);
          lastComputerAlive = t;
        } else if (zcanMsg.networkId == Z21_NID) {
          // keep alive message from Z21
          byte statusByte = zcanMsg.data[2];
          computeZ21StatusByte(statusByte);
        }
    } 
    // SYSTEM message from Z21
    else if (zcanMsg.group == ZCAN_GROUP::SYSTEM && 
        zcanMsg.command == 0 &&
        zcanMsg.mode == ACKNOWLEDGE && 
        zcanMsg.networkId == Z21_NID) {
        byte statusByte = zcanMsg.data[1];
        computeZ21StatusByte(statusByte);
    }      
  }

  // computer alive check
  if (t - lastComputerAlive > 2100) {
    setErroState(ERROR_MESSAGE::PC_OFFLINE_ERROR, true);
  } else {
    setErroState(ERROR_MESSAGE::PC_OFFLINE_ERROR, false);
  }

  // bruecke state check
  brueckeSchnitt = brueckeSchnitt + analogRead(A6);
  brueckenConnectionCounter++;
  if (brueckenConnectionCounter > 200) {
    if (brueckeSchnitt > 50000) {
      setErroState(ERROR_MESSAGE::BRUECKE_OFFEN, false);
    } else {
      setErroState(ERROR_MESSAGE::BRUECKE_OFFEN, true);
    }    
    brueckenConnectionCounter = 0;
    brueckeSchnitt = 0;
  }

  // buzzer 
  if (!analogOutON && buzzerCounter > 0) {
    if (buzzerAus > t) {
      // buzzer -> on first time
      analogWrite(BUZZER, buzzerPitch);
      analogOutON = true;
    } else if (buzzerAus + buzzerMS < t) {
      // buzzer -> on after pause
      buzzerAus = t + buzzerMS;
      analogWrite(BUZZER, buzzerPitch);
      analogOutON = true;
    }
  } else if (analogOutON && buzzerAus < t) {
    // buzzer -> off
    analogWrite(BUZZER,0);
    buzzerCounter--;
    analogOutON = false;
    if (buzzerCounter > 0) {
      buzzerAus = t;
    }
  }  

  // blue Z21 LED
  if (gleisStromAus) {
    if (t - lastBlueZ21Blink > 350) {
      blueZ21State = !blueZ21State;
      lastBlueZ21Blink = t;
      digitalWrite(LED_Z21_BLUE, blueZ21State ? HIGH : LOW);
    }
  } else if (blueZ21State && kurzschluss) {
    blueZ21State = false;
    digitalWrite(LED_Z21_BLUE, LOW);
  } else if (!blueZ21State) {
    blueZ21State = true;
    digitalWrite(LED_Z21_BLUE, HIGH);
  }


}
