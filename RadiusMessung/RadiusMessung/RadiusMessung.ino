#include "TM1637.h"
#include <Wire.h>
#include <AS5600.h>
#include <arduino.h>
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
  #define SYS_VOL   3.3
#else
  #define SERIAL Serial
  #define SYS_VOL   5
#endif

// consts
float minradius = 0.600;  // for buzzer
float maxradius = 1.500;
float len = 0.265;  // meter
float centerAng = 0.6519;  // radian  ~38 deg

// Pins definitions for TM1637 and can be changed to other ports
const int CLK = 2;  // YELLOQ
const int DIO = 3;  // WHITE
const int BUZZER = 4;
TM1637 tm1637(CLK, DIO);

AMS_5600 ams5600;
float ang = 0;
float radius = 0;

void setup() {
    tm1637.init();
    tm1637.set(BRIGHT_TYPICAL);   //BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;

    Serial.begin(9600); // set the baud rate
    pinMode(BUZZER, OUTPUT); // set buzzer as output

    Wire.begin();
    if(ams5600.detectMagnet() == 0 ){
    while(1){
        if(ams5600.detectMagnet() == 1 ) {
          break;
        }
        delay(1000);
    }
  }
}

/*******************************************************
/* Function: convertRawAngleToDegrees
/* In: angle data from AMS_5600::getRawAngle
/* Out: human readable degrees as float
/* Description: takes the raw angle and calculates
/* float value in degrees.
/*******************************************************/
float convertRawAngleToDegrees(word newAngle) {
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  // return (newAngle * 0.087891);
  /* Raw data reports 0 - 4095 segments, which is 0.001533981 of a radian */
  return (newAngle * 0.001533981);
}

void loop() {
    ang = convertRawAngleToDegrees(ams5600.getRawAngle()) - centerAng;
    if (sin(ang) != 0) {
      radius = len / (2*sin(ang));
      if (radius < 0) {
        radius = -radius;
      }
    } else {
      radius = 0;
    }
    if (radius > maxradius) {
      radius = 0;
    }

    tm1637.displayNum(radius, 3, false);

    if (radius > 0 && radius < minradius) {
      for(int i=0; i<8; i++) {
        digitalWrite(BUZZER, HIGH);
        delayMicroseconds(3500);
        digitalWrite(BUZZER, LOW);
        delayMicroseconds(3500);
      }
    } else {
      delay(50);
    }
}
