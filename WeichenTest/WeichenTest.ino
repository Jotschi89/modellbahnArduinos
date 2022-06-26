#include <Servo.h>

#define RELAY_PIN (4)  
#define SERVO_PIN (5)  

Servo servo;

void blinkShortly() {
  digitalWrite(RELAY_PIN, LOW);
  delay(1500);
  digitalWrite(RELAY_PIN, HIGH);
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
}

void loop() {
  servo.write(0); //Position 1 ansteuern mit dem Winkel 0°
  blinkShortly();
  delay(500); //Das Programm stoppt für 3 Sekunden
 
  delay(500);
  blinkShortly();
}
