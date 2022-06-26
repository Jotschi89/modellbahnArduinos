
// pins
int PULS_PIN = 9;
int DIR_PIN = 8;
int GREEN_BUTTON_PIN = 3;
int BLACK_BUTTON_PIN = 4;

boolean dir = HIGH;
float delayMS = 16;



void setup() {
  pinMode(PULS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(GREEN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BLACK_BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  int greenValue = digitalRead(GREEN_BUTTON_PIN);
  int blackValue = digitalRead(BLACK_BUTTON_PIN);
  if (greenValue == LOW) {
    digitalWrite(DIR_PIN, HIGH);
  }
  if (blackValue == LOW) {
    digitalWrite(DIR_PIN, LOW);
  }
  if (greenValue == LOW || blackValue == LOW) {
    digitalWrite(PULS_PIN, HIGH);
  }
  delayMicroseconds(delayMS);
  digitalWrite(PULS_PIN, LOW);
  delayMicroseconds(delayMS);

}
