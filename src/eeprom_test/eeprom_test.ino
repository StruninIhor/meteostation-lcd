#include <EEPROM.h>

int led_delay;
uint32_t timer = led_delay;
int value_address = 0;
bool state = false;

template <class T> int writeToMemory(int ee, const T& value) {
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++) {
    EEPROM.update(ee++, *p++);
  }
  return i;
}

template <class T> int readFromMemory(int ee, T& value) {
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++) {
    *p++ = EEPROM.read(ee++);
  }
  return i;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  readDelay();
  Serial.println("INT: " + String(sizeof(int)));
  Serial.println("UINT: " + String(sizeof(unsigned int)));
  Serial.println("LONG: " + String(sizeof(long)));
  Serial.println("FLOAT: " + String(sizeof(float)));
  Serial.println("DOUBLE: " + String(sizeof(double)));
}

void readDelay() {
  int value;
  readFromMemory(value_address, value);
  if (value > 0) {
    Serial.println("SETUP OK; delay=" + String(value));
    led_delay = value;
  }
}

void readInput() {
  String strValue = Serial.readString();
  int value = strValue.toInt();
  if (value > 0) {
    led_delay = value;
    writeToMemory(value_address, value);
    Serial.println("OK: delay=" + String(value));
  } else {
    Serial.println("ERROR: delay=" + String(led_delay));
  }
}

void loop() {
  if (Serial.available() > 0) {
    readInput();
  }
  if (millis() - timer > led_delay) {
    timer = millis();
    state = !state;
    digitalWrite(LED_BUILTIN, state);
  }
}
