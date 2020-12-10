#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#define UPDATE_TIME 10000;


SoftwareSerial mySerial(A0, A1); // A0 - к TX сенсора, A1 - к RX
LiquidCrystal_I2C lcd(0x27,16,2);

byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
unsigned char response[9];

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  lcd.init();                     
  lcd.backlight();
}

void readCo2() 
{
    mySerial.write(cmd, 9);
    memset(response, 0, 9);
    mySerial.readBytes(response, 9);
    int i;
    byte crc = 0;
    for (i = 1; i < 8; i++) crc+=response[i];
    crc = 255 - crc;
    crc++;
    lcd.setCursor(0,0);
    if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
      Serial.println("CRC error: " + String(crc) + " / "+ String(response[8]));
      lcd.print("Error: " + String(crc) + " / "+ String(response[8]));
    } else {
      unsigned int responseHigh = (unsigned int) response[2];
      unsigned int responseLow = (unsigned int) response[3];
      unsigned int ppm = (256*responseHigh) + responseLow;
      Serial.println(ppm);
      lcd.print(ppm);
      lcd.print(" ppm");
    }
}
long lastMillis = 0;

void loop() 
{
  if (millis() - lastMillis > 10000) {
   lastMillis = millis();
   readCo2();
  }
}
