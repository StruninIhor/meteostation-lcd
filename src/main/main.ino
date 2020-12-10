#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BME280.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // Устанавливаем дисплей
Adafruit_BME280 bme; // I2C

static unsigned int hours = 8;
static unsigned int minutes = 7;
static unsigned int seconds = 00;
static unsigned long lastMillis;
bool semicolonFlag = true;


/*
  ..OOO
  ..O.O
  ..OOO
  .....
  .....
  .....
  .....
*/
uint8_t gradus[8]  = {
  0b00111,
  0b00101,
  0b00111,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

/*
  .O...
  O.O.O
  .O.O.
  ..O..
  .O.O.
  O.O.O
  ...O.
*/
uint8_t percent[8]  = {
  0b01000,
  0b10101,
  0b01010,
  0b00100,
  0b01010,
  0b10101,
  0b00010
};

void setup()
{
  lcd.init();                     
  lcd.backlight();
  //lcd.print("Coming soon...");
  //lcd.setCursor(8, 1);
  lcd.createChar(0, gradus);
  lcd.createChar(1, percent);
  Serial.begin(9600);
  while(!Serial) delay(1);
  while (setupSensor()) {
    delay(10);
  }
}
void printNumber(unsigned int number) {
  if (number < 10) {
    lcd.print("0");
  }
  lcd.print(number);
}

bool setupSensor() {
  unsigned status;
  // default settings
  status = bme.begin();
  if (!status) {
     Serial.print("ERROR. Sensor id: 0x");
     Serial.println(bme.sensorID(),16);
  }
  return !status;
}

void printTime() { 
  lcd.setCursor(11, 1);
  lastMillis = millis();   
  seconds += 1;
  semicolonFlag = !semicolonFlag;
  if (seconds == 60) {
    seconds = 0;
    minutes += 1;
  }
  if (minutes == 60) {
    minutes = 0;
    hours +=1;
  }
  if (hours == 24) {
    hours = 0;
  }
  printNumber(hours);
  lcd.print(semicolonFlag ? ":" : " ");
  printNumber(minutes);
}
String translateFloat(float value) {
  return isnan(value) ? "null" : String(value); 
}
void printSensorData() {
    float temp = bme.readTemperature();
    float pressure = bme.readPressure() / 100.0F;
    float humidity = bme.readHumidity();

    lcd.setCursor(0,0);
    lcd.print(temp);
    lcd.print('\0');
    lcd.print("C");
    lcd.setCursor(10,0);
    lcd.print(humidity);
    lcd.print('\1');
    lcd.setCursor(0,1);
    lcd.print(pressure / 1.33322);
    lcd.print("mmHg");
    
//    Serial.print("{\"T\": ");
//    Serial.print(translateFloat(temp));
//    Serial.print(",\"P\": ");
//    Serial.print(translateFloat(pressure));
//    Serial.print(",\"H\": ");
//    Serial.print(translateFloat(humidity));
//    Serial.print('}');
//    Serial.println();
}


void loop()
{
  // Выводим на экран количество секунд с момента запуска ардуины
  if (millis() - lastMillis > 1000) {
    printSensorData();
    //printTime();
  }
  
}
