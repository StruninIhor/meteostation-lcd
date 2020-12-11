/********************************************************************/
// First we include the libraries
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#include <Adafruit_BME280.h>
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

/********************************************************************/
#define ONE_WIRE_BUS 12
#define UPDATE_TIME 2000
#define CO2_UPDATE_TIME 15000
#define HEAT_RELAY_PIN 8
#define HUMIDIFIER_RELAY_PIN 2
#define VENTILATION_PIN 3

boolean airAvailable, compostAvailable, humidityAvailable, co2Available;

/********************************************************************/
/// Temperature sensors DS18b2
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27,16,2);

/// Humidity (and also temp and pressure) sensor BME280
Adafruit_BME280 bme;

/********************************************************************/ 

int signum(float val) {
  return ((val > 0) ? 1 : ((val < 0) ? -1 : 0));
}

/********************************************************************/ 
///Temperature controller structure
typedef class {
  public: 
    int direction = 1;
    float input, setPoint;
    float deviation = 0;
    float k = 0;
    boolean output;
    void setUp(uint8_t relayPin) {
      _relayPin = relayPin;
      pinMode(_relayPin, OUTPUT);
    }

    void control(uint32_t deltaTime) {
      output = computeResult(deltaTime);
      digitalWrite(_relayPin, output);
    }
    
    boolean computeResult(uint32_t deltaTime) {
      float signal ;
      if (deltaTime > 0) {
        float derivative = (input - _previousInput) / deltaTime;
        _previousInput = input;      
        signal = input + k * derivative;
      } else {
        signal = input;
      }
      int8_t F = (signum(signal - setPoint - deviation / 2) + signum(signal - setPoint + deviation / 2)) / 2;
      if (F == 1) output = !direction;
      else if (F == -1) output = direction;
      return output;
    }
    
  private:
    float _previousInput = 0;
    int _relayPin;
} RelayController;
/********************************************************************/ 

/********************************************************************/ 
/// Data struct

typedef struct {
  float airTemperature;
  float compostTemperature;
  float humidity;
  float co2;
} SensorsData;
/********************************************************************/ 

typedef class {
  public: 
  void setUp(SoftwareSerial* serial) {
    _sensorSerial = serial;
    _sensorSerial->begin(9600);
  }
  unsigned int readData() {
      byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
      unsigned char response[9];
      _sensorSerial->write(cmd, 9);
      memset(response, 0, 9);
      _sensorSerial->readBytes(response, 9);
      int i;
      byte crc = 0;
      for (i = 1; i < 8; i++) crc+=response[i];
      crc = 255 - crc;
      crc++;
      
      if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
        Serial.println("CRC error: " + String(crc) + " / "+ String(response[8]));
        return 1;
      } else {
        unsigned int responseHigh = (unsigned int) response[2];
        unsigned int responseLow = (unsigned int) response[3];
        unsigned int ppm = (256*responseHigh) + responseLow;
        return ppm;
      }
  }
  void enableCalibration() {
    _sensorSerial->write(_enableAbcCommand, 9);
  }
  void disableCalibration() {
    _sensorSerial->write(_disableAbcCommand, 9);
  }
  void calibrate() {
    _sensorSerial->write(_calibrateCommand, 9);
  }
  private:
  SoftwareSerial* _sensorSerial;
  uint32_t _speed;
  unsigned char _response[9];
  byte _readCommand[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
  byte _calibrateCommand[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78 };
  byte _enableAbcCommand[9] = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6 };
  byte _disableAbcCommand[9] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86 };
  
} Co2Sensor;


/********************************************************************/ 
RelayController airTemperatureController;
RelayController humidifierController;
RelayController co2Controller;
SoftwareSerial serial(A0, A1);
Co2Sensor co2Sensor;

void setup(void) 
{ 
 // start serial port 
 setUpLCD();
 printLCDInfo("Wait...");
 while(!Serial) delay(1);
 Serial.begin(9600); 
 printLCDInfo("Wait...25%");
 // Start up temp sensors
 printLCDInfo("Wait...50%");
 //pinMode(HEAT_RELAY_PIN, OUTPUT);
 // Start up humidity sensor
 while (setupHumiditySensor()) {
      delay(50);
    }
  printLCDInfo("Wait...75%");
  setUpAirController();
  setUpHumidityController();
  setupC02Controller();
  printLCDInfo("Ready!");
  delay(500);
} 
/********************************************************************/ 
/********************************************************************/ 
/// Temperature setup
bool setupHumiditySensor() {
  unsigned status;
  // default settings
  status = bme.begin();
  if (!status) {
     Serial.print("ERROR. Sensor id: 0x");
     Serial.println(bme.sensorID(),16);
  }
  return !status;
}
/********************************************************************/ 

void setUpHumidityController() {
  humidifierController.setUp(HUMIDIFIER_RELAY_PIN);
  humidifierController.deviation = 2;
  humidityAvailable = true;
}

void setUpAirController() {
  sensors.begin(); 
  airTemperatureController.setUp(HEAT_RELAY_PIN);
  airTemperatureController.deviation = 1;
  airAvailable = compostAvailable = true;
}

void setupC02Controller() {
  co2Sensor.setUp(&serial);
  co2Available = true;
  co2Controller.setUp(VENTILATION_PIN);
  // Reverse the direction of co2 sensor
  co2Controller.direction = 0;
  co2Controller.deviation = 250;
}

void setUpLCD() {
  lcd.init();                     
  lcd.backlight();
}
void printLCDInfo(String info) {
  lcd.clear();
  lcd.print("SmartGH v0.1");
  lcd.setCursor(0,1);
  lcd.print(info);
}
/********************************************************************/ 

void printDataOnLCD(float airTemperature, float compostTemperature, float humidity, int co2) {
  lcd.clear();
  lcd.print("A:");
  if (airAvailable) {
    lcd.print(airTemperature);
  } else {
    lcd.print("--");
  }
  lcd.setCursor(8,0);
  lcd.print("C:");
  if (compostAvailable) {
    lcd.print(compostTemperature);
  } else {
    lcd.print("--");
  }
  lcd.setCursor(0,1);
  lcd.print("H:");
  if (humidityAvailable) {
    lcd.print(humidity);
  } else {
    lcd.print("--");
  }
  lcd.setCursor(8,1);
  lcd.print("co2:");
  if (co2Available) {
    lcd.print(co2);
  } else {
    lcd.print("--");
  }
}

/********************************************************************/ 

uint32_t timer = 0;
uint32_t co2Timer = 15000;



int co2;
void loop(void) 
{ 
  if (Serial.available() > 0) {
    processInput();
  }
  uint32_t millisValue = millis();
  uint32_t deltaTime = millisValue - timer;
  uint32_t co2DeltaTime = millisValue - co2Timer;
  if (deltaTime > UPDATE_TIME) {
    timer = millis();
    sensors.requestTemperatures(); // Send the command to get temperature readings    
    SensorsData data = {};
    float airTemperature = sensors.getTempCByIndex(0);
    float compostTemperature = sensors.getTempCByIndex(1);
    float humidity = bme.readHumidity();
    printDataOnLCD(airTemperature, compostTemperature, humidity, co2);
    airTemperatureController.input = airTemperature;
    airTemperatureController.control(deltaTime);
    humidifierController.input = humidity;
    humidifierController.control(deltaTime);   
    Serial.print("air:");
    Serial.print(airTemperature);
    Serial.print(' ');
    Serial.print("compost:");
    Serial.print(compostTemperature);
    Serial.print(' ');
    Serial.print("humidity:");
    Serial.print(humidity);
    Serial.print(' ');
    Serial.print("co2:");
    Serial.println(co2);
  } 
  if (co2DeltaTime > CO2_UPDATE_TIME) {
    co2Timer = millis();
    co2 = co2Sensor.readData();
    co2Controller.input = co2;
    co2Controller.control(co2DeltaTime);
  }
} 

void processInput() {
  String command = Serial.readString();
  int equalIndex = command.indexOf('=');
  int separatorIndex = command.indexOf(';');
  String coef = command.substring(separatorIndex+1, equalIndex);
  String controllerStr = command.substring(0, separatorIndex);
  RelayController* controller = &humidifierController;
  
  boolean needToProcessCo2Input = false;
  if (controllerStr == "H") {
    controller = &humidifierController;
  } else if (controllerStr == "A") {
    controller = &airTemperatureController;
  } else if (controllerStr == "C") {
    controller = &co2Controller;
  } else if (controllerStr == "CC") { // Calibration
    needToProcessCo2Input = true;
  }
  String stringValue = command.substring(equalIndex+1, command.length());
  float value = stringValue.toFloat();
  if (needToProcessCo2Input) {
    processCo2Input(coef);
    return;
  }
  if (coef == "SP") {
    controller->setPoint = value;
  } else if (coef == "DV") {
    controller->deviation = value;
  } else if (coef == "KD") {
    controller->k = value;
  }
}

void processCo2Input(String command) {
  Serial.println("Processing CO2 command. Command=" + command);
  if (command == "E") {
    //enable autocalibration
    co2Sensor.enableCalibration();
  } else if (command == "D")  {
    co2Sensor.disableCalibration();
  } else if (command == "C") {
    //calibrate
    co2Sensor.calibrate();
  }
} 
