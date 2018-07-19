#include "MainProgram.h"
/*
 * Setup Arduino
 */
#pragma region
// set the LCD address to 0x27 for a 16 chars and 2 line display
static LiquidCrystal_I2C lcd(0x3F, 20, 4);
// init light sensor object
static BH1750FVI LightSensor;
// init dht object
static DHT dht(DHT_PIN, DHT_TYPE);
//init keypad object to reading key
static Keypad keypad = Keypad(
  makeKeymap(keys),
  rowPins,
  colPins,
  ROWS,
  COLS
);
// init dc motor controller
static DCMotor dcMotor(ENA, ENB);
// init system status
static SystemStatus sysStatus;
static bool userControl = false;

void setupLightSensor() {
  //  LightSensor.SetAddress(Device_Address_H);//Address 0x5C
  //  To adjust the slave on other address , uncomment this line
  LightSensor.begin();
  LightSensor.SetAddress(Device_Address_L); //Address 0x5C
  LightSensor.SetMode(Continuous_L_resolution_Mode);
}

// setup lcd
void setupLCD() {
  lcd.init();
  lcd.backlight();
}

// set up pin mode for another pins
void setupPinMode() {
  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(L1, INPUT);
  pinMode(L2, INPUT);
  pinMode(BUTTON_A, INPUT);
  pinMode(BUTTON_B, INPUT);
  pinMode(BUTTON_C, INPUT);
  pinMode(BUTTON_D, INPUT);
}

// setup all arduino
void setup_arduino() {
  setupLCD();
  setupLightSensor();
  dht.begin();
  dcMotor.begin();
  setupPinMode();
  sysStatus = IDLING;
  Serial.begin(9600);
}
#pragma endregion

// this region for misc functions
#pragma region

/**
 * Read humidity and temperature from DHT
 */
void readDHT(double &humidity, double &temperature) {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
}

/**
 * Read switch signal and return
 */
Switch getSwitch() {
  bool l1 = digitalRead(L1);
  bool l2 = digitalRead(L2);
  if (!l2) return SWITCH_INSIDE;
  if (!l1) return SWITCH_OUTSIDE;
  return NO_SWITCH;
}

/**
 * Listen button and return the actions
 */
Action getActionFromKeyPad() {
  char key = keypad.getKey();
  if (key == 'A') {
    return PAUSE_MOTOR;
  }
  if (key == 'B') {
    return START_DRYER;
  }
  if (key == 'C') {
    return INCREASE_DRYER_TIME;
  }
  if (key == 'D') {
    return DECREASE_DRYER_TIME;
  }
  return NO_ACTION;
}

byte readRfButton() {
  if(digitalRead(BUTTON_A))
    return 1;
  if(digitalRead(BUTTON_B))
    return 2;
  if(digitalRead(BUTTON_C))
    return 3;
  if(digitalRead(BUTTON_D))
    return 4;
  return 0;
}

Action getActionFromRF() {
  byte key = readRfButton();
  if (key == 1) {
    return PAUSE_MOTOR;
  }
  if (key == 2) {
    return START_DRYER;
  }
  if (key == 3) {
    return INCREASE_DRYER_TIME;
  }
  if (key == 4) {
    return DECREASE_DRYER_TIME;
  }
  return NO_ACTION;
}

bool isNight(uint16_t lux) {
  return lux < 5;
}

bool isRaining() {
  return digitalRead(RAIN_SENSOR_PIN);
}

void runDC(DC_DIRECTION direct) {
  dcMotor.move(direct);
  sysStatus = MOVING;
}

void stopDC(SystemStatus stat) {
  dcMotor.move(STOP);
  sysStatus = stat;
}

#pragma endregion

// this region will be the main thread of arduino
#pragma region

void readData(SensorData &sensorData) {
  sensorData.humidity = dht.readHumidity();
  sensorData.temperature = dht.readTemperature();
  sensorData.lux = LightSensor.GetLightIntensity();
}

void controlDCAtNight() {
  switch (sysStatus) {
    case DRYING:
      runDC(FORWARD);
      break;
  }
}

void controlDCAtDay() {
  bool isRain = isRaining();
  switch (sysStatus) {
    case DRYING:
      if (isRain) {
        runDC(BACKWARD);
      }
      break;
    case IDLING:
      if (!isRain) {
        runDC(FORWARD);
      }
      break;
  }
}

void autoControl(SensorData sensorData) {
  if (isNight(SensorData.lux)) {
    controlDCAtNight();
    return;
  }
  controlDCAtDay();
}

void switchControl() {
  Switch swt = getSwitch();
  switch(swt) {
    case SWITCH_INSIDE: {
      if (dcMotor.getDirection() == BACKWARD) {
        stopDC(IDLING);
      }
      break;
    }
    case SWITCH_OUTSIDE: {
      if (dcMotor.getDirection() == FORWARD) {
        stopDC(DRYING);
      }
      break;
    }
    default: {
      break;
    }
  }
  return;
}

void buttonControl() {

}

void rfButtonControl() {

}

void wifiControl() {

}

void loop_event() {
  SensorData sensorData;
  readData(sensorData);
  autoControl(SensorData);
  buttonControl();
  rfButtonControl();
  wifiControl();
  switchControl();
}

#pragma endregion