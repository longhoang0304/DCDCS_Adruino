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
static byte dryerTimer = 30; // default 30 minutes
static byte motorBtnPulse = 0;
static byte dryerBtnPulse = 0;

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

Action getAction(key) {
  if (key == 1) {
    return determineMotorAction();
  }
  if (key == 2) {
    return determineDryerAction();;
  }
  if (key == 3) {
    return INCREASE_DRYER_TIME;
  }
  if (key == 4) {
    return DECREASE_DRYER_TIME;
  }
  return NO_ACTION;
}

/**
 * Listen button and return the actions
 */
Action getActionFromKeyPad() {
  char key = keypad.getKey();
  return getAction(key - 'A');
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
  return getAction(key);
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

void increaseTimer() {
  dryerTimer += 5;
  if (dryerTimer > 120)
    dryerTimer = 120;
}

void decreaseTimer() {
  dryerTimer -= 5;
  if (dryerTimer < 5)
    dryerTimer = 5;
}

Action determineMotorAction() {
  motorBtnPulse = (motorBtnPulse + 1) % 4;
  switch(motorBtnPulse) {
    case 1: {
      return DRY_CLOTHES;
    }
    case 3: {
      return COLLECT_CLOTHES;
    }
    case 0:
    case 2: {
      return PAUSE_MOTOR;
    }
  }
  return NO_ACTION;
}

Action determineDryerAction() {
  dryerBtnPulse = (dryerBtnPulse + 1) % 2;
  switch(dryerBtnPulse) {
    case 0: {
      return START_DRYER;
    }
    case 1: {
      return STOP_DRYER;
    }
  }
  return NO_ACTION;
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

void actionControl(Action action) {
  switch(action) {
    case PAUSE_MOTOR: {
      stopDC(PAUSED);
      break;
    }
    case DRY_CLOTHES: {
      runDC(FORWARD);
      break;
    }
    case COLLECT_CLOTHES: {
      runDC(BACKWARD);
      break;
    }
    case INCREASE_DRYER_TIME: {
      increaseTimer();
      break;
    }
    case DECREASE_DRYER_TIME: {
      decreaseTimer();
      break;
    }
    case RESET_DRYER_TIMER: {
      dryerTimer = 30;
      break;
    }
    case START_DRYER: {
      // control dryer
      break;
    }
    case STOP_DRYER: {
      //stop dryer
      break;
    }
    default: {
      break;
    }
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
  if (dcMotor.getDirection() == SCALAR)
    return;
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
  Action action = getActionFromKeyPad();
  actionControl(action);
}

void rfButtonControl() {
  Action action = getActionFromRF();
  actionControl(action);
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