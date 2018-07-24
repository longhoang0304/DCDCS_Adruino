#include "MainProgram.h"
/**
 * Setup Arduino
 */

void sendESP8266Data();
void handleESP8266Request(int numBytes);
void actionControl(Action action, byte timer = 0);

#pragma region
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
// check if user is taking control
static bool userControl = false;
// timer for dryer
static byte dryerTimer = 30; // default 30 minutes
// pulse cuont for button press
static byte motorBtnPulse = 0;
// pulse count for dryer button press
static byte dryerBtnPulse = 0;
// data from sensors
static SensorData sensorData;
// ip address
static IPAddress ip = {0};
// lcd5110
Adafruit_PCD8544 display = Adafruit_PCD8544(CLK, DIN, DC, SCE, RST);
//
static ul accTime = 0;

void setupLightSensor() {
  //  LightSensor.SetAddress(Device_Address_H);//Address 0x5C
  //  To adjust the slave on other address , uncomment this line
  LightSensor.begin();
  LightSensor.SetAddress(Device_Address_L); //Address 0x5C
  LightSensor.SetMode(Continuous_L_resolution_Mode);
}

// setup lcd
void setupLCD() {
  display.begin();
  display.setContrast(50);
  display.display(); // show splashscreen
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0, 0);
}

// set up pin mode for another pins
void setupPinMode() {
  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(L1, INPUT);
  pinMode(L2, INPUT);
  pinMode(DRYER_PIN, OUTPUT);
  pinMode(BUTTON_A, INPUT);
  pinMode(BUTTON_B, INPUT);
  pinMode(BUTTON_C, INPUT);
  pinMode(BUTTON_D, INPUT);
}

void setupI2C() {
  Wire.begin(MASTER_ADDRESS);
  Wire.onRequest(sendESP8266Data);
  Wire.onReceive(handleESP8266Request);
}

// setup all arduino
void setup_arduino() {
  setupLCD();
  setupLightSensor();
  setupI2C();
  dht.begin();
  dcMotor.begin();
  setupPinMode();
  sysStatus = DRYING;
  Serial.begin(9600);
}
#pragma endregion

// this region for misc functions
#pragma region

template<typename T> byte *convertValueToByteArray(T value) {
  if (value <= 1) value = 0;
  uint64_t v = *((uint64_t *)((void *)&value));
  const size_t len = sizeof(T);
  byte index = 0;
  byte * arr;
  arr = (byte *)calloc(len, sizeof(byte));
  while(v) {
    arr[index] = v & 0xff;
    v >>= 0x08;
    index++;
  }
  return arr;
}

template<typename T> void copyValueToByteArray(T value, byte *data, byte &i) {
  byte * arr = convertValueToByteArray<T>(value);
  byte len = sizeof(T);
  byte j = 0;
  while(len) {
    data[i++] = arr[j++];
    len--;
  }
  free(arr);
}

void sendESP8266Data() {
  const size_t length = sizeof(SensorData);
  byte i = 0;
  byte data[length] = {0};

  // copy humidity to array
  copyValueToByteArray<double>(sensorData.humidity, data, i);
  // copy temperatur to array
  copyValueToByteArray<double>(sensorData.temperature, data, i);
  // copy lux to array
  copyValueToByteArray<uint16_t>(sensorData.lux, data, i);
  // send data to esp8266
  Wire.write(data, length);
}

void handleESP8266Request(int numBytes) {
  byte data[numBytes] = {0};
  byte i = 0;
  while(1 < Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  switch(data[0]) {
    case PERFORM_ACTION: {
      actionControl(data[1], data[2]);
      break;
    }
    case UPDATE_IP: {
      uint32_t dword = 0;
      for(int i = 1; i < 5; i++) {
        dword += data[i];
        dword <<= 0x08;
      }
      ip.address.dword = dword;
      break;
    }
    default:
      break;
  }
}

/**
 * Read switch signal and return
 */
Switch getSwitch() {
  bool l1 = digitalRead(L1);
  bool l2 = digitalRead(L2);
  Serial.println(l1);
  Serial.println(l2);
  if (l1 && l2) return NO_SWITCH;
  if (!l2) return SWITCH_INSIDE;
  if (!l1) return SWITCH_OUTSIDE;
  return NO_SWITCH;
}

/**
 * Determine DC Motor action by button pulse
 */
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

/**
 * Determine dryer action by dryer button pulse
 */
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


/**
 * Get action from key
 */
Action getAction(byte key) {
  if (key == 1) {
    return determineMotorAction();
  }
  if (key == 2) {
    return determineDryerAction();
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
  if(key == NO_KEY) {
    Serial.println("NO KEY");
    return getAction(-1);
  }
  Serial.println("KEYPAD");
  Serial.println(key);
  return getAction(key - 'A');
}

/**
 * Read data from rf
 */
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

/**
 * Listen rf button and return the actions
 */
Action getActionFromRF() {
  byte key = readRfButton();
  return getAction(key);
}

/**
 * Check if is night or not
 */
bool isNight(uint16_t lux) {
  return lux < 5;
}

/**
 * Check if is raining
 */
bool isRaining() {
  return !digitalRead(RAIN_SENSOR_PIN);
}

/**
 * Control dc
 */
void runDC(DC_DIRECTION direct) {
  dcMotor.move(direct);
  sysStatus = MOVING;
}

/**
 * Stop dc
 */
void stopDC(SystemStatus stat) {
  dcMotor.move(STOP);
  sysStatus = stat;
}

/**
 * Increase dryer timer
 */
void increaseTimer() {
  dryerTimer += 5;
  if (dryerTimer > 120)
    dryerTimer = 120;
}

/**
 * Decrease dryer timer
 */
void decreaseTimer() {
  dryerTimer -= 5;
  if (dryerTimer < 5)
    dryerTimer = 5;
}

#pragma endregion

// this region will be the main thread of arduino
#pragma region

/**
 * Read data from sensor
 */
void readData() {
  sensorData.humidity = dht.readHumidity();
  sensorData.temperature = dht.readTemperature();
  sensorData.lux = LightSensor.GetLightIntensity();
}

/**
 * Control system when night
 */
void controlDCAtNight() {
  switch (sysStatus) {
    case DRYING:
      runDC(BACKWARD);
      break;
  }
}

/**
 * Control system at day
 */
void controlDCAtDay() {
  bool isRain = isRaining();
  Serial.println(isRain);
  switch (sysStatus) {
    case PAUSED:
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

/**
 * Control system based on action
 */
void actionControl(Action action, byte timer = 0) {
  bool isRain = isRaining();
  switch(action) {
    case PAUSE_MOTOR: {
      if (sysStatus != MOVING)
        break;
      stopDC(PAUSED);
      break;
    }
    case DRY_CLOTHES: {
      if(sysStatus == DRYER_ACTIVATED)
        actionControl(STOP_DRYER);
      if (isRain) return;
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
    case SET_DRYER_TIME: {
      dryerTimer = timer;
    }
    case RESET_DRYER_TIMER: {
      dryerTimer = 30;
      break;
    }
    case START_DRYER: {
      if(sysStatus != IDLING)
        break;
      sysStatus = DRYER_ACTIVATED;
      digitalWrite(DRYER_PIN, HIGH);
      break;
    }
    case STOP_DRYER: {
      sysStatus = IDLING;
      digitalWrite(DRYER_PIN, LOW);
      dryerTimer = 30;
      break;
    }
    default: {
      break;
    }
  }
}

/**
 * Autocontroller
 */
void autoControl() {
  if (isNight(sensorData.lux)) {
    controlDCAtNight();
    return;
  }
  controlDCAtDay();
}

/**
 * Control system by switch signal
 */
void switchControl() {
  Switch swt = getSwitch();
  if (dcMotor.getDirection() == SCALAR)
    return;
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

/**
 * Control system by button
 */
void buttonControl() {
  Action action = getActionFromKeyPad();
  actionControl(action);
}

/**
 * Control system by rf button
 */
void rfButtonControl() {
  Action action = getActionFromRF();
  actionControl(action);
}

void printData() {

}

void handleDryerTimer(ul start, ul end) {
  if (sysStatus != DRYER_ACTIVATED) {
    return;
  }
  accTime += (end - start);
  if (accTime < 0) accTime = 0; // incease that timer is reset
  if (accTime >= ONE_MINUTE) {
    dryerTimer -= 1;
    accTime = 0;
    if (dryerTimer < 1) {
      actionControl(STOP_DRYER);
    }
  }
}

/**
 * Main loop, single thread
 */
void loop_event() {
  ul start = millis();
  readData();
  autoControl();
  Serial.println("AUTO");
  Serial.println(sysStatus);
  buttonControl();
  Serial.println("BUTTON");
  Serial.println(sysStatus);
  rfButtonControl();
  Serial.println("RF");
  Serial.println(sysStatus);
  switchControl();
  Serial.println("SWITCH");
  Serial.println(sysStatus);
  printData();
  ul end = millis();
  handleDryerTimer(start, end);
  Serial.println("FINAL");
  Serial.println(sysStatus);
  Serial.println("\n\n");
  delay(1000);
}

#pragma endregion