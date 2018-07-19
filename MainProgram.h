#ifndef MAIN_PROGRAM_H
#define MAIN_PROGRAM_H
#include <Wire.h>
#include <BH1750FVI.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Thread.h>
#include <Keypad.h>
#include <DCMotor.h>

// RAIN SENSOR PIN
#define RAIN_SENSOR_PIN 2

// DC CONTROL PIN
#define ENA 3
#define ENB 4

// LIMIT SWITCH PIN
#define L1 5
#define L2 6

// DHT PIN
#define DHT_PIN 7

// RF PIN
#define BUTTON_A 8
#define BUTTON_B 9
#define BUTTON_C 10
#define BUTTON_D 11

#define DHT_TYPE DHT11

enum SystemStatus { IDLING, DRYING, MOVING, DRYER_ACTIVATED };
enum Switch { NO_SWITCH, SWITCH_OUTSIDE, SWITCH_INSIDE };
enum Action {
  DRY_CLOTHES,
  COLLECT_CLOTHES,
  PAUSE_MOTOR,
  INCREASE_DRYER_TIME,
  DECREASE_DRYER_TIME,
  START_DRYER,
  STOP_DRYER,
  RESET_DRYER_TIMER,
  NO_ACTION,
};

#define ROWS 4 //four rows
#define COLS 4 //four columns
static char keys[ROWS][COLS] = {
{'1', '2', '3', 'A'},
{'4', '5', '6', 'B'},
{'7', '8', '9', 'C'},
{'*', '0', '#', 'D'}};

static byte rowPins[ROWS] = {22, 24, 26, 28}; //connect to the row pinouts of the keypad
static byte colPins[COLS] = {23, 25, 27, 29}; //connect to the column pinouts of the keypad

typedef struct data {
  double humidity;
  double temperature;
  uint16_t lux;
} SensorData;

void loop_event();
void setup_arduino();

#endif