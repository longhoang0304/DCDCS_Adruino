#ifndef MAIN_PROGRAM_H
#define MAIN_PROGRAM_H
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <Thread.h>
#include <Keypad.h>
#include <DCMotor.h>
#include <BH1750FVI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

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
#define DHT_TYPE DHT11

// RF PIN
#define BUTTON_A 8
#define BUTTON_B 9
#define BUTTON_C 10
#define BUTTON_D 11

// LCD5110
#define RST 15
#define SCE 14
#define DC  16
#define DIN 17
#define CLK 18

//dryer H-Bridge pin
#define DRYER_EN4 30
#define DRYER_EN3 31

// I2C Communication
#define SCL 21
#define SDA 20

#define MASTER_ADDRESS 0x01
#define SLAVE_ADDRESS 0X02

enum WifiRequestType { PERFORM_ACTION, UPDATE_IP };
enum SystemStatus { IDLING, DRYING, MOVING, PAUSED, DRYER_ACTIVATED };
enum Switch { NO_SWITCH, SWITCH_OUTSIDE, SWITCH_INSIDE };
enum Action {
  DRY_CLOTHES,
  COLLECT_CLOTHES,
  PAUSE_MOTOR,
  INCREASE_DRYER_TIME,
  DECREASE_DRYER_TIME,
  SET_DRYER_TIME, //this action can only perform through android app
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
  uint16_t lux;
  double humidity;
  double temperature;
} SensorData;

union address {
    uint8_t bytes[4];  // IPv4 address
    uint32_t dword;
} ;

typedef struct IPAddress {
  address address;
} IPAddress;

typedef unsigned long ul;
#define ONE_MINUTE 1000 * 60

void loop_event();
void setup_arduino();

#endif