#include "Arduino.h"
#include "DCMotor.h"

DCMotor::DCMotor(byte ENA, byte ENB) {
  this->ENA = ENA;
  this->ENB = ENB;
}

void DCMotor::begin() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  this->stop();
}

void DCMotor::stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

void DCMotor::moveForward() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, HIGH);
}

void DCMotor::moveForward(byte speed) {
  analogWrite(ENA, 0);
  analogWrite(ENB, speed);
}

void DCMotor::moveBackward() {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, LOW);
}

void DCMotor::moveBackward(byte speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, 0);
}

void DCMotor::move(DC_DIRECTION direction) {
  switch(direction) {
    case FORWARD:
      moveForward();
      break;
    case BACKWARD:
      moveBackward();
      break;
    case SCALAR:
      stop();
      break;
  }
}

