#ifndef DCMOTOR_H
#define DCMOTOR_H

#include "Arduino.h"

enum DC_DIRECTION {FORWARD, BACKWARD, SCALAR};

#define STOP SCALAR

class DCMotor {
  private:
    byte ENA;
    byte ENB;
    DC_DIRECTION direction;
    void stop();
    void moveForward();
    void moveForward(byte speed);
    void moveBackward();
    void moveBackward(byte speed);
  public:
    DCMotor(byte ENA, byte ENB);
    void begin();
    void move(DC_DIRECTION direction);
    void move(DC_DIRECTION direction, byte speed);
    DC_DIRECTION getDirection();
};

#endif
