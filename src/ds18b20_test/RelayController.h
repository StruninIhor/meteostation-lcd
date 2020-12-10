#pragma once
#include "Arduino.h"

#ifndef RelayController_h
#define RelayController_h
class RelayController {
  public: 
    int relayPin;
    int direction = 1;
    float input, setPoint;
    float deviation = 0;
    float k = 0;
    boolean output;
    void setUp(uint8_t relayPin);
    void control(uint32_t deltaTime);
    boolean computeResult(uint32_t deltaTime);
  private:
    float _previousInput = 0;
}
#endif