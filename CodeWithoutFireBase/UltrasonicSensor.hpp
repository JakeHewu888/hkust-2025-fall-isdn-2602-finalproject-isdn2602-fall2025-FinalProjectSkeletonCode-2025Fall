#ifndef UltraSonicSensor_H
#define UltraSonicSensor_H
#include "Pinout.hpp"
#include "Arduino.h"

namespace UltrasonicSensor{
  void Init();
  float GetDistance();
}

#endif