#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp32-hal-ledc.h"
#include "MotorControl.hpp"

namespace Movement {
    // Servo positions
    /*Servo Physical Config on Car in deg
          45  90  135   
            \  |  /              
             \ | /
              \|/ 
       0 ------------- 180        */
    const uint8_t servoLeft   = 49.8;
    const uint8_t servoMiniLeft = 64.8;
    const uint8_t servoCenter = 94.8;
    const uint8_t servoRight  = 149.8;
    const uint8_t servoMiniRight = 119.8;

    // Global motor instances
    // extern DCMotor LeftWheel;
    // extern DCMotor RightWheel;
    // extern ServoMotor FrontWheel; 

    // Function declarations
    void RotateLeft();
    void RotateRight();
    void MoveForward();
    void MoveBackward();
    void FixLeft();
    void FixRight();
    void Stop();
}

#endif // ROBOT_MOVEMENT_H