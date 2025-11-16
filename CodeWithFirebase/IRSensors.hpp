#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <stdint.h>

namespace IRSensors {
    // IR Sensor Configuration
    /*
           L   M   R  
            \  |  /              
             \ | /       ^
              \|/        |
              Car      Front
    
    */


    // Robot States
    enum RobotState : uint8_t {
        /* The logic of the IR sensor and its output
        |    Tile Colour   |  digitalRead() |
        |      Black       |    1 (HIGH)    |
        |      White       |     0 (LOW)    |
         */
        /*  Logic Reference Table
        |    uint8_t     |     boolean ( digitalRead() )  |
        |   RobotState    |   IR_L   |   IR_M   |   IR_R   |
        |        0        |     0    |     1    |     0    |
        |        1        |     1    |     1    |     0    |
        |        2        |     1    |     1    |     1    |
        |        3        |     1    |     0    |     1    |
        |        4        |     0    |     1    |     1    |
        |        5        |     0    |     0    |     1    |
        |        6        |     1    |     0    |     0    |
        |        7        |     0    |     0    |     0    |
        
        |*/
        Middle_ON_Track,        // 0 
        Left_Middle_ON_Track,   // 1
        ALL_ON_Track,           // 2
        Left_Right_ON_Track,    // 3
        Middle_Right_ON_Track,   // 4
        Right_ON_Track,         // 5
        Left_ON_Track,          // 6
        All_OFF_Track           // 7 
    };

    // IR Sensor Structure
    struct IRSensorData {
        // Sensor states
        bool Read_IR_L;
        bool Read_IR_M;
        bool Read_IR_R;
        uint8_t state;
    };

    // Global IR sensor instance
    extern IRSensorData IRData;

    // Function declarations
    void Init();
    uint8_t ReadSensorState(IRSensorData& IRData);
}

#endif // IR_SENSORS_H