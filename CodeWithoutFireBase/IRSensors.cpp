#include "esp32-hal-gpio.h"
#include "IRSensors.hpp"
#include "pinout.hpp"
#include <Arduino.h>
namespace IRSensors {
    // Global IR sensor instance
    IRSensorData IRData = {
        false,          //bool Read_IR_L;
        false,          //bool Read_IR_M;
        false,          //bool Read_IR_R;
        Middle_ON_Track //RobotState state;
        };

    void Init() {
        // Initialize IR sensor pins
        pinMode(Pinout::IRLeft, INPUT);
        pinMode(Pinout::IRMiddle, INPUT);
        pinMode(Pinout::IRRight, INPUT);


    };


    uint8_t ReadSensorState(IRSensorData& IRData){
        /*Scan all the IR sensors to get the current status of the car*/
        IRData.Read_IR_L = digitalRead(Pinout::IRLeft);
        IRData.Read_IR_M = digitalRead(Pinout::IRMiddle);
        IRData.Read_IR_R = digitalRead(Pinout::IRRight);

        /*According to the current status of the IR Sensors, determine the RobotState*/
        if(IRData.Read_IR_L == 0 && IRData.Read_IR_M == 1 && IRData.Read_IR_R == 0 ){
            return Middle_ON_Track;
        }
        else if (IRData.Read_IR_L == 1 && IRData.Read_IR_M == 1 && IRData.Read_IR_R == 0 ) {
            return Left_Middle_ON_Track;
        }
        else if (IRData.Read_IR_L == 1 && IRData.Read_IR_M == 1 && IRData.Read_IR_R == 1 ) {
            return ALL_ON_Track;
        }
        else if (IRData.Read_IR_L == 1 && IRData.Read_IR_M == 0 && IRData.Read_IR_R == 1 ) {
            return Left_Right_ON_Track;
        }
        else if (IRData.Read_IR_L == 0 && IRData.Read_IR_M == 1 && IRData.Read_IR_R == 1 ) {
            return Middle_Right_ON_Track;
        }
        else if (IRData.Read_IR_L == 0 && IRData.Read_IR_M == 0 && IRData.Read_IR_R == 1 ) {
            return Right_ON_Track;
        }
        else if (IRData.Read_IR_L == 1 && IRData.Read_IR_M == 0 && IRData.Read_IR_R == 0 ) {
            return Left_ON_Track;
        }
        else if (IRData.Read_IR_L == 0 && IRData.Read_IR_M == 0 && IRData.Read_IR_R == 0 ) {
            return All_OFF_Track;
        }


        return 10; //Error Code if none of condition fits.
    }

}