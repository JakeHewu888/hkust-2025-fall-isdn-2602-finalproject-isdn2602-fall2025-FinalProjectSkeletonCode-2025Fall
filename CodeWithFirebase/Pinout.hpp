#ifndef PINOUT_H
#define PINOUT_H

#include <stdint.h>

namespace Pinout {
    // Debug and Status
    const uint8_t Debug = 0;
    const uint8_t Led = 13;

    // Buzzer 
    const uint8_t Buzzer = 40;

    // IR (Infra-red) Sensors
    const uint8_t IRLeft = 6;
    const uint8_t IRMiddle = 4;
    const uint8_t IRRight = 5;

    // Ultrasonic Sensor 
    const uint8_t UltrasonicTrigPin = 6;
    const uint8_t UltrasonicEchoPin = 4;
    // Left Motor (LM)
    const uint8_t LeftMotorIn1 = 12;
    const uint8_t LeftMotorIn2 = 11;
    const uint8_t LeftMotorEncoderA = 10;
    const uint8_t LeftMotorEncoderB = 9;

    // Right Motor (RM)
    const uint8_t RightMotorIn1 = 21;
    const uint8_t RightMotorIn2 = 14;
    const uint8_t RightMotorEncoderA = 47;
    const uint8_t RightMotorEncoderB = 48;

    //RFID Reader 
    const uint8_t RFID_RST = 42;
    const uint8_t  RFID_SCL = 41;
    const uint8_t  RFID_SDA = 36;
    const uint8_t  RFID_IRQ = 35;

    //IMU 
    const uint8_t IMU_SDA = 17;
    const uint8_t IMU_SCL = 18;
    const uint8_t IMU_DRDY = 7;

    // Servo Motor
    const uint8_t ServoPin = 2;
}

#endif // PINOUT_H