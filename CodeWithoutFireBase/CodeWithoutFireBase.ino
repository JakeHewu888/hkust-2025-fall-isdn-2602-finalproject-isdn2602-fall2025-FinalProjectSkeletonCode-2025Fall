#include <Arduino.h>

#include "Buzzer.hpp"  //Buzzer in case you want to debug using buzzer
#include "IMU.h"       //Optional IMU driver used in sensor lab,
#include "IRSensors.hpp"
#include "MotorControl.hpp"
#include "Movement.hpp"
#include "Pinout.hpp"
#include "RFIDReader.hpp"
#include "UltrasonicSensor.hpp"
#include "pitches.h"  //For Buzzer
// Please refer to the Sensor lab code for how to use IMU API

/*-------------------------------------------------------------------------------------------------------------------------*/
/*-------------RPM Measure Task-------------*/

/*--------Stack and Handle Settings---------*/
StackType_t SpeedControlTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t xSpeedControlTaskTCB;
TaskHandle_t SpeedControlTaskTCB;

/*-------------------------------------------------------------------------------------------------------------------------*/
/*-------------RPM Measure User Task-------------*/
/*Setting of the PID para.*/
struct PID_t {
  /*Creating the parameters for PID*/
  volatile float Kp;
  volatile float Ki;
  volatile float Kd;

  volatile float target_val;  // The target RPM
  float actual_val;           // Actual RPM Reading
  float err;                  // Error
  float err_last;
  float integral;

  /*General PID Function*/
  float PID_realize(float temp_val) {
    this->err = this->target_val - temp_val;

    this->integral += this->err;

    this->actual_val = this->Kp * this->err + this->Ki * this->integral +
                       this->Kd * (this->err - this->err_last);

    this->err_last = this->err;

    return this->actual_val;
  }

} PID;

/*Global PID controllers for left and right wheels*/
PID_t LeftWheelPID;
PID_t RightWheelPID;

void SpeedControlTask(void* pvPara) {
  /*Setup for the Task*/
  /*----------------------------------------------------*/
  /*Initalize the Speed of the motor*/
  MotorControl::LeftWheel.Speed = 0;
  MotorControl::RightWheel.Speed = 0;

  /*Initialize PID Parameters*/
  /*LeftMotor PID*/
  LeftWheelPID.Kp = 0.0f;
  LeftWheelPID.Ki = 0.0f;
  LeftWheelPID.Kd = 0.0f;
  LeftWheelPID.target_val = 150.0f;
  LeftWheelPID.err = 0.0f;
  LeftWheelPID.err_last = 0.0f;
  LeftWheelPID.integral = 0.0f;

  /*RightMotor PID*/
  RightWheelPID.Kp = 0.0f;
  RightWheelPID.Ki = 0.0f;
  RightWheelPID.Kd = 0.0f;
  RightWheelPID.target_val = 150.0f;
  RightWheelPID.err = 0.0f;
  RightWheelPID.err_last = 0.0f;
  RightWheelPID.integral = 0.0f;
  /*----------------------------------------------------*/
  while (true) {
    /*----------------------------------------------------*/
    /*Get the RPM from Encoder*/
    Encoder::RPMCounterFromEncoder(LeftWheelRPM);
    Encoder::RPMCounterFromEncoder(RightWheelRPM);
    /*----------------------------------------------------*/
    /*Compute the PID and Write the Result to Speed of the Wheel*/
    MotorControl::LeftWheel.Speed = LeftWheelPID.PID_realize(LeftWheelRPM.rpm);
    MotorControl::RightWheel.Speed =
        RightWheelPID.PID_realize(RightWheelRPM.rpm);
    /*----------------------------------------------------*/
    /*FOR DEBUG USAGE*/
    Serial.print("Speed Left: ");
    Serial.println(MotorControl::LeftWheel.Speed);

    Serial.print("Speed Right: ");
    Serial.println(MotorControl::RightWheel.Speed);
    /*----------------------------------------------------*/
    /*A delay must be added inside each User Task*/
    vTaskDelay(50);
  }
}

/*Creating User Task in FreeRTOS*/
/*-------------LED Blinking Task-------------*/
TaskHandle_t LEDBlinkingTaskHandle = NULL;
StaticTask_t xLEDBlinkingTCB;
void LEDBlinkingTask(void* pvPara) {
  /*The code before entering while loop will be only run once*/
  pinMode(Pinout::Led, OUTPUT);
  while (true) {
    /* The LED will start blinking if the task is running */
    digitalWrite(Pinout::Led, HIGH);
    vTaskDelay(500);
    digitalWrite(Pinout::Led, LOW);
    vTaskDelay(500);
  }
};

TaskHandle_t MovementTaskHandle = NULL;
StaticTask_t xMovementTCB;
void MovementTask(void* pvPara) {
  // IrSensorData IRData;
  while (true) {
    // Get the status of the IR sensor and store in IRData
    IRSensors::IRData.state = IRSensors::ReadSensorState(IRSensors::IRData);
    /*-------For Debug Use------*/
    // Serial.print("IR Status: ");
    // Serial.print(IRSensors::IRData.Read_IR_L);
    // Serial.print(IRSensors::IRData.Read_IR_M);
    // Serial.println(IRSensors::IRData.Read_IR_R);

    // Serial.print("Condition: ");
    // Serial.println(IRSensors::IRData.state);
    // Depend on the condition, do line tracking
    // Using switch case for the movement control
    switch (IRSensors::IRData.state) {
      case IRSensors::Middle_ON_Track:
        LeftWheelPID.target_val = 150.0f;
        RightWheelPID.target_val = 150.0f;
        Movement::MoveForward();
        vTaskDelay(100);
        break;

      case IRSensors::Left_Middle_ON_Track:
        LeftWheelPID.target_val = 150.0f;
        RightWheelPID.target_val = 150.0f;
        Movement::RotateLeft();
        vTaskDelay(100);
        break;

      case IRSensors::ALL_ON_Track:
        LeftWheelPID.target_val = 150.0f;
        RightWheelPID.target_val = 150.0f;
        Movement::Stop();
        vTaskDelay(100);
        break;

      case IRSensors::Left_Right_ON_Track:
        LeftWheelPID.target_val = 150.0f;
        RightWheelPID.target_val = 150.0f;
        Movement::Stop();
        vTaskDelay(100);
        break;

      case IRSensors::Middle_Right_ON_Track:
        LeftWheelPID.target_val = 150.0f;
        RightWheelPID.target_val = 150.0f;
        Movement::RotateRight();
        vTaskDelay(100);
        break;

      case IRSensors::Right_ON_Track:
        LeftWheelPID.target_val = 150.0f;
        RightWheelPID.target_val = 150.0f;
        Movement::RotateRight();
        vTaskDelay(100);
        break;

      case IRSensors::Left_ON_Track:
        LeftWheelPID.target_val = 150.0f;
        RightWheelPID.target_val = 150.0f;
        Movement::RotateLeft();
        vTaskDelay(100);
        break;

      case IRSensors::All_OFF_Track:
        LeftWheelPID.target_val = 150.0f;
        RightWheelPID.target_val = 150.0f;
        Movement::Stop();
        vTaskDelay(100);
        break;
    }
    // /*-------For Debug Use------*/
    // Serial.print("LeftWheel.Speed: ");
    // Serial.println(MotorControl::LeftWheel.Speed);
    // Serial.print("RightWheel.Speed: ");
    // Serial.println(MotorControl::RightWheel.Speed);

    // /*-------For Debug Use------*/
    // Serial.print("LeftWheel.PWMIN1: ");
    // Serial.print(MotorControl::LeftWheel.PWMChannelIN1);
    // Serial.println(MotorControl::LeftWheel.PWMChannelIN2);

    // Serial.print("RightWheel.PWMIN2: ");
    // Serial.print(MotorControl::RightWheel.PWMChannelIN1);
    // Serial.println(MotorControl::RightWheel.PWMChannelIN2);

    vTaskDelay(10);
  }
};

/*-------------RFID Tag Reader Task-------------*/
TaskHandle_t RFIDTagReaderTaskHandle = NULL;
StaticTask_t xRFIDTagReaderTCB;
void RFIDTagReaderTask(void* pvPara) {
  while (true) {
    // If no new card or read failed, wait and continue
    if (!RFIDReader::mfrc522.PICC_IsNewCardPresent() ||
        !RFIDReader::mfrc522.PICC_ReadCardSerial()) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    String currenttagUID = RFIDReader::GetTagUID();

    // Debug - print the current RFID tag
    // Serial.print("RFID Tag: ");
    // Serial.println(currenttagUID);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  IRSensors::Init();
  MotorControl::DCMotorControl::Init();
  MotorControl::ServoMotorControl::Init();
  Encoder::Init();
  Serial.println("Motors Pin Initialized.");
  UltrasonicSensor::Init();
  Buzzer::Init();
  RFIDReader::Init();

  // Create FreeRTOS tasks with proper namespaced functions
  // Task creation function:
  // TaskHandle_t xTaskCreateStaticPinnedToCore(
  //     TaskFunction_t pxTaskCode,
  //     const char *const pcName,
  //     const uint32_t ulStackDepth,
  //     void *const pvParameters,
  //     UBaseType_t uxPriority,
  //     StackType_t *const puxStackBuffer,
  //     StaticTask_t *const pxTaskBuffer,
  //     const BaseType_t xCoreID
  // );

  xTaskCreatePinnedToCore(LEDBlinkingTask, "Blinking", 2000, NULL, 1,
                          &LEDBlinkingTaskHandle, 1);

  xTaskCreatePinnedToCore(MovementTask, "Movement", 12000, NULL, 3,
                          &MovementTaskHandle, 1);

  xTaskCreatePinnedToCore(SpeedControlTask, "SpeedControl", 10000, NULL, 4,
                          &SpeedControlTaskTCB, 1);

  xTaskCreatePinnedToCore(RFIDTagReaderTask, "RFID Tag Reader Task", 4096, NULL,
                          2, &RFIDTagReaderTaskHandle, 1);

  Serial.println("Robot initialized and running");
  vTaskDelay(10);
}

void loop() {}