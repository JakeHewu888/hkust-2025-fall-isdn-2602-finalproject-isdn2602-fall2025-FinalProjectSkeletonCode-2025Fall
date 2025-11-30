#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

// Arduino / ESP32 Libraries
#include <Arduino.h>
#include <ArduinoJson.h>
#include <FirebaseClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "Buzzer.hpp"  //Buzzer in case you want to debug using buzzer
#include "IMU.h"       //Optional IMU driver used in sensor lab, Please refer to the Sensor lab code for how to use IMU API
#include "IRSensors.hpp"
#include "MotorControl.hpp"
#include "Movement.hpp"
#include "Pinout.hpp"
#include "RFIDReader.hpp"
#include "UltrasonicSensor.hpp"
#include "config.h"   // For Firebase and WiFi config
#include "pitches.h"  //For Buzzer
#include "wifi_manager.h"
#include "dijkstra.hpp"
#include "RFIDmap.hpp"

/*==================================================================================
 * Firebase Related
 *==================================================================================*/

/* Firebase Components */
FirebaseApp app;
WiFiClientSecure ssl_client1;
using AsyncClient = AsyncClientClass;
AsyncClient aClient1(ssl_client1);
RealtimeDatabase Database;
NoAuth no_auth;

/* Firebase Main Task */
StackType_t uxFirebaseMainTask[configMINIMAL_STACK_SIZE];
StaticTask_t xFirebaseMainTaskTCB;
TaskHandle_t FirebaseMainTaskTCB;

void FirebaseMainTask(void* pvPara) {
  while (true) {
    // Check WiFi connection first
    if (WiFiManager::isConnected()) {
    } else {
      Serial.println("WiFi disconnected, attempting to reconnect...");
      WiFiManager::reconnect();
    }
    // Handle Firebase tasks
    app.loop();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/* Firebase Read Task */
StackType_t uxFirebaseReadTask[configMINIMAL_STACK_SIZE];
StaticTask_t xFirebaseReadTaskTCB;
TaskHandle_t FirebaseReadTaskTCB;

void FirebaseReadTask(void* pvPara) {
  while (true) {
    // Check if authentication is ready
    if (app.ready()) {
      unsigned long currentTime = millis();
      // Periodic data reading every readInterval, but only if not busy
      if (currentTime - lastReadTime >= readInterval && !readingBusy) {
        lastReadTime = currentTime;  // Update the last read time
        readingBusy = true;          // Set busy before request

        // Debug - Log the request paths
        // Serial.println("Requesting exam state from: " + examStatePath);
        // Serial.println("Requesting traffic lights from: " +
        // trafficLightPath);

        Database.get(aClient1, examStatePath, processData, false,
                     "RTDB_GetExamState");
        Database.get(aClient1, trafficLightPath, processData, false,
                     "RTDB_GetTrafficLight");

        readingBusy = false;  // Clear busy after request
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

/* Firebase Data Processing Helper Function */
void processData(AsyncResult& aResult) {
  if (!aResult.isResult()) return;

  //   DEBUG - Log event, debug, and error messages
  // if (aResult.isEvent())
  //   Firebase.printf("Event task: %s, msg: %s, code: %d\n",
  //                   aResult.uid().c_str(),
  //                   aResult.eventLog().message().c_str(),
  //                   aResult.eventLog().code());

  // if (aResult.isDebug())
  //   Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(),
  //                   aResult.debug().c_str());

  //   if (aResult.isError())
  //     Firebase.printf("Error task: %s, msg: %s, code: %d\n",
  //                     aResult.uid().c_str(),
  //                     aResult.error().message().c_str(),
  //                     aResult.error().code());

  if (aResult.available()) {
    // DEBUG - Log the task and payload
    // Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(),
    //                 aResult.c_str());

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, aResult.c_str());
    if (error) {
      Firebase.printf("Failed to parse JSON: %s\n", error.c_str());
      return;
    }
    JsonObject obj = doc.as<JsonObject>();

    if (aResult.uid() == "RTDB_GetExamState") {
      examState.activated = obj["activated"].as<bool>();
      examState.start_point = obj["start_point"].as<int>();
      examState.end_point = obj["end_point"].as<int>();
      examState.field = obj["field"].as<String>();
      examState.task_id = obj["task_id"].as<int>();
      examState.time_remain = obj["time_remain"].as<int>();
      //  Debug - print the current exam state
      // Serial.println("===== Exam State =====");
      // Serial.println("Activated: " + String(examState.activated));
      // Serial.println("Start Point: " + String(examState.start_point));
      // Serial.println("End Point: " + String(examState.end_point));
      // Serial.println("Field: " + examState.field);
      // Serial.println("Task ID: " + String(examState.task_id));
      // Serial.println("Time Remain: " + String(examState.time_remain));
    }
    if (aResult.uid() == "RTDB_GetTrafficLight") {
      JsonArray arr = doc.as<JsonArray>();
      // Debug - print each traffic light
      // Serial.println("===== Traffic Light States =====");
      // Parse traffic lights data
      // Light ID 1-4 are for final exam, ID 5 is for the testing field
      for (int i = 0; i < numTrafficLights + 1 && i < arr.size(); i++) {
        JsonObject light = arr[i];
        if (!light.isNull()) {
          trafficLights[i].id = light["id"].as<int>();
          trafficLights[i].current_state = light["current_state"].as<String>();
          trafficLights[i].time_remain = light["time_remain"].as<int>();

          // Debug - print each traffic light
          // Serial.printf("Light %d: id=%d, state=%s, time_remain=%d\n", i,
          //               trafficLights[i].id,
          //               trafficLights[i].current_state.c_str(),
          //               trafficLights[i].time_remain);
        }
      }
    }
  }
}

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
  volatile float max_integral = 3000.0f;
  volatile float min_integral = 0.0f;

  volatile float target_val;  // The target RPM
  float actual_val;           // Actual RPM Reading
  float err;                  // Error
  float err_last;
  float integral;

  /*General PID Function*/
  float PID_realize(float temp_val) {
    this->err = this->target_val - temp_val;

    this->integral += this->err;

    // Limit -> in case too high accumulation
    if (integral > max_integral)
      integral = max_integral;
    if (integral < min_integral)
      integral = min_integral;

    this->actual_val = this->Kp * this->err + this->Ki * this->integral + this->Kd * (this->err - this->err_last);

    this->err_last = this->err;

    if (this->actual_val < 0) {
      this->actual_val = 0 - this->actual_val;
    }

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
  LeftWheelPID.Kp = 3.0f;
  LeftWheelPID.Ki = 0.7f;
  LeftWheelPID.Kd = 0.0f;
  LeftWheelPID.target_val = 200.0f;
  LeftWheelPID.err = 0.0f;
  LeftWheelPID.err_last = 0.0f;
  LeftWheelPID.integral = 0.0f;

  /*RightMotor PID*/
  RightWheelPID.Kp = 3.0f;
  RightWheelPID.Ki = 0.7f;
  RightWheelPID.Kd = 0.0f;
  RightWheelPID.target_val = 200.0f;
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
    // Serial.print("Speed Left: ");
    // Serial.println(MotorControl::LeftWheel.Speed);

    // Serial.print("Speed Right: ");
    // Serial.println(MotorControl::RightWheel.Speed);
    /*----------------------------------------------------*/
    /*A delay must be added inside each User Task*/
    vTaskDelay(50);
  }
}

/*-------------RFID Tag Reader Task-------------*/
TaskHandle_t RFIDTagReaderTaskHandle = NULL;
StaticTask_t xRFIDTagReaderTCB;
extern String currenttagUID = "";
void RFIDTagReaderTask(void* pvPara) {
  while (true) {
    // If no new card or read failed, wait and continue
    if (!RFIDReader::mfrc522.PICC_IsNewCardPresent() || !RFIDReader::mfrc522.PICC_ReadCardSerial()) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    currenttagUID = RFIDReader::GetTagUID();
    

    // Debug - print the current RFID tag
    Serial.print("RFID Tag: ");
    Serial.println(currenttagUID);

    vTaskDelay(20 / portTICK_PERIOD_MS);
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
        LeftWheelPID.target_val = 110.0f;
        RightWheelPID.target_val = 110.0f;
        Movement::MoveForward();
        vTaskDelay(20);
        break;

      case IRSensors::Left_Middle_ON_Track:
        LeftWheelPID.target_val = 0.0f;
        RightWheelPID.target_val = 350.0f;
        Movement::RotateLeft();
        vTaskDelay(400);
        break;

      case IRSensors::ALL_ON_Track:
        LeftWheelPID.target_val = 0.0f;
        RightWheelPID.target_val = 0.0f;
        Movement::Stop();
        vTaskDelay(50);
        break;

      case IRSensors::Left_Right_ON_Track:
        LeftWheelPID.target_val = 0.0f;
        RightWheelPID.target_val = 0.0f;
        Movement::Stop();
        vTaskDelay(50);
        break;

      case IRSensors::Middle_Right_ON_Track:
        LeftWheelPID.target_val = 350.0f;
        RightWheelPID.target_val = 0.0f;
        Movement::RotateRight();
        vTaskDelay(400);
        break;

      case IRSensors::Right_ON_Track:
        LeftWheelPID.target_val = 110.0f;
        RightWheelPID.target_val = 110.0f;
        Movement::FixRight();
        vTaskDelay(20);
        break;

      case IRSensors::Left_ON_Track:
        LeftWheelPID.target_val = 110.0f;
        RightWheelPID.target_val = 110.0f;
        Movement::FixLeft();
        vTaskDelay(20);
        break;

      case IRSensors::All_OFF_Track:
        LeftWheelPID.target_val = 110.0f;
        RightWheelPID.target_val = 110.0f;
        Movement::MoveForward();
        vTaskDelay(20);
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


void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  Serial.println("---------Start Initialization---------");

  if (demoMode) {
    examStatePath = "/admin/examState/";
  } else {
    examStatePath = "/users/" + UID + "/examState/";
  }

  IRSensors::Init();
  MotorControl::DCMotorControl::Init();
  MotorControl::ServoMotorControl::Init();
  Encoder::Init();
  UltrasonicSensor::Init();
  Buzzer::Init();
  RFIDReader::Init();

  Serial.println("All sensors initialized");

  // Initialize the Wi-Fi Connection
  // LED blinking while connecting to WiFi
  WiFiManager::initialize();

  // Configure SSL clients
  ssl_client1.setInsecure();
  ssl_client1.setConnectionTimeout(1000);
  ssl_client1.setHandshakeTimeout(5);

  // Initialize Firebase Realtime Database
  initializeApp(aClient1, app, getAuth(no_auth), processData);
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);
  Serial.println("FireBase Initialized");

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

  xTaskCreatePinnedToCore(FirebaseMainTask, "Firebase Main Task", 8192, NULL, 1,
                          &FirebaseMainTaskTCB, 0);
  xTaskCreatePinnedToCore(FirebaseReadTask, "Firebase Read Task", 8192, NULL, 2,
                          &FirebaseReadTaskTCB, 0);

  xTaskCreatePinnedToCore(LEDBlinkingTask, "Blinking", 2000, NULL, 1,
                          &LEDBlinkingTaskHandle, 1);

  xTaskCreatePinnedToCore(MovementTask, "Movement", 12000, NULL, 3,
                          &MovementTaskHandle, 1);

  xTaskCreatePinnedToCore(SpeedControlTask, "SpeedControl", 10000, NULL, 4,
                          &SpeedControlTaskTCB, 1);

  xTaskCreatePinnedToCore(RFIDTagReaderTask, "RFID Tag Reader Task", 4096, NULL,
                          2, &RFIDTagReaderTaskHandle, 1);

  Serial.println("---------Initialization Completed---------");
  vTaskDelay(10);
}

void loop() {
  /*---------------------------------Dijkstra_function----------------------------------------------------*/
  auto result = dijkstra(examState.start_point, examState.end_point);
  std::vector<int> path = result.first;
  int cost = result.second;

  /*------------------------DEBUG:Dijkstra_function----------------------------------------------------*/
  // Serial.print("Cost: ");
  // Serial.println(cost);

  // Serial.print("Path: ");
  // for (size_t i = 0; i < path.size(); i++) {
  //   Serial.print(path[i]);
  //   if (i < path.size() - 1) {
  //     Serial.print(" -> ");
  //   }
  // }
  // Serial.println();

  /*------------------------scan RFID turn to Tile----------------------------------------------------*/

  auto tileID = scanRFIDtoTILE(currenttagUID);

  /*------------------------DEBUG:scan RFID turn to Tile----------------------------------------------------*/
  // Serial.print(" This tagis in tile ");
  // Serial.print(tileID);
}