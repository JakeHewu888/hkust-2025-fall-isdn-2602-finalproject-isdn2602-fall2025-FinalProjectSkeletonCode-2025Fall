#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"

// Demo config
extern const bool demoMode;  // Set to true to enable demo mode

// WiFi Configuration
extern const char WIFI_SSID[];
extern const char WIFI_PASSWORD[];

// Firebase Credentials
extern const String API_KEY;
extern const String DATABASE_URL;

// Firebase reading config
extern bool readingBusy;  // prevent overlapping reads
extern unsigned long lastReadTime;
extern const unsigned long readInterval;  // 1 seconds
extern const String UID;
extern const String trafficLightPath;
extern String examStatePath;

// LED Pin
extern const int LED_PIN;

// Firebase data structures
struct ExamState {
  bool activated = false;
  int start_point;
  int end_point;
  String field;
  int task_id;
  int time_remain;
};

extern ExamState examState;
extern const int numTrafficLights;  // Adjust based on your setup
struct TrafficLight {
  int id;
  String current_state;
  int time_remain;
};

extern TrafficLight trafficLights[];  // Array of traffic lights

#endif  // CONFIG_H