#include "config.h"

// TODO
// Demo config
const bool demoMode = false;
// Set to true to enable demo mode - to read from examStates data from /admin
// Set to false to read from /users/<uid>/examState when testing by yourself

// WiFi Configuration
const char WIFI_SSID[] = "ISDN2602_2G";
const char WIFI_PASSWORD[] = "isdn2602@iot";

// Firebase Credentials
const String API_KEY = "AIzaSyAIfJ94DJqUXPEyM4N4dy_2bfSdFOatns0";
const String DATABASE_URL =
    "https://isdn2602-control-panel-2025-default-rtdb.firebaseio.com/";
// TODO
const String UID = "your-uid";  // you can find it from the
                                // control panel site

// LED Pin
const int LED_PIN = 13;

// Firebase reading config
bool readingBusy = false;  // prevent overlapping reads
unsigned long lastReadTime = 0;
const unsigned long readInterval = 1000;  // 1 seconds

ExamState examState;

// Traffic lights configuration
const int numTrafficLights = 5;
TrafficLight trafficLights[numTrafficLights + 1];

const String trafficLightPath = "/admin/trafficLights/";
String examStatePath = "";