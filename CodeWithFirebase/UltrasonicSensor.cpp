#include "UltrasonicSensor.hpp"
#define SOUND_SPEED 340
void UltrasonicSensor::Init(){ 
  pinMode(Pinout::UltrasonicTrigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(Pinout::UltrasonicEchoPin, INPUT); // Sets the echoPin as an Input
  Serial.println("Ultrasonic Sensor is set");
}

float UltrasonicSensor::GetDistance(){
  long duration = 0;  //initalize the temp para. 
  float distance = 0; 
  // Clears the trigPin
  digitalWrite(Pinout::UltrasonicTrigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(Pinout::UltrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Pinout::UltrasonicTrigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(Pinout::UltrasonicEchoPin, HIGH);
  
  // Calculate the distance (in m)
  distance = (duration * SOUND_SPEED/100)/2;
  return distance; 
  //For Debug Use
  // Prints the distance in the Serial Monitor
  // Serial.print("Distance (cm): ");
  // Serial.println(distance/100);
}