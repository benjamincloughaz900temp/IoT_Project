#include <SPI.h>
#include <LoRa.h>

#define SensorPin 11

int val = 0;
bool motion_detected = false;

void setup() {
  // Define the pins attached the the motion sensor
  pinMode(SensorPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  // Begin LoRa connection
  if (!LoRa.begin(868E6)) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (1);
 }
}

void loop() {
  // Read the motion sensor
  // val = HIGH = motion detected
  val = digitalRead(SensorPin);
  if (val == HIGH) {
    
    if (motion_detected == false) {
      motion_detected = true;
      // Indicate on the Arduino that motion was detected
      digitalWrite(LED_BUILTIN, HIGH);
      // Send packet containing the ID of the arduino
      // This script is for arduino 1
      // Replaced with 2 for the second arduino
      LoRa.beginPacket();
      LoRa.print(1);
      LoRa.endPacket();
      // LoRa has 30 second limit on transmission
      // Therefore 30 second wait is required
      delay(30000);
    }
    
  }
  else {
    if (motion_detected == true) {
      motion_detected = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
  
  }
  
}
