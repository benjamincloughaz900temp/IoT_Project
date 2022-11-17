#include <Servo.h>
#include <LoRa.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"

bool isTimeWithinRange(int day_of_week, int hour, int minute);
bool bothSensorsTripped();
void sendTimeReadingsToWebserver(int hour, int minute, int second, int day);
void lightSwitchServoMovement();

// Servo Variables
// 90 degrees = default position
// 120 degrees = best angle for flicking the light switch
Servo servo;
int servo_pin = 12;
int angle = 90;

// WiFi Variables (Adapted from: https://dev.to/mikulskibartosz/how-to-connect-arduino-uno-wifi-rev-2-to-wifi-59ml and https://docs.arduino.cc/tutorials/communication/wifi-nina-examples)
const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;

// NLP related code adapted from https://randomnerdtutorials.com/esp8266-nodemcu-date-time-ntp-client-server-arduino/ and https://www.elithecomputerguy.com/2019/06/arduino-uno-with-wifi-basic-setup/
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
int status = WL_IDLE_STATUS;
// associating words with indexes for readability
// days
int MONDAY = 0
int TUESDAY = 1
int WEDNESDAY = 2
int THURSDAY = 3
int FRIDAY = 4
int SATURDAY = 5
int SUNDAY = 6

// ranges indexes
int START = 0
int END = 1

// range index
int HOUR = 0
int MINUTE = 1

// Sample wake-up range
// All Days{Monday{Start{Hour, Minute}, End{Hour, Minute}}, ... Sunday{ }}
// wake_up_ranges[1][0][1] = Tuesday - Start - Minute
int wake_up_ranges[] = 
{
  {{12, 0}, {13, 0}}, 
  {{6, 45}, {7, 30}}, 
  {{12, 0}, {13, 0}}, 
  {{6, 45}, {7, 30}}, 
  {{10, 30}, {12, 0}}, 
  {{13, 30}, {15, 0}}, 
  {{13, 30}, {15, 0}}
};

bool alarmTriggered = 0;
// SSL Client & PHP Interaction adapted from WiFiSSLClient.ino (WiFiNINA example sketch)
char server[] = "ysjcs.net";
WiFiSSLClient SSLClient;

// Sensor Variable
unsigned long last_sensor_trip[] = {0, 0};


// TODO ADD METHOD OF DETECTING BOTH SENSORS
//  - Receives 1 sensor signal
//  - If within outlined time range
//  - Take current time
//  - Listen for 20 seconds with a bool value being true
//  - Turn bool value false after 20 seconds unless next sensor sends signal
//  - If both signals received, set off servo & send data to thingspeak
void setup() {
  servo.attach(servo_pin);
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    // Indicate connection status on the arduino
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    }

  // Ensure that LoRa is connected  
  if (!LoRa.begin(868E6)) {
    // Indicate that the connection has failed via the LED
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      }
    }
  }

  timeClient.begin();
  timeClient.update();  
  last_sensor_trip[0] = timeClient.getEpochTime();
  last_sensor_trip[1] = timeClient.getEpochTime();

  servo.write(90);
}

void loop() {
  //Serial.println(timeClient.getEpochTime());
// timeClient.getDay()
// timeClient.getHours()
// timeClient.getMinutes()
// timeClient.getSeconds()
// All in integer format

  // Check for LoRa Packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) 
  {
    timeClient.update();  
    unsigned long current_timestamp = timeClient.getEpochTime();
    // Indicate packet received on the arduino board
    digitalWrite(LED_BUILTIN, HIGH);
    int sensor_reading = (int)LoRa.read() - 49;
    last_sensor_trip[sensor_reading] = current_timestamp;
    if (bothSensorsTripped()) 
    {
      int day_of_week = timeClient.getDay();
      int hour = timeClient.getHours();
      int minute = timeClient.getMinutes();
      if (isTimeWithinRange(day_of_week, hour, minute))
      {
        lightSwitchServoMovement();
        sendTimeReadingsToWebserver(timeClient.getHours(), timeClient.getMinutes(), timeClient.getSeconds(), timeClient.getDay());
        digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

bool isTimeWithinRange(int day_of_week, int hour, int minute) 
{
    
}

void triggerAlarmIfPastTime(int day_of_week, int hour, int minute)
{
  if (!alarmTriggered)
  {
    if (hour >= wake_up_ranges[day_of_week - 1][END][HOUR] && minute >= wake_up_ranges[day_of_week - 1][END][MINUTE])
    {
      alarmTriggered = 1;
      
    }
  }
}

void adjustAlarm(int day_of_week, int hour)
{
  if (alarmTriggered && hour == 0) 
  {
    alarmTriggered = 0;
  }
}

bool bothSensorsTripped() 
{
  Serial.print("Time between sensors: "); Serial.println(abs(last_sensor_trip[0] - last_sensor_trip[1]));
  int time_between_sensors = abs(last_sensor_trip[0] - last_sensor_trip[1]);
  if (time_between_sensors <= 10) { return true; }
  return false;
}

void sendTimeReadingsToWebserver(int hour, int minute, int second, int day)
{
  String http_get_query = "GET /~benjamin.clough/IoT_Project/insertTripReading.php?hour="+String(hour)+"&day="+String(day)+"&minute="+String(minute)+"&second="+String(second)+" HTTP/1.1";
  SSLClient.connect(server, 443);
  SSLClient.println(http_get_query);
  SSLClient.println("Host: ysjcs.net");
  SSLClient.println("Connection: close");
  SSLClient.println();
  while (SSLClient.available()) {
    char c = SSLClient.read();
  }
  SSLClient.flush();
  SSLClient.stop();
  Serial.println(http_get_query);
}

void lightSwitchServoMovement()
{
  servo.write(140);
  delay(1000);
  servo.write(90); 
}
