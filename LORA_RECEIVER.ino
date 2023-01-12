#include <Servo.h>
#include <LoRa.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"

// Function definitions
bool isTimeWithinRange(int day_of_week, int hour, int minute);
void triggerAlarmIfPastTime(int day_of_week, int hour, int minute);
void adjustAlarm(int day_of_week, int hour);
bool bothSensorsTripped();
void sendTimeReadingsToWebserver(int hour, int minute, int second, int day);
void lightSwitchServoMovement();

// Tracking time since beginning for alarm purposes
int seconds_since_beginning = 0;
// Servo Variables
// 90 degrees = default position
// 120 degrees = best angle for flicking the light switch
Servo servo;
int servo_pin = 12;
int angle = 90;

// WiFi Variables adapted from: 
// - https://dev.to/mikulskibartosz/how-to-connect-arduino-uno-wifi-rev-2-to-wifi-59ml 
// - https://docs.arduino.cc/tutorials/communication/wifi-nina-examples)
const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;

// NLP related code adapted from: 
// - https://randomnerdtutorials.com/esp8266-nodemcu-date-time-ntp-client-server-arduino/ 
// - https://www.elithecomputerguy.com/2019/06/arduino-uno-with-wifi-basic-setup/
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
int status = WL_IDLE_STATUS;
int day_of_week = 0;
int hour = 0;
int minute = 0;

// associating words with indexes for readability
// days
int MONDAY = 1;
int TUESDAY = 2;
int WEDNESDAY = 3;
int THURSDAY = 4;
int FRIDAY = 5;
int SATURDAY = 6;
int SUNDAY = 0;

// ranges indexes
int START = 0;
int END = 1;

// time index
int HOUR = 0;
int MINUTE = 1;

// Sample wake-up range
// All Days{Monday{Start{Hour, Minute}, End{Hour, Minute}}, ... Sunday{ }}
// wake_up_ranges[1][0][1] = Monday - Start - Minute
int wake_up_ranges[7][2][2] = 
{
  {{13, 30}, {15, 0}},
  {{12, 0}, {12, 59}}, 
  {{6, 45}, {7, 30}}, 
  {{12, 0}, {13, 0}}, 
  {{13, 00}, {23, 30}}, 
  {{0, 20}, {3, 32}}, 
  {{13, 30}, {15, 0}}
};

// SSL Client & PHP Interaction adapted from WiFiSSLClient.ino (WiFiNINA example sketch)
char server[] = "ysjcs.net";
WiFiSSLClient SSLClient;

// Alarm trigger variables
unsigned long last_sensor_trip[] = {0, 0};
bool alarmTriggered = 0;

void setup() {
  servo.attach(servo_pin);
  
  pinMode(LED_BUILTIN, OUTPUT);

  // Connect to WiFI
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    // Indicate connection status on the arduino
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    }

  // Connect to LoRa
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

  // Connect to NLPClient for timekeeping
  timeClient.begin();
  timeClient.update();  

  // Set starting times
  last_sensor_trip[0] = timeClient.getEpochTime();
  last_sensor_trip[1] = timeClient.getEpochTime();
  day_of_week = timeClient.getDay();
  hour = timeClient.getHours();
  minute = timeClient.getMinutes();
  
  // Set default servo angle
  servo.write(90);
}

void loop() {
  // Alarm code
  // update time every minute to reduce cpu usage
  if (millis() % 60000 == 0) 
  { 
    timeClient.update(); 
    
    // Get precise time of day
    int day_of_week = timeClient.getDay();
    int hour = timeClient.getHours();
    int minute = timeClient.getMinutes();

    // Trigger alarm regardless of motion sensors if past certain time
    triggerAlarmIfPastTime(day_of_week, hour, minute);
    adjustAlarm(day_of_week, hour);    
  }
  
  // Check for LoRa Packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) 
  {
    // If LoRa packet received, update last trigger time of the
    // sensor arduino that sent the signal.
    timeClient.update();  
    unsigned long current_timestamp = timeClient.getEpochTime();
    // Indicate packet received on the arduino board
    digitalWrite(LED_BUILTIN, HIGH);
    int sensor_reading = (int)LoRa.read() - 49;
    
    // Update last sensor trigger times
    last_sensor_trip[sensor_reading] = current_timestamp;

    // If both sensors triggered within 10 seconds & within alarm time range, 
    // send date time data to web server
    if (bothSensorsTripped() && !alarmTriggered) 
    {
      
      // CurrentGet Day of the week, Hour, and Minute
      day_of_week = timeClient.getDay();
      hour = timeClient.getHours();
      minute = timeClient.getMinutes();

      // Check if the current time is within alarm time for
      // the current day
      if (isTimeWithinRange(day_of_week, hour, minute))
      {
        // Flick light switch
        lightSwitchServoMovement();

        // Send time data to webserver
        sendTimeReadingsToWebserver(timeClient.getHours(), 
                                    timeClient.getMinutes(), 
                                    timeClient.getSeconds(), 
                                    timeClient.getDay());
        digitalWrite(LED_BUILTIN, LOW);
      }
    } 
  }
}

// Check whether the provided time is within the wake up range for the current day
bool isTimeWithinRange(int day_of_week, int hour, int minute) 
{
  // Convert hours & minutes to total minutes
  int minutes = minute + (hour * 60);

  // compare total minutes to time ranges
  if (minutes >= ((wake_up_ranges[day_of_week][START][HOUR] * 60) + wake_up_ranges[day_of_week][START][MINUTE])  
      && minutes <= (wake_up_ranges[day_of_week][END][HOUR] * 60) + wake_up_ranges[day_of_week][END][MINUTE])
  {
    alarmTriggered = 1;
    return true; 
  }
  return false;
}

// Turn on light switch without motion sensor triggers if past a certain time
void triggerAlarmIfPastTime(int day_of_week, int hour, int minute)
{
  int minutes = minute + (hour * 60);
  if (!alarmTriggered)
  {
    // wake_up_ranges[day_of_week][END][HOUR] is the alarm time range end hour
    // (wake_up_ranges[day_of_week][END][HOUR] * 60) + wake_up_ranges[day_of_week][END][MINUTE]) is the
    // total minutes passed in the day
    if ((hour >= wake_up_ranges[day_of_week][END][HOUR]) 
    || ((minutes >= (wake_up_ranges[day_of_week][END][HOUR] * 60) + wake_up_ranges[day_of_week][END][MINUTE])))
    {
      alarmTriggered = 1;
      // Flick the light switch
      lightSwitchServoMovement();
    }
  }
}

// Reset the alarm upon reaching midnight
void adjustAlarm(int day_of_week, int hour)
{
  if (alarmTriggered && hour == 0) 
  {
    alarmTriggered = 0;
  }
}

// Returns whether both motion sensors were triggered within 10 seconds of each other
bool bothSensorsTripped() 
{
  int time_between_sensors = abs(last_sensor_trip[0] - last_sensor_trip[1]);
  if (time_between_sensors <= 10) { return true; }
  return false;
}

// Send data to the ysjcs.net server using HTTPS & SSL
void sendTimeReadingsToWebserver(int hour, int minute, int second, int day)
{
  // construct destination (PHP script on webserver)
  String http_get_query = "GET /~benjamin.clough/IoT_Project/insertTripReading.php?hour="
                          +String(hour)
                          +"&day="+String(day)
                          +"&minute="+String(minute)
                          +"&second="+String(second)
                          +" HTTP/1.1";
  // construct and send the HTTPS packet
  SSLClient.connect(server, 443);
  SSLClient.println(http_get_query);
  SSLClient.println("Host: ysjcs.net");
  SSLClient.println("Connection: close");
  SSLClient.println();
  while (SSLClient.available()) 
  {
    char c = SSLClient.read();
  }
  SSLClient.flush();
  SSLClient.stop();
}

// Flick the light switch with this movement
void lightSwitchServoMovement()
{
  servo.write(140);
  delay(1000);
  servo.write(90); 
}
