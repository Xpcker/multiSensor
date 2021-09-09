/*
V.18.16
    - Corrected reboot issue

 V.18.15
    - Corrected wifi crash
 
 V.18.14
  - Enabled OTA
  - Enabled WiFi.onEvent(WiFiEvent)
  - Reboot issue resolved
  - SPIFFS now working with RTOS
  - MQTT Publish function (publisher()) is now being handled by a separate timer
  - Note: SPIFFS is now storing a JSON of 6-10 objects on average. In order to increase the publishing time
          and data storage change eventTime_2_mqttPub, WDT_TIMEOUT and TIME_TO_SLEEP values. 
  - This version is tested with sensors
  
  V.18.13
  - Disabled OTA
  - disabled WiFi.onEvent();

  V.18.12
  -SPIFFS Error resolved, if SPIFFS is not formatted, the FW will auto-format it and try again
  -ESP32 Modem(WiFi+BT) will remain normally in sleep and will wake up when data needs to be published after 3 minutes

  
  V.18.11
  - Added unsigned long currentTime = 0;   , wasn't declared
  - in String readFileContent(), commented the return; it was crashing the compiler.




  V.18.10
  -Updated the FW to get sensor reading every 2 seconds
  -Append the Data to SPIFFS file on each reading
  -After 1000*60*3= 3 Minutes send the data to NodeRED and clear the file
  -Updated NodeRED flow to format the data, convert it into a matrix and send to GSheet


   V.18.9
  NTP moves to RTC
   
  
  V.18.8
  NTP arrives from MQTT
  
  
  V.18.7
  corrected NTP 
  added firmware version, ssid, ip, sensor number to setup
  
  V.18.6
  NTP time
    added to paylaod
  
  V.18.5
  Code cleaned
  
  V.18.4
  added watchdog
  
  V.18.3
  added KeepWiFi Alive
 
  V.18.2
  Sleep mode with interval
  
  V.18
  All code rewritten.

  V.16
  added 10 sensors
  added MultiWiFi
  Battery optimization
     commented non necessary calculations

  V.15
  added Watchdog

  V.14
  commented and removed from send data
  humidity
  atmospheric pressure
  battery voltage

  V.13
  Rearrange code in headers
  added readSensors
  added readNoise
  added wireless
  added googleScript

  V0.12
  Added Hostname, Commented it.
  Updated GScript
  OTA
  BH1750 Correction Multiplier Factor  (usually 1.2)
  removed unnecesarry IR filters for INMP441 and Corrected selected filter

  V0.11
  Go to Sleep if no connection.

  V0.1
  Wifi timeout
  Connection to Google Sheets
  Sensors:
  BH1750
  INM411
  BMW280
*/

#include <Arduino.h>
#include "readSensors.h"
#include "readNoise.h"
#include "wireless.h"

//RTC
#include <ESP32Time.h>
ESP32Time rtc;

String FIRMWARE_VERSION = "0.18.14";
//SPIFFS
#include "storageHandler.h"

//WatchDog
#include <esp_task_wdt.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#define WDT_TIMEOUT 20 // Watchdog Timer in Seconds

//MQTT SEND DATA INTERVAL
unsigned long previousMillis = 0; // Stores last time temperature was published
const long interval = 2000;       // Interval at which to read sensors

//
unsigned long currentTime = 0;

const long eventTime_2_mqttPub =  1000 * 12 * 1; // Interval at which to publish data(after every 1 minutes)

//DEEP SLEEP
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 25       /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;
TimerHandle_t sendDataTimer;
uint8_t writingToFile = 1;


void publisher()
{ 
  esp_task_wdt_reset();
  writingToFile=0;
  wakeModemSleep();
  Serial.print("Sending data: ");
   Serial.println("A");
  appendToFile("}"); //closes the JSON
  Serial.println("B");
  Serial.println(readFileContent());
  Serial.println("C");
  esp_task_wdt_reset();
Serial.println("D");

  
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_ID, 2, true, String(readFileContent()).c_str());
  Serial.printf("Publishing on topic %s at QoS 2, packetId: %i", MQTT_PUB_ID, packetIdPub1);
  Serial.println();
  Serial.println("E");
  
  Serial.print("Clearing the file");
  clearFile();
  Serial.println("F");
  startFileWithBracket();
  writingToFile=1;
  Serial.println("G");
}





void sleepCounter()
{
  ++bootCount;
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
                 " Seconds");
}

void watchDog()
{
  Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, false); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);               //add current thread to WDT watch
}


void getTimeMQTT()
{
  esp_task_wdt_reset(); // Reset WatchDog Counter

  onMqttMessage; //Get the time from Node-Red

  if (!timeClient.update())
  {
    timeClient.forceUpdate();
  }

  String rawNTP;
  rawNTP = timeClient.getFormattedDate();
  int a, m, d, h, n, s;
  a = rawNTP.substring(0, 4).toInt();
  m = rawNTP.substring(5, 7).toInt();
  d = rawNTP.substring(8, 10).toInt();
  h = rawNTP.substring(11, 13).toInt();
  n = rawNTP.substring(14, 16).toInt();
  s = rawNTP.substring(17, 19).toInt();

  rtc.setTime(s, n, h, d, m, a);



  delay(1000);
}


void setup()
{
  Serial.begin(115200);
  delay(500);
  setupSPIFFS();
  startFileWithBracket();

  Wire.begin();

  sleepCounter(); //Initialize Sleep Counter
  watchDog();

  lightMeter.begin(); //Initialize Sensors
  bme.begin(0x76);


  //MQTT
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  sendDataTimer = xTimerCreate("MyTimer", pdMS_TO_TICKS(eventTime_2_mqttPub), pdTRUE, (void *)1, reinterpret_cast<TimerCallbackFunction_t>(publisher));
  if (xTimerStart(sendDataTimer, 10) != pdPASS)
  {
    printf("Timer start error");
  }

  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(); //Mosquitto credentials
  connectToWifi();

  delay(500);

  timeClient.begin(); // GMT -4 = -14400
  timeClient.setTimeOffset(-14400);

  getTimeMQTT();



  Serial.println("-------------------- SETUP --------------------");

  Serial.print("Sensor ID: ");
  Serial.println(idSensor);
  Serial.print("Firmware Version :");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("WiFi MAC Address :");
  Serial.println(WiFi.macAddress());
  Serial.print("Connected to WiFi :");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

void loop()
{
   
    
  esp_task_wdt_reset(); // Reset WatchDog Counter

  /*
 if (WiFi.status() != WL_CONNECTED)
  { // WiFi Check
    Serial.println("Connection Failed, Going to Sleep in Loop");
    goToSleep();
  }
*/

  // MQTT Timer
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    //Read Sensors and String Values
    Serial.println();
    Serial.println("----- Reading Sensors -----");
    readLux();
    readBME();
    readNoise();
    readBatt();
    timeStamp = rtc.getTime("%Y %B %d  %H:%M:%S");   // (String) returns time with specified format 
    //timeStamp = rtc.getEpoch();
    Serial.println("Date Time  :");
    Serial.println(timeStamp);
    String lux_s(lux);
    String temp_s(temp);
    String noise_s(noise);
    String batt_s(batt);
    String dayStamp_s(dayStamp);
    String timeStamp_s(timeStamp);



    esp_task_wdt_reset(); // Reset WatchDog Counter
    if (writingToFile == 1)
    {
      //Create Payload for MQTT
      String payload = "{";
      payload += "\"Sensor\":\"" + idSensor + "\",";
      payload += "\"Time\":\"" + timeStamp_s + "\",";
      payload += "\"Lux\":\"" + lux_s + "\",";
      payload += "\"Temp\":\"" + temp_s + "\",";
      payload += "\"Noise\":\"" + noise_s + "\",";
      payload += "\"Battery\":\"" + batt_s + "\"";
      payload += "}";
      appendToFile(payload);
      payload = ""; //empty the variable
    }
  }


  
}
