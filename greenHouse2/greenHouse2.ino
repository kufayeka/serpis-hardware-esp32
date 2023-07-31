#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;

//MQTT Topics
//------>Publish
const char *clientId = "petra/iot/krpl/serpis/esp32Greenhouse2";
const char *greenhouse2HeartBeatTopic = "petra/iot/krpl/serpis/monitor/greenhouse2/heartbeat";

const char *greenhouse2TemperaturTopic = "petra/iot/krpl/serpis/monitor/greenhouse2/temperatur";
const char *greenhouse2KelembapanTopic = "petra/iot/krpl/serpis/monitor/greenhouse2/kelembapan";

const char *greenhouse2Hidroponik1Topic = "petra/iot/krpl/serpis/monitor/greenhouse2/water1";
const char *greenhouse2Hidroponik2Topic = "petra/iot/krpl/serpis/monitor/greenhouse2/water2";
const char *greenhouse2Hidroponik3Topic = "petra/iot/krpl/serpis/monitor/greenhouse2/water3";
const char *greenhouse2Hidroponik4Topic = "petra/iot/krpl/serpis/monitor/greenhouse2/water4";
const char *greenhouse2TandonTopic = "petra/iot/krpl/serpis/monitor/greenhouse2/tandon_sprayer";
const char *greenhouse2SprayerMonitorTopic = "petra/iot/krpl/serpis/monitor/greenhouse2/sprayer_greenhouse2";

//----->Subscribe
const char *sistemKontrolModeTopic = "petra/iot/krpl/serpis/kontrol/mode";
const char *greenhouse2SprayerKontrolTopic = "petra/iot/krpl/serpis/kontrol/greenhouse2/sprayer_greenhouse2";
const char *greenhouse2SprayerOnThresholdValueTopic = "petra/iot/krpl/serpis/kontrol/greenhouse2/sprayer_greenhouse2/on_thres_value";
const char *greenhouse2SprayerOffThresholdValueTopic = "petra/iot/krpl/serpis/kontrol/greenhouse2/sprayer_greenhouse2/off_thres_value";
const char *greenhouse2SprayerOnTimeTopic = "petra/iot/krpl/serpis/kontrol/greenhouse2/sprayer_greenhouse2/on_time_duration";
const char *greenhouse2SprayerOffTimeTopic = "petra/iot/krpl/serpis/kontrol/greenhouse2/sprayer_greenhouse2/off_time_duration";

//WiFi
const char *ssid = "KRPLSERPIS";  
const char *password = "krplserp1s"; 

//MQTT Broker
const char *mqtt_broker = "203.189.123.207";
const char *mqtt_username = "";
const char *mqtt_password = "";
const int mqtt_port = 1883;

//Pin
const byte relayPin = 17;
const byte buzzerPin = 16;

const byte water1 = 23;
const byte water2 = 13;
const byte water3 = 32;
const byte water4 = 33;
const byte tandon = 25;

const byte ledPowerPin = 4;//
const byte ledWifiPin = 18;
const byte ledMqttPin = 19;

//States
byte relayState = LOW;
byte waterSensorState_1 = LOW;
byte waterSensorState_2 = LOW;
byte waterSensorState_3 = LOW;
byte waterSensorState_4 = LOW;
byte tandonSensorState = LOW;

byte buzzerState = LOW;

char *waterSensor_1Status = "Habis";
char *waterSensor_2Status = "Habis";
char *waterSensor_3Status = "Habis";
char *waterSensor_4Status = "Habis";
char *tandonSensorStatus = "Habis";

bool automaticMode = false;
bool manualMode = false;
bool startPumpOnTimerToggle = false;
bool startPumpOffTimerToggle = false;
bool onThresholdReceived = false;
bool offThresholdReceived = false;
bool onTimeDurationReceived = false;
bool offTimeDurationReceived = false;

//Threshold & Calibration
int pumpOnToken = 3;
float sprayerOnThreshold = 0;
float sprayerOffThreshold = 0;
int sprayerOnTimerDuration = 0;
int sprayerOffTimerDuration = 0;

//Sensor Variable
float greenhouse2Temperature = 0;
float greenhouse2Humidity = 0;

//Intervals
unsigned long previousReadWaterSensorsMillis = 0;
unsigned long previousMqttPublishMillis = 0;

unsigned long readWaterSensorsInterval = 10*1000; // 10 Seconds.
unsigned long mqttPublishInterval = 20*1000; // 1 Minute.

int pumpOnTimerDuration = 0; // get value from server later.
int pumpOffTimerDuration = 0; // get value from server later.
int pumpOnTimerDurationCount = pumpOnTimerDuration;
int pumpOffTimerDurationCount = pumpOffTimerDuration;

unsigned long previousPumpOnMillis = 0;
unsigned long timerOnCountDownInterval = 1000;

unsigned long previousPumpOffMillis = 0;
unsigned long timerOffCountDownInterval = 1000;

unsigned long previousBuzzerMillis = 0;
unsigned long buzzerInterval = 1000;
 
void setup(){
  Serial.begin(9600); // open serial port, set the baud rate to 9600 bps
  int counterWifi = 0;

  pinMode(ledPowerPin, OUTPUT); digitalWrite(ledPowerPin, HIGH);
  pinMode(ledWifiPin, OUTPUT);
  pinMode(ledMqttPin, OUTPUT);
  pinMode(relayPin, OUTPUT); 
  
  pinMode(buzzerPin, OUTPUT);
  
  pinMode(water1, INPUT_PULLDOWN);
  pinMode(water2, INPUT_PULLDOWN);
  pinMode(water3, INPUT_PULLDOWN);
  pinMode(water4, INPUT_PULLDOWN);
  pinMode(tandon, INPUT_PULLDOWN);
  
  WiFi.begin(ssid, password);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    if (counterWifi== 20){
      ESP.restart();
    }
    counterWifi+=1;
    Serial.println("Connecting to WiFi..");
    delay(1000);
    digitalWrite(ledWifiPin, LOW);
  } Serial.println("Connected to the WiFi network"); digitalWrite(ledWifiPin, HIGH);
  
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
 
}

void maintainWifiConn(){
  int counterWifi = 0;
  while (WiFi.status() != WL_CONNECTED){
  if (counterWifi== 5){
      ESP.restart();
      relayState = LOW;
      buzzerState = LOW;
  }
  counterWifi+=1;
    Serial.print("\n Connecting to WiFi again..."); Serial.print(counterWifi);
    Serial.print("\n ");
    digitalWrite(ledWifiPin, LOW);
    delay(1000);
  } 
  Serial.print("\n Still connected to WiFi ");
}

void maintainMqttConn(){
   if (!client.connected()){
    digitalWrite(ledMqttPin, LOW);
      mqttReconnect();
   } 
   client.publish(greenhouse2HeartBeatTopic, "ON");
}

void mqttReconnect(){
  int counterMqtt = 0;
  while (!client.connected()) {
    if (counterMqtt==5){
      ESP.restart();
      relayState = LOW;
      buzzerState = LOW;
    }
  counterMqtt+=1;
  Serial.print("\n Attempting MQTT connection...");
  digitalWrite(ledMqttPin, LOW);
  if (client.connect(clientId)) {
      Serial.println("Mqtt Connected");
      client.subscribe(sistemKontrolModeTopic);
      client.subscribe(greenhouse2SprayerKontrolTopic);
      client.subscribe(greenhouse2SprayerOnThresholdValueTopic);
      client.subscribe(greenhouse2SprayerOffThresholdValueTopic);
      client.subscribe(greenhouse2SprayerOnTimeTopic);
      client.subscribe(greenhouse2SprayerOffTimeTopic);
      digitalWrite(ledMqttPin, HIGH);
  } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length){
  Serial.print("\n Message arrived on topic: "); Serial.print(topic);
  Serial.print("\n Message: "); 
  String messageTemp;
  
  for (int i = 0; i < length; i++){
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  if (String(topic) == sistemKontrolModeTopic){
    if (messageTemp == "Automatis"){
        manualMode = false;
        automaticMode = true;
    } else if (messageTemp == "Manual"){
        manualMode = true;
        automaticMode = false;
    }
  }
  
  if (String(topic) == greenhouse2SprayerKontrolTopic){
    if (messageTemp == "ON"){
        relayState = HIGH;
    } else if (messageTemp == "OFF"){
        relayState = LOW;
    }
  }
  
  if (String(topic) == greenhouse2SprayerOnThresholdValueTopic){
    sprayerOnThreshold = messageTemp.toInt();
    onThresholdReceived = true;
  } 
  
  if (String(topic) == greenhouse2SprayerOffThresholdValueTopic){
    sprayerOffThreshold = messageTemp.toInt();
    offThresholdReceived = true;
  } 

  if (String(topic) == greenhouse2SprayerOnTimeTopic){
    pumpOnTimerDuration = messageTemp.toInt()*60;
    onTimeDurationReceived = true;
  } 
  
  if (String(topic) == greenhouse2SprayerOffTimeTopic){
    pumpOffTimerDuration = messageTemp.toInt()*60;
    offTimeDurationReceived = true;
  } 
 
}

void automaticControl(){
  Serial.print("\n\n ----Kontrol Automatis----");
  Serial.print("\n Sprayer On Threshold:"); Serial.print(sprayerOnThreshold);
  Serial.print("\n Sprayer Off Threshold:"); Serial.print(sprayerOffThreshold);
  if (onThresholdReceived == true && offThresholdReceived == true){
    if(onTimeDurationReceived == true && offTimeDurationReceived == true){
      if (greenhouse2Temperature >= sprayerOnThreshold){
        startPumpOnTimerToggle = true;
      } else if (greenhouse2Temperature <= sprayerOffThreshold){
        startPumpOnTimerToggle = false;
        digitalWrite(relayPin, LOW);
      }
    } else {
    // do nun.
    }
  } else {
    //do nun.
  }
}

void manualControl(){
  //digitalWrite(relayPin, LOW);
  Serial.print("\n\n ----Kontrol Manual----");
  digitalWrite(relayPin, relayState);
}

void readBME280Sensor(){
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(100);
  }
    
    greenhouse2Temperature = bme.readTemperature();
    greenhouse2Humidity = bme.readHumidity();

    Serial.print("\n Temperatur :"); Serial.print(greenhouse2Temperature);
    Serial.print("\n Kelembapan :"); Serial.print(greenhouse2Humidity);
}

void readWaterSensors(){
  if(millis() > previousReadWaterSensorsMillis + readWaterSensorsInterval){
    previousReadWaterSensorsMillis = millis();
    waterSensorState_1 = digitalRead(water1);
    waterSensorState_2 = digitalRead(water2);
    waterSensorState_3 = digitalRead(water3);
    waterSensorState_4 = digitalRead(water4);
    tandonSensorState = digitalRead(tandon);
  }
  
  if (
    waterSensorState_1 == LOW ||
    waterSensorState_2 == LOW ||
    waterSensorState_3 == LOW ||
    waterSensorState_4 == LOW ||
    tandonSensorState == LOW ) {
      buzzerBeep();
    } else {
      buzzerNoBeep();
    }
}

void buzzerBeep(){
  if (millis() - previousBuzzerMillis >= buzzerInterval){
    previousBuzzerMillis = millis();
    if (buzzerState == LOW) {
      buzzerState = HIGH;
    } else {
      buzzerState = LOW;
    }
    digitalWrite(buzzerPin, buzzerState);
  }
}

void buzzerNoBeep(){
  digitalWrite(buzzerPin, LOW);
}

void waterSensorsMqttMsg(){
    //water 1
    if (waterSensorState_1 == LOW){
      waterSensor_1Status = "Habis";
    } else {
      waterSensor_1Status = "Penuh";
    }
  
  //water 2
  if (waterSensorState_2 == LOW){
      waterSensor_2Status = "Habis";
    } else {
      waterSensor_2Status = "Penuh";
    }
  
  //water 3
  if (waterSensorState_3 == LOW){
      waterSensor_3Status = "Habis";
    } else {
      waterSensor_3Status = "Penuh";
    }
  
  //water 4
  if (waterSensorState_4 == LOW){
      waterSensor_4Status = "Habis";
    } else {
      waterSensor_4Status = "Penuh";
    }
  
  //tandon 
  if (tandonSensorState == LOW){
      tandonSensorStatus = "Habis";
    } else {
      tandonSensorStatus = "Penuh";
    }
  
    Serial.print("\n Hidroponik 1:"); Serial.print(waterSensor_1Status);
    Serial.print("\n Hidroponik 2:"); Serial.print(waterSensor_2Status);
    Serial.print("\n Hidroponik 3:"); Serial.print(waterSensor_3Status);
    Serial.print("\n Hidroponik 4:"); Serial.print(waterSensor_4Status);
    Serial.print("\n Tandon Sprayer:"); Serial.print(tandonSensorStatus);
}

void publishSensorsReadings(){
  if(millis() > previousMqttPublishMillis + mqttPublishInterval){
    previousMqttPublishMillis = millis();
  client.publish(greenhouse2TemperaturTopic, String(greenhouse2Temperature).c_str(), true);
  client.publish(greenhouse2KelembapanTopic, String(greenhouse2Humidity).c_str(), true);
  
  client.publish(greenhouse2Hidroponik1Topic, String(waterSensor_1Status).c_str(), true);
  client.publish(greenhouse2Hidroponik2Topic, String(waterSensor_2Status).c_str(), true);
  client.publish(greenhouse2Hidroponik3Topic, String(waterSensor_3Status).c_str(), true);
  client.publish(greenhouse2Hidroponik4Topic, String(waterSensor_4Status).c_str(), true);
  client.publish(greenhouse2TandonTopic, String( tandonSensorStatus).c_str(), true); 
  }
}

void publishRelayStatus(){
  if(digitalRead(relayPin) == HIGH){
    client.publish(greenhouse2SprayerMonitorTopic,"ON", true);
    Serial.print("\n Relay status:"); Serial.print("ON");
  } else {
    client.publish(greenhouse2SprayerMonitorTopic,"OFF", true);
    Serial.print("\n Relay status:"); Serial.print("OFF");
  }
}

void pumpOnTimerSystem(){
    if (startPumpOnTimerToggle == true){
      if (millis() - previousPumpOnMillis >= timerOnCountDownInterval) {
        previousPumpOnMillis = millis();
        pumpOnTimerDurationCount--;
          if(pumpOnTimerDurationCount > 0){
            digitalWrite(relayPin, HIGH);
            pumpOffTimerDurationCount = pumpOffTimerDuration; // renew OFF timer duration.
          } else if (pumpOnTimerDurationCount < 0){
            digitalWrite(relayPin, LOW);
            startPumpOffTimerToggle = true; // start OFF timer.
            startPumpOnTimerToggle = false; // stop ON timer.
          } 
      }
    } else {
    //do nun.
    }
}

void pumpOffTimerSystem(){
  if (startPumpOffTimerToggle == true){
    if (millis() - previousPumpOffMillis >= timerOffCountDownInterval) {
    previousPumpOffMillis = millis();
    pumpOffTimerDurationCount--;
      if (pumpOffTimerDurationCount < 0){
      pumpOnTimerDurationCount = pumpOnTimerDuration; //renew ON timer duration.
      startPumpOffTimerToggle = false; // stop OFF timer.
      startPumpOnTimerToggle = true; // start ON timer.
      }
    }
  } else {
    //do nun.
    }
}

void loop(){
  client.loop();
  if (automaticMode == true && manualMode == false){
    automaticControl();
    pumpOnTimerSystem();
    pumpOffTimerSystem();
    readBME280Sensor();
    readWaterSensors();
    waterSensorsMqttMsg();
    publishSensorsReadings();
    publishRelayStatus();
    Serial.print("\n ON Count Down:"); Serial.print(pumpOnTimerDurationCount);
    Serial.print("\n OFF Count Down:"); Serial.print(pumpOffTimerDurationCount);
  } 
  
  if (automaticMode == false && manualMode == true){
    manualControl();
    readBME280Sensor();
    readWaterSensors();
    waterSensorsMqttMsg();
    publishSensorsReadings();
    publishRelayStatus();
  }
    maintainMqttConn();
    maintainWifiConn();
  delay(1000);
}
