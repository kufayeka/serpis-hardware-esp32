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
const char *clientId = "petra/iot/krpl/serpis/esp32MediaTanah1";
const char *mediaTanah1HeartBeatTopic = "petra/iot/krpl/serpis/monitor/mediatanah1/heartbeat";
const char *mediaTanahKelembapanTanah1Topic = "petra/iot/krpl/serpis/monitor/mediatanah/kelembapan_tanah1";
const char *mediaTanahSprayerMonitorTopic   = "petra/iot/krpl/serpis/monitor/mediatanah/sprayer_mediatanah";
const char *mediaTanahTandonTopic = "petra/iot/krpl/serpis/monitor/mediatanah/tandon_sprayer";

//----->Subscribe
const char *sistemKontrolModeTopic  = "petra/iot/krpl/serpis/kontrol/mode";
const char *mediaTanahKelembapanTanah2Topic = "petra/iot/krpl/serpis/monitor/mediatanah/kelembapan_tanah2";
const char *mediaTanahKelembapanTanah3Topic = "petra/iot/krpl/serpis/monitor/mediatanah/kelembapan_tanah3";
const char *mediaTanahKelembapanTanah4Topic = "petra/iot/krpl/serpis/monitor/mediatanah/kelembapan_tanah4";
const char *mediaTanahSprayerKontrolTopic = "petra/iot/krpl/serpis/kontrol/mediatanah/sprayer_mediatanah";
const char *mediaTanahSprayerOnThresholdValueTopic = "petra/iot/krpl/serpis/kontrol/mediatanah/sprayer_mediatanah/on_thres_value";
const char *mediaTanahSprayerOffThresholdValueTopic = "petra/iot/krpl/serpis/kontrol/mediatanah/sprayer_mediatanah/off_thres_value";
const char *mediaTanahSoilMoistureDryCalibrationTopic = "petra/iot/krpl/serpis/kontrol/mediatanah/kelembapan_tanah1/dry_cal";
const char *mediaTanahSoilMoistureWetCalibrationTopic = "petra/iot/krpl/serpis/kontrol/mediatanah/kelembapan_tanah/wet_cal";

//WiFi
const char *ssid = "KRPLSERPIS";  
const char *password = "krplserp1s"; 

//MQTT Broker
const char *mqtt_broker = "203.189.123.207";
const char *mqtt_username = "";
const char *mqtt_password = "";
const int mqtt_port = 1883;

//Pin
const byte relayPin = 27;
const byte soilSensorPin = 33;
const byte waterSensorPin = 13;//
const byte ledPowerPin = 4;//
const byte ledWifiPin = 18;
const byte ledMqttPin = 19;

//States
byte relayState = LOW;
byte waterSensorState_1 = LOW;
bool automaticMode = false;
bool manualMode = false;
bool onThresholdReceived = false;
bool offThresholdReceived = false;
char *waterSensorStatus = "Habis";

//Threshold & Calibration
int soilDryCal = 1900;
int soilWetCal = 1090;
const int soilMaxReading = soilDryCal;
const int soilMinReading = soilWetCal;
int sprayerOnThreshold = 0;
int sprayerOffThreshold = 0;

//Sensor Variable
int soilMoistureRealValue = 0;
int soilSensorPercentage1 = 0;
int soilSensorPercentage2 = 0;
int soilSensorPercentage3 = 0;
int soilSensorPercentage4 = 0;
int soilAveragePercentage = 0;

//Intervals
unsigned long previousPublishMillis = 0;  
unsigned long previousReadSensorsMillis = 0;  
unsigned long previousMillis = 0;
unsigned long mqttPublishInterval = 20*1000;
unsigned long readWaterSensorsInterval = 2*1000;

void setup() {
  Serial.begin(9600); // open serial port, set the baud rate to 9600 bps
  int counterWifi = 0;

  pinMode(ledPowerPin, OUTPUT); digitalWrite(ledPowerPin, HIGH);
  pinMode(ledWifiPin, OUTPUT);
  pinMode(ledMqttPin, OUTPUT);
  pinMode(relayPin, OUTPUT); 
  pinMode(soilSensorPin, INPUT); 
  pinMode(waterSensorPin, INPUT_PULLDOWN);
  
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
  }
  counterWifi+=1;
    Serial.print("\n Connecting to WiFi again..."); Serial.print(counterWifi);
    Serial.print("\n ");
    digitalWrite(ledWifiPin, LOW);
    delay(1000);
  } 
  Serial.print("\n Still connected to WiFi ");
  client.publish(mediaTanah1HeartBeatTopic, "ON");
}

void maintainMqttConn(){
   if (!client.connected()){
    digitalWrite(ledMqttPin, LOW);
      mqttReconnect();
   } 
}

void mqttReconnect(){
  int counterMqtt = 0;
  while (!client.connected()) {
    if (counterMqtt==5){
      ESP.restart();
    }
  counterMqtt+=1;
  Serial.print("\n Attempting MQTT connection...");
  digitalWrite(ledMqttPin, LOW);
  if (client.connect(clientId)) {
      Serial.println("Mqtt Connected");
      client.subscribe(sistemKontrolModeTopic);
      client.subscribe(mediaTanahKelembapanTanah2Topic);
      client.subscribe(mediaTanahKelembapanTanah3Topic);
      client.subscribe(mediaTanahKelembapanTanah4Topic);
      client.subscribe(mediaTanahSprayerKontrolTopic);
      client.subscribe(mediaTanahSprayerOnThresholdValueTopic);
      client.subscribe(mediaTanahSprayerOffThresholdValueTopic);
      client.subscribe(mediaTanahSoilMoistureDryCalibrationTopic);
      client.subscribe(mediaTanahSoilMoistureWetCalibrationTopic);
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
  
  if (String(topic) == mediaTanahSprayerKontrolTopic){
    if (messageTemp == "ON"){
        relayState = HIGH;
    } else if (messageTemp == "OFF"){
        relayState = LOW;
    }
  }
  
  if (String(topic) == mediaTanahSprayerOnThresholdValueTopic){
    sprayerOnThreshold = messageTemp.toInt();
    onThresholdReceived = true;
  } 
  
  if (String(topic) == mediaTanahSprayerOffThresholdValueTopic){
    sprayerOffThreshold = messageTemp.toInt();
    offThresholdReceived = true;
  } 

  /*
  if (String(topic) == mediaTanahSoilMoistureDryCalibrationTopic){
    soilDryCal = messageTemp.toInt();
  }
  
  if (String(topic) == mediaTanahSoilMoistureWetCalibrationTopic){
    soilWetCal = messageTemp.toInt();
  }
  */
  
  if (String(topic) == mediaTanahKelembapanTanah2Topic){
    soilSensorPercentage2 = messageTemp.toInt();
  }

  if (String(topic) == mediaTanahKelembapanTanah3Topic){
    soilSensorPercentage3 = messageTemp.toInt();
  }
  
  if (String(topic) == mediaTanahKelembapanTanah4Topic){
    soilSensorPercentage4 = messageTemp.toInt();
  }
}

void loop(){
  client.loop();
  if (automaticMode == true && manualMode == false){
    automaticControl();
    readSoilSensors();
    readWaterSensors();
    waterSensorsMqttMsg();
    publishSensorsReadings();
    publishRelayStatus();
  } 
  
  if (automaticMode == false && manualMode == true){
    manualControl();
    readSoilSensors();
    readWaterSensors();
    waterSensorsMqttMsg();
    publishSensorsReadings();
    publishRelayStatus();
  }
    maintainMqttConn();
    maintainWifiConn();
  delay(2000);
}

void automaticControl(){
  Serial.print("\n\n ----Kontrol Automatis----");
  Serial.print("\n Sprayer On Threshold:"); Serial.print(sprayerOnThreshold);
  Serial.print("\n Sprayer Off Threshold:"); Serial.print(sprayerOffThreshold);
  if(onThresholdReceived == true && offThresholdReceived == true){
    if (soilAveragePercentage <= sprayerOnThreshold){
      digitalWrite(relayPin, HIGH);
    } else if (soilAveragePercentage >= sprayerOffThreshold){
      digitalWrite(relayPin, LOW);
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

void readSoilSensors(){
  soilMoistureRealValue = analogRead(soilSensorPin);
  soilSensorPercentage1 = map(constrain(soilMoistureRealValue, soilMinReading, soilMaxReading), soilDryCal, soilWetCal, 0, 100);
  soilAveragePercentage = (soilSensorPercentage1+soilSensorPercentage2+soilSensorPercentage3+soilSensorPercentage4)/4;
  
  Serial.print("\n Soil Real Value:"); Serial.print(soilMoistureRealValue);
  Serial.print("\n Soil 1 :"); Serial.print(soilSensorPercentage1); Serial.print("%");
  Serial.print("\n Soil 2 :"); Serial.print(soilSensorPercentage2); Serial.print("%");
  Serial.print("\n Soil 3 :"); Serial.print(soilSensorPercentage3); Serial.print("%");
  Serial.print("\n Soil 4 :"); Serial.print(soilSensorPercentage4); Serial.print("%");
  Serial.print("\n Soil Average :"); Serial.print(soilAveragePercentage); Serial.print("%");
}

void readWaterSensors(){
  if(millis() > previousReadSensorsMillis + readWaterSensorsInterval){
    previousReadSensorsMillis = millis();
    waterSensorState_1 = digitalRead(waterSensorPin);
  }
}

void waterSensorsMqttMsg(){
    //water 1
    if (waterSensorState_1 == LOW){
      waterSensorStatus = "Habis";
    } else {
      waterSensorStatus = "Penuh";
    }
    Serial.print("\n Gentong Sprayer:"); Serial.print(waterSensorStatus);
}
 
void publishSensorsReadings(){
  if(millis() > previousPublishMillis + mqttPublishInterval){
    previousPublishMillis = millis();
    client.publish(mediaTanahKelembapanTanah1Topic, String(soilSensorPercentage1).c_str(), true);
    client.publish(mediaTanahTandonTopic, String(waterSensorStatus).c_str(), true);
  }
}
 
void publishRelayStatus(){
  if(digitalRead(relayPin) == HIGH){
    client.publish(mediaTanahSprayerMonitorTopic,"ON", true);
    Serial.print("\n Relay status:"); Serial.print("ON");
  } else {
    client.publish(mediaTanahSprayerMonitorTopic,"OFF", true);
    Serial.print("\n Relay status:"); Serial.print("OFF");
  }
}
