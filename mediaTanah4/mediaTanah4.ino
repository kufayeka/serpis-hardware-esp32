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
const char *clientId = "petra/iot/krpl/serpis/esp32MediaTanah4";
const char *mediaTanah4HeartBeatTopic = "petra/iot/krpl/serpis/monitor/mediatanah4/heartbeat";
const char *mediaTanahKelembapanTanah4Topic = "petra/iot/krpl/serpis/monitor/mediatanah/kelembapan_tanah4";

//WiFi
const char *ssid = "KRPLSERPIS";  
const char *password = "krplserp1s"; 

//MQTT Broker
const char *mqtt_broker = "203.189.123.207";
const char *mqtt_username = "";
const char *mqtt_password = "";
const int mqtt_port = 1883;

//Pin
const byte soilSensorPin = 33;
const byte ledPowerPin = 4;
const byte ledWifiPin = 18;
const byte ledMqttPin = 19;

//Threshold & Calibration
int soilDryCal = 1900;
int soilWetCal = 1090;
const int soilMaxReading = soilDryCal;
const int soilMinReading = soilWetCal;

//Sensor Variable
int soilMoistureRealValue = 0;
int soilSensorPercentage4 = 0;

//Intervals
unsigned long previousPublishMillis = 0;  
unsigned long mqttPublishInterval = 20*1000;

void setup(){
  Serial.begin(9600); // open serial port, set the baud rate to 9600 bps
  int counterWifi = 0;

  pinMode(ledPowerPin, OUTPUT); digitalWrite(ledPowerPin, HIGH);
  pinMode(ledWifiPin, OUTPUT);
  pinMode(ledMqttPin, OUTPUT);
  pinMode(soilSensorPin, INPUT);
  
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
  client.publish(mediaTanah4HeartBeatTopic, "ON");
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
}

void loop(){
  client.loop();
    readSoilSensors();
    publishSensorsReadings();
    maintainMqttConn();
    maintainWifiConn();
  delay(2000);
}

void readSoilSensors(){
  soilMoistureRealValue = analogRead(soilSensorPin);
  soilSensorPercentage4 = map(constrain(soilMoistureRealValue, soilMinReading, soilMaxReading), soilDryCal, soilWetCal, 0, 100);
  
  Serial.print("\n Soil Real Value:"); Serial.print(soilMoistureRealValue);
  Serial.print("\n Soil 4 :"); Serial.print(soilSensorPercentage4); Serial.print("%");
}

void publishSensorsReadings(){
  if(millis() > previousPublishMillis + mqttPublishInterval){
    previousPublishMillis = millis();
    client.publish(mediaTanahKelembapanTanah4Topic, String(soilSensorPercentage4).c_str(), true);
  }
}
 
