#include <Arduino.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// def MCU sensor serial interface
const byte rxPin = 3;
const byte txPin = 1;

// def Network credentials
const char* ssid = "wifi ni kace";
const char* password = "Gabbypass123!";
const char* mqtt_server = "192.168.254.108";

SoftwareSerial sensorSerial(rxPin, txPin);  // RX, TX
WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;

#define ledPin 2
// Define the number of bytes in a data packet
#define DATA_PACKET_SIZE 16

// def time interval per send
#define CO2_BUFFER_SIZE  1
unsigned long intervalPrintTime = CO2_BUFFER_SIZE * 1000UL;
uint16_t co2Values[CO2_BUFFER_SIZE];
int co2Index = 0;
int validMeasurements = 0;
unsigned long lastPrintTime = 0;

// def wifi connection
void setup_wifi() {
  delay(50);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int c=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    c++;
    if(c > 10){
        ESP.restart();
    }
  }

  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  return;
}

// def connection to mqtt server 
void connect_mqttServer() {
  while (!client.connected()) { // while client is not connected to mqtt
    if(WiFi.status() != WL_CONNECTED){  // if wifi is not connected
      setup_wifi();
    }
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32_client1")) { 
      Serial.println("connected");
      client.subscribe("rpi/broadcast");         
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state()); // print the connection status
      Serial.println(" trying again in 2 seconds");
      delay(2000);
    }
  }
  return; 
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTmp;  // temporary variable to hold the message
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTmp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "rpi/broadcast") {
    if(messageTmp == "10"){
      Serial.println("Action: blink LED");
    }
  }
}

void setup() {
  sensorSerial.begin(9600);
  Serial.begin(9600); 
  pinMode(ledPin, OUTPUT);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void printAverage() {
  uint16_t sum = 0;
  for (int i = 0; i < validMeasurements; i++) {
    sum += co2Values[i];
  }
  uint16_t average = static_cast<uint16_t>(sum / validMeasurements);
  
  if (!client.connected()) {
    connect_mqttServer();
  }
  JsonDocument JSONbuffer;
  JsonObject JSONencoder = JSONbuffer.to<JsonObject>();
  JSONencoder["device"] = "ESP32";
  JSONencoder["sensorType"] = "Temperature";
  JsonArray values = JSONencoder.createNestedArray("values");
 
  values.add(average);
 
  char JSONmessageBuffer[100];
  serializeJson(JSONencoder, JSONmessageBuffer, sizeof(JSONmessageBuffer));
  client.publish("esp32/sensor1", JSONmessageBuffer); 
  client.loop();
 
  delay(1000);
  Serial.print("Average CO2 Concentration: ");
  Serial.println(average);
}

void loop() {
  if (sensorSerial.available() >= DATA_PACKET_SIZE) {
    uint8_t dataPacket[DATA_PACKET_SIZE];
    sensorSerial.readBytes(dataPacket, DATA_PACKET_SIZE);

    if (dataPacket[0] == 0x42 && dataPacket[1] == 0x4D) {
      uint16_t co2Concentration = (dataPacket[6] << 8) | dataPacket[7];
      uint8_t checksum = 0;
      for (int i = 0; i < DATA_PACKET_SIZE - 1; i++) {
        checksum += dataPacket[i];
      }

      if (checksum == dataPacket[DATA_PACKET_SIZE - 1]) {
        co2Values[co2Index] = co2Concentration;
        co2Index = (co2Index + 1) % CO2_BUFFER_SIZE;
        validMeasurements = min(validMeasurements + 1, CO2_BUFFER_SIZE);

        if ((millis() - lastPrintTime >= intervalPrintTime) || (millis() < lastPrintTime)) {
          printAverage();
          lastPrintTime = millis();
        }
      } else {
        Serial.println("Checksum error!");
      }
    } else {
      Serial.println("Invalid data packet header!");
    }
  }
}