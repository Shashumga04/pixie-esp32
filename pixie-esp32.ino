#include <Arduino.h>   // Required entry point for PlatformIO Arduino framework
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "ThingSpeak.h"  // Official MathWorks ThingSpeak Library
#include <LD2450.hpp>

using namespace esphome::ld245x;

// ================= CREDENTIALS CONFIGURATION =================
const char* ssid = "smarthome_"; 
const char* password = "11121968";

unsigned long myChannelNumber = 3403474;         
const char* myWriteAPIKey = "K3JTOQ83W9QLU8RA";   

WiFiClient client;  

// ================= PERIPHERAL SETTINGS =================
// BLE
BLECharacteristic *sensorCharacteristic;
bool bleConnected = false;

#define SERVICE_UUID        "12345678"
#define CHARACTERISTIC_UUID "abcd1234"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    Serial.println("BLE connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    Serial.println("BLE disconnected");
    BLEDevice::startAdvertising(); 
  }
};

// SHT40
#define SDA_PIN 8
#define SCL_PIN 9
Adafruit_SHT4x sht4;
bool shtFound = false;

// HC-SR04P
#define TRIG_PIN 4
#define ECHO_PIN 5

// LD2450
#define LD2450_RX 17
#define LD2450_TX 18

HardwareSerial ld2450Serial(2);
LD2450 ld2450;

// MOTOR PINS
#define LEFT_IN1   10
#define RIGHT_IN1  12
#define LEFT_IN2   11
#define RIGHT_IN2  13
#define ENABLE     14
        
#define PWM_FREQ 20000
#define PWM_RES 8

// DISTANCE THRESHOLDS & MOTOR SPEED
const float targetDistance = 80.0;
const float deadband = 5.0; // Maintain target within +/- 5cm
const int motorSpeed = 120; // Fixed cruising speed (Value between 0-255)

unsigned long lastControlTime = 0;
unsigned long lastSend = 0;

const unsigned long controlInterval = 100;
const unsigned long sendInterval = 16000; 

// ================= FUNCTION PROTOTYPES =================
void setupMotors();
void stopMotors();
void moveForward(int speedValue);
void moveBackward(int speedValue);
void setupWiFi();
void setupBLE();
void robotControl();
float readUltrasonic();
void readLD2450Distance();

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ESP32-S3 Robot Full System Started (No PID)");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (sht4.begin()) {
    shtFound = true;
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    Serial.println("SHT40 found");
  } else {
    Serial.println("SHT40 not found");
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

// ld2450 start
  ld2450Serial.begin(
      LD2450_SERIAL_SPEED,
      SERIAL_8N1,
      LD2450_RX,
      LD2450_TX
  );

  ld2450Serial.setTimeout(1000);

  ld2450.begin(ld2450Serial);

  Serial.println("LD2450 initializing...");

  ld2450.beginConfigurationSession();
  ld2450.setMultiTargetTracking();
  ld2450.queryTargetTrackingMode();
  ld2450.queryFirmwareVersion();
  ld2450.queryMacAddress();
  ld2450.queryZoneFilter();
  ld2450.endConfigurationSession();


  Serial.print("Firmware: ");
  Serial.println(ld2450.getFirmwareString());

  Serial.print("MAC: ");
  Serial.println(ld2450.getMacAddressString());

  Serial.println("LD2450 ready");
  
// ld2450 end

  setupMotors();
  stopMotors();

  setupWiFi();
  ThingSpeak.begin(client);
  setupBLE();

  Serial.println("System ready");
}

// ================= MAIN LOOP =================
void loop() {
  // 1. FAST LOOP: High-speed distance checking and motor controls (Every 100ms)
  if (millis() - lastControlTime >= controlInterval) {
    lastControlTime = millis();
    robotControl();
  }

  // 2. SLOW LOOP: Non-blocking cloud updates (Every 16 seconds)
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\n[WARNING]: WiFi lost! Reconnecting in background...");
      WiFi.begin(ssid, password); 
    }

    float temperature = 0;
    float humidity = 0;
    float ultrasonic = readUltrasonic();

    if (shtFound) {
      sensors_event_t humEvent;
      sensors_event_t tempEvent;
      sht4.getEvent(&humEvent, &tempEvent);
      temperature = tempEvent.temperature;
      humidity = humEvent.relative_humidity;
    }

    Serial.println("\n--- Sending Multi-Field Stream to ThingSpeak ---");
    Serial.print("Temp: "); Serial.print(temperature);
    Serial.print(" | Hum: "); Serial.print(humidity);
    Serial.print(" | Dist: "); Serial.println(ultrasonic);

    ThingSpeak.setField(1, temperature);
    ThingSpeak.setField(2, humidity);
    ThingSpeak.setField(3, ultrasonic);

    if (WiFi.status() == WL_CONNECTED) {
      int responseCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      if (responseCode == 200) {
        Serial.println("ThingSpeak Upload Successful! (HTTP 200)");
      } else {
        Serial.print("ThingSpeak Upload Failed! Code: ");
        Serial.println(responseCode);
      }
    } else {
      Serial.println("ThingSpeak Skipped: Offline.");
    }

    if (bleConnected) {
      String blePayload = "field1=" + String(temperature) + "&field2=" + String(humidity) + "&field3=" + String(ultrasonic);
      sensorCharacteristic->setValue(blePayload.c_str());
      sensorCharacteristic->notify();
    }
  }
}

// ================= ROBOT CONTROL =================
void robotControl() {
  float distanceCM = readUltrasonic();


  // If timeout occurred or no object detected
  if (distanceCM <= 0) {
    stopMotors();
    return;
  }

  // Safety emergency cushion stop
  if (distanceCM < 25.0) {
    stopMotors();
    Serial.println("Distance: Emergency stop! Too close.");
    return;
  }
  
  readLD2450Distance();
  Serial.println("Distance: ");
  Serial.print(distanceCM);
  Serial.print(" cm | Action: ");

  // Direct Threshold Control Engine
  if (distanceCM > (targetDistance + deadband)) {
    Serial.println("Moving Forward");
    moveForward(motorSpeed);
  } else if (distanceCM < (targetDistance - deadband)) {
    Serial.println("Moving Backward");
    moveBackward(motorSpeed);
  } else {
    Serial.println("Holding Position");
    stopMotors();
  }
}

// ================= HC-SR04P =================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 15000us timeout restricts calculations to a practical tracking zone (~250cm max)
  long duration = pulseIn(ECHO_PIN, HIGH, 15000);

  if (duration == 0) {
    return -1.0;
  }

  return (duration * 0.0343) / 2.0;
}

// ================= LD2450 =================
void readLD2450Distance()
{
    if (!ld2450.update())
        return;

    int targetCount = ld2450.getNrValidTargets();

    while (targetCount > 0)
    {
        targetCount--;

        auto target = ld2450.getTarget(targetCount);
        Serial.println(target.format().c_str());
    }
}

// ================= MOTOR SETUP =================
void setupMotors() {
  pinMode(ENABLE, OUTPUT); 
  
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(LEFT_IN1, PWM_FREQ, PWM_RES);
    ledcAttach(LEFT_IN2, PWM_FREQ, PWM_RES);
    ledcAttach(RIGHT_IN1, PWM_FREQ, PWM_RES);
    ledcAttach(RIGHT_IN2, PWM_FREQ, PWM_RES);
  #else
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcSetup(1, PWM_FREQ, PWM_RES);
    ledcSetup(2, PWM_FREQ, PWM_RES);
    ledcSetup(3, PWM_FREQ, PWM_RES);
    
    ledcAttachPin(LEFT_IN1, 0);
    ledcAttachPin(LEFT_IN2, 1);
    ledcAttachPin(RIGHT_IN1, 2);
    ledcAttachPin(RIGHT_IN2, 3);
  #endif

  digitalWrite(ENABLE, HIGH);
}

// ================= MOTOR FUNCTIONS =================
void moveForward(int speedValue) {
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(LEFT_IN1, speedValue);
    ledcWrite(LEFT_IN2, 0);
    ledcWrite(RIGHT_IN1, speedValue);
    ledcWrite(RIGHT_IN2, 0);
  #else
    ledcWrite(0, speedValue);
    ledcWrite(1, 0);
    ledcWrite(2, speedValue);
    ledcWrite(3, 0);
  #endif
}

void moveBackward(int speedValue) {
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(LEFT_IN1, 0);
    ledcWrite(LEFT_IN2, speedValue);
    ledcWrite(RIGHT_IN1, 0);
    ledcWrite(RIGHT_IN2, speedValue);
  #else
    ledcWrite(0, 0);
    ledcWrite(1, speedValue);
    ledcWrite(2, 0);
    ledcWrite(3, speedValue);
  #endif
}

void stopMotors() {
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(LEFT_IN1, 0);
    ledcWrite(LEFT_IN2, 0);
    ledcWrite(RIGHT_IN1, 0);
    ledcWrite(RIGHT_IN2, 0);
  #else
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
  #endif
}

// ================= WIFI WITH DEEP RESET =================
void setupWiFi() {
  Serial.println("Initializing Wi-Fi Radio Isolation Layer...");
  WiFi.disconnect(true, true); 
  delay(1000); 

  WiFi.mode(WIFI_STA);
  delay(500);

  Serial.print("Connecting to WiFi Network: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected successfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi Handshake Terminated. Booting in local offline/BLE mode.");
  }
}

// ================= BLE =================
void setupBLE() {
  BLEDevice::init("ESP32S3_Robot");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);

  sensorCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  sensorCharacteristic->addDescriptor(new BLE2902());
  sensorCharacteristic->setValue("Waiting for data...");
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();

  Serial.println("BLE System Armed. Scan for 'ESP32S3_Robot'");
}