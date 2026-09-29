/*
  SafeSwim — ESP32 firmware (drowning detection armband)

  Reads exactly 3 sensors:
    - MAX30102   (I2C)  -> pulse (HR) + SpO2
    - MPU6050    (I2C)  -> motion (accel/gyro variance)
    - Water sensor (1 GPIO, digital) -> submersion state

  Serves the SafeSwim front end (index.html) from LittleFS and pushes
  live telemetry / alarms to it over a WebSocket at ws://<esp32-ip>/ws,
  matching the JSON contract the front end already expects:

    -> {"type":"telemetry","hr":86,"spo2":98,"motion":"active","submerged":false}
    -> {"type":"alarm","hr":61,"spo2":91,"reason":"submerged_no_motion"}
    -> {"type":"cleared"}
    <- {"type":"ack"}      (user dismissed the alarm, "I'm OK")
    <- {"type":"notify"}   (user tapped "Notify contacts now" -> fires SIM800L SMS)

  Libraries required (Arduino IDE Library Manager):
    - ESP Async WebServer      (me-no-dev / ESP32Async fork)
    - AsyncTCP                 (me-no-dev / ESP32Async fork)
    - ArduinoJson               (bblanchon)
    - SparkFun MAX3010x Pulse and Proximity Sensor Library
    - Adafruit MPU6050  + Adafruit Unified Sensor + Adafruit BusIO

  Board setup:
    - Upload this sketch normally.
    - Upload index.html into a "/data" folder next to this .ino using the
      "ESP32 Sketch Data Upload" tool (or arduino-cli's littlefs plugin) so
      it lands in LittleFS at "/index.html".

  Wiring:
    MAX30102   SDA -> GPIO21, SCL -> GPIO22, VIN -> 3.3V, GND -> GND
    MPU6050    SDA -> GPIO21, SCL -> GPIO22 (shares the I2C bus), VCC -> 3.3V, GND -> GND
    Water sensor  signal -> GPIO34 (input-only ADC pin used as digital in),
                  wired with a pull-down so it reads LOW when dry, HIGH when
                  the two probe pads are bridged by water
    SIM800L    TX -> GPIO16 (ESP32 RX2), RX -> GPIO17 (ESP32 TX2, through a
               voltage divider), own regulated 4V supply with a bulk
               capacitor (100-1000uF) across VCC/GND -- do NOT power it
               from the ESP32's 3.3V rail
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MAX30105.h>
#include "spo2_algorithm.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------- User configuration ----------------
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const int PIN_WATER_SENSOR = 34;   // digital-in, input-only ADC pin is fine for this
const int PIN_SIM800_RX    = 16;   // ESP32 RX2 <- SIM800L TX
const int PIN_SIM800_TX    = 17;   // ESP32 TX2 -> SIM800L RX (via divider)

const char* EMERGENCY_CONTACT_NUMBERS[] = {
  "+8801XXXXXXXXX",   // primary emergency contact
  "+8801YYYYYYYYY"    // lifeguard station / secondary contact
};
const int NUM_EMERGENCY_CONTACTS = 2;

// Fusion-logic thresholds
const unsigned long NO_MOTION_ALARM_MS = 9000;   // submerged + still this long -> alarm
const float MOTION_VARIANCE_THRESHOLD  = 0.02f;  // accel-magnitude variance below this = "still"
const int   SPO2_ALARM_THRESHOLD       = 92;     // SpO2 below this while pre-alarm -> confirm alarm
const int   HR_LOW_ALARM_THRESHOLD     = 40;     // implausibly low / lost pulse
const int   HR_HIGH_ALARM_THRESHOLD    = 180;    // implausibly high / lost pulse

const unsigned long TELEMETRY_INTERVAL_MS = 1500;
const unsigned long MOTION_SAMPLE_INTERVAL_MS = 50;

// ---------------- Globals ----------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

MAX30105 particleSensor;
Adafruit_MPU6050 mpu;
HardwareSerial sim800(2);

// MAX30102 buffers (Maxim reference SpO2/HR algorithm expects 100 samples)
#define BUFFER_SIZE 100
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int32_t spo2Value = 0;
int8_t  spo2Valid = 0;
int32_t heartRateValue = 0;
int8_t  heartRateValid = 0;

// Motion variance (rolling window over accel magnitude)
#define MOTION_WINDOW 20
float motionWindow[MOTION_WINDOW];
int motionWindowIndex = 0;
bool motionWindowFilled = false;
bool motionActive = true; // assume active until proven still

// State
bool submerged = false;
bool inPreAlarm = false;
bool inAlarm = false;
unsigned long stillSinceSubmergedStart = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastMotionSampleMs = 0;

// ---------------- Helpers: JSON over WebSocket ----------------
void sendTelemetry() {
  StaticJsonDocument<192> doc;
  doc["type"] = "telemetry";
  doc["hr"] = heartRateValid ? heartRateValue : 0;
  doc["spo2"] = spo2Valid ? spo2Value : 0;
  doc["motion"] = motionActive ? "active" : "still";
  doc["submerged"] = submerged;
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

void sendAlarm(const char* reason) {
  StaticJsonDocument<192> doc;
  doc["type"] = "alarm";
  doc["hr"] = heartRateValid ? heartRateValue : 0;
  doc["spo2"] = spo2Valid ? spo2Value : 0;
  doc["reason"] = reason;
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

void sendCleared() {
  StaticJsonDocument<64> doc;
  doc["type"] = "cleared";
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

// ---------------- Sensor: MAX30102 (pulse + SpO2) ----------------
void initMax30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found -- check wiring");
    return;
  }
  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;       // red + IR
  int sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

// Fills the 100-sample buffer and runs Maxim's reference algorithm.
// Blocking for ~1s at 100Hz sampleRate -- called periodically from loop().
void updatePulseAndSpo2() {
  for (int i = 0; i < BUFFER_SIZE; i++) {
    while (!particleSensor.available()) {
      particleSensor.check();
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_SIZE, redBuffer,
    &spo2Value, &spo2Valid, &heartRateValue, &heartRateValid
  );
}

// ---------------- Sensor: MPU6050 (motion) ----------------
void initMpu6050() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found -- check wiring");
    return;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void sampleMotion() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float magnitude = sqrt(a.acceleration.x * a.acceleration.x +
                          a.acceleration.y * a.acceleration.y +
                          a.acceleration.z * a.acceleration.z);
  // Subtract gravity (~9.8 m/s^2) so a still arm reads ~0
  float delta = magnitude - 9.8f;

  motionWindow[motionWindowIndex] = delta;
  motionWindowIndex = (motionWindowIndex + 1) % MOTION_WINDOW;
  if (motionWindowIndex == 0) motionWindowFilled = true;

  if (motionWindowFilled) {
    float mean = 0;
    for (int i = 0; i < MOTION_WINDOW; i++) mean += motionWindow[i];
    mean /= MOTION_WINDOW;

    float variance = 0;
    for (int i = 0; i < MOTION_WINDOW; i++) {
      float diff = motionWindow[i] - mean;
      variance += diff * diff;
    }
    variance /= MOTION_WINDOW;

    motionActive = variance > MOTION_VARIANCE_THRESHOLD;
  }
}

// ---------------- Sensor: water immersion ----------------
void sampleWater() {
  submerged = digitalRead(PIN_WATER_SENSOR) == HIGH;
}

// ---------------- SIM800L: send SMS on emergency ----------------
void sendSmsAll(const String& message) {
  for (int i = 0; i < NUM_EMERGENCY_CONTACTS; i++) {
    sim800.print("AT+CMGF=1\r");
    delay(200);
    sim800.print("AT+CMGS=\"");
    sim800.print(EMERGENCY_CONTACT_NUMBERS[i]);
    sim800.print("\"\r");
    delay(200);
    sim800.print(message);
    sim800.write(26); // Ctrl+Z sends the message
    delay(2000);
  }
}

// ---------------- Fusion logic ----------------
// Combines submersion + stillness + vitals so a single sensor glitch
// never triggers a false alarm on its own.
void runFusionLogic() {
  unsigned long now = millis();

  if (submerged && !motionActive) {
    if (stillSinceSubmergedStart == 0) {
      stillSinceSubmergedStart = now;
    }
    unsigned long stillDuration = now - stillSinceSubmergedStart;

    if (stillDuration >= NO_MOTION_ALARM_MS && !inPreAlarm && !inAlarm) {
      inPreAlarm = true; // local buzzer stage -- front end can also show this
    }

    bool vitalsBad = spo2Valid && spo2Value < SPO2_ALARM_THRESHOLD;
    vitalsBad |= heartRateValid && (heartRateValue < HR_LOW_ALARM_THRESHOLD ||
                                     heartRateValue > HR_HIGH_ALARM_THRESHOLD);

    if (inPreAlarm && vitalsBad && !inAlarm) {
      inAlarm = true;
      sendAlarm("submerged_no_motion");
    }
  } else {
    // Motion resumed or surfaced -- clear state
    stillSinceSubmergedStart = 0;
    inPreAlarm = false;
    if (inAlarm) {
      inAlarm = false;
      sendCleared();
    }
  }
}

// ---------------- WebSocket events ----------------
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, data, len);
    if (err) return;

    const char* msgType = doc["type"];
    if (msgType == nullptr) return;

    if (strcmp(msgType, "ack") == 0) {
      Serial.println("User dismissed alarm (I'm OK)");
      inAlarm = false;
      inPreAlarm = false;
      stillSinceSubmergedStart = 0;
    } else if (strcmp(msgType, "notify") == 0) {
      Serial.println("Notify contacts requested -> sending SMS via SIM800L");
      sendSmsAll("SafeSwim alert: possible drowning detected. HR " +
                 String(heartRateValue) + " SpO2 " + String(spo2Value) + "%.");
    }
  }
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL shared by MAX30102 + MPU6050

  pinMode(PIN_WATER_SENSOR, INPUT);

  initMax30102();
  initMpu6050();

  sim800.begin(9600, SERIAL_8N1, PIN_SIM800_RX, PIN_SIM800_TX);
  delay(2000);
  sim800.print("AT\r"); // wake/check modem

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.begin();
}

void loop() {
  unsigned long now = millis();

  sampleWater();

  if (now - lastMotionSampleMs >= MOTION_SAMPLE_INTERVAL_MS) {
    lastMotionSampleMs = now;
    sampleMotion();
  }

  // Recomputing pulse/SpO2 fills a 100-sample buffer (~1s at 100Hz) --
  // run it back-to-back so the values stay reasonably fresh.
  updatePulseAndSpo2();

  runFusionLogic();

  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    if (!inAlarm) {
      sendTelemetry();
    }
  }

  ws.cleanupClients();
}
