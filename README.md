# SafeSwim — Drowning Detection Armband

Everything needed to run the app: the ESP32 firmware and the phone/browser
front end it serves.

## What's in this folder

```
SafeSwim/
├── index.html                              <- front end, standalone copy
└── ESP32_Firmware/
    └── SafeSwim_ESP32/
        ├── SafeSwim_ESP32.ino              <- ESP32 sketch (Arduino IDE)
        └── data/
            └── index.html                  <- same front end, for flashing to the ESP32
```

## Option A — try the front end right now (no hardware)

Open `index.html` in any browser and tap **"Skip & use demo data"**. It
simulates live pulse/SpO2 and lets you fire a test alarm with the
**"Simulate alarm (demo)"** button on the dashboard, walking through the
full emergency -> notify-contacts flow.

## Option B — run it for real on the ESP32

### 1. Hardware
- ESP32 dev board
- MAX30102 (pulse + SpO2) — I2C: SDA -> GPIO21, SCL -> GPIO22
- MPU6050 (motion) — same I2C bus (SDA -> GPIO21, SCL -> GPIO22)
- Water immersion sensor (2-electrode) — signal -> GPIO34, wired with a
  pull-down so it reads LOW dry / HIGH when the probes are bridged by water
- SIM800L (GSM/SMS) — TX -> GPIO16, RX -> GPIO17 (through a voltage
  divider), powered from its own regulated ~4V supply with a bulk
  capacitor (100–1000 µF) across VCC/GND — **do not power it from the
  ESP32's 3.3V rail**, it will brown out on transmit

### 2. Arduino IDE setup
Install these libraries via Library Manager:
- ESP Async WebServer
- AsyncTCP
- ArduinoJson
- SparkFun MAX3010x Pulse and Proximity Sensor Library
- Adafruit MPU6050 (pulls in Adafruit Unified Sensor + Adafruit BusIO)

Install the **ESP32 Sketch Data Upload** tool (Arduino IDE plugin) if you
don't already have it — this is what flashes the `data/` folder into the
ESP32's LittleFS filesystem so the board can serve the front end itself.

### 3. Configure the sketch
Open `ESP32_Firmware/SafeSwim_ESP32/SafeSwim_ESP32.ino` and edit near the
top:
```cpp
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* EMERGENCY_CONTACT_NUMBERS[] = {
  "+8801XXXXXXXXX",
  "+8801YYYYYYYYY"
};
```

### 4. Flash it
1. Open the sketch folder (`SafeSwim_ESP32`) in Arduino IDE — the folder
   name must match the `.ino` filename, which it already does here.
2. Select your ESP32 board + correct COM port.
3. Run **Tools -> ESP32 Sketch Data Upload** first (flashes `data/index.html`
   into LittleFS).
4. Then click **Upload** as normal to flash the sketch itself.
5. Open the Serial Monitor at 115200 baud — after it connects to WiFi it
   prints the board's IP address.

### 5. Connect the front end to the board
- Open a browser to `http://<esp32-ip>/` — the ESP32 serves the front end
  itself, already wired to the WebSocket, or
- Open the standalone `index.html` from this folder, type the ESP32's IP
  into the home screen, and tap **"Connect device"**.

Either way it opens `ws://<esp32-ip>/ws` and starts receiving live
telemetry, with the alarm and notify-contacts flow driving the real
SIM800L SMS.

## How the pieces talk to each other

The ESP32 fuses all three sensors (submerged + no motion for ~9s +
abnormal SpO2/pulse) before raising an alarm, so a single sensor glitch
never falsely triggers it. Full logic is commented in the `.ino` file.

```
{"type":"telemetry","hr":86,"spo2":98,"motion":"active","submerged":false}   ESP32 -> app, live vitals
{"type":"alarm","hr":61,"spo2":91,"reason":"submerged_no_motion"}            ESP32 -> app, alarm fires
{"type":"cleared"}                                                          ESP32 -> app, condition cleared
{"type":"ack"}                                                              app -> ESP32, user dismissed alarm
{"type":"notify"}                                                           app -> ESP32, triggers SIM800L SMS
```
