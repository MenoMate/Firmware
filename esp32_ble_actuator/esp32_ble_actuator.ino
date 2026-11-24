/*
  MenoMate Prototype — ESP32-C3 BLE GATT Actuator Controller
  
  Controls flexible polyimide heating film and cylindrical brushed ERM motor
  via two electromechanical relay modules using Bluetooth Low Energy (BLE).
  
  Hardware:
    ESP32-C3 SuperMini (HW-466AB)
    Relay modules: Active-LOW (OUTPUT_OPEN_DRAIN trick)
      Heater relay: IN -> GPIO 3
      Motor relay : IN -> GPIO 4
      GPIO 5      : Unused (kept released HIGH)
    VCC -> 5V pin, GND -> GND
    Load wiring (per relay): 5V -> COM ; NO -> load ; load other wire -> GND

  BLE GATT Specification:
    Device Name           : MenoMate-Wearable
    Service UUID          : 00000001-e099-4d24-a5e9-1152a2818987
    Command Characteristic: 00000002-e099-4d24-a5e9-1152a2818987 (Write / Write without response)
    Status Characteristic : 00000003-e099-4d24-a5e9-1152a2818987 (Read, Notify)
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_system.h>

// ---------- Hardware Pins ----------
const int HEAT_PIN   = 3; // Heater relay IN (active-LOW)
const int MOTOR_PIN  = 4; // Motor relay IN (active-LOW)
const int UNUSED_PIN = 5; // Unused (kept released HIGH)

// ---------- BLE UUIDs ----------
#define SERVICE_UUID        "00000001-e099-4d24-a5e9-1152a2818987"
#define CHARACTERISTIC_CMD  "00000002-e099-4d24-a5e9-1152a2818987"
#define CHARACTERISTIC_STAT "00000003-e099-4d24-a5e9-1152a2818987"
#define DEVICE_NAME         "MenoMate-Wearable"

// ---------- Heater Levels (Time-Proportioning Relay Control) ----------
// Mechanical relays switch slowly over a 20-second period (NOT PWM, NOT temperature regulation)
const unsigned long HEAT_PERIOD_MS = 20000;
const int HEAT_ON_PCT[4] = {0, 20, 35, 50}; // level 0 (OFF), 1 (LOW: 20%), 2 (MED: 35%), 3 (HIGH: 50%)

// ---------- Motor Modes ----------
// Relay-safe patterns:
//   0: OFF
//   1: STEADY (continuous ON)
//   2: WAVE   (3000 ms ON / 2000 ms OFF)
// Rapid pulse/tap modes (400ms) are disabled to protect mechanical relay contacts.
bool motorPattern(int mode, unsigned long t) {
  switch (mode) {
    case 1: return true;                         // STEADY
    case 2: return (t % 5000) < 3000;            // WAVE: 3s on / 2s off
    default: return false;
  }
}

// ---------- Auto-off Safety (10 minutes) ----------
const unsigned long MAX_ON_MS = 10UL * 60UL * 1000UL;

// ---------- Actuator & System State ----------
bool masterOn = false;
bool heatRelayOn = false;
bool motorRelayOn = false;
unsigned long onSince = 0;
int heatLevel = 0;   // 0..3
int motorMode = 0;   // 0=off, 1=steady, 2=wave
unsigned long heatPhaseStart = 0;
unsigned long motorPhaseStart = 0;

// ---------- BLE State & Buffers ----------
BLEServer* pServer = nullptr;
BLECharacteristic* pCmdCharacteristic = nullptr;
BLECharacteristic* pStatusCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Command decoupling buffer (BLE callback -> main loop)
volatile bool hasPendingCommand = false;
char pendingCommandBuf[256];
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

// Tracking state for notifying on change
bool lastNotifiedHeatActive = false;
bool lastNotifiedMotorActive = false;

// ---------- Hardware Control ----------
void relayWrite(int pin, bool on) {
  // Active-LOW open-drain: LOW = relay energised, HIGH = relay de-energised
  digitalWrite(pin, on ? LOW : HIGH);
}

void allOff() {
  masterOn = false;
  heatLevel = 0;
  motorMode = 0;
  heatRelayOn = false;
  motorRelayOn = false;
  relayWrite(HEAT_PIN, false);
  relayWrite(MOTOR_PIN, false);
  Serial.println("[SAFETY] All actuators turned OFF");
}

// ---------- Status JSON Serialization ----------
String buildStatusJson() {
  String s = "{\"connected\":";
  s += (deviceConnected ? "true" : "false");
  s += ",\"on\":";
  s += (masterOn ? "true" : "false");
  s += ",\"heat\":";
  s += heatLevel;
  s += ",\"motor\":";
  if (motorMode == 1) s += "\"steady\"";
  else if (motorMode == 2) s += "\"wave\"";
  else s += "\"off\"";
  s += ",\"heater_active\":";
  s += (heatRelayOn ? "true" : "false");
  s += ",\"motor_active\":";
  s += (motorRelayOn ? "true" : "false");
  s += "}";
  return s;
}

void pushStatusNotification() {
  if (pStatusCharacteristic != nullptr) {
    String status = buildStatusJson();
    pStatusCharacteristic->setValue(status.c_str());
    if (deviceConnected) {
      pStatusCharacteristic->notify();
    }
  }
}

// ---------- BLE Server Callbacks ----------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("[BLE] Client connected");
    // Per requirement: Reconnection must never restore previous actuator states.
    // Relays remain off until an explicit command is received.
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("[BLE] Client disconnected");
    // Per requirement: Outputs OFF when a BLE client disconnects.
    allOff();
  }
};

// ---------- BLE Characteristic Callbacks ----------
class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0 && rxValue.length() < sizeof(pendingCommandBuf)) {
      portENTER_CRITICAL(&cmdMux);
      memcpy((void*)pendingCommandBuf, rxValue.c_str(), rxValue.length());
      pendingCommandBuf[rxValue.length()] = '\0';
      hasPendingCommand = true;
      portEXIT_CRITICAL(&cmdMux);
    }
  }
};

// ---------- Command Parser (Runs safely in loop context) ----------
void processPendingCommand(const char* raw) {
  String json = String(raw);
  json.trim();
  Serial.print("[CMD RX] ");
  Serial.println(json);

  // 1. Emergency / Master ALL OFF: {"cmd":"all_off"}
  if (json.indexOf("\"cmd\":\"all_off\"") >= 0 || json.indexOf("\"all_off\"") >= 0) {
    allOff();
    pushStatusNotification();
    return;
  }

  // 2. Set Power: {"cmd":"set_power","on":true/false}
  if (json.indexOf("\"cmd\":\"set_power\"") >= 0) {
    if (json.indexOf("\"on\":true") >= 0) {
      if (!masterOn) {
        masterOn = true;
        onSince = millis();
        heatPhaseStart = millis();
        motorPhaseStart = millis();
        Serial.println("[CMD] Power ON");
      }
    } else if (json.indexOf("\"on\":false") >= 0) {
      allOff();
      Serial.println("[CMD] Power OFF");
    }
    pushStatusNotification();
    return;
  }

  // 3. Set Heater Level: {"cmd":"set_heat","level":0..3}
  if (json.indexOf("\"cmd\":\"set_heat\"") >= 0) {
    if (!masterOn) {
      Serial.println("[CMD REJECT] Cannot set heat while power is OFF");
      pushStatusNotification();
      return;
    }
    int lvlIdx = json.indexOf("\"level\":");
    if (lvlIdx >= 0) {
      char lvlChar = json.charAt(lvlIdx + 8);
      int lvl = lvlChar - '0';
      if (lvl >= 0 && lvl <= 3) {
        heatLevel = lvl;
        heatPhaseStart = millis();
        Serial.print("[CMD] Heat level set to ");
        Serial.println(heatLevel);
      } else {
        Serial.println("[CMD REJECT] Invalid heat level (must be 0..3)");
      }
    }
    pushStatusNotification();
    return;
  }

  // 4. Set Motor Mode: {"cmd":"set_motor","mode":"off"|"steady"|"wave"}
  if (json.indexOf("\"cmd\":\"set_motor\"") >= 0) {
    if (!masterOn) {
      Serial.println("[CMD REJECT] Cannot set motor while power is OFF");
      pushStatusNotification();
      return;
    }
    if (json.indexOf("\"mode\":\"off\"") >= 0) {
      motorMode = 0;
      motorPhaseStart = millis();
      Serial.println("[CMD] Motor set to OFF");
    } else if (json.indexOf("\"mode\":\"steady\"") >= 0 || json.indexOf("\"mode\":\"smooth\"") >= 0) {
      motorMode = 1;
      motorPhaseStart = millis();
      Serial.println("[CMD] Motor set to STEADY");
    } else if (json.indexOf("\"mode\":\"wave\"") >= 0) {
      motorMode = 2;
      motorPhaseStart = millis();
      Serial.println("[CMD] Motor set to WAVE");
    } else {
      Serial.println("[CMD REJECT] Invalid motor mode (allowed: off, steady, wave)");
    }
    pushStatusNotification();
    return;
  }

  Serial.println("[CMD REJECT] Unrecognized JSON command");
}

// ---------- Arduino Setup ----------
void setup() {
  // CRITICAL: Configure open-drain first, then release (HIGH) so relays stay OFF at boot
  pinMode(HEAT_PIN,   OUTPUT_OPEN_DRAIN); digitalWrite(HEAT_PIN,   HIGH);
  pinMode(MOTOR_PIN,  OUTPUT_OPEN_DRAIN); digitalWrite(MOTOR_PIN,  HIGH);
  pinMode(UNUSED_PIN, OUTPUT_OPEN_DRAIN); digitalWrite(UNUSED_PIN, HIGH);

  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=========================================");
  Serial.println(" MenoMate ESP32-C3 BLE Actuator Firmware");
  Serial.println("=========================================");
  Serial.print("Reset reason: ");
  Serial.println((int)esp_reset_reason());

  // Initialize BLE Stack
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create GATT Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Command Characteristic: Write (with response) and Write Without Response
  pCmdCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_CMD,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCmdCharacteristic->setCallbacks(new CommandCallbacks());

  // Status Characteristic: Read and Notify
  pStatusCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_STAT,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pStatusCharacteristic->addDescriptor(new BLE2902());

  // Set initial status payload
  String initialStatus = buildStatusJson();
  pStatusCharacteristic->setValue(initialStatus.c_str());

  // Start Service
  pService->start();

  // Configure BLE Advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.print("[BLE] Advertising as '");
  Serial.print(DEVICE_NAME);
  Serial.println("'");
  Serial.print("[BLE] Service UUID: ");
  Serial.println(SERVICE_UUID);
}

// ---------- Arduino Main Loop ----------
void loop() {
  unsigned long now = millis();

  // 1. Check for incoming commands queued by the BLE write callback
  if (hasPendingCommand) {
    char localBuf[sizeof(pendingCommandBuf)];
    portENTER_CRITICAL(&cmdMux);
    memcpy(localBuf, (const void*)pendingCommandBuf, sizeof(localBuf));
    hasPendingCommand = false;
    portEXIT_CRITICAL(&cmdMux);

    processPendingCommand(localBuf);
  }

  // 2. Heater: time-proportioning duty-cycle for mechanical relay
  bool prevHeatRelayOn = heatRelayOn;
  heatRelayOn = false;
  if (masterOn && heatLevel > 0) {
    unsigned long t = (now - heatPhaseStart) % HEAT_PERIOD_MS;
    heatRelayOn = t < (HEAT_PERIOD_MS * HEAT_ON_PCT[heatLevel] / 100);
  }
  relayWrite(HEAT_PIN, heatRelayOn);

  // 3. Motor: relay-safe pattern
  bool prevMotorRelayOn = motorRelayOn;
  motorRelayOn = masterOn && motorMode > 0 && motorPattern(motorMode, now - motorPhaseStart);
  relayWrite(MOTOR_PIN, motorRelayOn);

  // 4. Auto-off safety timer: 10 minutes max continuous on-time
  if (MAX_ON_MS > 0 && masterOn && (now - onSince > MAX_ON_MS)) {
    Serial.println("[SAFETY] Auto-off: 10-minute maximum session reached");
    allOff();
    pushStatusNotification();
  }

  // 5. Notify Flutter if active relay states changed (e.g. duty cycle transition)
  if (deviceConnected && (prevHeatRelayOn != heatRelayOn || prevMotorRelayOn != motorRelayOn)) {
    // Only notify if there's an actual active-state flip
    pushStatusNotification();
  }

  // 6. Handle restart of advertising if disconnected
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();
    Serial.println("[BLE] Restarted advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    pushStatusNotification();
  }

  delay(20); // Small cooperative yield to FreeRTOS watchdog & BLE stack
}
