/*
  ================================================================================
  ESP32 Renogy BT-1 BLE-to-MQTT Bridge
  ================================================================================
  Architecture Overview:
    - Core 0: Dedicated FreeRTOS task ('bleTask') managing the continuous 
              Bluetooth Low Energy (BLE) link layer, Modbus RTU querying, 
              response timing, and automatic radio releases.
    - Core 1: Main Arduino loop ('loop') managing Wi-Fi, the embedded HTTP
              web dashboard, ElegantOTA firmware updates, and MQTT keep-alives.

  Key Resiliency & Safety Features:
    1. Inbound Modbus CRC16 Validation: Corrupted BLE frames are caught and
       discarded before corrupting database metrics.
    2. Response Timeout Guard: If a query gets no BLE response within 4 seconds,
       the connection is forcibly reset to prevent "zombie" link locks.
    3. Wi-Fi Loss Auto-Release: If home Wi-Fi drops, the ESP32 automatically drops 
       its BLE link to free up the BT-1 module for the official Renogy app.
       If the WiFi reconnects (as will happen automagically) the ESP32 reconnects 
       to the Bluetooth module and mqtt.
    4. Stale Telemetry Reboot Watchdog: If no valid telemetry frame decodes for 
       > 3 minutes, the system triggers a clean 'ESP.restart()'.
    5. Dual-Core Task Watchdog Timer (WDT): Hard kernel freezes trigger a hardware 
       reboot via 'esp_task_wdt'.
  ================================================================================
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <esp_task_wdt.h>

// --- Hardware Watchdog Configuration ---
// Must exceed the ESP32 BLE stack's internal 30-second blocking connection timeout
const int WDT_TIMEOUT = 35;  

// --- Network & MQTT Credentials ---
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "YOUR_MQTT_SERVER_IP";
const uint16_t mqtt_port = 1883;
const char* mqtt_user = "YOUR_MQTT_USERNAME";
const char* mqtt_pass = "YOUR_MQTT_PASSWORD";

// --- Target Renogy BT-1 Parameters ---
const char* targetDeviceAddress = "DE:AD:BE:EF:00:01";  // BT-1 MAC address
const uint8_t SLAVE_ID = 0xFF;                          // Default Renogy Modbus Slave ID (255)

// --- Renogy BLE Service & Characteristic UUIDs ---
// Write Channel (Command transmission)
static BLEUUID writeServiceUUID("FFD0");
static BLEUUID charWriteUUID("FFD1");

// Read/Notify Channel (Data reception)
static BLEUUID readServiceUUID("FFF0");
static BLEUUID charReadUUID("FFF1");

// --- Global Service & Driver Instances ---
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer server(80);
BLEClient* pClient = nullptr;

BLERemoteCharacteristic* pWriteChar = nullptr;
BLERemoteCharacteristic* pReadChar = nullptr;

TaskHandle_t bTaskHandle = NULL;  // Handle for Core 0 FreeRTOS BLE Task

// --- Global Operational State Flags ---
volatile bool deviceConnected = false;          // Tracks physical BLE link status
volatile bool bleEnabled = true;                // Web UI override (True = Active, False = Free Radio)
volatile bool waitingForResponse = false;       // Set to true when Modbus query is in-flight
unsigned long querySentTime = 0;                // Timestamp when the last query was transmitted
unsigned long lastQueryTime = 0;                // Timestamp of last polling interval start
unsigned long lastSuccessfulDecodeTime = 0;     // Heartbeat timestamp of last valid frame decode
const unsigned long queryInterval = 10000;      // Query controller every 10,000ms (10 seconds)
volatile bool isIntentionalDisconnect = false;  // Prevents false-alarm log warnings

// --- BLE Stream Buffer & Reassembly Engine ---
// Modbus RTU responses arrive sliced across multiple BLE MTU notification chunks.
uint8_t rxBuffer[256];
size_t rxIndex = 0;
size_t expectedRxLength = 0;

// --- Forward Declarations ---
void processIncomingData(uint8_t* pData, size_t length);
uint16_t calculateCRC16(uint8_t* buffer, uint16_t length);

// --- MQTT Logging Helper ---
// Emits messages to both the hardware Serial monitor and an MQTT debug channel
void logToMqtt(const char* topic, const char* message) {
  Serial.println(message);
  if (mqttClient.connected()) {
    mqttClient.publish(topic, message);
  }
}

// ================================================================================
// BLE INBOUND NOTIFICATION CALLBACK
// ================================================================================
// Executed asynchronously whenever the BT-1 pushes a BLE packet chunk via FFF1
void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  // 1. Safeguard against buffer overflow
  if (rxIndex + length > sizeof(rxBuffer)) {
    logToMqtt("solar/debug", "BUFFER CRITICAL: Incoming MTU chunks overflowed stream layout. Flashing tracker.");
    rxIndex = 0;
    expectedRxLength = 0;
    return;
  }

  // 2. Append incoming chunk to the stream assembly buffer
  memcpy(rxBuffer + rxIndex, pData, length);
  rxIndex += length;

  // 3. Check for Modbus Exception Response (High bit set on function code, e.g., 0x83)
  if (rxIndex == length && length >= 3 && (rxBuffer[1] & 0x80)) {
    char logBuf[128];
    snprintf(logBuf, sizeof(logBuf), "Modbus EXCEPTION verified: Slave=%02X, Func=%02X, Error=%02X", rxBuffer[0], rxBuffer[1], rxBuffer[2]);
    logToMqtt("solar/debug", logBuf);
    rxIndex = 0;
    expectedRxLength = 0;
    return;
  }

  // 4. Calculate total expected packet length upon receiving initial header bytes
  // Frame format: [Slave ID (1B)][Function Code (1B)][Byte Count (1B)][Data (NB)][CRC (2B)]
  if (rxIndex >= 3 && expectedRxLength == 0) {
    if (rxBuffer[0] == SLAVE_ID && rxBuffer[1] == 0x03) {
      expectedRxLength = rxBuffer[2] + 5;  // Data length + 1B Slave + 1B Func + 1B Count + 2B CRC
    } else {
      // Unrecognized or misaligned frame byte sequence; reset assembly buffer
      rxIndex = 0;
      return;
    }
  }

  // 5. Once the complete expected byte count is reassembled, trigger data processing
  if (expectedRxLength > 0 && rxIndex >= expectedRxLength) {
    processIncomingData(rxBuffer, expectedRxLength);
    rxIndex = 0;
    expectedRxLength = 0;
  }
}

// ================================================================================
// BLE CLIENT CONNECT/DISCONNECT EVENT CALLBACKS
// ================================================================================
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override {
    deviceConnected = true;
  }
  void onDisconnect(BLEClient* pclient) override {
    deviceConnected = false;
    waitingForResponse = false;

    if (isIntentionalDisconnect) {
      logToMqtt("solar/debug", "[BLE] Link closed gracefully by gateway.");
      isIntentionalDisconnect = false;  // Reset flag for next cycle
    } else {
      logToMqtt("solar/debug", "CRITICAL: BLE connection dropped by remote device.");
    }
  }
};

// ================================================================================
// NETWORK & MQTT MANAGEMENT ROUTINES
// ================================================================================
void setupWiFi() {
  delay(10);
  Serial.print("\nConnecting to WiFi SSID: ");
  Serial.println(ssid);
  WiFi.setAutoReconnect(true);  // Ensures background Wi-Fi auto-reassociation
  WiFi.persistent(true);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    esp_task_wdt_reset();  // Feed watchdog during blocking Wi-Fi connection loop
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nWiFi operational. Local Node IP: ");
  Serial.println(WiFi.localIP());
}

void reconnectMqtt() {
  static unsigned long lastReconnectAttempt = 0;

  if (!mqttClient.connected()) {
    unsigned long now = millis();
    // Non-blocking reconnect attempt every 5 seconds
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      Serial.print("Re-establishing MQTT state engine...");
      String clientId = "ESP32_Solar_Gateway_" + String(random(0xffff), HEX);

      if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
        Serial.println(" Connected.");
        mqttClient.publish("solar/status", "ESP32 Solar Gateway Engine Online");
        String currentIp = WiFi.localIP().toString();
        mqttClient.publish("solar/ipaddress", currentIp.c_str(), true);  // Retained topic
      } else {
        Serial.print(" Failed, rc=");
        Serial.println(mqttClient.state());
      }
    }
  }
}

// ================================================================================
// BLE PIPELINE & GATT HANDSHAKE ENGINE
// ================================================================================
bool connectToController() {
  char targetLog[128];
  snprintf(targetLog, sizeof(targetLog), "Initializing BT-1 link layer context target MAC: %s", targetDeviceAddress);
  logToMqtt("solar/debug", targetLog);

  // Instantiated once in static memory to avoid heap leaks on reconnection attempts
  static MyClientCallback myClientCallback;

  if (pClient == nullptr) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(&myClientCallback);  // Pass pointer to static instance
  }

  BLEAddress targetAddr(targetDeviceAddress);

  // Feed watchdog right before entering the blocking connect call
  esp_task_wdt_reset();

  // Connect to the physical BT-1 BLE radio (can block up to ~30s in underlying stack)
  if (!pClient->connect(targetAddr)) {
    logToMqtt("solar/debug", "ERROR: Physical link handshake failed. Deleting client context.");
    delete pClient;  // Clean up memory handle to prevent internal stack corruption
    pClient = nullptr;
    return false;
  }

  esp_task_wdt_reset();  // Refresh watchdog post-connect
  logToMqtt("solar/debug", "Physical link secured. Map out GATT database layout...");

  // Discover Write Service (FFD0) & Characteristic (FFD1)
  BLERemoteService* pWriteService = pClient->getService(writeServiceUUID);
  if (pWriteService == nullptr) {
    logToMqtt("solar/debug", "FATAL: Service FFD0 missing.");
    pClient->disconnect();
    return false;
  }
  pWriteChar = pWriteService->getCharacteristic(charWriteUUID);

  esp_task_wdt_reset();  // Refresh watchdog during discovery

  // Discover Read Service (FFF0) & Characteristic (FFF1)
  BLERemoteService* pReadService = pClient->getService(readServiceUUID);
  if (pReadService == nullptr) {
    logToMqtt("solar/debug", "FATAL: Service FFF0 missing.");
    pClient->disconnect();
    return false;
  }
  pReadChar = pReadService->getCharacteristic(charReadUUID);

  // Register notification listener on FFF1 and write the CCCD Descriptor (0x2902)
  if (pReadChar->canNotify()) {
    pReadChar->registerForNotify(notifyCallback);

    BLERemoteDescriptor* p2902Desc = pReadChar->getDescriptor(BLEUUID((uint16_t)0x2902));
    if (p2902Desc != nullptr) {
      uint8_t val[] = { 0x01, 0x00 };  // Enable notifications command
      p2902Desc->writeValue(val, 2, true);
      logToMqtt("solar/debug", "CCCD descriptor registration written successfully.");
    }
  } else {
    logToMqtt("solar/debug", "FATAL: Characteristic FFF1 rejected subscriber profile.");
    pClient->disconnect();
    return false;
  }

  logToMqtt("solar/debug", "Allowing BLE link layers to settle for 500ms...");
  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_task_wdt_reset();

  logToMqtt("solar/debug", "BLE Pipeline initialization successful!");
  return true;
}

// ================================================================================
// CORE 0 DEDICATED FREERTOS BLE TASK
// ================================================================================
// Runs independently on CPU Core 0 to prevent WiFi/HTTP traffic from jittering BLE timing
void bleTask(void* pvParameters) {
  esp_task_wdt_add(NULL);  // Register Core 0 thread to Task Watchdog Timer

  for (;;) {
    esp_task_wdt_reset();  // Feed Core 0 watchdog on every execution cycle

    // SAFEGUARD 1: Auto-release BLE link if WiFi drops.
    // Drops the physical connection so you can pair with the phone app immediately.
    if (WiFi.status() != WL_CONNECTED) {
      if (deviceConnected || (pClient != nullptr && pClient->isConnected())) {
        Serial.println("[BLE Guard] WiFi disconnected! Releasing BT-1 link for app usage...");
        isIntentionalDisconnect = true;
        if (pClient != nullptr) {
          pClient->disconnect();
        }
        deviceConnected = false;
        waitingForResponse = false;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // Handle Web UI manual radio override state ("Disconnect BLE" button pressed)
    if (!bleEnabled) {
      if (deviceConnected || (pClient != nullptr && pClient->isConnected())) {
        logToMqtt("solar/debug", "Web UI Request: Terminating active BLE link context to free radio.");
        isIntentionalDisconnect = true;
        if (pClient != nullptr) {
          pClient->disconnect();
        }
        deviceConnected = false;
        waitingForResponse = false;
      }
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    // Attempt link layer connection if currently disconnected
    if (!deviceConnected) {
      if (connectToController()) {
        logToMqtt("solar/debug", "Link secured, entering steady-state...");
      } else {
        // Connection failed: retry after a short, watchdog-safe backoff loop
        for (int i = 0; i < 50; i++) {
          esp_task_wdt_reset();
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        continue;
      }
    }

    // Steady-State Polling Loop
    if (deviceConnected) {
      unsigned long now = millis();

      // SAFEGUARD 2: Response Timeout Guard
      // Reset physical BLE connection if query hangs without response for > 4 seconds
      if (waitingForResponse && (now - querySentTime > 4000)) {
        logToMqtt("solar/debug", "Query timeout (no BLE response). Resetting connection.");
        isIntentionalDisconnect = true;
        if (pClient != nullptr) {
          pClient->disconnect();
        }
        deviceConnected = false;
        waitingForResponse = false;
      }

      // Periodic Modbus Polling Handler (Every 10 seconds)
      if (now - lastQueryTime >= queryInterval && !waitingForResponse) {
        lastQueryTime = now;

        rxIndex = 0;
        expectedRxLength = 0;

        // Construct Modbus RTU Read Holding Registers Frame (Func 0x03)
        // Register Start: 0x0100 (256 decimal), Quantity: 0x0022 (34 registers)
        uint8_t dynamicQuery[8] = { SLAVE_ID, 0x03, 0x01, 0x00, 0x00, 0x22, 0x00, 0x00 };

        // Calculate and append 16-bit CRC Checksum (Little-Endian)
        uint16_t crc = calculateCRC16(dynamicQuery, 6);
        dynamicQuery[6] = crc & 0xFF;         // Low byte
        dynamicQuery[7] = (crc >> 8) & 0xFF;  // High byte

        // Transmit query frame to characteristic FFD1 (without write response for speed)
        pWriteChar->writeValue(dynamicQuery, sizeof(dynamicQuery), false);

        querySentTime = now;        // Stamp query send time
        waitingForResponse = true;  // Arm timeout monitor
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Yield thread brief sleep for RTOS scheduler
  }
}

// ================================================================================
// MODBUS RTU CRC16 CHECKSUM ENGINE
// ================================================================================
// Standard Modbus RTU CRC16 algorithm using polynomial 0xA001 (Bitwise reverse of 0x8005)
uint16_t calculateCRC16(uint8_t* buffer, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// --- MQTT Formatting Helpers ---
void publishFloat(const char* topic, float val, int precision) {
  char buf[16];
  dtostrf(val, 1, precision, buf);
  mqttClient.publish(topic, buf);
}

void publishInt(const char* topic, int val) {
  char buf[16];
  itoa(val, buf, 10);
  mqttClient.publish(topic, buf);
}

// ================================================================================
// MODBUS DATA TRANSLATION & MQTT PUBLISHING ENGINE
// ================================================================================
void processIncomingData(uint8_t* pData, size_t length) {
  // SAFEGUARD 3: Inbound Checksum Validation
  // Verify trailing 2-byte CRC against calculated frame checksum before processing data
  uint16_t rxCrc = pData[length - 2] | (pData[length - 1] << 8);
  if (calculateCRC16(pData, length - 2) != rxCrc) {
    logToMqtt("solar/debug", "CRC mismatch on inbound frame! Discarding packet.");
    return;
  }

  // Packet successfully verified!
  waitingForResponse = false;           // Cancel timeout timer
  lastSuccessfulDecodeTime = millis();  // Refresh telemetry heartbeat timestamp

  // Lambda helper: Extracts 16-bit Big-Endian register from Modbus payload byte offset
  auto readRegister = [&](int regOffset) -> uint16_t {
    int byteIndex = 3 + (regOffset * 2);  // Skip [Slave][Func][ByteCount]
    return (uint16_t)((pData[byteIndex] << 8) | pData[byteIndex + 1]);
  };

  logToMqtt("solar/status", "Decoding reassembled telemetry registers...");

  // Register 0x0100: Battery State of Charge (%)
  uint16_t batterySoc = readRegister(0);
  publishInt("solar/battery/soc", batterySoc);

  // Register 0x0101: Battery Voltage (Scale: 0.1V)
  float batteryVoltage = readRegister(1) * 0.1f;
  publishFloat("solar/battery/voltage", batteryVoltage, 1);

  // Register 0x0102: Charging Current to Battery (Scale: 0.01A)
  float chargingCurrent = readRegister(2) * 0.01f;
  publishFloat("solar/charging/current", chargingCurrent, 2);

  // Register 0x0103: Controller & Battery Temperature (Packed high/low bytes in °C)
  uint16_t tempWord = readRegister(3);
  int8_t controllerTemp = (int8_t)((tempWord >> 8) & 0xFF);
  int8_t batteryTemp = (int8_t)(tempWord & 0xFF);
  publishInt("solar/temperature/controller", controllerTemp);
  publishInt("solar/temperature/battery", batteryTemp);

  // Registers 0x0104 - 0x0106: Load Output Voltage, Current, and Power
  float loadVoltage = readRegister(4) * 0.1f;
  float loadCurrent = readRegister(5) * 0.01f;
  uint16_t loadPower = readRegister(6);
  float calculatedWattage = loadVoltage * loadCurrent;

  publishFloat("solar/load/voltage", loadVoltage, 1);
  publishFloat("solar/load/current", loadCurrent, 2);
  publishInt("solar/load/power", loadPower);
  publishFloat("solar/load/wattage", calculatedWattage, 2);

  // Registers 0x0107 - 0x0109: Solar PV Input Voltage, Current, and Power
  float pvVoltage = readRegister(7) * 0.1f;
  float pvCurrent = readRegister(8) * 0.01f;
  uint16_t pvPower = readRegister(9);
  publishFloat("solar/pv/voltage", pvVoltage, 1);
  publishFloat("solar/pv/current", pvCurrent, 2);
  publishInt("solar/pv/power", pvPower);

  // Register 0x0112: Today's Power Generation (Watt-hours)
  uint16_t generationToday = readRegister(18);
  publishInt("solar/today/generation_wh", generationToday);

  // Register 0x011F: Operating State Bitmask (Lower byte = Charging Stage)
  uint16_t rawStatus = readRegister(31);
  uint8_t chargingState = (uint8_t)(rawStatus & 0xFF);
  publishInt("solar/status/charging_state", chargingState);

  // Publish raw hex string for debugging/tracing
  char rawLog[600] = "";
  for (size_t i = 3; i < length - 2; i++) {
    sprintf(rawLog + strlen(rawLog), "%02X", pData[i]);
  }
  mqttClient.publish("solar/data/raw_payload", rawLog);
}

// ================================================================================
// SYSTEM INITIALIZATION (SETUP)
// ================================================================================
void setup() {
  Serial.begin(115200);

  // Safely configure or reconfigure ESP32 Task Watchdog Timer
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  
  if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK) {
    esp_task_wdt_init(&wdt_config);
  }
  esp_task_wdt_add(NULL);  // Register main Core 1 thread to WDT

  BLEDevice::init("");

  setupWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);

  // --- HTTP Web Server Routes ---
  // Root dashboard serving BLE state and control button
  server.on("/", []() {
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Solar Gateway Control</title><style>";
    html += "body{font-family:-apple-system,BlinkMacSystemFont,sans-serif;background:#121212;color:#e0e0e0;padding:20px;text-align:center;}";
    html += ".card{background:#1e1e1e;padding:30px;border-radius:8px;max-width:400px;margin:40px auto;box-shadow:0 4px 10px rgba(0,0,0,0.5);}";
    html += ".status{font-size:18px;margin-bottom:25px;}";
    html += ".state-active{color:#2ecc71;font-weight:bold;} .state-disabled{color:#e74c3c;font-weight:bold;}";
    html += ".btn{display:inline-block;padding:12px 24px;color:#fff;text-decoration:none;border-radius:4px;font-weight:bold;transition:0.2s;}";
    html += ".btn-disable{background:#e74c3c;} .btn-disable:hover{background:#c0392b;}";
    html += ".btn-enable{background:#2ecc71;} .btn-enable:hover{background:#27ae60;}";
    html += ".ota-link{display:block;margin-top:25px;color:#3498db;font-size:14px;text-decoration:none;}";
    html += "</style></head><body><div class='card'>";
    html += "<h2>Solar Gateway</h2>";
    html += "<div class='status'>BLE Engine: ";

    if (bleEnabled) {
      html += "<span class='state-active'>ONLINE</span></div>";
      html += "<a href='/toggle' class='btn btn-disable'>Disconnect BLE</a>";
    } else {
      html += "<span class='state-disabled'>DISCONNECTED (FREE)</span></div>";
      html += "<a href='/toggle' class='btn btn-enable'>Connect BLE</a>";
    }

    html += "<a href='/update' class='ota-link'>Go to /update for OTA operations</a>";
    html += "</div></body></html>";

    server.send(200, "text/html", html);
  });

  // Hidden Action Route: Flips BLE enable flag and redirects back to root UI
  server.on("/toggle", []() {
    bleEnabled = !bleEnabled;
    server.sendHeader("Location", "/");
    server.send(303);
  });

  // Initialize Over-The-Air (OTA) flashing service
  ElegantOTA.begin(&server);

  // Freeze BLE operations during OTA flashing to ensure clean wireless bandwidth
  ElegantOTA.onStart([]() {
    Serial.println("[OTA] Firmware transmission detected! Isolating environment...");

    // 1. Mark disconnect as intentional so onDisconnect doesn't log a false alarm
    isIntentionalDisconnect = true;

    // 2. Gracefully drop BLE first while bleTask is still active to handle stack cleanup
    if (pClient != nullptr && pClient->isConnected()) {
      pClient->disconnect();
      Serial.println("[OTA] Bluetooth connection dropped gracefully to clear radio spectrum.");
      delay(100);  // Give the BLE stack 100ms to send the disconnect packet
    }

    // 3. Freeze Core 0 BLE task for the duration of the firmware flash
    if (bTaskHandle != NULL) {
      vTaskSuspend(bTaskHandle);
      Serial.println("[OTA] Core 0 BLE Engine frozen successfully.");
    }

    deviceConnected = false;
    waitingForResponse = false;
  });

  ElegantOTA.onEnd([](bool success) {
    if (success) {
      Serial.println("[OTA] Firmware flash complete! System rebooting instantly...");
    } else {
      Serial.println("[OTA ERROR] Flash upload disrupted. Resuming baseline operations...");
      if (bTaskHandle != NULL) {
        vTaskResume(bTaskHandle);  // Resume Core 0 thread if update aborted
      }
    }
  });

  server.begin();
  Serial.println("[Setup] HTTP server and OTA Isolation hooks initialized.");

  // Spawn dedicated FreeRTOS thread for BLE management, pinned strictly to Core 0
  xTaskCreatePinnedToCore(
    bleTask,            // Task function pointer
    "BLE_Engine_Task",  // Task name string
    8192,               // Stack size in words (8KB)
    NULL,               // Task input parameter
    2,                  // Priority level (Higher than background idle tasks)
    &bTaskHandle,       // Task handle reference
    0                   // Pin strictly to Core 0
  );
}

// ================================================================================
// MAIN LOOP (EXECUTES CONTINUOUSLY ON CPU CORE 1)
// ================================================================================
void loop() {
  esp_task_wdt_reset();  // Feed Core 1 watchdog

  // SAFEGUARD 4: Stale Telemetry Reboot Watchdog
  // Force reboot if telemetry has been silent for longer than 3 minutes (180,000ms)
  if (lastSuccessfulDecodeTime > 0 && (millis() - lastSuccessfulDecodeTime > 180000)) {
    Serial.println("CRITICAL: Stale telemetry detected (>3m). Forcing system restart...");
    ESP.restart();
  }

  // Process web dashboard requests and OTA server loops
  server.handleClient();
  ElegantOTA.loop();

  // Maintain MQTT broker connection state
  reconnectMqtt();
  if (mqttClient.connected()) {
    mqttClient.loop();
  }

  delay(20);  // Short yield to prevent CPU Core 1 starvation
}