#include <WiFi.h>
#include <PubSubClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WebServer.h>
#include <ElegantOTA.h>

#include <esp_task_wdt.h>
const int WDT_TIMEOUT = 15; // 15 seconds accommodates slow BLE handshakes

// --- Configuration (Credentials Restored) ---
const char* ssid                  = "YOUR_WIFI_NAME;
const char* password              = "YOUR_WIFI_PASSWORD";
const char* mqtt_server           = "YOUR_MQTT_SERVER_IP";
const uint16_t mqtt_port          = 1883;
const char* mqtt_user             = "YOUR_MQTT_USERNAME";
const char* mqtt_pass             = "YOUR_MQTT_PASSWORD";

// BT-1 Base MAC Address
const char* targetDeviceAddress   = "DE:AD:BE:EF:00:01"; //Your Renogy Bluetooth module Mac address

// Modbus Target Parameters
const uint8_t SLAVE_ID            = 0xFF; 

// --- Renogy BLE Dual-Service & Characteristic UUIDs ---
static BLEUUID writeServiceUUID("FFD0");
static BLEUUID charWriteUUID("FFD1"); 

static BLEUUID readServiceUUID("FFF0");
static BLEUUID charReadUUID("FFF1");  

// --- Global Engine Instances ---
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer server(80);
BLEClient* pClient = nullptr;

BLERemoteCharacteristic* pWriteChar = nullptr;
BLERemoteCharacteristic* pReadChar  = nullptr;

TaskHandle_t bTaskHandle          = NULL; 

volatile bool deviceConnected     = false;
volatile bool bleEnabled          = true; // Track Web UI override state
unsigned long lastQueryTime       = 0;
const unsigned long queryInterval = 10000; 

// --- BLE Packet Assembly Stream Buffer ---
uint8_t rxBuffer[256];
size_t rxIndex          = 0;
size_t expectedRxLength = 0;

// --- Forward Declarations ---
void processIncomingData(uint8_t* pData, size_t length);
uint16_t calculateCRC16(uint8_t* buffer, uint16_t length);

void logToMqtt(const char* topic, const char* message) {
    Serial.println(message);
    if (mqttClient.connected()) {
        mqttClient.publish(topic, message);
    }
}

// --- BLE Inbound Notification Callback ---
void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (rxIndex + length > sizeof(rxBuffer)) {
        logToMqtt("solar/debug", "BUFFER CRITICAL: Incoming MTU chunks overflowed stream layout. Flashing tracker.");
        rxIndex = 0;
        expectedRxLength = 0;
        return;
    }

    memcpy(rxBuffer + rxIndex, pData, length);
    rxIndex += length;

    if (rxIndex == length && length >= 3 && (rxBuffer[1] & 0x80)) {
        char logBuf[128];
        snprintf(logBuf, sizeof(logBuf), "Modbus EXCEPTION verified: Slave=%02X, Func=%02X, Error=%02X", rxBuffer[0], rxBuffer[1], rxBuffer[2]);
        logToMqtt("solar/debug", logBuf);
        rxIndex = 0;
        expectedRxLength = 0;
        return;
    }

    if (rxIndex >= 3 && expectedRxLength == 0) {
        if (rxBuffer[0] == SLAVE_ID && rxBuffer[1] == 0x03) {
            expectedRxLength = rxBuffer[2] + 5; 
        } else {
            rxIndex = 0;
            return;
        }
    }

    if (expectedRxLength > 0 && rxIndex >= expectedRxLength) {
        processIncomingData(rxBuffer, expectedRxLength);
        rxIndex = 0;
        expectedRxLength = 0;
    }
}

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        deviceConnected = true;
    }
    void onDisconnect(BLEClient* pclient) override {
        deviceConnected = false;
        logToMqtt("solar/debug", "CRITICAL: BLE connection dropped by remote device.");
    }
};

// --- Network Management Routines ---
void setupWiFi() {
    delay(10);
    Serial.print("\nConnecting to WiFi SSID: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
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
        if (now - lastReconnectAttempt > 5000) { 
            lastReconnectAttempt = now;
            Serial.print("Re-establishing MQTT state engine...");
            String clientId = "ESP32_Solar_Gateway_" + String(random(0xffff), HEX);
            
            if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
                Serial.println(" Connected.");
                mqttClient.publish("solar/status", "ESP32 Solar Gateway Engine Online");
                String currentIp = WiFi.localIP().toString();
                mqttClient.publish("solar/ipaddress", currentIp.c_str(), true);
            } else {
                Serial.print(" Failed, rc=");
                Serial.println(mqttClient.state());
            }
        }
    }
}

// --- Controller Handshake & Context Setup ---
bool connectToController() {
    char targetLog[128];
    snprintf(targetLog, sizeof(targetLog), "Initializing BT-1 link layer context target MAC: %s", targetDeviceAddress);
    logToMqtt("solar/debug", targetLog);
    
    if (pClient == nullptr) {
        pClient = BLEDevice::createClient();
        pClient->setClientCallbacks(new MyClientCallback());
    }

    BLEAddress targetAddr(targetDeviceAddress);
    if (!pClient->connect(targetAddr)) {
        logToMqtt("solar/debug", "ERROR: Physical link handshake failed.");
        return false;
    }
    logToMqtt("solar/debug", "Physical link secured. Map out GATT database layout...");

    BLERemoteService* pWriteService = pClient->getService(writeServiceUUID);
    if (pWriteService == nullptr) {
        logToMqtt("solar/debug", "FATAL: Service FFD0 missing.");
        pClient->disconnect();
        return false;
    }
    pWriteChar = pWriteService->getCharacteristic(charWriteUUID);

    BLERemoteService* pReadService = pClient->getService(readServiceUUID);
    if (pReadService == nullptr) {
        logToMqtt("solar/debug", "FATAL: Service FFF0 missing.");
        pClient->disconnect();
        return false;
    }
    pReadChar = pReadService->getCharacteristic(charReadUUID);

    if (pReadChar->canNotify()) {
        pReadChar->registerForNotify(notifyCallback);
        
        BLERemoteDescriptor* p2902Desc = pReadChar->getDescriptor(BLEUUID((uint16_t)0x2902));
        if (p2902Desc != nullptr) {
            uint8_t val[] = {0x01, 0x00};
            p2902Desc->writeValue(val, 2, true); 
            logToMqtt("solar/debug", "CCCD descriptor registration written successfully.");
        }
    } else {
        logToMqtt("solar/debug", "FATAL: Characteristic FFF1 rejected subscriber profile.");
        pClient->disconnect();
        return false;
    }

    logToMqtt("solar/debug", "Allowing BLE link layers to settle for 500ms...");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    logToMqtt("solar/debug", "BLE Pipeline initialization successful!");
    return true;
}

// --- Core 0 Dedicated Task for BLE Execution ---
void bleTask(void* pvParameters) {
    esp_task_wdt_add(NULL); 

    for (;;) {
        esp_task_wdt_reset(); // Feed the watchdog for Core 0

        // Handle Web UI forced Radio Kill Switch state
        if (!bleEnabled) {
            if (deviceConnected || (pClient != nullptr && pClient->isConnected())) {
                logToMqtt("solar/debug", "Web UI Request: Terminating active BLE link context to free radio.");
                if (pClient != nullptr) {
                    pClient->disconnect();
                }
                deviceConnected = false;
            }
            vTaskDelay(pdMS_TO_TICKS(500)); // Safe periodic check interval
            continue; 
        }

        if (!deviceConnected) {
            if (connectToController()) {
                logToMqtt("solar/debug", "Link secured, entering steady-state...");
            } else {
                for (int i = 0; i < 50; i++) {
                    esp_task_wdt_reset();
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                continue;
            }
        }

        if (deviceConnected) {
            unsigned long now = millis();
            if (now - lastQueryTime >= queryInterval) {
                lastQueryTime = now;
                
                rxIndex          = 0;
                expectedRxLength = 0;

                uint8_t dynamicQuery[8] = { SLAVE_ID, 0x03, 0x01, 0x00, 0x00, 0x22, 0xd1, 0xf1 };
                uint16_t crc = calculateCRC16(dynamicQuery, 6);
                dynamicQuery[6] = crc & 0xFF;         
                dynamicQuery[7] = (crc >> 8) & 0xFF;  

                pWriteChar->writeValue(dynamicQuery, sizeof(dynamicQuery), false); 
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

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

// --- Helper Formatting Publishing Engines ---
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

// --- Data Translation Engine ---
void processIncomingData(uint8_t* pData, size_t length) {
    auto readRegister = [&](int regOffset) -> uint16_t {
        int byteIndex = 3 + (regOffset * 2);
        return (uint16_t)((pData[byteIndex] << 8) | pData[byteIndex + 1]);
    };

    logToMqtt("solar/status", "Decoding reassembled telemetry registers...");

    uint16_t batterySoc = readRegister(0);
    publishInt("solar/battery/soc", batterySoc);

    float batteryVoltage = readRegister(1) * 0.1f;
    publishFloat("solar/battery/voltage", batteryVoltage, 1);

    float chargingCurrent = readRegister(2) * 0.01f;
    publishFloat("solar/charging/current", chargingCurrent, 2);

    uint16_t tempWord = readRegister(3);
    int8_t controllerTemp = (int8_t)((tempWord >> 8) & 0xFF);
    int8_t batteryTemp    = (int8_t)(tempWord & 0xFF);
    publishInt("solar/temperature/controller", controllerTemp);
    publishInt("solar/temperature/battery", batteryTemp);

    float loadVoltage = readRegister(4) * 0.1f;
    float loadCurrent = readRegister(5) * 0.01f;
    uint16_t loadPower = readRegister(6); 
    
    float calculatedWattage = loadVoltage * loadCurrent; 

    publishFloat("solar/load/voltage", loadVoltage, 1);
    publishFloat("solar/load/current", loadCurrent, 2);
    publishInt("solar/load/power", loadPower); 
    publishFloat("solar/load/wattage", calculatedWattage, 2); 

    float pvVoltage = readRegister(7) * 0.1f;
    float pvCurrent = readRegister(8) * 0.01f;
    uint16_t pvPower = readRegister(9);
    publishFloat("solar/pv/voltage", pvVoltage, 1);
    publishFloat("solar/pv/current", pvCurrent, 2);
    publishInt("solar/pv/power", pvPower);

    uint16_t generationToday = readRegister(18);
    publishInt("solar/today/generation_wh", generationToday);

    uint16_t rawStatus = readRegister(31);
    uint8_t chargingState = (uint8_t)(rawStatus & 0xFF);
    publishInt("solar/status/charging_state", chargingState);

    char rawLog[600] = ""; 
    for(size_t i = 3; i < length - 2; i++) {
        sprintf(rawLog + strlen(rawLog), "%02X", pData[i]);
    }
    mqttClient.publish("solar/data/raw_payload", rawLog);
}

// --- Initialization Runtime ---
void setup() {
    Serial.begin(115200);

    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,                
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, 
        .trigger_panic = true                            
    };
    
    esp_task_wdt_init(&wdt_config); 
    esp_task_wdt_add(NULL); 

    BLEDevice::init("");
    
    setupWiFi();
    mqttClient.setServer(mqtt_server, mqtt_port);
    
    // UI Portal serving a responsive status dashboard and toggle button
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

    // Hidden Action Route to handle state flip and redirect instantly
    server.on("/toggle", []() {
        bleEnabled = !bleEnabled;
        server.sendHeader("Location", "/");
        server.send(303); 
    });

    ElegantOTA.begin(&server);
    
    ElegantOTA.onStart([]() {
        Serial.println("[OTA] Firmware transmission detected! Isolating environment...");
        if (bTaskHandle != NULL) {
            vTaskSuspend(bTaskHandle);
            Serial.println("[OTA] Core 0 BLE Engine frozen successfully.");
        }
        if (pClient != nullptr && pClient->isConnected()) {
            pClient->disconnect();
            Serial.println("[OTA] Bluetooth connection dropped to clear radio spectrum.");
        }
        deviceConnected = false;
    });

    ElegantOTA.onEnd([](bool success) {
        if (success) {
            Serial.println("[OTA] Firmware flash complete! System rebooting instantly...");
        } else {
            Serial.println("[OTA ERROR] Flash upload disrupted. Resuming baseline operations...");
            if (bTaskHandle != NULL) {
                vTaskResume(bTaskHandle);
            }
        }
    });

    server.begin();
    Serial.println("[Setup] HTTP server and OTA Isolation hooks initialized.");

    xTaskCreatePinnedToCore(
        bleTask,             
        "BLE_Engine_Task",   
        8192,                
        NULL,                
        2,                   
        &bTaskHandle,        
        0                    
    );
}

// --- Core 1 Main Maintenance Loop ---
void loop() {
    esp_task_wdt_reset(); 

    server.handleClient();
    ElegantOTA.loop();

    reconnectMqtt(); 
    if (mqttClient.connected()) {
        mqttClient.loop();
    }
    
    delay(20); 
}