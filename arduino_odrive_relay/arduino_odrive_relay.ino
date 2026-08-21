/*
 * arduino_odrive_relay.ino
 * 
 * Target MCU: Arduino Uno R4 WiFi
 * Purpose: Dual ODrive v3.6 Omniwheel Relay with Proven Hardware Kinematics & Delays.
 * 
 * Hardware Wiring & Timing Configuration (Matches User Working Reference):
 *   - Rear ODrive SoftwareSerial: Pins 12 (RX), 11 (TX)
 *   - Front ODrive HardwareSerial: Serial1 (Pins 0 RX, 1 TX)
 *   - ODrive Setup Delays: 200ms between ASCII initialization commands.
 *   - ODrive Velocity Delays: 10ms between individual wheel 'v' commands.
 *   - Kinematics:
 *       Forward (W)  : FL=2,  FR=-2, RL=2,  RR=-2
 *       Backward (S) : FL=-2, FR=2,  RL=-2, RR=2
 *       Left (A)     : FL=-2, FR=-2, RL=2,  RR=2
 *       Right (D)    : FL=2,  FR=2,  RL=-2, RR=-2
 * 
 * Control Inputs:
 *   - Mac Local Relay Server: http://192.168.10.140:8000/api/poll
 *   - Render Cloud MQTT: omniwheel/cmd
 *   - USB Serial Monitor: 'w','s','a','d','x' + Enter
 */

#include <Arduino.h>
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <SoftwareSerial.h>

#define MAC_BAUDRATE 115200
#define ODRIVE_BAUDRATE 19200

// --- WI-FI CREDENTIALS ---
const char* WIFI_SSID = "sanhak";      // Wi-Fi SSID
const char* WIFI_PASS = "20020520";  // Wi-Fi Password

// --- MAC LOCAL RELAY SERVER ---
const char* MAC_SERVER_IP = "192.168.10.140";
const int   MAC_SERVER_PORT = 8000;

// --- CLOUD MQTT BROKER ---
const char mqttBroker[] = "broker.hivemq.com";
const int   mqttPort   = 1883;
const char topicCmd[]  = "omniwheel/cmd";

WiFiClient wifiClient;
WiFiClient macClient;
MqttClient mqttClient(wifiClient);

// Rear ODrive SoftwareSerial on Pins 12 (RX), 11 (TX)
SoftwareSerial odriveRear(12, 11);

// Mac Server Polling Interval (50ms)
const unsigned long MAC_POLL_INTERVAL_MS = 50;
unsigned long lastMacPollMs = 0;

// Reconnect Timer for MQTT
const unsigned long MQTT_RECONNECT_INTERVAL_MS = 3000;
unsigned long lastMqttConnectAttemptMs = 0;

// 500ms Deadman's Switch Safety Watchdog (Web mode only)
const unsigned long DEADMAN_TIMEOUT_MS = 500;
unsigned long lastMoveCommandMs = 0;

// System Control State
bool isSerialControlMode = false;
int driveSpeed = 2; // Default speed 2 turns/sec (matches proven working code)

// Track last executed command key
char lastExecutedKey = ' ';

// Function Prototypes
void connectToWiFi();
void pollMacServer();
void pollCloudMQTT();
void setupSerial1();
void setupSoftwareSerial(SoftwareSerial &odrive);
void handleWebControlInput();
void executeCommand(char key, const char* source);
void moveRobot(int fl, int fr, int rl, int rr);
void stopAllMotors();

void setup() {
    Serial.begin(MAC_BAUDRATE);
    delay(500);

    Serial1.begin(ODRIVE_BAUDRATE);
    odriveRear.begin(ODRIVE_BAUDRATE);
    delay(2000);

    Serial.println("\n==================================================");
    Serial.println("  🤖 Omni-Wheel Proven Hardware Relay Controller   ");
    Serial.println("==================================================");

    Serial.println("⚙️ Initializing Front & Rear ODrives...");
    setupSerial1();
    setupSoftwareSerial(odriveRear);

    stopAllMotors(); // Safety stop

    // Connect to Wi-Fi
    connectToWiFi();

    Serial.println("==================================================");
    Serial.print("▶️ Polling Mac Local Server: http://");
    Serial.print(MAC_SERVER_IP); Serial.print(":"); Serial.println(MAC_SERVER_PORT);
    Serial.println("▶️ Serial Input Control Ready: Type 'W','S','A','D','X' + Enter");
    Serial.println("==================================================\n");
}

void loop() {
    unsigned long currentMs = millis();

    // 1. Maintain Wi-Fi Connection & Poll Mac Local Server & Cloud MQTT
    if (WiFi.status() == WL_CONNECTED) {
        pollMacServer();
        pollCloudMQTT();
    }

    // 2. USB Serial Monitor Control & Passthrough
    handleWebControlInput();

    // 3. 500ms Deadman Watchdog (Web mode only): Auto-stop if no heartbeat received
    if (!isSerialControlMode && (currentMs - lastMoveCommandMs > DEADMAN_TIMEOUT_MS) && (lastExecutedKey != 'x' && lastExecutedKey != ' ')) {
        stopAllMotors();
        lastExecutedKey = 'x';
    }
}

/**
 * Handles Incoming Input from USB Serial Monitor Input Box
 */
void handleWebControlInput() {
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 1) {
            char key = toLowerCase(line.charAt(0));
            if (key == 'w' || key == 's' || key == 'a' || key == 'd') {
                isSerialControlMode = true;
                executeCommand(key, "USB Serial");
            } else if (key == 'x' || key == ' ') {
                isSerialControlMode = false;
                executeCommand('x', "USB Serial");
            } else {
                executeCommand(key, "USB Serial");
            }
        } else if (line.length() > 1) {
            Serial.print("🔧 [ODrive Direct Passthrough] -> ");
            Serial.println(line);
            Serial1.println(line);
            odriveRear.println(line);
        }
    }
}

/**
 * Polls Mac Local Relay Server (http://192.168.10.140:8000/api/poll) every 50ms
 */
void pollMacServer() {
    unsigned long now = millis();
    if (now - lastMacPollMs < MAC_POLL_INTERVAL_MS) return;
    lastMacPollMs = now;

    if (macClient.connect(MAC_SERVER_IP, MAC_SERVER_PORT)) {
        macClient.println("GET /api/poll HTTP/1.1");
        macClient.print("Host: "); macClient.println(MAC_SERVER_IP);
        macClient.println("Connection: close\r\n");

        unsigned long startWait = millis();
        while (!macClient.available() && (millis() - startWait < 40)) {
            delay(1);
        }

        String response = "";
        while (macClient.available()) {
            char c = macClient.read();
            response += c;
        }
        macClient.stop();

        if (response.length() > 0) {
            int lastNewline = response.lastIndexOf('\n');
            if (lastNewline != -1 && lastNewline < response.length() - 1) {
                String payload = response.substring(lastNewline + 1);
                payload.trim();
                if (payload.length() > 0) {
                    char cmdKey = toLowerCase(payload.charAt(0));
                    if (cmdKey == 'w' || cmdKey == 's' || cmdKey == 'a' || cmdKey == 'd' || 
                        cmdKey == 'x' || cmdKey == 'o' || (cmdKey >= '1' && cmdKey <= '9')) {
                        isSerialControlMode = false;
                        executeCommand(cmdKey, "Mac Local Server");
                    }
                }
            }
        }
    }
}

/**
 * Polls Cloud MQTT Broker (broker.hivemq.com:1883) for commands sent from Render Website
 */
void pollCloudMQTT() {
    if (!mqttClient.connected()) {
        unsigned long now = millis();
        if (now - lastMqttConnectAttemptMs >= MQTT_RECONNECT_INTERVAL_MS) {
            lastMqttConnectAttemptMs = now;
            mqttClient.setId("ArduinoR4_Omni");
            if (mqttClient.connect(mqttBroker, mqttPort)) {
                mqttClient.subscribe(topicCmd);
                Serial.println("✅ Connected to Cloud MQTT Broker (omniwheel/cmd)!");
            }
        }
        return;
    }

    mqttClient.poll();
    while (mqttClient.available()) {
        char c = (char)mqttClient.read();
        char cmdKey = toLowerCase(c);
        if (cmdKey == 'w' || cmdKey == 's' || cmdKey == 'a' || cmdKey == 'd' || 
            cmdKey == 'x' || cmdKey == 'o' || (cmdKey >= '1' && cmdKey <= '9')) {
            isSerialControlMode = false;
            executeCommand(cmdKey, "Render Cloud MQTT");
        }
    }
}

/**
 * Connect to Wi-Fi
 */
void connectToWiFi() {
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("❌ Error: Wi-Fi module not detected on Uno R4!");
        return;
    }

    WiFi.disconnect();
    delay(300);

    Serial.print("📶 Connecting Wi-Fi SSID: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    int attempts = 0;
    while ((WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)) && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
        Serial.println("\n✅ Wi-Fi Internet Connected!");
        Serial.print("🌐 Arduino IP: http://");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n⚠️ Wi-Fi DHCP Pending.");
    }
}

/**
 * Central Command Executor using Proven Wheel Kinematics
 */
void executeCommand(char key, const char* source) {
    int spd = driveSpeed;

    if (key == 'w') {
        lastMoveCommandMs = millis();
        moveRobot(spd, -spd, spd, -spd); // ⬆️ FORWARD

        if (lastExecutedKey != 'w') {
            lastExecutedKey = 'w';
            Serial.print("⬆️ ["); Serial.print(source); Serial.print("] FORWARD (speed: ");
            Serial.print(spd); Serial.println(")");
        }
    }
    else if (key == 's') {
        lastMoveCommandMs = millis();
        moveRobot(-spd, spd, -spd, spd); // ⬇️ BACKWARD

        if (lastExecutedKey != 's') {
            lastExecutedKey = 's';
            Serial.print("⬇️ ["); Serial.print(source); Serial.print("] BACKWARD (speed: ");
            Serial.print(-spd); Serial.println(")");
        }
    }
    else if (key == 'a') {
        lastMoveCommandMs = millis();
        moveRobot(-spd, -spd, spd, spd); // ⬅️ LEFT STRAFE

        if (lastExecutedKey != 'a') {
            lastExecutedKey = 'a';
            Serial.print("⬅️ ["); Serial.print(source); Serial.print("] LEFT STRAFE (speed: ");
            Serial.print(spd); Serial.println(")");
        }
    }
    else if (key == 'd') {
        lastMoveCommandMs = millis();
        moveRobot(spd, spd, -spd, -spd); // ➡️ RIGHT STRAFE

        if (lastExecutedKey != 'd') {
            lastExecutedKey = 'd';
            Serial.print("➡️ ["); Serial.print(source); Serial.print("] RIGHT STRAFE (speed: ");
            Serial.print(spd); Serial.println(")");
        }
    }
    else if (key == 'x' || key == ' ') {
        stopAllMotors(); // 🛑 STOP ALL
        if (lastExecutedKey != 'x') {
            lastExecutedKey = 'x';
            Serial.print("🛑 ["); Serial.print(source); Serial.println("] STOP ALL");
        }
    }
    else if (key >= '1' && key <= '9') {
        driveSpeed = (key - '0') * 2; // 1 -> speed 2, 2 -> speed 4, 3 -> speed 6...
        Serial.print("⚙️ ["); Serial.print(source); Serial.print("] Speed Set To: ");
        Serial.println(driveSpeed);
    }
}

/**
 * Proven Omniwheel Drive Kinematics with 10ms Inter-Command Delays
 */
void moveRobot(int fl, int fr, int rl, int rr) {
    // Front Wheels (Serial1)
    Serial1.print("v 0 "); Serial1.println(fl); // 2사분면 (FL)
    delay(10);
    Serial1.print("v 1 "); Serial1.println(fr); // 1사분면 (FR)
    delay(10);

    // Rear Wheels (odriveRear)
    odriveRear.print("v 1 "); odriveRear.println(rl); // 3사분면 (RL)
    delay(10);
    odriveRear.print("v 0 "); odriveRear.println(rr); // 4사분면 (RR)
    delay(10);
}

/**
 * Proven Safe Motor Stop Function
 */
void stopAllMotors() {
    Serial1.println("v 0 0"); delay(10);
    Serial1.println("v 1 0"); delay(10);
    odriveRear.println("v 0 0"); delay(10);
    odriveRear.println("v 1 0"); delay(10);
}

/**
 * Proven Front ODrive Setup with 200ms Delays
 */
void setupSerial1() {
    Serial1.println("c 0"); delay(200);
    Serial1.println("w axis0.requested_state 1"); delay(200);
    Serial1.println("w axis0.controller.config.control_mode 2"); delay(200);
    Serial1.println("w axis0.controller.config.input_mode 1"); delay(200);
    Serial1.println("w axis0.requested_state 8"); delay(200);

    Serial1.println("c 1"); delay(200);
    Serial1.println("w axis1.requested_state 1"); delay(200);
    Serial1.println("w axis1.controller.config.control_mode 2"); delay(200);
    Serial1.println("w axis1.controller.config.input_mode 1"); delay(200);
    Serial1.println("w axis1.requested_state 8"); delay(200);
}

/**
 * Proven Rear ODrive Setup with 200ms Delays
 */
void setupSoftwareSerial(SoftwareSerial &odrive) {
    odrive.println("c 0"); delay(200);
    odrive.println("w axis0.requested_state 1"); delay(200);
    odrive.println("w axis0.controller.config.control_mode 2"); delay(200);
    odrive.println("w axis0.controller.config.input_mode 1"); delay(200);
    odrive.println("w axis0.requested_state 8"); delay(200);

    odrive.println("c 1"); delay(200);
    odrive.println("w axis1.requested_state 1"); delay(200);
    odrive.println("w axis1.controller.config.control_mode 2"); delay(200);
    odrive.println("w axis1.controller.config.input_mode 1"); delay(200);
    odrive.println("w axis1.requested_state 8"); delay(200);
}
