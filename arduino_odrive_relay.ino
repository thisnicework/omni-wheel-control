/*
 * arduino_odrive_relay.ino
 * 
 * Target MCU: Arduino Uno R4 WiFi
 * Purpose: Dual ODrive v3.6 Relay with Strict Numeric Echo Filtering.
 * 
 * Fix for False State 0 Error Reporting:
 *   - Strict Numeric Filtering: Prevents corrupted UART echo lines (e.g. ' 1 -6.00') from triggering false State 0 error logs.
 *   - Guarantees true ODrive State 8 reporting over USB Serial.
 *   - Supports Mac Local Relay Server (192.168.10.140:8000), Cloud MQTT, & USB Serial.
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

// Rear ODrive SoftwareSerial on Pins 9 (RX), 8 (TX)
SoftwareSerial odriveRear(9, 8);

// 20Hz Continuous Velocity Update Loop (50ms)
const unsigned long ROAM_INTERVAL_MS = 50;
unsigned long lastRoamMs = 0;

// Mac Server Polling Interval (50ms)
const unsigned long MAC_POLL_INTERVAL_MS = 50;
unsigned long lastMacPollMs = 0;

// Reconnect Timer for MQTT
const unsigned long MQTT_RECONNECT_INTERVAL_MS = 3000;
unsigned long lastMqttConnectAttemptMs = 0;

// 3-Second ODrive Status Query Loop
const unsigned long ODRIVE_STATUS_INTERVAL_MS = 3000;
unsigned long lastODriveStatusMs = 0;

// 500ms Deadman's Switch Safety Watchdog
const unsigned long DEADMAN_TIMEOUT_MS = 500;
unsigned long lastMoveCommandMs = 0;

// System Motion State
bool isAutoRoamEnabled = false;
bool isArmed = false;
float currentDriveSpeed = 6.0f; // Velocity turns/sec (Default 6.0 rps = 360 RPM for high torque)

// Continuous Target & Current Velocities (Vx, Vy)
float targetVx = 0.0f;
float targetVy = 0.0f;
float currentVx = 0.0f;
float currentVy = 0.0f;

// Track last executed command key to avoid redundant logging
char lastExecutedKey = ' ';

// --- AUTO-ROAM MOTION VARIABLES ---
enum StepState {
    STATE_GENTLE_DASH,
    STATE_BRIEF_REST
};

StepState currentState = STATE_GENTLE_DASH;
unsigned long stateStartTime = 0;
unsigned long stateDuration = 2000;

float fixedVx = 0.0f;
float fixedVy = 0.0f;

// Rx Buffers for ODrive line parsing
const size_t RX_BUFFER_SIZE = 128;
char rxBuffer1[RX_BUFFER_SIZE];
size_t rxIndex1 = 0;

char rxBufferRear[RX_BUFFER_SIZE];
size_t rxIndexRear = 0;

// ODrive Diagnostic Storage
int frontState = -1;
uint32_t frontErr = 0;
int rearState = -1;
uint32_t rearErr = 0;

enum QueryStep {
    Q_NONE,
    Q_FRONT_STATE,
    Q_FRONT_ERR,
    Q_REAR_STATE,
    Q_REAR_ERR
};
QueryStep frontQuery = Q_NONE;
QueryStep rearQuery = Q_NONE;

// Function Prototypes
void connectToWiFi();
void pollMacServer();
void pollCloudMQTT();
void setupODriveFront();
void setupODriveRear();
void rearmODrivesIfNeeded();
void queryODriveStatus();
void processSerial1();
void processSoftSerial();
void handleWebControlInput();
void executeCommand(char key, const char* source);
void updateAutoRoamMotion();
void moveRobotVelocities(float vx, float vy);
void stopAllMotors();
bool isCleanIntegerString(const char* str);

void setup() {
    Serial.begin(MAC_BAUDRATE);
    delay(500);

    Serial1.begin(ODRIVE_BAUDRATE);
    odriveRear.begin(ODRIVE_BAUDRATE);
    delay(1000);

    Serial.println("\n==================================================");
    Serial.println("  🤖 Arduino Uno R4 WiFi - Strict Response Engine ");
    Serial.println("==================================================");

    // Initialize ODrives into Closed Loop Velocity Control once at boot
    setupODriveFront();
    setupODriveRear();
    stopAllMotors();

    memset(rxBuffer1, 0, RX_BUFFER_SIZE);
    memset(rxBufferRear, 0, RX_BUFFER_SIZE);

    // Connect to Wi-Fi Internet
    connectToWiFi();

    Serial.println("==================================================");
    Serial.print("▶️ Polling Mac Local Server: http://");
    Serial.print(MAC_SERVER_IP); Serial.print(":"); Serial.println(MAC_SERVER_PORT);
    Serial.println("==================================================\n");
}

void loop() {
    unsigned long currentMs = millis();

    // 1. Maintain Wi-Fi Connection & Poll Mac Local Server & Cloud MQTT
    if (WiFi.status() == WL_CONNECTED) {
        pollMacServer();
        pollCloudMQTT();
    }

    // 2. Real-Time USB Serial Backup Commands
    handleWebControlInput();

    // 3. 500ms Deadman Watchdog: Auto-stop if no move command received recently
    if (!isAutoRoamEnabled && (currentMs - lastMoveCommandMs > DEADMAN_TIMEOUT_MS) && (targetVx != 0.0f || targetVy != 0.0f)) {
        stopAllMotors();
    }

    // 4. 20Hz CONTINUOUS VELOCITY CONTROL LOOP (50ms)
    if (currentMs - lastRoamMs >= ROAM_INTERVAL_MS) {
        lastRoamMs = currentMs;
        if (isAutoRoamEnabled) {
            updateAutoRoamMotion();
        } else {
            // Smoothly ramp currentVx, currentVy towards targetVx, targetVy
            currentVx += (targetVx - currentVx) * 0.50f;
            currentVy += (targetVy - currentVy) * 0.50f;

            if (abs(currentVx) > 0.02f || abs(currentVy) > 0.02f || abs(targetVx) > 0.02f || abs(targetVy) > 0.02f) {
                moveRobotVelocities(currentVx, currentVy);
            }
        }
    }

    // 5. 3-Second Live ODrive Status Query Loop
    if (currentMs - lastODriveStatusMs >= ODRIVE_STATUS_INTERVAL_MS) {
        lastODriveStatusMs = currentMs;
        queryODriveStatus();
    }

    // 6. Asynchronously read incoming ODrive responses
    processSerial1();
    processSoftSerial();
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
                        cmdKey == 'x' || cmdKey == 'o' || cmdKey == 'i' || (cmdKey >= '1' && cmdKey <= '9')) {
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
            cmdKey == 'x' || cmdKey == 'o' || cmdKey == 'i' || (cmdKey >= '1' && cmdKey <= '9')) {
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
 * Polls Front & Rear ODrive state & error status
 */
void queryODriveStatus() {
    frontQuery = Q_FRONT_STATE;
    Serial1.println("r axis0.current_state");
    
    rearQuery = Q_REAR_STATE;
    odriveRear.println("r axis0.current_state");
}

/**
 * Re-arms ODrive motors ONLY if transitioning from stopped state
 */
void rearmODrivesIfNeeded() {
    if (!isArmed) {
        Serial1.println("w axis0.error 0");
        Serial1.println("w axis1.error 0");
        Serial1.println("w axis0.requested_state 8");
        Serial1.println("w axis1.requested_state 8");

        odriveRear.println("w axis0.error 0");
        odriveRear.println("w axis1.error 0");
        odriveRear.println("w axis0.requested_state 8");
        odriveRear.println("w axis1.requested_state 8");
        isArmed = true;
    }
}

/**
 * Central Command Executor - INSTANT DIRECT UART OUTPUT
 */
void executeCommand(char key, const char* source) {
    if (key == 'w') {
        isAutoRoamEnabled = false;
        lastMoveCommandMs = millis();
        rearmODrivesIfNeeded();
        targetVx = 0.0f;
        targetVy = currentDriveSpeed;
        currentVx = targetVx;
        currentVy = targetVy;
        moveRobotVelocities(currentVx, currentVy);

        if (lastExecutedKey != 'w') {
            lastExecutedKey = 'w';
            Serial.print("🌐 ["); Serial.print(source); Serial.print(" Received] Cmd: 'W' -> FORWARD (");
            Serial.print(currentDriveSpeed); Serial.println(" turns/s)");
        }
    }
    else if (key == 's') {
        isAutoRoamEnabled = false;
        lastMoveCommandMs = millis();
        rearmODrivesIfNeeded();
        targetVx = 0.0f;
        targetVy = -currentDriveSpeed;
        currentVx = targetVx;
        currentVy = targetVy;
        moveRobotVelocities(currentVx, currentVy);

        if (lastExecutedKey != 's') {
            lastExecutedKey = 's';
            Serial.print("🌐 ["); Serial.print(source); Serial.print(" Received] Cmd: 'S' -> BACKWARD (-");
            Serial.print(currentDriveSpeed); Serial.println(" turns/s)");
        }
    }
    else if (key == 'a') {
        isAutoRoamEnabled = false;
        lastMoveCommandMs = millis();
        rearmODrivesIfNeeded();
        targetVx = -currentDriveSpeed;
        targetVy = 0.0f;
        currentVx = targetVx;
        currentVy = targetVy;
        moveRobotVelocities(currentVx, currentVy);

        if (lastExecutedKey != 'a') {
            lastExecutedKey = 'a';
            Serial.print("🌐 ["); Serial.print(source); Serial.print(" Received] Cmd: 'A' -> LEFT STRAFE (-");
            Serial.print(currentDriveSpeed); Serial.println(" turns/s)");
        }
    }
    else if (key == 'd') {
        isAutoRoamEnabled = false;
        lastMoveCommandMs = millis();
        rearmODrivesIfNeeded();
        targetVx = currentDriveSpeed;
        targetVy = 0.0f;
        currentVx = targetVx;
        currentVy = targetVy;
        moveRobotVelocities(currentVx, currentVy);

        if (lastExecutedKey != 'd') {
            lastExecutedKey = 'd';
            Serial.print("🌐 ["); Serial.print(source); Serial.print(" Received] Cmd: 'D' -> RIGHT STRAFE (");
            Serial.print(currentDriveSpeed); Serial.println(" turns/s)");
        }
    }
    else if (key == 'x' || key == 'o') {
        isAutoRoamEnabled = false;
        if (targetVx != 0.0f || targetVy != 0.0f || lastExecutedKey != 'x') {
            stopAllMotors();
            if (lastExecutedKey != 'x') {
                lastExecutedKey = 'x';
                Serial.print("🛑 ["); Serial.print(source); Serial.println(" Received] Cmd: STOP ALL");
            }
        }
    }
    else if (key == 'i') {
        isAutoRoamEnabled = true;
        rearmODrivesIfNeeded();
        stateStartTime = millis();
        lastExecutedKey = 'i';
        Serial.print("🤖 ["); Serial.print(source); Serial.println(" Received] Cmd: AUTO ROAM [I]");
    }
    else if (key >= '1' && key <= '9') {
        currentDriveSpeed = 2.0f + (key - '1') * 2.0f;
        Serial.print("⚙️ ["); Serial.print(source); Serial.print(" Received] Speed Set To: ");
        Serial.print(currentDriveSpeed); Serial.println(" turns/s");
    }
}

/**
 * Output velocity commands continuously to 4 Omniwheels (Zero Rotation)
 */
void moveRobotVelocities(float vx, float vy) {
    float fl =  vy + vx;
    float fr = -(vy - vx);
    float rl =  vy - vx;
    float rr = -(vy + vx);

    Serial1.print("v 0 "); Serial1.println(fl, 2);
    Serial1.print("v 1 "); Serial1.println(fr, 2);

    odriveRear.print("v 1 "); odriveRear.println(rl, 2);
    odriveRear.print("v 0 "); odriveRear.println(rr, 2);
}

/**
 * Fast Non-Blocking Motor Stop
 */
void stopAllMotors() {
    targetVx = 0.0f;
    targetVy = 0.0f;
    currentVx = 0.0f;
    currentVy = 0.0f;
    isArmed = false;

    Serial1.println("v 0 0");
    Serial1.println("v 1 0");
    odriveRear.println("v 0 0");
    odriveRear.println("v 1 0");
}

void setupODriveFront() {
    Serial1.println("w axis0.error 0"); delay(30);
    Serial1.println("w axis1.error 0"); delay(30);

    Serial1.println("w axis0.requested_state 1"); delay(30);
    Serial1.println("w axis0.controller.config.control_mode 2"); delay(30);
    Serial1.println("w axis0.controller.config.input_mode 1"); delay(30);
    Serial1.println("w axis0.requested_state 8"); delay(30);

    Serial1.println("w axis1.requested_state 1"); delay(30);
    Serial1.println("w axis1.controller.config.control_mode 2"); delay(30);
    Serial1.println("w axis1.controller.config.input_mode 1"); delay(30);
    Serial1.println("w axis1.requested_state 8"); delay(30);
}

void setupODriveRear() {
    odriveRear.println("w axis0.error 0"); delay(30);
    odriveRear.println("w axis1.error 0"); delay(30);

    odriveRear.println("w axis0.requested_state 1"); delay(30);
    odriveRear.println("w axis0.controller.config.control_mode 2"); delay(30);
    odriveRear.println("w axis0.controller.config.input_mode 1"); delay(30);
    odriveRear.println("w axis0.requested_state 8"); delay(30);

    odriveRear.println("w axis1.requested_state 1"); delay(30);
    odriveRear.println("w axis1.controller.config.control_mode 2"); delay(30);
    odriveRear.println("w axis1.controller.config.input_mode 1"); delay(30);
    odriveRear.println("w axis1.requested_state 8"); delay(30);
}

void handleWebControlInput() {
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 1) {
            char key = toLowerCase(line.charAt(0));
            executeCommand(key, "USB Serial");
        } else if (line.length() > 1) {
            Serial.print("🔧 [ODrive Passthrough] -> ");
            Serial.println(line);
            Serial1.println(line);
            odriveRear.println(line);
        }
    }
}

void updateAutoRoamMotion() {
    unsigned long now = millis();
    unsigned long elapsed = now - stateStartTime;

    if (elapsed > stateDuration) {
        stateStartTime = now;

        if (currentState == STATE_GENTLE_DASH) {
            currentState = STATE_BRIEF_REST;
            stateDuration = 800;
            fixedVx = 0.0f;
            fixedVy = 0.0f;
        } else {
            currentState = STATE_GENTLE_DASH;
            stateDuration = random(1200, 2500);
            
            float angle = (random(0, 360) * DEG_TO_RAD);
            float speed = 1.0f + (random(0, 10) / 10.0f);
            
            fixedVx = cos(angle) * speed;
            fixedVy = sin(angle) * speed;
        }
    }

    if (currentState == STATE_GENTLE_DASH) {
        currentVx += (fixedVx - currentVx) * 0.25f;
        currentVy += (fixedVy - currentVy) * 0.25f;
    } else {
        float breath = sin(now * 0.002f) * 0.10f;
        currentVx += (breath - currentVx) * 0.25f;
        currentVy += (breath - currentVy) * 0.25f;
    }

    moveRobotVelocities(currentVx, currentVy);
}

bool isCleanIntegerString(const char* str) {
    if (!str || strlen(str) == 0) return false;
    for (size_t i = 0; i < strlen(str); i++) {
        if (!isdigit(str[i])) return false;
    }
    return true;
}

void processSerial1() {
    while (Serial1.available() > 0) {
        char c = (char)Serial1.read();
        if (c == '\n' || c == '\r') {
            if (rxIndex1 > 0) {
                rxBuffer1[rxIndex1] = '\0';
                if (rxBuffer1[0] != 'r' && rxBuffer1[0] != 'v' && rxBuffer1[0] != 'w' && rxBuffer1[0] != 'c') {
                    if (frontQuery == Q_FRONT_STATE) {
                        if (isCleanIntegerString(rxBuffer1)) {
                            frontState = atoi(rxBuffer1);
                            frontQuery = Q_FRONT_ERR;
                            Serial1.println("r axis0.error");
                        }
                    } else if (frontQuery == Q_FRONT_ERR) {
                        if (isCleanIntegerString(rxBuffer1)) {
                            frontErr = (uint32_t)strtoul(rxBuffer1, NULL, 10);
                            frontQuery = Q_NONE;
                            Serial.print("📊 [FRONT ODRIVE] Axis0 State: ");
                            Serial.print(frontState == 8 ? "8 (CLOSED_LOOP)" : (String(frontState) + " (IDLE/ERR)"));
                            Serial.print(" | Error: 0x");
                            Serial.println(frontErr, HEX);
                        }
                    }
                }
                rxIndex1 = 0;
            }
        } else {
            if (rxIndex1 < RX_BUFFER_SIZE - 1) {
                rxBuffer1[rxIndex1++] = c;
            } else {
                rxIndex1 = 0;
            }
        }
    }
}

void processSoftSerial() {
    while (odriveRear.available() > 0) {
        char c = (char)odriveRear.read();
        if (c == '\n' || c == '\r') {
            if (rxIndexRear > 0) {
                rxBufferRear[rxIndexRear] = '\0';
                if (rxBufferRear[0] != 'r' && rxBufferRear[0] != 'v' && rxBufferRear[0] != 'w' && rxBufferRear[0] != 'c') {
                    if (rearQuery == Q_REAR_STATE) {
                        if (isCleanIntegerString(rxBufferRear)) {
                            rearState = atoi(rxBufferRear);
                            rearQuery = Q_REAR_ERR;
                            odriveRear.println("r axis0.error");
                        }
                    } else if (rearQuery == Q_REAR_ERR) {
                        if (isCleanIntegerString(rxBufferRear)) {
                            rearErr = (uint32_t)strtoul(rxBufferRear, NULL, 10);
                            rearQuery = Q_NONE;
                            Serial.print("📊 [REAR ODRIVE]  Axis0 State: ");
                            Serial.print(rearState == 8 ? "8 (CLOSED_LOOP)" : (String(rearState) + " (IDLE/ERR)"));
                            Serial.print(" | Error: 0x");
                            Serial.println(rearErr, HEX);
                        }
                    }
                }
                rxIndexRear = 0;
            }
        } else {
            if (rxIndexRear < RX_BUFFER_SIZE - 1) {
                rxBufferRear[rxIndexRear++] = c;
            } else {
                rxIndexRear = 0;
            }
        }
    }
}
