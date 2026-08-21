/*
 * arduino_odrive_relay.ino
 * 
 * Target MCU: Arduino Uno R4 WiFi
 * Purpose: Dual ODrive v3.6 Relay with Dual Cloud MQTT (broker.hivemq.com:1883) & Local Web (Port 80).
 * 
 * Multi-Channel Architecture:
 *   1. Cloud MQTT (broker.hivemq.com:1883): Receives WSS commands published from https://omni-wheel-control.onrender.com/ globally over the Internet!
 *   2. Local Web Server (Port 80): 1ms zero-latency control at http://<Arduino_IP>/
 *   3. USB Serial (115200 baud): Real-time cable backup.
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

// --- CLOUD MQTT BROKER ---
const char mqttBroker[] = "broker.hivemq.com";
const int   mqttPort   = 1883;
const char topicCmd[]  = "omniwheel/cmd";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// Local HTTP Web Server on Port 80
WiFiServer localServer(80);

// Rear ODrive SoftwareSerial on Pins 9 (RX), 8 (TX)
SoftwareSerial odriveRear(9, 8);

// 20Hz Encoder Feedback Polling (50ms)
const unsigned long POLLING_INTERVAL_US = 50000;
unsigned long lastPollMicros = 0;

// 20Hz Continuous Velocity Update Loop (50ms)
const unsigned long ROAM_INTERVAL_MS = 50;
unsigned long lastRoamMs = 0;

// 5-Second Status Heartbeat
const unsigned long MQTT_RECONNECT_INTERVAL_MS = 3000;
unsigned long lastMqttConnectAttemptMs = 0;

// Encoder storage array: [Enc0 (FL), Enc1 (FR), Enc2 (RL), Enc3 (RR)]
float encoderPositions[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// System Motion State
bool isAutoRoamEnabled = false;
float currentDriveSpeed = 4.0f; // Velocity turns/sec (Default 4.0 rps = 240 RPM)

// Continuous Target & Current Velocities (Vx, Vy)
float targetVx = 0.0f;
float targetVy = 0.0f;
float currentVx = 0.0f;
float currentVy = 0.0f;

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

// Rx Buffers for line parsing
const size_t RX_BUFFER_SIZE = 128;
char rxBuffer1[RX_BUFFER_SIZE];
size_t rxIndex1 = 0;

char rxBufferRear[RX_BUFFER_SIZE];
size_t rxIndexRear = 0;

uint8_t axisQueryFront = 0;
uint8_t axisQueryRear = 0;

// Function Prototypes
void connectToWiFi();
void pollCloudMQTT();
void handleLocalWebClients();
void setupODriveFront();
void setupODriveRear();
void rearmODrives();
void pollODrives();
void processSerial1();
void processSoftSerial();
void parseFrontLine(const char* line);
void parseRearLine(const char* line);
void handleWebControlInput();
void executeCommand(char key, const char* source);
void updateAutoRoamMotion();
void moveRobotVelocities(float vx, float vy);
void stopAllMotors();

void setup() {
    Serial.begin(MAC_BAUDRATE);
    delay(500);

    Serial1.begin(ODRIVE_BAUDRATE);
    odriveRear.begin(ODRIVE_BAUDRATE);
    delay(1000);

    Serial.println("\n==================================================");
    Serial.println("  🤖 Arduino Uno R4 WiFi - Cloud MQTT Remote System");
    Serial.println("==================================================");

    // Initialize ODrives into Closed Loop Velocity Control
    setupODriveFront();
    setupODriveRear();
    stopAllMotors();

    memset(rxBuffer1, 0, RX_BUFFER_SIZE);
    memset(rxBufferRear, 0, RX_BUFFER_SIZE);

    // Connect to Wi-Fi Internet & Start Local Web Server
    connectToWiFi();

    Serial.println("==================================================");
    Serial.println("▶️ System Ready! Listening on Render, MQTT & USB.");
    Serial.println("==================================================\n");
}

void loop() {
    unsigned long currentMicros = micros();
    unsigned long currentMs = millis();

    // 1. Maintain Cloud MQTT Connection & Parse Render Website Commands
    if (WiFi.status() == WL_CONNECTED) {
        pollCloudMQTT();
        handleLocalWebClients();
    }

    // 2. Real-Time USB Serial Commands
    handleWebControlInput();

    // 3. 20Hz Encoder Feedback Polling
    if (currentMicros - lastPollMicros >= POLLING_INTERVAL_US) {
        lastPollMicros = currentMicros;
        pollODrives();
    }

    // 4. 20Hz CONTINUOUS VELOCITY CONTROL LOOP
    if (currentMs - lastRoamMs >= ROAM_INTERVAL_MS) {
        lastRoamMs = currentMs;
        if (isAutoRoamEnabled) {
            updateAutoRoamMotion();
        } else {
            // Smoothly ramp currentVx, currentVy towards targetVx, targetVy
            currentVx += (targetVx - currentVx) * 0.40f;
            currentVy += (targetVy - currentVy) * 0.40f;
            moveRobotVelocities(currentVx, currentVy);
        }
    }

    // 5. Asynchronously read incoming ODrive response lines
    processSerial1();
    processSoftSerial();
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
 * Connect to Wi-Fi & Start Local Web Server
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
    while (WiFi.status() != WL_CONNECTED && attempts < 15) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ Wi-Fi Internet Connected!");
        Serial.print("🌐 Local Web Controller URL: http://");
        Serial.println(WiFi.localIP());
        localServer.begin();
    } else {
        Serial.println("\n⚠️ Wi-Fi Internet Timeout.");
    }
}

/**
 * Handles Incoming Local Web Server Requests (Port 80)
 */
void handleLocalWebClients() {
    WiFiClient client = localServer.available();
    if (!client) return;

    String request = "";
    while (client.available()) {
        char c = client.read();
        request += c;
        if (c == '\n') break;
    }

    if (request.length() > 0) {
        int getIndex = request.indexOf("GET /");
        if (getIndex != -1) {
            char cmdKey = toLowerCase(request.charAt(getIndex + 5));
            if (cmdKey == 'w' || cmdKey == 's' || cmdKey == 'a' || cmdKey == 'd' || 
                cmdKey == 'x' || cmdKey == 'o' || cmdKey == 'i' || (cmdKey >= '1' && cmdKey <= '9')) {
                executeCommand(cmdKey, "Wi-Fi Web");
            }
        }

        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/plain");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Connection: close");
        client.println("Content-Length: 2\r\n");
        client.println("OK");
        client.flush();
    }
    client.stop();
}

/**
 * Re-arms ODrive motors: clears errors and enforces Closed Loop Mode 8
 */
void rearmODrives() {
    Serial1.println("w axis0.error 0");
    Serial1.println("w axis1.error 0");
    Serial1.println("c 0");
    Serial1.println("c 1");
    Serial1.println("w axis0.requested_state 8");
    Serial1.println("w axis1.requested_state 8");

    odriveRear.println("w axis0.error 0");
    odriveRear.println("w axis1.error 0");
    odriveRear.println("c 0");
    odriveRear.println("c 1");
    odriveRear.println("w axis0.requested_state 8");
    odriveRear.println("w axis1.requested_state 8");
}

/**
 * Central Command Executor
 */
void executeCommand(char key, const char* source) {
    if (key == 'w') {
        isAutoRoamEnabled = false;
        rearmODrives();
        targetVx = 0.0f;
        targetVy = currentDriveSpeed;
        Serial.print("🌐 ["); Serial.print(source); Serial.print(" Received] Cmd: 'W' -> FORWARD (");
        Serial.print(currentDriveSpeed); Serial.println(" turns/s)");
    }
    else if (key == 's') {
        isAutoRoamEnabled = false;
        rearmODrives();
        targetVx = 0.0f;
        targetVy = -currentDriveSpeed;
        Serial.print("🌐 ["); Serial.print(source); Serial.print(" Received] Cmd: 'S' -> BACKWARD (-");
        Serial.print(currentDriveSpeed); Serial.println(" turns/s)");
    }
    else if (key == 'a') {
        isAutoRoamEnabled = false;
        rearmODrives();
        targetVx = -currentDriveSpeed;
        targetVy = 0.0f;
        Serial.print("🌐 ["); Serial.print(source); Serial.print(" Received] Cmd: 'A' -> LEFT STRAFE (-");
        Serial.print(currentDriveSpeed); Serial.println(" turns/s)");
    }
    else if (key == 'd') {
        isAutoRoamEnabled = false;
        rearmODrives();
        targetVx = currentDriveSpeed;
        targetVy = 0.0f;
        Serial.print("🌐 ["); Serial.print(source); Serial.print(" Received] Cmd: 'D' -> RIGHT STRAFE (");
        Serial.print(currentDriveSpeed); Serial.println(" turns/s)");
    }
    else if (key == 'x' || key == 'o' || key == ' ') {
        isAutoRoamEnabled = false;
        targetVx = 0.0f;
        targetVy = 0.0f;
        stopAllMotors();
        Serial.print("🛑 ["); Serial.print(source); Serial.println(" Received] Cmd: STOP ALL");
    }
    else if (key == 'i') {
        isAutoRoamEnabled = true;
        rearmODrives();
        stateStartTime = millis();
        Serial.print("🤖 ["); Serial.print(source); Serial.println(" Received] Cmd: AUTO ROAM [I]");
    }
    else if (key >= '1' && key <= '9') {
        currentDriveSpeed = 1.0f + (key - '1') * 1.5f;
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

void stopAllMotors() {
    targetVx = 0.0f;
    targetVy = 0.0f;
    currentVx = 0.0f;
    currentVy = 0.0f;

    Serial1.println("v 0 0");
    Serial1.println("v 1 0");
    odriveRear.println("v 0 0");
    odriveRear.println("v 1 0");
}

void setupODriveFront() {
    Serial1.println("w axis0.error 0"); delay(50);
    Serial1.println("w axis1.error 0"); delay(50);
    Serial1.println("c 0"); delay(50);
    Serial1.println("c 1"); delay(50);
    Serial1.println("w axis0.requested_state 1"); delay(50);
    Serial1.println("w axis0.controller.config.control_mode 2"); delay(50);
    Serial1.println("w axis0.controller.config.input_mode 1"); delay(50);
    Serial1.println("w axis0.requested_state 8"); delay(50);

    Serial1.println("w axis1.requested_state 1"); delay(50);
    Serial1.println("w axis1.controller.config.control_mode 2"); delay(50);
    Serial1.println("w axis1.controller.config.input_mode 1"); delay(50);
    Serial1.println("w axis1.requested_state 8"); delay(50);
}

void setupODriveRear() {
    odriveRear.println("w axis0.error 0"); delay(50);
    odriveRear.println("w axis1.error 0"); delay(50);
    odriveRear.println("c 0"); delay(50);
    odriveRear.println("c 1"); delay(50);
    odriveRear.println("w axis0.requested_state 1"); delay(50);
    odriveRear.println("w axis0.controller.config.control_mode 2"); delay(50);
    odriveRear.println("w axis0.controller.config.input_mode 1"); delay(50);
    odriveRear.println("w axis0.requested_state 8"); delay(50);

    odriveRear.println("w axis1.requested_state 1"); delay(50);
    odriveRear.println("w axis1.controller.config.control_mode 2"); delay(50);
    odriveRear.println("w axis1.controller.config.input_mode 1"); delay(50);
    odriveRear.println("w axis1.requested_state 8"); delay(50);
}

void pollODrives() {
    Serial1.print("f 0\n");
    Serial1.print("f 1\n");

    odriveRear.print("f 1\n");
    odriveRear.print("f 0\n");
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

void processSerial1() {
    while (Serial1.available() > 0) {
        char c = (char)Serial1.read();
        if (c == '\n' || c == '\r') {
            if (rxIndex1 > 0) {
                rxBuffer1[rxIndex1] = '\0';
                parseFrontLine(rxBuffer1);
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
                parseRearLine(rxBufferRear);
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

void parseFrontLine(const char* line) {
    if (line == nullptr || line[0] == '\0') return;
    float pos = 0.0f, vel = 0.0f;
    if (sscanf(line, "%f %f", &pos, &vel) >= 1) {
        encoderPositions[axisQueryFront] = pos;
        axisQueryFront = (axisQueryFront + 1) % 2;
    } else {
        Serial.print("💬 [ODrive Front Response] ");
        Serial.println(line);
    }
}

void parseRearLine(const char* line) {
    if (line == nullptr || line[0] == '\0') return;
    float pos = 0.0f, vel = 0.0f;
    if (sscanf(line, "%f %f", &pos, &vel) >= 1) {
        if (axisQueryRear == 0) {
            encoderPositions[2] = pos;
        } else {
            encoderPositions[3] = pos;
        }
        axisQueryRear = (axisQueryRear + 1) % 2;
    } else {
        Serial.print("💬 [ODrive Rear Response] ");
        Serial.println(line);
    }
}
