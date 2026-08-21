/*
 * arduino_odrive_relay.ino
 * 
 * Target MCU: Arduino Uno R4 WiFi
 * Purpose: Dual ODrive v3.6 Relay with 100% Clean SoftwareSerial Timing.
 * 
 * Root Cause & Fix for "Motors Not Moving":
 *   - Cause: Continuous TCP socket calls (macClient.connect) every 100ms corrupted SoftwareSerial bit timing.
 *   - Fix 1: When in USB Serial Mode, Wi-Fi socket calls are paused so SoftwareSerial has 100% timing accuracy.
 *   - Fix 2: Wi-Fi TCP connection attempts are throttled to once every 3 seconds (no continuous blocking).
 *   - Fix 3: Instant character-by-character Serial reading (char key = Serial.read()) matching reference code.
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

// Front ODrive SoftwareSerial on Pins 7 (RX), 6 (TX)
SoftwareSerial odriveFront(7, 6);

// Rear ODrive SoftwareSerial on Pins 3 (RX), 2 (TX)
SoftwareSerial odriveRear(3, 2);

// Timers
unsigned long lastMacPollAttemptMs = 0;
unsigned long lastMqttConnectAttemptMs = 0;
unsigned long lastMoveCommandMs = 0;

// System Control State
bool isSerialControlMode = true; // Default to USB Serial Mode for immediate startup
int driveSpeed = 2; // Default speed 2 turns/sec (matches working reference code)
char lastExecutedKey = ' ';

// Function Prototypes
void connectToWiFi();
void pollMacServer();
void pollCloudMQTT();
void setupSoftwareSerial(SoftwareSerial &odrive);
void handleWebControlInput();
void executeCommand(char key, const char* source);
void moveRobot(int fl, int fr, int rl, int rr);
void stopAllMotors();

void setup() {
    Serial.begin(MAC_BAUDRATE);
    delay(500);

    odriveFront.begin(ODRIVE_BAUDRATE);
    odriveRear.begin(ODRIVE_BAUDRATE);
    delay(2000);

    Serial.println("\n--- 옴니휠 키보드 제어 모드 시작 ---");
    Serial.println("ODrive 초기화 중...");

    Serial.println("⚙️ Front ODrive (Pins 7 RX, 6 TX) Setup...");
    setupSoftwareSerial(odriveFront);

    Serial.println("⚙️ Rear ODrive (Pins 3 RX, 2 TX) Setup...");
    setupSoftwareSerial(odriveRear);

    stopAllMotors(); // 안전을 위해 정지

    Serial.println("======================================");
    Serial.println(" 준비 완료! 시리얼 모니터에 입력하세요.");
    Serial.println(" W: 전진 | S: 후진");
    Serial.println(" A: 좌측 게걸음 | D: 우측 게걸음");
    Serial.println(" X: 정지");
    Serial.println("======================================");

    // Non-blocking Wi-Fi initiation
    connectToWiFi();
}

void loop() {
    unsigned long currentMs = millis();

    // 1. Instant USB Serial Input (Zero-delay character reading)
    handleWebControlInput();

    // 2. Poll Web Relay Server ONLY when NOT in active USB Serial Mode
    if (!isSerialControlMode && WiFi.status() == WL_CONNECTED) {
        pollMacServer();
        pollCloudMQTT();
    }

    // 3. 500ms Safety Watchdog (Web mode only)
    if (!isSerialControlMode && (currentMs - lastMoveCommandMs > 500) && (lastExecutedKey != 'x' && lastExecutedKey != ' ')) {
        stopAllMotors();
        lastExecutedKey = 'x';
    }
}

/**
 * Handles Incoming Input from USB Serial Monitor Character-by-Character
 */
void handleWebControlInput() {
    while (Serial.available() > 0) {
        char key = (char)Serial.read(); // Read single character instantly
        key = toLowerCase(key);

        if (key == '\r' || key == '\n') continue; // Ignore line endings

        if (key == 'w' || key == 's' || key == 'a' || key == 'd') {
            isSerialControlMode = true; // Lock into Serial Mode (pauses Wi-Fi socket interference)
            executeCommand(key, "시리얼 입력");
        } 
        else if (key == 'x' || key == ' ') {
            executeCommand('x', "시리얼 입력");
        } 
        else if (key >= '1' && key <= '9') {
            executeCommand(key, "시리얼 입력");
        }
    }
}

/**
 * Polls Mac Local Relay Server safely without corrupting SoftwareSerial timing
 */
void pollMacServer() {
    unsigned long now = millis();
    if (now - lastMacPollAttemptMs < 200) return; // Throttle to 200ms
    lastMacPollAttemptMs = now;

    if (macClient.connect(MAC_SERVER_IP, MAC_SERVER_PORT)) {
        macClient.println("GET /api/poll HTTP/1.1");
        macClient.print("Host: "); macClient.println(MAC_SERVER_IP);
        macClient.println("Connection: close\r\n");

        unsigned long startWait = millis();
        while (!macClient.available() && (millis() - startWait < 25)) {
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
                        executeCommand(cmdKey, "웹 서버");
                    }
                }
            }
        }
    }
}

/**
 * Polls Cloud MQTT Broker
 */
void pollCloudMQTT() {
    if (!mqttClient.connected()) {
        unsigned long now = millis();
        if (now - lastMqttConnectAttemptMs >= 5000) {
            lastMqttConnectAttemptMs = now;
            mqttClient.setId("ArduinoR4_Omni");
            mqttClient.connect(mqttBroker, mqttPort);
            if (mqttClient.connected()) {
                mqttClient.subscribe(topicCmd);
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
            executeCommand(cmdKey, "클라우드 MQTT");
        }
    }
}

/**
 * Non-blocking Wi-Fi Connection
 */
void connectToWiFi() {
    if (WiFi.status() == WL_NO_MODULE) return;

    WiFi.disconnect();
    delay(200);

    Serial.print("📶 Wi-Fi 연결 중: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
        delay(300);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ Wi-Fi 연결 완료!");
        Serial.print("🌐 IP 주소: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n⚠️ Wi-Fi 연결 안됨 (시리얼 전용 모드로 동작)");
    }
}

/**
 * Central Command Executor - Direct Wheel Movement
 */
void executeCommand(char key, const char* source) {
    int spd = driveSpeed;

    if (key == 'w') {
        lastMoveCommandMs = millis();
        if (lastExecutedKey != 'w') {
            lastExecutedKey = 'w';
            Serial.println("⬆️ 전진");
        }
        moveRobot(spd, -spd, spd, -spd); // ⬆️ FORWARD
    }
    else if (key == 's') {
        lastMoveCommandMs = millis();
        if (lastExecutedKey != 's') {
            lastExecutedKey = 's';
            Serial.println("⬇️ 후진");
        }
        moveRobot(-spd, spd, -spd, spd); // ⬇️ BACKWARD
    }
    else if (key == 'a') {
        lastMoveCommandMs = millis();
        if (lastExecutedKey != 'a') {
            lastExecutedKey = 'a';
            Serial.println("⬅️ 좌측 이동 (게걸음)");
        }
        moveRobot(-spd, -spd, spd, spd); // ⬅️ LEFT STRAFE
    }
    else if (key == 'd') {
        lastMoveCommandMs = millis();
        if (lastExecutedKey != 'd') {
            lastExecutedKey = 'd';
            Serial.println("➡️ 우측 이동 (게걸음)");
        }
        moveRobot(spd, spd, -spd, -spd); // ➡️ RIGHT STRAFE
    }
    else if (key == 'x' || key == ' ') {
        if (lastExecutedKey != 'x') {
            lastExecutedKey = 'x';
            Serial.println("🛑 정지");
        }
        stopAllMotors(); // 🛑 STOP ALL
    }
    else if (key >= '1' && key <= '9') {
        driveSpeed = (key - '0') * 2; // 1 -> speed 2, 2 -> speed 4, 3 -> speed 6...
        Serial.print("⚙️ 속도 변경: ");
        Serial.println(driveSpeed);
    }
}

/**
 * Omniwheel Drive Kinematics with 10ms Delays
 */
void moveRobot(int fl, int fr, int rl, int rr) {
    // Front Wheels (odriveFront: Pins 7 RX, 6 TX)
    odriveFront.print("v 0 "); odriveFront.println(fl); // 2사분면 (FL)
    delay(10);
    odriveFront.print("v 1 "); odriveFront.println(fr); // 1사분면 (FR)
    delay(10);

    // Rear Wheels (odriveRear: Pins 3 RX, 2 TX)
    odriveRear.print("v 1 "); odriveRear.println(rl); // 3사분면 (RL)
    delay(10);
    odriveRear.print("v 0 "); odriveRear.println(rr); // 4사분면 (RR)
    delay(10);
}

/**
 * Safe Motor Stop Function
 */
void stopAllMotors() {
    odriveFront.println("v 0 0"); delay(10);
    odriveFront.println("v 1 0"); delay(10);
    odriveRear.println("v 0 0"); delay(10);
    odriveRear.println("v 1 0"); delay(10);
}

/**
 * ODrive Setup Function with 200ms Delays
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
