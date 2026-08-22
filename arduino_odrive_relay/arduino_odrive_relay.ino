/*
 * arduino_odrive_relay.ino
 * 
 * Target MCU: Arduino Uno R4 WiFi
 * Purpose: Dual ODrive v3.6 Relay with Reliable HTTP Poll + USB Serial Control.
 * 
 * Architecture:
 *   - USB Serial: Instant char-by-char 'w','s','a','d','x' control (no line-ending dependency)
 *   - HTTP Poll: Arduino polls Mac server (http://192.168.10.140:8000/api/poll) every 100ms
 *   - No UDP, no MQTT. Simple, reliable, and proven to work.
 *   - Hardware Pins: Front ODrive (Pins 7 RX, 6 TX), Rear ODrive (Pins 3 RX, 2 TX)
 */

#include <Arduino.h>
#include <WiFiS3.h>
#include <SoftwareSerial.h>

#define MAC_BAUDRATE 115200
#define ODRIVE_BAUDRATE 19200

// --- WI-FI CREDENTIALS ---
const char* WIFI_SSID = "sanhak";
const char* WIFI_PASS = "20020520";

// --- MAC LOCAL RELAY SERVER ---
const char* MAC_SERVER_IP = "192.168.10.140";
const int   MAC_SERVER_PORT = 8000;

WiFiClient macClient;

// Front ODrive SoftwareSerial on Pins 7 (RX), 6 (TX)
SoftwareSerial odriveFront(7, 6);

// Rear ODrive SoftwareSerial on Pins 3 (RX), 2 (TX)
SoftwareSerial odriveRear(3, 2);

// Timers
unsigned long lastPollMs = 0;
unsigned long lastMoveCommandMs = 0;

// System Control State
bool isSerialControlMode = true;
int driveSpeed = 2;
char lastExecutedKey = ' ';

// Function Prototypes
void connectToWiFi();
void pollMacServer();
void handleSerialInput();
void executeCommand(char key, const char* source);
void moveRobot(int fl, int fr, int rl, int rr);
void stopAllMotors();
void setupSoftwareSerial(SoftwareSerial &odrive);

void setup() {
    Serial.begin(MAC_BAUDRATE);
    delay(500);

    odriveFront.begin(ODRIVE_BAUDRATE);
    odriveRear.begin(ODRIVE_BAUDRATE);
    delay(2000);

    Serial.println("\n--- 옴니휠 키보드 제어 모드 시작 ---");
    Serial.println("ODrive 초기화 중...");

    setupSoftwareSerial(odriveFront);
    setupSoftwareSerial(odriveRear);

    stopAllMotors();

    Serial.println("======================================");
    Serial.println(" 준비 완료! 시리얼 모니터에 입력하세요.");
    Serial.println(" W: 전진 | S: 후진");
    Serial.println(" A: 좌측 게걸음 | D: 우측 게걸음");
    Serial.println(" X: 정지");
    Serial.println("======================================");

    connectToWiFi();
}

void loop() {
    unsigned long now = millis();

    // 1. Instant USB Serial Input
    handleSerialInput();

    // 2. HTTP Poll Mac Server (only when not in serial control mode)
    if (!isSerialControlMode && WiFi.status() == WL_CONNECTED) {
        if (now - lastPollMs >= 100) {
            lastPollMs = now;
            pollMacServer();
        }
    }

    // 3. Deadman watchdog (web mode only)
    if (!isSerialControlMode && (now - lastMoveCommandMs > 500) && lastExecutedKey != 'x') {
        stopAllMotors();
        lastExecutedKey = 'x';
    }
}

/**
 * Instant character-by-character Serial input (matches proven reference code)
 */
void handleSerialInput() {
    while (Serial.available() > 0) {
        char key = (char)Serial.read();
        key = toLowerCase(key);

        if (key == '\r' || key == '\n') continue;

        if (key == 'w' || key == 's' || key == 'a' || key == 'd') {
            isSerialControlMode = true;
            executeCommand(key, "시리얼");
        } 
        else if (key == 'x' || key == ' ') {
            isSerialControlMode = false;
            executeCommand('x', "시리얼");
        } 
        else if (key >= '1' && key <= '9') {
            executeCommand(key, "시리얼");
        }
    }
}

/**
 * Poll Mac Local Relay Server
 */
void pollMacServer() {
    if (!macClient.connect(MAC_SERVER_IP, MAC_SERVER_PORT)) {
        return; // Connection failed, skip this cycle
    }

    macClient.println("GET /api/poll HTTP/1.1");
    macClient.print("Host: "); macClient.println(MAC_SERVER_IP);
    macClient.println("Connection: close");
    macClient.println();

    // Wait for response (max 30ms)
    unsigned long startWait = millis();
    while (!macClient.available() && (millis() - startWait < 30)) {
        delay(1);
    }

    // Read response
    String response = "";
    while (macClient.available()) {
        response += (char)macClient.read();
    }
    macClient.stop();

    // Parse: last line of HTTP response is the command body
    if (response.length() > 0) {
        int lastNewline = response.lastIndexOf('\n');
        if (lastNewline >= 0 && lastNewline < (int)response.length() - 1) {
            String payload = response.substring(lastNewline + 1);
            payload.trim();
            if (payload.length() > 0) {
                char cmdKey = toLowerCase(payload.charAt(0));

                // Skip 'x' from server when in serial mode
                if (isSerialControlMode && (cmdKey == 'x' || cmdKey == 'o')) {
                    return;
                }

                if (cmdKey == 'w' || cmdKey == 's' || cmdKey == 'a' || cmdKey == 'd') {
                    isSerialControlMode = false;
                    executeCommand(cmdKey, "웹");
                } else if (cmdKey == 'x' || cmdKey == 'o') {
                    executeCommand('x', "웹");
                } else if (cmdKey >= '1' && cmdKey <= '9') {
                    executeCommand(cmdKey, "웹");
                }
            }
        }
    }
}

/**
 * Wi-Fi Connection
 */
void connectToWiFi() {
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("❌ Wi-Fi 모듈 없음!");
        return;
    }

    WiFi.disconnect();
    delay(200);

    Serial.print("📶 Wi-Fi 연결 중: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 15) {
        delay(400);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ Wi-Fi 연결 완료!");
        Serial.print("🌐 Arduino IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n⚠️ Wi-Fi 연결 안됨 (시리얼 전용 모드)");
    }
}

/**
 * Central Command Executor
 */
void executeCommand(char key, const char* source) {
    int spd = driveSpeed;

    if (key == 'w') {
        lastMoveCommandMs = millis();
        if (lastExecutedKey != 'w') {
            lastExecutedKey = 'w';
            Serial.print("⬆️ ["); Serial.print(source); Serial.println("] 전진");
        }
        moveRobot(spd, -spd, spd, -spd);
    }
    else if (key == 's') {
        lastMoveCommandMs = millis();
        if (lastExecutedKey != 's') {
            lastExecutedKey = 's';
            Serial.print("⬇️ ["); Serial.print(source); Serial.println("] 후진");
        }
        moveRobot(-spd, spd, -spd, spd);
    }
    else if (key == 'a') {
        lastMoveCommandMs = millis();
        if (lastExecutedKey != 'a') {
            lastExecutedKey = 'a';
            Serial.print("⬅️ ["); Serial.print(source); Serial.println("] 좌측 이동");
        }
        moveRobot(-spd, -spd, spd, spd);
    }
    else if (key == 'd') {
        lastMoveCommandMs = millis();
        if (lastExecutedKey != 'd') {
            lastExecutedKey = 'd';
            Serial.print("➡️ ["); Serial.print(source); Serial.println("] 우측 이동");
        }
        moveRobot(spd, spd, -spd, -spd);
    }
    else if (key == 'x' || key == ' ') {
        if (lastExecutedKey != 'x') {
            lastExecutedKey = 'x';
            Serial.print("🛑 ["); Serial.print(source); Serial.println("] 정지");
        }
        stopAllMotors();
    }
    else if (key >= '1' && key <= '9') {
        driveSpeed = (key - '0') * 2;
        Serial.print("⚙️ ["); Serial.print(source); Serial.print("] 속도: ");
        Serial.println(driveSpeed);
    }
}

/**
 * Omniwheel Kinematics (10ms inter-command delay)
 */
void moveRobot(int fl, int fr, int rl, int rr) {
    odriveFront.print("v 0 "); odriveFront.println(fl);
    delay(10);
    odriveFront.print("v 1 "); odriveFront.println(fr);
    delay(10);
    odriveRear.print("v 1 "); odriveRear.println(rl);
    delay(10);
    odriveRear.print("v 0 "); odriveRear.println(rr);
    delay(10);
}

/**
 * Motor Stop
 */
void stopAllMotors() {
    odriveFront.println("v 0 0"); delay(10);
    odriveFront.println("v 1 0"); delay(10);
    odriveRear.println("v 0 0"); delay(10);
    odriveRear.println("v 1 0"); delay(10);
}

/**
 * ODrive Setup (200ms delays)
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
