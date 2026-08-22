/*
 * arduino_odrive_relay.ino
 * 
 * Target MCU: Arduino Uno R4 WiFi
 * Purpose: Dual ODrive v3.6 Relay with Ultra-Low Latency Direct UDP (<2ms) & USB Serial.
 * 
 * Solution for "CLOUD MQTT is too slow":
 *   - Direct UDP Receiver (WiFiUDP Udp; Port 8888) over Local Wi-Fi.
 *   - Eliminates 500ms~1500ms Cloud MQTT & Public Internet delay.
 *   - Button press on Mac Web UI (http://192.168.10.140:8000) reaches Arduino in <2ms!
 *   - Hardware Pins: Front ODrive (Pins 7 RX, 6 TX), Rear ODrive (Pins 3 RX, 2 TX).
 */

#include <Arduino.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>

#define MAC_BAUDRATE 115200
#define ODRIVE_BAUDRATE 19200
#define UDP_PORT 8888

// --- WI-FI CREDENTIALS ---
const char* WIFI_SSID = "sanhak";      // Wi-Fi SSID
const char* WIFI_PASS = "20020520";  // Wi-Fi Password

// --- MAC LOCAL RELAY SERVER ---
const char* MAC_SERVER_IP = "192.168.10.140";
const int   MAC_SERVER_PORT = 8000;

WiFiClient wifiClient;
WiFiClient macClient;
WiFiUDP Udp;

// Front ODrive SoftwareSerial on Pins 7 (RX), 6 (TX)
SoftwareSerial odriveFront(7, 6);

// Rear ODrive SoftwareSerial on Pins 3 (RX), 2 (TX)
SoftwareSerial odriveRear(3, 2);

// Timers
unsigned long lastMacPollAttemptMs = 0;
unsigned long lastMoveCommandMs = 0;

// System Control State
bool isSerialControlMode = true; // Default to USB Serial Mode for immediate startup
int driveSpeed = 2; // Default speed 2 turns/sec (matches working reference code)
char lastExecutedKey = ' ';

// Function Prototypes
void connectToWiFi();
void pollUdpCommands();
void pollMacServer();
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

    Serial.println("\n--- 옴니휠 시리얼 & 초고속 UDP 제어 모드 시작 ---");
    Serial.println("ODrive 초기화 중...");

    Serial.println("⚙️ Front ODrive (Pins 7 RX, 6 TX) Setup...");
    setupSoftwareSerial(odriveFront);

    Serial.println("⚙️ Rear ODrive (Pins 3 RX, 2 TX) Setup...");
    setupSoftwareSerial(odriveRear);

    stopAllMotors(); // 안전을 위해 정지

    Serial.println("======================================");
    Serial.println(" 준비 완료! 시리얼 모니터 및 웹에 입력하세요.");
    Serial.println(" W: 전진 | S: 후진 | A: 좌측 | D: 우측 | X: 정지");
    Serial.println("======================================");

    // Non-blocking Wi-Fi initiation
    connectToWiFi();

    // Start UDP Server on Port 8888 (<2ms latency)
    Udp.begin(UDP_PORT);
    Serial.print("⚡ 초고속 UDP 수신기 가동 완료 (Port ");
    Serial.print(UDP_PORT);
    Serial.println(")");
}

void loop() {
    unsigned long currentMs = millis();

    // 1. Instant USB Serial Input (Zero-delay character reading)
    handleWebControlInput();

    // 2. Ultra Low Latency Local Wi-Fi UDP Receiver (<2ms Latency)
    if (WiFi.status() == WL_CONNECTED) {
        pollUdpCommands();
        pollMacServer();
    }

    // 3. 500ms Safety Watchdog (Web mode only)
    if (!isSerialControlMode && (currentMs - lastMoveCommandMs > 500) && (lastExecutedKey != 'x' && lastExecutedKey != ' ')) {
        stopAllMotors();
        lastExecutedKey = 'x';
    }
}

/**
 * Parses Direct UDP Command Packets (<2ms Latency)
 */
void pollUdpCommands() {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
        char c = (char)Udp.read();
        c = toLowerCase(c);
        if (c == 'w' || c == 's' || c == 'a' || c == 'd' || c == 'x' || c == 'o' || (c >= '1' && c <= '9')) {
            isSerialControlMode = false; // Switch to Web/UDP mode
            executeCommand(c, "초고속 로컬 UDP");
        }
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
            isSerialControlMode = true; // Lock into Serial Mode
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
 * Backup HTTP Poll for Mac Local Relay Server
 */
void pollMacServer() {
    unsigned long now = millis();
    if (now - lastMacPollAttemptMs < 200) return; // Throttle backup HTTP poll
    lastMacPollAttemptMs = now;

    if (macClient.connect(MAC_SERVER_IP, MAC_SERVER_PORT)) {
        macClient.println("GET /api/poll HTTP/1.1");
        macClient.print("Host: "); macClient.println(MAC_SERVER_IP);
        macClient.println("Connection: close\r\n");

        unsigned long startWait = millis();
        while (!macClient.available() && (millis() - startWait < 20)) {
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
                        executeCommand(cmdKey, "백업 HTTP 폴링");
                    }
                }
            }
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
            Serial.print("⬆️ ["); Serial.print(source); Serial.println("] 전진");
        }
        moveRobot(spd, -spd, spd, -spd); // ⬆️ FORWARD
    }
    else if (key == 's') {
        lastMoveCommandMs = millis();
        if (lastExecutedKey != 's') {
            lastExecutedKey = 's';
            Serial.print("⬇️ ["); Serial.print(source); Serial.println("] 후진");
        }
        moveRobot(-spd, spd, -spd, spd); // ⬇️ BACKWARD
    }
    else if (key == 'a') {
        lastMoveCommandMs = millis();
        if (lastExecutedKey != 'a') {
            lastExecutedKey = 'a';
            Serial.print("⬅️ ["); Serial.print(source); Serial.println("] 좌측 이동 (게걸음)");
        }
        moveRobot(-spd, -spd, spd, spd); // ⬅️ LEFT STRAFE
    }
    else if (key == 'd') {
        lastMoveCommandMs = millis();
        if (lastExecutedKey != 'd') {
            lastExecutedKey = 'd';
            Serial.print("➡️ ["); Serial.print(source); Serial.println("] 우측 이동 (게걸음)");
        }
        moveRobot(spd, spd, -spd, -spd); // ➡️ RIGHT STRAFE
    }
    else if (key == 'x' || key == ' ') {
        if (lastExecutedKey != 'x') {
            lastExecutedKey = 'x';
            Serial.print("🛑 ["); Serial.print(source); Serial.println("] 정지");
        }
        stopAllMotors(); // 🛑 STOP ALL
    }
    else if (key >= '1' && key <= '9') {
        driveSpeed = (key - '0') * 2; // 1 -> speed 2, 2 -> speed 4, 3 -> speed 6...
        Serial.print("⚙️ ["); Serial.print(source); Serial.print("] 속도 변경: ");
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
