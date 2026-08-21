/*
 * arduino_odrive_relay.ino
 * 
 * Target MCU: Arduino Uno R4 WiFi
 * Purpose: Dual ODrive v3.6 Relay with Supabase Realtime & REST API Cloud Integration.
 * 
 * Architecture:
 *   - Wi-Fi Connection via WiFiS3
 *   - Supabase HTTPS REST API via WiFiSSLClient (Port 443)
 *   - Command Endpoint: GET /rest/v1/robot_command?id=eq.1&select=cmd
 *   - Telemetry Endpoint: PATCH /rest/v1/robot_telemetry?id=eq.1
 *   - Backup USB Serial CDC @ 115200 baud
 */

#include <Arduino.h>
#include <WiFiS3.h>
#include <WiFiSSLClient.h>
#include <SoftwareSerial.h>

#define MAC_BAUDRATE 115200
#define ODRIVE_BAUDRATE 19200

// --- WI-FI CREDENTIALS ---
const char* WIFI_SSID = "sanhak";      // Wi-Fi SSID
const char* WIFI_PASS = "20020520";  // Wi-Fi Password

// --- SUPABASE CLOUD CONFIGURATION ---
// Enter your Supabase Project URL and Anon Key below:
const char* SUPABASE_HOST     = "your-project-id.supabase.co"; // e.g. "xyz123.supabase.co"
const char* SUPABASE_ANON_KEY = "YOUR_SUPABASE_ANON_KEY";       // Your Supabase Anon Public Key
const int   SUPABASE_PORT     = 443;

WiFiSSLClient sslClient;

// Rear ODrive SoftwareSerial on Pins 9 (RX), 8 (TX)
SoftwareSerial odriveRear(9, 8);

// 50Hz Encoder Polling (20,000 microseconds)
const unsigned long POLLING_INTERVAL_US = 20000;
unsigned long lastPollMicros = 0;

// Supabase Cloud Command Polling (300ms)
const unsigned long SUPABASE_POLL_INTERVAL_MS = 300;
unsigned long lastSupabasePollMs = 0;

// 20Hz Auto Motion Update Loop (50ms)
const unsigned long ROAM_INTERVAL_MS = 50;
unsigned long lastRoamMs = 0;

// 5-Second Status Heartbeat
const unsigned long STATUS_INTERVAL_MS = 5000;
unsigned long lastStatusMs = 0;

// Encoder storage array: [Enc0 (FL), Enc1 (FR), Enc2 (RL), Enc3 (RR)]
float encoderPositions[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// System Motion State
bool isAutoRoamEnabled = false;
float currentDriveSpeed = 2.0f;
char lastExecutedCmd = ' ';

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
float currentVx = 0.0f;
float currentVy = 0.0f;

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
void pollSupabaseCloud();
void printStatusHeartbeat();
void setupODriveFront();
void setupODriveRear();
void pollODrives();
void processSerial1();
void processSoftSerial();
void parseFrontLine(const char* line);
void parseRearLine(const char* line);
void sendCSVToMac();
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
    Serial.println("  ⚡ Arduino Uno R4 WiFi - Supabase Cloud System  ");
    Serial.println("==================================================");

    // Initialize ODrives
    setupODriveFront();
    setupODriveRear();
    stopAllMotors();

    memset(rxBuffer1, 0, RX_BUFFER_SIZE);
    memset(rxBufferRear, 0, RX_BUFFER_SIZE);

    // Connect to Wi-Fi Internet
    connectToWiFi();

    Serial.println("==================================================");
    Serial.println("▶️ Ready! Controlling via Supabase Cloud & USB Serial...");
    Serial.println("==================================================\n");
}

void loop() {
    unsigned long currentMicros = micros();
    unsigned long currentMs = millis();

    // 1. Poll Supabase Cloud Commands over HTTPS (every 300ms)
    if (WiFi.status() == WL_CONNECTED && (currentMs - lastSupabasePollMs >= SUPABASE_POLL_INTERVAL_MS)) {
        lastSupabasePollMs = currentMs;
        pollSupabaseCloud();
    }

    // 2. Process USB Serial Backup Commands
    handleWebControlInput();

    // 3. 50Hz Non-blocking Encoder Feedback
    if (currentMicros - lastPollMicros >= POLLING_INTERVAL_US) {
        lastPollMicros = currentMicros;
        pollODrives();
        sendCSVToMac();
    }

    // 4. 20Hz Auto-Roam Motion
    if (isAutoRoamEnabled && (currentMs - lastRoamMs >= ROAM_INTERVAL_MS)) {
        lastRoamMs = currentMs;
        updateAutoRoamMotion();
    }

    // 5. 5-Second Status Heartbeat
    if (currentMs - lastStatusMs >= STATUS_INTERVAL_MS) {
        lastStatusMs = currentMs;
        printStatusHeartbeat();
    }

    // 6. Asynchronously read incoming ODrive responses
    processSerial1();
    processSoftSerial();
}

/**
 * Wi-Fi Internet Connection Setup
 */
void connectToWiFi() {
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("❌ Error: Wi-Fi module not detected on Uno R4!");
        return;
    }

    Serial.print("📶 Connecting Wi-Fi SSID: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ Wi-Fi Internet Connected!");
        Serial.print("🌐 Assigned IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n⚠️ Wi-Fi Internet Timeout.");
    }
}

/**
 * Polls Supabase Database HTTPS REST API for live commands
 * GET /rest/v1/robot_command?id=eq.1&select=cmd
 */
void pollSupabaseCloud() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (String(SUPABASE_HOST).startsWith("your-project")) return; // Skip if default placeholder

    if (sslClient.connect(SUPABASE_HOST, SUPABASE_PORT)) {
        // Build Supabase HTTPS GET request
        sslClient.print("GET /rest/v1/robot_command?id=eq.1&select=cmd HTTP/1.1\r\n");
        sslClient.print("Host: "); sslClient.print(SUPABASE_HOST); sslClient.print("\r\n");
        sslClient.print("apikey: "); sslClient.print(SUPABASE_ANON_KEY); sslClient.print("\r\n");
        sslClient.print("Authorization: Bearer "); sslClient.print(SUPABASE_ANON_KEY); sslClient.print("\r\n");
        sslClient.print("Connection: close\r\n\r\n");

        String response = "";
        unsigned long timeout = millis();
        while (sslClient.connected() && millis() - timeout < 1000) {
            while (sslClient.available()) {
                char c = sslClient.read();
                response += c;
            }
        }
        sslClient.stop();

        // Parse JSON response: [{"cmd":"w"}]
        int cmdIndex = response.indexOf("\"cmd\":\"");
        if (cmdIndex != -1) {
            char cmdKey = toLowerCase(response.charAt(cmdIndex + 7));
            if (cmdKey != lastExecutedCmd) {
                lastExecutedCmd = cmdKey;
                executeCommand(cmdKey, "Supabase Cloud");
            }
        }
    }
}

/**
 * Central Command Executor
 */
void executeCommand(char key, const char* source) {
    if (key == 'w') {
        isAutoRoamEnabled = false;
        moveRobotVelocities(0.0f, currentDriveSpeed);
        Serial.print("⚡ ["); Serial.print(source); Serial.println("] GET /w -> FORWARD");
    }
    else if (key == 's') {
        isAutoRoamEnabled = false;
        moveRobotVelocities(0.0f, -currentDriveSpeed);
        Serial.print("⚡ ["); Serial.print(source); Serial.println("] GET /s -> BACKWARD");
    }
    else if (key == 'a') {
        isAutoRoamEnabled = false;
        moveRobotVelocities(-currentDriveSpeed, 0.0f);
        Serial.print("⚡ ["); Serial.print(source); Serial.println("] GET /a -> LEFT STRAFE");
    }
    else if (key == 'd') {
        isAutoRoamEnabled = false;
        moveRobotVelocities(currentDriveSpeed, 0.0f);
        Serial.print("⚡ ["); Serial.print(source); Serial.println("] GET /d -> RIGHT STRAFE");
    }
    else if (key == 'x' || key == 'o' || key == ' ') {
        isAutoRoamEnabled = false;
        stopAllMotors();
        Serial.print("⚡ ["); Serial.print(source); Serial.println("] GET /x -> STOP ALL");
    }
    else if (key == 'i') {
        isAutoRoamEnabled = true;
        stateStartTime = millis();
        Serial.print("⚡ ["); Serial.print(source); Serial.println("] GET /i -> AUTO ROAM");
    }
    else if (key >= '1' && key <= '9') {
        currentDriveSpeed = 1.0f + (key - '1') * 0.45f;
        Serial.print("⚡ ["); Serial.print(source); Serial.print("] Speed Level: ");
        Serial.println(currentDriveSpeed);
    }
}

void handleWebControlInput() {
    while (Serial.available() > 0) {
        char key = toLowerCase((char)Serial.read());
        executeCommand(key, "USB Serial");
    }
}

void printStatusHeartbeat() {
    Serial.print("💬 [HEARTBEAT ");
    Serial.print(millis() / 1000);
    Serial.print("s] Wi-Fi: ");
    Serial.print(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
    Serial.print(" | Mode: ");
    Serial.print(isAutoRoamEnabled ? "AUTO" : "MANUAL");
    Serial.print(" | Encoders: [");
    Serial.print(encoderPositions[0], 1); Serial.print(",");
    Serial.print(encoderPositions[1], 1); Serial.print(",");
    Serial.print(encoderPositions[2], 1); Serial.print(",");
    Serial.print(encoderPositions[3], 1);
    Serial.println("]");
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
    Serial1.println("v 0 0");
    Serial1.println("v 1 0");
    odriveRear.println("v 0 0");
    odriveRear.println("v 1 0");
    currentVx = 0.0f;
    currentVy = 0.0f;
}

void setupODriveFront() {
    Serial1.println("c 0"); delay(100);
    Serial1.println("w axis0.requested_state 1"); delay(100);
    Serial1.println("w axis0.controller.config.control_mode 2"); delay(100);
    Serial1.println("w axis0.controller.config.input_mode 1"); delay(100);
    Serial1.println("w axis0.requested_state 8"); delay(100);

    Serial1.println("c 1"); delay(100);
    Serial1.println("w axis1.requested_state 1"); delay(100);
    Serial1.println("w axis1.controller.config.control_mode 2"); delay(100);
    Serial1.println("w axis1.controller.config.input_mode 1"); delay(100);
    Serial1.println("w axis1.requested_state 8"); delay(100);
}

void setupODriveRear() {
    odriveRear.println("c 0"); delay(100);
    odriveRear.println("w axis0.requested_state 1"); delay(100);
    odriveRear.println("w axis0.controller.config.control_mode 2"); delay(100);
    odriveRear.println("w axis0.controller.config.input_mode 1"); delay(100);
    odriveRear.println("w axis0.requested_state 8"); delay(100);

    odriveRear.println("c 1"); delay(100);
    odriveRear.println("w axis1.requested_state 1"); delay(100);
    odriveRear.println("w axis1.controller.config.control_mode 2"); delay(100);
    odriveRear.println("w axis1.controller.config.input_mode 1"); delay(100);
    odriveRear.println("w axis1.requested_state 8"); delay(100);
}

void pollODrives() {
    Serial1.print("f 0\n");
    Serial1.print("f 1\n");

    odriveRear.print("f 1\n");
    odriveRear.print("f 0\n");
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
    }
}

void sendCSVToMac() {
    Serial.print(encoderPositions[0], 4);
    Serial.print(",");
    Serial.print(encoderPositions[1], 4);
    Serial.print(",");
    Serial.print(encoderPositions[2], 4);
    Serial.print(",");
    Serial.println(encoderPositions[3], 4);
}
