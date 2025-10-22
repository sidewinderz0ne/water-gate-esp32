#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

// Pin Definitions for ESP32
// Main A01NYUB Sensor (Primary water level)
#define MAIN_A01_RX 16 // GPIO16
#define MAIN_A01_TX 17 // GPIO17

// Secondary A01NYUB Sensor (Secondary water level)
#define SECONDARY_A01_RX 25 // GPIO25
#define SECONDARY_A01_TX 26 // GPIO26

// SHARED PWM PIN for all gates (since speed control not needed)
#define SHARED_PWM 14  // Single PWM pin for all BTS7960 drivers

// BTS7960 Motor Driver Pins for 4 Gates
// Gate 1
#define GATE1_REN 21
#define GATE1_LEN 22

// Gate 2
#define GATE2_REN 32
#define GATE2_LEN 33

// Gate 3
#define GATE3_REN 2
#define GATE3_LEN 4

// Gate 4
#define GATE4_REN 34
#define GATE4_LEN 35

// Constants
#define WDT_TIMEOUT 180
#define CONFIG_FILE "/config.json"
#define SERIAL_BUFFER_SIZE 20
#define NUM_GATES 4

String serialBuff[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// Global Variables
WebServer server(80);
unsigned long lastMeasurementTime = 0;
float mainWaterLevel = 0.0;
float secondaryWaterLevel = 0.0;
float mainRawDistance = 0.0;
float secondaryRawDistance = 0.0;

// Gate control structure
struct Gate {
    String name;
    bool enabled;           // Is this gate active in auto mode?
    bool isOpen;           // Current state (true=open, false=closed)
    bool isMoving;         // Is currently moving
    int renPin;            // Right enable pin
    int lenPin;            // Left enable pin
    
    Gate() : name("Gate"), enabled(false), isOpen(false), isMoving(false),
             renPin(0), lenPin(0) {}
};

Gate gates[NUM_GATES];
bool autoMode = false;
unsigned long lastAutoActionTime = 0;

// Configuration structure
struct Config {
    String stationName;
    unsigned long measurementInterval;
    
    // Main sensor configuration
    float mainCalibrationOffset;
    float mainSensorToBottomDistance;
    float mainUpperTarget;
    float mainLowerTarget;
    
    // Secondary sensor configuration
    float secondaryCalibrationOffset;
    float secondarySensorToBottomDistance;
    
    // Gate operation settings
    unsigned int gateOperationDelay;
    bool autoModeEnabled;
    
    Config() : stationName("Water Gate Controller"),
               measurementInterval(30),
               mainCalibrationOffset(0.0),
               mainSensorToBottomDistance(300.0),
               mainUpperTarget(-40.0),
               mainLowerTarget(-60.0),
               secondaryCalibrationOffset(0.0),
               secondarySensorToBottomDistance(300.0),
               gateOperationDelay(60),
               autoModeEnabled(false) {}
} config;

unsigned long startTime = 0;
const unsigned long MINIMUM_INTERVAL_SECONDS  = 5;

// Function declarations
bool setupLittleFS();
void setupWiFi();
void setupWebServer();
void setupPins();
void handleRoot();
void handleGetConfig();
void handleSettings();
void handleCurrentLevel();
void handleGateControl();
void measureWaterLevels();
String getFormattedDateTime();
bool loadConfig();
bool saveConfig();
void createDefaultConfig();
void initWatchdog();
void resetWatchdog();
void addToSerialBuffer(const String &message);
void handleRestart();
void handleUptime();
void validateMeasurementInterval();
float readA01NYUB(HardwareSerial &serial, int rx, int tx, const char* sensorName);
void controlAutoGates();
void openGate(int gateIndex);
void closeGate(int gateIndex);
void stopGate(int gateIndex);
bool shouldOpenGates();
bool shouldCloseGates();
void initializeGates();
template<typename T>
bool getIfPresent(JsonObject obj, const char* key, T& out);

void initializeGates()
{
    // Gate 1
    gates[0].name = "Gate 1";
    gates[0].renPin = GATE1_REN;
    gates[0].lenPin = GATE1_LEN;
    
    // Gate 2
    gates[1].name = "Gate 2";
    gates[1].renPin = GATE2_REN;
    gates[1].lenPin = GATE2_LEN;
    
    // Gate 3
    gates[2].name = "Gate 3";
    gates[2].renPin = GATE3_REN;
    gates[2].lenPin = GATE3_LEN;
    
    // Gate 4
    gates[3].name = "Gate 4";
    gates[3].renPin = GATE4_REN;
    gates[3].lenPin = GATE4_LEN;
}

void createDefaultConfig() {
    JsonDocument doc;
    
    doc["stationName"] = "Water Gate Controller";
    doc["measurementInterval"] = 30;
    doc["mainCalibrationOffset"] = 0.0;
    doc["mainSensorToBottomDistance"] = 300.0;
    doc["mainUpperTarget"] = -40.0;
    doc["mainLowerTarget"] = -60.0;
    doc["secondaryCalibrationOffset"] = 0.0;
    doc["secondarySensorToBottomDistance"] = 300.0;
    doc["gateOperationDelay"] = 60;
    doc["autoModeEnabled"] = false;
    
    JsonArray gatesArray = doc["gates"].to<JsonArray>();
    for (int i = 0; i < NUM_GATES; i++) {
        JsonObject gate = gatesArray.add<JsonObject>();
        gate["index"] = i;
        gate["name"] = "Gate " + String(i + 1);
        gate["enabled"] = (i == 0); // Only Gate 1 enabled by default
    }
    
    File file = LittleFS.open(CONFIG_FILE, "w");
    if (file) {
        serializeJsonPretty(doc, file);
        file.close();
        Serial.println("Default config.json created");
        addToSerialBuffer("Default config created");
    }
}

void addToSerialBuffer(const String &message)
{
    String timestampedMessage = getFormattedDateTime() + " - " + message;
    Serial.println(timestampedMessage);
    serialBuff[serialBufferIndex] = timestampedMessage;
    serialBufferIndex = (serialBufferIndex + 1) % SERIAL_BUFFER_SIZE;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("\nInitializing 4-Gate Water Controller (Single PWM)...");

    initializeGates();

    if (!setupLittleFS())
    {
        Serial.println("LittleFS failed!");
        delay(3000);
        ESP.restart();
    }

    if (!loadConfig())
    {
        Serial.println("Creating default config");
        createDefaultConfig();
        loadConfig();
    }

    setupPins();
    setupWiFi();
    setupWebServer();
    initWatchdog();

    startTime = millis();
    validateMeasurementInterval();

    Serial.println("System initialized");
    Serial.printf("Access: http://%s\n", WiFi.softAPIP().toString().c_str());
    measureWaterLevels();
}

void setupPins()
{
    // Setup shared PWM pin - set to HIGH (full speed)
    pinMode(SHARED_PWM, OUTPUT);
    digitalWrite(SHARED_PWM, HIGH);  // Always HIGH for full speed
    
    Serial.println("Shared PWM pin initialized on GPIO " + String(SHARED_PWM));
    
    // Initialize all gate enable pins
    for (int i = 0; i < NUM_GATES; i++)
    {
        pinMode(gates[i].renPin, OUTPUT);
        pinMode(gates[i].lenPin, OUTPUT);
        
        // Disable both directions initially (gate stopped)
        digitalWrite(gates[i].renPin, LOW);
        digitalWrite(gates[i].lenPin, LOW);
        
        gates[i].isMoving = false;
    }
    
    Serial.println("All gate enable pins initialized");
}

void setupWebServer()
{
    server.enableCORS(true);

    server.on("/", handleRoot);
    server.on("/settings", HTTP_GET, handleGetConfig);
    server.on("/settings", HTTP_POST, handleSettings);
    server.on("/current", handleCurrentLevel);
    server.on("/gate", HTTP_POST, handleGateControl);
    server.on("/restart", handleRestart);
    server.on("/uptime", handleUptime);

    server.begin();
    Serial.println("Web server started");
}

void handleGetConfig()
{
    JsonDocument doc;

    doc["stationName"] = config.stationName;
    doc["measurementInterval"] = config.measurementInterval;
    doc["mainCalibrationOffset"] = config.mainCalibrationOffset;
    doc["mainSensorToBottomDistance"] = config.mainSensorToBottomDistance;
    doc["mainUpperTarget"] = config.mainUpperTarget;
    doc["mainLowerTarget"] = config.mainLowerTarget;
    doc["secondaryCalibrationOffset"] = config.secondaryCalibrationOffset;
    doc["secondarySensorToBottomDistance"] = config.secondarySensorToBottomDistance;
    doc["gateOperationDelay"] = config.gateOperationDelay;
    doc["autoModeEnabled"] = config.autoModeEnabled;
    
    JsonArray gatesArray = doc["gates"].to<JsonArray>();
    for (int i = 0; i < NUM_GATES; i++)
    {
        JsonObject gate = gatesArray.add<JsonObject>();
        gate["index"] = i;
        gate["name"] = gates[i].name;
        gate["enabled"] = gates[i].enabled;
    }

    String jsonString;
    serializeJson(doc, jsonString);
    server.send(200, "application/json", jsonString);
}

void handleSettings() {
  // 1) Use JsonDocument with automatic memory management
  JsonDocument doc;
  bool parsed = false;

  // --- helpers ---
  auto parseBool = [&](JsonObject o, const char* key, bool& out) {
    if (o[key].isNull()) return false;
    if (o[key].is<bool>()) { out = o[key].as<bool>(); return true; }
    if (o[key].is<const char*>()) {
      String s = String(o[key].as<const char*>());
      s.toLowerCase();
      if (s == "1" || s == "true"  || s == "on"  || s == "yes")  { out = true;  return true; }
      if (s == "0" || s == "false" || s == "off" || s == "no")   { out = false; return true; }
    }
    // numeric string fallback
    if (o[key].is<long>() || o[key].is<int>()) { out = o[key].as<long>() != 0; return true; }
    return false;
  };
  auto parseULong = [&](JsonObject o, const char* key, unsigned long& out) {
    if (o[key].isNull()) return false;
    if (o[key].is<unsigned long>()) { out = o[key].as<unsigned long>(); return true; }
    if (o[key].is<long>())          { long v = o[key].as<long>(); out = v < 0 ? 0UL : (unsigned long)v; return true; }
    if (o[key].is<const char*>())   { out = strtoul(o[key].as<const char*>(), nullptr, 10); return true; }
    return false;
  };
  auto parseUInt = [&](JsonObject o, const char* key, unsigned int& out) {
    if (o[key].isNull()) return false;
    if (o[key].is<unsigned int>()) { out = o[key].as<unsigned int>(); return true; }
    if (o[key].is<int>())          { int v = o[key].as<int>(); out = v < 0 ? 0U : (unsigned int)v; return true; }
    if (o[key].is<const char*>())  { out = (unsigned int)strtoul(o[key].as<const char*>(), nullptr, 10); return true; }
    return false;
  };
  auto parseFloatIf = [&](JsonObject o, const char* key, float& out) {
    if (o[key].isNull()) return false;
    if (o[key].is<float>() || o[key].is<double>()) { out = o[key].as<float>(); return true; }
    if (o[key].is<const char*>()) { out = atof(o[key].as<const char*>()); return true; }
    if (o[key].is<long>() || o[key].is<int>()) { out = (float)o[key].as<long>(); return true; }
    return false;
  };
  auto parseStringIf = [&](JsonObject o, const char* key, String& out) {
    if (o[key].isNull()) return false;
    if (o[key].is<const char*>()) { out = o[key].as<const char*>(); return true; }
    return false;
  };

  // 2) Try JSON body first (ensure non-empty)
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    if (body.length() > 0) {
      DeserializationError err = deserializeJson(doc, body);
      if (!err) parsed = true;
      else {
        // helpful error back to client
        String msg = "Invalid JSON: ";
        msg += err.c_str();
        server.send(400, "text/plain", msg);
        return;
      }
    }
  }

  // 3) If not JSON, try to build from form fields (works for x-www-form-urlencoded; multipart needs upload handler)
  if (!parsed) {
    JsonObject o = doc.to<JsonObject>();
    auto put = [&](const char* k){ if (server.hasArg(k)) o[k] = server.arg(k); };
    put("stationName"); put("measurementInterval");
    put("mainCalibrationOffset"); put("mainSensorToBottomDistance");
    put("mainUpperTarget"); put("mainLowerTarget");
    put("secondaryCalibrationOffset"); put("secondarySensorToBottomDistance");
    put("gateOperationDelay"); put("autoModeEnabled");
    parsed = (o.size() > 0);
    if (!parsed) {
      server.send(400, "text/plain", "No settings in request (empty body).");
      return;
    }
  }

  JsonObject o = doc.as<JsonObject>();

  // 4) Update only provided fields (measurementInterval is in SECONDS)
  parseStringIf(o, "stationName", config.stationName);

  unsigned long tmpUL;
  if (parseULong(o, "measurementInterval", tmpUL)) config.measurementInterval = tmpUL;

  parseFloatIf(o, "mainCalibrationOffset", config.mainCalibrationOffset);
  parseFloatIf(o, "mainSensorToBottomDistance", config.mainSensorToBottomDistance);
  parseFloatIf(o, "mainUpperTarget", config.mainUpperTarget);
  parseFloatIf(o, "mainLowerTarget", config.mainLowerTarget);

  parseFloatIf(o, "secondaryCalibrationOffset", config.secondaryCalibrationOffset);
  parseFloatIf(o, "secondarySensorToBottomDistance", config.secondarySensorToBottomDistance);

  unsigned int tmpUI;
  if (parseUInt(o, "gateOperationDelay", tmpUI)) config.gateOperationDelay = tmpUI;

  bool tmpB;
  if (parseBool(o, "autoModeEnabled", tmpB)) { config.autoModeEnabled = tmpB; autoMode = tmpB; }

  // Gates (if present)
  if (!o["gates"].isNull() && o["gates"].is<JsonArray>()) {
    JsonArray gatesArray = o["gates"];
    for (int i = 0; i < NUM_GATES && i < (int)gatesArray.size(); i++) {
      JsonObject g = gatesArray[i];
      if (g.isNull()) continue;
      parseBool(g, "enabled", gates[i].enabled);
      parseStringIf(g, "name", gates[i].name);
    }
  }

  // 5) Validate interval (seconds)
  if (config.measurementInterval < 5UL) config.measurementInterval = 5UL;

  // 6) Save and report detailed errors
  if (saveConfig()) {
    server.send(200, "text/plain", "OK");
  } else {
    // Expand the message to help you debug FS issues on the client quickly
    server.send(500, "text/plain", "Save failed: LittleFS open/write error (check free space and that LittleFS.begin() succeeded).");
  }
}


template<typename T>
bool getIfPresent(JsonObject obj, const char* key, T& out) {
    if (obj[key].is<T>()) { out = obj[key].as<T>(); return true; }
    return false;
}

void handleCurrentLevel()
{
    JsonDocument doc;

    doc["mainWaterLevel"] = mainWaterLevel;
    doc["secondaryWaterLevel"] = secondaryWaterLevel;
    doc["mainRawDistance"] = mainRawDistance;
    doc["secondaryRawDistance"] = secondaryRawDistance;
    doc["autoMode"] = autoMode;

    doc["mainInTargetRange"] = (mainWaterLevel > config.mainLowerTarget && mainWaterLevel < config.mainUpperTarget);
    doc["shouldOpen"] = shouldOpenGates();
    doc["shouldClose"] = shouldCloseGates();
    
    JsonArray gatesArray = doc["gates"].to<JsonArray>();
    for (int i = 0; i < NUM_GATES; i++)
    {
        JsonObject gate = gatesArray.add<JsonObject>();
        gate["index"] = i;
        gate["name"] = gates[i].name;
        gate["enabled"] = gates[i].enabled;
        gate["isOpen"] = gates[i].isOpen;
        gate["isMoving"] = gates[i].isMoving;
    }

    JsonArray serialArray = doc["serialBuffer"].to<JsonArray>();
    for (int i = 0; i < SERIAL_BUFFER_SIZE; i++)
    {
        int idx = (serialBufferIndex + i) % SERIAL_BUFFER_SIZE;
        if (serialBuff[idx] != "")
            serialArray.add(serialBuff[idx]);
    }

    String jsonString;
    serializeJson(doc, jsonString);
    server.send(200, "application/json", jsonString);
}

void handleGateControl()
{
    if (server.hasArg("plain"))
    {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, server.arg("plain"));

        if (!error)
        {
            int gateIndex = doc["gate"].as<int>();
            String action = doc["action"].as<String>();

            if (gateIndex >= 0 && gateIndex < NUM_GATES)
            {
                if (action == "open")
                {
                    openGate(gateIndex);
                    server.send(200, "text/plain", "Gate opening");
                }
                else if (action == "close")
                {
                    closeGate(gateIndex);
                    server.send(200, "text/plain", "Gate closing");
                }
                else if (action == "stop")
                {
                    stopGate(gateIndex);
                    server.send(200, "text/plain", "Gate stopped");
                }
                else
                {
                    server.send(400, "text/plain", "Invalid action");
                }
            }
            else
            {
                server.send(400, "text/plain", "Invalid gate");
            }
        }
        else
        {
            server.send(400, "text/plain", "Invalid JSON");
        }
    }
}

void loop() {
    resetWatchdog();
    server.handleClient();

    unsigned long currentTime = millis();

    // multiply by 1000 to use seconds internally
    if (currentTime - lastMeasurementTime >= (config.measurementInterval * 1000UL)) {
        measureWaterLevels();
        lastMeasurementTime = currentTime;
    }

    if (autoMode) {
        controlAutoGates();
    }
    delay(100);
}

void measureWaterLevels()
{
    Serial.println("Measuring water levels...");

    HardwareSerial SerialA01_Main(1);
    HardwareSerial SerialA01_Secondary(2);

    mainRawDistance = readA01NYUB(SerialA01_Main, MAIN_A01_RX, MAIN_A01_TX, "Main");
    if (mainRawDistance > 0)
    {
        mainWaterLevel = config.mainSensorToBottomDistance - mainRawDistance + config.mainCalibrationOffset;
    }

    secondaryRawDistance = readA01NYUB(SerialA01_Secondary, SECONDARY_A01_RX, SECONDARY_A01_TX, "Secondary");
    if (secondaryRawDistance > 0)
    {
        secondaryWaterLevel = config.secondarySensorToBottomDistance - secondaryRawDistance + config.secondaryCalibrationOffset;
    }

    Serial.printf("Main: %.1f cm (raw: %.1f mm)\n", mainWaterLevel, mainRawDistance);
    Serial.printf("Secondary: %.1f cm (raw: %.1f mm)\n", secondaryWaterLevel, secondaryRawDistance);

    String logMsg = "Main: " + String(mainWaterLevel, 1) + "cm, Secondary: " + String(secondaryWaterLevel, 1) + "cm";
    addToSerialBuffer(logMsg);
}

float readA01NYUB(HardwareSerial &serial, int rx, int tx, const char* sensorName)
{
    serial.begin(9600, SERIAL_8N1, rx, tx);
    delay(100);

    const int NUM_MEASUREMENTS = 5;
    float measurements[NUM_MEASUREMENTS];
    int validMeasurements = 0;

    for (int i = 0; i < NUM_MEASUREMENTS; i++)
    {
        unsigned long startTime = millis();
        while (serial.available() < 4 && (millis() - startTime) < 500)
        {
            delay(10);
        }

        if (serial.available() >= 4)
        {
            byte header = serial.read();
            if (header == 0xFF)
            {
                byte highByte = serial.read();
                byte lowByte = serial.read();
                byte checksum = serial.read();

                byte calculatedChecksum = (0xFF + highByte + lowByte) & 0xFF;

                if (checksum == calculatedChecksum)
                {
                    int distance = (highByte << 8) | lowByte;
                    if (distance >= 30 && distance <= 4500)
                    {
                        measurements[validMeasurements++] = distance;
                    }
                }
            }
        }

        while (serial.available())
            serial.read();

        delay(100);
    }

    serial.end();

    if (validMeasurements == 0)
        return -1.0;

    float sum = 0;
    for (int i = 0; i < validMeasurements; i++)
        sum += measurements[i];
    
    return sum / validMeasurements;
}

bool shouldOpenGates()
{
    bool mainTooHigh = mainWaterLevel > config.mainUpperTarget && secondaryWaterLevel < mainWaterLevel;
    bool mainTooLow = mainWaterLevel < config.mainLowerTarget && secondaryWaterLevel > mainWaterLevel;

    return mainTooHigh || (mainTooLow);
}

bool shouldCloseGates()
{
    bool mainInRange = mainWaterLevel > config.mainLowerTarget && mainWaterLevel < config.mainUpperTarget;
    
    return mainInRange;
}

void controlAutoGates()
{
    unsigned long currentTime = millis();
    
    // Check operation delay
    if (currentTime - lastAutoActionTime < config.gateOperationDelay * 1000UL)
        return;
    
    if (shouldOpenGates())
    {
        for (int i = 0; i < NUM_GATES; i++)
        {
            if (gates[i].enabled && !gates[i].isOpen && !gates[i].isMoving)
            {
                openGate(i);
                addToSerialBuffer("AUTO: Opening " + gates[i].name);
            }
        }
        lastAutoActionTime = currentTime;
    }
    else if (shouldCloseGates())
    {
        for (int i = 0; i < NUM_GATES; i++)
        {
            if (gates[i].enabled && gates[i].isOpen && !gates[i].isMoving)
            {
                closeGate(i);
                addToSerialBuffer("AUTO: Closing " + gates[i].name);
            }
        }
        lastAutoActionTime = currentTime;
    }
}

void openGate(int gateIndex)
{
    if (gateIndex < 0 || gateIndex >= NUM_GATES) return;
    
    // Disable close direction, enable open direction
    digitalWrite(gates[gateIndex].lenPin, LOW);    // Stop close
    digitalWrite(gates[gateIndex].renPin, HIGH);   // Enable open
    gates[gateIndex].isMoving = true;
    gates[gateIndex].isOpen = true;
    
    addToSerialBuffer(gates[gateIndex].name + " OPENING");
}

void closeGate(int gateIndex)
{
    if (gateIndex < 0 || gateIndex >= NUM_GATES) return;
    
    // Disable open direction, enable close direction
    digitalWrite(gates[gateIndex].renPin, LOW);    // Stop open
    digitalWrite(gates[gateIndex].lenPin, HIGH);   // Enable close
    gates[gateIndex].isMoving = true;
    gates[gateIndex].isOpen = false;
    
    addToSerialBuffer(gates[gateIndex].name + " CLOSING");
}

void stopGate(int gateIndex)
{
    if (gateIndex < 0 || gateIndex >= NUM_GATES) return;
    
    // Disable both directions
    digitalWrite(gates[gateIndex].renPin, LOW);
    digitalWrite(gates[gateIndex].lenPin, LOW);
    gates[gateIndex].isMoving = false;
    
    addToSerialBuffer(gates[gateIndex].name + " STOPPED");
}

bool loadConfig() {
    if (!LittleFS.exists(CONFIG_FILE)) return false;
    File file = LittleFS.open(CONFIG_FILE, "r");
    if (!file) return false;

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) return false;

    config.stationName = doc["stationName"].as<String>();

    // migration: treat >=1000 as milliseconds from old config and convert to seconds
    unsigned long mi = doc["measurementInterval"] | 30UL;
    if (mi >= 1000UL) mi = mi / 1000UL;
    config.measurementInterval = mi;

    config.mainCalibrationOffset = doc["mainCalibrationOffset"].as<float>();
    config.mainSensorToBottomDistance = doc["mainSensorToBottomDistance"].as<float>();
    config.mainUpperTarget = doc["mainUpperTarget"].as<float>();
    config.mainLowerTarget = doc["mainLowerTarget"].as<float>();
    config.secondaryCalibrationOffset = doc["secondaryCalibrationOffset"].as<float>();
    config.secondarySensorToBottomDistance = doc["secondarySensorToBottomDistance"].as<float>();
    config.gateOperationDelay = doc["gateOperationDelay"].as<unsigned int>();
    config.autoModeEnabled = doc["autoModeEnabled"].as<bool>();
    autoMode = config.autoModeEnabled;

    JsonArray gatesArray = doc["gates"];
    for (int i = 0; i < NUM_GATES && i < gatesArray.size(); i++) {
        JsonObject gate = gatesArray[i];
        gates[i].enabled = gate["enabled"].as<bool>();
        gates[i].name = gate["name"].as<String>();
    }
    return true;
}

bool saveConfig()
{
    File file = LittleFS.open(CONFIG_FILE, "w");
    if (!file) return false;

    JsonDocument doc;

    doc["stationName"] = config.stationName;
    doc["measurementInterval"] = config.measurementInterval;
    doc["mainCalibrationOffset"] = config.mainCalibrationOffset;
    doc["mainSensorToBottomDistance"] = config.mainSensorToBottomDistance;
    doc["mainUpperTarget"] = config.mainUpperTarget;
    doc["mainLowerTarget"] = config.mainLowerTarget;
    doc["secondaryCalibrationOffset"] = config.secondaryCalibrationOffset;
    doc["secondarySensorToBottomDistance"] = config.secondarySensorToBottomDistance;
    doc["gateOperationDelay"] = config.gateOperationDelay;
    doc["autoModeEnabled"] = config.autoModeEnabled;
    
    JsonArray gatesArray = doc["gates"].to<JsonArray>();
    for (int i = 0; i < NUM_GATES; i++)
    {
        JsonObject gate = gatesArray.add<JsonObject>();
        gate["index"] = i;
        gate["name"] = gates[i].name;
        gate["enabled"] = gates[i].enabled;
    }

    if (serializeJson(doc, file) == 0) { file.close(); return false; }
    file.close();
    return true;
}

void handleRoot()
{
    if (LittleFS.exists("/index.html")) {
        File file = LittleFS.open("/index.html", "r");
        if (file) {
            server.streamFile(file, "text/html");
            file.close();
            return;
        }
    }
    
    String html = "<html><body><h1>4-Gate Water Controller</h1>";
    html += "<p>Main: " + String(mainWaterLevel) + " cm</p>";
    html += "<p>Secondary: " + String(secondaryWaterLevel) + " cm</p>";
    html += "<p>Auto Mode: " + String(autoMode ? "ON" : "OFF") + "</p>";
    html += "<h2>Gates:</h2>";
    for (int i = 0; i < NUM_GATES; i++) {
        html += "<p>" + gates[i].name + " - ";
        html += "Enabled: " + String(gates[i].enabled ? "YES" : "NO") + ", ";
        html += "State: " + String(gates[i].isOpen ? "OPEN" : "CLOSED") + "</p>";
    }
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handleRestart()
{
    server.send(200, "text/plain", "Restarting...");
    delay(1000);
    ESP.restart();
}

void handleUptime()
{
    unsigned long uptimeSeconds = millis() / 1000;
    unsigned long uptimeMinutes = uptimeSeconds / 60;
    unsigned long uptimeHours = uptimeMinutes / 60;
    unsigned long uptimeDays = uptimeHours / 24;

    String uptimeStr = String(uptimeDays) + "d " +
                       String(uptimeHours % 24) + "h " +
                       String(uptimeMinutes % 60) + "m " +
                       String(uptimeSeconds % 60) + "s";

    server.send(200, "text/plain", uptimeStr);
}

void validateMeasurementInterval() {
    if (config.measurementInterval < MINIMUM_INTERVAL_SECONDS) {
        config.measurementInterval = MINIMUM_INTERVAL_SECONDS;
        saveConfig();
    }
}

bool setupLittleFS()
{
    if (!LittleFS.begin())
    {
        Serial.println("LittleFS Mount Failed");
        return false;
    }
    Serial.println("LittleFS mounted");
    return true;
}

void setupWiFi()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP("WaterGateController", "12345678");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
}

String getFormattedDateTime()
{
    unsigned long currentTime = millis();
    unsigned long seconds = currentTime / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    
    return String(hours % 24) + ":" + 
           String((minutes % 60) < 10 ? "0" : "") + String(minutes % 60) + ":" + 
           String((seconds % 60) < 10 ? "0" : "") + String(seconds % 60);
}

void initWatchdog()
{
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
    Serial.println("Watchdog initialized");
}

void resetWatchdog()
{
    esp_task_wdt_reset();
}