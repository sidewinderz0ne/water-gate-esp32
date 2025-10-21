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

// BTS7960 Motor Driver Pins for 4 Gates
// Gate 1
#define GATE1_RPWM 18  // Open
#define GATE1_LPWM 19  // Close
#define GATE1_REN 21
#define GATE1_LEN 22

// Gate 2
#define GATE2_RPWM 23
#define GATE2_LPWM 32
#define GATE2_REN 33
#define GATE2_LEN 27

// Gate 3
#define GATE3_RPWM 14
#define GATE3_LPWM 12
#define GATE3_REN 13
#define GATE3_LEN 15

// Gate 4
#define GATE4_RPWM 2
#define GATE4_LPWM 4
#define GATE4_REN 5
#define GATE4_LEN 34

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
    int rpwmPin;
    int lpwmPin;
    int renPin;
    int lenPin;
    
    Gate() : name("Gate"), enabled(false), isOpen(false), isMoving(false),
             rpwmPin(0), lpwmPin(0), renPin(0), lenPin(0) {}
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
    float secondaryMinLevel;
    
    // Gate operation settings
    unsigned int gateOperationDelay;
    bool autoModeEnabled;
    
    Config() : stationName("Water Gate Controller"),
               measurementInterval(30000),
               mainCalibrationOffset(0.0),
               mainSensorToBottomDistance(300.0),
               mainUpperTarget(-40.0),
               mainLowerTarget(-60.0),
               secondaryCalibrationOffset(0.0),
               secondarySensorToBottomDistance(300.0),
               secondaryMinLevel(-80.0),
               gateOperationDelay(60),
               autoModeEnabled(false) {}
} config;

unsigned long startTime = 0;
const unsigned long MINIMUM_INTERVAL = 5000;

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

void initializeGates()
{
    // Gate 1
    gates[0].name = "Gate 1";
    gates[0].rpwmPin = GATE1_RPWM;
    gates[0].lpwmPin = GATE1_LPWM;
    gates[0].renPin = GATE1_REN;
    gates[0].lenPin = GATE1_LEN;
    
    // Gate 2
    gates[1].name = "Gate 2";
    gates[1].rpwmPin = GATE2_RPWM;
    gates[1].lpwmPin = GATE2_LPWM;
    gates[1].renPin = GATE2_REN;
    gates[1].lenPin = GATE2_LEN;
    
    // Gate 3
    gates[2].name = "Gate 3";
    gates[2].rpwmPin = GATE3_RPWM;
    gates[2].lpwmPin = GATE3_LPWM;
    gates[2].renPin = GATE3_REN;
    gates[2].lenPin = GATE3_LEN;
    
    // Gate 4
    gates[3].name = "Gate 4";
    gates[3].rpwmPin = GATE4_RPWM;
    gates[3].lpwmPin = GATE4_LPWM;
    gates[3].renPin = GATE4_REN;
    gates[3].lenPin = GATE4_LEN;
}

void createDefaultConfig() {
    JsonDocument doc;
    
    doc["stationName"] = "Water Gate Controller";
    doc["measurementInterval"] = 30000;
    doc["mainCalibrationOffset"] = 0.0;
    doc["mainSensorToBottomDistance"] = 300.0;
    doc["mainUpperTarget"] = -40.0;
    doc["mainLowerTarget"] = -60.0;
    doc["secondaryCalibrationOffset"] = 0.0;
    doc["secondarySensorToBottomDistance"] = 300.0;
    doc["secondaryMinLevel"] = -80.0;
    doc["gateOperationDelay"] = 60;
    doc["autoModeEnabled"] = false;
    
    JsonArray gatesArray = doc["gates"].to<JsonArray>();
    for (int i = 0; i < NUM_GATES; i++) {
        JsonObject gate = gatesArray.createNestedObject();
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
    Serial.println("\nInitializing 4-Gate Water Controller...");

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
    // Initialize all gate pins
    for (int i = 0; i < NUM_GATES; i++)
    {
        pinMode(gates[i].rpwmPin, OUTPUT);
        pinMode(gates[i].lpwmPin, OUTPUT);
        pinMode(gates[i].renPin, OUTPUT);
        pinMode(gates[i].lenPin, OUTPUT);
        
        // Enable the drivers
        digitalWrite(gates[i].renPin, HIGH);
        digitalWrite(gates[i].lenPin, HIGH);
        
        // Stop all gates initially
        digitalWrite(gates[i].rpwmPin, LOW);
        digitalWrite(gates[i].lpwmPin, LOW);
        
        gates[i].isMoving = false;
    }
}

void setupWebServer()
{
    server.enableCORS(true);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/settings", HTTP_POST, handleSettings);
    server.on("/config", HTTP_GET, handleGetConfig);
    server.on("/currentLevel", HTTP_GET, handleCurrentLevel);
    server.on("/gateControl", HTTP_POST, handleGateControl);
    server.on("/restart", HTTP_POST, handleRestart);
    server.on("/uptime", HTTP_GET, handleUptime);

    server.begin();
}

void handleGateControl()
{
    if (!server.hasArg("gate") || !server.hasArg("action"))
    {
        server.send(400, "text/plain", "Missing parameters");
        return;
    }
    
    String action = server.arg("action");
    
    // Handle auto mode toggle
    if (action == "auto")
    {
        autoMode = server.arg("state") == "1";
        config.autoModeEnabled = autoMode;
        saveConfig();
        server.send(200, "text/plain", autoMode ? "Auto mode ON" : "Auto mode OFF");
        return;
    }
    
    // Handle individual gate control
    int gateIndex = server.arg("gate").toInt();
    if (gateIndex < 0 || gateIndex >= NUM_GATES)
    {
        server.send(400, "text/plain", "Invalid gate index");
        return;
    }
    
    if (action == "open")
    {
        openGate(gateIndex);
        server.send(200, "text/plain", gates[gateIndex].name + " opening");
    }
    else if (action == "close")
    {
        closeGate(gateIndex);
        server.send(200, "text/plain", gates[gateIndex].name + " closing");
    }
    else if (action == "stop")
    {
        stopGate(gateIndex);
        server.send(200, "text/plain", gates[gateIndex].name + " stopped");
    }
    else
    {
        server.send(400, "text/plain", "Invalid action");
    }
}

void handleCurrentLevel()
{
    JsonDocument doc;
    
    doc["mainWaterLevel"] = mainWaterLevel;
    doc["mainRawDistance"] = mainRawDistance;
    doc["secondaryWaterLevel"] = secondaryWaterLevel;
    doc["secondaryRawDistance"] = secondaryRawDistance;
    doc["autoMode"] = autoMode;
    doc["shouldOpen"] = shouldOpenGates();
    doc["shouldClose"] = shouldCloseGates();
    
    bool mainInRange = (mainWaterLevel >= config.mainLowerTarget && 
                        mainWaterLevel <= config.mainUpperTarget);
    doc["mainInTargetRange"] = mainInRange;
    
    JsonArray gatesArray = doc["gates"].to<JsonArray>();
    for (int i = 0; i < NUM_GATES; i++)
    {
        JsonObject gate = gatesArray.createNestedObject();
        gate["index"] = i;
        gate["name"] = gates[i].name;
        gate["enabled"] = gates[i].enabled;
        gate["isOpen"] = gates[i].isOpen;
        gate["isMoving"] = gates[i].isMoving;
    }
    
    String jsonString;
    serializeJson(doc, jsonString);
    server.send(200, "application/json", jsonString);
}

void handleSettings()
{
    if (server.hasArg("stationName"))
        config.stationName = server.arg("stationName");
    
    if (server.hasArg("interval"))
        config.measurementInterval = max(MINIMUM_INTERVAL, 
            (unsigned long)server.arg("interval").toInt() * 1000UL);
    
    if (server.hasArg("mainCalibrationOffset"))
        config.mainCalibrationOffset = server.arg("mainCalibrationOffset").toFloat();
    
    if (server.hasArg("mainSensorToBottomDistance"))
        config.mainSensorToBottomDistance = server.arg("mainSensorToBottomDistance").toFloat();
    
    if (server.hasArg("mainUpperTarget"))
        config.mainUpperTarget = server.arg("mainUpperTarget").toFloat();
    
    if (server.hasArg("mainLowerTarget"))
        config.mainLowerTarget = server.arg("mainLowerTarget").toFloat();
    
    if (server.hasArg("secondaryCalibrationOffset"))
        config.secondaryCalibrationOffset = server.arg("secondaryCalibrationOffset").toFloat();
    
    if (server.hasArg("secondarySensorToBottomDistance"))
        config.secondarySensorToBottomDistance = server.arg("secondarySensorToBottomDistance").toFloat();
    
    if (server.hasArg("secondaryMinLevel"))
        config.secondaryMinLevel = server.arg("secondaryMinLevel").toFloat();
    
    if (server.hasArg("gateOperationDelay"))
        config.gateOperationDelay = max(1U, (unsigned int)server.arg("gateOperationDelay").toInt());
    
    if (server.hasArg("autoModeEnabled"))
        config.autoModeEnabled = server.arg("autoModeEnabled") == "1";
    
    // Handle gate configurations
    for (int i = 0; i < NUM_GATES; i++)
    {
        String prefix = "gate" + String(i) + "_";
        
        if (server.hasArg(prefix + "enabled"))
            gates[i].enabled = server.arg(prefix + "enabled") == "1";
        
        if (server.hasArg(prefix + "name"))
            gates[i].name = server.arg(prefix + "name");
    }

    if (saveConfig())
        server.send(200, "text/plain", "Settings saved");
    else
        server.send(500, "text/plain", "Save failed");
}

void handleGetConfig()
{
    JsonDocument doc;

    doc["stationName"] = config.stationName;
    doc["measurementInterval"] = config.measurementInterval / 1000;
    doc["mainCalibrationOffset"] = config.mainCalibrationOffset;
    doc["mainSensorToBottomDistance"] = config.mainSensorToBottomDistance;
    doc["mainUpperTarget"] = config.mainUpperTarget;
    doc["mainLowerTarget"] = config.mainLowerTarget;
    doc["secondaryCalibrationOffset"] = config.secondaryCalibrationOffset;
    doc["secondarySensorToBottomDistance"] = config.secondarySensorToBottomDistance;
    doc["secondaryMinLevel"] = config.secondaryMinLevel;
    doc["gateOperationDelay"] = config.gateOperationDelay;
    doc["autoModeEnabled"] = config.autoModeEnabled;

    JsonArray gatesArray = doc["gates"].to<JsonArray>();
    for (int i = 0; i < NUM_GATES; i++)
    {
        JsonObject gate = gatesArray.createNestedObject();
        gate["index"] = i;
        gate["name"] = gates[i].name;
        gate["enabled"] = gates[i].enabled;
        gate["isOpen"] = gates[i].isOpen;
        gate["isMoving"] = gates[i].isMoving;
    }

    String jsonString;
    serializeJson(doc, jsonString);
    server.send(200, "application/json", jsonString);
}

void loop()
{
    server.handleClient();

    unsigned long currentTime = millis();
    
    // Periodic measurements
    if (currentTime - lastMeasurementTime >= config.measurementInterval)
    {
        measureWaterLevels();
        
        // Auto control for enabled gates
        if (autoMode && config.autoModeEnabled)
        {
            controlAutoGates();
        }
        
        lastMeasurementTime = currentTime;
    }

    resetWatchdog();
}

void measureWaterLevels()
{
    // Read main sensor
    float mainDistance = readA01NYUB(Serial2, MAIN_A01_RX, MAIN_A01_TX, "Main");
    if (mainDistance >= 0)
    {
        mainRawDistance = mainDistance;
        mainWaterLevel = (config.mainSensorToBottomDistance - mainDistance) + 
                         config.mainCalibrationOffset;
        addToSerialBuffer("Main: " + String(mainWaterLevel) + "cm");
    }
    
    // Read secondary sensor
    float secondaryDistance = readA01NYUB(Serial1, SECONDARY_A01_RX, SECONDARY_A01_TX, "Secondary");
    if (secondaryDistance >= 0)
    {
        secondaryRawDistance = secondaryDistance;
        secondaryWaterLevel = (config.secondarySensorToBottomDistance - secondaryDistance) + 
                              config.secondaryCalibrationOffset;
        addToSerialBuffer("Secondary: " + String(secondaryWaterLevel) + "cm");
    }
}

float readA01NYUB(HardwareSerial &serial, int rx, int tx, const char* sensorName)
{
    const int numMeasurements = 5;
    float measurements[numMeasurements];
    int validMeasurements = 0;

    serial.begin(9600, SERIAL_8N1, rx, tx);
    delay(100);

    for (int i = 0; i < numMeasurements; i++)
    {
        if (serial.write(0x01) == 1)
        {
            delay(100);
            if (serial.available() >= 4)
            {
                byte response[4];
                serial.readBytes(response, 4);
                if (response[0] == 0xFF)
                {
                    int distance = (response[1] << 8) | response[2];
                    if (distance > 0 && distance < 7500)
                    {
                        measurements[validMeasurements++] = distance / 10.0;
                    }
                }
            }
        }
        delay(50);
    }

    serial.end();

    if (validMeasurements == 0)
    {
        addToSerialBuffer("Warning: No valid " + String(sensorName) + " readings");
        return -1;
    }

    float sum = 0;
    for (int i = 0; i < validMeasurements; i++)
        sum += measurements[i];
    
    return sum / validMeasurements;
}

bool shouldOpenGates()
{
    bool mainTooHigh = mainWaterLevel > config.mainUpperTarget;
    bool secondaryHasCapacity = secondaryWaterLevel > config.secondaryMinLevel;
    bool mainTooLow = mainWaterLevel < config.mainLowerTarget;
    bool secondaryHasWater = secondaryWaterLevel > mainWaterLevel;
    
    return (mainTooHigh && secondaryHasCapacity) || (mainTooLow && secondaryHasWater);
}

bool shouldCloseGates()
{
    bool mainInRange = (mainWaterLevel >= config.mainLowerTarget && 
                        mainWaterLevel <= config.mainUpperTarget);
    bool mainTooHigh = mainWaterLevel > config.mainUpperTarget;
    bool secondaryNoCapacity = secondaryWaterLevel <= config.secondaryMinLevel;
    bool mainTooLow = mainWaterLevel < config.mainLowerTarget;
    bool secondaryNoWater = secondaryWaterLevel <= mainWaterLevel;
    
    return mainInRange || (mainTooHigh && secondaryNoCapacity) || 
           (mainTooLow && secondaryNoWater);
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
    
    digitalWrite(gates[gateIndex].lpwmPin, LOW);   // Stop close direction
    digitalWrite(gates[gateIndex].rpwmPin, HIGH);  // Start open direction
    gates[gateIndex].isMoving = true;
    gates[gateIndex].isOpen = true;
    
    addToSerialBuffer(gates[gateIndex].name + " OPENING");
}

void closeGate(int gateIndex)
{
    if (gateIndex < 0 || gateIndex >= NUM_GATES) return;
    
    digitalWrite(gates[gateIndex].rpwmPin, LOW);   // Stop open direction
    digitalWrite(gates[gateIndex].lpwmPin, HIGH);  // Start close direction
    gates[gateIndex].isMoving = true;
    gates[gateIndex].isOpen = false;
    
    addToSerialBuffer(gates[gateIndex].name + " CLOSING");
}

void stopGate(int gateIndex)
{
    if (gateIndex < 0 || gateIndex >= NUM_GATES) return;
    
    digitalWrite(gates[gateIndex].rpwmPin, LOW);
    digitalWrite(gates[gateIndex].lpwmPin, LOW);
    gates[gateIndex].isMoving = false;
    
    addToSerialBuffer(gates[gateIndex].name + " STOPPED");
}

bool loadConfig()
{
    if (!LittleFS.exists(CONFIG_FILE))
        return false;

    File file = LittleFS.open(CONFIG_FILE, "r");
    if (!file) return false;

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) return false;

    config.stationName = doc["stationName"].as<String>();
    config.measurementInterval = doc["measurementInterval"].as<unsigned long>();
    config.mainCalibrationOffset = doc["mainCalibrationOffset"].as<float>();
    config.mainSensorToBottomDistance = doc["mainSensorToBottomDistance"].as<float>();
    config.mainUpperTarget = doc["mainUpperTarget"].as<float>();
    config.mainLowerTarget = doc["mainLowerTarget"].as<float>();
    config.secondaryCalibrationOffset = doc["secondaryCalibrationOffset"].as<float>();
    config.secondarySensorToBottomDistance = doc["secondarySensorToBottomDistance"].as<float>();
    config.secondaryMinLevel = doc["secondaryMinLevel"].as<float>();
    config.gateOperationDelay = doc["gateOperationDelay"].as<unsigned int>();
    config.autoModeEnabled = doc["autoModeEnabled"].as<bool>();
    
    autoMode = config.autoModeEnabled;

    JsonArray gatesArray = doc["gates"];
    for (int i = 0; i < NUM_GATES && i < gatesArray.size(); i++)
    {
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
    doc["secondaryMinLevel"] = config.secondaryMinLevel;
    doc["gateOperationDelay"] = config.gateOperationDelay;
    doc["autoModeEnabled"] = config.autoModeEnabled;
    
    JsonArray gatesArray = doc["gates"].to<JsonArray>();
    for (int i = 0; i < NUM_GATES; i++)
    {
        JsonObject gate = gatesArray.createNestedObject();
        gate["index"] = i;
        gate["name"] = gates[i].name;
        gate["enabled"] = gates[i].enabled;
    }
    
    if (serializeJson(doc, file) == 0)
    {
        file.close();
        return false;
    }

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

void validateMeasurementInterval()
{
    if (config.measurementInterval < MINIMUM_INTERVAL)
    {
        config.measurementInterval = MINIMUM_INTERVAL;
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