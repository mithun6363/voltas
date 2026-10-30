#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <WiFi.h>

// ─── Device Hostname (mDNS) ──────────────────────────────────────────────────
const char *HOSTNAME = "voltas"; // Access device at http://voltas.local/

// ─── WiFi credentials ────────────────────────────────────────────────────────
const char *WIFI_SSID = "TP-Link";
const char *WIFI_PASSWORD = "sakthi@home";

// ─── MQTT broker ─────────────────────────────────────────────────────────────
const char *MQTT_BROKER = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "esp32-voltas-relay-001"; // must be unique per device

// ─── MQTT topics ─────────────────────────────────────────────────────────────
const char *TOPIC_RELAY[4] = {
    "voltas/relay/1",
    "voltas/relay/2",
    "voltas/relay/3",
    "voltas/relay/4",
};
const char *TOPIC_VOLTAGE = "voltas/sensors/voltage";
const char *TOPIC_CURRENT = "voltas/sensors/current";
const char *TOPIC_POWER = "voltas/sensors/power";
const char *TOPIC_TEMPERATURE = "voltas/sensors/temperature";
const char *TOPIC_RELAY_STATUS = "voltas/relay/status";
const char *TOPIC_STATUS = "voltas/status"; // online / offline (LWT)

// ─── Relay GPIO pins & Logic Type ────────────────────────────────────────────
const uint8_t RELAY_PIN[4] = {26, 27, 14, 12};
bool relayActiveLow = true; // Dynamically configurable (saved in NVS)

// ─── Onboard LED ─────────────────────────────────────────────────────────────
const uint8_t LED_PIN = 2;

// ─── Configurable Sensor Parameters (Saved in Flash NVS) ──────────────────────
float batteryVoltage = 12.60f; // Nominal / No-load Voltage (V)
float internalResistance = 0.08f; // Internal Resistance (Ohms) for realistic voltage sag
float relayCurrent[4] = {1.0f, 2.5f, 0.5f, 0.8f}; // Dynamic Amps per relay

// ─── Intervals ───────────────────────────────────────────────────────────────
const unsigned long PUBLISH_INTERVAL_MS = 1000UL;   // Telemetry publish (1s)
const unsigned long HEARTBEAT_INTERVAL_MS = 30000UL; // Online heartbeat (30s)

// ─── State & Objects ─────────────────────────────────────────────────────────
bool relayState[4] = {false, false, false, false};

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer server(80);
Preferences preferences;

unsigned long lastPublishMs = 0;
unsigned long lastHeartbeatMs = 0;

// ─── Preferences (Flash Storage) ─────────────────────────────────────────────

void loadSettings() {
  preferences.begin("voltas", true); // read-only mode
  batteryVoltage = preferences.getFloat("v_bat", 12.60f);
  internalResistance = preferences.getFloat("r_int", 0.08f);
  relayCurrent[0] = preferences.getFloat("i_r1", 1.0f);
  relayCurrent[1] = preferences.getFloat("i_r2", 2.5f);
  relayCurrent[2] = preferences.getFloat("i_r3", 0.5f);
  relayCurrent[3] = preferences.getFloat("i_r4", 0.8f);
  relayActiveLow = preferences.getBool("r_low", true);
  preferences.end();
  Serial.println("[NVS] Configuration loaded from flash.");
}

void saveSettings() {
  preferences.begin("voltas", false); // read-write mode
  preferences.putFloat("v_bat", batteryVoltage);
  preferences.putFloat("r_int", internalResistance);
  preferences.putFloat("i_r1", relayCurrent[0]);
  preferences.putFloat("i_r2", relayCurrent[1]);
  preferences.putFloat("i_r3", relayCurrent[2]);
  preferences.putFloat("i_r4", relayCurrent[3]);
  preferences.putBool("r_low", relayActiveLow);
  preferences.end();
  Serial.println("[NVS] Configuration saved to flash.");
}

// ─── LED helper ──────────────────────────────────────────────────────────────

void blinkLED(uint8_t times, unsigned int onMs = 80, unsigned int offMs = 80) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    delay(offMs);
  }
}

// ─── Relay & Physics Helpers ──────────────────────────────────────────────────

void setRelayPin(uint8_t index, bool on) {
  if (relayActiveLow) {
    digitalWrite(RELAY_PIN[index], on ? LOW : HIGH);
  } else {
    digitalWrite(RELAY_PIN[index], on ? HIGH : LOW);
  }
}

/** Calculate total current draw from active relays. */
float calculateCurrent() {
  float total = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    if (relayState[i])
      total += relayCurrent[i];
  }
  return total;
}

/** Calculate realistic voltage with sag under load (V = V_nominal - I * R_int)
 * + sensor noise. */
float calculateVoltage() {
  float totalCurrent = calculateCurrent();

  // Voltage sag calculation based on load current and internal resistance
  float vSagged = batteryVoltage - (totalCurrent * internalResistance);

  // Add realistic ADC sensor noise (+/- 0.02V ripple)
  float noise = ((float)random(-20, 21)) / 1000.0f;
  vSagged += noise;

  if (vSagged < 0.0f)
    vSagged = 0.0f;
  return vSagged;
}

// ─── Publish helpers ─────────────────────────────────────────────────────────

void publishHeartbeat() {
  if (mqttClient.connected()) {
    mqttClient.publish(TOPIC_STATUS, "online", true);
    Serial.println("[Heartbeat] Published status: online");
  }
}

void publishSensorData() {
  char buf[32];
  float current = calculateCurrent();
  float vActual = calculateVoltage();
  float power = vActual * current;
  float chipTemp = temperatureRead(); // Reads internal CPU temperature in Celsius

  snprintf(buf, sizeof(buf), "%.2f", vActual);
  mqttClient.publish(TOPIC_VOLTAGE, buf, false);

  snprintf(buf, sizeof(buf), "%.2f", current);
  mqttClient.publish(TOPIC_CURRENT, buf, false);

  snprintf(buf, sizeof(buf), "%.2f", power);
  mqttClient.publish(TOPIC_POWER, buf, false);

  snprintf(buf, sizeof(buf), "%.2f", chipTemp);
  mqttClient.publish(TOPIC_TEMPERATURE, buf, false);

  Serial.printf(
      "[Publish 1s] Voltage=%.2fV  Current=%.2fA  Power=%.2fW  ChipTemp=%.2f°C\n",
      vActual, current, power, chipTemp);
}

void publishRelayStatus() {
  JsonDocument doc;
  for (uint8_t i = 0; i < 4; i++) {
    char key[8];
    snprintf(key, sizeof(key), "relay%u", i + 1);
    doc[key] = relayState[i] ? "ON" : "OFF";
  }

  char buf[128];
  serializeJson(doc, buf);
  mqttClient.publish(TOPIC_RELAY_STATUS, buf, true);
}

// ─── MQTT callback ────────────────────────────────────────────────────────────

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  char msg[16] = {0};
  if (length >= sizeof(msg))
    length = sizeof(msg) - 1;
  memcpy(msg, payload, length);

  Serial.printf("[MQTT] Topic: %s  Msg: %s\n", topic, msg);

  int8_t relayIdx = -1;
  for (uint8_t i = 0; i < 4; i++) {
    if (strcmp(topic, TOPIC_RELAY[i]) == 0) {
      relayIdx = i;
      break;
    }
  }
  if (relayIdx < 0)
    return;

  bool newState = (strcasecmp(msg, "ON") == 0);
  relayState[relayIdx] = newState;
  setRelayPin(relayIdx, newState);

  publishSensorData();
  publishRelayStatus();
}

// ─── Web Server Routes ───────────────────────────────────────────────────────

void handleApiData() {
  float current = calculateCurrent();
  float vActual = calculateVoltage();
  float power = vActual * current;
  float chipTemp = temperatureRead();

  JsonDocument doc;
  doc["v_nominal"] = batteryVoltage;
  doc["v_actual"] = vActual;
  doc["current"] = current;
  doc["power"] = power;
  doc["temperature"] = chipTemp;
  for (uint8_t i = 0; i < 4; i++) {
    doc["r" + String(i + 1)] = relayState[i] ? "ON" : "OFF";
  }

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleRoot() {
  float current = calculateCurrent();
  float vActual = calculateVoltage();
  float power = vActual * current;
  float chipTemp = temperatureRead();

  String html = "<!DOCTYPE html><html><head><title>ESP32 Control Dashboard</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 20px; background: #f4f6f9; color: #333; }";
  html += ".card { background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); margin-bottom: 20px; }";
  html += "h2 { margin-top: 0; color: #0066cc; }";
  html += ".btn { display: inline-block; padding: 10px 15px; color: #fff; text-decoration: none; border-radius: 5px; font-weight: bold; margin-right: 5px; }";
  html += ".btn-on { background: #28a745; } .btn-off { background: #dc3545; }";
  html += "input[type='number'], select { width: 140px; padding: 6px; margin: 5px; border: 1px solid #ccc; border-radius: 4px; }";
  html += "input[type='submit'] { background: #0066cc; color: white; padding: 8px 16px; border: none; border-radius: 4px; cursor: pointer; }";
  html += "</style>";
  
  // Dynamic Background Update Script (No full page reload)
  html += "<script>";
  html += "setInterval(function(){";
  html += "  fetch('/api/data').then(r => r.json()).then(d => {";
  html += "    document.getElementById('v_nom').innerText = d.v_nominal.toFixed(2);";
  html += "    document.getElementById('v_act').innerText = d.v_actual.toFixed(2);";
  html += "    document.getElementById('current').innerText = d.current.toFixed(2);";
  html += "    document.getElementById('power').innerText = d.power.toFixed(2);";
  html += "    document.getElementById('chip_temp').innerText = d.temperature.toFixed(2);";
  html += "    for(let i=1; i<=4; i++){";
  html += "      let el = document.getElementById('r_stat_' + i);";
  html += "      if(el) {";
  html += "        el.innerText = d['r' + i];";
  html += "        el.style.color = d['r' + i] === 'ON' ? 'green' : 'red';";
  html += "      }";
  html += "    }";
  html += "  }).catch(e => console.log(e));";
  html += "}, 1000);";
  html += "</script>";
  
  html += "</head><body>";

  html += "<div class='card'><h2>Live Telemetry (Dynamic Voltage Sag)</h2>";
  html += "<p><b>Nominal Voltage:</b> <span id='v_nom'>" + String(batteryVoltage, 2) + "</span> V</p>";
  html += "<p><b>Loaded Voltage:</b> <span id='v_act' style='color:#d9534f; font-size: 1.2em; font-weight:bold;'>" + String(vActual, 2) + "</span> V</p>";
  html += "<p><b>Total Current:</b> <span id='current'>" + String(current, 2) + "</span> A</p>";
  html += "<p><b>Total Power:</b> <span id='power'>" + String(power, 2) + "</span> W</p>";
  html += "<p><b>Chip Temperature:</b> <span id='chip_temp'>" + String(chipTemp, 2) + "</span> °C</p>";
  html += "</div>";

  html += "<div class='card'><h2>Relay Control</h2>";
  for (uint8_t i = 0; i < 4; i++) {
    html += "<p><b>Relay " + String(i + 1) + ":</b> <span id='r_stat_" + String(i + 1) + "' style='color:" + (relayState[i] ? "green" : "red") + ";'>" + (relayState[i] ? "ON" : "OFF") + "</span> ";
    html += "<a class='btn btn-on' href='/toggle?id=" + String(i) + "&state=1'>ON</a>";
    html += "<a class='btn btn-off' href='/toggle?id=" + String(i) + "&state=0'>OFF</a></p>";
  }
  html += "</div>";

  html += "<div class='card'><h2>System Settings</h2>";
  html += "<form action='/config' method='POST'>";
  
  // Relay Logic Type Selection
  html += "<label>Relay Logic Type: </label>";
  html += "<select name='r_low'>";
  html += "<option value='1'" + String(relayActiveLow ? " selected" : "") + ">Active LOW (LOW = ON)</option>";
  html += "<option value='0'" + String(!relayActiveLow ? " selected" : "") + ">Active HIGH (HIGH = ON)</option>";
  html += "</select><br>";

  html += "<label>No-Load Voltage (V): </label><input type='number' step='0.01' name='v_bat' value='" + String(batteryVoltage, 2) + "'><br>";
  html += "<label>Internal Resistance (Ω): </label><input type='number' step='0.01' name='r_int' value='" + String(internalResistance, 2) + "'><br><br>";
  for (uint8_t i = 0; i < 4; i++) {
    html += "<label>Relay " + String(i + 1) + " Current (A): </label>";
    html += "<input type='number' step='0.1' name='i_r" + String(i + 1) + "' value='" + String(relayCurrent[i], 2) + "'><br>";
  }
  html += "<br><input type='submit' value='Save Settings'>";
  html += "</form></div>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleToggle() {
  if (server.hasArg("id") && server.hasArg("state")) {
    uint8_t id = server.arg("id").toInt();
    bool state = server.arg("state").toInt() == 1;

    if (id < 4) {
      relayState[id] = state;
      setRelayPin(id, state);
      publishSensorData();
      publishRelayStatus();
    }
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleConfig() {
  if (server.hasArg("r_low"))
    relayActiveLow = (server.arg("r_low").toInt() == 1);
  if (server.hasArg("v_bat"))
    batteryVoltage = server.arg("v_bat").toFloat();
  if (server.hasArg("r_int"))
    internalResistance = server.arg("r_int").toFloat();
  if (server.hasArg("i_r1"))
    relayCurrent[0] = server.arg("i_r1").toFloat();
  if (server.hasArg("i_r2"))
    relayCurrent[1] = server.arg("i_r2").toFloat();
  if (server.hasArg("i_r3"))
    relayCurrent[2] = server.arg("i_r3").toFloat();
  if (server.hasArg("i_r4"))
    relayCurrent[3] = server.arg("i_r4").toFloat();

  // Re-apply trigger logic to all active relays
  for (uint8_t i = 0; i < 4; i++) {
    setRelayPin(i, relayState[i]);
  }

  saveSettings();
  publishSensorData();

  server.sendHeader("Location", "/");
  server.send(303);
}

// ─── WiFi & MQTT Setup ───────────────────────────────────────────────────────

void connectWiFi() {
  Serial.printf("\n[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    blinkLED(1, 100, 400);
    Serial.print(".");
  }

  Serial.printf("\n[WiFi] Connected! IP Address: http://%s/\n",
                WiFi.localIP().toString().c_str());

  // Initialize mDNS responder
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[mDNS] Started! Access device at: http://%s.local/\n", HOSTNAME);
  } else {
    Serial.println("[mDNS] Error starting mDNS responder!");
  }
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.printf("[MQTT] Connecting to %s:%d ...", MQTT_BROKER, MQTT_PORT);

    bool ok = mqttClient.connect(MQTT_CLIENT_ID, nullptr, nullptr, TOPIC_STATUS,
                                 0, true, "offline");

    if (ok) {
      Serial.println(" connected!");
      publishHeartbeat(); // Publish online state immediately upon connection

      for (uint8_t i = 0; i < 4; i++) {
        mqttClient.subscribe(TOPIC_RELAY[i]);
      }
      digitalWrite(LED_PIN, HIGH);

      publishSensorData();
      publishRelayStatus();
    } else {
      Serial.printf(" failed (rc=%d). Retry in 5 s...\n", mqttClient.state());
      blinkLED(5, 50, 50);
      delay(5000);
    }
  }
}

// ─── Setup & Loop ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(100);

  randomSeed(micros());
  loadSettings();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(RELAY_PIN[i], OUTPUT);
    setRelayPin(i, false);
  }

  connectWiFi();

  // Start Web Server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleApiData);
  server.on("/toggle", HTTP_GET, handleToggle);
  server.on("/config", HTTP_POST, handleConfig);
  server.begin();
  Serial.println("[HTTP] Web Server started.");

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);

  connectMQTT();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    connectMQTT();
  }

  if (!mqttClient.connected()) {
    connectMQTT();
  }

  mqttClient.loop();
  server.handleClient();

  unsigned long now = millis();

  // Periodic publish of sensor data + relay status every 1000ms
  if (now - lastPublishMs >= PUBLISH_INTERVAL_MS) {
    lastPublishMs = now;
    publishSensorData();
    publishRelayStatus();
  }

  // Periodic publish of online heartbeat status every 30000ms (30s)
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    publishHeartbeat();
  }
}