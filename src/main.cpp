#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <WiFi.h>

// ─── Device Hostname (mDNS) ──────────────────────────────────────────────────
const char *HOSTNAME = "voltas"; // Settings page at http://voltas.local/

// ─── WiFi credentials ────────────────────────────────────────────────────────
const char *WIFI_SSID = "TP-Link";
const char *WIFI_PASSWORD = "sakthi@home";

// ─── MQTT broker ─────────────────────────────────────────────────────────────
const char *MQTT_BROKER = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID =
    "esp32-voltas-relay-001"; // must be unique per device

// ─── MQTT topics ─────────────────────────────────────────────────────────────
// Subscribed (commands from dashboard)
const char *TOPIC_RELAY[4] = {
    "voltas/relay/1",
    "voltas/relay/2",
    "voltas/relay/3",
    "voltas/relay/4",
};
// Published (telemetry to dashboard)
const char *TOPIC_VOLTAGE = "voltas/sensors/voltage";
const char *TOPIC_CURRENT = "voltas/sensors/current";
const char *TOPIC_POWER = "voltas/sensors/power";
const char *TOPIC_TEMPERATURE = "voltas/sensors/temperature";
const char *TOPIC_RELAY_STATUS = "voltas/relay/status";
const char *TOPIC_STATUS = "voltas/status"; // online / offline (LWT)

// ─── Relay GPIO pins & Logic Type ────────────────────────────────────────────
const uint8_t RELAY_PIN[4] = {26, 27, 14, 12};
bool relayActiveLow = true; // Configurable (saved in NVS)

// ─── Onboard LED ─────────────────────────────────────────────────────────────
const uint8_t LED_PIN = 2;

// ─── Configurable Sensor Parameters (Saved in Flash NVS)
// ──────────────────────
float batteryVoltage = 12.60f;
float internalResistance = 0.08f;
float relayCurrent[4] = {1.0f, 2.5f, 0.5f, 0.8f};

// ─── Intervals ───────────────────────────────────────────────────────────────
const unsigned long PUBLISH_INTERVAL_MS = 1000UL;
const unsigned long HEARTBEAT_INTERVAL_MS = 30000UL;

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
  preferences.begin("voltas", true);
  batteryVoltage = preferences.getFloat("v_bat", 12.60f);
  internalResistance = preferences.getFloat("r_int", 0.08f);
  relayCurrent[0] = preferences.getFloat("i_r1", 1.0f);
  relayCurrent[1] = preferences.getFloat("i_r2", 2.5f);
  relayCurrent[2] = preferences.getFloat("i_r3", 0.5f);
  relayCurrent[3] = preferences.getFloat("i_r4", 0.8f);
  relayActiveLow = preferences.getBool("r_low", true);
  preferences.end();
}

void saveSettings() {
  preferences.begin("voltas", false);
  preferences.putFloat("v_bat", batteryVoltage);
  preferences.putFloat("r_int", internalResistance);
  preferences.putFloat("i_r1", relayCurrent[0]);
  preferences.putFloat("i_r2", relayCurrent[1]);
  preferences.putFloat("i_r3", relayCurrent[2]);
  preferences.putFloat("i_r4", relayCurrent[3]);
  preferences.putBool("r_low", relayActiveLow);
  preferences.end();
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

// ─── Relay & Physics Helpers
// ──────────────────────────────────────────────────

void setRelayPin(uint8_t index, bool on) {
  if (relayActiveLow) {
    digitalWrite(RELAY_PIN[index], on ? LOW : HIGH);
  } else {
    digitalWrite(RELAY_PIN[index], on ? HIGH : LOW);
  }
}

/** Total current draw from active relays. */
float calculateCurrent() {
  float total = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    if (relayState[i])
      total += relayCurrent[i];
  }
  return total;
}

/** Voltage with sag under load (V = V_nominal - I * R_int) + sensor noise. */
float calculateVoltage() {
  float vSagged = batteryVoltage - (calculateCurrent() * internalResistance);
  vSagged += ((float)random(-20, 21)) / 1000.0f;
  if (vSagged < 0.0f)
    vSagged = 0.0f;
  return vSagged;
}

// ─── Publish helpers ─────────────────────────────────────────────────────────

void publishHeartbeat() {
  if (mqttClient.connected()) {
    mqttClient.publish(TOPIC_STATUS, "online", true);
  }
}

void publishSensorData() {
  char buf[32];
  float current = calculateCurrent();
  float vActual = calculateVoltage();
  float power = vActual * current;
  float chipTemp = temperatureRead(); // Internal CPU temperature (°C)

  snprintf(buf, sizeof(buf), "%.2f", vActual);
  mqttClient.publish(TOPIC_VOLTAGE, buf, false);

  snprintf(buf, sizeof(buf), "%.2f", current);
  mqttClient.publish(TOPIC_CURRENT, buf, false);

  snprintf(buf, sizeof(buf), "%.2f", power);
  mqttClient.publish(TOPIC_POWER, buf, false);

  snprintf(buf, sizeof(buf), "%.1f", chipTemp);
  mqttClient.publish(TOPIC_TEMPERATURE, buf, false);
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

// ─── MQTT callback
// ────────────────────────────────────────────────────────────

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  char msg[16] = {0};
  if (length >= sizeof(msg))
    length = sizeof(msg) - 1;
  memcpy(msg, payload, length);

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

// ─── Web Server (Settings only) ──────────────────────────────────────────────

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>Voltas Settings</title>";
  html +=
      "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;margin:0;background:#f0f2f5;color:"
          "#333;}";
  html += "header{background:#0055a5;color:#fff;padding:16px "
          "24px;font-size:1.3em;font-weight:bold;}";
  html += ".card{background:#fff;border-radius:10px;box-shadow:0 2px 6px "
          "rgba(0,0,0,.1);padding:18px;margin:20px auto;max-width:480px;}";
  html += "h2{margin:0 0 "
          "14px;font-size:1em;color:#0055a5;text-transform:uppercase;letter-"
          "spacing:.05em;border-bottom:2px solid #e8eaf0;padding-bottom:6px;}";
  html += ".row{display:flex;justify-content:space-between;align-items:center;"
          "padding:5px 0;border-bottom:1px solid #f0f2f5;}";
  html += ".row:last-of-type{border-bottom:none;}";
  html += "input[type=number],select{width:150px;padding:6px;border:1px solid "
          "#ccc;border-radius:4px;}";
  html += "input[type=submit]{background:#0055a5;color:#fff;padding:9px "
          "20px;border:none;border-radius:5px;cursor:pointer;margin-top:12px;}";
  html += "input[type=submit]:hover{background:#003d7a;}";
  html += "</style></head><body>";
  html += "<header>⚡ Voltas Settings</header>";
  html += "<div class='card'><h2>⚙️ Settings</h2>";
  html += "<form action='/config' method='POST'>";

  html +=
      "<div class='row'><label>Relay Logic Type</label><select name='r_low'>";
  html += "<option value='1'" + String(relayActiveLow ? " selected" : "") +
          ">Active LOW (LOW = ON)</option>";
  html += "<option value='0'" + String(!relayActiveLow ? " selected" : "") +
          ">Active HIGH (HIGH = ON)</option>";
  html += "</select></div>";

  html += "<div class='row'><label>No-Load Voltage (V)</label><input "
          "type='number' step='0.01' name='v_bat' value='" +
          String(batteryVoltage, 2) + "'></div>";
  html += "<div class='row'><label>Internal Resistance (Ω)</label><input "
          "type='number' step='0.001' name='r_int' value='" +
          String(internalResistance, 3) + "'></div>";

  for (uint8_t i = 0; i < 4; i++) {
    html += "<div class='row'><label>Relay " + String(i + 1) +
            " Current (A)</label>";
    html += "<input type='number' step='0.1' name='i_r" + String(i + 1) +
            "' value='" + String(relayCurrent[i], 2) + "'></div>";
  }

  html += "<input type='submit' value='💾 Save Settings'></form></div>";
  html += "</body></html>";
  server.send(200, "text/html", html);
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

  // Re-apply trigger logic to all relays
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
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    blinkLED(1, 100, 400);
  }

  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
  }
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    bool ok = mqttClient.connect(MQTT_CLIENT_ID, nullptr, nullptr, TOPIC_STATUS,
                                 0, true, "offline");

    if (ok) {
      publishHeartbeat();

      for (uint8_t i = 0; i < 4; i++) {
        mqttClient.subscribe(TOPIC_RELAY[i]);
      }
      digitalWrite(LED_PIN, HIGH);

      publishSensorData();
      publishRelayStatus();
    } else {
      blinkLED(5, 50, 50);
      delay(5000);
    }
  }
}

// ─── Setup & Loop
// ─────────────────────────────────────────────────────────────

void setup() {
  randomSeed(micros());
  loadSettings();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(RELAY_PIN[i], OUTPUT);
    setRelayPin(i, false);
  }

  connectWiFi();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/config", HTTP_POST, handleConfig);
  server.begin();

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

  if (now - lastPublishMs >= PUBLISH_INTERVAL_MS) {
    lastPublishMs = now;
    publishSensorData();
    publishRelayStatus();
  }

  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    publishHeartbeat();
  }
}