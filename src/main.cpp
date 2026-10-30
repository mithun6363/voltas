#include <Arduino.h>

/*
 * ESP32 - 4 Channel Relay Control via MQTT
 * Broker  : broker.emqx.io
 * Port    : 1883 (TCP)
 *
 * Safe relay GPIO pins chosen:
 *   Relay 1 → GPIO 26
 *   Relay 2 → GPIO 27
 *   Relay 3 → GPIO 14
 *   Relay 4 → GPIO 12
 *
 * MQTT Topics (subscribe):
 *   voltas/relay/1   → payload "ON" / "OFF"
 *   voltas/relay/2   → payload "ON" / "OFF"
 *   voltas/relay/3   → payload "ON" / "OFF"
 *   voltas/relay/4   → payload "ON" / "OFF"
 *
 * MQTT Topics (publish every 5 s):
 *   voltas/sensors/voltage  → e.g. 12.60
 *   voltas/sensors/current  → dynamic based on relay states
 *   voltas/sensors/power    → voltage × current
 *   voltas/relay/status     → JSON snapshot of all relay states
 *
 * Current contribution per relay:
 *   Relay 1 ON  →  +1.0 A
 *   Relay 2 ON  →  +2.5 A
 *   Relay 3 ON  →  +0.5 A  (example)
 *   Relay 4 ON  →  +0.8 A  (example)
 *
 * Libraries needed (install via Arduino Library Manager):
 *   - PubSubClient  by Nick O'Leary
 *   - ArduinoJson   by Benoit Blanchon
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ─── WiFi credentials ───────────────────────────────────────────────────────
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// ─── MQTT broker ────────────────────────────────────────────────────────────
const char* MQTT_BROKER   = "broker.emqx.io";
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_CLIENT_ID = "esp32-voltas-relay-001";  // must be unique per device

// ─── MQTT topics ────────────────────────────────────────────────────────────
const char* TOPIC_RELAY_1     = "voltas/relay/1";
const char* TOPIC_RELAY_2     = "voltas/relay/2";
const char* TOPIC_RELAY_3     = "voltas/relay/3";
const char* TOPIC_RELAY_4     = "voltas/relay/4";
const char* TOPIC_VOLTAGE     = "voltas/sensors/voltage";
const char* TOPIC_CURRENT     = "voltas/sensors/current";
const char* TOPIC_POWER       = "voltas/sensors/power";
const char* TOPIC_RELAY_STATUS = "voltas/relay/status";

// ─── Relay GPIO pins (safe, not boot-sensitive) ──────────────────────────────
const uint8_t RELAY_PIN[4] = {26, 27, 14, 12};

// Most 4-channel relay modules are ACTIVE-LOW (LOW = relay energised).
// Set RELAY_ACTIVE_LOW = true  for active-low boards (most common).
// Set RELAY_ACTIVE_LOW = false for active-high boards.
const bool RELAY_ACTIVE_LOW = true;

// ─── Fake sensor constants ───────────────────────────────────────────────────
const float BATTERY_VOLTAGE = 12.6f;   // volts (fixed)

// Current added when each relay is ON
const float RELAY_CURRENT[4] = {1.0f, 2.5f, 0.5f, 0.8f};

// ─── Publish interval ────────────────────────────────────────────────────────
const unsigned long PUBLISH_INTERVAL_MS = 5000UL;  // 5 seconds

// ─── State ───────────────────────────────────────────────────────────────────
bool relayState[4] = {false, false, false, false};

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastPublishMs = 0;

// ─── Helpers ─────────────────────────────────────────────────────────────────

/** Drive a relay pin according to its desired logical state. */
void setRelayPin(uint8_t index, bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_PIN[index], on ? LOW : HIGH);
  } else {
    digitalWrite(RELAY_PIN[index], on ? HIGH : LOW);
  }
}

/** Calculate total fake current based on which relays are ON. */
float calculateCurrent() {
  float total = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    if (relayState[i]) total += RELAY_CURRENT[i];
  }
  return total;
}

/** Publish sensor readings (voltage, current, power). */
void publishSensorData() {
  char buf[32];
  float current = calculateCurrent();
  float power   = BATTERY_VOLTAGE * current;

  snprintf(buf, sizeof(buf), "%.2f", BATTERY_VOLTAGE);
  mqttClient.publish(TOPIC_VOLTAGE, buf, true);

  snprintf(buf, sizeof(buf), "%.2f", current);
  mqttClient.publish(TOPIC_CURRENT, buf, true);

  snprintf(buf, sizeof(buf), "%.2f", power);
  mqttClient.publish(TOPIC_POWER, buf, true);

  Serial.printf("[Publish] Voltage=%.2fV  Current=%.2fA  Power=%.2fW\n",
                BATTERY_VOLTAGE, current, power);
}

/** Publish a JSON snapshot of all relay states. */
void publishRelayStatus() {
  StaticJsonDocument<128> doc;
  doc["relay1"] = relayState[0] ? "ON" : "OFF";
  doc["relay2"] = relayState[1] ? "ON" : "OFF";
  doc["relay3"] = relayState[2] ? "ON" : "OFF";
  doc["relay4"] = relayState[3] ? "ON" : "OFF";

  char buf[128];
  serializeJson(doc, buf);
  mqttClient.publish(TOPIC_RELAY_STATUS, buf, true);
  Serial.printf("[Publish] Relay status: %s\n", buf);
}

// ─── MQTT callback ───────────────────────────────────────────────────────────

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Safely copy payload into a null-terminated string
  char msg[16] = {0};
  if (length >= sizeof(msg)) length = sizeof(msg) - 1;
  memcpy(msg, payload, length);

  Serial.printf("[MQTT] Topic: %s  Msg: %s\n", topic, msg);

  // Determine which relay was addressed
  int relayIdx = -1;
  if (strcmp(topic, TOPIC_RELAY_1) == 0) relayIdx = 0;
  else if (strcmp(topic, TOPIC_RELAY_2) == 0) relayIdx = 1;
  else if (strcmp(topic, TOPIC_RELAY_3) == 0) relayIdx = 2;
  else if (strcmp(topic, TOPIC_RELAY_4) == 0) relayIdx = 3;

  if (relayIdx < 0) return;  // unknown topic

  bool newState = (strcasecmp(msg, "ON") == 0);
  relayState[relayIdx] = newState;
  setRelayPin(relayIdx, newState);

  Serial.printf("[Relay %d] → %s\n", relayIdx + 1, newState ? "ON" : "OFF");

  // Immediately publish updated readings after any relay change
  publishSensorData();
  publishRelayStatus();
}

// ─── WiFi ────────────────────────────────────────────────────────────────────

void connectWiFi() {
  Serial.printf("\n[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
}

// ─── MQTT ────────────────────────────────────────────────────────────────────

void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.printf("[MQTT] Connecting to %s:%d ...", MQTT_BROKER, MQTT_PORT);

    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println(" connected!");

      // Subscribe to all four relay command topics
      mqttClient.subscribe(TOPIC_RELAY_1);
      mqttClient.subscribe(TOPIC_RELAY_2);
      mqttClient.subscribe(TOPIC_RELAY_3);
      mqttClient.subscribe(TOPIC_RELAY_4);

      Serial.println("[MQTT] Subscribed to relay topics.");

      // Publish initial readings right after connect
      publishSensorData();
      publishRelayStatus();
    } else {
      Serial.printf(" failed (rc=%d). Retry in 5 s...\n", mqttClient.state());
      delay(5000);
    }
  }
}

// ─── Setup & Loop ────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialise relay pins — all OFF at boot
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(RELAY_PIN[i], OUTPUT);
    setRelayPin(i, false);  // ensure all relays start OFF
  }
  Serial.println("[Init] Relay pins configured. All relays OFF.");

  connectWiFi();

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);

  connectMQTT();
}

void loop() {
  // Reconnect WiFi if dropped
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost connection — reconnecting...");
    connectWiFi();
  }

  // Reconnect MQTT if dropped
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] Disconnected — reconnecting...");
    connectMQTT();
  }

  mqttClient.loop();  // process incoming messages and keep connection alive

  // Periodic publish of sensor data
  unsigned long now = millis();
  if (now - lastPublishMs >= PUBLISH_INTERVAL_MS) {
    lastPublishMs = now;
    publishSensorData();
  }
}

