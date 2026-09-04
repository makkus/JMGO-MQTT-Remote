#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#include <NimBLEDevice.h>

#include "secrets.h"

// ===================== USER CONFIG =====================
static const char* WIFI_SSID = SECRET_WIFI_SSID;
static const char* WIFI_PASS = SECRET_WIFI_PASS;
static const char* MQTT_USER = SECRET_MQTT_USER;
static const char* MQTT_PASS = SECRET_MQTT_PASS;
static const char* OTA_HOST  = SECRET_OTA_HOST;
static const char* OTA_PASS  = SECRET_OTA_PASS;

static const char* MQTT_HOST = SECRET_MQTT_HOST;
static const uint16_t MQTT_PORT = SECRET_MQTT_PORT;

static IPAddress PROJECTOR_IP(SECRET_PROJECTOR_IP_OCTETS);
static const uint16_t PROJECTOR_PORT = SECRET_PROJECTOR_PORT;

// MQTT topics
static const char* TOPIC_CMD   = "jmgo/remote/cmd";
static const char* TOPIC_SEQUENCE = "jmgo/remote/sequence";
static const char* TOPIC_STATE = "jmgo/remote/state";

// Timings: LAN projector commands
static const uint32_t PROJECTOR_LAN_TIMEOUT_MS = 3000;
static const uint32_t LAN_KEY_DELAY_MS = 120;
static const uint32_t LAN_STEP_DELAY_MS = 120;
static const uint32_t LAN_SEQUENCE_MAX_DELAY_MS = 60000;
static const size_t LAN_SEQUENCE_MAX_KEYS = 16;
static const uint32_t LAN_OK_DELAY_MS = 2000;
static const uint32_t LAN_POWER_OFF_STEP_DELAY_MS = 250;
static const uint32_t POWER_ON_TO_HDMI_DELAY_MS = 60000;

// HDMI input macro tuning.
static const uint32_t HDMI_AFTER_UPS_DELAY_MS = 50;
static const uint32_t HDMI_RIGHT_STEP_DELAY_MS = 300;
static const uint32_t HDMI_AFTER_RIGHTS_DELAY_MS = 2000;
static const uint32_t HDMI_AFTER_FIRST_OK_DELAY_MS = 2000;
static const uint32_t HDMI_AFTER_SECOND_OK_DELAY_MS = 0;
static const uint8_t HDMI1_RIGHT_PRESSES = 4;
static const uint8_t HDMI2_RIGHT_PRESSES = 4;

// Timings: BLE wake advertising
static const uint32_t WAKE_TOTAL_MS = 4000;
static const uint32_t WAKE_PAYLOAD_ADV_MS = 250;
static const uint32_t WAKE_PAYLOAD_GAP_MS = 30;
// Timings: Wi-Fi, MQTT, startup
static const uint32_t STARTUP_DELAY_MS = 3000;
static const uint32_t WIFI_CONNECT_POLL_MS = 300;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
static const uint32_t MQTT_RECONNECT_DELAY_MS = 2000;

// ===================== MQTT =====================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
static bool otaStarted = false;

// ===================== JMGO BLE wake advertising =====================
static const uint8_t WAKE_PAYLOADS[][14] = {
  {0x46, 0x00, 0x01, SECRET_PROJECTOR_BLE_MAC_REVERSED, 0xff, 0xff, 0xff, 0xff, 0xff},
  {0x46, 0x00, 0x02, SECRET_PROJECTOR_BLE_MAC_REVERSED, 0xff, 0xff, 0xff, 0xff, 0xff},
  {0x46, 0x00, 0x03, SECRET_PROJECTOR_BLE_MAC_REVERSED, 0xff, 0xff, 0xff, 0xff, 0xff},
  {0x46, 0x00, 0x04, SECRET_PROJECTOR_BLE_MAC_REVERSED, 0xff, 0xff, 0xff, 0xff, 0xff},
  {0x46, 0x00, 0x05, SECRET_PROJECTOR_BLE_MAC_REVERSED, 0xff, 0xff, 0xff, 0xff, 0xff},
};

static const int WAKE_PAYLOAD_COUNT = sizeof(WAKE_PAYLOADS) / sizeof(WAKE_PAYLOADS[0]);

// ===================== JMGO LAN packets =====================
static const uint8_t LAN_POWER_MENU_PRESS[]   = {0x09, 0x12, 0x07, 0x0a, 0x05, 0x08, 0xdb, 0x0f, 0x10, 0x01};
static const uint8_t LAN_POWER_MENU_RELEASE[] = {0x09, 0x12, 0x07, 0x0a, 0x05, 0x08, 0xdb, 0x0f, 0x10, 0x00};

static const uint8_t LAN_UP_PRESS[]    = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x13, 0x10, 0x01};
static const uint8_t LAN_UP_RELEASE[]  = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x13, 0x10, 0x00};
static const uint8_t LAN_DOWN_PRESS[]  = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x14, 0x10, 0x01};
static const uint8_t LAN_DOWN_RELEASE[]= {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x14, 0x10, 0x00};
static const uint8_t LAN_LEFT_PRESS[]  = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x15, 0x10, 0x01};
static const uint8_t LAN_LEFT_RELEASE[]= {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x15, 0x10, 0x00};
static const uint8_t LAN_RIGHT_PRESS[] = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x16, 0x10, 0x01};
static const uint8_t LAN_RIGHT_RELEASE[]={0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x16, 0x10, 0x00};
static const uint8_t LAN_OK_PRESS[]    = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x17, 0x10, 0x01};
static const uint8_t LAN_OK_RELEASE[]  = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x17, 0x10, 0x00};
static const uint8_t LAN_VOLUME_UP_PRESS[]   = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x18, 0x10, 0x01};
static const uint8_t LAN_VOLUME_UP_RELEASE[] = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x18, 0x10, 0x00};
static const uint8_t LAN_VOLUME_DOWN_PRESS[]   = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x19, 0x10, 0x01};
static const uint8_t LAN_VOLUME_DOWN_RELEASE[] = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x19, 0x10, 0x00};
static const uint8_t LAN_BACK_PRESS[]  = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x04, 0x10, 0x01};
static const uint8_t LAN_BACK_RELEASE[]= {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x04, 0x10, 0x00};
static const uint8_t LAN_MENU_PRESS[]  = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x52, 0x10, 0x01};
static const uint8_t LAN_MENU_RELEASE[]= {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x52, 0x10, 0x00};
static const uint8_t LAN_SETTINGS_PRESS[]   = {0x09, 0x12, 0x07, 0x0a, 0x05, 0x08, 0xb0, 0x01, 0x10, 0x01};
static const uint8_t LAN_SETTINGS_RELEASE[] = {0x09, 0x12, 0x07, 0x0a, 0x05, 0x08, 0xb0, 0x01, 0x10, 0x00};
static const uint8_t LAN_HOME_PRESS[]   = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x03, 0x10, 0x01};
static const uint8_t LAN_HOME_RELEASE[] = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x03, 0x10, 0x00};
static const uint8_t LAN_SEARCH_PRESS[]   = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x54, 0x10, 0x01};
static const uint8_t LAN_SEARCH_RELEASE[] = {0x08, 0x12, 0x06, 0x0a, 0x04, 0x08, 0x54, 0x10, 0x00};

struct LanKey {
  const char* name;
  const uint8_t* press;
  size_t pressLen;
  const uint8_t* release;
  size_t releaseLen;
};

struct LanSequenceStep {
  const LanKey* key;
  uint32_t afterDelayMs;
};

static const LanKey KEY_POWER_MENU = {"power_menu", LAN_POWER_MENU_PRESS, sizeof(LAN_POWER_MENU_PRESS), LAN_POWER_MENU_RELEASE, sizeof(LAN_POWER_MENU_RELEASE)};
static const LanKey KEY_UP         = {"up",         LAN_UP_PRESS,         sizeof(LAN_UP_PRESS),         LAN_UP_RELEASE,         sizeof(LAN_UP_RELEASE)};
static const LanKey KEY_DOWN       = {"down",       LAN_DOWN_PRESS,       sizeof(LAN_DOWN_PRESS),       LAN_DOWN_RELEASE,       sizeof(LAN_DOWN_RELEASE)};
static const LanKey KEY_LEFT       = {"left",       LAN_LEFT_PRESS,       sizeof(LAN_LEFT_PRESS),       LAN_LEFT_RELEASE,       sizeof(LAN_LEFT_RELEASE)};
static const LanKey KEY_RIGHT      = {"right",      LAN_RIGHT_PRESS,      sizeof(LAN_RIGHT_PRESS),      LAN_RIGHT_RELEASE,      sizeof(LAN_RIGHT_RELEASE)};
static const LanKey KEY_OK         = {"ok",         LAN_OK_PRESS,         sizeof(LAN_OK_PRESS),         LAN_OK_RELEASE,         sizeof(LAN_OK_RELEASE)};
static const LanKey KEY_VOLUME_UP   = {"volume_up",   LAN_VOLUME_UP_PRESS,   sizeof(LAN_VOLUME_UP_PRESS),   LAN_VOLUME_UP_RELEASE,   sizeof(LAN_VOLUME_UP_RELEASE)};
static const LanKey KEY_VOLUME_DOWN = {"volume_down", LAN_VOLUME_DOWN_PRESS, sizeof(LAN_VOLUME_DOWN_PRESS), LAN_VOLUME_DOWN_RELEASE, sizeof(LAN_VOLUME_DOWN_RELEASE)};
static const LanKey KEY_BACK       = {"back",       LAN_BACK_PRESS,       sizeof(LAN_BACK_PRESS),       LAN_BACK_RELEASE,       sizeof(LAN_BACK_RELEASE)};
static const LanKey KEY_MENU       = {"menu",       LAN_MENU_PRESS,       sizeof(LAN_MENU_PRESS),       LAN_MENU_RELEASE,       sizeof(LAN_MENU_RELEASE)};
static const LanKey KEY_SETTINGS   = {"settings",   LAN_SETTINGS_PRESS,   sizeof(LAN_SETTINGS_PRESS),   LAN_SETTINGS_RELEASE,   sizeof(LAN_SETTINGS_RELEASE)};
static const LanKey KEY_HOME       = {"home",       LAN_HOME_PRESS,       sizeof(LAN_HOME_PRESS),       LAN_HOME_RELEASE,       sizeof(LAN_HOME_RELEASE)};
static const LanKey KEY_SEARCH     = {"search",     LAN_SEARCH_PRESS,     sizeof(LAN_SEARCH_PRESS),     LAN_SEARCH_RELEASE,     sizeof(LAN_SEARCH_RELEASE)};

static inline void waitMs(uint32_t ms) { delay(ms); }

static void connectWiFi();
static void ensureOtaReady();
static void mqttEnsureConnected();

static void publishState(const char* state, bool retained = true) {
  mqtt.publish(TOPIC_STATE, state, retained);
}

static void advertiseWakePayload(const uint8_t* payload, size_t len, uint32_t durationMs) {
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();

  NimBLEAdvertisementData advData;
  advData.setManufacturerData(std::string((const char*)payload, len));

  adv->stop();
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(NimBLEAdvertisementData());
  adv->setMinInterval(0x20);
  adv->setMaxInterval(0x40);
  adv->start();

  waitMs(durationMs);
  adv->stop();
  waitMs(WAKE_PAYLOAD_GAP_MS);
}

static void sendWakeAdvertisingBurst() {
  Serial.printf("Sending JMGO wake burst for %lu ms...\n", WAKE_TOTAL_MS);
  publishState("wake_burst_start");

  uint32_t start = millis();
  while (millis() - start < WAKE_TOTAL_MS) {
    for (int i = 0; i < WAKE_PAYLOAD_COUNT; i++) {
      advertiseWakePayload(WAKE_PAYLOADS[i], sizeof(WAKE_PAYLOADS[i]), WAKE_PAYLOAD_ADV_MS);
      if (millis() - start >= WAKE_TOTAL_MS) break;
    }
  }

  NimBLEDevice::getAdvertising()->stop();
  Serial.println("Wake burst finished.");
  publishState("wake_burst_done");
}

static bool sendRawLan(const uint8_t* data, size_t len) {
  WiFiClient client;
  client.setTimeout(PROJECTOR_LAN_TIMEOUT_MS);

  if (!client.connect(PROJECTOR_IP, PROJECTOR_PORT)) {
    Serial.println("Projector LAN connect failed");
    publishState("projector_lan_connect_failed");
    return false;
  }

  size_t written = client.write(data, len);
  client.flush();
  client.stop();

  if (written != len) {
    Serial.println("Projector LAN write failed");
    publishState("projector_lan_write_failed");
    return false;
  }

  return true;
}

static bool tapLanKey(const LanKey& key, uint32_t afterDelayMs = LAN_STEP_DELAY_MS) {
  Serial.print("LAN key: ");
  Serial.println(key.name);

  if (!sendRawLan(key.press, key.pressLen)) return false;
  waitMs(LAN_KEY_DELAY_MS);
  if (!sendRawLan(key.release, key.releaseLen)) return false;
  waitMs(afterDelayMs);

  return true;
}

static const LanKey* findLanKey(const String& name) {
  if (name == "up") return &KEY_UP;
  if (name == "down") return &KEY_DOWN;
  if (name == "left") return &KEY_LEFT;
  if (name == "right") return &KEY_RIGHT;
  if (name == "ok" || name == "enter") return &KEY_OK;
  if (name == "volume_up") return &KEY_VOLUME_UP;
  if (name == "volume_down") return &KEY_VOLUME_DOWN;
  if (name == "back") return &KEY_BACK;
  if (name == "menu") return &KEY_MENU;
  if (name == "settings") return &KEY_SETTINGS;
  if (name == "home") return &KEY_HOME;
  if (name == "search") return &KEY_SEARCH;
  return nullptr;
}

static bool parseSequenceDelay(const String& text, uint32_t* delayMs) {
  if (!text.length()) return false;

  uint32_t value = 0;
  for (size_t i = 0; i < text.length(); i++) {
    char c = text.charAt(i);
    if (c < '0' || c > '9') return false;

    uint32_t digit = c - '0';
    if (value > (LAN_SEQUENCE_MAX_DELAY_MS - digit) / 10) return false;
    value = value * 10 + digit;
  }

  *delayMs = value;
  return true;
}

static bool parseLanSequence(const String& payload, LanSequenceStep* steps, size_t* count) {
  *count = 0;
  size_t start = 0;

  while (start < payload.length()) {
    int comma = payload.indexOf(',', start);
    size_t end = comma < 0 ? payload.length() : static_cast<size_t>(comma);
    String token = payload.substring(start, end);
    token.trim();

    if (!token.length() || *count >= LAN_SEQUENCE_MAX_KEYS) return false;

    int delay = token.indexOf('@');
    if (delay >= 0 && token.indexOf('@', delay + 1) >= 0) {
      return false;
    }

    int nameEnd = delay >= 0 ? delay : token.length();
    String name = token.substring(0, nameEnd);
    name.trim();
    const LanKey* key = findLanKey(name);
    if (!key) return false;

    uint32_t afterDelayMs = LAN_STEP_DELAY_MS;
    if (delay >= 0) {
      String delayDuration = token.substring(delay + 1);
      delayDuration.trim();
      if (!parseSequenceDelay(delayDuration, &afterDelayMs)) return false;
    }

    steps[*count] = {key, afterDelayMs};
    (*count)++;

    if (comma < 0) return true;
    start = end + 1;
  }

  return false;
}

static void runLanSequence(const String& payload) {
  LanSequenceStep steps[LAN_SEQUENCE_MAX_KEYS];
  size_t count = 0;

  if (!parseLanSequence(payload, steps, &count)) {
    Serial.println("Invalid LAN key sequence");
    publishState("sequence_invalid");
    return;
  }

  publishState("sequence_start");
  for (size_t i = 0; i < count; i++) {
    uint32_t afterDelayMs = i + 1 == count ? 0 : steps[i].afterDelayMs;
    if (!tapLanKey(*steps[i].key, afterDelayMs)) {
      publishState("sequence_key_failed");
      return;
    }
  }

  publishState("sequence_done");
}

static void macroHdmiLan(const char* inputName, uint8_t rightPresses) {
  char state[24];
  snprintf(state, sizeof(state), "%s_start", inputName);
  publishState(state);

  tapLanKey(KEY_UP);
  tapLanKey(KEY_UP);
  waitMs(HDMI_AFTER_UPS_DELAY_MS);

  for (uint8_t i = 0; i < rightPresses; i++) {
    tapLanKey(KEY_RIGHT, HDMI_RIGHT_STEP_DELAY_MS);
  }
  waitMs(HDMI_AFTER_RIGHTS_DELAY_MS);

  tapLanKey(KEY_OK, HDMI_AFTER_FIRST_OK_DELAY_MS);
  tapLanKey(KEY_OK, HDMI_AFTER_SECOND_OK_DELAY_MS);
  tapLanKey(KEY_OK, HDMI_AFTER_SECOND_OK_DELAY_MS);
  
  snprintf(state, sizeof(state), "%s_done", inputName);
  publishState(state);
}

static void macroHdmi1Lan() {
  macroHdmiLan("hdmi1", HDMI1_RIGHT_PRESSES);
}

static void macroHdmi2Lan() {
  publishState("hdmi2_start");

  tapLanKey(KEY_UP);
  tapLanKey(KEY_UP);
  waitMs(HDMI_AFTER_UPS_DELAY_MS);

  for (uint8_t i = 0; i < HDMI2_RIGHT_PRESSES; i++) {
    tapLanKey(KEY_RIGHT, HDMI_RIGHT_STEP_DELAY_MS);
  }
  waitMs(HDMI_AFTER_RIGHTS_DELAY_MS);

  tapLanKey(KEY_OK, HDMI_AFTER_FIRST_OK_DELAY_MS);
  tapLanKey(KEY_DOWN);
  tapLanKey(KEY_OK, HDMI_AFTER_SECOND_OK_DELAY_MS);

  publishState("hdmi2_done");
}

static void macroPowerOffLan() {
  publishState("power_off_start");

  tapLanKey(KEY_POWER_MENU, LAN_POWER_OFF_STEP_DELAY_MS);
  tapLanKey(KEY_DOWN, LAN_POWER_OFF_STEP_DELAY_MS);
  tapLanKey(KEY_OK, 0);

  publishState("power_off_done");
}

static void macroPowerOnHdmi1() {
  publishState("power_on_start");
  sendWakeAdvertisingBurst();

  Serial.printf("Waiting %lu ms before HDMI1 macro...\n", POWER_ON_TO_HDMI_DELAY_MS);
  publishState("power_on_wait");
  waitMs(POWER_ON_TO_HDMI_DELAY_MS);

  macroHdmi1Lan();
}

static void macroPowerOnHdmi2() {
  publishState("power_on_start");
  sendWakeAdvertisingBurst();

  Serial.printf("Waiting %lu ms before HDMI2 macro...\n", POWER_ON_TO_HDMI_DELAY_MS);
  publishState("power_on_wait");
  waitMs(POWER_ON_TO_HDMI_DELAY_MS);

  macroHdmi2Lan();
}

// ===================== BLE init =====================
static void startBleWakeAdvertiser() {
  NimBLEDevice::init("JMGO-Wake");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::getAdvertising()->stop();
}

// ===================== Wi-Fi + MQTT =====================
static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Wi-Fi connecting");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(WIFI_CONNECT_POLL_MS);
    Serial.print(".");
    if (millis() - t0 > WIFI_CONNECT_TIMEOUT_MS) break;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Wi-Fi OK, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi FAIL");
  }
}

static void ensureOtaReady() {
  if (WiFi.status() != WL_CONNECTED || otaStarted) return;

  ArduinoOTA.setHostname(OTA_HOST);
  ArduinoOTA.setPassword(OTA_PASS);

  ArduinoOTA.onStart([]() {
    const char* type = ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem";
    Serial.printf("OTA start: %s\n", type);
    publishState("ota_start");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA done");
    publishState("ota_done");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint8_t lastPct = 255;
    uint8_t pct = total ? (progress * 100U) / total : 0;
    if (pct != lastPct && (pct % 10 == 0 || pct == 100)) {
      Serial.printf("OTA progress: %u%%\n", pct);
      lastPct = pct;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error[%u]\n", error);
    publishState("ota_error");
  });

  ArduinoOTA.begin();
  otaStarted = true;

  Serial.print("OTA ready: ");
  Serial.println(OTA_HOST);
  Serial.print("OTA IP: ");
  Serial.println(WiFi.localIP());
}

static void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String t(topic);
  String msg;
  msg.reserve(len);
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();
  msg.toLowerCase();

  Serial.print("MQTT: "); Serial.print(t); Serial.print(" -> "); Serial.println(msg);

  if (t == TOPIC_SEQUENCE) {
    runLanSequence(msg);
    return;
  }

  if (t != TOPIC_CMD) return;

  if (msg == "wake") {
    sendWakeAdvertisingBurst();
  } else if (msg == "on" || msg == "wake_hdmi1" || msg == "power_on") {
    macroPowerOnHdmi1();
  } else if (msg == "wake_hdmi2") {
    macroPowerOnHdmi2();
  } else if (msg == "hdmi1") {
    macroHdmi1Lan();
  } else if (msg == "hdmi2") {
    macroHdmi2Lan();
  } else if (msg == "power_menu") {
    publishState("power_menu_start");
    tapLanKey(KEY_POWER_MENU, 0);
    publishState("power_menu_done");
  } else if (msg == "power_off" || msg == "off") {
    macroPowerOffLan();
  } else if (msg == "up") {
    tapLanKey(KEY_UP);
  } else if (msg == "down") {
    tapLanKey(KEY_DOWN);
  } else if (msg == "left") {
    tapLanKey(KEY_LEFT);
  } else if (msg == "right") {
    tapLanKey(KEY_RIGHT);
  } else if (msg == "ok" || msg == "enter") {
    tapLanKey(KEY_OK);
  } else if (msg == "volume_up") {
    tapLanKey(KEY_VOLUME_UP);
  } else if (msg == "volume_down") {
    tapLanKey(KEY_VOLUME_DOWN);
  } else if (msg == "back") {
    tapLanKey(KEY_BACK);
  } else if (msg == "menu") {
    tapLanKey(KEY_MENU);
  } else if (msg == "settings") {
    tapLanKey(KEY_SETTINGS);
  } else if (msg == "home") {
    tapLanKey(KEY_HOME);
  } else if (msg == "search") {
    tapLanKey(KEY_SEARCH);
  } else {
    publishState("unknown_cmd");
  }
}

static void mqttEnsureConnected() {
  if (mqtt.connected()) return;

  while (!mqtt.connected()) {
    Serial.print("MQTT connecting...");
    String clientId = "jmgo-remote-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    bool ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS);

    if (ok) {
      Serial.println("OK");
      mqtt.subscribe(TOPIC_CMD);
      mqtt.subscribe(TOPIC_SEQUENCE);
      publishState("online");
    } else {
      Serial.print("FAIL rc=");
      Serial.println(mqtt.state());
      delay(MQTT_RECONNECT_DELAY_MS);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(STARTUP_DELAY_MS);

  Serial.println();
  Serial.println("========================================");
  Serial.println("=== JMGO LAN + BLE WAKE REMOTE ===");
  Serial.printf("Build: %s %s\n", __DATE__, __TIME__);
  Serial.printf("Chip rev: %d, heap: %u\n", ESP.getChipRevision(), ESP.getFreeHeap());
  Serial.println("========================================");
  Serial.flush();

  Serial.println("Starting BLE wake advertiser...");
  startBleWakeAdvertiser();

  Serial.println("Starting WiFi...");
  connectWiFi();
  ensureOtaReady();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqttEnsureConnected();

  Serial.println("Ready. Publish a documented command to jmgo/remote/cmd");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    otaStarted = false;
    connectWiFi();
    ensureOtaReady();
  }
  mqttEnsureConnected();
  mqtt.loop();
  ArduinoOTA.handle();
}
