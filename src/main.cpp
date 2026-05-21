#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <LittleFS.h>
#include <ArduinoOTA.h>

#include <Preferences.h>

#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Force ElegantOTA to use AsyncWebServer (also set in platformio.ini build_flags)
#ifndef ELEGANTOTA_USE_ASYNC_WEBSERVER
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1
#endif
#include <ElegantOTA.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "esp_task_wdt.h"
#include "esp_idf_version.h"

// tds pinout: 6 pin : GND, btn, tds output, vcc, sda, scl
// ======================
// PINOUT (ESP32 CLASSIC)
// ======================
static const int PIN_I2C_SDA   = 21;
static const int PIN_I2C_SCL   = 22;

static const int PIN_BUTTON    = 27;   // momentary to GND, INPUT_PULLUP
static const int PIN_TDS_ADC   = 34;   // ADC input-only on ESP32
static const int PIN_DS18B20   = 4;    // OneWire data pin for waterproof DS18B20
static const int PIN_LEVEL_ODD_ADC  = 33; // SW1/SW3/SW5/SW7/SW9 ladder
static const int PIN_LEVEL_EVEN_ADC = 32; // SW2/SW4/SW6/SW8 ladder

// ======================
// OLED CONFIG
// ======================
static const int OLED_W = 128;
static const int OLED_H = 32;
static const uint8_t OLED_ADDR = 0x3C; // try 0x3D if blank

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

// ======================
// TIMINGS (NON-BLOCKING)
// ======================
static const uint32_t WELCOME_MS      = 5000;
static const uint32_t SHOW_TDS_MS     = 60000;
static const uint32_t OLED_REFRESH_MS = 500;
static const uint32_t TDS_SAMPLE_MS   = 1000;

// Button debounce
static const uint32_t DEBOUNCE_MS     = 40;

// ======================
// TDS CONFIG
// ======================
static const float DEFAULT_WATER_TEMP_C = 25.0f;
static float TDS_CALIBRATION = 1.0f;   // tune if needed

static float latestTdsPpm = NAN;
static float latestVoltage = NAN;
static uint32_t latestSampleMs = 0;
static float latestWaterTempC = NAN;
static uint32_t latestTempSampleMs = 0;
static float latestOddLadderVoltage = NAN;
static float latestEvenLadderVoltage = NAN;
static int latestOddSwitch = 0;
static int latestEvenSwitch = 0;
static String latestWaterLevelLabel = "Unknown";
static float latestWaterLevelInches = NAN;
static float latestWaterLevelPercent = NAN;
static float latestWaterVolumeLiters = NAN;
static uint32_t latestLevelSampleMs = 0;

// ======================
// DS18B20 CONFIG
// ======================
static const uint32_t TEMP_SAMPLE_MS = 2000;
static const uint32_t DS18B20_CONVERSION_MS = 750;
static const uint32_t WATER_LEVEL_SAMPLE_MS = 250;
static const float TANK_DIAMETER_IN = 15.0f;
static const float TANK_HEIGHT_IN = 22.0f;
static const float REED_SPACING_IN = 2.5f;
static const float CUBIC_INCH_TO_LITER = 0.016387064f;

OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensors(&oneWire);

static bool ds18b20Detected = false;
static bool tempConversionPending = false;
static uint32_t tLastTempRequestMs = 0;
static uint32_t tLastWaterLevelSampleMs = 0;

struct LadderTap {
  int switchNumber;
  float voltage;
};

static const LadderTap ODD_LADDER_TAPS[] = {
  {1, 1.467f},
  {3, 0.945f},
  {5, 0.665f},
  {7, 0.533f},
  {9, 0.399f},
};

static const LadderTap EVEN_LADDER_TAPS[] = {
  {2, 1.509f},
  {4, 0.961f},
  {6, 0.701f},
  {8, 0.538f},
};

static const float LADDER_MIN_VALID_V = 0.15f;
static const float LADDER_MATCH_TOLERANCE_V = 0.20f;

static const char* DEVICE_ID         = "ESP32-TDS-01";
static const char* DEVICE_HOSTNAME   = "tds";
static const char* MDNS_INSTANCE     = "TDS Sensor";
static const uint16_t OTA_PORT       = 3232;

// ======================
// WEB / OTA
// ======================
AsyncWebServer server(80);

// ======================
// STATE
// ======================
enum class OledMode { Off, Welcome, ShowTds };

static OledMode oledMode = OledMode::Welcome;
static uint32_t tModeStartMs = 0;
static uint32_t tLastOledRefreshMs = 0;
static uint32_t tLastTdsSampleMs = 0;

// Button debounce state
static int lastButtonStable = HIGH;
static int lastButtonRead   = HIGH;
static uint32_t tDebounceMs = 0;

static const uint32_t WIFI_WAIT_REFRESH_MS = 500; // refresh waiting screen
static bool wifiConnectedShown = false;  // whether we've switched from waiting->ip screen
static uint32_t tLastWifiWaitDrawMs = 0;

// ======================
// WiFi (non-blocking) + AP fallback
// ======================
static Preferences prefs;
static String wifiSsid;
static String wifiPass;

static bool apRunning = false;
static bool mdnsRunning = false;
static bool staWasConnected = false;
static IPAddress lastStaIp;

static uint32_t tLastStaAttemptMs = 0;
static const uint32_t STA_RETRY_MS = 5000;

static uint32_t staConnectStartMs = 0;
static const uint32_t STA_CONNECT_TIMEOUT_MS = 15000;

static uint32_t tLastApCheckMs = 0;
static const uint32_t AP_CHECK_MS = 3000;

static const char* AP_SSID = "TDS-SENSOR-SETUP";
static const char* AP_PASS = ""; // keep open, or set a password

// Restart scheduler (avoid calling ESP.restart() inside async callbacks)
static bool restartScheduled = false;
static uint32_t restartAtMs = 0;

// ======================
// Watchdog (auto restart if stuck)
// ======================
static const int WDT_TIMEOUT_S = 8;

static bool otaInProgress = false;

static float getWaterTempC() {
  return isnan(latestWaterTempC) ? DEFAULT_WATER_TEMP_C : latestWaterTempC;
}

static void setupTemperatureSensor() {
  tempSensors.begin();
  tempSensors.setWaitForConversion(false);

  const int deviceCount = tempSensors.getDeviceCount();
  ds18b20Detected = (deviceCount > 0);

  if (ds18b20Detected) {
    tempSensors.requestTemperatures();
    tempConversionPending = true;
    tLastTempRequestMs = millis();
    Serial.printf("[TEMP] DS18B20 ready on GPIO %d (%d sensor)\n", PIN_DS18B20, deviceCount);
  } else {
    Serial.printf("[TEMP] No DS18B20 found on GPIO %d. Using fallback %.1fC\n",
                  PIN_DS18B20,
                  DEFAULT_WATER_TEMP_C);
  }
}

static void sampleTemperatureNonBlocking() {
  if (!ds18b20Detected) return;

  const uint32_t now = millis();

  if (!tempConversionPending) {
    if (now - tLastTempRequestMs >= TEMP_SAMPLE_MS) {
      tempSensors.requestTemperatures();
      tempConversionPending = true;
      tLastTempRequestMs = now;
    }
    return;
  }

  if (now - tLastTempRequestMs < DS18B20_CONVERSION_MS) return;

  const float tempC = tempSensors.getTempCByIndex(0);
  tempConversionPending = false;

  if (tempC == DEVICE_DISCONNECTED_C || tempC < -55.0f || tempC > 125.0f) {
    Serial.println("[TEMP] DS18B20 read failed.");
    return;
  }

  latestWaterTempC = tempC;
  latestTempSampleMs = now;
}

static float readAdcVoltage(int pin, int samples = 8) {
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) sum += analogRead(pin);
  const float adc = (float)sum / (float)samples;
  return (adc * 3.3f) / 4095.0f;
}

static int decodeLadderSwitch(float voltage, const LadderTap* taps, size_t tapCount) {
  if (voltage < LADDER_MIN_VALID_V) return 0;

  float bestDiff = 1000.0f;
  int bestSwitch = 0;
  for (size_t i = 0; i < tapCount; i++) {
    const float diff = fabsf(voltage - taps[i].voltage);
    if (diff < bestDiff) {
      bestDiff = diff;
      bestSwitch = taps[i].switchNumber;
    }
  }

  if (bestDiff > LADDER_MATCH_TOLERANCE_V) return 0;
  return bestSwitch;
}

static float switchHeightInches(int switchNumber) {
  if (switchNumber < 1 || switchNumber > 9) return NAN;
  return TANK_HEIGHT_IN - ((float)(switchNumber - 1) * REED_SPACING_IN);
}

static float computeWaterLevelInches(int oddSwitch, int evenSwitch) {
  if (oddSwitch <= 0 && evenSwitch <= 0) return NAN;

  if (oddSwitch == 9 && evenSwitch == 8) return 0.0f;

  if (oddSwitch > 0 && evenSwitch > 0) {
    const float oddHeight = switchHeightInches(oddSwitch);
    const float evenHeight = switchHeightInches(evenSwitch);
    return (oddHeight + evenHeight) * 0.5f;
  }

  if (oddSwitch > 0) return switchHeightInches(oddSwitch);
  return switchHeightInches(evenSwitch);
}

static float computeWaterLevelPercent(float waterHeightInches) {
  if (isnan(waterHeightInches)) return NAN;
  const float clampedHeight = constrain(waterHeightInches, 0.0f, TANK_HEIGHT_IN);
  return (clampedHeight / TANK_HEIGHT_IN) * 100.0f;
}

static float computeWaterVolumeLiters(float waterHeightInches) {
  if (isnan(waterHeightInches)) return NAN;
  const float clampedHeight = constrain(waterHeightInches, 0.0f, TANK_HEIGHT_IN);
  const float radiusIn = TANK_DIAMETER_IN * 0.5f;
  const float volumeCubicInches = PI * radiusIn * radiusIn * clampedHeight;
  return volumeCubicInches * CUBIC_INCH_TO_LITER;
}

static void updateWaterLevelState(int oddSwitch, int evenSwitch) {
  String label = "No switch active";

  if (oddSwitch > 0 && evenSwitch > 0) {
    if (oddSwitch == 9 && evenSwitch == 8) {
      label = "Tank Empty";
    } else {
    const int upper = min(oddSwitch, evenSwitch);
    const int lower = max(oddSwitch, evenSwitch);

      if (lower - upper == 1) {
        label = "Between SW" + String(upper) + " and SW" + String(lower);
      } else {
        label = "SW" + String(oddSwitch) + " + SW" + String(evenSwitch);
      }
    }
  } else if (oddSwitch > 0) {
    label = "At SW" + String(oddSwitch);
  } else if (evenSwitch > 0) {
    label = "At SW" + String(evenSwitch);
  }

  const float waterHeightInches = computeWaterLevelInches(oddSwitch, evenSwitch);

  latestWaterLevelLabel = label;
  latestWaterLevelInches = waterHeightInches;
  latestWaterLevelPercent = computeWaterLevelPercent(waterHeightInches);
  latestWaterVolumeLiters = computeWaterVolumeLiters(waterHeightInches);
}

static void sampleWaterLevelNonBlocking() {
  const uint32_t now = millis();
  if (now - tLastWaterLevelSampleMs < WATER_LEVEL_SAMPLE_MS) return;
  tLastWaterLevelSampleMs = now;

  latestOddLadderVoltage = readAdcVoltage(PIN_LEVEL_ODD_ADC);
  latestEvenLadderVoltage = readAdcVoltage(PIN_LEVEL_EVEN_ADC);

  latestOddSwitch = decodeLadderSwitch(
    latestOddLadderVoltage,
    ODD_LADDER_TAPS,
    sizeof(ODD_LADDER_TAPS) / sizeof(ODD_LADDER_TAPS[0])
  );
  latestEvenSwitch = decodeLadderSwitch(
    latestEvenLadderVoltage,
    EVEN_LADDER_TAPS,
    sizeof(EVEN_LADDER_TAPS) / sizeof(EVEN_LADDER_TAPS[0])
  );

  updateWaterLevelState(latestOddSwitch, latestEvenSwitch);
  latestLevelSampleMs = now;
}

// ---------------- OLED helpers ----------------
static void oledPower(bool on) {
  if (display.width() == 0) return;

  if (on) {
    display.ssd1306_command(SSD1306_DISPLAYON);
  } else {
    display.clearDisplay();
    display.display();
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}

static void oledDrawWelcome() {
  if (display.width() == 0) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Welcome!");
  display.println("TDS Sensor Ready");
  display.println("Press button...");
  display.display();
}

static void oledDrawWaitingWiFi() {
  if (display.width() == 0) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Welcome!");
  display.println("Waiting for WiFi...");
  display.print("AP: ");
  display.println(AP_SSID);
  display.display();
}

static void oledDrawIP(const IPAddress& ip) {
  if (display.width() == 0) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("WiFi Connected!");
  display.print("IP: ");
  display.println(ip.toString());
  display.println("OTA: /update");
  display.display();
}

static void oledDrawTds() {
  if (display.width() == 0) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("TDS Meter");

  display.setTextSize(2);
  display.setCursor(0, 12);
  if (isnan(latestTdsPpm)) display.print("--");
  else display.print((int)roundf(latestTdsPpm));
  display.print(" ppm");

  display.setTextSize(1);
  display.setCursor(92, 0);
  if (isnan(latestVoltage)) {
    display.print("--.-V");
  } else {
    display.print(latestVoltage, 2);
    display.print("V");
  }

  display.display();
}

static void setOledMode(OledMode mode) {
  oledMode = mode;
  tModeStartMs = millis();

  if (mode == OledMode::Off) {
    oledPower(false);
  } else {
    oledPower(true);
    if (mode == OledMode::Welcome) oledDrawWelcome();
    if (mode == OledMode::ShowTds) oledDrawTds();
  }
}

static void handleOledState() {
  uint32_t now = millis();

  if (oledMode == OledMode::Welcome) {
    if (WiFi.status() != WL_CONNECTED) {
      if (now - tLastWifiWaitDrawMs >= WIFI_WAIT_REFRESH_MS) {
        tLastWifiWaitDrawMs = now;
        oledPower(true);
        oledDrawWaitingWiFi();
      }
      return;
    }

    if (!wifiConnectedShown) {
      wifiConnectedShown = true;
      tModeStartMs = now;
      oledPower(true);
      oledDrawIP(WiFi.localIP());
    }

    if (now - tModeStartMs >= WELCOME_MS) {
      setOledMode(OledMode::Off);
    }

  } else if (oledMode == OledMode::ShowTds) {
    if (now - tModeStartMs >= SHOW_TDS_MS) {
      setOledMode(OledMode::Off);
      return;
    }

    if (now - tLastOledRefreshMs >= OLED_REFRESH_MS) {
      tLastOledRefreshMs = now;
      oledDrawTds();
    }
  }
}

// ---------------- Button (pressed event) ----------------
static bool buttonPressedEvent() {
  uint32_t now = millis();
  int reading = digitalRead(PIN_BUTTON);

  if (reading != lastButtonRead) {
    tDebounceMs = now;
    lastButtonRead = reading;
  }

  if (now - tDebounceMs > DEBOUNCE_MS) {
    if (reading != lastButtonStable) {
      lastButtonStable = reading;
      if (lastButtonStable == LOW) return true;
    }
  }
  return false;
}

// ---------------- TDS math ----------------
static float computeTdsPpmFromVoltage(float voltage, float tempC) {
  float compensationCoefficient = 1.0f + 0.02f * (tempC - 25.0f);
  float compensatedVoltage = voltage / compensationCoefficient;

  float v = compensatedVoltage;
  float tds = (133.42f * v * v * v - 255.86f * v * v + 857.39f * v) * 0.5f;
  return tds * TDS_CALIBRATION;
}

static void sampleTdsNonBlocking() {
  uint32_t now = millis();
  if (now - tLastTdsSampleMs < TDS_SAMPLE_MS) return;
  tLastTdsSampleMs = now;

  const int N = 8;
  uint32_t sum = 0;
  for (int i = 0; i < N; i++) sum += analogRead(PIN_TDS_ADC);
  float adc = (float)sum / (float)N;

  float voltage = (adc * 3.3f) / 4095.0f;
  float ppm = computeTdsPpmFromVoltage(voltage, getWaterTempC());

  latestVoltage = voltage;
  latestTdsPpm = ppm;
  latestSampleMs = now;
}

// ======================
// WiFi storage
// ======================
static void loadWifiCreds() {
  prefs.begin("wifi", true);
  wifiSsid = prefs.getString("ssid", "");
  wifiPass = prefs.getString("pass", "");
  prefs.end();
}

static void saveWifiCreds(const String& ssid, const String& pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

static void clearWifiCreds() {
  prefs.begin("wifi", false);
  prefs.remove("ssid");
  prefs.remove("pass");
  prefs.end();
  wifiSsid = "";
  wifiPass = "";
}

// ======================
// AP control
// ======================
static void startAP() {
  if (apRunning) return;

  WiFi.mode(WIFI_AP_STA);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  apRunning = ok;

  Serial.printf("[WiFi] AP %s. SSID=%s IP=%s\n",
                ok ? "STARTED" : "FAILED",
                AP_SSID,
                WiFi.softAPIP().toString().c_str());
}

static void stopAP() {
  if (!apRunning) return;
  WiFi.softAPdisconnect(true);
  apRunning = false;
  Serial.println("[WiFi] AP stopped (STA connected).");
}

static void stopMdns() {
  if (!mdnsRunning) return;

  MDNS.end();
  mdnsRunning = false;
  Serial.println("[mDNS] Stopped.");
}

static void startOrRestartMdns(const IPAddress& ip) {
  if (mdnsRunning) {
    MDNS.end();
    mdnsRunning = false;
  }

  if (!MDNS.begin(DEVICE_HOSTNAME)) {
    Serial.println("[mDNS] Failed to start.");
    return;
  }

  MDNS.setInstanceName(MDNS_INSTANCE);
  MDNS.addService("http", "tcp", 80);
  MDNS.addServiceTxt("http", "tcp", "path", "/");
  MDNS.enableArduino(OTA_PORT, false);

  mdnsRunning = true;
  Serial.printf("[mDNS] Ready: http://%s.local/ (IP=%s)\n",
                DEVICE_HOSTNAME,
                ip.toString().c_str());
}

static void syncMdnsState() {
  const bool staConnected = (WiFi.status() == WL_CONNECTED);
  const IPAddress currentIp = staConnected ? WiFi.localIP() : IPAddress((uint32_t)0);

  if (staConnected) {
    if (!staWasConnected || currentIp != lastStaIp) {
      startOrRestartMdns(currentIp);
    }
  } else if (staWasConnected) {
    stopMdns();
  }

  staWasConnected = staConnected;
  lastStaIp = currentIp;
}

// ======================
// STA connect/reconnect (non-blocking)
// ======================
static void attemptStaConnect() {
  if (wifiSsid.length() == 0) {
    startAP();
    return;
  }

  if (WiFi.getMode() != WIFI_STA && WiFi.getMode() != WIFI_AP_STA) {
    WiFi.mode(WIFI_AP_STA);
  }

  Serial.printf("[WiFi] STA connect attempt: %s\n", wifiSsid.c_str());
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  staConnectStartMs = millis();
}

static void wifiLoop() {
  const uint32_t now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    stopAP();
    return;
  }

  if (now - tLastApCheckMs >= AP_CHECK_MS) {
    tLastApCheckMs = now;
    startAP();
  }

  if (now - tLastStaAttemptMs >= STA_RETRY_MS) {
    tLastStaAttemptMs = now;
    attemptStaConnect();
  }

  if (staConnectStartMs != 0 && (now - staConnectStartMs > STA_CONNECT_TIMEOUT_MS)) {
    staConnectStartMs = 0;
  }
}

// ======================
// Watchdog
// ======================
static void setupWatchdog() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WDT_TIMEOUT_S * 1000;
  cfg.trigger_panic = true;
  cfg.idle_core_mask = 0;
  esp_task_wdt_init(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
}

static inline void feedWatchdog() {
  esp_task_wdt_reset();
}

// ======================
// Restart scheduler (safe)
// ======================
static void scheduleRestart(uint32_t delayMs = 250) {
  restartScheduled = true;
  restartAtMs = millis() + delayMs;
}

static void handleScheduledRestart() {
  if (!restartScheduled) return;
  if ((int32_t)(millis() - restartAtMs) >= 0) {
    Serial.println("[SYS] Restarting...");
    ESP.restart();
  }
}

// ======================
// Web
// ======================
static void setupWeb() {
  if (!LittleFS.begin(true)) {
    Serial.println("[LittleFS] Mount failed (even after format). Web UI will be unavailable.");
  } else {
    Serial.println("[LittleFS] Mounted.");
  }

  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "OK");
  });

  server.on("/api/tds", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;

    if (isnan(latestTdsPpm))   doc["tds_ppm"] = nullptr;
    else                       doc["tds_ppm"] = latestTdsPpm;

    if (isnan(latestVoltage))  doc["voltage"] = nullptr;
    else                       doc["voltage"] = latestVoltage;

    if (latestSampleMs == 0)   doc["age_ms"] = nullptr;
    else                       doc["age_ms"] = (uint32_t)(millis() - latestSampleMs);

    doc["uptime_ms"] = (uint32_t)millis();
    if (isnan(latestWaterTempC)) doc["temp_c"] = nullptr;
    else                         doc["temp_c"] = latestWaterTempC;

    if (latestTempSampleMs == 0) doc["temp_age_ms"] = nullptr;
    else                         doc["temp_age_ms"] = (uint32_t)(millis() - latestTempSampleMs);

    doc["temp_fallback_c"] = DEFAULT_WATER_TEMP_C;
    doc["water_level_label"] = latestWaterLevelLabel;

    if (isnan(latestWaterLevelInches)) doc["water_level_inches"] = nullptr;
    else                               doc["water_level_inches"] = latestWaterLevelInches;

    if (isnan(latestWaterLevelPercent)) doc["water_level_percent"] = nullptr;
    else                                doc["water_level_percent"] = latestWaterLevelPercent;

    if (isnan(latestWaterVolumeLiters)) doc["water_volume_liters"] = nullptr;
    else                                doc["water_volume_liters"] = latestWaterVolumeLiters;

    doc["tank_height_inches"] = TANK_HEIGHT_IN;
    doc["tank_diameter_inches"] = TANK_DIAMETER_IN;
    doc["tank_capacity_liters"] = computeWaterVolumeLiters(TANK_HEIGHT_IN);

    if (isnan(latestOddLadderVoltage))  doc["ladder_odd_voltage"] = nullptr;
    else                                doc["ladder_odd_voltage"] = latestOddLadderVoltage;

    if (isnan(latestEvenLadderVoltage)) doc["ladder_even_voltage"] = nullptr;
    else                                doc["ladder_even_voltage"] = latestEvenLadderVoltage;

    doc["ladder_odd_switch"] = latestOddSwitch;
    doc["ladder_even_switch"] = latestEvenSwitch;

    if (latestLevelSampleMs == 0) doc["water_level_age_ms"] = nullptr;
    else                          doc["water_level_age_ms"] = (uint32_t)(millis() - latestLevelSampleMs);

    String out;
    serializeJson(doc, out);

    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", out);
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
  });

  server.on("/setup", HTTP_GET, [](AsyncWebServerRequest *request) {
    const char* html =
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>WiFi Setup</title></head>"
      "<body style='font-family:Arial,sans-serif;padding:16px'>"
      "<h2>Configure WiFi</h2>"
      "<p>Enter SSID and password. Device will restart.</p>"
      "<form method='POST' action='/save'>"
      "<label>SSID</label><br><input name='s' style='width:100%;padding:10px' required><br><br>"
      "<label>Password</label><br><input name='p' type='password' style='width:100%;padding:10px'><br><br>"
      "<button style='padding:10px 16px'>Save & Restart</button>"
      "</form>"
      "<hr><form method='POST' action='/forget'>"
      "<button style='padding:10px 16px'>Forget WiFi</button>"
      "</form>"
      "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("s", true)) {
      request->send(400, "text/plain", "Missing ssid");
      return;
    }
    String s = request->getParam("s", true)->value();
    String p = request->hasParam("p", true) ? request->getParam("p", true)->value() : "";

    saveWifiCreds(s, p);
    request->send(200, "text/plain", "Saved. Rebooting...");
    scheduleRestart(300);
  });

  server.on("/forget", HTTP_POST, [](AsyncWebServerRequest *request) {
    clearWifiCreds();
    request->send(200, "text/plain", "Forgot WiFi. Rebooting...");
    scheduleRestart(300);
  });

  server.serveStatic("/", LittleFS, "/")
        .setDefaultFile("index.html")
        .setCacheControl("no-store, no-cache, must-revalidate");

  ElegantOTA.begin(&server);

  server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not Found");
  });

  server.begin();
  Serial.println("[Web] Async web server started on port 80");
}

// ======================
// OTA (ArduinoOTA, non-blocking)
// ======================
static void setupArduinoOta() {
  ArduinoOTA.setHostname(DEVICE_HOSTNAME);
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setMdnsEnabled(false);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    Serial.println("[OTA] Start");
    esp_task_wdt_delete(NULL);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] End");
    setupWatchdog();
    otaInProgress = false;
  });

  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("[OTA] Error: %u\n", e);
    setupWatchdog();
    otaInProgress = false;
  });

  ArduinoOTA.begin();
}

void setup() {
  Serial.begin(115200);
  delay(30);

  setupWatchdog();

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_DS18B20, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_TDS_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_LEVEL_ODD_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_LEVEL_EVEN_ADC, ADC_11db);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] init failed. Try addr 0x3D or check wiring.");
  } else {
    display.clearDisplay();
    display.display();
  }

  setOledMode(OledMode::Welcome);
  wifiConnectedShown = false;
  tLastWifiWaitDrawMs = 0;

  setupTemperatureSensor();

  loadWifiCreds();
  WiFi.mode(WIFI_AP_STA);
  startAP();
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  attemptStaConnect();

  setupArduinoOta();
  setupWeb();

  Serial.println("[BOOT] Ready.");
  Serial.printf("UI:   http://%s.local/  (or AP http://192.168.4.1/ )\n", DEVICE_HOSTNAME);
  Serial.printf("Setup http://%s.local/setup  (or http://192.168.4.1/setup)\n", DEVICE_HOSTNAME);
  Serial.printf("OTA:  http://%s.local/update\n", DEVICE_HOSTNAME);
}

void loop() {
  feedWatchdog();

  handleScheduledRestart();
  wifiLoop();
  syncMdnsState();

  sampleTemperatureNonBlocking();
  sampleWaterLevelNonBlocking();
  sampleTdsNonBlocking();
  handleOledState();

  if (buttonPressedEvent()) {
    setOledMode(OledMode::ShowTds);
  }

  ArduinoOTA.handle();
}
