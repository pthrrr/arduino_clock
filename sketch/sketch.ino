// WeMos D1 mini (ESP8266) + 4x MAX7219 LED matrix + LDR on A0
// - Wi-Fi from secrets.h (not committed)
// - NTP time with Berlin timezone (CET/CEST DST)
// - OTA updates via ArduinoOTA
// - Auto-brightness from A0 with smoothing/hysteresis
//
// Wiring (matrix):
//   VCC -> 3V3 (keep brightness low) or 5V (preferred if available)
//   GND -> G
//   DIN -> D7
//   CS  -> D4
//   CLK -> D5
// LDR divider output -> A0

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <math.h>
#include "secrets.h"  // Define WIFI_SSID, WIFI_PASS, OTA_HOSTNAME, OTA_PASSWORD
#include "p_font.h"

// ---------- Display configuration ----------
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW  // Try ICSTATION_HW if text looks wrong
#define MAX_DEVICES   4
#define CS_PIN        D4

MD_Parola matrix(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// ---------- NTP / Timezone ----------
const char* NTP1 = "pool.ntp.org";
const char* NTP2 = "time.cloudflare.com";
const char* NTP3 = "time.google.com";
// Berlin timezone with DST
const char* TZ_INFO = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// ---------- Brightness control (A0 light sensor) ----------
const bool     LDR_INVERTED    = false;  // set true if bright light gives lower ADC values
const int      LDR_MIN_RAW     = 0;      // clamp range
const int      LDR_MAX_RAW     = 1023;

// Matrix powered from 3.3V: keep brightness low (0..15 allowed by MAX7219)
const uint8_t  BRIGHTNESS_MIN  = 0;
const uint8_t  BRIGHTNESS_MAX  = 10;      // safe for 3V3; raise carefully if stable
const float    LPF_ALPHA       = 0.08f;  // ADC smoothing (0..1), lower = smoother
const uint8_t  BRIGHTNESS_HYST = 1;      // min steps change to apply new brightness
const uint16_t LDR_SAMPLE_MS   = 80;     // ADC sampling interval (ms)

// ---------- Behavior ----------
int lastShownSecond = -1;
uint8_t currentBrightness = 2;
float ldrFiltered = -1.0f;
bool otaStarted = false;

// ---------- Helpers ----------
static inline int clampInt(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

uint8_t ldrToBrightness()
{
  int raw = analogRead(A0);
  raw = clampInt(raw, LDR_MIN_RAW, LDR_MAX_RAW);
  if (LDR_INVERTED) raw = LDR_MAX_RAW - (raw - LDR_MIN_RAW);

  if (ldrFiltered < 0) ldrFiltered = raw;  // initialize filter
  ldrFiltered = (LPF_ALPHA * raw) + (1.0f - LPF_ALPHA) * ldrFiltered;

  float n = (ldrFiltered - LDR_MIN_RAW) / float(LDR_MAX_RAW - LDR_MIN_RAW);
  if (n < 0) n = 0; if (n > 1) n = 1;

  const float gamma = 1.6f;  // >1 dims low light levels for readability
  float g = powf(n, gamma);

  int b = int(BRIGHTNESS_MIN + g * (BRIGHTNESS_MAX - BRIGHTNESS_MIN) + 0.5f);
  b = clampInt(b, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
  return uint8_t(b);
}

void applyAutoBrightness()
{
  static uint32_t lastSample = 0;
  if (millis() - lastSample < LDR_SAMPLE_MS) return;
  lastSample = millis();

  uint8_t target = ldrToBrightness();
  if (abs(int(target) - int(currentBrightness)) >= BRIGHTNESS_HYST) {
    currentBrightness = target;
    matrix.setIntensity(currentBrightness);
  }
}

void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  // Optional: set DHCP hostname to match OTA hostname if defined
#ifdef OTA_HOSTNAME
  WiFi.hostname(OTA_HOSTNAME);
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
  }
}

bool initTime(uint32_t timeoutMs = 20000)
{
  configTime(0, 0, NTP1, NTP2, NTP3);  // get UTC via NTP
  setenv("TZ", TZ_INFO, 1);
  tzset();

  time_t now = time(nullptr);
  uint32_t start = millis();
  while ((now < 1609459200) && (millis() - start < timeoutMs)) {  // wait until >= 2021-01-01
    delay(200);
    now = time(nullptr);
  }
  return (now >= 1609459200);
}

void ensureOTAStarted()
{
  if (otaStarted || WiFi.status() != WL_CONNECTED) return;

#ifdef OTA_HOSTNAME
  ArduinoOTA.setHostname(OTA_HOSTNAME);
#endif
#ifdef OTA_PASSWORD
  if (OTA_PASSWORD[0] != '\0') {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }
#endif

  ArduinoOTA.onStart([]() {
    matrix.displayClear();
    matrix.setFont(nullptr);
    matrix.setCharSpacing(1);
    matrix.setTextAlignment(PA_CENTER);
    matrix.print("UPD");  // indicate OTA update
  });
  // onEnd/onError handlers optional

  ArduinoOTA.begin();
  otaStarted = true;
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  matrix.begin();
  //matrix.setZoneEffect(0, true, PA_FLIP_UD); // flip upside down
  //matrix.setZoneEffect(0, true, PA_FLIP_LR); // flip left-right (together = 180° rotation)
  matrix.displayClear();
  matrix.setIntensity(currentBrightness);
  matrix.setTextAlignment(PA_CENTER);
  matrix.print("WiFi");

  connectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi IP: ");
    Serial.println(WiFi.localIP());
    matrix.print("NTP");
  } else {
    Serial.println("WiFi connect timeout; will auto-retry.");
    matrix.print("NET?");
  }

  bool timeOK = initTime();
  if (!timeOK) {
    Serial.println("NTP/time not ready yet; continuing, will catch up.");
  }

  ensureOTAStarted();  // start OTA if WiFi is up
  matrix.displayClear();
  matrix.setFont(p_Font);
  //matrix.setCharSpacing(2);
}

void loop()
{
  // Keep WiFi and OTA alive
  static uint32_t lastWiFiKick = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastWiFiKick > 15000) {
    lastWiFiKick = millis();
    connectWiFi();
    otaStarted = false;  // re-init OTA after reconnect
  }
  ensureOTAStarted();
  if (otaStarted) ArduinoOTA.handle();  // non-blocking

  // Adjust brightness from LDR
  applyAutoBrightness();

  // Get local time
  time_t now = time(nullptr);
  struct tm tmLocal;
  localtime_r(&now, &tmLocal);

  // Update the clock once per second (blink the colon)
  if (tmLocal.tm_sec != lastShownSecond) {
    lastShownSecond = tmLocal.tm_sec;

    char buf[6];  // "HH:MM" + NUL
    char colon = (tmLocal.tm_sec % 2 == 0) ? ':' : ' ';

    if (now < 1609459200) {
      snprintf(buf, sizeof(buf), "--:--");
    } else {
      snprintf(buf, sizeof(buf), "%02d%c%02d", tmLocal.tm_hour, colon, tmLocal.tm_min);
    }
    matrix.print(buf);
  }
}
