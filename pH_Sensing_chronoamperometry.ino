/*
 * 8x LMP91000 + CD4051 mux + ESP32  ->  ThingSpeak (WiFi), no PC / Serial Monitor needed
 * Sensor 1 selected (mux X0, A=B=C=0)
 *
 * ThingSpeak channel fields:
 *   Field 1 = VOUT (mV)
 *   Field 2 = cell current (uA)
 *   Field 3 = pH (only sent once calibration values below are filled in)
 *   Field 4 = sensor number
 *
 * Pins (from your PCB, ESP32 DevKit V1):
 *   MUX_A=25  MUX_B=26  MUX_C=32  MUX_OUT=33 (ADC1_CH5 - OK with WiFi on)
 *   MENB (LMP1..8) = 27,13,14,4,5,18,19,23   SDA=21  SCL=22   TPS_En=2
 *
 * NOTE: ESP32 ADC2 does not work while WiFi is on. GPIO33 is ADC1, so it is fine.
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ============================ USER SETTINGS ===============================
const char* WIFI_SSID = "Aditya";
const char* WIFI_PASS = "Aditya10";
const char* TS_WRITE_API_KEY = "9B6OB5452DBHIQ2I";

const uint8_t  SENSOR_NUM      = 1;        // 1..8  (1 -> mux X0)
const uint32_t UPLOAD_MS       = 20000;    // ThingSpeak free tier: min 15 s
const uint32_t SETTLE_MS       = 30000;    // sensor conditioning after bias is applied

// ---- pH calibration (fill in after reading Field 2 in two buffers) ----
// Put the sensor in pH 4 buffer, wait until stable, note Field 2 (uA) -> CAL_LOW_UA
// Repeat with pH 7 buffer                                         -> CAL_HIGH_UA
float CAL_LOW_UA  = 0.0f;   float CAL_LOW_PH  = 4.00f;
float CAL_HIGH_UA = 0.0f;   float CAL_HIGH_PH = 7.00f;
// ===========================================================================

// ---------------------------- PINS ------------------------------------
const uint8_t PIN_MUX_A = 25, PIN_MUX_B = 26, PIN_MUX_C = 32, PIN_MUX_OUT = 33;
const uint8_t PIN_TPS_EN = 2, PIN_SDA = 21, PIN_SCL = 22;
const uint8_t LMP_EN[8] = {27, 13, 14, 4, 5, 18, 19, 23};

// ------------------------ LMP91000 REGISTERS ---------------------------
const uint8_t LMP_ADDR = 0x48;
const uint8_t REG_STATUS = 0x00, REG_LOCK = 0x01, REG_TIACN = 0x10,
              REG_REFCN = 0x11, REG_MODECN = 0x12;

// TIA_GAIN: 1=2.75k 2=3.5k 3=7k 4=14k 5=35k 6=120k 7=350k    RLOAD: 0=10R 1=33R 2=50R 3=100R
const float   TIA_OHMS[8] = {0, 2750, 3500, 7000, 14000, 35000, 120000, 350000};
const uint8_t TIA_GAIN_CODE = 5;
const uint8_t RLOAD_CODE    = 3;

// INT_Z: 0=20% 1=50% 2=67%   BIAS: 0=0%,1=1%,2=2%,3=4%,4=6%,5=8%,6=10%,7=12%,8=14%,9=16%,10=18%,11=20%,12=22%,13=24%
const uint8_t REF_SOURCE = 0;      // internal (VDD)
const uint8_t INT_Z_CODE = 1;      // 50 %
const uint8_t BIAS_SIGN  = 1;      // 1 = positive
const uint8_t BIAS_CODE  = 0;      // 0 mV
const float   INT_Z_FRAC[3] = {0.20f, 0.50f, 0.67f};
const uint8_t MODE_3LEAD = 0x03;

const float   VDD_AFE_V   = 3.3f;
const uint8_t ADC_SAMPLES = 64;

// ------------------------------ I2C ------------------------------------
bool lmpWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LMP_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool lmpRead(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(LMP_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)LMP_ADDR, 1) != 1) return false;
  val = Wire.read();
  return true;
}

void enableOnly(int idx) {                       // MENB is active LOW
  for (int i = 0; i < 8; i++) digitalWrite(LMP_EN[i], HIGH);
  if (idx >= 0) digitalWrite(LMP_EN[idx], LOW);
  delayMicroseconds(5);
}

void muxSelect(uint8_t ch) {                     // ch 0..7 (sensor 1 -> 0)
  digitalWrite(PIN_MUX_A, ch & 1);
  digitalWrite(PIN_MUX_B, (ch >> 1) & 1);
  digitalWrite(PIN_MUX_C, (ch >> 2) & 1);
  delay(5);
}

bool configureLMP(uint8_t sensorNum) {
  enableOnly(sensorNum - 1);

  uint8_t st = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    if (lmpRead(REG_STATUS, st) && (st & 1)) break;
    delay(10);
  }
  if (!(st & 1)) { enableOnly(-1); return false; }

  bool ok = true;
  ok &= lmpWrite(REG_LOCK, 0x00);
  ok &= lmpWrite(REG_TIACN, (TIA_GAIN_CODE << 2) | RLOAD_CODE);
  ok &= lmpWrite(REG_REFCN, (REF_SOURCE << 7) | (INT_Z_CODE << 5) | (BIAS_SIGN << 4) | BIAS_CODE);
  ok &= lmpWrite(REG_LOCK, 0x01);
  ok &= lmpWrite(REG_MODECN, MODE_3LEAD);

  enableOnly(-1);
  return ok;
}

// --------------------------- MEASUREMENT -------------------------------
float readVoutMilliVolts() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    sum += analogReadMilliVolts(PIN_MUX_OUT);
    delayMicroseconds(200);
  }
  return (float)sum / ADC_SAMPLES;
}

float cellCurrent_uA(float vout_mV) {
  float vzero_mV = INT_Z_FRAC[INT_Z_CODE] * VDD_AFE_V * 1000.0f;
  return ((vout_mV - vzero_mV) / 1000.0f) / TIA_OHMS[TIA_GAIN_CODE] * 1e6f;
}

bool calibrated() { return fabsf(CAL_HIGH_UA - CAL_LOW_UA) > 1e-6f; }

float currentToPH(float i_uA) {
  float slope = (CAL_HIGH_PH - CAL_LOW_PH) / (CAL_HIGH_UA - CAL_LOW_UA);
  return CAL_LOW_PH + slope * (i_uA - CAL_LOW_UA);
}

// ------------------------------ WIFI -----------------------------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
}

bool uploadThingSpeak(float vout_mV, float i_uA, bool hasPH, float pH) {
  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) return false;

  String url = "http://api.thingspeak.com/update?api_key=";
  url += TS_WRITE_API_KEY;
  url += "&field1=" + String(vout_mV, 1);
  url += "&field2=" + String(i_uA, 4);
  if (hasPH) url += "&field3=" + String(pH, 2);
  url += "&field4=" + String(SENSOR_NUM);

  HTTPClient http;
  http.begin(url);
  int code = http.GET();                  // 200 = OK; response body = entry id (0 = rejected)
  String body = http.getString();
  http.end();
  return (code == 200 && body != "0");
}

// ------------------------------ SETUP ----------------------------------
void setup() {
  Serial.begin(115200);                   // optional - harmless if no PC is attached

  for (int i = 0; i < 8; i++) { pinMode(LMP_EN[i], OUTPUT); digitalWrite(LMP_EN[i], HIGH); }
  pinMode(PIN_MUX_A, OUTPUT); pinMode(PIN_MUX_B, OUTPUT); pinMode(PIN_MUX_C, OUTPUT);

  pinMode(PIN_TPS_EN, OUTPUT);
  digitalWrite(PIN_TPS_EN, HIGH);         // power the +3V3_AFE rail (LMP91000s, mux, I2C pull-ups)
  delay(100);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MUX_OUT, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL, 100000);

  bool ok = configureLMP(SENSOR_NUM);
  Serial.println(ok ? "LMP91000 configured" : "LMP91000 config FAILED");
  muxSelect(SENSOR_NUM - 1);

  connectWiFi();
  delay(SETTLE_MS);                       // sensor conditioning
}

// ------------------------------- LOOP ----------------------------------
void loop() {
  float vout = readVoutMilliVolts();      // measure first (before WiFi TX bursts)
  float i_uA = cellCurrent_uA(vout);
  bool  hasPH = calibrated();
  float pH = hasPH ? currentToPH(i_uA) : 0;

  bool sent = uploadThingSpeak(vout, i_uA, hasPH, pH);
  Serial.printf("S%u VOUT=%.1f mV I=%.4f uA pH=%.2f upload=%s\n",
                SENSOR_NUM, vout, i_uA, pH, sent ? "OK" : "FAIL");

  delay(UPLOAD_MS);
}

