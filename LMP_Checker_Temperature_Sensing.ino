/*
 * LMP91000 TEMPERATURE TEST  (proves the chip, I2C, MENB, mux and ADC all work)
 * ESP32 + CD4051 mux + ThingSpeak.  No pH sensor needed.
 *
 * Tests sensor number SENSOR_NUM (default 1 -> LMP1, mux X0).
 *
 * ThingSpeak fields used by this test:
 *   Field 1 = VOUT (mV)
 *   Field 2 = temperature (degC)          <-- (normally "current" in the pH sketch)
 *   Field 3 = MODECN register read-back   (should be 6 when temperature mode is active)
 *   Field 4 = sensor number
 *
 * Expected at room temperature (25-30 degC): VOUT ~ 1320-1360 mV
 * Warm the chip with a finger: VOUT should drop ~8.2 mV per degC.
 *
 * Pins (ESP32 DevKit V1, from your PCB):
 *   MUX_A=25  MUX_B=26  MUX_C=32  MUX_OUT=33 (ADC1)
 *   MENB (LMP1..8) = 27,13,14,4,5,18,19,23   SDA=21  SCL=22   TPS_En=2
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ============================ USER SETTINGS ===============================
const char* WIFI_SSID = "Aditya";
const char* WIFI_PASS = "Aditya10";
const char* TS_WRITE_API_KEY = "RLMO4S32WY4P99IW";

const uint8_t  SENSOR_NUM = 1;        // 1..8
const uint32_t UPLOAD_MS  = 16000;    // ThingSpeak free tier: min 15 s
// ===========================================================================

// ---------------------------- PINS ------------------------------------
const uint8_t PIN_MUX_A = 25, PIN_MUX_B = 26, PIN_MUX_C = 32, PIN_MUX_OUT = 33;
const uint8_t PIN_TPS_EN = 2, PIN_SDA = 21, PIN_SCL = 22;
const uint8_t LMP_EN[8] = {27, 13, 14, 4, 5, 18, 19, 23};

// ------------------------ LMP91000 REGISTERS ---------------------------
const uint8_t LMP_ADDR   = 0x48;
const uint8_t REG_STATUS = 0x00;
const uint8_t REG_MODECN = 0x12;
const uint8_t MODE_TEMP_TIA_OFF = 0x06;   // 110 = temperature measurement (TIA OFF)

const uint8_t ADC_SAMPLES = 64;

// Datasheet Table 1 (temperature sensor transfer), every 10 degC + 85 degC
const int   N_PTS = 14;
const float T_C[N_PTS]  = {-40, -30, -20, -10,   0,  10,  20,  30,  40,  50,  60,  70,  80,  85};
const float V_MV[N_PTS] = {1875, 1797, 1719, 1640, 1560, 1480, 1399, 1318, 1236, 1154, 1071,  987,  903,  861};

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

// Put the chip in temperature mode. Returns MODECN read-back (0xFF = I2C failure)
uint8_t startTempMode(uint8_t sensorNum) {
  enableOnly(sensorNum - 1);

  uint8_t st = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    if (lmpRead(REG_STATUS, st) && (st & 1)) break;
    delay(10);
  }
  if (!(st & 1)) { enableOnly(-1); return 0xFF; }          // no ACK / not ready

  lmpWrite(REG_MODECN, MODE_TEMP_TIA_OFF);
  uint8_t m = 0xFF;
  lmpRead(REG_MODECN, m);

  enableOnly(-1);                                          // chip keeps the mode
  return m;
}

// ---------------------------- MEASUREMENT ------------------------------
float readVoutMilliVolts() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    sum += analogReadMilliVolts(PIN_MUX_OUT);
    delayMicroseconds(200);
  }
  return (float)sum / ADC_SAMPLES;
}

float vToTempC(float v) {                       // linear interpolation on Table 1
  if (v >= V_MV[0]) return T_C[0];
  if (v <= V_MV[N_PTS - 1]) return T_C[N_PTS - 1];
  for (int i = 0; i < N_PTS - 1; i++) {
    if (v <= V_MV[i] && v >= V_MV[i + 1]) {
      return T_C[i] + (V_MV[i] - v) * (T_C[i + 1] - T_C[i]) / (V_MV[i] - V_MV[i + 1]);
    }
  }
  return NAN;
}

// ------------------------------ WIFI -----------------------------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
}

bool upload(float vout, float tempC, uint8_t modecn) {
  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) return false;

  String url = "http://api.thingspeak.com/update?api_key=";
  url += TS_WRITE_API_KEY;
  url += "&field1=" + String(vout, 1);
  url += "&field2=" + String(tempC, 1);
  url += "&field3=" + String(modecn);
  url += "&field4=" + String(SENSOR_NUM);

  HTTPClient http;
  http.begin(url);
  int code = http.GET();
  String body = http.getString();
  http.end();
  return (code == 200 && body != "0");
}

// ------------------------------ SETUP ----------------------------------
uint8_t modecnReadback = 0xFF;

void setup() {
  Serial.begin(115200);                          // optional

  for (int i = 0; i < 8; i++) { pinMode(LMP_EN[i], OUTPUT); digitalWrite(LMP_EN[i], HIGH); }
  pinMode(PIN_MUX_A, OUTPUT); pinMode(PIN_MUX_B, OUTPUT); pinMode(PIN_MUX_C, OUTPUT);

  pinMode(PIN_TPS_EN, OUTPUT);
  digitalWrite(PIN_TPS_EN, HIGH);                // power the +3V3_AFE rail
  delay(200);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MUX_OUT, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL, 100000);

  modecnReadback = startTempMode(SENSOR_NUM);
  Serial.printf("MODECN read-back = 0x%02X (expect 0x06)\n", modecnReadback);

  muxSelect(SENSOR_NUM - 1);
  delay(500);
  connectWiFi();
}

// ------------------------------- LOOP ----------------------------------
void loop() {
  float vout  = readVoutMilliVolts();            // read before WiFi bursts
  float tempC = vToTempC(vout);

  bool ok = upload(vout, tempC, modecnReadback);
  Serial.printf("VOUT=%.1f mV  Temp=%.1f C  MODECN=0x%02X  upload=%s\n",
                vout, tempC, modecnReadback, ok ? "OK" : "FAIL");

  delay(UPLOAD_MS);
}
