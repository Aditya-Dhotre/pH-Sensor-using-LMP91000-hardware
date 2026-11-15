/*
 * OCP ESTIMATE with LMP91000 (zero-current bias search)  +  ThingSpeak
 *
 * The LMP91000 cannot read an open-circuit voltage directly. This sketch finds the
 * WE-vs-RE bias at which the cell current is zero. That bias equals the open-circuit
 * potential (OCP) of the working electrode vs the reference electrode.
 *
 * NOTE: this is an estimate, not a true open-circuit measurement. The cell is biased
 * during the search and sits at 0 mV bias between searches.
 *
 * Method:
 *   - Bias steps available on the chip: 0, 1, 2, 4, 6, ... 24 % of VDD (about 33 mV to 792 mV), +/-.
 *   - Walk outward from 0 mV until the current changes sign, then interpolate the zero crossing.
 *   - Current is measured at TWO TIA gains (35k and 120k). The difference cancels the
 *     unknown zero-level (Vzero) and ADC offset:  I = (V_B - V_A) / (R_B - R_A)   [mV / kOhm = uA]
 *
 * ThingSpeak fields:
 *   Field 1 = OCP estimate, WE vs RE (mV)
 *   Field 2 = pH (only once CAL_PH4_MV is filled in)
 *   Field 3 = status: 0 OK, 1 no zero crossing / no response, 2 chip not responding, 3 I2C write failed
 *   Field 4 = sensor number
 *
 * Pins (ESP32 DevKit V1, from your PCB):
 *   MUX_A=25  MUX_B=26  MUX_C=32  MUX_OUT=33 (ADC1)
 *   MENB (LMP1..8) = 27,13,14,4,5,18,19,23   SDA=21  SCL=22   TPS_En=2
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>

// ============================ USER SETTINGS ===============================
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* TS_WRITE_API_KEY = "YOUR_THINGSPEAK_WRITE_API_KEY";

const uint8_t  SENSOR_NUM        = 1;        // 1..8 (1 -> mux X0)
const uint32_t INITIAL_SETTLE_MS = 30000;    // after power-up, at 0 mV bias
const uint32_t STEP_SETTLE_MS    = 3000;     // wait after each bias step (raise if current keeps decaying)
const uint32_t GAIN_SETTLE_MS    = 400;      // wait after each TIA gain change
const uint32_t CYCLE_MS          = 120000;   // time between OCP measurements
const float    MIN_RESPONSE_UA   = 0.05f;    // smaller current change than this = "no response"

// ---- pH conversion from OCP (fill in after you read Field 1 in buffers) ----
// Two-point: set both. One-point: set only CAL_PH4_MV (uses the Nernst slope below).
float CAL_PH4_MV = NAN;                      // OCP (mV) measured in pH 4 buffer
float CAL_PH7_MV = NAN;                      // OCP (mV) measured in pH 7 buffer
const float NERNST_SLOPE_MV_PER_PH = -59.16f; // typical Nernstian slope; use +59.16 if your electrode rises with pH
// ===========================================================================

// ---------------------------- PINS ------------------------------------
const uint8_t PIN_MUX_A = 25, PIN_MUX_B = 26, PIN_MUX_C = 32, PIN_MUX_OUT = 33;
const uint8_t PIN_TPS_EN = 2, PIN_SDA = 21, PIN_SCL = 22;
const uint8_t LMP_EN[8] = {27, 13, 14, 4, 5, 18, 19, 23};

// ------------------------ LMP91000 REGISTERS ---------------------------
const uint8_t LMP_ADDR = 0x48;
const uint8_t REG_STATUS = 0x00, REG_LOCK = 0x01, REG_TIACN = 0x10,
              REG_REFCN = 0x11, REG_MODECN = 0x12;

// TIA_GAIN codes: 1=2.75k 2=3.5k 3=7k 4=14k 5=35k 6=120k 7=350k
const float   TIA_KOHM[8] = {0, 2.75f, 3.5f, 7.0f, 14.0f, 35.0f, 120.0f, 350.0f};
const uint8_t GAIN_A = 5;                    // 35k
const uint8_t GAIN_B = 6;                    // 120k
const uint8_t RLOAD_CODE = 3;                // 100 ohm

const uint8_t REF_SOURCE = 0;                // internal (VDD)
const uint8_t INT_Z_CODE = 1;                // 50 %
const uint8_t MODE_3LEAD = 0x03;
const float   VDD_AFE_V  = 3.3f;

// BIAS code k (0..13) = these percentages of VDD
const float   BIAS_PCT[14] = {0, 1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24};
const int     MAX_IDX = 13;                  // signed bias index range -13..+13

const uint8_t ADC_SAMPLES = 128;

// ------------------------------ STATE ----------------------------------
bool  chipOK    = false;
bool  i2cFault  = false;
float Vz_mV     = 0.5f * VDD_AFE_V * 1000.0f;   // running estimate of the TIA zero level

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

// ----------------------- BIAS / GAIN CONTROL ---------------------------
// idx -13..+13. Applied bias (V_WE - V_RE) = sign(idx) * BIAS_PCT[|idx|] % of VDD
float biasMV(int idx) {
  float mv = BIAS_PCT[abs(idx)] / 100.0f * VDD_AFE_V * 1000.0f;
  return idx < 0 ? -mv : mv;
}

bool setBias(int idx) {
  uint8_t code = abs(idx);
  uint8_t sign = (idx >= 0) ? 1 : 0;            // 1 = positive (V_WE - V_RE > 0)
  uint8_t reg  = (REF_SOURCE << 7) | (INT_Z_CODE << 5) | (sign << 4) | code;
  enableOnly(SENSOR_NUM - 1);
  bool ok = lmpWrite(REG_REFCN, reg);
  enableOnly(-1);
  if (!ok) i2cFault = true;
  return ok;
}

bool setGain(uint8_t code) {
  uint8_t val = (code << 2) | RLOAD_CODE;
  uint8_t rb = 0;
  enableOnly(SENSOR_NUM - 1);
  bool ok = lmpWrite(REG_TIACN, val) && lmpRead(REG_TIACN, rb) && (rb == val);
  enableOnly(-1);
  if (!ok) i2cFault = true;
  return ok;
}

bool lmpBegin() {
  enableOnly(SENSOR_NUM - 1);
  uint8_t st = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    if (lmpRead(REG_STATUS, st) && (st & 1)) break;
    delay(10);
  }
  if (!(st & 1)) { enableOnly(-1); return false; }

  bool ok = lmpWrite(REG_LOCK, 0x00);            // stay UNLOCKED so bias and gain can change
  ok = ok && lmpWrite(REG_TIACN, (GAIN_A << 2) | RLOAD_CODE);
  ok = ok && lmpWrite(REG_REFCN, (REF_SOURCE << 7) | (INT_Z_CODE << 5) | (1 << 4) | 0);  // 0 mV
  ok = ok && lmpWrite(REG_MODECN, MODE_3LEAD);
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

// Cell current in uA using two TIA gains (cancels Vzero and ADC offset)
float measureCurrent_uA() {
  setGain(GAIN_A);  delay(GAIN_SETTLE_MS);
  float vA = readVoutMilliVolts();
  setGain(GAIN_B);  delay(GAIN_SETTLE_MS);
  float vB = readVoutMilliVolts();
  setGain(GAIN_A);                                // back to the lower gain

  bool okA = (vA > 200 && vA < 3000);
  bool okB = (vB > 200 && vB < 3000);
  if (okA && okB) {
    float i = (vB - vA) / (TIA_KOHM[GAIN_B] - TIA_KOHM[GAIN_A]);   // mV / kOhm = uA
    Vz_mV = vA - i * TIA_KOHM[GAIN_A];            // refresh the zero-level estimate
    return i;
  }
  // High gain saturated: fall back to the single-gain estimate
  return (vA - Vz_mV) / TIA_KOHM[GAIN_A];
}

float measureAt(int idx) {
  setBias(idx);
  delay(STEP_SETTLE_MS);
  float i = measureCurrent_uA();
  Serial.printf("bias %+7.1f mV -> I = %+.3f uA\n", biasMV(idx), i);
  return i;
}

float zeroCross(float Ea, float Ia, float Eb, float Ib) {
  float d = Ib - Ia;
  if (fabsf(d) < 1e-9f) return Ea;
  return Ea + (0.0f - Ia) * (Eb - Ea) / d;
}

// Returns 0 = OK (ocp set), 1 = no zero crossing / no response
int findOCP(float &ocp) {
  float I0 = measureAt(0);
  float I1 = measureAt(1);
  float E1 = biasMV(1);

  if (fabsf(I1 - I0) < MIN_RESPONSE_UA) { setBias(0); return 1; }   // cell shows no current response

  float k = (I1 - I0) / E1;                       // uA per mV (sign tells which way is "up")
  int dir = (I0 * k > 0) ? -1 : +1;               // direction toward the zero crossing

  int   prevIdx = 0;
  float prevI   = I0;
  if (dir == +1) {
    if (I0 * I1 <= 0) { ocp = zeroCross(0, I0, E1, I1); setBias(0); return 0; }
    prevIdx = 1; prevI = I1;
  }

  for (int idx = prevIdx + dir; abs(idx) <= MAX_IDX; idx += dir) {
    float Ii = measureAt(idx);
    if (prevI * Ii <= 0) {
      ocp = zeroCross(biasMV(prevIdx), prevI, biasMV(idx), Ii);
      setBias(0);
      return 0;
    }
    prevIdx = idx; prevI = Ii;
  }
  setBias(0);
  return 1;                                       // OCP outside +/-792 mV window
}

bool phFromOCP(float e_mV, float &pH) {
  if (isnan(CAL_PH4_MV)) return false;
  float slope = isnan(CAL_PH7_MV) ? NERNST_SLOPE_MV_PER_PH
                                  : (CAL_PH7_MV - CAL_PH4_MV) / 3.0f;
  if (fabsf(slope) < 1e-3f) return false;
  pH = 4.0f + (e_mV - CAL_PH4_MV) / slope;
  return true;
}

// ------------------------------ WIFI -----------------------------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
}

bool upload(int status, float ocp, bool hasPH, float pH) {
  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) return false;

  String url = "http://api.thingspeak.com/update?api_key=";
  url += TS_WRITE_API_KEY;
  if (status == 0) {
    url += "&field1=" + String(ocp, 1);
    if (hasPH) url += "&field2=" + String(pH, 2);
  }
  url += "&field3=" + String(status);
  url += "&field4=" + String(SENSOR_NUM);

  HTTPClient http;
  http.begin(url);
  int code = http.GET();
  String body = http.getString();
  http.end();
  return (code == 200 && body != "0");
}

// ------------------------------ SETUP ----------------------------------
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 8; i++) { pinMode(LMP_EN[i], OUTPUT); digitalWrite(LMP_EN[i], HIGH); }
  pinMode(PIN_MUX_A, OUTPUT); pinMode(PIN_MUX_B, OUTPUT); pinMode(PIN_MUX_C, OUTPUT);

  pinMode(PIN_TPS_EN, OUTPUT);
  digitalWrite(PIN_TPS_EN, HIGH);                // power the +3V3_AFE rail
  delay(200);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MUX_OUT, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL, 100000);

  chipOK = lmpBegin();
  Serial.println(chipOK ? "LMP91000 configured" : "LMP91000 config FAILED");
  muxSelect(SENSOR_NUM - 1);

  delay(INITIAL_SETTLE_MS);                      // let the cell settle at 0 mV bias
}

// ------------------------------- LOOP ----------------------------------
void loop() {
  WiFi.mode(WIFI_OFF);                           // keep the radio quiet while measuring
  delay(200);

  i2cFault = false;
  float ocp = NAN;
  int status;

  if (!chipOK) chipOK = lmpBegin();
  if (!chipOK) {
    status = 2;
  } else {
    status = findOCP(ocp);
    if (status == 0 && i2cFault) status = 3;
  }

  float pH = NAN;
  bool hasPH = (status == 0) && phFromOCP(ocp, pH);

  bool sent = upload(status, ocp, hasPH, pH);
  Serial.printf("status=%d OCP=%.1f mV pH=%.2f upload=%s\n",
                status, ocp, hasPH ? pH : NAN, sent ? "OK" : "FAIL");

  delay(CYCLE_MS);
}
