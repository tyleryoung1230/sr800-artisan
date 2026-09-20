/*
 * stage4_full — Independent OT1 (heat) + OT2 (fan) phase-angle
 * See repo README for wiring, Artisan, and safety.
 */

#include <SPI.h>
#include <Adafruit_MAX31855.h>
#include <esp_system.h>
#include <math.h>
#include "config.h"

// Software SPI (SCK, CS, DO)
Adafruit_MAX31855 thermocouple(PIN_MAX_SCK, PIN_MAX_CS, PIN_MAX_MISO);

static const uint8_t FAN_MIN_PCT = 5;  // was 22 — SR800 may cog/pulse below ~20; try and see
static const uint32_t ZCD_MIN_GAP_US = 4000;      // fan-only debounce
static const uint32_t ZCD_MIN_GAP_HEAT_US = 7200; // under heat: ~1 edge per half-cycle max
static const uint32_t ZCD_PERIOD_LO_US = 5500;
static const uint32_t ZCD_PERIOD_HI_US = 11000;
static const uint32_t ZCD_CHECK_MS = 250;
static const uint32_t ZCD_HEAT_FAIL_MS = 1500;
static const uint32_t ZCD_FAN_FAIL_MS = 4000;
static const uint32_t FLYWHEEL_MAX_US = 20000;
static const uint32_t ALIVE_MS = 5000;
static const uint32_t CMD_IDLE_MS = 40;
static const uint32_t OVERTEMP_CHECK_MS = 500;
// Set 1 to print # OT1= / # alive (breaks some Artisan setups — keep 0 for roasting)
#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL 0
#endif

uint8_t levelOT1 = 0;
uint8_t levelOT2 = 0;
bool overtempLatch = false;
float lastGoodBtC = NAN;  // hold across MAX31855 faults so Artisan doesn't see 0 spikes
uint8_t tcFaultCount = 0;

bool heatPulseActive = false;
bool fanPulseActive = false;
uint32_t heatPulseEndUs = 0;
uint32_t fanPulseEndUs = 0;
uint32_t heatFireAtUs = 0;
uint32_t fanFireAtUs = 0;
bool heatFireSched = false;
bool fanFireSched = false;

uint32_t halfPeriodUs = HALF_PERIOD_US;
uint32_t lastEdgeUs = 0;
uint32_t lastGoodStampUs = 0;
uint32_t nextExpectUs = 0;
uint32_t zcdEdges = 0;
uint32_t rejectedEdges = 0;
bool zcdEverOk = false;
bool lastZcdLevel = true;

enum TempUnit : uint8_t { UNIT_C = 0, UNIT_F = 1 };
TempUnit tempUnit = DEFAULT_UNIT_F ? UNIT_F : UNIT_C;
uint8_t chanMap[4] = {1, 2, 0, 0};
String cmdBuf;

static void tripOvertemp(float btF);  // fwd — used from respondRead

static void otWrite(int pin, bool on) {
#if OT_ACTIVE_HIGH
  digitalWrite(pin, on ? HIGH : LOW);
#else
  digitalWrite(pin, on ? LOW : HIGH);
#endif
}

static void heatAllOff() {
  heatFireSched = false;
  heatPulseActive = false;
  otWrite(PIN_OT1_HEATER, false);
}

static void fanAllOff() {
  fanFireSched = false;
  fanPulseActive = false;
  otWrite(PIN_OT2_FAN, false);
}

static void outputsAllOff() {
  heatAllOff();
  fanAllOff();
}

static uint8_t effectiveHeat(uint8_t heatCmd, uint8_t fanCmd) {
  if (overtempLatch) return 0;
  if (fanCmd < HTR_CUTOFF_FAN_VAL) return 0;
  return heatCmd;
}

static uint8_t clampFan(uint8_t pct) {
  if (pct > 0 && pct < FAN_MIN_PCT) return 0;
  return pct;
}

static uint32_t delayForLevel(uint8_t lvl) {
  // Always use fixed mains half-period. EMI used to shrink halfPeriodUs
  // → same OT2% fired earlier → fan raced up with heat.
  if (lvl >= 100) return 0;
  if (lvl == 0) return UINT32_MAX;
  // Prefer (100-lvl)*HALF/100 over HALF/100*… (less truncation)
  return (uint32_t)((uint32_t)(100 - lvl) * HALF_PERIOD_US) / 100;
}

static bool gapLooksLikeMains(uint32_t gap, uint8_t* multOut) {
  // Only accept ~1× or 2× of the nominal 8333 µs half-cycle
  for (uint8_t n = 1; n <= 2; n++) {
    uint32_t expect = HALF_PERIOD_US * n;
    uint32_t lo = (expect * 70) / 100;
    uint32_t hi = (expect * 130) / 100;
    if (gap >= lo && gap <= hi) {
      if (multOut) *multOut = n;
      return true;
    }
  }
  return false;
}

static void scheduleFromStamp(uint32_t stamp) {
  uint8_t fan = levelOT2;
  uint8_t heat = effectiveHeat(levelOT1, levelOT2);

  uint32_t fanDelay = delayForLevel(fan);
  if (fanDelay == UINT32_MAX) {
    fanFireSched = false;
    fanPulseActive = false;
    otWrite(PIN_OT2_FAN, false);
  } else {
    fanFireAtUs = stamp + fanDelay;
    fanFireSched = true;
  }

  uint32_t heatDelay = delayForLevel(heat);
  if (heatDelay == UINT32_MAX) {
    heatFireSched = false;
    heatPulseActive = false;
    otWrite(PIN_OT1_HEATER, false);
  } else {
    heatFireAtUs = stamp + heatDelay;
    heatFireSched = true;
  }

  lastGoodStampUs = stamp;
  nextExpectUs = stamp + HALF_PERIOD_US;
  zcdEverOk = true;
}

// Poll ZCD pin — no interrupts (EMI-safe).
static void pollZcd() {
  bool level = digitalRead(PIN_ZCD);
  if (level == lastZcdLevel) return;
  lastZcdLevel = level;

  uint32_t now = micros();
  uint32_t gap = (lastEdgeUs == 0) ? HALF_PERIOD_US : (now - lastEdgeUs);

  // Under heat, demand near-full half-cycle spacing so EMI can't
  // inject extra PAC schedules (that made the fan surge with OT1).
  uint32_t minGap = (levelOT1 > 0) ? ZCD_MIN_GAP_HEAT_US : ZCD_MIN_GAP_US;
  if (lastEdgeUs != 0 && gap < minGap) {
    rejectedEdges++;
    return;
  }

  uint8_t mult = 1;
  if (lastEdgeUs != 0 && !gapLooksLikeMains(gap, &mult)) {
    rejectedEdges++;
    // Do NOT move lastEdgeUs — keep phase anchored to last good edge
    return;
  }

  lastEdgeUs = now;
  zcdEdges++;

  // Learn period only with heat OFF (clean ZCD) — STAT only; PAC uses fixed HALF
  if (levelOT1 == 0 && mult == 1 &&
      gap >= ZCD_PERIOD_LO_US && gap <= ZCD_PERIOD_HI_US) {
    halfPeriodUs = (halfPeriodUs * 7 + gap) / 8;
    if (halfPeriodUs < 6000 || halfPeriodUs > 10000) halfPeriodUs = HALF_PERIOD_US;
  }

  scheduleFromStamp(now);
}

static void servicePhase() {
  pollZcd();

  uint32_t now = micros();

  // Flywheel only when heat is OFF — under heat, extra predicted
  // edges also surge the fan. Real ZCD (filtered) drives PAC instead.
  if (levelOT1 == 0 && !heatFireSched && !fanFireSched && zcdEverOk &&
      levelOT2 > 0) {
    if ((int32_t)(now - nextExpectUs) >= 0 &&
        (now - lastGoodStampUs) < FLYWHEEL_MAX_US) {
      scheduleFromStamp(nextExpectUs);
    }
  }

  // --- Fan gate: ONLY high during a timed pulse. Never solid-hold. ---
  if (levelOT2 == 0) {
    fanFireSched = false;
    fanPulseActive = false;
    otWrite(PIN_OT2_FAN, false);
  } else {
    // Skip a fire if the loop was blocked so long we'd be into the next half-cycle
  if (fanFireSched && !fanPulseActive && (int32_t)(now - fanFireAtUs) >= 0) {
    if ((int32_t)(now - fanFireAtUs) < (int32_t)(HALF_PERIOD_US / 2)) {
      otWrite(PIN_OT2_FAN, true);
      fanPulseActive = true;
      fanPulseEndUs = now + TRIAC_PULSE_US;
    }
    fanFireSched = false;
  }
    if (fanPulseActive && (int32_t)(now - fanPulseEndUs) >= 0) {
      otWrite(PIN_OT2_FAN, false);
      fanPulseActive = false;
    }
    // Stuck-high guard: if nothing scheduled/active, force LOW
    if (!fanFireSched && !fanPulseActive) {
      otWrite(PIN_OT2_FAN, false);
    }
  }

  // --- Heat gate: same rules ---
  if (effectiveHeat(levelOT1, levelOT2) == 0) {
    heatFireSched = false;
    heatPulseActive = false;
    otWrite(PIN_OT1_HEATER, false);
  } else {
    if (heatFireSched && !heatPulseActive && (int32_t)(now - heatFireAtUs) >= 0) {
      if ((int32_t)(now - heatFireAtUs) < (int32_t)(HALF_PERIOD_US / 2)) {
        otWrite(PIN_OT1_HEATER, true);
        heatPulseActive = true;
        heatPulseEndUs = now + TRIAC_PULSE_US;
      }
      heatFireSched = false;
    }
    if (heatPulseActive && (int32_t)(now - heatPulseEndUs) >= 0) {
      otWrite(PIN_OT1_HEATER, false);
      heatPulseActive = false;
    }
    if (!heatFireSched && !heatPulseActive) {
      otWrite(PIN_OT1_HEATER, false);
    }
  }
}

static float toUnit(float celsius) {
  if (isnan(celsius)) return celsius;
  return (tempUnit == UNIT_F) ? (celsius * 9.0f / 5.0f + 32.0f) : celsius;
}

// Stable BT in °C: reject faults / EMI spikes, hold last good reading.
static float readBtCelsius() {
  // Median of 3 quick samples — kills single-bit SPI glitches
  float a = thermocouple.readCelsius();
  float b = thermocouple.readCelsius();
  float c = thermocouple.readCelsius();
  uint8_t err = thermocouple.readError();

  // sort a<=b<=c
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  float mid = b;

  if (err != 0 || isnan(mid) || isnan(a) || isnan(c)) {
    tcFaultCount++;
    return lastGoodBtC;
  }

  if (mid < -20.0f || mid > 400.0f) {
    tcFaultCount++;
    return lastGoodBtC;
  }

  // Open/intermittent TC often collapses toward cold-junction (~room temp)
  float cj = thermocouple.readInternal();
  if (!isnan(lastGoodBtC) && !isnan(cj) && lastGoodBtC > 50.0f &&
      fabsf(mid - cj) < 5.0f) {
    tcFaultCount++;
    return lastGoodBtC;
  }

  if (!isnan(lastGoodBtC) && fabsf(mid - lastGoodBtC) > 30.0f) {
    tcFaultCount++;
    return lastGoodBtC;
  }

  lastGoodBtC = mid;
  return mid;
}

static void respondRead() {
  float rawC = readBtCelsius();
  float amb = toUnit(thermocouple.readInternal());
  float bt = toUnit(rawC);

  if (!isnan(rawC)) {
    float btF = rawC * 9.0f / 5.0f + 32.0f;
    if (!overtempLatch && btF >= (float)OVERTEMP_F) tripOvertemp(btF);
    else if (overtempLatch && btF <= (float)OVERTEMP_CLEAR_F) {
      overtempLatch = false;
      Serial.print(F("# overtemp clear BT="));
      Serial.print(btF, 1);
      Serial.println(F("F — OT1 allowed again"));
    }
  }

  float t[4];
  for (int i = 0; i < 4; i++) {
    uint8_t phys = chanMap[i];
    if (phys == 1 || phys == 2) t[i] = bt;
    else t[i] = NAN;
  }
  auto p = [](float v) {
    if (isnan(v)) Serial.print("0.00");
    else Serial.print(v, 2);
  };
  uint8_t heatOut = effectiveHeat(levelOT1, levelOT2);
  p(amb);
  for (int i = 0; i < 4; i++) {
    Serial.print(',');
    p(t[i]);
  }
  Serial.print(',');
  Serial.print(heatOut);
  Serial.print(',');
  Serial.println(levelOT2);
}

static uint8_t parseDuty(const String& s) {
  int v = s.toInt();
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  return (uint8_t)v;
}

static void applyOutputs() {
  if (overtempLatch) levelOT1 = 0;
  if (levelOT2 == 0) fanAllOff();
  if (effectiveHeat(levelOT1, levelOT2) == 0) heatAllOff();
}

static void tripOvertemp(float btF) {
  if (overtempLatch) return;
  overtempLatch = true;
  levelOT1 = 0;
  heatAllOff();
  Serial.print(F("# FAILSAFE: BT "));
  Serial.print(btF, 1);
  Serial.print(F("F >= "));
  Serial.print(OVERTEMP_F);
  Serial.println(F("F — OT1 cut (clears below 480F)"));
}

// BT check in °F regardless of Artisan UNIT setting.
static void serviceOvertemp() {
  static uint32_t lastCheck = 0;
  uint32_t now = millis();
  if (now - lastCheck < OVERTEMP_CHECK_MS) return;
  lastCheck = now;

  // Skip when idle (no heat/fan and not latched)
  if (levelOT1 == 0 && levelOT2 == 0 && !overtempLatch) return;

  float c = readBtCelsius();
  if (isnan(c)) return;  // no valid sample yet
  float btF = c * 9.0f / 5.0f + 32.0f;

  if (!overtempLatch && btF >= (float)OVERTEMP_F) {
    tripOvertemp(btF);
    return;
  }
  if (overtempLatch && btF <= (float)OVERTEMP_CLEAR_F) {
    overtempLatch = false;
    Serial.print(F("# overtemp clear BT="));
    Serial.print(btF, 1);
    Serial.println(F("F — OT1 allowed again"));
  }
}

static void printResetReason() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.print(F("# reset="));
  switch (r) {
    case ESP_RST_POWERON: Serial.println(F("POWERON")); break;
    case ESP_RST_SW: Serial.println(F("SW")); break;
    case ESP_RST_PANIC: Serial.println(F("PANIC")); break;
    case ESP_RST_INT_WDT: Serial.println(F("INT_WDT")); break;
    case ESP_RST_TASK_WDT: Serial.println(F("TASK_WDT")); break;
    case ESP_RST_WDT: Serial.println(F("WDT")); break;
    case ESP_RST_BROWNOUT: Serial.println(F("BROWNOUT")); break;
    case ESP_RST_SDIO: Serial.println(F("SDIO")); break;
    default: Serial.println((int)r); break;
  }
}

static void handleCommand(String line) {
  line.trim();
  if (!line.length()) return;
  line.replace(',', ';');
  line.toUpperCase();

  if (line == "READ") {
    respondRead();
    return;
  }
  if (line == "STAT") {
    Serial.print(F("# zcdOk="));
    Serial.print(zcdEverOk ? 1 : 0);
    Serial.print(F(" edges="));
    Serial.print(zcdEdges);
    Serial.print(F(" rej="));
    Serial.print(rejectedEdges);
    Serial.print(F(" halfUs="));
    Serial.print(halfPeriodUs);
    Serial.print(F(" OT1="));
    Serial.print(levelOT1);
    Serial.print(F(" OT2="));
    Serial.print(levelOT2);
    Serial.print(F(" ot="));
    Serial.print(overtempLatch ? 1 : 0);
    Serial.print(F(" tcFaults="));
    Serial.println(tcFaultCount);
    return;
  }
  if (line == "RESET") {
    levelOT1 = 0;
    levelOT2 = 0;
    overtempLatch = false;
    outputsAllOff();
#if DEBUG_SERIAL
    Serial.println(F("# RESET"));
#endif
    return;
  }
  if (line.startsWith("OT1;")) {
    uint8_t req = parseDuty(line.substring(4));
    if (overtempLatch && req > 0) {
      levelOT1 = 0;
      heatAllOff();
      Serial.println(F("# OT1 blocked — overtemp latch (wait BT<480F)"));
      return;
    }
    levelOT1 = req;
    applyOutputs();
#if DEBUG_SERIAL
    Serial.print(F("# OT1="));
    Serial.print(levelOT1);
    Serial.print(F(" eff="));
    Serial.println(effectiveHeat(levelOT1, levelOT2));
#endif
    return;
  }
  if (line.startsWith("OT2;")) {
    levelOT2 = clampFan(parseDuty(line.substring(4)));
    applyOutputs();
#if DEBUG_SERIAL
    Serial.print(F("# OT2="));
    Serial.println(levelOT2);
#endif
    return;
  }
  if (line.startsWith("UNIT;")) {
    tempUnit = line.substring(5).startsWith("F") ? UNIT_F : UNIT_C;
    return;
  }
  if (line.startsWith("CHAN;")) {
    String body = line.substring(5);
    for (int i = 0; i < 4; i++) {
      if (i < (int)body.length() && isDigit(body[i]))
        chanMap[i] = (uint8_t)(body[i] - '0');
      else
        chanMap[i] = 0;
    }
    return;
  }
  if (line.startsWith("IO3;") || line.startsWith("DCFAN;") ||
      line.startsWith("FILT;") || line.startsWith("PID;")) {
    return;
  }
}

static void serviceWatchdog() {
  static uint32_t lastCheck = 0;
  static uint32_t lastEdgeSnap = 0;
  static uint32_t noEdgeMs = 0;
  static bool heatFailLatched = false;
  static uint32_t lastAlive = 0;

  uint32_t now = millis();

#if DEBUG_SERIAL
  if (now - lastAlive >= ALIVE_MS) {
    lastAlive = now;
    if (levelOT1 > 0 || levelOT2 > 0) {
      Serial.print(F("# alive OT1="));
      Serial.print(levelOT1);
      Serial.print(F(" OT2="));
      Serial.print(levelOT2);
      Serial.print(F(" edges="));
      Serial.println(zcdEdges);
    }
  }
#else
  (void)lastAlive;
#endif

  if (now - lastCheck < ZCD_CHECK_MS) return;
  uint32_t dt = now - lastCheck;
  lastCheck = now;

  if (zcdEdges != lastEdgeSnap) {
    lastEdgeSnap = zcdEdges;
    noEdgeMs = 0;
    heatFailLatched = false;
    return;
  }

  if (levelOT1 == 0 && levelOT2 == 0) {
    noEdgeMs = 0;
    return;
  }

  noEdgeMs += dt;

  if (levelOT1 > 0 && noEdgeMs >= ZCD_HEAT_FAIL_MS && !heatFailLatched) {
    levelOT1 = 0;
    heatAllOff();
    heatFailLatched = true;
    Serial.println(F("# FAILSAFE: no ZCD 1.5s — HEAT off (fan keeps OT2 %)"));
  }

  if (levelOT2 > 0 && noEdgeMs >= ZCD_FAN_FAIL_MS) {
    levelOT2 = 0;
    fanAllOff();
    zcdEverOk = false;
    Serial.println(F("# FAILSAFE: no ZCD 4s — FAN off"));
    noEdgeMs = 0;
  }
}

// Idle-flush only for bare words. OT1/OT2 always need NL/CR so "OT2;10"+"0"
// can't become OT2;10 instead of OT2;100.
static bool cmdLooksComplete(const String& s) {
  return s.equalsIgnoreCase("READ") || s.equalsIgnoreCase("STAT") ||
         s.equalsIgnoreCase("RESET");
}

static void serviceSerial() {
  static uint32_t lastByteMs = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    lastByteMs = millis();
    if (c == '\n' || c == '\r') {
      if (cmdBuf.length()) {
        handleCommand(cmdBuf);
        cmdBuf = "";
      }
    } else if (cmdBuf.length() < 80) {
      cmdBuf += c;
    } else {
      cmdBuf = "";
    }
  }

  // Idle flush only if command looks complete — avoids turning "OT2;" into OT2;0
  if (cmdBuf.length() && (millis() - lastByteMs) >= CMD_IDLE_MS &&
      cmdLooksComplete(cmdBuf)) {
    handleCommand(cmdBuf);
    cmdBuf = "";
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  pinMode(PIN_OT1_HEATER, OUTPUT);
  pinMode(PIN_OT2_FAN, OUTPUT);
  digitalWrite(PIN_OT1_HEATER, LOW);
  digitalWrite(PIN_OT2_FAN, LOW);
  levelOT1 = 0;
  levelOT2 = 0;
  outputsAllOff();

  // Soft-SPI MAX31855 — no SPI.begin needed for the amp
  thermocouple.begin();

  pinMode(PIN_ZCD, INPUT_PULLUP);
  lastZcdLevel = digitalRead(PIN_ZCD);

  printResetReason();
  Serial.println(F("# stage4 READY — OT1 cuts at BT>=500F (clears <480F)"));
  Serial.println(F("# OT1/OT2 need Newline (Artisan OK). FAN_MIN=5 heatCutoff=22"));
}

void loop() {
  serviceSerial();
  servicePhase();
  serviceSerial();
  serviceOvertemp();
  serviceWatchdog();
}
