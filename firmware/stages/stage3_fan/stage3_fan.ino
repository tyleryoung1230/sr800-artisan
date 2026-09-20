/*
 * stage3_fan — Phase-angle fan control (heater hard-off)
 *
 * After ZCD looks healthy (~100-140 Hz), auto-ramps fan:
 *   0% → 50% → 0% over 2 seconds (triangle), repeating
 *
 * Serial:
 *   AUTO;0     stop auto (fan → 0)
 *   AUTO;1     resume auto ramp (default on)
 *   STEADY;1   solid GPIO on/off bench (no ZCD; NPN/opto meter)
 *   STEADY;0   back to ZCD phase-angle
 *   OT2;n      manual fan; disables auto until AUTO;1
 *   READ       temps + current OT2
 */

#include <SPI.h>
#include <Adafruit_MAX31855.h>
#include "config.h"

Adafruit_MAX31855 thermocouple(PIN_MAX_CS);

portMUX_TYPE zcdMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t zcdStampUs = 0;
volatile bool zcdPending = false;
volatile uint8_t fanLevel = 0;
volatile uint32_t zcdEdges = 0;
volatile uint32_t lastZcdEdgeUs = 0;

bool pulseActive = false;
uint32_t pulseEndUs = 0;
uint32_t fireAtUs = 0;
bool fireScheduled = false;

bool autoCycle = true;
bool zcdLocked = false;      // true after we see healthy ZCD
bool fanHigh = false;        // steady-bench on/off state
bool steadyBench = false;    // true = hold GPIO solid on/off (opto meter test, no PAC)
uint32_t lastZcdCheckMs = 0;
uint32_t lastToggleMs = 0;
uint32_t lastRampMs = 0;
uint32_t rampStartMs = 0;
uint8_t healthySeconds = 0;  // need a couple good seconds before lock
uint8_t zcdBadSeconds = 0;   // debounce dropouts before killing fan

enum TempUnit : uint8_t { UNIT_C = 0, UNIT_F = 1 };
TempUnit tempUnit = UNIT_F;
uint8_t levelOT2 = 0;
String cmdBuf;

static const uint8_t RAMP_MAX_PCT = 50;
static const uint8_t RAMP_MIN_PCT = 22;  // below this SR800 fan tends to pulse/cog
static const uint32_t RAMP_PERIOD_MS = 10000;  // full cycle (5s up, 5s down)
static const uint32_t RAMP_UPDATE_MS = 50;
static const uint32_t STEADY_INTERVAL_MS = 5000;

// Fan EMI: ignore high edge counts; only treat *missing* ZCD as fatal.
// ISR debounce kills double-fires from noise.
static const uint32_t ZCD_HZ_MIN = 70;          // below this = weak/missing
static const uint32_t ZCD_HZ_MAX_WARN = 200;    // above = noisy, but keep fan
static const uint8_t ZCD_BAD_BEFORE_KILL = 3;
static const uint32_t ZCD_DEBOUNCE_US = 1500;   // min spacing between edges (~8ms real)

static void otWrite(int pin, bool on) {
#if OT_ACTIVE_HIGH
  digitalWrite(pin, on ? HIGH : LOW);
#else
  digitalWrite(pin, on ? LOW : HIGH);
#endif
}

static void setFan(uint8_t pct) {
  // In PAC mode, snap sticky low duties to 0 (except steady bench on/off)
  if (!steadyBench && pct > 0 && pct < RAMP_MIN_PCT) {
    pct = 0;
  }
  levelOT2 = pct;
  portENTER_CRITICAL(&zcdMux);
  fanLevel = pct;
  portEXIT_CRITICAL(&zcdMux);
}

void IRAM_ATTR onZcd() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&zcdMux);
  if ((now - lastZcdEdgeUs) < ZCD_DEBOUNCE_US) {
    portEXIT_CRITICAL_ISR(&zcdMux);
    return;
  }
  lastZcdEdgeUs = now;
  zcdStampUs = now;
  zcdPending = true;
  zcdEdges++;
  portEXIT_CRITICAL_ISR(&zcdMux);
}

static void scheduleFromZcd(uint32_t stamp, uint8_t lvl) {
  fireScheduled = false;
  if (lvl == 0) {
    otWrite(PIN_OT2_FAN, false);
    pulseActive = false;
    return;
  }
  uint32_t delayUs = 0;
  if (lvl < 100) {
    delayUs = (uint32_t)((100 - lvl) * (HALF_PERIOD_US / 100));
  }
  fireAtUs = stamp + delayUs;
  fireScheduled = true;
}

static void servicePhase() {
  // Steady bench mode: GPIO follows OT2 as solid level (for opto meter test)
  if (steadyBench) {
    fireScheduled = false;
    pulseActive = false;
    otWrite(PIN_OT2_FAN, levelOT2 > 0);
    otWrite(PIN_OT1_HEATER, false);
    return;
  }

  uint32_t now = micros();

  bool pending;
  uint32_t stamp;
  uint8_t lvl;
  portENTER_CRITICAL(&zcdMux);
  pending = zcdPending;
  stamp = zcdStampUs;
  lvl = fanLevel;
  if (pending) zcdPending = false;
  portEXIT_CRITICAL(&zcdMux);

  if (pending) {
    scheduleFromZcd(stamp, lvl);
  }

  if (fireScheduled && (int32_t)(now - fireAtUs) >= 0) {
    fireScheduled = false;
    otWrite(PIN_OT2_FAN, true);
    pulseActive = true;
    pulseEndUs = now + TRIAC_PULSE_US;
  }

  if (pulseActive && (int32_t)(now - pulseEndUs) >= 0) {
    otWrite(PIN_OT2_FAN, false);
    pulseActive = false;
  }
}

static uint8_t triangleDuty(uint32_t elapsedMs) {
  // Triangle between RAMP_MIN and RAMP_MAX, with a short 0% dwell at the bottom
  // so we don't crawl through the sticky low band.
  uint32_t t = elapsedMs % RAMP_PERIOD_MS;
  uint32_t quarter = RAMP_PERIOD_MS / 4;
  // 0..1/4: hold 0 (off)
  if (t < quarter) return 0;
  // 1/4..2/4: MIN → MAX
  if (t < quarter * 2) {
    uint32_t u = t - quarter;
    return (uint8_t)(RAMP_MIN_PCT + (u * (RAMP_MAX_PCT - RAMP_MIN_PCT)) / quarter);
  }
  // 2/4..3/4: MAX → MIN
  if (t < quarter * 3) {
    uint32_t u = t - quarter * 2;
    return (uint8_t)(RAMP_MAX_PCT - (u * (RAMP_MAX_PCT - RAMP_MIN_PCT)) / quarter);
  }
  // 3/4..4/4: hold 0
  return 0;
}

static void serviceZcdLockAndAuto() {
  uint32_t now = millis();

  // Steady bench: ignore ZCD; solid on/off every 5s
  if (steadyBench) {
    if (now - lastZcdCheckMs >= 1000) {
      lastZcdCheckMs = now;  // keep timing cadence quiet
    }
    if (!autoCycle) return;
    if (now - lastToggleMs >= STEADY_INTERVAL_MS) {
      lastToggleMs = now;
      fanHigh = !fanHigh;
      uint8_t pct = fanHigh ? 100 : 0;
      setFan(pct);
      Serial.print(F("# bench GPIO27="));
      Serial.println(pct);
    }
    return;
  }

  // ZCD health once per second
  if (now - lastZcdCheckMs >= 1000) {
    lastZcdCheckMs = now;

    noInterrupts();
    uint32_t edges = zcdEdges;
    zcdEdges = 0;
    interrupts();

    bool tooLow = (edges < ZCD_HZ_MIN);
    bool noisy = (edges > ZCD_HZ_MAX_WARN);
    bool healthy = !tooLow;  // high count = noise, still OK for keeping fan

    Serial.print(F("# zcd_hz="));
    Serial.print(edges);
    if (noisy) Serial.print(F(" (noisy)"));
    Serial.println();

    if (!zcdLocked) {
      // For initial lock, prefer a sane band (debounce should keep ~120)
      bool lockOk = (edges >= ZCD_HZ_MIN && edges <= ZCD_HZ_MAX_WARN);
      if (lockOk) {
        healthySeconds++;
        zcdBadSeconds = 0;
        Serial.print(F("# ZCD healthy "));
        Serial.print(healthySeconds);
        Serial.println(F("/2"));
        if (healthySeconds >= 2) {
          zcdLocked = true;
          zcdBadSeconds = 0;
          rampStartMs = now;
          lastRampMs = 0;
          setFan(0);
          Serial.println(F("# ZCD LOCKED — ramp OT2 0 / 22..50% over 10s (skips sticky low)"));
          Serial.println(F("# AUTO;0 to stop, OT2;n for manual"));
        }
      } else {
        healthySeconds = 0;
        setFan(0);
        Serial.println(F("# waiting for ZCD ~120Hz..."));
      }
      return;
    }

    // Locked: only missing ZCD (too low) can kill fan after debounce
    if (tooLow) {
      zcdBadSeconds++;
      Serial.print(F("# ZCD low "));
      Serial.print(zcdBadSeconds);
      Serial.print(F("/"));
      Serial.print(ZCD_BAD_BEFORE_KILL);
      Serial.print(F(" (hz="));
      Serial.print(edges);
      Serial.println(F(") — keeping fan"));
      if (zcdBadSeconds >= ZCD_BAD_BEFORE_KILL) {
        Serial.println(F("# ZCD lost — fan forced 0"));
        setFan(0);
        zcdLocked = false;
        healthySeconds = 0;
        zcdBadSeconds = 0;
      }
    } else {
      zcdBadSeconds = 0;
    }
  }

  if (!zcdLocked || !autoCycle) return;

  // Smooth triangle ramp every 50ms
  if (now - lastRampMs < RAMP_UPDATE_MS) return;
  lastRampMs = now;

  uint8_t pct = triangleDuty(now - rampStartMs);
  if (pct != levelOT2) {
    setFan(pct);
    Serial.print(F("# ramp OT2="));
    Serial.println(pct);
  }
}

static float toUnit(float celsius) {
  if (isnan(celsius)) return celsius;
  return (tempUnit == UNIT_F) ? (celsius * 9.0f / 5.0f + 32.0f) : celsius;
}

static void respondRead() {
  float amb = toUnit(thermocouple.readInternal());
  float bt = toUnit(thermocouple.readCelsius());
  auto p = [](float v) {
    if (isnan(v)) Serial.print("0.00");
    else Serial.print(v, 2);
  };
  p(amb);
  Serial.print(',');
  p(bt);
  Serial.print(',');
  p(bt);
  Serial.print(F(",0.00,0.00,0,"));
  Serial.println(levelOT2);
}

static uint8_t parseDuty(const String& s) {
  int v = s.toInt();
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  return (uint8_t)v;
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
  if (line.startsWith("STEADY;") || line.startsWith("BENCH;")) {
    // STEADY;1 = solid GPIO on/off for opto meter test (no SR800 needed)
    // STEADY;0 = normal phase-angle mode
    steadyBench = parseDuty(line.substring(line.indexOf(';') + 1)) != 0;
    autoCycle = steadyBench;  // reuse 5s 0<->20 auto while benching
    if (steadyBench) {
      zcdLocked = true;  // don't wait on ZCD for bench
      lastToggleMs = millis();
      fanHigh = false;
      setFan(0);
      Serial.println(F("# STEADY bench ON — GPIO27 solid 0<->20% every 5s (20 means ON)"));
      Serial.println(F("# No SR800 needed. Measure opto OUT resistance/voltage."));
    } else {
      setFan(0);
      otWrite(PIN_OT2_FAN, false);
      zcdLocked = false;
      healthySeconds = 0;
      Serial.println(F("# STEADY bench OFF — back to ZCD phase-angle"));
    }
    return;
  }
  if (line.startsWith("AUTO;")) {
    autoCycle = parseDuty(line.substring(5)) != 0;
    if (!autoCycle) {
      setFan(0);
      fanHigh = false;
      Serial.println(F("# auto OFF"));
    } else {
      lastToggleMs = millis();
      Serial.println(F("# auto ON"));
    }
    return;
  }
  if (line.startsWith("OT2;")) {
    autoCycle = false;  // manual takes over
    setFan(parseDuty(line.substring(4)));
    Serial.print(F("# manual OT2="));
    Serial.println(levelOT2);
    return;
  }
  if (line.startsWith("OT1;") || line.startsWith("IO3;") ||
      line.startsWith("DCFAN;") || line.startsWith("FILT;") ||
      line.startsWith("PID;") || line.startsWith("CHAN;") ||
      line == "RESET") {
    return;
  }
  if (line.startsWith("UNIT;")) {
    tempUnit = line.substring(5).startsWith("F") ? UNIT_F : UNIT_C;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  pinMode(PIN_OT1_HEATER, OUTPUT);
  pinMode(PIN_OT2_FAN, OUTPUT);
  otWrite(PIN_OT1_HEATER, false);
  otWrite(PIN_OT2_FAN, false);

  SPI.begin(PIN_MAX_SCK, PIN_MAX_MISO, 23, PIN_MAX_CS);
  thermocouple.begin();

  pinMode(PIN_ZCD, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ZCD), onZcd, CHANGE);

  Serial.println(F("# stage3_fan ready"));
  Serial.println(F("# STEADY;1 = opto meter test (no SR800). Normal: wait for ZCD auto PAC"));
}

void loop() {
  servicePhase();
  serviceZcdLockAndAuto();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuf.length()) {
        handleCommand(cmdBuf);
        cmdBuf = "";
      }
    } else if (cmdBuf.length() < 80) {
      cmdBuf += c;
    }
  }
}
