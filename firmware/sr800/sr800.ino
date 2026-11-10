/*
 * stage4_full — Independent OT1 (heat) + OT2 (fan) phase-angle
 *
 * ZCD is POLLED (no GPIO interrupt). Heater EMI on Q6B was causing
 * interrupt storms → Interrupt WDT → reboot (ets Jul 29...).
 *
 * Wiring (NPN + 10k base→GND pull-downs):
 *   26-[1k]-NPN B; C=yellow (heat); E=GND
 *   27-[1k]-NPN B; C=brown  (fan);  E=GND
 *   25=ZCD (Q6B)
 *   MAX31855: VIN=3V3 CLK=18 DO=19 CS=5
 *
 * SINGLE FILE — delete config.h if present in this sketch folder.
 *
 * Link: USB Serial and/or Bluetooth Classic SPP (name "SR800-TC4").
 * Artisan → Device TC4 → pick /dev/cu.SR800-TC4-... @ 115200.
 *
 * Stock Artisan reads one line per READ and float()-parses CSV fields.
 * Unsolicited # ZCD-BAD / failsafe spam on that port can steal the line
 * (→ −1 drops). After the first Artisan TC4 command we go roast-quiet:
 * only command replies on that port — no diag chatter. VERBOSE;1 re-enables.
 *
 * READ TX is formalized: ASCII-only CSV, one atomic write(), deferred until
 * TRIAC gate pulses are idle so we aren't bit-banging USB during the EMI
 * spike we create. That cuts self-mangling; external USB noise can still
 * flip bits in transit.
 *
 * USB roast stability: Classic BT stays OFF by default (BTON to enable).
 * When Artisan talks on USB we force BT off so SPP reconnect stalls / BT
 * radio current can't brown out or wedge the USB link mid-roast.
 */

#include <SPI.h>
#include <Adafruit_MAX31855.h>
#include <BluetoothSerial.h>
#include <esp_system.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth not enabled — use ESP32 board with Classic BT (not ESP32-C3).
#endif

// 0 = USB-first (recommended). Send BTON over USB if you want SPP.
#ifndef BT_ENABLE_AT_BOOT
#define BT_ENABLE_AT_BOOT 0
#endif

// ----- Pins / config (inlined — DELETE any config.h in this folder) -----
static const int PIN_OT1_HEATER = 26;
static const int PIN_OT2_FAN    = 27;
static const int PIN_ZCD        = 25;
static const int PIN_MAX_CS     = 5;
static const int PIN_MAX_SCK    = 18;
static const int PIN_MAX_MISO   = 19;
static const uint32_t HALF_PERIOD_US = 8333;  // US 60 Hz
static const uint32_t TRIAC_PULSE_US = 1200;
static const uint32_t SERIAL_BAUD = 115200;
static const uint8_t HTR_CUTOFF_FAN_VAL = 22;
static const int OVERTEMP_F = 500;
static const int OVERTEMP_CLEAR_F = 480;
static const int DEFAULT_UNIT_F = 1;
#ifndef OT_ACTIVE_HIGH
#define OT_ACTIVE_HIGH 1
#endif

// Software SPI (SCK, CS, DO) — breadboard Dupont wires are unreliable on HW SPI
Adafruit_MAX31855 thermocouple(PIN_MAX_SCK, PIN_MAX_CS, PIN_MAX_MISO);

// Bluetooth Classic SPP — same TC4 text protocol as USB
BluetoothSerial SerialBT;
static const char* BT_NAME = "SR800-TC4";
// Reply to the port that sent the command (USB or BT)
static Print* gReply = &Serial;
static Print* gRawOut = &Serial;  // RAW stream follows the port that enabled it
// Port Artisan is talking on — never emit unsolicited # lines there
static Print* gArtisanPort = nullptr;
static bool roastQuiet = false;   // set by Artisan TC4 cmds; VERBOSE;1 clears
static uint32_t lastArtisanMs = 0;

static bool btStackOk = false;
static bool btWanted = false;  // user/enabled intent; false during USB Artisan roast
static volatile bool btNeedRestart = false;
static esp_reset_reason_t gLastReset = ESP_RST_UNKNOWN;
RTC_DATA_ATTR uint32_t gBootCount = 0;

static bool btHasClient() {
  return btStackOk && SerialBT.hasClient();
}

static bool usbArtisanActive() {
  return roastQuiet && gArtisanPort == &Serial;
}

// SPP often won't accept a second Mac connect until the stack is bounced.
// Do NOT call end()/begin() from this callback — set a flag for loop().
static void onBtSppEvent(esp_spp_cb_event_t event, esp_spp_cb_param_t* /*param*/) {
  if (event == ESP_SPP_CLOSE_EVT) {
    btNeedRestart = true;
  }
}

static void btStopStack(const char* why) {
  if (!btStackOk) {
    btNeedRestart = false;
    return;
  }
  Serial.print(F("# BT off ("));
  Serial.print(why);
  Serial.println(F(")"));
  SerialBT.end();
  btStackOk = false;
  btNeedRestart = false;
}

static bool btStartStack() {
  if (usbArtisanActive()) {
    Serial.println(F("# BT blocked — USB Artisan active (BTOFF roast mode)"));
    return false;
  }
  if (!SerialBT.begin(BT_NAME)) {
    btStackOk = false;
    return false;
  }
  SerialBT.register_callback(onBtSppEvent);
  btStackOk = true;
  btWanted = true;
  btNeedRestart = false;
  return true;
}

static void btRestartStack(const char* why) {
  if (!btWanted || usbArtisanActive()) {
    btNeedRestart = false;
    return;
  }
  Serial.print(F("# BT SPP restart ("));
  Serial.print(why);
  Serial.println(F(")…"));
  SerialBT.end();
  btStackOk = false;
  delay(120);
  if (btStartStack()) {
    Serial.println(F("# BT SPP listening again — reconnect Mac / Artisan"));
  } else {
    Serial.println(F("# BT restart FAILED — use USB or power-cycle ESP"));
  }
}

// Declared later — .ino prototype gen breaks templates, so no template<> helpers
extern bool rawTempMode;
extern bool heatPulseActive;
extern bool fanPulseActive;
static void servicePhase();

static void markArtisanActive() {
  gArtisanPort = gReply;
  lastArtisanMs = millis();
  roastQuiet = true;
  rawTempMode = false;  // RAW spam must never run under Artisan
  // USB control path: kill Classic BT so reconnect delays / radio current
  // can't stall PAC or knock USB offline mid-roast.
  if (gReply == &Serial && btStackOk) {
    btWanted = false;
    btStopStack("USB Artisan");
  }
}

// Artisan/TC4 replies → requesting port only (explicit overloads; Arduino IDE
// strips/breaks template <typename T> in .ino files)
static void replyPrint(const __FlashStringHelper* v) { gReply->print(v); }
static void replyPrint(const char* v) { gReply->print(v); }
static void replyPrint(int v) { gReply->print(v); }
static void replyPrint(unsigned int v) { gReply->print(v); }
static void replyPrint(unsigned long v) { gReply->print(v); }
static void replyPrint(long v) { gReply->print(v); }
static void replyPrintln() { gReply->println(); }
static void replyPrintln(const __FlashStringHelper* v) { gReply->println(v); }
static void replyPrintln(const char* v) { gReply->println(v); }
static void replyPrintln(int v) { gReply->println(v); }
static void replyPrintln(unsigned int v) { gReply->println(v); }
static void replyPrintln(unsigned long v) { gReply->println(v); }

// Last good READ CSV (no newline) — fallback if format sanitize fails
static char lastArtisanCsv[96] = "0.00,0.00,0.00,0.00,0.00,0,0";

static bool csvByteOk(char c) {
  return (c >= '0' && c <= '9') || c == '.' || c == '-' || c == ',';
}

static bool txElectricallyQuiet() {
  return !heatPulseActive && !fanPulseActive;
}

// Don't shift UART bits while our own TRIAC gate is slamming the mains.
// Artisan waits ~100ms after READ before readline — budget a few ms here.
static void waitTxQuietWindow() {
  const uint32_t maxWaitUs = 6000;
  uint32_t t0 = micros();
  while (!txElectricallyQuiet()) {
    servicePhase();
    if ((uint32_t)(micros() - t0) >= maxWaitUs) break;
  }
  // ~200µs settle after gate-off edge
  uint32_t t1 = micros();
  while ((uint32_t)(micros() - t1) < 200) {
    servicePhase();
  }
}

// Stock Artisan CSV only: sanitize → single write(buf) incl. '\\n'. No multi-print.
static void replyArtisanCsv(const char* csv) {
  char body[96];
  size_t n = 0;
  bool ok = true;
  for (const char* p = csv; *p; ++p) {
    if (!csvByteOk(*p)) {
      ok = false;
      break;
    }
    if (n + 1 >= sizeof(body)) {
      ok = false;
      break;
    }
    body[n++] = *p;
  }
  body[n] = '\0';
  if (!ok || n < 11) {  // bare minimum "0.00,0.00,0"
    strncpy(body, lastArtisanCsv, sizeof(body) - 1);
    body[sizeof(body) - 1] = '\0';
    n = strlen(body);
  } else {
    memcpy(lastArtisanCsv, body, n + 1);
  }

  char out[100];
  if (n + 1 >= sizeof(out)) n = sizeof(out) - 2;
  memcpy(out, body, n);
  out[n++] = '\n';

  waitTxQuietWindow();
  gReply->write(reinterpret_cast<const uint8_t*>(out), n);
}

// Unsolicited diag/failsafe logs. Skip entirely while roast-quiet so stock
// Artisan never readline()'s a # ZCD-BAD instead of the READ CSV.
static bool logSinkUsb() {
  if (roastQuiet && gArtisanPort == &Serial) return false;
  return true;
}
static bool logSinkBt() {
  if (!btHasClient()) return false;
  if (roastQuiet && gArtisanPort == static_cast<Print*>(&SerialBT)) return false;
  return true;
}
static void logPrint(const __FlashStringHelper* s) {
  if (logSinkUsb()) Serial.print(s);
  if (logSinkBt()) SerialBT.print(s);
}
static void logPrintln(const __FlashStringHelper* s) {
  if (logSinkUsb()) Serial.println(s);
  if (logSinkBt()) SerialBT.println(s);
}
static void logPrint(float v, int digits) {
  if (logSinkUsb()) Serial.print(v, digits);
  if (logSinkBt()) SerialBT.print(v, digits);
}
static void logPrint(unsigned long v) {
  if (logSinkUsb()) Serial.print(v);
  if (logSinkBt()) SerialBT.print(v);
}
static void logPrint(unsigned int v) {
  if (logSinkUsb()) Serial.print(v);
  if (logSinkBt()) SerialBT.print(v);
}
static void logPrint(int v) {
  if (logSinkUsb()) Serial.print(v);
  if (logSinkBt()) SerialBT.print(v);
}
static void logPrintln(int v) {
  if (logSinkUsb()) Serial.println(v);
  if (logSinkBt()) SerialBT.println(v);
}
static void logPrintln(const char* s) {
  if (logSinkUsb()) Serial.println(s);
  if (logSinkBt()) SerialBT.println(s);
}
static void logPrintln() {
  if (logSinkUsb()) Serial.println();
  if (logSinkBt()) SerialBT.println();
}

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
float filtBtC = NAN;      // light EMA after spike rejection
float lastSentBtC = NAN;  // last BT actually emitted on READ (never send 0/nan)
uint8_t tcFaultCount = 0;
uint8_t btDropConfirm = 0;  // consecutive agreeing low samples
float btDropCandidateC = NAN;
uint32_t btDropStartMs = 0;   // wall-clock start of drop candidate
uint32_t lastBtSampleMs = 0;
static const uint8_t BT_DROP_NEED_N = 8;          // agree this many READ samples
static const uint32_t BT_DROP_NEED_MS = 8000;     // and ≥8s wall clock
static const float BT_DROP_DELTA_C = 6.0f;        // single-step drop threshold (°C)
static const float BT_CJ_NEAR_C = 25.0f;          // mid within CJ+this = ambient glitch
// Last line of defense for Artisan plot — max change per READ emit (~1Hz)
static const float ARTISAN_MAX_DOWN_C = 2.0f;
static const float ARTISAN_MAX_UP_C = 6.0f;
// Background ring: MAX31855 converts ~10Hz — space samples, don't triple-read same result
static const uint8_t BT_RING_N = 5;
static const uint32_t BT_RING_PERIOD_MS = 100;
static float btRingC[BT_RING_N];
static uint8_t btRingErr[BT_RING_N];
static uint8_t btRingIdx = 0;
static uint8_t btRingFill = 0;
static uint32_t btRingLastMs = 0;
static uint8_t btRingFaultStreak = 0;

bool heatPulseActive = false;  // also referenced earlier via extern for TX quiet-window
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
String cmdBufUsb;
String cmdBufBt;
bool rawTempMode = false;  // RAW — unfiltered BT stream (Serial Monitor)

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
    // Stale lock: if gap is way past 2 half-cycles, re-acquire on next edge
    // (star-ground / EMI can wedge lastEdgeUs so every gap looks "wrong").
    if (gap > (HALF_PERIOD_US * 3)) {
      lastEdgeUs = 0;
    }
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

// BT filter: hold last-good, hard-reject ambient EMI, wall-clock drop confirm.

static bool looksLikeAmbientGlitch(float mid, float cj, float lastC) {
  // Hot roast suddenly reading near cold-junction / room → EMI, never accept
  if (isnan(lastC) || lastC < 60.0f) return false;
  if (!isnan(cj) && mid < (cj + BT_CJ_NEAR_C) && (lastC - mid) > 20.0f)
    return true;
  // Absolute room-ish floor while we were already roasting-hot
  if (lastC > 80.0f && mid < 40.0f && (lastC - mid) > 25.0f)
    return true;
  return false;
}

// Continuous ~10Hz sampler — each slot is a *different* MAX31855 conversion.
static void serviceBtSampler() {
  uint32_t now = millis();
  if (btRingLastMs != 0 && (now - btRingLastMs) < BT_RING_PERIOD_MS) return;
  btRingLastMs = now;

  float c = thermocouple.readCelsius();
  uint8_t err = thermocouple.readError();
  btRingC[btRingIdx] = c;
  btRingErr[btRingIdx] = err;
  btRingIdx = (uint8_t)((btRingIdx + 1) % BT_RING_N);
  if (btRingFill < BT_RING_N) btRingFill++;

  if (err != 0 || isnan(c)) {
    if (btRingFaultStreak < 255) btRingFaultStreak++;
  } else {
    btRingFaultStreak = 0;
  }
}

// Median of recent *valid* ring samples. Needs ≥2 good conversions.
static bool btRingMedian(float* outMid, float* outSpread) {
  float vals[BT_RING_N];
  int n = 0;
  for (uint8_t i = 0; i < btRingFill; i++) {
    if (btRingErr[i] != 0) continue;
    float v = btRingC[i];
    if (isnan(v) || v < -20.0f || v > 400.0f) continue;
    vals[n++] = v;
  }
  if (n < 2) return false;
  // insertion sort — n≤5
  for (int i = 1; i < n; i++) {
    float key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) {
      vals[j + 1] = vals[j];
      j--;
    }
    vals[j + 1] = key;
  }
  *outMid = vals[n / 2];
  *outSpread = vals[n - 1] - vals[0];
  return true;
}

// forArtisanRead=true advances drop-confirm; overtemp path must pass false
// so 500ms OT checks don't accelerate false drop acceptance.
static float readBtCelsius(bool forArtisanRead) {
  serviceBtSampler();  // ensure at least one fresh attempt

  float mid = NAN;
  float spread = 0.0f;
  uint8_t err = 0;
  float cj = thermocouple.readInternal();

  if (forArtisanRead) {
    // Prefer ring median (multiple conversions over ~0.5s) over one SPI grab
    if (!btRingMedian(&mid, &spread)) {
      err = 1;
    } else if (spread > 12.0f) {
      err = 1;  // ring disagreeing → treat as fault, hold last-good
    }
  } else {
    mid = thermocouple.readCelsius();
    err = thermocouple.readError();
    spread = 0.0f;
  }

  uint32_t now = millis();
  uint32_t dtMs = (lastBtSampleMs == 0) ? 1000 : (now - lastBtSampleMs);
  if (dtMs < 1) dtMs = 1;
  lastBtSampleMs = now;

  // Burst of faults → hold (don't let Artisan chase noise)
  if (btRingFaultStreak >= 3) {
    tcFaultCount++;
    if (forArtisanRead) {
      btDropConfirm = 0;
      btDropCandidateC = NAN;
      btDropStartMs = 0;
    }
    return lastGoodBtC;
  }

  if (err != 0 || isnan(mid)) {
    tcFaultCount++;
    if (forArtisanRead) {
      btDropConfirm = 0;
      btDropCandidateC = NAN;
      btDropStartMs = 0;
    }
    return lastGoodBtC;  // may be NAN if never locked — caller must not emit 0
  }
  if (mid < -20.0f || mid > 400.0f) {
    tcFaultCount++;
    if (forArtisanRead) {
      btDropConfirm = 0;
      btDropCandidateC = NAN;
      btDropStartMs = 0;
    }
    return lastGoodBtC;
  }

  if (!isnan(lastGoodBtC)) {
    // Hard reject ambient/CJ EMI plunges — never "confirm" these into Artisan
    if (looksLikeAmbientGlitch(mid, cj, lastGoodBtC)) {
      tcFaultCount++;
      if (forArtisanRead) {
        btDropConfirm = 0;
        btDropCandidateC = NAN;
        btDropStartMs = 0;
      }
      return lastGoodBtC;
    }

    float delta = mid - lastGoodBtC;

    // --- Drops: wall-clock + count confirm; still reject if near ambient ---
    if (forArtisanRead && delta < -BT_DROP_DELTA_C) {
      if (btDropConfirm == 0 || isnan(btDropCandidateC) ||
          fabsf(mid - btDropCandidateC) > 5.0f) {
        btDropConfirm = 1;
        btDropCandidateC = mid;
        btDropStartMs = now;
        tcFaultCount++;
        return lastGoodBtC;
      }
      btDropConfirm++;
      btDropCandidateC = 0.7f * btDropCandidateC + 0.3f * mid;
      uint32_t lived = now - btDropStartMs;
      if (btDropConfirm < BT_DROP_NEED_N || lived < BT_DROP_NEED_MS) {
        tcFaultCount++;
        return lastGoodBtC;
      }
      // Final gate: still looks like ambient? refuse
      if (looksLikeAmbientGlitch(btDropCandidateC, cj, lastGoodBtC)) {
        btDropConfirm = 0;
        btDropCandidateC = NAN;
        btDropStartMs = 0;
        tcFaultCount++;
        return lastGoodBtC;
      }
      // Hot roast + big plunge: demand even longer (charge is gradual; EMI isn't)
      if (lastGoodBtC > 100.0f && (lastGoodBtC - btDropCandidateC) > 25.0f &&
          lived < 15000) {
        tcFaultCount++;
        return lastGoodBtC;
      }
      // sustained real drop accepted into filter (Artisan emit still slew-limited)
      mid = btDropCandidateC;
      btDropConfirm = 0;
      btDropCandidateC = NAN;
      btDropStartMs = 0;
    } else if (forArtisanRead) {
      btDropConfirm = 0;
      btDropCandidateC = NAN;
      btDropStartMs = 0;
    }

    // --- Rises: only reject absurd SPI junk ---
    if (delta > 80.0f) {
      tcFaultCount++;
      return lastGoodBtC;
    }
    float maxUp = 40.0f * ((float)dtMs / 1000.0f);
    if (maxUp < 8.0f) maxUp = 8.0f;
    if (maxUp > 60.0f) maxUp = 60.0f;
    if (delta > maxUp) mid = lastGoodBtC + maxUp;

    if (delta > -BT_DROP_DELTA_C && delta < 0.0f) {
      float maxDown = 15.0f * ((float)dtMs / 1000.0f);
      if (maxDown < 4.0f) maxDown = 4.0f;
      if (-delta > maxDown) mid = lastGoodBtC - maxDown;
    }
  }

  if (isnan(filtBtC)) {
    filtBtC = mid;
  } else if (mid >= filtBtC) {
    filtBtC = 0.45f * filtBtC + 0.55f * mid;
  } else {
    filtBtC = 0.75f * filtBtC + 0.25f * mid;
  }

  lastGoodBtC = filtBtC;
  return filtBtC;
}

static void respondRead() {
  float rawC = readBtCelsius(true);
  float ambC = thermocouple.readInternal();

  // Never emit 0/nan BT to Artisan (0°C→32°F glitch / 0°F cliff)
  if (isnan(rawC)) {
    if (!isnan(lastSentBtC)) rawC = lastSentBtC;
    else if (!isnan(lastGoodBtC)) rawC = lastGoodBtC;
  }

  // Emit slew clamp — Artisan plots whatever we send; never cliff in one sample.
  // (Filter may eventually accept a real drop; we ease Artisan there ≤2°C/READ.)
  if (!isnan(rawC) && !isnan(lastSentBtC)) {
    if (looksLikeAmbientGlitch(rawC, ambC, lastSentBtC)) {
      rawC = lastSentBtC;
    } else {
      float d = rawC - lastSentBtC;
      if (d < -ARTISAN_MAX_DOWN_C) rawC = lastSentBtC - ARTISAN_MAX_DOWN_C;
      else if (d > ARTISAN_MAX_UP_C) rawC = lastSentBtC + ARTISAN_MAX_UP_C;
    }
    // Hot roast: refuse room-ish emits even if filter slipped
    if (lastSentBtC > 80.0f && rawC < 50.0f) rawC = lastSentBtC;
  }

  // Still no BT lock — CJ only before first lock (desk). Never replace a hot
  // lastSent with ambient (that was a classic Artisan cliff).
  if (isnan(rawC)) {
    if (!isnan(lastSentBtC)) {
      rawC = lastSentBtC;
    } else if (!isnan(ambC)) {
      rawC = ambC;
    } else {
      replyArtisanCsv("0.00,0.00,0.00,0.00,0.00,0,0");
      return;
    }
  }

  if (!isnan(rawC)) lastSentBtC = rawC;

  float amb = toUnit(ambC);
  float bt = toUnit(rawC);

  if (!isnan(rawC)) {
    float btF = rawC * 9.0f / 5.0f + 32.0f;
    if (!overtempLatch && btF >= (float)OVERTEMP_F) tripOvertemp(btF);
    else if (overtempLatch && btF <= (float)OVERTEMP_CLEAR_F) {
      overtempLatch = false;
      logPrint(F("# overtemp clear BT="));
      logPrint(btF, 1);
      logPrintln(F("F — OT1 allowed again"));
    }
  }

  // Guard non-finite → snprintf "nan"/"inf" would break Artisan float()
  if (!isfinite(amb)) amb = 0.0f;
  if (!isfinite(bt)) {
    bt = toUnit(lastSentBtC);
    if (!isfinite(bt)) bt = 0.0f;
  }

  uint8_t heatOut = effectiveHeat(levelOT1, levelOT2);
  char line[96];
  snprintf(line, sizeof(line), "%.2f,%.2f,%.2f,0.00,0.00,%u,%u",
           amb, bt, bt,
           (unsigned)heatOut, (unsigned)levelOT2);
  replyArtisanCsv(line);
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
  logPrint(F("# FAILSAFE: BT "));
  logPrint(btF, 1);
  logPrint(F("F >= "));
  logPrint(OVERTEMP_F);
  logPrintln(F("F — OT1 cut (clears below 480F)"));
}

// BT check in °F regardless of Artisan UNIT setting.
static void serviceOvertemp() {
  static uint32_t lastCheck = 0;
  uint32_t now = millis();
  if (now - lastCheck < OVERTEMP_CHECK_MS) return;
  lastCheck = now;

  // Skip when idle (no heat/fan and not latched)
  if (levelOT1 == 0 && levelOT2 == 0 && !overtempLatch) return;

  float c = readBtCelsius(false);
  if (isnan(c)) {
    if (isnan(lastGoodBtC)) return;
    c = lastGoodBtC;
  }
  float btF = c * 9.0f / 5.0f + 32.0f;

  if (!overtempLatch && btF >= (float)OVERTEMP_F) {
    tripOvertemp(btF);
    return;
  }
  if (overtempLatch && btF <= (float)OVERTEMP_CLEAR_F) {
    overtempLatch = false;
    logPrint(F("# overtemp clear BT="));
    logPrint(btF, 1);
    logPrintln(F("F — OT1 allowed again"));
  }
}

static void printResetReason() {
  gLastReset = esp_reset_reason();
  gBootCount++;
  logPrint(F("# reset="));
  switch (gLastReset) {
    case ESP_RST_POWERON: logPrintln(F("POWERON")); break;
    case ESP_RST_SW: logPrintln(F("SW")); break;
    case ESP_RST_PANIC: logPrintln(F("PANIC")); break;
    case ESP_RST_INT_WDT: logPrintln(F("INT_WDT")); break;
    case ESP_RST_TASK_WDT: logPrintln(F("TASK_WDT")); break;
    case ESP_RST_WDT: logPrintln(F("WDT")); break;
    case ESP_RST_BROWNOUT: logPrintln(F("BROWNOUT")); break;
    case ESP_RST_SDIO: logPrintln(F("SDIO")); break;
    default: logPrintln((int)gLastReset); break;
  }
  logPrint(F("# boots="));
  logPrint((unsigned long)gBootCount);
  logPrintln();
  // These usually mean "Artisan USB dropped" was actually an ESP reboot.
  if (gLastReset == ESP_RST_BROWNOUT) {
    logPrintln(F("# !! BROWNOUT — weak USB 5V / EMI; use short cable + isolator, or 5V on VIN"));
  } else if (gLastReset == ESP_RST_INT_WDT || gLastReset == ESP_RST_TASK_WDT ||
             gLastReset == ESP_RST_WDT) {
    logPrintln(F("# !! WDT reboot — was crashing under load; note OT1/OT2 when it dies"));
  } else if (gLastReset == ESP_RST_SW) {
    logPrintln(F("# SW reset — often DTR/RTS from opening Serial (Artisan/Monitor)"));
  }
}

static void handleCommand(String line) {
  line.trim();
  if (!line.length()) return;
  line.replace(',', ';');
  line.toUpperCase();

  if (line == "READ") {
    markArtisanActive();
    respondRead();
    return;
  }
  if (line == "STAT") {
    // STAT is intentional diag — reply even when quiet; does not re-enable spam
    replyPrint(F("# zcdOk="));
    replyPrint(zcdEverOk ? 1 : 0);
    replyPrint(F(" edges="));
    replyPrint(zcdEdges);
    replyPrint(F(" rej="));
    replyPrint(rejectedEdges);
    replyPrint(F(" halfUs="));
    replyPrint(halfPeriodUs);
    replyPrint(F(" OT1="));
    replyPrint(levelOT1);
    replyPrint(F(" OT2="));
    replyPrint(levelOT2);
    replyPrint(F(" ot="));
    replyPrint(overtempLatch ? 1 : 0);
    replyPrint(F(" tcFaults="));
    replyPrint(tcFaultCount);
    replyPrint(F(" quiet="));
    replyPrint(roastQuiet ? 1 : 0);
    replyPrint(F(" bt="));
    replyPrint(btHasClient() ? 1 : 0);
    replyPrint(F(" btOk="));
    replyPrint(btStackOk ? 1 : 0);
    replyPrint(F(" btWant="));
    replyPrint(btWanted ? 1 : 0);
    replyPrint(F(" boots="));
    replyPrint((unsigned long)gBootCount);
    replyPrint(F(" rst="));
    replyPrint((int)gLastReset);
    replyPrint(F(" ring="));
    replyPrint(btRingFill);
    replyPrint(F(" faultStreak="));
    replyPrintln(btRingFaultStreak);
    return;
  }
  if (line == "BTON") {
    btWanted = true;
    if (btStartStack()) replyPrintln(F("# BT on — SR800-TC4 listening"));
    else replyPrintln(F("# BT on FAILED"));
    return;
  }
  if (line == "BTOFF") {
    btWanted = false;
    btStopStack("BTOFF");
    replyPrintln(F("# BT off"));
    return;
  }
  if (line == "BTRESTART") {
    // Manual recovery when Mac won't reconnect (also auto on drop)
    replyPrintln(F("# BTRESTART"));
    btWanted = true;
    btNeedRestart = false;
    btRestartStack("cmd");
    return;
  }
  if (line == "VERBOSE" || line == "VERBOSE;1" || line == "NOISY") {
    roastQuiet = false;
    gArtisanPort = nullptr;
    replyPrintln(F("# verbose on — ZCD-BAD / failsafe logs enabled"));
    return;
  }
  if (line == "VERBOSE;0" || line == "QUIET") {
    roastQuiet = true;
    gArtisanPort = gReply;
    rawTempMode = false;
    replyPrintln(F("# quiet on — Artisan-safe (no unsolicited logs)"));
    return;
  }
  if (line == "RAW" || line == "RAW;1" || line == "RAWON") {
    // RAW floods the port — force verbose diag path, leave Artisan closed
    roastQuiet = false;
    gArtisanPort = nullptr;
    rawTempMode = true;
    gRawOut = gReply;
    replyPrintln(F("# RAW on — unfiltered BT ~50Hz (close Artisan; RAW;0 to stop)"));
    return;
  }
  if (line == "RAW;0" || line == "RAWOFF") {
    rawTempMode = false;
    replyPrintln(F("# RAW off"));
    return;
  }
  if (line == "RESET") {
    markArtisanActive();
    levelOT1 = 0;
    levelOT2 = 0;
    overtempLatch = false;
    rawTempMode = false;
    btDropConfirm = 0;
    btDropCandidateC = NAN;
    btDropStartMs = 0;
    // keep lastGoodBtC / lastSentBtC — reconnect shouldn't cliff to 0/32
    outputsAllOff();
    // stock Artisan expects optional # ack; keep silent under quiet
#if DEBUG_SERIAL
    if (!roastQuiet) Serial.println(F("# RESET"));
#endif
    return;
  }
  if (line.startsWith("OT1;")) {
    markArtisanActive();
    uint8_t req = parseDuty(line.substring(4));
    if (overtempLatch && req > 0) {
      levelOT1 = 0;
      heatAllOff();
      // no reply — Artisan OT1 doesn't readline; stay quiet
      return;
    }
    levelOT1 = req;
    applyOutputs();
#if DEBUG_SERIAL
    if (!roastQuiet) {
      Serial.print(F("# OT1="));
      Serial.print(levelOT1);
      Serial.print(F(" eff="));
      Serial.println(effectiveHeat(levelOT1, levelOT2));
    }
#endif
    return;
  }
  if (line.startsWith("OT2;")) {
    markArtisanActive();
    levelOT2 = clampFan(parseDuty(line.substring(4)));
    applyOutputs();
#if DEBUG_SERIAL
    if (!roastQuiet) {
      Serial.print(F("# OT2="));
      Serial.println(levelOT2);
    }
#endif
    return;
  }
  // Artisan sends UNITS;F — accept UNIT; and UNITS;
  if (line.startsWith("UNITS;") || line.startsWith("UNIT;")) {
    markArtisanActive();
    int sep = line.indexOf(';');
    String body = line.substring(sep + 1);
    tempUnit = body.startsWith("F") ? UNIT_F : UNIT_C;
    // stock Artisan wants a '#' ack (or empty timeout)
    replyPrintln(F("# ok"));
    return;
  }
  if (line.startsWith("CHAN;")) {
    markArtisanActive();
    String body = line.substring(5);
    for (int i = 0; i < 4; i++) {
      if (i < (int)body.length() && isDigit(body[i]))
        chanMap[i] = (uint8_t)(body[i] - '0');
      else
        chanMap[i] = 0;
    }
    replyPrintln(F("# ok"));
    return;
  }
  if (line.startsWith("FILT;")) {
    markArtisanActive();
    // filtering is on-ESP already; ACK so Artisan init proceeds
    replyPrintln(F("# ok"));
    return;
  }
  if (line.startsWith("IO3;") || line.startsWith("DCFAN;") ||
      line.startsWith("PID;")) {
    markArtisanActive();
    return;
  }
}


// Continuous ZCD diagnostics when signal is not healthy (no need for stage2 flash).
// Quiet when ~40–180 accepted edges/s; otherwise prints once per second.

// Unfiltered BT stream for probe/EMI benching. No median/drop/EMA guards.
// MAX31855 updates ~10 Hz; we poll faster so spikes aren't hidden.
static void serviceRawTemp() {
  if (!rawTempMode) return;
  static uint32_t lastMs = 0;
  static uint32_t maskedFaults = 0;
  uint32_t now = millis();
  if (now - lastMs < 20) return;  // ~50 Hz print; chip data repeats between conversions
  lastMs = now;

  float c = thermocouple.readCelsius();
  float cj = thermocouple.readInternal();
  uint8_t err = thermocouple.readError();

  // Mask fault / NaN samples — don't print them (EMI often sets err=2/4 briefly).
  // USB "????" garble on good lines is wire EMI after TX; can't un-corrupt that here.
  if (err != 0 || isnan(c)) {
    maskedFaults++;
    static uint32_t lastMaskMsg = 0;
    if (now - lastMaskMsg >= 1000) {
      lastMaskMsg = now;
      gRawOut->print(F("# RAW masked faults/s~ "));
      gRawOut->print(maskedFaults);
      gRawOut->print(F(" lastErr="));
      gRawOut->println(err);
      maskedFaults = 0;
    }
    return;
  }

  float f = c * 9.0f / 5.0f + 32.0f;
  // One-shot line (fewer interleaved UART gaps under EMI)
  char line[96];
  snprintf(line, sizeof(line),
           "RAW t_ms=%lu BT_C=%.2f BT_F=%.2f CJ_C=%.2f err=0 pinZCD=%d",
           (unsigned long)now, c, f,
           isnan(cj) ? 0.0f : cj,
           digitalRead(PIN_ZCD) ? 1 : 0);
  gRawOut->println(line);
}

static void serviceZcdDiag() {
  static uint32_t lastPrintMs = 0;
  static uint32_t lastEdges = 0;
  static uint32_t lastRej = 0;

  uint32_t now = millis();
  if (now - lastPrintMs < 1000) return;
  lastPrintMs = now;

  uint32_t e = zcdEdges;
  uint32_t r = rejectedEdges;
  uint32_t de = e - lastEdges;
  uint32_t dr = r - lastRej;
  lastEdges = e;
  lastRej = r;

  // Still track health / re-lock while quiet — just don't print.
  bool zcdHealthy = (de >= 40 && de <= 180);
  if (!zcdHealthy && de == 0 && dr > 0) {
    lastEdgeUs = 0;  // force ZCD re-lock next edge
  }
  if (roastQuiet || zcdHealthy) return;

  char zline[160];
  snprintf(zline, sizeof(zline),
           "# ZCD-BAD edges/s=%lu rej/s=%lu totalEdges=%lu totalRej=%lu halfUs=%lu pin=%d OT1=%u OT2=%u zcdOk=%d",
           (unsigned long)de, (unsigned long)dr, (unsigned long)e, (unsigned long)r,
           (unsigned long)halfPeriodUs, digitalRead(PIN_ZCD) ? 1 : 0,
           (unsigned)levelOT1, (unsigned)levelOT2, zcdEverOk ? 1 : 0);
  logPrintln(zline);

  if (de == 0 && dr == 0) {
    logPrintln(F("#  → no toggles on GPIO25 — open wire, wrong pad, or SR800 off?"));
  } else if (de == 0 && dr > 0) {
    logPrintln(F("#  → pin noisy but gaps rejected — loose Q6B / bad GND / EMI"));
  } else if (de > 0 && de < 40) {
    logPrintln(F("#  → too few good edges — intermittent contact on Q6B"));
  } else if (de > 180) {
    logPrintln(F("#  → edge rate too high — noise; shorten wire / check pad"));
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
    logPrintln(F("# FAILSAFE: no ZCD 1.5s — HEAT off (fan keeps OT2 %)"));
  }

  if (levelOT2 > 0 && noEdgeMs >= ZCD_FAN_FAIL_MS) {
    levelOT2 = 0;
    fanAllOff();
    zcdEverOk = false;
    logPrintln(F("# FAILSAFE: no ZCD 4s — FAN off"));
    noEdgeMs = 0;
  }
}

// Idle-flush only for bare words. OT1/OT2 always need NL/CR so "OT2;10"+"0"
// can't become OT2;10 instead of OT2;100.
static bool cmdLooksComplete(const String& s) {
  return s.equalsIgnoreCase("READ") || s.equalsIgnoreCase("STAT") ||
         s.equalsIgnoreCase("RESET") || s.equalsIgnoreCase("RAW") ||
         s.equalsIgnoreCase("RAWON") || s.equalsIgnoreCase("RAWOFF") ||
         s.equalsIgnoreCase("VERBOSE") || s.equalsIgnoreCase("QUIET") ||
         s.equalsIgnoreCase("NOISY") || s.equalsIgnoreCase("BTRESTART") ||
         s.equalsIgnoreCase("BTON") || s.equalsIgnoreCase("BTOFF");
}

static bool serialCmdCharOk(char c) {
  // Drop USB/EMI garbage on RX so it can't form bogus OT/READ commands
  if (c >= 'A' && c <= 'Z') return true;
  if (c >= 'a' && c <= 'z') return true;
  if (c >= '0' && c <= '9') return true;
  return c == ';' || c == ',' || c == '.' || c == '-' || c == '_';
}

static void serviceOnePort(Stream& port, String& buf, uint32_t& lastByteMs) {
  while (port.available()) {
    char c = (char)port.read();
    lastByteMs = millis();
    if (c == '\n' || c == '\r') {
      if (buf.length()) {
        gReply = static_cast<Print*>(&port);
        handleCommand(buf);
        buf = "";
      }
    } else if (!serialCmdCharOk(c)) {
      buf = "";
    } else if (buf.length() < 80) {
      buf += c;
    } else {
      buf = "";
    }
  }
  if (buf.length() && (millis() - lastByteMs) >= CMD_IDLE_MS &&
      cmdLooksComplete(buf)) {
    gReply = static_cast<Print*>(&port);
    handleCommand(buf);
    buf = "";
  }
}

static void serviceSerial() {
  static uint32_t lastByteUsb = 0;
  static uint32_t lastByteBt = 0;
  serviceOnePort(Serial, cmdBufUsb, lastByteUsb);
  if (btHasClient()) {
    serviceOnePort(SerialBT, cmdBufBt, lastByteBt);
  } else if (cmdBufBt.length()) {
    cmdBufBt = "";  // drop partial if client gone
  }
}

// After Mac/Artisan drops SPP, bounce the server and fail-safe heater.
// (Defined here so levelOT1 / heatAllOff are in scope — Arduino .ino is picky.)
static void serviceBluetooth() {
  if (!btWanted) {
    btNeedRestart = false;
    return;
  }
  if (usbArtisanActive()) {
    // Should already be stopped by markArtisanActive; belt-and-suspenders.
    if (btStackOk) {
      btWanted = false;
      btStopStack("USB Artisan");
    }
    btNeedRestart = false;
    return;
  }

  static bool hadClient = false;
  bool client = btHasClient();

  if (hadClient && !client) {
    if (gArtisanPort == static_cast<Print*>(&SerialBT)) {
      levelOT1 = 0;
      heatAllOff();
      gArtisanPort = nullptr;
      roastQuiet = false;
      Serial.println(F("# BT dropped — OT1 cut (fan holds last OT2)"));
    } else {
      Serial.println(F("# BT client gone"));
    }
    btNeedRestart = true;
  } else if (client && !hadClient) {
    Serial.println(F("# BT client connected"));
  }
  hadClient = client;

  if (btNeedRestart && !client) {
    btNeedRestart = false;
    btRestartStack("disconnect");
    hadClient = false;
  }
}

void setup() {
  // Larger USB UART FIFOs — fewer overruns when heater EMI hits the link
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);
  delay(300);

  gReply = &Serial;
  gRawOut = &Serial;

#if BT_ENABLE_AT_BOOT
  if (!btStartStack()) {
    Serial.println(F("# BT start FAILED — USB-only mode"));
  } else {
    Serial.print(F("# BT SPP ready as "));
    Serial.println(BT_NAME);
  }
#else
  btWanted = false;
  Serial.println(F("# BT off at boot (USB roast mode). Send BTON for SPP."));
#endif

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
  logPrintln(F("# stage4 READY — OT1 cuts at BT>=500F (clears <480F)"));
  logPrintln(F("# OT1/OT2 need Newline (Artisan OK). FAN_MIN=5 heatCutoff=22"));
  logPrintln(F("# Artisan USB = CP2102 port @ 115200; BT optional via BTON"));
  logPrintln(F("# USB Artisan auto-kills BT (stops SPP restart stalls mid-roast)"));
  logPrintln(F("# After a drop: open Monitor, note # reset=… / STAT rst= / boots="));
  logPrintln(F("# BROWNOUT → power/EMI; SW → serial reopen reset; WDT → firmware hang"));
}

void loop() {
  serviceSerial();
  servicePhase();
  serviceBtSampler();
  serviceSerial();
  serviceBluetooth();
  serviceOvertemp();
  serviceWatchdog();
  serviceZcdDiag();
  serviceRawTemp();
}
