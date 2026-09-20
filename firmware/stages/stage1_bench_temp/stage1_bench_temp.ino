/*
 * stage1_bench_temp — Bench only (no SR800 wiring)
 *
 * Goals:
 *   - Read MAX31855 K-type as BT
 *   - Speak Artisan TC4-ish serial: READ / OT1 / OT2 / UNIT / CHAN
 *   - OT1/OT2 accepted but GPIO NOT driven (safe on the desk)
 *
 * Wiring:
 *   MAX31855 VCC→3V3 GND→GND SCK→18 SO→19 CS→5
 *   PC → USB isolator → ESP32
 *
 * Arduino IDE:
 *   Board: ESP32 Dev Module
 *   Library Manager: install "Adafruit MAX31855 library" (+ Adafruit BusIO)
 *   Open this folder as sketch, Upload
 *
 * Artisan:
 *   Device = TC4, Control checked, Port = ESP COM @ 115200
 *   BT channel 1 (or 2 — both carry BT in this stage)
 *   ON → finger on probe tip should raise BT
 */

#include <SPI.h>
#include <Adafruit_MAX31855.h>
#include "config.h"

Adafruit_MAX31855 thermocouple(PIN_MAX_CS);

enum TempUnit : uint8_t { UNIT_C = 0, UNIT_F = 1 };
TempUnit tempUnit = UNIT_F;

uint8_t levelOT1 = 0;
uint8_t levelOT2 = 0;

// Artisan CHAN;abcd — physical map nibbles; we keep a simple default
uint8_t chanMap[4] = {1, 2, 0, 0};  // T1=phys1, T2=phys2

String cmdBuf;

static float toUnit(float celsius) {
  if (isnan(celsius)) return celsius;
  return (tempUnit == UNIT_F) ? (celsius * 9.0f / 5.0f + 32.0f) : celsius;
}

static float readBT_C() {
  float c = thermocouple.readCelsius();
  if (isnan(c)) return NAN;
  return c;
}

static float readCJ_C() {
  float c = thermocouple.readInternal();
  if (isnan(c)) return NAN;
  return c;
}

static float physicalChannel(uint8_t phys /*1..4*/) {
  // phys1 = BT, phys2 = BT (duplicate), phys3/4 unused
  if (phys == 1 || phys == 2) return readBT_C();
  return NAN;
}

static void respondRead() {
  // TC4-style: ambient,T1,T2,T3,T4
  float amb = toUnit(readCJ_C());
  float t[4];
  for (int i = 0; i < 4; i++) {
    uint8_t phys = chanMap[i];
    float c = (phys == 0) ? NAN : physicalChannel(phys);
    t[i] = toUnit(c);
  }

  auto printVal = [](float v) {
    if (isnan(v)) Serial.print("0.00");
    else Serial.print(v, 2);
  };

  printVal(amb);
  for (int i = 0; i < 4; i++) {
    Serial.print(',');
    printVal(t[i]);
  }
  // Optional power echo helps Artisan logging later
  Serial.print(',');
  Serial.print(levelOT1);
  Serial.print(',');
  Serial.print(levelOT2);
  Serial.println();
}

static uint8_t parseDuty(const String& s) {
  int v = s.toInt();
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  return (uint8_t)v;
}

static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Normalize separators: Artisan may use ',' or ';'
  line.replace(',', ';');
  line.toUpperCase();

  if (line == "READ") {
    respondRead();
    return;
  }

  if (line.startsWith("OT1;")) {
    levelOT1 = parseDuty(line.substring(4));
    // Stage1: do not drive GPIO
    return;
  }
  if (line.startsWith("OT2;")) {
    levelOT2 = parseDuty(line.substring(4));
    return;
  }
  if (line.startsWith("IO3;")) {
    // Ignored in PAC builds; accept so Artisan doesn't stall
    return;
  }
  if (line.startsWith("UNIT;")) {
    String u = line.substring(5);
    tempUnit = (u.startsWith("F")) ? UNIT_F : UNIT_C;
    return;
  }
  if (line.startsWith("CHAN;")) {
    // CHAN;1200 style — up to 4 digits, each 0-4
    String body = line.substring(5);
    for (int i = 0; i < 4; i++) {
      if (i < (int)body.length() && isDigit(body[i])) {
        chanMap[i] = (uint8_t)(body[i] - '0');
      } else {
        chanMap[i] = 0;
      }
    }
    return;
  }
  if (line.startsWith("FILT;") || line.startsWith("PID;") ||
      line.startsWith("DCFAN;") || line == "RESET") {
    return;  // accepted no-ops for Artisan chatter
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  // Keep outputs safe even if pins float from earlier flashes
  pinMode(PIN_OT1_HEATER, OUTPUT);
  pinMode(PIN_OT2_FAN, OUTPUT);
  digitalWrite(PIN_OT1_HEATER, LOW);
  digitalWrite(PIN_OT2_FAN, LOW);

  SPI.begin(PIN_MAX_SCK, PIN_MAX_MISO, 23, PIN_MAX_CS);
  delay(100);
  if (!thermocouple.begin()) {
    Serial.println(F("# WARN: MAX31855 begin failed — check wiring"));
  }

  Serial.println(F("# stage1_bench_temp ready"));
  Serial.println(F("# MAX31855 + Artisan READ; default unit F; OT1/OT2 accepted, GPIO idle"));
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuf.length()) {
        handleCommand(cmdBuf);
        cmdBuf = "";
      }
    } else {
      if (cmdBuf.length() < 80) cmdBuf += c;
    }
  }
}
