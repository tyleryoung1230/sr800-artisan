/*
 * stage2_zcd — Zero-cross sense only (no heater/fan drive)
 *
 * Goals:
 *   - Interrupt on SR800 ZCD tap → GPIO25
 *   - Print edge rate over serial (expect ~120 Hz on 60 Hz mains)
 *   - OT outputs forced LOW
 *
 * Wiring (SR800 unplugged while soldering):
 *   SR800 ZCD sense (Q6B collector / derived logic ZCD) → ESP GPIO25
 *   SR800 logic GND → ESP GND
 *   PC → USB isolator → ESP32  (required once grounds are shared)
 *
 * Power-on: stock knobs still run the roaster; ESP only listens.
 *
 * Pass: "zcd_hz" near 120 (US) or 100 (50 Hz). Stable, not wild.
 */

#include "config.h"

volatile uint32_t zcdEdges = 0;
volatile uint32_t lastEdgeUs = 0;
volatile uint32_t lastPeriodUs = 0;

void IRAM_ATTR onZcd() {
  uint32_t now = micros();
  uint32_t prev = lastEdgeUs;
  lastEdgeUs = now;
  if (prev != 0) {
    lastPeriodUs = now - prev;
  }
  zcdEdges++;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  pinMode(PIN_OT1_HEATER, OUTPUT);
  pinMode(PIN_OT2_FAN, OUTPUT);
  digitalWrite(PIN_OT1_HEATER, LOW);
  digitalWrite(PIN_OT2_FAN, LOW);

  pinMode(PIN_ZCD, INPUT_PULLUP);
  // SR800 Q6B is often a ~60Hz square wave → CHANGE gives both edges (~120Hz).
  // If noisy, try FALLING or RISING instead.
  attachInterrupt(digitalPinToInterrupt(PIN_ZCD), onZcd, CHANGE);

  Serial.println(F("# stage2_zcd ready"));
  Serial.println(F("# Expect ~120 edges/s on 60Hz (CHANGE on both edges)"));
}

void loop() {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;

    noInterrupts();
    uint32_t edges = zcdEdges;
    zcdEdges = 0;
    uint32_t period = lastPeriodUs;
    interrupts();

    float hz = (float)edges;  // edges counted over ~1s
    Serial.print(F("zcd_hz="));
    Serial.print(hz, 1);
    Serial.print(F(" period_us="));
    Serial.print(period);
    Serial.print(F(" expect_half_us="));
    Serial.println(HALF_PERIOD_US);

    if (hz >= 50 && hz < 80) {
      Serial.println(F("# NOTE: ~60Hz = one edge/cycle. Reflash with CHANGE (default now)"));
      Serial.println(F("# or keep FALLING and we time on full-cycle period"));
    } else if (hz < 50) {
      Serial.println(F("# WARN: very low ZCD — wrong pad, bad GND, or no contact?"));
    } else if (hz > 200) {
      Serial.println(F("# WARN: high ZCD rate — noise; add series resistor / filtering"));
    } else {
      Serial.println(F("# OK: ZCD in band"));
    }
  }
}
