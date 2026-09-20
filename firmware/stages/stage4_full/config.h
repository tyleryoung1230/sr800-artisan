#pragma once
// =============================================================================
// SR800 + ESP32 Artisan control — shared pin / safety config
// Match these to your wiring. US mains = 60 Hz.
// =============================================================================

// ----- Pins (ESP32 DevKit / ESP-WROOM-32D) -----
static const int PIN_OT1_HEATER = 26;  // → opto CH1 (heater fire)
static const int PIN_OT2_FAN    = 27;  // → opto CH2 (fan fire)
static const int PIN_ZCD        = 25;  // ← SR800 ZCD sense (Q6B / orange path)

// MAX31855 SPI (VSPI-friendly)
static const int PIN_MAX_CS   = 5;
static const int PIN_MAX_SCK  = 18;
static const int PIN_MAX_MISO = 19;
// MOSI not required for MAX31855

// ----- Mains -----
// 1 = 60 Hz (US/Canada), 0 = 50 Hz
#ifndef MAINS_60HZ
#define MAINS_60HZ 1
#endif

#if MAINS_60HZ
static const uint32_t HALF_PERIOD_US = 8333;  // 1/(2*60)
#else
static const uint32_t HALF_PERIOD_US = 10000; // 1/(2*50)
#endif

// ----- Safety -----
// Heater forced off when fan duty < this (air-roaster element protection).
// Stage1/2 ignore this. Stage3+ enforce it. Tune after you find min fluidizing %.
#ifndef HTR_CUTOFF_FAN_VAL
#define HTR_CUTOFF_FAN_VAL 22
#endif

// Absolute BT heater cut (°F). OT1 forced off until BT cools below OVERTEMP_CLEAR_F.
#ifndef OVERTEMP_F
#define OVERTEMP_F 500
#endif
#ifndef OVERTEMP_CLEAR_F
#define OVERTEMP_CLEAR_F 480
#endif

// Triac gate pulse width (µs). Longer helps inductive fan loads.
#ifndef TRIAC_PULSE_US
#define TRIAC_PULSE_US 1200
#endif

// Serial (Artisan TC4-compatible)
#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

// Default temperature unit before Artisan sends UNIT;C / UNIT;F
// 1 = Fahrenheit, 0 = Celsius
#ifndef DEFAULT_UNIT_F
#define DEFAULT_UNIT_F 1
#endif

// Active level for OT outputs into PC817 LED input (HIGH = fire)
#ifndef OT_ACTIVE_HIGH
#define OT_ACTIVE_HIGH 1
#endif
