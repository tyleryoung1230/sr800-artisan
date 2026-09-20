# SR800 × Artisan (ESP32)

DIY **Artisan** control for a stock **FreshRoast SR800**: bean temperature (K-type), heater (OT1), and fan (OT2) via an ESP32 speaking the TC4 serial protocol.

> **Repo name:** `sr800-artisan` (not “PID” — Artisan can do software PID later; this firmware is TC4-compatible heat/fan/BT control.)

This is **not** a drop-in commercial product. It involves **mains-adjacent wiring** on the SR800 control board. Read the safety section before powering anything.

**Working setup (validated):** single MAX31855 BT probe, NPN open-collector drive of the stock heat/fan transistors, polled ZCD from Q6B, Artisan 4.x as TC4.

---

## What’s in this repo

| Path | Purpose |
|------|---------|
| [`firmware/sr800/sr800.ino`](firmware/sr800/sr800.ino) | **Recommended** — single-file production sketch (pins inlined; no `config.h`) |
| [`firmware/stages/`](firmware/stages/) | Staged bring-up sketches (bench → ZCD → fan → full) |
| [`WIRING.md`](WIRING.md) | **Soldering guide** — find Q6B/Q7B/Q8B, NPN hookup, bring-up order |
| [`LICENSE`](LICENSE) | MIT + explicit DIY mains disclaimer |

---

## Safety (read this)

- Work on the SR800 **unplugged** when soldering taps.
- Use a **USB isolator** between PC and ESP (recommended strongly).
- Fit **10kΩ base→GND pull-downs** on both NPNs so a dead/hung ESP cannot leave the fan latched on.
- Firmware cuts heater if fan &lt; 22%, if ZCD dies (~1.5s), and if BT ≥ **500°F** (clears below 480°F).
- **Never run heat with the fan off** — even for a “quick” manual test. The element needs airflow. Always set **Air / OT2 first** (≥ ~22%), then Burner / OT1. If you’re bench-testing heater wiring, keep bursts tiny and fan already spinning; prefer validating heat only after fan `OT2;40` (or similar) is confirmed.
- Keep a hand on the **mains switch**. Coffee roasting is attended.
- You are responsible for fusing, enclosure, and verifying every tap on *your* board.

---

## Parts list (BOM)

Quantities for one roaster. **Amazon links below are the parts used in this build** (paste/update as needed if ASINs change).

### Required

| Part | Qty | Notes | Link |
|------|-----|--------|------|
| ESP32 DevKit / ESP-WROOM-32D (30-pin) | 1 | Not ESP32-C3 unless you change pins | *add your Amazon link* |
| MAX31855 K-type amp breakout | 1 | Pins labeled VIN/GND/DO/CS/CLK are fine | *add your Amazon link* |
| K-type thermocouple probe | 1 | Sheathed probe for chamber BT | *add your Amazon link* |
| NPN transistors (2N2222 / 2N3904 / similar) | 2 | Open-collector drive of SR800 yellow/brown | *add your Amazon link* |
| Resistors 1kΩ | 2 | ESP GPIO → NPN base | *add your Amazon link* |
| Resistors 10kΩ | 2 | NPN base → GND (failsafe pull-down) | *add your Amazon link* |
| Hookup wire / Duponts | — | Prefer short SPI + TC leads | *add your Amazon link* |
| USB cable for ESP32 | 1 | Data-capable | *add your Amazon link* |

### Strongly recommended

| Part | Qty | Notes | Link |
|------|-----|--------|------|
| USB isolator (ADUM3160 or similar) | 1 | Breaks ground loops / PC protection | *add your Amazon link* |
| Breadboard or protoboard | 1 | Then migrate to soldered enclosure | *add your Amazon link* |
| Probe compression fitting / HT grommet | 1 | Stops a wobbly chamber probe | *add your Amazon link* |
| Multimeter | 1 | Continuity / voltage checks | — |

### Optional

| Part | Notes | Link |
|------|--------|------|
| PC817 optocouplers | Original plan; **NPNs worked more reliably** into the SR800 brown/yellow lines | *add if purchased* |
| RobotDyn Zero-Cross / AC ZCD module | If stock Q6B ZCD is too noisy under heat | *add if purchased* |
| Second MAX31855 | Future **ET** (inlet or exhaust) | — |

### Consumables / tools

Soldering iron, flux, heat-shrink, zip-ties, drill + bit for probe hole, isopropyl alcohol.

---

## Architecture (how it works)

```
Artisan (TC4)  --USB-->  ESP32  --SPI-->  MAX31855  --K-type-->  chamber BT
                           |--GPIO26-+1k-+NPN--> yellow (heat, Q7B path)
                           |--GPIO27-+1k-+NPN--> brown  (fan,  Q8B path)
                           |--GPIO25 <--- ZCD sense at Q6B
```

- **OT1** = heater phase-angle  
- **OT2** = fan phase-angle  
- **ZCD** = zero-cross timing from the SR800 logic board (Q6B), **polled** (no interrupt — EMI-safe)  
- Stock PIC still present; we parallel the heat/fan transistor bases so either side can fire the triacs  

---

## Pin map (ESP32)

| Function | GPIO | Connects to |
|----------|------|-------------|
| OT1 heater | **26** | 1k → NPN base; collector → **yellow**; emitter → GND |
| OT2 fan | **27** | 1k → NPN base; collector → **brown**; emitter → GND |
| ZCD | **25** | SR800 **Q6B** sense (logic-level, not raw mains) |
| MAX31855 CS | **5** | CS |
| MAX31855 CLK | **18** | CLK / SCK |
| MAX31855 DO | **19** | DO / SO / MISO |
| MAX31855 VIN | **3V3** | 3.3V only |
| MAX31855 GND | **GND** | Common with ESP |

**K-type probe:** **+** (often yellow) → **T+**, **−** (often red) → **T−**. If BT falls when you warm the tip, swap T+/T−.

**SR800 board notes (typical):** see **[WIRING.md](WIRING.md)** for how to find pads with silk + meter.

| Signal | Where |
|--------|--------|
| GND | Logic board ground (confirm with meter — not a random power-board “GND”) |
| Fan control | **Brown** near **Q8B** |
| Heat control | **Yellow** near **Q7B** |
| ZCD | **Q6B** collector/pad (logic level — not red/blue, not mains) |

---

## Firmware install

### Arduino IDE

1. Install **ESP32** board support (Espressif).  
2. Board: **ESP32 Dev Module**.  
3. Library Manager: **Adafruit MAX31855** (+ BusIO).  
4. Copy `firmware/sr800/` to `Documents/Arduino/sr800/` so you have:
   ```
   Documents/Arduino/sr800/sr800.ino
   ```
   **Do not** keep a broken leftover `config.h` in that folder (paste errors caused `a#pragma once` before).  
5. Select the ESP COM port, upload at the usual settings.  
6. Serial Monitor **115200** — you should see `# stage4 READY...`.

### Staged bring-up (first-time hardware)

Use `firmware/stages/` if you’re wiring from scratch:

| Stage | Sketch | Goal |
|-------|--------|------|
| 1 | `stage1_bench_temp` | BT + Artisan `READ` only |
| 2 | `stage2_zcd` | Confirm ~120 edges/s on ZCD |
| 3 | `stage3_fan` | Fan PAC, heater disconnected |
| 4 | `stage4_full` | Heat + fan + safety |

After bring-up, switch to the single-file `firmware/sr800/sr800.ino` for daily use.

---

## Wiring checklist (summary)

Full step-by-step: **[WIRING.md](WIRING.md)** (finding Q6B, soldering NPNs, order of operations).

1. Bench: ESP + MAX31855 only → finger raises BT.  
2. Common **GND** ESP ↔ **logic** GND (metered).  
3. **Q6B → GPIO25** → prove ~120 Hz-class ZCD before anything else.  
4. Fan NPN → **brown / Q8B**: `OT2;40` / `OT2;0` works.  
5. Heat NPN → **yellow / Q7B**: test **only with fan already on** (≥ ~22%).  
6. USB isolator; mount/seal chamber probe.

**Never** solder/test heat with the fan path unverified.

---

## Artisan setup

1. Close Serial Monitor (port must be free).  
2. **Config → Device**
   - Device: **TC4**
   - **Control** checked  
   - **PID Firmware** unchecked  
   - Port: ESP COM @ **115200**  
3. For a **single probe**, disable ET in the device mapping; set **BT** to thermocouple **CH1 or CH2** (firmware mirrors both).  
4. **Config → Events → Sliders**
   - **Air** (fan): `OT2,{}`  
   - **Burner** (heat): `OT1,{}`  
5. Hide ET curves if you like (`Config → Curves → UI`) so Designer isn’t cluttered.  
6. Hit **ON** so sampling runs, then use the Air/Burner sliders.

### Serial protocol (TC4 subset)

| Command | Meaning |
|---------|---------|
| `READ` | `ambient,T1,T2,T3,T4,heat,fan` |
| `OT1;n` / `OT1,n` | Heater 0–100 |
| `OT2;n` / `OT2,n` | Fan 0–100 |
| `UNIT;F` / `UNIT;C` | Temperature unit |
| `STAT` | Debug: ZCD edges, faults, overtemp latch |
| `RESET` | Outputs off, clear overtemp latch |

---

## Firmware behaviors worth knowing

| Behavior | Detail |
|----------|--------|
| Fan min | Duties 1–4% snap to 0; usable from **5%** (may cog below ~20%) |
| Heater cutoff | OT1 forced 0 while fan &lt; **22%** |
| Overtemp | OT1 cut at BT ≥ **500°F**; clears ≤ **480°F** |
| BT filtering | Soft-SPI MAX31855, median-of-3, hold last-good on faults |
| ZCD | Polled + filtered; under heat uses fixed 60 Hz timing so fan doesn’t surge |
| Debug spam | Off by default (`DEBUG_SERIAL 0`) so Artisan stays clean |

---

## Probe mounting

If the drilled hole is oversized / wobbly:

1. Prefer a **compression fitting** or **high-temp silicone grommet** sized to the sheath.  
2. Tip in the **bean fountain**, not jammed on the wall.  
3. Seal leaks — extra air changes the roast.  
4. Strain-relieve the cable so tugs don’t lever the probe.

---

## Troubleshooting

| Symptom | Likely fix |
|---------|------------|
| Artisan port garbage / `` | ESP bootloader @ 74880 baud on connect — ignore; wait for READY |
| Sliders do nothing | Sampling **ON**; Control checked; Serial Monitor closed; Air=`OT2`, Burner=`OT1` |
| Fan stuck on after crash | Add 10k base pull-downs; power-cycle; reflash polled-ZCD sketch |
| Fan surges when heat rises | Reflash latest sketch (fixed half-period under heat); consider external ZCD |
| BT spikes to 0 / floor | Reseat T+/T− and SPI; short wires; latest filter firmware |
| Compile: `a#pragma once` | Delete leftover corrupted `config.h`; use single-file `sr800.ino` only |
| `undefined reference to setup()` | Wrong/empty `.ino`; folder must be `sr800/sr800.ino` |

---

## ET vs BT (later)

On a fluid bed, chamber K-type is a mixed air/bean **BT** — that’s normal.

Long-term: add a second MAX31855 as **ET** (inlet or exhaust). Don’t scrape the SR800 LED digits; if you want “stock display temp,” you’d tap the stock NTC (harder) rather than the LED.

Until then: **BT only** in Artisan is the right call.

---

## Roadmap / nice-to-haves

- [ ] Second TC for real ET  
- [ ] Soldered enclosure + labeled connectors  
- [ ] External ZCD module if Q6B remains noisy at high heat  
- [ ] Optional Artisan PID once manual profiles feel repeatable  

---

## Credits & sources

This project stands on a lot of prior work. Thank you to:

### Community reverse-engineering (SR800 / TC4)

- **[Homeroasters Association](https://homeroasters.org/)** — community reverse-engineering of FreshRoast SR540/SR800 + TC4 control, especially:
  - [SR800/SR540 Fan Current TC4+](https://homeroasters.org/forum/viewthread.php?thread_id=6804)
  - [My SR540 with TC4+ Artisan Control](https://homeroasters.org/forum/viewthread.php?thread_id=6556)
  - [SR800 control monitoring?](https://homeroasters.org/forum/viewthread.php?thread_id=7104)
- **renatoa** — TC4/PAC guidance, heater-cutoff philosophy, RobotDyn ZCD suggestions, and related TC4ESP ideas on Homeroasters.
- **dwertz** (and others on those threads) — SR800 board observations, parallel drive of stock heat/fan transistors, isolation/ZCD practical notes.

### Software & protocol

- **[Artisan](https://artisan-scope.org/)** — roast logging/control UI ([GitHub](https://github.com/artisan-roaster-scope/artisan)).
- **TC4 / aArtisan** ecosystem — serial protocol (`READ`, `OT1`, `OT2`, `UNIT`, …) that this ESP32 sketch speaks so Artisan can treat it like a TC4 device. Original TC4 work by Jim Gallt and contributors; later ESP ports including community forks discussed on Homeroasters.

### Hardware references

- **MAX31855** K-type amp — [Adafruit MAX31855](https://www.adafruit.com/product/269) / Maxim datasheet; Arduino driver via **Adafruit MAX31855** library.
- **Espressif ESP32** Arduino core.
- **RobotDyn** AC dimmer / zero-cross modules — optional external ZCD path suggested in Homeroasters discussions when stock Q6B is too noisy.

### Related products (inspiration, not copied)

- Commercial/community SR800 computer-control projects (e.g. RoastLink-style Notion/product pages) helped scope “Artisan + stock SR800” as a goal; this repo’s firmware and NPN/Q6B approach were built and validated independently for this hardware.

If we missed a credit you deserve, open an issue or PR.

---

## Disclaimer

DIY mains / triac-adjacent control. Verify every connection on your SR800. No warranty — see [`LICENSE`](LICENSE).
