# Wiring & soldering guide (SR800 logic board)

**Unplug the SR800 from the wall** before opening the case or soldering.  
Use a USB isolator between the PC and ESP once grounds are shared.

This guide matches the **validated** build: NPN open-collector into the stock heat/fan transistor nodes, ZCD from **Q6B**, MAX31855 for BT.

---

## 1. What you’re tapping (big picture)

The stock PIC still runs the front panel. You **parallel** two control nodes so the ESP can also fire the same triac drivers:

| Signal | Wire / pad (typical) | Transistor silk | ESP |
|--------|----------------------|-----------------|-----|
| Logic GND | Black / GND pour | — | ESP **GND** |
| Fan command | **Brown wire between boards** | near **Q8B** | GPIO **27** via NPN |
| Heat command | **Yellow wire between boards** | near **Q7B** | GPIO **26** via NPN |
| Zero-cross | **Q6B** collector/pad | **Q6B** | GPIO **25** (direct sense) |

You are **not** soldering to mains hot/neutral for control. Stay on the **logic / low-voltage** control board unless you deliberately add an isolated AC ZCD module later.

> **Do not** grab random “GND” labels on the power board without metering — some SR discussions note markings that are **line neutral**, not safe logic ground. Always confirm continuity to the **logic board ground** you share with the ESP.

---

## 2. Open the roaster & find the logic board

1. Unplug. Remove the outer shell per usual SR800 teardown (screws under feet / rear — take photos as you go).  
2. Find the small **logic / CPU board** with the **PIC16F690** (or similar) and a row of discrete transistors.  
3. Look for silk labels **Q6B**, **Q7B**, **Q8B** (letter suffix may vary slightly by revision — match the digit + nearby wire colors).  
4. Note the ribbon / harness toward the power board: colors commonly include **black, brown, yellow**, plus others (red/blue are **not** your ZCD tap for this build).

---

## 3. Identify GND (do this first)

1. Put the meter on **continuity / ohms**.  
2. Find a solid **logic GND**: black wire in the harness and/or ground pour tied to the PIC Vss area.  
3. Confirm the point you will use for ESP GND beeps to that ground.  
4. Solder a flying lead here later labeled **GND → ESP**.

If you’re unsure, do **not** guess from the power-board earth tab alone.

---

## 4. Find fan (brown / Q8B) and heat (yellow / Q7B)

### By silk + wire color

1. Locate transistor **Q8B** → the **brown wire between the logic and power boards** near this area is typically **fan**.  
2. Locate transistor **Q7B** → the **yellow wire between boards** near this area is typically **heater**.  
3. These are the nodes the stock PIC already pulls to fire the fan/heat path. You will solder the **NPN collectors** here (in parallel with the stock drive).

### Confirm with a meter (recommended)

With the unit still unplugged:

1. Continuity from the brown wire pad to the Q8B collector/circuit node you intend to tap.  
2. Same for yellow ↔ Q7B.  
3. Neither brown nor yellow should be shorted to GND at rest.

---

## 5. Find ZCD at Q6B (this is the finicky one)

### What you want

A **logic-level** zero-cross-ish square/pulse that the ESP can read on GPIO25 with `INPUT_PULLUP`.  
On this build we use the **Q6B** transistor pad on the **logic board** — not raw AC, and not the red/blue low-voltage sense pair as a substitute without proving it.

### How to find the right pad

1. Find silk **Q6B** on the logic board (same family as Q7B/Q8B).  
2. Identify the **collector** (or the node that already feeds the PIC’s zero-cross sensing path). That is your tap.  
3. Solder a thin flying lead there → **GPIO25**. Share **GND** with the ESP.

### What Q6B is *not*

| Tempting point | Why skip for this firmware |
|----------------|----------------------------|
| Mains hot / neutral | Lethal; needs proper isolated ZCD module if ever used |
| Red / blue harness wires (~0.9 V / ~1.8 V on some boards) | Not the proven Q6B ZCD path for this sketch |
| Random via next to the PIC | Easy to brick sensing; stick to Q6B |

### Prove ZCD before driving heat

1. Flash `firmware/stages/stage2_zcd` (or production sketch and watch `STAT`).  
2. Power the SR800 (fan can stay low).  
3. Serial should show roughly **~100–140 edges/s** on 60 Hz with CHANGE-style sensing (both edges ≈ 120 Hz).  
4. If you see ~60 Hz only, you’re on one edge — still usable but note it.  
5. If you see 0, noise, or hundreds of kHz of chatter: wrong pad, bad GND, or need more filtering / external ZCD.

---

## 6. Build the two NPN drivers (fan + heat)

Use **two** small NPNs (2N2222 / 2N3904 / similar). Flat face toward you, pins left→right are usually **E B C** (confirm your part’s datasheet — TO-92 pinouts vary).

### Breadboard layout (one channel)

Build **fan** first (GPIO27 → brown). Copy the same pattern for **heat** (GPIO26 → yellow) on the next few rows.

```
ESP32                         breadboard row                   SR800 logic board
─────                         ──────────────                   ─────────────────

GPIO27 ── jumper ──►  [1k] ──► NPN base (middle pin)
                                 │
                              [10k] ──► GND rail
                                 │
                              emitter ──► GND rail  ◄── jumper ── ESP GND
                                                          and SR800 logic GND
                                 │
                              collector ── flying lead ──► brown wire between
                                                           boards (Q8B node)

GPIO26 ── same pattern ──► second NPN ──► yellow wire between boards (Q7B)
```

**Parts per channel:** 1× NPN, 1× 1k (GPIO→base), 1× 10k (base→GND), emitter on the shared GND rail.

```
        GPIO27                    GPIO26
          │                         │
         1k                        1k
          │                         │
          ├─10k─┐                   ├─10k─┐
          │     │                   │     │
         B│     │                  B│     │
       E─NPN─C  │                E─NPN─C  │
       │     │  │                │     │  │
      GND   brown              GND   yellow
            (fan/Q8B)                (heat/Q7B)
              ▲                        ▲
              └──── both emitters + both 10k bottoms share ESP/SR800 GND
```

### Soldering tips

1. Keep leads **short**. Star-ground at the ESP GND.  
2. **10k base→GND** is mandatory failsafe — if the ESP hangs or browns out, bases sit low and collectors float open (fan/heat not held on by a stuck GPIO).  
3. Solder collector wires to the brown/yellow **pads** with mechanical strain relief (hot-glue *after* electrical test, not as the electrical joint).  
4. Do **not** remove the stock transistors — you are paralleling control, not replacing the board.

### Why NPNs (not PC817) in this build

PC817 LED-side drive can look “alive” on a meter while still being too weak into the brown pull-up path. Open-collector NPNs to GND on the brown/yellow nodes worked reliably here. Optos remain optional isolation if you redesign later.

---

## 7. MAX31855 + thermocouple

| MAX31855 label | ESP32 |
|----------------|-------|
| VIN | **3V3** (not 5V unless your module is 5V-safe and level-shifted) |
| GND | **GND** |
| CLK | **GPIO 18** |
| DO | **GPIO 19** |
| CS | **GPIO 5** |

| K-type | Amp |
|--------|-----|
| + (often yellow / non-magnetic) | **T+** |
| − (often red / magnetic) | **T−** |

If BT **falls** when you warm the tip, swap T+/T−.

Mount the probe in the chamber (compression fitting / HT grommet). Seal air leaks.

---

## 8. Assembly order (do not skip)

Use the **staged firmware** under `firmware/stages/` so each physical connection is proven before the next. Prefer **temporary contact** (hold a Dupont/probe tip on the pad while watching Serial) before you solder that joint permanently.

| Step | Firmware | What you touch | Pass criteria |
|------|----------|----------------|---------------|
| A | `stage1_bench_temp` | ESP + MAX31855 only (no SR800) | Finger on probe tip raises BT in Serial / Artisan `READ` |
| B | (any) | Hold ESP **GND** ↔ logic **GND** | Continuity beep; then solder GND |
| C | `stage2_zcd` | Hold flying lead from **GPIO25** on **Q6B** pad | Serial `zcd_hz` ~100–140 (both edges) or steady ~60 (one edge). Wrong pad → 0 / junk / huge noise |
| D | `stage3_fan` | Breadboard fan NPN; hold collector on **brown** (Q8B) | `OT2;40` spins fan; `OT2;0` stops. Solder brown only after this works |
| E | `stage4_full` | Hold heat NPN collector on **yellow** (Q7B) | Fan already ≥ ~22%; short `OT1` bursts; BT rises. Then solder yellow |
| F | `sr800.ino` (or stage4) | USB isolator; probe mounted | Artisan Air/Burner sliders work |

### How to “hold the wire” safely

1. Flash the stage for the check you’re doing. Open Serial Monitor **115200**, Newline.  
2. Keep one hand free; SR800 plugged in only when that stage needs live ZCD/fan/heat.  
3. Use a **female Dupont on a short solid tip** or meter probe — touch the pad, don’t jab vias.  
4. For ZCD/fan/heat: ESP **GND must already be common** with logic GND (step B), or readings are nonsense and you can stress the ESP.  
5. When Serial shows a clean pass for several seconds, **then** solder that joint and strain-relieve it.  
6. Never hold heat (yellow) until fan (brown) is proven. Firmware also blocks heat below ~22% fan, but don’t rely on that alone during bring-up.

### Stage cheat sheet

| Stage | Folder | Drives OT1/OT2 GPIOs? | Purpose |
|-------|--------|------------------------|---------|
| 1 | `stage1_bench_temp` | **No** (commands accepted, pins idle) | Desk-safe BT + Artisan protocol |
| 2 | `stage2_zcd` | No outputs | Prove Q6B contact / edge rate only |
| 3 | `stage3_fan` | Fan (**GPIO27**) only | PAC fan; leave heat NPN disconnected |
| 4 | `stage4_full` | Fan + heat | Full control + cutoffs; daily roast path |
| — | `firmware/sr800/sr800.ino` | Fan + heat | Same as stage4, single file for daily use |

### Critical test rule

**Fan first, then heat. Never energize OT1 with OT2 at 0** during bring-up or “just checking the yellow wire.”

---

## 9. Quick continuity checklist (unpowered)

- [ ] ESP GND ↔ logic GND  
- [ ] GPIO27 path: ESP → 1k → NPN base; 10k base–GND; emitter–GND; collector–brown  
- [ ] GPIO26 path: same pattern to yellow  
- [ ] GPIO25 → Q6B only (not shorted to GND)  
- [ ] MAX31855 VIN/GND/CLK/DO/CS correct  
- [ ] No solder bridges on PIC pins  

---

## 10. If something’s wrong

| Symptom | Check |
|---------|--------|
| No ZCD edges | Wrong pad (not Q6B); GND not common; wire loose |
| Fan never moves | Brown vs yellow swapped; NPN orientation; missing GND; still using weak opto-only drive |
| Fan stuck on | Missing 10k pull-downs; GPIO stuck high; reboot ESP |
| Heat but no airflow | **Stop immediately** — fan path not working |
| BT at 0 / wild | T+/T− swapped or loose; SPI Duponts too long |

---

## Photo note

Board silk and harness colors can vary by SR800 revision. Trust **Q6B / Q7B / Q8B silk + meter confirmation** over color alone if your loom differs.
