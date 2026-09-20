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
| Fan command | **Brown** | near **Q8B** | GPIO **27** via NPN |
| Heat command | **Yellow** | near **Q7B** | GPIO **26** via NPN |
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

1. Locate transistor **Q8B** → the control node / harness wire that leaves this area is typically **brown** = **fan**.  
2. Locate transistor **Q7B** → typically **yellow** = **heater**.  
3. These are the nodes the stock PIC already pulls to fire the fan/heat path. You will solder the **NPN collectors** here (in parallel with the stock drive).

### Confirm with a meter (recommended)

With the unit still unplugged:

1. Continuity from the brown wire pad to the Q8B collector/circuit node you intend to tap.  
2. Same for yellow ↔ Q7B.  
3. Neither brown nor yellow should be shorted to GND at rest.

Optional live check (careful, isolated, lids as closed as practical): stock front-panel fan/heat changes should move activity on those lines. Prefer unpowered continuity ID when learning the board.

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

Use **two** small NPNs (2N2222 / 2N3904 / similar).

```
ESP GPIO27 (fan) ----[1k]---- NPN base
                              |
                         [10k to GND]
                              |
ESP/SR800 GND --------------- emitter
                              collector ----> SR800 BROWN (Q8B node)

ESP GPIO26 (heat) ---[1k]---- NPN base
                              |
                         [10k to GND]
                              |
ESP/SR800 GND --------------- emitter
                              collector ----> SR800 YELLOW (Q7B node)
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

| Step | Action | Pass criteria |
|------|--------|----------------|
| A | Bench: ESP + MAX31855 only, flash sketch | Finger raises BT in Serial/`READ` |
| B | Solder **GND** ESP ↔ logic GND | Continuity OK |
| C | Solder **Q6B → GPIO25** | stage2 / `STAT`: ~120 Hz class edges |
| D | Build **fan NPN**, collector → **brown** | `OT2;40` spins fan; `OT2;0` stops |
| E | Build **heat NPN**, collector → **yellow** | **Only with fan already ≥ ~22%** — short heat bursts; BT rises |
| F | USB isolator in series with PC | Artisan talks stably |
| G | Probe mounted & sealed | Roast-ready BT |

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
