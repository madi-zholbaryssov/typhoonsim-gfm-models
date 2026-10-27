# TyphoonSim models: RLC circuit and grid-forming inverter (REGFM_A1)

Three small Typhoon HIL Schematic Editor models for learning TyphoonSim (the offline simulator) and for studying how a **grid-forming (GFM) inverter** behaves.

| File | What it is |
|---|---|
| `rlc_circuit.tse` | Series source resistance feeding an L ‖ (R + C) tank. A first model for learning the TyphoonSim settings. |
| `siib.tse` | **S**ingle **I**nverter on an **I**nfinite **B**us: one droop-controlled GFM inverter connected to a stiff grid. Frequency step, voltage step and fault scenarios. |
| `two_gfm.tse` | The infinite bus replaced by a **second** GFM inverter, with a constant-impedance load and a load step. Shows droop power sharing without a stiff grid. |

All models were saved with Typhoon HIL Control Center **2026.4** (development build) and are meant to run in **TyphoonSim**. No hardware is needed.

---

## How to run

1. Open the `.tse` file in the Schematic Editor.
2. Start the TyphoonSim simulation. All settings are stored in the model:
   - Solver **ODE**, integration method **BDF**, max step 1e-5 s, tolerances 1e-3, simulation time 5 s, data sampling rate **1e-4 s** (the sampling rate is in *seconds*).
3. Results appear in the model's scopes. A 5 s run of `siib.tse` or `two_gfm.tse` takes about 30–50 s on a laptop.

---

## rlc_circuit.tse

An ideal 350 V rms, 3 kHz sine source feeds, through `Rin` = 5 Ω, a parallel tank: `L` = 10 µH in parallel with `R` = 0.1 Ω in series with `C` = 500 µF. `I` is the source current and `Va1` the tank voltage; `Scope1` records both. It is the shipped Typhoon HIL example "rlc circuit typhoonsim", unchanged apart from its name.

The steady state can be solved with phasors (tank impedance 0.2121 Ω at 3.8°, giving **67.16 A rms** and **14.24 V rms**), so the model checks that TyphoonSim agrees with a hand calculation. Its settings (DAE solver, BDF, data sampling rate 0) are a good starting point for discussing what each TyphoonSim option does.

---

## The inverter used in `siib.tse` and `two_gfm.tse`

Both models use the same subsystem, called **GFM Inverter**: an **average-value** model of the REGFM_A1 generic droop-controlled grid-forming inverter, as specified by PNNL:

> W. Du et al., *Model Specification of Droop-Controlled, Grid-Forming Inverters (REGFM_A1)*, Pacific Northwest National Laboratory, PNNL-32278. https://www.osti.gov/servlets/purl/2229442

The controller parameters are the PNNL validation set (Table 1 of the report).

### What "grid-forming" means here

A grid-following inverter injects current and needs a PLL to lock onto a grid that someone else forms. A **grid-forming** inverter behaves like a **voltage source with its own frequency and angle**, in the same way a synchronous machine does. It has **no PLL**: it synchronises to the grid through its own power–frequency droop.

### Power stage

The power stage is deliberately simple: a three-phase *signal-controlled voltage source* (the internal EMF, magnitude E and angle δ) behind a coupling impedance `Rc + jXL` per phase, connected to terminals A, B, C. There is **no PWM, no switching ripple and no DC link**. That is the right level of detail for power-system control studies, and it makes a 5 s study run in under a minute.

| Quantity | Value |
|---|---|
| Rated power `Sbase` | 100 kVA |
| Rated voltage `Vbase_LL` | 480 V line-to-line (rms) |
| Rated frequency `f0` | 60 Hz |
| Rated current | 120.3 A rms |
| Base impedance | Zbase = Vbase²/Sbase = 2.304 Ω |
| Coupling reactance `XL_pu` | 0.15 pu = 0.3456 Ω (L ≈ 0.917 mH per phase) |
| Coupling resistance `Rc_pu` | 0.03 pu = 0.0691 Ω per phase |

Everything in the controller is in per unit on this rating.

### The idea behind droop control

For an EMF `E∠δ` connected through a mostly inductive reactance X to a bus of voltage `V∠0`, the power it delivers is approximately

```
P ≈ (E·V / X) · sin δ
Q ≈ (E·V·cos δ − V²) / X
```

For a small angle, **P follows the angle δ and Q follows the voltage difference E − V**. The angle is the integral of frequency, so an inverter changes P by temporarily changing its frequency, and Q by changing its voltage. The two droop laws build this into the controller (per unit):

```
P–f droop:   (f − f0)/f0 = mp · (Pref − P)        mp = 0.01  →  1 pu of power moves frequency by 1 % (0.6 Hz)
Q–V droop:   V = Vref − mq · (Q − Qref)           mq = 0.05  →  1 pu of reactive power moves voltage by 5 %
```

Each inverter acts only on what it measures locally, yet inverters sharing a network settle at one common frequency, and each takes a share of any load change proportional to **1/mp**. Two inverters with equal `mp` share equally, whatever their set points.

### Controller structure (`GFM Inverter › REGFM_A1_CTRL`)

The controller runs every `Ts` = 100 µs; measurements, angle, EMF waveforms and the current limiter run every `Ts_mod` = 20 µs. Each step has a comment block in the model. In signal-flow order:

1. **Measurements:** terminal voltages and output currents → amplitude-invariant Clarke transform (αβ, stationary frame, no PLL).
2. **Magnitudes and power:** V (line-to-line rms, pu) filtered by `TVf`; P and Q computed from the αβ quantities and filtered by `TPf`, `TQf` (all 10 ms).
3. **P–f droop:** ω = 2πf0·(1 + w1 + w2), with w1 = mp·(Pref − P). The term w2 comes from the **overload-mitigation** PI loops (`kppmax`, `kipmax`), which act only when P leaves [`Pmin`, `Pmax`].
4. **Q–V droop and voltage loop:** the voltage reference is `Vref − mq·(Q − Qref)` plus a correction from the PI loops (`kpqmax`, `kiqmax`) that act only when Q leaves [`Qmin`, `Qmax`]. With `VFlag = 1` a PI regulator (`kpv`, `kiv`, output limited to [`Emin`, `Emax`]) holds the terminal voltage at that reference; with `VFlag = 0` the reference is used directly.
5. **Internal EMF:** θ = ∫ω dt; the three EMF references are E·sin(θ), E·sin(θ + 240°), E·sin(θ + 120°) (positive sequence).
6. **Fault current limiter** (C function `Fig4_Lim`, PNNL Fig. 4 and Eq. 7): it predicts the current the droop EMF would drive through the coupling impedance,
   `|I| = |Edroop − V| / √(Rc² + XL²)`. If `|I| > ImaxF`, the EMF is reduced to `E = V + (ImaxF/|I|)·(Edroop − V)`, which holds the current at `ImaxF` and keeps its angle. The flag `lim_on` shows when it is active. The limiter uses this **predicted** current, not the measured one, so it releases as soon as the fault clears.

### Parameters (GFM Inverter mask)

| Parameter | Value | Meaning |
|---|---|---|
| `mp` | 0.01 | P–f droop gain |
| `mq` | 0.05 | Q–V droop gain |
| `Pmax` / `Pmin` | 0.9 / 0 | Active-power band for overload mitigation (pu) |
| `Qmax` / `Qmin` | 0.44 / −0.44 | Reactive-power band (pu) |
| `kppmax` / `kipmax` | 0.01 / 0.1 | Overload-mitigation PI gains |
| `kpqmax` / `kiqmax` | 3 / 20 | Q-limit PI gains |
| `kpv` / `kiv` | 0 / 5.86 | Voltage-loop PI gains |
| `Emax` / `Emin` | 1.15 / 0 | Voltage-loop output limits (pu) |
| `ImaxF` | 2.0 | Current limit during faults (pu) |
| `VFlag` | 1 | 1 = voltage PI loop enabled |
| `TPf` = `TQf` = `TVf` | 0.01 s | Measurement filter time constants |
| `Ts` / `Ts_mod` | 100 µs / 20 µs | Controller / fast execution rates |

### Inputs and outputs

- **Inputs:** `Pref`, `Qref`, `Vref` (pu), wired to constant blocks at the top level (`Pref_set`, `Qref_set`, `Vref_set`).
- **Terminals:** `A`, `B`, `C` (three-phase power connection).
- **Useful probes** (all inside `REGFM_A1_CTRL`): `Pf_pu`, `Qf_pu` (filtered P and Q), `V_pu` (terminal voltage), `I_pu` (current magnitude), `E_pu` (internal EMF magnitude), `f_drp` (internal frequency in Hz), `lim_on`, `Pmax_ctrl`/`Pmin_ctrl` and `Qmax_ctrl`/`Qmin_ctrl` (limit-loop outputs).

---

## siib.tse: single inverter on an infinite bus

### Topology

```
  Grid ──[ Rg + jXg ]──●──────────[ Rc + jXL ]──( E∠δ )   GFM Inverter
  480 V, 60 Hz         │
                       └── Fault: 3 × 1 mΩ to ground, closed 3.0–3.5 s (as an option)
```

The fault block sits in series between the grid and the inverter but has no impedance of its own, so closing it shorts the inverter's terminals. The inverter's own coupling impedance `Rc + jXL` is inside the `GFM Inverter` subsystem.

| Block | What it is |
|---|---|
| **Grid** | Infinite bus: three signal-controlled sinusoidal sources, 480 V line-to-line, 60 Hz, behind `Rg` = 2.877 mΩ and `Lg` = 0.1526 mH, i.e. zg = 0.00125 + j0.0250 pu on the inverter base. **Short-circuit ratio SCR = 1/\|zg\| = 40**, a very stiff grid. The C function `ScenDrv` inside it sets the grid voltage and frequency steps. |
| **Fault** | Three contactors to ground through 1 mΩ per phase, closed while `FaultTF_c` < t < `FaultTFend_c`. It puts a bolted three-phase-to-ground fault at the inverter's terminals. |
| **GFM Inverter** | The REGFM_A1 inverter described above. |
| **Pref_set, Qref_set, Vref_set** | Set points: 0.6 pu (60 kW), 0 pu, 1.0 pu. |
| **Scope1** | Inverter and grid quantities: V, E, P, Q, f, grid frequency, \|I\|, phase currents, phase-A voltage. |
| **Scope2** | Controller internals: filtered P/Q/V, w1, w2, limit-loop outputs, V_mod, θ, `lim_on`. |

The top-level canvas also carries a comment, **Cmt_PowerFlow**, with the power-flow equations behind droop control, and the controller subsystem has a comment block for each processing step.

### Selecting a scenario

Disturbances happen at **t = 3 s**. Change these settings, then run:

| Scenario | `Grid › ScenDrv` init code | `FaultTF_c` / `FaultTFend_c` |
|---|---|---|
| Frequency step, 60 → 59.85 Hz | `Vstep_pu = 1; Fstep_pu = 0.9975; Tstep = 3.0;` | 100 / 100.5 (fault off) |
| Frequency step, 60 → 59.7 Hz | `Fstep_pu = 0.995` | 100 / 100.5 |
| Voltage step, 1.0 → 0.98 pu | `Vstep_pu = 0.98; Fstep_pu = 1; Tstep = 3.0;` | 100 / 100.5 |
| Three-phase-to-ground fault, 3.0–3.5 s | `Vstep_pu = 1; Fstep_pu = 1;` | 3 / 3.5 |
| No disturbance (start-up only) | `Vstep_pu = 1; Fstep_pu = 1;` | 100 / 100.5 |

**The file as delivered** is set to the voltage-step scenario (`Vstep_pu = 0.98`, fault off). Setting the fault times to 100 / 100.5 moves the fault past the end of the 5 s run, which switches it off.

### What to expect

| Scenario | Prediction | Result in TyphoonSim |
|---|---|---|
| Start-up | The inverter pulls itself into synchronism through the droop, with no PLL | P reaches Pref = 0.6 pu within ±2 % after ≈ 0.17 s; f = 60 Hz |
| Frequency step to 59.85 Hz | P = Pref + (f0 − f_grid)/(mp·f0) = 0.6 + 0.15/0.6 | **P = 0.850 pu**; the inverter follows the grid at 59.85 Hz |
| Frequency step to 59.7 Hz | The droop asks for 1.1 pu, above Pmax | **P held at Pmax = 0.900 pu** by the overload loop |
| Voltage step to 0.98 pu | V = Vref − mq·Q, so the inverter injects Q | **Q ≈ 0.255 pu**, terminal V ≈ 0.987 pu (the grid fell 0.020 pu, the terminal only 0.013 pu); P stays 0.600 pu |
| Fault, 3.0–3.5 s | Voltage collapses; the limiter holds the current near ImaxF | V ≈ 0.02 pu, \|I\| held at **≈ 1.99 pu** (first-cycle peak ≈ 3.1 pu); the internal frequency rises to ≈ 60.35 Hz because P ≈ 0, so the angle runs ahead of the grid; after clearing P peaks at ≈ 2.05 pu, the frequency dips to ≈ 57.9 Hz, and P is back within 2 % of Pref by ≈ 3.8 s |

---

## two_gfm.tse: two inverters, no infinite bus

### Topology

```
  GFM Inverter2 (Pref2 = 0.3) ──┐
  GFM Inverter  (Pref  = 0.6) ──┼── common bus ──┬── Load1  90 kW, constant impedance
                                │                └── CB_load ── Load2  30 kW (closes at LoadStep_t = 3 s)
                                └── Fault (optional, 3 × 1 mΩ to ground)
```

The `Grid` subsystem of `siib.tse` is gone. **GFM Inverter2** is an identical REGFM_A1 inverter with its own set points (`Pref2_set` = 0.3, `Qref2_set` = 0, `Vref2_set` = 1). Both inverters, `Load1` and the `Fault` element share one bus. `Load1` is 90 kW and `Load2` is 30 kW at 480 V, unity power factor; the circuit breaker `CB_load` (a contactor driven by a clock and a comparator, `Clock_Load` and `LoadCmp`) connects `Load2` at `LoadStep_t` = 3 s. The `Fault` block is present but switched off (`FaultTF_c` = 100).

### What to expect

With no stiff grid, the droop lines alone decide the frequency. Both inverters run at one frequency, so `mp1·ΔP1 = mp2·ΔP2`: with equal `mp`, **each inverter takes half of any load change, whatever its Pref**, and the frequency settles *below* 60 Hz because nothing restores it (no secondary control).

| | P1 (Pref 0.6) | P2 (Pref 0.3) | Frequency |
|---|---|---|---|
| Before the step (t < 3 s) | 0.600 pu | 0.300 pu | 60.00 Hz |
| After +30 kW (0.3 pu) | **0.750 pu** | **0.450 pu** | **59.91 Hz** (= 60 − 0.6 · 0.15) |

Exercise: set `mp = 0.02` on the **GFM Inverter2** mask. Inverter 1 now takes two thirds of the step: ΔP1 = 0.20 pu, ΔP2 = 0.10 pu, f = 59.88 Hz. **The droop gain, not the rating or the set point, decides how a load change is shared.**

The model-initialisation code of `two_gfm.tse` prints the predicted post-step frequency (`f = 60·(1 + 0.01·(Pref − P))` with P = 0.75) to the console when the model is loaded.

---

## Notes and limitations

- **Average-value inverter.** No PWM or DC link, so there is no converter-level behaviour such as modulation saturation. After a fault clears, the internal EMF `E_pu` can briefly exceed `Emax` (it is not capped), so treat the first ~150 ms after clearing as indicative.
- **The current limiter predicts the current; it does not measure it.** It holds the AC fault current near `ImaxF`, but the coupling inductor also carries a **DC-offset current** at fault inception that the limiter cannot see. That is why the current briefly overshoots `ImaxF` in the first cycle (≈ 3 pu) and just after clearing (≈ 2.2 pu). The coupling resistance `Rc` is what damps that offset (time constant ≈ XL/(ω·Rc) ≈ 13 ms); with `Rc_pu = 0` the offset almost never decays and the current swings at 60 Hz.
- **Frequency does not return to nominal** after a disturbance in `two_gfm.tse`: droop provides *primary* control only.
- **Droop in this controller is stiff** (`mp` = 1 %): 1 pu of power moves the frequency by only 0.6 Hz. Typical synchronous-machine governor droop is larger.
- The grid in `siib.tse` is very stiff (SCR = 40). Weaker grids change both the droop response and the post-fault angle swing; try raising `Lg` in the `Grid` subsystem.

## Reference

W. Du, *et al.*, **Model Specification of Droop-Controlled, Grid-Forming Inverters (REGFM_A1)**, Pacific Northwest National Laboratory, PNNL-32278. https://www.osti.gov/servlets/purl/2229442
