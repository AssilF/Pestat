# Firmware of the portable potentiostat

Runs on the ESP32 (NodeMCU-32S) of the instrument described in section 2.3 of the thesis: the analog front end of Anshori et al. (2022) (LM324, ±5 V in the published design, batteries on this board), with an MCP4725 12-bit I2C DAC supplied at 3.3 V in place of the ESP32's 8-bit DAC, and the ESP32's own ADC reading the transimpedance output (R6 = 5.1 kΩ, no divider). The supply facts were confirmed by the user on 2026-11-17. It does one thing: staircase cyclic voltammetry in **one locked measurement configuration**, calibrated against a multimeter, sent over USB serial in the `E_V,I_A` format of `data/potentiostat/PROTOCOL.md`. There is no Wi-Fi, Bluetooth or web page, and no scan parameter can be changed from outside.

It can also run **standalone**, with a 1.3″ OLED, a push button and an RGB LED: a button press measures a sample (two scans) and shows the verdict of the screening test's decision rule. That needs a rule exported from the phase-B data set; until then the device shows the peak signal and "no rule loaded".

| File | Content |
|---|---|
| `src/config.h` | the locked configuration (part 1), the circuit constants (part 2), display, button and LED (part 3) |
| `src/main.cpp` | DAC, ADC, calibration, scan, serial commands, standalone sequence |
| `src/ui.cpp`, `ui.h` | OLED, LED and button |
| `src/rule.cpp`, `rule.h` | the decision rule of `analysis/classify.py`, run on the device |
| `src/model.h` | the exported rule; for now a placeholder (no rule) |
| `host/record.py` | saves one scan to `data/potentiostat/scans/YYYYMMDD_NNN.csv` |
| `host/export_model.py` | writes `src/model.h` from the phase-B data set, after checking it against `classify.py` |
| `host/rule_check.cpp` | runs `rule.cpp` on the PC, for `export_model.py` |
| `platformio.ini` | build settings, toolchain and library pinned |

## Build and upload

Toolchain: **PlatformIO** (Core 6.1.19 used here), platform **espressif32 6.9.0**, which is the **Arduino-ESP32 2.0.17** framework on ESP-IDF 4.4, board `nodemcu-32s`. It is the toolchain of the earlier firmware of this instrument, now pinned to one version so that the same source always gives the same binary. One library: **U8g2 2.36.18** (OLED driver), fetched by PlatformIO on the first build. Build checked on 2026-11-17: no warnings with `-Wall`, 12.9 % of RAM, 27.2 % of flash (28.4 % with a rule loaded, checked with a rule exported from simulated data). Not yet run on the instrument.

From this folder:

```bash
pio run
```

```bash
pio run -t upload
```

```bash
pio device monitor
```

In the monitor, type `help`. The firmware echoes what is typed; Backspace works.

## Wiring

The first four lines are unchanged from the earlier firmware; the last three are for standalone use (part 3 of `config.h`).

| ESP32 (NodeMCU-32S) | Circuit |
|---|---|
| GPIO21 (SDA), GPIO22 (SCL) | MCP4725, address 0x60 (A0 to GND) |
| MCP4725 VOUT | node "in" (input of buffer U5, where the ESP32 DAC was) |
| GPIO32 (ADC1 channel 4) | node "vo" (output of buffer U4, after the D2–D1–R7 branch) |
| GND | common ground of the analog front end |
| GPIO21, GPIO22, 3.3 V, GND | OLED 1.3″ 128 × 64, I2C address 0x3C, on the same bus as the DAC |
| GPIO4 | push button to GND (internal pull-up) |
| GPIO25, 26, 27 | red, green, blue cathodes of the RGB LED, each through its resistor; common anode to 3.3 V |

- Power the OLED from **3.3 V**: many modules pull SDA and SCL up to their own supply, and the ESP32 pins do not tolerate 5 V. Most 1.3″ modules use the SH1106 controller; if the image is shifted or garbled, set `OLED_SSD1306` to 1.
- LED anode on **3.3 V, not 5 V**: with 5 V, a pin at 3.3 V (off) still leaves enough voltage to light the red, and pushes current back into the pin. Pick each resistor for a few mA (green and blue need a small one at 3.3 V), and balance red and green so that both together look yellow. `led yellow` shows the mix.
- GPIO32–39 (ADC1) stay free, e.g. for a later V₋ monitor. The pins avoid the ESP32's boot-strapping pins (0, 2, 5, 12, 15).
- At power-up the LED shows red, green, blue in turn: a wiring check.

## The locked configuration

Set in part 1 of `src/config.h`, named `PB-CV-1`. The firmware prints every value, with a CRC-32 of them, in the header of every file: two files with the same `config:` line were recorded in the same configuration.

| Item | Value |
|---|---|
| Program | 0 → +1.000 → 0 V against the reference electrode, anodic first, 1 cycle (PROTOCOL.md proposal) |
| Step | 7 DAC codes = **5.13 mV** (nominal; the calibrated value is in each header) |
| Step time | **100 ms** |
| Scan rate | **51.3 mV/s** (5.127 mV / 0.100 s) |
| Quiet time | 5 s at 0 V before the first step, not recorded |
| Averaging | mean of **80 ADC readings**, one every 250 µs, over the **last 20 ms** of each step |
| Points | 391 per scan (one per step), 39.1 s after the quiet time |

How the numbers follow from the circuit, with the MCP4725 supplied at 3.3 V:
- one DAC code is 3.3 V / 4096 = 0.806 mV at the DAC, hence 0.909 × 0.806 = 0.732 mV of electrode potential (eq. 2.2: E = 0.909 V_DAC − 1.5 V);
- 7 codes = 5.127 mV; 1.000 V needs 195 steps, so the vertex is 195 × 5.127 = 999.8 mV;
- 80 readings × 250 µs = 20 ms, one period of the 50 Hz mains, so mains interference averages out; the 80 ms before the window let the step settle (the published design needs 15 ms after its 53 Hz filter, Anshori et al. 2022, p. 4).

A step is a whole number of DAC codes, so every step of every scan has the same height and the same duration; the potential of each point is the calibrated potential of the code actually applied, not a target.

## How one point is measured (for section 2.3)

Draft sentences, valid once the instrument has been run in this configuration:

> The potential was applied as a staircase. The MCP4725 advanced by 7 codes (5.13 mV) every 100 ms, a scan rate of 51.3 mV/s, from 0 to +1.0 V and back, anodic first, one cycle, after 5 s at 0 V. The current of each step was the mean of 80 readings of the ESP32 converter taken over the last 20 ms of the step, one period of the 50 Hz mains, after 80 ms of settling; no other filtering was applied. The potential axis was calibrated against a multimeter at five points (linear fit), the converter against the same multimeter at up to eleven points (a table interpolated linearly), and the output at zero current was measured with the working electrode disconnected. The configuration and the calibration in force were written in the header of every file.

## Conversions

- Potential: E = a × code + b. Nominal a = 0.909 × V_DD / 4096 = 0.7324 mV per code; b = (R4/R3) V₋ = 0.3 V₋, which is −1.5 V at V₋ = −5 V (eq. 2.2). After `ecal`, a and b are the multimeter fit.
- Current (eq. 2.3, oxidation positive): I = (V_out − V_out(I=0)) / R6, with R6 = 5.1 kΩ. V_out comes from the mean ADC reading through the ADC table; V_out(I=0) = −V₋ R6/R5 (1.70 V at V₋ = −5 V) is measured by `zero`. The zero is stored as ADC counts, so a later ADC calibration applies to it too.
- Range: set by the ADC. Below about 0.1 V it reads 0 (Anshori et al. 2022, p. 4); at the top it saturates (4095 counts), at most at 3.3 V. By eq. 2.3 this is −0.31 to +0.31 mA at V₋ = −5 V. In practice the usable range is the range of the ADC table, which `adccal` prints in volts and in µA. One ADC count is about 0.8 mV, i.e. 0.16 µA; the mean of 80 readings resolves less.
- **Battery rails.** b and V_out(I=0) both follow V₋. At V₋ = −3.8 V, for example (the earlier firmware's note), b = −1.14 V, so the reachable window is −1.14 to +1.86 V (the firmware stops at +1.5 V), and V_out(I=0) = 1.29 V, so the current range becomes about −0.23 to +0.39 mA. The locked window, 0 to +1.0 V, is reachable at any V₋ between −3.0 and −5.5 V; the dummy window (down to −1.0 V) needs V₋ below −3.33 V. Both `ecal` and `zero` print the V₋ they imply, and every file header repeats it.

Every data line also carries `t_s`, `cycle`, `dac_code`, `adc_raw` (the mean in counts, before any conversion) and `flag`, so a file can be converted again if a calibration is later found wrong. `flag` adds: 1 = a reading of 0 counts (blind zone), 2 = a reading of 4095 counts (saturated), 4 = mean outside the calibrated ADC table (extrapolated), 8 = sampling started late. The footer counts the flagged points; a scan with flagged points in the region of interest should get `status = rejected` in the manifest, with the reason.

## Calibration against a multimeter

Three procedures, run from the serial monitor. Each is checked for plausibility, shown, and saved only on `y`. Saved values stay in the ESP32's flash across power cycles, get an increasing number (#1, #2, ...), and are printed in every file header with their number. `run` refuses to start until all three exist; `dummy` runs without them and says so in the header.

1. **ADC** (`adccal`) — once, and again after any change to the hardware. Connect a 4.7 kΩ resistor (3.9 kΩ reaches higher) between the WE lead and the joined RE and CE leads, so that the output sweeps most of the ADC range. Multimeter on DC volts, red probe on GPIO32, black on GND. The firmware sets 11 evenly spaced DAC codes within the reachable part of −1.5 to +1.5 V and, at each, shows the mean of 800 readings (10 windows); type the multimeter reading. Points where the ADC sits at 0 or 4095 are skipped. Accepted with at least 4 points rising together; the result is a table of mean counts against volts, interpolated linearly (the remedy of Anshori et al. 2022, p. 4: lookup table and averaging). The firmware also prints the chip's factory estimate next to each reading, which shows how large the correction is.
2. **Zero current** (`zero`) — at the start of each session. Disconnect the WE lead; the firmware averages 1 s of output (50 windows of 20 ms), which gives V_out(I=0), the V₋ it implies, and the noise, as the standard deviation of the 20 ms means, in µA. Rejected if the implied V₋ is outside −5.5 to −3.0 V.
3. **Potential** (`ecal`) — at the start of each session, because b follows the battery rail V₋. Connect the 10 kΩ dummy resistor (any 1–100 kΩ works). Multimeter on DC volts, red probe on the WE lead, black on the RE lead: the reading is then E itself. The firmware sets the codes of nominal −1.2, −0.6, 0, +0.6 and +1.2 V (shifted by the actual rail); type each reading. Least-squares line E = a × code + b, with the residual of each point and the V₋ implied by b. Rejected if a is more than 15 % from nominal, the implied V₋ outside −5.5 to −3.0 V, or a residual above 10 mV. On saving, the code for 0 V is also written to the MCP4725's EEPROM, so the cell sits at 0 V from power-up, before the ESP32 starts.

**Drift during a session.** The batteries sag, and with them b and V_out(I=0). At the end of each session run `zero` and `ecal` again and answer `n`: both print the difference from the saved calibration (zero in mV and µA; potential in mV at 0 V and at the vertex). Write the two differences in the lab notebook: they bound the drift of that session's scans. The two V₋ values implied by `zero` and `ecal` come through different resistor pairs, so with 5 % resistors they need not agree to better than a few tenths of a volt; it is their change during a session that matters.

Record the multimeter's make and model in the lab notebook; its accuracy bounds that of the calibration.

## Phase 0: dummy cell and comparison with the laboratory instrument

Ferri/ferrocyanide is not needed. PROTOCOL.md phase 0 checks the instrument in two ways:

- **Dummy cell.** A known resistor between the WE lead and the joined RE and CE leads, e.g. 10 kΩ (±0.1 mA at ±1 V). `dummy 10000` scans 0 → +1 → −1 → 0 V with the locked step, step time and averaging (781 points, 83 s), and the footer gives the straight-line fit I = E/R + I0 on the unflagged points: the measured R and its deviation from the nominal value, I0, r² and the rms residual. Five repeats, as in PROTOCOL.md. A deviation of a few percent is the tolerance of R6 and of the dummy resistor; measure the dummy resistor with the multimeter to separate them. This is the check of Anshori et al. (2022, p. 5–6) on resistors.
- **Same preparations on both instruments** (blank and propineb, three times each): the comparison section 3.3 reports. Use the laboratory instrument's scan rate if it can be matched (see the end).

## Recording scans

With the monitor closed (one program at a time on the port), from the project root:

```bash
python -I firmware/host/record.py run
```

```bash
python -I firmware/host/record.py run --name 20261120_007
```

```bash
python -I firmware/host/record.py dummy 10000
```

`--port COM5` if more than one serial port is present. The file goes to `data/potentiostat/scans/`, named `YYYYMMDD_NNN.csv`: by default the next number of the day not yet used by a file or by `manifest.csv`; give `--name` to use the name the run sheet assigned. A file is never overwritten. Ctrl+C stops the scan on the instrument and saves what was recorded, marked aborted (every scan is kept, PROTOCOL.md rule 3). Then fill in `status` and `notes` in the manifest.

A file looks like this (header shortened):

```
# saved: 2026-11-20T10:31:05 by firmware/host/record.py from COM5
# begin scan
# firmware: pbscreen-potentiostat 1.0.0, built ...
# mode: cv, locked configuration
# config: PB-CV-1, crc32 0x........
# config.step: 7 DAC codes every 100 ms
# scan.step_mV: 5.130 (7 codes x 0.7329 mV per code, calibrated)
# scan.rate_mV_s: 51.30
# cal.potential: #3, E = a*dac_code + b, a = 7.32900e-04 V/code, b = -1.50120 V; ...
# cal.adc: #2, multimeter table, 9 points ...
# cal.zero: #4, V_out(I=0) = 1.7034 V ...
E_V,I_A,t_s,cycle,dac_code,adc_raw,flag
0.0001,1.2345e-07,0.090,1,2048,2101.37,0
...
# end: points 391 of 391; flagged: blind 0, saturated 0, outside table 0, late 0; complete
# end scan
```

`analysis/classify.py` reads it as it is (`#` lines are comments, extra columns are ignored).

## Standalone use

The calibrations stay on the serial line (they need the multimeter and typed readings). Once they exist, the button runs a **sample**: two scans in the locked configuration, with a renewed surface in between, as each preparation of phase B (PROTOCOL.md). `analysis/classify.py` judges a preparation from all its scans, and the device does the same.

1. Idle screen: configuration, calibration numbers, rule. Press to begin; without the three calibrations the device refuses and says so.
2. "Scan 1 of 2: electrode in the sample". Press to start, hold 1.5 s to cancel.
3. The scan runs (about 45 s). Holding the button for half a second stops it; the sample is then abandoned.
4. "Scan 2 of 2: renew the surface, electrode back in". Press to start.
5. The result, on the screen and on the LED, until the next press.

| LED | Screen | Meaning |
|---|---|---|
| red | PROPINEB | screen positive: send the sample to a laboratory for confirmation |
| green | BLANK | screen negative |
| yellow | OTHER SUBSTANCE | something that is not classed as propineb |
| blue | INDETERMINATE | no answer (reason on the screen): retest or send to a laboratory |
| off | NO RULE LOADED | no rule on the device: the peak signal only, no verdict |

The colors follow the usual meaning of indicator lights (IEC 60073: red danger, yellow abnormal, blue action required, green normal). Propineb is red, not green, because on a residue test green reads as "safe"; and an indeterminate result is never green, because a missed positive is the error a screening test must avoid.

The screen also gives the mean class probabilities (`p b… p… o…`) and the peak height of each scan in µA. That peak height is a **signal, not a concentration**: the device does not quantify, and the class probability is not an amount either. A positive result only says the sample looks like the propineb samples of the training set, i.e. at or above the screening level the test was validated at.

One rule is added on the device, in the conservative direction only: a sample whose scans have flagged readings (ADC out of range or out of its table, sampling late) is indeterminate, as such scans would be rejected in the manifest.

During every calibration and every scan the LED is off and the display dark, also when the scan comes from the PC. The MCP4725 takes its reference from the 3.3 V rail, which also feeds the OLED (whose current depends on the pixels lit) and the LED, so the potential would otherwise move slightly with what is displayed; the display also shares the I2C bus with the DAC. Worth checking once on the bench: `zero` prints the noise; run it with the display on and off (e.g. unplugged). If they do not differ, `DISPLAY_DARK_DURING_SCANS = false` keeps a frozen "scanning" screen instead.

Without a display the sequence still works (press, scan, press, scan, LED), but the prompts are not visible. A standalone sample is not saved by the device: its scans and a `# sample:` line with the verdict go out on the serial line as usual, so a terminal that keeps a log keeps them. Data-set scans are recorded with `record.py`, as above.

## The decision rule on the device

`src/rule.cpp` repeats the arithmetic of `analysis/classify.py`: each scan read at the 49 grid potentials on the forward and return sweeps; standard scaling; 5 principal components; three-class logistic regression; the novelty limit (distance from the principal-component model above twice the 99th percentile of the training scans); then, over the scans of the sample, the mean probability, P_MIN = 0.70 and "any novel scan → indeterminate". The numbers come from `src/model.h`, which only the export script writes:

```bash
python -I firmware/host/export_model.py
```

It fits the rule exactly as `classify.py` does in each training fold, but once on the whole phase-B data set (the leave-one-day-out results of `classify.py` describe how this procedure performs; the exported rule is the procedure applied to all the data). It refuses unless every scan comes from `data/potentiostat`, from the `esp32` instrument, in one configuration (the same `# config:` line), and is not an invented curve; and it checks its fit against `classify.fit_predict`. If g++ is installed (it is, with Strawberry Perl, on this computer), it then compiles `rule.cpp` on the PC with the new model, runs it on every scan and every preparation, and writes `model.h` only if the probabilities, distances, peak heights and verdicts all match `classify.py`. Then rebuild and upload.

On the device, the rule is used only if it was trained in the configuration the firmware runs (CRC-32 of part 1 of `config.h`), and only if it passes its self-test: `model.h` embeds one scan of each class with the results `classify.py` gave them, and the firmware recomputes them at power-up (`selftest` shows the details). Otherwise the screen says "rule NOT USABLE" and no verdict is shown.

Tested on 2026-11-17 with the invented curves of `analysis/simulate_scans.py` (174 scans, 87 preparations): the device arithmetic matched scikit-learn to better than 10⁻⁷ in probability, about 10⁻⁷ (relative) in distance and 5 × 10⁻¹² A in peak height, and gave the same verdict for 87 of 87 preparations, on both the normal and the `--overlap` variant; the self-test passed. These figures concern the code only; invented curves say nothing about propineb.

## Serial commands

| Command | Does |
|---|---|
| `run` | staircase CV in the locked configuration (needs the three calibrations) |
| `dummy <ohm>` | dummy-cell scan with the straight-line fit, e.g. `dummy 10000` |
| `info` | configuration and calibration, as in a file header |
| `ecal`, `adccal`, `zero` | the three calibrations above |
| `set <V>` | hold the working electrode at a potential, e.g. `set 0.5` (limited to ±1.5 V) |
| `read [n]` | read the current n times at the present potential |
| `clearcal` | erase all calibrations (asks for `YES`) |
| `selftest` | check the decision rule of `model.h` on this chip |
| `led <color>` | `red`, `green`, `yellow`, `blue` or `off`: wiring check |

Any key, or the button, stops a running scan. The cell rests at 0 V after every scan and at power-up. A serial command that drives the cell abandons a standalone sample in progress.

Session routine: `zero`, `ecal`, one `dummy 10000`, then the scans of the day.

## Changing the configuration

Do not, during a data set. If a value of part 1 of `config.h` must change (before the data set, or for a new one), change `CONFIG_NAME` too, rebuild, and keep the scans of the two configurations apart. The CRC-32 in the header changes with any value of part 1, even if the name was forgotten.


