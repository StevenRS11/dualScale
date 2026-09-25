# dualScale modernization plan

Bring dualScale up to the maturity of `../pushbuttonDEF` in three areas: the
settings menu, NVS calibration storage, and the PC/agent-assisted commissioning
cycle. This plan covers only what applies to a two-channel static scale. Nothing
related to motion, the dial indicator, the frame curve, or USAP/UPAA is ported.

## Gap summary

| Area | pushbuttonDEF (reference) | dualScale today |
|---|---|---|
| Build | PlatformIO, pinned libs, vendored `lib/PaddleDNA` + `lib/MFRC522_I2C`, per-role envs | Arduino IDE / arduino-cli, libraries from the sketchbook |
| Menu entry | Hold START for 10 s with an OLED countdown; releasing cancels | Button held at the instant of boot drops straight into calibration |
| Menu | Scrolling list: tap moves, hold selects. Submenus for cal, toggles, offset, backup/restore, status, exit | None. One linear calibration that always redoes **both** channels |
| NVS layout | Named `KEY_*` constants in one namespace. Saves return checked sizes. Blob records use magic/version/size/CRC-32 plus layout `static_assert`s | Loose keys (`cal1/cal2/tare1/tare2/crc`), CRC over a bare struct with no magic or version |
| Backup | **Operator-controlled** snapshot saved as copies A/B in `*_backup`, write-then-readback checked. Restore is all-or-nothing, and version migration is pinned by `static_assert`s (`CalibrationBackupPolicy.h`) | Every save is auto-mirrored to `dualScaleBak`, so a bad cal overwrites the good backup |
| Persisted settings | NFC enable, test mode, manual offset, frame-comp toggle | None |
| Device identity/status | `QUAL INFO` / `FIELD INFO` JSON: efuse machine ID, sketch MD5, version, cal state, NFC state | Human-readable boot log only. `MACHINE_UUID` is hard-coded |
| Result telemetry | `FIELD RESULT {json}` after every button test | `printf` of head/handle grams |
| Host tooling | `setup_workstation.py`, `machine.py` (ports/status/capture/field-check), commissioning runner with resume, sealed SHA-256 manifests, per-run Git commits, fleet CSV, unit tests | None |
| Agent operation | `AGENT_RUNBOOK.md`, `WORKSTATION.md`, `COMMISSIONING.md`, machine profiles | CLAUDE.md only |
| Testable math | `DeflectionAnalysis.*` is hardware-free, so host and device results can be cross-checked | Regression and BP/ESW are inline in the `.ino` |
| USB serial | `CharacterizationSerial.h` paces HWCDC writes and drops output safely while detached | Blocks up to 2.3 s waiting for the host at boot |

## Phase 0: Build system (no behavior change)

1. Add `platformio.ini` with `src_dir = .` and the S3-Zero board block copied from
   pushbuttonDEF (qio/qspi PSRAM, `ARDUINO_USB_CDC_ON_BOOT=1`). Pin
   `Adafruit SSD1306@2.5.17`, `GFX@1.12.6`, `BusIO@1.17.4` and
   `SparkFun NAU7802@1.0.6`. Confirm that 1.0.6 exposes the
   `clearBit(NAU7802_PGA_PWR_PGA_CAP_EN, …)` call we rely on for the CH2 fix.
2. Vendor `lib/PaddleDNA` and `lib/MFRC522_I2C` by copying from pushbuttonDEF's
   `lib/` so both machines build against identical NFC/payload source, and add
   `lib/README.md`.
3. Add `.gitignore` (`.pio/`, `.venv/`, `__pycache__/`) and `.gitattributes` with
   `-text` on `qualification/results/**` and `qualification/handoffs/**`, then
   delete the stray `tmpclaude-*` file.
4. Envs: `commissioning` (serial cal commands enabled) and `field` (read-only
   status). Both use the same partition map so an upload preserves NVS.
5. Add a `SOFTWARE_VERSION` string, and show it on the splash and in status output.
6. **[DONE 2026-09-25; compiles, awaiting hardware check] Fix the I2C topology to match the current boards.** The OLED (0x3C) and the
   NFC reader (0x28) share `Wire` on SDA=8/SCL=7. The NAU7802 has `Wire1`
   (SDA=3/SCL=4) to itself. Today's sketch assumes the older NFC-on-10/11 board, so
   on current boards the NFC path is talking to the wrong pins. The fix:
   - Delete `selectBus()`, `BusOwner`/`busOwner` and every `selectBus(BUS_*)` call site.
   - Delete `RFID_SDA`/`RFID_SCL`.
   - Call `nfc.begin(Wire)`.
   - `Wire1.begin(3, 4)` once in setup.
   - In `scanI2CModules()`, probe the OLED and NFC on `Wire` and the NAU on `Wire1`.
   - Move `Wire.setClock(400000)` to after `display.begin()` (pushbuttonDEF note).
   - Update the I2C/bus sections of CLAUDE.md.

   This matches pushbuttonDEF's layout (OLED + NFC on `Wire`, NAU on `Wire1`).

**Exit:** `pio run -e field` builds. On hardware, the I2C report shows all three
modules and an NFC write succeeds. Otherwise it behaves like today's sketch.

## Phase 1: Structure

- Extract **`ScaleMath.h/.cpp`**, hardware-free like `DeflectionAnalysis`: the
  least-squares fit (returning slope, intercept and residuals), trimmed mean,
  grams conversion, BP and ESW. Remove the implicit use of the `lastVal1/lastVal2`
  globals from `calculate_BP()`. The host runner then reproduces device results
  exactly.
- Port `CharacterizationSerial.h` and drop the blocking boot wait. The boot I2C
  report stays, and `INFO` reprints it on demand.
- Port the UI primitives `ui_drawHeader` (status pill), `ui_drawFooter`,
  `ui_centerX`, `ui_screenError` and the splash screen with version.
- Optional: onboard WS2812 status LED using the boot-stage colors. Use
  `NEO_RGB`, **not** `NEO_GRB`, per the pushbuttonDEF note.

## Phase 2: NVS v2

**Namespace `"dscale"` (live keys)**

| Key | Type | Contents |
|---|---|---|
| `calrec` | blob | `ScaleCalRecord` |
| `nfcen` | bool | NFC enabled (default true) |
| `units` | u8 | display units (optional) |
| `bootcnt` | u32 | keep the rapid-boot guard |

`ScaleCalRecord`, one blob so the two channels can never be restored separately:

```c++
struct ScaleCalChannel { float countsPerGram; int32_t zeroRaw; float maxResidualG; uint8_t nPoints; uint8_t pad[3]; };
struct ScaleCalRecord {
  uint32_t magic;          // "DSCR"
  uint16_t version;        // 1
  uint16_t size;
  uint8_t  recordId[8];    // commissioning record hash prefix, zero if calibrated from the menu
  ScaleCalChannel head, handle;
  uint8_t  source;         // 0 = menu, 1 = host-installed
  uint8_t  reserved[3];
  uint32_t crc32;
};
static_assert(...layout...);   // host packs identical bytes and must reproduce the CRC
```

- Save = `putBytes` → read back → `memcmp`. Live values are replaced only on
  success, and a failed write shows on the OLED (pattern: `saveFrameCurve`).
- **Backup namespace `"dscale_bak"`, keys `copy_a` / `copy_b`.** The backup is
  written **only** from the menu's "Save backup" or the host's `BACKUP SAVE`, and
  restore is all-or-nothing. This replaces the auto-mirror. Add
  `ScaleCalPolicy.h` with `static_assert`s for any future version migration.
- **Legacy import (one time).** If `calrec` is absent and the old
  `dualScale`/`dualScaleBak` keys pass the existing CRC and range check, convert
  them to a v1 record with `source=menu` and `recordId=0`, then log it. Do not
  delete the old keys in this release.
- Runtime tare stays RAM-only. The stored `zeroRaw` is the regression intercept,
  as it is today.

## Phase 3: Settings menu

- Enter by holding the button continuously for 10 s during boot, with the
  countdown screen and "Release to cancel". This replaces the instant
  boot-hold-to-calibrate.
- The menu loop uses the same scrolling list and interaction as pushbuttonDEF:
  a tap moves the highlight, a 1.2 s hold selects, the selected row flashes, and
  the menu waits for release and resyncs the debouncer.
- Write one generic `runChoiceMenu(title, labels, n, current)`. pushbuttonDEF
  repeats that loop five times; don't copy the duplication.

| # | Item | Behavior |
|---|---|---|
| 1 | Calibrate head | 0/100/200/300 g on CH2 only. Live raw and grams, fit residual on the done screen. Saves via the v2 record |
| 2 | Calibrate handle | Same for CH1. **New:** a single channel can be recalibrated without redoing the other |
| 3 | Zero check | Live raw/grams on both channels, unloaded noise and drift readout |
| 4 | NFC mode | Enable / Disable / Back. Disabled skips `nfc.begin` and the tag prompt, and the result screen stays up (pushbuttonDEF behavior) |
| 5 | Backup / restore | Save / Restore / Back, "Two copies checked" feedback |
| 6 | Status | Version, machine ID, cal source/recordId, I2C presence, live grams per channel |
| 7 | Exit | Return to IDLE |

The idle behavior is unchanged: a short press measures and a 3 s hold tares.
If the fixed 2"/13" lever arms become a per-machine parameter (see open
decisions), they go in the calibration record, not in the menu.

## Phase 4: Serial protocol

**Both builds**
- `SCALE INFO` / `FIELD INFO` →
  `{"protocol":1,"build_mode":…,"machine_id":"esp32-<mac>","firmware_md5":…,"version":…,"cal_record_id":…,"cal_crc":…,"cal_source":…,"counts_per_gram_head":…,"counts_per_gram_handle":…,"nfc_enabled":…,"nfc_ready":…,"nau_present":…,"oled_present":…,"uptime_ms":…}`
- After every button measurement, emit
  `FIELD RESULT {"head_g":…,"handle_g":…,"raw_head":…,"raw_handle":…,"mass_oz":…,"bp_in":…,"esw":…,"nfc":"written|skipped|failed"}`.

**Commissioning build only.** Nothing moves, so these commands are safe to retry.
- `SCALE READ <HEAD|HANDLE> <n>` returns raw trimmed mean, min, max, SD and n,
  using the same `nauReadChannel` path as the field build.
- `SCALE TARE`
- `SCALE CAL INSTALL <recordId> <head_slope_hex> <head_zero> <handle_slope_hex> <handle_zero> <crc>`:
  floats travel as bit patterns (as in `QUAL CURVE NODE`). The device
  recomputes the CRC, saves, reads back, then replies
  `SCALE CAL INSTALLED <id> <crc>`.
- `SCALE BACKUP SAVE`

## Phase 5: Host tooling (`scripts/`)

Copy these generic pieces from pushbuttonDEF almost verbatim:
- `setup_workstation.py` and `requirements-workstation.txt`
- `digest`, `write_json`, `seal`, `verify`
- `machine_lock`, `commit_run` (`git commit --only`), `port_name`
- The `Connection`/query class, with the motion-specific retry rules removed

New files:
- **`scale_commissioning.py`** holds the recipe, capture helpers, fit and report,
  and replicates `ScaleMath` in Python.
- **`commission_scale.py`** provides `start | resume | verify | analyze | fleet | field-check`.
- **`machine.py`** provides `ports | status | capture` (listens for `FIELD RESULT`).
- **Tests** (`python -m unittest`):
  - fit math against fixed vectors
  - `ScaleCalRecord` packing/CRC against a known device-produced blob
  - install sequence against a fake connection
  - resume and integrity refusal

**Commissioning recipe v1.** Every reference artifact has a physical label ID.

| Stage | What | Why |
|---|---|---|
| Preflight | INFO, identity, firmware MD5, I2C presence | Gate before anything is recorded |
| Zero stability | 60 s unloaded, both channels | Noise SD, drift in counts/min |
| Calibration | Per channel, 0 → 100 → 200 → 300 g, then back down | Slope, intercept, linearity residual, hysteresis |
| **Cross-talk** | Weight on head while reading handle, and the reverse | Catches CH2 PGA-cap-style faults, which this check would have caught earlier |
| Install | Host fits, device validates CRC and reads back, INFO must match | Host and device hold identical calibration |
| Verification | Check weights not used in cal, on each channel | Independent error |
| Reference paddle | 10–20 placements of a labeled golden paddle | Mass/BP/ESW mean, SD, range, and host-vs-device agreement |
| Seal | manifest.json, report.md, drift and residual plots, Git commit | Status ENGINEERING_REVIEW |

**Field handoff (`field-check`).**
1. Flash the `field` env with a normal upload, never erase.
2. Operator-attested power cycle.
3. Check identity and the calibration CRC.
4. Measure the golden paddle with a button press. `FIELD RESULT` must agree with the host.
5. Operator enters the OLED value.
6. NFC tag read-back reports `VERIFIED` or `NOT REQUIRED: <reason>`.
7. Operator types `ACCEPT PROTOTYPE`, which sets PROTOTYPE_READY. Then take a menu or host backup snapshot.

Records go to
`qualification/results/<machine_id>/<run>/` and
`qualification/handoffs/<machine_id>/<run>/`.

## Phase 6: Docs and agent operability

- `WORKSTATION.md`: clone, setup, port discovery, Git identity.
- `AGENT_RUNBOOK.md`: short and in the same rule style:
  - Never infer a physical observation.
  - Discover the port, never assume it.
  - Allow one serial client.
  - Never erase flash or edit sealed records.
  - Relay runner prompts verbatim.
- `qualification/COMMISSIONING.md`: recipe, statuses, recovery.
- `qualification/machines/<id>.json`: profile per commissioned scale.
- Update `CLAUDE.md` with the menu, NVS v2, protocol, envs, and a pointer to the runbook.

## Phase 7: First commissioning

Commission one physical dualScale end to end, then produce a PROTOTYPE_READY
handoff and a profile.

## Suggested order and checkpoints

0 → 1 → 2 → 3 can land as separate commits, each flash-tested on hardware.
Phase 4 depends on 2. Phase 5 depends on 4, and its record-packing test needs one
real device blob. Phases 6 and 7 come last.

## Open decisions

1. **Two builds vs one.** A read-only `field` build (the pushbuttonDEF model) or a
   single build that accepts cal install only while in a menu "Commissioning"
   mode? Two builds keeps parity with the fleet tooling.
2. **Reference artifacts.** Which labeled cal/check weights and golden paddle
   exist? Is the golden paddle's mass/BP known from an independent scale?
3. **Lever arms.** Should the fixed 2"/13" support distances (and the ESW
   constant 2.08) be commissioned per machine? A known mass at two positions
   would measure them. This is the scale's analogue of frame compensation.
4. **Existing calibration.** Import it once (proposed), or force a recalibration on upgrade?
5. **Shared Python.** Copy the generic modules (proposed; keeps each repo
   self-contained per WORKSTATION.md), or extract a shared package later?
