# Project Falcon sensor firmware (prototype 0.1)
XIAO ESP32C3 + ADXL345, USB-C powered, no sleep. Sends HMAC-signed JSON to the backend's `POST /v1/ingest` (see API_CONTRACT.md).

## Wiring (ADXL345 breakout → XIAO)
| ADXL345 | XIAO |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | D4 (GPIO6) |
| SCL | D5 (GPIO7) |
| CS  | 3V3 (selects I2C) |
| SDO | GND (address 0x53) |

## Setup and flashing
1. `pip install platformio` (or use a venv).
2. `cp include/secrets.example.h include/secrets.h` and fill in the 2.4 GHz Wi-Fi details, `SERVER_URL`, and `DEVICE_ID`/`SIGNING_KEY` from the backend provisioning record. secrets.h is gitignored.
   Local backend: Docker on the Mac (port 8080), see FIRST_TEST_M5STICKS3.md "Local backend on the Mac".
   For https, paste the server's root CA PEM into `SERVER_CA_PEM`. Leaving it empty skips the certificate check, which is for testing only.
3. Plug in over USB-C, then run `pio run -t upload` and `pio device monitor`. If the upload fails, hold BOOT while you plug it in.
4. Wi-Fi can also be set over serial with `wifi <ssid> <pass>`, which is saved to NVS. BLE provisioning (Espressif's standard flow) comes later.

## Mounting
- **Top-loader lid:** put it flat on the lid. Opening the lid tilts it, so lid open = more than 20° held for 1 s. Set `set mount_door 0`.
- **Front-loader / dryer door:** `set mount_door 1`. A door on a vertical hinge mostly rotates *without* changing gravity, and the ADXL345 has no gyro. So detection uses a jerk spike plus a gravity-direction change over 15° within 2 s. That works best mounted near the latch edge, and on doors with any sag or tilt. Expect misses here; this is the main thing to test.
- Calibrate with the door closed and the machine still: on boot, with a 2 s long press of BOOT, or with `cal`.

## Serial console (115200 baud)
`help`, `dump` (thresholds), `set <key> <val>`, `save`, `defaults`, `cal`, `stream on|off` (accel x/y/z, angle, RMS, jerk, state at 10 Hz), `status`, `send <event>`.

## Tuning procedure
1. `stream on` with the machine idle and the door closed. Note the idle `rms` and `jerk`. Set `quiet_rms` to about 2× idle.
2. Start a cycle and note `rms` during washing and spinning. Set `run_rms` well below the lowest running value and above `quiet_rms`.
3. Open and close the lid or door 10 times and note the peak `ang` and `jerk`. Adjust `lid_open_deg` / `door_deg` / `door_jerk`.
4. Pick the puck up and set it down. Set `moved_jerk` above the door-slam jerk but below the pick-up jerk.
5. `save`. Confirm that `cycle_start` appears after 30 s of running and `cycle_end` after 3 min of quiet (`run_hold_s`, `done_quiet_s`).

## IMU option: LSM6DS3 / LSM6DSO (6-axis, gyro)
`pio run -e xiao_lsm6ds3 -t upload`. Wiring is the same I2C pins, with SDO/SA0 to GND for 0x6A. Mount the puck so the gyro axis set by `GYRO_DOOR_AXIS` (z by default) is parallel to the door hinge.
A door swing is detected when the gyro rotation integrated about that axis is more than `gyro_door_deg` (30°) within `door_window_ms` (2 s). If open and close come out backwards, flip it with `set gyro_open_sign -1`. The default build is still the ADXL345 (`seeed_xiao_esp32c3`).

## Flags
`f` is v=3 flags. The backend defines only bit2 (0x04) = charging. On USB power the prototype has no battery, so charging is unknown and always sent as 0. There is no armed flag bit: the armed state comes from the server reply's `a`.

## Anti-theft siren (fw 0.3)
- Wiring (prototype): passive piezo between **D2 (GPIO4)** and GND, which gives ~70 dB. For production loudness, D2 → PAM8904 DIN and D3 (GPIO5) → PAM8904 EN.
- Arming: the backend `arm` command enables the siren after a **30 s exit delay**. `disarm`, `silence_alarm`, or **load done (cycle_end)** turns it off. After load done, the siren stays off until the next explicit `arm`, even if a reply carries `a:1`.
- Trigger: only lift/move (jerk > `moved_jerk`, **2 hits within 2 s**). Lid/door tilt never triggers it: hits are ignored during a lid/door transition and for 3 s after a door_open/close. Motion within 10 s of a disarm/silence is ignored.
- Sound: 4 kHz LEDC square wave, 250 ms on / 100 ms off, for 30 s. Then 60 s before it can trigger again.
- Events: see the event table below. New command: `silence_alarm`.
- Serial test: `siren` (forces an alarm), `silence`, `status`.
- Mount with magnets only, no adhesive.

### Silence polling (fw 0.3)
While the siren sounds, the firmware posts an empty `heartbeat` every 5 s (`ALARM_POLL_MS`) so a `silence_alarm` from the app lands within ~5 s. It stops when the alarm ends. Note: an HTTPS post blocks up to 8 s on a bad link. The LEDC tone keeps running in hardware, but the beep pattern/30 s timeout can lag by that much.

### Command codes (match backend `src/protocol.ts` CMD_CODES)
| code | name (JSON `type`) | arg |
|---|---|---|
| 0x01 | `arm` | minutes, 0 = until disarm |
| 0x02 | `disarm` | – |
| 0x03 | `set_heartbeat_hrs` | hours |
| 0x04 | `reboot` | – |
| 0x05 | `silence_alarm` | – (**new; backend must add**) |

### User reset (all boards)
Hold the button (XIAO: BOOT; M5: Button A) for **10 s from power-up**. That wipes Wi-Fi creds + tuning but keeps the message counter, and ownership stays on the server.

## M5StickC Plus2 build (`env:m5stickc_plus2`)
No wiring needed. It uses the onboard MPU6886 (gyro door swing + accel), passive buzzer GPIO2, Button A GPIO37, and HOLD GPIO4 (driven high at boot so it stays on battery).
1. Copy `include/secrets.example.h` → `include/secrets.h` and fill in 2.4 GHz Wi-Fi, server URL, and device id/key.
2. Connect USB-C. If no port appears: unplug, hold the power button (left side, Button C) ~6 s until off, then replug. On Windows/macOS you may need the CH9102 driver from M5.
3. `pio run -e m5stickc_plus2 -t upload` then `pio device monitor -b 115200`.
4. Set `set mount_door 1` (door/lid swing via gyro) and `save`. `siren` tests the buzzer. **It is quiet (~60–70 dB, unverified), fine for bench tests but not an anti-theft level.**
5. On battery (200 mAh), expect only hours of runtime: Wi-Fi is always on, with no sleep.
Unverified: the MPU6886 I2C address 0x68 (firmware checks WHO_AM_I 0x19 and prints an error if not found), the red LED polarity (unused), the buzzer loudness, and the gyro hinge axis (`GYRO_DOOR_AXIS`=2, may need changing for how the stick is mounted).

## M5StickS3 build (`env:m5sticks3`)
Uses M5Unified 0.2.25 (it supports StickS3: BMI270, ES8311/AW8737 speaker, M5PM1 PMIC, buttons). Flash with `pio run -e m5sticks3 -t upload`. If that fails, hold the side button until the green LED flashes (download mode). Full test checklist: `FIRST_TEST_M5STICKS3.md`.
Button: click = arm/disarm, click while sounding = silence, hold 2 s = recalibrate, hold 10 s from boot = user reset. Siren = 600→1500 Hz sweep, volume 75% (100% only if `M5.Power.getVBUSVoltage()` reports USB; unverified on StickS3, so expect 75%).

### Event table (fw 0.3, names/codes agreed with Back End Developer)
| code | event | when |
|---|---|---|
| 0x01 | `door_open` | lid/door opened |
| 0x02 | `door_close` | lid/door closed |
| 0x03 | `tamper` | moved/lifted (any arm state, max 1 per 10 s) |
| 0x05 | `cycle_end` | load done |
| 0x06 | `heartbeat` | 15 min check-in; also the 5 s poll while sounding |
| 0x07 | `button` | (reserved; the firmware now sends armed_local/disarmed_local instead) |
| 0x08 | `boot` | startup |
| 0x09 | `ack` | carries a command ack only |
| 0x0D | `cycle_start` | vibration lasted 30 s |
| 0x0E | `alarm_start` | siren started |
| 0x0F | `alarm_timeout` | siren ran the full 30 s |
| 0x10 | `alarm_silenced` | silenced by the app (`silence_alarm`) or the button |
| 0x11 | `alarm_disarm` | siren stopped by disarm |
| 0x12 | `alarm_done` | siren stopped by load done |
| 0x13 | `armed_local` | button armed the puck locally |
| 0x14 | `disarmed_local` | button disarmed the puck locally |
| 0x15 | `cycle_stalled` | after cycle_start, vibration stopped > `STALL_MS` (90 s) and the cycle is still < `MIN_CYCLE_MS` (10 min). Extra field `el` = seconds since cycle start (placeholder thresholds) |

**Local arm toggle (all envs):** a short press toggles arm and immediately queues `armed_local`/`disarmed_local`. Until that event gets HTTP 200, the reply flag `a` is ignored, so a stale `a=0` can't undo the toggle. After the ack, the firmware follows `a` again. An explicit `arm`/`disarm` command from the server always wins and clears the pending state. If no ack comes within 5 min, it gives up and follows the server.

### 0x06 `set_state` and the machine-colour screen (M5StickS3)
| code | name | arg |
|---|---|---|
| 0x06 | `set_state` | 0 = available (GREEN "FREE"), 1 = claimed (RED "IN USE"), 2 = out of order (AMBER) |
Acked like the other commands. Every JSON reply's `st` field is treated as the truth. State is stored in NVS (and RTC memory in the sleep build).
- `m5sticks3` (USB): full-screen colour always on.
- `m5sticks3_sleep`: screen off. The colour flashes 3 s on motion wake / button press. Serial `disp 1 <N>` adds a periodic flash every N s (costly, see FIRST_TEST §5), and `disp 2` keeps it always on while on USB (no sleep then).
- Serial test: `state 0|1|2`.
StickS3 display facts (docs.m5stack.com/en/core/StickS3): ST7789P3, 1.14", 135×240, backlight on **G38** (ESP32 GPIO, PWM through M5GFX, not the M5PM1). The only LED documented is the internal green LED that flashes in download mode. It's not documented as user-controllable, so it's unverified and unused.

### Battery sleep (`m5sticks3_sleep`, -DSLEEP_ENABLED)
- Wake path (**confirmed in M5 docs**, docs.m5stack.com/en/arduino/m5sticks3/m5pm1): BMI270 INT1 → M5PM1 PYG4 → M5PM1 PYG1 (IRQ output) → ESP32-S3 **G13** (active low) → `ext0` wake. KEY1 (G11) → `ext1` wake (button active-low is unverified). A timer wake does the check-in (`heartbeat_min`, set it 15–60). RTC wall time decides check-in versus flash-only wakes.
- Idle 20 s with nothing to send → deep sleep (Wi-Fi off, `M5.Speaker.end()` = amp off, screen off, BMI270 any-motion ~46 mg/40 ms). During a load: light sleep with 1 s bursts at 100 Hz every ~10 s.
- Armed motion wake: the wake counts as the first jerk hit, so one more confirms the siren (target <1 s, unmeasured). The lid/door rules still apply. Network goes after. Offline, only the button silences. Pending server commands and `st` arrive at each check-in/post.
- State kept: counter (NVS), armed/siren-allowed (RTC + NVS), rest-gravity, lid state, pending local arm, machine state (RTC). HTTP keep-alive is reused within a wake.
- Libraries: M5PM1, SparkFun BMI270 (the any-motion config per M5's example).

### Heartbeat (`hb`)
- Default period: **120 s on `m5sticks3` (USB)**, 3600 s on `m5sticks3_sleep`, 900 s on other boards (`HB_DEFAULT_S`).
- Any server reply containing `"hb":<seconds>` sets the period, clamped to 30–3600 s and saved in NVS (`hb_s`). `set_heartbeat_hrs` (0x03) still works and sets hours × 3600 s.
- The old `heartbeat_min` tunable is no longer used.
- Sleep build: it follows `hb` anyway. **hb:120 on battery ≈ 720 check-ins/day × ~0.085 mAh ≈ 61 mAh/day → ~3.5 days even idle (estimate).**

### Data capture (all boards)
`log on` streams CSV at 50 Hz (`LOG_HZ`): `L,t_ms,ax,ay,az,gx,gy,gz,run,open,armed,rms`. Firmware events appear as `E,t_ms,event,arg`. `mark <label>` writes `M,t_ms,label` for phase labels. `log off` stops. gx/gy/gz are dps (0 on the ADXL345). Split with `python3 tools/split_log.py run.csv`.

## Microphone and privacy (M5StickS3)
The StickS3's MEMS mic (ES8311 codec) is used for three things: done-chime detection, bang evidence while guarded, and a sound level.

**No audio is ever recorded, stored, logged, printed or sent.** Samples exist only in one 32 ms RAM buffer that is analysed and then overwritten. Only levels, flags and tone matches leave the analysis code. The learned fingerprint (`sndlearn`) holds tone features only (up to 8 notes of frequency bin, duration in ms and level, plus total duration and a jingle/buzzer type), never audio or raw samples. It stays on the puck in NVS (`snd_fp`) and is never posted.

- Post fields: `db` (integer, dBFS+100, **uncalibrated**, relative only; omitted when there's no reading in the last 2 min) and `snd:1` (one-shot, on the post after a chime match).
- Events: `done_chime` (chime heard while QUIETING or within 5 min after `cycle_end`; never changes the run state, the server decides Done) and `bang` (arg/`el` = peak db, while armed, max one per 10 s; evidence only, never the siren).
- Reply field: `sndOn` 0/1 turns the mic off/on (saved).
- Serial: `snd` (toggle a stream of db, peak and top Goertzel bin), `sndlearn` (12 s capture of the machine's chime into the fingerprint), `sndforget`, and `status` shows `db=` and the last chime.
- Detection: Goertzel at 16 bins, 250 Hz–4 kHz, on 32 ms windows at 16 kHz. Learned fingerprint first (±1 bin, timing tolerant), then generic rules: JINGLE = ≥2 tonal notes (≥1 kHz, 100–800 ms) within 5 s; BUZZER = sustained harmonic tone ≤1.6 kHz for ≥1 s.
- Codec sharing: the mic and the speaker can't run together. The mic stops for the siren, the never-started jingle and button beeps, and the speaker is restarted before every tone.
- Battery build: continuous listening only in QUIETING (already awake), `snd` or `sndlearn`; otherwise 64 ms each second while awake, and only when it matters (chime window or armed). It never keeps the puck awake just to listen.
