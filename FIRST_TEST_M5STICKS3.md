# First test: M5StickS3 laundry sensor (USB power, no temperature test)

## 0. Setup (once)
1. Install VS Code + the **PlatformIO** extension (or `pip install platformio`).
2. Open the folder `laundry-sensor/firmware`. Copy `include/secrets.example.h` → `include/secrets.h` and fill in: `WIFI_SSID`/`WIFI_PASS` (**2.4 GHz network only**), the server URL (Back End Developer's local server on your LAN), and the `DEVICE_ID` + `SIGNING_KEY` from the backend's test device.
3. Start the backend: `npm run dev` in `laundry-sensor-backend`.
4. Plug the StickS3 in by USB-C. Flash: `pio run -e m5sticks3 -t upload`. If the upload fails, hold the side reset button until the green LED flashes (download mode) and retry.
5. Monitor: `pio device monitor -b 115200`. You should see `Project Falcon sensor fw 0.3`, `[WIFI] connecting`, then a `boot` event accepted (HTTP 200). Type `status` to see `wifi=1`, rssi, and ctr.
   - Wi-Fi change without reflashing: type `wifi <ssid> <password>`.

**Controls:** click the front button = arm/disarm (high beep = armed, low = disarmed). Click while the siren sounds = silence. Hold 2 s = recalibrate. Hold 10 s from power-up = user reset (wipes Wi-Fi + tuning).

## 1. Top-load washer (lid)
1. Mount: on top of the lid, flat, near the front edge (magnetic back), so it doesn't sit over the hinge. Leave the USB cable slack for the lid swing.
2. With the lid closed and the machine still: hold the button 2 s (calibrate). Type `set mount_door 0`, `save`.
3. Type `stream on`. Open the lid fully, then close it. Note the `ang=` value with the lid open and closed. You should get `door_open` then `door_close`. If not, adjust `set lid_open_deg <~half the open angle>` / `lid_close_deg`.
4. Start a wash. Watch `rms=` during agitation and spin and with it idle. You want `run_rms` between the idle and the gentlest-agitation values (default 0.020 g). `cycle_start` should appear after 30 s of vibration.
5. Load done: note the wall-clock time the machine actually stopped versus when `cycle_end` arrives (default 180 s of quiet). Note any false `cycle_end` during soak/fill pauses; if so, raise `done_quiet_s`.

## 2. Front-load dryer (door swing, gyro)
1. Mount: on the door face (outer side, away from the hinge), or on the cabinet side as an option (vibration + load done only, no door).
2. Type `set mount_door 1`, `save`, then calibrate with the door shut.
3. `stream on`, swing the door open and shut. If no `door_open`, the hinge axis is wrong. Try `GYRO_DOOR_AXIS` 0/1/2 in `config.h` (vertical hinge with the stick upright is likely 1 or 2, **unverified**) and reflash. Also try `set gyro_open_sign -1` if open/close are swapped, and `set gyro_door_deg` (default 30).
4. Run a dry cycle: note `rms` while tumbling, `cycle_start`, and stop time versus `cycle_end`.

## 3. Alarm (lift / silence)
1. Click the button → armed (serial: `[BTN] ARMED`, then `[ARM] server acked armed_local`). **Wait 30 s** (exit delay).
2. Lid open/close only → **must NOT sound**.
3. Lift or slide the puck off → siren within ~2 s (`[ALARM] siren ON`, event `alarm_start`). Volume is capped at 75% (100% only if USB power is detected, which is unverified on StickS3).
4. Click → silence (`alarm_silenced`). Repeat and let it run the full 30 s (`alarm_timeout`). If the backend has `silence_alarm`, try silencing from the app/API and note the latency (should be ≤5 s).
5. Disarm with a click, lift → no siren.

## 3b. Data capture for future ML (do this on every real cycle)
1. Close any open monitor, then start a logged monitor: `pio device monitor -b 115200 | tee washer_run1.csv` (use `dryer_run1.csv` etc.).
2. Type `log on`. The screen fills with `L,...` lines at 50 Hz.
3. Label phases as they happen, typing them while the log runs: `mark idle`, `mark door`, `mark fill`, `mark wash`, `mark rinse`, `mark spin`, `mark drain`, then `mark idle` at the end. Approximate is fine.
4. Note any `E,...,cycle_stalled,<s>` lines (placeholder thresholds: 90 s quiet within the first 10 min). Was that a real stop or a soak/fill pause?
5. `log off`, stop the monitor, then: `python3 tools/split_log.py washer_run1.csv`. It prints per-phase sample counts and mean vibration, and writes `washer_run1_split/labeled.csv` + `by_label/*.csv` + `events.csv`.
6. The heartbeat now runs every 2 min (`[HTTP] ... "t":"heartbeat"` lines). That's expected.

## 4. What to record and send back
- `dump` output (the final thresholds you used) for the washer and the dryer.
- The raw `*_run*.csv` logs and the `_split` folders (zip them).
- About 30 s of `stream on` lines for: idle, lid/door open+close, agitate/tumble, and spin.
- Event list with times: door_open/close, cycle_start, cycle_end, and the real machine stop time.
- Which `GYRO_DOOR_AXIS` / `gyro_open_sign` worked, and the mount spot (photo helps).
- Alarm: did lid-open stay silent? Lift-to-siren time? Loudness impression? Any reboots?
- Any `[ERR]` lines, HTTP codes other than 200, and the Wi-Fi RSSI (`status`) in the laundry spot.

## Local backend on the Mac (from Back End Developer)
1. `cd laundry-sensor-backend && docker compose up --build -d`
2. `docker compose ps`. Both containers should show healthy.
3. `curl http://localhost:8080/health`. It should return `{"ok":true}`.
4. `docker compose exec api npm run homekit:setup`. This prints the device ID, the signing key, the secrets.h lines and a dev login (dev@laundryguard.local / devpassword1).
5. `ipconfig getifaddr en0` gives the Mac's LAN IP. If it's empty, try `en1`.
6. In `include/secrets.h`, set `SERVER_URL "http://<MAC_IP>:8080"` and paste the device ID and key from step 4.
7. If macOS asks about incoming connections for Docker, click Allow. Check from a phone: `http://<MAC_IP>:8080/health`.
8. Watch the server logs with `docker compose logs -f api`.
This will be the first real `docker compose` run. The code passed its 75 tests, but if step 1 or 2 fails, send the output to the Back End Developer.

## 5. Optional: battery-sleep build (`m5sticks3_sleep`)
Tomorrow's USB test uses `m5sticks3` (unchanged, screen always shows the machine colour). To try battery mode: `pio run -e m5sticks3_sleep -t upload`. Serial `sleep off` keeps it awake for debugging, and `sleep on` re-enables sleep (saved). After a deep sleep, the USB serial port disappears until it wakes, so press the button to wake it, then type within 20 s.
1. Unplug USB and note the time. Leave it still: after 20 s, `[SLEEP] deep sleep` (screen off).
2. Lift it while disarmed: it wakes, flashes the colour for 3 s, and goes back to sleep. Armed: lift → siren in <1 s (target, so time it), then the network event follows.
3. Run a load: it should stay in light sleep with 1 s sampling bursts every ~10 s, and `cycle_end` should still arrive (note the extra delay versus USB).
4. Battery: start at full charge, record the hours until shutdown (or the % from the app), and the loads run. Send back the number, plus how often it woke from neighbouring machines (any-motion threshold `SLEEP_ANYMOTION_THR`, default ~46 mg).
5. Display modes (serial): `disp 0` = flash on wake/button (default), `disp 1 300` = also flash every 300 s, `disp 2` = always on while on USB.

### Battery estimate, 250 mAh (~225 mAh usable) — ALL ESTIMATES, not measured
Sources: M5 StickS3 spec page (whole-device figures at 4.2 V: L2 = ESP32 asleep **102 µA**, L1 = IMU + PMIC only **52 µA**, L3A active **36.7 mA**); ESP32-S3 datasheet (light sleep ~240 µA, Wi-Fi TX ~280–340 mA peaks); BMI270 datasheet (accel low-power ~25 µA, full acc+gyro ~685 µA). **M5PM1 quiescent current is not published separately.** It's inside M5's 102 µA board figure. Flagged unknown.

| Mode | Current | Note |
|---|---|---|
| Deep sleep, IMU any-motion on, Wi-Fi off, screen off | ≈ 0.12 mA | 102 µA board (M5) + ~25 µA BMI270 |
| Awake, no radio, screen off (wake window) | ≈ 37 mA | M5 L3A figure |
| Light sleep between load bursts | ≈ 1.0 mA | 0.24 ESP32-S3 + 0.1 board + 0.69 IMU full |
| Load sampling (1 s burst / 10 s) | ≈ 4.6 mA avg | (37×1 + 1.0×9)/10 |
| Wi-Fi wake + connect + TLS + post | ≈ 0.08 mAh per wake | ~2.5 s at ~110 mA; extra posts in the same wake ~0.015 mAh (keep-alive) |
| Screen colour flash 3 s (backlight ~12 mA at 160/255 + LCD ~3 mA [E]) | ≈ +0.013 mAh per flash | |

- **Per load** (~50 min): sampling 3.8 + ~4 posts 0.25 + ~3 motion wakes × 20 s awake 0.6 ≈ **4.7 mAh**.
- **Idle per day:** sleep floor 2.9 + check-ins (60 min: 24 × 0.085 = 2.0; 15 min: 8.2) ≈ **4.9 mAh/day (60-min check-in)** / 11.1 (15 min).
- **Battery life:** idle-only ≈ **46 days** (60 min) / 20 days (15 min). Armed-idle ≈ the same (any-motion is on either way), minus ~0.2 mAh per false wake. Laundromat **8 loads/day ≈ 42 mAh/day → ~5 days**. **15 loads/day ≈ 75 mAh/day → ~3 days**. Home (2 loads/day) ≈ 14 mAh/day → ~16 days.
- Biggest lever: a sampling burst every 30 s instead of 10 s gives ≈ 2.7 mAh/load (8 loads → ~8 days, 15 → ~5 days). Second: fewer spurious wakes from neighbouring machines (threshold tuning).
- Display options: default flash ≈ negligible (~0.25 mAh/day at 20 wakes). **Periodic flash every 60 s ≈ +70 mAh/day (→ ~3 days even idle), every 300 s ≈ +14 mAh/day.** Always-on is USB only (no battery cost).
