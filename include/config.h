#pragma once
// ---- Pins ----
#if defined(BOARD_M5STICKS3)
// M5StickS3 (ESP32-S3-PICO-1-N8R8). docs.m5stack.com/en/core/StickS3 (2026-10-13). Peripherals via M5Unified.
#define PIN_SDA        47  // internal I2C: BMI270 0x68, M5PM1 0x6E, ES8311 0x18
#define PIN_SCL        48
#define PIN_BOOT_BTN   11  // KEY1 (M5.BtnA); KEY2 = G12. Read through M5Unified, polarity handled there
#define BTN_PULL       INPUT
#define S3_VOL_BATTERY 191 // 75% of 255 - M5 docs: keep speaker <75% on battery or the device may reboot
#define S3_VOL_USB     255
#elif defined(BOARD_M5STICKC_PLUS2)
// M5StickC Plus2 (ESP32-PICO-V3-02). Pin facts from docs.m5stack.com/en/core/M5StickC PLUS2 (2026-10-13).
#define PIN_SDA        21  // internal I2C: MPU6886 + BM8563
#define PIN_SCL        22
#define PIN_BOOT_BTN   37  // Button A (front), active low, external pull-up (GPIO37 is input-only)
#define PIN_HOLD       4   // power HOLD: must be driven HIGH right after boot or the stick powers off on battery
#define PIN_LED        19  // red LED, shared with IR emitter (polarity UNVERIFIED, not used yet)
#define MPU6886_ADDR   0x68 // UNVERIFIED on this unit (MPU6886 default with AD0 low); WHO_AM_I 0x75 = 0x19
#define BTN_PULL       INPUT
#else
// XIAO ESP32C3
#define PIN_SDA        6   // D4
#define PIN_SCL        7   // D5
#define PIN_BOOT_BTN   9   // BOOT button, active low
#define BTN_PULL       INPUT_PULLUP
#endif
#define FACTORY_HOLD_MS 10000 // hold the button 10 s right after power-up -> user reset (wipe Wi-Fi + tuning, keep counter)
#define ADXL345_ADDR   0x53 // SDO->GND
// IMU selection: default ADXL345. Build with -DIMU_LSM6DS3 (env xiao_lsm6ds3) for the 6-axis gyro option.
#define LSM6_ADDR      0x6A // SDO/SA0->GND (0x6B if high)
#define GYRO_DOOR_AXIS 2    // 0=x 1=y 2=z gyro axis aligned with the door hinge

// ---- Sampling ----
#define SAMPLE_HZ          100
#define GRAVITY_LPF_ALPHA  0.03f   // gravity low-pass (per sample), ~0.33 s time constant
#define RMS_WINDOW_S       1.0f    // window for high-pass RMS
#define CAL_SAMPLES        200     // 2 s at rest
#define LONGPRESS_MS       2000

// ---- Defaults for runtime-tunable thresholds (serial `set`, persisted in NVS) ----
#define DEF_MOUNT_DOOR       0      // 0 = lid (top-loader), 1 = vertical door (front-loader / dryer)
#define DEF_LID_OPEN_DEG    180.0f  // vibration-only (Terry 10/14): lid/door off, open always 0
#define DEF_LID_CLOSE_DEG   179.0f
#define DEF_LID_HOLD_MS      1000
#define DEF_DOOR_DEG         15.0f  // angle change within window
#define DEF_DOOR_WINDOW_MS   2000
#define DEF_DOOR_JERK        8.0f   // g/s jerk spike that starts a door-swing window
#define DEF_RUN_RMS          0.030f // g: must stay at/above this for the whole hold (desk noise peaked 0.023, 10/14)
#define DEF_RUN_HOLD_S       10   // run=2 needs >=10 s steady vibration (Terry 10/14)
#define DEF_QUIET_RMS        0.025f // g, below this = quiet (kept under run_rms for hysteresis)
#define DEF_DONE_QUIET_S     30     // Terry: stopped load shows done within 30 s
#define DEF_MOVED_JERK       25.0f  // g/s, large jerk while not running
#define DEF_MOVED_DEG        30.0f  // (reserved)
#define DEF_RUN_MAX_DEG     180.0f  // running only if angle from calibration stays within this
#define DEF_RUN_DRIFT_DEG   180.0f  // max angle swing during the run_hold_s window
#define DEF_RUN_SETTLE_S     15     // ignore vibration this long after a jerk/move
#define DEF_GYRO_DOOR_DEG    30.0f  // LSM6DS3: integrated hinge rotation within door_window_ms
#define DEF_GYRO_OPEN_SIGN   1      // +1/-1: rotation direction that means "opening"

// ---- Networking ----
#define HEARTBEAT_MS       (15UL * 60UL * 1000UL)   // legacy tunable default (heartbeat_min); hbSec below is what's used
#if defined(BOARD_M5STICKS3) && defined(SLEEP_ENABLED)
#define HB_DEFAULT_S       3600   // battery: hourly check-in (server `hb` still wins; hb:120 costs ~60 mAh/day, ~3.5 days idle)
#elif defined(BOARD_M5STICKS3)
#define HB_DEFAULT_S       120    // USB test build: check in every 2 min
#else
#define HB_DEFAULT_S       900
#endif
#define MIN_TX_INTERVAL_MS 2000   // max 1 message / 2 s
#define HTTP_TIMEOUT_MS    5000   // connect + read cap (was 8 s); one retry
#define HTTP_SLOW_MS       2000   // log posts slower than this
#define PROTOCOL_VERSION   3
#define EVENT_QUEUE_LEN    16

// ---- Anti-theft siren (PAM8904 piezo driver + TDK PS1240P02BT 12.2 mm piezo, 4 kHz) ----
#if defined(BOARD_M5STICKS3)
#define PIN_BUZZER          -1     // speaker via ES8311/I2S + AW8737 (M5.Speaker)
#define PIN_BUZZER_EN       -1
#elif defined(BOARD_M5STICKC_PLUS2)
#define PIN_BUZZER          2      // onboard passive buzzer (quiet: roughly 60-70 dB, UNVERIFIED - not an anti-theft level)
#define PIN_BUZZER_EN       -1     // no driver enable
#else
#define PIN_BUZZER          4      // D2: LEDC PWM -> PAM8904 DIN (prototype: piezo direct GPIO->piezo->GND, ~70 dB)
#define PIN_BUZZER_EN       5      // D3: PAM8904 EN (high = on; boost off when low). Unused on prototype.
#endif
#define BUZZER_FREQ_HZ      4000   // PS1240P02BT resonance (datasheet 4 kHz)
#define BUZZER_LEDC_CH      0
#define BUZZER_LEDC_RES     8      // bits; 50% duty = 128
#define ALARM_SOUND_MS      30000  // siren length per alarm
#define ALARM_BEEP_ON_MS    250    // pulsed pattern: on/off
#define ALARM_BEEP_OFF_MS   100
#define ALARM_EXIT_DELAY_MS 30000  // ignore motion for 30 s after an arm command (time to walk away)
#define ALARM_DISARM_GRACE_MS 10000 // ignore motion for 10 s after disarm/silence even if reply flag says armed
#define ALARM_DOOR_GRACE_MS 3000   // motion within 3 s of a lid/door state change is treated as the lid, not a lift
#define ALARM_CONFIRM_HITS  2      // jerk hits needed...
#define ALARM_CONFIRM_MS    2000   // ...within this window (debounce: one bump/slam != theft)
#define ALARM_REARM_MS      60000  // after an alarm ends, wait 60 s before another can start
#define ALARM_POLL_MS       5000   // while sounding: poll the server every 5 s so silence_alarm lands fast

// ---- Server command codes (match backend src/protocol.ts CMD_CODES; JSON path uses the names) ----
#define CMD_ARM               0x01  // "arm"   arg = minutes, 0 = until disarm
#define CMD_DISARM            0x02  // "disarm"
#define CMD_SET_HEARTBEAT_HRS 0x03  // "set_heartbeat_hrs" arg = hours
#define CMD_REBOOT            0x04  // "reboot"
#define CMD_SILENCE_ALARM     0x05  // "silence_alarm"
#define CMD_SET_STATE         0x06  // "set_state" arg 0 = available/GREEN, 1 = claimed/RED, 2 = out-of-order/AMBER

// ---- Battery sleep (env m5sticks3_sleep, -DSLEEP_ENABLED) ----
#define SLEEP_IDLE_MS        20000  // still for 20 s (and nothing to send) -> deep sleep
#define SLEEP_LOAD_PERIOD_MS 9000   // during a load: 1 s sampling burst every ~10 s, light sleep in between
#define SLEEP_ANYMOTION_THR  0x60   // BMI270 any-motion threshold, 1 LSB = 0.48 mg -> ~46 mg
#define SLEEP_ANYMOTION_DUR  0x02   // 2 x 20 ms = 40 ms (short: fast alarm wake)

// ---- Machine-state display (M5StickS3) ----
#define DISPLAY_BRIGHTNESS    160   // 0-255 backlight (M5GFX PWM on G38)
#define DISPLAY_FLASH_MS      3000  // sleep build: colour flash on motion wake / button
#define DISPLAY_PERIOD_S_DEF  60    // sleep build, disp mode 1: periodic flash every N s

// ---- Cycle stall (PLACEHOLDERS - tune from test logs) ----
#define STALL_MS        90000UL          // vibration stopped > 90 s after cycle_start ...
#define MIN_CYCLE_MS    (10UL*60UL*1000UL) // ... and the cycle is shorter than 10 min -> cycle_stalled (0x15)
// ---- Data capture ----
#define LOG_HZ          50               // `log on` CSV rate (SAMPLE_HZ must be a multiple)

#define CAL_DRIFT_FLAG 0   // 1 = send cal:0 on angle drift (paused: vibration-only status)

// ---------- microphone (M5StickS3). db = dBFS+100, UNCALIBRATED. All need tuning on a real washer. ----------
#ifdef SLEEP_ENABLED
#define DEF_SND_ON           0     // battery: off by default (CFO)
#else
#define DEF_SND_ON           1     // 0 = mic never opened (server sndOn overrides, saved)
#endif
#define DEF_SND_TONE_RATIO   0.25f // Goertzel bin power / broadband power for a jingle note
#define DEF_SND_BUZZ_RATIO   0.10f // looser ratio for a harsh, harmonic-rich buzzer
#define DEF_SND_MIN_DB       45    // absolute level floor for any chime frame
#define DEF_SND_BEEP_MIN_MS  100
#define DEF_SND_BEEP_MAX_MS  800
#define DEF_SND_WIN_MS       5000  // >= 2 notes within this = generic jingle
#define DEF_SND_BUZZ_MS      1000  // sustained low-mid tone this long = generic buzzer
#define DEF_SND_BANG_DB      85    // peak level for a `bang` event (evidence only, never the siren)

#define RUN_MEAN_MIN 0.035f // g: mean rms over the hold (estimate, tune on a real washer)
#define RUN_CV_MAX 0.35f    // max stdev/mean of rms over the hold: machines are steady, typing is bursty (estimate)
#define RUN_MAX_RMS 0.60f   // g: above this during the hold = hand handling/pickup, not a machine (estimate, tune)
#define TILT_OPEN_DEG_DEF 20.0f
#define TILT_CLOSE_DEG_DEF 10.0f

// ---- Firmware version / OTA ----
#ifdef SLEEP_ENABLED
#define FW_BUILD "bat"
#else
#define FW_BUILD "usb"
#endif
#define FW_VERSION "0.9.0"          // reported as "fwv" in every post; server-driven OTA compares against this
#ifndef OTA_PASS
#define OTA_PASS ""                 // define OTA_PASS in secrets.h; empty => LAN OTA (ArduinoOTA) disabled with a serial warning
#endif
#define OTA_VERIFY_MS      300000UL // new image must get a 200 from the server within 5 min or it rolls back
#define OTA_RETRY_MS       3600000UL// same version retried at most once per hour
#define OTA_MAX_FAILS      3        // then give up until the server offers a different version
#define OTA_LAN_PORT       3232
