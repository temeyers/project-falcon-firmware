// Project Falcon laundry sensor - prototype firmware v0.3 (USB powered, no sleep)
// XIAO ESP32C3 + ADXL345/LSM6DS3, or M5StickC Plus2 (MPU6886). Posts HMAC-signed JSON to backend POST /v1/ingest.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <math.h>
#include "mbedtls/md.h"
#include "mbedtls/sha256.h"
#include <ArduinoOTA.h>
#include <Update.h>
#include "esp_ota_ops.h"
#include "secrets.h"   // before config.h so config.h #ifndef fallbacks (OTA_PASS) see the real values
#include "config.h"
#ifdef BOARD_M5STICKS3
#include <M5Unified.h>
#endif

Preferences prefs;
#ifdef SLEEP_ENABLED
extern bool sleepOn; extern int dispMode, dispPeriodS;
#endif
#ifdef BOARD_M5STICKS3
#define BTN_DOWN() (M5.update(), M5.BtnA.isPressed())
#else
#define BTN_DOWN() (digitalRead(PIN_BOOT_BTN) == LOW)
#endif

// ---------- tunables ----------
struct Tun {
  int mountDoor; float lidOpenDeg, lidCloseDeg; int lidHoldMs;
  float doorDeg; int doorWindowMs; float doorJerk;
  float runRms; int runHoldS; float quietRms; int doneQuietS;
  float movedJerk, movedDeg;
  float runMaxDeg, runDriftDeg; int runSettleS;
  int heartbeatMin; float gyroDoorDeg; int gyroOpenSign;
  int sndOn; float sndToneRatio, sndBuzzRatio; int sndMinDb, sndBeepMinMs, sndBeepMaxMs, sndWinMs, sndBuzzMs, sndBangDb;
} T;
struct TDesc { const char* k; char t; void* p; };
TDesc TD[] = {
  {"mount_door",'i',&T.mountDoor},{"lid_open_deg",'f',&T.lidOpenDeg},{"lid_close_deg",'f',&T.lidCloseDeg},
  {"lid_hold_ms",'i',&T.lidHoldMs},{"door_deg",'f',&T.doorDeg},{"door_window_ms",'i',&T.doorWindowMs},
  {"door_jerk",'f',&T.doorJerk},{"run_rms",'f',&T.runRms},{"run_hold_s",'i',&T.runHoldS},
  {"quiet_rms",'f',&T.quietRms},{"done_quiet_s",'i',&T.doneQuietS},{"moved_jerk",'f',&T.movedJerk},{"moved_deg",'f',&T.movedDeg},{"run_max_deg",'f',&T.runMaxDeg},{"run_drift_deg",'f',&T.runDriftDeg},{"run_settle_s",'i',&T.runSettleS},
  {"heartbeat_min",'i',&T.heartbeatMin},{"gyro_door_deg",'f',&T.gyroDoorDeg},{"gyro_open_sign",'i',&T.gyroOpenSign},
  {"snd_on",'i',&T.sndOn},{"snd_tone_ratio",'f',&T.sndToneRatio},{"snd_buzz_ratio",'f',&T.sndBuzzRatio},{"snd_min_db",'i',&T.sndMinDb},
  {"snd_beep_min_ms",'i',&T.sndBeepMinMs},{"snd_beep_max_ms",'i',&T.sndBeepMaxMs},{"snd_win_ms",'i',&T.sndWinMs},{"snd_buzz_ms",'i',&T.sndBuzzMs},{"snd_bang_db",'i',&T.sndBangDb}};
const int NTD = sizeof(TD)/sizeof(TD[0]);

void loadTun() {
  T = {DEF_MOUNT_DOOR, DEF_LID_OPEN_DEG, DEF_LID_CLOSE_DEG, DEF_LID_HOLD_MS, DEF_DOOR_DEG, DEF_DOOR_WINDOW_MS,
       DEF_DOOR_JERK, DEF_RUN_RMS, DEF_RUN_HOLD_S, DEF_QUIET_RMS, DEF_DONE_QUIET_S, DEF_MOVED_JERK, DEF_MOVED_DEG, DEF_RUN_MAX_DEG, DEF_RUN_DRIFT_DEG, DEF_RUN_SETTLE_S,
       (int)(HEARTBEAT_MS / 60000UL), DEF_GYRO_DOOR_DEG, DEF_GYRO_OPEN_SIGN,
       DEF_SND_ON, DEF_SND_TONE_RATIO, DEF_SND_BUZZ_RATIO, DEF_SND_MIN_DB, DEF_SND_BEEP_MIN_MS, DEF_SND_BEEP_MAX_MS, DEF_SND_WIN_MS, DEF_SND_BUZZ_MS, DEF_SND_BANG_DB};
  for (int i = 0; i < NTD; i++) {
    String k = String(TD[i].k).substring(0, 15);  // NVS key limit
    if (!prefs.isKey(k.c_str())) continue;
    if (TD[i].t == 'i') *(int*)TD[i].p = prefs.getInt(k.c_str());
    else *(float*)TD[i].p = prefs.getFloat(k.c_str());
  }
}
void saveTun() {
  for (int i = 0; i < NTD; i++) {
    String k = String(TD[i].k).substring(0, 15);
    if (TD[i].t == 'i') prefs.putInt(k.c_str(), *(int*)TD[i].p); else prefs.putFloat(k.c_str(), *(float*)TD[i].p);
  }
}
void dumpTun() {
  for (int i = 0; i < NTD; i++) {
    if (TD[i].t == 'i') Serial.printf("  %-15s %d\n", TD[i].k, *(int*)TD[i].p);
    else Serial.printf("  %-15s %.4f\n", TD[i].k, *(float*)TD[i].p);
  }
}

// ---------- IMU ----------
float gX = 0, gY = 0, gZ = 0;   // dps, all axes (for `log`); 0 on accel-only IMUs
#if defined(BOARD_M5STICKS3)
// BMI270 through M5Unified (handles init/config). gyro in dps, accel in g.
float gyroZ = 0;
bool axInit() { return M5.Imu.isEnabled(); }
bool axRead(float& x, float& y, float& z) {
  if (!M5.Imu.update()) { /* no new sample yet: reuse last */ }
  auto d = M5.Imu.getImuData();
  x = d.accel.x; y = d.accel.y; z = d.accel.z;
  gX = d.gyro.x; gY = d.gyro.y; gZ = d.gyro.z;
  gyroZ = (GYRO_DOOR_AXIS == 0 ? d.gyro.x : GYRO_DOOR_AXIS == 1 ? d.gyro.y : d.gyro.z); return true;
}
#elif defined(IMU_MPU6886)
// MPU6886 (M5StickC Plus2 onboard). Big-endian registers. Accel +-4 g, gyro +-500 dps.
void axWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU6886_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool axInit() {
  Wire.beginTransmission(MPU6886_ADDR); Wire.write(0x75); if (Wire.endTransmission(false)) return false;
  Wire.requestFrom(MPU6886_ADDR, 1); if (!Wire.available() || Wire.read() != 0x19) return false;
  axWrite(0x6B, 0x80); delay(50);   // PWR_MGMT_1 reset
  axWrite(0x6B, 0x01); delay(10);   // clock = PLL
  axWrite(0x1C, 0x08);              // ACCEL_CONFIG +-4 g
  axWrite(0x1B, 0x08);              // GYRO_CONFIG +-500 dps
  axWrite(0x1A, 0x01);              // CONFIG: DLPF
  axWrite(0x19, 0x09);              // SMPLRT_DIV -> 100 Hz
  return true;
}
float gyroZ = 0;  // dps
bool axRead(float& x, float& y, float& z) {
  Wire.beginTransmission(MPU6886_ADDR); Wire.write(0x3B); if (Wire.endTransmission(false)) return false;
  if (Wire.requestFrom(MPU6886_ADDR, 14) != 14) return false;
  int16_t v[7]; for (int i = 0; i < 7; i++) { uint8_t hi = Wire.read(), lo = Wire.read(); v[i] = (int16_t)(hi << 8 | lo); }
  const float s = 1.0f / 8192.0f; x = v[0]*s; y = v[1]*s; z = v[2]*s;   // v[3] = temperature
  gX = v[4] / 65.5f; gY = v[5] / 65.5f; gZ = v[6] / 65.5f;
  gyroZ = v[4 + GYRO_DOOR_AXIS] / 65.5f; return true;                    // 65.5 LSB/dps @500 dps
}
#elif defined(IMU_LSM6DS3)
// LSM6DS3/LSM6DSO (6-axis). Gyro z integrated for vertical-hinge door swing.
void axWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(LSM6_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool axInit() {
  Wire.beginTransmission(LSM6_ADDR); Wire.write(0x0F); if (Wire.endTransmission(false)) return false;
  Wire.requestFrom(LSM6_ADDR, 1); if (!Wire.available()) return false;
  uint8_t who = Wire.read(); if (who != 0x69 && who != 0x6A && who != 0x6C) return false;
  axWrite(0x10, 0x48);  // CTRL1_XL: 104 Hz, +-4 g
  axWrite(0x11, 0x44);  // CTRL2_G: 104 Hz, 500 dps
  axWrite(0x12, 0x44);  // CTRL3_C: BDU + auto-increment
  return true;
}
float gyroZ = 0;  // dps
bool axRead(float& x, float& y, float& z) {
  Wire.beginTransmission(LSM6_ADDR); Wire.write(0x22); if (Wire.endTransmission(false)) return false;
  if (Wire.requestFrom(LSM6_ADDR, 12) != 12) return false;
  int16_t v[6]; for (int i = 0; i < 6; i++) { uint8_t lo = Wire.read(), hi = Wire.read(); v[i] = (int16_t)(hi << 8 | lo); }
  gX = v[0] * 0.0175f; gY = v[1] * 0.0175f; gZ = v[2] * 0.0175f;
  gyroZ = v[GYRO_DOOR_AXIS] * 0.0175f;          // 17.5 mdps/LSB @500 dps
  const float s = 0.000122f; x = v[3]*s; y = v[4]*s; z = v[5]*s; return true;
}
#else
// ---------- ADXL345 (default) ----------
void axWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(ADXL345_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool axInit() {
  Wire.beginTransmission(ADXL345_ADDR); Wire.write(0x00); if (Wire.endTransmission(false)) return false;
  Wire.requestFrom(ADXL345_ADDR, 1); if (!Wire.available() || Wire.read() != 0xE5) return false;
  axWrite(0x31, 0x09);  // FULL_RES, +-4g
  axWrite(0x2C, 0x0A);  // 100 Hz ODR
  axWrite(0x2D, 0x08);  // measure
  return true;
}
bool axRead(float& x, float& y, float& z) {
  Wire.beginTransmission(ADXL345_ADDR); Wire.write(0x32); if (Wire.endTransmission(false)) return false;
  if (Wire.requestFrom(ADXL345_ADDR, 6) != 6) return false;
  int16_t v[3]; for (int i = 0; i < 3; i++) { uint8_t lo = Wire.read(), hi = Wire.read(); v[i] = (int16_t)(hi << 8 | lo); }
  const float s = 0.0039f;  // g/LSB full-res
  x = v[0]*s; y = v[1]*s; z = v[2]*s; return true;
}
#endif

// ---------- vector helpers ----------
struct V3 { float x, y, z; };
float vlen(V3 a) { return sqrtf(a.x*a.x + a.y*a.y + a.z*a.z); }
float vangle(V3 a, V3 b) {
  float d = (a.x*b.x + a.y*b.y + a.z*b.z) / (vlen(a)*vlen(b) + 1e-9f);
  d = fmaxf(-1.f, fminf(1.f, d)); return acosf(d) * 57.29578f;
}

// ---------- state ----------
V3 restG = {0,0,1}, grav = {0,0,1}, prevA = {0,0,1};
bool streamOn = false, armed = false, logOn = false;
uint32_t hbSec = HB_DEFAULT_S;                 // heartbeat period (s); server reply `hb` (30-3600) or set_heartbeat_hrs; NVS "hb_s"
uint32_t cycleStartMs = 0; bool stallSent = false;
uint32_t counter = 0;
// ack FIFO: one ack per message, oldest first
uint16_t ackq[8]; int ackHead = 0, ackTail = 0;
bool ackEmpty() { return ackHead == ackTail; }
void ackPush(uint16_t id) {
  for (int i = ackHead; i != ackTail; i = (i + 1) % 8) if (ackq[i] == id) return;
  int n = (ackTail + 1) % 8; if (n == ackHead) ackHead = (ackHead + 1) % 8; ackq[ackTail] = id; ackTail = n;
}
uint32_t txBackoffMs = 0, nextTxAt = 0;
enum Run { IDLE, VIBRATING, RUNNING, QUIETING } runSt = IDLE;
float holdSum = 0, holdSq = 0; uint32_t holdN = 0;
uint32_t runSince = 0, quietSince = 0, lastJerkMs = 0; float vibAngMin = 0, vibAngMax = 0;
bool openSt = false; uint32_t tiltSince = 0, untiltSince = 0;
uint32_t doorWinStart = 0; V3 doorWinG;
uint32_t lastMovedMs = 0;
// ---------- anti-theft siren ----------
// sirenAllowed: set only by an explicit `arm` command, cleared by disarm and by cycle_end (load done),
// so a stale reply flag a:1 after load-done never re-enables the siren.
enum Alarm { AL_IDLE, AL_SOUNDING } alSt = AL_IDLE;
#ifdef NET_CELL
#define HB_MIN_S 120   // CFO: cellular stays >=120 s, alarms still instant
#else
#define HB_MIN_S 5
#endif
bool sirenAllowed = false;
uint8_t sirenMode = 1;   // server-set: 0 off, 1 chirp ~3 s (default), 2 full; hard cap enforced below
#define SIREN_CHIRP_MS 3000
#define SIREN_CAP_MS   15000
uint32_t sirenLenMs() { uint32_t l = sirenMode == 0 ? 0 : sirenMode == 2 ? (uint32_t)ALARM_SOUND_MS : SIREN_CHIRP_MS; return l > SIREN_CAP_MS ? SIREN_CAP_MS : l; }
uint32_t armedAt = 0, graceUntil = 0, alarmStart = 0, alarmEnd = 0, lastDoorChange = 0;
uint32_t hitT[ALARM_CONFIRM_HITS]; int hitN = 0;
#ifdef BOARD_M5STICKS3
uint8_t s3Volume() {   // 100% only when USB power is detected, else 75%
  int16_t vbus = M5.Power.getVBUSVoltage();          // mV, -1 if the PMIC can't report it
  return (vbus > 4300) ? S3_VOL_USB : S3_VOL_BATTERY;
}
void buzzerInit() { M5.Speaker.begin(); M5.Speaker.setVolume(S3_VOL_BATTERY); }
void buzzer(bool on) { if (!on) M5.Speaker.stop(); }
#else
void buzzerInit() {
  if (PIN_BUZZER_EN >= 0) { pinMode(PIN_BUZZER_EN, OUTPUT); digitalWrite(PIN_BUZZER_EN, LOW); }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_BUZZER, BUZZER_FREQ_HZ, BUZZER_LEDC_RES); ledcWrite(PIN_BUZZER, 0);
#else
  ledcSetup(BUZZER_LEDC_CH, BUZZER_FREQ_HZ, BUZZER_LEDC_RES); ledcAttachPin(PIN_BUZZER, BUZZER_LEDC_CH); ledcWrite(BUZZER_LEDC_CH, 0);
#endif
}
void buzzer(bool on) {
  uint32_t d = on ? (1u << (BUZZER_LEDC_RES - 1)) : 0;   // 50% duty square wave at resonance
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_BUZZER, d);
#else
  ledcWrite(BUZZER_LEDC_CH, d);
#endif
}
#endif
void queueEvent(const char* t);
void alarmStop(const char* why) {
  if (alSt != AL_SOUNDING) return;
  alSt = AL_IDLE; buzzer(false); if (PIN_BUZZER_EN >= 0) digitalWrite(PIN_BUZZER_EN, LOW); alarmEnd = millis(); hitN = 0;
  Serial.printf("[ALARM] stop (%s)\n", why); queueEvent(why);
}
void setArmed(bool on, uint32_t now) {
  if (on) { if (!sirenAllowed) armedAt = now; sirenAllowed = true; }
  else { sirenAllowed = false; graceUntil = now + ALARM_DISARM_GRACE_MS; alarmStop("alarm_disarm"); }
}
// Called on every candidate lift/move jerk (never on lid/door tilt alone).
void alarmMotion(uint32_t now) {
  if (!armed || !sirenAllowed || alSt == AL_SOUNDING) return;
  if (now - armedAt < ALARM_EXIT_DELAY_MS || (int32_t)(graceUntil - now) > 0) return;
  if (alarmEnd && now - alarmEnd < ALARM_REARM_MS) return;
  if (lastDoorChange && now - lastDoorChange < ALARM_DOOR_GRACE_MS) return;   // lid/door slam, not a lift
  if (tiltSince || untiltSince || doorWinStart) { hitN = 0; return; }       // lid/door transition in progress
  if (hitN && now - hitT[0] > ALARM_CONFIRM_MS) hitN = 0;
  hitT[hitN++] = now;
  if (hitN < ALARM_CONFIRM_HITS) return;
  hitN = 0; alSt = AL_SOUNDING; alarmStart = now; if (PIN_BUZZER_EN >= 0) digitalWrite(PIN_BUZZER_EN, HIGH);
  Serial.println("[ALARM] siren ON"); queueEvent("alarm_start");
}
void spkReady();
void alarmTick(uint32_t now) {
  if (alSt != AL_SOUNDING) return;
  if (now - alarmStart >= sirenLenMs()) { alarmStop("alarm_timeout"); return; }
#ifdef BOARD_M5STICKS3
  // generated siren: 600->1500 Hz up-sweep every 1 s, 40 ms tone steps
  static uint32_t nextStep = 0;
  if ((int32_t)(now - nextStep) >= 0) { nextStep = now + 40;
    uint32_t ph = (now - alarmStart) % 1000; spkReady(); M5.Speaker.setVolume(s3Volume());
    M5.Speaker.tone(600 + ph * 9 / 10, 60); }
#else
  buzzer(((now - alarmStart) % (ALARM_BEEP_ON_MS + ALARM_BEEP_OFF_MS)) < ALARM_BEEP_ON_MS);
#endif
}
float hpSq[SAMPLE_HZ]; int hpIdx = 0; float rms = 0, jerk = 0, angle = 0;
uint32_t lastHb = 0;

// ---------- event queue ----------
char evq[EVENT_QUEUE_LEN][16]; uint32_t evArg[EVENT_QUEUE_LEN]; int evHead = 0, evTail = 0;   // each slot owns a copy; evArg = extra "el" seconds
void queueEventArg(const char* t, uint32_t arg) {
  int n = (evTail + 1) % EVENT_QUEUE_LEN; if (n == evHead) evHead = (evHead + 1) % EVENT_QUEUE_LEN;
  strlcpy(evq[evTail], t, sizeof(evq[0])); evArg[evTail] = arg; evTail = n;
  if (logOn) Serial.printf("E,%lu,%s,%lu\n", (unsigned long)millis(), t, (unsigned long)arg);
}
void queueEvent(const char* t) { queueEventArg(t, 0); }

extern int nsJingleStep;
// ---------- microphone (M5StickS3 ES8311 MEMS mic) ----------
// PRIVACY (hard rule): audio samples live only in one 32 ms RAM buffer that is overwritten by the next window.
// No audio is ever stored, logged, printed or sent. Only levels (db), flags (snd), tone matches (done_chime),
// bang peaks, and the learned fingerprint's tone features (bin, ms, level) leave the analysis code.
// db is dBFS + 100 (100 = full scale, ~30 = quiet room). UNCALIBRATED: relative only, not SPL.
// The mic and the speaker share the ES8311 codec + I2S: never both at once (spkReady()/micStart() swap them).
uint32_t cycleEndMs = 0;            // millis() of last cycle_end (chime window = 5 min after)
int sndDb = -1;                      // rolling level, -1 = no reading yet
uint32_t sndDbAt = 0, lastChimeMs = 0, lastBangMs = 0;
bool sndFlag = false;                // one-shot snd:1 on the next successful post
bool sndStream = false;
uint32_t spkBusyUntil = 0;           // button beeps: speaker owns the codec until then
#define SND_RATE 16000
#define SND_N 512                    // 32 ms window
#define SND_NB 16
static const uint16_t sndBinHz[SND_NB] = {250,315,400,500,630,800,1000,1250,1600,2000,2400,2700,3000,3200,3500,4000};
#define SND_BIN_1K 6                 // index of 1000 Hz: jingle notes >= this, buzzer fundamental <= this (+harmonics)
int sndTopBin = -1; float sndTopRatio = 0;
// learned fingerprint: up to 8 notes (bin, duration ms, level) + total ms + type. Tone features only, NVS "snd_fp".
struct SndNote { uint8_t bin; uint16_t ms; uint8_t lvl; };
struct SndFp { uint8_t n, type; uint16_t totalMs; SndNote note[8]; } sndFp = {0,0,0,{}};   // type 1 jingle, 2 buzzer
uint32_t sndLearnUntil = 0; SndFp sndLearn;
#ifdef BOARD_M5STICKS3
int16_t sndBuf[SND_N]; bool micOn = false, spkOn = true, micQueued = false;
void spkReady() {                    // call before any M5.Speaker.tone
  if (micOn) { M5.Mic.end(); micOn = false; micQueued = false; }
  if (!spkOn) { M5.Speaker.begin(); M5.Speaker.setVolume(s3Volume()); spkOn = true; }
}
bool micStart() {
  if (micOn) return true;
  if (spkOn) { M5.Speaker.end(); spkOn = false; }
  auto c = M5.Mic.config(); c.sample_rate = SND_RATE; M5.Mic.config(c);
  micOn = M5.Mic.begin(); if (!micOn) { Serial.println("[SND] mic begin failed"); spkReady(); }
  return micOn;
}
void micStop() { if (micOn) { M5.Mic.end(); micOn = false; micQueued = false; } spkReady(); }
void beep(uint16_t f, uint16_t ms) { spkReady(); M5.Speaker.setVolume(s3Volume()); M5.Speaker.tone(f, ms); spkBusyUntil = millis() + ms + 60; }
#else
void spkReady() {} void beep(uint16_t, uint16_t) {}
#endif
// live note segmentation
struct SndSeg { uint8_t bin; uint16_t ms; uint8_t lvl; uint32_t endMs; };
SndSeg segs[8]; int segN = 0;                  // most recent closed notes (oldest first)
int curBin = -1; uint32_t curStart = 0, curLast = 0; uint8_t curLvl = 0; int lowRunMs = 0, lowHist[SND_NB];
void segPush(const SndSeg& s) { if (segN == 8) { memmove(segs, segs+1, 7*sizeof(SndSeg)); segN = 7; } segs[segN++] = s; }
bool chimeWindow(uint32_t now) { return runSt == QUIETING || (cycleEndMs && now - cycleEndMs < 300000UL); }
void chimeHit(uint32_t now, const char* why) {
  if (lastChimeMs && now - lastChimeMs < 30000) return;
  lastChimeMs = now; sndFlag = true; queueEvent("done_chime");
  Serial.printf("[SND] done_chime (%s)\n", why);   // the chime never touches runSt: the server decides Done
}
bool fpMatchJingle(uint32_t now) {
  if (sndFp.type != 1 || sndFp.n < 2 || segN < 2) return false;
  int n = sndFp.n < segN ? sndFp.n : segN, ok = 0;
  if (now - segs[segN-n].endMs > (uint32_t)sndFp.totalMs * 2 + 2000) return false;
  for (int i = 0; i < n; i++) { const SndSeg& s = segs[segN-n+i]; const SndNote& f = sndFp.note[sndFp.n-n+i];
    if (abs((int)s.bin - (int)f.bin) <= 1 && s.ms * 2 >= f.ms && s.ms <= f.ms * 2 + 64) ok++; }
  return n >= 2 && ok * 10 >= sndFp.n * 7;
}
void genericJingle(uint32_t now) {
  int c = 0;
  for (int i = 0; i < segN; i++) if (now - segs[i].endMs <= (uint32_t)T.sndWinMs && segs[i].bin >= SND_BIN_1K - 1 &&
      segs[i].ms >= T.sndBeepMinMs && segs[i].ms <= T.sndBeepMaxMs) c++;
  if (c >= 2) { chimeHit(now, "jingle"); segN = 0; }
}
void segClose(uint32_t now) {
  if (curBin < 0) return;
  uint16_t ms = curLast - curStart + SND_N * 1000 / SND_RATE;
  if (ms >= 60) {
    SndSeg s = {(uint8_t)curBin, ms, curLvl, curLast}; segPush(s);
    if (sndLearnUntil && sndLearn.n < 8) { sndLearn.note[sndLearn.n++] = {s.bin, s.ms, s.lvl}; }
    else if (!sndLearnUntil && chimeWindow(now)) { if (fpMatchJingle(now)) { chimeHit(now, "learned"); segN = 0; } else genericJingle(now); }
  }
  curBin = -1;
}
// analyse one 32 ms window (samples are discarded after this function)
void sndAnalyse(const int16_t* x, int n, uint32_t now) {
  float mean = 0; for (int i = 0; i < n; i++) mean += x[i]; mean /= n;
  float e = 0, pk = 0; for (int i = 0; i < n; i++) { float v = x[i] - mean; e += v*v; if (fabsf(v) > pk) pk = fabsf(v); }
  float rmsv = sqrtf(e / n) + 1e-3f;
  int db = (int)lroundf(20.0f * log10f(rmsv / 32768.0f) + 100.0f); if (db < 0) db = 0;
  int pkDb = (int)lroundf(20.0f * log10f((pk + 1e-3f) / 32768.0f) + 100.0f); if (pkDb < 0) pkDb = 0;
  sndDb = sndDb < 0 ? db : (sndDb * 7 + db + 4) / 8; sndDbAt = now;
  // bang: evidence only. Never calls alarmMotion, never sounds the siren.
  if (armed && pkDb >= T.sndBangDb && (!lastBangMs || now - lastBangMs >= 10000)) { lastBangMs = now; queueEventArg("bang", (uint32_t)pkDb); Serial.printf("[SND] bang %d\n", pkDb); }
  // Goertzel at SND_NB bins; ratio ~1 for a pure tone on a bin, ~2/N for white noise
  int best = -1; float bestR = 0, norm = e * n / 2 + 1.0f;
  for (int b = 0; b < SND_NB; b++) {
    float w = 2.0f * (float)M_PI * sndBinHz[b] / SND_RATE, k = 2.0f * cosf(w), s1 = 0, s2 = 0;
    for (int i = 0; i < n; i++) { float s0 = (x[i] - mean) + k * s1 - s2; s2 = s1; s1 = s0; }
    float p = s1*s1 + s2*s2 - k*s1*s2, r = p / norm; if (r > bestR) { bestR = r; best = b; } }
  sndTopBin = best; sndTopRatio = bestR;
  bool loud = db >= T.sndMinDb, tonal = loud && bestR >= T.sndToneRatio, buzzy = loud && bestR >= T.sndBuzzRatio && best <= SND_BIN_1K + 2;   // fundamental/harmonics up to 1.6 kHz
  // note segmentation (+-1 bin = same note)
  if (tonal && curBin >= 0 && abs(best - curBin) <= 1 && now - curLast < 100) { curLast = now; if (db > curLvl) curLvl = db; }
  else { segClose(now); if (tonal) { curBin = best; curStart = curLast = now; curLvl = db; } }
  if (curBin >= 0 && now - curLast >= 100) segClose(now);
  // buzzer: sustained harsh low-mid band; top bin may hop between harmonics
  static uint32_t lowLast = 0;
  if (buzzy) { if (now - lowLast > 100) { lowRunMs = 0; memset(lowHist, 0, sizeof(lowHist)); lowLast = now; } lowRunMs += now - lowLast; lowLast = now; if (lowRunMs > 60000) lowRunMs = 60000; lowHist[best]++; }
  else if (now - lowLast > 100) {
    lowRunMs = 0; }
  if (!sndLearnUntil && chimeWindow(now) && lowRunMs > 0) {
    int dom = 0; for (int b = 1; b < SND_NB; b++) if (lowHist[b] > lowHist[dom]) dom = b;
    if (sndFp.type == 2 && sndFp.n && abs(dom - sndFp.note[0].bin) <= 1 && lowRunMs * 10 >= sndFp.totalMs * 7) { chimeHit(now, "learned buzzer"); lowRunMs = 0; }
    else if (lowRunMs >= T.sndBuzzMs) { chimeHit(now, "buzzer"); lowRunMs = 0; } }
  if (sndLearnUntil && lowRunMs > (int)sndLearn.totalMs) sndLearn.totalMs = lowRunMs;   // longest buzz seen
  if (sndStream) { static uint32_t ls = 0; if (now - ls >= 250) { ls = now;
      Serial.printf("snd db=%d pk=%d top=%uHz ratio=%.2f%s\n", db, pkDb, best >= 0 ? sndBinHz[best] : 0, bestR, tonal ? " TONE" : buzzy ? " BUZZ" : ""); } }
}
void sndLearnFinish() {
  sndLearnUntil = 0; segClose(millis());
  SndFp f = sndLearn;
  int jn = 0; uint16_t longest = 0; for (int i = 0; i < f.n; i++) { if (f.note[i].bin >= SND_BIN_1K - 1) jn++; if (f.note[i].ms > longest) longest = f.note[i].ms; }
  if (f.totalMs >= 1000 && f.totalMs >= longest) {   // sustained low band = buzzer; note[0] = dominant bin
    f.type = 2; int dom = 0; for (int b = 1; b < SND_NB; b++) if (lowHist[b] > lowHist[dom]) dom = b;
    if (f.n == 0) { f.n = 1; f.note[0] = {(uint8_t)dom, f.totalMs, 0}; } else f.note[0].bin = dom; }
  else { f.type = 1; uint32_t t = 0; for (int i = 0; i < f.n; i++) t += f.note[i].ms; f.totalMs = t; }
  if (f.n == 0) { Serial.println("[SND] learn: nothing heard (check `snd` levels / snd_min_db)"); return; }
  sndFp = f; prefs.putBytes("snd_fp", &sndFp, sizeof(sndFp));
  Serial.printf("[SND] learned %s, %u notes, total %ums (saved snd_fp)\n", f.type == 2 ? "BUZZER" : "JINGLE", f.n, f.totalMs);
  for (int i = 0; i < f.n; i++) Serial.printf("  %d: %uHz %ums lvl=%u\n", i, sndBinHz[f.note[i].bin], f.note[i].ms, f.note[i].lvl);
}
void sndLoad() { if (prefs.getBytesLength("snd_fp") == sizeof(sndFp)) prefs.getBytes("snd_fp", &sndFp, sizeof(sndFp)); if (sndFp.n > 8) sndFp.n = 0; }
// called every loop (S3 only does work)
void sndTick(uint32_t now) {
#ifdef BOARD_M5STICKS3
  if (sndLearnUntil && (int32_t)(now - sndLearnUntil) >= 0) sndLearnFinish();
  bool busy = alSt == AL_SOUNDING || nsJingleStep >= 0 || (int32_t)(spkBusyUntil - now) > 0 || M5.Speaker.isPlaying();
  bool want = T.sndOn && (chimeWindow(now) || armed || sndStream || sndLearnUntil);
#ifdef SLEEP_ENABLED
  // battery: continuous only in QUIETING (already awake) / learn / stream; else 64 ms every 1 s while awake
  bool cont = runSt == QUIETING || sndLearnUntil || sndStream;
  if (want && !cont) { uint32_t ph = now % 1000; want = ph < 64; }
#else
  if (T.sndOn) want = true;   // USB: always listen for the db level
#endif
  if (busy || !want) { if (micOn) micStop(); return; }
  if (!micOn && !micStart()) return;
  if (micQueued && M5.Mic.isRecording()) return;
  if (micQueued) sndAnalyse(sndBuf, SND_N, now);      // buffer is reused next window: no audio kept
  micQueued = M5.Mic.record(sndBuf, SND_N, SND_RATE);
#else
  (void)now;
#endif
}

bool calOk = true; uint32_t driftSince = 0;
bool tiltOn = false;   // server 'tilt' (default off): lid/door logic only when on
void calibrate() {
  Serial.println("[CAL] keep still...");
  V3 s = {0,0,0}; int n = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) { float x,y,z; if (axRead(x,y,z)) { s.x+=x; s.y+=y; s.z+=z; n++; } delay(1000/SAMPLE_HZ); }
  if (!n) { Serial.println("[CAL] no accel data"); return; }
  restG = {s.x/n, s.y/n, s.z/n}; grav = restG; prevA = restG;
  openSt = false; doorWinStart = 0;
  { float r[3] = {restG.x, restG.y, restG.z}; prefs.putBytes("rest_g", r, sizeof(r)); }
  if (!calOk) queueEvent("cal_restored"); calOk = true; driftSince = 0; prefs.putBool("cal_ok", true);
  Serial.printf("[CAL] rest g = %.3f %.3f %.3f (|g|=%.3f)\n", restG.x, restG.y, restG.z, vlen(restG));
}

// boot: keep the saved calibration (a power cycle must not hide a tilt); first boot ever calibrates
void bootCal() {
  float r[3]; if (prefs.getBytes("rest_g", r, sizeof(r)) != sizeof(r)) { calibrate(); return; }
  restG = {r[0], r[1], r[2]};
  V3 s = {0,0,0}; int n = 0;
  for (int i = 0; i < 20; i++) { float x,y,z; if (axRead(x,y,z)) { s.x+=x; s.y+=y; s.z+=z; n++; } delay(1000/SAMPLE_HZ); }
  if (n) { grav = {s.x/n, s.y/n, s.z/n}; prevA = grav; angle = vangle(grav, restG); }
  Serial.printf("[CAL] kept saved rest g, boot angle %.1f deg\n", angle);
  if (CAL_DRIFT_FLAG && angle > T.runMaxDeg && calOk) { calOk = false; prefs.putBool("cal_ok", false); queueEventArg("cal_drift", (uint32_t)angle); }
}

// ---------- detection, called at SAMPLE_HZ ----------
void processSample(V3 a, uint32_t now) {
  grav.x += GRAVITY_LPF_ALPHA*(a.x-grav.x); grav.y += GRAVITY_LPF_ALPHA*(a.y-grav.y); grav.z += GRAVITY_LPF_ALPHA*(a.z-grav.z);
  V3 hp = {a.x-grav.x, a.y-grav.y, a.z-grav.z};
  hpSq[hpIdx] = hp.x*hp.x + hp.y*hp.y + hp.z*hp.z; hpIdx = (hpIdx + 1) % SAMPLE_HZ;
  float s = 0; for (int i = 0; i < SAMPLE_HZ; i++) s += hpSq[i]; rms = sqrtf(s / SAMPLE_HZ);
  jerk = vlen({a.x-prevA.x, a.y-prevA.y, a.z-prevA.z}) * SAMPLE_HZ; prevA = a;
  angle = vangle(grav, restG);
  bool running = (runSt == RUNNING || runSt == QUIETING);
  if (jerk > T.movedJerk && !running) lastJerkMs = now;
  // drift: tilted past run_max_deg from calibration for 60 s (lid closed, not running) -> cal:0, never auto-recal
  if (CAL_DRIFT_FLAG && angle > T.runMaxDeg && !openSt && !running) { if (!driftSince) driftSince = now; else if (calOk && now - driftSince >= 60000) {
      calOk = false; prefs.putBool("cal_ok", false); queueEventArg("cal_drift", (uint32_t)angle); Serial.printf("[CAL] drift %.1f deg -> cal:0\n", angle); } }
  else driftSince = 0;
  bool stableNow = angle <= T.runMaxDeg && !openSt;   // vibration-only: jerk no longer blocks a start (washer vibration is jerky)

  // vibration / running / load done
  switch (runSt) {
    case IDLE: if (rms > T.runRms && stableNow) { runSt = VIBRATING; runSince = now; holdSum = holdSq = 0; holdN = 0; vibAngMin = vibAngMax = angle; } break;
    case VIBRATING:
      if (angle < vibAngMin) vibAngMin = angle; if (angle > vibAngMax) vibAngMax = angle;
      holdSum += rms; holdSq += rms*rms; holdN++;
      if (rms < T.runRms || rms > RUN_MAX_RMS || !stableNow || vibAngMax - vibAngMin > T.runDriftDeg) runSt = IDLE;   // any gap or handling spike restarts the 10 s hold   // handling, not a machine
      else if (now - runSince >= (uint32_t)T.runHoldS*1000) {
        float m = holdSum / holdN, sd = sqrtf(fmaxf(holdSq / holdN - m*m, 0)), cv = m > 0 ? sd / m : 9;
        if (m < RUN_MEAN_MIN || cv > RUN_CV_MAX) { Serial.printf("[RUN] hold rejected mean=%.3f cv=%.2f\n", m, cv); runSt = IDLE; break; }
        Serial.printf("[RUN] hold ok mean=%.3f cv=%.2f\n", m, cv); runSt = RUNNING; cycleStartMs = runSince; stallSent = false; queueEvent("cycle_start"); }
      break;
    case RUNNING: if (rms < T.quietRms) { runSt = QUIETING; quietSince = now; } break;
    case QUIETING:
      if (rms > T.runRms) runSt = RUNNING;
      else if (!stallSent && now - quietSince >= (uint32_t)STALL_MS && now - cycleStartMs < (uint32_t)MIN_CYCLE_MS) {
        stallSent = true; queueEventArg("cycle_stalled", (now - cycleStartMs) / 1000); }   // stays QUIETING: resumes or ends normally
      else if (now - quietSince >= (uint32_t)T.doneQuietS*1000) { runSt = IDLE; cycleEndMs = now ? now : 1; queueEvent("cycle_end"); sirenAllowed = false; alarmStop("alarm_done"); }
      break;
  }

  float lidO = T.lidOpenDeg >= 90 ? TILT_OPEN_DEG_DEF : T.lidOpenDeg, lidC = T.lidCloseDeg >= 90 ? TILT_CLOSE_DEG_DEF : T.lidCloseDeg;
  if (!tiltOn) { if (openSt) { openSt = false; lastDoorChange = now; } tiltSince = untiltSince = 0; doorWinStart = 0; }   // vibration-only
  else if (!T.mountDoor) {
    // lid: tilt from rest held
    if (!openSt) {
      if (angle > lidO) { if (!tiltSince) tiltSince = now; if (now - tiltSince >= (uint32_t)T.lidHoldMs) { openSt = true; lastDoorChange = now; hitN = 0; queueEvent("door_open"); } }
      else tiltSince = 0;
    } else {
      if (angle < lidC) { if (!untiltSince) untiltSince = now; if (now - untiltSince >= (uint32_t)T.lidHoldMs) { openSt = false; lastDoorChange = now; queueEvent("door_close"); } }
      else untiltSince = 0;
    }
  } else {
#if defined(IMU_LSM6DS3) || defined(IMU_MPU6886) || defined(BOARD_M5STICKS3)
    // vertical door: integrate gyro about the hinge axis over a sliding window
    static float gyroBias = 0; static float rot = 0; static uint32_t rotStart = 0;
    if (runSt == IDLE && fabsf(gyroZ - gyroBias) < 3) gyroBias += 0.001f * (gyroZ - gyroBias);  // slow bias tracking at rest
    float w = gyroZ - gyroBias;
    if (fabsf(w) > 5) { if (!rotStart) { rotStart = now; rot = 0; } rot += w / SAMPLE_HZ; }
    if (rotStart && now - rotStart > (uint32_t)T.doorWindowMs) { rotStart = 0; rot = 0; }
    if (rotStart && fabsf(rot) > T.gyroDoorDeg && !running) {
      bool opening = (rot > 0) == (T.gyroOpenSign > 0);
      if (opening != openSt) { openSt = opening; queueEvent(openSt ? "door_open" : "door_close"); }
      rotStart = 0; rot = 0;
    }
#else
    // vertical door: jerk spike opens a window; gravity direction change > door_deg inside it = swing.
    if (!doorWinStart && jerk > T.doorJerk && !running) { doorWinStart = now; doorWinG = grav; }
    if (doorWinStart) {
      if (vangle(grav, doorWinG) > T.doorDeg) {
        openSt = !openSt; queueEvent(openSt ? "door_open" : "door_close"); doorWinStart = 0;
      } else if (now - doorWinStart > (uint32_t)T.doorWindowMs) doorWinStart = 0;
    }
    // re-closing: back near rest pose
    if (openSt && angle < lidC && !doorWinStart) { if (!untiltSince) untiltSince = now;
      if (now - untiltSince >= (uint32_t)T.lidHoldMs) { openSt = false; lastDoorChange = now; queueEvent("door_close"); } } else untiltSince = 0;
#endif
  }

  // moved: big non-periodic jerk while not running (rate-limited 10 s)
  if (runSt == IDLE && jerk > T.movedJerk) {   // any vibration state (incl. the 30 s start hold) never alarms
    alarmMotion(now);                        // debounced; siren only if armed (lid tilt alone never gets here)
    if (now - lastMovedMs > 10000) { lastMovedMs = now; queueEvent("tamper"); }
  }
}

// ---------- networking ----------
String hmacHex(const String& body) {
  uint8_t key[16]; for (int i = 0; i < 16; i++) { char h[3] = {SIGNING_KEY[2*i], SIGNING_KEY[2*i+1], 0}; key[i] = strtoul(h, nullptr, 16); }
  uint8_t out[32]; mbedtls_md_context_t c; mbedtls_md_init(&c);
  mbedtls_md_setup(&c, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&c, key, 16); mbedtls_md_hmac_update(&c, (const uint8_t*)body.c_str(), body.length());
  mbedtls_md_hmac_finish(&c, out); mbedtls_md_free(&c);
  String s; char b[3]; for (int i = 0; i < 32; i++) { sprintf(b, "%02x", out[i]); s += b; } return s;
}

String wifiSsid = WIFI_SSID, wifiPass = WIFI_PASS;
volatile bool resyncPending = false; volatile uint8_t wifiLastReason = 0; volatile uint32_t wifiDiscN = 0; bool wifiEvtOn = false;
void wifiConnect() {
  if (!wifiEvtOn) { wifiEvtOn = true;
    WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t info) { wifiLastReason = info.wifi_sta_disconnected.reason; wifiDiscN++;
      Serial.printf("[WIFI] disconnected reason=%u (2/15=auth/wrong pass, 201=no AP found, 202=auth fail, 203=assoc fail)\n", wifiLastReason); },
      ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t info) { resyncPending = true; }, ARDUINO_EVENT_WIFI_STA_GOT_IP); }
  if (WiFi.status() == WL_CONNECTED) return;
  static uint32_t lastTry = 0; if (lastTry && millis() - lastTry < 10000) return; lastTry = millis();
  Serial.printf("[WIFI] connecting to %s\n", wifiSsid.c_str());
  WiFi.mode(WIFI_STA); WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
}

uint16_t appliedIds[16]; int appliedIdx = 0;  // ids already applied (server resends until acked)
bool alreadyApplied(uint16_t id) { for (int i = 0; i < 16; i++) if (appliedIds[i] == id) return true; return false; }

// ---------- machine state display (M5StickS3: ST7789P3 135x240, backlight G38 via M5GFX) ----------
// st: 0 = available (GREEN), 1 = claimed (RED), 2 = out of order (AMBER). Cmd 0x06 set_state; every reply's "st" is the truth.
uint8_t machineState = 0;
int64_t doneAtS = 0; bool unlPending = false;   // st 3: count-up since done; short press -> unl:1
static int64_t clkS() { struct timeval tv; gettimeofday(&tv, nullptr); return tv.tv_sec; }
String doneFor() { int64_t d = clkS() - doneAtS; if (d < 0) d = 0; uint32_t m = d / 60;
  char b[16]; if (m < 60) snprintf(b, sizeof b, "%lum", (unsigned long)m); else snprintf(b, sizeof b, "%luh %02lum", (unsigned long)(m/60), (unsigned long)(m%60)); return String(b); }
bool nsOn = false; uint32_t nsSince = 0, nsJingleAt = 0; int nsJingleStep = -1;   // ns:1 = claimed, never started: flash + one jingle
#ifdef BOARD_M5STICKS3
uint32_t flashUntil = 0;
void showState(bool on) {
  if (!on) { M5.Display.setBrightness(0); M5.Display.sleep(); return; }
  static const uint16_t col[4] = {0x07E0, 0xF800, 0x8410, 0x001F};
  static const char* txt[4] = {"FREE", "GUARDED", "OUT OF SERVICE", "DONE"};
  uint8_t st = machineState > 3 ? 2 : machineState;
  M5.Display.wakeup(); M5.Display.setRotation(1); M5.Display.fillScreen(col[st]);
  M5.Display.setTextColor(st == 0 ? 0x0000 : 0xFFFF); M5.Display.setTextDatum(middle_center); M5.Display.setTextSize(st == 2 ? 2 : 3);
  if (st == 3) { M5.Display.setTextSize(3); M5.Display.drawString("DONE", M5.Display.width()/2, M5.Display.height()/2 - 22);
    M5.Display.setTextSize(2); M5.Display.drawString(doneFor(), M5.Display.width()/2, M5.Display.height()/2 + 14);
    M5.Display.setTextSize(1); M5.Display.drawString("press to clear", M5.Display.width()/2, M5.Display.height() - 12); }
  else M5.Display.drawString(txt[st], M5.Display.width()/2, M5.Display.height()/2);
  M5.Display.setBrightness(DISPLAY_BRIGHTNESS);
}
void flashState(uint32_t ms) { showState(true); flashUntil = millis() + ms; }
#endif
void setMachineState(int st) {
  if (st < 0 || st > 3 || st == machineState) return;
  if (st == 3) { doneAtS = clkS(); prefs.putLong64("done_t", doneAtS); }
  machineState = st; prefs.putUChar("st", machineState);
  Serial.printf("[STATE] machine state -> %d\n", machineState);
  if (machineState == 2) { armed = false; if (sirenAllowed) setArmed(false, millis()); alarmStop("alarm_disarm"); }   // out of service wins: disarm, silence
#ifdef BOARD_M5STICKS3
#ifdef SLEEP_ENABLED
  flashState(DISPLAY_FLASH_MS);
#else
  showState(true);
#endif
#endif
}

// Local arm toggle (button): the reply flag `a` is ignored until the server has accepted (HTTP 200) the matching
// armed_local/disarmed_local event; after that the server's `a` wins again. Explicit arm/disarm commands always win.
bool pendingLocalArm = false; const char* pendingLocalType = ""; uint32_t pendingLocalAt = 0;
#define LOCAL_ARM_PENDING_MAX_MS (5UL * 60UL * 1000UL)   // give up waiting after 5 min (event lost/rejected)
void localArmToggle(uint32_t now) {
  armed = !armed; setArmed(armed, now);
  pendingLocalArm = true; pendingLocalType = armed ? "armed_local" : "disarmed_local"; pendingLocalAt = now;
  Serial.printf("[BTN] %s (local, waiting for server ack)\n", armed ? "ARMED" : "disarmed");
  queueEvent(pendingLocalType);
}
void otaOffer(const String& r);
void handleReply(const String& r) {
  otaOffer(r);
  // {"ok":true,"c":N,"a":0|1,"cmds":[{"cmdId":N,"type":"arm","arg":N}]} - tiny parser, no JSON lib
  if (pendingLocalArm && millis() - pendingLocalAt > LOCAL_ARM_PENDING_MAX_MS) { pendingLocalArm = false; Serial.println("[ARM] local toggle never acked; following server"); }
  { int hp = r.indexOf("\"hb\":"); if (hp >= 0) { long v = r.substring(hp+5).toInt();
      if (v > 0) { v = v < HB_MIN_S ? HB_MIN_S : v > 3600 ? 3600 : v; if ((uint32_t)v != hbSec) { hbSec = v; prefs.putUInt("hb_s", hbSec); Serial.printf("[HB] heartbeat %lus (server)\n", (unsigned long)hbSec); } } } }
  { int sm = r.indexOf("\"sn\":"); if (sm >= 0) { long v = r.substring(sm+5).toInt(); uint8_t m = (v >= 0 && v <= 2) ? (uint8_t)v : 1;
      if (m != sirenMode) { sirenMode = m; prefs.putUChar("siren_m", m); Serial.printf("[SIREN] mode %u (server)\n", m); } } }
  { int sp = r.indexOf("\"st\":"); if (sp >= 0) { int v = r.substring(sp+5).toInt(); if (!(unlPending && v == 3)) setMachineState(v); } }
  { int tp = r.indexOf("\"tilt\":"); if (tp >= 0) { bool v = r.charAt(tp+7) == '1'; if (v != tiltOn) { tiltOn = v; prefs.putBool("tilt", v); Serial.printf("[TILT] %s (server)\n", v ? "on" : "off"); } } }
  { int lp = r.indexOf("\"load\":\""); if (lp >= 0) { int door = r.charAt(lp+8) == 'f' ? 1 : 0; /* "f"/"t" or "front"/"top" */ if (door != T.mountDoor) { T.mountDoor = door; saveTun(); Serial.printf("[TILT] load %s (server)\n", door ? "front" : "top"); } } }
  { int np = r.indexOf("\"ns\":"); bool v = np >= 0 && r.charAt(np+5) == '1';
    if (v && !nsOn && runSt != RUNNING && runSt != QUIETING) { nsOn = true; nsSince = millis(); nsJingleStep = 0; nsJingleAt = millis(); Serial.println("[NS] never started: flash + jingle"); }
    else if (!v && nsOn) { nsOn = false; Serial.println("[NS] cleared"); } }
  { int so = r.indexOf("\"sndOn\":"); if (so >= 0) { int v = r.charAt(so+8) == '1' ? 1 : 0;
      if (v != T.sndOn) { T.sndOn = v; prefs.putInt("snd_on", v); Serial.printf("[SND] mic %s (server)\n", v ? "on" : "off"); } } }
  int a = r.indexOf("\"a\":"); if (a >= 0 && !pendingLocalArm) { bool na = r.charAt(a+4) == '1' && machineState != 2;
    if (na && !armed) { armed = true; setArmed(true, millis()); }          // claim-arm: starts the 30 s exit delay; lift/move only, door/lid never alarms
    else { armed = na; if (!armed && sirenAllowed) setArmed(false, millis()); } }
  int p = 0;
  while ((p = r.indexOf("\"cmdId\":", p)) >= 0) {
    uint16_t id = r.substring(p+8).toInt();
    int tp = r.indexOf("\"type\":\"", p); int te = r.indexOf('"', tp+8);
    String type = r.substring(tp+8, te);
    int ap = r.indexOf("\"arg\":", p); long arg = ap >= 0 ? r.substring(ap+6).toInt() : 0;
    p = te;
    if (alreadyApplied(id)) { ackPush(id); continue; }
    bool ok = true;
    int code = type == "arm" ? CMD_ARM : type == "disarm" ? CMD_DISARM : type == "set_heartbeat_hrs" ? CMD_SET_HEARTBEAT_HRS :
               type == "reboot" ? CMD_REBOOT : type == "silence_alarm" ? CMD_SILENCE_ALARM : type == "set_state" ? CMD_SET_STATE : 0;
    (void)code;  // names are authoritative on the JSON path; codes documented for the binary path
    if (type == "arm" || type == "disarm") pendingLocalArm = false;   // newer server command wins
    if (type == "arm") { armed = true; setArmed(true, millis()); }
    else if (type == "disarm") { armed = false; setArmed(false, millis()); }
    else if (type == "set_state" && arg >= 0 && arg <= 3) setMachineState(arg);
    else if (type == "recalibrate") { if (runSt == IDLE) calibrate(); else ok = false; }   // owner one-tap; refused mid-cycle
    else if (type == "silence_alarm") { graceUntil = millis() + ALARM_DISARM_GRACE_MS; alarmStop("alarm_silenced"); }
    else if (type == "set_heartbeat_hrs" && arg >= 1 && arg <= 168) {
      hbSec = arg * 3600UL; prefs.putUInt("hb_s", hbSec);
    }
    else if (type == "reboot") {
      prefs.putInt("ack", id); Serial.println("[CMD] reboot"); delay(200); ESP.restart();
    }
    else ok = false;
    Serial.printf("[CMD] #%u %s arg=%ld -> %s\n", id, type.c_str(), arg, ok ? "applied" : "rejected (no ack)");
    if (ok) { appliedIds[appliedIdx] = id; appliedIdx = (appliedIdx + 1) % 16; ackPush(id); }
  }
}

// flags (v=3): only bit2 (0x04) = charging is defined by the backend contract. USB-powered prototype has no
// battery, so charging is reported as 0 ("unknown/not charging"). There is no armed bit; armed comes from reply `a`.
#define FLAG_CHARGING 0x04
uint8_t currentFlags() { return 0; }

void backoff(uint32_t ms) {
  // cap retries at one heartbeat (5-60 s) so the puck re-posts within ~1 hb after the server comes back
  uint32_t cap = hbSec * 1000UL; cap = cap < 5000 ? 5000 : cap > 60000 ? 60000 : cap;
  txBackoffMs = ms ? min<uint32_t>(ms, 300000) : (txBackoffMs ? min<uint32_t>(txBackoffMs * 2, cap) : min<uint32_t>(2000, cap));
  nextTxAt = millis() + txBackoffMs;
  Serial.printf("[HTTP] backing off %lu ms\n", (unsigned long)txBackoffMs);
}


// ---------- server address cache: resolve .local once, re-resolve only after a failed post ----------
IPAddress srvIp; bool srvIpOk = false; bool mdnsUp = false;
String srvUrl() {
  String u = String(SERVER_URL);
  if (u.startsWith("https")) return u;   // TLS keeps the hostname (cert/SNI)
  int hs = u.indexOf("://") + 3, he = u.indexOf(':', hs); int sl = u.indexOf('/', hs);
  if (he < 0 || (sl >= 0 && sl < he)) he = sl < 0 ? u.length() : sl;
  String host = u.substring(hs, he);
  if (!srvIpOk) {
    uint32_t t0 = millis(); IPAddress ip;
    if (host.endsWith(".local")) { if (!mdnsUp) mdnsUp = MDNS.begin(DEVICE_ID); ip = MDNS.queryHost(host.substring(0, host.length()-6), 2000); }
    else WiFi.hostByName(host.c_str(), ip);
    uint32_t dt = millis() - t0;
    if (ip == IPAddress(0,0,0,0)) { Serial.printf("[DNS] resolve %s FAILED after %lums\n", host.c_str(), (unsigned long)dt); return u; }
    srvIp = ip; srvIpOk = true; Serial.printf("[DNS] %s -> %s in %lums (cached)\n", host.c_str(), ip.toString().c_str(), (unsigned long)dt);
  }
  return u.substring(0, hs) + srvIp.toString() + u.substring(he);
}
// returns true if delivered (or permanently rejected) -> dequeue
uint32_t curEvArg = 0;
void otaPostOk();
bool postEvent(const char* type) {
  if (WiFi.status() != WL_CONNECTED) return false;
  uint32_t c = counter + 1;
  uint16_t ack = ackEmpty() ? 0 : ackq[ackHead];
  String body = String("{\"c\":") + c + ",\"t\":\"" + type + "\",\"v\":" + PROTOCOL_VERSION +
                ",\"r\":" + WiFi.RSSI() + ",\"up\":" + (millis()/1000) + ",\"f\":" + currentFlags() + ",\"fwv\":\"" FW_VERSION "\",\"fwb\":\"" FW_BUILD "\"";
#ifdef NET_CELL
  body += ",\"net\":\"cell\"";
#else
  body += ",\"net\":\"wifi\"";
#endif
  body += String(",\"open\":") + (openSt?1:0) + ",\"arm\":" + (armed?1:0) + ",\"ang\":" + String(angle,1) + ",\"sir\":" + (alSt == AL_SOUNDING ? 1 : 0) + ",\"mv\":" + ((lastMovedMs && millis()-lastMovedMs < 60000) ? 1 : 0) + ",\"cal\":" + (calOk?1:0);
  if (sndDb >= 0 && millis() - sndDbAt < 120000) body += String(",\"db\":") + sndDb;   // dBFS+100, uncalibrated; omitted when no recent reading
  bool sentSnd = sndFlag; if (sentSnd) body += ",\"snd\":1";   // one-shot chime flag
  bool sentUnl = unlPending; if (sentUnl) body += ",\"unl\":1";
  body += String(",\"run\":") + (runSt == RUNNING || runSt == QUIETING ? 2 : runSt == VIBRATING ? 1 : 0);   // current run state on every post
  body += String(",\"nx\":") + hbSec;   // seconds until next scheduled post (server offline = nx + grace)
  if (ack) body += String(",\"ack\":") + ack;
  if (curEvArg) body += String(",\"el\":") + curEvArg;   // cycle_stalled: seconds since cycle start
  body += "}";   // battery fields (bv/bp) omitted = null: USB powered
  int code = -1; String resp, ra;
  for (int attempt = 0; attempt < 2; attempt++) {   // one retry on a network failure
  String url = srvUrl() + "/v1/ingest";
#ifdef SLEEP_ENABLED
  static HTTPClient http; static WiFiClient plain; static WiFiClientSecure tls; http.setReuse(true);   // keep-alive within a wake
#else
  HTTPClient http; WiFiClient plain; WiFiClientSecure tls;
#endif
  uint32_t t0 = millis();
  if (url.startsWith("https")) { if (strlen(SERVER_CA_PEM)) tls.setCACert(SERVER_CA_PEM); else tls.setInsecure(); http.begin(tls, url); }
  else http.begin(plain, url);
  http.setConnectTimeout(HTTP_TIMEOUT_MS); http.setTimeout(HTTP_TIMEOUT_MS);
  const char* hdrs[] = {"Retry-After"}; http.collectHeaders(hdrs, 1);
  http.addHeader("content-type", "application/json");
  http.addHeader("x-device-id", DEVICE_ID);
  http.addHeader("x-signature", hmacHex(body));
  code = http.POST(body); resp = http.getString(); ra = http.header("Retry-After"); http.end();
  uint32_t dt = millis() - t0;
  if (code <= 0 || dt > HTTP_SLOW_MS) Serial.printf("[HTTP] %s %s try %d: code %d in %lums\n", code <= 0 ? "FAIL" : "SLOW", type, attempt + 1, code, (unsigned long)dt);
  if (code > 0) break;
  srvIpOk = false;   // re-resolve before the retry
  }
  Serial.printf("[HTTP] %s -> %d %s\n", body.c_str(), code, resp.c_str());
  if (code == 200) {
    otaPostOk();
    counter = c; prefs.putUInt("ctr", counter); if (sentUnl) unlPending = false; if (sentSnd) sndFlag = false;
    int ci = resp.indexOf("\"c\":");
    if (ci < 0 || (uint32_t)resp.substring(ci+4).toInt() != c) { Serial.println("[HTTP] WARN reply counter mismatch; ignoring cmds (hb still applied)");
      int hp = resp.indexOf("\"hb\":"); if (hp >= 0) { long v = resp.substring(hp+5).toInt(); if (v >= HB_MIN_S && v <= 3600 && (uint32_t)v != hbSec) { hbSec = v; prefs.putUInt("hb_s", hbSec); Serial.printf("[HB] heartbeat %lus (server)\n", (unsigned long)hbSec); } } }
    else { if (ack) ackHead = (ackHead + 1) % 8;
      if (pendingLocalArm && strcmp(type, pendingLocalType) == 0) { pendingLocalArm = false; Serial.printf("[ARM] server acked %s\n", type); }
      handleReply(resp); }
    txBackoffMs = 0; return true;
  }
  if (code == 409) {   // replay: jump to server's counter, retry same event with lastCounter+1
    int i = resp.indexOf("lastCounter\":");
    if (i >= 0) { counter = resp.substring(i+13).toInt(); prefs.putUInt("ctr", counter); }
    else backoff(0);   // no lastCounter: don't hammer the same counter
    return false;
  }
  if (code == 429) { backoff(ra.length() ? (uint32_t)ra.toInt() * 1000 : 0); return false; }
  if (code == 400 || code == 401) return true;  // drop: bad schema / bad auth (check secrets.h)
  backoff(0);   // network/5xx
  return false;
}


// ---------- OTA: LAN (ArduinoOTA) + server-driven HTTP(S) with SHA-256 + rollback ----------
// Rollback: the prebuilt Arduino-ESP32 bootloader has CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y. We override
// verifyRollbackLater() so the core does NOT auto-confirm a fresh image at boot; it stays PENDING_VERIFY until the
// first 200 from the server (otaPostOk). No 200 within OTA_VERIFY_MS -> mark invalid + reboot -> bootloader boots the
// old slot. A crash/reset before confirmation also rolls back (bootloader marks a PENDING_VERIFY image ABORTED).
extern "C" bool verifyRollbackLater() { return true; }
bool otaPendingVerify = false, otaBusy = false, otaLanUp = false;
String otaWantV, otaWantUrl, otaWantSha; uint32_t otaLastTry = 0; bool otaTried = false;
bool usbPower() {
#ifdef BOARD_M5STICKS3
  return M5.Power.getVBUSVoltage() > 4300;   // -1 if the PMIC can't report -> treated as battery
#else
  return false;
#endif
}
void otaScreen(const char* msg) {
#ifdef BOARD_M5STICKS3
  M5.Display.wakeup(); M5.Display.setBrightness(DISPLAY_BRIGHTNESS); M5.Display.fillScreen(0x0000);
  M5.Display.setTextColor(0xFFFF, 0x0000); M5.Display.setTextDatum(middle_center); M5.Display.setTextSize(2);
  M5.Display.drawString(msg, M5.Display.width()/2, M5.Display.height()/2);
#endif
  Serial.printf("[OTA] %s\n", msg);
}
void otaQuiet() {   // stop siren + any sound before flashing
  alarmStop("alarm_disarm"); nsOn = false;
#ifdef BOARD_M5STICKS3
  spkReady(); M5.Speaker.stop();   // also releases the mic (shared ES8311)
#endif
}
String jStr(const String& o, const char* k) {
  String key = String("\"") + k + "\":"; int i = o.indexOf(key); if (i < 0) return "";
  i += key.length(); while (i < (int)o.length() && o[i] == ' ') i++; if (o[i] != '"') return "";
  int e = i + 1; while (e < (int)o.length() && !(o[e] == '"' && o[e-1] != '\\')) e++;
  String v = o.substring(i + 1, e); v.replace("\\/", "/"); return v;
}
void otaOffer(const String& r) {
  int f = r.indexOf("\"fw\":"); if (f < 0) return;
  int b = r.indexOf('{', f); if (b < 0 || b > f + 8) return;
  int e = r.indexOf('}', b); if (e < 0) return;
  String o = r.substring(b, e + 1);
  String v = jStr(o, "v"), url = jStr(o, "url"), sha = jStr(o, "sha256"); sha.toLowerCase();
  bool hex = sha.length() == 64; for (int i = 0; hex && i < 64; i++) hex = isxdigit((unsigned char)sha[i]);
  if (!v.length() || v == FW_VERSION || !url.startsWith("http") || !hex) { if (v.length() && v != FW_VERSION) Serial.println("[OTA] fw offer ignored: bad url/sha256"); return; }
  if (v != otaWantV) { otaWantV = v; otaTried = false; Serial.printf("[OTA] server offers %s (running %s)\n", v.c_str(), FW_VERSION); }
  otaWantUrl = url; otaWantSha = sha;
}
const char* otaBlocked() {
  if (armed) return "armed"; if (sirenAllowed) return "sirenAllowed"; if (alSt == AL_SOUNDING) return "alarm";
  if (runSt != IDLE) return "running"; if (machineState == 1) return "guarded"; if (otaPendingVerify) return "unverified";
#ifdef SLEEP_ENABLED
  if (!usbPower()) return "battery";   // battery build: USB power only (undetectable VBUS reads as battery)
#endif
  if (WiFi.status() != WL_CONNECTED) return "offline";
  return nullptr;
}
void otaFail(uint8_t reason, const char* why) {
  uint8_t n = prefs.getUChar("ota_n", 0) + 1; prefs.putUChar("ota_n", n);
  Serial.printf("[OTA] FAIL %s (reason %u, fail %u/%d for %s)\n", why, reason, n, OTA_MAX_FAILS, otaWantV.c_str());
  queueEventArg("ota_fail", reason); otaBusy = false;
#ifdef BOARD_M5STICKS3
  showState(true);
#endif
}
// reasons: 1 http, 2 size/begin, 3 write/stream, 4 sha256 mismatch, 5 end/boot set, 6 new image rolled back
void otaRun() {
  otaLastTry = millis(); otaTried = true;
  if (prefs.getString("ota_v", "") != otaWantV) { prefs.putString("ota_v", otaWantV); prefs.putUChar("ota_n", 0); }
  { uint32_t sa = curEvArg; curEvArg = 0; bool ok = postEvent("ota_start"); curEvArg = sa; Serial.printf("[OTA] ota_start %s\n", ok ? "sent" : "not delivered"); }   // sent now: the download blocks the loop
  otaBusy = true; otaQuiet(); otaScreen("UPDATING");
  HTTPClient http; WiFiClient plain; WiFiClientSecure tls;
  if (otaWantUrl.startsWith("https")) { if (strlen(SERVER_CA_PEM)) tls.setCACert(SERVER_CA_PEM); else tls.setInsecure(); http.begin(tls, otaWantUrl); }
  else http.begin(plain, otaWantUrl);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS); http.setConnectTimeout(HTTP_TIMEOUT_MS); http.setTimeout(15000);
  int code = http.GET();
  if (code != 200) { http.end(); otaFail(1, "http"); return; }
  int len = http.getSize();
  if (!Update.begin(len > 0 ? (size_t)len : UPDATE_SIZE_UNKNOWN, U_FLASH)) { http.end(); otaFail(2, Update.errorString()); return; }
  mbedtls_sha256_context sc; mbedtls_sha256_init(&sc); mbedtls_sha256_starts(&sc, 0);
  WiFiClient* st = http.getStreamPtr(); static uint8_t buf[2048]; size_t got = 0; uint32_t lastData = millis();
  while (http.connected() && (len <= 0 || got < (size_t)len)) {
    size_t a = st->available();
    if (!a) { if (millis() - lastData > 15000) break; delay(2); continue; }
    int n = st->readBytes(buf, a > sizeof(buf) ? sizeof(buf) : a); if (n <= 0) continue;
    lastData = millis(); mbedtls_sha256_update(&sc, buf, n);
    if (Update.write(buf, n) != (size_t)n) { mbedtls_sha256_free(&sc); Update.abort(); http.end(); otaFail(3, "write"); return; }
    got += n;
  }
  http.end();
  uint8_t h[32]; mbedtls_sha256_finish(&sc, h); mbedtls_sha256_free(&sc);
  if (len > 0 && got != (size_t)len) { Update.abort(); otaFail(3, "short read"); return; }
  char hx[65]; for (int i = 0; i < 32; i++) sprintf(hx + 2*i, "%02x", h[i]);
  if (otaWantSha != hx) { Update.abort(); Serial.printf("[OTA] sha256 got %s want %s\n", hx, otaWantSha.c_str()); otaFail(4, "sha256 mismatch"); return; }
  if (!Update.end(true)) { otaFail(5, Update.errorString()); return; }   // sets the boot partition only after the hash matched
  prefs.putString("ota_new", otaWantV); prefs.putUChar("ota_n", 0);
  Serial.printf("[OTA] %u bytes ok, sha256 verified -> reboot into %s\n", (unsigned)got, otaWantV.c_str()); otaScreen("REBOOTING"); delay(300); ESP.restart();
}
void otaPostOk() {
  if (!otaPendingVerify) return;
  esp_ota_mark_app_valid_cancel_rollback(); otaPendingVerify = false; prefs.remove("ota_new");
  Serial.printf("[OTA] image %s confirmed valid (first 200)\n", FW_VERSION); queueEvent("ota_ok");
}
void otaBoot() {
  esp_ota_img_states_t s; const esp_partition_t* run = esp_ota_get_running_partition();
  otaPendingVerify = run && esp_ota_get_state_partition(run, &s) == ESP_OK && s == ESP_OTA_IMG_PENDING_VERIFY;
  String nv = prefs.getString("ota_new", "");
  if (!otaPendingVerify && nv.length() && nv != FW_VERSION) {   // we flashed nv, but we're running the old image: it rolled back
    prefs.remove("ota_new"); otaWantV = nv; prefs.putUChar("ota_n", prefs.getUChar("ota_n", 0) + 1); otaTried = true; otaLastTry = millis();
    Serial.printf("[OTA] %s rolled back, still on %s\n", nv.c_str(), FW_VERSION); queueEventArg("ota_fail", 6); }
  else if (!otaPendingVerify && nv.length()) prefs.remove("ota_new");
  if (otaPendingVerify) Serial.printf("[OTA] new image %s pending verify (needs a 200 within %lus)\n", FW_VERSION, OTA_VERIFY_MS/1000);
}
String otaHost() { String h = "falcon-"; for (const char* p = DEVICE_ID; *p; p++) { char c = tolower((unsigned char)*p); if (isalnum((unsigned char)c) || c == '-') h += c; } return h; }
void otaLanTick() {
  static bool warned = false;
  if (!strlen(OTA_PASS)) { if (!warned) { warned = true; Serial.println("[OTA] WARN OTA_PASS not set in secrets.h: LAN OTA disabled"); } return; }
  bool want = WiFi.status() == WL_CONNECTED;
#ifdef SLEEP_ENABLED
  want = want && usbPower();   // battery build: LAN OTA only while awake and on USB
#endif
  if (want && !otaLanUp) {
    String h = otaHost(); ArduinoOTA.setHostname(h.c_str()); ArduinoOTA.setPassword(OTA_PASS); ArduinoOTA.setPort(OTA_LAN_PORT);
    ArduinoOTA.setMdnsEnabled(false); ArduinoOTA.setRebootOnSuccess(true);
    ArduinoOTA.onStart([]() { otaBusy = true; otaQuiet(); otaScreen("UPDATING"); });
    ArduinoOTA.onEnd([]() { otaScreen("REBOOTING"); });
    ArduinoOTA.onError([](ota_error_t e) { otaBusy = false; Serial.printf("[OTA] LAN error %u\n", e);
#ifdef BOARD_M5STICKS3
      showState(true);
#endif
    });
    ArduinoOTA.begin();
    if (!mdnsUp) mdnsUp = MDNS.begin(h.c_str());
    if (mdnsUp) MDNS.enableArduino(OTA_LAN_PORT, true);
    otaLanUp = true; Serial.printf("[OTA] LAN OTA up: %s @ %s:%d\n", h.c_str(), WiFi.localIP().toString().c_str(), OTA_LAN_PORT);
  } else if (!want && otaLanUp) { ArduinoOTA.end(); otaLanUp = false; Serial.println("[OTA] LAN OTA stopped"); }
  if (otaLanUp) ArduinoOTA.handle();
}
void otaTick() {
  otaLanTick();
  if (otaPendingVerify && millis() > OTA_VERIFY_MS) {
    Serial.println("[OTA] no server 200 within verify window -> rollback"); delay(200); esp_ota_mark_app_invalid_rollback_and_reboot(); }
  if (!otaWantV.length() || otaWantV == FW_VERSION || otaBusy) return;
  if (prefs.getString("ota_v", "") == otaWantV && prefs.getUChar("ota_n", 0) >= OTA_MAX_FAILS) return;   // gave up on this version
  if (otaTried && millis() - otaLastTry < OTA_RETRY_MS) return;
  static uint32_t lastLog = 0; const char* why = otaBlocked();
  if (why) { if (millis() - lastLog > 60000) { lastLog = millis(); Serial.printf("[OTA] %s waiting: %s\n", otaWantV.c_str(), why); } return; }
  otaRun();
}

// ---------- serial console ----------
void serialCmd(String l) {
  l.trim(); if (!l.length()) return;
  if (l == "help") Serial.println("cmds: dump | set <key> <val> | save | defaults | cal | stream on|off | wifi <ssid> <pass> | send <event> | status | siren (test) | silence | snd (stream level/top bin) | sndlearn | sndforget");
  else if (l == "dump") dumpTun();
  else if (l.startsWith("set ")) {
    int sp = l.indexOf(' ', 4); String k = l.substring(4, sp), v = l.substring(sp+1);
    for (int i = 0; i < NTD; i++) if (k == TD[i].k) { if (TD[i].t == 'i') *(int*)TD[i].p = v.toInt(); else *(float*)TD[i].p = v.toFloat(); Serial.println("ok (use save to persist)"); return; }
    Serial.println("unknown key");
  }
  else if (l == "save") { saveTun(); Serial.println("saved"); }
  else if (l == "defaults") { for (int i = 0; i < NTD; i++) prefs.remove(String(TD[i].k).substring(0,15).c_str()); loadTun(); Serial.println("defaults restored"); }
  else if (l == "cal") calibrate();
  else if (l == "snd") { sndStream = !sndStream; Serial.printf("snd stream %s (levels + top bin only, no audio)\n", sndStream ? "on" : "off"); }
  else if (l == "sndlearn") { memset(&sndLearn, 0, sizeof(sndLearn)); segN = 0; curBin = -1; lowRunMs = 0; memset(lowHist, 0, sizeof(lowHist)); sndLearnUntil = millis() + 12000;
    Serial.println("[SND] learning for 12 s: play the machine's done chime now"); }
  else if (l == "sndforget") { memset(&sndFp, 0, sizeof(sndFp)); prefs.remove("snd_fp"); Serial.println("[SND] fingerprint cleared (generic rules)"); }
  else if (l == "ota") { esp_ota_img_states_t st; const esp_partition_t* rp = esp_ota_get_running_partition(); const esp_partition_t* np = esp_ota_get_next_update_partition(NULL);
    bool hs = rp && esp_ota_get_state_partition(rp, &st) == ESP_OK;
    Serial.printf("fw %s run=%s@0x%lx state=%s next=%s pendingVerify=%d lan=%d(%s) offer=%s fails=%u blocked=%s\n", FW_VERSION, rp ? rp->label : "?", rp ? (unsigned long)rp->address : 0UL,
      !hs ? "n/a" : st == ESP_OTA_IMG_VALID ? "valid" : st == ESP_OTA_IMG_PENDING_VERIFY ? "pending_verify" : st == ESP_OTA_IMG_NEW ? "new" : st == ESP_OTA_IMG_UNDEFINED ? "undefined" : "invalid/aborted",
      np ? np->label : "?", otaPendingVerify, otaLanUp, otaHost().c_str(), otaWantV.length() ? otaWantV.c_str() : "-", prefs.getUChar("ota_n", 0), otaBlocked() ? otaBlocked() : "no"); }
  else if (l == "calstat") Serial.printf("cal:%d ang=%.1f max=%.1f\n", calOk?1:0, angle, T.runMaxDeg);
  else if (l == "log on") { logOn = true; Serial.println("H,t_ms,ax,ay,az,gx,gy,gz,run,open,armed,rms   # L=sample M=mark E=event"); }
  else if (l == "log off") logOn = false;
  else if (l.startsWith("mark ")) Serial.printf("M,%lu,%s\n", (unsigned long)millis(), l.substring(5).c_str());
  else if (l == "stream on") streamOn = true; else if (l == "stream off") streamOn = false;
  else if (l.startsWith("wifi ")) { int sp = l.indexOf(' ', 5); wifiSsid = l.substring(5, sp); wifiPass = sp > 0 ? l.substring(sp+1) : "";
    prefs.putString("ssid", wifiSsid); prefs.putString("pass", wifiPass); WiFi.disconnect(); Serial.println("wifi saved"); }
  else if (l.startsWith("send ")) queueEvent(l.substring(5).c_str());   // copied into the queue
  else if (l == "siren") { armed = true; sirenAllowed = true; armedAt = millis() - ALARM_EXIT_DELAY_MS; hitN = ALARM_CONFIRM_HITS - 1; hitT[0] = millis(); alarmMotion(millis()); }
  else if (l == "silence") alarmStop("alarm_silenced");
  else if (l.startsWith("state ")) setMachineState(l.substring(6).toInt());
#ifdef SLEEP_ENABLED
  else if (l.startsWith("disp ")) { int sp = l.indexOf(' ', 5); dispMode = l.substring(5).toInt(); if (sp > 0) dispPeriodS = l.substring(sp+1).toInt();
    prefs.putInt("disp", dispMode); prefs.putInt("disp_n", dispPeriodS); Serial.printf("display mode %d period %ds (saved)\n", dispMode, dispPeriodS); }
  else if (l == "sleep on" || l == "sleep off") { sleepOn = (l == "sleep on"); prefs.putBool("sleep", sleepOn); Serial.printf("sleep %s (saved)\n", sleepOn ? "ON" : "OFF"); }
#endif
  else if (l == "wifiscan") { int n = WiFi.scanNetworks(); for (int i = 0; i < n; i++) Serial.printf("  %s rssi=%d ch=%d enc=%d\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i), (int)WiFi.encryptionType(i)); Serial.printf("[WIFI] %d networks\n", n); }
  else if (l == "status" && (Serial.printf("wifi_status=%d (0 idle,1 no SSID,3 connected,4 failed,6 disconnected) last_reason=%u disc_count=%lu ssid=%s pass_len=%u\n", (int)WiFi.status(), wifiLastReason, (unsigned long)wifiDiscN, wifiSsid.c_str(), (unsigned)wifiPass.length()), true)) Serial.printf("siren=%d allowed=%d wifi=%d rssi=%d ctr=%u armed=%d run=%d open=%d angle=%.1f rms=%.4f\n",
      alSt == AL_SOUNDING, sirenAllowed, WiFi.status()==WL_CONNECTED, WiFi.RSSI(), counter, armed, runSt, openSt, angle, rms),
    Serial.printf("db=%d snd_on=%d last_chime=%lds_ago fp=%s/%u\n", sndDb, T.sndOn, lastChimeMs ? (long)((millis()-lastChimeMs)/1000) : -1L, sndFp.type == 2 ? "buzzer" : sndFp.type == 1 ? "jingle" : "none", sndFp.n);
  else Serial.println("? (help)");
}


// =====================================================================================
// Battery sleep (m5sticks3_sleep env: -DSLEEP_ENABLED). Default builds are unchanged.
// Wake path (M5 docs, docs.m5stack.com/en/arduino/m5sticks3/m5pm1): BMI270 INT1 -> M5PM1 PYG4 (input)
// -> M5PM1 PYG1 configured as IRQ output -> ESP32-S3 G13 (active low) -> ext0 wake. KEY1 (G11) -> ext1 wake.
// Idle: deep sleep, Wi-Fi off, speaker/amp + display off, IMU any-motion armed, timer wake for check-in.
// During a load: light sleep, waking every SLEEP_LOAD_PERIOD_MS to sample a 1 s burst at 100 Hz for load-done.
// =====================================================================================
#ifdef SLEEP_ENABLED
#include <M5PM1.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
M5PM1 pm1;
RTC_DATA_ATTR uint32_t rtcMagic = 0;
RTC_DATA_ATTR bool rtcArmed = false, rtcSirenAllowed = false, rtcOpen = false, rtcPendingLocal = false;
RTC_DATA_ATTR float rtcRest[3] = {0,0,1};
RTC_DATA_ATTR int64_t rtcLastHbS = 0;            // RTC wall seconds (gettimeofday survives deep sleep) of the last check-in
RTC_DATA_ATTR uint8_t rtcState = 0;
int dispMode = 0, dispPeriodS = DISPLAY_PERIOD_S_DEF;   // 0 = flash on wake/button, 1 = + periodic flash every N s, 2 = always on when on USB
int64_t rtcNowS() { struct timeval tv; gettimeofday(&tv, nullptr); return tv.tv_sec; }
bool onUsb() { return M5.Power.getVBUSVoltage() > 4300; }   // -1 if unsupported -> treated as battery
bool sleepOn = true;                              // runtime toggle: serial `sleep on|off` (NVS "sleep")
bool wokeByMotion = false, wokeByTimer = false, wokeByButton = false;
uint32_t lastActivityMs = 0;

void sleepSaveRtc() {
  rtcState = machineState; rtcMagic = 0xFA1C0001; rtcArmed = armed; rtcSirenAllowed = sirenAllowed; rtcOpen = openSt; rtcPendingLocal = pendingLocalArm;
  rtcRest[0] = restG.x; rtcRest[1] = restG.y; rtcRest[2] = restG.z;
  prefs.putBool("armed", armed); prefs.putBool("siren_ok", sirenAllowed);   // also in NVS (survives power loss)
}
// returns true if state was restored from a deep-sleep wake (skip calibration)
bool sleepRestore() {
  esp_sleep_wakeup_cause_t w = esp_sleep_get_wakeup_cause();
  wokeByMotion = (w == ESP_SLEEP_WAKEUP_EXT0); wokeByButton = (w == ESP_SLEEP_WAKEUP_EXT1); wokeByTimer = (w == ESP_SLEEP_WAKEUP_TIMER);
  if (w == ESP_SLEEP_WAKEUP_UNDEFINED || rtcMagic != 0xFA1C0001) {          // cold boot: NVS fallback
    armed = false; sirenAllowed = false; return false; }   // cold boot: never restore armed; first server reply (a) decides
  machineState = rtcState; armed = rtcArmed; sirenAllowed = rtcSirenAllowed; openSt = rtcOpen; pendingLocalArm = rtcPendingLocal;
  if (pendingLocalArm) pendingLocalType = armed ? "armed_local" : "disarmed_local";
  restG = {rtcRest[0], rtcRest[1], rtcRest[2]}; grav = restG; prevA = restG;
  armedAt = millis() - ALARM_EXIT_DELAY_MS;      // exit delay was already served before sleeping
  return true;
}
void pm1Init() {
  Wire.end(); Wire.begin(PIN_SDA, PIN_SCL, 100000U);   // per M5 example: share the internal bus with M5Unified
  if (pm1.begin(&Wire, M5PM1_DEFAULT_ADDR, PIN_SDA, PIN_SCL, M5PM1_I2C_FREQ_100K) != M5PM1_OK) { Serial.println("[SLEEP] PM1 init failed"); return; }
  pm1.irqClearGpioAll(); pm1.irqClearSysAll(); pm1.irqClearBtnAll();     // releases G13 after a motion wake
  pm1.irqSetGpioMaskAll(M5PM1_IRQ_MASK_ENABLE); pm1.irqSetSysMaskAll(M5PM1_IRQ_MASK_ENABLE); pm1.irqSetBtnMaskAll(M5PM1_IRQ_MASK_ENABLE);
  pm1.irqSetGpioMask(M5PM1_IRQ_GPIO4, M5PM1_IRQ_MASK_DISABLE);
  pm1.gpioSetMode(M5PM1_GPIO_NUM_4, M5PM1_GPIO_MODE_INPUT); pm1.gpioSetPull(M5PM1_GPIO_NUM_4, M5PM1_GPIO_PULL_UP);
  pm1.gpioSetMode(M5PM1_GPIO_NUM_1, M5PM1_GPIO_MODE_OUTPUT); pm1.gpioSetDrive(M5PM1_GPIO_NUM_1, M5PM1_GPIO_DRIVE_PUSHPULL);
  pm1.gpioSetFunc(M5PM1_GPIO_NUM_1, M5PM1_GPIO_FUNC_IRQ);
}
bool imuArmAnyMotion() {
  static BMI270 bmi;
  if (bmi.beginI2C(BMI2_I2C_PRIM_ADDR) != BMI2_OK) { Serial.println("[SLEEP] BMI270 (SparkFun) init failed"); return false; }
  int8_t r = bmi.enableFeature(BMI2_ANY_MOTION);
  bmi2_sens_config c; c.type = BMI2_ANY_MOTION; r |= bmi.getConfig(&c);
  c.cfg.any_motion.threshold = SLEEP_ANYMOTION_THR;    // 1 LSB = 0.48 mg
  c.cfg.any_motion.duration  = SLEEP_ANYMOTION_DUR;    // 1 LSB = 20 ms
  r |= bmi.setConfig(c);
  bmi2_int_pin_config ip; ip.pin_type = BMI2_INT1; ip.int_latch = BMI2_INT_NON_LATCH;
  ip.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW; ip.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
  ip.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE; ip.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
  r |= bmi.setInterruptPinConfig(ip); r |= bmi.mapInterruptToPin(BMI2_ANY_MOTION_INT, BMI2_INT1);
  return r == BMI2_OK;
}
void wifiOff() { if (WiFi.getMode() != WIFI_OFF) { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); } }
void goDeepSleep() {
  int64_t hbS = (int64_t)hbSec, since = rtcNowS() - rtcLastHbS;
  uint32_t left = since < hbS ? (uint32_t)(hbS - since) : 1;
  if (dispMode == 1 && dispPeriodS > 0 && (uint32_t)dispPeriodS < left) left = dispPeriodS;   // periodic flash wake
  Serial.printf("[SLEEP] deep sleep, armed=%d, next check-in in %lus\n", armed, (unsigned long)left); Serial.flush();
  sleepSaveRtc(); wifiOff();
  M5.Mic.end(); M5.Speaker.end();                   // releases I2S and turns the AW8737 amp off (PYG3)
  showState(false);
  imuArmAnyMotion(); pm1Init();                      // clear IRQ, route IMU INT -> G13
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 0); rtc_gpio_pullup_en(GPIO_NUM_13); rtc_gpio_pulldown_dis(GPIO_NUM_13);
  esp_sleep_enable_ext1_wakeup(1ULL << PIN_BOOT_BTN, ESP_EXT1_WAKEUP_ANY_LOW);   // KEY1 (active-low UNVERIFIED)
  esp_sleep_enable_timer_wakeup((uint64_t)left * 1000000ULL);
  esp_deep_sleep_start();
}
// light sleep between 1 s sampling bursts while a load runs (millis() keeps counting)
void lightSleepGap() {
  wifiOff(); Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_LOAD_PERIOD_MS * 1000ULL);
  esp_light_sleep_start();
}
void sleepTick(uint32_t now, bool motionNow) {
  if (!sleepOn) return;
  if (motionNow) lastActivityMs = now;
  bool busyNet = (evHead != evTail) || !ackEmpty() || pendingLocalArm;
  if (alSt == AL_SOUNDING || busyNet) return;
  if (flashUntil) { if ((int32_t)(now - flashUntil) < 0) return; flashUntil = 0; showState(false); }   // finish the colour flash
  if (dispMode == 2 && onUsb()) return;                               // always-on on USB: no sleep, screen stays lit                       // finish alarm / sends first (Wi-Fi reused within this wake)
  if (sirenAllowed && armed && now - armedAt < ALARM_EXIT_DELAY_MS) return;   // serve the exit delay awake
  if (tiltSince || untiltSince || doorWinStart) return;               // lid/door transition in progress
  static uint32_t burstStart = 0;
  if (runSt != IDLE) {                                                // load: 1 s burst, then light sleep
    if (!burstStart) burstStart = now;
    if (now - burstStart >= 1000) { burstStart = 0; lightSleepGap(); }
    return;
  }
  burstStart = 0;
  if (now - lastActivityMs >= SLEEP_IDLE_MS) goDeepSleep();
}
#endif

void setup() {
#ifdef BOARD_M5STICKS3
  { auto cfg = M5.config(); cfg.output_power = false; M5.begin(cfg); M5.BtnA.setHoldThresh(LONGPRESS_MS); }  // PMIC (M5PM1) keeps power; no hold pin
#endif
#ifdef PIN_HOLD
  pinMode(PIN_HOLD, OUTPUT); digitalWrite(PIN_HOLD, HIGH);   // keep M5StickC Plus2 powered on battery
#endif
  Serial.begin(115200);
#ifdef SLEEP_ENABLED
  M5.Display.setBrightness(0);                    // display not needed; saves ~20 mA
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) delay(1500);   // fast path on wake (<1 s alarm target)
#else
  delay(1500);
#endif
  Serial.println("\nProject Falcon sensor fw " FW_VERSION " (JSON /v1/ingest)");
  prefs.begin("falcon", false);
  counter = prefs.getUInt("ctr", 0);
  if (prefs.isKey("ack")) { ackPush(prefs.getInt("ack", 0)); prefs.remove("ack"); }  // reboot command ack
  if (prefs.isKey("ssid")) { wifiSsid = prefs.getString("ssid"); wifiPass = prefs.getString("pass"); }
  loadTun(); sndLoad();
  hbSec = prefs.getUInt("hb_s", HB_DEFAULT_S); sirenMode = prefs.getUChar("siren_m", 1); if (sirenMode > 2) sirenMode = 1;
  machineState = 0;   // cold boot: never restore GUARDED/DONE from flash; first server reply (st) decides
  doneAtS = prefs.getLong64("done_t", 0); if (doneAtS > clkS()) doneAtS = clkS();   // clock reset by reboot: restart count
  otaBoot();
  calOk = prefs.getBool("cal_ok", true); tiltOn = prefs.getBool("tilt", false);
  pinMode(PIN_BOOT_BTN, BTN_PULL);
  // user reset: button held for 10 s from power-up wipes Wi-Fi creds + tuning; counter (ctr) is kept, ownership stays on the server
  if (BTN_DOWN()) { uint32_t t0 = millis(); while (BTN_DOWN() && millis() - t0 < FACTORY_HOLD_MS) delay(20);
    if (millis() - t0 >= FACTORY_HOLD_MS) { uint32_t c = counter; prefs.clear(); prefs.putUInt("ctr", c); Serial.println("[RESET] user reset: Wi-Fi + tuning wiped, counter kept");
      wifiSsid = WIFI_SSID; wifiPass = WIFI_PASS; loadTun(); } }
#ifndef BOARD_M5STICKS3
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
#endif
  while (!axInit()) { Serial.println("[ERR] IMU not found - check wiring/board env"); delay(2000); }
  buzzerInit();
#ifdef SLEEP_ENABLED
  sleepOn = prefs.getBool("sleep", true);
  pm1Init();
  bool restored = sleepRestore();
  if (!restored) { bootCal(); queueEvent("boot"); }
  dispMode = prefs.getInt("disp", 0); dispPeriodS = prefs.getInt("disp_n", DISPLAY_PERIOD_S_DEF);
  bool checkin = !restored || rtcNowS() - rtcLastHbS >= (int64_t)hbSec - 5;
  if (checkin) { rtcLastHbS = rtcNowS(); if (restored) queueEvent("heartbeat"); }   // reply carries st + queued commands
  if (wokeByTimer && !checkin) flashState(DISPLAY_FLASH_MS);                       // periodic flash wake only
  else if (wokeByMotion || wokeByButton || !restored) flashState(DISPLAY_FLASH_MS);
  if (dispMode == 2 && onUsb()) showState(true);
  // motion wake while armed: the wake itself counts as the first jerk hit, so one more confirms (<1 s target).
  // Lid/door transitions still clear it (alarmMotion ignores hits while tilting / after door changes).
  if (wokeByMotion && armed && sirenAllowed) { hitN = 1; hitT[0] = millis(); }     // periodic check-in: picks up queued server commands in the reply
  lastActivityMs = millis(); lastHb = millis();
  if (wokeByTimer) lastActivityMs = millis() - SLEEP_IDLE_MS + 2000;   // check-in only: back to sleep ~2 s after the send
  Serial.printf("[SLEEP] wake: motion=%d timer=%d button=%d restored=%d armed=%d\n", wokeByMotion, wokeByTimer, wokeByButton, restored, armed);
#else
  bootCal();
  queueEvent("boot");
  wifiConnect();
#ifdef BOARD_M5STICKS3
  showState(true);                               // USB build: colour always on
#endif
#endif
}

void loop() {
  static uint32_t nextSample = 0, lastStream = 0, btnDown = 0, lastTx = 0;
  uint32_t now = millis();
  if ((int32_t)(now - nextSample) >= 0) {
    nextSample = now + 1000/SAMPLE_HZ;
    float x,y,z; if (axRead(x,y,z)) { processSample({x,y,z}, now);
      static uint8_t ld = 0;
      if (logOn && (++ld % (SAMPLE_HZ / LOG_HZ)) == 0)
        Serial.printf("L,%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%d,%d,%d,%.4f\n", (unsigned long)now, x, y, z, gX, gY, gZ, runSt, openSt, armed, rms); }
    if (streamOn && now - lastStream >= 100) { lastStream = now;
      Serial.printf("ax=%.3f ay=%.3f az=%.3f ang=%.1f rms=%.4f jerk=%.1f run=%d open=%d\n", x,y,z, angle, rms, jerk, runSt, openSt); }
  }
  // long press BOOT -> recalibrate
#ifdef BOARD_M5STICKS3
  // KEY1: click = arm/disarm toggle (silence while sounding), hold 2 s = recalibrate
  M5.update(); (void)btnDown;
  if (M5.BtnA.wasHold()) calibrate();
  else if (M5.BtnA.wasClicked()) {
    if (alSt == AL_SOUNDING) { graceUntil = now + ALARM_DISARM_GRACE_MS; alarmStop("alarm_silenced"); }
    else if (machineState == 3) { unlPending = true; setMachineState(0); queueEvent("heartbeat"); beep(1500, 120); Serial.println("[DONE] cleared at puck -> unl:1"); }
    else { localArmToggle(now); beep(armed ? 2000 : 800, 120); }
#ifdef SLEEP_ENABLED
    flashState(DISPLAY_FLASH_MS);
#endif
  }
#else
  // short press (50 ms..2 s) = arm/disarm toggle (silence while sounding); hold 2 s = recalibrate
  if (BTN_DOWN()) { if (!btnDown) btnDown = now; else if (now - btnDown > LONGPRESS_MS) { calibrate(); btnDown = 0; while (BTN_DOWN()) delay(10); } }
  else if (btnDown) { if (now - btnDown >= 50) {
      if (alSt == AL_SOUNDING) { graceUntil = now + ALARM_DISARM_GRACE_MS; alarmStop("alarm_silenced"); }
      else if (machineState == 3) { unlPending = true; setMachineState(0); queueEvent("heartbeat"); }
      else localArmToggle(now); }
    btnDown = 0; }
#endif

  otaTick();
  alarmTick(now);
  if (resyncPending) { resyncPending = false; Serial.println("[NET] (re)connected: resync st/a"); queueEvent("heartbeat"); }
  sndTick(now);
#ifdef BOARD_M5STICKS3
  // never-started nudge: rising "hey, start me!" jingle once (~2 s), then flash until a cycle starts, ns:0, or 10 min
  if (nsOn && (runSt == RUNNING || runSt == QUIETING || now - nsSince > 600000UL)) { nsOn = false; nsJingleStep = -1; showState(true); }
  if (nsOn) {
    static const uint16_t jf[] = {523, 659, 784, 0, 784, 1047, 0, 1319}; static const uint16_t jd[] = {180, 180, 260, 120, 160, 320, 120, 520};
    if (nsJingleStep >= 0 && alSt != AL_SOUNDING && (int32_t)(now - nsJingleAt) >= 0) {
      if (nsJingleStep < 8) { if (jf[nsJingleStep]) { spkReady(); M5.Speaker.setVolume(s3Volume()); M5.Speaker.tone(jf[nsJingleStep], jd[nsJingleStep]); } nsJingleAt = now + jd[nsJingleStep]; nsJingleStep++; }
      else nsJingleStep = -1; }
    static uint32_t lastFl = 0; static bool inv = false;
    if (now - lastFl >= 500) { lastFl = now; inv = !inv; if (inv) { M5.Display.wakeup(); M5.Display.fillScreen(0xFFFF); M5.Display.setBrightness(DISPLAY_BRIGHTNESS); } else showState(true);
#ifdef SLEEP_ENABLED
      lastActivityMs = now;   // stay awake while nudging
#endif
    }
  }
#endif
#if defined(BOARD_M5STICKS3) && !defined(SLEEP_ENABLED)
  { static uint32_t lastDoneDraw = 0; if (machineState == 3 && now - lastDoneDraw >= 60000) { lastDoneDraw = now; showState(true); } }   // tick the done timer
#endif
  // while the siren sounds, poll every 5 s (empty heartbeat) so a queued silence_alarm arrives in the reply
  { static uint32_t lastPoll = 0;
    if (alSt == AL_SOUNDING && now - lastPoll >= ALARM_POLL_MS && evHead == evTail) { lastPoll = now; queueEvent("heartbeat"); } }
#ifdef SLEEP_ENABLED
  if (!sleepOn || evHead != evTail || !ackEmpty()) wifiConnect();   // Wi-Fi off while idle
#else
  wifiConnect();
#endif
  { static uint32_t lastLoop = 0; if (lastLoop && now - lastLoop > 1000) Serial.printf("[LOOP] stalled %lums\n", (unsigned long)(now - lastLoop)); lastLoop = millis(); }
  if (now - lastHb >= hbSec * 1000UL) { lastHb = now; bool q = false; for (int i = evHead; i != evTail; i = (i + 1) % EVENT_QUEUE_LEN) if (!strcmp(evq[i], "heartbeat")) q = true;
    if (!q) queueEvent("heartbeat"); else Serial.println("[HB] previous heartbeat still queued (post backlog)"); }
  if (evHead == evTail && !ackEmpty()) queueEvent("ack");      // flush remaining acks, one per message
  if (evHead != evTail && now - lastTx >= MIN_TX_INTERVAL_MS && (int32_t)(now - nextTxAt) >= 0) {
    lastTx = now;
    curEvArg = evArg[evHead];
    if (postEvent(evq[evHead])) evHead = (evHead + 1) % EVENT_QUEUE_LEN;
  }
  static String line; while (Serial.available()) { char ch = Serial.read(); if (ch == '\n') { serialCmd(line); line = ""; } else if (ch != '\r') line += ch; }
#ifdef SLEEP_ENABLED
  sleepTick(millis(), rms > T.quietRms || jerk > T.movedJerk * 0.5f || streamOn || logOn || line.length() || otaPendingVerify || otaLanUp);
#endif
}
