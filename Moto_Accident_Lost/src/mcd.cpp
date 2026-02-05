#include "mcd.h"
#include <Arduino.h>
#include <math.h>

// Setup phần chống trộm
static const double LOST1_THRESHOLD_M = 20.0;
static const double LOST2_THRESHOLD_M = 50.0;
static const int MIN_SATS_FOR_ANTITHEFT = 4;
static const int ANTITHEFT_CONSEC_SAMPLES = 3;
static const double GPS_JITTER_IGNORE_M = 5.0;

// Setup còi và buzzer
static const int BUZZER_CH = 0;
static const int BUZZER_FREQ = 2500;
static const int BUZZER_RES = 8;
static const int BUZZER_DUTY_SOFT = 70;
static const int BUZZER_DUTY_LOUD = 150;

// Setup các chân cho đèn buzzer công tắc
static uint8_t LEDPIN = 255;
static uint8_t BUZZERPIN = 255;
static uint8_t ARMPIN = 255;

// Cờ bật tắt chế độ chống trộm
static bool antiTheftEnabled = false;

// Tránh trường hợp công tắc bị dội
static bool stableArm = HIGH;
static bool lastReadArm = HIGH;
static unsigned long lastDebounceTime = 0;
static const unsigned long debounceDelay = 80;

// Lưu vị trí ban đầu khi bật chống trộm
static bool homeSet = false;
static Location homeLocation = {0, 0};

// Đếm số lần vượt ngưỡng để cảnh báo
static int consecLost1 = 0;
static int consecLost2 = 0;
static unsigned long lastTheftBlink = 0;
static bool theftBlinkState = false;

// Thiết lập ngưỡng góc nghiêng phát hiện ngã xe
// 
static const float PITCH_SAFE_MIN = 55.0f;
static const float PITCH_SAFE_MAX = 90.0f;
// lưu độ thay đổi gia tốc
static float lastCrashDeltaG = 0.0f;

// Ngưỡng accchange phát hiện tai nạn
static const float CRASH_DELTA_G_THRESHOLD = 5.0f;

// hệ số để lọc để chống nhiễu khi xe bị rung
static const float PITCH_LPF_ALPHA = 0.3f;

// Crash delta state
static bool oldAccelInit = false;
static float oldAx = 0.0f, oldAy = 0.0f, oldAz = 0.0f;

// Pitch filter state
static bool pitchInit = false;
static float pitchAccFilt = 0.0f;

// Trạng thái tai nạn để ưu tiên output
static bool accidentActive = false;

// Điều kiện xác định ngã
static const int FALL_CONSEC_SAMPLES = 3;
static const unsigned long FALL_MIN_MS = 3000;
static int consecFall = 0;

// Khi crash thì giữ còi 10s
static const unsigned long CRASH_ALARM_MS = 10000;
static bool crashAlarmActive = false;
static unsigned long crashAlarmStartMs = 0;

//Hàm điều khiển buzzer
static inline void buzzerOnSoft() { ledcWrite(BUZZER_CH, BUZZER_DUTY_SOFT); }
static inline void buzzerOnLoud() { ledcWrite(BUZZER_CH, BUZZER_DUTY_LOUD); }
static inline void buzzerOff() { ledcWrite(BUZZER_CH, 0); }

static inline void outputsOff() {
  digitalWrite(LEDPIN, LOW);
  buzzerOff();
}
// In log ra serial để kiểm tra các thông số
static void printHomeLocation(const Location &loc, int sats) {
  Serial.print("[ANTI] HOME SET: ");
  Serial.print(loc.lat, 6);
  Serial.print(", ");
  Serial.print(loc.lng, 6);
  Serial.print(" | sats=");
  Serial.println(sats);

  Serial.print("[ANTI] MAP: https://maps.google.com/?q=");
  Serial.print(loc.lat, 6);
  Serial.print(",");
  Serial.println(loc.lng, 6);

  Serial.print("[ANTI] Luu vi tri ban dau: https://maps.google.com/?q=");
  Serial.print(loc.lat, 6);
  Serial.print(",");
  Serial.println(loc.lng, 6);
}

static double deg2rad(double x) { return x * M_PI / 180.0; }
// công thức harversine để tính khoảng cách
static double distanceMeters(Location a, Location b) {
  const double R = 6371.0;
  double dLat = deg2rad(b.lat - a.lat);
  double dLon = deg2rad(b.lng - a.lng);
  double sa = sin(dLat / 2.0);
  double sb = sin(dLon / 2.0);
  double tmp = sa * sa + cos(deg2rad(a.lat)) * cos(deg2rad(b.lat)) * sb * sb;
  double c = 2.0 * atan2(sqrt(tmp), sqrt(1.0 - tmp));
  return R * c * 1000.0;
}

// helper reset common anti-theft runtime states
static void resetAntiTheftRuntime() {
  consecLost1 = consecLost2 = 0;
  lastTheftBlink = 0;
  theftBlinkState = false;
}

// Đọc công tắc từ ARMPIN để bật tắt chống trộm
static void updateArmSwitch(TinyGPSPlus &gps) {
  unsigned long now = millis();
  bool reading = digitalRead(ARMPIN);

  // debounce
  if (reading != lastReadArm) {
    lastDebounceTime = now;
    lastReadArm = reading;
  }

  if ((now - lastDebounceTime) > debounceDelay && reading != stableArm) {
    stableArm = reading;

  // bật chống trộm (ARM)
    if (stableArm == LOW) {
      antiTheftEnabled = true;
      resetAntiTheftRuntime();
      homeSet = false;
      if (gps.location.isValid() && gps.satellites.isValid() &&
          gps.satellites.value() >= MIN_SATS_FOR_ANTITHEFT) {
        homeLocation = { gps.location.lat(), gps.location.lng() };
        homeSet = true;
        Serial.println("[ANTI] ARM ON => set HOME now");
        printHomeLocation(homeLocation, gps.satellites.value());
      } else {
        Serial.print("[ANTI] ARM ON but GPS not ready (fix=");
        Serial.print(gps.location.isValid() ? "YES" : "NO");
        Serial.print(", sats=");
        Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
        Serial.println(") => waiting to set HOME...");
      }

    } else {
      // tắt chống trộm (ARM)
      antiTheftEnabled = false;
      homeSet = false;
      resetAntiTheftRuntime();

      if (!accidentActive) outputsOff();
      Serial.println("[ANTI] ARM OFF => anti-theft disabled");
    }
  }

  // Nếu bật lúc chưa có GPS fix, set home khi có fix lần đầu
  if (antiTheftEnabled && !homeSet) {
    if (gps.location.isValid() && gps.satellites.isValid() &&
        gps.satellites.value() >= MIN_SATS_FOR_ANTITHEFT) {
      homeLocation = { gps.location.lat(), gps.location.lng() };
      homeSet = true;
      Serial.println("[ANTI] GPS ready => set HOME (late)!");
      printHomeLocation(homeLocation, gps.satellites.value());
    }
  }
}

// hàm để bật tắt chống trộm từ xa
void mcdRemoteSetAntiTheft(bool enable, TinyGPSPlus &gps) {
  if (enable == antiTheftEnabled) {
    Serial.print("[ANTI] REMOTE ");
    Serial.print(enable ? "ON" : "OFF");
    Serial.println(" ignored (same state)");
    return;
  }

  if (enable) {
    antiTheftEnabled = true;
    resetAntiTheftRuntime();
    homeSet = false;

    Serial.println("[ANTI] REMOTE ON => anti-theft enabled");

    if (gps.location.isValid() && gps.satellites.isValid() &&
        gps.satellites.value() >= MIN_SATS_FOR_ANTITHEFT) {
      homeLocation = { gps.location.lat(), gps.location.lng() };
      homeSet = true;
      Serial.println("[ANTI] REMOTE ON => set HOME now");
      printHomeLocation(homeLocation, gps.satellites.value());
    } else {
      Serial.print("[ANTI] REMOTE ON but GPS not ready (fix=");
      Serial.print(gps.location.isValid() ? "YES" : "NO");
      Serial.print(", sats=");
      Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
      Serial.println(") => waiting to set HOME...");
    }
  } else {
    antiTheftEnabled = false;
    homeSet = false;
    resetAntiTheftRuntime();

    if (!accidentActive) outputsOff();
    Serial.println("[ANTI] REMOTE OFF => anti-theft disabled");
  }
}

// hàm tính toán tai nạn
static DeviceStatus handleAccident(MPU6050 &mpu) {
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();

  // CRASH: delta gia tốc (accChange)
  bool crashDetected = false;
  if (!oldAccelInit) {
    oldAx = ax; oldAy = ay; oldAz = az;
    oldAccelInit = true;
  } else {
    float dx = ax - oldAx;
    float dy = ay - oldAy;
    float dz = az - oldAz;
    oldAx = ax; oldAy = ay; oldAz = az;

    float dMag = sqrtf(dx*dx + dy*dy + dz*dz);
    lastCrashDeltaG = dMag;
    if (dMag > CRASH_DELTA_G_THRESHOLD) crashDetected = true;
  }

  // FALL detect theo góc nghiêng 
  float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * (180.0f / PI);
  if (!pitchInit) {
    pitchAccFilt = pitchAcc;
    pitchInit = true;
  } else {
    pitchAccFilt = pitchAccFilt + PITCH_LPF_ALPHA * (pitchAcc - pitchAccFilt);
  }

  float absPitch = fabsf(pitchAccFilt);
  bool isSafe = (absPitch >= PITCH_SAFE_MIN && absPitch <= PITCH_SAFE_MAX);
  bool fallDetected = !isSafe;

  // ===== Nếu đang CRASH alarm thì giữ còi đủ 10s
  if (crashAlarmActive) {
    if (millis() - crashAlarmStartMs < CRASH_ALARM_MS) {
      accidentActive = true;
      digitalWrite(LEDPIN, HIGH);
      buzzerOnLoud();
      return DEV_CRASH;
    } else {
      crashAlarmActive = false;
      crashAlarmStartMs = 0;
      // đi xuống dưới để xét FALL / NONE
    }
  }

  // Điều kiện tại nạn bắt buộc accChange vượt ngưỡng và góc nghiêng vượt ngưỡng
  if (crashDetected && fallDetected) {
    crashAlarmActive = true;
    crashAlarmStartMs = millis();

    accidentActive = true;
    digitalWrite(LEDPIN, HIGH);
    buzzerOnLoud();
    return DEV_CRASH;
  }

  // Xét trạng thái ngã xe nếu nó giữ trong 3s
  static unsigned long fallStartMs = 0;

  if (fallDetected) {
    if (consecFall == 0) fallStartMs = millis();
    consecFall++;
  } else {
    consecFall = 0;
    fallStartMs = 0;
  }

  bool fallConfirmed =
      (consecFall >= FALL_CONSEC_SAMPLES) &&
      (fallStartMs != 0) &&
      (millis() - fallStartMs >= FALL_MIN_MS);

  if (fallConfirmed) {
    accidentActive = true;
    digitalWrite(LEDPIN, HIGH);
    buzzerOnSoft();
    return DEV_FALL;
  }

  accidentActive = false;
  outputsOff();
  return DEV_NONE;
}

// Hàm phát hiện xe bị di chuyển khói vị trí gốc
static DeviceStatus handleAntiTheft(TinyGPSPlus &gps) {
  if (!antiTheftEnabled || !homeSet) return DEV_NONE;

  if (!gps.location.isValid() || !gps.satellites.isValid() ||
      gps.satellites.value() < MIN_SATS_FOR_ANTITHEFT) {
    return DEV_NONE;
  }

  Location cur = { gps.location.lat(), gps.location.lng() };
  double d = distanceMeters(homeLocation, cur);

  if (d < GPS_JITTER_IGNORE_M) {
    consecLost1 = consecLost2 = 0;
    if (!accidentActive) outputsOff();
    return DEV_NONE;
  }

  if (d > LOST2_THRESHOLD_M) {
    consecLost2++;
    consecLost1 = 0;
  } else if (d > LOST1_THRESHOLD_M) {
    consecLost1++;
    consecLost2 = 0;
  } else {
    consecLost1 = consecLost2 = 0;
  }

  DeviceStatus st = DEV_NONE;
  if (consecLost2 >= ANTITHEFT_CONSEC_SAMPLES) st = DEV_LOST2;
  else if (consecLost1 >= ANTITHEFT_CONSEC_SAMPLES) st = DEV_LOST1;

  if (st != DEV_NONE && !accidentActive) {
    unsigned long now = millis();
    if (now - lastTheftBlink > 300) {
      lastTheftBlink = now;
      theftBlinkState = !theftBlinkState;

      digitalWrite(LEDPIN, theftBlinkState);
      if (theftBlinkState) buzzerOnSoft();
      else buzzerOff();
    }
  } else {
    if (!accidentActive) outputsOff();
  }

  return st;
}

//  Khởi tạo module (gán chân cấu hình cho cho coi)
void mcdInit(uint8_t ledPin, uint8_t buzzerPin, uint8_t armSwitchPin) {
  LEDPIN = ledPin;
  BUZZERPIN = buzzerPin;
  ARMPIN = armSwitchPin;

  pinMode(LEDPIN, OUTPUT);
  pinMode(ARMPIN, INPUT_PULLUP);

  ledcSetup(BUZZER_CH, BUZZER_FREQ, BUZZER_RES);
  ledcAttachPin(BUZZERPIN, BUZZER_CH);

  outputsOff();

  antiTheftEnabled = false;
  stableArm = digitalRead(ARMPIN);
  lastReadArm = stableArm;
  lastDebounceTime = millis();

  homeSet = false;
  resetAntiTheftRuntime();

  accidentActive = false;
  oldAccelInit = false;
  pitchInit = false;

  consecFall = 0;

  // reset crash alarm
  crashAlarmActive = false;
  crashAlarmStartMs = 0;
}
// hàm chạy mỗi vòng lăp, cập nhật trạng thái để trả về DeviceStatus
DeviceStatus mcdUpdate(MPU6050 &mpu, TinyGPSPlus &gps, bool *antiTheftEnabledOut) {
  updateArmSwitch(gps);
  if (antiTheftEnabledOut) *antiTheftEnabledOut = antiTheftEnabled;

  DeviceStatus acc = handleAccident(mpu);
  if (acc != DEV_NONE) return acc;

  return handleAntiTheft(gps);
}

float mcdGetLastCrashDeltaG() { return lastCrashDeltaG; }
float mcdGetCrashDeltaGThreshold() { return CRASH_DELTA_G_THRESHOLD; }
