#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <MPU6050_tockn.h>
#include <ArduinoJson.h>
#include <math.h>
#include "mcd.h"


#define I2C_SDA 32
#define I2C_SCL 33

#define GPS_RX 22
#define GPS_TX 23

#define SIM_RX 2
#define SIM_TX 4

#define LED_PIN 19
#define BUZZER_PIN 17
#define ARM_SW_PIN 18


HardwareSerial gpsSerial(1);
HardwareSerial simSerial(2);

// ====== Sensor & state ======
TinyGPSPlus gps;
MPU6050 mpu(Wire);

// In trạng thái định kỳ
unsigned long lastStatusPrint = 0;
const unsigned long STATUS_INTERVAL = 5000UL;

// Gọi kiểm tra SIM định kỳ
unsigned long lastSimCheck = 0;
const unsigned long SIM_CHECK_INTERVAL = 30000UL;

// Chu kì gửi dữ liệu
static unsigned long lastMqttSend = 0;
static const unsigned long MQTT_SEND_INTERVAL_NORMAL = 25000UL;
static const unsigned long MQTT_SEND_INTERVAL_ALERT  = 3000UL;

static inline bool isAbnormal(DeviceStatus st) {
  return (st == DEV_CRASH || st == DEV_FALL || st == DEV_LOST1 || st == DEV_LOST2);
}

static inline unsigned long getMqttInterval(DeviceStatus st) {
  return isAbnormal(st) ? MQTT_SEND_INTERVAL_ALERT : MQTT_SEND_INTERVAL_NORMAL;
}

// ====== Trạng thái hệ thống ======
DeviceStatus lastStatus = DEV_NONE;
bool lastAntiTheftEnabled = false;

// ====== TRẠNG THÁI SIM ======
static bool simModuleOk = false;
static bool simSimReady = false;
static int  simCsq = -1;
static int  simCgatt = -1;
static int  simCops = -1;
static int  simCereg = -1;
static bool simHasIp = false;
static bool simCanSend = false;
static int  lastSimState = -1;

// ===== MQTT config =====
static const char* MQTT_HOST      = "broker.emqx.io";
static const int   MQTT_PORT      = 1883;
static const char* MQTT_TOPIC     = "/mcd/location/ESP32-MOTO-01";
static const char* MQTT_CLIENT_ID = "ESP32-MOTO-01";
static const char* MQTT_CMD_TOPIC = "/mcd/cmd/ESP32-MOTO-01";


static bool mqttSessionOpen = false;
static bool mqttConnected   = false;

// Nhận lệnh từ server
static bool remoteAntiTheft = false;
static bool cmdSubscribed   = false;

// Tối ưu phản hồi tai nạn
static bool urgentPublishPending = false;
static unsigned long lastUrgentAttempt = 0;
static const unsigned long URGENT_RETRY_MS = 800;
static unsigned long lastAnyAtTime = 0;

// Timeout AT để tránh treo khi chờ phản hồi từ module SIM
static const uint32_t AT_TIMEOUT_DEFAULT = 1200;
static const uint32_t AT_TIMEOUT_LONG    = 12000;

// Tránh xung đột UART 
static volatile bool simBusy = false;


void handleSimUrcLine(const String& line);

// phản hồi AT và để mã hoá và giải mã payload 
static bool atOk(const String& r) { return r.indexOf("OK") != -1; }

static String toHex(const String& s) {
  const char* hex = "0123456789ABCDEF";
  String out; out.reserve(s.length() * 2);
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t b = (uint8_t)s[i];
    out += hex[b >> 4];
    out += hex[b & 0x0F];
  }
  return out;
}

static String hexToText(const String& hex) {
  String out;
  if (hex.length() < 2) return out;
  out.reserve(hex.length() / 2);
  for (int i = 0; i + 1 < (int)hex.length(); i += 2) {
    char c1 = hex[i];
    char c2 = hex[i + 1];
    int hi = (c1 <= '9') ? (c1 - '0') : (toupper(c1) - 'A' + 10);
    int lo = (c2 <= '9') ? (c2 - '0') : (toupper(c2) - 'A' + 10);
    out += (char)((hi << 4) | lo);
  }
  return out;
}

static bool isHexString(const String& s) {
  if (s.length() == 0) return false;
  if (s.length() % 2 != 0) return false;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    bool ok = (c >= '0' && c <= '9') ||
              (c >= 'a' && c <= 'f') ||
              (c >= 'A' && c <= 'F');
    if (!ok) return false;
  }
  return true;
}

// Cơ chế đọc URC
static void simDrainUrc(uint32_t msBudget) {
  unsigned long start = millis();
  static String line;

  while (millis() - start < msBudget) {
    while (simSerial.available()) {
      char c = (char)simSerial.read();

      if (c == '\n') {
        line.trim();
        if (line.length() > 0) {
          if (line[0] == '+') {
            if (line.indexOf("+CMQ") != -1) {
              Serial.print("[SIM-URC] ");
              Serial.println(line);
            }
            handleSimUrcLine(line);
          }
        }
        line = "";
      } else if (c != '\r') {
        line += c;
        if (line.length() > 400) line = "";
      }
    }
    delay(2);
  }
}

// hàm này đảm bảo AT command không bị treo vô hạn
String simSendCommand(const char *cmd, uint32_t timeoutMs = AT_TIMEOUT_DEFAULT) {
  simBusy = true;


  simDrainUrc(20);

  simSerial.print(cmd);
  simSerial.print("\r");

  String resp;
  String line;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (simSerial.available()) {
      char c = (char)simSerial.read();
      resp += c;

      if (c == '\n') {
        line.trim();
        if (line.length() > 0) {
          // URC thật bắt đầu bằng '+'
          if (line[0] == '+') {
            if (line.indexOf("+CMQ") != -1) {
              Serial.print("[SIM-URC] ");
              Serial.println(line);
            }
            handleSimUrcLine(line);
          }

          // Kết thúc khi AT trả kết quả
          if (line == "OK" || line.indexOf("ERROR") != -1) {
            lastAnyAtTime = millis();
            simBusy = false;
            simDrainUrc(40);
            return resp;
          }
        }
        line = "";
      } else if (c != '\r') {
        line += c;
        if (line.length() > 400) line = "";
      }
    }
    delay(2);
  }

  lastAnyAtTime = millis();
  simBusy = false;
  simDrainUrc(40);
  return resp;
}

// Kiểm tra xem SIM có đủ điều kiện để gửi dữ liệu hay không
bool parseCPIN_READY(const String &resp) { return resp.indexOf("CPIN: READY") != -1; }

bool parseCSQ(const String &resp, int &csqOut) {
  int idx = resp.indexOf("+CSQ:");
  if (idx == -1) return false;
  int colon = resp.indexOf(':', idx);
  int comma = resp.indexOf(',', colon);
  if (colon == -1 || comma == -1) return false;
  String v = resp.substring(colon + 1, comma); v.trim();
  csqOut = v.toInt();
  return true;
}

bool parseCGATT(const String &resp, int &cgattOut) {
  int idx = resp.indexOf("+CGATT:");
  if (idx == -1) return false;
  int colon = resp.indexOf(':', idx);
  if (colon == -1) return false;
  String v = resp.substring(colon + 1); v.trim();
  cgattOut = v.toInt();
  return true;
}

bool parseCOPS(const String &resp, int &modeOut) {
  int idx = resp.indexOf("+COPS:");
  if (idx == -1) return false;
  int colon = resp.indexOf(':', idx);
  if (colon == -1) return false;
  String v = resp.substring(colon + 1); v.trim();
  int comma = v.indexOf(',');
  if (comma == -1) return false;
  String m = v.substring(0, comma); m.trim();
  modeOut = m.toInt();
  return true;
}

bool parseCEREG(const String &resp, int &statOut) {
  int idx = resp.indexOf("+CEREG:");
  if (idx == -1) return false;
  int colon = resp.indexOf(':', idx);
  if (colon == -1) return false;

  String v = resp.substring(colon + 1); v.trim();
  int comma1 = v.indexOf(',');
  if (comma1 == -1) return false;

  String rest = v.substring(comma1 + 1); rest.trim();
  int comma2 = rest.indexOf(',');
  String statStr = (comma2 == -1) ? rest : rest.substring(0, comma2);
  statStr.trim();

  statOut = statStr.toInt();
  return true;
}

bool parseCGPADDR_hasIp(const String &resp) {
  int idx = resp.indexOf("+CGPADDR:");
  if (idx == -1) return false;
  int comma = resp.indexOf(',', idx);
  if (comma == -1) return false;
  String ip = resp.substring(comma + 1); ip.trim();
  return ip.length() >= 7;
}

// Kiểm tra tình trạng của module SIM để xác định có đủ đk gửi dữ liệu vầ in ra serial
bool simQuickHealthCheck(bool verboseOnChangeOnly) {
  String r1 = simSendCommand("AT");
  bool moduleOk = (r1.indexOf("OK") != -1);

  String r2 = simSendCommand("AT+CPIN?");
  bool simReady = parseCPIN_READY(r2);

  String r3 = simSendCommand("AT+CSQ");
  int csq = -1; parseCSQ(r3, csq);

  String r4 = simSendCommand("AT+CGATT?");
  int cgatt = -1; parseCGATT(r4, cgatt);

  String r5 = simSendCommand("AT+COPS?", 2500);
  int copsMode = -1; parseCOPS(r5, copsMode);

  String r6 = simSendCommand("AT+CEREG?", 2500);
  int ceregStat = -1; parseCEREG(r6, ceregStat);
  bool regOk = (ceregStat == 1 || ceregStat == 5);

  String r7 = simSendCommand("AT+CGPADDR=1", 2500);
  bool hasIp = parseCGPADDR_hasIp(r7);

  simModuleOk = moduleOk;
  simSimReady = simReady;
  simCsq = csq;
  simCgatt = cgatt;
  simCops = copsMode;
  simCereg = ceregStat;
  simHasIp = hasIp;

  bool canSend = moduleOk && simReady && (csq > 0) && (cgatt == 1) && (copsMode >= 0) && regOk && hasIp;

  int newState = canSend ? 1 : 0;
  if (verboseOnChangeOnly && newState == lastSimState) {
    simCanSend = canSend;
    return canSend;
  }

  Serial.println();
  Serial.println("[SIM] Kiểm tra nhanh xem SIM còn online không...");
  Serial.print("[SIM] Module: "); Serial.println(moduleOk ? "OK" : "KHÔNG PHẢN HỒI AT");
  Serial.print("[SIM] SIM: "); Serial.println(simReady ? "READY" : "NOT READY");
  Serial.print("[SIM] CSQ: "); Serial.println(csq);
  Serial.print("[SIM] CGATT: "); Serial.println(cgatt);
  Serial.print("[SIM] CEREG: "); Serial.println(ceregStat);
  Serial.print("[SIM] IP: "); Serial.println(hasIp ? "OK (có IP)" : "NO (chưa có IP)");

  lastSimState = newState;
  simCanSend = canSend;

  if (!simCanSend) {
    mqttConnected = false;
    mqttSessionOpen = false;
    cmdSubscribed = false;
  }

  return canSend;
}

// Chặn ngay từ đầu nếu SIM k đủ điều kiện gửi dữ liệu
static void mqttEnsureConnected() {
  if (!simCanSend) {
    mqttConnected = false;
    mqttSessionOpen = false;
    cmdSubscribed = false;
    return;
  }
// để modem tạo kết nối nền tới MQTT broker
  if (!mqttSessionOpen) {
    Serial.println("[MQTT] CMQNEW...");

   
    simSendCommand("AT+CMQDISCON=0", 3000);
    delay(200);

    simSendCommand("AT+CMEE=2", 1500);

    String cmd = String("AT+CMQNEW=\"") + MQTT_HOST + "\"," + String(MQTT_PORT) + ",6000,1024,0";
    String r = simSendCommand(cmd.c_str(), AT_TIMEOUT_LONG);

    Serial.println("[MQTT] CMQNEW resp:");
    Serial.println(r);

    if (r.indexOf("+CMQNEW: 0") != -1 && atOk(r)) mqttSessionOpen = true;
    else { mqttSessionOpen = false; mqttConnected = false; return; }
  }
// hàm kết nối đến MQTT broker bằng AT+CMQCON
  if (!mqttConnected) {
    Serial.println("[MQTT] CMQCON...");
    String con = String("AT+CMQCON=0,4,\"") + MQTT_CLIENT_ID + "\",60,1,0";
    String r = simSendCommand(con.c_str(), AT_TIMEOUT_LONG);

    Serial.println("[MQTT] CMQCON resp:");
    Serial.println(r);

    if (atOk(r)) mqttConnected = true;
    else { mqttConnected = false; return; }
  }
// nhận lệnh từ server thông qua qua việc subscribed bằng lệnh AT+CMQSUB
  if (mqttConnected && !cmdSubscribed) {
    Serial.println("[MQTT] CMQSUB (cmd topic)...");
    String sub = String("AT+CMQSUB=0,\"") + MQTT_CMD_TOPIC + "\",1";
    String rs = simSendCommand(sub.c_str(), AT_TIMEOUT_LONG);

    Serial.println("[MQTT] CMQSUB resp:");
    Serial.println(rs);

    if (atOk(rs)) {
      cmdSubscribed = true;
      Serial.println("[MQTT] CMD SUBSCRIBED OK");
    } else {
      Serial.println("[MQTT] CMQSUB FAILED (will retry later)");
    }
  }
}

// đóng gói trạng thái hiện tại thành các chuỗi JSON và public lên MQTT
static void mqttPublishState(DeviceStatus st, bool antiTheftEnabled) {
  mqttEnsureConnected();
  if (!mqttConnected) return;
// lấy dữ liệu GPS
  bool gpsOk = gps.location.isValid();
  double lat = gpsOk ? gps.location.lat() : 0.0;
  double lng = gpsOk ? gps.location.lng() : 0.0;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
// chuyển trạng thái trong device thành chuỗi để thiết bị dễ đọc
  const char* statusStr = "NONE";
  if (st == DEV_CRASH) statusStr = "CRASH";
  else if (st == DEV_LOST1) statusStr = "LOST1";
  else if (st == DEV_LOST2) statusStr = "LOST2";
  else if (st == DEV_FALL) statusStr = "FALL";
// lấy dữ liệu từ MPU và tính toán
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  float accTotal = sqrtf(ax*ax + ay*ay + az*az);
  float angleY = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
// tạo JSON payload dưới dạng text
  String json = "{";
  json += "\"deviceId\":\""; json += MQTT_CLIENT_ID; json += "\",";
  json += "\"gpsOk\":"; json += (gpsOk ? "1" : "0"); json += ",";
  json += "\"location\":\""; json += String(lat, 6); json += ","; json += String(lng, 6); json += "\",";
  json += "\"status\":\""; json += statusStr; json += "\",";
  json += "\"antiTheft\":"; json += (antiTheftEnabled ? "1" : "0"); json += ",";
  json += "\"accTotal\":"; json += String(accTotal, 2); json += ",";
  json += "\"angleY\":"; json += String(angleY, 1); json += ",";
  json += "\"sats\":"; json += String(sats);
  json += "}";
// Đổi JSON sang HEX và publish bằng AT+CMQPUB
  String hex = toHex(json);
  int len = hex.length();

  String cmd = "AT+CMQPUB=0,\"";
  cmd += MQTT_TOPIC;
  cmd += "\",1,0,0,";
  cmd += String(len);
  cmd += ",\"";
  cmd += hex;
  cmd += "\"";

  simSendCommand(cmd.c_str(), AT_TIMEOUT_LONG);
}

// In ra serial toàn bộ các thông tin quan trọng như MPU, SIM, GPS
void printStatusLine(DeviceStatus st, bool antiTheftEnabled) {
  Serial.println("----- STATUS -----");

  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();

  float rollAcc  = atan2f(ay, az) * 180.0f / PI;
  float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
  float absPitch = fabsf(pitchAcc);

  Serial.print("[MPU] acc(g): ");
  Serial.print("[CRASH] deltaG=");
Serial.print(mcdGetLastCrashDeltaG(), 3);
Serial.print(" | threshold=");
Serial.println(mcdGetCrashDeltaGThreshold(), 3);

  Serial.print(ax, 2); Serial.print(", ");
  Serial.print(ay, 2); Serial.print(", ");
  Serial.print(az, 2);
  Serial.print(" | pitchAcc: "); Serial.print(pitchAcc, 1);
  // Serial.print(" | rollAcc: "); Serial.print(rollAcc, 1);
  Serial.print(" | |pitch|: "); Serial.print(absPitch, 1);
  Serial.println();

  Serial.print("[GPS] sats=");
  Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
  Serial.print(" | fix=");
  Serial.print(gps.location.isValid() ? "YES" : "NO");
  Serial.print(" | Location: ");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6); Serial.print(", ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("INVALID");
  }

  Serial.print("[SIM] ");
  Serial.print(simModuleOk ? "ALIVE " : "NO MODULE ");
  Serial.print("| SIM ");
  Serial.print(simSimReady ? "READY" : "NOT READY");
  Serial.print(" | CSQ="); Serial.print(simCsq);
  Serial.print(" | CGATT="); Serial.print(simCgatt);
  Serial.print(" | CEREG="); Serial.print(simCereg);
  Serial.print(" | IP="); Serial.println(simHasIp ? "OK" : "NO");

  Serial.print("[MQTT] ");
  Serial.println(mqttConnected ? "CONNECTED" : "DISCONNECTED");

  Serial.print("[SYSTEM] Anti-theft: ");
  Serial.print(antiTheftEnabled ? "ON" : "OFF");
  Serial.print(" | Status = ");
  switch (st) {
    case DEV_NONE:  Serial.println("NONE"); break;
    case DEV_FALL:  Serial.println("FALL"); break;
    case DEV_CRASH: Serial.println("CRASH"); break;
    case DEV_LOST1: Serial.println("LOST1 (>10m)"); break;
    case DEV_LOST2: Serial.println("LOST2 (>50m)"); break;
    default:        Serial.println("UNKNOWN"); break;
  }

  Serial.println("------------------\n");
}

// Nhận lệnh từ server và thay đổi trạng thái chống trộm từ xa
void handleSimUrcLine(const String& line) {

  if (line.indexOf("+CMQPUB:") == -1) return;

  // Lọc topic nếu nó không phải từ MQTT_CMD_TOPIC thì k nhận
  int topicStart = line.indexOf('\"');
  if (topicStart < 0) return;
  int topicEnd = line.indexOf('\"', topicStart + 1);
  if (topicEnd < 0) return;

  String topic = line.substring(topicStart + 1, topicEnd);
  if (topic != MQTT_CMD_TOPIC) return;

  // Lấy payload và giải mã
  int lastQuote1 = line.lastIndexOf('\"');
  if (lastQuote1 <= topicEnd) return;
  int lastQuote0 = line.lastIndexOf('\"', lastQuote1 - 1);
  if (lastQuote0 < 0) return;

  String raw = line.substring(lastQuote0 + 1, lastQuote1);
  String payload = isHexString(raw) ? hexToText(raw) : raw;

  Serial.print("[CMD] payload=");
  Serial.println(payload);

  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.println("[CMD] JSON parse fail");
    Serial.print("[CMD] rawLine=");
    Serial.println(line);
    return;
  }

  JsonObject obj = doc.as<JsonObject>();
  const char* deviceId = obj["deviceId"] | "";
  if (strlen(deviceId) > 0 && String(deviceId) != MQTT_CLIENT_ID) return;
// thực hiện lênh để update biến remoteAntitheft và gọi để bật tắt chống trộm
  if (obj.containsKey("toggleAntiTheft")) {
    bool newVal = obj["toggleAntiTheft"].as<bool>();
    if (remoteAntiTheft != newVal) {
      remoteAntiTheft = newVal;
mcdRemoteSetAntiTheft(remoteAntiTheft, gps);
      for (int i = 0; i < 2; i++) {
        digitalWrite(BUZZER_PIN, HIGH); delay(80);
        digitalWrite(BUZZER_PIN, LOW);  delay(80);
      }

      Serial.print("[CMD] remoteAntiTheft=");
      Serial.println(remoteAntiTheft ? "ON" : "OFF");

      // ép publish ngay để web thấy antiTheft đổi
      lastMqttSend = 0;
    }
  }
}

// khởi tạo tất cả phần cứng và phần mềm một lần khi thiết bị hoat động
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== ESP32 Moto Accident & Anti-Theft (SIM7020C MQTT) ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.begin();
  mpu.calcGyroOffsets(true);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  simSerial.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
  delay(200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ARM_SW_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  mcdInit(LED_PIN, BUZZER_PIN, ARM_SW_PIN);

  simQuickHealthCheck(false);
  Serial.println("=== SYSTEM START ===");
}

// vòng lặp chạy liên tục để đọc cảm biến, phát hiện sự cô
void loop() {
  
  simDrainUrc(5);

  // 1) GPS feed
  while (gpsSerial.available()) {
    gps.encode((char)gpsSerial.read());
  }

  // 2) MPU + detect
  mpu.update();
  bool antiTheftLocal = false;
  DeviceStatus st = mcdUpdate(mpu, gps, &antiTheftLocal);

  // gộp chống trộm: công tắc vật lý OR lệnh từ web
  bool antiTheftCombined = antiTheftLocal || remoteAntiTheft;

  // nếu trạng thái là bất thường thì ưu tiên publish dữ liệu nhanh hơn
  urgentPublishPending = isAbnormal(st);

  // in trạng thái khi có sự thay đổi 
  if (st != lastStatus || antiTheftCombined != lastAntiTheftEnabled) {
    lastStatus = st;
    lastAntiTheftEnabled = antiTheftCombined;
    printStatusLine(st, antiTheftCombined);
    lastMqttSend = 0;
  }

  // In định kỳ
  if (millis() - lastStatusPrint > STATUS_INTERVAL) {
    lastStatusPrint = millis();
    printStatusLine(st, antiTheftCombined);
  }

  // Hàm check SIM định kỳ khi bình thường
  if (!isAbnormal(st) && (millis() - lastSimCheck > SIM_CHECK_INTERVAL)) {
    lastSimCheck = millis();
    simQuickHealthCheck(true);
  }

  // Gửi dữ liệu lệnh MQTT
  unsigned long interval = getMqttInterval(st);
  bool dueNormal = (millis() - lastMqttSend > interval);
  bool dueUrgent = urgentPublishPending && (millis() - lastUrgentAttempt > URGENT_RETRY_MS);

  bool allowAtNow = (millis() - lastAnyAtTime > 50);

  if (simCanSend && allowAtNow && (dueUrgent || dueNormal)) {
    if (dueUrgent) lastUrgentAttempt = millis();
    lastMqttSend = millis();
    mqttPublishState(st, antiTheftCombined);
  }

  delay(5);
}
