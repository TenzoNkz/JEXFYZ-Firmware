#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <OneButton.h>
#include <math.h>
#include <time.h>
#include <atomic>
#include <cstring>
#include "esp_heap_caps.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Update.h>

// ArduinoDroid auto-generates function prototypes before the sketch body.
// Keep all sketch-scoped enum types visible to those generated prototypes.
enum class OtaState : uint8_t;
enum class OnlineOtaFlow : uint8_t;
enum class OnlineOtaIndicatorNext : uint8_t;
enum class Voltage : uint8_t;
enum class LedMode : uint8_t;
enum class VoltageRequestSource : uint8_t;

static constexpr char FW_VERSION[] = "V2.1";

// ============================================================================
// ========================== HEAP DIAGNOSTIC BUILD ===========================
// Diagnostic-only instrumentation retained from the earlier V1.0 baseline. Do not interpret it as physical validation.
// ============================================================================
static bool heapDiagPostAdvertisingActive = false;
static uint8_t heapDiagPostAdvertisingChecks = 0;
static uint32_t heapDiagPostAdvertisingStartedAt = 0;

static void runHeapCheck(const char* checkpointLabel) {
  Serial.print("[HEAP_CHECK] ");
  Serial.print(checkpointLabel);
  Serial.print(" | Free: ");
  Serial.print(ESP.getFreeHeap());
  Serial.print(" | Result: ");

  const bool intact = heap_caps_check_integrity_all(true);
  Serial.println(intact ? "OK" : "CORRUPTED");
  if (!intact) {
    Serial.println("[HEAP_CHECK] CORRUPTION DETECTED - STOPPING DIAGNOSTIC CHECKPOINTS");
    Serial.flush();
  }
}

static void scheduleHeapDiagPostAdvertisingChecks() {
  heapDiagPostAdvertisingActive = true;
  heapDiagPostAdvertisingChecks = 0;
  heapDiagPostAdvertisingStartedAt = millis();
}

static void processHeapDiagPostAdvertisingChecks() {
  if (!heapDiagPostAdvertisingActive) return;

  const uint32_t elapsed = millis() - heapDiagPostAdvertisingStartedAt;
  if ((heapDiagPostAdvertisingChecks & 0x01U) == 0 && elapsed >= 1000U) {
    heapDiagPostAdvertisingChecks |= 0x01U;
    runHeapCheck("CP15_1S_AFTER_ADVERTISING");
  }

  if ((heapDiagPostAdvertisingChecks & 0x02U) == 0 && elapsed >= 30000U) {
    heapDiagPostAdvertisingChecks |= 0x02U;
    runHeapCheck("CP16_30S_AFTER_ADVERTISING");
    heapDiagPostAdvertisingActive = false;
  }
}

// ======================== END HEAP DIAGNOSTIC BUILD ========================

static constexpr char PROTOCOL_VERSION[] = "P1";
static constexpr char BLE_NAME[] = "JE X FYZ";

static constexpr char SERVICE_UUID[] = "6f1c0001-7f35-4d0f-9b9a-7e9b2f5c1001";
static constexpr char RX_UUID[]      = "6f1c0002-7f35-4d0f-9b9a-7e9b2f5c1001";
static constexpr char TX_UUID[]      = "6f1c0003-7f35-4d0f-9b9a-7e9b2f5c1001";

static constexpr uint8_t PIN_NTC     = 0;
static constexpr uint8_t PIN_TOUCH   = 1;
static constexpr uint8_t PIN_12V     = 3;
static constexpr uint8_t PIN_9V      = 4;
static constexpr uint8_t PIN_PELTIER = 6;
static constexpr uint8_t PIN_FAN     = 5;
static constexpr uint8_t PIN_LED     = 7;
static constexpr uint8_t LED_COUNT   = 8;

static constexpr uint8_t OUTPUT_ON  = HIGH;
static constexpr uint8_t OUTPUT_OFF = LOW;

static constexpr char AP_SSID[]     = "JE X FYZ";

static constexpr char AP_PASSWORD[] = "0987654321";
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_GATEWAY(192, 168, 4, 1);
static const IPAddress AP_SUBNET(255, 255, 255, 0);

// ============================================================================
// ============================ OTA SAFETY LOCK ==============================
// OTA is release-critical. Keep this section near the top of the file.
// Never remove or bypass OTA startup, OTA priority, OTA diagnostics, safe
// outputs, finalization, or boot-partition activation during feature changes.
// Every future firmware change must pass the OTA safety audit and build.
// ============================================================================
static constexpr uint32_t BOOT_AP_TIMEOUT_MS          = 10000;
static constexpr uint32_t BOOT_AP_HARD_LIMIT_MS       = 60000;
static constexpr uint32_t LED_FRAME_INTERVAL_MS       = 33;
static constexpr uint32_t OTA_RESTART_DELAY_MS        = 1200;
static constexpr uint32_t OTA_FAILURE_INDICATOR_MS    = 1000;
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS     = 15000;
static constexpr uint8_t WIFI_CONNECT_MAX_ATTEMPTS      = 3;
static constexpr uint32_t WIFI_CONNECT_RETRY_BACKOFF_MS = 3000UL;
static constexpr uint32_t ONLINE_OTA_HTTP_TIMEOUT_MS  = 60000;
static constexpr uint32_t ONLINE_OTA_RESULT_INDICATOR_MS = 1000;
static constexpr uint32_t OTA_MAX_FIRMWARE_BYTES     = 2 * 1024 * 1024;
static_assert(OTA_MAX_FIRMWARE_BYTES <= 2 * 1024 * 1024,
              "OTA SAFETY: firmware limit exceeded");

// ============================================================================
// ======================= OTA STATE / PRIORITY LOCK =========================
// These declarations intentionally precede application feature state.
// ============================================================================
enum class OtaState : uint8_t {
  IDLE,
  BOOT_AP,
  LOCAL_UPLOAD,
  CLOUD_CONNECTING,
  CLOUD_DOWNLOADING,
  CLOUD_APPLYING,
  SUCCESS,
  FAILED,
};

enum class OnlineOtaFlow : uint8_t {
  IDLE,
  RADIO_TEARDOWN,
  WIFI_CONNECTING,
  DOWNLOAD,
  FLASH_FINALIZE,
};

enum class OnlineOtaIndicatorNext : uint8_t {
  NONE,
  PREPARE_DOWNLOAD,
  DOWNLOAD,
  FLASH_FINALIZE,
  RESTART,
  RECOVER,
};

static bool serviceOtaPriority();
static void processPendingOnlineOtaStart();

// ========================== END OTA PRIORITY LOCK ===========================

static constexpr size_t BLE_TX_QUEUE_SIZE = 48;
static constexpr size_t BLE_RX_QUEUE_SIZE = 8;
static constexpr size_t BLE_RX_BUFFER_SIZE = 512;
static constexpr uint32_t BLE_TX_INTERVAL_MS = 15;
static constexpr uint32_t ONLINE_OTA_START_WAIT_MS = 1500;
static constexpr uint32_t BLE_START_RETRY_MS = 750;
static constexpr uint8_t BLE_START_MAX_ATTEMPTS = 8;
static constexpr uint8_t RADIO_OFF_SETTLE_YIELDS = 16;

static constexpr float NTC_FIXED_OHMS    = 10000.0f;
static constexpr float NTC_NOMINAL_OHMS  = 10000.0f;
static constexpr float NTC_NOMINAL_C     = 25.0f;
static constexpr float NTC_BETA          = 3950.0f;
static constexpr uint32_t NTC_READ_INTERVAL_MS = 1000;
static constexpr uint32_t NTC_FAULT_LOW_MV  = 50;
static constexpr uint32_t NTC_FAULT_HIGH_MV = 3290;

static constexpr uint32_t VOLTAGE_GUARD_MS             = 1000;
static constexpr uint32_t VOLTAGE_DEADTIME_MS           = 200;
static constexpr uint32_t HOT_PELTIER_OFF_MS          = 10000;
static constexpr uint32_t ADAPTIVE_UPWARD_DELAY_MS      = 5000;
static constexpr uint32_t BATTERY_VALID_TTL_MS          = 5000;
static constexpr uint32_t SETTINGS_SAVE_DEBOUNCE_MS     = 10000;

static constexpr uint32_t USER_CONTROL_GUARD_MS          = 250;

static constexpr int HOT_LIMIT_MIN = 40;
static constexpr int HOT_LIMIT_MAX = 50;
static constexpr int BATTERY_LIMIT_MIN = 20;
static constexpr int BATTERY5_LIMIT_MAX = 48;
static constexpr int BATTERY12_LIMIT_MIN = 22;
static constexpr int BATTERY12_LIMIT_MAX = 50;
static constexpr float BATTERY_HYSTERESIS_C = 0.8f;

static constexpr uint8_t FAN_PWM_MIN = 50;
static constexpr uint8_t FAN_PWM_MAX = 100;
static constexpr uint8_t FAN_PWM_STEP = 10;
static constexpr uint32_t FAN_PWM_FREQ = 25000;
static constexpr uint8_t FAN_PWM_RESOLUTION = 8;
static constexpr uint32_t PELTIER_PWM_FREQ = 20000;
static constexpr uint8_t PELTIER_PWM_RESOLUTION = 8;
static constexpr uint32_t PELTIER_SOFT_START_MS = 5000;

WebServer webServer(80);
Preferences prefs;
Adafruit_NeoPixel leds(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);
OneButton touchButton(PIN_TOUCH, false, false);

BLEServer* bleServer = nullptr;
BLECharacteristic* rxCharacteristic = nullptr;
BLECharacteristic* txCharacteristic = nullptr;
volatile bool bleClientConnected = false;
String bleTxQueue[BLE_TX_QUEUE_SIZE];
size_t bleTxHead = 0;
size_t bleTxTail = 0;
size_t bleTxCount = 0;
uint32_t bleNextNotifyAt = 0;

char bleRxQueue[BLE_RX_QUEUE_SIZE][BLE_RX_BUFFER_SIZE];
std::atomic<size_t> bleRxHead{0};
std::atomic<size_t> bleRxTail{0};

volatile bool bleQueueResetPending = false;

bool onlineOtaStartPending = false;
uint32_t onlineOtaRequestedAt = 0;
uint8_t onlineWifiConnectAttempt = 0;
uint32_t stateSequence = 0;

enum class Voltage : uint8_t {
  V5 = 0,
  V9 = 1,
  V12 = 2,
};

enum class LedMode : uint8_t {
  STATIC = 1,
  BREATH = 2,
  CENTER_OUT = 3,
  OUTER_IN = 4,
  DUAL_CHASE = 5,
  LEFT_RIGHT = 6,
  RAINBOW = 7,
};

enum class VoltageRequestSource : uint8_t {
  MANUAL,
  TOUCH,
  BATTERY,
  HOTSTEP,
  ADAPTIVE,
  SAFETY,
};

Voltage currentVoltage = Voltage::V5;
Voltage pendingVoltage = Voltage::V5;

bool fanOn = false;
bool peltierOn = false;
bool peltierSoftStarting = false;
uint32_t peltierSoftStartAtMs = 0;
uint8_t lastPeltierDuty = 0;
bool adaptiveOn = false;
Voltage adaptiveCeiling = Voltage::V12;

bool ntcValid = false;
float hotsideC = NAN;
uint32_t ntcMv = 0;
uint32_t lastNtcReadMs = 0;

bool batteryValid = false;
float batteryC = NAN;
uint32_t lastBatteryRxMs = 0;
int lastBatteryZone = -1;

int hotLimit = 45;
int battery5Limit = 25;
int battery12Limit = 35;

uint8_t fanPwm = 100;

bool rgbOn = false;
LedMode rgbMode = LedMode::STATIC;
uint8_t rgbBrightness = 100;
uint32_t rgbColor = 0xFFFFFF;
uint32_t color5V = 0xFF0000;
uint32_t color9V = 0x00FF00;
uint32_t color12V = 0x0000FF;

bool settingsDirty = false;
uint32_t settingsDirtyAt = 0;
bool prefsReady = false;

bool voltageTransition = false;
uint32_t voltageTransitionAt = 0;
uint32_t lastVoltageChangeMs = 0;
bool voltageGuardArmed = false;

String lastSafetyReasonSent;

bool voltageIndicatorActive = false;
uint32_t voltageIndicatorUntil = 0;
uint32_t lastLedFrameAtMs = 0;

bool hotProtectionActive = false;
bool adaptiveHotStepPending = false;
uint32_t adaptiveHotStepDueMs = 0;
uint32_t peltierHotOffUntilMs = 0;
bool adaptiveResumePeltierAfterHotStep = false;

bool adaptiveUpPending = false;
Voltage adaptiveUpTarget = Voltage::V5;
uint32_t adaptiveUpDueMs = 0;

bool adaptiveDownPending = false;
Voltage adaptiveDownTarget = Voltage::V5;

// ========================= OTA STATE — PROTECTED ===========================
bool otaActive = false;
OtaState otaState = OtaState::BOOT_AP;
uint32_t otaRestartAt = 0;
uint32_t otaFailureIndicatorUntil = 0;
String otaResult = "NONE";

String cloudSsid;
String cloudPassword;
String cloudUrl;
String cloudVersion;

WiFiClientSecure cloudClient;
size_t localOtaBytesReceived = 0;
size_t localOtaExpectedBytes = 0;
bool localOtaStarted = false;

bool bleStartPending = true;
uint8_t bleStartAttempts = 0;
uint32_t bleStartRetryAt = 0;
uint32_t cloudPhaseStartedAt = 0;

OnlineOtaFlow onlineOtaFlow = OnlineOtaFlow::IDLE;
bool onlineResultIndicatorActive = false;
uint32_t onlineResultIndicatorStartedAt = 0;
uint32_t onlineResultIndicatorColor = 0;
OnlineOtaIndicatorNext onlineResultIndicatorNext = OnlineOtaIndicatorNext::NONE;
uint32_t onlineResultIndicatorLastPhase = 0xFFFFFFFFUL;
bool onlineHttpActive = false;
int onlineHttpResponseCode = 0;
int onlineHttpExpectedBytes = 0;
bool onlineUpdateStarted = false;
HTTPClient onlineHttpClient;

// ====================== OTA DIAGNOSTICS — PROTECTED =========================
String otaDiagnosticStage = "IDLE";
String otaDiagnosticError = "NONE";
int otaDiagnosticErrorCode = 0;
uint8_t otaDiagnosticProgress = 0;
uint8_t otaDiagnosticLastPublishedProgress = 255;

// ==================== END OTA DIAGNOSTIC CORE — PROTECTED =================

bool bootApActive = false;
bool bootApPending = true;
uint32_t bootApStartedAt = 0;
bool runtimeFeaturesReady = false;
bool bootApHadClient = false;
bool bootApClientSafe = false;

volatile bool bleSyncPending = false;

volatile bool bleCommandGateOpen = false;

bool userControlGuardArmed = false;
uint32_t lastUserControlCommandMs = 0;

uint32_t ledBlinkAt = 0;
bool ledBlinkPhase = false;
std::atomic<bool> bleLedResumePending{false};

// Local OTA result indicator is separate from normal LED effects.
// Success: green blink for the result window, then restart.
// Failure: red blink through the existing OTA failure indicator path.
bool localOtaResultIndicatorActive = false;
uint32_t localOtaResultIndicatorStartedAt = 0;
uint32_t localOtaResultIndicatorLastPhase = 0xFFFFFFFFUL;

char bleRxBuffer[BLE_RX_BUFFER_SIZE];
size_t bleRxLength = 0;

static void showAll(uint32_t color, uint8_t brightness = 255);
static void sendText(const String& line);
static void sendDelta(const char* key, const String& value);
static void setOtaState(OtaState state);
static const char* otaStateText(OtaState state);
static void sendAck(const String& item, bool ok, const String& reason);
static void sendFullSync();
static void handleCommand(const String& command);
static void feedBleRx(const String& incoming);
static void startBle();
static void startLocalOtaAp();
static void stopLocalOtaAp();
static void startOnlineOta();
static void processOnlineOta();
static bool requestVoltage(Voltage target, VoltageRequestSource source, bool bypassGuard = false);
static void publishSafetyReason();
static void evaluateAdaptiveBattery();
static void serviceBleTx();
static void clearBleTxQueue();
static void clearBleRxQueue();
static void processBleRxQueue();
static void processPendingOnlineOtaStart();
static void setOtaDiagnosticStage(const String& stage);
static void setOtaDiagnosticProgress(uint8_t progress);
static void setOtaDiagnosticError(int errorCode, const String& error);
static String otaFailureResult(const String& mode, const String& stage, int errorCode, const String& error, uint8_t progress);
static String otaSuccessResult(const String& mode, const String& stage, const String& version);
static void failLocalOta(const String& stage, int errorCode, const String& error);
static void failOnlineOta(const String& stage, int errorCode, const String& error);
static bool validFirmwareVersion(const String& value);
static void clearCloudCredentials();
static void resetBlinkState();
static void startLocalOtaResultIndicator();
static bool processLocalOtaResultIndicator();
static void initializeRuntimeFeatures();

static const char* voltageToString(Voltage value) {
  switch (value) {
    case Voltage::V5:  return "5V";
    case Voltage::V9:  return "9V";
    case Voltage::V12: return "12V";
  }
  return "5V";
}

static Voltage voltageFromString(String value) {
  value.trim();
  value.toUpperCase();
  if (value == "12V") return Voltage::V12;
  if (value == "9V") return Voltage::V9;
  return Voltage::V5;
}

static bool isHigherVoltage(Voltage a, Voltage b) {
  return static_cast<uint8_t>(a) > static_cast<uint8_t>(b);
}

static Voltage lowerVoltage(Voltage value) {
  if (value == Voltage::V12) return Voltage::V9;
  return Voltage::V5;
}

static Voltage clampToAdaptiveCeiling(Voltage target) {
  return isHigherVoltage(target, adaptiveCeiling) ? adaptiveCeiling : target;
}

static String boolText(bool value) {
  return value ? "ON" : "OFF";
}

static String formatTemperature(float value, bool valid) {
  if (!valid || !isfinite(value)) return "--";
  return String(value, 1);
}

static const char* resetReasonText() {
  // Arduino-ESP32 Core 3.3.7 has no public ESP.getResetReason() API.
  // Do not substitute an invented or low-level lifecycle API here.
  return "UNAVAILABLE_CORE_3_3_7";
}

static bool parseHexColor(String value, uint32_t& output) {
  value.trim();
  if (value.startsWith("#")) value.remove(0, 1);
  if (value.length() != 6) return false;

  uint32_t parsed = 0;
  for (size_t i = 0; i < 6; ++i) {
    const char c = value.charAt(i);
    uint8_t digit = 0;

    if (c >= '0' && c <= '9') {
      digit = static_cast<uint8_t>(c - '0');
    } else if (c >= 'A' && c <= 'F') {
      digit = static_cast<uint8_t>(c - 'A' + 10);
    } else if (c >= 'a' && c <= 'f') {
      digit = static_cast<uint8_t>(c - 'a' + 10);
    } else {
      return false;
    }

    parsed = (parsed << 4) | digit;
  }

  output = parsed;
  return true;
}

static String colorToHex(uint32_t color) {
  char buffer[8];
  snprintf(buffer, sizeof(buffer), "#%06lX", static_cast<unsigned long>(color & 0xFFFFFFUL));
  return String(buffer);
}

static bool parseIntRange(const String& raw, int minValue, int maxValue, int& output) {
  String value = raw;
  value.trim();
  if (value.isEmpty()) return false;

  bool negative = false;
  size_t index = 0;
  if (value.charAt(0) == '-') {
    negative = true;
    index = 1;
  }
  if (index >= value.length()) return false;

  long parsed = 0;
  for (; index < value.length(); ++index) {
    const char c = value.charAt(index);
    if (c < '0' || c > '9') return false;
    parsed = parsed * 10L + static_cast<long>(c - '0');
    if (parsed > 2147483647L) return false;
  }

  if (negative) parsed = -parsed;
  if (parsed < minValue || parsed > maxValue) return false;

  output = static_cast<int>(parsed);
  return true;
}

static bool parseFloatRange(const String& raw, float minValue, float maxValue, float& output) {
  String value = raw;
  value.trim();
  if (value.isEmpty()) return false;

  bool negative = false;
  bool decimalSeen = false;
  bool digitSeen = false;
  size_t index = 0;
  if (value.charAt(0) == '-') {
    negative = true;
    index = 1;
  }
  if (index >= value.length()) return false;

  double integerPart = 0.0;
  double fractionPart = 0.0;
  double fractionScale = 0.1;

  for (; index < value.length(); ++index) {
    const char c = value.charAt(index);
    if (c >= '0' && c <= '9') {
      digitSeen = true;
      const int digit = c - '0';
      if (!decimalSeen) {
        integerPart = integerPart * 10.0 + digit;
        if (integerPart > 1000000.0) return false;
      } else {
        fractionPart += digit * fractionScale;
        fractionScale *= 0.1;
      }
      continue;
    }

    if (c == '.' && !decimalSeen) {
      decimalSeen = true;
      continue;
    }

    return false;
  }

  if (!digitSeen) return false;
  double parsed = integerPart + fractionPart;
  if (negative) parsed = -parsed;
  if (parsed < minValue || parsed > maxValue) return false;

  output = static_cast<float>(parsed);
  return isfinite(output);
}

static void loadSettings() {
  prefsReady = prefs.begin("jexfyz", false);
  if (!prefsReady) return;

  hotLimit = constrain(prefs.getInt("hotLimit", 45), HOT_LIMIT_MIN, HOT_LIMIT_MAX);
  battery5Limit = constrain(
      prefs.getInt("batt5", 25),
      BATTERY_LIMIT_MIN,
      BATTERY5_LIMIT_MAX);
  battery12Limit = constrain(
      prefs.getInt("batt12", 35),
      BATTERY12_LIMIT_MIN,
      BATTERY12_LIMIT_MAX);

  if (battery12Limit < battery5Limit + 2) {
    battery12Limit = min(BATTERY12_LIMIT_MAX, battery5Limit + 2);
  }

  rgbOn = prefs.getBool("rgbOn", false);
  const int storedMode = prefs.getUChar("rgbMode", 1);
  rgbMode = static_cast<LedMode>(constrain(storedMode, 1, 7));
  rgbBrightness = prefs.getUChar("brightness", 100);
  if (rgbBrightness > 100) rgbBrightness = 100;

  rgbColor = prefs.getUInt("rgbColor", 0xFFFFFF);
  color5V = prefs.getUInt("color5V", 0xFF0000);
  color9V = prefs.getUInt("color9V", 0x00FF00);
  color12V = prefs.getUInt("color12V", 0x0000FF);

  otaResult = prefs.getString("lastOta", "NONE");
}

static void markSettingsDirty() {
  settingsDirty = true;
  settingsDirtyAt = millis();
}

static void saveSettingsNow() {
  if (!prefsReady) return;

  prefs.putInt("hotLimit", hotLimit);
  prefs.putInt("batt5", battery5Limit);
  prefs.putInt("batt12", battery12Limit);
  prefs.putBool("rgbOn", rgbOn);
  prefs.putUChar("rgbMode", static_cast<uint8_t>(rgbMode));
  prefs.putUChar("brightness", rgbBrightness);
  prefs.putUInt("rgbColor", rgbColor);
  prefs.putUInt("color5V", color5V);
  prefs.putUInt("color9V", color9V);
  prefs.putUInt("color12V", color12V);
  settingsDirty = false;
}

static void processSettingsPersistence() {
  if (settingsDirty && millis() - settingsDirtyAt >= SETTINGS_SAVE_DEBOUNCE_MS) {
    saveSettingsNow();
  }
}

static void saveOtaResult(const String& result) {
  otaResult = result;
  if (prefsReady) prefs.putString("lastOta", otaResult);
}

static uint8_t fanDutyFromPercent(uint8_t percent) {
  return static_cast<uint8_t>(lroundf((percent / 100.0f) * 255.0f));
}

static void writeFanPwm() {
  if (!fanOn) {
    if (runtimeFeaturesReady) {
      ledcWrite(PIN_FAN, 0);
    }
    return;
  }

  ledcWrite(PIN_FAN, fanDutyFromPercent(fanPwm));
}

static void stopPeltierOutput() {
  peltierSoftStarting = false;
  peltierSoftStartAtMs = 0;
  if (lastPeltierDuty != 0) {
    ledcWrite(PIN_PELTIER, 0);
    lastPeltierDuty = 0;
  }
}

static void beginPeltierSoftStart() {
  peltierSoftStarting = true;
  peltierSoftStartAtMs = millis();
  if (lastPeltierDuty != 0) {
    ledcWrite(PIN_PELTIER, 0);
    lastPeltierDuty = 0;
  }
}

static void processPeltierSoftStart() {
  if (!peltierOn) {
    stopPeltierOutput();
    return;
  }

  if (!peltierSoftStarting) {
    if (lastPeltierDuty != 255) {
      ledcWrite(PIN_PELTIER, 255);
      lastPeltierDuty = 255;
    }
    return;
  }

  const uint32_t elapsed = millis() - peltierSoftStartAtMs;
  if (elapsed >= PELTIER_SOFT_START_MS) {
    peltierSoftStarting = false;
    if (lastPeltierDuty != 255) {
      ledcWrite(PIN_PELTIER, 255);
      lastPeltierDuty = 255;
    }
    return;
  }

  const uint8_t duty = static_cast<uint8_t>((elapsed * 255UL) / PELTIER_SOFT_START_MS);
  if (duty != lastPeltierDuty) {
    ledcWrite(PIN_PELTIER, duty);
    lastPeltierDuty = duty;
  }
}

static bool hotsideProtectionActiveNow() {
  return ntcValid && isfinite(hotsideC) && hotsideC >= static_cast<float>(hotLimit);
}

static bool safetyAllowsPeltier() {
  if (otaActive) return false;
  if (!ntcValid) return false;
  if (hotsideProtectionActiveNow()) return false;
  if (!fanOn) return false;

  if (adaptiveOn &&
      peltierHotOffUntilMs != 0 &&
      millis() < peltierHotOffUntilMs) {
    return false;
  }

  return true;
}

static void applyCoolingOutputs() {
  const bool previousFan = fanOn;
  const bool previousPeltier = peltierOn;
  const uint8_t previousPwm = fanPwm;

  if (!ntcValid) {
    fanOn = true;
    fanPwm = 100;
    peltierOn = false;
  }

  if (!fanOn) peltierOn = false;
  if (peltierOn && !safetyAllowsPeltier()) peltierOn = false;

  if (peltierOn && !previousPeltier) {
    beginPeltierSoftStart();
  } else if (!peltierOn) {
    stopPeltierOutput();
  }

  processPeltierSoftStart();
  writeFanPwm();

  if (fanOn != previousFan || peltierOn != previousPeltier || fanPwm != previousPwm) {
    ++stateSequence;
  }
  if (fanOn != previousFan) sendDelta("FAN", boolText(fanOn));
  if (peltierOn != previousPeltier) sendDelta("PELTIER", boolText(peltierOn));
  if (fanPwm != previousPwm) sendDelta("FAN_SPEED", String(fanPwm));
}

// Physical LED path: RIGHT TOP -> RIGHT BOTTOM -> LEFT BOTTOM -> LEFT TOP.
static constexpr uint8_t RIGHT_PIXELS[4] = {0, 1, 2, 3};
static constexpr uint8_t LEFT_PIXELS[4] = {7, 6, 5, 4};

static uint32_t scaleColor(uint32_t color, uint8_t brightness) {
  const uint8_t r = static_cast<uint8_t>((((color >> 16) & 0xFFU) * brightness) / 255U);
  const uint8_t g = static_cast<uint8_t>((((color >> 8) & 0xFFU) * brightness) / 255U);
  const uint8_t b = static_cast<uint8_t>(((color & 0xFFU) * brightness) / 255U);
  return leds.Color(r, g, b);
}

static uint32_t blendLevel(uint32_t color, uint8_t baseBrightness, float level) {
  if (level <= 0.0f) return 0;
  if (level > 1.0f) level = 1.0f;
  const uint8_t brightness = static_cast<uint8_t>(lroundf(baseBrightness * level));
  return scaleColor(color, brightness);
}

static void showAll(uint32_t color, uint8_t brightness) {
  const uint32_t scaled = scaleColor(color, brightness);
  for (uint8_t i = 0; i < LED_COUNT; ++i) {
    leds.setPixelColor(i, scaled);
  }
  leds.show();
}

static uint32_t currentVoltageColor() {
  switch (currentVoltage) {
    case Voltage::V5:  return color5V;
    case Voltage::V9:  return color9V;
    case Voltage::V12: return color12V;
  }
  return color5V;
}

static uint8_t hardwareBrightness() {
  return static_cast<uint8_t>(map(rgbBrightness, 0, 100, 0, 255));
}

static void clearAllLeds() {
  leds.clear();
  leds.show();
}

static void stopUserLedOutput() {
  clearAllLeds();
  lastLedFrameAtMs = 0;
}

static void resetLedAnimationState() {
  lastLedFrameAtMs = 0;
}

static void startUserLedEffect() {
  resetLedAnimationState();
}

static void renderBlink(uint32_t color) {
  if (millis() - ledBlinkAt < 250) return;

  ledBlinkAt = millis();
  ledBlinkPhase = !ledBlinkPhase;
  showAll(ledBlinkPhase ? color : 0x000000, 255);}

static void startOnlineOtaResultIndicator(
    uint32_t color,
    OnlineOtaIndicatorNext nextAction) {
  onlineResultIndicatorActive = true;
  onlineResultIndicatorStartedAt = millis();
  onlineResultIndicatorColor = color;
  onlineResultIndicatorNext = nextAction;
  onlineResultIndicatorLastPhase = 0xFFFFFFFFUL;
  // Keep the LED OFF until the indicator state is actually processed.
  // This prevents a false static-color flash at the stage boundary.
  showAll(0x000000, 0);
}

static bool processOnlineOtaResultIndicator() {
  if (!onlineResultIndicatorActive) return false;

  const uint32_t elapsed = millis() - onlineResultIndicatorStartedAt;
  if (elapsed < ONLINE_OTA_RESULT_INDICATOR_MS) {
    // FIX: Fase dihitung tiap 250ms secara presisi (ON, OFF, ON, OFF)
    const uint32_t phase = elapsed / 250U;
    if (phase != onlineResultIndicatorLastPhase) {
      onlineResultIndicatorLastPhase = phase;
      const bool ledOn = (phase == 0U || phase == 2U);
      showAll(ledOn ? onlineResultIndicatorColor : 0x000000, 255);
    }
    return true;
  }

  if (onlineResultIndicatorLastPhase != 0xFFFFFFFFUL) {
    showAll(0x000000, 0);
    onlineResultIndicatorLastPhase = 0xFFFFFFFFUL;
  }
  onlineResultIndicatorActive = false;

  const OnlineOtaIndicatorNext nextAction = onlineResultIndicatorNext;
  onlineResultIndicatorNext = OnlineOtaIndicatorNext::NONE;

  switch (nextAction) {
    case OnlineOtaIndicatorNext::PREPARE_DOWNLOAD:
      onlineOtaFlow = OnlineOtaFlow::DOWNLOAD;
      prepareOnlineOtaDownload();
      break;

    case OnlineOtaIndicatorNext::DOWNLOAD:
      onlineOtaFlow = OnlineOtaFlow::DOWNLOAD;
      setOtaState(OtaState::CLOUD_DOWNLOADING);
      setOtaDiagnosticStage("ONLINE_DOWNLOAD");
      break;

    case OnlineOtaIndicatorNext::FLASH_FINALIZE:
      onlineOtaFlow = OnlineOtaFlow::FLASH_FINALIZE;
      setOtaState(OtaState::CLOUD_APPLYING);
      setOtaDiagnosticStage("ONLINE_FLASH_FINALIZE");
      break;

    case OnlineOtaIndicatorNext::RESTART:
      onlineOtaFlow = OnlineOtaFlow::IDLE;
      otaActive = true;
      setOtaState(OtaState::SUCCESS);
      otaRestartAt = millis() + 200;
      break;

    case OnlineOtaIndicatorNext::RECOVER:
      onlineOtaFlow = OnlineOtaFlow::IDLE;
      onlineUpdateStarted = false;
      onlineHttpActive = false;
      clearCloudCredentials();
      otaActive = false;
      otaState = OtaState::IDLE;
      // FIX: Kembalikan kontrol LED normal jika OTA Gagal
      bleLedResumePending.store(true, std::memory_order_release);
      startBle();
      break;

    case OnlineOtaIndicatorNext::NONE:
      break;
  }

  return true;
}

static bool validFirmwareVersion(const String& value) {
  String normalized = value;
  normalized.trim();
  normalized.toUpperCase();
  if (!normalized.startsWith("V") || normalized.length() < 2) return false;

  bool dotSeen = false;
  bool digitSeen = false;
  for (size_t i = 1; i < normalized.length(); ++i) {
    const char c = normalized.charAt(i);
    if (c >= '0' && c <= '9') {
      digitSeen = true;
      continue;
    }
    if (c == '.' && !dotSeen && i > 1 && i + 1 < normalized.length()) {
      dotSeen = true;
      continue;
    }
    return false;
  }
  return digitSeen;
}

static void resetBlinkState() {
  ledBlinkAt = 0;
  ledBlinkPhase = false;
}

static void startLocalOtaResultIndicator() {
  localOtaResultIndicatorActive = true;
  localOtaResultIndicatorStartedAt = millis();
  resetBlinkState();
  localOtaResultIndicatorLastPhase = 0xFFFFFFFFUL;

  // Start from a known OFF frame. processLeds() will render the
  // non-blocking green result blink on subsequent loop iterations.
  showAll(0x000000, 0);
}

static bool processLocalOtaResultIndicator() {
  if (!localOtaResultIndicatorActive) return false;

  const uint32_t elapsed = millis() - localOtaResultIndicatorStartedAt;

  // Four 250 ms phases: ON, OFF, ON, OFF.
  // Render only when the phase changes so leds.show() is not called on every
  // main-loop iteration.
  if (elapsed < 1000U) {
    const uint32_t phase = elapsed / 250U;
    if (phase != localOtaResultIndicatorLastPhase) {
      localOtaResultIndicatorLastPhase = phase;
      const bool ledOn = (phase == 0U || phase == 2U);
      showAll(ledOn ? 0x00FF00 : 0x000000, 255);
    }
    return true;
  }

  if (localOtaResultIndicatorLastPhase != 0xFFFFFFFFUL) {
    showAll(0x000000, 0);
    localOtaResultIndicatorLastPhase = 0xFFFFFFFFUL;
  }
  localOtaResultIndicatorActive = false;

  // Keep SUCCESS asserted while the normal OTA supervisor owns the
  // final restart boundary. No blocking delay is introduced here.
  otaRestartAt = millis() + OTA_RESTART_DELAY_MS;
  return true;
}

static void startVoltageIndicator() {
  if (!rgbOn) {
    voltageIndicatorActive = false;
    stopUserLedOutput();
    return;
  }

  voltageIndicatorActive = true;
  voltageIndicatorUntil = millis() + 1000;
  showAll(currentVoltageColor(), hardwareBrightness());
}

static void renderCustomLedFrame(uint32_t now) {
  if (now - lastLedFrameAtMs < LED_FRAME_INTERVAL_MS) return;
  lastLedFrameAtMs = now;

  leds.clear();
  const uint8_t baseBrightness = hardwareBrightness();
  const uint32_t baseColor = rgbColor;

  switch (rgbMode) {
    case LedMode::STATIC: {
      const uint32_t color = scaleColor(baseColor, baseBrightness);
      for (uint8_t i = 0; i < LED_COUNT; ++i) leds.setPixelColor(i, color);
      break;
    }

    case LedMode::BREATH: {
      const float phase = (now % 2200UL) / 2200.0f;
      const float level = 0.12f + 0.88f * (0.5f + 0.5f * sinf(phase * 2.0f * PI));
      const uint32_t color = blendLevel(baseColor, baseBrightness, level);
      for (uint8_t i = 0; i < LED_COUNT; ++i) leds.setPixelColor(i, color);
      break;
    }

    case LedMode::CENTER_OUT: {
      const float progress = (now % 1600UL) / 400.0f;
      for (uint8_t pair = 0; pair < 4; ++pair) {
        const float distance = static_cast<float>(pair);
        float level = 0.0f;
        if (progress >= distance) level = 1.0f;
        else if (progress > distance - 1.0f) level = progress - distance + 1.0f;
        const uint32_t color = blendLevel(baseColor, baseBrightness, level);
        leds.setPixelColor(LEFT_PIXELS[3 - pair], color);
        leds.setPixelColor(RIGHT_PIXELS[3 - pair], color);
      }
      break;
    }

    case LedMode::OUTER_IN: {
      const float progress = (now % 1600UL) / 400.0f;
      for (uint8_t pair = 0; pair < 4; ++pair) {
        const float distance = static_cast<float>(pair);
        float level = 0.0f;
        if (progress >= distance) level = 1.0f;
        else if (progress > distance - 1.0f) level = progress - distance + 1.0f;
        const uint32_t color = blendLevel(baseColor, baseBrightness, level);
        leds.setPixelColor(LEFT_PIXELS[pair], color);
        leds.setPixelColor(RIGHT_PIXELS[pair], color);
      }
      break;
    }

    case LedMode::DUAL_CHASE: {
      static constexpr uint8_t sequence[] = {0, 1, 2, 3, 2, 1};
      const uint8_t position = static_cast<uint8_t>((now / 150UL) % 6U);
      const uint8_t pair = sequence[position];
      leds.setPixelColor(LEFT_PIXELS[pair], scaleColor(baseColor, baseBrightness));
      leds.setPixelColor(RIGHT_PIXELS[pair], scaleColor(baseColor, baseBrightness));
      break;
    }

    case LedMode::LEFT_RIGHT: {
      const float phase = (now % 1800UL) / 1800.0f;
      const float wave = 0.5f + 0.5f * sinf(phase * 2.0f * PI);
      const float leftLevel = 0.10f + 0.90f * wave;
      const float rightLevel = 0.10f + 0.90f * (1.0f - wave);
      const uint32_t leftColor = blendLevel(baseColor, baseBrightness, leftLevel);
      const uint32_t rightColor = blendLevel(baseColor, baseBrightness, rightLevel);
      for (uint8_t i = 0; i < 4; ++i) {
        leds.setPixelColor(LEFT_PIXELS[i], leftColor);
        leds.setPixelColor(RIGHT_PIXELS[i], rightColor);
      }
      break;
    }

    case LedMode::RAINBOW: {
      // Whole-strip rainbow: all 8 LEDs share one color at a time.
      // One full hue cycle takes 30 seconds and changes smoothly.
      static constexpr uint32_t RAINBOW_CYCLE_MS = 30000UL;
      const uint32_t cyclePosition = now % RAINBOW_CYCLE_MS;
      const uint16_t hue = static_cast<uint16_t>(
          (static_cast<uint64_t>(cyclePosition) * 65535ULL) /
          RAINBOW_CYCLE_MS);
      const uint32_t color = Adafruit_NeoPixel::ColorHSV(
          hue, 255, baseBrightness);
      for (uint8_t i = 0; i < LED_COUNT; ++i) {
        leds.setPixelColor(i, color);
      }
      break;
    }
  }

  leds.show();
}

static void processLeds() {
  // V1.2: OFF ownership is latched per state so stopUserLedOutput()/showAll()
  // are not repeatedly invoked from the main loop while OTA owns the LEDs.
  // This keeps the LED renderer event-driven without changing the existing
  // Local/Online result-indicator state machines.
  static bool ledsForcedOff = false;

  if (localOtaResultIndicatorActive) {
    ledsForcedOff = false;
    processLocalOtaResultIndicator();
    return;
  }

  if (onlineResultIndicatorActive) {
    ledsForcedOff = false;
    processOnlineOtaResultIndicator();
    return;
  }

  if (otaState == OtaState::SUCCESS) {
    return;
  }

  if (otaState == OtaState::FAILED) {
    ledsForcedOff = false;
    if (millis() < otaFailureIndicatorUntil) {
      renderBlink(0xFF0000);
      return;
    }

    resetBlinkState();
    setOtaState(bootApActive ? OtaState::BOOT_AP : OtaState::IDLE);
    if (!bootApActive && !otaActive) startUserLedEffect();
    return;
  }

  if (onlineOtaStartPending) {
    if (!ledsForcedOff) {
      stopUserLedOutput();
      ledsForcedOff = true;
    }
    return;
  }

  if (otaState == OtaState::CLOUD_CONNECTING ||
      otaState == OtaState::CLOUD_DOWNLOADING ||
      otaState == OtaState::CLOUD_APPLYING) {
    if (!ledsForcedOff) {
      showAll(0x000000, 0);
      ledsForcedOff = true;
    }
    return;
  }

  if (bootApClientSafe || otaActive || otaState == OtaState::LOCAL_UPLOAD) {
    if (!ledsForcedOff) {
      stopUserLedOutput();
      ledsForcedOff = true;
    }
    return;
  }

  if (!ntcValid || hotsideProtectionActiveNow()) {
    ledsForcedOff = false;
    renderBlink(0xFF0000);
    return;
  }

  if (!rgbOn) {
    if (!ledsForcedOff) {
      stopUserLedOutput();
      ledsForcedOff = true;
    }
    return;
  }

  ledsForcedOff = false;

  if (voltageIndicatorActive) {
    if (millis() >= voltageIndicatorUntil) {
      voltageIndicatorActive = false;
      startUserLedEffect();
    } else if (millis() - lastLedFrameAtMs >= LED_FRAME_INTERVAL_MS) {
      lastLedFrameAtMs = millis();
      showAll(currentVoltageColor(), hardwareBrightness());
    }
    return;
  }

  renderCustomLedFrame(millis());
}

static bool voltageGuardAllows(bool bypassGuard) {
  if (bypassGuard || !voltageGuardArmed) return true;

  return millis() - lastVoltageChangeMs >= VOLTAGE_GUARD_MS;
}

static bool manualVoltageRequestAllowed(VoltageRequestSource source) {  if (!adaptiveOn) return true;
  return source == VoltageRequestSource::BATTERY ||
         source == VoltageRequestSource::HOTSTEP ||
         source == VoltageRequestSource::ADAPTIVE ||
         source == VoltageRequestSource::SAFETY;
}

static bool requestVoltage(Voltage target, VoltageRequestSource source, bool bypassGuard) {
  if (otaActive || voltageTransition) return false;
  if (!manualVoltageRequestAllowed(source)) return false;
  if (!voltageGuardAllows(bypassGuard)) return false;

  if (!ntcValid && target != Voltage::V5) return false;

  if (adaptiveOn) {
    target = clampToAdaptiveCeiling(target);
  }

  if (target == currentVoltage) return true;

  if (!bypassGuard) {
    voltageGuardArmed = true;
    lastVoltageChangeMs = millis();
  }

  pendingVoltage = target;
  voltageTransition = true;
  voltageTransitionAt = millis();

  digitalWrite(PIN_9V, OUTPUT_OFF);
  digitalWrite(PIN_12V, OUTPUT_OFF);

  return true;
}

static void completeVoltageTransition() {
  if (!voltageTransition) return;
  if (millis() - voltageTransitionAt < VOLTAGE_DEADTIME_MS) return;

  currentVoltage = pendingVoltage;
  digitalWrite(PIN_9V, currentVoltage == Voltage::V9 ? OUTPUT_ON : OUTPUT_OFF);
  digitalWrite(PIN_12V, currentVoltage == Voltage::V12 ? OUTPUT_ON : OUTPUT_OFF);

  voltageTransition = false;

  if (!voltageGuardArmed) {
    voltageGuardArmed = true;
    lastVoltageChangeMs = millis();
  }

  if (currentVoltage != Voltage::V12 || adaptiveOn) {
    fanPwm = 100;
  }

  applyCoolingOutputs();
  startVoltageIndicator();

  ++stateSequence;
  sendDelta("VOLTAGE", voltageToString(currentVoltage));

  if (adaptiveOn && currentVoltage == Voltage::V5 && batteryValid && ntcValid) {
    evaluateAdaptiveBattery();
  }
}

static float ntcTemperatureFromOhms(float ohms) {
  const float t0 = NTC_NOMINAL_C + 273.15f;
  const float invT = (1.0f / t0) + (logf(ohms / NTC_NOMINAL_OHMS) / NTC_BETA);
  return (1.0f / invT) - 273.15f;
}

static void sampleNtcIfDue() {
  if (millis() - lastNtcReadMs < NTC_READ_INTERVAL_MS) return;
  lastNtcReadMs = millis();

  ntcMv = analogReadMilliVolts(PIN_NTC);

  if (ntcMv <= NTC_FAULT_LOW_MV || ntcMv >= NTC_FAULT_HIGH_MV) {
    ntcValid = false;
    hotsideC = NAN;
    return;
  }

  float voltage = ntcMv / 1000.0f;
  if (!isfinite(voltage) || voltage <= 0.0f) {
    ntcValid = false;
    hotsideC = NAN;
    return;
  }
  if (voltage > 3.28f) voltage = 3.28f;

  const float ntcOhms = NTC_FIXED_OHMS * ((3.3f / voltage) - 1.0f);

  if (!isfinite(ntcOhms) || ntcOhms <= 1.0f) {
    ntcValid = false;
    hotsideC = NAN;
    return;
  }

  const float temperature = ntcTemperatureFromOhms(ntcOhms);
  if (!isfinite(temperature)) {
    ntcValid = false;
    hotsideC = NAN;
    return;
  }

  ntcValid = true;
  hotsideC = temperature;
}

static void forceSafeFiveVolt() {
  const bool previousAdaptive = adaptiveOn;
  const Voltage previousCeiling = adaptiveCeiling;
  const uint8_t previousPwm = fanPwm;
  const bool previousFan = fanOn;
  const bool previousPeltier = peltierOn;

  adaptiveUpPending = false;
  adaptiveDownPending = false;
  adaptiveHotStepPending = false;
  peltierHotOffUntilMs = 0;
  adaptiveResumePeltierAfterHotStep = false;

  fanOn = true;
  peltierOn = false;
  peltierSoftStarting = false;
  peltierSoftStartAtMs = 0;
  adaptiveOn = false;
  adaptiveCeiling = Voltage::V12;
  fanPwm = 100;

  if (voltageTransition) {

    pendingVoltage = Voltage::V5;
  } else if (currentVoltage != Voltage::V5) {
    requestVoltage(Voltage::V5, VoltageRequestSource::SAFETY, true);
  }

  applyCoolingOutputs();

  if (previousFan != fanOn) sendDelta("FAN", boolText(fanOn));
  if (previousPeltier != peltierOn) sendDelta("PELTIER", boolText(peltierOn));
  if (previousAdaptive != adaptiveOn) sendDelta("ADAPTIVE", boolText(adaptiveOn));
  if (previousCeiling != adaptiveCeiling) {
    sendDelta("ADAPTIVE_CEILING", voltageToString(adaptiveCeiling));
  }
  if (previousPwm != fanPwm) sendDelta("FAN_SPEED", String(fanPwm));
  publishSafetyReason();
}

static void enforceSafety() {
  if (!ntcValid) {
    forceSafeFiveVolt();
    hotProtectionActive = false;
    return;
  }

  hotProtectionActive = hotsideProtectionActiveNow();

  if (hotProtectionActive) {
    const bool wasPeltierOn = peltierOn;
    const bool wasFanOn = fanOn;
    const uint8_t wasFanPwm = fanPwm;

    fanOn = true;
    fanPwm = 100;
    if (peltierOn) {
      peltierOn = false;
      stopPeltierOutput();
      sendDelta("PELTIER", "OFF");
    }
    if (!wasFanOn) sendDelta("FAN", "ON");
    if (wasFanPwm != 100) sendDelta("FAN_SPEED", "100");

    if (adaptiveOn && !adaptiveHotStepPending) {
      const uint32_t now = millis();

      adaptiveHotStepPending = true;
      peltierHotOffUntilMs = now + HOT_PELTIER_OFF_MS;
      adaptiveHotStepDueMs = peltierHotOffUntilMs;

      if (!adaptiveResumePeltierAfterHotStep) {
        adaptiveResumePeltierAfterHotStep = fanOn && wasPeltierOn;
      }

      adaptiveUpPending = false;
      adaptiveDownPending = false;
    }
  }

  if (adaptiveResumePeltierAfterHotStep &&
      adaptiveOn &&
      fanOn &&
      !voltageTransition &&
      peltierHotOffUntilMs != 0 &&
      millis() >= peltierHotOffUntilMs &&
      !hotsideProtectionActiveNow()) {
    adaptiveResumePeltierAfterHotStep = false;
    peltierHotOffUntilMs = 0;
    peltierOn = true;
  }

  if (peltierOn && !fanOn) {
    peltierOn = false;
    stopPeltierOutput();
  }

  applyCoolingOutputs();
}

static void processAdaptiveHotTimer() {
  if (!adaptiveOn || !ntcValid || !adaptiveHotStepPending) return;
  if (voltageTransition) return;
  if (millis() < adaptiveHotStepDueMs) return;

  adaptiveHotStepPending = false;
  adaptiveUpPending = false;
  adaptiveDownPending = false;
  const bool wasPeltierOn = peltierOn;
  peltierOn = false;
  stopPeltierOutput();
  if (wasPeltierOn) sendDelta("PELTIER", "OFF");

  if (!hotsideProtectionActiveNow()) {
    const bool resume = adaptiveResumePeltierAfterHotStep && fanOn;

    adaptiveResumePeltierAfterHotStep = false;
    peltierHotOffUntilMs = 0;

    if (resume) {
      peltierOn = true;
    }

    applyCoolingOutputs();
    return;
  }

  if (currentVoltage == Voltage::V12) {
    adaptiveCeiling = Voltage::V9;
    sendDelta("ADAPTIVE_CEILING", "9V");
  } else if (currentVoltage == Voltage::V9) {
    adaptiveCeiling = Voltage::V5;
    sendDelta("ADAPTIVE_CEILING", "5V");
  } else {

    adaptiveOn = false;
    adaptiveCeiling = Voltage::V12;
    adaptiveResumePeltierAfterHotStep = false;
    peltierHotOffUntilMs = 0;
    fanPwm = 100;

    applyCoolingOutputs();
    sendDelta("ADAPTIVE", "OFF");
    sendDelta("ADAPTIVE_CEILING", voltageToString(adaptiveCeiling));
    sendDelta("FAN_SPEED", String(fanPwm));
    return;
  }

  const Voltage target = lowerVoltage(currentVoltage);
  const bool accepted = requestVoltage(target, VoltageRequestSource::HOTSTEP, true);

  if (!accepted) {

    sendDelta("SAFETY_REASON", "VOLTAGE_STEP_FAILED");
  }
}

static String safetyReason() {
  if (!ntcValid) return "NTC_ERROR";
  if (hotsideProtectionActiveNow()) return "HOTSIDE_PROTECTION";
  if (peltierOn && !fanOn) return "FAN_REQUIRED";
  return "NONE";
}

static int batteryZoneFor(float temperature) {
  if (temperature < battery5Limit) return 5;
  if (temperature < battery12Limit) return 9;
  return 12;
}

static int batteryZoneForStable(float temperature) {
  if (!isfinite(temperature)) return 5;
  if (lastBatteryZone != 5 &&
      lastBatteryZone != 9 &&
      lastBatteryZone != 12) {
    return batteryZoneFor(temperature);
  }

  switch (lastBatteryZone) {
    case 5:
      if (temperature >= battery12Limit + BATTERY_HYSTERESIS_C) return 12;
      if (temperature >= battery5Limit + BATTERY_HYSTERESIS_C) return 9;
      return 5;

    case 9:
      if (temperature < battery5Limit - BATTERY_HYSTERESIS_C) return 5;
      if (temperature >= battery12Limit + BATTERY_HYSTERESIS_C) return 12;
      return 9;

    case 12:
      if (temperature < battery5Limit - BATTERY_HYSTERESIS_C) return 5;
      if (temperature < battery12Limit - BATTERY_HYSTERESIS_C) return 9;
      return 12;
  }

  return batteryZoneFor(temperature);
}

static Voltage batteryZoneVoltage(int zone) {
  if (zone >= 12) return Voltage::V12;
  if (zone >= 9) return Voltage::V9;
  return Voltage::V5;
}

static void updateBatteryValidity() {
  if (!batteryValid) return;
  if (millis() - lastBatteryRxMs <= BATTERY_VALID_TTL_MS) return;

  batteryValid = false;
  batteryC = NAN;
  lastBatteryZone = -1;
  adaptiveUpPending = false;
  adaptiveDownPending = false;
  sendDelta("BATTERY", "--");
  sendDelta("BATTERY_VALID", "0");
}

static void scheduleAdaptiveBatteryTarget(Voltage target) {
  target = clampToAdaptiveCeiling(target);

  if (target == currentVoltage) {
    adaptiveUpPending = false;
    adaptiveDownPending = false;
    return;
  }

  if (isHigherVoltage(target, currentVoltage)) {
    adaptiveDownPending = false;
    adaptiveUpPending = true;
    adaptiveUpTarget = target;
    adaptiveUpDueMs = millis() + ADAPTIVE_UPWARD_DELAY_MS;
    return;
  }

  adaptiveUpPending = false;
  if (!requestVoltage(target, VoltageRequestSource::BATTERY, false)) {
    adaptiveDownPending = true;
    adaptiveDownTarget = target;
  }
}

static void evaluateAdaptiveBattery() {
  if (!adaptiveOn || !ntcValid || !batteryValid) return;
  if (adaptiveHotStepPending || hotsideProtectionActiveNow()) return;

  const int zone = batteryZoneForStable(batteryC);
  if (zone == lastBatteryZone) return;

  lastBatteryZone = zone;
  scheduleAdaptiveBatteryTarget(batteryZoneVoltage(zone));
}

static void processAdaptiveBatteryDown() {
  if (!adaptiveOn || !ntcValid || !batteryValid) return;
  if (adaptiveHotStepPending || !adaptiveDownPending || voltageTransition) return;
  if (hotsideProtectionActiveNow()) return;

  const Voltage liveTarget = clampToAdaptiveCeiling(batteryZoneVoltage(batteryZoneFor(batteryC)));

  if (liveTarget == currentVoltage) {
    adaptiveDownPending = false;
    return;
  }

  if (isHigherVoltage(liveTarget, currentVoltage)) {
    adaptiveDownPending = false;
    scheduleAdaptiveBatteryTarget(liveTarget);
    return;
  }

  adaptiveDownTarget = liveTarget;
  if (requestVoltage(adaptiveDownTarget, VoltageRequestSource::BATTERY, false)) {
    adaptiveDownPending = false;
  }
}

static void processAdaptiveUpTimer() {
  if (!adaptiveOn || !ntcValid || !batteryValid) return;
  if (adaptiveHotStepPending || !adaptiveUpPending || voltageTransition) return;

  if (hotsideProtectionActiveNow()) {
    adaptiveUpPending = false;
    return;
  }

  const Voltage liveTarget = clampToAdaptiveCeiling(batteryZoneVoltage(batteryZoneFor(batteryC)));
  if (liveTarget != adaptiveUpTarget) {
    adaptiveUpTarget = liveTarget;
    adaptiveUpDueMs = millis() + ADAPTIVE_UPWARD_DELAY_MS;
    return;
  }

  if (millis() < adaptiveUpDueMs) return;

  if (requestVoltage(adaptiveUpTarget, VoltageRequestSource::BATTERY, false)) {
    adaptiveUpPending = false;
  }
}

static void clearBleTxQueue() {
  for (size_t i = 0; i < BLE_TX_QUEUE_SIZE; ++i) bleTxQueue[i] = "";
  bleTxHead = 0;
  bleTxTail = 0;
  bleTxCount = 0;
  bleNextNotifyAt = millis();
}

static void clearBleRxQueue() {
  const size_t currentTail =
      bleRxTail.load(std::memory_order_acquire);
  bleRxHead.store(currentTail, std::memory_order_release);
}

static void enqueueBleRxPayload(const String& payload) {
  if (payload.isEmpty()) return;

  const size_t copyLength =
      (static_cast<size_t>(payload.length()) < (BLE_RX_BUFFER_SIZE - 1))
          ? static_cast<size_t>(payload.length())
          : (BLE_RX_BUFFER_SIZE - 1);
  const size_t currentHead =
      bleRxHead.load(std::memory_order_acquire);
  const size_t currentTail =
      bleRxTail.load(std::memory_order_relaxed);
  const size_t nextTail = (currentTail + 1) % BLE_RX_QUEUE_SIZE;

  if (nextTail != currentHead) {
    memcpy(bleRxQueue[currentTail], payload.c_str(), copyLength);
    bleRxQueue[currentTail][copyLength] = '\0';
    bleRxTail.store(nextTail, std::memory_order_release);
  }
}

static void processBleRxQueue() {
  if (!bleClientConnected) {
    clearBleRxQueue();
    return;
  }

  char payloadBuffer[BLE_RX_BUFFER_SIZE];
  payloadBuffer[0] = '\0';

  const size_t currentHead =
      bleRxHead.load(std::memory_order_relaxed);
  const size_t currentTail =
      bleRxTail.load(std::memory_order_acquire);

  if (currentHead != currentTail) {
    memcpy(payloadBuffer, bleRxQueue[currentHead], BLE_RX_BUFFER_SIZE);
    bleRxQueue[currentHead][0] = '\0';

    const size_t nextHead = (currentHead + 1) % BLE_RX_QUEUE_SIZE;
    bleRxHead.store(nextHead, std::memory_order_release);

    feedBleRx(String(payloadBuffer));
  }
}

static void sendText(const String& line) {
  if (txCharacteristic == nullptr || !BLEDevice::getInitialized()) return;
  if (!bleClientConnected || line.isEmpty()) return;

  String payload = line;
  if (payload.length() > 177) payload = payload.substring(0, 177);
  payload += '\n';

  if (bleTxCount < BLE_TX_QUEUE_SIZE) {
    bleTxQueue[bleTxTail] = payload;
    bleTxTail = (bleTxTail + 1) % BLE_TX_QUEUE_SIZE;
    ++bleTxCount;
  }
}

static void serviceBleTx() {
  if (bleQueueResetPending) {
    bleQueueResetPending = false;
    clearBleTxQueue();
    clearBleRxQueue();
  }

  if (!bleClientConnected || txCharacteristic == nullptr || !BLEDevice::getInitialized()) {
    return;
  }
  if (millis() < bleNextNotifyAt) return;

  String payload;
  if (bleTxCount > 0) {
    payload = bleTxQueue[bleTxHead];
    bleTxQueue[bleTxHead] = "";
    bleTxHead = (bleTxHead + 1) % BLE_TX_QUEUE_SIZE;
    --bleTxCount;
  }

  if (payload.isEmpty()) return;
  txCharacteristic->setValue(payload);
  txCharacteristic->notify();
  bleNextNotifyAt = millis() + BLE_TX_INTERVAL_MS;
}

static void sendDelta(const char* key, const String& value) {
  sendText(String(key) + ":" + value);
}

static void sendAck(const String& item, bool ok, const String& reason) {
  sendText("ACK:" + item + "=" + (ok ? "OK" : "DENIED") + ";REASON=" + reason);
}

static const char* otaStateText(OtaState state) {
  switch (state) {
    case OtaState::BOOT_AP:          return "BOOT_AP";
    case OtaState::LOCAL_UPLOAD:     return "LOCAL_UPLOAD";
    case OtaState::CLOUD_CONNECTING: return "CLOUD_CONNECTING";    case OtaState::CLOUD_DOWNLOADING:return "CLOUD_DOWNLOADING";
    case OtaState::CLOUD_APPLYING:   return "CLOUD_APPLYING";
    case OtaState::SUCCESS:          return "SUCCESS";
    case OtaState::FAILED:           return "FAILED";
    case OtaState::IDLE:             return "IDLE";
  }
  return "IDLE";
}

// ====================== OTA DIAGNOSTIC CORE — LOCKED ========================
static String sanitizeOtaDetail(String value) {
  value.replace("|", "/");
  value.replace("\r", " ");
  value.replace("\n", " ");
  value.trim();
  if (value.isEmpty()) value = "UNKNOWN";
  if (value.length() > 96) value = value.substring(0, 96);
  return value;
}

static void setOtaDiagnosticStage(const String& stage) {
  otaDiagnosticStage = stage;
  if (bleClientConnected) {
    sendDelta("OTA_STAGE", otaDiagnosticStage);
  }
}

static void setOtaDiagnosticProgress(uint8_t progress) {
  progress = constrain(progress, static_cast<uint8_t>(0), static_cast<uint8_t>(100));
  otaDiagnosticProgress = progress;
  if (bleClientConnected &&
      (otaDiagnosticLastPublishedProgress == 255 ||
       progress == 0 ||
       progress == 100 ||
       progress >= otaDiagnosticLastPublishedProgress + 5)) {
    otaDiagnosticLastPublishedProgress = progress;
    sendDelta("OTA_PROGRESS", String(progress));
  }
}

static void setOtaDiagnosticError(int errorCode, const String& error) {
  otaDiagnosticErrorCode = errorCode;
  otaDiagnosticError = sanitizeOtaDetail(error);
  if (bleClientConnected) {
    sendDelta("OTA_ERROR_CODE", String(otaDiagnosticErrorCode));
    sendDelta("OTA_ERROR", otaDiagnosticError);
  }
}

static String otaFailureResult(
    const String& mode,
    const String& stage,
    int errorCode,
    const String& error,
    uint8_t progress) {
  return "OTA_FAILED|MODE=" + mode +
         "|STAGE=" + stage +
         "|CODE=" + String(errorCode) +
         "|ERROR=" + sanitizeOtaDetail(error) +
         "|PROGRESS=" + String(progress);
}

static String otaSuccessResult(
    const String& mode,
    const String& stage,
    const String& version) {
  return "OTA_SUCCESS|MODE=" + mode +
         "|STAGE=" + stage +
         "|VERSION=" + sanitizeOtaDetail(version) +
         "|PROGRESS=100";
}

static constexpr int OTA_ERROR_WRITE = 1;
static constexpr int OTA_ERROR_SPACE = 4;
static constexpr int OTA_ERROR_MAGIC_BYTE = 8;
static constexpr int OTA_ERROR_STREAM = 6;
static constexpr int OTA_ERROR_SIZE = 5;
// ================= END OTA DIAGNOSTIC CORE — LOCKED ========================
static void setOtaState(OtaState state) {
  if (otaState == state) return;
  otaState = state;
  sendDelta("OTA_STATE", otaStateText(otaState));
}

static void sendFullSync() {
  clearBleTxQueue();
  sendText("<SYNC_START>");
  sendDelta("STATE_SEQ", String(stateSequence));
  sendDelta("VERSION", FW_VERSION);
  sendDelta("PROTOCOL", PROTOCOL_VERSION);
  sendDelta("BLE_SESSION", bleCommandGateOpen ? "READY" : "SYNC_REQUIRED");
  sendDelta("VOLTAGE", voltageToString(currentVoltage));
  sendDelta("FAN", boolText(fanOn));
  sendDelta("PELTIER", boolText(peltierOn));
  sendDelta("HOT", formatTemperature(hotsideC, ntcValid));
  sendDelta("BATTERY", formatTemperature(batteryC, batteryValid));
  sendDelta("BATTERY_VALID", batteryValid ? "1" : "0");
  sendDelta("ADAPTIVE", boolText(adaptiveOn));
  sendDelta("ADAPTIVE_CEILING", voltageToString(adaptiveCeiling));
  sendDelta("RGB", boolText(rgbOn));
  sendDelta("MODE", String(static_cast<uint8_t>(rgbMode)));
  sendDelta("BRIGHTNESS", String(rgbBrightness));
  sendDelta("RGB_COLOR", colorToHex(rgbColor));
  sendDelta("COLOR_5V", colorToHex(color5V));
  sendDelta("COLOR_9V", colorToHex(color9V));
  sendDelta("COLOR_12V", colorToHex(color12V));
  sendDelta("FAN_SPEED", String(fanPwm));
  const String reason = safetyReason();
  sendDelta("SAFETY", reason == "NONE" ? "OK" : "LOCKED");
  lastSafetyReasonSent = reason;
  sendDelta("SAFETY_REASON", reason);
  sendDelta("OTA_STATE", otaStateText(otaState));
  sendDelta("OTA_STAGE", otaDiagnosticStage);
  sendDelta("OTA_PROGRESS", String(otaDiagnosticProgress));
  sendDelta("OTA_ERROR_CODE", String(otaDiagnosticErrorCode));
  sendDelta("OTA_ERROR", otaDiagnosticError);
  sendDelta("LAST_OTA_RESULT", otaResult);
  sendDelta("RESET_REASON", resetReasonText());
  sendDelta("TEMP_HOT_LIMIT", String(hotLimit));
  sendDelta("BATT_5", String(battery5Limit));
  sendDelta("BATT_12", String(battery12Limit));
  sendDelta("NTC_MV", String(ntcMv));
  sendText("<SYNC_END>");
}

static void publishSafetyReason() {
  const String reason = safetyReason();
  if (reason != lastSafetyReasonSent) {
    lastSafetyReasonSent = reason;
    sendDelta("SAFETY", reason == "NONE" ? "OK" : "LOCKED");
    sendDelta("SAFETY_REASON", reason);
  }
}

static void sendHotsideTelemetry() {

  sendDelta("HOT", formatTemperature(hotsideC, ntcValid));
  publishSafetyReason();
}

static bool isUserControlCommand(const String& raw) {
  String c = raw;
  c.trim();
  c.toUpperCase();

  return c == "RGB:ON" ||
         c == "RGB:OFF" ||
         c.startsWith("MODE:") ||
         c.startsWith("BRIGHTNESS:") ||
         c.startsWith("COLOR:") ||
         c.startsWith("V5COLOR:") ||
         c.startsWith("V9COLOR:") ||
         c.startsWith("V12COLOR:") ||
         c.startsWith("HOTLIMIT:") ||
         c.startsWith("BATT5:") ||
         c.startsWith("BATT12:") ||
         c.startsWith("VOLTAGE:") ||
         c == "FAN:ON" ||
         c == "FAN:OFF" ||
         c == "PELTIER:ON" ||
         c == "PELTIER:OFF" ||
         c == "ADAPTIVE:ON" ||
         c == "ADAPTIVE:OFF" ||
         c.startsWith("FANSPEED:") ||
         c == "RESET:TEMPERATURE" ||
         c == "RESET:VOLTAGECOLORS";
}

static String controlAckItem(const String& raw) {
  String c = raw;
  c.trim();
  c.toUpperCase();

  if (c == "RGB:ON" || c == "RGB:OFF") return "RGB";
  if (c.startsWith("MODE:")) return "MODE";
  if (c.startsWith("BRIGHTNESS:")) return "BRIGHTNESS";
  if (c.startsWith("COLOR:")) return "COLOR";
  if (c.startsWith("V5COLOR:")) return "V5COLOR";
  if (c.startsWith("V9COLOR:")) return "V9COLOR";
  if (c.startsWith("V12COLOR:")) return "V12COLOR";
  if (c.startsWith("HOTLIMIT:")) return "HOTLIMIT";
  if (c.startsWith("BATT5:")) return "BATT5";
  if (c.startsWith("BATT12:")) return "BATT12";
  if (c.startsWith("VOLTAGE:")) return "VOLTAGE";
  if (c.startsWith("FAN:")) return "FAN";
  if (c.startsWith("PELTIER:")) return "PELTIER";
  if (c.startsWith("ADAPTIVE:")) return "ADAPTIVE";
  if (c.startsWith("FANSPEED:")) return "FANSPEED";
  if (c == "RESET:TEMPERATURE") return "RESET_TEMPERATURE";
  if (c == "RESET:VOLTAGECOLORS") return "RESET_VOLTAGECOLORS";
  return "COMMAND";
}

static bool userControlRateLimited(const String& command) {
  if (!isUserControlCommand(command)) return false;

  const uint32_t now = millis();
  if (userControlGuardArmed &&
      static_cast<uint32_t>(now - lastUserControlCommandMs) < USER_CONTROL_GUARD_MS) {
    sendAck(controlAckItem(command), false, "RATE_LIMIT");
    return true;
  }

  userControlGuardArmed = true;
  lastUserControlCommandMs = now;
  return false;
}

static void dispatchBleLine(const String& raw) {
  String command = raw;
  command.trim();
  if (command.isEmpty()) return;
  if (command.length() >= BLE_RX_BUFFER_SIZE) return;

  if (!bleCommandGateOpen && command != "SYNC" && command != "SYNC_READY") {
    sendAck("SESSION", false, "SYNC_REQUIRED");
    return;
  }

  if (userControlRateLimited(command)) {
    return;
  }

  handleCommand(command);
}

static void feedBleRx(const String& incoming) {
  for (size_t i = 0; i < incoming.length(); ++i) {
    const char c = incoming[i];

    if (c == '\r') continue;

    if (c == '\n') {
      if (bleRxLength > 0) {
        bleRxBuffer[bleRxLength] = '\0';
        dispatchBleLine(String(bleRxBuffer));
        bleRxLength = 0;
      }
      continue;
    }

    if (bleRxLength >= BLE_RX_BUFFER_SIZE - 1) {

      bleRxLength = 0;
      continue;
    }

    bleRxBuffer[bleRxLength++] = c;
  }
}

class ServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer* server) override {
    bleClientConnected = true;

    bleSyncPending = false;
    bleCommandGateOpen = false;
    userControlGuardArmed = false;
    lastUserControlCommandMs = 0;
    bleQueueResetPending = true;
  }

  void onDisconnect(BLEServer* server) override {
    (void)server;

    bleClientConnected = false;
    bleSyncPending = false;
    bleCommandGateOpen = false;
    bleQueueResetPending = true;

    if (!otaActive && !bootApActive && BLEDevice::getInitialized()) {
      BLEDevice::startAdvertising();
      // The callback never writes LEDs directly. The main loop resumes the
      // normal LED effect on the very next safe iteration.
      bleLedResumePending.store(true, std::memory_order_release);
    }
  }




};

class RxCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic) override {
    enqueueBleRxPayload(characteristic->getValue());
  }
};
static ServerCallbacks serverCallbacks;
static RxCallbacks rxCallbacks;

static void stopBleRadio(bool releaseBluetoothMemory) {
  bleQueueResetPending = true;
  bleClientConnected = false;
  bleSyncPending = false;
  bleCommandGateOpen = false;

  if (BLEDevice::getInitialized()) {
    if (releaseBluetoothMemory) {
      BLEDevice::deinit(true);
    } else {
      BLEDevice::stopAdvertising(); // AMAN: Suspend radio tanpa merusak Heap!
    }
    // Give the BLE controller a short cooperative shutdown window.
    uint32_t start = millis();
    while (millis() - start < 50) { yield(); }
  }

  if (releaseBluetoothMemory) {
    bleServer = nullptr;
    rxCharacteristic = nullptr;
    txCharacteristic = nullptr;
  }
  bleStartPending = true;
  bleStartRetryAt = millis();
}

static void stopBleForBootWiFiClient() {
  // Preserve the established boot-AP/local-OTA BLE teardown behavior.
  stopBleRadio(true);
}

static void waitForWiFiOff() {
  WiFi.mode(WIFI_OFF);
  uint32_t start = millis();
  while (millis() - start < 160) {
    yield(); // Memberi waktu PHY Wi-Fi mati tanpa nge-block loop utama
  }
}

static void scheduleBleRetry() {
  bleStartPending = true;
  if (bleStartAttempts < BLE_START_MAX_ATTEMPTS) ++bleStartAttempts;
  bleStartRetryAt = millis() + BLE_START_RETRY_MS;
}

static void startBle() {
  if (otaActive) return;
  if (bootApActive) return;

  runHeapCheck("CP04_STARTBLE_ENTRY");

  // Wi-Fi shutdown is owned by the caller at the radio-handoff boundary.
  // Do not repeat disconnect/SoftAP disconnect/WIFI_OFF here.
  yield();

  runHeapCheck("CP05_AFTER_WIFI_OFF");

  if (BLEDevice::getInitialized()) {
    if (bleServer != nullptr &&
        rxCharacteristic != nullptr &&
        txCharacteristic != nullptr) {
      BLEDevice::startAdvertising();
      bleStartPending = false;
      bleStartAttempts = 0;
      bleStartRetryAt = 0;
      Serial.println("BLE: READY/ADVERTISING");
      return;
    }

    // Recover from a stale/partial controller state instead of treating
    // getInitialized() as a complete BLE startup.
    BLEDevice::deinit(true);
    yield();
    bleServer = nullptr;
    rxCharacteristic = nullptr;
    txCharacteristic = nullptr;
    runHeapCheck("CP05B_AFTER_CONDITIONAL_DEINIT");
  }

  Serial.println("BLE: START");

  if (!BLEDevice::init(BLE_NAME)) {
    Serial.println("BLE: INIT FAILED");
    scheduleBleRetry();
    return;
  }
  runHeapCheck("CP06_AFTER_BLE_INIT");

  BLEDevice::setMTU(247);

  bleServer = BLEDevice::createServer();
  if (bleServer == nullptr) {
    BLEDevice::deinit(true);
    scheduleBleRetry();
    return;
  }
  bleServer->setCallbacks(&serverCallbacks);
  runHeapCheck("CP07_AFTER_CREATE_SERVER");

  BLEService* service = bleServer->createService(SERVICE_UUID);
  if (service == nullptr) {
    BLEDevice::deinit(true);
    bleServer = nullptr;
    scheduleBleRetry();
    return;
  }
  runHeapCheck("CP08_AFTER_CREATE_SERVICE");

  const uint32_t rxProperties = BLECharacteristic::PROPERTY_WRITE;
  rxCharacteristic = service->createCharacteristic(RX_UUID, rxProperties);
  if (rxCharacteristic == nullptr) {
    BLEDevice::deinit(true);
    bleServer = nullptr;
    scheduleBleRetry();
    return;
  }
  rxCharacteristic->setCallbacks(&rxCallbacks);
  runHeapCheck("CP09_AFTER_CREATE_RX");

  txCharacteristic = service->createCharacteristic(
      TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  if (txCharacteristic == nullptr) {
    BLEDevice::deinit(true);
    bleServer = nullptr;
    rxCharacteristic = nullptr;
    scheduleBleRetry();
    return;
  }
  runHeapCheck("CP10_AFTER_CREATE_TX");
  txCharacteristic->addDescriptor(new BLE2902());
  runHeapCheck("CP11_AFTER_BLE2902");

  service->start();
  runHeapCheck("CP12_AFTER_SERVICE_START");

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  runHeapCheck("CP13_AFTER_GET_ADVERTISING");
  if (advertising == nullptr) {
    BLEDevice::deinit(true);
    bleServer = nullptr;
    rxCharacteristic = nullptr;
    txCharacteristic = nullptr;
    scheduleBleRetry();
    return;
  }

  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  runHeapCheck("CP14_AFTER_START_ADVERTISING");
  scheduleHeapDiagPostAdvertisingChecks();

  bleClientConnected = false;
  bleSyncPending = false;
  clearBleTxQueue();
  clearBleRxQueue();
  bleQueueResetPending = false;
  bleStartPending = false;
  bleStartAttempts = 0;
  bleStartRetryAt = 0;

  Serial.println("BLE: READY/ADVERTISING");

  if (rgbOn) {
    startUserLedEffect();
  } else {
    stopUserLedOutput();
  }
}

static void serviceBleStartRecovery() {
  if (otaActive || bootApActive) return;

  if (BLEDevice::getInitialized() &&
      bleServer != nullptr &&
      rxCharacteristic != nullptr &&
      txCharacteristic != nullptr) {
    return;
  }

  if (millis() < bleStartRetryAt) return;

  waitForWiFiOff();
  startBle();
}

// ============================================================================
// ========================== OTA ENGINE — LOCKED ============================
// Online OTA uses the official Arduino-ESP32 HTTPClient + Update lifecycle.
// Firebase is the App-side release metadata/control plane; GitHub is the binary plane.
// The firmware download is streamed directly into the inactive OTA partition.
// Data is processed in small fixed-size reads with explicit idle-timeout checks;
// no firmware-sized RAM buffer or resumable Range transport is used.
// ============================================================================

// Only the approved JE X FYZ GitHub repository may be used as the OTA binary source.
static bool validGitHubFirmwareUrl(const String& value) {
  String url = value;
  url.trim();
  if (url.length() == 0 || url.length() >= BLE_RX_BUFFER_SIZE) return false;
  if (!url.startsWith("https://raw.githubusercontent.com/TenzoNkz/JEXFYZ-Firmware/")) {
    return false;
  }
  if (!url.endsWith(".bin")) return false;
  return true;
}

static void clearCloudCredentials() {
  cloudSsid = "";
  cloudPassword = "";
  cloudUrl = "";
  cloudVersion = "";
}

static void stopWiFiRadio() {
  WiFi.disconnect(true, false);
  WiFi.softAPdisconnect(true);
  waitForWiFiOff();
}

static void applyOtaSafeOutputs() {
  fanOn = false;
  peltierOn = false;
  adaptiveOn = false;
  adaptiveCeiling = Voltage::V12;
  fanPwm = 100;
  currentVoltage = Voltage::V5;
  pendingVoltage = Voltage::V5;

  // OTA owns the physical outputs, including the user RGB LED.
  stopUserLedOutput();

  stopPeltierOutput();
  if (runtimeFeaturesReady) {
    ledcWrite(PIN_FAN, 0);
  }
  digitalWrite(PIN_9V, OUTPUT_OFF);
  digitalWrite(PIN_12V, OUTPUT_OFF);
}

static String updateErrorDetail() {
  const char* detail = Update.errorString();
  String message = detail == nullptr ? "UPDATE_FAILED" : String(detail);
  if (message.isEmpty()) message = "UPDATE_FAILED";
  return message;
}

static void finishOnlineOtaFailureRecovery() {
  onlineResultIndicatorActive = false;
  onlineResultIndicatorLastPhase = 0xFFFFFFFFUL;
  onlineResultIndicatorNext = OnlineOtaIndicatorNext::NONE;
  onlineOtaFlow = OnlineOtaFlow::IDLE;
  onlineUpdateStarted = false;
  onlineHttpActive = false;
  otaActive = false;
  clearCloudCredentials();
  startBle();
}

static void failOnlineOta(
    const String& stage,
    int errorCode,
    const String& error) {
  otaRestartAt = 0;
  onlineOtaFlow = OnlineOtaFlow::IDLE;

  setOtaDiagnosticStage(stage);
  setOtaDiagnosticError(errorCode, error);
  saveOtaResult(
      otaFailureResult("ONLINE", stage, errorCode, error, otaDiagnosticProgress));

  if (onlineHttpActive) {
    onlineHttpClient.end();
    onlineHttpActive = false;
  }
  cloudClient.stop();

  if (onlineUpdateStarted) {
    Update.onProgress(nullptr);
    Update.abort();
    onlineUpdateStarted = false;
  }

  stopWiFiRadio();
  clearCloudCredentials();
  applyOtaSafeOutputs();

  setOtaState(OtaState::FAILED);
  otaActive = true;
  startOnlineOtaResultIndicator(0xFF0000, OnlineOtaIndicatorNext::RECOVER);
}

static void onlineUpdateProgress(size_t current, size_t total) {
  if (total == 0) return;
  const uint8_t progress = static_cast<uint8_t>(
      min<size_t>(100, (current * 100ULL) / total));
  setOtaDiagnosticProgress(progress);
}

static bool prepareOnlineOtaDownload() {
  setOtaDiagnosticStage("ONLINE_GITHUB_CONNECT");
  setOtaDiagnosticProgress(0);
  setOtaDiagnosticError(0, "NONE");

  cloudClient.stop();
  cloudClient.setInsecure();
  cloudClient.setTimeout(ONLINE_OTA_HTTP_TIMEOUT_MS);

  onlineHttpClient.end();
  if (!onlineHttpClient.begin(cloudClient, cloudUrl)) {
    failOnlineOta("ONLINE_GITHUB_CONNECT", -1, "HTTP_BEGIN_FAILED");
    return false;
  }
  onlineHttpActive = true;
  onlineHttpResponseCode = 0;
  onlineHttpExpectedBytes = 0;

  onlineHttpClient.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  onlineHttpClient.setConnectTimeout(15000);
  onlineHttpClient.setTimeout(ONLINE_OTA_HTTP_TIMEOUT_MS);
  onlineHttpClient.setReuse(false);
  onlineHttpClient.setUserAgent("JE-X-FYZ-OTA");
  onlineHttpClient.setAcceptEncoding("identity");

  Serial.print("OTA ONLINE: GITHUB BIN URL=");
  Serial.println(cloudUrl);

  const int responseCode = onlineHttpClient.GET();
  onlineHttpResponseCode = responseCode;
  if (responseCode <= 0) {
    failOnlineOta(
        "ONLINE_GITHUB_CONNECT",
        responseCode,
        HTTPClient::errorToString(responseCode));
    return false;
  }

  if (responseCode != HTTP_CODE_OK) {
    failOnlineOta(
        "ONLINE_GITHUB_CONNECT",
        responseCode,
        "HTTP_" + String(responseCode));
    return false;
  }

  const int contentLength = onlineHttpClient.getSize();
  onlineHttpExpectedBytes = contentLength;
  if (contentLength <= 0) {
    failOnlineOta(
        "ONLINE_GITHUB_CONNECT",
        -102,
        "CONTENT_LENGTH_MISSING");
    return false;
  }

  if (static_cast<size_t>(contentLength) > OTA_MAX_FIRMWARE_BYTES ||
      static_cast<size_t>(contentLength) > ESP.getFreeSketchSpace()) {
    failOnlineOta(
        "ONLINE_GITHUB_CONNECT",
        OTA_ERROR_SPACE,
        "FIRMWARE_TOO_LARGE");
    return false;
  }

  NetworkClient* stream = onlineHttpClient.getStreamPtr();
  if (stream == nullptr) {
    failOnlineOta(
        "ONLINE_GITHUB_CONNECT",
        -103,
        "HTTP_STREAM_UNAVAILABLE");
    return false;
  }

  // Do not use Stream::peek() as a transport gate here. On some network/client
  // states the stream can legitimately have no byte buffered yet even though
  // the HTTP body is valid. The first actual body byte is validated after a
  // successful read in performOnlineOtaDownload().

  setOtaDiagnosticStage("ONLINE_GITHUB_CONNECTED");
  setOtaDiagnosticProgress(0);
  startOnlineOtaResultIndicator(
      0x00FF00,
      OnlineOtaIndicatorNext::DOWNLOAD);
  return true;
}

static bool performOnlineOtaDownload() {
  setOtaDiagnosticStage("ONLINE_DOWNLOAD");
  setOtaDiagnosticProgress(0);
  setOtaDiagnosticError(0, "NONE");

  if (!onlineHttpActive || onlineHttpExpectedBytes <= 0) {
    failOnlineOta(
        "ONLINE_DOWNLOAD",
        -104,
        "HTTP_SESSION_NOT_READY");
    return false;
  }

  if (!Update.begin(
          static_cast<size_t>(onlineHttpExpectedBytes),
          U_FLASH)) {
    const int code = static_cast<int>(Update.getError());
    failOnlineOta("ONLINE_FLASH_PREPARE", code, updateErrorDetail());
    return false;
  }

  onlineUpdateStarted = true;
  Update.onProgress(onlineUpdateProgress);

  NetworkClient* stream = onlineHttpClient.getStreamPtr();
  if (stream == nullptr) {
    failOnlineOta(
        "ONLINE_DOWNLOAD",
        -103,
        "HTTP_STREAM_UNAVAILABLE");
    return false;
  }

  static constexpr size_t ONLINE_OTA_READ_BUFFER_SIZE = 1024;
  static constexpr uint32_t ONLINE_OTA_STREAM_IDLE_TIMEOUT_MS = 10000;
  uint8_t buffer[ONLINE_OTA_READ_BUFFER_SIZE];
  size_t totalWritten = 0;
  uint32_t lastDataAt = millis();

  while (totalWritten < static_cast<size_t>(onlineHttpExpectedBytes)) {
    yield();

    const size_t availableBytes = static_cast<size_t>(stream->available());
    if (availableBytes == 0) {
      if (!onlineHttpClient.connected()) {
        failOnlineOta(
            "ONLINE_DOWNLOAD",
            OTA_ERROR_STREAM,
            "STREAM_CLOSED_PREMATURELY");
        return false;
      }

      if (millis() - lastDataAt >= ONLINE_OTA_STREAM_IDLE_TIMEOUT_MS) {
        failOnlineOta(
            "ONLINE_DOWNLOAD",
            OTA_ERROR_STREAM,
            "STREAM_TIMEOUT");
        return false;
      }

      yield();
      continue;
    }

    const size_t remaining =
        static_cast<size_t>(onlineHttpExpectedBytes) - totalWritten;
    const size_t toRead = min(
        min(availableBytes, ONLINE_OTA_READ_BUFFER_SIZE),
        remaining);

    const int bytesRead = stream->read(buffer, toRead);
    if (bytesRead <= 0) {
      if (!onlineHttpClient.connected()) {
        failOnlineOta("ONLINE_DOWNLOAD", OTA_ERROR_STREAM, "STREAM_READ_ERROR");
        return false;
      }
      if (millis() - lastDataAt >= ONLINE_OTA_STREAM_IDLE_TIMEOUT_MS) {
        failOnlineOta("ONLINE_DOWNLOAD", OTA_ERROR_STREAM, "STREAM_READ_TIMEOUT");
        return false;
      }
      // Cooperative retry: no blocking Arduino delay.
      yield();
      continue;
    }

    if (totalWritten == 0 && static_cast<uint8_t>(buffer[0]) != 0xE9) {
      failOnlineOta(
          "ONLINE_IMAGE_VALIDATE",
          OTA_ERROR_MAGIC_BYTE,
          "WRONG_FIRMWARE_MAGIC");
      return false;
    }

    lastDataAt = millis();

    const size_t written =
        Update.write(buffer, static_cast<size_t>(bytesRead));
    if (written != static_cast<size_t>(bytesRead)) {
      const int code = static_cast<int>(Update.getError());
      failOnlineOta(
          "ONLINE_DOWNLOAD",
          code != 0 ? code : OTA_ERROR_WRITE,
          code != 0 ? updateErrorDetail() : "UPDATE_WRITE_ERROR");
      return false;
    }

    totalWritten += written;
    onlineUpdateProgress(totalWritten, onlineHttpExpectedBytes);
    yield();
  }

  onlineHttpClient.end();
  onlineHttpActive = false;
  cloudClient.stop();

  setOtaDiagnosticProgress(100);
  setOtaDiagnosticStage("ONLINE_DOWNLOAD_COMPLETE");
  setOtaDiagnosticError(0, "NONE");

  startOnlineOtaResultIndicator(
      0x0000FF,
      OnlineOtaIndicatorNext::FLASH_FINALIZE);
  return true;
}

static bool finalizeOnlineOtaFlash() {
  setOtaDiagnosticStage("ONLINE_FLASH_FINALIZE");
  setOtaDiagnosticError(0, "NONE");

  if (!onlineUpdateStarted) {
    failOnlineOta(
        "ONLINE_FLASH_FINALIZE",
        -105,
        "UPDATE_NOT_STARTED");
    return false;
  }

  const bool finalized = Update.end();
  if (!finalized) {
    const int code = static_cast<int>(Update.getError());
    failOnlineOta(
        "ONLINE_FLASH_FINALIZE",
        code,
        updateErrorDetail());
    return false;
  }

  onlineUpdateStarted = false;
  Update.onProgress(nullptr);
  stopWiFiRadio();

  setOtaDiagnosticStage("ONLINE_FLASH_SUCCESS");
  setOtaDiagnosticProgress(100);
  setOtaDiagnosticError(0, "NONE");
  saveOtaResult(
      otaSuccessResult(
          "ONLINE",
          "ONLINE_FLASH_SUCCESS",
          cloudVersion.isEmpty() ? "UNKNOWN" : cloudVersion));

  clearCloudCredentials();

  // Final GREEN indicator executes only after Update.end() has succeeded.
  startOnlineOtaResultIndicator(
      0x00FF00,
      OnlineOtaIndicatorNext::RESTART);
  return true;
}

static void startOnlineOta() {
  if (otaActive) return;
  if (cloudSsid.isEmpty() || cloudPassword.isEmpty() || !validGitHubFirmwareUrl(cloudUrl)) return;

  otaActive = true;
  setOtaState(OtaState::CLOUD_CONNECTING);
  setOtaDiagnosticStage("ONLINE_REQUEST");
  setOtaDiagnosticProgress(0);
  setOtaDiagnosticError(0, "NONE");

  if (!validFirmwareVersion(cloudVersion)) {
    failOnlineOta("ONLINE_VERSION_CHECK", -1, "INVALID_VERSION");
    return;
  }

  setOtaDiagnosticStage("ONLINE_VERSION_CHECK");

  if (cloudVersion.equalsIgnoreCase(FW_VERSION)) {
    setOtaDiagnosticStage("ONLINE_UP_TO_DATE");
    setOtaDiagnosticProgress(100);
    saveOtaResult(
        String("OTA_UP_TO_DATE|MODE=ONLINE|STAGE=ONLINE_UP_TO_DATE|VERSION=") +
        FW_VERSION + "|PROGRESS=100");
    sendDelta("LAST_OTA_RESULT", otaResult);
    setOtaState(OtaState::IDLE);
    otaActive = false;
    onlineOtaStartPending = false;
    clearCloudCredentials();
    return;
  }

  setOtaDiagnosticStage("ONLINE_WIFI_CONNECT");
  otaRestartAt = 0;
  cloudPhaseStartedAt = millis();
  onlineOtaFlow = OnlineOtaFlow::WIFI_CONNECTING;
  onlineWifiConnectAttempt = 1;
  onlineResultIndicatorActive = false;
  onlineResultIndicatorNext = OnlineOtaIndicatorNext::NONE;
  onlineHttpActive = false;
  onlineUpdateStarted = false;

  saveOtaResult("IN_PROGRESS|MODE=ONLINE|STAGE=ONLINE_WIFI_CONNECT|PROGRESS=0");
  applyOtaSafeOutputs();
  // Online OTA must begin with the LED completely OFF while Wi-Fi connects.
  stopUserLedOutput();

  clearBleTxQueue();
  clearBleRxQueue();
  
  stopBleRadio(false); // AMAN: Tidak akan crash
  bleStartPending = false;
  bleStartAttempts = 0;

  WiFi.disconnect(true, false);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  
  // Langsung serahkan ke State Machine tanpa nge-block
  onlineOtaFlow = OnlineOtaFlow::RADIO_TEARDOWN;
  cloudPhaseStartedAt = millis();
}

static void processOnlineOta() {
  if (onlineResultIndicatorActive) {
    processOnlineOtaResultIndicator();
    return;
  }

  switch (onlineOtaFlow) {
    case OnlineOtaFlow::RADIO_TEARDOWN:
      // Menunggu 160ms secara non-blocking agar radio stack benar-benar bersih
      if (millis() - cloudPhaseStartedAt >= 160) {
        WiFi.mode(WIFI_STA);
        WiFi.setHostname("JE-X-FYZ");
        WiFi.setSleep(false);
        
        // KRUSIAL: Matikan AutoReconnect agar tidak stuck saat password salah!
        WiFi.setAutoReconnect(false); 
        
        WiFi.begin(cloudSsid.c_str(), cloudPassword.c_str());
        cloudPhaseStartedAt = millis();
        onlineOtaFlow = OnlineOtaFlow::WIFI_CONNECTING;
      }
      return;

    case OnlineOtaFlow::WIFI_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        setOtaDiagnosticStage("ONLINE_WIFI_CONNECTED");
        setOtaDiagnosticProgress(0);
        startOnlineOtaResultIndicator(
            0x0000FF,
            OnlineOtaIndicatorNext::PREPARE_DOWNLOAD); // LED BIRU -> langsung ke DOWNLOAD
        return;
      }

      // FAST FAIL: Tangani password salah tanpa harus menunggu 15 detik penuh
      if (otaActive && (millis() - cloudPhaseStartedAt >= WIFI_CONNECT_TIMEOUT_MS || 
                        WiFi.status() == WL_CONNECT_FAILED || 
                        WiFi.status() == WL_NO_SSID_AVAIL)) {
                        
        if (onlineWifiConnectAttempt < WIFI_CONNECT_MAX_ATTEMPTS) {
          ++onlineWifiConnectAttempt;
          setOtaDiagnosticStage("ONLINE_WIFI_RETRY_" + String(onlineWifiConnectAttempt));
          setOtaDiagnosticProgress(0);
          
          WiFi.disconnect(true, false);
          WiFi.mode(WIFI_OFF);
          
          // Kembali ke fase Teardown untuk Retry yang bersih
          onlineOtaFlow = OnlineOtaFlow::RADIO_TEARDOWN;
          cloudPhaseStartedAt = millis();
        } else {
          failOnlineOta("ONLINE_WIFI_FAILED", -1, "WIFI_FAILED_AFTER_RETRIES");
        }
      }
      return;

    case OnlineOtaFlow::DOWNLOAD:
      if (!onlineUpdateStarted) {
        performOnlineOtaDownload();
      }
      return;

    case OnlineOtaFlow::FLASH_FINALIZE:
      finalizeOnlineOtaFlash();
      return;

    case OnlineOtaFlow::IDLE:
      return;
  }
}

static const char LOCAL_OTA_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>JE X FYZ OTA</title>
<style>
body{margin:0;background:#0f1014;color:#fff;font-family:Arial,sans-serif}
main{max-width:560px;margin:28px auto;padding:22px}
.card{background:#1a1c23;border-radius:20px;padding:22px;box-shadow:0 8px 28px #0008}
h1{margin:0 0 6px;font-size:25px}p{color:#a8acb8}
input{width:100%;box-sizing:border-box;margin:14px 0;padding:14px;border-radius:12px;border:1px solid #3d414d;background:#101116;color:#fff}
button{width:100%;padding:14px;border:0;border-radius:12px;background:#4e8cff;color:#fff;font-weight:800}
progress{width:100%;height:18px;margin-top:16px}
#status{margin-top:12px;font-size:13px;color:#a8acb8}
.small{font-size:12px;color:#8b909d;margin-top:14px}
</style>
</head>
<body>
<main><div class="card">
<h1>JE X FYZ</h1>
<p>Firmware OTA</p>
<p>Current firmware: <b>%VERSION%</b></p>
<form id="otaForm">
<input id="firmware" type="file" accept=".bin" required>
<button id="uploadBtn" type="submit">Upload &amp; Update</button>
</form>
<progress id="progress" value="0" max="100"></progress>
<div id="status">Ready.</div>
<div class="small">Successful OTA: green blink, then restart. Failed OTA: red blink; Wi-Fi remains active for retry.</div>
</div></main>
<script>
const form=document.getElementById('otaForm');
const fileInput=document.getElementById('firmware');
const btn=document.getElementById('uploadBtn');
const progress=document.getElementById('progress');
const status=document.getElementById('status');
form.addEventListener('submit',e=>{
  e.preventDefault();
  if(!fileInput.files.length)return;
  const xhr=new XMLHttpRequest();
  const file=fileInput.files[0];
   xhr.open('POST','/update?size='+encodeURIComponent(file.size.toString()));
  xhr.upload.onprogress=e=>{
    if(e.lengthComputable)progress.value=(e.loaded/e.total)*100;
    status.textContent='Uploading... '+progress.value.toFixed(0)+'%';
  };
  xhr.onload=()=>{
    if(xhr.status===200){progress.value=100;status.textContent='Update verified. Green blink, then device will restart.';btn.disabled=true;}
    else { status.textContent='OTA failed: '+(xhr.responseText||'Unknown OTA error'); btn.disabled=false; }
  };
  xhr.onerror=()=>status.textContent='Connection error.';
  btn.disabled=true;
  const data=new FormData();
  data.append('firmware',fileInput.files[0]);
  xhr.send(data);
});
</script>
</body>
</html>
)HTML";

static void abortLocalOtaTransaction() {
  if (localOtaStarted) {
    Update.abort();
  }
  localOtaStarted = false;
}

static void failLocalOta(
    const String& stage,
    int errorCode,
    const String& error) {
  abortLocalOtaTransaction();
  setOtaDiagnosticStage(stage);
  setOtaDiagnosticError(errorCode, error);
  saveOtaResult(
      otaFailureResult("LOCAL", stage, errorCode, error, otaDiagnosticProgress));
  setOtaState(OtaState::FAILED);
  localOtaResultIndicatorActive = false;
  localOtaResultIndicatorLastPhase = 0xFFFFFFFFUL;
  resetBlinkState();
  showAll(0x000000, 0);
  otaFailureIndicatorUntil = millis() + OTA_FAILURE_INDICATOR_MS;
  otaActive = false;
  localOtaBytesReceived = 0;
  localOtaExpectedBytes = 0;
  applyOtaSafeOutputs();

  // Keep Local OTA AP active after a failure so another upload can retry.
  // The FAILED state is rendered as a non-blocking red blink.
  bootApActive = true;
  bootApHadClient = true;
  bootApClientSafe = true;
}

static void localOtaUploadHandler() {
  HTTPUpload& upload = webServer.upload();

  if (upload.status == UPLOAD_FILE_START) {
    otaRestartAt = 0;
    otaActive = true;
    setOtaState(OtaState::LOCAL_UPLOAD);
    setOtaDiagnosticStage("LOCAL_FILE_CHECK");
    setOtaDiagnosticProgress(0);
    setOtaDiagnosticError(0, "NONE");
    resetBlinkState();
    localOtaResultIndicatorActive = false;
    localOtaResultIndicatorLastPhase = 0xFFFFFFFFUL;
    showAll(0x000000, 0);
    abortLocalOtaTransaction();
    localOtaBytesReceived = 0;
    localOtaExpectedBytes = 0;

    String filename = upload.filename;
    filename.toLowerCase();
    if (!filename.endsWith(".bin")) {
      failLocalOta("LOCAL_FILE_CHECK", -1, "INVALID_FILE_TYPE");
      return;
    }

    if (!webServer.hasArg("size")) {
      failLocalOta("LOCAL_SIZE_CHECK", -1, "SIZE_REQUIRED");
      return;
    }

    const long parsedSize = webServer.arg("size").toInt();
    if (parsedSize <= 0 ||
        parsedSize > static_cast<long>(OTA_MAX_FIRMWARE_BYTES)) {
      failLocalOta("LOCAL_SIZE_CHECK", -1, "INVALID_FILE_SIZE");
      return;
    }

    localOtaExpectedBytes = static_cast<size_t>(parsedSize);

    const size_t freeSketchSpace =
        static_cast<size_t>(ESP.getFreeSketchSpace());
    if (localOtaExpectedBytes > freeSketchSpace) {
      failLocalOta(
          "LOCAL_SIZE_CHECK",
          OTA_ERROR_SPACE,
          "FIRMWARE_EXCEEDS_FREE_SKETCH_SPACE_" +
              String(static_cast<unsigned long>(freeSketchSpace)));
      return;
    }

    applyOtaSafeOutputs();
    setOtaDiagnosticStage("LOCAL_UPDATE_BEGIN");

    if (!Update.begin(localOtaExpectedBytes)) {
      const int updateErrorCode = static_cast<int>(Update.getError());
      const String updateError = Update.errorString();
      Update.printError(Serial);
      failLocalOta(
          "LOCAL_UPDATE_BEGIN",
          updateErrorCode,
          "UPDATE_BEGIN_FAILED_" + updateError);
      return;
    }

    localOtaStarted = true;
    setOtaDiagnosticStage("LOCAL_RECEIVE");
    saveOtaResult("IN_PROGRESS|MODE=LOCAL|STAGE=LOCAL_RECEIVE|PROGRESS=0");
    return;
  }

  if (upload.status == UPLOAD_FILE_WRITE) {
    if (otaState != OtaState::LOCAL_UPLOAD ||
        !otaActive ||
        !localOtaStarted) {
      return;
    }

    if (upload.currentSize == 0) return;
    setOtaDiagnosticStage("LOCAL_FLASH_WRITE");

    const size_t chunkSize = static_cast<size_t>(upload.currentSize);
    if (localOtaBytesReceived + chunkSize > localOtaExpectedBytes) {
      failLocalOta("LOCAL_SIZE_VERIFY", OTA_ERROR_SIZE, "UPLOAD_SIZE_EXCEEDED");
      return;
    }

    const size_t written = Update.write(upload.buf, chunkSize);
    if (written != chunkSize) {
      Update.printError(Serial);
      failLocalOta(
          "LOCAL_FLASH_WRITE",
          OTA_ERROR_WRITE,
          "UPDATE_WRITE_FAILED");
      return;
    }

    localOtaBytesReceived += chunkSize;
    const uint8_t progress = static_cast<uint8_t>(
        min<size_t>(
            100,
            (localOtaBytesReceived * 100ULL) /
                max<size_t>(1, localOtaExpectedBytes)));
    setOtaDiagnosticProgress(progress);
    return;
  }

  if (upload.status == UPLOAD_FILE_END) {
    if (otaState != OtaState::LOCAL_UPLOAD ||
        !otaActive ||
        !localOtaStarted) {
      return;
    }

    setOtaDiagnosticStage("LOCAL_SIZE_VERIFY");
    if (localOtaBytesReceived == 0 ||
        localOtaBytesReceived != localOtaExpectedBytes) {
      char detail[96];
      snprintf(
          detail,
          sizeof(detail),
          "UPLOAD_SIZE_MISMATCH_%lu_%lu",
          static_cast<unsigned long>(localOtaBytesReceived),
          static_cast<unsigned long>(localOtaExpectedBytes));
      failLocalOta("LOCAL_SIZE_VERIFY", OTA_ERROR_SIZE, detail);
      return;
    }

    setOtaDiagnosticProgress(100);
    setOtaDiagnosticStage("LOCAL_IMAGE_VERIFY");

    localOtaStarted = false;
    if (!Update.end(true)) {
      const int updateErrorCode = static_cast<int>(Update.getError());
      const String updateError = Update.errorString();
      Update.printError(Serial);
      failLocalOta(
          "LOCAL_IMAGE_VERIFY",
          updateErrorCode,
          "UPDATE_END_FAILED_" + updateError);
      return;
    }

    setOtaDiagnosticStage("LOCAL_BOOT_PARTITION");
    setOtaDiagnosticError(0, "NONE");
    saveOtaResult(
        otaSuccessResult("LOCAL", "LOCAL_BOOT_PARTITION", FW_VERSION));
    setOtaState(OtaState::SUCCESS);
    otaActive = true;
    otaRestartAt = 0;
    localOtaExpectedBytes = 0;

    // Local OTA success has its own visible green blink before restart.
    // The blink is non-blocking and remains under OTA priority supervision.
    startLocalOtaResultIndicator();
    return;
  }

  if (upload.status == UPLOAD_FILE_ABORTED) {
    failLocalOta("LOCAL_RECEIVE", OTA_ERROR_STREAM, "UPLOAD_ABORTED");
  }
}

static void handleOtaRoot() {
  String html = FPSTR(LOCAL_OTA_PAGE);
  html.replace("%VERSION%", FW_VERSION);
  webServer.send(200, "text/html", html);
}

static void handleOtaPost() {
  webServer.sendHeader("Connection", "close");

  if (otaState == OtaState::FAILED) {
    webServer.send(
        500,
        "text/plain",
        "OTA FAILED: " + otaResult);
    return;
  }

  if (otaState == OtaState::SUCCESS) {
    webServer.send(
        200,
        "text/plain",
        "OTA verified. Green indicator active; device will restart.");
    return;
  }

  webServer.send(
      409,
      "text/plain",
      "OTA not finalized: " + otaResult);
}

static void startLocalOtaAp() {
  onlineOtaStartPending = false;
  // Boot AP is not a safety state until a Wi-Fi client actually connects.
  // Keep the restored user LED state visible while the AP waits for a client.
  bootApActive = true;
  bootApHadClient = false;
  bootApClientSafe = false;
  otaActive = false;
  setOtaState(OtaState::BOOT_AP);

  Serial.println("WIFI AP: START");
  Serial.println("WIFI AP: STEP 1 SIMPLE SOFTAP");

  const bool apOk = WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.print("WIFI AP: SOFTAP RESULT=");
  Serial.println(apOk ? "OK" : "FAIL");
  if (!apOk) {
    Serial.println("WIFI AP: FAIL AT SOFTAP");
    bootApActive = false;
    setOtaState(OtaState::FAILED);
    otaFailureIndicatorUntil = millis() + OTA_FAILURE_INDICATOR_MS;
    WiFi.mode(WIFI_OFF);
    uint32_t start = millis();
    while (millis() - start < 160) {
      yield();
    }
    bleStartPending = true;
    bleStartRetryAt = millis();
    return;
  }

  Serial.println("WIFI AP: STEP 2 HTTP");
  bootApStartedAt = millis();
  Serial.print("WIFI AP: READY SSID=");
  Serial.print(AP_SSID);
  Serial.print(" IP=");
  Serial.println(WiFi.softAPIP());

  webServer.on("/", HTTP_GET, handleOtaRoot);
  webServer.on("/update", HTTP_POST, handleOtaPost, localOtaUploadHandler);
  webServer.begin();
  Serial.println("WIFI AP: HTTP READY");
}

static void stopLocalOtaAp() {
  runHeapCheck("CP01_BEFORE_WEBSERVER_STOP");
  webServer.stop();
  runHeapCheck("CP02_AFTER_WEBSERVER_STOP");

  stopWiFiRadio();
  runHeapCheck("CP03_AFTER_STOP_WIFI_RADIO");
  bootApActive = false;
  if (otaState == OtaState::BOOT_AP) {
    setOtaState(OtaState::IDLE);
  }
}

// ======================== END OTA ENGINE — LOCKED ==========================
static bool setFanState(bool desired, String& reason) {
  if (adaptiveOn) {
    reason = "ADAPTIVE_ACTIVE";
    return false;
  }

  if (!ntcValid) {
    reason = "NTC_ERROR";
    return false;
  }

  if (!desired && peltierOn) {
    reason = "PELTIER_ON";
    return false;
  }

  if (!desired && hotsideProtectionActiveNow()) {
    reason = "HOTSIDE_PROTECTION";
    return false;
  }

  if (!desired) {
    adaptiveResumePeltierAfterHotStep = false;
  }

  if (fanOn != desired) {
    fanOn = desired;
    applyCoolingOutputs();
  }

  reason = "OK";
  return fanOn == desired;
}

static bool setPeltierState(bool desired, String& reason) {
  if (adaptiveOn) {
    reason = "ADAPTIVE_ACTIVE";
    return false;
  }

  if (!desired) {
    adaptiveResumePeltierAfterHotStep = false;
    if (peltierOn) {
      peltierOn = false;
      applyCoolingOutputs();
    }
    reason = "OK";
    return true;
  }

  if (!ntcValid) {
    reason = "NTC_ERROR";
    return false;
  }

  if (!fanOn) {
    reason = "FAN_REQUIRED";
    return false;
  }

  if (hotsideProtectionActiveNow()) {
    reason = "HOTSIDE_PROTECTION";
    return false;
  }

  if (!peltierOn) {

    peltierOn = true;
    applyCoolingOutputs();
  }

  reason = peltierOn ? "OK" : safetyReason();
  return peltierOn;
}

static bool forceAdaptiveFiveVolt() {

  if (voltageTransition) {
    pendingVoltage = Voltage::V5;
    return true;
  }

  if (currentVoltage == Voltage::V5) return true;

  return requestVoltage(
      Voltage::V5,
      VoltageRequestSource::ADAPTIVE,
      true);
}

static void setAdaptiveState(bool desired, String& reason) {
  const bool previousAdaptive = adaptiveOn;
  const Voltage previousCeiling = adaptiveCeiling;
  if (desired) {
    if (!ntcValid) {
      adaptiveOn = false;
      adaptiveCeiling = Voltage::V12;
      reason = "NTC_ERROR";
      return;
    }

    adaptiveOn = true;
    if (!previousAdaptive) {
      adaptiveCeiling = Voltage::V12;
      adaptiveHotStepPending = false;
      peltierHotOffUntilMs = 0;
      adaptiveResumePeltierAfterHotStep = false;
      adaptiveUpPending = false;
      adaptiveDownPending = false;
      lastBatteryZone = -1;
      fanPwm = 100;
    }

    if (!forceAdaptiveFiveVolt()) {
      adaptiveOn = previousAdaptive;
      reason = "VOLTAGE_FORCE_FAILED";
      return;
    }

    applyCoolingOutputs();
    reason = "OK";

    if (currentVoltage == Voltage::V5 && batteryValid) {
      evaluateAdaptiveBattery();
    }
  } else {
    adaptiveOn = false;
    adaptiveHotStepPending = false;
    peltierHotOffUntilMs = 0;
    adaptiveUpPending = false;
    adaptiveDownPending = false;
    adaptiveResumePeltierAfterHotStep = false;
    adaptiveCeiling = Voltage::V12;
    fanPwm = 100;

    if (!forceAdaptiveFiveVolt()) {

      reason = "VOLTAGE_FORCE_FAILED";
    } else {
      reason = "OK";
    }

    applyCoolingOutputs();
  }

  if (adaptiveOn != previousAdaptive) {
    ++stateSequence;
    sendDelta("ADAPTIVE", boolText(adaptiveOn));
  }
  if (adaptiveCeiling != previousCeiling) {
    sendDelta("ADAPTIVE_CEILING", voltageToString(adaptiveCeiling));
  }
}

static bool setRgbState(bool desired) {
  if (rgbOn == desired) return true;

  rgbOn = desired;
  ++stateSequence;
  markSettingsDirty();

  if (rgbOn) {
    startUserLedEffect();
  } else {
    stopUserLedOutput();
  }

  sendDelta("RGB", boolText(rgbOn));
  return true;
}

static bool setFanSpeed(uint8_t value, String& reason) {
  if (value < FAN_PWM_MIN || value > FAN_PWM_MAX || value % FAN_PWM_STEP != 0) {
    reason = "RANGE_50_100_STEP_10";
    return false;
  }

  if (adaptiveOn) {
    reason = "ADAPTIVE_ACTIVE";
    return false;
  }

  if (currentVoltage != Voltage::V12) {
    reason = "MANUAL_12V_ONLY";
    return false;
  }

  if (fanPwm != value) {
    fanPwm = value;
    writeFanPwm();
    sendDelta("FAN_SPEED", String(fanPwm));
  }

  reason = "OK";
  return true;
}

static void handleSettingsReset(const String& group) {
  String normalized = group;
  normalized.trim();
  normalized.toUpperCase();

  if (normalized == "TEMPERATURE") {
    if (adaptiveOn) {
      sendAck("RESET_TEMPERATURE", false, "ADAPTIVE_ACTIVE");
      return;
    }
    const bool changed = hotLimit != 45 || battery5Limit != 25 || battery12Limit != 35;
    hotLimit = 45;
    battery5Limit = 25;
    battery12Limit = 35;
    if (changed) {
      markSettingsDirty();
      enforceSafety();
      sendDelta("TEMP_HOT_LIMIT", String(hotLimit));
      sendDelta("BATT_5", String(battery5Limit));
      sendDelta("BATT_12", String(battery12Limit));
    }
    sendAck("RESET_TEMPERATURE", true, "OK");
    return;
  }

  if (normalized == "VOLTAGECOLORS") {
    const bool changed = color5V != 0xFF0000 || color9V != 0x00FF00 || color12V != 0x0000FF;
    color5V = 0xFF0000;
    color9V = 0x00FF00;
    color12V = 0x0000FF;
    if (changed) {
      markSettingsDirty();
      sendDelta("COLOR_5V", colorToHex(color5V));
      sendDelta("COLOR_9V", colorToHex(color9V));
      sendDelta("COLOR_12V", colorToHex(color12V));
    }
    sendAck("RESET_VOLTAGECOLORS", true, "OK");
    return;
  }

  sendAck("RESET", false, "UNKNOWN_GROUP");
}

static void handleCommand(const String& command) {
  if (command == "SYNC") {
    sendFullSync();
    return;
  }

  if (command == "SYNC_READY") {
    bleCommandGateOpen = true;

    sendFullSync();
    sendAck("SYNC_READY", true, "OK");
    return;
  }

  if (command == "OTAENTER") {
    sendAck("OTAENTER", true, "READY");
    return;
  }

  if (command.startsWith("SSID:")) {
    cloudSsid = command.substring(5);
    cloudSsid.trim();
    sendAck("SSID", !cloudSsid.isEmpty(), cloudSsid.isEmpty() ? "INVALID" : "OK");
    return;
  }

  if (command.startsWith("PASS:")) {
    cloudPassword = command.substring(5);
    sendAck("PASS", !cloudPassword.isEmpty(), cloudPassword.isEmpty() ? "INVALID" : "OK");
    return;
  }

  if (command.startsWith("URL:")) {
    cloudUrl = command.substring(4);
    cloudUrl.trim();
    const bool ok = validGitHubFirmwareUrl(cloudUrl);
    sendAck("URL", ok, ok ? "OK" : "HTTPS_REQUIRED");
    return;
  }

  if (command.startsWith("VERSION:")) {
    cloudVersion = command.substring(8);
    cloudVersion.trim();
    sendAck("VERSION", !cloudVersion.isEmpty(), cloudVersion.isEmpty() ? "INVALID" : "OK");
    return;
  }

  if (command == "CLOUDOTA") {
    if (cloudSsid.isEmpty() || cloudPassword.isEmpty() || !validGitHubFirmwareUrl(cloudUrl) || cloudVersion.isEmpty()) {
      sendAck("CLOUDOTA", false, "MISSING_METADATA");
      return;
    }

    if (onlineOtaStartPending) {
      sendAck("CLOUDOTA", false, "ALREADY_PENDING");
      return;
    }

    sendAck("CLOUDOTA", true, "STARTING");
    onlineOtaStartPending = true;
    onlineOtaRequestedAt = millis();

    // Online OTA owns the LED immediately when the request is accepted.
    // This closes the BLE-drain handoff window where the normal user LED
    // effect could otherwise remain visible (often as a static blue frame).
    applyOtaSafeOutputs();
    stopUserLedOutput();
    return;
  }

  if (command == "RGB:ON") {
    const bool ok = setRgbState(true);
    sendAck("RGB", ok, ok ? "OK" : "FAILED");
    return;
  }

  if (command == "RGB:OFF") {
    const bool ok = setRgbState(false);
    sendAck("RGB", ok, ok ? "OK" : "FAILED");
    return;
  }

  if (command.startsWith("MODE:")) {
    int value = 0;
    if (!parseIntRange(command.substring(5), 1, 7, value)) {
      sendAck("MODE", false, "INVALID");
      return;
    }

    const LedMode newMode = static_cast<LedMode>(value);
    if (rgbMode != newMode) {
      rgbMode = newMode;
      markSettingsDirty();
      if (rgbOn) startUserLedEffect();
      sendDelta("MODE", String(value));
    }

    sendAck("MODE", true, "OK");
    return;
  }

  if (command.startsWith("BRIGHTNESS:")) {
    int value = 0;
    if (!parseIntRange(command.substring(11), 0, 100, value)) {
      sendAck("BRIGHTNESS", false, "RANGE_0_100");
      return;
    }

    if (rgbBrightness != static_cast<uint8_t>(value)) {
      rgbBrightness = static_cast<uint8_t>(value);
      markSettingsDirty();
      if (rgbOn) startUserLedEffect();
      sendDelta("BRIGHTNESS", String(value));
    }

    sendAck("BRIGHTNESS", true, "OK");
    return;
  }

  if (command.startsWith("COLOR:")) {
    uint32_t parsed = 0;
    if (!parseHexColor(command.substring(6), parsed)) {
      sendAck("COLOR", false, "INVALID_HEX");
      return;
    }

    if (rgbColor != parsed) {
      rgbColor = parsed;
      markSettingsDirty();
      if (rgbOn) startUserLedEffect();
      sendDelta("RGB_COLOR", colorToHex(rgbColor));
    }

    sendAck("COLOR", true, "OK");
    return;
  }

  if (command.startsWith("V5COLOR:")) {
    uint32_t parsed = 0;
    if (!parseHexColor(command.substring(8), parsed)) {
      sendAck("V5COLOR", false, "INVALID_HEX");
      return;
    }

    if (color5V != parsed) {
      color5V = parsed;
      markSettingsDirty();
      sendDelta("COLOR_5V", colorToHex(color5V));
    }

    sendAck("V5COLOR", true, "OK");
    return;
  }

  if (command.startsWith("V9COLOR:")) {
    uint32_t parsed = 0;
    if (!parseHexColor(command.substring(8), parsed)) {
      sendAck("V9COLOR", false, "INVALID_HEX");
      return;
    }

    if (color9V != parsed) {
      color9V = parsed;
      markSettingsDirty();
      sendDelta("COLOR_9V", colorToHex(color9V));
    }

    sendAck("V9COLOR", true, "OK");
    return;
  }

  if (command.startsWith("V12COLOR:")) {
    uint32_t parsed = 0;
    if (!parseHexColor(command.substring(9), parsed)) {
      sendAck("V12COLOR", false, "INVALID_HEX");
      return;
    }

    if (color12V != parsed) {
      color12V = parsed;
      markSettingsDirty();
      sendDelta("COLOR_12V", colorToHex(color12V));
    }

    sendAck("V12COLOR", true, "OK");
    return;
  }

  if (command.startsWith("HOTLIMIT:")) {
    if (adaptiveOn) {
      sendAck("HOTLIMIT", false, "ADAPTIVE_ACTIVE");
      return;
    }
    int value = 0;
    if (!parseIntRange(command.substring(9), HOT_LIMIT_MIN, HOT_LIMIT_MAX, value)) {
      sendAck("HOTLIMIT", false, "RANGE_40_50");
      return;
    }

    if (hotLimit != value) {
      hotLimit = value;
      markSettingsDirty();
      enforceSafety();
      sendDelta("TEMP_HOT_LIMIT", String(hotLimit));
    }

    sendAck("HOTLIMIT", true, "OK");
    return;
  }

  if (command.startsWith("BATT5:")) {
    if (adaptiveOn) {
      sendAck("BATT5", false, "ADAPTIVE_ACTIVE");
      return;
    }
    int value = 0;
    if (!parseIntRange(command.substring(6), BATTERY_LIMIT_MIN, BATTERY5_LIMIT_MAX, value)) {
      sendAck("BATT5", false, "RANGE_20_48");
      return;
    }

    if (value >= battery12Limit - 1) {      sendAck("BATT5", false, "MUST_BE_BELOW_BATT12");
      return;
    }

    if (battery5Limit != value) {
      battery5Limit = value;
      markSettingsDirty();
      sendDelta("BATT_5", String(battery5Limit));
    }

    sendAck("BATT5", true, "OK");
    return;
  }

  if (command.startsWith("BATT12:")) {
    if (adaptiveOn) {
      sendAck("BATT12", false, "ADAPTIVE_ACTIVE");
      return;
    }
    int value = 0;
    if (!parseIntRange(command.substring(7), BATTERY12_LIMIT_MIN, BATTERY12_LIMIT_MAX, value)) {
      sendAck("BATT12", false, "RANGE_22_50");
      return;
    }

    if (value <= battery5Limit + 1) {
      sendAck("BATT12", false, "MUST_BE_ABOVE_BATT5");
      return;
    }

    if (battery12Limit != value) {
      battery12Limit = value;
      markSettingsDirty();
      sendDelta("BATT_12", String(battery12Limit));
    }

    sendAck("BATT12", true, "OK");
    return;
  }

  if (command.startsWith("VOLTAGE:")) {
    if (adaptiveOn) {
      sendAck("VOLTAGE", false, "ADAPTIVE_ACTIVE");
      return;
    }

    String rawTarget = command.substring(8);
    rawTarget.trim();
    Voltage target = voltageFromString(rawTarget);
    const bool requestedValid = rawTarget.equalsIgnoreCase("5V") ||
                                rawTarget.equalsIgnoreCase("9V") ||
                                rawTarget.equalsIgnoreCase("12V");
    if (!requestedValid) {
      sendAck("VOLTAGE", false, "INVALID");
      return;
    }

    const bool accepted = requestVoltage(target, VoltageRequestSource::MANUAL, false);
    sendAck("VOLTAGE", accepted,
            accepted ? "ACCEPTED" : "VOLTAGE_GUARD_OR_BUSY");
    return;
  }

  if (command == "FAN:ON") {
    String reason;
    const bool ok = setFanState(true, reason);
    sendAck("FAN", ok, reason);
    return;
  }

  if (command == "FAN:OFF") {
    String reason;
    const bool ok = setFanState(false, reason);
    sendAck("FAN", ok, reason);
    return;
  }

  if (command == "PELTIER:ON") {
    String reason;
    const bool ok = setPeltierState(true, reason);
    if (!ok) sendDelta("SAFETY_REASON", reason);
    sendAck("PELTIER", ok, reason);
    return;
  }

  if (command == "PELTIER:OFF") {
    String reason;
    const bool ok = setPeltierState(false, reason);
    sendAck("PELTIER", ok, reason);
    return;
  }

  if (command == "ADAPTIVE:ON") {
    String reason;
    setAdaptiveState(true, reason);
    sendAck("ADAPTIVE", adaptiveOn, reason);
    return;
  }

  if (command == "ADAPTIVE:OFF") {
    String reason;
    setAdaptiveState(false, reason);
    sendAck("ADAPTIVE", true, reason);
    return;
  }

  if (command.startsWith("FANSPEED:")) {
    int value = 0;
    String reason;
    if (!parseIntRange(command.substring(9), FAN_PWM_MIN, FAN_PWM_MAX, value)) {
      sendAck("FANSPEED", false, "RANGE_50_100_STEP_10");
      return;
    }
    const bool ok = setFanSpeed(static_cast<uint8_t>(value), reason);
    sendAck("FANSPEED", ok, reason);
    return;
  }

  if (command.startsWith("RESET:")) {
    handleSettingsReset(command.substring(6));
    return;
  }

  if (command.startsWith("PHONE:BT=")) {
    float value = 0.0f;
    if (!parseFloatRange(command.substring(9), 0.0f, 100.0f, value)) {
      if (batteryValid || !isnan(batteryC)) {
        batteryValid = false;
        batteryC = NAN;
        lastBatteryZone = -1;
        adaptiveUpPending = false;
        adaptiveDownPending = false;
        sendDelta("BATTERY", "--");
        sendDelta("BATTERY_VALID", "0");
      }
      return;
    }

    if (!isfinite(value) || value < 0.0f || value > 100.0f) {
      if (batteryValid || !isnan(batteryC)) {
        batteryValid = false;
        batteryC = NAN;
        lastBatteryZone = -1;
        adaptiveUpPending = false;
        adaptiveDownPending = false;
        sendDelta("BATTERY", "--");
        sendDelta("BATTERY_VALID", "0");
      }
      return;
    }

    const int newZone = batteryZoneForStable(value);
    const bool wasValid = batteryValid;
    const bool zoneChanged = !wasValid || newZone != lastBatteryZone;
    const bool valueChanged = !wasValid || isnan(batteryC) || fabsf(batteryC - value) > 0.05f;

    batteryC = value;
    batteryValid = true;
    lastBatteryRxMs = millis();

    if (valueChanged) sendDelta("BATTERY", formatTemperature(batteryC, true));
    if (!wasValid) sendDelta("BATTERY_VALID", "1");

    if (zoneChanged) {
      lastBatteryZone = newZone;
      if (adaptiveOn) evaluateAdaptiveBattery();
    }
    return;
  }

  sendAck("COMMAND", false, "UNKNOWN");
}

static void processPendingOnlineOtaStart() {
  if (!onlineOtaStartPending || otaActive || bootApActive) return;

  const bool queueDrained = bleTxCount == 0;
  const bool txSettled = queueDrained && millis() >= bleNextNotifyAt;
  const bool waitExpired = millis() - onlineOtaRequestedAt >= ONLINE_OTA_START_WAIT_MS;
  if (!txSettled && !waitExpired && bleClientConnected) return;

  onlineOtaStartPending = false;
  startOnlineOta();
}

static void executeTouchSingle() {
  if (otaActive || adaptiveOn) return;

  Voltage next = Voltage::V5;
  if (currentVoltage == Voltage::V5) next = Voltage::V9;
  else if (currentVoltage == Voltage::V9) next = Voltage::V12;

  const bool accepted = requestVoltage(next, VoltageRequestSource::TOUCH, false);
  sendAck("TOUCH_VOLTAGE", accepted,
          accepted ? "OK" : "VOLTAGE_GUARD_OR_BUSY");
}

static void executeTouchDouble() {
  if (otaActive) return;
  setRgbState(!rgbOn);
  sendAck("TOUCH_RGB", true, "OK");
}

static void executeTouchLong() {
  if (otaActive || adaptiveOn) return;
  // Fan/Peltier are forced OFF during the initial boot AP window.
  // Touch remains active, but cooling cannot be started until startup completes.
  if (bootApActive && !bootApClientSafe) return;

  if (fanOn && peltierOn) {
    fanOn = false;
    peltierOn = false;
    applyCoolingOutputs();
    sendAck("TOUCH_COOLING", true, "OFF");
    return;
  }

  fanOn = true;
  peltierOn = true;
  applyCoolingOutputs();
  sendAck("TOUCH_COOLING", peltierOn, peltierOn ? "ON" : safetyReason());
}

static void onTouchSingleClick() {
  executeTouchSingle();
}

static void onTouchDoubleClick() {
  executeTouchDouble();
}

static void onTouchLongPress() {
  executeTouchLong();
}

// ============================================================================
// ====================== OTA PRIORITY SUPERVISOR — LOCKED ====================
// Executed first by loop(). OTA gets control before normal feature processing.
// ============================================================================
static bool serviceOtaPriority() {
  if (otaState == OtaState::SUCCESS &&
      otaRestartAt != 0 &&
      millis() >= otaRestartAt) {
    ESP.restart();
    return true;
  }

  if (bootApActive) {
    webServer.handleClient();
    const uint8_t clientCount = WiFi.softAPgetStationNum();

    if (clientCount > 0 && !bootApHadClient) {
      bootApHadClient = true;
      bootApClientSafe = true;
      applyOtaSafeOutputs();
      stopUserLedOutput();
      stopBleForBootWiFiClient();
    } else if (clientCount == 0 &&
               bootApClientSafe &&
               otaState == OtaState::BOOT_AP &&
               !otaActive) {
      // Safety is tied to an actual Wi-Fi client/session, not to AP power-on.
      // If the client leaves before Local OTA starts, restore normal runtime
      // immediately while keeping BLE disabled until Wi-Fi is off.
      bootApClientSafe = false;
      bootApHadClient = false;
      Serial.println("WIFI AP: CLIENT DISCONNECTED -> NORMAL RUNTIME");
      return false;
    } else if (otaState != OtaState::LOCAL_UPLOAD &&
               !otaActive &&
               millis() - bootApStartedAt >= BOOT_AP_HARD_LIMIT_MS) {
      stopLocalOtaAp();
      bootApClientSafe = false;
      Serial.println("WIFI AP: HARD TIMEOUT -> BLE HANDOFF");
      bootApHadClient = false;
      bleStartPending = true;
      bleStartRetryAt = millis();
      return false;
    } else if (clientCount == 0 &&
               otaState != OtaState::LOCAL_UPLOAD &&
               !otaActive &&
               millis() - bootApStartedAt >= BOOT_AP_TIMEOUT_MS) {
      stopLocalOtaAp();
      bootApClientSafe = false;
      Serial.println("WIFI AP: TIMEOUT -> BLE HANDOFF");
      bootApHadClient = false;
      bleStartPending = true;
      bleStartRetryAt = millis();
      return false;
    }

    if (bootApClientSafe ||
        otaState == OtaState::LOCAL_UPLOAD ||
        otaActive) {
      processLeds();
      return true;
    }

    // No client connected: boot AP is only a waiting network service.
    // Normal LED/touch/application processing continues immediately.
    return false;
  }

  if (otaState == OtaState::LOCAL_UPLOAD) {
    webServer.handleClient();
    processLeds();
    return true;
  }

  if (otaActive &&
      (otaState != OtaState::SUCCESS ||
       onlineResultIndicatorActive)) {
    processOnlineOta();
    // Result indicators are rendered exclusively by processOnlineOta().
    // The normal LED renderer resumes only after the indicator/recovery state
    // has ended, preventing double-driving and clipped red/green flashes.
    if (!onlineResultIndicatorActive) {
      processLeds();
    }
    return true;
  }

  return false;
}

// ================ END OTA PRIORITY SUPERVISOR — LOCKED =====================

static void setupTouch() {
  pinMode(PIN_TOUCH, INPUT);
  touchButton.setDebounceMs(50);
  touchButton.setClickMs(500);
  touchButton.setPressMs(5000);
  touchButton.attachClick(onTouchSingleClick);
  touchButton.attachDoubleClick(onTouchDoubleClick);
  touchButton.attachLongPressStart(onTouchLongPress);
}

static void initializeRuntimeFeatures() {
  if (runtimeFeaturesReady) return;

  Serial.println("BOOT: INIT HARDWARE");

  pinMode(PIN_12V, OUTPUT);
  pinMode(PIN_9V, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_PELTIER, OUTPUT);
  digitalWrite(PIN_12V, OUTPUT_OFF);
  digitalWrite(PIN_9V, OUTPUT_OFF);
  digitalWrite(PIN_FAN, OUTPUT_OFF);
  digitalWrite(PIN_PELTIER, OUTPUT_OFF);

  pinMode(PIN_NTC, INPUT);
  pinMode(PIN_TOUCH, INPUT);
  setupTouch();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_NTC, ADC_11db);

  ledcAttach(PIN_FAN, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  ledcWrite(PIN_FAN, 0);
  ledcAttach(PIN_PELTIER, PELTIER_PWM_FREQ, PELTIER_PWM_RESOLUTION);
  lastPeltierDuty = 0;

  leds.begin();
  // Initialize the LED hardware to a known OFF frame first.
  // loadSettings() restores the persisted user LED state immediately after this.
  clearAllLeds();

  loadSettings();

  currentVoltage = Voltage::V5;
  pendingVoltage = Voltage::V5;
  fanOn = false;
  peltierOn = false;
  peltierSoftStarting = false;
  peltierSoftStartAtMs = 0;
  lastPeltierDuty = 0;
  adaptiveOn = false;
  adaptiveCeiling = Voltage::V12;
  fanPwm = 100;

  batteryValid = false;
  batteryC = NAN;
  lastBatteryZone = -1;
  ntcValid = false;
  hotsideC = NAN;

  runtimeFeaturesReady = true;

  lastNtcReadMs = millis() - NTC_READ_INTERVAL_MS;
  sampleNtcIfDue();
  enforceSafety();

  Serial.println("BOOT: FEATURES READY");
}

void setup() {
  otaDiagnosticStage = "BOOT_AP";
  otaDiagnosticError = "NONE";
  otaDiagnosticErrorCode = 0;
  otaDiagnosticProgress = 0;
  otaDiagnosticLastPublishedProgress = 255;

  Serial.begin(115200);
  Serial.println();
  Serial.print("JE X FYZ: BOOT ");
  Serial.print(FW_VERSION);
  Serial.println(" / CORE 3.3.7 ARDUINO STANDARD HEAP-DIAG");

  initializeRuntimeFeatures();

  // ========================================================================
  // ======================= OTA BOOT PATH — PROTECTED =======================
  // Match the Horizon Cooler runtime context: complete setup() first, then
  // enter the official Arduino Wi-Fi AP path from loop(). BLE is not started
  // while the boot AP window owns the single radio.
  // ========================================================================
  bootApPending = true;
  Serial.println("BOOT: WIFI AP DEFERRED TO LOOP");
  // ==================== END OTA BOOT PATH — PROTECTED =====================
}

void loop() {
  // =================== OTA LOOP PRIORITY — PROTECTED =======================
  // ================= OTA PRIORITY: DO NOT MOVE DOWN =================
  if (serviceOtaPriority()) return;

  if (bootApPending) {
    bootApPending = false;
    Serial.println("BOOT: START WIFI AP FROM LOOP");
    startLocalOtaAp();
  }

  serviceBleStartRecovery();

  if (bleLedResumePending.load(std::memory_order_acquire) && !otaActive && !bootApActive) {
    bleLedResumePending.store(false, std::memory_order_relaxed);
    if (rgbOn) {
      startUserLedEffect();
    } else {
      stopUserLedOutput();
    }
  }

  if (bleQueueResetPending) {
    bleQueueResetPending = false;
    clearBleTxQueue();
    clearBleRxQueue();
  }

  if (bleSyncPending && bleClientConnected && txCharacteristic != nullptr) {
    bleSyncPending = false;
    clearBleTxQueue();
    sendFullSync();
  }

  serviceBleTx();
  processBleRxQueue();
  processPendingOnlineOtaStart();

  if (serviceOtaPriority()) return;
  // ================ END OTA LOOP PRIORITY — PROTECTED ======================

  processHeapDiagPostAdvertisingChecks();

  touchButton.tick();
  sampleNtcIfDue();
  updateBatteryValidity();
  processAdaptiveHotTimer();
  processAdaptiveBatteryDown();
  processAdaptiveUpTimer();
  completeVoltageTransition();

  const bool bootCoolingLocked =
      bootApActive &&
      !bootApClientSafe &&
      otaState == OtaState::BOOT_AP;
  if (bootCoolingLocked) {
    fanOn = false;
    peltierOn = false;
    peltierSoftStarting = false;
    peltierSoftStartAtMs = 0;
    if (runtimeFeaturesReady) {
      ledcWrite(PIN_FAN, 0);
      if (lastPeltierDuty != 0) {
        ledcWrite(PIN_PELTIER, 0);
        lastPeltierDuty = 0;
      }
    }
  } else {
    enforceSafety();
  }

  processPeltierSoftStart();
  processSettingsPersistence();

  static uint32_t lastHotTelemetryMs = 0;
  if (millis() - lastHotTelemetryMs >= NTC_READ_INTERVAL_MS) {
    lastHotTelemetryMs = millis();
    sendHotsideTelemetry();
  }

  processLeds();
  serviceBleTx();

  yield();
}

