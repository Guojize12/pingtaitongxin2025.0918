#include <Arduino.h>
#include "config.h"
#include "uart_utils.h"
#include "state_machine.h"
#include "platform_packet.h"
#include "at_commands.h"
#include "rtc_soft.h"
#include "capture_trigger.h"
#include "camera_module.h"
#include "sdcard_module.h"
#include "sd_async.h"
#include "flash_module.h"
#include <Preferences.h>
#include "esp_system.h"

volatile int g_monitorEventUploadFlag = 0;
unsigned long lastRtcPrint = 0;

RuntimeConfig g_cfg = {true, true, true};
RunStats g_stats = {0};
UpgradeState g_upg = {0};
bool g_debugMode = false;

Preferences prefs;
uint32_t photo_index = 1;
uint8_t g_flashDuty = DEFAULT_FLASH_DUTY;
uint32_t last_params_saved_ms = 0;

// 按钮消抖及 busy 标志
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // ms
bool captureBusy = false;

// 长按判定（非阻塞）
static bool waitingForLongPress = false;
static unsigned long buttonPressStartMs = 0;
static const unsigned long LONG_PRESS_MS = 10000UL; // 10秒

// 全局水浸传感器“报警/持续按住”状态（0=正常，1=持续按住）
volatile uint8_t g_waterSensorStatus = 0;
// 持续按住模式起始时间（用于10分钟周期对齐）
volatile uint32_t g_waterHoldStartMs = 0;

// Recovery configuration
static const int MAX_INIT_RECOVERY_TRIES = 5;
static const uint32_t STAGE_INIT_DELAY_MS = 250; // delay between heavy inits to avoid inrush

static void logResetReason() {
  // print numeric reset reason; string mapping can be added if需要
  esp_reset_reason_t r = esp_reset_reason();
  Serial.print("Reset reason: "); Serial.println((int)r);
}

// Toggle optional hardware pins (if available) to force peripherals to a known reset/power state.
// This is best-effort: if hardware doesn't wire these pins, they are -1 and ignored.
static void pre_reset_peripherals_once() {
  // Camera PWDN/RESET
  if (PWDN_GPIO >= 0) {
    pinMode(PWDN_GPIO, OUTPUT);
    // conservative cycle: assert then deassert to ensure reset
    digitalWrite(PWDN_GPIO, LOW);
    delay(80);
    digitalWrite(PWDN_GPIO, HIGH);
    delay(80);
    digitalWrite(PWDN_GPIO, LOW);
    delay(120);
  }

  if (RESET_GPIO >= 0) {
    pinMode(RESET_GPIO, OUTPUT);
    digitalWrite(RESET_GPIO, LOW);
    delay(50);
    digitalWrite(RESET_GPIO, HIGH);
    delay(80);
  }

  // Modem power/reset pins (if present)
  if (MODEM_RESET_GPIO >= 0) {
    pinMode(MODEM_RESET_GPIO, OUTPUT);
    digitalWrite(MODEM_RESET_GPIO, LOW);
    delay(200);
    digitalWrite(MODEM_RESET_GPIO, HIGH);
    delay(200);
  }
  if (MODEM_PWR_GPIO >= 0) {
    pinMode(MODEM_PWR_GPIO, OUTPUT);
    // cycle power: off -> on
    digitalWrite(MODEM_PWR_GPIO, LOW);
    delay(200);
    digitalWrite(MODEM_PWR_GPIO, HIGH);
    delay(400);
  }

  // Optional global peripheral power enable (if present)
  if (PERIPH_POWER_EN_GPIO >= 0) {
    pinMode(PERIPH_POWER_EN_GPIO, OUTPUT);
    digitalWrite(PERIPH_POWER_EN_GPIO, LOW);
    delay(50);
    digitalWrite(PERIPH_POWER_EN_GPIO, HIGH);
    delay(120);
  }
}

// Attempt a set of recovery actions when init fails: soft modem power-cycle via AT,
// hardware pin toggles for camera/modem and small backoff
static void attempt_recovery_cycle(int attempt) {
  Serial.print("[RECOV] Recovery attempt "); Serial.println(attempt);
  // 1) Try soft power-cycle of modem via AT (best-effort)
  modem_soft_power_cycle();
  delay(200);

  // 2) Toggle camera power/reset pins
  if (PWDN_GPIO >= 0) {
    digitalWrite(PWDN_GPIO, HIGH); delay(60);
    digitalWrite(PWDN_GPIO, LOW);  delay(120);
  }
  if (RESET_GPIO >= 0) {
    digitalWrite(RESET_GPIO, LOW); delay(60);
    digitalWrite(RESET_GPIO, HIGH); delay(120);
  }

  // 3) If modem power control pin exists, toggle it
  if (MODEM_PWR_GPIO >= 0) {
    digitalWrite(MODEM_PWR_GPIO, LOW); delay(200);
    digitalWrite(MODEM_PWR_GPIO, HIGH); delay(400);
  }

  // small backoff increasing with attempts
  uint32_t backoff = 200 * (1UL << min(attempt, 6)); // up to ~12s
  if (backoff > 12000) backoff = 12000;
  delay(backoff);
}

void setup() {
  Serial.begin(DTU_BAUD);
#if ENABLE_LOG2
  Serial2.begin(LOG_BAUD, SERIAL_8N1, RX2, TX2);
#endif

  Serial.println("==== System Boot ====");
  logResetReason();

  // Pre-reset peripherals to clear possible latch-up / half-power states
  pre_reset_peripherals_once();

  // Staged initialization with limited recovery attempts
  int recovery_try = 0;
  bool camera_initialized = false;
  bool sd_initialized = false;

  while (recovery_try <= MAX_INIT_RECOVERY_TRIES) {
    Serial.println("1. Initializing Camera...");
    camera_initialized = init_camera_multi();
    camera_ok = camera_initialized;
    if (camera_initialized) {
      Serial.println("Camera OK");
    } else {
      Serial.println("[ERR] Camera init failed!");
    }

    delay(STAGE_INIT_DELAY_MS);

    Serial.println("2. Initializing SD Card...");
    init_sd();
    if (SD.cardType() == CARD_NONE) {
      Serial.println("[ERR] SD card init failed!");
      sd_initialized = false;
    } else {
      Serial.println("SD card OK");
      sd_initialized = true;
    }

    delay(STAGE_INIT_DELAY_MS);

    // Start async SD only if sd initialised
    if (sd_initialized) {
      sd_async_init();
      sd_async_start();
      sd_async_on_sd_ready();
    }

    // init flash pwm
    flashInit();

    // preferences / NVS
    Serial.println("3. Loading config from NVS...");
    if (!prefs.begin("cfg", false)) {
      Serial.println("[ERR] NVS init failed!");
    } else {
      load_params_from_nvs();
      Serial.println("NVS OK");
    }

    Serial.println("4. Initializing RTC (soft)...");
    rtc_init();
    Serial.println("RTC init finish (valid after time sync)");

    // Initial test photo - only when camera+sd ok
    if (camera_initialized && sd_initialized) {
      Serial.println("5. Taking initial test photo (save only)...");
      bool prevSend = g_cfg.sendEnabled;
      bool prevAsync = g_cfg.asyncSDWrite;
      g_cfg.sendEnabled = false;
      (void)capture_and_process(TRIGGER_BUTTON, false);
      g_cfg.sendEnabled = prevSend;
      g_cfg.asyncSDWrite = prevAsync;
      Serial.println("All hardware OK, ready to start platform connection...");
      break; // initialization OK
    }

    // If reached here, something failed. Try recovery sequence or escalate.
    recovery_try++;
    if (recovery_try > MAX_INIT_RECOVERY_TRIES) {
      Serial.println("[FATAL] Init failed after recovery attempts. Performing ESP.restart()");
      delay(200);
      ESP.restart();
      // not reached
    } else {
      Serial.println("[WARN] Init incomplete, attempt recovery actions...");
      attempt_recovery_cycle(recovery_try);
      // loop and attempt init again
    }
  } // end recovery loop

  resetBackoff();
  gotoStep(STEP_IDLE);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop()
{
  readDTU();
  driveStateMachine();

  // 按钮检测与消抖
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW) { // 被按下（低有效）
      if (g_waterSensorStatus == 1) {
        // 已处于“持续按住”模式：不重新计时，不触发新的10秒长按
        waitingForLongPress = false; // 防止误触发二次计时
      } else {
        // 未处于“持续按住”模式：正常进行10秒长按判定
        if (!waitingForLongPress) {
          waitingForLongPress = true;
          buttonPressStartMs = millis();
        } else {
          if (!captureBusy && (millis() - buttonPressStartMs >= LONG_PRESS_MS)) {
            captureBusy = true;
#if ENABLE_LOG2
            Serial2.println("[BTN] Long press >10s detected, capture once and enter hold mode.");
#endif
            bool ok = capture_and_process(TRIGGER_BUTTON, true);
#if ENABLE_LOG2
            if (ok) Serial2.println("[BTN] Capture saved; event flagged for upload.");
            else    Serial2.println("[BTN] Capture failed!");
#endif
            // 进入“持续按住”模式，并记录起始时间（供10分钟周期逻辑对齐）
            g_waterSensorStatus = 1;
            g_waterHoldStartMs = millis();

            waitingForLongPress = false;
            buttonPressStartMs = 0;
            captureBusy = false;
          }
        }
      }
    } else { // 松开
      g_waterSensorStatus = 0;
      g_waterHoldStartMs = 0;
      waitingForLongPress = false;
      buttonPressStartMs = 0;
    }
  }

  lastButtonState = reading;

  // 仅在未校时时每10秒提示一次
  if (!rtc_is_valid() && millis() - lastRtcPrint > 10000) {
    lastRtcPrint = millis();
#if ENABLE_LOG2
    Serial2.println("[RTC] Not valid yet.");
#endif
  }
}