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

void setup() {
  Serial.begin(DTU_BAUD);
#if ENABLE_LOG2
  Serial2.begin(LOG_BAUD, SERIAL_8N1, RX2, TX2);
#endif

  Serial.println("==== System Boot ====");
  Serial.println("1. Initializing Camera...");
  bool camera_ok_local = init_camera_multi();
  camera_ok = camera_ok_local;
  if (camera_ok_local) {
    Serial.println("Camera OK");
  } else {
    Serial.println("[ERR] Camera init failed!");
    while(1) delay(1000);
  }

  Serial.println("2. Initializing SD Card...");
  init_sd();
  if (SD.cardType() == CARD_NONE) {
    Serial.println("[ERR] SD card init failed!");
    while(1) delay(1000);
  } else {
    Serial.println("SD card OK");
  }

  // 初始化异步SD队列
  sd_async_init();
  sd_async_start();
  sd_async_on_sd_ready();

  // 初始化补光灯PWM
  flashInit();

  Serial.println("3. Loading config from NVS...");
  if (!prefs.begin("cfg", false)) {
    Serial.println("[ERR] NVS init failed!");
    while(1) delay(1000);
  }
  load_params_from_nvs();
  Serial.println("NVS OK");

  Serial.println("4. Initializing RTC (soft)...");
  rtc_init();
  Serial.println("RTC init finish (valid after time sync)");

  // 可选：上电自检只保存一张
  Serial.println("5. Taking initial test photo (save only)...");
  bool prevSend = g_cfg.sendEnabled;
  bool prevAsync = g_cfg.asyncSDWrite;
  g_cfg.sendEnabled = false;
  (void)capture_and_process(TRIGGER_BUTTON, false);
  g_cfg.sendEnabled = prevSend;
  g_cfg.asyncSDWrite = prevAsync;

  Serial.println("All hardware OK, ready to start platform connection...");

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
        // 保持 g_waterHoldStartMs 不变，由上传层每10分钟拍一次
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

            // 本次长按已完成，避免在未抬起时重启计时
            waitingForLongPress = false;
            buttonPressStartMs = 0;
            captureBusy = false;
          }
        }
      }
    } else { // 松开
      // 退出“持续按住”模式，复位所有计时
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