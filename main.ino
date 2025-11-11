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
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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

// Recovery / init configuration
static const int MAX_INIT_RECOVERY_TRIES = 5;
static const uint32_t STAGE_INIT_DELAY_MS = 350; // 延长一点，降低同时唤醒概率
static const int SAFE_BOOT_THRESHOLD = 3; // 连续失败 >=3 次进入安全模式
static const int WDT_TIMEOUT_S = 12; // init task watchdog 超时时间（秒）
static const uint32_t INIT_STAGE_TIMEOUT_MS = 12000UL; // 每阶段最大等待时间

// 状态
static volatile bool g_init_complete = false;
static volatile bool g_safe_mode = false;

// forward
static void init_task(void* arg);

// 打印复位原因并统计 boot count（NVS）
static void log_and_count_boot() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.print("Reset reason: "); Serial.println((int)r);

  // 使用 preferences 计数连续失败的次数
  if (!prefs.begin("boot", false)) {
    Serial.println("[BOOT] NVS open failed");
    return;
  }
  uint32_t bc = prefs.getUInt("bootcnt", 0);
  bc++;
  prefs.putUInt("bootcnt", bc);
  Serial.print("[BOOT] boot_count="); Serial.println(bc);
  prefs.end();
}

// 清除 boot_count（在首次成功完成初始化时调用）
static void clear_boot_count() {
  if (!prefs.begin("boot", false)) return;
  prefs.putUInt("bootcnt", 0);
  prefs.end();
}

// 读取当前 boot count（用于判断是否进入安全模式）
static uint32_t read_boot_count() {
  if (!prefs.begin("boot", false)) return 0;
  uint32_t bc = prefs.getUInt("bootcnt", 0);
  prefs.end();
  return bc;
}

// 轻量的预复位（仅对已定义GPIO执行），保持与旧逻辑兼容
static void pre_reset_peripherals_once() {
  if (PWDN_GPIO >= 0) {
    pinMode(PWDN_GPIO, OUTPUT);
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
  // 其余可控电源/复位引脚若接线可在config.h中启用
  if (MODEM_RESET_GPIO >= 0) {
    pinMode(MODEM_RESET_GPIO, OUTPUT);
    digitalWrite(MODEM_RESET_GPIO, LOW);
    delay(200);
    digitalWrite(MODEM_RESET_GPIO, HIGH);
    delay(200);
  }
  if (MODEM_PWR_GPIO >= 0) {
    pinMode(MODEM_PWR_GPIO, OUTPUT);
    digitalWrite(MODEM_PWR_GPIO, LOW);
    delay(200);
    digitalWrite(MODEM_PWR_GPIO, HIGH);
    delay(400);
  }
  if (PERIPH_POWER_EN_GPIO >= 0) {
    pinMode(PERIPH_POWER_EN_GPIO, OUTPUT);
    digitalWrite(PERIPH_POWER_EN_GPIO, LOW);
    delay(50);
    digitalWrite(PERIPH_POWER_EN_GPIO, HIGH);
    delay(120);
  }
}

// 在 init_task 中按阶段初始化，带超时与 WDT 保护
static void run_staged_init(bool safe_mode) {
  // 如果进入安全模式，跳过 camera/SD 的启动
  Serial.print("[INIT] safe_mode=");
  Serial.println(safe_mode ? "YES" : "NO");

  bool camera_initialized = false;
  bool sd_initialized = false;

  if (!safe_mode) {
    // Camera init with stage timeout
    uint32_t t0 = millis();
    Serial.println("[INIT] Starting camera init...");
    // try init in a blocking call but we guard whole task by WDT and per-stage timeout
    camera_initialized = init_camera_multi();
    if (camera_initialized) {
      Serial.println("[INIT] Camera OK");
    } else {
      Serial.println("[INIT] Camera FAILED");
    }
    // if camera init exceeded stage time, it's okay: we check result and may recover later
    if (millis() - t0 > INIT_STAGE_TIMEOUT_MS) {
      Serial.println("[INIT] Camera init exceeded stage timeout");
    }
    vTaskDelay(pdMS_TO_TICKS(STAGE_INIT_DELAY_MS));
  } else {
    Serial.println("[INIT] Skipping camera in safe mode");
  }

  // SD init (non-blocking guarded)
  if (!safe_mode) {
    Serial.println("[INIT] Starting SD init...");
    uint32_t t0 = millis();
    init_sd(); // existing function; may be quick or blocking in some drivers
    // Wait briefly for card presence check, but don't block too long
    while (millis() - t0 < 2000) {
      if (SD.cardType() != CARD_NONE) { sd_initialized = true; break;}
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (sd_initialized) Serial.println("[INIT] SD OK");
    else Serial.println("[INIT] SD not ready");
    vTaskDelay(pdMS_TO_TICKS(STAGE_INIT_DELAY_MS));
  } else {
    Serial.println("[INIT] Skipping SD in safe mode");
  }

  // Start async SD task only if sd ok and not safe mode
  if (!safe_mode && sd_initialized) {
    if (sd_async_init()) {
      sd_async_start();
      sd_async_on_sd_ready();
      Serial.println("[INIT] SD async started");
    } else {
      Serial.println("[INIT] SD async init failed");
    }
  }

  // Flash / PWM / prefs / rtc are light-weight; do them last
  flashInit();

  if (!prefs.begin("cfg", false)) {
    Serial.println("[INIT] NVS prefs open failed");
  } else {
    load_params_from_nvs();
    prefs.end();
  }

  rtc_init();

  // initial test photo only if not in safe mode and both camera+sd present
  if (!safe_mode && camera_initialized && sd_initialized) {
    Serial.println("[INIT] Taking initial test photo (save only)...");
    bool prevSend = g_cfg.sendEnabled;
    bool prevAsync = g_cfg.asyncSDWrite;
    g_cfg.sendEnabled = false;
    (void)capture_and_process(TRIGGER_BUTTON, false);
    g_cfg.sendEnabled = prevSend;
    g_cfg.asyncSDWrite = prevAsync;
    Serial.println("[INIT] Initial test photo done");
  } else {
    Serial.println("[INIT] Skipping test photo");
  }

  // mark success: clear boot count
  clear_boot_count();
  g_init_complete = true;
}

// init_task: runs staged init; protected by task wdt
static void init_task(void* arg) {
  // register this task to task WDT
  esp_task_wdt_add(NULL);

  // read boot_count and decide safe mode
  uint32_t bc = read_boot_count();
  if (bc >= SAFE_BOOT_THRESHOLD) {
    g_safe_mode = true;
    Serial.println("[INIT_TASK] Entering SAFE MODE due to repeated boot failures");
  } else {
    g_safe_mode = false;
  }

  // attempt several recovery tries if necessary
  int attempt = 0;
  while (attempt <= MAX_INIT_RECOVERY_TRIES && !g_init_complete) {
    attempt++;
    Serial.print("[INIT_TASK] Attempt ");
    Serial.println(attempt);
    // feed watchdog periodically while doing init
    esp_task_wdt_reset();

    run_staged_init(g_safe_mode);

    // if init completed, break
    if (g_init_complete) break;

    // otherwise try some recovery actions (best-effort)
    Serial.println("[INIT_TASK] Init not complete, trying recovery actions...");
    // soft-power-cycle modem (AT commands best-effort)
    modem_soft_power_cycle();
    // toggle camera PWDN if available
    if (PWDN_GPIO >= 0) {
      digitalWrite(PWDN_GPIO, HIGH); vTaskDelay(pdMS_TO_TICKS(80));
      digitalWrite(PWDN_GPIO, LOW);  vTaskDelay(pdMS_TO_TICKS(150));
    }
    // backoff
    uint32_t backoff = 500 * (1UL << min(attempt, 6));
    if (backoff > 15000) backoff = 15000;
    Serial.print("[INIT_TASK] Backing off ");
    Serial.print(backoff);
    Serial.println(" ms");
    uint32_t st = millis();
    while (millis() - st < backoff) {
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }

  if (!g_init_complete) {
    Serial.println("[INIT_TASK] Initialization FAILED after attempts. Triggering restart.");
    // leave boot count increment as-is so next boot likely goes safe mode
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_task_wdt_reset();
    // perform controlled restart
    esp_task_wdt_delete(NULL);
    ESP.restart();
    // not reached
  } else {
    Serial.println("[INIT_TASK] Initialization COMPLETE.");
    // remove task wdt membership for this task since it's done
    esp_task_wdt_delete(NULL);
  }

  // terminate this task
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(DTU_BAUD);
#if ENABLE_LOG2
  Serial2.begin(LOG_BAUD, SERIAL_8N1, RX2, TX2);
#endif

  Serial.println("==== System Boot ====");
  // count boot and decide safe mode later in init_task
  if (!prefs.begin("boot", false)) {
    Serial.println("[BOOT] prefs begin failed");
  } else {
    uint32_t bc = prefs.getUInt("bootcnt", 0);
    Serial.print("[BOOT] previous boot_count=");
    Serial.println(bc);
    prefs.end();
  }

  log_and_count_boot();

  // small early pre-reset to clear possible latch-ups
  pre_reset_peripherals_once();

  // create init task with its own watchdog
  BaseType_t rc = xTaskCreatePinnedToCore(init_task, "init_task",
                                          8192, nullptr,
                                          2, nullptr, tskNO_AFFINITY);
  if (rc != pdPASS) {
    Serial.println("[BOOT] Failed to create init_task - performing direct init fallback");
    // fallback: do direct init in-line but with conservative delays
    run_staged_init(false);
    g_init_complete = true;
    clear_boot_count();
  }

  // Mark comm state machine ready to run; comm/state/upload will wait for init flags appropriately
  resetBackoff();
  gotoStep(STEP_IDLE);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop()
{
  // If initialization not complete, still drive minimal IO: readDTU and comm state to keep serial processing alive.
  readDTU();
  driveStateMachine();

  // 如果摄像头处于不可用状态，周期性尝试按退避策略重建
  if (!camera_ok) {
    attempt_camera_reinit_with_backoff();
  }  

  // If init not complete, avoid heavy tasks in loop (no periodic captures, etc.)
  if (!g_init_complete) {
    // small sleep to yield
    delay(50);
    return;
  }

  // 主运行逻辑（仅在 init 完成后执行）
  // 按钮检测与消抖
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW) { // 被按下（低有效）
      if (g_waterSensorStatus == 1) {
        waitingForLongPress = false;
      } else {
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