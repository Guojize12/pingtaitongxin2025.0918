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

void setup() {
  Serial.begin(DTU_BAUD);
#if ENABLE_LOG2
  Serial2.begin(LOG_BAUD, SERIAL_8N1, RX2, TX2);
#endif

  Serial.println("==== System Boot ====");
  Serial.println("1. Initializing Camera...");
  bool camera_ok_local = init_camera_multi(); // 本地变量，防止全局camera_ok被其它代码提前用
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
    static int lastStableState = HIGH;
    if (reading == LOW && lastStableState == HIGH && !captureBusy) {
      captureBusy = true;
#if ENABLE_LOG2
      Serial2.println("[BTN] Button pressed, start capture!");
#endif
      // 拍照并上传
      bool ok = capture_and_process(1); // 1 = TRIGGER_BUTTON
#if ENABLE_LOG2
      if (ok) Serial2.println("[BTN] Capture and upload OK.");
      else Serial2.println("[BTN] Capture failed!");
#endif
      captureBusy = false;
    }
    lastStableState = reading;
  }
  lastButtonState = reading;

  // 只在未校时时每10秒提示一次
  if (!rtc_is_valid() && millis() - lastRtcPrint > 10000) {
    lastRtcPrint = millis();
#if ENABLE_LOG2
    Serial2.println("[RTC] Not valid yet.");
#endif
  }
}