#include <Arduino.h>
#include "config.h"
#include "uart_utils.h"
#include "state_machine.h"
#include "platform_packet.h"
#include "at_commands.h"
#include "rtc_soft.h"
#include "capture_trigger.h"

volatile int g_monitorEventUploadFlag = 0;
unsigned long lastRtcPrint = 0;

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

  log2("Boot");
  log2("UART0 for DTU, UART2 for log");
  log2Str("Server: ", SERVER_IP);
  log2Val("Port: ", SERVER_PORT);

  rtc_init();
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