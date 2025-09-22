#include <Arduino.h>
#include "config.h"
#include "uart_utils.h"
#include "state_machine.h"
#include "platform_packet.h"
#include "at_commands.h"
#include "rtc_soft.h"

volatile int g_monitorEventUploadFlag = 0;
unsigned long lastRtcPrint = 0;

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
}

void loop() 
{
  readDTU();
  driveStateMachine();

  // 只在未校时时每10秒提示一次
  if (!rtc_is_valid() && millis() - lastRtcPrint > 10000) {
    lastRtcPrint = millis();
#if ENABLE_LOG2
    Serial2.println("[RTC] Not valid yet.");
#endif
  }
}