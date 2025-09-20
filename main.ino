#include <Arduino.h>
#include "config.h"
#include "uart_utils.h"
#include "state_machine.h"
#include "platform_packet.h"
#include "at_commands.h"
#include "rtc_soft.h"    // RTC头文件, 确保已包含

volatile int g_monitorEventUploadFlag = 0;

// <--- 这里加上全局变量
unsigned long lastRtcPrint = 0;

void setup() {
  Serial.begin(DTU_BAUD);
  Serial2.begin(LOG_BAUD, SERIAL_8N1, RX2, TX2);

  log2("Boot");
  log2("UART0 for DTU, UART2 for log");
  log2Str("Server: ", SERVER_IP);
  log2Val("Port: ", SERVER_PORT);

  rtc_init();         // RTC初始化
  resetBackoff();
  gotoStep(STEP_IDLE);
}

void loop() 
{
  readDTU();
  driveStateMachine();
/*
  // 每5秒打印一次当前RTC时间，验证RTC是否好用
  if (millis() - lastRtcPrint > 5000) {
    lastRtcPrint = millis();
    if (rtc_is_valid()) {
      PlatformTime t;
      rtc_now_fields(&t);
      Serial2.print("[RTC] Now: ");
      Serial2.print(t.year); Serial2.print("-");
      Serial2.print((int)t.month); Serial2.print("-");
      Serial2.print((int)t.day); Serial2.print(" ");
      Serial2.print((int)t.hour); Serial2.print(":");
      Serial2.print((int)t.minute); Serial2.print(":");
      Serial2.print((int)t.second);
      Serial2.println();
    } else {
      Serial2.println("[RTC] Not valid yet.");
    }
  }
*/
}