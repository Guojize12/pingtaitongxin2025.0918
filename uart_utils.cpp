#include "uart_utils.h"
#include "config.h"

static char lineBuf[LINE_BUF_MAX];
static size_t lineLen = 0;
static void (*lineHandler)(const char*) = nullptr;

void setLineHandler(void (*handler)(const char*)) {
  lineHandler = handler;
}

// Utility: simple Serial2 logger
void log2(const char* msg) { Serial2.println(msg); }
void log2Val(const char* k, int v) { Serial2.print(k); Serial2.println(v); }
void log2Str(const char* k, const char* v) { Serial2.print(k); Serial2.println(v); }

// UART0 write helpers
void sendRaw(const char* s) {
  Serial2.print("[UART0 TX RAW] ");
  Serial2.print(s);
  Serial.write((const uint8_t*)s, strlen(s));
}

void sendCmd(const char* cmd) {
  Serial2.print("[UART0 TX CMD] ");
  Serial2.print(cmd);
  Serial2.print("\\r\\n\n");
  Serial.write((const uint8_t*)cmd, strlen(cmd));
  Serial.write("\r\n");
}

// 串口0接收：每收到一个字节都同步打印到串口2
void readDTU() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    Serial2.write(c);
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        if (lineHandler) lineHandler(lineBuf);
        lineLen = 0;
      }
    } else {
      if (lineLen < LINE_BUF_MAX - 1) {
        lineBuf[lineLen++] = c;
      } else {
        lineLen = 0;
      }
    }
  }
}