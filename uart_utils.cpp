#include "uart_utils.h"
#include "config.h"
#include "rtc_soft.h"    // 新增

// Line buffer
static char lineBuf[LINE_BUF_MAX];
static size_t lineLen = 0;
static void (*lineHandler)(const char*) = nullptr;

// 时间解析相关全局变量
volatile bool g_platformTimeParsed = false;
PlatformTime g_platformTime = {0};

// Print functions
void log2(const char* msg) { Serial2.println(msg); }
void log2Val(const char* k, int v) { Serial2.print(k); Serial2.println(v); }
void log2Str(const char* k, const char* v) { Serial2.print(k); Serial2.println(v); }

void sendRaw(const char* s) {
  Serial.write((const uint8_t*)s, strlen(s));
}

void sendCmd(const char* cmd) {
  Serial.write((const uint8_t*)cmd, strlen(cmd));
  Serial.write("\r\n");
}

void setLineHandler(void (*handler)(const char*)) { lineHandler = handler; }

// 只打印HEX不分行
static void dumpHex(const uint8_t* d, int n) 
{
  Serial2.print("[HEX DUMP] ");
  for (int i = 0; i < n; ++i) {
    if (d[i] < 16) Serial2.print("0");
    Serial2.print(d[i], HEX);
    Serial2.print(" ");
  }
  Serial2.println();
}

// 解析平台时间包（仅CMD=0x0001时）
bool parsePlatformTime(const uint8_t* data, size_t len, PlatformTime* time) {
  if (len < 32) return false;
  if (data[0] != '$') return false;
  uint16_t cmd = (data[17] << 8) | data[18];
  if (cmd != 0x0001) return false;
  time->year = (data[23] << 8) | data[24];
  time->month = data[25];
  time->day = data[26];
  time->hour = data[27];
  time->minute = data[28];
  time->second = data[29];
  if (time->year < 2000 || time->year > 2100 ||
      time->month < 1 || time->month > 12 ||
      time->day < 1 || time->day > 31 ||
      time->hour > 23 ||
      time->minute > 59 ||
      time->second > 59) {
    return false;
  }
  Serial2.print("Platform time: ");
  Serial2.print(time->year);
  Serial2.print("-");
  if (time->month < 10) Serial2.print("0");
  Serial2.print(time->month);
  Serial2.print("-");
  if (time->day < 10) Serial2.print("0");
  Serial2.print(time->day);
  Serial2.print(" ");
  if (time->hour < 10) Serial2.print("0");
  Serial2.print(time->hour);
  Serial2.print(":");
  if (time->minute < 10) Serial2.print("0");
  Serial2.print(time->minute);
  Serial2.print(":");
  if (time->second < 10) Serial2.print("0");
  Serial2.println(time->second);
  return true;
}

// 按协议头动态接收完整包，并只在CMD=0x0001时解析时间
void readDTU() {
  static uint8_t packetBuf[256];
  static size_t packetLen = 0;
  static size_t expectedPacketLen = 0;

  while (Serial.available()) {
    uint8_t c = Serial.read();

    // 检查是否是数据包开始 (0x24)
    if (packetLen == 0 && c == 0x24) {
      packetBuf[packetLen++] = c;
      expectedPacketLen = 0;
      continue;
    }
    // 如果已经开始接收数据包
    if (packetLen > 0) {
      packetBuf[packetLen++] = c;
      if (packetLen == 4) {
        // 已收到头4字节，可以解析长度
        uint16_t dataLen = (packetBuf[2] << 8) | packetBuf[3];
        expectedPacketLen = 21 + 2 + dataLen + (dataLen > 0 ? 2 : 0);
        if (expectedPacketLen > sizeof(packetBuf)) expectedPacketLen = sizeof(packetBuf); // 防溢出
      }
      // 收齐一包再处理
      if (expectedPacketLen > 0 && packetLen >= expectedPacketLen) {
        dumpHex(packetBuf, packetLen);
        // 只在CMD=0x0001时解析时间
        if (packetLen >= 32) {
          uint16_t cmd = (packetBuf[17] << 8) | packetBuf[18];
          if (cmd == 0x0001) {
            PlatformTime parsedTime;
            if (parsePlatformTime(packetBuf, packetLen, &parsedTime)) {
              g_platformTime = parsedTime;
              g_platformTimeParsed = true;
              rtc_on_sync(&parsedTime, millis()); // 新增：收到即校RTC
            }
          }
        }
        packetLen = 0;
        expectedPacketLen = 0;
      }
      if (packetLen >= sizeof(packetBuf)) {
        packetLen = 0;
        expectedPacketLen = 0;
      }
      continue;
    }
    // 文本行模式
    if (c == '\r' || c == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        if (lineHandler) lineHandler(lineBuf);
        lineLen = 0;
      }
    } else {
      if (lineLen < LINE_BUF_MAX - 1) {
        lineBuf[lineLen++] = (char)c;
      } else {
        lineLen = 0;
      }
    }
  }
}