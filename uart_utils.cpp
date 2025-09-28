#include "uart_utils.h"
#include "config.h"
#include "rtc_soft.h"
#include <string.h>
#include <ctype.h>

// Line buffer
static char lineBuf[LINE_BUF_MAX];
static size_t lineLen = 0;
static void (*lineHandler)(const char*) = nullptr;

// 时间解析相关全局变量
volatile bool g_platformTimeParsed = false;
PlatformTime g_platformTime = {0};

#if ENABLE_LOG2
void log2(const char* msg) { Serial2.println(msg); }
void log2Val(const char* k, int v) { Serial2.print(k); Serial2.println(v); }
void log2Str(const char* k, const char* v) { Serial2.print(k); Serial2.println(v); }
#endif

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
#if ENABLE_LOG2
  Serial2.print("[HEX DUMP] ");
  for (int i = 0; i < n; ++i) {
    if (d[i] < 16) Serial2.print("0");
    Serial2.print(d[i], HEX);
    Serial2.print(" ");
  }
  Serial2.println();
#endif
}

// 协议数据包装配器（按0x24起始、头长21、双CRC长度规则）
static void feed_packet_bytes(const uint8_t* data, size_t len) {
  static uint8_t packetBuf[512];
  static size_t packetLen = 0;
  static size_t expectedPacketLen = 0;

  for (size_t i = 0; i < len; ++i) {
    uint8_t c = data[i];

    if (packetLen == 0) {
      if (c == 0x24) {
        packetBuf[packetLen++] = c;
        expectedPacketLen = 0;
      }
      continue;
    }

    packetBuf[packetLen++] = c;

    if (packetLen == 4) {
      // 已收到头4字节，可以解析payload长度
      uint16_t dataLen = (packetBuf[2] << 8) | packetBuf[3];
      expectedPacketLen = 21 + 2 + dataLen + (dataLen > 0 ? 2 : 0);
      if (expectedPacketLen > sizeof(packetBuf)) {
        // 超出缓冲，丢弃本包
        packetLen = 0;
        expectedPacketLen = 0;
        continue;
      }
    }

    if (expectedPacketLen > 0 && packetLen >= expectedPacketLen) {
      // 收齐一包
      dumpHex(packetBuf, packetLen);
      if (packetLen >= 32) {
        uint16_t cmd = (packetBuf[17] << 8) | packetBuf[18];
        if (cmd == 0x0001) {
          PlatformTime parsedTime;
          // 复用已有时间解析逻辑（要求整包）
          if (parsePlatformTime(packetBuf, packetLen, &parsedTime)) {
            g_platformTime = parsedTime;
            g_platformTimeParsed = true;
            rtc_on_sync(&parsedTime, millis()); // 收到即校RTC
          }
        }
      }
      packetLen = 0;
      expectedPacketLen = 0;
    }
  }
}

// 从一行 +MIPURC: "recv"... 提取HEX载荷，并解码喂给协议装配器
static bool handle_mipurc_recv_hex_line(const char* line) {
  // 仅处理包含 recv 的URC
  if (!strstr(line, "+MIPURC") || !strstr(line, "recv")) return false;

  // 取最后一个逗号后的字段，去掉引号得到HEX
  const char* lastComma = strrchr(line, ',');
  const char* token = nullptr;
  if (lastComma && *(lastComma + 1)) {
    token = lastComma + 1;
  } else {
    // 退而求其次：找最后一对引号
    const char* rquote = strrchr(line, '\"');
    if (!rquote) return false;
    const char* lquote = nullptr;
    for (const char* p = rquote - 1; p >= line; --p) {
      if (*p == '\"') { lquote = p; break; }
      if (p == line) break;
    }
    if (!lquote) return false;
    token = lquote + 1;
    size_t hexLen = (size_t)(rquote - lquote - 1);
    // 逐对HEX解码并喂给装配器
    uint8_t byte = 0;
    int nibbleCnt = 0;
    for (size_t i = 0; i < hexLen; ++i) {
      char ch = token[i];
      int v = -1;
      if (ch >= '0' && ch <= '9') v = ch - '0';
      else if (ch >= 'A' && ch <= 'F') v = 10 + (ch - 'A');
      else if (ch >= 'a' && ch <= 'f') v = 10 + (ch - 'a');
      else continue; // 跳过非HEX字符（容错）
      if (nibbleCnt == 0) {
        byte = (uint8_t)(v << 4);
        nibbleCnt = 1;
      } else {
        byte |= (uint8_t)v;
        feed_packet_bytes(&byte, 1);
        nibbleCnt = 0;
      }
    }
    return true;
  }

  // 去掉空格与引号
  while (*token == ' ' || *token == '\"') ++token;
  char hexbuf[512];
  size_t j = 0;
  for (const char* p = token; *p && j < sizeof(hexbuf) - 1; ++p) {
    if (*p == '\"' || *p == '\r' || *p == '\n') break;
    hexbuf[j++] = *p;
  }
  hexbuf[j] = 0;
  if (j < 2) return false;

  // 解码HEX
  uint8_t byte = 0;
  int nibbleCnt = 0;
  for (size_t i = 0; i < j; ++i) {
    char ch = hexbuf[i];
    int v = -1;
    if (ch >= '0' && ch <= '9') v = ch - '0';
    else if (ch >= 'A' && ch <= 'F') v = 10 + (ch - 'A');
    else if (ch >= 'a' && ch <= 'f') v = 10 + (ch - 'a');
    else continue;
    if (nibbleCnt == 0) {
      byte = (uint8_t)(v << 4);
      nibbleCnt = 1;
    } else {
      byte |= (uint8_t)v;
      feed_packet_bytes(&byte, 1);
      nibbleCnt = 0;
    }
  }
  return true;
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
#if ENABLE_LOG2
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
#endif
  return true;
}

// 统一读取串口：
// - 文本行用于AT状态机（原逻辑不变）
// - 同时拦截 +MIPURC: "recv"... 行，提取HEX还原二进制喂协议解析器，用于校时
// - 若未来模块支持原始二进制透传（很少见），下方原有的0x24起始解析也能兼容
void readDTU() {
  static uint8_t rawBuf[256];
  size_t rawLen = 0;

  while (Serial.available()) {
    uint8_t c = Serial.read();

    // 原始二进制流（极少用到，但保留容错）
    if (rawLen == 0 && c == 0x24) {
      rawBuf[rawLen++] = c;
      // 后续字节交给通用装配器
      continue;
    }
    if (rawLen > 0) {
      rawBuf[rawLen++] = c;
      if (rawLen >= sizeof(rawBuf)) {
        feed_packet_bytes(rawBuf, rawLen);
        rawLen = 0;
      }
      continue;
    }

    // 文本行模式
    if (c == '\r' || c == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        // 先走AT/通信状态机
        if (lineHandler) lineHandler(lineBuf);
        // 再尝试作为MIPURC接收行解码HEX载荷（用于平台协议包解析/校时）
        (void)handle_mipurc_recv_hex_line(lineBuf);
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

  // 刷新残留原始包缓冲
  if (rawLen > 0) {
    feed_packet_bytes(rawBuf, rawLen);
    rawLen = 0;
  }
}