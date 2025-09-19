#include "uart_utils.h"
#include "config.h"

// Line buffer
static char lineBuf[LINE_BUF_MAX];
static size_t lineLen = 0;
static void (*lineHandler)(const char*) = nullptr;

// Simple binary state (for testing: enter BIN mode when +MIPURC: "rtcp",...,<len>,$ is detected)
static bool binMode = false;
static int binExpected = 0;
static int binHave = 0;
static uint8_t binBuf[256];

// AT命令状态跟踪
static bool waitingForATResponse = false;

// 时间解析相关全局变量
volatile bool g_platformTimeParsed = false;
PlatformTime g_platformTime = {0};

// Print functions
void log2(const char* msg) { Serial2.println(msg); }
void log2Val(const char* k, int v) { Serial2.print(k); Serial2.println(v); }
void log2Str(const char* k, const char* v) { Serial2.print(k); Serial2.println(v); }

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
  
  // 设置等待AT命令响应的状态
  waitingForATResponse = true;
  Serial2.println("[DEBUG] Waiting for AT command response");
}

void setLineHandler(void (*handler)(const char*)) { lineHandler = handler; }

// Print HEX with detailed format
static void dumpHex(const uint8_t* d, int n) {
  Serial2.print("[HEX DUMP] ");
  for (int i = 0; i < n; ++i) {
    if (d[i] < 16) Serial2.print("0");
    Serial2.print(d[i], HEX);
    Serial2.print(" ");
    
    // 每16字节换行，便于阅读
    if ((i + 1) % 16 == 0) {
      Serial2.println();
      Serial2.print("[HEX DUMP] ");
    }
  }
  Serial2.println();
  
  // 打印ASCII字符（可打印字符）
  Serial2.print("[ASCII]    ");
  for (int i = 0; i < n; ++i) {
    if (d[i] >= 32 && d[i] <= 126) {
      Serial2.print((char)d[i]);
    } else {
      Serial2.print(".");
    }
    if ((i + 1) % 16 == 0) {
      Serial2.println();
      Serial2.print("[ASCII]    ");
    }
  }
  Serial2.println();
}

// Check +MIPURC line and enter binary mode
static bool parseMIPURC_EnterBin(const char* line) {
  Serial2.print("[DEBUG] parseMIPURC_EnterBin: "); Serial2.println(line);
  const char* p = strstr(line, "+MIPURC:");
  if (!p) return false;
  if (!strstr(line, "\"rtcp\"")) return false;
  int commaCount = 0;
  const char* scan = p; const char* lenStart = nullptr;
  while (*scan) {
    if (*scan == ',') {
      ++commaCount;
      if (commaCount == 2) lenStart = scan + 1;
      else if (commaCount == 3) break;
    }
    ++scan;
  }
  if (commaCount < 3 || !lenStart) {
    Serial2.println("[DEBUG] MIPURC parse failed: not enough commas");
    return false;
  }
  char tmp[12]; int ti = 0; const char* q = lenStart;
  while (*q && *q != ',' && ti < (int)sizeof(tmp) - 1) {
    if (*q >= '0' && *q <= '9') tmp[ti++] = *q;
    else break;
    ++q;
  }
  tmp[ti] = '\0';
  if (ti == 0) {
    Serial2.println("[DEBUG] MIPURC parse failed: empty length number");
    return false;
  }
  int totalLen = atoi(tmp);
  Serial2.print("[DEBUG] MIPURC parsed length: "); Serial2.println(totalLen);
  
  if (totalLen < 2 || totalLen > (int)sizeof(binBuf)) {
    Serial2.print("[DEBUG] MIPURC totalLen invalid: "); Serial2.println(totalLen);
    return false;
  }
  
  const char* dollar = strchr(line, '$');
  if (!dollar) {
    Serial2.println("[DEBUG] MIPURC no $ symbol, not entering BIN");
    return false;
  }
  
  // 检查$后面的数据是否是真正的二进制数据
  // 如果$后面只有1-2个字符，可能是AT命令响应的残留，不是真正的二进制数据
  int dollarLen = strlen(dollar);
  if (dollarLen < 3) {
    Serial2.print("[DEBUG] MIPURC $ data too short ("); Serial2.print(dollarLen);
    Serial2.println(" chars), not entering BIN");
    return false;
  }
  
  // 检查$后面的数据是否包含可打印字符（可能是文本数据）
  bool hasPrintableChars = false;
  for (int i = 1; i < dollarLen && i < 10; ++i) {  // 只检查前10个字符
    if (dollar[i] >= 32 && dollar[i] <= 126) {
      hasPrintableChars = true;
      break;
    }
  }
  
  if (hasPrintableChars) {
    Serial2.print("[DEBUG] MIPURC $ data contains printable chars: ");
    for (int i = 0; i < dollarLen && i < 20; ++i) {
      Serial2.print((char)dollar[i]);
    }
    Serial2.println(", not entering BIN");
    return false;
  }
  
  // 从$开始，接收指定长度的字节作为完整报文
  binMode = true;
  binExpected = totalLen;  // 使用解析出的长度
  binHave = 0;
  
  // 复制$及其后的字符到缓冲区
  int copyLen = (dollarLen > totalLen) ? totalLen : dollarLen;  // 最多复制totalLen字节
  for (int i = 0; i < copyLen; ++i) {
    binBuf[i] = (uint8_t)dollar[i];
  }
  binHave = copyLen;
  
  Serial2.print("[DEBUG] BIN mode started, expecting "); Serial2.print(totalLen);
  Serial2.print(" bytes, firstCopied="); Serial2.println(binHave);
  if (binHave > 0) { 
    Serial2.print("[DEBUG] BIN head HEX: "); 
    dumpHex(binBuf, binHave); 
  }
  return true;
}

// 解析平台时间数据
bool parsePlatformTime(const uint8_t* data, size_t len, PlatformTime* time) {
  // 打印接收到的数据长度
  Serial2.print("[DEBUG] Received packet length: "); Serial2.println((int)len);
  
  // 需要至少32字节才能包含完整的时间信息
  if (len < 32) {
    Serial2.print("[DEBUG] Time parse failed: packet too short (need 32, got "); 
    Serial2.print((int)len); Serial2.println(")");
    return false;
  }
  
  // 检查包头标识
  if (data[0] != '$') {
    Serial2.println("[DEBUG] Time parse failed: invalid header");
    return false;
  }
  
  // 检查CMD字段 (第17-18字节，高位在前)
  uint16_t cmd = (data[17] << 8) | data[18];
  Serial2.print("[DEBUG] Packet CMD: 0x"); Serial2.println(cmd, HEX);
  
  if (cmd != 0x0001) {
    Serial2.print("[DEBUG] Time parse failed: CMD mismatch (expected 0x0001, got 0x"); 
    Serial2.print(cmd, HEX); Serial2.println(")");
    return false;
  }
  
  // 解析时间字段 (从第24字节开始，持续到第30字节)
  // year: 2字节，高位在前
  time->year = (data[24] << 8) | data[25];
  time->month = data[26];
  time->day = data[27];
  time->hour = data[28];
  time->minute = data[29];
  time->second = data[30];
  
  // 验证时间数据的合理性
  if (time->year < 2000 || time->year > 2100 ||
      time->month < 1 || time->month > 12 ||
      time->day < 1 || time->day > 31 ||
      time->hour > 23 ||
      time->minute > 59 ||
      time->second > 59) {
    Serial2.println("[DEBUG] Time parse failed: invalid time values");
    return false;
  }
  
  Serial2.print("[DEBUG] Time parsed: ");
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

// Print complete binary packet
static void printBinPacket() {
  Serial2.println("==========================================");
  Serial2.print("[DEBUG] BIN packet complete, length="); Serial2.println(binHave);
  
  // 打印16进制数据
  dumpHex(binBuf, binHave);
  
  // 分析数据包结构
  if (binHave >= 32) {
    Serial2.println("[PACKET ANALYSIS]");
    Serial2.print("Header: $"); Serial2.println((char)binBuf[0]);
    Serial2.print("OpType: "); Serial2.println((int)binBuf[1]);
    Serial2.print("Length: "); Serial2.print((int)binBuf[2]); Serial2.print(" "); Serial2.println((int)binBuf[3]);
    Serial2.print("Device SN: ");
    for (int i = 4; i < 16; i++) {
      Serial2.print((char)binBuf[i]);
    }
    Serial2.println();
    Serial2.print("Version: 0x"); Serial2.println(binBuf[16], HEX);
    uint16_t cmd = (binBuf[17] << 8) | binBuf[18];
    Serial2.print("Command: 0x"); Serial2.println(cmd, HEX);
    Serial2.print("Device Model: 0x"); Serial2.println(binBuf[19], HEX);
    Serial2.print("Packet ID: "); Serial2.println((int)binBuf[20]);
    Serial2.print("Header CRC: 0x"); Serial2.print(binBuf[21], HEX); Serial2.print(" 0x"); Serial2.println(binBuf[22], HEX);
    Serial2.print("Time Data (bytes 24-30): ");
    Serial2.print("Year="); Serial2.print((int)binBuf[24]); Serial2.print(" "); Serial2.print((int)binBuf[25]);
    Serial2.print(" Month="); Serial2.print((int)binBuf[26]);
    Serial2.print(" Day="); Serial2.print((int)binBuf[27]);
    Serial2.print(" Hour="); Serial2.print((int)binBuf[28]);
    Serial2.print(" Minute="); Serial2.print((int)binBuf[29]);
    Serial2.print(" Second="); Serial2.println((int)binBuf[30]);
  }
  
  // 尝试解析时间数据
  PlatformTime parsedTime;
  if (parsePlatformTime(binBuf, binHave, &parsedTime)) {
    g_platformTime = parsedTime;
    g_platformTimeParsed = true;
    Serial2.println("[DEBUG] Platform time updated successfully");
  } else {
    Serial2.println("[DEBUG] Failed to parse platform time");
  }
  Serial2.println("==========================================");
}

// 简化的数据包解析函数
static void parsePacket(const uint8_t* data, size_t len) {
  if (len < 24) {
    Serial2.print("[DEBUG] Packet too short: "); Serial2.println((int)len);
    return;
  }
  
  // 检查第一个字节是否为0x24
  if (data[0] != 0x24) {
    Serial2.print("[DEBUG] Invalid start flag: 0x"); Serial2.println(data[0], HEX);
    return;
  }
  
  // 检查第二个字节是否为'R'
  if (data[1] != 'R') {
    Serial2.print("[DEBUG] Invalid operation type: 0x"); Serial2.println(data[1], HEX);
    return;
  }
  
  // 检查命令编号是否为0x0000 (第18-19字节，高位在前)
  uint16_t cmd = (data[17] << 8) | data[18];
  if (cmd != 0x0000) {
    Serial2.print("[DEBUG] Invalid command: 0x"); Serial2.println(cmd, HEX);
    return;
  }
  
  // 按16进制打印每个字段
  Serial2.println("==========================================");
  Serial2.println("[PACKET ANALYSIS - 24 bytes]");
  
  // 字段1: 开始标志 (第1字节)
  Serial2.print("Start Flag: 0x"); Serial2.println(data[0], HEX);
  
  // 字段2: 操作类型 (第2字节)
  Serial2.print("Operation Type: 0x"); Serial2.println(data[1], HEX);
  
  // 字段3: 数据长度 (第3-4字节，高位在前)
  uint16_t dataLen = (data[2] << 8) | data[3];
  Serial2.print("Data Length: 0x"); Serial2.print(data[2], HEX); Serial2.print(" 0x"); Serial2.println(data[3], HEX);
  Serial2.print("Data Length (decimal): "); Serial2.println(dataLen);
  
  // 字段4: 终端地址 (第5-16字节)
  Serial2.print("Terminal Address: ");
  for (int i = 4; i < 16; i++) {
    Serial2.print("0x"); Serial2.print(data[i], HEX); Serial2.print(" ");
  }
  Serial2.println();
  
  // 字段5: 版本号 (第17字节)
  Serial2.print("Version: 0x"); Serial2.println(data[16], HEX);
  
  // 字段6: 命令编号 (第18-19字节，高位在前)
  Serial2.print("Command: 0x"); Serial2.print(data[17], HEX); Serial2.print(" 0x"); Serial2.println(data[18], HEX);
  Serial2.print("Command (decimal): "); Serial2.println(cmd);
  
  // 字段7: 设备类型 (第20字节)
  Serial2.print("Device Type: 0x"); Serial2.println(data[19], HEX);
  
  // 字段8: 包序号 (第21字节)
  Serial2.print("Packet ID: 0x"); Serial2.println(data[20], HEX);
  
  // 字段9: CRC值 (第22-23字节，高位在前)
  uint16_t crc = (data[21] << 8) | data[22];
  Serial2.print("CRC: 0x"); Serial2.print(data[21], HEX); Serial2.print(" 0x"); Serial2.println(data[22], HEX);
  Serial2.print("CRC (decimal): "); Serial2.println(crc);
  
  // 字段10: 额外数据 (第24字节)
  Serial2.print("Extra Data: 0x"); Serial2.println(data[23], HEX);
  
  Serial2.println("==========================================");
  
  // 如果有时间数据（第24-30字节），解析时间
  if (len >= 30) {
    PlatformTime parsedTime;
    if (parsePlatformTime(data, len, &parsedTime)) {
      g_platformTime = parsedTime;
      g_platformTimeParsed = true;
      Serial2.println("[DEBUG] Platform time updated successfully");
    }
  }
}

void readDTU() {
  static uint8_t packetBuf[256];
  static size_t packetLen = 0;
  
  while (Serial.available()) {
    uint8_t c = Serial.read();
    Serial2.print("[DEBUG] Got byte: "); Serial2.print((int)c); Serial2.print(" '"); 
    if (c >= 32 && c <= 126) Serial2.print((char)c); else Serial2.print("?"); 
    Serial2.println("'");

    // Binary mode (处理二进制数据包)
    if (binMode) {
      if (binHave < binExpected) {
        binBuf[binHave++] = c;
        Serial2.print("[DEBUG] BIN progress: "); Serial2.print(binHave); Serial2.print("/"); Serial2.println(binExpected);
        if (binHave == binExpected) {
          printBinPacket();
          binMode = false;
          binHave = 0;
          binExpected = 0;
          Serial2.println("[DEBUG] Binary mode completed and exited");
        }
      }
      continue;
    }

    // 检查是否是数据包开始 (0x24) - 新的数据包解析逻辑
    if (packetLen == 0 && c == 0x24) {
      packetBuf[packetLen++] = c;
      Serial2.print("[DEBUG] Packet start detected: 0x"); Serial2.println(c, HEX);
      continue;
    }
    
    // 如果已经开始接收数据包
    if (packetLen > 0) {
      packetBuf[packetLen++] = c;
      
      // 检查是否接收完整的数据包（24字节）
      if (packetLen >= 24) {
        // 解析数据包
        parsePacket(packetBuf, packetLen);
        
        // 重置缓冲区
        packetLen = 0;
      }
      
      // 防止缓冲区溢出
      if (packetLen >= sizeof(packetBuf)) {
        Serial2.println("[DEBUG] Packet buffer overflow, reset");
        packetLen = 0;
      }
      continue;
    }

    // Line mode (处理AT命令响应和文本行)
    if (c == '\r') {
      Serial2.println("[DEBUG] Got CR");
      continue;
    }
    if (c == '\n') {
      Serial2.println("[DEBUG] Got LF, line end");
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        Serial2.print("[DEBUG] Line received: "); Serial2.println(lineBuf);
        
        // 如果正在等待AT命令响应，检查是否是AT命令响应
        if (waitingForATResponse) {
          if (strstr(lineBuf, "+MIPSEND:") || strstr(lineBuf, "OK") || strstr(lineBuf, "ERROR") || 
              strstr(lineBuf, "+MATREADY") || strstr(lineBuf, "+CEREG") || strstr(lineBuf, "+MIPURC")) {
            Serial2.print("[DEBUG] AT command response received: "); Serial2.println(lineBuf);
            waitingForATResponse = false;  // 重置状态
            // 继续处理，让handleLine函数也能收到这个响应
          }
        }
        
        // Check if enter BIN mode
        if (parseMIPURC_EnterBin(lineBuf)) {
          Serial2.println("[DEBUG] Entered BIN mode on this line, subsequent bytes will be collected as binary");
        } else {
          if (lineHandler) lineHandler(lineBuf);
        }
        lineLen = 0;
      }
    } else {
      if (lineLen < LINE_BUF_MAX - 1) {
        lineBuf[lineLen++] = (char)c;
        Serial2.print("[DEBUG] Line accumulate: "); Serial2.write(lineBuf, lineLen); Serial2.println();
      } else {
        Serial2.println("[DEBUG] Line buffer overflow, reset");
        lineLen = 0;
      }
    }
  }
}