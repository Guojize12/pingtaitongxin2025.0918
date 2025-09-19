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
}

void setLineHandler(void (*handler)(const char*)) { lineHandler = handler; }

// Print HEX
static void dumpHex(const uint8_t* d, int n) {
  for (int i = 0; i < n; ++i) {
    if (d[i] < 16) Serial2.print("0");
    Serial2.print(d[i], HEX);
    Serial2.print(" ");
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
  if (totalLen < 2 || totalLen > (int)sizeof(binBuf)) {
    Serial2.print("[DEBUG] MIPURC totalLen invalid: "); Serial2.println(totalLen);
    return false;
  }
  const char* dollar = strchr(line, '$');
  if (!dollar) {
    Serial2.println("[DEBUG] MIPURC no $ symbol, not entering BIN");
    return false;
  }
  // Copy all characters from $
  int n = strlen(dollar);
  if (n > totalLen) n = totalLen;
  for (int i = 0; i < n; ++i) binBuf[i] = (uint8_t)dollar[i];
  binMode = true;
  binExpected = totalLen;
  binHave = n;
  Serial2.print("[DEBUG] BIN mode started total="); Serial2.print(binExpected);
  Serial2.print(" firstCopied="); Serial2.println(binHave);
  if (binHave > 0) { Serial2.print("[DEBUG] BIN head HEX: "); dumpHex(binBuf, binHave); }
  return true;
}

// Print complete binary packet
static void printBinPacket() {
  Serial2.print("[DEBUG] BIN packet complete, length="); Serial2.println(binHave);
  dumpHex(binBuf, binHave);
}

void readDTU() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    Serial2.print("[DEBUG] Got char: "); Serial2.print((int)(uint8_t)c); Serial2.print(" '"); Serial2.print(c); Serial2.println("'");

    // Binary mode
    if (binMode) {
      if (binHave < binExpected) {
        binBuf[binHave++] = (uint8_t)c;
        Serial2.print("[DEBUG] BIN progress: "); Serial2.print(binHave); Serial2.print("/"); Serial2.println(binExpected);
        if (binHave == binExpected) {
          printBinPacket();
          binMode = false;
          binHave = 0;
          binExpected = 0;
        }
      }
      continue;
    }

    // Line mode
    if (c == '\r') {
      Serial2.println("[DEBUG] Got CR");
      continue;
    }
    if (c == '\n') {
      Serial2.println("[DEBUG] Got LF, line end");
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        Serial2.print("[DEBUG] Line received: "); Serial2.println(lineBuf);
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
        lineBuf[lineLen++] = c;
        Serial2.print("[DEBUG] Line accumulate: "); Serial2.write(lineBuf, lineLen); Serial2.println();
      } else {
        Serial2.println("[DEBUG] Line buffer overflow, reset");
        lineLen = 0;
      }
    }
  }
}