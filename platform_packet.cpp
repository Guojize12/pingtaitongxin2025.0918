#include "platform_packet.h"
#include "config.h"
#include "crc16.h"
#include "uart_utils.h"

// 构建平台协议数据包，含可选payload
size_t build_platform_packet(uint8_t* out, char opType, uint16_t cmd, uint8_t pid, const uint8_t* payload, uint16_t payloadLen) {
  out[0] = '$';
  out[1] = (uint8_t)opType;
  out[2] = (uint8_t)(payloadLen >> 8);
  out[3] = (uint8_t)(payloadLen & 0xFF);
  for (int i = 0; i < 12; ++i) out[4 + i] = (uint8_t)g_device_sn[i];
  out[16] = PLATFORM_VER;
  out[17] = (uint8_t)(cmd >> 8);
  out[18] = (uint8_t)(cmd & 0xFF);
  out[19] = PLATFORM_DMODEL;
  out[20] = pid;
  if (payloadLen > 0 && payload) {
    memcpy(out + 21, payload, payloadLen);
  }
  // CRC覆盖前21+payloadLen字节
  uint16_t crc = crc16_modbus(out, 21 + payloadLen);
  out[21 + payloadLen] = (uint8_t)((crc >> 8) & 0xFF);
  out[22 + payloadLen] = (uint8_t)(crc & 0xFF);
  return 23 + payloadLen;
}

// 发送平台数据包（HEX封装并AT命令发送）
void sendPlatformPacket(char opType, uint16_t cmd, uint8_t pid, const uint8_t* payload, uint16_t payloadLen) 
{
  uint8_t pkt[64];
  size_t n = build_platform_packet(pkt, opType, cmd, pid, payload, payloadLen);

  // HEX格式封装和发送
  const size_t maxNeeded = 18 + (n * 2) + 2 + 4;
  static char buf[256];
  if (maxNeeded >= sizeof(buf)) {
    Serial2.println("Platform packet buffer too small");
    return;
  }
  size_t pos = 0;
  pos += snprintf(buf + pos, sizeof(buf) - pos, "AT+MIPSEND=0,0,");
  static const char* hex = "0123456789ABCDEF";
  for (size_t i = 0; i < n; ++i) {
    uint8_t b = pkt[i];
    buf[pos++] = hex[(b >> 4) & 0x0F];
    buf[pos++] = hex[b & 0x0F];
  }
  buf[pos++] = '\r';
  buf[pos++] = '\n';
  buf[pos] = '\0';

  // 打印HEX内容到串口2
  Serial2.print("[UART0 TX PlatformPkt HEX] ");
  for (size_t i = 0; i < n; ++i) {
    if (pkt[i] < 16) Serial2.print("0");
    Serial2.print(pkt[i], HEX);
    Serial2.print(" ");
  }
  Serial2.println();
  Serial2.print("Send PlatformPkt len=");
  Serial2.println((int)n);

  Serial2.print("[UART0 TX CMD] ");
  Serial2.print(buf);
  Serial2.print("\n");

  Serial.write((const uint8_t*)buf, pos);
}


void sendHeartbeat() {
  sendPlatformPacket('R', CMD_HEARTBEAT_REQ, 0, nullptr, 0);
}

void sendRealtimeMonitorData
(
    uint8_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    uint8_t dataFmt,
    const uint8_t* exceptionStatus, // 6字节
    uint8_t waterStatus
) 
{
    uint8_t payload[14];
    payload[0] = year;
    payload[1] = month;
    payload[2] = day;
    payload[3] = hour;
    payload[4] = minute;
    payload[5] = second;
    payload[6] = dataFmt;
    for (int i = 0; i < 6; ++i) 
    {
        payload[7 + i] = exceptionStatus ? exceptionStatus[i] : 0;
    }
    payload[13] = waterStatus;
    sendPlatformPacket('R', 0x1d00, 0, payload, sizeof(payload));
}

#include <string.h>

// 塔吊基座监测事件上传
void sendMonitorEventUpload(
    uint8_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    uint8_t triggerCond,
    float realtimeValue,
    float thresholdValue,
    const uint8_t* imageData,
    uint32_t imageLen
) {
    if (imageLen > 65000) imageLen = 65000; // 协议最大限制
    uint32_t totalLen = 19 + imageLen;
    uint8_t* payload = (uint8_t*)malloc(totalLen);
    if (!payload) return;

    payload[0] = year;
    payload[1] = month;
    payload[2] = day;
    payload[3] = hour;
    payload[4] = minute;
    payload[5] = second;
    payload[6] = triggerCond;

    // 实时值 float
    memcpy(payload + 7, &realtimeValue, 4);
    // 触发阈值 float
    memcpy(payload + 11, &thresholdValue, 4);
    // 图片长度 int32
    payload[15] = (uint8_t)(imageLen & 0xFF);
    payload[16] = (uint8_t)((imageLen >> 8) & 0xFF);
    payload[17] = (uint8_t)((imageLen >> 16) & 0xFF);
    payload[18] = (uint8_t)((imageLen >> 24) & 0xFF);
    // 图片数据
    if (imageLen > 0 && imageData) {
        memcpy(payload + 19, imageData, imageLen);
    }
    sendPlatformPacket('R', 0x1d09, 0, payload, totalLen);
    free(payload);
}

