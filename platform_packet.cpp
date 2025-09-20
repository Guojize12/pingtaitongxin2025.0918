#include "platform_packet.h"
#include "config.h"
#include "crc16.h"
#include "uart_utils.h"
#include <string.h>

// 头部固定长度
static const uint16_t PLATFORM_HEADER_LEN = 21;

// 构建数据包：头(21) + 头CRC(2,大端) + [payload] + [payloadCRC(2,大端)]
size_t build_platform_packet(uint8_t* out,
                             char opType,
                             uint16_t cmd,
                             uint8_t pid,
                             const uint8_t* payload,
                             uint16_t payloadLen)
{
    out[0]  = '$';
    out[1]  = (uint8_t)opType;
    out[2]  = (uint8_t)(payloadLen >> 8);
    out[3]  = (uint8_t)(payloadLen & 0xFF);
    for (int i = 0; i < 12; ++i) out[4 + i] = (uint8_t)g_device_sn[i];
    out[16] = PLATFORM_VER;
    out[17] = (uint8_t)(cmd >> 8);
    out[18] = (uint8_t)(cmd & 0xFF);
    out[19] = PLATFORM_DMODEL;
    out[20] = pid;

    uint16_t headCrc = crc16_modbus(out, PLATFORM_HEADER_LEN);
    out[PLATFORM_HEADER_LEN]     = (uint8_t)(headCrc >> 8);
    out[PLATFORM_HEADER_LEN + 1] = (uint8_t)(headCrc & 0xFF);

    size_t offset = PLATFORM_HEADER_LEN + 2;

    if (payloadLen > 0 && payload) {
        memcpy(out + offset, payload, payloadLen);
        offset += payloadLen;
        uint16_t dataCrc = crc16_modbus(payload, payloadLen);
        out[offset]     = (uint8_t)(dataCrc >> 8);
        out[offset + 1] = (uint8_t)(dataCrc & 0xFF);
        offset += 2;
    }
    return offset;
}

// 发送（HEX封装）
void sendPlatformPacket(char opType,
                        uint16_t cmd,
                        uint8_t pid,
                        const uint8_t* payload,
                        uint16_t payloadLen)
{
    uint8_t pkt[256];
    size_t n = build_platform_packet(pkt, opType, cmd, pid, payload, payloadLen);
    static const char* HEXCHARS = "0123456789ABCDEF";

    // 构造 AT 命令
    static char buf[600];
    size_t pos = 0;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "AT+MIPSEND=0,0,");

    for (size_t i = 0; i < n; ++i) {
        uint8_t b = pkt[i];
        buf[pos++] = HEXCHARS[(b >> 4) & 0x0F];
        buf[pos++] = HEXCHARS[b & 0x0F];
    }
    buf[pos++] = '\r';
    buf[pos++] = '\n';
    buf[pos] = '\0';

    // 只保留关键日志
    Serial2.print("[UART0 TX PlatformPkt HEX] ");
    for (size_t i = 0; i < n; ++i) {
        if (pkt[i] < 16) Serial2.print("0");
        Serial2.print(pkt[i], HEX);
        Serial2.print(" ");
    }
    Serial2.println();
    Serial2.print("Send PlatformPkt len=");
    Serial2.println((int)n);

    Serial.write((const uint8_t*)buf, pos);
}

void sendHeartbeat() {
    sendPlatformPacket('R', CMD_HEARTBEAT_REQ, 0, nullptr, 0);
}

// year字段2字节，高位在前，payload长度15
void sendRealtimeMonitorData(
    uint16_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    uint8_t dataFmt,
    const uint8_t* exceptionStatus,
    uint8_t waterStatus
) {
    uint8_t payload[15];
    payload[0] = (uint8_t)(year >> 8);     // 高字节
    payload[1] = (uint8_t)(year & 0xFF);   // 低字节
    payload[2] = month;
    payload[3] = day;
    payload[4] = hour;
    payload[5] = minute;
    payload[6] = second;
    payload[7] = dataFmt;
    for (int i = 0; i < 6; ++i) {
        payload[8 + i] = exceptionStatus ? exceptionStatus[i] : 0;
    }
    payload[14] = waterStatus;
    sendPlatformPacket('R', 0x1d00, 0, payload, sizeof(payload));
}

void sendMonitorEventUpload(
    uint16_t year,
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
    if (imageLen > 65000) imageLen = 65000;
    uint32_t totalLen = 20 + imageLen; // year占2字节
    uint8_t* payload = (uint8_t*)malloc(totalLen);
    if (!payload) return;

    payload[0] = (uint8_t)(year >> 8);
    payload[1] = (uint8_t)(year & 0xFF);
    payload[2] = month;
    payload[3] = day;
    payload[4] = hour;
    payload[5] = minute;
    payload[6] = second;
    payload[7] = triggerCond;
    memcpy(payload + 8, &realtimeValue, 4);
    memcpy(payload + 12, &thresholdValue, 4);
    payload[16] = (uint8_t)(imageLen & 0xFF);
    payload[17] = (uint8_t)((imageLen >> 8) & 0xFF);
    payload[18] = (uint8_t)((imageLen >> 16) & 0xFF);
    payload[19] = (uint8_t)((imageLen >> 24) & 0xFF);
    if (imageLen > 0 && imageData) {
        memcpy(payload + 20, imageData, imageLen);
    }
    sendPlatformPacket('R', 0x1d09, 0, payload, (uint16_t)totalLen);
    free(payload);
}

void sendTimeSyncRequest() 
{
    sendPlatformPacket('R', CMD_TIME_SYNC_REQ, 0, nullptr, 0);
}