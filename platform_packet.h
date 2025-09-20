#pragma once
#include <Arduino.h>

// 构建平台数据包（双CRC：头CRC + 数据CRC）
// 返回总长度
size_t build_platform_packet(uint8_t* out,
                             char opType,
                             uint16_t cmd,
                             uint8_t pid,
                             const uint8_t* payload,
                             uint16_t payloadLen);

// 发送平台数据包
void sendPlatformPacket(char opType,
                        uint16_t cmd,
                        uint8_t pid,
                        const uint8_t* payload,
                        uint16_t payloadLen);

void sendHeartbeat();

void sendRealtimeMonitorData(
    uint8_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    uint8_t dataFmt,
    const uint8_t* exceptionStatus,
    uint8_t waterStatus
);

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
);

void sendTimeSyncRequest();