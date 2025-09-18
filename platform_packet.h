#pragma once

#include <Arduino.h>

// 构建平台数据包（通用，支持任意opType/cmd/payload），返回包长度
size_t build_platform_packet(uint8_t* out, char opType, uint16_t cmd, uint8_t pid, const uint8_t* payload, uint16_t payloadLen);

// 发送平台数据包（HEX封装并通过AT+MIPSEND发出）
void sendPlatformPacket(char opType, uint16_t cmd, uint8_t pid, const uint8_t* payload, uint16_t payloadLen);

// 发送心跳包
void sendHeartbeat();
//发送实时数据
// 塔吊基座监测实时数据上传
void sendRealtimeMonitorData(
    uint8_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    uint8_t dataFmt,
    const uint8_t* exceptionStatus, // 6字节
    uint8_t waterStatus
);

// 塔吊基座监测事件上传
void sendMonitorEventUpload
(
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

