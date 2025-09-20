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
    uint16_t year,
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
);

void sendTimeSyncRequest();

// ================= 新增：开机状态上报接口和状态码 =================
void sendStartupStatusReport(
    uint16_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    int32_t statusWord
);

// 状态字枚举（部分，详见协议图片，可根据需求补充）
#define STATUS_OK                0
#define STATUS_BOOT_MODE         1
#define STATUS_WAKEUP            2
#define STATUS_UPGRADE_WAIT      3
#define STATUS_APP_MD5_ERR       100
#define STATUS_APP_UPGRADE_FAIL  101
#define STATUS_NOT_UPGRADE       102
#define STATUS_GET_UPGRADE_INFO_TIMEOUT 103
#define STATUS_UPGRADE_MD5_ERR   104
#define STATUS_UPGRADE_DATA_ERR  105
#define STATUS_GET_UPGRADE_BLOCK_ERR 106
#define STATUS_GET_UPGRADE_DATA_TIMEOUT 107
#define STATUS_UPGRADE_FILE_TOO_BIG 108
#define STATUS_UPGRADE_SUCCESS_BUT_FAIL_MANY_TIMES 109