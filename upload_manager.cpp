#include "upload_manager.h"
#include "config.h"
#include "platform_packet.h"
#include "comm_manager.h"
#include <Arduino.h>

// 定时上传的计时器
static uint32_t lastRealtimeUploadMs = 0;

// 事件上传标志（在 main.ino 中定义，这里只声明使用）
extern volatile int g_monitorEventUploadFlag;

static void uploadRealtimeDataIfNeeded(uint32_t now) {
    if (!comm_isConnected()) return;
    if (now - lastRealtimeUploadMs < REALTIME_UPLOAD_INTERVAL_MS) return;

    // 将这里替换为你的真实采集数据
    sendRealtimeMonitorData(
        25, 9, 16, 13, 5, 45,   // year, month, day, hour, min, sec
        0,                      // dataFmt
        nullptr,                // exceptionStatus
        0                       // waterStatus
    );

    lastRealtimeUploadMs = now;
}

static void uploadMonitorEventIfNeeded() {
    if (!comm_isConnected()) return;
    if (g_monitorEventUploadFlag != 1) return;

    // 将这里替换为你的真实事件数据
    uint8_t year = 25, month = 9, day = 16, hour = 13, minute = 5, second = 45;
    uint8_t triggerCond = 1;
    float realtimeValue = 12.34f;
    float thresholdValue = 10.00f;
    const uint8_t* imageData = nullptr;
    uint32_t imageLen = 0;

    sendMonitorEventUpload(
        year, month, day, hour, minute, second, triggerCond,
        realtimeValue, thresholdValue, imageData, imageLen
    );

    g_monitorEventUploadFlag = 0; // 上传一次后清零，防止重复
}

void upload_drive() {
    uint32_t now = millis();
    uploadRealtimeDataIfNeeded(now);
    uploadMonitorEventIfNeeded();
}