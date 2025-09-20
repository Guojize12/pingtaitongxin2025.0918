#include "upload_manager.h"
#include "config.h"
#include "platform_packet.h"
#include "comm_manager.h"
#include "rtc_soft.h"
#include <Arduino.h>

// 定时上传的计时器
static uint32_t lastRealtimeUploadMs = 0;

// 事件上传标志（在 main.ino 中定义，这里只声明使用）
extern volatile int g_monitorEventUploadFlag;

static void uploadRealtimeDataIfNeeded(uint32_t now) {
    if (!comm_isConnected()) return;
    if (!rtc_is_valid()) {
        return;
    }
    if (now - lastRealtimeUploadMs < REALTIME_UPLOAD_INTERVAL_MS) return;

    PlatformTime t;
    rtc_now_fields(&t);

    sendRealtimeMonitorData(
        t.year, t.month, t.day, t.hour, t.minute, t.second, // 用2字节year
        0,
        nullptr,
        0
    );

    lastRealtimeUploadMs = now;
}

static void uploadMonitorEventIfNeeded() {
    if (!comm_isConnected()) return;
    if (!rtc_is_valid()) {
        return;
    }
    if (g_monitorEventUploadFlag != 1) return;

    PlatformTime t;
    rtc_now_fields(&t);

    uint8_t triggerCond = 1;
    float realtimeValue = 12.34f;
    float thresholdValue = 10.00f;
    const uint8_t* imageData = nullptr;
    uint32_t imageLen = 0;

    sendMonitorEventUpload(
        t.year, t.month, t.day, t.hour, t.minute, t.second, triggerCond,
        realtimeValue, thresholdValue, imageData, imageLen
    );

    g_monitorEventUploadFlag = 0; // 上传一次后清零
}

void upload_drive() {
    uint32_t now = millis();
    uploadRealtimeDataIfNeeded(now);
    uploadMonitorEventIfNeeded();
}