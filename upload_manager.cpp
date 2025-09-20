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

// ============ 新增：只上报一次开机状态 ============
static bool g_startupReported = false;

// ============ 新增：只模拟一次事件上传 ============
static bool g_testEventUploaded = false;

static bool g_simInfoUploaded = false;

static void uploadSimInfoIfNeeded() {
    if (g_simInfoUploaded) return;
    if (!comm_isConnected()) return;
    if (!rtc_is_valid()) return;

    SimInfo sim;
    if (!siminfo_query(&sim)) return;

    PlatformTime t;
    rtc_now_fields(&t);

    sendSimInfoUpload(
        t.year, t.month, t.day, t.hour, t.minute, t.second,
        sim.iccid, sim.iccid_len,
        sim.imsi,  sim.imsi_len,
        sim.signal
    );
    g_simInfoUploaded = true;
}


// 开机自动上报（只上报一次，校时+联网后）
static void uploadStartupStatusIfNeeded() {
    if (g_startupReported) return;
    if (!comm_isConnected()) return;
    if (!rtc_is_valid()) return;

    PlatformTime t;
    rtc_now_fields(&t);

    // CMD=0x0002开机状态上报，只支持新37字节协议
    sendStartupStatusReport(
        t.year, t.month, t.day, t.hour, t.minute, t.second,
        0 // 默认状态字0
    );
    g_startupReported = true;

    // ====== 新增：上报一次监测事件，带测试图片 ======
    if (!g_testEventUploaded) {
        uint8_t testImage[10] = {0xDE,0xAD,0xBE,0xEF,0x01,0x23,0x45,0x67,0x89,0xAB}; // 随机内容
        uint8_t triggerCond = 1;       // 进水报警
        float realtimeValue = 12.34f;  // 实时值
        float thresholdValue = 10.00f; // 阈值

        sendMonitorEventUpload(
            t.year, t.month, t.day, t.hour, t.minute, t.second,
            triggerCond,
            realtimeValue,
            thresholdValue,
            testImage,
            sizeof(testImage)
        );
        g_testEventUploaded = true;
    }
}

// 其余代码不变
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
    uploadStartupStatusIfNeeded();     // 开机状态上报
    uploadSimInfoIfNeeded();   // SIM卡状态上报
    uploadRealtimeDataIfNeeded(now);
    uploadMonitorEventIfNeeded();
}