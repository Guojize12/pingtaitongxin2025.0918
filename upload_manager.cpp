#include "upload_manager.h"
#include "config.h"
#include "platform_packet.h"
#include "comm_manager.h"
#include "rtc_soft.h"
#include "sd_async.h"
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include "camera_module.h"
#include "capture_trigger.h"

// 定时上传的计时器
static uint32_t lastRealtimeUploadMs = 0;

// 事件上传标志（在 main.ino 中定义，这里只声明使用）
extern volatile int g_monitorEventUploadFlag;

// 最后一张照片文件名（由 capture_trigger 维护）
extern char g_lastPhotoName[64];

// 新增：来自 main.ino 的水浸报警全局变量
extern volatile uint8_t g_waterSensorStatus;

// ============ 新增：只上报一次开机状态 ============
static bool g_startupReported = false;

// 新增：只上传一次“上电拍照事件”
static bool g_startupPhotoUploaded = false;

static bool g_simInfoUploaded = false;

// 进水拍照上传定时器
static uint32_t lastWaterPhotoUploadMs = 0;
static bool lastWaterActive = false;

// 进水状态下定时拍照上传
static void water_auto_capture_upload_if_needed(uint32_t now) {
    if (g_waterSensorStatus == 1) {
        if (!lastWaterActive || now - lastWaterPhotoUploadMs >= 600000UL) { // 10分钟
            bool ok = capture_and_process(TRIGGER_BUTTON, true, 1); // 1: 进水报警事件
            // 即使拍照失败也更新时间戳，避免死循环
            lastWaterPhotoUploadMs = now;
        }
        lastWaterActive = true;
    } else {
        // 离开进水状态，重置
        lastWaterPhotoUploadMs = now;
        lastWaterActive = false;
    }
}

static void uploadSimInfoIfNeeded() {
    if (g_simInfoUploaded) return;
    if (!comm_isConnected()) {
        return;
    }
    if (!rtc_is_valid()) {
        log2("[SIMUP] RTC not valid, skip.");
        return;
    }

    log2("[SIMUP] Try collect SIM info");
    SimInfo sim;
    if (!siminfo_query(&sim)) {
        log2("[SIMUP] SIM info collect fail, skip upload.");
        return;
    }

    PlatformTime t;
    rtc_now_fields(&t);

    log2Str("[SIMUP] Ready upload ICCID: ", sim.iccid);
    log2Str("[SIMUP] Ready upload IMSI: ", sim.imsi);

    sendSimInfoUpload(
        t.year, t.month, t.day, t.hour, t.minute, t.second,
        sim.iccid, sim.iccid_len,
        sim.imsi,  sim.imsi_len,
        sim.signal
    );
    log2("[SIMUP] SIM info upload sent.");
    g_simInfoUploaded = true;
}

// 开机自动上报（只上报一次，校时+联网后）
static void uploadStartupStatusIfNeeded() {
    if (g_startupReported) return;
    if (!comm_isConnected()) return;
    if (!rtc_is_valid()) return;

    PlatformTime t;
    rtc_now_fields(&t);

    // CMD=0x0002开机状态上报
    sendStartupStatusReport(
        t.year, t.month, t.day, t.hour, t.minute, t.second,
        0 // 默认状态字0
    );
    g_startupReported = true;
}

// 新增：联通并校时后，拍一次“上电拍照”并作为事件上传（触发条件=0）
static void uploadStartupPhotoIfNeeded() {
    if (g_startupPhotoUploaded) return;
    if (!comm_isConnected()) return;
    if (!rtc_is_valid()) return;

    bool ok = capture_and_process(TRIGGER_BUTTON, true, 0); // 0: 上电拍照事件
    if (ok) {
        g_startupPhotoUploaded = true;
    }
}

static void uploadRealtimeDataIfNeeded(uint32_t now) {
    if (!comm_isConnected()) return;
    if (!rtc_is_valid()) {
        return;
    }
    if (now - lastRealtimeUploadMs < REALTIME_UPLOAD_INTERVAL_MS) return;

    PlatformTime t;
    rtc_now_fields(&t);

    // 构造异常状态
    uint8_t exceptionStatus = 0;
    if (!camera_ok) exceptionStatus |= 0x01;
    if (g_waterSensorStatus) exceptionStatus |= 0x02;

    uint8_t waterStatus = g_waterSensorStatus ? 1 : 0;

    sendRealtimeMonitorData(
        t.year, t.month, t.day, t.hour, t.minute, t.second,
        0,
        &exceptionStatus,
        waterStatus
    );

    lastRealtimeUploadMs = now;
}

// 将 g_lastPhotoName 指向的文件读取到内存（≤65000），成功返回malloc的指针与长度
static uint8_t* read_photo_into_ram(size_t& outLen) {
    outLen = 0;
    if (!g_lastPhotoName[0]) return nullptr;

    // 如果启用异步写，且还未空闲，则暂缓上传，等下一轮
    if (g_cfg.asyncSDWrite && !sd_async_idle()) {
        return nullptr;
    }

    File f = SD.open(g_lastPhotoName, FILE_READ);
    if (!f) {
        Serial.println("[UPLOAD] Photo file open failed!");
        return nullptr;
    }
    size_t sz = f.size();
    if (sz == 0) {
        f.close();
        Serial.println("[UPLOAD] Photo file size=0!");
        return nullptr;
    }
    if (sz > 65000) {
        f.close();
        Serial.println("[UPLOAD] Photo too large (>65K), skip upload.");
        return nullptr;
    }
    uint8_t* buf = (uint8_t*)malloc(sz);
    if (!buf) {
        f.close();
        Serial.println("[UPLOAD] malloc failed for photo buffer!");
        return nullptr;
    }
    size_t n = f.read(buf, sz);
    f.close();
    if (n != sz) {
        free(buf);
        Serial.println("[UPLOAD] Photo read size mismatch!");
        return nullptr;
    }
    outLen = sz;
    return buf;
}

// 事件上传（支持两类事件：0=上电拍照；1=进水报警）
static void uploadMonitorEventIfNeeded() {
    if (!comm_isConnected()) return;
    if (!rtc_is_valid()) {
        return;
    }

    // 修复：只在 flag 为 0 或 1 时上传；-1 表示无事件
    int flag = g_monitorEventUploadFlag;
    if (flag != 0 && flag != 1) return;

    // 读取图片数据
    size_t imgLen = 0;
    uint8_t* imageData = read_photo_into_ram(imgLen);
    if (!imageData && g_cfg.asyncSDWrite) {
        // 异步未空闲，或读取失败，下一轮再试（不清标志）
        return;
    }

    PlatformTime t;
    rtc_now_fields(&t);

    uint8_t triggerCond = (uint8_t)flag; // 0=上电拍照，1=进水报警
    float realtimeValue = 0.0f;
    float thresholdValue = 0.0f;

    if (imageData && imgLen > 0 && imgLen <= 65000) {
        sendMonitorEventUpload(
            t.year, t.month, t.day, t.hour, t.minute, t.second, triggerCond,
            realtimeValue, thresholdValue, imageData, (uint32_t)imgLen
        );
        free(imageData);
    } else {
        Serial.println("[UPLOAD] No image attached (either too large or not ready). Send meta only.");
        sendMonitorEventUpload(
            t.year, t.month, t.day, t.hour, t.minute, t.second, triggerCond,
            realtimeValue, thresholdValue, nullptr, 0
        );
    }

    // 修复：上传完成后清为“无事件”
    g_monitorEventUploadFlag = -1;
}

void upload_drive() {
    uint32_t now = millis();
    uploadStartupStatusIfNeeded();     // 开机状态上报
    uploadSimInfoIfNeeded();           // SIM卡状态上报
    uploadStartupPhotoIfNeeded();      // 上电拍照事件（触发条件=0）
    uploadRealtimeDataIfNeeded(now);   // 实时数据上报
    uploadMonitorEventIfNeeded();      // 事件图片上传
    water_auto_capture_upload_if_needed(now); // 进水定时拍照上传
}