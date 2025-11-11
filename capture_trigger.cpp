#include "capture_trigger.h"
#include "flash_module.h"
#include "camera_module.h"
#include "sdcard_module.h"
#include "rtc_soft.h"
#include "config.h"
#include <string.h>

// 来自 main.ino 的事件上传标志
extern volatile int g_monitorEventUploadFlag;

// 全局保存最后一张照片的文件名（上传用）
char g_lastPhotoName[64] = {0};

// 运行统计（在 main.ino 中定义）
extern RunStats g_stats;

// 依据JPEG长度的“暗场”近似判定（补光被遮挡/光照很暗时，JPEG更小）
static inline bool is_dark_jpeg(size_t jpeg_len) {
    const size_t TH = JPEG_LEN_DARK_THRESH; // 默认16000
    return jpeg_len > 0 && jpeg_len < TH;
}

// 低照度短时提升：温和处理，避免误加亮导致更白
static void apply_lowlight_boost(bool enable) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;
    if (enable) {
        s->set_aec2(s, 1);
        s->set_ae_level(s, 1);                    // 曝光偏置温和提升
        s->set_gain_ctrl(s, 1);
        s->set_gainceiling(s, (gainceiling_t)3);  // 放宽增益但不过高
        s->set_brightness(s, 0);                  // 不额外提高亮度
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
    } else {
        s->set_aec2(s, 0);
        s->set_ae_level(s, -1);
        s->set_gain_ctrl(s, 1);
        s->set_gainceiling(s, (gainceiling_t)2);
        s->set_brightness(s, 0);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
    }
}

static void warmup_with_flash_and_discard() {
    flashOn();
    delay(FLASH_WARM_MS);
    if (DISCARD_FRAMES_EACH_SHOT > 0) {
        discard_frames(DISCARD_FRAMES_EACH_SHOT);
    }
}

// 在检测到连续运行时失败后做轻量恢复（只做 deinit + schedule backoff）
// 不在这里直接重启整个设备，避免把重启放到捕获逻辑里
static void handle_runtime_capture_failure() {
    // 标记摄像头不可用并安排退避重试
    camera_ok = false;
    deinit_camera_silent();
    schedule_camera_backoff();
}

uint8_t capture_once_internal(uint8_t trigger) {
    if (!camera_ok) return CR_CAMERA_NOT_READY;
    warmup_with_flash_and_discard();
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        // 抓帧失败计数
        g_stats.consecutive_capture_fail++;
        if (g_stats.consecutive_capture_fail >= RUNTIME_FAIL_REINIT_THRESHOLD) {
            g_stats.consecutive_capture_fail = 0;
            handle_runtime_capture_failure();
        }
        flashOff(); return CR_FRAME_GRAB_FAIL;
    }
    bool need_fallback = is_dark_jpeg(fb->len);
    bool ok = false;
    if (!need_fallback) {
        ok = save_frame_to_sd(fb, 0);
        flashOff();
        esp_camera_fb_return(fb);
        if (!ok) {
            g_stats.consecutive_sd_fail++;
            if (g_stats.consecutive_sd_fail >= SD_FAIL_REBOOT_THRESHOLD) {
                g_stats.consecutive_sd_fail = 0;
                // 若 SD 连续失败很多次，安排更强的恢复（此处只做退避）
                handle_runtime_capture_failure();
            }
        } else {
            g_stats.consecutive_capture_fail = 0;
            g_stats.consecutive_sd_fail = 0;
            g_stats.total_captures++;
            g_stats.last_frame_size = 0; // 无具体大小路径时可不更新
            g_stats.last_capture_ms = millis();
        }
        return ok ? CR_OK : CR_SD_SAVE_FAIL;
    }
    esp_camera_fb_return(fb);
    flashOff();
    apply_lowlight_boost(true);
    discard_frames(3);
    camera_fb_t *fb2 = esp_camera_fb_get();
    if (!fb2) {
        apply_lowlight_boost(false);
        g_stats.consecutive_capture_fail++;
        if (g_stats.consecutive_capture_fail >= RUNTIME_FAIL_REINIT_THRESHOLD) {
            g_stats.consecutive_capture_fail = 0;
            handle_runtime_capture_failure();
        }
        return CR_FRAME_GRAB_FAIL;
    }
    ok = save_frame_to_sd(fb2, 0);
    esp_camera_fb_return(fb2);
    apply_lowlight_boost(false);
    if (!ok) {
        g_stats.consecutive_sd_fail++;
        if (g_stats.consecutive_sd_fail >= SD_FAIL_REBOOT_THRESHOLD) {
            g_stats.consecutive_sd_fail = 0;
            handle_runtime_capture_failure();
        }
        return CR_SD_SAVE_FAIL;
    }
    // 成功情况，清零连续失败计数
    g_stats.consecutive_capture_fail = 0;
    g_stats.consecutive_sd_fail = 0;
    g_stats.total_captures++;
    g_stats.last_frame_size = fb2 ? fb2->len : 0;
    g_stats.last_capture_ms = millis();
    return CR_OK;
}

// 拍照保存与上传解耦：保存到SD并记录文件名，上传由 upload_manager 触发
bool capture_and_process(uint8_t trigger, bool upload) {
    if (!camera_ok) return false;
    warmup_with_flash_and_discard();
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        // 抓帧失败
        g_stats.consecutive_capture_fail++;
        if (g_stats.consecutive_capture_fail >= RUNTIME_FAIL_REINIT_THRESHOLD) {
            g_stats.consecutive_capture_fail = 0;
            handle_runtime_capture_failure();
        }
        flashOff();
        return false;
    }
    bool need_fallback = is_dark_jpeg(fb->len);
    bool sdOk = false;
    if (!need_fallback) {
        char photoFile[64] = {0};
        sdOk = save_frame_to_sd_with_name(fb, photoFile, sizeof(photoFile));
        flashOff();
        if (upload && sdOk) {
            strncpy(g_lastPhotoName, photoFile, sizeof(g_lastPhotoName) - 1);
            g_lastPhotoName[sizeof(g_lastPhotoName)-1] = '\0';
            g_monitorEventUploadFlag = 1;
        }
        esp_camera_fb_return(fb);

        if (!sdOk) {
            g_stats.consecutive_sd_fail++;
            if (g_stats.consecutive_sd_fail >= SD_FAIL_REBOOT_THRESHOLD) {
                g_stats.consecutive_sd_fail = 0;
                handle_runtime_capture_failure();
            }
        } else {
            g_stats.consecutive_capture_fail = 0;
            g_stats.consecutive_sd_fail = 0;
            g_stats.total_captures++;
            g_stats.last_frame_size = fb->len;
            g_stats.last_capture_ms = millis();
        }
        return sdOk;
    }
    esp_camera_fb_return(fb);
    flashOff();
    apply_lowlight_boost(true);
    discard_frames(3);
    camera_fb_t *fb2 = esp_camera_fb_get();
    if (!fb2) {
        apply_lowlight_boost(false);
        g_stats.consecutive_capture_fail++;
        if (g_stats.consecutive_capture_fail >= RUNTIME_FAIL_REINIT_THRESHOLD) {
            g_stats.consecutive_capture_fail = 0;
            handle_runtime_capture_failure();
        }
        return false;
    }
    char photoFile2[64] = {0};
    sdOk = save_frame_to_sd_with_name(fb2, photoFile2, sizeof(photoFile2));
    esp_camera_fb_return(fb2);
    apply_lowlight_boost(false);
    if (upload && sdOk) {
        strncpy(g_lastPhotoName, photoFile2, sizeof(g_lastPhotoName) - 1);
        g_lastPhotoName[sizeof(g_lastPhotoName)-1] = '\0';
        g_monitorEventUploadFlag = 1;
    }

    if (!sdOk) {
        g_stats.consecutive_sd_fail++;
        if (g_stats.consecutive_sd_fail >= SD_FAIL_REBOOT_THRESHOLD) {
            g_stats.consecutive_sd_fail = 0;
            handle_runtime_capture_failure();
        }
        return false;
    }

    // 成功
    g_stats.consecutive_capture_fail = 0;
    g_stats.consecutive_sd_fail = 0;
    g_stats.total_captures++;
    g_stats.last_frame_size = fb2 ? fb2->len : 0;
    g_stats.last_capture_ms = millis();
    return sdOk;
}

void load_params_from_nvs() {
    // TODO: 实现从NVS读取参数的逻辑。暂时空实现防止链接错误。
}