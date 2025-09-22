#include "capture_trigger.h"
#include "flash_module.h"
#include "camera_module.h"
#include "sdcard_module.h"
#include "platform_packet.h" // 新增，上传接口
#include "config.h"

extern RunStats g_stats;
extern RuntimeConfig g_cfg;
extern Preferences prefs;
extern uint32_t photo_index;
extern uint8_t g_flashDuty;
extern uint32_t last_params_saved_ms;

// #define TRIGGER_BUTTON 1 // 可以选择这里定义，也可以在 config.h

uint8_t capture_once_internal(uint8_t trigger) {
#if UPGRADE_ENABLE
    if (g_upg.state == UPG_DOWNLOADING || g_upg.state == UPG_FILE_INFO) {
        if (g_debugMode) /*日志已移除*/; return CR_CAMERA_NOT_READY;
    }
#endif
    if (!camera_ok) return CR_CAMERA_NOT_READY;
    if (DISCARD_FRAMES_EACH_SHOT > 0) discard_frames(DISCARD_FRAMES_EACH_SHOT);
    flashOn();
#if FLASH_MODE
    delay(FLASH_WARM_MS);
#else
    delay(FLASH_ON_TIME_MS_DIGITAL);
#endif
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        delay(20); fb = esp_camera_fb_get();
        if (!fb) { flashOff(); return CR_FRAME_GRAB_FAIL; }
    }
    uint32_t frame_len = fb->len;
    uint32_t index = photo_index;
    bool sdOk = true;

    // 拍照完成后本地保存
    if (g_cfg.saveEnabled) sdOk = save_frame_to_sd(fb, index);

    if (sdOk || !g_cfg.saveEnabled) {
        photo_index++;
        if (photo_index % SAVE_PARAMS_INTERVAL_IMAGES == 0) save_params_to_nvs();
        else if (millis() - last_params_saved_ms >= NVS_MIN_SAVE_INTERVAL_MS) save_params_to_nvs();
    }
    flashOff();

    // 新增：拍照成功后上传图片
    if (g_cfg.sendEnabled) {
        // 获取当前时间
        PlatformTime t;
        rtc_now_fields(&t);
        // 触发条件为按钮
        uint8_t triggerCond = trigger;
        float realtimeValue = 0.0f;
        float thresholdValue = 0.0f;
        sendMonitorEventUpload(
            t.year, t.month, t.day, t.hour, t.minute, t.second,
            triggerCond,
            realtimeValue,
            thresholdValue,
            fb->buf, fb->len
        );
    }

    esp_camera_fb_return(fb);

    g_stats.total_captures++;
    g_stats.last_frame_size = frame_len;
    g_stats.last_capture_ms = millis();
    if (!sdOk && g_cfg.saveEnabled) return CR_SD_SAVE_FAIL;
    return CR_OK;
}

bool capture_and_process(uint8_t trigger) {
    uint8_t r = capture_once_internal(trigger);
    if (r != CR_OK) {
        if (r == CR_FRAME_GRAB_FAIL) handle_camera_failure();
        else if (r == CR_SD_SAVE_FAIL) handle_sd_failure();
        else if (r == CR_CAMERA_NOT_READY && camera_ok == false) attempt_camera_reinit_with_backoff();
        return false;
    } else {
        g_stats.consecutive_capture_fail = 0;
        g_stats.consecutive_sd_fail = 0;
    }
    return true;
}

void wait_button_release_on_boot() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    if (digitalRead(BUTTON_PIN) == LOW)
        while (digitalRead(BUTTON_PIN) == LOW) delay(10);
}

void save_params_to_nvs() {
    prefs.putUChar("flashduty", g_flashDuty);
    prefs.putUInt("photo_idx", photo_index);
    last_params_saved_ms = millis();
}
void load_params_from_nvs() {
    g_flashDuty = prefs.getUChar("flashduty", DEFAULT_FLASH_DUTY);
    photo_index = prefs.getUInt("photo_idx", 1);
    if (photo_index == 0) photo_index = 1;
    last_params_saved_ms = millis();
}