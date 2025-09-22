#include "capture_trigger.h"
#include "flash_module.h"
#include "camera_module.h"
#include "sdcard_module.h"
#include "upgrade.h"
#include "logging.h"
#include "config.h"

extern RunStats g_stats;
extern RuntimeConfig g_cfg;
extern Preferences prefs;
extern uint32_t photo_index;
extern uint8_t g_flashDuty;
extern uint32_t last_params_saved_ms;
extern void platform_send_image(uint8_t trigger, const uint8_t* jpeg, uint32_t jpegLen, uint32_t frameIndex);

uint8_t capture_once_internal(uint8_t trigger) {
#if UPGRADE_ENABLE
    if (g_upg.state == UPG_DOWNLOADING || g_upg.state == UPG_FILE_INFO) {
        if (g_debugMode) LOG_WARN("[CAP] blocked by upgrade"); return CR_CAMERA_NOT_READY;
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

    if (g_cfg.sendEnabled && g_cfg.sendBeforeSave)
        platform_send_image(trigger, fb->buf, fb->len, index);
    if (g_cfg.saveEnabled) sdOk = save_frame_to_sd(fb, index);
    if (g_cfg.sendEnabled && !g_cfg.sendBeforeSave)
        platform_send_image(trigger, fb->buf, fb->len, index);

    if (sdOk || !g_cfg.saveEnabled) {
        photo_index++;
        if (photo_index % SAVE_PARAMS_INTERVAL_IMAGES == 0) save_params_to_nvs();
        else if (millis() - last_params_saved_ms >= NVS_MIN_SAVE_INTERVAL_MS) save_params_to_nvs();
    }
    esp_camera_fb_return(fb);
    flashOff();

    g_stats.total_captures++;
    g_stats.last_frame_size = frame_len;
    g_stats.last_capture_ms = millis();
    if (!sdOk && g_cfg.saveEnabled) return CR_SD_SAVE_FAIL;
    // check_and_reboot_on_low_heap(); // 保持原有主流程调用
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

// NVS参数存取
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