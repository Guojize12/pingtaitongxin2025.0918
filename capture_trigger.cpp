#include "capture_trigger.h"
#include "flash_module.h"
#include "camera_module.h"
#include "sdcard_module.h"
#include "rtc_soft.h"
#include "config.h"
#include <string.h>

extern volatile int g_monitorEventUploadFlag;
char g_lastPhotoName[64] = {0};

static inline bool is_dark_jpeg(size_t jpeg_len) {
    const size_t TH = JPEG_LEN_DARK_THRESH; // 默认16000
    return jpeg_len > 0 && jpeg_len < TH;
}

static void apply_lowlight_boost(bool enable) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;
    if (enable) {
        s->set_aec2(s, 1);
        s->set_ae_level(s, 2);
        s->set_gain_ctrl(s, 1);
        s->set_gainceiling(s, (gainceiling_t)4);
        s->set_brightness(s, 1);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
    } else {
        s->set_ae_level(s, 0);
        s->set_gainceiling(s, (gainceiling_t)3);
        s->set_brightness(s, 1);
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

uint8_t capture_once_internal(uint8_t trigger) {
    if (!camera_ok) return CR_CAMERA_NOT_READY;
    warmup_with_flash_and_discard();
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { flashOff(); return CR_FRAME_GRAB_FAIL; }
    bool need_fallback = is_dark_jpeg(fb->len);
    bool ok = false;
    if (!need_fallback) {
        ok = save_frame_to_sd(fb, 0);
        flashOff();
        esp_camera_fb_return(fb);
        return ok ? CR_OK : CR_SD_SAVE_FAIL;
    }
    esp_camera_fb_return(fb);
    flashOff();
    apply_lowlight_boost(true);
    discard_frames(3);
    camera_fb_t *fb2 = esp_camera_fb_get();
    if (!fb2) {
        apply_lowlight_boost(false);
        return CR_FRAME_GRAB_FAIL;
    }
    ok = save_frame_to_sd(fb2, 0);
    esp_camera_fb_return(fb2);
    apply_lowlight_boost(false);
    return ok ? CR_OK : CR_SD_SAVE_FAIL;
}

bool capture_and_process(uint8_t trigger, bool upload, uint8_t eventType) {
    if (!camera_ok) return false;
    warmup_with_flash_and_discard();
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { flashOff(); return false; }
    bool need_fallback = is_dark_jpeg(fb->len);
    bool sdOk = false;
    if (!need_fallback) {
        char photoFile[64] = {0};
        sdOk = save_frame_to_sd_with_name(fb, photoFile, sizeof(photoFile));
        flashOff();
        if (upload && sdOk) {
            strncpy(g_lastPhotoName, photoFile, sizeof(g_lastPhotoName) - 1);
            g_lastPhotoName[sizeof(g_lastPhotoName)-1] = '\0';
            g_monitorEventUploadFlag = eventType; // 事件类型由参数决定
        }
        esp_camera_fb_return(fb);
        return sdOk;
    }
    esp_camera_fb_return(fb);
    flashOff();
    apply_lowlight_boost(true);
    discard_frames(3);
    camera_fb_t *fb2 = esp_camera_fb_get();
    if (!fb2) {
        apply_lowlight_boost(false);
        return false;
    }
    char photoFile2[64] = {0};
    sdOk = save_frame_to_sd_with_name(fb2, photoFile2, sizeof(photoFile2));
    esp_camera_fb_return(fb2);
    apply_lowlight_boost(false);
    if (upload && sdOk) {
        strncpy(g_lastPhotoName, photoFile2, sizeof(g_lastPhotoName) - 1);
        g_lastPhotoName[sizeof(g_lastPhotoName)-1] = '\0';
        g_monitorEventUploadFlag = eventType;
    }
    return sdOk;
}

void load_params_from_nvs() {
    // TODO: 实现从NVS读取参数的逻辑。暂时空实现防止链接错误。
}