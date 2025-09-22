#include "capture_trigger.h"
#include "flash_module.h"
#include "camera_module.h"
#include "sdcard_module.h"
#include "platform_packet.h"
#include "rtc_soft.h"

uint8_t capture_once_internal(uint8_t trigger) {
    if (!camera_ok) return CR_CAMERA_NOT_READY;
    flashOn();
    delay(60);
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { flashOff(); return CR_FRAME_GRAB_FAIL; }
    bool sdOk = save_frame_to_sd(fb, 0); // 固定索引
    flashOff();
    esp_camera_fb_return(fb);
    return (sdOk ? CR_OK : CR_SD_SAVE_FAIL);
}

// 新增：可控制是否上传
bool capture_and_process(uint8_t trigger, bool upload) {
    if (!camera_ok) return false;
    flashOn();
    delay(60);
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { flashOff(); return false; }
    bool sdOk = save_frame_to_sd(fb, 0); // 固定索引
    flashOff();

    if (upload && sdOk) {
        // 上传照片，使用fb->buf, fb->len
        PlatformTime t;
        rtc_now_fields(&t);
        uint8_t triggerCond = 1; // 可根据实际条件设置
        float realtimeValue = 0;
        float thresholdValue = 0;
        sendMonitorEventUpload(
            t.year, t.month, t.day, t.hour, t.minute, t.second,
            triggerCond, realtimeValue, thresholdValue,
            fb->buf, fb->len
        );
    }

    esp_camera_fb_return(fb);
    return sdOk;
}

void load_params_from_nvs() {
    // TODO: 实现从NVS读取参数的逻辑。暂时空实现防止链接错误。
}