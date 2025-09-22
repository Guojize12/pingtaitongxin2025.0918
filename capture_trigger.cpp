#include "capture_trigger.h"
#include "flash_module.h"
#include "camera_module.h"
#include "sdcard_module.h"

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

bool capture_and_process(uint8_t trigger) {
    return capture_once_internal(trigger) == CR_OK;
}