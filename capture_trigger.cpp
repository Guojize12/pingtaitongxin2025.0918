#include "capture_trigger.h"
#include "flash_module.h"
#include "camera_module.h"
#include "sdcard_module.h"
#include "rtc_soft.h"
#include <string.h>

// 来自 main.ino 的事件上传标志
extern volatile int g_monitorEventUploadFlag;

// 全局保存最后一张照片的文件名（上传用）
char g_lastPhotoName[64] = {0};

uint8_t capture_once_internal(uint8_t trigger) {
    if (!camera_ok) return CR_CAMERA_NOT_READY;
    flashOn();
    delay(60);
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { flashOff(); return CR_FRAME_GRAB_FAIL; }
    bool sdOk = save_frame_to_sd(fb, 0); // 仅保存，不返回文件名
    flashOff();
    esp_camera_fb_return(fb);
    return (sdOk ? CR_OK : CR_SD_SAVE_FAIL);
}

// 拍照保存与上传解耦：保存到SD并记录文件名，上传由 upload_manager 触发
bool capture_and_process(uint8_t trigger, bool upload) {
    if (!camera_ok) return false;

    flashOn();
    delay(60);
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { flashOff(); return false; }

    char photoFile[64] = {0};
    bool sdOk = save_frame_to_sd_with_name(fb, photoFile, sizeof(photoFile));
    flashOff();

    if (upload && sdOk) {
        strncpy(g_lastPhotoName, photoFile, sizeof(g_lastPhotoName) - 1);
        g_lastPhotoName[sizeof(g_lastPhotoName)-1] = '\0';
        g_monitorEventUploadFlag = 1; // 通知上传管理器
    }

    esp_camera_fb_return(fb);
    return sdOk;
}

void load_params_from_nvs() {
    // TODO: 实现从NVS读取参数的逻辑。暂时空实现防止链接错误。
}