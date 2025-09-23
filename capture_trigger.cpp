#include "capture_trigger.h"
#include "flash_module.h"
#include "camera_module.h"
#include "sdcard_module.h"
#include "rtc_soft.h"

// 来自 main.ino 的事件上传标志
extern volatile int g_monitorEventUploadFlag;

uint8_t capture_once_internal(uint8_t trigger) {
    if (!camera_ok) return CR_CAMERA_NOT_READY;
    flashOn();
    delay(60);
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { flashOff(); return CR_FRAME_GRAB_FAIL; }
    bool sdOk = save_frame_to_sd(fb, 0); // 固定索引（内部已生成唯一文件名）
    flashOff();
    esp_camera_fb_return(fb);
    return (sdOk ? CR_OK : CR_SD_SAVE_FAIL);
}

// 新增：拍照保存与上传解耦（上传仅置标志，真正发送由 upload_manager 在联网时完成）
bool capture_and_process(uint8_t trigger, bool upload) {
    if (!camera_ok) return false;

    flashOn();
    delay(60);
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { flashOff(); return false; }

    // 异步优先的SD保存（内部若队列满会回退同步写）
    bool sdOk = save_frame_to_sd(fb, 0);
    flashOff();

    // 无论是否联网，拍照成功则请求一次事件上报（仅元数据，不夹带大图）
    // 这样断网也不会丢：标志会保留，联网后由 upload_manager 发送
    if (upload && sdOk) {
        g_monitorEventUploadFlag = 1;
    }

    esp_camera_fb_return(fb);
    return sdOk;
}

void load_params_from_nvs() {
    // TODO: 实现从NVS读取参数的逻辑。暂时空实现防止链接错误。
}