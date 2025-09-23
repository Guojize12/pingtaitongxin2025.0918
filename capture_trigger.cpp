#include "capture_trigger.h"
#include "flash_module.h"
#include "camera_module.h"
#include "sdcard_module.h"
#include "platform_packet.h"
#include "rtc_soft.h"
#include "config.h"

extern uint32_t photo_index; // 来自 main.ino

// 更健壮的单次拍照过程（保留本地保存与闪光逻辑，上传逻辑由上层控制）
uint8_t capture_once_internal(uint8_t /*trigger*/) {
    if (!camera_ok) return CR_CAMERA_NOT_READY;

    // 每次拍照前可丢弃若干帧，稳定曝光
    if (DISCARD_FRAMES_EACH_SHOT > 0) {
        discard_frames(DISCARD_FRAMES_EACH_SHOT);
    }

    flashOn();
#if FLASH_MODE
    delay(FLASH_WARM_MS);
#else
    delay(FLASH_ON_TIME_MS_DIGITAL);
#endif

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        delay(20);
        fb = esp_camera_fb_get();
        if (!fb) {
            flashOff();
            return CR_FRAME_GRAB_FAIL;
        }
    }

    uint32_t index = photo_index;
    bool sdOk = true;

    if (g_cfg.saveEnabled) {
        sdOk = save_frame_to_sd(fb, index);
    }

    if (sdOk || !g_cfg.saveEnabled) {
        photo_index++;
    }

    // 注意：上传时需要用到 fb->buf/fb->len，因此上层在上传完成前不能提前释放
    // 在本函数内不做上传，只负责拍与存

    flashOff();

    if (!g_cfg.saveEnabled || sdOk) {
        // 正常
    } else {
        // SD保存失败
        esp_camera_fb_return(fb);
        return CR_SD_SAVE_FAIL;
    }

    // 将 fb 的释放交给上层（上传后再释放）。
    // 但为了与现有接口兼容，这里不持有 fb 指针；上层立即使用并释放。
    // 返回 OK，让上层继续流程。
    esp_camera_fb_return(fb);
    return CR_OK;
}

// 兼容原项目：可选择上传
bool capture_and_process(uint8_t trigger, bool upload) {
    if (!camera_ok) return false;

    // 拍照与保存
    flashOn();
#if FLASH_MODE
    delay(FLASH_WARM_MS);
#else
    delay(FLASH_ON_TIME_MS_DIGITAL);
#endif
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        delay(20);
        fb = esp_camera_fb_get();
        if (!fb) {
            flashOff();
            // 失败则尝试按策略重初始化相机
            if (ENABLE_AUTO_REINIT) {
                attempt_camera_reinit_with_backoff();
            }
            return false;
        }
    }

    bool sdOk = true;
    uint32_t index = photo_index;
    if (g_cfg.saveEnabled) {
        sdOk = save_frame_to_sd(fb, index);
    }
    if (sdOk || !g_cfg.saveEnabled) {
        photo_index++;
    }

    // 可选上传：保持原来的上传协议逻辑不变
    if (upload && sdOk) {
        PlatformTime t;
        rtc_now_fields(&t);
        uint8_t triggerCond = 1;
        float realtimeValue = 0;
        float thresholdValue = 0;
        sendMonitorEventUpload(
            t.year, t.month, t.day, t.hour, t.minute, t.second,
            triggerCond, realtimeValue, thresholdValue,
            fb->buf, fb->len
        );
    }

    esp_camera_fb_return(fb);
    flashOff();

    if (!sdOk && g_cfg.saveEnabled) {
        // 保存失败
        return false;
    }

    return true;
}

void load_params_from_nvs() {
    // TODO: 实现从NVS读取参数的逻辑。暂时空实现防止链接错误。
}

void wait_button_release_on_boot(){
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    if (digitalRead(BUTTON_PIN) == LOW) {
        while (digitalRead(BUTTON_PIN) == LOW) delay(10);
    }
}