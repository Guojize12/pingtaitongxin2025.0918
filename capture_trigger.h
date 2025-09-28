#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "esp_camera.h"

// 最后一张照片的文件名（供上传用）
extern char g_lastPhotoName[64];

// 拍照接口：增加事件类型参数
bool capture_and_process(uint8_t trigger, bool upload, uint8_t eventType);

// 兼容旧接口（默认事件类型1：进水报警）
inline bool capture_and_process(uint8_t trigger, bool upload) { return capture_and_process(trigger, upload, 1); }

uint8_t capture_once_internal(uint8_t trigger);

void wait_button_release_on_boot();
void save_params_to_nvs();
void load_params_from_nvs();