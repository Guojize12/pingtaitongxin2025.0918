#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "esp_camera.h"

uint8_t capture_once_internal(uint8_t trigger);
bool capture_and_process(uint8_t trigger);
void wait_button_release_on_boot();
void save_params_to_nvs();
void load_params_from_nvs();
// 可选：#define TRIGGER_BUTTON 1 (config.h 已定义)