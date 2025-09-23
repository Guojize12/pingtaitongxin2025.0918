#pragma once
#include <Arduino.h>
#include <SD.h>
#include "esp_camera.h"
#include "config.h"

void init_sd();
void periodic_sd_check();
bool save_frame_to_sd(camera_fb_t *fb, uint32_t index);
bool save_frame_to_sd_raw(const uint8_t* data, size_t len, uint32_t index);