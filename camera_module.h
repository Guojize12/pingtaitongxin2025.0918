#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "config.h"

//bool try_camera_init_once(framesize_t size, int xclk, int q);
bool init_camera_multi();
void deinit_camera_silent();
bool discard_frames(int n);
bool reinit_camera_with_params(framesize_t size, int quality);
void schedule_camera_backoff();
void attempt_camera_reinit_with_backoff();
extern bool camera_ok;