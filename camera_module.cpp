#include "camera_module.h"
#include "logging.h"
#include "config.h"

bool camera_ok = false;
static uint32_t camera_reinit_backoff_ms = 0;
static uint32_t camera_next_reinit_allowed = 0;
static const uint32_t CAMERA_BACKOFF_BASE = 3000;
static const uint32_t CAMERA_BACKOFF_MAX = 30000;

camera_config_t make_config(framesize_t size, int xclk, int q) {
    camera_config_t c;
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer = LEDC_TIMER_0;
    c.pin_d0 = Y2_GPIO; c.pin_d1 = Y3_GPIO; c.pin_d2 = Y4_GPIO; c.pin_d3 = Y5_GPIO;
    c.pin_d4 = Y6_GPIO; c.pin_d5 = Y7_GPIO; c.pin_d6 = Y8_GPIO; c.pin_d7 = Y9_GPIO;
    c.pin_xclk = XCLK_GPIO; c.pin_pclk = PCLK_GPIO; c.pin_vsync = VSYNC_GPIO;
    c.pin_href = HREF_GPIO; c.pin_sscb_sda = SIOD_GPIO; c.pin_sscb_scl = SIOC_GPIO;
    c.pin_pwdn = PWDN_GPIO; c.pin_reset = RESET_GPIO;
    c.xclk_freq_hz = xclk;
    c.pixel_format = PIXFORMAT_JPEG;
    if (psramFound()) {
        c.frame_size = size; c.jpeg_quality = q; c.fb_count = 1;
    } else {
        c.frame_size = (size > FRAMESIZE_VGA ? FRAMESIZE_VGA : size);
        c.jpeg_quality = q + 5; c.fb_count = 1;
    }
    return c;
}

bool try_camera_init_once(framesize_t size, int xclk, int q) {
    camera_config_t cfg = make_config(size, xclk, q);
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) LOG_WARN("[CAM] init err=0x%X", (unsigned)err);
    return err == ESP_OK;
}

bool init_camera_multi() {
    pinMode(PWDN_GPIO, OUTPUT); digitalWrite(PWDN_GPIO, LOW); delay(30);
    for (int i = 0; i < INIT_RETRY_PER_CONFIG; i++) {
        if (try_camera_init_once(FRAME_SIZE_PREF, 20000000, JPEG_QUALITY_PREF)) return true;
        delay(120);
    }
    esp_camera_deinit(); delay(60);
    for (int i = 0; i < INIT_RETRY_PER_CONFIG; i++) {
        if (try_camera_init_once(FRAME_SIZE_FALLBACK, 10000000, JPEG_QUALITY_FALLBACK)) return true;
        delay(150);
    }
    esp_camera_deinit();
    return false;
}

void deinit_camera_silent() { esp_camera_deinit(); delay(50); }

bool discard_frames(int n) {
    for (int i = 0; i < n; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) return false;
        esp_camera_fb_return(fb);
    }
    return true;
}

bool reinit_camera_with_params(framesize_t size, int quality) {
    deinit_camera_silent();
    camera_config_t cfg = make_config(size, 20000000, quality);
    if (esp_camera_init(&cfg) == ESP_OK) {
        camera_ok = true; LOG_INFO("[CAM] reinit fast OK size=%d q=%d", (int)size, quality); return true;
    }
    deinit_camera_silent();
    cfg = make_config(size, 10000000, quality + 2);
    if (esp_camera_init(&cfg) == ESP_OK) {
        camera_ok = true; LOG_INFO("[CAM] reinit slow OK size=%d q=%d", (int)size, quality + 2); return true;
    }
    camera_ok = false; LOG_WARN("[CAM] reinit fail size=%d q=%d", (int)size, quality); return false;
}

void schedule_camera_backoff() {
    camera_reinit_backoff_ms = camera_reinit_backoff_ms ? min<uint32_t>(camera_reinit_backoff_ms * 2, CAMERA_BACKOFF_MAX) : CAMERA_BACKOFF_BASE;
    camera_next_reinit_allowed = millis() + camera_reinit_backoff_ms;
}
void attempt_camera_reinit_with_backoff() {
    uint32_t now = millis();
    if (now < camera_next_reinit_allowed) return;
    deinit_camera_silent();
    camera_ok = init_camera_multi();
    if (!camera_ok) schedule_camera_backoff(); else camera_reinit_backoff_ms = 0;
}