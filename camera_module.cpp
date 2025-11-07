#include "camera_module.h"
#include "config.h"

bool camera_ok = false;
static uint32_t camera_reinit_backoff_ms = 0;
static uint32_t camera_next_reinit_allowed = 0;
static const uint32_t CAMERA_BACKOFF_BASE = 3000;
static const uint32_t CAMERA_BACKOFF_MAX = 30000;

// 统一的传感器参数调优（初始化后调用）
static void tune_camera_sensor_defaults() {
    sensor_t * s = esp_camera_sensor_get();
    if (!s) return;

    // 基础画质（全部归零，避免人为偏亮）
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_sharpness(s, 0);

    // 色彩与白平衡
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);

    // 曝光与增益
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);           // 关闭AEC2，避免过度提亮
    s->set_ae_level(s, -1);      // 曝光偏置压低，避免过曝
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, (gainceiling_t)2); // 增益上限收敛

    // 矫正与缩放
    s->set_lenc(s, 1);
    s->set_dcw(s, 1);
    s->set_raw_gma(s, 1);
}

static camera_config_t make_config(framesize_t size, int xclk_hz, int jpeg_q) {
    camera_config_t c = {};
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer   = LEDC_TIMER_0;
    c.pin_d0 = Y2_GPIO;  c.pin_d1 = Y3_GPIO;  c.pin_d2 = Y4_GPIO;  c.pin_d3 = Y5_GPIO;
    c.pin_d4 = Y6_GPIO;  c.pin_d5 = Y7_GPIO;  c.pin_d6 = Y8_GPIO;  c.pin_d7 = Y9_GPIO;
    c.pin_xclk = XCLK_GPIO;  c.pin_pclk = PCLK_GPIO;  c.pin_vsync = VSYNC_GPIO;
    c.pin_href = HREF_GPIO;  c.pin_sscb_sda = SIOD_GPIO;  c.pin_sscb_scl = SIOC_GPIO;
    c.pin_pwdn = PWDN_GPIO;  c.pin_reset = RESET_GPIO;
    c.xclk_freq_hz = xclk_hz;
    c.pixel_format = PIXFORMAT_JPEG;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    if (psramFound()) {
        c.fb_location = CAMERA_FB_IN_PSRAM;
        c.frame_size  = size;
        c.jpeg_quality = jpeg_q;
        c.fb_count    = 1;
    } else {
        c.fb_location = CAMERA_FB_IN_DRAM;
        if (size > FRAMESIZE_VGA) size = FRAMESIZE_VGA;
        c.frame_size  = size;
        c.jpeg_quality = jpeg_q + 4;
        c.fb_count    = 1;
    }
    return c;
}

static bool try_camera_init_once(framesize_t size, int xclk_hz, int jpeg_q) {
    camera_config_t cfg = make_config(size, xclk_hz, jpeg_q);
    esp_err_t err = esp_camera_init(&cfg);
    if (err == ESP_OK) {
        tune_camera_sensor_defaults();
        return true;
    }
    return false;
}

bool init_camera_multi() {
    camera_ok = false;

    // Ensure PWDN pin is configured and drive a known sequence to avoid uncertain power states
    if (PWDN_GPIO >= 0) {
        pinMode(PWDN_GPIO, OUTPUT);
        // Start with LOW to ensure power-on enabled (some modules active LOW)
        digitalWrite(PWDN_GPIO, LOW);
        delay(50);
    }

    // Existing soft power toggle: try a short sequence to wake module
    if (PWDN_GPIO >= 0) {
        pinMode(PWDN_GPIO, OUTPUT);
        digitalWrite(PWDN_GPIO, HIGH);
        delay(120);
        digitalWrite(PWDN_GPIO, LOW);
        delay(120);
    } else {
        delay(120);
    }

    // Try preferred configurations first
    for (int i = 0; i < INIT_RETRY_PER_CONFIG; ++i) {
        if (try_camera_init_once(FRAMESIZE_VGA, 10000000, JPEG_QUALITY_FALLBACK)) {
            camera_ok = true;
            discard_frames(DISCARD_FRAMES_ON_START);
            return true;
        }
        delay(150);
    }

    // If initial attempts fail, apply a more aggressive PWDN power-cycle retry sequence
    if (PWDN_GPIO >= 0) {
        for (int cycle = 0; cycle < 3 && !camera_ok; ++cycle) {
            // Perform a longer toggle sequence to fully reset module internal state
            digitalWrite(PWDN_GPIO, HIGH);  // try alternate polarity/time
            delay(200);
            digitalWrite(PWDN_GPIO, LOW);
            delay(200);

            // Attempt initialization again
            for (int i = 0; i < INIT_RETRY_PER_CONFIG; ++i) {
                if (try_camera_init_once(FRAMESIZE_VGA, 10000000, JPEG_QUALITY_FALLBACK)) {
                    camera_ok = true;
                    discard_frames(DISCARD_FRAMES_ON_START);
                    return true;
                }
                delay(150);
            }
        }
    }

    // Fallback: try other configs as before
    esp_camera_deinit(); delay(80);
    for (int i = 0; i < INIT_RETRY_PER_CONFIG; ++i) {
        if (try_camera_init_once(FRAME_SIZE_PREF, 10000000, JPEG_QUALITY_PREF)) {
            camera_ok = true;
            discard_frames(DISCARD_FRAMES_ON_START);
            return true;
        }
        delay(150);
    }
    esp_camera_deinit(); delay(80);
    for (int i = 0; i < INIT_RETRY_PER_CONFIG; ++i) {
        if (try_camera_init_once(FRAME_SIZE_PREF, 20000000, JPEG_QUALITY_PREF)) {
            camera_ok = true;
            discard_frames(DISCARD_FRAMES_ON_START);
            return true;
        }
        delay(150);
    }
    esp_camera_deinit(); delay(80);
    for (int i = 0; i < INIT_RETRY_PER_CONFIG; ++i) {
        if (try_camera_init_once(FRAMESIZE_QVGA, 10000000, JPEG_QUALITY_FALLBACK + 2)) {
            camera_ok = true;
            discard_frames(DISCARD_FRAMES_ON_START);
            return true;
        }
        delay(150);
    }
    esp_camera_deinit();
    camera_ok = false;
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
    camera_config_t cfg = make_config(size, 10000000, quality);
    if (esp_camera_init(&cfg) == ESP_OK) {
        tune_camera_sensor_defaults();
        camera_ok = true;
        return true;
    }
    deinit_camera_silent();
    cfg = make_config(size, 20000000, quality + 2);
    if (esp_camera_init(&cfg) == ESP_OK) {
        tune_camera_sensor_defaults();
        camera_ok = true;
        return true;
    }
    camera_ok = false;
    return false;
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