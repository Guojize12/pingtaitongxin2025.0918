#include "sdcard_module.h"
#include "sd_async.h"
#include "logging.h"
#include "config.h"

SPIClass sdSPI(VSPI);
static uint32_t sd_remount_backoff_ms = 3000;
static uint32_t sd_next_remount_allowed = 0;
static const uint32_t SD_BACKOFF_MAX = 30000;

void init_sd() {
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, sdSPI)) LOG_WARN("[SD] init fail");
    else {
        static bool async_started = false;
        if (!async_started) {
            sd_async_init();
            sd_async_start();
            async_started = true;
        }
        sd_async_on_sd_ready();
    }
}

void periodic_sd_check() {
    uint32_t now = millis();
    if (SD.cardType() != CARD_NONE) { sd_remount_backoff_ms = 3000; return; }
    if (now < sd_next_remount_allowed) return;
    sd_async_on_sd_lost();
    init_sd();
    if (SD.cardType() == CARD_NONE) {
        sd_remount_backoff_ms = min<uint32_t>(sd_remount_backoff_ms * 2, SD_BACKOFF_MAX);
        sd_next_remount_allowed = now + sd_remount_backoff_ms;
    } else sd_remount_backoff_ms = 3000;
}

bool save_frame_to_sd_raw(const uint8_t* data, size_t len, uint32_t index) {
    if (SD.cardType() == CARD_NONE) return false;
    uint64_t free = (SD.totalBytes() - SD.usedBytes());
    if (free / (1024 * 1024) < SD_MIN_FREE_MB) return false;
    char name[48]; snprintf(name, sizeof(name), "/photo_%05lu.jpg", (unsigned long)index);
    File f = SD.open(name, FILE_WRITE); if (!f) return false;
    size_t w = f.write(data, len); f.close(); return w == len;
}

bool save_frame_to_sd(camera_fb_t *fb, uint32_t index) {
    if (!fb) return false;
    char name[48];
    snprintf(name, sizeof(name), "/photo_%05lu.jpg", (unsigned long)index);
    if (g_cfg.asyncSDWrite) {
        if (sd_async_submit(name, fb->buf, fb->len)) {
            return true;
        } else {
            LOG_WARN("[SDASYNC] enq fail, fallback to sync");
            return save_frame_to_sd_raw(fb->buf, fb->len, index);
        }
    } else {
        return save_frame_to_sd_raw(fb->buf, fb->len, index);
    }
}