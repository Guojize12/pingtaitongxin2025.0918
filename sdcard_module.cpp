#include "sdcard_module.h"
#include "config.h"
#include "sd_async.h"
#include "rtc_soft.h"

SPIClass sdSPI(VSPI);

// 本地递增计数用于生成唯一文件名（避免重复覆盖）
static uint32_t s_photo_counter = 0;

void init_sd() {
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, sdSPI)) {};
}

// 生成文件名：优先用RTC时间，其次用millis和本地计数
static void make_photo_name(char* out, size_t outSize) {
    if (rtc_is_valid()) {
        PlatformTime t;
        rtc_now_fields(&t);
        // /IMG_YYYYMMDD_HHMMSS_NNN.jpg
        snprintf(out, outSize, "/IMG_%04u%02u%02u_%02u%02u%02u_%03lu.jpg",
                 (unsigned)t.year, (unsigned)t.month, (unsigned)t.day,
                 (unsigned)t.hour, (unsigned)t.minute, (unsigned)t.second,
                 (unsigned long)(s_photo_counter % 1000));
    } else {
        // RTC无效时用millis+计数
        snprintf(out, outSize, "/IMG_ms%010lu_%03lu.jpg",
                 (unsigned long)millis(),
                 (unsigned long)(s_photo_counter % 1000));
    }
    s_photo_counter++;
}

bool save_frame_to_sd(camera_fb_t *fb, uint32_t index) {
    if (!fb) return false;
    // 忽略传入index，使用自带唯一命名
    char name[64];
    make_photo_name(name, sizeof(name));
    return save_frame_to_sd_raw(fb->buf, fb->len, index); // 内部会重新计算文件名
}

bool save_frame_to_sd_raw(const uint8_t* data, size_t len, uint32_t index) {
    if (SD.cardType() == CARD_NONE) return false;

    char name[64];
    make_photo_name(name, sizeof(name));

    // 若启用异步SD写，优先尝试入队；失败则回退同步写，避免数据丢失
    if (g_cfg.asyncSDWrite) {
        if (sd_async_submit(name, data, len)) {
            // 入队成功，立即返回，不阻塞
            return true;
        }
        // 队列满或内存不足，继续走同步写
    }

    File f = SD.open(name, FILE_WRITE);
    if (!f) return false;
    size_t w = f.write(data, len);
    f.close();
    return w == len;
}

// 新增：保存并返回实际文件名（时间命名）
bool save_frame_to_sd_with_name(camera_fb_t* fb, char* outFile, size_t outFileSize) {
    if (!fb) return false;
    if (!outFile || outFileSize < 4) return false;

    char name[64];
    make_photo_name(name, sizeof(name));

    bool ok = false;
    if (g_cfg.asyncSDWrite) {
        // 异步入队（写线程会逐块写入），此处立即返回true
        ok = sd_async_submit(name, fb->buf, fb->len);
        if (!ok) {
            // 回退同步写
            File f = SD.open(name, FILE_WRITE);
            if (f) {
                size_t w = f.write(fb->buf, fb->len);
                f.close();
                ok = (w == fb->len);
            }
        }
    } else {
        File f = SD.open(name, FILE_WRITE);
        if (f) {
            size_t w = f.write(fb->buf, fb->len);
            f.close();
            ok = (w == fb->len);
        }
    }

    if (ok) {
        strncpy(outFile, name, outFileSize - 1);
        outFile[outFileSize - 1] = '\0';
    }
    return ok;
}