#include "sdcard_module.h"
#include "config.h"

SPIClass sdSPI(VSPI);

void init_sd() {
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, sdSPI)) {};
}

bool save_frame_to_sd(camera_fb_t *fb, uint32_t index) {
    if (!fb) return false;
    char name[48];
    snprintf(name, sizeof(name), "/photo_%05lu.jpg", (unsigned long)index);
    return save_frame_to_sd_raw(fb->buf, fb->len, index);
}

bool save_frame_to_sd_raw(const uint8_t* data, size_t len, uint32_t index) {
    if (SD.cardType() == CARD_NONE) return false;
    char name[48];
    snprintf(name, sizeof(name), "/photo_%05lu.jpg", (unsigned long)index);
    File f = SD.open(name, FILE_WRITE); if (!f) return false;
    size_t w = f.write(data, len); f.close(); return w == len;
}