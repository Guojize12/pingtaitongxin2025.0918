#include "flash_module.h"
#include "config.h"

static uint8_t g_flashDuty = DEFAULT_FLASH_DUTY;

void flashInit() {
#if FLASH_MODE
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)8,
        .timer_num = FLASH_TIMER,
        .freq_hz = FLASH_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&tcfg);
    ledc_channel_config_t ccfg = {
        .gpio_num = FLASH_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = FLASH_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = FLASH_TIMER,
        .duty = 0,
        .hpoint = 0
#if ESP_IDF_VERSION_MAJOR>=5
        , .flags={ .output_invert=0 }
#endif
    };
    ledc_channel_config(&ccfg);
#else
    pinMode(FLASH_PIN, OUTPUT);
    digitalWrite(FLASH_PIN, LOW);
#endif
}
void flashSet(uint8_t d) {
#if FLASH_MODE
    if (d > 255) d = 255;
    if (d > 0 && d < 5) d = 5;
    g_flashDuty = d;
#else
    (void)d;
#endif
}
void flashOn() {
#if FLASH_MODE
    ledc_set_duty(LEDC_LOW_SPEED_MODE, FLASH_CHANNEL, g_flashDuty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, FLASH_CHANNEL);
#else
    digitalWrite(FLASH_PIN, HIGH);
#endif
}
void flashOff() {
#if FLASH_MODE
    ledc_set_duty(LEDC_LOW_SPEED_MODE, FLASH_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, FLASH_CHANNEL);
#else
    digitalWrite(FLASH_PIN, LOW);
#endif
}