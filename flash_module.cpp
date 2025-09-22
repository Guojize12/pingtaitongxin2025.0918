#include "flash_module.h"
#include "config.h"

// 不使用PWM，全部用数字IO
void flashInit() {
    pinMode(FLASH_PIN, OUTPUT);
    digitalWrite(FLASH_PIN, LOW);
}
void flashSet(uint8_t d) { (void)d; }
void flashOn() { digitalWrite(FLASH_PIN, HIGH); }
void flashOff() { digitalWrite(FLASH_PIN, LOW); }