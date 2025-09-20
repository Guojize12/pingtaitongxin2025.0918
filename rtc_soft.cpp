#include "rtc_soft.h"
#include <Arduino.h>

static bool s_valid = false;
static uint32_t s_base_epoch = 0;      // 上次校时的UTC秒
static uint32_t s_base_millis = 0;     // 上次校时时的millis

static uint32_t platformTimeToEpoch(const PlatformTime* t) {
    uint16_t year = t->year;
    uint8_t month = t->month;
    uint8_t day = t->day;
    uint8_t hour = t->hour;
    uint8_t minute = t->minute;
    uint8_t second = t->second;
    uint16_t y = year - 2000;
    static const uint16_t daysBeforeMonth[12] = {
        0,31,59,90,120,151,181,212,243,273,304,334
    };
    uint32_t days = y * 365 + (y+3)/4;
    days += daysBeforeMonth[month-1];
    if (month > 2 && ((year%4)==0)) days += 1;
    days += day - 1;
    return (days * 24UL + hour) * 3600UL + minute * 60UL + second + 946684800UL;
}

void rtc_init() {
    // 目前无初始化内容
}

bool rtc_is_valid() {
    return s_valid;
}

uint32_t rtc_now() {
    if (!s_valid) return 0;
    uint32_t now_millis = millis();
    uint32_t delta = now_millis - s_base_millis;
    return s_base_epoch + delta / 1000;
}

void rtc_now_fields(PlatformTime* out) {
    if (!s_valid) {
        out->year = 1970; out->month = 1; out->day = 1;
        out->hour = 0; out->minute = 0; out->second = 0;
        return;
    }
    uint32_t t = rtc_now();
    uint32_t s = t;
    out->second = s % 60; s /= 60;
    out->minute = s % 60; s /= 60;
    out->hour = s % 24; s /= 24;
    // 只做演示，未严格实现日期反推
    out->year = 1970; out->month = 1; out->day = 1;
}

void rtc_on_sync(const PlatformTime* plat, uint32_t recv_millis) {
    s_base_epoch = platformTimeToEpoch(plat);
    s_base_millis = recv_millis;
    s_valid = true;

    Serial2.print("[RTC] Sync OK: ");
    Serial2.print(plat->year); Serial2.print("-");
    Serial2.print((int)plat->month); Serial2.print("-");
    Serial2.print((int)plat->day); Serial2.print(" ");
    Serial2.print((int)plat->hour); Serial2.print(":");
    Serial2.print((int)plat->minute); Serial2.print(":");
    Serial2.print((int)plat->second);
    Serial2.print(" (millis=");
    Serial2.print(recv_millis);
    Serial2.println(")");
    Serial2.println("[RTC] RTC is now valid, uploading enabled.");
}