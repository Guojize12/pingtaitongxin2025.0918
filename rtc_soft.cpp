#include "rtc_soft.h"
#include <Arduino.h>

// 内部状态
static bool s_valid = false;
static uint32_t s_base_epoch = 0;      // 上次校时的UTC秒
static uint32_t s_base_millis = 0;     // 上次校时时的millis

// 辅助：将PlatformTime转为UTC时间戳（秒）
static uint32_t platformTimeToEpoch(const PlatformTime* t) {
    // 简单实现（只适用于2000-2099，未考虑闰秒、夏令时等）
    uint16_t year = t->year;
    uint8_t month = t->month;
    uint8_t day = t->day;
    uint8_t hour = t->hour;
    uint8_t minute = t->minute;
    uint8_t second = t->second;
    // 计算自2000-01-01 00:00:00的秒数
    uint16_t y = year - 2000;
    static const uint16_t daysBeforeMonth[12] = {
        0,31,59,90,120,151,181,212,243,273,304,334
    };
    uint32_t days = y * 365 + (y+3)/4; // 粗略加闰年
    days += daysBeforeMonth[month-1];
    if (month > 2 && ((year%4)==0)) days += 1; // 闰年补一天
    days += day - 1;
    return (days * 24UL + hour) * 3600UL + minute * 60UL + second + 946684800UL; // 2000-01-01转Unix
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
    // 简化：只输出校时获得的最新值
    if (!s_valid) {
        out->year = 1970; out->month = 1; out->day = 1;
        out->hour = 0; out->minute = 0; out->second = 0;
        return;
    }
    uint32_t t = rtc_now();
    // 反推年月日时分秒（略简化，正式可用timegm/localtime等）
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
}
