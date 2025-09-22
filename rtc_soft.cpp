#include "rtc_soft.h"
#include <Arduino.h>
#include "config.h"

static bool s_valid = false;
static uint32_t s_base_epoch = 0;      // 上次校时的UTC秒（UNIX epoch）
static uint32_t s_base_millis = 0;     // 上次校时时的millis

// 闰年判断（公历规则）
static inline bool isLeap(uint32_t y) {
    return ( (y % 4 == 0) && (y % 100 != 0) ) || (y % 400 == 0);
}

static inline uint8_t daysInMonth(uint32_t y, uint8_t m) {
    static const uint8_t mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (m == 2) return mdays[1] + (isLeap(y) ? 1 : 0);
    return mdays[m - 1];
}

// 平台时间（YYYY/MM/DD hh:mm:ss）转 UNIX epoch 秒（UTC）
// 注意：这里按 2000-01-01 为参考加上 946684800 偏移，与你原来的算法一致
static uint32_t platformTimeToEpoch(const PlatformTime* t) {
    uint16_t year = t->year;
    uint8_t month = t->month;
    uint8_t day = t->day;
    uint8_t hour = t->hour;
    uint8_t minute = t->minute;
    uint8_t second = t->second;

    // 计算从 2000-01-01 到指定日期的天数
    // 更稳妥的做法：逐年逐月累加，避免简化闰年误差
    uint32_t days = 0;

    // 加上从 2000 到 year-1 的整年天数
    for (uint32_t y = 2000; y < year; ++y) {
        days += isLeap(y) ? 366 : 365;
    }
    // 加上 year 年内从 1 月到 month-1 月的整月天数
    for (uint8_t m = 1; m < month; ++m) {
        days += daysInMonth(year, m);
    }
    // 加上当月天数（从 1 号到 day-1）
    days += (uint32_t)(day - 1);

    uint32_t seconds = days * 86400UL + hour * 3600UL + minute * 60UL + second;

    // 2000-01-01 到 1970-01-01 的秒数偏移：946684800
    return seconds + 946684800UL;
}

// UNIX epoch → 年月日时分秒（UTC）
static void epochToFields(uint32_t epoch, PlatformTime* out) {
    // 先取出一天内的时分秒
    uint32_t secsInDay = epoch % 86400UL;
    out->hour   = (uint8_t)(secsInDay / 3600UL);
    secsInDay  %= 3600UL;
    out->minute = (uint8_t)(secsInDay / 60UL);
    out->second = (uint8_t)(secsInDay % 60UL);

    // 计算天数（从 1970-01-01 起的天数）
    uint32_t days = epoch / 86400UL;

    // 推算年份
    uint32_t y = 1970;
    while (true) {
        uint32_t dy = isLeap(y) ? 366 : 365;
        if (days >= dy) {
            days -= dy;
            ++y;
        } else {
            break;
        }
    }
    out->year = (uint16_t)y;

    // 推算月份
    uint8_t m = 1;
    while (true) {
        uint8_t dm = daysInMonth(y, m);
        if (days >= dm) {
            days -= dm;
            ++m;
        } else {
            break;
        }
    }
    out->month = m;
    out->day   = (uint8_t)(days + 1);
}

void rtc_init() {
    // 目前无初始化内容
}

bool rtc_is_valid() {
    return s_valid;
}

// 当前 UNIX epoch（秒）
uint32_t rtc_now() {
    if (!s_valid) return 0;
    uint32_t now_millis = millis();
    uint32_t delta_ms = now_millis - s_base_millis;
    return s_base_epoch + (delta_ms / 1000UL);
}

// 当前 UTC 时间（YYYY-MM-DD hh:mm:ss）
void rtc_now_fields(PlatformTime* out) {
    if (!s_valid) {
        out->year = 1970; out->month = 1; out->day = 1;
        out->hour = 0; out->minute = 0; out->second = 0;
        return;
    }
    uint32_t t = rtc_now();
    epochToFields(t, out);
}

// 校时：收到平台时间包后调用
void rtc_on_sync(const PlatformTime* plat, uint32_t recv_millis) {
    s_base_epoch = platformTimeToEpoch(plat);
    s_base_millis = recv_millis;
    s_valid = true;

#if ENABLE_LOG2
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
#endif
}