#pragma once
#include <stdint.h>
#include "uart_utils.h"

// 软件RTC初始化（可选，通常自动完成）
void rtc_init();

// 是否已获得有效时间（首次校时成功后为true）
bool rtc_is_valid();

// 获取当前UTC时间（秒，1970纪元）
uint32_t rtc_now();

// 获取当前UTC时间（年月日时分秒，UTC，输出到PlatformTime结构体）
void rtc_now_fields(PlatformTime* out);

// 校时接口：收到新的平台时间包后调用（传入PlatformTime和本地接收时的millis）
void rtc_on_sync(const PlatformTime* plat, uint32_t recv_millis);
