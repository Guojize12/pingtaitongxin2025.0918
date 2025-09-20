#pragma once
#include <Arduino.h>

// SIM卡信息结构体
struct SimInfo {
    char iccid[21];        // ICCID，最多20字符+结束符
    char imsi[16];         // IMSI，最多15字符+结束符
    uint8_t signalStrength; // 信号强度 0-100
    bool valid;            // 数据是否有效
};

// 采集SIM卡信息（同步执行，失败可重试）
bool collectSimInfo(SimInfo* info);

// 解析CSQ响应，转换为0-100信号强度
uint8_t parseSignalStrength(const char* csqResponse);