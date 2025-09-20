#pragma once
#include <stdint.h>
#include <stdbool.h>

// SIM卡信息结构体
typedef struct {
    char iccid[21];    // ICCID字符串，最长20字节，结尾0
    uint8_t iccid_len; // 实际长度
    char imsi[16];     // IMSI字符串，最长15字节，结尾0
    uint8_t imsi_len;  // 实际长度
    uint8_t signal;    // 信号强度(0~100)
    bool valid;        // 是否采集成功
} SimInfo;

// 同步采集SIM卡信息（阻塞等待AT应答）
bool siminfo_query(SimInfo* out);
