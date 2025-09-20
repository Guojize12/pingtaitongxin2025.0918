#pragma once
#include <Arduino.h>

void log2(const char* msg);
void log2Val(const char* k, int v);
void log2Str(const char* k, const char* v);

void sendRaw(const char* s);
void sendCmd(const char* cmd);

void readDTU();
void setLineHandler(void (*handler)(const char*));

// 时间包解析成功标志（外部可读）
extern volatile bool g_platformTimeParsed;

// 时间数据结构
struct PlatformTime {
    uint16_t year;    // 年 (2字节)
    uint8_t month;    // 月 (1字节)
    uint8_t day;      // 日 (1字节)
    uint8_t hour;     // 时 (1字节)
    uint8_t minute;   // 分 (1字节)
    uint8_t second;   // 秒 (1字节)
};

// 全局时间变量（外部可读）
extern PlatformTime g_platformTime;

// 时间解析函数
bool parsePlatformTime(const uint8_t* data, size_t len, PlatformTime* time);