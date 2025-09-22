#pragma once
#include <Arduino.h>

// 初始化异步SD写任务、队列等资源
void sd_async_init();

// 启动后台异步SD写任务
void sd_async_start();

// 主动通知SD卡已准备好（一般在SD卡挂载后调用）
void sd_async_on_sd_ready();

// 主动通知SD卡已丢失（一般在SD卡拔出/挂载失败后调用）
void sd_async_on_sd_lost();

// 提交写入任务到异步队列
// 返回true代表已成功排队, false表示队列已满或资源不足
bool sd_async_submit(const char* path, const uint8_t* data, size_t len);

// 在主循环中调用以执行实际写入（处理队列）
void sd_async_loop();