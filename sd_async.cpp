#include "sd_async.h"
#include <SD.h>
#include <Arduino.h>

// 简单队列实现（环形缓冲区），仅支持8个任务
struct SDTask {
    char path[64];
    size_t len;
    uint8_t* buf; // 任务提交后由本实现负责释放
};

#define SDQ_LEN 8
static SDTask tasks[SDQ_LEN];
static volatile int head = 0, tail = 0;
static volatile bool running = false;

void sd_async_init() {
    head = tail = 0;
    running = false;
    // 清空队列
    for (int i = 0; i < SDQ_LEN; ++i) {
        tasks[i].len = 0;
        tasks[i].buf = nullptr;
        tasks[i].path[0] = 0;
    }
}

void sd_async_start() {
    if (running) return;
    running = true;
}

void sd_async_on_sd_ready() {
    // 这里可做扩展，暂为空
}

void sd_async_on_sd_lost() {
    // 这里可做扩展，暂为空
}

// 非阻塞提交任务，成功返回true
bool sd_async_submit(const char* path, const uint8_t* data, size_t len) {
    int next = (tail + 1) % SDQ_LEN;
    if (next == head) return false; // 队列满
    if (len == 0 || !data || !path || strlen(path) > 63) return false;
    tasks[tail].len = len;
    tasks[tail].buf = (uint8_t*)malloc(len);
    if (!tasks[tail].buf) return false;
    memcpy(tasks[tail].buf, data, len);
    strncpy(tasks[tail].path, path, 63);
    tasks[tail].path[63] = 0;
    tail = next;
    return true;
}

// 在主loop或定时任务中调用，实际执行写入
void sd_async_loop() {
    while (head != tail) {
        SDTask& t = tasks[head];
        if (t.len > 0 && t.buf) {
            File f = SD.open(t.path, FILE_WRITE);
            if (f) {
                f.write(t.buf, t.len);
                f.close();
            }
            free(t.buf);
            t.buf = nullptr;
            t.len = 0;
            t.path[0] = 0;
        }
        head = (head + 1) % SDQ_LEN;
    }
}