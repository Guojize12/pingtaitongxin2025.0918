#pragma once
#include <stdint.h>

// 保留原有状态枚举，兼容 main.ino 中的用法
typedef enum {
    STEP_IDLE = 0,
    STEP_WAIT_READY,
    STEP_AT_PING,
    STEP_CEREG,
    STEP_ENCODING,
    STEP_MIPCLOSE,
    STEP_MIPOPEN,
    STEP_MONITOR
} Step;

// 原有对外接口（保持不变，兼容 main.ino）
void gotoStep(Step s);
void resetBackoff();
void driveStateMachine();