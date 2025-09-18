#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "state_machine.h"

// 通信管理对外接口
void comm_gotoStep(Step s);
void comm_resetBackoff();
void comm_drive();
bool comm_isConnected();

// 声明对外 scheduleStatePoll
void scheduleStatePoll();