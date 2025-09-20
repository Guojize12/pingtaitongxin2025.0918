#include "state_machine.h"
#include "comm_manager.h"
#include "upload_manager.h"

// 保持原有 API，对外不变
void gotoStep(Step s) {
    comm_gotoStep(s);
}

void resetBackoff() {
    comm_resetBackoff();
}

void driveStateMachine() {
    // 底层通信维护（AT/TCP/心跳/重连/轮询）
    comm_drive();
    // 业务上传调度（实时/事件等）
    upload_drive();
}
