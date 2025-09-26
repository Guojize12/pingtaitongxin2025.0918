#include "comm_manager.h"
#include "config.h"
#include "uart_utils.h"
#include "at_commands.h"
#include "platform_packet.h"
#include <Arduino.h>

// ================== 通信状态机内部变量 ==================
static Step step = STEP_IDLE;
static uint32_t actionStartMs = 0;
static uint32_t nextStatePollMs = 0;
static uint32_t backoffMs = 2000;
static uint32_t lastHeartbeatMs = 0;
static bool tcpConnected = false;

// === 定时请求时间同步相关变量 ===
static uint32_t lastTimeSyncReqMs = 0;

// === 非阻塞退避调度 ===
enum NextAction { ACT_NONE, ACT_AT_PING, ACT_CEREG, ACT_OPEN_TCP };
static uint32_t retryNotBeforeMs = 0;
static bool waitingForRetry = false;
static NextAction nextAction = ACT_NONE;

// ================== 工具函数 ==================
static String trimLine(const char* s) { String t = s; t.trim(); return t; }
static bool lineHas(const char* line, const char* token) { return (strstr(line, token) != nullptr); }

void scheduleStatePoll() { nextStatePollMs = millis() + STATE_POLL_MS; }
void comm_resetBackoff() { backoffMs = 2000; waitingForRetry = false; nextAction = ACT_NONE; }

static void growBackoff() {
    uint32_t n = backoffMs * 2;
    if (n > BACKOFF_MAX_MS) n = BACKOFF_MAX_MS;
    backoffMs = n;
}

static void scheduleRetry(NextAction act) {
    uint32_t now = millis();
    growBackoff();
    retryNotBeforeMs = now + backoffMs;
    waitingForRetry = true;
    nextAction = act;
}

static void tryFireScheduled(uint32_t now) {
    if (!waitingForRetry) return;
    if (now < retryNotBeforeMs) return;
    waitingForRetry = false;
    NextAction act = nextAction;
    nextAction = ACT_NONE;
    switch (act) {
        case ACT_AT_PING:  startATPing(); break;
        case ACT_CEREG:    queryCEREG();  break;
        case ACT_OPEN_TCP: openTCP();     break;
        default: break;
    }
}

void comm_gotoStep(Step s) {
    step = s;
    actionStartMs = millis();
}

// ================== 各状态处理函数 ==================
static void handleStepIdle() {
    // 清晰的顺序：先等模块 +MATREADY，再开始 AT
    comm_gotoStep(STEP_WAIT_READY);
}

static void handleStepWaitReady(const String& line) {
    if (lineHas(line.c_str(), "+MATREADY")) {
        startATPing();
    }
}

static void handleStepAtPing(const String& line) {
    if (lineHas(line.c_str(), "OK")) {
        comm_resetBackoff();
        queryCEREG();
    }
}

static void handleStepCereg(const String& line) {
    if (lineHas(line.c_str(), "+CEREG")) {
        int stat = -1;
        const char* p = strchr(line.c_str(), ',');
        if (p) { stat = atoi(++p); }
        if (stat == 1 || stat == 5) {
            setEncoding();
        }
    }
}

static void handleStepEncoding(const String& line) {
    if (lineHas(line.c_str(), "OK") || lineHas(line.c_str(), "ERROR")) {
        closeCh0();
    }
}

static void handleStepMipclose(const String& line) {
    if (lineHas(line.c_str(), "OK") || lineHas(line.c_str(), "+MIPCLOSE")) {
        openTCP();
    }
}

static void handleStepMipopen(const String& line) {
    if (lineHas(line.c_str(), "+MIPOPEN")) {
        int ch = -1, code = -1;
        const char* p = strstr(line.c_str(), "+MIPOPEN");
        if (p) {
            p = strchr(p, ':'); if (p) ++p;
            while (*p == ' ') ++p;
            ch = atoi(p);
            p = strchr(p, ',');
            if (p) { code = atoi(++p); }
        }

        if (code == 0) {
            log2("TCP connected");
            tcpConnected = true;
            comm_resetBackoff();
            uint32_t now = millis();
            lastHeartbeatMs = now;
            // 建链成功后延迟几秒再发时间同步，避免一上来就挤占通道
            if (TIME_SYNC_INTERVAL_MS > 5000) {
                lastTimeSyncReqMs = now - TIME_SYNC_INTERVAL_MS + 5000;
            } else {
                lastTimeSyncReqMs = now; // 容错
            }
            comm_gotoStep(STEP_MONITOR);
            scheduleStatePoll();
        } else {
            log2("TCP open failed");
            tcpConnected = false;
            scheduleRetry(ACT_OPEN_TCP);
        }
    } else if (lineHas(line.c_str(), "ERROR")) {
        log2("TCP open ERROR");
        tcpConnected = false;
        scheduleRetry(ACT_OPEN_TCP);
    }
}

static void handleStepMonitor(const String& line) {
    if (lineHas(line.c_str(), "+MIPSTATE")) {
        if (strstr(line.c_str(), "CONNECTED")) {
            tcpConnected = true;
        } else {
            log2("TCP disconnected");
            tcpConnected = false;
            scheduleRetry(ACT_OPEN_TCP);
        }
    }
}

static void handleDisconnEvent(const String& line) {
    if (lineHas(line.c_str(), "+MIPURC") && lineHas(line.c_str(), "\"disconn\"")) {
        tcpConnected = false;
        log2("TCP disconnected");
        scheduleRetry(ACT_OPEN_TCP);
    }
}

// 心跳仅与通信维护有关，保留在通信层
static void sendHeartbeatIfNeeded(uint32_t now) {
    if (tcpConnected && (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS)) {
        sendHeartbeat();
        lastHeartbeatMs = now;
    }
}

// 定时发送时间同步请求
static void sendTimeSyncIfNeeded(uint32_t now) {
    if (tcpConnected && (now - lastTimeSyncReqMs >= TIME_SYNC_INTERVAL_MS)) {
        sendTimeSyncRequest();
        lastTimeSyncReqMs = now;
    }
}

// 行分发（注册到 uart_utils，主要用于AT命令应答和事件）
static void handleLine(const char* rawLine) {
    String line = trimLine(rawLine);
    if (line.length() == 0) return;

    // 断开事件
    handleDisconnEvent(line);

    switch (step) {
        case STEP_WAIT_READY: handleStepWaitReady(line); break;
        case STEP_AT_PING:    handleStepAtPing(line);    break;
        case STEP_CEREG:      handleStepCereg(line);     break;
        case STEP_ENCODING:   handleStepEncoding(line);  break;
        case STEP_MIPCLOSE:   handleStepMipclose(line);  break;
        case STEP_MIPOPEN:    handleStepMipopen(line);   break;
        case STEP_MONITOR:    handleStepMonitor(line);   break;
        default: break;
    }
}

// 主循环：仅负责通信维持（AT/TCP/心跳/状态轮询/时间同步）
void comm_drive() {
    uint32_t now = millis();

    // 先检查是否有“到点重试”的动作
    tryFireScheduled(now);

    switch (step) {
        case STEP_IDLE:
            handleStepIdle();
            break;

        case STEP_WAIT_READY:
            // 等 MATREADY 太久，则主动发 AT 探活
            if (now - actionStartMs > AT_TIMEOUT_MS) {
                startATPing();
            }
            break;

        case STEP_AT_PING:
            if (now - actionStartMs > AT_TIMEOUT_MS) {
                scheduleRetry(ACT_AT_PING);
            }
            break;

        case STEP_CEREG:
            if (now - actionStartMs > REG_TIMEOUT_MS) {
                scheduleRetry(ACT_CEREG);
            }
            break;

        case STEP_ENCODING:
            if (now - actionStartMs > AT_TIMEOUT_MS) {
                closeCh0();
            }
            break;

        case STEP_MIPCLOSE:
            if (now - actionStartMs > AT_TIMEOUT_MS) {
                openTCP();
            }
            break;

        case STEP_MIPOPEN:
            if (now - actionStartMs > OPEN_TIMEOUT_MS) {
                tcpConnected = false;
                scheduleRetry(ACT_OPEN_TCP);
            }
            break;

        case STEP_MONITOR: {
            if (now > nextStatePollMs) {
                pollMIPSTATE();
                scheduleStatePoll();
            }
            sendHeartbeatIfNeeded(now);
            sendTimeSyncIfNeeded(now);
            break;
        }
        default:
            break;
    }
}

bool comm_isConnected() { return tcpConnected; }

// 在加载阶段注册串口行处理器
struct CommInit {
    CommInit() { setLineHandler(handleLine); }
} _commInit;