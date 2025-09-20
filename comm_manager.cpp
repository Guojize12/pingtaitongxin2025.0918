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

// === 新增：定时请求时间同步相关变量 ===
static uint32_t lastTimeSyncReqMs = 0;
static const uint32_t TIME_SYNC_INTERVAL_MS = 10000; // 10秒

// ================== 工具函数 ==================
static String trimLine(const char* s) { String t = s; t.trim(); return t; }
static bool lineHas(const char* line, const char* token) { return (strstr(line, token) != nullptr); }

void scheduleStatePoll() { nextStatePollMs = millis() + STATE_POLL_MS; }
void comm_resetBackoff() { backoffMs = 2000; }

static void growBackoff() {
    uint32_t n = backoffMs * 2;
    if (n > BACKOFF_MAX_MS) n = BACKOFF_MAX_MS;
    backoffMs = n;
}

void comm_gotoStep(Step s) {
    step = s;
    actionStartMs = millis();
}

// ================== 各状态处理函数 ==================
static void handleStepIdle() {
    comm_gotoStep(STEP_WAIT_READY);
    actionStartMs = millis();
    startATPing();
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
            lastHeartbeatMs = millis();
            lastTimeSyncReqMs = millis() - TIME_SYNC_INTERVAL_MS; // 立即触发
            comm_gotoStep(STEP_MONITOR);
            scheduleStatePoll();
        } else {
            log2("TCP open failed");
            tcpConnected = false;
            growBackoff();
            delay(backoffMs);
            openTCP();
        }
    } else if (lineHas(line.c_str(), "ERROR")) {
        log2("TCP open ERROR");
        tcpConnected = false;
        growBackoff();
        delay(backoffMs);
        openTCP();
    }
}

static void handleStepMonitor(const String& line) {
    if (lineHas(line.c_str(), "+MIPSTATE")) {
        if (strstr(line.c_str(), "CONNECTED")) {
            tcpConnected = true;
        } else {
            log2("TCP disconnected");
            tcpConnected = false;
            growBackoff();
            delay(backoffMs);
            openTCP();
        }
    }
}

static void handleDisconnEvent(const String& line) {
    if (lineHas(line.c_str(), "+MIPURC") && lineHas(line.c_str(), "\"disconn\"")) {
        tcpConnected = false;
        log2("TCP disconnected");
        growBackoff();
        delay(backoffMs);
        openTCP();
    }
}

// 心跳仅与通信维护有关，保留在通信层
static void sendHeartbeatIfNeeded(uint32_t now) {
    if (tcpConnected && (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS)) {
        sendHeartbeat();
        lastHeartbeatMs = now;
    }
}

// 新增：定时发送时间同步请求
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

    switch (step) {
        case STEP_IDLE:
            handleStepIdle();
            break;

        case STEP_AT_PING:
            if (now - actionStartMs > AT_TIMEOUT_MS) {
                growBackoff();
                delay(backoffMs);
                startATPing();
            }
            break;

        case STEP_CEREG:
            if (now - actionStartMs > REG_TIMEOUT_MS) {
                growBackoff();
                delay(backoffMs);
                queryCEREG();
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
                growBackoff();
                delay(backoffMs);
                openTCP();
            }
            break;

        case STEP_MONITOR: {
            if (now > nextStatePollMs) {
                pollMIPSTATE();
                scheduleStatePoll();
            }
            sendHeartbeatIfNeeded(now);
            sendTimeSyncIfNeeded(now); // 新增：定时请求时间同步
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