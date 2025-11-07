#include "comm_manager.h"
#include "config.h"
#include "uart_utils.h"
#include "at_commands.h"
#include "platform_packet.h"
#include <Arduino.h>
#include <ctype.h> 

// ================== 通信状态机内部变量 ==================
static Step step = STEP_IDLE;
static uint32_t actionStartMs = 0;
static uint32_t nextStatePollMs = 0;
static uint32_t backoffMs = 2000;
static uint32_t lastHeartbeatMs = 0;
static bool tcpConnected = false;

// === 定时请求时间同步相关变量 ===
static uint32_t lastTimeSyncReqMs = 0;

// ================== 非阻塞重试调度变量 ==================
enum RetryAction : uint8_t {
    RA_NONE = 0,
    RA_START_AT_PING = 1,
    RA_QUERY_CEREG = 2,
    RA_OPEN_TCP = 3
};
static volatile RetryAction pendingRetryAction = RA_NONE;
static uint32_t nextRetryAllowedMs = 0;

// ================== 工具函数 ==================
// Trim copy: remove leading/trailing whitespace, write to out (always NUL-terminated)
static void trim_copy(const char* s, char* out, size_t outlen) {
    if (!s || outlen == 0) return;
    const char* a = s;
    // skip leading
    while (*a && isspace((unsigned char)*a)) ++a;
    // find end
    const char* b = a;
    while (*b) ++b;
    // back off trailing spaces
    if (b > a) {
        --b;
        while (b > a && isspace((unsigned char)*b)) --b;
        ++b; // one past last non-space
    }
    size_t len = (size_t)(b - a);
    if (len >= outlen) len = outlen - 1;
    if (len > 0) memcpy(out, a, len);
    out[len] = '\0';
}

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

// schedule a retry action without blocking the main loop
static void scheduleRetry(RetryAction a) {
    pendingRetryAction = a;
    nextRetryAllowedMs = millis() + backoffMs;
    // keep actionStartMs for step timeouts semantics
}

// ================== 各状态处理函数 ==================
static void handleStepIdle() {
    comm_gotoStep(STEP_WAIT_READY);
    actionStartMs = millis();
    startATPing();
}

static void handleStepWaitReady(const char* line) {
    if (lineHas(line, "+MATREADY")) {
        startATPing();
    }
}

static void handleStepAtPing(const char* line) {
    if (lineHas(line, "OK")) {
        comm_resetBackoff();
        queryCEREG();
    }
}

static void handleStepCereg(const char* line) {
    if (lineHas(line, "+CEREG")) {
        int stat = -1;
        const char* p = strchr(line, ',');
        if (p) { stat = atoi(++p); }
        if (stat == 1 || stat == 5) {
            setEncoding();
        }
    }
}

static void handleStepEncoding(const char* line) {
    if (lineHas(line, "OK") || lineHas(line, "ERROR")) {
        closeCh0();
    }
}

static void handleStepMipclose(const char* line) {
    if (lineHas(line, "OK") || lineHas(line, "+MIPCLOSE")) {
        openTCP();
    }
}

static void handleStepMipopen(const char* line) {
    if (lineHas(line, "+MIPOPEN")) {
        int ch = -1, code = -1;
        const char* p = strstr(line, "+MIPOPEN");
        if (p) {
            p = strchr(p, ':'); if (p) ++p;
            if (p) {
                while (*p == ' ') ++p;
                ch = atoi(p);
                p = strchr(p, ',');
                if (p) { code = atoi(++p); }
            }
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
            // non-blocking: schedule a retry to open TCP later
            scheduleRetry(RA_OPEN_TCP);
        }
    } else if (lineHas(line, "ERROR")) {
        log2("TCP open ERROR");
        tcpConnected = false;
        growBackoff();
        scheduleRetry(RA_OPEN_TCP);
    }
}

static void handleStepMonitor(const char* line) {
    if (lineHas(line, "+MIPSTATE")) {
        if (strstr(line, "CONNECTED")) {
            tcpConnected = true;
        } else {
            log2("TCP disconnected");
            tcpConnected = false;
            growBackoff();
            scheduleRetry(RA_OPEN_TCP);
        }
    }
}

static void handleDisconnEvent(const char* line) {
    if (lineHas(line, "+MIPURC") && lineHas(line, "\"disconn\"")) {
        tcpConnected = false;
        log2("TCP disconnected");
        growBackoff();
        scheduleRetry(RA_OPEN_TCP);
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
    char line[LINE_BUF_MAX];
    trim_copy(rawLine, line, sizeof(line));
    if (line[0] == '\0') return;

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
// 现在实现非阻塞的 backoff/retry：把原来 delay(backoffMs); action(); 替换为 scheduleRetry(...)
void comm_drive() {
    uint32_t now = millis();

    // 执行到期的重试动作（非阻塞）
    if (pendingRetryAction != RA_NONE && now >= nextRetryAllowedMs) {
        RetryAction a = pendingRetryAction;
        pendingRetryAction = RA_NONE;
        switch (a) {
            case RA_START_AT_PING: startATPing(); break;
            case RA_QUERY_CEREG:   queryCEREG(); break;
            case RA_OPEN_TCP:      openTCP(); break;
            default: break;
        }
    }

    switch (step) {
        case STEP_IDLE:
            handleStepIdle();
            break;

        case STEP_AT_PING:
            if (now - actionStartMs > AT_TIMEOUT_MS) {
                growBackoff();
                scheduleRetry(RA_START_AT_PING);
            }
            break;

        case STEP_CEREG:
            if (now - actionStartMs > REG_TIMEOUT_MS) {
                growBackoff();
                scheduleRetry(RA_QUERY_CEREG);
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
                scheduleRetry(RA_OPEN_TCP);
            }
            break;

        case STEP_MONITOR: {
            if (now > nextStatePollMs) {
                pollMIPSTATE();
                scheduleStatePoll();
            }
            sendHeartbeatIfNeeded(now);
            sendTimeSyncIfNeeded(now); // 定时请求时间同步
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