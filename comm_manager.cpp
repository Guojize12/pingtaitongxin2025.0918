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

// ================== 工具函数 ==================
static String trimLine(const char* s) { String t = s; t.trim(); return t; }
static bool lineHas(const char* line, const char* token) { return (strstr(line, token) != nullptr); }

// 现在是全局可见函数
void scheduleStatePoll() { nextStatePollMs = millis() + STATE_POLL_MS; }
void comm_resetBackoff() { backoffMs = 2000; }

static void growBackoff() {
    uint32_t n = backoffMs * 2;
    if (n > BACKOFF_MAX_MS) n = BACKOFF_MAX_MS;
    backoffMs = n;
    log2Val("Backoff ms: ", backoffMs);
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
        log2("Module ready");
        startATPing();
    }
}

static void handleStepAtPing(const String& line) {
    if (lineHas(line.c_str(), "OK")) {
        log2("AT OK");
        comm_resetBackoff();
        queryCEREG();
    }
}

static void handleStepCereg(const String& line) {
    if (lineHas(line.c_str(), "+CEREG")) {
        int stat = -1;
        const char* p = strchr(line.c_str(), ',');
        if (p) { stat = atoi(++p); }
        log2Val("CEREG stat: ", stat);
        if (stat == 1 || stat == 5) {
            log2("Network registered");
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
        Serial2.print("MIPOPEN ch="); Serial2.print(ch);
        Serial2.print(" code="); Serial2.println(code);

        if (code == 0) {
            log2("TCP connected");
            tcpConnected = true;
            comm_resetBackoff();
            lastHeartbeatMs = millis();
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
            if (!tcpConnected) log2("State says CONNECTED");
            tcpConnected = true;
        } else {
            log2("State not CONNECTED");
            tcpConnected = false;
            growBackoff();
            delay(backoffMs);
            openTCP();
        }
    }
}

static void handleDisconnEvent(const String& line) {
    if (lineHas(line.c_str(), "+MIPURC") && lineHas(line.c_str(), "\"disconn\"")) {
        int chl = -1, err = -1;
        const char* p = strstr(line.c_str(), "\"disconn\"");
        if (p) {
            p = strchr(p, ','); if (p) { chl = atoi(++p); }
            p = strchr(p, ','); if (p) { err = atoi(++p); }
        }
        Serial2.print("TCP disconnected. ch="); Serial2.print(chl);
        Serial2.print(" err="); Serial2.println(err);

        tcpConnected = false;
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

// 串口行分发（注册到 uart_utils）
static void handleLine(const char* rawLine) {
    Serial2.print("[UART0 RX LINE] ");
    Serial2.println(rawLine);

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

// 主循环：仅负责通信维持（AT/TCP/心跳/状态轮询）
void comm_drive() {
    uint32_t now = millis();

    switch (step) {
        case STEP_IDLE:
            handleStepIdle();
            break;

        case STEP_AT_PING:
            if (now - actionStartMs > AT_TIMEOUT_MS) {
                log2("AT timeout, retry");
                growBackoff();
                delay(backoffMs);
                startATPing();
            }
            break;

        case STEP_CEREG:
            if (now - actionStartMs > REG_TIMEOUT_MS) {
                log2("CEREG timeout, retry");
                growBackoff();
                delay(backoffMs);
                queryCEREG();
            }
            break;

        case STEP_ENCODING:
            if (now - actionStartMs > AT_TIMEOUT_MS) {
                log2("Encoding timeout, continue");
                closeCh0();
            }
            break;

        case STEP_MIPCLOSE:
            if (now - actionStartMs > AT_TIMEOUT_MS) {
                log2("MIPCLOSE timeout, continue");
                openTCP();
            }
            break;

        case STEP_MIPOPEN:
            if (now - actionStartMs > OPEN_TIMEOUT_MS) {
                log2("MIPOPEN timeout, retry");
                tcpConnected = false;
                growBackoff();
                delay(backoffMs);
                openTCP();
            }
            break;

        case STEP_MONITOR: {
            if (now > nextStatePollMs) {
                log2("Poll state");
                pollMIPSTATE();
                scheduleStatePoll();
            }
            sendHeartbeatIfNeeded(now);
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