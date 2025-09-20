#include "sim_info.h"
#include "uart_utils.h"
#include <Arduino.h>

// 发送AT并等待应答的简单同步实现
static bool waitForATResponse(const char* at, char* out, size_t outlen, const char* prefix, uint32_t timeout = 2000) {
    out[0] = 0;
    sendCmd(at);
    uint32_t start = millis();
    while (millis() - start < timeout) {
        while (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line.length() == 0) continue;
            if (prefix && !line.startsWith(prefix)) continue;
            strncpy(out, line.c_str(), outlen - 1);
            out[outlen - 1] = 0;
            return true;
        }
        delay(10);
    }
    return false;
}

bool siminfo_query(SimInfo* sim) {
    memset(sim, 0, sizeof(SimInfo));

    char buf[64];
    // 1. ICCID
    if (waitForATResponse("AT+MCCID", buf, sizeof(buf), "+MCCID:")) {
        // 格式: +MCCID: 8986xxxxxxxxxxxxxxx
        char* p = strchr(buf, ':');
        if (p) {
            ++p;
            while (*p == ' ') ++p;
            strncpy(sim->iccid, p, 20);
            sim->iccid[20] = 0;
            sim->iccid_len = strlen(sim->iccid);
        }
    }

    // 2. IMSI
    if (waitForATResponse("AT+CIMI", buf, sizeof(buf), nullptr)) {
        // IMSI直接在下一行返回，不带前缀
        if (isdigit(buf[0]) && strlen(buf) >= 10) {
            strncpy(sim->imsi, buf, 15);
            sim->imsi[15] = 0;
            sim->imsi_len = strlen(sim->imsi);
        }
    }

    // 3. 信号强度
    if (waitForATResponse("AT+CSQ", buf, sizeof(buf), "+CSQ:")) {
        // 格式: +CSQ: <rssi>,<ber>
        int rssi = 0;
        char* p = strchr(buf, ':');
        if (p) {
            ++p;
            while (*p == ' ') ++p;
            rssi = atoi(p);
            // 0~31, 99无效，换算为0~100
            if (rssi >= 0 && rssi <= 31) sim->signal = (uint8_t)(rssi * 100 / 31);
            else sim->signal = 0;
        }
    }

    sim->valid = sim->iccid_len > 0 && sim->imsi_len > 0;
    return sim->valid;
}