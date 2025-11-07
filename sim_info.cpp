#include "sim_info.h"
#include "uart_utils.h"
#include <Arduino.h>
#include <ctype.h>
#include <string.h>

#define SIM_DBG 1
#if SIM_DBG
#define SIM_LOG(msg) log2(msg)
#define SIM_LOG2(k, v) log2Str(k, v)
#define SIM_LOGVAL(k, v) log2Val(k, v)
#else
#define SIM_LOG(msg)
#define SIM_LOG2(k, v)
#define SIM_LOGVAL(k, v)
#endif

static bool waitForATResponse(const char* at, char* out, size_t outlen, const char* prefix, uint32_t timeout = 2000) {
    out[0] = 0;
    SIM_LOG2("[SIM] Send AT: ", at);
    sendCmd(at);
    uint32_t start = millis();
    while (millis() - start < timeout) {
        while (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line.length() == 0) continue;
            SIM_LOG2("[SIM] AT Resp: ", line.c_str());
            if (prefix && !line.startsWith(prefix)) continue;
            strncpy(out, line.c_str(), outlen - 1);
            out[outlen - 1] = 0;
            return true;
        }
        delay(10);
    }
    SIM_LOG("[SIM] AT timeout");
    return false;
}

static bool collectIMSI(char* out, size_t outlen, uint32_t timeout = 2000) {
    out[0] = 0;
    SIM_LOG2("[SIM] Send AT: ", "AT+CIMI");
    sendCmd("AT+CIMI");
    uint32_t start = millis();
    bool found = false;
    while (millis() - start < timeout) {
        while (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line.length() == 0) continue;
            SIM_LOG2("[SIM] AT Resp: ", line.c_str());
            if (line == "OK" || !isdigit(line[0])) continue;
            if (line.length() >= 10 && line.length() <= 16) {
                strncpy(out, line.c_str(), outlen - 1);
                out[outlen - 1] = 0;
                found = true;
                break;
            }
        }
        if (found) break;
        delay(10);
    }
    if (!found) SIM_LOG("[SIM] IMSI not found");
    return found;
}

bool siminfo_query(SimInfo* sim) {
    memset(sim, 0, sizeof(SimInfo));

    char buf[64];
    if (waitForATResponse("AT+MCCID", buf, sizeof(buf), "+MCCID:")) {
        char* p = strchr(buf, ':');
        if (p) {
            ++p;
            while (*p == ' ') ++p;
            strncpy(sim->iccid, p, 20);
            sim->iccid[20] = 0;
            sim->iccid_len = strlen(sim->iccid);
            SIM_LOG2("[SIM] ICCID: ", sim->iccid);
        } else {
            SIM_LOG("[SIM] ICCID line malformed");
        }
    } else {
        SIM_LOG("[SIM] ICCID read fail");
    }

    if (collectIMSI(buf, sizeof(buf))) {
        strncpy(sim->imsi, buf, 15);
        sim->imsi[15] = 0;
        sim->imsi_len = strlen(sim->imsi);
        SIM_LOG2("[SIM] IMSI: ", sim->imsi);
    } else {
        SIM_LOG("[SIM] IMSI read fail");
    }

    if (waitForATResponse("AT+CSQ", buf, sizeof(buf), "+CSQ:")) {
        int rssi = 0;
        char* p = strchr(buf, ':');
        if (p) {
            ++p;
            while (*p == ' ') ++p;
            rssi = atoi(p);
            if (rssi >= 0 && rssi <= 31) sim->signal = (uint8_t)(rssi * 100 / 31);
            else sim->signal = 0;
            SIM_LOGVAL("[SIM] Signal(0-100): ", sim->signal);
        }
    } else {
        SIM_LOG("[SIM] Signal read fail");
    }

    sim->valid = sim->iccid_len > 0 && sim->imsi_len > 0;
    if (sim->valid) SIM_LOG("[SIM] SIM info collect OK");
    else SIM_LOG("[SIM] SIM info invalid");
    return sim->valid;
}