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

// Read lines from Serial into out buffer (NUL-terminated).
// Blocking with timeout (ms). Returns true on a received non-empty line.
static bool readLineWithTimeout(char* out, size_t outlen, uint32_t timeout_ms) {
    if (!out || outlen == 0) return false;
    size_t idx = 0;
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        while (Serial.available()) {
            int c = Serial.read();
            if (c < 0) continue;
            if (c == '\r') continue;
            if (c == '\n') {
                // finish line
                while (idx > 0 && isspace((unsigned char)out[idx - 1])) --idx;
                out[idx] = '\0';
                // skip empty lines
                if (idx == 0) {
                    idx = 0;
                    continue;
                }
                return true;
            }
            if (idx < outlen - 1) {
                out[idx++] = (char)c;
            } else {
                // overflow, reset line
                idx = 0;
            }
        }
        delay(10);
    }
    return false;
}

static bool waitForATResponse(const char* at, char* out, size_t outlen, const char* prefix, uint32_t timeout) {
    if (!out || outlen == 0) return false;
    out[0] = '\0';
    SIM_LOG2("[SIM] Send AT: ", at);
    sendCmd(at);
    uint32_t start = millis();
    char line[LINE_BUF_MAX];
    while (millis() - start < timeout) {
        if (readLineWithTimeout(line, sizeof(line), timeout - (millis() - start))) {
            // trim leading spaces
            char tmp[LINE_BUF_MAX];
            trim_copy(line, tmp, sizeof(tmp));
            // rtrim already handled by trim_copy
            char* s = tmp;
            size_t l = strlen(s);
            if (l == 0) continue;
            SIM_LOG2("[SIM] AT Resp: ", s);
            if (prefix && strncmp(s, prefix, strlen(prefix)) != 0) continue;
            strncpy(out, s, outlen - 1);
            out[outlen - 1] = '\0';
            return true;
        }
        // no line yet
    }
    SIM_LOG("[SIM] AT timeout");
    return false;
}

// collect IMSI: send AT+CIMI and look for a numeric line of length 10..16
static bool collectIMSI(char* out, size_t outlen, uint32_t timeout) {
    if (!out || outlen == 0) return false;
    out[0] = '\0';
    SIM_LOG2("[SIM] Send AT: ", "AT+CIMI");
    sendCmd("AT+CIMI");
    uint32_t start = millis();
    char line[LINE_BUF_MAX];
    while (millis() - start < timeout) {
        if (readLineWithTimeout(line, sizeof(line), timeout - (millis() - start))) {
            // trim
            char tmp[LINE_BUF_MAX];
            size_t i = 0;
            // trim leading
            const char* p = line;
            while (*p && isspace((unsigned char)*p)) ++p;
            // copy and rtrim
            while (*p && i < sizeof(tmp) - 1) tmp[i++] = *p++;
            while (i > 0 && isspace((unsigned char)tmp[i - 1])) --i;
            tmp[i] = '\0';
            if (i == 0) continue;
            SIM_LOG2("[SIM] AT Resp: ", tmp);
            if (strcmp(tmp, "OK") == 0) continue;
            // check if starts with digit
            if (!isdigit((unsigned char)tmp[0])) continue;
            if (i >= 10 && i <= 16) {
                strncpy(out, tmp, outlen - 1);
                out[outlen - 1] = '\0';
                return true;
            }
        }
        delay(10);
    }
    SIM_LOG("[SIM] IMSI not found");
    return false;
}

bool siminfo_query(SimInfo* sim) {
    memset(sim, 0, sizeof(SimInfo));

    char buf[64];
    if (waitForATResponse("AT+MCCID", buf, sizeof(buf), "+MCCID:", 2000)) {
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

    if (collectIMSI(buf, sizeof(buf), 2000)) {
        strncpy(sim->imsi, buf, 15);
        sim->imsi[15] = 0;
        sim->imsi_len = strlen(sim->imsi);
        SIM_LOG2("[SIM] IMSI: ", sim->imsi);
    } else {
        SIM_LOG("[SIM] IMSI read fail");
    }

    if (waitForATResponse("AT+CSQ", buf, sizeof(buf), "+CSQ:", 2000)) {
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