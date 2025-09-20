#include "sim_info.h"
#include "uart_utils.h"
#include <string.h>

// 同步发送AT命令并等待响应
static bool sendATCommandSync(const char* cmd, char* response, size_t responseSize, uint32_t timeoutMs) {
    // 清空响应缓冲区
    memset(response, 0, responseSize);
    
    // 发送命令
    sendCmd(cmd);
    
    // 等待响应
    uint32_t startTime = millis();
    size_t responseLen = 0;
    bool foundResponse = false;
    
    while (millis() - startTime < timeoutMs && !foundResponse) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\r' || c == '\n') {
                if (responseLen > 0) {
                    response[responseLen] = '\0';
                    // 检查是否为有效响应（不是OK或ERROR）
                    if (strstr(response, "OK") == NULL && strstr(response, "ERROR") == NULL && responseLen > 2) {
                        foundResponse = true;
                    } else if (strstr(response, "ERROR") != NULL) {
                        return false; // 命令执行错误
                    } else {
                        // 重置缓冲区继续等待
                        responseLen = 0;
                        memset(response, 0, responseSize);
                    }
                }
            } else if (responseLen < responseSize - 1) {
                response[responseLen++] = c;
            }
        }
        delay(10); // 短暂延时避免过度占用CPU
    }
    
    return foundResponse;
}

// 解析CSQ响应，转换为0-100信号强度
uint8_t parseSignalStrength(const char* csqResponse) {
    // CSQ响应格式: +CSQ: <rssi>,<ber>
    // rssi范围: 0-31 (99表示未知)
    if (!csqResponse) return 0;
    
    const char* csqStart = strstr(csqResponse, "+CSQ:");
    if (!csqStart) return 0;
    
    // 跳过 "+CSQ: "
    csqStart += 6;
    while (*csqStart == ' ') csqStart++; // 跳过空格
    
    int rssi = 0;
    if (sscanf(csqStart, "%d", &rssi) != 1) return 0;
    
    // 转换为0-100范围
    if (rssi == 99) return 0; // 未知信号
    if (rssi < 0) return 0;
    if (rssi > 31) return 100;
    
    // 线性映射: 0-31 -> 0-100
    return (uint8_t)((rssi * 100) / 31);
}

// 采集SIM卡信息（同步执行，失败可重试）
bool collectSimInfo(SimInfo* info) {
    if (!info) return false;
    
    // 初始化结构体
    memset(info, 0, sizeof(SimInfo));
    info->valid = false;
    
    char response[64];
    bool success = true;
    
    // 获取ICCID (AT+MCCID)
    for (int retry = 0; retry < 3 && success; retry++) {
        if (sendATCommandSync("AT+MCCID", response, sizeof(response), 3000)) {
            // 解析ICCID响应，通常直接返回ICCID号码
            size_t len = strlen(response);
            if (len > 0 && len <= 20) {
                strncpy(info->iccid, response, sizeof(info->iccid) - 1);
                info->iccid[sizeof(info->iccid) - 1] = '\0';
                break;
            }
        }
        if (retry == 2) success = false;
        delay(500);
    }
    
    // 获取IMSI (AT+CIMI)
    if (success) {
        for (int retry = 0; retry < 3 && success; retry++) {
            if (sendATCommandSync("AT+CIMI", response, sizeof(response), 3000)) {
                // 解析IMSI响应，通常直接返回IMSI号码
                size_t len = strlen(response);
                if (len > 0 && len <= 15) {
                    strncpy(info->imsi, response, sizeof(info->imsi) - 1);
                    info->imsi[sizeof(info->imsi) - 1] = '\0';
                    break;
                }
            }
            if (retry == 2) success = false;
            delay(500);
        }
    }
    
    // 获取信号强度 (AT+CSQ)
    if (success) {
        for (int retry = 0; retry < 3 && success; retry++) {
            if (sendATCommandSync("AT+CSQ", response, sizeof(response), 3000)) {
                info->signalStrength = parseSignalStrength(response);
                break;
            }
            if (retry == 2) success = false;
            delay(500);
        }
    }
    
    info->valid = success;
    return success;
}