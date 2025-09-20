#include "sim_info.h"
#include "uart_utils.h"
#include <string.h>

// 全局变量用于AT命令响应处理
static char g_atResponse[64];
static bool g_atResponseReady = false;
static bool g_atCommandError = false;

// AT命令响应处理器
static void atResponseHandler(const char* line) {
    if (!line) return;
    
    // 跳过空行
    if (strlen(line) == 0) return;
    
    // 检查错误响应
    if (strstr(line, "ERROR") != NULL) {
        g_atCommandError = true;
        g_atResponseReady = true;
        return;
    }
    
    // 检查成功响应
    if (strstr(line, "OK") != NULL) {
        g_atResponseReady = true;
        return;
    }
    
    // 如果不是OK/ERROR，则可能是数据响应
    if (strlen(line) > 2 && !g_atResponseReady) {
        strncpy(g_atResponse, line, sizeof(g_atResponse) - 1);
        g_atResponse[sizeof(g_atResponse) - 1] = '\0';
        // 不立即设置ready，等待OK
    }
}

// 同步发送AT命令并等待响应
static bool sendATCommandSync(const char* cmd, char* response, size_t responseSize, uint32_t timeoutMs) {
    // 清空响应缓冲区
    memset(response, 0, responseSize);
    memset(g_atResponse, 0, sizeof(g_atResponse));
    g_atResponseReady = false;
    g_atCommandError = false;
    
    // 设置响应处理器
    setLineHandler(atResponseHandler);
    
    // 发送命令
    sendCmd(cmd);
    
    // 等待响应
    uint32_t startTime = millis();
    while (millis() - startTime < timeoutMs && !g_atResponseReady) {
        readDTU(); // 处理收到的数据
        delay(10); // 短暂延时避免过度占用CPU
    }
    
    // 清除响应处理器
    setLineHandler(nullptr);
    
    if (g_atCommandError) {
        return false;
    }
    
    if (g_atResponseReady && strlen(g_atResponse) > 0) {
        strncpy(response, g_atResponse, responseSize - 1);
        response[responseSize - 1] = '\0';
        return true;
    }
    
    return false;
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