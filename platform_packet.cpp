#include "platform_packet.h"
#include "config.h"
#include "crc16.h"
#include "uart_utils.h"
#include <string.h>

// 头部固定长度
static const uint16_t PLATFORM_HEADER_LEN = 21;

// 每次 MIPSEND 发送的二进制字节数上限（将被转成 2x HEX 字符）
// 取 512 字节更容易满足多数模组单行长度限制
static const size_t MIPSEND_BIN_CHUNK = 512;

// 保留原构建函数（小包可用）；大包发送走流式发送
size_t build_platform_packet(uint8_t* out,
                             char opType,
                             uint16_t cmd,
                             uint8_t pid,
                             const uint8_t* payload,
                             uint16_t payloadLen)
{
    out[0]  = '$';
    out[1]  = (uint8_t)opType;
    out[2]  = (uint8_t)(payloadLen >> 8);
    out[3]  = (uint8_t)(payloadLen & 0xFF);
    for (int i = 0; i < 12; ++i) out[4 + i] = (uint8_t)g_device_sn[i];
    out[16] = PLATFORM_VER;
    out[17] = (uint8_t)(cmd >> 8);
    out[18] = (uint8_t)(cmd & 0xFF);
    out[19] = PLATFORM_DMODEL;
    out[20] = pid;

    uint16_t headCrc = crc16_modbus(out, PLATFORM_HEADER_LEN);
    out[PLATFORM_HEADER_LEN]     = (uint8_t)(headCrc >> 8);
    out[PLATFORM_HEADER_LEN + 1] = (uint8_t)(headCrc & 0xFF);

    size_t offset = PLATFORM_HEADER_LEN + 2;

    if (payloadLen > 0 && payload) {
        memcpy(out + offset, payload, payloadLen);
        offset += payloadLen;
        uint16_t dataCrc = crc16_modbus(payload, payloadLen);
        out[offset]     = (uint8_t)(dataCrc >> 8);
        out[offset + 1] = (uint8_t)(dataCrc & 0xFF);
        offset += 2;
    }
    return offset;
}

// HEX输出辅助
static inline void hexByte(uint8_t b) {
    static const char* HEXCHARS = "0123456789ABCDEF";
    char tmp[2];
    tmp[0] = HEXCHARS[(b >> 4) & 0x0F];
    tmp[1] = HEXCHARS[b & 0x0F];
    Serial.write((const uint8_t*)tmp, 2);
}

static void hexWriteBuf(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) hexByte(data[i]);
}

// 发送一段 HEX 数据（包装成一条 AT+MIPSEND=0,0,<HEX>\r\n）
// 注意：多次调用将依次在 TCP 上连续发送，平台协议数据在流上保持连续
static void mipSendHex(const uint8_t* data, size_t len) {
    while (len) {
        size_t n = len > MIPSEND_BIN_CHUNK ? MIPSEND_BIN_CHUNK : len;
        Serial.write((const uint8_t*)"AT+MIPSEND=0,0,", 15);
        hexWriteBuf(data, n);
        Serial.write((const uint8_t*)"\r\n", 2);
        data += n;
        len  -= n;
        // 适度让出 CPU/给模组时间处理，避免串口拥塞
        delay(2);
    }
}

// 分块流式发送平台数据包：避免超长单行 AT，支持大payload（≤65K）
void sendPlatformPacket(char opType,
                        uint16_t cmd,
                        uint8_t pid,
                        const uint8_t* payload,
                        uint16_t payloadLen)
{
    // 1) 组装头部（不含头CRC）
    uint8_t header[PLATFORM_HEADER_LEN];
    header[0]  = '$';
    header[1]  = (uint8_t)opType;
    header[2]  = (uint8_t)(payloadLen >> 8);
    header[3]  = (uint8_t)(payloadLen & 0xFF);
    for (int i = 0; i < 12; ++i) header[4 + i] = (uint8_t)g_device_sn[i];
    header[16] = PLATFORM_VER;
    header[17] = (uint8_t)(cmd >> 8);
    header[18] = (uint8_t)(cmd & 0xFF);
    header[19] = PLATFORM_DMODEL;
    header[20] = pid;

    // 2) 计算CRC
    uint16_t headCrc = crc16_modbus(header, PLATFORM_HEADER_LEN);
    uint16_t dataCrc = 0;
    if (payloadLen > 0 && payload) {
        dataCrc = crc16_modbus(payload, payloadLen);
    }

    // 3) 先发送 header + 头CRC（23字节）
    uint8_t headBlock[PLATFORM_HEADER_LEN + 2];
    memcpy(headBlock, header, PLATFORM_HEADER_LEN);
    headBlock[PLATFORM_HEADER_LEN]     = (uint8_t)(headCrc >> 8);
    headBlock[PLATFORM_HEADER_LEN + 1] = (uint8_t)(headCrc & 0xFF);
    mipSendHex(headBlock, sizeof(headBlock));

    // 4) 分块发送 payload（如有）
    if (payloadLen > 0 && payload) {
        mipSendHex(payload, payloadLen);
        // 5) 发送payload CRC（大端）
        uint8_t dcrc_be[2] = { (uint8_t)(dataCrc >> 8), (uint8_t)(dataCrc & 0xFF) };
        mipSendHex(dcrc_be, 2);
    }
}

void sendHeartbeat() {
    sendPlatformPacket('R', CMD_HEARTBEAT_REQ, 0, nullptr, 0);
}

// year字段2字节，高位在前，payload长度14
void sendRealtimeMonitorData(
    uint16_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    uint8_t dataFmt,
    const uint8_t* exceptionStatus,
    uint8_t waterStatus
) {
    uint8_t payload[14] = {0};
    payload[0] = (uint8_t)(year >> 8);     // 高字节
    payload[1] = (uint8_t)(year & 0xFF);   // 低字节
    payload[2] = month;
    payload[3] = day;
    payload[4] = hour;
    payload[5] = minute;
    payload[6] = second;
    payload[7] = dataFmt;
    payload[8] = exceptionStatus ? exceptionStatus[0] : 0; // 只用1字节
    // payload[9-12] 默认0
    payload[13] = waterStatus;
    sendPlatformPacket('R', 0x1d00, 0, payload, sizeof(payload));
}

void sendMonitorEventUpload(
    uint16_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    uint8_t triggerCond,
    float realtimeValue,
    float thresholdValue,
    const uint8_t* imageData,
    uint32_t imageLen
) {
    if (imageLen > 65000) imageLen = 65000;
    uint32_t totalLen = 20 + imageLen; // year占2字节
    uint8_t* payload = (uint8_t*)malloc(totalLen);
    if (!payload) return;

    payload[0] = (uint8_t)(year >> 8);
    payload[1] = (uint8_t)(year & 0xFF);
    payload[2] = month;
    payload[3] = day;
    payload[4] = hour;
    payload[5] = minute;
    payload[6] = second;
    payload[7] = triggerCond;
    memcpy(payload + 8, &realtimeValue, 4);
    memcpy(payload + 12, &thresholdValue, 4);
    // ---- 大端序的imageLen ----
    payload[16] = (uint8_t)((imageLen >> 24) & 0xFF);
    payload[17] = (uint8_t)((imageLen >> 16) & 0xFF);
    payload[18] = (uint8_t)((imageLen >> 8) & 0xFF);
    payload[19] = (uint8_t)(imageLen & 0xFF);
    // -------------------------
    if (imageLen > 0 && imageData) {
        memcpy(payload + 20, imageData, imageLen);
    }
    sendPlatformPacket('R', 0x1d09, 0, payload, (uint16_t)totalLen);
    free(payload);
}

void sendTimeSyncRequest() 
{
    sendPlatformPacket('R', CMD_TIME_SYNC_REQ, 0, nullptr, 0);
}

void sendStartupStatusReport
(
    uint16_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    int32_t statusWord
) 
{
    uint8_t payload[37] = {0};
    payload[0] = (uint8_t)(year >> 8);
    payload[1] = (uint8_t)(year & 0xFF);
    payload[2] = month;
    payload[3] = day;
    payload[4] = hour;
    payload[5] = minute;
    payload[6] = second;
    payload[7]  = (uint8_t)((statusWord >> 24) & 0xFF);
    payload[8]  = (uint8_t)((statusWord >> 16) & 0xFF);
    payload[9]  = (uint8_t)((statusWord >> 8) & 0xFF);
    payload[10] = (uint8_t)(statusWord & 0xFF);
    payload[11] = SW_VER_HIGH;
    payload[12] = SW_VER_MID;
    payload[13] = SW_VER_LOW;
    payload[14] = HW_VER_HIGH;
    payload[15] = HW_VER_MID;
    payload[16] = HW_VER_LOW;

    // 产品型号ascii拷贝，最多19字节，第20字节留0
    const char* model = PRODUCT_MODEL_STR;
    size_t len = strlen(model);
    if (len > 19) len = 19;
    memcpy(payload + 17, model, len);
    payload[17 + len] = 0; // 保证结尾0

    sendPlatformPacket('R', 0x0002, 0, payload, sizeof(payload));
}

void sendSimInfoUpload(
    uint16_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    const char* iccid, uint8_t iccid_len,
    const char* imsi,  uint8_t imsi_len,
    uint8_t signal
) {
    log2("[PKT] sendSimInfoUpload called.");
    if (iccid_len > 20) iccid_len = 20;
    if (imsi_len > 15) imsi_len = 15;

    uint8_t payload[7 + 1 + 20 + 1 + 15 + 1]; // 足够大
    payload[0] = (uint8_t)(year >> 8);
    payload[1] = (uint8_t)(year & 0xFF);
    payload[2] = month;
    payload[3] = day;
    payload[4] = hour;
    payload[5] = minute;
    payload[6] = second;
    payload[7] = iccid_len;
    memcpy(payload + 8, iccid, iccid_len);
    payload[8 + iccid_len] = imsi_len;
    memcpy(payload + 8 + iccid_len + 1, imsi, imsi_len);
    payload[8 + iccid_len + 1 + imsi_len] = signal;
    uint16_t paylen = 8 + iccid_len + 1 + imsi_len + 1;
    sendPlatformPacket('R', 0x0007, 0, payload, paylen);
}