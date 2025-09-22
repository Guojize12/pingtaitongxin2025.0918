#pragma once
#include <stdint.h>  // for uint32_t
#include <stddef.h>  // for size_t

#define SERVER_IP   "47.104.5.75"
#define SERVER_PORT 9909
#define TX2         15
#define RX2         13

#define DTU_BAUD    115200
#define LOG_BAUD    115200

#define PLATFORM_VER        0x5b
#define PLATFORM_DMODEL     0x1d
#define CMD_HEARTBEAT_REQ   0x0000
#define CMD_TIME_SYNC_REQ   0x0001

static char g_device_sn[13] = "000000065531";

// Time constants (ms)
static const uint32_t AT_TIMEOUT_MS        = 3000;
static const uint32_t REG_TIMEOUT_MS       = 8000;
static const uint32_t OPEN_TIMEOUT_MS      = 15000;
static const uint32_t STATE_POLL_MS        = 10000;
static const uint32_t HEARTBEAT_INTERVAL_MS = 30000;
static const uint32_t BACKOFF_MAX_MS = 30000;
static const uint32_t REALTIME_UPLOAD_INTERVAL_MS = 30000;

// RTC校时周期（10分钟）
static const uint32_t TIME_SYNC_INTERVAL_MS = 600000; // 10min

static const size_t LINE_BUF_MAX = 512;

extern volatile int g_monitorEventUploadFlag;

#define SW_VER_HIGH  1
#define SW_VER_MID   1
#define SW_VER_LOW   1

#define HW_VER_HIGH  1
#define HW_VER_MID   1
#define HW_VER_LOW   1

// 20字节产品型号，末尾补0
#define PRODUCT_MODEL_STR "MODEL-TEST"