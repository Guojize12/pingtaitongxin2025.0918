#pragma once
#include <stdint.h>  // for uint32_t
#include <stddef.h>  // for size_t
#include <Arduino.h>
#include "esp_camera.h"

// ================== 现有配置 ==================
#define ENABLE_LOG2 0 // 日志和串口2总开关，0为全部屏蔽，1为全部开启

#define SERVER_IP   "47.104.5.75"
#define SERVER_PORT 9909

#if ENABLE_LOG2
#define TX2         15
#define RX2         13
#endif

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
static const uint32_t HEARTBEAT_INTERVAL_MS = 60000;
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


// ===== 异步SD写与内存池（务必放在最前，确保其它头文件引用这些宏时已定义）=====
#ifndef ASYNC_SD_ENABLE
#define ASYNC_SD_ENABLE 1
#endif

#ifndef ASYNC_SD_POOL_BLOCK_SIZE
#define ASYNC_SD_POOL_BLOCK_SIZE (256 * 1024)
#endif

#ifndef ASYNC_SD_POOL_BLOCKS
#define ASYNC_SD_POOL_BLOCKS 3
#endif

#ifndef ASYNC_SD_QUEUE_LENGTH
#define ASYNC_SD_QUEUE_LENGTH 8
#endif

#ifndef ASYNC_SD_TASK_STACK
#define ASYNC_SD_TASK_STACK 4096
#endif

#ifndef ASYNC_SD_TASK_PRIO
#define ASYNC_SD_TASK_PRIO 3
#endif

#ifndef ASYNC_SD_MAX_PATH
#define ASYNC_SD_MAX_PATH 96
#endif

#ifndef ASYNC_SD_SUBMIT_TIMEOUT_MS
#define ASYNC_SD_SUBMIT_TIMEOUT_MS 50
#endif

#ifndef ASYNC_SD_FLUSH_TIMEOUT_MS
#define ASYNC_SD_FLUSH_TIMEOUT_MS 5000
#endif
// ===== 异步SD写与内存池 END =====

// === 开关 ===
#define UPGRADE_ENABLE 1

// === 基本参数 ===
#define SERIAL_BAUD                    115200
#define DEFAULT_FLASH_DUTY             80
#define FRAME_SIZE_PREF                FRAMESIZE_SVGA
#define FRAME_SIZE_FALLBACK            FRAMESIZE_VGA
#define JPEG_QUALITY_PREF              15
#define JPEG_QUALITY_FALLBACK          20
#define INIT_RETRY_PER_CONFIG          3
#define RUNTIME_FAIL_REINIT_THRESHOLD  3
#define CAPTURE_FAIL_REBOOT_THRESHOLD  10
#define SD_FAIL_REBOOT_THRESHOLD       10
#define HEAP_MIN_REBOOT                10000
#define SD_MIN_FREE_MB                 5
#define DISCARD_FRAMES_ON_START        3
#define DISCARD_FRAMES_EACH_SHOT       0
#define ENABLE_AUTO_REINIT             1
#define ENABLE_STATS_LOG               1
#define ENABLE_FRAME_HEADER            0
#define ENABLE_ASYNC_SD_WRITE          0
#define SAVE_PARAMS_INTERVAL_IMAGES    50
#define DEFAULT_SEND_BEFORE_SAVE       1
#define FLASH_MODE                     1
#define FLASH_WARM_MS                  60
#define FLASH_ON_TIME_MS_DIGITAL       40
#define HEAP_WARN_THRESHOLD            16000
#define HEAP_LARGEST_BLOCK_WARN        12000
static constexpr uint32_t NVS_MIN_SAVE_INTERVAL_MS = 60UL * 1000UL;

#ifndef PROTO_MIN_SEND_INTERVAL_MS
#define PROTO_MIN_SEND_INTERVAL_MS 150  // 最小发送间隔(ms)，设置为0可完全关闭节流
#endif

// === 引脚 ===
#define PWDN_GPIO 32
#define RESET_GPIO -1
#define XCLK_GPIO 0
#define SIOD_GPIO 26
#define SIOC_GPIO 27
#define Y9_GPIO 35
#define Y8_GPIO 34
#define Y7_GPIO 39
#define Y6_GPIO 36
#define Y5_GPIO 21
#define Y4_GPIO 19
#define Y3_GPIO 18
#define Y2_GPIO 5
#define VSYNC_GPIO 25
#define HREF_GPIO 23
#define PCLK_GPIO 22

// SD Pins
#define SD_CS   13
#define SD_SCK  14
#define SD_MISO 2
#define SD_MOSI 15

// Flash
#define FLASH_PIN 4
#if FLASH_MODE
  #define FLASH_FREQ_HZ      5000
  #define FLASH_RES_BITS     8
  #define FLASH_TIMER        LEDC_TIMER_1
  #define FLASH_SPEED_MODE   LEDC_LOW_SPEED_MODE
  #define FLASH_CHANNEL      LEDC_CHANNEL_4
#endif

// Button
#define BUTTON_PIN 12
#define TRIGGER_BUTTON 1

typedef struct {
    bool saveEnabled;
    bool sendEnabled;
    bool asyncSDWrite;
} RuntimeConfig;

extern RuntimeConfig g_cfg;

typedef struct {
    uint32_t total_captures;
    uint32_t last_frame_size;
    uint32_t last_capture_ms;
    uint32_t consecutive_capture_fail;
    uint32_t consecutive_sd_fail;
} RunStats;

extern RunStats g_stats;

// 拍照返回码
#define CR_OK 0
#define CR_CAMERA_NOT_READY 1
#define CR_FRAME_GRAB_FAIL 2
#define CR_SD_SAVE_FAIL 3

// 升级相关占位
typedef struct {
    int state;
} UpgradeState;
#define UPG_DOWNLOADING 1
#define UPG_FILE_INFO 2
extern UpgradeState g_upg;
extern bool g_debugMode;

// 占位失败处理函数
inline void handle_camera_failure() {}
inline void handle_sd_failure() {}

// 禁用异步SD写
#undef ENABLE_ASYNC_SD_WRITE
#define ENABLE_ASYNC_SD_WRITE 0
#undef ASYNC_SD_ENABLE
#define ASYNC_SD_ENABLE 0
// 禁用FLASH_MODE，强制数字IO闪光
#undef FLASH_MODE
#define FLASH_MODE 0