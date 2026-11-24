#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <driver/gpio.h>
#include <vector>
#include <set>
#include <map>
#include <memory>
#include <esp_heap_caps.h>
#include <esp_netif.h>
#include <esp_pm.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_http_client.h>
#include <esp_timer.h>
#include <esp_crt_bundle.h>
#include "spi_eth_modem/spi_eth_modem.h"


// HTTP 测速开关：1 = 执行对应测试，0 = 跳过（测试函数保留，随时可打开）
// 下载测速：GET 拉取 speedtest.bin 统计下行速度
// 当前只跑 HTTP 下载测速
#define USE_HTTP_DOWNLOAD_TEST 1
// 上传测速：流式 POST 上传 2MB 到调试服务器。
// 注意：该服务器在上传结束后会报错（预期行为），只统计上传 2MB 的耗时与速度，
//       响应状态码仅打印、不当作失败。
#define USE_HTTP_UPLOAD_TEST 1
// AT 命令通道测试：通过 SFDT type=AT 帧访问 EC718 AT 引擎（只读命令）
#define USE_AT_COMMAND_TEST 1

static const char *TAG = "HTTPS_DEMO";

// GPIO 配置
static constexpr gpio_num_t RESET_BUTTON_PIN = GPIO_NUM_1;  // 重启按钮引脚
static constexpr gpio_num_t MODEM_RI_PIN = GPIO_NUM_7;    // EC718 SRDY -> ESP32 input (light-sleep wake source)
static constexpr gpio_num_t MODEM_DTR_PIN = GPIO_NUM_10;  // ESP32 MRDY -> EC718 input

// ---- EC718（4G 模组）复位控制 ----
// GPIO1 归还给芯片重启按钮（RESET_BUTTON_PIN），从机复位改用 GPIO0。
// 选 GPIO0 的依据：
//   - 不是 strapping pin（ESP32-C3 的 strapping 是 GPIO2/GPIO8/GPIO9）
//   - 不是 flash 引脚（GPIO12-17）、不是 USB（GPIO18/19）、不是 VDD_SPI（GPIO11）
//   - 不是 UART0（GPIO20/21），保留 UART0 下载/日志的余地
//   - 本工程完全未使用（SPI 用 3/4/5/6，SRDY 用 7，MRDY 用 10，按钮用 1）
//   - GPIO1 已用作重启按钮，说明板上没有把 GPIO0/1 占用为 32.768kHz 晶振
// GPIO0 拉低 = 从机保持复位；拉高 = 从机开始启动。
// 主机每次重启都硬复位从机，让从机 WAN 侧从干净状态开始（清掉旧 TCP 流/NAT 会话）。
static constexpr gpio_num_t SLAVE_RESET_PIN = GPIO_NUM_0;
// EC718 复位时序：复位脚拉低 10ms，拉高后等待 1000ms，再走 SPI/AT 初始化流程
static constexpr uint32_t SLAVE_RESET_LOW_MS = 10;      // 复位低电平脉宽
static constexpr uint32_t SLAVE_RESET_SETTLE_MS = 1000; // 释放复位后的稳定等待

// app_main 最先调用：把 GPIO0 配成输出并拉低，从机在主机整个启动期间保持复位
static void SlaveResetHold() {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << SLAVE_RESET_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(SLAVE_RESET_PIN, 0);
    // 复位低脉宽 10ms（EC718 复位脚时序），释放后的 1000ms 稳定等待在
    // SlaveResetRelease() 里完成
    vTaskDelay(pdMS_TO_TICKS(SLAVE_RESET_LOW_MS));
    ESP_LOGI(TAG, "EC718 slave held in reset (GPIO%d low, %lu ms pulse)",
             (int)SLAVE_RESET_PIN, (unsigned long)SLAVE_RESET_LOW_MS);
}

// 4G 流程开始前调用：拉高释放从机复位，等 1000ms 复位延时后立即启动 SPI/AT 流程
// （从机启动期间由 AT 检测重试覆盖，不再固定等 10 秒）
static void SlaveResetRelease() {
    gpio_set_level(SLAVE_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(SLAVE_RESET_SETTLE_MS));
    ESP_LOGI(TAG, "EC718 slave reset released (GPIO%d high, %lu ms settle), starting SFDT/AT flow",
             (int)SLAVE_RESET_PIN, (unsigned long)SLAVE_RESET_SETTLE_MS);
}

// 4G Modem 全局变量
static std::unique_ptr<SpiEthModem> g_modem;

/**
 * GPIO1 低电平中断处理函数
 * 触发后立即重启系统
 */
static void IRAM_ATTR gpio_reset_isr_handler(void* arg) {
    // 防风暴自保险：先拔掉本引脚中断再重启，防止电平/快速沿在 esp_restart
    // 执行期间嵌套触发造成中断看门狗复位 (Interrupt wdt timeout)。
    gpio_intr_disable(RESET_BUTTON_PIN);
    // 在中断中直接调用软重启
    esp_restart();
}

/**
 * 初始化重启按钮 GPIO
 * 配置 GPIO1 为低电平触发中断，并支持低功耗唤醒
 */
static void InitResetButton() {
    ESP_LOGI(TAG, "Initializing reset button on GPIO%d...", RESET_BUTTON_PIN);
    
    // 配置 GPIO1 为输入，启用上拉电阻
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RESET_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // 按下(下降沿)触发一次；电平触发在 GPIO 被外部长期拉低时会形成中断风暴
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // 添加中断处理
    ESP_ERROR_CHECK(gpio_isr_handler_add(RESET_BUTTON_PIN, gpio_reset_isr_handler, nullptr));
    
    // 配置为低功耗唤醒源（支持 light sleep）
    ESP_ERROR_CHECK(gpio_wakeup_enable(RESET_BUTTON_PIN, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    
    ESP_LOGI(TAG, "Reset button initialized, low level will trigger system restart");
}

// Event bits for 4G network initialization
static constexpr uint32_t kEventNetworkConnected = (1 << 0);
static constexpr uint32_t kEventNetworkError = (1 << 1);

/**
 * 初始化4G网卡并等待网络就绪
 * 使用 event group 等待网络连接或错误事件
 */
static bool Init4GNetwork() {
    ESP_LOGI(TAG, "Initializing 4G Network...");
    
    esp_netif_init();
    esp_event_loop_create_default();
    
    SpiEthModem::Config config = {
        .spi_host = SPI2_HOST,
        .sclk_pin = GPIO_NUM_4,
        .mosi_pin = GPIO_NUM_6,   // -> EC718 SPI0 MISO input
        .miso_pin = GPIO_NUM_5,   // <- EC718 SPI0 MOSI output
        .cs_pin = GPIO_NUM_3,     // ESP32 GPIO3 -> EC718 SPI SS 输入（低有效，硬件自动按事务拉低）
        .mrdy_pin = MODEM_DTR_PIN,
        .srdy_pin = MODEM_RI_PIN,
        .clock_speed_hz = 16000000,
    };
    
    g_modem = std::make_unique<SpiEthModem>(config);
    //g_modem->SetDebug(true);  // 2-board joint debug: dump every TX/RX window

    // Create event group for waiting network events
    EventGroupHandle_t network_event_group = xEventGroupCreate();
    if (network_event_group == nullptr) {
        ESP_LOGE(TAG, "Failed to create event group");
        g_modem.reset();
        return false;
    }

    // Set callback to notify event group on network events
    g_modem->SetNetworkEventCallback([network_event_group](SpiEthModem::SpiEthModemEvent event) {
        ESP_LOGI(TAG, "Network event: %s", SpiEthModem::GetNetworkEventName(event));
        
        switch (event) {
            case SpiEthModem::SpiEthModemEvent::Connected:
                xEventGroupSetBits(network_event_group, kEventNetworkConnected);
                break;
            case SpiEthModem::SpiEthModemEvent::ErrorInitFailed:
            case SpiEthModem::SpiEthModemEvent::ErrorNoSim:
                xEventGroupSetBits(network_event_group, kEventNetworkError);
                break;
            case SpiEthModem::SpiEthModemEvent::Connecting:
            case SpiEthModem::SpiEthModemEvent::Disconnected:
                // These are informational, no action needed
                break;
        }
    });
    
    esp_err_t ret = g_modem->Start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start modem: %s", esp_err_to_name(ret));
        vEventGroupDelete(network_event_group);
        g_modem.reset();
        return false;
    }
    
    ESP_LOGI(TAG, "Waiting for network ready...");
    
    // Wait for either Connected or Error event with 180s timeout
    // (SPIC 握手最长 60s + AT 初始化: SIM 检查 10s + 注册等待 60s + 拨号)
    EventBits_t bits = xEventGroupWaitBits(
        network_event_group,
        kEventNetworkConnected | kEventNetworkError,
        pdTRUE,   // Clear bits on exit
        pdFALSE,  // Wait for any bit (OR)
        pdMS_TO_TICKS(180000)
    );
    
    if (bits & kEventNetworkError) {
        ESP_LOGE(TAG, "Network initialization failed with error");
        g_modem->Stop();
        g_modem.reset();
        return false;
    }
    
    if (!(bits & kEventNetworkConnected)) {
        ESP_LOGE(TAG, "Network connection timeout");
        g_modem->Stop();
        g_modem.reset();
        return false;
    }
    
    ESP_LOGI(TAG, "EC718 SFDT SPI slave netif ready: %p", g_modem->GetNetif());
    ESP_LOGI(TAG, "4G Network ready!");
    return true;
}

/**
 * HTTP 客户端下载测试
 * 使用 ESP-IDF 自带的 esp_http_client 下载文件并打印速度
 */
static void TestHttpClientDownload(const char* url) {
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "=== HTTP Client Download Speed Test ===");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "URL: %s", url);

    // Configure HTTP client
    esp_http_client_config_t config = {};
    config.url = url;
    config.timeout_ms = 30000;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.buffer_size = 4096;
    config.buffer_size_tx = 1024;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return;
    }

    // Open connection
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return;
    }

    // Fetch headers
    int content_length = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP Status: %d, Content-Length: %d", status_code, content_length);

    if (status_code != 200) {
        ESP_LOGE(TAG, "HTTP request failed with status %d", status_code);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }

    // Allocate read buffer
    static constexpr size_t kReadBufferSize = 20*1024;
    uint8_t* buffer = static_cast<uint8_t*>(malloc(kReadBufferSize));
    if (buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate read buffer");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }

    // Start downloading and measure speed
    int64_t start_time = esp_timer_get_time();
    int64_t last_report_time = start_time;
    size_t total_bytes = 0;
    size_t bytes_since_last_report = 0;
    int read_len;
    bool first_read_reported = false;

    ESP_LOGI(TAG, "Starting download...");

    while ((read_len = esp_http_client_read(client, reinterpret_cast<char*>(buffer), kReadBufferSize)) > 0) {
        if (!first_read_reported) {
            first_read_reported = true;
            ESP_LOGI(TAG, "First read returned %d bytes at %.3f s since download start", read_len,
                     (esp_timer_get_time() - start_time) / 1000000.0f);
        }
        total_bytes += read_len;
        bytes_since_last_report += read_len;

        int64_t now = esp_timer_get_time();
        int64_t elapsed_since_report = now - last_report_time;

        // Report every second
        if (elapsed_since_report >= 1000000) {
            float instant_speed_bytes_per_sec = (bytes_since_last_report * 1000000.0f) / elapsed_since_report;  // bytes/s
            float overall_speed_bytes_per_sec = (total_bytes * 1000000.0f) / (now - start_time);  // bytes/s

            ESP_LOGI(TAG, "Downloaded: %zu bytes (%.1f KB), Instant: %.1f KB/s (%.0f B/s), Average: %.1f KB/s (%.0f B/s)",
                     total_bytes, total_bytes / 1024.0f, 
                     instant_speed_bytes_per_sec / 1024.0f, instant_speed_bytes_per_sec,
                     overall_speed_bytes_per_sec / 1024.0f, overall_speed_bytes_per_sec);

            last_report_time = now;
            bytes_since_last_report = 0;
        }
    }

    int64_t end_time = esp_timer_get_time();
    float elapsed_sec = (end_time - start_time) / 1000000.0f;
    float speed_bytes_per_sec = total_bytes / elapsed_sec;  // bytes/s
    float speed_kb_per_sec = speed_bytes_per_sec / 1024.0f;  // KB/s
    float speed_mb_per_sec = speed_bytes_per_sec / (1024.0f * 1024.0f);  // MB/s

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "=== Download Complete ===");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Total bytes: %zu (%.2f KB, %.2f MB)", total_bytes, total_bytes / 1024.0f, total_bytes / (1024.0f * 1024.0f));
    ESP_LOGI(TAG, "Time elapsed: %.2f seconds", elapsed_sec);
    ESP_LOGI(TAG, "Average speed: %.0f B/s (%.2f KB/s, %.2f MB/s)", speed_bytes_per_sec, speed_kb_per_sec, speed_mb_per_sec);

    if (read_len < 0) {
        ESP_LOGW(TAG, "Read error occurred at the end: %d", read_len);
    }

    // Cleanup
    free(buffer);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    ESP_LOGI(TAG, "HTTP download test completed");
}

/**
 * HTTP 客户端上传测试
 * 使用 esp_http_client 流式 POST 上传固定大小的数据并打印速度。
 * 流程：open(声明 Content-Length) -> 循环 write 写满 body -> fetch_headers 读响应 -> close。
 * 计时只覆盖 body 写入阶段（第一笔 write 前到最后一笔 write 后）。
 * 注意：测试服务器在上传结束后会返回错误（预期行为），只记录响应，不当作失败。
 */
static void TestHttpClientUpload(const char* url) {
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "=== HTTP Client Upload Speed Test ===");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "URL: %s", url);

    // 上传总量 2MB
    static constexpr size_t kUploadTotalSize = 2 * 1024 * 1024;

    // Configure HTTP client
    esp_http_client_config_t config = {};
    config.url = url;
    config.timeout_ms = 30000;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.buffer_size = 4096;
    config.buffer_size_tx = 1024;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return;
    }

    // POST 方法 + 二进制内容类型（默认是 GET，必须显式设置）
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/octet-stream");

    // Open：连接并发送请求行 + 头（Content-Length = kUploadTotalSize），不发送 body
    esp_err_t err = esp_http_client_open(client, static_cast<int>(kUploadTotalSize));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return;
    }

    // Allocate upload buffer, 8KB 填充固定模式
    static constexpr size_t kChunkSize = 8 * 1024;
    uint8_t* buffer = static_cast<uint8_t*>(malloc(kChunkSize));
    if (buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate upload buffer");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }
    for (size_t i = 0; i < kChunkSize; i++) {
        buffer[i] = static_cast<uint8_t>(i & 0xFF);
    }

    // Start uploading and measure speed (body 写入阶段)
    int64_t start_time = esp_timer_get_time();
    int64_t last_report_time = start_time;
    size_t total_bytes = 0;
    size_t bytes_since_last_report = 0;
    bool write_failed = false;

    ESP_LOGI(TAG, "Starting upload of %zu bytes (%.2f MB)...", kUploadTotalSize, kUploadTotalSize / (1024.0f * 1024.0f));

    while (total_bytes < kUploadTotalSize) {
        size_t to_write = kChunkSize;
        if (total_bytes + to_write > kUploadTotalSize) {
            to_write = kUploadTotalSize - total_bytes;
        }
        // 同步模式下 esp_http_client_write 内部循环写满，要么返回 to_write 要么 <=0 报错
        int write_len = esp_http_client_write(client, reinterpret_cast<const char*>(buffer), static_cast<int>(to_write));
        if (write_len <= 0) {
            ESP_LOGE(TAG, "Upload write failed after %zu bytes: %d", total_bytes, write_len);
            write_failed = true;
            break;
        }
        total_bytes += write_len;
        bytes_since_last_report += write_len;

        int64_t now = esp_timer_get_time();
        int64_t elapsed_since_report = now - last_report_time;

        // Report every second
        if (elapsed_since_report >= 1000000) {
            float instant_speed_bytes_per_sec = (bytes_since_last_report * 1000000.0f) / elapsed_since_report;  // bytes/s
            float overall_speed_bytes_per_sec = (total_bytes * 1000000.0f) / (now - start_time);  // bytes/s

            ESP_LOGI(TAG, "Uploaded: %zu bytes (%.1f KB), Instant: %.1f KB/s (%.0f B/s), Average: %.1f KB/s (%.0f B/s)",
                     total_bytes, total_bytes / 1024.0f,
                     instant_speed_bytes_per_sec / 1024.0f, instant_speed_bytes_per_sec,
                     overall_speed_bytes_per_sec / 1024.0f, overall_speed_bytes_per_sec);

            last_report_time = now;
            bytes_since_last_report = 0;
        }
    }

    int64_t end_time = esp_timer_get_time();
    float elapsed_sec = (end_time - start_time) / 1000000.0f;
    float speed_bytes_per_sec = (elapsed_sec > 0.001f) ? (total_bytes / elapsed_sec) : 0.0f;  // bytes/s
    float speed_kb_per_sec = speed_bytes_per_sec / 1024.0f;  // KB/s
    float speed_mb_per_sec = speed_bytes_per_sec / (1024.0f * 1024.0f);  // MB/s

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "=== Upload Complete ===");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Total bytes: %zu / %zu (%.2f KB, %.2f MB)", total_bytes, kUploadTotalSize,
             total_bytes / 1024.0f, total_bytes / (1024.0f * 1024.0f));
    ESP_LOGI(TAG, "Time elapsed: %.2f seconds", elapsed_sec);
    ESP_LOGI(TAG, "Average speed: %.0f B/s (%.2f KB/s, %.2f MB/s)", speed_bytes_per_sec, speed_kb_per_sec, speed_mb_per_sec);

    if (write_failed) {
        ESP_LOGW(TAG, "Upload incomplete: %zu / %zu bytes sent", total_bytes, kUploadTotalSize);
    }

    // 读响应：服务器上传结束后会报错（预期行为），只记录状态码，不校验、不当作失败
    int64_t hdr_len = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "Server response after upload: status %d, content length %lld (errors expected and ignored)",
             status_code, (long long)hdr_len);

    // Cleanup
    free(buffer);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    ESP_LOGI(TAG, "HTTP upload test completed");
}


// ============================================================================
// AT 命令通道测试（SFDT type=AT）
// ============================================================================
// 通过 SFDT 帧头新增的 type 字段（0=Ethernet / 1=AT）把 AT 命令发给 EC718 的
// AT 引擎，响应以 AT 帧回传。验证链路：SendAt -> TX 队列(type=AT) -> 从机
// atRilAtCmdReq -> 响应逐行回传 -> HandleAtPayload 累积 -> OK/ERROR 判定。
static void TestAtCommands() {
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "=== AT Command Channel Test (SFDT type=AT) ===");
    ESP_LOGI(TAG, "===========================================");

    if (!g_modem) {
        ESP_LOGE(TAG, "Modem not initialized, skip AT test");
        return;
    }

    // 只发只读命令，避免改动从机状态
    static const char* const kCmds[] = {
        "AT+CGMR",  // 固件版本
        "AT+CGSN",  // IMEI
    };

    for (const char* cmd : kCmds) {
        std::string resp;
        int64_t t0 = esp_timer_get_time();
        esp_err_t ret = g_modem->SendAt(cmd, resp, 3000);
        int64_t dt_ms = (esp_timer_get_time() - t0) / 1000;

        // CR/LF 转义后单行打印，便于日志检索
        std::string flat;
        flat.reserve(resp.size());
        for (char c : resp) {
            switch (c) {
                case '\r': flat += "\\r"; break;
                case '\n': flat += "\\n"; break;
                case '\0': flat += "\\0"; break;
                default:   flat += c;     break;
            }
        }

        ESP_LOGI(TAG, "AT %s -> %s (%lld ms)", cmd, esp_err_to_name(ret), (long long)dt_ms);
        ESP_LOGI(TAG, "  response: %s", flat.c_str());
    }

    ESP_LOGI(TAG, "=== AT Command Channel Test Done ===");
}



extern "C" void app_main(void) {
    // GPIO0 = EC718 从机复位控制线（拉低复位 / 拉高启动）；GPIO1 归还给重启按钮
    ESP_LOGI(TAG, "HTTP Demo Starting... GPIO0 = EC718 slave reset, GPIO1 = reset button");

    // 主机一上电就把从机按在复位里（避免复位线浮空窗口期从机自行启动）
    SlaveResetHold();

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM);

    InitResetButton();

    // ========== 4G 模式 ==========
    ESP_LOGI(TAG, "Running in 4G mode");

    // 拉高释放从机复位，等从机启动完成（~10s）再跑 SPI 握手/网络流程
    SlaveResetRelease();
    
    if (!Init4GNetwork()) {
        ESP_LOGE(TAG, "Failed to initialize 4G network");
        return;
    }

    // esp_pm_config_t pm_config = {
    //     .max_freq_mhz = 240,
    //     .min_freq_mhz = 80,
    //     .light_sleep_enable = true,
    // };
    // ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    // AT 命令通道测试（SFDT type=AT：AT 帧与以太网帧在同一 SPI 链路上分流）
#if USE_AT_COMMAND_TEST
    TestAtCommands();
#endif
    
    // HTTP 下载速度测试
#if USE_HTTP_DOWNLOAD_TEST
    const char* test_url = "http://";
    TestHttpClientDownload(test_url);
#endif

    // HTTP 上传速度测试
#if USE_HTTP_UPLOAD_TEST
    TestHttpClientUpload("http://");
#endif

    ESP_LOGI(TAG, "HTTP Demo completed");


    static char buffer[1024];
    vTaskList(buffer);
    ESP_LOGI(TAG, "Task list:\n%s", buffer);
    
    ESP_LOGI(TAG, "Free SRAM: %d bytes, minimal SRAM: %d bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL), 
        heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
