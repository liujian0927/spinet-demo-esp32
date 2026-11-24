// Copyright 2025 Terrence
// SPDX-License-Identifier: Apache-2.0
//
// SPI Ethernet Modem for EC718 SFDT slave boards (EOS = Ethernet Over SPI).
// The transfer engine is a 1:1 port of EC718's sfdt_master.c
// (project/ec7xx_0h00/ap/apps/spi_2boards/src/sfdt_master.c); the iot_eth /
// esp_netif glue mirrors uart_eth_modem.cc.

#include "spi_eth_modem.h"

#include <cstdio>
#include <cstring>

#include <esp_check.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_sleep.h>
#include <esp_rom_sys.h>
#include <hal/gpio_ll.h>
#include <lwip/dns.h>
#include <lwip/tcpip.h>

// ---------------------------------------------------------------------------
// Frame diagnostics: decode eth/IP/TCP/ARP headers so SPI-boundary traffic can
// be correlated with lwIP TCP behaviour (seq/ack advancement, window, ARP ops).
// ---------------------------------------------------------------------------

namespace {

constexpr const char* kDiagTag = "SpiEthModem";

void LogFrameDiag(const char* dir, const uint8_t* f, size_t len, bool enabled) {
    if (!enabled) return;
    if (len < 14) return;
    uint16_t ethertype = (uint16_t)((f[12] << 8) | f[13]);
    if (ethertype == 0x0800) {
        if (len < 34) return;
        uint8_t ihl = (f[14] & 0x0F) * 4;
        if (ihl < 20 || len < (size_t)(14 + ihl)) return;
        uint8_t proto = f[23];
        uint16_t ipTotal = (uint16_t)((f[16] << 8) | f[17]);
        if (proto == 6) {
            if (len < (size_t)(14 + ihl + 20)) return;
            const uint8_t* t = f + 14 + ihl;
            uint32_t seq = ((uint32_t)t[4] << 24) | ((uint32_t)t[5] << 16) | ((uint32_t)t[6] << 8) | t[7];
            uint32_t ack = ((uint32_t)t[8] << 24) | ((uint32_t)t[9] << 16) | ((uint32_t)t[10] << 8) | t[11];
            uint8_t flags = t[13];
            uint16_t win = (uint16_t)((t[14] << 8) | t[15]);
            uint8_t tcpHdr = (uint8_t)((t[12] >> 4) * 4);
            uint16_t payload = (ipTotal > (uint16_t)(ihl + tcpHdr)) ? (uint16_t)(ipTotal - ihl - tcpHdr) : 0;
            ESP_LOGI(kDiagTag, "TCP_%s seq=%u ack=%u fl=0x%02x len=%u win=%u", dir,
                     (unsigned)seq, (unsigned)ack, flags, (unsigned)payload, (unsigned)win);
        } else if (proto == 1 && len >= (size_t)(14 + ihl + 8)) {
            ESP_LOGI(kDiagTag, "ICMP_%s type=%u code=%u", dir, f[14 + ihl], f[15 + ihl]);
        }
    } else if (ethertype == 0x0806 && len >= 42) {
        uint16_t op = (uint16_t)((f[20] << 8) | f[21]);
        ESP_LOGI(kDiagTag, "ARP_%s op=%u snd=%u.%u.%u.%u tgt=%u.%u.%u.%u", dir, op,
                 f[28], f[29], f[30], f[31], f[38], f[39], f[40], f[41]);
    }
}

}  // namespace

// ---------------------------------------------------------------------------

SpiEthModem::SpiEthModem(const Config& config) : config_(config) {
    // Generate MAC address (mirrors UartEthModem)
    esp_read_mac(mac_addr_, ESP_MAC_ETH);
    mac_addr_[5] ^= 0x01;  // Make unique

    event_group_ = xEventGroupCreate();
    if (!event_group_) {
        ESP_LOGE(kTag, "Failed to create event group");
        abort();
    }

    // Initialize driver structure (mirrors UartEthModem)
    driver_.name = "spi_eth";
    driver_.init = [](iot_eth_driver_t* driver) -> esp_err_t {
        return ESP_OK;
    };
    driver_.deinit = [](iot_eth_driver_t* driver) -> esp_err_t {
        return ESP_OK;
    };
    driver_.start = [](iot_eth_driver_t* driver) -> esp_err_t {
        return ESP_OK;
    };
    driver_.stop = [](iot_eth_driver_t* driver) -> esp_err_t {
        return ESP_OK;
    };
    driver_.transmit = [](iot_eth_driver_t* driver, uint8_t* buf, size_t len) -> esp_err_t {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
        auto* self = reinterpret_cast<SpiEthModem*>(
                reinterpret_cast<char*>(driver) - offsetof(SpiEthModem, driver_));
#pragma GCC diagnostic pop
        if (!self->handshake_done_.load()) {
            return ESP_ERR_INVALID_STATE;
        }
        // Non-blocking: enqueue frame for the transfer task to clock out
        return self->EnqueueTxFrame(buf, len);
    };
    driver_.get_addr = [](iot_eth_driver_t* driver, uint8_t* mac) -> esp_err_t {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
        auto* self = reinterpret_cast<SpiEthModem*>(
                reinterpret_cast<char*>(driver) - offsetof(SpiEthModem, driver_));
#pragma GCC diagnostic pop
        memcpy(mac, self->mac_addr_, 6);
        return ESP_OK;
    };
    driver_.set_mediator = [](iot_eth_driver_t* driver, iot_eth_mediator_t* mediator) -> esp_err_t {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
        auto* self = reinterpret_cast<SpiEthModem*>(
                reinterpret_cast<char*>(driver) - offsetof(SpiEthModem, driver_));
#pragma GCC diagnostic pop
        self->mediator_ = mediator;
        return ESP_OK;
    };
}

SpiEthModem::~SpiEthModem() {
    Stop();

    if (event_group_) {
        vEventGroupDelete(event_group_);
        event_group_ = nullptr;
    }
}

const char* SpiEthModem::GetNetworkEventName(SpiEthModemEvent event) {
    switch (event) {
        case SpiEthModemEvent::Connecting: return "Connecting";
        case SpiEthModemEvent::Connected: return "Connected";
        case SpiEthModemEvent::Disconnected: return "Disconnected";
        case SpiEthModemEvent::ErrorInitFailed: return "ErrorInitFailed";
        case SpiEthModemEvent::ErrorNoSim: return "ErrorNoSim";
        default: return "Unknown";
    }
}

esp_err_t SpiEthModem::Start() {
    ESP_LOGI(kTag, "Starting SpiEthModem...");

    if (started_.load()) {
        ESP_LOGW(kTag, "Already started");
        return ESP_ERR_INVALID_STATE;
    }

    stop_flag_ = false;
    handshake_done_ = false;
    initialized_ = false;
    link_up_ = false;
    seq_no_ = 0;
    srdy_mask_ = 0;
    slave_flow_control_off_ = false;
    xoff_events_ = 0;
    xoff_log_ms_ = 0;
    master_xoff_ = false;
    fc_pending_ = false;
    master_xoff_until_ = 0;
    rx_congest_events_ = 0;
    rx_congest_log_ms_ = 0;
    pending_rx_count_ = 0;
    rx_frame_payload_len_ = 0;

    // Clear stale bits left over from a previous Start()/Stop() cycle
    if (event_group_) {
        xEventGroupClearBits(event_group_, 0x00FFFFFF);
    }

    // Create TX queue (depth mirrors EC718 MASTER_TX_MSGQUEUE_CAPACITY = 32)
    tx_queue_ = xQueueCreate(kTxQueueDepth, sizeof(TxQueueItem));
    if (!tx_queue_) {
        ESP_LOGE(kTag, "Failed to create TX queue");
        return ESP_ERR_NO_MEM;
    }

    // AT queue: independent of the ETH XOFF gate (see TransferTaskRun)
    at_queue_ = xQueueCreate(kAtQueueDepth, sizeof(TxQueueItem));
    if (!at_queue_) {
        ESP_LOGE(kTag, "Failed to create AT queue");
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    // Allocate DMA scratch buffers (must be DMA capable and word aligned)
    window_tx_buffer_ = static_cast<uint8_t*>(heap_caps_malloc(kSfdtFrameMtu, MALLOC_CAP_DMA));
    window_rx_buffer_ = static_cast<uint8_t*>(heap_caps_malloc(kSfdtFrameMtu, MALLOC_CAP_DMA));
    rx_assembly_buffer_ = static_cast<uint8_t*>(malloc(kSfdtFrameMtu));
    if (!window_tx_buffer_ || !window_rx_buffer_ || !rx_assembly_buffer_) {
        ESP_LOGE(kTag, "Failed to allocate window buffers");
        if (window_tx_buffer_) {
            free(window_tx_buffer_);
            window_tx_buffer_ = nullptr;
        }
        if (window_rx_buffer_) {
            free(window_rx_buffer_);
            window_rx_buffer_ = nullptr;
        }
        if (rx_assembly_buffer_) {
            free(rx_assembly_buffer_);
            rx_assembly_buffer_ = nullptr;
        }
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
        vQueueDelete(at_queue_);
        at_queue_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    pending_rx_count_ = 0;
    rx_frame_payload_len_ = 0;

    // Initialize SPI (DMA)
    esp_err_t ret = InitSpi();
    if (ret != ESP_OK) {
        free(window_tx_buffer_);
        window_tx_buffer_ = nullptr;
        free(window_rx_buffer_);
        window_rx_buffer_ = nullptr;
        free(rx_assembly_buffer_);
        rx_assembly_buffer_ = nullptr;
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
        vQueueDelete(at_queue_);
        at_queue_ = nullptr;
        return ret;
    }

    // Initialize handshake GPIOs (MRDY output low, SRDY input + posedge ISR)
    ret = InitGpio();
    if (ret != ESP_OK) {
        DeinitSpi();
        free(window_tx_buffer_);
        window_tx_buffer_ = nullptr;
        free(window_rx_buffer_);
        window_rx_buffer_ = nullptr;
        free(rx_assembly_buffer_);
        rx_assembly_buffer_ = nullptr;
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
        vQueueDelete(at_queue_);
        at_queue_ = nullptr;
        return ret;
    }

    started_.store(true);

    // Transfer task: SFDT master engine.
    // Priority must sit ABOVE the lwIP tcpip task (CONFIG_LWIP_TCPIP_TASK_PRIO=18):
    // the previous frame's tcpip processing used to preempt this task right
    // after the SRDY edge, delaying the next SPI window by ~400 us (measured
    // run130: irq2wake=403 us with xfer prio 12). Starting the DMA first and
    // letting tcpip run during the transfer removes that from the critical
    // path. Kept below the esp_timer daemon (22) so the ISR's deferred event
    // group set still runs promptly.
    if (xTaskCreate([](void* arg) {
            static_cast<SpiEthModem*>(arg)->TransferTaskRun();
            vTaskDelete(nullptr);
        }, "spi_eth_xfer", 6144, this, 21, &transfer_task_) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create transfer task");
        stop_flag_ = true;
        vTaskDelay(pdMS_TO_TICKS(50));
        CleanupResources();
        return ESP_ERR_NO_MEM;
    }

    // Init task: SPIC handshake + iot_eth bring-up
    if (xTaskCreate([](void* arg) {
            static_cast<SpiEthModem*>(arg)->InitTaskRun();
            vTaskDelete(nullptr);
        }, "spi_eth_init", 6144, this, 5, &init_task_) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create init task");
        stop_flag_ = true;
        if (transfer_task_) {
            xTaskNotifyGive(transfer_task_);
        }
        xEventGroupWaitBits(event_group_, kEventTransferTaskDone, pdTRUE, pdFALSE, pdMS_TO_TICKS(3000));
        init_task_ = nullptr;
        CleanupResources();
        return ESP_ERR_NO_MEM;
    }

    // Signal start
    xEventGroupSetBits(event_group_, kEventStart);

    ESP_LOGI(kTag, "SpiEthModem starting asynchronously...");
    return ESP_OK;
}

esp_err_t SpiEthModem::Stop() {
    if (!started_.load()) {
        return ESP_OK;
    }

    ESP_LOGI(kTag, "Stopping SpiEthModem...");

    stop_flag_ = true;

    if (event_group_) {
        xEventGroupSetBits(event_group_, kEventStop);
    }
    if (transfer_task_) {
        xTaskNotifyGive(transfer_task_);
    }

    // Wake tasks blocked in queues
    if (tx_queue_) {
        TxQueueItem dummy = {};
        xQueueSend(tx_queue_, &dummy, 0);
    }
    if (at_queue_) {
        TxQueueItem dummy = {};
        xQueueSend(at_queue_, &dummy, 0);
    }

    // Wait for tasks to finish
    if (event_group_) {
        auto bits = xEventGroupWaitBits(event_group_, kEventAllTasksDone, pdTRUE, pdTRUE,
                                        pdMS_TO_TICKS(10000));
        if (!(bits & kEventAllTasksDone)) {
            ESP_LOGE(kTag, "Timeout waiting for tasks to finish");
        }
    }
    init_task_ = nullptr;
    transfer_task_ = nullptr;

    CleanupResources();

    started_ = false;
    initialized_ = false;
    handshake_done_ = false;

    ESP_LOGI(kTag, "SpiEthModem stopped");
    return ESP_OK;
}

esp_err_t SpiEthModem::SendAt(const std::string& cmd, std::string& response, uint32_t timeout_ms) {
    std::lock_guard<std::mutex> lock(at_mutex_);

    if (stop_flag_.load()) {
        return ESP_ERR_INVALID_STATE;
    }
    // AT traffic only needs the SFDT transfer engine running; it does not
    // require the SPIC netdev handshake (see EnqueueTxFrame).
    if (!started_.load()) {
        ESP_LOGE(kTag, "Failed to send AT command: modem not started");
        return ESP_ERR_INVALID_STATE;
    }

    at_command_response_.clear();
    waiting_for_at_response_ = true;
    xEventGroupClearBits(event_group_, kEventAtResponse);  // Clear any pending

    // AT line must end with CR; an extra NUL keeps the slave-side C-string
    // handling safe (the AT engine takes the length verbatim).
    std::string payload = cmd;
    if (payload.empty() || payload.back() != '\r') {
        payload += '\r';
    }
    payload += '\0';

    if (debug_enabled_.load()) {
        ESP_LOGI(kTag, "AT>>> %s", cmd.c_str());
    }

    esp_err_t ret = EnqueueTxFrame(reinterpret_cast<const uint8_t*>(payload.data()),
                                   payload.size(), FrameType::kAtCommand);
    if (ret != ESP_OK) {
        waiting_for_at_response_ = false;
        ESP_LOGE(kTag, "Failed to enqueue AT command: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for response. The EC718 AT engine delivers one AT frame per
    // response line, so keep accumulating until the final result code
    // (OK / ERROR) shows up or the caller's budget expires.
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    bool gotAny = false;
    while (true) {
        TickType_t now = xTaskGetTickCount();
        TickType_t remain = (now >= deadline) ? 0 : (deadline - now);
        EventBits_t bits = xEventGroupWaitBits(event_group_, kEventAtResponse | kEventStop,
                                               pdTRUE,   // clear on exit
                                               pdFALSE,  // wait for any bit
                                               remain);
        if (bits & kEventStop) {
            waiting_for_at_response_ = false;
            return ESP_ERR_INVALID_STATE;
        }
        if (bits & kEventAtResponse) {
            gotAny = true;
            if (at_command_response_.find("OK") != std::string::npos ||
                at_command_response_.find("ERROR") != std::string::npos) {
                break;
            }
            if (remain == 0) {
                break;  // last chunk arrived at the deadline
            }
            continue;
        }
        break;  // timeout
    }

    // Absorb trailing frames of this response. The EC718 AT engine may deliver
    // the final result code and the remaining lines in separate frames, and a
    // frame from a previously timed-out command can also arrive late. A short
    // quiet window prevents such leftovers from truncating this response or
    // leaking into the next command.
    if (gotAny) {
        for (int i = 0; i < 3; i++) {
            EventBits_t extra = xEventGroupWaitBits(event_group_, kEventAtResponse | kEventStop,
                                                    pdTRUE, pdFALSE, pdMS_TO_TICKS(kAtDrainMs));
            if (!(extra & kEventAtResponse)) {
                break;
            }
        }
    }

    waiting_for_at_response_ = false;
    response = at_command_response_;

    if (!gotAny) {
        ESP_LOGW(kTag, "AT timeout: %s", cmd.c_str());
        return ESP_ERR_TIMEOUT;
    }
    if (response.find("OK") != std::string::npos) {
        return ESP_OK;
    }
    if (response.find("ERROR") != std::string::npos) {
        return ESP_FAIL;
    }
    // Partial response without a final result code within the budget
    return ESP_FAIL;
}

void SpiEthModem::SetNetworkEventCallback(SpiEthModemEventCallback callback) {
    network_event_callback_ = std::move(callback);
}

void SpiEthModem::SetDebug(bool enabled) {
    debug_enabled_.store(enabled);
}

void SpiEthModem::SetNetworkEvent(SpiEthModemEvent event) {
    SpiEthModemEvent old_event = network_event_.exchange(event);
    if (event_group_) {
        xEventGroupSetBits(event_group_, kEventNetworkEventChanged);
    }

    if (old_event != event) {
        ESP_LOGI(kTag, "Network event: %s -> %s", GetNetworkEventName(old_event),
                 GetNetworkEventName(event));
        if (network_event_callback_) {
            network_event_callback_(event);
        }
    }
}

// ---------------------------------------------------------------------------
// Hardware init
// ---------------------------------------------------------------------------

esp_err_t SpiEthModem::InitSpi() {
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = config_.mosi_pin;
    buscfg.miso_io_num = config_.miso_pin;
    buscfg.sclk_io_num = config_.sclk_pin;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = kSfdtFrameMtu + 4;
    buscfg.flags = SPICOMMON_BUSFLAG_MASTER;

    esp_err_t ret = spi_bus_initialize(config_.spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    bus_initialized_ = true;

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = (int)config_.clock_speed_hz;
    // EC718 SFDT_SPI_FRAME_FORMAT = ARM_SPI_CPOL0_CPHA1 -> mode 1
    devcfg.mode = 1;
    devcfg.spics_io_num =
            (config_.cs_pin == GPIO_NUM_NC || config_.cs_pin < 0) ? -1 : ((int)config_.cs_pin);
    devcfg.queue_size = 4;
    devcfg.post_cb = nullptr;   // completion is reaped via spi_device_get_trans_result()

    ret = spi_bus_add_device(config_.spi_host, &devcfg, &spi_dev_);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(config_.spi_host);
        bus_initialized_ = false;
        return ret;
    }

    int actual_khz = 0;
    spi_device_get_actual_freq(spi_dev_, &actual_khz);
    ESP_LOGI(kTag, "SPI master ready: host %d, requested %lu Hz, actual %d kHz, mode %d, cs io %d",
             config_.spi_host, (unsigned long)config_.clock_speed_hz, actual_khz, devcfg.mode,
             (int)config_.cs_pin);
    return ESP_OK;
}

void SpiEthModem::DeinitSpi() {
    if (spi_dev_) {
        spi_bus_remove_device(spi_dev_);
        spi_dev_ = nullptr;
    }
    if (bus_initialized_) {
        spi_bus_free(config_.spi_host);
        bus_initialized_ = false;
    }
}

esp_err_t SpiEthModem::InitGpio() {
    // MRDY pin (output), idle LOW (EC718 master keeps MRDY low between pulses)
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << config_.mrdy_pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    esp_err_t ret = gpio_config(&io_conf);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to configure MRDY pin");

    gpio_set_level(config_.mrdy_pin, 0);
    // Keep MRDY actively driven (low) while the chip is in light sleep:
    // a floating pad could glitch the EC718 slave MRDY wakeup input.
    gpio_sleep_set_direction(config_.mrdy_pin, GPIO_MODE_OUTPUT);
    gpio_sleep_set_pull_mode(config_.mrdy_pin, GPIO_FLOATING);

    // SRDY pin (input, posedge interrupt, idle LOW; pulldown keeps it definable).
    // HARDENED ORDER: the edge IRQ is kept disabled until the per-pin handler
    // is registered, so the gpio_config()..gpio_isr_handler_add() window can
    // never dispatch interrupts with no handler installed.
    gpio_intr_disable(config_.srdy_pin);

    io_conf.pin_bit_mask = (1ULL << config_.srdy_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    ret = gpio_config(&io_conf);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to configure SRDY pin");
    // gpio_config() arms the IRQ for a non-DISABLE type; keep it fused off
    // until the handler is in place.
    gpio_intr_disable(config_.srdy_pin);

    // NOTE: no gpio_wakeup_enable() here. It rewrites the pin's runtime
    // interrupt type to the wake level (gpio.c:663 sets HIGH_LEVEL), turning
    // SRDY into a level-triggered IRQ: while the slave holds SRDY high
    // waiting for clocks, the ISR re-enters continuously (measured 31.8 kHz)
    // instead of once per edge and pegs CPU0. Auto light sleep is
    // intentionally not enabled (comment below), so the wake source is dead
    // code with a live side effect.

    // Install ISR service (main.cc may already have installed one -> ignore that error)
    esp_err_t isr_ret = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(isr_ret, kTag, "Failed to install GPIO ISR service");
    }

    ret = gpio_isr_handler_add(config_.srdy_pin, SrdyIsrHandler, this);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to add ISR handler");

    // Handler registered -> safe to arm the edge IRQ now
    ret = gpio_intr_enable(config_.srdy_pin);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to enable SRDY interrupt");

    // Light-sleep wakeup on SRDY is intentionally NOT configured here (a
    // future explicit esp_light_sleep_start() user must re-add a level-type
    // wakeup around the sleep call; see the NOTE above).
    //
    // AUTO light sleep is intentionally NOT enabled here. With
    // CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y (sdkconfig:1110) the ESP32-C3 USB
    // CDC peripheral drops off the bus during sleep windows: Windows loses
    // the COM port or re-enumerates it as an "unknown USB device (descriptor
    // request failed)". esp_pm only guards UART console TX idleness before
    // sleeping (idf pm_impl.c:963-990); it has no notion of USB CDC, so an
    // auto-sleep window can land in the middle of CDC activity. Note that
    // InitGpio is the sole place enabling esp_pm auto light sleep
    // (CONFIG_PM_ENABLE=y, sdkconfig:1043); re-evaluate auto light sleep only
    // after moving the console to UART0 or designing an explicit sleep
    // strategy around USB CDC.

    return ESP_OK;
}

void SpiEthModem::DeinitGpio() {
    gpio_wakeup_disable(config_.srdy_pin);
    gpio_isr_handler_remove(config_.srdy_pin);
    gpio_sleep_set_direction(config_.mrdy_pin, GPIO_MODE_INPUT);
    gpio_reset_pin(config_.srdy_pin);
    gpio_reset_pin(config_.mrdy_pin);
}

void SpiEthModem::SetMrdy(bool high) {
    gpio_set_level(config_.mrdy_pin, high ? 1 : 0);
}

// ---------------------------------------------------------------------------
// iot_eth / esp_netif glue (mirrors UartEthModem)
// ---------------------------------------------------------------------------

esp_err_t SpiEthModem::InitIotEth() {
    // Install iot_eth driver
    iot_eth_config_t eth_cfg = {
        .driver = &driver_,
        .stack_input = nullptr,
        .user_data = this,
    };

    esp_err_t ret = iot_eth_install(&eth_cfg, &eth_handle_);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to install iot_eth driver");

    // Create netif with GARP disabled (mirrors UartEthModem)
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_inherent_config_t base_cfg = *netif_cfg.base;
    base_cfg.flags = static_cast<esp_netif_flags_t>(base_cfg.flags & ~ESP_NETIF_FLAG_GARP);
    netif_cfg.base = &base_cfg;
    eth_netif_ = esp_netif_new(&netif_cfg);
    if (!eth_netif_) {
        ESP_LOGE(kTag, "Failed to create netif");
        iot_eth_uninstall(eth_handle_);
        eth_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    // Create glue and attach
    glue_ = iot_eth_new_netif_glue(eth_handle_);
    if (!glue_) {
        ESP_LOGE(kTag, "Failed to create netif glue");
        esp_netif_destroy(eth_netif_);
        eth_netif_ = nullptr;
        iot_eth_uninstall(eth_handle_);
        eth_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    esp_netif_attach(eth_netif_, glue_);

    // Start iot_eth
    ret = iot_eth_start(eth_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to start iot_eth");
        iot_eth_del_netif_glue(glue_);
        glue_ = nullptr;
        esp_netif_destroy(eth_netif_);
        eth_netif_ = nullptr;
        iot_eth_uninstall(eth_handle_);
        eth_handle_ = nullptr;
        return ret;
    }

    // Register IP event handler to detect when we get an IP address
    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &IpEventHandler, this,
                                              &ip_event_handler_instance_);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to register IP event handler");
        iot_eth_stop(eth_handle_);
        iot_eth_del_netif_glue(glue_);
        glue_ = nullptr;
        esp_netif_destroy(eth_netif_);
        eth_netif_ = nullptr;
        iot_eth_uninstall(eth_handle_);
        eth_handle_ = nullptr;
        return ret;
    }

    // Notify MAC address is available. The LINK stage is NOT signaled here:
    // it follows the EC718's GEN_STATUS_IND (SPIC_STATUS_MEDIA_CONN / MEDIA_DISCONN),
    // which drives the netif up/down (and DHCP start/stop) at the right time.
    if (mediator_) {
        mediator_->on_stage_changed(mediator_, IOT_ETH_STAGE_GET_MAC, nullptr);
    }

    // Clear DNS cache
    tcpip_callback([](void* arg) -> void {
        dns_clear_cache();
    }, nullptr);

    return ESP_OK;
}

void SpiEthModem::DeinitIotEth() {
    if (ip_event_handler_instance_) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                              ip_event_handler_instance_);
        ip_event_handler_instance_ = nullptr;
    }
    if (eth_handle_) {
        iot_eth_stop(eth_handle_);
    }
    if (glue_) {
        iot_eth_del_netif_glue(glue_);
        glue_ = nullptr;
    }
    if (eth_netif_) {
        // Stop DHCP client first to prevent use-after-free in dhcp_fine_tmr
        esp_netif_dhcpc_stop(eth_netif_);
        esp_netif_destroy(eth_netif_);
        eth_netif_ = nullptr;
    }
    if (eth_handle_) {
        iot_eth_uninstall(eth_handle_);
        eth_handle_ = nullptr;
    }
    mediator_ = nullptr;
}

void SpiEthModem::NotifyLink(bool up) {
    if (!mediator_ || !handshake_done_.load()) {
        return;
    }
    // Ethernet frames are dropped by lwIP while the netif is down (they are
    // not congestion). The link stage event is asynchronous, so the reliable
    // "netif can accept frames" point is GOT_IP (see IpEventHandler); only
    // the down transition is handled here.
    if (!up) {
        link_up_ = false;
    }
    int link_status = up ? IOT_ETH_LINK_UP : IOT_ETH_LINK_DOWN;
    mediator_->on_stage_changed(mediator_, IOT_ETH_STAGE_LINK, &link_status);
    if (!up) {
        SetNetworkEvent(SpiEthModemEvent::Disconnected);
    }
}

void SpiEthModem::IpEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id,
                                 void* event_data) {
    auto* self = static_cast<SpiEthModem*>(arg);

    if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_GOT_IP) {
        // Netif is up and has an address: frames can now be accepted by lwIP,
        // so an input failure from here on really means congestion.
        self->link_up_ = true;
        // Network is ready now
        self->SetNetworkEvent(SpiEthModemEvent::Connected);
        if (self->event_group_) {
            xEventGroupSetBits(self->event_group_, kEventNetworkReady);
        }
    }
}

// ---------------------------------------------------------------------------
// TX path
// ---------------------------------------------------------------------------

esp_err_t SpiEthModem::EnqueueTxFrame(const uint8_t* buf, size_t len) {
    return EnqueueTxFrame(buf, len, FrameType::kEthernet);
}

esp_err_t SpiEthModem::EnqueueTxFrame(const uint8_t* buf, size_t len, FrameType type) {
    // The SPIC handshake only gates the Ethernet data path. The AT channel is
    // independent of it (the slave dispatches type=AT frames straight to its
    // AT task), so AT commands must be allowed before RunSpicHandshake().
    if (type == FrameType::kEthernet && !handshake_done_.load()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len == 0 || len > kMaxPayloadLen) {
        ESP_LOGW(kTag, "Drop too large TX frame: %d bytes", (int)len);
        return ESP_ERR_INVALID_ARG;
    }

    if (type == FrameType::kEthernet) {
        LogFrameDiag("TX", buf, len, debug_enabled_.load());
    }

    // Buffer layout [4B header space][payload], header written per window at pop time
    uint8_t* frame = static_cast<uint8_t*>(heap_caps_malloc(kSfdtHeaderSize + len, MALLOC_CAP_DMA));
    if (!frame) {
        ESP_LOGE(kTag, "Failed to allocate TX buffer");
        return ESP_ERR_NO_MEM;
    }
    memset(frame, 0, kSfdtHeaderSize);
    memcpy(frame + kSfdtHeaderSize, buf, len);

    TxQueueItem item = {
        .buffer = frame,
        .payloadLen = (uint16_t)len,
        .type = type,
    };
    // AT traffic bypasses the ETH queue: the peer's XOFF only gates ETH
    // payloads, so an AT command must never queue behind (or be dropped
    // because of) an ETH backlog.
    QueueHandle_t q = (type == FrameType::kAtCommand) ? at_queue_ : tx_queue_;
    if (q == nullptr) {
        free(frame);
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(q, &item, 0) != pdTRUE) {
        ESP_LOGW(kTag, "%s queue full, dropping frame",
                 (type == FrameType::kAtCommand) ? "AT" : "ETH");
        free(frame);
        return ESP_ERR_NO_MEM;
    }

    if (transfer_task_) {
        xTaskNotifyGive(transfer_task_);
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Raw TX enqueue bypassing the handshake gate (used by the init task)
// ---------------------------------------------------------------------------

esp_err_t SpiEthModem::EnqueueRawFrame(const uint8_t* payload, uint16_t len) {
    // Buffer layout [4B header space][payload], header written per window at pop time
    uint8_t* frame = static_cast<uint8_t*>(heap_caps_malloc(kSfdtHeaderSize + len, MALLOC_CAP_DMA));
    if (!frame) {
        return ESP_ERR_NO_MEM;
    }
    memset(frame, 0, kSfdtHeaderSize);
    memcpy(frame + kSfdtHeaderSize, payload, len);

    TxQueueItem item = {};
    item.buffer = frame;
    item.payloadLen = len;
    item.type = FrameType::kEthernet;  // init-time SPIC frames are control plane
    if (xQueueSend(tx_queue_, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        free(frame);
        return ESP_ERR_NO_MEM;
    }
    if (transfer_task_) {
        xTaskNotifyGive(transfer_task_);
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// SFDT master transfer engine (1:1 port of sfdt_master.c SFDT_masterTransferTaskEntry)
// ---------------------------------------------------------------------------

// Download backpressure: the ESP32 could not hand a received ETH frame to
// lwIP. Tell the slave to stop sending ETH by setting the XOFF bit in our
// headers for a short while. Runs in the transfer task (HandleSfdtPayload is
// called from its RX path), so plain members are safe.
void SpiEthModem::NoteRxCongestion() {
    // Before the netif is up every frame is dropped by lwIP by design
    // (ethernetif_input() -> "!netif_is_up" -> ESP_FAIL). That is not
    // congestion, so keep the peer sending until the link is actually up.
    if (!link_up_.load()) {
        return;
    }

    rx_congest_events_++;
    if (!master_xoff_) {
        master_xoff_ = true;
        fc_pending_ = true;  // make sure the next window is an ETH header
    }
    // Every new drop extends the hold: the peer stays quiet until lwIP has
    // had a chance to drain.
    master_xoff_until_ = xTaskGetTickCount() + pdMS_TO_TICKS(kMasterXoffHoldMs);

    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (rx_congest_events_ <= 1 || now_ms - rx_congest_log_ms_ >= 1000) {
        rx_congest_log_ms_ = now_ms;
        ESP_LOGW(kTag, "RX congested, assert XOFF to slave (events=%lu, hold=%lu ms)",
                 (unsigned long)rx_congest_events_, (unsigned long)kMasterXoffHoldMs);
    }
}

void SpiEthModem::TransferTaskRun() {
    ESP_LOGD(kTag, "Transfer task started");

    SetMrdy(false);  // MRDY low when idle (EC master keeps it low between pulses)

    uint32_t txSeqNo = 0;
    uint32_t pendingRxCnt = 0;      // continuation tail bytes of current frame
    uint32_t rxFramePayloadLen = 0; // payload length of in-flight (partial) frame
    FrameType rxFrameType = FrameType::kEthernet; // payload class of in-flight frame
    uint32_t timeout = kMrdyHandshakeTimeoutMs;

    while (!stop_flag_.load()) {
        timeout = kMrdyHandshakeTimeoutMs;  // EC resets timeout every outer loop
        bool triggeredBySrdy = false;
        bool continuation = (pendingRxCnt != 0);

        // Download backpressure: the XOFF hold is time based -- while it is
        // asserted the slave stops sending ETH, so there are no RX frames to
        // count "successes" on. When it expires, force one window so the peer
        // actually receives the XON bit (in XOFF it is not arming ETH windows
        // on its own).
        if (master_xoff_ && (int32_t)(xTaskGetTickCount() - master_xoff_until_) >= 0) {
            master_xoff_ = false;
            fc_pending_ = true;
        }

        if (!continuation && !fc_pending_) {
            // Wait for tx or rx trigger. Both sources give this task directly
            // (SRDY ISR -> vTaskNotifyGiveFromISR, TX enqueue -> xTaskNotifyGive,
            // stop -> xTaskNotifyGive), so no esp_timer-daemon round-trip is in
            // the SRDY->SPI path. A leftover notification with no armed window
            // and no queued TX is simply ignored and the wait repeats.
            while (true) {
                // While XOFF is asserted the peer stops arming windows, so a
                // plain portMAX_DELAY wait would never wake up to clear it.
                TickType_t wait = portMAX_DELAY;
                if (master_xoff_) {
                    TickType_t now = xTaskGetTickCount();
                    wait = (master_xoff_until_ > now) ? (master_xoff_until_ - now) : 1;
                }
                ulTaskNotifyTake(pdTRUE, wait);
                if (stop_flag_.load()) {
                    goto exit;
                }
                // Fast path (EC reference sfdt_master.c: "skip handshake if rx
                // flag is set"): srdy_mask_ set means the slave raised SRDY
                // since the last window was clocked, i.e. it armed a window.
                if (srdy_mask_ != 0) {
                    triggeredBySrdy = true;
                    break;
                }
                if (uxQueueMessagesWaiting(at_queue_) > 0 ||
                    uxQueueMessagesWaiting(tx_queue_) > 0) {
                    break;
                }
                // Woken because the XOFF hold expired: send the XON window now
                // instead of blocking until the peer arms another one.
                if (master_xoff_ &&
                    (int32_t)(xTaskGetTickCount() - master_xoff_until_) >= 0) {
                    master_xoff_ = false;
                    fc_pending_ = true;
                }
                if (fc_pending_) {
                    break;
                }
            }
        }

        // NOTE: the former "SRDY already high at handshake gate -- level
        // fallback" fast path is GONE on purpose. A high SRDY *level* does not
        // certify that the slave's SPI is armed for the NEXT window: the slave
        // pulls SRDY low the moment a window completes (sfdt_slave_fc.c L405)
        // and only re-arms after its per-window turnaround (measured up to
        // ~30 ms while it forwards the received frame up its own stack).
        // Clocking on a stale level or a leftover RX flag from the previous
        // window fired straight into that unarmed gap and permanently ate the
        // frame built for it. Observed failure (run100): every download ACK
        // built right after a multi-window slave forward vanished this way --
        // the master's RX header read 00 00 00 00 / ff ff ff ff (floating,
        // unarmed slave), the slave's loop counter never advanced for those
        // windows, and the server stopped receiving ACKs entirely. Like the
        // proven EC master<->slave pairing, every window below is now backed
        // by a fresh SRDY posedge delivered into a wait.

        if (!triggeredBySrdy) {
            // TRIG_HANDSHAKE / WAIT_HANDSHAKE (EC reference): obtain a fresh
            // slave SRDY assertion before clocking. Non-continuation windows
            // pulse MRDY first (a new window request). Continuation windows
            // only wait at first -- the slave is mid-frame and re-arms +
            // re-asserts SRDY for the tail on its own (EC: goto
            // WAIT_HANDSHAKE) -- and only fall back to an MRDY pulse when the
            // assertion does not arrive in time (EC: timeout ->
            // goto TRIG_HANDSHAKE). Without this wait the tail clock ran
            // unverified while the slave's tail assertion leaked into the
            // NEXT window's gate, letting that window clock with an already
            // spent assertion.
            //
            // A high SRDY *level* is not a new-window request: the slave
            // pulls SRDY high before the transfer and drops it again only
            // after its task wakes up from the completion IRQ (sfdt_slave.c
            // L582 high -> L583 wait done -> L584 low), so at this gate the
            // line is usually the tail of the window just clocked and the
            // slave's SPI is not re-armed yet. Clocking on that level is what
            // lost frames in run100, so the wait below still starts only on a
            // fresh posedge. But there is no need to *request* a window
            // either: the slave is already past its blocking wait and raises
            // SRDY by itself after the turnaround. Skip the first MRDY pulse
            // then, and keep MRDY as the fallback for a genuinely idle slave
            // (SRDY low = blocked on gSlaveTransferTriggerFlag, L535).
            bool srdyHighAtGate = !continuation && IsSrdyHigh();
            bool pulse = (!continuation && !srdyHighAtGate);
            while (true) {
                if (pulse) {
                    SetMrdy(true);  // rising edge
                }
                pulse = false;
                uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout));
                SetMrdy(false);  // falling edge (EC pulls low right after the wait)

                if (stop_flag_.load()) {
                    goto exit;
                }
                // Only a real SRDY edge counts; a leftover notification must not
                // be mistaken for a handshake (that would clock an unarmed slave).
                if (srdy_mask_ != 0) {
                    break;
                }

                if (notified == 0) {
                    // Genuine timeout: escalate and re-pulse MRDY.
                    timeout += kMrdyHandshakeEscalationMs;
                    ESP_LOGW(kTag, "MRDY handshake timeout, retry in %lu ms", (unsigned long)timeout);
                    pulse = true;
                }
                // else: woken by an unrelated notification (TX enqueue / stale
                // one). The slave is already triggered, so keep waiting for its
                // SRDY edge without pinning another MRDY pulse.
            }
        }

        // ---- TRANSFER: pop one TX frame, clock one window ----
        // AT first: it is never gated by the peer's ETH flow control (the
        // slave routes AT payloads to its AT engine, not through the CCIO ETH
        // ring, and only reads the XOFF bit from ETH frames).
        uint32_t txMsgCnt = uxQueueMessagesWaiting(tx_queue_);
        uint32_t atMsgCnt = uxQueueMessagesWaiting(at_queue_);

        // A flow-control transition must ride an ETH header: the slave ignores
        // the FC bit of AT frames. Skip the pop for this one window and send a
        // zero-payload ETH header instead (AT waits at most one window).
        bool fc_window = fc_pending_;

        TxQueueItem item = {};
        bool popped = false;
        if (!fc_window) {
            if (xQueueReceive(at_queue_, &item, 0) == pdTRUE) {
                popped = true;
            } else if (!slave_flow_control_off_) {
                // Shall not consume ETH item in slave-XOFF state
                if (xQueueReceive(tx_queue_, &item, 0) == pdTRUE) {
                    popped = true;
                }
            }
        }

        // Build header over the window buffer (EC writes header into the popped buffer)
        FrameHeader* header = reinterpret_cast<FrameHeader*>(window_tx_buffer_);
        memset(header, 0, kSfdtHeaderSize);
        uint16_t payloadLen = popped ? item.payloadLen : 0;
        header->SetPayloadLength(payloadLen);
        header->SetFlowControl(master_xoff_);  // master XON/XOFF (download backpressure)
        header->SetSequence((uint8_t)(txSeqNo++));
        header->SetType(item.type);     // ETH for windows without a queued frame (zeroed item)
        // Slave bounces into endless req in case of slave XOFF (see EC comment).
        // ETH only: an AT frame is not a "more ETH behind this one" hint, and
        // AT is not gated by XOFF.
        if (popped && item.type == FrameType::kEthernet && txMsgCnt > 1 &&
            !slave_flow_control_off_) {
            header->SetContinue(true);
        }
        header->UpdateChecksum();

        if (popped) {
            memcpy(window_tx_buffer_ + kSfdtHeaderSize, item.buffer + kSfdtHeaderSize, item.payloadLen);
        }

        uint32_t txNum = payloadLen + kSfdtHeaderSize;
        if (txNum < pendingRxCnt) {
            txNum = pendingRxCnt;
        }
        if (txNum < kSfdtMinTransferDummy) {
            txNum = kSfdtMinTransferDummy;
        }
        // Slave arms exactly SFDT_FRAME_MTU RX bytes per window; never exceed it
        if (txNum > kSfdtFrameMtu) {
            txNum = kSfdtFrameMtu;
        }
        memset(window_tx_buffer_ + kSfdtHeaderSize + payloadLen, kSfdtFillByte,
               txNum - kSfdtHeaderSize - payloadLen);

        // NOTE: the per-window log is emitted *after* the DMA is queued (see
        // below). Emitting it here cost ~330 us of dead time between the SRDY
        // edge and the SPI start.

        // Clear srdy-mask gate so a posedge race around this window still wakes us
        portENTER_CRITICAL(&srdy_mux_);
        srdy_mask_ = 0;
        portEXIT_CRITICAL(&srdy_mux_);

        // Full duplex DMA transfer
        spi_transaction_t* t = &tx_transaction_;
        memset(t, 0, sizeof(spi_transaction_t));
        t->length = txNum * 8;
        t->rxlength = txNum * 8;
        t->tx_buffer = window_tx_buffer_;
        t->rx_buffer = window_rx_buffer_;
        t->user = this;

        esp_err_t ret = spi_device_queue_trans(spi_dev_, t, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(kTag, "spi_device_queue_trans failed: %s", esp_err_to_name(ret));
            if (popped) {
                free(item.buffer);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // The header of this window carries the current FC state; no need to
        // force another empty window for the same transition.
        fc_pending_ = false;

        // Per-window debug log, emitted only after the DMA is queued so it
        // overlaps the transfer instead of delaying the window.
        if (debug_enabled_.load()) {
            ESP_LOGI(kTag, "TX window: payLoadLen=%u seqNo=%u cont=%d type=%u txNum=%lu pendRx=%lu csum=0x%02x",
                     header->GetPayloadLength(), header->GetSequence(), header->GetContinue(),
                     (unsigned)header->GetType(),
                     (unsigned long)txNum, (unsigned long)pendingRxCnt, header->GetChecksum());
        }

        // Reap the window straight from the SPI driver: its ISR gives the
        // transaction semaphore directly, so there is no esp_timer-daemon hop
        // in the completion path (the old done-flag event-group wait had one).
        {
            spi_transaction_t* r = nullptr;
            if (spi_device_get_trans_result(spi_dev_, &r, pdMS_TO_TICKS(200)) != ESP_OK) {
                ESP_LOGE(kTag, "Failed to reap SPI transaction");
                if (popped) {
                    free(item.buffer);
                }
                continue;
            }
            uint32_t rxCnt = (uint32_t)(r->rxlength >> 3);  // master always clocks exactly txNum
            if (debug_enabled_.load() && rxCnt > 0) {
                ESP_LOGI(kTag, "RX window: %lu bytes, hdr: %02x %02x %02x %02x", (unsigned long)rxCnt,
                         window_rx_buffer_[0], window_rx_buffer_[1], window_rx_buffer_[2],
                         window_rx_buffer_[3]);
            }

            // ---- Parse RX data (EC lines 467-536) ----
            // Multi-window frames are assembled in rx_assembly_buffer_: every
            // DMA read overwrites window_rx_buffer_ from offset 0, so a frame
            // longer than the clocked window (slave DHCP OFFER ~346B vs the
            // 256B minimum poll) must have its tail appended at the absolute
            // offset -- delivering straight from window_rx_buffer_ mixed the
            // tail into the destroyed head and corrupted the frame.
            FrameHeader* rxHeader = reinterpret_cast<FrameHeader*>(window_rx_buffer_);

            if (pendingRxCnt == 0) {
                if (rxHeader->ValidateChecksum()) {
                    rxFramePayloadLen = rxHeader->GetPayloadLength();
                    rxFrameType = rxHeader->GetType();
                    if (rxFramePayloadLen <= kMaxPayloadLen) {
                        // The peer's XOFF bit only gates ETH payloads. AT
                        // frames are enqueued with a hardcoded XON on the
                        // slave side (sfdt_slave.c SFDT_atSend), so sampling
                        // them here would clear a real ETH XOFF on every AT
                        // URC and push frames into a full slave RX ring.
                        if (rxFrameType == FrameType::kEthernet) {
                            bool slaveXoff = rxHeader->GetFlowControl();
                            if (slave_flow_control_off_ && !slaveXoff && txMsgCnt > 0) {
                                // Resume tx when flow control switches from XOFF to XON
                                if (transfer_task_) {
                                    xTaskNotifyGive(transfer_task_);
                                }
                            }
                            if (slaveXoff != slave_flow_control_off_) {
                                slave_flow_control_off_ = slaveXoff;
                                if (slaveXoff) {
                                    xoff_events_++;
                                }
                                // Transition log, rate limited: XOFF can flap
                                // during congestion and a UART line in this
                                // loop costs real window time.
                                uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                                if (xoff_events_ <= 1 || now_ms - xoff_log_ms_ >= 1000) {
                                    xoff_log_ms_ = now_ms;
                                    ESP_LOGW(kTag, "Slave ETH flow control: %s, events=%lu (ethq=%lu atq=%lu)",
                                             slaveXoff ? "XOFF" : "XON",
                                             (unsigned long)xoff_events_,
                                             (unsigned long)txMsgCnt,
                                             (unsigned long)atMsgCnt);
                                }
                            }
                        }

                        if ((rxFramePayloadLen + kSfdtHeaderSize) > rxCnt) {
                            // slave frame spanned more clocks than this window:
                            // save the head, read the tail in the next window(s)
                            pendingRxCnt = rxFramePayloadLen + kSfdtHeaderSize - rxCnt;
                            memcpy(rx_assembly_buffer_, window_rx_buffer_, rxCnt);
                        } else {
                            HandleSfdtPayload(window_rx_buffer_ + kSfdtHeaderSize,
                                              (uint16_t)rxFramePayloadLen, rxFrameType);
                        }
                    }
                    // payLoadLen too big: silently ignore (EC does the same)
                }
                // Invalid checksum: idle filler (hardware 0xF0F0) or noise, drop silently
            } else if (rxCnt >= pendingRxCnt) {
                // Continuation round completed the frame: append the tail at
                // its absolute offset, deliver the assembled frame
                uint32_t received = rxFramePayloadLen + kSfdtHeaderSize - pendingRxCnt;
                memcpy(rx_assembly_buffer_ + received, window_rx_buffer_, pendingRxCnt);
                HandleSfdtPayload(rx_assembly_buffer_ + kSfdtHeaderSize,
                                  (uint16_t)rxFramePayloadLen, rxFrameType);
                pendingRxCnt = 0;
            } else {
                // Cannot happen: txNum is floored at pendingRxCnt. Match EC assert with a log.
                ESP_LOGE(kTag, "Bad continuation: rxCnt=%lu pendingRxCnt=%lu", (unsigned long)rxCnt,
                         (unsigned long)pendingRxCnt);
                pendingRxCnt = 0;
            }
        }

        if (popped) {
            free(item.buffer);
        }
    }

exit:
    // Ensure MRDY is low before exiting
    SetMrdy(false);
    ESP_LOGD(kTag, "Transfer task exiting");
    transfer_task_ = nullptr;   // no stale-handle notifications after vTaskDelete
    xEventGroupSetBits(event_group_, kEventTransferTaskDone);
}

// ---------------------------------------------------------------------------
// Frame payload dispatch
// ---------------------------------------------------------------------------

void SpiEthModem::HandleSfdtPayload(uint8_t* payload, uint16_t length, FrameType type) {
    if (length == 0) {
        // Zero payload frame = flow control only
        return;
    }

    // AT class: text exchanged with the EC718 AT engine (never an Ethernet frame)
    if (type == FrameType::kAtCommand) {
        HandleAtPayload(payload, length);
        return;
    }
    if (type != FrameType::kEthernet) {
        ESP_LOGW(kTag, "Drop frame with unsupported type %u (%u bytes)", (unsigned)type, length);
        return;
    }

    // SPIC control plane: payload starting with the "SPIC" magic
    if (length >= 12) {
        uint32_t magic = 0;
        memcpy(&magic, payload, sizeof(magic));
        if (magic == kSpicMagic32) {
            HandleSpicMessage(payload, length);
            return;
        }
    }

    // Ethernet frame -> LwIP (only after the SPIC handshake completed)
    if (!handshake_done_.load()) {
        return;
    }

    LogFrameDiag("RX", payload, length, debug_enabled_.load());

    uint8_t* copy = static_cast<uint8_t*>(malloc(length));
    if (copy) {
        memcpy(copy, payload, length);
        if (mediator_) {
            // stack_input takes ownership of the buffer (frees it), also on
            // failure: ethernetif_input() pbuf_free()s the pbuf it built
            // around it. A non-OK return means the frame never reached lwIP
            // (mailbox full / pbuf alloc failure), i.e. the RX path is
            // congested and the peer should stop sending ETH for a while.
            esp_err_t err = mediator_->stack_input(mediator_, copy, length);
            if (err != ESP_OK) {
                NoteRxCongestion();
            }
        } else {
            free(copy);
        }
    } else {
        ESP_LOGE(kTag, "Failed to allocate RX buffer");
        NoteRxCongestion();
    }
}

void SpiEthModem::HandleAtPayload(const uint8_t* payload, uint16_t length) {
    // Trim trailing NUL padding (sender includes one for C-string safety)
    uint16_t textLen = length;
    while (textLen > 0 && payload[textLen - 1] == '\0') {
        textLen--;
    }

    if (!waiting_for_at_response_.load()) {
        // URC / boot banner (e.g. "\r\nECRDY\r\n") when no SendAt() is pending
        ESP_LOGI(kTag, "AT RX (unsolicited, %u bytes): %.*s", textLen, (int)textLen,
                 reinterpret_cast<const char*>(payload));
        return;
    }

    at_command_response_.append(reinterpret_cast<const char*>(payload), textLen);
    xEventGroupSetBits(event_group_, kEventAtResponse);

    if (debug_enabled_.load()) {
        ESP_LOGI(kTag, "AT<<< %.*s", (int)textLen, reinterpret_cast<const char*>(payload));
    }
}

void SpiEthModem::HandleSpicMessage(const uint8_t* payload, uint16_t length) {
    uint16_t type = 0;
    uint16_t msgLen = 0;
    memcpy(&type, payload + 4, sizeof(type));
    memcpy(&msgLen, payload + 6, sizeof(msgLen));

    const uint8_t* body = payload + 12;

    switch (type) {
        case kSpicNetInitialCmplt: {
            if (length < 32) {
                ESP_LOGW(kTag, "NET_INITIAL_CMPLT too short: %d", length);
                break;
            }
            uint32_t sid = 0;
            uint32_t status = 0;
            uint16_t majVer = 0;
            uint16_t minVer = 0;
            uint32_t maxXferSize = 0;
            uint16_t maxPktsPerXfer = 0;
            uint16_t pktAlignValue = 0;
            memcpy(&sid, body + 0, 4);
            memcpy(&status, body + 4, 4);
            memcpy(&majVer, body + 8, 2);
            memcpy(&minVer, body + 10, 2);
            memcpy(&maxXferSize, body + 12, 4);
            memcpy(&maxPktsPerXfer, body + 16, 2);
            memcpy(&pktAlignValue, body + 18, 2);

            ESP_LOGI(kTag, "NET_INITIAL_CMPLT: sid=%lu status=0x%lx ver=%u.%u maxXfer=%lu "
                            "maxPkts=%u align=%u",
                     (unsigned long)sid, (unsigned long)status, majVer, minVer,
                     (unsigned long)maxXferSize, maxPktsPerXfer, pktAlignValue);

            if (status == 0) {
                if (!handshake_done_.load()) {
                    handshake_done_ = true;
                    initialized_ = true;
                    SetNetworkEvent(SpiEthModemEvent::Connecting);
                    xEventGroupSetBits(event_group_, kEventHandshakeDone);
                    ESP_LOGI(kTag, "SPIC handshake done, waiting for WAN media status...");
                }
            } else {
                ESP_LOGW(kTag, "NET_INITIAL_CMPLT failure, will retry");
            }
            break;
        }

        case kSpicNetResetCmplt: {
            uint32_t status = 0;
            if (length >= 16) {
                memcpy(&status, body, 4);
            }
            ESP_LOGI(kTag, "NET_RESET_CMPLT: status=0x%lx", (unsigned long)status);
            break;
        }

        case kSpicGenStatusIndMsg: {
            uint32_t status = 0;
            if (length >= 16) {
                memcpy(&status, body, 4);
            }
            if (status == kSpicStatusMediaDisconn) {
                ESP_LOGW(kTag, "GEN_STATUS: WAN media disconnected");
                NotifyLink(false);
            } else {
                // MEDIA_CONN (0xf0000003) and any non-disconnect status bring
                // the link up. The spi_2boards slave enters NIC mode by
                // sending GEN_STATUS right after NET_INITIAL; its status
                // value is not guaranteed to be the canonical MEDIA_CONN
                // constant, and a media-status indication that is not
                // "disconnected" means the medium is available. The raw
                // value is always logged so the mapping can be tightened.
                ESP_LOGI(kTag, "GEN_STATUS: status=0x%08lx -> link up", (unsigned long)status);
                NotifyLink(true);
            }
            break;
        }

        case kSpicGenNotificationMsg: {
            uint32_t notif = 0;
            if (length >= 16) {
                memcpy(&notif, body, 4);
            }
            if (notif == kSpicNotifFlowCtlOn) {
                slave_flow_control_off_ = false;
                if (uxQueueMessagesWaiting(tx_queue_) > 0) {
                    if (transfer_task_) {
                        xTaskNotifyGive(transfer_task_);
                    }
                }
                ESP_LOGD(kTag, "SPIC notif: peer flow control ON");
            } else if (notif == kSpicNotifFlowCtlOff) {
                slave_flow_control_off_ = true;
                ESP_LOGD(kTag, "SPIC notif: peer flow control OFF");
            }
            break;
        }

        default:
            ESP_LOGI(kTag, " Ignore SPIC msg type 0x%04x len %d", type, msgLen);
            break;
    }
}

// ---------------------------------------------------------------------------
// Init task: SPIC handshake + bring-up (mirrors UartEthModem InitTaskRun)
// ---------------------------------------------------------------------------

void SpiEthModem::InitTaskRun() {
    ESP_LOGI(kTag, "Init task started");

    xEventGroupWaitBits(event_group_, kEventStart, pdTRUE, pdTRUE, portMAX_DELAY);

    if (stop_flag_.load()) {
        goto exit_fail;
    }

    // Install iot_eth / netif first so GEN_STATUS and CMPLT can be applied
    // the moment they arrive.
    if (InitIotEth() != ESP_OK) {
        ESP_LOGE(kTag, "Failed to initialize iot_eth");
        goto exit_fail;
    }

    // AT-based modem init (reference: UartEthModem::RunNormalModeInitSequence).
    // Must run before any internet use: AT detect -> CFUN=1 -> SIM check ->
    // registration -> AT+ECNETDEVCTL dial -> SPIC NET_INITIAL handshake.
    // The SPIC handshake is the SPI transport's equivalent of the UART build's
    // kHandshakeRequest: it starts the netdev data path and is therefore the
    // last step of the sequence, not a prerequisite of the AT channel.
    if (RunNormalModeInitSequence() != ESP_OK) {
        ESP_LOGE(kTag, "AT init sequence failed");
        SetNetworkEvent(SpiEthModemEvent::ErrorInitFailed);
        stop_flag_ = true;
        if (transfer_task_) {
            xTaskNotifyGive(transfer_task_);
        }
        xEventGroupSetBits(event_group_, kEventStop);
        goto exit_fail;
    }

    ESP_LOGI(kTag, "Init complete");
    xEventGroupSetBits(event_group_, kEventInitDone);
    xEventGroupSetBits(event_group_, kEventInitTaskDone);
    return;

exit_fail:
    ESP_LOGD(kTag, "Init task exiting (fail)");
    xEventGroupSetBits(event_group_, kEventInitTaskDone);
}

// ---------------------------------------------------------------------------
// SPIC transport handshake (NET_INITIAL)
// ---------------------------------------------------------------------------

esp_err_t SpiEthModem::RunSpicHandshake() {
    SetNetworkEvent(SpiEthModemEvent::Connecting);

    // SPIC NET_INITIAL, header-only — same 12-byte form as the UART reference
    // (UartEthModem::kHandshakeRequest). `len` counts the 12-byte SPIC header;
    // there is no body. The slave echoes its (zero-length) body fields back in
    // NET_INITIAL_CMPLT, but the status/cmpLT is driven by its EOS entity, so
    // the handshake completes regardless of the echoed values.
    static constexpr uint8_t kNetInitialMsg[12] = {
            0x53, 0x50, 0x49, 0x43,  // magic "SPIC"
            0x02, 0x04,              // type 0x0402 (NET_INITIAL)
            0x0C, 0x00,              // length 12 = SPIC header only
            0x00, 0x00, 0x00, 0x00   // reserved
    };

    // A re-handshake (e.g. after AT+ECRST) starts from a clean transport state
    handshake_done_ = false;
    initialized_ = false;
    xEventGroupClearBits(event_group_, kEventHandshakeDone);

    uint32_t startMs = (uint32_t)(xTaskGetTickCount()) * portTICK_PERIOD_MS;
    uint32_t retryCount = 0;
    while (!handshake_done_.load()) {
        if (stop_flag_.load()) {
            return ESP_ERR_INVALID_STATE;
        }

        uint32_t elapsed = (uint32_t)(xTaskGetTickCount()) * portTICK_PERIOD_MS - startMs;
        if (elapsed >= kSpicHandshakeTotalMs) {
            ESP_LOGE(kTag, "SPIC handshake timeout (%lu ms)", (unsigned long)elapsed);
            return ESP_ERR_TIMEOUT;
        }

        esp_err_t ret = EnqueueRawFrame(kNetInitialMsg, sizeof(kNetInitialMsg));
        if (ret != ESP_OK) {
            ESP_LOGE(kTag, "Failed to send NET_INITIAL: %s", esp_err_to_name(ret));
        }
        retryCount++;
        if ((retryCount % 5) == 1) {
            ESP_LOGI(kTag, "Handshake retry #%lu (elapsed %lu ms): No NET_INITIAL_CMPLT "
                            "from EC718 yet (slave still booting?)",
                     (unsigned long)retryCount, (unsigned long)elapsed);
        }

        EventBits_t bits = xEventGroupWaitBits(event_group_, kEventHandshakeDone | kEventStop,
                                               pdFALSE, pdFALSE,
                                               pdMS_TO_TICKS(kSpicHandshakeTimeoutMs));
        if (bits & kEventStop) {
            ESP_LOGW(kTag, "Stop received during handshake");
            return ESP_ERR_INVALID_STATE;
        }
        if (!(bits & kEventHandshakeDone)) {
            vTaskDelay(pdMS_TO_TICKS(kSpicHandshakeRetryMs));
        } else {
            break;
        }
    }

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// AT-based modem initialization
// Mirrors UartEthModem::RunNormalModeInitSequence(): the AT engine of the
// EC718 performs the actual network provisioning and dial. Runs after the
// SPIC transport handshake, before any internet use.
// ---------------------------------------------------------------------------

esp_err_t SpiEthModem::SendAtWithRetry(const std::string& cmd, std::string& response,
                                       uint32_t timeout_ms, int max_retries) {
    esp_err_t ret = ESP_FAIL;
    for (int i = 0; i < max_retries; i++) {
        if (stop_flag_.load()) {
            return ESP_ERR_INVALID_STATE;
        }
        ret = SendAt(cmd, response, timeout_ms);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
        if (i < max_retries - 1) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    return ret;
}

esp_err_t SpiEthModem::AtDetect() {
    std::string resp;
    // The slave needs ~1 s after reset before its AT task answers, and a frame
    // queued while it is still booting would be replayed later (duplicate AT
    // responses). Use a wide first-attempt budget so the initial AT usually
    // succeeds on its own.
    esp_err_t ret = SendAtWithRetry("AT", resp, 2000, 4);
    if (ret == ESP_OK) {
        ESP_LOGI(kTag, "Modem detected over SFDT AT channel");
        return ESP_OK;
    }
    ESP_LOGE(kTag, "Modem not responding to AT over SFDT: %s", esp_err_to_name(ret));
    return ret;
}

bool SpiEthModem::CheckSimCard() {
    std::string resp;
    for (int i = 0; i < 10; i++) {
        if (stop_flag_.load()) {
            ESP_LOGW(kTag, "CheckSimCard aborted due to stop flag");
            return false;
        }
        if (SendAt("AT+CPIN?", resp, 1000) == ESP_OK) {
            if (resp.find("+CPIN: READY") != std::string::npos) {
                return true;
            }
        }
        if (resp.find("+CME ERROR: 10") != std::string::npos) {
            // SIM not inserted
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return false;
}

void SpiEthModem::ParseAtResponse(const std::string& response) {
    auto cereg_pos = response.find("+CEREG:");
    if (cereg_pos == std::string::npos) {
        return;
    }

    int n = 0, stat = 0, act = 0;
    char tac[16] = {0}, ci[16] = {0};

    // +CEREG: n,stat,"tac","ci",act
    if (sscanf(response.c_str() + cereg_pos, "+CEREG: %d,%d,\"%15[^\"]\",\"%15[^\"]\",%d",
               &n, &stat, tac, ci, &act) == 5 ||
        // +CEREG: stat,"tac","ci",act
        sscanf(response.c_str() + cereg_pos, "+CEREG: %d,\"%15[^\"]\",\"%15[^\"]\",%d",
               &stat, tac, ci, &act) == 4 ||
        // +CEREG: n,stat
        sscanf(response.c_str() + cereg_pos, "+CEREG: %d,%d", &n, &stat) == 2 ||
        // +CEREG: stat
        sscanf(response.c_str() + cereg_pos, "+CEREG: %d", &stat) == 1) {
        cell_reg_stat_ = stat;
    }
}

bool SpiEthModem::WaitForRegistration(uint32_t timeout_ms) {
    std::string resp;
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t lastLog = 0;

    while (!stop_flag_.load()) {
        if (SendAt("AT+CEREG?", resp, 1000) == ESP_OK) {
            ParseAtResponse(resp);
            if (cell_reg_stat_ == 1 || cell_reg_stat_ == 5) {
                ESP_LOGI(kTag, "Network registered (CEREG stat=%d)", cell_reg_stat_);
                return true;
            }
            if (cell_reg_stat_ == 3) {
                ESP_LOGE(kTag, "Network registration denied");
                return false;
            }
        }

        uint32_t elapsed = xTaskGetTickCount() * portTICK_PERIOD_MS - start;
        if (elapsed >= timeout_ms) {
            ESP_LOGW(kTag, "Registration timeout (CEREG stat=%d)", cell_reg_stat_);
            return false;
        }
        if (elapsed - lastLog >= 10000) {
            ESP_LOGI(kTag, "Waiting for registration... (%lu/%lu ms, stat=%d)",
                     (unsigned long)elapsed, (unsigned long)timeout_ms, cell_reg_stat_);
            lastLog = elapsed;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return false;
}

void SpiEthModem::QueryModemInfo() {
    std::string resp;
    char value[128] = {0};

    if (SendAt("AT+CGSN=1", resp, 1000) == ESP_OK &&
        sscanf(resp.c_str(), "\r\n+CGSN: \"%15s", value) == 1) {
        ESP_LOGI(kTag, "Modem IMEI: %s", value);
    }

    if (SendAt("AT+ECICCID", resp, 1000) == ESP_OK &&
        sscanf(resp.c_str(), "\r\n+ECICCID: %20s", value) == 1) {
        ESP_LOGI(kTag, "SIM ICCID: %s", value);
    }

    if (SendAt("AT+CGMR", resp, 1000) == ESP_OK) {
        memset(value, 0, sizeof(value));
        if (resp.find("+CGMR:") != std::string::npos) {
            if (sscanf(resp.c_str(), "\r\n+CGMR: \r\n%127[^\r\n]", value) == 1) {
                ESP_LOGI(kTag, "Modem revision: %s", value);
            }
        } else if (sscanf(resp.c_str(), "\r\n%127[^\r\n]", value) == 1) {
            ESP_LOGI(kTag, "Modem revision: %s", value);
        }
    }
}

esp_err_t SpiEthModem::RunNormalModeInitSequence() {
    std::string resp;
    esp_err_t ret;

    // Step 1: AT test
    ESP_LOGI(kTag, "AT init: detecting modem...");
    ret = AtDetect();
    if (ret != ESP_OK) {
        return ret;
    }

    // Step 2: enter full functionality mode (CFUN=1)
    ESP_LOGI(kTag, "AT init: CFUN=1");
    ret = SendAt("AT+CFUN=1", resp, 3000);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to enter full functionality mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 3: network configuration (NAT). The query can come back as a bare
    // "OK" right after the EC718 boots, so retry a few times before concluding
    // anything. Only an explicit "nat",0 entry triggers provisioning — that
    // path ends with AT+ECRST (EC718 reboot), so a false positive is expensive.
    bool natConfigured = false;
    bool natDisabled = false;
    for (int i = 0; i < 5; i++) {
        if (stop_flag_.load()) {
            return ESP_ERR_INVALID_STATE;
        }
        ret = SendAt("AT+ECNETCFG?", resp, 1000);
        if (ret == ESP_OK) {
            if (resp.find("+ECNETCFG: \"nat\",1") != std::string::npos) {
                natConfigured = true;
                break;
            }
            if (resp.find("+ECNETCFG: \"nat\",0") != std::string::npos) {
                natDisabled = true;
                break;
            }
        }
        if (i < 4) {
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }

    if (natDisabled) {
        ESP_LOGW(kTag, "AT init: nat disabled, provisioning network config");
        SendAtWithRetry("AT+ECPCFG=\"usbCtrl\",1", resp, 1000, 3);
        SendAtWithRetry("AT+ECNETCFG=\"nat\",1,\"192.168.10.2\"", resp, 1000, 3);
        //SendAtWithRetry("AT&W", resp, 300, 3);
        // AT&W returns OK before the NVRAM/Flash write is necessarily committed.
        // Resetting immediately (AT+ECRST) can leave a half-written record
        // (observed as `"nat",0` with an empty netmask on the next boot).
        vTaskDelay(pdMS_TO_TICKS(300));

        // AT+ECRST reboots the EC718. The AT channel does not depend on the
        // SPIC handshake, so just wait for the AT engine to come back; the
        // netdev handshake happens once, at the end of this sequence.
        ESP_LOGW(kTag, "AT init: EC718 reset required (AT+ECRST)");
        SendAt("AT+ECRST", resp, 500);
        vTaskDelay(pdMS_TO_TICKS(1500));
        if (SendAtWithRetry("AT", resp, 500, 20) != ESP_OK) {
            ESP_LOGE(kTag, "Modem not responding after reset");
            return ESP_ERR_TIMEOUT;
        }
        ret = SendAt("AT+CFUN=1", resp, 3000);
        if (ret != ESP_OK) {
            ESP_LOGE(kTag, "Failed to enter full functionality mode after reset");
            return ret;
        }
    } else if (!natConfigured) {
        ESP_LOGW(kTag, "AT init: ECNETCFG reported no nat entry (%s), assuming configured",
                 esp_err_to_name(ret));
    }

    // Step 4: SIM card
    ESP_LOGI(kTag, "AT init: checking SIM card...");
    if (!CheckSimCard()) {
        ESP_LOGE(kTag, "SIM card not ready");
        SetNetworkEvent(SpiEthModemEvent::ErrorNoSim);
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(kTag, "AT init: SIM ready");

    // Step 5: modem info (log only)
    QueryModemInfo();

    // Step 6: wait for network registration
    ESP_LOGI(kTag, "AT init: waiting for network registration...");
    // The EC718 auto-starts its netdev at boot, so the iot_eth glue may already
    // have reported Connected. Do not flap back to Connecting in that case.
    if (network_event_.load() != SpiEthModemEvent::Connected) {
        SetNetworkEvent(SpiEthModemEvent::Connecting);
    }
    SendAt("AT+CEREG=2", resp);
    if (!WaitForRegistration(60000)) {
        return ESP_ERR_TIMEOUT;
    }

    // Step 7: start (dial) the network device
    int state = 0;
    if (SendAt("AT+ECNETDEVCTL?", resp, 1000) == ESP_OK) {
        sscanf(resp.c_str(), "\r\n+ECNETDEVCTL: %*d,%*d,%*d,%d", &state);
    }
    if (state == 1) {
        ESP_LOGI(kTag, "AT init: network device already started");
    } else {
        ESP_LOGI(kTag, "AT init: starting network device (dial)...");
        ret = SendAt("AT+ECNETDEVCTL=2,1,1", resp, 5000);
        if (ret != ESP_OK) {
            // The EC718 may reject a redundant dial when its netdev is already
            // up (host restarted without resetting the module, e.g. no RESET
            // line). If the link is live, keep going.
            if (network_event_.load() == SpiEthModemEvent::Connected) {
                ESP_LOGW(kTag, "AT init: dial returned %s but link is already up, continue",
                         esp_err_to_name(ret));
            } else {
                ESP_LOGE(kTag, "Failed to start network device: %s", esp_err_to_name(ret));
                return ret;
            }
        }
    }

    // Step 8: SPIC handshake (NET_INITIAL) — the SPI transport's equivalent of
    // UartEthModem's kHandshakeRequest. Sent at the end of the AT sequence,
    // after the modem has dialed: it initializes the netdev data path (EOS
    // media) on the EC718, which then reports GEN_STATUS / media and lets IP
    // traffic flow. The AT channel itself does not depend on it.
    ESP_LOGI(kTag, "AT init: SPIC netdev handshake (NET_INITIAL)...");
    if (RunSpicHandshake() != ESP_OK) {
        ESP_LOGE(kTag, "SPIC handshake failed");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(kTag, "AT init sequence complete");
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Resource cleanup
// ---------------------------------------------------------------------------

void SpiEthModem::CleanupResources() {
    DeinitIotEth();
    DeinitSpi();
    DeinitGpio();

    if (window_tx_buffer_) {
        free(window_tx_buffer_);
        window_tx_buffer_ = nullptr;
    }
    if (window_rx_buffer_) {
        free(window_rx_buffer_);
        window_rx_buffer_ = nullptr;
    }
    if (rx_assembly_buffer_) {
        free(rx_assembly_buffer_);
        rx_assembly_buffer_ = nullptr;
    }
    if (tx_queue_) {
        // Drain leftovers so nothing leaks
        TxQueueItem item = {};
        while (xQueueReceive(tx_queue_, &item, 0) == pdTRUE) {
            if (item.buffer) {
                free(item.buffer);
            }
        }
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
    }
    if (at_queue_) {
        TxQueueItem item = {};
        while (xQueueReceive(at_queue_, &item, 0) == pdTRUE) {
            if (item.buffer) {
                free(item.buffer);
            }
        }
        vQueueDelete(at_queue_);
        at_queue_ = nullptr;
    }
}

// ---------------------------------------------------------------------------
// ISR handlers
// ---------------------------------------------------------------------------

void IRAM_ATTR SpiEthModem::SrdyIsrHandler(void* arg) {
    auto* self = static_cast<SpiEthModem*>(arg);
    BaseType_t higher_prio_woken = pdFALSE;

    // SRDY is a push-pull output driven by the EC718 slave: one posedge per
    // armed window, no bouncing. Latch the first edge of each window
    // (srdy_mask_ is cleared right before the transfer clocks, so the next
    // window always needs a new edge).
    portENTER_CRITICAL_ISR(&self->srdy_mux_);
    if (self->transfer_task_ && self->srdy_mask_ == 0) {
        self->srdy_mask_ = 1;
        portEXIT_CRITICAL_ISR(&self->srdy_mux_);

        // Direct task notification: xEventGroupSetBitsFromISR would detour
        // through the esp_timer daemon (measured ~170 us) before the transfer
        // task could start the window.
        vTaskNotifyGiveFromISR(self->transfer_task_, &higher_prio_woken);
    } else {
        portEXIT_CRITICAL_ISR(&self->srdy_mux_);
    }

    if (higher_prio_woken) {
        portYIELD_FROM_ISR();
    }
}

// ---------------------------------------------------------------------------
// SRDY interrupt (ISR context cannot log)
// ---------------------------------------------------------------------------

