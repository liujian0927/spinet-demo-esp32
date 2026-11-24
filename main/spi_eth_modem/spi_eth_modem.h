// Copyright 2025 Terrence
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "iot_eth.h"
#include "iot_eth_netif_glue.h"
#include "iot_eth_types.h"

/**
* @brief SPI Ethernet Modem driver for EC718 family modules (EIGENCOMM SFDT slave).
*
* Communicates with an EC718 board running the spi_2boards SFDT slave application
* over SPI (full duplex, DMA). The EC718 board is the SPI slave and manages its own
* protocol stack (EOS = Ethernet Over SPI); the ESP32 acts as the SFDT SPI master.
*
* Protocol summary (mirrors EC718's project/ec7xx_0h00/ap/apps/spi_2boards/ sfdt_master.c):
* - Every octet of SRDY rising edge means the slave armed a transfer window.
*   The master may pulse MRDY (high>low) to ask the slave to arm a window.
* - Per window: meta frame = [4B SFDT header][payload][0xF0 filler] full duplex,
*   clocked length txNum = MAX(256, 4+payload, pendingRxTail), never exceeding 1600.
* - Flow control is carried in each SFDT header bit (XON=0 / XOFF=1) in every direction.
* - Control plane (SPIC messages) travels as SFDT frame payloads starting with the
*   "SPIC" magic (0x43495053 "SPIC" bytes "SPIC"): NET_INITIAL handshake and
*   GEN_STATUS_IND connectivity notifications. All other payloads are Ethernet frames.
*
* - Two payload classes are discriminated by the header "type" field (mirrors
*   uart_eth_modem's FrameHeader and EC718 sfdt.h SFDTPkgType_e):
*   - type = 0 (Ethernet): Ethernet frames plus SPIC control messages
*   - type = 1 (AT): AT command/response text exchanged with the EC718 AT engine
*     (the slave routes these to atRilAtCmdReq, see spi_2boards sfdt_slave.c)
*
* Wiring (EC718 spi_2boards slave pins, sfdt.h):
*   EC718 SPI0 MOSI (out) -> ESP32 MISO
*   EC718 SPI0 MISO (in)  <- ESP32 MOSI
*   EC718 SPI0 CLK  (in)  <- ESP32 SCLK
*   EC718 SRDY      (out) -> ESP32 SRDY input (idle LOW, HIGH pulse = window armed)
*   EC718 MRDY      (in)  <- ESP32 MRDY output (idle LOW, HIGH pulse = trigger)
*
* Light sleep is supported: esp_pm light-sleep is enabled while waiting for the
* slave, and SRDY is configured as a wakeup source (HIGH level).
*/
class SpiEthModem {
public:
    // Network event enumeration (aligned with UartEthModem for drop-in use)
    enum class SpiEthModemEvent {
        Connecting,              // Network is connecting (handshake done, waiting for WAN)
        Connected,               // Network connected successfully (got IP address)
        Disconnected,            // Network disconnected (WAN media down)
        ErrorInitFailed,         // SPI handshake / AT init sequence failed
        ErrorNoSim,              // SIM card not ready during AT init
    };

    // Configuration
    struct Config {
        spi_host_device_t spi_host = SPI2_HOST;
        gpio_num_t sclk_pin = GPIO_NUM_4;
        gpio_num_t mosi_pin = GPIO_NUM_6;   // -> EC718 SPI MISO input
        gpio_num_t miso_pin = GPIO_NUM_5;   // <- EC718 SPI MOSI output
        gpio_num_t cs_pin = GPIO_NUM_NC;    // optional hardware CS (active low, framed per SFDT window); NC/-1 disables
        gpio_num_t mrdy_pin = GPIO_NUM_10;  // Master Ready output (idle low; SuperMini header, no strap role)
        gpio_num_t srdy_pin = GPIO_NUM_7;   // Slave Ready input (idle low, HIGH pulse = window armed; light-sleep wake)
        uint32_t clock_speed_hz = 10000000; // SPI clock speed (Hz)
    };

    // Callback types
    using SpiEthModemEventCallback = std::function<void(SpiEthModemEvent event)>;

    explicit SpiEthModem(const Config& config);
    ~SpiEthModem();

    // Non-copyable
    SpiEthModem(const SpiEthModem&) = delete;
    SpiEthModem& operator=(const SpiEthModem&) = delete;

    /**
    * @brief Start the modem
    *
    * This will:
    * 1. Initialize SPI bus (DMA) and handshake GPIOs
    * 2. Create transfer task and start task
    * 3. Run SPIC NET_INITIAL handshake with the EC718 (with retries)
    * 4. Install iot_eth driver and create netif (link state follows EC718 reports)
    * 5. Enable light sleep with SRDY GPIO wakeup
    *
    * @return ESP_OK on success
    */
    esp_err_t Start();

    /**
    * @brief Stop the modem
    *
    * This will stop all tasks and release resources, and stop the light sleep pm.
    *
    * @return ESP_OK on success
    */
    esp_err_t Stop();

    /**
    * @brief Send an AT command to the EC718 AT engine over the SFDT link
    *
    * Encodes the command as an SFDT frame with header type = AT (1). The EC718
    * slave routes it to its AT engine (atRilAtCmdReq) and returns the response
    * as AT-type frames, which are accumulated into @p response.
    * Mirrors UartEthModem::SendAt() (OK/ERROR judgement) but keeps accumulating
    * response lines until the final result code arrives, because the EC718 AT
    * engine reports one response line per AT frame.
    *
    * The payload is "cmd\r\0" so the slave-side string handling stays safe
    * while the AT line parser still sees a CR-terminated line.
    *
    * @param cmd        AT command, with or without the trailing CR
    * @param response   Output: accumulated AT response text
    * @param timeout_ms Response wait budget
    * @return ESP_OK when the response contains "OK", ESP_FAIL on "ERROR" or an
    *         unexpected response, ESP_ERR_INVALID_STATE if not initialized,
    *         ESP_ERR_TIMEOUT when no response arrived
    */
    esp_err_t SendAt(const std::string& cmd, std::string& response, uint32_t timeout_ms = 1000);

    /**
     * @brief Check if network is initialized
     *
     * @return true if the SPIC handshake completed, false otherwise
     */
    bool IsInitialized() const { return initialized_.load(); }

    /**
      * @brief Set callback for network event changes
      */
    void SetNetworkEventCallback(SpiEthModemEventCallback callback);

    /**
     * @brief Enable or disable debug logging
     */
    void SetDebug(bool enabled);

    /**
     * @brief Get the network interface
     *
     * Use this for network operations after network is ready.
     *
     * @return esp_netif_t pointer, or nullptr if not initialized
     */
    esp_netif_t* GetNetif() const { return eth_netif_; }

    /**
     * @brief Get network event name for logging
     */
    static const char* GetNetworkEventName(SpiEthModemEvent event);

private:
    // Payload class carried in the SFDT header "type" field (2 bits).
    // Mirrors uart_eth_modem's FrameType and EC718's SFDTPkgType_e.
    enum class FrameType : uint8_t {
        kEthernet = 0,   // Ethernet frame or SPIC control message
        kAtCommand = 1,  // AT command/response text (EC718 AT engine)
    };
    static constexpr uint8_t kFrameTypeMask = 0x03;

    // SFDT frame header structure (4 bytes, identical bitfield to EC718 sfdt.h SFDTFrameHeader_t)
    // Layout (little endian, union over uint32):
    //   payLoadLen: 12 bits (byte0 + byte1 low nibble)
    //   seqNo:       4 bits (byte1 high nibble)
    //   flowControl: 1 bit  (byte2 bit0, 0 = XON permit, 1 = XOFF shall not send)
    //   bContinue:   1 bit  (byte2 bit1, more TX queued behind this frame)
    //   type:        2 bits (byte2 bits2-3, FrameType: 0 = ETH, 1 = AT)
    //   resv:        4 bits (byte2 bits4-7, zero)
    //   checkSum:    8 bits (byte3, (b0+b1+b2 >> 8) ^ (b0+b1+b2) ^ 0x3)
    struct FrameHeader {
        uint8_t raw[4];

        uint16_t GetPayloadLength() const {
            return raw[0] | ((raw[1] & 0x0F) << 8);
        }
        void SetPayloadLength(uint16_t len) {
            raw[0] = len & 0xFF;
            raw[1] = (raw[1] & 0xF0) | ((len >> 8) & 0x0F);
        }

        uint8_t GetSequence() const { return (raw[1] >> 4) & 0x0F; }
        void SetSequence(uint8_t seq) {
            raw[1] = (raw[1] & 0x0F) | ((seq & 0x0F) << 4);
        }

        // Flow control: 0 = XON (permit to send), 1 = XOFF (shall not send)
        bool GetFlowControl() const { return raw[2] & 0x01; }
        void SetFlowControl(bool xoff) {
            raw[2] = (raw[2] & 0xFE) | (xoff ? 1 : 0);
        }

        bool GetContinue() const { return (raw[2] >> 1) & 0x01; }
        void SetContinue(bool cont) {
            raw[2] = (raw[2] & 0xFD) | ((cont ? 1 : 0) << 1);
        }

        // Payload type: kEthernet (0) or kAtCommand (1)
        FrameType GetType() const {
            return static_cast<FrameType>((raw[2] >> 2) & kFrameTypeMask);
        }
        void SetType(FrameType type) {
            raw[2] = (raw[2] & (uint8_t)~(kFrameTypeMask << 2)) |
                     ((static_cast<uint8_t>(type) & kFrameTypeMask) << 2);
        }

        uint8_t GetChecksum() const { return raw[3]; }
        void SetChecksum(uint8_t sum) { raw[3] = sum; }

        uint8_t CalculateChecksum() const {
            uint32_t sum = raw[0] + raw[1] + raw[2];
            return static_cast<uint8_t>((sum >> 8) ^ sum ^ 0x03);
        }

        bool ValidateChecksum() const {
            return GetChecksum() == CalculateChecksum();
        }
        void UpdateChecksum() { SetChecksum(CalculateChecksum()); }
    } __attribute__((packed));

    static_assert(sizeof(FrameHeader) == 4, "FrameHeader must be 4 bytes");

    // SPIC protocol constants (mirrors EC718 ccio_spic.h)
    static constexpr uint32_t kSpicMagic32 = 0x43495053;
    static constexpr uint16_t kSpicGenNotificationMsg = 0x0001;
    static constexpr uint16_t kSpicGenStatusIndMsg = 0x8001;
    static constexpr uint16_t kSpicNetResetCmplt = 0x8401;
    static constexpr uint16_t kSpicNetInitialMsg = 0x0402;
    static constexpr uint16_t kSpicNetInitialCmplt = 0x8402;
    static constexpr uint32_t kSpicStatusMediaConn = 0xf0000003;
    static constexpr uint32_t kSpicStatusMediaDisconn = 0xf0000004;
    static constexpr uint32_t kSpicNotifFlowCtlOn = 0x70000001;
    static constexpr uint32_t kSpicNotifFlowCtlOff = 0x70000002;

    // Frame sizes (mirrors EC718 sfdt.h)
    static constexpr uint16_t kSfdtFrameMtu = 1600;          // includes 4B header
    static constexpr uint16_t kSfdtHeaderSize = 4;
    static constexpr uint16_t kMaxPayloadLen = kSfdtFrameMtu - kSfdtHeaderSize;
    static constexpr uint32_t kSfdtMinTransferDummy = 256;   // min clocked bytes per window
    static constexpr uint8_t kSfdtFillByte = 0xF0;           // matches EC718 SET_DEFAULT_TX_VALUE 0xF0F0

    // TX queue element: buffer layout [4B header][payload], allocated from DMA heap
    struct TxQueueItem {
        uint8_t* buffer;      // malloc'ed buffer with header + payload
        uint16_t payloadLen;  // payload length (not counting header)
        FrameType type;       // payload class (ETH or AT)
    };

    // Initialization and cleanup
    esp_err_t InitSpi();
    esp_err_t InitGpio();
    esp_err_t InitIotEth();
    void DeinitSpi();
    void DeinitGpio();
    void DeinitIotEth();

    // Tasks
    void TransferTaskRun();   // SFDT master engine (1:1 port of sfdt_master.c task)
    void InitTaskRun();       // SPIC handshake + AT init sequence + iot_eth bring-up
    void DoTransferWindow(uint32_t& txSeqNo);  // one SFDT window (handshake pulse + clock)
    void NoteRxCongestion();  // assert master->slave XOFF (download backpressure)

    // AT-based modem initialization, mirrors UartEthModem::RunNormalModeInitSequence().
    // Runs over the SFDT AT channel after the SPIC transport handshake; ends with
    // AT+ECNETDEVCTL=2,1,1 which dials the network device.
    esp_err_t RunNormalModeInitSequence();
    esp_err_t RunSpicHandshake();  // SPIC NET_INITIAL retry loop (needs kEventStart)
    esp_err_t SendAtWithRetry(const std::string& cmd, std::string& response,
                              uint32_t timeout_ms, int max_retries);
    esp_err_t AtDetect();
    bool CheckSimCard();
    bool WaitForRegistration(uint32_t timeout_ms);
    void QueryModemInfo();
    void ParseAtResponse(const std::string& response);

    // Event handling
    void HandleSfdtPayload(uint8_t* payload, uint16_t length, FrameType type);
    void HandleSpicMessage(const uint8_t* payload, uint16_t length);
    void HandleAtPayload(const uint8_t* payload, uint16_t length);
    void NotifyLink(bool up);

    // TX path (called from iot_eth transmit = LWIP context)
    esp_err_t EnqueueTxFrame(const uint8_t* buf, size_t len);
    esp_err_t EnqueueTxFrame(const uint8_t* buf, size_t len, FrameType type);
    // TX path bypassing the handshake gate (used by the init task for SPIC handshake)
    esp_err_t EnqueueRawFrame(const uint8_t* payload, uint16_t len);

    // GPIO helpers
    void SetMrdy(bool high);
    bool IsSrdyHigh() { return gpio_get_level(config_.srdy_pin) == 1; }
    static void IRAM_ATTR SrdyIsrHandler(void* arg);

    // State management
    void SetNetworkEvent(SpiEthModemEvent event);

    // Resource cleanup
    void CleanupResources();

    // IP event handler
    static void IpEventHandler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);

    // Configuration
    Config config_;

    // Network initialization flag (atomic for thread safety)
    std::atomic<bool> initialized_{false};

    // Network event (atomic for thread safety)
    std::atomic<SpiEthModemEvent> network_event_{SpiEthModemEvent::Disconnected};

    // Synchronization primitives
    std::mutex at_mutex_;                             // serializes SendAt() callers
    EventGroupHandle_t event_group_ = nullptr;   // task lifecycle + network events
    portMUX_TYPE srdy_mux_ = portMUX_INITIALIZER_UNLOCKED;

    // AT command/response state
    std::string at_command_response_;                 // accumulated AT response text
    std::atomic<bool> waiting_for_at_response_{false};// true while SendAt() waits
    int cell_reg_stat_ = 0;                           // last +CEREG stat (1/5 = registered)

    // SRDY is a push-pull output driven by the EC718 and produces exactly one
    // posedge per armed window; the historical 31.8 kHz "storm" was
    // self-inflicted by configuring the pin as a wake source (esp-idf turns
    // that into a level interrupt).

    // SFDT wake sources are direct task notifications (no event group):
    //   SRDY posedge    -> vTaskNotifyGiveFromISR (ISR)
    //   TX frame queued -> xTaskNotifyGive
    //   stop request    -> xTaskNotifyGive
    // The transfer task re-checks state (srdy_mask_, TX queue, stop_flag_)
    // after every wake, so a leftover notification is harmless.


    // Lifecycle/network event bits (event_group_)
    static constexpr uint32_t kEventStart = (1 << 0);
    static constexpr uint32_t kEventHandshakeDone = (1 << 1);
    static constexpr uint32_t kEventStop = (1 << 2);
    static constexpr uint32_t kEventNetworkReady = (1 << 3);
    static constexpr uint32_t kEventNetworkEventChanged = (1 << 4);
    static constexpr uint32_t kEventInitDone = (1 << 5);
    static constexpr uint32_t kEventAtResponse = (1 << 6);

    // Grace window after an AT result code, to absorb trailing response frames
    // (per-frame delivery) and late frames of a previously timed-out command.
    static constexpr uint32_t kAtDrainMs = 10;

    static constexpr uint32_t kEventTransferTaskDone = (1 << 8);
    static constexpr uint32_t kEventInitTaskDone = (1 << 10);

    static constexpr uint32_t kEventAllTasksDone =
            kEventTransferTaskDone | kEventInitTaskDone;

    // SFDT master engine state (owned by TransferTaskRun only)
    volatile uint32_t srdy_mask_ = 0;        // guards against double SRDY counts (gMasterSrdyMask)
    bool slave_flow_control_off_ = false;    // gSlaveFlowControlState (true = XOFF)
    uint32_t xoff_events_ = 0;               // ETH XOFF assertions seen (diagnostics)
    uint32_t xoff_log_ms_ = 0;               // rate limit for the XOFF transition log

    // Master -> slave flow control (download direction). Asserted when the
    // ESP32's own RX path drops a frame (lwIP mailbox full / pbuf alloc
    // failure). It rides the XOFF bit of every ETH header we send; the slave
    // stops picking its ETH TX buffers while it is set (sfdt_slave.c:533).
    // AT traffic is untouched in both directions.
    bool master_xoff_ = false;
    bool fc_pending_ = false;                // next window must carry the new FC state
    TickType_t master_xoff_until_ = 0;       // auto-clear deadline
    uint32_t rx_congest_events_ = 0;         // diagnostics
    uint32_t rx_congest_log_ms_ = 0;         // rate limit
    static constexpr uint32_t kMasterXoffHoldMs = 10;  // one lwIP mailbox drain
    uint8_t* window_tx_buffer_ = nullptr;    // DMA tx scratch buffer, kSfdtFrameMtu bytes
    uint8_t* window_rx_buffer_ = nullptr;    // DMA rx scratch buffer, kSfdtFrameMtu bytes
    uint8_t* rx_assembly_buffer_ = nullptr;  // multi-window frame assembly (plain heap)
    uint32_t pending_rx_count_ = 0;          // continuation bytes still missing of current frame
    uint32_t rx_frame_payload_len_ = 0;      // payload length of in-flight (partial) frame
    spi_transaction_t tx_transaction_ = {};

    // Task handles
    TaskHandle_t init_task_ = nullptr;
    TaskHandle_t transfer_task_ = nullptr;

    // TX queues. ETH frames are gated by the peer's XOFF bit; AT traffic has
    // its own queue so it never sits behind, and is never dropped because of,
    // ETH flow control. The slave routes AT payloads straight to its AT engine
    // and only applies the XOFF bit to ETH frames (sfdt_slave.c:676).
    // ETH depth mirrors EC718 MASTER_TX_MSGQUEUE_CAPACITY = 32.
    QueueHandle_t tx_queue_ = nullptr;
    QueueHandle_t at_queue_ = nullptr;
    static constexpr size_t kTxQueueDepth = 32;
    static constexpr size_t kAtQueueDepth = 8;

    // State flags (atomic for thread safety)
    std::atomic<bool> stop_flag_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> handshake_done_{false};
    std::atomic<bool> link_up_{false};       // netif can accept frames (SPIC WAN status)
    std::atomic<uint8_t> seq_no_{0};
    std::atomic<bool> debug_enabled_{false};

    // MAC address
    uint8_t mac_addr_[6] = {0};

    // Callback
    SpiEthModemEventCallback network_event_callback_;

    // iot_eth related
    iot_eth_driver_t driver_{};
    iot_eth_handle_t eth_handle_ = nullptr;
    iot_eth_mediator_t* mediator_ = nullptr;
    iot_eth_netif_glue_handle_t glue_ = nullptr;
    esp_netif_t* eth_netif_ = nullptr;
    esp_event_handler_instance_t ip_event_handler_instance_ = nullptr;
    spi_device_handle_t spi_dev_ = nullptr;
    bool bus_initialized_ = false;

    // Timing constants
    static constexpr uint32_t kMrdyHandshakeTimeoutMs = 200; // first MRDY pulse wait (sfdt_master.c)
    static constexpr uint32_t kMrdyHandshakeEscalationMs = 1000;
    static constexpr uint32_t kSpicHandshakeTimeoutMs = 5000; // per NET_INITIAL attempt
    static constexpr uint32_t kSpicHandshakeTotalMs = 60000;  // EC718 boot window
    static constexpr uint32_t kSpicHandshakeRetryMs = 1000;

    static constexpr const char* kTag = "SpiEthModem";
};
