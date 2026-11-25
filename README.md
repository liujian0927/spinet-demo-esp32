# EC718 SFDT SPI 4G 模组示例（ESP32-C3 主机）

ESP32-C3 通过一条 SPI 链路把 EC718（移芯 Cat.1 模组）当作 4G 网卡使用的完整示例：
以太网数据和 AT 命令共用同一条 SPI，内置拨号初始化、DHCP、上下行测速，以及主控侧的
软流控和 lwIP 参数调优。

- **主机**：ESP32-C3（ESP-IDF 5.5+ / lwIP）
- **从机**：EC718 运行 `spi_2boards`（SFDT slave 固件，不在本仓库）
- **协议**：SFDT（Ethernet Over SPI）+ MRDY/SRDY 窗口握手

## 特性

- AT 初始化/拨号（`CFUN` / `CPIN?` / `CEREG` / `ECNETDEVCTL`）→ SPIC 握手 → DHCP
- ETH 与 AT 两类帧共用一条 SPI，靠帧头 `type` 区分
- 全双工 DMA，每窗 256~1600 字节
- 双向软流控（帧头 `flowControl` 位），AT 通道不参与
- 16 MHz 实测：下载约 **995 KB/s**，上传约 **500 KB/s**（FDD 卡）
- 附带下载 / 上传测速用例（2 MB 上传），**测速服务器地址需自行配置**

## 硬件连接（ESP32-C3）

| 信号 | ESP32-C3 | EC718 | 说明 |
|---|---|---|---|
| SCLK | GPIO4 | SPI0 CLK | |
| MOSI | GPIO6 | SPI0 MOSI | 同名直连，无需交叉 |
| MISO | GPIO5 | SPI0 MISO | 同名直连，无需交叉 |
| CS | GPIO3 | SPI0 SS | **必须连接**，硬件自动 CS |
| MRDY | GPIO10 | MRDY | 主机请求窗口，空闲低 |
| SRDY | GPIO7 | SRDY | 从机就绪，**上升沿中断** |
| RESET | GPIO0 | RESET | 低电平复位 |
| 按键 | GPIO1 | — | 复位按键（可选） |

> **SPI 必须使用 DMA 整窗连续时钟。** EC718 是 SPI 从机，接收完全依赖主控的 SCLK；
> 用 CPU 逐字节/分段打时钟会在段间结束 transaction，导致从机短帧、丢帧。
> 详见移植指南 2.1 节。

## 快速开始

要求 ESP-IDF v5.5+：

```bash
idf.py set-target esp32c3
idf.py build
idf.py -p COMx flash monitor
```

启动流程：

```
复位 EC718（10 ms 低 + 1000 ms 稳定）
  → AT 初始化/拨号
  → SPIC NET_INITIAL 握手
  → DHCP 获取 IP
  → 执行下载 / 上传测速（需先配置自己的服务器地址）
```

### 配置测速服务器（必读）

本仓库**不包含测速服务器**，`main.cc` 中原有的测试地址已清空（当前为 `"http://"`），
直接烧录不会产生有效的测速结果。测试前请改成你自己的 HTTP 服务器：

| 测试 | 修改位置 | 要求 |
|---|---|---|
| 下载测速 | `app_main()` 中的 `test_url` | 可下载文件的 URL，如 `http://<你的服务器>/speedtest.bin`；文件建议 ≥1 MB |
| 上传测速 | `TestHttpClientUpload("http://")` | 接受 `POST` 的 URL，程序会流式上传 2 MB |

HTTP/HTTPS 均可（客户端已挂载 `esp_crt_bundle`）。只验证链路时，可把 `main.cc` 顶部的
`USE_HTTP_DOWNLOAD_TEST` / `USE_HTTP_UPLOAD_TEST` 置 `0` 跳过测速。

> 文末“实测性能”是使用自建/临时测试服务器得到的参考值，实际结果取决于服务器带宽、
> 运营商网络和信号质量。

## 关键配置

见 `sdkconfig.defaults`：

| 配置 | 值 | 说明 |
|---|---|---|
| `CONFIG_LWIP_TCP_TMR_INTERVAL` | 500 | TCP 慢定时器 = 2×TMR |
| `CONFIG_LWIP_TCP_RTO_TIME` | 2000 | RTO = 2 s，覆盖 4G 上的 RTT 尖峰 |
| `CONFIG_LWIP_TCP_SND_BUF_DEFAULT` | 65535 | 上行发送缓冲（无窗口缩放上限） |
| `CONFIG_LWIP_TCP_WND_DEFAULT` | 65535 | 下行接收窗口（无窗口缩放上限） |
| `CONFIG_LWIP_TCP_RECVMBOX_SIZE` | 47 | 按 `WND/MSS + 2` 计算 |
| `CONFIG_ESP_NETIF_RECEIVE_REPORT_ERRORS` | y | 让驱动感知收包失败，供流控使用 |
| SPI 时钟 | 16 MHz | `main.cc` 中配置 |

## 实测性能（16 MHz）

| 方向 | FDD 卡 | TDD（移动）卡 |
|---|---|---|
| 下载 1.71 MB | 1.76~1.77 s ≈ **991~998 KB/s** | 约 800 KB/s |
| 上传 2 MB | 3.72~4.08 s ≈ **502~551 KB/s** | 约 115 KB/s（上行带宽受限） |

两种卡下均 0 错误；TDD 上传受空口上行能力限制，与固件配置无关。

## 目录结构

```
spinet-demo-esp32/
├── main/
│   ├── main.cc                  # 入口：复位、AT 初始化、SPIC 握手、测速
│   ├── spi_eth_modem/           # EC718 SFDT SPI 网卡驱动（本示例核心）
│   │   ├── spi_eth_modem.h
│   │   └── spi_eth_modem.cc
│   ├── CMakeLists.txt           # main 组件构建脚本
│   └── idf_component.yml        # 组件依赖（espressif/iot_eth）
├── docs/
│   └── EC718_SFDT_SPI_AT_移植指南.md
├── partitions.csv               # 分区表
├── sdkconfig.defaults           # 默认 Kconfig（lwIP / mbedTLS 等）
├── CMakeLists.txt               # 工程入口：project(http-client)
└── README.md
```

> `build/`、`sdkconfig`、`sdkconfig.old`、`dependencies.lock`、`managed_components/`
> 均为构建或组件管理器生成的产物，已在 `.gitignore` 中忽略，不纳入版本管理。

## 文档

- [EC718 SFDT SPI + AT 通道移植指南](docs/EC718_SFDT_SPI_AT_移植指南.md)
  - 帧格式与窗口规则
  - MRDY/SRDY 握手（主机状态机、典型时序、超时处理）
  - 初始化与 AT 命令序列
  - 主控 TCP/IP（lwIP）配置与实测依据
  - 双向软流控的实现与坑
  - 移植检查清单

## 说明

- 本仓库只包含**主控（ESP32）侧**代码；EC718 的 `spi_2boards` 从机固件是独立的，不在此仓库。
- AT 通道不依赖 SPIC 握手：SPI 通了就能发 AT，SPIC 握手只启动网卡数据路径。
- 低功耗：SRDY 可配置为 light-sleep 唤醒源。

## License

Apache-2.0（见源文件头部 SPDX 标识）。
