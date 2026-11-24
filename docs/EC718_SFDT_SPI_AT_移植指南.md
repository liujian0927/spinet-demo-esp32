# EC718 SFDT SPI 网卡 + AT 通道 —— 主控 MCU 移植指南

适用对象：把 EC718（SPI 从机）当作 4G 网卡使用的主控 MCU。

- 参考实现：ESP32-C3 主机，工程 `http3-demo-esp32`（ESP-IDF 5.5 + lwIP）。
- EC718 固件 `spi_2boards` 视为黑盒：本文只描述它对外的可观察行为，不涉及从机内部实现。
- 协议部分与具体 MCU 无关；第 7 章的 TCP/IP 参数按 lwIP 给出，其它协议栈可按同样的原则换算。

---

## 0. 一页速览

```
复位 EC718 → 启动 SFDT 传输引擎 → AT 初始化/拨号 → SPIC NET_INITIAL → DHCP → 上网
```

| 项目 | 结论 |
|---|---|
| 物理链路 | SPI（模式 1）+ MRDY/SRDY 两根握手线 + RESET，全双工；**SPI 必须 DMA 整窗连续时钟** |
| 窗口大小 | 每窗 256 ~ 1600 字节 |
| 帧类型 | `type=0` 以太网 / `type=1` AT，共用一个窗口 |
| AT 通道 | 不依赖 SPIC 握手，SPI 通了就能发 |
| 流控 | 帧头 1 bit，双向；AT 不参与 |
| 16 MHz 实测 | 下载约 950 KB/s，上传约 500 KB/s（4G 上行受限） |

---

## 1. 总体思路

一条 SPI 链路同时承载两类帧：

| 帧类型 (`type`) | 用途 |
|---|---|
| 0 = ETH | 以太网帧（ARP / DHCP / IP / TCP…），以及 SPIC 控制消息（载荷以 `"SPIC"` 开头） |
| 1 = AT | 与 EC718 AT 引擎交互的文本命令 / 响应 |

三条关键结论（均经实测验证）：

1. **AT 通道不依赖 SPIC 握手**。从机收到 `type=AT` 的窗口后直接交给自己的 AT 任务，不经过网卡数据路径的状态检查。
2. **SPIC 握手（NET_INITIAL）相当于“网卡数据路径启动”命令**，要放在 AT 初始化序列的**最后**。
3. 因此主机流程固定为：**复位 → 启动传输引擎 → AT 初始化/拨号 → SPIC 握手 → DHCP → 上网**。

---

## 2. 硬件连接

| 信号 | 方向 | 说明 |
|---|---|---|
| SCLK / MOSI / MISO / CS | 主机 ↔ 从机 | SPI 模式 1（CPOL=0, CPHA=1），建议 16 MHz |
| MRDY | 主机 → 从机 | 主机请求窗口。空闲**低**，脉冲（高→低）触发，从机上升沿唤醒 |
| SRDY | 从机 → 主机 | 从机就绪。**上升沿**表示窗口已武装，必须配边沿中断 |
| RESET | 主机 → 从机 | 低电平复位 |

要点：

- **SRDY 必须用上升沿中断**，不能用电平中断，也不能用“高电平=可收”来判断（见 4.6）。
- MRDY 是推挽输出，空闲低；SRDY 是输入，建议内部下拉。
- SPI 建议先跑 16 MHz；降频只会降低上限，不会改善 4G 侧的吞吐。

### 2.1 SPI 必须使用 DMA（强制要求）

EC718 是 SPI **从机**，SCLK 完全由主控提供。从机每个窗口武装的是固定 **1600 字节**的收发 DMA，
并依赖主控把整个窗口的时钟**连续打完**。因此主控必须用 **SPI 外设 + DMA** 完成每个窗口：

- 一个窗口 = **一次连续的 SPI 传输**，长度 `txNum × 8` bit（256~1600 字节），中间不能停；
- **不要**用 GPIO 模拟 SPI、按字节/小段轮询、在一个窗口内多次拉 CS，或把一次窗口拆成多次 transaction；
- 收发都要 DMA（全双工），缓冲放在 DMA 可访问内存并对齐，TX/RX 长度一致；
- 传输完成用 DMA 完成中断通知，CPU 只在传输结束后解析 RX 缓冲。

**为什么必须 DMA**：从机的接收完全依赖主控的 SCLK。SCLK 上的间隙本身不会让同步 SPI 错位，
但用 CPU 逐字节/逐段打时钟时，任务切换、中断和循环开销会在段间制造间隙，而 PIO/分段驱动
通常会在间隙处结束一次 transaction（拉高 CS 或重配外设），从机就会看到不完整的窗口：

| 现象 | 后果 |
|---|---|
| 一次窗口被拆成多次 transaction | 从机按短帧解析，`GetDataCount()` 小于本窗长度 → 校验失败、丢帧 |
| 段间 CS 拉高 / 重配外设 | 从机把它当成多个不完整窗口，帧错位 |
| 间隙超过 200 ms | 从机窗口超时，重新拉 SRDY，本窗数据作废 |

结论：**主控 SPI 必须 DMA 整窗连续时钟**。这是 SFDT 链路能稳定工作的前提，不是性能优化项。

---

## 3. 链路层：帧格式与窗口规则

### 3.1 帧头（4 字节，小端位域）

| 位域 | 位宽 | 说明 |
|---|---|---|
| payLoadLen | 12 | 载荷长度（不含 4 字节头），byte0 + byte1 低 4 位 |
| seqNo | 4 | 调试用，从机不校验 |
| flowControl | 1 | 0 = XON 允许发送；1 = XOFF 停止发送（见第 8 章） |
| bContinue | 1 | 本帧之后还有后续窗口 |
| type | 2 | 0 = ETH，1 = AT |
| resv | 4 | 0 |
| checkSum | 8 | `((b0+b1+b2) >> 8) ^ (b0+b1+b2) ^ 0x3` |

示例：12 字节 SPIC NET_INITIAL（`seqNo=3`）

```
0C 30 00 3F | 53 50 49 43 02 04 0C 00 00 00 00 00
```

### 3.2 窗口长度

```
txNum = max(payloadLen + 4, 待接收续传字节数, 256)
txNum = min(txNum, 1600)
```

- 从机每个窗口固定武装 1600 字节的接收缓冲，主机不得时钟超过 1600；
- 载荷之后不足 `txNum` 的部分填 `0xF0`；
- 从机空闲窗口会回 `0xF0F0...` 或 `00 00 00 00`，**校验失败一律静默丢弃**，不是错误。

### 3.3 多窗口帧

一帧可以跨多个窗口：

- 主机发送：从机用 `bContinue` 提示“后面还有”，主机继续等窗口；
- 从机发送：主机按 `pendingRxCnt` 记录剩余字节，把后续窗口的载荷**按绝对偏移拼接到同一个缓冲**，不能直接用单窗口缓冲投递。

---

## 4. 窗口握手（MRDY / SRDY）

每次数据传输称为一个“窗口”。SPI 时钟本身不携带开始/结束语义，窗口何时开始完全由这两根线协商。

### 4.1 信号语义

- **MRDY**：主机请求一个窗口（“我要发数据 / 叫醒你”）。
- **SRDY**：从机已经武装好 SPI DMA，请主机来时钟。
- 两者都是“电平 + 边沿”双重信息，但协议只允许用 **SRDY 上升沿**启动传输。

### 4.2 三条铁律

1. **一个 SRDY 上升沿 = 一个窗口**。主机每传一窗消耗一个边沿，下一次必须等新边沿。
2. **只有新的上升沿才能启动时钟**。看到 SRDY 高就去收，会踩进从机换窗空档而丢帧。
3. **MRDY 只用于唤醒空闲从机**。从机自己有数据要发时会主动武装并拉高 SRDY，主机无需发 MRDY。

> **补充（硬性）**：一个窗口的 `txNum` 字节必须由主控 **DMA 连续时钟**，不能中途停顿或拆包（见 2.1）。

### 4.3 从机的对外行为（主机只需要知道这些）

| 场景 | 从机行为 |
|---|---|
| 有 AT 数据 | 优先于 ETH 发送，帧头 `type=AT` |
| 有 ETH 数据 | 发 ETH 帧，帧头携带自己的流控状态 |
| 完全没数据 | 阻塞，**SRDY 保持低**，等待 MRDY 或自身数据触发 |
| 武装完成 | `SRDY=1`，等待主机时钟 |
| 主机时钟完成 | `SRDY=0`，解析载荷后进入下一轮 |
| 超时未被时钟 | 200 ms → +1000 ms → 重新拉高 SRDY |
| 本窗没发完 | 下一窗继续发同一帧的尾部（帧跨窗） |
| 收到坏校验 | 静默丢弃 |

> 从机的 AT 帧头**恒为 XON**，ETH 帧头才携带真实流控状态。主机解析流控时必须以 `type=ETH` 为条件（见第 8 章）。

### 4.4 主机状态机

```c
SetMrdy(false);                        // 空闲低
while (!stop) {
    timeout = 200;
    triggeredBySrdy = false;
    continuation = (pendingRxCnt != 0);

    /* 下载方向 XOFF 到期 → 清标志并强制一个窗口把 XON 送回去 */
    if (master_xoff && expired(master_xoff_until)) {
        master_xoff = false;
        fc_pending = true;
    }

    if (!continuation && !fc_pending) {
        // XOFF 期间等待必须带超时，否则从机也闲着时任务永远睡着
        wait(TX_QUEUED | SRDY_EDGE | XOFF_EXPIRY, ...);
        if (got SRDY_EDGE) triggeredBySrdy = true;
        if (fc_pending) { /* 直接去握手，强制一个窗口 */ }
    }

    if (!triggeredBySrdy) {
        srdyHighAtGate = !continuation && IsSrdyHigh();
        do {
            if (pulsedOnce || (!continuation && !srdyHighAtGate))
                SetMrdy(true);                     // 请求窗口
            flags = wait(SRDY_EDGE | STOP, timeout);
            SetMrdy(false);
            if (timeout) { timeout += 1000; retry; }
        } while (!(flags & SRDY_EDGE));
    }

    /* 传输 */
    if (fc_pending) {
        不 pop 任何帧，发 0 载荷 ETH 帧头（FC 切换必须走 ETH 帧头）
    } else {
        先 pop AT；队列空且从机未 XOFF 时再 pop ETH
    }
    组头：len / seq / type / bContinue / flowControl(master_xoff) / csum;
    txNum = max(payloadLen + 4, pendingRxCnt, 256);
    txNum = min(txNum, 1600);
    载荷之后填 0xF0;
    srdy_mask_ = 0;                            // 清除已消费边沿
    spi full-duplex(txBuf, rxBuf, txNum);
    fc_pending = false;                        // 本窗已携带新的 FC 状态

    /* 收尾 */
    校验 RX 头 → 单窗完成则分发，跨窗则记 pendingRxCnt 并按偏移拼接;
}
```

参考实现用**任务通知**而不是事件组：SRDY ISR 和 TX 入队各给传输任务一次通知，任务被唤醒后再检查 `srdy_mask_` / 队列。

主机判定表：

| 条件 | 动作 | 说明 |
|---|---|---|
| 有新 SRDY 边沿标志 | **直接传输，不发 MRDY** | 从机已经武装好 |
| 无新边沿 + SRDY 低 | 拉高 MRDY → 等边沿 → 拉低 | 从机空闲，必须请求 |
| 无新边沿 + SRDY 高 | 先不发 MRDY，只等新边沿；超时才补脉冲 | 高电平大概率是上一窗残高 |
| 续传窗口（`pendingRxCnt ≠ 0`） | 先等边沿，超时才补 MRDY | 从机正在发尾部，会自己重新武装 |
| 等待超时 | `timeout += 1000` 后重试 | 200 → 1200 → 2200 ms |
| 从机 XOFF | 不出队 ETH，只发空窗口 | 见第 8 章 |

### 4.5 典型时序

**A. 从机主动发送（下载主路径，无 MRDY）**

```
主机                                  从机
 |                                     | 有数据，武装 DMA(1600)
 | <──────── SRDY 上升沿（中断） ───────┘
 | ═══════ 全双工时钟 txNum 字节 ═══════> |
 | （同时把主机的 ACK 送给从机）          | SPI 完成 → SRDY=0
```

**B. 主机主动发送（从机空闲，必须 MRDY）**

```
主机                                  从机
 | 有 TX 帧，无 SRDY 边沿               | 阻塞，SRDY 低
 | MRDY 脉冲 ─────────────────────────> | 唤醒 → 武装 → SRDY=1
 | <──────── SRDY 上升沿 ───────────────|
 | ═══════ 全双工时钟 ═════════════════> |
```

**C. 双方都有数据（最常见）**

从机拉高 SRDY，主机直接用这个边沿传输，自己的 TX 帧搭同一窗发出去，**不需要 MRDY**。

**D. 续传（从机帧跨窗）**

```
第 1 窗：从机发 346 B，主机只钟 256 B
  → 主机 pendingRxCnt = 94
第 2 窗：主机不 pop 新帧，txNum = max(4, 94, 256) = 256
  → 从机自己重新武装并拉高 SRDY
  → 主机等新边沿 → 时钟 → 按偏移拼接 → 分发
```

### 4.6 超时与异常

| 事件 | 主机处理 |
|---|---|
| 等不到 SRDY 边沿 | 200 ms 超时 → `timeout += 1000` → 重发 MRDY |
| 从机重启 / 未就绪 | 首次 AT 前常见 1 次 MRDY 超时（约 450 ms 处），属正常 |
| 收到坏校验 | 丢弃，视为空窗 |
| 帧跨窗 | 按 `pendingRxCnt` 续收并拼接 |
| 从机 XOFF | 停发 ETH，继续 clock 空窗；XON 后恢复 |

**绝对不能做的事**：用 SRDY 电平代替边沿。从机在传输前置高、传输完成后才拉低，完成到任务写低之间有空档，此时下一次 DMA 尚未武装；在残高窗口时钟会收到 `00 00 00 00` / `ff ff ff ff`，从机计数器不前进，整帧永久丢失。

### 4.7 SRDY 边沿去重

```c
// ISR
if (srdy_mask_ == 0) { srdy_mask_ = 1; notify(transfer_task); }

// 每个窗口传输前
srdy_mask_ = 0;
```

作用：从机一次 SRDY 高电平期间可能因超时重发多个上升沿，主机保证**一个窗口只消费一个边沿**。

---

## 5. 初始化流程

### 5.1 复位 EC718

```
RESET 拉低 10 ms → 拉高 → 等 1000 ms → 开始 SPI/AT 流程
```

释放后约 1.0 s 从机 AT 任务就绪，会主动上报 `\r\nECRDY\r\n`，属正常。

> 如果主控和模组之间**没有接 RESET 线**，可以跳过复位；此时模组可能仍在运行、网卡已经启动，
> 初始化时要靠第 7 步的 `AT+ECNETDEVCTL?` 判断是否需要拨号。

### 5.2 启动 SFDT 传输引擎

- TX 队列（建议深度 32）、AT 独立队列（建议 8）；
- DMA 收发缓冲、SRDY 边沿中断、传输任务；
- 此时不需要 SPIC 握手，AT 帧即可收发。

### 5.3 AT 初始化序列

| # | 命令 | 超时 | 判定 / 失败处理 |
|---|---|---|---|
| 1 | `AT` | 2 s × 4 次 | 收到 OK 即检测成功；从机启动约 1 s，首次超时属正常 |
| 2 | `AT+CFUN=1` | 3 s | OK |
| 3 | `AT+ECNETCFG?` | 1 s，最多 5 次（间隔 300 ms） | 见下方 NAT 说明 |
| 4 | `AT+CPIN?` | 1 s，最多 10 次（间隔 1 s） | `+CPIN: READY` 即就绪；`+CME ERROR: 14` = SIM 忙，继续等；`10` = 无卡 |
| 5 | `AT+CGSN=1` / `AT+ECICCID` / `AT+CGMR` | 1 s | 仅日志 |
| 6 | `AT+CEREG=2` 后轮询 `AT+CEREG?` | 1 s/次，总 60 s | `stat` = 1（本地）或 5（漫游）即注册；3 = 拒绝 |
| 7 | `AT+ECNETDEVCTL?` → 若状态 ≠ 1，发 `AT+ECNETDEVCTL=2,1,1` | 5 s | 拨号；若失败但链路已 Connected（主控重启、模组未重启，网卡仍在运行），继续 |

**NAT 配置（第 3 步）**

- 只查询、不盲目预置：刚启动时可能只回裸 `OK`，所以要重试 5 次；
- **只有明确返回 `"nat",0` 才预置**，否则按已配置继续（预置会重启模块）；
- 预置流程：

```
AT+ECPCFG="usbCtrl",1
AT+ECNETCFG="nat",1,"192.168.10.2"
等待 300 ms            ← 参考实现未发送 AT&W（ECNETCFG 已写入）；
                         若你的模组需要显式保存，先发 AT&W 再等 ≥300 ms
AT+ECRST               ← 模块重启
等待 1.5 s
重试 AT（最多 20 次，每次 500 ms） ← 重启后 AT 直接恢复，不需要重新握手
AT+CFUN=1
```

**第 7 步响应**

```
+ECNETDEVCTL: 2,1,1,1    ← 第 4 个字段 = 1，网卡已启动
+ECNETDEVCTL: 0,0,0,0    ← 第 4 个字段 = 0，需要拨号
```

**为什么要先查询**：主控和模组之间可能没有接 RESET 线（或者主控复位时没有拉低模组复位）。
主控重启时，EC718 可能仍在运行、网卡已经启动（第 4 个字段 = 1），此时直接跳过拨号，
避免重复拨号被模组拒绝；只有查询到状态为 0 时才需要发 `AT+ECNETDEVCTL=2,1,1`。

### 5.4 SPIC 数据路径握手

在 AT 序列**最后**执行，载荷 12 字节、`type=ETH`：

```
53 50 49 43   "SPIC" 魔数
02 04         消息类型 0x0402 = SPIC_NET_INITIAL_MSG
0C 00         len = 12（仅 SPIC 头，无 body）
00 00 00 00   reserved
```

从机返回 `SPIC_NET_INITIAL_CMPLT`（0x8402），body 布局：

| 偏移 | 字段 |
|---|---|
| 0 | sid (u32) |
| 4 | status (u32)，0 = 成功 |
| 8 | majVer (u16) |
| 10 | minVer (u16) |
| 12 | maxXferSize (u32) |
| 16 | maxPktsPerXfer (u16) |
| 18 | pktAlignValue (u16) |

- **判定只看 `status == 0`**；
- header-only 时从机会把窗口填充的 `0xF0` 当 body 回显（`sid=0xF0F0F0F0`、`ver=61680.61680`），不影响功能；也可以附带 12 字节 body（len 改为 0x18），两种写法都能起网卡；
- 握手要重试直到收到 CMPLT 或总超时。

### 5.5 链路就绪与 IP

- 从机随后上报 `SPIC_GEN_STATUS_IND_MSG`（0x8001）：`0xF0000003` = WAN media 连接，`0xF0000004` = 断开；
- 之后以太网帧正常收发，主机走 DHCP 获取 IP（EC718 内置 NAT/DHCP，默认 `192.168.10.2` 网关）；
- 收到 IP 后即可上网。

---

## 6. AT 通道实现

- **命令编码**：`"命令" + '\r' + '\0'`（结尾 NUL 让从机的 C 字符串处理安全，`\r` 是行结束）；
- **响应按行回帧**：一条响应可能分成多帧，不要假设一次只来一帧；
- **累积到结果码**：一直收，直到出现 `OK` / `ERROR` 或超时（默认 1 s）；
- **结果码后再等 3×10 ms 收尾**，吸收同一响应的拆帧和上一条超时命令的迟到响应；
- **首次 AT 检测超时放宽到 2 s**：超时命令的帧可能还在发送队列里，立即重发会造成重复命令、重复 OK；
- **AT 与 SPIC 解耦**：`AT+ECRST` 导致模块重启后，直接重试 `AT` 即可，无需重新握手。

---

## 7. 主控 MCU 的 TCP/IP 配置（以 lwIP 为例）

SFDT 链路的特点决定了 TCP 参数不能直接套用默认值：

1. SPI 带宽足够（16 MHz 约 1.5 MB/s），瓶颈在 4G 空口；
2. 4G 上行 RTT 约 40~60 ms，但存在偶发 RTT 尖峰；
3. 服务器不协商 SACK，丢一组包只能从洞开始整窗重发。

### 7.1 建议配置

| 配置项 | 值 | 原因 |
|---|---|---|
| `CONFIG_LWIP_TCP_TMR_INTERVAL` | **500** | `TCP_SLOW_INTERVAL = 2 × TMR`，RTO 和 RTT 采样都以这个 tick 为单位 |
| `CONFIG_LWIP_TCP_RTO_TIME` | **2000** | `RTO = 2000 / (2×500) = 2` ticks = 2 s |
| `CONFIG_LWIP_TCP_SND_BUF_DEFAULT` | **65535** | 无窗口缩放时的上限；降到 32768 实测更慢 |
| `CONFIG_LWIP_TCP_WND_DEFAULT` | **65535** | 下行接收窗口（无窗口缩放时的上限） |
| `CONFIG_LWIP_TCP_RECVMBOX_SIZE` | **47** | 按 `WND/MSS + 2 = 65535/1440 + 2` |
| `CONFIG_ESP_NETIF_RECEIVE_REPORT_ERRORS` | **y** | 否则 `esp_netif_receive()` 恒返回 OK，主机侧流控无法感知收包失败 |
| TCP MSS | 1440 | 与上面的窗口 / 邮箱公式配套 |
| SPI 时钟 | 16 MHz | 实测下载约 950 KB/s、上传约 500 KB/s |

### 7.2 为什么 RTO 要 2 s

lwIP 的 RTO 不是按 RTT 自适应的：它用慢定时器 tick（`tcp_ticks`）量 RTT，47 ms 的 RTT 不足 1 个 tick，采样值恒为 0，RTO 实际由下面两行决定：

```
TCP_SLOW_INTERVAL = 2 × TCP_TMR_INTERVAL
RTO(tick)         = LWIP_TCP_RTO_TIME / TCP_SLOW_INTERVAL
```

同一设备、同一网络、交错 A/B（`500 ms → 2 s → 500 ms → 2 s`）实测上传：

| TMR | SLOW | RTO_TIME | 实际 RTO | 上传 |
|---|---|---|---|---|
| 250 | 500 ms | 500 | 500 ms | 179 / 204 / 201 / 192 KB/s |
| 500 | 1000 ms | 1500 | 1 s | 400 KB/s |
| **500** | **1000 ms** | **2000** | **2 s** | **508 / 535 / 524 / 532 / 518 / 521 KB/s** |
| 500 | 1000 ms | 3000 | 3 s | 537 / 491 KB/s |

结论：**2 s 是本链路的折中值**；3 s 收益不再增加，1 s 以下明显变差。

> 这里的改善不是“ACK 被延迟”造成的。实测主机 RX 侧 ACK 到达间隔 ≤ 94 ms，与 EC718 侧抓包一致；
> 真正的原因是偶发 RTT 尖峰超过 500 ms 时，短 RTO 会误判丢包，整窗重发并把 `cwnd` 打到 1 MSS。

### 7.3 不要做的事

- **开 SACK**：服务器 SYN-ACK 不带 SACK-permitted，协商不上；
- **提高 `MEMP_NUM_TCP_SEG`**：实测上传从 ~450 KB/s 掉到 310~362 KB/s；
- **缩小 `SND_BUF`**：32768 实测 305~313 KB/s，BDP 不够。

### 7.4 怎么验证

- 在主机 RX 路径临时统计“相邻 ETH 帧到达间隔”，或直接抓 EC718 侧 pcap，确认 ACK 有没有被上游攒批；
- `tcp.analysis.retransmission` / `spurious_retransmission` 需要微秒级时间戳：1 ms 粒度的 pcap 在同一毫秒内顺序不可信。

---

## 8. 主控 MCU 的软件流控

流控位在每个 SFDT 帧头里（`flowControl:1`，0 = XON，1 = XOFF）。它是**发送方对自己接收能力的声明**：

- 从机在自己的 ETH 帧头里置 XOFF = “别再给我发 ETH”（控制**上传**，主机 → EC718）；
- 主机在自己的帧头里置 XOFF = “别再给我发 ETH”（控制**下载**，EC718 → 主机）。

### 8.1 两条队列

ETH 和 AT 必须分开，AT 不参与任何流控：

```c
QueueHandle_t tx_queue_;   // ETH，深度 32
QueueHandle_t at_queue_;   // AT， 深度 8
```

入队按类型分流；出队时 **AT 优先**，且 AT 不受 XOFF 约束。

### 8.2 上传方向：从机 XOFF

1. **只从 ETH 帧采样 FC**。从机的 AT 帧头恒为 XON，采信它会把真实 XOFF 误清掉，随后把帧灌进已满的从机接收缓冲而丢帧。
2. XOFF 时 **不出队 ETH**，但**继续 clock 空窗**（0 载荷 ETH 帧）——从机只有收到窗口才能把 XON 带回来，停掉窗口会死锁。
3. 收到 XON 且 ETH 队列非空时，唤醒传输任务恢复发送。
4. `bContinue` 只在“取出的是 ETH 且后面还有 ETH”时置位，避免 AT 造成无意义的续窗。

### 8.3 下载方向：主机 XOFF

1. **触发**：`stack_input()` 返回非 `ESP_OK`（lwIP 邮箱满 / pbuf 分配失败），或 RX `malloc` 失败。前提是打开 `CONFIG_ESP_NETIF_RECEIVE_REPORT_ERRORS`。
2. **门控**：只在拿到 IP（`IP_EVENT_ETH_GOT_IP`）之后才认为失败是拥塞；netif 未 up 时 `ethernetif_input()` 也会返回 `ESP_FAIL`，那是正常丢弃。
3. **保持**：置位 10 ms，期间每次新的失败都会把期限往后推。
4. **投递**：FC 状态只挂在 ETH 帧头。状态切换时会**强制发一个 0 载荷 ETH 窗口**（从机不读 AT 帧的 FC 位），AT 最多被推后一个窗口。
5. **恢复**：XOFF 期间没有 RX 帧，不能用“连续成功 N 次”来清，只能**定时清**；空闲等待也必须从 `portMAX_DELAY` 改成**带超时**，否则从机也闲着时任务永远睡着。
6. 清掉 XOFF 后要**主动要一个窗口**（SRDY 低就脉冲 MRDY），把 XON 送回去；否则从机停着、主机也停着，两边互等。

### 8.4 必须遵守的几条

| 规则 | 不遵守的后果 |
|---|---|
| 只从 ETH 帧采样 FC | AT 帧的 XON 会误清 XOFF，丢帧 |
| AT 走独立队列、不受 XOFF | AT 命令被 ETH 积压堵住，`SendAt` 超时 |
| XOFF 期间继续 clock 空窗 | 从机无法把 XON 带回来，死锁 |
| 清 XOFF 用定时 | XOFF 期间没有 RX 帧，永远等不到“成功” |
| 空闲等待带超时 | 任务睡死，定时清除永不执行 |
| 清 XOFF 后强制一个窗口 | 两边互等，死锁 |

### 8.5 诊断

- 主机日志（下载方向触发）：`RX congested, assert XOFF to slave (events=N, hold=10 ms)`；
- 主机日志（上传方向状态变化）：`Slave ETH flow control: XOFF/XON, events=N (ethq=N atq=N)`；
- 从机日志：`spid(0): flow ctrl on/off`（从机接收缓冲阈值）；
- 需要确认 ACK 是否被上游攒批时，可在主机 RX 路径临时统计帧到达间隔，或直接抓 EC718 侧 pcap。

---

## 9. 实测性能（16 MHz）

| 项目 | 结果 |
|---|---|
| 下载 | 1.71 MB / 1.76~1.77 s ≈ **991~998 KB/s**（FDD，WND 65535） |
| 上传 | 2.00 MB / 3.72~4.08 s ≈ **502~551 KB/s**（FDD，SND_BUF 65535） |
| SPI | 16 MHz（实际 16.000 MHz） |
| 错误 | 0（坏续传 / 丢帧 / 超时） |
| 空闲 SRAM | 约 265 KB |

上传受运营商上行能力限制：FDD 卡约 500 KB/s，TDD（移动）卡约 115 KB/s；下载受 4G 下行限制（FDD 约 995 KB/s，TDD 约 800 KB/s）。

冷启动时间线（16 MHz）：

| 时间 | 事件 |
|---|---|
| 0–10 ms | RESET 低电平 |
| 10 ms | 拉高，等 1000 ms |
| ~1010 ms | 开始 SPI/AT 流程 |
| ~1.0–1.2 s | 从机 `ECRDY`，`AT` 首次成功（1000 ms 稳定期已覆盖从机启动，通常不再出现 MRDY 超时） |
| 1.2–2.4 s | AT 初始化（CFUN / NETCFG / SIM / 信息 / 注册 / 拨号） |
| ~2.4 s | SPIC NET_INITIAL 完成 |
| ~3.4 s | Got IP（DHCP 完成；随网络注册时间波动，TDD 可能更晚） |

---

## 10. 移植检查清单

**硬性要求：SPI 必须使用 DMA 整窗连续时钟（见 2.1），否则从机接收会异常。**

1. SRDY 必须用上升沿中断；不要用电平中断，也不要用高电平当“已就绪”。
2. 每个窗口传输前清除“已消费边沿”锁，保证一窗一边沿。
3. 窗口最小 256、最大 1600；`txNum` 不得小于待接收续传字节数。
4. 跨窗帧必须按绝对偏移拼接，不能直接投递单窗口缓冲。
5. 校验失败视为空闲填充，静默丢弃。
6. AT 通道先于 SPIC 握手可用，不要把 AT 初始化卡在握手之后。
7. 若使用 `AT&W` 持久化，需等待 ≥300 ms 再 `AT+ECRST`（参考实现中 ECNETCFG 已写入，未发送 AT&W）。
8. `AT+ECRST` 后直接重试 AT，不需要重新握手。
9. AT 响应按“多帧累积 + 结果码 + 收尾窗口”处理。
10. 流控只从 ETH 帧采样；AT 走独立队列，不参与流控。
11. XOFF 期间继续 clock 空窗；清 XOFF 用定时，清完强制一个窗口。
12. TCP 参数按第 7 章配置，尤其是 RTO 的 tick 换算。
13. 先用 16 MHz 打通，不要为了“稳定”降频——降频只会降低上限。

---

## 11. 参考伪代码

### 11.1 传输主循环

见 4.4 的主机状态机。

### 11.2 SendAt

```c
int SendAt(const char *cmd, char *resp, int resp_size, int timeout_ms) {
    char payload[MAX_AT];
    int n = snprintf(payload, sizeof(payload), "%s\r", cmd);
    payload[n++] = '\0';                       // 结尾 NUL

    enqueue_at_frame(payload, n);              // type=AT，独立队列

    int got = 0;
    for (;;) {
        if (wait_at_response(timeout_ms) != OK) break;
        append(resp, got);
        if (strstr(resp, "\r\nOK\r\n") || strstr(resp, "\r\nERROR\r\n")) break;
    }
    drain_quiet(3, 10);                        // 最多 3 次 10 ms 静默窗口
    return contains(resp, "\r\nOK\r\n") ? 0 : -1;
}
```

### 11.3 初始化

```c
reset_slave();                 // 10 ms 低 + 1000 ms 延时
start_sfdt_engine();

if (send_at_retry("AT", ..., 2000, 4) != OK) fail();
send_at("AT+CFUN=1", ..., 3000);

/* NAT：重试查询，只有明确 nat=0 才预置 */
if (netcfg_reports_nat_0()) {
    send_at("AT+ECPCFG=\"usbCtrl\",1", ...);
    send_at("AT+ECNETCFG=\"nat\",1,\"192.168.10.2\"", ...);
    /* 参考实现未发 AT&W（ECNETCFG 已写入）；如需显式保存再加 AT&W */
    sleep(300);
    send_at("AT+ECRST", ...);
    sleep(1500);
    send_at_retry("AT", ..., 500, 20);
    send_at("AT+CFUN=1", ...);
}

wait_cpin_ready();                            // CME ERROR 14 继续等
query_info();                                 // CGSN / ECICCID / CGMR（可选）
send_at("AT+CEREG=2", ...);
wait_registered(60 s);                        // CEREG stat 1/5

if (netdev_state() != 1)
    send_at("AT+ECNETDEVCTL=2,1,1", ..., 5000);

spic_net_initial();                           // "SPIC" 0x0402 len=0x0C，等 CMPLT status=0
wait_link_up();                               // GEN_STATUS 0xF0000003
/* 之后交给 DHCP / 协议栈 */
```
