# Visionarm MCU 重构说明

## 目标

本次重构只整理 MCU 应用层，不修改 `portable/mcu_uart` 的线上协议、frame parser、消息 codec 与 transaction cache。UART/RS-485 的既有接收方式（RXNE 直接取字节、HAL 负责 TXE→TC）和 `ProtocolTxTask` 唯一 UART TX owner 也保持不变。

主要目标：

- 去掉 M1/M2/M3/M4/M5 阶段标签，文件名只表达当前职责。
- 收紧模块边界，避免多个全局 metrics/state 互相引用。
- 删除生产代码不读取的 bench、自测、RAM diagnostics 与历史兼容字段。
- 保留 STATUS 在线协议实际使用的统计字段。
- 将 RTOS 对象改为静态分配，关闭未使用的软件定时器、trace/task-inspection API 和动态 heap。
- 保持 `ACK/NACK > HELLO_ACK > PONG > STATUS > diagnostics` 的 TX 排队设计；当前协议没有 diagnostics message type，因此该最低等级仅保留为扩展位置，不存在 producer。

## 新模块边界

| 新文件 | 单一职责 |
|---|---|
| `app_config.*` | UART、任务、超时、缓冲区等平台配置 |
| `app_init.*` | 创建/初始化整个应用 |
| `rs485_uart.*` | USART2 + RS-485 DE/RE 硬件驱动 |
| `uart_rx_ring.*` | ISR→RX task 单生产者/单消费者字节环 |
| `protocol_engine.*` | portable parser 的 MCU 适配、frame encode、STATUS 所需 RX 统计 |
| `protocol_message.*` | Linux→MCU payload decode；可靠事务 payload CRC 在原始 payload 上直接计算 |
| `protocol_policy.*` | HELLO/CONTROL/CLEAR 的业务契约 |
| `protocol_state.*` | link/peer/sequence/stop/control 的运行状态 |
| `protocol_watchdog.*` | link freshness / control freshness |
| `protocol_dispatcher.*` | 消息→状态变化/响应/可靠事务的唯一业务路由 |
| `protocol_response.*` | 构造 HELLO_ACK/STATUS/ACK/NACK/PONG payload |
| `protocol_tx_task.*` | 响应优先级排队、MCU TX sequence、frame encode、唯一 UART TX owner |
| `protocol_rx_task.*` | ring drain、parser poll、resync、watchdog check |
| `control_mailbox.*` | 最新 CONTROL_UPDATE 快照；不再使用 FreeRTOS queue |
| `gimbal_stub.*` | 50 Hz RAM-only 控制消费者 |
| `app_fatal.*` | assert/stack overflow fail-stop |
| `stm32_irq.c` | SysTick 与 USART2 IRQ glue |

## 主要旧→新映射

- `r4_m1_config.h + r4_m2_config.h + r4_m3_config.h + r4_m5_config.h` → `app_config.h`
- `r4_m2_app.*` → `app_init.*`
- `protocol_core_m3.*` → `protocol_engine.*`
- `protocol_message_decode.*` → `protocol_message.*`
- `r4_m4_contract.*` → `protocol_policy.*`
- `r4_m4_state.*` → `protocol_state.*`
- `watchdogs.*` → `protocol_watchdog.*`
- `gimbal_control_stub.*` → `gimbal_stub.*`
- `r4_fatal.*` → `app_fatal.*`
- `r4_m1_irq.c` → `stm32_irq.c`
- `rs485_uart_hal.*` → `rs485_uart.*`

## 删除的生产无用内容

完全删除：

- `protocol_m3_bench.*`
- M1 TX self-test 配置
- M2/M3 bench responder 配置
- `r4_transport_metrics.*` 的整套 debugger-only 统计
- `r4_m5_diagnostics.*` RAM snapshot
- Gimbal stub 的 applied/rejected/skipped/last-processed 调试计数
- watchdog timeout 计数/age diagnostics
- UART driver 的 debugger-only TX/error counters 与 getters
- 未使用的 `freertos_assert.c`

仍然保留，因为 STATUS 在线消息真实使用：

- RX valid frame count
- RX CRC/length/version/unknown-type error count
- RX sequence gap count
- RX ring overflow count
- control mailbox overwrite count
- last RX/control wire sequence
- link/remote-stop/control-valid 状态
- RAM gimbal pan/tilt stub

`ProtocolEngine` 不再每次 feed 复制整套 parser stats。只有 parser resync 时把 STATUS 需要的 5 个累计值汇总，正常 STATUS 时用 `累计值 + 当前 parser.stats` 得到总数。

## 内存相关改动

### 1. FreeRTOS 改为全静态 RTOS 对象

RX、TX、Gimbal 与 Idle task 的 TCB/stack 均由应用显式提供；两个 TX counting semaphore 使用 `xSemaphoreCreateCountingStatic()`。

`FreeRTOSConfig.h`：

- `configSUPPORT_STATIC_ALLOCATION = 1`
- `configSUPPORT_DYNAMIC_ALLOCATION = 0`
- 不再需要 `configTOTAL_HEAP_SIZE = 10 KB`
- `configUSE_TIMERS = 0`
- `configMAX_PRIORITIES` 从 32 降到 6
- trace、stats formatting、queue registry、queue sets、mutex、绝大多数未使用 optional API 关闭

**构建系统必须同步从工程中移除 `heap_4.c`（或其他 `heap_x.c`）**。FreeRTOS 在 dynamic allocation=0 时继续编译 heap_x.c 会报错。

### 2. TX slot 缩小

当前 MCU 实际发送的最大 payload 是 STATUS=52 bytes，因此每个 TX slot 从 128-byte payload 缩到 52 bytes。6 个 slot 在 32-bit ABI 下大约从 840 B 降到约 384 B。

TX task 的 encoded 临时缓冲也从 portable 最大 294 B 缩到当前 response 最坏 escaping 所需的 142 B。

### 3. CONTROL mailbox 不再创建 FreeRTOS queue

该 mailbox 的并发模型固定为 RX task 写、Gimbal task 读；现在用一个静态结构 + 短 critical section，加入 `generation` 字段。这样：

- 不需要 queue control block / queue storage；
- Gimbal 不再依赖 `control_received_count` 这种 metrics 来判断新 generation；
- `sender_boot_id`、`received_tick_ms` 两个没有消费者的字段删除。

### 4. Decoded message 不再复制 128-byte raw payload

可靠 STOP/CLEAR 的 payload CRC 在 decode 时直接对 `frame->payload` 计算并只保存 `uint16_t payload_crc`。这保持“CRC 基于原始 wire payload、避免 C struct padding”的原始设计，同时显著缩小 RX task 调用栈上的 `ProtocolMessage`。

### 5. 预计 SRAM 变化

仅从源码可确定：

- 原 10 KB FreeRTOS heap 整块取消，改为约 4.35 KB 的显式 task stack + 少量 TCB/semaphore storage；
- Timer Service task 的 256-word（约 1 KB）stack 不再存在；
- TX slot 约节省 456 B；
- diagnostics/历史 metrics/旧 parser aggregate/UART debug counters 等再节省数百字节；
- `configMAX_PRIORITIES` 32→6 还会缩小 kernel ready-list 数据。

净 SRAM 节省预计为数 KB。**准确值应以目标工具链的 `.map` 文件为准**，因为 `StaticTask_t`/`StaticSemaphore_t` 大小取决于 FreeRTOS 配置与端口。

## 行为保持与有意收紧

保持：

- HELLO→HELLO_ACK
- HEARTBEAT→STATUS，且只响应 token
- CONTROL_UPDATE→latest mailbox，不回复
- REMOTE_STOP/CLEAR→可靠 ACK/NACK + duplicate replay
- PING→PONG
- peer boot-id / wire sequence 检查
- remote-stop 在 link loss/re-HELLO 时不会自动清除
- link watchdog 与 control freshness watchdog
- `ProtocolTxTask` 是唯一 UART TX owner
- MCU 不产生 unsolicited transmission

收紧：

- `ProtocolTxTask_EnqueueResponse()` 现在只接受 MCU 合法 response type：ACK、NACK、HELLO_ACK、PONG、STATUS。以前“任意已知 message type”理论上都可进入 TX。
- Linux 发来 MCU-response-only type 时，在 `ProtocolMessage_Decode()` 就直接丢弃，不再为 debugger-only `unexpected_input_message_count` 保存计数；线上行为仍为“不响应”。
- `control_mailbox_overwrite_count` 现在按“一个实际 CONTROL_UPDATE 替换前一个实际 CONTROL_UPDATE”计数；安全 invalidation 不再人为制造一个 queue item。

## 集成步骤

1. 在 Keil/Make/CMake 工程中移除旧 `App/visionarm_uart/src` 文件并添加新 `src` 文件。
2. 用新的 `Core/Inc/FreeRTOSConfig.h` 替换旧配置。
3. 添加 `Core/Src/freertos_static.c`。
4. 从 FreeRTOS build 中移除 `heap_4.c`/其他 `heap_x.c`；软件 timer 未使用时也可不编译 `timers.c`。
5. 保持 include path：`App/visionarm_uart/include`、`portable/mcu_uart/include`。
6. `portable/mcu_uart` 不需要改 Linux 端或 wire protocol。
7. 重新编译后检查 linker `.map` 的 RAM/Flash，并做 HELLO、STATUS、CONTROL freshness、STOP duplicate、CLEAR、PING、link-loss 回归。

## 建议回归用例

至少覆盖：

1. HELLO accepted/rejected；重新 HELLO 后 transaction cache 清空，remote-stop 不清空。
2. HEARTBEAT 只产生一个 STATUS；静默时 MCU 不主动发帧。
3. CONTROL burst：mailbox 始终读取最新 generation；duplicate/old/invalid control 立即使 control invalid。
4. 200 ms control freshness timeout 后 gimbal stub 在下一 20 ms 周期归零，但 link 仍 READY。
5. 1000 ms link timeout 后 link LOST、mailbox invalid，需要重新 HELLO。
6. STOP transaction ACK 丢失后，以新 wire sequence 重传同 transaction ID/payload：只 replay ACK，不重复副作用。
7. 相同 transaction ID、不同 payload：NACK transaction conflict。
8. TX 同时积压 STATUS/PONG/ACK：下一个未开始发送的 frame 顺序为 ACK > PONG > STATUS；正在发送的 frame 不被抢断。
9. RX ring overflow/UART receive error 后 parser resync，STATUS 累计计数保持单调。
10. `configCHECK_FOR_STACK_OVERFLOW=2` 的 hook 能让 PD7 直接进入 RX/non-driving 并 fail-stop。
