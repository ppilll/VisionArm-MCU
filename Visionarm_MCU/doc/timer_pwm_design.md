# timer_pwm_design.md

## 时钟树
- HSE：根据当前固件配置为8 MHz
- PLL：x9倍频
- SYSCLK：72 MHz
- HCLK：72 MHz
- APB1预分频器：/2
- PCLK1：36 MHz
- TIM3输入时钟：72 MHz

## 定时器
- 定时器：TIM3
- 宽度：16位
- 计数模式：上升沿
- PSC（预分频系数）：71
- ARR（自动重载寄存器）：19999
- 计数时钟：
72 MHz / (71 + 1) = 1 MHz
- 分辨率：
1个计数 = 1微秒
- PWM周期：
(19999 + 1) × 1 μs = 20,000 μs
- PWM频率：
1 / 20 ms = 50 Hz

## Channels
- CH1 / PA6：水平偏移（Pan）
- CH2 / PA7：倾斜（Tilt）
- PWM模式：PWM1
- 极性：高电平有效
- CCR预加载：启用
- ARR预加载：启用

## 脉冲映射
- CCR=500 → 高脉冲持续500 μs
- CCR=1500 → 高脉冲持续1500 μs
- CCR=2500 → 高脉冲持续2500 μs

## Startup
1. GPIO 保持低电平 / PWM 通道已禁用。
2. 在未启用输出的情况下初始化 TIM3。
3. 加载已知的CCR。
4. 仅在执行器模块和安全政策允许的情况下才启用PWM。

## Safe Stop
1. 禁用CH1/CH2输出。
2. 确保PA6/PA7的电导率低。
3. 不要保留或重新启用旧的邮箱生成。


## Evidence
- Requested frequency: 50 Hz
- Measured frequency: TBD
- Measured period: TBD
- CH1 500 us: TBD
- CH1 1500 us: TBD
- CH1 2500 us: TBD
- CH2 equivalent: TBD
- Logic analyzer screenshot: TBD