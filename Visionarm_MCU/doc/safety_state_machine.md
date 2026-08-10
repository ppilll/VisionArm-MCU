# 安全状态机.md

优先级：
> 故障(FAULT)
> 远程停止(REMOTE_STOP)
> 链接丢失(LINK_LOST)
> 控制失效 / 信箱无效(CONTROL_STALE / MAILBOX_INVALID)
> 限位被阻塞(LIMIT_BLOCKED)
> 准备就绪(READY)
> 活动中(ACTIVE)

## 启动安全(BOOT_SAFE)
输入：
- 重置/电源开启
输出：
- PWM 禁用
退出：
- 执行器驱动已初始化
锁存：
- 无

## 已禁用(DISABLED)
输入：
- 稳定架已初始化但未允许控制
输出：
- PWM 禁用
退出：
- 有效安全条件 + 新的控制生成

## 准备就绪(READY)
输入：
- 链接就绪
- 无停止
- 控制子系统有效
- 执行器健康
- 等待新的禁用后生成
输出：
- PWM 禁用

## 活动中(ACTIVE)
输入：
- 安全门已启用
- 存在可接受的新生成
输出：
- 控制PWM
退出：
- 任何更高优先级的安全事件

## 远程停止(REMOTE_STOP)
输入：
- 远程停止锁存
输出：
- PWM 禁用
退出：
- 锁存被明确清除，且新的 CONTROL_UPDATE 生成到达
锁存：
- 是，由现有的 V5 协议状态拥有

## 链接丢失(LINK_LOST)
输入：
- 链接看门狗超时
输出：
- PWM 禁用
退出：
- 新的HELLO/会话 + 新的控制

## 控制失效(CONTROL_STALE)
输入：
- control_valid 为 false / 信箱因新鲜度看门狗而无效
输出：
- PWM 禁用
退出：
- 新的 CONTROL_UPDATE- 保持夹紧安全位置  
- 向外方向被阻断  
- 反向方向允许  

## 故障(FAULT)  
输入：  
- 执行器/定时器初始化或本地致命故障  
输出：  
- PWM被禁用  
退出：  
- 正常复位/恢复程序