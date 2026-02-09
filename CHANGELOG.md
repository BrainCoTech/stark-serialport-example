# BrainCo RevoHand SDK Examples 更新日志

> 📋 此更新日志面向外部客户和集成开发者。如需了解 SDK 内部实现细节，请联系 BrainCo 技术支持团队。

## v1.1.4 (2026/02/09)

### 🚀 新增功能

#### 设备上下文查询
- 新增 `stark_get_protocol_type`、`stark_get_port_name`、`stark_get_baudrate`、`stark_get_can_arb_baudrate`、`stark_get_can_data_baudrate` 查询接口
- 新增 `init_device_handler_can()` / `init_device_handler_can_with_hw_type()` CAN 设备初始化
- 新增 `StarkProtocolType::Auto = 0` 枚举值，用于自动检测所有协议

### 🔧 示例代码改进
- C++: `CollectorContext` 重命名为 `DeviceContext`，移除 `protocol`、`port_name`、`baudrate` 字段，改用 C API 查询接口
- C++: CAN 初始化函数统一改用 `init_device_handler_can()` 存储波特率
- C++: `stark_auto_detect` 返回类型改用 `StarkProtocolType` 枚举，增强类型安全
- C++: 修复 `hand_demo.cpp` 编译错误：使用 `STARK_PROTOCOL_TYPE_AUTO` 替代字面量 `0`
- Python: 全面添加类型注解，提升代码质量

---

## v1.1.2 (2026/02/06)

### 🚀 新增功能

#### 协议支持扩展
- **ZQWL CAN 内置** - SDK 内置驱动，无需额外 DLL，支持 Linux/macOS/Windows
- **SocketCAN 内置** (Linux) - 无需外部代码
- **Protobuf 协议** - Revo1 串口协议，波特率 115200，Slave ID 10-254

#### 统一自动检测
- `auto_detect()` 支持 Modbus、ZQWL CAN/CANFD 多协议自动检测
- `init_from_detected()` 从检测结果直接初始化设备

#### Revo1 触觉 API
- 支持 Modbus 和 CAN 2.0 协议的触觉传感器
- 支持 Revo1Touch 和 Revo1AdvancedTouch 设备

#### 新增硬件类型
- `Revo1Advanced` - 一代进阶版（序列号 BCMEL/BCMER）
- `Revo1AdvancedTouch` - 一代进阶触觉版（序列号 BCMTL2/BCMTR2）

#### 示例与工具
- `c/demo/` - 跨平台 C++ 示例（hand_demo、hand_monitor、hand_dfu、auto_detect）
- `python/demo/` - 统一 Python 示例
- `python/gui/` - 图形化调试工具（仅供调试）

### ⚠️ 迁移指南

#### API 重命名

| 旧 API | 新 API |
|--------|--------|
| `is_revo1()` | `uses_revo1_motor_api()` |
| `is_revo1_touch()` | `uses_revo1_touch_api()` |
| `PyDeviceContext` | `DeviceContext` |

> 注：判断是否使用 Revo2 API 可用 `!uses_revo1_motor_api()` / `!uses_revo1_touch_api()`

#### 初始化 API 变更

**旧 API：**
```python
sdk.init_config(protocol, log_level)
ctx = PyDeviceContext.init_canfd(master_id)
```

**新 API：**
```python
sdk.init_logging(log_level)
ctx = sdk.init_device_handler(StarkProtocolType.CanFd, master_id)
```

#### C 结构体命名变更

所有 C 导出结构体添加 `C` 前缀：`MotorStatusData` → `CMotorStatusData`、`DeviceInfo` → `CDeviceInfo` 等。

#### 弃用通知
- `linux/` 和 `windows/` 文件夹已弃用，请迁移至 `c/` 文件夹

---

## v1.0.6 (2026/01/26)

### 🚀 新增功能

#### 新增硬件支持
- **Revo1Advanced** - 一代进阶版（序列号 BCMEL/BCMER），使用 Revo2 API
- **Revo1AdvancedTouch** - 一代进阶触觉版（序列号 BCMTL2/BCMTR2）

#### 序列号自动识别

| 序列号前缀 | 硬件类型 |
|-----------|---------|
| `BCMRL/BCMRR` | Revo1Basic |
| `BCMEL/BCMER` | Revo1Advanced |
| `BCMTL1/BCMTR1` | Revo1Touch |
| `BCMTL2/BCMTR2` | Revo1AdvancedTouch |
| `BCXTL/BCXTR` | Revo2Touch |
| `BCX*` | Revo2Basic |

#### 架构优化
- 马达/触觉状态采用底层多线程采集，上层被动读取
- 物理量模式控制逻辑修复
- Revo2 RS-485 DFU 波特率自动检测
- SocketCAN 后端支持

#### 自动检测增强
- 多端口自动遍历
- Revo1/Revo2/Protobuf 协议自动识别
- Quick 模式快速检测

### 📚 新增示例
- `revo2_touch_collector.py` - 触觉数据采集
- `revo2_timing_test_gui.py` - 时序测试 GUI

### ⚠️ 注意
- Revo1Advanced（BCMEL/BCMER）需使用 `revo2` 目录下的示例代码

---

## v1.0.4 (2026/01/23)

- 支持 Data Collector 数据采集器
- 支持轨迹控制功能

---

## v1.0.1 (2025/12/23)

- 支持 EtherCAT 多从站通信

---

## v1.0.0 (2025/12/08)

### 🎉 正式版本
- 支持 Revo1 和 Revo2 设备
- 支持 RS-485、CAN、CANFD、EtherCAT 协议
- 提供 Python 和 C++ 示例代码

---

## v0.9.9 (2025/11/19)

- 支持 Revo1 进阶版设备
- 统一控制参数范围：位置 0~1000，速度/电流/PWM -1000~+1000

> ⚠️ Revo1 进阶版设备需要 SDK v0.9.9+

---

## v0.9.8 (2025/11/04)

### � 新增功能

#### CAN/CANFD 支持
- Revo2 CAN2.0/CANFD 协议栈
- ZLG CAN/CANFD 驱动封装
- CANFD 分块读写（支持超过 29 寄存器）

#### EtherCAT 支持
- 触觉传感器数据采集（PDO/SDO）
- 触觉压力传感器支持

#### 通用功能
- ProtectedCurrent 读写接口
- `run_action_sequence` 动作序列执行
- 序列号设备类型判断

#### 性能优化
- C/C++ 端 `set` 类指令异步执行
- 高频接口禁用重试，避免指令堆积

### � 问题修复
- TurboConfig 字节序问题
- Modbus C API 异步调用
- DFU 升级流程优化

---

如有问题，请访问 [官方文档](https://www.brainco-hz.com/docs/revolimb-hand/index.html) 或联系 BrainCo 技术支持。
