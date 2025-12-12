# 激光视觉追踪系统

基于鲁班猫开发板的激光自动瞄准系统，使用计算机视觉识别目标，通过PID控制步进电机实现激光点的精确锁定。

## 项目概述

本项目是一个嵌入式视觉伺服系统，核心功能是通过摄像头实时检测目标（圆形/矩形），计算目标与画面中心的偏差，利用PID算法控制双轴步进电机调整激光指向，最终实现激光点对目标的自动追踪和锁定。

## 系统架构

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   摄像头    │────▶│  图像处理    │────▶│  目标检测   │
│  (USB)      │     │  (OpenCV)    │     │ (圆形/矩形) │
└─────────────┘     └──────────────┘     └──────┬──────┘
                                                │
                                                ▼
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   激光器    │◀────│  PID控制     │◀────│  误差计算   │
│  (GPIO)     │     │  (X/Y轴)     │     │             │
└─────────────┘     └──────┬───────┘     └─────────────┘
                           │
                           ▼
                    ┌──────────────┐
                    │  步进电机    │
                    │  (UART)      │
                    └──────────────┘
```

## 硬件配置

| 组件 | 型号/规格 | 接口 |
|------|----------|------|
| 开发板 | 鲁班猫 (RK3588) | - |
| 摄像头 | USB摄像头 | /dev/video9 |
| 步进电机 | 张大头42步进电机 x2 | UART (/dev/ttyUSB0) |
| 激光器 | 激光模块 | GPIO (gpiochip3, line17) |
| 按键 | 物理按键 x3 | GPIO (gpiochip4) |

### GPIO引脚分配

| 功能 | GPIO芯片 | 引脚 |
|------|---------|------|
| 按键1 (矩形锁定) | gpiochip4 | 19 |
| 按键2 (圆形锁定-左转) | gpiochip4 | 21 |
| 按键3 (圆形锁定-右转) | gpiochip4 | 18 |
| 激光器 | gpiochip3 | 17 |

### 电机地址

| 轴 | 地址 | 功能 |
|----|------|------|
| X轴 | 0x01 | 水平方向控制 |
| Y轴 | 0x02 | 垂直方向控制 |

## 文件结构

```
.
├── main.py              # 主程序入口
├── motor_cmd.py         # 步进电机控制模块
├── pid.py               # PID控制器
├── GPIOButton.py        # GPIO按键和LED控制（多线程轮询）
├── uart_thread.py       # 串口通信模块（多线程异步收发）
├── uart.ini             # 串口配置文件
├── README.md            # 项目文档
├── .gitignore           # Git忽略文件配置
└── tests/               # 测试脚本
    ├── test_camera.py   # 摄像头测试
    └── test_gpio_led.py # GPIO LED测试
```

## 模块说明

### 1. main.py - 主程序

**核心功能：**
- 图像采集与处理
- 目标检测（圆形/矩形）
- 任务调度与状态管理
- 按键事件处理

**主要类/函数：**

| 名称 | 类型 | 说明 |
|------|------|------|
| `Config` | 类 | 系统配置参数 |
| `circle_detection_pipeline()` | 函数 | 圆形检测管道 |
| `rectangle_detection_pipeline()` | 函数 | 矩形检测管道 |
| `locked()` | 函数 | PID锁定目标点 |
| `task_rectangle_lock()` | 函数 | 任务1：矩形锁定 |
| `task_circle_lock()` | 函数 | 任务2/3：圆形锁定 |

### 2. motor_cmd.py - 电机控制

**支持的命令模式：**
- `speed` - 速度模式
- `pos` - 位置模式（脉冲控制）
- `enable` - 使能控制

**关键方法：**

```python
motor.send_command(uart_send, 'pos', pulsesx=100, addr=0x01, speed=200, dir=1)
motor.trigger_homing(uart_send, uart_receive, addr=0x01, homing_mode=0x00)
```

### 3. pid.py - PID控制器

**特性：**
- 比例(P)、积分(I)、微分(D) 完整实现
- 积分限幅防饱和
- 微分限幅防抖动

**使用示例：**

```python
pid = PID(kp=1.0, ki=0.3, kd=0.2)
output = pid.pid_control(error, last_error, dt)
```

### 4. GPIOButton.py - GPIO控制

**组件：**
- `GPIOButton` - 按键输入管理
- `GPIOLED` - LED/激光器输出控制
- `ButtonManager` - 多按键轮询管理

### 5. uart_thread.py - 串口通信

**特性：**
- 异步收发（独立线程）
- 队列缓冲机制
- 配置文件支持

## 比赛任务

### 任务1：矩形锁定

1. 检测画面中最大的矩形目标
2. 计算矩形中心与画面中心的偏差
3. PID控制电机调整激光指向
4. 连续2帧误差≤5像素时开启激光
5. 激光亮1秒后自动关闭

### 任务2：圆形锁定（左转搜索）

1. 初始状态电机左转搜索目标
2. 检测到圆形后停止搜索
3. 锁定最小圆形的圆心
4. 误差≤3像素时开启激光

### 任务3：圆形锁定（右转搜索）

与任务2相同，但初始搜索方向为右转。

## 快速开始

### 1. 环境依赖

```bash
pip install opencv-python numpy pyserial gpiod
```

### 2. 配置串口

编辑 `uart.ini`：

```ini
[com1]
port = /dev/ttyUSB0
baudrate = 115200
bytesize = 8
parity = N
stopbits = 1
timeout = 1
```

### 3. 运行程序

```bash
python main.py
```

### 4. 操作说明

| 操作 | 功能 |
|------|------|
| 按键1 | 启动/停止 矩形锁定 |
| 按键2 | 启动/停止 圆形锁定（左转搜索） |
| 按键3 | 启动/停止 圆形锁定（右转搜索） |
| 键盘 q | 退出程序 |

## 参数调优

### PID参数

位于 `main.py` 的 `Config` 类和任务状态类中：

```python
# 矩形锁定 PID
pid_x = PID(kp=1, ki=0.3, kd=0.2)
pid_y = PID(kp=1, ki=0.3, kd=0.2)

# 圆形锁定 PID
Task2State.pid_x = PID(kp=1.4, ki=0.5, kd=1.2)
Task2State.pid_y = PID(kp=1.2, ki=0.5, kd=1.2)
```

### 图像检测参数

```python
class Config:
    # 圆形检测
    CIRCLE_MIN_AREA = 50
    CIRCLE_CIRCULARITY = 0.5
    CIRCLE_MIN_RADIUS = 25
    CIRCLE_MAX_RADIUS = 100

    # 矩形检测
    RECT_MIN_AREA = 100
    RECT_MIN_ASPECT = 0.3
    RECT_MAX_ASPECT = 3.0
```

### 画面中心校准

根据实际激光器安装位置调整：

```python
CENTER_PX = 92  # X轴中心（像素）
CENTER_PY = 68  # Y轴中心（像素）
```

## 调试技巧

1. **摄像头无法打开**：检查设备号 `CAMERA_ID`，使用 `ls /dev/video*` 确认
2. **电机不响应**：检查串口连接和配置，确认波特率匹配
3. **激光不亮**：检查GPIO引脚配置和电平逻辑
4. **检测不稳定**：调整Canny边缘检测阈值和圆度/面积阈值

## 技术栈

- **语言**: Python 3
- **视觉处理**: OpenCV
- **GPIO控制**: libgpiod
- **串口通信**: pyserial
- **控制算法**: PID

## 许可证

本项目仅供学习和比赛使用。
