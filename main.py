"""
激光视觉追踪系统 - 主程序
基于鲁班猫开发板的激光自动瞄准系统
"""

import cv2
import numpy as np
import time
import math
import threading

import uart_thread as uart
from motor_cmd import MotorController
from pid import PID
from GPIOButton import GPIOButton, ButtonManager, GPIOLED


# ==============================================================================
# 配置参数
# ==============================================================================
class Config:
    """系统配置参数"""
    # 摄像头参数
    CAMERA_ID = 9
    CAMERA_FPS = 20
    CAMERA_WIDTH = 160
    CAMERA_HEIGHT = 120
    CAMERA_EXPOSURE = 50

    # 画面中心（激光校准点）
    CENTER_PX = 92
    CENTER_PY = 68

    # 电机地址
    MOTOR_X_ADDR = 0x01
    MOTOR_Y_ADDR = 0x02

    # GPIO引脚配置
    GPIO_BUTTON1 = ('gpiochip4', 19)  # 按键1: 矩形锁定
    GPIO_BUTTON2 = ('gpiochip4', 21)  # 按键2: 圆形锁定(左转)
    GPIO_BUTTON3 = ('gpiochip4', 18)  # 按键3: 圆形锁定(右转)
    GPIO_LASER = ('gpiochip3', 17)    # 激光器

    # 圆形检测参数
    CIRCLE_MIN_AREA = 50
    CIRCLE_CIRCULARITY = 0.5
    CIRCLE_MIN_RADIUS = 25
    CIRCLE_MAX_RADIUS = 100

    # 矩形检测参数
    RECT_MIN_AREA = 100
    RECT_MIN_ASPECT = 0.3
    RECT_MAX_ASPECT = 3.0


# ==============================================================================
# 摄像头采集线程（始终保留最新帧，解决 V4L2 缓冲区延迟）
# ==============================================================================
class CameraCapture:
    """独立线程采集摄像头画面，主线程始终获取最新帧"""

    def __init__(self, camera_id, fps=20, width=160, height=120, exposure=50):
        self.cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        self._frame = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def is_opened(self):
        return self.cap.isOpened()

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        while self._running:
            ret, frame = self.cap.read()
            if ret:
                with self._lock:
                    self._frame = frame

    def read(self):
        with self._lock:
            if self._frame is None:
                return False, None
            return True, self._frame.copy()

    def release(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self.cap.release()


# 预创建 CLAHE 对象，避免每帧重建
_clahe = cv2.createCLAHE(clipLimit=5, tileGridSize=(4, 4))


# ==============================================================================
# 图像处理模块
# ==============================================================================
def circle_detection_pipeline(frame, config=Config):
    """
    圆形检测管道

    Args:
        frame: 输入图像帧
        config: 配置参数

    Returns:
        dict: 包含circles列表、可视化帧和边缘图
    """
    # CLAHE 增强：BGR→YCrCb 提取 Y 通道，增强后直接作为灰度图使用
    # 省去了 merge→YCrCb→BGR→Gray 三次冗余转换
    ycbcr = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
    gray = _clahe.apply(ycbcr[:, :, 0])

    gray = cv2.GaussianBlur(gray, (5, 5), 1)
    edges = cv2.Canny(gray, 50, 60)

    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    circles = []
    min_area = config.CIRCLE_MIN_AREA
    min_circularity = config.CIRCLE_CIRCULARITY
    min_r = config.CIRCLE_MIN_RADIUS
    max_r = config.CIRCLE_MAX_RADIUS

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue

        circularity = (4 * np.pi * area) / (perimeter * perimeter)
        if circularity < min_circularity:
            continue

        (x, y), r = cv2.minEnclosingCircle(cnt)
        if min_r <= r <= max_r:
            circles.append((int(x), int(y), int(r)))

    return {
        "circles": circles,
        "visualization": frame,
        "edges": edges
    }


def rectangle_detection_pipeline(frame, config=Config):
    """
    矩形检测管道

    Args:
        frame: 输入图像帧
        config: 配置参数

    Returns:
        dict: 包含max_rect(最大矩形)、可视化帧和边缘图
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 60, 180)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_rect = None
    max_area = config.RECT_MIN_AREA

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < max_area:
            continue

        perimeter = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)

        if len(approx) == 4 and cv2.isContourConvex(approx):
            rect = cv2.minAreaRect(approx)
            (x_center, y_center), (w, h), angle = rect
            if w < h:
                w, h = h, w
            aspect_ratio = w / h

            if config.RECT_MIN_ASPECT <= aspect_ratio <= config.RECT_MAX_ASPECT:
                box = np.int0(cv2.boxPoints(rect))
                max_rect = {
                    "area": area,
                    "center": (int(x_center), int(y_center)),
                    "box": box,
                    "angle": angle
                }
                max_area = area

    return {
        "max_rect": max_rect,
        "visualization": frame,
        "edges": edges
    }


# ==============================================================================
# 几何工具模块
# ==============================================================================
def generate_8_points(center_x, center_y, radius=13):
    """
    生成围绕中心的8个均匀分布点

    Args:
        center_x, center_y: 中心坐标
        radius: 分布半径

    Returns:
        list: 8个点的坐标列表
    """
    points = []
    for i in range(8):
        angle = i * math.pi / 4
        x = int(center_x + radius * math.cos(angle))
        y = int(center_y + radius * math.sin(angle))
        points.append((x, y))
    return points


def sort_points_counterclockwise_from_left(points, center):
    """
    从最左点开始逆时针排序

    Args:
        points: 点列表
        center: 中心点

    Returns:
        list: 排序后的点列表
    """
    if not points:
        return []

    leftmost = min(points, key=lambda p: (p[0], p[1]))

    def get_angle(point):
        dx = point[0] - center[0]
        dy = point[1] - center[1]
        angle = math.atan2(dy, dx)
        return angle if angle >= 0 else angle + 2 * math.pi

    start_angle = get_angle(leftmost)

    def sort_key(p):
        diff = get_angle(p) - start_angle
        return diff if diff >= 0 else diff + 2 * math.pi

    return sorted(points, key=sort_key)


# ==============================================================================
# 运动控制模块
# ==============================================================================
def locked(motor, center_x, center_y, center_px, center_py,
           pid_x, pid_y, last_error_x, last_error_y, dt,
           error=10, pulses_max=10):
    """
    PID锁定目标点

    Args:
        motor: 电机控制器
        center_x, center_y: 目标中心坐标
        center_px, center_py: 画面中心坐标
        pid_x, pid_y: X/Y轴PID控制器
        last_error_x, last_error_y: 上次误差
        dt: 时间间隔
        error: 误差容限
        pulses_max: 最大脉冲数

    Returns:
        tuple: (当前X误差, 当前Y误差)
    """
    # X轴控制
    error_x = center_x - center_px
    if abs(error_x) <= error:
        motor.send_command(uart.uart_send, 'pos', pulses=1, addr=0x01, speed=0, pos_mode=0)
        pid_x.integral_sum = 0
    else:
        pid_output_x = pid_x.pid_control(error_x, last_error_x, dt)
        pulses_x = max(1, min(int(abs(pid_output_x)), pulses_max))
        direction = 1 if error_x > 0 else 0
        motor.send_command(uart.uart_send, 'pos', pulses=pulses_x, addr=0x01,
                          speed=1, pos_mode=0, dir=direction)

    # Y轴控制
    error_y = center_y - center_py
    if abs(error_y) <= error:
        motor.send_command(uart.uart_send, 'pos', pulses=1, addr=0x02, speed=0, pos_mode=0)
        pid_y.integral_sum = 0
    else:
        pid_output_y = pid_y.pid_control(error_y, last_error_y, dt)
        pulses_y = max(1, min(int(abs(pid_output_y)), pulses_max))
        direction = 1 if error_y > 0 else 0
        motor.send_command(uart.uart_send, 'pos', pulses=pulses_y, addr=0x02,
                          speed=1, pos_mode=0, dir=direction)

    return error_x, error_y


# ==============================================================================
# 任务函数模块
# ==============================================================================
class Task1State:
    """任务1状态：矩形锁定"""
    stable_count = 0


def task_rectangle_lock(frame, motor, pid_x, pid_y, center_px, center_py,
                        last_errors, dt, laser, laser_state):
    """
    任务1：检测矩形并锁定中心，稳定后开启激光
    """
    last_error_x, last_error_y = last_errors
    rect_result = rectangle_detection_pipeline(frame)
    max_rect = rect_result["max_rect"]
    visualization = rect_result["visualization"]

    if max_rect is not None:
        center_x, center_y = max_rect["center"]
        current_error_x, current_error_y = locked(
            motor, center_x, center_y,
            center_px, center_py,
            pid_x, pid_y,
            last_error_x, last_error_y,
            dt, error=3, pulses_max=10
        )

        # 激光控制：连续2帧误差达标才触发
        if abs(current_error_x) <= 5 and abs(current_error_y) <= 5:
            Task1State.stable_count += 1
            if Task1State.stable_count >= 2:
                if not laser_state["active"]:
                    laser.on()
                    laser_state["active"] = True
                    laser_state["start_time"] = time.time()
                elif time.time() - laser_state["start_time"] >= 1:
                    laser.off()
                    laser_state["active"] = True
                    Task1State.stable_count = 0
        else:
            Task1State.stable_count = 0
            if laser_state["active"]:
                laser.off()
                laser_state["active"] = False

        return (current_error_x, current_error_y), visualization
    else:
        # 未检测到矩形
        Task1State.stable_count = 0
        laser.off()
        laser_state["active"] = False
        motor.send_command(uart.uart_send, 'pos', pulses=1, addr=0x01, speed=0, pos_mode=0)
        motor.send_command(uart.uart_send, 'pos', pulses=1, addr=0x02, speed=0, pos_mode=0)
        pid_x.integral_sum = 0
        pid_y.integral_sum = 0
        return (last_error_x, last_error_y), visualization


class Task2State:
    """任务2状态：圆形锁定"""
    pid_x = None
    pid_y = None
    initialized = False

    @classmethod
    def init(cls):
        if not cls.initialized:
            cls.pid_x = PID(kp=1.4, ki=0.5, kd=1.2)
            cls.pid_y = PID(kp=1.2, ki=0.5, kd=1.2)
            cls.initialized = True


def task_circle_lock(frame, motor, center_px, center_py, last_errors, dt,
                     searching, laser, search_dir):
    """
    任务2/3：检测最小圆形并锁定，支持左转/右转搜索

    Args:
        search_dir: 搜索方向 (0=左转, 1=右转)
    """
    Task2State.init()
    last_error_x, last_error_y = last_errors

    circle_result = circle_detection_pipeline(frame)
    circles = circle_result["circles"]
    visualized_frame = circle_result["visualization"]

    if circles:
        # 筛选最小圆形
        min_circle = min(circles, key=lambda c: c[2])
        min_x, min_y, min_r = min_circle

        # 绘制标识
        cv2.circle(visualized_frame, (min_x, min_y), min_r, (0, 255, 0), 3)

        # PID锁定
        current_error_x, current_error_y = locked(
            motor, min_x, min_y, center_px, center_py,
            Task2State.pid_x, Task2State.pid_y,
            last_error_x, last_error_y,
            dt, error=2, pulses_max=80
        )
        last_errors = (current_error_x, current_error_y)

        # 误差达标时开启激光
        if abs(current_error_x) <= 3 and abs(current_error_y) <= 3:
            laser.on()
        else:
            laser.off()
        searching = False
    else:
        if searching:
            # 搜索模式：按指定方向旋转
            motor.send_command(uart.uart_send, 'pos', pulses=100, addr=0x01,
                              speed=200, acc=210, pos_mode=0, dir=search_dir)
        else:
            # 停止
            motor.send_command(uart.uart_send, 'pos', pulses=1, addr=0x01, speed=0, pos_mode=0)
            motor.send_command(uart.uart_send, 'pos', pulses=1, addr=0x02, speed=0, pos_mode=0)
            Task2State.pid_x.integral_sum = 0
            Task2State.pid_y.integral_sum = 0

    return visualized_frame, last_errors, searching


# ==============================================================================
# 主程序
# ==============================================================================
def main():
    """主程序入口"""
    # 初始化串口
    uart.uart_init()
    motor = MotorController()

    # 初始化GPIO
    button_manager = ButtonManager()
    button1 = GPIOButton(*Config.GPIO_BUTTON1, "按键1-矩形")
    button2 = GPIOButton(*Config.GPIO_BUTTON2, "按键2-圆形左")
    button3 = GPIOButton(*Config.GPIO_BUTTON3, "按键3-圆形右")
    laser = GPIOLED(*Config.GPIO_LASER, "激光")
    laser.setup()

    button_manager.add_button(button1)
    button_manager.add_button(button2)
    button_manager.add_button(button3)

    # 状态变量
    lock_enabled = False      # 任务1激活
    circle_mode = None        # 圆形模式 (None/0=左转/1=右转)
    last_rect_errors = (0, 0)
    last_circle_errors = (0, 0)
    searching_for_circle = False
    laser_state = {"active": False, "start_time": 0}

    # PID控制器
    pid_x = PID(kp=1, ki=0.3, kd=0.2)
    pid_y = PID(kp=1, ki=0.3, kd=0.2)

    # 按键回调函数
    def on_button1_pressed(button):
        nonlocal lock_enabled, circle_mode, searching_for_circle
        lock_enabled = not lock_enabled
        circle_mode = None
        searching_for_circle = False

        if lock_enabled:
            print("=== 任务1：矩形锁定已启动 ===")
            motor.send_command(uart.uart_send, 'enable', enable=0x01, addr=0x01)
            time.sleep(0.01)
            motor.send_command(uart.uart_send, 'enable', enable=0x01, addr=0x02)
        else:
            print("=== 任务1：矩形锁定已停止 ===")
            _stop_motors()

    def on_button2_pressed(button):
        nonlocal circle_mode, lock_enabled, searching_for_circle
        circle_mode = 0 if circle_mode != 0 else None
        lock_enabled = False

        if circle_mode == 0:
            print("=== 任务2：圆形锁定已启动（左转搜索） ===")
            _enable_and_home_motors()
            searching_for_circle = True
        else:
            print("=== 任务2：圆形锁定已停止 ===")
            laser.off()
            _stop_motors()
            searching_for_circle = False

    def on_button3_pressed(button):
        nonlocal circle_mode, lock_enabled, searching_for_circle
        circle_mode = 1 if circle_mode != 1 else None
        lock_enabled = False

        if circle_mode == 1:
            print("=== 任务3：圆形锁定已启动（右转搜索） ===")
            _enable_and_home_motors()
            searching_for_circle = True
        else:
            print("=== 任务3：圆形锁定已停止 ===")
            laser.off()
            _stop_motors()
            searching_for_circle = False

    def _enable_and_home_motors():
        motor.send_command(uart.uart_send, 'enable', enable=0x01, addr=0x01)
        time.sleep(0.01)
        motor.send_command(uart.uart_send, 'enable', enable=0x01, addr=0x02)
        time.sleep(0.01)
        motor.trigger_homing(uart.uart_send, uart.uart_receive, addr=0x02, homing_mode=0x00)
        time.sleep(0.05)

    def _stop_motors():
        motor.send_command(uart.uart_send, 'pos', pulses=1, addr=0x01, speed=0, pos_mode=0)
        motor.send_command(uart.uart_send, 'pos', pulses=1, addr=0x02, speed=0, pos_mode=0)
        pid_x.integral_sum = 0
        pid_y.integral_sum = 0

    # 绑定回调
    button1.add_callback(on_button1_pressed, 'pressed')
    button2.add_callback(on_button2_pressed, 'pressed')
    button3.add_callback(on_button3_pressed, 'pressed')
    button_manager.start_monitoring(poll_interval=0.05)

    # 初始化摄像头（独立采集线程，始终获取最新帧）
    cam = CameraCapture(
        Config.CAMERA_ID,
        fps=Config.CAMERA_FPS,
        width=Config.CAMERA_WIDTH,
        height=Config.CAMERA_HEIGHT,
        exposure=Config.CAMERA_EXPOSURE
    )

    if not cam.is_opened():
        print("错误：无法打开摄像头")
        return

    cam.start()

    print("=" * 50)
    print("激光视觉追踪系统 - 初始化完成")
    print("=" * 50)
    print("操作说明：")
    print("  按键1：启动/停止 矩形锁定")
    print("  按键2：启动/停止 圆形锁定（左转搜索）")
    print("  按键3：启动/停止 圆形锁定（右转搜索）")
    print("  键盘q：退出程序")
    print("=" * 50)

    last_time = time.time()

    # 主循环
    try:
        while True:
            ret, frame = cam.read()
            if not ret:
                continue

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            key = cv2.waitKey(1)
            if key == ord('q'):
                break

            # 执行当前任务
            if lock_enabled:
                last_rect_errors, visualized_frame = task_rectangle_lock(
                    frame, motor, pid_x, pid_y,
                    Config.CENTER_PX, Config.CENTER_PY,
                    last_rect_errors, dt, laser, laser_state
                )
            elif circle_mode is not None:
                visualized_frame, last_circle_errors, searching_for_circle = task_circle_lock(
                    frame, motor,
                    Config.CENTER_PX, Config.CENTER_PY,
                    last_circle_errors, dt,
                    searching_for_circle, laser,
                    search_dir=circle_mode
                )
            else:
                visualized_frame = frame

    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        button_manager.cleanup()
        cam.release()
        laser.off()
        print("程序已退出")


if __name__ == "__main__":
    main()
