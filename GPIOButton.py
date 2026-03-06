import gpiod
import time
import threading

VALID_EVENT_TYPES = ('pressed', 'released', 'both')


class GPIOButton:
    def __init__(self, chip_name, line_offset, name=None, active_low=True):
        """初始化一个GPIO按键

        Args:
            chip_name: GPIO芯片名称，如'gpiochip3'
            line_offset: 引脚偏移量
            name: 按键名称，默认为None
            active_low: True表示低电平为按下（外接上拉），False表示高电平为按下
        """
        self.chip_name = chip_name
        self.line_offset = line_offset
        self.name = name or f"Button_{chip_name}_{line_offset}"
        self.active_low = active_low
        self.chip = None
        self.line = None
        self.is_pressed = False
        self.callbacks = []

    def setup(self):
        """设置GPIO按键"""
        try:
            self.chip = gpiod.Chip(self.chip_name, gpiod.Chip.OPEN_BY_NAME)
            self.line = self.chip.get_line(self.line_offset)
            self.line.request(consumer=self.name, type=gpiod.LINE_REQ_DIR_IN)
            print(f"{self.name} 初始化成功: {self.chip_name} line {self.line_offset}")
        except OSError as e:
            print(f"{self.name} 初始化失败: {e}")
            raise

    def cleanup(self):
        """清理GPIO资源"""
        if self.line:
            self.line.release()
        if self.chip:
            self.chip.close()

    def get_value(self):
        """获取按键当前值"""
        if self.line:
            return self.line.get_value()
        return None

    def add_callback(self, callback, event_type='pressed'):
        """添加按键事件回调函数

        Args:
            callback: 回调函数，接收按键对象作为参数
            event_type: 'pressed'、'released' 或 'both'
        """
        if event_type not in VALID_EVENT_TYPES:
            raise ValueError(
                f"event_type 必须是 {VALID_EVENT_TYPES} 之一，收到: '{event_type}'"
            )
        self.callbacks.append((callback, event_type))

    def handle_event(self, is_pressed):
        """处理按键事件"""
        event_type = 'pressed' if is_pressed else 'released'
        for callback, cb_event_type in self.callbacks:
            if cb_event_type == 'both' or cb_event_type == event_type:
                callback(self)


class GPIOLED:
    def __init__(self, chip_name, line_offset, name=None):
        """初始化一个GPIO LED

        Args:
            chip_name: GPIO芯片名称，如'gpiochip3'
            line_offset: 引脚偏移量
            name: LED名称，默认为None
        """
        self.chip_name = chip_name
        self.line_offset = line_offset
        self.name = name or f"LED_{chip_name}_{line_offset}"
        self.chip = None
        self.line = None
        self.is_on = False

    def setup(self):
        """设置GPIO LED"""
        try:
            self.chip = gpiod.Chip(self.chip_name, gpiod.Chip.OPEN_BY_NAME)
            self.line = self.chip.get_line(self.line_offset)
            self.line.request(consumer=self.name, type=gpiod.LINE_REQ_DIR_OUT, default_val=0)
            print(f"{self.name} 初始化成功: {self.chip_name} line {self.line_offset}")
        except OSError as e:
            print(f"LED初始化失败 (OSError): {e}")
            raise

    def cleanup(self):
        """清理GPIO资源"""
        if self.line and self.line.is_requested():
            self.line.set_value(0)
            self.line.release()
        if self.chip:
            self.chip.close()

    def on(self):
        """点亮LED"""
        if self.line:
            self.line.set_value(1)
            self.is_on = True

    def off(self):
        """关闭LED"""
        if self.line:
            self.line.set_value(0)
            self.is_on = False

    def toggle(self):
        """切换LED状态"""
        if self.is_on:
            self.off()
        else:
            self.on()

    def blink(self, duration=1.0, times=1, blocking=True):
        """LED闪烁

        Args:
            duration: 每次亮/灭的持续时间（秒）
            times: 闪烁次数
            blocking: True为阻塞模式，False为后台线程执行
        """
        def _do_blink():
            for _ in range(times):
                self.on()
                time.sleep(duration)
                self.off()
                time.sleep(duration)

        if blocking:
            _do_blink()
        else:
            threading.Thread(target=_do_blink, daemon=True).start()


class ButtonManager:
    def __init__(self, debounce_interval=0.2):
        """初始化按键管理器

        Args:
            debounce_interval: 消抖时间窗口（秒），同一按键在此间隔内的重复触发会被忽略
        """
        self.buttons = []
        self.running = False
        self.monitor_thread = None
        self.debounce_interval = debounce_interval

    def add_button(self, button):
        """添加一个按键到管理器

        Args:
            button: GPIOButton对象
        """
        self.buttons.append(button)
        button.setup()

    def start_monitoring(self, poll_interval=0.05):
        """开始监控所有按键

        Args:
            poll_interval: 轮询间隔，单位为秒
        """
        if self.running:
            return

        self.running = True
        self.monitor_thread = threading.Thread(
            target=self._monitor_loop, args=(poll_interval,)
        )
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def stop_monitoring(self):
        """停止监控按键"""
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)

    def cleanup(self):
        """清理所有资源"""
        self.stop_monitoring()
        for button in self.buttons:
            button.cleanup()

    def _monitor_loop(self, poll_interval):
        """按键监控循环（含消抖、极性适配、异常保护）"""
        prev_states = [button.get_value() for button in self.buttons]
        last_trigger = [0.0] * len(self.buttons)

        while self.running:
            try:
                current_states = [button.get_value() for button in self.buttons]
                now = time.time()

                for i, button in enumerate(self.buttons):
                    prev = prev_states[i]
                    curr = current_states[i]

                    if prev is None or curr is None:
                        continue

                    if prev == curr:
                        continue

                    if now - last_trigger[i] < self.debounce_interval:
                        continue

                    if button.active_low:
                        is_pressed = (prev == 1 and curr == 0)
                    else:
                        is_pressed = (prev == 0 and curr == 1)

                    last_trigger[i] = now
                    button.is_pressed = is_pressed
                    button.handle_event(is_pressed)

                prev_states = current_states.copy()
            except Exception as e:
                print(f"按键监控异常: {e}")

            time.sleep(poll_interval)


# 示例用法
def main():
    button_manager = ButtonManager()

    buttons = [
        GPIOButton('gpiochip4', 19, "按键1"),
        GPIOButton('gpiochip4', 21, "按键2"),
        GPIOButton('gpiochip4', 18, "按键3"),
        GPIOButton('gpiochip3', 5, "按键4"),
        GPIOButton('gpiochip3', 6, "按键5"),
        GPIOButton('gpiochip3', 7, "按键6"),
    ]

    for button in buttons:
        button_manager.add_button(button)

    def on_button_pressed(button):
        print(f"{button.name} 被按下")

    def on_button_released(button):
        print(f"{button.name} 被释放")

    for button in buttons:
        button.add_callback(on_button_pressed, 'pressed')
        button.add_callback(on_button_released, 'released')

    button_manager.start_monitoring()

    try:
        print("按键监测程序运行中，按Ctrl+C退出")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("程序已退出")
    finally:
        button_manager.cleanup()


if __name__ == "__main__":
    main()
