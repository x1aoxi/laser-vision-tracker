#!/usr/bin/env python3
import time
from GPIOButton import GPIOLED

def main():
    """简单LED测试：开3秒关3秒循环"""
    print("GPIO LED简单测试程序")
    print("硬件配置: GPIO3_C1 (gpiochip3, line 17)")
    print("按Ctrl+C退出程序")
    
    led = GPIOLED('gpiochip3', 17, "测试LED")
    
    try:
        led.setup()
        print("LED初始化成功，开始循环测试...")
        
        while True:
            print("LED点亮...")
            led.on()
            time.sleep(3)
            
            print("LED关闭...")
            led.off()
            time.sleep(3)
            
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"测试失败: {e}")
    finally:
        led.cleanup()
        print("LED资源已清理，程序退出")

if __name__ == "__main__":
    main()
