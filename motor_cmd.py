import time

class MotorController:
    """张大头42步进电机控制器类（使用外部串口管理）"""
    
    def __init__(self):
        """初始化控制器（不再管理串口）"""
        print("电机控制器初始化完成")

    def set_zero_point(self, uart_send_func, uart_receive_func, addr=0x01, save_flag=0x01, check=0x6B):
        """设置单圈回零的零点位置"""
        cmd_str = f"{addr:02X} 93 88 {save_flag:02X} {check:02X}"
        
        try:
            cmd_bytes = list(bytes.fromhex(cmd_str.replace(" ", "")))
            uart_send_func(cmd_bytes)
            print(f"发送设置零点命令：{cmd_str}")
            
            time.sleep(0.1)
            response = uart_receive_func()
            response_str = ' '.join([f"{b:02X}" for b in response]) if response else ""
            
            status = "未知"
            if response_str:
                resp_parts = response_str.split()
                if len(resp_parts) >= 3 and resp_parts[1] == "93":
                    status_code = resp_parts[2]
                    status = "成功" if status_code == "02" else f"失败（状态码：{status_code}）"
            
            return {
                "success": status == "成功",
                "message": f"零点设置{status}",
                "command": cmd_str,
                "response": response_str
            }
            
        except Exception as e:
            return {"success": False, "message": f"命令错误：{str(e)}", "command": cmd_str, "response": None}

    def trigger_homing(self, uart_send_func, uart_receive_func, addr=0x01, homing_mode=0x00, sync=0x00, check=0x6B):
        """触发回零（使用外部串口函数）"""
        cmd_str = f"{addr:02X} 9A {homing_mode:02X} {sync:02X} {check:02X}"
        
        try:
            cmd_bytes = list(bytes.fromhex(cmd_str.replace(" ", "")))
            uart_send_func(cmd_bytes)
            print(f"发送回零命令：{cmd_str}")
            
            time.sleep(0.1)
            response = uart_receive_func()
            response_str = ' '.join([f"{b:02X}" for b in response]) if response else ""
            
            status = "未知"
            if response_str:
                resp_parts = response_str.split()
                if len(resp_parts) >= 3 and int(resp_parts[0], 16) == addr:
                    status_code = resp_parts[2]
                    if status_code == "02":
                        status = "成功（开始回零）"
                    elif status_code == "E2":
                        status = "条件不满足（检查使能/零点）"
                    else:
                        status = f"响应状态码：{status_code}"
            
            return {
                "success": status.startswith("成功"),
                "message": f"回零触发{status}",
                "command": cmd_str,
                "response": response_str
            }
            
        except Exception as e:
            return {"success": False, "message": f"命令错误：{str(e)}", "command": cmd_str, "response": None}

    def get_command(self, mode, **kwargs):
        """生成运动控制命令（速度/位置/使能）"""
        if mode == 'speed':
            params = {
                'addr': kwargs.get('addr', 0x01),
                'speed': kwargs.get('speed', 1000),
                'dir': kwargs.get('dir', 0x00),
                'acc': kwargs.get('acc', 0x0A),
                'sync': kwargs.get('sync', 0x00),
                'check': kwargs.get('check', 0x6B)
            }
            if not isinstance(params['speed'], int) or params['speed'] <= 0:
                return "错误：速度必须为正整数（单位RPM）"
            try:
                speed_bytes = params['speed'].to_bytes(2, byteorder='big', signed=False)
            except OverflowError:
                return "错误：速度超出范围（最大65535）"
            speed_hex = speed_bytes.hex().upper()
            return (
                f"{params['addr']:02X} F6 {params['dir']:02X} "
                f"{speed_hex[:2]} {speed_hex[2:]} {params['acc']:02X} "
                f"{params['sync']:02X} {params['check']:02X}"
            )
        
        elif mode == 'pos':
            params = {
                'addr': kwargs.get('addr', 0x01),
                'speed': kwargs.get('speed', 1000),
                'dir': kwargs.get('dir', 0x00),
                'acc': kwargs.get('acc', 0x00),
                'pulses': kwargs.get('pulses', 32000),
                'pos_mode': kwargs.get('pos_mode', 0x00),
                'sync': kwargs.get('sync', 0x00),
                'check': kwargs.get('check', 0x6B)
            }
            if not isinstance(params['pulses'], int) or params['pulses'] <= 0:
                return "错误：脉冲数必须为正整数"
            try:
                speed_bytes = params['speed'].to_bytes(2, byteorder='big')
                pulse_bytes = params['pulses'].to_bytes(4, byteorder='big')
            except OverflowError:
                return "错误：参数超出范围"
            speed_hex = speed_bytes.hex().upper()
            pulse_hex = pulse_bytes.hex().upper()
            return (
                f"{params['addr']:02X} FD {params['dir']:02X} "
                f"{speed_hex[:2]} {speed_hex[2:]} {params['acc']:02X} "
                f"{pulse_hex[:2]} {pulse_hex[2:4]} {pulse_hex[4:6]} {pulse_hex[6:]} "
                f"{params['pos_mode']:02X} {params['sync']:02X} {params['check']:02X}"
            )
        
        elif mode == 'enable':
            params = {
                'addr': kwargs.get('addr', 0x01),
                'enable': kwargs.get('enable', 0x01),
                'sync': kwargs.get('sync', 0x00),
                'check': kwargs.get('check', 0x6B)
            }
            return f"{params['addr']:02X} F3 AB {params['enable']:02X} {params['sync']:02X} {params['check']:02X}"
        
        else:
            return "错误：支持模式为'speed'/'pos'/'enable'"

    def send_command(self, uart_send_func, mode, **kwargs):
        """发送运动控制命令"""
        cmd_str = self.get_command(mode, **kwargs)
        if cmd_str.startswith("错误"):
            return {"success": False, "message": cmd_str, "command": None, "response": None}
        
        try:
            cmd_bytes = list(bytes.fromhex(cmd_str.replace(" ", "")))
            uart_send_func(cmd_bytes)
            print(f"发送命令：{cmd_str}")
            
            return {
                "success": True,
                "message": "命令发送成功",
                "command": cmd_str,
                "response": None
            }
        except Exception as e:
            return {"success": False, "message": f"命令失败：{str(e)}", "command": cmd_str, "response": None}

    def sync_all_motors(self, uart_send_func):
        """发送多机同步命令（使用外部串口函数）"""
        try:
            sync_cmd = "00 FF 66 6B"
            cmd_bytes = list(bytes.fromhex(sync_cmd.replace(" ", "")))
            uart_send_func(cmd_bytes)
            print(f"发送同步命令：{sync_cmd}")
            
            return {
                "success": True,
                "message": "同步命令已发送",
                "command": sync_cmd,
                "response": None
            }
        except Exception as e:
            return {"success": False, "message": f"同步失败：{str(e)}", "command": sync_cmd, "response": None}

    def wait_for_multi_completion(self, uart_receive_func, addrs=[0x01, 0x02], timeout=30):
        """等待多个电机到位（使用外部串口函数）"""
        start_time = time.time()
        remaining_addrs = set(addrs)
        print(f"等待电机{list(remaining_addrs)}到位（超时{timeout}秒）...")

        while time.time() - start_time < timeout and remaining_addrs:
            response = uart_receive_func()
            if response:
                self._process_response(response, remaining_addrs)
            time.sleep(0.05)

        result = {
            "all_completed": len(remaining_addrs) == 0,
            "completed": [addr for addr in addrs if addr not in remaining_addrs],
            "timeout": list(remaining_addrs)
        }
        if result["all_completed"]:
            result["message"] = f"所有电机到位"
        else:
            result["message"] = f"超时：电机{result['timeout']}未到位"
        return result

    def _process_response(self, response, remaining_addrs):
        """辅助函数：解析响应并更新到位状态"""
        response_str = ' '.join([f"{b:02X}" for b in response]) if response else ""
        print(f"收到数据：{response_str}")
        response_parts = response_str.split()

        for i in range(0, len(response_parts), 4):
            if i + 3 >= len(response_parts):
                break
            resp_addr = int(response_parts[i], 16)
            resp_status = response_parts[i+2]

            if resp_addr in remaining_addrs and resp_status == "9F":
                remaining_addrs.remove(resp_addr)
                print(f"电机{resp_addr}已到位")

    def close(self):
        """关闭串口连接"""
        if self.ser.is_open:
            self.ser.close()
            print("串口已关闭")


# 使用示例
if __name__ == "__main__":
    try:
        motor = MotorController(port='COM28')
        
        # 1. 使能电机
        print("\n=== 1. 使能电机 ===")
        motor.send_command('enable', enable=0x01, addr=0x01)
        
        # 2. 短距离移动测试（快速到位场景）
        print("\n=== 2. 短距离移动（100脉冲） ===")
        motor.send_command('pos', addr=0x01, pulses=1000, speed=50, pos_mode=1)
        # 超时时间设为短距离所需时间（0.5秒）
        motor.wait_for_multi_completion(addrs=[0x01], timeout=0.5)
        
        # 3. 回零流程
        print("\n=== 3. 回零相关操作 ===")
        # 移动到零点物理位置
        motor.send_command('pos', addr=0x01, pulses=4000, speed=50, pos_mode=1)
        motor.wait_for_multi_completion(addrs=[0x01], timeout=2)
        
        # # 设置零点
        # set_result = motor.set_zero_point(addr=0x02, save_flag=0x01)
        # print(f"零点设置：{set_result['message']}，响应：{set_result['response']}")
        
        # 移动到非零点位置
        motor.send_command('pos', addr=0x01, pulses=1000, speed=50, pos_mode=1)
        motor.wait_for_multi_completion(addrs=[0x01], timeout=2)
        
        # 触发回零
        home_result = motor.trigger_homing(addr=0x01, homing_mode=0x00)
        print(f"回零触发：{home_result['message']}，响应：{home_result['response']}")
        
    except Exception as e:
        print(f"初始化失败：{str(e)}")
    finally:
        if 'motor' in locals():
            motor.close()
