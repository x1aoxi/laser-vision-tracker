class PID():
    def __init__(self, kp, ki, kd, integral_limit=100, derivative_limit=10):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_sum = 0  # 用于存储积分项的累积值
        self.integral_limit = integral_limit  # 积分限幅
        self.derivative_limit = derivative_limit  # 微分限幅

    # kp: 比例控制
    # error: 当前误差
    def proportion(self, error):
        return self.kp * error

    # ki: 积分控制
    # dt: 时间间隔
    def integral(self, error, dt):
        # 计算积分项
        self.integral_sum += error * dt
        # 积分限幅
        self.integral_sum = max(-self.integral_limit, min(self.integral_limit, self.integral_sum))
        return self.ki * self.integral_sum

    # kd: 微分控制
    # last_error: 上一时刻的误差
    def derivative(self, error, last_error, dt):
        # 检查时间间隔是否为零，避免除以零错误
        if dt == 0:
            return 0
        derivative = self.kd * (error - last_error) / dt
        # 微分限幅
        derivative = max(-self.derivative_limit, min(self.derivative_limit, derivative))
        return derivative

    def pid_control(self, error, last_error, dt):
        p = self.proportion(error)
        i = self.integral(error, dt)
        d = self.derivative(error, last_error, dt)
        return p + i + d
    
    def reset(self):
        """重置PID状态（用于任务切换时）"""
        self.integral_sum = 0
        self.last_error = 0