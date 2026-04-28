import Hobot.GPIO as GPIO
import time

# 全局初始化（官方示例标准写法，避免多实例重复配置冲突）
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

class GPIN:
    def __init__(self, pin=1, mode=1):
        self.pin = pin
        self.mode = mode
        self.status = GPIO.LOW  # 软件电平缓存，减少无效总线写入
        # 主线程心跳检测
        self.last_heartbeat = time.time()
        self.timeout = 3.0  # 3秒超时熔断
        # 呼吸灯状态机
        self.duty = 0
        self.step = 2
        self._counter = 0
        # 严格模式隔离配置
        if self.mode == 1:
            GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        elif self.mode == 0:
            GPIO.setup(self.pin, GPIO.IN)

    def set_value(self, value):
        # 输出模式专属，拦截误写
        if self.mode != 1:
            print(f"错误: 引脚 {self.pin} 是输入模式，无法设置值")
            return
        # 开火极简映射
        self.status = GPIO.LOW if value == 1 else GPIO.HIGH
        GPIO.output(self.pin, self.status)

    def read_status(self):
        # 输入模式专属，拦截误读
        if self.mode == 1:
            print(f"错误: 引脚 {self.pin} 是输出模式，无法读取状态")
            return None
        # 返回1或0, 1 表示按下(低电平)，0 表示松开(高电平)
        return int(GPIO.input(self.pin) == GPIO.LOW)

    def heartbeat(self):
        # 刷新主线程存活时间戳
        self.last_heartbeat = time.time()

    def flash(self):
        # 外部主循环每调用一次，内部推进一帧。绝不 sleep 阻塞业务
        self.heartbeat()
        self._update_breathing()

    def _update_breathing(self):
        # 超时熔断：主线程卡死>3秒，强制拉低灭灯
        if time.time() - self.last_heartbeat > self.timeout:
            GPIO.output(self.pin, GPIO.LOW)
            self.duty = 0
            self.step = 2
            self._counter = 0
            return

        # PWM步进：10次调用为1个周期，高电平次数由占空比决定
        self._counter = (self._counter + 1) % 10
        high_ticks = int(self.duty / 10)
        GPIO.output(self.pin, GPIO.HIGH if self._counter < high_ticks else GPIO.LOW)

        # 亮度渐变：每完成1个周期调整一次占空比
        if self._counter == 0:
            self.duty += self.step
            if self.duty >= 100:
                self.duty = 100
                self.step = -2
            elif self.duty <= 0:
                self.duty = 0
                self.step = 2

    def cleanup(self):
        # 安全释放，避免残留高电平
        GPIO.output(self.pin, GPIO.LOW)
        GPIO.cleanup()