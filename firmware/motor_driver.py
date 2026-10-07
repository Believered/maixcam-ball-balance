# 摆杆平衡电机驱动 + PID 控制 (MaixCam 直控 ZDT_X42S, 单轴 pitch)
# 基于 pan-tilt code_velocity_feedforward/motor_driver.py 改单轴(去 yaw)
# 方案: 速度模式 PID (0xF6), 串口控制
# 详见 emm_protocol.md (命令) 和 云台架构决策.md (方案)
#
# 依赖: maix (uart, pinmap, err, time)
# 串口: UART1 A19(TX)/A18(RX) -> pitch 电机(地址2) + 主控(0x24, 前馈用)
# 接线: 原 pan-tilt pitch 轴拆下, 接线不变

from maix import uart, pinmap, err, time
from frame_decoder import FrameDecoder

# ==================================================
# 串口初始化
# ==================================================
UART_DEVICE = "/dev/ttyS1"   # 调试; 上车改 "/dev/ttyS0"
UART_BAUD = 115200

serial = None


def init_uart():
    """Configure pins and UART on first use; importing this module sends no command."""
    global serial
    if serial is None:
        for pin, function in {"A19": "UART1_TX", "A18": "UART1_RX"}.items():
            err.check_raise(pinmap.set_pin_function(pin, function), f"set {pin} to {function} failed")
        serial = uart.UART(UART_DEVICE, UART_BAUD)
        print(f"[motor_driver] UART: {UART_DEVICE} @ {UART_BAUD}")
    return serial


CHECKSUM = 0x6B

# ==================================================
# 串口路由 + RX 消化
# ==================================================
_last_tx_ts = {}
_last_rx_ts = {}
_decoder = FrameDecoder()

def _send(addr, payload):
    """发送: addr + payload + 0x6B (首字节即路由地址)"""
    init_uart().write(bytes([addr]) + payload + bytes([CHECKSUM]))
    _last_tx_ts[addr] = time.ticks_ms()

_position = {}   # addr -> 当前角度(度), 读位置命令(0x36)填充
_status = {}     # addr -> 状态标志(0x3A), bit2堵转/bit3堵转保护
DEBUG_RX = True   # 调试: 打印0x36/0x3A原始帧. 交付改False

def digest_rx(max_bytes=128):
    """读取电机返回帧. 位置回复按8字节、状态/ACK按4字节增量解析；支持分片和噪声恢复。"""
    port = init_uart()
    n = min(port.available(), max_bytes)
    if n <= 0:
        return 0
    data = port.read(n) or b""
    for addr, command, payload in _decoder.feed(data):
        _last_rx_ts[addr] = time.ticks_ms()
        if command == 0x36:
            angle = int.from_bytes(payload[1:], 'big') * 360.0 / 65536.0
            _position[addr] = -angle if payload[0] else angle
        elif command == 0x3A:
            _status[addr] = payload[0]
        if DEBUG_RX:
            print(f"rx: addr={addr} command=0x{command:02x} payload={payload.hex()}")
    return len(data)


LINK_TX_RECENT = 1000
LINK_RX_TIMEOUT = 800

def link_status(addr):
    now = time.ticks_ms()
    tx_ts = _last_tx_ts.get(addr)
    rx_ts = _last_rx_ts.get(addr)
    if tx_ts is not None and (now - tx_ts) <= LINK_TX_RECENT:
        if rx_ts is not None and (now - rx_ts) <= LINK_RX_TIMEOUT:
            return 'ok'
        return 'lost'
    return 'idle'

def master_send(payload):
    """发主控帧 (首字节 0x24). payload 不含地址和校验"""
    _send(0x24, payload)

# ==================================================
# 电机命令 (Emm 5.0)
# ==================================================
def motor_enable(addr, state=True):
    _send(addr, bytes([0xF3, 0xAB, 0x01 if state else 0x00, 0x00]))

def motor_stop(addr):
    _send(addr, bytes([0xFE, 0x98, 0x00]))

def motor_zero(addr):
    _send(addr, bytes([0x0A, 0x6D]))

def motor_set_origin(addr, save=True):
    """设置当前位置为永久零点(手册5.4.1: 93 88 [存] 6B). save=True掉电保存.
    标定: 摆杆放水平后调用一次, 之后上电0°=水平固定(不再随上电位置变)"""
    _send(addr, bytes([0x93, 0x88, 0x01 if save else 0x00]))

def motor_home(addr, mode=0x04, sync=False):
    """触发回零(5.4.2: 9A [模式] [同步] 6B). mode=04回绝对零点(转到93 88设置的零点).
    上电后调用, 电机转到永久零点(水平), 回零后0x36的0=水平.
    ⚠️电机会转, 确保摆杆能自由转不撞"""
    _send(addr, bytes([0x9A, mode, 0x01 if sync else 0x00]))

def motor_set_closed_loop(addr, closed_loop=True, save=False):
    _send(addr, bytes([0x46, 0x69, 0x01 if save else 0x00, 0x01 if closed_loop else 0x00]))

def motor_speed(addr, speed_rpm, accel=50, sync=False):
    """速度模式 (0xF6, 手册5.3.7). speed_rpm 带符号: +CW / -CCW, |speed|<=3000"""
    direction = 0 if speed_rpm > 0 else 1
    spd = min(abs(int(speed_rpm)), 3000)
    _send(addr, bytes([0xF6, direction, (spd >> 8) & 0xFF, spd & 0xFF, accel & 0xFF, 0x01 if sync else 0x00]))

def motor_move_rel(addr, pulses, speed_rpm=20, accel=50):
    """相对当前位置移动 pulses 脉冲 (正CW/负CCW). 手册5.3.12, mode=02"""
    direction = 0 if pulses >= 0 else 1
    p = min(abs(int(pulses)), 0x7FFFFFFF)
    _send(addr, bytes([0xFD, direction,
                       (speed_rpm >> 8) & 0xFF, speed_rpm & 0xFF, accel & 0xFF,
                       (p >> 24) & 0xFF, (p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF,
                       0x02, 0x00]))

def motor_read_position(addr):
    """读电机当前位置(度). 发 addr 0x36 0x6B, 异步返回(digest_rx解析到 _position[addr])"""
    _send(addr, bytes([0x36]))

def motor_get_position(addr):
    """返回电机实际角度(度), 由digest_rx解析0x36填充. 没读过返回None"""
    return _position.get(addr)

def motor_read_status(addr):
    """读电机状态标志. 发 addr 0x3A 0x6B, 异步返回(digest_rx解析到 _status[addr])"""
    _send(addr, bytes([0x3A]))

def motor_get_status(addr):
    """返回状态标志(0x3A). bit2=堵转, bit3=堵转保护触发. 没读过返回None"""
    return _status.get(addr)

# 位置模式换算 (16细分, 3200脉冲/圈)
PULSES_PER_DEG = 3200 / 360.0   # 8.89 脉冲/度

def motor_move_to(addr, angle_deg, speed_rpm=80, accel=200):
    """绝对位置模式 (0xFD mode 01). angle_deg: 目标角度(度, 0=清零位置).
    电机转到绝对角度, 适合球杆平衡(直接控倾角, 稳态归水平)"""
    pulses = int(angle_deg * PULSES_PER_DEG)
    direction = 0 if pulses >= 0 else 1
    p = min(abs(pulses), 0x7FFFFFFF)
    _send(addr, bytes([0xFD, direction,
                       (speed_rpm >> 8) & 0xFF, speed_rpm & 0xFF, accel & 0xFF,
                       (p >> 24) & 0xFF, (p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF,
                       0x01, 0x00]))  # mode 01 = 绝对坐标

# ==================================================
# PID 控制器
# ==================================================
class PID:
    def __init__(self, kp, ki, kd, out_max, ki_max=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.ki_max = ki_max
        self._err_prev = 0
        self._integ = 0

    def reset(self):
        self._err_prev = 0
        self._integ = 0

    def update(self, error):
        deriv = error - self._err_prev
        self._err_prev = error
        out = self.kp * error + self._integ + self.kd * deriv
        # 抗积分饱和: out未饱和(theta未满限位)才积分(球卡死看角度不看err, theta满限位积分无意义)
        if -self.out_max < out < self.out_max:
            self._integ += error * self.ki
            if self._integ > self.ki_max: self._integ = self.ki_max
            elif self._integ < -self.ki_max: self._integ = -self.ki_max
            out = self.kp * error + self._integ + self.kd * deriv
        if out > self.out_max: out = self.out_max
        elif out < -self.out_max: out = -self.out_max
        return out

# ==================================================
# 三段式速度/加速度调度
# ==================================================
ERR_SMALL = 6
ERR_LARGE = 12
DEAD_ZONE = 3

# pitch 速度上限 (RPM) 和加速度三档 -- 初始值, 现场调
# 摆杆负载和云台不同, 球杆二阶系统易振荡, 用保守值
PITCH_LIMITS = (6, 10, 12)
PITCH_ACCELS = (60, 100, 150)

def _profile(error, limits, accels):
    a = abs(error)
    if a < ERR_SMALL:
        return limits[0], accels[0]
    elif a >= ERR_LARGE:
        return limits[2], accels[2]
    else:
        return limits[1], accels[1]

# ==================================================
# 摆杆控制器 (单轴 pitch)
# ==================================================
class BallController:
    """单轴摆杆 PID 速度控制. main.py 每帧调用 track(err_x)
    err_x: 钢球 x - 目标 x (像素, 偏右为正)
    -> pitch 电机速度 (调摆杆倾角让球回中)

    kp 符号: 球偏右(err>0) -> 摆杆抬高某端让球滚回.
    方向取决于齿轮装法, 机械到位后实测, 反了给 kp 加/去负号.
    """
    def __init__(self, pitch_addr=2):
        self.pitch_addr = pitch_addr
        # PID 参数(保守初始, 现场调): 球杆二阶系统, kd 要大抑振荡
        # self.pid = PID(kp=0.25, ki=0, kd=0.06, out_max=25, ki_max=5)   
        # self.pid = PID(kp=0.2, ki=0.002, kd=0.3, out_max=15, ki_max=5)   # 球杆二阶系统: kd=速度反馈强阻尼(deriv=球速度); out_max=15(=MAX_ANGLE抗饱和, theta满限位不积分)   
        self.pid = PID(kp=0.2, ki=0.01, kd=9, out_max=25, ki_max=8)   # task1平衡用(已调稳)
        
        # self.pid_t3 = PID(kp=0.205, ki=0.0023, kd=13, out_max=25, ki_max=6)  # task3轨迹跟踪用(单独调, 初始同task1) 
        self.pid_t3 = PID(kp=0.205, ki=0.01, kd=13, out_max=25, ki_max=6)  # task3轨迹跟踪用(单独调, 初始同task1)   
  

        self.enabled = False
        self.dead_zone = 3      # 死区(像素), 偏差小于此停转防抖
        self.min_speed = 1      # 速度<此值停转(RPM)

        # 摆杆角度限位(软件累计速度估算角度, 超限停转防机械碰撞)
        self.MAX_ANGLE_T1 = 15     # task1平衡摆杆限位(±度)
        self.MAX_ANGLE_T3 = 18     # task3轨迹跟踪摆杆限位(±度, 单独调)
        self.MAX_DTHETA = 2.0        # theta每帧最多变2°(防高频跳变电机卡死, 70FPS=140°/s)
        self.DTHETA_DEAD = 0.5       # theta变化<0.5°不发命令(减少move_to频率让电机执行)
        self.current_angle = 0.0     # 上次发出的目标角度(度)
        self._last_speed = 0         # 上次输出速度(累计用)
        self._last_ms = None         # 上次 track 时间戳
        self.RPM_TO_DEG_PER_S = 6.0  # RPM->度/秒(电机1RPM=6°/s; 齿轮比g: 改6/g; 直驱=6.0, 标定)

    def enable(self):
        motor_enable(self.pitch_addr, True)
        self.enabled = True

    def disable(self):
        motor_enable(self.pitch_addr, False)
        self.enabled = False

    def reset(self):
        """重置 PID 积分 + 角度累计 (切换任务/目标丢失时调用)"""
        self.pid.reset()
        self.pid_t3.reset()
        self.current_angle = 0.0
        self._last_speed = 0
        self._last_ms = None

    def track(self, err_x, pid=None):
        """每帧调用. 位置模式: PID输出θ, clamp±MAX_ANGLE, 变化率限幅+变化阈值发命令
        pid=None用self.pid(task1平衡), 传self.pid_t3用task3参数. (防70FPS高频发move_to+theta跳变电机卡死)"""
        if not self.enabled:
            return
        if pid is None:
            pid = self.pid
            max_angle = self.MAX_ANGLE_T1
        else:
            max_angle = self.MAX_ANGLE_T3
        if abs(err_x) < self.dead_zone:
            # 球到目标, 摆杆归水平(θ=0). 只在偏离0时发, 避免每帧重发
            if abs(self.current_angle) > self.DTHETA_DEAD:
                motor_move_to(self.pitch_addr, 0.0)
                self.current_angle = 0.0
            pid.reset()
        else:
            theta = pid.update(err_x)
            if theta > max_angle: theta = max_angle
            elif theta < -max_angle: theta = -max_angle
            # 变化率限幅: 每帧最多变MAX_DTHETA度, 防theta高频跳变电机收到冲突目标卡死
            dtheta = theta - self.current_angle
            if dtheta > self.MAX_DTHETA: theta = self.current_angle + self.MAX_DTHETA
            elif dtheta < -self.MAX_DTHETA: theta = self.current_angle - self.MAX_DTHETA
            # 变化阈值: theta变化小不发, 减少move_to频率让电机有时间执行
            if abs(theta - self.current_angle) < self.DTHETA_DEAD:
                return
            self.current_angle = theta
            motor_move_to(self.pitch_addr, theta)

    def stop(self):
        motor_stop(self.pitch_addr)
