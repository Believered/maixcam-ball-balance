# 摆杆平衡 - 视觉检测钢球 + 单轴 PID 控制
# task0 校准(钢球放目标位置保存 target_x) / task1 平衡(PID 控球)
# 触摸屏交互: IDLE选task, task点击退出; USER按钮退出应用
# 注: draw_string 用英文(MaixCam 默认字体不支持中文)

from maix import app, time, display, image, touchscreen
from motor_driver import BallController, digest_rx, motor_read_position, motor_get_position, motor_read_status, motor_get_status, motor_move_to
from find_ball import FindBall

def main():
    ball_ctrl = None
    try:
        # ===== 初始化 =====
        disp = display.Display()
        finder = FindBall(disp)
        finder.auto_show = False
        finder.debug_draw_err_line = False    # 画检测线(center->ball红线)
        finder.debug_draw_rect = False       # 控制循环严禁draw(省2-4ms), 调试改True看效果
        finder.debug_draw_center = True      # 画钢球中心红点(开销<检测线, 直观看检测到没)
        center_pos = finder.center_pos   # 画面中心 [160,112] (320x224)

        ball_ctrl = BallController()
        ball_ctrl.enable()
        time.sleep_ms(500)

        # 触摸屏
        ts = touchscreen.TouchScreen()
        _ts_pressed = False

        # ===== target_x 持久化 =====
        DATA_FILE = "/root/target_x.txt"

        def save_target_x(x):
            try:
                with open(DATA_FILE, "w") as f:
                    f.write(str(x))
                return True
            except OSError:
                return False

        def load_target_x():
            try:
                with open(DATA_FILE, "r") as f:
                    return int(f.read().strip())
            except (OSError, ValueError):
                return None

        _loaded = load_target_x()
        target_x = _loaded if _loaded is not None else center_pos[0]
        print(f"target_x: {target_x}")

        # ===== 状态机 =====
        STATE_IDLE = 0
        STATE_T0 = 1  # 校准
        STATE_T1 = 2  # 平衡
        STATE_T3 = 3  # task3: O->+5cm->-5cm ≤5s
        state = STATE_IDLE

        _show_counter = 0
        SHOW_EVERY_N = 2   # 归档值；显示节奏及帧率须在匹配硬件重新验证
        _last_fps_print = 0
        _last_err = None   # 调试: 最近水平偏差(像素), FPS打印用
        DEBUG_MOTOR_POS = True   # 调试: 打印电机实际角度(周期读0x36). 交付改False零开销

        # ===== task3 参数 (O->+5cm->-5cm, ≤5s, ±1cm误差) =====
        POS_5CM_PX_POS = 62   # +5cm对应像素(标定: 球放+5cm看err) -- +5/-5不对称(镜头畸变/装配)
        POS_5CM_PX_NEG = 66     # -5cm对应像素(标定: 球放-5cm看err)
        T3_T_POS = 2       # O->+5cm 时间(s)
        T3_T_NEG = 2.5       # +5->-5cm 时间(s, 从+5开始算) -- 当前用T3_T_POS后直接切-5, 此值备查
        T3_MODE = 1          # 0=时间写死(目标+5/-5,PID闭环) 1=角度写死(开环发T3_FIXED_ANGLES)
        # 角度写死4段式(模式1用): a1往+5加速 -> a2往-5加速 -> a3刹车 -> 0稳定
        T3_A1 = -18;  T3_N1 = 1.0   # 阶段1: 角度a1(负,往+5), 持续n1秒(球到+5)
        T3_A2 = 18;   T3_N2 = 1.5   # 阶段2: 角度a2(正,往-5), 持续n2秒
        T3_A3 = -15;  T3_N3 = 0.8   # 阶段3: 角度a3(负,刹车), 持续n3秒(球停-5)
        # 阶段4: 0° 无限稳定
        _t3_start = None     # None=归O中, 有值=开始计时
        _t3_target = 0       # task3 当前目标x

        def _t3_fixed_angle(t):
            """模式1: 4段式角度. a1往+5 -> a2往-5 -> a3刹车 -> 0稳定"""
            if t < T3_N1:
                return T3_A1
            elif t < T3_N1 + T3_N2:
                return T3_A2
            elif t < T3_N1 + T3_N2 + T3_N3:
                return T3_A3
            else:
                return 0

        # 颜色
        WHITE  = image.Color(255, 255, 255)
        GREEN  = image.Color(0, 255, 0)
        YELLOW = image.Color(255, 255, 0)
        CYAN   = image.Color(0, 255, 255)
        RED    = image.Color(255, 0, 0)
        BLUE   = image.Color(0, 0, 255)

        def draw_idle(img):
            """IDLE: 画 3 task 按钮 (上中下)"""
            img.clear()
            w, h = img.width(), img.height()
            tasks = ["task0 Calib", "task1 Balance", "task3 Run"]
            for i, name in enumerate(tasks):
                by = i * (h // 3)
                img.draw_rect(0, by, w, h // 3, WHITE, thickness=2)
                img.draw_string(10, by + 10, name, GREEN, scale=2)
            img.draw_string(2, h - 25, "Tap to select", YELLOW, scale=1.5)

        def get_touched_task(x, y):
            w, h = disp.width(), disp.height()
            if y < h // 3: return 0
            elif y < 2 * h // 3: return 1
            else: return 2

        def draw_task_ui(img, extra=""):
            if extra:
                img.draw_string(2, 2, extra, CYAN, scale=1.0)

        # ===== FPS (滑动平均) =====
        _last_t = time.ticks_ms()
        _fps = 0.0

        def draw_fps(img):
            img.draw_string(img.width() - 60, 2, f"{int(_fps)} FPS", CYAN, scale=1.0)

        print("===== Ball Balance State Machine =====")
        print(f"center: {center_pos}, target_x: {target_x}")

        # ===== 主循环 =====
        while not app.need_exit():
            x, y, pressed = ts.read()
            clicked = False
            if pressed and not _ts_pressed:
                _ts_pressed = True
                clicked = True
            elif not pressed:
                _ts_pressed = False

            _show_counter += 1

            # FPS 滑动平均
            _now = time.ticks_ms()
            _dt = _now - _last_t
            if _dt > 0:
                _inst = 1000.0 / _dt
                _fps = _fps * 0.85 + _inst * 0.15 if _fps > 0 else _inst
            _last_t = _now

            if _now - _last_fps_print >= 1000:
                if DEBUG_MOTOR_POS:
                    _actual = motor_get_position(ball_ctrl.pitch_addr)
                    _flag = motor_get_status(ball_ctrl.pitch_addr)
                    _stall = bool(_flag & 0x04) if _flag is not None else None
                    print(f"FPS: {int(_fps)} | err={_last_err} | theta={ball_ctrl.current_angle:.1f} | actual={_actual} | stall={_stall}")
                else:
                    print(f"FPS: {int(_fps)} | err={_last_err} | theta={ball_ctrl.current_angle:.1f}")
                _last_fps_print = _now

            if state == STATE_IDLE:
                if SHOW_EVERY_N > 0 and _show_counter % SHOW_EVERY_N == 0:
                    img = image.Image(disp.width(), disp.height())
                    draw_idle(img)
                    disp.show(img)
                if clicked:
                    t = get_touched_task(x, y)
                    if t == 0:
                        state = STATE_T0
                        ball_ctrl.disable()  # 校准电机失能
                        print("-> task0 Calib (motor disabled)")
                    elif t == 1:
                        state = STATE_T1
                        ball_ctrl.enable(); ball_ctrl.reset()
                        print("-> task1 Balance")
                    elif t == 2:
                        state = STATE_T3
                        ball_ctrl.enable(); ball_ctrl.reset()
                        _t3_start = None   # 进入task3第一帧开始计时
                        print("-> task3 O->+5->-5")
                time.sleep_ms(30)

            elif state == STATE_T0:  # 校准: 钢球放目标位置, 点击保存 ball_x 为 target_x
                finder.run()
                img = finder.last_img
                if img is not None and SHOW_EVERY_N > 0 and _show_counter % SHOW_EVERY_N == 0:
                    draw_task_ui(img, "task0: tap save")
                    # 画当前 target_x 竖线(蓝)
                    img.draw_line(target_x, 0, target_x, img.height(), BLUE, thickness=2)
                    disp.show(img)
                if clicked:
                    if finder.updated and finder.last_center:
                        target_x = finder.last_center[0]
                        save_target_x(target_x)
                        print(f"Calib saved: target_x={target_x}")
                    else:
                        print("Calib failed: no ball")
                    ball_ctrl.enable()
                    state = STATE_IDLE
                digest_rx()
                time.sleep_ms(20)

            elif state == STATE_T1:  # 平衡: err = ball_x - target_x, PID 控摆杆
                finder.run()
                if finder.updated:
                    ball_x = finder.last_center[0]
                    err_x = ball_x - target_x      # 偏右正(约定)
                    _last_err = err_x
                    ball_ctrl.track(err_x)
                    if DEBUG_MOTOR_POS and _show_counter % 10 == 0:
                        motor_read_position(ball_ctrl.pitch_addr)   # 10帧读一次实际角度, 避免串口过载
                        motor_read_status(ball_ctrl.pitch_addr)     # 读状态标志(堵转检测)
                else:
                    ball_ctrl.stop()
                    ball_ctrl.reset()
                    _last_err = None
                digest_rx()
                img = finder.last_img
                if img is not None and SHOW_EVERY_N > 0 and _show_counter % SHOW_EVERY_N == 0:
                    draw_task_ui(img, f"target_x={target_x}")
                    # 画 target_x 竖线(红)
                    img.draw_line(target_x, 0, target_x, img.height(), RED, thickness=2)
                    disp.show(img)
                if clicked:
                    ball_ctrl.stop()
                    state = STATE_IDLE
                    print("<- exit balance")

            elif state == STATE_T3:  # task3: O->+5cm->-5cm ≤5s
                finder.run()
                if finder.updated:
                    ball_x = finder.last_center[0]
                    if _t3_start is None:
                        _t3_start = time.ticks_ms()   # 球已在O(用户保证), 直接开始计时
                        print(f"t3: start (mode={T3_MODE})")
                    t = (time.ticks_ms() - _t3_start) / 1000.0
                    if T3_MODE == 0:
                        # 模式0: 时间写死(目标+5/-5, PID闭环)
                        if t < T3_T_POS:
                            _t3_target = target_x + POS_5CM_PX_POS   # 阶段1: O->+5cm
                        else:
                            _t3_target = target_x - POS_5CM_PX_NEG   # 阶段2: +5->-5cm
                        err_x = ball_x - _t3_target
                        ball_ctrl.track(err_x, ball_ctrl.pid_t3)
                        theta = ball_ctrl.current_angle
                    else:
                        # 模式1: 角度写死(开环, 按时间发固定角度, 不走PID)
                        theta = _t3_fixed_angle(t)
                        ball_ctrl.current_angle = theta
                        motor_move_to(ball_ctrl.pitch_addr, theta)
                        err_x = ball_x - target_x   # 仅打印用(相对O)
                        _t3_target = target_x
                    _last_err = err_x
                    # 频繁打印(每3帧~40ms), 方便提取好的theta序列写死
                    if _show_counter % 3 == 0:
                        print(f"t3 t={t:.2f} ball={ball_x} err={err_x} theta={theta:.1f}")
                else:
                    ball_ctrl.stop()
                    ball_ctrl.reset()
                    _last_err = None
                digest_rx()
                img = finder.last_img
                if img is not None and SHOW_EVERY_N > 0 and _show_counter % SHOW_EVERY_N == 0:
                    _t = (time.ticks_ms() - _t3_start) / 1000.0 if _t3_start else 0
                    draw_task_ui(img, f"t3 t={_t:.1f} tgt={_t3_target}")
                    img.draw_line(int(_t3_target), 0, int(_t3_target), img.height(), RED, thickness=2)
                    img.draw_line(target_x, 0, target_x, img.height(), BLUE, thickness=2)
                    disp.show(img)
                if clicked:
                    ball_ctrl.stop()
                    state = STATE_IDLE
                    _t3_start = None
                    print("<- exit task3")

        ball_ctrl.disable()
        print("===== Exit =====")

    finally:
        if ball_ctrl is not None:
            # A lost camera/model/UART operation must not leave the motor enabled.
            try:
                ball_ctrl.stop()
            finally:
                ball_ctrl.disable()


if __name__ == "__main__":
    main()
