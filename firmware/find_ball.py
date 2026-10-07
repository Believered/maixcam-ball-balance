'''
    电赛H题摆杆平衡 - YOLO26 检测钢球, 输出钢球中心 x(沿轨道方向)
    基于 pan-tilt find_circle.py 简化(去掉透视矫正/circle3, 摆杆只要球位置)
    依赖: 模型 /root/models/Local/SteelBall/steel_ball.mud
    注: draw_string 用英文 (MaixCam 默认字体不支持中文)

    
    select_by_conf = True         # True=取置信度最大框, False=取面积最大框(原逻辑)
'''

from maix import camera, display, image, nn, app, time
import os


class FindBall:
    # ===== config =====
    model_path = "/root/models/Local/SteelBall/steel_ball.mud"
    model_dual_buff_mode = True   # dual_buff 提帧 (采集检测并行, +1帧延迟)
    cam_buff_num = 1   # 实测2未提帧(瓶颈在detect不在缓冲), 保持1(零延迟)
    # cam_fps = 80     # 实测80未提帧(瓶颈在detect不在sensor), 注释用默认60
    contrast = 80
    debug_draw_err_line = True    # 误差线 (main.py 提帧时覆盖 False)
    select_by_conf = True         # True=取置信度最大框, False=取面积最大框(原逻辑)
    debug_draw_rect = False        # 画钢球框(调试, 交付关省~0.5ms)
    debug_draw_center = True      # 画中心点(调试, 交付关省~0.2ms)
    DEBUG_DETECT = False           # 调试: 每帧print检测结果(N/置信度/MISS), 交付改False

    def __init__(self, disp):
        self.disp = disp
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Steel-ball model is required: {self.model_path}")
        self.detector = nn.YOLO26(model=self.model_path, dual_buff=self.model_dual_buff_mode)
        # cam 直接采 YOLO 输入尺寸(320x224), 无大图无 resize
        self.cam = camera.Camera(self.detector.input_width(), self.detector.input_height(),
                                 self.detector.input_format(), buff_num=self.cam_buff_num)
        self.cam.constrast(self.contrast)

        self.center_pos = [self.cam.width() // 2, self.cam.height() // 2]  # 画面中心
        self.last_center = self.center_pos   # 最近钢球中心 [cx, cy]
        self.updated = False
        self.last_img = None                 # 最近一帧图, 供外部叠加 UI
        self.auto_show = True                # main 叠加 UI 后再 show 时置 False

    def run(self):
        '''YOLO 检测最大钢球. 返回 last_center [cx, cy]'''
        self.updated = False
        img = self.cam.read()
        objs = self.detector.detect(img, conf_th=0.5, iou_th=0.45)
        if self.DEBUG_DETECT:
            if objs:
                confs = [round(o.score, 2) for o in objs]
                print(f"det: N={len(objs)} confs={confs}")
            else:
                print("det: MISS")

        if objs:
            # 多检测时选哪个框(单目标场景都一样, 多误检时择优)
            if self.select_by_conf:
                obj = max(objs, key=lambda o: o.score)   # 置信度最大(推荐)
            else:
                obj = max(objs, key=lambda o: o.w * o.h)  # 面积最大(原逻辑)
            cx = obj.x + obj.w // 2
            cy = obj.y + obj.h // 2
            self.last_center = [cx, cy]
            self.updated = True
            # 画钢球框 + 中心点(调试, 交付关 debug_draw_rect/center 提帧)
            if self.debug_draw_rect:
                img.draw_rect(obj.x, obj.y, obj.w, obj.h, image.Color(0, 255, 0), thickness=2)
            if self.debug_draw_center:
                img.draw_circle(cx, cy, 3, image.COLOR_RED, thickness=-1)
        else:
            self.last_center = self.center_pos   # 无钢球 -> 偏差归 0

        if self.debug_draw_err_line:
            img.draw_line(self.center_pos[0], self.center_pos[1],
                          self.last_center[0], self.last_center[1], image.COLOR_RED, thickness=3)

        self.last_img = img
        if self.auto_show:
            self.disp.show(img)
        return self.last_center


# 降频 show: 每 N 帧显示一次(show 时含绘制内容, 不 show 的帧纯检测提帧); 0=完全关 show(最快但看不到)
SHOW_EVERY_N = 10

if __name__ == "__main__":
    disp = display.Display()
    finder = FindBall(disp)
    finder.debug_draw_err_line = True
    finder.auto_show = False   # 关自动 show, 下面手动降频 show
    _t0 = time.ticks_ms()
    _cnt = 0
    while not app.need_exit():
        finder.run()
        _cnt += 1
        # 降频 show (show 时有绘制内容, 不影响观察; 不 show 的帧纯检测提帧)
        if SHOW_EVERY_N > 0 and _cnt % SHOW_EVERY_N == 0:
            if finder.last_img is not None:
                disp.show(finder.last_img)
        # 每 30 帧 print 一次 FPS
        if _cnt % 30 == 0:
            dt = time.ticks_ms() - _t0
            fps = _cnt * 1000.0 / dt if dt > 0 else 0
            print(f"FPS: {fps:.1f} ({_cnt} frames / {dt} ms)")
            _t0 = time.ticks_ms()
            _cnt = 0
