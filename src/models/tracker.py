import math
import numpy as np
from .Kalman import KalmanFilter
import time

class Tracker:
    def __init__(self, img_width=640, img_height=480, vfov=48.0, hfov =80.0, use_kf = False,  f_pixel_h=725.6, real_height=17.5):
        
        self.f_pixel_h = f_pixel_h # 标定得到的垂直焦距 (像素)  
        self.real_height = real_height # 目标物理高度 (cm)
        self.img_width = img_width
        self.img_height = img_height
        self.vfov = vfov # 相机垂直视场角 
        self.hfov = hfov # 相机水平视场角 
        self.use_kf = use_kf    # 是否启用卡尔曼滤波器
        self.kf_cx = KalmanFilter(q_scale=0.01, r_scale=0.1) # cx 卡尔曼滤波器
        self.kf_cy = KalmanFilter(q_scale=0.01, r_scale=0.1)    # cy 卡尔曼滤波器
        self.kf_dist = KalmanFilter(q_scale=0.01, r_scale=2.0)   # distance 卡尔曼滤波器
        self.lost = 0  # 丢帧数
        self.frame_tol = 5   # 丢帧容忍度 
        self.filtered_center = (img_width / 2, img_height / 2) 
        self.filtered_dist = 0.0
        self.last_time = None
        # 物理偏移补偿 (单位: cm)
        self.ref_point = np.array([0.0, 1.4, 0.0])      # 激光笔测量得出位于相机光轴上方1.4cm,视差补偿为 np.array([0.0, 1.4, 0.0])

    def time_diff(self):
        """
        起到一个动态dt的作用
        """
        current_time = time.time_ns()

        if self.last_time[0] is None:
            self.last_time[0] = current_time
            return 0.033
        else :
            diff = (current_time - self.last_time[0]) 
            self.last_time[0] = current_time
            return diff / 1e9  # 转换为秒
        
    def get_dist(self, board):
        # 计算距离
        pts = board.points
        h_left = math.sqrt((pts[0][0]-pts[1][0])**2 + (pts[0][1]-pts[1][1])**2)
        h_right = math.sqrt((pts[3][0]-pts[2][0])**2 + (pts[3][1]-pts[2][1])**2)
        avg_h_px = (h_left + h_right) / 2.0
        dist = (self.real_height * self.f_pixel_h) / avg_h_px
        if avg_h_px < 1: return 1000.0
        return dist      

    def frame_add(self, board):
        dt = self.time_diff()
        self.kf_cx.predict(dt)
        self.kf_cy.predict(dt)
        self.kf_dist.predict(dt)
        # 如果丢帧了
        if not board or not board.is_valid:
            if self.use_kf == True:
                self.lost += 1
                if self.lost <= self.frame_tol:
                    self.lost_center = (self.kf_cx.get_state()[0], self.kf_cy.get_state()[0])
                    self.lost_dist = self.kf_dist.get_state()[0]
                else:
                    # 超过丢帧容忍度，重置状态
                    self.lost_center = (0, 0)
                    self.lost_dist = 0.0
                    self.kf_cx.reset()
                    self.kf_cy.reset()
                    self.kf_dist.reset()
            else:
                self.lost_center = (0, 0)
                self.lost_dist = 0.0
        else:
            self.lost = 0 # 重置丢帧计数器
            if self.use_kf == True:
                self.kf_cx.update(board.center[0])
                self.kf_cy.update(board.center[1])

    def solve(self):

        dist = self.filtered_dist if self.filtered_dist > 0 else 0.1

        u, v = self.filtered_center

        offset_x = u - self.img_width / 2
        offset_y = v - self.img_height / 2

        # 考虑视差补偿,目前视差补偿为0，后续如果装激光笔了再调整
        dx = offset_x - (self.ref_point[0] * self.f_pixel_h / dist if dist != 0 else 0)
        dy = offset_y - (self.ref_point[1] * self.f_pixel_h / dist if dist != 0 else 0)
        dz = self.f_pixel_h

        # 计算云台角度
        # Yaw: 逆时针为正 
        yaw = -math.degrees(math.atan2(dx, dz))
        # Pitch: 向下为正
        pitch = math.degrees(math.atan2(dy, dz))

        return yaw, pitch, dist