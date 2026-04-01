import math
import numpy as np

class Tracker:
    def __init__(self, img_width=640, img_height=480, vfov=48.0, hfov =80.0, f_pixel_h=725.6, real_height=17.5):
        
        self.f_pixel_h = f_pixel_h # 标定得到的垂直焦距 (像素)  
        self.real_height = real_height # 目标物理高度 (cm)
        self.img_width = img_width
        self.img_height = img_height
        self.vfov = vfov # 相机垂直视场角 
        self.hfov = hfov # 相机水平视场角 

        # 物理偏移补偿 (单位: cm)
        # 未装激光笔，先忽略视差 
        # 如果以后激光笔装在相机左边 3cm, 就写 np.array([-3.0, 0, 0])
        self.ref_point = np.array([0.0, 0.0, 0.0]) 
        
    def solve(self, board):
        if not board or not board.is_valid:
            return None

        # 计算距离
        pts = board.points
        h_left = math.sqrt((pts[0][0]-pts[1][0])**2 + (pts[0][1]-pts[1][1])**2)
        h_right = math.sqrt((pts[3][0]-pts[2][0])**2 + (pts[3][1]-pts[2][1])**2)
        avg_h_px = (h_left + h_right) / 2.0
        dist = (self.real_height * self.f_pixel_h) / avg_h_px

        # 计算像素偏移
        u, v = board.center
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