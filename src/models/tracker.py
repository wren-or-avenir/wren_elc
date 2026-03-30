import math
import numpy as np

class Tracker:
    def __init__(self, img_width=640, img_height=480, vfov=48.0, hfov =80.0, f_pixel_h=725.6, real_height=17.5):
        
        self.f_pixel_h = f_pixel_h # 标定得到的垂直焦距 (像素)  
        self.real_height = real_height # 目标物理高度 (cm)
        self.img_width = img_width
        self.img_height = img_height
        self.vfov = vfov # 相机视场角 (垂直)
        self.hfov = hfov # 相机视场角 (水平)

        # 物理偏移补偿 (单位: cm)
        # --- 未装激光笔，先忽略视差 ---
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
        
        # 物理距离 D = (H_real * f_pix) / H_pix
        dist = (self.real_height * self.f_pixel_h) / avg_h_px

        # 空间向量构造 
        # 归一化坐标 (-1 到 1)
        u, v = board.center
        norm_x = (u - self.img_width / 2) / (self.img_width / 2)
        norm_y = (v - self.img_height / 2) / (self.img_height / 2)

        # 将归一化坐标转换为物理坐标 (cm)，考虑视场角
        # x_phys = dist * math.tan(math.radians(self.hfov / 2)) * norm_x
        # y_phys = dist * math.tan(math.radians(self.vfov / 2)) * norm_y
        # z_phys = dist 
        # 如果云台水平转过头，说明商家给的视场角虚标，直接使用这一段代码替代利用FOV换算的方法
        x_phys = dist * offset_x_px / self.f_pixel_h
        y_phys = dist * offset_y_px / self.f_pixel_h
        z_phys = dist
        p_target_cam = np.array([x_phys, y_phys, z_phys])

        # 视差补偿 
        # 执行器指向目标的向量 = 目标物理位置 - 执行器相对相机的偏移
        delta = p_target_cam - self.ref_point

        # 计算最终云台角度
        yaw = math.degrees(math.atan2(delta[0], delta[2]))
        pitch = math.degrees(math.atan2(delta[1], math.sqrt(delta[0]**2 + delta[2]**2)))

        return yaw, pitch, dist