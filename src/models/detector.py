import cv2
import numpy as np

class Board:

    def __init__(self):
        self.points = []       # 四个端点坐标，顺序: [左上, 左下, 右下, 右上]
        self.center = None     # 对角线交点坐标 (x, y)
        self.area = 0.0        

class Detector:
    
    def __init__(self, min_area=3000, max_area=500000):
        self.board_min_area = min_area
        self.board_max_area = max_area
        self.threshold_value = 127
        self.boards = []
        self.raw = None
        self.binary = None 

    def process_image(self, frame):
        self.raw = frame
        # 转灰度
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)     
        # 反二值化,大津法自适应阈值
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)        
        self.binary = binary       # 存储当前帧的二值化结果
        return binary
        
    def find_board(self, binary):
        boards = []
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        
        # 1. 极度严谨的“空值与越界”拦截
        if hierarchy is None or len(contours) == 0 or len(hierarchy) == 0 or len(hierarchy[0]) != len(contours):
            self.boards = []
            return []

        inner_contours = []
        for i, contour in enumerate(contours):
            if hierarchy[0][i][3] != -1: 
                inner_contours.append((i, contour))
        
        if inner_contours:
            target_contours = inner_contours
        else:
            target_contours = [(i, c) for i, c in enumerate(contours) if hierarchy[0][i][3] == -1]
        
        for i, contour in target_contours:
            area = cv2.contourArea(contour)
            if self.board_min_area < area < self.board_max_area:
                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
                
                if len(approx) == 4:
                    # 2. 凸包性校验：物理世界的靶纸透视畸变后必定是凸多边形
                    if not cv2.isContourConvex(approx):
                        continue

                    # 3. 长宽比过滤与除零保护：排除电线、细长支架等干扰 (放宽比例到 0.8 ~ 1.8)
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / max(h, 1)
                    if not (0.8 < aspect_ratio < 1.8):
                        continue

                    # 质心极角排序：彻底解决不规则形状导致的点重合Bug
                    points = approx.reshape(4, 2)
                    cx, cy = np.mean(points, axis=0)
                    angles = np.arctan2(points[:, 1] - cy, points[:, 0] - cx)
                    sorted_idx = np.argsort(angles)
                    
                    # atan2在图像坐标系(y轴朝下)的排序结果严格依次为：左上, 右上, 右下, 左下
                    # 按照你原代码 [左上, 左下, 右下, 右上] 的顺序装填
                    sorted_points = [
                        points[sorted_idx[0]],  # 左上
                        points[sorted_idx[3]],  # 左下
                        points[sorted_idx[2]],  # 右下
                        points[sorted_idx[1]]   # 右上
                    ]
                    
                    board = Board()
                    board.points = [tuple(map(int, pt)) for pt in sorted_points]
                    board.area = area
                    board.center = self._calculate_intersection(board.points)
                    
                    if board.center is not None:
                        boards.append(board)
        
        if boards:
            # 按面积从大到小排序 
            boards.sort(key=lambda b: b.area, reverse=True)
            self.boards = boards
        else:
            self.boards = []

        return boards

    def _calculate_intersection(self, points):
        """
        使用两点式直线方程求交点公式
        直线1:点0(左上) -> 点2(右下)
        直线2:点1(左下) -> 点3(右上)
        """
        x1, y1 = points[0]
        x2, y2 = points[2]
        x3, y3 = points[1]
        x4, y4 = points[3]

        # 求解线性方程组的分母
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

        # 防止分母为0，从而导致程序崩溃
        if denominator == 0:
            return None

        # 两点式相交的中心点坐标公式
        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator

        return (int(px), int(py))

    def _draw_annotations(self, image):
        """
        靶纸绘制逻辑：
        绿色框框住整体矩形，蓝色画出对角线，绿色画出两直线交点
        """
        if not self.boards:
            return
            
        board = self.boards[0]  # 永远取排序后的第一个
        if not board.points or board.center is None:
            return
        
        pts = board.points
        
        # 用绿色的线框住识别出来的矩形: 左上->左下->右下->右上->闭合回左上
        cv2.line(image, pts[0], pts[1], (0, 255, 0), 2)
        cv2.line(image, pts[1], pts[2], (0, 255, 0), 2)
        cv2.line(image, pts[2], pts[3], (0, 255, 0), 2)
        cv2.line(image, pts[3], pts[0], (0, 255, 0), 2)

        # 蓝色画出对角线 
        cv2.line(image, pts[0], pts[2], (255, 0, 0), 2)
        cv2.line(image, pts[1], pts[3], (255, 0, 0), 2)
        
        # 绿色实心点画出两直线交点
        if board.center:
            cv2.circle(image, board.center, 5, (0, 255, 0), -1)

    def draw_center_dot(self, image):
        """绘制相机光轴"""
        h, w = image.shape[:2]
        # 橙色 
        cv2.circle(image, (w // 2, h // 2), 5, (0, 165, 255), -1)

    def draw(self, image):
        if image is None:
            return image

        # 绘制靶纸识别框、对角线、交点
        self._draw_annotations(image)
        # 绘制相机光轴
        self.draw_center_dot(image)
        
        return image

    def detect(self, frame):
        """对外接口，直接返回检测结果"""
        target = None
        # 核心检测逻辑
        bin = self.process_image(frame)
        boards = self.find_board(bin)
        return boards[0] if boards else None
    
    def display(self, dis):
        """对外接口,直接返回detector的图像"""
        # 绘制可视化结果
        bin, res = None, None
        # 防止首帧 self.raw 为 None 导致 .copy() 崩溃
        if self.raw is None:
            return None, self.binary
        # 避免修改原图数据
        vis = self.raw.copy()
        if dis == 1:
            res = self.draw(vis)
            bin = self.binary
        return res, bin
        