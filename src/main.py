import cv2
import time
from models.detector import Detector
from models.tracker import Tracker, Status

# 初始化模块
detector = Detector(min_area=5000, max_area=500000)
tracker = Tracker(f_pixel_h=725.6, real_height=17.5, use_kf=True) # 默认开启卡尔曼
camera_index = 4   # 相机索引

# 用于记录上一次的阈值，实现去重打印
last_thresh = -1

def nothing(x):
    pass

def init_board():
    """初始化调试窗口和滑动条"""
    cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Controls', 300, 150)
    cv2.namedWindow('Result', cv2.WINDOW_FREERATIO)
    cv2.namedWindow('Mask', cv2.WINDOW_FREERATIO)
    cv2.createTrackbar('Threshold', 'Controls', 127, 255, nothing)

def update_params():
    """回调获取滑块参数，若数值改变则在终端打印"""
    global last_thresh
    
    # 获取当前滑动条数值
    current_thresh = cv2.getTrackbarPos('Threshold', 'Controls')
    
    # 同步给检测器
    detector.threshold_value = current_thresh
    
    # 如果数值发生变化，打印到终端
    if current_thresh != last_thresh:
        print(f"[Update] New Threshold: {current_thresh}")
        last_thresh = current_thresh

def main():
    print("视觉跟踪系统启动... 按 'q' 键退出。")
    init_board()

    # 使用 V4L2 后端打开摄像头（Linux 下推荐）
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"错误: 无法打开摄像头 {camera_index}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    prev_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        update_params()
        
        # 目标检测：返回 Board 对象或 None
        target = detector.detect(frame)
        
        # 滤波跟踪与解算：统一入口，返回 5 元组 (yaw, pitch, dist, status, laser_pos)
        yaw, pitch, dist, status, laser_pos = tracker.track(target)
        
        # 计算整个主循环的 FPS
        curr_time = time.time()
        fps = 1.0 / max(curr_time - prev_time, 1e-6)  # 防除零保护
        prev_time = curr_time
        
        # 格式化显示文本（根据状态自动切换）
        if status == Status.TRACK:
            info = f"[TRACK] Yaw:{yaw:.2f}° Pitch:{pitch:.2f}° Dist:{dist:.1f}cm"
        elif status == Status.TMP_LOST:
            info = f"[PREDICT] 惯性预测中... Dist:{dist:.1f}cm"
        else:
            info = "[LOST] 搜索目标中..."
            
        # 统一渲染：在副本上绘制，绝不污染原始帧
        vis_frame, mask = detector.display(
            dis=1, fps=fps, info_text=info, laser_pos=laser_pos, show_binary=True
        )
        
        # 窗口显示
        if vis_frame is not None:
            cv2.imshow("Result", vis_frame)
        if mask is not None:
            cv2.imshow("Mask", mask)

        # 退出控制
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n" + "="*30)
            print(f"程序退出。最终确认阈值 Threshold: {detector.threshold_value}")
            print("="*30)
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()