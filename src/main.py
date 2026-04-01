import cv2
import time
from models.detector import Detector
from models.tracker import Tracker
detector = Detector(min_area=5000, max_area=500000)
tracker = Tracker(f_pixel_h=725.6, real_height=17.5)
camera_index = 0    # 相机索引

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
    print("正在启动视觉系统... 按 'q' 键退出。")
    init_board()

    # 使用 V4L2 后端打开摄像头
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

        # 执行参数回调与打印逻辑
        update_params()
        # 目标检测，及绘制结果
        annotated_frame, board = detector.process_image(frame)
        # 角度解算
        if board.is_valid:
            res = tracker.solve(board)
            if res:
                yaw, pitch, dist = res
                info = f"Yaw:{yaw:.2f} Pitch:{pitch:.2f} Dis:{dist:.1f}cm"
                cv2.putText(annotated_frame, info, (10, 70), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # 计算 FPS
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 窗口显示
        cv2.imshow("Mask", detector.last_binary)
        cv2.imshow("Result", annotated_frame)

        # 退出按q
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n" + "="*30)
            print(f"程序退出。最终确认阈值 Threshold: {detector.threshold_value}")
            print("="*30)
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()