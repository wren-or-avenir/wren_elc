import cv2
import time
from models.detector import Detector

# 定义全局 detector 供回调同步使用
detector = Detector(min_area=25000, max_area=50000)
camera_index = 4

def nothing(x):
    pass

def init_board():
    """初始化窗口和滑动条"""
    # 创建控制窗口
    cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Controls', 300, 200)
    
    # 创建结果展示窗口
    cv2.namedWindow('Result', cv2.WINDOW_FREERATIO)
    cv2.namedWindow('Mask', cv2.WINDOW_FREERATIO)
    
    # 窗口布局排版 (可根据实际屏幕微调)
    cv2.moveWindow('Controls', 0, 0)
    cv2.moveWindow('Mask', 320, 0)
    cv2.moveWindow('Result', 700, 0)

    # 创建阈值滑动条
    cv2.createTrackbar('Threshold', 'Controls', 127, 255, nothing)
    # 模式开关：0为比赛模式（高性能），1为调试模式
    cv2.createTrackbar('Mode', 'Controls', 1, 1, nothing)

def update_params():
    """获取滑动条数值并更新到 detector"""
    # 获取当前阈值并同步给 detector
    thresh = cv2.getTrackbarPos('Threshold', 'Controls')
    detector.threshold_value = thresh
    
    # 获取当前模式
    mode = cv2.getTrackbarPos('Mode', 'Controls')
    return mode

def main():
    print("初始化系统...")
    
    # 初始化控制面板
    init_board()

    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print(f"错误: 无法打开摄像头 /dev/video{camera_index}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print(f"摄像头已启动 (按 'q' 键退出)。")

    prev_time = time.time()

    while True:
        # 1. 读取当前帧
        ret, frame = cap.read()
        if not ret:
            break

        # 2. 同步滑块参数并获取模式
        mode = update_params()

        # 3. 将画面传入检测器进行处理
        # 此时 detector 内部的 threshold_value 已经被 update_params 更新
        annotated_frame, board = detector.process_image(frame)

        # 4. 模式逻辑处理
        if mode == 1:
            # 调试模式：显示 FPS 和 Mask 窗口
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time)
            prev_time = curr_time
            cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # 显示二值化后的图像，方便调参
            if hasattr(detector, 'last_binary'):
                cv2.imshow("Mask", detector.last_binary)
            
            if board.is_valid:
                print(f"发现靶纸! 中心坐标: {board.center}, 面积: {board.area:.1f}")
        else:
            # 比赛模式：不打印日志，不显示 Mask 以节省算力
            pass

        # 5. 显示最终处理后的画面
        cv2.imshow("Result", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("程序已安全退出。")

if __name__ == '__main__':
    main()