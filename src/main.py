import cv2
import time
from models.detector import Detector

def main():
    print("初始化系统...")
    
    # 实例化我们的检测器，可以根据实际摄像头高度和靶纸大小微调面积阈值
    detector = Detector(min_area=5000, max_area=500000)
    
    # 打开摄像头
    # 在 Linux 下，通常 /dev/video0 对应索引 0。
    # 如果画面报错或打不开，请把 0 改成 1 (对应 /dev/video1)
    # cv2.CAP_V4L2 是 Linux 下 Video4Linux2 框架的后端，加上它能避免一些奇怪的报错和延迟
    camera_index = 0
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print(f"错误: 无法打开摄像头 /dev/video{camera_index}。请尝试更改 camera_index = 1")
        return

    # 设置摄像头分辨率 (建议设为 640x480 以保证处理帧率，电赛E题不需要太高的分辨率)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print(f"摄像头已启动 (按 'q' 键退出)。")

    # 用于计算帧率 (FPS)
    prev_time = time.time()

    while True:
        # 1. 读取当前帧
        ret, frame = cap.read()
        if not ret:
            print("错误: 无法获取画面帧。")
            break

        # 2. 将画面传入检测器进行处理
        annotated_frame, board = detector.process_image(frame)

        # 3. 计算并显示 FPS (可选，用于评估系统性能)
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 4. 终端打印调试信息（如果检测到了靶纸）
        if board.is_valid:
            print(f"发现靶纸! 中心坐标: {board.center}, 面积: {board.area:.1f}")
        else:
            print("寻找靶纸中...")

        # 5. 显示处理后的画面
        cv2.imshow("Target Board Detector (Press 'q' to exit)", annotated_frame)

        # 6. 监听键盘事件，按 'q' 键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("收到退出指令...")
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已安全退出。")

if __name__ == '__main__':
    main()