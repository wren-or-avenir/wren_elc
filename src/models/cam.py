import cv2
import time
import queue
import threading

class Camera:
    def __init__(self, index = 0, width=640, height=480):
        # 尝试打开摄像头，直到成功为止
        index = self.find_index(index)
        if index is None:
            raise RuntimeError("不能打开任何摄像头")
        self.cam = cv2.VideoCapture(index)
        
        # ====== 极其严格的设置顺序 ======
        # 1. 必须先设置格式为 MJPG (用单引号拆开写，兼容性最好)
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        # 2. 再设置宽高
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)     
        
        # 3. 最后强求 60 帧
        self.cam.set(cv2.CAP_PROP_FPS, 60)
        # ==============================

        # 获取实际生效的画面宽度与高度
        self.width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # --- 暴力排雷：直接打印底层最终妥协的参数 ---
        actual_fps = self.cam.get(cv2.CAP_PROP_FPS)
        actual_fourcc = int(self.cam.get(cv2.CAP_PROP_FOURCC))
        # 解码 FOURCC 变成人类可读的字符串
        fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)]) if actual_fourcc != 0 else "未知"
        print(f"\n[摄像头底层核查] 格式: {fourcc_str} | 目标帧率: 60 | 实际生效帧率: {actual_fps}\n")
        # ----------------------------------------

        # 长度为1的线程队列
        self.q = queue.Queue(maxsize=1)
        self.running = True
        # 线程设置
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

        # 等待第一帧画面出来
        print("等待摄像头初始化...")
        while self.q.empty() and self.running:
            time.sleep(0.01)
        print(f"摄像头初始化完成")
    
    def find_index(self, index=0):
        max_index = index + 20
        for i in range(index, max_index):
            cam = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if cam.isOpened():
                cam.release()
                return i
            cam.release()
        return None
    
    def _update(self):
        """后台线程：疯狂读图，永远只在队列保留最新一帧"""
        while self.running:
            ret, frame = self.cam.read()
            if ret:
                # 如果队列有旧帧，丢掉
                if self.q.full():
                    try:
                        self.q.get_nowait()
                    except queue.Empty:
                        pass
                # 把最新帧放进队列
                self.q.put((ret, frame))
            else:
                # 如果硬件卡住没读到，稍微休眠防止死循环榨干CPU
                time.sleep(0.01)
        
    def read(self):
        """对外接口：获取最新的画面"""
        try:
            # 阻塞等待，最多等1秒。防止相机掉线导致主程序死锁卡死
            ret, frame = self.q.get(timeout=1.0)
            return ret, frame
        except queue.Empty:
            print("警告：读取摄像头超时 (1秒未收到新画面)")
            return False, None
    
    def release(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.cam.release()