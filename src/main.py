import cv2
import time
import threading
from models.cam import Camera
from models.detector import Detector
from models.tracker import Tracker, Status
from models.stepper import SysParams, EmmMotor
from models.pid import PIDController
import Hobot.GPIO as GPIO
from models.status import GPIN
from models.dm_imu import imu


# ---------放在最前面：特别注意的接口和开关----------
camera_index = 0        # 摄像头索引，需根据实际情况调整
yaw_port = '/dev/ttyS1'      # yaw轴电机串口
pitch_port = '/dev/ttyS2'     # pitch轴电机串口

use_kf = True           # 是否启用卡尔曼滤波
show_windows = True     # 是否显示调试窗口
# ------------------------------------------------------------------------------

# -----------------模块初始化--------------------
camera = Camera(index = camera_index, width = 640, height = 480)
detector = Detector(min_area = 5000, max_area = 500000)
tracker = Tracker(f_pixel_h = 725.6, real_height = 17.5, use_kf = use_kf) 

stepper_yaw = EmmMotor(port = yaw_port, baudrate = 115200, timeout = 1, motor_id = 1)
stepper_pitch = EmmMotor(port = pitch_port, baudrate = 115200, timeout = 1, motor_id = 2)
pid_yaw = PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)
pid_pitch = PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)
lazer = GPIN(pin=16, mode=1) #激光笔控制
heart_beat = GPIN(pin=18, mode=1) #呼吸灯，用于表示主程序还在跑
# 世界坐标系下的目标位置，单位为度
system_running = True       # 开关
w_target_yaw = 0.0
w_target_pitch = 0.0
# 供控制线程读取的滑块参数
ctrl_vel_rpm = 3000
ctrl_acc = 100
# -------------------------------------------------------------------------------

def nothing(x):
    pass

def init_board():
    """初始化调试窗口和滑动条"""
    cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Controls', 300, 150)
    cv2.namedWindow('DETECTOR', cv2.WINDOW_FREERATIO)  
    cv2.namedWindow('BIN', cv2.WINDOW_FREERATIO)      

    cv2.createTrackbar('yaw_kp', 'Controls', 0, 100, nothing)   
    cv2.createTrackbar('yaw_ki', 'Controls', 0, 100, nothing)   
    cv2.createTrackbar('yaw_kd', 'Controls', 0, 100, nothing)

    cv2.createTrackbar('pitch_kp', 'Controls', 0, 100, nothing)   
    cv2.createTrackbar('pitch_ki', 'Controls', 0, 100, nothing)   
    cv2.createTrackbar('pitch_kd', 'Controls', 0, 100, nothing)  

    cv2.createTrackbar('onfire_tol', 'Controls', 5, 100, nothing) # 开火容忍度，单位为0.1度

    cv2.createTrackbar('vel_rpm', 'Controls', 3000, 5000, nothing)
    cv2.createTrackbar('acc', 'Controls', 100, 255, nothing)
    cv2.createTrackbar('show', 'Controls', 1, 1, nothing)

def update_params():
    """回调获取滑块参数"""
    yaw_kp = cv2.getTrackbarPos('yaw_kp', 'Controls')/1000
    yaw_ki = cv2.getTrackbarPos('yaw_ki', 'Controls')/10000
    yaw_kd = cv2.getTrackbarPos('yaw_kd', 'Controls')/10000

    pitch_kp = cv2.getTrackbarPos('pitch_kp', 'Controls')/1000
    pitch_ki = cv2.getTrackbarPos('pitch_ki', 'Controls')/10000
    pitch_kd = cv2.getTrackbarPos('pitch_kd', 'Controls')/10000

    onfire_tol = cv2.getTrackbarPos('onfire_tol', 'Controls')/10

    vel_rpm = cv2.getTrackbarPos('vel_rpm', 'Controls')
    acc = cv2.getTrackbarPos('acc', 'Controls')

    tracker.onfire_tol = onfire_tol
    
    #赋值给模块
    pid_yaw.set_Kp(yaw_kp)
    pid_yaw.set_Ki(yaw_ki)
    pid_yaw.set_Kd(yaw_kd)
    
    pid_pitch.set_Kp(pitch_kp)
    pid_pitch.set_Ki(pitch_ki)
    pid_pitch.set_Kd(pitch_kd)

    return vel_rpm, acc

def control_thread():
    """
    内环：高频姿态控制线程
    以 200Hz 运行，死咬靶纸的世界坐标，疯狂下发指令对抗底盘运动
    """
    global w_target_yaw, w_target_pitch, system_running, ctrl_vel_rpm

    while system_running:
        start_time = time.time()
        
        # 1. 极速读取 IMU
        pkt, _, _ = imu.dev.get_latest()
        curr_yaw, curr_pitch = 0.0, 0.0
        if pkt is not None:
            _, (_, _, curr_yaw) = pkt
            
        # 2. 计算空间角度误差
        error_yaw = w_target_yaw - curr_yaw
        error_pitch = w_target_pitch - curr_pitch

        # 3. PID 计算出目标速度
        # 此时的 correction 已经不再是角度增量，而是"RPM转速趋势"
        correction_yaw = pid_yaw.compute(error_yaw)
        correction_pitch = pid_pitch.compute(error_pitch)
        
        # 映射为电机的绝对转速 (RPM)
        # 提示: 如果你滑块里的 kp 依然是 /1000 的小数值，这里必须乘以 1000 放大，否则电机没劲
        vel_out_yaw = int(abs(correction_yaw) * 1000)
        vel_out_pitch = int(abs(correction_pitch) * 1000)

        # 速度封顶保护
        vel_out_yaw = min(vel_out_yaw, ctrl_vel_rpm)
        vel_out_pitch = min(vel_out_pitch, ctrl_vel_rpm)

        # 4. 判断转向逻辑 (核心陷阱)
        # 如果你发现云台不是去追目标，而是越跑越偏，直接把这里的 0 和 1 互换！
        dir_yaw = 1 if correction_yaw > 0 else 0
        dir_pitch = 1 if correction_pitch > 0 else 0
        
        # 5. 下发纯速度指令 (acc=0 彻底关闭内部加减速缓冲)
        try:
            # 加入 0.5 度的死区，防止到位后电机高频微调发出滋滋声
            if abs(error_yaw) > 0.5:
                stepper_yaw.emm_v5_vel_control(dir=dir_yaw, vel=vel_out_yaw, acc=0, snF=False)
            else:
                stepper_yaw.emm_v5_vel_control(dir=dir_yaw, vel=0, acc=0, snF=False) # 瞬间刹停
                
            if abs(error_pitch) > 0.5:
                stepper_pitch.emm_v5_vel_control(dir=dir_pitch, vel=vel_out_pitch, acc=0, snF=False)
            else:
                stepper_pitch.emm_v5_vel_control(dir=dir_pitch, vel=0, acc=0, snF=False)
        except:
            pass
            
        # 严格保持 200Hz 刷新率
        cost_time = time.time() - start_time
        sleep_time = 0.005 - cost_time
        if sleep_time > 0:
            time.sleep(sleep_time)
            
def main():
    # 拿到全局变量
    global w_target_yaw, w_target_pitch, system_running

    print("视觉跟踪系统启动... 按 'q' 键退出。")
    init_board()
    prev_time = time.time()

    # 启动高速控制线程
    ctrl_thread = threading.Thread(target=control_thread, daemon=True)
    ctrl_thread.start()
    print("高速 IMU 控制内环已启动")

    try:
        while True:

            # 呼吸灯，证明主程序在运行(单线程中闪烁频率完全受制于主循环的运行速度)
            heart_beat.flash()
            #更新参数
            vel_rpm, acc = update_params()

            # 读帧
            ret, frame = camera.read()
            if not ret: continue

            # 目标检测
            target = detector.detect(frame)
            
            # 滤波跟踪与解算
            vis_yaw, vis_pitch, dist, status, laser_pos = tracker.track(target)
            
            # 计算 FPS（终端打印）
            curr_time = time.time()
            fps = 1.0 / max(curr_time - prev_time, 1e-6)
            prev_time = curr_time

            # 坐标系变换与目标刷新
            if status in [Status.TRACK, Status.TMP_LOST]:
                w_target_yaw, _ = imu.get_abs(vis_yaw, vis_pitch)
                w_target_pitch = vis_pitch
            else:
                # 纯视觉环的保命机制：彻底丢失目标时，立刻将 Pitch 误差清零刹车！
                w_target_pitch = 0.0
                pid_pitch.reset()  # 清空 PID 积分，防止残留的“力气”让电机乱窜

            # 激光开火判断
            if not tracker.onfire and status in (Status.TRACK, Status.TMP_LOST):
                lazer.set_value(0)
            else:
                lazer.set_value(1)
            
            # 格式化状态文本
            if status == Status.TRACK:
                info = f"[TRACK] Yaw:{vis_yaw:.2f} Pitch:{vis_pitch:.2f} Dist:{dist:.1f}cm"
            elif status == Status.TMP_LOST:
                info = f"[PREDICT] Predicting... Dist:{dist:.1f}cm"
            else:
                info = "[LOST] Searching..."
            
            # 终端打印 FPS + 状态
            print(f"FPS: {fps:.1f} | {info}")
            
            # 同步 raw 帧
            detector.raw = frame
            tracker.raw = frame
            
            # 绘制与展示
            if show_windows:
                # 获取绘制结果
                vis_det, bin_img = detector.display(dis=1)
                vis_trk = tracker.display(dis=1, laser_pos=laser_pos)
                # 窗口显示
                if vis_det is not None:
                    cv2.imshow("DETECTOR", vis_det)
                if bin_img is not None:
                    cv2.imshow("BIN", bin_img)
                if vis_trk is not None:
                    cv2.imshow("Tracker", vis_trk)
            else:
                cv2.destroyAllWindows()
    
            # 退出控制
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            
    except Exception as e:
        print(f"\n主循环异常: {str(e)}")
    except KeyboardInterrupt:
        print("\n收到中断信号...")
    finally:
        system_running = False
        ctrl_thread.join() # 等待电机安全退出
        print("\n正在释放资源...")
        camera.release()
        try:
            stepper_yaw.close()
            stepper_pitch.close()
        except: pass
        cv2.destroyAllWindows()
        lazer.cleanup()
        heart_beat.cleanup()
        print("系统已安全关闭")

if __name__ == '__main__':
    main()